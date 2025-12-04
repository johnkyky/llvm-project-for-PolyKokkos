//===- CodeGeneration.cpp - Code generate the Scops using ISL. ---------======//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
//
//===----------------------------------------------------------------------===//

#include "polly/Test/TriangularLoopFix.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/Analysis/AssumptionCache.h"
#include "llvm/Analysis/LoopInfo.h"
#include "llvm/Analysis/ScalarEvolution.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/Dominators.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/PassManager.h"
#include "llvm/IR/Verifier.h"
#include "llvm/Passes/PassBuilder.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Transforms/Utils/BasicBlockUtils.h"
#include "llvm/Transforms/Utils/LoopPeel.h"
#include "llvm/Transforms/Utils/LoopUtils.h"
#include <cstdint>
#include <utility>

#define DEBUG_TYPE "polly-triangular-loop-fix"

using namespace llvm;
using namespace polly;

namespace {

PHINode *getInductionVariable(Loop *L, ScalarEvolution &SE, DominatorTree &DT,
                              LoopInfo &LI) {
  if (PHINode *IndVar = L->getCanonicalInductionVariable()) {
    return IndVar;
  }

  for (PHINode &PN : L->getHeader()->phis()) {
    if (not L->getLoopPreheader()) {
      BasicBlock *Pred = L->getLoopPredecessor();
      BasicBlock *Succ = L->getHeader();
      SplitEdge(Pred, Succ, &DT, &LI);
      SE.forgetLoop(L);
    }

    InductionDescriptor D;
    if (InductionDescriptor::isInductionPHI(&PN, L, &SE, D)) {
      return &PN;
    }
  }
  return nullptr;
}

Loop *getOutermostLoop(Loop *L) {
  Loop *Outermost = L;
  while (Loop *Parent = Outermost->getParentLoop()) {
    Outermost = Parent;
  }
  return Outermost;
}

std::vector<Loop *> getTriangularLoops(LoopInfo &LI, ScalarEvolution &SE,
                                       DominatorTree &DT) {
  std::vector<Loop *> TriangularLoops;
  for (Loop *L : LI.getLoopsInPreorder()) {
    PHINode *IndVar = getInductionVariable(L, SE, DT, LI);
    if (!IndVar) {
      continue;
    }

    Loop *OuterLoop = getOutermostLoop(L);
    const SCEV *BackedgeCount = SE.getBackedgeTakenCount(L);

    if (isa<SCEVCouldNotCompute>(BackedgeCount))
      continue;

    if (not SE.isLoopInvariant(BackedgeCount, OuterLoop))
      TriangularLoops.push_back(L);
  }
  return TriangularLoops;
}

int64_t getEmptyInnerLoopIterations(const SCEV *Bound, ScalarEvolution &SE) {
  // 1. On vérifie que c'est bien une AddRecurrence (Triangulaire)
  auto *AddRec = dyn_cast<SCEVAddRecExpr>(Bound);
  if (!AddRec)
    return 0;

  // 2. On suppose que c'est une expression affine simple (constantes)
  auto *StartC = dyn_cast<SCEVConstant>(AddRec->getStart());
  auto *StepC = dyn_cast<SCEVConstant>(AddRec->getStepRecurrence(SE));

  if (!StartC || !StepC)
    return 0; // Trop complexe si ce n'est pas constant

  int64_t Start = StartC->getValue()->getSExtValue();
  int64_t Step = StepC->getValue()->getSExtValue();

  // Si le démarrage est déjà positif, la boucle existe dès le début.
  if (Start >= 0)
    return 0;

  // Si le pas est négatif ou nul, la boucle ne deviendra jamais positive (cas
  // dégénéré)
  if (Step <= 0)
    return -1; // Ou gérer comme erreur

  // 3. Résolution de l'équation : Start + i * Step < 0
  // On cherche combien d'itérations 'i' donnent un résultat négatif.
  // Formule : ceil(abs(Start) / Step)

  // Utilisation de l'arithmétique entière pour faire le ceil : (Num + Denom -
  // 1) / Denom
  int64_t AbsStart = std::abs(Start);
  int64_t SkippedIterations = (AbsStart + Step - 1) / Step;

  return SkippedIterations;
}

DenseMap<Loop *, int64_t>
mapTriangularLoopToOuterIV(std::vector<Loop *> TriangularLoops, LoopInfo &LI,
                           ScalarEvolution &SE, DominatorTree &DT) {
  DenseMap<Loop *, int64_t> Map;

  for (auto *L : TriangularLoops) {
    const SCEV *Bound = SE.getBackedgeTakenCount(L);
    errs() << "Bound " << *Bound << "\n";

    Loop *OuterLoop = getOutermostLoop(L);

    if (const SCEVAddRecExpr *AR = dyn_cast<SCEVAddRecExpr>(Bound)) {
      if (AR->getLoop() == OuterLoop) {
        int64_t SkipedIters = getEmptyInnerLoopIterations(Bound, SE);
        Map.insert({L, SkipedIters});
      } else
        llvm_unreachable("AddRecExpr loop is not the outer loop ?");
    } else
      llvm_unreachable("Bound is not an AddRecExpr ?");
  }

  return Map;
}

std::pair<Loop *, size_t> findTriangularLoops(LoopInfo &LI, ScalarEvolution &SE,
                                              DominatorTree &DT) {
  SmallVector<std::pair<Loop *, size_t>, 2> TriangularLoops;
  for (Loop *L : LI) {
    if (L->getSubLoops().empty()) {
      errs() << "\n\tLoop sans sous-boucle, on passe.\n";
      continue;
    }

    PHINode *IndVar = getInductionVariable(L, SE, DT, LI);
    if (!IndVar) {
      errs() << "\tPas de variable d'induction, on passe.\n";
      continue;
    }

    errs() << "getBackedgeTakenCount pour la boucle "
           << *SE.getBackedgeTakenCount(L) << "\n";

    for (Loop *SubL : L->getSubLoops()) {
      size_t EmptyIterations = 0;
      errs() << "\tAnalyse de la boucle interne " << *SubL << "\n";
      const SCEV *BackedgeCount = SE.getBackedgeTakenCount(SubL);
      errs() << "\tBackedge count de la boucle " << *SubL << " : "
             << *BackedgeCount << "\n";

      if (isa<SCEVCouldNotCompute>(BackedgeCount)) {
        errs() << "\tBackedge count non calculable, on "
                  "passe.\n";
        continue;
      }

      if (SE.isLoopInvariant(BackedgeCount, L)) {
        errs() << "\tBackedge count invariant on passe.\n";
        continue;
      }

      if (auto *AddRect = dyn_cast<SCEVAddRecExpr>(BackedgeCount)) {
        if (AddRect->getLoop() == L and AddRect->isAffine()) {
          errs() << "\tBackedge count affine detecte: " << *AddRect << "\n";

          const SCEV *StartSCEV = AddRect->getStart();
          const SCEV *StepSCEV = AddRect->getStepRecurrence(SE);
          if (auto *StartConst = dyn_cast<SCEVConstant>(StartSCEV)) {
            if (auto *StepConst = dyn_cast<SCEVConstant>(StepSCEV)) {
              APInt StartVal = StartConst->getAPInt();
              APInt StepVal = StepConst->getAPInt();

              if (StepVal.isStrictlyPositive()) {
                // Si Start == 0 (Cas j < i) -> 1 itération vide (i=0)
                if (StartVal.isZero()) {
                  errs() << "Iteration 0 est vide (1 itération a skipper).\n";
                  EmptyIterations = 1;
                  // return 1;
                }
                // Si Start > 0 (Cas j < i+1) -> Aucune itération vide
                else if (StartVal.isStrictlyPositive()) {
                  errs() << "Aucune itération vide au début.\n";
                  // return 0;
                }
                // Si Start < 0 (Cas j < i - K) -> Plusieurs itérations vides
                else {
                  // Formule : Ceil(abs(Start) / Step)
                  // En arithmétique entière : (abs(Start) + Step - 1) / Step
                  APInt AbsStart = StartVal.abs();
                  APInt EmptyCount = (AbsStart + StepVal - 1).udiv(StepVal);

                  // Attention, on ajoute +1 car l'itération "0" compte aussi si
                  // on traverse l'axe Mais généralement si Start = -2 et Step =
                  // 1 : i=0 -> -2 (Vide) i=1 -> -1 (Vide) i=2 -> 0 (Vide ou 1er
                  // passage selon sémantique LT/LE) Souvent BackedgeCount est
                  // le nombre de tours, donc si SCEV <= 0, tours = 0.

                  errs() << "Iterations vides: " << EmptyCount << "\n";
                  EmptyIterations = EmptyCount.getZExtValue();
                }
              }
            }
          }
        }
      }
      if (not EmptyIterations) {
        errs() << "\tBackedge count non triangulaire, on "
                  "passe.\n";
        continue;
      }

      TriangularLoops.push_back({L, EmptyIterations});
      errs() << "\tBoucle triangulaire detectee ! " << *L << "\n";
    }
  }

  if (TriangularLoops.empty()) {
    return {nullptr, 0};
  }
  return TriangularLoops[0];
}

void transformTriangularLoops(Loop *L, int64_t EmptyIterations, LoopInfo &LI,
                              ScalarEvolution &SE, DominatorTree &DT,
                              AssumptionCache &AC) {
  LLVM_DEBUG(errs() << "\tModification de la boucle "
                    << L->getHeader()->getName() << " pour skippper "
                    << EmptyIterations << " itérations vides.\n";);

  if (not L->isLoopSimplifyForm()) {
    errs() << "on force la simplification de la boucle " << L->getName()
           << "\n";

    bool Simplified = simplifyLoop(L, &DT, &LI, &SE, &AC, nullptr, false);

    if (!Simplified) {
      errs() << "Impossible de simplifier la boucle " << L->getName() << "\n";
      return;
    }
  }

  if (canPeel(L) == false) {
    LLVM_DEBUG(
        errs() << "\tPeeling non possible sur cette boucle, on passe.\n";);
    return;
  }
  errs() << "Peeling possible sur la boucle " << L->getName() << "\n";

  ValueToValueMapTy VMap;
  bool IsDone = peelLoop(L, EmptyIterations, &LI, &SE, DT, &AC, false, VMap);

  if (not IsDone) {
    errs() << "Peeling échoué sur la boucle " << L->getName() << "\n";
  } else {
    errs() << "Peeling réussi sur la boucle " << L->getName() << "\n";
  }
}

} // namespace

PreservedAnalyses TriangularLoopFixPass::run(Function &F,
                                             FunctionAnalysisManager &AM) {
  if (not F.hasFnAttribute("polly.findSCoP"))
    return PreservedAnalyses::all();

  LLVM_DEBUG(errs() << "TriangularLoopFixPass pass run on " << F.getName()
                    << "\n";);

  LoopInfo &LI = AM.getResult<LoopAnalysis>(F);
  auto &DT = AM.getResult<DominatorTreeAnalysis>(F);
  for (auto *L : LI.getLoopsInPreorder())
    errs() << "LCSSA de " << L->getHeader()->getName() << "   "
           << L->isRecursivelyLCSSAForm(DT, LI) << "\n";

  if (verifyFunction(F, &errs())) {
    report_fatal_error("IR verification failed.");
  }

  ScalarEvolution &SE = AM.getResult<ScalarEvolutionAnalysis>(F);

  auto TriangularLoops = getTriangularLoops(LI, SE, DT);
  for (auto *L : TriangularLoops) {
    errs() << "Boucle triangulaire detectee: \n"
           << L->getHeader()->getName() << "\n";
  }

  auto Map = mapTriangularLoopToOuterIV(TriangularLoops, LI, SE, DT);

  auto &AC = AM.getResult<AssumptionAnalysis>(F);
  for (auto &Entry : Map) {
    errs() << "Boucle: " << *Entry.first
           << " -> Iterations vides: " << Entry.second << "\n";
    Loop *OuterLoop = getOutermostLoop(Entry.first);
    transformTriangularLoops(OuterLoop, Entry.second, LI, SE, DT, AC);
  }

  //
  //

  // auto LoopToModified = findTriangularLoops(LI, SE, DT);

  // if (not LoopToModified.first) {
  //   LLVM_DEBUG(errs() << "\tAucune boucle triangulaire detectee.\n";);
  //   return PreservedAnalyses::all();
  // }
  // transformTriangularLoops(LoopToModified, LI, SE, DT, AC);

  if (verifyFunction(F, &errs())) {
    report_fatal_error("IR verification failed.");
  }

  LLVM_DEBUG(errs() << "TriangularLoopFixPass pass done\n";);

  return PreservedAnalyses::none();
}
