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
#include "llvm/Analysis/AssumptionCache.h"
#include "llvm/Analysis/LoopInfo.h"
#include "llvm/Analysis/ScalarEvolution.h"
#include "llvm/Analysis/ScalarEvolutionExpressions.h"
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

enum class PeelDirection { NONE, FRONT, BACK };

struct PeelResult {
  const SCEV *Count;
  PeelDirection Direction;
  int64_t getIntValue() const {
    if (auto *ConstCount = dyn_cast<SCEVConstant>(Count)) {
      return ConstCount->getValue()->getSExtValue();
    }
    return 0;
  }
};

const PeelResult computeDuration(ScalarEvolution &SE,
                                 const SCEVAddRecExpr *AddRec) {
  if (not AddRec->isAffine()) {
    return {SE.getCouldNotCompute(), PeelDirection::NONE};
  }

  const SCEV *Start = AddRec->getStart();
  const SCEV *Step = AddRec->getStepRecurrence(SE);

  const SCEVConstant *StepConst = dyn_cast<SCEVConstant>(Step);
  if (not StepConst)
    return {SE.getCouldNotCompute(), PeelDirection::NONE};

  APInt StepVal = StepConst->getAPInt();

  const SCEV *Zero = SE.getZero(Start->getType());
  const SCEV *One = SE.getOne(Start->getType());

  if (StepVal.isStrictlyPositive()) {
    if (SE.isKnownNonNegative(Start))
      return {Zero, PeelDirection::NONE};

    // Formule : ceil( (-Start) / Step )
    // En arithmétique entière : ( (-Start) + Step - 1 ) / Step

    const SCEV *NegStart = SE.getNegativeSCEV(Start);
    const SCEV *StepMinusOne = SE.getMinusSCEV(Step, One);
    const SCEV *Numerator = SE.getAddExpr(NegStart, StepMinusOne);

    return {SE.getUDivExpr(Numerator, Step), PeelDirection::FRONT};
  }

  if (StepVal.isNegative()) {
    if (SE.isKnownNonPositive(Start))
      return {Zero, PeelDirection::BACK};

    // Formule : ceil( Start / (-Step) )
    // Soit AbsStep = -Step
    // En arithmétique entière : (Start + AbsStep - 1) / AbsStep

    const SCEV *AbsStep = SE.getNegativeSCEV(Step); // Car Step est négatif
    const SCEV *AbsStepMinusOne = SE.getMinusSCEV(AbsStep, One);
    const SCEV *Numerator = SE.getAddExpr(Start, AbsStepMinusOne);

    const auto *Div = SE.getUDivExpr(Numerator, AbsStep);

    const auto *L = AddRec->getLoop();
    const auto *BTC = SE.getBackedgeTakenCount(L);

    auto *Res = SE.getAbsExpr(SE.getMinusSCEV(BTC, Div), false);
    return {Res, PeelDirection::BACK};
  }

  return {SE.getCouldNotCompute(), PeelDirection::NONE};
}

DenseMap<Loop *, PeelResult>
mapTriangularLoopToOuterIV(std::vector<Loop *> TriangularLoops, LoopInfo &LI,
                           ScalarEvolution &SE, DominatorTree &DT) {
  DenseMap<Loop *, PeelResult> Map;

  for (auto *L : TriangularLoops) {
    const SCEV *Bound = SE.getBackedgeTakenCount(L);
    errs() << "Bound " << *Bound << "\n";

    Loop *OuterLoop = getOutermostLoop(L);

    if (const SCEVAddRecExpr *AR = dyn_cast<SCEVAddRecExpr>(Bound)) {
      if (AR->getLoop() == OuterLoop) {
        auto PeelRes = computeDuration(SE, AR);
        Map.insert({L, PeelRes});
      } else
        llvm_unreachable("AddRecExpr loop is not the outer loop ?");
    } else
      llvm_unreachable("Bound is not an AddRecExpr ?");
  }

  return Map;
}

void transformTriangularLoops(Loop *L, PeelResult PeelRes, LoopInfo &LI,
                              ScalarEvolution &SE, DominatorTree &DT,
                              AssumptionCache &AC) {
  LLVM_DEBUG(errs() << "\tModification de la boucle "
                    << L->getHeader()->getName() << " pour skippper "
                    << PeelRes.getIntValue() << " itérations vides.\n";);

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
  bool IsDone = false;

  if (PeelRes.Direction == PeelDirection::FRONT)
    IsDone = peelLoop(L, PeelRes.getIntValue(), false, &LI, &SE, DT, &AC, false,
                      VMap);
  else if (PeelRes.Direction == PeelDirection::BACK) {
    llvm_unreachable("Not implemented yet");
    // IsDone = peelLoop(L, PeelRes.getIntValue(), &LI, &SE, DT, &AC, VMap);
  }

  if (not IsDone) {
    errs() << "Peeling échoué sur la boucle " << L->getName() << "\n";
  } else {
    errs() << "Peeling réussi sur la boucle " << L->getName() << "\n";
  }
}

} // namespace

std::vector<Loop *> polly::getTriangularLoops(LoopInfo &LI, ScalarEvolution &SE,
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

PreservedAnalyses TriangularLoopFixPass::run(Function &F,
                                             FunctionAnalysisManager &AM) {
  if (not F.hasFnAttribute("polly.findSCoP"))
    return PreservedAnalyses::all();

  LLVM_DEBUG(errs() << "TriangularLoopFixPass pass run on " << F.getName()
                    << "\n";);

  LoopInfo &LI = AM.getResult<LoopAnalysis>(F);
  auto &DT = AM.getResult<DominatorTreeAnalysis>(F);

  ScalarEvolution &SE = AM.getResult<ScalarEvolutionAnalysis>(F);

  auto TriangularLoops = getTriangularLoops(LI, SE, DT);
  for (auto *L : TriangularLoops) {
    errs() << "Boucle triangulaire detectee: \n"
           << L->getHeader()->getName() << "\n";
  }

  if (TriangularLoops.empty()) {
    LLVM_DEBUG(errs() << "TriangularLoopFixPass pass done\n";);
    return PreservedAnalyses::none();
  }

  auto Map = mapTriangularLoopToOuterIV(TriangularLoops, LI, SE, DT);

  auto &AC = AM.getResult<AssumptionAnalysis>(F);
  for (auto &[L, PeelRes] : Map) {
    errs() << "Boucle: " << *L
           << " -> Iterations vides: " << PeelRes.getIntValue() << "\n";
    if (PeelRes.getIntValue() == 0)
      continue;

    Loop *OuterLoop = getOutermostLoop(L);
    transformTriangularLoops(OuterLoop, PeelRes, LI, SE, DT, AC);
  }

  if (verifyFunction(F, &errs())) {
    report_fatal_error("IR verification failed.");
  }

  LLVM_DEBUG(errs() << "TriangularLoopFixPass pass done\n";);

  return PreservedAnalyses::none();
}
