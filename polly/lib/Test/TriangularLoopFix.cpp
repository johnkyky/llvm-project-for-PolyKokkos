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
#include "llvm/IR/IRBuilder.h"
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
#include <map>
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
std::string toString(PeelDirection D) {
  switch (D) {
  case PeelDirection::NONE:
    return "NONE";
  case PeelDirection::FRONT:
    return "FRONT";
  case PeelDirection::BACK:
    return "BACK";
  }
  return "UNKNOWN";
}

struct PeelResult {
  const SCEV *Count;
  PeelDirection Direction;
  const Loop *LoopToPeel;

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
    return {SE.getCouldNotCompute(), PeelDirection::NONE, nullptr};
  }

  const SCEV *Start = AddRec->getStart();
  const SCEV *Step = AddRec->getStepRecurrence(SE);

  const SCEVConstant *StepConst = dyn_cast<SCEVConstant>(Step);
  if (not StepConst)
    return {SE.getCouldNotCompute(), PeelDirection::NONE, nullptr};

  APInt StepVal = StepConst->getAPInt();

  const SCEV *Zero = SE.getZero(Start->getType());
  const SCEV *One = SE.getOne(Start->getType());

  if (StepVal.isStrictlyPositive()) {
    if (SE.isKnownNonNegative(Start))
      return {Zero, PeelDirection::NONE, nullptr};

    // Formule : ceil( (-Start) / Step )
    // En arithmétique entière : ( (-Start) + Step - 1 ) / Step

    const SCEV *NegStart = SE.getNegativeSCEV(Start);
    const SCEV *StepMinusOne = SE.getMinusSCEV(Step, One);
    const SCEV *Numerator = SE.getAddExpr(NegStart, StepMinusOne);

    return {SE.getUDivExpr(Numerator, Step), PeelDirection::FRONT,
            AddRec->getLoop()};
  }

  if (StepVal.isNegative()) {
    if (SE.isKnownNonPositive(Start))
      return {Zero, PeelDirection::BACK, AddRec->getLoop()};

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
    return {Res, PeelDirection::BACK, AddRec->getLoop()};
  }

  return {SE.getCouldNotCompute(), PeelDirection::NONE, nullptr};
}

DenseMap<Loop *, PeelResult>
mapTriangularLoopToOuterIV(std::vector<Loop *> TriangularLoops, LoopInfo &LI,
                           ScalarEvolution &SE, DominatorTree &DT) {
  DenseMap<Loop *, PeelResult> Map;

  for (auto *L : TriangularLoops) {
    const SCEV *Bound = SE.getBackedgeTakenCount(L);
    errs() << "Bound " << *Bound << "\n";

    // Loop *OuterLoop = getOutermostLoop(L);

    if (const SCEVAddRecExpr *AR = dyn_cast<SCEVAddRecExpr>(Bound)) {
      auto PeelRes = computeDuration(SE, AR);
      Map.insert({L, PeelRes});
    } else
      llvm_unreachable("Bound is not an AddRecExpr ?");
  }

  return Map;
}

Value *getLoopUpperBound(Loop *L, ScalarEvolution &SE) {
  std::optional<Loop::LoopBounds> Bounds = L->getBounds(SE);

  if (Bounds) {
    // Récupère la borne finale
    return &Bounds->getFinalIVValue();

    // Récupère la valeur de sortie exacte (Trip Count)
    // const SCEV *ExitCount = Bounds->getStepInst(); // etc.
  }
  return nullptr;
}

using PeelItem = std::pair<Loop *, PeelResult>;

SmallVector<PeelItem, 1>
getOrderedPeelingList(DenseMap<Loop *, PeelResult> &Map) {

  SmallVector<PeelItem, 1> Order;
  Order.reserve(Map.size());

  for (auto &KV : Map) {
    Order.push_back(KV);
  }

  std::sort(Order.begin(), Order.end(),
            [](const PeelItem &A, const PeelItem &B) {
              unsigned DepthA = A.first->getLoopDepth();
              unsigned DepthB = B.first->getLoopDepth();

              if (DepthA != DepthB)
                return DepthA < DepthB;

              return A.first->getHeader()->getName() <
                     B.first->getHeader()->getName();
            });

  return Order;
}

struct PeelingAction {
  Loop *TargetLoop;
  PeelDirection Direction;
  int64_t MaxPeelCount;

  // Pour retrouver le SCEV original si besoin,
  // ou on garde juste l'entier pour l'appel à peelLoop
};

std::vector<PeelingAction>
getOptimizedPeelingPlan(DenseMap<Loop *, PeelResult> &TriggerMap) {
  std::map<std::pair<const Loop *, PeelDirection>, int64_t> MergedRequests;

  for (auto &[KV, PR] : TriggerMap) {
    const Loop *Target = PR.LoopToPeel;

    int64_t Count = PR.getIntValue();
    if (Count <= 0)
      continue;

    auto Key = std::make_pair(Target, PR.Direction);

    if (MergedRequests.count(Key)) {
      if (Count > MergedRequests[Key]) {
        MergedRequests[Key] = Count;
      }
    } else {
      MergedRequests[Key] = Count;
    }
  }

  std::vector<PeelingAction> ActionList;
  ActionList.reserve(MergedRequests.size());

  for (auto &KV : MergedRequests) {
    ActionList.push_back(
        {const_cast<Loop *>(KV.first.first), KV.first.second, KV.second});
  }

  std::sort(ActionList.begin(), ActionList.end(),
            [](const PeelingAction &A, const PeelingAction &B) {
              unsigned DepthA = A.TargetLoop->getLoopDepth();
              unsigned DepthB = B.TargetLoop->getLoopDepth();

              if (DepthA != DepthB)
                return DepthA < DepthB;

              if (A.Direction != B.Direction)
                return A.Direction < B.Direction;
              return false;
            });

  size_t Accumulator = 0;
  for (auto It = ActionList.rbegin(); It != ActionList.rend(); ++It) {
    Accumulator += It->MaxPeelCount;
    It->MaxPeelCount = Accumulator;
  }

  return ActionList;
}

void transformTriangularLoops(PeelingAction &PA, LoopInfo &LI,
                              ScalarEvolution &SE, DominatorTree &DT,
                              AssumptionCache &AC) {
  LLVM_DEBUG(errs() << "\tModification de la boucle "
                    << PA.TargetLoop->getHeader()->getName()
                    << " pour skippper " << PA.MaxPeelCount
                    << " itérations vides.\n";);

  if (not PA.TargetLoop->isLoopSimplifyForm()) {
    errs() << "on force la simplification de la boucle "
           << PA.TargetLoop->getName() << "\n";

    bool Simplified =
        simplifyLoop(PA.TargetLoop, &DT, &LI, &SE, &AC, nullptr, false);

    if (!Simplified) {
      errs() << "Impossible de simplifier la boucle "
             << PA.TargetLoop->getName() << "\n";
      return;
    }
  }

  if (canPeel(PA.TargetLoop) == false) {
    LLVM_DEBUG(
        errs() << "\tPeeling non possible sur cette boucle, on passe.\n";);
    return;
  }
  errs() << "Peeling possible sur la boucle " << PA.TargetLoop->getName()
         << "\n";

  ValueToValueMapTy VMap;
  bool IsDone = false;

  if (PA.Direction == PeelDirection::FRONT) {
    IsDone = peelLoop(PA.TargetLoop, PA.MaxPeelCount, false, &LI, &SE, DT, &AC,
                      false, VMap);
    if (not IsDone) {
      errs() << "Peeling échoué sur la boucle " << PA.TargetLoop->getName()
             << "\n";
    } else {
      errs() << "Peeling réussi sur la boucle " << PA.TargetLoop->getName()
             << "\n";
    }
  } else if (PA.Direction == PeelDirection::BACK) {
    if (PA.MaxPeelCount > 1) {
      LLVM_DEBUG(errs() << "Peeling back with count > 1 not supported yet.\n";);
      return;
    }

    PHINode *OriginalIV = PA.TargetLoop->getInductionVariable(SE);
    if (!OriginalIV) {
      llvm_unreachable("Expected induction variable");
    }
    auto *UpperBound = getLoopUpperBound(PA.TargetLoop, SE);
    errs() << "Original IV: " << *OriginalIV << "\n";

    IsDone = peelLoop(PA.TargetLoop, PA.MaxPeelCount, true, &LI, &SE, DT, &AC,
                      false, VMap);

    if (not IsDone) {
      errs() << "Peeling échoué sur la boucle " << PA.TargetLoop->getName()
             << "\n";
    } else {
      errs() << "Peeling réussi sur la boucle " << PA.TargetLoop->getName()
             << "\n";
    }
    if (not IsDone) {
      return;
    }

    { // Replace uses of the IV in the peeled loop with the hardcoded value (N -
      // k)
      Value *MappedVal = VMap[OriginalIV];
      Instruction *PeeledLoopIV = dyn_cast_or_null<Instruction>(MappedVal);
      if (!PeeledLoopIV) {
        llvm_unreachable("Expected instruction for peeled loop IV");
      }

      IRBuilder<> Builder(PeeledLoopIV->getParent()->getFirstNonPHI());

      Value *One = ConstantInt::get(UpperBound->getType(), PA.MaxPeelCount);
      Value *HardcodedIV =
          Builder.CreateSub(UpperBound, One, "fixed_iv_n_minus_k");

      PeeledLoopIV->replaceAllUsesWith(HardcodedIV);
    }

    SE.forgetAllLoops();

    const SCEV *BTC = SE.getBackedgeTakenCount(PA.TargetLoop);
    if (!isa<SCEVCouldNotCompute>(BTC)) {
      for (BasicBlock *BB : PA.TargetLoop->blocks()) {
        for (Instruction &I : llvm::make_early_inc_range(*BB)) {
          if (auto *Cmp = dyn_cast<ICmpInst>(&I)) {
            if (Cmp->getPredicate() == CmpInst::ICMP_ULT ||
                Cmp->getPredicate() == CmpInst::ICMP_SLT) {

              const SCEV *LHS = SE.getSCEV(Cmp->getOperand(0));
              const SCEV *RHS = SE.getSCEV(Cmp->getOperand(1));
              const SCEV *Delta = SE.getMinusSCEV(RHS, LHS);

              if (auto *AR = dyn_cast<SCEVAddRecExpr>(Delta)) {
                const SCEV *BTC = SE.getBackedgeTakenCount(PA.TargetLoop);

                if (!isa<SCEVCouldNotCompute>(BTC)) {
                  const SCEV *LastValue = AR->evaluateAtIteration(BTC, SE);
                  if (SE.isKnownPositive(LastValue)) {
                    Cmp->replaceAllUsesWith(
                        ConstantInt::getTrue(Cmp->getContext()));
                  }
                }
              }
            }
          }
        }
      }
    }
  }
  for (auto [K, V] : VMap) {
    errs() << "VMap: " << *K << " -> " << *V << "\n";
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

  errs() << "MAP -> " << Map.size() << " entrées\n";
  for (auto &[L, PeelRes] : Map) {
    errs() << "Boucle: " << *L
           << " -> Iterations vides: " << PeelRes.getIntValue()
           << "\n    on doit peel " << toString(PeelRes.Direction)
           << " de la boucle "
           << (PeelRes.LoopToPeel ? PeelRes.LoopToPeel->getName()
                                  : "nullptr loop")
           << "\n\n";
  }
  auto Order = getOptimizedPeelingPlan(Map);

  for (auto &L : Order) {
    errs() << " -> Iterations vides: " << L.MaxPeelCount
           << "\n    on doit peel " << toString(L.Direction) << " de la boucle "
           << L.TargetLoop->getName() << "\n\n";
  }

  auto &AC = AM.getResult<AssumptionAnalysis>(F);
  for (auto &LoopItem : Order) {
    transformTriangularLoops(LoopItem, LI, SE, DT, AC);
  }

  if (verifyFunction(F, &errs())) {
    report_fatal_error("IR verification failed.");
  }

  LLVM_DEBUG(errs() << "TriangularLoopFixPass pass done\n";);

  return PreservedAnalyses::none();
}
