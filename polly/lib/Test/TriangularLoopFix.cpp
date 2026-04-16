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
    errs().indent(6) << "AddRec is not affine: " << *AddRec << "\n";
    return {SE.getCouldNotCompute(), PeelDirection::NONE, nullptr};
  }

  const SCEV *Start = AddRec->getStart();
  const SCEV *Step = AddRec->getStepRecurrence(SE);

  const SCEVConstant *StepConst = dyn_cast<SCEVConstant>(Step);
  if (not StepConst) {
    errs().indent(6) << "Step is not constant: " << *Step << "\n";
    return {SE.getCouldNotCompute(), PeelDirection::NONE, nullptr};
  }

  APInt StepVal = StepConst->getAPInt();

  errs().indent(6) << "AddRec Start: " << *Start << ", Step: " << StepVal
                   << "\n";

  llvm::Type *BaseType = SE.getWiderType(Start->getType(), Step->getType());
  Start = SE.getNoopOrSignExtend(Start, BaseType);
  Step = SE.getNoopOrSignExtend(Step, BaseType);

  const SCEV *Zero = SE.getZero(BaseType);
  const SCEV *One = SE.getOne(BaseType);

  if (StepVal.isStrictlyPositive()) {
    errs().indent(6) << "Step is positive\n";
    if (SE.isKnownNonNegative(Start)) {
      errs().indent(6) << "No peeling needed (Start >= 0)\n";
      return {Zero, PeelDirection::NONE, nullptr};
    }

    // Formule : ceil( (-Start) / Step )
    // En arithmétique entière : ( (-Start) + Step - 1 ) / Step

    const SCEV *NegStart = SE.getNegativeSCEV(Start);
    const SCEV *StepMinusOne = SE.getMinusSCEV(Step, One);
    const SCEV *Numerator = SE.getAddExpr(NegStart, StepMinusOne);

    return {SE.getUDivExpr(Numerator, Step), PeelDirection::FRONT,
            AddRec->getLoop()};
  }

  if (StepVal.isNegative()) {
    errs().indent(6) << "Step is negative\n";
    if (SE.isKnownNonPositive(Start)) {
      errs().indent(6) << "No peeling needed (Start <= 0)\n";
      return {Zero, PeelDirection::BACK, AddRec->getLoop()};
    }

    // Formule : ceil( Start / (-Step) )
    // Soit AbsStep = -Step
    // En arithmétique entière : (Start + AbsStep - 1) / AbsStep

    const SCEV *AbsStep = SE.getNegativeSCEV(Step);
    const SCEV *AbsStepMinusOne = SE.getMinusSCEV(AbsStep, One);
    const SCEV *Numerator = SE.getAddExpr(Start, AbsStepMinusOne);

    const auto *Div = SE.getUDivExpr(Numerator, AbsStep);

    const auto *L = AddRec->getLoop();
    const auto *BTC = SE.getBackedgeTakenCount(L);

    llvm::Type *WiderType = SE.getWiderType(BTC->getType(), Div->getType());
    const SCEV *ExtendedBTC = SE.getNoopOrSignExtend(BTC, WiderType);
    const SCEV *ExtendedDiv = SE.getNoopOrSignExtend(Div, WiderType);

    auto *Res = SE.getAbsExpr(SE.getMinusSCEV(ExtendedBTC, ExtendedDiv), false);
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
    errs() << "Bound " << *Bound << " pour " << L->getName() << "\n";

    // Loop *OuterLoop = getOutermostLoop(L);

    if (const SCEVAddRecExpr *AR = dyn_cast<SCEVAddRecExpr>(Bound)) {
      auto PeelRes = computeDuration(SE, AR);
      errs().indent(2) << "Peel Result for loop " << L->getName() << ": "
                       << "Count = " << *(PeelRes.Count)
                       << ", Direction = " << toString(PeelRes.Direction)
                       << "\n";
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

struct PeelingAction {
  Loop *TargetLoop;
  PeelDirection Direction;
  int64_t MaxPeelCount;
};

using LoopPeelPlanMap =
    std::map<std::pair<const Loop *, PeelDirection>, int64_t>;

DenseMap<Loop *, SmallVector<PeelingAction, 2>>
getOptimizedPeelingPlan(DenseMap<Loop *, PeelResult> &TriggerMap) {
  LoopPeelPlanMap MergedPlan;

  for (auto &[KV, PR] : TriggerMap) {
    const Loop *Target = PR.LoopToPeel;

    int64_t Count = PR.getIntValue();
    if (Count <= 0)
      continue;

    auto Key = std::make_pair(Target, PR.Direction);

    if (MergedPlan.count(Key)) {
      if (Count > MergedPlan[Key]) {
        MergedPlan[Key] = Count;
      }
    } else {
      MergedPlan[Key] = Count;
    }
  }

  DenseMap<Loop *, SmallVector<PeelingAction, 2>> ActionMap;
  for (auto &[Key, Value] : MergedPlan) {
    Loop *L = const_cast<Loop *>(Key.first);
    Loop *Outermost = getOutermostLoop(L);
    ActionMap[Outermost].push_back(
        {const_cast<Loop *>(Key.first), Key.second, Value});
  }

  errs() << "\n\n\nPeeling Plan (merged):\n";
  for (auto &[Key, Value] : ActionMap) {
    errs() << "Outermost Loop: " << Key->getName() << "\n";
    for (auto &Action : Value) {
      errs().indent(2) << "\tLoop: " << Action.TargetLoop->getName()
                       << ", Direction: " << toString(Action.Direction)
                       << ", Count: " << Action.MaxPeelCount << "\n";
    }
  }
  errs() << "\n\n\n";

  for (auto &[Key, Value] : ActionMap) {
    std::sort(Value.begin(), Value.end(),
              [](const PeelingAction &A, const PeelingAction &B) {
                unsigned DepthA = A.TargetLoop->getLoopDepth();
                unsigned DepthB = B.TargetLoop->getLoopDepth();

                if (DepthA != DepthB)
                  return DepthA < DepthB;

                if (A.Direction != B.Direction)
                  return A.Direction < B.Direction;
                return false;
              });
  }

  errs() << "\n\n\nPeeling Plan (merged):\n";
  for (auto &[Key, Value] : ActionMap) {
    errs() << "Outermost Loop: " << Key->getName() << "\n";
    for (auto &Action : Value) {
      errs().indent(2) << "\tLoop: " << Action.TargetLoop->getName()
                       << ", Direction: " << toString(Action.Direction)
                       << ", Count: " << Action.MaxPeelCount << "\n";
    }
  }
  errs() << "\n\n\n";

  for (auto &[Key, Value] : ActionMap) {
    size_t Accumulator = 0;
    for (auto It = Value.rbegin(); It != Value.rend(); ++It) {
      Accumulator += It->MaxPeelCount;
      It->MaxPeelCount = Accumulator;
    }
  }

  errs() << "\n\n\nPeeling Plan (merged):\n";
  for (auto &[Key, Value] : ActionMap) {
    errs() << "Outermost Loop: " << Key->getName() << "\n";
    for (auto &Action : Value) {
      errs().indent(2) << "\tLoop: " << Action.TargetLoop->getName()
                       << ", Direction: " << toString(Action.Direction)
                       << ", Count: " << Action.MaxPeelCount << "\n";
    }
  }
  errs() << "\n\n\n";

  return ActionMap;
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

    // Replace uses of the IV in the peeled loop with the hardcoded value (N-k)
    {
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
  }
}

void clearCondition(PeelingAction &PA, ScalarEvolution &SE) {
  SE.forgetAllLoops();

  const SCEV *BTC = SE.getBackedgeTakenCount(PA.TargetLoop);
  if (not isa<SCEVCouldNotCompute>(BTC)) {
    for (BasicBlock *BB : PA.TargetLoop->blocks()) {
      for (Instruction &I : llvm::make_early_inc_range(*BB)) {
        if (auto *Cmp = dyn_cast<ICmpInst>(&I)) {
          if (Cmp->getPredicate() == CmpInst::ICMP_ULT ||
              Cmp->getPredicate() == CmpInst::ICMP_SLT) {
            errs() << "Examining ICmp: " << *Cmp << "\n";

            const SCEV *LHS = SE.getSCEV(Cmp->getOperand(0));
            const SCEV *RHS = SE.getSCEV(Cmp->getOperand(1));
            const SCEV *Delta = SE.getMinusSCEV(RHS, LHS);

            errs() << "  LHS: " << *LHS << "\n";
            errs() << "  RHS: " << *RHS << "\n";
            errs() << "  Delta: " << *Delta << "\n";

            if (auto *AR = dyn_cast<SCEVAddRecExpr>(Delta)) {
              const SCEV *BTC = SE.getBackedgeTakenCount(PA.TargetLoop);

              errs() << "  Found AddRec in Delta: " << *AR << "\n";
              errs() << "  Backedge Taken Count: " << *BTC << "\n";

              if (!isa<SCEVCouldNotCompute>(BTC)) {
                errs() << "  Evaluating at iteration BTC\n";

                const SCEV *Step = AR->getStepRecurrence(SE);

                const SCEV *CriticalValue = nullptr;

                if (SE.isKnownNonPositive(Step)) {
                  CriticalValue = AR->evaluateAtIteration(BTC, SE);
                } else if (SE.isKnownNonNegative(Step)) {
                  CriticalValue = AR->getStart();
                }

                if (CriticalValue && SE.isKnownPositive(CriticalValue)) {
                  errs() << "  Proven always true based on SCEV!\n";
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
    errs() << "Boucle triangulaire detectee: " << L->getHeader()->getName()
           << "\n";
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
  auto PeelingPlanMap = getOptimizedPeelingPlan(Map);

  for (auto &[Key, Value] : PeelingPlanMap) {
    errs() << "Outermost Loop: " << Key->getName() << "\n";
    for (auto &Action : Value) {
      errs().indent(2) << "\tLoop: " << Action.TargetLoop->getName()
                       << ", Direction: " << toString(Action.Direction)
                       << ", Count: " << Action.MaxPeelCount << "\n";
    }
  }

  auto &AC = AM.getResult<AssumptionAnalysis>(F);
  for (auto &[Key, Value] : PeelingPlanMap) {
    for (auto &PA : Value) {
      transformTriangularLoops(PA, LI, SE, DT, AC);
      clearCondition(PA, SE);
    }
  }

  if (verifyFunction(F, &errs())) {
    report_fatal_error("IR verification failed.");
  }

  LLVM_DEBUG(errs() << "TriangularLoopFixPass pass done\n";);

  return PreservedAnalyses::none();
}
