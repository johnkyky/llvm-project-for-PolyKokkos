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

#include "polly/Test/RemoveLoopBoundCondition.h"
#include "polly/ScopDetectionDiagnostic.h"
#include "polly/Test/LoopFusion.h"
#include "llvm/Analysis/AssumptionCache.h"
#include "llvm/Analysis/LoopInfo.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/CFG.h"
#include "llvm/IR/Dominators.h"
#include "llvm/IR/Instruction.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/PassManager.h"
#include "llvm/IR/Use.h"
#include "llvm/IR/Verifier.h"
#include "llvm/Passes/PassBuilder.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"

#define DEBUG_TYPE "polly-remove-loop-bound-condition"

using namespace llvm;
using namespace polly;

namespace {

BasicBlock *getTrueCondition(BranchInst *Branch, const Instruction *LeftOp,
                             ICmpInst::Predicate Predica,
                             const Instruction *RightOp) {
  switch (Predica) {
  case ICmpInst::ICMP_ULT: {
    const auto *LeftMD = LeftOp->getMetadata("loop_bound_information");
    MDString *LeftMDStr = dyn_cast<MDString>(LeftMD->getOperand(1));
    const auto LeftBoundKind = LeftMDStr->getString();

    const auto *RightMD = RightOp->getMetadata("loop_bound_information");
    MDString *RightMDStr = dyn_cast<MDString>(RightMD->getOperand(1));
    const auto RightBoundKind = RightMDStr->getString();

    if (LeftBoundKind == "lower" && RightBoundKind == "upper") {
      return Branch->getSuccessor(0);
    }
    if (LeftBoundKind == "upper" && RightBoundKind == "lower") {
      return Branch->getSuccessor(1);
    }
    break;
  }
  default:
    break;
  }
  report_fatal_error("Invalid loop bound condition");
}

unsigned getValidCondition(const Instruction *LeftOp,
                           ICmpInst::Predicate Predica,
                           const Instruction *RightOp) {
  switch (Predica) {
  case ICmpInst::ICMP_ULT: {
    const auto *LeftMD = LeftOp->getMetadata("loop_bound_information");
    MDString *LeftMDStr = dyn_cast<MDString>(LeftMD->getOperand(1));
    const auto LeftBoundKind = LeftMDStr->getString();

    const auto *RightMD = RightOp->getMetadata("loop_bound_information");
    MDString *RightMDStr = dyn_cast<MDString>(RightMD->getOperand(1));
    const auto RightBoundKind = RightMDStr->getString();

    if (LeftBoundKind == "lower" && RightBoundKind == "upper") {
      return 1;
    }
    if (LeftBoundKind == "upper" && RightBoundKind == "lower") {
      return 2;
    }
    break;
  }
  default:
    break;
  }
  report_fatal_error("Invalid loop bound condition");
}

void removeLoopBoundConditions(Function &F,
                               const SmallVector<LoopBoundT, 4> LoopBounds) {

  auto CheckInstIsLoopBounds = [&](const Instruction *LHSInst,
                                   const Instruction *RHSInst) {
    const auto *ItLeft =
        std::find_if(LoopBounds.begin(), LoopBounds.end(),
                     [=](LoopBoundT LB) { return LB.Inst == LHSInst; });
    const auto *ItRight =
        std::find_if(LoopBounds.begin(), LoopBounds.end(),
                     [=](LoopBoundT LB) { return LB.Inst == RHSInst; });

    if (not(ItLeft != LoopBounds.end() and ItRight != LoopBounds.end()))
      return false;
    return true;
  };

  for (auto &BB : F) {
    auto *Term = BB.getTerminator();
    if (auto *Branch = dyn_cast<BranchInst>(Term)) {
      if (not Branch->isConditional())
        continue;

      Value *Cond = Branch->getCondition();

      if (auto *ICmp = dyn_cast<ICmpInst>(Cond)) {
        Value *LHS = ICmp->getOperand(0);
        Value *RHS = ICmp->getOperand(1);
        ICmpInst::Predicate Pred = ICmp->getPredicate();

        const auto *LHSInst = dyn_cast<Instruction>(LHS);
        const auto *RHSInst = dyn_cast<Instruction>(RHS);

        if (not CheckInstIsLoopBounds(LHSInst, RHSInst))
          continue;

        auto *NextBB = getTrueCondition(Branch, LHSInst, Pred, RHSInst);

        IRBuilder<> Builder(Branch);
        Value *Assume = Builder.CreateAssumption(ICmp);
        LLVM_DEBUG(errs() << "Registering assumption: " << *Assume << "\n");

        BranchInst::Create(NextBB, Branch);
        LLVM_DEBUG(errs() << "Removing loop bound condition " << *ICmp << "\n");
        Branch->eraseFromParent();
      } else if (auto *Select = dyn_cast<SelectInst>(Cond)) {
        unsigned IndexOp = 0;
        ICmpInst *ICmp1 = nullptr;
        if ((ICmp1 = dyn_cast<ICmpInst>(Select->getCondition()))) {
          Value *LHS = ICmp1->getOperand(0);
          Value *RHS = ICmp1->getOperand(1);
          ICmpInst::Predicate Pred = ICmp1->getPredicate();

          const auto *LHSInst = dyn_cast<Instruction>(LHS);
          const auto *RHSInst = dyn_cast<Instruction>(RHS);

          if (not CheckInstIsLoopBounds(LHSInst, RHSInst))
            continue;

          IndexOp = getValidCondition(LHSInst, Pred, RHSInst);
        }
        if (IndexOp == 0)
          continue;
        if (auto *ICmp2 = dyn_cast<ICmpInst>(Select->getOperand(IndexOp))) {
          Value *LHS = ICmp2->getOperand(0);
          Value *RHS = ICmp2->getOperand(1);
          ICmpInst::Predicate Pred = ICmp2->getPredicate();

          const auto *LHSInst = dyn_cast<Instruction>(LHS);
          const auto *RHSInst = dyn_cast<Instruction>(RHS);

          if (not CheckInstIsLoopBounds(LHSInst, RHSInst))
            continue;

          auto *NextBB = getTrueCondition(Branch, LHSInst, Pred, RHSInst);

          IRBuilder<> Builder(Branch);
          Value *Assume1 = Builder.CreateAssumption(ICmp1);
          Value *Assume2 = Builder.CreateAssumption(ICmp2);
          LLVM_DEBUG(errs() << "Registering assumption: " << *Assume1 << "\n");
          LLVM_DEBUG(errs() << "Registering assumption: " << *Assume2 << "\n");

          BranchInst::Create(NextBB, Branch);
          LLVM_DEBUG(errs() << "Removing loop bound condition " << *ICmp1
                            << "   " << *ICmp2 << "\n");
          Branch->eraseFromParent();
        }
      } else if (isa<FCmpInst>(Cond)) {
        // Do nothing for floating point comparisons for now
      } else {
        std::string Msg =
            "Unknown branch condition type : " + Cond->getName().str();
        StringRef MsgRef(Msg);
        report_fatal_error(MsgRef);
      }
    }
  }
  return;
}

} // namespace

PreservedAnalyses
RemoveLoopBoundConditionPass::run(Function &F, FunctionAnalysisManager &AM) {
  if (not F.hasFnAttribute("polly.findSCoP"))
    return PreservedAnalyses::all();

  LLVM_DEBUG(errs() << "RemoveLoopBoundConditionPass pass run on "
                    << F.getName() << "\n";);

  auto &LBA = AM.getResult<LoopBoundAnalysis>(F);
  LLVM_DEBUG(errs() << LBA << "\n";);
  removeLoopBoundConditions(F, LBA);

  if (verifyFunction(F, &errs())) {
    report_fatal_error("IR verification failed.");
  }

  LLVM_DEBUG(errs() << "RemoveLoopBoundConditionPass pass done\n";);

  return PreservedAnalyses::none();
}
