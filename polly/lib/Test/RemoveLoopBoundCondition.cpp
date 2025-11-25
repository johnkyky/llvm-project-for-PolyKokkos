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

BasicBlock *getTrueCondition(BranchInst *Branch, const Value *LeftOpVal,
                             ICmpInst::Predicate Predica,
                             const Value *RightOpVal, const ICmpInst *ICmp) {
  switch (Predica) {
  case ICmpInst::ICMP_ULT: {
    bool LeftIsConst = isa<ConstantInt>(LeftOpVal);
    bool RightIsConst = isa<ConstantInt>(RightOpVal);

    StringRef LeftBoundKind;
    StringRef RightBoundKind;
    if (not LeftIsConst and not RightIsConst) {
      const Instruction *LeftOp = dyn_cast<Instruction>(LeftOpVal);
      const auto *LeftMD = LeftOp->getMetadata("loop_bound_information");
      MDString *LeftMDStr = dyn_cast<MDString>(LeftMD->getOperand(1));
      LeftBoundKind = LeftMDStr->getString();

      const Instruction *RightOp = dyn_cast<Instruction>(RightOpVal);
      const auto *RightMD = RightOp->getMetadata("loop_bound_information");
      MDString *RightMDStr = dyn_cast<MDString>(RightMD->getOperand(1));
      RightBoundKind = RightMDStr->getString();
    } else {
      if (not ICmp->hasMetadata("old_loop_bound")) {
        report_fatal_error(
            "Loop bound condition missing old_loop_bound metadata");
      }
      auto *OldMD = ICmp->getMetadata("old_loop_bound");
      MDString *OldMDStr = dyn_cast<MDString>(OldMD->getOperand(1));
      StringRef OldBoundKind = OldMDStr->getString();
      if (LeftIsConst) {
        LeftBoundKind = OldBoundKind;
        const Instruction *RightOp = dyn_cast<Instruction>(RightOpVal);
        const auto *RightMD = RightOp->getMetadata("loop_bound_information");
        MDString *RightMDStr = dyn_cast<MDString>(RightMD->getOperand(1));
        RightBoundKind = RightMDStr->getString();
      } else {
        RightBoundKind = OldBoundKind;
        const Instruction *LeftOp = dyn_cast<Instruction>(LeftOpVal);
        const auto *LeftMD = LeftOp->getMetadata("loop_bound_information");
        MDString *LeftMDStr = dyn_cast<MDString>(LeftMD->getOperand(1));
        LeftBoundKind = LeftMDStr->getString();
      }
    }

    if (LeftBoundKind == "lower" && RightBoundKind == "upper") {
      return Branch->getSuccessor(0);
    }
    if (LeftBoundKind == "upper" && RightBoundKind == "lower") {
      return Branch->getSuccessor(1);
    }
    break;
  }
  default:
    llvm_unreachable("Unsupported loop bound condition");
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
                                   const Instruction *RHSInst,
                                   const ICmpInst *ICmp) {
    const auto *ItLeft =
        std::find_if(LoopBounds.begin(), LoopBounds.end(),
                     [=](LoopBoundT LB) { return LB.Inst == LHSInst; });
    const auto *ItRight =
        std::find_if(LoopBounds.begin(), LoopBounds.end(),
                     [=](LoopBoundT LB) { return LB.Inst == RHSInst; });

    if ((ItLeft == LoopBounds.end()) xor (ItRight == LoopBounds.end())) {
      return ICmp->hasMetadata("old_loop_bound");
    }

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

        if (not CheckInstIsLoopBounds(LHSInst, RHSInst, ICmp))
          continue;

        auto *NextBB = getTrueCondition(Branch, LHS, Pred, RHS, ICmp);

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

          if (not CheckInstIsLoopBounds(LHSInst, RHSInst, ICmp1))
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

          if (not CheckInstIsLoopBounds(LHSInst, RHSInst, ICmp2))
            continue;

          auto *NextBB =
              getTrueCondition(Branch, LHSInst, Pred, RHSInst, ICmp2);

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

void removeLoopBoundVarConditions(Function &F) {
  // useful with the annotation of loop bounds inside parallel_for
  for (auto &BB : F) {
    for (auto It = BB.begin(); It != BB.end();) {
      Instruction *I = &*It;
      ++It;
      if (auto *CallInst = dyn_cast<llvm::CallInst>(I)) {
        const Function *Callee = CallInst->getCalledFunction();
        if (not Callee)
          continue;
        if (Callee->getName().starts_with("llvm.annotation")) {
          for (User *U : CallInst->users()) {
            if (Instruction *ICmp = dyn_cast<ICmpInst>(U)) {
              bool IsConstantZero = false;
              for (auto &Op : ICmp->operands()) {
                if (Op == CallInst)
                  continue;
                if (auto *ConstOp = dyn_cast<ConstantInt>(Op)) {
                  if (ConstOp->isZero())
                    IsConstantZero = true;
                }
              }
              if (not IsConstantZero)
                continue;

              LLVM_DEBUG(errs() << "Removing loop bound variable condition "
                                << *ICmp << "\n";);
              Value *FalseVal = ConstantInt::getFalse(ICmp->getContext());
              ICmp->replaceAllUsesWith(FalseVal);
            }
          }
        }
      }
    }
  }
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
  removeLoopBoundVarConditions(F);

  if (verifyFunction(F, &errs())) {
    report_fatal_error("IR verification failed.");
  }

  LLVM_DEBUG(errs() << "RemoveLoopBoundConditionPass pass done\n";);

  return PreservedAnalyses::none();
}
