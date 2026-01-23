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
#include "polly/Test/ExtractAnnotatedFromLoop.h"
#include "polly/Test/LoopFusion.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/Analysis/AssumptionCache.h"
#include "llvm/Analysis/LoopInfo.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/CFG.h"
#include "llvm/IR/Dominators.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/InstrTypes.h"
#include "llvm/IR/Instruction.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/PassManager.h"
#include "llvm/IR/Use.h"
#include "llvm/IR/Verifier.h"
#include "llvm/Passes/PassBuilder.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/ErrorHandling.h"

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
    errs() << "bute\n";
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

Value *getBooleanValue(ICmpInst *ICmp) {
  ICmpInst::Predicate Pred = ICmp->getPredicate();

  int ZeroOpIndex = 0;
  if (isa<ConstantInt>(ICmp->getOperand(1)))
    ZeroOpIndex = 1;

  switch (Pred) {
  case ICmpInst::ICMP_EQ: {
    ICmp->setPredicate(ICmp->getInversePredicate());
    return ConstantInt::getFalse(ICmp->getContext());
  }
  case ICmpInst::ICMP_ULT:
  case ICmpInst::ICMP_UGT:
  case ICmpInst::ICMP_SGT: {
    if (ZeroOpIndex == 0)
      llvm_unreachable("Something's wrong with loop bound condition");
    return ConstantInt::getTrue(ICmp->getContext());
  }
  default:
    llvm_unreachable("Unsupported loop bound condition in getBooleanValue");
    break;
  }

  return ConstantInt::getFalse(ICmp->getContext());
}

void removeLoopBoundVarConditions(Function &F, AssumptionCache &AC) {
  struct HoistedData {
    CallInst *Call;
    BinaryOperator *Operation;
    ICmpInst *ICmp;
    bool IsLoopBound = false;
  };

  auto Lambda = [&](ICmpInst *ICmp, CallInst *CallInst, BinaryOperator *BinOp,
                    SmallVectorImpl<HoistedData> &ToChangee) {
    bool HasConstant = false;
    {
      for (auto &Op : ICmp->operands()) {
        Instruction *Tmp = BinOp ? dyn_cast<Instruction>(BinOp)
                                 : dyn_cast<Instruction>(CallInst);
        if (Op == Tmp)
          continue;
        if (isa<ConstantInt>(Op)) {
          HasConstant = true;
        }
      }
    }
    bool IsLoopBound = false;
    {
      for (auto &Op : CallInst->operands()) {
        auto *Inst = dyn_cast<Instruction>(Op);
        if (not Inst)
          continue;
        errs() << "Checking metadata for instruction: " << *Inst << "\n";

        if (Inst->hasMetadata("loop_bound_information")) {
          errs() << "Found loop bound metadata on instruction: " << *Inst
                 << "\n";
          IsLoopBound = true;
          break;
        }
      }
    }
    if (not HasConstant and not IsLoopBound)
      return;

    if (HasConstant) {
      Value *BooleanVal2 = getBooleanValue(ICmp);
      ICmp->replaceAllUsesWith(BooleanVal2);
      ToChangee.push_back(HoistedData{CallInst, BinOp, ICmp});
    }
    if (IsLoopBound) {
      ToChangee.push_back(HoistedData{CallInst, BinOp, ICmp, true});
    }

    LLVM_DEBUG(errs() << "Removing loop bound variable condition " << *ICmp
                      << "\n";);
  };

  // useful with the annotation of loop bounds inside parallel_for with
  // KOKKOS_LOOP_BOUND
  SmallVector<HoistedData, 2> ToChangee;
  for (auto &BB : F) {
    for (auto It = BB.begin(); It != BB.end();) {
      Instruction *I = &*It;
      ++It;
      auto [CallInst, StrRef] = polly::isAnnotationInstruction(I, "var ");
      if (not CallInst)
        continue;

      bool IsOnlyUseForAssumption = true;
      for (User *U : CallInst->users()) {
        if (auto *ICmp = dyn_cast<ICmpInst>(U)) {
          for (auto *UserOfICmp : ICmp->users()) {
            if (not isa<AssumeInst>(UserOfICmp)) {
              IsOnlyUseForAssumption = false;
            }
          }
        } else if (auto *BinOp = dyn_cast<BinaryOperator>(U)) {
          for (auto *UserOfBinOp : BinOp->users()) {
            if (auto *ICmp = dyn_cast<ICmpInst>(UserOfBinOp)) {
              for (auto *UserOfICmp : ICmp->users()) {
                if (not isa<AssumeInst>(UserOfICmp)) {
                  IsOnlyUseForAssumption = false;
                }
              }
            } else {
              IsOnlyUseForAssumption = false;
            }
          }
        } else {
          IsOnlyUseForAssumption = false;
        }
      }
      if (IsOnlyUseForAssumption)
        continue;

      errs() << "Processing annotation call: " << *CallInst << "\n";

      for (User *U : CallInst->users()) {
        if (auto *ICmp = dyn_cast<ICmpInst>(U)) {
          Lambda(ICmp, CallInst, nullptr, ToChangee);
        } else if (auto *BinOp = dyn_cast<BinaryOperator>(U)) {
          errs() << "Binary operator found: " << *BinOp << "\n";
          // find the constant
          unsigned ConstantIndex = 2;
          for (unsigned I = 0; I < 2; ++I) {
            if (isa<ConstantInt>(BinOp->getOperand(I)))
              ConstantIndex = I;
          }
          if (ConstantIndex == 2)
            continue;

          for (auto *UserOfBinOp : BinOp->users()) {
            if (auto *ICmp = dyn_cast<ICmpInst>(UserOfBinOp)) {
              Lambda(ICmp, CallInst, BinOp, ToChangee);
            }
          }
        }
      }
    }
  }

  for (auto &[CallInst, BinaryOp, ICmp, IsLoopBound] : ToChangee) {
    // replace the annotation value with the original value
    auto *Val = CallInst->getArgOperand(0);
    CallInst->replaceAllUsesWith(Val);

    if (IsLoopBound) {
      errs() << "Skipping assumption creation for loop bound variable: "
             << *ICmp << "\n";
      continue;
    }

    auto *ValInst = dyn_cast_or_null<Instruction>(Val);
    if (not ValInst)
      llvm_unreachable("Expected instruction");

    // Clone the instruction comparing
    auto *NewICmpInst = ICmp->clone();
    auto *NewICmp = dyn_cast_or_null<ICmpInst>(NewICmpInst);
    NewICmp->setName("var_loop_bound_icmp");
    NewICmp->insertAfter(ValInst->getNextNode());

    // Assumption creation and insertion
    auto Builder = IRBuilder<>(NewICmp->getNextNode());
    Value *Assumption = Builder.CreateAssumption(NewICmp);
    auto *AssumptionInst = dyn_cast_or_null<Instruction>(Assumption);
    auto *AssumptionCall = dyn_cast_or_null<llvm::AssumeInst>(AssumptionInst);
    if (not AssumptionInst or not AssumptionCall)
      llvm_unreachable("Expected assume instruction");
    AC.registerAssumption(AssumptionCall);

    // if BianryOp is not null, we need to clone it
    if (BinaryOp) {
      auto *NewBinaryOp = BinaryOp->clone();
      NewBinaryOp->setName("var_loop_bound_binop");
      NewBinaryOp->insertBefore(NewICmp);

      unsigned ConstantIndexCmp = 2;
      for (unsigned I = 0; I < 2; ++I) {
        if (isa<ConstantInt>(NewICmp->getOperand(I)))
          ConstantIndexCmp = I;
      }

      errs() << "binary op: " << *BinaryOp << "\n";

      NewICmp->setOperand(1 - ConstantIndexCmp, NewBinaryOp);
    }

    LLVM_DEBUG(errs() << "Registering assumption: " << *NewICmp << "  "
                      << *Assumption << "\n");
  }
}

} // namespace

PreservedAnalyses
RemoveLoopBoundConditionPass::run(Function &F, FunctionAnalysisManager &AM) {
  if (not F.hasFnAttribute("polly.findSCoP"))
    return PreservedAnalyses::all();

  LLVM_DEBUG(errs() << "RemoveLoopBoundConditionPass pass run on "
                    << F.getName() << "\n";);

  auto LBA = LoopBoundAnalysis().run(F, AM);
  LLVM_DEBUG(errs() << LBA << "\n";);
  removeLoopBoundConditions(F, LBA);
  auto AC = AssumptionAnalysis().run(F, AM);
  removeLoopBoundVarConditions(F, AC);

  if (verifyFunction(F, &errs())) {
    report_fatal_error("IR verification failed.");
  }

  LLVM_DEBUG(errs() << "RemoveLoopBoundConditionPass pass done\n";);

  return PreservedAnalyses::none();
}
