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
  case ICmpInst::ICMP_SLT:
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

  auto CheckInstIsLoopBounds = [&](const Value *LHS, const Value *RHS,
                                   const ICmpInst *ICmp) {
    const auto *LHSInst = dyn_cast_or_null<Instruction>(LHS);
    const auto *RHSInst = dyn_cast_or_null<Instruction>(RHS);

    if (auto *LCast = llvm::dyn_cast_or_null<CastInst>(LHSInst))
      LHSInst = llvm::dyn_cast<llvm::Instruction>(LCast->getOperand(0));
    if (auto *RCast = llvm::dyn_cast_or_null<llvm::CastInst>(RHSInst))
      RHSInst = llvm::dyn_cast<llvm::Instruction>(RCast->getOperand(0));

    auto *ItLeft = LoopBounds.end();
    auto *ItRight = LoopBounds.end();

    if (LHSInst)
      ItLeft = std::find_if(LoopBounds.begin(), LoopBounds.end(),
                            [=](LoopBoundT LB) { return LB.Inst == LHSInst; });
    if (RHSInst)
      ItRight = std::find_if(LoopBounds.begin(), LoopBounds.end(),
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

        errs() << "LHS : " << *LHS << "\n";
        errs() << "RHS : " << *RHS << "\n";

        if (auto *LCast = llvm::dyn_cast_or_null<CastInst>(LHS)) {
          if (isa<Instruction>(LCast->getOperand(0)))
            LHS = llvm::dyn_cast<llvm::Instruction>(LCast->getOperand(0));
          else if (isa<ConstantInt>(LCast->getOperand(0)))
            LHS = llvm::dyn_cast<llvm::ConstantInt>(LCast->getOperand(0));
          else
            llvm_unreachable("Loop bound annotation operand is neither an "
                             "instruction nor a constant integer");
        }
        if (auto *RCast = llvm::dyn_cast_or_null<llvm::CastInst>(RHS)) {
          if (isa<Instruction>(RCast->getOperand(0)))
            RHS = llvm::dyn_cast<llvm::Instruction>(RCast->getOperand(0));
          else if (isa<ConstantInt>(RCast->getOperand(0)))
            RHS = llvm::dyn_cast<llvm::ConstantInt>(RCast->getOperand(0));
          else
            llvm_unreachable("Loop bound annotation operand is neither an "
                             "instruction nor a constant integer");
        }

        if (not CheckInstIsLoopBounds(LHS, RHS, ICmp))
          continue;

        auto *NextBB = getTrueCondition(Branch, LHS, Pred, RHS, ICmp);

        IRBuilder<> Builder(Branch);
        Value *Assume = Builder.CreateAssumption(ICmp);
        LLVM_DEBUG(errs() << "Registering assumption: " << *Assume << "\n");

        BranchInst::Create(NextBB, Branch->getIterator());
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

          BranchInst::Create(NextBB, Branch->getIterator());
          LLVM_DEBUG(errs() << "Removing loop bound condition " << *ICmp1
                            << "   " << *ICmp2 << "\n");
          Branch->eraseFromParent();
        }
      } else if (isa<FCmpInst>(Cond)) {
        // Do nothing for floating point comparisons
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
  case ICmpInst::ICMP_SLT: {
    if (ZeroOpIndex == 1)
      llvm_unreachable("Something's wrong with loop bound condition");
    return ConstantInt::getTrue(ICmp->getContext());
  }
  default:
    errs() << *ICmp->getParent() << "\n";
    llvm_unreachable("Unsupported loop bound condition in getBooleanValue");
    break;
  }

  return ConstantInt::getFalse(ICmp->getContext());
}

bool isValidBindOp(const BinaryOperator *BinOp) {
  bool HasConstant = false;
  for (auto &Op : BinOp->operands()) {
    if (isa<ConstantInt>(Op)) {
      HasConstant = true;
    }
  }

  return HasConstant;
}

void removeLoopBoundVarConditions(Function &F, AssumptionCache &AC) {
  struct HoistedData {
    ICmpInst *ICmp;
    Instruction *VarLoopBound;
    BinaryOperator *BinOp;
  };

  // useful with the annotation of loop bounds inside parallel_for with
  // KOKKOS_LOOP_BOUND
  SmallVector<HoistedData, 2> ToChangee;
  for (auto &BB : F) {
    for (auto &I : BB) {
      if ((not I.hasMetadata("cond_variable_annotation") and
           not I.hasMetadata("old_loop_bound")) or
          I.hasMetadata("used_for_versioning"))
        continue;

      auto *ICmp = dyn_cast<ICmpInst>(&I);
      if (not ICmp)
        continue;

      errs() << "\n\nFound loop bound variable condition: " << *ICmp << "\n";

      HoistedData HD{ICmp, nullptr, nullptr};

      bool HasConstant = false;
      bool HasBinOp = false;
      bool HasLoopBound = false;

      auto *Op0 = ICmp->getOperand(0);
      errs() << "Operand 0: " << *Op0 << "\n";
      if (auto *BinOp = dyn_cast<BinaryOperator>(Op0)) {
        if (isValidBindOp(BinOp)) {
          HD.BinOp = BinOp;
          auto *LoopBound = isa<ConstantInt>(BinOp->getOperand(0))
                                ? BinOp->getOperand(1)
                                : BinOp->getOperand(0);
          HD.VarLoopBound = dyn_cast_or_null<Instruction>(LoopBound);
          HasBinOp = true;
        }
      } else if (isa<ConstantInt>(Op0)) {
        HasConstant = true;
      } else {
        HD.VarLoopBound = dyn_cast_or_null<Instruction>(Op0);
        HasLoopBound = true;
      }

      errs() << "Has constant: " << HasConstant << "\n";
      errs() << "Has binary operator: " << HasBinOp << "\n";
      errs() << "Has loop bound variable: " << HasLoopBound << "\n";

      auto *Op1 = ICmp->getOperand(1);
      errs() << "Operand 1: " << *Op1 << "\n";
      if (auto *BinOp = dyn_cast<BinaryOperator>(Op1)) {
        if (isValidBindOp(BinOp)) {
          if (not HasConstant) {
            errs() << "Invalid loop bound condition with binary "
                      "operator without constant\n";
            continue;
          }

          auto *LoopBound = isa<ConstantInt>(BinOp->getOperand(0))
                                ? BinOp->getOperand(1)
                                : BinOp->getOperand(0);
          HD.VarLoopBound = dyn_cast_or_null<Instruction>(LoopBound);
          HD.BinOp = BinOp;
        }
      } else if (isa<ConstantInt>(Op1)) {
        if (not HasBinOp and not HasLoopBound) {
          errs() << "Invalid loop bound condition with constant "
                    "without binary operator or "
                    "loop bound variable\n";
          continue;
        }
        HasConstant = true;
      } else {
        if (not HasConstant) {
          errs() << "Invalid loop bound condition with variable "
                    "without constant\n";
          continue;
        }
        HD.VarLoopBound = dyn_cast_or_null<Instruction>(Op1);
      }

      errs() << "Processing variable loop bound condition: " << *ICmp << "\n";

      if (not HD.VarLoopBound)
        continue;
      if (HD.VarLoopBound->hasMetadata("loop_bound_information"))
        continue;

      if (HasConstant)
        ToChangee.push_back(HD);

      Value *BooleanVal = getBooleanValue(ICmp);
      errs() << "on replace all uses of " << *ICmp << " with " << *BooleanVal
             << "\n";
      ICmp->replaceAllUsesWith(BooleanVal);
    }
  }

  for (auto &[ICmp, VarLoopBound, BinaryOp] : ToChangee) {
    errs() << "Loop bound variable condition to remove: " << *ICmp << "\n";
    errs() << "  Variable loop bound: " << *VarLoopBound << "\n";
    if (BinaryOp)
      errs() << "  Binary operator: " << *BinaryOp << "\n";
  }

  for (auto &[ICmp, VarLoopBound, BinaryOp] : ToChangee) {
    // Clone the instruction comparing
    auto *NewICmpInst = ICmp->clone();
    auto *NewICmp = dyn_cast_or_null<ICmpInst>(NewICmpInst);
    NewICmp->setName("var_loop_bound_icmp");
    NewICmp->insertAfter(VarLoopBound->getNextNode());

    // Assumption creation and insertion
    auto Builder = IRBuilder<>(NewICmp->getNextNode());
    auto *Assumption = Builder.CreateAssumption(NewICmp);
    auto *AssumptionInst = dyn_cast_or_null<Instruction>(Assumption);
    auto *AssumptionCall = dyn_cast_or_null<llvm::AssumeInst>(AssumptionInst);
    if (not AssumptionInst or not AssumptionCall)
      llvm_unreachable("Expected assume instruction");
    AC.registerAssumption(AssumptionCall);

    // if BianryOp is not null, we need to clone it
    if (BinaryOp) {
      auto *NewBinaryOp = BinaryOp->clone();
      NewBinaryOp->setName("var_loop_bound_binop");
      NewBinaryOp->insertBefore(NewICmp->getIterator());

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
  auto AC = AM.getResult<AssumptionAnalysis>(F);
  removeLoopBoundVarConditions(F, AC);

  auto Anno = ExtractAnnotatedSizes().run(F, AM);
  for (const auto &[InstArray, Data] : Anno) {
    bool SkippedFirstIt = false;
    for (auto *SizeInst : Data.Sizes) {
      if (not SkippedFirstIt) {
        SkippedFirstIt = true;
        continue;
      }

      // create an assume instruction that the size is > 0
      IRBuilder<> Builder(SizeInst->getNextNode());
      auto *SizeType = SizeInst->getType();
      auto *Zero = ConstantInt::get(SizeType, 0);
      auto *Cond = Builder.CreateICmpSGT(SizeInst, Zero, "array_size_positive");
      auto *Assume = Builder.CreateAssumption(Cond);
      auto *AI = dyn_cast_or_null<AssumeInst>(Assume);
      AC.registerAssumption(AI);
      Assume->setMetadata("array_size_positive",
                          MDNode::get(SizeInst->getContext(), {}));
    }
  }

  if (verifyFunction(F, &errs())) {
    report_fatal_error("IR verification failed.");
  }

  LLVM_DEBUG(errs() << "RemoveLoopBoundConditionPass pass done\n";);

  return PreservedAnalyses::none();
}
