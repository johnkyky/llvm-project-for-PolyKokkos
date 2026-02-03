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

#include "polly/Test/LowerInstructionForReduction.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/PassManager.h"
#include "llvm/IR/Verifier.h"
#include "llvm/Passes/PassBuilder.h"

#define DEBUG_TYPE "polly-lower-instruction-for-reduction"

using namespace llvm;
using namespace polly;

namespace {

/* Lower fmuladd calls into separate fmul and fadd instructions.
 *  This is useful to enable optimizations like those in Polly that may not
 *  handle fmuladd directly.
 */
void lowerFmulAdd(Function &F) {
  SmallVector<CallInst *, 4> ToLower;

  for (auto &BB : F) {
    for (auto &I : BB) {
      if (auto *CI = dyn_cast<CallInst>(&I)) {
        if (auto *F = CI->getCalledFunction()) {
          if (F->getName().starts_with("llvm.fmuladd")) {
            ToLower.push_back(CI);
          }
        }
      }
    }
  }

  for (auto *CI : ToLower) {
    IRBuilder<> Builder(CI);

    // fmuladd(a, b, c)  ===  (a * b) + c
    Value *Op0 = CI->getArgOperand(0);
    Value *Op1 = CI->getArgOperand(1);
    Value *Op2 = CI->getArgOperand(2);

    auto *Mul = Builder.CreateFMul(Op0, Op1, "mul.lowered");
    auto *MulInst = dyn_cast<Instruction>(Mul);

    auto *Add = Builder.CreateFAdd(Mul, Op2, "add.lowered");
    auto *AddInst = dyn_cast<Instruction>(Add);

    if (isa<FPMathOperator>(CI)) {
      MulInst->copyFastMathFlags(CI);
      AddInst->copyFastMathFlags(CI);
    }

    LLVM_DEBUG(errs().indent(4) << "Lowered fmuladd: " << *CI << " to\n"
                                << indent(6) << "->" << *MulInst << "\n"
                                << indent(6) << "->" << *AddInst << "\n");
    CI->replaceAllUsesWith(AddInst);
    CI->eraseFromParent();
  }
}

/* Lower fsub instructions into separate fneg and fadd instructions.
 *  This is useful to enable optimizations reduction that may not handle fsub
 *  directly.
 */
void lowerFSub(Function &F) {
  SmallVector<BinaryOperator *, 4> ToLower;

  for (auto &BB : F) {
    for (auto &I : BB) {
      if (auto *BinOp = dyn_cast<BinaryOperator>(&I)) {
        if (BinOp->getOpcode() == Instruction::FSub) {
          ToLower.push_back(BinOp);
        }
      }
    }
  }

  for (auto *BO : ToLower) {
    LLVM_DEBUG(errs() << "Found FSub to lower: " << *BO << "\n");
  }

  for (auto *Sub : ToLower) {
    IRBuilder<> Builder(Sub);

    // A - B  ===>  A + (-B)
    Value *Op0 = Sub->getOperand(0);
    Value *Op1 = Sub->getOperand(1);

    auto *Neg = Builder.CreateFNeg(Op1, "neg.lowered");
    auto *NegInst = dyn_cast<Instruction>(Neg);

    auto *Add = Builder.CreateFAdd(Op0, Neg, "add.sub_lowered");
    auto *AddInst = dyn_cast<Instruction>(Add);

    if (isa<FPMathOperator>(Sub)) {
      NegInst->copyFastMathFlags(Sub);
      AddInst->copyFastMathFlags(Sub);
    }

    LLVM_DEBUG(errs().indent(4) << "Lowered FSub: " << *Sub << " to\n"
                                << indent(6) << "->" << *NegInst << "\n"
                                << indent(6) << "->" << *AddInst << "\n");
    Sub->replaceAllUsesWith(Add);
    Sub->eraseFromParent();
  }
}
} // namespace

PreservedAnalyses
LowerInstructionForReductionPass::run(Function &F,
                                      FunctionAnalysisManager &AM) {
  if (not F.hasFnAttribute("polly.findSCoP")) {
    return PreservedAnalyses::all();
  }

  LLVM_DEBUG(errs() << "\nLowerInstructionForReductionPass : " << F.getName()
                    << "\n");

  lowerFmulAdd(F);
  lowerFSub(F);

  if (verifyFunction(F, &errs())) {
    report_fatal_error("IR verification failed.");
  }

  LLVM_DEBUG(errs() << "LowerInstructionForReductionPass pass done\n");

  return PreservedAnalyses::all();
}
