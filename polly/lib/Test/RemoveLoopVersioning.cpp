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

#include "polly/Test/RemoveLoopVersioning.h"
#include "polly/ScopDetectionDiagnostic.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/Analysis/AssumptionCache.h"
#include "llvm/Analysis/LoopInfo.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/CFG.h"
#include "llvm/IR/Dominators.h"
#include "llvm/IR/InstrTypes.h"
#include "llvm/IR/Instruction.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/PassManager.h"
#include "llvm/IR/Use.h"
#include "llvm/IR/Verifier.h"
#include "llvm/Passes/PassBuilder.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"

#define DEBUG_TYPE "polly-remove-loop-versioning"

using namespace llvm;
using namespace polly;

namespace {

bool isLoopBoundCondition(Value *Val, LoopInfo &LI) {
  llvm::SmallVector<Instruction *, 4> InstructionToTest;
  for (auto &U : Val->uses()) {
    Instruction *UserInst = dyn_cast<Instruction>(U.getUser());
    if (not UserInst)
      continue;

    if (isa<ZExtInst>(UserInst) or isa<SExtInst>(UserInst) or
        isa<TruncInst>(UserInst)) {
      for (auto &UU : UserInst->uses()) {
        Instruction *UserUserInst = dyn_cast<Instruction>(UU.getUser());
        if (not UserUserInst)
          continue;
        InstructionToTest.push_back(UserUserInst);
      }
    } else {
      InstructionToTest.push_back(UserInst);
    }
  }

  for (auto &UserInst : InstructionToTest) {
    // errs() << "User instruction: " << *UserInst << "\n";
    if (not UserInst)
      continue;

    ICmpInst *CmpInst = dyn_cast<ICmpInst>(UserInst);
    if (not CmpInst)
      continue;
    BasicBlock *CmpBlock = CmpInst->getParent();
    auto *Loop = LI.getLoopFor(CmpBlock);
    if (not Loop)
      continue;

    if (BranchInst *BI = dyn_cast<BranchInst>(CmpBlock->getTerminator())) {
      if (BI->isConditional() && BI->getCondition() == CmpInst) {
        if (Loop->isLoopExiting(CmpBlock)) {
          return true;
          // errs() << "  [FOUND] Cette instruction est une condition "
          //        << "de sortie de boucle !\n";
          break;
        }
      }
    }
  }
  return false;
}

void removeUnswitchedBranches(Function &F, LoopInfo &LI) {
  for (BasicBlock &BB : F) {
    // errs() << "\n\nVisiting block: " << BB.getName() << "\n";

    // check if block have itself in its successors (loop backedge)
    if (std::find(succ_begin(&BB), succ_end(&BB), &BB) != succ_end(&BB)) {
      // errs() << "Skipping block with self-loop\n";
      continue;
    }

    Instruction *Term = BB.getTerminator();
    // errs() << "Terminator: " << *Term << "\n";

    if (BranchInst *Branch = dyn_cast<BranchInst>(Term)) {
      if (!Branch->isConditional() || Branch->getNumSuccessors() != 2) {
        // errs() << "Skipping non-conditional or non-binary branch\n";
        continue;
      }
      Value *Condition = Branch->getCondition();
      if (auto *ICmp = dyn_cast<ICmpInst>(Condition)) {
        Value *LHS = ICmp->getOperand(0);
        Value *RHS = ICmp->getOperand(1);
        ICmpInst::Predicate Pred = ICmp->getPredicate();
        // errs() << "Removing unswitched branch: " << *ICmp << "\n";
        // errs() << "LHS: " << *LHS << "\n";
        // errs() << "RHS: " << *RHS << "\n";
        // errs() << "Predicate: " << Pred << "\n";

        ConstantInt *ConstOp = nullptr;
        Value *VariableOp = nullptr;

        if ((ConstOp = dyn_cast<ConstantInt>(RHS))) {
          VariableOp = LHS;
        } else if ((ConstOp = dyn_cast<ConstantInt>(LHS))) {
          VariableOp = RHS;
        }

        if (not VariableOp or not ConstOp) {
          // errs() << "No constant operand found, skipping\n";
          continue;
        }

        if (not isLoopBoundCondition(VariableOp, LI)) {
          // errs() << "Variable operand is not from a loop bound, skipping\n";
          continue;
        }

        switch (Pred) {
        case ICmpInst::ICMP_EQ: {
          // errs() << "on sup Branch " << *Branch << "\n";
          // errs() << "on branch to " << *Branch->getSuccessor(1) << "\n";
          BasicBlock *ParentBlock = Branch->getParent();
          BasicBlock *NextBB = Branch->getSuccessor(1);
          BasicBlock *RemovedBlock = Branch->getSuccessor(0);

          for (PHINode &Phi : RemovedBlock->phis()) {
            int Idx = Phi.getBasicBlockIndex(ParentBlock);
            if (Idx != -1) {
              Phi.removeIncomingValue(Idx);
            }
          }

          IRBuilder<> Builder(Branch);
          ICmp->setPredicate(
              CmpInst::getInversePredicate(ICmp->getPredicate()));
          Value *Assume = Builder.CreateAssumption(ICmp);
          LLVM_DEBUG(errs() << "Registering assumption: " << *Assume << "\n");
          LLVM_DEBUG(errs() << "Removing branch to " << *Branch << "\n";);
          BranchInst *NewBranch = BranchInst::Create(NextBB);
          Branch->eraseFromParent();
          NewBranch->insertInto(ParentBlock, ParentBlock->end());
          break;
        }
        default: {
          LLVM_DEBUG(errs() << "Unsupported predicate, skipping\n";);
          break;
        }
        }
      }
    }
  }
}

} // namespace

PreservedAnalyses RemoveLoopVersioningPass::run(Function &F,
                                                FunctionAnalysisManager &AM) {
  if (not F.hasFnAttribute("polly.findSCoP"))
    return PreservedAnalyses::all();

  LLVM_DEBUG(errs() << "RemoveLoopVersioningPass pass run on " << F.getName()
                    << "\n";);

  LoopInfo &LI = AM.getResult<LoopAnalysis>(F);
  removeUnswitchedBranches(F, LI);

  if (verifyFunction(F, &errs())) {
    report_fatal_error("IR verification failed.");
  }

  LLVM_DEBUG(errs() << "RemoveLoopVersioningPass pass done\n";);

  return PreservedAnalyses::none();
}
