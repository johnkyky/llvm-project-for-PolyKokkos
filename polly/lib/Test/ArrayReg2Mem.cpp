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

#include "polly/Test/ArrayReg2Mem.h"
#include "llvm/Analysis/ScalarEvolution.h"
#include "llvm/Analysis/ScalarEvolutionExpressions.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/Dominators.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/Instruction.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/PassManager.h"
#include "llvm/IR/Value.h"
#include "llvm/IR/Verifier.h"
#include "llvm/Passes/PassBuilder.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Transforms/Utils/Local.h"
#include <algorithm>

#define DEBUG_TYPE "polly-array-reg2mem"

using namespace llvm;
using namespace polly;

namespace {

/* Find a store instruction in the given basic block (BB) that stores the
   given value (Val) to a pointer that is invariant with respect to the loop
   (i.e., the pointer is valid at the beginning of the loop, LoopHeader).
   If such a store is found, return the pointer operand of the store.
   Otherwise, return nullptr.
*/
Value *findInvariantStorePointer(Value *Val, BasicBlock *BB, DominatorTree &DT,
                                 BasicBlock *LoopHeader) {

  for (Instruction &I : llvm::reverse(*BB)) {

    if (auto *Store = dyn_cast<StoreInst>(&I)) {
      if (Store->getValueOperand() == Val) {

        Value *Ptr = Store->getPointerOperand();

        bool IsInvariant = true;
        if (auto *InstPtr = dyn_cast<Instruction>(Ptr)) {
          if (!DT.dominates(InstPtr, LoopHeader)) {
            IsInvariant = false;
          }
        }

        if (IsInvariant)
          return Ptr;
      }
    }
  }
  return nullptr;
}

/* Check if the given PHI node (Phi) has a relation to stores in its
   incoming blocks that would allow us to demote it to memory.
   If such a relation is found, return the pointer operand of the store.
   Otherwise, return nullptr.
*/
Value *checkPhiStoreRelation(PHINode *Phi, DominatorTree &DT) {
  if (Phi->getNumIncomingValues() != 2)
    return nullptr;

  BasicBlock *Header = Phi->getParent();

  Value *InVals[2];
  BasicBlock *InBlocks[2];
  Value *StorePtrs[2] = {nullptr, nullptr};

  for (int I = 0; I < 2; ++I) {
    InVals[I] = Phi->getIncomingValue(I);
    InBlocks[I] = Phi->getIncomingBlock(I);
    StorePtrs[I] =
        findInvariantStorePointer(InVals[I], InBlocks[I], DT, Header);
  }

  for (int I = 0; I < 2; ++I) {
    errs().indent(4) << "PHI Incoming Value " << I << ": " << *InVals[I]
                     << " from Block: " << InBlocks[I]->getName() << "\n";
    if (StorePtrs[I]) {
      errs().indent(6) << "-> Found Store to Pointer: " << *StorePtrs[I]
                       << "\n";
    } else {
      errs().indent(6) << "-> No Store found for this incoming value.\n";
    }
  }

  if (StorePtrs[0] && StorePtrs[1] && StorePtrs[0] == StorePtrs[1]) {
    errs().indent(4) << "Match: Both branches use the same pointer.\n";
    return StorePtrs[0];
  }

  if (isa<Constant>(InVals[0]) && StorePtrs[1]) {
    errs().indent(4)
        << "Found Constant entry and Pointer in other branch. Reusing: "
        << *StorePtrs[1] << "\n";
    return StorePtrs[1];
  }

  if (isa<Constant>(InVals[1]) && StorePtrs[0]) {
    errs().indent(4)
        << "Found Constant entry and Pointer in other branch. Reusing: "
        << *StorePtrs[0] << "\n";
    return StorePtrs[0];
  }

  for (int I = 0; I < 2; ++I) {
    if (auto *Load = dyn_cast<LoadInst>(InVals[I])) {
      Value *LoadPtr = Load->getPointerOperand();
      int Other = 1 - I;
      if (StorePtrs[Other] && StorePtrs[Other] == LoadPtr) {
        return LoadPtr;
      }
    }
  }

  return nullptr;
}

/* Materialize PHI Node into memory (Alloca + Stores + Load)
   with proper handling of LCSSA uses outside the loop.
*/
void materializePhi(PHINode *Phi, DominatorTree &DT, LoopInfo &LI) {
  Function *F = Phi->getFunction();
  IRBuilder<> BuilderEntry(&F->getEntryBlock(), F->getEntryBlock().begin());
  AllocaInst *Alloca = BuilderEntry.CreateAlloca(Phi->getType(), nullptr,
                                                 Phi->getName() + ".addr");

  for (unsigned I = 0; I < Phi->getNumIncomingValues(); ++I) {
    Value *Val = Phi->getIncomingValue(I);
    BasicBlock *IncomingBB = Phi->getIncomingBlock(I);

    Instruction *TI = IncomingBB->getTerminator();
    IRBuilder<> BuilderStore(TI);
    BuilderStore.CreateStore(Val, Alloca);
  }

  IRBuilder<> BuilderLoad(Phi->getParent(),
                          Phi->getParent()->getFirstInsertionPt());
  LoadInst *Load = BuilderLoad.CreateLoad(Phi->getType(), Alloca,
                                          Phi->getName() + ".reload");

  Phi->replaceAllUsesWith(Load);
}

void demoteNonIndexPhis(Function &F, LoopInfo &LI, ScalarEvolution &SE,
                        DominatorTree &DT) {

  SmallVector<PHINode *, 16> PhisToDemote;

  for (auto &BB : F) {
    Loop *L = LI.getLoopFor(&BB);
    if (!L || L->getHeader() != &BB)
      continue;

    for (auto &I : BB) {
      auto *Phi = dyn_cast<PHINode>(&I);
      if (!Phi)
        continue;

      if (SE.isSCEVable(Phi->getType())) {
        const auto *S = SE.getSCEV(Phi);
        if (auto *AR = dyn_cast<SCEVAddRecExpr>(S)) {
          if (AR->getLoop() == L)
            continue;
        }
      }
      PhisToDemote.push_back(Phi);
    }
  }

  errs() << "Found " << PhisToDemote.size() << " PHIs to demote in function "
         << F.getName() << "\n";
  for (auto *Phi : PhisToDemote) {
    errs().indent(2) << "PHI to demote: " << *Phi << "\n";
  }

  SmallVector<PHINode *, 0> DeadPhis;
  for (auto *Phi : PhisToDemote) {
    errs() << "Processing PHI: " << *Phi << "\n";

    Value *Ptr = checkPhiStoreRelation(Phi, DT);

    if (Ptr) {
      errs().indent(2) << "Found Existing Pointer for PHI: " << *Ptr << "\n";
      bool PointerIsSafe = true;

      if (auto *InstPtr = dyn_cast<Instruction>(Ptr)) {
        for (unsigned I = 0; I < Phi->getNumIncomingValues(); ++I) {
          BasicBlock *InBlock = Phi->getIncomingBlock(I);
          if (!DT.dominates(InstPtr, InBlock->getTerminator())) {
            PointerIsSafe = false;
            break;
          }
        }
      }

      if (PointerIsSafe) {
        for (unsigned I = 0; I < Phi->getNumIncomingValues(); ++I) {
          Value *InVal = Phi->getIncomingValue(I);
          if (isa<Constant>(InVal)) {
            BasicBlock *InBlock = Phi->getIncomingBlock(I);
            Instruction *Terminator = InBlock->getTerminator();

            IRBuilder<> ConstBuilder(Terminator);
            ConstBuilder.CreateStore(InVal, Ptr);

            errs().indent(2) << "Safe Store for constant: " << *InVal << "\n";
          }
        }
      }
    } else {
      errs().indent(2) << "Materializing PHI to New Alloca: " << *Phi << "\n";
      materializePhi(Phi, DT, LI);
      DeadPhis.push_back(Phi);
    }
  }

  for (auto *Phi : DeadPhis) {
    Phi->eraseFromParent();
  }
}

/* Demote loads that have multiple uses within the same basic block by
   cloning the load for each use. This is useful to enable optimizations like
   those in Polly that may not handle multiple uses of the same load within a
   basic block.
*/
void demoteUsesOfArrays(Function &F, LoopInfo &LI, ScalarEvolution &SE,
                        DominatorTree &DT) {
  errs() << "Demoting uses of arrays\n";
  for (auto &BB : F) {
    if (!LI.getLoopFor(&BB))
      continue;

    SmallVector<LoadInst *, 16> LoadsToDemote;

    for (auto &I : BB) {
      if (auto *Load = dyn_cast<LoadInst>(&I)) {
        BasicBlock *LoadBB = Load->getParent();
        if (Load->getNumUses() > 1) {
          bool NeedsDemotion = true;
          for (auto *U : Load->users()) {
            Instruction *Inst = dyn_cast<Instruction>(U);
            BasicBlock *InstBB = Inst->getParent();

            if (InstBB != LoadBB) {
              NeedsDemotion = false;
              break;
            }
          }

          if (NeedsDemotion) {
            LoadsToDemote.push_back(Load);
          }
        }
      }
    }

    for (LoadInst *Load : LoadsToDemote) {
      errs().indent(2) << "Demoting Load: " << *Load << " with "
                       << Load->getNumUses() << " uses.\n";

      SmallVector<User *, 8> Users;
      SmallPtrSet<User *, 8> Seen;
      for (auto *U : Load->users()) {
        if (Seen.insert(U).second) {
          Users.push_back(U);
        }
      }

      std::sort(Users.begin(), Users.end(), [](User *A, User *B) {
        auto *InstA = cast<Instruction>(A);
        auto *InstB = cast<Instruction>(B);
        return InstA->comesBefore(InstB);
      });

      for (size_t I = 1; I < Users.size(); ++I) {
        auto *CurrentUser = Users[I];
        Instruction *CurrentUserInst = dyn_cast<Instruction>(CurrentUser);

        Instruction *NewLoad = Load->clone();

        if (Load->hasName())
          NewLoad->setName(Load->getName() + ".split." + Twine(I));

        NewLoad->insertAfter(CurrentUserInst->getPrevNode());

        unsigned IndexUse = 0;
        for (auto &Op : CurrentUserInst->operands()) {
          if (Op == Load)
            break;
          IndexUse++;
        }
        CurrentUser->setOperand(IndexUse, NewLoad);
      }
    }
  }
}
} // namespace

PreservedAnalyses ArrayReg2MemPass::run(Function &F,
                                        FunctionAnalysisManager &AM) {
  if (not F.hasFnAttribute("polly.findSCoP")) {
    return PreservedAnalyses::all();
  }

  LLVM_DEBUG(errs() << "ArrayReg2MemPass pass run on " << F.getName() << "\n");

  auto &LI = AM.getResult<LoopAnalysis>(F);
  auto &SE = AM.getResult<ScalarEvolutionAnalysis>(F);
  auto &DT = AM.getResult<DominatorTreeAnalysis>(F);
  demoteNonIndexPhis(F, LI, SE, DT);

  demoteUsesOfArrays(F, LI, SE, DT);

  if (verifyFunction(F, &errs())) {
    report_fatal_error("IR verification failed.");
  }

  LLVM_DEBUG(errs() << "ArrayReg2MemPass pass done\n");

  return PreservedAnalyses::none();
}
