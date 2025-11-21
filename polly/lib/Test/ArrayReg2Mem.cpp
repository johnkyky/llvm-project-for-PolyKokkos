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
#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/Instruction.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/PassManager.h"
#include "llvm/IR/Value.h"
#include "llvm/Passes/PassBuilder.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Transforms/Utils/Local.h"
#include <array>

#define DEBUG_TYPE "polly-array-reg2mem"

using namespace llvm;
using namespace polly;

namespace {
Value *checkPhiStoreRelation(PHINode *Phi) {
  errs() << "\nChecking PHI store relation " << *Phi << "\n";
  if (Phi->getNumIncomingValues() > 2) {
    return nullptr;
  }

  errs() << "Number of incoming values: " << Phi->getNumIncomingValues()
         << "\n";
  if (Phi->getNumIncomingValues() == 1) {

    Value *Val = Phi->getIncomingValue(0);
    BasicBlock *Block = Phi->getIncomingBlock(0);

    for (auto *U : Val->users()) {
      Instruction *Inst = llvm::dyn_cast<Instruction>(U);
      if (Inst->getParent() != Block)
        continue;

      errs() << "   On check le use " << *Inst << "\n";
      if (auto *Store = llvm::dyn_cast<llvm::StoreInst>(Inst)) {
        errs() << "    Found store: " << *Store << "\n";
        return Store->getPointerOperand();
      }
    }

    return nullptr;
  }

  Value *FirstVal = Phi->getIncomingValue(0);
  BasicBlock *FirstBlock = Phi->getIncomingBlock(0);
  Value *FirstStorePtr = nullptr;

  Value *SecondVal = Phi->getIncomingValue(1);
  BasicBlock *SecondBlock = Phi->getIncomingBlock(1);

  if (isa<Constant>(FirstVal)) {
    errs() << "First value is constant\n";
    for (auto *U : SecondVal->users()) {
      Instruction *Inst = dyn_cast<Instruction>(U);
      if (Inst->getParent() != SecondBlock)
        continue;

      errs() << "   On check le second use " << *Inst << "\n";
      if (auto *Store = llvm::dyn_cast<llvm::StoreInst>(Inst)) {
        errs() << "    Found store: " << *Store << "\n";
        return Store->getPointerOperand();
      }
    }
    return SecondVal;
  }
  if (isa<Constant>(SecondVal)) {
    errs() << "Second value is constant\n";
    return FirstVal;
  }

  // First
  for (auto *U : FirstVal->users()) {
    Instruction *Inst = llvm::dyn_cast<Instruction>(U);
    if (Inst->getParent() != FirstBlock)
      continue;
    errs() << "   On check le first use " << *Inst << "\n";
    if (auto *Store = llvm::dyn_cast<llvm::StoreInst>(Inst)) {
      errs() << "    Found store: " << *Store << "\n";
      FirstStorePtr = Store->getPointerOperand();
    }
  }

  // Second
  for (auto *U : SecondVal->users()) {
    Instruction *Inst = dyn_cast<Instruction>(U);
    if (Inst->getParent() != SecondBlock)
      continue;

    errs() << "   On check le second use " << *Inst << "\n";
    if (auto *Store = llvm::dyn_cast<llvm::StoreInst>(Inst)) {
      errs() << "    Found store: " << *Store << "\n";
      if (Store->getPointerOperand() == FirstStorePtr) {
        errs() << "    Matching store pointer found: " << *FirstStorePtr
               << "\n";
        return FirstStorePtr;
      }
    }
  }
  return nullptr;
}

void demoteNonIndexPhis(llvm::Function &F, llvm::LoopInfo &LI,
                        llvm::ScalarEvolution &SE) {

  std::vector<llvm::PHINode *> PhisToDemote;
  for (auto &BB : F) {
    llvm::Loop *L = LI.getLoopFor(&BB);

    for (auto &I : BB) {
      auto *Phi = llvm::dyn_cast<llvm::PHINode>(&I);
      if (!Phi)
        continue;

      errs() << "Examining PHI: " << *Phi << "\n";
      errs() << "  In Loop: " << (L ? L->getHeader()->getName() : "None")
             << "\n";

      if (not SE.isSCEVable(Phi->getType())) {
        errs() << "  Not SCEVable. Demoting.\n";
        PhisToDemote.push_back(Phi);
        continue;
      }

      const auto *S = SE.getSCEV(Phi);
      errs() << "  SCEV: " << *S << "\n";
      if (auto *AR = llvm::dyn_cast<llvm::SCEVAddRecExpr>(S)) {
        if (AR->getLoop() == L) {
          errs() << "  AddRec is for the current loop. Not demoting.\n";
          continue;
        }
      }

      PhisToDemote.push_back(Phi);
    }
  }

  for (auto *Phi : PhisToDemote) {
    if (auto *Val = checkPhiStoreRelation(Phi)) {
      errs() << "Demoting PHI: " << *Phi << "\n";

      IRBuilder<> Builder(Phi);
      LoadInst *Load = Builder.CreateLoad(Phi->getType(), Val, "demoted_phi");
      Phi->replaceAllUsesWith(Load);
      Phi->eraseFromParent();
      errs() << "  Demoted PHI to load from " << *Val << "\n";
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
  demoteNonIndexPhis(F, LI, SE);

  LLVM_DEBUG(errs() << "ArrayReg2MemPass pass done\n");

  return PreservedAnalyses::none();
}
