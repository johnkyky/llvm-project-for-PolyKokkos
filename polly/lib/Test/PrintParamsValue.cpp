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

#include "polly/Test/PrintParamsValue.h"
#include "polly/CodeGen/RuntimeDebugBuilder.h"
#include "llvm/Analysis/LoopInfo.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/Instruction.h"
#include "llvm/Support/Debug.h"
#include <deque>

#define DEBUG_TYPE "polly-runtime-print-params"

using namespace llvm;
using namespace polly;

namespace {
SmallVector<BasicBlock *, 4> getBlocksBeforeLoop(Function &F, LoopInfo &LI) {
  std::deque<BasicBlock *> Queue;
  std::set<BasicBlock *> Visited;

  BasicBlock *EntryBB = &F.getEntryBlock();

  SmallVector<BasicBlock *, 4> Result;

  if (LI.getLoopFor(EntryBB)) {
    return Result;
  }

  Queue.push_back(EntryBB);
  Visited.insert(EntryBB);

  while (!Queue.empty()) {
    BasicBlock *CurrentBB = Queue.front();
    Queue.pop_front();

    Result.push_back(CurrentBB);

    for (BasicBlock *Succ : successors(CurrentBB)) {
      if (Visited.count(Succ))
        continue;

      if (LI.getLoopFor(Succ) != nullptr) {
        continue;
      }

      Visited.insert(Succ);
      Queue.push_back(Succ);
    }
  }
  return Result;
}

SmallVector<Instruction *, 8>
collectIntegerInstructions(const SmallVectorImpl<BasicBlock *> &Blocks) {
  SmallVector<Instruction *, 8> Result;

  for (BasicBlock *BB : Blocks) {
    if (!BB)
      continue;

    for (Instruction &I : *BB) {
      if (I.getType()->isIntegerTy()) {
        LLVM_DEBUG(errs() << "  -> Found integer instruction.\n";);
        Result.push_back(&I);
      }
    }
  }

  return Result;
}

std::string getValueName(llvm::Value *V) {
  std::string Str;
  llvm::raw_string_ostream OS(Str);

  V->printAsOperand(OS, false);

  return OS.str();
}
} // namespace

PreservedAnalyses
polly::PrintParamsValuePass::run(Function &F, FunctionAnalysisManager &AM) {
  if (not F.hasFnAttribute("polly.findSCoP"))
    return PreservedAnalyses::all();

  LLVM_DEBUG(errs() << "FunctionPassTest : " << F.getName().str() << "\n";);

  ScopAnnotator Annotator;

  BasicBlock &EnteringBB = F.getEntryBlock();
  PollyIRBuilder Builder(EnteringBB.getContext(), ConstantFolder(),
                         IRInserter(Annotator));

  auto &LI = AM.getResult<LoopAnalysis>(F);
  auto Blocks = getBlocksBeforeLoop(F, LI);

  for (auto *BB : Blocks)
    LLVM_DEBUG(errs() << "Block before loop: " << BB->getName() << "\n";);

  auto IntInst = collectIntegerInstructions(Blocks);

  for (auto *V : IntInst) {
    LLVM_DEBUG(errs() << "Integer Instruction: " << *V << "\n";);
    Builder.SetInsertPoint(V->getParent()->getTerminator());
    auto Name = getValueName(V);
    RuntimeDebugBuilder::createCPUPrinter(Builder, Name, " = ", V, "\n");
  }

  LLVM_DEBUG(errs() << "FunctionPassTest pass done\n";);

  return PreservedAnalyses::all();
}
