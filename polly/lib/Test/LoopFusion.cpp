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

#include "polly/Test/LoopFusion.h"
#include "polly/Test/ExtractAnnotatedFromLoop.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/Analysis/AssumptionCache.h"
#include "llvm/Analysis/LoopInfo.h"
#include "llvm/IR/CFG.h"
#include "llvm/IR/Dominators.h"
#include "llvm/IR/Instruction.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/PassManager.h"
#include "llvm/IR/Verifier.h"
#include "llvm/Passes/PassBuilder.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include <algorithm>
#include <utility>
#include <vector>

using namespace llvm;
using namespace polly;

AnalysisKey LoopBoundAnalysis::Key;

BasicBlock *LoopBoundAnalysis::getExitBlock(Function &F) {
  SmallVector<BasicBlock *, 1> ExitBlocks;
  for (auto &BB : F) {
    auto *Term = BB.getTerminator();
    if (isa<ReturnInst>(Term))
      return &BB;
  }

  if (ExitBlocks.size() != 1)
    llvm_unreachable("Function does not have a single exit block");
  return ExitBlocks[0];
}

SmallVector<LoopBoundT, 4>
LoopBoundAnalysis::getLoopBoundInstructions(Function &F, DominatorTree &DT) {
  SmallVector<LoopBoundT, 4> LoopBoundVec;

  // Read loop bound information from metadata
  for (auto &BB : F) {
    for (auto &Inst : BB) {
      auto *Node = Inst.getMetadata("loop_bound_information");
      if (Node) {
        LoopBoundT LB;
        LB.Inst = &Inst;

        if (Node->getNumOperands() >= 3) {
          if (llvm::ConstantAsMetadata *CAM =
                  llvm::dyn_cast<llvm::ConstantAsMetadata>(
                      Node->getOperand(0))) {
            if (llvm::ConstantInt *CI =
                    llvm::dyn_cast<llvm::ConstantInt>(CAM->getValue())) {
              LB.IndexPolicy = CI->getZExtValue();
            }
          }

          if (llvm::MDString *KindMetadata =
                  llvm::dyn_cast<llvm::MDString>(Node->getOperand(1))) {
            auto BoundTypeStr = KindMetadata->getString();
            if (BoundTypeStr == "lower")
              LB.BoundType = LoopBoundT::Lower;
            else if (BoundTypeStr == "upper")
              LB.BoundType = LoopBoundT::Upper;
            else
              errs() << "Unknown loop bound kind: " << BoundTypeStr << "\n";
          }

          if (llvm::ConstantAsMetadata *CAM =
                  llvm::dyn_cast<llvm::ConstantAsMetadata>(
                      Node->getOperand(2))) {
            if (llvm::ConstantInt *CI =
                    llvm::dyn_cast<llvm::ConstantInt>(CAM->getValue())) {
              LB.Depth = CI->getZExtValue();
            }
          }
        }
        LoopBoundVec.push_back(std::move(LB));
      }
    }
  }

  return LoopBoundVec;
}

const LoopBoundAnalysis::Result
LoopBoundAnalysis::run(Function &F, FunctionAnalysisManager &AM) {
  auto &DT = AM.getResult<DominatorTreeAnalysis>(F);
  return getLoopBoundInstructions(F, DT);
}

size_t LoopBoundAnalysis::getNumPolicies(LoopBoundAnalysis::Result &R) {
  if (R.empty())
    return 0;
  return R.back().IndexPolicy + 1;
}

size_t LoopBoundAnalysis::getNumDimForPolicy(LoopBoundAnalysis::Result &R,
                                             size_t PolicyIdx) {
  size_t MaxDim = 1;
  for (auto &LB : R) {
    if (LB.IndexPolicy == PolicyIdx && LB.Depth + 1 > MaxDim) {
      MaxDim = LB.Depth + 1;
    }
  }
  return MaxDim;
}

raw_ostream &polly::operator<<(raw_ostream &OS,
                               const SmallVector<LoopBoundT, 4> &LBA) {
  for (const auto LoopBound : LBA) {
    errs() << "Loop bound : " << "policy" << LoopBound.IndexPolicy << "."
           << (LoopBound.BoundType == LoopBoundT::Lower ? "lower" : "upper")
           << LoopBound.Depth << " -> " << *LoopBound.Inst << "\n";
  }
  return OS;
}

namespace {

void implMoveBlock(Loop *L1, Loop *L2, BasicBlock *BlockToMove,
                   BasicBlock *InsertBeforeBlock) {
  auto ValidBlock = [&L1, &L2](auto Blocks) -> BasicBlock * {
    // Check if blocks have multi predecessor/successor blocks and chose the
    // outside loop block
    if (std::distance(Blocks.begin(), Blocks.end()) == 1) {
      return *Blocks.begin();
    }
    for (auto *Block : Blocks) {
      if (L1->contains(Block) || L2->contains(Block))
        continue;
      return Block;
    }
    return nullptr;
  };

  // errs() << "Moving block: " << *BlockToMove << "\n";
  // for (auto *Pred : llvm::predecessors(BlockToMove)) {
  //   errs() << "  Predecessor: " << Pred->getName() << "\n";
  // }
  BasicBlock *SrcPredBlock = ValidBlock(llvm::predecessors(BlockToMove));
  // errs() << "Source predecessor: " << *SrcPredBlock << "\n";
  BasicBlock *SrcNextBlock = ValidBlock(llvm::successors(BlockToMove));
  // errs() << "Source next: " << *SrcNextBlock << "\n";

  BasicBlock *DstPredBlock = ValidBlock(llvm::predecessors(InsertBeforeBlock));
  // errs() << "Destination predecessor: " << *DstPredBlock << "\n";
  BasicBlock *DstNextBlock = InsertBeforeBlock;
  // errs() << "Destination next: " << *DstNextBlock << "\n";

  // return;
  // DST
  DstPredBlock->getTerminator()->setSuccessor(0, BlockToMove);

  BlockToMove->getTerminator()->setSuccessor(0, DstNextBlock);

  for (Instruction &I : *DstNextBlock)
    if (auto *Phi = dyn_cast<PHINode>(&I))
      Phi->replaceIncomingBlockWith(DstPredBlock, BlockToMove);

  // SRC
  auto *InstBranch = SrcPredBlock->getTerminator();
  auto *Branch = dyn_cast<BranchInst>(InstBranch);
  bool IsConditional = Branch && Branch->isConditional();
  if (not IsConditional) {
    SrcPredBlock->getTerminator()->setSuccessor(0, SrcNextBlock);
  } else {
    BasicBlock *TrueBlock = Branch->getSuccessor(0);
    BasicBlock *FalseBlock = Branch->getSuccessor(1);

    if (not L1->contains(TrueBlock)) {
      Branch->setSuccessor(0, SrcNextBlock);
    } else if (L2->contains(FalseBlock)) {
      Branch->setSuccessor(1, SrcNextBlock);
    }
  }

  for (Instruction &I : *SrcNextBlock)
    if (auto *Phi = dyn_cast<PHINode>(&I))
      Phi->replaceIncomingBlockWith(BlockToMove, SrcPredBlock);
}

void moveBlockBetweenLoopsImpl(Loop *L1, Loop *L2) {
  BasicBlock *Start = L1->getExitBlock();
  BasicBlock *End = L2->getHeader();
  BasicBlock *InsertionPoint = L1->getHeader();
  std::vector<BasicBlock *> ToMove;

  // errs() << "Start: " << *Start << "\n";
  // errs() << "End: " << *End << "\n";
  // errs() << "InsertionPoint: " << *InsertionPoint << "\n";

  auto Size = [](llvm::succ_range R) {
    return std::distance(R.begin(), R.end());
  };

  auto Succ = llvm::successors(Start);
  ToMove.push_back(Start);

  if (Size(Succ) != 1)
    llvm_unreachable("Start block has multiple successors");

  while (true) {
    auto *Current = *Succ.begin();
    if (Current == End)
      break;

    ToMove.push_back(Current);
    Succ = llvm::successors(Current);

    if (size(Succ) != 1)
      llvm_unreachable("Intermediate block has multiple successors");
  }

  // for (auto *BB : ToMove) {
  //   errs() << "Block to move: " << *BB << "\n\n";
  // }

  for (auto *BB : ToMove) {
    if (L1->contains(BB) || L2->contains(BB))
      llvm_unreachable("Block to move is inside one of the loops");
    implMoveBlock(L1, L2, BB, InsertionPoint);
  }
}

bool moveBlockBetweenLoops(const SmallVector<Loop *, 2> &Loops) {
  if (Loops.size() < 2)
    return false;

  for (size_t I = Loops.size() - 1; I >= 1; I--) {
    Loop *L1 = Loops[I - 1];
    Loop *L2 = Loops[I];
    moveBlockBetweenLoopsImpl(L1, L2);
  }
  return true;
}

} // namespace

PreservedAnalyses LoopFusionPass::run(Function &F,
                                      FunctionAnalysisManager &AM) {
  if (not F.hasFnAttribute("polly.multi_parallel_for"))
    return PreservedAnalyses::all();

  errs() << "LoopFusionPass pass run on " << F.getName() << "\n";

  auto &LI = AM.getResult<LoopAnalysis>(F);
  auto &DT = AM.getResult<DominatorTreeAnalysis>(F);
  auto Loops = findLoop(F, LI, DT);
  moveBlockBetweenLoops(Loops);

  if (verifyFunction(F, &errs())) {
    report_fatal_error("IR verification failed.");
  }

  errs() << "LoopFusionPass pass done\n";

  return PreservedAnalyses::none();
}
