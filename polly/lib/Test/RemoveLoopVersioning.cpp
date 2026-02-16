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
#include "polly/Test/TriangularLoopFix.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/Analysis/AssumptionCache.h"
#include "llvm/Analysis/LoopInfo.h"
#include "llvm/Analysis/PostDominators.h"
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
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"

#define DEBUG_TYPE "polly-remove-loop-versioning"

using namespace llvm;
using namespace polly;

namespace {

bool isLoopBoundCondition(BranchInst *Branch, Value *Val, LoopInfo &LI,
                          DominatorTree &DT, PostDominatorTree &PDT) {
  auto *ValInst = dyn_cast<Instruction>(Val);
  auto *FissionBlock = ValInst->getParent();
  BasicBlock *Succ0 = Branch->getParent()->getTerminator()->getSuccessor(0);
  BasicBlock *Succ1 = Branch->getParent()->getTerminator()->getSuccessor(1);
  BasicBlock *FusionBlock = PDT.findNearestCommonDominator(Succ0, Succ1);

  errs() << "Fission block: " << FissionBlock->getName() << "\n";
  errs() << "Fusion block: " << FusionBlock->getName() << "\n";
  errs() << "Successor 0: " << Succ0->getName() << "\n";
  errs() << "Successor 1: " << Succ1->getName() << "\n";
  errs() << "Value instruction: " << *ValInst << "\n";

  if (Succ0 == FusionBlock || Succ1 == FusionBlock) {
    errs() << "Fusion block is one of the successors, skipping\n";
    return false;
  }

  llvm::SmallVector<Instruction *, 4> InstructionToTest;

  if (auto *CallInst = dyn_cast<llvm::CallInst>(Val)) {
    const Function *Callee = CallInst->getCalledFunction();
    if (Callee and Callee->getName().starts_with("llvm.annotation")) {
      auto *Val = CallInst->getArgOperand(0);
      for (User *U : Val->users()) {
        InstructionToTest.push_back(dyn_cast<Instruction>(U));
        for (auto *UU : U->users()) {
          Instruction *UserUserBinOp = dyn_cast<ICmpInst>(UU);
          if (not UserUserBinOp)
            continue;
          InstructionToTest.push_back(UserUserBinOp);
        }
      }
    }
  }

  for (auto &U : Val->uses()) {
    Instruction *UserInst = dyn_cast<Instruction>(U.getUser());
    if (not UserInst)
      continue;
    errs() << "User of value: " << *UserInst << "\n";

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

  for (auto &Op : ValInst->operands()) {
    Instruction *OpInst = dyn_cast<Instruction>(Op);
    if (not OpInst)
      continue;
    errs() << "Operand instruction: " << *OpInst << "\n";
    if (OpInst->hasMetadata("variable_annotation")) {
      errs() << "  [FOUND] Cette instruction est une condition "
             << "de sortie de boucle par variable_annotation !\n";
      return true;
    }
  }

  auto IsBetweenBlocks = [&](BasicBlock *BB) {
    if (BB == FissionBlock || BB == FusionBlock)
      return true;

    bool DominatedByStart = DT.dominates(FissionBlock, BB);
    bool PostDominatedByEnd = PDT.dominates(FusionBlock, BB);
    return DominatedByStart && PostDominatedByEnd;
  };

  for (auto &UserInst : InstructionToTest) {
    errs() << "InstructionToTest  instruction: " << *UserInst << "\n";
    if (not UserInst)
      continue;

    if (not IsBetweenBlocks(UserInst->getParent()))
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
          errs() << "  [FOUND] Cette instruction est une condition "
                 << "de sortie de boucle !\n";
          return true;
        }
      }
    }
  }
  return false;
}

BasicBlock *getMergeBlock(BasicBlock *SplitBlock, PostDominatorTree &PDT) {
  DomTreeNode *Node = PDT.getNode(SplitBlock);

  if (Node and Node->getIDom()) {
    return Node->getIDom()->getBlock();
  }

  return nullptr;
}

std::vector<Loop *> getAllSubLoops(Loop *L) {
  std::vector<Loop *> SubLoops;
  for (Loop *SubLoop : L->getSubLoops()) {
    SubLoops.push_back(SubLoop);
    auto NestedSubLoops = getAllSubLoops(SubLoop);
    SubLoops.insert(SubLoops.end(), NestedSubLoops.begin(),
                    NestedSubLoops.end());
  }
  return SubLoops;
}

std::pair<Loop *, BasicBlock *>
getVersionedLoop(BasicBlock *Fision, BasicBlock *Fusion, LoopInfo &LI) {
  Instruction *Terminator = Fision->getTerminator();
  if (!Terminator)
    return {nullptr, nullptr};

  Loop *BaseLoop = LI.getLoopFor(Fision);
  unsigned BaseDepth = BaseLoop ? BaseLoop->getLoopDepth() : 0;

  BasicBlock *BestCandidateBB = nullptr;
  Loop *BestCandidate = nullptr;
  unsigned MaxFoundDepth = BaseDepth;

  for (unsigned I = 0; I < Terminator->getNumSuccessors(); ++I) {
    BasicBlock *StartBB = Terminator->getSuccessor(I);

    if (StartBB == Fusion)
      continue;

    unsigned BranchMaxDepth = 0;
    Loop *BranchDeepestLoop = nullptr;

    SmallVector<BasicBlock *, 16> Worklist;
    SmallPtrSet<BasicBlock *, 32> Visited;

    Worklist.push_back(StartBB);
    Visited.insert(StartBB);

    while (!Worklist.empty()) {
      BasicBlock *BB = Worklist.pop_back_val();

      Loop *L = LI.getLoopFor(BB);
      if (L) {
        unsigned D = L->getLoopDepth();
        if (D > BranchMaxDepth) {
          BranchMaxDepth = D;
          BranchDeepestLoop = L;
        }
      }

      for (BasicBlock *Succ : successors(BB)) {
        if (Succ != Fusion && Visited.insert(Succ).second) {
          Worklist.push_back(Succ);
        }
      }
    }

    errs() << "\tBranch starting at " << StartBB->getName()
           << " has max depth: " << BranchMaxDepth << "\n";

    if (BranchMaxDepth > BaseDepth && BranchMaxDepth > MaxFoundDepth) {
      MaxFoundDepth = BranchMaxDepth;
      BestCandidate = BranchDeepestLoop;
      BestCandidateBB = StartBB;
    } else if (BranchMaxDepth > BaseDepth && BranchMaxDepth == MaxFoundDepth) {
      llvm_unreachable("Multiple candidate loops with same depth found, "
                       "ambiguous versioned loop");
    }
  }

  return {BestCandidate, BestCandidateBB};
}

bool isValidBranchInst(Instruction *Inst, LoopInfo &LI, DominatorTree &DT,
                       PostDominatorTree &PDT) {
  BranchInst *Branch = dyn_cast<BranchInst>(Inst);
  if (not Branch)
    return false;
  if (!Branch->isConditional() || Branch->getNumSuccessors() != 2) {
    errs() << "Skipping non-conditional or non-binary branch\n";
    return false;
  }
  Value *Condition = Branch->getCondition();
  if (auto *ICmp = dyn_cast<ICmpInst>(Condition)) {
    Value *LHS = ICmp->getOperand(0);
    Value *RHS = ICmp->getOperand(1);
    ICmpInst::Predicate Pred = ICmp->getPredicate();
    errs() << "LHS: " << *LHS << "\n";
    errs() << "RHS: " << *RHS << "\n";
    errs() << "Predicate: " << Pred << "\n";

    ConstantInt *ConstOp = nullptr;
    Value *VariableOp = nullptr;

    if ((ConstOp = dyn_cast<ConstantInt>(RHS))) {
      VariableOp = LHS;
    } else if ((ConstOp = dyn_cast<ConstantInt>(LHS))) {
      VariableOp = RHS;
    }

    if (not VariableOp or not ConstOp) {
      errs() << "No constant operand found, skipping\n";
      return false;
    }

    if (auto *VariableInst = dyn_cast<Instruction>(VariableOp)) {
      if (VariableInst->hasMetadata("variable_annotation")) {
        errs()
            << "Variable operand has variable_annotation metadata, treating as "
            << "loop bound condition\n";
        return true;
      }
    }

    if (not isLoopBoundCondition(Branch, VariableOp, LI, DT, PDT)) {
      errs() << "Variable operand is not from a loop bound, skipping\n";
      return false;
    }
  }
  return true;
}

void removeUnswitchedBranches(Function &F, LoopInfo &LI, PostDominatorTree &PDT,
                              DominatorTree &DT, ScalarEvolution &SE) {
  for (BasicBlock &BB : F) {
    errs() << "\n\nVisiting block: " << BB.getName() << "\n";

    // check if block have itself in its successors (loop backedge)
    if (std::find(succ_begin(&BB), succ_end(&BB), &BB) != succ_end(&BB)) {
      errs() << "Skipping block with self-loop\n";
      continue;
    }

    Instruction *Term = BB.getTerminator();
    if (not isValidBranchInst(Term, LI, DT, PDT))
      continue;

    auto *EndVersioningBB = getMergeBlock(&BB, PDT);
    if (not EndVersioningBB)
      llvm_unreachable("No merge block found for versioning, cannot proceed");

    bool FoundMergeBlockAsSuccessor = false;
    for (auto *SuccBB : successors(&BB)) {
      if (EndVersioningBB == SuccBB) {
        FoundMergeBlockAsSuccessor = true;
        continue;
      }
    }
    if (FoundMergeBlockAsSuccessor) {
      errs() << "Merge block is a direct successor, skipping\n";
      continue;
    }

    errs() << "EndVersioningBB: "
           << (EndVersioningBB ? EndVersioningBB->getName() : "null") << "\n";

    auto Versionned = getVersionedLoop(&BB, EndVersioningBB, LI);
    BasicBlock *VersionnedBB = Versionned.second;
    errs() << "VersionnedBB: "
           << (VersionnedBB ? VersionnedBB->getName() : "null") << "\n";

    // TODO use getTriangularLoops to check if the loop is triangular, and skip
    auto TriangularLoops = getTriangularLoops(LI, SE, DT);
    errs() << "Triangular loops in the function: " << TriangularLoops.size()
           << "\n";
    for (auto *TriangularLoop : TriangularLoops) {
      errs() << "Triangular loop: " << TriangularLoop->getName() << "\n";
    }
    if (auto *L = LI.getLoopFor(&BB)) {
      errs() << "BB is inside a loop: " << L->getName() << "\n";

      Loop *VersionedLoop = Versionned.first;
      if (not VersionedLoop) {
        errs() << "No versioned loop found, skipping\n";
        continue;
      }

      auto SubLoops = getAllSubLoops(VersionedLoop);
      SubLoops.push_back(VersionedLoop);

      bool IsVariantInAllSubLoops = false;
      for (auto *SubL : SubLoops) {
        errs() << "\tSubLoop: " << SubL->getName() << "\n";
        if (std::find(TriangularLoops.begin(), TriangularLoops.end(), SubL) !=
            TriangularLoops.end()) {
          IsVariantInAllSubLoops = true;
          break;
        }
      }
      if (IsVariantInAllSubLoops) {
        errs() << "\tLoop is triangular, skipping\n";
        continue;
      }

      errs() << "Loop Invariant -> on la traite\n";
    }

    errs() << "Terminator: " << *Term << "\n";

    if (BranchInst *Branch = dyn_cast<BranchInst>(Term)) {
      IRBuilder<> Builder(Branch);
      Value *Condition = nullptr;
      if (Branch->getSuccessor(0) == VersionnedBB) {
        Condition = Builder.getTrue();
      } else if (Branch->getSuccessor(1) == VersionnedBB) {
        Condition = Builder.getFalse();
      } else {
        llvm_unreachable("VersionnedBB is not a successor of the branch");
      }

      auto *ICmp = dyn_cast<ICmpInst>(Branch->getCondition());
      ICmp->setMetadata("used_for_versioning",
                        MDNode::get(ICmp->getContext(), {}));

      ICmp->setPredicate(CmpInst::getInversePredicate(ICmp->getPredicate()));
      Value *Assume = Builder.CreateAssumption(ICmp);
      LLVM_DEBUG(errs() << "Registering assumption: " << *Assume << "\n");
      LLVM_DEBUG(errs() << "Removing branch to " << *Branch << "\n";);

      Branch->setCondition(Condition);
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
  PostDominatorTree &PDT = AM.getResult<PostDominatorTreeAnalysis>(F);
  DominatorTree &DT = AM.getResult<DominatorTreeAnalysis>(F);
  ScalarEvolution &SE = AM.getResult<ScalarEvolutionAnalysis>(F);
  removeUnswitchedBranches(F, LI, PDT, DT, SE);

  if (verifyFunction(F, &errs())) {
    report_fatal_error("IR verification failed.");
  }

  LLVM_DEBUG(errs() << "RemoveLoopVersioningPass pass done\n";);

  return PreservedAnalyses::none();
}
