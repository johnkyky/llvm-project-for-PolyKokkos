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
  Instruction *Term = FissionBlock->getTerminator();
  BasicBlock *Succ0 = Branch->getParent()->getTerminator()->getSuccessor(0);
  BasicBlock *Succ1 = Branch->getParent()->getTerminator()->getSuccessor(1);
  BasicBlock *FusionBlock = PDT.findNearestCommonDominator(Succ0, Succ1);

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

  auto IsBetweenBlocks = [&](BasicBlock *BB) {
    if (BB == FissionBlock || BB == FusionBlock)
      return true;

    bool DominatedByStart = DT.dominates(FissionBlock, BB);
    bool PostDominatedByEnd = PDT.dominates(FusionBlock, BB);
    return DominatedByStart && PostDominatedByEnd;
  };

  for (auto &UserInst : InstructionToTest) {
    errs() << "User instruction: " << *UserInst << "\n";
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

unsigned getMaxDepthRecursive(Loop *L) {
  unsigned CurrentDepth = L->getLoopDepth();
  unsigned MaxDepth = CurrentDepth;

  for (Loop *SubLoop : L->getSubLoops()) {
    unsigned SubDepth = getMaxDepthRecursive(SubLoop);
    MaxDepth = std::max(MaxDepth, SubDepth);
  }

  return MaxDepth;
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

Loop *getVersionedLoop(BasicBlock *Fision, BasicBlock *Fusion, LoopInfo &LI) {
  Instruction *Terminator = Fision->getTerminator();
  if (!Terminator)
    return nullptr;

  Loop *BaseLoop = LI.getLoopFor(Fision);
  unsigned BaseDepth = BaseLoop ? BaseLoop->getLoopDepth() : 0;

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
    } else if (BranchMaxDepth > BaseDepth && BranchMaxDepth == MaxFoundDepth) {
      llvm_unreachable("Multiple candidate loops with same depth found, "
                       "ambiguous versioned loop");
    }
  }

  return BestCandidate;
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

    auto TriangularLoops = getTriangularLoops(LI, SE, DT);
    if (auto *L = LI.getLoopFor(&BB)) {
      errs() << "BB is inside a loop: " << L->getName() << "\n";

      auto *EndVersioningBB = getMergeBlock(&BB, PDT);
      errs() << "EndVersioningBB: "
             << (EndVersioningBB ? EndVersioningBB->getName() : "null") << "\n";

      if (not EndVersioningBB) {
        errs() << "No merge block found, skipping\n";
        continue;
      }

      Loop *VersionedLoop = getVersionedLoop(&BB, EndVersioningBB, LI);
      if (not VersionedLoop) {
        errs() << "No versioned loop found, skipping\n";
        continue;
      }

      errs() << "Versioned loop: " << VersionedLoop->getName() << "\n ";
      auto SubLoops = getAllSubLoops(VersionedLoop);
      SubLoops.push_back(VersionedLoop);
      for (auto *SubL : SubLoops) {
        errs() << "\tSubLoop: " << SubL->getName() << "\n";
      }

      bool IsVariantInAllSubLoops = false;
      for (auto *SubL : SubLoops) {
        const SCEV *SubBackedgeCount = SE.getBackedgeTakenCount(SubL);
        errs() << "\tSubLoop Backedge SCEV: " << *SubBackedgeCount << "\n";
        if (SE.isLoopInvariant(SubBackedgeCount, L)) {
          errs() << "\tBackedge count invariant pour " << SubL->getName()
                 << "\n";
        } else {
          errs() << "\tBackedge count variant pour " << SubL->getName() << "\n";
          IsVariantInAllSubLoops = true;
        }
      }

      if (IsVariantInAllSubLoops) {
        errs() << "\tBackedge count variant on passe.\n";
        continue;
      }

      errs() << "Loop Invariant -> on la traite\n";
    }

    errs() << "Terminator: " << *Term << "\n";

    if (BranchInst *Branch = dyn_cast<BranchInst>(Term)) {
      Value *Condition = Branch->getCondition();
      if (auto *ICmp = dyn_cast<ICmpInst>(Condition)) {
        Value *LHS = ICmp->getOperand(0);
        Value *RHS = ICmp->getOperand(1);
        ICmpInst::Predicate Pred = ICmp->getPredicate();
        errs() << "Removing unswitched branch: " << *ICmp << "\n";
        errs() << "LHS: " << *LHS << "\n";
        errs() << "RHS: " << *RHS << "\n";
        errs() << "Predicate: " << Pred << "\n";

        switch (Pred) {
        case ICmpInst::ICMP_EQ: {
          errs() << "on sup Branch " << *Branch << "\n";
          errs() << "on branch to " << *Branch->getSuccessor(1) << "\n";
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
