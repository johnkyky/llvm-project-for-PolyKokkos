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

#include "polly/Test/MergeRedundantInvariantLoads.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/Analysis/AssumptionCache.h"
#include "llvm/Analysis/ScalarEvolution.h"
#include "llvm/Analysis/ScalarEvolutionExpressions.h"
#include "llvm/Analysis/ValueTracking.h"
#include "llvm/IR/Dominators.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/PassManager.h"
#include "llvm/IR/Verifier.h"
#include "llvm/Passes/PassBuilder.h"
#include "llvm/Support/raw_ostream.h"

#define DEBUG_TYPE "polly-merge-redundant-invariant-loads"

using namespace llvm;
using namespace polly;

namespace {

struct AvailableLoadTracker {
  llvm::DenseMap<Value *, LoadInst *> Table;

  std::vector<std::pair<Value *, LoadInst *>> History;

  size_t save() { return History.size(); }

  void restore(size_t SavePoint) {
    while (History.size() > SavePoint) {
      auto &Entry = History.back();
      Value *Ptr = Entry.first;
      LoadInst *OldLoad = Entry.second;

      if (OldLoad == nullptr)
        Table.erase(Ptr);
      else
        Table[Ptr] = OldLoad;

      History.pop_back();
    }
  }

  void add(LoadInst *Load) {
    Value *Ptr = Load->getPointerOperand();
    LoadInst *Old = nullptr;
    if (Table.count(Ptr)) {
      Old = Table[Ptr];
    }

    History.push_back({Ptr, Old});
    Table[Ptr] = Load;
  }

  void invalidate(Value *PtrToKill) {
    if (Table.count(PtrToKill)) {
      errs() << "  Invalidate load for pointer: " << *Table[PtrToKill] << "\n";
      History.push_back({PtrToKill, Table[PtrToKill]});
      Table.erase(PtrToKill);
    }
  }

  void invalidateAll() {
    for (auto &KV : Table) {
      History.push_back({KV.first, KV.second});
    }
    Table.clear();
  }

  LoadInst *find(Value *Ptr) {
    if (Table.count(Ptr))
      return Table[Ptr];
    return nullptr;
  }
};

struct SCEVNonZeroValidator : public SCEVVisitor<SCEVNonZeroValidator, bool> {
  ScalarEvolution &SE;
  Instruction *Context;

  SCEVNonZeroValidator(ScalarEvolution &SE, Instruction *Ctx)
      : SE(SE), Context(Ctx) {}

  bool isNonZero(const SCEV *S) { return visit(S); }

  bool visitConstant(const SCEVConstant *C) {
    return not C->getValue()->isZero();
  }

  bool visitAddRecExpr(const SCEVAddRecExpr *AR) {
    const SCEV *Start = AR->getStart();
    const SCEV *Step = AR->getStepRecurrence(SE);

    bool StartIsPositive = visit(Start) or SE.isKnownPositive(Start);
    bool StartIsNegative = SE.isKnownNegative(Start);

    if (not StartIsPositive and not StartIsNegative)
      return false;

    if (StartIsPositive and SE.isKnownNonNegative(Step))
      return true;
    if (StartIsNegative and SE.isKnownNonPositive(Step))
      return true;

    const Loop *L = AR->getLoop();
    const SCEV *BTC = SE.getBackedgeTakenCount(L);

    if (isa<SCEVCouldNotCompute>(BTC))
      return false;

    const SCEV *EndValue = AR->evaluateAtIteration(BTC, SE);

    if (StartIsPositive) {
      if (visit(EndValue) or SE.isKnownPositive(EndValue))
        return true;
    }

    if (StartIsNegative) {
      if (SE.isKnownNegative(EndValue))
        return true;
    }

    return false;
  }

  bool visitAddExpr(const SCEVAddExpr *Add) {
    bool AllNonNegative = true;
    bool OneStrictlyPositive = false;

    const SCEV *Zero = SE.getZero(Add->getType());

    for (const SCEV *Op : Add->operands()) {
      if (SE.isKnownPredicateAt(ICmpInst::ICMP_SLT, Add, Zero, Context)) {
        AllNonNegative = false;
        break;
      }

      if (visit(Op)) {
        OneStrictlyPositive = true;
      }

      else if (SE.isKnownPredicateAt(ICmpInst::ICMP_SGT, Add, Zero, Context)) {
        OneStrictlyPositive = true;
      }
    }

    return AllNonNegative && OneStrictlyPositive;
  }

  bool visitMulExpr(const SCEVMulExpr *Mul) {
    for (const SCEV *Op : Mul->operands()) {
      errs().indent(30) << "on va visiter " << *Op << " avec comme resultat "
                        << visit(Op) << "\n";
      if (not visit(Op) && not SE.isKnownNonZero(Op)) {
        return false;
      }
    }
    return true;
  }

  bool visitZeroExtendExpr(const SCEVZeroExtendExpr *ZE) {
    return visit(ZE->getOperand());
  }
  bool visitSignExtendExpr(const SCEVSignExtendExpr *SExt) {
    return visit(SExt->getOperand());
  }
  bool visitTruncateExpr(const SCEVTruncateExpr *TE) {
    return SE.isKnownNonZero(TE);
  }

  bool visitUnknown(const SCEVUnknown *U) {
    return SE.isKnownPredicateAt(ICmpInst::ICMP_NE, U, SE.getZero(U->getType()),
                                 Context);
  }

  bool visitUDivExpr(const SCEVUDivExpr *S) {
    const SCEV *Zero = SE.getZero(S->getType());
    return SE.isKnownPredicateAt(ICmpInst::ICMP_NE, S, Zero, Context);
  }
  bool visitVScale(const SCEVVScale *S) {
    const SCEV *Zero = SE.getZero(S->getType());
    return SE.isKnownPredicateAt(ICmpInst::ICMP_NE, S, Zero, Context);
  }
  bool visitSMaxExpr(const SCEVSMaxExpr *S) {
    const SCEV *Zero = SE.getZero(S->getType());
    return SE.isKnownPredicateAt(ICmpInst::ICMP_NE, S, Zero, Context);
  }
  bool visitUMaxExpr(const SCEVUMaxExpr *S) {
    const SCEV *Zero = SE.getZero(S->getType());
    return SE.isKnownPredicateAt(ICmpInst::ICMP_NE, S, Zero, Context);
  }
  bool visitSMinExpr(const SCEVSMinExpr *S) {
    const SCEV *Zero = SE.getZero(S->getType());
    return SE.isKnownPredicateAt(ICmpInst::ICMP_NE, S, Zero, Context);
  }
  bool visitUMinExpr(const SCEVUMinExpr *S) {
    const SCEV *Zero = SE.getZero(S->getType());
    return SE.isKnownPredicateAt(ICmpInst::ICMP_NE, S, Zero, Context);
  }
  bool visitSequentialUMinExpr(const SCEVSequentialUMinExpr *S) {
    const SCEV *Zero = SE.getZero(S->getType());
    return SE.isKnownPredicateAt(ICmpInst::ICMP_NE, S, Zero, Context);
  }
  bool visitPtrToIntExpr(const SCEVPtrToIntExpr *S) {
    return visit(S->getOperand());
  }

  bool visitCouldNotCompute(const SCEVCouldNotCompute *S) { return false; }
};

bool arePointersDistinct(Value *P1, Value *P2, ScalarEvolution &SE,
                         Instruction *Context) {
  if (P1 == P2) {
    errs().indent(20) << "Pointers are identical.\n";
    return false;
  }

  if (P1->getType() != P2->getType()) {
    errs().indent(8) << "Pointer types differ: " << *P1->getType() << " vs "
                     << *P2->getType() << "\n";
    return true;
  }

  if (!SE.isSCEVable(P1->getType()) || !SE.isSCEVable(P2->getType())) {
    errs().indent(20) << "One of the pointer types is not SCEVable.\n";
    if (!SE.isSCEVable(P1->getType()))
      errs().indent(22) << "Type1: " << *P1->getType() << "\n";
    else
      errs().indent(22) << "Type2: " << *P2->getType() << "\n";
    return false;
  }

  const SCEV *S1 = SE.getSCEV(P1);
  const SCEV *S2 = SE.getSCEV(P2);

  errs().indent(6) << "SCEV1: " << *S1 << "\n";
  errs().indent(6) << "SCEV2: " << *S2 << "\n";

  if (S1 == S2) {
    errs().indent(20) << "SCEV expressions are identical.\n";
    return false;
  }

  const SCEV *Delta = SE.getMinusSCEV(S1, S2);
  errs() << indent(6) << "Delta (S1 - S2): " << *Delta << "\n";

  if (isa<SCEVCouldNotCompute>(Delta)) {
    Value *Obj1 = getUnderlyingObject(P1);
    Value *Obj2 = getUnderlyingObject(P2);
    errs().indent(20) << "Could not compute SCEV delta.\n";
    errs().indent(20) << *Obj1 << " vs " << *Obj2 << "\n";

    if (Obj1 != Obj2)
      return true;
    return false;
  }

  SCEVNonZeroValidator Validator(SE, Context);
  if (Validator.visit(Delta)) {
    errs().indent(20) << "Delta is known non-zero by SCEVNonZeroValidator.\n";
    return true;
  }
  errs().indent(20) << "Delta is NOT known non-zero by SCEVNonZeroValidator.\n";
  return false;
}

bool arePointersEquivalent(Value *P1, Value *P2) {
  if (P1 == P2)
    return true;
  if (auto *I1 = dyn_cast<Instruction>(P1)) {
    if (auto *I2 = dyn_cast<Instruction>(P2)) {
      return I1->isIdenticalTo(I2);
    }
  }
  return false;
}

void runOnDomNode(DomTreeNode *Node, AvailableLoadTracker &Tracker,
                  ScalarEvolution &SE) {
  BasicBlock *BB = Node->getBlock();

  size_t SavePoint = Tracker.save();

  for (Instruction &I : llvm::make_early_inc_range(*BB)) {
    if (auto *CurrentLoad = dyn_cast<LoadInst>(&I)) {
      errs().indent(2) << "Examining Load: " << *CurrentLoad << "\n";
      if (!CurrentLoad->isSimple()) {
        errs().indent(4)
            << "  Non-simple load, invalidating all tracked loads.\n";
        Tracker.invalidateAll();
        continue;
      }

      Value *Ptr = CurrentLoad->getPointerOperand();
      LoadInst *Replacement = nullptr;

      if (LoadInst *Found = Tracker.find(Ptr)) {
        if (Found->getType() == CurrentLoad->getType()) {
          Replacement = Found;
        }
      } else {
        for (auto &KV : Tracker.Table) {
          if (arePointersEquivalent(KV.first, Ptr) &&
              KV.second->getType() == CurrentLoad->getType()) {
            Replacement = KV.second;
            break;
          }
        }
      }

      if (Replacement) {
        errs().indent(4) << "  Found redundant load, replacing with: "
                         << *Replacement << "\n";
        CurrentLoad->replaceAllUsesWith(Replacement);
        CurrentLoad->eraseFromParent();
      } else {
        Tracker.add(CurrentLoad);
      }
      continue;
    }

    if (auto *Store = dyn_cast<StoreInst>(&I)) {
      errs().indent(2) << "\n\nExamining Store: " << *Store << "\n";
      Value *StorePtr = Store->getPointerOperand();

      SmallVector<Value *, 4> PtrsToInvalidate;

      for (auto &[LoadAdrr, Load] : Tracker.Table) {
        errs() << "  Checking against tracked load pointer: " << *Load << "   "
               << *LoadAdrr << "\n";

        if (arePointersDistinct(StorePtr, LoadAdrr, SE, Store)) {
          errs().indent(4) << "-> " << *Load << "\n";
          continue;
        }

        errs().indent(4)
            << "  Pointers are NOT distinct, will invalidate load for pointer: "
            << *Load << "\n";
        PtrsToInvalidate.push_back(LoadAdrr);
      }

      for (Value *Ptr : PtrsToInvalidate) {
        Tracker.invalidate(Ptr);
      }
      errs() << "\n\n";

      continue;
    }

    if (auto *Call = dyn_cast<CallInst>(&I)) {
      if (Function *Callee = Call->getCalledFunction()) {
        StringRef Name = Callee->getName();

        if (Name.starts_with("llvm.annotation") ||
            Name.starts_with("llvm.assume")) {
          continue;
        }
      }
      if (not Call->onlyReadsMemory() && not Call->doesNotAccessMemory()) {
        errs() << "INVALIDATE ALL due to call: " << *Call << "\n";
        Tracker.invalidateAll();
      }
    }
  }

  for (auto *Child : Node->children()) {
    runOnDomNode(Child, Tracker, SE);
  }

  Tracker.restore(SavePoint);
}

void globalLoadElimination(Function &F, DominatorTree &DT,
                           ScalarEvolution &SE) {
  AvailableLoadTracker Tracker;

  if (DomTreeNode *Root = DT.getRootNode()) {
    runOnDomNode(Root, Tracker, SE);
  }
}
} // namespace

PreservedAnalyses
MergeRedundantInvariantLoadsPass::run(Function &F,
                                      FunctionAnalysisManager &AM) {
  if (not F.hasFnAttribute("polly.findSCoP"))
    return PreservedAnalyses::all();

  LLVM_DEBUG(errs() << "\nMergeRedundantInvariantLoadsPass pass run on "
                    << F.getName() << "\n";);

  AM.clear();

  auto &SE = AM.getResult<ScalarEvolutionAnalysis>(F);

  auto &AC = AM.getResult<AssumptionAnalysis>(F);
  errs() << AC.assumptions().size() << " assumptions found:\n";
  for (auto &Assumtion : AC.assumptions()) {
    LLVM_DEBUG(errs() << "Assumption: " << *Assumtion << "\n";);
  }

  auto &DT = AM.getResult<DominatorTreeAnalysis>(F);
  globalLoadElimination(F, DT, SE);

  if (verifyFunction(F, &errs())) {
    report_fatal_error("IR verification failed.");
  }

  LLVM_DEBUG(errs() << "MergeRedundantInvariantLoadsPass pass done\n";);

  return PreservedAnalyses::none();
}
