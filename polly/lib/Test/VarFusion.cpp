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

#include "polly/Test/VarFusion.h"
#include "polly/Test/UserAssumptions.h"
#include "llvm/ADT/SetVector.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/IR/Dominators.h"
#include "llvm/IR/Instruction.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/Metadata.h"
#include "llvm/IR/Verifier.h"
#include "llvm/Support/ErrorHandling.h"

#define DEBUG_TYPE "polly-var-fusion"

using namespace llvm;
using namespace polly;

namespace {

void fusionVarAnnotations(Function &F, DominatorTree &DT) {
  auto Annotations = findVarInstructions(F);

  for (auto &[V, Name] : Annotations) {
    errs() << "Final Variable annotation: " << *V << " -> " << Name << "\n";
  }

  DenseMap<StringRef, std::vector<Instruction *>> Groups;
  for (auto &[V, Name] : Annotations) {

    if (auto *Inst = dyn_cast<Instruction>(V))
      Groups[Name].push_back(Inst);
  }

  for (auto &Entry : Groups) {
    StringRef Name = Entry.first;
    std::vector<Instruction *> &Insts = Entry.second;

    if (Insts.size() <= 1)
      continue;

    std::sort(Insts.begin(), Insts.end(), [&](Instruction *A, Instruction *B) {
      if (A->getParent() == B->getParent()) {
        return A->comesBefore(B);
      }
      return DT.dominates(A->getParent(), B->getParent());
    });

    // if no valid dominating order we move the first instruction to the common
    Instruction *FirstInst = Insts[0];
    bool IsValid = true;
    for (size_t I = 1; I < Insts.size(); I++) {
      Instruction *Inst = Insts[I];
      if (FirstInst->getParent() == Inst->getParent()) {
        IsValid &= FirstInst->comesBefore(Inst);
      } else {
        IsValid &= DT.dominates(FirstInst->getParent(), Inst->getParent());
      }
    }
    if (not IsValid) {
      errs() << "Cannot merge variable " << Name
             << " : instructions are not in a dominating order\n";
      BasicBlock *FirstInstBlock = FirstInst->getParent();
      BasicBlock *CommonDom = FirstInstBlock;

      for (size_t I = 1; I < Insts.size(); I++) {
        Instruction *Inst = Insts[I];
        BasicBlock *BB = Inst->getParent();
        CommonDom = DT.findNearestCommonDominator(CommonDom, BB);

        if (!CommonDom ||
            CommonDom == &FirstInstBlock->getParent()->getEntryBlock())
          break;
      }
      errs() << "  Common dominator block: "
             << (CommonDom ? CommonDom->getName() : "null") << "\n";
      if (not hoistInstructionChain(FirstInst, CommonDom, DT))
        llvm_unreachable("Cannot hoist instruction to common dominator");
    }

    for (size_t I = 1; I < Insts.size(); I++) {
      Instruction *CurrInst = Insts[I];
      LLVM_DEBUG(errs() << "Merging variable " << Name << " : " << *CurrInst
                        << " into " << *FirstInst << "\n";);
      CurrInst->replaceAllUsesWith(FirstInst);
      CurrInst->eraseFromParent();
    }
  }
}

ICmpInst *findICmpUser(Instruction *Inst) {
  SmallVector<Instruction *, 8> Worklist;
  Worklist.push_back(Inst);

  SmallPtrSet<Instruction *, 8> Visited;

  while (!Worklist.empty()) {
    Instruction *Curr = Worklist.pop_back_val();
    if (!Visited.insert(Curr).second)
      continue;

    for (User *U : Curr->users()) {
      Instruction *UserInst = dyn_cast<Instruction>(U);
      if (!UserInst)
        continue;

      if (auto *ICI = dyn_cast<ICmpInst>(UserInst)) {
        return ICI;
      }

      if (UserInst->isBinaryOp()) {
        Worklist.push_back(UserInst);
      }
    }
  }

  return nullptr;
}

bool onlyUseInsideBlock(Instruction *Inst) {
  BasicBlock *BB = Inst->getParent();
  for (User *U : Inst->users()) {
    Instruction *UserInst = dyn_cast<Instruction>(U);
    if (!UserInst)
      continue;
    if (UserInst->getParent() != BB)
      return false;
  }
  return true;
}

void removeVarAnnotations(Function &F) {
  auto Annotations = findVarInstructions(F);

  SmallVector<CallInst *, 4> ToRemove;
  SmallVector<std::pair<ICmpInst *, CallInst *>, 4> ToAnnotate;

  for (auto &[Inst, Name] : Annotations) {
    for (auto *User : Inst->users()) {
      if (auto *CallInst = dyn_cast<llvm::CallInst>(User)) {
        const Function *Callee = CallInst->getCalledFunction();
        if (Callee and Callee->getName().starts_with("llvm.annotation")) {
          ToRemove.push_back(CallInst);

          auto *ICmp = findICmpUser(CallInst);
          if (ICmp and ICmp->getParent() == CallInst->getParent() and
              onlyUseInsideBlock(ICmp)) {
            ToAnnotate.push_back({ICmp, CallInst});
          }
        }
      }
    }
  }

  for (auto *CI : ToRemove) {
    errs() << "Removing annotation call: " << *CI << "\n";
  }
  for (auto &[ICmp, CallInst] : ToAnnotate) {
    errs() << "Annotating instruction: " << *ICmp << "\n";
  }

  for (auto *CI : ToRemove) {
    auto *Val = CI->getArgOperand(0);
    CI->replaceAllUsesWith(Val);
  }
  for (auto &[ICmp, CallInst] : ToAnnotate) {
    ICmp->setMetadata("cond_variable_annotation",
                      MDNode::get(ICmp->getContext(), {}));
  }
}

} // namespace

SmallVector<std::pair<Instruction *, StringRef>, 2>
polly::findVarInstructions(Function &F) {
  SmallSetVector<std::pair<Instruction *, StringRef>, 2> Annotations;
  for (auto &BB : F) {
    for (auto &I : BB) {
      if (I.hasMetadata("variable_annotation")) {
        MDNode *Node = I.getMetadata("variable_annotation");
        if (Node->getNumOperands() != 1) {
          report_fatal_error("VarFusion pass on " + F.getName() +
                             " : Variable annotation bad format\n");
        }
        auto *VarNameMD = dyn_cast<MDString>(Node->getOperand(0));
        if (!VarNameMD) {
          report_fatal_error("VarFusion pass on " + F.getName() +
                             " : Variable annotation bad format\n");
        }
        StringRef VarName = VarNameMD->getString();
        Annotations.insert({&I, VarName});
        errs() << "Variable annotation found: " << I << " -> " << VarName
               << "\n";
      }
    }
  }
  SmallVector<std::pair<Instruction *, StringRef>, 2> Result(
      {Annotations.begin(), Annotations.end()});
  return Result;
}

PreservedAnalyses VarFusionPass::run(Function &F, FunctionAnalysisManager &FM) {
  if (not F.hasFnAttribute("polly.findSCoP"))
    return PreservedAnalyses::all();

  LLVM_DEBUG(errs() << "VarFusionPass pass run on " << F.getName() << "\n");

  auto &DT = FM.getResult<DominatorTreeAnalysis>(F);

  fusionVarAnnotations(F, DT);

  removeVarAnnotations(F);

  if (verifyFunction(F, &errs())) {
    report_fatal_error(
        "Function verification failed after ExtractAnnotatedFromLoop pass on " +
        F.getName());
  }

  LLVM_DEBUG(errs() << "VarFusionPass pass done\n");
  return PreservedAnalyses::none();
}
