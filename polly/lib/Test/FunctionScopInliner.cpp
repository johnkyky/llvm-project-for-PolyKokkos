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

#include "polly/Test/FunctionScopInliner.h"
#include "polly/ScopInfo.h"
#include "llvm/Analysis/BlockFrequencyInfo.h"
#include "llvm/Analysis/OptimizationRemarkEmitter.h"
#include "llvm/Analysis/ProfileSummaryInfo.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/Module.h"
#include "llvm/IR/PassManager.h"
#include "llvm/Passes/PassBuilder.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Transforms/Utils/Cloning.h"
#include "llvm/Transforms/Utils/ModuleUtils.h"

using namespace polly;
using namespace llvm;

#define DEBUG_TYPE "inline"

PreservedAnalyses impl1(Function &F, FunctionAnalysisManager &AM) {
  ScopDetection &SD = AM.getResult<ScopAnalysis>(F);
  ScopInfo &SI = AM.getResult<ScopInfoAnalysis>(F);
  (void)SD;
  (void)SI;

  SmallSetVector<CallBase *, 16> Calls;
  bool Changed = false;
  bool InlinedFunction = false;
  SmallVector<Function *, 16> InlinedFunctions;
  Module &M = *F.getParent();

  auto GetAssumptionCache = [&](Function &F) -> AssumptionCache & {
    return AM.getResult<AssumptionAnalysis>(F);
  };
  auto GetBFI = [&](Function &F) -> BlockFrequencyInfo & {
    return AM.getResult<BlockFrequencyAnalysis>(F);
  };
  auto GetAAR = [&](Function &F) -> AAResults & {
    return AM.getResult<AAManager>(F);
  };

  // When callee coroutine function is inlined into caller coroutine function
  // before coro-split pass,
  // coro-early pass can not handle this quiet well.
  // So we won't inline the coroutine function if it have not been unsplited
  if (F.isPresplitCoroutine())
    return PreservedAnalyses::none();

  if (F.isDeclaration() or not isInlineViable(F).isSuccess()) {
    return PreservedAnalyses::none();
  }

  F.dump();

  for (User *U : F.users()) {
    if (auto *CB = dyn_cast<CallBase>(U)) {
      Function *Caller = CB->getCaller();
      ScopDetection &TSD = AM.getResult<ScopAnalysis>(*Caller);
      ScopInfo &TSI = AM.getResult<ScopInfoAnalysis>(*Caller);
      (void)TSD;
      (void)TSI;
      CB->dump();
      if (CB->getCalledFunction() == &F) {
        llvm::errs() << "CB->getCalledFunction() == &F" << "\n";
      }
      if (CB->hasFnAttr(Attribute::AlwaysInline)) {
        llvm::errs() << "always_inline" << "\n";
      }
      if (not CB->getAttributes().hasFnAttr(Attribute::NoInline)) {
        llvm::errs() << "not noinline" << "\n";
      }
      if (CB->getMetadata("cppoly.inline") != nullptr) {
        llvm::errs() << "cppoly inline" << "\n";
      }
      if (CB->getMetadata("cppoly.inline") != nullptr) {
        llvm::errs() << "CB->getMetadata(\"cppoly.inline\") != nullptr" << "\n";
      }
      if (CB->getCalledFunction() == &F &&
          CB->getMetadata("cppoly.inline") != nullptr) {
        AM.invalidate(*Caller, PreservedAnalyses::none());
        Calls.insert(CB);
      }
    }
  }

  for (CallBase *CB : Calls) {
    Function *Caller = CB->getCaller();
    OptimizationRemarkEmitter ORE(Caller);
    DebugLoc DLoc = CB->getDebugLoc();
    BasicBlock *Block = CB->getParent();

    ProfileSummaryInfo *PSI =
        AM.getResult<ModuleAnalysisManagerFunctionProxy>(*Caller)
            .getCachedResult<ProfileSummaryAnalysis>(
                *CB->getParent()->getParent()->getParent());

    InlineFunctionInfo IFI(GetAssumptionCache, PSI, &GetBFI(*Caller),
                           &GetBFI(F));

    InlineResult Res = InlineFunction(*CB, IFI, true, &GetAAR(F), true);
    if (!Res.isSuccess()) {
      ORE.emit([&]() {
        return OptimizationRemarkMissed(DEBUG_TYPE, "NotInlined", DLoc, Block)
               << "'" << ore::NV("Callee", &F) << "' is not inlined into '"
               << ore::NV("Caller", Caller)
               << "': " << ore::NV("Reason", Res.getFailureReason());
      });
      continue;
    }

    llvm::errs() << "\n\n Inlined " << F.getName() << " into "
                 << Caller->getName() << "\n";

    emitInlinedIntoBasedOnCost(ORE, DLoc, Block, F, *Caller,
                               InlineCost::getAlways("always inline attribute"),
                               /*ForProfileContext=*/false, DEBUG_TYPE);

    Changed = true;
    InlinedFunction = true;

    AM.invalidate(*Caller, PreservedAnalyses::none());
  }

  if (F.hasFnAttribute(Attribute::AlwaysInline) or InlinedFunction) {
    // Remember to try and delete this function afterward. This both avoids
    // re-walking the rest of the module and avoids dealing with any
    // iterator invalidation issues while deleting functions.
    InlinedFunctions.push_back(&F);
  }

  // Remove any live functions.
  erase_if(InlinedFunctions, [&](Function *F) {
    F->removeDeadConstantUsers();
    return !F->isDefTriviallyDead();
  });

  // Delete the non-comdat ones from the module and also from our vector.
  auto *NonComdatBegin =
      partition(InlinedFunctions, [&](Function *F) { return F->hasComdat(); });
  for (Function *F : make_range(NonComdatBegin, InlinedFunctions.end())) {
    M.getFunctionList().erase(F);
    Changed = true;
  }
  InlinedFunctions.erase(NonComdatBegin, InlinedFunctions.end());

  if (!InlinedFunctions.empty()) {
    // Now we just have the comdat functions. Filter out the ones whose
    // comdats are not actually dead.
    filterDeadComdatFunctions(InlinedFunctions);
    // The remaining functions are actually dead.
    for (Function *F : InlinedFunctions) {
      M.getFunctionList().erase(F);
      Changed = true;
    }
  }

  llvm::errs() << "End of Inliner\n\n\n";

  return PreservedAnalyses::none();
}

PreservedAnalyses FuncCallScopInliner::run(Function &F,
                                           FunctionAnalysisManager &AM) {
  llvm::errs() << "Running Inliner on function: " << F.getName() << "\n";
  return impl1(F, AM);
}
