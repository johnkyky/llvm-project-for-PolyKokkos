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

#include "polly/Test/FunctionMarkedInliner.h"
#include "llvm/ADT/SetVector.h"
#include "llvm/Analysis/AliasAnalysis.h"
#include "llvm/Analysis/AssumptionCache.h"
#include "llvm/Analysis/InlineCost.h"
#include "llvm/Analysis/OptimizationRemarkEmitter.h"
#include "llvm/Analysis/ProfileSummaryInfo.h"
#include "llvm/IR/Module.h"
#include "llvm/InitializePasses.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Transforms/IPO/AlwaysInliner.h"
#include "llvm/Transforms/IPO/Inliner.h"
#include "llvm/Transforms/Utils/Cloning.h"
#include "llvm/Transforms/Utils/ModuleUtils.h"

using namespace llvm;
using namespace polly;

#define DEBUG_TYPE "inline"

namespace {
bool inlineImpl(Module &M, bool InsertLifetime, ProfileSummaryInfo &PSI,
                function_ref<AssumptionCache &(Function &)> GetAssumptionCache,
                function_ref<AAResults &(Function &)> GetAAR,
                function_ref<BlockFrequencyInfo &(Function &)> GetBFI) {
  SmallSetVector<CallBase *, 8> Calls;
  bool Changed = false;
  for (Function &F : M) {
    // Exit early if the function is not marked with polly metadata
    if (not F.hasFnAttribute("polly.inline"))
      continue;

    llvm::errs() << "on cherche a inline " << F.getName() << "\n";

    // When callee coroutine function is inlined into caller coroutine function
    // before coro-split pass,
    // coro-early pass can not handle this quiet well.
    // So we won't inline the coroutine function if it have not been unsplited
    if (F.isPresplitCoroutine())
      continue;

    if (!F.isDeclaration() && isInlineViable(F).isSuccess()) {
      Calls.clear();

      for (User *U : F.users())
        if (auto *CB = dyn_cast<CallBase>(U)) {
          if (CB->getCalledFunction() == &F and
              CB->getMetadata("polly.inline") != nullptr) {
            Calls.insert(CB);
          }
        }

      for (CallBase *CB : Calls) {
        Function *Caller = CB->getCaller();
        OptimizationRemarkEmitter ORE(Caller);
        DebugLoc DLoc = CB->getDebugLoc();
        BasicBlock *Block = CB->getParent();

        InlineFunctionInfo IFI(GetAssumptionCache, &PSI,
                               GetBFI ? &GetBFI(*Caller) : nullptr,
                               GetBFI ? &GetBFI(F) : nullptr);

        InlineResult Res = InlineFunction(*CB, IFI, /*MergeAttributes=*/true,
                                          &GetAAR(F), InsertLifetime);
        if (!Res.isSuccess()) {
          // std::cout << "on echoue a inline " << F.getName().str() << "\n";
          ORE.emit([&]() {
            return OptimizationRemarkMissed(DEBUG_TYPE, "NotInlined", DLoc,
                                            Block)
                   << "'" << ore::NV("Callee", &F) << "' is not inlined into '"
                   << ore::NV("Caller", Caller)
                   << "': " << ore::NV("Reason", Res.getFailureReason());
          });
          continue;
        }

        emitInlinedIntoBasedOnCost(
            ORE, DLoc, Block, F, *Caller,
            InlineCost::getAlways("always inline attribute"),
            /*ForProfileContext=*/false, DEBUG_TYPE);

        Changed = true;
      }
    }
  }
  return Changed;
}
} // namespace

PreservedAnalyses FunctionMarkedInliner::run(Module &M,
                                             ModuleAnalysisManager &AM) {
  llvm::errs() << "FunctionMarkedInliner run on " << M.getName() << "\n";
  FunctionAnalysisManager &FAM =
      AM.getResult<FunctionAnalysisManagerModuleProxy>(M).getManager();
  auto GetAssumptionCache = [&](Function &F) -> AssumptionCache & {
    return FAM.getResult<AssumptionAnalysis>(F);
  };
  auto GetBFI = [&](Function &F) -> BlockFrequencyInfo & {
    return FAM.getResult<BlockFrequencyAnalysis>(F);
  };
  auto GetAAR = [&](Function &F) -> AAResults & {
    return FAM.getResult<AAManager>(F);
  };
  auto &PSI = AM.getResult<ProfileSummaryAnalysis>(M);

  bool Changed = inlineImpl(M, true, PSI, GetAssumptionCache, GetAAR, GetBFI);

  llvm::errs() << "FunctionMarkedInliner done\n";

  return Changed ? PreservedAnalyses::none() : PreservedAnalyses::all();
}
