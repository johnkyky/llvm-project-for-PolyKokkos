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

#include "polly/Test/MarkFunctionToFindScop.h"

using namespace polly;

#include "llvm/IR/PassManager.h"
#include "llvm/Passes/PassBuilder.h"

using namespace llvm;

PreservedAnalyses MarkFunctionToFindScop::run(Function &F,
                                              FunctionAnalysisManager &AM) {
  LLVMContext &Ctx = F.getContext();
  MDNode *PollyMD = MDNode::get(Ctx, MDString::get(Ctx, "findSCoP"));
  llvm::Attribute CustomAttr = llvm::Attribute::get(Ctx, "MyCustomAttribute");

  for (auto &BB : F) {
    for (auto &I : BB) {
      if (I.getMetadata("cppoly.pragma") != nullptr) {
        /*llvm::errs() << "Found metadata in " << F.getName().str() << '\n';*/
        F.setMetadata("polly", PollyMD);
        F.addFnAttr(CustomAttr);
        return PreservedAnalyses::all();
      }
    }
  }

  return PreservedAnalyses::all();
}
