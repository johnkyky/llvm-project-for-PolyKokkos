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
#include "llvm/IR/PassManager.h"
#include "llvm/Passes/PassBuilder.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;
using namespace polly;

void markFunction(Function *F, const bool Mark = false) {
  LLVMContext &Ctx = F->getContext();
  MDNode *PollyMD = MDNode::get(Ctx, MDString::get(Ctx, "findSCoP"));
  llvm::Attribute CustomAttr = llvm::Attribute::get(Ctx, "MyCustomAttribute");

  if (F == nullptr or F->isDeclaration())
    return;

  bool AddedAttr = false;
  if (Mark) {
    F->addFnAttr(CustomAttr);
    F->addFnAttr("caca");
    F->setMetadata("polly", PollyMD);
    F->removeFnAttr(llvm::Attribute::NoInline);
    llvm::errs() << "add Attribute: " << F->getName() << "\n";
    AddedAttr = true;
  }

  for (auto &BB : *F) {
    for (auto &I : BB) {
      bool HasCppolyPragma = false;

      if (I.getMetadata("cppoly.pragma") != nullptr)
        HasCppolyPragma = true;

      if (Mark) {
        if (not HasCppolyPragma) {
          I.setMetadata("cppoly.pragma", PollyMD);
        } else {
          if (auto *CB = dyn_cast<CallBase>(&I)) {
            markFunction(CB->getCalledFunction(), true);
          }
        }
      } else {
        if (HasCppolyPragma) {
          F->setMetadata("polly", PollyMD);
          F->addFnAttr(CustomAttr);
          F->addFnAttr("caca");
          AddedAttr = true;
        }
        if (auto *CB = dyn_cast<CallBase>(&I)) {
          Function *Callee = CB->getCalledFunction();
          markFunction(Callee, HasCppolyPragma ? true : false);
        }
      }
    }
  }

  llvm::errs() << "After -> " << *F << "\n\n";
}

PreservedAnalyses MarkFunctionToFindScop::run(Function &F,
                                              FunctionAnalysisManager &AM) {
  if (F.getName().str() == "main")
    markFunction(&F, false);

  return PreservedAnalyses::none();
}

PreservedAnalyses MarkFunctionToFindScopModule::run(Module &M,
                                                    ModuleAnalysisManager &AM) {
  for (auto &F : M) {
    if (F.getName().str() == "main")
      markFunction(&F, false);
  }
  return PreservedAnalyses::none();
}
