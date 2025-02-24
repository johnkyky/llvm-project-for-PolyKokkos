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
#include "llvm/IR/Attributes.h"
#include "llvm/IR/PassManager.h"
#include "llvm/Passes/PassBuilder.h"
#include "llvm/Support/raw_ostream.h"
#include <iostream>

using namespace llvm;
using namespace polly;

namespace {
void markFunctionWithAnnotation(Module &M) {
  GlobalVariable *GlobalAnnotations =
      M.getGlobalVariable("llvm.global.annotations");

  if (GlobalAnnotations) {
    for (auto &GlobalAnnotation : GlobalAnnotations->operands()) {
      auto *A = cast<ConstantArray>(GlobalAnnotation);
      for (auto &Op : A->operands()) {
        auto *E = cast<ConstantStruct>(Op);
        if (auto *Func = dyn_cast<Function>(E->getOperand(0))) {
          auto *GlobalStr = cast<GlobalVariable>(E->getOperand(1));
          auto Str =
              cast<ConstantDataArray>(GlobalStr->getOperand(0))->getAsCString();
          if (Str == "findscop") {
            llvm::errs() << "\tFound findscop annotation on function "
                         << Func->getName() << "\n";
            Func->addFnAttr("polly.findSCoP");
          } else if (Str == "multi_parallel_for") {
            llvm::errs() << "\tFound multi_parallel_for annotation on function "
                         << Func->getName() << "\n";
            Func->addFnAttr("polly.multi_parallel_for");
            Func->addFnAttr("polly.findSCoP");
          }
        }
      }
    }
  }
}

void markFunctionWithPragma(int Depth, Function *F, const bool Mark = false) {
  if (F == nullptr or F->isDeclaration()) {
    return;
  }

  LLVMContext &Ctx = F->getContext();
  MDNode *PollyMD = MDNode::get(Ctx, MDString::get(Ctx, "findSCoP"));

  if (F == nullptr or F->isDeclaration())
    return;

  if (Mark) {
    F->setMetadata("polly", PollyMD);
    F->removeFnAttr(llvm::Attribute::NoInline);
    F->addFnAttr("polly.findSCoP");
  }

  bool HasCppolyPragma = false;
  for (auto &BB : *F) {
    for (auto &I : BB) {
      if (I.getMetadata("cppoly.pragma") != nullptr)
        HasCppolyPragma = true;
    }
  }

  for (auto &BB : *F) {
    for (auto &I : BB) {
      if ((HasCppolyPragma or Mark) and
          I.getMetadata("cppoly.pragma") != nullptr) {
        // llvm::errs() << "add cppoly " << I << "\n";
        I.setMetadata("cppoly.pragma", PollyMD);
      }

      if (HasCppolyPragma and not Mark) {
        if (F->getMetadata("polly") == nullptr) {
          F->setMetadata("polly", PollyMD);
          F->addFnAttr("polly.findSCoP");
          // for (int I = 0; I < Depth; I++)
          //   llvm::errs() << "\t";
          // llvm::errs() << "->->-> marking2 " << F->getName() << "\n";
        }
      }
      if (HasCppolyPragma or Mark) {
        auto *CB = dyn_cast<CallBase>(&I);
        if (CB) {
          I.setMetadata("polly.inline", PollyMD);
          Function *Callee = CB->getCalledFunction();
          markFunctionWithPragma(Depth + 1, Callee, true);
        }
      }

      auto *CB = dyn_cast<CallBase>(&I);
      if (not HasCppolyPragma and CB) {
        Function *Callee = CB->getCalledFunction();
        markFunctionWithPragma(Depth + 1, Callee, false);
      }
    }
  }

  /*llvm::errs() << "After -> " << *F << "\n\n";*/
}

void test(Function *F, const bool IntoMultiParallelFor = false) {
  if (not F)
    return;

  LLVMContext &Ctx = F->getContext();
  static MDNode *PollyMD =
      MDNode::get(Ctx, MDString::get(Ctx, "polly.metadata"));

  for (auto &BB : *F) {
    for (auto &I : BB) {
      auto *CB = dyn_cast<CallBase>(&I);
      if (CB) {
        if (F->hasFnAttribute("polly.multi_parallel_for") or
            IntoMultiParallelFor) {
          auto *CalledFunction = CB->getCalledFunction();

          llvm::errs() << "on ajoute polly.inline sur " << *CB << " "
                       << CalledFunction->getName() << "\n";
          CB->setMetadata("polly.inline", PollyMD);
          if (CalledFunction)
            CalledFunction->addFnAttr("polly.inline");
        }
        Function *Callee = CB->getCalledFunction();
        test(Callee, F->hasFnAttribute("polly.multi_parallel_for"));
      }
    }
  }
}

} // namespace

PreservedAnalyses MarkFunctionToFindScop::run(Module &M,
                                              ModuleAnalysisManager &AM) {
  llvm::errs() << "MarkFunctionToFindScop run on " << M.getName() << "\n";

  markFunctionWithAnnotation(M);

  for (auto &F : M) {
    if (F.getName().str() == "main") {
      // markFunctionWithPragma(0, &F, false);
      test(&F);
    }
  }

  llvm::errs() << "MarkFunctionToFindScop done\n";

  return PreservedAnalyses::none();
}
