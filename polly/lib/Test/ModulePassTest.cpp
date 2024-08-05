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

#include "polly/Test/ModulePassTest.h"
#include <iostream>

using namespace polly;

AnalysisKey AnalysisModuleTest::Key;

AnalysisModuleTest::Result AnalysisModuleTest::run(Module &M,
                                                   ModuleAnalysisManager &AM) {

  int Count = 0;
  for (auto &F : M) {
    for (auto &BB : F) {
      for (auto &I : BB) {
        (void)I;
        Count++;
      }
    }
  }
  return {Count};
}

PreservedAnalyses
AnalysisModulePrinterPassTest::run(Module &M, ModuleAnalysisManager &AM) {
  OS << "Printing analysis results of AnalysisModuleTest for Module " << "'"
     << M.getName() << "': ";
  OS << AM.getResult<AnalysisModuleTest>(M).front() << "\n";

  return PreservedAnalyses::all();
}

PreservedAnalyses ModulePassTest::run(Module &M, ModuleAnalysisManager &AM) {
  std::cout << M.getName().str() << std::endl;
  return PreservedAnalyses::all();
}
