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

#include "polly/Test/FunctionPassTest.h"
#include <iostream>

using namespace polly;

#include "llvm/IR/PassManager.h"
#include "llvm/Passes/PassBuilder.h"

#include <iostream>

using namespace llvm;

AnalysisKey AnalysisFunctionTest::Key;

AnalysisFunctionTest::Result
AnalysisFunctionTest::run(Function &F, FunctionAnalysisManager &AM) {

  int Count = 0;
  for (Function::iterator BB = F.begin(); BB != F.end(); BB++) {
    for (BasicBlock::iterator I = BB->begin(); I != BB->end(); I++) {
      Count++;
    }
  }

  return {Count};
}

PreservedAnalyses
AnalysisFunctionPrinterPassTest::run(Function &F, FunctionAnalysisManager &AM) {
  OS << "Printing analysis results of AnalysisFunctionTest for function " << "'"
     << F.getName() << "': ";
  OS << AM.getResult<AnalysisFunctionTest>(F).front() << "\n";

  return PreservedAnalyses::all();
}

PreservedAnalyses FunctionPassTest::run(Function &F,
                                        FunctionAnalysisManager &AM) {
  std::cout << F.getName().str() << std::endl;
  return PreservedAnalyses::all();
}
