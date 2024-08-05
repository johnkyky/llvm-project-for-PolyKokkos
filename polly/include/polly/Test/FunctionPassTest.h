//===- polly/CodeGeneration.h - The Polly code generator --------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef POLLY_FUNCTIONPASSTEST_H
#define POLLY_FUNCTIONPASSTEST_H

#include "llvm/IR/PassManager.h"
#include <vector>

using namespace llvm;

namespace polly {

class AnalysisFunctionTest : public AnalysisInfoMixin<AnalysisFunctionTest> {
  friend AnalysisInfoMixin<AnalysisFunctionTest>;

  static AnalysisKey Key;

public:
  using Result = std::vector<int>;
  Result run(Function &F, FunctionAnalysisManager &AM);
};

class AnalysisFunctionPrinterPassTest
    : public AnalysisInfoMixin<AnalysisFunctionPrinterPassTest> {
  raw_ostream &OS;

public:
  explicit AnalysisFunctionPrinterPassTest(raw_ostream &OS) : OS(OS) {}
  PreservedAnalyses run(Function &F, FunctionAnalysisManager &AM);
};

struct FunctionPassTest final : PassInfoMixin<FunctionPassTest> {
  PreservedAnalyses run(Function &F, FunctionAnalysisManager &AM);
};

} // namespace polly

#endif // POLLY_FUNCTIONPASSTEST_H
