//===- polly/CodeGeneration.h - The Polly code generator --------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef POLLY_MODULEPASSTEST_H
#define POLLY_MODULEPASSTEST_H

#include "llvm/IR/PassManager.h"

using namespace llvm;

namespace polly {

class AnalysisModuleTest : public AnalysisInfoMixin<AnalysisModuleTest> {
  friend AnalysisInfoMixin<AnalysisModuleTest>;

  static AnalysisKey Key;

public:
  using Result = std::vector<int>;
  Result run(Module &M, ModuleAnalysisManager &AM);
};

class AnalysisModulePrinterPassTest
    : public AnalysisInfoMixin<AnalysisModulePrinterPassTest> {
  raw_ostream &OS;

public:
  explicit AnalysisModulePrinterPassTest(raw_ostream &OS) : OS(OS) {}
  PreservedAnalyses run(Module &M, ModuleAnalysisManager &AM);
};

struct ModulePassTest final : PassInfoMixin<ModulePassTest> {
  PreservedAnalyses run(Module &M, ModuleAnalysisManager &AM);
};

} // namespace polly

#endif // POLLY_MODULEPASSTEST_H
