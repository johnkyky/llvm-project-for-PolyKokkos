//===- polly/CodeGeneration.h - The Polly code generator --------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef POLLY_FUNCTIONMARKEDINLINER_H
#define POLLY_FUNCTIONMARKEDINLINER_H

#include "llvm/IR/PassManager.h"

using namespace llvm;

namespace polly {

struct FunctionMarkedInliner final : PassInfoMixin<FunctionMarkedInliner> {
  PreservedAnalyses run(Module &M, ModuleAnalysisManager &AM);
};

} // namespace polly

#endif // POLLY_FUNCTIONMARKEDINLINER_H
