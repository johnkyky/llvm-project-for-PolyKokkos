//===- polly/CodeGeneration.h - The Polly code generator --------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef POLLY_MARKFUNCTION_H
#define POLLY_MARKFUNCTION_H

#include "llvm/IR/PassManager.h"
#include <vector>

using namespace llvm;

namespace polly {

struct MarkFunctionToFindScop final : PassInfoMixin<MarkFunctionToFindScop> {
  PreservedAnalyses run(Function &F, FunctionAnalysisManager &AM);
};

} // namespace polly

#endif // POLLY_MARKFUNCTION_H
