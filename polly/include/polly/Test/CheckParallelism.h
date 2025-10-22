//===- polly/CodeGeneration.h - The Polly code generator --------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef POLLY_CHECKPARALLELISM_H
#define POLLY_CHECKPARALLELISM_H

#include "polly/ScopPass.h"
#include "llvm/IR/PassManager.h"

using namespace llvm;

namespace polly {

struct CheckParallelismPass final : PassInfoMixin<CheckParallelismPass> {
  PreservedAnalyses run(Scop &, ScopAnalysisManager &,
                        ScopStandardAnalysisResults &, SPMUpdater &);
};

} // namespace polly

#endif // POLLY_CHECKPARALLELISM_H
