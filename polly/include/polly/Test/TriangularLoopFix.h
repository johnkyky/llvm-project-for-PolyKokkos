//===- polly/CodeGeneration.h - The Polly code generator --------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef POLLY_TRIANGULARLOOPFIX_H
#define POLLY_TRIANGULARLOOPFIX_H

#include "llvm/Analysis/LoopInfo.h"
#include "llvm/Analysis/ScalarEvolution.h"
#include "llvm/IR/PassManager.h"

using namespace llvm;

namespace polly {

std::vector<Loop *> getTriangularLoops(LoopInfo &LI, ScalarEvolution &SE,
                                       DominatorTree &DT);

struct TriangularLoopFixPass final : PassInfoMixin<TriangularLoopFixPass> {
  PreservedAnalyses run(Function &F, FunctionAnalysisManager &AM);
};

} // namespace polly

#endif // POLLY_TRIANGULARLOOPFIX_H
