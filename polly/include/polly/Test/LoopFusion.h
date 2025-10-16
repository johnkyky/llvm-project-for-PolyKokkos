//===- polly/CodeGeneration.h - The Polly code generator --------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef POLLY_LOOPFUSION_H
#define POLLY_LOOPFUSION_H

#include "llvm/Analysis/LoopInfo.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/Dominators.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/PassManager.h"
#include <utility>

using namespace llvm;

namespace polly {

struct LoopBoundT {
  enum Type { Lower, Upper };

  Instruction *Inst = nullptr;
  Type BoundType = Type::Lower;
  size_t Depth = 0;
  size_t IndexPolicy = 0;
};

raw_ostream &operator<<(raw_ostream &OS, const SmallVector<LoopBoundT, 4> &LBA);

class LoopBoundAnalysis : public AnalysisInfoMixin<LoopBoundAnalysis> {
  friend AnalysisInfoMixin<LoopBoundAnalysis>;

  static AnalysisKey Key;

private:
  BasicBlock *getExitBlock(Function &F);

  SmallVector<LoopBoundT, 4> getLoopBoundInstructions(Function &F,
                                                      DominatorTree &DT);

public:
  using Result = SmallVector<LoopBoundT, 4>;
  Result run(Function &F, FunctionAnalysisManager &AM);
};

struct LoopFusionPass final : PassInfoMixin<LoopFusionPass> {
  PreservedAnalyses run(Function &F, FunctionAnalysisManager &AM);
};

} // namespace polly

#endif // POLLY_LOOPFUSION_H
