//===- polly/CodeGeneration.h - The Polly code generator --------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef POLLY_VARFUSION_H
#define POLLY_VARFUSION_H

#include "llvm/IR/Instruction.h"
#include "llvm/IR/PassManager.h"
#include "llvm/Pass.h"

using namespace llvm;

namespace polly {

SmallVector<std::pair<Instruction *, StringRef>, 2>
findVarInstructions(Function &F);

struct VarFusionPass final : PassInfoMixin<VarFusionPass> {
  PreservedAnalyses run(Function &F, FunctionAnalysisManager &FM);
};

} // namespace polly

#endif // POLLY_VARFUSION_H
