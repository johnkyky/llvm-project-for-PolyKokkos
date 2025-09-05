//===- polly/CodeGeneration.h - The Polly code generator --------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef POLLY_PLUTOSCHEDULEOPTIMIZERPASS_H
#define POLLY_PLUTOSCHEDULEOPTIMIZERPASS_H

#include "polly/ScopPass.h"

using namespace llvm;

namespace polly {

struct PlutoScheduleOptimizerPass final
    : PassInfoMixin<PlutoScheduleOptimizerPass> {
  llvm::PreservedAnalyses run(Scop &S, ScopAnalysisManager &SAM,
                              ScopStandardAnalysisResults &SAR, SPMUpdater &U);
};

} // namespace polly

#endif // POLLY_PLUTOSCHEDULEOPTIMIZERPASS_H
