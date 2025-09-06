//===- polly/JSONExporter.h - Import/Export to/from jscop files.-*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef POLLY_OPENSCOPEXPORTER_H
#define POLLY_OPENSCOPEXPORTER_H

#include "polly/ScopPass.h"
#include "llvm/IR/PassManager.h"
#include <osl/osl.h>

namespace polly {

/// This pass exports a scop to a openscop file. The filename is generated from
/// the concatenation of the function and scop name.
struct OpenSCoPExportPass final : llvm::PassInfoMixin<OpenSCoPExportPass> {
  static void exportOpenScop(Scop &S, std::string FileName);
  llvm::PreservedAnalyses run(Scop &, ScopAnalysisManager &,
                              ScopStandardAnalysisResults &, SPMUpdater &);
};

/// This pass imports a scop from a openscop file. The filename is deduced from
/// the concatenation of the function and scop name.
struct OpenSCoPImportPass final : llvm::PassInfoMixin<OpenSCoPImportPass> {
  static void importOpenScop(Scop &S, std::string FileName);
  llvm::PreservedAnalyses run(Scop &, ScopAnalysisManager &,
                              ScopStandardAnalysisResults &, SPMUpdater &);
};
} // namespace polly

#endif /* POLLY_OPENSCOPEXPORTER_H */
