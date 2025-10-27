//===- CodeGeneration.cpp - Code generate the Scops using ISL. ---------======//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
//
//===----------------------------------------------------------------------===//

#include "polly/Test/CheckParallelism.h"
#include "polly/CodeGen/IslAst.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include "isl/ast.h"

using namespace llvm;
using namespace polly;

namespace {
bool visite(__isl_take isl_ast_node *Node, Scop &S, unsigned Depth = 0);

bool visiteMark(__isl_take isl_ast_node *Node, Scop &S) {
  auto *Child = isl_ast_node_mark_get_node(Node);
  isl_ast_node_free(Node);

  return visite(Child, S);
}

bool visiteFor(__isl_take isl_ast_node *For, Scop &S, unsigned Depth) {
  bool IsParallel = IslAstInfo::isExecutedInParallel(
      isl::manage_copy(For), S.getBackend() > Scop::Serial);

  isl_ast_node_free(For);

  if (IsParallel) {
    return true;
  }

  return false;
}

bool visiteIf(__isl_take isl_ast_node *If, Scop &S, unsigned Depth) {
  bool IfValid = visite(isl_ast_node_if_get_then(If), S);

  if (isl_ast_node_if_has_else(If))
    IfValid &= visite(isl_ast_node_if_get_else(If), S);

  isl_ast_node_free(If);

  return IfValid;
}

bool visiteUser(__isl_take isl_ast_node *User) {
  isl_ast_node_free(User);
  return true;
}

bool visiteBlock(__isl_take isl_ast_node *Block, Scop &S, unsigned Depth) {
  isl_ast_node_list *List = isl_ast_node_block_get_children(Block);

  bool BlockValid = true;
  for (int I = 0; I < isl_ast_node_list_n_ast_node(List); ++I)
    BlockValid &= visite(isl_ast_node_list_get_ast_node(List, I), S);

  isl_ast_node_free(Block);
  isl_ast_node_list_free(List);

  return BlockValid;
}

bool visite(__isl_take isl_ast_node *Node, Scop &S, unsigned Depth) {
  switch (isl_ast_node_get_type(Node)) {
  case isl_ast_node_error:
    llvm_unreachable("CheckParallelism Error");
  case isl_ast_node_mark:
    return visiteMark(Node, S);
  case isl_ast_node_for:
    return visiteFor(Node, S, Depth);
  case isl_ast_node_if:
    return visiteIf(Node, S, Depth);
  case isl_ast_node_user:
    return visiteUser(Node);
  case isl_ast_node_block:
    return visiteBlock(Node, S, Depth);
  }

  llvm_unreachable("Unknown isl_ast_node type");
}

} // namespace

PreservedAnalyses CheckParallelismPass::run(Scop &S, ScopAnalysisManager &SAM,
                                            ScopStandardAnalysisResults &AR,
                                            SPMUpdater &) {
  if (S.getBackend() == Scop::BackendTy::Undefined)
    report_fatal_error("CheckParallelismPass: Scop has no backend\n");
  if (S.getBackend() == Scop::BackendTy::Serial) {
    errs() << "CheckParallelismPass: Scop has Serial backend\n";
    return PreservedAnalyses::all();
  }

  errs() << "CheckParallelismPass pass run on " << S.getName() << "\n";

  auto &AI = SAM.getResult<IslAstAnalysis>(S, AR);

  IslAst &Ast = AI.getIslAst();
  isl::ast_node AstRoot = Ast.getAst();

  if (AstRoot.is_null()) {
    errs() << "CheckParallelismPass: AstRoot is null\n";
    return PreservedAnalyses::all();
  }

  bool IsValid = true;
  // Kokkos generate parallel for only on the outermost loop
  IsValid = visite(AstRoot.release(), S, 0);

  errs() << "CheckParallelismPass: Scop " << S.getName()
         << (IsValid ? " is " : " is not ") << "valid for Kokkos backend "
         << S.getBackendStr() << "\n";

  errs() << "CheckParallelismPass pass done\n";

  return PreservedAnalyses::none();
}
