//===- polly/CodeGeneration.h - The Polly code generator --------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef POLLY_EXTRACTANNOTATEDFROMLOOP_H
#define POLLY_EXTRACTANNOTATEDFROMLOOP_H

#include "llvm/IR/PassManager.h"
#include "llvm/Pass.h"
#include <unordered_map>

using namespace llvm;

namespace polly {

std::pair<CallInst *, StringRef> isAnnotationInstruction(Instruction *Instr,
                                                         StringRef StrStart);

using SizesT = std::vector<Instruction *>;
using NameT = StringRef;

struct ArrayData {
  NameT Name;
  SizesT Sizes;

  ArrayData(NameT N, SizesT S) : Name(N), Sizes(S) {}
  ArrayData() = default;
};

using ArrayToDataT = std::unordered_map<Instruction *, ArrayData>;

struct AnnotationData {
  ArrayToDataT Map;

  AnnotationData() = default;
  void print(raw_ostream &OS) const;

  auto begin() { return Map.begin(); }
  auto end() { return Map.end(); }
  auto begin() const { return Map.begin(); }
  auto end() const { return Map.end(); }
  bool empty() const { return Map.empty(); }
  size_t size() const { return Map.size(); }
};

class ExtractAnnotatedSizes : public AnalysisInfoMixin<ExtractAnnotatedSizes> {
  friend AnalysisInfoMixin<ExtractAnnotatedSizes>;
  static AnalysisKey Key;

public:
  using Result = AnnotationData;
  Result run(Function &F, FunctionAnalysisManager &AM);
};

class ExtractAnnotatedSizesWrapperPass : public FunctionPass {
  AnnotationData Anno;

public:
  static char ID;

  ExtractAnnotatedSizesWrapperPass();

  AnnotationData &getAnnotationSizes() { return Anno; }
  const AnnotationData &getAnnotationSizes() const { return Anno; }

  bool runOnFunction(Function &F) override;

  void getAnalysisUsage(AnalysisUsage &AU) const override {
    AU.setPreservesAll();
  }

  void print(raw_ostream &OS, const Module *M = nullptr) const override;
};

struct ExtractAnnotatedFromLoop final
    : PassInfoMixin<ExtractAnnotatedFromLoop> {
  PreservedAnalyses run(Function &F, FunctionAnalysisManager &FM);
};

} // namespace polly

#endif // POLLY_EXTRACTANNOTATEDFROMLOOP_H
