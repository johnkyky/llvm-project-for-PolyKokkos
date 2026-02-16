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

#include "polly/Test/ArrayFusion.h"
#include "polly/CodeGen/IRBuilder.h"
#include "polly/Test/ExtractAnnotatedFromLoop.h"
#include "llvm/Analysis/AssumptionCache.h"
#include "llvm/IR/Dominators.h"
#include "llvm/IR/Verifier.h"
#include "llvm/Support/Debug.h"

#define DEBUG_TYPE "polly-array-fusion"

using namespace llvm;
using namespace polly;

namespace {
bool sameArraysFusion(Function &F, ExtractAnnotatedSizes::Result &Anno,
                      DominatorTree &DT) {
  bool Res = false;

  for (auto &[InstArray, Data] : Anno) {
    auto *MDStr = MDString::get(InstArray->getContext(), Data.Name);
    MDNode *MD = MDNode::get(InstArray->getContext(), {MDStr});
    InstArray->setMetadata("array", MD);
    Res = true;
  }

  using NameToInstructionsT =
      llvm::DenseMap<llvm::StringRef, std::vector<Instruction *>>;

  NameToInstructionsT NameToArray;

  // Fill map with instructions grouped by name
  for (const auto &[Inst, Data] : Anno) {
    NameToArray[Data.Name].push_back(Inst);
  }

  // Sort the map in each group by their dominance relationship
  for (auto &[Name, Arrays] : NameToArray) {
    std::sort(
        Arrays.begin(), Arrays.end(), [&](Instruction *A, Instruction *B) {
          if (A->getParent() == B->getParent()) {
            for (Instruction &I : *A->getParent()) {
              if (&I == A)
                return true;
              if (&I == B)
                return false;
            }
            LLVM_DEBUG(errs()
                       << "Instructions not found in their parent block!\n");
          }

          return DT.dominates(A, B);
        });
  }

  // Replace all use of arrays and sizes with the first one in each group
  // (first by dominance)
  for (const auto &[Name, Arrays] : NameToArray) {
    if (Arrays.size() == 1)
      continue;

    auto &FirstArray = Arrays.front();
    auto &FirstArrayData = Anno.Map[FirstArray];

    for (size_t I = 1; I < Arrays.size(); ++I) {
      LLVM_DEBUG(errs() << "Replacing all uses of array: " << *Arrays[I]
                        << " with " << *FirstArray << "\n";);
      Arrays[I]->replaceAllUsesWith(FirstArray);

      auto &FirstArrayDataSizes = FirstArrayData.Sizes;
      auto &OtherArraySizes = Anno.Map.at(Arrays[I]).Sizes;
      for (size_t I = 0; I < FirstArrayDataSizes.size(); ++I) {
        LLVM_DEBUG(errs() << "Replacing all uses of size: "
                          << *OtherArraySizes[I] << " with "
                          << *FirstArrayDataSizes[I] << "\n";);
        OtherArraySizes[I]->replaceAllUsesWith(FirstArrayDataSizes[I]);
      }
    }
  }

  return Res;
}

} // namespace

PreservedAnalyses ArrayFusion::run(Function &F, FunctionAnalysisManager &AM) {
  if (not F.hasFnAttribute("polly.findSCoP")) {
    return PreservedAnalyses::all();
  }

  LLVM_DEBUG(errs() << "ArrayFusion pass run on " << F.getName() << "\n");

  auto DT = DominatorTreeAnalysis().run(F, AM);
  DT.recalculate(F);
  auto Anno = ExtractAnnotatedSizes().run(F, AM);
  sameArraysFusion(F, Anno, DT);

  if (verifyFunction(F, &errs())) {
    report_fatal_error("IR verification failed.");
  }

  LLVM_DEBUG(errs() << "ArrayFusion pass done\n");

  return PreservedAnalyses::none();
}
