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

#include "polly/Test/ExtractAnnotatedFromLoop.h"
#include "llvm/Analysis/ScalarEvolution.h"
#include "llvm/Analysis/ScalarEvolutionExpressions.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/DiagnosticInfo.h"
#include "llvm/IR/Dominators.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/Metadata.h"
#include "llvm/IR/PassManager.h"
#include "llvm/Pass.h"
#include "llvm/Passes/PassBuilder.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;
using namespace polly;

std::pair<CallInst *, StringRef>
polly::isAnnotationInstruction(Instruction *Instr, StringRef StrStart) {
  if (auto *Call = dyn_cast<CallInst>(Instr)) {
    if (const Function *Callee = Call->getCalledFunction()) {
      if (Callee->getName().starts_with("llvm.annotation")) {
        if (auto *Str = dyn_cast<GlobalVariable>(Call->getArgOperand(1))) {
          if (auto *Init = dyn_cast<ConstantDataArray>(Str->getInitializer())) {
            auto StrRef = Init->getAsCString();
            if (StrRef.starts_with(StrStart)) {
              return {Call, StrRef};
            }
          }
        }
      }
    }
  }
  return {nullptr, ""};
}

namespace {

AnnotationData extractArrayInfo(Function &F) {
  AnnotationData Anno;

  for (auto &BB : F) {
    for (auto It = BB.begin(); It != BB.end(); It++) {
      if (auto *AnnoInstArray = isAnnotationInstruction(&*It, "array").first) {
        auto *Op = AnnoInstArray->getOperand(0);
        if (not Op)
          continue;

        auto *PTI = dyn_cast<PtrToIntInst>(Op);
        if (not PTI)
          continue;

        Value *ArrayValue = PTI->getOperand(0);
        if (not ArrayValue)
          continue;

        auto *ArrayInst = dyn_cast<Instruction>(ArrayValue);
        if (not ArrayInst)
          continue;

        It++;

        auto [CallInst, StrRef] = isAnnotationInstruction(&*It, "name");
        if (not CallInst)
          continue;

        Op = CallInst->getOperand(0);
        if (not Op)
          continue;

        bool HasName = false;
        StringRef StrRefName;
        if (auto *CE = dyn_cast<ConstantExpr>(Op)) {
          if (CE->getOpcode() == Instruction::PtrToInt) {
            Value *PtrOperand = CE->getOperand(0);
            if (auto *GV = dyn_cast<GlobalVariable>(PtrOperand)) {
              if (GV->hasInitializer()) {
                if (auto *StructInit =
                        dyn_cast<ConstantStruct>(GV->getInitializer())) {
                  Value *FirstElem = StructInit->getOperand(0);
                  if (auto *Array = dyn_cast<ConstantDataArray>(FirstElem)) {
                    if (Array->isCString()) {
                      HasName = true;
                      StrRefName = Array->getAsCString();
                    }
                  }
                }
              }
            }
          }
        }
        if (not HasName)
          continue;

        It++;
        SizesT S;

        while (auto *AnnoInstDim = isAnnotationInstruction(&*It, "dim").first) {
          Op = AnnoInstDim->getOperand(0);
          if (not Op)
            continue;

          auto *Size = dyn_cast<Instruction>(Op);
          if (not Size) {
            break;
          }

          S.push_back(Size);
          It++;
        }

        std::reverse(S.begin(), S.end());
        Anno.Map.insert({ArrayInst, {StrRefName, S}});
      }
    }
  }
  return Anno;
}

bool extractLoopBoundAnnotation(Function &F) {
  bool Res = false;
  for (auto &BB : F) {
    for (auto &I : BB) {
      auto [CallInstLower, StrRefLower] =
          isAnnotationInstruction(&I, "lower bound");
      auto [CallInstUpper, StrRefUpper] =
          isAnnotationInstruction(&I, "upper bound");
      if (CallInstLower or CallInstUpper) {
        auto StrRef = CallInstLower ? StrRefLower : StrRefUpper;
        auto *CallInst = CallInstLower ? CallInstLower : CallInstUpper;

        if (auto *Inst = dyn_cast<Instruction>(CallInst->getOperand(0))) {
          llvm::LLVMContext &Ctx = Inst->getContext();

          llvm::SmallVector<llvm::StringRef, 4> StrRefParts;
          StrRef.split(StrRefParts, ' ');

          if (StrRefParts.size() != 3) {
            errs() << "ExtractAnnotatedFromLoop pass on " << F.getName()
                   << " : Bad annotation loop bound format ici\n";
          }

          llvm::StringRef Kind;
          size_t Depth = 0;
          if (StrRefParts[0] == "lower") {
            Kind = StrRefParts[0];
            Depth = 0;
            StrRefParts[2].getAsInteger(10, Depth);
          } else if (StrRefParts[0] == "upper") {
            Kind = StrRefParts[0];
            Depth = 0;
            StrRefParts[2].getAsInteger(10, Depth);
          } else {
            errs() << "ExtractAnnotatedFromLoop pass on " << F.getName()
                   << " : Bad annotation loop bound format\n";
          }

          llvm::Metadata *BoundLoopKindMetadata =
              llvm::MDString::get(Ctx, Kind);
          llvm::Metadata *DepthLoopMetadata = llvm::ConstantAsMetadata::get(
              llvm::ConstantInt::get(llvm::Type::getInt32Ty(Ctx), Depth));
          llvm::MDNode *Node = llvm::MDNode::get(
              Ctx, {BoundLoopKindMetadata, DepthLoopMetadata});
          Inst->setMetadata("loop_bound_information", Node);
          Res = true;
        }
      }
    }
  }
  return Res;
}

bool moveInnerLoopLoad(Function &F) {
  bool Res = false;

  auto Lambda = [](Instruction *Inst, StringRef Name) {
    auto *LoadArray = dyn_cast<LoadInst>(Inst);
    if (not LoadArray)
      return false;

    Value *Ptr = LoadArray->getPointerOperand();
    auto *GEP = dyn_cast<GetElementPtrInst>(Ptr);
    if (!GEP)
      return false;

    if (GEP->getParent() == LoadArray->getParent())
      return false;

    IRBuilder<> Builder(GEP->getNextNode());
    auto *HoistedLoad = Builder.CreateLoad(LoadArray->getType(), GEP);
    HoistedLoad->setName(LoadArray->getName() + Name);

    LoadArray->replaceAllUsesWith(HoistedLoad);
    LoadArray->eraseFromParent();
    return true;
  };

  auto Annotations = extractArrayInfo(F);
  for (auto &[Inst, Data] : Annotations) {
    Res = Lambda(Inst, "hoistedArray");

    for (auto *Size : Data.Sizes) {
      Res = Lambda(Size, "hoistedDim");
    }
  }
  return Res;
}

} // namespace

void AnnotationData::print(raw_ostream &OS) const {
  if (Map.empty())
    OS << "Empty\n";
  for (const auto &[Key, Value] : Map) {
    OS << "Array : " << *Key << "\n";
    OS << "    Name : " << Value.Name << "\n";
    for (const auto *V : Value.Sizes) {
      OS << "    Size : " << *V << "\n";
    }
  }
}

AnalysisKey ExtractAnnotatedSizes::Key;
ExtractAnnotatedSizes::Result
ExtractAnnotatedSizes::run(Function &F, FunctionAnalysisManager &AM) {
  return extractArrayInfo(F);
}

char ExtractAnnotatedSizesWrapperPass::ID = 0;
bool ExtractAnnotatedSizesWrapperPass::runOnFunction(Function &F) {
  Anno = extractArrayInfo(F);
  return true;
}
void ExtractAnnotatedSizesWrapperPass::print(raw_ostream &OS,
                                             const Module *) const {
  Anno.print(OS);
}

PreservedAnalyses ExtractAnnotatedFromLoop::run(Function &F,
                                                FunctionAnalysisManager &FM) {
  if (not F.hasFnAttribute("polly.findSCoP"))
    return PreservedAnalyses::all();

  errs() << "ExtractAnnotatedFromLoop pass run on " << F.getName() << "\n";

  bool Changed = false;

  Changed = extractLoopBoundAnnotation(F);

  // auto &SE = FM.getResult<ScalarEvolutionAnalysis>(F);
  // auto &LI = FM.getResult<LoopAnalysis>(F);
  // Changed = extractAssumption(F, SE, LI);

  Changed = moveInnerLoopLoad(F);

  errs() << "ExtractAnnotatedFromLoop pass done\n";
  return Changed ? PreservedAnalyses::none() : PreservedAnalyses::all();
}
