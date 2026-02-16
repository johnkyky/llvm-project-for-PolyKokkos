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

#include "polly/Test/ArrayReg2Mem.h"
#include "llvm/Analysis/AliasAnalysis.h"
#include "llvm/Analysis/ScalarEvolution.h"
#include "llvm/Analysis/ScalarEvolutionExpressions.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/Dominators.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/Instruction.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/PassManager.h"
#include "llvm/IR/Value.h"
#include "llvm/IR/Verifier.h"
#include "llvm/Passes/PassBuilder.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Transforms/Utils/Local.h"
#include <algorithm>

#define DEBUG_TYPE "polly-array-reg2mem"

using namespace llvm;
using namespace polly;

namespace {

/* Find a store instruction in the given basic block (BB) that stores the
   given value (Val) to a pointer that is invariant with respect to the loop
   (i.e., the pointer is valid at the beginning of the loop, LoopHeader).
   If such a store is found, return the pointer operand of the store.
   Otherwise, return nullptr.
*/
Value *findInvariantStorePointer(Value *Val, BasicBlock *BB, DominatorTree &DT,
                                 BasicBlock *LoopHeader) {

  for (Instruction &I : llvm::reverse(*BB)) {

    if (auto *Store = dyn_cast<StoreInst>(&I)) {
      if (Store->getValueOperand() == Val) {

        Value *Ptr = Store->getPointerOperand();

        bool IsInvariant = true;
        if (auto *InstPtr = dyn_cast<Instruction>(Ptr)) {
          if (!DT.dominates(InstPtr, LoopHeader)) {
            IsInvariant = false;
          }
        }

        if (IsInvariant)
          return Ptr;
      }
    }
  }
  return nullptr;
}

bool isInvariantPtr(Value *Ptr, DominatorTree &DT, BasicBlock *LoopHeader) {
  if (isa<Argument>(Ptr))
    return true;

  if (isa<Constant>(Ptr))
    return true;

  if (auto *InstPtr = dyn_cast<Instruction>(Ptr)) {
    if (DT.dominates(InstPtr, LoopHeader))
      return true;

    if (auto *Load = dyn_cast<LoadInst>(InstPtr)) {
      Value *LoadAddr = Load->getPointerOperand();

      Value *Current = LoadAddr;
      while (Current) {
        if (isa<Argument>(Current))
          return true;

        if (auto *GEP = dyn_cast<GetElementPtrInst>(Current)) {
          Current = GEP->getPointerOperand();
          continue;
        }

        if (auto *Cast = dyn_cast<CastInst>(Current)) {
          Current = Cast->getOperand(0);
          continue;
        }

        break;
      }
    }
  }
  return false;
}

/* Check if the given PHI node (Phi) has a relation to stores in its
   incoming blocks that would allow us to demote it to memory.
   If such a relation is found, return the pointer operand of the store.
   Otherwise, return nullptr.
*/
Value *checkPhiStoreRelation(PHINode *Phi, DominatorTree &DT,
                             ScalarEvolution &SE) {
  if (Phi->getNumIncomingValues() != 2)
    return nullptr;

  BasicBlock *Header = Phi->getParent();

  Value *InVals[2] = {nullptr, nullptr};
  BasicBlock *InBlocks[2] = {nullptr, nullptr};
  Value *StorePtrs[2] = {nullptr, nullptr};

  for (int I = 0; I < 2; ++I) {
    InVals[I] = Phi->getIncomingValue(I);
    InBlocks[I] = Phi->getIncomingBlock(I);

    Value *FoundPtr =
        findInvariantStorePointer(InVals[I], InBlocks[I], DT, Header);

    if (!FoundPtr) {
      if (auto *Load = dyn_cast<LoadInst>(InVals[I])) {
        Value *PtrOperand = Load->getPointerOperand();

        // On vérifie que ce pointeur est valide pour toute la boucle
        if (isInvariantPtr(PtrOperand, DT, Header)) {
          FoundPtr = PtrOperand;
          errs().indent(6) << "-> Found Load from Pointer: " << *FoundPtr
                           << "\n";
        }
      }
    }

    StorePtrs[I] = FoundPtr;
  }

  for (int I = 0; I < 2; ++I) {
    errs().indent(4) << "PHI Incoming Value " << I << ": " << *InVals[I]
                     << " from Block: " << InBlocks[I]->getName() << "\n";
    if (StorePtrs[I]) {
      errs().indent(6) << "-> Found Store to Pointer: " << *StorePtrs[I]
                       << "\n";
    } else {
      errs().indent(6) << "-> No Store found for this incoming value.\n";
    }
  }

  if (StorePtrs[0] && StorePtrs[1] && StorePtrs[0] == StorePtrs[1]) {
    errs().indent(4) << "Match: Both branches use the same pointer.\n";
    return StorePtrs[0];
  }

  if (isa<Constant>(InVals[0]) && StorePtrs[1]) {
    errs().indent(4)
        << "Found Constant entry and Pointer in other branch. Reusing: "
        << *StorePtrs[1] << "\n";
    return StorePtrs[1];
  }

  if (isa<Constant>(InVals[1]) && StorePtrs[0]) {
    errs().indent(4)
        << "Found Constant entry and Pointer in other branch. Reusing: "
        << *StorePtrs[0] << "\n";
    return StorePtrs[0];
  }

  for (int I = 0; I < 2; ++I) {
    if (auto *Load = dyn_cast<LoadInst>(InVals[I])) {
      errs().indent(6) << "Incoming value " << I << " is a load: " << *Load
                       << "\n";
      Value *LoadPtr = Load->getPointerOperand();
      int Other = 1 - I;
      if (StorePtrs[Other] && StorePtrs[Other] == LoadPtr) {
        return LoadPtr;
      }
    }
  }

  errs().indent(4) << "No suitable store relation found for PHI.\n";
  return nullptr;
}

/* Materialize PHI Node into memory (Alloca + Stores + Load)
   with proper handling of LCSSA uses outside the loop.
*/
void materializePhi(PHINode *Phi, DominatorTree &DT, LoopInfo &LI) {
  Function *F = Phi->getFunction();
  IRBuilder<> BuilderEntry(&F->getEntryBlock(), F->getEntryBlock().begin());
  AllocaInst *Alloca = BuilderEntry.CreateAlloca(Phi->getType(), nullptr,
                                                 Phi->getName() + ".addr");

  errs() << "Materializing PHI: " << *Phi
         << " into memory with alloca: " << *Alloca << "\n";

  for (unsigned I = 0; I < Phi->getNumIncomingValues(); ++I) {
    errs() << "Processing incoming value " << I << ": "
           << *Phi->getIncomingValue(I)
           << " from block: " << Phi->getIncomingBlock(I)->getName() << "\n";
    Value *Val = Phi->getIncomingValue(I);
    BasicBlock *IncomingBB = Phi->getIncomingBlock(I);

    Instruction *TI = IncomingBB->getTerminator();
    IRBuilder<> Builder(TI);
    auto *S = Builder.CreateStore(Val, Alloca);
    errs() << "Inserted store of " << *Val << " to " << *Alloca << " in " << *S
           << "\n";
  }

  Instruction *IncomingInst0 =
      dyn_cast_or_null<Instruction>(Phi->getIncomingValue(0));
  Instruction *IncomingInst1 =
      dyn_cast_or_null<Instruction>(Phi->getIncomingValue(1));
  Value *AccumulatorResult = nullptr;

  if (not IncomingInst0 and not IncomingInst1) {
    llvm_unreachable(
        "Expected both incoming values to be instructions for PHI");
  } else if (IncomingInst0 and not IncomingInst1) {
    AccumulatorResult = IncomingInst0;
  } else if (not IncomingInst0 and IncomingInst1) {
    AccumulatorResult = IncomingInst1;
  } else {
    AccumulatorResult = DT.dominates(IncomingInst0, IncomingInst1)
                            ? Phi->getIncomingValue(1)
                            : Phi->getIncomingValue(0);
  }

  assert(AccumulatorResult &&
         "Expected at least one non-constant incoming value for PHI");

  IRBuilder<> BuilderLoad(Phi->getParent(),
                          Phi->getParent()->getFirstInsertionPt());
  LoadInst *Load = BuilderLoad.CreateLoad(Phi->getType(), Alloca,
                                          Phi->getName() + ".reload");

  Phi->replaceAllUsesWith(Load);

  for (unsigned I = 0; I < Phi->getNumIncomingValues(); ++I) {
    Value *Val = Phi->getIncomingValue(I);
    Instruction *ValInst = dyn_cast_or_null<Instruction>(Val);

    BasicBlock *IncomingBB = Phi->getParent();
    errs() << "IncomingBB => " << IncomingBB->getName() << "\n";

    if (ValInst) {
      for (auto *U : ValInst->users()) {
        errs() << "Checking use: " << *U << "\n";
        auto *UserInst = dyn_cast<Instruction>(U);
        if (not UserInst)
          continue;
        if (UserInst->getParent() != IncomingBB) {
          errs().indent(6) << "Use in different block: " << *UserInst << "\n";
          errs().indent(20)
              << "BB -> " << UserInst->getParent()->getName() << "\n";
          IRBuilder<> Builder(UserInst->getParent(),
                              UserInst->getParent()->getFirstInsertionPt());
          auto *L = Builder.CreateLoad(ValInst->getType(), Alloca,
                                       Val->getName() + ".reloadoutside");
          UserInst->replaceUsesOfWith(AccumulatorResult, L);
          if (AccumulatorResult)
            errs() << "AccumulatorResult: " << *AccumulatorResult << "\n";
          else
            errs() << "AccumulatorResult: nullptr\n";
        }
      }
    }
  }
}

void demoteNonIndexPhis(Function &F, LoopInfo &LI, ScalarEvolution &SE,
                        DominatorTree &DT) {

  SmallVector<PHINode *, 16> PhisToDemote;

  for (auto &BB : F) {
    Loop *L = LI.getLoopFor(&BB);
    if (!L || L->getHeader() != &BB)
      continue;

    for (auto &I : BB) {
      auto *Phi = dyn_cast<PHINode>(&I);
      if (!Phi)
        continue;

      if (SE.isSCEVable(Phi->getType())) {
        const auto *S = SE.getSCEV(Phi);
        if (auto *AR = dyn_cast<SCEVAddRecExpr>(S)) {
          if (AR->getLoop() == L)
            continue;
        }
      }
      PhisToDemote.push_back(Phi);
    }
  }

  errs() << "Found " << PhisToDemote.size() << " PHIs to demote in function "
         << F.getName() << "\n";
  for (auto *Phi : PhisToDemote) {
    errs().indent(2) << "PHI to demote: " << *Phi << "\n";
  }
  errs() << "\n\n";

  SmallVector<PHINode *, 0> DeadPhis;
  for (auto *Phi : PhisToDemote) {
    errs() << "Processing PHI: " << *Phi << "\n";

    Value *Ptr = checkPhiStoreRelation(Phi, DT, SE);

    if (Ptr) {
      errs().indent(2) << "Found Existing Pointer for PHI: " << *Ptr << "\n";

      for (unsigned I = 0; I < Phi->getNumIncomingValues(); ++I) {
        Value *InVal = Phi->getIncomingValue(I);
        BasicBlock *InBlock = Phi->getIncomingBlock(I);

        bool AlreadyInMemory = false;
        if (auto *InLoad = dyn_cast<LoadInst>(InVal)) {
          if (InLoad->getPointerOperand() == Ptr) {
            AlreadyInMemory = true;
          }
        }

        if (!AlreadyInMemory) {
          Instruction *Terminator = InBlock->getTerminator();
          IRBuilder<> BuilderStore(Terminator);
          BuilderStore.CreateStore(InVal, Ptr);

          errs().indent(4) << "Inserted Store in block " << InBlock->getName()
                           << "\n";
        } else {
          errs().indent(4)
              << "Incoming value already loaded from pointer in block "
              << InBlock->getName() << "\n";
        }
      }

      IRBuilder<> BuilderLoad(Phi->getParent(),
                              Phi->getParent()->getFirstInsertionPt());

      LoadInst *NewLoad = BuilderLoad.CreateLoad(Phi->getType(), Ptr,
                                                 Phi->getName() + ".demoted");

      Phi->replaceAllUsesWith(NewLoad);

      DeadPhis.push_back(Phi);

    } else {
      errs().indent(2) << "Materializing PHI to New Alloca: " << *Phi << "\n";
      materializePhi(Phi, DT, LI);
      DeadPhis.push_back(Phi);
    }
    errs() << "\n\n";
  }

  for (auto *Phi : DeadPhis) {
    Phi->eraseFromParent();
  }
}

SmallVector<Instruction *, 2> getRealUsesInBlock(Instruction *Val,
                                                 BasicBlock *BB) {
  if (isa<Constant>(Val)) {
    return {};
  }

  SmallVector<Instruction *, 2> Res;
  for (auto *U : Val->users()) {
    if (Instruction *Inst = dyn_cast<Instruction>(U)) {
      if (Inst and Inst->getParent() == BB and isa<StoreInst>(Inst)) {
        Res.push_back(Inst);
      }
    }
  }
  return Res;
}

/* Demote loads that have multiple uses within the same basic block by
   cloning the load for each use. This is useful to enable optimizations like
   those in Polly that may not handle multiple uses of the same load within a
   basic block.
*/
void demoteUsesOfArrays(Function &F, LoopInfo &LI, ScalarEvolution &SE,
                        DominatorTree &DT) {
  errs() << "Demoting uses of arrays\n";
  for (auto &BB : F) {
    if (!LI.getLoopFor(&BB))
      continue;

    SmallVector<LoadInst *, 16> LoadsToDemote;
    SmallVector<Instruction *, 16> MultiStoreValuesToDemote;

    for (auto &I : BB) {
      if (auto *Load = dyn_cast<LoadInst>(&I)) {
        BasicBlock *LoadBB = Load->getParent();
        if (Load->getNumUses() > 1) {
          bool NeedsDemotion = true;
          for (auto *U : Load->users()) {
            Instruction *Inst = dyn_cast<Instruction>(U);
            BasicBlock *InstBB = Inst->getParent();

            if (InstBB != LoadBB) {
              NeedsDemotion = false;
              break;
            }
          }

          if (NeedsDemotion) {
            LoadsToDemote.push_back(Load);
          }
        }
      } else if (auto *Store = dyn_cast<StoreInst>(&I)) {
        BasicBlock *StoreBB = Store->getParent();

        Value *Val = Store->getValueOperand();
        Instruction *ValInst = dyn_cast_or_null<Instruction>(Val);
        if (not ValInst)
          continue;

        auto Uses = getRealUsesInBlock(ValInst, StoreBB);
        if (Uses.size() > 1) {
          MultiStoreValuesToDemote.push_back(ValInst);
          errs() << "on doit mettre aussi " << *Val << " avec "
                 << Val->getNumUses() << " uses.\n";
          for (auto *U : Uses) {
            errs().indent(4) << "Use: " << *U << "\n";
          }
        }
      }
    }

    for (LoadInst *Load : LoadsToDemote) {
      errs().indent(2) << "Demoting Load: " << *Load << " with "
                       << Load->getNumUses() << " uses.\n";

      SmallVector<User *, 8> Users;
      SmallPtrSet<User *, 8> Seen;
      for (auto *U : Load->users()) {
        if (Seen.insert(U).second) {
          Users.push_back(U);
        }
      }

      std::sort(Users.begin(), Users.end(), [](User *A, User *B) {
        auto *InstA = cast<Instruction>(A);
        auto *InstB = cast<Instruction>(B);
        return InstA->comesBefore(InstB);
      });

      for (size_t I = 1; I < Users.size(); ++I) {
        auto *CurrentUser = Users[I];
        Instruction *CurrentUserInst = dyn_cast<Instruction>(CurrentUser);

        Instruction *NewLoad = Load->clone();

        if (Load->hasName())
          NewLoad->setName(Load->getName() + ".split." + Twine(I));

        NewLoad->insertAfter(CurrentUserInst->getPrevNode());

        CurrentUser->replaceUsesOfWith(Load, NewLoad);
      }
    }

    SmallVector<Instruction *, 8> SeenMultiStoreValues;
    for (Instruction *Inst : MultiStoreValuesToDemote) {
      if (std::find(SeenMultiStoreValues.begin(), SeenMultiStoreValues.end(),
                    Inst) != SeenMultiStoreValues.end()) {
        errs() << "Already demoted value: " << *Inst << "\n";
        continue;
      }
      errs() << "Demoting value with multiple stores: " << *Inst << " with "
             << getRealUsesInBlock(Inst, Inst->getParent()).size()
             << " uses.\n";

      auto Uses = getRealUsesInBlock(Inst, Inst->getParent());

      std::sort(Uses.begin(), Uses.end(), [](Instruction *A, Instruction *B) {
        return A->comesBefore(B);
      });

      for (auto &U : Uses) {
        errs() << "Use: " << *U << "\n";
      }

      StoreInst *FirstStore = dyn_cast<StoreInst>(Uses[0]);
      auto *Ptr = FirstStore->getPointerOperand();
      for (size_t I = 1; I < Uses.size(); ++I) {
        auto *CurrentStore = dyn_cast<StoreInst>(Uses[I]);

        IRBuilder<> Builder(CurrentStore);
        auto *NewLoad =
            Builder.CreateLoad(CurrentStore->getValueOperand()->getType(), Ptr,
                               Inst->getName() + ".reload_bite");

        CurrentStore->setOperand(0, NewLoad);
      }
      SeenMultiStoreValues.push_back(Inst);
    }
    errs() << "\n\n";
  }
}

bool sinkInstructionToFirstUse(LoadInst *Inst, AAResults &AA) {
  if (!Inst || Inst->use_empty())
    return false;

  if (Inst->mayHaveSideEffects()) {
    return false;
  }

  BasicBlock *BB = Inst->getParent();

  Instruction *TargetPosition = BB->getTerminator();

  BasicBlock::iterator It(Inst);
  ++It;

  for (; It != BB->end(); ++It) {
    Instruction *Curr = &*It;

    bool IsUser = false;
    for (Value *Op : Curr->operands()) {
      if (Op == Inst) {
        IsUser = true;
        break;
      }
    }

    if (IsUser) {
      TargetPosition = Curr;
      break;
    }

    if (Curr->mayWriteToMemory()) {
      MemoryLocation LoadLoc = MemoryLocation::get(Inst);

      if (auto *Store = dyn_cast<StoreInst>(Curr)) {
        MemoryLocation StoreLoc = MemoryLocation::get(Store);
        if (AA.alias(LoadLoc, StoreLoc) != AliasResult::NoAlias) {
          errs() << "Blocked by Store alias: " << *Store << "\n";
          TargetPosition = Curr; // On s'arrête juste devant ce mur
          break;
        }
      } else if (Curr->mayReadOrWriteMemory()) {
        if (isModSet(AA.getModRefInfo(Curr, LoadLoc))) {
          errs() << "Blocked by ModRef (Call/Fence): " << *Curr << "\n";
          TargetPosition = Curr;
          break;
        }
      }
    }
  }

  if (Inst->getNextNode() != TargetPosition) {
    errs() << "Moving Load: " << *Inst << " \n     before: " << *TargetPosition
           << "\n";
    Inst->moveBefore(TargetPosition->getIterator());
    return true;
  }

  return false;
}

void sinkInstructionsToFirstUse(Function &F, AAResults &AA, LoopInfo &LI) {
  for (auto &BB : F) {
    if (not LI.getLoopFor(&BB))
      continue;
    for (auto &I : BB) {
      if (auto *Load = dyn_cast<LoadInst>(&I)) {
        sinkInstructionToFirstUse(Load, AA);
      }
    }
  }
}

} // namespace

PreservedAnalyses ArrayReg2MemPass::run(Function &F,
                                        FunctionAnalysisManager &AM) {
  if (not F.hasFnAttribute("polly.findSCoP")) {
    return PreservedAnalyses::all();
  }

  LLVM_DEBUG(errs() << "ArrayReg2MemPass pass run on " << F.getName() << "\n");

  auto &LI = AM.getResult<LoopAnalysis>(F);
  auto &SE = AM.getResult<ScalarEvolutionAnalysis>(F);
  auto &DT = AM.getResult<DominatorTreeAnalysis>(F);
  demoteNonIndexPhis(F, LI, SE, DT);

  demoteUsesOfArrays(F, LI, SE, DT);

  auto &AA = AM.getResult<AAManager>(F);

  sinkInstructionsToFirstUse(F, AA, LI);

  if (verifyFunction(F, &errs())) {
    report_fatal_error("IR verification failed.");
  }

  LLVM_DEBUG(errs() << "ArrayReg2MemPass pass done\n");

  return PreservedAnalyses::none();
}
