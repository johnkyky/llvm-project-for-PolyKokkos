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

#include "polly/Test/LoopFusion.h"
#include "polly/Test/ExtractAnnotatedFromLoop.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/Analysis/AssumptionCache.h"
#include "llvm/IR/CFG.h"
#include "llvm/IR/Dominators.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/IntrinsicInst.h"
#include "llvm/IR/PassManager.h"
#include "llvm/Passes/PassBuilder.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include <algorithm>
#include <charconv>
#include <optional>
#include <regex>
#include <stack>
#include <string>
#include <utility>
#include <variant>
#include <vector>

using namespace llvm;
using namespace polly;

namespace {

BasicBlock *getExitBlock(Function &F) {
  SmallVector<BasicBlock *, 1> ExitBlocks;
  for (auto &BB : F) {
    auto *Term = BB.getTerminator();
    if (isa<ReturnInst>(Term))
      return &BB;
  }

  if (ExitBlocks.size() != 1)
    llvm_unreachable("Function does not have a single exit block");
  return ExitBlocks[0];
}

void implMoveBlock(Loop *L1, Loop *L2, BasicBlock *BlockToMove,
                   BasicBlock *InsertBeforeBlock) {
  auto ValidBlock = [&L1, &L2](auto Blocks) -> BasicBlock * {
    // Check if blocks have multi predecessor/successor blocks and chose the
    // outside loop block
    if (std::distance(Blocks.begin(), Blocks.end()) == 1) {
      // errs() << "Only one block found, returning it: " << **Blocks.begin()
      //        << "\n";
      return *Blocks.begin();
    }
    for (auto *Block : Blocks) {
      if (L1->contains(Block) || L2->contains(Block))
        continue;
      return Block;
    }
    return nullptr;
  };

  // errs() << "Moving block: " << *BlockToMove << "\n";
  BasicBlock *SrcPredBlock = ValidBlock(llvm::predecessors(BlockToMove));
  // errs() << "Source predecessor: " << *SrcPredBlock << "\n";
  BasicBlock *SrcNextBlock = ValidBlock(llvm::successors(BlockToMove));
  // errs() << "Source next: " << *SrcNextBlock << "\n";

  BasicBlock *DstPredBlock = ValidBlock(llvm::predecessors(InsertBeforeBlock));
  // errs() << "Destination predecessor: " << *DstPredBlock << "\n";
  BasicBlock *DstNextBlock = InsertBeforeBlock;
  // errs() << "Destination next: " << *DstNextBlock << "\n";

  // return;
  // DST
  DstPredBlock->getTerminator()->setSuccessor(0, BlockToMove);

  BlockToMove->getTerminator()->setSuccessor(0, DstNextBlock);

  for (Instruction &I : *DstNextBlock)
    if (auto *Phi = dyn_cast<PHINode>(&I))
      Phi->replaceIncomingBlockWith(DstPredBlock, BlockToMove);

  // SRC
  auto *InstBranch = SrcPredBlock->getTerminator();
  auto *Branch = dyn_cast<BranchInst>(InstBranch);
  bool IsConditional = Branch && Branch->isConditional();
  if (not IsConditional) {
    SrcPredBlock->getTerminator()->setSuccessor(0, SrcNextBlock);
  } else {
    BasicBlock *TrueBlock = Branch->getSuccessor(0);
    BasicBlock *FalseBlock = Branch->getSuccessor(1);

    if (not L1->contains(TrueBlock)) {
      Branch->setSuccessor(0, SrcNextBlock);
    } else if (L2->contains(FalseBlock)) {
      Branch->setSuccessor(1, SrcNextBlock);
    }
  }

  for (Instruction &I : *SrcNextBlock)
    if (auto *Phi = dyn_cast<PHINode>(&I))
      Phi->replaceIncomingBlockWith(BlockToMove, SrcPredBlock);
}

void moveBlockBetweenLoopsImpl(Loop *L1, Loop *L2) {
  BasicBlock *Start = L1->getExitBlock();
  BasicBlock *End = L2->getHeader();
  BasicBlock *InsertionPoint = L1->getHeader();
  std::vector<BasicBlock *> ToMove;

  // errs() << "Start: " << *Start << "\n";
  // errs() << "End: " << *End << "\n";
  // errs() << "InsertionPoint: " << *InsertionPoint << "\n";

  auto Size = [](llvm::succ_range R) {
    return std::distance(R.begin(), R.end());
  };

  auto Succ = llvm::successors(Start);
  ToMove.push_back(Start);

  if (Size(Succ) != 1)
    llvm_unreachable("Start block has multiple successors");

  while (true) {
    auto *Current = *Succ.begin();
    if (Current == End)
      break;

    ToMove.push_back(Current);
    Succ = llvm::successors(Current);

    if (size(Succ) != 1)
      llvm_unreachable("Intermediate block has multiple successors");
  }

  // for (auto *BB : ToMove) {
  //   errs() << "Block to move: " << *BB << "\n\n";
  // }

  for (auto *BB : ToMove) {
    if (L1->contains(BB) || L2->contains(BB))
      llvm_unreachable("Block to move is inside one of the loops");
    implMoveBlock(L1, L2, BB, InsertionPoint);
  }
}

bool moveBlockBetweenLoops(const SmallVector<Loop *, 2> &Loops) {
  if (Loops.size() < 2)
    return false;

  for (size_t I = Loops.size() - 1; I >= 1; I--) {
    Loop *L1 = Loops[I - 1];
    Loop *L2 = Loops[I];
    moveBlockBetweenLoopsImpl(L1, L2);
  }
  return true;
}

SmallVector<Loop *, 2> findLoop(Function &F, LoopInfo &LI, DominatorTree &DT) {
  std::set<Loop *> LoopsBetween;
  std::set<BasicBlock *> Visited;
  std::stack<BasicBlock *> Stack;
  Stack.push(&F.getEntryBlock());
  BasicBlock *ExitBlock = getExitBlock(F);

  while (!Stack.empty()) {
    BasicBlock *Curr = Stack.top();
    Stack.pop();

    if (!Visited.insert(Curr).second)
      continue;

    if (Loop *L = LI.getLoopFor(Curr)) {
      while (L->getParentLoop())
        L = L->getParentLoop();
      LoopsBetween.insert(L);
    }

    if (Curr == ExitBlock)
      continue;

    for (BasicBlock *Succ : successors(Curr))
      Stack.push(Succ);
  }

  auto LoopsVec =
      SmallVector<Loop *, 2>(LoopsBetween.begin(), LoopsBetween.end());
  std::sort(LoopsVec.begin(), LoopsVec.end(), [&](Loop *A, Loop *B) {
    return DT.dominates(A->getHeader(), B->getHeader());
  });

  return LoopsVec;
}

struct LoopBoundT {
  enum Type { Lower, Upper };

  Instruction *Inst = nullptr;
  Type BoundType = Type::Lower;
  size_t Depth = 0;
  Loop *L = nullptr;
  size_t IndexPolicy = 0;
};

SmallVector<LoopBoundT, 4>
getLoopBoundInstructions(Function &F, SmallVector<Loop *, 2> &Loops,
                         DominatorTree &DT) {
  SmallVector<LoopBoundT, 4> LoopBoundVec;

  // Read loop bound information from metadata
  for (auto &BB : F) {
    for (auto &Inst : BB) {
      auto *Node = Inst.getMetadata("loop_bound_information");
      if (Node) {
        LoopBoundT LB;
        LB.Inst = &Inst;

        if (Node->getNumOperands() >= 2) {
          if (llvm::MDString *KindMetadata =
                  llvm::dyn_cast<llvm::MDString>(Node->getOperand(0))) {
            auto BoundTypeStr = KindMetadata->getString();
            if (BoundTypeStr == "lower")
              LB.BoundType = LoopBoundT::Lower;
            else if (BoundTypeStr == "upper")
              LB.BoundType = LoopBoundT::Upper;
            else
              errs() << "Unknown loop bound kind: " << BoundTypeStr << "\n";
          }

          if (llvm::ConstantAsMetadata *CAM =
                  llvm::dyn_cast<llvm::ConstantAsMetadata>(
                      Node->getOperand(1))) {
            if (llvm::ConstantInt *CI =
                    llvm::dyn_cast<llvm::ConstantInt>(CAM->getValue())) {
              LB.Depth = CI->getZExtValue();
            }
          }
        }
        LoopBoundVec.push_back(std::move(LB));
      }
    }
  }

  // Sort LoopBoundVec by dominance relationship
  std::sort(LoopBoundVec.begin(), LoopBoundVec.end(),
            [&](const LoopBoundT &A, const LoopBoundT &B) {
              if (A.Inst->getParent() == B.Inst->getParent()) {
                for (Instruction &I : *(A.Inst->getParent())) {
                  if (&I == A.Inst)
                    return true;
                  if (&I == B.Inst)
                    return false;
                }
                errs() << "Instructions not found in their parent block!\n";
              }

              return DT.dominates(A.Inst, B.Inst);
            });

  // Assign Loop to each LoopBoundT based on dominance
  size_t IndexLoop = 0;
  for (size_t I = 0; I < LoopBoundVec.size(); ++I) {
    if (DT.dominates(LoopBoundVec[I].Inst->getParent(),
                     Loops[IndexLoop]->getHeader())) {
      LoopBoundVec[I].L = Loops[IndexLoop];
      LoopBoundVec[I].IndexPolicy = IndexLoop;
    } else {
      IndexLoop++;
      I--;
    }
  }

  return LoopBoundVec;
}

BasicBlock *getTrueCondition(BranchInst *Branch, const Instruction *LeftOp,
                             ICmpInst::Predicate Predica,
                             const Instruction *RightOp) {

  switch (Predica) {
  case ICmpInst::ICMP_ULT: {
    const auto *LeftMD = LeftOp->getMetadata("loop_bound_information");
    MDString *LeftMDStr = dyn_cast<MDString>(LeftMD->getOperand(0));
    const auto LeftBoundKind = LeftMDStr->getString();

    const auto *RightMD = RightOp->getMetadata("loop_bound_information");
    MDString *RightMDStr = dyn_cast<MDString>(RightMD->getOperand(0));
    const auto RightBoundKind = RightMDStr->getString();

    if (LeftBoundKind == "lower" && RightBoundKind == "upper") {
      return Branch->getSuccessor(0);
    }
    if (LeftBoundKind == "upper" && RightBoundKind == "lower") {
      return Branch->getSuccessor(1);
    }
    break;
  }
  default:
    return nullptr;
  }
  return nullptr;
}

void removeLoopBoundConditions(Function &F,
                               const SmallVector<LoopBoundT, 4> LoopBounds) {

  for (auto &BB : F) {
    auto *Term = BB.getTerminator();
    if (auto *Branch = dyn_cast<BranchInst>(Term)) {
      if (not Branch->isConditional())
        continue;

      Value *Cond = Branch->getCondition();

      if (auto *ICmp = dyn_cast<ICmpInst>(Cond)) {
        Value *LHS = ICmp->getOperand(0);
        Value *RHS = ICmp->getOperand(1);
        ICmpInst::Predicate Pred = ICmp->getPredicate();

        const auto *LHSInst = dyn_cast<Instruction>(LHS);
        const auto *RHSInst = dyn_cast<Instruction>(RHS);

        const auto *ItLeft =
            std::find_if(LoopBounds.begin(), LoopBounds.end(),
                         [=](LoopBoundT LB) { return LB.Inst == LHSInst; });
        const auto *ItRight =
            std::find_if(LoopBounds.begin(), LoopBounds.end(),
                         [=](LoopBoundT LB) { return LB.Inst == RHSInst; });

        if (not(ItLeft != LoopBounds.end() and ItRight != LoopBounds.end()))
          continue;

        auto *NextBB = getTrueCondition(Branch, LHSInst, Pred, RHSInst);

        BranchInst::Create(NextBB, Branch);
        Branch->eraseFromParent();
      }
    }
  }

  return;
}

bool fusionSameArrays(Function &F, ExtractAnnotatedSizes::Result &Anno,
                      DominatorTree &DT) {
  bool Res = false;

  for (auto &[InstArray, Data] : Anno) {
    auto *MDStr = MDString::get(InstArray->getContext(), Data.Name);
    MDNode *MD = MDNode::get(InstArray->getContext(), {MDStr});
    InstArray->setMetadata("name", MD);
    Res = true;
  }

  /////////

  using NameToInstructionsT =
      llvm::DenseMap<llvm::StringRef, std::vector<Instruction *>>;

  NameToInstructionsT NameToArray;

  // Fill map with instructions grouped by name
  for (const auto &[Inst, Data] : Anno) {
    NameToArray[Data.Name].push_back(Inst);
  }

  // Sort the map in each group by their dominance relationship
  for (auto &[Name, Arrays] : NameToArray) {
    std::sort(Arrays.begin(), Arrays.end(),
              [&](Instruction *A, Instruction *B) {
                if (A->getParent() == B->getParent()) {
                  for (Instruction &I : *A->getParent()) {
                    if (&I == A)
                      return true;
                    if (&I == B)
                      return false;
                  }
                  errs() << "Instructions not found in their parent block!\n";
                }

                return DT.dominates(A, B);
              });
  }

  // Replace all use of arrays and sizes with the first one in each group (first
  // by dominance)
  for (const auto &[Name, Arrays] : NameToArray) {
    if (Arrays.size() == 1)
      continue;

    auto &FirstArray = Arrays.front();
    auto &FirstArrayData = Anno.Map[FirstArray];

    for (size_t I = 1; I < Arrays.size(); ++I) {
      Arrays[I]->replaceAllUsesWith(FirstArray);
      // errs() << "Replacing " << *Arrays[I] << " with " << *FirstArray <<
      // "\n";

      auto &FirstArrayDataSizes = FirstArrayData.Sizes;
      auto &OtherArraySizes = Anno.Map.at(Arrays[I]).Sizes;
      for (size_t I = 0; I < FirstArrayDataSizes.size(); ++I) {
        // errs() << "Replacing size " << *OtherArraySizes[I] << " with "
        //        << *FirstArrayDataSizes[I] << "\n";
        OtherArraySizes[I]->replaceAllUsesWith(FirstArrayDataSizes[I]);
      }
    }
  }

  return Res;
}

} // namespace

namespace {

struct PolicyBound {
  std::string Policy;
  size_t PolicyIndex;
  std::string BoundType;
  size_t BoundIndex;

  // IR
  Loop *L = nullptr;
  Instruction *InstBound = nullptr;

  // If assumption set policy bound to a literal value
  // we need to remember it here
  bool IsLiteral = false;
  long LiteralValue = 0;

  bool operator==(const PolicyBound &Other) const {
    return Policy == Other.Policy && PolicyIndex == Other.PolicyIndex &&
           BoundType == Other.BoundType && BoundIndex == Other.BoundIndex;
  }
};
using Variable = std::string;
using Literal = long;

using Operand = std::variant<PolicyBound, Variable, Literal>;

struct Comparison {
  enum class Operator {
    UNKNOWN,
    EQUAL,
    NOT_EQUAL,
    LESS,
    GREATER,
    LESS_EQUAL,
    GREATER_EQUAL
  };
  Operand LHS;
  Operator Op;
  Operand RHS;
};

std::string operandToString(const Operand &Operand) {
  return std::visit(
      [](auto &&Arg) -> std::string {
        using T = std::decay_t<decltype(Arg)>;
        if constexpr (std::is_same_v<T, PolicyBound>) {
          return Arg.Policy + std::to_string(Arg.PolicyIndex) + "." +
                 Arg.BoundType + std::to_string(Arg.BoundIndex);
        } else if constexpr (std::is_same_v<T, Literal>) {
          return "int" + std::to_string(Arg);
        } else {
          return "variable" + Arg;
        }
      },
      Operand);
}

std::string operatorToString(const Comparison::Operator &Op) {
  switch (Op) {
  case Comparison::Operator::EQUAL:
    return "==";
  case Comparison::Operator::NOT_EQUAL:
    return "!=";
  case Comparison::Operator::LESS:
    return "<";
  case Comparison::Operator::GREATER:
    return ">";
  case Comparison::Operator::LESS_EQUAL:
    return "<=";
  case Comparison::Operator::GREATER_EQUAL:
    return ">=";
  case Comparison::Operator::UNKNOWN:
    return "??";
  }
}

Comparison::Operator stringToOperator(const std::string &OpStr) {
  if (OpStr == "==")
    return Comparison::Operator::EQUAL;
  if (OpStr == "!=")
    return Comparison::Operator::NOT_EQUAL;
  if (OpStr == "<")
    return Comparison::Operator::LESS;
  if (OpStr == ">")
    return Comparison::Operator::GREATER;
  if (OpStr == "<=")
    return Comparison::Operator::LESS_EQUAL;
  if (OpStr == ">=")
    return Comparison::Operator::GREATER_EQUAL;
  return Comparison::Operator::UNKNOWN;
}

std::string comparisonToString(const Comparison &C) {
  return operandToString(C.LHS) + " " + operatorToString(C.Op) + " " +
         operandToString(C.RHS);
}

std::optional<Operand> parseOperand(const std::string &S) {
  static const std::regex PolicyRegex(
      "(policy|p)(\\d+)\\.(lower|l|upper|u)(\\d+)");
  std::smatch Match;

  if (std::regex_match(S, Match, PolicyRegex)) {
    std::string BoundTypeStr = Match[3].str();

    if (BoundTypeStr == "l") {
      BoundTypeStr = "lower";
    } else if (BoundTypeStr == "u") {
      BoundTypeStr = "upper";
    }

    return PolicyBound{"policy", (size_t)std::stoul(Match[2].str()),
                       BoundTypeStr, (size_t)std::stoul(Match[4].str())};
  }

  long Val;
  const char *StartPtr = S.data();
  const char *EndPtr = S.data() + S.size();

  auto Result = std::from_chars(StartPtr, EndPtr, Val);

  if (Result.ec == std::errc() && Result.ptr == EndPtr) {
    return Val;
  }

  static const std::regex VarRegex("[a-zA-Z_][a-zA-Z0-9_]*");
  if (std::regex_match(S, VarRegex)) {
    return S;
  }

  return std::nullopt;
}

std::vector<Comparison> parseComparisons(const std::string &AssumptionsStr) {
  std::vector<Comparison> Results;

  static const std::regex ComparisonRegex(
      "\\s*([^,=\\!<>\\s]+)\\s*(==|!=|<=|>=|<|>)\\s*([^,\\s]+)\\s*");

  StringRef StrRef(AssumptionsStr);
  SmallVector<StringRef, 4> ComparisonStrs;
  StrRef.split(ComparisonStrs, ',');

  for (StringRef CompStr : ComparisonStrs) {
    if (CompStr.trim().empty())
      continue;

    std::string CompStdStr = CompStr.str();
    std::smatch Match;

    if (!std::regex_match(CompStdStr, Match, ComparisonRegex)) {
      report_fatal_error("Invalid assumption format: '" + Twine(CompStdStr) +
                         "'");
    }

    std::string LhsStr = Match[1].str();
    std::string OpStr = Match[2].str();
    std::string RhsStr = Match[3].str();

    std::optional<Operand> LhsOpt = parseOperand(LhsStr);
    if (!LhsOpt) {
      report_fatal_error("Left assumption operand invalid: '" + Twine(LhsStr) +
                         "'");
    } else if (not std::holds_alternative<PolicyBound>(LhsOpt.value())) {
      report_fatal_error("Left assumption must be a policy bound");
    }

    auto Op = stringToOperator(OpStr);
    if (Op == Comparison::Operator::UNKNOWN) {
      report_fatal_error("Invalid comparison operator: '" + Twine(OpStr) + "'");
    }

    std::optional<Operand> RhsOpt = parseOperand(RhsStr);
    if (!RhsOpt) {
      report_fatal_error("Right assumption operand invalid: '" + Twine(RhsStr) +
                         "'");
    }

    if (std::holds_alternative<PolicyBound>(RhsOpt.value())) {
      auto &LHSBound = std::get<PolicyBound>(LhsOpt.value());
      auto &RHSBound = std::get<PolicyBound>(RhsOpt.value());
      if (LHSBound.PolicyIndex == RHSBound.PolicyIndex and
          LHSBound.BoundType == RHSBound.BoundType and
          LHSBound.BoundIndex == RHSBound.BoundIndex) {
        report_fatal_error("Left and right assumption policy bound must be "
                           "different: '" +
                           Twine(CompStdStr) + "'");
      }
    }

    Comparison Comp;
    Comp.LHS = LhsOpt.value();
    Comp.Op = Op;
    Comp.RHS = RhsOpt.value();

    Results.push_back(Comp);
  }

  return Results;
}

StringRef
extractAssumptionAnnotation(Function &F, SmallVector<Loop *, 2> Loops,
                            SmallVector<LoopBoundT, 4> &LoopBoundVec) {
  StringRef AssumptionsStr;
  bool FindedAssumption = false;
  for (auto &BB : F) {
    if (FindedAssumption)
      break;

    for (auto &I : BB) {
      auto [CallInst, StrRef] = isAnnotationInstruction(&I, "assumption");
      if (CallInst) {
        Value *Op = CallInst->getOperand(0);
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
                      AssumptionsStr = Array->getAsCString();
                    }
                  }
                }
              }
            }
          }
        }
        FindedAssumption = true;
        break;
      }
    }
  }
  return AssumptionsStr;
}

std::vector<Comparison>
parseAssumptions(StringRef AssumptionsStr, SmallVector<Loop *, 2> &Loops,
                 SmallVector<LoopBoundT, 4> &LoopBoundVec) {

  auto ComparaisonVec = parseComparisons(AssumptionsStr.str());

  auto Lambda = [&Loops, &LoopBoundVec](auto &C) -> bool {
    if (std::holds_alternative<PolicyBound>(C)) {
      auto &HS = std::get<PolicyBound>(C);

      if (Loops.size() < HS.PolicyIndex) {
        return false;
      }

      HS.L = Loops[HS.PolicyIndex - 1];

      auto BoundType =
          HS.BoundType == "lower" ? LoopBoundT::Lower : LoopBoundT::Upper;

      auto It = std::find_if(
          LoopBoundVec.begin(), LoopBoundVec.end(), [&](const LoopBoundT &LB) {
            if (HS.L == LB.L and BoundType == LB.BoundType and
                (HS.BoundIndex - 1) == LB.Depth) {
              return true;
            }
            return false;
          });
      if (It != LoopBoundVec.end()) {
        HS.InstBound = It->Inst;
      } else {
        return false;
      }
    }
    return true;
  };

  for (auto C = ComparaisonVec.begin(); C != ComparaisonVec.end();) {
    if (not Lambda(C->LHS)) {
      errs() << "LHS assumption failed in : " << comparisonToString(*C) << "\n";
      C = ComparaisonVec.erase(C);
      continue;
    }
    if (not Lambda(C->RHS)) {
      errs() << "RHS assumption failed in : " << comparisonToString(*C) << "\n";
      C = ComparaisonVec.erase(C);
      continue;
    }
    C++;
  }

  return ComparaisonVec;

  // for (auto &BB : F) {
  //   for (auto &I : BB) {
  //     auto [CallInst, StrRef] = isAnnotationInstruction(&I, "var");
  //     if (CallInst) {
  //       Value *Op = CallInst->getOperand(0);
  //       errs() << "New var : " << *Op << " " << StrRef << "\n";
  //       auto *IndOP = dyn_cast<PHINode>(Op);
  //
  //       auto *Inst = dyn_cast<Instruction>(Op);
  //       if (not Inst)
  //         continue;
  //
  //       errs() << "getLoopUpperBound "
  //              << *getLoopUpperBound(Inst,
  //              LI.getLoopFor(Inst->getParent()), SE)
  //              << "\n";
  //     }
  //   }
  // }
}

void applyPolicyVsLiteral(Comparison &C, Function &F, AssumptionCache &AC,
                          IRBuilder<> &Builder,
                          std::vector<Comparison> &Assumptions);

void applyPolicyVsPolicy(Comparison &C, Function &F, AssumptionCache &AC,
                         IRBuilder<> &Builder,
                         std::vector<Comparison> &Assumptions) {
  auto &LHS = std::get<PolicyBound>(C.LHS);
  auto &RHS = std::get<PolicyBound>(C.RHS);

  if (LHS.IsLiteral and RHS.IsLiteral) {
    return;
  }

  if (LHS.IsLiteral) {
    std::swap(C.LHS, C.RHS);
    errs() << "On a un littéral à gauche, on swap\n";
    applyPolicyVsLiteral(C, F, AC, Builder, Assumptions);
    return;
  }
  if (RHS.IsLiteral) {
    applyPolicyVsLiteral(C, F, AC, Builder, Assumptions);
    return;
  }

  if (LHS.PolicyIndex > RHS.PolicyIndex)
    std::swap(LHS, RHS);

  auto *LHSInst = LHS.InstBound;
  auto *RHSInst = RHS.InstBound;

  if (!LHSInst || !RHSInst) {
    llvm_unreachable("Instruction de borne nulle, la liaison a échoué.");
  }

  switch (C.Op) {
  case Comparison::Operator::EQUAL: {
    errs() << "Replacing " << *RHSInst << " with " << *LHSInst << "\n";
    RHSInst->replaceAllUsesWith(LHSInst);

    // Update other assumptions that reference RHSInst to use LHSInst
    for (auto &OtherC : Assumptions) {
      if (&OtherC == &C)
        continue;
      bool Updated = false;
      if (std::holds_alternative<PolicyBound>(OtherC.LHS)) {
        auto &OtherLHS = std::get<PolicyBound>(OtherC.LHS);
        if (OtherLHS.InstBound == RHSInst) {
          OtherLHS.InstBound = LHSInst;
          Updated = true;
        }
      }
      if (std::holds_alternative<PolicyBound>(OtherC.RHS)) {
        auto &OtherRHS = std::get<PolicyBound>(OtherC.RHS);
        if (OtherRHS.InstBound == RHSInst) {
          OtherRHS.InstBound = LHSInst;
          Updated = true;
        }
      }
      if (Updated) {
        errs() << "Updated related assumption: " << comparisonToString(OtherC)
               << "\n";
      }
    }
    break;
  }
  case Comparison::Operator::NOT_EQUAL:
  case Comparison::Operator::LESS:
  case Comparison::Operator::GREATER:
  case Comparison::Operator::LESS_EQUAL:
  case Comparison::Operator::GREATER_EQUAL: {
    CmpInst::Predicate Pred = CmpInst::Predicate::ICMP_EQ; // Init
    if (C.Op == Comparison::Operator::NOT_EQUAL)
      Pred = CmpInst::Predicate::ICMP_NE;
    if (C.Op == Comparison::Operator::LESS)
      Pred = CmpInst::Predicate::ICMP_SLT; // Signed Less Than
    if (C.Op == Comparison::Operator::GREATER)
      Pred = CmpInst::Predicate::ICMP_SGT;
    if (C.Op == Comparison::Operator::LESS_EQUAL)
      Pred = CmpInst::Predicate::ICMP_SLE;
    if (C.Op == Comparison::Operator::GREATER_EQUAL)
      Pred = CmpInst::Predicate::ICMP_SGE;

    Value *Cmp = Builder.CreateICmp(Pred, LHSInst, RHSInst);
    errs() << "Registering assumption: " << *Cmp << "\n";

    AC.registerAssumption(cast<AssumeInst>(Cmp));
    break;
  }
  default:
    llvm_unreachable("Opérateur inconnu dans une comparaison validée.");
  }
}

void applyPolicyVsLiteral(Comparison &C, Function &F, AssumptionCache &AC,
                          IRBuilder<> &Builder,
                          std::vector<Comparison> &Assumptions) {
  auto &LHS = std::get<PolicyBound>(C.LHS);
  auto &RHS = std::get<Literal>(C.RHS);

  auto *BoundInst = LHS.InstBound;
  if (!BoundInst)
    llvm_unreachable("Instruction de borne nulle.");

  Constant *ConstVal = ConstantInt::get(BoundInst->getType(), RHS);

  switch (C.Op) {
  case Comparison::Operator::EQUAL: {
    errs() << "Replacing " << *BoundInst << " with constant " << *ConstVal
           << "\n";
    BoundInst->replaceAllUsesWith(ConstVal);
    LHS.IsLiteral = true;
    LHS.LiteralValue = RHS;

    // Update other assumptions that reference BoundInst to use ConstVal
    auto Begin = std::find_if(
        Assumptions.begin(), Assumptions.end(),
        [&C](const Comparison &Assumption) { return &Assumption == &C; });
    if (Begin != Assumptions.end())
      Begin++;
    for (auto It = Begin; It != Assumptions.end(); ++It) {
      auto &OtherC = *It;
      bool Updated = false;
      if (std::holds_alternative<PolicyBound>(OtherC.LHS) and
          std::holds_alternative<PolicyBound>(OtherC.RHS)) {
        auto &OtherLHS = std::get<PolicyBound>(OtherC.LHS);
        if (OtherLHS == LHS) {
          OtherC.LHS = RHS;
          std::swap(OtherC.LHS, OtherC.RHS);
          Updated = true;
        }
      }
      if (std::holds_alternative<PolicyBound>(OtherC.RHS)) {
        auto &OtherRHS = std::get<PolicyBound>(OtherC.RHS);
        if (OtherRHS == LHS) {
          OtherC.RHS = RHS;
          Updated = true;
        }
      }
      if (Updated) {
        errs() << "Updated related assumption: " << comparisonToString(OtherC)
               << "\n";
      }
    }

    break;
  }
  case Comparison::Operator::NOT_EQUAL:
  case Comparison::Operator::LESS:
  case Comparison::Operator::GREATER:
  case Comparison::Operator::LESS_EQUAL:
  case Comparison::Operator::GREATER_EQUAL: {
    CmpInst::Predicate Pred = CmpInst::Predicate::ICMP_EQ;
    if (C.Op == Comparison::Operator::NOT_EQUAL)
      Pred = CmpInst::Predicate::ICMP_NE;
    if (C.Op == Comparison::Operator::LESS)
      Pred = CmpInst::Predicate::ICMP_SLT;
    if (C.Op == Comparison::Operator::GREATER)
      Pred = CmpInst::Predicate::ICMP_SGT;
    if (C.Op == Comparison::Operator::LESS_EQUAL)
      Pred = CmpInst::Predicate::ICMP_SLE;
    if (C.Op == Comparison::Operator::GREATER_EQUAL)
      Pred = CmpInst::Predicate::ICMP_SGE;

    Value *Cmp = Builder.CreateICmp(Pred, BoundInst, ConstVal);
    Value *Assumption = Builder.CreateAssumption(Cmp);

    // AC.registerAssumption(cast<AssumeInst>(Assumption));
    // errs() << "size " << AC.assumptions().size() << "\n";
    errs() << "Registering assumption: " << *Assumption << "\n";
    break;
  }
  default:
    llvm_unreachable("Opérateur inconnu dans une comparaison validée.");
  }
}

void applyAssumptions(Function &F, std::vector<Comparison> &Assumptions,
                      AssumptionCache &AC) {
  IRBuilder<> Builder(&F.getEntryBlock(), F.getEntryBlock().begin());

  for (auto &Assumption : Assumptions) {
    errs() << "\n\nApplying assumption: " << comparisonToString(Assumption)
           << "\n";

    std::visit(
        [&](auto &&LHS, auto &&RHS) {
          using T1 = std::decay_t<decltype(LHS)>;
          using T2 = std::decay_t<decltype(RHS)>;

          if constexpr (std::is_same_v<T1, PolicyBound> &&
                        std::is_same_v<T2, PolicyBound>) {
            applyPolicyVsPolicy(Assumption, F, AC, Builder, Assumptions);
          } else if constexpr (std::is_same_v<T1, PolicyBound> &&
                               std::is_same_v<T2, Literal>) {
            applyPolicyVsLiteral(Assumption, F, AC, Builder, Assumptions);
          } else {
            llvm_unreachable("Other combinations are not supported yet.");
          }
        },
        Assumption.LHS, Assumption.RHS);
  }
}

} // namespace

PreservedAnalyses LoopFusionPass::run(Function &F,
                                      FunctionAnalysisManager &AM) {
  if (not F.hasFnAttribute("polly.multi_parallel_for"))
    return PreservedAnalyses::all();

  errs() << "LoopFusionPass pass run on " << F.getName() << "\n";

  auto &DT = AM.getResult<DominatorTreeAnalysis>(F);
  auto Loops = findLoop(F, AM.getResult<LoopAnalysis>(F), DT);

  // for (auto &L : Loops) {
  //   errs() << "Loop found: " << *L << "\n";
  // }

  auto LoopBoundVec = getLoopBoundInstructions(F, Loops, DT);

  // for (const auto LoopBound : LoopBoundVec) {
  //   errs() << "Instruction de borne de boucle : " << *LoopBound.Inst << "   "
  //          << (LoopBound.BoundType == LoopBoundT::Lower ? "Lower" : "Upper")
  //          << "   " << LoopBound.Depth << "   "
  //          << (LoopBound.L ? "boucle" : "pas boucle") << "   "
  //          << LoopBound.IndexPolicy << "\n";
  // }

  removeLoopBoundConditions(F, LoopBoundVec);

  moveBlockBetweenLoops(Loops);

  DT.recalculate(F);
  fusionSameArrays(F, AM.getResult<ExtractAnnotatedSizes>(F), DT);

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////

  auto AssumptionsStr = extractAssumptionAnnotation(F, Loops, LoopBoundVec);
  auto Assumptions = parseAssumptions(AssumptionsStr, Loops, LoopBoundVec);

  errs() << "Contenu de la chaîne : \n";
  for (const auto &Cmp : Assumptions)
    errs() << operandToString(Cmp.LHS) << " " << operatorToString(Cmp.Op) << " "
           << operandToString(Cmp.RHS) << "\n";

  auto &AC = AM.getResult<AssumptionAnalysis>(F);
  applyAssumptions(F, Assumptions, AC);

  errs() << "LoopFusionPass pass done\n";

  return PreservedAnalyses::none();
}
