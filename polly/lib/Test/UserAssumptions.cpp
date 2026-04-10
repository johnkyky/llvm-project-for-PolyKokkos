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

#include "polly/Test/UserAssumptions.h"
#include "polly/Test/ExtractAnnotatedFromLoop.h"
#include "polly/Test/LoopFusion.h"
#include "polly/Test/VarFusion.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/Metadata.h"
#include "llvm/IR/Verifier.h"
#include "llvm/Passes/PassBuilder.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include <charconv>
#include <optional>
#include <regex>

#define DEBUG_TYPE "polly-user-assumptions"

using namespace llvm;
using namespace polly;

namespace {

struct BoundBase {
  enum BoundKind { BK_PolicyBound, BK_Variable };

private:
  BoundKind Kind;

public:
  BoundBase(BoundKind K) : Kind(K) {}
  virtual ~BoundBase() = default;

  virtual bool operator==(const BoundBase &Other) const = 0;

  Instruction *Inst = nullptr;
  bool IsLiteral = false;
  long LiteralValue = 0;

  BoundKind getKind() const { return Kind; }
};

struct PolicyBound : public BoundBase {
  PolicyBound() : BoundBase(BK_PolicyBound) {}
  PolicyBound(const std::string &PolicyStr, size_t PolicyIndex,
              bool HasBoundType, std::string BoundType, bool HasBoundIndex,
              size_t BoundIndex)
      : BoundBase(BK_PolicyBound), PolicyStr(PolicyStr),
        PolicyIndex(PolicyIndex), HasBoundType(HasBoundType),
        BoundType(BoundType), HasBoundIndex(HasBoundIndex),
        BoundIndex(BoundIndex) {}

  static bool classof(const BoundBase *B) {
    return B->getKind() == BK_PolicyBound;
  }

  std::string PolicyStr;
  size_t PolicyIndex = 0;

  bool HasBoundType = false;
  std::string BoundType;

  bool HasBoundIndex = false;
  size_t BoundIndex = 0;

  bool operator==(const BoundBase &Other) const override {
    const auto *Derived = dyn_cast<const PolicyBound>(&Other);

    if (!Derived)
      return false;

    return PolicyStr == Derived->PolicyStr &&
           PolicyIndex == Derived->PolicyIndex &&
           BoundType == Derived->BoundType && BoundIndex == Derived->BoundIndex;
  }
};

struct Variable : public BoundBase {
  Variable() : BoundBase(BK_Variable) {}
  Variable(const std::string &N) : BoundBase(BK_Variable), Name(N) {}

  static bool classof(const BoundBase *B) {
    return B->getKind() == BK_Variable;
  }

  std::string Name;

  bool operator==(const BoundBase &Other) const override {
    const auto *Derived = dyn_cast<const Variable>(&Other);

    if (!Derived)
      return false;

    return Name == Derived->Name && IsLiteral == Derived->IsLiteral &&
           LiteralValue == Derived->LiteralValue;
  }
};
// struct BoundBase {
//
//   Instruction *Inst = nullptr;
//
//   bool IsLiteral = false;
//   long LiteralValue = 0;
//
//   virtual bool operator==(const BoundBase &Other) = 0 const;
// };
//
// struct PolicyBound : BoundBase {
//   std::string Policy;
//   size_t PolicyIndex;
//
//   bool HasBoundType = false;
//   std::string BoundType;
//
//   bool HasBoundIndex = false;
//   size_t BoundIndex;
//
//   // IR
//   Instruction *Inst = nullptr;
//
//   // If assumption set policy bound to a literal value
//   // we need to remember it here
//   bool IsLiteral = false;
//   long LiteralValue = 0;
//
//   bool operator==(const PolicyBound &Other) const {
//     return Policy == Other.Policy && PolicyIndex == Other.PolicyIndex &&
//            BoundType == Other.BoundType && BoundIndex == Other.BoundIndex;
//   }
// };
//
// struct Variable {
//   std::string Name;
//
//   Instruction *Inst = nullptr;
//
//   // If assumption set Variable bound to a literal value
//   // we need to remember it here
//   bool IsLiteral = false;
//   long LiteralValue = 0;
//
//   bool operator==(const Variable &Other) const { return Name == Other.Name; }
// };
using Literal = long;

using Operand = std::variant<PolicyBound, Variable, Literal>;

BoundBase *getBoundBase(Operand &Op) {
  return std::visit(
      [](auto &&Arg) -> BoundBase * {
        // On récupère le type réel stocké (T)
        using T = std::decay_t<decltype(Arg)>;

        // Si T est un enfant de BoundBase (PolicyBound ou Variable)
        if constexpr (std::is_base_of_v<BoundBase, T>) {
          return &Arg; // On retourne l'adresse castée implicitement en
                       // BoundBase*
        } else {
          // Sinon (c'est un Literal / long), on retourne nullptr
          return nullptr;
        }
      },
      Op);
}

std::string operandToString(const Operand &Operand) {
  return std::visit(
      [](auto &&Arg) -> std::string {
        using T = std::decay_t<decltype(Arg)>;
        if constexpr (std::is_same_v<T, PolicyBound>) {
          return Arg.PolicyStr + std::to_string(Arg.PolicyIndex) + "." +
                 Arg.BoundType + std::to_string(Arg.BoundIndex);
        } else if constexpr (std::is_same_v<T, Literal>) {
          return "int(" + std::to_string(Arg) + ")";
        } else if constexpr (std::is_same_v<T, Variable>) {
          return "variable(" + Arg.Name + ")";
        }
      },
      Operand);
}

bool operandEquals(const Operand &A, const Operand &B) {
  return std::visit(
      [&](auto &&ArgA, auto &&ArgB) -> bool {
        using TA = std::decay_t<decltype(ArgA)>;
        using TB = std::decay_t<decltype(ArgB)>;
        if constexpr (std::is_same_v<TA, TB>) {
          return ArgA == ArgB;
        } else {
          return false;
        }
      },
      A, B);
}

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

  static std::string operatorToString(const Comparison::Operator &Op) {
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

  static Comparison::Operator stringToOperator(const std::string &OpStr) {
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

  Operand LHS;
  Operator Op;
  Operand RHS;

  void reverseOperator() { Op = Comparison::getReverseOperator(Op); }

  static Operator getReverseOperator(Operator Op) {
    switch (Op) {
    case Comparison::Operator::EQUAL:
      return Comparison::Operator::EQUAL;
    case Comparison::Operator::NOT_EQUAL:
      return Comparison::Operator::NOT_EQUAL;
    case Comparison::Operator::LESS:
      return Comparison::Operator::GREATER;
    case Comparison::Operator::GREATER:
      return Comparison::Operator::LESS;
    case Comparison::Operator::LESS_EQUAL:
      return Comparison::Operator::GREATER_EQUAL;
    case Comparison::Operator::GREATER_EQUAL:
      return Comparison::Operator::LESS_EQUAL;
    case Comparison::Operator::UNKNOWN:
      return Comparison::Operator::UNKNOWN;
    }
  }

  std::string toString() const {
    return operandToString(LHS) + " " + operatorToString(Op) + " " +
           operandToString(RHS);
  }
};

// SmallVector<Comparison, 0>
// getOtherComparaisonWith(Operand Ope, SmallVectorImpl<Comparison> &AllComps)
// {
//   SmallVector<Comparison, 0> Results;
//
//   for (const Comparison &OtherC : AllComps) {
//     if (operandEquals(OtherC.LHS, Ope) || operandEquals(OtherC.RHS, Ope)) {
//       Results.push_back(OtherC);
//     }
//   }
//
//   return Results;
// }

struct ComparaisonEqualsOperand {
  ComparaisonEqualsOperand(Comparison *Comp, bool IsLeftOperand)
      : Comp(Comp), IsLeft(IsLeftOperand) {}

  bool isLeftOperand() const { return IsLeft; }
  bool isRightOperand() const { return not IsLeft; }

  template <typename Iterator>
  static SmallVector<ComparaisonEqualsOperand, 0>
  getOtherComparaisonWith(Operand Ope, Iterator Begin, Iterator End) {
    SmallVector<ComparaisonEqualsOperand, 0> Results;

    errs() << "Searching comparisons with operand: " << operandToString(Ope)
           << "\n";

    for (Iterator It = Begin; It != End; ++It) {
      auto &OtherC = *It;
      if (operandEquals(OtherC.LHS, Ope)) {
        Results.push_back(ComparaisonEqualsOperand(&OtherC, true));
      } else if (operandEquals(OtherC.RHS, Ope)) {
        Results.push_back(ComparaisonEqualsOperand(&OtherC, false));
      }
    }

    return Results;
  }

  Comparison *Comp;

  Comparison *operator()() const { return Comp; }

private:
  bool IsLeft;
};

std::optional<Operand> parseOperand(const std::string &S) {
  static const std::regex PolicyRegex(
      "(policy|p)(\\d+)\\.(?:(lower|l|upper|u)(\\d*))?");
  std::smatch Match;

  if (std::regex_match(S, Match, PolicyRegex)) {
    size_t PolicyIndex = std::stoul(Match[2].str());

    std::string BoundTypeStr;
    bool HasBoundType = Match[3].matched;

    if (HasBoundType) {
      BoundTypeStr = Match[3].str();
      if (BoundTypeStr == "l")
        BoundTypeStr = "lower";
      else if (BoundTypeStr == "u")
        BoundTypeStr = "upper";
    }

    size_t Dimension = 0;
    bool HasBoundIndex = false;
    if (HasBoundType && Match[4].matched && !Match[4].str().empty()) {
      Dimension = (size_t)std::stoul(Match[4].str());
      HasBoundIndex = true;
    }

    return PolicyBound("policy", PolicyIndex, HasBoundType, BoundTypeStr,
                       HasBoundIndex, Dimension);
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
    return Variable{S};
  }

  return std::nullopt;
}

std::vector<Comparison>
parseComparisons(const std::string &AssumptionsStr,
                 SmallVector<LoopBoundT, 4> &LoopBoundPolicyVec) {
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
    if (not LhsOpt) {
      report_fatal_error("Left assumption operand invalid: '" + Twine(LhsStr) +
                         "'");
    }

    auto Op = Comparison::stringToOperator(OpStr);
    if (Op == Comparison::Operator::UNKNOWN) {
      report_fatal_error("Invalid comparison operator: '" + Twine(OpStr) + "'");
    }

    std::optional<Operand> RhsOpt = parseOperand(RhsStr);
    if (not RhsOpt) {
      report_fatal_error("Right assumption operand invalid: '" + Twine(RhsStr) +
                         "'");
    }

    if (std::holds_alternative<PolicyBound>(LhsOpt.value()) and
        std::holds_alternative<PolicyBound>(RhsOpt.value())) {
      auto &LHSBound = std::get<PolicyBound>(LhsOpt.value());
      auto &RHSBound = std::get<PolicyBound>(RhsOpt.value());

      if (LHSBound.PolicyIndex != RHSBound.PolicyIndex and
          LHSBound.HasBoundType == RHSBound.HasBoundType and
          LHSBound.HasBoundType == false) {
        size_t LHSDim = LoopBoundAnalysis::getNumDimForPolicy(
            LoopBoundPolicyVec, LHSBound.PolicyIndex);
        size_t RHSDim = LoopBoundAnalysis::getNumDimForPolicy(
            LoopBoundPolicyVec, RHSBound.PolicyIndex);

        if (LHSDim != RHSDim) {
          report_fatal_error("Left and right assumption policy bound must have "
                             "the same number of dimensions if no bound type "
                             "is specified: '" +
                             Twine(CompStdStr) + "'");
        }

        for (size_t Dim = 0; Dim < LHSDim; ++Dim) {
          PolicyBound LBoundLower = LHSBound;
          LBoundLower.BoundType = "lower";
          LBoundLower.BoundIndex = Dim;
          LBoundLower.HasBoundType = true;

          PolicyBound RBoundLower = RHSBound;
          RBoundLower.BoundType = "lower";
          RBoundLower.BoundIndex = Dim;
          RBoundLower.HasBoundType = true;

          Comparison CompLower{LBoundLower, Op, RBoundLower};
          Results.push_back(CompLower);

          PolicyBound LBoundUpper = LHSBound;
          LBoundUpper.BoundType = "upper";
          LBoundUpper.BoundIndex = Dim;
          LBoundUpper.HasBoundType = true;

          PolicyBound RBoundUpper = RHSBound;
          RBoundUpper.BoundType = "upper";
          RBoundUpper.BoundIndex = Dim;
          RBoundUpper.HasBoundType = true;

          Comparison CompUpper{LBoundUpper, Op, RBoundUpper};
          Results.push_back(CompUpper);
        }
        continue;
      }

      if (LHSBound.HasBoundType != RHSBound.HasBoundType) {
        report_fatal_error("Left and right assumption policy bound must "
                           "either both have a bound type or none: '" +
                           Twine(CompStdStr) + "'");
      }

      if (LHSBound.PolicyIndex != RHSBound.PolicyIndex and
          LHSBound.HasBoundIndex == RHSBound.HasBoundIndex and
          LHSBound.HasBoundIndex == false) {
        size_t LHSDim = LoopBoundAnalysis::getNumDimForPolicy(
            LoopBoundPolicyVec, LHSBound.PolicyIndex);
        size_t RHSDim = LoopBoundAnalysis::getNumDimForPolicy(
            LoopBoundPolicyVec, RHSBound.PolicyIndex);

        if (LHSDim != RHSDim) {
          report_fatal_error("Left and right assumption policy bound must have "
                             "the same number of dimensions if no bound index "
                             "is specified: '" +
                             Twine(CompStdStr) + "'");
        }

        for (size_t Dim = 0; Dim < LHSDim; ++Dim) {
          PolicyBound LBound = LHSBound;
          LBound.HasBoundIndex = true;
          LBound.BoundIndex = Dim;

          PolicyBound RBound = RHSBound;
          RBound.HasBoundIndex = true;
          RBound.BoundIndex = Dim;

          Comparison Comp{LBound, Op, RBound};
          Results.push_back(Comp);
        }
        continue;
      }
      if (LHSBound.HasBoundIndex != RHSBound.HasBoundIndex) {
        report_fatal_error("Left and right assumption policy bound must "
                           "either both have a bound index or none: '" +
                           Twine(CompStdStr) + "'");
      }

      if (LHSBound.PolicyIndex == RHSBound.PolicyIndex and
          LHSBound.BoundType == RHSBound.BoundType and
          LHSBound.BoundIndex == RHSBound.BoundIndex) {
        report_fatal_error("Left and right assumption policy bound must be "
                           "different: '" +
                           Twine(CompStdStr) + "'");
      }
    }

    if (std::holds_alternative<Literal>(LhsOpt.value()) and
        std::holds_alternative<Literal>(RhsOpt.value())) {
      report_fatal_error(
          "At least one operand must be a policy bound or variable: '" +
          Twine(CompStdStr) + "'");
    } else if (std::holds_alternative<Literal>(LhsOpt.value())) {
      std::swap(LhsOpt, RhsOpt);
      Op = Comparison::getReverseOperator(Op);
    }

    if (std::holds_alternative<PolicyBound>(LhsOpt.value()) and
        std::holds_alternative<Literal>(RhsOpt.value())) {
      auto &LHSBound = std::get<PolicyBound>(LhsOpt.value());
      auto &RHSLiteral = std::get<Literal>(RhsOpt.value());

      if (not LHSBound.HasBoundType) {
        report_fatal_error("Policy bound must have a bound type when "
                           "compared to a literal: '" +
                           Twine(CompStdStr) + "'");
      }

      size_t LHSDim = LoopBoundAnalysis::getNumDimForPolicy(
          LoopBoundPolicyVec, LHSBound.PolicyIndex);

      for (size_t Dim = 0; Dim < LHSDim; ++Dim) {
        PolicyBound LBoundLower = LHSBound;
        LBoundLower.BoundType = LHSBound.BoundType;
        LBoundLower.BoundIndex = Dim;
        LBoundLower.HasBoundType = true;

        Comparison CompLower{LBoundLower, Op, RHSLiteral};
        Results.push_back(CompLower);
      }
      continue;
    }

    if (std::holds_alternative<Variable>(LhsOpt.value()) and
        std::holds_alternative<Variable>(RhsOpt.value())) {
      auto &LHSVar = std::get<Variable>(LhsOpt.value());
      auto &RHSVar = std::get<Variable>(RhsOpt.value());
      if (LHSVar.Name == RHSVar.Name) {
        report_fatal_error("Left and right assumption variable must be "
                           "different: '" +
                           Twine(CompStdStr) + "'");
      }
    }

    if (std::holds_alternative<Literal>(LhsOpt.value()) and
        std::holds_alternative<Literal>(RhsOpt.value())) {
      report_fatal_error(
          "At least one operand must be a policy bound or variable: '" +
          Twine(CompStdStr) + "'");
    }

    Comparison Comp;
    Comp.LHS = LhsOpt.value();
    Comp.Op = Op;
    Comp.RHS = RhsOpt.value();

    Results.push_back(Comp);
  }

  return Results;
}

StringRef extractAssumptionAnnotation(Function &F) {
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

std::vector<Comparison> parseAssumptions(
    StringRef AssumptionsStr, SmallVector<LoopBoundT, 4> &LoopBoundPolicyVec,
    SmallVector<std::pair<Instruction *, StringRef>, 2> &LoopBoundVarVec) {

  auto ComparaisonVec =
      parseComparisons(AssumptionsStr.str(), LoopBoundPolicyVec);

  auto IsValidLoopBound = [&LoopBoundPolicyVec,
                           &LoopBoundVarVec](auto &C) -> bool {
    if (std::holds_alternative<PolicyBound>(C)) {
      auto &HS = std::get<PolicyBound>(C);

      auto BoundType =
          HS.BoundType == "lower" ? LoopBoundT::Lower : LoopBoundT::Upper;

      auto It = std::find_if(
          LoopBoundPolicyVec.begin(), LoopBoundPolicyVec.end(),
          [&](const LoopBoundT &LB) {
            if (HS.PolicyIndex == LB.IndexPolicy and
                BoundType == LB.BoundType and (HS.BoundIndex) == LB.Depth) {
              return true;
            }
            return false;
          });
      if (It != LoopBoundPolicyVec.end()) {
        HS.Inst = It->Inst;
      } else {
        return false;
      }
    }
    if (std::holds_alternative<Variable>(C)) {
      auto &Var = std::get<Variable>(C);
      auto It =
          std::find_if(LoopBoundVarVec.begin(), LoopBoundVarVec.end(),
                       [&](const std::pair<Instruction *, StringRef> &VarInst) {
                         return Var.Name == VarInst.second.str();
                       });
      if (It == LoopBoundVarVec.end()) {
        return false;
      }
      Var.Inst = It->first;
    }
    return true;
  };

  for (auto C = ComparaisonVec.begin(); C != ComparaisonVec.end();) {
    if (not IsValidLoopBound(C->LHS)) {
      errs() << "LHS assumption failed in : " << C->toString() << "\n";
      C = ComparaisonVec.erase(C);
      continue;
    }
    if (not IsValidLoopBound(C->RHS)) {
      errs() << "RHS assumption failed in : " << C->toString() << "\n";
      C = ComparaisonVec.erase(C);
      continue;
    }
    C++;
  }

  return ComparaisonVec;
}

bool collectDependenciesToHoist(Instruction *I, BasicBlock *TargetBlock,
                                DominatorTree &DT,
                                SetVector<Instruction *> &ToHoist) {
  if (ToHoist.count(I))
    return true;

  for (Value *Op : I->operands()) {
    Instruction *OpInst = dyn_cast<Instruction>(Op);
    if (!OpInst)
      continue;

    if (DT.dominates(OpInst->getParent(), TargetBlock))
      continue;

    if (OpInst->getParent() == I->getParent()) {
      if (!collectDependenciesToHoist(OpInst, TargetBlock, DT, ToHoist))
        return false;
    } else {
      return false;
    }
  }
  ToHoist.insert(I);
  return true;
}

bool hoistInstructionChain(Instruction *I, BasicBlock *TargetBlock,
                           DominatorTree &DT) {
  SetVector<Instruction *> InstructionsToMove;
  Instruction *InsertionPoint = TargetBlock->getTerminator()->getPrevNode();
  if (collectDependenciesToHoist(I, TargetBlock, DT, InstructionsToMove)) {
    for (Instruction *Inst : InstructionsToMove) {
      errs() << "  Hoisting: " << *Inst << "\n";
    }
    for (Instruction *Inst : InstructionsToMove) {
      Inst->moveBefore(InsertionPoint);
    }
    return true;
  }
  return false;
}

Instruction *findDominatingOrHoistChain(Instruction *A, Instruction *B,
                                        DominatorTree &DT) {
  if (DT.dominates(A, B))
    return A;
  if (DT.dominates(B, A))
    return B;

  BasicBlock *CommonBB =
      DT.findNearestCommonDominator(A->getParent(), B->getParent());

  if (hoistInstructionChain(A, CommonBB, DT))
    return A;

  llvm_unreachable("No dominating instruction found between the two.");
}

template <typename T>
void applyNotLiteralVsLiteral(Comparison &C, Function &F, IRBuilder<> &Builder,
                              std::vector<Comparison> &Assumptions) {
  LLVM_DEBUG(errs() << "Applying policy/variable vs literal assumption: "
                    << C.toString() << "\n");
  auto &LHS = std::get<T>(C.LHS);
  auto &RHS = std::get<Literal>(C.RHS);

  auto *Inst = LHS.Inst;

  if (!Inst)
    llvm_unreachable("Instruction de variable nulle.");

  Constant *ConstVal = ConstantInt::get(Inst->getType(), RHS);

  switch (C.Op) {
  case Comparison::Operator::EQUAL: {
    LLVM_DEBUG(errs() << "Replacing " << *Inst << " with constant " << *ConstVal
                      << "\n");

    for (auto *User : Inst->users()) {
      if (auto *ICmp = dyn_cast<ICmpInst>(User)) {
        Module *M = Inst->getModule();
        unsigned SourceKindID = M->getMDKindID("loop_bound_information");
        unsigned DestKindID = M->getMDKindID("old_loop_bound");

        if (MDNode *Node = Inst->getMetadata(SourceKindID))
          ICmp->setMetadata(DestKindID, Node);
      }
    }

    Inst->replaceAllUsesWith(ConstVal);

    LHS.IsLiteral = true;
    LHS.LiteralValue = RHS;

    auto Begin = std::find_if(
        Assumptions.begin(), Assumptions.end(),
        [&C](const Comparison &Assumption) { return &Assumption == &C; });
    if (Begin != Assumptions.end())
      Begin++;

    auto Vec = ComparaisonEqualsOperand::getOtherComparaisonWith(
        C.LHS, Begin, Assumptions.end());
    for (auto OtherEqual : Vec) {
      auto &OtherC = *(OtherEqual());

      bool IsLeft = OtherEqual.isLeftOperand();

      if (IsLeft) {
        errs() << "   left operand\n";
      } else {
        errs() << "   right operand\n";
      }

      if (not std::holds_alternative<Literal>(OtherC.LHS) and
          not std::holds_alternative<Literal>(OtherC.RHS)) {

        if (IsLeft)
          OtherC.LHS = RHS;
        else
          OtherC.RHS = RHS;

        std::swap(OtherC.LHS, OtherC.RHS);
        OtherC.reverseOperator();
      }

      // if (std::holds_alternative<T>(OtherC.LHS) and
      //     not std::holds_alternative<Literal>(OtherC.RHS)) {
      //   auto &OtherVarLHS = std::get<T>(OtherC.LHS);
      //   errs() << "       OtherVar " << operandToString(OtherC.LHS) <<
      //   "\n"; if (OtherVarLHS == LHS) {
      //     OtherC.LHS = RHS;
      //     std::swap(OtherC.LHS, OtherC.RHS);
      //     OtherC.reverseOperator();
      //   }
      // }
      // if (std::holds_alternative<T>(OtherC.RHS)) {
      //   auto &OtherRHS = std::get<T>(OtherC.RHS);
      //   errs() << "       OtherBite " << operandToString(OtherC.RHS) <<
      //   "\n"; if (OtherRHS == LHS) {
      //     OtherC.RHS = RHS;
      //   }
      // } else {
      //   errs() << "       OtherLiteral " << operandToString(OtherC.RHS) <<
      //   "\n";
      // }
      LLVM_DEBUG(errs() << "Updated related assumption: " << OtherC.toString()
                        << "\n");
    }

    // for (auto It = Begin; It != Assumptions.end(); ++It) {
    //   auto &OtherC = *It;
    //   bool Updated = false;
    //   errs() << "     other " << OtherC.toString() << "\n";
    //   if (std::holds_alternative<T>(OtherC.LHS) and
    //       not std::holds_alternative<Literal>(OtherC.RHS)) {
    //     auto &OtherVarLHS = std::get<T>(OtherC.LHS);
    //     errs() << "       OtherVar " << operandToString(OtherC.LHS) <<
    //     "\n"; if (OtherVarLHS == LHS) {
    //       OtherC.LHS = RHS;
    //       std::swap(OtherC.LHS, OtherC.RHS);
    //       OtherC.reverseOperator();
    //       Updated = true;
    //     }
    //   }
    //   if (std::holds_alternative<T>(OtherC.RHS)) {
    //     auto &OtherRHS = std::get<T>(OtherC.RHS);
    //     if (OtherRHS == LHS) {
    //       OtherC.RHS = RHS;
    //       Updated = true;
    //     }
    //   }
    //   if (Updated) {
    //     LLVM_DEBUG(errs() << "Updated related assumption: " <<
    //     OtherC.toString()
    //                       << "\n");
    //   }
    // }

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

    Builder.SetInsertPoint(Inst->getNextNode());
    llvm::Value *Cmp = Builder.CreateICmp(Pred, Inst, ConstVal);
    llvm::Value *Assumption = Builder.CreateAssumption(Cmp);
    LLVM_DEBUG(errs() << "Registering assumption " << *Cmp << " -> "
                      << *Assumption << "\n");
    break;
  }
  default:
    break;
  }
}

template <typename T, typename U>
void applyNotLiteralVsNotLiteral(Comparison &C, Function &F,
                                 IRBuilder<> &Builder,
                                 std::vector<Comparison> &Assumptions,
                                 DominatorTree &DT) {
  LLVM_DEBUG(
      errs() << "Applying policy/variable vs policy/variable assumption: "
             << C.toString() << "\n");
  auto &LHS = std::get<T>(C.LHS);
  auto &RHS = std::get<U>(C.RHS);

  if (LHS.IsLiteral and RHS.IsLiteral) {
    if (LHS.LiteralValue != RHS.LiteralValue) {
      report_fatal_error("Conflicting literal values in assumption: " +
                         Twine(C.toString()));
    }
    return;
  }

  if (LHS.IsLiteral) {
    std::swap(C.LHS, C.RHS);
    C.reverseOperator();
    applyNotLiteralVsLiteral<U>(C, F, Builder, Assumptions);
    return;
  }
  if (RHS.IsLiteral) {
    applyNotLiteralVsLiteral<T>(C, F, Builder, Assumptions);
    return;
  }

  auto *LHSInst = LHS.Inst;
  auto *RHSInst = RHS.Inst;

  if (not LHSInst or not RHSInst) {
    report_fatal_error("Failed to bind policy/variable bound in assumption: " +
                       Twine(C.toString()));
    return;
  }

  switch (C.Op) {
  case Comparison::Operator::EQUAL: {
    Instruction *Replacer = findDominatingOrHoistChain(LHSInst, RHSInst, DT);
    Instruction *Replaced = (Replacer == LHSInst) ? RHSInst : LHSInst;

    if (Replacer == Replaced) {
      return;
    }

    if (Replacer->getType() != Replaced->getType()) {
      if (Replaced->getType()->isIntegerTy(64) and
          Replacer->getType()->isIntegerTy(32)) {
        Builder.SetInsertPoint(Replaced->getIterator()->getNextNode());
        auto *Cast =
            Builder.CreateTrunc(Replaced, Replacer->getType(), "trunc_val");
        Value *Cmp =
            Builder.CreateICmp(CmpInst::Predicate::ICMP_EQ, Cast, Replacer);
        Value *Assumption = Builder.CreateAssumption(Cmp);
        errs() << "Registering assumption " << *Cmp << " -> " << *Assumption
               << "\n";
        return;
      }
    }

    LLVM_DEBUG(errs() << "Replacing " << *Replaced << " with " << *Replacer
                      << "\n");
    Replaced->replaceAllUsesWith(Replacer);

    if (Replaced == RHSInst) {
      errs() << "   right replaced\n";
    } else {
      errs() << "   left replaced\n";
    }

    // Update other assumptions that reference RHSInst to use LHSInst
    auto Begin = std::find_if(
        Assumptions.begin(), Assumptions.end(),
        [&C](const Comparison &Assumption) { return &Assumption == &C; });
    if (Begin != Assumptions.end())
      Begin++;

    auto Vec = ComparaisonEqualsOperand::getOtherComparaisonWith(
        LHSInst == Replaced ? C.LHS : C.RHS, Begin, Assumptions.end());
    for (auto OtherEqual : Vec) {
      auto &OtherC = *(OtherEqual());
      bool IsLeft = OtherEqual.isLeftOperand();

      if (IsLeft) {
        errs() << "   left operand\n";
      } else {
        errs() << "   right operand\n";
      }

      auto Other = getBoundBase(IsLeft ? OtherC.LHS : OtherC.RHS);
      Other->Inst = Replacer;

      LLVM_DEBUG(errs() << "Updated related assumption: " << OtherC.toString()
                        << "\n");
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

    Instruction *InsertLoc =
        findDominatingOrHoistChain(LHSInst, RHSInst, DT) == LHSInst ? RHSInst
                                                                    : LHSInst;
    Builder.SetInsertPoint(InsertLoc->getNextNode());
    Value *Cmp = Builder.CreateICmp(Pred, LHSInst, RHSInst);
    Value *Assumption = Builder.CreateAssumption(Cmp);
    LLVM_DEBUG(errs() << "Registering assumption: " << *Assumption << "\n");
    break;
  }
  default:
    llvm_unreachable("Opérateur inconnu dans une comparaison validée.");
  }
}

void applyAssumptions(Function &F, std::vector<Comparison> &Assumptions,
                      DominatorTree &DT) {
  IRBuilder<> Builder(&F.getEntryBlock(), F.getEntryBlock().begin());

  for (auto &Assumption : Assumptions) {
    LLVM_DEBUG(errs() << "\n\nApplying assumption: " << Assumption.toString()
                      << "\n");

    std::visit(
        [&](auto &&LHS, auto &&RHS) {
          using T1 = std::decay_t<decltype(LHS)>;
          using T2 = std::decay_t<decltype(RHS)>;

          if constexpr (std::is_same_v<T1, PolicyBound> &&
                        std::is_same_v<T2, PolicyBound>) {
            applyNotLiteralVsNotLiteral<PolicyBound, PolicyBound>(
                Assumption, F, Builder, Assumptions, DT);
          } else if constexpr (std::is_same_v<T1, Variable> &&
                               std::is_same_v<T2, Variable>) {
            applyNotLiteralVsNotLiteral<Variable, Variable>(
                Assumption, F, Builder, Assumptions, DT);
          } else if constexpr (std::is_same_v<T1, Variable> &&
                               std::is_same_v<T2, PolicyBound>) {
            applyNotLiteralVsNotLiteral<Variable, PolicyBound>(
                Assumption, F, Builder, Assumptions, DT);
          } else if constexpr (std::is_same_v<T1, PolicyBound> &&
                               std::is_same_v<T2, Variable>) {
            applyNotLiteralVsNotLiteral<PolicyBound, Variable>(
                Assumption, F, Builder, Assumptions, DT);
          } else if constexpr (std::is_same_v<T1, PolicyBound> &&
                               std::is_same_v<T2, Literal>) {
            applyNotLiteralVsLiteral<PolicyBound>(Assumption, F, Builder,
                                                  Assumptions);
          } else if constexpr (std::is_same_v<T1, Variable> &&
                               std::is_same_v<T2, Literal>) {
            applyNotLiteralVsLiteral<Variable>(Assumption, F, Builder,
                                               Assumptions);
          } else {
            llvm_unreachable("Other combinations are not supported yet.");
          }
        },
        Assumption.LHS, Assumption.RHS);
  }
}

} // namespace

bool polly::hoistInstructionChain(Instruction *I, BasicBlock *TargetBlock,
                                  DominatorTree &DT) {
  SetVector<Instruction *> InstructionsToMove;
  Instruction *InsertionPoint = TargetBlock->getTerminator()->getPrevNode();
  if (collectDependenciesToHoist(I, TargetBlock, DT, InstructionsToMove)) {
    for (Instruction *Inst : InstructionsToMove) {
      errs() << "  Hoisting: " << *Inst << "\n";
    }
    for (Instruction *Inst : InstructionsToMove) {
      Inst->moveBefore(InsertionPoint);
    }
    return true;
  }
  return false;
}

PreservedAnalyses UserAssumptions::run(Function &F,
                                       FunctionAnalysisManager &AM) {
  if (not F.hasFnAttribute("polly.findSCoP"))
    return PreservedAnalyses::all();

  LLVM_DEBUG(errs() << "UserAssumptions pass run on " << F.getName().str()
                    << "\n");

  auto LBA = LoopBoundAnalysis().run(F, AM);
  LLVM_DEBUG(errs() << LBA << "\n");

  auto AssumptionsStr = extractAssumptionAnnotation(F);
  auto LoopBoundVariableInstructions = polly::findVarInstructions(F);
  auto Assumptions =
      parseAssumptions(AssumptionsStr, LBA, LoopBoundVariableInstructions);

  for (const auto &Cmp : Assumptions)
    LLVM_DEBUG(errs() << Cmp.toString() << "\n");

  auto DT = DominatorTreeAnalysis().run(F, AM);
  applyAssumptions(F, Assumptions, DT);

  if (verifyFunction(F, &errs())) {
    report_fatal_error("IR verification failed.");
  }

  LLVM_DEBUG(errs() << "UserAssumptions pass done\n");
  return PreservedAnalyses::all();
}
