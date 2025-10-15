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
#include "llvm/Passes/PassBuilder.h"
#include "llvm/Support/raw_ostream.h"
#include <charconv>
#include <optional>
#include <regex>

using namespace llvm;
using namespace polly;

namespace {

struct PolicyBound {
  std::string Policy;
  size_t PolicyIndex;
  std::string BoundType;
  size_t BoundIndex;

  // IR
  // Loop *L = nullptr;
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
extractAssumptionAnnotation(Function &F,
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
parseAssumptions(StringRef AssumptionsStr,
                 SmallVector<LoopBoundT, 4> &LoopBoundVec) {

  auto ComparaisonVec = parseComparisons(AssumptionsStr.str());

  auto Lambda = [&LoopBoundVec](auto &C) -> bool {
    if (std::holds_alternative<PolicyBound>(C)) {
      auto &HS = std::get<PolicyBound>(C);

      auto BoundType =
          HS.BoundType == "lower" ? LoopBoundT::Lower : LoopBoundT::Upper;

      auto It = std::find_if(
          LoopBoundVec.begin(), LoopBoundVec.end(), [&](const LoopBoundT &LB) {
            if (HS.PolicyIndex == LB.IndexPolicy and
                BoundType == LB.BoundType and (HS.BoundIndex) == LB.Depth) {
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

void applyPolicyVsLiteral(Comparison &C, Function &F, IRBuilder<> &Builder,
                          std::vector<Comparison> &Assumptions);

void applyPolicyVsPolicy(Comparison &C, Function &F, IRBuilder<> &Builder,
                         std::vector<Comparison> &Assumptions) {
  errs() << "Applying policy vs policy assumption: " << comparisonToString(C)
         << "\n";
  auto &LHS = std::get<PolicyBound>(C.LHS);
  auto &RHS = std::get<PolicyBound>(C.RHS);

  if (LHS.IsLiteral and RHS.IsLiteral) {
    return;
  }

  if (LHS.IsLiteral) {
    std::swap(C.LHS, C.RHS);
    errs() << "On a un littéral à gauche, on swap\n";
    applyPolicyVsLiteral(C, F, Builder, Assumptions);
    return;
  }
  if (RHS.IsLiteral) {
    applyPolicyVsLiteral(C, F, Builder, Assumptions);
    return;
  }

  if (LHS.PolicyIndex > RHS.PolicyIndex)
    std::swap(LHS, RHS);

  auto *LHSInst = LHS.InstBound;
  auto *RHSInst = RHS.InstBound;

  if (!LHSInst || !RHSInst) {
    llvm_unreachable("Instruction de borne nulle, la liaison a échoué.");
  }

  if (not LHSInst or not RHSInst) {
    errs() << "Failed to bind policy bound in assumption: "
           << comparisonToString(C) << "\n";
    return;
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
    Value *Assumption = Builder.CreateAssumption(Cmp);
    errs() << "Registering assumption: " << *Assumption << "\n";
    break;
  }
  default:
    llvm_unreachable("Opérateur inconnu dans une comparaison validée.");
  }
}

void applyPolicyVsLiteral(Comparison &C, Function &F, IRBuilder<> &Builder,
                          std::vector<Comparison> &Assumptions) {
  errs() << "Applying policy vs literal assumption: " << comparisonToString(C)
         << "\n";
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

    errs() << "Registering assumption: " << *Assumption << "\n";
    break;
  }
  default:
    llvm_unreachable("Opérateur inconnu dans une comparaison validée.");
  }
}

void applyAssumptions(Function &F, std::vector<Comparison> &Assumptions) {
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
            applyPolicyVsPolicy(Assumption, F, Builder, Assumptions);
          } else if constexpr (std::is_same_v<T1, PolicyBound> &&
                               std::is_same_v<T2, Literal>) {
            applyPolicyVsLiteral(Assumption, F, Builder, Assumptions);
          } else {
            llvm_unreachable("Other combinations are not supported yet.");
          }
        },
        Assumption.LHS, Assumption.RHS);
  }
}

} // namespace

PreservedAnalyses UserAssumptions::run(Function &F,
                                       FunctionAnalysisManager &AM) {
  if (not F.hasFnAttribute("polly.findSCoP"))
    return PreservedAnalyses::all();

  errs() << "UserAssumptions pass run on " << F.getName().str() << "\n";

  auto &LBA = AM.getResult<LoopBoundAnalysis>(F);
  // errs() << LBA << "\n";

  auto AssumptionsStr = extractAssumptionAnnotation(F, LBA);
  errs() << "AssumptionsStr = '" << AssumptionsStr << "\n";
  auto Assumptions = parseAssumptions(AssumptionsStr, LBA);

  errs() << "Contenu de la chaîne : \n";
  for (const auto &Cmp : Assumptions)
    errs() << operandToString(Cmp.LHS) << " " << operatorToString(Cmp.Op) << " "
           << operandToString(Cmp.RHS) << "\n";

  applyAssumptions(F, Assumptions);

  errs() << "UserAssumptions pass done\n";

  return PreservedAnalyses::all();
}
