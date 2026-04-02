//===-- JSONExporter.cpp  - Export Scops as JSON  -------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// Export the Scops build by ScopInfo pass as a JSON file.
//
//===----------------------------------------------------------------------===//

#include "polly/Test/OpenSCoPExporter.h"
#include "polly/JSONExporter.h"
#include "polly/ScopDetection.h"
#include "polly/ScopInfo.h"
#include "polly/ScopPass.h"
#include "polly/Support/ISLOStream.h"
#include "polly/Support/ISLTools.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/Analysis/MemorySSA.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"
#include "isl/isl-noexceptions.h"
#include "isl/mat.h"
#include <cstdio>
#include <osl/scop.h>
#include <string>

#define DEBUG_TYPE "polly-openscop-exporter"

using namespace llvm;
using namespace polly;

namespace {

static isl_stat readSetConstraints(__isl_take isl_constraint *Constraint,
                                   void *User) {
  auto *Mat = static_cast<std::vector<std::vector<long long>> *>(User);
  std::vector<long long> Row;

  Row.push_back(isl_constraint_is_equality(Constraint) ? 0 : 1);

  isl_space *Space = isl_constraint_get_space(Constraint);
  int NbIter = isl_space_dim(Space, isl_dim_set);
  int NbParam = isl_space_dim(Space, isl_dim_param);
  isl_space_free(Space);

  for (int I = 0; I < NbIter; ++I) {
    isl_val *V = isl_constraint_get_coefficient_val(Constraint, isl_dim_set, I);
    Row.push_back(isl_val_get_num_si(V));
    isl_val_free(V);
  }

  for (int I = 0; I < NbParam; ++I) {
    isl_val *V =
        isl_constraint_get_coefficient_val(Constraint, isl_dim_param, I);
    Row.push_back(isl_val_get_num_si(V));
    isl_val_free(V);
  }

  isl_val *V = isl_constraint_get_constant_val(Constraint);
  Row.push_back(isl_val_get_num_si(V));
  isl_val_free(V);

  Mat->push_back(std::move(Row));
  isl_constraint_free(Constraint);
  return isl_stat_ok;
}
static isl_stat readMapConstraints(__isl_take isl_constraint *Constraint,
                                   void *User) {
  auto *Mat = static_cast<std::vector<std::vector<long long>> *>(User);
  std::vector<long long> Row;

  Row.push_back(isl_constraint_is_equality(Constraint) ? 0 : 1);

  isl_space *Space = isl_constraint_get_space(Constraint);
  int NbOut = isl_space_dim(Space, isl_dim_out);
  int NbIn = isl_space_dim(Space, isl_dim_in);
  int NbParam = isl_space_dim(Space, isl_dim_param);
  isl_space_free(Space);

  for (int I = 0; I < NbOut; I++) {
    isl_val *V = isl_constraint_get_coefficient_val(Constraint, isl_dim_out, I);
    Row.push_back(isl_val_get_num_si(V));
    isl_val_free(V);
  }

  for (int I = 0; I < NbIn; I++) {
    isl_val *V = isl_constraint_get_coefficient_val(Constraint, isl_dim_in, I);
    Row.push_back(isl_val_get_num_si(V));
    isl_val_free(V);
  }

  for (int I = 0; I < NbParam; I++) {
    isl_val *V =
        isl_constraint_get_coefficient_val(Constraint, isl_dim_param, I);
    Row.push_back(isl_val_get_num_si(V));
    isl_val_free(V);
  }

  isl_val *V = isl_constraint_get_constant_val(Constraint);
  Row.push_back(isl_val_get_num_si(V));
  isl_val_free(V);

  Mat->push_back(std::move(Row));
  isl_constraint_free(Constraint);
  return isl_stat_ok;
}

static isl_stat readMapConstraintsAccess(__isl_take isl_constraint *Constraint,
                                         void *User) {
  auto *Mat = static_cast<std::vector<std::vector<long long>> *>(User);
  std::vector<long long> Row;
  errs() << "Reading access constraint\n";

  Row.push_back(isl_constraint_is_equality(Constraint) ? 0 : 1);
  if (isl_constraint_is_equality(Constraint) == 0) {
    errs() << "Warning: we have an inequality in an access relation\n";
    isl_constraint_free(Constraint);
    return isl_stat_ok;
  }

  Row.push_back(0);

  isl_space *Space = isl_constraint_get_space(Constraint);
  int NbOut = isl_space_dim(Space, isl_dim_out);
  int NbIn = isl_space_dim(Space, isl_dim_in);
  int NbParam = isl_space_dim(Space, isl_dim_param);
  isl_space_free(Space);

  bool Valid = false;
  for (int I = 0; I < NbOut; I++) {
    isl_val *V = isl_constraint_get_coefficient_val(Constraint, isl_dim_out, I);
    Row.push_back(isl_val_get_num_si(V));
    isl_val_free(V);
    if (Row[Row.size() - 1] != 0)
      Valid = true;
  }

  for (int I = 0; I < NbIn; I++) {
    isl_val *V = isl_constraint_get_coefficient_val(Constraint, isl_dim_in, I);
    Row.push_back(isl_val_get_num_si(V));
    isl_val_free(V);
    if (Row[Row.size() - 1] != 0)
      Valid = true;
  }

  for (int I = 0; I < NbParam; I++) {
    isl_val *V =
        isl_constraint_get_coefficient_val(Constraint, isl_dim_param, I);
    Row.push_back(isl_val_get_num_si(V));
    isl_val_free(V);
  }

  isl_val *V = isl_constraint_get_constant_val(Constraint);
  Row.push_back(isl_val_get_num_si(V));
  isl_val_free(V);

  if (not Valid) {
    errs() << "Warning we remove some constraints from the access relation\n";
    isl_constraint_free(Constraint);
    return isl_stat_ok;
  }
  Mat->push_back(std::move(Row));
  isl_constraint_free(Constraint);
  return isl_stat_ok;
}

template <typename ISLType>
unsigned extractDimValue(ISLType &ISLObjet, isl::dim Dim) {
  static_assert(std::is_same<ISLType, isl::set>::value ||
                    std::is_same<ISLType, isl::space>::value ||
                    std::is_same<ISLType, isl::map>::value,
                "extractDimValue is only defined for isl::set or isl::map");

  isl::size ISLNbParams = ISLObjet.dim(Dim);
  return unsignedFromIslSize(ISLNbParams);
}

osl_relation_p convertContext(isl::set ISLContext, Scop &S) {
  errs() << ISLContext << "\n";
  // isl::basic_set_list BasicSetList = ISLContext.get_basic_set_list();
  // std::vector<std::vector<long long>> Matrix;
  // BasicSetList.foreach ([&](isl::basic_set BasicSet) -> isl::stat {
  //   isl_basic_set *BasicSetC = BasicSet.copy();
  //   isl_basic_set_foreach_constraint(BasicSetC, &readSetConstraints,
  //   &Matrix); isl_basic_set_free(BasicSetC);
  //
  //   return isl::stat::ok();
  // });

  for (auto &Array : S.arrays()) {
    errs() << "array " << Array->getSpace() << "\n";
  }

  unsigned NbParams = extractDimValue(ISLContext, isl::dim::param);
  osl_relation_p OSLContext = osl_relation_malloc(0, 2 + NbParams);
  OSLContext->type = OSL_TYPE_CONTEXT;
  OSLContext->precision = OSL_PRECISION_DP;
  OSLContext->nb_output_dims = 0;
  OSLContext->nb_input_dims = 0;
  OSLContext->nb_local_dims = 0;
  OSLContext->nb_parameters = NbParams;

  // const auto Min = static_cast<long long>(std::numeric_limits<int>::min());
  // const auto Max = static_cast<long long>(std::numeric_limits<int>::max());

  // for (size_t I = 0; I < Matrix.size(); ++I) {
  //   for (size_t J = 0; J < Matrix[I].size(); ++J) {
  //     errs() << "I " << I << "  J " << J << "    ->   " << Matrix[I][J] <<
  //     "\n";
  //
  //     long long Val = Matrix[I][J];
  //     int ConvertedVal = static_cast<int>(std::clamp(Val, Min, Max));
  //
  //     // osl_int_set_si(OSL_PRECISION_DP, OSLContext->m[I] + J,
  //     ConvertedVal);
  //   }
  // }

  return OSLContext;
}

osl_statement_p convertStmt(ScopStmt &Stmt,
                            SmallDenseMap<StringRef, int> &ArraysNameToId) {
  osl_statement_p OSLStmt = osl_statement_malloc();

  // Domain
  isl::set Domain = Stmt.getDomain();
  isl::basic_set_list BasicSetList = Domain.get_basic_set_list();

  errs() << "before domain read\n";
  std::vector<std::vector<long long>> Matrix;
  BasicSetList.foreach ([&](isl::basic_set BasicSet) -> isl::stat {
    isl_basic_set *BasicSetC = BasicSet.copy();
    isl_basic_set_foreach_constraint(BasicSetC, &readSetConstraints, &Matrix);
    isl_basic_set_free(BasicSetC);

    return isl::stat::ok();
  });

  errs() << "after domain read\n";

  // Parameters
  auto NbDomainParams = extractDimValue(Domain, isl::dim::param);
  // Output dimensions
  auto NbDomainOutputDims = extractDimValue(Domain, isl::dim::out);

  OSLStmt->domain = osl_relation_malloc(Matrix.size(), 2 + NbDomainOutputDims +
                                                           NbDomainParams);
  OSLStmt->domain->type = OSL_TYPE_DOMAIN;
  OSLStmt->domain->precision = OSL_PRECISION_DP;
  OSLStmt->domain->nb_output_dims = NbDomainOutputDims;
  OSLStmt->domain->nb_input_dims = 0;
  OSLStmt->domain->nb_local_dims = 0;
  OSLStmt->domain->nb_parameters = NbDomainParams;
  OSLStmt->usr = strdup(Stmt.getBaseName());

  for (size_t I = 0; I < Matrix.size(); ++I) {
    for (size_t J = 0; J < Matrix[I].size(); ++J) {
      osl_int_set_si(OSL_PRECISION_DP, OSLStmt->domain->m[I] + J, Matrix[I][J]);
    }
  }

  errs() << "after parameters read\n";

  // Scattering
  auto Schedule = Stmt.getSchedule();

  std::vector<std::vector<long long>> Matrix2;
  isl::basic_map_list BasicMapList = Schedule.get_basic_map_list();

  BasicMapList.foreach ([&](isl::basic_map BM) -> isl::stat {
    isl_basic_map *BasicMapC = BM.copy();
    isl_basic_map_foreach_constraint(BasicMapC, &readMapConstraints, &Matrix2);
    isl_basic_map_free(BasicMapC);
    return isl::stat::ok();
  });

  // Parameters
  auto NbScheduleParams = extractDimValue(Schedule, isl::dim::param);

  // Input dimensions
  auto NbScheduleInputDims = extractDimValue(Schedule, isl::dim::in);

  // Output dimensions
  auto NbScheduleOutputDims = extractDimValue(Schedule, isl::dim::out);

  OSLStmt->scattering = osl_relation_malloc(
      Matrix2.size(),
      2 + NbScheduleOutputDims + NbScheduleInputDims + NbScheduleParams);
  OSLStmt->scattering->type = OSL_TYPE_SCATTERING;
  OSLStmt->scattering->precision = OSL_PRECISION_DP;
  OSLStmt->scattering->nb_output_dims = NbScheduleOutputDims;
  OSLStmt->scattering->nb_input_dims = NbScheduleInputDims;
  OSLStmt->scattering->nb_local_dims = 0;
  OSLStmt->scattering->nb_parameters = NbScheduleParams;

  for (size_t I = 0; I < Matrix2.size(); ++I) {
    for (size_t J = 0; J < Matrix2[I].size(); ++J) {
      osl_int_set_si(OSL_PRECISION_DP, OSLStmt->scattering->m[I] + J,
                     Matrix2[I][J]);
    }
  }

  errs() << "after schedule read\n";

  // Accesses
  // Parameters
  auto NbAccessParams = extractDimValue(Schedule, isl::dim::param);

  // Input dimensions
  auto NbAccessInputDims = extractDimValue(Schedule, isl::dim::in);

  // Output dimensions
  // auto NbAccessOutputDims = extractDimValue(Schedule, isl::dim::out);

  for (auto *MA : Stmt) {
    osl_relation_list_p A = osl_relation_list_malloc();

    isl::map AccessMap = MA->getAccessRelation();
    std::vector<std::vector<long long>> Matrix3;
    isl::basic_map_list BasicAccessMapList = AccessMap.get_basic_map_list();

    errs() << unsignedFromIslSize(BasicAccessMapList.size())
           << " basic access maps\n";
    for (auto BM : BasicAccessMapList) {
      errs() << "  " << BM << "\n";
    }

    errs() << AccessMap << "\n";
    BasicAccessMapList.foreach ([&](isl::basic_map BM) -> isl::stat {
      std::vector<long long> Row(2 + 1 /* Array*/ +
                                     extractDimValue(AccessMap, isl::dim::out) +
                                     NbAccessInputDims + NbAccessParams,
                                 0);

      auto ArrayName = AccessMap.get_tuple_id(isl::dim::out).get_name();
      Row[1] = -1;
      Row[Row.size() - 1] = ArraysNameToId[ArrayName];
      Matrix3.push_back(std::move(Row));

      isl_basic_map *BasicMapC = BM.copy();
      isl_basic_map_foreach_constraint(BasicMapC, &readMapConstraintsAccess,
                                       &Matrix3);
      isl_basic_map_free(BasicMapC);
      return isl::stat::ok();
    });

    std::set<unsigned> ToRemove;
    // remove identical rows
    for (size_t I = 0; I < Matrix3.size(); ++I) {
      for (size_t J = I + 1; J < Matrix3.size(); ++J) {
        if (Matrix3[I] == Matrix3[J]) {
          ToRemove.insert(J);
        }
      }
    }

    size_t CurrentIndex = 0;
    auto NewEnd =
        std::remove_if(Matrix3.begin(), Matrix3.end(), [&](const auto &) {
          bool Delete = (ToRemove.count(CurrentIndex) > 0);
          CurrentIndex++;
          return Delete;
        });

    Matrix3.erase(NewEnd, Matrix3.end());

    errs() << "Size matrix3: " << Matrix3.size() << "\n";
    std::vector<int> RowsToUse;
    for (size_t I = 0; I < Matrix3.size(); ++I) {
      bool UsedRow = false;
      for (size_t J = 1; J <= extractDimValue(AccessMap, isl::dim::out) + 1;
           ++J) {
        if (Matrix3[I][J] != 0) {
          RowsToUse.push_back(I);
          UsedRow = true;
          break;
        }
      }
      if (not UsedRow) {
        errs() << "Warning we remove some constraints from the access relation "
                  "for array "
               << AccessMap.get_tuple_id(isl::dim::out).get_name()
               << " in statement " << Stmt.getBaseName() << "    " << I << "\n";
      }
    }

    A->elt = osl_relation_malloc(RowsToUse.size(),
                                 2 + 1 /* Array*/ +
                                     extractDimValue(AccessMap, isl::dim::out) +
                                     NbAccessInputDims + NbAccessParams);
    A->elt->type = MA->isRead() ? OSL_TYPE_READ : OSL_TYPE_WRITE;
    A->elt->precision = OSL_PRECISION_DP;
    A->elt->nb_output_dims =
        1 /* Array*/ + extractDimValue(AccessMap, isl::dim::out);
    A->elt->nb_input_dims = NbAccessInputDims;
    A->elt->nb_local_dims = 0;
    A->elt->nb_parameters = NbAccessParams;

    for (size_t I = 0; I < RowsToUse.size(); ++I) {
      for (size_t J = 0; J < Matrix3[I].size(); ++J) {
        osl_int_set_si(OSL_PRECISION_DP, A->elt->m[I] + J,
                       Matrix3[RowsToUse[I]][J]);
      }
    }
    for (auto R : RowsToUse) {
      errs() << "Row: ";
      for (auto V : Matrix3[R]) {
        errs() << V << " ";
      }
      errs() << "\n";
    }

    osl_body_p Body = osl_body_malloc();
    Body->iterators = osl_strings_malloc();
    isl::space Space = Domain.get_space();
    auto NbIterators = extractDimValue(Space, isl::dim::set);
    isl_space *SpaceC = Space.copy();

    for (unsigned int I = 0; I < NbIterators; I++) {
      std::string Name = "i" + std::to_string(I);
      osl_strings_add(Body->iterators, Name.c_str());
    }
    isl_space_free(SpaceC);

    Body->expression = osl_strings_malloc();

    osl_strings_add(
        Body->expression,
        std::string(Stmt.getBaseName() + std::string("\0")).c_str());

    OSLStmt->extension = osl_generic_malloc();
    OSLStmt->extension->interface = osl_body_interface();
    OSLStmt->extension->data = Body;

    errs() << "Adding access relation\n";
    osl_relation_list_print(stderr, A);
    osl_relation_list_add(&OSLStmt->access, A);
    errs() << "Access relation added\n";
  }
  return OSLStmt;
}

SmallDenseMap<StringRef, int> addArraysExtension(osl_scop_p OSLScop, Scop &S) {
  if (not OSLScop)
    llvm_unreachable("OSLScop is null, cannot add arrays extension.");

  SmallDenseMap<StringRef, int, 4> ArraysMap;

  OSLScop->extension = osl_generic_malloc();
  OSLScop->extension->interface = osl_arrays_interface();
  osl_arrays_p Arrays = osl_arrays_malloc();
  unsigned NbArrays = std::distance(S.array_begin(), S.array_end());
  Arrays->nb_names = NbArrays;
  int *Id = (int *)malloc(sizeof(int) * NbArrays);
  char **Names = (char **)malloc(sizeof(char *) * NbArrays);

  int Counter = 0;
  for (auto &SAI : S.arrays()) {
    auto Name = SAI->getName();
    Id[Counter] = Counter + 1;
    Names[Counter] = (char *)malloc(Name.size() + 1);
    strcpy(Names[Counter], Name.c_str());
    ArraysMap[StringRef(Names[Counter], Name.size())] = Counter + 1;
    Counter++;
  }
  Arrays->id = Id;
  Arrays->names = Names;
  OSLScop->extension->data = Arrays;

  return ArraysMap;
}

} // namespace

namespace {
enum class EquationType { Equality, Inequality };

template <EquationType T>
__isl_give isl_mat *extractEquationsOSL(isl::ctx Ctx, osl_relation_p Relation) {
  int NbCol = Relation->nb_columns;

  std::vector<int> RowIndexes;
  for (int I = 0; I < Relation->nb_rows; I++) {
    if constexpr (T == EquationType::Equality) {
      if (osl_int_zero(Relation->precision, Relation->m[I][0])) {
        RowIndexes.push_back(I);
      }
    } else { // T == EquationType::Inequality
      if (not osl_int_zero(Relation->precision, Relation->m[I][0])) {
        RowIndexes.push_back(I);
      }
    }
  }

  isl_mat *Eq = isl_mat_alloc(Ctx.get(), RowIndexes.size(), NbCol - 1);

  if (RowIndexes.empty())
    return Eq;

  for (size_t I = 0; I < RowIndexes.size(); ++I) {
    for (int J = 0; J < NbCol - 1; ++J) {
      isl::val V = isl::val(Ctx, 0);
      Eq = isl_mat_set_element_val(Eq, I, J, V.release());
    }
  }

  for (size_t I = 0; I < RowIndexes.size(); ++I) {
    for (int J = 0; J < NbCol - 1; ++J) {
      int Row = RowIndexes[I];
      long Val = osl_int_get_si(Relation->precision, Relation->m[Row][1 + J]);
      isl::val ISLVal = isl::val(Ctx, Val);
      Eq = isl_mat_set_element_val(Eq, I, J, ISLVal.release());
    }
  }

  return Eq;
}

isl::set islDomainStmtFromOsl(osl_relation_p OSLDomain, isl::space Space) {
  isl::ctx Ctx = Space.ctx();

  isl_mat *Ineq = extractEquationsOSL<EquationType::Inequality>(Ctx, OSLDomain);
  isl_mat *Eq = extractEquationsOSL<EquationType::Equality>(Ctx, OSLDomain);

  isl::basic_set BasicSet = isl::manage(isl_basic_set_from_constraint_matrices(
      Space.copy(), Eq, Ineq, isl_dim_set, isl_dim_div, isl_dim_param,
      isl_dim_cst));

  return isl::set(BasicSet);
}

isl::set islDomainStmtFromOsl(osl_statement_p OSLStmt, osl_scop_p OSLScop,
                              Scop &S) {
  int NbIterator = osl_statement_get_nb_iterators(OSLStmt);
  int NbParameter = osl_scop_get_nb_parameters(OSLScop);

  auto NbParameterScop = static_cast<int>(
      unsignedFromIslSize(S.getParamSpace().dim(isl::dim::param)));
  if (NbParameter != NbParameterScop)
    llvm_unreachable(
        "Number of parameters into scop and openscop are different");

  isl::space Space = S.getParamSpace();
  Space = Space.add_dims(isl::dim::set, NbIterator);

  if (NbIterator > 0) {
    osl_body_p StmtBody =
        (osl_body_p)osl_generic_lookup(OSLStmt->extension, OSL_URI_BODY);
    if (StmtBody) {
      for (int I = 0; I < NbIterator; I++) {
        Space = Space.set_dim_id(
            isl::dim::set, I,
            isl::id(S.getIslCtx(), StmtBody->iterators->string[I]));
      }
    }
  }

  return islDomainStmtFromOsl(OSLStmt->domain, Space);
}

isl::map islScheduleStmtFromOsl(osl_statement_p OSLStmt,
                                osl_statement_p OldOSLStmt, isl::space Space) {
  isl::ctx Ctx = Space.ctx();

  isl_mat *Ineq =
      extractEquationsOSL<EquationType::Inequality>(Ctx, OSLStmt->scattering);
  isl_mat *Eq =
      extractEquationsOSL<EquationType::Equality>(Ctx, OSLStmt->scattering);

  isl::basic_map BasicMap = isl::manage(isl_basic_map_from_constraint_matrices(
      Space.copy(), Eq, Ineq, isl_dim_out, isl_dim_in, isl_dim_div,
      isl_dim_param, isl_dim_cst));

  return isl::map(BasicMap);
}

isl::map islScheduleStmtFromOsl(osl_statement_p OSLStmt, osl_scop_p OSLScop,
                                Scop &S, ScopStmt &Stmt,
                                osl_statement_p OldOSLStmt) {
  int NbParameter = osl_scop_get_nb_parameters(OSLScop);
  int NbIterator = osl_statement_get_nb_iterators(OSLStmt);
  int NbOutput = OSLStmt->scattering->nb_output_dims;

  auto NbParameterScop = static_cast<int>(
      unsignedFromIslSize(S.getParamSpace().dim(isl::dim::param)));
  if (NbParameter != NbParameterScop)
    llvm_unreachable(
        "Number of parameters into scop and openscop are different");

  isl::space Space = S.getParamSpace();
  Space = Space.add_dims(isl::dim::in, NbIterator);
  Space = Space.add_dims(isl::dim::out, NbOutput);

  if (NbIterator > 0) {
    osl_body_p StmtBody =
        (osl_body_p)osl_generic_lookup(OSLStmt->extension, OSL_URI_BODY);
    if (StmtBody) {
      for (int I = 0; I < NbIterator; I++) {
        Space = Space.set_dim_id(
            isl::dim::in, I,
            isl::id(S.getIslCtx(), StmtBody->iterators->string[I]));
      }
    }
  }

  return islScheduleStmtFromOsl(OSLStmt, OldOSLStmt, Space);
}

ScopStmt &getStmtByName(Scop &S, StringRef Name) {
  for (ScopStmt &Stmt : S) {
    if (not strcmp(Stmt.getBaseName(), Name.data())) {
      return Stmt;
    }
  }
  llvm_unreachable("Statement not found");
}

SmallVector<std::pair<int, int>, 4>
linkIteratorsBetweenPlutoAndPolly(osl_statement_p OSLStmt,
                                  osl_statement_p OldOSLStmt) {
  SmallVector<std::pair<int, int>, 4> Links;
  osl_body_p Body =
      (osl_body_p)osl_generic_lookup(OSLStmt->extension, OSL_URI_BODY);

  osl_body_p OldBody =
      (osl_body_p)osl_generic_lookup(OldOSLStmt->extension, OSL_URI_BODY);

  for (int I = 0; I < OSLStmt->domain->nb_output_dims; ++I) {
    std::string PlutoIterName = Body->iterators->string[I];
    for (int J = 0; J < OldOSLStmt->domain->nb_output_dims; ++J) {
      std::string PollyIterName = OldBody->iterators->string[J];
      if (PlutoIterName == PollyIterName) {
        Links.push_back(std::make_pair(I, J));
      }
    }
  }

  for (auto Link : Links) {
    LLVM_DEBUG(errs() << "Final Link: Pluto iterator " << Link.first
                      << " with Polly iterator " << Link.second << "\n");
  }

  return Links;
}

isl::map convertPlutotransformationToPolly(isl::set PlutoDomain,
                                           isl::map PlutoScattering,
                                           isl::set PollyDomain,
                                           osl_statement_p OSLStmt,
                                           osl_statement_p OldOSLStmt) {
  LLVM_DEBUG(errs() << "\n\nTest function\n");
  LLVM_DEBUG(errs() << "Polly Domain: " << PollyDomain << "\n");
  LLVM_DEBUG(errs() << "Pluto Domain: " << PlutoDomain << "\n");
  LLVM_DEBUG(errs() << "Pluto Scattering: " << PlutoScattering << "\n\n");

  isl::map MapPollyDomain = isl::map::from_domain(PollyDomain);
  LLVM_DEBUG(errs() << "Map Polly Domain: " << MapPollyDomain << "\n");

  isl::map MapPlutoDomain = isl::map::from_domain(PlutoDomain);
  LLVM_DEBUG(errs() << "Map Pluto Domain: " << MapPlutoDomain << "\n");

  //
  isl::map MapPollyDomainInv = MapPollyDomain.reverse();
  LLVM_DEBUG(errs() << "Map Pluto Domain Inverse: " << MapPollyDomainInv
                    << "\n");

  //
  isl::map MapNewToOldBroken = MapPlutoDomain.apply_range(MapPollyDomainInv);
  LLVM_DEBUG(errs() << "\nMap New to Old Broken: " << MapNewToOldBroken
                    << "\n");

  isl::space Space = MapNewToOldBroken.get_space();
  LLVM_DEBUG(errs() << "Space: " << Space << "\n");

  isl::map FixMap = isl::map::universe(Space);
  auto LinkedIterators = linkIteratorsBetweenPlutoAndPolly(OSLStmt, OldOSLStmt);
  for (auto Link : LinkedIterators)
    FixMap =
        FixMap.equate(isl::dim::in, Link.first, isl::dim::out, Link.second);

  LLVM_DEBUG(errs() << "Fix Map: " << FixMap << "\n");

  isl::map MapNewToOld = MapNewToOldBroken.intersect(FixMap);
  LLVM_DEBUG(errs() << "Map New to Old: " << MapNewToOld << "\n\n");

  isl::map MapIntermediaire = MapNewToOld.reverse();
  LLVM_DEBUG(errs() << "Map Intermediaire: " << MapIntermediaire << "\n");

  isl::map Scattering = MapIntermediaire.apply_range(PlutoScattering);
  LLVM_DEBUG(errs() << "Scattering: " << Scattering << "\n");

  isl::map ScatteringClean = Scattering.gist_domain(PollyDomain);
  LLVM_DEBUG(errs() << "Scattering Clean: " << ScatteringClean << "\n");

  return ScatteringClean;
}
} // namespace

void OpenSCoPExportPass::exportOpenScop(Scop &S, std::string FileName) {
  osl_scop_p OSLScop = osl_scop_malloc();
  OSLScop->version = 1;
  OSLScop->language = strdup("C++");

  auto ArraysNameToId = addArraysExtension(OSLScop, S);

  // Context
  auto Context = S.getDefinedBehaviorContext();

  osl_relation_p OSLContext = convertContext(Context, S);
  OSLScop->context = OSLContext;

  // Parameters
  OSLScop->parameters = osl_generic_malloc();
  OSLScop->parameters->interface = osl_strings_interface();
  osl_strings_p Str = osl_strings_malloc();
  for (int I = 0; I < OSLScop->context->nb_parameters; I++) {
    isl::id Id = Context.get_dim_id(isl::dim::param, I);
    if (Id.is_null()) {
      return;
    }
    osl_strings_add(Str, isl_id_get_name(Id.get()));
  }
  OSLScop->parameters->data = Str;

  // Statements
  for (ScopStmt &Stmt : S) {
    osl_statement_p OSLStmt = convertStmt(Stmt, ArraysNameToId);
    osl_statement_add(&OSLScop->statement, OSLStmt);
  }

  FILE *File = std::fopen(FileName.c_str(), "w");
  if (!File)
    llvm_unreachable("OpenSCoPExportPass::exportOpenScop Could not open the "
                     "OpenSCoP file for writing.");

  osl_scop_print(File, OSLScop);
  osl_scop_free(OSLScop);

  std::fclose(File);
}

PreservedAnalyses OpenSCoPExportPass::run(Scop &S, ScopAnalysisManager &SAM,
                                          ScopStandardAnalysisResults &SAR,
                                          SPMUpdater &) {
  LLVM_DEBUG(errs() << "Exporting OpenScop for SCoP '" << S.getNameStr()
                    << "\n");

  std::string FileName = getFileName(S, "", "scop");
  exportOpenScop(S, FileName);

  LLVM_DEBUG(errs() << "End of exporting OpenScop for SCoP '" << S.getNameStr()
                    << "\n");
  return PreservedAnalyses::all();
}

void OpenSCoPImportPass::importOpenScop(Scop &S, std::string FileName,
                                        std::string OldFileName) {
  LLVM_DEBUG(errs() << "Importing OpenScop file: " << FileName << "\n");

  FILE *File = std::fopen(FileName.c_str(), "r");
  if (!File)
    llvm_unreachable("OpenSCoPImportPass::importOpenScop Could not open the "
                     "OpenSCoP file for writing.");

  osl_scop_p OSLScop = osl_scop_read(File);

  std::fclose(File);

  if (!OSLScop)
    llvm_unreachable(
        "OpenSCoPImportPass::importOpenScop Could not read the OpenSCoP file.");

  //
  File = std::fopen(OldFileName.c_str(), "r");
  if (!File)
    llvm_unreachable("OpenSCoPImportPass::importOpenScop Could not open the "
                     "OpenSCoP file for writing.");

  osl_scop_p OldOSLScop = osl_scop_read(File);

  std::fclose(File);

  if (!OldOSLScop)
    llvm_unreachable(
        "OpenSCoPImportPass::importOpenScop Could not read the OpenSCoP file.");

  isl_ctx *Ctx = S.getIslCtx().get();

  int StmtIndex = 0;
  isl::union_map USchedule = isl::union_map::empty(Ctx);
  for (osl_statement_p OSLStmt = OSLScop->statement,
                       OldOSLStmt = OldOSLScop->statement;
       OSLStmt; OSLStmt = OSLStmt->next, OldOSLStmt = OldOSLStmt->next) {
    // Statement name
    std::string StmtName;
    osl_body_p StmtBody =
        (osl_body_p)osl_generic_lookup(OSLStmt->extension, OSL_URI_BODY);
    if (StmtBody) {
      StmtName = StmtBody->expression->string[0];
    }
    ScopStmt &Stmt = getStmtByName(S, StmtName);
    isl::id Id = Stmt.getDomainId();

    // Statement domain
    isl::set PlutoDomain = islDomainStmtFromOsl(OSLStmt, OSLScop, S);
    PlutoDomain = PlutoDomain.set_tuple_id(Id);
    LLVM_DEBUG(errs() << "Pluto Domain " << PlutoDomain << "\n");
    LLVM_DEBUG(errs() << "Polly Domain " << Stmt.getDomain() << "\n");

    // Statement scattering
    isl::map PlutoScattering =
        islScheduleStmtFromOsl(OSLStmt, OSLScop, S, Stmt, OldOSLStmt);
    PlutoScattering = PlutoScattering.set_tuple_id(isl::dim::in, Id);
    LLVM_DEBUG(errs() << "Pluto Scattering " << PlutoScattering << "\n");

    PlutoScattering = convertPlutotransformationToPolly(
        PlutoDomain, PlutoScattering, Stmt.getDomain(), OSLStmt, OldOSLStmt);
    LLVM_DEBUG(errs() << "Converted Scattering " << PlutoScattering << "\n");

    // Statement schedule
    isl::map NewSchedule = PlutoScattering.intersect_domain(Stmt.getDomain());

    // useful to order statements in the same order as in the openscop when
    // they have the same scattering
    {
      unsigned OutDims = unsignedFromIslSize(NewSchedule.dim(isl::dim::out));
      NewSchedule = NewSchedule.add_dims(isl::dim::out, 1);
      isl::local_space LSpace(NewSchedule.get_space());
      isl::constraint Eq = isl::constraint::alloc_equality(LSpace);
      Eq = Eq.set_coefficient_si(isl::dim::out, OutDims, 1);
      Eq = Eq.set_constant_si(-StmtIndex);
      NewSchedule = NewSchedule.add_constraint(Eq);
      StmtIndex++;
    }

    isl::union_map UMap = isl::union_map(NewSchedule);

    USchedule = USchedule.unite(UMap);
  }

  S.setSchedule(USchedule);
  osl_scop_free(OSLScop);

  LLVM_DEBUG(errs() << "Importing OpenScop file done.\n");
}

PreservedAnalyses OpenSCoPImportPass::run(Scop &S, ScopAnalysisManager &SAM,
                                          ScopStandardAnalysisResults &SAR,
                                          SPMUpdater &) {
  LLVM_DEBUG(errs() << "Importing OpenScop for SCoP '" << S.getNameStr()
                    << "\n");

  std::string FileName = getFileName(S, "", "scop");
  std::string OldFileName = getFileName(S, "old_", "scop");

  importOpenScop(S, FileName, OldFileName);

  LLVM_DEBUG(errs() << "End of importOpenScop\n");
  return PreservedAnalyses::all();
}
