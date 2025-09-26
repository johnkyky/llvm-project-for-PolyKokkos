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

  Row.push_back(isl_constraint_is_equality(Constraint) ? 0 : 1);

  Row.push_back(0);

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

template <typename ISLType>
unsigned extractDimValue(ISLType &ISLObjet, isl::dim Dim) {
  static_assert(std::is_same<ISLType, isl::set>::value ||
                    std::is_same<ISLType, isl::space>::value ||
                    std::is_same<ISLType, isl::map>::value,
                "extractDimValue is only defined for isl::set or isl::map");

  isl::size ISLNbParams = ISLObjet.dim(Dim);
  return unsignedFromIslSize(ISLNbParams);
}

osl_statement_p convertStmt(ScopStmt &Stmt,
                            SmallDenseMap<StringRef, int> &ArraysNameToId) {
  osl_statement_p OSLStmt = osl_statement_malloc();

  // Domain
  isl::set Domain = Stmt.getDomain();
  isl::basic_set_list BasicSetList = Domain.get_basic_set_list();
  auto Size = unsignedFromIslSize(BasicSetList.size());

  if (Size > 1) {
    errs() << "Warning: Domain has more than one basic set, only the first one "
              "will be processed.\n";
  }

  std::vector<std::vector<long long>> Matrix;
  BasicSetList.foreach ([&](isl::basic_set BasicSet) -> isl::stat {
    isl_basic_set *BasicSetC = BasicSet.copy();
    isl_basic_set_foreach_constraint(BasicSetC, &readSetConstraints, &Matrix);
    isl_basic_set_free(BasicSetC);

    return isl::stat::ok();
  });

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

  // Accesses
  // Parameters
  auto NbAccessParams = extractDimValue(Schedule, isl::dim::param);

  // Input dimensions
  auto NbAccessInputDims = extractDimValue(Schedule, isl::dim::in);

  // Output dimensions
  // auto NbAccessOutputDims = extractDimValue(Schedule, isl::dim::out);

  for (auto *MA : Stmt) {
    osl_relation_list_p A = osl_relation_list_malloc();

    auto AccessMap = MA->getAccessRelation();
    std::vector<std::vector<long long>> Matrix3;
    isl::basic_map_list BasicAccessMapList = AccessMap.get_basic_map_list();

    BasicAccessMapList.foreach ([&](isl::basic_map BM) -> isl::stat {
      std::vector<long long> Row(2 + 1 /* Array*/ + MA->getNumSubscripts() +
                                     NbAccessInputDims + NbAccessParams,
                                 0);

      auto ArrayName = MA->getScopArrayInfo()->getName();
      Row[1] = -1;
      Row[Row.size() - 1] = ArraysNameToId[ArrayName];
      Matrix3.push_back(std::move(Row));

      isl_basic_map *BasicMapC = BM.copy();
      isl_basic_map_foreach_constraint(BasicMapC, &readMapConstraintsAccess,
                                       &Matrix3);
      isl_basic_map_free(BasicMapC);
      return isl::stat::ok();
    });

    A->elt = osl_relation_malloc(Matrix3.size(),
                                 2 + 1 /* Array*/ + MA->getNumSubscripts() +
                                     NbAccessInputDims + NbAccessParams);
    A->elt->type = MA->isRead() ? OSL_TYPE_READ : OSL_TYPE_WRITE;
    A->elt->precision = OSL_PRECISION_DP;
    A->elt->nb_output_dims = 1 /* Array*/ + MA->getNumSubscripts();
    A->elt->nb_input_dims = NbAccessInputDims;
    A->elt->nb_local_dims = 0;
    A->elt->nb_parameters = NbAccessParams;

    for (size_t I = 0; I < Matrix3.size(); ++I) {
      for (size_t J = 0; J < Matrix3[I].size(); ++J) {
        osl_int_set_si(OSL_PRECISION_DP, A->elt->m[I] + J, Matrix3[I][J]);
      }
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

    osl_relation_list_add(&OSLStmt->access, A);
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
  int NbRow = Relation->nb_rows;

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

  for (int I = 0; I < NbRow; ++I) {
    for (int J = 0; J < NbCol - 1; ++J) {
      isl::val V = isl::val(Ctx, 0);
      Eq = isl_mat_set_element_val(Eq, I, J, V.release());
    }
  }

  for (int I = 0; I < NbRow; ++I) {
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

isl::map islScheduleStmtFromOsl(osl_relation_p Scattering, isl::space Space) {
  isl::ctx Ctx = Space.ctx();

  isl_mat *Ineq =
      extractEquationsOSL<EquationType::Inequality>(Ctx, Scattering);
  isl_mat *Eq = extractEquationsOSL<EquationType::Equality>(Ctx, Scattering);

  isl::basic_map BasicMap = isl::manage(isl_basic_map_from_constraint_matrices(
      Space.copy(), Eq, Ineq, isl_dim_out, isl_dim_in, isl_dim_div,
      isl_dim_param, isl_dim_cst));

  return isl::map(BasicMap);
}

isl::map islScheduleStmtFromOsl(osl_statement_p OSLStmt, osl_scop_p OSLScop,
                                Scop &S) {
  int NbIterator = osl_statement_get_nb_iterators(OSLStmt);
  int NbParameter = osl_scop_get_nb_parameters(OSLScop);
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

  return islScheduleStmtFromOsl(OSLStmt->scattering, Space);
}

isl::map islAccessFromOsl(osl_relation_p Scattering, isl::space Space) {
  isl::ctx Ctx = Space.ctx();

  isl_mat *Ineq =
      extractEquationsOSL<EquationType::Inequality>(Ctx, Scattering);
  isl_mat *Eq = extractEquationsOSL<EquationType::Equality>(Ctx, Scattering);

  // Remove osl column and row for array identifier
  Eq = isl_mat_drop_rows(Eq, 0, 1);
  Eq = isl_mat_drop_cols(Eq, 0, 1);
  Ineq = isl_mat_drop_cols(Ineq, 0, 1);

  isl::basic_map BasicMap = isl::manage(isl_basic_map_from_constraint_matrices(
      Space.copy(), Eq, Ineq, isl_dim_out, isl_dim_in, isl_dim_div,
      isl_dim_param, isl_dim_cst));

  return isl::map(BasicMap);
}

isl::map islAccessFromOSL(osl_relation_p OSLAccess, polly::MemoryAccess *MA,
                          Scop &S, osl_scop_p OSLScop,
                          osl_statement_p OSLStmt) {
  isl::id Id = MA->getId();

  int NbParameter = osl_scop_get_nb_parameters(OSLScop);
  int NbIterator = osl_statement_get_nb_iterators(OSLStmt);
  int NbOutput = MA->getNumSubscripts();

  auto NbParameterScop = static_cast<int>(
      unsignedFromIslSize(S.getParamSpace().dim(isl::dim::param)));
  if (NbParameter != NbParameterScop)
    llvm_unreachable(
        "Number of parameters into scop and openscop are different");

  isl::space Space = S.getContext().space();
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

  return islAccessFromOsl(OSLAccess, Space);
}

ScopStmt &getStmtByName(Scop &S, StringRef Name) {
  for (ScopStmt &Stmt : S) {
    if (not strcmp(Stmt.getBaseName(), Name.data())) {
      return Stmt;
    }
  }
  llvm_unreachable("Statement not found");
}

[[maybe_unused]] void importOpenScopTest(Scop &S, osl_scop_p OSLScop,
                                         std::string FileName) {
  errs() << "Test importOpenScopTest\n";

  isl_ctx *Ctx = S.getIslCtx().get();
  errs() << "Domain: " << S.getDomains() << "\n";

  // Space

  isl::space Space = isl::space::params_alloc(Ctx, 2);
  Space = Space.set_dim_id(isl::dim::param, 0, "N");
  Space = Space.set_dim_id(isl::dim::param, 1, "M");

  // Context
  isl::set Context = isl::set::universe(Space);
  isl::local_space LocalSpace = isl::local_space(Space);
  isl::constraint C01 = isl::constraint::alloc_inequality(LocalSpace);
  C01 = C01.set_coefficient_si(isl::dim::param, 0, 1);
  Context = Context.add_constraint(C01);
  isl::constraint C02 = isl::constraint::alloc_inequality(LocalSpace);
  C02 = C02.set_coefficient_si(isl::dim::param, 1, 1);
  Context = Context.add_constraint(C02);

  errs() << "Context: " << Context << "\n";

  // Stmt1
  isl::space Space1 = isl::space(Ctx, 2, 2);
  Space1 = Space1.set_dim_id(isl::dim::set, 0, "i");
  Space1 = Space1.set_dim_id(isl::dim::set, 1, "j");
  Space1 = Space1.set_dim_id(isl::dim::param, 0, "N");
  Space1 = Space1.set_dim_id(isl::dim::param, 1, "M");
  errs() << "Space1: " << Space1 << "\n";

  isl::set Set1 = isl::set::universe(Space1);
  isl::local_space LocalSpace1 = isl::local_space(Space1);
  isl::constraint C1 = isl::constraint::alloc_inequality(LocalSpace1);
  C1 = C1.set_coefficient_si(isl::dim::set, 0, 1);
  isl::constraint C2 = isl::constraint::alloc_inequality(LocalSpace1);
  C2 = C2.set_coefficient_si(isl::dim::set, 0, -1);
  C2 = C2.set_coefficient_si(isl::dim::param, 0, 1);
  isl::constraint C3 = isl::constraint::alloc_inequality(LocalSpace1);
  C3 = C3.set_coefficient_si(isl::dim::set, 1, 1);
  isl::constraint C4 = isl::constraint::alloc_inequality(LocalSpace1);
  C4 = C4.set_coefficient_si(isl::dim::set, 1, -1);
  C4 = C4.set_coefficient_si(isl::dim::param, 1, 1);

  Set1 = Set1.add_constraint(C1);
  Set1 = Set1.add_constraint(C2);
  Set1 = Set1.add_constraint(C3);
  Set1 = Set1.add_constraint(C4);

  isl::id Id1 = isl::id::alloc(Ctx, "chien", nullptr);
  Set1 = Set1.set_tuple_id(Id1);

  errs() << "Set1: " << Set1 << "\n";

  isl::union_set USet1 = isl::union_set(Set1);
  errs() << "USet1: " << USet1 << "\n";

  auto StmtSchedule1 = isl::schedule::from_domain(USet1);
  errs() << "Initial schedule 1: " << StmtSchedule1 << "\n";

  // Stmt2
  isl::space Space2 = isl::space(Ctx, 2, 2);
  Space2 = Space2.set_dim_id(isl::dim::set, 0, "i");
  Space2 = Space2.set_dim_id(isl::dim::set, 1, "j");
  Space2 = Space2.set_dim_id(isl::dim::param, 0, "N");
  Space2 = Space2.set_dim_id(isl::dim::param, 1, "M");
  errs() << "Space2: " << Space2 << "\n";

  isl::set Set2 = isl::set::universe(Space2);
  isl::local_space LocalSpace2 = isl::local_space(Space2);
  C1 = isl::constraint::alloc_inequality(LocalSpace1);
  C1 = C1.set_coefficient_si(isl::dim::set, 0, 1);
  C2 = isl::constraint::alloc_inequality(LocalSpace1);
  C2 = C2.set_coefficient_si(isl::dim::set, 0, -1);
  C2 = C2.set_coefficient_si(isl::dim::param, 0, 1);

  C3 = isl::constraint::alloc_inequality(LocalSpace1);
  C3 = C3.set_coefficient_si(isl::dim::set, 1, 1);
  C4 = isl::constraint::alloc_inequality(LocalSpace1);
  C4 = C4.set_coefficient_si(isl::dim::set, 1, -1);
  C4 = C4.set_coefficient_si(isl::dim::param, 1, 1);

  Set2 = Set2.add_constraint(C1);
  Set2 = Set2.add_constraint(C2);
  Set2 = Set2.add_constraint(C3);
  Set2 = Set2.add_constraint(C4);

  isl::id Id2 = isl::id::alloc(Ctx, "chat", nullptr);
  Set2 = Set2.set_tuple_id(Id2);

  errs() << "Set2: " << Set2 << "\n";

  isl::union_set USet2 = isl::union_set(Set2);
  errs() << "USet2: " << USet2 << "\n";

  auto StmtSchedule2 = isl::schedule::from_domain(USet2);
  errs() << "Initial schedule 2: " << StmtSchedule2 << "\n";

  isl::schedule StmtSchedule = StmtSchedule1.sequence(StmtSchedule2);
  errs() << "Final schedule: " << StmtSchedule << "\n";

  //////////
  auto MapToDimension = [&](isl::union_set Domain, unsigned N) {
    errs() << "Domain: " << Domain << "\n";

    auto Result = isl::union_pw_multi_aff::empty(Domain.get_space());
    errs() << "\n\nResult: " << Result << "\n";

    for (isl::set S : Domain.get_set_list()) {
      errs() << "Set: " << S << "\n";
      unsigned Dim = unsignedFromIslSize(S.tuple_dim());

      auto PMA = isl::pw_multi_aff::project_out_map(S.get_space(),
                                                    isl::dim::set, N, Dim - N);
      errs() << "PMA: " << PMA << "\n";
      if (N > 1)
        PMA = PMA.drop_dims(isl::dim::out, 0, N - 1);

      Result = Result.add_pw_multi_aff(PMA);
      errs() << "Result: " << Result << "\n";
    }

    auto MUPA = isl::multi_union_pw_aff(isl::union_pw_multi_aff(Result));
    errs() << "MUPA: " << MUPA << "\n";

    return MUPA;
  };

  isl::union_set Domain = StmtSchedule.get_domain();
  StmtSchedule =
      StmtSchedule.insert_partial_schedule(MapToDimension(Domain, 2));
  errs() << "Final schedule with MUPA: " << StmtSchedule << "\n";
  StmtSchedule =
      StmtSchedule.insert_partial_schedule(MapToDimension(Domain, 1));
  errs() << "Final schedule with MUPA: " << StmtSchedule << "\n";

  isl_ast_build *Build = isl_ast_build_from_context(Context.copy());
  errs() << "Context " << S.getContext() << "\n";

  isl::ast_node Root = isl::manage(
      isl_ast_build_node_from_schedule(Build, StmtSchedule.release()));

  isl_ast_build_free(Build);

  errs() << "AST root " << Root << "\n";

  errs() << "End of test importOpenScopTest\n\n\n";
}
} // namespace

void OpenSCoPExportPass::exportOpenScop(Scop &S, std::string FileName) {
  osl_scop_p OSLScop = osl_scop_malloc();
  OSLScop->version = 1;
  OSLScop->language = strdup("C++");

  auto ArraysNameToId = addArraysExtension(OSLScop, S);

  // Context
  auto C = S.getContext();

  auto NbParams = extractDimValue(C, isl::dim::param);
  OSLScop->context = osl_relation_malloc(0, 2 + NbParams);
  OSLScop->context->type = OSL_TYPE_CONTEXT;
  OSLScop->context->precision = OSL_PRECISION_DP;
  OSLScop->context->nb_output_dims = 0;
  OSLScop->context->nb_input_dims = 0;
  OSLScop->context->nb_local_dims = 0;
  OSLScop->context->nb_parameters = NbParams;

  // Parameters
  OSLScop->parameters = osl_generic_malloc();
  OSLScop->parameters->interface = osl_strings_interface();
  osl_strings_p Str = osl_strings_malloc();
  for (unsigned I = 0; I < NbParams; I++) {
    isl::id Id = C.get_dim_id(isl::dim::param, I);
    if (Id.is_null()) {
      errs() << "Error retrieving the parameter id at index " << I << ".\n";
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
    llvm_unreachable("Could not open the OpenSCoP file for writing.");

  osl_scop_print(File, OSLScop);
  osl_scop_free(OSLScop);

  std::fclose(File);
}

PreservedAnalyses OpenSCoPExportPass::run(Scop &S, ScopAnalysisManager &SAM,
                                          ScopStandardAnalysisResults &SAR,
                                          SPMUpdater &) {
  errs() << "Exporting OpenScop for SCoP '" << S.getNameStr() << "\n";

  std::string FileName = getFileName(S, "", "scop");
  exportOpenScop(S, FileName);

  errs() << "End of exporting OpenScop for SCoP '" << S.getNameStr() << "\n";
  return PreservedAnalyses::all();
}

void OpenSCoPImportPass::importOpenScop(Scop &S, std::string FileName) {
  FILE *File = std::fopen(FileName.c_str(), "r");
  if (!File)
    llvm_unreachable("Could not open the OpenSCoP file for writing.");

  osl_scop_p OSLScop = osl_scop_read(File);

  std::fclose(File);

  if (!OSLScop)
    llvm_unreachable("Could not read the OpenSCoP file.");

  isl_ctx *Ctx = S.getIslCtx().get();

  isl::union_map USchedule = isl::union_map::empty(Ctx);
  for (osl_statement_p OSLStmt = OSLScop->statement; OSLStmt;
       OSLStmt = OSLStmt->next) {
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
    isl::set NewDomain = islDomainStmtFromOsl(OSLStmt, OSLScop, S);
    NewDomain = NewDomain.set_tuple_id(Id);
    errs() << "NewDomain : " << NewDomain << "\n";

    Stmt.setDomain(NewDomain);

    //
    // Statement scattering
    isl::map NewMap = islScheduleStmtFromOsl(OSLStmt, OSLScop, S);
    NewMap = NewMap.set_tuple_id(isl::dim::in, Id);
    errs() << "NewMap : " << NewMap << "\n";

    // Memory accesses
    osl_relation_list_p OSLAccessNode = OSLStmt->access;
    for (auto *MA : Stmt) {
      osl_relation_p OSLAccess = OSLAccessNode->elt;

      isl::map NewAccessMap =
          islAccessFromOSL(OSLAccess, MA, S, OSLScop, OSLStmt);
      NewAccessMap = NewAccessMap.set_tuple_id(isl::dim::in, Id);
      isl::id ArrayId = MA->getAccessRelation().get_tuple_id(isl::dim::out);
      NewAccessMap = NewAccessMap.set_tuple_id(isl::dim::out, ArrayId);
      errs() << "Access " << NewAccessMap << "\n";

      MA->setNewAccessRelation(NewAccessMap);

      OSLAccessNode = OSLAccessNode->next;
    }

    // Statement schedule
    isl::map NewSchedule = NewMap.intersect_domain(NewDomain);
    errs() << "NewSchedule : " << NewSchedule << "\n";

    isl::union_map UMap = isl::union_map(NewSchedule);

    USchedule = USchedule.unite(UMap);

    errs() << "\n";
  }
  errs() << "\n\nUSchedule : " << USchedule << "\n\n";

  S.setSchedule(USchedule);
  osl_scop_free(OSLScop);
}

PreservedAnalyses OpenSCoPImportPass::run(Scop &S, ScopAnalysisManager &SAM,
                                          ScopStandardAnalysisResults &SAR,
                                          SPMUpdater &) {
  errs() << "Importing OpenScop for SCoP '" << S.getNameStr() << "\n";

  std::string FileName = getFileName(S, "", "scop");

  importOpenScop(S, FileName);

  errs() << "End of importOpenScop\n\n\n";
  return PreservedAnalyses::all();
}
