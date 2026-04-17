//===------ PhaseManager.cpp ------------------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "polly/Pass/PhaseManager.h"
#include "polly/CodeGen/CodeGeneration.h"
#include "polly/CodeGen/IslAst.h"
#include "polly/CodeGen/PPCGCodeGeneration.h"
#include "polly/CodePreparation.h"
#include "polly/DeLICM.h"
#include "polly/DeadCodeElimination.h"
#include "polly/DependenceInfo.h"
#include "polly/FlattenSchedule.h"
#include "polly/ForwardOpTree.h"
#include "polly/JSONExporter.h"
#include "polly/MaximalStaticExpansion.h"
#include "polly/PruneUnprofitable.h"
#include "polly/ScheduleOptimizer.h"
#include "polly/ScopDetection.h"
#include "polly/ScopDetectionDiagnostic.h"
#include "polly/ScopGraphPrinter.h"
#include "polly/ScopInfo.h"
#include "polly/Simplify.h"
#include "polly/Support/PollyDebug.h"
#include "polly/Test/ArrayFusion.h"
#include "polly/Test/ArrayReg2Mem.h"
#include "polly/Test/CheckParallelism.h"
#include "polly/Test/LoopFusion.h"
#include "polly/Test/LowerInstructionForReduction.h"
#include "polly/Test/MergeRedundantInvariantLoads.h"
#include "polly/Test/PrintParamsValue.h"
#include "polly/Test/RemoveLoopBoundCondition.h"
#include "polly/Test/RemoveLoopVersioning.h"
#include "polly/Test/ScheduleOptimizer.h"
#include "polly/Test/TriangularLoopFix.h"
#include "polly/Test/UserAssumptions.h"
#include "polly/Test/VarFusion.h"
#include "llvm/ADT/PriorityWorklist.h"
#include "llvm/Analysis/AssumptionCache.h"
#include "llvm/Analysis/OptimizationRemarkEmitter.h"
#include "llvm/Analysis/TargetTransformInfo.h"
#include "llvm/IR/Dominators.h"
#include "llvm/IR/Module.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Transforms/InstCombine/InstCombine.h"
#include "llvm/Transforms/Scalar/ADCE.h"
#include "llvm/Transforms/Scalar/ConstraintElimination.h"
#include "llvm/Transforms/Scalar/IndVarSimplify.h"
#include "llvm/Transforms/Scalar/LoopPassManager.h"
#include "llvm/Transforms/Scalar/LoopRotation.h"
#include "llvm/Transforms/Scalar/SimplifyCFG.h"
#include "llvm/Transforms/Utils/LoopUtils.h"

#define DEBUG_TYPE "polly-pass"

using namespace polly;
using namespace llvm;

namespace {

enum class Backend { CPU, GPU };
Backend readBackend(const Function &F) {
  if (F.hasFnAttribute("polly.backend")) {
    auto BackendAttr = F.getFnAttribute("polly.backend");
    std::string BackendStr = BackendAttr.getValueAsString().str();
    if (BackendStr == "Serial")
      return Backend::CPU;
    if (BackendStr == "OpenMP")
      return Backend::CPU;
    if (BackendStr == "Cuda")
      return Backend::GPU;
  }
  return Backend::CPU;
}

/// Recurse through all subregions and all regions and add them to RQ.
static void addRegionIntoQueue(Region &R, SmallVector<Region *> &RQ) {
  RQ.push_back(&R);
  for (const auto &E : R)
    addRegionIntoQueue(*E, RQ);
}

/// The phase pipeline of Polly to be embedded into another pass manager than
/// runs passes on functions.
///
/// Polly holds state besides LLVM-IR (RegionInfo and ScopInfo) between phases
/// that LLVM pass managers do not consider when scheduling analyses and passes.
/// That is, the ScopInfo must persist between phases that a pass manager must
/// not invalidate to recompute later.
class PhaseManager {
private:
  Function &F;
  FunctionAnalysisManager &FAM;
  PollyPassOptions Opts;

public:
  PhaseManager(Function &F, FunctionAnalysisManager &FAM, PollyPassOptions Opts)
      : F(F), FAM(FAM), Opts(std::move(Opts)) {}

  /// Execute Polly's phases as indicated by the options.
  bool run() {
    if (not F.hasFnAttribute("polly.findSCoP"))
      return false;
    // Get analyses from the function pass manager.
    // These must be preserved during all phases so that if processing one SCoP
    // has finished, the next SCoP can still use them. Recomputing is not an
    // option because ScopDetection stores references to the old results.
    // TODO: CodePreparation doesn't actually need these analysis, it just keeps
    // them up-to-date. If they are not computed yet, can also compute after the
    // prepare phase.
    LoopInfo &LI = FAM.getResult<LoopAnalysis>(F);
    DominatorTree &DT = FAM.getResult<DominatorTreeAnalysis>(F);
    bool ModifiedIR = false;

    // Phase: prepare
    // TODO: Setting ModifiedIR will invalidate any analysis, even if DT, LI are
    // preserved.
    if (Opts.isPhaseEnabled(PassPhase::Prepare)) {

      auto PA = createFunctionToLoopPassAdaptor(LoopRotatePass()).run(F, FAM);
      FAM.invalidate(F, PA);

      if (runCodePreparation(F, &DT, &LI, nullptr)) {
        PreservedAnalyses PA;
        PA.preserve<DominatorTreeAnalysis>();
        PA.preserve<LoopAnalysis>();
        FAM.invalidate(F, PA);
        ModifiedIR = true;
      }

      PA = ExtractAnnotatedFromLoop().run(F, FAM);
      FAM.invalidate(F, PA);
      PA = VarFusionPass().run(F, FAM);
      FAM.invalidate(F, PA);
      PA = UserAssumptions().run(F, FAM);
      FAM.invalidate(F, PA);
      PA = SimplifyCFGPass().run(F, FAM);
      FAM.invalidate(F, PA);
      PA = ConstraintEliminationPass().run(F, FAM);
      FAM.invalidate(F, PA);
      PA = createFunctionToLoopPassAdaptor(IndVarSimplifyPass()).run(F, FAM);
      FAM.invalidate(F, PA);
      PA = SimplifyCFGPass().run(F, FAM);
      FAM.invalidate(F, PA);

      PA = RemoveLoopVersioningPass().run(F, FAM);
      FAM.invalidate(F, PA);
      PA = SimplifyCFGPass().run(F, FAM);
      FAM.invalidate(F, PA);

      PA = RemoveLoopBoundConditionPass().run(F, FAM);
      FAM.invalidate(F, PA);
      PA = InstCombinePass().run(F, FAM);
      FAM.invalidate(F, PA);
      PA = SimplifyCFGPass().run(F, FAM);
      FAM.invalidate(F, PA);

      PA = LoopFusionPass().run(F, FAM);
      FAM.invalidate(F, PA);
      PA = InstCombinePass().run(F, FAM);
      FAM.invalidate(F, PA);
      PA = SimplifyCFGPass().run(F, FAM);
      FAM.invalidate(F, PA);

      PA = TriangularLoopFixPass().run(F, FAM);
      FAM.invalidate(F, PA);
      PA = InstCombinePass().run(F, FAM);
      FAM.invalidate(F, PA);
      PA = SimplifyCFGPass().run(F, FAM);
      FAM.invalidate(F, PA);
      PA = createFunctionToLoopPassAdaptor(IndVarSimplifyPass()).run(F, FAM);
      FAM.invalidate(F, PA);
      PA = InstCombinePass().run(F, FAM);
      FAM.invalidate(F, PA);
      PA = SimplifyCFGPass().run(F, FAM);
      FAM.invalidate(F, PA);

      PA = MergeRedundantInvariantLoadsPass().run(F, FAM);
      FAM.invalidate(F, PA);

      PA = ArrayFusion().run(F, FAM);
      FAM.invalidate(F, PA);
      PA = InstCombinePass().run(F, FAM);
      FAM.invalidate(F, PA);
      PA = ArrayReg2MemPass().run(F, FAM);
      FAM.invalidate(F, PA);
      PA = InstCombinePass().run(F, FAM);
      FAM.invalidate(F, PA);
      PA = SimplifyCFGPass().run(F, FAM);
      FAM.invalidate(F, PA);
      PA = ADCEPass().run(F, FAM);
      FAM.invalidate(F, PA);
      // PA = LowerInstructionForReductionPass().run(F, FAM);
      // FAM.invalidate(F, PA);
    }

    // Can't do anything without detection
    if (!Opts.isPhaseEnabled(PassPhase::Detection))
      return false;

    AAResults &AA = FAM.getResult<AAManager>(F);
    ScalarEvolution &SE = FAM.getResult<ScalarEvolutionAnalysis>(F);
    OptimizationRemarkEmitter &ORE =
        FAM.getResult<OptimizationRemarkEmitterAnalysis>(F);

    // ScopDetection is modifying RegionInfo, do not cache it, nor use a cached
    // version.
    RegionInfo &RI = FAM.getResult<RegionInfoAnalysis>(F);
    auto AnnotedSizes = ExtractAnnotatedSizes().run(F, FAM);

    LoopInfo &LI2 = FAM.getResult<LoopAnalysis>(F);
    DominatorTree &DT2 = FAM.getResult<DominatorTreeAnalysis>(F);

    // Phase: detection
    ScopDetection SD(DT2, SE, LI2, RI, AA, AnnotedSizes, ORE);

    if (not Opts.isPhaseEnabled(PassPhase::Detection) or
        (Opts.isPhaseEnabled(PassPhase::Detection) and
         (F.getMetadata("polly") or F.hasFnAttribute("polly.findSCoP")))) {
      errs() << "on fait la recherche des scop dans " << F.getName() << "\n";
      SD.detect(F);
    }

    if (Opts.isPhaseEnabled(PassPhase::PrintDetect)) {
      outs() << "Detected Scops in Function " << F.getName() << "\n";
      for (const Region *R : SD.ValidRegions)
        outs() << "Valid Region for Scop: " << R->getNameStr() << '\n';
      outs() << "\n";
    }

    if (Opts.isPhaseEnabled(PassPhase::DotScops))
      printGraphForFunction(F, &SD, "scops", false);
    if (Opts.isPhaseEnabled(PassPhase::DotScopsOnly))
      printGraphForFunction(F, &SD, "scopsonly", true);

    auto ViewScops = [&](const char *Name, bool IsSimply) {
      if (Opts.ViewFilter.empty() && !F.getName().count(Opts.ViewFilter))
        return;

      if (Opts.ViewAll || std::distance(SD.begin(), SD.end()) > 0)
        viewGraphForFunction(F, &SD, Name, IsSimply);
    };
    if (Opts.isPhaseEnabled(PassPhase::ViewScops))
      ViewScops("scops", false);
    if (Opts.isPhaseEnabled(PassPhase::ViewScopsOnly))
      ViewScops("scopsonly", true);

    // Phase: scops
    AssumptionCache AC = AssumptionAnalysis().run(F, FAM);
    const DataLayout &DL = F.getParent()->getDataLayout();
    ScopInfo Info(DL, SD, SE, LI2, AA, DT2, AC, ORE);
    if (Opts.isPhaseEnabled(PassPhase::PrintScopInfo)) {
      if (Region *TLR = RI.getTopLevelRegion()) {
        SmallVector<Region *> Regions;
        addRegionIntoQueue(*TLR, Regions);

        // reverse iteration because the regression tests expect it.
        for (Region *R : reverse(Regions)) {
          Scop *S = Info.getScop(R);
          outs() << "Printing analysis 'Polly - Create polyhedral "
                    "description of Scops' for region: '"
                 << R->getNameStr() << "' in function '" << F.getName()
                 << "':\n";
          if (S)
            outs() << *S;
          else
            outs() << "Invalid Scop!\n";
        }
      }
    }

    SmallPriorityWorklist<Region *, 4> Worklist;
    for (auto &[R, S] : Info)
      if (S)
        Worklist.insert(R);

    TargetTransformInfo &TTI = FAM.getResult<TargetIRAnalysis>(F);
    while (!Worklist.empty()) {
      Region *R = Worklist.pop_back_val();
      Scop *S = Info.getScop(R);
      if (!S) {
        // This can happen if codegenning of a previous SCoP made this region
        // not-a-SCoP anymore.
        POLLY_DEBUG(dbgs() << "SCoP in Region '" << *R << "' disappeared");
        continue;
      }

      if (!SD.isMaxRegionInScop(*R, /*Verify=*/false))
        continue;

      // Phase: flatten
      if (Opts.isPhaseEnabled(PassPhase::Flatten))
        runFlattenSchedulePass(*S);

      // Phase: deps
      // Actual analysis runs on-demand, so it does not matter whether the phase
      // is actually enabled, but use this location to print dependencies.
      DependenceAnalysis::Result DA = runDependenceAnalysis(*S);
      if (Opts.isPhaseEnabled(PassPhase::PrintDependences)) {
        assert(Opts.isPhaseEnabled(PassPhase::Dependences));
        const Dependences &D = DA.getDependences(Opts.PrintDepsAnalysisLevel);
        D.print(outs());
      }

      // Phase: import-jscop
      if (Opts.isPhaseEnabled(PassPhase::ImportJScop))
        runImportJSON(*S, DA);

      // Phase: simplify-0
      bool ModifiedSinceSimplify = true;
      if (Opts.isPhaseEnabled(PassPhase::Simplify0)) {
        runSimplify(*S, 0);
        ModifiedSinceSimplify = false;
      }

      // Phase: optree
      if (Opts.isPhaseEnabled(PassPhase::Optree)) {
        bool ModifiedByOptree = runForwardOpTree(*S);
        ModifiedSinceSimplify |= ModifiedByOptree;
      }

      // Phase: delicm
      if (Opts.isPhaseEnabled(PassPhase::DeLICM)) {
        bool ModifiedByDelicm = runDeLICM(*S);
        ModifiedSinceSimplify |= ModifiedByDelicm;
      }

      // Phase: simplify-1
      // If we have already run simplify-0, do not re-run it if the SCoP has not
      // changed since then.
      if (ModifiedSinceSimplify && Opts.isPhaseEnabled(PassPhase::Simplify1)) {
        runSimplify(*S, 1);
        ModifiedSinceSimplify = false;
      }

      // Phase: dce
      if (Opts.isPhaseEnabled(PassPhase::DeadCodeElimination))
        runDeadCodeElim(*S, DA);

      // Phase: mse
      if (Opts.isPhaseEnabled(PassPhase::MaximumStaticExtension))
        runMaximalStaticExpansion(*S, DA);

      // Phase: prune
      if (Opts.isPhaseEnabled(PassPhase::PruneUnprofitable))
        runPruneUnprofitable(*S);

      if (Opts.isPhaseEnabled(PassPhase::CheckParallelism))
        runCheckParallelism(*S);

      if (F.hasFnAttribute("polly.backend")) {
        auto BackendAttr = F.getFnAttribute("polly.backend");
        std::string Backend = BackendAttr.getValueAsString().str();
      }

      bool ModifiedByCodeGen = false;
      if (readBackend(F) == Backend::GPU) {
        ModifiedByCodeGen = runPPCGCodeGeneration(*S, LI2, DT2, SE, DL, RI);
      } else {
        // Phase: opt-isl
        if (Opts.isPhaseEnabled(PassPhase::Optimization))
          runIslScheduleOptimizer(*S, &TTI, DA);
        if (Opts.isPhaseEnabled(PassPhase::OptimizationPluto))
          runPlutoScheduleOptimizer(*S);

        // Phase: import-jscop
        if (Opts.isPhaseEnabled(PassPhase::ExportJScop))
          runExportJSON(*S);

        // Phase: ast
        // Cannot run codegen unless ast is enabled
        if (!Opts.isPhaseEnabled(PassPhase::AstGen))
          continue;
        std::unique_ptr<IslAstInfo> IslAst = runIslAstGen(*S, DA);

        // Phase: codegen
        if (!Opts.isPhaseEnabled(PassPhase::CodeGen))
          continue;
        ModifiedByCodeGen = runCodeGeneration(*S, RI, *IslAst);
      }
      if (ModifiedByCodeGen) {
        ModifiedIR = true;

        // For all regions, create new polly::Scop objects because the old ones
        // refere to invalidated LLVM-IR.
        // FIXME: Adds all SCoPs again to statistics
        Info.recompute();
      }
    }

    return ModifiedIR;
  }
};
} // namespace

StringRef polly::getPhaseName(PassPhase Phase) {
  switch (Phase) {
  case PassPhase::Prepare:
    return "prepare";
  case PassPhase::Detection:
    return "detect";
  case PassPhase::PrintDetect:
    return "print-detect";
  case PassPhase::DotScops:
    return "dot-scops";
  case PassPhase::DotScopsOnly:
    return "dot-scops-only";
  case PassPhase::ViewScops:
    return "view-scops";
  case PassPhase::ViewScopsOnly:
    return "view-scops-only";
  case PassPhase::ScopInfo:
    return "scops";
  case PassPhase::PrintScopInfo:
    return "print-scops";
  case PassPhase::Flatten:
    return "flatten";
  case PassPhase::Dependences:
    return "deps";
  case PassPhase::PrintDependences:
    return "print-deps";
  case PassPhase::ImportJScop:
    return "import-jscop";
  case PassPhase::Simplify0:
    return "simplify-0";
  case PassPhase::Optree:
    return "optree";
  case PassPhase::DeLICM:
    return "delicm";
  case PassPhase::Simplify1:
    return "simplify-1";
  case PassPhase::DeadCodeElimination:
    return "dce";
  case PassPhase::MaximumStaticExtension:
    return "mse";
  case PassPhase::PruneUnprofitable:
    return "prune";
  case PassPhase::CheckParallelism:
    return "check-parallelism";
  case PassPhase::Optimization:
    return "opt-isl"; // "opt" would conflict with the llvm executable
  case PassPhase::OptimizationPluto:
    return "opt-pluto"; // "opt" would conflict with the llvm executable
  case PassPhase::ExportJScop:
    return "export-jscop";
  case PassPhase::AstGen:
    return "ast";
  case PassPhase::CodeGen:
    return "codegen";
  default:
    llvm_unreachable("Unexpected phase");
  }
}

PassPhase polly::parsePhase(StringRef Name) {
  return StringSwitch<PassPhase>(Name)
      .Case("prepare", PassPhase::Prepare)
      .Case("detect", PassPhase::Detection)
      .Case("print-detect", PassPhase::PrintDetect)
      .Case("dot-scops", PassPhase::DotScops)
      .Case("dot-scops-only", PassPhase::DotScopsOnly)
      .Case("view-scops", PassPhase::ViewScops)
      .Case("view-scops-only", PassPhase::ViewScopsOnly)
      .Case("scops", PassPhase::ScopInfo)
      .Case("print-scops", PassPhase::PrintScopInfo)
      .Case("flatten", PassPhase::Flatten)
      .Case("deps", PassPhase::Dependences)
      .Case("print-deps", PassPhase::PrintDependences)
      .Case("import-jscop", PassPhase::ImportJScop)
      .Case("simplify-0", PassPhase::Simplify0)
      .Case("optree", PassPhase::Optree)
      .Case("delicm", PassPhase::DeLICM)
      .Case("simplify-1", PassPhase::Simplify1)
      .Case("dce", PassPhase::DeadCodeElimination)
      .Case("mse", PassPhase::MaximumStaticExtension)
      .Case("prune", PassPhase::PruneUnprofitable)
      .Case("check-parallelism", PassPhase::CheckParallelism)
      .Case("opt-isl", PassPhase::Optimization)
      .Case("opt-isl", PassPhase::OptimizationPluto)
      .Case("export-jscop", PassPhase::ExportJScop)
      .Case("ast", PassPhase::AstGen)
      .Case("codegen", PassPhase::CodeGen)
      .Default(PassPhase::None);
}

bool polly::dependsOnDependenceInfo(PassPhase Phase) {
  // Nothing before dep phase can depend on it
  if (static_cast<size_t>(Phase) <= static_cast<size_t>(PassPhase::Dependences))
    return false;

  switch (Phase) {
  case PassPhase::Simplify0:
  case PassPhase::Optree:
  case PassPhase::DeLICM:
  case PassPhase::Simplify1:
  case PassPhase::PruneUnprofitable:
  case PassPhase::ImportJScop:
  case PassPhase::ExportJScop:
  case PassPhase::AstGen: // transitively through codegen
  case PassPhase::CodeGen:
    return false;
  default:
    return true;
  }
}

void PollyPassOptions::enableEnd2End() {
  setPhaseEnabled(PassPhase::Detection);
  setPhaseEnabled(PassPhase::ScopInfo);
  setPhaseEnabled(PassPhase::Dependences);
  setPhaseEnabled(PassPhase::AstGen);
  setPhaseEnabled(PassPhase::CodeGen);
}

void PollyPassOptions::enableDefaultOpts() {
  setPhaseEnabled(PassPhase::Prepare);
  setPhaseEnabled(PassPhase::Simplify0);
  setPhaseEnabled(PassPhase::Optree);
  setPhaseEnabled(PassPhase::DeLICM);
  setPhaseEnabled(PassPhase::Simplify1);
  setPhaseEnabled(PassPhase::PruneUnprofitable);
  setPhaseEnabled(PassPhase::Optimization);
}

void PollyPassOptions::disableAfter(PassPhase Phase) {
  assert(Phase != PassPhase::None);
  for (PassPhase P : enum_seq_inclusive(Phase, PassPhase::PassPhaseLast)) {
    if (P == Phase)
      continue;
    setPhaseEnabled(P, false);
  }
}

Error PollyPassOptions::checkConsistency() const {
  for (PassPhase P : enum_seq_inclusive(PassPhase::PassPhaseFirst,
                                        PassPhase::PassPhaseLast)) {
    if (!isPhaseEnabled(P))
      continue;

    // Prepare and Detection have no requirements
    if (P == PassPhase::Prepare || P == PassPhase::Detection)
      continue;

    if (!isPhaseEnabled(PassPhase::Detection))
      return make_error<StringError>(
          formatv("'{0}' requires 'detect' to be enabled", getPhaseName(P))
              .str(),
          inconvertibleErrorCode());

    if (static_cast<size_t>(P) < static_cast<size_t>(PassPhase::ScopInfo))
      continue;

    if (!isPhaseEnabled(PassPhase::ScopInfo))
      return make_error<StringError>(
          formatv("'{0}' requires 'scops' to be enabled", getPhaseName(P))
              .str(),
          inconvertibleErrorCode());

    if (dependsOnDependenceInfo(P) && !isPhaseEnabled(PassPhase::Dependences))
      return make_error<StringError>(
          formatv("'{0}' requires 'deps' to be enabled", getPhaseName(P)).str(),
          inconvertibleErrorCode());
  }

  if (isPhaseEnabled(PassPhase::CodeGen) && !isPhaseEnabled(PassPhase::AstGen))
    return make_error<StringError>("'codegen' requires 'ast' to be enabled",
                                   inconvertibleErrorCode());

  return Error::success();
}

bool polly::runPollyPass(Function &F, FunctionAnalysisManager &FAM,
                         PollyPassOptions Opts) {
  return PhaseManager(F, FAM, std::move(Opts)).run();
}
