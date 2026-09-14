//
// The baked evaluation mode must be the dynamic path's answer, exactly.
//
// The program is a second implementation of the evaluation semantics, so the
// only test that means anything is equality with the path it replaces -- on
// the whole published generation, at every frame, on every shipped shape of
// the biped stage (flat, layered and animated) plus a matrix-mover rig, so
// both geometry operations the program expresses are compared. Exact
// equality, not a tolerance: "close" here is a second rig.
//
// argv[1] = path to the examples directory (containing biped/Biped.usda).
// The codeless schema plugin is expected at
// <examples>/../plugin/rigExecSchema/resources.
//
#include "rigExecPoseCompare.h"

#include "rigExec/bakedProgram.h"
#include "rigExec/moverGraph.h"
#include "rigExec/rigEvaluator.h"

#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/base/tf/fileUtils.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/editTarget.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/xform.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <iterator>
#include <string>
#include <vector>

using namespace rigExec;

PXR_NAMESPACE_USING_DIRECTIVE

static int failures = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            ++failures;                                                    \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
        }                                                                  \
    } while (0)

static SdfPath
FindRig(const UsdStageRefPtr &stage)
{
    for (const UsdPrim &prim : stage->Traverse()) {
        if (prim.GetTypeName() == "RigExecRoot") {
            return prim.GetPath();
        }
    }
    return SdfPath();
}

// The two environment variables the parity harness runs this suite under,
// read the way the evaluator reads them -- once, into a function-local
// static -- because the evaluator fixes both at the construction of its
// first one and a test that re-read them could disagree with the path it is
// measuring. They are here so that the suite is exact under
// RIGEXEC_EVALUATION_MODE=parity RIGEXEC_BAKE_REQUIRED=1 rather than merely
// runnable: an assertion about the DEFAULT mode is an assertion about what
// this variable says, and the fallback line bake-required adds is asserted
// present in that environment and absent outside it.
static RigExecEvaluationMode
DefaultEvaluationMode()
{
    static const RigExecEvaluationMode mode = [] {
        const std::string requested = TfGetenv("RIGEXEC_EVALUATION_MODE", "");
        if (requested == "baked") {
            return RigExecEvaluationMode::Baked;
        }
        if (requested == "parity") {
            return RigExecEvaluationMode::BakedWithParityCheck;
        }
        return RigExecEvaluationMode::Dynamic;
    }();
    return mode;
}

static bool
BakeRequired()
{
    static const bool required =
        TfGetenvBool("RIGEXEC_BAKE_REQUIRED", false);
    return required;
}

// The REFERENCE half of a comparison says its mode out loud.
//
// It used to be enough to leave such an evaluator alone: one nobody set was
// Dynamic, or whatever RIGEXEC_EVALUATION_MODE asked this whole suite for.
// examples/biped now authors `uniform bool rigExec:baked = true` on its rig
// root, and an evaluator nobody sets compiles that stage into the PROGRAM --
// which would leave every comparison below holding the program against
// itself, passing, and proving nothing. Stating the harness's own mode is
// exactly the behaviour these evaluators had before the attribute existed,
// in both environments, and makes the stage unable to change it: the source
// becomes Explicit, which outranks the attribute.
//
// Only for the evaluators a test uses AS a reference. An evaluator the test
// is measuring says what it is measuring for itself.
static void
MakeItTheReference(RigExecRigEvaluator *rig)
{
    rig->SetEvaluationMode(DefaultEvaluationMode());
    CHECK(rig->GetEvaluationModeSource() ==
          RigExecEvaluationModeSource::Explicit);
}

// The prefix RigExecRigEvaluator::_ReportBakeRequired pushes onto a
// generation that ran dynamically while its mode asked for the program.
static const char *const kBakeRequiredPrefix =
    "baked parity mismatch: bake required, evaluated dynamically: ";

// What a consumer would see with that line filtered out, and how many were
// filtered. Splitting them is what lets a fallback test assert BOTH halves:
// that nothing else on the generation moved, and that the announcement is
// there exactly when the environment asked for it.
static std::vector<std::string>
WithoutTheBakeRequiredLine(const std::vector<std::string> &diagnostics,
                           size_t *announced)
{
    std::vector<std::string> rest;
    *announced = 0;
    for (const std::string &line : diagnostics) {
        if (line.rfind(kBakeRequiredPrefix, 0) == 0) {
            ++*announced;
        } else {
            rest.push_back(line);
        }
    }
    return rest;
}

// One line per fallen generation under RIGEXEC_BAKE_REQUIRED, and none at
// all in a mode that never asked for the program.
static size_t
ExpectedBakeRequiredLines(RigExecEvaluationMode mode)
{
    return BakeRequired() && mode != RigExecEvaluationMode::Dynamic ? 1 : 0;
}

// "The same published generation" is defined once, in the shared header, so
// that this suite and the comparator the parity mode is made of cannot
// disagree about what one is. Everything below reports into `failures`.
using rigExecTest::SameFrame;

template <class Map, class Equal>
static void
CompareMaps(const char *what, const std::string &where, const Map &reference,
            const Map &baked, Equal equal)
{
    rigExecTest::CompareMaps(&failures, what, where, reference, baked, equal);
}

static void
TestModeIsExact(const std::string &examplesDir, const char *stageName)
{
    const std::string stagePath =
        examplesDir + "/" + std::string(stageName);
    UsdStageRefPtr dynamicStage = UsdStage::Open(stagePath);
    UsdStageRefPtr bakedStage = UsdStage::Open(stagePath);
    CHECK(dynamicStage);
    CHECK(bakedStage);
    if (!dynamicStage || !bakedStage) return;
    const SdfPath rigPath = FindRig(dynamicStage);
    CHECK(!rigPath.IsEmpty());
    if (rigPath.IsEmpty()) return;

    // Two evaluators on two stages, so neither can leak compiled state or a
    // warm exec cache into the other.
    RigExecRigEvaluator dynamicRig(dynamicStage, rigPath);
    RigExecRigEvaluator bakedRig(bakedStage, rigPath);
    MakeItTheReference(&dynamicRig);
    bakedRig.SetEvaluationMode(RigExecEvaluationMode::Baked);
    CHECK(bakedRig.GetEvaluationMode() == RigExecEvaluationMode::Baked);

    std::vector<std::string> errors;
    CHECK(dynamicRig.Compile(&errors));
    errors.clear();
    CHECK(bakedRig.Compile(&errors));

    // The biped uses none of the features the program cannot express, so a
    // fallback here would make the rest of this test compare the dynamic
    // path with itself and pass while proving nothing.
    std::vector<std::string> reasons;
    if (!bakedRig.IsBakeable(&reasons)) {
        ++failures;
        std::printf("FAIL %s: the biped is not bakeable\n", stageName);
        for (const std::string &reason : reasons) {
            std::printf("    %s\n", reason.c_str());
        }
        return;
    }
    CHECK(dynamicRig.GetBindingEpochDigest() ==
          bakedRig.GetBindingEpochDigest());

    for (double frame = 1; frame <= 8; ++frame) {
        const std::string where =
            std::string(stageName) + " frame " + std::to_string(int(frame));
        const RigExecRigPose reference = dynamicRig.Evaluate(UsdTimeCode(frame));
        const RigExecRigPose baked = bakedRig.Evaluate(UsdTimeCode(frame));
        CHECK(reference.valid);
        CHECK(baked.valid);
        if (!reference.valid || !baked.valid) continue;
        CompareMaps("base joint frame", where, reference.jointFramesBase,
                    baked.jointFramesBase, SameFrame);
        CompareMaps("final joint frame", where, reference.jointFramesFinal,
                    baked.jointFramesFinal, SameFrame);
        CompareMaps("joint matrix", where, reference.jointMatricesFinal,
                    baked.jointMatricesFinal,
                    [](const GfMatrix4d &a, const GfMatrix4d &b) {
                        return a == b;
                    });
        CompareMaps("control frame", where, reference.controlFrames,
                    baked.controlFrames, SameFrame);
        CompareMaps("provider transform", where, reference.providerXforms,
                    baked.providerXforms,
                    [](const GfMatrix4d &a, const GfMatrix4d &b) {
                        return a == b;
                    });
        CompareMaps("moved property", where, reference.movedProperties,
                    baked.movedProperties,
                    [](const VtValue &a, const VtValue &b) { return a == b; });
        CompareMaps("solver frames", where, reference.solverFrames,
                    baked.solverFrames,
                    [](const std::vector<RigExecPointFrame> &a,
                       const std::vector<RigExecPointFrame> &b) {
                        if (a.size() != b.size()) return false;
                        for (size_t i = 0; i < a.size(); ++i) {
                            if (!SameFrame(a[i], b[i])) return false;
                        }
                        return true;
                    });
        // The diagnostics are part of the generation, not commentary: a
        // consumer reads them to learn a constraint passed through.
        if (reference.diagnostics != baked.diagnostics) {
            ++failures;
            std::printf("FAIL %s: diagnostics differ (%zu vs %zu)\n",
                        where.c_str(), reference.diagnostics.size(),
                        baked.diagnostics.size());
        }
        CHECK(reference.solverOverrideRounds == baked.solverOverrideRounds);
        CHECK(baked.bakedParityMismatches == 0);
    }
}

// A consumer that disabled the observational guides gets none from either
// path: the dynamic walk skips the guide request, and the program must skip
// its publication too, or the parity check reports one mismatch per solver.
static void
TestParityModeWithGuidesDisabled(const std::string &examplesDir,
                                 const char *stageName)
{
    UsdStageRefPtr stage =
        UsdStage::Open(examplesDir + "/" + std::string(stageName));
    CHECK(stage);
    if (!stage) return;
    const SdfPath rigPath = FindRig(stage);
    if (rigPath.IsEmpty()) { ++failures; return; }
    RigExecRigEvaluator rig(stage, rigPath);
    rig.SetSolverGuidesEnabled(false);
    rig.SetEvaluationMode(RigExecEvaluationMode::BakedWithParityCheck);
    std::vector<std::string> errors;
    CHECK(rig.Compile(&errors));
    CHECK(rig.IsBakeable(nullptr));
    for (double frame = 1; frame <= 3; ++frame) {
        const RigExecRigPose pose = rig.Evaluate(UsdTimeCode(frame));
        CHECK(pose.valid);
        CHECK(pose.solverFrames.empty());
        if (pose.bakedParityMismatches != 0) {
            ++failures;
            std::printf("FAIL %s frame %d with guides disabled: %zu baked "
                        "parity mismatch(es)\n",
                        stageName, int(frame), pose.bakedParityMismatches);
        }
    }
    // And back on: the program publishes them again, still in agreement.
    rig.SetSolverGuidesEnabled(true);
    const RigExecRigPose withGuides = rig.Evaluate(UsdTimeCode(2));
    CHECK(withGuides.valid);
    CHECK(!withGuides.solverFrames.empty());
    CHECK(withGuides.bakedParityMismatches == 0);
}

// The same comparison the mode performs on itself, which is what a caller
// gets when they ask for it rather than writing the loop above.
static void
TestParityModeReportsNoMismatch(const std::string &examplesDir,
                                const char *stageName)
{
    UsdStageRefPtr stage =
        UsdStage::Open(examplesDir + "/" + std::string(stageName));
    CHECK(stage);
    if (!stage) return;
    const SdfPath rigPath = FindRig(stage);
    if (rigPath.IsEmpty()) { ++failures; return; }
    RigExecRigEvaluator rig(stage, rigPath);
    rig.SetEvaluationMode(RigExecEvaluationMode::BakedWithParityCheck);
    std::vector<std::string> errors;
    CHECK(rig.Compile(&errors));
    CHECK(rig.IsBakeable(nullptr));
    for (double frame = 1; frame <= 8; ++frame) {
        const RigExecRigPose pose = rig.Evaluate(UsdTimeCode(frame));
        CHECK(pose.valid);
        if (pose.bakedParityMismatches != 0) {
            ++failures;
            std::printf("FAIL %s frame %d: %zu baked parity mismatch(es)\n",
                        stageName, int(frame), pose.bakedParityMismatches);
            for (const std::string &diagnostic : pose.diagnostics) {
                if (diagnostic.rfind("baked parity", 0) == 0) {
                    std::printf("    %s\n", diagnostic.c_str());
                }
            }
        }
    }
}

// The mode is a request. An evaluator that never asks for it must be
// bit-identical to one that does not know it exists, and asking for it on a
// rig that cannot bake must still evaluate.
static void
TestDynamicModeIsTheDefault(const std::string &examplesDir)
{
    UsdStageRefPtr stage = UsdStage::Open(examplesDir + "/biped/Biped.usda");
    CHECK(stage);
    if (!stage) return;
    const SdfPath rigPath = FindRig(stage);
    if (rigPath.IsEmpty()) { ++failures; return; }
    RigExecRigEvaluator rig(stage, rigPath);
    // The default is the PROCESS default: Dynamic, unless the harness asked
    // for another with RIGEXEC_EVALUATION_MODE, which is how this suite is
    // re-run against the program. Asserting Dynamic unconditionally would
    // fail the whole suite in exactly that environment while saying nothing
    // about the evaluator.
    CHECK(rig.GetEvaluationMode() == DefaultEvaluationMode());
    std::vector<std::string> errors;
    CHECK(rig.Compile(&errors));
    // Compile is where the STAGE gets its say, and this one has something to
    // say: the biped authors rigExec:baked. The assertion above is about
    // what an evaluator starts in, which is a fact about the process; this
    // is about what the rig asked for, and only the second survives a
    // compile. Under the parity harness the variable outranks the attribute
    // and nothing moves, which is the half that keeps the suite exact in
    // both environments.
    if (DefaultEvaluationMode() == RigExecEvaluationMode::Dynamic) {
        CHECK(rig.GetEvaluationMode() == RigExecEvaluationMode::Baked);
        CHECK(rig.GetEvaluationModeSource() ==
              RigExecEvaluationModeSource::Attribute);
    } else {
        CHECK(rig.GetEvaluationMode() == DefaultEvaluationMode());
        CHECK(rig.GetEvaluationModeSource() ==
              RigExecEvaluationModeSource::Environment);
    }
    // Switching after a compile builds the program without a recompile, and
    // switching back drops it.
    rig.SetEvaluationMode(RigExecEvaluationMode::Baked);
    const RigExecRigPose baked = rig.Evaluate(UsdTimeCode(1.0));
    CHECK(baked.valid);
    rig.SetEvaluationMode(RigExecEvaluationMode::Dynamic);
    CHECK(rig.GetEvaluationMode() == RigExecEvaluationMode::Dynamic);
    const RigExecRigPose dynamic = rig.Evaluate(UsdTimeCode(1.0));
    CHECK(dynamic.valid);
    CompareMaps("final joint frame", "mode toggle", dynamic.jointFramesFinal,
                baked.jointFramesFinal, SameFrame);
}

// The program captures values, and nothing in the epoch digest would say that
// a value behind one of them moved -- the digest is unchanged by exactly the
// edits that make a captured constant wrong. So the program carries its own
// index of what the bake read, and the tests below are what makes that index
// a fact rather than a comment: each moves something and demands the
// published generation follow, with the mode left on Baked throughout.

// One joint's final frame, so a test can say "the answer moved".
static RigExecPointFrame
JointFrame(const RigExecRigPose &pose, const SdfPath &joint)
{
    const auto found = pose.jointFramesFinal.find(joint);
    return found == pose.jointFramesFinal.end() ? RigExecPointFrame()
                                                : found->second;
}

static SdfPath
FirstControlWithAvar(const UsdStageRefPtr &stage, const SdfPath &rigPath,
                     const TfToken &avar)
{
    for (const UsdPrim &prim : UsdPrimRange(stage->GetPrimAtPath(rigPath))) {
        if (prim.GetTypeName() == "RigExecControl" &&
            prim.GetAttribute(avar)) {
            return prim.GetPath();
        }
    }
    return SdfPath();
}

static void
TestAnEditAfterTheBakeIsFollowed(const std::string &examplesDir)
{
    UsdStageRefPtr stage = UsdStage::Open(examplesDir + "/biped/Biped.usda");
    CHECK(stage);
    if (!stage) return;
    const SdfPath rigPath = FindRig(stage);
    if (rigPath.IsEmpty()) { ++failures; return; }
    const TfToken avar("avars:tx");
    const SdfPath control = FirstControlWithAvar(stage, rigPath, avar);
    CHECK(!control.IsEmpty());
    if (control.IsEmpty()) return;

    RigExecRigEvaluator rig(stage, rigPath);
    rig.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::vector<std::string> errors;
    CHECK(rig.Compile(&errors));
    CHECK(rig.IsBakeable(nullptr));
    const RigExecRigPose before = rig.Evaluate(UsdTimeCode(1.0));
    CHECK(before.valid);

    // Author into the session layer, which is a value edit: the epoch digest
    // hashes it, but a program that captured the old value would not notice
    // on its own.
    stage->SetEditTarget(UsdEditTarget(stage->GetSessionLayer()));
    UsdAttribute attribute =
        stage->GetPrimAtPath(control).GetAttribute(avar);
    double authored = 0;
    attribute.Get(&authored);
    CHECK(attribute.Set(authored + 3.0));

    const RigExecRigPose after = rig.Evaluate(UsdTimeCode(1.0));
    CHECK(after.valid);
    CHECK(rig.GetEvaluationMode() == RigExecEvaluationMode::Baked);

    // A second evaluator that only ever ran dynamically is the reference for
    // what the edit should have produced.
    UsdStageRefPtr referenceStage =
        UsdStage::Open(examplesDir + "/biped/Biped.usda");
    referenceStage->SetEditTarget(
        UsdEditTarget(referenceStage->GetSessionLayer()));
    CHECK(referenceStage->GetPrimAtPath(control)
              .GetAttribute(avar)
              .Set(authored + 3.0));
    RigExecRigEvaluator referenceRig(referenceStage, rigPath);
    MakeItTheReference(&referenceRig);
    CHECK(referenceRig.Compile(&errors));
    const RigExecRigPose reference = referenceRig.Evaluate(UsdTimeCode(1.0));

    CompareMaps("final joint frame after an edit", "Biped.usda",
                reference.jointFramesFinal, after.jointFramesFinal,
                SameFrame);
    // And the edit actually moved something, so the comparison above is not
    // passing because nothing happened.
    bool moved = false;
    for (const auto &[path, frame] : after.jointFramesFinal) {
        if (!SameFrame(frame, JointFrame(before, path))) { moved = true; break; }
    }
    CHECK(moved);
}

static void
TestAnInteractiveOverrideAfterTheBakeIsFollowed(const std::string &examplesDir)
{
    UsdStageRefPtr stage = UsdStage::Open(examplesDir + "/biped/Biped.usda");
    CHECK(stage);
    if (!stage) return;
    const SdfPath rigPath = FindRig(stage);
    if (rigPath.IsEmpty()) { ++failures; return; }
    const TfToken avar("avars:tx");
    const SdfPath control = FirstControlWithAvar(stage, rigPath, avar);
    CHECK(!control.IsEmpty());
    if (control.IsEmpty()) return;

    RigExecRigEvaluator rig(stage, rigPath);
    rig.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::vector<std::string> errors;
    CHECK(rig.Compile(&errors));
    const RigExecRigPose before = rig.Evaluate(UsdTimeCode(1.0));
    CHECK(before.valid);

    double authored = 0;
    stage->GetPrimAtPath(control).GetAttribute(avar).Get(&authored);
    rig.SetInteractiveOverrides(
        {RigExecValueOverride{control, TfToken(), avar,
                              VtValue(authored + 3.0)}});
    const RigExecRigPose dragged = rig.Evaluate(UsdTimeCode(1.0));
    CHECK(dragged.valid);
    CHECK(rig.GetEvaluationMode() == RigExecEvaluationMode::Baked);

    UsdStageRefPtr referenceStage =
        UsdStage::Open(examplesDir + "/biped/Biped.usda");
    RigExecRigEvaluator referenceRig(referenceStage, rigPath);
    MakeItTheReference(&referenceRig);
    CHECK(referenceRig.Compile(&errors));
    referenceRig.SetInteractiveOverrides(
        {RigExecValueOverride{control, TfToken(), avar,
                              VtValue(authored + 3.0)}});
    const RigExecRigPose reference = referenceRig.Evaluate(UsdTimeCode(1.0));

    CompareMaps("final joint frame under an override", "Biped.usda",
                reference.jointFramesFinal, dragged.jointFramesFinal,
                SameFrame);
    bool moved = false;
    for (const auto &[path, frame] : dragged.jointFramesFinal) {
        if (!SameFrame(frame, JointFrame(before, path))) { moved = true; break; }
    }
    CHECK(moved);

    // Releasing the drag must not leave the rig stuck on either path.
    rig.ClearInteractiveOverrides();
    const RigExecRigPose released = rig.Evaluate(UsdTimeCode(1.0));
    CompareMaps("final joint frame after release", "Biped.usda",
                before.jointFramesFinal, released.jointFramesFinal,
                SameFrame);
}


// ---------------------------------------------------------------------------
// Invalidation: the index, and the two answers it has to give.
// ---------------------------------------------------------------------------

using _Edit = std::function<void(const UsdStageRefPtr &, const SdfPath &)>;

// Every published MAP domain of one generation, so a scenario compares the
// whole answer rather than the part it happened to think of. The scalars are
// separate (CompareGenerationScalars) because the mover-graph counters
// describe how warm an evaluator is, and several callers here hold a fresh
// evaluator against a running one on purpose.
//
// Each of these three carries the coverage its NAME carries in the shared
// header -- maps, scalars, both -- so that a test written later gets the
// comparison it asked for rather than the one this file happened to bind to
// the shortest name.
static void
CompareEveryMap(const std::string &where, const RigExecRigPose &reference,
                const RigExecRigPose &baked)
{
    rigExecTest::CompareEveryMap(&failures, where, reference, baked);
}

// The work counters, the override rounds, the convergence flag and the
// diagnostics in order -- for a test that is ABOUT the counters and says so.
static void
CompareGenerationScalars(const std::string &where,
                         const RigExecRigPose &reference,
                         const RigExecRigPose &baked)
{
    rigExecTest::CompareGenerationScalars(&failures, where, reference, baked);
}

// The whole generation: every map and every compared scalar. For the callers
// whose two evaluators are at the same point in their lives, which is the
// only place the work counters mean anything.
static void
ComparePose(const std::string &where, const RigExecRigPose &reference,
            const RigExecRigPose &baked)
{
    rigExecTest::ComparePose(&failures, where, reference, baked);
}

// Bakes a rig, THEN edits the stage under it, and demands the eight
// generations that follow equal an evaluator that only ever ran dynamically
// under the same edit.
//
// \p expectRebuild is the other half of the contract and the reason this is
// not just another equality test: an edit that moved something the bake
// captured has to rebuild the program, and an edit that did not has to leave
// it standing. Both directions are asserted, because a program that rebuilds
// on every notice is also "never wrong" and is what this work replaced.
static void
TestEditAfterTheBake(const std::string &examplesDir, const char *stageName,
                     const char *what, const _Edit &edit, bool expectRebuild,
                     bool expectMoved = true)
{
    const std::string stagePath = examplesDir + "/" + std::string(stageName);
    const std::string where = std::string(what) + " on " + stageName;
    UsdStageRefPtr stage = UsdStage::Open(stagePath);
    CHECK(stage);
    if (!stage) return;
    const SdfPath rigPath = FindRig(stage);
    if (rigPath.IsEmpty()) { ++failures; return; }

    RigExecRigEvaluator rig(stage, rigPath);
    rig.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::vector<std::string> errors;
    CHECK(rig.Compile(&errors));
    CHECK(rig.IsBakeable(nullptr));
    const RigExecRigPose before = rig.Evaluate(UsdTimeCode(1.0));
    CHECK(before.valid);
    // The program answered that generation, so the comparison below is
    // between the two paths and not between the dynamic path and itself.
    CHECK(rig.GetBakedGenerationCount() == 1);
    const size_t builds = rig.GetBakedProgramBuildCount();
    CHECK(builds == 1);
    const size_t digestBefore = rig.GetBindingEpochDigest();

    edit(stage, rigPath);

    UsdStageRefPtr referenceStage = UsdStage::Open(stagePath);
    edit(referenceStage, rigPath);
    RigExecRigEvaluator referenceRig(referenceStage, rigPath);
    MakeItTheReference(&referenceRig);
    CHECK(referenceRig.Compile(&errors));

    bool moved = false;
    for (double frame = 1; frame <= 8; ++frame) {
        const RigExecRigPose reference =
            referenceRig.Evaluate(UsdTimeCode(frame));
        const RigExecRigPose baked = rig.Evaluate(UsdTimeCode(frame));
        CHECK(reference.valid);
        CHECK(baked.valid);
        if (!reference.valid || !baked.valid) continue;
        CompareEveryMap(where + " frame " + std::to_string(int(frame)),
                        reference, baked);
        for (const auto &[path, frames] : baked.jointFramesFinal) {
            const auto found = before.jointFramesFinal.find(path);
            if (found == before.jointFramesFinal.end() ||
                !SameFrame(frames, found->second)) {
                moved = true;
            }
        }
    }
    // An edit that changed nothing would let every comparison above pass
    // without the program having had to follow anything.
    if (moved != expectMoved) {
        ++failures;
        std::printf("FAIL %s: the edit %s the answer\n", where.c_str(),
                    expectMoved ? "did not move" : "unexpectedly moved");
    }
    // Every one of the eight was the program's: the edit settled into the
    // epoch before the first of them chose a path.
    CHECK(rig.GetBakedGenerationCount() == 9);
    if (expectRebuild) {
        if (rig.GetBakedProgramBuildCount() <= builds) {
            ++failures;
            std::printf("FAIL %s: the program was not rebuilt\n",
                        where.c_str());
        }
    } else if (rig.GetBakedProgramBuildCount() != builds) {
        ++failures;
        std::printf("FAIL %s: the program was rebuilt for an edit that "
                    "touched nothing it captured (digest %zu -> %zu)\n",
                    where.c_str(), digestBefore,
                    rig.GetBindingEpochDigest());
    }
    CHECK(rig.GetEvaluationMode() == RigExecEvaluationMode::Baked);
}

// The first prim of \p type under \p rigPath, so a test names a FEATURE and
// not a path that the asset is free to move.
static UsdPrim
FirstPrimOfType(const UsdStageRefPtr &stage, const SdfPath &rigPath,
                const char *type)
{
    for (const UsdPrim &prim : UsdPrimRange(stage->GetPrimAtPath(rigPath))) {
        if (prim.GetTypeName() == type) {
            return prim;
        }
    }
    return UsdPrim();
}

static UsdPrim
FirstConstraint(const UsdStageRefPtr &stage, const SdfPath &rigPath)
{
    for (const UsdPrim &prim : UsdPrimRange(stage->GetPrimAtPath(rigPath))) {
        const std::string type = prim.GetTypeName().GetString();
        if (type.size() > 10 &&
            type.compare(type.size() - 10, 10, "Constraint") == 0) {
            return prim;
        }
    }
    return UsdPrim();
}

static void
EditInSession(const UsdStageRefPtr &stage)
{
    stage->SetEditTarget(UsdEditTarget(stage->GetSessionLayer()));
}



// The same comparison with the edit in place BEFORE the bake, which is the
// other thing a keyed overlay has to work under: the bake classifies an input
// that is ALREADY animated, rather than reclassifying one that became so.
static void
TestEditBeforeTheBake(const std::string &examplesDir, const char *stageName,
                      const char *what, const _Edit &edit)
{
    const std::string stagePath = examplesDir + "/" + std::string(stageName);
    const std::string where = std::string(what) + " on " + stageName;
    UsdStageRefPtr bakedStage = UsdStage::Open(stagePath);
    UsdStageRefPtr dynamicStage = UsdStage::Open(stagePath);
    CHECK(bakedStage && dynamicStage);
    if (!bakedStage || !dynamicStage) return;
    const SdfPath rigPath = FindRig(bakedStage);
    if (rigPath.IsEmpty()) { ++failures; return; }
    edit(bakedStage, rigPath);
    edit(dynamicStage, rigPath);

    RigExecRigEvaluator bakedRig(bakedStage, rigPath);
    RigExecRigEvaluator dynamicRig(dynamicStage, rigPath);
    MakeItTheReference(&dynamicRig);
    bakedRig.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::vector<std::string> errors;
    CHECK(bakedRig.Compile(&errors));
    CHECK(dynamicRig.Compile(&errors));
    std::vector<std::string> reasons;
    if (!bakedRig.IsBakeable(&reasons)) {
        ++failures;
        std::printf("FAIL %s: not bakeable\n", where.c_str());
        for (const std::string &reason : reasons) {
            std::printf("    %s\n", reason.c_str());
        }
        return;
    }
    for (double frame = 1; frame <= 8; ++frame) {
        const RigExecRigPose reference = dynamicRig.Evaluate(UsdTimeCode(frame));
        const RigExecRigPose baked = bakedRig.Evaluate(UsdTimeCode(frame));
        CHECK(reference.valid);
        CHECK(baked.valid);
        if (!reference.valid || !baked.valid) continue;
        CompareEveryMap(where + " frame " + std::to_string(int(frame)),
                        reference, baked);
    }
    CHECK(bakedRig.GetBakedGenerationCount() == 8);
}

// The edits the scenarios above run. Each is written against a FEATURE of the
// biped rather than a path, so the asset can move without the test rotting.

// The IK/FK blend weight keyed, a constraint switched off, and a rest moved:
// three captured constants in one edit, of the three shapes the index has to
// recognise -- a value that became animated, a property that did not exist,
// and a value replaced in place.
static void
EditKeyedBlendDisabledConstraintAndRest(const UsdStageRefPtr &stage,
                                        const SdfPath &rigPath)
{
    EditInSession(stage);
    // The weight the blend reads through a connection, so the edit lands on
    // the far end of an input's walk rather than on the input itself.
    const UsdPrim blend =
        FirstPrimOfType(stage, rigPath, "RigExecBlendPointFrames");
    CHECK(blend);
    if (blend) {
        SdfPathVector connections;
        const UsdAttribute weight =
            blend.GetAttribute(TfToken("inputs:weight"));
        CHECK(weight);
        if (weight && weight.GetConnections(&connections) &&
            !connections.empty()) {
            const UsdAttribute source =
                stage->GetAttributeAtPath(connections[0]);
            CHECK(source);
            for (int frame = 1; frame <= 8; ++frame) {
                source.Set(float(frame) / 8.0f, UsdTimeCode(frame));
            }
        } else {
            ++failures;
        }
    }
    const UsdPrim constraint = FirstConstraint(stage, rigPath);
    CHECK(constraint);
    if (constraint) {
        constraint
            .CreateAttribute(TfToken("inputs:enabled"),
                             SdfValueTypeNames->Bool)
            .Set(false);
    }
    const UsdPrim joint = FirstPrimOfType(stage, rigPath, "RigExecJoint");
    CHECK(joint);
    if (joint) {
        const UsdAttribute rest = joint.GetAttribute(TfToken("rest:space"));
        CHECK(rest);
        GfMatrix4d space(1.0);
        if (rest && rest.Get(&space)) {
            space[3][1] += 1.5;
            rest.Set(space);
        }
    }
}

// The foot roll, which reaches the rig ONLY through property chains: it
// drives the envelope of the roll constraints through five float-math movers.
// The program runs those chains itself, off the authored stage, so nothing
// about the roll is captured.
//
// Two edits, because they are two different questions. Keying it makes the
// attribute animated, which is a STRUCTURAL change the epoch digest hashes --
// the rig recompiles and rebakes, and what is being tested is that eight
// keyed frames come out right. Moving its value is not: the chains re-read it
// every generation, so the answer has to follow with nothing rebuilt.
static void
EditKeyedFootRoll(const UsdStageRefPtr &stage, const SdfPath &rigPath)
{
    EditInSession(stage);
    size_t keyed = 0;
    for (const UsdPrim &prim : UsdPrimRange(stage->GetPrimAtPath(rigPath))) {
        const UsdAttribute roll = prim.GetAttribute(TfToken("foot:roll"));
        if (!roll) {
            continue;
        }
        for (int frame = 1; frame <= 8; ++frame) {
            roll.Set(float(frame) * 6.0f, UsdTimeCode(frame));
        }
        ++keyed;
    }
    CHECK(keyed > 0);
}

static void
EditFootRollValue(const UsdStageRefPtr &stage, const SdfPath &rigPath)
{
    EditInSession(stage);
    size_t edited = 0;
    for (const UsdPrim &prim : UsdPrimRange(stage->GetPrimAtPath(rigPath))) {
        const UsdAttribute roll = prim.GetAttribute(TfToken("foot:roll"));
        if (!roll) {
            continue;
        }
        float value = 0;
        roll.Get(&value);
        roll.Set(value + 24.0f);
        ++edited;
    }
    CHECK(edited > 0);
}

// A keyed avar's VALUE moved. The program re-reads it every frame through a
// retained query, so this must change the answer and rebuild nothing.
static void
EditAKeyedAvarValue(const UsdStageRefPtr &stage, const SdfPath &rigPath)
{
    // Into the layer that already holds the key, so this is a value moving
    // and not a property spec appearing -- the distinction the index draws.
    size_t edited = 0;
    for (const UsdPrim &prim : UsdPrimRange(stage->GetPrimAtPath(rigPath))) {
        const UsdAttribute avar = prim.GetAttribute(TfToken("avars:rz"));
        if (!avar || avar.GetNumTimeSamples() == 0) {
            continue;
        }
        for (int frame = 1; frame <= 8; ++frame) {
            double value = 0;
            avar.Get(&value, UsdTimeCode(frame));
            avar.Set(value * 2.0 + 1.0, UsdTimeCode(frame));
        }
        ++edited;
        break;
    }
    CHECK(edited == 1);
}

// A prim the rig has never heard of. The whole point of the index is that
// this costs nothing: the epoch is unchanged, no captured value moved, and
// the program keeps answering.
static void
TestAnUnrelatedEditLeavesTheProgramStanding(const std::string &examplesDir)
{
    UsdStageRefPtr stage = UsdStage::Open(examplesDir + "/biped/Biped.usda");
    CHECK(stage);
    if (!stage) return;
    const SdfPath rigPath = FindRig(stage);
    if (rigPath.IsEmpty()) { ++failures; return; }
    RigExecRigEvaluator rig(stage, rigPath);
    rig.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::vector<std::string> errors;
    CHECK(rig.Compile(&errors));
    const RigExecRigPose before = rig.Evaluate(UsdTimeCode(1.0));
    CHECK(before.valid);
    const size_t builds = rig.GetBakedProgramBuildCount();

    stage->SetEditTarget(UsdEditTarget(stage->GetSessionLayer()));
    const UsdPrim scratch =
        stage->DefinePrim(SdfPath("/Scratch"), TfToken("Xform"));
    CHECK(scratch);
    const UsdAttribute note = scratch.CreateAttribute(
        TfToken("custom:note"), SdfValueTypeNames->Double);
    CHECK(note.Set(1.0));
    CHECK(note.Set(2.0));

    const RigExecRigPose after = rig.Evaluate(UsdTimeCode(1.0));
    CHECK(after.valid);
    CHECK(rig.GetBakedGenerationCount() == 2);
    if (rig.GetBakedProgramBuildCount() != builds) {
        ++failures;
        std::printf("FAIL unrelated edit: the program was rebuilt (%zu -> "
                    "%zu)\n", builds, rig.GetBakedProgramBuildCount());
    }
    CompareEveryMap("an unrelated edit on Biped.usda", before, after);
}

// An interactive override on a constraint's envelope. The avar case above is
// the animator dragging a control; this is the one that lands on a mover's
// own input, and it is placed by the same rule -- read the long way, through
// the resolved inputs the override was written into.
static void
TestAnOverrideOnAConstraintWeightIsFollowed(const std::string &examplesDir)
{
    const std::string stagePath = examplesDir + "/biped/Biped.usda";
    UsdStageRefPtr stage = UsdStage::Open(stagePath);
    CHECK(stage);
    if (!stage) return;
    const SdfPath rigPath = FindRig(stage);
    if (rigPath.IsEmpty()) { ++failures; return; }
    const UsdPrim constraint = FirstConstraint(stage, rigPath);
    CHECK(constraint);
    if (!constraint) return;
    const std::vector<RigExecValueOverride> overrides{RigExecValueOverride{
        constraint.GetPath(), TfToken(), TfToken("inputs:defaultWeight"),
        VtValue(0.25f)}};

    RigExecRigEvaluator rig(stage, rigPath);
    rig.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::vector<std::string> errors;
    CHECK(rig.Compile(&errors));
    const RigExecRigPose before = rig.Evaluate(UsdTimeCode(1.0));
    CHECK(before.valid);
    rig.SetInteractiveOverrides(overrides);
    const RigExecRigPose dragged = rig.Evaluate(UsdTimeCode(1.0));
    CHECK(dragged.valid);
    // Placed, not fallen back to: an override the program declines is also a
    // correct answer, and would make this test prove nothing.
    CHECK(rig.GetBakedGenerationCount() == 2);

    UsdStageRefPtr referenceStage = UsdStage::Open(stagePath);
    RigExecRigEvaluator referenceRig(referenceStage, rigPath);
    MakeItTheReference(&referenceRig);
    CHECK(referenceRig.Compile(&errors));
    referenceRig.SetInteractiveOverrides(overrides);
    const RigExecRigPose reference = referenceRig.Evaluate(UsdTimeCode(1.0));
    CompareEveryMap("a constraint weight override on Biped.usda", reference,
                    dragged);

    // The constraint may drive a control rather than a joint, so both
    // domains count: what matters is that the override reached the walk.
    bool moved = false;
    for (const auto &[path, frame] : dragged.jointFramesFinal) {
        if (!SameFrame(frame, JointFrame(before, path))) { moved = true; break; }
    }
    for (const auto &[path, frame] : dragged.controlFrames) {
        const auto found = before.controlFrames.find(path);
        if (found == before.controlFrames.end() ||
            !SameFrame(frame, found->second)) {
            moved = true;
            break;
        }
    }
    CHECK(moved);
    rig.ClearInteractiveOverrides();
    const RigExecRigPose released = rig.Evaluate(UsdTimeCode(1.0));
    CompareEveryMap("a released constraint weight override on Biped.usda",
                    before, released);
}

// A rig whose refusal is not a gap in the bake but a property of the rig:
// one provider's posed:space is CONNECTED, so its value is whatever an
// arbitrary exec computation says from the middle of the pose walk, and
// there is no epoch-constant summary of that to compile. Every other refusal
// in IsBakeable is a feature waiting to be baked, and this suite's negative
// direction used to rest on two of them (11_VolumeWeights and
// 12_CurvenetProfile) -- so it would have evaporated the moment those
// landed, taking the only test that the fallback works at all with it.
//
// Built in memory rather than shipped as an example, because an example is
// something a rigger should copy and this is a rig that deliberately opts
// out of the fast path.
static UsdStageRefPtr
MakeAConnectedSpaceRig()
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const UsdPrim root = stage->DefinePrim(SdfPath("/Asset/Rig/Root"),
                                           TfToken("RigExecControl"));
    root.GetAttribute(TfToken("avars:tx")).Set(3.0);
    const UsdPrim joint = stage->DefinePrim(SdfPath("/Asset/Rig/Bone"),
                                            TfToken("RigExecJoint"));
    joint.GetAttribute(TfToken("avars:ty")).Set(2.0);

    // The refusal. The connection is to the control's own posed:space, which
    // is the identity the joint would have composed anyway -- so the rig
    // still evaluates to the pose the comparison below demands, and the only
    // thing the connection changes is that a value which was an epoch
    // constant is now an exec answer.
    joint.CreateAttribute(TfToken("posed:space"), SdfValueTypeNames->Matrix4d)
        .AddConnection(root.GetPath().AppendProperty(TfToken("posed:space")));

    stage->DefinePrim(SdfPath("/Asset/Geom"), TfToken("Scope"));
    const UsdPrim mesh = stage->DefinePrim(SdfPath("/Asset/Geom/Slab"),
                                           TfToken("Mesh"));
    VtVec3fArray points{GfVec3f(0, 0, 0), GfVec3f(1, 0, 0), GfVec3f(0, 1, 0)};
    mesh.GetAttribute(TfToken("points")).Set(points);
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const UsdPrim skin = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Skin"), TfToken("RigExecSkinMover"));
    skin.ApplyAPI(TfToken("RigExecMoverAPI"));
    skin.GetRelationship(TfToken("rigExec:moves"))
        .SetTargets({mesh.GetPath().AppendProperty(TfToken("points"))});
    skin.CreateRelationship(TfToken("rigExec:influences"))
        .SetTargets({joint.GetPath()});
    skin.CreateAttribute(TfToken("rigExec:elementSize"),
                         SdfValueTypeNames->Int).Set(1);
    skin.CreateAttribute(TfToken("rigExec:jointIndices"),
                         SdfValueTypeNames->IntArray).Set(VtIntArray{0, 0, 0});
    skin.CreateAttribute(TfToken("rigExec:jointWeights"),
                         SdfValueTypeNames->FloatArray)
        .Set(VtFloatArray{1.0f, 1.0f, 1.0f});
    return stage;
}

// A rig the program cannot express. Asking for the mode must name the
// features that stopped it and then evaluate anyway -- a silent fallback
// reads as the mode not working, and a failure reads as the mode being
// dangerous. Neither is what a request means.
static void
TestANonBakeableRigFallsBack(const char *what, const char *expectReason)
{
    UsdStageRefPtr stage = MakeAConnectedSpaceRig();
    CHECK(stage);
    if (!stage) return;
    const SdfPath rigPath = FindRig(stage);
    if (rigPath.IsEmpty()) { ++failures; return; }
    RigExecRigEvaluator rig(stage, rigPath);
    rig.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::vector<std::string> errors;
    if (!rig.Compile(&errors)) {
        // The fixture is built here rather than shipped, so a mistake in it
        // reads as a bake failure unless the compile errors come out.
        ++failures;
        std::printf("FAIL %s: the fixture does not compile\n", what);
        for (const std::string &error : errors) {
            std::printf("    %s\n", error.c_str());
        }
        return;
    }

    std::vector<std::string> reasons;
    CHECK(!rig.IsBakeable(&reasons));
    CHECK(!reasons.empty());
    bool named = false;
    for (const std::string &reason : reasons) {
        if (reason.find(expectReason) != std::string::npos) named = true;
    }
    if (!named) {
        ++failures;
        std::printf("FAIL %s: no reason mentions \"%s\"\n", what,
                    expectReason);
        for (const std::string &reason : reasons) {
            std::printf("    %s\n", reason.c_str());
        }
    }
    CHECK(rig.GetBakedProgramBuildCount() == 0);
    // Asked once, for the epoch. Bakeability is a property of the compiled
    // epoch, so re-asking it per frame would charge every frame of a rig
    // that will never bake for the same refusal -- and the build count above
    // cannot see that, because it does not move for such a rig at all.
    const size_t attempts = rig.GetBakedProgramBuildAttemptCount();
    CHECK(attempts == 1);

    UsdStageRefPtr referenceStage = MakeAConnectedSpaceRig();
    RigExecRigEvaluator referenceRig(referenceStage, rigPath);
    CHECK(referenceRig.Compile(&errors));
    for (double frame = 1; frame <= 4; ++frame) {
        const RigExecRigPose reference =
            referenceRig.Evaluate(UsdTimeCode(frame));
        const RigExecRigPose fallen = rig.Evaluate(UsdTimeCode(frame));
        CHECK(fallen.valid);
        // The generation is a real one: a comparison of two empty poses
        // agrees perfectly and says nothing about the fallback at all.
        CHECK(!fallen.jointFramesFinal.empty());
        CHECK(!fallen.controlFrames.empty());
        CHECK(!fallen.movedProperties.empty());
        CompareEveryMap(std::string(what) + " frame " +
                            std::to_string(int(frame)),
                        reference, fallen);
        // Silent: the fallback is not a diagnostic on the generation, which
        // a consumer would have to filter out of a rig's real problems.
        // Under RIGEXEC_BAKE_REQUIRED it is the opposite -- saying so is the
        // whole point of that variable -- so both halves are asserted: one
        // announcement per fallen generation there and none elsewhere, and
        // nothing else on either generation moved.
        size_t announcedFallen = 0, announcedReference = 0;
        const std::vector<std::string> quietFallen =
            WithoutTheBakeRequiredLine(fallen.diagnostics, &announcedFallen);
        const std::vector<std::string> quietReference =
            WithoutTheBakeRequiredLine(reference.diagnostics,
                                       &announcedReference);
        CHECK(quietReference == quietFallen);
        CHECK(announcedFallen == ExpectedBakeRequiredLines(
                                     RigExecEvaluationMode::Baked));
        CHECK(announcedReference ==
              ExpectedBakeRequiredLines(DefaultEvaluationMode()));
    }
    // Nothing ran baked, and nothing pretended to. The REQUEST stands,
    // though: a rig that declines has not had its mode taken away, and a
    // consumer reading the mode back must see what it asked for rather than
    // infer the refusal from a silently changed setting.
    CHECK(rig.GetEvaluationMode() == RigExecEvaluationMode::Baked);
    CHECK(rig.GetBakedGenerationCount() == 0);
    // Four frames later the question has still been asked once.
    CHECK(rig.GetBakedProgramBuildAttemptCount() == attempts);
}


// A curvenet weight driving a matrix mover: the one baked object that reads
// an ARRAY off its own prim every frame.
static UsdStageRefPtr
MakeACurvenetWeightRig()
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const UsdPrim mesh =
        stage->DefinePrim(SdfPath("/Asset/Mesh"), TfToken("Mesh"));
    mesh.GetAttribute(TfToken("points"))
        .Set(VtVec3fArray{GfVec3f(0, 0, 0), GfVec3f(1, 0, 0),
                          GfVec3f(2, 0, 0), GfVec3f(0, 1, 0),
                          GfVec3f(1, 1, 0), GfVec3f(2, 1, 0)});
    mesh.GetAttribute(TfToken("faceVertexCounts")).Set(VtIntArray{4, 4});
    mesh.GetAttribute(TfToken("faceVertexIndices"))
        .Set(VtIntArray{0, 1, 4, 3, 1, 2, 5, 4});
    const UsdPrim net =
        stage->DefinePrim(SdfPath("/Asset/Net"), TfToken("RigExecCurvenet"));
    net.GetAttribute(TfToken("points"))
        .Set(VtVec3fArray{GfVec3f(0, 0.5f, 0), GfVec3f(0.67f, 0.5f, 0),
                          GfVec3f(1.33f, 0.5f, 0), GfVec3f(2, 0.5f, 0)});
    net.GetAttribute(TfToken("rigExec:splineIndices"))
        .Set(VtIntArray{0, 1, 2, 3});

    const SdfPath target = mesh.GetPath().AppendProperty(TfToken("points"));
    const UsdPrim weight = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/Net"), TfToken("RigExecCurvenetWeight"));
    weight.GetRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({target});
    weight.GetRelationship(TfToken("rigExec:curvenetPoints"))
        .SetTargets({net.GetPath().AppendProperty(TfToken("points"))});
    weight.GetRelationship(TfToken("rigExec:curvenetSplineIndices"))
        .SetTargets({net.GetPath().AppendProperty(
            TfToken("rigExec:splineIndices"))});
    weight.GetRelationship(TfToken("rigExec:meshFaceCounts"))
        .SetTargets({mesh.GetPath().AppendProperty(
            TfToken("faceVertexCounts"))});
    weight.GetRelationship(TfToken("rigExec:meshFaceIndices"))
        .SetTargets({mesh.GetPath().AppendProperty(
            TfToken("faceVertexIndices"))});
    weight.GetAttribute(TfToken("inputs:weights"))
        .Set(VtFloatArray{1.0f, 0.75f, 0.25f, 0.0f});

    const UsdPrim driver = stage->DefinePrim(
        SdfPath("/Asset/Rig/Controls/Push"), TfToken("RigExecControl"));
    driver.GetAttribute(TfToken("avars:tz")).Set(4.0);
    const UsdPrim mover = stage->DefinePrim(SdfPath("/Asset/Rig/Movers/M"),
                                            TfToken("RigExecMatrixMover"));
    mover.ApplyAPI(TfToken("RigExecMoverAPI"));
    mover.GetRelationship(TfToken("rigExec:moves")).SetTargets({target});
    mover.GetRelationship(TfToken("rigExec:transform"))
        .SetTargets({driver.GetPath()});
    mover.GetRelationship(TfToken("rigExec:weightObject"))
        .SetTargets({weight.GetPath()});
    return stage;
}

// The fourth shape of unplaceable, and the one that is unplaceable because
// the DYNAMIC path cannot hold it either.
//
// A curvenet weight's inputs:weights and rigExec:autoSmooth are arrays, read
// through the generation's resolved inputs every frame -- which an override
// is written into, so the program would honour one. Exec cannot: its
// computeWeightPacket declares both as AttributeValue<float>/<int>, and an
// override carrying the array they actually hold is rejected there by type
// ("expected 'float', got 'VtArray<float>'"), leaving the dynamic path
// answering from the authored value. Honouring it here would be the program
// answering a question the dynamic path refuses -- pose.valid on both sides
// and a different mesh. Measured before the two properties were declared
// unplaceable: the program published z = 2.368 where the dynamic path
// published z = 4.
//
// It lives in THIS suite and not beside the curvenet's own tests because a
// deliberate fallback is a "bake required" line, and that suite runs under
// RIGEXEC_BAKE_REQUIRED=1 where such a line is a failure -- correctly.
static void
TestAnArrayOverrideOnACurvenetWeightFallsBack()
{
    UsdStageRefPtr stage = MakeACurvenetWeightRig();
    const SdfPath rigPath("/Asset/Rig");
    const SdfPath target("/Asset/Mesh.points");
    RigExecRigEvaluator rig(stage, rigPath);
    rig.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::vector<std::string> errors;
    CHECK(rig.Compile(&errors));
    CHECK(rig.Evaluate(UsdTimeCode(1.0)).valid);
    // The program is standing and answering, so a generation that does not
    // run baked below fell back rather than never having been baked at all.
    CHECK(rig.GetBakedGenerationCount() == 1);

    const std::vector<std::pair<const char *, RigExecValueOverride>> cases{
        {"an overridden inputs:weights",
         RigExecValueOverride{
             SdfPath("/Asset/Rig/Weights/Net"), TfToken(),
             TfToken("inputs:weights"),
             VtValue(VtFloatArray{0.5f, 0.375f, 0.125f, 0.0f})}},
        {"an overridden rigExec:autoSmooth",
         RigExecValueOverride{SdfPath("/Asset/Rig/Weights/Net"), TfToken(),
                              TfToken("rigExec:autoSmooth"),
                              VtValue(VtIntArray{1, 1, 1, 1})}}};
    for (const auto &[what, override] : cases) {
        rig.SetInteractiveOverrides({override});
        const size_t bakedGenerations = rig.GetBakedGenerationCount();
        const RigExecRigPose held = rig.Evaluate(UsdTimeCode(1.0));
        CHECK(held.valid);
        if (rig.GetBakedGenerationCount() != bakedGenerations) {
            ++failures;
            std::printf("FAIL %s: the program answered a generation holding "
                        "an override exec rejects by type\n", what);
        }
        // And what it fell back to is the dynamic path's own answer, which
        // is the authored field: the override reaches neither side.
        UsdStageRefPtr referenceStage = MakeACurvenetWeightRig();
        RigExecRigEvaluator reference(referenceStage, rigPath);
        reference.SetEvaluationMode(RigExecEvaluationMode::Dynamic);
        CHECK(reference.Compile(&errors));
        reference.SetInteractiveOverrides({override});
        const RigExecRigPose expected = reference.Evaluate(UsdTimeCode(1.0));
        CHECK(expected.valid);
        CompareEveryMap(what, expected, held);
        CHECK(expected.movedProperties.count(target) == 1);
        CHECK(held.movedProperties.count(target) == 1);
    }
    rig.SetInteractiveOverrides({});
}

// The other half of override placement, and the half that has to be wrong
// SAFELY: an override the program cannot place must send the generation down
// the dynamic path, not be quietly ignored.
//
// Three shapes of unplaceable, one per authored reason:
//   * a value folded into bake state -- a SPACE EXPRESSION, which the bake
//     accepted because it is unauthored and whose authoring would replace
//     the compose rather than move a value in it;
//   * a property the program never reads, so there is no slot to put it in
//     and no promise that some other reader would pick it up;
//   * a computation override, which names something only exec can answer.
//
// A rest used to be the first of these and is not one any more: the ladder
// is a per-frame input, so a rest drag PLACES. The second half of this test
// is that half, because "it falls back" and "it is answered correctly" are
// the two ways an override can be handled and only one of them is progress.
static void
TestAnUnplaceableOverrideFallsBack(const std::string &examplesDir)
{
    const std::string stagePath = examplesDir + "/biped/Biped.usda";
    UsdStageRefPtr stage = UsdStage::Open(stagePath);
    CHECK(stage);
    if (!stage) return;
    const SdfPath rigPath = FindRig(stage);
    if (rigPath.IsEmpty()) { ++failures; return; }
    const UsdPrim joint = FirstPrimOfType(stage, rigPath, "RigExecJoint");
    CHECK(joint);
    if (!joint) return;
    GfMatrix4d rest(1.0);
    joint.GetAttribute(TfToken("rest:space")).Get(&rest);
    GfMatrix4d moved = rest;
    moved[3][1] += 4.0;

    RigExecRigEvaluator rig(stage, rigPath);
    rig.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::vector<std::string> errors;
    CHECK(rig.Compile(&errors));
    const RigExecRigPose before = rig.Evaluate(UsdTimeCode(1.0));
    CHECK(before.valid);
    CHECK(rig.GetBakedGenerationCount() == 1);

    // A prim computation the pose-seed request actually carries, so the
    // dynamic path it falls back to has something to hold the override
    // against; the program has no exec and cannot hold it at all.
    RigExecPointFrame frame = JointFrame(before, joint.GetPath());
    CHECK(frame.IsValid());
    for (GfVec3d &point : frame.points) {
        point[1] += 5.0;
    }
    const std::vector<std::pair<const char *, RigExecValueOverride>> cases{
        {"a folded space expression",
         RigExecValueOverride{joint.GetPath(), TfToken(),
                              TfToken("parent:space"), VtValue(moved)}},
        {"a property the bake never read",
         RigExecValueOverride{joint.GetPath(), TfToken(),
                              TfToken("custom:notAnInput"), VtValue(1.0)}},
        {"a computation",
         RigExecValueOverride{joint.GetPath(),
                              TfToken("computePointFrame"), TfToken(),
                              VtValue(frame)}},
    };
    for (const auto &[what, override] : cases) {
        rig.SetInteractiveOverrides({override});
        const size_t bakedGenerations = rig.GetBakedGenerationCount();
        const RigExecRigPose held = rig.Evaluate(UsdTimeCode(1.0));
        CHECK(held.valid);
        if (rig.GetBakedGenerationCount() != bakedGenerations) {
            ++failures;
            std::printf("FAIL %s: the program answered a generation holding "
                        "an override it cannot place\n", what);
        }
        // And the dynamic answer it fell back to is the right one.
        UsdStageRefPtr referenceStage = UsdStage::Open(stagePath);
        RigExecRigEvaluator referenceRig(referenceStage, rigPath);
        MakeItTheReference(&referenceRig);
        CHECK(referenceRig.Compile(&errors));
        referenceRig.SetInteractiveOverrides({override});
        CompareEveryMap(std::string(what) + " on Biped.usda",
                        referenceRig.Evaluate(UsdTimeCode(1.0)), held);
    }
    // And the rest drag, which the ladder work moved from the list above to
    // here: the program ANSWERS it, and answers it the way exec does. Both
    // halves matter -- a program that placed the override and then composed
    // against the authored rest would still count a baked generation.
    {
        const RigExecValueOverride restDrag{joint.GetPath(), TfToken(),
                                            TfToken("rest:space"),
                                            VtValue(moved)};
        rig.SetInteractiveOverrides({restDrag});
        const size_t bakedGenerations = rig.GetBakedGenerationCount();
        const RigExecRigPose dragged = rig.Evaluate(UsdTimeCode(1.0));
        CHECK(dragged.valid);
        CHECK(rig.GetBakedGenerationCount() == bakedGenerations + 1);
        UsdStageRefPtr referenceStage = UsdStage::Open(stagePath);
        RigExecRigEvaluator referenceRig(referenceStage, rigPath);
        MakeItTheReference(&referenceRig);
        CHECK(referenceRig.Compile(&errors));
        referenceRig.SetInteractiveOverrides({restDrag});
        CompareEveryMap("a placed rest drag on Biped.usda",
                        referenceRig.Evaluate(UsdTimeCode(1.0)), dragged);
    }
    // Releasing every one of them puts the rig back on the program.
    rig.ClearInteractiveOverrides();
    const size_t bakedGenerations = rig.GetBakedGenerationCount();
    const RigExecRigPose released = rig.Evaluate(UsdTimeCode(1.0));
    CHECK(rig.GetBakedGenerationCount() == bakedGenerations + 1);
    CompareEveryMap("released, on Biped.usda", before, released);
}

// ---------------------------------------------------------------------------
// A value that is keyed ONCE.
//
// UsdStage reports ValueMightBeTimeVarying() == false for an attribute whose
// strongest opinion is exactly one time sample of a non-composable type, while
// a Default read -- which is what the bake captures at -- never sees a time
// sample at all. So a single-keyed input is the one shape that can be
// classified as an epoch constant and then captured as the value it does NOT
// have. An animator's first pose key is exactly that shape, which is why
// these two edits are the ones worth spending fixtures on.
// ---------------------------------------------------------------------------

static void
EditOneKeyOnAControlAvar(const UsdStageRefPtr &stage, const SdfPath &rigPath)
{
    EditInSession(stage);
    const SdfPath control =
        FirstControlWithAvar(stage, rigPath, TfToken("avars:tx"));
    if (control.IsEmpty()) { ++failures; return; }
    const UsdAttribute avar =
        stage->GetPrimAtPath(control).GetAttribute(TfToken("avars:tx"));
    // One sample and no default opinion underneath it in the session layer.
    avar.Set(25.0, UsdTimeCode(1.0));
}

static void
EditOneKeyOnEveryConstraintEnable(const UsdStageRefPtr &stage,
                                  const SdfPath &rigPath)
{
    EditInSession(stage);
    size_t keyed = 0;
    for (const UsdPrim &prim : UsdPrimRange(stage->GetPrimAtPath(rigPath))) {
        const std::string type = prim.GetTypeName().GetString();
        if (type.size() > 10 &&
            type.compare(type.size() - 10, 10, "Constraint") == 0) {
            if (const UsdAttribute enabled =
                    prim.GetAttribute(TfToken("inputs:enabled"))) {
                enabled.Set(false, UsdTimeCode(1.0));
                ++keyed;
            }
        }
    }
    CHECK(keyed > 0);
}

// A skin mover at partial constant strength.
//
// The full-strength constant envelope is the one every unweighted mover gets,
// and both geometry loops skip the blend for it. Nothing in examples/ or
// tests/ had a PARTIAL constant envelope on a bakeable skin mover, so the
// two loops' agreement about when the skip is safe was never compared on a
// case where they could differ.
static void
EditHalfStrengthSkinEnvelope(const UsdStageRefPtr &stage,
                             const SdfPath &rigPath)
{
    EditInSession(stage);
    const UsdPrim skin = FirstPrimOfType(stage, rigPath, "RigExecSkinMover");
    CHECK(skin);
    if (!skin) return;
    UsdAttribute weight = skin.GetAttribute(TfToken("inputs:defaultWeight"));
    if (!weight) {
        weight = skin.CreateAttribute(TfToken("inputs:defaultWeight"),
                                      SdfValueTypeNames->Float);
    }
    weight.Set(0.5f);
}

// ---------------------------------------------------------------------------
// An interactive override on an attribute that is a connection SOURCE.
//
// A shared-envelope idiom: many constraints' inputs:defaultWeight connected to
// one upstream attribute, and the drag that switches them all off stands on
// the upstream one. The program classified each input by walking that
// connection chain, so the override belongs to every input on the walk -- not
// only to the head attribute the walk started at.
// ---------------------------------------------------------------------------

static SdfPath
ShareOneEnvelopeAcrossConstraints(const UsdStageRefPtr &stage,
                                  const SdfPath &rigPath)
{
    EditInSession(stage);
    const UsdPrim skin = FirstPrimOfType(stage, rigPath, "RigExecSkinMover");
    if (!skin) { ++failures; return SdfPath(); }
    UsdAttribute shared = skin.GetAttribute(TfToken("inputs:defaultWeight"));
    if (!shared) {
        shared = skin.CreateAttribute(TfToken("inputs:defaultWeight"),
                                      SdfValueTypeNames->Float);
    }
    shared.Set(1.0f);
    size_t connected = 0;
    for (const UsdPrim &prim : UsdPrimRange(stage->GetPrimAtPath(rigPath))) {
        const std::string type = prim.GetTypeName().GetString();
        if (type.size() <= 10 ||
            type.compare(type.size() - 10, 10, "Constraint") != 0) {
            continue;
        }
        UsdAttribute weight =
            prim.GetAttribute(TfToken("inputs:defaultWeight"));
        if (!weight) {
            weight = prim.CreateAttribute(TfToken("inputs:defaultWeight"),
                                          SdfValueTypeNames->Float);
        }
        // SetConnections, not AddConnection: an explicit list replaces the
        // weaker opinion rather than joining it, and a scalar input may
        // carry at most one connection.
        weight.SetConnections({shared.GetPath()});
        ++connected;
    }
    CHECK(connected > 0);
    return shared.GetPath();
}

static void
TestAnOverrideOnAConnectionSourceIsFollowed(const std::string &examplesDir)
{
    const std::string stagePath = examplesDir + "/biped/Biped_anim.usda";
    UsdStageRefPtr stage = UsdStage::Open(stagePath);
    UsdStageRefPtr referenceStage = UsdStage::Open(stagePath);
    CHECK(stage && referenceStage);
    if (!stage || !referenceStage) return;
    const SdfPath rigPath = FindRig(stage);
    if (rigPath.IsEmpty()) { ++failures; return; }
    const SdfPath shared = ShareOneEnvelopeAcrossConstraints(stage, rigPath);
    CHECK(ShareOneEnvelopeAcrossConstraints(referenceStage, rigPath) == shared);
    if (shared.IsEmpty()) return;

    const std::vector<RigExecValueOverride> overrides{RigExecValueOverride{
        shared.GetPrimPath(), TfToken(), shared.GetNameToken(),
        VtValue(0.0f)}};

    RigExecRigEvaluator rig(stage, rigPath);
    rig.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::vector<std::string> errors;
    if (!rig.Compile(&errors)) {
        ++failures;
        for (const std::string &e : errors) std::printf("  ERR %s\n", e.c_str());
    }
    std::vector<std::string> reasons;
    if (!rig.IsBakeable(&reasons)) {
        ++failures;
        std::printf("FAIL shared envelope: not bakeable\n");
        for (const std::string &reason : reasons) {
            std::printf("    %s\n", reason.c_str());
        }
        return;
    }
    const RigExecRigPose before = rig.Evaluate(UsdTimeCode(1.0));
    CHECK(before.valid);
    rig.SetInteractiveOverrides(overrides);
    const RigExecRigPose dragged = rig.Evaluate(UsdTimeCode(1.0));
    CHECK(dragged.valid);
    // Answered BY THE PROGRAM. A fallback is also a correct answer and would
    // make the comparison below compare the dynamic path with itself.
    CHECK(rig.GetBakedGenerationCount() == 2);

    RigExecRigEvaluator referenceRig(referenceStage, rigPath);
    MakeItTheReference(&referenceRig);
    CHECK(referenceRig.Compile(&errors));
    referenceRig.SetInteractiveOverrides(overrides);
    const RigExecRigPose reference = referenceRig.Evaluate(UsdTimeCode(1.0));
    CHECK(reference.valid);
    CompareEveryMap("an override on a shared envelope, on Biped_anim.usda",
                    reference, dragged);

    // And it is a drag that DOES something, so an override silently dropped
    // on both paths could not pass this.
    size_t moved = 0;
    for (const auto &[path, frame] : dragged.jointFramesFinal) {
        if (!SameFrame(frame, JointFrame(before, path))) ++moved;
    }
    CHECK(moved > 0);
    rig.ClearInteractiveOverrides();
    CompareEveryMap("a released shared-envelope override, on Biped_anim.usda",
                    before, rig.Evaluate(UsdTimeCode(1.0)));
}

// ---------------------------------------------------------------------------
// The parity comparator itself.
//
// Every other assertion about BakedWithParityCheck in this suite, and all
// three *BakedParity ctest entries, say the comparator found NOTHING -- which
// is exactly what a comparator that does nothing also says. These are the
// positive direction: two poses that DO disagree, one domain at a time, and
// the exact count and diagnostic the comparison must produce.
// ---------------------------------------------------------------------------

static RigExecPointFrame
MovedFrame(double dy)
{
    RigExecPointFrame frame;
    frame.points[0] += GfVec3d(0, dy, 0);
    return frame;
}

static void
CheckOneMismatch(const char *what, const RigExecRigPose &reference,
                 const RigExecRigPose &baked, const char *expectedSubstring)
{
    RigExecRigPose out;
    RigExecComparePoses(reference, baked, &out);
    if (out.bakedParityMismatches != 1) {
        ++failures;
        std::printf("FAIL parity comparator: %s produced %zu mismatch(es), "
                    "expected 1\n", what, out.bakedParityMismatches);
        return;
    }
    bool found = false;
    for (const std::string &diagnostic : out.diagnostics) {
        if (diagnostic.rfind("baked parity: ", 0) == 0 &&
            diagnostic.find(expectedSubstring) != std::string::npos) {
            found = true;
        }
    }
    if (!found) {
        ++failures;
        std::printf("FAIL parity comparator: %s reported no diagnostic "
                    "naming \"%s\"\n", what, expectedSubstring);
        for (const std::string &diagnostic : out.diagnostics) {
            std::printf("    %s\n", diagnostic.c_str());
        }
    }
}

static void
TestTheParityComparatorFindsWhatIsThere()
{
    const SdfPath a("/Rig/Joints/a");
    const SdfPath b("/Rig/Joints/b");
    const SdfPath solver("/Rig/Solvers/s");
    const SdfPath weight("/Rig/Weights/w");
    const SdfPath volume("/Rig/Weights/v");

    // A pose the two paths agree on, so every case below differs in one
    // domain and one domain only.
    RigExecRigPose agreed;
    agreed.jointFramesBase[a] = RigExecPointFrame();
    agreed.jointFramesFinal[a] = RigExecPointFrame();
    agreed.jointMatricesFinal[a] = GfMatrix4d(1.0);
    agreed.controlFrames[a] = RigExecPointFrame();
    agreed.providerXforms[a] = GfMatrix4d(1.0);
    agreed.providerBaseXforms[a] = GfMatrix4d(1.0);
    agreed.movedProperties[a.AppendProperty(TfToken("points"))] =
        VtValue(1.0f);
    agreed.solverFrames[solver] =
        std::vector<RigExecPointFrame>{RigExecPointFrame()};
    // The two weight domains and the convergence flag. Nothing that bakes
    // today publishes any of them, which is exactly why they are here: a
    // domain nobody fills is a domain nobody notices the comparator is blind
    // to, until the generation that fills it.
    agreed.weightFields[weight] =
        RigExecResolvedWeightField{a.AppendProperty(TfToken("points")),
                                   {1.0f, 0.5f, 0.0f}};
    agreed.weightFrames[volume] = GfMatrix4d(1.0);
    {
        RigExecRigPose out;
        RigExecComparePoses(agreed, agreed, &out);
        CHECK(out.bakedParityMismatches == 0);
        CHECK(out.diagnostics.empty());
    }

    // Each of the ten compared domains, one at a time. Not a loop: the
    // point is that every domain is named, and a loop over a list would be
    // the same omission the comparison could make.
    {
        RigExecRigPose d = agreed;
        d.jointFramesBase[a] = MovedFrame(1.0);
        CheckOneMismatch("base joint frame", agreed, d, "base joint frame");
    }
    {
        RigExecRigPose d = agreed;
        d.jointFramesFinal[a] = MovedFrame(1.0);
        CheckOneMismatch("final joint frame", agreed, d, "final joint frame");
    }
    {
        RigExecRigPose d = agreed;
        d.jointMatricesFinal[a][3][1] = 1.0;
        CheckOneMismatch("joint matrix", agreed, d, "joint matrix");
    }
    {
        RigExecRigPose d = agreed;
        d.controlFrames[a] = MovedFrame(1.0);
        CheckOneMismatch("control frame", agreed, d, "control frame");
    }
    {
        RigExecRigPose d = agreed;
        d.providerXforms[a][3][1] = 1.0;
        CheckOneMismatch("provider transform", agreed, d,
                         "provider transform");
    }
    {
        RigExecRigPose d = agreed;
        d.providerBaseXforms[a][3][1] = 1.0;
        CheckOneMismatch("provider base transform", agreed, d,
                         "provider base transform");
    }
    {
        RigExecRigPose d = agreed;
        d.movedProperties[a.AppendProperty(TfToken("points"))] =
            VtValue(2.0f);
        CheckOneMismatch("moved property", agreed, d, "moved property");
    }
    {
        RigExecRigPose d = agreed;
        d.solverFrames[solver].push_back(RigExecPointFrame());
        CheckOneMismatch("solver frames", agreed, d, "solver frames");
    }
    // A resolved weight field is the pair (what it weights, the floats): a
    // field that weights the right property with the wrong numbers and one
    // that weights the wrong property with the right numbers are each a
    // different influence, and both have to be one mismatch.
    {
        RigExecRigPose d = agreed;
        d.weightFields[weight].weights[1] = 0.25f;
        CheckOneMismatch("weight field values", agreed, d, "weight field");
    }
    {
        RigExecRigPose d = agreed;
        d.weightFields[weight].target = b.AppendProperty(TfToken("points"));
        CheckOneMismatch("weight field target", agreed, d, "weight field");
    }
    // A field that ran one element short: the cardinality flips with what
    // the weight object names as its target, so a truncated field is a real
    // shape of disagreement rather than a hypothetical one.
    {
        RigExecRigPose d = agreed;
        d.weightFields[weight].weights.pop_back();
        CheckOneMismatch("a short weight field", agreed, d, "weight field");
    }
    {
        RigExecRigPose d = agreed;
        d.weightFrames[volume][3][1] = 1.0;
        CheckOneMismatch("weight frame", agreed, d, "weight frame");
    }

    // The scalars of the generation. A program that lands on the right
    // points while reporting different work is still a second rig, and a map
    // comparison cannot see that at all.
    {
        RigExecRigPose d = agreed;
        d.moverGraphRevisionsCreated = 3;
        CheckOneMismatch("mover graph revisions created", agreed, d,
                         "mover graph revisions created");
    }
    {
        RigExecRigPose d = agreed;
        d.moverGraphRevisionsExecuted = 1;
        CheckOneMismatch("mover graph revisions executed", agreed, d,
                         "mover graph revisions executed");
    }
    {
        RigExecRigPose d = agreed;
        d.moverGraphSchedulesBuilt = 2;
        CheckOneMismatch("mover graph schedules built", agreed, d,
                         "mover graph schedules built");
    }
    // solverEvaluations is the one published scalar the comparator leaves
    // alone: it counts requests the dynamic path's per-batch exec cache lets
    // it skip and the program has no cache to skip with. Asserted here so
    // the omission is a decision and not an oversight.
    {
        RigExecRigPose d = agreed;
        d.solverEvaluations = 14;
        RigExecRigPose out;
        RigExecComparePoses(agreed, d, &out);
        CHECK(out.bakedParityMismatches == 0);
    }
    {
        RigExecRigPose d = agreed;
        d.solverOverrideRounds = 4;
        CheckOneMismatch("solver override rounds", agreed, d,
                         "solver override rounds");
    }
    // Convergence: the scalar whose wrong answer is the quietest, because a
    // consumer that reads "the overrides settled" reads every point above it
    // as final.
    {
        RigExecRigPose d = agreed;
        d.solverOverridesConverged = false;
        CheckOneMismatch("solver overrides converged", agreed, d,
                         "solver overrides converged");
    }
    // Order-sensitive: the diagnostics are a sequence, and the same lines in
    // a different order describe a different walk.
    {
        RigExecRigPose reference = agreed;
        reference.diagnostics = {"MoverFailed A", "constraint B"};
        RigExecRigPose d = agreed;
        d.diagnostics = {"constraint B", "MoverFailed A"};
        CheckOneMismatch("reordered diagnostics", reference, d,
                         "diagnostics differ");
    }
    {
        RigExecRigPose reference = agreed;
        reference.diagnostics = {"MoverFailed A"};
        CheckOneMismatch("a diagnostic only the reference has", reference,
                         agreed, "diagnostics differ");
    }

    // The two asymmetric cases, which are the ones a one-directional
    // comparison would miss: a key only the reference has, and a key only
    // the baked pose has.
    {
        RigExecRigPose reference = agreed;
        reference.jointFramesFinal[b] = RigExecPointFrame();
        CheckOneMismatch("a joint the program never published", reference,
                         agreed, "no final joint frame");
    }
    {
        RigExecRigPose d = agreed;
        d.jointFramesFinal[b] = RigExecPointFrame();
        CheckOneMismatch("a joint only the program published", agreed, d,
                         "unexpected final joint frame");
    }
    // And the same two directions on the domains that have no baked
    // counterpart yet: a program that publishes NO weight field for a mover
    // that consumed one is the exact shape this work has to catch.
    {
        RigExecRigPose reference = agreed;
        reference.weightFields[SdfPath("/Rig/Weights/x")] =
            RigExecResolvedWeightField{a.AppendProperty(TfToken("points")),
                                       {1.0f}};
        CheckOneMismatch("a weight field the program never published",
                         reference, agreed, "no weight field");
    }
    {
        RigExecRigPose d = agreed;
        d.weightFrames[SdfPath("/Rig/Weights/y")] = GfMatrix4d(1.0);
        CheckOneMismatch("a weight frame only the program published", agreed,
                         d, "unexpected weight frame");
    }

    // Every domain at once, so the count is a count and not a flag.
    {
        RigExecRigPose d = agreed;
        d.jointFramesBase[a] = MovedFrame(1.0);
        d.jointFramesFinal[a] = MovedFrame(1.0);
        d.jointMatricesFinal[a][3][1] = 1.0;
        d.controlFrames[a] = MovedFrame(1.0);
        d.providerXforms[a][3][1] = 1.0;
        d.providerBaseXforms[a][3][1] = 1.0;
        d.movedProperties[a.AppendProperty(TfToken("points"))] =
            VtValue(2.0f);
        d.solverFrames[solver].clear();
        d.weightFields[weight].weights[0] = 0.25f;
        d.weightFrames[volume][3][1] = 1.0;
        d.moverGraphRevisionsCreated = 3;
        d.moverGraphRevisionsExecuted = 1;
        d.moverGraphSchedulesBuilt = 2;
        d.solverOverrideRounds = 4;
        d.solverOverridesConverged = false;
        d.diagnostics = {"MoverFailed A"};
        RigExecRigPose out;
        RigExecComparePoses(agreed, d, &out);
        CHECK(out.bakedParityMismatches == 16);
        CHECK(out.diagnostics.size() == 16);
    }
}

// Exact equality, not a tolerance -- on a real generation of the shipped rig,
// where "close" is the failure mode a tolerance would hide.
static void
TestTheParityComparatorIsExact(const std::string &examplesDir)
{
    UsdStageRefPtr stage = UsdStage::Open(examplesDir + "/biped/Biped.usda");
    CHECK(stage);
    if (!stage) return;
    const SdfPath rigPath = FindRig(stage);
    if (rigPath.IsEmpty()) { ++failures; return; }
    RigExecRigEvaluator rig(stage, rigPath);
    std::vector<std::string> errors;
    CHECK(rig.Compile(&errors));
    const RigExecRigPose reference = rig.Evaluate(UsdTimeCode(1.0));
    CHECK(reference.valid);
    CHECK(!reference.jointMatricesFinal.empty());
    if (reference.jointMatricesFinal.empty()) return;

    // A copy with ONE number moved: everything else -- the diagnostics and
    // the work counters the comparator also compares -- has to stay equal,
    // or the count below stops being about the perturbation.
    RigExecRigPose perturbed = reference;
    perturbed.bakedParityMismatches = 0;
    perturbed.jointMatricesFinal.begin()->second[3][1] += 1e-9;
    RigExecRigPose out;
    RigExecComparePoses(reference, perturbed, &out);
    CHECK(out.bakedParityMismatches == 1);

    // And the same pose against itself is silent, so the case above is the
    // perturbation and not the comparison being noisy.
    RigExecRigPose quiet;
    RigExecComparePoses(reference, reference, &quiet);
    CHECK(quiet.bakedParityMismatches == 0);
}

// ---------------------------------------------------------------------------
// Baked requested on a DIRTY epoch.
//
// SetEvaluationMode can only build while the epoch is settled, and every
// notice raises the dirty flag -- so "edit the scene, then turn the mode on",
// which is what a UI does every time, used to leave the program unbuilt for
// the rest of the epoch while IsBakeable kept saying yes.
// ---------------------------------------------------------------------------

static void
TestBakedModeRequestedOnADirtyEpoch(const std::string &examplesDir)
{
    const std::string stagePath = examplesDir + "/biped/Biped.usda";
    UsdStageRefPtr stage = UsdStage::Open(stagePath);
    UsdStageRefPtr referenceStage = UsdStage::Open(stagePath);
    CHECK(stage && referenceStage);
    if (!stage || !referenceStage) return;
    const SdfPath rigPath = FindRig(stage);
    if (rigPath.IsEmpty()) { ++failures; return; }

    RigExecRigEvaluator rig(stage, rigPath);
    RigExecRigEvaluator referenceRig(referenceStage, rigPath);
    MakeItTheReference(&referenceRig);
    std::vector<std::string> errors;
    CHECK(rig.Compile(&errors));
    CHECK(referenceRig.Compile(&errors));
    // One dynamic generation first, so the mode is turned on mid-session
    // rather than on a rig that has never run.
    CHECK(rig.Evaluate(UsdTimeCode(1.0)).valid);
    CHECK(referenceRig.Evaluate(UsdTimeCode(1.0)).valid);

    // The edit: a prim the rig has never heard of, which is the cheapest
    // thing that raises the dirty flag without moving the epoch.
    stage->DefinePrim(SdfPath("/Scratch"), TfToken("Scope"));
    referenceStage->DefinePrim(SdfPath("/Scratch"), TfToken("Scope"));

    rig.SetEvaluationMode(RigExecEvaluationMode::Baked);
    CHECK(rig.IsBakeable(nullptr));

    for (double frame = 1; frame <= 5; ++frame) {
        const std::string where =
            "Baked requested on a dirty epoch, frame " +
            std::to_string(int(frame));
        const RigExecRigPose reference =
            referenceRig.Evaluate(UsdTimeCode(frame));
        const RigExecRigPose baked = rig.Evaluate(UsdTimeCode(frame));
        CHECK(reference.valid);
        CHECK(baked.valid);
        if (!reference.valid || !baked.valid) continue;
        CompareEveryMap(where, reference, baked);
        CompareMaps("provider transform", where, reference.providerXforms,
                    baked.providerXforms,
                    [](const GfMatrix4d &a, const GfMatrix4d &b) {
                        return a == b;
                    });
    }
    // The request was honoured: a program was built and it answered the
    // generations. Without the lazy build both of these stay at zero and
    // every comparison above passes by comparing the dynamic path with
    // itself.
    CHECK(rig.GetBakedProgramBuildCount() > 0);
    CHECK(rig.GetBakedGenerationCount() > 0);
}

// ---------------------------------------------------------------------------
// A rebuild inside an epoch publishes the same GENERATION, counters included.
//
// A value edit that hits the capture index rebuilds the program while the
// geometry graphs it accounts for stand. The dynamic path keeps its graphs
// across the same edit and reports nothing created; a program that started
// its accounting over would report everything created, and say so in the
// mover-graph diagnostic, for a rig that built nothing.
// ---------------------------------------------------------------------------

// rest:tx on the biped's root joint: a captured constant, so the program
// rebuilds, and a value the epoch digest is blind to, so nothing recompiles.
static void
MoveARestChannel(const UsdStageRefPtr &stage)
{
    EditInSession(stage);
    const UsdPrim hips =
        stage->GetPrimAtPath(SdfPath("/Biped/Rig/Joints/hips_bind"));
    CHECK(hips);
    if (!hips) return;
    UsdAttribute rest = hips.GetAttribute(TfToken("rest:tx"));
    if (!rest) {
        rest = hips.CreateAttribute(TfToken("rest:tx"),
                                    SdfValueTypeNames->Double);
    }
    CHECK(rest);
    if (rest) CHECK(rest.Set(3.0));
}

static void
TestAnInEpochRebuildPublishesTheSameCounters(const std::string &examplesDir)
{
    const std::string stagePath = examplesDir + "/biped/Biped.usda";
    UsdStageRefPtr bakedStage = UsdStage::Open(stagePath);
    UsdStageRefPtr dynamicStage = UsdStage::Open(stagePath);
    UsdStageRefPtr parityStage = UsdStage::Open(stagePath);
    CHECK(bakedStage && dynamicStage && parityStage);
    if (!bakedStage || !dynamicStage || !parityStage) return;
    const SdfPath rigPath = FindRig(bakedStage);
    if (rigPath.IsEmpty()) { ++failures; return; }

    RigExecRigEvaluator bakedRig(bakedStage, rigPath);
    RigExecRigEvaluator dynamicRig(dynamicStage, rigPath);
    RigExecRigEvaluator parityRig(parityStage, rigPath);
    MakeItTheReference(&dynamicRig);
    bakedRig.SetEvaluationMode(RigExecEvaluationMode::Baked);
    parityRig.SetEvaluationMode(RigExecEvaluationMode::BakedWithParityCheck);
    std::vector<std::string> errors;
    CHECK(bakedRig.Compile(&errors));
    CHECK(dynamicRig.Compile(&errors));
    CHECK(parityRig.Compile(&errors));
    CHECK(bakedRig.IsBakeable(nullptr));
    const size_t epoch = bakedRig.GetBindingEpochDigest();

    const auto step = [&](double frame, const char *what) {
        const std::string where =
            std::string(what) + " frame " + std::to_string(int(frame));
        const RigExecRigPose reference = dynamicRig.Evaluate(UsdTimeCode(frame));
        const RigExecRigPose baked = bakedRig.Evaluate(UsdTimeCode(frame));
        const RigExecRigPose parity = parityRig.Evaluate(UsdTimeCode(frame));
        CHECK(reference.valid);
        CHECK(baked.valid);
        CHECK(parity.valid);
        if (!reference.valid || !baked.valid) return;
        CompareEveryMap(where, reference, baked);
        CompareGenerationScalars(where, reference, baked);
        // The mode's own comparator over the same sequence, which is what a
        // caller gets when they ask rather than writing the loop above.
        if (parity.bakedParityMismatches != 0) {
            ++failures;
            std::printf("FAIL %s: %zu parity mismatch(es)\n", where.c_str(),
                        parity.bakedParityMismatches);
            for (const std::string &diagnostic : parity.diagnostics) {
                if (diagnostic.rfind("baked parity", 0) == 0) {
                    std::printf("    %s\n", diagnostic.c_str());
                }
            }
        }
    };

    step(1, "before the rest edit");
    step(2, "before the rest edit");
    const size_t builds = bakedRig.GetBakedProgramBuildCount();
    MoveARestChannel(bakedStage);
    MoveARestChannel(dynamicStage);
    MoveARestChannel(parityStage);
    step(3, "after the rest edit");
    step(4, "after the rest edit");

    // The edit did what the test needs it to do: it rebuilt the program
    // INSIDE the epoch. Either half missing makes the comparisons above
    // vacuous -- a recompile would legitimately rebuild the graphs too.
    if (bakedRig.GetBakedProgramBuildCount() <= builds) {
        ++failures;
        std::printf("FAIL in-epoch rebuild: the rest edit rebuilt nothing\n");
    }
    CHECK(bakedRig.GetBindingEpochDigest() == epoch);
    CHECK(bakedRig.GetBakedGenerationCount() == 4);
}

// ---------------------------------------------------------------------------
// Every shipped example, both modes.
//
// The four stages above are the ones chosen for being bakeable; this is the
// other direction -- whatever is in examples/, whether it bakes or not. A rig
// that declines has to SAY why and build nothing, and a rig that bakes has to
// publish the same generation as the dynamic path -- every map, every
// compared scalar and the diagnostics in order. Two evaluators over two
// independently opened stages, so neither can leak a warm cache into the
// other.
// ---------------------------------------------------------------------------

// The examples that are ALLOWED to decline the bake, by basename.
//
// IT IS EMPTY, and that is the point the four Phase 3 groups were aiming at:
// every example rig in the tree now bakes. It stays here, empty, rather than
// being deleted with its machinery, because the machinery is what makes the
// emptiness mean something -- an example that declines is now unconditionally
// a failure, with no line anyone can add quietly to make the sweep green
// again. Adding a name back is a deliberate, reviewable act.
//
// Both directions are FAILURES, and so is an entry nothing matched. A list
// that is only read when a rig declines goes stale silently -- a group that
// bakes its feature and leaves its line here hands the next person a list
// that no longer says what still has to be done, and re-declining that same
// rig later would then be green.
//
// The deliberate negative is NOT here and must not be moved here: it is
// TestANonBakeableRigFallsBack, which builds its rig in memory (a connected
// posed:space on a provider) precisely so that the shipped examples can all
// be required to bake.
static const char *const *const kExpectedToDecline = nullptr;
static constexpr size_t kExpectedToDeclineCount = 0;

// The index of \p name in kExpectedToDecline, or -1 if it is not listed.
static int
_ExpectedToDeclineIndex(const std::string &name)
{
    for (size_t i = 0; i < kExpectedToDeclineCount; ++i) {
        if (name == kExpectedToDecline[i]) return int(i);
    }
    return -1;
}

// The frames to sweep a stage at, from the stage's OWN authored range.
//
// The sweep used to run every example at frames 1, 2 and 3. The numbered
// examples are authored over 1001-1048, so all three reads held the first
// key and the sweep compared one static pose three times -- it had never
// compared an interpolated frame of any of them. Start, middle and end of
// the authored range instead, and 1-3 for a stage with no authored range
// (the biped, which is keyed over 1-8), so a newly added example is swept at
// frames that differ the day it lands rather than the day someone adds it to
// a table.
static std::vector<double>
_SweepFrames(const UsdStageRefPtr &stage)
{
    if (!stage->HasAuthoredTimeCodeRange()) {
        return {1, 2, 3};
    }
    const double start = stage->GetStartTimeCode();
    const double end = stage->GetEndTimeCode();
    if (!(end > start)) {
        return {start};
    }
    // Truncated, not rounded: a whole-numbered frame reads an authored key
    // on a stage keyed on whole frames and interpolates on one that is not,
    // and both are worth sweeping.
    return {start, start + double(long((end - start) / 2)), end};
}


// ---------------------------------------------------------------------------
// A volume weight that is ITSELF A CONSTRAINT TARGET: the second deliberate
// negative, and like the first it is a property of the rig rather than a gap
// in the bake.
//
// A RigExecSphereWeight is exec-seeded like a joint, so it has a pose seed
// frame and an avar composition. It is also not a RigExecControl and not a
// RigExecJoint, so the moment a constraint targets it the compiler
// catalogues it as a plain UsdGeomXformable as well -- and the two families
// the program's slot table merges, which its comment calls disjoint, are not
// disjoint for this one provider.
//
// The dynamic walk resolves the collision by last writer: the xform-derived
// pass runs after the compose and replaces the volume's rest, base and final
// with an identity rest and a transform read off the stage. Exec's
// computeWeightPacket goes on placing the same volume from its avars. So the
// volume has two placements at once, the program has one slot to hold them
// in, and there is no reading of either side that says which is meant -- the
// dynamic path's own CPU parity mode refuses to publish a reference-phase
// field on such a volume rather than choose.
//
// Refused, therefore, and narrowly: a volume weight on a constraint bakes
// (testRigExecVolumeWeights covers both sample phases); a volume weight a
// constraint MOVES does not.
// ---------------------------------------------------------------------------

static UsdStageRefPtr
MakeAConstrainedVolumeRig()
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));

    stage->DefinePrim(SdfPath("/Asset/Geom"), TfToken("Scope"));
    const UsdPrim mesh = stage->DefinePrim(SdfPath("/Asset/Geom/Slab"),
                                           TfToken("Points"));
    mesh.CreateAttribute(TfToken("points"), SdfValueTypeNames->Point3fArray)
        .Set(VtVec3fArray{GfVec3f(0, 0, 0), GfVec3f(0, 4, 0),
                          GfVec3f(0, 8, 0)});

    const UsdPrim lift = stage->DefinePrim(
        SdfPath("/Asset/Rig/Controls/Lift"), TfToken("RigExecControl"));
    lift.CreateAttribute(TfToken("avars:ty"), SdfValueTypeNames->Double)
        .Set(8.0);

    const UsdPrim sphere = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/Sphere"), TfToken("RigExecSphereWeight"));
    sphere.CreateRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({SdfPath("/Asset/Geom/Slab.points")});
    sphere.CreateAttribute(TfToken("inputs:falloffMin"),
                           SdfValueTypeNames->Float).Set(0.0f);
    sphere.CreateAttribute(TfToken("inputs:falloffMax"),
                           SdfValueTypeNames->Float).Set(8.0f);
    sphere.CreateAttribute(TfToken("rigExec:falloffProfile"),
                           SdfValueTypeNames->Token).Set(TfToken("linear"));
    // `current`, so the field is resolved by the oracle on both paths: a
    // `reference` field on a constrained volume is the arm the dynamic
    // path's own parity mode refuses, and this test is about the SLOT
    // collision rather than about that.
    sphere.CreateAttribute(TfToken("rigExec:samplePhase"),
                           SdfValueTypeNames->Token).Set(TfToken("current"));

    // The constraint that moves the volume. This is the whole fixture.
    const UsdPrim move = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Pose/MoveVolume"),
        TfToken("RigExecPositionConstraint"));
    move.ApplyAPI(TfToken("RigExecMoverAPI"));
    move.CreateRelationship(TfToken("rigExec:moves"))
        .SetTargets({sphere.GetPath()});
    move.CreateRelationship(TfToken("rigExec:sources"))
        .SetTargets({lift.GetPath()});

    const UsdGeomXform pull =
        UsdGeomXform::Define(stage, SdfPath("/Asset/PullTo"));
    pull.MakeMatrixXform().Set(
        GfMatrix4d(1.0).SetTranslate(GfVec3d(6, 0, 0)));
    const UsdPrim constraint = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Sweep/Pull"),
        TfToken("RigExecPositionConstraint"));
    constraint.ApplyAPI(TfToken("RigExecMoverAPI"));
    constraint.CreateRelationship(TfToken("rigExec:moves"))
        .SetTargets({SdfPath("/Asset/Geom/Slab.points")});
    constraint.CreateRelationship(TfToken("rigExec:sources"))
        .SetTargets({pull.GetPath()});
    constraint.CreateRelationship(TfToken("rigExec:weightObject"))
        .SetTargets({sphere.GetPath()});
    return stage;
}

static void
TestAConstrainedVolumeWeightFallsBack()
{
    const char *const what = "a volume weight a constraint moves";
    UsdStageRefPtr stage = MakeAConstrainedVolumeRig();
    const SdfPath rigPath = FindRig(stage);
    if (rigPath.IsEmpty()) { ++failures; return; }
    RigExecRigEvaluator rig(stage, rigPath);
    rig.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::vector<std::string> errors;
    if (!rig.Compile(&errors)) {
        ++failures;
        std::printf("FAIL %s: the fixture does not compile\n", what);
        for (const std::string &error : errors) {
            std::printf("    %s\n", error.c_str());
        }
        return;
    }
    std::vector<std::string> reasons;
    CHECK(!rig.IsBakeable(&reasons));
    bool named = false;
    for (const std::string &reason : reasons) {
        named = named ||
                reason.find("both exec-seeded and xform-derived") !=
                    std::string::npos;
    }
    if (!named) {
        ++failures;
        std::printf("FAIL %s: no reason names the slot collision\n", what);
        for (const std::string &reason : reasons) {
            std::printf("    %s\n", reason.c_str());
        }
    }

    // And the fallback is a real generation, identical to a plain dynamic
    // evaluator's: a refusal that also changed the answer would be worse
    // than the divergence it exists to avoid.
    UsdStageRefPtr referenceStage = MakeAConstrainedVolumeRig();
    RigExecRigEvaluator referenceRig(referenceStage, rigPath);
    errors.clear();
    CHECK(referenceRig.Compile(&errors));
    for (double frame = 1; frame <= 2; ++frame) {
        const RigExecRigPose reference =
            referenceRig.Evaluate(UsdTimeCode(frame));
        const RigExecRigPose fallen = rig.Evaluate(UsdTimeCode(frame));
        CHECK(fallen.valid);
        CHECK(!fallen.movedProperties.empty());
        CompareEveryMap(std::string(what) + " frame " +
                            std::to_string(int(frame)),
                        reference, fallen);
    }
    CHECK(rig.GetBakedGenerationCount() == 0);
}

// ---------------------------------------------------------------------------
// A read phase on rigExec:transform that names a POINT IN THE POSE WALK.
//
// The general form of the three shorthands (base, preceding, final): the
// matrix a mover consumes is the provider's frame as it stood immediately
// after one named constraint, rather than before the walk or after all of
// it. Nothing in examples/ authors it and no other suite builds it, which is
// why the program could refuse it for three phases with no parity evidence
// either way -- so the fixture comes first and the bake follows it.
//
// Two constraints revise ONE joint in the walk, and the mover names the
// first. That is what makes the case discriminating: the phase's answer is
// neither the joint's base frame nor its final one, and a program that
// silently read either would deform the slab to a different place with
// nothing to say so. TestAReadPhaseOnTheTransformIsExact asserts exactly
// that, by building the same rig with a "final" phase and demanding the two
// disagree.
// ---------------------------------------------------------------------------

static UsdStageRefPtr
MakeAPoseWalkReadPhaseRig(const char *phase)
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));

    // Two sources, each moved by an avar, so the two revisions of the joint
    // land in different places and the phase has something to choose between.
    const auto control = [&](const char *name, double tx, double ty) {
        const UsdPrim prim = stage->DefinePrim(
            SdfPath(std::string("/Asset/Rig/Controls/") + name),
            TfToken("RigExecControl"));
        prim.CreateAttribute(TfToken("avars:tx"), SdfValueTypeNames->Double)
            .Set(tx);
        prim.CreateAttribute(TfToken("avars:ty"), SdfValueTypeNames->Double)
            .Set(ty);
        return prim;
    };
    const UsdPrim first = control("First", 3.0, 0.0);
    const UsdPrim second = control("Second", 0.0, 7.0);

    const SdfPath jointPath("/Asset/Rig/Joints/Arm");
    stage->DefinePrim(jointPath, TfToken("RigExecJoint"));

    // Both constraints revise the same joint, in namespace order: A leaves it
    // where First is, B then leaves it where Second is.
    const auto constrain = [&](const char *name, const UsdPrim &source) {
        const UsdPrim prim = stage->DefinePrim(
            SdfPath(std::string("/Asset/Rig/Movers/Constrain/") + name),
            TfToken("RigExecPositionConstraint"));
        prim.ApplyAPI(TfToken("RigExecMoverAPI"));
        prim.CreateRelationship(TfToken("rigExec:moves"))
            .SetTargets({jointPath});
        prim.CreateRelationship(TfToken("rigExec:sources"))
            .SetTargets({source.GetPath()});
        return prim;
    };
    const UsdPrim constraintA = constrain("A", first);
    constrain("B", second);

    stage->DefinePrim(SdfPath("/Asset/Geom"), TfToken("Scope"));
    const UsdPrim mesh = stage->DefinePrim(SdfPath("/Asset/Geom/Slab"),
                                           TfToken("Points"));
    mesh.CreateAttribute(TfToken("points"), SdfValueTypeNames->Point3fArray)
        .Set(VtVec3fArray{GfVec3f(0, 0, 0), GfVec3f(1, 0, 0),
                          GfVec3f(0, 1, 0)});

    const UsdPrim mover = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Geometry/Slide"),
        TfToken("RigExecMatrixMover"));
    mover.ApplyAPI(TfToken("RigExecMoverAPI"));
    mover.CreateRelationship(TfToken("rigExec:moves"))
        .SetTargets({mesh.GetPath().AppendProperty(TfToken("points"))});
    const UsdRelationship transform =
        mover.CreateRelationship(TfToken("rigExec:transform"));
    transform.SetTargets({jointPath});
    // The phase itself, as METADATA on the relationship rather than through
    // the role-named attribute: rigExec:transformReadPhase is the v0.1
    // spelling and admits only base and final, so a pose-walk point can only
    // be said the general way. "A" is spelled as the constraint's own path
    // because an AtPrim phase is an ABSOLUTE prim path and nothing else.
    const std::string authored = std::string(phase) == "atPrim"
                                     ? constraintA.GetPath().GetString()
                                     : std::string(phase);
    transform.SetMetadata(TfToken(RigExecReadPhaseMetadataName), authored);

    // A SKIN mover with the same phase on rigExec:influences is deliberately
    // NOT here. Compile validates an AtPrim phase against
    // binding.transform alone (rigEvaluator.cpp, "names a point in the pose
    // walk, but ... is revised by no pose mover"), and a skin mover's
    // binding.transform is empty -- so such a rig is refused by the DYNAMIC
    // path before either evaluator sees it. The fold answers the influence
    // entries out of the store anyway, because that is what the dynamic
    // fold does with them; neither branch is reachable while the validator
    // stands, and making it reachable would be a change to the dynamic
    // path's answer.
    return stage;
}

// Compiles \p stage twice -- once dynamic, once baked -- and demands the bake
// happened and every published map agrees over four frames. Returns the moved
// points of the last frame so a caller can assert the fixture discriminates.
static VtVec3fArray
BakedAndDynamicAgree(const char *what, const UsdStageRefPtr &stage,
                     const UsdStageRefPtr &referenceStage)
{
    VtVec3fArray moved;
    const SdfPath rigPath = FindRig(stage);
    if (rigPath.IsEmpty()) { ++failures; return moved; }
    RigExecRigEvaluator rig(stage, rigPath);
    rig.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::vector<std::string> errors;
    if (!rig.Compile(&errors)) {
        ++failures;
        std::printf("FAIL %s: the fixture does not compile\n", what);
        for (const std::string &error : errors) {
            std::printf("    %s\n", error.c_str());
        }
        return moved;
    }
    // Named, and not just counted: a refusal here is the whole point of the
    // case, so the reason has to reach the log.
    std::vector<std::string> reasons;
    if (!rig.IsBakeable(&reasons)) {
        ++failures;
        std::printf("FAIL %s: the fixture is not bakeable\n", what);
        for (const std::string &reason : reasons) {
            std::printf("    %s\n", reason.c_str());
        }
        return moved;
    }

    RigExecRigEvaluator referenceRig(referenceStage, rigPath);
    errors.clear();
    CHECK(referenceRig.Compile(&errors));
    for (double frame = 1; frame <= 4; ++frame) {
        const RigExecRigPose reference =
            referenceRig.Evaluate(UsdTimeCode(frame));
        const RigExecRigPose baked = rig.Evaluate(UsdTimeCode(frame));
        CHECK(baked.valid);
        CHECK(!baked.movedProperties.empty());
        CompareEveryMap(std::string(what) + " frame " +
                            std::to_string(int(frame)),
                        reference, baked);
        const auto it =
            baked.movedProperties.find(SdfPath("/Asset/Geom/Slab.points"));
        if (it != baked.movedProperties.end() &&
            it->second.IsHolding<VtVec3fArray>()) {
            moved = it->second.UncheckedGet<VtVec3fArray>();
        }
    }
    // Every generation came from the program: a fallback would have compared
    // the dynamic path with itself.
    CHECK(rig.GetBakedGenerationCount() == 4);
    return moved;
}

static void
TestAReadPhaseOnTheTransformIsExact()
{
    const VtVec3fArray atPrim = BakedAndDynamicAgree(
        "a pose-walk read phase on rigExec:transform",
        MakeAPoseWalkReadPhaseRig("atPrim"),
        MakeAPoseWalkReadPhaseRig("atPrim"));
    const VtVec3fArray final = BakedAndDynamicAgree(
        "a final read phase on rigExec:transform",
        MakeAPoseWalkReadPhaseRig("final"),
        MakeAPoseWalkReadPhaseRig("final"));
    const VtVec3fArray base = BakedAndDynamicAgree(
        "a base read phase on rigExec:transform",
        MakeAPoseWalkReadPhaseRig("base"),
        MakeAPoseWalkReadPhaseRig("base"));
    // The fixture discriminates, measured rather than assumed: if the phase
    // named a point the two shorthands already reach, a program that ignored
    // it entirely would pass every comparison above.
    CHECK(!atPrim.empty());
    CHECK(atPrim != final);
    CHECK(atPrim != base);
}

static void
TestEveryExampleStage(const std::string &examplesDir)
{
    std::vector<std::string> stagePaths =
        TfGlob({examplesDir + "/*.usd*", examplesDir + "/biped/*.usda"});
    std::sort(stagePaths.begin(), stagePaths.end());
    size_t opened = 0, baked = 0, declined = 0;
    // Which allowlist entries the sweep actually reached, so that a line
    // nobody matched -- a rig that now bakes, or one that was renamed or
    // deleted -- is reported instead of sitting there.
    std::vector<bool> declineListHit(kExpectedToDeclineCount, false);
    for (const std::string &stagePath : stagePaths) {
        // A payload file, a clip manifest, a sublayer: an example directory
        // holds plenty of .usd files that are not a rig, and none of them is
        // a failure.
        UsdStageRefPtr dynamicStage = UsdStage::Open(stagePath);
        if (!dynamicStage) continue;
        const SdfPath rigPath = FindRig(dynamicStage);
        if (rigPath.IsEmpty()) continue;
        UsdStageRefPtr bakedStage = UsdStage::Open(stagePath);
        if (!bakedStage) continue;
        ++opened;
        const std::string name = TfGetBaseName(stagePath);

        RigExecRigEvaluator dynamicRig(dynamicStage, rigPath);
        RigExecRigEvaluator bakedRig(bakedStage, rigPath);
        MakeItTheReference(&dynamicRig);
        bakedRig.SetEvaluationMode(RigExecEvaluationMode::Baked);
        std::vector<std::string> errors;
        if (!dynamicRig.Compile(&errors) || !bakedRig.Compile(&errors)) {
            // A rig that does not compile is not this suite's business: the
            // dynamic path reports it and the other suites hold it.
            std::printf("  %-34s does not compile\n", name.c_str());
            continue;
        }
        std::vector<std::string> reasons;
        const bool bakeable = bakedRig.IsBakeable(&reasons);
        const int listed = _ExpectedToDeclineIndex(name);
        if (listed >= 0) declineListHit[listed] = true;
        if (bakeable) {
            ++baked;
            std::printf("  %-34s bakes\n", name.c_str());
            // The other direction: the feature landed, and the line that
            // allowed this rig to decline is now a hole in the sweep.
            if (listed >= 0) {
                ++failures;
                std::printf("FAIL %s: bakes, and is still on the "
                            "expected-to-decline list -- delete the line\n",
                            name.c_str());
            }
        } else {
            ++declined;
            // A silent fallback reads as the mode not working, so a rig that
            // declines must name the feature that stopped it -- and must not
            // have built a program anyway.
            if (reasons.empty()) {
                ++failures;
                std::printf("FAIL %s: declines the bake with no reason\n",
                            name.c_str());
            }
            CHECK(bakedRig.GetBakedProgramBuildCount() == 0);
            // And a decline is a FAILURE unless this example is one of the
            // ones still expected to decline. Printing it and counting it,
            // which is what this did, makes every refusal a passing test --
            // so nothing here can go red when a feature group's work is
            // incomplete, which is the one thing the sweep is for.
            if (listed >= 0) {
                std::printf("  %-34s declines: %s\n", name.c_str(),
                            reasons.front().c_str());
            } else {
                ++failures;
                std::printf("FAIL %s: declines the bake and is not on the "
                            "expected-to-decline list\n", name.c_str());
                for (const std::string &reason : reasons) {
                    std::printf("    %s\n", reason.c_str());
                }
            }
        }
        const std::vector<double> frames = _SweepFrames(dynamicStage);
        const size_t generationsBefore = bakedRig.GetBakedGenerationCount();
        for (const double frame : frames) {
            const std::string where =
                name + " frame " + TfStringify(frame);
            const RigExecRigPose reference =
                dynamicRig.Evaluate(UsdTimeCode(frame));
            const RigExecRigPose answer = bakedRig.Evaluate(UsdTimeCode(frame));
            CHECK(reference.valid == answer.valid);
            if (!reference.valid || !answer.valid) continue;
            // Both evaluators are fresh and have answered exactly the same
            // generations, so the work counters are comparable here in a way
            // they are not where a running evaluator is held against a new
            // one -- a rig that lands on the right points while reporting
            // different work is still a second rig. ComparePose is every map
            // AND every scalar.
            ComparePose(where, reference, answer);
        }
        // A bakeable rig must have ANSWERED from the program, or every
        // comparison above compared the dynamic path with itself.
        if (bakeable && bakedRig.GetBakedGenerationCount() !=
                            generationsBefore + frames.size()) {
            ++failures;
            std::printf("FAIL %s: bakeable, but %zu of %zu generations came "
                        "from the program\n", name.c_str(),
                        bakedRig.GetBakedGenerationCount() -
                            generationsBefore, frames.size());
        }
        if (!bakeable) {
            CHECK(bakedRig.GetBakedProgramBuildCount() == 0);
            CHECK(bakedRig.GetBakedGenerationCount() == generationsBefore);
        }
    }
    std::printf("example sweep: %zu rig stage(s), %zu bake, %zu decline\n",
                opened, baked, declined);
    // The glob found the examples at all: an empty sweep passes silently and
    // proves nothing.
    CHECK(opened > 10);
    // The allowlist is a debt, not a configuration: every line has to be
    // earned by a rig that declined in THIS run, and when the last group
    // lands there are no lines left and `declined` is zero.
    for (size_t i = 0; i < kExpectedToDeclineCount; ++i) {
        if (!declineListHit[i]) {
            ++failures;
            std::printf("FAIL %s: on the expected-to-decline list, but the "
                        "sweep never reached it -- delete the line\n",
                        kExpectedToDecline[i]);
        }
    }
    CHECK(declined == kExpectedToDeclineCount);
}

// ---------------------------------------------------------------------------
// THE INTERVENING-XFORM NEGATIVES, and why they live in THIS suite.
//
// They were written against tests/testRigExecEpochRests, which is where the
// rest work keeps its fixtures. That suite carries REQUIRE_BAKE as of the
// provider-ladder work, and a bake requirement reports a FALLBACK with the
// same "baked parity mismatch" prefix a real disagreement carries -- so a
// fixture that declines on purpose cannot live under one without turning
// the suite's scoreboard red for a feature nobody claimed. They moved here
// at the Phase 4 merge, beside this file's other deliberate negatives (a
// connected posed:space, an unplaceable override, a constrained volume
// weight), and this suite is deliberately not a parity entry. Nothing about
// them weakened in the move: the comparison against the dynamic path is now
// CompareEveryMap, which is every map domain the comparator has rather than
// the four the other suite's local helper compared.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// A plain Xform standing BETWEEN a provider and its anchor.
//
// Exec resolves such a prim as the identity and drops it, so the rig would
// evaluate as if the grouping transform were not there at all. The dynamic
// walk composes it back in at evaluation (_ComposeInterveningXforms): X(P)
// lands BETWEEN a provider and its anchor, which is a uniform right-multiply
// only while every such Xform sits above every chain root, and is not one
// otherwise.
//
// The program refuses both shapes of it -- a non-identity transform, and one
// that is identity today but animates -- and refused them BLIND: no rig and
// no fixture in the tree tripped either, so there was no parity evidence
// either way and no way to tell a correct bake from a plausible one.
//
// These fixtures are that evidence. They assert the refusal by NAME and then
// assert the fallback generation is a plain dynamic evaluator's, every
// published domain of it, so the refusal cannot quietly become a wrong
// answer -- and the day the correction is baked, the one line each case
// carries flips from false to true.
//
// WHY IT IS STILL REFUSED, measured rather than assumed. The correction is
// not confined to the walk. It rewrites the REST frames as well as the base
// ones, and the two halves do not reach the same consumers: the walk's
// frames are pushed back into exec as computePointFrame overrides before
// the authoritative snapshot, so exec's computeMatrix is built from the
// CORRECTED pose and the UN-corrected rest, while the walk's own
// jointMatricesFinal is built from both corrected. Two matrices, one slot,
// one generation -- see
// TestAnInterveningXformMovesTheMeshAndNotTheJointMatrix below, which
// measures both of them on this very fixture. A solver's element rests are
// a third reader on the exec side: they come from computeRestFrame, which
// no override touches.
//
// The program holds ONE rest per slot and derives both matrices from it, so
// expressing that means holding an exec rest and a walk rest side by side
// and routing every consumer to the right one -- which is the same rework
// an animated rest:tx needs, and is why the animated case below is a second
// refusal rather than a second arm of the first. A bake that corrected the
// one rest the program has would publish the right joint matrices and move
// a skinned mesh somewhere nobody asked for, or the reverse; that is
// exactly the shape of wrong the refusal is in front of.
// ---------------------------------------------------------------------------

// \p animated keys the grouping transform instead of authoring it plainly;
// \p identityToday additionally makes every sample the identity at the frames
// the sweep reads, which is the case the "is it identity" test cannot see.
static UsdStageRefPtr
MakeAnInterveningXformRig(bool animated, bool identityToday = false)
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->SetStartTimeCode(1.0);
    stage->SetEndTimeCode(3.0);
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));

    // The grouping Xform. Every rig has these -- a Scope named Joints, a
    // Scope named Controls -- and they compose to the identity, which is
    // why the program's candidate list is not by itself a refusal. This one
    // carries a transform.
    const UsdGeomXform group =
        UsdGeomXform::Define(stage, SdfPath("/Asset/Rig/Group"));
    const UsdGeomXformOp op = group.AddTranslateOp();
    if (animated) {
        op.Set(GfVec3d(0, identityToday ? 0 : 3, 0), UsdTimeCode(1.0));
        op.Set(GfVec3d(0, identityToday ? 0 : 6, 0), UsdTimeCode(3.0));
    } else {
        op.Set(GfVec3d(0, 3, 0));
    }

    const UsdPrim joint = stage->DefinePrim(
        SdfPath("/Asset/Rig/Group/Arm"), TfToken("RigExecJoint"));
    joint.CreateAttribute(TfToken("avars:tx"), SdfValueTypeNames->Double)
        .Set(2.0);
    // A child of the joint, so the correction is exercised where it is NOT a
    // uniform right-multiply: the child's own anchor is the joint, which the
    // pass has already corrected.
    const UsdPrim tip = stage->DefinePrim(
        SdfPath("/Asset/Rig/Group/Arm/Tip"), TfToken("RigExecJoint"));
    tip.CreateAttribute(TfToken("avars:tz"), SdfValueTypeNames->Double)
        .Set(1.0);

    stage->DefinePrim(SdfPath("/Asset/Geom"), TfToken("Scope"));
    const UsdPrim mesh =
        stage->DefinePrim(SdfPath("/Asset/Geom/M"), TfToken("Points"));
    mesh.CreateAttribute(TfToken("points"), SdfValueTypeNames->Point3fArray)
        .Set(VtVec3fArray{GfVec3f(0, 0, 0), GfVec3f(1, 0, 0)});
    const UsdPrim mover = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/M"), TfToken("RigExecMatrixMover"));
    mover.ApplyAPI(TfToken("RigExecMoverAPI"));
    mover.CreateRelationship(TfToken("rigExec:moves"))
        .SetTargets({SdfPath("/Asset/Geom/M.points")});
    mover.CreateRelationship(TfToken("rigExec:transform"))
        .SetTargets({tip.GetPath()});
    return stage;
}

// Compiles \p stage in baked mode, reports whether it bakes and why not, and
// compares every published domain against a dynamic evaluator over the
// sweep. Returns the baked generation count.
static size_t
BakedAgreesWithDynamic(const char *what, const UsdStageRefPtr &stage,
                       const UsdStageRefPtr &referenceStage,
                       bool expectBakeable,
                       std::vector<std::string> *refusals = nullptr)
{
    const SdfPath rig("/Asset/Rig");
    RigExecRigEvaluator baked(stage, rig);
    baked.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::vector<std::string> errors;
    if (!baked.Compile(&errors)) {
        ++failures;
        std::printf("FAIL %s: the fixture does not compile\n", what);
        for (const std::string &error : errors) {
            std::printf("    %s\n", error.c_str());
        }
        return 0;
    }
    std::vector<std::string> reasons;
    const bool bakeable = baked.IsBakeable(&reasons);
    if (bakeable != expectBakeable) {
        ++failures;
        std::printf("FAIL %s: bakeable=%d, expected %d\n", what, int(bakeable),
                    int(expectBakeable));
    }
    if (refusals) {
        *refusals = reasons;
    }

    RigExecRigEvaluator reference(referenceStage, rig);
    errors.clear();
    CHECK(reference.Compile(&errors));
    for (double frame : {1.0, 2.0, 3.0}) {
        const RigExecRigPose expected = reference.Evaluate(UsdTimeCode(frame));
        const RigExecRigPose actual = baked.Evaluate(UsdTimeCode(frame));
        CHECK(expected.valid && actual.valid);
        CHECK(!actual.movedProperties.empty());
        CompareEveryMap(std::string(what) + " frame " +
                            std::to_string(int(frame)),
                        expected, actual);
    }
    return baked.GetBakedGenerationCount();
}

// Asserts that \p reasons names \p expected, so a refusal that changed its
// mind about WHY is a failure rather than a pass.
static void
RefusalNames(const char *what, const std::vector<std::string> &reasons,
             const char *expected)
{
    for (const std::string &reason : reasons) {
        if (reason.find(expected) != std::string::npos) {
            return;
        }
    }
    ++failures;
    std::printf("FAIL %s: no refusal mentions \"%s\"\n", what, expected);
    for (const std::string &reason : reasons) {
        std::printf("    %s\n", reason.c_str());
    }
}

static void
TestAnInterveningXformAboveAProvider()
{
    // The grouping transform reaches the pose: without it the tip would sit
    // at (2, 0, 1) and the mesh with it. Measured, not assumed, because a
    // program that dropped X(P) would agree with a reference that also
    // dropped it -- and the reference here is the DYNAMIC path, which does
    // not.
    const UsdStageRefPtr probeStage = MakeAnInterveningXformRig(false);
    RigExecRigEvaluator probe(probeStage, SdfPath("/Asset/Rig"));
    CHECK(probe.Compile());
    const RigExecRigPose pose = probe.Evaluate(UsdTimeCode(1.0));
    CHECK(pose.valid);
    const auto tip = pose.jointFramesFinal.find(
        SdfPath("/Asset/Rig/Group/Arm/Tip"));
    CHECK(tip != pose.jointFramesFinal.end());
    if (tip != pose.jointFramesFinal.end()) {
        CHECK(std::abs(tip->second.points[0][1] - 3.0) < 1e-9);
    }

    // The refusal, by name, and the fallback, in full.
    std::vector<std::string> refusals;
    const char *const what = "an intervening Xform above a provider";
    const size_t generations = BakedAgreesWithDynamic(
        what, MakeAnInterveningXformRig(false),
        MakeAnInterveningXformRig(false), /* expectBakeable = */ false,
        &refusals);
    RefusalNames(what, refusals, "intervening Xform above provider");
    // Nothing ran baked, and the comparison above still held: the fallback
    // is the dynamic path, not a program that answered anyway.
    CHECK(generations == 0);
}

static void
TestAnAnimatedXformAboveAProvider()
{
    std::vector<std::string> refusals;
    const char *const what = "an animated Xform above a provider";
    const size_t generations = BakedAgreesWithDynamic(
        what, MakeAnInterveningXformRig(true), MakeAnInterveningXformRig(true),
        /* expectBakeable = */ false, &refusals);
    RefusalNames(what, refusals, "animated Xform above provider");
    CHECK(generations == 0);

    // And the shape an "is it identity today" test cannot see: every sample
    // the sweep reads IS the identity, so the transform composes to nothing
    // at every frame anyone looks at -- and it is still refused, because the
    // epoch is not a frame. A bake that judged the transform once would pass
    // this one and be wrong about the case above.
    refusals.clear();
    const char *const quietWhat =
        "an identity-valued animated Xform above a provider";
    const size_t quiet = BakedAgreesWithDynamic(
        quietWhat, MakeAnInterveningXformRig(true, /* identityToday = */ true),
        MakeAnInterveningXformRig(true, /* identityToday = */ true),
        /* expectBakeable = */ false, &refusals);
    RefusalNames(quietWhat, refusals, "animated Xform above provider");
    CHECK(quiet == 0);
}

// WHY the refusals above are still refusals, as a running measurement of the
// DYNAMIC path rather than a paragraph about it.
//
// The correction rewrites the walk's rest frames as well as its base ones,
// and the two halves do not reach the same consumers. The walk's frames are
// pushed BACK INTO EXEC as computePointFrame overrides before the
// authoritative snapshot is evaluated (`_taps->Evaluate(time,
// jointOverrides)`), and every exec computation downstream of a frame then
// sees the corrected pose -- while computeRestFrame, which no override
// touches, goes on reading the authored rest:space and rest avars and knows
// nothing about the grouping transform. So exec's computeMatrix, which a
// geometry mover reads in the base phase, is
//
//     PointsToMatrix(rest_exec, pose_corrected)
//
// while pose.jointMatricesFinal, built in the walk, is
//
//     PointsToMatrix(rest_corrected, pose_corrected).
//
// On this fixture those are two different matrices -- translate (2, 3, 1)
// and translate (2, 0, 1) -- and both are published in the same generation.
// A program that holds ONE rest per slot can produce one of them or the
// other and not both: correcting its rest publishes the right joint matrix
// and moves the mesh to the wrong place, and leaving it uncorrected does the
// reverse. That is the rework the refusals above are in front of (an exec
// rest and a walk rest side by side, every consumer routed to the right
// one), and it is the same rework an animated rest:tx needs.
//
// Pinned here as three assertions about today's dynamic answers, so the
// branch that lands the correction has to decide about this case
// deliberately: if any of the three moves, the dynamic path's answer
// changed, and that is a decision rather than a test to update.
static void
TestAnInterveningXformMovesTheMeshAndNotTheJointMatrix()
{
    // The same rig twice, with and without the grouping transform. Nothing
    // else differs, so every difference below is X(P) and only X(P).
    const UsdStageRefPtr withXform = MakeAnInterveningXformRig(false);
    const UsdStageRefPtr withoutXform = MakeAnInterveningXformRig(false);
    {
        bool resets = false;
        const std::vector<UsdGeomXformOp> ops =
            UsdGeomXform(
                withoutXform->GetPrimAtPath(SdfPath("/Asset/Rig/Group")))
                .GetOrderedXformOps(&resets);
        CHECK(ops.size() == 1);
        if (ops.size() != 1) return;
        ops.front().Set(GfVec3d(0, 0, 0));
    }

    const SdfPath rigPath("/Asset/Rig");
    RigExecRigEvaluator moved(withXform, rigPath);
    RigExecRigEvaluator plain(withoutXform, rigPath);
    CHECK(moved.Compile());
    CHECK(plain.Compile());
    const RigExecRigPose a = moved.Evaluate(UsdTimeCode(1.0));
    const RigExecRigPose b = plain.Evaluate(UsdTimeCode(1.0));
    CHECK(a.valid && b.valid);
    if (!a.valid || !b.valid) return;

    // 1. The published FRAMES follow the grouping Xform: the correction is
    //    what put them in the group's space.
    const SdfPath tipPath("/Asset/Rig/Group/Arm/Tip");
    const auto tipA = a.jointFramesFinal.find(tipPath);
    const auto tipB = b.jointFramesFinal.find(tipPath);
    CHECK(tipA != a.jointFramesFinal.end());
    CHECK(tipB != b.jointFramesFinal.end());
    if (tipA == a.jointFramesFinal.end() ||
        tipB == b.jointFramesFinal.end()) {
        return;
    }
    CHECK(std::abs((tipA->second.points[0][1] - tipB->second.points[0][1]) -
                   3.0) < 1e-9);

    // 2. The published MATRIX does not. Rest and pose are corrected
    //    together, so X cancels out of the rest-to-pose map -- translate
    //    (2, 0, 1) either way.
    const auto matrixA = a.jointMatricesFinal.find(tipPath);
    const auto matrixB = b.jointMatricesFinal.find(tipPath);
    CHECK(matrixA != a.jointMatricesFinal.end());
    CHECK(matrixB != b.jointMatricesFinal.end());
    if (matrixA != a.jointMatricesFinal.end() &&
        matrixB != b.jointMatricesFinal.end()) {
        CHECK(matrixA->second == matrixB->second);
        CHECK(GfIsClose(matrixA->second.ExtractTranslation(),
                        GfVec3d(2, 0, 1), 1e-9));
    }

    // 3. And the mesh a base-phase mover drives off that same joint DOES
    //    follow it, by exactly X: exec composed its matrix from the
    //    overridden pose and the un-overridden rest. Two answers, one
    //    generation, one slot.
    const SdfPath target("/Asset/Geom/M.points");
    const auto pointsA = a.movedProperties.find(target);
    const auto pointsB = b.movedProperties.find(target);
    CHECK(pointsA != a.movedProperties.end());
    CHECK(pointsB != b.movedProperties.end());
    if (pointsA == a.movedProperties.end() ||
        pointsB == b.movedProperties.end()) {
        return;
    }
    const VtVec3fArray movedWith = pointsA->second.Get<VtVec3fArray>();
    const VtVec3fArray movedWithout = pointsB->second.Get<VtVec3fArray>();
    CHECK(movedWith.size() == movedWithout.size());
    if (movedWith.size() != movedWithout.size()) return;
    for (size_t i = 0; i < movedWith.size(); ++i) {
        if (!GfIsClose(GfVec3f(movedWith[i] - movedWithout[i]),
                       GfVec3f(0, 3, 0), 1e-5)) {
            ++failures;
            std::printf("FAIL an intervening Xform: moved point %zu did not "
                        "follow the grouping transform (%g %g %g against "
                        "%g %g %g); the asymmetry the refusal stands in "
                        "front of has changed\n",
                        i, movedWith[i][0], movedWith[i][1], movedWith[i][2],
                        movedWithout[i][0], movedWithout[i][1],
                        movedWithout[i][2]);
            break;
        }
    }
}

static std::string
_SchemaResourceDir(const std::string &examplesDir)
{
#ifdef RIGEXEC_SCHEMA_RESOURCE_DIR
    (void)examplesDir;
    return TfAbsPath(RIGEXEC_SCHEMA_RESOURCE_DIR);
#else
    return TfAbsPath(examplesDir + "/../plugin/rigExecSchema/resources");
#endif
}

int
main(int argc, char **argv)
{
    if (argc < 2) {
        std::printf("usage: testRigExecBakedMode <examplesDir>\n");
        return 2;
    }
    const std::string examplesDir = argv[1];
    const std::string resources = _SchemaResourceDir(examplesDir);
    if (PlugRegistry::GetInstance().RegisterPlugins(resources).empty()) {
        std::printf("FATAL: no schema plugin found at %s\n",
                    resources.c_str());
        return 2;
    }

    for (const char *stageName : {"biped/Biped.usda",
                                  "biped/Biped_layered.usda",
                                  "biped/Biped_anim.usda",
                                  // A matrix-mover rig with keyed avars: the
                                  // biped's chains are all skin, so without
                                  // this the second geometry operation the
                                  // program expresses is never compared.
                                  "simple_rig_anim.usd"}) {
        TestModeIsExact(examplesDir, stageName);
        TestParityModeReportsNoMismatch(examplesDir, stageName);
    }
    TestDynamicModeIsTheDefault(examplesDir);
    TestAnEditAfterTheBakeIsFollowed(examplesDir);
    TestAnInteractiveOverrideAfterTheBakeIsFollowed(examplesDir);
    TestAnOverrideOnAConstraintWeightIsFollowed(examplesDir);
    TestAnUnplaceableOverrideFallsBack(examplesDir);
    TestAnArrayOverrideOnACurvenetWeightFallsBack();

    // Invalidation, both directions.
    TestEditAfterTheBake(examplesDir, "biped/Biped.usda",
                         "a keyed blend weight, a disabled constraint and a "
                         "moved rest",
                         EditKeyedBlendDisabledConstraintAndRest,
                         /* expectRebuild = */ true);
    TestEditAfterTheBake(examplesDir, "biped/Biped.usda",
                         "a moved foot roll", EditFootRollValue,
                         /* expectRebuild = */ false);
    TestEditBeforeTheBake(examplesDir, "biped/Biped.usda",
                          "a keyed foot roll", EditKeyedFootRoll);
    TestEditAfterTheBake(examplesDir, "biped/Biped_anim.usda",
                         "a keyed avar value", EditAKeyedAvarValue,
                         /* expectRebuild = */ false);
    TestAnUnrelatedEditLeavesTheProgramStanding(examplesDir);

    // A value keyed exactly once -- the shape USD reports as not
    // time-varying while a Default read still cannot see it.
    TestEditBeforeTheBake(examplesDir, "biped/Biped.usda",
                          "one key on a control avar",
                          EditOneKeyOnAControlAvar);
    TestEditBeforeTheBake(examplesDir, "biped/Biped.usda",
                          "one key on every constraint's enable",
                          EditOneKeyOnEveryConstraintEnable);
    // A partial constant envelope on a skin mover: the case where the two
    // geometry loops' "skip the blend" predicates could disagree.
    TestEditBeforeTheBake(examplesDir, "biped/Biped_anim.usda",
                          "a half-strength skin envelope",
                          EditHalfStrengthSkinEnvelope);
    // An override standing on an attribute several hops up an input's
    // connection chain.
    TestAnOverrideOnAConnectionSourceIsFollowed(examplesDir);
    // Guides disabled by the consumer: neither path publishes them.
    TestParityModeWithGuidesDisabled(examplesDir, "biped/Biped_anim.usda");
    // The comparator the parity mode is made of, in the positive direction.
    TestTheParityComparatorFindsWhatIsThere();
    TestTheParityComparatorIsExact(examplesDir);

    // The mode turned on while the epoch was dirty, and a program rebuilt
    // inside its epoch publishing the same counters as the dynamic path.
    TestBakedModeRequestedOnADirtyEpoch(examplesDir);
    TestAnInEpochRebuildPublishesTheSameCounters(examplesDir);

    // And a rig the program refuses -- one whose refusal survives every
    // feature group, because it is an exec answer computed from the middle
    // of the pose walk and not a kernel waiting to be hoisted.
    TestANonBakeableRigFallsBack("a connected posed:space",
                                 "connected posed:space");

    // A read phase naming a point in the pose walk, which no shipped rig
    // authors and no other suite builds.
    TestAReadPhaseOnTheTransformIsExact();
    // The second deliberate negative: a volume weight a constraint moves,
    // which the DYNAMIC path gives two placements at once.
    TestAConstrainedVolumeWeightFallsBack();
    // And the two the rest suite cannot hold under its bake requirement:
    // a grouping Xform between a provider and its anchor, authored and
    // animated, plus the measurement of the dynamic-path asymmetry both
    // refusals stand on.
    TestAnInterveningXformAboveAProvider();
    TestAnAnimatedXformAboveAProvider();
    TestAnInterveningXformMovesTheMeshAndNotTheJointMatrix();

    // And everything in examples/, whether it bakes or not.
    TestEveryExampleStage(examplesDir);

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecBakedMode: all tests passed\n");
    return 0;
}
