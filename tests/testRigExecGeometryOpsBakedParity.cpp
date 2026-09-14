//
// The geometry operators that bake with no shipped rig to say so.
//
// blendShape, curvenet, emitGuidePoints, ribbon and every read phase but the
// default one bake, and not one of them is reachable from an example that
// bakes: 04, 12 and 13 bind a weight object, 05 and ArmRig drive their curve
// movers from a RigExecRibbon, and neither refusal is this group's to
// remove. Left at that, five operations would ship with their only evidence
// in somebody's scratch directory, and a regression in any of them would sit
// invisible until another group's work happened to uncover it.
//
// So this suite takes the shipped rigs and removes the OTHER group's blocker
// in memory, on a session layer: a face whose weight object is unbound is
// still the same blend shape over the same channels and in-between samples,
// and a curvenet with no envelope is still the same net. Nothing here waits
// on anyone. When the weights and solvers groups land, the example entries
// pick up the weighted and ribbon-driven halves and these cases keep holding
// the unweighted ones.
//
// The claim, per case, is the one testRigExecExampleParity makes: every
// frame agrees with the dynamic path exactly, and every generation CAME FROM
// THE PROGRAM -- a rig that quietly declined would otherwise compare the
// dynamic path with itself and pass having proved nothing.
//
// argv[1] = the examples directory, argv[2] = the ribbon probe layer.
//
#include "rigExec/rigEvaluator.h"
#include "rigExec/types.h"

#include "pxr/base/plug/registry.h"
#include "pxr/base/vt/value.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"

#include <cstdio>
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

namespace {

SdfPath
FindRig(const UsdStageRefPtr &stage)
{
    for (const UsdPrim &prim : stage->Traverse()) {
        if (prim.GetTypeName() == "RigExecRoot") {
            return prim.GetPath();
        }
    }
    return SdfPath();
}

// Unbind every weight object in the rig, on the session layer, so the file
// on disk is untouched and the rig that remains is the one the shipped
// example authors minus an envelope this group does not own. An explicit
// empty target list is what blocks the weaker opinion; clearing the
// authoring would only expose it again.
void
UnbindWeightObjects(const UsdStageRefPtr &stage)
{
    stage->SetEditTarget(UsdEditTarget(stage->GetSessionLayer()));
    static const TfToken weightObject("rigExec:weightObject");
    size_t unbound = 0;
    for (const UsdPrim &prim : stage->Traverse()) {
        const UsdRelationship relationship =
            prim.GetRelationship(weightObject);
        SdfPathVector targets;
        if (relationship && relationship.GetTargets(&targets) &&
            !targets.empty()) {
            relationship.SetTargets({});
            ++unbound;
        }
    }
    // A rig whose weight objects moved out from under this helper would
    // still bake, and the case would silently stop being the one it says it
    // is -- so say how many were found rather than assume.
    CHECK(unbound > 0);
}

// One rig across its frames, both paths, compared. Returns what was
// published on \p watch at each frame, so the caller can go on to prove the
// operation moved something rather than passing on two identical copies of
// the rest pose.
std::vector<VtValue>
RunParity(const std::string &where, const UsdStageRefPtr &stage,
          const std::vector<UsdTimeCode> &frames, const SdfPath &watch,
          RigExecRigEvaluator **keep = nullptr)
{
    std::vector<VtValue> published;
    const SdfPath rigPath = FindRig(stage);
    CHECK(!rigPath.IsEmpty());
    if (rigPath.IsEmpty()) {
        return published;
    }
    // Leaked deliberately when the caller asks to keep it: a drag has to run
    // on the evaluator that already holds the program, and this is a test
    // binary that exits straight after.
    RigExecRigEvaluator *rig = new RigExecRigEvaluator(stage, rigPath);
    // The guides are a compared domain, and the example entries ask for
    // them; a suite that left them off would measure less than its siblings.
    rig->SetSolverGuidesEnabled(true);
    rig->SetEvaluationMode(RigExecEvaluationMode::BakedWithParityCheck);
    std::vector<std::string> errors;
    if (!rig->Compile(&errors)) {
        ++failures;
        std::printf("FAIL %s: does not compile\n", where.c_str());
        for (const std::string &error : errors) {
            std::printf("    %s\n", error.c_str());
        }
        return published;
    }
    std::vector<std::string> reasons;
    if (!rig->IsBakeable(&reasons)) {
        ++failures;
        std::printf("FAIL %s: declined the bake\n", where.c_str());
        for (const std::string &reason : reasons) {
            std::printf("    %s\n", reason.c_str());
        }
        return published;
    }
    for (const UsdTimeCode frame : frames) {
        const RigExecRigPose pose = rig->Evaluate(frame);
        CHECK(pose.valid);
        if (pose.bakedParityMismatches) {
            ++failures;
            std::printf("FAIL %s: %zu baked parity mismatch(es) at %g\n",
                        where.c_str(), pose.bakedParityMismatches,
                        frame.GetValue());
            for (const std::string &diagnostic : pose.diagnostics) {
                std::printf("    %s\n", diagnostic.c_str());
            }
        }
        const auto moved = pose.movedProperties.find(watch);
        if (moved == pose.movedProperties.end()) {
            ++failures;
            std::printf("FAIL %s: nothing published on %s at %g\n",
                        where.c_str(), watch.GetText(), frame.GetValue());
        } else {
            published.push_back(moved->second);
        }
    }
    if (rig->GetBakedGenerationCount() != frames.size()) {
        ++failures;
        std::printf("FAIL %s: %zu of %zu generation(s) came from the "
                    "program\n", where.c_str(), rig->GetBakedGenerationCount(),
                    frames.size());
    }
    std::printf("  %-42s %2zu frame(s) from the program\n", where.c_str(),
                frames.size());
    if (keep) {
        *keep = rig;
    } else {
        delete rig;
    }
    return published;
}

// The operation has to have DONE something: an op that silently published
// its input would agree with a dynamic path that did the same, and every
// comparison above would pass.
void
CheckMoves(const std::string &where, const std::vector<VtValue> &published)
{
    if (published.size() < 2) {
        ++failures;
        std::printf("FAIL %s: %zu published value(s)\n", where.c_str(),
                    published.size());
        return;
    }
    for (size_t i = 1; i < published.size(); ++i) {
        if (published[i] != published.front()) {
            return;
        }
    }
    ++failures;
    std::printf("FAIL %s: every frame published the same value; the "
                "operation is a pass-through here\n", where.c_str());
}

// One drag, held across the frames, asserting the override PLACED -- an
// override the program cannot reach falls back to the dynamic path, answers
// identically, and shows up nowhere except in this count. Each frame twice,
// because SetInteractiveOverrides invalidates the static input cache on the
// way in and a cache that refilled mid-drag only gets its chance on a second
// pass.
void
DragOne(const std::string &where, RigExecRigEvaluator *rig,
        const std::vector<RigExecValueOverride> &overrides,
        const std::vector<UsdTimeCode> &frames)
{
    const size_t before = rig->GetBakedGenerationCount();
    rig->SetInteractiveOverrides(overrides);
    size_t generations = 0;
    for (const UsdTimeCode frame : frames) {
        for (int pass = 0; pass < 2; ++pass) {
            const RigExecRigPose pose = rig->Evaluate(frame);
            CHECK(pose.valid);
            if (pose.bakedParityMismatches) {
                ++failures;
                std::printf("FAIL %s: %zu baked parity mismatch(es) at %g\n",
                            where.c_str(), pose.bakedParityMismatches,
                            frame.GetValue());
            }
            ++generations;
        }
    }
    if (rig->GetBakedGenerationCount() - before != generations) {
        ++failures;
        std::printf("FAIL %s: %zu of %zu generation(s) came from the "
                    "program; the drag was not placed\n", where.c_str(),
                    rig->GetBakedGenerationCount() - before, generations);
    }
    rig->ClearInteractiveOverrides();
    std::printf("  %-42s %2zu generation(s) under a drag\n", where.c_str(),
                generations);
}

const std::vector<UsdTimeCode> &
ExampleFrames()
{
    // The frames tests/exampleFixtures.cmake gives 04, 12 and 13, so a case
    // here and the example entry that will replace it measure one thing.
    static const std::vector<UsdTimeCode> frames{
        UsdTimeCode(1001), UsdTimeCode(1012), UsdTimeCode(1024),
        UsdTimeCode(1036), UsdTimeCode(1048)};
    return frames;
}

// blendShape: two channels, one of them carrying an in-between sample, and
// the two interactive inputs that reach them -- a channel's weight and a
// sample's activation, which place through resolvedRoutedPrims rather than
// through a bound value.
void
TestBlendShape(const std::string &examplesDir)
{
    const auto stage =
        UsdStage::Open(examplesDir + "/04_BlendShapeFace.usda");
    CHECK(stage);
    if (!stage) {
        return;
    }
    UnbindWeightObjects(stage);
    const SdfPath card("/FaceAsset/Geom/FaceCard.points");
    RigExecRigEvaluator *rig = nullptr;
    CheckMoves("04 blendShape",
               RunParity("04 blendShape", stage, ExampleFrames(), card, &rig));
    if (!rig) {
        return;
    }
    DragOne("04 blendShape (channel weight)", rig,
            {RigExecValueOverride{SdfPath("/FaceAsset/Rig/BlendInputs/Smile"),
                                  TfToken(), TfToken("inputs:weight"),
                                  VtValue(0.75f)}},
            ExampleFrames());
    DragOne("04 blendShape (sample activation)", rig,
            {RigExecValueOverride{
                SdfPath("/FaceAsset/Rig/BlendInputs/Smile/Half"), TfToken(),
                TfToken("rigExec:activation"), VtValue(0.35f)}},
            ExampleFrames());
    delete rig;
}

// curvenet: the profile mover, its program-owned bind cache and the
// cross-chain edge from the posed net's own chain. The bind's diagnostic is
// drained once per (re)bind, so the FIRST generation is the only one that
// can show a cache shared between the two paths -- and RunParity compares
// the diagnostics of exactly that generation.
void
TestCurvenet(const std::string &examplesDir)
{
    const auto stage =
        UsdStage::Open(examplesDir + "/12_CurvenetProfile.usda");
    CHECK(stage);
    if (!stage) {
        return;
    }
    UnbindWeightObjects(stage);
    const SdfPath tube("/CurvenetAsset/Geom/Tube.points");
    CheckMoves("12 curvenet",
               RunParity("12 curvenet", stage, ExampleFrames(), tube));
}

// Read phases, all three forms a geometry input can name, on the lattice
// whose cage two movers rewrite. `base` reads the authored cage and takes no
// snapshot; `final` and the mid-walk prim both come out of the run-local
// snapshot store, which is the half no shipped bakeable rig reaches -- and
// the three answers must DIFFER, or the store is feeding one value under
// three names.
void
TestReadPhases(const std::string &examplesDir)
{
    static const TfToken readPhase("rigExecReadPhase");
    static const TfToken cage("rigExec:cage");
    const SdfPath lattice("/ReadPhaseAsset/Rig/Movers/Geometry/SlabLattice");
    const SdfPath slab("/ReadPhaseAsset/Geom/Slab.points");
    std::vector<std::vector<VtValue>> answers;
    for (const char *phase : {"base", "final",
                              "/ReadPhaseAsset/Rig/Movers/Cage/CageLift"}) {
        // A fresh stage per phase: the phase is compiled into the chain, and
        // re-authoring it under a live evaluator would test the rebuild
        // rather than the phase.
        const auto stage =
            UsdStage::Open(examplesDir + "/13_ReadPhases.usda");
        CHECK(stage);
        if (!stage) {
            return;
        }
        UnbindWeightObjects(stage);
        const UsdRelationship input =
            stage->GetPrimAtPath(lattice).GetRelationship(cage);
        CHECK(input && input.SetMetadata(readPhase,
                                         VtValue(std::string(phase))));
        const std::string where =
            std::string("13 read phase ") + phase;
        answers.push_back(
            RunParity(where, stage, ExampleFrames(), slab));
        // Not "base": the authored cage is what it reads, and the authored
        // cage does not animate -- a slab that stood still under it would be
        // this example working, not a pass-through. The other two read a
        // cage two animated movers rewrote, so they have to move.
        if (std::string(phase) != "base") {
            CheckMoves(where, answers.back());
        }
    }
    CHECK(answers.size() == 3);
    if (answers.size() != 3 || answers[0].empty()) {
        return;
    }
    // Pairwise, because "final" agreeing with "base" and the mid-walk point
    // agreeing with "final" are two different failures: the first says no
    // snapshot was taken, the second says the walk's record was taken at the
    // wrong point.
    CHECK(answers[0] != answers[1]);
    CHECK(answers[1] != answers[2]);
    CHECK(answers[0] != answers[2]);
}

// emitGuidePoints and ribbon, off a solver that bakes. The probe layer is
// the only thing that reaches them until RigExecRibbon bakes; see its own
// header for why it sublayers the animated biped.
void
TestCurveModes(const std::string &probeLayer)
{
    const auto stage = UsdStage::Open(probeLayer);
    CHECK(stage);
    if (!stage) {
        return;
    }
    const std::vector<UsdTimeCode> frames{
        UsdTimeCode(1), UsdTimeCode(2), UsdTimeCode(3), UsdTimeCode(4),
        UsdTimeCode(5), UsdTimeCode(6), UsdTimeCode(7), UsdTimeCode(8)};
    CheckMoves("probe emitGuidePoints",
               RunParity("probe emitGuidePoints", stage, frames,
                         SdfPath("/Biped/SpineGuides.points")));
    CheckMoves("probe ribbon",
               RunParity("probe ribbon", stage, frames,
                         SdfPath("/Biped/SpineStrip.points")));
}

}  // namespace

int
main(int argc, char **argv)
{
    if (argc < 3) {
        std::printf("usage: testRigExecGeometryOpsBakedParity <examplesDir> "
                    "<ribbonProbe.usda>\n");
        return 1;
    }
    PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);
    const std::string examplesDir = argv[1];
    TestBlendShape(examplesDir);
    TestCurvenet(examplesDir);
    TestReadPhases(examplesDir);
    TestCurveModes(argv[2]);
    std::printf("testRigExecGeometryOpsBakedParity: %d failure(s)\n", failures);
    return failures ? 1 : 0;
}
