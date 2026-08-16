//
// Proves the engine's central invariant: evaluation and Hydra publication
// author NOTHING to the source stage (spec §7.2 revised).
//
// The existing assertions elsewhere probe for specific known artifacts -- the
// absence of a __RigExecGenerated prim, a session layer with no sublayers.
// Those only catch the mechanism that used to exist. This instead captures the
// whole composed scene, runs everything, and demands byte equality, so it
// catches any authoring at all, including a mechanism nobody has invented yet.
//
// argv[1] = path to the examples directory. The codeless schema plugin is
// expected at <examples>/../plugin/rigExecSchema/resources.
//
#include "rigExec/rigEvaluator.h"
#include "rigExecImaging/registry.h"

#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

using namespace rigExec;

static int failures = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            ++failures;                                                    \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
        }                                                                  \
    } while (0)

namespace {

// Everything about the stage that authoring could possibly disturb.
struct StageSnapshot {
    std::string flattened;      ///< the entire composed scene
    std::string rootLayer;
    std::string sessionLayer;
    bool rootDirty = false;
    bool sessionDirty = false;
    size_t usedLayers = 0;
    size_t primCount = 0;
    size_t propertyCount = 0;
};

StageSnapshot
Capture(const UsdStageRefPtr &stage)
{
    StageSnapshot s;
    // addSourceFileComment=false: the comment carries a path/timestamp that
    // would differ between captures for reasons having nothing to do with us.
    stage->ExportToString(&s.flattened, /* addSourceFileComment */ false);
    stage->GetRootLayer()->ExportToString(&s.rootLayer);
    if (stage->GetSessionLayer()) {
        stage->GetSessionLayer()->ExportToString(&s.sessionLayer);
        s.sessionDirty = stage->GetSessionLayer()->IsDirty();
    }
    s.rootDirty = stage->GetRootLayer()->IsDirty();
    s.usedLayers = stage->GetUsedLayers().size();
    for (const UsdPrim &prim : stage->TraverseAll()) {
        ++s.primCount;
        s.propertyCount += prim.GetPropertyNames().size();
    }
    return s;
}

void
Compare(const char *what, const StageSnapshot &a, const StageSnapshot &b)
{
    // Report the cheap structural fields individually -- a diff in one of them
    // localises the problem far faster than "the flattened scene changed".
    if (a.primCount != b.primCount) {
        ++failures;
        std::printf("FAIL %s: prim count %zu -> %zu\n",
                    what, a.primCount, b.primCount);
    }
    if (a.propertyCount != b.propertyCount) {
        ++failures;
        std::printf("FAIL %s: property count %zu -> %zu\n",
                    what, a.propertyCount, b.propertyCount);
    }
    if (a.usedLayers != b.usedLayers) {
        ++failures;
        std::printf("FAIL %s: used layer count %zu -> %zu\n",
                    what, a.usedLayers, b.usedLayers);
    }
    if (b.rootDirty && !a.rootDirty) {
        ++failures;
        std::printf("FAIL %s: root layer became dirty\n", what);
    }
    if (b.sessionDirty && !a.sessionDirty) {
        ++failures;
        std::printf("FAIL %s: session layer became dirty\n", what);
    }
    if (a.rootLayer != b.rootLayer) {
        ++failures;
        std::printf("FAIL %s: root layer contents changed\n", what);
    }
    if (a.sessionLayer != b.sessionLayer) {
        ++failures;
        std::printf("FAIL %s: session layer contents changed "
                    "(%zu -> %zu bytes)\n",
                    what, a.sessionLayer.size(), b.sessionLayer.size());
    }
    // The catch-all: any authoring anywhere in the layer stack lands here.
    if (a.flattened != b.flattened) {
        ++failures;
        std::printf("FAIL %s: composed scene changed (%zu -> %zu bytes)\n",
                    what, a.flattened.size(), b.flattened.size());
    }
}

const std::vector<UsdTimeCode> &
Frames()
{
    // Default plus the animated range of ArmShotAnim, so time-varying
    // resolution, recompilation and republication all get exercised.
    static const std::vector<UsdTimeCode> frames = {
        UsdTimeCode::Default(), UsdTimeCode(1001.0), UsdTimeCode(1012.0),
        UsdTimeCode(1024.0),    UsdTimeCode(1036.0), UsdTimeCode(1048.0)};
    return frames;
}

// The evaluator on its own: compile, then evaluate every frame.
void
TestEvaluatorAuthorsNothing(const std::string &examplesDir)
{
    const UsdStageRefPtr stage =
        UsdStage::Open(examplesDir + "/ArmShotAnim.usda");
    CHECK(stage);
    if (!stage) {
        return;
    }
    const SdfPath rigPath("/Shot/HeroArm/Rig");
    const StageSnapshot before = Capture(stage);

    RigExecRigEvaluator evaluator(stage, rigPath);
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    for (const std::string &e : errors) {
        std::printf("  compile error: %s\n", e.c_str());
    }

    // A compile alone must not author -- check before evaluating, so a
    // failure here is not confused with one from evaluation.
    Compare("after Compile", before, Capture(stage));

    for (const UsdTimeCode &t : Frames()) {
        const RigExecRigPose pose = evaluator.Evaluate(t);
        CHECK(pose.valid);
    }
    Compare("after Evaluate x6", before, Capture(stage));

    // The evaluation stage must BE the source stage, not a derived copy.
    CHECK(evaluator.GetEvaluationStage() == stage);
}

// The Hydra path: activation, per-frame publication, teardown.
void
TestImagingPublicationAuthorsNothing(const std::string &examplesDir)
{
    const UsdStageRefPtr stage =
        UsdStage::Open(examplesDir + "/ArmShotAnim.usda");
    CHECK(stage);
    if (!stage) {
        return;
    }
    const SdfPath rigPath("/Shot/HeroArm/Rig");
    const StageSnapshot before = Capture(stage);

    RigExecImagingRegistry &registry = RigExecImagingRegistry::GetInstance();
    std::vector<std::string> errors;
    const bool activated =
        registry.Activate(stage, rigPath, UsdTimeCode(1001.0), &errors);
    CHECK(activated);
    for (const std::string &e : errors) {
        std::printf("  activate error: %s\n", e.c_str());
    }
    Compare("after imaging Activate", before, Capture(stage));

    if (activated) {
        for (const UsdTimeCode &t : Frames()) {
            registry.SetTime(t);
        }
        CHECK(registry.GetStore() != nullptr);
        Compare("after imaging SetTime x6", before, Capture(stage));
    }

    registry.Deactivate();
    Compare("after imaging Deactivate", before, Capture(stage));
}

// EVERY example, not one.
//
// ArmShotAnim above is the deepest rig, but depth is not the axis that
// matters here: a write-back is introduced by a KIND of output, and the
// examples are exactly the per-kind fixtures. The property-domain movers are
// the case that motivated this -- a mover that computes a float, a vector, or
// a matrix for an attribute that already exists on the stage is one careless
// `attr.Set()` away from becoming an authoring engine, and nothing about the
// point-chain fixtures would notice.
void
TestEveryExampleAuthorsNothing(const std::string &examplesDir)
{
    // Rig paths discovered by type, so a new example cannot silently skip.
    const char *files[] = {
        "/01_FkChainTail.usda",   "/02_TwoBoneIkLeg.usda",
        "/03_IkFkBlendClamp.usda", "/04_BlendShapeFace.usda",
        "/05_TwistRibbonSpine.usda", "/06_LatticeBulge.usda",
        "/07_SurfaceDrape.usda",  "/08_AimEyes.usda",
        "/09_PropertyMathMovers.usda", "/10_AimXformTurret.usda",
        "/11_VolumeWeights.usda", "/12_CurvenetProfile.usda",
        "/13_ReadPhases.usda",
        "/ArmRig.usda",           "/rigexec_flat.usda",
    };

    for (const char *file : files) {
        const UsdStageRefPtr stage = UsdStage::Open(examplesDir + file);
        CHECK(stage);
        if (!stage) {
            continue;
        }
        SdfPath rigPath;
        for (const UsdPrim &p : stage->Traverse()) {
            if (p.GetTypeName() == TfToken("RigExecRoot")) {
                rigPath = p.GetPath();
                break;
            }
        }
        CHECK(!rigPath.IsEmpty());
        if (rigPath.IsEmpty()) {
            continue;
        }

        const StageSnapshot before = Capture(stage);
        RigExecRigEvaluator evaluator(stage, rigPath);
        std::vector<std::string> errors;
        const bool compiled = evaluator.Compile(&errors);
        CHECK(compiled);
        if (!compiled) {
            for (const std::string &e : errors) {
                std::printf("  %s compile error: %s\n", file, e.c_str());
            }
            continue;
        }
        Compare((std::string(file) + " after Compile").c_str(), before,
                Capture(stage));

        // Default plus each file's own animated range, so a stage whose
        // frames differ from ArmShotAnim's is still exercised where it moves.
        std::vector<UsdTimeCode> times = {UsdTimeCode::Default()};
        if (stage->HasAuthoredTimeCodeRange()) {
            const double start = stage->GetStartTimeCode();
            const double end = stage->GetEndTimeCode();
            times.push_back(UsdTimeCode(start));
            times.push_back(UsdTimeCode((start + end) * 0.5));
            times.push_back(UsdTimeCode(end));
        }
        for (const UsdTimeCode &t : times) {
            const RigExecRigPose pose = evaluator.Evaluate(t);
            CHECK(pose.valid);
        }
        Compare((std::string(file) + " after Evaluate").c_str(), before,
                Capture(stage));
        CHECK(evaluator.GetEvaluationStage() == stage);
    }
}

// The specific temptation the property movers create: a mover's result is a
// value OF THE SAME TYPE as the attribute it targets, so writing it back is
// one line and would look correct from every consumer's side.
//
// Assert against the attribute itself rather than the whole scene, so the
// failure names the property instead of "the composed scene changed": the
// authored opinion must survive an evaluation that published a different
// value, at Default AND at an animated time.
void
TestPropertyMoverDoesNotWriteBack(const std::string &examplesDir)
{
    const UsdStageRefPtr stage =
        UsdStage::Open(examplesDir + "/09_PropertyMathMovers.usda");
    CHECK(stage);
    if (!stage) {
        return;
    }
    const SdfPath gainPath("/PropMathAsset/Rig/Channels/Dials.rigExec:gain");
    const UsdAttribute gain = stage->GetAttributeAtPath(gainPath);
    CHECK(gain);
    if (!gain) {
        return;
    }

    RigExecRigEvaluator evaluator(stage, SdfPath("/PropMathAsset/Rig"));
    CHECK(evaluator.Compile(nullptr));

    for (const UsdTimeCode &t :
         {UsdTimeCode::Default(), UsdTimeCode(1001.0), UsdTimeCode(1024.0)}) {
        const RigExecRigPose pose = evaluator.Evaluate(t);
        CHECK(pose.valid);

        // The rig published the clamped value...
        const auto it = pose.movedProperties.find(gainPath);
        CHECK(it != pose.movedProperties.end());
        if (it != pose.movedProperties.end()) {
            CHECK(std::abs(it->second.Get<float>() - 1.0f) < 1e-6f);
        }

        // ...and the stage still holds the author's 2.5.
        float authored = 0;
        CHECK(gain.Get(&authored, t));
        if (std::abs(authored - 2.5f) > 1e-6f) {
            std::printf("  rigExec:gain is %f on the stage; evaluation wrote "
                        "its result back\n", double(authored));
        }
        CHECK(std::abs(authored - 2.5f) < 1e-6f);
        // No opinion appeared anywhere, in any layer.
        CHECK(!gain.GetNumTimeSamples());
        CHECK(gain.GetPropertyStack(t).size() == 1);
    }

    // Same for the exec-overridden weight in 03: the override is an
    // evaluation-time value, not an authored one.
    const UsdStageRefPtr blend =
        UsdStage::Open(examplesDir + "/03_IkFkBlendClamp.usda");
    CHECK(blend);
    if (!blend) {
        return;
    }
    const UsdAttribute weight = blend->GetAttributeAtPath(
        SdfPath("/BlendArmAsset/Rig/Solvers/IKFKBlend.inputs:weight"));
    CHECK(weight);
    if (!weight) {
        return;
    }
    const size_t stackBefore = weight.GetPropertyStack(UsdTimeCode(1001)).size();
    RigExecRigEvaluator blendEval(blend, SdfPath("/BlendArmAsset/Rig"));
    CHECK(blendEval.Compile(nullptr));
    for (double t : {1001.0, 1024.0, 1030.0}) {
        CHECK(blendEval.Evaluate(UsdTimeCode(t)).valid);
    }
    // The overdriven authored value is untouched: -0.25 at 1001, NOT the 0
    // the clamp published and handed to the blend.
    float authoredWeight = 99;
    CHECK(weight.Get(&authoredWeight, UsdTimeCode(1001)));
    if (std::abs(authoredWeight + 0.25f) > 1e-6f) {
        std::printf("  inputs:weight is %f at 1001; the exec override leaked "
                    "into the stage\n", double(authoredWeight));
    }
    CHECK(std::abs(authoredWeight + 0.25f) < 1e-6f);
    CHECK(weight.GetPropertyStack(UsdTimeCode(1001)).size() == stackBefore);
}

}  // namespace

// The codeless schema's resource directory.
//
// The GENERATED one when the build supplied it: only that copy carries the
// LibraryPath that lets Plug load the compute-extent registration on demand,
// which is what makes UsdGeomBBoxCache answer for RigExec prims. The source
// tree's copy is data-only and is the fallback for an ad hoc build.
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
        std::printf("usage: testRigExecNoAuthoring <examplesDir>\n");
        return 2;
    }
    const std::string examplesDir = argv[1];

    const std::string resources =
        _SchemaResourceDir(examplesDir);
    if (PlugRegistry::GetInstance().RegisterPlugins(resources).empty()) {
        std::printf("FATAL: no schema plugin found at %s\n", resources.c_str());
        return 2;
    }

    TestEvaluatorAuthorsNothing(examplesDir);
    TestEveryExampleAuthorsNothing(examplesDir);
    TestPropertyMoverDoesNotWriteBack(examplesDir);
    TestImagingPublicationAuthorsNothing(examplesDir);

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecNoAuthoring: all tests passed\n");
    return 0;
}
