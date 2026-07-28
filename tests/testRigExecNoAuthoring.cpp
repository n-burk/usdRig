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
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"

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

}  // namespace

int
main(int argc, char **argv)
{
    if (argc < 2) {
        std::printf("usage: testRigExecNoAuthoring <examplesDir>\n");
        return 2;
    }
    const std::string examplesDir = argv[1];

    const std::string resources =
        TfAbsPath(examplesDir + "/../plugin/rigExecSchema/resources");
    if (PlugRegistry::GetInstance().RegisterPlugins(resources).empty()) {
        std::printf("FATAL: no schema plugin found at %s\n", resources.c_str());
        return 2;
    }

    TestEvaluatorAuthorsNothing(examplesDir);
    TestImagingPublicationAuthorsNothing(examplesDir);

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecNoAuthoring: all tests passed\n");
    return 0;
}
