#include "rigExecImaging/fallbackStage.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/weakBase.h"
#include "pxr/base/ts/knot.h"
#include "pxr/base/ts/spline.h"
#include "pxr/usd/sdf/changeBlock.h"
#include "pxr/usd/usdGeom/points.h"
#include "pxr/usd/usdGeom/xform.h"
#include <future>
#include <iostream>
#include <stdexcept>
#include <thread>

using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE
#define CHECK(x) do { if (!(x)) throw std::runtime_error(#x); } while (false)

namespace {
const SdfPath kRig("/Rig"), kControl("/Rig/C"), kPoints("/Rig/Cloud.points");
using Snapshot = std::shared_ptr<const RigExecFallbackStageSnapshot>;

struct Capture : TfWeakBase {
    RigExecFallbackStageSource source;
    std::thread::id owner = std::this_thread::get_id();
    TfNotice::Key key;
    explicit Capture(const UsdStageRefPtr &stage) : source(stage) {
        key = TfNotice::Register(TfCreateWeakPtr(this), &Capture::Changed, stage);
    }
    ~Capture() { TfNotice::Revoke(key); }
    void Changed(const UsdNotice::ObjectsChanged &notice, const UsdStageWeakPtr &stage) {
        CHECK(std::this_thread::get_id() == owner);
        CHECK(source.CaptureChanges(UsdStageRefPtr(stage), notice));
    }
};

UsdStageRefPtr MakeStage() {
    const SdfLayerRefPtr weaker = SdfLayer::CreateAnonymous("weaker.usda");
    CHECK(weaker->ImportFromString(R"USD(#usda 1.0
 def RigExecRoot "Rig" {
  def RigExecControl "C" {
   double avars:tx = 1
   double avars:ty.timeSamples = {1: 2, 3: 6}
  }
  def Scope "Settings" {
   double custom:gain = 4
  }
  def RigExecBoneFrame "Provider" {
   double inputs:tx.connect = </Rig/Settings.custom:gain>
   rel rigExec:sourceObject = </Rig/C>
  }
  def RigExecJoint "J" {
   matrix4d posed:space.connect = </Rig/Provider.outputs:matrix>
  }
  def Points "Cloud" {
   point3f[] points = [(0,0,0),(1,2,3)]
  }
  def RigExecMatrixMover "Move" (prepend apiSchemas = ["RigExecMoverAPI"]) {
   rel rigExec:moves = </Rig/Cloud.points>
   rel rigExec:transform = </Rig/C>
  }
 }
)USD"));
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->GetRootLayer()->SetSubLayerPaths({weaker->GetIdentifier()});
    // The source stage's layer stack retains its weaker authored layer.
    CHECK(stage->GetPrimAtPath(kControl));
    return stage;
}

std::string Content(const SdfLayerRefPtr &layer) {
    std::string text; CHECK(layer->ExportToString(&text)); return text;
}

RigExecRigPose Live(const UsdStageRefPtr &stage, UsdTimeCode time) {
    const std::string before = Content(stage->GetRootLayer());
    RigExecRigEvaluator evaluator(stage, kRig);
    std::vector<std::string> diagnostics;
    if (!evaluator.Compile(&diagnostics)) {
        for (const auto &message : diagnostics) std::cerr << message << '\n';
        CHECK(false);
    }
    RigExecRigPose pose = evaluator.Evaluate(time);
    CHECK(pose.valid);
    CHECK(Content(stage->GetRootLayer()) == before);
    return pose;
}

void Same(const RigExecRigPose &a, const RigExecRigPose &b) {
    CHECK(a.valid && b.valid);
    CHECK(a.controlFrames.size() == b.controlFrames.size());
    for (const auto &[path, frame] : a.controlFrames) {
        const auto &other = b.controlFrames.at(path);
        for (size_t i = 0; i < 4; ++i) CHECK(GfIsClose(frame.points[i], other.points[i], 1e-10));
    }
    CHECK(a.jointFramesFinal.size() == b.jointFramesFinal.size());
    for (const auto &[path, frame] : a.jointFramesFinal) {
        const auto &other = b.jointFramesFinal.at(path);
        for (size_t i = 0; i < 4; ++i) CHECK(GfIsClose(frame.points[i], other.points[i], 1e-10));
    }
    CHECK(a.movedProperties == b.movedProperties);
    CHECK(a.movedProperties.count(kPoints));
}

RigExecRigPose Worker(RigExecFallbackStageMirror &mirror, const Snapshot &snapshot,
                      UsdTimeCode time, const RigExecFallbackStageOptions &options = {}) {
    const std::thread::id owner = std::this_thread::get_id();
    return std::async(std::launch::async, [&mirror, snapshot, time, owner, options] {
        CHECK(std::this_thread::get_id() != owner);
        RigExecRigPose pose; CHECK(mirror.Evaluate(snapshot, time, &pose, options)); return pose;
    }).get();
}

void CheckCurrent(const UsdStageRefPtr &stage, RigExecFallbackStageMirror &mirror,
                  const Snapshot &snapshot, UsdTimeCode time) {
    const RigExecRigPose expected = Live(stage, time);
    std::vector<std::pair<SdfLayerRefPtr, std::string>> contents;
    for (const SdfLayerHandle &layer : stage->GetLayerStack()) {
        const SdfLayerRefPtr retained(layer);
        contents.emplace_back(retained, Content(retained));
    }
    const RigExecRigPose actual = Worker(mirror, snapshot, time);
    for (const auto &[layer, before] : contents) CHECK(Content(layer) == before);
    Same(actual, expected);
}

void TestImmutableInFlightAndWeakerUndo() {
    UsdStageRefPtr stage = MakeStage();
    Capture capture(stage);
    const UsdTimeCode time(2);
    const Snapshot old = capture.source.GetSnapshot();
    CHECK(old && old->baseline && old->patches.empty());
    const std::string baseline = Content(old->baseline);
    const RigExecRigPose oldExpected = Live(stage, time);
    RigExecFallbackStageMirror mirror(kRig);
    std::promise<void> entered, resume;
    const auto enteredFuture = entered.get_future();
    auto resumeFuture = resume.get_future();
    const std::thread::id owner = std::this_thread::get_id();
    auto inFlight = std::async(std::launch::async, [&] {
        CHECK(std::this_thread::get_id() != owner);
        entered.set_value(); resumeFuture.wait();
        RigExecRigPose pose; CHECK(mirror.Evaluate(old, time, &pose)); return pose;
    });
    struct ResumeOnExit {
        std::promise<void> &signal;
        ~ResumeOnExit() { try { signal.set_value(); } catch (const std::future_error &) {} }
    } resumeOnExit{resume};
    enteredFuture.wait();
    const UsdAttribute tx = stage->GetAttributeAtPath(kControl.AppendProperty(TfToken("avars:tx")));
    CHECK(tx.Set(9.0));
    const Snapshot edited = capture.source.GetSnapshot();
    CHECK(edited->revision > old->revision);
    CHECK(edited->baseline == old->baseline && !edited->patches.empty());
    const RigExecRigPose editedExpected = Live(stage, time);
    CHECK(editedExpected.controlFrames.at(kControl).Origin()[0] == 9);
    resume.set_value();
    Same(inFlight.get(), oldExpected);
    CHECK(Content(old->baseline) == baseline && old->patches.empty());
    Same(Worker(mirror, edited, time), editedExpected);
    const auto patch = edited->patches.at(tx.GetPath()).layer;
    const std::string patchBefore = Content(patch);
    CHECK(tx.Clear()); // Undo the stronger opinion, revealing weaker default 1.
    CheckCurrent(stage, mirror, capture.source.GetSnapshot(), time);
    double revealed = 0;
    CHECK(stage->GetAttributeAtPath(tx.GetPath()).Get(&revealed, time));
    CHECK(revealed == 1);
    CHECK(Content(patch) == patchBefore);
    RigExecRigPose rejected;
    auto oldJob = std::async(std::launch::async, [&] { return mirror.Evaluate(old, time, &rejected); });
    CHECK(!oldJob.get());
}

void TestAnimationCustomPropertiesAndStructuralBaseline() {
    const UsdStageRefPtr stage = MakeStage();
    Capture capture(stage);
    RigExecFallbackStageMirror mirror(kRig);
    for (double time : {1., 2., 3.}) {
        CheckCurrent(stage, mirror, capture.source.GetSnapshot(), UsdTimeCode(time));
    }
    UsdAttribute tx = stage->GetAttributeAtPath(kControl.AppendProperty(TfToken("avars:tx")));
    TsSpline spline(tx.GetTypeName().GetType());
    for (double time : {1., 3.}) {
        TsKnot knot(tx.GetTypeName().GetType()); knot.SetTime(time); knot.SetValue(time * 2);
        knot.SetNextInterpolation(TsInterpLinear); spline.SetKnot(knot);
    }
    CHECK(tx.SetSpline(spline));
    const UsdAttribute gain = stage->GetAttributeAtPath(SdfPath("/Rig/Settings.custom:gain"));
    CHECK(gain.Set(7.0));
    CheckCurrent(stage, mirror, capture.source.GetSnapshot(), UsdTimeCode(2));
    CHECK(gain.Clear());
    CheckCurrent(stage, mirror, capture.source.GetSnapshot(), UsdTimeCode(2));
    UsdPrim settings = stage->GetPrimAtPath(SdfPath("/Rig/Settings"));
    CHECK(settings.CreateAttribute(TfToken("custom:unused"), SdfValueTypeNames->Double).Set(12.0));
    CheckCurrent(stage, mirror, capture.source.GetSnapshot(), UsdTimeCode(2));
    CHECK(settings.RemoveProperty(TfToken("custom:unused")));
    const Snapshot removed = capture.source.GetSnapshot();
    CHECK(removed->patches.at(SdfPath("/Rig/Settings.custom:unused")).removed);
    CheckCurrent(stage, mirror, removed, UsdTimeCode(2));
    const auto oldBaseline = removed->baseline;
    const std::string oldContents = Content(oldBaseline);
    stage->DefinePrim(SdfPath("/Rig/NewControl"), TfToken("RigExecControl"));
    const Snapshot structural = capture.source.GetSnapshot();
    CHECK(structural->baseline != oldBaseline && structural->patches.empty());
    const RigExecRigPose added = Worker(mirror, structural, UsdTimeCode(2));
    CHECK(added.controlFrames.count(SdfPath("/Rig/NewControl")));
    Same(added, Live(stage, UsdTimeCode(2)));
    CHECK(Content(oldBaseline) == oldContents);
    CHECK(stage->RemovePrim(SdfPath("/Rig/NewControl")));
    CheckCurrent(stage, mirror, capture.source.GetSnapshot(), UsdTimeCode(2));
    stage->SetInterpolationType(UsdInterpolationTypeHeld);
    const Snapshot held = capture.source.GetSnapshot();
    CHECK(held->interpolation == UsdInterpolationTypeHeld);
    CheckCurrent(stage, mirror, held, UsdTimeCode(2));
    const size_t editedProperties = held->patches.size() +
        (held->patches.count(gain.GetPath()) ? 0 : 1);
    for (int i = 0; i < 1000; ++i) CHECK(gain.Set(double(i)));
    CHECK(capture.source.GetSnapshot()->patches.size() == editedProperties);
    CheckCurrent(stage, mirror, capture.source.GetSnapshot(), UsdTimeCode(2));
}

void TestInvalidCompileCanRetryAfterPropertyCorrection() {
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(kRig, TfToken("RigExecRoot"));
    const UsdPrim driver = stage->DefinePrim(SdfPath("/ExternalDriver"), TfToken("RigExecControl"));
    CHECK(driver.GetAttribute(TfToken("avars:tx")).Set(4.0));
    const UsdGeomPoints points = UsdGeomPoints::Define(stage, kPoints.GetPrimPath());
    CHECK(points.CreatePointsAttr().Set(VtVec3fArray{GfVec3f(0), GfVec3f(1, 2, 3)}));
    const UsdPrim mover = stage->DefinePrim(SdfPath("/Rig/Move"), TfToken("RigExecMatrixMover"));
    CHECK(mover.ApplyAPI(TfToken("RigExecMoverAPI")));
    CHECK(mover.GetRelationship(TfToken("rigExec:moves")).SetTargets({kPoints}));
    const UsdRelationship target = mover.GetRelationship(TfToken("rigExec:transform"));
    CHECK(target.SetTargets({})); // Its only output operation is invalid.
    Capture capture(stage);
    const Snapshot invalid = capture.source.GetSnapshot();
    RigExecFallbackStageMirror mirror(kRig);
    auto first = std::async(std::launch::async, [&] {
        RigExecRigPose pose;
        return mirror.Evaluate(invalid, UsdTimeCode(2), &pose);
    });
    CHECK(!first.get());
    CHECK(mirror.HasFailed(invalid->revision, UsdTimeCode(2), {}));
    CHECK(!mirror.HasFailed(invalid->revision, UsdTimeCode(3), {}));
    RigExecFallbackStageOptions otherOptions;
    otherOptions.solverGuidesEnabled = false;
    CHECK(!mirror.HasFailed(invalid->revision, UsdTimeCode(2), otherOptions));
    mirror.ClearFailures();
    CHECK(!mirror.HasFailed(invalid->revision, UsdTimeCode(2), {}));
    CHECK(target.SetTargets({driver.GetPath()}));
    const Snapshot repaired = capture.source.GetSnapshot();
    CHECK(repaired->baseline == invalid->baseline);
    CHECK(!repaired->patches.empty());
    CHECK(!mirror.HasFailed(repaired->revision, UsdTimeCode(2), {}));
    CheckCurrent(stage, mirror, repaired, UsdTimeCode(2));
}

void TestSourceLifetimeDoesNotLeakIntoWorker() {
    UsdStageRefPtr stage = MakeStage();
    const RigExecRigPose expected = Live(stage, UsdTimeCode(2));
    RigExecFallbackStageSource source(stage);
    const Snapshot snapshot = source.GetSnapshot();
    const UsdStageWeakPtr weak(stage);
    stage.Reset();
    CHECK(!weak);
    RigExecFallbackStageMirror mirror(kRig);
    Same(Worker(mirror, snapshot, UsdTimeCode(2)), expected);
}

void TestEvaluationOptionsMatchFreshEvaluator() {
    const UsdStageRefPtr stage = MakeStage();
    RigExecFallbackStageSource source(stage);
    RigExecFallbackStageMirror mirror(kRig);
    for (int variant = 0; variant < 5; ++variant) {
        RigExecFallbackStageOptions options;
        options.preferProgram = variant == 1;
        options.explicitMode = variant == 2 || variant == 3;
        options.mode = variant == 3 ? RigExecEvaluationMode::BakedWithParityCheck
                                    : RigExecEvaluationMode::Dynamic;
        options.publishWeightFields = variant != 3;
        options.solverGuidesEnabled = variant != 3;
        RigExecRigEvaluator live(stage, kRig, options.preferProgram);
        if (options.explicitMode) live.SetEvaluationMode(options.mode);
        live.SetPublishWeightFields(options.publishWeightFields);
        live.SetSolverGuidesEnabled(options.solverGuidesEnabled);
        CHECK(live.Compile());
        const RigExecRigPose expected = live.Evaluate(UsdTimeCode(2));
        const RigExecRigPose actual = Worker(mirror, source.GetSnapshot(), UsdTimeCode(2), options);
        Same(actual, expected);
        CHECK(actual.bakedParityMismatches == expected.bakedParityMismatches);
        CHECK(actual.solverFrames == expected.solverFrames);
        CHECK(actual.weightFields.size() == expected.weightFields.size());
        for (const auto &[path, field] : expected.weightFields) {
            CHECK(actual.weightFields.at(path).weights == field.weights);
        }
    }
}
} // namespace

int main(int argc, char **argv) {
    try {
        CHECK(argc == 2); PlugRegistry::GetInstance().RegisterPlugins(argv[1]);
        TestImmutableInFlightAndWeakerUndo();
        TestAnimationCustomPropertiesAndStructuralBaseline();
        TestInvalidCompileCanRetryAfterPropertyCorrection();
        TestSourceLifetimeDoesNotLeakIntoWorker();
        TestEvaluationOptionsMatchFreshEvaluator();
        std::cout << "private fallback stage worker isolation PASS\n";
        return 0;
    } catch (const std::exception &error) { std::cerr << error.what() << '\n'; return 1; }
}
