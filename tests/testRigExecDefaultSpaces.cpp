// Public computed spaces and mutable default-pose channels.
#include "rigExec/rigEvaluator.h"
#include "rigExec/tapSet.h"
#include "rigExec/types.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/setenv.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/relationship.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <limits>
#include <string>
#include <vector>

using namespace rigExec;
static int failures = 0;
#define CHECK(condition) do { if (!(condition)) { ++failures; \
    std::printf("FAIL %d: %s\n", __LINE__, #condition); } } while (0)

static GfMatrix4d Translate(double x, double y = 0, double z = 0)
{
    return GfMatrix4d(1.0).SetTranslate(GfVec3d(x, y, z));
}

static bool Near(const GfVec3d &a, const GfVec3d &b)
{
    return (a - b).GetLength() < 1e-7;
}

static void CheckFrame(const RigExecPointFrame &frame, const GfMatrix4d &matrix)
{
    const GfVec3d probes[] = {GfVec3d(0), GfVec3d(1, 0, 0),
                            GfVec3d(0, 1, 0), GfVec3d(0, 0, 1)};
    CHECK(frame.IsValid());
    for (size_t i = 0; i < 4; ++i) {
        CHECK(Near(frame.points[i], matrix.TransformAffine(probes[i])));
    }
}

static void TestDefaultChannels()
{
    const auto stage = UsdStage::CreateInMemory();
    const UsdPrim control = stage->DefinePrim(SdfPath("/Control"),
                                             TfToken("RigExecControl"));
    control.GetAttribute(TfToken("rest:space")).Set(Translate(10));
    RigExecTapSet taps(stage);
    const auto rest = taps.Add(RigExecValueAddress::Prim(
        control.GetPath(), TfToken("computeRestFrame")));
    const auto pose = taps.Add(RigExecValueAddress::Prim(
        control.GetPath(), TfToken("computePointFrame")));
    const auto zero = taps.Add(RigExecValueAddress::Prim(
        control.GetPath(), TfToken("computeDefaultFrame")));
    const auto defaultSpace = taps.Add(RigExecValueAddress::Property(
        control.GetPath().AppendProperty(TfToken("default:space"))));
    CHECK(taps.Prepare());
    const char *channels[] = {"default:tx", "default:ty", "default:tz",
                              "default:rx", "default:ry", "default:rz"};
    for (int i = 0; i < 6; ++i) {
        control.GetAttribute(TfToken(channels[i])).Set(i < 3 ? 2.0 : 90.0);
        GfVec3d axis(0);
        axis[i % 3] = 1;
        GfMatrix4d delta = i < 3 ? GfMatrix4d(1).SetTranslate(axis * 2.0)
            : GfMatrix4d(GfRotation(axis, 90.0), GfVec3d(0));
        const GfMatrix4d expected = delta * Translate(10);
        const auto snapshot = taps.Evaluate(UsdTimeCode::Default());
        CHECK(snapshot.IsComplete());
        CheckFrame(snapshot.Get<RigExecPointFrame>(rest), Translate(10));
        CheckFrame(snapshot.Get<RigExecPointFrame>(pose), expected);
        CheckFrame(snapshot.Get<RigExecPointFrame>(zero), expected);
        CHECK(GfIsClose(snapshot.Get<GfMatrix4d>(defaultSpace), expected, 1e-8));
        control.GetAttribute(TfToken(channels[i])).Set(0.0);
    }
}

static void TestDefaultHierarchyAndOverrides()
{
    const auto stage = UsdStage::CreateInMemory();
    const UsdPrim parent = stage->DefinePrim(SdfPath("/Parent"), TfToken("RigExecJoint"));
    const UsdPrim child = stage->DefinePrim(SdfPath("/Parent/Child"), TfToken("RigExecControl"));
    parent.GetAttribute(TfToken("rest:space")).Set(Translate(10));
    // Rest is relative to the namespace frame provider, so the child's 15
    // of world bind offset is authored as 5 past its parent's 10. Every
    // expectation below is a world value and is unchanged by that.
    child.GetAttribute(TfToken("rest:space")).Set(Translate(5));
    parent.GetAttribute(TfToken("default:tx")).Set(2.0);
    parent.GetAttribute(TfToken("avars:tx")).Set(3.0);
    child.GetAttribute(TfToken("default:ty")).Set(4.0);
    child.GetAttribute(TfToken("avars:tx")).Set(1.0);
    child.GetAttribute(TfToken("avars:unitScaleFactor")).Set(2.0);
    RigExecTapSet taps(stage);
    const auto pose = taps.Add(RigExecValueAddress::Prim(child.GetPath(), TfToken("computePointFrame")));
    std::map<std::string, RigExecTapId> spaces;
    for (const char *name : {"default:space", "avars:defaultSpace",
                             "posed:defaultSpace", "parent:defaultSpace", "parent:space"}) {
        spaces[name] = taps.Add(RigExecValueAddress::Property(
            child.GetPath().AppendProperty(TfToken(name))));
    }
    CHECK(taps.Prepare());
    auto snapshot = taps.Evaluate(UsdTimeCode::Default());
    CHECK(snapshot.IsComplete());
    CheckFrame(snapshot.Get<RigExecPointFrame>(pose), Translate(22, 4));
    for (const char *name : {"default:space", "avars:defaultSpace", "posed:defaultSpace"}) {
        CHECK(GfIsClose(snapshot.Get<GfMatrix4d>(spaces[name]), Translate(17, 4), 1e-8));
    }
    CHECK(GfIsClose(snapshot.Get<GfMatrix4d>(spaces["parent:defaultSpace"]), Translate(12), 1e-8));
    CHECK(GfIsClose(snapshot.Get<GfMatrix4d>(spaces["parent:space"]), Translate(15), 1e-8));

    // Each explicit zero-space input can independently steer the pose.
    child.GetAttribute(TfToken("default:space")).Set(Translate(30));
    snapshot = taps.Evaluate(UsdTimeCode::Default());
    CheckFrame(snapshot.Get<RigExecPointFrame>(pose), Translate(35));
    child.GetAttribute(TfToken("avars:defaultSpace")).Set(Translate(40));
    snapshot = taps.Evaluate(UsdTimeCode::Default());
    CheckFrame(snapshot.Get<RigExecPointFrame>(pose), Translate(45));
    child.GetAttribute(TfToken("posed:defaultSpace")).Set(Translate(50));
    child.GetAttribute(TfToken("parent:defaultSpace")).Set(Translate(20));
    child.GetAttribute(TfToken("parent:space")).Set(Translate(60));
    snapshot = taps.Evaluate(UsdTimeCode::Default());
    CheckFrame(snapshot.Get<RigExecPointFrame>(pose), Translate(92));

    // An identity connection overrides a non-identity authored value and
    // survives edits without re-creating the prepared request.
    const auto identity = stage->DefinePrim(SdfPath("/Driver"))
        .CreateAttribute(TfToken("matrix"), SdfValueTypeNames->Matrix4d);
    identity.Set(GfMatrix4d(1));
    child.GetAttribute(TfToken("posed:defaultSpace")).SetConnections({identity.GetPath()});
    snapshot = taps.Evaluate(UsdTimeCode::Default());
    CHECK(snapshot.IsComplete());
    CheckFrame(snapshot.Get<RigExecPointFrame>(pose), Translate(42));
    identity.Set(Translate(5));
    snapshot = taps.Evaluate(UsdTimeCode::Default());
    CheckFrame(snapshot.Get<RigExecPointFrame>(pose), Translate(47));
    identity.Set(Translate(7), UsdTimeCode(1));
    identity.Set(Translate(9), UsdTimeCode(2));
    snapshot = taps.Evaluate(UsdTimeCode(2));
    CheckFrame(snapshot.Get<RigExecPointFrame>(pose), Translate(51));
}

static void TestInvalidSpacesPropagate()
{
    const auto stage = UsdStage::CreateInMemory();
    const auto parent = stage->DefinePrim(SdfPath("/Parent"), TfToken("RigExecControl"));
    const auto child = stage->DefinePrim(SdfPath("/Parent/Child"), TfToken("RigExecControl"));
    parent.GetAttribute(TfToken("rest:tx")).Set(2.0);
    RigExecTapSet taps(stage);
    const auto parentPose = taps.Add(RigExecValueAddress::Prim(parent.GetPath(), TfToken("computePointFrame")));
    const auto childPose = taps.Add(RigExecValueAddress::Prim(child.GetPath(), TfToken("computePointFrame")));
    CHECK(taps.Prepare());
    auto snapshot = taps.Evaluate(UsdTimeCode::Default());
    CHECK(snapshot.Get<RigExecPointFrame>(parentPose).IsValid());
    CHECK(snapshot.Get<RigExecPointFrame>(childPose).IsValid());
    parent.GetAttribute(TfToken("rest:tx")).Set(std::numeric_limits<double>::quiet_NaN());
    snapshot = taps.Evaluate(UsdTimeCode::Default());
    CHECK(snapshot.Get<RigExecPointFrame>(parentPose).IsDegenerate());
    CHECK(snapshot.Get<RigExecPointFrame>(childPose).IsDegenerate());
    parent.GetAttribute(TfToken("rest:tx")).Set(2.0);
    snapshot = taps.Evaluate(UsdTimeCode::Default());
    CHECK(snapshot.Get<RigExecPointFrame>(parentPose).IsValid());
    CHECK(snapshot.Get<RigExecPointFrame>(childPose).IsValid());
}

static void TestAnimatedTwistTurns()
{
    const auto stage = UsdStage::CreateInMemory();
    const auto start = stage->DefinePrim(SdfPath("/Start"), TfToken("RigExecControl"));
    const auto end = stage->DefinePrim(SdfPath("/End"), TfToken("RigExecControl"));
    const auto twist = stage->DefinePrim(SdfPath("/Twist"), TfToken("RigExecTwistDistribution"));
    twist.GetRelationship(TfToken("rigExec:start")).SetTargets({start.GetPath()});
    twist.GetRelationship(TfToken("rigExec:end")).SetTargets({end.GetPath()});
    twist.GetAttribute(TfToken("rigExec:weights")).Set(VtFloatArray{0, 0.25f, 0.5f, 1});
    const auto turns = twist.GetAttribute(TfToken("inputs:twistTurns"));
    CHECK(turns);
    turns.Set(1.0, UsdTimeCode(1));
    turns.Set(-1.0, UsdTimeCode(2));
    RigExecTapSet taps(stage);
    const auto tap = taps.Add(RigExecValueAddress::Prim(twist.GetPath(), TfToken("computePointFrameArray")));
    CHECK(taps.Prepare());
    for (const double time : {1.0, 2.0}) {
        const auto snapshot = taps.Evaluate(UsdTimeCode(time));
        CHECK(snapshot.IsComplete());
        const auto frames = snapshot.Get<RigExecPointFrameArray>(tap);
        CHECK(frames.GetSize() == 4);
        if (frames.GetSize() != 4) continue;
        const double sign = time == 1 ? 1 : -1;
        CHECK(Near(frames.frames[1].Y(), GfVec3d(0, 0, sign)));
        CHECK(Near(frames.frames[2].Y(), GfVec3d(0, -1, 0)));
        CHECK(Near(frames.frames[3].Y(), GfVec3d(0, 1, 0)));
    }
    turns.Set(0.5, UsdTimeCode(2));
    const auto changed = taps.Evaluate(UsdTimeCode(2)).Get<RigExecPointFrameArray>(tap);
    CHECK(changed.GetSize() == 4);
    if (changed.GetSize() == 4) CHECK(Near(changed.frames.back().Y(), GfVec3d(0, -1, 0)));
}

// Two controls whose default:spaces are connected to each other. The compile's
// pose-input closure follows connections and the default-space chain, so the
// pair is one cycle of that graph, and the closure has to come back from it
// with everything the cycle reaches: through B, the parent's default chain,
// and through A's rest channel, a connected parent:space that names another
// control as a pose provider and makes the pose connected. main() turns on
// RIGEXEC_VERIFY_POSEINFO, under which every such closure is also walked prim
// by prim and a difference is fatal; what this checks on top is that the
// compile got as far as computing them. Nothing here asks the cycle to
// evaluate -- only that it is read -- so the exec cycle reports the compile's
// tap prepares print on the way are expected.
static void TestMutualDefaultSpaceCycleClosure()
{
    const auto stage = UsdStage::CreateInMemory();
    const SdfPath rigPath("/Rig");
    stage->DefinePrim(rigPath, TfToken("RigExecRoot"));
    stage->DefinePrim(SdfPath("/Rig/Controls"), TfToken("Scope"));
    const auto parent = stage->DefinePrim(SdfPath("/Rig/Controls/P"),
                                          TfToken("RigExecControl"));
    const auto a = stage->DefinePrim(SdfPath("/Rig/Controls/P/A"),
                                     TfToken("RigExecControl"));
    const auto b = stage->DefinePrim(SdfPath("/Rig/Controls/P/B"),
                                     TfToken("RigExecControl"));
    stage->DefinePrim(SdfPath("/Rig/Controls/Q"), TfToken("RigExecControl"));
    const auto c = stage->DefinePrim(SdfPath("/Rig/Controls/Q/C"),
                                     TfToken("RigExecControl"));
    parent.GetAttribute(TfToken("rest:space")).Set(Translate(1));
    const TfToken defaultSpace("default:space");
    CHECK(a.GetAttribute(defaultSpace).SetConnections(
        {b.GetPath().AppendProperty(defaultSpace)}));
    CHECK(b.GetAttribute(defaultSpace).SetConnections(
        {a.GetPath().AppendProperty(defaultSpace)}));
    CHECK(a.GetAttribute(TfToken("rest:tx")).SetConnections(
        {c.GetPath().AppendProperty(TfToken("parent:space"))}));

    RigExecRigEvaluator rig(stage, rigPath);
    rig.SetProfilingEnabled(true);
    std::vector<std::string> errors;
    rig.Compile(&errors);
    const auto events = rig.GetProfiler().GetEvents();
    CHECK(std::any_of(events.begin(), events.end(), [](const auto &event) {
        return event.name == "PoseInfoPrefetch";
    }));
}

int main()
{
    // Before the first compile: the verifier reads its switch once.
    TfSetenv("RIGEXEC_VERIFY_POSEINFO", "1");
    PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);
    TestMutualDefaultSpaceCycleClosure();
    TestDefaultChannels();
    TestDefaultHierarchyAndOverrides();
    TestInvalidSpacesPropagate();
    TestAnimatedTwistTurns();
    std::printf("testRigExecDefaultSpaces: %d failure(s)\n", failures);
    return failures ? 1 : 0;
}
