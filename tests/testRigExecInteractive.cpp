// Persistent graph and interactive edit regressions through the public evaluator.
#include "rigExec/rigEvaluator.h"
#include "pxr/base/plug/registry.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/relationship.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

using namespace rigExec;
static int failures = 0;
#define CHECK(condition) do { if (!(condition)) { ++failures; \
    std::printf("FAIL %d: %s\n", __LINE__, #condition); } } while (0)

static void TestPersistentChains()
{
    constexpr size_t count = 256;
    auto stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const UsdPrim driver = stage->DefinePrim(
        SdfPath("/Asset/Rig/Driver"), TfToken("RigExecControl"));
    driver.GetAttribute(TfToken("avars:tx")).Set(1.0);
    const UsdPrim sideDriver = stage->DefinePrim(
        SdfPath("/Asset/Rig/SideDriver"), TfToken("RigExecControl"));
    sideDriver.GetAttribute(TfToken("avars:ty")).Set(2.0);
    const SdfPath target("/Asset/Shape.points");
    const SdfPath sideTarget("/Asset/Side.points");
    for (const SdfPath &path : {target, sideTarget}) {
        const UsdPrim points = stage->DefinePrim(path.GetPrimPath(), TfToken("Points"));
        points.GetAttribute(TfToken("points")).Set(VtVec3fArray{GfVec3f(0)});
    }
    stage->GetPrimAtPath(target.GetPrimPath()).GetAttribute(TfToken("extent"))
        .Set(VtVec3fArray{GfVec3f(0), GfVec3f(0)});
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    for (size_t i = 0; i <= count; ++i) {
        const UsdPrim mover = stage->DefinePrim(
            SdfPath("/Asset/Rig/Movers/M" + std::to_string(i)),
            TfToken("RigExecMatrixMover"));
        mover.ApplyAPI(TfToken("RigExecMoverAPI"));
        mover.GetRelationship(TfToken("rigExec:moves"))
            .SetTargets({i == count ? sideTarget : target});
        mover.GetRelationship(TfToken("rigExec:transform"))
            .SetTargets({i == count ? sideDriver.GetPath() : driver.GetPath()});
        mover.GetAttribute(TfToken("inputs:defaultWeight")).Set(1.0f);
    }
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    for (const auto &error : errors) std::printf("compile: %s\n", error.c_str());
    const size_t epoch = evaluator.GetBindingEpochDigest();
    auto checkPose = [&](const RigExecRigPose &pose, float expectedX) {
        CHECK(pose.valid);
        const auto found = pose.movedProperties.find(target);
        CHECK(found != pose.movedProperties.end());
        if (found == pose.movedProperties.end()) return;
        const auto points = found->second.Get<VtVec3fArray>();
        CHECK(points.size() == 1);
        if (!points.empty()) CHECK(std::abs(points[0][0] - expectedX) < 1e-4f);
        const auto extent = pose.movedProperties.at(SdfPath("/Asset/Shape.extent"))
            .Get<VtVec3fArray>();
        CHECK(extent.size() == 2);
        if (extent.size() == 2) CHECK(std::abs(extent[1][0] - expectedX) < 1e-4f);
        const auto side = pose.movedProperties.at(sideTarget).Get<VtVec3fArray>();
        CHECK(side.size() == 1 && side[0] == GfVec3f(0, 2, 0));
    };
    auto pose = evaluator.Evaluate(UsdTimeCode::Default());
    checkPose(pose, float(count));
    CHECK(pose.moverGraphRevisionsCreated == count + 2);
    CHECK(pose.moverGraphRevisionsExecuted == count + 2);
    CHECK(pose.moverGraphSchedulesBuilt == 3);
    CHECK(pose.moverGraphParityAgreements == 0);
    pose = evaluator.Evaluate(UsdTimeCode::Default());
    checkPose(pose, float(count));
    CHECK(pose.moverGraphRevisionsCreated == 0);
    CHECK(pose.moverGraphRevisionsExecuted == 0);
    CHECK(pose.moverGraphSchedulesBuilt == 0);

    // Change the last operation in the composed execution order. Its whole
    // prefix and the independent branch must remain cached.
    SdfPath last;
    for (const auto &mover : evaluator.GetMoverOrder()) {
        if (mover.targets == std::vector<SdfPath>{target}) last = mover.moverPath;
    }
    CHECK(!last.IsEmpty());
    const auto weight = stage->GetPrimAtPath(last)
        .GetAttribute(TfToken("inputs:defaultWeight"));
    weight.Set(0.5f);
    pose = evaluator.Evaluate(UsdTimeCode::Default());
    checkPose(pose, float(count) - 0.5f);
    CHECK(pose.moverGraphRevisionsExecuted == 2); // tail and derived extent
    CHECK(pose.moverGraphRevisionsCreated == 0);
    CHECK(pose.moverGraphSchedulesBuilt == 0);
    CHECK(evaluator.GetBindingEpochDigest() == epoch);

    stage->GetAttributeAtPath(target).Set(VtVec3fArray{GfVec3f(10, 0, 0)});
    pose = evaluator.Evaluate(UsdTimeCode::Default());
    checkPose(pose, float(count) + 9.5f);
    CHECK(pose.moverGraphRevisionsExecuted == count + 1);
    CHECK(pose.moverGraphRevisionsCreated == 0);
    CHECK(pose.moverGraphSchedulesBuilt == 0);
    CHECK(evaluator.GetBindingEpochDigest() == epoch);

    const auto enabled = stage->GetPrimAtPath(last)
        .GetAttribute(TfToken("inputs:enabled"));
    enabled.Set(false);
    pose = evaluator.Evaluate(UsdTimeCode::Default());
    checkPose(pose, float(count) + 9.0f);
    CHECK(pose.moverGraphRevisionsExecuted == 2);
    CHECK(pose.moverGraphRevisionsCreated == 0);
    CHECK(pose.moverGraphSchedulesBuilt == 0);
    enabled.Set(true);
    pose = evaluator.Evaluate(UsdTimeCode::Default());
    checkPose(pose, float(count) + 9.5f);
    CHECK(pose.moverGraphRevisionsExecuted == 2);
    CHECK(evaluator.GetBindingEpochDigest() == epoch);

    evaluator.cpuParityMode = true;
    pose = evaluator.Evaluate(UsdTimeCode::Default());
    checkPose(pose, float(count) + 9.5f);
    CHECK(pose.moverGraphRevisionsExecuted == 0);
    CHECK(pose.moverGraphParityAgreements == 2);
    CHECK(pose.moverGraphParityMismatches == 0);
}

static void TestStructuralSplices()
{
    auto stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    auto scope = stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    auto driver = stage->DefinePrim(SdfPath("/Asset/Rig/C"), TfToken("RigExecControl"));
    driver.GetAttribute(TfToken("avars:tx")).Set(1.0);
    auto other = stage->DefinePrim(SdfPath("/Asset/Rig/D"), TfToken("RigExecControl"));
    other.GetAttribute(TfToken("avars:tx")).Set(3.0);
    for (const char *name : {"Shape", "Side"}) {
        stage->DefinePrim(SdfPath(std::string("/Asset/") + name), TfToken("Points"))
            .GetAttribute(TfToken("points")).Set(VtVec3fArray{GfVec3f(0)});
    }
    const SdfPath target("/Asset/Shape.points"), side("/Asset/Side.points");
    auto add = [&](const std::string &name, const SdfPath &out) {
        auto prim = stage->DefinePrim(scope.GetPath().AppendChild(TfToken(name)),
                                     TfToken("RigExecMatrixMover"));
        prim.ApplyAPI(TfToken("RigExecMoverAPI"));
        prim.GetRelationship(TfToken("rigExec:moves")).SetTargets({out});
        prim.GetRelationship(TfToken("rigExec:transform")).SetTargets({driver.GetPath()});
        return prim;
    };
    constexpr int count = 128;
    for (int i = 0; i < count; ++i) add("M" + std::to_string(i), target);
    add("Side", side);
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    auto pull = [&]() { auto p = evaluator.Evaluate(UsdTimeCode::Default());
        for (const auto &s : p.diagnostics) if (!p.valid) std::printf("%s\n", s.c_str());
        CHECK(p.valid); return p; };
    auto x = [&](const RigExecRigPose &p) {
        return p.movedProperties.at(target).Get<VtVec3fArray>()[0][0]; };
    auto pose = pull();
    CHECK(x(pose) == count);
    // Stable tap systems share compiler nodes across requests and epochs.
    RigExecTapSet first(stage), second(stage);
    CHECK(first.GetSystem() == second.GetSystem());
    ExecUsdSystem *system = first.GetSystem();

    auto tail = add("Inserted", target);
    auto order = scope.GetChildrenReorder();
    // Reverse sibling traversal: first child executes last.
    TfTokenVector names{TfToken("Inserted")};
    for (const auto &child : scope.GetChildren())
        if (child != tail) names.push_back(child.GetName());
    scope.SetChildrenReorder(names);
    pose = pull();
    CHECK(x(pose) == count + 1);
    CHECK(pose.moverGraphRevisionsCreated == 1);
    CHECK(pose.moverGraphRevisionsExecuted == 1);
    CHECK(pose.moverGraphSchedulesBuilt == 1);
    CHECK(first.GetSystem() == system);
    tail.GetRelationship(TfToken("rigExec:transform")).SetTargets({other.GetPath()});
    pose = pull();
    CHECK(x(pose) == count + 3);
    CHECK(pose.moverGraphRevisionsCreated == 0);
    CHECK(pose.moverGraphRevisionsExecuted == 1);
    CHECK(pose.moverGraphSchedulesBuilt == 0);
    CHECK(first.GetSystem() == system);

    // Deleting a prim retires the stock listener before its invalid-prim
    // predicate runs; surviving point checkpoints remain live.
    CHECK(stage->RemovePrim(tail.GetPath()));
    pose = pull();
    CHECK(x(pose) == count);
    CHECK(pose.moverGraphRevisionsCreated == 0);
    CHECK(pose.moverGraphRevisionsExecuted == 0);
    CHECK(pose.moverGraphSchedulesBuilt == 1);
    CHECK(pull().moverGraphRevisionsExecuted == 0);

    // Reorder the last two operations: preserve all nodes, dirty just two.
    names.erase(names.begin());
    std::swap(names[0], names[1]);
    scope.SetChildrenReorder(names);
    pose = pull();
    CHECK(x(pose) == count);
    CHECK(pose.moverGraphRevisionsCreated == 0);
    CHECK(pose.moverGraphRevisionsExecuted == 2);
    CHECK(pose.moverGraphSchedulesBuilt == 1);
    CHECK(pose.movedProperties.at(side).Get<VtVec3fArray>()[0][0] == 1);
}

static void TestBlendSampleReadPhases()
{
    auto stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const auto scope = stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const SdfPath target("/Asset/Shape.points"), samplePoints("/Asset/Sample.points");
    for (const auto &path : {target, samplePoints}) {
        stage->DefinePrim(path.GetPrimPath(), TfToken("Points"))
            .GetAttribute(TfToken("points"))
            .Set(VtVec3fArray{path == target ? GfVec3f(0) : GfVec3f(10, 0, 0)});
    }
    const auto x = stage->DefinePrim(SdfPath("/Asset/Rig/X"), TfToken("RigExecControl"));
    const auto y = stage->DefinePrim(SdfPath("/Asset/Rig/Y"), TfToken("RigExecControl"));
    x.GetAttribute(TfToken("avars:tx")).Set(2.0);
    y.GetAttribute(TfToken("avars:ty")).Set(3.0);
    for (const auto &entry : {std::make_pair("Early", x), std::make_pair("Late", y)}) {
        const auto mover = stage->DefinePrim(scope.GetPath().AppendChild(TfToken(entry.first)),
                                             TfToken("RigExecMatrixMover"));
        mover.ApplyAPI(TfToken("RigExecMoverAPI"));
        mover.GetRelationship(TfToken("rigExec:moves")).SetTargets({samplePoints});
        mover.GetRelationship(TfToken("rigExec:transform")).SetTargets({entry.second.GetPath()});
    }
    const auto sample = stage->DefinePrim(SdfPath("/Asset/Rig/S"), TfToken("RigExecBlendSample"));
    sample.GetRelationship(TfToken("rigExec:targetPoints")).SetTargets({samplePoints});
    const auto channel = stage->DefinePrim(SdfPath("/Asset/Rig/B"), TfToken("RigExecBlendInput"));
    channel.GetAttribute(TfToken("inputs:weight")).Set(1.0f);
    channel.GetRelationship(TfToken("rigExec:samples")).SetTargets({sample.GetPath()});
    const auto blend = stage->DefinePrim(scope.GetPath().AppendChild(TfToken("Blend")),
                                         TfToken("RigExecBlendShapeMover"));
    blend.ApplyAPI(TfToken("RigExecMoverAPI"));
    blend.GetRelationship(TfToken("rigExec:moves")).SetTargets({target});
    blend.GetRelationship(TfToken("rigExec:blendInputs")).SetTargets({channel.GetPath()});
    scope.SetChildrenReorder({TfToken("Late"), TfToken("Blend"), TfToken("Early")});
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    evaluator.cpuParityMode = true;
    auto check = [&](const GfVec3f &expected, bool retained) {
        const auto pose = evaluator.Evaluate(UsdTimeCode::Default());
        CHECK(pose.valid);
        if (!pose.valid) {
            for (const auto &s : pose.diagnostics) std::printf("%s\n", s.c_str());
            return;
        }
        CHECK(pose.movedProperties.at(target).Get<VtVec3fArray>()[0] == expected);
        CHECK(pose.moverGraphParityMismatches == 0);
        if (retained) CHECK(pose.moverGraphRevisionsCreated == 0);
    };
    check(GfVec3f(10, 0, 0), false);
    sample.GetAttribute(TfToken("rigExec:pointsReadPhase")).Set(TfToken("final"));
    check(GfVec3f(12, 3, 0), true);
    x.GetAttribute(TfToken("avars:tx")).Set(4.0);
    check(GfVec3f(14, 3, 0), true);
    sample.GetAttribute(TfToken("rigExec:pointsReadPhase")).Set(TfToken("preceding"));
    check(GfVec3f(14, 0, 0), true);
    sample.GetRelationship(TfToken("rigExec:targetPoints"))
        .SetMetadata(TfToken(RigExecReadPhaseMetadataName), std::string("final"));
    check(GfVec3f(14, 3, 0), true);
    sample.GetRelationship(TfToken("rigExec:targetPoints")).SetTargets({target});
    CHECK(!evaluator.Evaluate(UsdTimeCode::Default()).valid);
    sample.GetRelationship(TfToken("rigExec:targetPoints")).SetTargets({samplePoints});
    check(GfVec3f(14, 3, 0), true);
    const SdfPath secondTarget("/Asset/Second.points");
    stage->DefinePrim(secondTarget.GetPrimPath(), TfToken("Points"))
        .GetAttribute(TfToken("points")).Set(VtVec3fArray{GfVec3f(100, 0, 0)});
    blend.GetRelationship(TfToken("rigExec:moves")).SetTargets({target, secondTarget});
    channel.GetAttribute(TfToken("inputs:weight")).Set(0.5f);
    const auto fanout = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(fanout.valid);
    CHECK(fanout.movedProperties.at(target).Get<VtVec3fArray>()[0] == GfVec3f(7, 1.5f, 0));
    CHECK(fanout.movedProperties.at(secondTarget).Get<VtVec3fArray>()[0] == GfVec3f(57, 1.5f, 0));
    CHECK(fanout.moverGraphRevisionsCreated == 1);
    CHECK(fanout.moverGraphParityMismatches == 0);
}

static void TestCurvenetWeightsUpdate()
{
    auto stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"),TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"),TfToken("RigExecRoot"));
    const auto mesh = stage->DefinePrim(SdfPath("/Asset/Mesh"),TfToken("Mesh"));
    mesh.GetAttribute(TfToken("points")).Set(VtVec3fArray{{0,0,0},{1,0,0},{1,1,0},{0,1,0}});
    mesh.GetAttribute(TfToken("faceVertexCounts")).Set(VtIntArray{4});
    mesh.GetAttribute(TfToken("faceVertexIndices")).Set(VtIntArray{0,1,2,3});
    const auto net = stage->DefinePrim(SdfPath("/Asset/Net"),TfToken("RigExecCurvenet"));
    net.GetAttribute(TfToken("points")).Set(VtVec3fArray{{0,0.5f,0},{0.33f,0.5f,0},{0.67f,0.5f,0},{1,0.5f,0}});
    net.GetAttribute(TfToken("rigExec:splineIndices")).Set(VtIntArray{0,1,2,3});
    const auto weight = stage->DefinePrim(SdfPath("/Asset/Rig/W"),TfToken("RigExecCurvenetWeight"));
    weight.GetRelationship(TfToken("rigExec:weightTarget")).SetTargets({mesh.GetPath().AppendProperty(TfToken("points"))});
    weight.GetRelationship(TfToken("rigExec:curvenetPoints")).SetTargets({net.GetPath().AppendProperty(TfToken("points"))});
    weight.GetRelationship(TfToken("rigExec:curvenetSplineIndices")).SetTargets({net.GetPath().AppendProperty(TfToken("rigExec:splineIndices"))});
    weight.GetRelationship(TfToken("rigExec:meshFaceCounts")).SetTargets({mesh.GetPath().AppendProperty(TfToken("faceVertexCounts"))});
    weight.GetRelationship(TfToken("rigExec:meshFaceIndices")).SetTargets({mesh.GetPath().AppendProperty(TfToken("faceVertexIndices"))});
    weight.GetAttribute(TfToken("inputs:weights")).Set(VtFloatArray(4,1.0f));
    const auto driver = stage->DefinePrim(SdfPath("/Asset/Rig/C"),TfToken("RigExecControl"));
    driver.GetAttribute(TfToken("avars:tz")).Set(2.0);
    const auto mover = stage->DefinePrim(SdfPath("/Asset/Rig/Movers/M"),TfToken("RigExecMatrixMover"));
    mover.ApplyAPI(TfToken("RigExecMoverAPI"));
    mover.GetRelationship(TfToken("rigExec:moves")).SetTargets({mesh.GetPath().AppendProperty(TfToken("points"))});
    mover.GetRelationship(TfToken("rigExec:transform")).SetTargets({driver.GetPath()});
    mover.GetRelationship(TfToken("rigExec:weightObject")).SetTargets({weight.GetPath()});
    RigExecRigEvaluator evaluator(stage,SdfPath("/Asset/Rig"));
    evaluator.cpuParityMode = true;
    auto check = [&](float expected) {
        auto p = evaluator.Evaluate(UsdTimeCode::Default());
        CHECK(p.valid);
        if (!p.valid) { for (const auto &s:p.diagnostics) std::printf("%s\n",s.c_str()); return p; }
        const auto points = p.movedProperties.at(mesh.GetPath().AppendProperty(TfToken("points"))).Get<VtVec3fArray>();
        for (const auto &point:points) CHECK(std::abs(point[2]-expected)<1e-4f);
        CHECK(p.moverGraphParityMismatches == 0);
        return p;
    };
    check(2.0f);
    const size_t epoch = evaluator.GetBindingEpochDigest();
    for (float value:{0.25f,0.5f,0.75f}) {
        weight.GetAttribute(TfToken("inputs:weights")).Set(VtFloatArray(4,value));
        const auto p = check(2*value);
        CHECK(p.moverGraphRevisionsCreated == 0);
        CHECK(p.moverGraphRevisionsExecuted == 1);
        CHECK(evaluator.GetBindingEpochDigest() == epoch);
    }
    weight.GetRelationship(TfToken("rigExec:weightTarget")).SetTargets({mesh.GetPath()});
    CHECK(!evaluator.Evaluate(UsdTimeCode::Default()).valid);
    weight.GetRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({mesh.GetPath().AppendProperty(TfToken("points"))});
    check(1.5f);
    weight.GetRelationship(TfToken("rigExec:meshFaceCounts"))
        .SetTargets({mesh.GetPath().AppendProperty(TfToken("faceVertexIndices"))});
    CHECK(!evaluator.Evaluate(UsdTimeCode::Default()).valid);
    weight.GetRelationship(TfToken("rigExec:meshFaceCounts"))
        .SetTargets({mesh.GetPath().AppendProperty(TfToken("faceVertexCounts"))});
    check(1.5f);
}

static void TestBlendSurfaceFrames()
{
    auto stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const auto mesh = stage->DefinePrim(SdfPath("/Asset/Mesh"), TfToken("Mesh"));
    const VtVec3fArray rest{{0,0,0}, {1,0,0}, {1,1,0}, {0,1,0}};
    mesh.GetAttribute(TfToken("points")).Set(rest);
    mesh.GetAttribute(TfToken("faceVertexCounts")).Set(VtIntArray{4});
    mesh.GetAttribute(TfToken("faceVertexIndices")).Set(VtIntArray{0,1,2,3});
    const auto control = stage->DefinePrim(SdfPath("/Asset/Rig/C"), TfToken("RigExecControl"));
    control.GetAttribute(TfToken("avars:rx")).Set(90.0);
    const SdfPath target("/Asset/Mesh.points");
    const auto rotate = stage->DefinePrim(SdfPath("/Asset/Rig/Movers/A"), TfToken("RigExecMatrixMover"));
    rotate.ApplyAPI(TfToken("RigExecMoverAPI"));
    rotate.GetRelationship(TfToken("rigExec:moves")).SetTargets({target});
    rotate.GetRelationship(TfToken("rigExec:transform")).SetTargets({control.GetPath()});
    const auto sculpt = stage->DefinePrim(SdfPath("/Asset/Sculpt"), TfToken("Points"));
    auto shape = rest;
    for (auto &p : shape) p[2] += 1.0f;
    sculpt.GetAttribute(TfToken("points")).Set(shape);
    const auto sample = stage->DefinePrim(SdfPath("/Asset/Rig/Sample"), TfToken("RigExecBlendSample"));
    sample.GetRelationship(TfToken("rigExec:targetPoints")).SetTargets({SdfPath("/Asset/Sculpt.points")});
    const auto channel = stage->DefinePrim(SdfPath("/Asset/Rig/Channel"), TfToken("RigExecBlendInput"));
    channel.GetRelationship(TfToken("rigExec:samples")).SetTargets({sample.GetPath()});
    channel.GetAttribute(TfToken("inputs:weight")).Set(1.0f);
    const auto blend = stage->DefinePrim(SdfPath("/Asset/Rig/Movers/B"), TfToken("RigExecBlendShapeMover"));
    blend.ApplyAPI(TfToken("RigExecMoverAPI"));
    blend.GetRelationship(TfToken("rigExec:moves")).SetTargets({target});
    blend.GetRelationship(TfToken("rigExec:blendInputs")).SetTargets({channel.GetPath()});
    blend.GetAttribute(TfToken("rigExec:deltaSpace")).Set(TfToken("surfaceFrame"));
    stage->GetPrimAtPath(SdfPath("/Asset/Rig/Movers"))
        .SetChildrenReorder({TfToken("B"), TfToken("A")});
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    evaluator.cpuParityMode = true;
    CHECK(evaluator.Compile());
    auto check = [&](const GfVec3f &expected) {
        const auto pose = evaluator.Evaluate(UsdTimeCode::Default());
        CHECK(pose.valid && pose.moverGraphParityMismatches == 0);
        if (pose.valid) CHECK((pose.movedProperties.at(target).Get<VtVec3fArray>()[0] - expected).GetLength() < 1e-5f);
        return pose;
    };
    check(GfVec3f(0,-1,0));
    channel.GetAttribute(TfToken("inputs:weight")).Set(0.5f);
    const auto edited = check(GfVec3f(0,-0.5f,0));
    CHECK(edited.moverGraphRevisionsCreated == 0);
    CHECK(edited.moverGraphRevisionsExecuted == 1);
    blend.GetAttribute(TfToken("rigExec:deltaSpace")).Set(TfToken("target"));
    check(GfVec3f(0,0,0.5f));
    blend.GetAttribute(TfToken("rigExec:deltaSpace")).Set(TfToken("unknown"));
    CHECK(!evaluator.Evaluate(UsdTimeCode::Default()).valid);
    blend.GetAttribute(TfToken("rigExec:deltaSpace")).Set(TfToken("surfaceFrame"));
    check(GfVec3f(0,-0.5f,0));
}

static void TestGeometryConstraintsCompose()
{
    auto stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const auto scope = stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const SdfPath target("/Asset/Shape.points");
    stage->DefinePrim(target.GetPrimPath(), TfToken("Points"))
        .GetAttribute(TfToken("points")).Set(VtVec3fArray{GfVec3f(1,0,0)});
    auto driver = [&](const char *name, double x) {
        const auto prim = stage->DefinePrim(SdfPath(std::string("/Asset/Rig/") + name), TfToken("RigExecControl"));
        prim.GetAttribute(TfToken("avars:tx")).Set(x);
        return prim;
    };
    const auto a = driver("A", 10), b = driver("B", 2), c = driver("C", 3);
    auto mover = [&](const char *name, const char *type, const char *rel, const UsdPrim &source) {
        const auto prim = stage->DefinePrim(scope.GetPath().AppendChild(TfToken(name)), TfToken(type));
        prim.ApplyAPI(TfToken("RigExecMoverAPI"));
        prim.GetRelationship(TfToken("rigExec:moves")).SetTargets({target});
        prim.GetRelationship(TfToken(rel)).SetTargets({source.GetPath()});
        return prim;
    };
    mover("Matrix", "RigExecMatrixMover", "rigExec:transform", a);
    const auto first = mover("First", "RigExecPositionConstraint", "rigExec:sources", b);
    const auto last = mover("Last", "RigExecPositionConstraint", "rigExec:sources", c);
    scope.SetChildrenReorder({TfToken("Last"), TfToken("First"), TfToken("Matrix")});
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    evaluator.cpuParityMode = true;
    auto check = [&](float expected) {
        const auto pose = evaluator.Evaluate(UsdTimeCode::Default());
        CHECK(pose.valid && pose.moverGraphParityMismatches == 0);
        if (pose.valid) CHECK(std::abs(pose.movedProperties.at(target).Get<VtVec3fArray>()[0][0] - expected) < 1e-5f);
        else for (const auto &d : pose.diagnostics) std::printf("%s\n", d.c_str());
        return pose;
    };
    check(16);
    CHECK(check(16).moverGraphRevisionsExecuted == 0);
    c.GetAttribute(TfToken("avars:tx")).Set(4.0);
    auto pose = check(17);
    CHECK(pose.moverGraphRevisionsExecuted == 1 && pose.moverGraphRevisionsCreated == 0);
    b.GetAttribute(TfToken("avars:tx")).Set(5.0);
    CHECK(check(20).moverGraphRevisionsExecuted == 2);
    first.GetAttribute(TfToken("inputs:defaultWeight")).Set(0.0f);
    CHECK(check(15).moverGraphRevisionsExecuted == 2);
    last.GetAttribute(TfToken("inputs:enabled")).Set(false);
    check(11);
    last.GetAttribute(TfToken("inputs:enabled")).Set(true);
    check(15);
    last.GetAttribute(TfToken("inputs:translationOffset"))
        .Set(GfVec3d(std::numeric_limits<double>::quiet_NaN(), 0, 0));
    pose = check(11);
    CHECK(std::any_of(pose.diagnostics.begin(), pose.diagnostics.end(),
        [](const std::string &d) { return d.find("MoverFailed /Asset/Rig/Movers/Last") != std::string::npos; }));
    last.GetAttribute(TfToken("inputs:translationOffset")).Set(GfVec3d(0));
    check(15);
}

static void TestAnimatedPointCounts()
{
    auto stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const auto control = stage->DefinePrim(
        SdfPath("/Asset/Rig/C"), TfToken("RigExecControl"));
    control.GetAttribute(TfToken("avars:tx")).Set(2.0);
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    for (const char *name : {"Changing", "Fixed"}) {
        const SdfPath target(std::string("/Asset/") + name);
        const auto points = stage->DefinePrim(target, TfToken("Points"))
            .GetAttribute(TfToken("points"));
        points.Set(VtVec3fArray{GfVec3f(0)}, UsdTimeCode(1));
        if (std::string(name) == "Changing") {
            points.Set(VtVec3fArray{GfVec3f(10, 0, 0), GfVec3f(20, 0, 0)},
                       UsdTimeCode(2));
        }
        const auto mover = stage->DefinePrim(
            SdfPath(std::string("/Asset/Rig/Movers/") + name),
            TfToken("RigExecMatrixMover"));
        mover.ApplyAPI(TfToken("RigExecMoverAPI"));
        mover.GetRelationship(TfToken("rigExec:moves"))
            .SetTargets({target.AppendProperty(TfToken("points"))});
        mover.GetRelationship(TfToken("rigExec:transform"))
            .SetTargets({control.GetPath()});
    }
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    CHECK(evaluator.Compile());
    const size_t epoch = evaluator.GetBindingEpochDigest();
    auto pose = evaluator.Evaluate(UsdTimeCode(1));
    CHECK(pose.valid && pose.moverGraphRevisionsCreated == 2);
    for (double time : {2.0, 1.0, 2.0}) {
        pose = evaluator.Evaluate(UsdTimeCode(time));
        CHECK(pose.valid);
        CHECK(pose.moverGraphRevisionsCreated == 1);
        CHECK(pose.moverGraphRevisionsExecuted == 1);
        CHECK(pose.moverGraphSchedulesBuilt == 1);
        CHECK(evaluator.GetBindingEpochDigest() == epoch);
        const auto result = pose.movedProperties.at(SdfPath("/Asset/Changing.points"))
            .Get<VtVec3fArray>();
        CHECK(result.size() == (time == 1.0 ? 1 : 2));
        CHECK(!result.empty() && result.front()[0] == (time == 1.0 ? 2.0f : 12.0f));
    }
}

// Uncommitted manipulation values change the generation and author nothing
// (docs/superpowers/specs/2026-09-10-hydra-preview-manipulation-design.md).
//
// The fixture is the ordering fixture's arithmetic, reused because it makes
// every candidate behaviour a different number: dial = 2, multiplied by 10,
// then 1 added, published as the chain's result at /Asset/Rig/Channels
// .rigExec:dial. A mover also moves a point by a control avar, so one rig
// covers both what exec computes and what the property chains compute.
static const char *kPreviewFixture = R"USDA(#usda 1.0
(
)

def Scope "Asset"
{
    def RigExecRoot "Rig"
    {
        def Scope "Channels"
        {
            float rigExec:dial = 2
        }

        def RigExecControl "Driver"
        {
            double avars:tx = 1
        }

        def Scope "Movers"
        {
            reorder nameChildren = ["AddOne", "TimesTen"]

            def RigExecFloatMathMover "TimesTen" (
                prepend apiSchemas = ["RigExecMoverAPI"]
            )
            {
                uniform token rigExec:operation = "multiply"
                float inputs:value = 10
                rel rigExec:moves = </Asset/Rig/Channels.rigExec:dial>
            }

            def RigExecFloatMathMover "AddOne" (
                prepend apiSchemas = ["RigExecMoverAPI"]
            )
            {
                uniform token rigExec:operation = "add"
                float inputs:value = 1
                rel rigExec:moves = </Asset/Rig/Channels.rigExec:dial>
            }

            def RigExecMatrixMover "MoveShape" (
                prepend apiSchemas = ["RigExecMoverAPI"]
            )
            {
                float inputs:defaultWeight = 1
                rel rigExec:moves = </Asset/Shape.points>
                rel rigExec:transform = </Asset/Rig/Driver>
            }
        }
    }

    def Points "Shape"
    {
        point3f[] points = [(0, 0, 0)]
    }
}
)USDA";

static void TestInteractiveOverrides()
{
    const SdfLayerRefPtr layer = SdfLayer::CreateAnonymous(".usda");
    CHECK(layer);
    if (!layer || !layer->ImportFromString(kPreviewFixture)) {
        std::printf("  could not build the preview fixture\n");
        ++failures;
        return;
    }
    const UsdStageRefPtr stage = UsdStage::Open(layer);
    CHECK(stage);
    if (!stage) {
        return;
    }
    const SdfPath rigPath("/Asset/Rig");
    const SdfPath dial("/Asset/Rig/Channels.rigExec:dial");
    const SdfPath shapePoints("/Asset/Shape.points");
    const SdfPath driver("/Asset/Rig/Driver");
    const SdfPath timesTen("/Asset/Rig/Movers/TimesTen");

    RigExecRigEvaluator evaluator(stage, rigPath);
    std::vector<std::string> errors;
    if (!evaluator.Compile(&errors)) {
        for (const std::string &e : errors) {
            std::printf("    %s\n", e.c_str());
        }
    }
    CHECK(errors.empty());

    auto dialValue = [&](const RigExecRigPose &pose, float *out) {
        const auto it = pose.movedProperties.find(dial);
        if (it == pose.movedProperties.end() ||
            !it->second.IsHolding<float>()) {
            return false;
        }
        *out = it->second.Get<float>();
        return true;
    };
    auto pointX = [&](const RigExecRigPose &pose, float *out) {
        const auto it = pose.movedProperties.find(shapePoints);
        if (it == pose.movedProperties.end()) {
            return false;
        }
        const VtVec3fArray points = it->second.Get<VtVec3fArray>();
        if (points.size() != 1) {
            return false;
        }
        *out = points[0][0];
        return true;
    };

    // The authored rig, for every later comparison to be against.
    auto exported = [&]() {
        std::string text;
        layer->ExportToString(&text);
        return text;
    };
    const std::string authored = exported();
    CHECK(!authored.empty());
    float value = 0, x = 0;
    RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(pose.valid);
    CHECK(dialValue(pose, &value) && std::abs(value - 21.0f) < 1e-6f);
    CHECK(pointX(pose, &x) && std::abs(x - 1.0f) < 1e-6f);

    // 1. An override on an avar EXEC reads: the point follows it, and the
    //    authored avar does not move.
    evaluator.SetInteractiveOverrides({RigExecValueOverride{
        driver, TfToken(), TfToken("avars:tx"), VtValue(4.0)}});
    CHECK(evaluator.HasInteractiveOverrides());
    pose = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(pose.valid);
    CHECK(pointX(pose, &x) && std::abs(x - 4.0f) < 1e-6f);
    double authoredTx = 0;
    CHECK(stage->GetAttributeAtPath(driver.AppendProperty(TfToken("avars:tx")))
              .Get(&authoredTx) &&
          std::abs(authoredTx - 1.0) < 1e-12);
    // The whole point of the exercise: the document is untouched. Compared as
    // text rather than by probing the one attribute, so a spec authored
    // anywhere else -- a different layer arm, a sibling property -- fails too.
    CHECK(exported() == authored);

    // 2. Evaluating twice with the same override is stable: an override is a
    //    value, not an increment.
    pose = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(pointX(pose, &x) && std::abs(x - 4.0f) < 1e-6f);

    // 3. An override on a value a property CHAIN reads: the chain computes
    //    from the override, so TimesTen multiplying by 3 gives (2*3)+1.
    evaluator.SetInteractiveOverrides({RigExecValueOverride{
        timesTen, TfToken(), TfToken("inputs:value"), VtValue(3.0f)}});
    pose = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(dialValue(pose, &value) && std::abs(value - 7.0f) < 1e-6f);
    // Setting replaces rather than accumulates: the avar override from step 1
    // is gone, so the point is back at its authored 1.
    CHECK(pointX(pose, &x) && std::abs(x - 1.0f) < 1e-6f);
    CHECK(exported() == authored);

    // 4. An override on the property a chain WRITES outranks the chain: the
    //    held value is what the generation carries, not 21 and not 51.
    evaluator.SetInteractiveOverrides({RigExecValueOverride{
        dial.GetPrimPath(), TfToken(), dial.GetNameToken(), VtValue(5.0f)}});
    pose = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(dialValue(pose, &value) && std::abs(value - 5.0f) < 1e-6f);
    CHECK(exported() == authored);

    // 5. Two overrides at once, on both kinds of consumer.
    evaluator.SetInteractiveOverrides({
        RigExecValueOverride{driver, TfToken(), TfToken("avars:tx"),
                             VtValue(-2.0)},
        RigExecValueOverride{timesTen, TfToken(), TfToken("inputs:value"),
                             VtValue(4.0f)}});
    pose = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(pointX(pose, &x) && std::abs(x + 2.0f) < 1e-6f);
    CHECK(dialValue(pose, &value) && std::abs(value - 9.0f) < 1e-6f);

    // 6. Releasing the drag: the authored rig is back, with nothing to undo.
    evaluator.ClearInteractiveOverrides();
    CHECK(!evaluator.HasInteractiveOverrides());
    pose = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(pointX(pose, &x) && std::abs(x - 1.0f) < 1e-6f);
    CHECK(dialValue(pose, &value) && std::abs(value - 21.0f) < 1e-6f);
    CHECK(exported() == authored);
}

int main()
{
    PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);
    TestPersistentChains();
    TestStructuralSplices();
    TestBlendSampleReadPhases();
    TestCurvenetWeightsUpdate();
    TestBlendSurfaceFrames();
    TestGeometryConstraintsCompose();
    TestAnimatedPointCounts();
    TestInteractiveOverrides();
    std::printf("testRigExecInteractive: %d failure(s)\n", failures);
    return failures ? 1 : 0;
}
