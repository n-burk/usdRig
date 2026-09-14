#include "rigExec/rigEvaluator.h"
#include "rigExecRigging/rigBuilder.h"
#include "pxr/base/plug/registry.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usdGeom/xformable.h"
#include <cmath>
#include <cstdio>

using namespace rigExec;
static int failures=0;
#define CHECK(x) do { if (!(x)) { ++failures; std::printf("FAIL %d: %s\n",__LINE__,#x); } } while(0)
static bool Near(const GfVec3f &a,const GfVec3f &b) { return (a-b).GetLength()<2e-4; }

// The rig both curvenet-weight tests stand on: a two-quad mesh, a four-point
// net across it, and a matrix mover that moves the mesh by the field.
static UsdStageRefPtr MakeACurvenetWeightRig() {
    const auto stage=UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"),TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"),TfToken("RigExecRoot"));
    const auto mesh=stage->DefinePrim(SdfPath("/Asset/Mesh"),TfToken("Mesh"));
    // Two quads, so the field has somewhere to fall off across the surface
    // rather than resolving to one value everywhere.
    mesh.GetAttribute(TfToken("points")).Set(VtVec3fArray{
        {0,0,0},{1,0,0},{2,0,0},{0,1,0},{1,1,0},{2,1,0}});
    mesh.GetAttribute(TfToken("faceVertexCounts")).Set(VtIntArray{4,4});
    mesh.GetAttribute(TfToken("faceVertexIndices")).Set(
        VtIntArray{0,1,4,3, 1,2,5,4});
    const auto net=stage->DefinePrim(SdfPath("/Asset/Net"),
        TfToken("RigExecCurvenet"));
    net.GetAttribute(TfToken("points")).Set(VtVec3fArray{
        {0,0.5f,0},{0.67f,0.5f,0},{1.33f,0.5f,0},{2,0.5f,0}});
    net.GetAttribute(TfToken("rigExec:splineIndices")).Set(VtIntArray{0,1,2,3});

    const auto weight=stage->DefinePrim(SdfPath("/Asset/Rig/Weights/Net"),
        TfToken("RigExecCurvenetWeight"));
    const auto target=mesh.GetPath().AppendProperty(TfToken("points"));
    weight.GetRelationship(TfToken("rigExec:weightTarget")).SetTargets({target});
    weight.GetRelationship(TfToken("rigExec:curvenetPoints")).SetTargets(
        {net.GetPath().AppendProperty(TfToken("points"))});
    weight.GetRelationship(TfToken("rigExec:curvenetSplineIndices")).SetTargets(
        {net.GetPath().AppendProperty(TfToken("rigExec:splineIndices"))});
    weight.GetRelationship(TfToken("rigExec:meshFaceCounts")).SetTargets(
        {mesh.GetPath().AppendProperty(TfToken("faceVertexCounts"))});
    weight.GetRelationship(TfToken("rigExec:meshFaceIndices")).SetTargets(
        {mesh.GetPath().AppendProperty(TfToken("faceVertexIndices"))});
    // A gradient along the net, so the solved field is not constant and a
    // program that resolved nothing would land somewhere visibly else.
    weight.GetAttribute(TfToken("inputs:weights")).Set(
        VtFloatArray{1.0f,0.75f,0.25f,0.0f});

    const auto driver=stage->DefinePrim(SdfPath("/Asset/Rig/Controls/Push"),
        TfToken("RigExecControl"));
    driver.GetAttribute(TfToken("avars:tz")).Set(4.0);
    const auto mover=stage->DefinePrim(SdfPath("/Asset/Rig/Movers/M"),
        TfToken("RigExecMatrixMover"));
    mover.ApplyAPI(TfToken("RigExecMoverAPI"));
    mover.GetRelationship(TfToken("rigExec:moves")).SetTargets({target});
    mover.GetRelationship(TfToken("rigExec:transform")).SetTargets(
        {driver.GetPath()});
    mover.GetRelationship(TfToken("rigExec:weightObject")).SetTargets(
        {weight.GetPath()});

    return stage;
}

// A RigExecCurvenetWeight driving a matrix mover, in the BAKED program.
//
// The sixth weight-object schema, and the one the program left out: its
// field is not a formula over a few floats but the solution of a factorized
// Laplacian over the cut mesh, so a packet cannot be built without the BIND
// that factorization lives in. The exec computation holds one in a static
// LRU behind a mutex; a step body may take no lock, so the program resolves
// its own -- in the prologue, into program-owned state -- and the step then
// evaluates the same right-hand side through the same kernel.
//
// cpuParityMode is deliberately OFF: it asks for the dynamic path, so an
// evaluator that pinned it would never build a program at all. Under
// RIGEXEC_EVALUATION_MODE=parity this generation runs both paths over one
// rig and compares them exactly.
static void TestCurvenetWeightObject() {
    const auto stage=MakeACurvenetWeightRig();
    const auto weight=stage->GetPrimAtPath(
        SdfPath("/Asset/Rig/Weights/Net"));
    const SdfPath target("/Asset/Mesh.points");
    RigExecRigEvaluator evaluator(stage,SdfPath("/Asset/Rig"));
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    for (const auto &e:errors) std::printf("curvenet weight compile: %s\n",
        e.c_str());
    std::vector<std::string> reasons;
    if (!evaluator.IsBakeable(&reasons)) {
        ++failures;
        std::printf("FAIL curvenet weight object: not bakeable\n");
        for (const auto &r:reasons) std::printf("    %s\n",r.c_str());
        return;
    }
    const auto pose=evaluator.Evaluate(UsdTimeCode(1));
    CHECK(pose.valid && pose.bakedParityMismatches==0);
    if (!pose.valid || !pose.movedProperties.count(target)) { ++failures; return; }
    const auto points=pose.movedProperties.at(target).Get<VtVec3fArray>();
    CHECK(points.size()==6);
    if (points.size()!=6) return;
    // The field is a real gradient: fully on at the net's first control
    // point, off at its last, and strictly between them in the middle. An
    // all-ones or all-zeros field would satisfy nothing below.
    CHECK(std::abs(points[0][2]-4.0f)<1e-3f);
    CHECK(std::abs(points[2][2])<1e-3f);
    CHECK(points[1][2]>0.01f && points[1][2]<3.99f);
    // And an ANIMATED weight moves it without rebuilding the bind, which is
    // what separates the bind from the right-hand side.
    weight.GetAttribute(TfToken("inputs:weights")).Set(
        VtFloatArray{0.5f,0.375f,0.125f,0.0f});
    const auto halved=evaluator.Evaluate(UsdTimeCode(1));
    CHECK(halved.valid && halved.bakedParityMismatches==0);
    if (!halved.valid || !halved.movedProperties.count(target)) return;
    const auto after=halved.movedProperties.at(target).Get<VtVec3fArray>();
    CHECK(after.size()==6);
    // Strictly less and still there. NOT half: the solved field is clamped
    // into [0,1] before it becomes a packet, so halving the control values
    // is not a scaling of the answer -- which is exactly why the
    // right-hand side has to be re-solved rather than scaled.
    if (after.size()==6) {
        CHECK(after[0][2]>0.01f && after[0][2]<points[0][2]-0.01f);
    }
}

// A rebuild carries the BIND. Editing the rig's structure ends the epoch and
// builds a second program; the cut mesh and its factorized Laplacian did not
// move, so AdoptGeometryStateFrom hands them over -- and the step compares
// the layout it inherited against the frame's own before using it, so the
// answer has to be the dynamic path's either way. That is what this asserts:
// the carry is a cost decision, and a wrong one would show up here as a
// wrong field rather than as a slow one.
static void TestARebuildKeepsTheCurvenetBind() {
    const auto stage=MakeACurvenetWeightRig();
    const SdfPath target("/Asset/Mesh.points");
    RigExecRigEvaluator rig(stage,SdfPath("/Asset/Rig"));
    rig.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::vector<std::string> errors;
    CHECK(rig.Compile(&errors));
    CHECK(rig.Evaluate(UsdTimeCode(1)).valid);
    const size_t builds=rig.GetBakedProgramBuildCount();

    // Structural, and unrelated to the weight: a control nothing drives.
    stage->DefinePrim(SdfPath("/Asset/Rig/Controls/Spare"),
        TfToken("RigExecControl"));
    const auto after=rig.Evaluate(UsdTimeCode(1));
    CHECK(after.valid);
    if (rig.GetBakedProgramBuildCount()<=builds) {
        ++failures;
        std::printf("FAIL a rebuild after a structural edit: the program was "
            "not rebuilt, so the carry was never exercised\n");
    }
    // Still the program's own generation, and still the dynamic answer.
    CHECK(rig.GetBakedGenerationCount()==2);
    const auto referenceStage=MakeACurvenetWeightRig();
    referenceStage->DefinePrim(SdfPath("/Asset/Rig/Controls/Spare"),
        TfToken("RigExecControl"));
    RigExecRigEvaluator reference(referenceStage,SdfPath("/Asset/Rig"));
    reference.SetEvaluationMode(RigExecEvaluationMode::Dynamic);
    CHECK(reference.Compile(&errors));
    const auto expected=reference.Evaluate(UsdTimeCode(1));
    CHECK(expected.valid);
    if (expected.movedProperties.count(target) &&
        after.movedProperties.count(target)) {
        CHECK(expected.movedProperties.at(target)==
              after.movedProperties.at(target));
    } else {
        ++failures;
    }
}

int main() {
    PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);
    TestCurvenetWeightObject();
    TestARebuildKeepsTheCurvenetBind();
    const auto stage=UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"),TfToken("Scope"));
    auto builder=RigExecRigBuilder::Create(stage,SdfPath("/Asset/Rig"));
    const std::vector<GfVec3f> rest{
        {0,0,0},{1,0,0},{2,0,0},{3,0,0},
        {0,1,0},{0,2,0},{0,3,0}, {0,0,1},{0,0,2},{0,0,3}};
    auto net=builder.AddCurvenet("Net",rest);
    const GfMatrix4d netToAsset=GfMatrix4d(1).SetTranslate(GfVec3d(10,20,30));
    UsdGeomXformable(net.GetPrim()).AddTransformOp().Set(netToAsset);
    net.AddSpline(0,1,2,3); net.AddSpline(0,4,5,6); net.AddSpline(0,7,8,9);
    auto knot=builder.AddCurvenetAdjustment("Knot",net.GetPath(),0);
    knot.SetAvarTranslation(1,0,0);
    auto tangent=knot.AddTangent("Tangent",1);
    tangent.SetAvarTranslation(2,0,0);
    auto warp=builder.AddControl("Warp");
    warp.SetAvarRotation(0,0,90);
    warp.SetAvarTranslation(5,6,7);
    const auto target=net.GetPath().AppendProperty(TfToken("points"));
    auto chain=builder.NewMoverChain("Shape",target);
    auto adjuster=chain.AddCurvenetAdjusterMover("Adjust",{knot.GetPath()});
    chain.AddMatrixMover("Warp",warp.GetPath());
    RigExecRigEvaluator evaluator(stage,builder.GetRootPath());
    evaluator.cpuParityMode=true;
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    for (const auto &e:errors) std::printf("compile: %s\n",e.c_str());
    auto pose=evaluator.Evaluate(UsdTimeCode(1));
    CHECK(pose.valid && pose.movedProperties.count(target));
    if (!pose.valid || !pose.movedProperties.count(target)) return 1;
    const auto points=[&]() { return pose.movedProperties.at(target).Get<VtVec3fArray>(); };
    CHECK(Near(points()[0],GfVec3f(5,7,7)));
    CHECK(Near(points()[1],GfVec3f(5,10,7)));
    CHECK(pose.controlFrames.count(knot.GetPath()) && pose.controlFrames.count(tangent.GetPath()));
    CHECK(Near(GfVec3f(pose.controlFrames.at(knot.GetPath()).Origin()),
        GfVec3f(netToAsset.Transform(GfVec3d(points()[0])))));
    CHECK(Near(GfVec3f(pose.controlFrames.at(tangent.GetPath()).Origin()),
        GfVec3f(netToAsset.Transform(GfVec3d(points()[1])))));
    CHECK(pose.moverGraphParityMismatches==0);

    // The same rig once more with the CPU oracle OFF, because cpuParityMode
    // does not merely add an oracle -- it ASKS for the dynamic path, so the
    // evaluator above never builds a baked program and the adjuster's bake
    // has no coverage anywhere else. Under RIGEXEC_EVALUATION_MODE=parity
    // this generation runs both paths over one rig whose curvenet carries an
    // xform op, which is what exercises the net -> asset ladder the control
    // frames are published through.
    {
        RigExecRigEvaluator bakedEvaluator(stage,builder.GetRootPath());
        std::vector<std::string> bakedErrors;
        CHECK(bakedEvaluator.Compile(&bakedErrors));
        const auto bakedPose=bakedEvaluator.Evaluate(UsdTimeCode(1));
        CHECK(bakedPose.valid && bakedPose.bakedParityMismatches==0);
        CHECK(bakedPose.movedProperties.count(target));
        CHECK(bakedPose.controlFrames.count(knot.GetPath()) &&
            bakedPose.controlFrames.count(tangent.GetPath()));
    }

    pose=evaluator.Evaluate(UsdTimeCode(1));
    CHECK(pose.valid && pose.moverGraphRevisionsExecuted==0);
    CHECK(pose.controlFrames.count(knot.GetPath()));
    knot.SetAvarTranslation(2,0,0);
    pose=evaluator.Evaluate(UsdTimeCode(1));
    CHECK(pose.valid && pose.moverGraphRevisionsCreated==0);
    CHECK(pose.moverGraphRevisionsExecuted==1 && pose.moverGraphSchedulesBuilt==0);
    CHECK(Near(points()[0],GfVec3f(5,8,7)));
    CHECK(Near(points()[1],GfVec3f(5,11,7)));
    CHECK(pose.moverGraphParityMismatches==0);

    warp.SetAvarRotation(0,0,180);
    pose=evaluator.Evaluate(UsdTimeCode(1));
    CHECK(Near(points()[0],GfVec3f(3,6,7)));
    CHECK(Near(points()[1],GfVec3f(0,6,7)));
    CHECK(pose.moverGraphRevisionsExecuted==2);

    adjuster.SetDefaultWeight(0.5f);
    pose=evaluator.Evaluate(UsdTimeCode(1));
    CHECK(Near(points()[0],GfVec3f(4,6,7)));
    CHECK(Near(GfVec3f(pose.controlFrames.at(knot.GetPath()).Origin()),
        GfVec3f(netToAsset.Transform(GfVec3d(points()[0])))));
    CHECK(pose.moverGraphRevisionsExecuted==1 && pose.moverGraphParityMismatches==0);
    adjuster.SetEnabled(false);
    pose=evaluator.Evaluate(UsdTimeCode(1));
    CHECK(Near(points()[0],GfVec3f(5,6,7)));
    CHECK(Near(points()[1],GfVec3f(4,6,7)));
    adjuster.SetEnabled(true);
    adjuster.SetDefaultWeight(1);

    // Rest-pool edits remain live and replace neither operation's graph node.
    auto editedRest=rest; editedRest[1]=GfVec3f(2,0,0);
    net.SetPoints(editedRest);
    pose=evaluator.Evaluate(UsdTimeCode(1));
    CHECK(pose.valid && pose.moverGraphRevisionsCreated==0);
    CHECK(Near(points()[1],GfVec3f(-1,6,7)));

    // Animated controls scrub backward on the retained adjuster revision.
    tangent.GetPrim().GetAttribute(TfToken("avars:tx")).Set(2.0,UsdTimeCode(1));
    tangent.GetPrim().GetAttribute(TfToken("avars:tx")).Set(4.0,UsdTimeCode(2));
    for (double time:{2.0,1.0,2.0}) {
        pose=evaluator.Evaluate(UsdTimeCode(time));
        CHECK(pose.valid && pose.moverGraphRevisionsCreated==0);
        CHECK(Near(points()[1],GfVec3f(time==2 ? -3 : -1,6,7)));
    }

    // Compilation diagnoses a malformed raw edit instead of accepting a stub.
    knot.GetPrim().GetAttribute(TfToken("rigExec:knotIndex")).Set(999);
    errors.clear();
    CHECK(!evaluator.Compile(&errors) && !errors.empty());
    knot.SetKnotIndex(0);
    CHECK(evaluator.Compile());
    const auto rejectsAdjustmentRead = [&]() {
        errors.clear();
        CHECK(!evaluator.Compile(&errors));
        bool diagnosed=false;
        for (const auto &error:errors)
            diagnosed=diagnosed || error.find("Adjustment frames are point-graph outputs")!=std::string::npos;
        CHECK(diagnosed);
    };
    // Point-output control frames cannot silently feed earlier pose taps.
    auto matrix=chain.AddMatrixMover("BadFrame",warp.GetPath());
    matrix.SetTransformProvider(knot.GetPath());
    rejectsAdjustmentRead();
    matrix.SetTransformProvider(warp.GetPath());
    CHECK(evaluator.Compile());
    const auto inherited=stage->DefinePrim(knot.GetPath().AppendChild(TfToken("Inherited")),
        TfToken("RigExecControl"));
    rejectsAdjustmentRead();
    stage->RemovePrim(inherited.GetPath());
    const auto solver=stage->DefinePrim(SdfPath("/Asset/Rig/BadSolver"),TfToken("RigExecFkChain"));
    solver.GetRelationship(TfToken("rigExec:controls")).SetTargets({knot.GetPath()});
    rejectsAdjustmentRead();
    stage->RemovePrim(solver.GetPath());
    const auto constraint=stage->DefinePrim(SdfPath("/Asset/Rig/BadConstraint"),
        TfToken("RigExecPositionConstraint"));
    constraint.GetRelationship(TfToken("rigExec:sources")).SetTargets({knot.GetPath()});
    rejectsAdjustmentRead();
    stage->RemovePrim(constraint.GetPath());

    // A two-hop external connection can expose an Adjustment through an
    // ordinary control's computed default-space namespace fallback.
    stage->DefinePrim(SdfPath("/External/Knot"),TfToken("RigExecCurvenetAdjustment"));
    const auto external=stage->DefinePrim(SdfPath("/External/Knot/Carrier"),TfToken("RigExecControl"));
    const auto relay=stage->DefinePrim(SdfPath("/External/Relay"),TfToken("Scope"))
        .CreateAttribute(TfToken("matrix"),SdfValueTypeNames->Matrix4d);
    relay.SetConnections({external.GetAttribute(TfToken("posed:defaultSpace")).GetPath()});
    auto consumer=builder.AddControl("Consumer");
    consumer.GetPrim().GetAttribute(TfToken("parent:space")).SetConnections({relay.GetPath()});
    rejectsAdjustmentRead();
    relay.SetConnections({warp.GetPrim().GetAttribute(TfToken("default:space")).GetPath()});
    CHECK(evaluator.Compile());
    consumer.GetPrim().GetAttribute(TfToken("avars:tx")).SetConnections(
        {knot.GetPrim().GetAttribute(TfToken("avars:tx")).GetPath()});
    CHECK(evaluator.Compile()); // ordinary scalar channel reads stay legal
    CHECK(evaluator.Evaluate(UsdTimeCode(1)).valid);
    std::printf("testRigExecCurvenetAdjuster: %d failure(s)\n",failures);
    return failures ? 1 : 0;
}
