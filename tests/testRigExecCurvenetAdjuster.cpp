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

int main() {
    PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);
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
