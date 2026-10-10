#include "pxr/base/ts/spline.h"
#include "rigExecStandalone/system.h"
#include "rigExecStandalone/sceneAccess.h"
#include "rigExecStandalone/graphFrontend.h"
#include "rigExecStandalone/providerRuntime.h"
#include "rigExecStandalone/sceneRuntime.h"
#include "rigExecGraph/providerRecordExport.h"
#include "rigExecGraph/providerContextBinding.h"
#include "rigExecGraph/sceneCompileInputs.h"
#include "rigExecGraph/solverSceneLowering.h"
#include "rigExecGraph/propertySceneLowering.h"
#include "pxr/usd/sdf/attributeSpec.h"
#include "rigExec/types.h"
#include "rigExecRigging/rigBuilder.h"
#include "pxr/usd/usdGeom/mesh.h"
#include "pxr/usd/usdGeom/xform.h"
#include "pxr/usd/usdGeom/xformOp.h"
#include "rigExec/frameExtraction.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/errorMark.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/resolveInfo.h"
#include "pxr/usd/usdGeom/metrics.h"
#include <cstdio>
#include <cmath>
#include <functional>
#include <algorithm>

using namespace rigExec;
static int failures = 0;
#define CHECK(condition) do { if (!(condition)) { ++failures; std::printf("FAIL %d: %s\n", __LINE__, #condition); } } while (0)

static VtDictionary Metadata(const UsdObject &object) {
    VtDictionary result;
    for (const auto &[key, value] : object.GetAllMetadata()) result[key.GetString()] = value;
    return result;
}

// Test-only producer. Production pack export independently traverses USD and
// serializes the same public Db contract; the runtime sees only these rows.
static RigExecSceneDb Capture(const UsdStageRefPtr &stage, const std::vector<UsdTimeCode> &times)
{
    RigExecSceneDb db;
    db.upAxis=UsdGeomGetStageUpAxis(stage);
    for (const auto time : times) db.identities.insert(RigExecStandaloneTimeKey(time));
    db.prims[SdfPath::AbsoluteRootPath()] = {};
    for (const auto &child : stage->GetPseudoRoot().GetAllChildren())
        db.prims[SdfPath::AbsoluteRootPath()].children.push_back(child.GetPath());
    for (const UsdPrim &prim : stage->Traverse()) {
        db.prims[prim.GetPath()] = {prim.GetTypeName(), prim.GetAppliedSchemas(), Metadata(prim), true};
        for (const auto &child : prim.GetAllChildren()) db.prims[prim.GetPath()].children.push_back(child.GetPath());
        for (const auto &attribute : prim.GetAttributes()) {
            RigExecStandaloneAttribute row;
            row.type = attribute.GetTypeName();
            row.metadata = Metadata(attribute);
            if(attribute.HasSpline())row.spline=VtValue(attribute.GetSpline());
            attribute.GetConnections(&row.connections);
            row.hasValue = attribute.HasValue();
            row.variability = attribute.GetVariability();
            row.hasAuthoredValue = attribute.HasAuthoredValueOpinion();
            row.hasAuthoredReadableValue=attribute.HasAuthoredValue();
            row.hasAuthoredConnections = attribute.HasAuthoredConnections();
            row.mightBeTimeVarying = attribute.ValueMightBeTimeVarying();
            attribute.GetTimeSamples(&row.sampleTimes);
            for (const auto &spec : attribute.GetPropertyStack(UsdTimeCode::Default())) {
                if (!spec->HasInfo(TfToken("default"))) continue;
                row.hasAuthoredDefault = true;
                row.authoredDefault=spec->GetInfo(TfToken("default"));
                row.defaultBlocked = row.authoredDefault.IsHolding<SdfValueBlock>();
                break;
            }
            for (const auto time : times) {
                VtValue value;
                attribute.Get(&value, time);
                if (attribute.GetResolveInfo(time).ValueIsBlocked()) row.blockedIdentities.insert(RigExecStandaloneTimeKey(time));
                row.resolved[RigExecStandaloneTimeKey(time)] = value;
            }
            db.attributes[attribute.GetPath()] = row;
        }
        for (const auto &relationship : prim.GetRelationships()) {
            RigExecStandaloneRelationship row;
            row.metadata = Metadata(relationship);
            relationship.GetTargets(&row.targets);
            db.relationships[relationship.GetPath()] = row;
        }
    }
    std::string error;
    CHECK(db.Validate(&error));
    if (!error.empty()) std::printf("Db: %s\n", error.c_str());
    return db;
}
static bool SameFrame(const RigExecPointFrame &a, const RigExecPointFrame &b)
{
    if (a.IsValid() != b.IsValid() || a.IsDegenerate() != b.IsDegenerate()) return false;
    for (size_t i = 0; i < a.points.size(); ++i) if ((a.points[i] - b.points[i]).GetLength() > 1e-9) return false;
    return true;
}
static bool Same(const VtValue &a, const VtValue &b)
{
    if (a.IsHolding<RigExecPointFrame>() && b.IsHolding<RigExecPointFrame>())
        return SameFrame(a.UncheckedGet<RigExecPointFrame>(), b.UncheckedGet<RigExecPointFrame>());
    if (a.IsHolding<RigExecPointFrameArray>() && b.IsHolding<RigExecPointFrameArray>()) {
        const auto &x = a.UncheckedGet<RigExecPointFrameArray>(), &y = b.UncheckedGet<RigExecPointFrameArray>();
        if (x.frames.size() != y.frames.size() || x.rests != y.rests) return false;
        for (size_t i = 0; i < x.frames.size(); ++i) if (!SameFrame(x.frames[i], y.frames[i])) return false;
        return true;
    }
    return a == b;
}
static void TestRequestBoundaries(const RigExecSceneDb &db)
{
    auto invalidSampleDb = db;
    auto &sampleAttribute = invalidSampleDb.attributes.begin()->second;
    sampleAttribute.hasAuthoredValue = true;
    sampleAttribute.sampleTimes = {2.0, 1.0};
    CHECK(!invalidSampleDb.Validate());
    auto invalidBlockDb = db;
    invalidBlockDb.attributes.begin()->second.defaultBlocked = true;
    invalidBlockDb.attributes.begin()->second.hasAuthoredDefault = false;
    CHECK(!invalidBlockDb.Validate());
    RigExecSceneDbAccess access(db);
    for (const auto &[path, attribute] : db.attributes) {
        RigExecSceneAttribute descriptor;
        CHECK(access.Attribute(path, &descriptor));
        CHECK(descriptor.sampleTimes == attribute.sampleTimes);
        CHECK(descriptor.hasAuthoredValue == attribute.hasAuthoredValue);
        CHECK(descriptor.hasAuthoredReadableValue==attribute.hasAuthoredReadableValue);
        VtValue state;
        CHECK(access.Resolve(path, UsdTimeCode::Default(), &state));
        CHECK(state == attribute.resolved.at("default"));
        CHECK(!access.Resolve(path, UsdTimeCode(123456789.0), &state));
    }
    auto wrongConnectionDb = db;
    wrongConnectionDb.attributes[SdfPath("/Blend.inputs:weight")].connections = {SdfPath("/Unused.value")};
    CHECK(wrongConnectionDb.Validate());
    RigExecStandaloneSystem doubleTail(wrongConnectionDb);
    const int weightTap=doubleTail.AddTap(RigExecValueAddress::Property(SdfPath("/Blend.inputs:weight")));
    const auto doubleWeight=doubleTail.Evaluate(UsdTimeCode::Default());
    CHECK(doubleWeight.valid && doubleWeight.Get<float>(weightTap)==1.0f);
    auto missingSourceDb=db;
    missingSourceDb.attributes[SdfPath("/Blend.inputs:weight")].connections={SdfPath("/Missing.value")};
    CHECK(!missingSourceDb.Validate());
    auto multipleConnectionDb = db;
    multipleConnectionDb.attributes[SdfPath("/Blend.inputs:weight")].connections = {SdfPath("/Driver.w1"), SdfPath("/Driver.w2")};
    CHECK(!multipleConnectionDb.Validate());
    // Final-phase blend sample reads compile through actual typed producer routes.
    auto phasedDb = db;
    phasedDb.relationships[SdfPath("/Sample.rigExec:targetPoints")].metadata["rigExecReadPhase"] = VtValue(std::string("final"));
    CHECK(phasedDb.Validate());
    CHECK(phasedDb.ValidateCapabilities());
    RigExecStandaloneSystem empty(db);
    CHECK(empty.Prepare());
    const auto first = empty.Evaluate(UsdTimeCode::Default());
    const auto second = empty.Evaluate(UsdTimeCode::Default());
    CHECK(first.valid && first.values.empty() && first.generation == 1);
    CHECK(second.valid && second.generation == 2);
    CHECK(!empty.SetConnections(SdfPath("/Blend.inputs:weight"), {SdfPath("/Driver.w1"), SdfPath("/Driver.w2")}));
    RigExecStandaloneSystem missing(db);
    missing.AddTap(RigExecValueAddress::Prim(SdfPath("/A"), TfToken("unregisteredComputation")));
    const auto unavailable = missing.Evaluate(UsdTimeCode::Default());
    CHECK(!unavailable.valid && !unavailable.diagnostics.empty());
    RigExecStandaloneSystem blocked(db);
    blocked.AddTap(RigExecValueAddress::Property(SdfPath("/Unused.value")));
    CHECK(blocked.Evaluate(UsdTimeCode::Default()).Get<double>(0) == 1.0);
    CHECK(blocked.SetValue(SdfPath("/Unused.value"), UsdTimeCode::Default(), VtValue()));
    CHECK(!blocked.Evaluate(UsdTimeCode::Default()).valid);
    CHECK(blocked.SetValue(SdfPath("/Unused.value"), UsdTimeCode::Default(), VtValue(9.0)));
    const auto revived = blocked.Evaluate(UsdTimeCode::Default());
    CHECK(revived.valid && revived.Get<double>(0) == 9.0);
    for (const char *path : {"/A.avars:tx", "/A.userValue", "/Api.value"}) {
        RigExecStandaloneSystem raw(db);
        raw.AddTap(RigExecValueAddress::Property(SdfPath(path)));
        CHECK(raw.Evaluate(UsdTimeCode::Default()).valid);
        CHECK(raw.SetValue(SdfPath(path), UsdTimeCode::Default(), VtValue()));
        CHECK(!raw.Evaluate(UsdTimeCode::Default()).valid);
        CHECK(raw.SetValue(SdfPath(path), UsdTimeCode::Default(), VtValue(19.0)));
        const auto restored = raw.Evaluate(UsdTimeCode::Default());
        CHECK(restored.valid && restored.Get<double>(0) == 19.0);
    }
    auto unknownApi = db;
    unknownApi.prims.at(SdfPath("/Api")).appliedSchemas = {TfToken("UnqualifiedExpressionAPI")};
    RigExecStandaloneSystem unsupportedApi(unknownApi);
    CHECK(!unsupportedApi.Prepare());

    RigExecStandaloneSystem alias(db);
    alias.AddTap(RigExecValueAddress::Property(SdfPath("/Alias.value")));
    CHECK(alias.Evaluate(UsdTimeCode::Default()).Get<double>(0) == 1.0);
    CHECK(alias.SetPrimActive(SdfPath("/Unused"), false));
    const auto fallback = alias.Evaluate(UsdTimeCode::Default());
    CHECK(fallback.valid && fallback.Get<double>(0) == 4.0);
    CHECK(alias.SetValue(SdfPath("/Alias.value"), UsdTimeCode::Default(), VtValue()));
    CHECK(!alias.Evaluate(UsdTimeCode::Default()).valid);
    CHECK(alias.SetPrimActive(SdfPath("/Unused"), true));
    const auto connected = alias.Evaluate(UsdTimeCode::Default());
    CHECK(connected.valid && connected.Get<double>(0) == 1.0);

    RigExecStandaloneSystem expression(db);
    expression.AddTap(RigExecValueAddress::Property(SdfPath("/B.default:space")));
    CHECK(expression.SetConnections(SdfPath("/B.default:space"), {SdfPath("/Driver.matrix")}));
    CHECK(expression.SetValue(SdfPath("/B.default:space"), UsdTimeCode::Default(), VtValue()));
    const auto computed = expression.Evaluate(UsdTimeCode::Default());
    CHECK(computed.valid && computed.Get<GfMatrix4d>(0).ExtractTranslation() == GfVec3d(17, 0, 0));
    for (const char *type : {"RigExecUnregisteredSolver", "RigExecUnregisteredWeight",
                             "RigExecUnregisteredConstraint", "RigExecUnregisteredMover"}) {
        auto unsupportedDb = db;
        unsupportedDb.prims[SdfPath("/Unsupported")].type = TfToken(type);
        unsupportedDb.prims[SdfPath::AbsoluteRootPath()].children.push_back(SdfPath("/Unsupported"));
        RigExecStandaloneSystem unsupported(unsupportedDb);
        std::string error;
        CHECK(!unsupported.Prepare(&error));
        if(error.find(type)==std::string::npos)std::printf("unsupported %s diagnostic: %s\n",type,error.c_str());
        CHECK(error.find(type) != std::string::npos);
    }
}
static void TestSchemaIdentityLifetime(const RigExecSceneDb &db)
{
    const void *controlKey = db.SchemaKey(SdfPath("/A"));
    for (int i = 0; i < 8; ++i) {
        auto fresh = db;
        fresh.prims.at(SdfPath("/A")).type = TfToken(i % 2 ? "RigExecControl" : "RigExecJoint");
        CHECK(fresh.Validate());
        if (i % 2) CHECK(fresh.SchemaKey(SdfPath("/A")) == controlKey);
        // Each request and database is destroyed before the next iteration.
        // Registry keys must not alias a different schema when memory is reused.
        RigExecStandaloneSystem runtime(fresh);
        runtime.AddTap(RigExecValueAddress::Prim(SdfPath("/A"), TfToken("computePointFrame")));
        runtime.AddTap(RigExecValueAddress::Prim(SdfPath("/Channel"), TfToken("computeBlendChannel")));
        const auto values = runtime.Evaluate(UsdTimeCode::Default());
        CHECK(values.valid && values.Get<RigExecPointFrame>(0).IsValid());
        const auto channel = values.Get<RigExecBlendChannel>(1);
        CHECK(channel.samples.size() == 1 && channel.samples.front().points.size() == 2);
    }
}
static void TestSceneDescriptors()
{
    const auto stage = UsdStage::CreateInMemory();
    const auto rig = stage->DefinePrim(SdfPath("/Rig"), TfToken("RigExecRoot"));
    const auto joint = stage->DefinePrim(SdfPath("/Rig/Joint"), TfToken("RigExecJoint"));
    const auto solver = stage->DefinePrim(SdfPath("/Rig/Solver"), TfToken("RigExecFkChain"));
    solver.GetRelationship(TfToken("rigExec:joints")).SetTargets({joint.GetPath()});
    solver.GetRelationship(TfToken("rigExec:controls")).SetTargets({joint.GetPath()});
    const auto source = stage->DefinePrim(SdfPath("/Outside"));
    source.CreateAttribute(TfToken("value"), SdfValueTypeNames->Double).Set(7.0);
    const auto input = rig.CreateAttribute(TfToken("input"), SdfValueTypeNames->Double);
    input.Set(2.0);
    input.SetConnections({SdfPath("/Outside.value")});
    source.CreateAttribute(TfToken("matrix"),SdfValueTypeNames->Matrix4d).Set(GfMatrix4d(1.0));
    const auto parentSpace=joint.GetAttribute(TfToken("parent:space"));
    parentSpace.SetConnections({SdfPath("/Outside.matrix")});
    // The detached descriptor adapter also accepts token metadata.
    CHECK(parentSpace.SetMetadata(TfToken("rigExecReadPhase"),VtValue(TfToken("final"))));
    const auto propertyMover=stage->DefinePrim(SdfPath("/Rig/PropertyMover"),TfToken("RigExecFloatMathMover"));
    propertyMover.GetAttribute(TfToken("rigExec:operation")).Set(TfToken("add"));
    propertyMover.GetAttribute(TfToken("inputs:value")).Set(1.25f);
    const auto database = Capture(stage, {UsdTimeCode::Default()});
    const RigExecSceneDbAccess access(database);
    RigExecSceneDescriptors scene;
    std::string error;
    CHECK(RigExecCaptureSceneDescriptors(access,rig.GetPath(),{UsdTimeCode::Default()},&scene,&error));
    const auto &resolved = scene.attributes.at(input.GetPath()).inputs.front();
    CHECK(resolved.raw == VtValue(2.0));
    CHECK(resolved.resolved == VtValue(7.0));
    CHECK(resolved.state == RigExecSceneInputState::Connected);
    const RigExecSceneCompileInputs compileInputs(scene);
    RigExecSceneBoundInput rawBinding,connectedBinding,computedBinding;
    CHECK(compileInputs.Bind(input.GetPath(),RigExecSceneReadRoute::Raw,&rawBinding,&error));
    CHECK(compileInputs.Bind(input.GetPath(),RigExecSceneReadRoute::ConnectionResolved,&connectedBinding,&error));
    double scalar=0;
    CHECK(compileInputs.Read(rawBinding,UsdTimeCode::Default(),&scalar,&error));
    CHECK(scalar==2.0);
    CHECK(compileInputs.Read(connectedBinding,UsdTimeCode::Default(),&scalar,&error));
    CHECK(scalar==7.0);
    CHECK(connectedBinding.source==SdfPath("/Outside.value"));
    CHECK(!compileInputs.Read(connectedBinding,UsdTimeCode(123),&scalar,&error));
    CHECK(compileInputs.Bind(joint.GetPath().AppendProperty(TfToken("default:space")),
        RigExecSceneReadRoute::ConnectionResolved,&computedBinding,&error));
    VtValue computedValue;
    CHECK(!compileInputs.Read(computedBinding,UsdTimeCode::Default(),&computedValue,nullptr,&error));
    CHECK(error.find("computed input requires graph producer")!=std::string::npos);
    CHECK(compileInputs.Targets(solver.GetPath().AppendProperty(TfToken("rigExec:joints")))==SdfPathVector{joint.GetPath()});

    CHECK(scene.jointBindings.size() == 1);
    CHECK(scene.jointBindings.front().joint == joint.GetPath());
    CHECK(scene.nodes.at(solver.GetPath()).stackOrdinal < scene.nodes.at(joint.GetPath()).stackOrdinal);
    const auto restFrame=RigExecFrameFromMatrix(GfMatrix4d(1.0));
    GfMatrix4d movedMatrix(1.0);movedMatrix.SetTranslate(GfVec3d(3,4,5));
    const auto movedFrame=RigExecFrameFromMatrix(movedMatrix);
    RigExecSceneSolverDescriptor loweredSolver;
    CHECK(RigExecLowerSceneSolver(scene,solver.GetPath(),{{joint.GetPath(),restFrame}},&loweredSolver,&error));
    RigExecSolverInputs solverInputs;
    CHECK(RigExecResolveSceneSolverInputs(scene,loweredSolver,UsdTimeCode::Default(),
        {{joint.GetPath(),movedFrame}},{},{},&solverInputs,&error));
    RigExecSolverWorkspace solverWorkspace;
    RigExecPointFrameArray solverResult;
    CHECK(RigExecRunSolver(loweredSolver.record,solverInputs,&solverWorkspace,&solverResult,&error));
    RigExecFkChainElement expectedElement;
    expectedElement.restPoints=restFrame.points;expectedElement.posePoints=movedFrame.points;
    expectedElement.hasOutRest=false;expectedElement.parentIndex=-1;
    const auto expectedFk=RigExecSolveFkChain({expectedElement});
    CHECK(solverResult.frames==expectedFk);
    CHECK(solverResult.rests==std::vector<RigExecSolverRest>{restFrame.points});

    RigExecScenePropertyDescriptor propertyDescriptor;
    CHECK(RigExecLowerSceneProperty(scene,propertyMover.GetPath(),input.GetPath(),&propertyDescriptor,&error));
    bool enabled=false;float defaultWeight=0;
    CHECK(RigExecResolveSceneProperty(scene,&propertyDescriptor,UsdTimeCode::Default(),{},&enabled,&defaultWeight,&error));
    CHECK(enabled && defaultWeight==1.0f);
    const auto &propertyRecord=std::get<RigExecFloatPropertyRecord>(propertyDescriptor.record);
    double propertyResult=0;
    CHECK(RigExecRunProperty(propertyRecord,2.0,defaultWeight,&propertyResult));
    RigExecPropertyMathParams<float> expectedProperty;
    expectedProperty.op=RigExecPropertyOp::Add;expectedProperty.value=1.25f;expectedProperty.weight=1.0f;
    CHECK(propertyResult==double(RigExecApplyFloatMath(2.0f,expectedProperty)));
    RigExecProviderProgram provider;
    CHECK(RigExecBuildProviderProgram(scene,false,&provider,&error));
    CHECK(provider.routedInputs.size()==1);
    if(!provider.routedInputs.empty()) {
        const auto &route=provider.routedInputs.front();
        CHECK(route.consumer==parentSpace.GetPath());
        CHECK(route.source==SdfPath("/Outside.matrix"));
        CHECK(route.readPhase=="final");
        RigExecTypedValueStore store(provider.valueKeys.size());
        std::vector<RigExecValueId> changed;
        CHECK(RigExecSampleProviderProgram(provider,scene,0,{},&store,&changed,&error));
        RigExecProviderPlainProgram plain;
        CHECK(RigExecExportProviderRecords(provider,store,&plain,&error));
        CHECK(plain.routedInputs.size()==1);
        if(!plain.routedInputs.empty())CHECK(plain.routedInputs.front().value==route.value);
    }
    RigExecCompiledGraph graph;
    CHECK(!RigExecCompileStandaloneGraph(database,rig.GetPath(),{UsdTimeCode::Default()},
        {},RigExecCyclePolicy::Reject,&scene,&graph,&error));
    CHECK(error == "scene compiler needs production typed kernel lowering");
}
static void TestUnavailableDriverFramesRevision()
{
    auto stage=UsdStage::CreateInMemory();
    auto rig=RigExecRigBuilder::Create(stage,SdfPath("/Rig"));
    auto control=stage->DefinePrim(SdfPath("/Rig/Driver"),TfToken("RigExecControl"));
    CHECK(control.GetAttribute(TfToken("avars:tx")).Set(2.0));
    auto mesh=UsdGeomMesh::Define(stage,SdfPath("/Rig/Body"));
    CHECK(mesh.CreatePointsAttr().Set(VtVec3fArray{GfVec3f(0),GfVec3f(1,0,0)}));
    auto chain=rig.NewMoverChain("Geometry",mesh.GetPointsAttr().GetPath());
    chain.AddMatrixMover("Move",control.GetPath());
    const SdfPath driver("/Rig/NotAnAggregate");
    stage->DefinePrim(driver,TfToken("RigExecControl"));
    auto curve=stage->DefinePrim(SdfPath("/Rig/Curve"),TfToken("BasisCurves"));
    CHECK(curve.CreateAttribute(TfToken("points"),SdfValueTypeNames->Point3fArray)
        .Set(VtVec3fArray{GfVec3f(0),GfVec3f(1,0,0)}));
    CHECK(curve.CreateAttribute(TfToken("curveVertexCounts"),SdfValueTypeNames->IntArray).Set(VtIntArray{2}));
    CHECK(curve.GetAttribute(TfToken("type")).Set(TfToken("linear")));
    CHECK(curve.GetAttribute(TfToken("wrap")).Set(TfToken("nonperiodic")));
    const auto missing=chain.AddCurveMover("MissingFrames",curve.GetPath(),driver,SdfPath(),TfToken("emitGuidePoints"));
    auto database=Capture(stage,{UsdTimeCode::Default()});
    RigExecStandaloneSceneRuntime captured;std::string error;
    CHECK(captured.Prepare(database,rig.GetRootPath(),{UsdTimeCode::Default()},&error));
    const auto &program=captured.GetProgram();
    const std::string reason="driver frames "+driver.GetString()+" is not a solver of this rig; mover set aside";
    CHECK(program.skippedOperations.count(missing.GetPath())==1);
    if(program.skippedOperations.count(missing.GetPath()))CHECK(program.skippedOperations.at(missing.GetPath())==reason);
    bool surviving=false;
    for(const auto &operation:program.operations)if(const auto *geometry=std::get_if<RigExecSceneGeometryOp>(&operation)) {
        CHECK(geometry->binding.descriptor.mover!=missing.GetPath());
        surviving=true;
    }
    CHECK(surviving);
    RigExecStandaloneSystem system(database);
    const int points=system.AddTap(RigExecValueAddress::Property(mesh.GetPointsAttr().GetPath()));
    CHECK(system.Prepare(&error));
    const auto pose=system.Evaluate(UsdTimeCode::Default());CHECK(pose.valid);
    CHECK(pose.Get<VtVec3fArray>(points)==VtVec3fArray({GfVec3f(2,0,0),GfVec3f(3,0,0)}));
    CHECK(std::find(pose.diagnostics.begin(),pose.diagnostics.end(),missing.GetPath().GetString()+": "+reason)!=pose.diagnostics.end());
    // A genuine malformed typed read remains a compiler failure, not an
    // optional-driver skip: invalid authored phase is checked before admission.
    const auto malformedRead=stage->GetPrimAtPath(missing.GetPath()).GetRelationship(TfToken("rigExec:bindCoordinates"));
    CHECK(bool(malformedRead));
    CHECK(malformedRead.SetMetadata(TfToken("rigExecReadPhase"),VtValue(std::string("not-a-phase"))));
    auto malformed=Capture(stage,{UsdTimeCode::Default()});
    RigExecStandaloneSceneRuntime refused;error.clear();
    CHECK(!refused.Prepare(malformed,rig.GetRootPath(),{UsdTimeCode::Default()},&error));
    CHECK(!error.empty());
}

static void TestPublicProductionDomains()
{
    auto stage=UsdStage::CreateInMemory();
    auto rig=RigExecRigBuilder::Create(stage,SdfPath("/Rig"));
    auto control=stage->DefinePrim(SdfPath("/Rig/Driver"),TfToken("RigExecControl"));
    CHECK(control.GetAttribute(TfToken("avars:tx")).Set(2.0));
    auto data=stage->DefinePrim(SdfPath("/Rig/Data"),TfToken("Scope"));
    auto scalar=data.CreateAttribute(TfToken("value"),SdfValueTypeNames->Float);CHECK(scalar.Set(2.0f));
    auto multiply=rig.NewMoverChain("Scalar",scalar.GetPath()).AddFloatMathMover("Multiply",TfToken("multiply"),3.0f);
    auto mesh=UsdGeomMesh::Define(stage,SdfPath("/Rig/Body"));
    CHECK(mesh.CreatePointsAttr().Set(VtVec3fArray{GfVec3f(0),GfVec3f(1,0,0)}));
    rig.NewMoverChain("Geometry",mesh.GetPointsAttr().GetPath()).AddMatrixMover("Move",control.GetPath());
    auto selected=data.CreateAttribute(TfToken("selected"),SdfValueTypeNames->Float3);
    CHECK(selected.Set(GfVec3f(-1)));
    CHECK(selected.SetMetadata(TfToken("rigExecInputElement"),VtValue(1)));
    CHECK(selected.SetMetadata(TfToken("rigExecReadPhase"),VtValue(std::string("final"))));
    CHECK(selected.SetConnections({mesh.GetPointsAttr().GetPath()}));
    auto database=Capture(stage,{UsdTimeCode::Default()});
    RigExecStandaloneSystem system(database);
    const int scalarFinal=system.AddTap(RigExecValueAddress::Property(scalar.GetPath()));
    const int scalarBase=system.AddTap(RigExecValueAddress::Property(scalar.GetPath(),TfToken("base")));
    const int posed=system.AddTap(RigExecValueAddress::Property(mesh.GetPointsAttr().GetPath()));
    const int frame=system.AddTap(RigExecValueAddress::Prim(control.GetPath(),TfToken("computePointFrame")));
    const int selectedTap=system.AddTap(RigExecValueAddress::Property(selected.GetPath()));
    std::string prepareError; CHECK(system.Prepare(&prepareError)); if(!prepareError.empty())std::printf("production prepare: %s\n",prepareError.c_str());const void *identity=system.GetCompilerIdentity();
    const auto first=system.Evaluate(UsdTimeCode::Default());CHECK(first.valid);
    CHECK(first.Get<float>(scalarFinal)==6.0f && first.Get<float>(scalarBase)==2.0f);
    CHECK(first.Get<VtVec3fArray>(posed)==VtVec3fArray({GfVec3f(2,0,0),GfVec3f(3,0,0)}));
    CHECK(first.Get<RigExecPointFrame>(frame).Origin()==GfVec3d(2,0,0));
    CHECK(first.Get<GfVec3f>(selectedTap)==GfVec3f(3,0,0));
    CHECK(system.SetValue(multiply.GetPath().AppendProperty(TfToken("inputs:value")),UsdTimeCode::Default(),VtValue(4.0f)));
    const auto changed=system.Evaluate(UsdTimeCode::Default());CHECK(changed.valid);
    CHECK(changed.Get<float>(scalarFinal)==8.0f && changed.Get<float>(scalarBase)==2.0f);
    CHECK(system.GetCompilerIdentity()==identity);
    const auto held=system.Evaluate(UsdTimeCode::Default());CHECK(held.valid);
    CHECK(held.Get<float>(scalarFinal)==8.0f);
    CHECK(first.Get<float>(scalarFinal)==6.0f);
    CHECK(system.SetValue(mesh.GetPointsAttr().GetPath(),UsdTimeCode::Default(),VtValue(VtVec3fArray{GfVec3f(0)})));
    const auto lost=system.Evaluate(UsdTimeCode::Default());CHECK(lost.valid);
    CHECK(lost.Get<GfVec3f>(selectedTap)==GfVec3f(-1));
    CHECK(system.GetCompilerIdentity()==identity);
    CHECK(system.SetValue(mesh.GetPointsAttr().GetPath(),UsdTimeCode::Default(),VtValue(VtVec3fArray{GfVec3f(0),GfVec3f(1,0,0)})));
    const auto recovered=system.Evaluate(UsdTimeCode::Default());CHECK(recovered.valid);
    CHECK(recovered.Get<GfVec3f>(selectedTap)==GfVec3f(3,0,0));
    stage.Reset();CHECK(system.Evaluate(UsdTimeCode::Default()).valid);
}
static void TestProjectorCapturedDefaultEpoch()
{
    auto stage=UsdStage::CreateInMemory();auto rig=RigExecRigBuilder::Create(stage,SdfPath("/Rig"));
    auto parent=UsdGeomXform::Define(stage,SdfPath("/Rig/Parent"));
    auto translate=parent.AddTranslateOp();CHECK(translate.Set(GfVec3d(0)));
    auto mesh=UsdGeomMesh::Define(stage,SdfPath("/Rig/Parent/Mesh"));
    CHECK(mesh.CreatePointsAttr().Set(VtVec3fArray{GfVec3f(-1,0,1),GfVec3f(1,0,1),GfVec3f(0,1,1)}));
    CHECK(mesh.CreateFaceVertexCountsAttr().Set(VtIntArray{3}));CHECK(mesh.CreateFaceVertexIndicesAttr().Set(VtIntArray{0,1,2}));
    auto control=stage->DefinePrim(SdfPath("/Rig/Control"),TfToken("RigExecControl"));
    CHECK(control.GetAttribute(TfToken("avars:tx")).Set(1.0));
    rig.NewMoverChain("Geometry",mesh.GetPointsAttr().GetPath()).AddMatrixMover("Move",control.GetPath());
    auto projector=stage->DefinePrim(SdfPath("/Rig/Projector"),TfToken("RigExecSurfaceProjector"));
    CHECK(projector.AddAppliedSchema(TfToken("RigExecMoverAPI")));
    CHECK(projector.GetRelationship(TfToken("rigExec:moves")).SetTargets({mesh.GetPointsAttr().GetPath()}));
    CHECK(projector.GetRelationship(TfToken("rigExec:sources")).SetTargets({control.GetPath()}));
    auto database=Capture(stage,{UsdTimeCode::Default()});std::string error;
    RigExecStandaloneSceneRuntime captured;CHECK(captured.Prepare(database,SdfPath("/Rig"),{UsdTimeCode::Default()},&error));
    if(!error.empty())std::printf("projector prepare: %s\n",error.c_str());
    bool found=false;
    for(const auto &operation:captured.GetProgram().operations)if(const auto *geometry=std::get_if<RigExecSceneGeometryOp>(&operation))
        if(geometry->binding.descriptor.record.op==RigExecRevisionOp::SurfaceProjector) {
            found=true;CHECK(geometry->binding.descriptor.record.binding.meshWorldInverse==GfMatrix4d(1));
        }
    CHECK(found);
    RigExecStandaloneSystem system(database);system.AddTap(RigExecValueAddress::Property(mesh.GetPointsAttr().GetPath()));
    CHECK(system.Prepare(&error));const auto before=system.GetCompilerIdentity();
    CHECK(system.SetValue(translate.GetAttr().GetPath(),UsdTimeCode::Default(),VtValue(GfVec3d(4,0,0))));
    CHECK(system.Prepare(&error));CHECK(system.GetCompilerIdentity()!=before);
    database.attributes.at(translate.GetAttr().GetPath()).resolved["default"]=VtValue(GfVec3d(4,0,0));
    CHECK(captured.Prepare(database,SdfPath("/Rig"),{UsdTimeCode::Default()},&error));found=false;
    for(const auto &operation:captured.GetProgram().operations)if(const auto *geometry=std::get_if<RigExecSceneGeometryOp>(&operation))
        if(geometry->binding.descriptor.record.op==RigExecRevisionOp::SurfaceProjector) {
            found=true;CHECK(geometry->binding.descriptor.record.binding.meshWorldInverse==GfMatrix4d(1).SetTranslate(GfVec3d(-4,0,0)));
        }
    CHECK(found);
    std::map<SdfPath,RigExecStandaloneResolvedState> rows;
    for(const auto &[path,attribute]:database.attributes)rows[path]={attribute.resolved.at("default"),attribute.blockedIdentities.count("default")!=0};
    rows[translate.GetAttr().GetPath()].value=VtValue(GfVec3d(5,0,0));
    const auto refused=system.EvaluateResolved(UsdTimeCode(19),rows);CHECK(!refused.valid);
    CHECK(!refused.diagnostics.empty());if(!refused.diagnostics.empty())CHECK(refused.diagnostics.front().find("transient structural state")!=std::string::npos);
    CHECK(system.GetCompilerIdentity()!=nullptr);
}

static void TestScopedExternalProducers()
{
    auto stage=UsdStage::CreateInMemory();
    auto first=RigExecRigBuilder::Create(stage,SdfPath("/First"));
    auto second=RigExecRigBuilder::Create(stage,SdfPath("/Second"));
    auto externalControl=stage->DefinePrim(SdfPath("/External/Control"),TfToken("RigExecControl"));
    CHECK(externalControl.GetAttribute(TfToken("avars:tx")).Set(3.0));
    auto external=stage->DefinePrim(SdfPath("/External/FK"),TfToken("RigExecFkChain"));
    CHECK(external.GetRelationship(TfToken("rigExec:controls")).SetTargets({externalControl.GetPath()}));
    auto blend=stage->DefinePrim(SdfPath("/First/Blend"),TfToken("RigExecBlendPointFrames"));
    CHECK(blend.GetRelationship(TfToken("rigExec:inputA")).SetTargets({external.GetPath()}));
    CHECK(blend.GetRelationship(TfToken("rigExec:inputB")).SetTargets({external.GetPath()}));
    CHECK(blend.GetAttribute(TfToken("inputs:weight")).Set(0.25f));
    auto data=stage->DefinePrim(SdfPath("/Second/Data"),TfToken("Scope"));
    auto value=data.CreateAttribute(TfToken("value"),SdfValueTypeNames->Float);CHECK(value.Set(2.0f));
    second.NewMoverChain("Multiply",value.GetPath()).AddFloatMathMover("TimesThree",TfToken("multiply"),3.0f);
    auto unrelated=stage->DefinePrim(SdfPath("/Unrelated"),TfToken("RigExecFutureSchema"));
    auto raw=unrelated.CreateAttribute(TfToken("value"),SdfValueTypeNames->Double);CHECK(raw.Set(99.0));
    auto database=Capture(stage,{UsdTimeCode::Default()});
    RigExecStandaloneSceneRuntime scoped;std::string error;
    CHECK(scoped.Prepare(database,SdfPath("/First"),{UsdTimeCode::Default()},&error));
    CHECK(scoped.GetProgram().layout.solverAggregates.count(external.GetPath())==1);
    CHECK(scoped.GetProgram().layout.sampleIndex.count(raw.GetPath())==0);
    CHECK(scoped.GetProgram().layout.sampleIndex.count(value.GetPath())==0);
    CHECK(scoped.Evaluate(0,{},&error));VtValue aggregate;
    CHECK(scoped.Read(blend.GetPath(),RigExecSceneValueDomain::SolverAggregate,&aggregate));
    CHECK(aggregate.IsHolding<RigExecPointFrameArray>());
    if(aggregate.IsHolding<RigExecPointFrameArray>()) {
        const auto &frames=aggregate.UncheckedGet<RigExecPointFrameArray>().frames;
        CHECK(frames.size()==1);if(frames.size()==1)CHECK(frames.front().Origin()==GfVec3d(3,0,0));
    }
    RigExecStandaloneSystem both(database);
    const auto posed=both.AddTap(RigExecValueAddress::Prim(blend.GetPath(),TfToken("computePointFrameArray")));
    const auto scalar=both.AddTap(RigExecValueAddress::Property(value.GetPath()));
    const auto unknownRaw=both.AddTap(RigExecValueAddress::Property(raw.GetPath()));
    CHECK(both.Prepare(&error));if(!error.empty())std::printf("multiple-root prepare: %s\n",error.c_str());const auto result=both.Evaluate(UsdTimeCode::Default());CHECK(result.valid);
    CHECK(result.Get<float>(scalar)==6.0f);CHECK(result.Get<double>(unknownRaw)==99.0);
    const auto frames=result.Get<RigExecPointFrameArray>(posed).frames;
    CHECK(frames.size()==1);if(frames.size()==1)CHECK(frames.front().Origin()==GfVec3d(3,0,0));
    RigExecStandaloneSystem unsupported(database);
    unsupported.AddTap(RigExecValueAddress::Prim(unrelated.GetPath(),TfToken("computePointFrame")));
    CHECK(!unsupported.Prepare(&error));CHECK(error.find("RigExecFutureSchema")!=std::string::npos);
}

static void TestPortableProviderKernels()
{
    const auto stage=UsdStage::CreateInMemory();
    const auto rig=stage->DefinePrim(SdfPath("/Rig"),TfToken("RigExecRoot"));
    const auto parent=stage->DefinePrim(SdfPath("/Rig/Parent"),TfToken("RigExecControl"));
    const auto child=stage->DefinePrim(SdfPath("/Rig/Parent/Child"),TfToken("RigExecJoint"));
    parent.GetAttribute(TfToken("rest:ry")).Set(30.0);
    parent.GetAttribute(TfToken("avars:ty")).Set(2.0);
    child.GetAttribute(TfToken("rest:tx")).Set(3.0);
    child.GetAttribute(TfToken("default:tz")).Set(1.0);
    child.GetAttribute(TfToken("avars:rx")).Set(45.0);
    child.GetAttribute(TfToken("avars:rotationOrder")).Set(TfToken("YXZ"));
    child.GetAttribute(TfToken("avars:rotationSign")).Set(GfVec3d(-1,1,1),UsdTimeCode(1));
    child.GetAttribute(TfToken("avars:rotationSign")).Set(GfVec3d(1,-1,1),UsdTimeCode(2));
    const std::vector<UsdTimeCode> times={UsdTimeCode::Default(),UsdTimeCode(1),UsdTimeCode(2)};
    const auto database=Capture(stage,times);
    RigExecStandaloneProviderRuntime portable;
    std::string error;
    const bool prepared=portable.Prepare(database,rig.GetPath(),times,&error);
    CHECK(prepared);
    if(!prepared) { std::printf("provider prepare: %s\n",error.c_str());return; }
    RigExecTapSet reference(stage);
    std::vector<std::pair<SdfPath,const char *>> requests;
    for(const auto &path:{parent.GetPath(),child.GetPath()})
        for(const auto *computation:{"computeRestFrame","computePointFrame","computeMatrix"}) {
            requests.emplace_back(path,computation);
            reference.Add(RigExecValueAddress::Prim(path,TfToken(computation)));
        }
    for(size_t time=0;time<times.size();++time) {
        const auto expected=reference.Evaluate(times[time]);
        CHECK(expected.IsComplete());
        if(!expected.IsComplete())continue;
        CHECK(portable.Evaluate(time,{},&error));
        for(size_t i=0;i<requests.size();++i) {
            const auto id=portable.GetProgram().FindValue(RigExecProviderValueKey(requests[i].first,requests[i].second));
            const auto &value=expected.Get(int(i));
            if(value.IsHolding<RigExecPointFrame>()) {
                const auto *actual=portable.GetValues().Read<RigExecPointFrame>(id);
                CHECK(actual && *actual==value.UncheckedGet<RigExecPointFrame>());
            } else {
                const auto *actual=portable.GetValues().Read<GfMatrix4d>(id);
                CHECK(actual && *actual==value.UncheckedGet<GfMatrix4d>());
            }
        }
        CHECK(portable.Evaluate(time,{},&error));
        CHECK(portable.GetExecution().executed==0);
    }
    const SdfPath posed=child.GetPath().AppendProperty(TfToken("posed:space"));
    CHECK(portable.Evaluate(2,{{posed,VtValue(GfMatrix4d(1.0))}},&error));
    const auto poseId=portable.GetProgram().FindValue(RigExecProviderValueKey(child.GetPath(),"computePointFrame"));
    const auto *identityPose=portable.GetValues().Read<RigExecPointFrame>(poseId);
    CHECK(identityPose && identityPose->points==RigExecPointFrame().points);
    CHECK(portable.Evaluate(2,{},&error));
    const auto *restoredPose=portable.GetValues().Read<RigExecPointFrame>(poseId);
    CHECK(restoredPose && *restoredPose==reference.Evaluate(times[2]).Get<RigExecPointFrame>(4));
}
static void TestInactiveProviderAvailability()
{
    auto stage=UsdStage::CreateInMemory();
    auto rig=RigExecRigBuilder::Create(stage,SdfPath("/Rig"));
    auto goal=stage->DefinePrim(SdfPath("/External/Goal"),TfToken("RigExecControl"));
    CHECK(goal.GetAttribute(TfToken("avars:ty")).Set(7.0));
    auto target=stage->DefinePrim(SdfPath("/Rig/Target"),TfToken("RigExecControl"));
    CHECK(target.GetAttribute(TfToken("avars:tx")).Set(2.0));
    auto aim=rig.NewMoverChain("Aim",target.GetPath()).AddAimConstraint("Aim");
    aim.SetAimTarget(goal.GetPath());
    auto data=stage->DefinePrim(SdfPath("/Rig/Data"),TfToken("Scope"));
    auto value=data.CreateAttribute(TfToken("value"),SdfValueTypeNames->Float);
    CHECK(value.Set(2.0f));
    rig.NewMoverChain("Numeric",value.GetPath()).AddFloatMathMover("Multiply",TfToken("multiply"),3.0f);
    const auto captured=Capture(stage,{UsdTimeCode::Default()});
    const auto evaluate=[&](const RigExecSceneDb &db,bool available,RigExecPointFrame *frame){
        RigExecStandaloneSceneRuntime runtime;std::string error;
        const bool prepared=runtime.Prepare(db,SdfPath("/Rig"),{UsdTimeCode::Default()},&error);
        CHECK(prepared);if(!prepared){std::printf("inactive provider prepare: %s\n",error.c_str());return;}
        CHECK(runtime.Evaluate(0,{},&error));VtValue result;
        CHECK(runtime.ReadPublic(value.GetPath(),&result));CHECK(result.IsHolding<float>());
        if(result.IsHolding<float>())CHECK(result.UncheckedGet<float>()==6.0f);
        CHECK(runtime.ReadNamed(goal.GetPath(),TfToken("computePointFrame"),&result));
        CHECK(result.IsHolding<RigExecPointFrame>());
        if(result.IsHolding<RigExecPointFrame>()){
            const auto &source=result.UncheckedGet<RigExecPointFrame>();CHECK(source.IsValid()==available);
            if(!available){RigExecPointFrame identity;CHECK(source.points==identity.points);CHECK(source.flags==0);}
        }
        CHECK(runtime.Read(target.GetPath(),RigExecSceneValueDomain::Pose,&result));
        CHECK(result.IsHolding<RigExecPointFrame>());
        if(result.IsHolding<RigExecPointFrame>())*frame=result.UncheckedGet<RigExecPointFrame>();
    };
    RigExecPointFrame active,inactive,recovered,ancestor;
    evaluate(captured,true,&active);
    auto direct=captured;direct.prims.at(goal.GetPath()).active=false;
    evaluate(direct,false,&inactive);CHECK(inactive.IsValid());CHECK(inactive.Origin()==GfVec3d(2,0,0));
    direct.prims.at(goal.GetPath()).active=true;evaluate(direct,true,&recovered);CHECK(recovered==active);
    auto hidden=captured;hidden.prims.at(SdfPath("/External")).active=false;
    evaluate(hidden,false,&ancestor);CHECK(ancestor==inactive);
}

static bool ContextReferenceFail(std::string *error,const std::string &message)
{ if(error)*error=message;return false; }
static bool BindProviderContextReference(RigExecProviderProgram *program,
    const std::vector<std::string> &fullValueKeys,RigExecValueId sourceResult,
    const SdfPath &consumer,const std::string &phase,
    const std::map<RigExecValueId,RigExecValueId> &selectedFrames,
    RigExecValueId *result,size_t *beginOp,std::string *error,
    const std::map<std::string,std::string> &externalNames) {
    if(!program || !result || !beginOp || consumer.IsEmpty() || phase.empty() ||
       program->valueKeys.size()>fullValueKeys.size() || sourceResult>=program->valueKeys.size() ||
       program->ops.size()!=program->descriptors.size())
        return ContextReferenceFail(error,"invalid provider consumer context");
    for(size_t i=0;i<program->valueKeys.size();++i)if(program->valueKeys[i]!=fullValueKeys[i])
        return ContextReferenceFail(error,"provider context changed an existing typed value identity");
    std::map<std::string,RigExecValueId> ids;
    for(size_t i=0;i<fullValueKeys.size();++i)if(!ids.emplace(fullValueKeys[i],RigExecValueId(i)).second)
        return ContextReferenceFail(error,"duplicate typed value key in provider consumer context: "+fullValueKeys[i]);
    for(const auto &[before,after]:selectedFrames)if(before>=program->valueKeys.size() || after>=fullValueKeys.size())
        return ContextReferenceFail(error,"provider consumer context references an unknown frame version");
    std::map<RigExecValueId,size_t> producers;
    for(size_t i=0;i<program->ops.size();++i)producers.emplace(program->ops[i].output,i);
    std::map<RigExecValueId,std::string> externals;
    for(const auto &input:program->externalInputs)externals.emplace(input.value,input.computation);
    // A back edge is conservatively affected. It is cloned, never normalized
    // away or rejected before cross-domain SCC compilation.
    std::map<RigExecValueId,unsigned char> state;
    std::function<bool(RigExecValueId)> affected=[&](RigExecValueId value) {
        const auto replacement=selectedFrames.find(value);
        if(replacement!=selectedFrames.end())return replacement->second!=value;
        const auto external=externals.find(value);
        if(external!=externals.end())return externalNames.count(external->second)!=0;
        auto &mark=state[value];if(mark)return mark!=2;
        const auto producer=producers.find(value);if(producer==producers.end()){mark=2;return false;}
        mark=1;bool changed=false;
        const auto &op=program->ops[producer->second];
        for(auto input:op.inputs)if(input!=RigExecNoProviderValue)changed=affected(input)||changed;
        for(const auto &input:op.xforms)if(input.raw!=RigExecNoProviderValue)changed=affected(input.raw)||changed;
        mark=changed?3:2;return changed;
    };
    std::map<RigExecValueId,RigExecValueId> replacements=selectedFrames;
    for(const auto &[value,index]:producers)
        if(value!=sourceResult && !replacements.count(value) && !affected(value))replacements.emplace(value,value);
    const std::string prefix="context:"+consumer.GetString()+":"+phase+":"+
        program->valueKeys[size_t(sourceResult)]+":";
    program->valueKeys=fullValueKeys;program->valueIds=std::move(ids);
    *beginOp=program->ops.size();
    if(!RigExecCloneProviderContext(program,sourceResult,prefix,externalNames,replacements,result,error))return false;
    for(size_t i=*beginOp;i<program->descriptors.size();++i) {
        auto &descriptor=program->descriptors[i];
        for(const auto &xform:program->ops[i].xforms)
            if(xform.raw!=RigExecNoProviderValue)descriptor.reads.push_back(xform.raw);
        std::sort(descriptor.reads.begin(),descriptor.reads.end());
        descriptor.reads.erase(std::unique(descriptor.reads.begin(),descriptor.reads.end()),descriptor.reads.end());
    }
    return true;
}

static void TestUnaffectedProviderContext()
{
    const auto make=[] {
        RigExecProviderProgram p;p.valueKeys={"raw","child","root","other","replacement","external"};
        for(size_t i=0;i<p.valueKeys.size();++i)p.valueIds.emplace(p.valueKeys[i],RigExecValueId(i));
        const auto add=[&](RigExecValueId output,std::vector<RigExecValueId> inputs) {
            RigExecProviderOp op{RigExecProviderOpKind::PosedFrame,SdfPath("/Provider"),output,std::move(inputs)};
            op.xforms.push_back({TfToken("xform"),0,2});p.ops.push_back(op);
            RigExecOpDescriptor d;d.key=p.valueKeys[size_t(output)];d.kind=7;d.reads=op.inputs;
            d.writes={output};d.predecessors={0};d.volatileInput=true;p.descriptors.push_back(d);
        };
        add(1,{0});add(2,{1});add(3,{0});return p;
    };
    const auto compare=[](const RigExecProviderProgram &input,RigExecValueId source,
        const std::map<RigExecValueId,RigExecValueId> &frames,
        const std::map<std::string,std::string> &names,bool expected) {
        auto old=input,now=input;auto keys=input.valueKeys;keys.push_back("additional");
        RigExecValueId a=99,b=99;size_t ia=99,ib=99;std::string ea,eb;
        CHECK(BindProviderContextReference(&old,keys,source,SdfPath("/Consumer"),"base",frames,&a,&ia,&ea,names)==expected);
        CHECK(RigExecBindProviderContext(&now,keys,source,SdfPath("/Consumer"),"base",frames,&b,&ib,&eb,names)==expected);
        CHECK(a==b&&ia==ib&&ea==eb&&old.valueKeys==now.valueKeys&&old.valueIds==now.valueIds);
        CHECK(old.ops.size()==now.ops.size()&&old.descriptors.size()==now.descriptors.size());
        CHECK(old.externalInputs.size()==now.externalInputs.size());
        for(size_t i=0;i<old.externalInputs.size();++i) {
            CHECK(old.externalInputs[i].value==now.externalInputs[i].value);
            CHECK(old.externalInputs[i].owner==now.externalInputs[i].owner);
            CHECK(old.externalInputs[i].computation==now.externalInputs[i].computation);
        }
        for(size_t i=0;i<old.ops.size();++i) {
            const auto &x=old.ops[i],&y=now.ops[i];
            CHECK(x.kind==y.kind&&x.owner==y.owner&&x.output==y.output&&x.inputs==y.inputs&&x.scaleAvars==y.scaleAvars);
            CHECK(x.xforms.size()==y.xforms.size());
            for(size_t k=0;k<x.xforms.size();++k)
                CHECK(x.xforms[k].name==y.xforms[k].name&&x.xforms[k].raw==y.xforms[k].raw&&x.xforms[k].type==y.xforms[k].type);
        }
        for(size_t i=0;i<old.descriptors.size();++i) {
            const auto &x=old.descriptors[i],&y=now.descriptors[i];
            CHECK(x.key==y.key&&x.kind==y.kind&&x.reads==y.reads&&x.writes==y.writes);
            CHECK(x.predecessors==y.predecessors&&x.volatileInput==y.volatileInput);
        }
    };
    auto p=make();compare(p,2,{}, {},true);compare(p,2,{{1,1}}, {},true);
    compare(p,2,{{1,4}}, {},true);compare(p,2,{{2,4}}, {},true);
    compare(p,0,{}, {},true);compare(p,2,{{0,4}}, {},true);
    auto cycle=p;cycle.ops[0].inputs={2};compare(cycle,2,{}, {},true);
    // Unreachable cycles must not alter the unaffected root clone.
    auto disconnected=p;disconnected.ops[2].inputs={3};compare(disconnected,2,{}, {},true);
    auto duplicate=p;duplicate.ops.push_back(p.ops[1]);duplicate.ops.back().inputs={4};
    duplicate.descriptors.push_back(p.descriptors[1]);compare(duplicate,2,{}, {},true);
    auto external=p;external.externalInputs.push_back({1,SdfPath("/External"),"frame"});
    external.externalInputs.push_back({1,SdfPath("/Other"),"later"});
    compare(external,2,{}, {},true);compare(external,2,{},{{"frame","mapped"}},true);
    compare(external,1,{}, {},true);compare(external,1,{},{{"frame","mapped"}},true);
    auto existing=p;existing.valueKeys[4]="context:/Consumer:base:root:root";
    existing.valueIds.clear();
    for(size_t i=0;i<existing.valueKeys.size();++i)existing.valueIds.emplace(existing.valueKeys[i],RigExecValueId(i));
    compare(existing,2,{}, {},true);
    auto reused=existing;RigExecValueId reusedResult=99;size_t reusedBegin=99;std::string reusedError;
    CHECK(RigExecBindProviderContext(&reused,existing.valueKeys,2,SdfPath("/Consumer"),"base",{},
        &reusedResult,&reusedBegin,&reusedError,{}));
    CHECK(reusedResult==4&&reusedBegin==existing.ops.size()&&reused.ops.size()==existing.ops.size());
    auto xformOnly=p;xformOnly.ops[1].inputs={0};xformOnly.ops[1].xforms[0].raw=1;
    compare(xformOnly,2,{}, {},true);compare(xformOnly,2,{{1,4}}, {},true);
    auto unknown=p;unknown.ops[1].inputs={88};compare(unknown,2,{}, {},true);
    compare(p,2,{{1,99}}, {},false);
    auto invalid=p;invalid.descriptors.pop_back();compare(invalid,2,{}, {},false);
}

static void TestIndexedParentDelivery()
{
    const SdfPath consumer("/Provider.parent:space");
    const auto program=[] {
        RigExecProviderProgram p;
        p.valueKeys={"raw","expression","fallback"};
        for(size_t i=0;i<p.valueKeys.size();++i)p.valueIds.emplace(p.valueKeys[i],RigExecValueId(i));
        p.ops.push_back({RigExecProviderOpKind::SpaceExpression,SdfPath("/Provider"),1,
            {0,RigExecNoProviderValue,2}});
        return p;
    };
    const auto compare=[&](const RigExecProviderProgram &input,RigExecValueId expression,
        const std::map<RigExecValueId,size_t> &index,bool expected) {
        auto linear=input,indexed=input;RigExecValueId a=99,b=99;std::string ea,eb;
        CHECK(RigExecBindProviderParentDelivery(&linear,consumer,expression,&a,&ea)==expected);
        CHECK(RigExecBindProviderParentDelivery(&indexed,consumer,expression,&b,&eb,&index)==expected);
        CHECK(a==b&&ea==eb&&linear.valueKeys==indexed.valueKeys&&linear.valueIds==indexed.valueIds);
        CHECK(linear.ops.size()==indexed.ops.size()&&linear.descriptors.size()==indexed.descriptors.size());
        for(size_t i=0;i<linear.ops.size();++i) {
            CHECK(linear.ops[i].kind==indexed.ops[i].kind&&linear.ops[i].owner==indexed.ops[i].owner);
            CHECK(linear.ops[i].output==indexed.ops[i].output&&linear.ops[i].inputs==indexed.ops[i].inputs);
        }
        for(size_t i=0;i<linear.descriptors.size();++i) {
            CHECK(linear.descriptors[i].key==indexed.descriptors[i].key);
            CHECK(linear.descriptors[i].reads==indexed.descriptors[i].reads&&linear.descriptors[i].writes==indexed.descriptors[i].writes);
        }
    };
    auto base=program();std::map<RigExecValueId,size_t> first;
    size_t indexed=0;
    const auto extend=[&] {
        for(;indexed<base.ops.size();++indexed)first.emplace(base.ops[indexed].output,indexed);
    };
    extend();compare(base,1,first,true);
    // Context binding can append new expression outputs between deliveries.
    base.valueKeys.push_back("appended");base.valueIds.emplace("appended",3);
    base.ops.push_back({RigExecProviderOpKind::SpaceExpression,SdfPath("/Appended"),3,
        {0,RigExecNoProviderValue,2}});
    extend();CHECK(first.at(3)==1);compare(base,3,first,true);
    for(int invalid=0;invalid<3;++invalid) {
        auto bad=program();auto later=bad.ops.front();
        if(invalid==0)bad.ops.front().kind=RigExecProviderOpKind::PosedFrame;
        if(invalid==1)bad.ops.front().inputs.pop_back();
        if(invalid==2)bad.ops.front().inputs[1]=0;
        bad.ops.push_back(later);
        // A valid later duplicate and prior delivery key cannot hide bad FIRST input.
        bad.valueIds.emplace("parentDelivery:"+consumer.GetString(),2);
        std::map<RigExecValueId,size_t> badFirst;
        for(size_t i=0;i<bad.ops.size();++i)badFirst.emplace(bad.ops[i].output,i);
        CHECK(badFirst.at(1)==0);compare(bad,1,badFirst,false);
    }
    compare(program(),2,first,false); // No producing op for the known fallback value.
}

int main()
{
    TestUnaffectedProviderContext();
    TestIndexedParentDelivery();
    CHECK(!PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR).empty());
    RigExecLoadComputations(); // Independent authored-USD TapSet reference only.
    TestInactiveProviderAvailability();
    TestSceneDescriptors();
    TestPortableProviderKernels();
    TestPublicProductionDomains();
    TestUnavailableDriverFramesRevision();
    TestScopedExternalProducers();
    TestProjectorCapturedDefaultEpoch();
    TfErrorMark errors;
    auto stage = UsdStage::CreateInMemory();
    const auto control = [&](const char *path, double rest) {
        const auto prim = stage->DefinePrim(SdfPath(path), TfToken("RigExecControl"));
        prim.GetAttribute(TfToken("rest:tx")).Set(rest);
        return prim;
    };
    const auto a = control("/A", 0), b = control("/B", 3), goal = control("/Goal", 6), pole = control("/Pole", 0);
    a.CreateAttribute(TfToken("userValue"), SdfValueTypeNames->Double).Set(1.0);
    a.GetAttribute(TfToken("avars:ty")).Set(1.0, UsdTimeCode(1));
    a.GetAttribute(TfToken("avars:ty")).Set(2.0, UsdTimeCode(2));
    pole.GetAttribute(TfToken("rest:ty")).Set(5.0);
    const auto fk = stage->DefinePrim(SdfPath("/FK"), TfToken("RigExecFkChain"));
    fk.GetRelationship(TfToken("rigExec:controls")).SetTargets({a.GetPath(), b.GetPath()});
    const auto ik = stage->DefinePrim(SdfPath("/IK"), TfToken("RigExecTwoBoneIk"));
    ik.GetRelationship(TfToken("rigExec:rootControl")).SetTargets({a.GetPath()});
    ik.GetRelationship(TfToken("rigExec:effectorControl")).SetTargets({goal.GetPath()});
    ik.GetRelationship(TfToken("rigExec:poleControl")).SetTargets({pole.GetPath()});
    // Bone lengths are measured from the rests of the joints the solver
    // names -- there is no absolute length attribute to seed. Rests of
    // 0/3/7 give the 3-unit upper and 4-unit lower this used to author.
    const auto joint = [&](const char *path, double rest) {
        const auto prim =
            stage->DefinePrim(SdfPath(path), TfToken("RigExecJoint"));
        prim.GetAttribute(TfToken("rest:tx")).Set(rest);
        return prim;
    };
    ik.GetRelationship(TfToken("rigExec:joints")).SetTargets(
        {joint("/JRoot", 0).GetPath(), joint("/JMid", 3).GetPath(),
         joint("/JEnd", 7).GetPath()});
    const auto blend = stage->DefinePrim(SdfPath("/Blend"), TfToken("RigExecBlendPointFrames"));
    blend.GetRelationship(TfToken("rigExec:inputA")).SetTargets({fk.GetPath()});
    blend.GetRelationship(TfToken("rigExec:inputB")).SetTargets({fk.GetPath()});
    const auto driver = stage->DefinePrim(SdfPath("/Driver"));
    driver.CreateAttribute(TfToken("w1"), SdfValueTypeNames->Float).Set(0.25f);
    driver.CreateAttribute(TfToken("w2"), SdfValueTypeNames->Float).Set(0.75f);
    driver.CreateAttribute(TfToken("matrix"), SdfValueTypeNames->Matrix4d).Set(
        GfMatrix4d(1.0).SetTranslate(GfVec3d(17, 0, 0)));
    blend.GetAttribute(TfToken("inputs:weight")).SetConnections({SdfPath("/Driver.w1")});
    const auto unused = stage->DefinePrim(SdfPath("/Unused"));
    unused.CreateAttribute(TfToken("value"), SdfValueTypeNames->Double).Set(1.0);
    const auto api = stage->DefinePrim(SdfPath("/Api"));
    CHECK(api.AddAppliedSchema(TfToken("MaterialBindingAPI")));
    api.CreateAttribute(TfToken("value"), SdfValueTypeNames->Double).Set(1.0);
    const auto alias = stage->DefinePrim(SdfPath("/Alias"));
    const auto aliasValue = alias.CreateAttribute(TfToken("value"), SdfValueTypeNames->Double);
    aliasValue.Set(4.0);
    aliasValue.SetConnections({SdfPath("/Unused.value")});
    const auto geometry = stage->DefinePrim(SdfPath("/Geometry"), TfToken("Points"));
    const auto points = geometry.GetAttribute(TfToken("points"));
    points.Set(VtVec3fArray{{0, 0, 0}, {1, 0, 0}});
    points.Set(VtVec3fArray{{1, 0, 0}, {2, 0, 0}}, UsdTimeCode(1));
    points.Set(VtVec3fArray{{2, 0, 0}, {3, 0, 0}, {4, 0, 0}}, UsdTimeCode(2));
    const auto sample = stage->DefinePrim(SdfPath("/Sample"), TfToken("RigExecBlendSample"));
    sample.GetRelationship(TfToken("rigExec:targetPoints")).SetTargets({points.GetPath()});
    const auto channel = stage->DefinePrim(SdfPath("/Channel"), TfToken("RigExecBlendInput"));
    channel.GetRelationship(TfToken("rigExec:samples")).SetTargets({sample.GetPath()});
    channel.GetAttribute(TfToken("inputs:weight")).Set(0.4f);
    const std::vector<UsdTimeCode> times{UsdTimeCode::Default(), UsdTimeCode(1), UsdTimeCode(2), UsdTimeCode::PreTime(2)};
    const auto db = Capture(stage, times);
    TestSchemaIdentityLifetime(db);
    TestRequestBoundaries(db);
    const std::vector<RigExecValueAddress> addresses{
        RigExecValueAddress::Prim(a.GetPath(), TfToken("computePointFrame")),
        RigExecValueAddress::Prim(b.GetPath(), TfToken("computePointFrame")),
        RigExecValueAddress::Prim(fk.GetPath(), TfToken("computePointFrameArray")),
        RigExecValueAddress::Prim(ik.GetPath(), TfToken("computePointFrameArray")),
        RigExecValueAddress::Prim(blend.GetPath(), TfToken("computePointFrameArray")),
        RigExecValueAddress::Property(points.GetPath()),
        RigExecValueAddress::Prim(channel.GetPath(), TfToken("computeBlendChannel"))};
    RigExecStandaloneSystem portable(db);
    auto reference = std::make_unique<RigExecTapSet>(stage);
    for (const auto &address : addresses) { portable.AddTap(address); reference->Add(address); }
    std::string error;
    CHECK(portable.Prepare(&error));
    if (!error.empty()) std::printf("prepare: %s\n", error.c_str());
    CHECK(reference->Prepare());
    const void *compiler = portable.GetCompilerIdentity();
    const auto compare = [&](UsdTimeCode time,bool retainedCompiler=true) {
        const auto actual = portable.Evaluate(time);
        const auto expected = reference->Evaluate(time);
        CHECK(actual.valid && expected.IsComplete());
        if (!actual.valid) for (const auto &message : actual.diagnostics) std::printf("runtime: %s\n", message.c_str());
        for (size_t i = 0; i < addresses.size(); ++i) {
            if (!Same(actual.Get(int(i)), expected.Get(int(i)))) {
                std::printf("tap %zu mismatch actual=%s expected=%s\n", i,
                    actual.Get(int(i)).GetTypeName().c_str(), expected.Get(int(i)).GetTypeName().c_str());
                if (i == 6) {
                    const auto x = actual.Get<RigExecBlendChannel>(6), y = expected.Get<RigExecBlendChannel>(6);
                    std::printf("channel actual weight=%f samples=%zu expected weight=%f samples=%zu\n", x.weight, x.samples.size(), y.weight, y.samples.size());
                    for (const auto &sample : x.samples) std::printf("actual activation=%f points=%zu\n",sample.activation,sample.points.size());
                    for (const auto &sample : y.samples) std::printf("expected activation=%f points=%zu\n",sample.activation,sample.points.size());
                }
                CHECK(false);
            }
        }
        if(retainedCompiler)CHECK(portable.GetCompilerIdentity()==compiler);
        else {CHECK(portable.GetCompilerIdentity()!=compiler);compiler=portable.GetCompilerIdentity();}
        return actual;
    };
    for (const auto time : times) {
        const auto snapshot = compare(time);
        const auto blendChannel = snapshot.Get<RigExecBlendChannel>(6);
        CHECK(blendChannel.samples.size() == 1);
        if (!blendChannel.samples.empty()) {
            const auto native = snapshot.Get<VtVec3fArray>(5);
            CHECK(blendChannel.samples.front().points == std::vector<GfVec3f>(native.begin(), native.end()));
        }
    }
    const auto retained = compare(UsdTimeCode::Default());
    for (double rest : {1.0, 2.0, 4.0}) {
        CHECK(portable.SetValue(SdfPath("/B.rest:tx"), UsdTimeCode::Default(), VtValue(rest)));
        b.GetAttribute(TfToken("rest:tx")).Set(rest);
        compare(UsdTimeCode::Default());
    }
    CHECK(retained.Get<RigExecPointFrame>(1).Origin() == GfVec3d(3, 0, 0));
    const auto beforeUnused = portable.GetValueInvalidationCount();
    CHECK(portable.SetValue(SdfPath("/Unused.value"), UsdTimeCode::Default(), VtValue(2.0)));
    CHECK(portable.GetValueInvalidationCount() == beforeUnused);
    CHECK(!portable.SetValue(SdfPath("/B.rest:tx"), UsdTimeCode::Default(), VtValue(5.0f)));
    CHECK(portable.SetTargets(SdfPath("/FK.rigExec:controls"), {b.GetPath(), a.GetPath()}));
    fk.GetRelationship(TfToken("rigExec:controls")).SetTargets({b.GetPath(), a.GetPath()});
    compare(UsdTimeCode::Default(),false);
    CHECK(portable.SetConnections(SdfPath("/Blend.inputs:weight"), {SdfPath("/Driver.w2")}));
    blend.GetAttribute(TfToken("inputs:weight")).SetConnections({SdfPath("/Driver.w2")});
    compare(UsdTimeCode::Default(),false);
    CHECK(portable.SetPrimActive(SdfPath("/B"), false));
    CHECK(!portable.Evaluate(UsdTimeCode::Default()).valid);
    CHECK(portable.SetPrimActive(SdfPath("/B"), true));
    compare(UsdTimeCode::Default(),false);
    CHECK(!portable.Evaluate(UsdTimeCode(3.5)).valid);
    CHECK(!portable.EvaluateResolved(UsdTimeCode(3.5), std::map<SdfPath,VtValue>{}).valid);
    std::map<SdfPath, VtValue> complete;
    for (const auto &[path, attribute] : db.attributes) complete[path] = attribute.resolved.at("default");
    complete[SdfPath("/B.rest:tx")] = VtValue(11.0);
    const auto firstOverride = portable.EvaluateResolved(UsdTimeCode(3.5), complete);
    CHECK(firstOverride.valid && firstOverride.Get<RigExecPointFrame>(1).Origin() == GfVec3d(11, 0, 0));
    complete[SdfPath("/B.rest:tx")] = VtValue(12.0);
    const auto secondOverride = portable.EvaluateResolved(UsdTimeCode(3.5), complete);
    CHECK(secondOverride.valid && secondOverride.Get<RigExecPointFrame>(1).Origin() == GfVec3d(12, 0, 0));
    CHECK(firstOverride.Get<RigExecPointFrame>(1).Origin() == GfVec3d(11, 0, 0));
    compare(UsdTimeCode::Default());
    {
        RigExecStandaloneSystem explicitStates(db);
        const int raw=explicitStates.AddTap(RigExecValueAddress::Property(SdfPath("/Unused.value")));
        CHECK(explicitStates.Prepare());const auto compilerId=explicitStates.GetCompilerIdentity();
        std::map<SdfPath,RigExecStandaloneResolvedState> rows;
        for(const auto &[path,attribute]:db.attributes)
            rows[path]={attribute.resolved.at("default"),attribute.blockedIdentities.count("default")!=0};
        rows[SdfPath("/Unused.value")]={VtValue(),true};
        const auto blocked=explicitStates.EvaluateResolved(UsdTimeCode(77),rows);CHECK(!blocked.valid);
        rows[SdfPath("/Unused.value")]={VtValue(9.0),true};
        CHECK(!explicitStates.EvaluateResolved(UsdTimeCode(77),rows).valid); // block/value contradiction
        rows[SdfPath("/Unused.value")]={VtValue(9.0),false};
        const auto recovered=explicitStates.EvaluateResolved(UsdTimeCode(77),rows);
        CHECK(recovered.valid && recovered.Get<double>(raw)==9.0);
        CHECK(explicitStates.GetCompilerIdentity()==compilerId);
        CHECK(!explicitStates.Evaluate(UsdTimeCode(77)).valid); // temporary row was removed
        CHECK(explicitStates.Evaluate(UsdTimeCode::Default()).Get<double>(raw)==1.0);
        rows[SdfPath("/Blend.rigExec:rotationBlend")].value=VtValue(TfToken("longArc"));
        const auto structural=explicitStates.EvaluateResolved(UsdTimeCode(77),rows);
        CHECK(!structural.valid && !structural.diagnostics.empty());
        if(!structural.diagnostics.empty())CHECK(structural.diagnostics.front().find("transient structural state")!=std::string::npos);
    }
    reference.reset();
    const UsdStageWeakPtr weakStage(stage);
    stage.Reset();
    CHECK(!weakStage);
    CHECK(portable.Evaluate(UsdTimeCode::Default()).valid);
    CHECK(errors.IsClean());
    for (const auto &entry : errors) std::printf("USD error: %s\n", entry.GetCommentary().c_str());
    errors.Clear();
    std::printf("testRigExecStandalone: %d failure(s)\n", failures);
    return failures ? 1 : 0;
}
