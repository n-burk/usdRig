// The evaluator has one compiled graph; legacy custom policy is inert.
#include "rigExec/rigEvaluator.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/schemaRegistry.h"
#include "pxr/usd/usd/primDefinition.h"
#include "pxr/usd/usd/stage.h"
#include <cstdio>
#include <string>
using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE
static int failures=0;
#define CHECK(x) do { if(!(x)) { ++failures; std::printf("FAIL line %d: %s\n",__LINE__,#x); } } while(0)
static const TfToken kBaked("rigExec:baked");
static const SdfPath kRigPath("/Asset/Rig");
enum class Authored { Nothing,False,True };
static UsdStageRefPtr
MakeARig(Authored baked, bool connectedSpace = false)
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    const UsdPrim rig = stage->DefinePrim(kRigPath, TfToken("RigExecRoot"));
    if (baked != Authored::Nothing) {
        // Through GetAttribute rather than CreateAttribute: the schema
        // declares the attribute, and authoring it the way an artist's
        // layer does is the only authoring this feature is about.
        const UsdAttribute attribute = rig.CreateAttribute(kBaked,SdfValueTypeNames->Bool,true);
        if (!attribute) {
            return UsdStageRefPtr();
        }
        attribute.Set(baked == Authored::True);
    }
    const UsdPrim control = stage->DefinePrim(SdfPath("/Asset/Rig/Root"),
                                              TfToken("RigExecControl"));
    control.GetAttribute(TfToken("avars:tx")).Set(3.0);
    const UsdPrim joint = stage->DefinePrim(SdfPath("/Asset/Rig/Bone"),
                                            TfToken("RigExecJoint"));
    joint.GetAttribute(TfToken("avars:ty")).Set(2.0);
    if (connectedSpace) {
        // The connection is to the control's own posed:space, which is the
        // identity the joint would have composed anyway: the rig still
        // evaluates to the pose a plain one does, and the only thing that
        // changed is that an epoch constant became an exec answer.
        joint.CreateAttribute(TfToken("posed:space"),
                              SdfValueTypeNames->Matrix4d)
            .AddConnection(control.GetPath().AppendProperty(
                TfToken("posed:space")));
    }

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

static void CheckAuthoredValues(const RigExecRigPose &pose,double controlX=3,double boneY=2) {
    CHECK(pose.valid); CHECK(pose.referenceAgreements>0 && pose.referenceMismatches==0);
    const auto control=pose.controlFrames.find(SdfPath("/Asset/Rig/Root"));
    const auto joint=pose.jointFramesFinal.find(SdfPath("/Asset/Rig/Bone"));
    CHECK(control!=pose.controlFrames.end() && joint!=pose.jointFramesFinal.end());
    if(control!=pose.controlFrames.end()) CHECK(control->second.Origin()==GfVec3d(controlX,0,0));
    if(joint!=pose.jointFramesFinal.end()) CHECK(joint->second.Origin()==GfVec3d(0,boneY,0));
    const auto value=pose.movedProperties.find(SdfPath("/Asset/Geom/Slab.points"));
    CHECK(value!=pose.movedProperties.end() && value->second.IsHolding<VtVec3fArray>());
    if(value!=pose.movedProperties.end() && value->second.IsHolding<VtVec3fArray>()) {
        const auto &points=value->second.UncheckedGet<VtVec3fArray>(); CHECK(points.size()==3);
        if(points.size()==3) CHECK(points[0]==GfVec3f(0,float(boneY),0) &&
            points[1]==GfVec3f(1,float(boneY),0) && points[2]==GfVec3f(0,float(boneY)+1,0));
    }
}
static std::string Resources(const std::string &examples) {
#ifdef RIGEXEC_SCHEMA_RESOURCE_DIR
    return TfAbsPath(RIGEXEC_SCHEMA_RESOURCE_DIR);
#else
    return TfAbsPath(examples+"/../plugin/rigExecSchema/resources");
#endif
}

static void TestLegacyPolicyIsInert() {
    const auto *definition=UsdSchemaRegistry::GetInstance().FindConcretePrimDefinition(TfToken("RigExecRoot"));
    CHECK(definition && !definition->GetSchemaAttributeSpec(kBaked));
    for(auto authored:{Authored::Nothing,Authored::False,Authored::True}) {
        auto stage=MakeARig(authored); CHECK(stage); if(!stage) continue;
        RigExecRigEvaluator evaluator(stage,kRigPath); evaluator.cpuReference=true;
        CHECK(evaluator.Compile()); CHECK(!evaluator.GetOpGraph().empty());
        CheckAuthoredValues(evaluator.Evaluate(UsdTimeCode(1)));
    }
}

static void TestSingleGraphDispatch() {
    auto stage=MakeARig(Authored::Nothing); CHECK(stage); if(!stage) return;
    RigExecRigEvaluator evaluator(stage,kRigPath); evaluator.cpuReference=true;
    CHECK(evaluator.Compile()); const auto graph=evaluator.GetOpGraph(); CHECK(!graph.empty());
    CheckAuthoredValues(evaluator.Evaluate(UsdTimeCode(1)));
    CheckAuthoredValues(evaluator.Evaluate(UsdTimeCode(1)));
    // Exact authored results, plus the independent scalar geometry oracle.
    // This exercises changed-value cutoff without comparing two graph runs as goldens.
    const auto control=stage->GetPrimAtPath(SdfPath("/Asset/Rig/Root")).GetAttribute(TfToken("avars:tx"));
    CHECK(control.Set(5.0)); CheckAuthoredValues(evaluator.Evaluate(UsdTimeCode(1)),5,2);
    const auto bone=stage->GetPrimAtPath(SdfPath("/Asset/Rig/Bone")).GetAttribute(TfToken("avars:ty"));
    CHECK(bone.Set(4.0)); CheckAuthoredValues(evaluator.Evaluate(UsdTimeCode(1)),5,4);
    CHECK(evaluator.GetOpGraph().size()==graph.size());
    CHECK(control.Set(3.0) && bone.Set(2.0)); CheckAuthoredValues(evaluator.Evaluate(UsdTimeCode(1)));
    RigExecRigEvaluator absent(stage,SdfPath("/Missing")); CHECK(!absent.Compile());
}

int main(int argc,char **argv) {
    if(argc<2) return 2;
    CHECK(!PlugRegistry::GetInstance().RegisterPlugins(Resources(argv[1])).empty());
    TestLegacyPolicyIsInert();
    TestSingleGraphDispatch();
    std::printf("Single graph policy and dispatch: %d failures\n",failures);
    return failures?1:0;
}
