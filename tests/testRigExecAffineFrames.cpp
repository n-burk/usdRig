#include "rigExec/rigEvaluator.h"
#include "rigExec/movers/moverRegistry.h"
#include "rigExecMath/affineFrameKernels.h"
#include "pxr/base/plug/registry.h"
#include "pxr/usd/usdGeom/points.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE
#define CHECK(x) do {if(!(x))throw std::runtime_error(#x);}while(false)
static GfMatrix4d Translate(double x,double y=0,double z=0) {GfMatrix4d m(1);m.SetTranslateOnly(GfVec3d(x,y,z));return m;}
static void Near(GfVec3d a,GfVec3d b) {if((a-b).GetLength()>=2e-6)std::cerr<<"actual "<<a<<" expected "<<b<<"\n";CHECK((a-b).GetLength()<2e-6);}
static UsdPrim Provider(const UsdStageRefPtr &stage,const char *name,const char *type,const SdfPath &source) {
    auto prim=stage->DefinePrim(SdfPath(std::string("/Rig/Computations/")+name),TfToken(type));
    if(prim.GetRelationship(TfToken("rigExec:source")))CHECK(prim.GetRelationship(TfToken("rigExec:source")).SetTargets({source}));
    CHECK(prim.GetRelationship(TfToken("rigExec:poseInputs")).SetTargets({source}));
    return prim;
}
static void Case() {
    const char *types[]={"RigExecCopyFrame","RigExecMappedFrame","RigExecSkinInfluence","RigExecArmatureParent","RigExecBoneFrame","RigExecConstraintFrame","RigExecLayeredSkinMover"};
    auto stage=UsdStage::CreateInMemory();stage->DefinePrim(SdfPath("/Rig"),TfToken("RigExecRoot"));
    auto source=stage->DefinePrim(SdfPath("/Rig/Controls/Source"),TfToken("RigExecControl"));
    CHECK(source.GetAttribute(TfToken("avars:tx")).Set(2.0));
    const char *labels[]={"copy","mapped","influence","parent","bone","constraint"};
    GfVec3d expected[]={GfVec3d(2,0,0),GfVec3d(2,1,0),GfVec3d(2,0,0),GfVec3d(2,3,0),GfVec3d(4,1,0),GfVec3d(1,1.5,0)};
    for(int i=0;i<6;++i) {
        auto computation=Provider(stage,labels[i],types[i],source.GetPath());
        CHECK(computation.GetAttribute(TfToken("outputs:matrix")));
        if(i==1)CHECK(computation.GetAttribute(TfToken("inputs:targetRest")).Set(Translate(0,1)));
        if(i==3) {CHECK(computation.GetAttribute(TfToken("inputs:useIncoming")).Set(true));CHECK(computation.GetAttribute(TfToken("inputs:incoming")).Set(Translate(0,3)));}
        if(i==4) {CHECK(computation.GetAttribute(TfToken("inputs:local")).Set(Translate(0,1)));CHECK(computation.GetAttribute(TfToken("inputs:tx")).Set(2.0));CHECK(computation.GetRelationship(TfToken("rigExec:sourceObject")).SetTargets({source.GetPath()}));}
        if(i==5) {CHECK(computation.GetAttribute(TfToken("inputs:incoming")).Set(Translate(0,3)));CHECK(computation.GetAttribute(TfToken("inputs:influence")).Set(.5));}
        auto joint=stage->DefinePrim(SdfPath(std::string("/Rig/Joints/")+labels[i]),TfToken("RigExecJoint"));
        CHECK(joint.GetAttribute(TfToken("posed:space")).SetConnections({computation.GetAttribute(TfToken("outputs:matrix")).GetPath()}));
    }
    auto points=UsdGeomPoints::Define(stage,SdfPath("/Rig/Cloud"));points.CreatePointsAttr().Set(VtVec3fArray{GfVec3f(0),GfVec3f(0,1,0)});
    auto skin=stage->DefinePrim(SdfPath("/Rig/Ops/Skin"),TfToken(types[6]));skin.AddAppliedSchema(TfToken("RigExecMoverAPI"));
    CHECK(skin.GetRelationship(TfToken("rigExec:moves")).SetTargets({points.GetPointsAttr().GetPath()}));
    CHECK(skin.GetRelationship(TfToken("rigExec:influences")).SetTargets({SdfPath("/Rig/Joints/influence")}));
    CHECK(skin.GetAttribute(TfToken("rigExec:jointIndices")).Set(VtIntArray{0,0}));
    CHECK(skin.GetAttribute(TfToken("rigExec:jointWeights")).Set(VtFloatArray{1,1}));
    CHECK(skin.GetAttribute(TfToken("inputs:mask")).Set(VtFloatArray{.25,.75}));
    RigExecRigEvaluator evaluator(stage,SdfPath("/Rig"));evaluator.SetEvaluationMode(RigExecEvaluationMode::BakedWithParityCheck);
    std::vector<std::string> diagnostics;CHECK(evaluator.Compile(&diagnostics));
    std::string before;stage->GetRootLayer()->ExportToString(&before);
    auto pose=evaluator.Evaluate(UsdTimeCode(1));CHECK(pose.valid);CHECK(pose.bakedParityMismatches==0);CHECK(pose.moverGraphParityMismatches==0);
    for(int i=0;i<6;++i)Near(pose.jointFramesFinal.at(SdfPath(std::string("/Rig/Joints/")+labels[i])).Origin(),expected[i]);
    auto moved=pose.movedProperties.at(points.GetPointsAttr().GetPath()).Get<VtVec3fArray>();Near(GfVec3d(moved[0]),GfVec3d(.5,0,0));Near(GfVec3d(moved[1]),GfVec3d(1.5,1,0));
    // A control edit must invalidate both provider expressions and the external skin payload.
    CHECK(source.GetAttribute(TfToken("avars:tx")).Set(3.0));
    pose=evaluator.Evaluate(UsdTimeCode(1));CHECK(pose.valid);CHECK(pose.bakedParityMismatches==0);moved=pose.movedProperties.at(points.GetPointsAttr().GetPath()).Get<VtVec3fArray>();Near(GfVec3d(moved[0]),GfVec3d(.75,0,0));Near(GfVec3d(moved[1]),GfVec3d(2.25,1,0));
    CHECK(source.GetAttribute(TfToken("avars:tx")).Set(2.0));
    std::string after;stage->GetRootLayer()->ExportToString(&after);CHECK(before==after);
}
int main(int argc,char **argv) {try {
    CHECK(argc==2);PlugRegistry::GetInstance().RegisterPlugins(argv[1]);
    Case();
    // Standalone numerical entry point uses no stage or Exec context.
    RigExecAffineFrameInputs in;in.source=Translate(2);in.incoming=Translate(0,3);in.preserveLocation=true;
    Near(RigExecComputeCopyTransforms(in).ExtractTranslation(),GfVec3d(0,3,0));
    std::cout<<"core affine providers + layered skin PASS\n";return 0;
} catch(const std::exception &e) {std::cerr<<e.what()<<"\n";return 1;}}
