#include "rigExec/rigEvaluator.h"
#include "rigExec/movers/moverRegistry.h"
#include "rigExecMath/affineFrameKernels.h"
#include "rigExecRuntime/affineMath.h"
#include "pxr/base/plug/registry.h"
#include "pxr/usd/usdGeom/points.h"
#include <cmath>
#include <cstring>
#include <iostream>
#include <random>
#include <stdexcept>
#include <type_traits>
using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE
#define CHECK(x) do {if(!(x))throw std::runtime_error(#x);}while(false)
#include "rigExecFrozenParity.h"
#include "rigExecRuntimeDrive.h"
static GfMatrix4d Translate(double x,double y=0,double z=0) {GfMatrix4d m(1);m.SetTranslateOnly(GfVec3d(x,y,z));return m;}
static void Near(GfVec3d a,GfVec3d b) {if((a-b).GetLength()>=2e-6)std::cerr<<"actual "<<a<<" expected "<<b<<"\n";CHECK((a-b).GetLength()<2e-6);}
static UsdPrim Provider(const UsdStageRefPtr &stage,const char *name,const char *type,const SdfPath &source) {
    auto prim=stage->DefinePrim(SdfPath(std::string("/Rig/Computations/")+name),TfToken(type));
    if(prim.GetRelationship(TfToken("rigExec:source")))CHECK(prim.GetRelationship(TfToken("rigExec:source")).SetTargets({source}));
    CHECK(prim.GetRelationship(TfToken("rigExec:poseInputs")).SetTargets({source}));
    return prim;
}
// The .rigexec plays the affine frames and the layered skin through the
// runtime's own kernels -- the expressions over the runtime mirror, and the
// skin's playback kernel -- bit for bit as native evaluation, unposed and
// with the control's avar set as an input.
static void Runtime(const UsdStageRefPtr &stage) {
    RigExecRigEvaluator evaluator(stage,SdfPath("/Rig"));
    std::vector<std::string> diagnostics;CHECK(evaluator.Compile(&diagnostics));
    std::vector<uint8_t> bytes;std::string error;
    if(!RigExecTestBakeAt(evaluator,1,&bytes,&error))throw std::runtime_error("bake: "+error);
    RigExecTestPlayer player;
    if(!player.Open(bytes,stage,&error))throw std::runtime_error("open: "+error);
    const auto *handler=RigExecFindMoverHandler(TfToken("RigExecLayeredSkinMover"));
    CHECK(handler && handler->runtimeKernel.IsSet());
    CHECK(player->SetExternalKernel("RigExecLayeredSkinMover",handler->runtimeKernel,&error));
    CHECK(player->GetMissingExternalKernels().empty());
    const auto compare=[&](const RigExecRigPose &pose,const char *what) {
        std::vector<std::string> diffs;
        if(!RigExecCompareRuntimeOutputs(pose,player.Reader(),&diffs)) {
            for(const auto &line:diffs)std::cerr<<what<<": "<<line<<"\n";
            throw std::runtime_error(std::string(what)+": the runtime differs from native evaluation");
        }
    };
    if(!player.Play(1,&error))throw std::runtime_error("play: "+error);
    compare(evaluator.Evaluate(UsdTimeCode(1)),"unposed");
    const SdfPath avar("/Rig/Controls/Source.avars:tx");
    if(!player.Hold(avar.GetString(),3.0,&error) || !player.Play(1,&error))throw std::runtime_error("posed play: "+error);
    std::vector<RigExecRigPose> poses;
    if(!RigExecTestEditedPoses(stage,SdfPath("/Rig"),{{avar,VtValue(3.0)}},{1.0},&poses,&error))
        throw std::runtime_error("posed reference: "+error);
    compare(poses.front(),"posed");
}
// The runtime's instantiation of the expressions is Gf's, bit for bit: every
// expression over random frames, operations, spaces and settings.
template<class A,class B> static void Assign(A &,const B &) {}
static void Assign(RrMat4d &to,const GfMatrix4d &from) {std::memcpy(to._mtx,from.GetArray(),sizeof(double)*16);}
static void Assign(RrVec3d &to,const GfVec3d &from) {to=RrVec3d(from[0],from[1],from[2]);}
static void Assign(RrVec3i &to,const GfVec3i &from) {to=RrVec3i(from[0],from[1],from[2]);}
static void Assign(double &to,const double &from) {to=from;}
static void Assign(bool &to,const bool &from) {to=from;}
static void Assign(int &to,const int &from) {to=from;}
static void Assign(std::string &to,const std::string &from) {to=from;}
static void Assign(std::vector<int> &to,const std::vector<int> &from) {to=from;}
static void Assign(std::vector<double> &to,const std::vector<double> &from) {to=from;}
static void Assign(std::vector<RrMat4d> &to,const std::vector<GfMatrix4d> &from) {
    to.resize(from.size());for(size_t k=0;k<from.size();++k)Assign(to[k],from[k]);
}
static RrAffineFrameInputs ToRuntime(RigExecAffineFrameInputs in) {
    RrAffineFrameInputs out;
    for(int f=0;f<=int(RigExecAffineField::parent);++f)
        RigExecVisitAffineField(in,RigExecAffineField(f),[&](auto &from) {
            RigExecVisitAffineField(out,RigExecAffineField(f),[&](auto &to){Assign(to,from);});
        });
    Assign(out.targets,in.targets);Assign(out.targetObjects,in.targetObjects);
    return out;
}
static void KernelParity() {
    std::mt19937 random(20261009);
    const auto u=[&](double a,double b){return std::uniform_real_distribution<double>(a,b)(random);};
    const auto coin=[&]{return std::uniform_int_distribution<int>(0,1)(random)==1;};
    const auto pick=[&](std::initializer_list<const char *> options) {
        return std::string(*(options.begin()+std::uniform_int_distribution<size_t>(0,options.size()-1)(random)));
    };
    const auto matrix=[&] {
        GfMatrix4d scale(1),shear(1);
        scale.SetScale(GfVec3d(u(.5,1.5),u(.5,1.5),u(.5,1.5))*(coin()?1.0:-1.0));
        shear[1][0]=u(-.3,.3);shear[2][1]=u(-.3,.3);
        GfMatrix4d m=scale*shear*GfMatrix4d(GfRotation(GfVec3d(u(-1,1),u(-1,1),u(.5,1.5)),u(-180,180)),GfVec3d(0));
        m.SetTranslateOnly(GfVec3d(u(-3,3),u(-3,3),u(-3,3)));
        return m;
    };
    const char *scales[]={"FULL","FIX_SHEAR","ALIGNED","AVERAGE","NONE","NONE_LEGACY"};
    size_t compared=0,failed=0;
    for(int trial=0;trial<4000;++trial) {
        RigExecAffineFrameInputs in;
        for(int f=0;f<=int(RigExecAffineField::parent);++f)
            RigExecVisitAffineField(in,RigExecAffineField(f),[&](auto &out) {
                using T=std::decay_t<decltype(out)>;
                if constexpr(std::is_same_v<T,GfMatrix4d>)out=coin() && coin()?GfMatrix4d(1):matrix();
                else if constexpr(std::is_same_v<T,bool>)out=coin();
                else if constexpr(std::is_same_v<T,double>)out=u(.25,2);
                else if constexpr(std::is_same_v<T,GfVec3d>)out=GfVec3d(u(-1,1),u(-1,1),u(-1,1));
            });
        in.operation=pick({"COPY_LOCATION","COPY_ROTATION","COPY_SCALE","COPY_TRANSFORMS","TRANSFORM_LOCATION",
            "DAMPED_TRACK","STRETCH_TO","ARMATURE","ARMATURE_BLEND","PRESERVE_ORIGIN","LIMIT_ROTATION"});
        in.ownerSpace=pick({"WORLD","LOCAL","CUSTOM","POSE"});
        in.targetSpace=pick({"WORLD","LOCAL","CUSTOM","POSE","LOCAL_OWNER_ORIENT"});
        in.rotationMix=pick({"REPLACE","BEFORE","AFTER","BEFORE_FULL","AFTER_FULL","BEFORE_SPLIT","AFTER_SPLIT"});
        in.mapFrom=pick({"LOCATION","ROTATION","SCALE"});in.mapMix=pick({"ADD","REPLACE"});
        in.trackAxis=pick({"TRACK_X","TRACK_Y","TRACK_Z","TRACK_NEGATIVE_X","TRACK_NEGATIVE_Y","TRACK_NEGATIVE_Z"});
        in.keepAxis=pick({"PLANE_X","PLANE_Z","SWING_Y"});
        in.volume=pick({"NO_VOLUME","VOLUME_XZX","VOLUME_X","VOLUME_Z"});
        in.spaceKind=pick({"pose","rotation","translation"});
        in.inheritScale=scales[random()%6];in.ownerInheritScale=scales[random()%6];in.sourceInheritScale=scales[random()%6];
        in.influence=coin()?1.0:u(0,1);in.axisMask=int(random()%8);in.invertMask=int(random()%8);
        in.mapAxes=GfVec3i(int(random()%3),int(random()%3),int(random()%3));
        in.mapFromMin=GfVec3d(u(-2,0),u(-2,0),u(-2,0));in.mapFromMax=GfVec3d(u(0,2),u(0,2),u(0,2));
        in.bulgeMin=u(0,1);in.bulgeMax=u(1,3);in.bulgeSmooth=u(0,1);
        const size_t targets=random()%4;
        for(size_t k=0;k<targets;++k) {
            in.targets.push_back(matrix());in.targetObjects.push_back(matrix());in.targetBinds.push_back(matrix());
            in.targetWeights.push_back(coin() && coin()?0.0:u(.1,1));
            in.targetIndices.push_back(int(k));in.objectIndices.push_back(int(random()%targets));
        }
        // Now and then a blend whose arrays disagree, which both refuse.
        if(targets && random()%10==0)in.targetWeights.pop_back();
        const RrAffineFrameInputs runtime=ToRuntime(in);
        for(uint32_t kind=0;kind<RigExecAffineFrameTypeCount;++kind) {
            GfMatrix4d native(1);RrMat4d played(1.0);
            const char *nativeFailure=nullptr,*playedFailure=nullptr;
            const bool a=RigExecGfAffineFrameKernel::Compute(kind,in,&native,&nativeFailure);
            const bool b=RrAffineFrameKernel::Compute(kind,runtime,&played,&playedFailure);
            CHECK(a==b);
            if(!a){CHECK(nativeFailure && playedFailure && std::strcmp(nativeFailure,playedFailure)==0);++failed;continue;}
            if(std::memcmp(native.GetArray(),played._mtx,sizeof(double)*16)!=0) {
                std::cerr<<"trial "<<trial<<" kind "<<kind<<" operation "<<in.operation<<" owner "<<in.ownerSpace
                         <<" target "<<in.targetSpace<<" mix "<<in.rotationMix<<"\n native "<<native<<"\n runtime";
                for(int r=0;r<4;++r)for(int c=0;c<4;++c)std::cerr<<" "<<played[r][c];
                std::cerr<<"\n";
                throw std::runtime_error("the runtime's affine frame differs from Gf's");
            }
            ++compared;
        }
    }
    CHECK(compared>20000 && failed>0);
    std::cout<<"affine kernel parity: "<<compared<<" results bit for bit, "<<failed<<" refusals agree\n";
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
    RigExecRigEvaluator evaluator(stage,SdfPath("/Rig"));
    std::vector<std::string> diagnostics;CHECK(evaluator.Compile(&diagnostics));
    std::string before;stage->GetRootLayer()->ExportToString(&before);
    auto pose=evaluator.Evaluate(UsdTimeCode(1));CHECK(pose.valid);
    // The frame-cache worker evaluates the same expressions bit for bit.
    std::string why;
    if(!RigExecFrozenMatchesLive(&evaluator,SdfPath("/Rig"),UsdTimeCode(1),&why))throw std::runtime_error("frozen: "+why);
    for(int i=0;i<6;++i)Near(pose.jointFramesFinal.at(SdfPath(std::string("/Rig/Joints/")+labels[i])).Origin(),expected[i]);
    auto moved=pose.movedProperties.at(points.GetPointsAttr().GetPath()).Get<VtVec3fArray>();Near(GfVec3d(moved[0]),GfVec3d(.5,0,0));Near(GfVec3d(moved[1]),GfVec3d(1.5,1,0));
    // A control edit must invalidate both provider expressions and the external skin payload.
    CHECK(source.GetAttribute(TfToken("avars:tx")).Set(3.0));
    pose=evaluator.Evaluate(UsdTimeCode(1));CHECK(pose.valid);moved=pose.movedProperties.at(points.GetPointsAttr().GetPath()).Get<VtVec3fArray>();Near(GfVec3d(moved[0]),GfVec3d(.75,0,0));Near(GfVec3d(moved[1]),GfVec3d(2.25,1,0));
    if(!RigExecFrozenMatchesLive(&evaluator,SdfPath("/Rig"),UsdTimeCode(1),&why))throw std::runtime_error("frozen: "+why);
    CHECK(source.GetAttribute(TfToken("avars:tx")).Set(2.0));
    std::string after;stage->GetRootLayer()->ExportToString(&after);CHECK(before==after);
    Runtime(stage);
}
int main(int argc,char **argv) {try {
    CHECK(argc==2);PlugRegistry::GetInstance().RegisterPlugins(argv[1]);
    KernelParity();
    Case();
    // Standalone numerical entry point uses no stage or Exec context.
    RigExecAffineFrameInputs in;in.source=Translate(2);in.incoming=Translate(0,3);in.preserveLocation=true;
    Near(RigExecComputeCopyTransforms(in).ExtractTranslation(),GfVec3d(0,3,0));
    std::cout<<"core affine providers + layered skin PASS\n";return 0;
} catch(const std::exception &e) {std::cerr<<e.what()<<"\n";return 1;}}
