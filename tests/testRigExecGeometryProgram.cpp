#include "rigExecGraph/geometryProgram.h"
#include "rigExecGraph/geometrySceneLowering.h"
#include "rigExecGraph/geometryGraphBinding.h"
#include "rigExec/moverGraph.h"
#include <cstdio>
using namespace rigExec;
int main() {
    int failures=0;
#define CHECK(x) do {std::printf("GeometryProgram check line %d: %s\n",__LINE__,#x);std::fflush(stdout);if(!(x)){++failures;std::printf("failure line %d\n",__LINE__);std::fflush(stdout);}}while(0)
    std::puts("GeometryProgram begin");std::fflush(stdout);
    RigExecRevisionKernelTouchTokens();
    std::puts("GeometryProgram tokens ready");std::fflush(stdout);
    RigExecGeometryRecord record;
    record.binding.moverPath=SdfPath("/Rig/Move");
    record.binding.target=SdfPath("/Mesh.points");
    std::puts("GeometryProgram declaring matrix leaves");std::fflush(stdout);
    RigExecDeclareRevisionLeaves(record.op,record.binding.moverPath,record.binding,&record.leaves);
    std::vector<VtValue> sampled;
    for(const auto &key:record.leaves.keys)sampled.push_back(key.fallback);
    GfMatrix4d matrix(1.0);matrix.SetTranslateOnly(GfVec3d(4,2,-1));
    RigExecGeometryInputs inputs;inputs.leaves=&sampled;inputs.transform=&matrix;
    std::puts("GeometryProgram assembling matrix leaves");std::fflush(stdout);
    auto parameters=RigExecAssembleGeometry(record,inputs);
    CHECK(parameters.valid && parameters.enabled);
    RigExecGeometryWorkspace workspace;
    std::vector<GfVec3f> points={{0,0,0},{1,3,5}};
    CHECK(RigExecRunGeometry(record,parameters,&workspace,&points,false));
    CHECK(points==std::vector<GfVec3f>({{4,2,-1},{5,5,4}}));
    parameters.weights=RigExecWeightPacket::Constant(0.5f);
    points={{0,0,0},{1,3,5}};
    CHECK(RigExecRunGeometry(record,parameters,&workspace,&points,false));
    CHECK(points==std::vector<GfVec3f>({{2,1,-0.5f},{3,4,4.5f}}));
    const auto preceding=points;parameters.valid=false;
    CHECK(!RigExecRunGeometry(record,parameters,&workspace,&points,false));
    CHECK(points==preceding);
    CHECK(!RigExecRunGeometry(record,parameters,nullptr,&points,false));
    std::vector<RigExecBlendChannel> channels(1);
    channels[0].weight=0.5f;channels[0].samples.resize(1);
    channels[0].samples[0].activation=1.0f;
    channels[0].samples[0].points={{2,0,0},{1,5,5}};
    std::vector<GfVec3f> deltas;
    CHECK(RigExecGeometryBlendDeltas(channels,{{0,0,0},{1,3,5}},&deltas));
    CHECK(deltas==std::vector<GfVec3f>({{1,0,0},{0,1,0}}));
    // Missing transform is a failed packet, never an invented identity.
    inputs.transform=nullptr;
    CHECK(!RigExecAssembleGeometry(record,inputs).valid);
    RigExecSceneGeometryDescriptor blendDescriptor;
    RigExecSceneGeometryBlendChannel blendChannel;blendChannel.weight=0;
    RigExecSceneGeometryBlendSample blendSample;
    blendSample.activation=1;blendSample.offsets=2;blendSample.indices=3;
    blendSample.sparse=true;blendSample.shapeExists=true;
    blendChannel.samples.push_back(blendSample);blendDescriptor.blendChannels.push_back(blendChannel);
    std::vector<VtValue> blendLeaves={VtValue(0.5f),VtValue(1.0f),
        VtValue(VtVec3fArray{{2,0,0}}),VtValue(VtIntArray{1})};
    std::vector<RigExecBlendChannel> gathered;
    CHECK(RigExecGatherSceneGeometryBlendChannels(blendDescriptor,blendLeaves,2,&gathered));
    if(gathered.empty() || gathered[0].samples.empty()) {
        std::fprintf(stderr,"geometry blend gather did not produce its requested channel/sample\n");
        return 1;
    }
    const auto *layout=gathered[0].samples[0].layout.get();
    CHECK(layout && layout->valid);
    CHECK(RigExecGatherSceneGeometryBlendChannels(blendDescriptor,blendLeaves,2,&gathered));
    CHECK(gathered[0].samples[0].layout.get()==layout);
    CHECK(RigExecGeometryBlendDeltas(gathered,{{0,0,0},{0,0,0}},&deltas));
    CHECK(deltas==std::vector<GfVec3f>({{0,0,0},{1,0,0}}));
    blendLeaves[3]=VtValue(VtIntArray{2});
    CHECK(RigExecGatherSceneGeometryBlendChannels(blendDescriptor,blendLeaves,2,&gathered));
    CHECK(gathered[0].samples[0].layout);
    if(!gathered[0].samples[0].layout)return 1;
    CHECK(!gathered[0].samples[0].layout->valid);
    CHECK(!RigExecGeometryBlendDeltas(gathered,{{0,0,0},{0,0,0}},&deltas));
    // Detached lowering preserves declared provider phase and missing targets.
    RigExecSceneDescriptors scene;
    scene.rigRoot=SdfPath("/Rig");scene.identities={UsdTimeCode::Default()};
    RigExecSceneNodeDescriptor mover;
    mover.fact.path=SdfPath("/Rig/Move");mover.fact.type=TfToken("RigExecMatrixMover");
    mover.fact.active=true;scene.nodes.emplace(mover.fact.path,mover);
    RigExecSceneAttributeDescriptor target;
    target.fact.path=SdfPath("/Mesh.points");target.fact.type=SdfValueTypeNames->Point3fArray;
    scene.attributes.emplace(target.fact.path,target);
    RigExecSceneRelationshipDescriptor relationship;
    relationship.fact.path=SdfPath("/Rig/Move.rigExec:transform");
    relationship.readPhase=TfToken("final");
    relationship.forwardedTargets={SdfPath("/Rig/Missing")};relationship.targetExists={0};
    scene.relationships.emplace(relationship.fact.path,relationship);
    RigExecSceneGeometryDescriptor lowered;
    std::string error;
    CHECK(RigExecLowerSceneGeometry(scene,mover.fact.path,target.fact.path,&lowered,&error));
    CHECK(lowered.record.binding.transform==SdfPath("/Rig/Missing"));
    CHECK(lowered.record.binding.transformPhase.kind==RigExecReadPhaseKind::Final);
    std::vector<VtValue> detachedLeaves;
    CHECK(RigExecResolveSceneGeometryLeaves(scene,lowered,UsdTimeCode::Default(),{},&detachedLeaves,&error));
    CHECK(detachedLeaves.size()==lowered.record.leaves.keys.size());
    scene.relationships[relationship.fact.path].readPhase=TfToken("bad relative path");
    CHECK(!RigExecLowerSceneGeometry(scene,mover.fact.path,target.fact.path,&lowered,&error));
    // The extended settings' frame providers (a Delta Mush's rigExec:frame,
    // a lattice's or a surface's rigExec:frames) bind as influences at their
    // relationship's phase, as each mover's Bind binds them.
    const std::pair<const char *,const char *> framedMovers[]={{"RigExecDeltaMushMover","rigExec:frame"},
        {"RigExecLatticeMover","rigExec:frames"},{"RigExecSurfaceMover","rigExec:frames"}};
    for(const auto &[type,name]:framedMovers) {
        RigExecSceneDescriptors framed;
        framed.rigRoot=SdfPath("/Rig");framed.identities={UsdTimeCode::Default()};
        RigExecSceneNodeDescriptor deformer;
        deformer.fact.path=SdfPath("/Rig/Deform");deformer.fact.type=TfToken(type);deformer.fact.active=true;
        framed.nodes.emplace(deformer.fact.path,deformer);
        framed.attributes.emplace(target.fact.path,target);
        RigExecSceneRelationshipDescriptor frames;
        frames.fact.path=deformer.fact.path.AppendProperty(TfToken(name));frames.readPhase=TfToken("final");
        frames.forwardedTargets={SdfPath("/Rig/CageFrame"),SdfPath("/Rig/TargetFrame")};frames.targetExists={1,1};
        framed.relationships.emplace(frames.fact.path,frames);
        RigExecSceneGeometryDescriptor detached;
        CHECK(RigExecLowerSceneGeometry(framed,deformer.fact.path,target.fact.path,&detached,&error));
        CHECK(detached.record.binding.influences==SdfPathVector({SdfPath("/Rig/CageFrame"),SdfPath("/Rig/TargetFrame")}));
        CHECK(detached.record.binding.transformPhase.kind==RigExecReadPhaseKind::Final);
        framed.relationships[frames.fact.path].readPhase=TfToken("bad relative path");
        CHECK(!RigExecLowerSceneGeometry(framed,deformer.fact.path,target.fact.path,&detached,&error));
    }
    RigExecSceneGeometryDescriptor graphDescriptor;
    graphDescriptor.mover=SdfPath("/Rig/Move");
    graphDescriptor.record.leaves.keys.push_back({SdfPath("/Mesh.points"),
        RigExecRevisionLeafType::Vec3fArray,RigExecRevisionLeafTime::AtDefault,
        RigExecRevisionLeafFlavour::Raw,VtValue(VtVec3fArray())});
    graphDescriptor.bound={1};graphDescriptor.inputs.resize(1);
    graphDescriptor.inputs[0].consumer=SdfPath("/Mesh.points");
    graphDescriptor.inputs[0].source=SdfPath("/Mesh.points");
    RigExecSceneGraphBindingContext link;
    bool capturedRaw=false,capturedDefault=false;
    link.resolve=[&](const RigExecSceneGraphReadRequest &request,RigExecValueId *id,std::string *) {
        capturedRaw=request.raw;capturedDefault=request.atDefault;*id=0;return true;
    };
    RigExecGeometryGraphBinding graphBinding;
    CHECK(RigExecBindGeometryGraph(graphDescriptor,link,&graphBinding,&error));
    CHECK(capturedRaw && capturedDefault && graphBinding.reads.size()==1);
    RigExecTypedValueStore current(1);
    current.PublishSource(0,VtValue(VtVec3fArray{GfVec3f(3,2,1)}));
    CHECK(RigExecReadGeometryGraphLeaves(graphBinding,current,&detachedLeaves,&error));
    CHECK(!detachedLeaves.empty() && detachedLeaves[0].IsHolding<VtVec3fArray>() && !detachedLeaves[0].UncheckedGet<VtVec3fArray>().empty());
    if(detachedLeaves.empty() || !detachedLeaves[0].IsHolding<VtVec3fArray>() || detachedLeaves[0].UncheckedGet<VtVec3fArray>().empty())return 1;
    CHECK(detachedLeaves[0].UncheckedGet<VtVec3fArray>()[0]==GfVec3f(3,2,1));
    current.PublishSource(0,VtValue(VtVec3fArray{GfVec3f(4,2,1)}));
    CHECK(RigExecReadGeometryGraphLeaves(graphBinding,current,&detachedLeaves,&error));
    CHECK(!detachedLeaves.empty() && detachedLeaves[0].IsHolding<VtVec3fArray>() && !detachedLeaves[0].UncheckedGet<VtVec3fArray>().empty());
    if(detachedLeaves.empty() || !detachedLeaves[0].IsHolding<VtVec3fArray>() || detachedLeaves[0].UncheckedGet<VtVec3fArray>().empty())return 1;
    CHECK(detachedLeaves[0].UncheckedGet<VtVec3fArray>()[0]==GfVec3f(4,2,1));
    RigExecSceneGeometryDescriptor ordered;
    ordered.mover=SdfPath("/Rig/Move");ordered.bound={1};ordered.inputs.resize(1);ordered.typedLeaves.resize(1);
    ordered.record.leaves.keys.push_back({SdfPath("/Rig/Move.inputs:amount"),
        RigExecRevisionLeafType::Float,RigExecRevisionLeafTime::AtTime,
        RigExecRevisionLeafFlavour::Resolved,VtValue(float(0))});
    auto &typed=ordered.typedLeaves[0];typed.consumer=SdfPath("/Rig/Move.inputs:amount");
    typed.cppType=typeid(float);typed.readPhase=TfToken("base");
    RigExecSceneBoundInput headHop,middleHop;headHop.consumer=typed.consumer;middleHop.consumer=SdfPath("/Rig/Middle.inputs:value");
    typed.hops={headHop,middleHop};
    link.resolve=[&](const RigExecSceneGraphReadRequest &request,RigExecValueId *id,std::string *) {
        *id=(request.source==typed.consumer?0:2)+(request.raw?0:1);return true;
    };
    CHECK(RigExecBindGeometryGraph(ordered,link,&graphBinding,&error));
    RigExecTypedValueStore overlayStore(4);
    overlayStore.PublishSource(0,VtValue(float(10)));overlayStore.PublishSource(2,VtValue(float(20)));
    overlayStore.PublishSource(3,VtValue(float(99)),false,true);
    CHECK(RigExecReadGeometryGraphLeaves(graphBinding,overlayStore,&detachedLeaves,&error));
    CHECK(!detachedLeaves.empty() && detachedLeaves[0].IsHolding<float>());
    if(detachedLeaves.empty() || !detachedLeaves[0].IsHolding<float>())return 1;
    CHECK(detachedLeaves[0].UncheckedGet<float>()==99);
    overlayStore.PublishSource(1,VtValue(float(7)),false,true);
    CHECK(RigExecReadGeometryGraphLeaves(graphBinding,overlayStore,&detachedLeaves,&error));
    CHECK(!detachedLeaves.empty() && detachedLeaves[0].IsHolding<float>());
    if(detachedLeaves.empty() || !detachedLeaves[0].IsHolding<float>())return 1;
    CHECK(detachedLeaves[0].UncheckedGet<float>()==7);
    overlayStore.PublishSource(1,VtValue(double(7)),false,true);
    CHECK(RigExecReadGeometryGraphLeaves(graphBinding,overlayStore,&detachedLeaves,&error));
    CHECK(!detachedLeaves.empty() && detachedLeaves[0].IsHolding<float>());
    if(detachedLeaves.empty() || !detachedLeaves[0].IsHolding<float>())return 1;
    CHECK(detachedLeaves[0].UncheckedGet<float>()==99);
    RigExecSceneNodeDescriptor mesh;
    mesh.fact.path=SdfPath("/Mesh");mesh.fact.type=TfToken("Mesh");mesh.fact.active=true;mesh.pointBased=true;
    scene.nodes[mesh.fact.path]=mesh;
    RigExecSceneNodeDescriptor projector;
    projector.fact.path=SdfPath("/Rig/Projector");projector.fact.type=TfToken("RigExecSurfaceProjector");
    projector.fact.active=true;scene.nodes[projector.fact.path]=projector;
    std::vector<RigExecSceneGeometryDescriptor> variants;
    CHECK(RigExecLowerSceneGeometryVariants(scene,projector.fact.path,SdfPath("/Mesh.points"),&variants,&error));
    CHECK(variants.size()==1 && variants[0].matrixOutput && variants[0].derived);
    if(variants.empty())return 1;
    CHECK(variants[0].target==SdfPath("/Mesh.primvars:eyeProjector") && variants[0].pointsTarget==SdfPath("/Mesh.points"));
    CHECK(variants[0].record.binding.meshWorldInverse==GfMatrix4d(1));
    RigExecSceneAttributeDescriptor extent;
    extent.fact.path=SdfPath("/Mesh.extent");extent.fact.type=SdfValueTypeNames->Float3Array;
    extent.fact.hasAuthoredValue=true;extent.inputs.resize(1);
    extent.inputs[0].raw=VtValue(VtVec3fArray{GfVec3f(-1),GfVec3f(1)});
    scene.attributes[extent.fact.path]=extent;
    CHECK(RigExecLowerSceneDerivedGeometry(scene,SdfPath("/Mesh.points"),&variants,&error));
    CHECK(variants.size()==1 && variants[0].derived && !variants[0].matrixOutput);
    CHECK(variants[0].record.op==RigExecRevisionOp::RecomputeExtent && variants[0].target==extent.fact.path);
    return failures?1:0;
}
