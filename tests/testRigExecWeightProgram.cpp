#include "rigExecScene/weightProgram.h"
#include "rigExecScene/weightGraphBinding.h"
#include "rigExecScene/sceneTypedReads.h"
#include "rigExecScene/sceneGraphTypedRead.h"
#include "rigExecScene/weightSceneLowering.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace rigExec;
namespace {
void Check(bool condition,const char *message) { if(!condition)throw std::runtime_error(message); }
void Near(float actual,float expected) { Check(std::abs(actual-expected)<1e-6f,"unexpected weight"); }
RigExecWeightRecord Record(int kind,const char *name) {
    RigExecWeightRecord result;result.kind=kind;result.name=name;result.representation=0;
    result.representationToken=TfToken("constant");result.rangePolicy=TfToken("strict");
    result.type=TfToken(kind==0?"RigExecStaticWeight":kind==1?"RigExecDynamicWeight":
        kind==2?"RigExecCombineWeight":kind==3?"RigExecSphereWeight":kind==4?"RigExecPlaneWeight":"RigExecCurveWeight");
    return result;
}
void TestPaintedDynamicAndComposition() {
    std::vector<RigExecWeightRecord> records{Record(0,"painted"),Record(1,"dynamic"),Record(0,"other"),Record(2,"combine")};
    std::vector<RigExecWeightFieldInputs> inputs(4);std::vector<float> result;std::string error;
    inputs[0].scalars[0]=.25f;records[1].base=0;
    inputs[1].scalars[1]=.5f;inputs[1].scalars[2]=2;inputs[1].scalars[3]=.1f;
    Check(RigExecRunWeightField(records,1,inputs,3,nullptr,&result,&error),"dynamic field failed");
    for(float value:result)Near(value,.35f);
    records[0].representation=2;records[0].indices={1};records[0].values={.7f};inputs[0].scalars[0]=.1f;
    Check(RigExecRunWeightField(records,0,inputs,3,nullptr,&result,&error),"sparse field failed");
    Near(result[0],.1f);Near(result[1],.7f);Near(result[2],.1f);
    records[0].indices={1,1};records[0].values={.7f,.2f};
    Check(!RigExecRunWeightField(records,0,inputs,3,nullptr,&result,&error) && result.empty() &&
        error=="duplicate sparse index on painted","duplicate support did not fail atomically");
    records[0].representation=1;records[0].values={.8f,.6f,.4f};inputs[0].scalars[0]=0;
    inputs[2].scalars[0]=.3f;records[3].inputs={0,2};records[3].combineMode=int(RigExecWeightCombine::Subtract);
    Check(RigExecRunWeightField(records,3,inputs,3,nullptr,&result,&error),"composite field failed");
    Near(result[0],.5f);Near(result[1],.3f);Near(result[2],.1f);
    records[3].inputs={2,0};records[3].clamp=true;
    Check(RigExecRunWeightField(records,3,inputs,3,nullptr,&result,&error),"reversed composite failed");
    for(float value:result)Near(value,0);
    records[0].values.pop_back();
    Check(!RigExecRunWeightField(records,0,inputs,3,nullptr,&result,&error) && result.empty() &&
        error=="dense weight cardinality mismatch on painted","dense count loss reused a field");
}
void TestRetainedNestedComposition() {
    std::vector<RigExecWeightRecord> records{Record(0,"a"),Record(0,"b"),Record(0,"c"),Record(2,"fold"),Record(2,"nested")};
    std::vector<RigExecWeightFieldInputs> inputs(5);inputs[0].scalars[0]=.2f;
    inputs[1].scalars[0]=.8f;inputs[2].scalars[0]=.3f;
    records[3].inputs={0,1,2};records[3].clamp=true;
    const float expected[]={.048f,1.0f,0.0f,.8f,.2f,1.3f/3.0f,.192f};
    RigExecWeightFieldWorkspace workspace;std::vector<float> result;std::string error;
    for(int mode=0;mode<7;++mode) {
        records[3].combineMode=mode;
        Check(RigExecRunWeightField(records,3,inputs,3,nullptr,&workspace,&result,&error),"retained ordered fold failed");
        for(float value:result)Near(value,expected[mode]);
    }
    records[3].inputs={0,1};records[3].combineMode=int(RigExecWeightCombine::Average);
    records[4].inputs={3,2};records[4].combineMode=int(RigExecWeightCombine::Multiply);
    Check(RigExecRunWeightField(records,4,inputs,2,nullptr,&workspace,&result,&error),"nested streamed fold failed");
    for(float value:result)Near(value,.15f);
    records[1].staticError="blocked child";
    Check(!RigExecRunWeightField(records,4,inputs,2,nullptr,&workspace,&result,&error) &&
        result.empty() && error=="blocked child","nested fold changed child error or retained partial output");
    records[1].staticError.clear();
    Check(RigExecRunWeightField(records,4,inputs,1,nullptr,&workspace,&result,&error),"nested fold recovery failed");
    Near(result[0],.15f);
    Check(RigExecRunWeightField(records,4,inputs,0,nullptr,&workspace,&result,&error) &&
        result.empty(),"empty nested field changed cardinality");
}
void TestVolumesAndCurrentPhases() {
    std::vector<GfVec3f> points{{0,0,0},{.5f,0,0},{1,0,0}},curve{{0,0,0},{1,0,0}};
    std::vector<RigExecWeightRecord> records{Record(3,"sphere"),Record(4,"plane"),Record(5,"curve")};
    std::vector<RigExecWeightFieldInputs> inputs(3);std::vector<float> result;std::string error;
    for(auto &input:inputs){input.hasPlacement=true;input.rawPoints[1]={false,true,points.data(),points.size()};}
    Check(RigExecRunWeightField(records,0,inputs,3,nullptr,&result,&error),"sphere failed");
    Near(result[0],1);Near(result[1],.5f);Near(result[2],0);
    records[0].samplesInFlight=true;
    std::vector<GfVec3f> unrelated(3,GfVec3f(1));
    inputs[0].phasedPoints[0]={true,true,unrelated.data(),unrelated.size()};
    Check(RigExecRunWeightField(records,0,inputs,3,&points,&result,&error),"preceding field lost current points");
    Near(result[0],1);Near(result[1],.5f);Near(result[2],0);
    Check(!RigExecRunWeightField(records,0,inputs,3,nullptr,&result,&error) && result.empty() &&
        error=="sphere: rigExec:weightTarget reads `preceding` but no in-flight points were supplied",
        "preceding field substituted phased sampleSource for missing current points");
    records[0].samplesInFlight=false;inputs[0].phasedPoints[0]={};
    inputs[0].phasedPoints[1]={true,false,nullptr,0};
    Check(!RigExecRunWeightField(records,0,inputs,3,nullptr,&result,&error) && result.empty() &&
        error=="sphere: could not read the points to sample","unavailable current phase reused raw samples");
    inputs[0].phasedPoints[1]={true,true,points.data(),2};
    Check(!RigExecRunWeightField(records,0,inputs,3,nullptr,&result,&error) && result.empty(),"short phase reused prior count");
    inputs[0].phasedPoints[1]={true,true,points.data(),3};
    Check(RigExecRunWeightField(records,0,inputs,3,nullptr,&result,&error),"phase recovery failed");Near(result[1],.5f);
    inputs[0].phasedPoints[1]={true,true,nullptr,0};
    Check(RigExecRunWeightField(records,0,inputs,0,nullptr,&result,&error) && result.empty(),"available empty phase failed");
    inputs[1].bounds="bounded";inputs[1].scalars[17]=.25f;inputs[1].scalars[18]=.25f;
    points[1][1]=2;
    Check(RigExecRunWeightField(records,1,inputs,3,nullptr,&result,&error),"bounded plane failed");
    Near(result[0],1);Near(result[1],0);Near(result[2],0);
    points[1][1]=0;inputs[2].rawPoints[2]={false,true,curve.data(),curve.size()};
    Check(RigExecRunWeightField(records,2,inputs,3,nullptr,&result,&error),"curve volume failed");
    for(float value:result)Near(value,1);
    inputs[2].phasedPoints[2]={true,true,nullptr,0};
    Check(!RigExecRunWeightField(records,2,inputs,3,nullptr,&result,&error) && result.empty() &&
        error=="curve: rigExec:curve must name exactly one points source","empty phased curve accepted");
}
void TestPacketPolicyAndProfiles() {
    auto record=Record(0,"packet");RigExecWeightPacketInputs inputs;
    inputs.painted={TfToken("constant"),TfToken("strict"),nullptr,0,nullptr,0,.25f};
    const auto packet=RigExecRunWeightPacket(record,inputs);Check(packet.valid,"constant packet failed");
    Near(packet.Resolve(2,3),.25f);
    inputs.painted.defaultWeight=2;Check(!RigExecRunWeightPacket(record,inputs).valid,"strict packet accepted out of range");
    const auto smooth=RigExecBakeWeightFalloff(TfToken("smooth"));Check(smooth.size()==257,"profile count changed");
    Near(smooth.front(),0);Near(smooth[128],.5f);Near(smooth.back(),1);
    const auto undrawn=RigExecBakeWeightFalloff(TfToken("curve"));Check(undrawn.size()==257,"undrawn curve not linear");Near(undrawn[64],.25f);
}
void TestTypedInputPrecisionAndFallback() {
    RigExecSceneDescriptors scene;scene.identities={UsdTimeCode::Default()};
    const SdfPath prim("/Inputs"),head("/Inputs.head"),upstream("/Inputs.upstream"),precise("/Inputs.precise");
    RigExecSceneNodeDescriptor node;node.fact.path=prim;node.fact.active=true;scene.nodes.emplace(prim,node);
    auto attribute=[&](const SdfPath &path,const SdfValueTypeName &type,const VtValue &value,const SdfPathVector &connections) {
        RigExecSceneAttributeDescriptor result;result.fact.path=path;result.fact.type=type;result.fact.connections=connections;
        RigExecSceneInput input;input.raw=value;result.inputs.push_back(input);scene.attributes[path]=std::move(result);
    };
    attribute(head,SdfValueTypeNames->Float,VtValue(.75f),{upstream});
    attribute(upstream,SdfValueTypeNames->Float,VtValue(),{});
    RigExecSceneTypedRead read;VtValue value;std::string error;
    Check(RigExecBindSceneTypedRead(scene,head,SdfValueTypeNames->Float,&read,&error),"float read capture failed");
    Check(RigExecResolveSceneTypedRead(scene,read,UsdTimeCode::Default(),{},&value,&error),"deep empty lost nearest raw fallback");
    Near(value.UncheckedGet<float>(),.75f);
    attribute(head,SdfValueTypeNames->Float,VtValue(.75f),{precise});
    attribute(precise,SdfValueTypeNames->Double,VtValue(3.25),{});
    Check(RigExecBindSceneTypedRead(scene,head,SdfValueTypeNames->Float,&read,&error),"double tail capture failed");
    Check(RigExecResolveSceneTypedRead(scene,read,UsdTimeCode::Default(),{},&value,&error),"normal double tail failed");
    Near(value.UncheckedGet<float>(),3.25f);
    Check(RigExecResolveSceneTypedRead(scene,read,UsdTimeCode::Default(),{{head,VtValue(.5f)}},&value,&error),"float overlay failed");
    Near(value.UncheckedGet<float>(),.5f);
    Check(RigExecBindSceneTypedRead(scene,precise,SdfValueTypeNames->Float,&read,&error),"fresh double capture failed");
    Check(RigExecResolveSceneTypedRead(scene,read,UsdTimeCode::Default(),{{precise,VtValue(9.0f)}},&value,&error),"fresh double read failed");
    Near(value.UncheckedGet<float>(),3.25f);
    scene.attributes[precise].inputs[0].raw=VtValue();
    Check(RigExecBindSceneTypedRead(scene,precise,SdfValueTypeNames->Float,&read,&error),"missing double capture failed");
    Check(!RigExecResolveSceneTypedRead(scene,read,UsdTimeCode::Default(),{{precise,VtValue(9.0f)}},&value,&error) &&
        value.IsEmpty(),"fresh double widened float publication or retained prior value");
    const SdfPath array("/Inputs.points");
    attribute(head,SdfValueTypeNames->Float3,VtValue(GfVec3f(99)),{array});
    attribute(array,SdfValueTypeNames->Point3fArray,VtValue(VtVec3fArray{GfVec3f(1),GfVec3f(2)}),{});
    scene.attributes[array].inputs[0].state=RigExecSceneInputState::Computed;
    scene.attributes[array].inputs[0].source=array;
    scene.attributes[head].readPhase=TfToken("final");
    scene.attributes[head].fact.metadata["rigExecInputElement"]=VtValue(1);
    Check(RigExecBindSceneTypedRead(scene,head,SdfValueTypeNames->Float3,&read,&error),"element binding failed");
    Check(RigExecResolveSceneTypedRead(scene,read,UsdTimeCode::Default(),
        {{array,VtValue(VtVec3fArray{GfVec3f(1),GfVec3f(7)})}},&value,&error),"selected current array failed");
    Check(value.UncheckedGet<GfVec3f>()==GfVec3f(7),"selected wrong current element");
    Check(RigExecResolveSceneTypedRead(scene,read,UsdTimeCode::Default(),
        {{array,VtValue(VtVec3fArray{GfVec3f(1)})}},&value,&error),"selected count loss lost typed fallback");
    Check(value.UncheckedGet<GfVec3f>()==GfVec3f(99) && error.find("element 1 is unavailable")!=std::string::npos,
        "selected count loss used raw/prior array or lost diagnostic");
    Check(RigExecResolveSceneTypedRead(scene,read,UsdTimeCode::Default(),
        {{array,VtValue(VtVec3fArray{GfVec3f(1),GfVec3f(8)})}},&value,&error) && error.empty(),"selection recovery retained stale failure");
    Check(value.UncheckedGet<GfVec3f>()==GfVec3f(8),"selection recovery retained prior value");
    Check(!RigExecResolveSceneTypedRead(scene,read,UsdTimeCode::Default(),{},&value,&error) &&
        value.IsEmpty(),"missing computed array used authored fallback");
    RigExecSceneGraphBindingContext context;
    context.resolve=[&](const RigExecSceneGraphReadRequest &request,RigExecValueId *id,std::string *) {
        *id=request.source==array?(request.raw?2:3):(request.raw?0:1);return true;
    };
    RigExecGraphTypedRead graphRead;
    Check(RigExecBindGraphTypedRead(read,context,&graphRead,&error),"computed element graph binding failed");
    RigExecTypedValueStore store(4);
    store.Publish(0,VtValue(GfVec3f(99)));store.Publish(2,VtValue(VtVec3fArray{GfVec3f(42),GfVec3f(43)}));
    store.Publish(3,VtValue(VtVec3fArray{GfVec3f(1)}));
    Check(RigExecReadGraphTypedRead(graphRead,store,&value,&error) && value.UncheckedGet<GfVec3f>()==GfVec3f(99) &&
        !error.empty(),"short computed graph array did not use typed fallback with diagnostic");
    store.Publish(3,VtValue(VtVec3fArray{GfVec3f(1),GfVec3f(8)}));
    Check(RigExecReadGraphTypedRead(graphRead,store,&value,&error) && value.UncheckedGet<GfVec3f>()==GfVec3f(8) &&
        error.empty(),"computed graph selection did not recover");
    store.values[3].blocked=true;
    Check(!RigExecReadGraphTypedRead(graphRead,store,&value,&error) && value.IsEmpty(),
        "missing computed graph array used authored/prior fallback");
    scene.attributes[head].fact.metadata["rigExecInputElement"]=VtValue(-1);
    Check(!RigExecBindSceneTypedRead(scene,head,SdfValueTypeNames->Float3,&read,&error),"negative selection accepted");
    scene.attributes[head].fact.metadata.erase("rigExecInputElement");
    scene.attributes[array].inputs[0].state=RigExecSceneInputState::Raw;
    Check(RigExecBindSceneTypedRead(scene,head,SdfValueTypeNames->Float3,&read,&error),"ordinary vec binding failed");
    Check(RigExecResolveSceneTypedRead(scene,read,UsdTimeCode::Default(),
        {{array,VtValue(VtVec3fArray{GfVec3f(1),GfVec3f(8)})}},&value,&error) &&
        value.UncheckedGet<GfVec3f>()==GfVec3f(99),"array implicitly coerced without selection metadata");
}
void TestCentralWeightIdsAndPublication() {
    RigExecSceneWeightProgram program;program.root=0;program.records={Record(0,"/Weights/Painted")};
    program.objects.resize(1);program.objects[0].path=SdfPath("/Weights/Painted");
    RigExecSceneTypedRead read;read.consumer=SdfPath("/Weights/Painted.rigExec:defaultWeight");read.readPhase=TfToken("base");
    RigExecSceneBoundInput hop;hop.consumer=read.consumer;read.hops={hop};
    program.objects[0].scalarReads.emplace(0,read);
    RigExecWeightGraphBinding binding;std::string error;
    const auto resolve=[](const SdfPath &,const SdfPath &,RigExecWeightValueRole role,const TfToken &,
                          RigExecValueId *id,std::string *) {
        *id=role==RigExecWeightValueRole::Packet?2:role==RigExecWeightValueRole::Overlay?1:0;return true;
    };
    Check(RigExecBindWeightGraph(program,resolve,&binding,&error),"typed ID binding failed");
    RigExecWeightGraphWorkspace workspace;RigExecPrepareWeightGraphWorkspace(binding,&workspace);
    RigExecTypedValueStore values(4);values.Publish(0,0.4f);values.Publish(1,0.6f);
    Check(RigExecRunBoundWeightField(binding,values,2,UINT64_MAX,&workspace,&error),"ID field failed");
    Check(workspace.result.size()==2,"ID field count lost");Near(workspace.result[0],0.6f);
    RigExecWeightPacket packet;Check(RigExecRunBoundWeightPacket(binding,0,values,&workspace,&packet,&error),"ID packet failed");
    Check(packet.valid,"ID packet invalid");Near(packet.defaultWeight,0.6f);
    Check(RigExecPublishWeightPacket(&values,2,packet),"packet first publication lost");
    Check(!RigExecPublishWeightPacket(&values,2,packet),"identical packet changed");
    packet.defaultWeight=0.0f;Check(RigExecPublishWeightPacket(&values,2,packet),"positive zero publication lost");
    packet.defaultWeight=-0.0f;Check(RigExecPublishWeightPacket(&values,2,packet),"signed zero change lost");
    Check(RigExecPublishWeightField(&values,3,workspace.result,2,true,{}),"field publication lost");
    Check(!RigExecPublishWeightField(&values,3,workspace.result,2,true,{}),"identical field changed");
    Check(RigExecPublishWeightField(&values,3,workspace.result,3,false,"missing"),"invalid count state lost");
    Check(values.values[3].blocked && values.values[3].count==3 && values.values[3].raw.UncheckedGet<VtFloatArray>().empty(),"failure retained prior field");
    Check(RigExecPublishWeightField(&values,3,workspace.result,2,true,{}),"field recovery lost");
    std::vector<float> effective;
    Check(RigExecRunEffectiveWeightField(program.records,-1,{},2,nullptr,0.25f,&effective,&error),"default field failed");
    Check(effective.size()==2,"default field count lost");Near(effective[1],0.25f);
    Check(!RigExecRunEffectiveWeightField(program.records,-1,{},2,nullptr,2.0f,&effective,&error) && effective.empty(),"invalid default carried prior field");
}
void TestCurrentDefaultPaintedArrays() {
    RigExecSceneWeightProgram program;program.root=0;program.records={Record(0,"/Painted")};
    program.records[0].representation=1;program.records[0].representationToken=TfToken("dense");
    program.records[0].values={.1f,.2f};program.objects.resize(1);
    auto &object=program.objects[0];object.path=SdfPath("/Painted");
    object.paintedValues=SdfPath("/Painted.rigExec:values");object.paintedIndices=SdfPath("/Painted.rigExec:indices");
    RigExecSceneGraphBindingContext context;
    context.resolve=[&](const RigExecSceneGraphReadRequest &request,RigExecValueId *id,std::string *) {
        Check(request.raw && request.atDefault,"painted arrays did not bind raw Default rows");
        *id=request.source==object.paintedValues?0:1;return true;
    };
    context.weightPacket=[](const SdfPath &,RigExecValueId *id,std::string *){*id=2;return true;};
    RigExecWeightGraphBinding binding;std::string error;
    Check(RigExecBindWeightGraph(program,context,{},TfToken("base"),&binding,&error),"Default ID binding failed");
    Check(binding.objects[0].fieldReads==std::vector<RigExecValueId>({0,1}) &&
        binding.objects[0].packetReads==std::vector<RigExecValueId>({0,1}),"Default rows absent from graph dependencies");
    RigExecWeightGraphWorkspace workspace;RigExecPrepareWeightGraphWorkspace(binding,&workspace);
    RigExecTypedValueStore store(3);store.Publish(0,VtValue(VtFloatArray{.3f,.4f}));store.Publish(1,VtValue(VtIntArray{}));
    RigExecWeightPacket packet;
    for(const auto &samples:{VtFloatArray{.3f,.4f},VtFloatArray{.7f,.8f}}) {
        store.Publish(0,VtValue(samples));
        Check(RigExecRunBoundWeightField(binding,store,2,UINT64_MAX,&workspace,&error),"current Default field failed");
        Check(RigExecRunBoundWeightPacket(binding,0,store,&workspace,&packet,&error) && packet.valid,"current Default packet failed");
        Near(workspace.result[0],samples[0]);Near(packet.values[1],samples[1]);
    }
    store.Publish(0,VtValue(VtFloatArray{.5f}));
    Check(!RigExecRunBoundWeightField(binding,store,2,UINT64_MAX,&workspace,&error) && workspace.result.empty(),
        "Default count loss retained prior field");
    store.Publish(0,VtValue(VtFloatArray{.2f,.9f}));
    Check(RigExecRunBoundWeightField(binding,store,2,UINT64_MAX,&workspace,&error),"Default count recovery failed");
    Near(workspace.result[1],.9f);
    // Retained sparse marks must not survive an invocation or count change.
    binding.records[0].representation=2;binding.records[0].representationToken=TfToken("sparse");
    store.Publish(0,VtValue(VtFloatArray{.6f}));store.Publish(1,VtValue(VtIntArray{1}));
    Check(RigExecRunBoundWeightField(binding,store,2,UINT64_MAX,&workspace,&error),"sparse current Default failed");
    Check(RigExecRunBoundWeightField(binding,store,2,UINT64_MAX,&workspace,&error),"sparse retained marks leaked");
    store.Publish(0,VtValue(VtFloatArray{.6f,.7f}));store.Publish(1,VtValue(VtIntArray{1,1}));
    Check(!RigExecRunBoundWeightField(binding,store,2,UINT64_MAX,&workspace,&error) &&
        error=="duplicate sparse index on /Painted","sparse duplicate diagnostic changed");
    store.Publish(0,VtValue(VtFloatArray{.8f}));store.Publish(1,VtValue(VtIntArray{0}));
    Check(RigExecRunBoundWeightField(binding,store,1,UINT64_MAX,&workspace,&error),"sparse count/error recovery failed");
    Near(workspace.result[0],.8f);
    store.Publish(0,VtValue(VtFloatArray{.8f,.2f}));store.Publish(1,VtValue(VtIntArray{1,0}));
    Check(RigExecRunBoundWeightPacket(binding,0,store,&workspace,&packet,&error) && packet.valid,
        "cached sparse packet failed");
    Check(packet.indices==std::vector<int>({0,1}),"sparse canonical support changed");Near(packet.values[0],.2f);
    store.Publish(0,VtValue(VtFloatArray{.6f,.3f}));
    Check(RigExecRunBoundWeightPacket(binding,0,store,&workspace,&packet,&error) && packet.valid,
        "same support numeric packet edit failed");Near(packet.values[0],.3f);Near(packet.values[1],.6f);
    store.Publish(0,VtValue(VtFloatArray{.8f,.1f,.4f}));store.Publish(1,VtValue(VtIntArray{2,0,2}));
    Check(RigExecRunBoundWeightPacket(binding,0,store,&workspace,&packet,&error) && !packet.valid &&
        packet.indices==std::vector<int>({0,2}),"duplicate support changed partial invalid packet");
    Near(packet.values[0],.1f);Near(packet.values[1],.4f);
}
void TestCompositionUsesCommonCycleReporter() {
    RigExecSceneDescriptors scene;scene.identities={UsdTimeCode::Default()};scene.rigRoot=SdfPath("/Weights/Rig");
    const SdfPath a("/Weights/A"),b("/Weights/B"),geometry("/Weights/Geometry"),points("/Weights/Geometry.points");
    for(const auto &path:{a,b,geometry}) {
        RigExecSceneNodeDescriptor node;node.fact.path=path;node.fact.active=true;
        node.fact.type=TfToken(path==geometry?"Mesh":"RigExecDynamicWeight");node.pointBased=path==geometry;
        scene.nodes.emplace(path,std::move(node));
    }
    RigExecSceneAttributeDescriptor attr;attr.fact.path=points;attr.fact.type=SdfValueTypeNames->Point3fArray;
    RigExecSceneInput raw;raw.raw=VtValue(VtVec3fArray{GfVec3f(0)});attr.inputs.push_back(raw);scene.attributes.emplace(points,attr);
    auto relation=[&](const SdfPath &path,const char *name,const SdfPath &target) {
        RigExecSceneRelationshipDescriptor r;r.fact.path=path.AppendProperty(TfToken(name));r.fact.targets={target};
        scene.relationships.emplace(r.fact.path,std::move(r));
    };
    relation(a,"rigExec:baseWeight",b);relation(b,"rigExec:baseWeight",a);
    relation(a,"rigExec:weightTarget",points);relation(b,"rigExec:weightTarget",points);
    RigExecSceneWeightProgram program;std::string error;
    Check(RigExecLowerSceneWeight(scene,a,&program,&error),"capture rejected a cycle before common graph compilation");
    Check(program.records.size()==2 && program.records[0].base==1 && program.records[1].base==0,"capture lost cyclic producer IDs");
    std::vector<RigExecOpDescriptor> ops(3);
    for(size_t i=0;i<2;++i){ops[i].key=program.records[i].name;ops[i].writes={i};ops[i].reads={RigExecValueId(program.records[i].base)};}
    ops[2].key="unrelated";ops[2].writes={2};RigExecCompiledGraph graph;
    Check(RigExecCompileOpGraph(ops,{},RigExecCyclePolicy::SetAside,&graph,&error),"common cycle compile failed");
    Check(graph.cycles.size()==1 && graph.canonicalIndex[0]<0 && graph.canonicalIndex[1]<0 &&
        graph.canonicalIndex[2]>=0,"common reporter failed to preserve unrelated operation");
    std::vector<RigExecWeightFieldInputs> inputs(2);for(size_t i=0;i<2;++i)inputs[i].blocked=graph.canonicalIndex[i]<0;
    std::vector<float> values;
    Check(!RigExecRunWeightField(program.records,program.root,inputs,1,nullptr,&values,&error) &&
        values.empty() && error=="operation cycle","blocked field followed a cyclic closure");
}
void TestVolumeEnvelopeTargetDomains() {
    for(const auto &targetType:{SdfValueTypeNames->Float,SdfValueTypeNames->Matrix4d}) {
        RigExecSceneDescriptors scene;scene.identities={UsdTimeCode::Default()};scene.rigRoot=SdfPath("/Asset/Rig");
        const SdfPath weight("/Asset/Weight"),geometry("/Asset/Samples"),owner("/Asset/Target");
        const auto points=geometry.AppendProperty(TfToken("points")),target=owner.AppendProperty(TfToken("value"));
        for(const auto &path:{weight,geometry,owner}) {
            RigExecSceneNodeDescriptor node;node.fact.path=path;node.fact.active=true;
            node.fact.type=TfToken(path==weight?"RigExecSphereWeight":path==geometry?"Mesh":"Scope");
            node.pointBased=path==geometry;scene.nodes.emplace(path,std::move(node));
        }
        const VtValue targetValue=targetType==SdfValueTypeNames->Float?VtValue(.25f):VtValue(GfMatrix4d(1));
        auto attribute=[&](const SdfPath &path,const SdfValueTypeName &type,const VtValue &value) {
            RigExecSceneAttributeDescriptor a;a.fact.path=path;a.fact.type=type;
            RigExecSceneInput input;input.raw=value;a.inputs.push_back(input);scene.attributes.emplace(path,std::move(a));
        };
        attribute(points,SdfValueTypeNames->Point3fArray,VtValue(VtVec3fArray{GfVec3f(.5f,0,0)}));
        attribute(target,targetType,targetValue);
        auto relation=[&](const char *name,const SdfPath &source) {
            RigExecSceneRelationshipDescriptor r;r.fact.path=weight.AppendProperty(TfToken(name));r.fact.targets={source};
            scene.relationships.emplace(r.fact.path,std::move(r));
        };
        relation("rigExec:sampleSource",geometry);relation("rigExec:weightTarget",target);
        RigExecSceneWeightProgram program;std::string error;
        Check(RigExecLowerSceneWeight(scene,weight,&program,&error),"volume envelope capture refused target domain");
        Check(program.objects[0].points[0].pointArrays==std::vector<char>({1}) &&
            program.objects[0].points[1].pointArrays==std::vector<char>({0}),"volume target raw type fact lost");
        RigExecSceneGraphBindingContext context;
        context.resolve=[&](const RigExecSceneGraphReadRequest &request,RigExecValueId *id,std::string *) {
            if(request.domain==RigExecSceneValueDomain::Points)
                Check(request.source==points,"scalar/frame weight target requested a points producer");
            *id=request.source==points?0:request.source==target?1:2;return true;
        };
        context.weightPacket=[](const SdfPath &,RigExecValueId *id,std::string *){*id=3;return true;};
        RigExecWeightGraphBinding binding;
        Check(RigExecBindWeightGraph(program,context,owner,TfToken("base"),&binding,&error),"volume envelope target binding failed");
        Check(binding.objects[0].rawPoints[1]==UINT64_MAX,"nonarray target acquired a raw points slot");
        RigExecTypedValueStore store(4);store.Publish(0,VtValue(VtVec3fArray{GfVec3f(.5f,0,0)}));
        store.Publish(1,targetValue);store.Publish(2,GfMatrix4d(1));
        RigExecWeightGraphWorkspace workspace;RigExecPrepareWeightGraphWorkspace(binding,&workspace);
        Check(RigExecRunBoundWeightField(binding,store,1,UINT64_MAX,&workspace,&error),"singleton sample failed for envelope target");
        Near(workspace.result[0],.5f);
        store.Publish(0,VtValue(VtVec3fArray{GfVec3f(0),GfVec3f(.5f,0,0)}));
        Check(!RigExecRunBoundWeightField(binding,store,1,UINT64_MAX,&workspace,&error) && workspace.result.empty() &&
            error=="/Asset/Weight: sampled point count does not match the target","volume envelope aggregated samples or retained output");
        store.Publish(0,VtValue(VtVec3fArray{GfVec3f(.5f,0,0)}));
        Check(RigExecRunBoundWeightField(binding,store,1,UINT64_MAX,&workspace,&error),"volume envelope sample recovery failed");
        Near(workspace.result[0],.5f);
    }
}
}
int main() {
    try { TestPaintedDynamicAndComposition();TestRetainedNestedComposition();TestVolumesAndCurrentPhases();TestPacketPolicyAndProfiles();TestTypedInputPrecisionAndFallback();TestCompositionUsesCommonCycleReporter();TestCentralWeightIdsAndPublication();TestCurrentDefaultPaintedArrays();TestVolumeEnvelopeTargetDomains(); }
    catch(const std::exception &error){std::cerr<<error.what()<<'\n';return 1;}
    std::cout<<"Shared weight records, numerical fields, packet policy and phase recovery passed\n";return 0;
}
