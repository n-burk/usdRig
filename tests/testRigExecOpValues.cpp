#include "rigExec/bakedOpValues.h"
#include "rigExec/bakedProgramImpl.h"
#include "pxr/base/vt/value.h"
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>

using namespace rigExec;
namespace {
int failures=0;
#define CHECK(value) do { if(!(value)) { ++failures; \
    std::printf("FAIL %s:%d: %s\n",__FILE__,__LINE__,#value); } } while(0)
std::string Key(const RigExecBakedProgramImpl &B, RigExecBakedSlotDomain domain,
    uint32_t slot=0)
{
    std::string key; RigExecBakedOpValueKey(B,domain,slot,&key); return key;
}
float FloatBits(uint32_t bits)
{
    float value; std::memcpy(&value,&bits,sizeof(value)); return value;
}
void TestFloatPayloadBits()
{
    RigExecBakedProgramImpl B;
    B.poseWeights={0.0f};
    const auto positive=Key(B,RigExecBakedSlotDomain::PoseWeight);
    B.poseWeights[0]=-0.0f;
    CHECK(Key(B,RigExecBakedSlotDomain::PoseWeight)!=positive);
    B.poseWeights[0]=FloatBits(UINT32_C(0x7fc00001));
    const auto firstNaN=Key(B,RigExecBakedSlotDomain::PoseWeight);
    B.poseWeights[0]=FloatBits(UINT32_C(0x7fc00002));
    CHECK(Key(B,RigExecBakedSlotDomain::PoseWeight)!=firstNaN);
    B.poseWeights[0]=FloatBits(UINT32_C(0xffc00001));
    CHECK(Key(B,RigExecBakedSlotDomain::PoseWeight)!=firstNaN);
}
void TestFieldValidityCountError()
{
    RigExecBakedProgramImpl B;
    B.weightFields.resize(1);
    auto &field=B.weightFields[0]; field.ok=true; field.values={0.25f};
    const auto valid=Key(B,RigExecBakedSlotDomain::WeightField);
    field.changed=true;
    CHECK(Key(B,RigExecBakedSlotDomain::WeightField)==valid);
    field.ok=false;
    const auto invalid=Key(B,RigExecBakedSlotDomain::WeightField);
    CHECK(invalid!=valid);
    field.count=7;
    CHECK(Key(B,RigExecBakedSlotDomain::WeightField)!=invalid);
    field.count=0;
    field.error="point count differs";
    CHECK(Key(B,RigExecBakedSlotDomain::WeightField)!=invalid);
    field.ok=true; field.error.clear(); field.values.push_back(0.25f);
    CHECK(Key(B,RigExecBakedSlotDomain::WeightField)!=valid);
    field.values={0.25f};
    std::string retained; retained.reserve(4096);
    RigExecBakedOpValueKey(B,RigExecBakedSlotDomain::WeightField,0,&retained);
    const char *buffer=retained.data(); const size_t capacity=retained.capacity();
    for(int i=0;i<32;++i) {
        RigExecBakedOpValueKey(B,RigExecBakedSlotDomain::WeightField,0,&retained);
        CHECK(retained==valid && retained.data()==buffer && retained.capacity()==capacity);
    }
}
void TestPropertyValidityAndLadderState()
{
    RigExecBakedProgramImpl B;
    B.propertyValues.resize(1); B.propertyVersionValid={0};
    B.propertyChains.resize(1); B.propertyVersionCount=1; B.propertyRecordById={-1};
    const auto absent=Key(B,RigExecBakedSlotDomain::PropertyResult);
    B.propertyVersionValid[0]=1;
    CHECK(Key(B,RigExecBakedSlotDomain::PropertyResult)!=absent);
    B.restRoundTrip={GfMatrix4d(1.0)}; B.defaultRoundTrip={GfMatrix4d(1.0)};
    B.selfD={GfMatrix4d(1.0)}; B.parentDinv={GfMatrix4d(1.0)};
    B.posedAuthored={0}; B.posedAuthoredM={GfMatrix4d(1.0)};
    B.rotOrder={TfToken("XYZ")}; B.posedD={GfMatrix4d(1.0)};
    B.parentSpaceM={GfMatrix4d(1.0)}; B.parentSpaceAuthored={0}; B.rotationSign={0};
    const auto ladder=Key(B,RigExecBakedSlotDomain::Ladder);
    B.selfD[0][3][0]=0.125;
    CHECK(Key(B,RigExecBakedSlotDomain::Ladder)!=ladder);
    B.selfD[0][3][0]=0.0; B.parentDinv[0][3][0]=-0.0;
    CHECK(Key(B,RigExecBakedSlotDomain::Ladder)!=ladder);
    B.parentDinv[0][3][0]=0.0; B.posedAuthored[0]=1;
    CHECK(Key(B,RigExecBakedSlotDomain::Ladder)!=ladder);
    B.posedAuthored[0]=0; B.posedAuthoredM[0][0][1]=0.25;
    CHECK(Key(B,RigExecBakedSlotDomain::Ladder)!=ladder);
    B.posedAuthoredM[0][0][1]=0.0; B.rotOrder[0]=TfToken("ZYX");
    CHECK(Key(B,RigExecBakedSlotDomain::Ladder)!=ladder);
    B.rotOrder[0]=TfToken("XYZ"); B.posedD[0][0][1]=0.5;
    CHECK(Key(B,RigExecBakedSlotDomain::Ladder)!=ladder);
    B.posedD[0][0][1]=0.0; B.parentSpaceM[0][1][0]=0.5;
    CHECK(Key(B,RigExecBakedSlotDomain::Ladder)!=ladder);
    B.parentSpaceM[0][1][0]=0.0; B.parentSpaceAuthored[0]=1;
    CHECK(Key(B,RigExecBakedSlotDomain::Ladder)!=ladder);
    B.parentSpaceAuthored[0]=0; B.rotationSign[0]=3;
    CHECK(Key(B,RigExecBakedSlotDomain::Ladder)!=ladder);
}
void TestChunkRangeIsolation()
{
    RigExecBakedProgramImpl B;
    B.chains.resize(1); B.chains[0].revisions.resize(1);
    B.revisionIndex={{0,0}}; B.revisionChunkBase={0}; B.revisionChunkCount={2};
    auto &revision=B.chains[0].revisions[0];
    revision.stagingOutput={GfVec3f(0.0f),GfVec3f(1.0f)}; revision.chunks.resize(2);
    revision.chunks[0].begin=0; revision.chunks[0].end=1; revision.chunks[0].ok=true;
    revision.chunks[1].begin=1; revision.chunks[1].end=2; revision.chunks[1].ok=true;
    const auto first=Key(B,RigExecBakedSlotDomain::RevisionOut,0);
    const auto second=Key(B,RigExecBakedSlotDomain::RevisionOut,1);
    revision.stagingOutput[1][0]=2.0f;
    CHECK(Key(B,RigExecBakedSlotDomain::RevisionOut,0)==first);
    CHECK(Key(B,RigExecBakedSlotDomain::RevisionOut,1)!=second);
    revision.stagingOutput[0][0]=-0.0f;
    CHECK(Key(B,RigExecBakedSlotDomain::RevisionOut,0)!=first);
    revision.stagingOutput[0][0]=0.0f; revision.chunks[0].ok=false;
    CHECK(Key(B,RigExecBakedSlotDomain::RevisionOut,0)!=first);
    revision.chunks[0].ok=true; revision.stagingOutput.clear();
    const auto missing=Key(B,RigExecBakedSlotDomain::RevisionOut,0);
    CHECK(!missing.empty() && missing[0]==1 && missing!=first);
}
void TestPacketStatusAndOpaqueBoundary()
{
    RigExecBakedProgramImpl B;
    B.chains.resize(1); B.chains[0].revisions.resize(1); B.revisionIndex={{0,0}};
    auto &revision=B.chains[0].revisions[0];
    revision.parameters.valid=true; revision.parameters.kind=TfToken("matrix");
    revision.status.state=TfToken("ok");
    const auto initial=Key(B,RigExecBakedSlotDomain::RevisionPacket);
    revision.ran=true; revision.staticDirty=true;
    CHECK(Key(B,RigExecBakedSlotDomain::RevisionPacket)==initial);
    revision.parameters.referenceVolume=-0.0;
    CHECK(Key(B,RigExecBakedSlotDomain::RevisionPacket)!=initial);
    revision.parameters.referenceVolume=0.0;
    revision.status.firstBadAddress="/Rig/Mover.inputs:weight";
    CHECK(Key(B,RigExecBakedSlotDomain::RevisionPacket)!=initial);
    CHECK(RigExecBakedOpValueKeyIsExact(B,RigExecBakedSlotDomain::RevisionPacket,0));
    revision.parameters.externalData=VtValue(std::string("plugin-owned payload"));
    CHECK(!RigExecBakedOpValueKeyIsExact(B,RigExecBakedSlotDomain::RevisionPacket,0));
}
void TestAvarEffectiveSelection()
{
    RigExecBakedProgramImpl B; RigExecBakedStep step;
    step.kind=RigExecBakedStepKind::AvarInputs; step.object=0;
    step.readerWalks={0}; step.leaves={0};
    B.headLeaves.resize(1); B.headLeaves[0].type=RigExecBakedHeadValueType::Double;
    B.headLeaves[0].typeMatches=true; B.headLeaves[0].value=VtValue(double(3));
    B.readerWalks.resize(1); auto &walk=B.readerWalks[0];
    walk.walk.flavour=RigExecBakedWalk::Flavour::Connected;
    walk.walk.type=RigExecBakedHeadValueType::Double;
    RigExecBakedWalkHop hop; hop.leaf=0; walk.walk.hops={hop}; walk.leaves={0};
    RigExecBakedProgramImpl::AvarBinding binding;
    binding.slot=0; binding.input.walk=0; binding.input.constant=2;
    binding.input.varying=false; B.avarConstantBindings={binding};
    auto &input=B.avarConstantBindings[0].input;
    std::vector<uint32_t> covered; std::string selected, key, raw;
    CHECK(RigExecBakedLeafRead(B,input)==3);
    CHECK(RigExecBakedOpEffectiveInputKey(B,step,&selected,&covered));
    CHECK(RigExecBakedOpInputKey(B,step,&raw));
    input.constant=91; // Frozen input patch, successful selected walk unchanged.
    CHECK(RigExecBakedLeafRead(B,input)==3);
    CHECK(RigExecBakedOpEffectiveInputKey(B,step,&key,&covered) && key==selected);
    CHECK(RigExecBakedOpInputKey(B,step,&key) && key!=raw);
    B.headLeaves[0].value=VtValue(double(4));
    CHECK(RigExecBakedOpEffectiveInputKey(B,step,&key,&covered) && key!=selected);
    B.headLeaves[0].value=VtValue();
    CHECK(RigExecBakedLeafRead(B,input)==91);
    CHECK(RigExecBakedOpEffectiveInputKey(B,step,&key,&covered));const auto absent=key;
    input.constant=92;
    CHECK(RigExecBakedOpEffectiveInputKey(B,step,&key,&covered) && key!=absent);
    B.headLeaves[0].value=VtValue(float(4));
    CHECK(RigExecBakedLeafRead(B,input)==92); // Wrong type takes the argument fallback.
    B.headLeaves[0].value=VtValue(double(0));
    CHECK(RigExecBakedOpEffectiveInputKey(B,step,&key,&covered));const auto zero=key;
    B.headLeaves[0].value=VtValue(double(-0.0));
    CHECK(RigExecBakedOpEffectiveInputKey(B,step,&key,&covered) && key!=zero);
    uint64_t nanBits=UINT64_C(0x7ff8000000000001);double nan;
    std::memcpy(&nan,&nanBits,sizeof(nan));B.headLeaves[0].value=VtValue(nan);
    CHECK(RigExecBakedOpEffectiveInputKey(B,step,&key,&covered));const auto nanKey=key;
    input.constant=-5;
    CHECK(RigExecBakedOpEffectiveInputKey(B,step,&key,&covered) && key==nanKey);
    ++nanBits;std::memcpy(&nan,&nanBits,sizeof(nan));B.headLeaves[0].value=VtValue(nan);
    CHECK(RigExecBakedOpEffectiveInputKey(B,step,&key,&covered) && key!=nanKey);
    input.walk=999;
    CHECK(!RigExecBakedOpEffectiveInputKey(B,step,&key,&covered));
    input.walk=-1; input.leaf=0; step.readerWalks.clear(); step.leaves.clear();
    B.leaves.Of<double>().value={7};
    CHECK(RigExecBakedLeafRead(B,input)==7);
    CHECK(RigExecBakedOpEffectiveInputKey(B,step,&key,&covered));const auto leafKey=key;
    input.constant=101;
    CHECK(RigExecBakedOpEffectiveInputKey(B,step,&key,&covered) && key==leafKey);
    input.leaf=-1;
    CHECK(RigExecBakedLeafRead(B,input)==101);
    CHECK(RigExecBakedOpEffectiveInputKey(B,step,&key,&covered));const auto constantKey=key;
    input.constant=102;
    CHECK(RigExecBakedOpEffectiveInputKey(B,step,&key,&covered) && key!=constantKey);
}

void TestRawInputAndProviderKeys()
{
    RigExecBakedProgramImpl B; RigExecBakedStep step;
    B.leaves.Of<double>().value={0.0};
    B.leafRefs.push_back({RigExecBakedLeafType::Double,0});
    step.bindingLeaves={0}; std::string key;
    CHECK(RigExecBakedOpInputKey(B,step,&key)); const auto first=key;
    B.leaves.Of<double>().changed={1};
    CHECK(RigExecBakedOpInputKey(B,step,&key) && key==first);
    B.leaves.Of<double>().value[0]=-0.0;
    CHECK(RigExecBakedOpInputKey(B,step,&key) && key!=first);
    B.headLeaves.resize(1); B.headLeaves[0].value=VtValue(float(1));
    B.headOverrides={VtValue(float(2))}; B.readerWalks.resize(1);
    B.readerWalks[0].leaves={0}; B.readerWalks[0].slots={0}; step.readerWalks={0};
    CHECK(RigExecBakedOpInputKey(B,step,&key)); const auto walk=key;
    B.headOverrides[0]=VtValue(float(3));
    CHECK(RigExecBakedOpInputKey(B,step,&key) && key!=walk);
    step.bindingLeaves={999}; CHECK(!RigExecBakedOpInputKey(B,step,&key));
    step=RigExecBakedStep(); step.kind=RigExecBakedStepKind::AvarInputs; step.object=0;
    RigExecBakedProgramImpl::AvarBinding binding; binding.slot=0; binding.input.leaf=0;
    B.avarBindings.push_back(binding);
    CHECK(RigExecBakedOpInputKey(B,step,&key)); const auto avar=key;
    B.leaves.Of<double>().value[0]=0.25;
    CHECK(RigExecBakedOpInputKey(B,step,&key) && key!=avar);
    B.chains.resize(1); B.chains[0].sampledHaveBase=true;
    B.chains[0].sampledBase=VtVec3fArray{GfVec3f(0)};
    const auto incoming=Key(B,RigExecBakedSlotDomain::ChainInput);
    B.chains[0].sampledBase[0][0]=-0.0f;
    CHECK(Key(B,RigExecBakedSlotDomain::ChainInput)!=incoming);
    B.chains[0].sampledBase[0][0]=0.0f; B.chains[0].sampledHaveBase=false;
    CHECK(Key(B,RigExecBakedSlotDomain::ChainInput)!=incoming);
    B.providerProgram.sampled.resize(1); B.providerLeaves.values={VtValue(double(0))};
    B.providerLeafBlocked={0};
    const auto raw=Key(B,RigExecBakedSlotDomain::SpaceLeaf);
    B.providerLeafBlocked[0]=1; CHECK(Key(B,RigExecBakedSlotDomain::SpaceLeaf)!=raw);
    B.providerLeafBlocked[0]=0;
    B.providerLeaves.values[0]=VtValue(double(-0.0));
    CHECK(Key(B,RigExecBakedSlotDomain::SpaceLeaf)!=raw);
    B.providerLeaves.values[0]=VtValue(double(0));
    B.routedOverrides[B.providerProgram.sampled[0].attribute]=VtValue(double(0));
    CHECK(Key(B,RigExecBakedSlotDomain::SpaceLeaf)!=raw);
    B.providerValues.values.resize(1); auto &state=B.providerValues.values[0];
    state.value=double(0); state.initialized=true;
    const auto provider=Key(B,RigExecBakedSlotDomain::SpaceValue);
    state.changed=true; state.revision=9;
    CHECK(Key(B,RigExecBakedSlotDomain::SpaceValue)==provider);
    state.value=double(-0.0); CHECK(Key(B,RigExecBakedSlotDomain::SpaceValue)!=provider);
    state.value=double(0); state.authoritative=true;
    CHECK(Key(B,RigExecBakedSlotDomain::SpaceValue)!=provider);
}

void TestConstraintSourceAndPropertyAliasKeys()
{
    RigExecBakedProgramImpl B; RigExecBakedStep step;
    step.kind=RigExecBakedStepKind::Constraint; step.object=0;
    B.walkSteps.resize(1); B.walkSteps[0].index=0;
    B.constraints.resize(1); B.constraints[0].sourceNatives={0};
    B.nativeFrames.resize(1); B.nativeFrameOk={1};
    B.deltaBaseOk={1}; B.deltaBaseMatrix={GfMatrix4d(1)};
    B.constraints[0].deltaBase=0;
    std::string key;
    CHECK(RigExecBakedOpInputKey(B,step,&key)); const auto initial=key;
    B.nativeFrames[0].points[0][0]=-0.0f;
    CHECK(RigExecBakedOpInputKey(B,step,&key) && key!=initial);
    B.nativeFrames[0].points[0][0]=0.0f; B.nativeFrameOk[0]=0;
    CHECK(RigExecBakedOpInputKey(B,step,&key) && key!=initial);
    B.nativeFrameOk[0]=1; B.deltaBaseMatrix[0][0][0]=2;
    CHECK(RigExecBakedOpInputKey(B,step,&key) && key!=initial);
    B.propertyChains.resize(1); B.propertyVersionCount=2;
    B.propertyVersionValid={1,0}; B.propertyRecords.resize(1);
    B.propertyRecords[0].id=1; B.propertyRecordById={-1,0};
    B.recordStoodAside={0}; B.recordValues={VtValue(float(1))};
    const auto alias=Key(B,RigExecBakedSlotDomain::PropertyResult,1);
    B.recordValues[0]=VtValue(float(2));
    CHECK(Key(B,RigExecBakedSlotDomain::PropertyResult,1)!=alias);
    B.recordValues[0]=VtValue(float(1)); B.propertyVersionValid[0]=0;
    CHECK(Key(B,RigExecBakedSlotDomain::PropertyResult,1)!=alias);
    B.propertyVersionValid[0]=1; B.recordStoodAside[0]=1;
    CHECK(Key(B,RigExecBakedSlotDomain::PropertyResult,1)!=alias);
}

void TestSkinEffectiveSelectedTopology()
{
    RigExecBakedProgramImpl B;
    B.chains.resize(1);B.chains[0].revisions.resize(1);B.revisionIndex={{0,0}};
    auto &r=B.chains[0].revisions[0];r.op=RigExecRevisionOp::Skin;
    RigExecDeclareSkinLayoutLeaves(SdfPath("/Skin"),&r.leaves.decl);
    r.leaves.values={VtValue(VtIntArray{0}),VtValue(VtFloatArray{1.0f}),VtValue(1)};
    for(uint32_t k=0;k<3;++k)B.pathLeafRefs.push_back({RigExecBakedPathLeafOwner::Revision,0,0,k});
    RigExecBakedStep step;step.kind=RigExecBakedStepKind::RevisionStatic;step.object=0;
    step.bindingLeaves={0,1,2};
    auto key=[&](bool effective){std::string out;std::vector<uint32_t> covered;CHECK(effective?RigExecBakedOpEffectiveInputKey(B,step,&out,&covered):RigExecBakedOpInputKey(B,step,&out));return out;};
    auto topology=std::make_shared<RigExecSkinTopology>();
    topology->indices={0};topology->weights={1.0f};topology->elementSize=1;
    topology->pointCount=1;topology->influenceCount=1;topology->validated=true;
    r.topologyResolved=true;r.topology=topology;r.layoutHandle=topology;
    const auto validTopology=Key(B,RigExecBakedSlotDomain::SkinTopology);
    topology->validated=false;
    CHECK(Key(B,RigExecBakedSlotDomain::SkinTopology)!=validTopology);
    const auto selected=key(true),raw=key(false);
    r.leaves.values={VtValue(),VtValue(),VtValue()};
    CHECK(key(true)==selected);CHECK(key(false)!=raw);
    // A stale packet topology cannot shadow current null-handle fallback.
    r.layoutHandle.reset();
    const auto unavailable=key(true);
    r.leaves.values={VtValue(VtIntArray{1}),VtValue(VtFloatArray{0.5f}),VtValue(2)};
    CHECK(key(true)!=unavailable);
    // A new current handle shadows raw rows before the packet adopts it.
    r.topology.reset();r.topologyResolved=false;r.layoutHandle=topology;
    const auto newlySelected=key(true);r.leaves.values[2]=VtValue(3);
    CHECK(key(true)==newlySelected);
    r.layoutHandle.reset();
    const auto unresolved=key(true);r.leaves.values[2]=VtValue(4);
    CHECK(key(true)!=unresolved);
}
}
int main()
{
    TestSkinEffectiveSelectedTopology(); TestFloatPayloadBits(); TestFieldValidityCountError(); TestPropertyValidityAndLadderState();
    TestChunkRangeIsolation(); TestConstraintSourceAndPropertyAliasKeys();
    TestPacketStatusAndOpaqueBoundary(); TestRawInputAndProviderKeys(); TestAvarEffectiveSelection();
    std::printf("OpValues: %d failures\n",failures);
    return failures ? 1 : 0;
}
