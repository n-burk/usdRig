#include "rigExec/bakedOpValues.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/pathText.h"
#include "rigExecGraph/opValues.h"
#include "rigExecGraph/providerProgram.h"
#include "pxr/base/gf/quatd.h"
#include "pxr/base/gf/vec2d.h"
#include "pxr/base/gf/vec4f.h"
#include "pxr/base/vt/types.h"
#include "pxr/base/vt/value.h"
#include <array>
#include <climits>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <set>
#include <string>
#include <variant>
#include <vector>

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
// A point-carrying value's key over its points' bytes, which its value key
// replaces with a content version.
std::string ContentKey(const RigExecBakedProgramImpl &B, RigExecBakedSlotDomain domain,
    uint32_t slot=0)
{
    std::string key; CHECK(RigExecBakedChainContentKey(B,domain,slot,&key)); return key;
}
float FloatBits(uint32_t bits)
{
    float value; std::memcpy(&value,&bits,sizeof(value)); return value;
}
double DoubleBits(uint64_t bits)
{
    double value; std::memcpy(&value,&bits,sizeof(value)); return value;
}
void PutU64(std::string *out,uint64_t value)
{
    out->append(reinterpret_cast<const char *>(&value),sizeof(value));
}
// A string as the keys write one: its length, then its bytes.
std::string Prefixed(const std::string &text)
{
    std::string out; PutU64(&out,text.size()); return out+text;
}
// One provider leaf holding \p value, keyed as its SpaceLeaf.
std::string LeafKey(const VtValue &value,bool *exact=nullptr)
{
    RigExecBakedProgramImpl B;
    B.providerProgram.sampled.resize(1); B.providerLeaves.values={value};
    B.providerLeafBlocked={0};
    if(exact) *exact=RigExecBakedOpValueKeyIsExact(B,RigExecBakedSlotDomain::SpaceLeaf,0);
    return Key(B,RigExecBakedSlotDomain::SpaceLeaf);
}
bool EndsWith(const std::string &text,const std::string &suffix)
{
    return text.size()>=suffix.size() &&
        text.compare(text.size()-suffix.size(),suffix.size(),suffix)==0;
}
void TestPathTextSpelling()
{
    // A path below plain prims spells exactly as GetString(), with or
    // without the memo.
    for(const char *text:{"/","/A","/A/B","/Rig/Joint_1/ctl","/A/B.attr",
                          "/A/B.ns:attr","/A.rest:space"}) {
        const SdfPath path(text); std::string spelled;
        CHECK(!path.IsEmpty());
        CHECK(RigExecSpellPathText(path,&spelled) && spelled==path.GetString());
        RigExecPathText memo; CHECK(memo(path)==path.GetString());
    }
    // Any other path is refused with its text cleared: relative, under a
    // variant selection (GetPrimPath() would step over it and spell
    // '/A/B{v=x}.attr' as '/A/B.attr'), or a target, mapper or expression.
    for(const char *text:{"/A{v=x}","/A/B{v=x}.attr","/A{v=x}B","/A{v=x}B.attr",
                          "/A{v=x}{w=y}","A","A/B","../A",".","A.attr","/A.rel[/B]",
                          "/A.rel[/B].ra","/A.attr.mapper[/B]","/A.attr.expression"}) {
        const SdfPath path(text); std::string spelled="stale";
        CHECK(!path.IsEmpty());
        CHECK(!RigExecSpellPathText(path,&spelled) && spelled.empty());
        RigExecPathText memo; CHECK(memo(path)==path.GetString());
    }
    std::string spelled="stale";
    CHECK(!RigExecSpellPathText(SdfPath(),&spelled) && spelled.empty());
    // One memo across a variant selection and the paths around it.
    RigExecPathText memo; CHECK(memo(SdfPath()).empty());
    for(const char *text:{"/A","/A{v=x}","/A{v=x}B","/A{v=x}B.attr","/A/B",
                          "/A/B{v=x}","/A/B{v=x}.attr","/A/B.attr"})
        CHECK(memo(SdfPath(text))==SdfPath(text).GetString());
}
void TestBoxTags()
{
    // Equal payload bytes under different types stay apart.
    CHECK(LeafKey(VtValue(int(0)))!=LeafKey(VtValue(0.0f)));
    CHECK(LeafKey(VtValue(0.0f))!=LeafKey(VtValue(0.0)));
    CHECK(LeafKey(VtValue(int(0)))==LeafKey(VtValue(int(0))));
    CHECK(LeafKey(VtValue(VtFloatArray{0.0f}))!=LeafKey(VtValue(VtIntArray{0})));
    const std::string text="/Rig/A";
    const auto asString=LeafKey(VtValue(text)),asToken=LeafKey(VtValue(TfToken(text)));
    const auto asPath=LeafKey(VtValue(SdfPath(text)));
    CHECK(asString!=asToken && asToken!=asPath && asString!=asPath);
    CHECK(LeafKey(VtValue(VtStringArray{text}))!=LeafKey(VtValue(VtTokenArray{TfToken(text)})));
    // A spelled path keys by the text GetString() gives.
    CHECK(EndsWith(asPath,Prefixed(SdfPath(text).GetString())));
    CHECK(EndsWith(LeafKey(VtValue(SdfPath("/Rig/A.attr"))),Prefixed("/Rig/A.attr")));
    // Two types Box does not know stay apart, and stay non-exact.
    bool exact=true;
    const auto vec4=LeafKey(VtValue(GfVec4f(0.0f)),&exact); CHECK(!exact);
    exact=true;
    const auto vec2=LeafKey(VtValue(GfVec2d(0.0)),&exact); CHECK(!exact);
    CHECK(vec4!=vec2);
    // Every path value is exact, and distinct paths key apart, including a
    // property under a variant selection and the same property without it.
    const std::vector<SdfPath> paths={SdfPath("/A/B.attr"),SdfPath("/A/B{v=x}.attr"),
        SdfPath("/A/B{v=y}.attr"),SdfPath("/A{v=x}B.attr"),SdfPath("/A/B{v=x}"),
        SdfPath("/A/B"),SdfPath("A/B"),SdfPath("../A/B"),SdfPath("A/B.attr"),
        SdfPath("/A.rel[/B]"),SdfPath("/A.rel[/C]"),SdfPath("/A.rel[/B].ra"),
        SdfPath("/A.attr.mapper[/B]"),SdfPath("/A.attr.expression"),SdfPath("."),
        SdfPath()};
    std::set<std::string> keys;
    for(const SdfPath &path:paths) {
        exact=false; keys.insert(LeafKey(VtValue(path),&exact)); CHECK(exact);
    }
    CHECK(keys.size()==paths.size());
    // An equal path built another way keys alike.
    CHECK(LeafKey(VtValue(SdfPath("/A/B{v=x}.attr")))==
          LeafKey(VtValue(SdfPath("/A/B{v=x}").AppendProperty(TfToken("attr")))));
    CHECK(LeafKey(VtValue(SdfPath("/A.rel[/B]")))==
          LeafKey(VtValue(SdfPath("/A.rel").AppendTarget(SdfPath("/B")))));
}
void TestHeadValueBits()
{
    const auto same=[](const VtValue &a,const VtValue &b) { return RigExecBakedHeadValueSame(a,b); };
    const float fnan=FloatBits(UINT32_C(0x7fc00001)),fnan2=FloatBits(UINT32_C(0x7fc00002));
    const double dnan=DoubleBits(UINT64_C(0x7ff8000000000001)),dnan2=DoubleBits(UINT64_C(0x7ff8000000000002));
    // A signed zero is a change; the same NaN bits in another buffer are
    // not, and another payload is.
    CHECK(!same(VtValue(VtFloatArray{0.0f}),VtValue(VtFloatArray{-0.0f})));
    CHECK(same(VtValue(VtFloatArray{fnan}),VtValue(VtFloatArray{fnan})));
    CHECK(!same(VtValue(VtFloatArray{fnan}),VtValue(VtFloatArray{fnan2})));
    CHECK(!same(VtValue(VtFloatArray{1.0f}),VtValue(VtFloatArray{1.0f,1.0f})));
    const VtFloatArray shared{fnan}; CHECK(same(VtValue(shared),VtValue(shared)));
    CHECK(!same(VtValue(VtDoubleArray{0.0}),VtValue(VtDoubleArray{-0.0})));
    CHECK(same(VtValue(VtDoubleArray{dnan}),VtValue(VtDoubleArray{dnan})));
    CHECK(!same(VtValue(VtDoubleArray{dnan}),VtValue(VtDoubleArray{dnan2})));
    VtMatrix4dArray positive{GfMatrix4d(1.0)},negative{GfMatrix4d(1.0)};
    negative[0][3][0]=-0.0;
    CHECK(!same(VtValue(positive),VtValue(negative)));
    VtMatrix4dArray left{GfMatrix4d(1.0)},right{GfMatrix4d(1.0)};
    left[0][2][1]=dnan; right[0][2][1]=dnan;
    CHECK(same(VtValue(left),VtValue(right)));
    right[0][2][1]=dnan2; CHECK(!same(VtValue(left),VtValue(right)));
    // Floating payloads beyond the arrays above, alone and in arrays.
    CHECK(!same(VtValue(GfVec4f(0.0f)),VtValue(GfVec4f(-0.0f,0.0f,0.0f,0.0f))));
    CHECK(same(VtValue(GfQuatd(dnan)),VtValue(GfQuatd(dnan))));
    CHECK(!same(VtValue(VtVec2dArray{GfVec2d(0.0)}),VtValue(VtVec2dArray{GfVec2d(0.0,-0.0)})));
    // Integral and text values compare by value; types and emptiness first.
    CHECK(same(VtValue(VtTokenArray{TfToken("a")}),VtValue(VtTokenArray{TfToken("a")})));
    CHECK(!same(VtValue(VtFloatArray{0.0f}),VtValue(VtDoubleArray{0.0})));
    CHECK(same(VtValue(),VtValue()) && !same(VtValue(),VtValue(0.0f)));
}
void TestCandidateFallbackBytes()
{
    RigExecBakedProgramImpl B;
    B.paths={SdfPath("/Rig/Arm"),SdfPath("/Rig/Arm/Hand")};
    B.pathTexts=RigExecBakedSpellPathTexts(B.paths);
    B.solvers.resize(1); B.solvers[0].fallbackSlots={1,0};
    // The bytes the key wrote when the solver held the joints' paths: no
    // frames, no presence bytes, then a count and each path's GetString()
    // with its length.
    std::string expected(1,char(1));
    const auto domain=RigExecBakedSlotDomain::Candidates;
    expected.append(reinterpret_cast<const char *>(&domain),sizeof(domain));
    PutU64(&expected,0); PutU64(&expected,0); PutU64(&expected,2);
    expected+=Prefixed(B.paths[1].GetString())+Prefixed(B.paths[0].GetString());
    CHECK(Key(B,RigExecBakedSlotDomain::Candidates)==expected);
}
void TestProviderOwnerText()
{
    RigExecProviderProgram program; program.valueKeys={"out","order","translate"};
    RigExecProviderOp op; op.kind=RigExecProviderOpKind::LocalXform;
    op.owner=SdfPath("/Rig/Xf"); op.output=0; op.inputs={1};
    op.xforms.push_back({TfToken("xformOp:translate"),2,0});
    program.ops.push_back(op);
    RigExecTypedValueStore store(3); std::string error;
    // A body never spells an owner itself.
    CHECK(!RigExecRunProviderOp(program,0,&store,&error));
    RigExecSpellProviderOwners(&program);
    CHECK(program.ownerTexts==std::vector<std::string>{"/Rig/Xf"});
    error.clear();
    CHECK(RigExecRunProviderOp(program,0,&store,&error) &&
          error=="xform order unavailable: "+op.owner.GetString());
    // One text per owner; a copied op keeps its own.
    program.ops.push_back(program.ops[0]);
    op.owner=SdfPath("/Rig/Other"); program.ops.push_back(op); program.ops.push_back(op);
    RigExecSpellProviderOwners(&program);
    CHECK(program.ownerTexts.size()==2 && program.ops[1].ownerText==program.ops[0].ownerText &&
          program.ops[2].ownerText==program.ops[3].ownerText &&
          program.ownerTexts[program.ops[2].ownerText]=="/Rig/Other");
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
    RigExecBakedIndexAvarBindings(&B);
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

// An AvarInputs op keys, notes and reads exactly its provider's bindings
// (flat slot / 11) of each list, in list order, through the Build-time
// index: the bytes a scan of the whole list would write, and nothing of
// another provider's bindings.
void TestAvarBindingIndex()
{
    using Binding=RigExecBakedProgramImpl::AvarBinding;
    RigExecBakedProgramImpl B;
    B.avarConstants.assign(4*11,0.0); B.avars.assign(4*11,-1.0);
    B.leaves.Of<double>().value={1.5,2.5,3.5,4.5};
    const auto add=[](std::vector<Binding> *list,size_t slot,int leaf,double constant) {
        Binding binding; binding.slot=slot; binding.input.leaf=leaf;
        binding.input.constant=constant; list->push_back(binding);
    };
    // Provider 1 binds nothing; provider 2 only constants; 0 and 3 both.
    add(&B.avarBindings,0*11+1,0,10); add(&B.avarBindings,0*11+4,1,11);
    add(&B.avarBindings,3*11+0,2,12);
    add(&B.avarConstantBindings,0*11+7,-1,20); add(&B.avarConstantBindings,2*11+2,-1,21);
    add(&B.avarConstantBindings,2*11+10,3,22); add(&B.avarConstantBindings,3*11+5,-1,23);
    RigExecBakedIndexAvarBindings(&B);
    CHECK(B.avarBindingBegin==std::vector<uint32_t>({0,2,2,2,3}));
    CHECK(B.avarConstantBindingBegin==std::vector<uint32_t>({0,1,1,3,4}));
    for(int provider=-2;provider<7;++provider) {
        RigExecBakedStep step; step.kind=RigExecBakedStepKind::AvarInputs; step.object=provider;
        // The provider's entries by a scan, which the index must name.
        std::vector<size_t> varying,constant;
        for(size_t i=0;i<B.avarBindings.size();++i)
            if(provider>=0 && B.avarBindings[i].slot/11==size_t(provider)) varying.push_back(i);
        for(size_t i=0;i<B.avarConstantBindings.size();++i)
            if(provider>=0 && B.avarConstantBindings[i].slot/11==size_t(provider)) constant.push_back(i);
        const auto listed=[&](const std::vector<uint32_t> &begin) {
            std::vector<size_t> out; const auto range=RigExecBakedAvarBindingRange(begin,provider);
            for(size_t i=range.first;i<range.second;++i) out.push_back(i);
            return out;
        };
        CHECK(listed(B.avarBindingBegin)==varying);
        CHECK(listed(B.avarConstantBindingBegin)==constant);
        // The raw source key's avar block, written as the scan wrote it.
        std::string block; block.push_back(char(26));
        block.append(reinterpret_cast<const char *>(&provider),sizeof(provider));
        const auto put=[&](const std::vector<Binding> &list,const std::vector<size_t> &ids) {
            PutU64(&block,ids.size());
            for(size_t i:ids) {
                const auto &input=list[i].input; PutU64(&block,list[i].slot);
                block.append(reinterpret_cast<const char *>(&input.leaf),sizeof(input.leaf));
                block.append(reinterpret_cast<const char *>(&input.constant),sizeof(input.constant));
                if(input.leaf>=0) {
                    const double value=B.leaves.Of<double>().value[size_t(input.leaf)];
                    block.append(reinterpret_cast<const char *>(&value),sizeof(value));
                }
            }
        };
        put(B.avarBindings,varying); put(B.avarConstantBindings,constant);
        std::string raw,effective; std::vector<uint32_t> covered;
        CHECK(RigExecBakedOpInputKey(B,step,&raw) && raw.find(block)!=std::string::npos);
        // A program holding only this provider's bindings keys it the same.
        RigExecBakedProgramImpl alone; alone.leaves.Of<double>().value=B.leaves.Of<double>().value;
        for(size_t i:varying) alone.avarBindings.push_back(B.avarBindings[i]);
        for(size_t i:constant) alone.avarConstantBindings.push_back(B.avarConstantBindings[i]);
        RigExecBakedIndexAvarBindings(&alone);
        std::string aloneRaw,aloneEffective;
        CHECK(RigExecBakedOpInputKey(alone,step,&aloneRaw) && aloneRaw==raw);
        CHECK(RigExecBakedOpEffectiveInputKey(B,step,&effective,&covered));
        CHECK(RigExecBakedOpEffectiveInputKey(alone,step,&aloneEffective,&covered) &&
              aloneEffective==effective);
        // The body writes the provider's avars and no other.
        std::vector<double> expected=B.avars;
        for(size_t i:varying) expected[B.avarBindings[i].slot]=RigExecBakedLeafRead(B,B.avarBindings[i].input);
        for(size_t i:constant) expected[B.avarConstantBindings[i].slot]=RigExecBakedLeafRead(B,B.avarConstantBindings[i].input);
        RigExecBakedRunAvarOp(&B,&step);
        CHECK(B.avars==expected);
    }
    // Another provider's leaf moves no key of provider 0; its own does.
    RigExecBakedStep zero; zero.kind=RigExecBakedStepKind::AvarInputs; zero.object=0;
    std::string before,after;
    CHECK(RigExecBakedOpInputKey(B,zero,&before));
    B.leaves.Of<double>().value[2]=-0.0; B.leaves.Of<double>().value[3]=99;
    CHECK(RigExecBakedOpInputKey(B,zero,&after) && after==before);
    B.leaves.Of<double>().value[1]=-0.0;
    CHECK(RigExecBakedOpInputKey(B,zero,&after) && after!=before);
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
    B.avarBindings.push_back(binding); RigExecBakedIndexAvarBindings(&B);
    CHECK(RigExecBakedOpInputKey(B,step,&key)); const auto avar=key;
    B.leaves.Of<double>().value[0]=0.25;
    CHECK(RigExecBakedOpInputKey(B,step,&key) && key!=avar);
    B.chains.resize(1); B.chains[0].sampledHaveBase=true;
    B.chains[0].sampledBase=VtVec3fArray{GfVec3f(0)};
    const auto incoming=ContentKey(B,RigExecBakedSlotDomain::ChainInput);
    const auto incomingVersion=Key(B,RigExecBakedSlotDomain::ChainInput);
    B.chains[0].sampledBase[0][0]=-0.0f;
    CHECK(ContentKey(B,RigExecBakedSlotDomain::ChainInput)!=incoming);
    // The value key moves with the content version publication bumps, not
    // with the bytes themselves.
    CHECK(Key(B,RigExecBakedSlotDomain::ChainInput)==incomingVersion);
    ++B.chains[0].inputVersion;
    CHECK(Key(B,RigExecBakedSlotDomain::ChainInput)!=incomingVersion);
    B.chains[0].sampledBase[0][0]=0.0f; B.chains[0].sampledHaveBase=false;
    CHECK(ContentKey(B,RigExecBakedSlotDomain::ChainInput)!=incoming);
    CHECK(Key(B,RigExecBakedSlotDomain::ChainInput)!=incomingVersion);
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
    RigExecBakedResetPathLeafVersions(&r.leaves);
    // Writes as a sampler makes them, after a run read the keys.
    const auto set=[&](size_t k,VtValue value) {
        RigExecBakedSetPathLeaf(&r.leaves,k,std::move(value),++B.pathLeafRun);
    };
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
    for(size_t k=0;k<3;++k) set(k,VtValue());
    CHECK(key(true)==selected);CHECK(key(false)!=raw);
    // A stale packet topology cannot shadow current null-handle fallback.
    r.layoutHandle.reset();
    const auto unavailable=key(true);
    set(0,VtValue(VtIntArray{1})); set(1,VtValue(VtFloatArray{0.5f})); set(2,VtValue(2));
    CHECK(key(true)!=unavailable);
    // A new current handle shadows raw rows before the packet adopts it.
    r.topology.reset();r.topologyResolved=false;r.layoutHandle=topology;
    const auto newlySelected=key(true);set(2,VtValue(3));
    CHECK(key(true)==newlySelected);
    r.layoutHandle.reset();
    const auto unresolved=key(true);set(2,VtValue(4));
    CHECK(key(true)!=unresolved);
}

// A geometry path leaf keys as its content version in both tiers. Built at
// one run and compared with the previous run's, a version key changes
// exactly when the same key over the leaves' contents does, whatever writes
// land between the runs (a value that moves and comes back included). A
// leaf the effective tier resolves through a produced version, and a
// provider leaf, keep their values.
void TestPathLeafContentVersions()
{
    using Type=RigExecRevisionLeafType; using Time=RigExecRevisionLeafTime;
    using Flavour=RigExecRevisionLeafFlavour;
    RigExecBakedProgramImpl B;
    B.chains.resize(1); B.chains[0].revisions.resize(1); B.revisionIndex={{0,0}};
    auto &leaves=B.chains[0].revisions[0].leaves;
    leaves.decl.Add({SdfPath("/Mover.points"),Type::Vec3fArray,Time::AtTime,
        Flavour::OverlayThenRaw,VtValue(VtVec3fArray())});
    leaves.decl.Add({SdfPath("/Mover.weight"),Type::Float,Time::AtTime,Flavour::Resolved,VtValue(1.0f)});
    leaves.values={leaves.decl.keys[0].fallback,leaves.decl.keys[1].fallback};
    RigExecBakedResetPathLeafVersions(&leaves);
    B.pathLeafRefs={{RigExecBakedPathLeafOwner::Revision,0,0,0},{RigExecBakedPathLeafOwner::Revision,0,0,1}};
    RigExecBakedStep step; step.kind=RigExecBakedStepKind::RevisionStatic; step.object=0;
    step.bindingLeaves={0,1};
    struct Keys { std::string source,effective; bool sourceExact=false,effectiveExact=false; };
    const auto keys=[&](bool content) {
        Keys out; std::vector<uint32_t> covered;
        out.sourceExact=RigExecBakedOpInputKey(B,step,&out.source,nullptr,content);
        out.effectiveExact=RigExecBakedOpEffectiveInputKey(B,step,&out.effective,&covered,nullptr,nullptr,content);
        return out;
    };
    const auto write=[&](size_t k,const VtValue &value) {
        return RigExecBakedSetPathLeaf(&leaves,k,value,B.pathLeafRun);
    };
    // A run increments the counter before it builds its keys.
    const auto run=[&] { ++B.pathLeafRun; return std::make_pair(keys(false),keys(true)); };

    // The key carries the version, not the elements.
    const VtVec3fArray many(100000,GfVec3f(0.5f));
    write(0,VtValue(many));
    const auto big=run();
    CHECK(big.first.source.size()<256 && big.second.source.size()>many.size()*sizeof(GfVec3f));
    CHECK(big.first.effective.size()<256 && big.second.effective.size()>many.size()*sizeof(GfVec3f));
    // The same bytes in another buffer, and a move that comes back before
    // the next run, change nothing; the same move across a run changes it.
    write(0,VtValue(VtVec3fArray(many.begin(),many.end())));
    CHECK(run().first.source==big.first.source);
    VtVec3fArray moved=many; moved[7][1]=-0.5f;
    write(0,VtValue(moved)); write(0,VtValue(many));
    CHECK(run().first.source==big.first.source);
    write(0,VtValue(moved));
    const auto there=run();
    CHECK(there.first.source!=big.first.source && there.first.effective!=big.first.effective);
    write(0,VtValue(many));
    const auto back=run();
    CHECK(back.first.source!=there.first.source && back.first.effective!=there.first.effective);

    // Every decision agrees with the contents' over runs of random writes,
    // first with both leaves plain, then with the weight resolved through a
    // produced version that also moves.
    const float nanA=FloatBits(0x7fc00001u),nanB=FloatBits(0x7fc00002u);
    const std::vector<VtValue> points{
        VtValue(VtVec3fArray()),VtValue(VtVec3fArray{GfVec3f(0.0f)}),
        VtValue(VtVec3fArray{GfVec3f(-0.0f,0.0f,0.0f)}),VtValue(VtVec3fArray{GfVec3f(nanA)}),
        VtValue(VtVec3fArray{GfVec3f(nanB)}),VtValue(VtVec3fArray{GfVec3f(1,2,3)}),
        VtValue(VtVec3fArray{GfVec3f(1,2,3)}),VtValue(),VtValue(VtFloatArray{1.0f}),
        VtValue(GfVec4f(1.0f))};
    const std::vector<VtValue> weights{VtValue(1.0f),VtValue(0.5f),VtValue(0.0f),
        VtValue(-0.0f),VtValue(nanA),VtValue(nanB),VtValue(1.0)};
    uint32_t seed=12345;
    const auto next=[&](uint32_t n) { seed=seed*1664525u+1013904223u; return (seed>>8)%n; };
    size_t sourceChanges=0,effectiveChanges=0,compared=0;
    for(int pass=0;pass<2;++pass) {
        if(pass==1) {
            leaves.exactVersions={-1,0}; leaves.exactRecordIndices={-1,-1};
            leaves.exactValueTypes={-1,int(RigExecBakedPropertyChain::Arm::Float)};
            B.propertyVersionValid={1}; B.propertyValues.resize(1); B.propertyValues[0].f=0.25f;
        }
        auto last=run();
        for(int i=0;i<4000;++i) {
            for(uint32_t w=next(4);w>0;--w) {
                if(next(2)) write(0,points[next(uint32_t(points.size()))]);
                else write(1,weights[next(uint32_t(weights.size()))]);
            }
            if(pass==1 && next(4)==0) B.propertyValues[0].f=next(2)?0.25f:-0.0f;
            const auto now=run();
            const auto &[version,content]=now;
            CHECK(version.sourceExact==content.sourceExact);
            CHECK(version.effectiveExact==content.effectiveExact);
            if(version.sourceExact && last.first.sourceExact) {
                const bool changed=version.source!=last.first.source;
                CHECK(changed==(content.source!=last.second.source));
                sourceChanges+=changed; ++compared;
            }
            if(version.effectiveExact && last.first.effectiveExact) {
                const bool changed=version.effective!=last.first.effective;
                CHECK(changed==(content.effective!=last.second.effective));
                effectiveChanges+=changed;
            }
            last=now;
        }
    }
    CHECK(compared>1000 && sourceChanges>100 && sourceChanges+100<compared && effectiveChanges>100);
    // A resolved leaf's effective key follows its produced value, which no
    // write to the leaf moves.
    const auto before=run();
    B.propertyValues[0].f=B.propertyValues[0].f==0.25f?0.75f:0.25f;
    const auto after=run();
    CHECK(after.first.source==before.first.source && after.first.effective!=before.first.effective);

    // A provider leaf keys its value, written in place by its own sampler.
    B.providerLeaves.decl.Add({SdfPath("/Provider.attr"),Type::Double,Time::AtTime,Flavour::Raw,VtValue(0.0)});
    B.providerLeaves.values={VtValue(0.0)};
    B.pathLeafRefs.push_back({RigExecBakedPathLeafOwner::Provider,0,0,0});
    RigExecBakedStep refresh; refresh.kind=RigExecBakedStepKind::Solve; refresh.bindingLeaves={2};
    std::string first,second;
    CHECK(RigExecBakedOpInputKey(B,refresh,&first));
    B.providerLeaves.values[0]=VtValue(-0.0);
    CHECK(RigExecBakedOpInputKey(B,refresh,&second) && second!=first);
    // Unsized versions are a broken writer contract: never an equal key.
    leaves.versions.clear();
    CHECK(!RigExecBakedOpInputKey(B,step,&first));
}

// A provider leaf's key and exactness from RigExecBakedSpaceLeafKey are
// RigExecBakedOpValueKey's and RigExecBakedOpValueKeyIsExact's, and the
// overlay through the publication index is the one the Build maps find,
// whichever table stands on the leaf.
void TestSpaceLeafIndexOverlay()
{
    RigExecBakedProgramImpl B;
    const SdfPath a("/Rig/A.avars:tx"),b("/Rig/B.avars:tx"),c("/Rig/C.avars:tx");
    B.providerProgram.sampled.resize(3);
    B.providerProgram.sampled[0].attribute=a; B.providerProgram.sampled[1].attribute=b;
    B.providerProgram.sampled[2].attribute=c;
    B.providerLeaves.values={VtValue(1.0),VtValue(2.0f),VtValue(GfVec4f(0.0f))};
    B.providerLeafBlocked={0,1,0};
    B.headOverrideSlots={{b,0u}}; B.headOverrides.resize(1);
    B.overridableInputs={{a,{0,1}},{c,{2}}}; B.overridden.assign(3,0);
    RigExecResolvedInputs resolved; resolved.SetProperty(a,VtValue(7.0));
    B.resolvedInputs=&resolved;
    auto index=std::make_shared<RigExecBakedSpaceLeafIndex>();
    index->headSlot={-1,0,-1}; index->numberBegin={0,2,2,3}; index->numbers={0,1,2};
    using D=RigExecBakedSlotDomain;
    const auto check=[&](const char *what) {
        for(uint32_t k=0;k<3;++k) {
            B.spaceLeafIndex.reset();
            const VtValue *found=RigExecBakedSpaceLeafOverlay(B,k);
            const bool exact=RigExecBakedOpValueKeyIsExact(B,D::SpaceLeaf,k);
            const std::string key=Key(B,D::SpaceLeaf,k);
            B.spaceLeafIndex=index;
            std::string fast;
            const bool same=RigExecBakedSpaceLeafOverlay(B,k)==found &&
                RigExecBakedSpaceLeafKey(B,k,&fast)==exact && fast==key;
            if(!same) std::printf("  %s, leaf %u\n",what,k);
            CHECK(same);
        }
    };
    check("no overlay");
    B.overridden[1]=1; check("override number");
    B.overridden[2]=1; check("override number without a resolved value");
    B.headOverrides[0]=VtValue(3.0f); check("head override");
    B.upstream[b]=VtValue(4.0); check("upstream");
    B.routedOverrides[c]=VtValue(5.0); check("routed");
    B.resolvedInputs=nullptr; check("no resolved inputs");
}

// The executor never rebuilds a source key the classifier calls constant,
// so that key must hold its bytes whatever sampled, overridden, published or
// provider state holds, and every gate the classifier tests must key state.
void TestConstantSourceKeys()
{
    RigExecBakedProgramImpl B;
    B.leaves.Of<double>().value={0.0}; B.leafRefs.push_back({RigExecBakedLeafType::Double,0});
    B.headLeaves.resize(1); B.headOverrides.resize(1);
    B.readerWalks.resize(1); B.readerWalks[0].leaves={0}; B.readerWalks[0].slots={0};
    B.walkSteps.resize(1); B.walkSteps[0].index=0;
    B.constraints.resize(1); B.constraints[0].sourceNatives={0};
    B.nativeFrames.resize(1); B.nativeFrameOk={1};
    B.chains.resize(1); B.chains[0].revisions.resize(2); B.revisionIndex={{0,0},{0,1}};
    B.chains[0].revisions[1].weightObject=0;
    B.providerValues.values.resize(1); B.providerValues.values[0].initialized=true;
    const auto set=[&](int k) {
        B.leaves.Of<double>().value[0]=double(k);
        B.headLeaves[0].value=VtValue(float(k)); B.headOverrides[0]=VtValue(float(k));
        B.nativeFrames[0].points[0][0]=float(k);
        B.publishWeightFields=k!=0;
        B.providerValues.values[0].value=double(k);
    };
    using K=RigExecBakedStepKind;
    const auto make=[](K kind,int object,int part,int list) {
        RigExecBakedStep step; step.kind=kind; step.object=object; step.part=part;
        if(list==0) step.bindingLeaves={0}; else if(list==1) step.leaves={0};
        else if(list==2) step.overrideSlots={0}; else if(list==3) step.readerWalks={0};
        return step;
    };
    std::vector<std::pair<RigExecBakedStep,bool>> cases{
        {make(K::SpaceExpression,0,0,-1),true}, {make(K::Solve,0,-1,-1),true},
        {make(K::RevisionStatic,0,-1,-1),true},
        // The weight overlay toggle, a constraint's native frames, each list.
        {make(K::RevisionStatic,1,-1,-1),false}, {make(K::Constraint,0,-1,-1),false}};
    for(int list=0;list<4;++list) cases.push_back({make(K::SpaceExpression,0,0,list),false});
    std::string fixed; for(int i=0;i<4;++i) PutU64(&fixed,0);
    for(const auto &[step,constant]:cases) {
        CHECK(RigExecBakedOpInputKeyIsConstant(B,step)==constant);
        std::string before,after;
        set(0); const bool exact=RigExecBakedOpInputKey(B,step,&before);
        set(1); CHECK(RigExecBakedOpInputKey(B,step,&after)==exact);
        CHECK((before==after)==constant);
        if(constant) CHECK(exact && before==fixed);
    }
    // AvarInputs keys its bindings whatever its lists hold.
    CHECK(!RigExecBakedOpInputKeyIsConstant(B,make(K::AvarInputs,0,-1,-1)));
}

// The per-element encoding keys used before contiguous runs: each field's
// bytes in memory order. A run must reproduce it byte for byte.
template<class T> void RefPut(std::string *out,const T &value)
{
    static_assert(std::is_arithmetic<T>::value,"fields only");
    out->append(reinterpret_cast<const char *>(&value),sizeof(value));
}
void RefPut(std::string *out,const GfVec2f &v) { for(int i=0;i<2;++i) RefPut(out,v[i]); }
void RefPut(std::string *out,const GfVec3f &v) { for(int i=0;i<3;++i) RefPut(out,v[i]); }
void RefPut(std::string *out,const GfVec3d &v) { for(int i=0;i<3;++i) RefPut(out,v[i]); }
void RefPut(std::string *out,const GfVec3i &v) { for(int i=0;i<3;++i) RefPut(out,v[i]); }
void RefPut(std::string *out,const GfMatrix4d &v)
{
    for(int r=0;r<4;++r) for(int c=0;c<4;++c) RefPut(out,v[r][c]);
}
template<class A> std::string RefRun(const A &values)
{
    std::string out; for(const auto &value:values) RefPut(&out,value); return out;
}
std::string U64(uint64_t value) { std::string out; PutU64(&out,value); return out; }
template<class A> std::string RefArray(const A &values) { return U64(values.size())+RefRun(values); }
// The shared key encoder's elements before runs: each one's object bytes.
template<class V> std::string RefObjectArray(const V &values)
{
    std::string out=U64(values.size());
    for(const auto &value:values) {
        const typename V::value_type element=value;
        out.append(reinterpret_cast<const char *>(&element),sizeof(element));
    }
    return out;
}
std::string ValueHeader(RigExecBakedSlotDomain domain)
{
    std::string out(1,char(1));
    out.append(reinterpret_cast<const char *>(&domain),sizeof(domain));
    return out;
}
const float kNaN=FloatBits(UINT32_C(0x7fc00001)),kNaN2=FloatBits(UINT32_C(0xffc00002));
const double kDNaN=DoubleBits(UINT64_C(0x7ff8000000000001)),kDNaN2=DoubleBits(UINT64_C(0xfff8000000000002));
GfMatrix4d SignedMatrix()
{
    GfMatrix4d m(1.0); m[3][0]=-0.0; m[2][1]=kDNaN; m[0][3]=kDNaN2; m[1][2]=-7.5;
    return m;
}
// A boxed array keys as its tag, its count, then its elements' bytes.
template<class T> void CheckBoxRun(const VtArray<T> &values)
{
    const auto key=LeafKey(VtValue(values)),empty=LeafKey(VtValue(VtArray<T>()));
    CHECK(EndsWith(empty,U64(0)));
    CHECK(key==empty.substr(0,empty.size()-8)+RefArray(values));
}
void TestBulkValueKeyBytes()
{
    // Every element type that keys as one run, with signed zeros, NaN
    // payloads of both signs and empty arrays.
    CheckBoxRun(VtDoubleArray{0.0,-0.0,kDNaN,kDNaN2,-2.25});
    CheckBoxRun(VtFloatArray{0.0f,-0.0f,kNaN,kNaN2,1.5f});
    CheckBoxRun(VtIntArray{0,-1,INT_MAX,INT_MIN});
    CheckBoxRun(VtBoolArray{true,false,true});
    CheckBoxRun(VtVec2fArray{GfVec2f(-0.0f,kNaN),GfVec2f(1.0f,kNaN2)});
    CheckBoxRun(VtVec3fArray{GfVec3f(0.0f,-0.0f,kNaN),GfVec3f(kNaN2,1.0f,-1.0f)});
    CheckBoxRun(VtVec3dArray{GfVec3d(-0.0,kDNaN,1.0),GfVec3d(kDNaN2)});
    CheckBoxRun(VtVec3iArray{GfVec3i(1,-2,INT_MIN),GfVec3i(0)});
    CheckBoxRun(VtMatrix4dArray{SignedMatrix(),GfMatrix4d(2.0)});
    CheckBoxRun(VtDoubleArray()); CheckBoxRun(VtVec3fArray()); CheckBoxRun(VtMatrix4dArray());
    // Text arrays keep one length-prefixed record per element.
    const VtTokenArray tokens{TfToken("a"),TfToken(""),TfToken("bc")};
    const auto tokenKey=LeafKey(VtValue(tokens)),noTokens=LeafKey(VtValue(VtTokenArray()));
    CHECK(tokenKey==noTokens.substr(0,noTokens.size()-8)+U64(3)+Prefixed("a")+Prefixed("")+Prefixed("bc"));
    const VtStringArray strings{"a","","bc"};
    const auto stringKey=LeafKey(VtValue(strings)),noStrings=LeafKey(VtValue(VtStringArray()));
    CHECK(stringKey==noStrings.substr(0,noStrings.size()-8)+U64(3)+Prefixed("a")+Prefixed("")+Prefixed("bc"));

    // Arrays inside value keys: VtArray and std::vector points, scalar
    // vectors, matrices and presence bytes.
    RigExecBakedProgramImpl B;
    B.chains.resize(1); B.chains[0].sampledHaveBase=true;
    const auto noBase=ContentKey(B,RigExecBakedSlotDomain::ChainInput);
    B.chains[0].sampledBase=VtVec3fArray{GfVec3f(-0.0f,kNaN,1.0f),GfVec3f(kNaN2,0.0f,2.0f)};
    CHECK(ContentKey(B,RigExecBakedSlotDomain::ChainInput)==
          noBase.substr(0,noBase.size()-8)+RefArray(B.chains[0].sampledBase));
    B.chains[0].inputVersion=7;
    std::string input=ValueHeader(RigExecBakedSlotDomain::ChainInput);
    RefPut(&input,true); input+=U64(7);
    CHECK(Key(B,RigExecBakedSlotDomain::ChainInput)==input);
    B.weightFields.resize(1); B.weightFields[0].ok=true;
    const auto noValues=Key(B,RigExecBakedSlotDomain::WeightField);
    B.weightFields[0].values={0.0f,-0.0f,kNaN,kNaN2};
    CHECK(Key(B,RigExecBakedSlotDomain::WeightField)==
          noValues.substr(0,noValues.size()-8)+RefArray(B.weightFields[0].values));
    B.solvers.resize(1); B.solvers[0].ribbonPointsVarying=true;
    const auto noRibbon=Key(B,RigExecBakedSlotDomain::SolverPoints);
    B.solvers[0].ribbonPoints={GfVec3f(kNaN,-0.0f,3.0f),GfVec3f(0.5f)};
    CHECK(Key(B,RigExecBakedSlotDomain::SolverPoints)==
          noRibbon.substr(0,noRibbon.size()-8)+RefArray(B.solvers[0].ribbonPoints));
    B.commits.resize(1);
    const auto noDeltas=Key(B,RigExecBakedSlotDomain::CommitDelta);
    CHECK(EndsWith(noDeltas,U64(0)+U64(0)));
    B.commits[0].deltas={SignedMatrix(),GfMatrix4d(1.0)}; B.commits[0].deltaOk={1,0,char(0xff)};
    CHECK(Key(B,RigExecBakedSlotDomain::CommitDelta)==noDeltas.substr(0,noDeltas.size()-16)+
          RefArray(B.commits[0].deltas)+RefArray(B.commits[0].deltaOk));

    // A revision's completed points, the same bytes as its chain dirty
    // edge, and a chunk's half-open range of staged points.
    RigExecBakedProgramImpl G;
    G.chains.resize(1); G.chains[0].revisions.resize(1); G.revisionIndex={{0,0}};
    auto &revision=G.chains[0].revisions[0]; revision.currentSource=0;
    const auto noOutput=ContentKey(G,RigExecBakedSlotDomain::RevisionDone);
    revision.output={GfVec3f(-0.0f,kNaN,1.0f),GfVec3f(kNaN2,0.0f,-3.0f),GfVec3f(4.0f)};
    const auto done=ContentKey(G,RigExecBakedSlotDomain::RevisionDone);
    CHECK(done==noOutput.substr(0,noOutput.size()-8)+RefArray(revision.output));
    CHECK(ContentKey(G,RigExecBakedSlotDomain::ChainDirty)==done);
    // The value keys: the same fields with the content version in place of
    // the points, and the dirty edge the same bytes as the completion.
    revision.doneVersion=3;
    const auto versioned=Key(G,RigExecBakedSlotDomain::RevisionDone);
    CHECK(versioned==noOutput.substr(0,noOutput.size()-8)+U64(3));
    CHECK(Key(G,RigExecBakedSlotDomain::ChainDirty)==versioned);
    G.revisionChunkBase={0}; G.revisionChunkCount={1};
    revision.stagingOutput=revision.output; revision.chunks.resize(1);
    revision.chunks[0].begin=1; revision.chunks[0].end=3; revision.chunks[0].ok=true;
    std::string staged=ValueHeader(RigExecBakedSlotDomain::RevisionOut);
    RefPut(&staged,true); staged+=U64(3)+U64(2);
    staged+=RefRun(std::vector<GfVec3f>(revision.stagingOutput.begin()+1,revision.stagingOutput.end()));
    CHECK(Key(G,RigExecBakedSlotDomain::RevisionOut)==staged);
    revision.chunks[0].begin=3;
    std::string none=ValueHeader(RigExecBakedSlotDomain::RevisionOut);
    RefPut(&none,true); none+=U64(3)+U64(0);
    CHECK(Key(G,RigExecBakedSlotDomain::RevisionOut)==none);

    // A rest's four points after its matrix and frame.
    RigExecBakedProgramImpl R;
    const std::array<GfVec3d,4> points{{GfVec3d(-0.0,kDNaN,1.0),GfVec3d(2.0),GfVec3d(kDNaN2),GfVec3d(0.0)}};
    RigExecPointFrame frame; frame.points[1]=GfVec3d(-0.0,kDNaN,3.0);
    R.restM={SignedMatrix()}; R.restFrames={frame}; R.restPts={points};
    std::string rest=ValueHeader(RigExecBakedSlotDomain::Rest);
    RefPut(&rest,R.restM[0]); RefPut(&rest,frame.flags); rest+=RefRun(frame.points)+RefRun(points);
    CHECK(Key(R,RigExecBakedSlotDomain::Rest)==rest);

    // A weight field's raw point leaf inside its effective input key.
    RigExecBakedProgramImpl W;
    W.weightFields.resize(1); auto &field=W.weightFields[0];
    field.form=RigExecBakedProgramImpl::WeightField::Form::EnvelopeProperty; field.objects={0};
    W.weightProgram.resize(1); W.weightProgram[0].kind=3; W.weightObjects.resize(1);
    RigExecBakedStep step; step.kind=RigExecBakedStepKind::WeightField; step.object=0;
    const auto fieldKey=[&](const VtVec3fArray &leaf) {
        W.weightObjects[0].oracleLeaves.values={VtValue(leaf)};
        std::string key; std::vector<uint32_t> covered;
        CHECK(RigExecBakedOpEffectiveInputKey(W,step,&key,&covered));
        return key;
    };
    const VtVec3fArray leaf{GfVec3f(-0.0f,kNaN,1.0f),GfVec3f(kNaN2),GfVec3f(0.25f)};
    const auto emptyLeaf=fieldKey(VtVec3fArray()),fullLeaf=fieldKey(leaf);
    const std::string run=U64(leaf.size())+RefRun(leaf);
    const size_t at=fullLeaf.find(run);
    CHECK(at!=std::string::npos &&
          fullLeaf.substr(0,at)+U64(0)+fullLeaf.substr(at+run.size())==emptyLeaf);
}
std::string PlainKey(const RigExecProviderPlainValue &value)
{
    std::string out; RigExecOpKeyPlainValue(&out,value); return out;
}
// A provider vector keys by its contents: the same contents in another
// buffer agree, and an edit in place, which keeps the container's
// pointer, size and capacity, does not.
template<class V> void CheckPlainContents(const V &values,const typename V::value_type &other,
    const std::string &expected)
{
    RigExecProviderPlainValue held=values; const auto before=PlainKey(held);
    CHECK(before==expected);
    V spare; spare.reserve(values.size()*4+8); spare.assign(values.begin(),values.end());
    CHECK(PlainKey(RigExecProviderPlainValue(spare))==before);
    std::get<V>(held)[0]=other;
    CHECK(PlainKey(held)!=before);
}
void TestSharedKeyRunsAndPlainValues()
{
    // The shared encoder's runs: the same bytes as each element's object
    // bytes, signed zeros, NaN payloads and empty vectors included.
    const auto same=[](const auto &values) {
        std::string key; RigExecOpKeyArray(&key,values); CHECK(key==RefObjectArray(values));
    };
    same(std::vector<float>{0.0f,-0.0f,kNaN,kNaN2}); same(std::vector<float>());
    same(std::vector<double>{-0.0,kDNaN,kDNaN2}); same(std::vector<int32_t>{INT_MIN,0,7});
    same(std::vector<char>{0,1,char(0xff)}); same(std::vector<uint8_t>{2,0});
    same(std::vector<std::array<float,2>>{{{-0.0f,kNaN}}});
    same(std::vector<std::array<float,3>>{{{kNaN2,-0.0f,1.0f}},{{0.0f,0.0f,0.0f}}});
    same(std::vector<std::array<double,3>>{{{-0.0,kDNaN,0.5}}});
    std::array<double,16> matrix{}; matrix[3]=-0.0; matrix[9]=kDNaN2;
    same(std::vector<std::array<double,16>>{matrix}); same(std::vector<std::array<double,16>>());
    same(std::vector<bool>{true,false,true});
    static_assert(RigExecOpKeyBulkElement<std::array<double,16>>::value &&
                  !RigExecOpKeyBulkElement<std::string>::value &&
                  !RigExecOpKeyBulkElement<std::array<std::string,2>>::value &&
                  RigExecOpKeyContiguous<std::vector<float>>::value &&
                  RigExecOpKeyContiguous<VtVec3fArray>::value &&
                  !RigExecOpKeyContiguous<std::vector<bool>>::value,"key run eligibility");

    // Every vector alternative of a provider value.
    CheckPlainContents(std::vector<float>{-0.0f,kNaN,2.0f},0.0f,
                       RefObjectArray(std::vector<float>{-0.0f,kNaN,2.0f}));
    CheckPlainContents(std::vector<double>{-0.0,kDNaN},0.0,
                       RefObjectArray(std::vector<double>{-0.0,kDNaN}));
    CheckPlainContents(std::vector<int32_t>{3,-4},5,RefObjectArray(std::vector<int32_t>{3,-4}));
    const std::vector<std::array<float,3>> vec3f{{{-0.0f,kNaN2,1.0f}}};
    CheckPlainContents(vec3f,std::array<float,3>{{0.0f,0.0f,1.0f}},RefObjectArray(vec3f));
    const std::vector<std::array<double,3>> vec3d{{{1.0,-0.0,kDNaN}},{{2.0,2.0,2.0}}};
    CheckPlainContents(vec3d,std::array<double,3>{{1.0,0.0,0.0}},RefObjectArray(vec3d));
    const std::vector<std::array<float,2>> vec2f{{{kNaN,-0.0f}}};
    CheckPlainContents(vec2f,std::array<float,2>{{kNaN,0.0f}},RefObjectArray(vec2f));
    const std::vector<std::array<double,16>> matrices{matrix};
    std::array<double,16> moved=matrix; moved[9]=kDNaN;
    CheckPlainContents(matrices,moved,RefObjectArray(matrices));
    CheckPlainContents(std::vector<std::string>{"xformOp:translate",""},std::string("xformOp:rotateXYZ"),
                       U64(2)+Prefixed("xformOp:translate")+Prefixed(""));
    CheckPlainContents(std::vector<bool>{true,false},false,U64(2)+std::string("\1\0",2));
    // Scalars, fixed arrays and frames keep their object bytes; an empty
    // value adds nothing beyond its variant index, which the caller writes.
    std::string scalar; RigExecOpKeyAppend(&scalar,-0.0);
    CHECK(PlainKey(-0.0)==scalar && PlainKey(std::monostate()).empty());
    CHECK(PlainKey(std::string("ab"))==Prefixed("ab"));
    RigExecProviderPlainFrame plain; plain.points[2][1]=-0.0; plain.flags=5;
    std::string frameBytes; RigExecOpKeyAppend(&frameBytes,plain.points); RigExecOpKeyAppend(&frameBytes,plain.flags);
    CHECK(PlainKey(plain)==frameBytes);
}
// Point-carrying values key their points by a content version their writer
// bumps on a byte difference: signed zeros differ, a NaN equals only its
// own payload. The value key moves with the version and never with the
// bytes alone; the content key moves with the bytes.
void TestPointContentVersions()
{
    const std::vector<GfVec3f> a{GfVec3f(0.0f,kNaN,1.0f)};
    std::vector<GfVec3f> b=a;
    CHECK(RigExecBakedSamePoints(a.data(),a.size(),b.data(),b.size()));
    b[0][0]=-0.0f;
    CHECK(!RigExecBakedSamePoints(a.data(),a.size(),b.data(),b.size()));
    b=a; b[0][1]=kNaN2;
    CHECK(!RigExecBakedSamePoints(a.data(),a.size(),b.data(),b.size()));
    CHECK(!RigExecBakedSamePoints(a.data(),a.size(),a.data(),0));
    CHECK(RigExecBakedSamePoints(nullptr,0,a.data(),0));

    using D=RigExecBakedSlotDomain;
    RigExecBakedProgramImpl B;
    B.chains.resize(1); auto &chain=B.chains[0];
    chain.haveBase=chain.haveResult=true;
    chain.lastBase=VtVec3fArray{GfVec3f(1.0f)}; chain.result=VtVec3fArray{GfVec3f(2.0f)};
    chain.derived.resize(1); B.derivedIndex={{0,0}};
    auto &derived=chain.derived[0];
    derived.haveResult=true; derived.result=VtVec3fArray{GfVec3f(3.0f)};
    const auto bytesMove=[&](D domain,VtVec3fArray *points,uint64_t *version) {
        const auto key=Key(B,domain),content=ContentKey(B,domain);
        (*points)[0][0]=-(*points)[0][0];
        CHECK(Key(B,domain)==key);
        CHECK(ContentKey(B,domain)!=content);
        ++*version;
        CHECK(Key(B,domain)!=key);
    };
    bytesMove(D::ChainBase,&chain.lastBase,&chain.baseVersion);
    bytesMove(D::ChainPoints,&chain.result,&chain.resultVersion);
    bytesMove(D::DerivedOut,&derived.result,&derived.resultVersion);
    // A matrix target keys its matrix, not its points.
    derived.matrixTarget=true;
    const auto matrix=Key(B,D::DerivedOut);
    ++derived.resultVersion;
    CHECK(Key(B,D::DerivedOut)==matrix);
    derived.matrix[3][0]=1.0;
    CHECK(Key(B,D::DerivedOut)!=matrix);
}
// Publishes one value in place and, beside it, through its key: both must
// report \p changed and leave the same revision, and the stored key must be
// the key the value has now, which the value calls exact.
bool PublishBothWays(const RigExecBakedProgramImpl &B,RigExecOpValueState *typed,
    RigExecOpValueState *keyed,bool changed)
{
    const auto domain=RigExecBakedSlotDomain(typed->domain);
    const bool small=RigExecBakedPublishSmallValue(B,typed);
    RigExecOpPublishValue(keyed,[&](uint32_t d,uint32_t slot,std::string *key) {
        RigExecBakedOpValueKey(B,RigExecBakedSlotDomain(d),slot,key);
    });
    return small && typed->initialized && typed->changed==char(changed) &&
        keyed->changed==char(changed) && typed->revision==keyed->revision &&
        typed->key==keyed->key && typed->key==Key(B,domain,typed->slot) &&
        RigExecBakedOpValueKeyIsExact(B,domain,typed->slot);
}
// The floating payloads a sequence writes into one field, by case: a
// value, +0, -0 and two NaN payloads.
const double kSmallDoubles[]={1.5,0.0,-0.0,kDNaN,kDNaN2};
const float kSmallFloats[]={1.5f,0.0f,-0.0f,kNaN,kNaN2};
// \p set writes case k into one floating field of value (\p domain,
// \p slot); \p other then moves another field. Each publication agrees with
// the key's, and moves exactly when the bytes do: -0 differs from +0 and a
// NaN equals only its own payload.
template<class Set,class Other> void CheckSmallValue(RigExecBakedSlotDomain domain,
    uint32_t slot,const RigExecBakedProgramImpl &B,Set set,Other other,const char *what)
{
    RigExecOpValueState typed,keyed;
    typed.domain=keyed.domain=uint32_t(domain); typed.slot=keyed.slot=slot;
    const std::pair<int,bool> steps[]={{0,true},{0,false},{1,true},{2,true},{2,false},
        {3,true},{3,false},{4,true},{0,true}};
    int step=0;
    for(const auto &entry:steps) {
        set(entry.first);
        if(!PublishBothWays(B,&typed,&keyed,entry.second)) {
            ++failures; std::printf("FAIL small value %s, step %d\n",what,step);
        }
        ++step;
    }
    other();
    if(!PublishBothWays(B,&typed,&keyed,true) || !PublishBothWays(B,&typed,&keyed,false)) {
        ++failures; std::printf("FAIL small value %s, other field\n",what);
    }
    // Six moves in the sequence and the other field's.
    if(typed.revision!=7) {
        ++failures; std::printf("FAIL small value %s: revision %llu\n",what,
                                (unsigned long long)typed.revision);
    }
}
// A value the in-place publication refuses is left as it was.
bool RefusesSmall(const RigExecBakedProgramImpl &B,RigExecBakedSlotDomain domain,uint32_t slot)
{
    RigExecOpValueState value; value.domain=uint32_t(domain); value.slot=slot;
    value.key="kept"; value.revision=5; value.initialized=true; value.changed=0;
    return !RigExecBakedPublishSmallValue(B,&value) && value.key=="kept" &&
        value.revision==5 && value.initialized && value.changed==0;
}
void TestSmallValuePublication()
{
    using D=RigExecBakedSlotDomain;
    RigExecBakedProgramImpl B;
    B.avars.assign(22,0.25);
    CheckSmallValue(D::Avars,1,B,[&](int k) { B.avars[15]=kSmallDoubles[k]; },
                    [&] { B.avars[11]=2.0; },"Avars");
    B.base.resize(2); B.fin.resize(2);
    CheckSmallValue(D::PoseBase,1,B,[&](int k) { B.base[1].points[2][1]=kSmallDoubles[k]; },
                    [&] { B.base[1].flags^=0x10u; },"PoseBase");
    CheckSmallValue(D::PoseFin,1,B,[&](int k) { B.fin[1].points[0][0]=kSmallDoubles[k]; },
                    [&] { B.fin[1].flags^=0x10u; },"PoseFin");
    const auto matrices=[&](D domain,std::vector<GfMatrix4d> *values,const char *what) {
        values->assign(2,GfMatrix4d(1.0));
        CheckSmallValue(domain,1,B,[&](int k) { (*values)[1][3][1]=kSmallDoubles[k]; },
                        [&] { (*values)[1][0][0]=2.0; },what);
    };
    matrices(D::PosedM,&B.posedM,"PosedM");
    matrices(D::FinalMatrix,&B.finalMatrix,"FinalMatrix");
    matrices(D::BaseMatrix,&B.baseMatrix,"BaseMatrix");
    matrices(D::SwitchFrame,&B.switchFrames,"SwitchFrame");
    matrices(D::WeightFrames,&B.volumePlacement,"WeightFrames");
    matrices(D::WeightFramesBase,&B.volumePlacementBase,"WeightFramesBase");
    B.poseWeights.assign(2,0.5f);
    CheckSmallValue(D::PoseWeight,1,B,[&](int k) { B.poseWeights[1]=kSmallFloats[k]; },
                    [&] { B.poseWeights[1]=3.0f; },"PoseWeight");
    B.deltaValues.assign(2,GfMatrix4d(1.0)); B.deltaPresent.assign(2,1);
    CheckSmallValue(D::ConstraintDelta,1,B,[&](int k) { B.deltaValues[1][2][0]=kSmallDoubles[k]; },
                    [&] { B.deltaPresent[1]=0; },"ConstraintDelta");
    B.frameMatrix.assign(2,GfMatrix4d(1.0)); B.frameMatrixValid.assign(2,1);
    CheckSmallValue(D::FrameMatrix,1,B,[&](int k) { B.frameMatrix[1][0][3]=kSmallDoubles[k]; },
                    [&] { B.frameMatrixValid[1]=0; },"FrameMatrix");
    // Staging slot 4 is the second pair of the split commit based at 3; the
    // unsplit commit before it, whose tables are as long, owns no slot.
    B.commits.resize(2);
    B.commits[0].staged.resize(8); B.commits[0].outcome.assign(8,0);
    B.commits[1].split=true; B.commits[1].stagingBase=3;
    B.commits[1].staged.resize(2); B.commits[1].outcome.assign(2,0);
    CheckSmallValue(D::CommitStaging,4,B,
                    [&](int k) { B.commits[1].staged[1].points[3][2]=kSmallDoubles[k]; },
                    [&] { B.commits[1].outcome[1]=2; },"CommitStaging");

    // A space value per unboxed alternative, then its flags.
    B.providerValues.values.resize(2);
    auto &space=B.providerValues.values[1];
    space.initialized=true; space.count=1;
    CheckSmallValue(D::SpaceValue,1,B,[&](int k) {
        space.value.emplace<GfMatrix4d>(1.0); std::get<GfMatrix4d>(space.value)[1][3]=kSmallDoubles[k];
    },[&] { space.blocked=!space.blocked; },"SpaceValue matrix");
    CheckSmallValue(D::SpaceValue,1,B,[&](int k) { space.value.emplace<double>(kSmallDoubles[k]); },
                    [&] { space.authoritative=!space.authoritative; },"SpaceValue double");
    CheckSmallValue(D::SpaceValue,1,B,[&](int k) { space.value.emplace<float>(kSmallFloats[k]); },
                    [&] { space.count=3; },"SpaceValue float");
    CheckSmallValue(D::SpaceValue,1,B,
                    [&](int k) { space.value.emplace<GfVec3d>(1.0,kSmallDoubles[k],-2.0); },
                    [&] { space.initialized=!space.initialized; },"SpaceValue vector");
    CheckSmallValue(D::SpaceValue,1,B,[&](int k) {
        RigExecPointFrame frame; frame.points[3][0]=kSmallDoubles[k];
        space.value.emplace<RigExecPointFrame>(frame);
    },[&] { std::get<RigExecPointFrame>(space.value).flags^=0x10u; },"SpaceValue frame");
    {
        // An empty value keys its flags and index alone.
        space.value.emplace<std::monostate>();
        RigExecOpValueState typed,keyed;
        typed.domain=keyed.domain=uint32_t(D::SpaceValue); typed.slot=keyed.slot=1;
        CHECK(PublishBothWays(B,&typed,&keyed,true));
        CHECK(PublishBothWays(B,&typed,&keyed,false));
        space.count=5;
        CHECK(PublishBothWays(B,&typed,&keyed,true) && typed.revision==2);
    }

    // What keys by length or box, an unconverted domain, and every slot
    // the key refuses, go through the key.
    space.value.emplace<TfToken>(TfToken("a"));
    CHECK(RefusesSmall(B,D::SpaceValue,1));
    space.value.emplace<VtValue>(VtValue(1.0));
    CHECK(RefusesSmall(B,D::SpaceValue,1));
    space.value.emplace<GfMatrix4d>(1.0); space.error="unavailable";
    CHECK(RefusesSmall(B,D::SpaceValue,1));
    CHECK(RefusesSmall(B,D::SpaceValue,2));
    B.weightFields.resize(1); B.weightFields[0].ok=true;
    CHECK(RefusesSmall(B,D::WeightField,0));
    CHECK(RefusesSmall(B,D::RevisionDone,0));
    CHECK(RefusesSmall(B,D::Avars,2));
    CHECK(RefusesSmall(B,D::PoseFin,2) && RefusesSmall(B,D::PosedM,2) &&
          RefusesSmall(B,D::SwitchFrame,2) && RefusesSmall(B,D::PoseWeight,2) &&
          RefusesSmall(B,D::ConstraintDelta,2) && RefusesSmall(B,D::FrameMatrix,2));
    CHECK(RefusesSmall(B,D::CommitStaging,7) && RefusesSmall(B,D::CommitStaging,2));
    B.commits[1].outcome.resize(1);
    CHECK(RefusesSmall(B,D::CommitStaging,4));
}
}
int main()
{
    TestConstantSourceKeys(); TestPathLeafContentVersions(); TestSpaceLeafIndexOverlay();
    TestSkinEffectiveSelectedTopology(); TestFloatPayloadBits(); TestFieldValidityCountError(); TestPropertyValidityAndLadderState();
    TestChunkRangeIsolation(); TestConstraintSourceAndPropertyAliasKeys();
    TestPacketStatusAndOpaqueBoundary(); TestRawInputAndProviderKeys(); TestAvarEffectiveSelection();
    TestAvarBindingIndex();
    TestPathTextSpelling(); TestBoxTags(); TestHeadValueBits(); TestCandidateFallbackBytes();
    TestProviderOwnerText(); TestBulkValueKeyBytes(); TestSharedKeyRunsAndPlainValues();
    TestPointContentVersions(); TestSmallValuePublication();
    std::printf("OpValues: %d failures\n",failures);
    return failures ? 1 : 0;
}
