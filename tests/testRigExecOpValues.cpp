#include "rigExec/bakedOpValues.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/pathText.h"
#include "rigExecGraph/providerProgram.h"
#include "pxr/base/gf/quatd.h"
#include "pxr/base/gf/vec2d.h"
#include "pxr/base/gf/vec4f.h"
#include "pxr/base/vt/types.h"
#include "pxr/base/vt/value.h"
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <set>
#include <string>
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
    TestPathTextSpelling(); TestBoxTags(); TestHeadValueBits(); TestCandidateFallbackBytes();
    TestProviderOwnerText();
    std::printf("OpValues: %d failures\n",failures);
    return failures ? 1 : 0;
}
