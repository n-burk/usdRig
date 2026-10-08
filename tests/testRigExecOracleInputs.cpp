#include "rigExec/weightReference.h"
#include "rigExec/oracleInputs.h"
#include "rigExec/scalarReferenceAdapter.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/rigEvaluator.h"
#include "pxr/usd/usd/stage.h"
#include <iostream>
#define CHECK(x) do { if (!(x)) { std::cerr << "failed: " << #x << "\n"; return 1; } } while(false)
#include <limits>
using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE
int main() {
    RigExecWeightReferenceContext context;
    RigExecWeightReferencePrim staticWeight;
    staticWeight.exists=true;staticWeight.path=SdfPath("/W");staticWeight.type=TfToken("RigExecStaticWeight");
    RigExecWeightReferenceAttribute values;values.exists=true;values.raw=VtValue(VtFloatArray{0.25f,0.75f});
    staticWeight.attributes[TfToken("rigExec:values")]=values;
    RigExecWeightReferenceAttribute representation;representation.exists=true;representation.raw=VtValue(TfToken("dense"));
    staticWeight.attributes[TfToken("rigExec:representation")]=representation;
    context.prims[staticWeight.path]=staticWeight;
    std::vector<float> result;std::string error;
    CHECK(RigExecResolveWeightReference(context,staticWeight.path,2,&result,&error));
    CHECK(result==std::vector<float>({0.25f,0.75f}));
    // Raw candidates, not pre-clamped production fields, reach the reference.
    context.prims[staticWeight.path].attributes[TfToken("rigExec:values")].raw=VtValue(VtFloatArray{1.25f,0.75f});
    CHECK(!RigExecResolveWeightReference(context,staticWeight.path,2,&result,&error));
    CHECK(error=="weight range violation on /W");
    RigExecWeightReferenceAttribute policy;policy.exists=true;policy.raw=VtValue(TfToken("clamp"));
    context.prims[staticWeight.path].attributes[TfToken("rigExec:rangePolicy")]=policy;
    CHECK(RigExecResolveWeightReference(context,staticWeight.path,2,&result,&error));
    CHECK(result==std::vector<float>({1.0f,0.75f}));
    context.prims[staticWeight.path].attributes[TfToken("rigExec:values")].timeSamples=1;
    CHECK(!RigExecResolveWeightReference(context,staticWeight.path,2,&result,&error));
    CHECK(error=="static weight field rigExec:values has time samples or connections on /W");
    // Cycle safety derives only from captured original source relationships.
    // It is independent of the production compiler's SCC result.
    RigExecWeightReferenceContext cyclic;
    RigExecWeightReferencePrim combine;
    combine.exists=true;combine.path=SdfPath("/Cycle");combine.type=TfToken("RigExecCombineWeight");
    RigExecWeightReferenceRelationship inputs;
    inputs.exists=true;inputs.targets={combine.path};
    combine.relationships[TfToken("rigExec:inputWeights")]=inputs;
    cyclic.prims[combine.path]=combine;
    CHECK(!RigExecResolveWeightReference(cyclic,combine.path,1,&result,&error));
    CHECK(error=="cyclic weight object closure at /Cycle");
    cyclic.prims[combine.path].relationships[TfToken("rigExec:inputWeights")].targets.clear();
    CHECK(RigExecResolveWeightReference(cyclic,combine.path,1,&result,&error));
    // A typed oracle read distinguishes successful empty from missing values.
    RigExecOracleAttribute a;a.exists=true;a.atTime=VtValue(VtVec3fArray());
    VtVec3fArray points{GfVec3f(1)};
    CHECK(a.Get(&points,UsdTimeCode(1))&&points.empty());
    a.atTime=VtValue();points=VtVec3fArray{GfVec3f(1)};
    CHECK(!a.Get(&points,UsdTimeCode(1))&&points.size()==1);
    // Float and double walks retain their separately captured typed answers.
    a.resolvedTime[std::type_index(typeid(float))]=VtValue(0.125f);
    a.resolvedTime[std::type_index(typeid(double))]=VtValue(0.12500000000000003);
    RigExecOracleScene scene;float f=0;double d=0;
    CHECK(scene.GetAttribute(a,UsdTimeCode(1),&f));CHECK(scene.GetAttribute(a,UsdTimeCode(1),&d));
    CHECK(f==0.125f&&d==0.12500000000000003);
    const SdfPath shared("/Input.value"),readerAlias("/Reader.value");
    RigExecOracleScene source;source.overlay[shared]=VtValue(99.0f);
    source.overlay[readerAlias]=VtValue(7.0f);
    RigExecOraclePublicationContext publications;
    publications.Begin(1,source,2,{shared,readerAlias},{readerAlias});
    CHECK(!publications.ReaderScene({0,1}).Find(shared));
    // Completion order differs from original writer order: chain 1 still wins.
    CHECK(publications.Finish(1,{{shared,VtValue(2.0f)}}));
    CHECK(publications.Finish(0,{{shared,VtValue(1.0f)},{readerAlias,VtValue(3.0f)}}));
    auto both=publications.ReaderScene({0,1});
    CHECK(both.Get(shared,&f)&&f==2.0f);
    CHECK(both.Get(readerAlias,&f)&&f==7.0f);
    auto earlier=publications.ReaderScene({0});CHECK(earlier.Get(shared,&f)&&f==1.0f);
    publications.Begin(2,source,2,{shared,readerAlias},{readerAlias});
    CHECK(!publications.ReaderScene({0,1}).Find(shared));
    CHECK(!publications.Finish(2,{}));
    // Reader overlays are private; owned captured facts survive a later Begin.
    RigExecOracleScene factsSource;
    RigExecOraclePrim factsPrim;factsPrim.exists=true;factsPrim.path=SdfPath("/Facts");
    factsPrim.type=TfToken("Xform");
    RigExecOracleAttribute factsAttr;factsAttr.exists=true;factsAttr.path=SdfPath("/Facts.value");
    factsAttr.type=SdfValueTypeNames->Double;
    factsAttr.atDefault=VtValue(0.25);factsAttr.atTime=VtValue(0.5);
    factsAttr.resolvedDefault[std::type_index(typeid(double))]=VtValue(0.375);
    factsAttr.resolvedTime[std::type_index(typeid(double))]=VtValue(0.625);
    factsAttr.connected=true;factsAttr.connections={shared};
    factsPrim.attributes[TfToken("value")]=factsAttr;
    RigExecOracleRelationship factsRel;factsRel.exists=true;factsRel.targets={SdfPath("/Target")};
    factsPrim.relationships[TfToken("relationship")]=factsRel;
    factsSource.prims[factsPrim.path]=factsPrim;
    factsSource.primOrder={factsPrim.path};factsSource.blendShapes.insert(factsPrim.path);
    publications.Begin(3,factsSource,1,{},{});
    auto factsReader=publications.ReaderScene({0});
    auto otherReader=publications.ReaderScene({0});
    CHECK(&factsReader.Prims()==&otherReader.Prims());
    CHECK(factsReader.PrimOrder()==std::vector<SdfPath>({factsPrim.path}));
    CHECK(factsReader.BlendShapes().count(factsPrim.path)==1);
    const auto readerAttr=factsReader.GetAttributeAtPath(factsAttr.path);
    CHECK(readerAttr.type==factsAttr.type&&readerAttr.connected);
    CHECK(readerAttr.connections==factsAttr.connections);
    CHECK(readerAttr.atDefault==factsAttr.atDefault&&readerAttr.atTime==factsAttr.atTime);
    CHECK(readerAttr.resolvedDefault==factsAttr.resolvedDefault);
    CHECK(readerAttr.resolvedTime==factsAttr.resolvedTime);
    CHECK(!factsReader.GetAttributeAtPath(SdfPath("/Missing.value")));
    CHECK(!factsReader.GetAttributeAtPath(SdfPath("/Facts.missing")));
    CHECK(factsSource.GetAttributeAtPath(factsAttr.path).atTime==factsAttr.atTime);
    auto independentAttribute=factsReader.GetAttributeAtPath(factsAttr.path);
    independentAttribute.atTime=VtValue(17.0);
    independentAttribute.connections.clear();
    independentAttribute.resolvedTime.clear();
    CHECK(factsReader.GetAttributeAtPath(factsAttr.path).atTime==factsAttr.atTime);
    CHECK(factsReader.GetAttributeAtPath(factsAttr.path).connections==factsAttr.connections);
    CHECK(factsReader.GetAttributeAtPath(factsAttr.path).resolvedTime==factsAttr.resolvedTime);
    CHECK(factsReader.GetPrimAtPath(factsPrim.path).GetRelationship(TfToken("relationship")).targets==factsRel.targets);
    factsReader.overlay[shared]=VtValue(8.0f);
    CHECK(!otherReader.Find(shared)&&!publications.SourceScene().Find(shared));
    factsSource.prims[factsPrim.path].attributes[TfToken("value")].atTime=VtValue(9.0);
    publications.Begin(4,std::move(factsSource),1,{},{});
    CHECK(factsReader.GetAttributeAtPath(factsAttr.path).atTime==VtValue(0.5));
    CHECK(publications.ReaderScene({0}).GetAttributeAtPath(factsAttr.path).atTime==VtValue(9.0));
    CHECK(factsReader.Get(shared,&f)&&f==8.0f);
    CHECK(factsReader.resolveFromFacts&&otherReader.resolveFromFacts);
    RigExecOracleScene ownerlessReader;
    {
        RigExecOraclePublicationContext temporary;
        temporary.Begin(1,factsReader,0,{},{});
        ownerlessReader=temporary.ReaderScene({});
    }
    CHECK(ownerlessReader.GetAttributeAtPath(factsAttr.path).atTime==VtValue(0.5));
    // A float hop onto a double restarts traversal with its own cycle guard.
    RigExecOraclePrim prim;prim.exists=true;prim.path=SdfPath("/P");
    RigExecOracleAttribute floatHop;floatHop.exists=true;floatHop.path=SdfPath("/P.f");
    floatHop.type=SdfValueTypeNames->Float;floatHop.atTime=VtValue(0.5f);
    floatHop.connected=true;floatHop.connections={SdfPath("/P.d")};
    RigExecOracleAttribute doubleHop;doubleHop.exists=true;doubleHop.path=SdfPath("/P.d");
    doubleHop.type=SdfValueTypeNames->Double;doubleHop.atTime=VtValue(0.75);
    doubleHop.connected=true;doubleHop.connections={SdfPath("/P.f")};
    prim.attributes[TfToken("f")]=floatHop;prim.attributes[TfToken("d")]=doubleHop;
    RigExecOracleScene walk;walk.prims[prim.path]=prim;walk.resolveFromFacts=true;
    CHECK(walk.GetAttribute(floatHop,UsdTimeCode(1),&f)&&f==0.75f);
    CHECK(walk.GetAttribute(doubleHop,UsdTimeCode(1),&d)&&d==0.75);
    publications.Begin(5,walk,0,{},{});
    const auto sharedWalk=publications.ReaderScene({});
    CHECK(sharedWalk.GetAttribute(floatHop,UsdTimeCode(1),&f)&&f==0.75f);
    CHECK(sharedWalk.GetAttribute(doubleHop,UsdTimeCode(1),&d)&&d==0.75);
    const auto stage=UsdStage::CreateInMemory();
    const auto sourcePrim=stage->DefinePrim(SdfPath("/Source"));
    auto sf=sourcePrim.CreateAttribute(TfToken("f"),SdfValueTypeNames->Float);
    auto sd=sourcePrim.CreateAttribute(TfToken("d"),SdfValueTypeNames->Double);
    CHECK(sf.Set(0.5f));CHECK(sd.Set(0.75));CHECK(sd.Set(0.875,UsdTimeCode(1)));
    CHECK(sf.SetConnections({sd.GetPath()}));CHECK(sd.SetConnections({sf.GetPath()}));
    RigExecResolvedInputs original;
    for(const auto time:{UsdTimeCode::Default(),UsdTimeCode(1)}) {
        auto captured=RigExecCaptureOracleInputs(stage,original,time,0,{sourcePrim.GetPath()},{});
        captured.resolveFromFacts=true;
        float expectedFloat=0.0f,actualFloat=0.0f;
        double expectedDouble=0.0,actualDouble=0.0;
        CHECK(original.GetAttribute(sf,time,&expectedFloat));
        CHECK(captured.GetAttribute(captured.GetAttributeAtPath(sf.GetPath()),time,&actualFloat));
        CHECK(expectedFloat==actualFloat);
        CHECK(original.GetAttribute(sd,time,&expectedDouble));
        CHECK(captured.GetAttribute(captured.GetAttributeAtPath(sd.GetPath()),time,&actualDouble));
        CHECK(expectedDouble==actualDouble);
    }
    // Full generic capture keeps its typed maps. Publication capture retains
    // every source fact but resolves independently from raw facts and overlays.
    const auto remote=stage->DefinePrim(SdfPath("/Remote"));
    auto hop=remote.CreateAttribute(TfToken("hop"),SdfValueTypeNames->Double);
    CHECK(hop.Set(0.25));CHECK(hop.Set(0.5,UsdTimeCode(1)));
    auto fallback=sourcePrim.CreateAttribute(TfToken("fallback"),SdfValueTypeNames->Float);
    CHECK(fallback.Set(0.375f));CHECK(fallback.SetConnections({SdfPath("/Missing.value")}));
    auto multiple=sourcePrim.CreateAttribute(TfToken("multiple"),SdfValueTypeNames->Float);
    CHECK(multiple.Set(0.625f));CHECK(multiple.SetConnections({sf.GetPath(),hop.GetPath()}));
    auto linked=sourcePrim.CreateAttribute(TfToken("linked"),SdfValueTypeNames->Float);
    CHECK(linked.Set(0.125f));CHECK(linked.SetConnections({hop.GetPath()}));
    auto authoredPointsAttr=sourcePrim.CreateAttribute(TfToken("points"),SdfValueTypeNames->Point3fArray);
    CHECK(authoredPointsAttr.Set(VtVec3fArray{GfVec3f(1,2,3)}));
    auto targets=sourcePrim.CreateRelationship(TfToken("targets"));
    const auto shape=stage->DefinePrim(SdfPath("/Shape"),TfToken("BlendShape"));
    CHECK(targets.SetTargets({remote.GetPath(),shape.GetPath()}));
    const std::map<SdfPath,VtValue> upstream={{authoredPointsAttr.GetPath(),VtValue(VtVec3fArray{GfVec3f(4,5,6)})}};
    RigExecStaticInputCache captureCache;original.SetStaticCache(&captureCache);
    original.SetProperty(hop.GetPath(),VtValue(0.9375));
    for(const auto time:{UsdTimeCode::Default(),UsdTimeCode(1)}) {
        auto full=RigExecCaptureOracleInputs(stage,original,time,0,{sourcePrim.GetPath()},upstream);
        auto facts=RigExecCaptureOracleInputs(stage,original,time,0,{sourcePrim.GetPath()},upstream,
            RigExecOracleCaptureMode::PublicationFacts);
        CHECK(!full.resolveFromFacts&&facts.resolveFromFacts);
        CHECK(!full.GetAttributeAtPath(sd.GetPath()).resolvedDefault.empty());
        CHECK(!full.GetAttributeAtPath(sd.GetPath()).resolvedTime.empty());
        CHECK(full.Prims().size()==facts.Prims().size());
        CHECK(full.PrimOrder()==facts.PrimOrder()&&full.BlendShapes()==facts.BlendShapes());
        CHECK(full.overlay==facts.overlay);
        CHECK(original.Find(hop.GetPath())&&*original.Find(hop.GetPath())==VtValue(0.9375));
        CHECK(facts.BlendShapes().count(shape.GetPath())==1);
        CHECK(facts.GetPrimAtPath(remote.GetPath()));
        for(const auto &entry:full.Prims()) {
            const auto &a=entry.second;const auto &b=facts.Prims().at(entry.first);
            CHECK(a.exists==b.exists&&a.path==b.path&&a.type==b.type);
            CHECK(a.attributes.size()==b.attributes.size()&&a.relationships.size()==b.relationships.size());
            for(const auto &attribute:a.attributes) {
                const auto &x=attribute.second;const auto &y=b.attributes.at(attribute.first);
                CHECK(x.exists==y.exists&&x.path==y.path&&x.type==y.type);
                CHECK(x.connected==y.connected&&x.connections==y.connections);
                CHECK(x.atDefault==y.atDefault&&x.atTime==y.atTime);
                CHECK(y.resolvedDefault.empty()&&y.resolvedTime.empty());
            }
            for(const auto &relation:a.relationships) {
                const auto &x=relation.second;const auto &y=b.relationships.at(relation.first);
                CHECK(x.exists==y.exists&&x.targets==y.targets);
                CHECK(x.phase.kind==y.phase.kind&&x.phase.prim==y.phase.prim);
            }
        }
        RigExecOraclePublicationContext fullContext,factsContext;
        fullContext.Begin(10,std::move(full),1,{},{});
        factsContext.Begin(10,std::move(facts),1,{},{});
        auto fullReader=fullContext.ReaderScene({});auto factsReader=factsContext.ReaderScene({});
        for(const auto &attribute:{sf,sd,fallback,multiple,linked,hop}) {
            float expected=0,fullValue=0,factsValue=0;
            const bool expectedOk=original.GetAttribute(attribute,time,&expected);
            const bool fullOk=fullReader.GetAttribute(fullReader.GetAttributeAtPath(attribute.GetPath()),time,&fullValue);
            const bool factsOk=factsReader.GetAttribute(factsReader.GetAttributeAtPath(attribute.GetPath()),time,&factsValue);
            CHECK(expectedOk==fullOk&&fullOk==factsOk);
            CHECK(!expectedOk||(expected==fullValue&&fullValue==factsValue));
        }
        double fullDouble=0,factsDouble=0,expectedDouble=0;
        CHECK(original.GetAttribute(sd,time,&expectedDouble));
        CHECK(fullReader.GetAttribute(fullReader.GetAttributeAtPath(sd.GetPath()),time,&fullDouble));
        CHECK(factsReader.GetAttribute(factsReader.GetAttributeAtPath(sd.GetPath()),time,&factsDouble));
        CHECK(expectedDouble==fullDouble&&fullDouble==factsDouble);
        VtVec3fArray actualPoints;
        CHECK(factsReader.GetAttributeAtPath(authoredPointsAttr.GetPath()).Get(&actualPoints,time));
        CHECK(actualPoints==upstream.at(authoredPointsAttr.GetPath()).UncheckedGet<VtVec3fArray>());
        const RigExecOraclePublicationContext::Writes writes={{hop.GetPath(),VtValue(0.8125)}};
        CHECK(fullContext.Finish(0,writes)&&factsContext.Finish(0,writes));
        const auto publishedFull=fullContext.ReaderScene({0});
        const auto publishedFacts=factsContext.ReaderScene({0});
        float publishedValue=0,publishedExpected=0;
        CHECK(publishedFull.GetAttribute(publishedFull.GetAttributeAtPath(linked.GetPath()),time,&publishedExpected));
        CHECK(publishedFacts.GetAttribute(publishedFacts.GetAttributeAtPath(linked.GetPath()),time,&publishedValue));
        CHECK(publishedValue==0.8125f&&publishedValue==publishedExpected);
        CHECK(factsReader.GetAttribute(factsReader.GetAttributeAtPath(linked.GetPath()),time,&publishedValue));
        CHECK(publishedValue==0.9375f);
    }
    // Detached after-join reference has no stage or evaluator; disagreement
    // is observable even when the production chain retained a result.
    RigExecBakedProgramImpl detached;
    detached.oraclePublications.emplace();
    RigExecOracleScene sampled;
    RigExecOraclePrim geometry;geometry.exists=true;geometry.path=SdfPath("/G");
    RigExecOracleAttribute pointsFact;pointsFact.exists=true;pointsFact.path=SdfPath("/G.points");
    pointsFact.type=SdfValueTypeNames->Point3fArray;
    pointsFact.atDefault=pointsFact.atTime=VtValue(VtVec3fArray{GfVec3f(1,2,3)});
    geometry.attributes[TfToken("points")]=pointsFact;sampled.prims[geometry.path]=geometry;
    detached.oraclePublications->Begin(7,std::move(sampled),0,{},{});
    detached.chains.emplace_back();auto &geometryChain=detached.chains.back();
    geometryChain.target=pointsFact.path;geometryChain.haveBase=true;geometryChain.haveResult=true;
    geometryChain.result=VtVec3fArray{GfVec3f(1,2,3)};
    RigExecRigPose referencePose;
    CHECK(RigExecBakedRunScalarReference(&detached,UsdTimeCode::Default(),&referencePose));
    CHECK(referencePose.referenceAgreements==1);
    CHECK(referencePose.movedPropertiesCpu.at(pointsFact.path)==pointsFact.atDefault);
    geometryChain.result[0]=GfVec3f(1,2,4);
    RigExecRigPose mismatchPose;
    CHECK(!RigExecBakedRunScalarReference(&detached,UsdTimeCode::Default(),&mismatchPose));
    CHECK(mismatchPose.referenceMismatches==1);
    detached.opGraph.cycles={{"a","b","a"},{"x","x"}};
    RigExecRigPose cyclePose;
    RigExecBakedAppendCycleDiagnostics(detached,&cyclePose);
    RigExecBakedAppendCycleDiagnostics(detached,&cyclePose);
    CHECK(cyclePose.diagnostics.size()==2);
    CHECK(cyclePose.diagnostics[0]=="operation cycle: a -> b -> a");
    CHECK(cyclePose.diagnostics[1]=="operation cycle: x -> x");
    RigExecBakedPrepareOracleReference(&detached,UsdTimeCode::Default());
    CHECK(detached.oraclePublications->Generation()==8);
    return 0;
}
