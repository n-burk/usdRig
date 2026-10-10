#include "rigExecMath/geometryKernels.h"
#include "rigExec/rigEvaluator.h"
#include "rigExecRigging/rigBuilder.h"
#include "rigExecBake/bake.h"
#include "rigExecRuntime/runtime.h"
#include <cstring>
#include "pxr/base/plug/registry.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/usd/usdGeom/mesh.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <stdexcept>
using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE
#define CHECK(x) do {if(!(x))throw std::runtime_error(#x);}while(false)
#include "rigExecRuntimeDrive.h"
#include "deltaMushReferenceFixtures.h"
static bool Near(const std::vector<GfVec3f> &a,const std::vector<GfVec3f> &b,double e=1e-5) {
    if(a.size()!=b.size())return false;
    for(size_t i=0;i<a.size();++i)if((a[i]-b[i]).GetLength()>e)return false;
    return true;
}
static void CheckReferenceFixtures() {
    for (const auto &fixture : DeltaMushReferenceFixtures()) {
        RigExecDeltaMushSettings settings;
        settings.smoothing = fixture.lengthWeighted ? RigExecDeltaMushSmoothing::LengthWeighted : RigExecDeltaMushSmoothing::Simple;
        settings.frameTransport = RigExecDeltaMushFrameTransport::Corner;
        settings.smoothWeights = fixture.weights;
        settings.edges = fixture.edges;
        settings.onlySmooth = fixture.onlySmooth;
        auto actual = fixture.input;
        CHECK(RigExecApplyDeltaMush(&actual, fixture.rest, fixture.counts, fixture.indices,
            4, fixture.step, fixture.pin, 0, fixture.detail, settings));
        CHECK(Near(actual, fixture.expected, 2e-6));
        // A nonuniform affine owner map must preserve local smoothing behavior.
        GfMatrix4d computationToTarget(1);
        computationToTarget.SetScale(GfVec3d(1.4, 0.73, 1.15));
        GfMatrix4d rotate(1); rotate.SetRotate(GfRotation(GfVec3d(1, 2, 3), 28));
        computationToTarget *= rotate;
        computationToTarget.SetTranslateOnly(GfVec3d(0.3, -0.2, 0.4));
        auto transformedInput = fixture.input, transformedExpected = fixture.expected;
        for (auto &p : transformedInput) p = GfVec3f(computationToTarget.Transform(GfVec3d(p)));
        for (auto &p : transformedExpected) p = GfVec3f(computationToTarget.Transform(GfVec3d(p)));
        actual = transformedInput;
        CHECK(RigExecApplyDeltaMush(&actual, fixture.rest, fixture.counts, fixture.indices,
            4, fixture.step, fixture.pin, 0, fixture.detail, settings, computationToTarget));
        CHECK(Near(actual, transformedExpected, 3e-6));
        auto stage = UsdStage::CreateInMemory();
        auto rig = RigExecRigBuilder::Create(stage, SdfPath("/Rig"));
        auto mesh = UsdGeomMesh::Define(stage, SdfPath("/Rig/Body"));
        mesh.CreatePointsAttr().Set(VtVec3fArray(transformedInput.begin(), transformedInput.end()));
        mesh.CreateFaceVertexCountsAttr().Set(VtIntArray(fixture.counts.begin(), fixture.counts.end()));
        mesh.CreateFaceVertexIndicesAttr().Set(VtIntArray(fixture.indices.begin(), fixture.indices.end()));
        auto mush = rig.NewMoverChain("deform", mesh.GetPointsAttr().GetPath()).AddDeltaMushMover("mush");
        mush.SetRestPoints(fixture.rest); mush.SetIterations(4); mush.SetStep(fixture.step);
        mush.SetPinBorders(fixture.pin);
        auto prim = mush.GetPrim();
        CHECK(prim.GetAttribute(TfToken("inputs:smoothing")).Set(TfToken(fixture.lengthWeighted ? "lengthWeighted" : "simple")));
        CHECK(prim.GetAttribute(TfToken("inputs:frameTransport")).Set(TfToken("corner")));
        CHECK(prim.GetAttribute(TfToken("inputs:displacement")).Set(fixture.detail));
        CHECK(prim.GetAttribute(TfToken("inputs:smoothWeights")).Set(VtFloatArray(fixture.weights.begin(), fixture.weights.end())));
        CHECK(prim.GetAttribute(TfToken("inputs:edges")).Set(VtIntArray(fixture.edges.begin(), fixture.edges.end())));
        CHECK(prim.GetAttribute(TfToken("inputs:onlySmooth")).Set(fixture.onlySmooth));
        CHECK(prim.GetAttribute(TfToken("inputs:computationToTarget")).Set(computationToTarget));
        RigExecRigEvaluator evaluator(stage, SdfPath("/Rig"));
        // The independent scalar reference judges every evaluation.
        evaluator.cpuReference = true;
        std::vector<std::string> diagnostics;
        CHECK(evaluator.Compile(&diagnostics));
        std::string before; stage->GetRootLayer()->ExportToString(&before);
        auto pose = evaluator.Evaluate(UsdTimeCode(1));
        CHECK(pose.valid); CHECK(pose.referenceMismatches == 0); CHECK(pose.referenceAgreements > 0);
        auto value = pose.movedProperties.at(mesh.GetPointsAttr().GetPath()).Get<VtVec3fArray>();
        CHECK(Near({value.begin(), value.end()}, transformedExpected, 3e-6));
        RigExecBakeOpts options; options.time = 1;
        RigExecBakeResult result; std::string error;
        CHECK(RigExecBakeToBinary(evaluator, options, &result, &error));
        auto reader = RigExecRuntimeReader::Open(result.bytes.data(), result.bytes.size(), &error);
        CHECK(reader); CHECK(reader->Execute(&error));
        bool found = false;
        for (const auto &points : reader->GetPoints()) {
            if (points.path != mesh.GetPointsAttr().GetPath().GetString()) continue;
            found = true; CHECK(points.points.size() == value.size());
            for (size_t i = 0; i < value.size(); ++i)
                CHECK(std::memcmp(points.points[i].data(), value[i].data(), 3*sizeof(float)) == 0);
        }
        CHECK(found);
        std::string after; stage->GetRootLayer()->ExportToString(&after); CHECK(before == after);
        // Live frame folding must use the evaluated provider, including binary
        // avar edits after an export containing only its identity pose.
        auto owner = rig.AddControl("Owner", GfMatrix4d(1));
        CHECK(prim.GetRelationship(TfToken("rigExec:frame")).SetTargets({owner.GetPath()}));
        mush.SetReadPhase(TfToken("rigExec:frame"), "final");
        CHECK(RigExecBakeToBinary(evaluator, options, &result, &error));
        reader = RigExecRuntimeReader::Open(result.bytes.data(), result.bytes.size(), &error);
        CHECK(reader);
        for (double scale : {1.0, 0.74, 1.31, 1.0}) {
            owner.SetAvarScale(1, scale, 1);
            CHECK(reader->SetInput(owner.GetPath().AppendProperty(TfToken("avars:sy")).GetString(), scale, &error));
            pose = evaluator.Evaluate(UsdTimeCode(1));
            CHECK(pose.valid); CHECK(pose.referenceMismatches == 0); CHECK(pose.referenceAgreements > 0);
            CHECK(reader->Execute(&error));
            value = pose.movedProperties.at(mesh.GetPointsAttr().GetPath()).Get<VtVec3fArray>();
            GfMatrix4d scaling(1); scaling.SetScale(GfVec3d(1, scale, 1));
            actual = transformedInput;
            CHECK(RigExecApplyDeltaMush(&actual, fixture.rest, fixture.counts, fixture.indices,
                4, fixture.step, fixture.pin, 0, fixture.detail, settings, computationToTarget * scaling));
            CHECK(Near({value.begin(), value.end()}, actual, 3e-6));
            found = false;
            for (const auto &points : reader->GetPoints()) {
                if (points.path != mesh.GetPointsAttr().GetPath().GetString()) continue;
                found = true; CHECK(points.points.size() == value.size());
                for (size_t i = 0; i < value.size(); ++i)
                    CHECK(std::memcmp(points.points[i].data(), value[i].data(), 3*sizeof(float)) == 0);
            }
            CHECK(found);
        }
        mush.SetReadPhase(TfToken("rigExec:frame"), "/Rig/Movers");
        RigExecRigEvaluator invalidPhase(stage, SdfPath("/Rig"));
        diagnostics.clear(); CHECK(invalidPhase.Compile(&diagnostics));
        CHECK(std::any_of(diagnostics.begin(), diagnostics.end(), [](const std::string &d) {
            return d.find("frame read phase must be base or final") != std::string::npos;
        }));
    }
}
int main(int argc,char **argv) {
 try {
    CHECK(argc==2);PlugRegistry::GetInstance().RegisterPlugins(argv[1]);
    CheckReferenceFixtures();
    std::vector<GfVec3f> rest{{1,0,0},{0,1.2f,0},{-1,0,0},{0,-1,0},{0,0,1},{0,0,-1}};
    std::vector<int> counts(8,3),indices{0,1,4,1,2,4,2,3,4,3,0,4,1,0,5,2,1,5,3,2,5,0,3,5};
    auto saved=rest;CHECK(RigExecApplyDeltaMush(&saved,rest,counts,indices));CHECK(Near(saved,rest));
    GfMatrix4d matrix(1);matrix.SetRotate(GfRotation(GfVec3d(0,0,1),37));matrix.SetTranslateOnly(GfVec3d(4,2,-3));
    auto rigid=rest;for(auto &p:rigid)p=GfVec3f(matrix.Transform(GfVec3d(p)));
    auto expected=rigid;CHECK(RigExecApplyDeltaMush(&rigid,rest,counts,indices,3));CHECK(Near(rigid,expected));
    auto posed=rest;posed[4]+=GfVec3f(0.7f,0,0.2f);
    auto full=posed;CHECK(RigExecApplyDeltaMush(&full,rest,counts,indices,3));CHECK(!Near(full,posed,0.01));
    auto zero=posed;CHECK(RigExecApplyDeltaMush(&zero,rest,counts,indices,0));CHECK(zero==posed);
    auto invalid=posed;auto bad=indices;bad.back()=100;
    CHECK(!RigExecApplyDeltaMush(&invalid,rest,counts,bad));CHECK(invalid==posed);
    CHECK(!RigExecApplyDeltaMush(&invalid,rest,counts,indices,-1));CHECK(invalid==posed);
    CHECK(!RigExecApplyDeltaMush(&invalid,{},counts,indices));CHECK(invalid==posed);
    // An open quad has only border vertices, all of which must remain exact.
    std::vector<GfVec3f> quad{{0,0,0},{1,0,0},{1,1,0},{0,1,0}},bent=quad;
    bent[2][2]=1;auto pinned=bent;
    CHECK(RigExecApplyDeltaMush(&pinned,quad,{4},{0,1,2,3}));CHECK(pinned==bent);
    // Author a native operator and exercise runtime edits and serialization.
    auto stage=UsdStage::CreateInMemory();auto rig=RigExecRigBuilder::Create(stage,SdfPath("/Rig"));
    auto mesh=UsdGeomMesh::Define(stage,SdfPath("/Rig/Body"));
    mesh.CreatePointsAttr().Set(VtVec3fArray(posed.begin(),posed.end()));
    mesh.CreateFaceVertexCountsAttr().Set(VtIntArray(counts.begin(),counts.end()));
    mesh.CreateFaceVertexIndicesAttr().Set(VtIntArray(indices.begin(),indices.end()));
    auto mush=rig.NewMoverChain("deform",mesh.GetPointsAttr().GetPath()).AddDeltaMushMover("mush");
    mush.SetRestPoints(rest);mush.SetIterations(3);
    RigExecRigEvaluator evaluator(stage,SdfPath("/Rig"));std::vector<std::string> diagnostics;
    CHECK(evaluator.Compile(&diagnostics));
    if(!evaluator.IsBakeable(&diagnostics)) {
        for(const auto &d:diagnostics)std::cerr<<d<<'\n';
        CHECK(false);
    }
    auto evaluate=[&](){auto p=evaluator.Evaluate(UsdTimeCode::Default());CHECK(p.valid);
        CHECK(p.comparisonMismatches==0);CHECK(p.referenceMismatches==0);
        CHECK(evaluator.GetBakedProgram()!=nullptr);
        auto v=p.movedProperties.at(mesh.GetPointsAttr().GetPath()).Get<VtVec3fArray>();
        return std::vector<GfVec3f>(v.begin(),v.end());};
    CHECK(Near(evaluate(),full));
    mush.SetDefaultWeight(0.5f);auto halfway=posed;
    for(size_t i=0;i<posed.size();++i)halfway[i]=(posed[i]+full[i])*0.5f;
    CHECK(Near(evaluate(),halfway));
    mush.SetDefaultWeight(0);CHECK(Near(evaluate(),posed));
    mush.SetDefaultWeight(1);mush.SetIterations(0);CHECK(Near(evaluate(),posed));
    mush.SetIterations(3);CHECK(Near(evaluate(),full));
    auto weight=rig.AddStaticWeight("mask",mesh.GetPointsAttr().GetPath(),{0,1,0.5f,1,0,1});
    mush.SetWeightObject(weight.GetPath());auto masked=posed;
    const float mask[6]={0,1,0.5f,1,0,1};
    for(size_t i=0;i<posed.size();++i)masked[i]=posed[i]+mask[i]*(full[i]-posed[i]);
    CHECK(Near(evaluate(),masked));
    mush.GetPrim().GetRelationship(TfToken("rigExec:weightObject")).ClearTargets(true);
    mush.SetDistanceWeight(1);auto weighted=posed;
    CHECK(RigExecApplyDeltaMush(&weighted,rest,counts,indices,3,0.5,true,1));
    CHECK(Near(evaluate(),weighted));CHECK(!Near(weighted,full));
    mush.SetDistanceWeight(0);
    mush.SetDisplacement(0);auto smooth=posed;
    CHECK(RigExecApplyDeltaMush(&smooth,rest,counts,indices,3,0.5,true,0,0));CHECK(Near(evaluate(),smooth));
    std::string text;stage->GetRootLayer()->ExportToString(&text);
    auto layer=SdfLayer::CreateAnonymous("roundtrip.usda");CHECK(layer->ImportFromString(text));
    auto reopened=UsdStage::Open(layer);RigExecRigEvaluator e2(reopened,SdfPath("/Rig"));
    e2.cpuReference=true;CHECK(e2.Compile(&diagnostics));
    auto p=e2.Evaluate(UsdTimeCode::Default());CHECK(p.valid);
    CHECK(p.referenceMismatches==0);CHECK(p.referenceAgreements==1);
    auto value=p.movedProperties.at(mesh.GetPointsAttr().GetPath()).Get<VtVec3fArray>();CHECK(Near({value.begin(),value.end()},smooth));
    // Binary playback must run the shared kernel, take every parameter from
    // its inputs and publish bitwise-identical points, including revisiting
    // earlier frames: baked at the first frame, played through its inputs.
    auto binaryParity = [&](const std::vector<double> &frames) {
        RigExecBakeOpts options; options.time=frames.front();
        RigExecBakeResult result; std::string error;
        CHECK(RigExecBakeToBinary(evaluator,options,&result,&error));
        CHECK(!result.bytes.empty());
        RigExecTestPlayer reader;
        if(!reader.Open(result.bytes,stage,&error))throw std::runtime_error(error);
        auto playback=frames;playback.insert(playback.end(),frames.rbegin(),frames.rend());
        for(double frame:playback) {
            auto expectedPose=evaluator.Evaluate(UsdTimeCode(frame));
            CHECK(expectedPose.valid);CHECK(expectedPose.comparisonMismatches==0);
            if(!reader.Play(frame,&error))throw std::runtime_error(error);
            bool found=false;
            for(const auto &actual:reader->GetPoints()) {
                if(actual.path!=mesh.GetPointsAttr().GetPath().GetString())continue;
                found=true;
                auto expected=expectedPose.movedProperties.at(mesh.GetPointsAttr().GetPath()).Get<VtVec3fArray>();
                CHECK(actual.points.size()==expected.size());
                for(size_t i=0;i<expected.size();++i)
                    CHECK(std::memcmp(actual.points[i].data(),expected[i].data(),3*sizeof(float))==0);
            }
            CHECK(found);
        }
    };
    const auto prim=mush.GetPrim();
    auto sample=[&](const char *name,auto value,int frame) {
        CHECK(prim.GetAttribute(TfToken(name)).Set(value,UsdTimeCode(frame)));
    };
    // Every setting differs from its fallback in at least one frame.
    sample("inputs:iterations",3,1);sample("inputs:iterations",0,2);sample("inputs:iterations",4,3);
    sample("inputs:step",0.3f,1);sample("inputs:step",0.7f,3);
    sample("inputs:distanceWeight",1.0f,1);sample("inputs:distanceWeight",2.0f,3);
    sample("inputs:displacement",0.7f,1);sample("inputs:displacement",0.0f,3);
    sample("inputs:defaultWeight",0.35f,1);sample("inputs:defaultWeight",1.0f,3);
    sample("inputs:enabled",true,1);sample("inputs:enabled",false,4);sample("inputs:enabled",true,5);
    binaryParity({1,2,3,4,5});
    auto driver=stage->GetPrimAtPath(SdfPath("/Rig")).CreateAttribute(
        TfToken("mushStep"),SdfValueTypeNames->Float);
    driver.Set(0.2f,UsdTimeCode(1));driver.Set(0.7f,UsdTimeCode(3));
    prim.GetAttribute(TfToken("inputs:step")).SetConnections({driver.GetPath()});
    mush.SetWeightObject(weight.GetPath());binaryParity({1,2,3,4,5});
    // Open border pinning and changing topology use the same serialized reads.
    mesh.GetPointsAttr().Set(VtVec3fArray(bent.begin(),bent.end()));
    mesh.GetFaceVertexCountsAttr().Set(VtIntArray{4});
    mesh.GetFaceVertexIndicesAttr().Set(VtIntArray{0,1,2,3});
    mush.SetRestPoints(quad);
    prim.GetRelationship(TfToken("rigExec:weightObject")).ClearTargets(true);
    sample("inputs:pinBorders",true,1);sample("inputs:pinBorders",false,3);
    binaryParity({1,2,3,4,5});
    // Empty rest array exercises the authored-base fallback in both hosts.
    mush.SetRestPoints({});binaryParity({1,3});
    // Invalid parameter is an atomic pass-through in both hosts.
    sample("inputs:iterations",-1,6);binaryParity({6});
    {
        // A baked input frame contains only the rest pose. Avar inputs set
        // on the binary must rerun skin -> deltaMush, not replay captured
        // deformed points.
        auto liveStage=UsdStage::CreateInMemory();
        auto builder=RigExecRigBuilder::Create(liveStage,SdfPath("/Rig"));
        auto body=UsdGeomMesh::Define(liveStage,SdfPath("/Rig/Body"));
        body.CreatePointsAttr().Set(VtVec3fArray(rest.begin(),rest.end()));
        body.CreateFaceVertexCountsAttr().Set(VtIntArray(counts.begin(),counts.end()));
        body.CreateFaceVertexIndicesAttr().Set(VtIntArray(indices.begin(),indices.end()));
        auto anchor=builder.AddControl("Anchor",GfMatrix4d(1));
        auto tip=builder.AddControl("Tip",GfMatrix4d(1));
        auto chain=builder.NewMoverChain("Deform",body.GetPointsAttr().GetPath());
        auto detail=chain.AddDeltaMushMover("Detail");
        detail.SetRestPoints(rest);detail.SetIterations(3);
        auto skin=chain.AddSkinMover("Skin",{anchor.GetPath(),tip.GetPath()});
        skin.SetJointInfluences({0,0,0,0,1,0},{1,1,1,1,1,1},1);
        RigExecRigEvaluator live(liveStage,SdfPath("/Rig"));
        RigExecBakeOpts options;options.time=1;
        RigExecBakeResult result;std::string error;
        CHECK(RigExecBakeToBinary(live,options,&result,&error));
        auto reader=RigExecRuntimeReader::Open(result.bytes.data(),result.bytes.size(),&error);
        if(!reader)throw std::runtime_error(error);
        for(double translation:{0.0,0.7,-0.4,0.0}) {
            tip.SetAvarTranslation(translation,0,0);
            CHECK(reader->SetInput(tip.GetPath().AppendProperty(TfToken("avars:tx")).GetString(),translation,&error));
            auto pose=live.Evaluate(UsdTimeCode(1));CHECK(pose.valid);
            CHECK(pose.comparisonMismatches==0);
            if(!reader->Execute(&error))throw std::runtime_error(error);
            auto points=pose.movedProperties.at(body.GetPointsAttr().GetPath()).Get<VtVec3fArray>();
            auto input=rest;input[4][0]+=float(translation);
            auto corrected=input;CHECK(RigExecApplyDeltaMush(&corrected,rest,counts,indices,3));
            CHECK(Near({points.begin(),points.end()},corrected));
            if(translation!=0)CHECK(!Near(corrected,input));
            bool found=false;
            for(const auto &actual:reader->GetPoints()) {
                if(actual.path!="/Rig/Body.points")continue;
                found=true;CHECK(actual.points.size()==points.size());
                for(size_t i=0;i<points.size();++i)
                    CHECK(std::memcmp(actual.points[i].data(),points[i].data(),3*sizeof(float))==0);
            }
            CHECK(found);
        }
    }
    std::cout<<"DeltaMush: shared kernel, USD and binary parity, animated parameters, masks, borders, invalid inputs and reload passed\n";
 }catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}
}
