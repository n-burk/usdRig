#include "rigExecStandalone/pack.h"
#include "rigExecStandalone/system.h"
#include "rigExec/inputReplayValues.h"
#include "rigExec/types.h"
#include "rigExecRigging/rigBuilder.h"
#include <cstring>
#include <cstdint>
#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/vt/array.h"
#include "pxr/usd/sdf/assetPath.h"
#include "pxr/usd/sdf/attributeSpec.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/sdf/timeCode.h"
#include "pxr/usd/sdf/zipFile.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/resolveInfo.h"
#include <cstdio>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <limits>

using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE
static int failures = 0;
#define CHECK(value) do { if (!(value)) { ++failures; std::printf("FAIL %d: %s\n",__LINE__,#value); } } while (0)
static std::string Bytes(const std::string &path) {
    std::ifstream stream(path,std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(stream),{});
}

static bool Exact(const VtValue &a,const VtValue &b) {
    std::string x,y,error;
    return RigExecEncodeInputValue(a,&x,&error) && RigExecEncodeInputValue(b,&y,&error) && x==y;
}
static void TestPack(const std::filesystem::path &scratch)
{
    auto stage = UsdStage::CreateInMemory();
    stage->SetTimeCodesPerSecond(48); stage->SetFramesPerSecond(24);
    stage->SetInterpolationType(UsdInterpolationTypeHeld);
    const auto world = stage->DefinePrim(SdfPath("/World"),TfToken("Xform"));
    stage->DefinePrim(SdfPath("/World/Rig"),TfToken("RigExecRoot"));
    const auto control = stage->DefinePrim(SdfPath("/World/Rig/Control"),TfToken("RigExecControl"));
    const auto drive = world.CreateAttribute(TfToken("drive"),SdfValueTypeNames->Double);
    drive.Set(3.0); drive.Set(5.0,UsdTimeCode(1)); drive.Set(7.0,UsdTimeCode(2));
    control.GetAttribute(TfToken("avars:tx")).SetConnections({drive.GetPath()});
    world.CreateRelationship(TfToken("provider")).SetTargets({control.GetPath()});
    auto blocked = world.CreateAttribute(TfToken("blocked"),SdfValueTypeNames->Float);
    blocked.Set(2.0f,UsdTimeCode(1)); blocked.Set(SdfValueBlock(),UsdTimeCode(2));
    auto matrix = world.CreateAttribute(TfToken("matrix"),SdfValueTypeNames->Matrix4d);
    GfMatrix4d m(1); m.SetTranslate(GfVec3d(1,2,3)); matrix.Set(m);
    world.CreateAttribute(TfToken("nativeTime"),SdfValueTypeNames->TimeCode).Set(SdfTimeCode(7.25));
    const auto points = stage->DefinePrim(SdfPath("/World/Points"),TfToken("Points"));
    const VtVec3fArray vertices = {GfVec3f(1,2,3),GfVec3f(-1,4,5)};
    points.GetAttribute(TfToken("points")).Set(vertices);
    auto colors = points.CreateAttribute(TfToken("primvars:color"),SdfValueTypeNames->Color3fArray);
    colors.Set(vertices); colors.SetMetadata(TfToken("interpolation"),VtValue(TfToken("vertex")));
    colors.SetCustomDataByKey(TfToken("owner"),VtValue(std::string("test")));
    const auto assetPath = (scratch / "asset.usda").string();
    SdfLayer::CreateAnonymous("asset.usda")->Export(assetPath);
    world.CreateAttribute(TfToken("asset"),SdfValueTypeNames->Asset).Set(SdfAssetPath(assetPath));
    world.CreateAttribute(TfToken("assets"),SdfValueTypeNames->AssetArray).Set(VtArray<SdfAssetPath>{SdfAssetPath(assetPath)});
    // Semantic state comes from exact codec bytes, never the text/crate copy.
    const uint32_t nanBits=0x7fc12345;float payloadNan;
    std::memcpy(&payloadNan,&nanBits,sizeof(payloadNan));
    auto bitValues=world.CreateAttribute(TfToken("bitValues"),SdfValueTypeNames->FloatArray);
    CHECK(bitValues.Set(VtFloatArray{-0.0f,payloadNan,1.0f}));
    CHECK(bitValues.Set(VtFloatArray{},UsdTimeCode(2)));
    CHECK(bitValues.Set(VtFloatArray{payloadNan,-0.0f},UsdTimeCode(3)));
    const VtValue bitMetadata(VtDictionary{{"nested",VtValue(VtFloatArray{-0.0f,payloadNan})}});
    bitValues.SetCustomDataByKey(TfToken("bits"),bitMetadata);
    CHECK(Exact(bitValues.GetCustomDataByKey(TfToken("bits")),bitMetadata));
    const std::vector<UsdTimeCode> times = {UsdTimeCode::Default(),UsdTimeCode(1.5),
        UsdTimeCode(2),UsdTimeCode(3),UsdTimeCode::PreTime(2),UsdTimeCode(0),UsdTimeCode(-0.0)};
    const auto file = (scratch / "test.rigpack").string();
    std::string before,after,error;
    stage->GetRootLayer()->ExportToString(&before);
    CHECK(RigExecExportRigPack(stage,times,file,"asset/version1",&error));
    if (!error.empty()) std::printf("export diagnostic: %s\n",error.c_str());
    stage->GetRootLayer()->ExportToString(&after); CHECK(before == after);
    auto loaded = RigExecLoadRigPack(file,&error);
    CHECK(loaded);
    if (!loaded) { std::printf("load diagnostic: %s\n",error.c_str()); return; }
    CHECK(loaded->timeCodesPerSecond == 48 && loaded->framesPerSecond == 24);
    CHECK(loaded->interpolation == "held" && loaded->sourceAssetId == "asset/version1");
    CHECK(loaded->identities.size() == 6);
    CHECK(loaded->attributes.at(colors.GetPath()).type == SdfValueTypeNames->Color3fArray);
    CHECK(loaded->attributes.at(control.GetAttribute(TfToken("avars:tx")).GetPath()).connections == SdfPathVector{drive.GetPath()});
    CHECK(loaded->relationships.at(world.GetRelationship(TfToken("provider")).GetPath()).targets == SdfPathVector{control.GetPath()});
    for (const auto &prim : stage->TraverseAll()) {
        SdfPathVector children;
        for (const auto &child : prim.GetAllChildren()) children.push_back(child.GetPath());
        CHECK(loaded->prims.at(prim.GetPath()).children == children);
        for (const auto &attr : prim.GetAttributes()) {
            const auto &fact = loaded->attributes.at(attr.GetPath());
            std::vector<double> sampleTimes;
            attr.GetTimeSamples(&sampleTimes);
            CHECK(fact.sampleTimes == sampleTimes);
            CHECK(fact.hasValue == attr.HasValue());
            CHECK(fact.hasAuthoredValue == attr.HasAuthoredValueOpinion());
            CHECK(fact.hasAuthoredReadableValue == attr.HasAuthoredValue());
            CHECK(fact.hasAuthoredConnections == attr.HasAuthoredConnections());
            CHECK(fact.mightBeTimeVarying == attr.ValueMightBeTimeVarying());
            CHECK(fact.variability == attr.GetVariability());
            for (const auto time : times) {
                VtValue expected,actual;
                const bool hasExpected = attr.Get(&expected,time);
                CHECK(loaded->Get(attr.GetPath(),time,&actual) == hasExpected);
                if (hasExpected) CHECK(Exact(actual,expected));
                CHECK((fact.blockedIdentities.count(RigExecStandaloneTimeKey(time)) != 0) ==
                    attr.GetResolveInfo(time).ValueIsBlocked());
            }
        }
    }
    const auto &bitFact=loaded->attributes.at(bitValues.GetPath());
    CHECK(Exact(bitFact.authoredDefault,VtValue(VtFloatArray{-0.0f,payloadNan,1.0f})));
    VtValue expectedMetadata;CHECK(bitValues.GetMetadata(TfToken("customData"),&expectedMetadata));
    const auto customData=bitFact.metadata.find("customData");
    CHECK(customData!=bitFact.metadata.end());
    if(customData!=bitFact.metadata.end())CHECK(Exact(customData->second,expectedMetadata));
    VtValue bits;
    CHECK(loaded->Get(bitValues.GetPath(),UsdTimeCode(2),&bits));
    CHECK(bits.IsHolding<VtFloatArray>() && bits.UncheckedGet<VtFloatArray>().empty());
    CHECK(loaded->Get(bitValues.GetPath(),UsdTimeCode(3),&bits));
    CHECK(Exact(bits,VtValue(VtFloatArray{payloadNan,-0.0f})));
    VtValue value;
    CHECK(!loaded->Get(drive.GetPath(),UsdTimeCode(1.25),&value));
    CHECK(!loaded->Get(blocked.GetPath(),UsdTimeCode(2),&value));
    CHECK(loaded->Get(blocked.GetPath(),UsdTimeCode::PreTime(2),&value));
    const auto archive = SdfZipFile::Open(file);
    CHECK(archive && archive.Find("source.usdc") != archive.end() && archive.Find("manifest.usda") != archive.end());
    const auto second = (scratch / "again.rigpack").string();
    CHECK(RigExecExportRigPack(stage,times,second,"asset/version1",&error));
    CHECK(Bytes(file) == Bytes(second));
    // Manifest corruption is rejected before constructing a runtime adapter.
    const auto tamper = [&](const char *name, const auto &change) {
        const auto manifest = SdfLayer::CreateAnonymous("manifest.usda");
        const auto item = archive.Find("manifest.usda");
        CHECK(manifest->ImportFromString(std::string(item.GetFile(),item.GetFileInfo().size)));
        change(manifest);
        const auto manifestFile = (scratch / "tampered.usda").string();
        CHECK(manifest->Export(manifestFile));
        const auto sourceItem = archive.Find("source.usdc");
        const auto sourceFile = (scratch / "payload.usdc").string();
        { std::ofstream stream(sourceFile,std::ios::binary);
          stream.write(sourceItem.GetFile(),sourceItem.GetFileInfo().size); }
        const auto damaged = (scratch / name).string();
        auto writer = SdfZipFileWriter::CreateNew(damaged);
        CHECK(!writer.AddFile(sourceFile,"source.usdc").empty());
        CHECK(!writer.AddFile(manifestFile,"manifest.usda").empty()); CHECK(writer.Save());
        CHECK(!RigExecLoadRigPack(damaged,&error));
    };
    tamper("bad-identity.rigpack",[](const auto &manifest) {
        auto header = manifest->GetCustomLayerData();
        header["identities"] = VtValue(VtStringArray{"exact:broken"});
        manifest->SetCustomLayerData(header);
    });
    tamper("old-version.rigpack",[](const auto &manifest) {
        auto header = manifest->GetCustomLayerData();
        header["packVersion"] = VtValue(1);
        manifest->SetCustomLayerData(header);
    });
    tamper("missing-order.rigpack",[](const auto &manifest) {
        auto header = manifest->GetCustomLayerData();
        header.erase("children");
        manifest->SetCustomLayerData(header);
    });
    tamper("invalid-exact-codec.rigpack",[](const auto &manifest) {
        manifest->GetAttributeAtPath(SdfPath("/World.matrix"))->SetInfo(TfToken("customData"),
            VtValue(VtDictionary{{"__rigexecExactMetadata",VtValue(std::string("5249563100ff"))}}));
    });
    tamper("bad-schema.rigpack",[](const auto &manifest) {
        manifest->GetAttributeAtPath(SdfPath("/World.matrix"))->SetInfo(TfToken("typeName"),VtValue(TfToken("double")));
    });
    tamper("unsupported-provider.rigpack",[](const auto &manifest) {
        manifest->GetPrimAtPath(SdfPath("/World/Rig/Control"))->SetTypeName("RigExecUnregisteredSolver");
    });

    // Production movers are admitted by the full scene compiler.
    auto scalar=world.CreateAttribute(TfToken("scalar"),SdfValueTypeNames->Float);CHECK(scalar.Set(2.0f));
    auto rig=RigExecRigBuilder::Create(stage,SdfPath("/World/Rig"));
    rig.NewMoverChain("Property",scalar.GetPath()).AddFloatMathMover("Multiply",TfToken("multiply"),3.0f);
    CHECK(RigExecExportRigPack(stage,times,file,"asset/production",&error));
    auto supported=RigExecLoadRigPack(file,&error);CHECK(supported);
    if(supported) {
        RigExecStandaloneSystem production(*supported);
        const int tap=production.AddTap(RigExecValueAddress::Property(scalar.GetPath()));
        const auto result=production.Evaluate(UsdTimeCode::Default());
        CHECK(result.valid && result.Get<float>(tap)==6.0f);
    }
    // A genuinely unknown domain refuses without replacing the valid archive.
    const auto saved = Bytes(file);
    const auto unknown = stage->DefinePrim(SdfPath("/World/Rig/Unsupported"),TfToken("RigExecUnregisteredSolver"));
    CHECK(!RigExecExportRigPack(stage,times,file,"asset/version1",&error));
    CHECK(error.find("RigExecUnregisteredSolver")!=std::string::npos && Bytes(file)==saved);
    stage->RemovePrim(unknown.GetPath());
    // Solver outputs and reverse joint bindings are actual scene producers.
    const auto joint=stage->DefinePrim(SdfPath("/World/Rig/Joint"),TfToken("RigExecJoint"));
    const auto fk = stage->DefinePrim(SdfPath("/World/Rig/Fk"),TfToken("RigExecFkChain"));
    CHECK(fk.GetRelationship(TfToken("rigExec:controls")).SetTargets({control.GetPath()}));
    CHECK(fk.GetRelationship(TfToken("rigExec:joints")).SetTargets({joint.GetPath()}));
    const auto fullFile=(scratch/"full-scene.rigpack").string();
    CHECK(RigExecExportRigPack(stage,times,fullFile,"asset/full-scene",&error));
    if(!error.empty())std::printf("full-scene export: %s\n",error.c_str());
    auto full=RigExecLoadRigPack(fullFile,&error);CHECK(full);
    if(full) {
        RigExecStandaloneSystem system(*full);
        const int aggregate=system.AddTap(RigExecValueAddress::Prim(fk.GetPath(),TfToken("computePointFrameArray")));
        const int committed=system.AddTap(RigExecValueAddress::Prim(joint.GetPath(),TfToken("computePointFrame")));
        const auto pose=system.Evaluate(UsdTimeCode::Default());CHECK(pose.valid);
        const auto frames=pose.Get<RigExecPointFrameArray>(aggregate);CHECK(frames.frames.size()==1);
        if(frames.frames.size()==1)CHECK(frames.frames.front().Origin()==GfVec3d(3,0,0));
        CHECK(pose.Get<RigExecPointFrame>(committed).Origin()==GfVec3d(3,0,0));
    }
    CHECK(!RigExecExportRigPack(stage,{UsdTimeCode(std::numeric_limits<double>::infinity())},file,"asset/version1",&error));
    stage.Reset();
    CHECK(loaded->Get(SdfPath("/World/Points.points"),UsdTimeCode::Default(),&value));
    CHECK(value.IsHolding<VtVec3fArray>() && value.UncheckedGet<VtVec3fArray>() == vertices);
}

int main()
{
    PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);
    const auto scratch = std::filesystem::temp_directory_path() / ("rigexec-pack-conformance-" +
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    if (!std::filesystem::create_directory(scratch)) return 1;
    TestPack(scratch);
    if (!failures) std::filesystem::remove_all(scratch);
    if (failures) { std::printf("testRigExecPack: %d failures; fixture %s\n",failures,scratch.string().c_str()); return 1; }
    std::puts("testRigExecPack: all tests passed");
    return 0;
}
