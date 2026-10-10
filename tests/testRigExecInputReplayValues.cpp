// Input-only codec proofs: expected bytes originate in authored input values.
#include "rigExec/inputReplayValues.h"
#include "rigExec/inputReplay.h"
#include "rigExec/inputReplayWire.h"
#include "rigExec/rigEvaluator.h"
#include "pxr/base/tf/setenv.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/attribute.h"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <cstdlib>
#include <algorithm>
#include <vector>
#include "pxr/base/gf/half.h"
#include "pxr/base/gf/range3d.h"
#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/ts/spline.h"
#include "pxr/base/ts/knot.h"
#include "pxr/base/ts/raii.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/vt/dictionary.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/sdf/listOp.h"
#include "pxr/usd/sdf/reference.h"
#include "pxr/usd/sdf/assetPath.h"
#include <cstdio>
#include <cstring>
#include <cstdint>
PXR_NAMESPACE_USING_DIRECTIVE
using namespace rigExec;
namespace {
int failures=0;
#define CHECK(x) do {if(!(x)){++failures;std::printf("FAIL %d: %s\n",__LINE__,#x);}}while(0)
void Check(const VtValue& input) {
    std::string bytes,error,again;VtValue decoded;
    CHECK(RigExecEncodeInputValue(input,&bytes,&error));
    CHECK(RigExecDecodeInputValue(bytes,&decoded,&error));
    CHECK(input.GetTypeid()==decoded.GetTypeid());
    CHECK(RigExecEncodeInputValue(decoded,&again,&error));
    CHECK(bytes==again);
    // Every truncated prefix fails and preserves the caller's previous value.
    for(size_t n=0;n<bytes.size();++n){VtValue sentinel(std::string("sentinel"));CHECK(!RigExecDecodeInputValue(bytes.substr(0,n),&sentinel,&error));CHECK(sentinel==VtValue(std::string("sentinel")));}
    VtValue sentinel(27);CHECK(!RigExecDecodeInputValue(bytes+"x",&sentinel,&error));CHECK(sentinel==VtValue(27));
}
bool SetCaptureEnvironment(const char *name, const std::string &value) {
#ifdef _WIN32
    // USD changes the Windows environment; the recorder reads the CRT table.
    if (_putenv_s(name, value.c_str()) != 0) return false;
#endif
    return TfSetenv(name, value);
}
bool UnsetCaptureEnvironment(const char *name) {
#ifdef _WIN32
    if (_putenv_s(name, "") != 0) return false;
#endif
    return TfUnsetenv(name);
}
void CheckSourceLifetimes() {
    namespace fs=std::filesystem;
    using namespace rigExec::inputReplay;
    const char *existing=std::getenv("RIGEXEC_INPUT_REPLAY");
    CHECK(!existing||!*existing);
    if(existing&&*existing)return; // An external capture must not be replaced.
    const char *old=std::getenv("RIGEXEC_INPUT_REPLAY_PROVENANCE");
    const bool hadProvenance=old!=nullptr;const std::string oldProvenance=old?old:"";
    const auto dir=fs::temp_directory_path()/("rigexec-source-lifetime-"+
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    CHECK(fs::create_directory(dir));
    const auto source=dir/"source.usda",actions=dir/"source.actions",provenance=dir/"provenance.txt";
    {std::ofstream out(provenance);out<<"test-owned source lifetime inputs\n";CHECK(bool(out));}
    CHECK(SetCaptureEnvironment("RIGEXEC_INPUT_REPLAY_PROVENANCE",provenance.string()));
    CHECK(SetCaptureEnvironment("RIGEXEC_INPUT_REPLAY","capture:"+actions.string()));
    const SdfPath rig("/Rig"),valuePath("/Rig.value");
    auto stage=UsdStage::CreateNew(source.string());CHECK(stage);if(!stage)return;
    const bool initialArrayAdmission=RigExecUpstreamArrayAdmission();
    // A global caller control before the first owner must still enter the log.
    RigExecSetUpstreamArrayAdmissionForTesting(false);
    CHECK(!RigExecUpstreamArrayAdmission());
    const auto value=stage->DefinePrim(rig,TfToken("Scope")).CreateAttribute(
        TfToken("value"),SdfValueTypeNames->Double);
    CHECK(value.Set(1.0));CHECK(stage->GetRootLayer()->Save());
    SdfLayerRefPtr heldLayer=stage->GetRootLayer();SdfLayerHandle weakLayer(heldLayer);
    UsdStageWeakPtr weakStage(stage);
    auto observer=RigExecInputReplayObserver::Create(stage,rig);CHECK(observer);
    RigExecSetUpstreamArrayAdmissionForTesting(true);CHECK(RigExecUpstreamArrayAdmission());
    CHECK(value.Set(2.0));observer.reset();stage.Reset();
    CHECK(!weakStage);CHECK(weakLayer);
    // The caller still owns this layer: reopening must retain the authored2.
    stage=UsdStage::Open(heldLayer);CHECK(stage);if(!stage)return;
    double actual=0;CHECK(stage->GetAttributeAtPath(valuePath).Get(&actual));CHECK(actual==2.0);
    observer=RigExecInputReplayObserver::Create(stage,rig);CHECK(observer);
    weakStage=stage;observer.reset();stage.Reset();CHECK(!weakStage);
    heldLayer.Reset();CHECK(!weakLayer);
    // Actual final release loses memory-only edits; a new file identity loads1.
    stage=UsdStage::Open(source.string());CHECK(stage);if(!stage)return;
    CHECK(stage->GetAttributeAtPath(valuePath).Get(&actual));CHECK(actual==1.0);
    observer=RigExecInputReplayObserver::Create(stage,rig);CHECK(observer);
    weakStage=stage;weakLayer=stage->GetRootLayer();observer.reset();stage.Reset();
    CHECK(!weakStage);CHECK(!weakLayer);
    RigExecSetUpstreamArrayAdmissionForTesting(initialArrayAdmission);
    RigExecFinalizeInputReplay();CHECK(UnsetCaptureEnvironment("RIGEXEC_INPUT_REPLAY"));
    if(hadProvenance)CHECK(SetCaptureEnvironment("RIGEXEC_INPUT_REPLAY_PROVENANCE",oldProvenance));
    else CHECK(UnsetCaptureEnvironment("RIGEXEC_INPUT_REPLAY_PROVENANCE"));
    RigExecSetUpstreamArrayAdmissionForTesting(!initialArrayAdmission);
    std::string error;CHECK(RigExecReplayInputActions(actions.string(),&error));CHECK(error.empty());
    CHECK(RigExecUpstreamArrayAdmission()==!initialArrayAdmission);
    RigExecSetUpstreamArrayAdmissionForTesting(initialArrayAdmission);
    const auto read=[](const fs::path &path){std::ifstream in(path,std::ios::binary);
        return std::string(std::istreambuf_iterator<char>(in),std::istreambuf_iterator<char>());};
    const auto bytes=read(actions);const std::string magic="rigexec-input-actions-5\n";
    CHECK(bytes.rfind(magic,0)==0);if(bytes.rfind(magic,0)!=0)return;
    const std::string body=bytes.substr(magic.size());Reader input(body);
    const auto provenancePath=input.Text(),provenanceBytes=input.Text();
    struct Row {Event event;std::string payload;};std::vector<Row> rows;
    while(!input.Done()){input.U64();const auto event=Event(input.U8());rows.push_back({event,input.Text()});}
    CHECK(!rows.empty()&&rows.front().event==Event::ArrayAdmission);
    std::vector<unsigned> arrayStates;
    for(const auto &row:rows)if(row.event==Event::ArrayAdmission){Reader state(row.payload);arrayStates.push_back(state.U8());state.Finish();}
    CHECK((arrayStates==std::vector<unsigned>{unsigned(initialArrayAdmission),0,1,unsigned(initialArrayAdmission)}));
    const auto stageRetire=std::find_if(rows.begin(),rows.end(),[](const auto &row){return row.event==Event::StageRetire;});
    const auto layerRetire=std::find_if(rows.begin(),rows.end(),[](const auto &row){return row.event==Event::LayerRetire;});
    CHECK(stageRetire!=rows.end());CHECK(layerRetire!=rows.end());
    const auto malformed=[&](const char *name,std::vector<Row> changed,const char *expected){
        Writer out;out.Text(provenancePath);out.Text(provenanceBytes);
        for(size_t i=0;i<changed.size();++i){out.U64(i);out.U8(uint8_t(changed[i].event));
            if(changed[i].event==Event::End){Writer count;count.U64(i);changed[i].payload.replace(0,8,count.bytes);}
            out.Text(changed[i].payload);}
        const auto path=dir/name;{std::ofstream file(path,std::ios::binary);const auto data=magic+out.bytes;file.write(data.data(),std::streamsize(data.size()));CHECK(bool(file));}
        const bool admissionBefore=RigExecUpstreamArrayAdmission();
        error.clear();CHECK(!RigExecReplayInputActions(path.string(),&error));CHECK(RigExecUpstreamArrayAdmission()==admissionBefore);CHECK(!error.empty());if(expected&&*expected)CHECK(error.find(expected)!=std::string::npos);
    };
    if(!rows.empty()){
        auto changed=rows;changed.front().payload=std::string(1,char(2));
        malformed("invalid-array-state.actions",std::move(changed),"invalid array admission state");
        changed=rows;
        while(!changed.empty()&&changed.front().event==Event::ArrayAdmission)
            changed.erase(changed.begin());
        CHECK(!changed.empty());
        malformed("missing-array-state.actions",std::move(changed),"missing initial array admission state");
    }
    if(stageRetire!=rows.end()){
        auto changed=rows;const size_t i=size_t(stageRetire-rows.begin());
        changed.insert(changed.begin()+i+1,*stageRetire);
        malformed("duplicate.actions",std::move(changed),"duplicate source stage retirement");
        const auto create=std::find_if(rows.begin(),rows.end(),[](const auto &row){return row.event==Event::Create;});
        CHECK(create!=rows.end());if(create!=rows.end()){
            Reader payload(create->payload);Writer id;id.U32(payload.U32());changed=rows;
            changed.insert(changed.begin()+size_t(create-rows.begin())+1,{Event::StageRetire,id.bytes});
            malformed("live-owner.actions",std::move(changed),"live evaluator");}
    }
    if(layerRetire!=rows.end()){
        const auto operation=std::find_if(rows.begin(),rows.end(),[](const auto &row){return row.event==Event::Operation;});
        CHECK(operation!=rows.end());if(operation!=rows.end()){
            auto changed=rows;changed.insert(changed.begin()+size_t(operation-rows.begin())+1,*layerRetire);
            malformed("unfinished-batch.actions",std::move(changed),"unclosed source notice batch");}
        auto changed=rows;Writer invalid;invalid.U32(UINT32_MAX);
        changed[size_t(layerRetire-rows.begin())].payload=invalid.bytes;
        malformed("invalid-layer.actions",std::move(changed),"");
    }
    fs::remove_all(dir);
}
}
int main() {
    uint32_t bits=0x7fc01234;float nan;std::memcpy(&nan,&bits,4);
    uint64_t dbits=0x7ff8000000004321ULL;double dnan;std::memcpy(&dnan,&dbits,8);
    GfHalf half;half.setBits(0xfe55);
    Check(VtValue(-0.0f));Check(VtValue(nan));Check(VtValue(dnan));Check(VtValue(half));
    Check(VtValue(VtFloatArray{-0.0f,nan,1.0f}));
    VtDictionary nested;nested["empty"]=VtValue();nested["block"]=VtValue(SdfValueBlock());nested["emptyArray"]=VtValue(VtVec3fArray());nested["signedZero"]=VtValue(-0.0);nested["nan"]=VtValue(nan);
    VtDictionary root;root["nested"]=VtValue(nested);root["asset"]=VtValue(SdfAssetPath("../input.usda","D:/observed/input.usda"));Check(VtValue(root));
    Check(VtValue(VtBoolArray{true,false}));Check(VtValue(GfVec3f(-0.0f,nan,2.0f)));
    GfMatrix4d matrix(1);matrix[0][1]=-0.0;matrix[3][2]=dnan;Check(VtValue(matrix));
    SdfReferenceListOp refs;CHECK(refs.SetPrependedItems({SdfReference("a.usda",SdfPath("/Root"),SdfLayerOffset(-0.0,2.0),nested)}));Check(VtValue(refs));
    SdfPathListOp explicitEmpty;CHECK(explicitEmpty.SetExplicitItems({}));Check(VtValue(explicitEmpty));Check(VtValue(SdfPathListOp()));
    SdfTimeSampleMap times;times[1.0]=VtValue(-0.0f);times[2.0]=VtValue(SdfValueBlock());times[3.0]=VtValue(VtFloatArray());Check(VtValue(times));
    SdfVariantSelectionMap variants;variants["look"]="red";Check(VtValue(variants));
    TsEditBehaviorBlock noAutomaticEdits;TsAntiRegressionAuthoringSelector noRegression(TsAntiRegressionNone);
    TsSpline spline(TfType::Find<double>());TsKnot a(TfType::Find<double>()),b(TfType::Find<double>());
    CHECK(a.SetTime(1));CHECK(a.SetValue(-0.0));CHECK(a.SetPreValue(2.0));CHECK(a.SetNextInterpolation(TsInterpCurve));CHECK(a.SetPostTanWidth(.25));CHECK(a.SetPostTanSlope(-0.0));CHECK(a.SetCustomData(nested));
    CHECK(b.SetTime(3));CHECK(b.SetValue(4.0));CHECK(spline.SetKnot(a));CHECK(spline.SetKnot(b));
    TsExtrapolation extrap;extrap.mode=TsExtrapSloped;extrap.slope=-0.0;spline.SetPreExtrapolation(extrap);Check(VtValue(spline));
    // Unsupported data must fail by type, never use formatted numbers.
    std::string bytes="sentinel",error;CHECK(!RigExecEncodeInputValue(VtValue(GfRange3d()),&bytes,&error));CHECK(bytes=="sentinel");CHECK(error.find("unsupported input value type")!=std::string::npos);
    CheckSourceLifetimes();
    std::printf("%d input replay codec failure(s)\n",failures);return failures?1:0;
}
