// Source-input protocol driver. This is not an original numerical judge.
#include "rigExec/inputReplay.h"
#include "rigExec/bakedProgram.h"
#include "rigExec/inputReplayWire.h"
#include "rigExec/frozenContext.h"
#include "rigExec/rigEvaluator.h"
#include "rigExec/moverGraph.h"
#include "pxr/base/plug/registry.h"
#include "pxr/usd/sdf/attributeSpec.h"
#include "pxr/usd/sdf/changeBlock.h"
#include "pxr/usd/sdf/layerStateDelegate.h"
#include "pxr/usd/sdf/primSpec.h"
#include "pxr/usd/sdf/types.h"
#include <cstdio>
#include <fstream>
#include <iterator>
#include <string>

using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE
namespace {
class CustomDelegate:public SdfSimpleLayerStateDelegate {
public:static SdfLayerStateDelegateBaseRefPtr New(){return TfCreateRefPtr(new CustomDelegate);}
};
UsdStageRefPtr Stage() {
    auto child=SdfLayer::CreateAnonymous("input-child.usda");
    auto source=SdfCreatePrimInLayer(child,SdfPath("/Source"));
    auto assets=SdfAttributeSpec::New(source,"assets",SdfValueTypeNames->AssetArray);
    assets->SetDefaultValue(VtValue(VtArray<SdfAssetPath>{SdfAssetPath(child->GetIdentifier())}));
    auto stage=UsdStage::CreateInMemory();stage->GetRootLayer()->SetSubLayerPaths({child->GetIdentifier()});
    auto root=stage->DefinePrim(SdfPath("/Rig"),TfToken("RigExecRoot"));
    auto joint=stage->DefinePrim(SdfPath("/Rig/J"),TfToken("RigExecJoint"));
    joint.CreateAttribute(TfToken("rest:space"),SdfValueTypeNames->Matrix4d).Set(GfMatrix4d(1));
    joint.CreateAttribute(TfToken("avars:tx"),SdfValueTypeNames->Double).Set(2.0);
    root.CreateAttribute(TfToken("literal"),SdfValueTypeNames->String).Set(child->GetIdentifier());
    root.CreateAttribute(TfToken("strings"),SdfValueTypeNames->StringArray).Set(VtStringArray{child->GetIdentifier()});
    root.SetCustomData({{"literal",VtValue(child->GetIdentifier())}});
    root.CreateAttribute(TfToken("samples"),SdfValueTypeNames->FloatArray).Set(VtFloatArray{-0.0f,1.0f});
    return stage;
}
bool Compile(RigExecRigEvaluator &rig){std::vector<std::string> errors;const bool okay=rig.Compile(&errors);for(const auto &v:errors)std::fprintf(stderr,"%s\n",v.c_str());return okay;}
int Fixture(const std::string &kind) {
    auto stage=Stage();
    if(kind=="custom")stage->GetRootLayer()->SetStateDelegate(CustomDelegate::New());
    if(kind=="during") {
        auto observer=RigExecInputReplayObserver::Create(stage,SdfPath("/Rig"));
        if(!observer)return 2;
        observer->BeginEvaluate(UsdTimeCode(1));
        stage->GetAttributeAtPath(SdfPath("/Rig/J.avars:tx")).Set(3.0);
        return 3;
    }
    RigExecRigEvaluator first(stage,SdfPath("/Rig")),second(stage,SdfPath("/Rig"));
    if(kind=="bulk") {stage->GetRootLayer()->ImportFromString("#usda 1.0\ndef Scope \"Replacement\" {}\n");return 3;}
    if(kind=="bulk-clear"){stage->GetRootLayer()->Clear();return 3;}
    if(kind=="bulk-transfer") {
        auto replacement=SdfLayer::CreateAnonymous("unsupported-transfer.usda");
        replacement->ImportFromString("#usda 1.0\ndef Scope \"Replacement\" {}\n");
        stage->GetRootLayer()->TransferContent(replacement);return 3;
    }
    if(kind=="mixed-import") {
        SdfChangeBlock block;
        stage->GetAttributeAtPath(SdfPath("/Rig/J.avars:tx")).Set(3.0);
        RigExecInputReplayImportFromString(stage->GetRootLayer(),"#usda 1.0\ndef Scope \"Replacement\" {}\n");return 3;
    }
    if(kind=="late") {
        auto layer=SdfLayer::CreateAnonymous("late-input.usda");
        SdfCreatePrimInLayer(layer,SdfPath("/Late"));
        const auto existing=stage->GetRootLayer()->GetSubLayerPaths();
        std::vector<std::string> paths(existing.begin(),existing.end());paths.push_back(layer->GetIdentifier());
        stage->GetRootLayer()->SetSubLayerPaths(paths);first.Evaluate(UsdTimeCode(1));return 3;
    }
    if(kind=="population") {
        stage->SetPopulationMask(UsdStagePopulationMask({SdfPath("/Rig")}));
        stage->SetPopulationMask(UsdStagePopulationMask::All());return 3;
    }
    if(kind=="load") {
        stage->SetLoadRules(UsdStageLoadRules::LoadNone());
        stage->SetLoadRules(UsdStageLoadRules::LoadAll());return 3;
    }
    if(kind=="mute") {
        const std::string layer=stage->GetRootLayer()->GetSubLayerPaths().front();
        stage->MuteLayer(layer);stage->UnmuteLayer(layer);return 3;
    }
    if(kind=="unclosed") {auto *leaked=new RigExecRigEvaluator(stage,SdfPath("/Rig"));(void)leaked;return 0;}
    if(!Compile(first)||!Compile(second))return 1;
    if(kind=="held-owner-close") {
        auto owner=std::make_unique<RigExecRigEvaluator>(stage,SdfPath("/Rig"));
        if(!Compile(*owner))return 1;
        auto program=RigExecInputReplayHeldProgram::Build(owner.get(),nullptr);
        if(!*program)return 1;
        owner.reset();return 3; // Capture must refuse before the dangling owner can run.
    }
    if(kind=="held"||kind=="held-unclosed"||kind=="held-router-live") {
        auto program=RigExecInputReplayHeldProgram::Build(&first,nullptr);
        if(!*program)return 1;
        if(kind=="held-unclosed"){program.release();return 0;}
        if(kind=="held-router-live") {
            RigExecInputReplayHeldProgram::Router(program->get(),true);
            program.reset();return 3;
        }
        RigExecRigPose before,held,requested;
        if(!program->Run(UsdTimeCode(1),&before,false)||!before.valid||!before.time.IsDefault())return 1;
        if(!program->Run(UsdTimeCode(1),&held,false)||!held.valid)return 1;
        if(!program->Run(UsdTimeCode(2),&requested,true)||!requested.valid||requested.time!=UsdTimeCode(2))return 1;
        return 0;
    }
    if(kind=="freeze") {
        std::shared_ptr<const RigExecFrozenProgram> program;std::string error;
        RigExecFreezeProgram(first,&program,&error);return 3;
    }
    if(kind=="comparison") {
        std::shared_ptr<const RigExecFrozenProgram> program;std::string error;
        if(!RigExecFreezeProgram(first,&program,&error))return 1;
    }
    if(kind=="nested") {
        RigExecInputReplayComparisonScope comparison("protocol nested caller");
        first.Evaluate(UsdTimeCode(1));return 3;
    }
    first.SetSolverGuidesEnabled(true);second.SetPublishWeightFields(true);
    if(!first.Evaluate(UsdTimeCode::Default()).valid)return 1;
    if(!second.Evaluate(UsdTimeCode(1)).valid)return 1;
    const SdfPath path("/Rig/J.avars:tx");
    first.SetUpstreamInputs({{path.GetPrimPath(),{},path.GetNameToken(),VtValue(5.0)},
                            {path.GetPrimPath(),{},path.GetNameToken(),VtValue(7.0)}});
    first.SetInteractiveOverrides({{path.GetPrimPath(),{},path.GetNameToken(),VtValue(-0.0)}});
    first.Evaluate(UsdTimeCode(3));first.Evaluate(UsdTimeCode(3));
    first.ClearInteractiveOverrides();first.SetUpstreamInputs({});first.Evaluate(UsdTimeCode(1));
    {
        SdfChangeBlock block;
        stage->GetAttributeAtPath(path).Set(3.0,UsdTimeCode(2));
        stage->GetAttributeAtPath(SdfPath("/Rig.samples")).Set(VtFloatArray(),UsdTimeCode(2));
        stage->GetAttributeAtPath(SdfPath("/Rig.samples")).Set(SdfValueBlock(),UsdTimeCode(3));
        stage->GetAttributeAtPath(path).Set(4.0,UsdTimeCode(2));
    }
    first.Evaluate(UsdTimeCode(2));second.Evaluate(UsdTimeCode(2));
    stage->SetEditTarget(stage->GetSessionLayer());
    stage->GetAttributeAtPath(path).Set(9.0);
    if(kind=="bulk-adapters") {
        auto saved=SdfLayer::CreateAnonymous("authored-session-before.usda");
        saved->TransferContent(stage->GetSessionLayer());
        RigExecInputReplayClearLayer(stage->GetSessionLayer());
        RigExecInputReplayTransferLayerContent(stage->GetSessionLayer(),saved);
    }
    first.Evaluate(UsdTimeCode(2));second.Evaluate(UsdTimeCode::Default());
    if(!RigExecInputReplayImportFromString(stage->GetRootLayer(),
        "#usda 1.0\ndef RigExecRoot \"Rig\" {\n def RigExecJoint \"J\" {\n  double avars:tx = 4\n }\n}\n"))return 1;
    first.Evaluate(UsdTimeCode(2));second.Evaluate(UsdTimeCode::Default());
    return 0;
}
}
int main(int argc,char **argv) {
    if(argc<3)return 2;
    if(std::string(argv[1])=="mutate") {
        if(argc!=5)return 2;
        std::ifstream input(argv[2],std::ios::binary);
        std::string bytes{std::istreambuf_iterator<char>(input),std::istreambuf_iterator<char>()};
        if(bytes.empty())return 2;
        if(std::string(argv[4])=="truncate")bytes.pop_back();
        else if(std::string(argv[4])=="extra")bytes.push_back('x');
        else if(std::string(argv[4])=="sequence") {
            inputReplay::Reader r(bytes);r.at=std::string("rigexec-input-actions-2\n").size();r.Text();r.Text();
            if(r.at>=bytes.size())return 2;bytes[r.at]=char(uint8_t(bytes[r.at])+1);
        } else if(std::string(argv[4]).find("notice")==0) {
            const std::string kind(argv[4]);
            inputReplay::Reader r(bytes);r.at=std::string("rigexec-input-actions-2\n").size();r.Text();r.Text();
            bool changed=false;
            while(!r.Done()) {
                r.U64();const auto event=inputReplay::Event(r.U8());const size_t start=r.at;const auto payload=r.Text();
                if(event!=inputReplay::Event::Batch)continue;
                inputReplay::Reader batch(payload);batch.U32();if(batch.U8()!=1)continue;
                const auto notice=batch.Text();inputReplay::Reader n(notice);
                if(n.U32()!=1)return 2;n.U32();const auto count=n.U32();if(!count)return 2;
                const size_t prefix=n.at,absolute=start+8+4+1+8;
                std::vector<std::string> entries;
                size_t pathByte=0,flagsByte=0,fieldByte=0,valueByte=0;
                for(unsigned entry=0;entry<count;++entry) {
                    const size_t begin=n.at;if(!entry)pathByte=n.at+8;n.Text();
                    if(!entry)flagsByte=n.at;n.U32();n.Text();n.Text();const auto fields=n.U32();
                    for(unsigned field=0;field<fields;++field) {
                        if(!entry&&!field)fieldByte=n.at+8;n.Text();
                        const auto old=n.Text();if(!entry&&!field&&!old.empty())valueByte=n.at-1;
                        n.Text();
                    }
                    const auto sublayers=n.U32();for(unsigned i=0;i<sublayers;++i){n.Text();n.U32();}
                    entries.push_back(notice.substr(begin,n.at-begin));
                }
                n.Finish();
                if(kind=="notice-order") {
                    if(entries.size()<2)return 2;
                    std::string reordered=notice.substr(0,prefix);
                    for(auto i=entries.rbegin();i!=entries.rend();++i)reordered+=*i;
                    bytes.replace(absolute,notice.size(),reordered);
                } else {
                    size_t target=0;uint8_t mask=1;
                    if(kind=="notice"){target=absolute+flagsByte;mask=16;}
                    else if(kind=="notice-value"){if(!valueByte)return 2;target=absolute+valueByte;}
                    else if(kind=="notice-path")target=absolute+pathByte;
                    else if(kind=="notice-field"){if(!fieldByte)return 2;target=absolute+fieldByte;}
                    else if(kind=="notice-boundary")target=start+8;
                    else return 2;
                    if(target>=bytes.size())return 2;bytes[target]=char(uint8_t(bytes[target])^mask);
                }
                changed=true;break;
            }
            if(!changed)return 2;
        } else return 2;
        std::ofstream output(argv[3],std::ios::binary);output.write(bytes.data(),std::streamsize(bytes.size()));return output?0:1;
    }
    if(std::string(argv[1])=="replay") {
        if(argc!=4)return 2;
        PlugRegistry::GetInstance().RegisterPlugins(argv[3]);RigExecLoadComputations();
        std::string error;const bool okay=RigExecReplayInputActions(argv[2],&error);
        if(!okay)std::fprintf(stderr,"%s\n",error.c_str());return okay?0:1;
    }
    if(argc!=4)return 2;
    PlugRegistry::GetInstance().RegisterPlugins(argv[3]);
    RigExecLoadComputations();
    return Fixture(argv[2]);
}
