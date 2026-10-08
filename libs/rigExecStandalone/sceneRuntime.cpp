#include "sceneRuntime.h"
#include <algorithm>
namespace rigExec {
bool RigExecStandaloneSceneRuntime::Prepare(const RigExecSceneDb &database,
    const SdfPath &root,const std::vector<UsdTimeCode> &identities,std::string *error,const SdfPathVector &publicAttributes) {
    _prepared=false;_published.clear();_publicValues.clear();
    auto captureIdentities=identities;
    if(std::find(captureIdentities.begin(),captureIdentities.end(),UsdTimeCode::Default())==captureIdentities.end())captureIdentities.push_back(UsdTimeCode::Default());
    RigExecSceneDbAccess access(database);RigExecSceneDescriptors scene;
    if(!RigExecCaptureSceneDescriptors(access,root,captureIdentities,&scene,error))return false;
    scene.compilerTargets=publicAttributes;
    if(!RigExecLowerSceneProgram(scene,&_program,error))return false;
    _publicValues=_program.publicValues;
    if(!_runtime.Prepare(_program,error))return false;
    for(const auto &[key,chain]:_program.layout.routes.chains)
        _published.emplace(key,chain.versions.empty()?
            (key.domain==RigExecSceneValueDomain::Pose?chain.poseBase:chain.base):chain.versions.back().value);
    _prepared=true;return true;
}
bool RigExecStandaloneSceneRuntime::RefreshSamples(const RigExecSceneDb &database,
    const std::vector<UsdTimeCode> &identities,std::string *error) {
    if(!_prepared || identities.empty()){if(error)*error="missing prepared sample identities";return false;}
    RigExecSceneDbAccess source(database);
    auto samples=_program.layout.samples;
    for(auto &sample:samples) {
        const auto attribute=database.attributes.find(sample.path);
        if(attribute==database.attributes.end() || std::type_index(attribute->second.type.GetType().GetTypeid())!=sample.type) {
            if(error)*error="sample layout changed: "+sample.path.GetString();return false;
        }
        sample.values.clear();sample.available.clear();sample.blocked.clear();
        for(const auto time:identities) {
            VtValue value;sample.available.push_back(source.Resolve(sample.path,time,&value));
            sample.blocked.push_back(source.ValueBlocked(sample.path,time));sample.values.push_back(std::move(value));
        }
        sample.defaultAvailable=source.Resolve(sample.path,UsdTimeCode::Default(),&sample.defaultValue);
        sample.defaultBlocked=source.ValueBlocked(sample.path,UsdTimeCode::Default());
    }
    _program.layout.samples=std::move(samples);_program.layout.identityCount=identities.size();return true;
}
RigExecValueId RigExecStandaloneSceneRuntime::PublicValueId(const SdfPath &path) const {
    const auto found=_publicValues.find(path);return found==_publicValues.end()?UINT64_MAX:found->second;
}
bool RigExecStandaloneSceneRuntime::ReadPublic(const SdfPath &path,VtValue *out,const TfToken &phase) const {
    if(out)*out=VtValue();if(!_prepared || !out)return false;
    const auto domain=path.GetNameToken()=="points"?RigExecSceneValueDomain::Points:RigExecSceneValueDomain::Property;
    const auto chain=_program.layout.routes.chains.find({path,domain});
    if(phase=="final" && chain!=_program.layout.routes.chains.end() && !chain->second.versions.empty())
        return RigExecReadSceneGraphValue(_runtime.values,chain->second.versions.back().value,out);
    const auto found=_publicValues.find(path);
    return found!=_publicValues.end() && RigExecReadSceneGraphValue(_runtime.values,found->second,out);
}
bool RigExecStandaloneSceneRuntime::ReadRaw(const SdfPath &path,VtValue *out) const {
    if(out)*out=VtValue();if(!_prepared || !out)return false;
    const auto found=_program.layout.sampleIndex.find(path);
    return found!=_program.layout.sampleIndex.end() && RigExecReadSceneGraphValue(_runtime.values,_program.layout.samples[found->second].raw,out);
}
bool RigExecStandaloneSceneRuntime::ReadNamed(const SdfPath &path,const TfToken &computation,
    VtValue *out,bool frameAsMatrix) const {
    if(out)*out=VtValue();if(!_prepared || !out)return false;
    const auto id=_program.layout.providers.FindValue(RigExecProviderValueKey(path,computation.GetString()));
    return id!=UINT64_MAX && RigExecReadSceneGraphValue(_runtime.values,id,out,frameAsMatrix);
}
bool RigExecStandaloneSceneRuntime::ReadPhase(const SdfPath &path,RigExecSceneValueDomain domain,
    const TfToken &phase,VtValue *out,bool frameAsMatrix) const {
    if(out)*out=VtValue();if(!_prepared || !out)return false;
    const auto found=_program.layout.routes.chains.find({path,domain});if(found==_program.layout.routes.chains.end())return false;
    const auto &chain=found->second;
    const auto id=phase=="base"?(domain==RigExecSceneValueDomain::Pose?chain.poseBase:chain.base):
        (chain.versions.empty()?(domain==RigExecSceneValueDomain::Pose?chain.poseBase:chain.base):chain.versions.back().value);
    return RigExecReadSceneGraphValue(_runtime.values,id,out,frameAsMatrix);
}
bool RigExecStandaloneSceneRuntime::Evaluate(size_t identity,
    const std::map<SdfPath,VtValue> &overlays,std::string *error) {
    if(!_prepared){if(error)*error="standalone scene program is not prepared";return false;}
    return _runtime.Evaluate(_program,identity,overlays,error);
}
bool RigExecStandaloneSceneRuntime::Read(const SdfPath &path,RigExecSceneValueDomain domain,
    VtValue *out,bool frameAsMatrix) const {
    if(out)*out=VtValue();if(!_prepared || !out)return false;
    const auto found=_published.find({path,domain});
    return found!=_published.end() && RigExecReadSceneGraphValue(_runtime.values,found->second,out,frameAsMatrix);
}
}
