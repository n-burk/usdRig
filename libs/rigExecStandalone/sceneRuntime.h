#ifndef RIGEXEC_STANDALONE_SCENE_RUNTIME_H
#define RIGEXEC_STANDALONE_SCENE_RUNTIME_H
#include "sceneAccess.h"
#include "rigExecGraph/sceneProgramLowering.h"
namespace rigExec {
/// Direct SceneDb frontend for the shared production scene program. Preparation
/// captures detached facts; evaluation samples owned rows and runs typed SSA.
class RigExecStandaloneSceneRuntime {
public:
    bool Prepare(const RigExecSceneDb &,const SdfPath &rigRoot,
        const std::vector<UsdTimeCode> &,std::string *error=nullptr,const SdfPathVector &publicAttributes={});
    bool RefreshSamples(const RigExecSceneDb &,const std::vector<UsdTimeCode> &,std::string *error=nullptr);
    bool ReadPublic(const SdfPath &,VtValue *,const TfToken &phase=TfToken("final")) const;
    RigExecValueId PublicValueId(const SdfPath &) const;
    bool ReadRaw(const SdfPath &,VtValue *) const;
    bool ReadNamed(const SdfPath &,const TfToken &,VtValue *,bool frameAsMatrix=false) const;
    bool ReadPhase(const SdfPath &,RigExecSceneValueDomain,const TfToken &,VtValue *,bool frameAsMatrix=false) const;
    bool Evaluate(size_t identity,const std::map<SdfPath,VtValue> &overlays={},
        std::string *error=nullptr);
    bool Read(const SdfPath &,RigExecSceneValueDomain,VtValue *,
        bool frameAsMatrix=false) const;
    const RigExecSceneProgram &GetProgram() const{return _program;}
    const RigExecSceneProgramRuntime &GetRuntime() const{return _runtime;}
private:
    RigExecSceneProgram _program;
    RigExecSceneProgramRuntime _runtime;
    std::map<RigExecSceneVersionKey,RigExecValueId> _published;
    std::map<SdfPath,RigExecValueId> _publicValues;
    bool _prepared=false;
};
}
#endif
