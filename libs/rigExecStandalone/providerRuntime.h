#ifndef RIGEXEC_STANDALONE_PROVIDER_RUNTIME_H
#define RIGEXEC_STANDALONE_PROVIDER_RUNTIME_H
#include "sceneDb.h"
#include "rigExecGraph/providerProgram.h"
namespace rigExec {
/// Concrete stage-free provider execution. Full rig execution must additionally
/// lower solver, mover, weight and switch domain kernels into the same graph.
class RigExecStandaloneProviderRuntime {
public:
    bool Prepare(const RigExecSceneDb &database,const SdfPath &rigRoot,
        const std::vector<UsdTimeCode> &identities,std::string *error=nullptr);
    bool Evaluate(size_t identity,const std::map<SdfPath,VtValue> &overlays={},
        std::string *error=nullptr);
    const RigExecProviderProgram &GetProgram() const { return _program; }
    const RigExecTypedValueStore &GetValues() const { return _values; }
    const RigExecCompiledGraph &GetGraph() const { return _graph; }
    const RigExecOpExecution &GetExecution() const { return _execution; }
private:
    RigExecSceneDescriptors _scene;
    RigExecProviderProgram _program;
    RigExecTypedValueStore _values;
    RigExecCompiledGraph _graph;
    RigExecOpExecution _execution;
    RigExecOpWorkspace _workspace;
    std::vector<RigExecValueId> _changed;
    bool _prepared=false,_everRan=false;
};
}
#endif
