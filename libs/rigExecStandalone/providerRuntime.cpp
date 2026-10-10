#include "providerRuntime.h"
#include "sceneAccess.h"
namespace rigExec {
bool RigExecStandaloneProviderRuntime::Prepare(const RigExecSceneDb &database,
    const SdfPath &root,const std::vector<UsdTimeCode> &identities,std::string *error) {
    _prepared=false;_everRan=false;
    if(!database.Validate(error) || !database.ValidateCapabilities(error))return false;
    const RigExecSceneDbAccess source(database);
    RigExecSceneDescriptors scene;
    if(!RigExecCaptureSceneDescriptors(source,root,identities,&scene,error))return false;
    std::set<SdfPath> providers;
    for(const auto &[path,node]:scene.nodes) {
        if(!node.fact.active || !path.HasPrefix(root))continue;
        if(node.domain==RigExecSceneDomain::Provider)providers.insert(path);
        else if(node.domain!=RigExecSceneDomain::Data) {
            if(error)*error="standalone provider graph needs production domain lowering: "+path.GetString();
            return false;
        }
    }
    if(!scene.applications.empty() || !scene.jointBindings.empty()) {
        if(error)*error="standalone provider graph needs mover/solver revision lowering";
        return false;
    }
    RigExecProviderProgram program;
    if(!RigExecBuildProviderProgram(scene,true,&program,error,&providers))return false;
    RigExecCompiledGraph graph;
    if(!RigExecCompileOpGraph(program.descriptors,program.leaves,RigExecCyclePolicy::Reject,&graph,error))return false;
    _scene=std::move(scene);_program=std::move(program);_graph=std::move(graph);
    _values=RigExecTypedValueStore(_program.valueKeys.size());
    _changed.reserve(_program.leaves.size());
    _execution.candidates.resize(_graph.ops.size());_execution.ran.resize(_graph.ops.size());
    _execution.completion.resize(_graph.ops.size());
    _workspace.seeded.resize(_graph.ops.size());_workspace.pending.resize(_graph.ops.size());
    _workspace.ready.reserve(_graph.ops.size());_workspace.unresolved.resize(_graph.ops.size());
    _prepared=true;return true;
}
bool RigExecStandaloneProviderRuntime::Evaluate(size_t identity,
    const std::map<SdfPath,VtValue> &overlays,std::string *error) {
    if(!_prepared) { if(error)*error="standalone provider graph is not prepared";return false; }
    _values.ResetChanges();
    if(!RigExecSampleProviderProgram(_program,_scene,identity,overlays,&_values,&_changed,error))return false;
    RigExecOpCallbacks callbacks;
    callbacks.run=[&](uint32_t op){return RigExecRunProviderOp(_program,_graph.ops.at(op).originalIndex,&_values,error);};
    callbacks.skip=[](uint32_t){};
    callbacks.changed=[&](RigExecValueId value){return _values.values.at(size_t(value)).changed;};
    const bool valid=RigExecExecuteOpGraph(_graph,_changed,{},!_everRan,callbacks,&_execution,error,&_workspace);
    _everRan=valid;
    return valid;
}
}
