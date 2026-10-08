#ifndef RIGEXEC_GRAPH_SOLVER_SCENE_LOWERING_H
#define RIGEXEC_GRAPH_SOLVER_SCENE_LOWERING_H
#include "sceneCompileInputs.h"
#include <functional>
#include "sceneTypedReads.h"
#include "solverProgram.h"
namespace rigExec {
struct RigExecSceneSolverFrameBinding {
    TfToken phase;
    std::map<SdfPath,SdfPath> keys;
};
struct RigExecSceneSolverDescriptor {
    SdfPath path;
    RigExecSolverRecord record;
    SdfPathVector controls, joints;
    std::vector<int> jointElements;
    SdfPath start,root,mid,end,pole,space,blendA,blendB,ribbonPoints;
    TfToken ribbonReadPhase;
    RigExecSolverRest spaceRest{};
    /// Typed input graph bindings. Keys are the exact schema property names.
    std::map<std::string,RigExecSceneBoundInput> inputs;
    std::map<std::string,RigExecSceneTypedRead> typedReads;
    std::map<std::string,RigExecSceneSolverFrameBinding> frameBindings;
};
/// Lower structural solver facts using already compiled provider rest values.
/// The caller binds frame versions and scalar producer IDs from these paths.
/// Missing required providers yield the production empty aggregate guard.
bool RigExecLowerSceneSolver(const RigExecSceneDescriptors &,const SdfPath &,
    const std::map<SdfPath,RigExecPointFrame> &providerRests,
    RigExecSceneSolverDescriptor *,std::string *error=nullptr);
/// Resolve one solver's current typed graph values without source access.
/// deliveredInputs is keyed by the consumer property: it holds the version
/// selected by that consumer's readPhase, including computed expressions.
/// frames uses relationship.AppendTarget(provider) keys for phased bindings;
/// base-only reads may use provider path keys.
bool RigExecResolveSceneSolverInputs(const RigExecSceneDescriptors &,
    const RigExecSceneSolverDescriptor &,UsdTimeCode,
    const std::map<SdfPath,RigExecPointFrame> &frames,
    const std::map<SdfPath,RigExecPointFrameArray> &aggregates,
    const std::map<SdfPath,VtValue> &deliveredInputs,
    RigExecSolverInputs *,std::string *error=nullptr);

/// Shared parameter refresh from delivered boxed current graph values.
bool RigExecRefreshSolverParameters(RigExecSolverKind,
    const std::function<bool(const char *,VtValue *)> &,RigExecSolverInputs *,
    std::string *error=nullptr);

}
#endif
