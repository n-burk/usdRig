#ifndef RIGEXEC_STANDALONE_GRAPH_FRONTEND_H
#define RIGEXEC_STANDALONE_GRAPH_FRONTEND_H
#include "sceneDb.h"
#include "rigExecGraph/sceneDescriptors.h"
namespace rigExec {
/// Captures directly from compact source rows into the shared compiler input.
/// Production lowering must bind typed kernels before this graph can execute.
bool RigExecCompileStandaloneGraph(const RigExecSceneDb &database,
    const SdfPath &rigRoot, const std::vector<UsdTimeCode> &identities,
    const RigExecSceneKernelLowering &lower, RigExecCyclePolicy policy,
    RigExecSceneDescriptors *descriptors, RigExecCompiledGraph *graph,
    std::string *error = nullptr);
}
#endif
