#include "graphFrontend.h"
#include "sceneAccess.h"
namespace rigExec {
bool RigExecCompileStandaloneGraph(const RigExecSceneDb &database,
    const SdfPath &root, const std::vector<UsdTimeCode> &identities,
    const RigExecSceneKernelLowering &lower, RigExecCyclePolicy policy,
    RigExecSceneDescriptors *descriptors, RigExecCompiledGraph *graph,
    std::string *error) {
    if (!descriptors || !graph) {
        if (error) *error = "standalone graph compiler needs descriptor and graph outputs";
        return false;
    }
    if (!database.Validate(error)) return false;
    RigExecSceneDbAccess access(database);
    RigExecSceneDescriptors captured;
    if (!RigExecCaptureSceneDescriptors(access,root,identities,&captured,error)) return false;
    RigExecCompiledGraph compiled;
    if (!RigExecCompileSceneDescriptors(captured,lower,policy,&compiled,error)) return false;
    *descriptors = std::move(captured);
    *graph = std::move(compiled);
    return true;
}
}
