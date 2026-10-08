#ifndef RIGEXEC_GRAPH_SCENE_GRAPH_BINDING_H
#define RIGEXEC_GRAPH_SCENE_GRAPH_BINDING_H
#include "sceneVersionRoutes.h"
#include "typedValues.h"
namespace rigExec {
struct RigExecSceneGraphReadRequest {
    SdfPath consumer,source,reader;
    TfToken phase;
    std::string computation;
    bool *liveRest=nullptr;
    RigExecSceneValueDomain domain=RigExecSceneValueDomain::Property;
    bool atDefault=false,raw=false;
};
/// Compile-only linker. Domain adapters resolve every read once and retain only
/// typed value IDs in their runtime packets, never callbacks or source paths.
struct RigExecGraphTypedRead;
struct RigExecSceneGraphBindingContext {
    std::function<bool(const RigExecGraphTypedRead &,RigExecValueId *,std::string *)> effectiveRead;
    std::function<bool(const RigExecSceneGraphReadRequest &,RigExecValueId *,std::string *)> resolve;
    std::function<bool(const SdfPath &,RigExecValueId *,std::string *)> weightPacket;
};
/// Exact boxed transport from a selected current graph value. Frames cross a
/// matrix boundary only on explicit request; unavailable values clear output.
bool RigExecReadSceneGraphValue(const RigExecTypedValueStore &,RigExecValueId,
    VtValue *,bool frameAsMatrix=false);
}
#endif
