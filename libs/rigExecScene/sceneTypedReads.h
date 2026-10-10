#ifndef RIGEXEC_GRAPH_SCENE_TYPED_READS_H
#define RIGEXEC_GRAPH_SCENE_TYPED_READS_H
#include "sceneCompileInputs.h"
#include <typeindex>
namespace rigExec {
struct RigExecSceneTypedRead {
    SdfPath consumer;
    TfToken readPhase;
    SdfValueTypeName requested;
    std::type_index cppType{typeid(void)};
    std::vector<RigExecSceneBoundInput> hops,doubleHops;
    int inputElement=-1;
    SdfPathVector elementSources,computedSources;
};
/// Captures the original typed connection traversal. Float alone supports a
/// fresh Double tail; other requested types do not widen or narrow values.
bool RigExecBindSceneTypedRead(const RigExecSceneDescriptors &,const SdfPath &,
    const SdfValueTypeName &requested,RigExecSceneTypedRead *,std::string *error=nullptr);
/// Current overlays in traversal order precede deepest readable authored raw
/// facts. False means unavailable; result is cleared, never a prior-run value.
bool RigExecResolveSceneTypedRead(const RigExecSceneDescriptors &,
    const RigExecSceneTypedRead &,UsdTimeCode,const std::map<SdfPath,VtValue> &delivered,
    VtValue *,std::string *error=nullptr);
}
#endif
