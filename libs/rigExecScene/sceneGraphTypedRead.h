#ifndef RIGEXEC_GRAPH_SCENE_GRAPH_TYPED_READ_H
#define RIGEXEC_GRAPH_SCENE_GRAPH_TYPED_READ_H
#include "sceneGraphBinding.h"
#include "sceneTypedReads.h"
namespace rigExec {
struct RigExecGraphTypedHop {
    RigExecValueId raw=UINT64_MAX,overlay=UINT64_MAX;
    bool element=false,computed=false;
};
struct RigExecGraphTypedRead {
    std::vector<RigExecGraphTypedHop> hops,doubleHops;
    std::type_index type{typeid(void)};
    int element=-1;
    bool base=true;
    RigExecValueId effective=UINT64_MAX;
    std::string unavailable,elementUnavailable;
};
bool RigExecBindGraphTypedRead(const RigExecSceneTypedRead &,
    const RigExecSceneGraphBindingContext &,RigExecGraphTypedRead *,std::string *error=nullptr);
bool RigExecReadGraphTypedRead(const RigExecGraphTypedRead &,
    const RigExecTypedValueStore &,VtValue *,std::string *error=nullptr);
}
#endif
