#ifndef RIGEXEC_GRAPH_PROPERTY_GRAPH_BINDING_H
#define RIGEXEC_GRAPH_PROPERTY_GRAPH_BINDING_H
#include "propertySceneLowering.h"
#include "sceneGraphTypedRead.h"
namespace rigExec {
struct RigExecPropertyGraphBinding {
    RigExecPropertyRecord record;
    std::map<std::string,RigExecGraphTypedRead> inputs;
    RigExecValueId incoming=UINT64_MAX,output=UINT64_MAX,envelope=UINT64_MAX;
    std::vector<RigExecValueId> reads;
};
bool RigExecBindPropertyGraph(const RigExecScenePropertyDescriptor &,
    const RigExecSceneGraphBindingContext &,RigExecValueId incoming,RigExecValueId output,
    RigExecValueId envelope,RigExecPropertyGraphBinding *,std::string *error=nullptr);
bool RigExecRunBoundProperty(const RigExecPropertyGraphBinding &,
    RigExecTypedValueStore *,std::string *error=nullptr);
}
#endif
