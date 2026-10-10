#ifndef RIGEXEC_GRAPH_SCENE_PROGRAM_LAYOUT_H
#define RIGEXEC_GRAPH_SCENE_PROGRAM_LAYOUT_H
#include "providerProgram.h"
#include "sceneGraphBinding.h"
#include <typeindex>
namespace rigExec {
struct RigExecSceneBaseSelection { RigExecValueId raw,overlay,output; };
struct RigExecSceneSampleSlot {
    SdfPath path;
    RigExecValueId raw=UINT64_MAX,rawDefault=UINT64_MAX,selectedDefault=UINT64_MAX,overlay=UINT64_MAX,providerRaw=UINT64_MAX;
    std::type_index type{typeid(void)};
    std::vector<VtValue> values;
    std::vector<char> available,blocked;
    VtValue defaultValue;
    bool defaultAvailable=false,defaultBlocked=false;
};
/// Compile-time layout and captured leaf rows. Runtime packets refer only to
/// these immutable typed slot IDs; all revisions share one producer graph.
class RigExecSceneProgramLayout {
public:
    RigExecProviderProgram providers;
    size_t initialProviderOps=0,identityCount=0;
    RigExecSceneVersionRoutes routes;
    std::vector<std::string> valueKeys;
    std::vector<RigExecSceneSampleSlot> samples;
    std::vector<RigExecSceneBaseSelection> baseSelections;
    std::vector<RigExecValueId> unavailableFrames;
    std::map<RigExecValueId,VtValue> constants;
    std::map<SdfPath,size_t> sampleIndex;
    std::map<SdfPath,RigExecValueId> weightPackets,solverAggregates;
    std::vector<RigExecOpDescriptor> descriptors;
    std::vector<RigExecValueId> leaves;
    RigExecValueId Allocate(const std::string &);
    bool RunBaseSelection(size_t,RigExecTypedValueStore *) const;
    bool Prepare(const RigExecSceneDescriptors &,std::string *error=nullptr);
    bool Resolve(const RigExecSceneDescriptors &,const RigExecSceneGraphReadRequest &,
        RigExecValueId *,std::string *error=nullptr) const;
    RigExecSceneGraphBindingContext BindingContext(const RigExecSceneDescriptors &) const;
    bool Sample(size_t identity,const std::map<SdfPath,VtValue> &overlays,
        RigExecTypedValueStore *,std::vector<RigExecValueId> *,std::string *error=nullptr) const;
};
}
#endif
