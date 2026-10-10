#ifndef RIGEXEC_GRAPH_SCENE_VERSION_ROUTES_H
#define RIGEXEC_GRAPH_SCENE_VERSION_ROUTES_H
#include "sceneDescriptors.h"
namespace rigExec {
enum class RigExecSceneValueDomain { Property,Points,Pose,ProviderSpace,Weight,SolverAggregate };
struct RigExecSceneVersionKey {
    SdfPath path;
    RigExecSceneValueDomain domain;
    bool operator<(const RigExecSceneVersionKey &other) const {
        return path<other.path || (!(other.path<path) && domain<other.domain);
    }
};
struct RigExecSceneValueVersion {
    RigExecValueId value=UINT64_MAX;
    SdfPath writer;
    int ordinal=-1;
};
struct RigExecSceneValueChain {
    RigExecValueId base=UINT64_MAX;
    /// Pose base is the solver-produced frame; other domains use base.
    RigExecValueId poseBase=UINT64_MAX;
    std::vector<RigExecSceneValueVersion> versions;
};
/// Immutable after compilation. Every consumer receives one explicit typed
/// SSA value ID; execution never walks paths or chooses a chain version.
class RigExecSceneVersionRoutes {
public:
    std::map<RigExecSceneVersionKey,RigExecSceneValueChain> chains;
    bool RegisterBase(const RigExecSceneVersionKey &,RigExecValueId,std::string *error=nullptr);
    bool Append(const RigExecSceneVersionKey &,const RigExecSceneValueVersion &,std::string *error=nullptr);
    bool Resolve(const RigExecSceneDescriptors &,const RigExecSceneVersionKey &,
        const SdfPath &reader,const TfToken &phase,RigExecValueId *,std::string *error=nullptr) const;
};
}
#endif
