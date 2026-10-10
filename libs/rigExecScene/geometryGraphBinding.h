#ifndef RIGEXEC_GRAPH_GEOMETRY_GRAPH_BINDING_H
#define RIGEXEC_GRAPH_GEOMETRY_GRAPH_BINDING_H
#include "geometrySceneLowering.h"
#include "sceneGraphBinding.h"
#include "sceneGraphTypedRead.h"
namespace rigExec {
struct RigExecGeometryGraphBinding {
    RigExecSceneGeometryDescriptor descriptor;
    std::vector<RigExecValueId> leaves,influences;
    std::vector<RigExecGraphTypedRead> typedLeaves;
    RigExecValueId transform=UINT64_MAX,transformSpace=UINT64_MAX,carry=UINT64_MAX;
    RigExecValueId weights=UINT64_MAX,driverFrames=UINT64_MAX;
    std::vector<RigExecValueId> influenceRest;
    RigExecValueId transformRest=UINT64_MAX,transformSpaceRest=UINT64_MAX,carryRest=UINT64_MAX;
    std::array<RigExecValueId,3> projectorBase{{UINT64_MAX,UINT64_MAX,UINT64_MAX}};
    std::array<RigExecValueId,3> projectorFinal{{UINT64_MAX,UINT64_MAX,UINT64_MAX}};
    std::vector<RigExecValueId> reads;
};
bool RigExecBindGeometryGraph(const RigExecSceneGeometryDescriptor &,
    const RigExecSceneGraphBindingContext &,RigExecGeometryGraphBinding *,std::string *error=nullptr);
struct RigExecGeometryGraphWorkspace {
    std::vector<VtValue> leaves;
    std::vector<GfMatrix4d> influences;
    GfMatrix4d transform{1},carry{1};
    VtValue weights,driverFrames;
    std::shared_ptr<const RigExecSkinTopology> skinTopology;
};
bool RigExecReadGeometryGraphInputs(const RigExecGeometryGraphBinding &,
    const RigExecTypedValueStore &,bool posedPoints,RigExecGeometryGraphWorkspace *,
    RigExecGeometryInputs *,std::string *error=nullptr);
bool RigExecReadGeometryGraphProjector(const RigExecGeometryGraphBinding &,
    const RigExecTypedValueStore &,RigExecGeometryGraphWorkspace *,
    RigExecSurfaceProjectorFrames *,RigExecProjectorReads *,std::string *error=nullptr);
bool RigExecReadGeometryGraphLeaves(const RigExecGeometryGraphBinding &,
    const RigExecTypedValueStore &,std::vector<VtValue> *,std::string *error=nullptr);
}
#endif
