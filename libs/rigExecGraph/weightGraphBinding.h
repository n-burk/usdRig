#ifndef RIGEXEC_GRAPH_WEIGHT_GRAPH_BINDING_H
#define RIGEXEC_GRAPH_WEIGHT_GRAPH_BINDING_H
#include "weightSceneLowering.h"
#include "typedValues.h"
#include "sceneGraphBinding.h"
#include <functional>
namespace rigExec {
enum class RigExecWeightValueRole { Raw,Overlay,FieldPoints,PhasedPoints,Placement,BaseFrame,Packet,RawDefault };
/// Compile-only resolver. Missing optional values use UINT64_MAX. Every
/// returned ID names a current typed slot in the central scene graph.
using RigExecWeightValueResolver=std::function<bool(const SdfPath &reader,const SdfPath &source,
    RigExecWeightValueRole,const TfToken &phase,RigExecValueId *,std::string *)>;
struct RigExecWeightGraphRead {
    std::vector<RigExecValueId> overlays,raw,doubleOverlays,doubleRaw,required,doubleRequired;
    bool doubleTail=false;
};
struct RigExecWeightGraphObject {
    std::map<int,RigExecWeightGraphRead> scalars;
    std::array<RigExecValueId,3> rawPoints{{UINT64_MAX,UINT64_MAX,UINT64_MAX}},
        phasedPoints{{UINT64_MAX,UINT64_MAX,UINT64_MAX}};
    std::array<bool,3> phaseDeclared{{false,false,false}};
    std::array<std::vector<RigExecWeightGraphRead>,3> packetPoints;
    RigExecValueId paintedValues=UINT64_MAX,paintedIndices=UINT64_MAX;
    bool paintedArraysDeclared=false;
    RigExecValueId axis=UINT64_MAX,bounds=UINT64_MAX,placement=UINT64_MAX,baseFrame=UINT64_MAX,
        packet=UINT64_MAX,basePacket=UINT64_MAX;
    std::vector<RigExecValueId> inputPackets;
    bool axisAttribute=false,boundsAttribute=false;
    bool cycleBlocked=false;
    std::vector<RigExecValueId> fieldReads,packetReads;
};
struct RigExecWeightGraphBinding {
    std::vector<RigExecWeightRecord> records;
    std::vector<RigExecWeightGraphObject> objects;
    std::vector<RigExecValueId> fieldReads; ///< complete selected root closure
    int root=-1;
};
struct RigExecWeightGraphWorkspace {
    std::vector<RigExecWeightFieldInputs> fields;
    std::vector<float> result;
    RigExecWeightFieldWorkspace field;
    std::vector<GfVec3f> localCurve;
    RigExecWeightPacketInputs packet;
    RigExecWeightPacketWorkspace packetScratch;
    RigExecWeightPacket unavailablePacket;
};
bool RigExecBindWeightGraph(const RigExecSceneWeightProgram &,const RigExecWeightValueResolver &,
    RigExecWeightGraphBinding *,std::string *error=nullptr);
bool RigExecBindWeightGraph(const RigExecSceneWeightProgram &,const RigExecSceneGraphBindingContext &,
    const SdfPath &consumer,const TfToken &placementPhase,
    RigExecWeightGraphBinding *,std::string *error=nullptr);
void RigExecBindWeightGraphCycles(const std::vector<RigExecOpDescriptor> &,
    const RigExecCompiledGraph &,RigExecWeightGraphBinding *);
void RigExecPrepareWeightGraphWorkspace(const RigExecWeightGraphBinding &,RigExecWeightGraphWorkspace *);
/// Bodies consume only compiled IDs/current typed values. A declared packet
/// with missing data becomes invalid; absence of a base remains a null base.
bool RigExecRunBoundWeightPacket(const RigExecWeightGraphBinding &,int object,
    const RigExecTypedValueStore &,RigExecWeightGraphWorkspace *,RigExecWeightPacket *,std::string *error=nullptr);
bool RigExecRunBoundWeightField(const RigExecWeightGraphBinding &,const RigExecTypedValueStore &,
    size_t count,RigExecValueId enteringPoints,RigExecWeightGraphWorkspace *,std::string *error);
/// Exact bytes, count, validity and error participate in change propagation.
bool RigExecPublishWeightPacket(RigExecTypedValueStore *,RigExecValueId,const RigExecWeightPacket &);
bool RigExecPublishWeightField(RigExecTypedValueStore *,RigExecValueId,const std::vector<float> &,
    size_t count,bool valid,const std::string &error);
}
#endif
