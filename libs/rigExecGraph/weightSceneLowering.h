#ifndef RIGEXEC_GRAPH_WEIGHT_SCENE_LOWERING_H
#define RIGEXEC_GRAPH_WEIGHT_SCENE_LOWERING_H
#include "sceneCompileInputs.h"
#include "sceneTypedReads.h"
#include "weightProgram.h"
namespace rigExec {
struct RigExecSceneWeightPointBinding {
    SdfPath reader;
    TfToken readPhase;
    SdfPathVector targets, canonicalTargets;
    std::vector<char> targetExists;
    /// Exact raw array type; scalar/frame domain anchors are not points inputs.
    std::vector<char> pointArrays;
    /// Raw coordinates; phase routing is assigned explicit SSA IDs by caller.
    std::vector<RigExecSceneBoundInput> raw;
};
struct RigExecSceneWeightObject {
    SdfPath path, assetAnchor,paintedValues,paintedIndices;
    std::map<int,RigExecSceneBoundInput> scalars;
    std::map<int,RigExecSceneTypedRead> scalarReads;
    std::array<RigExecSceneWeightPointBinding,3> points;
    std::array<std::vector<RigExecSceneTypedRead>,3> packetReads;
    RigExecSceneBoundInput axis,bounds;
    bool axisAttribute=false,boundsAttribute=false;
    SdfPathVector baseTargets,inputTargets;
    std::vector<char> baseExists,inputExists;
};
struct RigExecSceneWeightProgram {
    int root=-1;
    std::vector<RigExecWeightRecord> records;
    std::vector<RigExecSceneWeightObject> objects;
    std::map<SdfPath,int> indices;
};
struct RigExecSceneWeightInputs {
    std::vector<RigExecWeightFieldInputs> fields;
    std::vector<std::array<VtVec3fArray,3>> rawStorage,phasedStorage;
    std::vector<VtFloatArray> paintedValues;
    std::vector<VtIntArray> paintedIndices;
    /// Ordinary packets concatenate only exact authored property targets.
    std::vector<std::array<std::vector<GfVec3f>,3>> packetPoints;
};
bool RigExecLowerSceneWeight(const RigExecSceneDescriptors &,const SdfPath &,
    RigExecSceneWeightProgram *,std::string *error=nullptr);
/// Sampling/capture boundary over detached identity rows. Current producer
/// values are keyed by consuming attribute/relationship, including empty boxes
/// for declared unavailable phases. Returned views live with output storage.
bool RigExecResolveSceneWeight(const RigExecSceneDescriptors &,
    const RigExecSceneWeightProgram &,UsdTimeCode,
    const std::map<SdfPath,VtValue> &delivered,
    const std::map<SdfPath,GfMatrix4d> &placements,
    RigExecSceneWeightInputs *,std::string *error=nullptr);
bool RigExecMakeSceneWeightPacketInputs(const RigExecSceneWeightProgram &,
    const RigExecSceneWeightInputs &,int object,
    const std::vector<RigExecWeightPacket> &currentPackets,
    const std::map<SdfPath,RigExecPointFrame> &baseFrames,
    RigExecWeightPacketInputs *,std::string *error=nullptr);
}
#endif
