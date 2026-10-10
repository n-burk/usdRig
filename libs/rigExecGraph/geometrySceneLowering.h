#ifndef RIGEXEC_GRAPH_GEOMETRY_SCENE_LOWERING_H
#define RIGEXEC_GRAPH_GEOMETRY_SCENE_LOWERING_H
#include "sceneCompileInputs.h"
#include "sceneTypedReads.h"
#include "geometryProgram.h"
namespace rigExec {
struct RigExecSceneGeometryBlendSample {
    SdfPath path;
    int activation=-1,points=-1,offsets=-1,indices=-1;
    bool sparse=false,shapeExists=false;
    std::vector<int> additionalPoints; ///< public sample relationship flattened in target order
};
struct RigExecSceneGeometryBlendChannel {
    int weight=-1;
    std::vector<RigExecSceneGeometryBlendSample> samples;
};
struct RigExecSceneGeometryDescriptor {
    RigExecGeometryRecord record;
    SdfPath mover,target;
    SdfValueTypeName targetType;
    bool targetExists=false;
    SdfPath pointsTarget;
    bool derived=false,matrixOutput=false;
    std::vector<RigExecSceneBoundInput> inputs;
    std::vector<char> bound;
    std::vector<RigExecSceneTypedRead> typedLeaves;
    std::vector<RigExecSceneGeometryBlendChannel> blendChannels;
};
bool RigExecLowerSceneBlendChannel(const RigExecSceneDescriptors &,const SdfPath &,
    RigExecSceneGeometryDescriptor *,std::string *error=nullptr);
bool RigExecLowerSceneGeometryVariants(const RigExecSceneDescriptors &,const SdfPath &mover,
    const SdfPath &pointsTarget,std::vector<RigExecSceneGeometryDescriptor> *,std::string *error=nullptr);
bool RigExecLowerSceneDerivedGeometry(const RigExecSceneDescriptors &,const SdfPath &pointsTarget,
    std::vector<RigExecSceneGeometryDescriptor> *,std::string *error=nullptr);
/// Shared binding facts and read-site declarations for every builtin revision.
/// normalized supplies compiler-owned derived/projector facts (not source handles).
bool RigExecLowerSceneGeometry(const RigExecSceneDescriptors &,const SdfPath &mover,
    const SdfPath &target,RigExecSceneGeometryDescriptor *,std::string *error=nullptr,
    const RigExecGeometryRecord *normalized=nullptr);
bool RigExecGatherSceneGeometryBlendChannels(const RigExecSceneGeometryDescriptor &,
    const std::vector<VtValue> &,size_t baseCount,std::vector<RigExecBlendChannel> *,
    std::string *error=nullptr,bool publicChannelOrder=false);
/// Sample only declared leaves. Typed producer values supplied by the common graph
/// take precedence according to each read site's exact flavour and time policy.
bool RigExecResolveSceneGeometryLeaves(const RigExecSceneDescriptors &,
    const RigExecSceneGeometryDescriptor &,UsdTimeCode,
    const std::map<SdfPath,VtValue> &delivered,std::vector<VtValue> *,
    std::string *error=nullptr);
}
#endif
