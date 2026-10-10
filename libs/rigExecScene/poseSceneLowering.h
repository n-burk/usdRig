#ifndef RIGEXEC_GRAPH_POSE_SCENE_LOWERING_H
#define RIGEXEC_GRAPH_POSE_SCENE_LOWERING_H
#include "poseProgram.h"
#include "sceneCompileInputs.h"
#include "sceneTypedReads.h"
namespace rigExec {
struct RigExecSceneSpaceSwitchDescriptor {
    SdfPath path,target,space,activeAttribute;
    SdfPathVector sources;
    std::vector<TfToken> labels;
    bool tokenIndex=false;
    double activeFallback=0;
    RigExecSceneTypedRead active;
    RigExecSpaceSwitchRecord record;
};
struct RigExecScenePoseInterpolatorDescriptor {
    SdfPath path,driver,parent;
    SdfPathVector driverAttributes,poseWeights,disabledPoseWeights;
    RigExecSceneTypedRead enabled;
    std::vector<RigExecSceneTypedRead> numericReads;
    RigExecPoseInterpolatorRecord record;
    std::vector<std::string> notes;
};
bool RigExecLowerSceneSpaceSwitch(const RigExecSceneDescriptors &,const SdfPath &,
    RigExecSceneSpaceSwitchDescriptor *,std::string *error=nullptr);
bool RigExecResolveSceneSpaceSwitch(const RigExecSceneDescriptors &,
    const RigExecSceneSpaceSwitchDescriptor &,UsdTimeCode,
    const std::map<SdfPath,VtValue> &delivered,double *active,std::string *error=nullptr);
bool RigExecLowerScenePoseInterpolator(const RigExecSceneDescriptors &,const SdfPath &,
    RigExecScenePoseInterpolatorDescriptor *,std::string *error=nullptr);
bool RigExecResolveScenePoseInterpolator(const RigExecSceneDescriptors &,
    const RigExecScenePoseInterpolatorDescriptor &,UsdTimeCode,
    const std::map<SdfPath,VtValue> &delivered,RigExecPoseInterpolatorInputs *,
    std::string *error=nullptr);
}
#endif
