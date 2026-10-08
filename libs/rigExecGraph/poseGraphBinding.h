#ifndef RIGEXEC_GRAPH_POSE_GRAPH_BINDING_H
#define RIGEXEC_GRAPH_POSE_GRAPH_BINDING_H
#include "poseSceneLowering.h"
#include "sceneGraphTypedRead.h"
namespace rigExec {
struct RigExecBoundSpaceSwitch {
    RigExecSpaceSwitchRecord record;
    RigExecSpaceSwitchInputs inputs;
    RigExecGraphTypedRead active;
    std::vector<TfToken> labels;
    bool tokenIndex=false;
    double activeFallback=0;
    RigExecValueId avars=UINT64_MAX,posedDefault=UINT64_MAX,parentDefault=UINT64_MAX;
    RigExecValueId parentPosed=UINT64_MAX,parentExpression=UINT64_MAX;
    RigExecValueId spaceDefault=UINT64_MAX,spacePosed=UINT64_MAX;
    std::vector<RigExecValueId> sourceDefault,sourcePosed,reads;
    std::string unavailable;
};
struct RigExecBoundPoseInterpolator {
    RigExecPoseInterpolatorRecord record;
    RigExecPoseInterpolatorInputs inputs;
    RigExecGraphTypedRead enabled;
    std::vector<RigExecGraphTypedRead> numeric;
    RigExecValueId driverFinal=UINT64_MAX,driverRest=UINT64_MAX;
    RigExecValueId parentFinal=UINT64_MAX,parentRest=UINT64_MAX;
    std::vector<RigExecValueId> reads,weights,disabledWeights;
    std::vector<double> scratch;
    std::string unusableRotation,unusableTranslation,countMismatch;
};
bool RigExecBindSceneSpaceSwitch(const RigExecSceneDescriptors &,
    const RigExecSceneSpaceSwitchDescriptor &,const RigExecSceneGraphBindingContext &,
    RigExecBoundSpaceSwitch *,std::string *error=nullptr,
    RigExecValueId parentContext=UINT64_MAX,RigExecValueId spaceContext=UINT64_MAX);
bool RigExecBindScenePoseInterpolator(const RigExecScenePoseInterpolatorDescriptor &,
    const RigExecSceneGraphBindingContext &,const std::vector<RigExecValueId> &weights,
    const std::vector<RigExecValueId> &disabledWeights,RigExecBoundPoseInterpolator *,
    std::string *error=nullptr);
/// Runs only delivered current typed values; no source paths are resolved here.
bool RigExecRunBoundSpaceSwitch(RigExecBoundSpaceSwitch *,RigExecTypedValueStore *,
    RigExecValueId output,std::string *error=nullptr);
bool RigExecRunBoundPoseInterpolator(RigExecBoundPoseInterpolator *,
    RigExecTypedValueStore *,std::string *error=nullptr);
}
#endif
