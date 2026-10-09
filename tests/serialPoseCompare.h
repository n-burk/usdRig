// Shared by the suites that used to relaunch themselves for a serial or
// fine-cluster process. A built program keeps the clusters and the executor
// it was compiled with; these helpers change the live switches and compile
// again.
#ifndef RIGEXEC_SERIAL_POSE_COMPARE_H
#define RIGEXEC_SERIAL_POSE_COMPARE_H

#include "rigExec/parallel.h"
#include "rigExec/rigEvaluator.h"

#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/setenv.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"

#include <cstring>
#include <map>
#include <string>
#include <utility>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExecTest {

// Restores the previous value, including empty when the variable was unset.
struct EnvOverride {
    std::string name;
    std::string previous;
    EnvOverride(const char *variable, const char *value)
        : name(variable), previous(TfGetenv(variable))
    {
        TfSetenv(name, value ? value : "");
    }
    ~EnvOverride() { TfSetenv(name, previous); }
    EnvOverride(const EnvOverride &) = delete;
    EnvOverride &operator=(const EnvOverride &) = delete;
};

inline bool
StageIsBiped(const char *stage)
{
    return stage && std::strncmp(stage, "biped/", 6) == 0;
}

// The large rigs, plus one small chain so a serial compile of a short
// graph is covered beside the biped.
inline bool
StageIsSerialSample(const char *stage)
{
    return StageIsBiped(stage) ||
           (stage && std::strcmp(stage, "01_FkChainTail.usda") == 0);
}

inline SdfPath
FindRigPath(const UsdStageRefPtr &stage)
{
    if (!stage) {
        return SdfPath();
    }
    for (const UsdPrim &prim : stage->TraverseAll()) {
        if (prim.GetTypeName() == TfToken("RigExecRoot")) {
            return prim.GetPath();
        }
    }
    return SdfPath();
}

inline bool
_SameWeightFields(
    const std::map<SdfPath, rigExec::RigExecResolvedWeightField> &a,
    const std::map<SdfPath, rigExec::RigExecResolvedWeightField> &b)
{
    if (a.size() != b.size()) {
        return false;
    }
    for (const auto &entry : a) {
        const auto found = b.find(entry.first);
        if (found == b.end() || found->second.target != entry.second.target ||
            found->second.weights != entry.second.weights) {
            return false;
        }
    }
    return true;
}

// Published pose of a fresh parallel evaluator against a fresh serial one.
// Empty means they match. The runtime reader does not consult the parallel
// switch, and a bake's bytes do not change with it, so a serial evaluator
// that matches the parallel evaluator matches the runtime play the
// production pass already compared.
inline std::string
SerialPoseMatchesParallel(const UsdStageRefPtr &stage,
                          const std::vector<double> &frames)
{
    if (!stage || frames.empty()) {
        return "no stage or frames";
    }
    const SdfPath rigPath = FindRigPath(stage);
    if (rigPath.IsEmpty()) {
        return "no rig";
    }
    if (!rigExec::RigExecParallelEvaluationEnabled()) {
        return "serial compare requires parallel evaluation to start on";
    }
    std::vector<rigExec::RigExecRigPose> parallelPoses;
    parallelPoses.reserve(frames.size());
    {
        rigExec::RigExecRigEvaluator parallelEval(stage, rigPath);
        for (double frame : frames) {
            rigExec::RigExecRigPose pose =
                parallelEval.Evaluate(UsdTimeCode(frame));
            if (!pose.valid) {
                return "parallel pose invalid";
            }
            parallelPoses.push_back(std::move(pose));
        }
    }
    EnvOverride off("RIGEXEC_ENABLE_PARALLEL_EVAL", "0");
    if (rigExec::RigExecParallelEvaluationEnabled()) {
        return "parallel flag stayed on";
    }
    rigExec::RigExecRigEvaluator serialEval(stage, rigPath);
    for (size_t i = 0; i < frames.size(); ++i) {
        const rigExec::RigExecRigPose serialPose =
            serialEval.Evaluate(UsdTimeCode(frames[i]));
        if (!serialPose.valid) {
            return "serial pose invalid";
        }
        const rigExec::RigExecRigPose &want = parallelPoses[i];
        if (serialPose.jointMatricesFinal != want.jointMatricesFinal) {
            return "jointMatricesFinal";
        }
        if (serialPose.jointFramesFinal != want.jointFramesFinal) {
            return "jointFramesFinal";
        }
        if (serialPose.providerXforms != want.providerXforms) {
            return "providerXforms";
        }
        if (serialPose.providerBaseXforms != want.providerBaseXforms) {
            return "providerBaseXforms";
        }
        if (serialPose.movedProperties != want.movedProperties) {
            return "movedProperties";
        }
        if (serialPose.weightFrames != want.weightFrames) {
            return "weightFrames";
        }
        if (!_SameWeightFields(serialPose.weightFields, want.weightFields)) {
            return "weightFields";
        }
        if (serialPose.diagnostics != want.diagnostics) {
            return "diagnostics";
        }
    }
    return {};
}

}  // namespace rigExecTest

#endif
