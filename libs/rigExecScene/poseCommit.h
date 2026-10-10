#ifndef RIGEXEC_GRAPH_POSE_COMMIT_H
#define RIGEXEC_GRAPH_POSE_COMMIT_H
#include "poseProgram.h"
#include "typedValues.h"
namespace rigExec {
struct RigExecPoseCommitCandidate {RigExecValueId before=UINT64_MAX,candidate=UINT64_MAX,output=UINT64_MAX;};
struct RigExecPoseCommitDescendant {RigExecValueId current=UINT64_MAX,output=UINT64_MAX;size_t candidate=0;bool parentBlocked=false,solverOutput=false;};
struct RigExecPoseCommitBinding {
    std::vector<RigExecPoseCommitCandidate> candidates;
    std::vector<RigExecPoseCommitDescendant> descendants;
    std::vector<RigExecValueId> reads,writes;
};
struct RigExecPoseCommitWorkspace {std::vector<GfMatrix4d> deltas;std::vector<char> deltaOk;std::vector<RigExecPointFrame> staged;};
/// Stages the whole commit before publishing any changed subtree. Numerical
/// guards and carry order are the same shared production pose primitives.
bool RigExecRunPoseCommit(const RigExecPoseCommitBinding &,RigExecTypedValueStore *,
    RigExecPoseCommitWorkspace *,std::string *error=nullptr);
}
#endif
