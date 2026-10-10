#ifndef RIGEXEC_GRAPH_CONSTRAINT_PROGRAM_H
#define RIGEXEC_GRAPH_CONSTRAINT_PROGRAM_H
#include "rigExecMath/solvers.h"
#include "rigExecMath/singleChainIk.h"
namespace rigExec {
enum class RigExecConstraintKind { Position,Rotation,Scale,Parent,Aim,SingleChainIk };
/// Detached constraint parameters. Carry pointers in parameter structs must
/// remain null; the current graph input owns the explicitly named carry.
struct RigExecConstraintRecord {
    RigExecConstraintKind kind=RigExecConstraintKind::Position;
    RigExecPositionConstraintParams position;
    RigExecRotationConstraintParams rotation;
    RigExecScaleConstraintParams scale;
    RigExecParentConstraintParams parent;
    RigExecAimConstraintParams aim;
    RigExecSingleChainIkParams singleChain;
};
struct RigExecConstraintInputs {
    RigExecPointFrame incoming;
    std::vector<RigExecConstraintSource> sources;
    std::optional<GfMatrix4d> carry;
    GfVec3d aimTarget{0,0,0};
    std::vector<RigExecPointFrame> chain;
    RigExecPointFrame effector;
    std::optional<RigExecPointFrame> worldUp;
    std::vector<RigExecPointFrame> poleObjects;
};
struct RigExecConstraintResult {
    RigExecPointFrame frame;
    std::vector<RigExecPointFrame> chain;
};
/// Dispatches existing production kernels without stage/schema/query handles.
/// Domain binding/defaulting precedes this call; kernel guards are preserved.
bool RigExecConstraintFrameUsable(const RigExecPointFrame &);
bool RigExecRunConstraint(const RigExecConstraintRecord &,
    const RigExecConstraintInputs &,RigExecConstraintResult *,std::string *error=nullptr);
}
#endif
