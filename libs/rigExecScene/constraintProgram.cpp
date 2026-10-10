#include "constraintProgram.h"
#include <cmath>
namespace rigExec {
bool RigExecConstraintFrameUsable(const RigExecPointFrame &frame) {
    if(!frame.IsValid() || frame.IsDegenerate())return false;
    for(const auto &point:frame.points)for(size_t i=0;i<3;++i)if(!std::isfinite(point[i]))return false;
    return true;
}
bool RigExecRunConstraint(const RigExecConstraintRecord &record,
    const RigExecConstraintInputs &input,RigExecConstraintResult *output,std::string *error) {
    if(!output) { if(error)*error="null constraint output";return false; }
    output->chain.clear();
    output->frame=input.incoming;
    switch(record.kind) {
    case RigExecConstraintKind::Position:
        output->frame=RigExecApplyPositionConstraint(input.incoming,input.sources,record.position);return true;
    case RigExecConstraintKind::Rotation: {
        auto params=record.rotation;
        params.carry=input.carry?&*input.carry:nullptr;
        output->frame=RigExecApplyRotationConstraint(input.incoming,input.sources,params);return true;
    }
    case RigExecConstraintKind::Scale:
        output->frame=RigExecApplyScaleConstraint(input.incoming,input.sources,record.scale);return true;
    case RigExecConstraintKind::Parent: {
        auto params=record.parent;
        params.carry=input.carry?&*input.carry:nullptr;
        output->frame=RigExecApplyParentConstraint(input.incoming,input.sources,params);return true;
    }
    case RigExecConstraintKind::Aim:
        output->frame=RigExecApplyAimConstraint(input.incoming,input.aimTarget,record.aim);return true;
    case RigExecConstraintKind::SingleChainIk:
        output->chain=RigExecSolveSingleChainIk(input.chain,input.effector,record.singleChain);return true;
    }
    if(error)*error="unknown constraint kind";
    return false;
}
}
