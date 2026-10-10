#ifndef RIGEXEC_TEST_INDEPENDENT_CONSTRAINT_REFERENCE_H
#define RIGEXEC_TEST_INDEPENDENT_CONSTRAINT_REFERENCE_H
#include "rigExecMath/solvers.h"
#include <cmath>

namespace rigExec {
// Input-only packet populated by the retained ORIGINAL stage/context resolver.
// Never populate these fields from ConstraintProgram output or geometry deltas.
struct RigExecIndependentConstraintInputs {
    enum class Kind { Aim, Rotation, Parent } kind = Kind::Aim;
    RigExecPointFrame entering;
    std::vector<RigExecConstraintSource> sources;
    RigExecAimConstraintParams aim;
    RigExecRotationConstraintParams rotation;
    RigExecParentConstraintParams parent;
};

// This helper is deliberately only numerical dispatch. Original enable,
// source-array admission, world-up resolution and commit policy remain the
// independently captured reference callback's responsibility.
inline bool RigExecIndependentConstraintCandidate(
    const RigExecIndependentConstraintInputs &input,
    RigExecPointFrame *candidate)
{
    if (!candidate) return false;
    *candidate = input.entering;
    switch (input.kind) {
    case RigExecIndependentConstraintInputs::Kind::Rotation:
        *candidate = RigExecApplyRotationConstraint(input.entering,input.sources,input.rotation);
        return true;
    case RigExecIndependentConstraintInputs::Kind::Parent:
        *candidate = RigExecApplyParentConstraint(input.entering,input.sources,input.parent);
        return true;
    case RigExecIndependentConstraintInputs::Kind::Aim: {
        GfVec3d target(0);
        double total = 0;
        for (const auto &source:input.sources) {
            if (!std::isfinite(source.normalizedWeight) ||
                source.normalizedWeight < 0) return false;
            target += source.frame.Origin() * source.normalizedWeight;
            total += source.normalizedWeight;
        }
        if (total > 0) {
            target /= total;
            *candidate = RigExecApplyAimConstraint(input.entering,target,input.aim);
        }
        return true;
    }
    }
    return false;
}
}
#endif
