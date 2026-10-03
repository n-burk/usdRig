// Private evaluator implementation types and helpers.
#ifndef RIGEXEC_RIG_EVALUATOR_CONSTRAINTS_H
#define RIGEXEC_RIG_EVALUATOR_CONSTRAINTS_H

#include "rigEvaluatorInternal.h"

namespace rigExec {

namespace evaluatorDetail {

/// Everything a constraint solve needs that is common to every operator. The
/// per-operator reads -- offsets, masks -- happen inside the solve, because
/// that is exactly what differs between operators.
struct _ConstraintSolveContext {
    const RigExecResolvedInputs *resolved = nullptr;
    UsdPrim prim;
    UsdTimeCode time;
    RigExecPointFrame inputFrame;
    const std::vector<RigExecConstraintSource> *sources = nullptr;
    RigExecConstraintAxisMask affect;
    RigExecEulerOrder order = RigExecEulerOrder::XYZ;
    double weight = 1.0;
    bool masksStatic = false;
    RigExecConstraintAxisMask precompTranslation;
    RigExecConstraintAxisMask precompRotation;
    RigExecConstraintAxisMask precompScale;
};

using _ConstraintSolveFn =
    RigExecPointFrame (*)(const _ConstraintSolveContext &);

// The translation, rotation, or scale channels addressed by a constraint mask.
enum class _ChannelGroup { None, Translation, Rotation, Scale, All };

// Operator dispatch and authored-channel contracts shared by compile and evaluate.
struct _ConstraintHandler {
    const char *schemaType;
    bool sourceFrame;      ///< blends rigExec:sources into one revision
    bool frameConstraint;  ///< compiles to frame wiring at all
    bool usesRotationOrder;
    bool dispatchesInline;
    _ChannelGroup maskGroup;
    /// Which scalar offset the operator honors. Parent is None: it composes
    /// PER-SOURCE offset ARRAYS instead, so the inherited scalar offsets
    /// would be silently ignored on it.
    _ChannelGroup offsetGroup;
    _ConstraintSolveFn solve;
};

const _ConstraintHandler *
_FindConstraintHandler(const TfToken &typeName);

bool
_IsSourceFrameConstraintType(const TfToken &typeName);

bool
_IsFrameConstraintType(const TfToken &typeName);

RigExecEulerOrder
_ParseConstraintEulerOrder(const TfToken &token);

bool
_IsUsableConstraintFrame(const RigExecPointFrame &frame);

bool
_TokenIsOneOf(const TfToken &value,
              std::initializer_list<const char *> allowed);

RigExecConstraintAxisMask
_ReadGroupMask(const RigExecResolvedInputs &resolved, const UsdPrim &prim,
               _ChannelGroup group, UsdTimeCode time);

} // namespace evaluatorDetail

} // namespace rigExec

#endif
