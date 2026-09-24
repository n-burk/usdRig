//
// Shared scaffolding for mover-owned exec computations (spec §4.1).
//
// Every concrete mover schema with exec-side computations owns a
// statically registered computeMoverParameters over its own declared
// inputs, plus the scalar computeMoverStatus. The registration blocks
// and builders live in the mover's own TU under libs/rigExec/movers/;
// what is identical across them lives here: the token vocabulary, the
// common MoverAPI inputs, the shared envelope/status builders, and the
// vectorized-input collectors.
//
// Weight objects, blend inputs/samples, and the ribbon keep their own
// private tokens and adapters in moverKernels.cpp; nothing is shared
// with them but the packet kernels.
//

#ifndef RIGEXEC_MOVERS_MOVER_EXEC_COMMON_H
#define RIGEXEC_MOVERS_MOVER_EXEC_COMMON_H

#include "../types.h"

#include "pxr/base/tf/staticTokens.h"
#include "pxr/exec/vdf/context.h"
#include "pxr/exec/vdf/readIterator.h"
#include "pxr/usd/sdf/path.h"

PXR_NAMESPACE_USING_DIRECTIVE

#define RIGEXEC_MOVER_EXEC_TOKENS                                       \
    (computeMoverParameters)                                            \
    (computeMoverStatus)                                                \
    (computeWeightPacket)                                               \
    (computeBlendChannel)                                               \
    (computePointFrameArray)                                            \
    (computeMatrix)                                                     \
    (enabled)                                                           \
    (transform)                                                         \
    (weightPacket)                                                      \
    (blendChannels)                                                     \
    (basePoints)                                                        \
    (topologyCounts)                                                    \
    (topologyIndices)                                                   \
    (cagePoints)                                                        \
    (surfacePoints)                                                     \
    (bindCoords)                                                        \
    (driverFrames)                                                      \
    (moverPath)                                                         \
    ((inputsEnabled, "inputs:enabled"))                                 \
    ((inputsDefaultWeight, "inputs:defaultWeight"))                     \
    ((weightObjectRel, "rigExec:weightObject"))                         \
    ((deltaSpace, "rigExec:deltaSpace"))                                \
    ((modeAttr, "rigExec:mode"))                                         \
    ((divisionsAttr, "rigExec:divisions"))                              \
    ((restCagePointsAttr, "rigExec:restCagePoints"))                     \
    ((resolvedTransform, "rigExec:resolvedTransform"))                   \
    ((resolvedBlendInputs, "rigExec:resolvedBlendInputs"))               \
    ((resolvedBase, "rigExec:resolvedBase"))                             \
    ((resolvedTopologyCounts, "rigExec:resolvedTopologyCounts"))         \
    ((resolvedTopologyIndices, "rigExec:resolvedTopologyIndices"))       \
    ((resolvedCagePoints, "rigExec:resolvedCagePoints"))                 \
    ((resolvedSurfacePoints, "rigExec:resolvedSurfacePoints"))           \
    ((resolvedBindCoords, "rigExec:resolvedBindCoords"))                 \
    ((resolvedDriverFrames, "rigExec:resolvedDriverFrames"))

namespace rigExec {

TF_DECLARE_PUBLIC_TOKENS(RigExecMoverExecTokens, RIGEXEC_MOVER_EXEC_TOKENS);

// The common MoverAPI inputs every computeMoverParameters declares:
// inputs:enabled, inputs:defaultWeight, and the optional bound weight
// object's computeWeightPacket. Token references are fully qualified so
// the macro expands anywhere.
#define RIGEXEC_MOVER_COMMON_INPUTS                                     \
    AttributeValue<bool>(                                                \
        rigExec::RigExecMoverExecTokens->inputsEnabled),                \
        AttributeValue<float>(                                           \
            rigExec::RigExecMoverExecTokens->inputsDefaultWeight),      \
        Relationship(                                                   \
            rigExec::RigExecMoverExecTokens->weightObjectRel)           \
            .TargetedObjects<rigExec::RigExecWeightPacket>(             \
                rigExec::RigExecMoverExecTokens->computeWeightPacket)   \
            .InputName(                                                 \
                rigExec::RigExecMoverExecTokens->weightPacket)

// Collects a vectorized input into transient scratch
// (spec §6.5 ephemeral-scratch rule).
template <typename T>
std::vector<T>
RigExecMoverCollect(const VdfContext &ctx, const TfToken &name)
{
    VdfReadIterator<T> it(ctx, name);
    std::vector<T> out;
    out.reserve(it.ComputeSize());
    for (; !it.IsAtEnd(); ++it) {
        out.push_back(*it);
    }
    return out;
}

// The common MoverAPI envelope: a bound object supersedes the scalar.
// False when the envelope is invalid (MoverFailed).
inline bool
RigExecMoverSetCommonEnvelope(
    const VdfContext &ctx, RigExecMoverParameters *params)
{
    const RigExecWeightPacket *weights =
        ctx.GetInputValuePtr<RigExecWeightPacket>(
            RigExecMoverExecTokens->weightPacket);
    const float *defaultWeight =
        ctx.GetInputValuePtr<float>(
            RigExecMoverExecTokens->inputsDefaultWeight);
    params->weights = weights
        ? *weights
        : RigExecWeightPacket::Constant(defaultWeight ? *defaultWeight : 1.0f);
    return params->weights.valid;
}

// computeMoverStatus (spec §7.1): validates parameters and reports
// success, disabled, or MoverFailed; the property application consumes
// this scalar status and preserves the preceding vector when it does not
// allow applying.
inline RigExecMoverStatus
RigExecMoverBuildStatus(const VdfContext &ctx)
{
    RigExecMoverStatus status;
    const RigExecMoverParameters *params =
        ctx.GetInputValuePtr<RigExecMoverParameters>(
            RigExecMoverExecTokens->computeMoverParameters);
    if (!params) {
        status.state = TfToken("moverFailed");
        return status;
    }
    if (!params->enabled) {
        status.state = TfToken("disabled");
    } else if (params->valid) {
        status.state = TfToken("ok");
    } else {
        status.state = TfToken("moverFailed");
        // First bad canonical public address (spec §6.6). v0.1 reports
        // the failed mover's own canonical path; per-input attribution
        // is future diagnostic work.
        const SdfPath *path =
            ctx.GetInputValuePtr<SdfPath>(
                RigExecMoverExecTokens->moverPath);
        if (path) {
            status.firstBadAddress = path->GetString();
        }
    }
    return status;
}

}  // namespace rigExec

#endif  // RIGEXEC_MOVERS_MOVER_EXEC_COMMON_H
