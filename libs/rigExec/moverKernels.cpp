//
// RigExec exec computations for solvers and mover operands
// (spec §7.2, §12.1).
//
// Weight objects publish computeWeightPacket, blend samples/inputs publish
// their descriptors, and the aggregate solvers publish
// computePointFrameArray. The geometry writer callbacks that used to live
// here belonged to compiler-authored hidden application prims; those, the
// compiler, and its derived stage are all gone -- point chains evaluate
// through the in-memory RigExecMoverGraph instead.
//
#include "types.h"
#include "moverGraph.h"

#include "rigExecMath/pointFrame.h"
#include "rigExecMath/geometryKernels.h"
#include "rigExecMath/simdKernels.h"
#include "rigExecMath/solvers.h"

#include "pxr/pxr.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/staticTokens.h"
#include "pxr/exec/exec/builtinComputations.h"
#include "pxr/exec/exec/registerSchema.h"
#include "pxr/exec/vdf/context.h"
#include "pxr/exec/vdf/readIterator.h"
#include "pxr/exec/vdf/readWriteIterator.h"

#include <algorithm>
#include <cmath>

using rigExec::RigExecBlendChannel;
using rigExec::RigExecBlendSampleData;
using rigExec::RigExecMoverParameters;
using rigExec::RigExecPointFrame;
using rigExec::RigExecPointFrameArray;
using rigExec::RigExecPointsPacket;
using rigExec::RigExecWeightPacket;

PXR_NAMESPACE_USING_DIRECTIVE

TF_DEFINE_PRIVATE_TOKENS(
    _tokens,

    // Computation names.
    (computeWeightPacket)
    (computeBlendSampleData)
    (computeBlendChannel)
    (computeMoverParameters)
    (computePointFrame)
    (computeMatrix)

    // Input names.
    (previous)
    (parameters)
    (basePacket)
    (samplePoints)
    (sampleData)
    (enabled)
    (transform)
    (weightPacket)
    (blendChannels)
    (basePoints)

    // Attribute tokens.
    ((representation, "rigExec:representation"))
    ((rangePolicy, "rigExec:rangePolicy"))
    ((values, "rigExec:values"))
    ((indices, "rigExec:indices"))
    ((defaultWeight, "rigExec:defaultWeight"))
    ((baseWeight, "rigExec:baseWeight"))
    ((inputsDriver, "inputs:driver"))
    ((inputsScale, "inputs:scale"))
    ((inputsBias, "inputs:bias"))
    ((activation, "rigExec:activation"))
    ((targetPoints, "rigExec:targetPoints"))
    ((samples, "rigExec:samples"))
    ((inputsWeight, "inputs:weight"))
    ((moverRel, "rigExec:mover"))
    ((weightObjectRel, "rigExec:weightObject"))
    ((inputsEnabled, "inputs:enabled"))
    ((resolvedTransform, "rigExec:resolvedTransform"))
    ((resolvedBlendInputs, "rigExec:resolvedBlendInputs"))
    ((resolvedBase, "rigExec:resolvedBase"))
    ((resolvedFinalPoints, "rigExec:resolvedFinalPoints"))
    ((resolvedTopologyCounts, "rigExec:resolvedTopologyCounts"))
    ((resolvedTopologyIndices, "rigExec:resolvedTopologyIndices"))
    ((resolvedCagePoints, "rigExec:resolvedCagePoints"))
    ((resolvedSurfacePoints, "rigExec:resolvedSurfacePoints"))
    ((resolvedBindCoords, "rigExec:resolvedBindCoords"))
    ((resolvedDriverFrames, "rigExec:resolvedDriverFrames"))
    ((computeDriverPoints, "rigExec:computeDriverPoints"))
    ((computeRestDriverPoints, "rigExec:computeRestDriverPoints"))
    ((resolvedWidths, "rigExec:resolvedWidths"))
    ((restCagePointsAttr, "rigExec:restCagePoints"))
    ((divisionsAttr, "rigExec:divisions"))
    ((sampleCountAttr, "rigExec:sampleCount"))
    ((modeAttr, "rigExec:mode"))
    ((strengthAttr, "inputs:strength"))
    (computePointFrameArray)
    (finalPoints)
    (topologyCounts)
    (topologyIndices)
    (cagePoints)
    (surfacePoints)
    (bindCoords)
    (driverFrames)
    (driverPoints)
    (restDriver)
    (widthsInput)
    (computeMoverStatus)
    (status)
    (moverPath)
    ((outputsExpression, "outputs:expression"))
);

namespace {

// Applies the range policy to a candidate weight; returns false on a
// strict violation or non-finite input (spec §4.1 RangePolicy).
bool
_ApplyRangePolicy(const TfToken &policy, float *w)
{
    if (!std::isfinite(*w)) {
        return false;
    }
    if (*w < 0.0f || *w > 1.0f) {
        if (policy == "clamp") {
            *w = std::min(std::max(*w, 0.0f), 1.0f);
            return true;
        }
        return false;
    }
    return true;
}

RigExecWeightPacket
_BuildStaticWeightPacket(const VdfContext &ctx)
{
    RigExecWeightPacket packet;
    static const TfToken constant("constant");
    static const TfToken strict("strict");
    const TfToken *repr =
        ctx.GetInputValuePtr<TfToken>(_tokens->representation);
    const TfToken *policy = ctx.GetInputValuePtr<TfToken>(_tokens->rangePolicy);
    packet.representation = repr ? *repr : constant;
    packet.rangePolicy = policy ? *policy : strict;
    // Unknown structural tokens are rejected, never coerced (spec §4.1);
    // allowedTokens metadata alone is not validation.
    if (packet.representation != "constant" &&
        packet.representation != "dense" &&
        packet.representation != "sparse") {
        return packet;
    }
    if (packet.rangePolicy != "strict" && packet.rangePolicy != "clamp") {
        return packet;
    }
    const float *def = ctx.GetInputValuePtr<float>(_tokens->defaultWeight);
    packet.defaultWeight = def ? *def : 0.0f;

    std::vector<std::pair<int, float>> pairs;
    {
        VdfReadIterator<float> vIt(ctx, _tokens->values);
        VdfReadIterator<int> iIt(ctx, _tokens->indices);
        if (packet.representation == "sparse") {
            for (; !vIt.IsAtEnd() && !iIt.IsAtEnd(); ++vIt, ++iIt) {
                pairs.emplace_back(*iIt, *vIt);
            }
            if (!vIt.IsAtEnd() || !iIt.IsAtEnd()) {
                return packet;  // size mismatch: invalid
            }
        } else {
            for (; !vIt.IsAtEnd(); ++vIt) {
                packet.values.push_back(*vIt);
            }
        }
    }
    // Canonical sorted sparse support; authored pair order is
    // non-semantic and duplicates are rejected (spec §4.1).
    if (packet.representation == "sparse") {
        std::sort(pairs.begin(), pairs.end());
        for (size_t i = 0; i < pairs.size(); ++i) {
            if (i > 0 && pairs[i].first == pairs[i - 1].first) {
                return packet;  // duplicate index: invalid
            }
            packet.indices.push_back(pairs[i].first);
            packet.values.push_back(pairs[i].second);
        }
    }
    if (packet.representation == "dense" && packet.defaultWeight != 0.0f) {
        return packet;  // canonical dense default is zero
    }

    for (float &w : packet.values) {
        if (!_ApplyRangePolicy(packet.rangePolicy, &w)) {
            return packet;
        }
    }
    if (!_ApplyRangePolicy(packet.rangePolicy, &packet.defaultWeight)) {
        return packet;
    }
    packet.valid = true;
    return packet;
}

RigExecWeightPacket
_BuildDynamicWeightPacket(const VdfContext &ctx)
{
    RigExecWeightPacket packet;
    static const TfToken constant("constant");
    static const TfToken strict("strict");
    const TfToken *repr =
        ctx.GetInputValuePtr<TfToken>(_tokens->representation);
    const TfToken *policy = ctx.GetInputValuePtr<TfToken>(_tokens->rangePolicy);
    packet.representation = repr ? *repr : constant;
    packet.rangePolicy = policy ? *policy : strict;
    if (packet.representation != "constant" &&
        packet.representation != "dense" &&
        packet.representation != "sparse") {
        return packet;
    }
    if (packet.rangePolicy != "strict" && packet.rangePolicy != "clamp") {
        return packet;
    }

    const float *driver = ctx.GetInputValuePtr<float>(_tokens->inputsDriver);
    const float *scale = ctx.GetInputValuePtr<float>(_tokens->inputsScale);
    const float *bias = ctx.GetInputValuePtr<float>(_tokens->inputsBias);
    const float d = driver ? *driver : 1.0f;
    const float s = scale ? *scale : 1.0f;
    const float a = bias ? *bias : 0.0f;

    const RigExecWeightPacket *base =
        ctx.GetInputValuePtr<RigExecWeightPacket>(_tokens->basePacket);
    if (!base) {
        // Without a base only constant is legal and b_i = 1 (spec §4.1).
        if (packet.representation != "constant") {
            return packet;
        }
        packet.defaultWeight = (1.0f * d) * s + a;
        if (!_ApplyRangePolicy(packet.rangePolicy, &packet.defaultWeight)) {
            return packet;
        }
        packet.valid = true;
        return packet;
    }
    if (!base->valid || base->representation != packet.representation) {
        return packet;
    }

    // r_i = (b_i d) s + a over the base field; dense keeps canonical
    // packet default zero (spec §4.1).
    packet.indices = base->indices;
    packet.values.reserve(base->values.size());
    for (float b : base->values) {
        float r = (b * d) * s + a;
        if (!_ApplyRangePolicy(packet.rangePolicy, &r)) {
            return packet;
        }
        packet.values.push_back(r);
    }
    if (packet.representation == "dense") {
        packet.defaultWeight = 0.0f;
    } else {
        packet.defaultWeight = (base->defaultWeight * d) * s + a;
        if (!_ApplyRangePolicy(packet.rangePolicy, &packet.defaultWeight)) {
            return packet;
        }
    }
    packet.valid = true;
    return packet;
}

RigExecBlendSampleData
_BuildBlendSampleData(const VdfContext &ctx)
{
    RigExecBlendSampleData data;
    const float *activation =
        ctx.GetInputValuePtr<float>(_tokens->activation);
    data.activation = activation ? *activation : 1.0f;
    VdfReadIterator<GfVec3f> it(ctx, _tokens->samplePoints);
    data.points.reserve(it.ComputeSize());
    for (; !it.IsAtEnd(); ++it) {
        data.points.push_back(*it);
    }
    return data;
}

RigExecBlendChannel
_BuildBlendChannel(const VdfContext &ctx)
{
    RigExecBlendChannel channel;
    const float *weight = ctx.GetInputValuePtr<float>(_tokens->inputsWeight);
    channel.weight = weight ? *weight : 0.0f;
    VdfReadIterator<RigExecBlendSampleData> it(ctx, _tokens->sampleData);
    for (; !it.IsAtEnd(); ++it) {
        channel.samples.push_back(*it);
    }
    std::sort(channel.samples.begin(), channel.samples.end(),
              [](const RigExecBlendSampleData &x,
                 const RigExecBlendSampleData &y) {
                  return x.activation < y.activation;
              });
    return channel;
}

// Mover-owned parameter packets (spec §4.1: every concrete mover schema
// owns a statically registered computeMoverParameters). The compiler's
// compiler helpers (RigExecResolvedMatrix and friends, spec §12.1) author
// the rigExec:resolved* relationships in the generated session layer,
// binding each declared dependency to exactly one catalogued provider.

RigExecMoverParameters
_BuildMatrixMoverParameters(const VdfContext &ctx)
{
    RigExecMoverParameters params;
    params.kind = TfToken("matrix");
    const bool *enabled = ctx.GetInputValuePtr<bool>(_tokens->enabled);
    params.enabled = enabled ? *enabled : true;
    if (!params.enabled) {
        params.valid = true;  // disabled is an ordinary pass-through
        return params;
    }

    const GfMatrix4d *transform =
        ctx.GetInputValuePtr<GfMatrix4d>(_tokens->transform);
    const RigExecWeightPacket *weights =
        ctx.GetInputValuePtr<RigExecWeightPacket>(_tokens->weightPacket);
    if (!transform || !weights || !weights->valid) {
        return params;  // MoverFailed
    }
    // The matrix must be finite and affine (spec §7.4).
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (!std::isfinite((*transform)[i][j])) {
                return params;
            }
        }
    }
    if ((*transform)[0][3] != 0 || (*transform)[1][3] != 0 ||
        (*transform)[2][3] != 0 || (*transform)[3][3] != 1) {
        return params;
    }
    params.transform = *transform;
    params.weights = *weights;
    params.valid = true;
    return params;
}

RigExecMoverParameters
_BuildBlendMoverParameters(const VdfContext &ctx)
{
    RigExecMoverParameters params;
    params.kind = TfToken("blendShape");
    const bool *enabled = ctx.GetInputValuePtr<bool>(_tokens->enabled);
    params.enabled = enabled ? *enabled : true;
    if (!params.enabled) {
        params.valid = true;
        return params;
    }

    // Base points for delta derivation (spec §7.3): deltas derive against
    // the authored base, never the preceding revision.
    std::vector<GfVec3f> base;
    {
        VdfReadIterator<GfVec3f> it(ctx, _tokens->basePoints);
        base.reserve(it.ComputeSize());
        for (; !it.IsAtEnd(); ++it) {
            base.push_back(*it);
        }
    }
    if (base.empty()) {
        return params;
    }
    // Summed by the shared kernel (moverGraph.h): RigExecRigEvaluator builds
    // the same packet from tapped channels with no derived stage, and the two
    // must agree exactly.
    std::vector<RigExecBlendChannel> channelValues;
    {
        VdfReadIterator<RigExecBlendChannel> channels(
            ctx, _tokens->blendChannels);
        channelValues.reserve(channels.ComputeSize());
        for (; !channels.IsAtEnd(); ++channels) {
            channelValues.push_back(*channels);
        }
    }
    if (!rigExec::RigExecSumBlendChannels(channelValues, base,
                                          &params.blendDeltas)) {
        return params;  // structural error: fails atomically
    }
    // Optional per-point mask (spec §7.3).
    const RigExecWeightPacket *mask =
        ctx.GetInputValuePtr<RigExecWeightPacket>(_tokens->weightPacket);
    if (mask) {
        if (!mask->valid) {
            return params;
        }
        params.weights = *mask;
    }
    params.valid = true;
    return params;
}

// computeMoverStatus (spec §7.1): validates parameters and reports
// success, disabled, or MoverFailed; the property application consumes
// this scalar status and preserves the preceding vector when it does not
// allow applying.
rigExec::RigExecMoverStatus
_BuildMoverStatus(const VdfContext &ctx)
{
    rigExec::RigExecMoverStatus status;
    const RigExecMoverParameters *params =
        ctx.GetInputValuePtr<RigExecMoverParameters>(
            _tokens->computeMoverParameters);
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
            ctx.GetInputValuePtr<SdfPath>(_tokens->moverPath);
        if (path) {
            status.firstBadAddress = path->GetString();
        }
    }
    return status;
}

// The stock v26.08 callback form for a point3f[] application (spec §12.1):
// reads the preceding vector with VdfReadIterator, allocates/writes the
// result with stock VDF vector APIs, and references the prior input
// directly for disabled/failure pass-through. Each host's kernel is fixed
// by its application schema — there is no runtime operation dispatch
// (spec §4.1: no universal array host).

// Gate shared by both hosts: the scalar computeMoverStatus decides
// whether the kernel may apply (spec §7.1).
bool
_StatusAllowsApply(const VdfContext &ctx)
{
    const rigExec::RigExecMoverStatus *status =
        ctx.GetInputValuePtr<rigExec::RigExecMoverStatus>(_tokens->status);
    return status && status->AllowsApply();
}

void
_EvaluateMatrixPointArrayExpression(const VdfContext &ctx)
{
    const RigExecMoverParameters *params =
        ctx.GetInputValuePtr<RigExecMoverParameters>(_tokens->parameters);
    VdfReadIterator<GfVec3f> previous(ctx, _tokens->previous);
    const size_t count = previous.ComputeSize();

    auto passThrough = [&ctx]() {
        ctx.SetOutputToReferenceInput(_tokens->previous);
    };
    if (!_StatusAllowsApply(ctx) || !params || !params->valid ||
        params->kind != "matrix") {
        passThrough();
        return;
    }
    // A dense field whose cardinality mismatches the target fails the
    // application atomically (spec §7.4).
    if (params->weights.representation == "dense" &&
        params->weights.values.size() != count) {
        passThrough();
        return;
    }
    // Resolve weights first so a cardinality failure passes through
    // before any output is written.
    std::vector<float> weights(count);
    for (size_t i = 0; i < count; ++i) {
        weights[i] = params->weights.Resolve(i, count);
        if (weights[i] < 0.0f) {
            passThrough();
            return;
        }
    }
    // CPU SIMD over the contiguous elements (spec 6.5): parity-gated
    // against the scalar reference; RIGEXEC_ENABLE_SIMD=false forces the
    // scalar path for debugging.
    static const bool useSimd = TfGetenvBool("RIGEXEC_ENABLE_SIMD", true);
    auto out = VdfReadWriteIterator<GfVec3f>::Allocate(ctx, count);
    if (useSimd) {
        std::vector<GfVec3f> scratch;
        scratch.reserve(count);
        for (; !previous.IsAtEnd(); ++previous) {
            scratch.push_back(*previous);
        }
        rigExec::RigExecApplyWeightedMatrixSimd(
            scratch.data(), scratch.data(), weights.data(), count,
            params->transform);
        size_t i = 0;
        for (; !out.IsAtEnd(); ++out, ++i) {
            *out = scratch[i];
        }
    } else {
        size_t i = 0;
        for (; !previous.IsAtEnd(); ++previous, ++out, ++i) {
            *out = GfVec3f(rigExec::RigExecApplyWeightedMatrix(
                GfVec3d(*previous), params->transform, weights[i]));
        }
    }
}

void
_EvaluateBlendPointArrayExpression(const VdfContext &ctx)
{
    const RigExecMoverParameters *params =
        ctx.GetInputValuePtr<RigExecMoverParameters>(_tokens->parameters);
    VdfReadIterator<GfVec3f> previous(ctx, _tokens->previous);
    const size_t count = previous.ComputeSize();

    auto passThrough = [&ctx]() {
        ctx.SetOutputToReferenceInput(_tokens->previous);
    };
    if (!_StatusAllowsApply(ctx) || !params || !params->valid ||
        params->kind != "blendShape") {
        passThrough();
        return;
    }
    if (params->blendDeltas.size() != count) {
        passThrough();
        return;
    }
    const bool hasMask = params->weights.valid;
    if (hasMask && params->weights.representation == "dense" &&
        params->weights.values.size() != count) {
        passThrough();
        return;
    }
    auto out = VdfReadWriteIterator<GfVec3f>::Allocate(ctx, count);
    size_t i = 0;
    for (; !previous.IsAtEnd(); ++previous, ++out, ++i) {
        float mask = 1.0f;
        if (hasMask) {
            mask = params->weights.Resolve(i, count);
            if (mask < 0.0f) {
                passThrough();
                return;
            }
        }
        *out = *previous + params->blendDeltas[i] * mask;
    }
}

// Collects a vectorized element input into transient scratch
// (spec §6.5 ephemeral-scratch rule).
template <typename T>
std::vector<T>
_Collect(const VdfContext &ctx, const TfToken &name)
{
    VdfReadIterator<T> it(ctx, name);
    std::vector<T> out;
    out.reserve(it.ComputeSize());
    for (; !it.IsAtEnd(); ++it) {
        out.push_back(*it);
    }
    return out;
}

// RigExecSmoothMover parameters: fixed-adjacency Laplacian smoothing of
// the destination's standard topology (spec §7.6 revised).
RigExecMoverParameters
_BuildSmoothMoverParameters(const VdfContext &ctx)
{
    RigExecMoverParameters params;
    params.kind = TfToken("smooth");
    const bool *enabled = ctx.GetInputValuePtr<bool>(_tokens->enabled);
    params.enabled = enabled ? *enabled : true;
    if (!params.enabled) {
        params.valid = true;
        return params;
    }
    const float *strength =
        ctx.GetInputValuePtr<float>(_tokens->strengthAttr);
    params.strength = strength ? *strength : 0.5f;
    if (!std::isfinite(params.strength)) {
        return params;
    }
    params.topologyCounts = _Collect<int>(ctx, _tokens->topologyCounts);
    params.topologyIndices = _Collect<int>(ctx, _tokens->topologyIndices);
    params.valid = !params.topologyCounts.empty();
    return params;
}

// RigExecVolumeCorrectMover parameters: the correction reference is the
// authored base bound volume (spec §7.6 revised).
RigExecMoverParameters
_BuildVolumeCorrectMoverParameters(const VdfContext &ctx)
{
    RigExecMoverParameters params;
    params.kind = TfToken("volumeCorrect");
    const bool *enabled = ctx.GetInputValuePtr<bool>(_tokens->enabled);
    params.enabled = enabled ? *enabled : true;
    if (!params.enabled) {
        params.valid = true;
        return params;
    }
    const float *strength =
        ctx.GetInputValuePtr<float>(_tokens->strengthAttr);
    params.strength = strength ? *strength : 0.0f;
    if (!std::isfinite(params.strength)) {
        return params;
    }
    const std::vector<GfVec3f> base =
        _Collect<GfVec3f>(ctx, _tokens->basePoints);
    if (base.empty()) {
        return params;
    }
    params.referenceVolume =
        rigExec::RigExecBoundVolume(base.data(), base.size());
    params.valid = true;
    return params;
}

// Synthesized derived-maintenance parameters (spec §7.6 revised): the
// hosts are compiler-authored with no authored mover and no enable; the
// rig-level derived policy gates synthesis at compile time.
RigExecMoverParameters
_BuildLatticeMoverParameters(const VdfContext &ctx)
{
    RigExecMoverParameters params;
    params.kind = TfToken("lattice");
    const bool *enabled = ctx.GetInputValuePtr<bool>(_tokens->enabled);
    params.enabled = enabled ? *enabled : true;
    if (!params.enabled) {
        params.valid = true;
        return params;
    }
    const GfVec3i *divisions =
        ctx.GetInputValuePtr<GfVec3i>(_tokens->divisionsAttr);
    params.divisions = divisions ? *divisions : GfVec3i(0);
    params.restPoints = _Collect<GfVec3f>(ctx, _tokens->basePoints);
    params.auxPoints = _Collect<GfVec3f>(ctx, _tokens->restCagePointsAttr);
    params.auxPointsB = _Collect<GfVec3f>(ctx, _tokens->cagePoints);
    const size_t cageCount = size_t(params.divisions[0]) *
                             size_t(params.divisions[1]) *
                             size_t(params.divisions[2]);
    params.valid = params.divisions[0] >= 2 && params.divisions[1] >= 2 &&
                   params.divisions[2] >= 2 &&
                   params.auxPoints.size() == cageCount &&
                   params.auxPointsB.size() == cageCount &&
                   !params.restPoints.empty();
    return params;
}

RigExecMoverParameters
_BuildSurfaceMoverParameters(const VdfContext &ctx)
{
    RigExecMoverParameters params;
    params.kind = TfToken("surfaceProject");
    const bool *enabled = ctx.GetInputValuePtr<bool>(_tokens->enabled);
    params.enabled = enabled ? *enabled : true;
    if (!params.enabled) {
        params.valid = true;
        return params;
    }
    params.strength = 1.0f;  // v0.1 attach/project maps fully
    params.auxPoints = _Collect<GfVec3f>(ctx, _tokens->surfacePoints);
    params.topologyCounts = _Collect<int>(ctx, _tokens->topologyCounts);
    params.topologyIndices = _Collect<int>(ctx, _tokens->topologyIndices);
    params.valid = !params.auxPoints.empty() &&
                   !params.topologyCounts.empty();
    return params;
}

RigExecMoverParameters
_BuildCurveMoverParameters(const VdfContext &ctx)
{
    RigExecMoverParameters params;
    static const TfToken ribbon("ribbon");
    const TfToken *mode = ctx.GetInputValuePtr<TfToken>(_tokens->modeAttr);
    params.kind = mode ? *mode : ribbon;
    const bool *enabled = ctx.GetInputValuePtr<bool>(_tokens->enabled);
    params.enabled = enabled ? *enabled : true;
    if (!params.enabled) {
        params.valid = true;
        return params;
    }
    const RigExecPointFrameArray *frames =
        ctx.GetInputValuePtr<RigExecPointFrameArray>(_tokens->driverFrames);
    if (!frames || frames->IsEmpty() ||
        frames->rests.size() != frames->GetSize()) {
        return params;
    }
    params.frames = *frames;
    if (params.kind == "ribbon") {
        params.bindCoords = _Collect<GfVec2f>(ctx, _tokens->bindCoords);
        params.valid = !params.bindCoords.empty();
    } else if (params.kind == "emitGuidePoints") {
        params.valid = true;
    }
    return params;
}

// Shared scratch-collect / kernel / write-back body for point3f[] hosts
// (spec §6.5: transient scratch is released before the callback returns).
template <typename Kernel>
void
_EvaluateScratchKernel(const VdfContext &ctx, const TfToken &expectedKind,
                       Kernel &&kernel)
{
    const RigExecMoverParameters *params =
        ctx.GetInputValuePtr<RigExecMoverParameters>(_tokens->parameters);
    VdfReadIterator<GfVec3f> previous(ctx, _tokens->previous);
    auto passThrough = [&ctx]() {
        ctx.SetOutputToReferenceInput(_tokens->previous);
    };
    if (!_StatusAllowsApply(ctx) || !params || !params->valid ||
        params->kind != expectedKind) {
        passThrough();
        return;
    }
    std::vector<GfVec3f> scratch;
    scratch.reserve(previous.ComputeSize());
    for (; !previous.IsAtEnd(); ++previous) {
        scratch.push_back(*previous);
    }
    if (!kernel(*params, &scratch)) {
        passThrough();
        return;
    }
    auto out = VdfReadWriteIterator<GfVec3f>::Allocate(ctx, scratch.size());
    size_t i = 0;
    for (; !out.IsAtEnd(); ++out, ++i) {
        *out = scratch[i];
    }
}

// normal3f[] host: recomputed vertex normals from final same-generation
// points and standard topology (spec §7.6).
// float3[] host: recomputed two-element extent from final points and the
// authoritative winning widths (spec §7.6).
// RigExecRibbon solver: rotation-minimizing frame samples along the
// driver curve, published with paired rest frames (spec §7.5). The rest
// driver points are the compiler-captured bind-time curve value.
RigExecPointFrameArray
_ComputeRibbonFrames(const VdfContext &ctx)
{
    RigExecPointFrameArray result;
    const int *countPtr = ctx.GetInputValuePtr<int>(_tokens->sampleCountAttr);
    const int sampleCount = countPtr ? *countPtr : 5;
    const RigExecPointsPacket *const posedPtr =
        ctx.GetInputValuePtr<RigExecPointsPacket>(_tokens->driverPoints);
    const RigExecPointsPacket *const restPtr =
        ctx.GetInputValuePtr<RigExecPointsPacket>(_tokens->restDriver);
    const std::vector<GfVec3f> posed =
        posedPtr ? posedPtr->points : std::vector<GfVec3f>();
    const std::vector<GfVec3f> rest =
        restPtr ? restPtr->points : std::vector<GfVec3f>();
    if (posed.empty() || rest.empty() || sampleCount < 2) {
        return result;
    }
    const rigExec::RigExecCurveFrameSamples posedSamples =
        rigExec::RigExecSampleCurveRMF(posed, sampleCount);
    const rigExec::RigExecCurveFrameSamples restSamples =
        rigExec::RigExecSampleCurveRMF(rest, sampleCount);
    if (posedSamples.GetSize() != size_t(sampleCount) ||
        restSamples.GetSize() != size_t(sampleCount)) {
        return result;
    }
    result.frames.reserve(sampleCount);
    result.rests.reserve(sampleCount);
    for (int k = 0; k < sampleCount; ++k) {
        RigExecPointFrame frame;
        frame.points = {
            GfVec3d(posedSamples.positions[k]),
            GfVec3d(posedSamples.positions[k] + posedSamples.tangents[k]),
            GfVec3d(posedSamples.positions[k] + posedSamples.normals[k]),
            GfVec3d(posedSamples.positions[k] + posedSamples.binormals[k])};
        frame.flags = rigExec::RigExecPointFrameValid;
        result.frames.push_back(frame);
        result.rests.push_back({
            GfVec3d(restSamples.positions[k]),
            GfVec3d(restSamples.positions[k] + restSamples.tangents[k]),
            GfVec3d(restSamples.positions[k] + restSamples.normals[k]),
            GfVec3d(restSamples.positions[k] + restSamples.binormals[k])});
    }
    return result;
}


}  // namespace

// ---------------------------------------------------------------------------
// Weight objects publish computeWeightPacket (spec §12.1).
// ---------------------------------------------------------------------------

EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(RigExecStaticWeight)
{
    self.PrimComputation(_tokens->computeWeightPacket)
        .Callback<RigExecWeightPacket>(&_BuildStaticWeightPacket)
        .Inputs(
            AttributeValue<TfToken>(_tokens->representation),
            AttributeValue<TfToken>(_tokens->rangePolicy),
            AttributeValue<float>(_tokens->values),
            AttributeValue<int>(_tokens->indices),
            AttributeValue<float>(_tokens->defaultWeight));
}

EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(RigExecDynamicWeight)
{
    self.PrimComputation(_tokens->computeWeightPacket)
        .Callback<RigExecWeightPacket>(&_BuildDynamicWeightPacket)
        .Inputs(
            AttributeValue<TfToken>(_tokens->representation),
            AttributeValue<TfToken>(_tokens->rangePolicy),
            AttributeValue<float>(_tokens->inputsDriver),
            AttributeValue<float>(_tokens->inputsScale),
            AttributeValue<float>(_tokens->inputsBias),
            Relationship(_tokens->baseWeight)
                .TargetedObjects<RigExecWeightPacket>(
                    _tokens->computeWeightPacket)
                .InputName(_tokens->basePacket));
}

// ---------------------------------------------------------------------------
// Blend descriptors (spec §12.1).
// ---------------------------------------------------------------------------

EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(RigExecBlendSample)
{
    self.PrimComputation(_tokens->computeBlendSampleData)
        .Callback<RigExecBlendSampleData>(&_BuildBlendSampleData)
        .Inputs(
            AttributeValue<float>(_tokens->activation),
            Relationship(_tokens->targetPoints)
                .TargetedObjects<GfVec3f>(
                    ExecBuiltinComputations->computeValue)
                .InputName(_tokens->samplePoints)
                .Required());
}

EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(RigExecBlendInput)
{
    self.PrimComputation(_tokens->computeBlendChannel)
        .Callback<RigExecBlendChannel>(&_BuildBlendChannel)
        .Inputs(
            AttributeValue<float>(_tokens->inputsWeight),
            Relationship(_tokens->samples)
                .TargetedObjects<RigExecBlendSampleData>(
                    _tokens->computeBlendSampleData)
                .InputName(_tokens->sampleData)
                .Required());
}

// ---------------------------------------------------------------------------
// Mover-owned parameter/status computations (spec §4.1): each concrete
// mover schema statically registers computeMoverParameters over its own
// declared attributes/relationships, plus the scalar computeMoverStatus.
// The rigExec:resolved* relationships are authored by the compiler's
// the compiler (spec §12.1 RigExecResolvedMatrix and friends), binding the
// declared read phase to exactly one catalogued provider.
// ---------------------------------------------------------------------------

EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(RigExecMatrixMover)
{
    self.PrimComputation(_tokens->computeMoverParameters)
        .Callback<RigExecMoverParameters>(&_BuildMatrixMoverParameters)
        .Inputs(
            AttributeValue<bool>(_tokens->inputsEnabled),
            Relationship(_tokens->resolvedTransform)
                .TargetedObjects<GfMatrix4d>(_tokens->computeMatrix)
                .InputName(_tokens->transform),
            Relationship(_tokens->weightObjectRel)
                .TargetedObjects<RigExecWeightPacket>(
                    _tokens->computeWeightPacket)
                .InputName(_tokens->weightPacket));

    self.PrimComputation(_tokens->computeMoverStatus)
        .Callback<rigExec::RigExecMoverStatus>(&_BuildMoverStatus)
        .Inputs(
            Computation<RigExecMoverParameters>(
                _tokens->computeMoverParameters)
                .Required(),
            Computation<SdfPath>(ExecBuiltinComputations->computePath)
                .InputName(_tokens->moverPath));
}

EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(RigExecBlendShapeMover)
{
    self.PrimComputation(_tokens->computeMoverParameters)
        .Callback<RigExecMoverParameters>(&_BuildBlendMoverParameters)
        .Inputs(
            AttributeValue<bool>(_tokens->inputsEnabled),
            Relationship(_tokens->resolvedBlendInputs)
                .TargetedObjects<RigExecBlendChannel>(
                    _tokens->computeBlendChannel)
                .InputName(_tokens->blendChannels),
            Relationship(_tokens->resolvedBase)
                .TargetedObjects<GfVec3f>(
                    ExecBuiltinComputations->computeValue)
                .InputName(_tokens->basePoints),
            Relationship(_tokens->weightObjectRel)
                .TargetedObjects<RigExecWeightPacket>(
                    _tokens->computeWeightPacket)
                .InputName(_tokens->weightPacket));

    self.PrimComputation(_tokens->computeMoverStatus)
        .Callback<rigExec::RigExecMoverStatus>(&_BuildMoverStatus)
        .Inputs(
            Computation<RigExecMoverParameters>(
                _tokens->computeMoverParameters)
                .Required(),
            Computation<SdfPath>(ExecBuiltinComputations->computePath)
                .InputName(_tokens->moverPath));
}

// ---------------------------------------------------------------------------
// Operation/type-specific property applications (spec §4.1, §7.2): each
// host's frozen signature consumes the preceding exact-typed vector, the
// realizing mover's computeMoverParameters, and computeMoverStatus.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// RigExecPointFrameMoverApplication: one writer revision of a transform
// provider's point frame (spec §4.2), publishing paired views.
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// RigExecRibbon solver (spec §7.5): frame samples from the driver curve.
// ---------------------------------------------------------------------------

EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(RigExecRibbon)
{
    // Driver-curve points, live and at bind time, as boxed single values.
    //
    // These callbacks return EMPTY and are meant to be overridden: the
    // evaluator resolves rigExec:driverCurve to the target's points and
    // supplies both, the same way a solver-bound joint's frame is supplied.
    // A ribbon whose driver cannot be resolved therefore samples an empty
    // curve and publishes no frames, rather than silently using stale data.
    //
    // Two exec constraints force this shape. A relationship accessor can
    // request computations on its TARGETS but not a named attribute of them,
    // so the points cannot be reached from the ribbon prim. And an override
    // of a `point3f[]` attribute is type-checked against the ELEMENT type,
    // so an array cannot be handed over that way -- hence the packet.
    self.PrimComputation(_tokens->computeDriverPoints)
        .Callback<RigExecPointsPacket>(
            +[](const VdfContext &) { return RigExecPointsPacket(); });
    self.PrimComputation(_tokens->computeRestDriverPoints)
        .Callback<RigExecPointsPacket>(
            +[](const VdfContext &) { return RigExecPointsPacket(); });

    self.PrimComputation(_tokens->computePointFrameArray)
        .Callback<rigExec::RigExecPointFrameArray>(&_ComputeRibbonFrames)
        // Both point arrays arrive as VALUE OVERRIDES on schema-declared
        // attributes, never from authored scene description. The evaluator
        // resolves rigExec:driverCurve to the target's points attribute and
        // supplies the live and bind-time values, exactly as a solver-bound
        // joint receives its frame -- which is what let the last authoring
        // pass, and the derived stage that held it, be deleted.
        //
        // A Relationship accessor cannot do this job: TargetedObjects
        // requests a computation on the TARGETS, and rigExec:driverCurve
        // targets the curve prim, not its points attribute.
        .Inputs(
            AttributeValue<int>(_tokens->sampleCountAttr),
            Computation<RigExecPointsPacket>(_tokens->computeRestDriverPoints)
                .InputName(_tokens->restDriver),
            Computation<RigExecPointsPacket>(_tokens->computeDriverPoints)
                .InputName(_tokens->driverPoints));
}

// ---------------------------------------------------------------------------
// Mover-owned parameter/status computations for the Phase 3 operations
// (spec §4.1): each concrete mover schema statically declares its exact
// inputs; the compiler authors the rigExec:resolved* wiring.
// ---------------------------------------------------------------------------

EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(RigExecSmoothMover)
{
    self.PrimComputation(_tokens->computeMoverParameters)
        .Callback<RigExecMoverParameters>(&_BuildSmoothMoverParameters)
        .Inputs(
            AttributeValue<bool>(_tokens->inputsEnabled),
            AttributeValue<float>(_tokens->strengthAttr),
            Relationship(_tokens->resolvedTopologyCounts)
                .TargetedObjects<int>(ExecBuiltinComputations->computeValue)
                .InputName(_tokens->topologyCounts),
            Relationship(_tokens->resolvedTopologyIndices)
                .TargetedObjects<int>(ExecBuiltinComputations->computeValue)
                .InputName(_tokens->topologyIndices));

    self.PrimComputation(_tokens->computeMoverStatus)
        .Callback<rigExec::RigExecMoverStatus>(&_BuildMoverStatus)
        .Inputs(
            Computation<RigExecMoverParameters>(
                _tokens->computeMoverParameters)
                .Required(),
            Computation<SdfPath>(ExecBuiltinComputations->computePath)
                .InputName(_tokens->moverPath));
}

EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(RigExecVolumeCorrectMover)
{
    self.PrimComputation(_tokens->computeMoverParameters)
        .Callback<RigExecMoverParameters>(
            &_BuildVolumeCorrectMoverParameters)
        .Inputs(
            AttributeValue<bool>(_tokens->inputsEnabled),
            AttributeValue<float>(_tokens->strengthAttr),
            Relationship(_tokens->resolvedBase)
                .TargetedObjects<GfVec3f>(
                    ExecBuiltinComputations->computeValue)
                .InputName(_tokens->basePoints));

    self.PrimComputation(_tokens->computeMoverStatus)
        .Callback<rigExec::RigExecMoverStatus>(&_BuildMoverStatus)
        .Inputs(
            Computation<RigExecMoverParameters>(
                _tokens->computeMoverParameters)
                .Required(),
            Computation<SdfPath>(ExecBuiltinComputations->computePath)
                .InputName(_tokens->moverPath));
}

EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(RigExecLatticeMover)
{
    self.PrimComputation(_tokens->computeMoverParameters)
        .Callback<RigExecMoverParameters>(&_BuildLatticeMoverParameters)
        .Inputs(
            AttributeValue<bool>(_tokens->inputsEnabled),
            AttributeValue<GfVec3i>(_tokens->divisionsAttr),
            AttributeValue<GfVec3f>(_tokens->restCagePointsAttr),
            Relationship(_tokens->resolvedBase)
                .TargetedObjects<GfVec3f>(
                    ExecBuiltinComputations->computeValue)
                .InputName(_tokens->basePoints),
            Relationship(_tokens->resolvedCagePoints)
                .TargetedObjects<GfVec3f>(
                    ExecBuiltinComputations->computeValue)
                .InputName(_tokens->cagePoints));

    self.PrimComputation(_tokens->computeMoverStatus)
        .Callback<rigExec::RigExecMoverStatus>(&_BuildMoverStatus)
        .Inputs(
            Computation<RigExecMoverParameters>(
                _tokens->computeMoverParameters)
                .Required(),
            Computation<SdfPath>(ExecBuiltinComputations->computePath)
                .InputName(_tokens->moverPath));
}

EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(RigExecSurfaceMover)
{
    self.PrimComputation(_tokens->computeMoverParameters)
        .Callback<RigExecMoverParameters>(&_BuildSurfaceMoverParameters)
        .Inputs(
            AttributeValue<bool>(_tokens->inputsEnabled),
            Relationship(_tokens->resolvedSurfacePoints)
                .TargetedObjects<GfVec3f>(
                    ExecBuiltinComputations->computeValue)
                .InputName(_tokens->surfacePoints),
            Relationship(_tokens->resolvedTopologyCounts)
                .TargetedObjects<int>(ExecBuiltinComputations->computeValue)
                .InputName(_tokens->topologyCounts),
            Relationship(_tokens->resolvedTopologyIndices)
                .TargetedObjects<int>(ExecBuiltinComputations->computeValue)
                .InputName(_tokens->topologyIndices));

    self.PrimComputation(_tokens->computeMoverStatus)
        .Callback<rigExec::RigExecMoverStatus>(&_BuildMoverStatus)
        .Inputs(
            Computation<RigExecMoverParameters>(
                _tokens->computeMoverParameters)
                .Required(),
            Computation<SdfPath>(ExecBuiltinComputations->computePath)
                .InputName(_tokens->moverPath));
}

EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(RigExecCurveMover)
{
    self.PrimComputation(_tokens->computeMoverParameters)
        .Callback<RigExecMoverParameters>(&_BuildCurveMoverParameters)
        .Inputs(
            AttributeValue<bool>(_tokens->inputsEnabled),
            AttributeValue<TfToken>(_tokens->modeAttr),
            Relationship(_tokens->resolvedBindCoords)
                .TargetedObjects<GfVec2f>(
                    ExecBuiltinComputations->computeValue)
                .InputName(_tokens->bindCoords),
            Relationship(_tokens->resolvedDriverFrames)
                .TargetedObjects<rigExec::RigExecPointFrameArray>(
                    _tokens->computePointFrameArray)
                .InputName(_tokens->driverFrames));

    self.PrimComputation(_tokens->computeMoverStatus)
        .Callback<rigExec::RigExecMoverStatus>(&_BuildMoverStatus)
        .Inputs(
            Computation<RigExecMoverParameters>(
                _tokens->computeMoverParameters)
                .Required(),
            Computation<SdfPath>(ExecBuiltinComputations->computePath)
                .InputName(_tokens->moverPath));
}

// ---------------------------------------------------------------------------
// Operation/type-specific Phase 3 property applications (spec §4.1, §7.2):
// each host's frozen signature consumes the preceding exact-typed vector,
// the realizing mover's computeMoverParameters, and computeMoverStatus.
// ---------------------------------------------------------------------------

#define RIGEXEC_REGISTER_ARRAY_HOST(HostSchema, KernelFn)                    \
    EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(HostSchema)                        \
    {                                                                        \
        self.AttributeExpression(_tokens->outputsExpression)                 \
            .Callback<GfVec3f>(&KernelFn)                                    \
            .Inputs(                                                         \
                Connections<GfVec3f>(ExecBuiltinComputations->computeValue)  \
                    .InputName(_tokens->previous)                            \
                    .Required(),                                             \
                Prim().Relationship(_tokens->moverRel)                       \
                    .TargetedObjects<RigExecMoverParameters>(                \
                        _tokens->computeMoverParameters)                     \
                    .InputName(_tokens->parameters)                          \
                    .Required(),                                             \
                Prim().Relationship(_tokens->moverRel)                       \
                    .TargetedObjects<rigExec::RigExecMoverStatus>(           \
                        _tokens->computeMoverStatus)                         \
                    .InputName(_tokens->status)                              \
                    .Required());                                            \
    }

// Synthesized derived-maintenance hosts (spec §7.6 revised): the host is
// self-realized — the compiler points rigExec:mover at the host itself,
// so the standard rel wiring resolves the host-owned parameter/status
// computations declared here.
#define RIGEXEC_REGISTER_DERIVED_HOST(HostSchema, KernelFn, ParamsFn)        \
    EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(HostSchema)                        \
    {                                                                        \
        self.AttributeExpression(_tokens->outputsExpression)                 \
            .Callback<GfVec3f>(&KernelFn)                                    \
            .Inputs(                                                         \
                Connections<GfVec3f>(ExecBuiltinComputations->computeValue)  \
                    .InputName(_tokens->previous)                            \
                    .Required(),                                             \
                Prim().Relationship(_tokens->moverRel)                       \
                    .TargetedObjects<RigExecMoverParameters>(                \
                        _tokens->computeMoverParameters)                     \
                    .InputName(_tokens->parameters)                          \
                    .Required(),                                             \
                Prim().Relationship(_tokens->moverRel)                       \
                    .TargetedObjects<rigExec::RigExecMoverStatus>(           \
                        _tokens->computeMoverStatus)                         \
                    .InputName(_tokens->status)                              \
                    .Required());                                            \
                                                                             \
        self.PrimComputation(_tokens->computeMoverParameters)                \
            .Callback<RigExecMoverParameters>(&ParamsFn)                     \
            .Inputs(                                                         \
                Relationship(_tokens->resolvedFinalPoints)                   \
                    .TargetedObjects<GfVec3f>(                               \
                        ExecBuiltinComputations->computeValue)               \
                    .InputName(_tokens->finalPoints),                        \
                Relationship(_tokens->resolvedTopologyCounts)                \
                    .TargetedObjects<int>(                                   \
                        ExecBuiltinComputations->computeValue)               \
                    .InputName(_tokens->topologyCounts),                     \
                Relationship(_tokens->resolvedTopologyIndices)               \
                    .TargetedObjects<int>(                                   \
                        ExecBuiltinComputations->computeValue)               \
                    .InputName(_tokens->topologyIndices),                    \
                Relationship(_tokens->resolvedWidths)                        \
                    .TargetedObjects<float>(                                 \
                        ExecBuiltinComputations->computeValue)               \
                    .InputName(_tokens->widthsInput));                       \
                                                                             \
        self.PrimComputation(_tokens->computeMoverStatus)                    \
            .Callback<rigExec::RigExecMoverStatus>(&_BuildMoverStatus)       \
            .Inputs(                                                         \
                Computation<RigExecMoverParameters>(                         \
                    _tokens->computeMoverParameters)                         \
                    .Required(),                                             \
                Computation<SdfPath>(ExecBuiltinComputations->computePath)   \
                    .InputName(_tokens->moverPath));                         \
    }


#undef RIGEXEC_REGISTER_DERIVED_HOST
#undef RIGEXEC_REGISTER_ARRAY_HOST
