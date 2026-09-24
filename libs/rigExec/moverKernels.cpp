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
#include "frameExtraction.h"
#include "solverKernels.h"
#include "weightPackets.h"
#include "movers/moverExecCommon.h"

#include "rigExecMath/pointFrame.h"
#include "rigExecMath/envelope.h"
#include "rigExecMath/geometryKernels.h"
#include "rigExecMath/simdKernels.h"
#include "rigExecMath/solvers.h"
#include "rigExecMath/weightFields.h"

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
    ((inputsDefaultWeight, "inputs:defaultWeight"))
    ((baseWeight, "rigExec:baseWeight"))
    ((inputsDriver, "inputs:driver"))
    ((inputsScale, "inputs:scale"))
    ((inputsBias, "inputs:bias"))
    ((activation, "rigExec:activation"))
    ((targetPoints, "rigExec:targetPoints"))
    ((samples, "rigExec:samples"))
    ((inputsWeight, "inputs:weight"))
    ((deltaSpace, "rigExec:deltaSpace"))
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
    ((joints, "rigExec:joints"))
    (jointRests)
    (computeRestFrame)
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

    // Volumetric weight objects (spec §4.1 volumetric extension).
    (computeFalloffLut)
    (falloffLut)
    (weightTargetPoints)
    (sampleSourcePoints)
    (curvePoints)
    (inputPackets)
    (selfMatrix)
    ((weightTargetRel, "rigExec:weightTarget"))
    ((sampleSourceRel, "rigExec:sampleSource"))
    ((curveRel, "rigExec:curve"))
    ((inputWeightsRel, "rigExec:inputWeights"))
    ((falloffMin, "inputs:falloffMin"))
    ((falloffMax, "inputs:falloffMax"))
    ((inputsInvert, "inputs:invert"))
    ((inputsScaleX, "inputs:scaleX"))
    ((inputsScaleY, "inputs:scaleY"))
    ((inputsScaleZ, "inputs:scaleZ"))
    ((inputsExtentU, "inputs:extentU"))
    ((inputsExtentV, "inputs:extentV"))
    ((planeAxisAttr, "rigExec:planeAxis"))
    ((planeBoundsAttr, "rigExec:planeBounds"))
    ((combineModeAttr, "rigExec:combineMode"))
    ((samplePhaseAttr, "rigExec:samplePhase"))
);

namespace {

// ---------------------------------------------------------------------------
// Weight objects (spec §4.1).
//
// The packet arithmetic lives in weightPackets.h, not here: the baked
// evaluation program has to publish the identical packet from the values
// it bound at compile time, and the only way to guarantee that is one
// definition called from both sides. What is left below is the adapter --
// it turns a VdfContext into the plain values those kernels take, and
// nothing else.
//
// Absent inputs are defaulted HERE rather than inside the kernels,
// because exec distinguishes "the input has no value" from "the attribute
// is authored empty" and a kernel that collapsed the two would accept a
// rig exec rejects.
// ---------------------------------------------------------------------------

// Collects a vectorized GfVec3f input into a plain vector.
std::vector<GfVec3f>
_CollectPoints(const VdfContext &ctx, const TfToken &input)
{
    std::vector<GfVec3f> points;
    VdfReadIterator<GfVec3f> it(ctx, input);
    points.reserve(it.ComputeSize());
    for (; !it.IsAtEnd(); ++it) {
        points.push_back(*it);
    }
    return points;
}

// The same for a vectorized scalar input (rigExec:values, rigExec:indices).
template <typename T>
std::vector<T>
_CollectScalars(const VdfContext &ctx, const TfToken &input)
{
    std::vector<T> values;
    VdfReadIterator<T> it(ctx, input);
    values.reserve(it.ComputeSize());
    for (; !it.IsAtEnd(); ++it) {
        values.push_back(*it);
    }
    return values;
}

float
_Scalar(const VdfContext &ctx, const TfToken &input, float fallback)
{
    const float *v = ctx.GetInputValuePtr<float>(input);
    return v ? *v : fallback;
}

RigExecWeightPacket
_BuildStaticWeightPacket(const VdfContext &ctx)
{
    static const TfToken constant("constant");
    static const TfToken strict("strict");
    rigExec::RigExecStaticWeightInputs inputs;
    const TfToken *repr =
        ctx.GetInputValuePtr<TfToken>(_tokens->representation);
    const TfToken *policy = ctx.GetInputValuePtr<TfToken>(_tokens->rangePolicy);
    inputs.representation = repr ? *repr : constant;
    inputs.rangePolicy = policy ? *policy : strict;
    const float *def = ctx.GetInputValuePtr<float>(_tokens->defaultWeight);
    inputs.defaultWeight = def ? *def : 0.0f;
    inputs.values = _CollectScalars<float>(ctx, _tokens->values);
    // rigExec:indices pairs with the values in the sparse representation
    // and means nothing in the other two, which is why the callback this
    // replaced never dereferenced its iterator outside that arm either.
    if (inputs.representation == "sparse") {
        inputs.indices = _CollectScalars<int>(ctx, _tokens->indices);
    }
    return rigExec::RigExecBuildStaticWeightPacket(inputs);
}

RigExecWeightPacket
_BuildDynamicWeightPacket(const VdfContext &ctx)
{
    static const TfToken constant("constant");
    static const TfToken strict("strict");
    rigExec::RigExecDynamicWeightInputs inputs;
    const TfToken *repr =
        ctx.GetInputValuePtr<TfToken>(_tokens->representation);
    const TfToken *policy = ctx.GetInputValuePtr<TfToken>(_tokens->rangePolicy);
    inputs.representation = repr ? *repr : constant;
    inputs.rangePolicy = policy ? *policy : strict;
    inputs.driver = _Scalar(ctx, _tokens->inputsDriver, 1.0f);
    inputs.scale = _Scalar(ctx, _tokens->inputsScale, 1.0f);
    inputs.bias = _Scalar(ctx, _tokens->inputsBias, 0.0f);
    return rigExec::RigExecBuildDynamicWeightPacket(
        inputs, ctx.GetInputValuePtr<RigExecWeightPacket>(_tokens->basePacket));
}

// ---------------------------------------------------------------------------
// Volumetric weight objects (spec §4.1 volumetric extension).
//
// The three shapes share a prologue (placement, band, sampled points) and
// differ only in the extra inputs their distance function needs, so the
// common half is read once here and each callback adds its own.
// ---------------------------------------------------------------------------

// Reads the band, the invert/strength pair, and the baked remap.
rigExec::RigExecFalloffParams
_ReadFalloffParams(const VdfContext &ctx)
{
    rigExec::RigExecFalloffParams params;
    params.falloffMin = _Scalar(ctx, _tokens->falloffMin, 0.0f);
    params.falloffMax = _Scalar(ctx, _tokens->falloffMax, 1.0f);
    params.invert = _Scalar(ctx, _tokens->inputsInvert, 0.0f);
    params.strength = _Scalar(ctx, _tokens->strengthAttr, 1.0f);
    if (const rigExec::RigExecFalloffLut *lut =
            ctx.GetInputValuePtr<rigExec::RigExecFalloffLut>(
                _tokens->falloffLut)) {
        params.curve = lut->samples;
    }
    return params;
}

// Per-axis divisors. Whether they describe a volume at all is the
// kernel's judgement (see RigExecBuildVolumeWeightPacket); this reads.
void
_ReadAxisScales(const VdfContext &ctx, GfVec3f *scales)
{
    (*scales)[0] = _Scalar(ctx, _tokens->inputsScaleX, 1.0f);
    (*scales)[1] = _Scalar(ctx, _tokens->inputsScaleY, 1.0f);
    (*scales)[2] = _Scalar(ctx, _tokens->inputsScaleZ, 1.0f);
}

// The inputs every volumetric shape reads, LESS the point arrays.
//
// The points are a whole mesh, so they are gathered separately, after
// RigExecVolumeWeightCanBuild has said the volume can produce a field at
// all -- the callbacks this replaced checked the placement before
// touching a single point for the same reason.
rigExec::RigExecVolumeWeightInputs
_ReadVolumeWeightInputs(const VdfContext &ctx)
{
    static const TfToken dense("dense");
    static const TfToken clamp("clamp");
    rigExec::RigExecVolumeWeightInputs inputs;
    const TfToken *repr =
        ctx.GetInputValuePtr<TfToken>(_tokens->representation);
    const TfToken *policy = ctx.GetInputValuePtr<TfToken>(_tokens->rangePolicy);
    inputs.representation = repr ? *repr : dense;
    inputs.rangePolicy = policy ? *policy : clamp;
    // The placement comes from computePointFrame, NOT computeMatrix; see
    // RigExecRigidWorldToLocal for why that distinction is load bearing.
    if (const RigExecPointFrame *posed =
            ctx.GetInputValuePtr<RigExecPointFrame>(_tokens->selfMatrix)) {
        inputs.placement = *posed;
        inputs.hasPlacement = true;
    }
    inputs.params = _ReadFalloffParams(ctx);
    return inputs;
}

// The weighted domain and, when authored, the shape it is sampled on.
void
_ReadVolumeWeightPoints(
    const VdfContext &ctx, rigExec::RigExecVolumeWeightInputs *inputs)
{
    inputs->targetPoints = _CollectPoints(ctx, _tokens->weightTargetPoints);
    inputs->samplePoints = _CollectPoints(ctx, _tokens->sampleSourcePoints);
}

RigExecWeightPacket
_BuildSphereWeightPacket(const VdfContext &ctx)
{
    static const TfToken sphereType("RigExecSphereWeight");
    rigExec::RigExecVolumeWeightInputs inputs = _ReadVolumeWeightInputs(ctx);
    _ReadAxisScales(ctx, &inputs.scales);
    if (rigExec::RigExecVolumeWeightCanBuild(sphereType, inputs)) {
        _ReadVolumeWeightPoints(ctx, &inputs);
    }
    return rigExec::RigExecBuildVolumeWeightPacket(sphereType, inputs);
}

RigExecWeightPacket
_BuildPlaneWeightPacket(const VdfContext &ctx)
{
    static const TfToken planeType("RigExecPlaneWeight");
    static const TfToken yAxis("y");
    static const TfToken unbounded("unbounded");
    static const TfToken bounded("bounded");
    rigExec::RigExecVolumeWeightInputs inputs = _ReadVolumeWeightInputs(ctx);
    const TfToken *axis = ctx.GetInputValuePtr<TfToken>(_tokens->planeAxisAttr);
    const TfToken *bounds =
        ctx.GetInputValuePtr<TfToken>(_tokens->planeBoundsAttr);
    inputs.planeAxis = axis ? *axis : yAxis;
    inputs.planeBounds = bounds ? *bounds : unbounded;
    // Only the bounded arm consults the extents; an unbounded plane is an
    // infinite half-space gradient and a bad extent on one is a
    // legibility problem, not a reason to invalidate the rig.
    if (inputs.planeBounds == bounded) {
        inputs.extentU = _Scalar(ctx, _tokens->inputsExtentU, 1.0f);
        inputs.extentV = _Scalar(ctx, _tokens->inputsExtentV, 1.0f);
    }
    if (rigExec::RigExecVolumeWeightCanBuild(planeType, inputs)) {
        _ReadVolumeWeightPoints(ctx, &inputs);
    }
    return rigExec::RigExecBuildVolumeWeightPacket(planeType, inputs);
}

RigExecWeightPacket
_BuildCurveWeightPacket(const VdfContext &ctx)
{
    static const TfToken curveType("RigExecCurveWeight");
    rigExec::RigExecVolumeWeightInputs inputs = _ReadVolumeWeightInputs(ctx);
    _ReadAxisScales(ctx, &inputs.scales);
    if (rigExec::RigExecVolumeWeightCanBuild(curveType, inputs)) {
        _ReadVolumeWeightPoints(ctx, &inputs);
        inputs.curvePoints = _CollectPoints(ctx, _tokens->curvePoints);
    }
    return rigExec::RigExecBuildVolumeWeightPacket(curveType, inputs);
}

RigExecWeightPacket
_BuildCombineWeightPacket(const VdfContext &ctx)
{
    static const TfToken dense("dense");
    static const TfToken clamp("clamp");
    static const TfToken multiply("multiply");
    const TfToken *repr =
        ctx.GetInputValuePtr<TfToken>(_tokens->representation);
    const TfToken *policy = ctx.GetInputValuePtr<TfToken>(_tokens->rangePolicy);
    const TfToken *mode =
        ctx.GetInputValuePtr<TfToken>(_tokens->combineModeAttr);
    // Authored target order is preserved: subtract and overlay are order
    // dependent by design (see the schema doc).
    std::vector<RigExecWeightPacket> inputs;
    {
        VdfReadIterator<RigExecWeightPacket> it(ctx, _tokens->inputPackets);
        for (; !it.IsAtEnd(); ++it) {
            inputs.push_back(*it);
        }
    }
    // rigExec:weightTarget is only ever read for its SIZE, when no input
    // is dense enough to carry the cardinality itself -- so take the size
    // and never the points.
    VdfReadIterator<GfVec3f> target(ctx, _tokens->weightTargetPoints);
    return rigExec::RigExecBuildCombineWeightPacket(
        repr ? *repr : dense, policy ? *policy : clamp,
        mode ? *mode : multiply, inputs, target.ComputeSize(),
        _Scalar(ctx, _tokens->strengthAttr, 1.0f),
        _Scalar(ctx, _tokens->inputsInvert, 0.0f));
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

// The matrix mover builder and registration live in
// movers/matrixMover.cpp with the rest of the matrix mover.

// The blend-shape mover builder and registration live in
// movers/blendShapeMover.cpp with the rest of the blend-shape mover.

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
    // Resolve the common envelope first so a cardinality failure passes
    // through before any output is written.
    std::vector<float> weights(count);
    if (!params->weights.ResolveAll(count, &weights)) {
        passThrough();
        return;
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
    std::vector<float> envelope;
    if (!params->weights.ResolveAll(count, &envelope)) {
        passThrough();
        return;
    }
    auto out = VdfReadWriteIterator<GfVec3f>::Allocate(ctx, count);
    size_t i = 0;
    for (; !previous.IsAtEnd(); ++previous, ++out, ++i) {
        const GfVec3f preceding = *previous;
        *out = rigExec::RigExecBlendEnvelope(
            preceding, preceding + params->blendDeltas[i], envelope[i]);
    }
}

// The smooth mover builder and registration live in
// movers/smoothMover.cpp with the rest of the smooth mover.

// The volume-correct mover builder and registration live in
// movers/volumeCorrectMover.cpp with the rest of the volume-correct mover.

// The lattice mover builder and registration live in
// movers/latticeMover.cpp with the rest of the lattice mover.

// The surface mover builder and registration live in
// movers/surfaceMover.cpp with the rest of the surface mover.

// The curve mover builder and registration live in
// movers/curveMover.cpp with the rest of the curve mover.

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
    const std::vector<GfVec3f> preceding = scratch;
    if (!kernel(*params, &scratch)) {
        passThrough();
        return;
    }
    if (scratch.size() != preceding.size()) {
        passThrough();
        return;
    }
    std::vector<float> envelope;
    if (!params->weights.ResolveAll(scratch.size(), &envelope)) {
        passThrough();
        return;
    }
    for (size_t i = 0; i < scratch.size(); ++i) {
        scratch[i] = rigExec::RigExecBlendEnvelope(
            preceding[i], scratch[i], envelope[i]);
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
//
// The sampling itself is RigExecSampleRibbonFrames, which the baked program
// calls as well; only the reads below are exec's.
RigExecPointFrameArray
_ComputeRibbonFrames(const VdfContext &ctx)
{
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
    // The joints' rest references, in rigExec:joints order, and which of
    // them a pose step BELOW the ribbon wrote (RigExecPointFrameLiveRest).
    // A live one re-bases its sample so the step's frame is carried through
    // the ribbon instead of replaced (spec 4.2); every other sample keeps the
    // rest curve, which is what leaves an unstacked ribbon untouched.
    std::vector<std::array<GfVec3d, 4>> jointRests;
    std::vector<bool> jointRestLive;
    {
        VdfReadIterator<RigExecPointFrame> it(ctx, _tokens->jointRests);
        for (; !it.IsAtEnd(); ++it) {
            jointRests.push_back((*it).points);
            jointRestLive.push_back(
                ((*it).flags & rigExec::RigExecPointFrameLiveRest) != 0);
        }
    }
    return rigExec::RigExecSampleRibbonFrames(posed, rest, sampleCount,
                                              jointRests, jointRestLive);
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
// Volumetric weight objects (spec §4.1 volumetric extension).
//
// computeFalloffLut is a STUB that returns an empty (linear) table. The
// real bake -- a named analytic profile, or the resampled spline authored
// on rigExec:falloffCurve -- arrives as a RigExecValueOverride supplied
// once per binding epoch by the evaluator, because exec has no accessor
// for an attribute's spline (see RigExecFalloffLut in types.h). This is
// the same shape the ribbon uses for its driver-curve points, and for the
// same class of reason.
//
// The shared inputs are spelled out per concrete type rather than shared
// through a macro over the abstract base: the three shapes genuinely
// differ in what they read, and a registration that lied about its inputs
// would silently miss an invalidation.
// ---------------------------------------------------------------------------

#define RIGEXEC_VOLUME_WEIGHT_COMMON_INPUTS                                  \
    AttributeValue<TfToken>(_tokens->representation),                        \
        AttributeValue<TfToken>(_tokens->rangePolicy),                       \
        AttributeValue<float>(_tokens->falloffMin),                          \
        AttributeValue<float>(_tokens->falloffMax),                          \
        AttributeValue<float>(_tokens->inputsInvert),                        \
        AttributeValue<float>(_tokens->strengthAttr),                        \
        Computation<RigExecPointFrame>(_tokens->computePointFrame)           \
            .InputName(_tokens->selfMatrix)                                  \
            .Required(),                                                     \
        Computation<rigExec::RigExecFalloffLut>(_tokens->computeFalloffLut)  \
            .InputName(_tokens->falloffLut),                                 \
        Relationship(_tokens->weightTargetRel)                               \
            .TargetedObjects<GfVec3f>(ExecBuiltinComputations->computeValue) \
            .InputName(_tokens->weightTargetPoints),                         \
        Relationship(_tokens->sampleSourceRel)                               \
            .TargetedObjects<GfVec3f>(ExecBuiltinComputations->computeValue) \
            .InputName(_tokens->sampleSourcePoints)

#define RIGEXEC_VOLUME_WEIGHT_AXIS_SCALES                                    \
    AttributeValue<float>(_tokens->inputsScaleX),                            \
        AttributeValue<float>(_tokens->inputsScaleY),                        \
        AttributeValue<float>(_tokens->inputsScaleZ)

// The stub the evaluator overrides. Registered per concrete type for the
// same reason the packets are.
#define RIGEXEC_REGISTER_FALLOFF_LUT_STUB                                    \
    self.PrimComputation(_tokens->computeFalloffLut)                         \
        .Callback<rigExec::RigExecFalloffLut>(                               \
            +[](const VdfContext &) { return rigExec::RigExecFalloffLut(); })

EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(RigExecSphereWeight)
{
    RIGEXEC_REGISTER_FALLOFF_LUT_STUB;

    self.PrimComputation(_tokens->computeWeightPacket)
        .Callback<RigExecWeightPacket>(&_BuildSphereWeightPacket)
        .Inputs(
            RIGEXEC_VOLUME_WEIGHT_COMMON_INPUTS,
            RIGEXEC_VOLUME_WEIGHT_AXIS_SCALES);
}

EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(RigExecPlaneWeight)
{
    RIGEXEC_REGISTER_FALLOFF_LUT_STUB;

    self.PrimComputation(_tokens->computeWeightPacket)
        .Callback<RigExecWeightPacket>(&_BuildPlaneWeightPacket)
        .Inputs(
            RIGEXEC_VOLUME_WEIGHT_COMMON_INPUTS,
            AttributeValue<TfToken>(_tokens->planeAxisAttr),
            AttributeValue<TfToken>(_tokens->planeBoundsAttr),
            AttributeValue<float>(_tokens->inputsExtentU),
            AttributeValue<float>(_tokens->inputsExtentV));
}

EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(RigExecCurveWeight)
{
    RIGEXEC_REGISTER_FALLOFF_LUT_STUB;

    self.PrimComputation(_tokens->computeWeightPacket)
        .Callback<RigExecWeightPacket>(&_BuildCurveWeightPacket)
        .Inputs(
            RIGEXEC_VOLUME_WEIGHT_COMMON_INPUTS,
            RIGEXEC_VOLUME_WEIGHT_AXIS_SCALES,
            Relationship(_tokens->curveRel)
                .TargetedObjects<GfVec3f>(
                    ExecBuiltinComputations->computeValue)
                .InputName(_tokens->curvePoints));
}

#undef RIGEXEC_REGISTER_FALLOFF_LUT_STUB
#undef RIGEXEC_VOLUME_WEIGHT_AXIS_SCALES
#undef RIGEXEC_VOLUME_WEIGHT_COMMON_INPUTS

EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(RigExecCombineWeight)
{
    self.PrimComputation(_tokens->computeWeightPacket)
        .Callback<RigExecWeightPacket>(&_BuildCombineWeightPacket)
        .Inputs(
            AttributeValue<TfToken>(_tokens->representation),
            AttributeValue<TfToken>(_tokens->rangePolicy),
            AttributeValue<TfToken>(_tokens->combineModeAttr),
            AttributeValue<float>(_tokens->strengthAttr),
            AttributeValue<float>(_tokens->inputsInvert),
            Relationship(_tokens->inputWeightsRel)
                .TargetedObjects<RigExecWeightPacket>(
                    _tokens->computeWeightPacket)
                .InputName(_tokens->inputPackets),
            // Only ever read for its SIZE, when no input is dense enough
            // to carry the cardinality itself.
            Relationship(_tokens->weightTargetRel)
                .TargetedObjects<GfVec3f>(
                    ExecBuiltinComputations->computeValue)
                .InputName(_tokens->weightTargetPoints));
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
            Relationship(_tokens->joints)
                .TargetedObjects<RigExecPointFrame>(_tokens->computeRestFrame)
                .InputName(_tokens->jointRests),
            Computation<RigExecPointsPacket>(_tokens->computeRestDriverPoints)
                .InputName(_tokens->restDriver),
            Computation<RigExecPointsPacket>(_tokens->computeDriverPoints)
                .InputName(_tokens->driverPoints));
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
            .Callback<rigExec::RigExecMoverStatus>(&rigExec::RigExecMoverBuildStatus)       \
            .Inputs(                                                         \
                Computation<RigExecMoverParameters>(                         \
                    _tokens->computeMoverParameters)                         \
                    .Required(),                                             \
                Computation<SdfPath>(ExecBuiltinComputations->computePath)   \
                    .InputName(_tokens->moverPath));                         \
    }


#undef RIGEXEC_REGISTER_DERIVED_HOST
#undef RIGEXEC_REGISTER_ARRAY_HOST
