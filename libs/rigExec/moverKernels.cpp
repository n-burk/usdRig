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

// ---------------------------------------------------------------------------
// Volumetric weight objects (spec §4.1 volumetric extension).
//
// One shared body: read the placement and the band, pick a distance
// function, remap, and publish a dense packet. Only the distance function
// and the extra inputs it needs differ between sphere, plane, and curve.
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

float
_Scalar(const VdfContext &ctx, const TfToken &input, float fallback)
{
    const float *v = ctx.GetInputValuePtr<float>(input);
    return v ? *v : fallback;
}

// The rigid placement of a volume weight, inverted.
//
// Built from computePointFrame, NOT computeMatrix. computeMatrix is the
// rest->posed target-local map -- a DEFORMATION, which is exactly what a
// matrix mover wants and exactly what a placement is not. An unanimated
// volume has posed == rest, so its computeMatrix is the identity, and a
// volume authored at rest:space Y=5 would generate its field about the
// origin. The posed frame is the absolute location, so the placement is
// the map taking the identity landmarks to it.
//
// Scale and shear are then REMOVED rather than inverted along with the
// rest: the guide a viewer draws is built from the orthonormalized posed
// frame (guides are rigid), so a scale left in the matrix would deform
// the field without deforming the drawn volume, and the artist would be
// painting with a shape they cannot see. inputs:scaleX/Y/Z is the sole
// authority on anisotropy, and it is applied per axis below.
bool
_RigidWorldToLocal(const VdfContext &ctx, GfMatrix4d *result)
{
    const RigExecPointFrame *posed =
        ctx.GetInputValuePtr<RigExecPointFrame>(_tokens->selfMatrix);
    if (!posed || !posed->IsValid() || posed->IsDegenerate()) {
        return false;
    }
    GfMatrix4d placement(1.0);
    if (!rigExec::RigExecPointsToMatrix(
            rigExec::RigExecIdentityLandmarks(), posed->points, &placement)) {
        return false;
    }
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (!std::isfinite(placement[i][j])) {
                return false;
            }
        }
    }
    GfMatrix4d rigid = placement.RemoveScaleShear();
    const double det = rigid.GetDeterminant();
    if (!std::isfinite(det) || std::abs(det) < 1e-12) {
        return false;
    }
    *result = rigid.GetInverse();
    return true;
}

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

// Per-axis divisors. Non-positive or non-finite collapses the volume, so
// it invalidates the packet rather than dividing by zero.
bool
_ReadScales(const VdfContext &ctx, GfVec3f *scales)
{
    (*scales)[0] = _Scalar(ctx, _tokens->inputsScaleX, 1.0f);
    (*scales)[1] = _Scalar(ctx, _tokens->inputsScaleY, 1.0f);
    (*scales)[2] = _Scalar(ctx, _tokens->inputsScaleZ, 1.0f);
    for (int a = 0; a < 3; ++a) {
        if (!std::isfinite((*scales)[a]) || (*scales)[a] <= 0.0f) {
            return false;
        }
    }
    return true;
}

// The shared prologue: structural tokens, placement, band, and the points
// the field is measured over. Returns false when the packet is invalid.
bool
_BeginVolumeWeight(
    const VdfContext &ctx,
    RigExecWeightPacket *packet,
    GfMatrix4d *worldToLocal,
    rigExec::RigExecFalloffParams *params,
    std::vector<GfVec3f> *points)
{
    static const TfToken dense("dense");
    static const TfToken clamp("clamp");
    const TfToken *repr =
        ctx.GetInputValuePtr<TfToken>(_tokens->representation);
    const TfToken *policy = ctx.GetInputValuePtr<TfToken>(_tokens->rangePolicy);
    packet->representation = repr ? *repr : dense;
    packet->rangePolicy = policy ? *policy : clamp;
    // A generated field has a value at every element, so dense is the only
    // representation it can honestly publish. Rejected, never coerced.
    if (packet->representation != "dense") {
        return false;
    }
    if (packet->rangePolicy != "strict" && packet->rangePolicy != "clamp") {
        return false;
    }
    if (!_RigidWorldToLocal(ctx, worldToLocal)) {
        return false;
    }
    *params = _ReadFalloffParams(ctx);

    // rigExec:sampleSource, when authored, replaces the weightTarget for
    // SAMPLING only -- the weighted domain stays the target, so the two
    // must still agree in cardinality.
    const std::vector<GfVec3f> target =
        _CollectPoints(ctx, _tokens->weightTargetPoints);
    if (target.empty()) {
        return false;
    }
    std::vector<GfVec3f> source =
        _CollectPoints(ctx, _tokens->sampleSourcePoints);
    if (!source.empty() && source.size() != target.size()) {
        return false;  // a reference shape of another cardinality is a lie
    }
    *points = source.empty() ? target : std::move(source);
    return true;
}

// The shared epilogue: range policy over the generated field.
bool
_FinishVolumeWeight(
    RigExecWeightPacket *packet, std::vector<float> *weights)
{
    for (float &w : *weights) {
        if (!_ApplyRangePolicy(packet->rangePolicy, &w)) {
            return false;
        }
    }
    packet->values = std::move(*weights);
    packet->defaultWeight = 0.0f;  // canonical dense default
    packet->valid = true;
    return true;
}

// Divides local coordinates by the per-axis scales, turning the sphere's
// iso-surfaces into ellipsoids and the curve's tube into an elliptical
// one. Folded into the matrix so the hot loop stays a single transform.
GfMatrix4d
_ApplyAxisScales(const GfMatrix4d &worldToLocal, const GfVec3f &scales)
{
    GfMatrix4d divide(1.0);
    divide.SetScale(GfVec3d(1.0 / double(scales[0]), 1.0 / double(scales[1]),
                            1.0 / double(scales[2])));
    return worldToLocal * divide;
}

RigExecWeightPacket
_BuildSphereWeightPacket(const VdfContext &ctx)
{
    RigExecWeightPacket packet;
    GfMatrix4d worldToLocal;
    rigExec::RigExecFalloffParams params;
    std::vector<GfVec3f> points;
    if (!_BeginVolumeWeight(ctx, &packet, &worldToLocal, &params, &points)) {
        return packet;
    }
    GfVec3f scales;
    if (!_ReadScales(ctx, &scales)) {
        return packet;
    }
    std::vector<float> weights;
    rigExec::RigExecSphereWeightField(
        points, _ApplyAxisScales(worldToLocal, scales), params, &weights);
    if (!_FinishVolumeWeight(&packet, &weights)) {
        return RigExecWeightPacket();
    }
    return packet;
}

RigExecWeightPacket
_BuildPlaneWeightPacket(const VdfContext &ctx)
{
    RigExecWeightPacket packet;
    GfMatrix4d worldToLocal;
    rigExec::RigExecFalloffParams params;
    std::vector<GfVec3f> points;
    if (!_BeginVolumeWeight(ctx, &packet, &worldToLocal, &params, &points)) {
        return packet;
    }
    static const TfToken yAxis("y");
    const TfToken *axisToken =
        ctx.GetInputValuePtr<TfToken>(_tokens->planeAxisAttr);
    const TfToken axis = axisToken ? *axisToken : yAxis;
    int axisIndex;
    if (axis == "x") {
        axisIndex = 0;
    } else if (axis == "y") {
        axisIndex = 1;
    } else if (axis == "z") {
        axisIndex = 2;
    } else {
        return packet;  // unknown structural token: rejected, not coerced
    }

    // Bounded clips the field to the in-plane rectangle; unbounded is
    // the infinite half-space gradient. The extents are read ONLY in the
    // bounded arm: unbounded does not consult them for the field (they
    // still size the drawn guide), so a bad extent there is a legibility
    // problem, not a reason to invalidate the whole rig.
    static const TfToken unbounded("unbounded");
    static const TfToken bounded("bounded");
    const TfToken *boundsToken =
        ctx.GetInputValuePtr<TfToken>(_tokens->planeBoundsAttr);
    const TfToken boundsMode = boundsToken ? *boundsToken : unbounded;
    rigExec::RigExecPlaneBounds extent;
    const rigExec::RigExecPlaneBounds *extentPtr = nullptr;
    if (boundsMode == bounded) {
        extent.extentU = _Scalar(ctx, _tokens->inputsExtentU, 1.0f);
        extent.extentV = _Scalar(ctx, _tokens->inputsExtentV, 1.0f);
        for (const float e : {extent.extentU, extent.extentV}) {
            if (!std::isfinite(e) || e <= 0.0f) {
                return packet;  // no such rectangle; same rule as scales
            }
        }
        extentPtr = &extent;
    } else if (boundsMode != unbounded) {
        return packet;  // unknown structural token: rejected, not coerced
    }

    std::vector<float> weights;
    rigExec::RigExecPlaneWeightField(
        points, worldToLocal, axisIndex, params, &weights, extentPtr);
    if (!_FinishVolumeWeight(&packet, &weights)) {
        return RigExecWeightPacket();
    }
    return packet;
}

RigExecWeightPacket
_BuildCurveWeightPacket(const VdfContext &ctx)
{
    RigExecWeightPacket packet;
    GfMatrix4d worldToLocal;
    rigExec::RigExecFalloffParams params;
    std::vector<GfVec3f> points;
    if (!_BeginVolumeWeight(ctx, &packet, &worldToLocal, &params, &points)) {
        return packet;
    }
    GfVec3f scales;
    if (!_ReadScales(ctx, &scales)) {
        return packet;
    }
    const std::vector<GfVec3f> curve =
        _CollectPoints(ctx, _tokens->curvePoints);
    if (curve.empty()) {
        return packet;  // a curve weight with no curve is not a field
    }
    std::vector<float> weights;
    rigExec::RigExecCurveWeightField(
        points, curve, _ApplyAxisScales(worldToLocal, scales), params,
        &weights);
    if (!_FinishVolumeWeight(&packet, &weights)) {
        return RigExecWeightPacket();
    }
    return packet;
}

// Composition. Every input is resolved to a dense field over the same
// element count before folding, so a static sparse paint and a generated
// volume field compose without either knowing about the other.
RigExecWeightPacket
_BuildCombineWeightPacket(const VdfContext &ctx)
{
    RigExecWeightPacket packet;
    static const TfToken dense("dense");
    static const TfToken clamp("clamp");
    const TfToken *repr =
        ctx.GetInputValuePtr<TfToken>(_tokens->representation);
    const TfToken *policy = ctx.GetInputValuePtr<TfToken>(_tokens->rangePolicy);
    packet.representation = repr ? *repr : dense;
    packet.rangePolicy = policy ? *policy : clamp;
    if (packet.representation != "dense") {
        return packet;
    }
    if (packet.rangePolicy != "strict" && packet.rangePolicy != "clamp") {
        return packet;
    }

    static const TfToken multiply("multiply");
    const TfToken *modeToken =
        ctx.GetInputValuePtr<TfToken>(_tokens->combineModeAttr);
    const TfToken modeName = modeToken ? *modeToken : multiply;
    rigExec::RigExecWeightCombine mode;
    if (modeName == "multiply") {
        mode = rigExec::RigExecWeightCombine::Multiply;
    } else if (modeName == "add") {
        mode = rigExec::RigExecWeightCombine::Add;
    } else if (modeName == "subtract") {
        mode = rigExec::RigExecWeightCombine::Subtract;
    } else if (modeName == "max") {
        mode = rigExec::RigExecWeightCombine::Max;
    } else if (modeName == "min") {
        mode = rigExec::RigExecWeightCombine::Min;
    } else if (modeName == "average") {
        mode = rigExec::RigExecWeightCombine::Average;
    } else if (modeName == "overlay") {
        mode = rigExec::RigExecWeightCombine::Overlay;
    } else {
        return packet;
    }

    // Authored target order is preserved: subtract and overlay are order
    // dependent by design (see the schema doc).
    std::vector<RigExecWeightPacket> inputs;
    {
        VdfReadIterator<RigExecWeightPacket> it(ctx, _tokens->inputPackets);
        for (; !it.IsAtEnd(); ++it) {
            inputs.push_back(*it);
        }
    }
    // Element count. A dense input carries it; when every input is
    // CONSTANT -- which is the schema default for an authored weight, so
    // it is not an exotic case -- nothing among the inputs knows the
    // cardinality and the combine has to get it from its own
    // rigExec:weightTarget.
    //
    // Reading the target here is not belt-and-braces: without it a
    // constant-only combine publishes an invalid packet and the mover
    // passes through, while the CPU oracle resolves every constant to
    // `count` values and moves the points. That divergence is a parity
    // mismatch, and it is reachable the first time someone multiplies
    // two freshly created static weights together.
    size_t elementCount = 0;
    for (const RigExecWeightPacket &in : inputs) {
        if (!in.valid) {
            return packet;  // one bad input fails the fold atomically
        }
        if (in.representation == "dense") {
            if (elementCount && in.values.size() != elementCount) {
                return packet;
            }
            elementCount = in.values.size();
        }
    }
    if (!elementCount) {
        VdfReadIterator<GfVec3f> it(ctx, _tokens->weightTargetPoints);
        elementCount = it.ComputeSize();
    }
    if (!elementCount) {
        return packet;
    }

    std::vector<std::vector<float>> fields;
    fields.reserve(inputs.size());
    for (const RigExecWeightPacket &in : inputs) {
        std::vector<float> field(elementCount);
        for (size_t i = 0; i < elementCount; ++i) {
            field[i] = in.Resolve(i, elementCount);
            if (field[i] < 0.0f) {
                return packet;  // cardinality mismatch
            }
        }
        fields.push_back(std::move(field));
    }

    std::vector<float> folded;
    if (!rigExec::RigExecCombineWeightFields(
            mode, fields, elementCount, &folded)) {
        return packet;
    }
    const float strength = _Scalar(ctx, _tokens->strengthAttr, 1.0f);
    const float invert = _Scalar(ctx, _tokens->inputsInvert, 0.0f);
    for (float &w : folded) {
        w = (w + (1.0f - 2.0f * w) * invert) * strength;
        if (!_ApplyRangePolicy(packet.rangePolicy, &w)) {
            return RigExecWeightPacket();
        }
    }
    packet.values = std::move(folded);
    packet.defaultWeight = 0.0f;
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

bool
_SetCommonMoverEnvelope(
    const VdfContext &ctx, RigExecMoverParameters *params)
{
    const RigExecWeightPacket *weights =
        ctx.GetInputValuePtr<RigExecWeightPacket>(_tokens->weightPacket);
    const float *defaultWeight =
        ctx.GetInputValuePtr<float>(_tokens->inputsDefaultWeight);
    params->weights = weights
        ? *weights
        : RigExecWeightPacket::Constant(defaultWeight ? *defaultWeight : 1.0f);
    return params->weights.valid;
}

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
    if (!transform || !_SetCommonMoverEnvelope(ctx, &params)) {
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
    const TfToken *deltaSpace = ctx.GetInputValuePtr<TfToken>(_tokens->deltaSpace);
    if (deltaSpace && *deltaSpace != "target" && *deltaSpace != "surfaceFrame") return params;
    params.blendSurfaceFrame = deltaSpace && *deltaSpace == "surfaceFrame";
    if (params.blendSurfaceFrame) {
        params.restPoints = base;
        for (VdfReadIterator<int> it(ctx, _tokens->topologyCounts); !it.IsAtEnd(); ++it)
            params.topologyCounts.push_back(*it);
        for (VdfReadIterator<int> it(ctx, _tokens->topologyIndices); !it.IsAtEnd(); ++it)
            params.topologyIndices.push_back(*it);
        if (params.topologyCounts.empty()) return params;
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
    // Common MoverAPI envelope. A bound object supersedes the scalar.
    if (!_SetCommonMoverEnvelope(ctx, &params)) {
        return params;
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
    if (!_SetCommonMoverEnvelope(ctx, &params)) {
        return params;
    }
    params.strength = 1.0f;
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
    if (!_SetCommonMoverEnvelope(ctx, &params)) {
        return params;
    }
    params.strength = 1.0f;
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
    if (!_SetCommonMoverEnvelope(ctx, &params)) {
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
    if (!_SetCommonMoverEnvelope(ctx, &params)) {
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
    if (!_SetCommonMoverEnvelope(ctx, &params)) {
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
    return rigExec::RigExecSampleRibbonFrames(posed, rest, sampleCount);
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
// Mover-owned parameter/status computations (spec §4.1): each concrete
// mover schema statically registers computeMoverParameters over its own
// declared attributes/relationships, plus the scalar computeMoverStatus.
// The rigExec:resolved* relationships are authored by the compiler's
// the compiler (spec §12.1 RigExecResolvedMatrix and friends), binding the
// declared read phase to exactly one catalogued provider.
// ---------------------------------------------------------------------------

#define RIGEXEC_MOVER_COMMON_INPUTS                                         \
    AttributeValue<bool>(_tokens->inputsEnabled),                           \
        AttributeValue<float>(_tokens->inputsDefaultWeight),                \
        Relationship(_tokens->weightObjectRel)                              \
            .TargetedObjects<RigExecWeightPacket>(                          \
                _tokens->computeWeightPacket)                               \
            .InputName(_tokens->weightPacket)

EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(RigExecMatrixMover)
{
    self.PrimComputation(_tokens->computeMoverParameters)
        .Callback<RigExecMoverParameters>(&_BuildMatrixMoverParameters)
        .Inputs(
            RIGEXEC_MOVER_COMMON_INPUTS,
            Relationship(_tokens->resolvedTransform)
                .TargetedObjects<GfMatrix4d>(_tokens->computeMatrix)
                .InputName(_tokens->transform));

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
            RIGEXEC_MOVER_COMMON_INPUTS,
            AttributeValue<TfToken>(_tokens->deltaSpace),
            Relationship(_tokens->resolvedTopologyCounts)
                .TargetedObjects<int>(ExecBuiltinComputations->computeValue)
                .InputName(_tokens->topologyCounts),
            Relationship(_tokens->resolvedTopologyIndices)
                .TargetedObjects<int>(ExecBuiltinComputations->computeValue)
                .InputName(_tokens->topologyIndices),
            Relationship(_tokens->resolvedBlendInputs)
                .TargetedObjects<RigExecBlendChannel>(
                    _tokens->computeBlendChannel)
                .InputName(_tokens->blendChannels),
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
            RIGEXEC_MOVER_COMMON_INPUTS,
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
            RIGEXEC_MOVER_COMMON_INPUTS,
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
            RIGEXEC_MOVER_COMMON_INPUTS,
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
            RIGEXEC_MOVER_COMMON_INPUTS,
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
            RIGEXEC_MOVER_COMMON_INPUTS,
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

#undef RIGEXEC_MOVER_COMMON_INPUTS

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
