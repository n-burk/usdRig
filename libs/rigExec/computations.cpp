//
// RigExec OpenExec computation registrations (spec §12.1).
//
// Every semantically addressable transform provider publishes paired
// computePointFrame / computeMatrix. Packed solver boundaries publish the
// aggregate computePointFrameArray; scalar scene-addressable providers
// (joints, views) consume the aggregate and publish scalar frames
// (spec §5.7 universal point extraction invariant).
//
#include "types.h"
#include "frameExtraction.h"

#include "rigExecMath/pointFrame.h"
#include "rigExecMath/solvers.h"

#include "pxr/pxr.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/base/gf/vec4d.h"
#include "pxr/base/tf/staticTokens.h"
#include "pxr/exec/exec/builtinComputations.h"
#include "pxr/exec/exec/registerSchema.h"
#include "pxr/exec/vdf/context.h"
#include "pxr/exec/vdf/readIterator.h"
#include "pxr/exec/vdf/readWriteIterator.h"

#include <algorithm>
#include <cmath>

using rigExec::RigExecPointFrame;
using rigExec::RigExecPointFrameArray;

// EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA opens/closes the pxr namespace
// itself, so this file stays at global scope with a using-directive.
PXR_NAMESPACE_USING_DIRECTIVE

TF_DEFINE_PRIVATE_TOKENS(
    _tokens,

    // Computation names (spec §4.1).
    (computePointFrame)
    (computeMatrix)
    (computeRestFrame)
    (computePointFrameArray)

    // Input names.
    (parentFrame)
    (controlFrames)
    (controlRests)
    (rootFrame)
    (rootRest)
    (effectorFrame)
    (effectorRest)
    (poleFrame)
    (inputAFrames)
    (inputBFrames)
    (startFrame)
    (startRest)
    (endFrame)
    (endRest)

    // Attribute tokens.
    ((controls, "rigExec:controls"))
    ((rootControl, "rigExec:rootControl"))
    ((effectorControl, "rigExec:effectorControl"))
    ((poleControl, "rigExec:poleControl"))
    ((upperLength, "rigExec:upperLength"))
    ((lowerLength, "rigExec:lowerLength"))
    ((preferredBendRadians, "rigExec:preferredBendRadians"))
    ((inputA, "rigExec:inputA"))
    ((inputB, "rigExec:inputB"))
    ((rotationBlend, "rigExec:rotationBlend"))
    ((scaleBlend, "rigExec:scaleBlend"))
    ((startRel, "rigExec:start"))
    ((endRel, "rigExec:end"))
    ((weights, "rigExec:weights"))
    ((count, "rigExec:count"))
    ((inputsWeight, "inputs:weight"))
    ((inputsStretch, "inputs:stretch"))
    ((inputsSoftness, "inputs:softness"))

    // Ir-aligned joint contract (IrXformable mirror).
    ((restSpace, "rest:space"))
    ((posedSpace, "posed:space"))
    ((restTx, "rest:tx"))
    ((restTy, "rest:ty"))
    ((restTz, "rest:tz"))
    ((restRx, "rest:rx"))
    ((restRy, "rest:ry"))
    ((restRz, "rest:rz"))
    ((avarTx, "avars:tx"))
    ((avarTy, "avars:ty"))
    ((avarTz, "avars:tz"))
    ((avarRx, "avars:rx"))
    ((avarRy, "avars:ry"))
    ((avarRz, "avars:rz"))
    ((avarRspin, "avars:rspin"))
    ((avarRotationOrder, "avars:rotationOrder"))
    (posedConnected)
    (parentPosedFrame)
    (parentRestFrame)
    (selfRestFrame)
);

namespace {

std::array<GfVec3d, 4>
_IdentityLandmarks()
{
    return {GfVec3d(0, 0, 0), GfVec3d(1, 0, 0), GfVec3d(0, 1, 0),
            GfVec3d(0, 0, 1)};
}

}  // namespace

// RigExecControl and RigExecJoint both register the shared Ir-xformable
// computations below: animation lives on avars / posed:space, and the rest
// frame comes from rest:space plus the rest avars. There is no landmark-based
// provider path -- RigExecPointTransformAPI carried one and was removed as
// dead surface (see docs/dead-surface-removal.md).

// ---------------------------------------------------------------------------
// RigExecJoint (IrXformable mirror, user-directed alignment 2026-07-25):
// matrix4d rest/posed spaces with scalar avars. A solver-posed joint's frame
// arrives as a value override from RigExecRigEvaluator (exec cannot traverse
// the solver's rigExec:joints backwards, so the binding lives in the compiled
// graph); otherwise the posed space comes from a posed:space connection, and
// an unconnected joint follows its namespace-parent joint's posed space with
// its local rest offset and avars applied. Point frames stay the internal
// value type: the spaces convert to landmark frames at this boundary.
// ---------------------------------------------------------------------------

static double
_ScalarInput(const VdfContext &ctx, const TfToken &name, double fallback)
{
    const double *value = ctx.GetInputValuePtr<double>(name);
    return value ? *value : fallback;
}

// Composes a local avar transform: rotations applied in avars:rotationOrder
// sequence, then rspin about the +X aim axis, then the translation
// (row-vector convention: leftmost factor applies first).
static GfMatrix4d
_ComposeAvars(
    double tx, double ty, double tz, double rx, double ry, double rz,
    double rspin, const TfToken &order)
{
    static const GfVec3d axes[3] = {
        GfVec3d(1, 0, 0), GfVec3d(0, 1, 0), GfVec3d(0, 0, 1)};
    const double angles[3] = {rx, ry, rz};
    std::string sequence = order.GetString();
    if (sequence.size() != 3) {
        sequence = "XYZ";
    }
    GfMatrix4d m(1.0);
    for (const char axis : sequence) {
        const int index = axis == 'X' ? 0 : axis == 'Y' ? 1 : 2;
        if (angles[index] != 0.0) {
            m = m * GfMatrix4d(
                        GfRotation(axes[index], angles[index]), GfVec3d(0));
        }
    }
    if (rspin != 0.0) {
        m = m * GfMatrix4d(GfRotation(axes[0], rspin), GfVec3d(0));
    }
    GfMatrix4d t(1.0);
    t.SetTranslate(GfVec3d(tx, ty, tz));
    return m * t;
}

// Shared with RigExecRigEvaluator, which extracts joint frames directly from
// a solver's aggregate using its in-memory binding (frameExtraction.h). One
// definition, so the two paths cannot drift.
static RigExecPointFrame
_FrameFromMatrix(const GfMatrix4d &m)
{
    return rigExec::RigExecFrameFromMatrix(m);
}

// The selected aggregate element's posed frame as a local-to-world matrix:
// normalized posed axes scaled by the posed/rest axis-length ratios (rest
// spaces are orthonormalized per the Ir contract, so the ratio is what
// carries stretch). Identity when the element is absent or degenerate.
// This is the former RigExecPointFrameView out:space math, reused for
// view-free per-element extraction (user-directed 2026-07-25), with identical
// numerics to the deleted view's out:space -> _FrameFromMatrix.
static GfMatrix4d
_ElementOutSpace(const RigExecPointFrameArray *source, size_t index)
{
    return rigExec::RigExecElementOutSpace(source, index);
}

// Local-to-world rest space: authored rest:space with the rest avars as a
// preceding local delta (all-default avars leave rest:space authoritative).
static GfMatrix4d
_JointRestSpace(const VdfContext &ctx)
{
    const GfMatrix4d *space =
        ctx.GetInputValuePtr<GfMatrix4d>(_tokens->restSpace);
    const GfMatrix4d local = _ComposeAvars(
        _ScalarInput(ctx, _tokens->restTx, 0),
        _ScalarInput(ctx, _tokens->restTy, 0),
        _ScalarInput(ctx, _tokens->restTz, 0),
        _ScalarInput(ctx, _tokens->restRx, 0),
        _ScalarInput(ctx, _tokens->restRy, 0),
        _ScalarInput(ctx, _tokens->restRz, 0),
        0.0, TfToken("XYZ"));
    GfMatrix4d rest = local * (space ? *space : GfMatrix4d(1.0));
    // Rest spaces are always orthonormalized (Ir contract).
    rest.Orthonormalize(/* issueWarning = */ false);
    return rest;
}

static RigExecPointFrame
_ComputeJointRestFrame(const VdfContext &ctx)
{
    return _FrameFromMatrix(_JointRestSpace(ctx));
}

static RigExecPointFrame
_ComputeJointPointFrame(const VdfContext &ctx)
{
    // A solver-posed joint never reaches this callback: RigExecRigEvaluator
    // supplies its frame as a value override, because exec cannot traverse
    // the solver's rigExec:joints backwards to find it. What remains here is
    // every other way a joint gets posed.
    //
    // 1. Connected posed:space (a provider's out:space) is authoritative.
    if (const GfMatrix4d *connected =
            ctx.GetInputValuePtr<GfMatrix4d>(_tokens->posedConnected)) {
        return _FrameFromMatrix(*connected);
    }
    // 2. A non-identity authored posed:space is used directly.
    const GfMatrix4d *authored =
        ctx.GetInputValuePtr<GfMatrix4d>(_tokens->posedSpace);
    if (authored && *authored != GfMatrix4d(1.0)) {
        return _FrameFromMatrix(*authored);
    }
    // 3. Fallback: follow the namespace-parent joint's posed space with
    // the local rest offset, avars applied as a local delta (Ir's
    // "follows the parent's posed space" behavior).
    const GfMatrix4d rest = _JointRestSpace(ctx);
    GfMatrix4d parentRest(1.0), parentPosed(1.0);
    if (const RigExecPointFrame *frame =
            ctx.GetInputValuePtr<RigExecPointFrame>(
                _tokens->parentRestFrame)) {
        if (frame->IsValid()) {
            static const std::array<GfVec3d, 4> identity = {
                GfVec3d(0), GfVec3d(1, 0, 0), GfVec3d(0, 1, 0),
                GfVec3d(0, 0, 1)};
            rigExec::RigExecPointsToMatrix(
                identity, frame->points, &parentRest);
        }
    }
    if (const RigExecPointFrame *frame =
            ctx.GetInputValuePtr<RigExecPointFrame>(
                _tokens->parentPosedFrame)) {
        if (frame->IsValid()) {
            static const std::array<GfVec3d, 4> identity = {
                GfVec3d(0), GfVec3d(1, 0, 0), GfVec3d(0, 1, 0),
                GfVec3d(0, 0, 1)};
            rigExec::RigExecPointsToMatrix(
                identity, frame->points, &parentPosed);
        }
    }
    const TfToken *order =
        ctx.GetInputValuePtr<TfToken>(_tokens->avarRotationOrder);
    const GfMatrix4d avars = _ComposeAvars(
        _ScalarInput(ctx, _tokens->avarTx, 0),
        _ScalarInput(ctx, _tokens->avarTy, 0),
        _ScalarInput(ctx, _tokens->avarTz, 0),
        _ScalarInput(ctx, _tokens->avarRx, 0),
        _ScalarInput(ctx, _tokens->avarRy, 0),
        _ScalarInput(ctx, _tokens->avarRz, 0),
        _ScalarInput(ctx, _tokens->avarRspin, 0),
        order ? *order : TfToken("XYZ"));
    // world = avars * (rest relative to parentRest) * parentPosed.
    return _FrameFromMatrix(
        avars * rest * parentRest.GetInverse() * parentPosed);
}

static GfMatrix4d
_ComputeJointMatrix(const VdfContext &ctx)
{
    // The rest->posed target-local map (spec §7.4 consumers): identical
    // semantics to the landmark-based PointsToMatrix path.
    const RigExecPointFrame *rest =
        ctx.GetInputValuePtr<RigExecPointFrame>(_tokens->selfRestFrame);
    const RigExecPointFrame *posed =
        ctx.GetInputValuePtr<RigExecPointFrame>(_tokens->computePointFrame);
    GfMatrix4d m(1.0);
    if (!rest || !rest->IsValid() || !posed || !posed->IsValid()) {
        ctx.Warn("joint computeMatrix: missing or degenerate frames");
        return m;
    }
    rigExec::RigExecPointsToMatrix(rest->points, posed->points, &m);
    return m;
}

#define RIGEXEC_REGISTER_XFORMABLE(SchemaName)                               \
    EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(SchemaName)                        \
    {                                                                        \
        self.PrimComputation(_tokens->computeRestFrame)                      \
            .Callback<RigExecPointFrame>(&_ComputeJointRestFrame)            \
            .Inputs(                                                         \
                AttributeValue<GfMatrix4d>(_tokens->restSpace),              \
                AttributeValue<double>(_tokens->restTx),                     \
                AttributeValue<double>(_tokens->restTy),                     \
                AttributeValue<double>(_tokens->restTz),                     \
                AttributeValue<double>(_tokens->restRx),                     \
                AttributeValue<double>(_tokens->restRy),                     \
                AttributeValue<double>(_tokens->restRz));                    \
                                                                             \
        self.PrimComputation(_tokens->computePointFrame)                     \
            .Callback<RigExecPointFrame>(&_ComputeJointPointFrame)           \
            .Inputs(                                                         \
                Attribute(_tokens->posedSpace)                               \
                    .Connections<GfMatrix4d>(                                \
                        ExecBuiltinComputations->computeValue)               \
                    .InputName(_tokens->posedConnected),                     \
                AttributeValue<GfMatrix4d>(_tokens->posedSpace),             \
                AttributeValue<GfMatrix4d>(_tokens->restSpace),              \
                AttributeValue<double>(_tokens->restTx),                     \
                AttributeValue<double>(_tokens->restTy),                     \
                AttributeValue<double>(_tokens->restTz),                     \
                AttributeValue<double>(_tokens->restRx),                     \
                AttributeValue<double>(_tokens->restRy),                     \
                AttributeValue<double>(_tokens->restRz),                     \
                AttributeValue<double>(_tokens->avarTx),                     \
                AttributeValue<double>(_tokens->avarTy),                     \
                AttributeValue<double>(_tokens->avarTz),                     \
                AttributeValue<double>(_tokens->avarRx),                     \
                AttributeValue<double>(_tokens->avarRy),                     \
                AttributeValue<double>(_tokens->avarRz),                     \
                AttributeValue<double>(_tokens->avarRspin),                  \
                AttributeValue<TfToken>(_tokens->avarRotationOrder),         \
                NamespaceAncestor<RigExecPointFrame>(                        \
                    _tokens->computePointFrame)                              \
                    .InputName(_tokens->parentPosedFrame),                   \
                NamespaceAncestor<RigExecPointFrame>(                        \
                    _tokens->computeRestFrame)                               \
                    .InputName(_tokens->parentRestFrame));                   \
                                                                             \
        self.PrimComputation(_tokens->computeMatrix)                         \
            .Callback<GfMatrix4d>(&_ComputeJointMatrix)                      \
            .Inputs(                                                         \
                Computation<RigExecPointFrame>(_tokens->computePointFrame)   \
                    .Required(),                                             \
                Computation<RigExecPointFrame>(_tokens->computeRestFrame)    \
                    .InputName(_tokens->selfRestFrame)                       \
                    .Required());                                            \
    }

RIGEXEC_REGISTER_XFORMABLE(RigExecJoint)
RIGEXEC_REGISTER_XFORMABLE(RigExecControl)
#undef RIGEXEC_REGISTER_XFORMABLE

// ---------------------------------------------------------------------------
// RigExecFkChain: applies control frames to a rest hierarchy. v0.1 treats
// the targeted control list as an ordered chain (each element's parent is
// the preceding element).
// ---------------------------------------------------------------------------

static RigExecPointFrameArray
_ComputeFkChain(const VdfContext &ctx)
{
    // A frame/rest cardinality mismatch is a structural error: fail the
    // whole solver explicitly instead of silently truncating (spec §6.6).
    VdfReadIterator<RigExecPointFrame> poseCount(ctx, _tokens->controlFrames);
    VdfReadIterator<RigExecPointFrame> restCount(ctx, _tokens->controlRests);
    if (poseCount.ComputeSize() != restCount.ComputeSize()) {
        ctx.Warn("FkChain: control frame/rest cardinality mismatch (%zu/%zu)",
                 poseCount.ComputeSize(), restCount.ComputeSize());
        return RigExecPointFrameArray();
    }

    std::vector<rigExec::RigExecFkChainElement> elements;
    VdfReadIterator<RigExecPointFrame> poseIt(ctx, _tokens->controlFrames);
    VdfReadIterator<RigExecPointFrame> restIt(ctx, _tokens->controlRests);
    int index = 0;
    for (; !poseIt.IsAtEnd() && !restIt.IsAtEnd(); ++poseIt, ++restIt) {
        rigExec::RigExecFkChainElement e;
        e.restPoints = (*restIt).points;
        e.posePoints = (*poseIt).points;
        e.parentIndex = index - 1;
        elements.push_back(e);
        ++index;
    }

    RigExecPointFrameArray result;
    result.frames = rigExec::RigExecSolveFkChain(elements);
    result.rests.reserve(elements.size());
    for (const auto &e : elements) {
        result.rests.push_back(e.restPoints);
    }
    return result;
}

EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(RigExecFkChain)
{
    self.PrimComputation(_tokens->computePointFrameArray)
        .Callback<RigExecPointFrameArray>(&_ComputeFkChain)
        .Inputs(
            Relationship(_tokens->controls)
                .TargetedObjects<RigExecPointFrame>(_tokens->computePointFrame)
                .InputName(_tokens->controlFrames)
                .Required(),
            Relationship(_tokens->controls)
                .TargetedObjects<RigExecPointFrame>(_tokens->computeRestFrame)
                .InputName(_tokens->controlRests)
                .Required());
}

// ---------------------------------------------------------------------------
// RigExecTwoBoneIk: analytic solve publishing [root, mid, end] frames.
// ---------------------------------------------------------------------------

static RigExecPointFrameArray
_ComputeTwoBoneIk(const VdfContext &ctx)
{
    RigExecPointFrameArray result;

    const RigExecPointFrame *root =
        ctx.GetInputValuePtr<RigExecPointFrame>(_tokens->rootFrame);
    const RigExecPointFrame *effector =
        ctx.GetInputValuePtr<RigExecPointFrame>(_tokens->effectorFrame);
    const RigExecPointFrame *pole =
        ctx.GetInputValuePtr<RigExecPointFrame>(_tokens->poleFrame);
    if (!root || !effector || !pole) {
        return result;
    }

    rigExec::RigExecTwoBoneIkParams params;
    const double *upper = ctx.GetInputValuePtr<double>(_tokens->upperLength);
    const double *lower = ctx.GetInputValuePtr<double>(_tokens->lowerLength);
    const double *bend =
        ctx.GetInputValuePtr<double>(_tokens->preferredBendRadians);
    const float *stretch = ctx.GetInputValuePtr<float>(_tokens->inputsStretch);
    const float *softness =
        ctx.GetInputValuePtr<float>(_tokens->inputsSoftness);
    params.upperLength = upper ? *upper : 1.0;
    params.lowerLength = lower ? *lower : 1.0;
    params.preferredBendRadians = bend ? *bend : 0.0;
    params.stretch = stretch ? *stretch : 1.0;
    params.softness = softness ? *softness : 0.0;

    // Rest landmark sets for the three outputs. The mid rest derives from
    // the root rest translated along its rest aim by the upper length.
    const RigExecPointFrame *rootRest =
        ctx.GetInputValuePtr<RigExecPointFrame>(_tokens->rootRest);
    const RigExecPointFrame *effectorRest =
        ctx.GetInputValuePtr<RigExecPointFrame>(_tokens->effectorRest);
    std::array<std::array<GfVec3d, 4>, 3> rests;
    rests[0] = rootRest ? rootRest->points : _IdentityLandmarks();
    const GfVec3d restAim =
        (rests[0][1] - rests[0][0]).GetNormalized();
    rests[1] = rests[0];
    for (auto &p : rests[1]) {
        p += restAim * params.upperLength;
    }
    rests[2] = effectorRest ? effectorRest->points : _IdentityLandmarks();

    const auto frames = rigExec::RigExecSolveTwoBoneIk(
        *root, *effector, *pole, rests, params);
    result.frames.assign(frames.begin(), frames.end());
    result.rests.assign(rests.begin(), rests.end());
    return result;
}

EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(RigExecTwoBoneIk)
{
    self.PrimComputation(_tokens->computePointFrameArray)
        .Callback<RigExecPointFrameArray>(&_ComputeTwoBoneIk)
        .Inputs(
            Relationship(_tokens->rootControl)
                .TargetedObjects<RigExecPointFrame>(_tokens->computePointFrame)
                .InputName(_tokens->rootFrame)
                .Required(),
            Relationship(_tokens->rootControl)
                .TargetedObjects<RigExecPointFrame>(_tokens->computeRestFrame)
                .InputName(_tokens->rootRest),
            Relationship(_tokens->effectorControl)
                .TargetedObjects<RigExecPointFrame>(_tokens->computePointFrame)
                .InputName(_tokens->effectorFrame)
                .Required(),
            Relationship(_tokens->effectorControl)
                .TargetedObjects<RigExecPointFrame>(_tokens->computeRestFrame)
                .InputName(_tokens->effectorRest),
            Relationship(_tokens->poleControl)
                .TargetedObjects<RigExecPointFrame>(_tokens->computePointFrame)
                .InputName(_tokens->poleFrame)
                .Required(),
            AttributeValue<double>(_tokens->upperLength),
            AttributeValue<double>(_tokens->lowerLength),
            AttributeValue<double>(_tokens->preferredBendRadians),
            AttributeValue<float>(_tokens->inputsStretch),
            AttributeValue<float>(_tokens->inputsSoftness));
}

// ---------------------------------------------------------------------------
// RigExecBlendPointFrames: element-wise blend of two aggregates.
// ---------------------------------------------------------------------------

static RigExecPointFrameArray
_ComputeBlendPointFrames(const VdfContext &ctx)
{
    const RigExecPointFrameArray *a =
        ctx.GetInputValuePtr<RigExecPointFrameArray>(_tokens->inputAFrames);
    const RigExecPointFrameArray *b =
        ctx.GetInputValuePtr<RigExecPointFrameArray>(_tokens->inputBFrames);
    if (!a) {
        return b ? *b : RigExecPointFrameArray();
    }
    if (!b) {
        return *a;
    }

    const float *weightPtr =
        ctx.GetInputValuePtr<float>(_tokens->inputsWeight);
    // Clamp to [0, 1]: the composed ClampIKFKWeight property mover in the
    // reference asset lowers to exactly this bound.
    const double w =
        std::min(std::max(weightPtr ? double(*weightPtr) : 0.0, 0.0), 1.0);

    const TfToken *scaleTok =
        ctx.GetInputValuePtr<TfToken>(_tokens->scaleBlend);
    const rigExec::RigExecScaleBlend scaleMode =
        (scaleTok && *scaleTok == "linear")
            ? rigExec::RigExecScaleBlend::Linear
            : rigExec::RigExecScaleBlend::Log;
    // rotationBlend currently admits only "shortestArc" (schema
    // allowedTokens); parse it so authored intent is honored, and reject
    // unknown tokens explicitly rather than silently substituting.
    const TfToken *rotTok =
        ctx.GetInputValuePtr<TfToken>(_tokens->rotationBlend);
    if (rotTok && !rotTok->IsEmpty() && *rotTok != "shortestArc") {
        ctx.Warn("BlendPointFrames: unsupported rotationBlend '%s'",
                 rotTok->GetText());
        return RigExecPointFrameArray();
    }
    const rigExec::RigExecRotationBlend rotationMode =
        rigExec::RigExecRotationBlend::ShortestArc;

    // Aggregate cardinality mismatch is structural: fail explicitly
    // instead of silently dropping addressable elements (spec §6.6).
    if (a->GetSize() != b->GetSize()) {
        ctx.Warn("BlendPointFrames: input cardinality mismatch (%zu/%zu)",
                 a->GetSize(), b->GetSize());
        return RigExecPointFrameArray();
    }

    // Rest sets travel with the aggregate; a missing rest set is the same
    // structural mismatch as a missing frame (no silent identity).
    if (a->rests.size() != a->GetSize()) {
        ctx.Warn("BlendPointFrames: input rest-set cardinality mismatch "
                 "(%zu rests for %zu frames)",
                 a->rests.size(), a->GetSize());
        return RigExecPointFrameArray();
    }

    RigExecPointFrameArray result;
    const size_t n = a->GetSize();
    result.frames.reserve(n);
    result.rests.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        const std::array<GfVec3d, 4> &rest = a->rests[i];
        result.frames.push_back(rigExec::RigExecBlendFrames(
            a->frames[i], b->frames[i], rest, w, rotationMode, scaleMode));
        result.rests.push_back(rest);
    }
    return result;
}

EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(RigExecBlendPointFrames)
{
    self.PrimComputation(_tokens->computePointFrameArray)
        .Callback<RigExecPointFrameArray>(&_ComputeBlendPointFrames)
        .Inputs(
            Relationship(_tokens->inputA)
                .TargetedObjects<RigExecPointFrameArray>(
                    _tokens->computePointFrameArray)
                .InputName(_tokens->inputAFrames)
                .Required(),
            Relationship(_tokens->inputB)
                .TargetedObjects<RigExecPointFrameArray>(
                    _tokens->computePointFrameArray)
                .InputName(_tokens->inputBFrames)
                .Required(),
            AttributeValue<float>(_tokens->inputsWeight),
            AttributeValue<TfToken>(_tokens->rotationBlend),
            AttributeValue<TfToken>(_tokens->scaleBlend));
}

// ---------------------------------------------------------------------------
// RigExecTwistDistribution: N frames between start and end providers.
// ---------------------------------------------------------------------------

static RigExecPointFrameArray
_ComputeTwistDistribution(const VdfContext &ctx)
{
    RigExecPointFrameArray result;
    const RigExecPointFrame *start =
        ctx.GetInputValuePtr<RigExecPointFrame>(_tokens->startFrame);
    const RigExecPointFrame *end =
        ctx.GetInputValuePtr<RigExecPointFrame>(_tokens->endFrame);
    if (!start || !end) {
        return result;
    }
    const RigExecPointFrame *startRest =
        ctx.GetInputValuePtr<RigExecPointFrame>(_tokens->startRest);
    const RigExecPointFrame *endRest =
        ctx.GetInputValuePtr<RigExecPointFrame>(_tokens->endRest);
    const std::array<GfVec3d, 4> sRest =
        startRest ? startRest->points : _IdentityLandmarks();
    const std::array<GfVec3d, 4> eRest =
        endRest ? endRest->points : _IdentityLandmarks();

    std::vector<double> weights;
    VdfReadIterator<float> wIt(ctx, _tokens->weights);
    for (; !wIt.IsAtEnd(); ++wIt) {
        weights.push_back(*wIt);
    }
    if (weights.empty()) {
        const int *count = ctx.GetInputValuePtr<int>(_tokens->count);
        const int n = count ? std::max(*count, 1) : 1;
        for (int k = 0; k < n; ++k) {
            weights.push_back(n == 1 ? 0.0 : double(k) / (n - 1));
        }
    }

    result.frames =
        rigExec::RigExecDistributeTwist(*start, *end, sRest, eRest, weights);
    result.rests.assign(result.frames.size(), sRest);
    return result;
}

EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(RigExecTwistDistribution)
{
    self.PrimComputation(_tokens->computePointFrameArray)
        .Callback<RigExecPointFrameArray>(&_ComputeTwistDistribution)
        .Inputs(
            Relationship(_tokens->startRel)
                .TargetedObjects<RigExecPointFrame>(_tokens->computePointFrame)
                .InputName(_tokens->startFrame)
                .Required(),
            Relationship(_tokens->startRel)
                .TargetedObjects<RigExecPointFrame>(_tokens->computeRestFrame)
                .InputName(_tokens->startRest),
            Relationship(_tokens->endRel)
                .TargetedObjects<RigExecPointFrame>(_tokens->computePointFrame)
                .InputName(_tokens->endFrame)
                .Required(),
            Relationship(_tokens->endRel)
                .TargetedObjects<RigExecPointFrame>(_tokens->computeRestFrame)
                .InputName(_tokens->endRest),
            AttributeValue<float>(_tokens->weights),
            AttributeValue<int>(_tokens->count));
}
