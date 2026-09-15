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
#include "solverKernels.h"

#include "rigExecMath/avarScale.h"
#include "rigExecMath/pointFrame.h"
#include "rigExecMath/solvers.h"
#include "rigExecMath/splineIk.h"

#include "pxr/pxr.h"
#include "pxr/base/gf/math.h"
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
#include <limits>

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
    (computeDefaultFrame)
    (computePointFrameArray)
    (computedDefaultSpace)
    (computedParentDefaultSpace)
    (computedParentSpace)
    (computedAvarDefaultSpace)
    (computedPosedDefaultSpace)
    (rawSpace)
    (connectedSpace)
    (fallbackSpace)
    (parentDefaultFrame)

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
    (midFrame)
    (midRest)
    (jointRests)

    // Attribute tokens.
    ((controls, "rigExec:controls"))
    ((controlSpace, "rigExec:controlSpace"))
    ((controlSpaceWorld, "world"))
    ((controlSpaceParentRelative, "parentRelative"))
    ((rootControl, "rigExec:rootControl"))
    ((effectorControl, "rigExec:effectorControl"))
    ((poleControl, "rigExec:poleControl"))
    ((preferredBendRadians, "rigExec:preferredBendRadians"))
    ((inputA, "rigExec:inputA"))
    ((inputB, "rigExec:inputB"))
    ((rotationBlend, "rigExec:rotationBlend"))
    ((scaleBlend, "rigExec:scaleBlend"))
    ((startRel, "rigExec:start"))
    ((startFrameRel, "rigExec:startFrame"))
    ((endRel, "rigExec:end"))
    ((weights, "rigExec:weights"))
    ((count, "rigExec:count"))
    ((twistTurns, "inputs:twistTurns"))
    ((joints, "rigExec:joints"))
    ((jointElements, "rigExec:jointElements"))
    ((upperLengthOffset, "rigExec:upperLengthOffset"))
    ((lowerLengthOffset, "rigExec:lowerLengthOffset"))
    ((inputsWeight, "inputs:weight"))
    ((inputsStretch, "inputs:stretch"))
    ((inputsSoftness, "inputs:softness"))
    ((midControl, "rigExec:midControl"))
    ((endControl, "rigExec:endControl"))
    ((volumeWeights, "rigExec:volumeWeights"))
    ((restLength, "rigExec:restLength"))
    ((inputsPreserveVolume, "inputs:preserveVolume"))
    ((inputsMidFollowWeight, "inputs:midFollowWeight"))
    ((inputsMinLengthRatio, "inputs:minLengthRatio"))
    ((rootTangent, "rigExec:rootTangent"))
    ((inputsRoll, "inputs:roll"))
    ((inputsTwist, "inputs:twist"))

    // Ir-aligned joint contract (IrXformable mirror).
    ((restSpace, "rest:space"))
    ((posedSpace, "posed:space"))
    ((restTx, "rest:tx"))
    ((restTy, "rest:ty"))
    ((restTz, "rest:tz"))
    ((restRx, "rest:rx"))
    ((restRy, "rest:ry"))
    ((restRz, "rest:rz"))
    ((defaultSpace, "default:space"))
    ((defaultTx, "default:tx"))
    ((defaultTy, "default:ty"))
    ((defaultTz, "default:tz"))
    ((defaultRx, "default:rx"))
    ((defaultRy, "default:ry"))
    ((defaultRz, "default:rz"))
    ((posedDefaultSpace, "posed:defaultSpace"))
    ((avarDefaultSpace, "avars:defaultSpace"))
    ((avarUnitScaleFactor, "avars:unitScaleFactor"))
    ((parentSpace, "parent:space"))
    ((parentDefaultSpace, "parent:defaultSpace"))
    ((avarTx, "avars:tx"))
    ((avarTy, "avars:ty"))
    ((avarTz, "avars:tz"))
    ((avarSx, "avars:sx"))
    ((avarSy, "avars:sy"))
    ((avarSz, "avars:sz"))
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

// Composes a local avar transform: per-axis scale, rotations applied in
// avars:rotationOrder sequence, rspin about the +X aim axis, then translation
// (row-vector convention: leftmost factor applies first).
static GfMatrix4d
_ComposeAvars(
    double tx, double ty, double tz, double sx, double sy, double sz,
    double rx, double ry, double rz, double rspin, const TfToken &order)
{
    static const GfVec3d axes[3] = {
        GfVec3d(1, 0, 0), GfVec3d(0, 1, 0), GfVec3d(0, 0, 1)};
    const double angles[3] = {rx, ry, rz};
    std::string sequence = order.GetString();
    if (sequence.size() != 3) {
        sequence = "XYZ";
    }
    GfMatrix4d m(1.0);
    m.SetScale(GfVec3d(
        rigExec::RigExecNormalizeAvarScale(sx),
        rigExec::RigExecNormalizeAvarScale(sy),
        rigExec::RigExecNormalizeAvarScale(sz)));
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

static GfMatrix4d
_SpaceFromFrame(const RigExecPointFrame *frame)
{
    GfMatrix4d result(1.0);
    if (frame) {
        if (!frame->IsValid() || frame->IsDegenerate() ||
            !rigExec::RigExecPointsToMatrix(_IdentityLandmarks(), frame->points, &result)) {
            // Missing ancestors select identity; an existing invalid frame
            // must retain failure through the matrix-typed space expressions.
            // FrameFromMatrix will classify this sentinel as degenerate.
            result[3][0] = std::numeric_limits<double>::quiet_NaN();
        }
    }
    return result;
}

// Rest space relative to the namespace frame provider: the authored
// rest:space with the rest avars as a preceding local delta (all-default
// avars leave rest:space authoritative), carried into the parent's rest
// frame. A provider with no RigExec ancestor resolves against identity --
// _SpaceFromFrame returns it for a null frame -- so a top-level rest keeps
// its authored local-to-world meaning.
static GfMatrix4d
_JointRestSpace(const VdfContext &ctx)
{
    const GfMatrix4d *space =
        ctx.GetInputValuePtr<GfMatrix4d>(_tokens->restSpace);
    const GfMatrix4d local = _ComposeAvars(
        _ScalarInput(ctx, _tokens->restTx, 0),
        _ScalarInput(ctx, _tokens->restTy, 0),
        _ScalarInput(ctx, _tokens->restTz, 0),
        1.0, 1.0, 1.0,
        _ScalarInput(ctx, _tokens->restRx, 0),
        _ScalarInput(ctx, _tokens->restRy, 0),
        _ScalarInput(ctx, _tokens->restRz, 0),
        0.0, TfToken("XYZ"));
    GfMatrix4d rest = local * (space ? *space : GfMatrix4d(1.0));
    // Rest spaces are always orthonormalized (Ir contract). Orthonormalize
    // the local factor BEFORE the parent multiply: the parent's frame is
    // already orthonormal, so the product is too, and an invalid ancestor's
    // NaN sentinel survives the multiply instead of being scrubbed by it.
    rest.Orthonormalize(/* issueWarning = */ false);
    return rest * _SpaceFromFrame(
        ctx.GetInputValuePtr<RigExecPointFrame>(_tokens->parentRestFrame));
}

static RigExecPointFrame
_ComputeJointRestFrame(const VdfContext &ctx)
{
    return _FrameFromMatrix(_JointRestSpace(ctx));
}

// Matrix spaces preserve the existing posed:space convention: a connection
// (including identity) is authoritative, a non-identity authored value is an
// explicit space, and the schema identity selects the computed fallback.
static GfMatrix4d
_ComputeSpaceExpression(const VdfContext &ctx)
{
    if (const auto *connected =
            ctx.GetInputValuePtr<GfMatrix4d>(_tokens->connectedSpace)) {
        return *connected;
    }
    if (const auto *raw = ctx.GetInputValuePtr<GfMatrix4d>(_tokens->rawSpace)) {
        if (*raw != GfMatrix4d(1.0)) return *raw;
    }
    const auto *fallback =
        ctx.GetInputValuePtr<GfMatrix4d>(_tokens->fallbackSpace);
    return fallback ? *fallback : GfMatrix4d(1.0);
}

static GfMatrix4d
_ComputeDefaultSpace(const VdfContext &ctx)
{
    const GfMatrix4d rest = _SpaceFromFrame(
        ctx.GetInputValuePtr<RigExecPointFrame>(_tokens->selfRestFrame));
    const GfMatrix4d parentRest = _SpaceFromFrame(
        ctx.GetInputValuePtr<RigExecPointFrame>(_tokens->parentRestFrame));
    const auto *parentDefault =
        ctx.GetInputValuePtr<GfMatrix4d>(_tokens->parentDefaultSpace);
    const GfMatrix4d offset = _ComposeAvars(
        _ScalarInput(ctx, _tokens->defaultTx, 0),
        _ScalarInput(ctx, _tokens->defaultTy, 0),
        _ScalarInput(ctx, _tokens->defaultTz, 0), 1, 1, 1,
        _ScalarInput(ctx, _tokens->defaultRx, 0),
        _ScalarInput(ctx, _tokens->defaultRy, 0),
        _ScalarInput(ctx, _tokens->defaultRz, 0), 0, TfToken("XYZ"));
    return offset * rest * parentRest.GetInverse() *
           (parentDefault ? *parentDefault : GfMatrix4d(1.0));
}

static RigExecPointFrame
_ComputeXformablePointFrame(const VdfContext &ctx, bool readScaleAvars)
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
    // 3. Start from the effective default pose, follow the selected parent,
    // then apply local avars. With unmodified default channels this reduces
    // exactly to avars * rest * parentRest^-1 * parentPosed.
    const auto *defaultSpace =
        ctx.GetInputValuePtr<GfMatrix4d>(_tokens->posedDefaultSpace);
    const auto *parentDefault =
        ctx.GetInputValuePtr<GfMatrix4d>(_tokens->parentDefaultSpace);
    const auto *parentPosed =
        ctx.GetInputValuePtr<GfMatrix4d>(_tokens->parentSpace);
    const double units = _ScalarInput(ctx, _tokens->avarUnitScaleFactor, 1);
    const TfToken *order =
        ctx.GetInputValuePtr<TfToken>(_tokens->avarRotationOrder);
    const GfMatrix4d avars = _ComposeAvars(
        _ScalarInput(ctx, _tokens->avarTx, 0) * units,
        _ScalarInput(ctx, _tokens->avarTy, 0) * units,
        _ScalarInput(ctx, _tokens->avarTz, 0) * units,
        readScaleAvars ? _ScalarInput(ctx, _tokens->avarSx, 1) : 1.0,
        readScaleAvars ? _ScalarInput(ctx, _tokens->avarSy, 1) : 1.0,
        readScaleAvars ? _ScalarInput(ctx, _tokens->avarSz, 1) : 1.0,
        _ScalarInput(ctx, _tokens->avarRx, 0),
        _ScalarInput(ctx, _tokens->avarRy, 0),
        _ScalarInput(ctx, _tokens->avarRz, 0),
        _ScalarInput(ctx, _tokens->avarRspin, 0),
        order ? *order : TfToken("XYZ"));
    return _FrameFromMatrix(
        avars * (defaultSpace ? *defaultSpace : GfMatrix4d(1.0)) *
        (parentDefault ? parentDefault->GetInverse() : GfMatrix4d(1.0)) *
        (parentPosed ? *parentPosed : GfMatrix4d(1.0)));
}

static RigExecPointFrame
_ComputeJointPointFrame(const VdfContext &ctx)
{
    return _ComputeXformablePointFrame(ctx, /* readScaleAvars = */ true);
}

static RigExecPointFrame
_ComputeVolumeWeightPointFrame(const VdfContext &ctx)
{
    // Volume shape is owned exclusively by inputs:scaleX/Y/Z. Its placement
    // contract is rigid, so do not expose transform-scale avars as silent
    // no-ops on RigExecVolumeWeight.
    return _ComputeXformablePointFrame(ctx, /* readScaleAvars = */ false);
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

#define RIGEXEC_AVAR_SCALE_INPUTS                                            \
    AttributeValue<double>(_tokens->avarSx),                                 \
    AttributeValue<double>(_tokens->avarSy),                                 \
    AttributeValue<double>(_tokens->avarSz),

#define RIGEXEC_NO_AVAR_SCALE_INPUTS

#define RIGEXEC_SPACE_EXPRESSION(AttributeToken, FallbackComputation)        \
    self.AttributeExpression(AttributeToken)                               \
        .Callback<GfMatrix4d>(&_ComputeSpaceExpression)                      \
        .Inputs(                                                            \
            Computation<GfMatrix4d>(                                        \
                ExecBuiltinComputations->computeResolvedValue)             \
                .InputName(_tokens->rawSpace),                              \
            Connections<GfMatrix4d>(                                        \
                ExecBuiltinComputations->computeValue)                     \
                .InputName(_tokens->connectedSpace),                        \
            Prim().Computation<GfMatrix4d>(FallbackComputation)              \
                .InputName(_tokens->fallbackSpace));

#define RIGEXEC_REGISTER_XFORMABLE(                                         \
    SchemaName, PointFrameCallback, ScaleInputs)                             \
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
                AttributeValue<double>(_tokens->restRz),                     \
                NamespaceAncestor<RigExecPointFrame>(                        \
                    _tokens->computeRestFrame)                               \
                    .InputName(_tokens->parentRestFrame));                   \
        self.PrimComputation(_tokens->computedDefaultSpace)                 \
            .Callback<GfMatrix4d>(&_ComputeDefaultSpace)                     \
            .Inputs(                                                        \
                Computation<RigExecPointFrame>(_tokens->computeRestFrame)   \
                    .InputName(_tokens->selfRestFrame),                     \
                NamespaceAncestor<RigExecPointFrame>(                       \
                    _tokens->computeRestFrame)                              \
                    .InputName(_tokens->parentRestFrame),                   \
                AttributeValue<GfMatrix4d>(_tokens->parentDefaultSpace),     \
                AttributeValue<double>(_tokens->defaultTx),                 \
                AttributeValue<double>(_tokens->defaultTy),                 \
                AttributeValue<double>(_tokens->defaultTz),                 \
                AttributeValue<double>(_tokens->defaultRx),                 \
                AttributeValue<double>(_tokens->defaultRy),                 \
                AttributeValue<double>(_tokens->defaultRz));                \
        self.PrimComputation(_tokens->computeDefaultFrame)                  \
            .Callback<RigExecPointFrame>(+[](const VdfContext &ctx) {        \
                return _FrameFromMatrix(ctx.GetInputValue<GfMatrix4d>(       \
                    _tokens->defaultSpace));                                \
            })                                                              \
            .Inputs(AttributeValue<GfMatrix4d>(_tokens->defaultSpace));       \
        self.PrimComputation(_tokens->computedParentDefaultSpace)           \
            .Callback<GfMatrix4d>(+[](const VdfContext &ctx) {               \
                return _SpaceFromFrame(                                     \
                    ctx.GetInputValuePtr<RigExecPointFrame>(                \
                        _tokens->parentDefaultFrame));                      \
            })                                                              \
            .Inputs(NamespaceAncestor<RigExecPointFrame>(                   \
                _tokens->computeDefaultFrame)                               \
                .InputName(_tokens->parentDefaultFrame));                   \
        self.PrimComputation(_tokens->computedParentSpace)                  \
            .Callback<GfMatrix4d>(+[](const VdfContext &ctx) {               \
                return _SpaceFromFrame(                                     \
                    ctx.GetInputValuePtr<RigExecPointFrame>(                \
                        _tokens->parentPosedFrame));                        \
            })                                                              \
            .Inputs(NamespaceAncestor<RigExecPointFrame>(                   \
                _tokens->computePointFrame)                                 \
                .InputName(_tokens->parentPosedFrame));                     \
        self.PrimComputation(_tokens->computedAvarDefaultSpace)             \
            .Callback<GfMatrix4d>(+[](const VdfContext &ctx) {               \
                return ctx.GetInputValue<GfMatrix4d>(_tokens->defaultSpace); \
            })                                                              \
            .Inputs(AttributeValue<GfMatrix4d>(_tokens->defaultSpace));       \
        self.PrimComputation(_tokens->computedPosedDefaultSpace)            \
            .Callback<GfMatrix4d>(+[](const VdfContext &ctx) {               \
                return ctx.GetInputValue<GfMatrix4d>(                        \
                    _tokens->avarDefaultSpace);                             \
            })                                                              \
            .Inputs(AttributeValue<GfMatrix4d>(_tokens->avarDefaultSpace));   \
        RIGEXEC_SPACE_EXPRESSION(                                           \
            _tokens->defaultSpace, _tokens->computedDefaultSpace)            \
        RIGEXEC_SPACE_EXPRESSION(                                           \
            _tokens->avarDefaultSpace, _tokens->computedAvarDefaultSpace)    \
        RIGEXEC_SPACE_EXPRESSION(                                           \
            _tokens->posedDefaultSpace, _tokens->computedPosedDefaultSpace)  \
        RIGEXEC_SPACE_EXPRESSION(                                           \
            _tokens->parentSpace, _tokens->computedParentSpace)              \
        RIGEXEC_SPACE_EXPRESSION(                                           \
            _tokens->parentDefaultSpace, _tokens->computedParentDefaultSpace)\
                                                                             \
        self.PrimComputation(_tokens->computePointFrame)                     \
            .Callback<RigExecPointFrame>(PointFrameCallback)                 \
            .Inputs(                                                         \
                Attribute(_tokens->posedSpace)                               \
                    .Connections<GfMatrix4d>(                                \
                        ExecBuiltinComputations->computeValue)               \
                    .InputName(_tokens->posedConnected),                     \
                AttributeValue<GfMatrix4d>(_tokens->posedSpace),             \
                AttributeValue<GfMatrix4d>(_tokens->posedDefaultSpace),      \
                AttributeValue<GfMatrix4d>(_tokens->parentDefaultSpace),     \
                AttributeValue<GfMatrix4d>(_tokens->parentSpace),            \
                AttributeValue<double>(_tokens->avarUnitScaleFactor),       \
                AttributeValue<double>(_tokens->avarTx),                     \
                AttributeValue<double>(_tokens->avarTy),                     \
                AttributeValue<double>(_tokens->avarTz),                     \
                ScaleInputs                                                  \
                AttributeValue<double>(_tokens->avarRx),                     \
                AttributeValue<double>(_tokens->avarRy),                     \
                AttributeValue<double>(_tokens->avarRz),                     \
                AttributeValue<double>(_tokens->avarRspin),                  \
                AttributeValue<TfToken>(_tokens->avarRotationOrder));        \
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

RIGEXEC_REGISTER_XFORMABLE(
    RigExecJoint, &_ComputeJointPointFrame, RIGEXEC_AVAR_SCALE_INPUTS)
RIGEXEC_REGISTER_XFORMABLE(
    RigExecControl, &_ComputeJointPointFrame, RIGEXEC_AVAR_SCALE_INPUTS)

// The volumetric weight objects are RigExecXformables too (spec §4.1
// volumetric extension), so they get the same placement contract: a
// volume authored inside a joint follows it through the same
// NamespaceAncestor chain, with nothing wired. Unlike controls and joints,
// it deliberately does not bind avars:sx/sy/sz: volume shape is authored only
// through inputs:scaleX/Y/Z, and its placement is rigidized by both evaluation
// and imaging.
//
// Registered ONCE on the ABSTRACT base, unlike the two above. Exec
// composes a prim's computation set by walking its full ancestor type
// vector strongest-to-weakest (exec/definitionRegistry.cpp
// _GetFullyExpandedSchemaTypeVector), so the three concrete volume
// weights inherit these three computations from RigExecVolumeWeight.
//
// It has to be the base rather than the concrete types, because the
// concrete types already carry an
// EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA block in moverKernels.cpp for
// computeWeightPacket, and one schema may only be opened once: the macro
// emits a whole registration function plus its TF_REGISTRY_FUNCTION per
// invocation, and a second block for the same schema is a second,
// independent registration pass over a type the first pass has already
// marked complete. Splitting by TYPE instead of by file keeps each
// schema opened exactly once.
RIGEXEC_REGISTER_XFORMABLE(
    RigExecVolumeWeight, &_ComputeVolumeWeightPointFrame,
    RIGEXEC_NO_AVAR_SCALE_INPUTS)
#undef RIGEXEC_REGISTER_XFORMABLE
#undef RIGEXEC_SPACE_EXPRESSION
#undef RIGEXEC_NO_AVAR_SCALE_INPUTS
#undef RIGEXEC_AVAR_SCALE_INPUTS

// ---------------------------------------------------------------------------
// RigExecFkChain: applies control frames to a rest hierarchy. v0.1 treats
// the targeted control list as an ordered chain (each element's parent is
// the preceding element).
//
// rigExec:controlSpace says what a control's frame already contains.
// `world` (default): sibling controls, each frame carrying only its own
// delta A_i, composed here as W_i = W_(i-1) . A_i. `parentRelative`:
// controls nested one under the next, whose computePointFrame already
// travels with the parent control's posed frame (the NamespaceAncestor
// input of the xformable computations above), so the asset-space delta
// pose_i . rest_i^-1 IS W_i and composing the parent in again would apply
// its motion twice -- every element is solved as a chain root instead.
// The joints come out the same either way; only the control frames differ.
//
// rigExec:startFrame (optional) is the frame the chain HANGS FROM. Without
// it this solver is absolute: it composes control deltas onto the controls'
// own asset-space rests and nothing tells it that its joints live under,
// say, a wrist. Measured on a synthetic `a -> b -> c` with a chain over
// `[b, c]` only: posing `a` by avars:rz = 90 left b at (10,0,0) and c at
// (20,0,0), unmoved. With the relationship authored, the start provider's
// rest-to-pose delta S is PREPENDED as a synthetic element -- reusing the
// kernel rather than adding a case to it -- so W_0 = S . A_0 in `world`
// and W_i = S . A_i in `parentRelative` (where each delta is already the
// whole chain map, hence every real element parents onto the synthetic one
// rather than onto -1). The synthetic element's own frame is then DROPPED,
// which is what keeps the published cardinality equal to the control count
// and every joint's element index unchanged.
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

    // Unauthored or `world` keeps the linear parent chain; `parentRelative`
    // makes every element its own root (see the header comment). An
    // unknown token is reported and treated as `world`, so a typo degrades
    // to the documented default rather than to an empty solve.
    bool parentRelative = false;
    if (const TfToken *space =
            ctx.GetInputValuePtr<TfToken>(_tokens->controlSpace)) {
        if (*space == _tokens->controlSpaceParentRelative) {
            parentRelative = true;
        } else if (!space->IsEmpty() &&
                   *space != _tokens->controlSpaceWorld) {
            ctx.Warn("FkChain: unsupported controlSpace '%s'; using world",
                     space->GetText());
        }
    }

    // The optional base frame. Both halves are needed to form a delta at
    // all, so a start provider that published a pose but no rest is an
    // authoring/compile gap worth naming rather than silently solving as
    // if the chain were absolute again.
    const RigExecPointFrame *start =
        ctx.GetInputValuePtr<RigExecPointFrame>(_tokens->startFrame);
    const RigExecPointFrame *startRest =
        ctx.GetInputValuePtr<RigExecPointFrame>(_tokens->startRest);
    if (start && !startRest) {
        ctx.Warn("FkChain: rigExec:startFrame provider published no rest "
                 "frame; solving without it");
    }
    const bool hasStart = start && startRest;

    std::vector<rigExec::RigExecFkChainElement> elements;
    if (hasStart) {
        // Synthetic element 0: rest and pose of the start provider, so the
        // kernel's own rest->pose map for it IS S.
        rigExec::RigExecFkChainElement e;
        e.restPoints = startRest->points;
        e.posePoints = start->points;
        e.parentIndex = -1;
        elements.push_back(e);
    }
    // Where real element i lands in the solved array, and therefore what
    // "the element before me" and "chain root" mean. With no start frame
    // this is 0 and the indices below are exactly the historical
    // `index - 1` / `-1`.
    const int base = hasStart ? 1 : 0;

    VdfReadIterator<RigExecPointFrame> poseIt(ctx, _tokens->controlFrames);
    VdfReadIterator<RigExecPointFrame> restIt(ctx, _tokens->controlRests);
    int index = 0;
    for (; !poseIt.IsAtEnd() && !restIt.IsAtEnd(); ++poseIt, ++restIt) {
        rigExec::RigExecFkChainElement e;
        e.restPoints = (*restIt).points;
        e.posePoints = (*poseIt).points;
        e.parentIndex = parentRelative ? base - 1 : index + base - 1;
        elements.push_back(e);
        ++index;
    }

    RigExecPointFrameArray result;
    result.frames = rigExec::RigExecSolveFkChain(elements);
    // Discard the synthetic base: it is a solve input, not an output, and
    // publishing it would shift every joint's element index by one.
    if (hasStart && !result.frames.empty()) {
        result.frames.erase(result.frames.begin());
    }
    result.rests.reserve(elements.size() - size_t(base));
    for (size_t i = size_t(base); i < elements.size(); ++i) {
        result.rests.push_back(elements[i].restPoints);
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
                .Required(),
            // Optional by design: an unauthored relationship contributes no
            // input, and the kernel then runs the historical absolute path.
            Relationship(_tokens->startFrameRel)
                .TargetedObjects<RigExecPointFrame>(_tokens->computePointFrame)
                .InputName(_tokens->startFrame),
            Relationship(_tokens->startFrameRel)
                .TargetedObjects<RigExecPointFrame>(_tokens->computeRestFrame)
                .InputName(_tokens->startRest),
            AttributeValue<TfToken>(_tokens->controlSpace));
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
    const double *bend =
        ctx.GetInputValuePtr<double>(_tokens->preferredBendRadians);
    const float *stretch = ctx.GetInputValuePtr<float>(_tokens->inputsStretch);
    const float *softness =
        ctx.GetInputValuePtr<float>(_tokens->inputsSoftness);
    const double *upperOff =
        ctx.GetInputValuePtr<double>(_tokens->upperLengthOffset);
    const double *lowerOff =
        ctx.GetInputValuePtr<double>(_tokens->lowerLengthOffset);
    const double upperOffset = upperOff ? *upperOff : 0.0;
    const double lowerOffset = lowerOff ? *lowerOff : 0.0;
    params.preferredBendRadians = bend ? *bend : 0.0;
    params.stretch = stretch ? *stretch : 1.0;
    params.softness = softness ? *softness : 0.0;

    // Controls supply rest fallbacks while the solver is being wired. Once
    // a joint is bound, its live rest frame is the reference for that output.
    // In particular, the root joint's rest up controls the degenerate-pole
    // fallback: reading only the root control's rest made a joint rest edit
    // invisible to the solve even though the evaluator updated bone lengths.
    const RigExecPointFrame *rootRest =
        ctx.GetInputValuePtr<RigExecPointFrame>(_tokens->rootRest);
    const RigExecPointFrame *effectorRest =
        ctx.GetInputValuePtr<RigExecPointFrame>(_tokens->effectorRest);
    std::array<std::array<GfVec3d, 4>, 3> rests;
    rests[0] = rootRest ? rootRest->points : _IdentityLandmarks();
    const GfVec3d restAim =
        (rests[0][1] - rests[0][0]).GetNormalized();
    // Placeholder mid rest. Every bound joint's rest overwrites this
    // below, and the solve bails if all three are not bound, so this
    // only has to be a well-formed frame on the root's aim.
    rests[1] = rests[0];
    for (auto &p : rests[1]) {
        p += restAim;
    }
    rests[2] = effectorRest ? effectorRest->points : _IdentityLandmarks();

    // Which of [root, mid, end] a bound joint actually supplied a rest
    // for; the control-derived defaults above are not measurable bones.
    bool seen[3] = {false, false, false};
    VdfReadIterator<RigExecPointFrame> jointRestIt(
        ctx, _tokens->jointRests);
    VdfReadIterator<int> elementIt(ctx, _tokens->jointElements);
    const bool remapped = elementIt.ComputeSize() != 0;
    if (remapped && elementIt.ComputeSize() != jointRestIt.ComputeSize()) {
        ctx.Warn("TwoBoneIk: joint/rest element cardinality mismatch");
        return result;
    }
    size_t jointIndex = 0;
    for (; !jointRestIt.IsAtEnd(); ++jointRestIt, ++jointIndex) {
        const int element = remapped ? *elementIt : int(jointIndex);
        if (remapped) {
            ++elementIt;
        }
        if (element < 0 || element >= 3) {
            ctx.Warn("TwoBoneIk: joint element %d is out of range", element);
            return result;
        }
        rests[element] = jointRestIt->points;
        seen[element] = true;
    }

    // Bone lengths are MEASURED here, from the rests of the joints this
    // solver names, plus the authored offsets. There is no absolute
    // length attribute to author or to inject: rigExec:joints is the
    // single declaration of the chain, and a solver whose joints are
    // posed by a downstream consumer (an IK feeding an IK/FK blend)
    // still names them for their rests -- the claim check treats a
    // consumed solver's rigExec:joints as a rest reference, not an
    // output claim, precisely so this measurement has an input.
    if (!seen[0] || !seen[1] || !seen[2]) {
        ctx.Warn("TwoBoneIk: needs three bound joint rests to measure its "
                 "bone lengths; rigExec:joints binds %d of 3",
                 int(seen[0]) + int(seen[1]) + int(seen[2]));
        return result;
    }
    params.upperLength =
        (rests[1][0] - rests[0][0]).GetLength() + upperOffset;
    params.lowerLength =
        (rests[2][0] - rests[1][0]).GetLength() + lowerOffset;

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
            Relationship(_tokens->joints)
                .TargetedObjects<RigExecPointFrame>(_tokens->computeRestFrame)
                .InputName(_tokens->jointRests),
            AttributeValue<int>(_tokens->jointElements),
            AttributeValue<double>(_tokens->upperLengthOffset),
            AttributeValue<double>(_tokens->lowerLengthOffset),
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
    // Clamp to [0, 1]: a blend weight outside the unit interval extrapolates
    // past both inputs, which is never what a blend means.
    //
    // This is a BOUND, not the author's clamp. RigExecFloatMathMover is
    // evaluated now, and the reference assets' ClampIKFKWeight /
    // ClampBlendWeight movers reach this computation as a value override on
    // inputs:weight -- so the authored clamp is what shapes the weight, and
    // this line only catches a raw authored weight that no mover bounds.
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

    // Drained here, defaulted there: the read iterator is the ctx half, the
    // "nothing authored" rule is the half the bake must share.
    std::vector<double> weights;
    VdfReadIterator<float> wIt(ctx, _tokens->weights);
    for (; !wIt.IsAtEnd(); ++wIt) {
        weights.push_back(*wIt);
    }
    const int *count = ctx.GetInputValuePtr<int>(_tokens->count);
    rigExec::RigExecResolveTwistWeights(count ? *count : 1, &weights);

    return rigExec::RigExecSolveTwistDistribution(*start, *end, sRest, eRest,
        weights, _ScalarInput(ctx, _tokens->twistTurns, 0));
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
            AttributeValue<int>(_tokens->count),
            AttributeValue<double>(_tokens->twistTurns));
}

// ---------------------------------------------------------------------------
// RigExecSplineIk: control-driven spline IK over an ordered joint chain.
// Three control frames shape the curve; the chain named on rigExec:joints
// supplies the rest CVs and the rest spacing, and receives one frame per
// entry. All of it is pose-phase: the curve is never scene data, so a
// control-driven pose reaches it directly (contrast RigExecRibbon, whose
// native driver curve cannot see mover output).
// ---------------------------------------------------------------------------

static RigExecPointFrameArray
_ComputeSplineIk(const VdfContext &ctx)
{
    RigExecPointFrameArray result;

    const RigExecPointFrame *root =
        ctx.GetInputValuePtr<RigExecPointFrame>(_tokens->rootFrame);
    const RigExecPointFrame *mid =
        ctx.GetInputValuePtr<RigExecPointFrame>(_tokens->midFrame);
    const RigExecPointFrame *end =
        ctx.GetInputValuePtr<RigExecPointFrame>(_tokens->endFrame);
    if (!root || !mid || !end) {
        return result;
    }
    // The rest->pose maps that carry the CVs need the controls' rest
    // frames. Every control publishes computeRestFrame, so a missing one
    // means an unwired relationship rather than a value to default.
    const RigExecPointFrame *rootRest =
        ctx.GetInputValuePtr<RigExecPointFrame>(_tokens->rootRest);
    const RigExecPointFrame *midRest =
        ctx.GetInputValuePtr<RigExecPointFrame>(_tokens->midRest);
    const RigExecPointFrame *endRest =
        ctx.GetInputValuePtr<RigExecPointFrame>(_tokens->endRest);
    if (!rootRest || !midRest || !endRest) {
        ctx.Warn("SplineIk: root, mid, and end controls must each publish "
                 "a rest frame");
        return result;
    }

    // Joint rests in chain-slot order. The chain IS the cardinality: one
    // aggregate element per rigExec:joints entry, and a jointElements
    // remap is a permutation of those slots (the compile-time claim check
    // enforces the same shape; this is the runtime counterpart).
    VdfReadIterator<RigExecPointFrame> jointRestIt(ctx, _tokens->jointRests);
    VdfReadIterator<int> elementIt(ctx, _tokens->jointElements);
    const size_t count = jointRestIt.ComputeSize();
    if (count == 0) {
        ctx.Warn("SplineIk: rigExec:joints binds no joints; there is no "
                 "chain to measure rest CVs from");
        return result;
    }
    const bool remapped = elementIt.ComputeSize() != 0;
    if (remapped && elementIt.ComputeSize() != count) {
        ctx.Warn("SplineIk: joint/rest element cardinality mismatch");
        return result;
    }
    std::vector<RigExecPointFrame> restJoints(count);
    std::vector<bool> filled(count, false);
    size_t jointIndex = 0;
    for (; !jointRestIt.IsAtEnd(); ++jointRestIt, ++jointIndex) {
        const int slot = remapped ? *elementIt : int(jointIndex);
        if (remapped) {
            ++elementIt;
        }
        if (slot < 0 || size_t(slot) >= count) {
            ctx.Warn("SplineIk: joint element %d is out of range", slot);
            return result;
        }
        if (filled[slot]) {
            ctx.Warn("SplineIk: chain slot %d is filled twice", slot);
            return result;
        }
        restJoints[slot] = *jointRestIt;
        filled[slot] = true;
    }

    // Per-joint volume weights, parallel to the chain slots; empty means
    // no thinning anywhere.
    std::vector<double> weights;
    VdfReadIterator<float> wIt(ctx, _tokens->volumeWeights);
    for (; !wIt.IsAtEnd(); ++wIt) {
        weights.push_back(*wIt);
    }
    if (!weights.empty() && weights.size() != count) {
        ctx.Warn("SplineIk: rigExec:volumeWeights has %zu entries for a "
                 "%zu-joint chain", weights.size(), count);
        return result;
    }

    const TfToken *restTok =
        ctx.GetInputValuePtr<TfToken>(_tokens->restLength);
    rigExec::RigExecSplineIkRestLength restLength =
        rigExec::RigExecSplineIkRestLength::Curve;
    if (restTok && *restTok == "chain") {
        restLength = rigExec::RigExecSplineIkRestLength::Chain;
    } else if (restTok && !restTok->IsEmpty() && *restTok != "curve") {
        ctx.Warn("SplineIk: unsupported restLength '%s'", restTok->GetText());
        return result;
    }

    // The rest description is rebuilt every evaluation, exactly as
    // TwoBoneIk re-measures its bones: a rest edit on a bound joint or a
    // control re-shapes the rest curve with no recompile.
    const rigExec::RigExecSplineIkRest rest = rigExec::RigExecSplineIkMakeRest(
        restJoints, *rootRest, *midRest, *endRest, weights, restLength);

    rigExec::RigExecSplineIkParams params;
    params.preserveVolume =
        _ScalarInput(ctx, _tokens->inputsPreserveVolume, 1.0);
    params.midFollowWeight =
        _ScalarInput(ctx, _tokens->inputsMidFollowWeight, 0.5);
    // The schema attributes are degrees (an animator-facing angle, like
    // the avars); the kernel is radians.
    params.roll =
        GfDegreesToRadians(_ScalarInput(ctx, _tokens->inputsRoll, 0.0));
    params.twist =
        GfDegreesToRadians(_ScalarInput(ctx, _tokens->inputsTwist, 0.0));
    // Length floor as a fraction of the rest chord; 0 (the schema
    // default) is off, so an asset authored before it existed solves
    // exactly as it did.
    params.minLengthRatio =
        _ScalarInput(ctx, _tokens->inputsMinLengthRatio, 0.0);
    // rigid (the schema default) carries cv1 with the root control; aim
    // turns it onto the chord to the (floored) end, the conventional neck.
    const TfToken *tangentTok =
        ctx.GetInputValuePtr<TfToken>(_tokens->rootTangent);
    if (tangentTok && *tangentTok == "aim") {
        params.aimRootTangent = true;
    } else if (tangentTok && !tangentTok->IsEmpty() &&
               *tangentTok != "rigid") {
        ctx.Warn("SplineIk: unsupported rootTangent '%s'",
                 tangentTok->GetText());
        return result;
    }

    rigExec::RigExecSplineIkControls controls;
    controls.root = *root;
    controls.mid = *mid;
    controls.end = *end;

    // A degenerate solve (collapsed curve, singular rest control) still
    // returns one finite frame per joint, flagged degenerate, so the
    // failure propagates through extraction instead of vanishing into an
    // identity frame. Only a shape mismatch (already rejected above)
    // clears the result.
    rigExec::RigExecSplineIkResult solved;
    rigExec::RigExecSolveSplineIk(rest, controls, params, &solved);
    if (solved.joints.size() != count) {
        ctx.Warn("SplineIk: solve produced %zu frames for %zu joints",
                 solved.joints.size(), count);
        return result;
    }
    result.frames.reserve(count);
    result.rests.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        // The Y/Z handles carry (1, s, s): element extraction measures
        // posed/rest handle-length ratios per axis, so the non-uniform
        // squash survives into the joint's matrix (frameExtraction.h).
        result.frames.push_back(solved.joints[i].frame);
        result.rests.push_back(restJoints[i].points);
    }
    return result;
}

EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(RigExecSplineIk)
{
    self.PrimComputation(_tokens->computePointFrameArray)
        .Callback<RigExecPointFrameArray>(&_ComputeSplineIk)
        .Inputs(
            Relationship(_tokens->rootControl)
                .TargetedObjects<RigExecPointFrame>(_tokens->computePointFrame)
                .InputName(_tokens->rootFrame)
                .Required(),
            Relationship(_tokens->rootControl)
                .TargetedObjects<RigExecPointFrame>(_tokens->computeRestFrame)
                .InputName(_tokens->rootRest),
            Relationship(_tokens->midControl)
                .TargetedObjects<RigExecPointFrame>(_tokens->computePointFrame)
                .InputName(_tokens->midFrame)
                .Required(),
            Relationship(_tokens->midControl)
                .TargetedObjects<RigExecPointFrame>(_tokens->computeRestFrame)
                .InputName(_tokens->midRest),
            Relationship(_tokens->endControl)
                .TargetedObjects<RigExecPointFrame>(_tokens->computePointFrame)
                .InputName(_tokens->endFrame)
                .Required(),
            Relationship(_tokens->endControl)
                .TargetedObjects<RigExecPointFrame>(_tokens->computeRestFrame)
                .InputName(_tokens->endRest),
            Relationship(_tokens->joints)
                .TargetedObjects<RigExecPointFrame>(_tokens->computeRestFrame)
                .InputName(_tokens->jointRests),
            AttributeValue<int>(_tokens->jointElements),
            AttributeValue<float>(_tokens->volumeWeights),
            AttributeValue<TfToken>(_tokens->restLength),
            AttributeValue<double>(_tokens->inputsPreserveVolume),
            AttributeValue<double>(_tokens->inputsMidFollowWeight),
            AttributeValue<double>(_tokens->inputsRoll),
            AttributeValue<double>(_tokens->inputsTwist),
            AttributeValue<double>(_tokens->inputsMinLengthRatio),
            AttributeValue<TfToken>(_tokens->rootTangent));
}
