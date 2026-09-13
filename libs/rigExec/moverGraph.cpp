//
// RigExec compiled mover graph (spec §7.2). See moverGraph.h.
//
#include "moverGraph.h"
#include "curvenetAdjuster.h"
#include "parallel.h"

#include "rigExecMath/geometryKernels.h"
#include "rigExecMath/envelope.h"
#include "rigExecMath/pointFrame.h"
#include "rigExecMath/simdKernels.h"
#include "rigExecMath/solvers.h"

#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/hash.h"
#include "pxr/base/work/loops.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usdGeom/mesh.h"
#include "pxr/base/tf/staticTokens.h"
#include "pxr/exec/vdf/connectorSpecs.h"
#include "pxr/exec/vdf/context.h"
#include "pxr/exec/vdf/dataManagerVector.h"
#include "pxr/exec/vdf/executor.h"
#include "pxr/exec/vdf/inputVector.h"
#include "pxr/exec/vdf/input.h"
#include "pxr/exec/vdf/mask.h"
#include "pxr/exec/vdf/node.h"
#include "pxr/exec/vdf/readIterator.h"
#include "pxr/exec/vdf/readWriteIterator.h"
#include "pxr/exec/vdf/pullBasedExecutorEngine.h"
#include "pxr/exec/vdf/request.h"
#include "pxr/exec/vdf/schedule.h"
#include "pxr/exec/vdf/scheduler.h"
#include "pxr/exec/vdf/tokens.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

TF_DEFINE_PRIVATE_TOKENS(
    _tokens,
    ((previous, "previous"))
    ((parameters, "parameters"))
    ((status, "status"))
    ((out, "out"))
);

// The packet kind RigExecAssembleParameters stamps on each operation. The
// revision kernels check it before they run: a packet assembled for one
// operation arriving at another's kernel is a compile bug, and the revision
// passes through rather than running the wrong maths on it.
TF_DEFINE_PRIVATE_TOKENS(
    _kindTokens,
    ((matrix, "matrix"))
    ((skin, "skin"))
    ((blendShape, "blendShape"))
    ((volumeCorrect, "volumeCorrect"))
    ((smooth, "smooth"))
    ((lattice, "lattice"))
    ((surfaceProject, "surfaceProject"))
    ((ribbon, "ribbon"))
    ((emitGuidePoints, "emitGuidePoints"))
    ((curvenet, "curvenet"))
    ((curvenetAdjuster, "curvenetAdjuster"))
    ((recomputeNormals, "recomputeNormals"))
    ((recomputeExtent, "recomputeExtent"))
);

namespace rigExec {

namespace {

// One revision of an exact native point3f[] target.
//
// The connector shape is VDF's own idiom for this: `previous` is a READWRITE
// connector associated with `out`, so an unmodified revision passes its input
// straight through (SetOutputToReferenceInput) with no copy, and a modified one
// writes in place. That is precisely the pass-through contract disabled and
// failed movers rely on (spec §6.6).
class _RevisionNode final : public VdfNode
{
public:
    _RevisionNode(VdfNetwork *network, RigExecRevisionOp op,
                  size_t *executionCount)
        : VdfNode(
              network,
              VdfInputSpecs()
                  .ReadConnector<RigExecMoverParameters>(_tokens->parameters)
                  .ReadConnector<RigExecMoverStatus>(_tokens->status)
                  .ReadWriteConnector<GfVec3f>(_tokens->previous, _tokens->out),
              VdfOutputSpecs()
                  .Connector<GfVec3f>(_tokens->out))
        , _op(op)
        , _executionCount(executionCount)
    {
    }

    void Compute(const VdfContext &ctx) const override;
    const RigExecMoverStatus &GetStatus() const { return _resultStatus; }
    const std::vector<GfMatrix4d> &GetControlFrames() const { return _controlFrames; }

private:
    mutable RigExecMoverStatus _resultStatus;
    mutable std::vector<GfMatrix4d> _controlFrames;
    RigExecRevisionOp _op;
    size_t *const _executionCount;
};

bool
_StatusAllowsApply(const VdfContext &ctx)
{
    const RigExecMoverStatus *status =
        ctx.GetInputValuePtr<RigExecMoverStatus>(_tokens->status);
    return status && status->AllowsApply();
}

// The kind token an assembled packet must carry to be the packet for \p op.
//
// One table, read by the revision node's guard and by the shared kernel
// entry point, so the two cannot disagree about which packet belongs to
// which operation.
const TfToken &
_RevisionKindToken(RigExecRevisionOp op)
{
    switch (op) {
    case RigExecRevisionOp::Matrix:
        return _kindTokens->matrix;
    case RigExecRevisionOp::Skin:
        return _kindTokens->skin;
    case RigExecRevisionOp::BlendShape:
        return _kindTokens->blendShape;
    case RigExecRevisionOp::VolumeCorrect:
        return _kindTokens->volumeCorrect;
    case RigExecRevisionOp::Smooth:
        return _kindTokens->smooth;
    case RigExecRevisionOp::Lattice:
        return _kindTokens->lattice;
    case RigExecRevisionOp::SurfaceProject:
        return _kindTokens->surfaceProject;
    case RigExecRevisionOp::Ribbon:
        return _kindTokens->ribbon;
    case RigExecRevisionOp::EmitGuidePoints:
        return _kindTokens->emitGuidePoints;
    case RigExecRevisionOp::Curvenet:
        return _kindTokens->curvenet;
    case RigExecRevisionOp::CurvenetAdjuster:
        return _kindTokens->curvenetAdjuster;
    case RigExecRevisionOp::RecomputeNormals:
        return _kindTokens->recomputeNormals;
    case RigExecRevisionOp::RecomputeExtent:
        return _kindTokens->recomputeExtent;
    }
    // No runtime dispatch beyond the frozen operation set: an unhandled op is
    // a build error, not a silently mismatched packet.
    static const TfToken unknown;
    return unknown;
}

// Shared scratch-collect / kernel / write-back body for every revision
// (spec §6.5: transient scratch is released before the callback returns).
// Peer of _EvaluateScratchKernel in moverKernels.cpp, with the write-back
// changed from Allocate to in-place through the READWRITE connector.
//
// Everything between the collect and the write-back is
// RigExecRunRevisionKernel: the kind check, the full-strength fast path, the
// operation itself and the "apply once" blend. The baked geometry loop calls
// that same function with the same packet, so the node is the only place the
// VdfContext appears and neither path holds a copy of the other's dispatch.
void
_RunRevisionOp(const VdfContext &ctx, RigExecRevisionOp op,
               RigExecMoverStatus *resultStatus,
               std::vector<GfMatrix4d> *controlFrames)
{
    const RigExecMoverParameters *params =
        ctx.GetInputValuePtr<RigExecMoverParameters>(_tokens->parameters);
    auto passThrough = [&ctx, resultStatus]() {
        if (_StatusAllowsApply(ctx)) {
            resultStatus->state = TfToken("moverFailed");
        }
        ctx.SetOutputToReferenceInput(_tokens->previous);
    };
    // RigExecRunRevisionKernel owns the packet check; repeating it here is
    // what keeps a disabled, failed or mis-assembled revision from paying for
    // the scratch copy it is about to throw away, exactly as before the
    // kernel bodies moved out of this callback.
    if (!_StatusAllowsApply(ctx) || !params || !params->valid ||
        params->kind != _RevisionKindToken(op)) {
        passThrough();
        return;
    }

    // Every kernel reads the preceding points as a contiguous array, so the
    // callback materialises one for all of them. The blend-shape revision
    // used to stream through the READWRITE connector instead and now pays a
    // copy of the points it already allocates an envelope for; deciding here
    // which ops could still stream would put back into the node exactly the
    // per-operation knowledge this hoist took out of it.
    std::vector<GfVec3f> scratch;
    {
        VdfReadIterator<GfVec3f> previous(ctx, _tokens->previous);
        scratch.reserve(previous.ComputeSize());
        for (; !previous.IsAtEnd(); ++previous) {
            scratch.push_back(*previous);
        }
    }
    if (!RigExecRunRevisionKernel(op, *params, &scratch, controlFrames)) {
        passThrough();
        return;
    }

    // The revision writes in place. Unlike the generated-application kernel --
    // which allocated a fresh output buffer because its expression output had
    // no associated input -- a READWRITE connector hands the input buffer
    // straight through as the output, so constructing the iterator on
    // `previous` is what grants write access. Allocating here instead would
    // fail ("output cannot hold a boxed value") and silently pass through.
    VdfReadWriteIterator<GfVec3f> out(ctx, _tokens->previous);
    size_t i = 0;
    for (; !out.IsAtEnd() && i < scratch.size(); ++out, ++i) {
        *out = scratch[i];
    }
}

void
_RevisionNode::Compute(const VdfContext &ctx) const
{
    ++*_executionCount;
    const auto *status = ctx.GetInputValuePtr<RigExecMoverStatus>(_tokens->status);
    _resultStatus = status ? *status : RigExecMoverStatus{TfToken("moverFailed"), {}};
    // Node state that outlives one Compute, so a revision that passes through
    // must not leave the frames of the last one that did not.
    _controlFrames.clear();
    _RunRevisionOp(ctx, _op, &_resultStatus, &_controlFrames);
}

}  // namespace

// The matrix kernel, shared by the mover-graph revision node and by the
// baked program. Unlike the point3f[] ops the envelope is NOT a separate
// blend here: the weighted-matrix rule folds it into the movement itself
// (p' = q + w (T q - q)), so resolving it is part of the kernel.
//
// One definition, so a second caller cannot drift into a different movement
// -- including over the SIMD choice, which must be the same on both paths or
// the two disagree in the last bits.
bool
RigExecApplyMatrixKernel(const RigExecMoverParameters &p,
                         std::vector<GfVec3f> *pts)
{
    const size_t count = pts->size();
    std::vector<float> weights(count);
    if (!p.weights.ResolveAll(count, &weights)) {
        return false;  // cardinality mismatch fails atomically
    }
    static const bool useSimd = TfGetenvBool("RIGEXEC_ENABLE_SIMD", true);
    if (useSimd) {
        RigExecApplyWeightedMatrixSimd(
            pts->data(), pts->data(), weights.data(), count, p.transform);
    } else {
        // Element i is written only after it is read, so in-place is safe.
        for (size_t i = 0; i < count; ++i) {
            (*pts)[i] = GfVec3f(RigExecApplyWeightedMatrix(
                GfVec3d((*pts)[i]), p.transform, weights[i]));
        }
    }
    return true;
}

// The skin kernel, shared by the mover-graph revision node and by the
// baked program, which runs the same operation with no VdfNetwork around
// it. One definition, so a second caller cannot drift into a different
// deformation.
bool
RigExecApplySkinKernel(const RigExecMoverParameters &p,
                       std::vector<GfVec3f> *pts)
{
    // The incoming revision IS the rest pose the influences were bound
    // against. A blend shape upstream of the skin is skinned -- exactly as a
    // skinCluster skins its input geometry and UsdSkel applies blend shapes
    // before skinning -- and when the skin is first in its chain the incoming
    // points are the authored base. No separate rest input is needed for
    // that, and every skinning method reads the same gather below.
    RigExecSkinLayout layout;
    layout.transforms = p.skinTransforms.data();
    layout.transformCount = p.skinTransforms.size();
    const RigExecSkinTopology *const topology = p.skinTopology.get();
    if (topology) {
        layout.indices = topology->indices.data();
        layout.weights = topology->weights.data();
        layout.indexCount = topology->indices.size();
        layout.elementSize =
            topology->elementSize < 1 ? 0 : size_t(topology->elementSize);
    } else {
        layout.indices = p.skinIndices.data();
        layout.weights = p.skinWeights.data();
        layout.indexCount = p.skinIndices.size();
        layout.elementSize =
            p.skinElementSize < 1 ? 0 : size_t(p.skinElementSize);
    }
    layout.pointCount = pts->size();
    if (topology) {
        // O(1). The epoch checked the element shape, the index range and the
        // weights; the assembler checked THIS frame's matrices and would have
        // failed the packet otherwise, so the only question left is whether
        // the points that arrived are the points the layout describes --
        // which is the one thing the assembler could not know.
        if (!topology->validated ||
            topology->influenceCount != layout.transformCount ||
            layout.indexCount != layout.pointCount * layout.elementSize) {
            return false;
        }
    } else if (p.skinWeights.size() != p.skinIndices.size() ||
               !layout.Validate()) {
        return false;  // cardinality mismatch fails atomically
    }
    static const bool useSimd = TfGetenvBool("RIGEXEC_ENABLE_SIMD", true);

    // ---- Method dispatch ---------------------------------------------
    // The one point where the skinning methods part. Everything above is
    // the shared per-point gather (indices, weights, influence matrices,
    // rest point); only the accumulation differs.
    if (p.skinningMethod == "classicLinear") {
        // sum_k w_k T_k p, with the weight complement held at the rest
        // point (see RigExecApplyLinearBlendSkin).
        const auto runLbs =
            [&layout](GfVec3f *points, size_t begin, size_t end) {
            // A point range IS a layout: the indices and weights of point i
            // live at i * elementSize and nowhere else, and no point reads
            // another's result. Both kernels loop one point at a time, so a
            // split boundary cannot land inside a vectorised block either --
            // which is what makes the parallel result bit-identical to the
            // serial one rather than merely equal to tolerance.
            RigExecSkinLayout part = layout;
            part.indices = layout.indices + begin * layout.elementSize;
            part.weights = layout.weights + begin * layout.elementSize;
            part.indexCount = (end - begin) * layout.elementSize;
            part.pointCount = end - begin;
            if (useSimd) {
                RigExecApplyLinearBlendSkinSimd(
                    points + begin, points + begin, part);
            } else {
                RigExecApplyLinearBlendSkin(
                    points + begin, points + begin, part);
            }
        };
        GfVec3f *const points = pts->data();
        if (RigExecParallelEvaluationEnabled() &&
            layout.pointCount >= RigExecGeometryParallelThreshold) {
            WorkParallelForN(
                layout.pointCount,
                [&runLbs, points](size_t begin, size_t end) {
                    runLbs(points, begin, end);
                },
                RigExecGeometryGrainSize);
        } else {
            runLbs(points, 0, layout.pointCount);
        }
        return true;
    }
    if (p.skinningMethod == "dualQuaternion") {
        // Scale-aware DQS from libs/rigExecMath/dualQuat.h: each influence
        // split once per evaluation into a pre-rotation stretch and a unit
        // dual quaternion, weighted sum over the same layout with
        // shortest-arc sign correction, ONE normalisation, then the direct
        // point transform. Weight shortfall enters as an identity influence
        // (see RigExecApplyDualQuatSkin). Scalar only; a degenerate blend
        // fails atomically.
        return RigExecApplyDualQuatSkin(pts->data(), pts->data(), layout);
    }
    // A token neither kernel owns is a compile error upstream; a packet that
    // reaches here anyway fails the application rather than silently running
    // the wrong maths.
    return false;
}

// The blend-shape kernel, shared by the mover-graph revision node and by the
// baked program. Like the matrix kernel and unlike the point3f[] ops the
// envelope is NOT a separate blend: the deltas are added to the preceding
// revision and the result blended back against it in one pass, so resolving
// the envelope is part of the kernel.
//
// One definition, so a second caller cannot drift into a different blend.
bool
RigExecApplyBlendShapeKernel(const RigExecMoverParameters &p,
                             std::vector<GfVec3f> *pts)
{
    const size_t count = pts->size();
    if (p.blendDeltas.size() != count) {
        return false;
    }
    // Resolve the common envelope up front so a cardinality failure fails the
    // application before any element is written (the in-place write cannot be
    // rolled back).
    std::vector<float> envelope;
    if (!p.weights.ResolveAll(count, &envelope)) {
        return false;
    }

    std::vector<GfVec3f> transported;
    const std::vector<GfVec3f> *deltas = &p.blendDeltas;
    if (p.blendSurfaceFrame) {
        // The incoming points ARE the posed surface the offsets ride on.
        if (!RigExecTransportSurfaceOffsets(p.restPoints, *pts,
                p.topologyCounts, p.topologyIndices, p.blendDeltas,
                &transported)) {
            return false;
        }
        deltas = &transported;
    }

    for (size_t i = 0; i < count; ++i) {
        const GfVec3f preceding = (*pts)[i];
        (*pts)[i] = RigExecBlendEnvelope(
            preceding, preceding + (*deltas)[i], envelope[i]);
    }
    return true;
}

// The derived-maintenance kernel, shared by the mover-graph revision node and
// by the baked program: normal3f[] and float3[] hosts recomputed from the
// final same-generation points rather than from the preceding revision
// (spec §7.6). Self-enveloping, like the matrix and blend-shape kernels.
//
// One definition, so the size rules and the envelope cannot drift between the
// two paths that maintain the same property.
bool
RigExecApplyDerivedKernel(RigExecRevisionOp op,
                          const RigExecMoverParameters &p,
                          std::vector<GfVec3f> *pts)
{
    const bool extent = op == RigExecRevisionOp::RecomputeExtent;
    const std::vector<GfVec3f> values =
        extent ? RigExecComputeExtent(p.auxPoints, p.widths)
               : RigExecComputeVertexNormals(p.auxPoints, p.topologyCounts,
                                             p.topologyIndices);
    if (values.empty() || (extent && values.size() != 2)) {
        return false;
    }

    // The derived property keeps its authored cardinality: writing in place
    // cannot resize it, so a recomputation that disagrees fails the
    // application rather than truncating.
    if (pts->size() != values.size()) {
        return false;
    }
    std::vector<float> envelope;
    if (!p.weights.ResolveAll(values.size(), &envelope)) {
        return false;
    }

    for (size_t i = 0; i < values.size(); ++i) {
        (*pts)[i] = RigExecBlendEnvelope((*pts)[i], values[i], envelope[i]);
    }
    return true;
}

// Every revision operation, over the same kernels the revision node ran when
// they were lambdas inside its VdfContext callback. \p controlFrames receives
// the curvenet adjuster's fully adjusted control frames and is unread by every
// other operation.
//
// ONE definition, called by the mover-graph revision node and by the baked
// program: a second copy of a deformation agrees on the fixtures that exist
// and drifts on the ones that do not.
//
// The envelope is NOT applied here for the ops that take a separate blend --
// RigExecRunRevisionKernel wraps this, which is where the "apply once" rule
// lives; matrix, blendShape and the two derived recomputations fold it into
// their own arithmetic and are routed there instead.
bool
RigExecApplyRevisionKernel(RigExecRevisionOp op,
                           const RigExecMoverParameters &p,
                           std::vector<GfVec3f> *pts,
                           std::vector<GfMatrix4d> *controlFrames)
{
    switch (op) {
    case RigExecRevisionOp::Matrix:
        return RigExecApplyMatrixKernel(p, pts);
    case RigExecRevisionOp::Skin:
        return RigExecApplySkinKernel(p, pts);
    case RigExecRevisionOp::BlendShape:
        return RigExecApplyBlendShapeKernel(p, pts);
    case RigExecRevisionOp::VolumeCorrect:
        RigExecApplyVolumeCorrect(pts, p.referenceVolume, p.strength);
        return true;
    case RigExecRevisionOp::Smooth:
        RigExecApplyLaplacianSmooth(
            pts, p.topologyCounts, p.topologyIndices, p.strength);
        return true;
    case RigExecRevisionOp::Lattice:
        if (p.restPoints.size() != pts->size()) {
            return false;  // cardinality mismatch fails atomically
        }
        RigExecApplyLattice(
            pts, p.restPoints, p.auxPoints, p.auxPointsB, p.divisions);
        return true;
    case RigExecRevisionOp::SurfaceProject:
        RigExecApplySurfaceProject(
            pts, p.auxPoints, p.topologyCounts, p.topologyIndices,
            p.strength);
        return true;
    case RigExecRevisionOp::EmitGuidePoints:
        if (p.frames.GetSize() != pts->size()) {
            return false;
        }
        for (size_t i = 0; i < pts->size(); ++i) {
            (*pts)[i] = GfVec3f(p.frames.frames[i].Origin());
        }
        return true;
    case RigExecRevisionOp::Ribbon: {
        if (p.bindCoords.size() != pts->size()) {
            return false;
        }
        // Rest-relative rigid transport: per-sample maps from the
        // aggregate's rest frames to its posed frames (spec §7.5).
        const size_t n = p.frames.GetSize();
        if (n < 2) {
            return false;
        }
        std::vector<GfMatrix4d> maps(n);
        for (size_t k = 0; k < n; ++k) {
            if (!RigExecPointsToMatrix(
                    p.frames.rests[k], p.frames.frames[k].points,
                    &maps[k])) {
                maps[k].SetIdentity();
            }
        }
        for (size_t i = 0; i < pts->size(); ++i) {
            const float u =
                std::min(1.0f, std::max(0.0f, p.bindCoords[i][0]));
            const float s = u * float(n - 1);
            const size_t k = std::min(n - 2, size_t(s));
            const float t = s - float(k);
            const GfVec3d a = maps[k].TransformAffine(GfVec3d((*pts)[i]));
            const GfVec3d b = maps[k + 1].TransformAffine(GfVec3d((*pts)[i]));
            (*pts)[i] = GfVec3f(a + (b - a) * double(t));
        }
        return true;
    }
    case RigExecRevisionOp::CurvenetAdjuster: {
        if (!controlFrames) {
            return false;  // the adjuster's whole second output
        }
        const auto preceding = *pts;
        if (!RigExecApplyCurvenetAdjustments(pts, p.restPoints,
            p.topologyIndices, p.curvenetAdjustmentBasis,
            p.curvenetAdjustments, controlFrames)) return false;
        // The frames follow the UNBLENDED adjusted points, weighted per
        // control point; the points themselves are blended afterwards by
        // the wrapper, so the envelope still lands exactly once on each.
        for (size_t i = 0; i < controlFrames->size(); ++i) {
            const int point = p.curvenetAdjustments[i].pointIndex;
            const float weight = p.weights.Resolve(point, pts->size());
            (*controlFrames)[i].SetTranslateOnly(GfVec3d(
                preceding[point] + ((*pts)[point]-preceding[point])*weight));
        }
        return true;
    }
    case RigExecRevisionOp::Curvenet: {
        if (!p.curvenetBinding) {
            return false;
        }
        // The incoming points ARE the rest surface (§5): a curvenet
        // layered on top of skinning deforms from the skinned shape,
        // and when it is first in the chain they are the projection
        // pose and the solve takes its fast path.
        std::vector<GfVec3f> solved;
        std::string error;
        if (!RigExecEvaluateProfileMover(*p.curvenetBinding,
                                         p.auxPoints, *pts, p.strength,
                                         &solved, &error)) {
            return false;
        }
        if (solved.size() != pts->size()) {
            return false;
        }
        pts->swap(solved);
        return true;
    }
    case RigExecRevisionOp::RecomputeNormals:
    case RigExecRevisionOp::RecomputeExtent:
        return RigExecApplyDerivedKernel(op, p, pts);
    }
    // No runtime dispatch beyond the frozen operation set: an unhandled op is
    // a build error, not a silently skipped revision.
    return false;
}

// One revision, envelope included: the packet check, the full-strength fast
// path, RigExecApplyRevisionKernel and the "apply once" blend against the
// preceding revision.
//
// ONE definition of "apply once", called by the mover-graph revision node and
// by the baked geometry loop. Two hand-written wrappers would have to agree
// about which operations blend and which fold the envelope into their own
// arithmetic, and the rigs that would show a disagreement are the ones no
// fixture happened to have.
bool
RigExecRunRevisionKernel(RigExecRevisionOp op,
                         const RigExecMoverParameters &p,
                         std::vector<GfVec3f> *pts,
                         std::vector<GfMatrix4d> *controlFrames)
{
    if (!p.valid || p.kind != _RevisionKindToken(op)) {
        return false;
    }
    // Matrix, blendShape and the two derived recomputations resolve the
    // envelope inside their own arithmetic; blending their result again would
    // apply it twice.
    if (op == RigExecRevisionOp::Matrix ||
        op == RigExecRevisionOp::BlendShape ||
        op == RigExecRevisionOp::RecomputeNormals ||
        op == RigExecRevisionOp::RecomputeExtent) {
        return RigExecApplyRevisionKernel(op, p, pts, controlFrames);
    }

    // A constant envelope at exactly full strength makes the blend below the
    // identity at every point, so the copy of the preceding revision, the
    // resolved envelope array and the blend loop are all dead. The predicate
    // lives in moverGraph.h next to the packet it reads; see
    // RigExecEnvelopeIsFullStrength.
    const bool fullStrengthEnvelope = RigExecEnvelopeIsFullStrength(p.weights);
    const size_t precedingSize = pts->size();
    std::vector<GfVec3f> preceding;
    if (!fullStrengthEnvelope) {
        preceding = *pts;
    }
    if (!RigExecApplyRevisionKernel(op, p, pts, controlFrames)) {
        return false;
    }
    if (pts->size() != precedingSize) {
        return false;
    }
    if (!fullStrengthEnvelope) {
        std::vector<float> envelope;
        if (!p.weights.ResolveAll(pts->size(), &envelope)) {
            return false;
        }
        // Per point, reading two arrays and writing a third at the same
        // index: a point range is an independent sub-problem, so splitting
        // it changes nothing about the arithmetic.
        GfVec3f *const blended = pts->data();
        const GfVec3f *const before = preceding.data();
        const float *const strength = envelope.data();
        const size_t count = pts->size();
        if (RigExecParallelEvaluationEnabled() &&
            count >= RigExecGeometryParallelThreshold) {
            WorkParallelForN(
                count,
                [blended, before, strength](size_t begin, size_t end) {
                    for (size_t i = begin; i < end; ++i) {
                        blended[i] = RigExecBlendEnvelope(
                            before[i], blended[i], strength[i]);
                    }
                },
                RigExecGeometryGrainSize);
        } else {
            for (size_t i = 0; i < count; ++i) {
                blended[i] = RigExecBlendEnvelope(
                    before[i], blended[i], strength[i]);
            }
        }
    }
    return true;
}

bool
RigExecRevisionBinding::operator==(const RigExecRevisionBinding &o) const
{
    return moverPath == o.moverPath && target == o.target &&
           transform == o.transform && influences == o.influences &&
           weightObject == o.weightObject &&
           base == o.base && topologyCounts == o.topologyCounts &&
           topologyIndices == o.topologyIndices &&
           cagePoints == o.cagePoints && surfacePoints == o.surfacePoints &&
           bindCoords == o.bindCoords && driverFrames == o.driverFrames &&
           widths == o.widths && curvenet == o.curvenet &&
           curvenetPoints == o.curvenetPoints &&
           blendInputs == o.blendInputs && blendSamples == o.blendSamples &&
           phases == o.phases && transformPhase == o.transformPhase;
}

std::optional<RigExecRevisionOp>
RigExecRevisionOpForSchema(const TfToken &schemaType, const TfToken &curveMode)
{
    if (schemaType == "RigExecMatrixMover") {
        return RigExecRevisionOp::Matrix;
    }
    if (schemaType == "RigExecSkinMover") {
        return RigExecRevisionOp::Skin;
    }
    if (schemaType == "RigExecBlendShapeMover") {
        return RigExecRevisionOp::BlendShape;
    }
    if (schemaType == "RigExecVolumeCorrectMover") {
        return RigExecRevisionOp::VolumeCorrect;
    }
    if (schemaType == "RigExecSmoothMover") {
        return RigExecRevisionOp::Smooth;
    }
    if (schemaType == "RigExecLatticeMover") {
        return RigExecRevisionOp::Lattice;
    }
    if (schemaType == "RigExecSurfaceMover") {
        return RigExecRevisionOp::SurfaceProject;
    }
    if (schemaType == "RigExecCurvenetMover") {
        return RigExecRevisionOp::Curvenet;
    }
    if (schemaType == "RigExecCurvenetAdjusterMover") {
        return RigExecRevisionOp::CurvenetAdjuster;
    }
    if (schemaType == "RigExecCurveMover") {
        // The curve mover's frozen signature branches on its authored mode.
        return curveMode == "emitGuidePoints"
            ? RigExecRevisionOp::EmitGuidePoints
            : RigExecRevisionOp::Ribbon;
    }
    return std::nullopt;
}

std::shared_ptr<const RigExecProfileMoverBinding>
RigExecCurvenetBindCache::Resolve(
    const SdfPath &mover, const SdfPath &target, size_t digest,
    const std::function<bool(RigExecProfileMoverBinding *, std::string *)>
        &build,
    std::string *error)
{
    _Entry &entry = _entries[{mover, target}];
    if (entry.digest == digest && (entry.binding || !entry.error.empty())) {
        if (!entry.binding && error) {
            *error = entry.error;
        }
        return entry.binding;
    }

    entry.digest = digest;
    entry.binding.reset();
    entry.error.clear();
    auto built = std::make_shared<RigExecProfileMoverBinding>();
    std::string reason;
    if (!build(built.get(), &reason)) {
        // Remembered, so a rig with an unbindable curvenet reports once
        // instead of re-cutting the mesh on every frame.
        entry.error = reason.empty() ? "curvenet bind failed" : reason;
        if (error) {
            *error = entry.error;
        }
        return nullptr;
    }
    // Reported once per (re)bind, not per frame: the cache exists precisely
    // so this happens on a layout change and nowhere else.
    const RigExecCutMeshReport &cut = built->report;
    char summary[320];
    std::snprintf(
        summary, sizeof(summary),
        "curvenet bind %s -> %s: %d cut faces, %d samples, %d cracks, "
        "%d traced, %d unknowns, %zu factor nonzeros",
        mover.GetString().c_str(), target.GetString().c_str(),
        cut.cutFaceCount, cut.sampleCount, cut.crackCount, cut.tracedSegments,
        built->cutMesh.unknownCount,
        built->solver ? built->solver->GetFactorNonzeros() : size_t(0));
    _pending.push_back(summary);
    for (const std::string &warning : cut.warnings) {
        _pending.push_back("curvenet bind " + mover.GetString() + ": " +
                           warning);
    }

    entry.binding = std::move(built);
    return entry.binding;
}

std::vector<std::string>
RigExecCurvenetBindCache::TakeDiagnostics()
{
    std::vector<std::string> out;
    out.swap(_pending);
    return out;
}


std::string
RigExecReadPhase::GetAsString() const
{
    switch (kind) {
    case RigExecReadPhaseKind::Base:      return "base";
    case RigExecReadPhaseKind::Preceding: return "preceding";
    case RigExecReadPhaseKind::Final:     return "final";
    case RigExecReadPhaseKind::AtPrim:    return prim.GetString();
    }
    return "base";
}

bool
RigExecParseReadPhase(
    const std::string &authored, RigExecReadPhase *phase, std::string *error)
{
    if (!phase) {
        return false;
    }
    if (authored.empty() || authored == "base") {
        *phase = RigExecReadPhase{RigExecReadPhaseKind::Base, SdfPath()};
        return true;
    }
    if (authored == "preceding") {
        *phase = RigExecReadPhase{RigExecReadPhaseKind::Preceding, SdfPath()};
        return true;
    }
    if (authored == "final") {
        *phase = RigExecReadPhase{RigExecReadPhaseKind::Final, SdfPath()};
        return true;
    }
    // Anything else must be an absolute prim path. A relative path would have
    // to be resolved against something, and there are two equally plausible
    // somethings here (the mover, the target), so it is rejected rather than
    // guessed.
    if (!SdfPath::IsValidPathString(authored)) {
        if (error) {
            *error = "'" + authored +
                     "' is not base, preceding, final, or a valid prim path";
        }
        return false;
    }
    const SdfPath path(authored);
    if (!path.IsAbsolutePath() || !path.IsPrimPath()) {
        if (error) {
            *error = "'" + authored +
                     "' must be an ABSOLUTE prim path (or base, preceding, "
                     "final)";
        }
        return false;
    }
    *phase = RigExecReadPhase{RigExecReadPhaseKind::AtPrim, path};
    return true;
}

bool
RigExecResolveReadPhase(
    const UsdObject &property,
    const char *legacyAttribute,
    RigExecReadPhase *phase,
    std::string *error)
{
    if (!phase) {
        return false;
    }
    *phase = RigExecReadPhase();
    if (!property.IsValid()) {
        return true;
    }

    // Metadata on the property itself wins: it is the most specific place the
    // phase can be said, and the only one that works for an input with no
    // schema attribute of its own.
    std::string authored;
    if (property.GetMetadata(TfToken(RigExecReadPhaseMetadataName),
                             &authored) &&
        !authored.empty()) {
        std::string why;
        if (!RigExecParseReadPhase(authored, phase, &why)) {
            if (error) {
                *error = property.GetPath().GetString() + ": " +
                         RigExecReadPhaseMetadataName + " " + why;
            }
            return false;
        }
        return true;
    }

    // Then the role-named schema attribute, so every asset authored before the
    // metadata existed keeps meaning exactly what it meant.
    if (legacyAttribute) {
        const UsdPrim owner = property.GetPrim();
        if (const UsdAttribute a =
                owner.GetAttribute(TfToken(legacyAttribute))) {
            TfToken value;
            if (a.Get(&value) && !value.IsEmpty()) {
                std::string why;
                if (!RigExecParseReadPhase(value.GetString(), phase, &why)) {
                    if (error) {
                        *error = owner.GetPath().GetString() + ": " +
                                 legacyAttribute + " " + why;
                    }
                    return false;
                }
                return true;
            }
        }
    }
    return true;
}

void
RigExecChainSnapshots::Record(
    const SdfPath &target, const SdfPath &afterMover, const VtValue &value)
{
    _chains[target].revisions.emplace_back(afterMover, value);
}

void
RigExecChainSnapshots::RecordFinal(const SdfPath &target, const VtValue &value)
{
    _Chain &chain = _chains[target];
    chain.final = value;
    chain.hasFinal = true;
}

void
RigExecChainSnapshots::Merge(RigExecChainSnapshots &&other)
{
    for (auto &[target, chain] : other._chains) {
        _Chain &destination = _chains[target];
        destination.revisions.insert(
            destination.revisions.end(),
            std::make_move_iterator(chain.revisions.begin()),
            std::make_move_iterator(chain.revisions.end()));
        if (chain.hasFinal) {
            destination.final = std::move(chain.final);
            destination.hasFinal = true;
        }
    }
    other._chains.clear();
}

const VtValue *
RigExecChainSnapshots::Lookup(
    const SdfPath &target, const RigExecReadPhase &phase,
    const SdfPath &readerMover) const
{
    const auto it = _chains.find(target);
    if (it == _chains.end()) {
        return nullptr;
    }
    const _Chain &chain = it->second;

    switch (phase.kind) {
    case RigExecReadPhaseKind::Base:
        // The stage answers this one; nothing is recorded for it.
        return nullptr;

    case RigExecReadPhaseKind::Final:
        return chain.hasFinal ? &chain.final : nullptr;

    case RigExecReadPhaseKind::Preceding: {
        // The value going INTO the reader. Absent when the reader is the
        // chain's first revision, which is the authored base -- so returning
        // null correctly sends the caller to the stage.
        for (size_t i = 0; i < chain.revisions.size(); ++i) {
            if (chain.revisions[i].first == readerMover) {
                return i == 0 ? nullptr : &chain.revisions[i - 1].second;
            }
        }
        // The reader does not write this chain at all, so "preceding" has no
        // position to be relative to.
        return nullptr;
    }

    case RigExecReadPhaseKind::AtPrim: {
        // The LAST revision at or beneath the named prim. For a mover that is
        // the mover itself; for a Scope it is whatever ran last inside it,
        // which is what post-order makes that Scope mean.
        const VtValue *found = nullptr;
        for (const auto &[mover, value] : chain.revisions) {
            if (mover == phase.prim || mover.HasPrefix(phase.prim)) {
                found = &value;
            }
        }
        return found;
    }
    }
    return nullptr;
}

namespace {

SdfPathVector
_Targets(const UsdPrim &prim, const char *rel)
{
    SdfPathVector targets;
    if (const UsdRelationship r = prim.GetRelationship(TfToken(rel))) {
        r.GetTargets(&targets);
    }
    return targets;
}

TfToken
_Token(const UsdPrim &prim, const char *attr, const char *fallback)
{
    TfToken value(fallback);
    if (const UsdAttribute a = prim.GetAttribute(TfToken(attr))) {
        a.Get(&value);
    }
    return value;
}

// A PointBased prim binds to its .points property by the standard rule; an
// exact property path is already canonical.
SdfPath
_PointsOf(const SdfPath &path)
{
    return path.IsPrimPath() ? path.AppendProperty(TfToken("points")) : path;
}

// Every scalar mover input consults the generation's resolved property set
// before the authored stage. This is what lets a property-domain mover drive
// another mover's common envelope without a second evaluation model.
float
_Float(const UsdPrim &prim, const char *attr, float fallback,
       UsdTimeCode time, const RigExecResolvedInputs *resolved)
{
    float value = fallback;
    if (const UsdAttribute a = prim.GetAttribute(TfToken(attr))) {
        if (resolved && resolved->GetAttribute(a, time, &value)) {
            return value;
        }
        a.Get(&value, time);
    }
    return value;
}

}  // namespace

std::shared_ptr<const RigExecSkinTopology>
RigExecSkinTopologyCache::Resolve(
    const SdfPath &mover,
    const std::function<bool(RigExecSkinTopology *)> &build)
{
    // Held across the build as well as the lookup: concurrent chain tasks
    // would otherwise insert into the same map at once, and building the
    // same layout twice would only waste the read.
    std::lock_guard<std::mutex> lock(_mutex);
    const auto found = _entries.find(mover);
    if (found != _entries.end()) {
        // Including a remembered refusal, which is a null entry.
        return found->second;
    }
    auto built = std::make_shared<RigExecSkinTopology>();
    if (!build(built.get())) {
        _entries.emplace(mover, nullptr);
        return nullptr;
    }
    // The layout this mover had before the last Clear(). If the fresh read
    // produced the same arrays -- which is what an edit anywhere else on the
    // stage produces -- hand back the SAME pointer, so the mover's packet
    // still compares equal and the per-point kernel does not re-run for a
    // binding that did not move. One array compare per notice, against one
    // kernel pass per notice.
    const auto candidate = _candidates.find(mover);
    if (candidate != _candidates.end() && candidate->second &&
        *candidate->second == *built) {
        return _entries.emplace(mover, candidate->second).first->second;
    }
    return _entries.emplace(mover, std::move(built)).first->second;
}

RigExecRevisionBinding
RigExecResolveRevisionBinding(
    const UsdPrim &moverPrim,
    const SdfPath &target,
    const std::map<SdfPath, SdfPath> &frameChainHeads)
{
    RigExecRevisionBinding binding;
    if (!moverPrim) {
        return binding;
    }
    binding.moverPath = moverPrim.GetPath();
    binding.target = target;

    const TfToken schemaType = moverPrim.GetTypeName();
    const SdfPath ownerPath = target.GetPrimPath();

    // A phase declared on the relationship that NAMES an input governs that
    // input. Recorded against the exact property path the phase applies to,
    // so the assembler's read of that path is what consults it.
    auto phaseFor = [&moverPrim](const char *rel, const char *legacyAttr) {
        RigExecReadPhase phase;
        if (const UsdRelationship r = moverPrim.GetRelationship(TfToken(rel))) {
            RigExecResolveReadPhase(r, legacyAttr, &phase, nullptr);
        } else if (legacyAttr) {
            // No relationship to hang metadata on, but the legacy attribute
            // may still be authored.
            if (const UsdAttribute a =
                    moverPrim.GetAttribute(TfToken(legacyAttr))) {
                RigExecResolveReadPhase(a, legacyAttr, &phase, nullptr);
            }
        }
        return phase;
    };

    if (schemaType == "RigExecMatrixMover") {
        // "final" binds the provider's frame-chain head instead of the
        // provider itself; every other phase binds the provider (spec §12.1).
        const SdfPathVector transforms = _Targets(moverPrim, "rigExec:transform");
        SdfPath provider = transforms.empty() ? SdfPath() : transforms[0];
        binding.transformPhase =
            phaseFor("rigExec:transform", "rigExec:transformReadPhase");
        if (binding.transformPhase.kind == RigExecReadPhaseKind::Final) {
            const auto it = frameChainHeads.find(provider);
            if (it != frameChainHeads.end()) {
                provider = it->second;
            }
        }
        binding.transform = provider;
    } else if (schemaType == "RigExecSkinMover") {
        // Every influence shares one declared phase, on rigExec:influences
        // or the legacy attribute, and "final" binds each provider's
        // frame-chain head exactly as the matrix mover does for its one.
        binding.influences = _Targets(moverPrim, "rigExec:influences");
        binding.transformPhase =
            phaseFor("rigExec:influences", "rigExec:transformReadPhase");
        if (binding.transformPhase.kind == RigExecReadPhaseKind::Final) {
            for (SdfPath &provider : binding.influences) {
                const auto it = frameChainHeads.find(provider);
                if (it != frameChainHeads.end()) {
                    provider = it->second;
                }
            }
        }
    } else if (schemaType == "RigExecBlendShapeMover") {
        if (moverPrim.GetStage()->GetPrimAtPath(ownerPath).IsA<UsdGeomMesh>()) {
            binding.topologyCounts = ownerPath.AppendProperty(TfToken("faceVertexCounts"));
            binding.topologyIndices = ownerPath.AppendProperty(TfToken("faceVertexIndices"));
        }
        binding.blendInputs = _Targets(moverPrim, "rigExec:blendInputs");
        std::sort(binding.blendInputs.begin(), binding.blendInputs.end());
        for (const SdfPath &input : binding.blendInputs) {
            const UsdPrim channel = moverPrim.GetStage()->GetPrimAtPath(input);
            for (const SdfPath &samplePath : _Targets(channel, "rigExec:samples")) {
                const UsdPrim sample = moverPrim.GetStage()->GetPrimAtPath(samplePath);
                const SdfPathVector points = _Targets(sample, "rigExec:targetPoints");
                if (points.size() != 1) continue; // compile validates cardinality
                RigExecReadPhase phase;
                std::string error;
                RigExecResolveReadPhase(
                    sample.GetRelationship(TfToken("rigExec:targetPoints")),
                    "rigExec:pointsReadPhase", &phase, &error);
                binding.blendSamples[input].push_back({samplePath, _PointsOf(points[0]), phase});
            }
        }
        binding.base = target;
    } else if (schemaType == "RigExecVolumeCorrectMover") {
        binding.base = target;
    } else if (schemaType == "RigExecSmoothMover") {
        binding.topologyCounts =
            ownerPath.AppendProperty(TfToken("faceVertexCounts"));
        binding.topologyIndices =
            ownerPath.AppendProperty(TfToken("faceVertexIndices"));
    } else if (schemaType == "RigExecLatticeMover") {
        binding.base = target;
        const SdfPathVector cages = _Targets(moverPrim, "rigExec:cage");
        if (!cages.empty()) {
            binding.cagePoints = _PointsOf(cages[0]);
            const RigExecReadPhase phase =
                phaseFor("rigExec:cage", "rigExec:cageReadPhase");
            if (!phase.IsBase()) {
                binding.phases[binding.cagePoints] = phase;
            }
        }
    } else if (schemaType == "RigExecSurfaceMover") {
        const SdfPathVector surfaces = _Targets(moverPrim, "rigExec:surface");
        if (!surfaces.empty()) {
            const SdfPath surfacePrim = surfaces[0].GetPrimPath();
            binding.surfacePoints =
                surfacePrim.AppendProperty(TfToken("points"));
            const RigExecReadPhase phase =
                phaseFor("rigExec:surface", "rigExec:surfaceReadPhase");
            if (!phase.IsBase()) {
                binding.phases[binding.surfacePoints] = phase;
            }
            binding.topologyCounts =
                surfacePrim.AppendProperty(TfToken("faceVertexCounts"));
            binding.topologyIndices =
                surfacePrim.AppendProperty(TfToken("faceVertexIndices"));
        }
    } else if (schemaType == "RigExecCurvenetAdjusterMover") {
        binding.base = target;
        binding.curvenet = target.GetPrimPath();
    } else if (schemaType == "RigExecCurvenetMover") {
        // The Profile Mover reads the target's own topology to cut it, and
        // its authored base points are the projection pose the cut is
        // computed against (§4.1).
        binding.base = target;
        binding.topologyCounts =
            ownerPath.AppendProperty(TfToken("faceVertexCounts"));
        binding.topologyIndices =
            ownerPath.AppendProperty(TfToken("faceVertexIndices"));
        const SdfPathVector nets = _Targets(moverPrim, "rigExec:curvenet");
        if (!nets.empty()) {
            binding.curvenet = nets[0].GetPrimPath();
            binding.curvenetPoints =
                binding.curvenet.AppendProperty(TfToken("points"));
            const RigExecReadPhase phase =
                phaseFor("rigExec:curvenet", nullptr);
            if (!phase.IsBase()) {
                binding.phases[binding.curvenetPoints] = phase;
            }
        }
    } else if (schemaType == "RigExecCurveMover") {
        const SdfPathVector binds = _Targets(moverPrim, "rigExec:bindCoordinates");
        if (!binds.empty()) {
            binding.bindCoords = binds[0];
        }
        const SdfPathVector frames = _Targets(moverPrim, "rigExec:driverFrames");
        if (!frames.empty()) {
            binding.driverFrames = frames[0];
        }
        if (!binding.bindCoords.IsEmpty()) {
            const RigExecReadPhase phase =
                phaseFor("rigExec:bindCoordinates", nullptr);
            if (!phase.IsBase()) {
                binding.phases[binding.bindCoords] = phase;
            }
        }
    }

    // Every point-chain mover may narrow its application with a weight object.
    const SdfPathVector weights = _Targets(moverPrim, "rigExec:weightObject");
    if (!weights.empty()) {
        binding.weightObject = weights[0];
    }

    return binding;
}

RigExecMoverParameters
RigExecAssembleMatrixParameters(
    const UsdPrim &moverPrim,
    const GfMatrix4d *transform,
    const RigExecWeightPacket *weights,
    UsdTimeCode time,
    const RigExecResolvedInputs *resolved)
{
    RigExecMoverParameters params;
    params.kind = TfToken("matrix");

    bool enabled = true;
    if (moverPrim) {
        if (const UsdAttribute a =
                moverPrim.GetAttribute(TfToken("inputs:enabled"))) {
            if (!resolved ||
                !resolved->GetAttribute(a, time, &enabled)) {
                a.Get(&enabled, time);
            }
        }
    }
    params.enabled = enabled;
    if (!params.enabled) {
        params.valid = true;  // disabled is an ordinary pass-through
        return params;
    }

    if (!transform) {
        return params;  // MoverFailed
    }
    params.weights = weights
        ? *weights
        : RigExecWeightPacket::Constant(_Float(
              moverPrim, "inputs:defaultWeight", 1.0f, time, resolved));
    if (!params.weights.valid) {
        return params;  // invalid common envelope => MoverFailed
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

RigExecMoverStatus
RigExecStatusForParameters(
    const RigExecMoverParameters &parameters, const SdfPath &moverPath)
{
    RigExecMoverStatus status;
    if (!parameters.enabled) {
        status.state = TfToken("disabled");
    } else if (parameters.valid) {
        status.state = TfToken("ok");
    } else {
        status.state = TfToken("moverFailed");
        // First bad canonical public address (spec §6.6): v0.1 reports the
        // failed mover's own path; per-input attribution is future work.
        status.firstBadAddress = moverPath.GetString();
    }
    return status;
}

namespace {

// Reads a typed array from an exact property path on the mover's stage.
//
// \p resolved is null for BIND-TIME reads and only for those. A rest cage, a
// rest curvenet, a bind-time topology: those are the authored neutral pose the
// deformation is measured against, so a read phase has nothing to say about
// them -- and serving one the same phased value as the live read makes the two
// operands equal and the whole deformation an identity.
template <typename T>
std::vector<T>
_Array(const UsdPrim &moverPrim, const SdfPath &path, UsdTimeCode time,
       const RigExecResolvedInputs *resolved = nullptr)
{
    std::vector<T> out;
    if (path.IsEmpty() || !moverPrim) {
        return out;
    }
    VtArray<T> value;
    if (resolved && resolved->Get(path, &value)) {
        out.assign(value.begin(), value.end());
        return out;
    }
    if (const UsdAttribute a =
            moverPrim.GetStage()->GetAttributeAtPath(path)) {
        // At the EVALUATED time, not Default: the kernels read these same
        // inputs through exec computeValue at the current time, so an
        // animated cage/surface/topology would otherwise silently diverge.
        a.Get(&value, time);
    }
    out.assign(value.begin(), value.end());
    return out;
}

bool
_Enabled(const UsdPrim &prim, UsdTimeCode time,
         const RigExecResolvedInputs *resolved)
{
    bool enabled = true;
    if (prim) {
        if (const UsdAttribute a =
                prim.GetAttribute(TfToken("inputs:enabled"))) {
            if (resolved && resolved->GetAttribute(a, time, &enabled)) {
                return enabled;
            }
            a.Get(&enabled, time);
        }
    }
    return enabled;
}

}  // namespace

std::shared_ptr<const RigExecSkinTopology>
RigExecResolveSkinTopology(
    const UsdPrim &moverPrim,
    size_t influenceCount,
    UsdTimeCode time,
    const RigExecResolvedInputs *resolved,
    RigExecSkinTopologyCache *cache)
{
    if (!cache || !moverPrim) {
        return nullptr;
    }
    const SdfPath primPath = moverPrim.GetPath();
    const SdfPath indicesPath =
        primPath.AppendProperty(TfToken("rigExec:jointIndices"));
    const SdfPath weightsPath =
        primPath.AppendProperty(TfToken("rigExec:jointWeights"));
    // The layout is epoch state, so everything about it that does not involve
    // the influence MATRICES is settled once and shared: the two array reads,
    // the copy into the packet, and the per-element range and weight checks.
    // What is left per frame is the matrix table -- which is the only part of
    // the layout an animated rig changes.
    return cache->Resolve(
        primPath,
        [&](RigExecSkinTopology *topology) {
            // Whether the layout is epoch state is re-asked HERE, where the
            // cache is filled, and not only where the graph was compiled.
            // Authoring a time sample on jointWeights (or connecting it)
            // moves no epoch digest, so it does not recompile; it does send a
            // notice, and a notice clears this cache -- so this is the one
            // place that sees the stage as it is now. Refusing puts the
            // packet back on the per-frame arrays, which is what Compile
            // would have done had the sample been there. Costs one answer per
            // notice.
            for (const char *name : {"rigExec:jointIndices",
                                     "rigExec:jointWeights",
                                     "rigExec:elementSize"}) {
                const UsdAttribute a = moverPrim.GetAttribute(TfToken(name));
                if (a && (a.ValueMightBeTimeVarying() ||
                          a.HasAuthoredConnections())) {
                    return false;
                }
            }
            topology->indices =
                _Array<int>(moverPrim, indicesPath, time, resolved);
            topology->weights =
                _Array<float>(moverPrim, weightsPath, time, resolved);
            topology->elementSize = 1;
            if (const UsdAttribute a =
                    moverPrim.GetAttribute(TfToken("rigExec:elementSize"))) {
                if (!resolved ||
                    !resolved->GetAttribute(a, time, &topology->elementSize)) {
                    a.Get(&topology->elementSize, time);
                }
            }
            topology->influenceCount = influenceCount;
            if (topology->elementSize < 1 ||
                topology->weights.size() != topology->indices.size() ||
                topology->indices.size() %
                        size_t(topology->elementSize) != 0) {
                return true;
            }
            topology->pointCount =
                topology->indices.size() / size_t(topology->elementSize);
            // Exactly RigExecSkinLayout::Validate's shape, range and weight
            // rules, against a table whose SIZE is epoch state. The matrices
            // themselves are checked every frame by the caller.
            if (topology->influenceCount == 0) {
                return true;
            }
            for (size_t i = 0; i < topology->indices.size(); ++i) {
                const int index = topology->indices[i];
                if (index < 0 ||
                    size_t(index) >= topology->influenceCount) {
                    return true;
                }
                const float weight = topology->weights[i];
                if (!std::isfinite(weight) || weight < 0.0f) {
                    return true;
                }
            }
            topology->validated = true;
            return true;
        });
}

RigExecMoverParameters
RigExecAssembleSkinParameters(
    const UsdPrim &moverPrim,
    const std::vector<GfMatrix4d> *influenceTransforms,
    const RigExecWeightPacket *weights,
    UsdTimeCode time,
    const RigExecResolvedInputs *resolved,
    RigExecSkinTopologyCache *topologyCache,
    const std::shared_ptr<const RigExecSkinTopology> *resolvedTopology)
{
    RigExecMoverParameters params;
    params.kind = TfToken("skin");
    params.enabled = _Enabled(moverPrim, time, resolved);
    if (!params.enabled) {
        params.valid = true;  // disabled is an ordinary pass-through
        return params;
    }
    if (!moverPrim || !influenceTransforms) {
        return params;  // MoverFailed
    }
    params.weights = weights
        ? *weights
        : RigExecWeightPacket::Constant(_Float(
              moverPrim, "inputs:defaultWeight", 1.0f, time, resolved));
    if (!params.weights.valid) {
        return params;  // invalid common envelope => MoverFailed
    }

    params.skinTransforms = *influenceTransforms;
    const SdfPath primPath = moverPrim.GetPath();
    const SdfPath indicesPath =
        primPath.AppendProperty(TfToken("rigExec:jointIndices"));
    const SdfPath weightsPath =
        primPath.AppendProperty(TfToken("rigExec:jointWeights"));
    const auto readElementSize = [&moverPrim, time, resolved](int *out) {
        *out = 1;
        if (const UsdAttribute a =
                moverPrim.GetAttribute(TfToken("rigExec:elementSize"))) {
            if (!resolved || !resolved->GetAttribute(a, time, out)) {
                a.Get(out, time);
            }
        }
    };
    params.skinningMethod = TfToken("classicLinear");
    if (const UsdAttribute a =
            moverPrim.GetAttribute(TfToken("rigExec:skinningMethod"))) {
        if (!resolved ||
            !resolved->GetAttribute(a, time, &params.skinningMethod)) {
            a.Get(&params.skinningMethod, time);
        }
    }

    if (resolvedTopology) {
        // Already answered by a caller that may not take the cache's lock
        // where it assembles.
        params.skinTopology = *resolvedTopology;
    } else if (topologyCache) {
        params.skinTopology = RigExecResolveSkinTopology(
            moverPrim, influenceTransforms->size(), time, resolved,
            topologyCache);
    }
    // Null means the cache refused this mover -- the layout can move within
    // the epoch after all -- so the packet falls through to the per-frame
    // arrays below.
    if (params.skinTopology) {
        params.skinElementSize = params.skinTopology->elementSize;
        if (!params.skinTopology->validated ||
            params.skinTransforms.size() !=
                params.skinTopology->influenceCount) {
            return params;
        }
        // The frame half of RigExecSkinLayout::Validate.
        for (const GfMatrix4d &m : params.skinTransforms) {
            for (int r = 0; r < 4; ++r) {
                for (int c = 0; c < 4; ++c) {
                    if (!std::isfinite(m[r][c])) {
                        return params;
                    }
                }
            }
            if (m[0][3] != 0 || m[1][3] != 0 || m[2][3] != 0 ||
                m[3][3] != 1) {
                return params;
            }
        }
        if (params.skinningMethod != "classicLinear" &&
            params.skinningMethod != "dualQuaternion") {
            return params;
        }
        params.valid = true;
        return params;
    }

    params.skinIndices = _Array<int>(moverPrim, indicesPath, time, resolved);
    params.skinWeights = _Array<float>(moverPrim, weightsPath, time, resolved);
    readElementSize(&params.skinElementSize);

    // The point count is not known here, so the layout is checked against
    // its own length; the kernel re-checks against the points it receives.
    // Validated now rather than only in the kernel so a bad packet reports
    // MoverFailed through the status rather than by a silent pass-through.
    if (params.skinElementSize < 1 ||
        params.skinWeights.size() != params.skinIndices.size() ||
        params.skinIndices.size() % size_t(params.skinElementSize) != 0) {
        return params;
    }
    RigExecSkinLayout layout;
    layout.transforms = params.skinTransforms.data();
    layout.transformCount = params.skinTransforms.size();
    layout.indices = params.skinIndices.data();
    layout.weights = params.skinWeights.data();
    layout.indexCount = params.skinIndices.size();
    layout.elementSize = size_t(params.skinElementSize);
    layout.pointCount = layout.indexCount / layout.elementSize;
    if (!layout.Validate()) {
        return params;
    }
    // Both declared tokens assemble; the kernel is what has (or lacks) a
    // branch for them, and the compiler is what tells the author.
    if (params.skinningMethod != "classicLinear" &&
        params.skinningMethod != "dualQuaternion") {
        return params;
    }
    params.valid = true;
    return params;
}

bool
RigExecSumBlendChannels(
    const std::vector<RigExecBlendChannel> &channels,
    const std::vector<GfVec3f> &base,
    std::vector<GfVec3f> *deltas)
{
    if (!deltas || base.empty()) {
        return false;
    }
    deltas->assign(base.size(), GfVec3f(0));

    for (const RigExecBlendChannel &channel : channels) {
        if (channel.samples.empty() || !std::isfinite(channel.weight)) {
            return false;  // structural error: fails atomically
        }
        for (size_t k = 0; k < channel.samples.size(); ++k) {
            const RigExecBlendSampleData &sample = channel.samples[k];
            if (!std::isfinite(sample.activation) ||
                sample.activation <= 0 ||
                sample.points.size() != base.size() ||
                (k > 0 && sample.activation ==
                              channel.samples[k - 1].activation)) {
                return false;
            }
        }
        // The channel weight is clamped into the authored activation range;
        // an implicit zero-delta sample sits at activation 0.
        const float w = std::min(std::max(channel.weight, 0.0f),
                                 channel.samples.back().activation);
        if (w == 0.0f) {
            continue;
        }
        size_t hi = 0;
        while (hi < channel.samples.size() &&
               channel.samples[hi].activation < w) {
            ++hi;
        }
        if (hi >= channel.samples.size()) {
            hi = channel.samples.size() - 1;
        }
        const float aHi = channel.samples[hi].activation;
        const float aLo = hi > 0 ? channel.samples[hi - 1].activation : 0.0f;
        const float t = aHi > aLo ? (w - aLo) / (aHi - aLo) : 1.0f;
        const std::vector<GfVec3f> *lo =
            hi > 0 ? &channel.samples[hi - 1].points : nullptr;
        const std::vector<GfVec3f> &hiPts = channel.samples[hi].points;
        for (size_t i = 0; i < base.size(); ++i) {
            const GfVec3f dHi = hiPts[i] - base[i];
            const GfVec3f dLo = lo ? (*lo)[i] - base[i] : GfVec3f(0);
            (*deltas)[i] += dLo + (dHi - dLo) * t;
        }
    }
    return true;
}

RigExecMoverParameters
RigExecAssembleParameters(
    const UsdPrim &moverPrim,
    RigExecRevisionOp op,
    const RigExecRevisionBinding &binding,
    const RigExecProviderValues &values,
    UsdTimeCode time)
{
    if (op == RigExecRevisionOp::Matrix) {
        return RigExecAssembleMatrixParameters(
            moverPrim, values.transform, values.weights, time,
            values.resolved);
    }
    if (op == RigExecRevisionOp::Skin) {
        return RigExecAssembleSkinParameters(
            moverPrim, values.influenceTransforms, values.weights, time,
            values.resolved, values.skinTopologyCache, values.skinTopology);
    }
    if (op == RigExecRevisionOp::CurvenetAdjuster) {
        return RigExecAssembleCurvenetAdjusterParameters(
            moverPrim, binding.target, values.weights, time, values.resolved);
    }

    RigExecMoverParameters params;
    const bool synthesizedDerived =
        op == RigExecRevisionOp::RecomputeNormals ||
        op == RigExecRevisionOp::RecomputeExtent;
    // Derived maintenance has no authored mover. The owning gprim is only a
    // convenient topology/property source and must not accidentally acquire
    // mover semantics from custom attributes with familiar names.
    params.enabled = synthesizedDerived
        ? true
        : _Enabled(moverPrim, time, values.resolved);

    switch (op) {
    case RigExecRevisionOp::BlendShape:
        params.kind = TfToken("blendShape");
        break;
    case RigExecRevisionOp::VolumeCorrect:
        params.kind = TfToken("volumeCorrect");
        break;
    case RigExecRevisionOp::Smooth:
        params.kind = TfToken("smooth");
        break;
    case RigExecRevisionOp::Lattice:
        params.kind = TfToken("lattice");
        break;
    case RigExecRevisionOp::SurfaceProject:
        params.kind = TfToken("surfaceProject");
        break;
    case RigExecRevisionOp::Ribbon:
        params.kind = TfToken("ribbon");
        break;
    case RigExecRevisionOp::EmitGuidePoints:
        params.kind = TfToken("emitGuidePoints");
        break;
    case RigExecRevisionOp::Curvenet:
        params.kind = TfToken("curvenet");
        break;
    case RigExecRevisionOp::RecomputeNormals:
        params.kind = TfToken("recomputeNormals");
        break;
    case RigExecRevisionOp::RecomputeExtent:
        params.kind = TfToken("recomputeExtent");
        break;
    case RigExecRevisionOp::Matrix:
        break;  // handled above
    case RigExecRevisionOp::CurvenetAdjuster:
        break;  // handled above
    }

    if (!params.enabled) {
        params.valid = true;  // disabled is an ordinary pass-through
        return params;
    }

    // One envelope contract for every operation. A bound object is the total
    // field and supersedes the scalar fallback; without one, synthesize the
    // normalized constant packet supplied by RigExecMoverAPI.
    params.weights = synthesizedDerived
        ? RigExecWeightPacket::Constant(1.0f)
        : (values.weights
               ? *values.weights
               : RigExecWeightPacket::Constant(_Float(
                     moverPrim, "inputs:defaultWeight", 1.0f, time,
                     values.resolved)));
    if (!params.weights.valid) {
        return params;  // MoverFailed, preserving the preceding revision
    }

    switch (op) {
    case RigExecRevisionOp::BlendShape: {
        params.blendDeltas = values.blendDeltas;
        const TfToken space = _Token(moverPrim, "rigExec:deltaSpace", "target");
        if (space != "target" && space != "surfaceFrame") break;
        params.blendSurfaceFrame = space == "surfaceFrame";
        if (params.blendSurfaceFrame) {
            params.restPoints = values.basePoints;
            params.topologyCounts = _Array<int>(moverPrim, binding.topologyCounts, time, values.resolved);
            params.topologyIndices = _Array<int>(moverPrim, binding.topologyIndices, time, values.resolved);
            if (params.topologyCounts.empty()) break;
        }
        params.valid = !params.blendDeltas.empty();
        break;
    }

    case RigExecRevisionOp::VolumeCorrect:
        params.strength = 1.0f;
        // The correction reference is the bound volume of the authored base.
        if (!values.basePoints.empty()) {
            params.referenceVolume = RigExecBoundVolume(
                values.basePoints.data(), values.basePoints.size());
            params.valid = true;
        }
        break;

    case RigExecRevisionOp::Smooth:
        params.strength = 1.0f;
        params.topologyCounts = _Array<int>(moverPrim, binding.topologyCounts, time, values.resolved);
        params.topologyIndices =
            _Array<int>(moverPrim, binding.topologyIndices, time, values.resolved);
        params.valid = !params.topologyCounts.empty();
        break;

    case RigExecRevisionOp::Curvenet: {
        params.strength = 1.0f;
        const UsdPrim netPrim =
            binding.curvenet.IsEmpty()
                ? UsdPrim()
                : moverPrim.GetStage()->GetPrimAtPath(binding.curvenet);
        if (!netPrim) {
            break;  // no curvenet: MoverFailed pass-through
        }

        // The projection pose is the curvenet and the surface as AUTHORED --
        // Default time on both. Everything the cut depends on is read here,
        // and its digest is what decides whether the cache still applies.
        const std::vector<GfVec3f> restNet = _Array<GfVec3f>(moverPrim, binding.curvenetPoints, UsdTimeCode::Default(), /*resolved=*/nullptr);
        std::vector<int> splineIndices;
        if (const UsdAttribute a =
                netPrim.GetAttribute(TfToken("rigExec:splineIndices"))) {
            VtIntArray value;
            a.Get(&value, UsdTimeCode::Default());
            splineIndices.assign(value.begin(), value.end());
        }
        int samplesPerSpline = 5;
        if (const UsdAttribute a =
                netPrim.GetAttribute(TfToken("rigExec:samplesPerSpline"))) {
            a.Get(&samplesPerSpline);
        }
        const TfToken basisToken = _Token(netPrim, "rigExec:basis", "bezier");
        params.topologyCounts =
            _Array<int>(moverPrim, binding.topologyCounts, UsdTimeCode::Default(), /*resolved=*/nullptr);
        params.topologyIndices =
            _Array<int>(moverPrim, binding.topologyIndices, UsdTimeCode::Default(), /*resolved=*/nullptr);
        // The projection surface is the target's points at DEFAULT, not at
        // the evaluated time. It is a fixed neutral pose by definition (§4.1
        // cuts against it once), and reading the animated value instead would
        // put a per-frame quantity in the bind digest -- re-cutting the mesh
        // and re-factorizing its Laplacian on every frame, while also making
        // the cut mean something different at each one.
        params.restPoints = _Array<GfVec3f>(moverPrim, binding.base, UsdTimeCode::Default(), /*resolved=*/nullptr);
        if (restNet.empty() || splineIndices.empty() ||
            params.topologyCounts.empty() || params.restPoints.empty()) {
            break;
        }

        // The posed net: the evaluator hands over the result of the
        // curvenet's own mover chain when it has one, so knots articulated by
        // ordinary movers arrive already posed. Otherwise the authored value
        // at this time, which is what a keyframed or sculpted net gives.
        params.auxPoints = values.curvenetPoints.empty()
                               ? _Array<GfVec3f>(moverPrim,
                                                 binding.curvenetPoints, time)
                               : values.curvenetPoints;
        if (params.auxPoints.size() != restNet.size()) {
            break;  // the pool changed shape under the bind
        }

        size_t digest = TfHash()(basisToken);
        digest = TfHash::Combine(digest, samplesPerSpline);
        for (int index : splineIndices) {
            digest = TfHash::Combine(digest, index);
        }
        for (const GfVec3f &p : restNet) {
            digest = TfHash::Combine(digest, p[0], p[1], p[2]);
        }
        for (const GfVec3f &p : params.restPoints) {
            digest = TfHash::Combine(digest, p[0], p[1], p[2]);
        }
        for (int c : params.topologyCounts) {
            digest = TfHash::Combine(digest, c);
        }
        for (int i : params.topologyIndices) {
            digest = TfHash::Combine(digest, i);
        }

        auto build = [&](RigExecProfileMoverBinding *out, std::string *why) {
            RigExecCurvenetTopology topology;
            const RigExecCurvenetBasis basis =
                (basisToken == "catmullRom")
                    ? RigExecCurvenetBasis::CatmullRom
                    : RigExecCurvenetBasis::Bezier;
            if (!RigExecBuildCurvenetTopology(splineIndices, restNet.size(),
                                              basis, restNet, nullptr,
                                              &topology, why)) {
                return false;
            }
            return RigExecBindProfileMover(
                topology, restNet, params.restPoints, params.topologyCounts,
                params.topologyIndices, samplesPerSpline, out, why);
        };

        std::string reason;
        if (values.curvenetCache) {
            params.curvenetBinding = values.curvenetCache->Resolve(
                binding.moverPath, binding.target, digest, build, &reason);
        } else {
            auto fresh = std::make_shared<RigExecProfileMoverBinding>();
            if (build(fresh.get(), &reason)) {
                params.curvenetBinding = fresh;
            }
        }
        params.valid = params.curvenetBinding != nullptr;
        break;
    }

    case RigExecRevisionOp::Lattice: {
        params.restPoints = values.basePoints;
        // Operand order matters and is not symmetric: the shared applier is
        // RigExecApplyLattice(points, restPoints, restCage, posedCage, divs),
        // so auxPoints is the BIND-TIME cage and auxPointsB the live one --
        // matching _BuildLatticeMoverParameters. Reversing them is invisible
        // at rest (the two cages are equal) and inverts the deformation as
        // soon as the cage moves.
        // The rest cage is the cage at Default time, read directly. It used to
        // be a compiler-authored capture in rigExec:restCagePoints, but that
        // capture was itself only `a.Get(&v, UsdTimeCode::Default())` on this
        // same attribute -- so reading it here is identical and needs nothing
        // authored. The live cage is the same attribute at the evaluated time.
        params.auxPoints = _Array<GfVec3f>(moverPrim, binding.cagePoints, UsdTimeCode::Default(), /*resolved=*/nullptr);
        params.auxPointsB = _Array<GfVec3f>(moverPrim, binding.cagePoints, time, values.resolved);
        if (const UsdAttribute a =
                moverPrim.GetAttribute(TfToken("rigExec:divisions"))) {
            a.Get(&params.divisions, time);
        }
        // Same cardinality contract as the kernel: a cage that does not match
        // the declared lattice resolution fails atomically instead of
        // indexing garbage.
        const size_t cageCount = size_t(params.divisions[0]) *
                                 size_t(params.divisions[1]) *
                                 size_t(params.divisions[2]);
        params.valid = params.divisions[0] >= 2 && params.divisions[1] >= 2 &&
                       params.divisions[2] >= 2 &&
                       params.auxPoints.size() == cageCount &&
                       params.auxPointsB.size() == cageCount &&
                       !params.restPoints.empty();
        break;
    }

    case RigExecRevisionOp::SurfaceProject:
        // Fixed at full, matching _BuildSurfaceMoverParameters: v0.1
        // attach/project maps fully. RigExecSurfaceMover declares no
        // inputs:strength, so reading one here silently applied a 0.5
        // default and projected half way.
        params.strength = 1.0f;
        params.auxPoints = _Array<GfVec3f>(moverPrim, binding.surfacePoints, time, values.resolved);
        params.topologyCounts = _Array<int>(moverPrim, binding.topologyCounts, time, values.resolved);
        params.topologyIndices =
            _Array<int>(moverPrim, binding.topologyIndices, time, values.resolved);
        params.valid =
            !params.auxPoints.empty() && !params.topologyCounts.empty();
        break;

    case RigExecRevisionOp::Ribbon:
    case RigExecRevisionOp::EmitGuidePoints: {
        const RigExecPointFrameArray *frames = values.driverFrames;
        if (!frames || frames->IsEmpty() ||
            frames->rests.size() != frames->GetSize()) {
            break;  // MoverFailed
        }
        params.frames = *frames;
        if (op == RigExecRevisionOp::Ribbon) {
            params.bindCoords =
                _Array<GfVec2f>(moverPrim, binding.bindCoords, time, values.resolved);
            params.valid = !params.bindCoords.empty();
        } else {
            params.valid = true;
        }
        break;
    }

    case RigExecRevisionOp::RecomputeNormals:
    case RigExecRevisionOp::RecomputeExtent:
        // Derived maintenance reads the final same-generation points, which
        // the caller supplies as the base value for this revision.
        params.auxPoints = values.basePoints;
        params.topologyCounts = _Array<int>(moverPrim, binding.topologyCounts, time, values.resolved);
        params.topologyIndices =
            _Array<int>(moverPrim, binding.topologyIndices, time, values.resolved);
        // Authored widths widen the extent bounds; omitting them silently
        // under-reports the bound of a curves/points gprim.
        if (op == RigExecRevisionOp::RecomputeExtent) {
            params.widths = _Array<float>(moverPrim, binding.widths, time, values.resolved);
        }
        // Vertex normals need the adjacency; without it the kernel reports
        // MoverFailed rather than emitting garbage normals. Extent needs only
        // the points (matches _BuildRecomputeNormals/ExtentParameters).
        params.valid =
            !params.auxPoints.empty() &&
            (op == RigExecRevisionOp::RecomputeExtent ||
             !params.topologyCounts.empty());
        break;

    case RigExecRevisionOp::Matrix:
    case RigExecRevisionOp::Skin:
    case RigExecRevisionOp::CurvenetAdjuster:
        break;
    }

    return params;
}

struct RigExecMoverGraph::_Runtime {
    struct _RevisionSources {
        VdfInputVector<RigExecMoverParameters> *parameters;
        VdfInputVector<RigExecMoverStatus> *status;
        VdfMaskedOutput previous;
    };

    VdfExecutor<VdfPullBasedExecutorEngine,
        VdfDataManagerVector<VdfDataManagerDeallocationMode::Immediate>> executor;
    std::unique_ptr<VdfSchedule> schedule;
    std::map<VdfMaskedOutput, VdfInputVector<GfVec3f> *> sources;
    std::map<VdfMaskedOutput, _RevisionSources> revisions;
    VdfMaskedOutputVector outputs;
    VdfMaskedOutputVector dirty;
    size_t executionCount = 0;
    size_t scheduleBuildCount = 0;

    void TopologyChanged() {
        schedule.reset();
        executor.InvalidateTopologicalState();
    }
};

RigExecMoverGraph::RigExecMoverGraph() : _runtime(new _Runtime) {}
RigExecMoverGraph::~RigExecMoverGraph() = default;

VdfMaskedOutput
RigExecMoverGraph::AddPointSource(
    const SdfPath &target, const VtVec3fArray &points)
{
    const size_t count = points.size();
    VdfInputVector<GfVec3f> *const source =
        new VdfInputVector<GfVec3f>(&_network, count);
    for (size_t i = 0; i < count; ++i) {
        source->SetValue(i, points[i]);
    }
    TF_UNUSED(target);
    const VdfMaskedOutput output(
        source->GetOutput(), VdfMask::AllOnes(count ? count : 1));
    _runtime->sources.emplace(output, source);
    _runtime->outputs.push_back(output);
    _runtime->TopologyChanged();
    return output;
}

VdfMaskedOutput
RigExecMoverGraph::AddRevision(
    RigExecRevisionOp op,
    const VdfMaskedOutput &previous,
    const RigExecMoverParameters &parameters,
    const RigExecMoverStatus &status)
{
    // The packet and status are mutable graph inputs. The evaluator refreshes
    // them after scene edits without replacing any of the connected nodes.
    VdfInputVector<RigExecMoverParameters> *const paramSource =
        new VdfInputVector<RigExecMoverParameters>(&_network, 1);
    paramSource->SetValue(0, parameters);

    VdfInputVector<RigExecMoverStatus> *const statusSource =
        new VdfInputVector<RigExecMoverStatus>(&_network, 1);
    statusSource->SetValue(0, status);

    _RevisionNode *const revision =
        new _RevisionNode(&_network, op, &_runtime->executionCount);

    const VdfMask one = VdfMask::AllOnes(1);
    _network.Connect(
        paramSource->GetOutput(), revision, _tokens->parameters, one);
    _network.Connect(
        statusSource->GetOutput(), revision, _tokens->status, one);
    _network.Connect(
        previous.GetOutput(), revision, _tokens->previous, previous.GetMask());

    ++_revisionCount;
    const VdfMaskedOutput output(revision->GetOutput(_tokens->out),
                                 previous.GetMask());
    _runtime->revisions.emplace(
        output, _Runtime::_RevisionSources{paramSource, statusSource, previous});
    _runtime->outputs.push_back(output);
    _runtime->TopologyChanged();
    return output;
}

bool
RigExecMoverGraph::UpdatePointSource(
    const VdfMaskedOutput &source, const VtVec3fArray &points)
{
    const auto it = _runtime->sources.find(source);
    if (it == _runtime->sources.end() ||
        it->second->GetSize() != points.size()) {
        return false;
    }
    bool changed = false;
    for (size_t i = 0; i < points.size(); ++i) {
        if (!it->second->IsValueEqual(i, points[i])) {
            it->second->SetValue(i, points[i]);
            changed = true;
        }
    }
    if (changed) {
        // Nonlocal kernels (smooth, surface, volume) require the full input
        // when any point changes. Do not propagate an element mask through
        // them until their individual dependency masks are implemented.
        _runtime->dirty.push_back(source);
    }
    return true;
}

bool
RigExecMoverGraph::UpdateRevision(
    const VdfMaskedOutput &revision,
    const RigExecMoverParameters &parameters,
    const RigExecMoverStatus &status)
{
    const auto it = _runtime->revisions.find(revision);
    if (it == _runtime->revisions.end()) {
        return false;
    }
    const auto &sources = it->second;
    const VdfMask one = VdfMask::AllOnes(1);
    if (!sources.parameters->IsValueEqual(0, parameters)) {
        sources.parameters->SetValue(0, parameters);
        _runtime->dirty.emplace_back(sources.parameters->GetOutput(), one);
    }
    if (!sources.status->IsValueEqual(0, status)) {
        sources.status->SetValue(0, status);
        _runtime->dirty.emplace_back(sources.status->GetOutput(), one);
    }
    return true;
}

bool
RigExecMoverGraph::ReconnectRevision(
    const VdfMaskedOutput &revision, const VdfMaskedOutput &previous)
{
    const auto it = _runtime->revisions.find(revision);
    if (it == _runtime->revisions.end() || revision == previous ||
        revision.GetMask() != previous.GetMask() ||
        (!_runtime->sources.count(previous) &&
         !_runtime->revisions.count(previous))) return false;
    if (it->second.previous == previous) return true;
    // Invalidate on the old topology before disconnecting; downstream cached
    // values must not survive a changed predecessor with an equal packet.
    _runtime->dirty.push_back(revision);
    _runtime->executor.InvalidateValues(_runtime->dirty);
    _runtime->dirty.clear();
    VdfNode *const node = &revision.GetOutput()->GetNode();
    VdfInput *const input = node->GetInput(_tokens->previous);
    while (input->GetNumConnections()) {
        _network.Disconnect(&input->GetNonConstConnection(0));
    }
    _network.Connect(previous, node, _tokens->previous);
    it->second.previous = previous;
    _runtime->dirty.push_back(revision);
    _runtime->TopologyChanged();
    return true;
}

bool
RigExecMoverGraph::RemoveRevision(const VdfMaskedOutput &revision)
{
    const auto it = _runtime->revisions.find(revision);
    if (it == _runtime->revisions.end()) return false;
    _runtime->dirty.push_back(revision);
    _runtime->executor.InvalidateValues(_runtime->dirty);
    _runtime->dirty.clear();
    VdfNode *const node = &revision.GetOutput()->GetNode();
    VdfNode *const parameters = it->second.parameters;
    VdfNode *const status = it->second.status;
    _runtime->TopologyChanged();
    for (VdfOutput *output : {revision.GetOutput(),
            it->second.parameters->GetOutput(), it->second.status->GetOutput()}) {
        _runtime->executor.ClearDataForOutput(
            output->GetId(), output->GetNode().GetId());
    }
    _runtime->outputs.erase(std::remove(_runtime->outputs.begin(),
        _runtime->outputs.end(), revision), _runtime->outputs.end());
    _runtime->revisions.erase(it);
    _network.DisconnectAndDelete(node);
    _network.DisconnectAndDelete(parameters);
    _network.DisconnectAndDelete(status);
    --_revisionCount;
    return true;
}

VtVec3fArray
RigExecMoverGraph::Evaluate(const VdfMaskedOutput &output) const
{
    if (_runtime->sources.find(output) == _runtime->sources.end() &&
        _runtime->revisions.find(output) == _runtime->revisions.end()) {
        return {};
    }
    if (!_runtime->schedule) {
        _runtime->schedule = std::make_unique<VdfSchedule>();
        // Requesting the checkpoints preserves them across READWRITE buffer
        // passing. The compute request below still pulls only this output's
        // dependencies, so unrelated branches remain untouched.
        VdfScheduler::Schedule(
            VdfRequest(_runtime->outputs), _runtime->schedule.get(), true);
        _runtime->executor.Resize(_network);
        ++_runtime->scheduleBuildCount;
    }
    if (!_runtime->dirty.empty()) {
        _runtime->executor.InvalidateValues(_runtime->dirty);
        _runtime->dirty.clear();
    }
    _runtime->executor.Run(*_runtime->schedule, VdfRequest(output));

    const VdfVector *const value = _runtime->executor.GetOutputValue(
        *output.GetOutput(), output.GetMask());
    if (!value) {
        return {};
    }
    VdfVector::ReadAccessor<GfVec3f> values =
        value->GetReadAccessor<GfVec3f>();

    VtVec3fArray result(values.GetNumValues());
    for (size_t i = 0; i < values.GetNumValues(); ++i) {
        result[i] = values[i];
    }
    return result;
}

size_t
RigExecMoverGraph::GetRevisionExecutionCount() const
{
    return _runtime->executionCount;
}

std::vector<GfMatrix4d>
RigExecMoverGraph::GetRevisionControlFrames(const VdfMaskedOutput &revision) const
{
    const auto it = _runtime->revisions.find(revision);
    return it == _runtime->revisions.end() ? std::vector<GfMatrix4d>()
        : static_cast<const _RevisionNode &>(revision.GetOutput()->GetNode()).GetControlFrames();
}

RigExecMoverStatus
RigExecMoverGraph::GetRevisionStatus(const VdfMaskedOutput &revision) const
{
    if (!_runtime->revisions.count(revision)) return {};
    return static_cast<const _RevisionNode &>(revision.GetOutput()->GetNode()).GetStatus();
}

size_t
RigExecMoverGraph::GetScheduleBuildCount() const
{
    return _runtime->scheduleBuildCount;
}

}  // namespace rigExec
