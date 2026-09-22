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
#include <atomic>
#include <cmath>
#include <iterator>
#include <memory>
#include <mutex>
#include <unordered_map>
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
    ((wire, "wire"))
    ((emitGuidePoints, "emitGuidePoints"))
    ((curvenet, "curvenet"))
    ((curvenetAdjuster, "curvenetAdjuster"))
    ((recomputeNormals, "recomputeNormals"))
    ((recomputeExtent, "recomputeExtent"))
);

// The attribute and value names the PER-FRAME assemblers read. Hoisted out of
// their bodies because TfToken(const char *) takes the token registry's spin
// lock on every construction -- on the hit path as much as on the miss path --
// and an assembler runs once per revision per frame, inside a baked step that
// is not allowed to take a lock at all (docs/specs/baked-step-graph.md §2). The
// bind-time readers below keep their inline tokens: they run once per
// generation, off any step.
TF_DEFINE_PRIVATE_TOKENS(
    _attrTokens,
    ((enabled, "inputs:enabled"))
    ((defaultWeight, "inputs:defaultWeight"))
    ((jointIndices, "rigExec:jointIndices"))
    ((jointWeights, "rigExec:jointWeights"))
    ((elementSize, "rigExec:elementSize"))
    ((skinningMethod, "rigExec:skinningMethod"))
    ((deltaSpace, "rigExec:deltaSpace"))
    ((basis, "rigExec:basis"))
    ((splineIndices, "rigExec:splineIndices"))
    ((samplesPerSpline, "rigExec:samplesPerSpline"))
    ((divisions, "rigExec:divisions"))
);

// The values those reads fall back to, and the three states a revision's
// status reports. Same reason.
TF_DEFINE_PRIVATE_TOKENS(
    _valueTokens,
    ((classicLinear, "classicLinear"))
    ((target, "target"))
    ((bezier, "bezier"))
    ((disabled, "disabled"))
    ((ok, "ok"))
    ((moverFailed, "moverFailed"))
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
    case RigExecRevisionOp::Wire:
        return _kindTokens->wire;
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
            resultStatus->state = _valueTokens->moverFailed;
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
    _resultStatus =
        status ? *status
               : RigExecMoverStatus{_valueTokens->moverFailed, {}};
    // Node state that outlives one Compute, so a revision that passes through
    // must not leave the frames of the last one that did not.
    _controlFrames.clear();
    _RunRevisionOp(ctx, _op, &_resultStatus, &_controlFrames);
}

}  // namespace

// Public alias of the table above: the frozen assembler stamps the same
// kind tokens onto worker-assembled packets.
const TfToken &
RigExecRevisionKindToken(RigExecRevisionOp op)
{
    return _RevisionKindToken(op);
}

// The matrix kernel, shared by the mover-graph revision node and by the
// baked program. Unlike the point3f[] ops the envelope is NOT a separate
// blend here: the weighted-matrix rule folds it into the movement itself
// (p' = q + w (T q - q)), so resolving it is part of the kernel.
//
// One definition, so a second caller cannot drift into a different movement
// -- including over the SIMD choice, which must be the same on both paths or
// the two disagree in the last bits.
void
RigExecApplyMatrixKernelRange(const RigExecMoverParameters &p,
                              const float *envelope,
                              size_t begin, size_t end, GfVec3f *pts)
{
    static const bool useSimd = TfGetenvBool("RIGEXEC_ENABLE_SIMD", true);
    if (useSimd) {
        RigExecApplyWeightedMatrixSimd(
            pts + begin, pts + begin, envelope + begin, end - begin,
            p.transform);
    } else {
        // Element i is written only after it is read, so in-place is safe.
        for (size_t i = begin; i < end; ++i) {
            pts[i] = GfVec3f(RigExecApplyWeightedMatrix(
                GfVec3d(pts[i]), p.transform, envelope[i]));
        }
    }
}

namespace {

// The wire basis cache. A basis depends on the bind table, the weighted
// point set, the knots, the order, the control point count and the dropoff
// -- never on where the control points are -- so it is built once and every
// later frame is a lookup. Keyed by a hash of those inputs' CONTENT, not by
// their addresses: an edited bind table can reuse a freed buffer's address,
// and a stale basis would be a silently wrong deformation. The full inputs
// are kept beside each entry and compared on a hit, so a hash collision
// rebuilds rather than answers. Shared by the dynamic graph and the baked
// program, which is what keeps the two paths bit-identical.
struct _WireBasisEntry {
    std::vector<GfVec2f> binds;
    std::vector<int> indices;
    std::vector<double> knots;
    int order = 0;
    size_t controlPoints = 0;
    size_t meshPoints = 0;
    double dropoff = 0.0;
    std::shared_ptr<const RigExecWireBasis> basis;
};

uint64_t
_HashBytes(uint64_t h, const void *data, size_t size)
{
    const unsigned char *bytes = static_cast<const unsigned char *>(data);
    for (size_t i = 0; i < size; ++i) {
        h = (h ^ bytes[i]) * 1099511628211ull;
    }
    return h;
}

std::shared_ptr<const RigExecWireBasis>
_CachedWireBasis(const RigExecMoverParameters &p,
                 const std::vector<int> &indices, size_t meshPoints)
{
    static std::mutex mutex;
    static std::unordered_map<uint64_t, _WireBasisEntry> cache;

    uint64_t h = 1469598103934665603ull;
    h = _HashBytes(h, p.wireBindCoords.cdata(),
                   p.wireBindCoords.size() * sizeof(GfVec2f));
    h = _HashBytes(h, indices.data(), indices.size() * sizeof(int));
    h = _HashBytes(h, p.curveKnots.data(), p.curveKnots.size() * sizeof(double));
    const size_t controlPoints = p.restPoints.size();
    h = _HashBytes(h, &p.curveOrder, sizeof(p.curveOrder));
    h = _HashBytes(h, &controlPoints, sizeof(controlPoints));
    h = _HashBytes(h, &meshPoints, sizeof(meshPoints));
    h = _HashBytes(h, &p.dropoffDistance, sizeof(p.dropoffDistance));

    const auto matches = [&](const _WireBasisEntry &e) {
        return e.order == p.curveOrder && e.controlPoints == controlPoints &&
               e.meshPoints == meshPoints && e.dropoff == p.dropoffDistance &&
               e.knots == p.curveKnots && e.indices == indices &&
               e.binds.size() == p.wireBindCoords.size() &&
               std::equal(e.binds.begin(), e.binds.end(),
                          p.wireBindCoords.cbegin());
    };
    {
        std::lock_guard<std::mutex> lock(mutex);
        const auto it = cache.find(h);
        if (it != cache.end() && matches(it->second)) {
            return it->second.basis;
        }
    }
    auto basis = std::make_shared<RigExecWireBasis>();
    if (!RigExecBuildWireBasis(p.wireBindCoords.cdata(),
                               p.wireBindCoords.size(), meshPoints, indices,
                               p.curveOrder, p.curveKnots, controlPoints,
                               p.dropoffDistance, basis.get())) {
        return nullptr;
    }
    _WireBasisEntry entry;
    entry.binds.assign(p.wireBindCoords.cbegin(), p.wireBindCoords.cend());
    entry.indices = indices;
    entry.knots = p.curveKnots;
    entry.order = p.curveOrder;
    entry.controlPoints = controlPoints;
    entry.meshPoints = meshPoints;
    entry.dropoff = p.dropoffDistance;
    entry.basis = basis;
    std::lock_guard<std::mutex> lock(mutex);
    if (cache.size() > 512) {
        cache.clear();  // edits accumulate entries nothing will ask for again
    }
    cache[h] = std::move(entry);
    return basis;
}

}  // namespace

bool
RigExecApplyMatrixKernel(const RigExecMoverParameters &p,
                         std::vector<GfVec3f> *pts)
{
    const size_t count = pts->size();
    // A sparse field with a zero default touches only its named points: a
    // face cluster weights a few hundred of a body's tens of thousands, so
    // resolving and walking the dense array is almost all waste. Validated
    // exactly as ResolveAll would, so the same packets fail.
    const RigExecWeightPacket &w = p.weights;
    if (w.valid && w.representation == "sparse" && w.defaultWeight == 0.0f &&
        w.indices.size() == w.values.size() &&
        (w.rangePolicy.IsEmpty() || w.rangePolicy == "strict" ||
         w.rangePolicy == "clamp")) {
        for (size_t k = 0; k < w.indices.size(); ++k) {
            const int index = w.indices[k];
            const float value = w.values[k];
            if (index < 0 || size_t(index) >= count ||
                (k > 0 && index <= w.indices[k - 1]) ||
                !std::isfinite(value) || value < 0.0f || value > 1.0f) {
                return false;
            }
        }
        if (p.transform == GfMatrix4d(1.0)) {
            return true;  // at rest every weighted point maps to itself
        }
        GfVec3f *data = pts->data();
        for (size_t k = 0; k < w.indices.size(); ++k) {
            GfVec3f &point = data[size_t(w.indices[k])];
            point = GfVec3f(RigExecApplyWeightedMatrix(
                GfVec3d(point), p.transform, w.values[k]));
        }
        return true;
    }
    std::vector<float> weights(count);
    if (!p.weights.ResolveAll(count, &weights)) {
        return false;  // cardinality mismatch fails atomically
    }
    RigExecApplyMatrixKernelRange(p, weights.data(), 0, count, pts->data());
    return true;
}

void
RigExecBlendEnvelopeRange(const GfVec3f *preceding, const float *envelope,
                          size_t begin, size_t end, GfVec3f *blended)
{
    for (size_t i = begin; i < end; ++i) {
        blended[i] =
            RigExecBlendEnvelope(preceding[i], blended[i], envelope[i]);
    }
}

void
RigExecBlendEnvelopeAll(const GfVec3f *preceding, const float *envelope,
                        size_t count, GfVec3f *blended)
{
    // Per point, reading two arrays and writing a third at the same index: a
    // point range is an independent sub-problem, so splitting it changes
    // nothing about the arithmetic -- only who performs it.
    if (RigExecParallelEvaluationEnabled() && !RigExecFrozenSerialActive() &&
        count >= RigExecGeometryParallelThreshold) {
        WorkParallelForN(
            count,
            [blended, preceding, envelope](size_t begin, size_t end) {
                RigExecBlendEnvelopeRange(preceding, envelope, begin, end,
                                          blended);
            },
            RigExecGeometryGrainSize);
        return;
    }
    RigExecBlendEnvelopeRange(preceding, envelope, 0, count, blended);
}

// The skin kernel, shared by the mover-graph revision node and by the
// baked program, which runs the same operation with no VdfNetwork around
// it. One definition, so a second caller cannot drift into a different
// deformation.
//
// It is now three pieces rather than one, because a chunked caller needs the
// per-vertex body without the decisions AROUND it -- and every one of those
// decisions is a statement about the WHOLE array, not about a vertex:
// the layout's element shape, index range and weights; the influence table's
// finite/affine check; the method token. So the validation splits in two by
// what it reads (RigExecSkinLayoutIsUsable, RigExecSkinTransformsAreUsable),
// the per-vertex body becomes RigExecApplySkinKernelRange, and this function
// is what it always was: validate, then run every vertex.

RigExecSkinTransformsView
RigExecSkinTransformsOf(const RigExecMoverParameters &p)
{
    RigExecSkinTransformsView view;
    view.transforms = p.skinTransforms.data();
    view.transformCount = p.skinTransforms.size();
    return view;
}

RigExecSkinLayout
RigExecSkinLayoutForPacket(const RigExecMoverParameters &p,
                           const RigExecSkinTransformsView &transforms,
                           size_t pointCount)
{
    // The incoming revision IS the rest pose the influences were bound
    // against. A blend shape upstream of the skin is skinned -- exactly as a
    // skinCluster skins its input geometry and UsdSkel applies blend shapes
    // before skinning -- and when the skin is first in its chain the incoming
    // points are the authored base. No separate rest input is needed for
    // that, and every skinning method reads the same gather below.
    RigExecSkinLayout layout;
    layout.transforms = transforms.transforms;
    layout.transformCount = transforms.transformCount;
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
    layout.pointCount = pointCount;
    return layout;
}

bool
RigExecSkinLayoutIsUsable(const RigExecMoverParameters &p, size_t pointCount)
{
    const size_t transformCount = p.skinTransforms.size();
    if (const RigExecSkinTopology *const topology = p.skinTopology.get()) {
        // O(1). The epoch checked the element shape, the index range and the
        // weights; the assembler checked THIS frame's matrices and would have
        // failed the packet otherwise, so the only question left is whether
        // the points that arrived are the points the layout describes --
        // which is the one thing the assembler could not know.
        const size_t elementSize =
            topology->elementSize < 1 ? 0 : size_t(topology->elementSize);
        return topology->validated &&
               topology->influenceCount == transformCount &&
               topology->indices.size() == pointCount * elementSize;
    }
    if (p.skinWeights.size() != p.skinIndices.size()) {
        return false;  // cardinality mismatch fails atomically
    }
    // RigExecSkinLayout::Validate, minus its influence-matrix loop, which is
    // RigExecSkinTransformsAreUsable below. The two together are the same
    // conjunction Validate() is, so splitting them changes no answer.
    const size_t elementSize =
        p.skinElementSize < 1 ? 0 : size_t(p.skinElementSize);
    if (elementSize < 1 || transformCount == 0) {
        return false;
    }
    if (p.skinIndices.size() != pointCount * elementSize) {
        return false;
    }
    for (size_t i = 0; i < p.skinIndices.size(); ++i) {
        if (p.skinIndices[i] < 0 ||
            size_t(p.skinIndices[i]) >= transformCount) {
            return false;
        }
        if (!std::isfinite(p.skinWeights[i]) || p.skinWeights[i] < 0.0f) {
            return false;
        }
    }
    return true;
}

bool
RigExecSkinTransformsAreUsable(const GfMatrix4d *transforms, size_t count)
{
    if (!transforms || count == 0) {
        return false;
    }
    for (size_t t = 0; t < count; ++t) {
        const GfMatrix4d &m = transforms[t];
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                if (!std::isfinite(m[r][c])) {
                    return false;
                }
            }
        }
        if (m[0][3] != 0 || m[1][3] != 0 || m[2][3] != 0 || m[3][3] != 1) {
            return false;
        }
    }
    return true;
}

bool
RigExecApplySkinKernelRange(const RigExecMoverParameters &p,
                            const RigExecSkinTransformsView &transforms,
                            size_t begin, size_t end,
                            std::vector<GfVec3f> *pts)
{
    const RigExecSkinLayout layout =
        RigExecSkinLayoutForPacket(p, transforms, pts->size());
    // A point range IS a layout: the indices and weights of point i live at
    // i * elementSize and nowhere else, and no point reads another's result.
    // Both kernels loop one point at a time, so a split boundary cannot land
    // inside a vectorised block either -- which is what makes a chunked
    // result bit-identical to the whole-array one rather than merely equal to
    // tolerance.
    RigExecSkinLayout part = layout;
    part.indices = layout.indices + begin * layout.elementSize;
    part.weights = layout.weights + begin * layout.elementSize;
    part.indexCount = (end - begin) * layout.elementSize;
    part.pointCount = end - begin;
    GfVec3f *const points = pts->data();
    static const bool useSimd = TfGetenvBool("RIGEXEC_ENABLE_SIMD", true);

    // ---- Method dispatch ---------------------------------------------
    // The one point where the skinning methods part. Everything above is
    // the shared per-point gather (indices, weights, influence matrices,
    // rest point); only the accumulation differs.
    if (p.skinningMethod == "classicLinear") {
        // sum_k w_k T_k p, with the weight complement held at the rest
        // point (see RigExecApplyLinearBlendSkin).
        if (useSimd) {
            RigExecApplyLinearBlendSkinSimd(
                points + begin, points + begin, part, transforms.rows);
        } else {
            RigExecApplyLinearBlendSkin(
                points + begin, points + begin, part);
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
        //
        // The split is per matrix, so a caller skinning several ranges
        // against one table hands the palette in and pays for it once.
        std::vector<RigExecScaledDualQuat> local;
        const RigExecScaledDualQuat *palette = transforms.palette;
        size_t paletteSize = transforms.paletteSize;
        if (!palette) {
            local = RigExecSkinDualQuatPalette(layout);
            palette = local.data();
            paletteSize = local.size();
        }
        return RigExecApplyDualQuatSkin(points + begin, points + begin, part,
                                        palette, paletteSize);
    }
    // A token neither kernel owns is a compile error upstream; a packet that
    // reaches here anyway fails the application rather than silently running
    // the wrong maths.
    return false;
}

bool
RigExecApplySkinKernelWithTransforms(
    const RigExecMoverParameters &p,
    const RigExecSkinTransformsView &transforms,
    std::vector<GfVec3f> *pts)
{
    const size_t count = pts->size();
    if (!RigExecSkinLayoutIsUsable(p, count)) {
        return false;
    }
    if (!p.skinTopology &&
        !RigExecSkinTransformsAreUsable(transforms.transforms,
                                        transforms.transformCount)) {
        return false;
    }

    // BOTH methods split, not just the linear one. A point range is an
    // independent sub-problem under either kernel -- the indices and weights
    // of point i live at i * elementSize and no point reads another's result
    // -- so which method is running says nothing about whether the work can
    // be divided. Only the linear path was split, which left every
    // dualQuaternion character skinning its whole mesh on one thread;
    // measured on a 26,276-point body, that was the single largest cost in
    // a drag.
    const bool splittable =
        RigExecParallelEvaluationEnabled() && !RigExecFrozenSerialActive() &&
        count >= RigExecGeometryParallelThreshold &&
        (p.skinningMethod == "classicLinear" ||
         p.skinningMethod == "dualQuaternion");
    if (!splittable) {
        return RigExecApplySkinKernelRange(p, transforms, 0, count, pts);
    }

    // THE PER-MATRIX TABLES ARE DERIVED ONCE HERE AND HANDED TO EVERY CHUNK.
    // Both kernels build their own when handed none -- the SIMD path narrows
    // the rows to float, the dual-quaternion path splits each matrix into a
    // stretch and a unit dual quaternion -- and both are pure functions of
    // the influence table, so a chunk that derives its own gets the same
    // table every other chunk derived. Identical, and paid for once per task
    // instead of once: with 137 influences and a grain of 512 points that is
    // fifty-odd redundant derivations of the same thing. The linear path has
    // been splitting without hoisting since it gained the split.
    RigExecSkinTransformsView shared = transforms;
    const RigExecSkinLayout layout =
        RigExecSkinLayoutForPacket(p, transforms, count);
    std::vector<float> rows;
    std::vector<RigExecScaledDualQuat> palette;
    if (p.skinningMethod == "classicLinear" && !shared.rows &&
        layout.transforms) {
        rows.resize(layout.transformCount * RigExecSkinRowStride);
        for (size_t t = 0; t < layout.transformCount; ++t) {
            RigExecNarrowSkinRows(layout.transforms[t],
                                  &rows[t * RigExecSkinRowStride]);
        }
        shared.rows = rows.data();
    } else if (p.skinningMethod == "dualQuaternion" && !shared.palette) {
        palette = RigExecSkinDualQuatPalette(layout);
        shared.palette = palette.data();
        shared.paletteSize = palette.size();
    }

    // A degenerate blend fails one range, and the answer for the operation is
    // that it failed. Relaxed ordering is enough: nothing is published
    // through this flag, and WorkParallelForN joins before it is read.
    std::atomic<bool> ok(true);
    WorkParallelForN(
        count,
        [&p, &shared, pts, &ok](size_t begin, size_t end) {
            if (!RigExecApplySkinKernelRange(p, shared, begin, end, pts)) {
                ok.store(false, std::memory_order_relaxed);
            }
        },
        RigExecGeometryGrainSize);
    return ok.load(std::memory_order_relaxed);
}

bool
RigExecApplySkinKernel(const RigExecMoverParameters &p,
                       std::vector<GfVec3f> *pts)
{
    return RigExecApplySkinKernelWithTransforms(
        p, RigExecSkinTransformsOf(p), pts);
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

    // Per point, reading two arrays and writing a third at the same index --
    // the same independent sub-problem RigExecBlendEnvelopeAll already
    // splits, and for the same reason: dividing the range changes nothing
    // about the arithmetic, only who performs it. Everything above stays on
    // this thread, because the envelope resolve and the surface transport
    // are statements about the WHOLE array and fail it atomically.
    //
    // WORTH KNOWING WHAT THIS DID NOT FIX. On a 26,276-point body with 161
    // corrective targets the blend-shape revision is the largest deformer
    // cost left in an interactive drag once the skin kernel learned to
    // split -- about 2 ms against the skin's 0.5 -- and splitting this loop
    // moved it 1.07x. So the loop is not where that time goes: it is in
    // assembling the channels (bakedGeometry.cpp, the per-sample
    // GetAttribute reads), which is an epoch-caching problem and not a
    // parallelism one. The split is kept because it is correct, free and
    // matches every other point kernel, not because it paid here.
    GfVec3f *const points = pts->data();
    const GfVec3f *const delta = deltas->data();
    const float *const weight = envelope.data();
    const auto blendRange = [points, delta, weight](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            const GfVec3f preceding = points[i];
            points[i] = RigExecBlendEnvelope(
                preceding, preceding + delta[i], weight[i]);
        }
    };
    if (RigExecParallelEvaluationEnabled() && !RigExecFrozenSerialActive() &&
        count >= RigExecGeometryParallelThreshold) {
        WorkParallelForN(count, blendRange, RigExecGeometryGrainSize);
        return true;
    }
    blendRange(0, count);
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
    case RigExecRevisionOp::Wire: {
        if (p.auxPoints.size() != p.restPoints.size()) {
            return false;
        }
        const RigExecNurbsCurve rest{&p.restPoints, p.curveOrder,
                                     &p.curveKnots};
        const RigExecNurbsCurve posed{&p.auxPoints, p.curveOrder,
                                      &p.curveKnots};
        if (!rest.IsValid() || !posed.IsValid()) {
            return false;
        }
        // A sparse zero-default envelope: evaluate only the named points.
        // RigExecRunRevisionKernel routes a wire here with its envelope
        // unapplied only when this packet shape is what it holds.
        if (RigExecWireTakesSparseEnvelope(p.weights)) {
            const RigExecWeightPacket &w = p.weights;
            for (size_t k = 0; k < w.indices.size(); ++k) {
                if (w.indices[k] < 0 || size_t(w.indices[k]) >= pts->size() ||
                    (k > 0 && w.indices[k] <= w.indices[k - 1]) ||
                    !std::isfinite(w.values[k]) || w.values[k] < 0.0f ||
                    w.values[k] > 1.0f) {
                    return false;
                }
            }
            const std::shared_ptr<const RigExecWireBasis> basis =
                _CachedWireBasis(p, w.indices, pts->size());
            if (!basis) {
                return false;
            }
            return RigExecApplyWireBasis(pts, *basis, w.indices, w.values,
                                         p.restPoints, p.auxPoints);
        }
        if (p.wireBindCoords.size() != pts->size()) {
            return false;  // a sparse bind table needs a sparse envelope
        }
        // Per-point and independent, so the range splits across threads;
        // a small mesh stays on this thread.
        bool ok = true;
        if (RigExecParallelEvaluationEnabled() && !RigExecFrozenSerialActive() &&
            pts->size() >= 4096) {
            std::atomic<bool> good(true);
            WorkParallelForN(pts->size(), [&](size_t b, size_t e) {
                if (!RigExecApplyWire(pts, rest, posed,
                                      p.wireBindCoords.cdata(),
                                      p.wireBindCoords.size(),
                                      p.dropoffDistance, b, e)) {
                    good = false;
                }
            });
            ok = good;
        } else {
            ok = RigExecApplyWire(pts, rest, posed, p.wireBindCoords.cdata(),
                                  p.wireBindCoords.size(),
                                  p.dropoffDistance, 0, pts->size());
        }
        return ok;
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
        op == RigExecRevisionOp::RecomputeExtent ||
        (op == RigExecRevisionOp::Wire &&
         RigExecWireTakesSparseEnvelope(p.weights))) {
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
        RigExecBlendEnvelopeAll(preceding.data(), envelope.data(),
                                pts->size(), pts->data());
    }
    return true;
}

bool
RigExecRevisionBinding::operator==(const RigExecRevisionBinding &o) const
{
    return moverPath == o.moverPath && target == o.target &&
           transform == o.transform &&
           transformSpace == o.transformSpace && influences == o.influences &&
           weightObject == o.weightObject &&
           base == o.base && topologyCounts == o.topologyCounts &&
           topologyIndices == o.topologyIndices &&
           cagePoints == o.cagePoints && surfacePoints == o.surfacePoints &&
           bindCoords == o.bindCoords && driverFrames == o.driverFrames &&
           driverCurvePoints == o.driverCurvePoints &&
           driverCurveOrder == o.driverCurveOrder &&
           driverCurveKnots == o.driverCurveKnots &&
           driverTransformCount == o.driverTransformCount &&
           driverSpaceCount == o.driverSpaceCount &&
           driverBaseTransformCount == o.driverBaseTransformCount &&
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
        if (curveMode == "emitGuidePoints") {
            return RigExecRevisionOp::EmitGuidePoints;
        }
        if (curveMode == "wire") {
            return RigExecRevisionOp::Wire;
        }
        return RigExecRevisionOp::Ribbon;
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
_Token(const UsdPrim &prim, const TfToken &attr, const TfToken &fallback)
{
    TfToken value = fallback;
    if (const UsdAttribute a = prim.GetAttribute(attr)) {
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
_Float(const UsdPrim &prim, const TfToken &attr, float fallback,
       UsdTimeCode time, const RigExecResolvedInputs *resolved)
{
    float value = fallback;
    if (const UsdAttribute a = prim.GetAttribute(attr)) {
        if (resolved && resolved->GetAttribute(a, time, &value)) {
            return value;
        }
        a.Get(&value, time);
    }
    return value;
}

// Forward declarations: the matrix assembler below predates the hook
// helpers and consumes the same enabled contract through them.
RigExecBakeReadRecorder *
_RecorderOf(const RigExecResolvedInputs *resolved);

bool
_Enabled(const UsdPrim &prim, UsdTimeCode time,
         const RigExecResolvedInputs *resolved,
         RigExecBakeReadRecorder *bakeRecorder);

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

std::shared_ptr<const RigExecBlendSampleLayout>
RigExecBlendSampleCache::Resolve(
    const SdfPath &sample,
    const std::function<bool(RigExecBlendSampleLayout *)> &build)
{
    // Held across the build as well as the lookup, exactly as the skin
    // topology cache does: concurrent chain tasks would otherwise insert
    // into the same map at once.
    std::lock_guard<std::mutex> lock(_mutex);
    const auto found = _entries.find(sample);
    if (found != _entries.end()) {
        // Including a remembered refusal, which is a null entry.
        return found->second;
    }
    auto built = std::make_shared<RigExecBlendSampleLayout>();
    if (!build(built.get())) {
        _entries.emplace(sample, nullptr);
        return nullptr;
    }
    const auto candidate = _candidates.find(sample);
    if (candidate != _candidates.end() && candidate->second &&
        *candidate->second == *built) {
        return _entries.emplace(sample, candidate->second).first->second;
    }
    return _entries.emplace(sample, std::move(built)).first->second;
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
        const SdfPathVector spaces =
            _Targets(moverPrim, "rigExec:transformSpace");
        SdfPath space = spaces.empty() ? SdfPath() : spaces[0];
        if (!space.IsEmpty() &&
            binding.transformPhase.kind == RigExecReadPhaseKind::Final) {
            const auto it = frameChainHeads.find(space);
            if (it != frameChainHeads.end()) {
                space = it->second;
            }
        }
        binding.transformSpace = space;
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
                const SdfPathVector shapes = _Targets(sample, "rigExec:blendShape");
                // Exactly one of the two. Both authored is not a precedence
                // question: two shapes that disagree with a silent winner is
                // the worst of the three outcomes, so the sample is dropped
                // here and compile reports it.
                if (points.size() + shapes.size() != 1) continue;
                RigExecReadPhase phase;
                std::string error;
                if (shapes.size() == 1) {
                    // A sparse sample has no phased points property to read:
                    // its offsets are authored data on a UsdSkelBlendShape,
                    // not a chain result, so there is no "preceding" or
                    // "final" revision of them to select.
                    binding.blendSamples[input].push_back(
                        {samplePath, SdfPath(), phase, shapes[0]});
                    continue;
                }
                RigExecResolveReadPhase(
                    sample.GetRelationship(TfToken("rigExec:targetPoints")),
                    "rigExec:pointsReadPhase", &phase, &error);
                binding.blendSamples[input].push_back(
                    {samplePath, _PointsOf(points[0]), phase, SdfPath()});
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
        // The wire's driver: a NURBS curve prim, read at its declared phase
        // (a curve deformed by its own chain reads "final").
        const SdfPathVector curves = _Targets(moverPrim, "rigExec:driverCurve");
        if (!curves.empty()) {
            const SdfPath curvePrim = curves[0].GetPrimPath();
            binding.driverCurvePoints =
                curvePrim.AppendProperty(TfToken("points"));
            binding.driverCurveOrder =
                curvePrim.AppendProperty(TfToken("order"));
            binding.driverCurveKnots =
                curvePrim.AppendProperty(TfToken("knots"));
            const RigExecReadPhase phase = phaseFor(
                "rigExec:driverCurve", "rigExec:driverCurveReadPhase");
            if (!phase.IsBase()) {
                binding.phases[binding.driverCurvePoints] = phase;
            }
        }
        // Or the wire's control points moved by matrix providers directly:
        // one transform (or one for all) per unique control point, measured
        // against its space. Carried as influences -- transforms first, then
        // spaces -- so every path delivers them the way it delivers a skin's.
        const SdfPathVector driverTransforms =
            _Targets(moverPrim, "rigExec:driverTransforms");
        if (!driverTransforms.empty()) {
            binding.transformPhase = phaseFor("rigExec:driverTransforms",
                                              "rigExec:transformReadPhase");
            const auto provider = [&](SdfPath path) {
                if (binding.transformPhase.kind ==
                    RigExecReadPhaseKind::Final) {
                    const auto it = frameChainHeads.find(path);
                    if (it != frameChainHeads.end()) {
                        path = it->second;
                    }
                }
                return path;
            };
            for (const SdfPath &t : driverTransforms) {
                binding.influences.push_back(provider(t));
            }
            const SdfPathVector spaces =
                _Targets(moverPrim, "rigExec:driverTransformSpaces");
            for (const SdfPath &s : spaces) {
                binding.influences.push_back(provider(s));
            }
            const SdfPathVector baseTransforms =
                _Targets(moverPrim, "rigExec:driverBaseTransforms");
            for (const SdfPath &b : baseTransforms) {
                binding.influences.push_back(provider(b));
            }
            for (const SdfPath &b :
                     _Targets(moverPrim, "rigExec:driverBaseTransformSpaces")) {
                binding.influences.push_back(provider(b));
            }
            binding.driverTransformCount = int(driverTransforms.size());
            binding.driverSpaceCount = int(spaces.size());
            binding.driverBaseTransformCount = int(baseTransforms.size());
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
    params.kind = _kindTokens->matrix;

    params.enabled =
        _Enabled(moverPrim, time, resolved, _RecorderOf(resolved));
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
              moverPrim, _attrTokens->defaultWeight, 1.0f, time, resolved));
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
        status.state = _valueTokens->disabled;
    } else if (parameters.valid) {
        status.state = _valueTokens->ok;
    } else {
        status.state = _valueTokens->moverFailed;
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
RigExecBakeReadRecorder *
_RecorderOf(const RigExecResolvedInputs *resolved)
{
    return resolved ? resolved->bakeRecorder : nullptr;
}

template <typename T>
std::vector<T>
_Array(const UsdPrim &moverPrim, const SdfPath &path, UsdTimeCode time,
       const RigExecResolvedInputs *resolved = nullptr,
       RigExecBakeReadRecorder *bakeRecorder = nullptr)
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
        // The overlay arm above returns unrecorded: overlay values are
        // recomputed, not replayed. The stage value records under
        // (path, was-Default) so a rest/live pair keeps both.
        if (bakeRecorder) {
            bakeRecorder->RecordPath(path, time == UsdTimeCode::Default(),
                                     VtValue(value),
                                     /*forceFrame=*/false);
        }
    } else if (bakeRecorder) {
        // A dangling binding path reads as the empty array; the runtime
        // tells that apart from a gap by the absent mark.
        bakeRecorder->RecordPath(path, time == UsdTimeCode::Default(),
                                 VtValue(), /*forceFrame=*/false);
    }
    out.assign(value.begin(), value.end());
    return out;
}

bool
_Enabled(const UsdPrim &prim, UsdTimeCode time,
         const RigExecResolvedInputs *resolved,
         RigExecBakeReadRecorder *bakeRecorder = nullptr)
{
    bool enabled = true;
    const UsdAttribute a =
        prim ? prim.GetAttribute(_attrTokens->enabled) : UsdAttribute();
    if (a) {
        if (resolved && resolved->GetAttribute(a, time, &enabled)) {
            RigExecRecordStageRead(resolved, bakeRecorder, a.GetPath(), a, time,
                             VtValue(enabled), /*forceFrame=*/true);
            return enabled;
        }
        a.Get(&enabled, time);
    }
    if (prim) {
        // Null resolved: this arm read past the overlay, so the tier
        // check must not skip what it consumed.
        RigExecRecordStageRead(
            nullptr, bakeRecorder,
            a ? a.GetPath()
              : prim.GetPath().AppendProperty(_attrTokens->enabled),
            a, time, VtValue(enabled), /*forceFrame=*/true);
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
        primPath.AppendProperty(_attrTokens->jointIndices);
    const SdfPath weightsPath =
        primPath.AppendProperty(_attrTokens->jointWeights);
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
            for (const TfToken &name : {_attrTokens->jointIndices,
                                        _attrTokens->jointWeights,
                                        _attrTokens->elementSize}) {
                const UsdAttribute a = moverPrim.GetAttribute(name);
                if (a && (a.ValueMightBeTimeVarying() ||
                          a.HasAuthoredConnections())) {
                    return false;
                }
            }
            // No recorder: this build fills the epoch cache (a varying
            // layout is refused above), so these reads are epoch state
            // the geometry section carries. Recording them would print
            // frame-1-only records; the per-frame fallback below records
            // its own.
            topology->indices =
                _Array<int>(moverPrim, indicesPath, time, resolved);
            topology->weights =
                _Array<float>(moverPrim, weightsPath, time, resolved);
            topology->elementSize = 1;
            if (const UsdAttribute a =
                    moverPrim.GetAttribute(_attrTokens->elementSize)) {
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
    params.kind = _kindTokens->skin;
    params.enabled =
        _Enabled(moverPrim, time, resolved, _RecorderOf(resolved));
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
              moverPrim, _attrTokens->defaultWeight, 1.0f, time, resolved));
    if (!params.weights.valid) {
        return params;  // invalid common envelope => MoverFailed
    }

    params.skinTransforms = *influenceTransforms;
    const SdfPath primPath = moverPrim.GetPath();
    const SdfPath indicesPath =
        primPath.AppendProperty(_attrTokens->jointIndices);
    const SdfPath weightsPath =
        primPath.AppendProperty(_attrTokens->jointWeights);
    const auto readElementSize = [&moverPrim, time, resolved](int *out) {
        *out = 1;
        if (const UsdAttribute a =
                moverPrim.GetAttribute(_attrTokens->elementSize)) {
            if (resolved && resolved->GetAttribute(a, time, out)) {
                RigExecRecordStageRead(
                    resolved, _RecorderOf(resolved), a.GetPath(), a,
                    time, VtValue(*out), /*forceFrame=*/true);
            } else {
                a.Get(out, time);
                RigExecRecordStageRead(
                    nullptr, _RecorderOf(resolved), a.GetPath(), a,
                    time, VtValue(*out), /*forceFrame=*/false);
            }
        } else {
            RigExecRecordStageRead(
                nullptr, _RecorderOf(resolved),
                moverPrim.GetPath().AppendProperty(
                    _attrTokens->elementSize),
                UsdAttribute(), time, VtValue(*out),
                /*forceFrame=*/false);
        }
    };
    params.skinningMethod = _valueTokens->classicLinear;
    if (const UsdAttribute a =
            moverPrim.GetAttribute(_attrTokens->skinningMethod)) {
        if (resolved &&
            resolved->GetAttribute(a, time, &params.skinningMethod)) {
            RigExecRecordStageRead(
                resolved, _RecorderOf(resolved), a.GetPath(), a, time,
                VtValue(params.skinningMethod), /*forceFrame=*/true);
        } else {
            a.Get(&params.skinningMethod, time);
            RigExecRecordStageRead(
                nullptr, _RecorderOf(resolved), a.GetPath(), a, time,
                VtValue(params.skinningMethod), /*forceFrame=*/false);
        }
    } else {
        RigExecRecordStageRead(
            nullptr, _RecorderOf(resolved),
            moverPrim.GetPath().AppendProperty(
                _attrTokens->skinningMethod),
            UsdAttribute(), time, VtValue(params.skinningMethod),
            /*forceFrame=*/false);
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

    // The cache refused this mover, so the layout reads per frame and
    // records per frame with it.
    params.skinIndices = _Array<int>(moverPrim, indicesPath, time, resolved,
                                     _RecorderOf(resolved));
    params.skinWeights = _Array<float>(moverPrim, weightsPath, time,
                                       resolved, _RecorderOf(resolved));
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

// Adds `scale` times one sample's delta into `deltas`.
//
// The point of the sparse form: a real corrective moves 1,279 of 26,276
// points, so the indexed loop touches 4.87% of what the dense one does. The
// dense branch here exists for a sample that carries a layout with empty
// indices (offsets parallel to the base points), which is what an authored
// UsdSkelBlendShape with no pointIndices means, and for the dense-points
// endpoint of a mixed pair.
static void
_AccumulateBlendSample(const RigExecBlendSampleData &sample,
                       const std::vector<GfVec3f> &base,
                       float scale,
                       std::vector<GfVec3f> *deltas)
{
    if (sample.layout) {
        const RigExecBlendSampleLayout &layout = *sample.layout;
        if (layout.indices.empty()) {
            for (size_t i = 0; i < layout.offsets.size(); ++i) {
                (*deltas)[i] += layout.offsets[i] * scale;
            }
            return;
        }
        for (size_t k = 0; k < layout.indices.size(); ++k) {
            (*deltas)[size_t(layout.indices[k])] += layout.offsets[k] * scale;
        }
        return;
    }
    for (size_t i = 0; i < base.size(); ++i) {
        (*deltas)[i] += (sample.points[i] - base[i]) * scale;
    }
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
            // A sample carries its shape one of two ways, and exactly one:
            // dense moved points, or an epoch-resolved sparse layout. Both
            // have to describe THIS mesh.
            const bool shapeOk =
                sample.layout
                    ? (sample.layout->valid &&
                       sample.layout->pointCount == base.size() &&
                       sample.points.empty())
                    : sample.points.size() == base.size();
            if (!std::isfinite(sample.activation) ||
                sample.activation <= 0 || !shapeOk ||
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
        const RigExecBlendSampleData *loSample =
            hi > 0 ? &channel.samples[hi - 1] : nullptr;
        const RigExecBlendSampleData &hiSample = channel.samples[hi];

        if (!hiSample.layout && (!loSample || !loSample->layout)) {
            // The all-dense case, kept letter for letter as it was. The
            // sparse branch below is algebraically the same lerp but not
            // BIT-identical -- `dLo*(1-t) + dHi*t` and `dLo + (dHi-dLo)*t`
            // round differently in the last place -- and examples/
            // 04_BlendShapeFace.usda plus testRigExecArm's blend tests are
            // gates on this path's exact output. Nothing that only ever
            // authored targetPoints should move by even an ulp.
            const std::vector<GfVec3f> *lo =
                loSample ? &loSample->points : nullptr;
            const std::vector<GfVec3f> &hiPts = hiSample.points;
            // Each point is independent and every point's contribution keeps
            // the original channel-order accumulation, so spreading the point
            // range across workers is bit-identical to the serial loop. The
            // per-channel setup above (weights, hi/lo sample pick, t) is cheap
            // and stays sequential; only the O(points) inner loop parallelises.
            const GfVec3f *baseData = base.data();
            const GfVec3f *hiData = hiPts.data();
            const GfVec3f *loData = lo ? lo->data() : nullptr;
            GfVec3f *deltaData = deltas->data();
            const size_t nPts = base.size();
            auto denseRange = [&](size_t begin, size_t end) {
                for (size_t i = begin; i < end; ++i) {
                    const GfVec3f dHi = hiData[i] - baseData[i];
                    const GfVec3f dLo = loData ? (loData[i] - baseData[i])
                                               : GfVec3f(0);
                    deltaData[i] += dLo + (dHi - dLo) * t;
                }
            };
            if (RigExecParallelEvaluationEnabled() &&
                nPts >= RigExecGeometryParallelThreshold) {
                WorkParallelForN(nPts, denseRange, RigExecGeometryGrainSize);
            } else {
                denseRange(0, nPts);
            }
            continue;
        }

        // At least one endpoint is sparse. Written as the equivalent
        // `dLo*(1-t) + dHi*t` so each sample is visited over ITS OWN indices
        // and the union of the two index sets never has to be formed -- a
        // point only one endpoint moves simply gets one of the two terms.
        // Mixed dense/sparse endpoints fall out of this for free, which
        // matters because an in-between and its full target need not have
        // been authored the same way.
        if (loSample) {
            _AccumulateBlendSample(*loSample, base, 1.0f - t, deltas);
        }
        _AccumulateBlendSample(hiSample, base, t, deltas);
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
        : _Enabled(moverPrim, time, values.resolved, _RecorderOf(values.resolved));

    switch (op) {
    case RigExecRevisionOp::BlendShape:
        params.kind = _kindTokens->blendShape;
        break;
    case RigExecRevisionOp::VolumeCorrect:
        params.kind = _kindTokens->volumeCorrect;
        break;
    case RigExecRevisionOp::Smooth:
        params.kind = _kindTokens->smooth;
        break;
    case RigExecRevisionOp::Lattice:
        params.kind = _kindTokens->lattice;
        break;
    case RigExecRevisionOp::SurfaceProject:
        params.kind = _kindTokens->surfaceProject;
        break;
    case RigExecRevisionOp::Ribbon:
        params.kind = _kindTokens->ribbon;
        break;
    case RigExecRevisionOp::Wire:
        params.kind = _kindTokens->wire;
        break;
    case RigExecRevisionOp::EmitGuidePoints:
        params.kind = _kindTokens->emitGuidePoints;
        break;
    case RigExecRevisionOp::Curvenet:
        params.kind = _kindTokens->curvenet;
        break;
    case RigExecRevisionOp::RecomputeNormals:
        params.kind = _kindTokens->recomputeNormals;
        break;
    case RigExecRevisionOp::RecomputeExtent:
        params.kind = _kindTokens->recomputeExtent;
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
                     moverPrim, _attrTokens->defaultWeight, 1.0f, time,
                     values.resolved)));
    if (!params.weights.valid) {
        return params;  // MoverFailed, preserving the preceding revision
    }

    switch (op) {
    case RigExecRevisionOp::BlendShape: {
        params.blendDeltas = values.blendDeltas;
        const TfToken space =
            _Token(moverPrim, _attrTokens->deltaSpace, _valueTokens->target);
        if (space != "target" && space != "surfaceFrame") break;
        params.blendSurfaceFrame = space == "surfaceFrame";
        if (params.blendSurfaceFrame) {
            params.restPoints = values.basePoints;
            params.topologyCounts = _Array<int>(moverPrim, binding.topologyCounts, time, values.resolved, _RecorderOf(values.resolved));
            params.topologyIndices = _Array<int>(moverPrim, binding.topologyIndices, time, values.resolved, _RecorderOf(values.resolved));
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
        params.topologyCounts = _Array<int>(moverPrim, binding.topologyCounts, time, values.resolved, _RecorderOf(values.resolved));
        params.topologyIndices =
            _Array<int>(moverPrim, binding.topologyIndices, time, values.resolved, _RecorderOf(values.resolved));
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
        const std::vector<GfVec3f> restNet = _Array<GfVec3f>(moverPrim, binding.curvenetPoints, UsdTimeCode::Default(), /*resolved=*/nullptr, _RecorderOf(values.resolved));
        std::vector<int> splineIndices;
        if (const UsdAttribute a =
                netPrim.GetAttribute(_attrTokens->splineIndices)) {
            VtIntArray value;
            a.Get(&value, UsdTimeCode::Default());
            splineIndices.assign(value.begin(), value.end());
        }
        int samplesPerSpline = 5;
        if (const UsdAttribute a =
                netPrim.GetAttribute(_attrTokens->samplesPerSpline)) {
            a.Get(&samplesPerSpline);
        }
        const TfToken basisToken =
            _Token(netPrim, _attrTokens->basis, _valueTokens->bezier);
        params.topologyCounts =
            _Array<int>(moverPrim, binding.topologyCounts, UsdTimeCode::Default(), /*resolved=*/nullptr, _RecorderOf(values.resolved));
        params.topologyIndices =
            _Array<int>(moverPrim, binding.topologyIndices, UsdTimeCode::Default(), /*resolved=*/nullptr, _RecorderOf(values.resolved));
        // The projection surface is the target's points at DEFAULT, not at
        // the evaluated time. It is a fixed neutral pose by definition (§4.1
        // cuts against it once), and reading the animated value instead would
        // put a per-frame quantity in the bind digest -- re-cutting the mesh
        // and re-factorizing its Laplacian on every frame, while also making
        // the cut mean something different at each one.
        params.restPoints = _Array<GfVec3f>(moverPrim, binding.base, UsdTimeCode::Default(), /*resolved=*/nullptr, _RecorderOf(values.resolved));
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

        // Retain the bind's inputs for the bake, on whichever path read
        // them: the early path below skips the digest and the cache, but the
        // reads above already happened, and a remembered failure re-binds
        // from the same arrays a success did.
        if (values.curvenetBindInputs && !values.curvenetBindInputs->held) {
            values.curvenetBindInputs->restNet = restNet;
            values.curvenetBindInputs->splineIndices = splineIndices;
            values.curvenetBindInputs->samplesPerSpline = samplesPerSpline;
            values.curvenetBindInputs->basis = basisToken;
            values.curvenetBindInputs->meshPoints = params.restPoints;
            values.curvenetBindInputs->meshCounts = params.topologyCounts;
            values.curvenetBindInputs->meshIndices = params.topologyIndices;
            values.curvenetBindInputs->held = true;
        }
        // A caller that resolved the bind for itself -- because it may not
        // touch the cache where it assembles -- says so here, before the
        // digest, which is the cache's KEY and nothing else. Everything
        // above this line is still read and still lands in the packet: the
        // rest surface and the topology are compared per frame whoever bound
        // the mover.
        if (values.curvenetBinding) {
            params.curvenetBinding = *values.curvenetBinding;
            params.valid = params.curvenetBinding != nullptr;
            break;
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
        params.auxPoints = _Array<GfVec3f>(moverPrim, binding.cagePoints, UsdTimeCode::Default(), /*resolved=*/nullptr, _RecorderOf(values.resolved));
        params.auxPointsB = _Array<GfVec3f>(moverPrim, binding.cagePoints, time, values.resolved, _RecorderOf(values.resolved));
        if (const UsdAttribute a =
                moverPrim.GetAttribute(_attrTokens->divisions)) {
            a.Get(&params.divisions, time);
            RigExecRecordStageRead(
                nullptr, _RecorderOf(values.resolved), a.GetPath(), a,
                time, VtValue(params.divisions), /*forceFrame=*/false);
        } else {
            RigExecRecordStageRead(
                nullptr, _RecorderOf(values.resolved),
                moverPrim.GetPath().AppendProperty(
                    _attrTokens->divisions),
                UsdAttribute(), time, VtValue(params.divisions),
                /*forceFrame=*/false);
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
        params.auxPoints = _Array<GfVec3f>(moverPrim, binding.surfacePoints, time, values.resolved, _RecorderOf(values.resolved));
        params.topologyCounts = _Array<int>(moverPrim, binding.topologyCounts, time, values.resolved, _RecorderOf(values.resolved));
        params.topologyIndices =
            _Array<int>(moverPrim, binding.topologyIndices, time, values.resolved, _RecorderOf(values.resolved));
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
                _Array<GfVec2f>(moverPrim, binding.bindCoords, time, values.resolved, _RecorderOf(values.resolved));
            params.valid = !params.bindCoords.empty();
        } else {
            params.valid = true;
        }
        break;
    }

    case RigExecRevisionOp::Wire: {
        // Posed control points at the declared phase; rest control points
        // are the curve's AUTHORED ones, which is what the bind coordinates
        // were computed against.
        if (const UsdAttribute a = moverPrim.GetStage()->GetAttributeAtPath(
                binding.driverCurvePoints)) {
            VtVec3fArray rest;
            a.Get(&rest, UsdTimeCode::Default());
            RigExecRecordStageRead(
                nullptr, _RecorderOf(values.resolved), a.GetPath(), a,
                UsdTimeCode::Default(), VtValue(rest),
                /*forceFrame=*/false);
            params.restPoints.assign(rest.begin(), rest.end());
        } else if (!binding.driverCurvePoints.IsEmpty()) {
            RigExecRecordStageRead(
                nullptr, _RecorderOf(values.resolved),
                binding.driverCurvePoints, UsdAttribute(),
                UsdTimeCode::Default(), VtValue(VtVec3fArray()),
                /*forceFrame=*/false);
        }
        if (binding.driverTransformCount > 0) {
            // Posed control points from the providers: C_j = C0_j +
            // w_j (M_j C0_j - C0_j), M_j the transform measured in its
            // space. A periodic curve's repeated points wrap onto the
            // unique ones, so one entry per unique point is enough.
            const size_t t = size_t(binding.driverTransformCount);
            const size_t s = size_t(binding.driverSpaceCount);
            const size_t bt = size_t(binding.driverBaseTransformCount);
            const std::vector<GfMatrix4d> *table = values.influenceTransforms;
            if (!table || table->size() < t + s + bt) {
                break;  // MoverFailed
            }
            const size_t bs = table->size() - t - s - bt;
            const auto floats = [&](const char *name) {
                VtFloatArray out;
                if (const UsdAttribute a =
                        moverPrim.GetAttribute(TfToken(name))) {
                    if (values.resolved && values.resolved->GetAttribute(
                                               a, time, &out)) {
                        RigExecRecordStageRead(
                            values.resolved,
                            _RecorderOf(values.resolved), a.GetPath(), a,
                            time, VtValue(out), /*forceFrame=*/true);
                    } else {
                        a.Get(&out, time);
                        RigExecRecordStageRead(
                            nullptr, _RecorderOf(values.resolved),
                            a.GetPath(), a, time, VtValue(out),
                            /*forceFrame=*/false);
                    }
                } else {
                    RigExecRecordStageRead(
                        nullptr, _RecorderOf(values.resolved),
                        moverPrim.GetPath().AppendProperty(TfToken(name)),
                        UsdAttribute(), time, VtValue(out),
                        /*forceFrame=*/false);
                }
                return out;
            };
            const VtFloatArray weights = floats("inputs:driverWeights");
            const VtFloatArray baseWeights = floats("inputs:driverBaseWeights");
            const auto pick = [](size_t count, size_t j) {
                return count <= 1 ? size_t(0) : j % count;
            };
            const auto measured = [&](size_t first, size_t count,
                                      size_t spaceFirst, size_t spaceCount,
                                      size_t j) {
                GfMatrix4d m = (*table)[first + pick(count, j)];
                if (spaceCount > 0) {
                    m = RigExecMeasureInSpace(
                        m, (*table)[spaceFirst + pick(spaceCount, j)]);
                }
                return m;
            };
            params.auxPoints.resize(params.restPoints.size());
            for (size_t j = 0; j < params.restPoints.size(); ++j) {
                GfVec3f &rest = params.restPoints[j];
                // A base motion moves the curve AND its rest: the wire
                // then deforms by the driver's motion on top of it, as a
                // wire does whose base curve rides the same deformers.
                if (bt > 0) {
                    const GfMatrix4d b = measured(t + s, bt, t + s + bt, bs, j);
                    const float wb = baseWeights.empty()
                        ? 1.0f : baseWeights[pick(baseWeights.size(), j)];
                    const GfVec3f moved(b.TransformAffine(GfVec3d(rest)));
                    rest = rest + (moved - rest) * wb;
                }
                const GfMatrix4d m = measured(0, t, t, s, j);
                const float w =
                    weights.empty() ? 1.0f : weights[pick(weights.size(), j)];
                const GfVec3f moved(m.TransformAffine(GfVec3d(rest)));
                params.auxPoints[j] = rest + (moved - rest) * w;
            }
        } else {
            params.auxPoints = _Array<GfVec3f>(
                moverPrim, binding.driverCurvePoints, time, values.resolved, _RecorderOf(values.resolved));
        }
        if (const UsdAttribute a = moverPrim.GetStage()->GetAttributeAtPath(
                binding.driverCurveOrder)) {
            VtIntArray order;
            if (a.Get(&order, UsdTimeCode::Default()) && !order.empty()) {
                params.curveOrder = order[0];
            }
            RigExecRecordStageRead(
                nullptr, _RecorderOf(values.resolved), a.GetPath(), a,
                UsdTimeCode::Default(), VtValue(order),
                /*forceFrame=*/false);
        } else if (!binding.driverCurveOrder.IsEmpty()) {
            RigExecRecordStageRead(
                nullptr, _RecorderOf(values.resolved),
                binding.driverCurveOrder, UsdAttribute(),
                UsdTimeCode::Default(), VtValue(VtIntArray()),
                /*forceFrame=*/false);
        }
        if (const UsdAttribute a = moverPrim.GetStage()->GetAttributeAtPath(
                binding.driverCurveKnots)) {
            VtDoubleArray knots;
            a.Get(&knots, UsdTimeCode::Default());
            RigExecRecordStageRead(
                nullptr, _RecorderOf(values.resolved), a.GetPath(), a,
                UsdTimeCode::Default(), VtValue(knots),
                /*forceFrame=*/false);
            params.curveKnots.assign(knots.begin(), knots.end());
        } else if (!binding.driverCurveKnots.IsEmpty()) {
            RigExecRecordStageRead(
                nullptr, _RecorderOf(values.resolved),
                binding.driverCurveKnots, UsdAttribute(),
                UsdTimeCode::Default(), VtValue(VtDoubleArray()),
                /*forceFrame=*/false);
        }
        if (const UsdAttribute a = moverPrim.GetAttribute(
                TfToken("inputs:dropoffDistance"))) {
            float dropoff = 0.0f;
            a.Get(&dropoff, time);
            RigExecRecordStageRead(
                nullptr, _RecorderOf(values.resolved), a.GetPath(), a,
                time, VtValue(dropoff), /*forceFrame=*/false);
            params.dropoffDistance = dropoff;
        } else {
            RigExecRecordStageRead(
                nullptr, _RecorderOf(values.resolved),
                moverPrim.GetPath().AppendProperty(
                    TfToken("inputs:dropoffDistance")),
                UsdAttribute(), time, VtValue(0.0f),
                /*forceFrame=*/false);
        }
        if (!binding.bindCoords.IsEmpty()) {
            if (!values.resolved ||
                !values.resolved->Get(binding.bindCoords,
                                      &params.wireBindCoords)) {
                if (const UsdAttribute a =
                        moverPrim.GetStage()->GetAttributeAtPath(
                            binding.bindCoords)) {
                    a.Get(&params.wireBindCoords, time);
                    RigExecRecordStageRead(
                        nullptr, _RecorderOf(values.resolved),
                        a.GetPath(), a, time,
                        VtValue(params.wireBindCoords),
                        /*forceFrame=*/false);
                } else {
                    RigExecRecordStageRead(
                        nullptr, _RecorderOf(values.resolved),
                        binding.bindCoords, UsdAttribute(), time,
                        VtValue(params.wireBindCoords),
                        /*forceFrame=*/false);
                }
            }
        }
        const RigExecNurbsCurve rest{&params.restPoints, params.curveOrder,
                                     &params.curveKnots};
        params.valid = rest.IsValid() &&
                       params.auxPoints.size() == params.restPoints.size() &&
                       !params.wireBindCoords.empty();
        break;
    }

    case RigExecRevisionOp::RecomputeNormals:
    case RigExecRevisionOp::RecomputeExtent:
        // Derived maintenance reads the final same-generation points, which
        // the caller supplies as the base value for this revision.
        params.auxPoints = values.basePoints;
        params.topologyCounts = _Array<int>(moverPrim, binding.topologyCounts, time, values.resolved, _RecorderOf(values.resolved));
        params.topologyIndices =
            _Array<int>(moverPrim, binding.topologyIndices, time, values.resolved, _RecorderOf(values.resolved));
        // Authored widths widen the extent bounds; omitting them silently
        // under-reports the bound of a curves/points gprim.
        if (op == RigExecRevisionOp::RecomputeExtent) {
            params.widths = _Array<float>(moverPrim, binding.widths, time, values.resolved, _RecorderOf(values.resolved));
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
