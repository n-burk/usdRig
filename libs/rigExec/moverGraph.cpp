// RigExec compiled mover graph (spec §7.2). See moverGraph.h.
#include "moverGraph.h"
#include "parallel.h"
#include "frameExtraction.h"
#include "movers/moverRegistry.h"

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

#include <algorithm>
#include <atomic>
#include <cmath>
#include <iterator>
#include <memory>
#include <mutex>
#include <set>
#include <type_traits>
#include <unordered_map>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

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
    ((deltaMush, "deltaMush"))
    ((wrinkle, "wrinkle"))
    ((lattice, "lattice"))
    ((surfaceProject, "surfaceProject"))
    ((ribbon, "ribbon"))
    ((wire, "wire"))
    ((emitGuidePoints, "emitGuidePoints"))
    ((external, "external"))
    ((recomputeNormals, "recomputeNormals"))
    ((recomputeExtent, "recomputeExtent"))
    ((surfaceProjector, "surfaceProjector"))
    ((shaderDials, "shaderDials"))
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
    ((divisions, "rigExec:divisions"))
    ((restPoints, "inputs:restPoints"))
    ((iterations, "inputs:iterations"))
    ((topology, "inputs:topology"))
    ((neighborDistance, "inputs:neighborDistance"))
    ((restLengthScale, "inputs:restLengthScale"))
    ((stretchStiffness, "inputs:stretchStiffness"))
    ((compressionStiffness, "inputs:compressionStiffness"))
    ((bendStiffness, "inputs:bendStiffness"))
    ((maxDisplacement, "inputs:maxDisplacement"))
    ((pinBorders, "inputs:pinBorders"))
    ((pinPoints, "inputs:pinPoints"))
    ((tangentPlaneCollisions, "inputs:tangentPlaneCollisions"))
    ((tangentPlaneInset, "inputs:tangentPlaneInset"))
    ((wrinkleScale, "inputs:wrinkleScale"))
    ((smoothingIterations, "inputs:smoothingIterations"))
    ((weightBlend, "rigExec:weightBlend"))
    ((pointFrame, "rigExec:pointFrame"))
    ((step, "inputs:step"))
    ((distanceWeight, "inputs:distanceWeight"))
    ((displacement, "inputs:displacement"))
    ((driverWeights, "inputs:driverWeights"))
    ((driverBaseWeights, "inputs:driverBaseWeights"))
    ((driverDeltaFrame, "rigExec:driverDeltaFrame"))
    ((dropoffDistance, "inputs:dropoffDistance"))
    ((rayOrigin, "rigExec:rayOrigin"))
    ((rayDirection, "rigExec:rayDirection"))
    ((rayUp, "rigExec:rayUp"))
    ((shaderOffset, "rigExec:shaderOffset"))
    ((projectionMode, "rigExec:projectionMode"))
);

// The values those reads fall back to, and the three states a revision's
// status reports. Same reason.
TF_DEFINE_PRIVATE_TOKENS(
    _valueTokens,
    ((classicLinear, "classicLinear"))
    ((target, "target"))
    ((disabled, "disabled"))
    ((ok, "ok"))
    ((moverFailed, "moverFailed"))
    ((cloth, "cloth"))
    ((surfaceStruts, "surfaceStruts"))
    ((rest, "rest"))
    ((local, "local"))
    ((material, "material"))
    ((radial, "radial"))
);

namespace rigExec {

namespace {

const TfToken _noKindToken;
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
    case RigExecRevisionOp::DeltaMush:
        return _kindTokens->deltaMush;
    case RigExecRevisionOp::Wrinkle:
        return _kindTokens->wrinkle;
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
    case RigExecRevisionOp::External:
        return _kindTokens->external;
    case RigExecRevisionOp::RecomputeNormals:
        return _kindTokens->recomputeNormals;
    case RigExecRevisionOp::RecomputeExtent:
        return _kindTokens->recomputeExtent;
    case RigExecRevisionOp::SurfaceProjector:
        return _kindTokens->surfaceProjector;
    case RigExecRevisionOp::ShaderDials:
        return _kindTokens->shaderDials;
    }
    // No runtime dispatch beyond the frozen operation set: an unhandled op is
    // a build error, not a silently mismatched packet.
    return _noKindToken;
}

// RIGEXEC_ENABLE_SIMD, read once while the library loads: a namespace-scope
// constant, so a kernel on a worker reads it with no guard and no lock.
const bool _simdEnabled = TfGetenvBool("RIGEXEC_ENABLE_SIMD", true);

}  // namespace

bool
RigExecSimdEnabled()
{
    return _simdEnabled;
}

void
RigExecRevisionKernelTouchTokens()
{
    (void)_kindTokens.Get();
    (void)_attrTokens.Get();
    (void)_valueTokens.Get();
}

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
// One definition, so a second caller cannot drift into a different movement
// -- including over the SIMD choice, which must be the same on both paths or
// the two disagree in the last bits.
void
RigExecApplyMatrixKernelRange(const RigExecMoverParameters &p,
                              const float *envelope,
                              size_t begin, size_t end, GfVec3f *pts,
                              bool useSimd)
{
    if (p.radialWeight) {
        // A fraction of the ROTATION, not of the position: the linear form
        // below follows the chord of the arc, and a chord falls inside its
        // arc, toward the axis. Factored per WEIGHT rather than per point,
        // so a falloff that is mostly 0 and 1 -- which is most of them --
        // costs two factorizations and not one per point.
        double cachedWeight = -1.0;
        RigExecPartialDecomposition decomposition;
        bool haveDecomposition=false;
        GfMatrix4d partial(1.0);
        for (size_t i = begin; i < end; ++i) {
            const double w = envelope[i];
            if (w != cachedWeight) {
                const double clamped=GfClamp(w,0.0,1.0);
                if(clamped<=0.0) partial=GfMatrix4d(1.0);
                else if(clamped>=1.0) partial=p.transform;
                else {
                    if(!haveDecomposition) {
                        decomposition=RigExecDecomposePartialTransform(p.transform);
                        haveDecomposition=true;
                    }
                    partial=RigExecApplyPartialDecomposition(decomposition,clamped);
                }
                cachedWeight = w;
            }
            pts[i] = GfVec3f(partial.TransformAffine(GfVec3d(pts[i])));
        }
        return;
    }
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

uint64_t
_HashBytes(uint64_t h, const void *data, size_t size)
{
    const unsigned char *bytes = static_cast<const unsigned char *>(data);
    for (size_t i = 0; i < size; ++i) {
        h = (h ^ bytes[i]) * 1099511628211ull;
    }
    return h;
}

}  // namespace

// The inputs a basis was built from, kept beside it so a hit compares them
// in full. Immutable once inserted: copies of a cache share it.
struct RigExecWireBasisCache::_Entry {
    std::vector<GfVec2f> binds;
    std::vector<int> indices;
    std::vector<double> knots;
    int order = 0;
    size_t controlPoints = 0;
    size_t meshPoints = 0;
    double dropoff = 0.0;
    std::shared_ptr<const RigExecWireBasis> basis;
};

std::shared_ptr<const RigExecWireBasis>
RigExecWireBasisCache::Get(const RigExecMoverParameters &p,
                           const std::vector<int> &indices, size_t meshPoints)
{
    const auto matches=[&](const _Entry &e) {
        if(e.order!=p.curveOrder || e.controlPoints!=p.restPoints.size() ||
            e.meshPoints!=meshPoints || e.indices!=indices ||
            e.knots.size()!=p.curveKnots.size() || e.binds.size()!=p.wireBindCoords.size() ||
            std::memcmp(&e.dropoff,&p.dropoffDistance,sizeof(double))) return false;
        for(size_t i=0;i<e.knots.size();++i)
            if(std::memcmp(&e.knots[i],&p.curveKnots[i],sizeof(double))) return false;
        for(size_t i=0;i<e.binds.size();++i) for(int axis=0;axis<2;++axis) {
            const float a=e.binds[i][axis],b=p.wireBindCoords[i][axis];
            if(std::memcmp(&a,&b,sizeof(a))) return false;
        }
        return true;
    };
    if(_last && matches(*_last)) return _last->basis;
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

    const auto found = _entries.find(h);
    if (found != _entries.end()) {
        const _Entry &e = *found->second;
        if(matches(e)) { _last=found->second; return e.basis; }
    }
    auto basis = std::make_shared<RigExecWireBasis>();
    if (!RigExecBuildWireBasis(p.wireBindCoords.cdata(),
                               p.wireBindCoords.size(), meshPoints, indices,
                               p.curveOrder, p.curveKnots, controlPoints,
                               p.dropoffDistance, basis.get())) {
        return nullptr;
    }
    ++_builds;
    auto entry = std::make_shared<_Entry>();
    entry->binds.assign(p.wireBindCoords.cbegin(), p.wireBindCoords.cend());
    entry->indices = indices;
    entry->knots = p.curveKnots;
    entry->order = p.curveOrder;
    entry->controlPoints = controlPoints;
    entry->meshPoints = meshPoints;
    entry->dropoff = p.dropoffDistance;
    entry->basis = basis;
    if (_entries.size() >= kCapacity) {
        // Edits accumulate entries nothing will ask for again.
        _entries.clear();
    }
    _last=entry;
    _entries[h] = std::move(entry);
    return basis;
}

namespace {
// The full-strength matrix movement, which rewrites every point: \p in and
// \p out may be the same array, since point i is written only after it is
// read.
void
ApplyMatrixFullStrength(const RigExecMoverParameters &p, const GfVec3f *in,
                        GfVec3f *out, size_t count, bool useSimd)
{
    if (p.radialWeight) {
        for (size_t i = 0; i < count; ++i)
            out[i] = GfVec3f(p.transform.TransformAffine(GfVec3d(in[i])));
    } else if (useSimd) {
        RigExecApplyWeightedMatrixSimd(in,out,1.0f,count,p.transform);
    } else {
        for (size_t i = 0; i < count; ++i)
            out[i] = GfVec3f(RigExecApplyWeightedMatrix(GfVec3d(in[i]),p.transform,1.0));
    }
}
}  // namespace

// The validation the matrix, blend-shape and wire kernels run before they
// write, each ONE definition that the kernel runs and
// RigExecRevisionKernelAcceptance decides by, so the two cannot disagree.
namespace {

// The packet check every revision kernel makes first.
bool
_PacketMatches(RigExecRevisionOp op, const RigExecMoverParameters &p)
{
    return p.valid && p.kind == _RevisionKindToken(op);
}

// The sparse field the matrix kernel walks point by point instead of
// resolving it densely: the shape a wire takes its envelope in.
bool
_MatrixWalksSparse(const RigExecWeightPacket &w)
{
    return RigExecWireTakesSparseEnvelope(w);
}

// Every entry a sparse walk names is usable over \p count points: in range,
// strictly ascending, finite and in [0, 1]. Validated exactly as ResolveAll
// would, so the same packets fail.
bool
_SparseWalkIsUsable(const RigExecWeightPacket &w, size_t count)
{
    for (size_t k = 0; k < w.indices.size(); ++k) {
        const int index = w.indices[k];
        const float value = w.values[k];
        if (index < 0 || size_t(index) >= count ||
            (k > 0 && index <= w.indices[k - 1]) ||
            !std::isfinite(value) || value < 0.0f || value > 1.0f) {
            return false;
        }
    }
    return true;
}

// RigExecApplyMatrixKernel's validation over \p count points: a full-strength
// envelope, a sparse walk whose entries are usable, or a dense resolve. Given
// \p weights, the dense resolve writes into it (ResolveAll validates exactly
// as ResolvesAll does), so the kernel validates once.
bool
_MatrixKernelAccepts(const RigExecMoverParameters &p, size_t count,
                     std::vector<float> *weights)
{
    const RigExecWeightPacket &w = p.weights;
    if (RigExecEnvelopeIsFullStrength(w)) {
        return true;
    }
    if (_MatrixWalksSparse(w)) {
        return _SparseWalkIsUsable(w, count);
    }
    // A cardinality mismatch fails atomically.
    return weights ? w.ResolveAll(count, weights) : w.ResolvesAll(count);
}

// RigExecApplyBlendShapeKernel's validation over \p count points: one delta
// per point and an envelope that resolves, into \p envelope when given (left
// untouched at full strength). A surface-frame transport after it reads the
// entering points and can still refuse them.
bool
_BlendShapeKernelAccepts(const RigExecMoverParameters &p, size_t count,
                         std::vector<float> *envelope)
{
    if (p.blendDeltas.size() != count) {
        return false;
    }
    if (RigExecEnvelopeIsFullStrength(p.weights)) {
        return true;
    }
    return envelope ? p.weights.ResolveAll(count, envelope)
                    : p.weights.ResolvesAll(count);
}

// The checks the wire operation makes over \p count points before it builds
// or applies anything: both control polygons alike and evaluable, then a
// sparse walk's entries, or a dense walk's one bind coordinate per point.
bool
_WireKernelPrefix(const RigExecMoverParameters &p, size_t count)
{
    if (p.auxPoints.size() != p.restPoints.size()) {
        return false;
    }
    const RigExecNurbsCurve rest{&p.restPoints, p.curveOrder, &p.curveKnots};
    const RigExecNurbsCurve posed{&p.auxPoints, p.curveOrder, &p.curveKnots};
    if (!rest.IsValid() || !posed.IsValid()) {
        return false;
    }
    if (RigExecWireTakesSparseEnvelope(p.weights)) {
        return _SparseWalkIsUsable(p.weights, count);
    }
    // A sparse bind table needs a sparse envelope.
    return p.wireBindCoords.size() == count;
}

// The wire operation's answer over \p count points: its prefix, then the
// checks of the basis build or the dense walk it goes on to make
// (RigExecWireBasisInputsAreUsable, RigExecWireInputsAreUsable). An empty
// bind table is Deferred: its answer is whether the table has storage, which
// a memoized basis never asks.
RigExecRevisionAcceptance
_WireKernelAcceptance(const RigExecMoverParameters &p, size_t count)
{
    using Acceptance = RigExecRevisionAcceptance;
    if (!_WireKernelPrefix(p, count)) {
        return Acceptance::Refuses;
    }
    if (p.wireBindCoords.empty()) {
        return Acceptance::Deferred;
    }
    if (RigExecWireTakesSparseEnvelope(p.weights)) {
        // The indices were proved in range above, and a memoized basis exists
        // only for inputs that passed these same checks.
        return RigExecWireBasisInputsAreUsable(
                   p.wireBindCoords.cdata(), p.wireBindCoords.size(), count,
                   p.weights.indices.size(), p.curveOrder, p.curveKnots,
                   p.restPoints.size())
            ? Acceptance::Applies
            : Acceptance::Refuses;
    }
    const RigExecNurbsCurve rest{&p.restPoints, p.curveOrder, &p.curveKnots};
    const RigExecNurbsCurve posed{&p.auxPoints, p.curveOrder, &p.curveKnots};
    return RigExecWireInputsAreUsable(rest, posed, p.wireBindCoords.cdata(),
                                      p.wireBindCoords.size(), count)
        ? Acceptance::Applies
        : Acceptance::Refuses;
}

} // namespace

bool
RigExecApplyMatrixKernel(const RigExecMoverParameters &p,
                         std::vector<GfVec3f> *pts, bool useSimd)
{
    const size_t count = pts->size();
    // Validated before any point is written; a dense envelope is resolved
    // into `weights` by the same call.
    std::vector<float> weights;
    if (!_MatrixKernelAccepts(p, count, &weights)) {
        return false;
    }
    if (RigExecEnvelopeIsFullStrength(p.weights)) {
        ApplyMatrixFullStrength(p, pts->data(), pts->data(), count, useSimd);
        return true;
    }
    // A sparse field with a zero default touches only its named points: a
    // face cluster weights a few hundred of a body's tens of thousands, so
    // resolving and walking the dense array is almost all waste. Its entries
    // were validated above, exactly as ResolveAll would, so the same packets
    // fail.
    const RigExecWeightPacket &w = p.weights;
    if (_MatrixWalksSparse(w)) {
        if (p.transform == GfMatrix4d(1.0)) {
            return true;  // at rest every weighted point maps to itself
        }
        GfVec3f *data = pts->data();
        if (p.radialWeight) {
            // The sparse walk must take the same arc the dense kernel
            // does. A painted cluster falloff is almost always sparse --
            // the upper blink weights 414 points of a 26276-point body --
            // so EVERY radial cluster on this rig arrived here and got a
            // linear blend, the one case RigExecPartialTransform exists to
            // prevent. Measured: at a full blink the lid's half-weighted
            // points cut the chord and sank 0.2951 into an eyeball of
            // radius 2.4846, which is the lid clipping through the eye.
            //
            // Factored per WEIGHT, as the dense kernel is, and for the
            // same reason: a falloff is mostly a few distinct values.
            double cachedWeight = -1.0;
            RigExecPartialDecomposition decomposition;
            bool haveDecomposition=false;
            GfMatrix4d partial(1.0);
            for (size_t k = 0; k < w.indices.size(); ++k) {
                const double value = w.values[k];
                if (value != cachedWeight) {
                    const double clamped=GfClamp(value,0.0,1.0);
                    if(clamped<=0.0)partial=GfMatrix4d(1.0);
                    else if(clamped>=1.0)partial=p.transform;
                    else {
                        if(!haveDecomposition) {
                            decomposition=RigExecDecomposePartialTransform(p.transform);
                            haveDecomposition=true;
                        }
                        partial=RigExecApplyPartialDecomposition(decomposition,clamped);
                    }
                    cachedWeight = value;
                }
                GfVec3f &point = data[size_t(w.indices[k])];
                point = GfVec3f(partial.TransformAffine(GfVec3d(point)));
            }
            return true;
        }
        for (size_t k = 0; k < w.indices.size(); ++k) {
            GfVec3f &point = data[size_t(w.indices[k])];
            point = GfVec3f(RigExecApplyWeightedMatrix(
                GfVec3d(point), p.transform, w.values[k]));
        }
        return true;
    }
    RigExecApplyMatrixKernelRange(p, weights.data(), 0, count, pts->data(),
                                  useSimd);
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
// It is now three pieces rather than one, because a chunked caller needs the
// per-vertex body without the decisions AROUND it -- and every one of those
// decisions is a statement about the WHOLE array, not about a vertex:
// the layout's element shape, index range and weights; the influence table's
// finite/affine check; the method token. So the validation splits in two by
// what it reads (RigExecSkinLayoutIsUsable, RigExecSkinTransformsAreUsable),
// the per-vertex body becomes RigExecApplySkinKernelRange, and this function
// is what it always was: validate, then run every vertex.

RigExecSkinMethod
RigExecSkinMethodOf(const RigExecMoverParameters &p)
{
    if (p.skinningMethod == "classicLinear") {
        return RigExecSkinMethod::ClassicLinear;
    }
    if (p.skinningMethod == "dualQuaternion") {
        return RigExecSkinMethod::DualQuaternion;
    }
    return RigExecSkinMethod::Unknown;
}

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
                            std::vector<GfVec3f> *pts, bool useSimd)
{
    const RigExecSkinMethod method = RigExecSkinMethodOf(p);
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

    // The one point where the skinning methods part. Everything above is
    // the shared per-point gather (indices, weights, influence matrices,
    // rest point); only the accumulation differs.
    if (method == RigExecSkinMethod::ClassicLinear) {
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
    if (method == RigExecSkinMethod::DualQuaternion) {
        // Scale-aware DQS from libs/rigExecMath/dualQuat.h: each influence
        // split once per evaluation into a pre-rotation stretch and a unit
        // dual quaternion, weighted sum over the same layout with
        // shortest-arc sign correction, ONE normalisation, then the direct
        // point transform. Weight shortfall enters as an identity influence
        // (see RigExecApplyDualQuatSkin). Scalar only; a degenerate blend
        // fails atomically.
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
    std::vector<GfVec3f> *pts, bool useSimd)
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
        return RigExecApplySkinKernelRange(p, transforms, 0, count, pts,
                                           useSimd);
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
        [&p, &shared, pts, &ok, useSimd](size_t begin, size_t end) {
            if (!RigExecApplySkinKernelRange(p, shared, begin, end, pts,
                                             useSimd)) {
                ok.store(false, std::memory_order_relaxed);
            }
        },
        RigExecGeometryGrainSize);
    return ok.load(std::memory_order_relaxed);
}

bool
RigExecApplySkinKernel(const RigExecMoverParameters &p,
                       std::vector<GfVec3f> *pts, bool useSimd)
{
    return RigExecApplySkinKernelWithTransforms(
        p, RigExecSkinTransformsOf(p), pts, useSimd);
}

// The blend-shape kernel, shared by the mover-graph revision node and by the
// baked program. Like the matrix kernel and unlike the point3f[] ops the
// envelope is NOT a separate blend: the deltas are added to the preceding
// revision and the result blended back against it in one pass, so resolving
// the envelope is part of the kernel.
// One definition, so a second caller cannot drift into a different blend.
bool
RigExecApplyBlendShapeKernel(const RigExecMoverParameters &p,
                             std::vector<GfVec3f> *pts,
    RigExecSurfaceKernelCache<GfVec3f,GfVec3d> *cache)
{
    const size_t count = pts->size();
    // Resolve the common envelope up front so a cardinality failure fails the
    // application before any element is written (the in-place write cannot be
    // rolled back).
    std::vector<float> envelope;
    if (!_BlendShapeKernelAccepts(p, count, &envelope)) {
        return false;
    }
    const bool fullStrength = RigExecEnvelopeIsFullStrength(p.weights);

    std::vector<GfVec3f> transported;
    const std::vector<GfVec3f> *deltas = &p.blendDeltas;
    if (p.blendSurfaceFrame) {
        // The incoming points ARE the posed surface the offsets ride on.
        const auto *rest=cache ? cache->TransportRest(p.restPoints,
            p.topologyCounts,p.topologyIndices) : nullptr;
        const bool ok=cache ? (rest && RigExecApplyTransportWithRestData(
            *pts,p.topologyCounts,p.topologyIndices,p.blendDeltas,*rest,&transported))
            : RigExecTransportSurfaceOffsets(p.restPoints,*pts,p.topologyCounts,
                p.topologyIndices,p.blendDeltas,&transported);
        if (!ok) {
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
    const auto blendRange = [points, delta, weight, fullStrength](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            const GfVec3f preceding = points[i];
            points[i] = RigExecBlendEnvelope(
                preceding, preceding + delta[i], fullStrength ? 1.0f : weight[i]);
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
// One definition, so the size rules and the envelope cannot drift between the
// two paths that maintain the same property.
bool
RigExecApplyDerivedKernel(RigExecRevisionOp op,
                          const RigExecMoverParameters &p,
                          const GfVec3f *authored, size_t authoredCount,
                          std::vector<GfVec3f> *result)
{
    if (!result || (authoredCount && !authored)) return false;
    const bool extent = op == RigExecRevisionOp::RecomputeExtent;
    std::vector<GfVec3f> values =
        extent ? RigExecComputeExtent(p.auxPoints, p.widths)
               : RigExecComputeVertexNormals(p.auxPoints, p.topologyCounts,
                                             p.topologyIndices);
    if (values.empty() || (extent && values.size() != 2) ||
        authoredCount != values.size()) return false;
    if (!RigExecEnvelopeIsFullStrength(p.weights)) {
        std::vector<float> envelope;
        if (!p.weights.ResolveAll(values.size(), &envelope)) return false;
        for (size_t i = 0; i < values.size(); ++i)
            values[i] = RigExecBlendEnvelope(authored[i], values[i], envelope[i]);
    }
    // Do not touch the caller's output until all original refusal checks pass.
    *result = std::move(values);
    return true;
}

bool
RigExecApplyDerivedKernel(RigExecRevisionOp op,
                          const RigExecMoverParameters &p,
                          std::vector<GfVec3f> *pts)
{
    return pts && RigExecApplyDerivedKernel(op,p,pts->data(),pts->size(),pts);
}

// Every revision operation, over the same kernels the revision node ran when
// they were lambdas inside its VdfContext callback.
// ONE definition, called by the mover-graph revision node and by the baked
// program: a second copy of a deformation agrees on the fixtures that exist
// and drifts on the ones that do not.
// The envelope is NOT applied here for the ops that take a separate blend --
// RigExecRunRevisionKernel wraps this, which is where the "apply once" rule
// lives; matrix, blendShape and the two derived recomputations fold it into
// their own arithmetic and are routed there instead.
namespace {
bool
ApplyRevisionKernel(RigExecRevisionOp op,
                           const RigExecMoverParameters &p,
                           std::vector<GfVec3f> *pts, bool useSimd,
                           RigExecWireBasisCache *wireBasis,
    RigExecSurfaceKernelCache<GfVec3f,GfVec3d> *cache, bool discardable)
{
    switch (op) {
    case RigExecRevisionOp::Matrix:
        return RigExecApplyMatrixKernel(p, pts, useSimd);
    case RigExecRevisionOp::Skin:
        return RigExecApplySkinKernel(p, pts, useSimd);
    case RigExecRevisionOp::BlendShape:
        return RigExecApplyBlendShapeKernel(p, pts,cache);
    case RigExecRevisionOp::VolumeCorrect:
        RigExecApplyVolumeCorrect(pts, p.referenceVolume, p.strength);
        return true;
    case RigExecRevisionOp::Smooth:
        if(cache) {
            const auto *adj=cache->MeshAdjacency(pts->size(),p.topologyCounts,p.topologyIndices);
            if(adj) RigExecApplyLaplacianSmoothWithAdjacency(pts,*adj,p.strength);
        } else RigExecApplyLaplacianSmooth(
            pts,p.topologyCounts,p.topologyIndices,p.strength);
        return true;
    case RigExecRevisionOp::DeltaMush: {
        const auto *rest=cache ? cache->MushRest(p.restPoints,p.topologyCounts,
            p.topologyIndices,p.mushIterations,p.mushStep,p.mushPinBorders,p.mushDistanceWeight)
            : nullptr;
        if(cache && !rest) return false;
        return RigExecApplyDeltaMush(pts,p.restPoints,p.topologyCounts,
            p.topologyIndices,p.mushIterations,p.mushStep,p.mushPinBorders,
            p.mushDistanceWeight,p.mushDisplacement,rest);
    }
    case RigExecRevisionOp::Wrinkle: {
        const auto *topology=cache ? cache->WrinkleTopology(p.restPoints.size(),
            p.topologyCounts,p.topologyIndices,p.wrinkleSettings.topology,
            p.wrinkleSettings.neighborDistance) : nullptr;
        if(cache && !topology) return false;
        return RigExecApplyWrinkle(pts,p.restPoints,p.topologyCounts,
            p.topologyIndices,p.wrinkleSettings,topology);
    }
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
        // The checks before anything is built or applied; the basis build
        // and RigExecApplyWire below make the rest of the decision's.
        if (!_WireKernelPrefix(p, pts->size())) {
            return false;
        }
        const RigExecNurbsCurve rest{&p.restPoints, p.curveOrder,
                                     &p.curveKnots};
        const RigExecNurbsCurve posed{&p.auxPoints, p.curveOrder,
                                      &p.curveKnots};
        // A sparse zero-default envelope: evaluate only the named points.
        // RigExecRunRevisionKernel routes a wire here with its envelope
        // unapplied only when this packet shape is what it holds.
        if (RigExecWireTakesSparseEnvelope(p.weights)) {
            const RigExecWeightPacket &w = p.weights;
            std::shared_ptr<const RigExecWireBasis> basis;
            if (wireBasis) {
                basis = wireBasis->Get(p, w.indices, pts->size());
            } else {
                auto built = std::make_shared<RigExecWireBasis>();
                if (RigExecBuildWireBasis(
                        p.wireBindCoords.cdata(), p.wireBindCoords.size(),
                        pts->size(), w.indices, p.curveOrder, p.curveKnots,
                        p.restPoints.size(), p.dropoffDistance,
                        built.get())) {
                    basis = std::move(built);
                }
            }
            if (!basis) {
                return false;
            }
            return RigExecApplyWireBasis(pts, *basis, w.indices, w.values,
                                         p.restPoints, p.auxPoints);
        }
        const auto *restEvals=wireBasis ? wireBasis->restEvaluations.Get(
            rest,p.wireBindCoords.cdata(),p.wireBindCoords.size(),p.dropoffDistance) : nullptr;
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
                                      p.dropoffDistance,b,e,restEvals ? restEvals->data() : nullptr,
                                      restEvals ? restEvals->size() : 0)) {
                    good = false;
                }
            });
            ok = good;
        } else {
            ok = RigExecApplyWire(pts, rest, posed, p.wireBindCoords.cdata(),
                                  p.wireBindCoords.size(),
                                  p.dropoffDistance,0,pts->size(),restEvals ? restEvals->data() : nullptr,
                                  restEvals ? restEvals->size() : 0);
        }
        return ok;
    }
    case RigExecRevisionOp::External: {
        const RigExecMoverHandler *handler = p.externalHandler;
        if (!handler || !handler->applyExternal) return false;
        const size_t enteringCount = pts->size();
        std::vector<GfVec3f> candidate;
        std::vector<GfVec3f> *output = pts;
        if (!discardable) { candidate = *pts; output = &candidate; }
        if (!handler->applyExternal(p.externalData, output) ||
            output->size() != enteringCount) return false;
        for (const GfVec3f &point : *output) {
            if (!std::isfinite(point[0]) || !std::isfinite(point[1]) ||
                !std::isfinite(point[2])) return false;
        }
        if (!discardable) pts->swap(candidate);
        return true;
    }
    case RigExecRevisionOp::RecomputeNormals:
    case RigExecRevisionOp::RecomputeExtent:
        return RigExecApplyDerivedKernel(op, p, pts);
    case RigExecRevisionOp::SurfaceProjector:
    case RigExecRevisionOp::ShaderDials:
        // Matrix targets: evaluated by RigExecEvaluateProjectorTarget,
        // never through a point kernel.
        return false;
    }
    // No runtime dispatch beyond the frozen operation set: an unhandled op is
    // a build error, not a silently skipped revision.
    return false;
}

} // namespace

bool RigExecApplyRevisionKernel(RigExecRevisionOp op,
    const RigExecMoverParameters &p, std::vector<GfVec3f> *pts, bool useSimd,
    RigExecWireBasisCache *wireBasis, RigExecSurfaceKernelCache<GfVec3f,GfVec3d> *cache)
{
    return ApplyRevisionKernel(op,p,pts,useSimd,wireBasis,cache,false);
}

bool
RigExecRevisionTakesSeparateBlend(RigExecRevisionOp op,
                                  const RigExecWeightPacket &w)
{
    // Matrix, blendShape and the two derived recomputations resolve the
    // envelope inside their own arithmetic; blending their result again would
    // apply it twice.
    return !(op == RigExecRevisionOp::Matrix ||
             op == RigExecRevisionOp::BlendShape ||
             op == RigExecRevisionOp::RecomputeNormals ||
             op == RigExecRevisionOp::RecomputeExtent ||
             (op == RigExecRevisionOp::Wire &&
              RigExecWireTakesSparseEnvelope(w)));
}

RigExecRevisionAcceptance
RigExecRevisionKernelAcceptance(RigExecRevisionOp op,
                                const RigExecMoverParameters &p, size_t count)
{
    using Acceptance = RigExecRevisionAcceptance;
    if (!_PacketMatches(op, p)) {
        return Acceptance::Refuses;
    }
    switch (op) {
    case RigExecRevisionOp::Matrix:
        return _MatrixKernelAccepts(p, count, nullptr) ? Acceptance::Applies
                                                       : Acceptance::Refuses;
    case RigExecRevisionOp::BlendShape:
        if (!_BlendShapeKernelAccepts(p, count, nullptr)) {
            return Acceptance::Refuses;
        }
        // The surface-frame transport reads the entering points.
        return p.blendSurfaceFrame ? Acceptance::Deferred
                                   : Acceptance::Applies;
    case RigExecRevisionOp::Wire: {
        const Acceptance wire = _WireKernelAcceptance(p, count);
        if (wire == Acceptance::Refuses) {
            return Acceptance::Refuses;
        }
        // The "apply once" envelope resolves at the full count or the
        // revision fails, whatever the kernel answered.
        if (RigExecRevisionTakesSeparateBlend(op, p.weights) &&
            !RigExecEnvelopeIsFullStrength(p.weights) &&
            !p.weights.ResolvesAll(count)) {
            return Acceptance::Refuses;
        }
        return wire;
    }
    default:
        return Acceptance::Deferred;
    }
}

// One revision, envelope included: the packet check, the full-strength fast
// path, RigExecApplyRevisionKernel and the "apply once" blend against the
// preceding revision.
// ONE definition of "apply once", called by the mover-graph revision node and
// by the baked geometry loop. Two hand-written wrappers would have to agree
// about which operations blend and which fold the envelope into their own
// arithmetic, and the rigs that would show a disagreement are the ones no
// fixture happened to have.
namespace {
// \p source, when set, holds the \p sourceCount points entering the revision
// and \p pts receives the result out of place: the matrix kernel reads them
// where they are, and every other operation copies them into \p pts first,
// bit for bit, and then runs in place. The "apply once" blend reads the
// entering points from \p source rather than from a copy of them.
bool
RunRevisionKernel(RigExecRevisionOp op,
                         const RigExecMoverParameters &p,
                         std::vector<GfVec3f> *pts, bool useSimd,
                         RigExecWireBasisCache *wireBasis,
    RigExecSurfaceKernelCache<GfVec3f,GfVec3d> *cache, bool discardable,
    const std::vector<float> *envelope,
    const GfVec3f *source = nullptr, size_t sourceCount = 0)
{
    if (!_PacketMatches(op, p)) {
        return false;
    }
    if (source) {
        if (op == RigExecRevisionOp::Matrix &&
            RigExecEnvelopeIsFullStrength(p.weights)) {
            pts->resize(sourceCount);
            ApplyMatrixFullStrength(p, source, pts->data(), sourceCount,
                                    useSimd);
            return true;
        }
        pts->assign(source, source + sourceCount);
    }
    if (!RigExecRevisionTakesSeparateBlend(op, p.weights)) {
        return ApplyRevisionKernel(op, p, pts, useSimd, wireBasis,cache,discardable);
    }

    // A constant envelope at exactly full strength makes the blend below the
    // identity at every point, so the copy of the preceding revision, the
    // resolved envelope array and the blend loop are all dead. The predicate
    // lives in moverGraph.h next to the packet it reads; see
    // RigExecEnvelopeIsFullStrength.
    const bool fullStrengthEnvelope = RigExecEnvelopeIsFullStrength(p.weights);
    const size_t precedingSize = pts->size();
    std::vector<GfVec3f> preceding;
    if (!fullStrengthEnvelope && !source) {
        preceding = *pts;
    }
    if (!ApplyRevisionKernel(op, p, pts, useSimd, wireBasis,cache,discardable)) {
        return false;
    }
    if (pts->size() != precedingSize) {
        return false;
    }
    if (!fullStrengthEnvelope) {
        // \p envelope, when it covers these points, is this packet's already
        // resolved by the same ResolveAll (RevisionStatic); otherwise it
        // resolves here.
        std::vector<float> resolved;
        const float *weights =
            envelope && envelope->size() == pts->size() ? envelope->data()
                                                        : nullptr;
        if (!weights) {
            if (!p.weights.ResolveAll(pts->size(), &resolved)) {
                return false;
            }
            weights = resolved.data();
        }
        RigExecBlendEnvelopeAll(source ? source : preceding.data(), weights,
                                pts->size(), pts->data());
    }
    return true;
}

} // namespace

bool RigExecRunRevisionKernel(RigExecRevisionOp op,
    const RigExecMoverParameters &p, std::vector<GfVec3f> *pts, bool useSimd,
    RigExecWireBasisCache *wireBasis, RigExecSurfaceKernelCache<GfVec3f,GfVec3d> *cache)
{
    return RunRevisionKernel(op,p,pts,useSimd,wireBasis,cache,false,nullptr);
}

namespace geometryDetail {
bool RunDiscardableRevisionKernel(RigExecRevisionOp op,
    const RigExecMoverParameters &p, std::vector<GfVec3f> *pts, bool useSimd,
    RigExecWireBasisCache *wireBasis, RigExecSurfaceKernelCache<GfVec3f,GfVec3d> *cache,
    const std::vector<float> *envelope)
{
    return RunRevisionKernel(op,p,pts,useSimd,wireBasis,cache,true,envelope);
}
bool RunDiscardableRevisionKernel(RigExecRevisionOp op,
    const RigExecMoverParameters &p, const GfVec3f *in, size_t count,
    std::vector<GfVec3f> *out, bool useSimd, RigExecWireBasisCache *wireBasis,
    RigExecSurfaceKernelCache<GfVec3f,GfVec3d> *cache,
    const std::vector<float> *envelope)
{
    // An empty entering array may come with no storage at all.
    if (!in) {
        out->clear();
        return RunRevisionKernel(op,p,out,useSimd,wireBasis,cache,true,envelope);
    }
    return RunRevisionKernel(op,p,out,useSimd,wireBasis,cache,true,envelope,
                             in,count);
}
} // namespace geometryDetail

bool
RigExecRevisionBinding::operator==(const RigExecRevisionBinding &o) const
{
    return moverPath == o.moverPath && target == o.target &&
           transform == o.transform &&
           transformSpace == o.transformSpace &&
           carrySpace == o.carrySpace && influences == o.influences &&
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
           widths == o.widths &&
           shaderDials == o.shaderDials &&
           meshWorldInverse == o.meshWorldInverse &&
           blendInputs == o.blendInputs && blendSamples == o.blendSamples &&
           phases == o.phases && transformPhase == o.transformPhase &&
           externalSchema == o.externalSchema && externalCompileData == o.externalCompileData && externalStructure == o.externalStructure && externalInputs == o.externalInputs;
}

std::optional<RigExecRevisionOp>
RigExecRevisionOpForSchema(const TfToken &schemaType, const TfToken &curveMode)
{
    // Resolved by the mover's own TU (see movers/): most movers name a
    // fixed op, the curve mover resolves its authored mode, and anything
    // without a row revises no point chain.
    const RigExecMoverHandler *handler =
        RigExecFindMoverHandler(schemaType);
    if (!handler) {
        return std::nullopt;
    }
    return handler->resolveOp(curveMode);
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

namespace {

// Interned once, at load: some readers resolve a phase every frame, and
// interning a name takes the token registry's lock.
const TfToken _readPhaseField(RigExecReadPhaseMetadataName);

}  // namespace

bool
RigExecResolveReadPhase(
    const UsdObject &property,
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
    std::string authored;
    if (!property.GetMetadata(_readPhaseField, &authored) ||
        authored.empty()) {
        return true;
    }
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

namespace {

TfToken
_Token(const UsdPrim &prim, const TfToken &attr, const TfToken &fallback)
{
    TfToken value = fallback;
    if (const UsdAttribute a = prim.GetAttribute(attr)) {
        a.Get(&value);
    }
    return value;
}

// Every scalar mover input consults the generation's resolved property set
// before the authored stage. This is what lets a property-domain mover drive
// another mover's common envelope without a second evaluation model.
template<class T> T
_Read(const UsdPrim &prim, const TfToken &attr, T fallback,
       UsdTimeCode time, const RigExecResolvedInputs *resolved)
{
    T value = fallback;
    if (const UsdAttribute a = prim.GetAttribute(attr)) {
        if (resolved && resolved->GetAttribute(a, time, &value)) {
            return value;
        }
        a.Get(&value, time);
    }
    return value;
}

float _Float(const UsdPrim &prim, const TfToken &attr, float fallback,
             UsdTimeCode time, const RigExecResolvedInputs *resolved)
{
    return _Read<float>(prim, attr, fallback, time, resolved);
}

// A scalar mover input read through its connections; a .rigexec runtime
// evaluates it over the file's input slots as a path read row.
template<class T> T
_RecordedInput(const UsdPrim &prim, const TfToken &name, T fallback,
               UsdTimeCode time, const RigExecResolvedInputs *resolved)
{
    return _Read<T>(prim, name, fallback, time, resolved);
}

// Forward declaration: the matrix assembler below predates the hook
// helpers and consumes the same enabled contract through them.
bool
_Enabled(const UsdPrim &prim, UsdTimeCode time,
         const RigExecResolvedInputs *resolved);

// A structural token on the mover (rigExec:weightBlend, rigExec:pointFrame,
// rigExec:driverDeltaFrame), read off the authored stage. The runtime and
// the frozen replay have no stage to ask; they replay this read (the file
// holds it as a path read), and a read nobody holds replays as the
// fallback. Guarded on the prim because the frozen replay calls the
// assemblers with none.
TfToken
_RecordedToken(const UsdPrim &prim, const TfToken &name,
               const TfToken &fallback, UsdTimeCode time)
{
    TfToken value = fallback;
    if (!prim) {
        return value;
    }
    if (const UsdAttribute a = prim.GetAttribute(name)) {
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

    // Per-mover inputs, bound by the mover's own TU (see movers/). A
    // type with no row, or a row with no binder, binds nothing here.
    if (const RigExecMoverHandler *handler =
            RigExecFindMoverHandler(moverPrim.GetTypeName())) {
        binding.handler = handler;
        binding.externalSchema = moverPrim.GetTypeName();
        if (handler->bind) {
            const SdfPath ownerPath = target.GetPrimPath();
            const RigExecMoverBindContext ctx{
                moverPrim, target, ownerPath, frameChainHeads, &binding};
            handler->bind(ctx);
        }
        if (handler->declareExternalInputs) {
            const RigExecMoverBindContext ctx{
                moverPrim, target, target.GetPrimPath(), frameChainHeads, &binding};
            handler->declareExternalInputs(ctx, &binding.externalInputs);
        }
    }

    // Every point-chain mover may narrow its application with a weight object.
    const SdfPathVector weights = RigExecRelationshipTargets(
        moverPrim, "rigExec:weightObject");
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

    // Read in the assembly every path shares, not only in the exec builder:
    // the baked program fills these parameters through this function.
    params.radialWeight =
        _RecordedToken(moverPrim, _attrTokens->weightBlend, TfToken(), time) ==
        _valueTokens->radial;
    // The USD paths take rigExec:pointFrame from compile (it selects
    // RigExecClusterInPointFrame in the fold); it is read here too, and the
    // runtime's fold replays that read to make the same choice.
    _RecordedToken(moverPrim, _attrTokens->pointFrame, TfToken(), time);

    params.enabled = _Enabled(moverPrim, time, resolved);
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
    const RigExecMoverParameters &parameters, const std::string &moverText)
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
        status.firstBadAddress = moverText;
    }
    return status;
}

RigExecMoverStatus
RigExecStatusForParameters(
    const RigExecMoverParameters &parameters, const SdfPath &moverPath)
{
    // Spelled only on the failure arm.
    if (parameters.enabled && !parameters.valid) {
        return RigExecStatusForParameters(parameters, moverPath.GetString());
    }
    return RigExecStatusForParameters(parameters, std::string());
}

namespace {

// An upstream array at \p path: authored-level, so it answers a read that
// bypasses the resolved inputs (rest data read at Default).
template <typename A>
bool
_StageUpstreamArray(const std::map<SdfPath, VtValue> *upstream,
                    const SdfPath &path, A *value)
{
    if (!upstream || path.IsEmpty()) {
        return false;
    }
    const auto it = upstream->find(path);
    if (it == upstream->end() || !it->second.IsHolding<A>()) {
        return false;
    }
    *value = it->second.UncheckedGet<A>();
    return true;
}

// Reads a typed array from an exact property path on the mover's stage.
// \p resolved is null for BIND-TIME reads and only for those. A rest cage, a
// rest geometry, a bind-time topology: those are the authored neutral pose the
// deformation is measured against, so a read phase has nothing to say about
// them -- and serving one the same phased value as the live read makes the two
// operands equal and the whole deformation an identity. \p upstream, when
// given, answers before the stage at any time.
template <typename T>
std::vector<T>
_Array(const UsdPrim &moverPrim, const SdfPath &path, UsdTimeCode time,
       const RigExecResolvedInputs *resolved = nullptr,
       const std::map<SdfPath, VtValue> *upstream = nullptr)
{
    std::vector<T> out;
    if (path.IsEmpty() || !moverPrim) {
        return out;
    }
    VtArray<T> value;
    if ((resolved && resolved->Get(path, &value)) ||
        _StageUpstreamArray(upstream, path, &value)) {
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
    const UsdAttribute a =
        prim ? prim.GetAttribute(_attrTokens->enabled) : UsdAttribute();
    if (a) {
        if (resolved && resolved->GetAttribute(a, time, &enabled)) {
            return enabled;
        }
        a.Get(&enabled, time);
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
    RIGEXEC_PURITY_CHECK();
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
            if (!RigExecSkinLayoutIsFixed(moverPrim)) {
                return false;
            }
            // This build fills the epoch cache (a varying layout is
            // refused above), so these reads are epoch state the file's
            // skin topology carries.
            const std::vector<int> indices =
                _Array<int>(moverPrim, indicesPath, time, resolved);
            const std::vector<float> weights =
                _Array<float>(moverPrim, weightsPath, time, resolved);
            int elementSize = 1;
            if (const UsdAttribute a =
                    moverPrim.GetAttribute(_attrTokens->elementSize)) {
                if (!resolved ||
                    !resolved->GetAttribute(a, time, &elementSize)) {
                    a.Get(&elementSize, time);
                }
            }
            RigExecBuildSkinTopology(indices, weights, elementSize,
                                     influenceCount, topology);
            return true;
        });
}

namespace {

// Whether none of \p names on \p moverPrim might vary with time or has an
// authored connection; false for an invalid prim.
bool
_SkinLayoutAttributesAreFixed(const UsdPrim &moverPrim,
                              std::initializer_list<TfToken> names)
{
    if (!moverPrim) {
        return false;
    }
    for (const TfToken &name : names) {
        const UsdAttribute a = moverPrim.GetAttribute(name);
        if (a && (a.ValueMightBeTimeVarying() || a.HasAuthoredConnections())) {
            return false;
        }
    }
    return true;
}

}  // namespace

bool
RigExecSkinLayoutIsFixed(const UsdPrim &moverPrim)
{
    return _SkinLayoutAttributesAreFixed(
        moverPrim, {_attrTokens->jointIndices, _attrTokens->jointWeights,
                    _attrTokens->elementSize});
}

bool
RigExecSkinLayoutTopologyIsFixed(const UsdPrim &moverPrim)
{
    return _SkinLayoutAttributesAreFixed(
        moverPrim, {_attrTokens->jointIndices, _attrTokens->elementSize});
}

bool
RigExecSkinLayoutWeightsAreFixed(const UsdPrim &moverPrim)
{
    return _SkinLayoutAttributesAreFixed(moverPrim,
                                         {_attrTokens->jointWeights});
}

void
RigExecBuildSkinTopology(TfSpan<const int> indices,
                         TfSpan<const float> weights, int elementSize,
                         size_t influenceCount, RigExecSkinTopology *topology)
{
    topology->indices.assign(indices.begin(), indices.end());
    topology->weights.assign(weights.begin(), weights.end());
    topology->elementSize = elementSize;
    topology->influenceCount = influenceCount;
    if (topology->elementSize < 1 ||
        topology->weights.size() != topology->indices.size() ||
        topology->indices.size() % size_t(topology->elementSize) != 0) {
        return;
    }
    topology->pointCount =
        topology->indices.size() / size_t(topology->elementSize);
    // Exactly RigExecSkinLayout::Validate's shape, range and weight rules,
    // against a table whose SIZE is epoch state. The matrices themselves are
    // checked every frame by the caller.
    if (topology->influenceCount == 0) {
        return;
    }
    for (size_t i = 0; i < topology->indices.size(); ++i) {
        const int index = topology->indices[i];
        if (index < 0 || size_t(index) >= topology->influenceCount) {
            return;
        }
        const float weight = topology->weights[i];
        if (!std::isfinite(weight) || weight < 0.0f) {
            return;
        }
    }
    topology->validated = true;
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
            if (!resolved || !resolved->GetAttribute(a, time, out)) {
                a.Get(out, time);
            }
        }
    };
    params.skinningMethod = _valueTokens->classicLinear;
    if (const UsdAttribute a =
            moverPrim.GetAttribute(_attrTokens->skinningMethod)) {
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

    // The cache refused this mover, so the layout reads per frame.
    params.skinIndices = _Array<int>(moverPrim, indicesPath, time, resolved);
    params.skinWeights = _Array<float>(moverPrim, weightsPath, time,
                                       resolved);
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
// The point of the sparse form: a real corrective moves 1,279 of 26,276
// points, so the indexed loop touches 4.87% of what the dense one does. The
// dense branch here exists for a sample that carries a layout with empty
// indices (offsets parallel to the base points), which is what an authored
// UsdSkelBlendShape with no pointIndices means, and for the dense-points
// endpoint of a mixed pair.
static void
_AccumulateBlendSample(const RigExecBlendSampleData &sample,
                       const GfVec3f *base, size_t baseCount,
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
    for (size_t i = 0; i < baseCount; ++i) {
        (*deltas)[i] += (sample.PointData()[i] - base[i]) * scale;
    }
}

bool
RigExecSumBlendChannels(
    const std::vector<RigExecBlendChannel> &channels,
    const GfVec3f *base, size_t baseCount,
    std::vector<GfVec3f> *deltas)
{
    if (!deltas || !base || !baseCount) {
        return false;
    }
    deltas->assign(baseCount, GfVec3f(0));

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
                       sample.layout->pointCount == baseCount &&
                       sample.PointCount() == 0)
                    : (sample.PointCount() == baseCount && sample.PointData());
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
            const GfVec3f *lo = loSample ? loSample->PointData() : nullptr;
            const GfVec3f *hiPts = hiSample.PointData();
            // Each point is independent and every point's contribution keeps
            // the original channel-order accumulation, so spreading the point
            // range across workers is bit-identical to the serial loop. The
            // per-channel setup above (weights, hi/lo sample pick, t) is cheap
            // and stays sequential; only the O(points) inner loop parallelises.
            const GfVec3f *baseData = base;
            const GfVec3f *hiData = hiPts;
            const GfVec3f *loData = lo;
            GfVec3f *deltaData = deltas->data();
            const size_t nPts = baseCount;
            auto denseRange = [&](size_t begin, size_t end) {
                for (size_t i = begin; i < end; ++i) {
                    const GfVec3f dHi = hiData[i] - baseData[i];
                    const GfVec3f dLo = loData ? (loData[i] - baseData[i])
                                               : GfVec3f(0);
                    deltaData[i] += dLo + (dHi - dLo) * t;
                }
            };
            // Gated like every other launch site in this file: the frozen
            // assembler calls this kernel too (frozenContext.cpp's
            // blend-shape replica), and a frozen frame must run on its own
            // thread alone. The serial branch computes the same bits, which
            // is what makes the gate free to take.
            if (RigExecParallelEvaluationEnabled() &&
                !RigExecFrozenSerialActive() &&
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
            _AccumulateBlendSample(*loSample, base, baseCount, 1.0f - t, deltas);
        }
        _AccumulateBlendSample(hiSample, base, baseCount, t, deltas);
    }
    return true;
}

bool RigExecSumBlendChannels(
    const std::vector<RigExecBlendChannel> &channels,
    const std::vector<GfVec3f> &base,std::vector<GfVec3f> *deltas)
{
    return RigExecSumBlendChannels(channels,base.data(),base.size(),deltas);
}

RigExecMoverParameters
RigExecAssembleParameters(
    const UsdPrim &moverPrim,
    RigExecRevisionOp op,
    const RigExecRevisionBinding &binding,
    const RigExecProviderValues &values,
    UsdTimeCode time)
{
    RIGEXEC_PURITY_CHECK();
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
        params.kind = _kindTokens->blendShape;
        break;
    case RigExecRevisionOp::VolumeCorrect:
        params.kind = _kindTokens->volumeCorrect;
        break;
    case RigExecRevisionOp::Smooth:
        params.kind = _kindTokens->smooth;
        break;
    case RigExecRevisionOp::DeltaMush:
        params.kind = _kindTokens->deltaMush;
        break;
    case RigExecRevisionOp::Wrinkle:
        params.kind = _kindTokens->wrinkle;
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
    case RigExecRevisionOp::External:
        params.kind = _kindTokens->external;
        break;
    case RigExecRevisionOp::RecomputeNormals:
        params.kind = _kindTokens->recomputeNormals;
        break;
    case RigExecRevisionOp::RecomputeExtent:
        params.kind = _kindTokens->recomputeExtent;
        break;
    case RigExecRevisionOp::SurfaceProjector:
        params.kind = _kindTokens->surfaceProjector;
        break;
    case RigExecRevisionOp::ShaderDials:
        params.kind = _kindTokens->shaderDials;
        break;
    case RigExecRevisionOp::Matrix:
    case RigExecRevisionOp::Skin:
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

    case RigExecRevisionOp::DeltaMush:
        params.restPoints = _Array<GfVec3f>(moverPrim, moverPrim.GetPath().AppendProperty(_attrTokens->restPoints), UsdTimeCode::Default(), values.resolved);
        if (params.restPoints.empty()) params.restPoints = values.basePoints;
        params.topologyCounts = _Array<int>(moverPrim, binding.topologyCounts, time, values.resolved);
        params.topologyIndices = _Array<int>(moverPrim, binding.topologyIndices, time, values.resolved);
        params.mushIterations = _RecordedInput<int>(moverPrim, _attrTokens->iterations, 10, time, values.resolved);
        params.mushStep = _RecordedInput<float>(moverPrim, _attrTokens->step, 0.5f, time, values.resolved);
        params.mushPinBorders = _RecordedInput<bool>(moverPrim, _attrTokens->pinBorders, true, time, values.resolved);
        params.mushDistanceWeight = _RecordedInput<float>(moverPrim, _attrTokens->distanceWeight, 0.0f, time, values.resolved);
        params.mushDisplacement = _RecordedInput<float>(moverPrim, _attrTokens->displacement, 1.0f, time, values.resolved);
        params.valid = !params.restPoints.empty() && !params.topologyCounts.empty();
        break;
    case RigExecRevisionOp::Wrinkle: {
        params.restPoints = _Array<GfVec3f>(moverPrim,
            moverPrim.GetPath().AppendProperty(_attrTokens->restPoints),
            UsdTimeCode::Default(), values.resolved);
        if (params.restPoints.empty()) params.restPoints = values.basePoints;
        params.topologyCounts = _Array<int>(moverPrim, binding.topologyCounts,
            time, values.resolved);
        params.topologyIndices = _Array<int>(moverPrim, binding.topologyIndices,
            time, values.resolved);
        auto &settings = params.wrinkleSettings;
        const TfToken topology = _RecordedInput<TfToken>(moverPrim,
            _attrTokens->topology, _valueTokens->cloth,
            UsdTimeCode::Default(), values.resolved);
        if (topology != _valueTokens->cloth && topology != _valueTokens->surfaceStruts) break;
        settings.topology = topology == _valueTokens->cloth
            ? RigExecWrinkleTopology::Cloth : RigExecWrinkleTopology::SurfaceStruts;
        settings.pinPoints = _Array<int>(moverPrim,
            moverPrim.GetPath().AppendProperty(_attrTokens->pinPoints),
            UsdTimeCode::Default(), values.resolved);
        settings.iterations = _RecordedInput<int>(moverPrim,
            _attrTokens->iterations, 80, time, values.resolved);
        settings.neighborDistance = _RecordedInput<int>(moverPrim,
            _attrTokens->neighborDistance, 2, time, values.resolved);
        settings.restLengthScale = _RecordedInput<float>(moverPrim,
            _attrTokens->restLengthScale, 1.0f, time, values.resolved);
        settings.stretchStiffness = _RecordedInput<float>(moverPrim,
            _attrTokens->stretchStiffness, 1.0f, time, values.resolved);
        settings.compressionStiffness = _RecordedInput<float>(moverPrim,
            _attrTokens->compressionStiffness, 1.0f, time, values.resolved);
        settings.bendStiffness = _RecordedInput<float>(moverPrim,
            _attrTokens->bendStiffness, 0.1f, time, values.resolved);
        settings.maxDisplacement = _RecordedInput<float>(moverPrim,
            _attrTokens->maxDisplacement, 0.2f, time, values.resolved);
        settings.pinBorders = _RecordedInput<bool>(moverPrim,
            _attrTokens->pinBorders, true, time, values.resolved);
        settings.tangentPlaneCollisions = _RecordedInput<bool>(moverPrim,
            _attrTokens->tangentPlaneCollisions, true, time, values.resolved);
        settings.tangentPlaneInset = _RecordedInput<float>(moverPrim,
            _attrTokens->tangentPlaneInset, 0.0f, time, values.resolved);
        settings.wrinkleScale = _RecordedInput<float>(moverPrim,
            _attrTokens->wrinkleScale, 1.0f, time, values.resolved);
        settings.smoothingIterations = _RecordedInput<int>(moverPrim,
            _attrTokens->smoothingIterations, 0, time, values.resolved);
        params.valid = !params.restPoints.empty() && !params.topologyCounts.empty();
        break;
    }
    case RigExecRevisionOp::External: {
        RigExecExternalPayload payload;
        RigExecAssembleExternalPayload(moverPrim, binding, values, time,
                                       &payload);
        params.externalHandler = payload.handler;
        params.externalSchema = payload.schema;
        params.valid = payload.valid;
        params.externalData = std::move(payload.data);
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
        params.auxPoints = _Array<GfVec3f>(moverPrim, binding.cagePoints, UsdTimeCode::Default(), /*resolved=*/nullptr, values.upstream);
        params.auxPointsB = _Array<GfVec3f>(moverPrim, binding.cagePoints, time, values.resolved);
        if (const UsdAttribute a =
                moverPrim.GetAttribute(_attrTokens->divisions)) {
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

    case RigExecRevisionOp::Wire: {
        // Posed control points at the declared phase; rest control points
        // are the curve's AUTHORED ones, which is what the bind coordinates
        // were computed against.
        params.restPoints = _Array<GfVec3f>(moverPrim,
            binding.driverCurvePoints, UsdTimeCode::Default(),
            /*resolved=*/nullptr, values.upstream);
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
            const auto floats = [&](const TfToken &name) {
                VtFloatArray out;
                if (const UsdAttribute a = moverPrim.GetAttribute(name)) {
                    if (!values.resolved ||
                        !values.resolved->GetAttribute(a, time, &out)) {
                        a.Get(&out, time);
                    }
                }
                return out;
            };
            const VtFloatArray weights = floats(_attrTokens->driverWeights);
            const VtFloatArray baseWeights =
                floats(_attrTokens->driverBaseWeights);
            // Which frame the wire's points are already in, and which frame
            // the driver's offset is applied in. The runtime and the frozen
            // replay read the same tokens, so they make the same choice.
            RigExecWireDriverFrame frame;
            frame.posedPoints =
                _RecordedToken(moverPrim, _attrTokens->pointFrame,
                               _valueTokens->rest, time) == "posed";
            frame.posedDelta =
                _RecordedToken(moverPrim, _attrTokens->driverDeltaFrame,
                               _valueTokens->local, time) == "posed";
            frame.carry = values.carry;
            RigExecPoseWireDrivers(*table, t, s, bt, weights, baseWeights,
                                   frame, &params.restPoints,
                                   &params.auxPoints);
        } else {
            params.auxPoints = _Array<GfVec3f>(
                moverPrim, binding.driverCurvePoints, time, values.resolved);
        }
        {
            VtIntArray order;
            if (_StageUpstreamArray(values.upstream, binding.driverCurveOrder,
                                    &order)) {
                if (!order.empty()) {
                    params.curveOrder = order[0];
                }
            } else if (const UsdAttribute a =
                           moverPrim.GetStage()->GetAttributeAtPath(
                               binding.driverCurveOrder)) {
                if (a.Get(&order, UsdTimeCode::Default()) && !order.empty()) {
                    params.curveOrder = order[0];
                }
            }
        }
        params.curveKnots = _Array<double>(moverPrim,
            binding.driverCurveKnots, UsdTimeCode::Default(),
            /*resolved=*/nullptr, values.upstream);
        if (const UsdAttribute a =
                moverPrim.GetAttribute(_attrTokens->dropoffDistance)) {
            float dropoff = 0.0f;
            a.Get(&dropoff, time);
            params.dropoffDistance = dropoff;
        }
        if (!binding.bindCoords.IsEmpty()) {
            if (!values.resolved ||
                !values.resolved->Get(binding.bindCoords,
                                      &params.wireBindCoords)) {
                if (const UsdAttribute a =
                        moverPrim.GetStage()->GetAttributeAtPath(
                            binding.bindCoords)) {
                    a.Get(&params.wireBindCoords, time);
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
        break;
    }

    return params;
}

void
RigExecAssembleExternalPayload(const UsdPrim &moverPrim,
                               const RigExecRevisionBinding &binding,
                               const RigExecProviderValues &values,
                               UsdTimeCode time,
                               RigExecExternalPayload *payload)
{
    payload->schema = moverPrim.GetTypeName();
    payload->handler = binding.handler;
    std::vector<VtValue> inputs;
    inputs.reserve(binding.externalInputs.size());
    for (const auto &key : binding.externalInputs) {
        inputs.push_back(RigExecSampleRevisionLeaf(
            key, moverPrim.GetStage()->GetAttributeAtPath(key.path),
            values.resolved, time, values.upstream));
    }
    const RigExecExternalProviderValues pureValues(values);
    const RigExecExternalInputContext ctx{binding, pureValues, inputs};
    payload->valid = payload->handler && payload->handler->assembleExternal &&
                     payload->handler->assembleExternal(ctx, &payload->data);
}

namespace {

// The iterative deformers' scalar inputs, in the stage assembler's order,
// each with its fallback and the packet field it fills.
template <class Fn>
void
_IterativeScalars(RigExecRevisionOp op, RigExecMoverParameters &params,
                  Fn &&fn)
{
    if (op == RigExecRevisionOp::DeltaMush) {
        fn(_attrTokens->iterations, 10, params.mushIterations);
        fn(_attrTokens->step, 0.5f, params.mushStep);
        fn(_attrTokens->pinBorders, true, params.mushPinBorders);
        fn(_attrTokens->distanceWeight, 0.0f, params.mushDistanceWeight);
        fn(_attrTokens->displacement, 1.0f, params.mushDisplacement);
    } else if (op == RigExecRevisionOp::Wrinkle) {
        RigExecWrinkleSettings &s = params.wrinkleSettings;
        fn(_attrTokens->iterations, 80, s.iterations);
        fn(_attrTokens->neighborDistance, 2, s.neighborDistance);
        fn(_attrTokens->restLengthScale, 1.0f, s.restLengthScale);
        fn(_attrTokens->stretchStiffness, 1.0f, s.stretchStiffness);
        fn(_attrTokens->compressionStiffness, 1.0f, s.compressionStiffness);
        fn(_attrTokens->bendStiffness, 0.1f, s.bendStiffness);
        fn(_attrTokens->maxDisplacement, 0.2f, s.maxDisplacement);
        fn(_attrTokens->pinBorders, true, s.pinBorders);
        fn(_attrTokens->tangentPlaneCollisions, true,
           s.tangentPlaneCollisions);
        fn(_attrTokens->tangentPlaneInset, 0.0f, s.tangentPlaneInset);
        fn(_attrTokens->wrinkleScale, 1.0f, s.wrinkleScale);
        fn(_attrTokens->smoothingIterations, 0, s.smoothingIterations);
    }
}

template <class T>
constexpr RigExecRevisionLeafType
_ScalarLeafType()
{
    if constexpr (std::is_same_v<T, bool>) {
        return RigExecRevisionLeafType::Bool;
    } else if constexpr (std::is_same_v<T, int>) {
        return RigExecRevisionLeafType::Int;
    } else {
        static_assert(std::is_same_v<T, float>, "an iterative scalar type");
        return RigExecRevisionLeafType::Float;
    }
}

}  // namespace

bool
RigExecRevisionOpAssemblesFromLeaves(RigExecRevisionOp op)
{
    switch (op) {
    case RigExecRevisionOp::Matrix:
    case RigExecRevisionOp::Skin:
    case RigExecRevisionOp::BlendShape:
    case RigExecRevisionOp::VolumeCorrect:
    case RigExecRevisionOp::Smooth:
    case RigExecRevisionOp::SurfaceProject:
    case RigExecRevisionOp::Ribbon:
    case RigExecRevisionOp::EmitGuidePoints:
    case RigExecRevisionOp::RecomputeNormals:
    case RigExecRevisionOp::RecomputeExtent:
    case RigExecRevisionOp::DeltaMush:
    case RigExecRevisionOp::Wrinkle:
    case RigExecRevisionOp::Lattice:
    case RigExecRevisionOp::Wire:
    case RigExecRevisionOp::External:
    case RigExecRevisionOp::SurfaceProjector:
    case RigExecRevisionOp::ShaderDials:
        return true;
    }
    return false;
}

bool
RigExecRevisionLeafRoleIsTopology(RigExecRevisionLeafRole role)
{
    switch (role) {
    case RigExecRevisionLeafRole::ElementSize:
    case RigExecRevisionLeafRole::JointIndices:
    case RigExecRevisionLeafRole::TopologyCounts:
    case RigExecRevisionLeafRole::TopologyIndices:
    case RigExecRevisionLeafRole::Divisions:
    case RigExecRevisionLeafRole::CurveOrder:
    case RigExecRevisionLeafRole::CurveKnots:
        return true;
    default:
        return false;
    }
}

void
RigExecDeclareRevisionLeaves(RigExecRevisionOp op, const SdfPath &moverPath,
                             const RigExecRevisionBinding &binding,
                             RigExecRevisionLeafDecl *decl)
{
    using Role = RigExecRevisionLeafRole;
    using Type = RigExecRevisionLeafType;
    using Time = RigExecRevisionLeafTime;
    using Flavour = RigExecRevisionLeafFlavour;
    decl->assembles = RigExecRevisionOpAssemblesFromLeaves(op);
    if (!decl->assembles) {
        return;
    }
    const auto add = [decl](Role role, const SdfPath &path, Type type,
                            Time time, Flavour flavour, VtValue fallback) {
        if (path.IsEmpty()) {
            return;  // _Array's empty path: the read answers an empty array
        }
        decl->roles[size_t(role)] =
            decl->Add({path, type, time, flavour, std::move(fallback)});
    };
    const auto mover = [&moverPath](const TfToken &name) {
        return moverPath.AppendProperty(name);
    };
    // Every arm's reads in its own order. A read only some branch takes is
    // declared regardless: the sample is a superset, the assembly reads the
    // leaf exactly where the stage assembler reads the stage.
    const auto envelope = [&]() {
        add(Role::Enabled, mover(_attrTokens->enabled), Type::Bool,
            Time::AtTime, Flavour::Resolved, VtValue(true));
        add(Role::DefaultWeight, mover(_attrTokens->defaultWeight),
            Type::Float, Time::AtTime, Flavour::Resolved, VtValue(1.0f));
    };
    const auto array = [&](Role role, const SdfPath &path, Type type,
                           VtValue empty) {
        add(role, path, type, Time::AtTime, Flavour::OverlayThenRaw,
            std::move(empty));
    };
    const auto topology = [&]() {
        array(Role::TopologyCounts, binding.topologyCounts, Type::IntArray,
              VtValue(VtIntArray()));
        array(Role::TopologyIndices, binding.topologyIndices, Type::IntArray,
              VtValue(VtIntArray()));
    };
    switch (op) {
    case RigExecRevisionOp::Matrix:
        // rigExec:pointFrame is read and discarded, as the stage assembler
        // reads it: the runtime's fold replays the read.
        add(Role::WeightBlend, mover(_attrTokens->weightBlend),
            Type::Token, Time::AtTime, Flavour::Raw, VtValue(TfToken()));
        add(Role::PointFrame, mover(_attrTokens->pointFrame),
            Type::Token, Time::AtTime, Flavour::Raw, VtValue(TfToken()));
        envelope();
        break;
    case RigExecRevisionOp::Skin:
        envelope();
        add(Role::SkinningMethod, mover(_attrTokens->skinningMethod),
            Type::Token, Time::AtTime, Flavour::Resolved,
            VtValue(_valueTokens->classicLinear));
        array(Role::JointIndices, mover(_attrTokens->jointIndices),
              Type::IntArray, VtValue(VtIntArray()));
        array(Role::JointWeights, mover(_attrTokens->jointWeights),
              Type::FloatArray, VtValue(VtFloatArray()));
        add(Role::ElementSize, mover(_attrTokens->elementSize), Type::Int,
            Time::AtTime, Flavour::Resolved, VtValue(1));
        break;
    case RigExecRevisionOp::RecomputeNormals:
    case RigExecRevisionOp::RecomputeExtent:
        // Synthesized maintenance: no enable and no envelope read.
        topology();
        if (op == RigExecRevisionOp::RecomputeExtent) {
            array(Role::Widths, binding.widths, Type::FloatArray,
                  VtValue(VtFloatArray()));
        }
        break;
    case RigExecRevisionOp::BlendShape:
        envelope();
        add(Role::DeltaSpace, mover(_attrTokens->deltaSpace), Type::Token,
            Time::AtDefault, Flavour::Raw, VtValue(_valueTokens->target));
        topology();
        break;
    case RigExecRevisionOp::Smooth:
        envelope();
        topology();
        break;
    case RigExecRevisionOp::SurfaceProject:
        envelope();
        array(Role::SurfacePoints, binding.surfacePoints, Type::Vec3fArray,
              VtValue(VtVec3fArray()));
        topology();
        break;
    case RigExecRevisionOp::Ribbon:
        envelope();
        array(Role::BindCoords, binding.bindCoords, Type::Vec2fArray,
              VtValue(VtArray<GfVec2f>()));
        break;
    case RigExecRevisionOp::VolumeCorrect:
    case RigExecRevisionOp::EmitGuidePoints:
        envelope();
        break;
    case RigExecRevisionOp::External:
        envelope();
        decl->externalBegin = int(decl->keys.size());
        for (const auto &key : binding.externalInputs) decl->Add(key);
        break;
    case RigExecRevisionOp::DeltaMush:
    case RigExecRevisionOp::Wrinkle: {
        envelope();
        add(Role::RestPoints, mover(_attrTokens->restPoints),
            Type::Vec3fArray, Time::AtDefault, Flavour::OverlayThenRaw,
            VtValue(VtVec3fArray()));
        topology();
        if (op == RigExecRevisionOp::Wrinkle) {
            add(Role::WrinkleTopology, mover(_attrTokens->topology),
                Type::Token, Time::AtDefault, Flavour::Resolved,
                VtValue(_valueTokens->cloth));
            add(Role::PinPoints, mover(_attrTokens->pinPoints),
                Type::IntArray, Time::AtDefault, Flavour::OverlayThenRaw,
                VtValue(VtIntArray()));
        }
        decl->scalarBegin = int(decl->keys.size());
        RigExecMoverParameters fields;
        _IterativeScalars(op, fields,
                          [&](const TfToken &name, auto fallback, auto &) {
                              using T = decltype(fallback);
                              decl->Add({mover(name), _ScalarLeafType<T>(),
                                         Time::AtTime, Flavour::Resolved,
                                         VtValue(fallback)});
                          });
        break;
    }
    case RigExecRevisionOp::Lattice:
        envelope();
        // The bind-time cage reads past the resolved inputs (and the phase
        // overlay); the live cage through them.
        add(Role::RestCage, binding.cagePoints, Type::Vec3fArray,
            Time::AtDefault, Flavour::Raw, VtValue(VtVec3fArray()));
        array(Role::LiveCage, binding.cagePoints, Type::Vec3fArray,
              VtValue(VtVec3fArray()));
        add(Role::Divisions, mover(_attrTokens->divisions), Type::Vec3i,
            Time::AtTime, Flavour::Raw,
            VtValue(RigExecMoverParameters().divisions));
        break;
    case RigExecRevisionOp::Wire:
        envelope();
        add(Role::CurveRest, binding.driverCurvePoints, Type::Vec3fArray,
            Time::AtDefault, Flavour::Raw, VtValue(VtVec3fArray()));
        add(Role::DriverWeights, mover(_attrTokens->driverWeights),
            Type::FloatArray, Time::AtTime, Flavour::Resolved,
            VtValue(VtFloatArray()));
        add(Role::DriverBaseWeights, mover(_attrTokens->driverBaseWeights),
            Type::FloatArray, Time::AtTime, Flavour::Resolved,
            VtValue(VtFloatArray()));
        add(Role::PointFrame, mover(_attrTokens->pointFrame), Type::Token,
            Time::AtTime, Flavour::Raw, VtValue(_valueTokens->rest));
        add(Role::DeltaFrame, mover(_attrTokens->driverDeltaFrame),
            Type::Token, Time::AtTime, Flavour::Raw,
            VtValue(_valueTokens->local));
        array(Role::CurveLive, binding.driverCurvePoints, Type::Vec3fArray,
              VtValue(VtVec3fArray()));
        add(Role::CurveOrder, binding.driverCurveOrder, Type::IntArray,
            Time::AtDefault, Flavour::Raw, VtValue(VtIntArray()));
        add(Role::CurveKnots, binding.driverCurveKnots, Type::DoubleArray,
            Time::AtDefault, Flavour::Raw, VtValue(VtDoubleArray()));
        // Read only where the attribute stands: an empty fallback.
        add(Role::Dropoff, mover(_attrTokens->dropoffDistance), Type::Float,
            Time::AtTime, Flavour::Raw, VtValue());
        array(Role::BindCoords, binding.bindCoords, Type::Vec2fArray,
              VtValue(VtArray<GfVec2f>()));
        break;
    case RigExecRevisionOp::SurfaceProjector:
        // RigExecReadProjectorTarget's reads, through the generation's
        // resolved inputs alone (a projector declares no phase).
        add(Role::RayOrigin, mover(_attrTokens->rayOrigin), Type::Vec3d,
            Time::AtTime, Flavour::Resolved,
            VtValue(GfVec3d(0.0, 0.0, 0.0)));
        add(Role::RayDirection, mover(_attrTokens->rayDirection),
            Type::Vec3d, Time::AtTime, Flavour::Resolved,
            VtValue(GfVec3d(0.0, 0.0, 1.0)));
        add(Role::RayUp, mover(_attrTokens->rayUp), Type::Vec3d,
            Time::AtTime, Flavour::Resolved,
            VtValue(GfVec3d(0.0, 1.0, 0.0)));
        add(Role::ShaderOffset, mover(_attrTokens->shaderOffset),
            Type::Matrix4d, Time::AtTime, Flavour::Resolved,
            VtValue(GfMatrix4d(1.0)));
        add(Role::ProjectionMode, mover(_attrTokens->projectionMode),
            Type::Token, Time::AtTime, Flavour::Raw,
            VtValue(_valueTokens->material));
        topology();
        break;
    case RigExecRevisionOp::ShaderDials:
        // One dial per source, in order (compile admits property paths
        // only; a missing attribute answers 0).
        decl->dialBegin = int(decl->keys.size());
        for (const SdfPath &dial : binding.shaderDials) {
            decl->Add({dial, Type::Dial, Time::AtTime, Flavour::Resolved,
                       VtValue(0.0)});
        }
        break;
    }
}

void
RigExecDeclareSkinLayoutLeaves(const SdfPath &moverPath,
                               RigExecRevisionLeafDecl *decl)
{
    using Role = RigExecRevisionLeafRole;
    using Type = RigExecRevisionLeafType;
    using Time = RigExecRevisionLeafTime;
    using Flavour = RigExecRevisionLeafFlavour;
    decl->roles[size_t(Role::JointIndices)] = decl->Add(
        {moverPath.AppendProperty(_attrTokens->jointIndices), Type::IntArray,
         Time::AtTime, Flavour::OverlayThenRaw, VtValue(VtIntArray())});
    decl->roles[size_t(Role::JointWeights)] = decl->Add(
        {moverPath.AppendProperty(_attrTokens->jointWeights),
         Type::FloatArray, Time::AtTime, Flavour::OverlayThenRaw,
         VtValue(VtFloatArray())});
    decl->roles[size_t(Role::ElementSize)] = decl->Add(
        {moverPath.AppendProperty(_attrTokens->elementSize), Type::Int,
         Time::AtTime, Flavour::Resolved, VtValue(1)});
}

namespace {

template <class T>
struct _IsVtArray : std::false_type {};
template <class E>
struct _IsVtArray<VtArray<E>> : std::true_type {};

// An upstream array at \p path in \p layer: authored-level, so it answers a
// raw read at any time, Default included, as a stage holding it would.
template <class T>
bool
_UpstreamArray(const std::map<SdfPath, VtValue> *layer, const SdfPath &path,
               T *value)
{
    if constexpr (_IsVtArray<T>::value) {
        if (layer && !layer->empty()) {
            const auto it = layer->find(path);
            if (it != layer->end() && it->second.IsHolding<T>()) {
                *value = it->second.UncheckedGet<T>();
                return true;
            }
        }
    }
    return false;
}

// One typed read, site for site: \p flavour over \p fallback. An array read
// of any flavour answers from the upstream \p layer before the stage.
template <class T>
T
_LeafRead(RigExecRevisionLeafFlavour flavour, const SdfPath &path,
          const UsdAttribute &a, const RigExecResolvedInputs *resolved,
          const std::map<SdfPath, VtValue> *layer, UsdTimeCode time, T value)
{
    switch (flavour) {
    case RigExecRevisionLeafFlavour::Raw:
        if (_UpstreamArray(layer, path, &value)) {
            return value;
        }
        if (a) {
            a.Get(&value, time);
        }
        return value;
    case RigExecRevisionLeafFlavour::Resolved:
        if (a) {
            if (resolved &&
                resolved->GetAttributeOverStageLayer(a, time, layer, &value)) {
                return value;
            }
            a.Get(&value, time);
        }
        return value;
    case RigExecRevisionLeafFlavour::ResolvedOnly:
        if (resolved) {
            resolved->GetAttributeOverStageLayer(a, time, layer, &value);
        }
        return value;
    case RigExecRevisionLeafFlavour::OverlayThenRaw:
        if (resolved && resolved->Get(path, &value)) {
            return value;
        }
        if (_UpstreamArray(layer, path, &value)) {
            return value;
        }
        if (a) {
            a.Get(&value, time);
        }
        return value;
    case RigExecRevisionLeafFlavour::Present:
        break;
    }
    return value;
}

template <class T>
VtValue
_LeafSample(const RigExecRevisionLeafKey &key, const UsdAttribute &a,
            const RigExecResolvedInputs *resolved,
            const std::map<SdfPath, VtValue> *layer, UsdTimeCode time)
{
    if (key.flavour == RigExecRevisionLeafFlavour::Raw && key.fallback.IsEmpty()) {
        T value;
        if (_UpstreamArray(layer, key.path, &value) || (a && a.Get(&value, time)))
            return VtValue(value); // A successful empty array is still present.
        return VtValue(); // Failed Get must not synthesize T{} presence.
    }
    const T fallback =
        key.fallback.IsHolding<T>() ? key.fallback.UncheckedGet<T>() : T();
    return VtValue(
        _LeafRead<T>(key.flavour, key.path, a, resolved, layer, time,
                     fallback));
}

}  // namespace

VtValue
RigExecSampleRevisionLeaf(const RigExecRevisionLeafKey &key,
                          const UsdAttribute &attribute,
                          const RigExecResolvedInputs *resolved,
                          UsdTimeCode time,
                          const std::map<SdfPath, VtValue> *upstream)
{
    if (key.flavour == RigExecRevisionLeafFlavour::Present) {
        return VtValue(resolved && resolved->Find(key.path) != nullptr);
    }
    const UsdTimeCode at = key.time == RigExecRevisionLeafTime::AtDefault
                               ? UsdTimeCode::Default()
                               : time;
    // A site that reads only an attribute that exists: nothing stands.
    if (key.flavour == RigExecRevisionLeafFlavour::Raw &&
        key.fallback.IsEmpty() && !attribute) {
        return VtValue();
    }
    switch (key.type) {
    case RigExecRevisionLeafType::Double:
        return _LeafSample<double>(key, attribute, resolved, upstream, at);
    case RigExecRevisionLeafType::Vec3f:
        return _LeafSample<GfVec3f>(key, attribute, resolved, upstream, at);
    case RigExecRevisionLeafType::Bool:
        return _LeafSample<bool>(key, attribute, resolved, upstream, at);
    case RigExecRevisionLeafType::Int:
        return _LeafSample<int>(key, attribute, resolved, upstream, at);
    case RigExecRevisionLeafType::Float:
        return _LeafSample<float>(key, attribute, resolved, upstream, at);
    case RigExecRevisionLeafType::Token:
        return _LeafSample<TfToken>(key, attribute, resolved, upstream, at);
    case RigExecRevisionLeafType::IntArray:
        return _LeafSample<VtIntArray>(key, attribute, resolved, upstream, at);
    case RigExecRevisionLeafType::FloatArray:
        return _LeafSample<VtFloatArray>(key, attribute, resolved, upstream,
                                         at);
    case RigExecRevisionLeafType::Vec2fArray:
        return _LeafSample<VtArray<GfVec2f>>(key, attribute, resolved,
                                             upstream, at);
    case RigExecRevisionLeafType::Vec3fArray:
        return _LeafSample<VtVec3fArray>(key, attribute, resolved, upstream,
                                         at);
    case RigExecRevisionLeafType::Vec3i:
        return _LeafSample<GfVec3i>(key, attribute, resolved, upstream, at);
    case RigExecRevisionLeafType::Vec3d:
        return _LeafSample<GfVec3d>(key, attribute, resolved, upstream, at);
    case RigExecRevisionLeafType::Matrix4d:
        return _LeafSample<GfMatrix4d>(key, attribute, resolved, upstream, at);
    case RigExecRevisionLeafType::DoubleArray:
        return _LeafSample<VtDoubleArray>(key, attribute, resolved, upstream,
                                          at);
    case RigExecRevisionLeafType::Dial: {
        // RigExecReadProjectorTarget's dial read, through \p resolved.
        double value = 0.0;
        const UsdAttribute &a = attribute;
        if (a && a.GetTypeName() == SdfValueTypeNames->Float) {
            float asFloat = 0.0f;
            if (!(resolved && resolved->GetAttributeOverStageLayer(
                                  a, at, upstream, &asFloat))) {
                a.Get(&asFloat, at);
            }
            value = double(asFloat);
        } else if (a) {
            if (!(resolved && resolved->GetAttributeOverStageLayer(
                                  a, at, upstream, &value))) {
                a.Get(&value, at);
            }
        }
        return VtValue(value);
    }
    }
    return key.fallback;
}

void
RigExecRevisionLeafHops(const RigExecRevisionLeafKey &key,
                        const UsdAttribute &attribute,
                        std::vector<SdfPath> *hops, bool *varying)
{
    hops->clear();
    *varying = false;
    hops->push_back(key.path);
    const bool atTime = key.time == RigExecRevisionLeafTime::AtTime;
    if (key.flavour == RigExecRevisionLeafFlavour::Present || !attribute) {
        return;
    }
    *varying = atTime && attribute.ValueMightBeTimeVarying();
    if (key.flavour != RigExecRevisionLeafFlavour::Resolved &&
        key.flavour != RigExecRevisionLeafFlavour::ResolvedOnly) {
        return;
    }
    // GetAttribute's walk: single authored connections, followed until a hop
    // has none or more than one, or the walk meets itself. The float read of
    // a double hop recurses from that hop over the same walk.
    std::set<SdfPath> visiting{attribute.GetPath()};
    UsdAttribute a = attribute;
    while (a) {
        SdfPathVector connections;
        if (a.HasAuthoredConnections()) {
            a.GetConnections(&connections);
        }
        if (connections.size() != 1) {
            break;
        }
        a = a.GetPrim().GetStage()->GetAttributeAtPath(connections[0]);
        if (!a || !visiting.insert(a.GetPath()).second) {
            break;
        }
        hops->push_back(a.GetPath());
        *varying = *varying || (atTime && a.ValueMightBeTimeVarying());
    }
}

namespace {

// The leaves of one covered operation, read where the stage assembler reads
// the stage.
struct _Leaves {
    const RigExecRevisionLeafView &view;

    const VtValue *At(RigExecRevisionLeafRole role) const {
        const int k = view.decl ? view.decl->Role(role) : -1;
        if (k < 0 || !view.values || size_t(k) >= view.values->size()) {
            return nullptr;
        }
        return &(*view.values)[size_t(k)];
    }

    // A scalar the operation always reads: an undeclared one is reported
    // (when asked) and answers the site's fallback.
    template <class T>
    T Scalar(RigExecRevisionLeafRole role, const T &fallback,
             const char *name) const {
        const VtValue *v = At(role);
        if (!v) {
            if (view.missing) {
                view.missing->push_back(name);
            }
            return fallback;
        }
        return v->IsHolding<T>() ? v->UncheckedGet<T>() : fallback;
    }

    // _Array: the leaf's array copied out, empty when undeclared. A points
    // read through the resolved inputs takes the phase overlay first; a raw
    // one (the lattice's bind-time cage, the wire's rest curve) never does.
    template <class T>
    std::vector<T> Array(RigExecRevisionLeafRole role) const {
        std::vector<T> out;
        const VtValue *v = At(role);
        if (!v) {
            return out;
        }
        if constexpr (std::is_same_v<T, GfVec3f>) {
            const RigExecRevisionLeafKey &key =
                view.decl->keys[size_t(view.decl->Role(role))];
            VtVec3fArray phased;
            if (view.phased &&
                key.flavour == RigExecRevisionLeafFlavour::OverlayThenRaw &&
                view.phased->Get(key.path, &phased)) {
                out.assign(phased.begin(), phased.end());
                return out;
            }
        }
        if (v->IsHolding<VtArray<T>>()) {
            const VtArray<T> &held = v->UncheckedGet<VtArray<T>>();
            out.assign(held.begin(), held.end());
        }
        return out;
    }

    // The leaf's VtArray itself, shared rather than copied; empty when
    // undeclared or holding another type.
    template <class T>
    VtArray<T> Shared(RigExecRevisionLeafRole role) const {
        const VtValue *v = At(role);
        return v && v->IsHolding<VtArray<T>>() ? v->UncheckedGet<VtArray<T>>()
                                                : VtArray<T>();
    }

    // A read only an existing attribute answers: false when the leaf is
    // empty (or undeclared, which is reported when asked).
    template <class T>
    bool IfPresent(RigExecRevisionLeafRole role, const char *name,
                   T *out) const {
        const VtValue *v = At(role);
        if (!v) {
            if (view.missing) {
                view.missing->push_back(name);
            }
            return false;
        }
        if (!v->IsHolding<T>()) {
            return false;
        }
        *out = v->UncheckedGet<T>();
        return true;
    }

    // Key \p k of the declaration as \p T, or \p fallback.
    template <class T>
    T Key(int k, const T &fallback) const {
        if (k < 0 || !view.values || size_t(k) >= view.values->size()) {
            return fallback;
        }
        const VtValue &v = (*view.values)[size_t(k)];
        return v.IsHolding<T>() ? v.UncheckedGet<T>() : fallback;
    }

    bool Enabled() const {
        return Scalar<bool>(RigExecRevisionLeafRole::Enabled, true,
                            "inputs:enabled");
    }
    float DefaultWeight() const {
        return Scalar<float>(RigExecRevisionLeafRole::DefaultWeight, 1.0f,
                             "inputs:defaultWeight");
    }
};

RigExecMoverParameters
_MatrixFromLeaves(const _Leaves &L, const RigExecProviderValues &values)
{
    RigExecMoverParameters params;
    params.kind = _kindTokens->matrix;
    params.radialWeight =
        L.Scalar<TfToken>(RigExecRevisionLeafRole::WeightBlend, TfToken(),
                          "rigExec:weightBlend") == "radial";
    params.enabled = L.Enabled();
    if (!params.enabled) {
        params.valid = true;  // disabled is an ordinary pass-through
        return params;
    }
    const GfMatrix4d *transform = values.transform;
    if (!transform) {
        return params;  // MoverFailed
    }
    params.weights = values.weights
        ? *values.weights
        : RigExecWeightPacket::Constant(L.DefaultWeight());
    if (!params.weights.valid) {
        return params;  // invalid common envelope => MoverFailed
    }
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
_SkinFromLeaves(const _Leaves &L, const RigExecProviderValues &values)
{
    RigExecMoverParameters params;
    params.kind = _kindTokens->skin;
    params.enabled = L.Enabled();
    if (!params.enabled) {
        params.valid = true;  // disabled is an ordinary pass-through
        return params;
    }
    if (!values.influenceTransforms) {
        return params;  // MoverFailed
    }
    params.weights = values.weights
        ? *values.weights
        : RigExecWeightPacket::Constant(L.DefaultWeight());
    if (!params.weights.valid) {
        return params;  // invalid common envelope => MoverFailed
    }
    params.skinTransforms = *values.influenceTransforms;
    params.skinningMethod = L.Scalar<TfToken>(
        RigExecRevisionLeafRole::SkinningMethod, _valueTokens->classicLinear,
        "rigExec:skinningMethod");
    // The layout the caller resolved; the cache is the stage assembler's.
    if (values.skinTopology) {
        params.skinTopology = *values.skinTopology;
    }
    if (params.skinTopology) {
        params.skinElementSize = params.skinTopology->elementSize;
        if (!params.skinTopology->validated ||
            params.skinTransforms.size() !=
                params.skinTopology->influenceCount) {
            return params;
        }
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
    // No resolved layout: the per-frame arrays.
    params.skinIndices = L.Array<int>(RigExecRevisionLeafRole::JointIndices);
    params.skinWeights =
        L.Array<float>(RigExecRevisionLeafRole::JointWeights);
    params.skinElementSize = L.Scalar<int>(
        RigExecRevisionLeafRole::ElementSize, 1, "rigExec:elementSize");
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
    if (params.skinningMethod != "classicLinear" &&
        params.skinningMethod != "dualQuaternion") {
        return params;
    }
    params.valid = true;
    return params;
}

}  // namespace

RigExecMoverParameters
RigExecAssembleFromLeaves(RigExecRevisionOp op,
                          const RigExecRevisionBinding &binding,
                          const RigExecRevisionLeafView &leaves,
                          const RigExecProviderValues &values)
{
    using Role = RigExecRevisionLeafRole;
    const _Leaves L{leaves};
    if (op == RigExecRevisionOp::Matrix) {
        return _MatrixFromLeaves(L, values);
    }
    if (op == RigExecRevisionOp::Skin) {
        return _SkinFromLeaves(L, values);
    }
    RigExecMoverParameters params;
    const bool synthesizedDerived =
        op == RigExecRevisionOp::RecomputeNormals ||
        op == RigExecRevisionOp::RecomputeExtent;
    params.enabled = synthesizedDerived ? true : L.Enabled();
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
    case RigExecRevisionOp::SurfaceProject:
        params.kind = _kindTokens->surfaceProject;
        break;
    case RigExecRevisionOp::Ribbon:
        params.kind = _kindTokens->ribbon;
        break;
    case RigExecRevisionOp::EmitGuidePoints:
        params.kind = _kindTokens->emitGuidePoints;
        break;
    case RigExecRevisionOp::RecomputeNormals:
        params.kind = _kindTokens->recomputeNormals;
        break;
    case RigExecRevisionOp::RecomputeExtent:
        params.kind = _kindTokens->recomputeExtent;
        break;
    case RigExecRevisionOp::DeltaMush:
        params.kind = _kindTokens->deltaMush;
        break;
    case RigExecRevisionOp::Wrinkle:
        params.kind = _kindTokens->wrinkle;
        break;
    case RigExecRevisionOp::Lattice:
        params.kind = _kindTokens->lattice;
        break;
    case RigExecRevisionOp::Wire:
        params.kind = _kindTokens->wire;
        break;
    case RigExecRevisionOp::External:
        params.kind = _kindTokens->external;
        break;
    case RigExecRevisionOp::SurfaceProjector:
        params.kind = _kindTokens->surfaceProjector;
        break;
    case RigExecRevisionOp::ShaderDials:
        params.kind = _kindTokens->shaderDials;
        break;
    case RigExecRevisionOp::Matrix:
    case RigExecRevisionOp::Skin:
        break;  // handled above
    }
    if (!params.enabled) {
        params.valid = true;  // disabled is an ordinary pass-through
        return params;
    }
    params.weights = synthesizedDerived
        ? RigExecWeightPacket::Constant(1.0f)
        : (values.weights
               ? *values.weights
               : RigExecWeightPacket::Constant(L.DefaultWeight()));
    if (!params.weights.valid) {
        return params;  // MoverFailed, preserving the preceding revision
    }
    switch (op) {
    case RigExecRevisionOp::BlendShape: {
        params.blendDeltas = values.blendDeltas;
        const TfToken space = L.Scalar<TfToken>(
            Role::DeltaSpace, _valueTokens->target, "rigExec:deltaSpace");
        if (space != "target" && space != "surfaceFrame") break;
        params.blendSurfaceFrame = space == "surfaceFrame";
        if (params.blendSurfaceFrame) {
            values.CopyBasePoints(&params.restPoints);
            params.topologyCounts = L.Array<int>(Role::TopologyCounts);
            params.topologyIndices = L.Array<int>(Role::TopologyIndices);
            if (params.topologyCounts.empty()) break;
        }
        params.valid = !params.blendDeltas.empty();
        break;
    }
    case RigExecRevisionOp::VolumeCorrect:
        params.strength = 1.0f;
        if (values.BasePointCount() != 0) {
            params.referenceVolume = RigExecBoundVolume(
                values.BasePointData(), values.BasePointCount());
            params.valid = true;
        }
        break;
    case RigExecRevisionOp::Smooth:
        params.strength = 1.0f;
        params.topologyCounts = L.Array<int>(Role::TopologyCounts);
        params.topologyIndices = L.Array<int>(Role::TopologyIndices);
        params.valid = !params.topologyCounts.empty();
        break;
    case RigExecRevisionOp::SurfaceProject:
        params.strength = 1.0f;
        params.auxPoints = L.Array<GfVec3f>(Role::SurfacePoints);
        params.topologyCounts = L.Array<int>(Role::TopologyCounts);
        params.topologyIndices = L.Array<int>(Role::TopologyIndices);
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
            params.bindCoords = L.Array<GfVec2f>(Role::BindCoords);
            params.valid = !params.bindCoords.empty();
        } else {
            params.valid = true;
        }
        break;
    }
    case RigExecRevisionOp::RecomputeNormals:
    case RigExecRevisionOp::RecomputeExtent:
        values.CopyBasePoints(&params.auxPoints);
        params.topologyCounts = L.Array<int>(Role::TopologyCounts);
        params.topologyIndices = L.Array<int>(Role::TopologyIndices);
        if (op == RigExecRevisionOp::RecomputeExtent) {
            params.widths = L.Array<float>(Role::Widths);
        }
        params.valid =
            !params.auxPoints.empty() &&
            (op == RigExecRevisionOp::RecomputeExtent ||
             !params.topologyCounts.empty());
        break;
    case RigExecRevisionOp::DeltaMush:
    case RigExecRevisionOp::Wrinkle: {
        params.restPoints = L.Array<GfVec3f>(Role::RestPoints);
        if (params.restPoints.empty()) values.CopyBasePoints(&params.restPoints);
        params.topologyCounts = L.Array<int>(Role::TopologyCounts);
        params.topologyIndices = L.Array<int>(Role::TopologyIndices);
        if (op == RigExecRevisionOp::Wrinkle) {
            const TfToken topology = L.Scalar<TfToken>(
                Role::WrinkleTopology, _valueTokens->cloth, "inputs:topology");
            if (topology != _valueTokens->cloth &&
                topology != _valueTokens->surfaceStruts) {
                break;
            }
            params.wrinkleSettings.topology =
                topology == _valueTokens->cloth
                    ? RigExecWrinkleTopology::Cloth
                    : RigExecWrinkleTopology::SurfaceStruts;
            params.wrinkleSettings.pinPoints = L.Array<int>(Role::PinPoints);
        }
        // The scalars, key by key in the declaration's order; an operation
        // whose declaration holds none answers each site's fallback.
        int k = leaves.decl ? leaves.decl->scalarBegin : -1;
        _IterativeScalars(op, params, [&](const TfToken &name, auto fallback,
                                          auto &field) {
            using T = decltype(fallback);
            if (k < 0) {
                if (leaves.missing) {
                    leaves.missing->push_back(name.GetString());
                }
                field = fallback;
                return;
            }
            field = L.Key<T>(k++, fallback);
        });
        params.valid =
            !params.restPoints.empty() && !params.topologyCounts.empty();
        break;
    }
    case RigExecRevisionOp::Lattice: {
        // auxPoints is the BIND-TIME cage and auxPointsB the live one, as
        // the stage assembler orders them.
        values.CopyBasePoints(&params.restPoints);
        params.auxPoints = L.Array<GfVec3f>(Role::RestCage);
        params.auxPointsB = L.Array<GfVec3f>(Role::LiveCage);
        params.divisions = L.Scalar<GfVec3i>(
            Role::Divisions, params.divisions, "rigExec:divisions");
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
    case RigExecRevisionOp::Wire: {
        params.restPoints = L.Array<GfVec3f>(Role::CurveRest);
        if (binding.driverTransformCount > 0) {
            const size_t t = size_t(binding.driverTransformCount);
            const size_t s = size_t(binding.driverSpaceCount);
            const size_t bt = size_t(binding.driverBaseTransformCount);
            const std::vector<GfMatrix4d> *table = values.influenceTransforms;
            if (!table || table->size() < t + s + bt) {
                break;  // MoverFailed
            }
            const VtFloatArray weights =
                L.Shared<float>(Role::DriverWeights);
            const VtFloatArray baseWeights =
                L.Shared<float>(Role::DriverBaseWeights);
            RigExecWireDriverFrame frame;
            frame.posedPoints =
                L.Scalar<TfToken>(Role::PointFrame, _valueTokens->rest,
                                  "rigExec:pointFrame") == "posed";
            frame.posedDelta =
                L.Scalar<TfToken>(Role::DeltaFrame, _valueTokens->local,
                                  "rigExec:driverDeltaFrame") == "posed";
            frame.carry = values.carry;
            RigExecPoseWireDrivers(*table, t, s, bt, weights, baseWeights,
                                   frame, &params.restPoints,
                                   &params.auxPoints);
        } else {
            params.auxPoints = L.Array<GfVec3f>(Role::CurveLive);
        }
        const VtIntArray order = L.Shared<int>(Role::CurveOrder);
        if (!order.empty()) {
            params.curveOrder = order[0];
        }
        params.curveKnots = L.Array<double>(Role::CurveKnots);
        float dropoff = 0.0f;
        if (L.IfPresent<float>(Role::Dropoff, "inputs:dropoffDistance",
                               &dropoff)) {
            params.dropoffDistance = dropoff;
        }
        params.wireBindCoords = L.Shared<GfVec2f>(Role::BindCoords);
        const RigExecNurbsCurve rest{&params.restPoints, params.curveOrder,
                                     &params.curveKnots};
        params.valid = rest.IsValid() &&
                       params.auxPoints.size() == params.restPoints.size() &&
                       !params.wireBindCoords.empty();
        break;
    }
    case RigExecRevisionOp::External: {
        params.externalHandler = binding.handler;
        if (!binding.handler || !binding.handler->assembleExternal ||
            !leaves.decl || !leaves.values || leaves.decl->externalBegin < 0) {
            if (leaves.missing) leaves.missing->push_back("external inputs");
            break;
        }
        params.externalSchema = binding.externalSchema;
        std::vector<VtValue> inputs;
        inputs.reserve(binding.externalInputs.size());
        for (size_t i = 0; i < binding.externalInputs.size(); ++i) {
            const auto &key = binding.externalInputs[i];
            const size_t slot = size_t(leaves.decl->externalBegin) + i;
            if (slot >= leaves.values->size()) break;
            const VtValue *phased = leaves.phased ? leaves.phased->Find(key.path) : nullptr;
            const bool phaseRead = key.time == RigExecRevisionLeafTime::AtTime &&
                (key.flavour == RigExecRevisionLeafFlavour::Resolved ||
                 key.flavour == RigExecRevisionLeafFlavour::ResolvedOnly ||
                 key.flavour == RigExecRevisionLeafFlavour::OverlayThenRaw);
            inputs.push_back(phased && phaseRead && key.type == RigExecRevisionLeafType::Vec3fArray
                ? *phased : (*leaves.values)[slot]);
        }
        const RigExecExternalProviderValues pureValues(values);
        const RigExecExternalInputContext ctx{binding, pureValues, inputs};
        params.valid = inputs.size() == binding.externalInputs.size() &&
            binding.handler->assembleExternal(ctx, &params.externalData);
        break;
    }
    case RigExecRevisionOp::SurfaceProjector:
    case RigExecRevisionOp::ShaderDials:
    case RigExecRevisionOp::Matrix:
    case RigExecRevisionOp::Skin:
        break;
    }
    return params;
}

bool
RigExecExternalPayloadIsRead(const RigExecRevisionLeafView &leaves,
                             const RigExecProviderValues &values)
{
    const _Leaves L{leaves};
    if (!L.Enabled()) {
        return false;
    }
    return values.weights ? values.weights->valid
                          : RigExecWeightPacket::Constant(L.DefaultWeight())
                                .valid;
}

void
RigExecReadProjectorTargetFromLeaves(RigExecRevisionOp op,
                                     const RigExecRevisionBinding &binding,
                                     const RigExecRevisionLeafView &leaves,
                                     RigExecProjectorReads *reads)
{
    using Role = RigExecRevisionLeafRole;
    const _Leaves L{leaves};
    if (op == RigExecRevisionOp::ShaderDials) {
        reads->dials.clear();
        const int begin = leaves.decl ? leaves.decl->dialBegin : -1;
        for (size_t i = 0; i < binding.shaderDials.size(); ++i) {
            reads->dials.push_back(
                begin < 0 ? 0.0 : L.Key<double>(begin + int(i), 0.0));
        }
        return;
    }
    reads->rayOrigin = L.Scalar<GfVec3d>(
        Role::RayOrigin, GfVec3d(0.0, 0.0, 0.0), "rigExec:rayOrigin");
    reads->rayDirection = L.Scalar<GfVec3d>(
        Role::RayDirection, GfVec3d(0.0, 0.0, 1.0), "rigExec:rayDirection");
    reads->rayUp = L.Scalar<GfVec3d>(Role::RayUp, GfVec3d(0.0, 1.0, 0.0),
                                     "rigExec:rayUp");
    reads->shaderOffset = L.Scalar<GfMatrix4d>(
        Role::ShaderOffset, GfMatrix4d(1.0), "rigExec:shaderOffset");
    reads->reproject =
        L.Scalar<TfToken>(Role::ProjectionMode, _valueTokens->material,
                          "rigExec:projectionMode") == "reproject";
    reads->faceVertexCounts = L.Array<int>(Role::TopologyCounts);
    reads->faceVertexIndices = L.Array<int>(Role::TopologyIndices);
}

// Source adapter and pure projector arithmetic survive executor retirement.
GfMatrix4d
RigExecWorldFromRest(const std::array<GfVec3d, 4> &restPoints,
                     const GfMatrix4d &restToPose)
{
    GfMatrix4d rest(1.0);
    RigExecPointsToMatrix(RigExecIdentityLandmarks(), restPoints, &rest);
    return rest * restToPose;
}

void
RigExecReadProjectorTarget(
    const UsdPrim &projectorPrim, RigExecRevisionOp op,
    const RigExecRevisionBinding &binding,
    const RigExecResolvedInputs *resolved, UsdTimeCode time,
    RigExecProjectorReads *reads)
{
    if (!projectorPrim) {
        return;
    }
    if (op == RigExecRevisionOp::ShaderDials) {
        // The dials, each read through the generation's resolved inputs so
        // a property chain revising one is what the shader sees.
        const UsdStageRefPtr stage = projectorPrim.GetStage();
        reads->dials.clear();
        for (const SdfPath &dial : binding.shaderDials) {
            const UsdAttribute a = stage->GetAttributeAtPath(dial);
            double value = 0.0;
            if (a && a.GetTypeName() == SdfValueTypeNames->Float) {
                float asFloat = 0.0f;
                if (!(resolved && resolved->GetAttribute(a, time, &asFloat))) {
                    a.Get(&asFloat, time);
                }
                value = double(asFloat);
            } else if (a) {
                if (!(resolved && resolved->GetAttribute(a, time, &value))) {
                    a.Get(&value, time);
                }
            }
            reads->dials.push_back(value);
        }
        return;
    }
    reads->rayOrigin = _RecordedInput<GfVec3d>(
        projectorPrim, _attrTokens->rayOrigin, GfVec3d(0.0, 0.0, 0.0), time,
        resolved);
    reads->rayDirection = _RecordedInput<GfVec3d>(
        projectorPrim, _attrTokens->rayDirection, GfVec3d(0.0, 0.0, 1.0),
        time, resolved);
    reads->rayUp = _RecordedInput<GfVec3d>(
        projectorPrim, _attrTokens->rayUp, GfVec3d(0.0, 1.0, 0.0), time,
        resolved);
    reads->shaderOffset = _RecordedInput<GfMatrix4d>(
        projectorPrim, _attrTokens->shaderOffset, GfMatrix4d(1.0), time,
        resolved);
    reads->reproject = _RecordedToken(projectorPrim,
                                      _attrTokens->projectionMode,
                                      _valueTokens->material, time) ==
                       "reproject";
    reads->faceVertexCounts = _Array<int>(
        projectorPrim, binding.topologyCounts, time, resolved);
    reads->faceVertexIndices = _Array<int>(
        projectorPrim, binding.topologyIndices, time, resolved);
}

bool
RigExecRunProjectorTarget(
    RigExecRevisionOp op, const RigExecRevisionBinding &binding,
    const RigExecSurfaceProjectorFrames &frames,
    const RigExecProjectorReads &reads,
    const std::vector<GfVec3f> &basePoints,
    const std::vector<GfVec3f> &finalPoints, const std::string &who,
    GfMatrix4d *matrix, std::vector<std::string> *diagnostics,
    RigExecSurfaceKernelCache<GfVec3f,GfVec3d> *cache)
{
    if (op == RigExecRevisionOp::ShaderDials) {
        *matrix = RigExecPackShaderDialsT<GfMatrix4d>(reads.dials);
        return true;
    }
    if (op != RigExecRevisionOp::SurfaceProjector) {
        return false;
    }
    RigExecSurfaceProjectorInputs<GfMatrix4d, GfVec3d> in;
    in.rayOrigin = reads.rayOrigin;
    in.rayDirection = reads.rayDirection;
    in.rayUp = reads.rayUp;
    in.shaderOffset = reads.shaderOffset;
    in.reproject = reads.reproject;
    in.hasSource = frames.named[0] && frames.resolved[0];
    in.sourceBase = frames.base[0];
    in.sourceFinal = frames.final[0];
    in.sourceSpaceNamed = frames.named[1];
    in.hasSourceSpace = frames.named[1] && frames.resolved[1];
    in.sourceSpaceBase = frames.base[1];
    in.sourceSpaceFinal = frames.final[1];
    in.spaceNamed = frames.named[2];
    in.hasSpace = frames.named[2] && frames.resolved[2];
    in.spaceFinal = frames.final[2];
    in.worldToMesh = binding.meshWorldInverse;
    return RigExecSolveSurfaceProjectorT(
        in, basePoints, finalPoints, reads.faceVertexCounts,
        reads.faceVertexIndices, static_cast<RigExecVertexNormalsFn>(&RigExecComputeVertexNormals),
        who, matrix, diagnostics, cache);
}

bool
RigExecEvaluateProjectorTarget(
    const UsdPrim &projectorPrim, RigExecRevisionOp op,
    const RigExecRevisionBinding &binding,
    const RigExecSurfaceProjectorFrames &frames,
    const std::vector<GfVec3f> &basePoints,
    const std::vector<GfVec3f> &finalPoints,
    const RigExecResolvedInputs *resolved, UsdTimeCode time,
    GfMatrix4d *matrix, std::vector<std::string> *diagnostics)
{
    if (!projectorPrim) {
        return false;
    }
    RigExecProjectorReads reads;
    RigExecReadProjectorTarget(projectorPrim, op, binding, resolved, time,
                               &reads);
    // Only the surface projector names its mover in a diagnostic.
    const std::string who = op == RigExecRevisionOp::SurfaceProjector
                                ? binding.moverPath.GetString()
                                : std::string();
    return RigExecRunProjectorTarget(op, binding, frames, reads, basePoints,
                                     finalPoints, who, matrix, diagnostics);
}

}  // namespace rigExec
