#include "rigExecGraph/blendLayout.h"
// Compiled revision bindings, typed leaves and pure revision kernels.
#ifndef RIGEXEC_MOVER_GRAPH_H
#define RIGEXEC_MOVER_GRAPH_H

#include "bodyPurity.h"
#include "moverGraphCaches.h"
#include "moverGraphTypes.h"
#include "types.h"

#include "rigExecMath/simdKernels.h"
#include "rigExecMath/solvers.h"
#include "rigExecMath/surfaceKernelCache.h"
#include "rigExecMath/wireKernelCache.h"

#include "pxr/base/tf/span.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/vt/value.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/usd/object.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/timeCode.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <typeinfo>
#include <unordered_map>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

/// The read phase declared for \p property: its rigExecReadPhase metadata,
/// or Base when none is authored.
///
/// Metadata is the only way to declare one; no schema attribute stands in
/// for it. Returns false and fills \p error on an unparseable authored
/// value; an absent declaration is Base and true.
bool RigExecResolveReadPhase(
    const UsdObject &property,
    RigExecReadPhase *phase,
    std::string *error);

/// The epoch-fixed skin layout of \p moverPrim, through \p cache.
///
/// The one call a dynamic frame makes that takes a lock
/// (RigExecSkinTopologyCache's, held across the build). A null return is the
/// cache's remembered REFUSAL: the layout can move within the epoch, so the
/// packet must read the arrays per frame. The baked program and its frozen
/// jobs never call it: their SkinTopology head op builds the same layout
/// from sampled leaves through RigExecBuildSkinTopology.
std::shared_ptr<const RigExecSkinTopology> RigExecResolveSkinTopology(
    const UsdPrim &moverPrim,
    size_t influenceCount,
    UsdTimeCode time,
    const RigExecResolvedInputs *resolved,
    RigExecSkinTopologyCache *cache);

/// Whether \p moverPrim's skin layout is epoch state: none of
/// rigExec:jointIndices, rigExec:jointWeights and rigExec:elementSize might
/// vary with time or has an authored connection. The question
/// RigExecResolveSkinTopology asks wherever the cache is refilled. Reads the
/// stage: owning thread only.
bool RigExecSkinLayoutIsFixed(const UsdPrim &moverPrim);

/// RigExecSkinLayoutIsFixed's two halves. The topology half asks it of
/// rigExec:jointIndices and rigExec:elementSize, whose authored edits
/// rebuild the program, so a program asks it once, at Build; the weights
/// half asks it of rigExec:jointWeights, a per-frame value whose edits do
/// not. Their conjunction is RigExecSkinLayoutIsFixed. Owning thread only.
bool RigExecSkinLayoutTopologyIsFixed(const UsdPrim &moverPrim);
bool RigExecSkinLayoutWeightsAreFixed(const UsdPrim &moverPrim);

/// Fills \p topology from a skin layout's arrays and element size against an
/// influence table of \p influenceCount entries: the copies, then the shape,
/// index range and weight checks of RigExecSkinLayout::Validate, which set
/// `pointCount` and `validated` only as far as they pass. The influence
/// matrices are the caller's to check per frame. Pure: no stage, no lock.
void RigExecBuildSkinTopology(TfSpan<const int> indices,
                              TfSpan<const float> weights, int elementSize,
                              size_t influenceCount,
                              RigExecSkinTopology *topology);

/// Resolves a mover's side-input bindings from the authored stage.
///
/// \p frameChainHeads maps a transform provider to its final frame-chain head,
/// which is what a "final" read phase selects; an empty map resolves every
/// phase to the authored provider. Pure resolution: reads the stage, authors
/// nothing.
RigExecRevisionBinding RigExecResolveRevisionBinding(
    const UsdPrim &moverPrim,
    const SdfPath &target,
    const std::map<SdfPath, SdfPath> &frameChainHeads);

/// The revision op a mover's schema type performs, or nullopt when the type is
/// not a point-chain mover.
std::optional<RigExecRevisionOp> RigExecRevisionOpForSchema(
    const TfToken &schemaType, const TfToken &curveMode);

/// Assembles a matrix mover's parameter packet.
///
/// Peer of _BuildMatrixMoverParameters in movers/matrixMover.cpp, but built from
/// values rather than from a VdfContext: \p transform and \p weights are the
/// already-evaluated results of the providers named by the revision binding,
/// pulled through a tap set on the authored stage. A null transform fails the
/// application. Null weights mean no object is bound, so the assembler reads
/// inputs:defaultWeight and synthesizes the common constant envelope.
/// The kind token an assembled packet carries for op (the table the
/// revision guard and kernel entry read).
const TfToken &RigExecRevisionKindToken(RigExecRevisionOp op);

RigExecMoverParameters RigExecAssembleMatrixParameters(
    const UsdPrim &moverPrim,
    const GfMatrix4d *transform,
    const RigExecWeightPacket *weights,
    UsdTimeCode time = UsdTimeCode::Default(),
    const RigExecResolvedInputs *resolved = nullptr);

/// Assembles a skin mover's parameter packet.
///
/// \p influenceTransforms are the already-evaluated computeMatrix results of
/// the providers named by rigExec:influences, in that order; null fails the
/// application. The per-point layout (rigExec:jointIndices, jointWeights,
/// elementSize) and the method token are static reads off the mover prim at
/// \p time -- unless \p topologyCache is given, in which case the layout is
/// resolved through it once per epoch and carried in the packet by handle.
/// The caller passes a cache only when Compile established that the layout
/// cannot change within the epoch. The packet is valid only when the layout
/// indexes the influence table in range with finite non-negative weights, so
/// the kernel never has to guard an element; the point-count half of the
/// check happens in the kernel, which is the first place the count is known.
RigExecMoverParameters RigExecAssembleSkinParameters(
    const UsdPrim &moverPrim,
    const std::vector<GfMatrix4d> *influenceTransforms,
    const RigExecWeightPacket *weights,
    UsdTimeCode time = UsdTimeCode::Default(),
    const RigExecResolvedInputs *resolved = nullptr,
    RigExecSkinTopologyCache *topologyCache = nullptr,
    const std::shared_ptr<const RigExecSkinTopology> *resolvedTopology =
        nullptr);

/// Derives a mover's status from its packet (spec §6.6): disabled and failed
/// movers both pass their preceding revision through, and a failure records the
/// first bad canonical address.
RigExecMoverStatus RigExecStatusForParameters(
    const RigExecMoverParameters &parameters, const SdfPath &moverPath);
/// The same, with the mover's path already spelled: a step body passes the
/// text Build captured rather than asking SdfPath for it.
RigExecMoverStatus RigExecStatusForParameters(
    const RigExecMoverParameters &parameters, const std::string &moverText);

/// Applies the matrix operation of \p p to \p pts in place, returning false
/// when the packet fails atomically (the envelope does not resolve to the
/// point count).
///
/// The envelope is resolved and applied INSIDE the kernel, unlike the
/// point3f[] ops: the weighted-matrix rule folds the weight into the movement
/// (p' = q + w (T q - q)) rather than blending a finished result.
///
/// Shared by the mover-graph revision node and by the baked program, which
/// runs the same operation with no VdfNetwork around it.
/// The scale a posed frame carries: the length of each column of its linear
/// part.
///
/// USD's row-vector convention puts a master's own transform on the RIGHT of
/// its descendants (world = local * parent), so a frame under a scaled
/// master factors as rigid * scale -- and for an orthonormal L,
/// (L S)^T (L S) = S L^T L S = S^2, whose diagonal is exactly those column
/// lengths. A frame nothing has scaled returns (1, 1, 1), so dividing by it
/// is the identity: a rig with no scaled master measures what it always did,
/// to the last bit.
///
/// A zero-length column is a degenerate frame that cannot be divided by, so
/// it reports 1 and leaves that axis alone rather than producing infinities.
inline GfVec3d
RigExecFrameScale(const GfMatrix4d &frame)
{
    GfVec3d scale(1.0, 1.0, 1.0);
    for (size_t c = 0; c < 3; ++c) {
        const double length =
            GfVec3d(frame[0][c], frame[1][c], frame[2][c]).GetLength();
        if (length > 0.0) {
            scale[c] = length;
        }
    }
    return scale;
}

/// M(transform) * inverse(M(space)) for rigExec:transformSpace, with the
/// projective column set exactly: the product of an affine matrix and an
/// affine inverse is affine, but not to the last bit, and the matrix mover
/// refuses a transform whose last column is not exactly (0, 0, 0, 1).
inline GfMatrix4d
RigExecMeasureInSpace(const GfMatrix4d &transform, const GfMatrix4d &space)
{
    GfMatrix4d m = transform * space.GetInverse();
    m[0][3] = 0.0;
    m[1][3] = 0.0;
    m[2][3] = 0.0;
    m[3][3] = 1.0;
    return m;
}

/// Remove an explicit neutral solve before applying the animated map.
/// As above, restore the exact affine column after inverse multiplication.
inline GfMatrix4d
RigExecMeasureFromReference(const GfMatrix4d &transform, const GfMatrix4d &reference)
{
    GfMatrix4d m = reference.GetInverse() * transform;
    m[0][3] = m[1][3] = m[2][3] = 0.0;
    m[3][3] = 1.0;
    return m;
}

/// inverse(M(space)) * M(transform): the same offset RigExecMeasureInSpace
/// measures, conjugated into the space's CURRENT frame rather than left in
/// the space's own coordinates.
///
/// The pair exists because world = local * parent in USD's row-vector
/// convention, so transform * space^-1 cancels the space and hands back the
/// driver's plain LOCAL matrix. Applied to world-space control points that
/// is a local offset used as a world one, and it points wherever it pointed
/// at bind however far the space has since been carried. Reversing the
/// product gives space^-1 * local * space, which rides the space.
///
/// Both return identity when the transform sits exactly at its space, so
/// swapping one for the other never makes a still rig move.
inline GfMatrix4d
RigExecMeasureInPosedSpace(const GfMatrix4d &transform, const GfMatrix4d &space)
{
    GfMatrix4d m = space.GetInverse() * transform;
    m[0][3] = 0.0;
    m[1][3] = 0.0;
    m[2][3] = 0.0;
    m[3][3] = 1.0;
    return m;
}


/// The offset a cluster applies, in the frame its POINTS are already in.
///
/// RigExecMeasureInSpace hands back `transform * space^-1`, which IS
/// invariant under anything applied above the rig: `(X*T) * (S*T)^-1` is
/// `X * S^-1` again. What is NOT invariant is applying that offset to
/// WORLD points, because those points have been carried:
///
///     gets    (p + T) * M  =  p*M + T*R
///     wants    p*M + T
///
/// which agree only when M's rotation R is the identity.
///
/// \p carry is the rig's own rest->pose map -- rigExec:space's
/// computeMatrix, normally a TRS master's -- and conjugating by it is the
/// whole correction: `(p+T) * T^-1*M*T` is `p*M + T`, which is what
/// rigidity wants, and the same product handles a rotated or scaled master
/// because nothing about it assumed a translation. Measured 2026-09-24 on
/// the biped with the face posed and Main moved (40, 0, 25):
/// head_top_aim_cluster alone put 5.70 units of shear into body_geo, and
/// with the carry named it is 0.0157 -- the residual every OTHER mover
/// contributes, which this function cannot see.
///
/// WITHOUT a carry the correction falls back to conjugating by the SPACE's
/// own scale, which is what this function did before a carry could be
/// named: it fixes a scaled master (the squetch clusters went 25.02 ->
/// 3.1e-5 on M_HeadwireTop tx=2.0) and does nothing whatever for a
/// translated or rotated one. Kept, and kept bit-identical, because a rig
/// that names no space must not lose the half-fix it already had.
///
/// The two never compose: a named carry already contains the master's
/// scale, so applying the scale conjugation as well would apply it twice.
///
/// A rig naming no space and standing under an unscaled master measures
/// (1,1,1) and gets \p m back untouched, so it is bit-identical.
inline GfMatrix4d
RigExecClusterInPointFrame(const GfMatrix4d &m, const GfMatrix4d &space,
                           bool posedPoints,
                           const GfMatrix4d *carry = nullptr)
{
    if (!posedPoints) {
        return m;
    }
    GfMatrix4d out;
    if (carry) {
        // The guard is "a carry was NAMED", never "the carry is identity":
        // `carry^-1 * m * carry` for a master standing at its rest is m to
        // the last few ulps and not bit for bit, and a rig that named a
        // space has asked for that product. What must stay untouched is the
        // rig that named NOTHING, which is the branch below.
        out = carry->GetInverse() * m * *carry;
    } else {
        const GfVec3d k = RigExecFrameScale(space);
        if (k == GfVec3d(1.0)) {
            return m;
        }
        GfMatrix4d scale(1.0), unscale(1.0);
        scale.SetScale(k);
        unscale.SetScale(GfVec3d(1.0 / k[0], 1.0 / k[1], 1.0 / k[2]));
        out = unscale * m * scale;
    }
    out[0][3] = 0.0;
    out[1][3] = 0.0;
    out[2][3] = 0.0;
    out[3][3] = 1.0;
    return out;
}

/// Carries a transform-driven wire's two control polygons into the frame
/// its POINTS are already in. The wire's counterpart of
/// RigExecClusterInPointFrame: the same correction, applied to a
/// difference vector instead of an offset matrix.
///
/// The applier adds `posed(u) - rest(u)`, and for a driver measured as
/// `T * S^-1` that difference is master-invariant by construction: under a
/// master motion M both T and S pick up M and it cancels. The control
/// points the driver is applied to are the curve's AUTHORED ones, so the
/// delta is computed wholly in the uncarried frame -- and then added to
/// points a skin has already carried by M:
///
///     gets    p*M + d
///     wants   (p + d)*M  =  p*M + d*M_linear
///
/// A translation leaves a difference vector alone, which is why this is
/// invisible under a moved master and shows only under a rotated or
/// scaled one. Transforming BOTH polygons by the carry makes the kernel's
/// difference `d*M_linear`: the translation cancels in the subtraction,
/// and NURBS evaluation commutes with an affine map because the basis is
/// a partition of unity, so Evaluate(carried CVs) == carry(Evaluate(CVs)).
/// The bind table (u, d) was measured against the authored rest curve and
/// stays valid, because the carry moves both curves equally. Measured
/// 2026-09-24 on the biped with the face posed and Main turned ry=35: the
/// nine carried head wires put 1.0257 units of non-rigid residual into
/// hair_geo, and with the carry applied the rig is rigid to 2e-5 -- the
/// floor every other mover leaves.
///
/// Same guard rule as the cluster: called when a carry was NAMED, never
/// because it is identity, so a rig naming nothing is bit for bit what it
/// was. A named carry already holds the master's scale, so the scale-only
/// correction an uncarried posed wire gets (its displacement multiplied by
/// the measuring space's scale) is NOT applied beside it; the two never
/// compose.
inline void
RigExecCarryWireCurves(std::vector<GfVec3f> *rest,
                       std::vector<GfVec3f> *posed,
                       const GfMatrix4d &carry)
{
    for (GfVec3f &p : *rest) {
        p = GfVec3f(carry.TransformAffine(GfVec3d(p)));
    }
    for (GfVec3f &p : *posed) {
        p = GfVec3f(carry.TransformAffine(GfVec3d(p)));
    }
}

/// How a transform-driven wire measures its drivers.
///
/// \p posedPoints is rigExec:pointFrame == "posed" (the wire runs after a
/// skin on its target), \p posedDelta is rigExec:driverDeltaFrame ==
/// "posed" (the offset is applied in the space's current frame), and
/// \p carry is rigExec:space's computeMatrix, or null when none is named.
struct RigExecWireDriverFrame {
    bool posedPoints = false;
    bool posedDelta = false;
    const GfMatrix4d *carry = nullptr;
};

/// One driver's offset from its space, as a wire applies it.
///
/// "local" is RigExecMeasureInSpace and "posed" RigExecMeasureInPosedSpace.
/// When either frame option is posed and the space carries a scale, both
/// matrices are unscaled first so the offset is measured in the asset's own
/// units; \p spaceScale receives that scale for the posed-point correction.
/// A wire asking for neither keeps the plain measurement bit for bit. The
/// runtime twin is the `measured` lambda in RrGeoAssembleWire.
inline GfMatrix4d
RigExecMeasureWireDriver(const GfMatrix4d &transform, const GfMatrix4d &space,
                         const RigExecWireDriverFrame &frame,
                         GfVec3d *spaceScale)
{
    const GfVec3d k = RigExecFrameScale(space);
    if (spaceScale) {
        *spaceScale = k;
    }
    if ((frame.posedPoints || frame.posedDelta) &&
        k != GfVec3d(1.0, 1.0, 1.0)) {
        GfMatrix4d unscale(1.0);
        unscale.SetScale(GfVec3d(1.0 / k[0], 1.0 / k[1], 1.0 / k[2]));
        const GfMatrix4d unscaledTransform = transform * unscale;
        const GfMatrix4d unscaledSpace = space * unscale;
        return frame.posedDelta
            ? RigExecMeasureInPosedSpace(unscaledTransform, unscaledSpace)
            : RigExecMeasureInSpace(unscaledTransform, unscaledSpace);
    }
    return frame.posedDelta ? RigExecMeasureInPosedSpace(transform, space)
                            : RigExecMeasureInSpace(transform, space);
}

/// Poses a transform-driven wire's control polygon from its provider table.
///
/// \p table holds the driver transforms, then their spaces, then the base
/// transforms, then their spaces (counts \p t, \p s, \p bt, and the rest).
/// \p restPoints is the authored polygon on input and the base-moved rest
/// polygon on output; \p auxPoints receives the posed polygon. A posed wire
/// with no carry has its displacement multiplied by the space's scale; one
/// with a carry has both polygons carried instead (RigExecCarryWireCurves).
/// Shared by native and frozen parameter assembly; runtime mirrors the same
/// measurement and per-call selected-pair cache.
inline void
RigExecPoseWireDrivers(const std::vector<GfMatrix4d> &table, size_t t,
                       size_t s, size_t bt, const VtFloatArray &weights,
                       const VtFloatArray &baseWeights,
                       const RigExecWireDriverFrame &frame,
                       std::vector<GfVec3f> *restPoints,
                       std::vector<GfVec3f> *auxPoints)
{
    const size_t bs = table.size() - t - s - bt;
    const auto pick = [](size_t count, size_t j) {
        return count <= 1 ? size_t(0) : j % count;
    };
    // One driver/space pair serves most CVs, so each selected pair's
    // measured matrix and scale compute once per call: the measurement
    // inverts the space, and a linear scan over the few distinct pairs
    // is nothing beside that. Shared by the base-motion and driver
    // arms; spaceless pairs are a table read and skip the cache.
    struct _MeasuredWirePair {
        size_t driver;
        size_t space;
        GfMatrix4d m;
        GfVec3d scale;
    };
    std::vector<_MeasuredWirePair> measuredCache;
    const auto measured = [&](size_t first, size_t count, size_t spaceFirst,
                              size_t spaceCount, size_t j,
                              GfVec3d *spaceScale) {
        const size_t ti = first + pick(count, j);
        if (spaceCount == 0) {
            return table[ti];
        }
        const size_t si = spaceFirst + pick(spaceCount, j);
        for (const _MeasuredWirePair &hit : measuredCache) {
            if (hit.driver == ti && hit.space == si) {
                if (spaceScale) {
                    *spaceScale = hit.scale;
                }
                return hit.m;
            }
        }
        GfVec3d scale(1.0, 1.0, 1.0);
        GfMatrix4d m = RigExecMeasureWireDriver(
            table[ti], table[si], frame, &scale);
        if (spaceScale) {
            *spaceScale = scale;
        }
        measuredCache.push_back({ti, si, m, scale});
        return m;
    };
    const bool carried = frame.posedPoints && frame.carry;
    auxPoints->resize(restPoints->size());
    for (size_t j = 0; j < restPoints->size(); ++j) {
        GfVec3f &rest = (*restPoints)[j];
        // A base motion moves the curve AND its rest: the wire then deforms
        // by the driver's motion on top of it.
        if (bt > 0) {
            const GfMatrix4d b = measured(t + s, bt, t + s + bt, bs, j,
                                          nullptr);
            const float wb = baseWeights.empty()
                ? 1.0f : baseWeights[pick(baseWeights.size(), j)];
            const GfVec3f moved(b.TransformAffine(GfVec3d(rest)));
            rest = rest + (moved - rest) * wb;
        }
        GfVec3d scale(1.0, 1.0, 1.0);
        const GfMatrix4d m = measured(0, t, t, s, j, &scale);
        const float w =
            weights.empty() ? 1.0f : weights[pick(weights.size(), j)];
        const GfVec3f moved(m.TransformAffine(GfVec3d(rest)));
        GfVec3f displacement = (moved - rest) * w;
        if (frame.posedPoints && !carried) {
            displacement = GfVec3f(displacement[0] * float(scale[0]),
                                   displacement[1] * float(scale[1]),
                                   displacement[2] * float(scale[2]));
        }
        (*auxPoints)[j] = rest + displacement;
    }
    if (carried) {
        RigExecCarryWireCurves(restPoints, auxPoints, *frame.carry);
    }
}

/// Whether a wire applies its envelope itself: a valid sparse field with a
/// zero default, where only the named points are worth evaluating.
inline bool
RigExecWireTakesSparseEnvelope(const RigExecWeightPacket &w)
{
    return w.valid && w.representation == "sparse" &&
           w.defaultWeight == 0.0f && w.indices.size() == w.values.size() &&
           (w.rangePolicy.IsEmpty() || w.rangePolicy == "strict" ||
            w.rangePolicy == "clamp");
}

/// RIGEXEC_ENABLE_SIMD (default true), read once when the library loads.
/// The kernels below take the choice as \p useSimd, so a step body passes
/// its program's copy and reads neither the environment nor a static; every
/// caller starts from this answer, so all paths make the same choice.
bool RigExecSimdEnabled();

/// Constructs this file's token tables on the calling thread. Build calls it
/// so that their lazy construction, which builds tokens from text, never runs
/// first on a worker.
void RigExecRevisionKernelTouchTokens();

bool RigExecApplyMatrixKernel(const RigExecMoverParameters &p,
                              std::vector<GfVec3f> *pts, bool useSimd);

/// Applies the skin operation of \p p to \p pts in place, returning false
/// when the packet fails atomically (cardinality mismatch, unknown method).
///
/// The envelope is NOT applied here: the caller blends the result against the
/// preceding revision, because that is where the "apply once" rule lives.
///
/// Shared by the mover-graph revision node and by the baked program, which
/// runs the same operation with no VdfNetwork around it.
bool RigExecApplySkinKernel(const RigExecMoverParameters &p,
                            std::vector<GfVec3f> *pts, bool useSimd);
/// One vertex range of the matrix operation, against an envelope the caller
/// already resolved at the FULL point count.
///
/// Peer of the skin range form below and there for the same reason: the
/// envelope resolves atomically over the whole array (a cardinality mismatch
/// fails the application before any point is written), so it cannot be
/// resolved per range -- a chunked caller resolves it once and hands the same
/// array to every range, indexed absolutely.
void RigExecApplyMatrixKernelRange(const RigExecMoverParameters &p,
                                   const float *envelope,
                                   size_t begin, size_t end, GfVec3f *pts,
                                   bool useSimd);

/// Blends \p blended over \p preceding for one vertex range, with
/// \p envelope the FULL resolved envelope and every array indexed
/// absolutely.
///
/// The "apply once" blend of RigExecRunRevisionKernel, as a range: per point
/// it reads two arrays and writes a third at the same index, so a range is an
/// independent sub-problem and splitting it changes nothing about the
/// arithmetic. The envelope is passed in already resolved because resolving
/// it is the whole-array decision the range form may not repeat.
void RigExecBlendEnvelopeRange(const GfVec3f *preceding, const float *envelope,
                               size_t begin, size_t end, GfVec3f *blended);

/// The same blend over the WHOLE array, split across threads the way
/// RigExecRunRevisionKernel splits it.
///
/// One definition of "which threshold and which grain the blend uses", so a
/// caller that owns the blend itself -- the baked program's revision step,
/// which resolved the envelope in an earlier step -- cannot end up threading
/// it differently from the mover-graph node beside it. Values are unaffected
/// either way: the blend reads and writes index i and nothing else.
void RigExecBlendEnvelopeAll(const GfVec3f *preceding, const float *envelope,
                             size_t count, GfVec3f *blended);

/// One influence split into a stretch and a unit dual quaternion; the
/// dual-quaternion skinning path blends these rather than the matrices.
/// Named here only as a pointer, so dualQuat.h stays out of every
/// translation unit that assembles a packet.
struct RigExecScaledDualQuat;

/// The influence tables one skin range reads.
///
/// The matrices themselves, plus the two forms the kernels want them in: the
/// float rows the SIMD linear-blend path loads and the split the
/// dual-quaternion path blends. Both are pure per-matrix functions of
/// `transforms`, so a caller that skins several ranges against one table
/// builds them once and hands them to every range; null means "derive it
/// here", which is what the full-range kernel passes.
struct RigExecSkinTransformsView {
    const GfMatrix4d *transforms = nullptr;
    size_t transformCount = 0;
    /// transformCount * RigExecSkinRowStride floats, or null.
    const float *rows = nullptr;
    /// transformCount + 1 entries (the last one the weight complement's
    /// identity), or null.
    const RigExecScaledDualQuat *palette = nullptr;
    size_t paletteSize = 0;
};

/// The view of \p p's own influence table, which is what an unchunked caller
/// skins against.
RigExecSkinTransformsView RigExecSkinTransformsOf(
    const RigExecMoverParameters &p);

/// The layout \p p and \p transforms describe over \p pointCount points.
///
/// One definition of "where are the indices, the weights and the element
/// size", because the answer depends on whether the packet carries an
/// epoch-fixed layout by handle, and a second copy of that rule is a second
/// chance to read the wrong array.
RigExecSkinLayout RigExecSkinLayoutForPacket(
    const RigExecMoverParameters &p,
    const RigExecSkinTransformsView &transforms,
    size_t pointCount);

/// The half of RigExecApplySkinKernel's whole-array validation that does NOT
/// depend on the influence matrices: the element shape, the index range and
/// the weights, against \p pointCount points.
///
/// Split out because the two halves are decided in different places once a
/// revision is chunked -- the layout is static for the frame and the matrices
/// are the last thing the pose walk produces -- and because ANDing the two
/// gives exactly the boolean the unsplit check gives.
bool RigExecSkinLayoutIsUsable(const RigExecMoverParameters &p,
                               size_t pointCount);

/// The other half: every influence matrix finite and affine.
bool RigExecSkinTransformsAreUsable(const GfMatrix4d *transforms,
                                    size_t count);

/// One vertex range of the skin operation of \p p, against \p transforms,
/// written in place over [\p begin, \p end) of \p pts.
///
/// The per-vertex body, and nothing else: NO validation -- the caller has
/// done it, whole-array, because every check the skin kernel makes is a
/// statement about the whole array -- and NO WorkParallelForN, so a range is
/// unconditionally serial and a caller that already split the work does not
/// split it again. Returns false only for a method neither kernel owns.
///
/// This is the ONE definition of the per-vertex skin body: the full-range
/// RigExecApplySkinKernel validates and then calls this inside its own
/// parallel loop, so a chunked caller and an unchunked one cannot deform a
/// vertex differently.
bool RigExecApplySkinKernelRange(const RigExecMoverParameters &p,
                                 const RigExecSkinTransformsView &transforms,
                                 size_t begin, size_t end,
                                 std::vector<GfVec3f> *pts, bool useSimd);

/// RigExecApplySkinKernel against an influence table other than the packet's
/// own, for a caller that folds the matrices outside the packet.
bool RigExecApplySkinKernelWithTransforms(
    const RigExecMoverParameters &p,
    const RigExecSkinTransformsView &transforms,
    std::vector<GfVec3f> *pts, bool useSimd);


/// Applies the blend-shape operation of \p p to \p pts in place, returning
/// false when the packet fails atomically (the deltas or the envelope do not
/// resolve to the point count, or the surface-frame transport fails).
///
/// The envelope is resolved and applied INSIDE the kernel, like the matrix
/// kernel and unlike the point3f[] ops: the deltas are added to the preceding
/// revision and blended back against it in one pass.
///
/// ONE definition, called by the mover-graph revision node and by the baked
/// program, which runs the same operation with no VdfNetwork around it: a
/// second copy of the blend would have to agree about where the envelope is
/// folded in, and the rigs that would show a disagreement are the ones no
/// fixture happened to have.
bool RigExecApplyBlendShapeKernel(const RigExecMoverParameters &p,
                                  std::vector<GfVec3f> *pts,
    RigExecSurfaceKernelCache<GfVec3f,GfVec3d> *cache = nullptr);

/// Recomputes the derived property \p op maintains -- vertex normals or an
/// extent -- from \p p and blends it over \p pts in place, returning false
/// when the packet fails atomically (nothing computed, or a cardinality the
/// authored property cannot hold).
///
/// The envelope is resolved and applied INSIDE the kernel, as for the matrix
/// and blend-shape kernels.
///
/// ONE definition, called by the mover-graph revision node and by the baked
/// program, which maintains the same property with no VdfNetwork around it:
/// two copies would have to keep agreeing about the cardinality rules that
/// decide when a recomputation fails instead of truncating.
bool RigExecApplyDerivedKernel(RigExecRevisionOp op,
                               const RigExecMoverParameters &p,
                               std::vector<GfVec3f> *pts);
/// Borrow authored output without first copying it; failed calls leave result intact.
bool RigExecApplyDerivedKernel(RigExecRevisionOp op,
    const RigExecMoverParameters &p,const GfVec3f *authored,size_t authoredCount,
    std::vector<GfVec3f> *result);


/// Applies \p op to \p pts in place, returning false when the packet fails
/// atomically.
///
/// The envelope is NOT applied here for the operations that take a separate
/// blend: RigExecRunRevisionKernel below is where the "apply once" rule
/// lives. Matrix, blendShape and the two derived recomputations fold the
/// envelope into their own arithmetic and are routed to the kernels above.
///
/// ONE definition, called by the mover-graph revision node and by the baked
/// program: a second copy of a deformation agrees on the fixtures that exist
/// and drifts on the ones that do not.
///
/// \p wireBasis is the caller's own memo for a sparse-envelope wire; null
/// builds the basis for this call only.
bool RigExecApplyRevisionKernel(RigExecRevisionOp op,
                                const RigExecMoverParameters &p,
                                std::vector<GfVec3f> *pts, bool useSimd,
                                RigExecWireBasisCache *wireBasis,
    RigExecSurfaceKernelCache<GfVec3f,GfVec3d> *cache = nullptr);

/// Runs one revision of \p op over \p pts in place, envelope included: the
/// packet check, the full-strength fast path, RigExecApplyRevisionKernel and
/// the "apply once" blend against the preceding revision. Returns false when
/// the revision must pass its preceding value through unchanged.
///
/// ONE definition of "apply once", called by the mover-graph revision node --
/// which is then only scratch-collect / call / write-back -- and by the baked
/// geometry loop. Two hand-written wrappers would have to agree about which
/// operations blend and which fold the envelope into their own arithmetic.
/// \p wireBasis as for RigExecApplyRevisionKernel.
bool RigExecRunRevisionKernel(RigExecRevisionOp op,
                              const RigExecMoverParameters &p,
                              std::vector<GfVec3f> *pts, bool useSimd,
                              RigExecWireBasisCache *wireBasis,
    RigExecSurfaceKernelCache<GfVec3f,GfVec3d> *cache = nullptr);

namespace geometryDetail {
/// Internal owned staging only: failure may modify output; the caller must
/// discard it and retain the preceding published version. Never borrowed points.
/// \p envelope, when given, is the separate-blend envelope its caller already
/// resolved from the packet over the points' count; null resolves it here.
bool RunDiscardableRevisionKernel(RigExecRevisionOp,const RigExecMoverParameters &,
    std::vector<GfVec3f> *,bool,RigExecWireBasisCache *,
    RigExecSurfaceKernelCache<GfVec3f,GfVec3d> *,
    const std::vector<float> *envelope = nullptr);
/// The same revision out of place: reads the \p count entering points at
/// \p in, which must not alias \p out, and writes the result to \p out.
bool RunDiscardableRevisionKernel(RigExecRevisionOp,const RigExecMoverParameters &,
    const GfVec3f *in,size_t count,std::vector<GfVec3f> *out,bool,
    RigExecWireBasisCache *,RigExecSurfaceKernelCache<GfVec3f,GfVec3d> *,
    const std::vector<float> *envelope = nullptr);
}

/// Whether \p envelope makes the "apply once" blend the identity, so the
/// copy of the preceding revision, the resolved envelope array and the blend
/// loop are all dead work.
///
/// RigExecBlendEnvelope's `weight >= 1` branch returns the candidate itself,
/// with no arithmetic, and RigExecWeightPacket::ResolveAll can only answer 1
/// for every element of a constant packet carrying no values, no indices and
/// a range policy it accepts. This is the packet EVERY unweighted mover gets,
/// because it is what inputs:defaultWeight synthesizes.
///
/// ONE definition, called by the mover-graph revision node and by the baked
/// geometry loop: two hand-copied predicates could disagree about when the
/// skip is safe, and the only rigs that would show it are the ones no
/// fixture happened to have (a partial constant envelope on a skin mover).
inline bool
RigExecEnvelopeIsFullStrength(const RigExecWeightPacket &envelope)
{
    return envelope.valid && envelope.representation == "constant" &&
           envelope.values.empty() && envelope.indices.empty() &&
           envelope.defaultWeight == 1.0f &&
           (envelope.rangePolicy.IsEmpty() ||
            envelope.rangePolicy == "strict" ||
            envelope.rangePolicy == "clamp");
}

/// A revision's apply-or-fail answer, decided from its packet before any
/// point is written. `Refuses`: the kernel refuses the packet. `Applies`: it
/// applies it, and nothing after its validation can refuse. `Deferred`: only
/// running the kernel answers, because its acceptance reads what it computes
/// (External's finite-output check, a surface-frame transport, a
/// dual-quaternion blend) or no validation is factored for the operation.
enum class RigExecRevisionAcceptance : uint8_t {
    Refuses = 0,
    Applies = 1,
    Deferred = 2,
};

/// The skinning arithmetic \p p names, which RigExecApplySkinKernelRange
/// dispatches on. Unknown fails the application; a dual-quaternion blend can
/// still fail at a vertex, a linear blend cannot.
enum class RigExecSkinMethod : uint8_t {
    Unknown,
    ClassicLinear,
    DualQuaternion,
};
RigExecSkinMethod RigExecSkinMethodOf(const RigExecMoverParameters &p);

/// Whether \p op blends its result back over the entering points afterwards
/// (RigExecRunRevisionKernel's "apply once") rather than folding the
/// envelope \p w into its own arithmetic.
bool RigExecRevisionTakesSeparateBlend(RigExecRevisionOp op,
                                       const RigExecWeightPacket &w);

/// RigExecRunRevisionKernel's answer over \p count entering points, from the
/// validation the matrix, blend-shape and wire kernels run first (one
/// definition each, shared with the kernel) and, for a wire that blends
/// separately, its envelope. Every other operation is Deferred once its
/// packet passes; a skin's answer is the baked program's, from the halves
/// RevisionStatic and the fold hold. \p envelopeResolves, when given, is
/// `p.weights.ResolvesAll(count)` already answered by a resolve of the
/// envelope at \p count, so it is not validated a second time.
RigExecRevisionAcceptance RigExecRevisionKernelAcceptance(
    RigExecRevisionOp op, const RigExecMoverParameters &p, size_t count,
    const bool *envelopeResolves = nullptr);


/// A provider's asset frame from its rest landmarks and a rest->pose map:
/// row-vector frame = rest * M. Identity rest landmarks give M itself.
GfMatrix4d RigExecWorldFromRest(const std::array<GfVec3d, 4> &restPoints,
                                const GfMatrix4d &restToPose);


/// Reads (and records, for the bake) what \p op needs off the stage,
/// through the generation's resolved inputs where the live assemblers do.
void RigExecReadProjectorTarget(
    const UsdPrim &projectorPrim, RigExecRevisionOp op,
    const RigExecRevisionBinding &binding,
    const RigExecResolvedInputs *resolved, UsdTimeCode time,
    RigExecProjectorReads *reads);

/// Runs a surface projector target (SurfaceProjector or ShaderDials) on its
/// gathered reads: the shared kernel on \p finalPoints against the authored
/// \p basePoints. False, with diagnostics, when no matrix is published.
/// \p who is the mover's path text the diagnostics name; a step body passes
/// the text Build spelled.
bool RigExecRunProjectorTarget(
    RigExecRevisionOp op, const RigExecRevisionBinding &binding,
    const RigExecSurfaceProjectorFrames &frames,
    const RigExecProjectorReads &reads,
    const std::vector<GfVec3f> &basePoints,
    const std::vector<GfVec3f> &finalPoints, const std::string &who,
    GfMatrix4d *matrix,
    std::vector<std::string> *diagnostics,
    RigExecSurfaceKernelCache<GfVec3f,GfVec3d> *cache = nullptr);

/// RigExecReadProjectorTarget then RigExecRunProjectorTarget: what the
/// dynamic walk and the baked program both call.
bool RigExecEvaluateProjectorTarget(
    const UsdPrim &projectorPrim, RigExecRevisionOp op,
    const RigExecRevisionBinding &binding,
    const RigExecSurfaceProjectorFrames &frames,
    const std::vector<GfVec3f> &basePoints,
    const std::vector<GfVec3f> &finalPoints,
    const RigExecResolvedInputs *resolved, UsdTimeCode time,
    GfMatrix4d *matrix, std::vector<std::string> *diagnostics);

/// Sums blend channels into dense per-point deltas against \p base
/// (spec §7.3): deltas derive against the authored base, never the preceding
/// revision, and each channel's weight is clamped to [0, lastActivation] then
/// interpolated between the bracketing samples.
///
/// Shared by the mover-owned computeMoverParameters kernel and by
/// RigExecRigEvaluator, which assembles the same packet from tapped channels
/// with no derived stage. One definition, so the two cannot drift.
///
/// Returns false on a structural error -- a channel with no samples, a
/// non-finite weight, a non-positive or repeated activation, or a sample whose
/// point count disagrees with \p base -- which fails the mover atomically
/// rather than applying a partial blend.
bool RigExecSumBlendChannels(
    const std::vector<RigExecBlendChannel> &channels,
    const std::vector<GfVec3f> &base,
    std::vector<GfVec3f> *deltas);
bool RigExecSumBlendChannels(
    const std::vector<RigExecBlendChannel> &channels,
    const GfVec3f *base, size_t baseCount, std::vector<GfVec3f> *deltas);

/// Assembles any revision's parameter packet without a derived stage.
///
/// Peer of the _Build*MoverParameters family in movers/. Static inputs
/// (strength, divisions, mode, topology, cage and bind arrays) are read from the
/// authored stage through \p binding; dynamic ones arrive in \p values.
/// \p time is the evaluation time for every static scene read the packet
/// needs (cage, surface, topology, bind coords, strength, enable). The kernels
/// read the same inputs through exec at the current time, so passing anything
/// else silently diverges on animated input.
RigExecMoverParameters RigExecAssembleParameters(
    const UsdPrim &moverPrim,
    RigExecRevisionOp op,
    const RigExecRevisionBinding &binding,
    const RigExecProviderValues &values,
    UsdTimeCode time = UsdTimeCode::Default());


/// The External arm's plugin call, shared by RigExecAssembleParameters and
/// the sampled boundary: reads declared leaves and calls the immutable
/// compiled handler. Reads the stage: owning thread only.
void RigExecAssembleExternalPayload(const UsdPrim &moverPrim,
                                    const RigExecRevisionBinding &binding,
                                    const RigExecProviderValues &values,
                                    UsdTimeCode time,
                                    RigExecExternalPayload *payload);


/// Whether \p role reads topology, which is epoch state: a skin's
/// jointIndices and elementSize, a mesh's face counts and indices, a
/// lattice's divisions, a wire curve's order and knots. An authored edit to
/// such a path rebuilds the program, so inside one program its read moves
/// only with the time (an attribute that varies), an interactive override
/// on one of its hops, or a rebind.
bool RigExecRevisionLeafRoleIsTopology(RigExecRevisionLeafRole role);

/// Whether RigExecAssembleFromLeaves covers \p op: every operation. A
/// projector's matrix targets read through
/// RigExecReadProjectorTargetFromLeaves, and an external mover's payload is
/// a leaf the caller samples (RigExecRevisionLeafView::external).
bool RigExecRevisionOpAssemblesFromLeaves(RigExecRevisionOp op);

/// Declares every read RigExecAssembleParameters can make for \p op on the
/// mover at \p moverPath -- a superset of what one call reads, whichever
/// branch its values take -- with the read site's flavour, time policy and
/// fallback. Declares nothing when \p op is not covered. Stage-free.
void RigExecDeclareRevisionLeaves(RigExecRevisionOp op,
                                  const SdfPath &moverPath,
                                  const RigExecRevisionBinding &binding,
                                  RigExecRevisionLeafDecl *decl);

/// Declares the three reads of the skin mover at \p moverPath's layout, as
/// RigExecResolveSkinTopology makes them: rigExec:jointIndices and
/// rigExec:jointWeights through the overlay at their exact paths then raw,
/// and rigExec:elementSize resolved then raw, over 1 (roles JointIndices,
/// JointWeights and ElementSize). `assembles` stays false. Stage-free.
void RigExecDeclareSkinLayoutLeaves(const SdfPath &moverPath,
                                    RigExecRevisionLeafDecl *decl);

/// The value \p key's read site answers at \p time (Default for an
/// AtDefault key) through \p resolved, or the key's fallback when it finds
/// nothing. \p attribute is the attribute at the key's path, invalid when
/// none stands there. A non-empty \p upstream is the upstream layer: every
/// stage read a connection-following flavour (and its head fallback) makes
/// answers from it first (GetAttributeOverStageLayer); Raw and
/// OverlayThenRaw reads never consult it. Reads the stage: owning thread
/// only.
VtValue RigExecSampleRevisionLeaf(
    const RigExecRevisionLeafKey &key, const UsdAttribute &attribute,
    const RigExecResolvedInputs *resolved, UsdTimeCode time,
    const std::map<SdfPath, VtValue> *upstream = nullptr);

/// The attributes \p key's read can reach from \p attribute: the attribute
/// itself, and for a connection-following flavour every hop of the
/// single-connection walk RigExecResolvedInputs::GetAttribute takes. Sets
/// \p varying when the read is at the time and some hop's value might vary
/// with it. Reads the stage: owning thread only.
void RigExecRevisionLeafHops(const RigExecRevisionLeafKey &key,
                             const UsdAttribute &attribute,
                             std::vector<SdfPath> *hops, bool *varying);


/// RigExecAssembleParameters for a covered operation, from sampled leaves:
/// the same arms, gates and order, with every stage read replaced by its
/// leaf. Reads no stage, takes no lock and builds no token from text.
/// \p values carries the provider values as for the stage assembler; its
/// `resolved` and `skinTopologyCache` are not read.
RigExecMoverParameters RigExecAssembleFromLeaves(
    RigExecRevisionOp op, const RigExecRevisionBinding &binding,
    const RigExecRevisionLeafView &leaves,
    const RigExecProviderValues &values);

/// Whether RigExecAssembleFromLeaves reaches an External revision's payload
/// over \p leaves and \p values: the enable and the envelope, which the
/// stage assembler checks before it calls the plugin.
bool RigExecExternalPayloadIsRead(const RigExecRevisionLeafView &leaves,
                                  const RigExecProviderValues &values);

/// RigExecReadProjectorTarget over sampled leaves: the same reads, each
/// answered by its leaf. The caller holds a valid projector prim (the
/// program refuses a missing one at Build). Reads no stage.
void RigExecReadProjectorTargetFromLeaves(
    RigExecRevisionOp op, const RigExecRevisionBinding &binding,
    const RigExecRevisionLeafView &leaves, RigExecProjectorReads *reads);

}  // namespace rigExec

#endif  // RIGEXEC_MOVER_GRAPH_H
