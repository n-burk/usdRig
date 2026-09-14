//
// RigExec exec-facing value types (spec §12.1).
//
// Only genuinely RigExec-specific, non-geometry value types are registered
// with ExecTypeRegistry. Geometry stays native (VtArray never registered).
//
#ifndef RIGEXEC_TYPES_H
#define RIGEXEC_TYPES_H

#include "rigExecMath/pointFrame.h"
#include "rigExecMath/curvenetAdjustments.h"

#include "pxr/base/gf/vec2f.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/gf/vec3i.h"

#include <memory>
#include <vector>

namespace rigExec {

/// Retain the shared computation/type registration library in headless hosts.
void RigExecLoadComputations();

struct RigExecProfileMoverBinding;

/// Aggregate result of a packed solver boundary (spec §4.3): an immutable
/// sequence of frames plus the per-element reference (rest) landmark sets
/// needed by downstream SRT-aware consumers (blend, twist, views, joints).
struct RigExecPointFrameArray {
    std::vector<RigExecPointFrame> frames;
    std::vector<std::array<GfVec3d, 4>> rests;

    bool operator==(const RigExecPointFrameArray &o) const {
        return frames == o.frames && rests == o.rests;
    }
    bool operator!=(const RigExecPointFrameArray &o) const {
        return !(*this == o);
    }

    bool IsEmpty() const { return frames.empty(); }
    size_t GetSize() const { return frames.size(); }
};

/// Immutable resolved weight field value published by weight objects
/// (spec §7.1 RigExecWeightPacket, v0.1 subset).
/// A point array carried as ONE exec value.
///
/// Exec models a `point3f[]` attribute as an output holding a vector of
/// GfVec3f elements, so a value override for one is type-checked against
/// GfVec3f and there is no way to hand it a whole array
/// (exec/system.cpp `_ComputeWithOverrides`). Boxing the array inside a
/// struct makes it a single scalar value, which an override can carry --
/// the same shape as every other packet type here.
///
/// This is what lets a ribbon's driver-curve points reach exec with nothing
/// authored: a relationship accessor can request computations on its targets
/// but not a named attribute of them, so the points cannot be reached from
/// the ribbon prim directly.
struct RigExecPointsPacket {
    std::vector<GfVec3f> points;

    bool operator==(const RigExecPointsPacket &o) const {
        return points == o.points;
    }
    bool operator!=(const RigExecPointsPacket &o) const {
        return !(*this == o);
    }
};

struct RigExecWeightPacket {
    TfToken representation;      ///< constant, dense, or sparse
    TfToken rangePolicy;         ///< strict or clamp
    std::vector<float> values;   ///< 0, logicalCount, or indices.size()
    std::vector<int> indices;    ///< sparse support (sorted)
    float defaultWeight = 0.0f;  ///< constant broadcast or sparse fallback
    bool valid = false;

    bool operator==(const RigExecWeightPacket &o) const {
        return representation == o.representation &&
               rangePolicy == o.rangePolicy && values == o.values &&
               indices == o.indices && defaultWeight == o.defaultWeight &&
               valid == o.valid;
    }
    bool operator!=(const RigExecWeightPacket &o) const {
        return !(*this == o);
    }

    /// Effective weight for logical element i of a count-element target,
    /// or -1 for a cardinality mismatch.
    float Resolve(size_t i, size_t count) const;

    /// A normalized constant envelope. Non-finite or out-of-range values
    /// produce an invalid packet, which makes the consuming mover pass its
    /// preceding revision through atomically.
    static RigExecWeightPacket Constant(float weight);

    /// Resolves the complete normalized field for a count-element target.
    /// Returns false for an invalid packet, a cardinality mismatch, or a
    /// non-finite/out-of-range element; resolved is unchanged on failure.
    bool ResolveAll(size_t count, std::vector<float> *resolved) const;
};

/// Baked distance-to-weight remap for one volumetric weight object
/// (spec §4.1 volumetric extension): uniformly spaced samples of the
/// falloff curve over the ramp parameter r in [0, 1]. Empty means linear.
///
/// This exists as a packet, and reaches exec as a value OVERRIDE on a
/// stub computation, because exec has no accessor for an attribute's
/// spline -- `// XXX:TODO Accessors for AnimSpline` in
/// exec/computationBuilders.h is still open in v26.08. A computation can
/// resolve an attribute at ONE time; a falloff curve is the whole
/// function, so it cannot be an ordinary input.
///
/// That is not a workaround so much as the correct shape: the curve is
/// epoch-structural (spec §4.1 shape stability), so baking it once per
/// binding epoch and overriding is exactly what the ribbon's driver
/// points already do for the same class of reason (see
/// RigExecPointsPacket and RigExecValueOverride::attribute).
struct RigExecFalloffLut {
    std::vector<float> samples;

    bool operator==(const RigExecFalloffLut &o) const {
        return samples == o.samples;
    }
    bool operator!=(const RigExecFalloffLut &o) const {
        return !(*this == o);
    }
};

/// One blend sample's shape, resolved once per binding epoch and shared.
///
/// Peer of RigExecSkinTopology, and there for the same reason: the expensive
/// half of the operation depends only on data that cannot move within an
/// epoch, so it is read once and handed round by pointer.
///
/// `indices` empty means `offsets` is dense and parallel to the base points;
/// otherwise the two are parallel to each other and name the points that
/// move. The real correctives move 4.87% of a 26,276-point body (1,279 points
/// on average), which is why the sparse case is the one worth having:
/// tools/biped/spikes/blend_cost.py measures a dense sample at 0.37-0.38 ms
/// per frame REGARDLESS of its channel weight, because the cost is reading
/// and copying the full points array and not the accumulate loop. 169 dense
/// correctives is ~65 ms/frame with the rig standing at rest.
struct RigExecBlendSampleLayout {
    std::vector<GfVec3f> offsets;
    std::vector<int> indices;
    size_t pointCount = 0;
    bool valid = false;

    /// Whether two layouts describe the same shape.
    ///
    /// Packets compare layouts by POINTER once a layout is shared. This is
    /// the one place the arrays are compared by value: a cache dropped and
    /// re-filled by a notice that touched something else entirely asks it
    /// once, to decide whether it can hand back the pointer it already had
    /// rather than make the mover's packet compare unequal and re-run the
    /// whole accumulate for a shape that did not move.
    bool operator==(const RigExecBlendSampleLayout &o) const {
        return valid == o.valid && pointCount == o.pointCount &&
               indices == o.indices && offsets == o.offsets;
    }
    bool operator!=(const RigExecBlendSampleLayout &o) const {
        return !(*this == o);
    }
};

/// One blend sample's activation and shape (spec §7.3), delivered by
/// RigExecBlendSample.computeBlendSampleData.
///
/// The shape arrives one of two ways. `points` is the original dense form:
/// the target's full moved-points array, from which the accumulator
/// reconstructs a delta by subtracting the base. `layout` is the sparse form
/// resolved from rigExec:blendShape, which carries the offsets directly.
///
/// Exactly one is populated. `layout` is compared by POINTER, not by value:
/// two packets naming the same epoch-resolved shape name the same object, so
/// an unchanged sample costs one pointer compare instead of a 26,276-element
/// array compare -- the same trick, and the same reason, as
/// RigExecMoverParameters::skinTopology.
struct RigExecBlendSampleData {
    float activation = 1.0f;
    std::vector<GfVec3f> points;
    std::shared_ptr<const RigExecBlendSampleLayout> layout;

    bool operator==(const RigExecBlendSampleData &o) const {
        return activation == o.activation && points == o.points &&
               layout == o.layout;
    }
    bool operator!=(const RigExecBlendSampleData &o) const {
        return !(*this == o);
    }
};

/// One independently composable blend channel: authored weight plus its
/// samples, delivered by RigExecBlendInput.computeBlendChannel.
struct RigExecBlendChannel {
    float weight = 0.0f;
    std::vector<RigExecBlendSampleData> samples;

    bool operator==(const RigExecBlendChannel &o) const {
        return weight == o.weight && samples == o.samples;
    }
    bool operator!=(const RigExecBlendChannel &o) const {
        return !(*this == o);
    }
};

/// The per-point influence layout of one skin mover, resolved once per
/// binding epoch.
///
/// rigExec:jointIndices and rigExec:jointWeights are the largest static
/// inputs any mover reads -- one int and one float per influence slot per
/// point -- and they are LAYOUT: which joints move a point and how much,
/// which is exactly what a binding epoch fixes. Carrying them by value in
/// the packet meant re-reading them off the stage, re-copying them into the
/// packet, and re-validating every element on every frame, all to arrive at
/// the same arrays the frame before had.
///
/// Held by shared_ptr and compared by identity, for the same reason
/// RigExecProfileMoverBinding is: two packets naming the same layout name
/// the same arrays, and that identity IS the equality that matters.
struct RigExecSkinTopology {
    std::vector<int> indices;     ///< pointCount * elementSize
    std::vector<float> weights;   ///< parallel to indices
    int elementSize = 0;          ///< influence slots per point
    size_t pointCount = 0;        ///< indices.size() / elementSize
    /// The influence-table size the indices were range-checked against.
    /// Epoch state (rigExec:influences is), and the O(1) guard that lets the
    /// kernel trust the range check without repeating it.
    size_t influenceCount = 0;
    /// The shape, the index range and the weight values all passed; only
    /// the influence matrices and the point count are still frame business.
    bool validated = false;

    /// Whether two layouts describe the same binding.
    ///
    /// Packets compare layouts by POINTER, which is the equality that
    /// matters once a layout is shared. This is the one place the arrays are
    /// compared by value: a cache that has been dropped and re-filled asks
    /// it once, to decide whether it can hand back the pointer it already
    /// had -- which is what keeps an edit that touched nothing about the
    /// binding from re-running the per-point kernel.
    bool operator==(const RigExecSkinTopology &o) const {
        return elementSize == o.elementSize && pointCount == o.pointCount &&
               influenceCount == o.influenceCount &&
               validated == o.validated && indices == o.indices &&
               weights == o.weights;
    }
    bool operator!=(const RigExecSkinTopology &o) const {
        return !(*this == o);
    }
};

/// Immutable per-mover parameter packet (spec §4.1: every concrete mover
/// schema owns a statically registered computeMoverParameters). The kind
/// token names the owning operation; unused fields stay default.
struct RigExecMoverParameters {
    /// Operation kind: matrix, skin, blendShape, volumeCorrect, smooth,
    /// lattice, surfaceProject, ribbon, emitGuidePoints,
    /// recomputeNormals, or recomputeExtent.
    TfToken kind;
    bool enabled = true;
    bool valid = false;    ///< false => MoverFailed pass-through
    GfMatrix4d transform{1.0};
    /// Common MoverAPI envelope. A bound rigExec:weightObject supplies this
    /// packet; otherwise it is the constant packet synthesized from
    /// inputs:defaultWeight. It is applied after the operation computes its
    /// full-strength candidate.
    RigExecWeightPacket weights;
    /// Dense per-point combined blend delta (already channel-scaled).
    std::vector<GfVec3f> blendDeltas;
    bool blendSurfaceFrame = false;

    /// Operation scalars: volumeCorrect reference volume and the internal
    /// full-step strength consumed by smooth/volume/surface/curvenet kernels.
    /// The user-facing blend is always the common weights packet above.
    double referenceVolume = 0.0;
    float strength = 0.0f;

    /// Standard topology for smooth/normals/surface kernels.
    std::vector<int> topologyCounts;
    std::vector<int> topologyIndices;

    /// Auxiliary native point sets: lattice rest cage or surface driver
    /// points (auxPoints), lattice posed cage (auxPointsB), destination
    /// rest points for lattice binding (restPoints).
    std::vector<GfVec3f> auxPoints;
    std::vector<GfVec3f> auxPointsB;
    std::vector<GfVec3f> restPoints;
    GfVec3i divisions{0, 0, 0};

    /// Curve binding: two-component bind coordinates and the driver's
    /// posed+rest frame samples.
    std::vector<GfVec2f> bindCoords;
    RigExecPointFrameArray frames;

    /// Widths for the extent computation (empty, one, or per-point).
    std::vector<float> widths;

    /// Skin: one matrix per rigExec:influences entry, UsdSkel-layout
    /// indices and weights (skinElementSize slots per point), and the
    /// method token the kernel dispatches on (classicLinear |
    /// dualQuaternion).
    std::vector<GfMatrix4d> skinTransforms;
    std::vector<int> skinIndices;
    std::vector<float> skinWeights;
    int skinElementSize = 0;
    TfToken skinningMethod;

    /// The epoch-fixed layout, when the evaluator resolved one. Set means
    /// skinIndices/skinWeights are empty and the kernel reads the arrays
    /// here instead -- which avoids re-reading, re-copying and re-validating
    /// them once per frame.
    std::shared_ptr<const RigExecSkinTopology> skinTopology;

    /// Profile Mover state: the epoch's cut-mesh and factorization, shared
    /// rather than copied because it is large and identity IS the equality
    /// that matters -- two packets naming the same binding name the same
    /// cut. Held as an incomplete type so the geometry solver's headers stay
    /// out of every exec translation unit.
    std::shared_ptr<const RigExecProfileMoverBinding> curvenetBinding;
    RigExecCurvenetBasis curvenetAdjustmentBasis = RigExecCurvenetBasis::Bezier;
    std::vector<RigExecCurvenetAdjustmentCommand> curvenetAdjustments;

    bool operator==(const RigExecMoverParameters &o) const {
        return kind == o.kind && enabled == o.enabled && valid == o.valid &&
               transform == o.transform && weights == o.weights &&
               blendDeltas == o.blendDeltas && blendSurfaceFrame == o.blendSurfaceFrame &&
               referenceVolume == o.referenceVolume &&
               strength == o.strength &&
               topologyCounts == o.topologyCounts &&
               topologyIndices == o.topologyIndices &&
               auxPoints == o.auxPoints && auxPointsB == o.auxPointsB &&
               restPoints == o.restPoints && divisions == o.divisions &&
               bindCoords == o.bindCoords && frames == o.frames &&
               widths == o.widths &&
               skinTransforms == o.skinTransforms &&
               skinIndices == o.skinIndices &&
               skinWeights == o.skinWeights &&
               skinTopology == o.skinTopology &&
               skinElementSize == o.skinElementSize &&
               skinningMethod == o.skinningMethod &&
               curvenetBinding == o.curvenetBinding &&
               curvenetAdjustmentBasis == o.curvenetAdjustmentBasis &&
               curvenetAdjustments == o.curvenetAdjustments;
    }
    bool operator!=(const RigExecMoverParameters &o) const {
        return !(*this == o);
    }
};

/// Scalar status published by every concrete mover's computeMoverStatus
/// (spec §7.1): the property application consumes it and preserves the
/// preceding vector exactly when the state is not "ok".
struct RigExecMoverStatus {
    TfToken state;               ///< "ok", "disabled", or "moverFailed"
    std::string firstBadAddress; ///< first bad canonical value address

    bool operator==(const RigExecMoverStatus &o) const {
        return state == o.state && firstBadAddress == o.firstBadAddress;
    }
    bool operator!=(const RigExecMoverStatus &o) const {
        return !(*this == o);
    }

    bool AllowsApply() const { return state == "ok"; }
};

}  // namespace rigExec

#endif  // RIGEXEC_TYPES_H
