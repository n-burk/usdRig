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

/// One blend sample's activation and native target-shape points
/// (spec §7.3), delivered by RigExecBlendSample.computeBlendSampleData.
struct RigExecBlendSampleData {
    float activation = 1.0f;
    std::vector<GfVec3f> points;

    bool operator==(const RigExecBlendSampleData &o) const {
        return activation == o.activation && points == o.points;
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

/// Immutable per-mover parameter packet (spec §4.1: every concrete mover
/// schema owns a statically registered computeMoverParameters). The kind
/// token names the owning operation; unused fields stay default.
struct RigExecMoverParameters {
    /// Operation kind: matrix, blendShape, volumeCorrect, smooth,
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
               widths == o.widths && curvenetBinding == o.curvenetBinding &&
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
