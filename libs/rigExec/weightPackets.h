// Weight-packet kernels shared by OpenExec registrations and graph bodies.
// Callers supply resolved typed inputs; kernels perform no source access.
// weightReference.cpp remains an independent numerical judge and does not
// use these production packet builders.
#ifndef RIGEXEC_WEIGHT_PACKETS_H
#define RIGEXEC_WEIGHT_PACKETS_H

#include "types.h"

#include "rigExecMath/weightFields.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/tf/token.h"

#include <cstddef>
#include <vector>
#include <utility>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

/// Scratch belongs to one packet producer and is never published. Unique
/// sparse support order is reused until the authored index array changes.
struct RigExecWeightPacketWorkspace {
    std::vector<std::pair<int,float>> pairs;
    std::vector<int> support;
    std::vector<size_t> supportOrder;
    bool uniqueSupport=false;
    std::vector<float> combineField;
};

/// Applies the range policy to a candidate weight; returns false on a
/// strict violation or non-finite input (spec §4.1 RangePolicy).
bool RigExecApplyWeightRangePolicy(const TfToken &policy, float *w);

/// The resolved inputs of a RigExecStaticWeight.
///
/// Absent inputs are defaulted by the CALLER, not here: exec distinguishes
/// "the attribute has no value" from "the attribute is authored empty",
/// and collapsing the two in the kernel would quietly accept a rig that
/// exec rejects. The schema fallbacks each field expects are noted below.
struct RigExecStaticWeightInputs {
    TfToken representation;      ///< rigExec:representation, fallback
                                 ///< "constant"
    TfToken rangePolicy;         ///< rigExec:rangePolicy, fallback "strict"
    std::vector<float> values;   ///< rigExec:values
    std::vector<int> indices;    ///< rigExec:indices, sparse only
    float defaultWeight = 0.0f;  ///< rigExec:defaultWeight, fallback 0
};

/// Builds the packet an authored (painted) weight publishes. An invalid
/// packet is returned by value rather than signalled, because that is what
/// a consumer acts on: a mover with an invalid envelope passes its
/// preceding revision through atomically.
RigExecWeightPacket RigExecBuildStaticWeightPacket(
    const RigExecStaticWeightInputs &inputs);

/// Borrowed view of RigExecStaticWeightInputs for callers whose arrays
/// already live elsewhere: identical semantics, no intermediate copies.
/// Null with a zero size stands for an empty array; the source arrays
/// must outlive the call and remain unchanged.
struct RigExecStaticWeightInputsView {
    TfToken representation;      ///< rigExec:representation, fallback
                                 ///< "constant"
    TfToken rangePolicy;         ///< rigExec:rangePolicy, fallback "strict"
    const float *values = nullptr;  ///< rigExec:values
    size_t valuesSize = 0;
    const int *indices = nullptr;   ///< rigExec:indices, sparse only
    size_t indicesSize = 0;
    float defaultWeight = 0.0f;  ///< rigExec:defaultWeight, fallback 0
};

/// Builds the packet an authored (painted) weight publishes from
/// borrowed arrays. Same packet as RigExecBuildStaticWeightPacket for
/// the same array contents.
RigExecWeightPacket RigExecBuildStaticWeightPacket(
    const RigExecStaticWeightInputsView &inputs);
RigExecWeightPacket RigExecBuildStaticWeightPacket(
    const RigExecStaticWeightInputsView &inputs,RigExecWeightPacketWorkspace *);

/// The resolved inputs of a RigExecDynamicWeight, less its base packet.
struct RigExecDynamicWeightInputs {
    TfToken representation;  ///< rigExec:representation, fallback "constant"
    TfToken rangePolicy;     ///< rigExec:rangePolicy, fallback "strict"
    float driver = 1.0f;     ///< inputs:driver, fallback 1
    float scale = 1.0f;      ///< inputs:scale, fallback 1
    float bias = 0.0f;       ///< inputs:bias, fallback 0
};

/// Remaps \p base by r_i = (b_i d) s + a. \p base is null when
/// rigExec:baseWeight targets nothing, which is legal only for a constant
/// packet (spec §4.1).
RigExecWeightPacket RigExecBuildDynamicWeightPacket(
    const RigExecDynamicWeightInputs &inputs,
    const RigExecWeightPacket *base);

/// The rigid placement of a volume weight, inverted.
///
/// Built from the volume's POSED FRAME, not from a computeMatrix. A
/// computeMatrix is the rest->posed target-local map -- a DEFORMATION,
/// which is exactly what a matrix mover wants and exactly what a placement
/// is not. An unanimated volume has posed == rest, so its computeMatrix is
/// the identity, and a volume authored at rest:space Y=5 would generate
/// its field about the origin. The posed frame is the absolute location,
/// so the placement is the map taking the identity landmarks to it.
///
/// Scale and shear are then REMOVED rather than inverted along with the
/// rest: the guide a viewer draws is built from the orthonormalized posed
/// frame (guides are rigid), so a scale left in the matrix would deform
/// the field without deforming the drawn volume, and the artist would be
/// painting with a shape they cannot see. inputs:scaleX/Y/Z is the sole
/// authority on anisotropy, and it is applied per axis by the builders
/// below.
bool RigExecRigidWorldToLocal(
    const RigExecPointFrame &posed, GfMatrix4d *result);

/// The placement a volume publishes (pose.weightFrames) and the CPU oracle
/// measures its field in: the map taking the identity landmarks to the
/// volume's FINAL frame, scale and shear kept. Identity unless the frame is
/// valid, non-degenerate and finite. The dynamic walk's refresh and the
/// baked program's VolumePlacements steps, live and frozen, place a volume
/// through this one function. The exec packet path above places against
/// the BASE frame instead, and the two differ for a volume a constraint
/// revises.
GfMatrix4d RigExecVolumePlacement(const RigExecPointFrame &final);

/// The resolved inputs of a volumetric weight object.
///
/// One struct for all three shapes: the fields a shape does not read are
/// simply left at their defaults, which is cheaper and far less
/// error-prone than three near-identical structs that would still share
/// the whole placement/band prologue.
struct RigExecVolumeWeightInputs {
    TfToken representation;  ///< rigExec:representation, fallback "dense"
    TfToken rangePolicy;     ///< rigExec:rangePolicy, fallback "clamp"

    /// The volume's posed frame. \p hasPlacement is false when the
    /// computation has no frame at all, which is not the same as an
    /// identity one: a default-constructed RigExecPointFrame is valid and
    /// would place the field at the origin instead of rejecting it.
    RigExecPointFrame placement;
    bool hasPlacement = false;

    RigExecFalloffParams params;  ///< band, invert/strength, baked remap

    std::vector<GfVec3f> targetPoints;  ///< rigExec:weightTarget points
    /// rigExec:weightTarget cardinality, when the coordinates are not
    /// materialized: a caller that sized the target without gathering
    /// it sets this and leaves targetPoints empty. Zero (the default)
    /// derives the count from targetPoints, so existing callers that
    /// fill the coordinates are unaffected. Only the prologue reads
    /// it; the field kernels measure the selected points.
    size_t targetPointCount = 0;
    std::vector<GfVec3f> samplePoints;  ///< rigExec:sampleSource, may be
                                        ///< empty
    GfVec3f positiveScales = GfVec3f(1.0f);
    GfVec3f negativeScales = GfVec3f(1.0f);
    GfVec3f scales = GfVec3f(1.0f);     ///< inputs:scaleX/Y/Z (sphere,
                                        ///< curve)

    TfToken planeAxis;      ///< rigExec:planeAxis, fallback "y"
    TfToken planeBounds;    ///< rigExec:planeBounds, fallback "unbounded"
    float extentU = 1.0f;   ///< inputs:extentU, bounded plane only
    float extentV = 1.0f;   ///< inputs:extentV, bounded plane only

    std::vector<GfVec3f> curvePoints;  ///< rigExec:curve points (curve)
    /// Optional immutable call-scoped views replace the three owning arrays.
    bool usePointViews = false;
    RigExecWeightPointView targetView, sampleView, curveView;
    std::vector<GfVec3f> *localCurveScratch = nullptr;
};

/// True when a volumetric weight could still produce a field once its
/// points are known: its structural tokens are ones the shape accepts,
/// its placement inverts, and the divisors it is about to divide by
/// describe a volume. Nothing it consults costs anything to gather.
///
/// It exists so that a caller which must COPY a whole mesh to fill in
/// targetPoints -- the exec adapter reads its rigExec:weightTarget
/// through a read iterator -- can ask first and skip the copy for a
/// volume that is going to be rejected anyway. It is advisory only:
/// RigExecBuildVolumeWeightPacket repeats every one of these checks, so a
/// caller that never asks still gets exactly the same packet.
bool RigExecVolumeWeightCanBuild(
    const TfToken &typeName, const RigExecVolumeWeightInputs &inputs);

/// Builds the dense field a volumetric weight publishes. \p typeName is
/// the concrete schema type ("RigExecSphereWeight", "RigExecPlaneWeight",
/// "RigExecCurveWeight"); anything else yields an invalid packet
/// carrying the caller's representation and range policy, exactly as a
/// rejected one does.
///
/// Dispatch is on the type token rather than three entry points because
/// every caller but the exec adapters -- which know their own type
/// statically -- starts from a UsdPrim's type name, and because the three
/// shapes share the whole prologue and epilogue anyway.
RigExecWeightPacket RigExecBuildVolumeWeightPacket(
    const TfToken &typeName, const RigExecVolumeWeightInputs &inputs);

/// Constructs the packet builders' token table on the calling thread; Build
/// calls it so the table's lazy construction never runs first on a worker.
void RigExecWeightPacketsTouchTokens();

/// Folds \p inputs together in AUTHORED ORDER (subtract and overlay are
/// order dependent by design) and applies invert, strength, and the range
/// policy.
///
/// \p weightTargetCount is the combine's own rigExec:weightTarget point
/// count, consulted only when no input is dense enough to carry the
/// cardinality itself.
RigExecWeightPacket RigExecBuildCombineWeightPacket(
    const TfToken &representation, const TfToken &rangePolicy,
    const TfToken &combineMode,
    const std::vector<RigExecWeightPacket> &inputs, size_t weightTargetCount,
    float strength, float invert);

/// Folds borrowed input packets in AUTHORED ORDER. Same packet as the
/// owning overload for the same inputs; the packets must outlive the
/// call. Null entries are rejected, never skipped.
RigExecWeightPacket RigExecBuildCombineWeightPacket(
    const TfToken &representation, const TfToken &rangePolicy,
    const TfToken &combineMode,
    const std::vector<const RigExecWeightPacket *> &inputs,
    size_t weightTargetCount,
    float strength, float invert);

RigExecWeightPacket RigExecBuildCombineWeightPacket(
    const TfToken &representation, const TfToken &rangePolicy,
    const TfToken &combineMode,
    const std::vector<const RigExecWeightPacket *> &inputs,
    size_t weightTargetCount, float strength, float invert,
    RigExecWeightPacketWorkspace *workspace);

}  // namespace rigExec

#endif  // RIGEXEC_WEIGHT_PACKETS_H
