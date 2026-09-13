//
// RigExec weight-packet kernels (spec §4.1 weight objects).
//
// Every weight object resolves to one RigExecWeightPacket, and there are
// two places that have to produce one: the computeWeightPacket callbacks
// in moverKernels.cpp, which see a VdfContext, and the baked evaluation
// program, which sees dense arrays it bound at compile time. They must
// agree bit for bit -- the baked program is accepted only when every
// published value matches the dynamic path exactly -- so the arithmetic
// lives here once and both sides call it with plain values.
//
// This is the same arrangement RigExecComputeCurvenetWeightPacket already
// has (curvenetWeightComputations.h): the exec adapter reads the context
// and hands over C++ values, and nothing in this header knows that exec
// exists.
//
// It is NOT the CPU oracle. RigExecRigEvaluator::_ResolveWeights and
// _ResolveVolumeWeights are a deliberately independent second
// implementation (see the comment at the head of bakedProgram.cpp) and
// must never be refactored onto these functions -- their whole value is
// that they were written separately.
//
#ifndef RIGEXEC_WEIGHT_PACKETS_H
#define RIGEXEC_WEIGHT_PACKETS_H

#include "types.h"

#include "rigExecMath/weightFields.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/tf/token.h"

#include <cstddef>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

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
    std::vector<GfVec3f> samplePoints;  ///< rigExec:sampleSource, may be
                                        ///< empty
    GfVec3f scales = GfVec3f(1.0f);     ///< inputs:scaleX/Y/Z (sphere,
                                        ///< curve)

    TfToken planeAxis;      ///< rigExec:planeAxis, fallback "y"
    TfToken planeBounds;    ///< rigExec:planeBounds, fallback "unbounded"
    float extentU = 1.0f;   ///< inputs:extentU, bounded plane only
    float extentV = 1.0f;   ///< inputs:extentV, bounded plane only

    std::vector<GfVec3f> curvePoints;  ///< rigExec:curve points (curve)
};

/// Builds the dense field a volumetric weight publishes. \p typeName is
/// the concrete schema type ("RigExecSphereWeight", "RigExecPlaneWeight",
/// "RigExecCurveWeight"); anything else yields an invalid packet.
///
/// Dispatch is on the type token rather than three entry points because
/// every caller but the exec adapters -- which know their own type
/// statically -- starts from a UsdPrim's type name, and because the three
/// shapes share the whole prologue and epilogue anyway.
RigExecWeightPacket RigExecBuildVolumeWeightPacket(
    const TfToken &typeName, const RigExecVolumeWeightInputs &inputs);

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

}  // namespace rigExec

#endif  // RIGEXEC_WEIGHT_PACKETS_H
