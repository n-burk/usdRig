#include "rigExecGraph/blendLayout.h"
// rigExecRuntime geometry family (M2): RevisionStatic, InfluenceFold,
// RevisionChunk, RevisionFuse, ChainStatus, Derived.
// A zero-USD port of the baked geometry loop (libs/rigExec/bakedGeometry.cpp
// driven by the movers in libs/rigExec/moverGraph.cpp and the kernels in
// libs/rigExecMath). The scalars an assembler reads through their
// connections, blend channel weights and default weights are read over the
// input slots, and so is every array the file lists as an input (path-read
// array rows, blend sample points, chain bases, fixed skin layouts); the
// remaining stage reads replay from the bake's captured path reads
// (RrStatic); epoch state (skin layouts, blend shapes, partitions) replays
// from the wire geometry tables. Bitwise: same ops in the same order, float
// stays float.

#include "rigExecRuntime/labels.h"
#include "rigExecRuntime/store.h"
#include "rigExecMath/deltaMushKernel.h"
#include "rigExecMath/latticeKernel.h"
#include "rigExecMath/surfaceKernelCache.h"
#include "rigExecMath/wireKernelCache.h"
#include "rigExecMath/wrinkleKernel.h"
#include "rigExecMath/surfaceProjectorKernel.h"
#include "poseInternal.h"

#include <algorithm>
#include <array>
#include <optional>
#include <tuple>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <queue>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

// The skin kernels below load the same SSE2 rows the baked path loads;
// the scalar fallback covers the same platforms it covers there.
#if !defined(RIGEXEC_DISABLE_SSE2)                                     \
    && (defined(__SSE2__) || (defined(_M_X64) && !defined(_M_ARM64EC)) \
        || (defined(_M_IX86_FP) && _M_IX86_FP >= 2))
#define RIGEXEC_RUNTIME_HAS_SSE2 1
#include <emmintrin.h>
#include <xmmintrin.h>
#else
#define RIGEXEC_RUNTIME_HAS_SSE2 0
#endif

namespace rigExec {
namespace {

// Operation ordinals, in RigExecRevisionOp order. The wire stores the
// ordinal; unknown operation values fail the step naming them.
enum RrGeoOp {
    RrGeoOpMatrix = 0,
    RrGeoOpSkin = 1,
    RrGeoOpBlendShape = 2,
    RrGeoOpVolumeCorrect = 3,
    RrGeoOpSmooth = 4,
    RrGeoOpLattice = 5,
    RrGeoOpSurfaceProject = 6,
    RrGeoOpRibbon = 7,
    RrGeoOpWire = 8,
    RrGeoOpEmitGuidePoints = 9,
    RrGeoOpRecomputeNormals = 12,
    RrGeoOpRecomputeExtent = 13,
    RrGeoOpDeltaMush = 14,
    RrGeoOpWrinkle = 15,
    RrGeoOpExternal = 16,
    RrGeoOpSurfaceProjector = 17,
    RrGeoOpShaderDials = 18,
};

const char *
RrGeoOpName(int op)
{
    switch (op) {
    case RrGeoOpMatrix: return "Matrix";
    case RrGeoOpSkin: return "Skin";
    case RrGeoOpBlendShape: return "BlendShape";
    case RrGeoOpVolumeCorrect: return "VolumeCorrect";
    case RrGeoOpSmooth: return "Smooth";
    case RrGeoOpDeltaMush: return "DeltaMush";
    case RrGeoOpWrinkle: return "Wrinkle";
    case RrGeoOpLattice: return "Lattice";
    case RrGeoOpSurfaceProject: return "SurfaceProject";
    case RrGeoOpRibbon: return "Ribbon";
    case RrGeoOpWire: return "Wire";
    case RrGeoOpEmitGuidePoints: return "EmitGuidePoints";
    case RrGeoOpRecomputeNormals: return "RecomputeNormals";
    case RrGeoOpRecomputeExtent: return "RecomputeExtent";
    case RrGeoOpExternal: return "External";
    case RrGeoOpSurfaceProjector: return "SurfaceProjector";
    case RrGeoOpShaderDials: return "ShaderDials";
    default: return nullptr;
    }
}

// The kind token the packet of each operation carries, in the same order.
const char *
RrGeoKindToken(int op)
{
    switch (op) {
    case RrGeoOpMatrix: return "matrix";
    case RrGeoOpSkin: return "skin";
    case RrGeoOpBlendShape: return "blendShape";
    case RrGeoOpVolumeCorrect: return "volumeCorrect";
    case RrGeoOpSmooth: return "smooth";
    case RrGeoOpDeltaMush: return "deltaMush";
    case RrGeoOpWrinkle: return "wrinkle";
    case RrGeoOpLattice: return "lattice";
    case RrGeoOpSurfaceProject: return "surfaceProject";
    case RrGeoOpRibbon: return "ribbon";
    case RrGeoOpWire: return "wire";
    case RrGeoOpEmitGuidePoints: return "emitGuidePoints";
    case RrGeoOpRecomputeNormals: return "recomputeNormals";
    case RrGeoOpRecomputeExtent: return "recomputeExtent";
    case RrGeoOpExternal: return "external";
    case RrGeoOpSurfaceProjector: return "surfaceProjector";
    case RrGeoOpShaderDials: return "shaderDials";
    default: return nullptr;
    }
}

// The mover-relative attributes the assemblers read, each a property of
// the revision's mover prim, named as the bake's path-read enumeration
// spells them. Open resolves every revision's to its property node and
// path-read rows.
enum RrGeoAttr : int {
    RrGeoAttrEnabled = 0,
    RrGeoAttrWeightBlend,
    RrGeoAttrPointFrame,
    RrGeoAttrDriverDeltaFrame,
    RrGeoAttrJointIndices,
    RrGeoAttrJointWeights,
    RrGeoAttrElementSize,
    RrGeoAttrSkinningMethod,
    RrGeoAttrDivisions,
    RrGeoAttrRestPoints,
    RrGeoAttrIterations,
    RrGeoAttrStep,
    RrGeoAttrPinBorders,
    RrGeoAttrDistanceWeight,
    RrGeoAttrDisplacement,
    RrGeoAttrTopology,
    RrGeoAttrPinPoints,
    RrGeoAttrNeighborDistance,
    RrGeoAttrRestLengthScale,
    RrGeoAttrStretchStiffness,
    RrGeoAttrCompressionStiffness,
    RrGeoAttrBendStiffness,
    RrGeoAttrMaxDisplacement,
    RrGeoAttrTangentPlaneCollisions,
    RrGeoAttrTangentPlaneInset,
    RrGeoAttrWrinkleScale,
    RrGeoAttrSmoothingIterations,
    RrGeoAttrDriverWeights,
    RrGeoAttrDriverBaseWeights,
    RrGeoAttrDropoffDistance,
    RrGeoAttrRayOrigin,
    RrGeoAttrRayDirection,
    RrGeoAttrRayUp,
    RrGeoAttrShaderOffset,
    RrGeoAttrProjectionMode,
    RrGeoAttrCount,
};

// Property names, in RrGeoAttr order.
constexpr const char *RrGeoAttrNames[RrGeoAttrCount] = {
    "inputs:enabled",
    "rigExec:weightBlend",
    "rigExec:pointFrame",
    "rigExec:driverDeltaFrame",
    "rigExec:jointIndices",
    "rigExec:jointWeights",
    "rigExec:elementSize",
    "rigExec:skinningMethod",
    "rigExec:divisions",
    "inputs:restPoints",
    "inputs:iterations",
    "inputs:step",
    "inputs:pinBorders",
    "inputs:distanceWeight",
    "inputs:displacement",
    "inputs:topology",
    "inputs:pinPoints",
    "inputs:neighborDistance",
    "inputs:restLengthScale",
    "inputs:stretchStiffness",
    "inputs:compressionStiffness",
    "inputs:bendStiffness",
    "inputs:maxDisplacement",
    "inputs:tangentPlaneCollisions",
    "inputs:tangentPlaneInset",
    "inputs:wrinkleScale",
    "inputs:smoothingIterations",
    "inputs:driverWeights",
    "inputs:driverBaseWeights",
    "inputs:dropoffDistance",
    "rigExec:rayOrigin",
    "rigExec:rayDirection",
    "rigExec:rayUp",
    "rigExec:shaderOffset",
    "rigExec:projectionMode",
};

// The stage handles of a revision's binding the assemblers read.
enum RrGeoBinding : int {
    RrGeoBindTopologyCounts = 0,
    RrGeoBindTopologyIndices,
    RrGeoBindCagePoints,
    RrGeoBindSurfacePoints,
    RrGeoBindBindCoords,
    RrGeoBindDriverCurvePoints,
    RrGeoBindDriverCurveOrder,
    RrGeoBindDriverCurveKnots,
    RrGeoBindWidths,
    RrGeoBindCount,
};

RrMat4d
RrGeoIdentity()
{
    RrMat4d m;
    m.SetIdentity();
    return m;
}

RrVec3f
RrGeoToVec3f(const RrVec3d &v)
{
    return RrVec3f(float(v[0]), float(v[1]), float(v[2]));
}

RrVec3d
RrGeoToVec3d(const RrVec3f &v)
{
    return RrVec3d(double(v[0]), double(v[1]), double(v[2]));
}

// GfDot(const GfQuatd&, const GfQuatd&), verbatim from quatd.h: the
// imaginary dot first, then the real product.
double
RrGeoQuatDot(const RrQuatd &a, const RrQuatd &b)
{
    return RrDot(a.GetImaginary(), b.GetImaginary()) +
           a.GetReal() * b.GetReal();
}

// GfMatrix3d::SetRotate(const GfQuatd&): the same _SetRotateFromQuat the
// 4x4 uses (verified equal on the USD build over random quaternions).
RrMat3d
RrGeoMat3SetRotate(const RrQuatd &q)
{
    const double r = q.GetReal();
    const RrVec3d i = q.GetImaginary();
    RrMat3d m;
    m[0][0] = 1.0 - 2.0 * (i[1] * i[1] + i[2] * i[2]);
    m[0][1] = 2.0 * (i[0] * i[1] + i[2] * r);
    m[0][2] = 2.0 * (i[2] * i[0] - i[1] * r);
    m[1][0] = 2.0 * (i[0] * i[1] - i[2] * r);
    m[1][1] = 1.0 - 2.0 * (i[2] * i[2] + i[0] * i[0]);
    m[1][2] = 2.0 * (i[1] * i[2] + i[0] * r);
    m[2][0] = 2.0 * (i[2] * i[0] + i[1] * r);
    m[2][1] = 2.0 * (i[1] * i[2] - i[0] * r);
    m[2][2] = 1.0 - 2.0 * (i[1] * i[1] + i[0] * i[0]);
    return m;
}

// GfMatrix3d::Orthonormalize: orthogonalize and normalize the row vectors
// (matrix3d.cpp), without the convergence warning, which the frame path
// never reads.
void
RrGeoMat3Orthonormalize(RrMat3d *m)
{
    RrVec3d r0(m->GetRow(0)), r1(m->GetRow(1)), r2(m->GetRow(2));
    RrOrthogonalizeBasis(&r0, &r1, &r2, true);
    m->SetRow(0, r0);
    m->SetRow(1, r1);
    m->SetRow(2, r2);
}

// GfMatrix3d::ExtractRotationQuaternion (matrix3d.cpp, transcribed
// verbatim including the int-typed 4 in the else arm), followed by the
// GfRotation round trip ExtractRotation().GetQuat() performs: SetQuat's
// length gate and axis/angle extraction, then GetQuat's half-angle sine
// and cosine with one normalization (rotation.cpp).
RrQuatd
RrGeoMat3ExtractRotationQuat(const RrMat3d &m)
{
    int i;
    if (m[0][0] > m[1][1]) {
        i = (m[0][0] > m[2][2] ? 0 : 2);
    } else {
        i = (m[1][1] > m[2][2] ? 1 : 2);
    }
    RrVec3d im(0.0, 0.0, 0.0);
    double r = 0.0;
    if (m[0][0] + m[1][1] + m[2][2] > m[i][i]) {
        r = 0.5 * std::sqrt(m[0][0] + m[1][1] + m[2][2] + 1);
        im[0] = (m[1][2] - m[2][1]) / (4.0 * r);
        im[1] = (m[2][0] - m[0][2]) / (4.0 * r);
        im[2] = (m[0][1] - m[1][0]) / (4.0 * r);
    } else {
        const int j = (i + 1) % 3;
        const int k = (i + 2) % 3;
        const double q =
            0.5 * std::sqrt(m[i][i] - m[j][j] - m[k][k] + 1);
        im[i] = q;
        im[j] = (m[i][j] + m[j][i]) / (4 * q);
        im[k] = (m[k][i] + m[i][k]) / (4 * q);
        r = (m[j][k] - m[k][j]) / (4 * q);
    }
    r = RrClamp(r, -1.0, 1.0);
    RrRotation rotation;
    rotation.SetQuat(RrQuatd(r, im));
    return rotation.GetQuat();
}

// Packet mirrors: the per-mover parameter packet and its shared layouts,
// compared exactly the way RigExecMoverParameters::operator== compares
// (layouts and bindings by pointer, everything else by value).

struct RrGeoWeightPacket {
    std::string representation;
    std::string rangePolicy;
    std::vector<float> values;
    std::vector<int> indices;
    struct Arrays {std::vector<float> values;std::vector<int> indices;};
    std::shared_ptr<const Arrays> arrays;
    const std::vector<float> &Values() const {return arrays?arrays->values:values;}
    const std::vector<int> &Indices() const {return arrays?arrays->indices:indices;}
    float defaultWeight = 0.0f;
    bool valid = false;

    bool operator==(const RrGeoWeightPacket &o) const
    {
        return representation == o.representation &&
               rangePolicy == o.rangePolicy && Values() == o.Values() &&
               Indices() == o.Indices() && defaultWeight == o.defaultWeight &&
               valid == o.valid;
    }
    bool operator!=(const RrGeoWeightPacket &o) const
    {
        return !(*this == o);
    }

    float Resolve(size_t i, size_t count) const
    {
        const auto &values=Values();const auto &indices=Indices();
        if (representation == "dense") {
            if (values.size() != count || i >= count) {
                return -1.0f;
            }
            return values[i];
        }
        if (representation == "sparse") {
            if (indices.size() != values.size()) {
                return -1.0f;
            }
            const auto it = std::lower_bound(
                indices.begin(), indices.end(), static_cast<int>(i));
            if (it != indices.end() && *it == static_cast<int>(i)) {
                return values[it - indices.begin()];
            }
            return defaultWeight;
        }
        // constant
        return defaultWeight;
    }

    static RrGeoWeightPacket Constant(float weight)
    {
        RrGeoWeightPacket packet;
        packet.representation = "constant";
        packet.rangePolicy = "strict";
        packet.defaultWeight = weight;
        packet.valid =
            std::isfinite(weight) && weight >= 0.0f && weight <= 1.0f;
        return packet;
    }

    // ResolveAll's validation, which it runs before it writes, so a decision
    // made ahead of the resolve cannot disagree with it.
    bool ResolvesAll(size_t count) const
    {
        const auto &values=Values();const auto &indices=Indices();
        if (!valid) {
            return false;
        }
        if (!rangePolicy.empty() &&
            rangePolicy != "strict" && rangePolicy != "clamp") {
            return false;
        }
        if (representation == "constant") {
            if (!values.empty() || !indices.empty()) {
                return false;
            }
        } else if (representation == "dense") {
            if (!indices.empty() || values.size() != count) {
                return false;
            }
        } else if (representation == "sparse") {
            if (indices.size() != values.size()) {
                return false;
            }
            for (size_t i = 0; i < indices.size(); ++i) {
                if (indices[i] < 0 ||
                    static_cast<size_t>(indices[i]) >= count ||
                    (i > 0 && indices[i] <= indices[i - 1])) {
                    return false;
                }
            }
        } else {
            return false;
        }
        const auto usable = [](float v) {
            return std::isfinite(v) && v >= 0.0f && v <= 1.0f;
        };
        if (representation == "constant") {
            return usable(defaultWeight);
        }
        if (representation == "dense") {
            for (size_t i = 0; i < count; ++i) {
                if (!usable(values[i])) {
                    return false;
                }
            }
            return true;
        }
        // Sparse: the default is only checked when some point can actually
        // read it.
        if (indices.size() < count && !usable(defaultWeight)) {
            return false;
        }
        for (const float value : values) {
            if (!usable(value)) {
                return false;
            }
        }
        return true;
    }

    bool ResolveAll(size_t count, std::vector<float> *resolved) const
    {
        const auto &values=Values();const auto &indices=Indices();
        if (!resolved || !ResolvesAll(count)) {
            return false;
        }

        if (representation == "constant") {
            resolved->assign(count, defaultWeight);
            return true;
        }

        if (representation == "dense") {
            resolved->assign(values.begin(), values.begin() + count);
            return true;
        }

        // Sparse: the default everywhere, then the authored entries
        // scattered over it.
        std::vector<float> valuesOut(count, defaultWeight);
        for (size_t i = 0; i < indices.size(); ++i) {
            valuesOut[static_cast<size_t>(indices[i])] = values[i];
        }
        resolved->swap(valuesOut);
        return true;
    }
};

struct RrGeoSkinTopology {
    std::vector<int> indices;
    std::vector<float> weights;
    int elementSize = 0;
    size_t pointCount = 0;
    size_t influenceCount = 0;
    bool validated = false;

    bool operator==(const RrGeoSkinTopology &o) const
    {
        return elementSize == o.elementSize && pointCount == o.pointCount &&
               influenceCount == o.influenceCount &&
               validated == o.validated && indices == o.indices &&
               weights.size() == o.weights.size() &&
               (weights.empty() || std::memcmp(weights.data(), o.weights.data(),
                   weights.size() * sizeof(float)) == 0);
    }
    bool operator!=(const RrGeoSkinTopology &o) const
    {
        return !(*this == o);
    }
};

struct RrGeoBlendLayout {
    std::vector<RrVec3f> offsets;
    std::vector<int> indices;
    size_t pointCount = 0;
    bool valid = false;

    bool operator==(const RrGeoBlendLayout &o) const
    {
        return valid == o.valid && pointCount == o.pointCount &&
               indices == o.indices && offsets == o.offsets;
    }
    bool operator!=(const RrGeoBlendLayout &o) const
    {
        return !(*this == o);
    }
};

struct RrGeoExternalInput {
    uint8_t type=0;
    bool have=false;
    size_t count=0;
    std::vector<uint8_t> bytes;
    std::string token;
    std::vector<std::string> tokens;
    bool operator==(const RrGeoExternalInput &other) const {
        return type==other.type && have==other.have && count==other.count &&
               bytes==other.bytes && token==other.token && tokens==other.tokens;
    }
};

struct RrGeoMoverParameters {
    std::string kind;
    bool enabled = true;
    bool valid = false;
    RrMat4d transform;
    // rigExec:weightBlend == "radial": a partial weight takes a fraction
    // of the rotation about the transform's own pivot instead of a
    // fraction of the resulting position. Replayed from the recorded
    // read, because there is no stage here to ask.
    bool radialWeight = false;
    RrGeoWeightPacket weights;
    std::vector<RrVec3f> blendDeltas;
    bool blendSurfaceFrame = false;
    double referenceVolume = 0.0;
    float strength = 0.0f;
    int mushIterations = 10;
    float mushStep = 0.5f;
    bool mushPinBorders = true;
    float mushDistanceWeight = 0.0f;
    float mushDisplacement = 1.0f;
    RigExecWrinkleSettings wrinkleSettings;
    std::vector<int> topologyCounts;
    std::vector<int> topologyIndices;
    std::vector<RrVec3f> auxPoints;
    std::vector<RrVec3f> auxPointsB;
    std::vector<RrVec3f> restPoints;
    RrVec3i divisions{0, 0, 0};
    std::vector<RrVec2f> bindCoords;
    RrPointFrameArray frames;
    std::vector<RrVec2f> wireBindCoords;
    int curveOrder = 0;
    std::vector<double> curveKnots;
    double dropoffDistance = 0.0;
    std::vector<float> widths;
    std::vector<RrMat4d> skinTransforms;
    std::vector<int> skinIndices;
    std::vector<float> skinWeights;
    int skinElementSize = 0;
    std::string skinningMethod;
    std::shared_ptr<const RrGeoSkinTopology> skinTopology;
    // A plugin mover: its playback state, this frame's bytes (null when the
    // export's assembly failed on the frame), and the points its binding
    // reads at a declared phase, as this run evaluated them.
    const RrProgram::ExternalRevision *external = nullptr;
    // The prepared state the packet was assembled under, so installing a
    // kernel re-runs a revision whose bytes did not move.
    const void *externalState = nullptr;
    const std::vector<uint8_t> *externalFrame = nullptr;
    std::vector<RrGeoExternalInput> externalInputs;
    std::vector<std::string> externalPhasedPaths;
    std::vector<uint8_t> externalPhasedHave;
    std::vector<std::vector<RrVec3f>> externalPhased;

    RrGeoMoverParameters()
    {
        transform.SetIdentity();
    }

    bool operator==(const RrGeoMoverParameters &o) const
    {
        return kind == o.kind && enabled == o.enabled && valid == o.valid &&
               radialWeight == o.radialWeight &&
               transform == o.transform && weights == o.weights &&
               blendDeltas == o.blendDeltas &&
               blendSurfaceFrame == o.blendSurfaceFrame &&
               referenceVolume == o.referenceVolume &&
               strength == o.strength &&
               mushIterations == o.mushIterations && mushStep == o.mushStep &&
               mushPinBorders == o.mushPinBorders &&
               mushDistanceWeight == o.mushDistanceWeight &&
               mushDisplacement == o.mushDisplacement &&
               wrinkleSettings == o.wrinkleSettings &&
               topologyCounts == o.topologyCounts &&
               topologyIndices == o.topologyIndices &&
               auxPoints == o.auxPoints && auxPointsB == o.auxPointsB &&
               restPoints == o.restPoints && divisions == o.divisions &&
               bindCoords == o.bindCoords && frames == o.frames &&
               wireBindCoords == o.wireBindCoords &&
               curveOrder == o.curveOrder && curveKnots == o.curveKnots &&
               dropoffDistance == o.dropoffDistance &&
               widths == o.widths &&
               skinTransforms == o.skinTransforms &&
               skinIndices == o.skinIndices &&
               skinWeights == o.skinWeights &&
               skinTopology == o.skinTopology &&
               skinElementSize == o.skinElementSize &&
               skinningMethod == o.skinningMethod &&
               external == o.external &&
               externalState == o.externalState &&
               externalFrame == o.externalFrame &&
               externalInputs == o.externalInputs &&
               externalPhasedPaths == o.externalPhasedPaths &&
               externalPhasedHave == o.externalPhasedHave &&
               externalPhased == o.externalPhased;
    }
    bool operator!=(const RrGeoMoverParameters &o) const
    {
        return !(*this == o);
    }
};

struct RrGeoMoverStatus {
    std::string state;
    std::string firstBadAddress;

    bool operator==(const RrGeoMoverStatus &o) const
    {
        return state == o.state && firstBadAddress == o.firstBadAddress;
    }
    bool operator!=(const RrGeoMoverStatus &o) const
    {
        return !(*this == o);
    }

    bool AllowsApply() const { return state == "ok"; }
};

// Envelope (envelope.h, solvers.cpp, moverGraph.cpp): the common blend,
// the weighted-matrix rule, and the full-strength predicate.

template <class T, class Weight>
T
RrGeoBlendEnvelope(const T &preceding, const T &full, Weight weight)
{
    if (weight <= Weight(0)) {
        return preceding;
    }
    if (weight >= Weight(1)) {
        return full;
    }
    return preceding + (full - preceding) * weight;
}

RrVec3d
RrGeoApplyWeightedMatrix(const RrVec3d &point, const RrMat4d &transform,
                         double weight)
{
    if (weight <= 0.0) {
        return point;
    }
    const RrVec3d moved = transform.TransformAffine(point);
    if (weight >= 1.0) {
        return moved;
    }
    return point + (moved - point) * weight;
}

// RrPartialTransform in runtimeMath.h, shared with the pose constraint
// step so a transform-domain matrix mover takes the same arc.
RrMat4d
RrGeoPartialTransform(const RrMat4d &transform, double weight)
{
    return RrPartialTransform(transform, weight);
}

bool
RrGeoEnvelopeIsFullStrength(const RrGeoWeightPacket &envelope)
{
    return envelope.valid && envelope.representation == "constant" &&
           envelope.Values().empty() && envelope.Indices().empty() &&
           envelope.defaultWeight == 1.0f &&
           (envelope.rangePolicy.empty() ||
            envelope.rangePolicy == "strict" ||
            envelope.rangePolicy == "clamp");
}

// RigExecRevisionAcceptance: a revision's apply-or-fail answer, decided from
// its packet before any point is written.
enum class RrGeoAcceptance : uint8_t {
    Refuses = 0,
    Applies = 1,
    Deferred = 2,
};

// A valid sparse field with a zero default, which the matrix and wire
// kernels walk point by point (RigExecWireTakesSparseEnvelope).
bool
RrGeoEnvelopeIsSparseWalk(const RrGeoWeightPacket &w)
{
    return w.valid && w.representation == "sparse" &&
           w.defaultWeight == 0.0f && w.Indices().size() == w.Values().size() &&
           (w.rangePolicy.empty() || w.rangePolicy == "strict" ||
            w.rangePolicy == "clamp");
}

// Every entry a sparse walk names in range, strictly ascending, finite and
// in [0, 1], as ResolveAll would validate it: the matrix and wire kernels'
// check before they write (moverGraph.cpp, _SparseWalkIsUsable).
bool
RrGeoSparseEnvelopeIsUsable(const RrGeoWeightPacket &w, size_t count)
{
    for (size_t k = 0; k < w.Indices().size(); ++k) {
        const int index = w.Indices()[k];
        const float value = w.Values()[k];
        if (index < 0 || size_t(index) >= count ||
            (k > 0 && index <= w.Indices()[k - 1]) ||
            !std::isfinite(value) || value < 0.0f || value > 1.0f) {
            return false;
        }
    }
    return true;
}

void
RrGeoBlendEnvelopeRange(const RrVec3f *preceding, const float *envelope,
                       size_t begin, size_t end, RrVec3f *blended)
{
    for (size_t i = begin; i < end; ++i) {
        blended[i] =
            RrGeoBlendEnvelope(preceding[i], blended[i], envelope[i]);
    }
}

void
RrGeoBlendEnvelopeAll(const RrVec3f *preceding, const float *envelope,
                     size_t count, RrVec3f *blended)
{
    // The runtime runs serially; a point range is an independent
    // sub-problem, so the serial loop is the parallel loop's answer.
    RrGeoBlendEnvelopeRange(preceding, envelope, 0, count, blended);
}

// Skin tables (simdKernels.*, dualQuat.*, solvers.cpp, pointFrame.cpp).

enum { RrGeoSkinRowStride = 16 };

void
RrGeoNarrowSkinRows(const RrMat4d &transform, float *rows)
{
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 3; ++c) {
            rows[r * 4 + c] = float(transform[r][c]);
        }
        rows[r * 4 + 3] = 0.0f;
    }
}

struct RrGeoSkinLayout {
    const RrMat4d *transforms = nullptr;
    size_t transformCount = 0;
    const int *indices = nullptr;
    const float *weights = nullptr;
    size_t indexCount = 0;
    size_t elementSize = 0;
    size_t pointCount = 0;

    float Weight(size_t point, size_t slot) const
    {
        return weights[point * elementSize + slot];
    }
    const RrMat4d &Transform(size_t point, size_t slot) const
    {
        return transforms[size_t(
            indices[point * elementSize + slot])];
    }
};

RrVec3d
RrGeoApplyLinearBlendSkin(const RrVec3d &point,
                         const RrGeoSkinLayout &layout, size_t i)
{
    RrVec3d sum(0.0);
    double total = 0.0;
    for (size_t k = 0; k < layout.elementSize; ++k) {
        const double w = layout.Weight(i, k);
        if (w == 0.0) {
            continue;
        }
        sum += layout.Transform(i, k).TransformAffine(point) * w;
        total += w;
    }
    // The complement stays with the rest point; exact when total == 1.
    return point * (1.0 - total) + sum;
}

void
RrGeoApplyLinearBlendSkin(const RrVec3f *in, RrVec3f *out,
                         const RrGeoSkinLayout &layout)
{
    for (size_t i = 0; i < layout.pointCount; ++i) {
        out[i] = RrGeoToVec3f(RrGeoApplyLinearBlendSkin(
            RrGeoToVec3d(in[i]), layout, i));
    }
}

#if !RIGEXEC_RUNTIME_HAS_SSE2

void
RrGeoApplyWeightedMatrixSimd(const RrVec3f *in, RrVec3f *out,
                            const float *weights, size_t count,
                            const RrMat4d &transform)
{
    for (size_t i = 0; i < count; ++i) {
        out[i] = RrGeoToVec3f(RrGeoApplyWeightedMatrix(
            RrGeoToVec3d(in[i]), transform, (weights ? weights[i] : 1.0f)));
    }
}

void
RrGeoApplyLinearBlendSkinSimd(const RrVec3f *in, RrVec3f *out,
                             const RrGeoSkinLayout &layout, const float *rows)
{
    (void)rows;
    RrGeoApplyLinearBlendSkin(in, out, layout);
}

#else

void
RrGeoApplyWeightedMatrixSimd(const RrVec3f *in, RrVec3f *out,
                            const float *weights, size_t count,
                            const RrMat4d &transform)
{
    // Row-vector convention: p' = x*row0 + y*row1 + z*row2 + row3.
    const __m128 row0 = _mm_setr_ps(
        float(transform[0][0]), float(transform[0][1]),
        float(transform[0][2]), 0.0f);
    const __m128 row1 = _mm_setr_ps(
        float(transform[1][0]), float(transform[1][1]),
        float(transform[1][2]), 0.0f);
    const __m128 row2 = _mm_setr_ps(
        float(transform[2][0]), float(transform[2][1]),
        float(transform[2][2]), 0.0f);
    const __m128 row3 = _mm_setr_ps(
        float(transform[3][0]), float(transform[3][1]),
        float(transform[3][2]), 0.0f);

    for (size_t i = 0; i < count; ++i) {
        const __m128 q = _mm_setr_ps(in[i][0], in[i][1], in[i][2], 0.0f);
        __m128 moved = _mm_add_ps(
            _mm_add_ps(
                _mm_mul_ps(_mm_shuffle_ps(q, q, _MM_SHUFFLE(0, 0, 0, 0)),
                           row0),
                _mm_mul_ps(_mm_shuffle_ps(q, q, _MM_SHUFFLE(1, 1, 1, 1)),
                           row1)),
            _mm_add_ps(
                _mm_mul_ps(_mm_shuffle_ps(q, q, _MM_SHUFFLE(2, 2, 2, 2)),
                           row2),
                row3));
        __m128 blended;
        if ((weights ? weights[i] : 1.0f) <= 0.0f) {
            blended = q;
        } else if ((weights ? weights[i] : 1.0f) >= 1.0f) {
            blended = moved;
        } else {
            const __m128 w = _mm_set1_ps((weights ? weights[i] : 1.0f));
            blended =
                _mm_add_ps(q, _mm_mul_ps(w, _mm_sub_ps(moved, q)));
        }
        alignas(16) float result[4];
        _mm_store_ps(result, blended);
        out[i] = RrVec3f(result[0], result[1], result[2]);
    }
}

void
RrGeoApplyLinearBlendSkinSimd(const RrVec3f *in, RrVec3f *out,
                             const RrGeoSkinLayout &layout, const float *rows)
{
    std::vector<float> narrowed;
    if (!rows) {
        narrowed.resize(layout.transformCount * RrGeoSkinRowStride);
        for (size_t t = 0; t < layout.transformCount; ++t) {
            RrGeoNarrowSkinRows(layout.transforms[t],
                               &narrowed[t * RrGeoSkinRowStride]);
        }
        rows = narrowed.data();
    }

    for (size_t i = 0; i < layout.pointCount; ++i) {
        const __m128 q = _mm_setr_ps(in[i][0], in[i][1], in[i][2], 0.0f);
        const __m128 qx = _mm_shuffle_ps(q, q, _MM_SHUFFLE(0, 0, 0, 0));
        const __m128 qy = _mm_shuffle_ps(q, q, _MM_SHUFFLE(1, 1, 1, 1));
        const __m128 qz = _mm_shuffle_ps(q, q, _MM_SHUFFLE(2, 2, 2, 2));
        __m128 sum = _mm_setzero_ps();
        float total = 0.0f;
        for (size_t k = 0; k < layout.elementSize; ++k) {
            const float w = layout.Weight(i, k);
            if (w == 0.0f) {
                continue;
            }
            const float *t =
                rows + size_t(layout.indices[i * layout.elementSize + k]) *
                           RrGeoSkinRowStride;
            // Row-vector convention: p' = x*row0 + y*row1 + z*row2 + row3.
            const __m128 moved = _mm_add_ps(
                _mm_add_ps(_mm_mul_ps(qx, _mm_loadu_ps(t)),
                           _mm_mul_ps(qy, _mm_loadu_ps(t + 4))),
                _mm_add_ps(_mm_mul_ps(qz, _mm_loadu_ps(t + 8)),
                           _mm_loadu_ps(t + 12)));
            sum = _mm_add_ps(sum, _mm_mul_ps(_mm_set1_ps(w), moved));
            total += w;
        }
        // The complement stays with the rest point; exact when total == 1.
        const __m128 blended =
            _mm_add_ps(_mm_mul_ps(_mm_set1_ps(1.0f - total), q), sum);
        alignas(16) float result[4];
        _mm_store_ps(result, blended);
        out[i] = RrVec3f(result[0], result[1], result[2]);
    }
}

#endif  // RIGEXEC_RUNTIME_HAS_SSE2

// \p useSimd is the program's RrGeoSettings::useSimd. Reads \p in and writes
// \p pts, which may be the same array: point i is written only after it is
// read.
void
RrGeoApplyMatrixKernelRange(const RrGeoMoverParameters &p,
                           const float *envelope, size_t begin, size_t end,
                           const RrVec3f *in, RrVec3f *pts, bool useSimd)
{
    if (p.radialWeight) {
        // Factored per WEIGHT rather than per point: a painted falloff is
        // mostly a handful of distinct values. Mirrors
        // RigExecApplyMatrixKernelRange exactly.
        double cachedWeight = -1.0;
        RrPartialDecomposition decomposition;
        bool haveDecomposition=false;
        RrMat4d partial(1.0);
        for (size_t i = begin; i < end; ++i) {
            const double w = envelope ? envelope[i] : 1.0;
            if (w != cachedWeight) {
                const double clamped=RrClamp(w,0.0,1.0);
                if(clamped<=0.0)partial=RrMat4d(1.0);
                else if(clamped>=1.0)partial=p.transform;
                else {
                    if(!haveDecomposition) {
                        decomposition=RrDecomposePartialTransform(p.transform);
                        haveDecomposition=true;
                    }
                    partial=RrApplyPartialDecomposition(decomposition,clamped);
                }
                cachedWeight = w;
            }
            pts[i] = RrGeoToVec3f(partial.TransformAffine(
                RrGeoToVec3d(in[i])));
        }
        return;
    }
    if (useSimd) {
        RrGeoApplyWeightedMatrixSimd(
            in + begin, pts + begin, envelope ? envelope + begin : nullptr, end - begin,
            p.transform);
    } else {
        // Element i is written only after it is read, so in-place is safe.
        for (size_t i = begin; i < end; ++i) {
            pts[i] = RrGeoToVec3f(RrGeoApplyWeightedMatrix(
                RrGeoToVec3d(in[i]), p.transform, envelope ? envelope[i] : 1.0f));
        }
    }
}

// The matrix kernel's validation, the one definition it runs first and
// RrGeoRevisionKernelAcceptance decides by (moverGraph.cpp,
// _MatrixKernelAccepts). Given \p weights, a dense resolve writes into it.
bool
RrGeoMatrixKernelAccepts(const RrGeoMoverParameters &p, size_t count,
                         std::vector<float> *weights)
{
    const RrGeoWeightPacket &w = p.weights;
    if (RrGeoEnvelopeIsFullStrength(w)) {
        return true;
    }
    if (RrGeoEnvelopeIsSparseWalk(w)) {
        return RrGeoSparseEnvelopeIsUsable(w, count);
    }
    // A cardinality mismatch fails atomically.
    return weights ? w.ResolveAll(count, weights) : w.ResolvesAll(count);
}

bool
RrGeoApplyMatrixKernel(const RrGeoMoverParameters &p, std::vector<RrVec3f> *pts,
                       bool useSimd)
{
    const size_t count = pts->size();
    std::vector<float> weights;
    if (!RrGeoMatrixKernelAccepts(p, count, &weights)) {
        return false;
    }
    const RrGeoWeightPacket &w = p.weights;
    if (RrGeoEnvelopeIsSparseWalk(w)) {
        if (p.transform == RrGeoIdentity()) {
            return true;  // at rest every weighted point maps to itself
        }
        RrVec3f *data = pts->data();
        if (p.radialWeight) {
            // The sparse walk takes the same arc the dense kernel does; a
            // painted cluster falloff is almost always sparse, so this is
            // the branch every radial cluster on a real rig reaches.
            double cachedWeight = -1.0;
        RrPartialDecomposition decomposition;
        bool haveDecomposition=false;
            RrMat4d partial(1.0);
            for (size_t k = 0; k < w.Indices().size(); ++k) {
                const double value = w.Values()[k];
                if (value != cachedWeight) {
                    const double clamped=RrClamp(value,0.0,1.0);
                if(clamped<=0.0)partial=RrMat4d(1.0);
                else if(clamped>=1.0)partial=p.transform;
                else {
                    if(!haveDecomposition) {
                        decomposition=RrDecomposePartialTransform(p.transform);
                        haveDecomposition=true;
                    }
                    partial=RrApplyPartialDecomposition(decomposition,clamped);
                }
                    cachedWeight = value;
                }
                RrVec3f &point = data[size_t(w.Indices()[k])];
                point = RrGeoToVec3f(
                    partial.TransformAffine(RrGeoToVec3d(point)));
            }
            return true;
        }
        for (size_t k = 0; k < w.Indices().size(); ++k) {
            RrVec3f &point = data[size_t(w.Indices()[k])];
            point = RrGeoToVec3f(RrGeoApplyWeightedMatrix(
                RrGeoToVec3d(point), p.transform, w.Values()[k]));
        }
        return true;
    }
    if(RrGeoEnvelopeIsFullStrength(p.weights)) {
        RrGeoApplyMatrixKernelRange(p,nullptr,0,count,pts->data(),pts->data(),useSimd);
        return true;
    }
    RrGeoApplyMatrixKernelRange(p, weights.data(), 0, count, pts->data(),
                                pts->data(), useSimd);
    return true;
}

// Dual-quaternion skinning (dualQuat.*, pointFrame.cpp): the per-matrix
// stretch/rotation split, the weighted palette blend, and the Kabsch SVD
// the non-rigid split fits through.

constexpr double RrGeoDualQuatEpsilon = 1e-9;

struct RrGeoDualQuat {
    RrQuatd real{1.0};
    RrQuatd dual{0.0};
};

struct RrGeoScaledDualQuat {
    RrGeoDualQuat rigid;
    RrMat3d stretch{1.0};
    bool isRigid = true;
};

struct RrGeoTransformParams {
    RrQuatd rotation{1.0};
    RrVec3d translation{0.0, 0.0, 0.0};
    RrVec3d scale{1.0, 1.0, 1.0};
    RrVec3d shear{0.0, 0.0, 0.0};
    int reflectionAxis = 2;
};

namespace {

// Jacobi eigen decomposition of a symmetric 3x3, transcribed from
// pointFrame.cpp: eigenvalues descending, eigenvectors as the columns
// of V, hemisphere- and repeated-eigenspace-canonicalized.
void
RrGeoSymmetricEigen3(const double m[3][3], double eval[3],
                    double evec[3][3])
{
    double a[3][3];
    double v[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            a[i][j] = m[i][j];

    for (int sweep = 0; sweep < 64; ++sweep) {
        double off = std::abs(a[0][1]) + std::abs(a[0][2]) +
                     std::abs(a[1][2]);
        if (off < 1e-15) break;
        for (int p = 0; p < 2; ++p) {
            for (int q = p + 1; q < 3; ++q) {
                if (std::abs(a[p][q]) < 1e-18) continue;
                const double theta = (a[q][q] - a[p][p]) / (2 * a[p][q]);
                const double t = (theta >= 0 ? 1.0 : -1.0) /
                    (std::abs(theta) + std::sqrt(theta * theta + 1));
                const double cth = 1.0 / std::sqrt(t * t + 1);
                const double sth = t * cth;
                for (int k = 0; k < 3; ++k) {
                    const double akp = a[k][p], akq = a[k][q];
                    a[k][p] = cth * akp - sth * akq;
                    a[k][q] = sth * akp + cth * akq;
                }
                for (int k = 0; k < 3; ++k) {
                    const double apk = a[p][k], aqk = a[q][k];
                    a[p][k] = cth * apk - sth * aqk;
                    a[q][k] = sth * apk + cth * aqk;
                }
                for (int k = 0; k < 3; ++k) {
                    const double vkp = v[k][p], vkq = v[k][q];
                    v[k][p] = cth * vkp - sth * vkq;
                    v[k][q] = sth * vkp + cth * vkq;
                }
            }
        }
    }

    int order[3] = {0, 1, 2};
    double d[3] = {a[0][0], a[1][1], a[2][2]};
    std::sort(order, order + 3, [&](int x, int y) { return d[x] > d[y]; });
    for (int i = 0; i < 3; ++i) {
        eval[i] = d[order[i]];
        for (int k = 0; k < 3; ++k) {
            evec[k][i] = v[k][order[i]];
        }
        int maxK = 0;
        for (int k = 1; k < 3; ++k) {
            if (std::abs(evec[k][i]) > std::abs(evec[maxK][i])) {
                maxK = k;
            }
        }
        if (evec[maxK][i] < 0) {
            for (int k = 0; k < 3; ++k) {
                evec[k][i] = -evec[k][i];
            }
        }
    }

    const double tol =
        1e-8 * std::max({std::abs(eval[0]), std::abs(eval[2]), 1.0});
    const bool eq01 = std::abs(eval[0] - eval[1]) <= tol;
    const bool eq12 = std::abs(eval[1] - eval[2]) <= tol;
    if (eq01 && eq12) {
        // Full isotropy: the canonical basis is the world basis.
        for (int i = 0; i < 3; ++i)
            for (int k = 0; k < 3; ++k)
                evec[k][i] = (k == i) ? 1.0 : 0.0;
    } else if (eq01 || eq12) {
        const int a = eq01 ? 0 : 1;
        const int other = eq01 ? 2 : 0;
        const double n[3] = {evec[0][other], evec[1][other], evec[2][other]};
        int bestAxis = 0;
        double bestLen = -1.0;
        double b1[3] = {0, 0, 0};
        for (int axis = 0; axis < 3; ++axis) {
            double p[3];
            for (int k = 0; k < 3; ++k) {
                p[k] = ((k == axis) ? 1.0 : 0.0) - n[k] * n[axis];
            }
            const double len =
                std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
            if (len > bestLen) {
                bestLen = len;
                bestAxis = axis;
                for (int k = 0; k < 3; ++k) b1[k] = p[k] / len;
            }
        }
        (void)bestAxis;
        double b2[3] = {
            n[1] * b1[2] - n[2] * b1[1],
            n[2] * b1[0] - n[0] * b1[2],
            n[0] * b1[1] - n[1] * b1[0]};
        int maxK = 0;
        for (int k = 1; k < 3; ++k) {
            if (std::abs(b2[k]) > std::abs(b2[maxK])) maxK = k;
        }
        if (b2[maxK] < 0) {
            for (int k = 0; k < 3; ++k) b2[k] = -b2[k];
        }
        for (int k = 0; k < 3; ++k) {
            evec[k][a] = b1[k];
            evec[k][a + 1] = b2[k];
        }
    }
}

bool
RrGeoPointsToParams(const std::array<RrVec3d, 4> &restPoints,
                   const std::array<RrVec3d, 4> &posePoints,
                   int reflectionAxis, RrGeoTransformParams *params)
{
    RrMat4d m;
    if (!RrPointsToMatrix(restPoints, posePoints, &m)) {
        return false;
    }

    double L[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            L[i][j] = m[j][i];

    double ltl[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double sum = 0;
            for (int k = 0; k < 3; ++k) sum += L[k][i] * L[k][j];
            ltl[i][j] = sum;
        }
    }
    double s2[3], V[3][3];
    RrGeoSymmetricEigen3(ltl, s2, V);

    double sigma[3];
    for (int i = 0; i < 3; ++i) {
        sigma[i] = std::sqrt(std::max(s2[i], 0.0));
    }
    const double detL =
        L[0][0] * (L[1][1] * L[2][2] - L[1][2] * L[2][1]) -
        L[0][1] * (L[1][0] * L[2][2] - L[1][2] * L[2][0]) +
        L[0][2] * (L[1][0] * L[2][1] - L[1][1] * L[2][0]);
    if (sigma[2] < 1e-14 * std::max(1.0, sigma[0])) {
        return false;  // singular posed frame: no SRT decomposition
    }

    double U[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double sum = 0;
            for (int k = 0; k < 3; ++k) sum += L[i][k] * V[k][j];
            U[i][j] = sum / sigma[j];
        }
    }

    const double delta = (detL >= 0) ? 1.0 : -1.0;

    int k = 2;
    if (delta < 0) {
        const int axis = reflectionAxis;
        double best = -1.0;
        for (int i = 0; i < 3; ++i) {
            const double align = std::abs(V[axis][i]);
            if (align > best) {
                best = align;
                k = i;
            }
        }
    }

    double R[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double sum = 0;
            for (int kk = 0; kk < 3; ++kk) {
                const double dkk = (kk == k) ? delta : 1.0;
                sum += U[i][kk] * dkk * V[j][kk];
            }
            R[i][j] = sum;
        }
    }

    double H[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double sum = 0;
            for (int kk = 0; kk < 3; ++kk) sum += R[kk][i] * L[kk][j];
            H[i][j] = sum;
        }
    }

    RrMat3d rowR(
        R[0][0], R[1][0], R[2][0],
        R[0][1], R[1][1], R[2][1],
        R[0][2], R[1][2], R[2][2]);
    RrGeoMat3Orthonormalize(&rowR);
    const RrQuatd q = RrGeoMat3ExtractRotationQuat(rowR);

    params->translation = RrVec3d(m[3][0], m[3][1], m[3][2]);
    params->rotation = q.GetNormalized();
    params->scale = RrVec3d(H[0][0], H[1][1], H[2][2]);
    params->shear = RrVec3d(
        H[0][1] / H[0][0],
        H[0][2] / H[0][0],
        H[1][2] / H[1][1]);
    params->reflectionAxis = reflectionAxis;
    return true;
}

bool
RrGeoQuatIsFinite(const RrQuatd &q)
{
    const RrVec3d &v = q.GetImaginary();
    return std::isfinite(q.GetReal()) && std::isfinite(v[0]) &&
           std::isfinite(v[1]) && std::isfinite(v[2]);
}

bool
RrGeoIsProperRotation(const RrMat4d &m, double tolerance)
{
    const RrVec3d r0(m[0][0], m[0][1], m[0][2]);
    const RrVec3d r1(m[1][0], m[1][1], m[1][2]);
    const RrVec3d r2(m[2][0], m[2][1], m[2][2]);
    const double gram[6] = {
        RrDot(r0, r0) - 1.0, RrDot(r1, r1) - 1.0, RrDot(r2, r2) - 1.0,
        RrDot(r0, r1), RrDot(r0, r2), RrDot(r1, r2),
    };
    for (const double g : gram) {
        if (!(std::abs(g) <= tolerance)) {
            return false;
        }
    }
    return RrDot(RrCross(r0, r1), r2) > 0.0;
}

bool
RrGeoDualQuatNormalize(RrGeoDualQuat *dq)
{
    const double norm = dq->real.GetLength();
    if (!(norm > RrGeoDualQuatEpsilon) || !std::isfinite(norm) ||
        !RrGeoQuatIsFinite(dq->dual)) {
        *dq = RrGeoDualQuat();
        return false;
    }
    const double inv = 1.0 / norm;
    dq->real *= inv;
    dq->dual *= inv;
    dq->dual -= dq->real * RrGeoQuatDot(dq->real, dq->dual);
    return true;
}

template <class GetDq, class GetWeight>
bool
RrGeoBlendDualQuats(size_t count, GetDq getDq, GetWeight getWeight,
                   RrGeoDualQuat *result)
{
    *result = RrGeoDualQuat();

    RrQuatd realSum(0.0);
    RrQuatd dualSum(0.0);
    RrQuatd reference(0.0);
    bool haveReference = false;
    double absWeightSum = 0.0;

    for (size_t i = 0; i < count; ++i) {
        const double w = getWeight(i);
        if (!std::isfinite(w)) {
            return false;
        }
        if (w == 0.0) {
            continue;
        }
        const RrGeoDualQuat *dq = getDq(i);
        if (!dq) {
            return false;
        }
        double sign = 1.0;
        if (!haveReference) {
            reference = dq->real;
            haveReference = true;
        } else if (RrGeoQuatDot(dq->real, reference) < 0.0) {
            sign = -1.0;
        }
        const double sw = sign * w;
        realSum += dq->real * sw;
        dualSum += dq->dual * sw;
        absWeightSum += std::abs(w);
    }

    if (!haveReference) {
        return false;
    }
    result->real = realSum;
    result->dual = dualSum;
    const double norm = realSum.GetLength();
    if (!(norm > RrGeoDualQuatEpsilon * std::max(1.0, absWeightSum))) {
        *result = RrGeoDualQuat();
        return false;
    }
    return RrGeoDualQuatNormalize(result);
}

RrGeoDualQuat
RrGeoDualQuatFromRotationTranslation(const RrQuatd &rotation,
                                    const RrVec3d &translation)
{
    RrGeoDualQuat dq;
    dq.real = rotation.GetNormalized();
    dq.dual = (RrQuatd(0.0, translation) * dq.real) * 0.5;
    return dq;
}

bool
RrGeoDecomposeMatrix(const RrMat4d &matrix, RrQuatd *rotation,
                    RrMat3d *stretch)
{
    if (RrGeoIsProperRotation(matrix, RrGeoDualQuatEpsilon)) {
        *rotation = matrix.ExtractRotationQuat();
        if (stretch) {
            stretch->SetIdentity();
        }
        return true;
    }

    const std::array<RrVec3d, 4> unitRest = {
        RrVec3d(0, 0, 0), RrVec3d(1, 0, 0),
        RrVec3d(0, 1, 0), RrVec3d(0, 0, 1)};
    RrGeoTransformParams params;
    const bool nonSingular = RrGeoPointsToParams(
        unitRest, RrMatrixToPoints(unitRest, matrix).points,
        /*reflectionAxis=*/2, &params);
    *rotation = nonSingular ? params.rotation : RrQuatd(1.0);

    if (stretch) {
        const RrMat3d linear = matrix.ExtractRotationMatrix();
        if (!nonSingular) {
            *stretch = linear;
        } else {
            // L = S * R  =>  S = L * R^T; symmetrise away rounding.
            const RrMat3d rowR = RrGeoMat3SetRotate(*rotation);
            const RrMat3d s = linear * rowR.GetTranspose();
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    (*stretch)[i][j] = 0.5 * (s[i][j] + s[j][i]);
                }
            }
        }
    }
    return false;
}

RrVec3d
RrGeoMulRow(const RrVec3d &p, const RrMat3d &s)
{
    return RrVec3d(
        p[0] * s[0][0] + p[1] * s[1][0] + p[2] * s[2][0],
        p[0] * s[0][1] + p[1] * s[1][1] + p[2] * s[2][1],
        p[0] * s[0][2] + p[1] * s[1][2] + p[2] * s[2][2]);
}

RrVec3d
RrGeoDualQuatRotateVector(const RrGeoDualQuat &dq, const RrVec3d &vector)
{
    const double w = dq.real.GetReal();
    const RrVec3d &u = dq.real.GetImaginary();
    const RrVec3d c = RrCross(u, vector);
    return vector + (c * w + RrCross(u, c)) * 2.0;
}

RrVec3d
RrGeoDualQuatTranslation(const RrGeoDualQuat &dq)
{
    const double w = dq.real.GetReal();
    const RrVec3d &v = dq.real.GetImaginary();
    const double dw = dq.dual.GetReal();
    const RrVec3d &dv = dq.dual.GetImaginary();
    return (dv * w - v * dw + RrCross(v, dv)) * 2.0;
}

RrVec3d
RrGeoDualQuatTransformPoint(const RrGeoDualQuat &dq, const RrVec3d &point)
{
    return RrGeoDualQuatRotateVector(dq, point) +
           RrGeoDualQuatTranslation(dq);
}

template <class GetSdq, class GetWeight>
bool
RrGeoBlendStretch(size_t count, GetSdq getSdq, GetWeight getWeight,
                 RrGeoScaledDualQuat *result)
{
    bool allRigid = true;
    double weightSum = 0.0;
    double absWeightSum = 0.0;
    for (size_t i = 0; i < count; ++i) {
        const double w = getWeight(i);
        if (w == 0.0) {
            continue;
        }
        weightSum += w;
        absWeightSum += std::abs(w);
        if (!getSdq(i)->isRigid) {
            allRigid = false;
        }
    }
    if (allRigid) {
        // Exact identity stretch: nothing to normalise, nothing dropped.
        return true;
    }
    if (!(std::abs(weightSum) >
          RrGeoDualQuatEpsilon * std::max(1.0, absWeightSum))) {
        *result = RrGeoScaledDualQuat();
        return false;
    }
    RrMat3d stretchSum(0.0);
    for (size_t i = 0; i < count; ++i) {
        const double w = getWeight(i);
        if (w == 0.0) {
            continue;
        }
        stretchSum += getSdq(i)->stretch * w;
    }
    result->stretch = stretchSum * (1.0 / weightSum);
    result->isRigid = false;
    return true;
}

RrGeoScaledDualQuat
RrGeoScaledDualQuatFromMatrix(const RrMat4d &matrix)
{
    RrGeoScaledDualQuat sdq;
    RrQuatd rotation(1.0);
    sdq.isRigid = RrGeoDecomposeMatrix(matrix, &rotation, &sdq.stretch);
    sdq.rigid = RrGeoDualQuatFromRotationTranslation(
        rotation, matrix.ExtractTranslation());
    return sdq;
}

bool
RrGeoBlendScaledDualQuats(const RrGeoScaledDualQuat *palette,
                         size_t paletteSize, const int *indices,
                         const double *weights, size_t count,
                         RrGeoScaledDualQuat *result)
{
    *result = RrGeoScaledDualQuat();
    if (count > 0 && (!palette || !indices || !weights)) {
        return false;
    }
    auto getSdq = [palette, paletteSize, indices](size_t i)
        -> const RrGeoScaledDualQuat * {
        const int index = indices[i];
        if (index < 0 || static_cast<size_t>(index) >= paletteSize) {
            return nullptr;
        }
        return &palette[index];
    };
    auto getWeight = [weights](size_t i) { return weights[i]; };
    if (!RrGeoBlendDualQuats(
            count,
            [&getSdq](size_t i) -> const RrGeoDualQuat * {
                const RrGeoScaledDualQuat *sdq = getSdq(i);
                return sdq ? &sdq->rigid : nullptr;
            },
            getWeight, &result->rigid)) {
        return false;
    }
    // Every non-zero-weight index was validated by the rigid blend.
    return RrGeoBlendStretch(count, getSdq, getWeight, result);
}

RrVec3d
RrGeoScaledDualQuatTransformPoint(const RrGeoScaledDualQuat &sdq,
                                 const RrVec3d &point)
{
    if (sdq.isRigid) {
        return RrGeoDualQuatTransformPoint(sdq.rigid, point);
    }
    return RrGeoDualQuatTransformPoint(
        sdq.rigid, RrGeoMulRow(point, sdq.stretch));
}

void
RrGeoGatherDualQuatInfluences(const RrGeoSkinLayout &layout, size_t i,
                             std::vector<int> *indices,
                             std::vector<double> *weights)
{
    indices->clear();
    weights->clear();
    size_t pivot = 0;
    float pivotWeight = -1.0f;
    double total = 0.0;
    for (size_t k = 0; k < layout.elementSize; ++k) {
        const float w = layout.Weight(i, k);
        if (pivotWeight < w) {
            pivotWeight = w;
            pivot = k;
        }
        total += w;
    }
    auto push = [&](size_t k) {
        const double w = layout.Weight(i, k);
        if (w != 0.0) {
            indices->push_back(layout.indices[i * layout.elementSize + k]);
            weights->push_back(w);
        }
    };
    push(pivot);
    for (size_t k = 0; k < layout.elementSize; ++k) {
        if (k != pivot) {
            push(k);
        }
    }
    const double complement = 1.0 - total;
    if (complement != 0.0) {
        indices->push_back(static_cast<int>(layout.transformCount));
        weights->push_back(complement);
    }
}

}  // namespace

std::vector<RrGeoScaledDualQuat>
RrGeoSkinDualQuatPalette(const RrGeoSkinLayout &layout)
{
    std::vector<RrGeoScaledDualQuat> palette;
    palette.reserve(layout.transformCount + 1);
    for (size_t t = 0; t < layout.transformCount; ++t) {
        palette.push_back(
            RrGeoScaledDualQuatFromMatrix(layout.transforms[t]));
    }
    palette.emplace_back();  // identity: the weight complement's influence
    return palette;
}

bool
RrGeoApplyDualQuatSkin(const RrVec3f *in, RrVec3f *out,
                      const RrGeoSkinLayout &layout,
                      const RrGeoScaledDualQuat *palette, size_t paletteSize)
{
    std::vector<int> indices;
    std::vector<double> weights;
    indices.reserve(layout.elementSize + 1);
    weights.reserve(layout.elementSize + 1);
    for (size_t i = 0; i < layout.pointCount; ++i) {
        RrGeoGatherDualQuatInfluences(layout, i, &indices, &weights);
        RrGeoScaledDualQuat blend;
        if (!RrGeoBlendScaledDualQuats(
                palette, paletteSize, indices.data(),
                weights.data(), indices.size(), &blend)) {
            return false;
        }
        out[i] = RrGeoToVec3f(
            RrGeoScaledDualQuatTransformPoint(blend, RrGeoToVec3d(in[i])));
    }
    return true;
}

// Geometry kernels (geometryKernels.cpp): volume, smooth, normals, extent,
// lattice, surface projection, wire/NURBS. RMF sampling, ribbon transport,
// wire binding and the sparse wire apply are unreachable from the revision
// path and stay out.

double
RrGeoBoundVolume(const RrVec3f *points, size_t count)
{
    if (count < 2) {
        return 0.0;
    }
    RrVec3f lo = points[0], hi = points[0];
    for (size_t i = 1; i < count; ++i) {
        for (int a = 0; a < 3; ++a) {
            lo[a] = std::min(lo[a], points[i][a]);
            hi[a] = std::max(hi[a], points[i][a]);
        }
    }
    const RrVec3f d = hi - lo;
    return double(d[0]) * double(d[1]) * double(d[2]);
}

void
RrGeoApplyVolumeCorrect(std::vector<RrVec3f> *points, double referenceVolume,
                       double strength)
{
    if (points->empty() || referenceVolume <= 0.0 || strength <= 0.0) {
        return;
    }
    const double current =
        RrGeoBoundVolume(points->data(), points->size());
    if (current <= 0.0) {
        return;  // degenerate bound: no deterministic correction exists
    }
    const double full = std::cbrt(referenceVolume / current);
    const double scale = 1.0 + std::min(std::max(strength, 0.0), 1.0) *
                                   (full - 1.0);
    RrVec3f centroid(0.0f);
    for (const RrVec3f &p : *points) {
        centroid += p;
    }
    centroid /= float(points->size());
    for (RrVec3f &p : *points) {
        p = centroid + (p - centroid) * float(scale);
    }
}

std::vector<std::vector<int>>
RrGeoBuildAdjacency(size_t pointCount,
                   const std::vector<int> &faceVertexCounts,
                   const std::vector<int> &faceVertexIndices)
{
    std::vector<std::set<int>> adjacency(pointCount);
    size_t offset = 0;
    for (int faceCount : faceVertexCounts) {
        for (int c = 0; c < faceCount; ++c) {
            const size_t ia = offset + c;
            const size_t ib = offset + (c + 1) % faceCount;
            if (ia >= faceVertexIndices.size() ||
                ib >= faceVertexIndices.size()) {
                return {};
            }
            const int a = faceVertexIndices[ia];
            const int b = faceVertexIndices[ib];
            if (a < 0 || b < 0 ||
                static_cast<size_t>(a) >= pointCount ||
                static_cast<size_t>(b) >= pointCount) {
                return {};
            }
            adjacency[a].insert(b);
            adjacency[b].insert(a);
        }
        offset += faceCount;
    }
    std::vector<std::vector<int>> result(pointCount);
    for (size_t i = 0; i < pointCount; ++i) {
        result[i].assign(adjacency[i].begin(), adjacency[i].end());
    }
    return result;
}

bool
RrGeoTransportSurfaceOffsets(const std::vector<RrVec3f> &rest,
                            const std::vector<RrVec3f> &posed,
                            const std::vector<int> &faceCounts,
                            const std::vector<int> &faceIndices,
                            const std::vector<RrVec3f> &deltas,
                            std::vector<RrVec3f> *out)
{
    return RigExecTransportSurfaceOffsetsKernel<RrVec3f, RrVec3d>(
        rest, posed, faceCounts, faceIndices, deltas, out);
}

void
RrGeoApplyLaplacianSmooth(std::vector<RrVec3f> *points,
                         const std::vector<int> &faceVertexCounts,
                         const std::vector<int> &faceVertexIndices,
                         double strength, RigExecSurfaceKernelCache<RrVec3f,RrVec3d> *cache = nullptr)
{
    const double s = std::min(std::max(strength, 0.0), 1.0);
    if (points->empty() || s <= 0.0) {
        return;
    }
    std::vector<std::vector<int>> built;
    const RigExecMeshAdjacency *retained=cache?cache->MeshAdjacency(
        points->size(),faceVertexCounts,faceVertexIndices):nullptr;
    if(cache && !retained) return;
    if(!cache)built=RrGeoBuildAdjacency(points->size(),faceVertexCounts,faceVertexIndices);
    const auto &adjacency=cache?retained->neighbors:built;
    if (adjacency.empty()) {
        return;  // invalid topology: pass through
    }
    const std::vector<RrVec3f> source = *points;
    for (size_t i = 0; i < source.size(); ++i) {
        if (adjacency[i].empty()) {
            continue;
        }
        RrVec3f average(0.0f);
        for (int n : adjacency[i]) {
            average += source[n];
        }
        average /= float(adjacency[i].size());
        (*points)[i] = source[i] + (average - source[i]) * float(s);
    }
}

std::vector<RrVec3f>
RrGeoComputeVertexNormals(const std::vector<RrVec3f> &points,
                         const std::vector<int> &faceVertexCounts,
                         const std::vector<int> &faceVertexIndices)
{
    std::vector<RrVec3f> normals(points.size(), RrVec3f(0.0f));
    std::vector<int> ring;
    size_t offset = 0;
    for (int faceCount : faceVertexCounts) {
        if (faceCount < 3 ||
            offset + static_cast<size_t>(faceCount) >
                faceVertexIndices.size()) {
            return normals;
        }
        ring.clear();
        for (int k = 0; k < faceCount; ++k) {
            const int v = faceVertexIndices[offset + k];
            if (v >= 0 && static_cast<size_t>(v) < points.size()) {
                ring.push_back(v);
            }
        }
        offset += faceCount;
        const size_t n = ring.size();
        if (n < 3) {
            continue;
        }

        RrVec3f faceNormal(0.0f);
        for (size_t k = 0; k < n; ++k) {
            const RrVec3f &p = points[ring[k]];
            const RrVec3f &q = points[ring[(k + 1) % n]];
            faceNormal += RrVec3f((p[1] - q[1]) * (p[2] + q[2]),
                                  (p[2] - q[2]) * (p[0] + q[0]),
                                  (p[0] - q[0]) * (p[1] + q[1]));
        }
        const float faceLength = faceNormal.GetLength();
        if (faceLength < 1e-20f) {
            continue;  // a fully degenerate face contributes nothing
        }
        faceNormal /= faceLength;

        for (size_t k = 0; k < n; ++k) {
            const RrVec3f &p = points[ring[k]];
            const RrVec3f a = points[ring[(k + 1) % n]] - p;
            const RrVec3f b = points[ring[(k + n - 1) % n]] - p;
            const float la = a.GetLength(), lb = b.GetLength();
            float weight = 1.0f;
            if (la > 1e-20f && lb > 1e-20f) {
                const RrVec3f ua = a / la, ub = b / lb;
                weight = std::atan2(RrCross(ua, ub).GetLength(),
                                    RrDot(ua, ub));
            }
            normals[ring[k]] += faceNormal * weight;
        }
    }
    for (RrVec3f &n : normals) {
        const float len = n.GetLength();
        if (len > 1e-12f) {
            n /= len;
        }
    }
    return normals;
}

std::vector<RrVec3f>
RrGeoComputeExtent(const std::vector<RrVec3f> &points,
                  const std::vector<float> &widths)
{
    if (points.empty()) {
        return {};
    }
    RrVec3f lo = points[0], hi = points[0];
    for (size_t i = 0; i < points.size(); ++i) {
        float pad = 0.0f;
        if (widths.size() == points.size()) {
            pad = widths[i] * 0.5f;
        } else if (widths.size() == 1) {
            pad = widths[0] * 0.5f;
        }
        for (int a = 0; a < 3; ++a) {
            lo[a] = std::min(lo[a], points[i][a] - pad);
            hi[a] = std::max(hi[a], points[i][a] + pad);
        }
    }
    return {lo, hi};
}

// The shared lattice kernel (latticeKernel.h); \p cache, the revision's own,
// retains the bind's Bernstein factors across frames.
void
RrGeoApplyLattice(std::vector<RrVec3f> *points,
                 const std::vector<RrVec3f> &restPoints,
                 const std::vector<RrVec3f> &restCage,
                 const std::vector<RrVec3f> &posedCage,
                 const RrVec3i &divisions,
                 RigExecSurfaceKernelCache<RrVec3f, RrVec3d> *cache)
{
    RigExecApplyLatticeKernel(
        points, restPoints.data(), restPoints.size(), restCage.data(),
        restCage.size(), posedCage.data(), posedCage.size(), divisions[0],
        divisions[1], divisions[2], cache);
}

RrVec3f
RrGeoClosestPointOnTriangle(const RrVec3f &p, const RrVec3f &a,
                           const RrVec3f &b, const RrVec3f &c)
{
    // Ericson, Real-Time Collision Detection, 5.1.5.
    const RrVec3f ab = b - a, ac = c - a, ap = p - a;
    const float d1 = RrDot(ab, ap), d2 = RrDot(ac, ap);
    if (d1 <= 0 && d2 <= 0) return a;
    const RrVec3f bp = p - b;
    const float d3 = RrDot(ab, bp), d4 = RrDot(ac, bp);
    if (d3 >= 0 && d4 <= d3) return b;
    const float vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) {
        const float v = d1 / (d1 - d3);
        return a + ab * v;
    }
    const RrVec3f cp = p - c;
    const float d5 = RrDot(ab, cp), d6 = RrDot(ac, cp);
    if (d6 >= 0 && d5 <= d6) return c;
    const float vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) {
        const float w = d2 / (d2 - d6);
        return a + ac * w;
    }
    const float va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
        const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return b + (c - b) * w;
    }
    const float denom = 1.0f / (va + vb + vc);
    const float v = vb * denom, w = vc * denom;
    return a + ab * v + ac * w;
}

void
RrGeoApplySurfaceProject(std::vector<RrVec3f> *points,
                        const std::vector<RrVec3f> &surfacePoints,
                        const std::vector<int> &faceVertexCounts,
                        const std::vector<int> &faceVertexIndices,
                        double weight)
{
    const double w = std::min(std::max(weight, 0.0), 1.0);
    if (points->empty() || surfacePoints.empty() || w <= 0.0) {
        return;
    }
    for (RrVec3f &p : *points) {
        float bestDistSq = std::numeric_limits<float>::max();
        RrVec3f best = p;
        size_t offset = 0;
        for (int faceCount : faceVertexCounts) {
            for (int c = 1; c + 1 < faceCount; ++c) {
                const size_t i2 = offset + c + 1;
                if (i2 >= faceVertexIndices.size()) {
                    return;
                }
                const int ia = faceVertexIndices[offset];
                const int ib = faceVertexIndices[offset + c];
                const int ic = faceVertexIndices[i2];
                if (ia < 0 || ib < 0 || ic < 0 ||
                    size_t(ia) >= surfacePoints.size() ||
                    size_t(ib) >= surfacePoints.size() ||
                    size_t(ic) >= surfacePoints.size()) {
                    continue;
                }
                const RrVec3f q = RrGeoClosestPointOnTriangle(
                    p, surfacePoints[ia], surfacePoints[ib],
                    surfacePoints[ic]);
                const float distSq = (q - p).GetLengthSq();
                if (distSq < bestDistSq) {
                    bestDistSq = distSq;
                    best = q;
                }
            }
            offset += faceCount;
        }
        p = p + (best - p) * float(w);
    }
}

struct RrGeoNurbsCurve {
    const std::vector<RrVec3f> *points = nullptr;
    int order = 0;
    const std::vector<double> *knots = nullptr;

    bool IsValid() const
    {
        if (!points || !knots || order < 2) {
            return false;
        }
        const size_t n = points->size();
        return order <= 16 && n >= size_t(order) &&
               knots->size() == n + size_t(order) &&
               (*knots)[size_t(order) - 1] < (*knots)[n];
    }

    double DomainStart() const
    {
        return (*knots)[size_t(order) - 1];
    }

    double DomainEnd() const
    {
        return (*knots)[points->size()];
    }

    RrVec3f Evaluate(double u) const
    {
        const std::vector<double> &k = *knots;
        const std::vector<RrVec3f> &cv = *points;
        const int p = order - 1;
        const size_t n = cv.size();
        u = std::min(std::max(u, DomainStart()), DomainEnd());
        size_t s = size_t(p);
        while (s + 1 < n && k[s + 1] <= u) {
            ++s;
        }
        RrVec3d d[16];
        const int count = std::min(order, 16);
        for (int j = 0; j < count; ++j) {
            d[j] = RrGeoToVec3d(cv[s - size_t(p) + size_t(j)]);
        }
        for (int r = 1; r <= p && r < 16; ++r) {
            for (int j = p; j >= r; --j) {
                const size_t i = s - size_t(p) + size_t(j);
                const double denom = k[i + size_t(p) + 1 - size_t(r)] - k[i];
                const double a = denom > 0.0 ? (u - k[i]) / denom : 0.0;
                d[j] = d[j - 1] * (1.0 - a) + d[j] * a;
            }
        }
        return RrGeoToVec3f(d[p]);
    }
};

struct RrGeoWireBasis {
    // Per control point, the (weighted-point ordinal, coefficient) pairs.
    std::vector<std::vector<std::pair<uint32_t, float>>> byControlPoint;
};

// RigExecWireBasisInputsAreUsable: the basis build's checks before it builds.
bool
RrGeoWireBasisInputsAreUsable(const RrVec2f *bindCoords, size_t bindCount,
                              size_t meshPointCount, size_t indexCount,
                              int order, const std::vector<double> &knots,
                              size_t controlPointCount)
{
    const size_t n = controlPointCount;
    return bindCoords && order >= 1 && order <= 16 && n >= size_t(order) &&
           knots.size() == n + size_t(order) &&
           knots[size_t(order - 1)] < knots[n] &&
           (bindCount == meshPointCount || bindCount == indexCount);
}

bool
RrGeoBuildWireBasis(const RrVec2f *bindCoords, size_t bindCount,
                   size_t meshPointCount, const std::vector<int> &indices,
                   int order, const std::vector<double> &knots,
                   size_t controlPointCount, double dropoffDistance,
                   RrGeoWireBasis *basis)
{
    const size_t n = controlPointCount;
    if (!basis ||
        !RrGeoWireBasisInputsAreUsable(bindCoords, bindCount, meshPointCount,
                                       indices.size(), order, knots, n)) {
        return false;
    }
    const bool parallel = bindCount == indices.size();
    const int p = order - 1;
    const double u0 = knots[size_t(p)];
    const double u1 = knots[n];
    basis->byControlPoint.assign(n, {});
    double left[16], right[16], N[16];
    for (size_t k = 0; k < indices.size(); ++k) {
        const int index = indices[k];
        if (index < 0 || size_t(index) >= meshPointCount) {
            return false;
        }
        const RrVec2f &bind = bindCoords[parallel ? k : size_t(index)];
        double f = 1.0;
        if (dropoffDistance > 0.0) {
            const double s =
                std::min(std::max(double(bind[1]) / dropoffDistance, 0.0), 1.0);
            f = 1.0 - s * s * (3.0 - 2.0 * s);
        }
        if (f <= 0.0) {
            continue;
        }
        const double u = std::min(std::max(double(bind[0]), u0), u1);
        size_t span = size_t(p);
        while (span + 1 < n && knots[span + 1] <= u) {
            ++span;
        }
        N[0] = 1.0;
        for (int j = 1; j <= p; ++j) {
            left[j] = u - knots[span + 1 - size_t(j)];
            right[j] = knots[span + size_t(j)] - u;
            double saved = 0.0;
            for (int r = 0; r < j; ++r) {
                const double denom = right[r + 1] + left[j - r];
                const double temp = denom != 0.0 ? N[r] / denom : 0.0;
                N[r] = saved + right[r + 1] * temp;
                saved = left[j - r] * temp;
            }
            N[j] = saved;
        }
        for (int r = 0; r <= p; ++r) {
            const double c = f * N[r];
            if (c != 0.0) {
                basis->byControlPoint[span - size_t(p) + size_t(r)].emplace_back(
                    uint32_t(k), float(c));
            }
        }
    }
    return true;
}

bool
RrGeoApplyWireBasis(std::vector<RrVec3f> *points,
                   const RrGeoWireBasis &basis,
                   const std::vector<int> &indices,
                   const std::vector<float> &weights,
                   const std::vector<RrVec3f> &restControlPoints,
                   const std::vector<RrVec3f> &posedControlPoints)
{
    const size_t n = basis.byControlPoint.size();
    if (!points || restControlPoints.size() != n ||
        posedControlPoints.size() != n || indices.size() != weights.size()) {
        return false;
    }
    RrVec3f *data = points->data();
    for (size_t j = 0; j < n; ++j) {
        const RrVec3f delta = posedControlPoints[j] - restControlPoints[j];
        if (delta == RrVec3f(0.0f)) {
            continue;  // a control point at rest moves nothing
        }
        for (const auto &[k, coefficient] : basis.byControlPoint[j]) {
            data[size_t(indices[k])] += delta * (coefficient * weights[k]);
        }
    }
    return true;
}

// RigExecWireInputsAreUsable: RrGeoApplyWire's checks before it writes.
bool
RrGeoWireInputsAreUsable(const RrGeoNurbsCurve &restCurve,
                         const RrGeoNurbsCurve &posedCurve,
                         const RrVec2f *bindCoords, size_t bindCount,
                         size_t pointCount)
{
    return bindCoords && restCurve.IsValid() && posedCurve.IsValid() &&
           restCurve.order == posedCurve.order &&
           restCurve.points->size() == posedCurve.points->size() &&
           !(*restCurve.knots != *posedCurve.knots) &&
           bindCount == pointCount;
}

bool
RrGeoApplyWire(std::vector<RrVec3f> *points,
              const RrGeoNurbsCurve &restCurve,
              const RrGeoNurbsCurve &posedCurve,
              const RrVec2f *bindCoords, size_t bindCount,
              double dropoffDistance, size_t begin, size_t end, const std::vector<RrVec3f> *restEvaluations = nullptr)
{
    if (!points ||
        !RrGeoWireInputsAreUsable(restCurve, posedCurve, bindCoords,
                                  bindCount, points->size())) {
        return false;
    }
    end = std::min(end, points->size());
    for (size_t i = begin; i < end; ++i) {
        const double u = bindCoords[i][0];
        const double d = bindCoords[i][1];
        double f = 1.0;
        if (dropoffDistance > 0.0) {
            const double s = std::min(std::max(d / dropoffDistance, 0.0), 1.0);
            f = 1.0 - s * s * (3.0 - 2.0 * s);
        }
        if (f <= 0.0) {
            continue;
        }
        const RrVec3f delta = posedCurve.Evaluate(u) -
            (restEvaluations ? (*restEvaluations)[i] : restCurve.Evaluate(u));
        (*points)[i] += delta * float(f);
    }
    return true;
}

struct RrGeoSkinTransformsView {
    const RrMat4d *transforms = nullptr;
    size_t transformCount = 0;
    const float *rows = nullptr;
    const RrGeoScaledDualQuat *palette = nullptr;
    size_t paletteSize = 0;
};

RrGeoSkinTransformsView
RrGeoSkinTransformsOf(const RrGeoMoverParameters &p)
{
    RrGeoSkinTransformsView view;
    view.transforms = p.skinTransforms.data();
    view.transformCount = p.skinTransforms.size();
    return view;
}

RrGeoSkinLayout
RrGeoSkinLayoutForPacket(const RrGeoMoverParameters &p,
                        const RrGeoSkinTransformsView &transforms,
                        size_t pointCount)
{
    RrGeoSkinLayout layout;
    layout.transforms = transforms.transforms;
    layout.transformCount = transforms.transformCount;
    const RrGeoSkinTopology *const topology = p.skinTopology.get();
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
RrGeoSkinLayoutIsUsable(const RrGeoMoverParameters &p, size_t pointCount)
{
    const size_t transformCount = p.skinTransforms.size();
    if (const RrGeoSkinTopology *const topology = p.skinTopology.get()) {
        const size_t elementSize =
            topology->elementSize < 1 ? 0 : size_t(topology->elementSize);
        return topology->validated &&
               topology->influenceCount == transformCount &&
               topology->indices.size() == pointCount * elementSize;
    }
    if (p.skinWeights.size() != p.skinIndices.size()) {
        return false;  // cardinality mismatch fails atomically
    }
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
RrGeoSkinTransformsAreUsable(const RrMat4d *transforms, size_t count)
{
    if (!transforms || count == 0) {
        return false;
    }
    for (size_t t = 0; t < count; ++t) {
        const RrMat4d &m = transforms[t];
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

// RigExecSkinMethodOf: the arithmetic the range kernel dispatches on.
enum class RrGeoSkinMethod : uint8_t { Unknown, ClassicLinear, DualQuaternion };

RrGeoSkinMethod
RrGeoSkinMethodOf(const RrGeoMoverParameters &p)
{
    if (p.skinningMethod == "classicLinear") {
        return RrGeoSkinMethod::ClassicLinear;
    }
    if (p.skinningMethod == "dualQuaternion") {
        return RrGeoSkinMethod::DualQuaternion;
    }
    return RrGeoSkinMethod::Unknown;
}

bool
RrGeoApplySkinKernelRange(const RrGeoMoverParameters &p,
                         const RrGeoSkinTransformsView &transforms,
                         size_t begin, size_t end, std::vector<RrVec3f> *pts,
                         bool useSimd)
{
    const RrGeoSkinMethod method = RrGeoSkinMethodOf(p);
    const RrGeoSkinLayout layout =
        RrGeoSkinLayoutForPacket(p, transforms, pts->size());
    RrGeoSkinLayout part = layout;
    part.indices = layout.indices + begin * layout.elementSize;
    part.weights = layout.weights + begin * layout.elementSize;
    part.indexCount = (end - begin) * layout.elementSize;
    part.pointCount = end - begin;
    RrVec3f *const points = pts->data();

    if (method == RrGeoSkinMethod::ClassicLinear) {
        if (useSimd) {
            RrGeoApplyLinearBlendSkinSimd(
                points + begin, points + begin, part, transforms.rows);
        } else {
            RrGeoApplyLinearBlendSkin(
                points + begin, points + begin, part);
        }
        return true;
    }
    if (method == RrGeoSkinMethod::DualQuaternion) {
        std::vector<RrGeoScaledDualQuat> local;
        const RrGeoScaledDualQuat *palette = transforms.palette;
        size_t paletteSize = transforms.paletteSize;
        if (!palette) {
            local = RrGeoSkinDualQuatPalette(layout);
            palette = local.data();
            paletteSize = local.size();
        }
        return RrGeoApplyDualQuatSkin(points + begin, points + begin, part,
                                      palette, paletteSize);
    }
    return false;
}

bool
RrGeoApplySkinKernelWithTransforms(
    const RrGeoMoverParameters &p,
    const RrGeoSkinTransformsView &transforms, std::vector<RrVec3f> *pts,
    bool useSimd)
{
    const size_t count = pts->size();
    if (!RrGeoSkinLayoutIsUsable(p, count)) {
        return false;
    }
    if (!p.skinTopology &&
        !RrGeoSkinTransformsAreUsable(transforms.transforms,
                                      transforms.transformCount)) {
        return false;
    }

    // The runtime runs serially; a point range is an independent
    // sub-problem, so the serial call is the parallel loop's answer.
    return RrGeoApplySkinKernelRange(p, transforms, 0, count, pts, useSimd);
}

// One sample as the gather consumes it. `points` (the dense points, empty
// for a sparse sample) is Open's conversion and is never null.
struct RrGeoBlendSampleData {
    float activation = 1.0f;
    std::shared_ptr<const RrGeoBlendLayout> layout;
    const std::vector<RrVec3f> *points = nullptr;
};

struct RrGeoBlendChannel {
    float weight = 0.0f;
    std::vector<RrGeoBlendSampleData> samples;
};

void
RrGeoAccumulateBlendSample(const RrGeoBlendSampleData &sample,
                          const std::vector<RrVec3f> &base, float scale,
                          std::vector<RrVec3f> *deltas)
{
    if (sample.layout) {
        const RrGeoBlendLayout &layout = *sample.layout;
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
        (*deltas)[i] += ((*sample.points)[i] - base[i]) * scale;
    }
}

bool
RrGeoSumBlendChannels(const std::vector<RrGeoBlendChannel> &channels,
                     const std::vector<RrVec3f> &base,
                     std::vector<RrVec3f> *deltas)
{
    if (!deltas || base.empty()) {
        return false;
    }
    deltas->assign(base.size(), RrVec3f(0.0f));

    for (const RrGeoBlendChannel &channel : channels) {
        if (channel.samples.empty() || !std::isfinite(channel.weight)) {
            return false;  // structural error: fails atomically
        }
        for (size_t k = 0; k < channel.samples.size(); ++k) {
            const RrGeoBlendSampleData &sample = channel.samples[k];
            const bool shapeOk =
                sample.layout
                    ? (sample.layout->valid &&
                       sample.layout->pointCount == base.size() &&
                       sample.points->empty())
                    : sample.points->size() == base.size();
            if (!std::isfinite(sample.activation) ||
                sample.activation <= 0 || !shapeOk ||
                (k > 0 && sample.activation ==
                              channel.samples[k - 1].activation)) {
                return false;
            }
        }
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
        const RrGeoBlendSampleData *loSample =
            hi > 0 ? &channel.samples[hi - 1] : nullptr;
        const RrGeoBlendSampleData &hiSample = channel.samples[hi];

        if (!hiSample.layout && (!loSample || !loSample->layout)) {
            const std::vector<RrVec3f> *lo =
                loSample ? loSample->points : nullptr;
            const std::vector<RrVec3f> &hiPts = *hiSample.points;
            for (size_t i = 0; i < base.size(); ++i) {
                const RrVec3f dHi = hiPts[i] - base[i];
                const RrVec3f dLo = lo ? (*lo)[i] - base[i] : RrVec3f(0.0f);
                (*deltas)[i] += dLo + (dHi - dLo) * t;
            }
            continue;
        }

        if (loSample) {
            RrGeoAccumulateBlendSample(*loSample, base, 1.0f - t, deltas);
        }
        RrGeoAccumulateBlendSample(hiSample, base, t, deltas);
    }
    return true;
}

// The blend-shape kernel's validation, the one definition it runs first and
// RrGeoRevisionKernelAcceptance decides by (moverGraph.cpp,
// _BlendShapeKernelAccepts): one delta per point and an envelope that
// resolves, into \p envelope when given.
bool
RrGeoBlendShapeKernelAccepts(const RrGeoMoverParameters &p, size_t count,
                             std::vector<float> *envelope)
{
    if (p.blendDeltas.size() != count) {
        return false;
    }
    if (RrGeoEnvelopeIsFullStrength(p.weights)) {
        return true;
    }
    return envelope ? p.weights.ResolveAll(count, envelope)
                    : p.weights.ResolvesAll(count);
}

bool
RrGeoApplyBlendShapeKernel(const RrGeoMoverParameters &p,
                           std::vector<RrVec3f> *pts, RigExecSurfaceKernelCache<RrVec3f,RrVec3d> *cache = nullptr)
{
    const size_t count = pts->size();
    std::vector<float> envelope;
    if (!RrGeoBlendShapeKernelAccepts(p, count, &envelope)) {
        return false;
    }
    const bool full=RrGeoEnvelopeIsFullStrength(p.weights);

    std::vector<RrVec3f> transported;
    const std::vector<RrVec3f> *deltas = &p.blendDeltas;
    if (p.blendSurfaceFrame) {
        const auto *rest=cache?cache->TransportRest(p.restPoints,p.topologyCounts,p.topologyIndices):nullptr;
        const bool applied=cache
            ? rest && RigExecApplyTransportWithRestData(*pts,p.topologyCounts,p.topologyIndices,p.blendDeltas,*rest,&transported)
            : RrGeoTransportSurfaceOffsets(p.restPoints,*pts,p.topologyCounts,p.topologyIndices,p.blendDeltas,&transported);
        if(!applied)return false;
        deltas = &transported;
    }

    RrVec3f *const points = pts->data();
    const RrVec3f *const delta = deltas->data();
    const float *const weight = envelope.data();
    // The runtime runs serially; a point range is an independent
    // sub-problem, so the serial loop is the parallel loop's answer.
    for (size_t i = 0; i < count; ++i) {
        const RrVec3f preceding = points[i];
        points[i] = RrGeoBlendEnvelope(
            preceding, preceding + delta[i], full ? 1.0f : weight[i]);
    }
    return true;
}

bool
RrGeoApplyDerivedKernel(int op, const RrGeoMoverParameters &p,
                       std::vector<RrVec3f> *pts,size_t expectedCount=SIZE_MAX)
{
    const bool extent = op == RrGeoOpRecomputeExtent;
    std::vector<RrVec3f> values =
        extent ? RrGeoComputeExtent(p.auxPoints, p.widths)
               : RrGeoComputeVertexNormals(p.auxPoints, p.topologyCounts,
                                           p.topologyIndices);
    if (values.empty() || (extent && values.size() != 2)) {
        return false;
    }

    if ((expectedCount==SIZE_MAX?pts->size():expectedCount) != values.size()) {
        return false;
    }
    if(RrGeoEnvelopeIsFullStrength(p.weights)){*pts=std::move(values);return true;}
    std::vector<float> envelope;
    if (!p.weights.ResolveAll(values.size(), &envelope)) {
        return false;
    }

    for (size_t i = 0; i < values.size(); ++i) {
        (*pts)[i] = RrGeoBlendEnvelope((*pts)[i], values[i], envelope[i]);
    }
    return true;
}

bool
RrGeoWireTakesSparseEnvelope(const RrGeoWeightPacket &w)
{
    return RrGeoEnvelopeIsSparseWalk(w);
}

// The checks the wire operation makes before it builds or applies anything
// (moverGraph.cpp, _WireKernelPrefix).
bool
RrGeoWireKernelPrefix(const RrGeoMoverParameters &p, size_t count)
{
    if (p.auxPoints.size() != p.restPoints.size()) {
        return false;
    }
    const RrGeoNurbsCurve rest{&p.restPoints, p.curveOrder, &p.curveKnots};
    const RrGeoNurbsCurve posed{&p.auxPoints, p.curveOrder, &p.curveKnots};
    if (!rest.IsValid() || !posed.IsValid()) {
        return false;
    }
    if (RrGeoWireTakesSparseEnvelope(p.weights)) {
        return RrGeoSparseEnvelopeIsUsable(p.weights, count);
    }
    // A sparse bind table needs a sparse envelope.
    return p.wireBindCoords.size() == count;
}

// The wire operation's answer (moverGraph.cpp, _WireKernelAcceptance): its
// prefix, then the checks of the basis build or the dense walk. Deferred for
// an empty bind table, whose answer is its storage, which a remembered basis
// does not consult.
RrGeoAcceptance
RrGeoWireKernelAcceptance(const RrGeoMoverParameters &p, size_t count)
{
    if (!RrGeoWireKernelPrefix(p, count)) {
        return RrGeoAcceptance::Refuses;
    }
    if (p.wireBindCoords.empty()) {
        return RrGeoAcceptance::Deferred;
    }
    if (RrGeoWireTakesSparseEnvelope(p.weights)) {
        return RrGeoWireBasisInputsAreUsable(
                   p.wireBindCoords.data(), p.wireBindCoords.size(), count,
                   p.weights.Indices().size(), p.curveOrder, p.curveKnots,
                   p.restPoints.size())
            ? RrGeoAcceptance::Applies
            : RrGeoAcceptance::Refuses;
    }
    const RrGeoNurbsCurve rest{&p.restPoints, p.curveOrder, &p.curveKnots};
    const RrGeoNurbsCurve posed{&p.auxPoints, p.curveOrder, &p.curveKnots};
    return RrGeoWireInputsAreUsable(rest, posed, p.wireBindCoords.data(),
                                    p.wireBindCoords.size(), count)
        ? RrGeoAcceptance::Applies
        : RrGeoAcceptance::Refuses;
}

// RigExecRevisionTakesSeparateBlend: the ops RrGeoRunRevisionKernel blends
// back over the entering points afterwards.
bool
RrGeoRevisionTakesSeparateBlend(int op, const RrGeoWeightPacket &w)
{
    return !(op == RrGeoOpMatrix || op == RrGeoOpBlendShape ||
             op == RrGeoOpRecomputeNormals || op == RrGeoOpRecomputeExtent ||
             (op == RrGeoOpWire && RrGeoWireTakesSparseEnvelope(w)));
}

// The packet check every revision kernel makes first.
bool
RrGeoPacketMatches(int op, const RrGeoMoverParameters &p)
{
    const char *const kind = RrGeoKindToken(op);
    return p.valid && kind && p.kind == kind;
}

// RigExecRevisionKernelAcceptance: RrGeoRunRevisionKernel's answer over
// \p count entering points.
RrGeoAcceptance
RrGeoRevisionKernelAcceptance(int op, const RrGeoMoverParameters &p,
                              size_t count)
{
    if (!RrGeoPacketMatches(op, p)) {
        return RrGeoAcceptance::Refuses;
    }
    switch (op) {
    case RrGeoOpMatrix:
        return RrGeoMatrixKernelAccepts(p, count, nullptr)
            ? RrGeoAcceptance::Applies
            : RrGeoAcceptance::Refuses;
    case RrGeoOpBlendShape:
        if (!RrGeoBlendShapeKernelAccepts(p, count, nullptr)) {
            return RrGeoAcceptance::Refuses;
        }
        // The surface-frame transport reads the entering points.
        return p.blendSurfaceFrame ? RrGeoAcceptance::Deferred
                                   : RrGeoAcceptance::Applies;
    case RrGeoOpWire: {
        const RrGeoAcceptance wire = RrGeoWireKernelAcceptance(p, count);
        if (wire == RrGeoAcceptance::Refuses) {
            return RrGeoAcceptance::Refuses;
        }
        // The "apply once" envelope resolves at the full count or the
        // revision fails, whatever the kernel answered.
        if (RrGeoRevisionTakesSeparateBlend(op, p.weights) &&
            !RrGeoEnvelopeIsFullStrength(p.weights) &&
            !p.weights.ResolvesAll(count)) {
            return RrGeoAcceptance::Refuses;
        }
        return wire;
    }
    default:
        return RrGeoAcceptance::Deferred;
    }
}


bool RrGeoPointBitsEqual(const std::vector<RrVec3f> &a,const std::vector<RrVec3f> &b)
{
    static_assert(sizeof(RrVec3f)==3*sizeof(float),"points compare as packed floats");
    return a.size()==b.size() &&
        (a.empty() || std::memcmp(a.data(),b.data(),a.size()*sizeof(RrVec3f))==0);
}
// The program's CopyMovedPoints: \p output takes \p staging's points, block
// by block where the bytes differ; returns whether any did.
bool RrGeoCopyMovedPoints(const std::vector<RrVec3f> &staging,std::vector<RrVec3f> *output)
{
    if(output->size()!=staging.size()) { *output=staging; return true; }
    constexpr size_t kBlock=1024;
    bool moved=false;
    for(size_t begin=0;begin<staging.size();begin+=kBlock) {
        const size_t count=std::min(kBlock,staging.size()-begin);
        if(std::memcmp(staging.data()+begin,output->data()+begin,count*sizeof(RrVec3f))!=0) {
            std::copy(staging.begin()+long(begin),staging.begin()+long(begin+count),
                      output->begin()+long(begin));
            moved=true;
        }
    }
    return moved;
}
struct RrGeoWireBasisEntry {
    std::vector<RrVec2f> binds;
    std::vector<int> indices;
    std::vector<double> knots;
    int order = 0;
    size_t controlPoints = 0;
    size_t meshPoints = 0;
    double dropoff = 0.0;
    std::shared_ptr<const RrGeoWireBasis> basis;
};

uint64_t
RrGeoHashBytes(uint64_t h, const void *data, size_t size)
{
    const unsigned char *bytes = static_cast<const unsigned char *>(data);
    for (size_t i = 0; i < size; ++i) {
        h = (h ^ bytes[i]) * 1099511628211ull;
    }
    return h;
}

// The wire basis cache, keyed by the inputs' content with the full inputs
// compared on a hit. The runtime is serial, so no lock; the cache lives
// in the geometry scratch instead of a process static, which changes no
// answer (a rebuild is pure).
std::shared_ptr<const RrGeoWireBasis>
RrGeoCachedWireBasis(const RrGeoMoverParameters &p,
                     const std::vector<int> &indices, size_t meshPoints,
                     std::unordered_map<uint64_t, RrGeoWireBasisEntry> *cache,
                     std::shared_ptr<const RrGeoWireBasisEntry> *last = nullptr)
{
    const size_t controlPoints = p.restPoints.size();
    const auto matches = [&](const RrGeoWireBasisEntry &e) {
        return e.order == p.curveOrder && e.controlPoints == controlPoints &&
               e.meshPoints == meshPoints && !std::memcmp(&e.dropoff,&p.dropoffDistance,sizeof(double)) &&
               e.knots.size()==p.curveKnots.size() &&
               (e.knots.empty() || !std::memcmp(e.knots.data(),p.curveKnots.data(),e.knots.size()*sizeof(double))) && e.indices == indices &&
               e.binds.size() == p.wireBindCoords.size() &&
               std::equal(e.binds.begin(), e.binds.end(),p.wireBindCoords.cbegin(),
                   [](const RrVec2f&a,const RrVec2f&b){for(int axis=0;axis<2;++axis){const float x=a[axis],y=b[axis];if(std::memcmp(&x,&y,sizeof(float)))return false;}return true;});
    };
    if(last && *last && matches(**last)) return (*last)->basis;
    uint64_t h = 1469598103934665603ull;
    if(!last) {
    h = RrGeoHashBytes(h, p.wireBindCoords.data(),
                       p.wireBindCoords.size() * sizeof(RrVec2f));
    h = RrGeoHashBytes(h, indices.data(), indices.size() * sizeof(int));
    h = RrGeoHashBytes(h, p.curveKnots.data(),
                       p.curveKnots.size() * sizeof(double));
    h = RrGeoHashBytes(h, &p.curveOrder, sizeof(p.curveOrder));
    h = RrGeoHashBytes(h, &controlPoints, sizeof(controlPoints));
    h = RrGeoHashBytes(h, &meshPoints, sizeof(meshPoints));
    h = RrGeoHashBytes(h, &p.dropoffDistance, sizeof(p.dropoffDistance));

    }
    if(!last) {
        const auto it = cache->find(h);
        if (it != cache->end() && matches(it->second)) {
            if(last)*last=std::make_shared<const RrGeoWireBasisEntry>(it->second);
            return it->second.basis;
        }
    }
    auto basis = std::make_shared<RrGeoWireBasis>();
    if (!RrGeoBuildWireBasis(p.wireBindCoords.data(),
                             p.wireBindCoords.size(), meshPoints, indices,
                             p.curveOrder, p.curveKnots, controlPoints,
                             p.dropoffDistance, basis.get())) {
        return nullptr;
    }
    RrGeoWireBasisEntry entry;
    entry.binds.assign(p.wireBindCoords.cbegin(), p.wireBindCoords.cend());
    entry.indices = indices;
    entry.knots = p.curveKnots;
    entry.order = p.curveOrder;
    entry.controlPoints = controlPoints;
    entry.meshPoints = meshPoints;
    entry.dropoff = p.dropoffDistance;
    entry.basis = basis;
    if (!last && cache->size() > 512) {
        cache->clear();
    }
    if(last)*last=std::make_shared<const RrGeoWireBasisEntry>(entry);
    if(!last)(*cache)[h] = std::move(entry);
    return basis;
}

RrMat4d
RrGeoMeasureInSpace(const RrMat4d &transform, const RrMat4d &space)
{
    return RrMeasureInSpace(transform, space);
}

// inverse(space) * transform: the same offset, conjugated into the space's
// CURRENT frame. The USD-side twin is RigExecMeasureInPosedSpace in
// moverGraph.h; what rigExec:driverDeltaFrame = "posed" asks a wire for.
// Same arithmetic in the same order, so the two agree bit for bit.
RrMat4d
RrGeoMeasureInPosedSpace(const RrMat4d &transform, const RrMat4d &space)
{
    RrMat4d m = space.GetInverse() * transform;
    m[0][3] = 0.0;
    m[1][3] = 0.0;
    m[2][3] = 0.0;
    m[3][3] = 1.0;
    return m;
}

// The scale a posed frame carries: the length of each column of its linear
// part. The USD-side twin is RigExecFrameScale in moverGraph.h and the two
// must agree to the last bit, so the arithmetic here is the same arithmetic
// in the same order. A frame nothing has scaled returns (1, 1, 1) and every
// caller skips its correction outright, which is what keeps an unscaled rig
// bit for bit what it was.
RrVec3d
RrGeoFrameScale(const RrMat4d &frame)
{
    RrVec3d scale(1.0, 1.0, 1.0);
    for (size_t c = 0; c < 3; ++c) {
        const double length =
            RrVec3d(frame[0][c], frame[1][c], frame[2][c]).GetLength();
        if (length > 0.0) {
            scale[c] = length;
        }
    }
    return scale;
}

// The offset a cluster applies, in the frame its POINTS are already in.
// The USD-side twin is RigExecClusterInPointFrame in moverGraph.h; the two
// are compared bit for bit, so this is the same arithmetic in the same
// order, including which of the two branches is taken.
//
// \p carry is rigExec:space's rest->pose map -- the rig's own carry,
// normally a TRS master's -- and conjugating by it is what makes a
// post-skin cluster ride a moved master instead of shearing the mesh by
// T*R - T. A null \p carry falls back to conjugating by the space's own
// SCALE, which is all the correction there was before a carry could be
// named; the two never compose, because a named carry already holds the
// master's scale. A rig naming no space under an unscaled master measures
// (1, 1, 1) and \p m comes back untouched.
RrMat4d
RrGeoClusterInPointFrame(const RrMat4d &m, const RrMat4d &space,
                         bool posedPoints, const RrMat4d *carry)
{
    if (!posedPoints) {
        return m;
    }
    RrMat4d out;
    if (carry) {
        out = carry->GetInverse() * m * *carry;
    } else {
        const RrVec3d k = RrGeoFrameScale(space);
        if (k == RrVec3d(1.0, 1.0, 1.0)) {
            return m;
        }
        RrMat4d scale(1.0), unscale(1.0);
        scale.SetScale(k);
        unscale.SetScale(RrVec3d(1.0 / k[0], 1.0 / k[1], 1.0 / k[2]));
        out = unscale * m * scale;
    }
    out[0][3] = 0.0;
    out[1][3] = 0.0;
    out[2][3] = 0.0;
    out[3][3] = 1.0;
    return out;
}

// Carries a transform-driven wire's two control polygons into the frame
// its POINTS are already in. The USD-side twin is RigExecCarryWireCurves in
// moverGraph.h: the applier adds posed(u) - rest(u), a difference computed
// in the asset's uncarried frame and added to points a skin has already
// carried by the master, so both polygons are transformed by the carry and
// the difference becomes d * M_linear. NURBS evaluation commutes with an
// affine map, so the bind table stays valid. Called only when a carry was
// NAMED, and then in place of the scale-only correction, which the carry
// already holds.
void
RrGeoCarryWireCurves(std::vector<RrVec3f> *rest, std::vector<RrVec3f> *posed,
                     const RrMat4d &carry)
{
    for (RrVec3f &p : *rest) {
        p = RrGeoToVec3f(carry.TransformAffine(RrGeoToVec3d(p)));
    }
    for (RrVec3f &p : *posed) {
        p = RrGeoToVec3f(carry.TransformAffine(RrGeoToVec3d(p)));
    }
}

// A plugin mover through the kernel the host installed. With none it is a
// no-op -- Execute names it -- and the preceding points stand.
bool
RrGeoApplyExternal(const RrGeoMoverParameters &p, std::vector<RrVec3f> *pts)
{
    if (!p.external || !p.external->state) {
        return true;
    }
    if (!p.externalFrame && p.externalInputs.empty()) {
        return false;
    }
    std::vector<float> xyz(pts->size() * 3);
    for (size_t i = 0; i < pts->size(); ++i) {
        for (size_t a = 0; a < 3; ++a) {
            xyz[i * 3 + a] = (*pts)[i][a];
        }
    }
    std::vector<std::vector<float>> phasedXyz(p.externalPhased.size());
    std::vector<RigExecExternalPhasedPoints> phased(p.externalPhased.size());
    for (size_t k = 0; k < phased.size(); ++k) {
        phased[k].path = p.externalPhasedPaths[k].c_str();
        if (!p.externalPhasedHave[k]) {
            continue;
        }
        const std::vector<RrVec3f> &points = p.externalPhased[k];
        phasedXyz[k].resize(points.size() * 3);
        for (size_t i = 0; i < points.size(); ++i) {
            for (size_t a = 0; a < 3; ++a) {
                phasedXyz[k][i * 3 + a] = points[i][a];
            }
        }
        phased[k].xyz = phasedXyz[k].data();
        phased[k].count = points.size();
    }
    std::vector<RigExecExternalInputValue> inputs(p.externalInputs.size());
    std::vector<std::vector<const char *>> tokenArrays(p.externalInputs.size());
    for(size_t k=0;k<inputs.size();++k) {
        const auto &source=p.externalInputs[k]; auto &input=inputs[k];
        input.type=source.type; input.hasValue=source.have; input.count=source.count;
        if(!source.have) continue;
        if(source.type==uint8_t(RigExecWireInputTag::Token)) input.data=source.token.c_str();
        else if(source.type==uint8_t(RigExecWireInputTag::TokenArray)) {
            for(const auto &token:source.tokens) tokenArrays[k].push_back(token.c_str());
            input.data=tokenArrays[k].data();
        } else input.data=source.bytes.data();
    }
    const uint8_t *frameData=p.externalFrame?p.externalFrame->data():nullptr;
    const size_t frameSize=p.externalFrame?p.externalFrame->size():0;
    if (!p.external->kernel.apply(p.external->state.get(), frameData,
                                  frameSize, phased.data(), phased.size(),
                                  inputs.data(),inputs.size(),xyz.data(), pts->size())) {
        return false;
    }
    // The same atomic check the stage-side host makes: a non-finite
    // candidate fails the mover rather than publishing.
    for (const float value : xyz) {
        if (!std::isfinite(value)) {
            return false;
        }
    }
    for (size_t i = 0; i < pts->size(); ++i) {
        (*pts)[i] = RrVec3f(xyz[i * 3], xyz[i * 3 + 1], xyz[i * 3 + 2]);
    }
    return true;
}

bool
RrGeoApplyRevisionKernel(
    int op, const RrGeoMoverParameters &p, std::vector<RrVec3f> *pts,
    std::unordered_map<uint64_t, RrGeoWireBasisEntry> *wireCache,
    bool useSimd, RigExecSurfaceKernelCache<RrVec3f,RrVec3d> *surfaceCache = nullptr,
    RigExecWireRestCache<RrVec3f,RrVec2f> *wireRestCache = nullptr,
    std::shared_ptr<const RrGeoWireBasisEntry> *lastWireBasis = nullptr)
{
    switch (op) {
    case RrGeoOpExternal:
        return RrGeoApplyExternal(p, pts);
    case RrGeoOpMatrix:
        return RrGeoApplyMatrixKernel(p, pts, useSimd);
    case RrGeoOpSkin:
        return RrGeoApplySkinKernelWithTransforms(
            p, RrGeoSkinTransformsOf(p), pts, useSimd);
    case RrGeoOpBlendShape:
        return RrGeoApplyBlendShapeKernel(p, pts, surfaceCache);
    case RrGeoOpVolumeCorrect:
        RrGeoApplyVolumeCorrect(pts, p.referenceVolume, p.strength);
        return true;
    case RrGeoOpSmooth:
        RrGeoApplyLaplacianSmooth(
            pts, p.topologyCounts, p.topologyIndices, p.strength, surfaceCache);
        return true;
    case RrGeoOpDeltaMush:
        if(surfaceCache) {
            const auto *rest=surfaceCache->MushRest(p.restPoints,p.topologyCounts,
                p.topologyIndices,p.mushIterations,p.mushStep,p.mushPinBorders,p.mushDistanceWeight);
            return rest && RigExecApplyDeltaMushWithRestData(pts,*rest,p.mushDisplacement);
        }
        return RigExecApplyDeltaMushKernel<RrVec3f,RrVec3d>(pts,p.restPoints,
            p.topologyCounts,p.topologyIndices,p.mushIterations,p.mushStep,
            p.mushPinBorders,p.mushDistanceWeight,p.mushDisplacement);
    case RrGeoOpWrinkle:
        if(surfaceCache) {
            const auto *topology=surfaceCache->WrinkleTopology(p.restPoints.size(),
                p.topologyCounts,p.topologyIndices,p.wrinkleSettings.topology,p.wrinkleSettings.neighborDistance);
            return topology && RigExecApplyWrinkleWithTopology<RrVec3f,RrVec3d>(
                pts,p.restPoints,p.topologyCounts,p.topologyIndices,p.wrinkleSettings,*topology);
        }
        return RigExecApplyWrinkleKernel<RrVec3f,RrVec3d>(pts,p.restPoints,
            p.topologyCounts,p.topologyIndices,p.wrinkleSettings);
    case RrGeoOpLattice:
        if (p.restPoints.size() != pts->size()) {
            return false;  // cardinality mismatch fails atomically
        }
        RrGeoApplyLattice(
            pts, p.restPoints, p.auxPoints, p.auxPointsB, p.divisions,
            surfaceCache);
        return true;
    case RrGeoOpSurfaceProject:
        RrGeoApplySurfaceProject(
            pts, p.auxPoints, p.topologyCounts, p.topologyIndices,
            p.strength);
        return true;
    case RrGeoOpEmitGuidePoints:
        if (p.frames.frames.size() != pts->size()) {
            return false;
        }
        for (size_t i = 0; i < pts->size(); ++i) {
            (*pts)[i] = RrGeoToVec3f(p.frames.frames[i].points[0]);
        }
        return true;
    case RrGeoOpRibbon: {
        if (p.bindCoords.size() != pts->size()) {
            return false;
        }
        const size_t n = p.frames.frames.size();
        if (n < 2) {
            return false;
        }
        std::vector<RrMat4d> maps(n);
        for (size_t k = 0; k < n; ++k) {
            if (!RrPointsToMatrix(
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
            const RrVec3d a = maps[k].TransformAffine(RrGeoToVec3d((*pts)[i]));
            const RrVec3d b = maps[k + 1].TransformAffine(RrGeoToVec3d((*pts)[i]));
            (*pts)[i] = RrGeoToVec3f(a + (b - a) * double(t));
        }
        return true;
    }
    case RrGeoOpWire: {
        // The checks before anything is built or applied; the basis build
        // and RrGeoApplyWire below make the rest of the decision's.
        if (!RrGeoWireKernelPrefix(p, pts->size())) {
            return false;
        }
        const RrGeoNurbsCurve rest{&p.restPoints, p.curveOrder,
                                  &p.curveKnots};
        const RrGeoNurbsCurve posed{&p.auxPoints, p.curveOrder,
                                   &p.curveKnots};
        if (RrGeoWireTakesSparseEnvelope(p.weights)) {
            const RrGeoWeightPacket &w = p.weights;
            const std::shared_ptr<const RrGeoWireBasis> basis =
                RrGeoCachedWireBasis(p, w.Indices(), pts->size(), wireCache,lastWireBasis);
            if (!basis) {
                return false;
            }
            return RrGeoApplyWireBasis(pts, *basis, w.Indices(), w.Values(),
                                       p.restPoints, p.auxPoints);
        }
        // The runtime runs serially; a point range is an independent
        // sub-problem, so the serial call is the parallel loop's answer.
        const auto *restEvaluations=wireRestCache?wireRestCache->Get(rest,
            p.wireBindCoords.data(),p.wireBindCoords.size(),p.dropoffDistance):nullptr;
        return RrGeoApplyWire(pts, rest, posed, p.wireBindCoords.data(),
                              p.wireBindCoords.size(),
                              p.dropoffDistance, 0, pts->size(),restEvaluations);
    }
    case RrGeoOpRecomputeNormals:
    case RrGeoOpRecomputeExtent:
        return RrGeoApplyDerivedKernel(op, p, pts);
    }
    return false;
}

// \p source, when set, holds the \p sourceCount points entering the revision
// and \p pts receives the result out of place, as the program's
// RunRevisionKernel: a full-strength matrix reads them where they are, every
// other operation copies them first, and the blend reads \p source.
// \p envelope, when it covers the points, is the separate-blend envelope
// RevisionStatic already resolved from this packet; otherwise it resolves
// here.
bool
RrGeoRunRevisionKernel(
    int op, const RrGeoMoverParameters &p, std::vector<RrVec3f> *pts,
    std::unordered_map<uint64_t, RrGeoWireBasisEntry> *wireCache,
    bool useSimd, RigExecSurfaceKernelCache<RrVec3f,RrVec3d> *surfaceCache = nullptr,
    RigExecWireRestCache<RrVec3f,RrVec2f> *wireRestCache = nullptr,
    std::shared_ptr<const RrGeoWireBasisEntry> *lastWireBasis = nullptr,
    const std::vector<float> *envelope = nullptr,
    const RrVec3f *source = nullptr, size_t sourceCount = 0)
{
    if (!RrGeoPacketMatches(op, p)) {
        return false;
    }
    if (source) {
        if (op == RrGeoOpMatrix && RrGeoEnvelopeIsFullStrength(p.weights)) {
            pts->resize(sourceCount);
            RrGeoApplyMatrixKernelRange(p, nullptr, 0, sourceCount, source,
                                        pts->data(), useSimd);
            return true;
        }
        pts->assign(source, source + sourceCount);
    }
    if (!RrGeoRevisionTakesSeparateBlend(op, p.weights)) {
        return RrGeoApplyRevisionKernel(op, p, pts, wireCache, useSimd, surfaceCache, wireRestCache, lastWireBasis);
    }

    const bool fullStrengthEnvelope = RrGeoEnvelopeIsFullStrength(p.weights);
    const size_t precedingSize = pts->size();
    std::vector<RrVec3f> preceding;
    if (!fullStrengthEnvelope && !source) {
        preceding = *pts;
    }
    if (!RrGeoApplyRevisionKernel(op, p, pts, wireCache, useSimd, surfaceCache, wireRestCache, lastWireBasis)) {
        return false;
    }
    if (pts->size() != precedingSize) {
        return false;
    }
    if (!fullStrengthEnvelope) {
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
        RrGeoBlendEnvelopeAll(source ? source : preceding.data(), weights,
                              pts->size(), pts->data());
    }
    return true;
}

RrGeoMoverStatus
RrGeoStatusForParameters(const RrGeoMoverParameters &parameters,
                         const std::string &moverPath)
{
    RrGeoMoverStatus status;
    if (!parameters.enabled) {
        status.state = "disabled";
    } else if (parameters.valid) {
        status.state = "ok";
    } else {
        status.state = "moverFailed";
        status.firstBadAddress = moverPath;
    }
    return status;
}

}  // namespace

// Scratch: revision packets, influence tables, output buffers, chunks, skin
// topologies, blend layouts and points, base points and the resolved stage
// reads. Mirrors GeomChain/GeomRevision/GeomChunk.

struct RrGeometryScratch {
    struct Chunk {
        int begin = 0;
        int end = 0;
        std::vector<int32_t> key;
        std::vector<RrMat4d> transforms;
        std::vector<float> rows;
        std::vector<RrGeoScaledDualQuat> palette;
        bool keyChanged = false;
        bool ok = false;
    };
    struct Revision {
        std::map<std::tuple<uint32_t,bool,uint8_t>,const RigExecWireExternalDeclaredInput *> leafSites;
        mutable RigExecSurfaceKernelCache<RrVec3f,RrVec3d> surfaceCache;
        RigExecWireRestCache<RrVec3f,RrVec2f> wireRestCache;
        std::shared_ptr<const RrGeoWireBasisEntry> lastWireBasis;
        RrGeoMoverParameters parameters;
        RrGeoMoverParameters lastParameters;
        RrGeoWeightPacket nativePacketCache;
        std::shared_ptr<const RrGeoWeightPacket::Arrays> currentPhaseArrays;
        RrGeoMoverStatus status;
        RrGeoMoverStatus lastStatus;
        bool ran = false;
        bool created = true;
        bool executed = false;
        bool staticDirty = false;
        std::vector<RrVec3f> output;
        std::vector<RrVec3f> stagingOutput;
        int currentSource = -1;
        // As GeomRevision's: the chunk's unapplied result is in staging; the
        // copy of the points a pass-through publication carried; and the
        // content version RevisionDone and ChainDirty key the points by.
        bool stagingFresh = false;
        std::vector<RrVec3f> passedPoints;
        uint64_t doneVersion = 0;
        std::vector<RrMat4d> influences;
        bool influencesValid = false;
        std::vector<RrMat4d> packetInfluences;
        bool influencesChanged = false;
        RrMat4d transform;
        bool haveTransform = false;
        // rigExec:space's matrix this run and whether one was named, out
        // of the fold, for the cluster conjugation and the wire assembly.
        RrMat4d carry = RrMat4d(1.0);
        bool haveCarry = false;
        std::vector<float> rows;
        std::vector<RrGeoScaledDualQuat> palette;
        std::vector<Chunk> chunks;
        bool chunked = false;
        std::shared_ptr<const RrGeoSkinTopology> topology;
        bool topologyResolved = false;
        // A fixed revision's layout as its layout slots describe it (the
        // live SkinTopology op's handle): the Open layout while they stand
        // at their defaults, else one built from them, kept until they
        // change. The prologue adopts it.
        std::shared_ptr<const RrGeoSkinTopology> layoutHandle;
        std::shared_ptr<const RrGeoSkinTopology> partitionTopology;
        std::vector<float> envelope;
        bool envelopeOk = false;
        bool fullStrength = false;
        bool layoutUsable = false;
        // RevisionStatic's apply-or-fail decision (GeomRevision::acceptance).
        RrGeoAcceptance acceptance = RrGeoAcceptance::Refuses;
        float defaultWeight = 0.0f;
        float lastDefaultWeight = 0.0f;
        bool partitionStale = false;
        size_t precedingCount = 0;
        int partitionElementSize = 0;
        size_t partitionIndexCount = 0;
        size_t partitionPointCount = 0;
        RrGeoWeightPacket currentPhasePacket;
        RrRetainedArray<float> weightField;
        bool weightFieldPublished = false;
        std::vector<RrVec3f> lastAuxPoints;
        std::string resultStatus;
        // Per entry of the wire revision's point_bindings, the chain
        // version its assembly read: null where the binding answered
        // nothing and the site reads its resolved input. Sized at Open,
        // filled by each assembly before any site reads it.
        std::vector<const std::vector<RrVec3f> *> phasedPoints;
        // The stage reads the assembly makes, resolved at Open. Per
        // mover-relative attribute, its property node (0: none); per
        // binding handle, its path. Per time (0 live, 1 rest) and read,
        // its path-read row (-1: none, the site reads its fallback); and
        // each shader dial's live row.
        std::array<uint32_t, RrGeoAttrCount> attrPath{};
        std::array<std::array<int32_t, RrGeoAttrCount>, 2> attrRead;
        std::array<uint32_t, RrGeoBindCount> bindingPath{};
        std::array<std::array<int32_t, RrGeoBindCount>, 2> bindingRead;
        std::vector<int32_t> dialRead;
        Revision()
        {
            transform.SetIdentity();
            for (std::array<int32_t, RrGeoAttrCount> &rows : attrRead) {
                rows.fill(-1);
            }
            for (std::array<int32_t, RrGeoBindCount> &rows : bindingRead) {
                rows.fill(-1);
            }
        }
    };
    struct Chain {
        // The base points: the target's points input when the file lists
        // one (baseSlot), else the ones the bake captured, converted at
        // Open (staticBase). haveBase is false when the file holds none.
        int32_t baseSlot = -1;
        std::vector<RrVec3f> sampleMissingBase;
        std::vector<RrVec3f> staticBase;
        bool haveBase = false;
        bool sampleHaveBase = false;
        std::vector<RrVec3f> sampledBase;
        std::vector<RrVec3f> lastBase;
        bool haveResult = false;
        RrRetainedArray<RrVec3f> result;
        std::string resultStatus;
        std::vector<RrVec3f> spare;
        // ChainInput, ChainBase and ChainPoints content versions, each
        // bumped exactly when its array's bytes move. ChainInput's moves at
        // publication, against `publishedInput`, the sampled base it last
        // published; `sampledMoved` says a prologue wrote `sampledBase`
        // since (RrGeometryPublishChainInputs).
        uint64_t inputVersion = 0, baseVersion = 0, resultVersion = 0;
        std::vector<RrVec3f> publishedInput;
        bool sampledMoved = false;
        bool scheduleDirty = true;
        std::vector<Revision> revisions;
        uint32_t createdCount = 0;
        uint32_t scheduleCount = 0;
    };
    struct Derived {
        std::vector<RrVec3f> base;
        bool haveBase = false;
        bool sampleHaveBase = false;
        std::vector<RrVec3f> sampledBase;
        std::vector<RrVec3f> lastBase;
        Revision revision;
        RrRetainedArray<RrVec3f> result;
        std::vector<RrVec3f> spare;
        // DerivedOut's content version of `result`.
        uint64_t resultVersion = 0;
        bool haveResult = false;
        bool baseDirty = false;
        uint32_t createdCount = 0;
        uint32_t scheduleCount = 0;
    };

    // The program's path-read table (RrProgram::pathReads), whose read rows
    // the prologue evaluates and whose rows the resolved reads index.
    const RrProgram *program=nullptr;
    const std::vector<RrPathRead> *pathReads = nullptr;
    // Per chain, per revision (blendLayouts) or derived target
    // (derivedBlendLayouts), per binding channel, per sample: a sparse
    // sample's layout, built at Open; null for a dense sample or one with
    // no layout.
    using SampleLayouts =
        std::vector<std::vector<std::shared_ptr<const RrGeoBlendLayout>>>;
    std::vector<std::vector<SampleLayouts>> blendLayouts;
    std::vector<std::vector<SampleLayouts>> derivedBlendLayouts;
    // The same shape, beside each sparse sample's layout: its offsets and
    // point indices reads at Default (path and rest row, resolved at Open),
    // the input slots they walk, and what the held layout was built from --
    // per slot its content version and its authored and present marks, and
    // the raw point count. The layout is a pure function of those, so an
    // assembly rebuilds it only when one of them moved.
    struct SparseSource {
        uint32_t offsetsPath = 0;
        int32_t offsetsRow = -1;
        uint32_t indicesPath = 0;
        int32_t indicesRow = -1;
        std::vector<uint32_t> slots;
        bool built = false;
        std::vector<uint64_t> versions;
        std::vector<uint8_t> marks;
        size_t pointCount = 0;
    };
    using SampleSources = std::vector<std::vector<SparseSource>>;
    std::vector<std::vector<SampleSources>> blendSources;
    std::vector<std::vector<SampleSources>> derivedBlendSources;
    // The same shape for the dense points each sample consumed at the bake
    // time, converted at Open, one vector per pool entry: noPoints for a
    // sparse sample or one the file holds no points for.
    using SamplePoints =
        std::vector<std::vector<const std::vector<RrVec3f> *>>;
    std::vector<std::vector<SamplePoints>> blendPoints;
    std::vector<std::vector<SamplePoints>> derivedBlendPoints;
    std::map<uint32_t, std::vector<RrVec3f>> blendPointPool;
    std::vector<RrVec3f> noPoints;
    std::vector<Chain> chains;
    std::vector<Derived> derived;
    std::unordered_map<uint64_t, RrGeoWireBasisEntry> wireBasis;
    std::vector<std::shared_ptr<const RrGeoSkinTopology>> epochTopologies;
    std::vector<std::shared_ptr<const RrGeoSkinTopology>>
        epochPartitionTopologies;

    // The sample point tables point into noPoints and blendPointPool, so
    // the scratch never moves or copies.
    RrGeometryScratch() = default;
    RrGeometryScratch(const RrGeometryScratch &) = delete;
    RrGeometryScratch &operator=(const RrGeometryScratch &) = delete;
};

namespace {

RrGeometryScratch *
RrGeoScratch(RrProgram *program)
{
    return static_cast<RrGeometryScratch *>(program->geo.get());
}

// One stage read an assembler makes, resolved at Open: the attribute's path
// id (0 when the file names none), its path-read row at the site's time
// (-1: none, so the site reads its fallback), and whether that time is the
// rest (Default) one.
struct RrGeoRead {
    uint32_t path = 0;
    int32_t row = -1;
    bool rest = false;
    const RrGeometryScratch::Revision *owner = nullptr;
};

// Mover-relative attribute \p attr of revision \p rev, read at the rest
// time when \p rest, else live.
RrGeoRead
RrGeoAttrRead(const RrGeometryScratch::Revision &rev, RrGeoAttr attr,
              bool rest = false)
{
    RrGeoRead read;
    read.path = rev.attrPath[size_t(attr)];
    read.row = rev.attrRead[rest ? 1 : 0][size_t(attr)];
    read.rest = rest;
    read.owner = &rev;
    return read;
}

// Binding handle \p which of revision \p rev.
RrGeoRead
RrGeoBindingRead(const RrGeometryScratch::Revision &rev, RrGeoBinding which,
                 bool rest = false)
{
    RrGeoRead read;
    read.path = rev.bindingPath[size_t(which)];
    read.row = rev.bindingRead[rest ? 1 : 0][size_t(which)];
    read.rest = rest;
    read.owner = &rev;
    return read;
}

std::optional<RrPathValue>
RrGeoPathRead(const RrGeometryScratch *scratch, const RrGeoRead &read, int expected=-1)
{
    const auto *row=read.row<0 ? nullptr : &(*scratch->pathReads)[size_t(read.row)];
    RrPathValue value=row?row->value:RrPathValue();
    if(expected<0 && row && row->read && row->read->read) expected=int(row->read->read->tag);
    if (read.owner) {
        auto site=read.owner->leafSites.find({read.path,read.rest,uint8_t(expected)});
        if(site==read.owner->leafSites.end() && expected==int(RigExecWireInputTag::Double)) {
            const auto floatSite=read.owner->leafSites.find(
                {read.path,read.rest,uint8_t(RigExecWireInputTag::Float)});
            if(floatSite!=read.owner->leafSites.end() && floatSite->second->allowFloatToDouble)
                site=floatSite;
        }
        if (site!=read.owner->leafSites.end() &&
            !RigExecFormatIsArrayTag(site->second->read->tag)) {
            if(read.rest && row && !row->read &&
               site->second->flavour==fb::ExternalInputFlavour::Raw) {
                const auto &walk=site->second->read->walk;
                const bool authored=!walk.empty() &&
                    scratch->program->inputState.slotAuthored[walk.front()]!=0;
                if(!authored) return value;
            }
            RrWireValue current;
            if (!RrReadExternalScalar(scratch->program,*site->second,&current)) return std::nullopt;
            RrPathValueFromWire(site->second->read->tag,current,&value);
            return value;
        }
    }
    if(!row) return std::nullopt;
    if(row->read && row->read->read && !RigExecFormatIsArrayTag(row->read->read->tag))
        RrPathValueFromRead(*row->read,RrReadPathScalar(scratch->program,*row->read),&value);
    return value;
}

RrGeoWeightPacket
RrGeoStorePacket(const RrProgram *program, const RrWeightPacket &packet,RrGeoWeightPacket *cache=nullptr)
{
    RrGeoWeightPacket out;
    out.representation = program->TextOrEmpty(packet.representation);
    out.rangePolicy = program->TextOrEmpty(packet.rangePolicy);
    if(cache && cache->representation==out.representation && cache->rangePolicy==out.rangePolicy &&
       std::memcmp(&cache->defaultWeight,&packet.defaultWeight,sizeof(float))==0 && cache->valid==packet.valid &&
       cache->Values().size()==packet.values.size() && cache->Indices().size()==packet.indices.size() &&
       (packet.values.empty() || std::memcmp(cache->Values().data(),packet.values.data(),packet.values.size()*sizeof(float))==0) &&
       std::equal(cache->Indices().begin(),cache->Indices().end(),packet.indices.begin()))return *cache;
    auto arrays=std::make_shared<RrGeoWeightPacket::Arrays>();
    arrays->values=packet.values;arrays->indices.assign(packet.indices.begin(),packet.indices.end());
    out.arrays=std::move(arrays);
    out.defaultWeight = packet.defaultWeight;
    out.valid = packet.valid;
    if(cache)*cache=out;
    return out;
}

// A skin layout as the baked program holds it: a raw layout's stored
// arrays, or the sparse table expanded to its dense rows. Null when the
// revision has none.
std::shared_ptr<const RrGeoSkinTopology>
RrGeoWireTopology(const RigExecWireSkinTopology *wire)
{
    if (!wire) {
        return nullptr;
    }
    auto topology = std::make_shared<RrGeoSkinTopology>();
    std::vector<int32_t> indices;
    RigExecFormatExpandTopology(*wire, &indices, &topology->weights);
    topology->indices.assign(indices.begin(), indices.end());
    topology->elementSize = wire->elementSize;
    topology->pointCount = size_t(wire->pointCount);
    topology->influenceCount = size_t(wire->influenceCount);
    topology->validated = wire->validated;
    return topology;
}

// A typed stage read: the resolved-inputs overlay first (RrInputsOverlay:
// a standing override or a property-chain result, exact type, as
// RigExecResolvedInputs::GetAttribute checks it), then this run's stage
// value (a connection-following scalar evaluated over the input slots, else
// the recorded one), else the fallback. Array sites take the phase
// overlay's points instead of the property results, which carry no arrays.
bool
RrGeoReadBool(const RrGeometryScratch *scratch, const RrGeoRead &at,
              bool fallback)
{
    const auto read = RrGeoPathRead(scratch, at, int(RigExecWireInputTag::Bool));
    if (read && read->tag == RrPathValue::Tag::Bool) {
        return read->boolean;
    }
    return fallback;
}

float
RrGeoReadFloat(const RrProgram *program,
               const RrGeometryScratch *scratch, const RrGeoRead &at,
               float fallback)
{
    const auto read = RrGeoPathRead(scratch, at, int(RigExecWireInputTag::Float));
    if (read && read->tag == RrPathValue::Tag::Float) {
        return read->f32;
    }
    return fallback;
}

double
RrGeoReadDouble(const RrProgram *program,
                const RrGeometryScratch *scratch, const RrGeoRead &at,
                double fallback)
{
    const auto read = RrGeoPathRead(scratch, at, int(RigExecWireInputTag::Double));
    if (read && read->tag == RrPathValue::Tag::Double) {
        return read->f64;
    }
    // A float attribute the site reads as a double, widened here.
    if (read && read->tag == RrPathValue::Tag::Float) {
        return double(read->f32);
    }
    return fallback;
}

std::string
RrGeoReadToken(const RrProgram *program,
               const RrGeometryScratch *scratch, const RrGeoRead &at,
               const std::string &fallback)
{
    const auto read = RrGeoPathRead(scratch, at, int(RigExecWireInputTag::Token));
    if (read && read->tag == RrPathValue::Tag::Token) {
        return program->TextOrEmpty(read->token);
    }
    return fallback;
}

// An array site's read of row \p at this run (RrPathArray: a static value,
// or the elements its input slots answer at the site's time), copied into
// \p out; empty when the row holds no array of the site's type.
template <class T>
void
RrGeoReadArray(const RrProgram *program, const RrGeometryScratch *scratch,
               const RrGeoRead &at, std::vector<T> *out)
{
    const std::vector<T> *owned = nullptr;
    if (at.owner) {
        const auto site=at.owner->leafSites.find({at.path,at.rest,uint8_t(RrArrayTag<T>::value)});
        if (site!=at.owner->leafSites.end() && site->second->read->tag==RrArrayTag<T>::value) {
            // The same sampled slot can serve current and Default reads. Its
            // bake-time default is not the raw Default property's snapshot.
            // The retained path row applies authored-vs-sampled selection exactly.
            if(at.rest && site->second->flavour==fb::ExternalInputFlavour::Raw && at.row>=0)
                owned=RrPathArray<T>(program,(*scratch->pathReads)[size_t(at.row)]);
            else owned=static_cast<const std::vector<T> *>(RrReadExternalArray(program,*site->second));
        }
        if (site!=at.owner->leafSites.end()) {
            if (owned) out->assign(owned->begin(),owned->end()); else out->clear();
            return;
        }
    }
    const std::vector<T> *read =
        at.row < 0
            ? nullptr
            : RrPathArray<T>(program, (*scratch->pathReads)[size_t(at.row)]);
    if (read) {
        out->assign(read->begin(), read->end());
        return;
    }
    out->clear();
}

void
RrGeoReadVec3fArray(const RrProgram *program,
                    const RrGeometryScratch *scratch, const RrGeoRead &at,
                    const std::vector<RrVec3f> *overlay,
                    std::vector<RrVec3f> *out)
{
    if (overlay) {
        *out = *overlay;
        return;
    }
    RrGeoReadArray(program, scratch, at, out);
}

void
RrGeoReadIntArray(const RrProgram *program, const RrGeometryScratch *scratch,
                  const RrGeoRead &at, std::vector<int> *out)
{
    RrGeoReadArray(program, scratch, at, out);
}

void
RrGeoReadFloatArray(const RrProgram *program,
                    const RrGeometryScratch *scratch, const RrGeoRead &at,
                    std::vector<float> *out)
{
    RrGeoReadArray(program, scratch, at, out);
}

void
RrGeoReadVec2fArray(const RrProgram *program,
                    const RrGeometryScratch *scratch, const RrGeoRead &at,
                    std::vector<RrVec2f> *out)
{
    RrGeoReadArray(program, scratch, at, out);
}

void
RrGeoReadDoubleArray(const RrProgram *program,
                     const RrGeometryScratch *scratch, const RrGeoRead &at,
                     std::vector<double> *out)
{
    RrGeoReadArray(program, scratch, at, out);
}

RrVec3i
RrGeoReadVec3i(const RrGeometryScratch *scratch, const RrGeoRead &at,
               const RrVec3i &fallback)
{
    const auto read = RrGeoPathRead(scratch, at, int(RigExecWireInputTag::Vec3i));
    if (read && read->tag == RrPathValue::Tag::Vec3i) {
        return RrVec3i(read->vec3i[0], read->vec3i[1], read->vec3i[2]);
    }
    return fallback;
}

int
RrGeoReadInt(const RrGeometryScratch *scratch, const RrGeoRead &at,
             int fallback)
{
    const auto read = RrGeoPathRead(scratch, at, int(RigExecWireInputTag::Int));
    if (read && read->tag == RrPathValue::Tag::Int) {
        return int(read->i32);
    }
    return fallback;
}

// The revision's phase overlay (RigExecBakedOverlayPointReads): the points
// the point binding of input \p path resolved to in this assembly's
// prologue, or null -- no binding, or one that answered nothing -- for the
// resolved input.
const std::vector<RrVec3f> *
RrGeoPhaseOverlay(const RrGeometryScratch::Revision &rev,
                  const RigExecWireRevision &wire, uint32_t path)
{
    for (size_t i = 0;
         i < wire.pointBindings.size() && i < rev.phasedPoints.size(); ++i) {
        if (wire.pointBindings[i].inputPath == path) {
            return rev.phasedPoints[i];
        }
    }
    return nullptr;
}

// Resolves revision \p wire's stage reads into \p rev at Open. A
// mover-relative attribute is the property node of that name under the
// revision's mover prim (none without a mover prim); every read is its
// path's row at the site's time, none for path 0.
void
RrGeoResolveReads(const std::vector<RrPathRead> &table,
                  const std::map<std::pair<uint32_t, int>, uint32_t>
                      &properties,
                  const RigExecWireRevision &wire,
                  RrGeometryScratch::Revision *rev)
{
    rev->leafSites.clear();
    for (const auto *sites : {&wire.leafSites,&wire.layoutLeafSites})
        for (const auto &site : *sites)
            rev->leafSites.emplace(std::make_tuple(site.path,site.time==fb::ExternalInputTime::AtDefault,uint8_t(site.read->tag)),&site);
    const auto row = [&](uint32_t path, bool rest) {
        return path == 0 ? int32_t(-1) : RrFindPathReadRow(table, path, rest);
    };
    for (int a = 0; a < RrGeoAttrCount; ++a) {
        uint32_t path = 0;
        if (wire.moverPrim != 0) {
            const auto found =
                properties.find(std::make_pair(wire.moverPrim, a));
            if (found != properties.end()) {
                path = found->second;
            }
        }
        rev->attrPath[size_t(a)] = path;
        rev->attrRead[0][size_t(a)] = row(path, false);
        rev->attrRead[1][size_t(a)] = row(path, true);
    }
    const RigExecWireRevisionBinding &binding = *wire.binding;
    const uint32_t handles[RrGeoBindCount] = {
        binding.topologyCounts,    binding.topologyIndices,
        binding.cagePoints,        binding.surfacePoints,
        binding.bindCoords,        binding.driverCurvePoints,
        binding.driverCurveOrder,  binding.driverCurveKnots,
        binding.widths,
    };
    for (int b = 0; b < RrGeoBindCount; ++b) {
        rev->bindingPath[size_t(b)] = handles[b];
        rev->bindingRead[0][size_t(b)] = row(handles[b], false);
        rev->bindingRead[1][size_t(b)] = row(handles[b], true);
    }
    rev->dialRead.clear();
    for (const uint32_t dial : wire.shaderDials) {
        rev->dialRead.push_back(row(dial, false));
    }
}

// Adds to \p slots, once each, the input slots an array read of \p rev
// walks: path-read row \p row's, and every leaf site's at \p path.
void
RrGeoArrayReadSlots(const std::vector<RrPathRead> &table,
                    const RrGeometryScratch::Revision &rev, uint32_t path,
                    int32_t row, std::vector<uint32_t> *slots)
{
    const auto add = [slots](const RigExecWireInput *read) {
        if (!read || !RigExecFormatIsArrayTag(read->tag)) {
            return;
        }
        for (const uint32_t slot : read->walk) {
            if (std::find(slots->begin(), slots->end(), slot) ==
                slots->end()) {
                slots->push_back(slot);
            }
        }
    };
    if (row >= 0 && size_t(row) < table.size()) {
        const RrPathRead &entry = table[size_t(row)];
        add(entry.read ? entry.read->read.get() : nullptr);
    }
    if (path == 0) {
        return;
    }
    for (const auto &[identity, site] : rev.leafSites) {
        if (std::get<0>(identity) == path) {
            add(site->read.get());
        }
    }
}

// Marks the input slots revision \p rev's topology reads walk
// (RrInputsMarkTopology): its skin's joint indices, its mesh's face counts
// and indices, and its driver curve's order and knots, live and at rest.
void
RrGeoMarkTopologySlots(RrProgram *program, const RigExecWireRevision &wire,
                       const RrGeometryScratch::Revision &rev)
{
    std::vector<uint32_t> slots;
    if (wire.jointIndicesSlot >= 0) {
        slots.push_back(uint32_t(wire.jointIndicesSlot));
    }
    const auto read = [&](uint32_t path) {
        if (path == 0) {
            return;
        }
        for (const bool rest : {false, true}) {
            RrGeoArrayReadSlots(program->pathReads, rev, path,
                                RrFindPathReadRow(program->pathReads, path,
                                                  rest),
                                &slots);
        }
    };
    read(rev.attrPath[size_t(RrGeoAttrJointIndices)]);
    for (const RrGeoBinding binding :
         {RrGeoBindTopologyCounts, RrGeoBindTopologyIndices,
          RrGeoBindDriverCurveOrder, RrGeoBindDriverCurveKnots}) {
        read(rev.bindingPath[size_t(binding)]);
    }
    for (const uint32_t slot : slots) {
        RrInputsMarkTopology(program, slot);
    }
}

// The Raw leaf site at Default that \p rev declares for \p attribute
// (".offsets" or ".pointIndices") of blend shape \p shape, the read the
// sparse assembly makes: its path and rest row, or path 0 and row -1 when
// \p rev declares none. Spells paths, so Open alone calls it.
std::pair<uint32_t, int32_t>
RrGeoSparseSourceRead(const RigExecWireFile &file,
                      const std::vector<RrPathRead> &table,
                      const RrGeometryScratch::Revision &rev,
                      const std::string &shape, const char *attribute,
                      uint8_t tag)
{
    const std::string path = shape + attribute;
    for (const auto &[identity, site] : rev.leafSites) {
        if (!std::get<1>(identity) || std::get<2>(identity) != tag ||
            site->flavour != fb::ExternalInputFlavour::Raw) {
            continue;
        }
        if (RigExecFormatPathText(file, site->path) != path) {
            continue;
        }
        return {site->path, RrFindPathReadRow(table, site->path, true)};
    }
    return {0, -1};
}

// Slot \p slot's authored (1) and present (2) marks, which decide which of
// its elements a read at Default takes.
uint8_t
RrGeoSlotMarks(const RrProgram *program, uint32_t slot)
{
    return uint8_t((RrInputArrayAuthored(program, slot) ? 1 : 0) |
                   (RrInputHasValue(program, slot) ? 2 : 0));
}

// Whether \p source's held layout was built from what its reads answer
// now: the same raw \p pointCount, and per slot they walk the same content
// version and marks. A slot's version moves whenever a run reads other
// elements than the run before it, and never back, so an unmoved version
// is the elements the layout was built from.
bool
RrGeoSparseSourceHeld(const RrProgram *program,
                      const RrGeometryScratch::SparseSource &source,
                      size_t pointCount)
{
    if (!source.built || source.pointCount != pointCount ||
        source.versions.size() != source.slots.size() ||
        source.marks.size() != source.slots.size()) {
        return false;
    }
    for (size_t i = 0; i < source.slots.size(); ++i) {
        const uint32_t slot = source.slots[i];
        if (RrInputArrayVersion(program, slot) != source.versions[i] ||
            RrGeoSlotMarks(program, slot) != source.marks[i]) {
            return false;
        }
    }
    return true;
}

// Records on \p source what its layout was just built from.
void
RrGeoNoteSparseSource(const RrProgram *program, size_t pointCount,
                      RrGeometryScratch::SparseSource *source)
{
    source->built = true;
    source->pointCount = pointCount;
    source->versions.clear();
    source->marks.clear();
    for (const uint32_t slot : source->slots) {
        source->versions.push_back(RrInputArrayVersion(program, slot));
        source->marks.push_back(RrGeoSlotMarks(program, slot));
    }
}

}  // namespace

bool
RrGeometryPartitionStaleForTesting(const RrProgram *program,
                                   const std::string &moverPath)
{
    const auto *scratch =
        static_cast<const RrGeometryScratch *>(program->geo.get());
    if (!scratch) {
        return false;
    }
    const RigExecWireDomainGeometry &geo = *program->geometry;
    for (size_t c = 0; c < geo.chains.size() && c < scratch->chains.size();
         ++c) {
        const auto &revisions = geo.chains[c].revisions;
        for (size_t r = 0; r < revisions.size() &&
                           r < scratch->chains[c].revisions.size();
             ++r) {
            if (program->TextOrEmpty(revisions[r].moverPath) == moverPath) {
                return scratch->chains[c].revisions[r].partitionStale;
            }
        }
    }
    return false;
}

bool
RrGeometryRevisionDecisionForTesting(const RrProgram *program,
                                     const std::string &moverPath,
                                     int *acceptance, bool *chunksOk)
{
    const auto *scratch =
        static_cast<const RrGeometryScratch *>(program->geo.get());
    if (!scratch) {
        return false;
    }
    const RigExecWireDomainGeometry &geo = *program->geometry;
    for (size_t c = 0; c < geo.chains.size() && c < scratch->chains.size();
         ++c) {
        const auto &revisions = geo.chains[c].revisions;
        for (size_t r = 0; r < revisions.size() &&
                           r < scratch->chains[c].revisions.size();
             ++r) {
            if (program->TextOrEmpty(revisions[r].moverPath) != moverPath) {
                continue;
            }
            const auto &rev = scratch->chains[c].revisions[r];
            *acceptance = int(rev.acceptance);
            *chunksOk = !rev.chunks.empty();
            for (const auto &chunk : rev.chunks) {
                *chunksOk = *chunksOk && chunk.ok;
            }
            return true;
        }
    }
    return false;
}

bool
RrGeometrySizeScratch(RrProgram *program, std::string *error)
{
    auto scratch = std::make_shared<RrGeometryScratch>();
    const RigExecWireDomainGeometry &geo = *program->geometry;
    RrStore &store = program->store;
    const RrStatic &statics = program->statics;
    if (!statics.file) {
        if (error) {
            *error = "geometry sizing needs the opened file";
        }
        return false;
    }
    const RigExecWireFile &file = *statics.file;

    // The property node of each mover-relative attribute, by (prim,
    // attribute). The validator holds every property under a prim and one
    // node per (parent, name, kind).
    std::map<std::pair<uint32_t, int>, uint32_t> properties;
    {
        std::map<std::string, int> wanted;
        for (int a = 0; a < RrGeoAttrCount; ++a) {
            wanted.emplace(RrGeoAttrNames[a], a);
        }
        std::vector<int> attrOfName(file.names.size(), -1);
        for (size_t n = 0; n < file.names.size(); ++n) {
            const auto found = wanted.find(file.names[n]);
            if (found != wanted.end()) {
                attrOfName[n] = found->second;
            }
        }
        for (size_t i = 1; i < file.paths.size(); ++i) {
            const RigExecWirePathNode &node = file.paths[i];
            if (node.kind() != RigExecWirePathKind::Property ||
                node.name() >= attrOfName.size() ||
                attrOfName[node.name()] < 0) {
                continue;
            }
            properties.emplace(
                std::make_pair(node.parent(), attrOfName[node.name()]),
                uint32_t(i));
        }
    }

    scratch->chains.resize(geo.chains.size());
    scratch->epochTopologies.resize(geo.revisionIndex.size() + geo.derivedIndex.size());
    scratch->epochPartitionTopologies.resize(geo.revisionIndex.size());
    for (size_t c = 0; c < geo.chains.size(); ++c) {
        const RigExecWireChain &chain = geo.chains[c];
        RrGeometryScratch::Chain &out = scratch->chains[c];
        out.revisions.resize(chain.revisions.size());
        for (size_t r = 0; r < chain.revisions.size(); ++r) {
            const RigExecWireRevision &wire = chain.revisions[r];
            if (!RrGeoOpName(wire.op)) {
                if (error) {
                    *error = "geometry references unknown revision op " +
                             std::to_string(wire.op);
                }
                return false;
            }
            RrGeometryScratch::Revision &rev = out.revisions[r];
            const size_t id = size_t(
                geo.chainRevisionBegin[c] + int(r));
            if (id >= store.revisionPublish.size()) {
                if (error) {
                    *error = "geometry revision index out of range";
                }
                return false;
            }
            const size_t influences = wire.influenceSlots.size();
            const RrMat4d identity = RrGeoIdentity();
            rev.influences.assign(influences, identity);
            rev.packetInfluences.assign(influences, identity);
            for (size_t k = 0; k < influences &&
                               k < wire.packetInfluences.size();
                 ++k) {
                // The wire's retained table is identity by construction;
                // the loop below only guards a hand-edited program.
                for (size_t row = 0; row < 4; ++row) {
                    for (size_t col = 0; col < 4; ++col) {
                        rev.packetInfluences[k][row][col] =
                            wire.packetInfluences[k][row * 4 + col];
                    }
                }
            }
            rev.chunked = wire.chunked;
            rev.chunks.resize(wire.chunks.size());
            for (size_t k = 0; k < wire.chunks.size(); ++k) {
                RrGeometryScratch::Chunk &chunk = rev.chunks[k];
                chunk.begin = wire.chunks[k].begin;
                chunk.end = wire.chunks[k].end;
                chunk.key = wire.chunks[k].key;
                if (wire.chunked) {
                    chunk.transforms.assign(influences, identity);
                    chunk.rows.assign(
                        influences * size_t(RrGeoSkinRowStride), 0.0f);
                    for (size_t t = 0; t < influences; ++t) {
                        RrGeoNarrowSkinRows(
                            chunk.transforms[t],
                            &chunk.rows[t * size_t(RrGeoSkinRowStride)]);
                    }
                }
            }
            // One expansion per layout: a partition the bake shared with
            // the topology shares its expansion here too.
            const std::shared_ptr<const RrGeoSkinTopology> topology =
                RrGeoWireTopology(wire.topology.get());
            if (wire.topologyResolved) {
                rev.topology = topology;
                scratch->epochTopologies[id] = rev.topology;
            }
            rev.layoutHandle = scratch->epochTopologies[id];
            rev.partitionTopology =
                wire.partitionSameAsTopology
                    ? topology
                    : RrGeoWireTopology(wire.partitionTopology.get());
            scratch->epochPartitionTopologies[id] = rev.partitionTopology;
            rev.partitionElementSize = wire.partitionElementSize;
            rev.partitionIndexCount = size_t(wire.partitionIndexCount);
            rev.partitionPointCount = size_t(wire.partitionPointCount);
            store.revisionPublish[id].weightFieldTarget =
                wire.weightFieldTarget;
            rev.phasedPoints.assign(wire.pointBindings.size(), nullptr);
            RrGeoResolveReads(program->pathReads, properties, wire, &rev);
            RrGeoMarkTopologySlots(program, wire, rev);
        }
    }
    scratch->derived.resize(geo.derivedIndex.size());
    for (size_t d = 0; d < geo.derivedIndex.size(); ++d) {
        const RigExecWireRevision &wire =
            *geo.chains[size_t(geo.derivedIndex[d].first)]
                 .derived[size_t(geo.derivedIndex[d].second)]
                 .revision;
        if (!RrGeoOpName(wire.op)) {
            if (error) {
                *error = "geometry references unknown revision op " +
                         std::to_string(wire.op);
            }
            return false;
        }
        scratch->derived[d].revision.phasedPoints.assign(
            wire.pointBindings.size(), nullptr);
        RrGeoResolveReads(program->pathReads, properties, wire,
                          &scratch->derived[d].revision);
        auto &revision = scratch->derived[d].revision;
        RrGeoMarkTopologySlots(program, wire, revision);
        const size_t layout = geo.revisionIndex.size() + d;
        if (wire.topologyResolved)
            scratch->epochTopologies[layout] = RrGeoWireTopology(wire.topology.get());
        revision.layoutHandle = scratch->epochTopologies[layout];
    }

    // The chains' and derived targets' base points, converted once: the
    // file holds the ones the bake captured, the same every run.
    const auto convert = [](const std::vector<RigExecWireVec3f> &points,
                            std::vector<RrVec3f> *out) {
        out->reserve(points.size());
        for (const RigExecWireVec3f &p : points) {
            out->push_back(RrVec3f(p[0], p[1], p[2]));
        }
    };
    // A chain with a base slot reads it every run instead.
    for (size_t c = 0; c < geo.chains.size(); ++c) {
        RrGeometryScratch::Chain &chain = scratch->chains[c];
        const std::vector<RigExecWireVec3f> *base = statics.ChainBase(c);
        chain.haveBase = base != nullptr;
        chain.baseSlot = geo.chains[c].baseSlot;
        if (base && chain.baseSlot < 0) {
            convert(*base, &chain.staticBase);
        }
    }
    // A layout slot's default is its stored layout's expansion.
    {
        const RrInputState &inputs = program->inputState;
        for (size_t s = 0; s < inputs.arrayOf.size(); ++s) {
            if (inputs.arrayOf[s] < 0) {
                continue;
            }
            const fb::RigExecWireValue &value =
                file.values[file.inputs[s].value()];
            if (value.arraySource == RigExecWireArraySource::Pool ||
                value.array >= scratch->epochTopologies.size() ||
                !scratch->epochTopologies[value.array]) {
                continue;
            }
            const RrGeoSkinTopology &layout =
                *scratch->epochTopologies[value.array];
            RrInputsBindArrayDefault(
                program, uint32_t(s),
                value.arraySource == RigExecWireArraySource::SkinIndices
                    ? static_cast<const void *>(&layout.indices)
                    : static_cast<const void *>(&layout.weights));
        }
    }
    for (size_t d = 0; d < geo.derivedIndex.size(); ++d) {
        RrGeometryScratch::Derived &derived = scratch->derived[d];
        const std::vector<RigExecWireVec3f> *base = statics.DerivedBase(d);
        derived.haveBase = base != nullptr;
        if (base) {
            convert(*base, &derived.base);
        }
    }

    // Allocate only immutable sample-table shape at Open. Selected owner
    // bodies normalize current raw sparse sources into their private entries.
    const auto layoutsOf = [](const RigExecWireRevision &wire) {
        RrGeometryScratch::SampleLayouts out(wire.blendChannels.size());
        for (size_t c = 0; c < wire.blendChannels.size(); ++c) {
            const RigExecWireBlendChannel &bound = wire.blendChannels[c];
            out[c].resize(bound.samples.size());
        }
        return out;
    };
    scratch->blendLayouts.resize(geo.chains.size());
    scratch->derivedBlendLayouts.resize(geo.chains.size());
    for (size_t c = 0; c < geo.chains.size(); ++c) {
        const RigExecWireChain &chain = geo.chains[c];
        for (const RigExecWireRevision &wire : chain.revisions) {
            scratch->blendLayouts[c].push_back(layoutsOf(wire));
        }
        for (const RigExecWireDerived &derived : chain.derived) {
            scratch->derivedBlendLayouts[c].push_back(
                layoutsOf(*derived.revision));
        }
    }
    // Each sparse sample's two source reads, resolved once against the
    // revision that assembles it; the slots they walk are topology.
    const auto sourcesOf = [&](const RigExecWireRevision &wire,
                               const RrGeometryScratch::Revision *rev) {
        RrGeometryScratch::SampleSources out(wire.blendChannels.size());
        for (size_t c = 0; c < wire.blendChannels.size(); ++c) {
            const RigExecWireBlendChannel &bound = wire.blendChannels[c];
            out[c].resize(bound.samples.size());
            for (size_t s = 0; rev && s < bound.samples.size(); ++s) {
                if (!bound.samples[s].blendShape) {
                    continue;
                }
                RrGeometryScratch::SparseSource &source = out[c][s];
                const std::string shape = RigExecFormatPathText(
                    *program->file, bound.samples[s].blendShape);
                std::tie(source.offsetsPath, source.offsetsRow) =
                    RrGeoSparseSourceRead(
                        *program->file, program->pathReads, *rev, shape,
                        ".offsets",
                        uint8_t(RigExecWireInputTag::Vec3fArray));
                std::tie(source.indicesPath, source.indicesRow) =
                    RrGeoSparseSourceRead(
                        *program->file, program->pathReads, *rev, shape,
                        ".pointIndices",
                        uint8_t(RigExecWireInputTag::IntArray));
                RrGeoArrayReadSlots(program->pathReads, *rev,
                                    source.offsetsPath, source.offsetsRow,
                                    &source.slots);
                RrGeoArrayReadSlots(program->pathReads, *rev,
                                    source.indicesPath, source.indicesRow,
                                    &source.slots);
                for (const uint32_t slot : source.slots) {
                    RrInputsMarkTopology(program, slot);
                }
            }
        }
        return out;
    };
    // A derived target's revision is the one its derivedIndex entry names.
    std::map<std::pair<size_t, size_t>, size_t> derivedAt;
    for (size_t d = 0; d < geo.derivedIndex.size(); ++d) {
        derivedAt.emplace(
            std::make_pair(size_t(geo.derivedIndex[d].first),
                           size_t(geo.derivedIndex[d].second)),
            d);
    }
    scratch->blendSources.resize(geo.chains.size());
    scratch->derivedBlendSources.resize(geo.chains.size());
    for (size_t c = 0; c < geo.chains.size(); ++c) {
        const RigExecWireChain &chain = geo.chains[c];
        for (size_t r = 0; r < chain.revisions.size(); ++r) {
            scratch->blendSources[c].push_back(sourcesOf(
                chain.revisions[r], &scratch->chains[c].revisions[r]));
        }
        for (size_t d = 0; d < chain.derived.size(); ++d) {
            const auto at = derivedAt.find(std::make_pair(c, d));
            scratch->derivedBlendSources[c].push_back(sourcesOf(
                *chain.derived[d].revision,
                at == derivedAt.end() ? nullptr
                                      : &scratch->derived[at->second].revision));
        }
    }
    // Each dense sample's points, converted once per pool entry.
    RrGeometryScratch &tables = *scratch;
    const auto pointsOf = [&](const RigExecWireRevision &wire) {
        RrGeometryScratch::SamplePoints out(wire.blendChannels.size());
        for (size_t c = 0; c < wire.blendChannels.size(); ++c) {
            const RigExecWireBlendChannel &bound = wire.blendChannels[c];
            out[c].assign(bound.samples.size(), &tables.noPoints);
            for (size_t s = 0; s < bound.samples.size(); ++s) {
                const RigExecWireBlendSample &sample = bound.samples[s];
                if (sample.blendShape) {
                    continue;
                }
                auto found = tables.blendPointPool.find(sample.pointsValue);
                if (found == tables.blendPointPool.end()) {
                    found = tables.blendPointPool
                                .emplace(sample.pointsValue,
                                         std::vector<RrVec3f>())
                                .first;
                    convert(file.vec3fArrays[sample.pointsValue].v,
                            &found->second);
                }
                out[c][s] = &found->second;
            }
        }
        return out;
    };
    scratch->blendPoints.resize(geo.chains.size());
    scratch->derivedBlendPoints.resize(geo.chains.size());
    for (size_t c = 0; c < geo.chains.size(); ++c) {
        const RigExecWireChain &chain = geo.chains[c];
        for (const RigExecWireRevision &wire : chain.revisions) {
            scratch->blendPoints[c].push_back(pointsOf(wire));
        }
        for (const RigExecWireDerived &derived : chain.derived) {
            scratch->derivedBlendPoints[c].push_back(
                pointsOf(*derived.revision));
        }
    }
    scratch->program=program;
    scratch->pathReads = &program->pathReads;
    program->geo = std::move(scratch);
    return true;
}

namespace {

// Step plumbing: step labels.

std::string
RrGeoStepLabel(const RrProgram *program, size_t step)
{
    return "step " + (program ? RrStepLabel(*program, step)
                              : std::to_string(step));
}

// The points of version \p version of \p chain (the program's PointsAt):
// 0 is the authored base, v > 0 what revision v - 1's fuse left, which is
// the output of the revision its currentSource names, or the base when it
// names none. Null for a version past the chain's revisions, which Open's
// validation refuses.
const std::vector<RrVec3f> *
RrGeoPointsVersion(const RrGeometryScratch::Chain &chain, size_t version)
{
    if (version > chain.revisions.size()) {
        return nullptr;
    }
    const int source =
        version == 0 ? -1 : chain.revisions[version - 1].currentSource;
    if (source < 0) {
        return &chain.lastBase;
    }
    if (size_t(source) >= chain.revisions.size()) {
        return nullptr;
    }
    return &chain.revisions[size_t(source)].output;
}

// The points entering revision r of a scratch chain (version r) and after
// it (version r + 1), as a pointer and a count; a null pointer for none.
void
RrGeoPointsBefore(const RrGeometryScratch::Chain &chain, size_t r,
                  const RrVec3f **points, size_t *count)
{
    if (!points || !count) {
        return;
    }
    const std::vector<RrVec3f> *version = RrGeoPointsVersion(chain, r);
    *points = version && !version->empty() ? version->data() : nullptr;
    *count = version ? version->size() : 0;
}

void
RrGeoPointsAfter(const RrGeometryScratch::Chain &chain, size_t r,
                 const RrVec3f **points, size_t *count)
{
    if (!points || !count) {
        return;
    }
    const std::vector<RrVec3f> *version = RrGeoPointsVersion(chain, r + 1);
    *points = version && !version->empty() ? version->data() : nullptr;
    *count = version ? version->size() : 0;
}

std::string
RrGeoPhaseName(const RrProgram *program, const RigExecWireReadPhase &phase)
{
    switch (phase.kind) {
    case 0:
        return "base";
    case 1:
        return "preceding";
    case 2:
        return "final";
    case 3:
        return program->TextOrEmpty(phase.prim);
    default:
        return "base";
    }
}

// The points \p binding resolves to this run (RigExecBakedResolvePoints):
// its first candidate whose chain read a base -- the chain's published
// points for a final read, else that version -- or null for the tail, when
// the reader reads its resolved input. A candidate past the chains, which
// Open's validation refuses, answers nothing.
const std::vector<RrVec3f> *
RrGeoResolvePoints(const RrGeometryScratch &scratch,
                   const RigExecWirePointsBinding &binding)
{
    for (const RigExecWirePointVersion &candidate : binding.candidates) {
        if (candidate.chain() < 0 ||
            size_t(candidate.chain()) >= scratch.chains.size()) {
            continue;
        }
        const RrGeometryScratch::Chain &chain =
            scratch.chains[size_t(candidate.chain())];
        if (!chain.haveBase) {
            continue;
        }
        if (binding.finalRead) {
            if(chain.haveResult)return &chain.result.Read();
            continue;
        }
        if (candidate.version() < 0) {
            return nullptr;
        }
        return RrGeoPointsVersion(chain, size_t(candidate.version()));
    }
    return nullptr;
}

// The revision's overlay prologue (RigExecBakedOverlayPointReads): each
// point binding, in binding order, resolves to the chain version it reads
// for the sites below, and a tail the author did not mean as "preceding"
// is diagnosed. The overlay only ever serves Vec3fArray sites; every other
// read misses it, as the baked one does.
void
RrGeoResolveRevisionPhases(RrProgram *program,
                           const RrGeometryScratch &scratch,
                           const RigExecWireRevision &wire,
                           RrGeometryScratch::Revision *rev,
                           std::vector<std::string> *diagnostics)
{
    for (size_t i = 0;
         i < wire.pointBindings.size() && i < rev->phasedPoints.size(); ++i) {
        const RigExecWirePointsBinding &binding = wire.pointBindings[i];
        const std::vector<RrVec3f> *answer =
            RrGeoResolvePoints(scratch, binding);
        rev->phasedPoints[i] = answer;
        if (!answer && binding.diagnoseMiss && diagnostics) {
            diagnostics->push_back(
                "diag " + program->TextOrEmpty(wire.moverPath) +
                ": read phase '" + RrGeoPhaseName(program, binding.phase) +
                "' for " + program->TextOrEmpty(binding.inputPath) +
                " resolved to nothing; read the authored base");
        }
    }
}

// Assembly: one revision's packet out of the static data, the path reads
// and the epoch tables. A port of AssembleRevision plus the RigExecAssemble*
// per-operation bodies it ends in.

struct RrGeoAssembleInputs {
    RrProgram *program = nullptr;
    RrGeometryScratch *scratch = nullptr;
    const RigExecWireRevision *wire = nullptr;
    RrGeometryScratch::Revision *rev = nullptr;
    const RrVec3f *basePoints = nullptr;
    size_t basePointCount = 0;
    size_t chain = 0;
    size_t revision = 0;
    bool derived = false;
    std::vector<std::string> *diagnostics = nullptr;
};

// inputs:enabled, replayed. A missing mover answers true without reading,
// exactly as _Enabled does.
bool
RrGeoAssembleEnabled(const RrGeoAssembleInputs &in)
{
    if (in.wire->moverPrim == 0) {
        return true;
    }
    return RrGeoReadBool(in.scratch, RrGeoAttrRead(*in.rev, RrGeoAttrEnabled),
                         true);
}

// inputs:defaultWeight as RevisionStatic reads it, for every main
// revision -- the same value the assembly's own resolved read answers,
// because a phase overlay holds no scalars. A derived target reads 1.
float
RrGeoAssembleDefaultWeight(const RrGeoAssembleInputs &in)
{
    if (in.derived) {
        return 1.0f;
    }
    return RrReadDefaultWeight(in.program, in.chain, in.revision);
}

RrGeoWeightPacket
RrGeoAssembleEnvelope(const RrGeoAssembleInputs &in, bool synthesizedDerived)
{
    if (synthesizedDerived) {
        return RrGeoWeightPacket::Constant(1.0f);
    }
    if (in.wire->weightObject >= 0 &&
        size_t(in.wire->weightObject) <
            in.program->store.weightPackets.size()) {
        if (in.wire->weightCurrentPhase) {
            return in.rev->currentPhasePacket;
        }
        return RrGeoStorePacket(
            in.program,
            in.program->store.weightPackets[size_t(in.wire->weightObject)],&in.rev->nativePacketCache);
    }
    return RrGeoWeightPacket::Constant(RrGeoAssembleDefaultWeight(in));
}

void
RrGeoReadBindingVec3fArray(const RrGeoAssembleInputs &in,
                           const RrGeoRead &at, std::vector<RrVec3f> *out)
{
    // Rest reads pass resolved=nullptr on the baked path: the phase
    // overlay only ever serves a live (time) read. A Default read
    // always replays the authored value.
    const std::vector<RrVec3f> *overlay =
        at.rest ? nullptr : RrGeoPhaseOverlay(*in.rev, *in.wire, at.path);
    RrGeoReadVec3fArray(in.program, in.scratch, at, overlay, out);
}

// Mover-relative attribute \p attr and binding handle \p which of the
// revision being assembled.
RrGeoRead
RrGeoAttrRead(const RrGeoAssembleInputs &in, RrGeoAttr attr,
              bool rest = false)
{
    return RrGeoAttrRead(*in.rev, attr, rest);
}

RrGeoRead
RrGeoBindingRead(const RrGeoAssembleInputs &in, RrGeoBinding which,
                 bool rest = false)
{
    return RrGeoBindingRead(*in.rev, which, rest);
}

// The blend channels: each channel's weight and each sample's activation
// read over the slots, in binding order, each channel's samples
// stable-sorted by activation. A dense sample with a point binding reads
// the chain version it resolves to this run; one with no binding, or whose
// binding answers nothing, reads its points read over the slots, or else
// its resolved points as Open converted them. Sparse samples take the
// layout Open built out of the sample's fields.
bool
RrGeoAssembleBlendDeltas(const RrGeoAssembleInputs &in,
                         const std::vector<RrVec3f> &base,
                         std::vector<RrVec3f> *deltas)
{
    auto &layoutTable = in.derived ? in.scratch->derivedBlendLayouts
                                         : in.scratch->blendLayouts;
    RrGeometryScratch::SampleLayouts *layouts =
        in.chain < layoutTable.size() &&
                in.revision < layoutTable[in.chain].size()
            ? &layoutTable[in.chain][in.revision]
            : nullptr;
    const auto &pointTable = in.derived ? in.scratch->derivedBlendPoints
                                        : in.scratch->blendPoints;
    const RrGeometryScratch::SamplePoints *points =
        in.chain < pointTable.size() &&
                in.revision < pointTable[in.chain].size()
            ? &pointTable[in.chain][in.revision]
            : nullptr;
    auto &sourceTable = in.derived ? in.scratch->derivedBlendSources
                                   : in.scratch->blendSources;
    RrGeometryScratch::SampleSources *sources =
        in.chain < sourceTable.size() &&
                in.revision < sourceTable[in.chain].size()
            ? &sourceTable[in.chain][in.revision]
            : nullptr;
    std::vector<RrGeoBlendChannel> channels;
    channels.reserve(in.wire->blendChannels.size());
    for (size_t c = 0; c < in.wire->blendChannels.size(); ++c) {
        const RigExecWireBlendChannel &bound = in.wire->blendChannels[c];
        RrGeoBlendChannel channel;
        channel.weight = RrReadBlendWeight(in.program, in.chain, in.revision,
                                           in.derived, c);
        for (size_t s = 0; s < bound.samples.size(); ++s) {
            const RigExecWireBlendSample &boundSample =
                bound.samples[s];
            RrGeoBlendSampleData sample;
            sample.activation = RrReadBlendActivation(
                in.program, in.chain, in.revision, in.derived, c, s);
            const std::vector<RrVec3f> *phased =
                boundSample.pointBinding
                    ? RrGeoResolvePoints(*in.scratch,
                                         *boundSample.pointBinding)
                    : nullptr;
            const std::vector<RrVec3f> *read =
                !phased && boundSample.pointsRead
                    ? RrArrayRead<RrVec3f>(in.program,
                                           *boundSample.pointsRead)
                    : nullptr;
            if (phased) {
                sample.points = phased;
            } else if (boundSample.pointsRead) {
                sample.points = read ? read : &in.scratch->noPoints;
            } else {
                sample.points = points && c < points->size() &&
                                        s < (*points)[c].size()
                                    ? (*points)[c][s]
                                    : &in.scratch->noPoints;
            }
            if (boundSample.blendShape && layouts && c < layouts->size() &&
                s < (*layouts)[c].size() && sources &&
                c < sources->size() && s < (*sources)[c].size()) {
                // Exact RawDefault leaves; sparse connections never imply
                // Final. The reads were resolved at Open; the layout is
                // rebuilt only when what they answer or the count moved.
                RrGeometryScratch::SparseSource &source = (*sources)[c][s];
                auto &held=(*layouts)[c][s];
                const size_t rawCount=in.chain<in.scratch->chains.size()?in.scratch->chains[in.chain].lastBase.size():0;
                if (!held || !RrGeoSparseSourceHeld(in.program, source, rawCount)) {
                    const auto restRead=[&](uint32_t path,int32_t row) {
                        RrGeoRead at;at.rest=true;at.owner=in.rev;
                        at.path=path;at.row=row;
                        return at;
                    };
                    std::vector<RrVec3f> offsets;std::vector<int> indices;
                    RrGeoReadArray(in.program,in.scratch,restRead(source.offsetsPath,source.offsetsRow),&offsets);
                    RrGeoReadArray(in.program,in.scratch,restRead(source.indicesPath,source.indicesRow),&indices);
                    auto layout=std::make_shared<RrGeoBlendLayout>();
                    layout->pointCount=rawCount;
                    if(boundSample.shapeValid)
                        RigExecBuildBlendLayout(offsets,indices,rawCount,layout.get());
                    if(!held || !RigExecSameBlendLayout(*held,*layout))held=std::move(layout);
                    RrGeoNoteSparseSource(in.program, rawCount, &source);
                }
                sample.layout = held;
            }
            channel.samples.push_back(std::move(sample));
        }
        std::stable_sort(channel.samples.begin(), channel.samples.end(),
                         [](const RrGeoBlendSampleData &a,
                            const RrGeoBlendSampleData &b) {
                             return a.activation < b.activation;
                         });
        channels.push_back(std::move(channel));
    }
    if (!RrGeoSumBlendChannels(channels, base, deltas)) {
        deltas->clear();
    }
    return true;
}

void
RrGeoAssembleMatrix(const RrGeoAssembleInputs &in,
                    RrGeoMoverParameters *params)
{
    params->kind = RrGeoKindToken(RrGeoOpMatrix);
    params->enabled = RrGeoAssembleEnabled(in);
    if (!params->enabled) {
        params->valid = true;
        return;
    }
    // rigExec:weightBlend, exactly as RigExecAssembleMatrixParameters
    // reads it: "radial" turns a fraction of the rotation about the
    // transform's own pivot, "linear" -- the fallback -- blends the
    // position. A recorded read, so the runtime sees what the bake saw.
    params->radialWeight =
        RrGeoReadToken(in.program, in.scratch,
                       RrGeoAttrRead(in, RrGeoAttrWeightBlend),
                       "linear") == "radial";
    if (!in.rev->haveTransform) {
        return;
    }
    params->weights = RrGeoAssembleEnvelope(in, false);
    if (!params->weights.valid) {
        return;
    }
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (!std::isfinite(in.rev->transform[i][j])) {
                return;
            }
        }
    }
    if (in.rev->transform[0][3] != 0 || in.rev->transform[1][3] != 0 ||
        in.rev->transform[2][3] != 0 || in.rev->transform[3][3] != 1) {
        return;
    }
    params->transform = in.rev->transform;
    params->valid = true;
}

bool
RrGeoSkinMethodToken(const std::string &method)
{
    return method == "classicLinear" || method == "dualQuaternion";
}

void
RrGeoAssembleSkin(const RrGeoAssembleInputs &in,
                  RrGeoMoverParameters *params)
{
    params->kind = RrGeoKindToken(RrGeoOpSkin);
    params->enabled = RrGeoAssembleEnabled(in);
    if (!params->enabled) {
        params->valid = true;
        return;
    }
    // The packet carries the identity table and never the folded one: it
    // is assembled before the matrices are folded, so a chunk can start
    // on its own joints.
    const std::vector<RrMat4d> &table = in.rev->packetInfluences;
    if (in.wire->moverPrim == 0 || table.empty()) {
        return;
    }
    params->weights = RrGeoAssembleEnvelope(in, false);
    if (!params->weights.valid) {
        return;
    }
    params->skinTransforms = table;
    params->skinningMethod = RrGeoReadToken(
        in.program, in.scratch, RrGeoAttrRead(in, RrGeoAttrSkinningMethod),
        "classicLinear");
    if (in.rev->topologyResolved && in.rev->topology) {
        params->skinTopology = in.rev->topology;
        params->skinElementSize = in.rev->topology->elementSize;
        if (!in.rev->topology->validated ||
            params->skinTransforms.size() !=
                in.rev->topology->influenceCount) {
            return;
        }
        for (const RrMat4d &m : params->skinTransforms) {
            for (int r = 0; r < 4; ++r) {
                for (int c = 0; c < 4; ++c) {
                    if (!std::isfinite(m[r][c])) {
                        return;
                    }
                }
            }
            if (m[0][3] != 0 || m[1][3] != 0 || m[2][3] != 0 ||
                m[3][3] != 1) {
                return;
            }
        }
        if (!RrGeoSkinMethodToken(params->skinningMethod)) {
            return;
        }
        params->valid = true;
        return;
    }
    // The cache refused this mover: the layout reads per frame.
    RrGeoReadIntArray(in.program, in.scratch,
                      RrGeoAttrRead(in, RrGeoAttrJointIndices),
                      &params->skinIndices);
    RrGeoReadFloatArray(in.program, in.scratch,
                        RrGeoAttrRead(in, RrGeoAttrJointWeights),
                        &params->skinWeights);
    params->skinElementSize = RrGeoReadInt(
        in.scratch, RrGeoAttrRead(in, RrGeoAttrElementSize), 1);
    if (params->skinElementSize < 1 ||
        params->skinWeights.size() != params->skinIndices.size() ||
        params->skinIndices.size() % size_t(params->skinElementSize) != 0) {
        return;
    }
    // RigExecSkinLayout::Validate over the packet's own arrays: the point
    // count is not known here, so against its own length.
    {
        const size_t elementSize = size_t(params->skinElementSize);
        const size_t pointCount =
            params->skinIndices.size() / elementSize;
        if (params->skinIndices.size() != pointCount * elementSize ||
            params->skinTransforms.empty()) {
            return;
        }
        for (const RrMat4d &m : params->skinTransforms) {
            for (int r = 0; r < 4; ++r) {
                for (int c = 0; c < 4; ++c) {
                    if (!std::isfinite(m[r][c])) {
                        return;
                    }
                }
            }
            if (m[0][3] != 0 || m[1][3] != 0 || m[2][3] != 0 ||
                m[3][3] != 1) {
                return;
            }
        }
        for (size_t i = 0; i < params->skinIndices.size(); ++i) {
            if (params->skinIndices[i] < 0 ||
                size_t(params->skinIndices[i]) >=
                    params->skinTransforms.size()) {
                return;
            }
            if (!std::isfinite(params->skinWeights[i]) ||
                params->skinWeights[i] < 0.0f) {
                return;
            }
        }
    }
    if (!RrGeoSkinMethodToken(params->skinningMethod)) {
        return;
    }
    params->valid = true;
}

void
RrGeoAssembleBlendShape(const RrGeoAssembleInputs &in,
                        const std::vector<RrVec3f> &base,
                        RrGeoMoverParameters *params)
{
    RrGeoAssembleBlendDeltas(in, base, &params->blendDeltas);
    // rigExec:deltaSpace reads at Default and is never recorded, so the
    // runtime cannot replay it; no shipped rig authors anything but the
    // "target" default the bake therefore always saw. (Framework need:
    // record the token or carry it on the wire.)
    params->blendSurfaceFrame = false;
    params->valid = !params->blendDeltas.empty();
}

void
RrGeoAssembleLattice(const RrGeoAssembleInputs &in,
                     std::vector<RrVec3f> &&base,
                     RrGeoMoverParameters *params)
{
    params->restPoints = std::move(base);
    RrGeoReadBindingVec3fArray(
        in, RrGeoBindingRead(in, RrGeoBindCagePoints, true),
        &params->auxPoints);
    RrGeoReadBindingVec3fArray(
        in, RrGeoBindingRead(in, RrGeoBindCagePoints, false),
        &params->auxPointsB);
    params->divisions = RrGeoReadVec3i(
        in.scratch, RrGeoAttrRead(in, RrGeoAttrDivisions),
        RrVec3i(0, 0, 0));
    const size_t cageCount = size_t(params->divisions[0]) *
                             size_t(params->divisions[1]) *
                             size_t(params->divisions[2]);
    params->valid = params->divisions[0] >= 2 && params->divisions[1] >= 2 &&
                    params->divisions[2] >= 2 &&
                    params->auxPoints.size() == cageCount &&
                    params->auxPointsB.size() == cageCount &&
                    !params->restPoints.empty();
}

void
RrGeoAssembleWire(const RrGeoAssembleInputs &in,
                  RrGeoMoverParameters *params)
{
    RrGeoReadBindingVec3fArray(
        in, RrGeoBindingRead(in, RrGeoBindDriverCurvePoints, true),
        &params->restPoints);
    if (in.wire->binding->driverTransformCount > 0) {
        const size_t t = size_t(in.wire->binding->driverTransformCount);
        const size_t s = size_t(in.wire->binding->driverSpaceCount);
        const size_t bt =
            size_t(in.wire->binding->driverBaseTransformCount);
        // Every operation but a skin assembles against the folded table.
        const std::vector<RrMat4d> &table = in.rev->influences;
        if (table.size() < t + s + bt) {
            return;
        }
        const size_t bs = table.size() - t - s - bt;
        std::vector<float> weights, baseWeights;
        RrGeoReadFloatArray(in.program, in.scratch,
                            RrGeoAttrRead(in, RrGeoAttrDriverWeights),
                            &weights);
        RrGeoReadFloatArray(in.program, in.scratch,
                            RrGeoAttrRead(in, RrGeoAttrDriverBaseWeights),
                            &baseWeights);
        const auto pick = [](size_t count, size_t j) {
            return count <= 1 ? size_t(0) : j % count;
        };
        // Which frame the points this wire moves are already in, exactly as
        // RigExecAssembleParameters reads it: "posed" is a wire that runs
        // after the skin on its target, "rest" -- the fallback -- one that
        // runs before it. See rigExec:pointFrame in the schema.
        const bool posedPoints =
            RrGeoReadToken(in.program, in.scratch,
                           RrGeoAttrRead(in, RrGeoAttrPointFrame),
                           "rest") == "posed";
        // Which frame the driver's offset from its space is applied in,
        // exactly as RigExecAssembleParameters reads it: "local" (the
        // fallback) is T * S^-1, "posed" conjugates that into the space's
        // current frame. Both tokens are recorded reads, so the runtime
        // sees what the bake saw.
        const bool posedDelta =
            RrGeoReadToken(in.program, in.scratch,
                           RrGeoAttrRead(in, RrGeoAttrDriverDeltaFrame),
                           "local") == "posed";
        // MEASURE THE OFFSET IN THE ASSET'S OWN UNITS, and carry only the
        // finished displacement out of them. The curve, its bind distances
        // and the mesh rest points are authored at the asset's scale, while
        // the driver's offset from its space arrives in whatever scale that
        // space is in: under a master scaled to two the offset is twice as
        // long while the curve it moves has not grown. A space nothing has
        // scaled measures one, so both corrections vanish and an unscaled
        // rig is untouched, and so is a wire that asks for neither posed
        // frame.
        // Each selected driver/space pair is measured once per assembly call.
        struct _RrMeasuredWirePair {
            size_t driver;
            size_t space;
            RrMat4d m;
            RrVec3d scale;
        };
        std::vector<_RrMeasuredWirePair> measuredCache;
        const auto measured = [&](size_t first, size_t count,
                                  size_t spaceFirst, size_t spaceCount,
                                  size_t j, RrVec3d *spaceScale) {
            const size_t ti = first + pick(count, j);
            if (spaceCount == 0) {
                return table[ti];
            }
            const size_t si = spaceFirst + pick(spaceCount, j);
            for (const _RrMeasuredWirePair &hit : measuredCache) {
                if (hit.driver == ti && hit.space == si) {
                    if (spaceScale) {
                        *spaceScale = hit.scale;
                    }
                    return hit.m;
                }
            }
            RrMat4d m = table[ti];
            RrMat4d space = table[si];
            const RrVec3d k = RrGeoFrameScale(space);
            // Only for a wire that asked for a posed frame: one that
            // did not keeps the plain measurement bit for bit, as
            // RigExecMeasureWireDriver decides it.
            if ((posedPoints || posedDelta) &&
                k != RrVec3d(1.0, 1.0, 1.0)) {
                RrMat4d unscale(1.0);
                unscale.SetScale(
                    RrVec3d(1.0 / k[0], 1.0 / k[1], 1.0 / k[2]));
                m = m * unscale;
                space = space * unscale;
            }
            m = posedDelta ? RrGeoMeasureInPosedSpace(m, space)
                           : RrGeoMeasureInSpace(m, space);
            if (spaceScale) {
                *spaceScale = k;
            }
            measuredCache.push_back({ti, si, m, k});
            return m;
        };
        // With rigExec:space named the displacement is carried out by the
        // rig's whole rest->pose map instead of by the space's scale alone:
        // both polygons are transformed by the carry after the loop, and
        // the scale-only correction is NOT applied beside it, exactly as
        // RigExecAssembleParameters decides it. See RrGeoCarryWireCurves.
        const bool carried = posedPoints && in.rev->haveCarry;
        params->auxPoints.resize(params->restPoints.size());
        for (size_t j = 0; j < params->restPoints.size(); ++j) {
            RrVec3f &rest = params->restPoints[j];
            // The base motion cancels out of the displacement -- both
            // curves carry it -- so it stays in the asset's units with the
            // rest curve it moves.
            if (bt > 0) {
                const RrMat4d b =
                    measured(t + s, bt, t + s + bt, bs, j, nullptr);
                const float wb = baseWeights.empty()
                    ? 1.0f
                    : baseWeights[pick(baseWeights.size(), j)];
                const RrVec3f moved = RrGeoToVec3f(
                    b.TransformAffine(RrGeoToVec3d(rest)));
                rest = rest + (moved - rest) * wb;
            }
            RrVec3d scale(1.0, 1.0, 1.0);
            const RrMat4d m = measured(0, t, t, s, j, &scale);
            const float w = weights.empty()
                ? 1.0f
                : weights[pick(weights.size(), j)];
            const RrVec3f moved = RrGeoToVec3f(
                m.TransformAffine(RrGeoToVec3d(rest)));
            RrVec3f displacement = (moved - rest) * w;
            if (posedPoints && !carried) {
                displacement = RrVec3f(displacement[0] * float(scale[0]),
                                       displacement[1] * float(scale[1]),
                                       displacement[2] * float(scale[2]));
            }
            params->auxPoints[j] = rest + displacement;
        }
        if (carried) {
            RrGeoCarryWireCurves(&params->restPoints, &params->auxPoints,
                                 in.rev->carry);
        }
    } else {
        // A curve-driven wire's posed polygon comes from its own chain, so
        // there is no uncarried delta to carry: a named space is left alone.
        RrGeoReadBindingVec3fArray(
            in, RrGeoBindingRead(in, RrGeoBindDriverCurvePoints, false),
            &params->auxPoints);
    }
    {
        std::vector<int> order;
        RrGeoReadIntArray(in.program, in.scratch,
                          RrGeoBindingRead(in, RrGeoBindDriverCurveOrder,
                                           true),
                          &order);
        if (!order.empty()) {
            params->curveOrder = order[0];
        }
    }
    RrGeoReadDoubleArray(in.program, in.scratch,
                         RrGeoBindingRead(in, RrGeoBindDriverCurveKnots, true),
                         &params->curveKnots);
    params->dropoffDistance = double(RrGeoReadFloat(
        in.program, in.scratch, RrGeoAttrRead(in, RrGeoAttrDropoffDistance),
        0.0f));
    if (in.wire->binding->bindCoords != 0) {
        RrGeoReadVec2fArray(in.program, in.scratch,
                            RrGeoBindingRead(in, RrGeoBindBindCoords),
                            &params->wireBindCoords);
    }
    const RrGeoNurbsCurve rest{&params->restPoints, params->curveOrder,
                               &params->curveKnots};
    params->valid = rest.IsValid() &&
                    params->auxPoints.size() == params->restPoints.size() &&
                    !params->wireBindCoords.empty();
}

void
RrGeoAssembleRibbon(const RrGeoAssembleInputs &in, bool emitGuides,
                    RrGeoMoverParameters *params)
{
    const RrPointFrameArray *frames = nullptr;
    if (in.wire->driverFramesSolver >= 0 &&
        size_t(in.wire->driverFramesSolver) <
            in.program->store.aggregates.size()) {
        frames = &in.program->store
                      .aggregates[size_t(in.wire->driverFramesSolver)];
    }
    if (!frames || frames->frames.empty() ||
        frames->rests.size() != frames->frames.size()) {
        return;
    }
    params->frames = *frames;
    if (!emitGuides) {
        RrGeoReadVec2fArray(in.program, in.scratch,
                            RrGeoBindingRead(in, RrGeoBindBindCoords),
                            &params->bindCoords);
        params->valid = !params->bindCoords.empty();
    } else {
        params->valid = true;
    }
}

void
RrGeoAssembleDerived(const RrGeoAssembleInputs &in,
                     std::vector<RrVec3f> &&base, bool recomputeExtent,
                     RrGeoMoverParameters *params)
{
    params->auxPoints = std::move(base);
    RrGeoReadIntArray(in.program, in.scratch,
                      RrGeoBindingRead(in, RrGeoBindTopologyCounts),
                      &params->topologyCounts);
    RrGeoReadIntArray(in.program, in.scratch,
                      RrGeoBindingRead(in, RrGeoBindTopologyIndices),
                      &params->topologyIndices);
    if (recomputeExtent) {
        RrGeoReadFloatArray(in.program, in.scratch,
                            RrGeoBindingRead(in, RrGeoBindWidths),
                            &params->widths);
    }
    params->valid =
        !params->auxPoints.empty() &&
        (recomputeExtent || !params->topologyCounts.empty());
}

// A plugin mover's packet: its playback state, the frame bytes of the
// bake's run (the entry's v2_frame), and its phased point inputs: per
// phase input, whether its point binding answered this run and the points
// it answered with. No kernel here makes it a valid no-op; no frame bytes
// -- the bake's assembly failed -- leave it invalid.
void
RrGeoAssembleExternal(const RrGeoAssembleInputs &in,
                      RrGeoMoverParameters *params)
{
    RrProgram &program = *in.program;
    const auto found = program.externalIndex.find(
        std::make_pair(uint32_t(in.chain), uint32_t(in.revision)));
    if (in.derived || found == program.externalIndex.end()) {
        return;
    }
    const RrProgram::ExternalRevision &state =
        program.externals[found->second];
    params->external = &state;
    params->externalState = state.state.get();
    if (!state.state) {
        params->valid = true;
        return;
    }
    params->externalFrame = program.statics.ExternalFrame(found->second);
    const auto &externalWire=program.file->externalMovers[found->second];
    for(const auto &declaration:externalWire.declaredInputs) {
        RrGeoExternalInput value;
        const auto tag=declaration.read->tag;
        value.type=uint8_t(tag);
        const auto copyBytes=[&](const void *data,size_t bytes) {
            value.bytes.resize(bytes);
            if(bytes) std::memcpy(value.bytes.data(),data,bytes);
        };
        if(RigExecFormatIsArrayTag(tag)) {
            const void *raw=RrReadExternalArray(&program,declaration);
            const auto copyArray=[&](const auto *array) {
                if(!array) return;
                value.have=true; value.count=array->size();
                using T=typename std::decay_t<decltype(*array)>::value_type;
                copyBytes(array->data(),array->size()*sizeof(T));
            };
            switch(tag) {
            case RigExecWireInputTag::IntArray: copyArray(static_cast<const std::vector<int32_t> *>(raw)); break;
            case RigExecWireInputTag::FloatArray: copyArray(static_cast<const std::vector<float> *>(raw)); break;
            case RigExecWireInputTag::DoubleArray: copyArray(static_cast<const std::vector<double> *>(raw)); break;
            case RigExecWireInputTag::Vec2fArray: copyArray(static_cast<const std::vector<RrVec2f> *>(raw)); break;
            case RigExecWireInputTag::Vec3fArray: copyArray(static_cast<const std::vector<RrVec3f> *>(raw)); break;
            case RigExecWireInputTag::Vec3dArray: copyArray(static_cast<const std::vector<RigExecWireVec3d> *>(raw)); break;
            case RigExecWireInputTag::Matrix4dArray: copyArray(static_cast<const std::vector<RigExecWireMatrix4d> *>(raw)); break;
            case RigExecWireInputTag::BoolArray: copyArray(static_cast<const std::vector<uint8_t> *>(raw)); break;
            case RigExecWireInputTag::TokenArray:
                if(raw) {
                    const auto &tokens=*static_cast<const std::vector<uint32_t> *>(raw);
                    value.have=true; value.count=tokens.size();
                    for(uint32_t token:tokens) value.tokens.push_back(program.TextOrEmpty(token));
                }
                break;
            default: break;
            }
        } else {
            RrWireValue scalar;
            value.have=RrReadExternalScalar(&program,declaration,&scalar);
            if(value.have) {
                value.count=1;
                switch(tag) {
                case RigExecWireInputTag::Double: copyBytes(&scalar.bits,sizeof(double)); break;
                case RigExecWireInputTag::Float:
                case RigExecWireInputTag::Int: {
                    const uint32_t bits=uint32_t(scalar.bits); copyBytes(&bits,sizeof(bits)); break;
                }
                case RigExecWireInputTag::Bool: {
                    const uint8_t bit=scalar.bits?1:0; copyBytes(&bit,sizeof(bit)); break;
                }
                case RigExecWireInputTag::Token: value.token=program.TextOrEmpty(uint32_t(scalar.bits)); break;
                case RigExecWireInputTag::Matrix4d: copyBytes(scalar.matrix.data(),sizeof(double)*16); break;
                case RigExecWireInputTag::Vec3d: copyBytes(scalar.vec3d.data(),sizeof(double)*3); break;
                case RigExecWireInputTag::Vec3f: copyBytes(scalar.vec3f.data(),sizeof(float)*3); break;
                case RigExecWireInputTag::Vec3i: copyBytes(scalar.vec3i.data(),sizeof(int32_t)*3); break;
                default: value.have=false; value.count=0; break;
                }
            }
        }
        params->externalInputs.push_back(std::move(value));
    }
    if (!params->externalFrame && params->externalInputs.empty()) return;
    const RigExecWireRevisionBinding &binding = *in.wire->binding;
    for (size_t i = 0;
         i < binding.phaseInputs.size() && i < binding.phases.size(); ++i) {
        const uint32_t path = binding.phaseInputs[i];
        const std::vector<RrVec3f> *answer =
            RrGeoPhaseOverlay(*in.rev, *in.wire, path);
        params->externalPhasedPaths.push_back(program.TextOrEmpty(path));
        params->externalPhasedHave.push_back(answer ? 1 : 0);
        params->externalPhased.push_back(answer ? *answer
                                                : std::vector<RrVec3f>());
    }
    params->valid = true;
}

// The revision's packet. Chain revisions assemble against the authored
// base; derived ones against the chain's final points.
RrGeoMoverParameters
RrGeoAssembleRevision(const RrGeoAssembleInputs &in)
{
    RrGeoMoverParameters params;
    const int op = int(in.wire->op);
    RrGeoResolveRevisionPhases(in.program, *in.scratch, *in.wire, in.rev,
                               in.diagnostics);
    if (op == RrGeoOpMatrix) {
        RrGeoAssembleMatrix(in, &params);
        return params;
    }
    if (op == RrGeoOpSkin) {
        RrGeoAssembleSkin(in, &params);
        return params;
    }
    const bool synthesizedDerived = op == RrGeoOpRecomputeNormals ||
                                    op == RrGeoOpRecomputeExtent;
    params.kind = RrGeoKindToken(op);
    params.enabled =
        synthesizedDerived ? true : RrGeoAssembleEnabled(in);
    if (!params.enabled) {
        params.valid = true;
        return params;
    }
    params.weights = RrGeoAssembleEnvelope(in, synthesizedDerived);
    if (!params.weights.valid) {
        return params;
    }
    std::vector<RrVec3f> base;
    if (op != RrGeoOpMatrix && op != RrGeoOpSkin && op != RrGeoOpWire &&
        in.basePoints && in.basePointCount > 0) {
        base.assign(in.basePoints, in.basePoints + in.basePointCount);
    }
    switch (op) {
    case RrGeoOpBlendShape:
        RrGeoAssembleBlendShape(in, base, &params);
        break;
    case RrGeoOpVolumeCorrect:
        params.strength = 1.0f;
        if (!base.empty()) {
            params.referenceVolume =
                RrGeoBoundVolume(base.data(), base.size());
            params.valid = true;
        }
        break;
    case RrGeoOpSmooth:
        params.strength = 1.0f;
        RrGeoReadIntArray(in.program, in.scratch,
                          RrGeoBindingRead(in, RrGeoBindTopologyCounts),
                          &params.topologyCounts);
        RrGeoReadIntArray(in.program, in.scratch,
                          RrGeoBindingRead(in, RrGeoBindTopologyIndices),
                          &params.topologyIndices);
        params.valid = !params.topologyCounts.empty();
        break;
    case RrGeoOpDeltaMush: {
        const auto attr = [&](RrGeoAttr name, bool rest = false) {
            return RrGeoAttrRead(in, name, rest);
        };
        RrGeoReadBindingVec3fArray(in, attr(RrGeoAttrRestPoints, true),
                                 &params.restPoints);
        if (params.restPoints.empty()) params.restPoints = std::move(base);
        RrGeoReadIntArray(in.program, in.scratch,
                          RrGeoBindingRead(in, RrGeoBindTopologyCounts),
                          &params.topologyCounts);
        RrGeoReadIntArray(in.program, in.scratch,
                          RrGeoBindingRead(in, RrGeoBindTopologyIndices),
                          &params.topologyIndices);
        params.mushIterations = RrGeoReadInt(in.scratch,
            attr(RrGeoAttrIterations), 10);
        params.mushStep = RrGeoReadFloat(in.program, in.scratch,
            attr(RrGeoAttrStep), 0.5f);
        params.mushPinBorders = RrGeoReadBool(in.scratch,
            attr(RrGeoAttrPinBorders), true);
        params.mushDistanceWeight = RrGeoReadFloat(in.program, in.scratch,
            attr(RrGeoAttrDistanceWeight), 0.0f);
        params.mushDisplacement = RrGeoReadFloat(in.program, in.scratch,
            attr(RrGeoAttrDisplacement), 1.0f);
        params.valid = !params.restPoints.empty() && !params.topologyCounts.empty();
        break;
    }
    case RrGeoOpWrinkle: {
        const auto attr = [&](RrGeoAttr name, bool rest = false) {
            return RrGeoAttrRead(in, name, rest);
        };
        RrGeoReadBindingVec3fArray(in, attr(RrGeoAttrRestPoints, true),
                                 &params.restPoints);
        if (params.restPoints.empty()) params.restPoints = std::move(base);
        RrGeoReadIntArray(in.program, in.scratch,
                          RrGeoBindingRead(in, RrGeoBindTopologyCounts),
                          &params.topologyCounts);
        RrGeoReadIntArray(in.program, in.scratch,
                          RrGeoBindingRead(in, RrGeoBindTopologyIndices),
                          &params.topologyIndices);
        auto &settings = params.wrinkleSettings;
        settings.iterations = RrGeoReadInt(in.scratch,
            attr(RrGeoAttrIterations), 80);
        const std::string topology = RrGeoReadToken(in.program, in.scratch,
            attr(RrGeoAttrTopology, true), "cloth");
        if (topology == "cloth") {
            settings.topology = RigExecWrinkleTopology::Cloth;
        } else if (topology == "surfaceStruts") {
            settings.topology = RigExecWrinkleTopology::SurfaceStruts;
        } else {
            params.valid = false;
            break;
        }
        settings.neighborDistance = RrGeoReadInt(in.scratch,
            attr(RrGeoAttrNeighborDistance), 2);
        settings.restLengthScale = RrGeoReadFloat(in.program, in.scratch,
            attr(RrGeoAttrRestLengthScale), 1.0f);
        settings.stretchStiffness = RrGeoReadFloat(in.program, in.scratch,
            attr(RrGeoAttrStretchStiffness), 1.0f);
        settings.compressionStiffness = RrGeoReadFloat(in.program, in.scratch,
            attr(RrGeoAttrCompressionStiffness), 1.0f);
        settings.bendStiffness = RrGeoReadFloat(in.program, in.scratch,
            attr(RrGeoAttrBendStiffness), 0.1f);
        settings.maxDisplacement = RrGeoReadFloat(in.program, in.scratch,
            attr(RrGeoAttrMaxDisplacement), 0.2f);
        settings.pinBorders = RrGeoReadBool(in.scratch,
            attr(RrGeoAttrPinBorders), true);
        RrGeoReadIntArray(in.program, in.scratch,
                          attr(RrGeoAttrPinPoints, true),
                          &settings.pinPoints);
        settings.tangentPlaneCollisions = RrGeoReadBool(in.scratch,
            attr(RrGeoAttrTangentPlaneCollisions), true);
        settings.tangentPlaneInset = RrGeoReadFloat(in.program, in.scratch,
            attr(RrGeoAttrTangentPlaneInset), 0.0f);
        settings.wrinkleScale = RrGeoReadFloat(in.program, in.scratch,
            attr(RrGeoAttrWrinkleScale), 1.0f);
        settings.smoothingIterations = RrGeoReadInt(in.scratch,
            attr(RrGeoAttrSmoothingIterations), 0);
        params.valid = !params.restPoints.empty() && !params.topologyCounts.empty();
        break;
    }
    case RrGeoOpLattice:
        RrGeoAssembleLattice(in, std::move(base), &params);
        break;
    case RrGeoOpSurfaceProject:
        params.strength = 1.0f;
        RrGeoReadBindingVec3fArray(
            in, RrGeoBindingRead(in, RrGeoBindSurfacePoints),
            &params.auxPoints);
        RrGeoReadIntArray(in.program, in.scratch,
                          RrGeoBindingRead(in, RrGeoBindTopologyCounts),
                          &params.topologyCounts);
        RrGeoReadIntArray(in.program, in.scratch,
                          RrGeoBindingRead(in, RrGeoBindTopologyIndices),
                          &params.topologyIndices);
        params.valid =
            !params.auxPoints.empty() && !params.topologyCounts.empty();
        break;
    case RrGeoOpRibbon:
        RrGeoAssembleRibbon(in, false, &params);
        break;
    case RrGeoOpEmitGuidePoints:
        RrGeoAssembleRibbon(in, true, &params);
        break;
    case RrGeoOpWire:
        RrGeoAssembleWire(in, &params);
        break;
    case RrGeoOpRecomputeNormals:
        RrGeoAssembleDerived(in, std::move(base), false, &params);
        break;
    case RrGeoOpRecomputeExtent:
        RrGeoAssembleDerived(in, std::move(base), true, &params);
        break;
    case RrGeoOpExternal:
        RrGeoAssembleExternal(in, &params);
        break;
    default:
        break;
    }
    return params;
}

// The fold: influence tables out of the pose half's matrices. A port of
// FoldInfluences, FoldTransformForms, GatherChunkTransforms and SkinRange.

bool
RrGeoFoldInfluences(RrProgram *program, RrGeometryScratch *scratch,
                    const RigExecWireRevision &wire,
                    RrGeometryScratch::Revision *rev, bool *changedOut,
                    std::string *error)
{
    RrStore &store = program->store;
    const auto matrixAt = [&](int slot, const RrMat4d **out) {
        if (slot < 0 || size_t(slot) >= store.baseMatrix.size() ||
            size_t(slot) >= store.finalMatrix.size()) {
            return false;
        }
        *out = wire.finalPhase ? &store.finalMatrix[size_t(slot)]
                               : &store.baseMatrix[size_t(slot)];
        return true;
    };
    bool changed = false;
    rev->haveTransform = wire.transformSlot >= 0;
    if (rev->haveTransform) {
        const RrMat4d *matrix = nullptr;
        if (!matrixAt(wire.transformSlot, &matrix)) {
            if (error) {
                *error = "transform slot names no provider matrix";
            }
            return false;
        }
        rev->transform = *matrix;
    }
    // rigExec:space, out of the same phase-resolved matrix table the
    // transform came from, so this is what the baked fold reads. Held on
    // the revision for the assembly, exactly as the baked GeomRevision
    // holds it.
    rev->haveCarry = wire.carrySpaceSlot >= 0;
    if (rev->haveCarry) {
        const RrMat4d *carry = nullptr;
        if (!matrixAt(wire.carrySpaceSlot, &carry)) {
            if (error) {
                *error = "carry space slot names no provider matrix";
            }
            return false;
        }
        if (rev->carry != *carry) {
            rev->carry = *carry;
            changed = true;
        }
    }
    // A geometry-domain constraint's solved delta lands here, after the
    // provider read and before the phase substitution: the pose family
    // publishes per-frame deltaValues/deltaPresent on the store (the baked
    // B pair). Rigs without geometry-domain constraints --
    // constraintDelta < 0 everywhere -- never enter this arm.
    if (wire.constraintDelta >= 0 &&
        size_t(wire.constraintDelta) < store.deltaPresent.size() &&
        size_t(wire.constraintDelta) < store.deltaValues.size() &&
        store.deltaPresent[size_t(wire.constraintDelta)]) {
        rev->transform = store.deltaValues[size_t(wire.constraintDelta)];
        rev->haveTransform = true;
    }
    // An AtPrim phase reads the provider as one named writer left it, out of
    // the frame records bound at bake time, after the delta as the baked
    // fold orders them. The first record of a list whose FrameMatrix step
    // found it valid answers (the newest the walk made under the phase's
    // prim); a list with none valid leaves the dense-table matrix, silently.
    // `found` says whether one answered.
    const auto phased = [&](const std::vector<uint32_t> &records,
                            RrMat4d *matrix, bool *found) {
        *found = false;
        for (const uint32_t record : records) {
            if (size_t(record) >= store.frameMatrixValid.size() ||
                size_t(record) >= store.frameMatrix.size()) {
                if (error) {
                    *error = "a frame record list names no frame record";
                }
                return false;
            }
            if (store.frameMatrixValid[size_t(record)]) {
                *matrix = store.frameMatrix[size_t(record)];
                *found = true;
                return true;
            }
        }
        return true;
    };
    const bool atPrim = wire.binding->transformPhase.kind == 3;
    if (atPrim) {
        bool found = false;
        if (!phased(wire.transformRecords, &rev->transform, &found)) {
            return false;
        }
        if (found) {
            // Set rather than left alone, as the baked fold does.
            rev->haveTransform = true;
        }
    }
    const bool hasReference = wire.op == uint8_t(RrGeoOpMatrix) &&
                              !wire.influenceSlots.empty();
    const auto normalize = [&](size_t index, RrMat4d *value) {
        const RrMat4d *reference = nullptr;
        if (!matrixAt(wire.influenceSlots[index], &reference)) {
            if (error) *error = "reference slot names no provider matrix";
            return false;
        }
        *value = reference->GetInverse() * *value;
        (*value)[0][3] = (*value)[1][3] = (*value)[2][3] = 0.0;
        (*value)[3][3] = 1.0;
        return true;
    };
    if (rev->haveTransform && hasReference && !normalize(0, &rev->transform)) {
        return false;
    }
    if (rev->haveTransform && wire.transformSpaceSlot >= 0) {
        const RrMat4d *space = nullptr;
        if (!matrixAt(wire.transformSpaceSlot, &space)) {
            if (error) {
                *error = "transform space slot names no provider matrix";
            }
            return false;
        }
        const bool clusterPosedPoints =
            RrGeoReadToken(program, scratch,
                           RrGeoAttrRead(*rev, RrGeoAttrPointFrame),
                           "rest") == "posed";
        // The reference refines the SPACE first, then the carry consumes
        // it -- the twin of the same two steps in bakedGeometry.cpp.
        RrMat4d measuredSpace = *space;
        if (hasReference && wire.influenceSlots.size() > 1 &&
            !normalize(1, &measuredSpace)) {
            return false;
        }
        // rigExec:space, as read above. A POINTER, because a revision
        // naming no carry must take the untouched branch and not one
        // multiplied by an identity.
        const RrMat4d *carry = rev->haveCarry ? &rev->carry : nullptr;
        rev->transform = RrGeoClusterInPointFrame(
            RrGeoMeasureInSpace(rev->transform, measuredSpace), measuredSpace,
            clusterPosedPoints, carry);
    }
    if (rev->influences.size() != wire.influenceSlots.size()) {
        if (error) {
            *error = "influence table does not match its slots";
        }
        return false;
    }
    for (size_t k = 0; k < wire.influenceSlots.size(); ++k) {
        const int slot = wire.influenceSlots[k];
        const RrMat4d *matrix = nullptr;
        if (!matrixAt(slot, &matrix)) {
            if (error) {
                *error = "influence slot names no provider matrix";
            }
            return false;
        }
        RrMat4d entry = *matrix;
        // Per entry, the entry's own provider's records: a matrix mover's
        // reference providers. The Matrix op does not read them, but its
        // packet carries this table into the executed decision.
        if (atPrim && k < wire.influenceRecords.size()) {
            bool found = false;
            if (!phased(wire.influenceRecords[k].v, &entry, &found)) {
                return false;
            }
        }
        if (rev->influences[k] != entry) {
            rev->influences[k] = entry;
            changed = true;
        }
    }
    if (changedOut) {
        *changedOut = changed;
    }
    return true;
}

void
RrGeoFoldTransformForms(RrGeometryScratch::Revision *rev, bool useSimd)
{
    const size_t count = rev->influences.size();
    if (rev->parameters.skinningMethod == "dualQuaternion") {
        rev->rows.clear();
        rev->palette.resize(count + 1);
        for (size_t t = 0; t < count; ++t) {
            rev->palette[t] =
                RrGeoScaledDualQuatFromMatrix(rev->influences[t]);
        }
        rev->palette[count] = RrGeoScaledDualQuat();
        return;
    }
    rev->palette.clear();
    if (!useSimd) {
        rev->rows.clear();
        return;
    }
    rev->rows.resize(count * size_t(RrGeoSkinRowStride));
    for (size_t t = 0; t < count; ++t) {
        RrGeoNarrowSkinRows(rev->influences[t],
                            &rev->rows[t * size_t(RrGeoSkinRowStride)]);
    }
}

RrGeoSkinTransformsView
RrGeoWholeTransformsView(RrGeometryScratch::Revision *rev)
{
    RrGeoSkinTransformsView view;
    view.transforms = rev->influences.data();
    view.transformCount = rev->influences.size();
    if (!rev->rows.empty()) {
        view.rows = rev->rows.data();
    }
    if (!rev->palette.empty()) {
        view.palette = rev->palette.data();
        view.paletteSize = rev->palette.size();
    }
    return view;
}

RrGeoSkinTransformsView
RrGeoChunkTransformsView(const RrGeometryScratch::Chunk &chunk, bool useSimd)
{
    RrGeoSkinTransformsView view;
    view.transforms = chunk.transforms.data();
    view.transformCount = chunk.transforms.size();
    if (useSimd && !chunk.rows.empty()) {
        view.rows = chunk.rows.data();
    }
    if (!chunk.palette.empty()) {
        view.palette = chunk.palette.data();
        view.paletteSize = chunk.palette.size();
    }
    return view;
}

bool
RrGeoGatherChunkTransforms(RrProgram *program,
                           const RigExecWireRevision &wire,
                           const RrGeometryScratch::Revision &rev,
                           RrGeometryScratch::Chunk *chunk,
                           std::string *error)
{
    const RrStore &store = program->store;
    const bool dualQuat =
        rev.parameters.skinningMethod == "dualQuaternion";
    const size_t count = chunk->transforms.size();
    const bool splitAlready = chunk->palette.size() == count + 1;
    for (const int32_t position : chunk->key) {
        if (position < 0 ||
            size_t(position) >= wire.influenceSlots.size() ||
            size_t(position) >= chunk->transforms.size() ||
            (size_t(position) + 1) * size_t(RrGeoSkinRowStride) >
                chunk->rows.size()) {
            if (error) {
                *error = "chunk key names no influence";
            }
            return false;
        }
        const int slot = wire.influenceSlots[size_t(position)];
        if (slot < 0) {
            continue;
        }
        if (size_t(slot) >= store.baseMatrix.size() ||
            size_t(slot) >= store.finalMatrix.size()) {
            if (error) {
                *error = "chunk influence slot names no provider matrix";
            }
            return false;
        }
        const RrMat4d &matrix = wire.finalPhase
            ? store.finalMatrix[size_t(slot)]
            : store.baseMatrix[size_t(slot)];
        if (chunk->transforms[size_t(position)] == matrix) {
            continue;
        }
        chunk->transforms[size_t(position)] = matrix;
        chunk->keyChanged = true;
        RrGeoNarrowSkinRows(
            matrix,
            &chunk->rows[size_t(position) * size_t(RrGeoSkinRowStride)]);
        if (splitAlready &&
            size_t(position) < chunk->palette.size()) {
            chunk->palette[size_t(position)] =
                RrGeoScaledDualQuatFromMatrix(matrix);
        }
    }
    if (dualQuat && !splitAlready) {
        chunk->palette.resize(count + 1);
        for (size_t t = 0; t < count; ++t) {
            chunk->palette[t] =
                RrGeoScaledDualQuatFromMatrix(chunk->transforms[t]);
        }
        chunk->palette[count] = RrGeoScaledDualQuat();
    }
    return true;
}

// SkinPacketIsUsable (bakedGeometry.cpp): the skin's validation over the half
// RevisionStatic holds, for its decision and every chunk's guard.
bool
RrGeoSkinPacketIsUsable(const RrGeometryScratch::Revision &rev)
{
    return rev.parameters.valid &&
           rev.parameters.kind == RrGeoKindToken(RrGeoOpSkin) &&
           rev.layoutUsable && rev.envelopeOk;
}

// SkinAcceptance (bakedGeometry.cpp): past that half a linear blend cannot
// fail; a dual-quaternion blend can still be degenerate at a vertex.
RrGeoAcceptance
RrGeoSkinAcceptance(const RrGeometryScratch::Revision &rev)
{
    if (!RrGeoSkinPacketIsUsable(rev)) {
        return RrGeoAcceptance::Refuses;
    }
    switch (RrGeoSkinMethodOf(rev.parameters)) {
    case RrGeoSkinMethod::ClassicLinear:
        return RrGeoAcceptance::Applies;
    case RrGeoSkinMethod::DualQuaternion:
        return RrGeoAcceptance::Deferred;
    case RrGeoSkinMethod::Unknown:
        break;
    }
    return RrGeoAcceptance::Refuses;
}

// \p target is the buffer written, at the full count: staging for a chunk.
bool
RrGeoSkinRange(RrGeometryScratch::Revision *rev, const RrVec3f *preceding,
               const RrGeoSkinTransformsView &view, size_t begin,
               size_t end, bool whole, bool useSimd,
               std::vector<RrVec3f> *target)
{
    std::vector<RrVec3f> &out = *target;
    if (end > begin && preceding) {
        std::copy(preceding + begin, preceding + end,
                  out.begin() + long(begin));
    }
    if (whole) {
        if (!RrGeoApplySkinKernelWithTransforms(rev->parameters, view,
                                                &out, useSimd)) {
            return false;
        }
    } else if (!RrGeoApplySkinKernelRange(rev->parameters, view, begin,
                                           end, &out, useSimd)) {
        return false;
    }
    if (!rev->fullStrength) {
        if (whole) {
            RrGeoBlendEnvelopeAll(preceding, rev->envelope.data(), end,
                                  out.data());
        } else {
            RrGeoBlendEnvelopeRange(preceding, rev->envelope.data(), begin,
                                    end, out.data());
        }
    }
    return true;
}

// Into \p fused, never staging: staging holds the ranges the chunks'
// published RevisionOut keys describe (the program's FuseWholeRevision).
bool
RrGeoFuseWholeRevision(const RrGeometryScratch::Chain &chain,
                       RrGeometryScratch::Revision *rev, size_t revisionIndex,
                       bool useSimd, std::vector<RrVec3f> *fused)
{
    const RrVec3f *points = nullptr;
    size_t count = 0;
    RrGeoPointsBefore(chain, revisionIndex, &points, &count);
    if (!rev->layoutUsable || !rev->envelopeOk ||
        count != rev->precedingCount) {
        return false;
    }
    fused->resize(count);
    return RrGeoSkinRange(rev, points, RrGeoWholeTransformsView(rev), 0,
                          count, true, useSimd, fused);
}

// A port of RigExecBakedAdoptPartition. The chunk keys stay Build's,
// because each chunk step's reads were declared from its key: a layout
// is adopted as the partition only when it holds the indices and element
// size the keys were cut from (\p build, the file's partition layout).
// Otherwise the partition stays, and partitionStale runs the revision
// whole until a layout with Build's arrays returns.
void
RrGeoAdoptPartition(RrGeometryScratch::Revision *rev,
                    const RrGeoSkinTopology *build)
{
    if (!rev->chunked || !rev->topology ||
        rev->topology == rev->partitionTopology) {
        return;
    }
    if (build && rev->topology->elementSize == build->elementSize &&
        rev->topology->indices == build->indices) {
        rev->partitionTopology = rev->topology;
    }
}

void
RrGeoResetRevision(RrGeometryScratch::Revision *rev)
{
    rev->created = true;
    rev->ran = false;
    rev->output.clear();
    rev->stagingOutput.clear();
    rev->stagingFresh = false;
    rev->passedPoints.clear();
    rev->currentSource = -1;
    rev->lastParameters = RrGeoMoverParameters();
    rev->lastAuxPoints.clear();
    rev->lastStatus = RrGeoMoverStatus();
}

// \p chain's base points this run: its base slot's elements, else the ones
// the bake captured.
const std::vector<RrVec3f> &
RrGeoChainBase(const RrProgram *program,
               const RrGeometryScratch::Chain &chain)
{
    if (chain.baseSlot >= 0) {
        if (const std::vector<RrVec3f> *held =
                RrInputArray<RrVec3f>(program, uint32_t(chain.baseSlot))) {
            return *held;
        }
        return chain.sampleMissingBase;
    }
    return chain.staticBase;
}

// The owning layout operation consumes its captured typed leaf routes and
// publishes an exact current topology, including missing-at-bake recovery.
// Equal topology bits retain the existing immutable handle.
void
RrGeoRunLayoutOp(const RrProgram *program, const RrGeometryScratch &scratch,
                 const RigExecWireRevision &wire, size_t id,
                 RrGeometryScratch::Revision *rev)
{
    const std::vector<int32_t> *indices=nullptr;
    const std::vector<float> *weights=nullptr;
    int elementSize=1;
    for(const auto &site:wire.layoutLeafSites) {
        if(site.read->tag==RigExecWireInputTag::IntArray)
            indices=static_cast<const std::vector<int32_t> *>(RrReadExternalArray(program,site));
        else if(site.read->tag==RigExecWireInputTag::FloatArray)
            weights=static_cast<const std::vector<float> *>(RrReadExternalArray(program,site));
        else if(site.read->tag==RigExecWireInputTag::Int) {
            RrWireValue value;
            if(RrReadExternalScalar(program,site,&value)) elementSize=int32_t(uint32_t(value.bits));
        }
    }
    auto built = std::make_shared<RrGeoSkinTopology>();
    if (indices) built->indices.assign(indices->begin(), indices->end());
    if (weights) built->weights.assign(weights->begin(), weights->end());
    built->elementSize = elementSize;
    built->influenceCount = wire.influenceSlots.size();
    if (elementSize >= 1 && built->indices.size() == built->weights.size() &&
        built->indices.size() % size_t(elementSize) == 0) {
        built->pointCount = built->indices.size() / size_t(elementSize);
    }
    built->validated = RigExecFormatSkinLayoutValidates(
        built->indices.data(), built->indices.size(), built->weights.data(),
        built->weights.size(), elementSize, built->influenceCount);
    if (id < scratch.epochTopologies.size() && scratch.epochTopologies[id] &&
        *scratch.epochTopologies[id] == *built) {
        rev->layoutHandle = scratch.epochTopologies[id];
        return;
    }
    if (rev->layoutHandle && *rev->layoutHandle == *built) {
        return;
    }
    rev->layoutHandle = std::move(built);
}

}  // namespace

bool
RrRunTopologyHead(RrProgram *program, size_t id)
{
    auto *scratch = static_cast<RrGeometryScratch *>(program->geo.get());
    if (!scratch) return false;
    const auto &geo = *program->geometry;
    // Each layout read resolves lazily in this owning operation.
    const bool derived = id >= geo.revisionIndex.size();
    const size_t local = derived ? id - geo.revisionIndex.size() : id;
    if (derived && local >= geo.derivedIndex.size()) return false;
    const auto &index = derived ? geo.derivedIndex[local] : geo.revisionIndex[local];
    auto &revision = derived ? scratch->derived[local].revision
        : scratch->chains[size_t(index.first)].revisions[size_t(index.second)];
    const auto &wire = derived ? *geo.chains[size_t(index.first)].derived[size_t(index.second)].revision
        : geo.chains[size_t(index.first)].revisions[size_t(index.second)];
    const auto previous = revision.layoutHandle;
    RrGeoRunLayoutOp(program, *scratch, wire, id, &revision);
    return previous != revision.layoutHandle &&
        (!previous || !revision.layoutHandle || !(*previous == *revision.layoutHandle));
}

bool
RrGeometrySkinLayoutIsOpenForTesting(const RrProgram *program,
                                     const std::string &moverPath)
{
    const auto *scratch =
        static_cast<const RrGeometryScratch *>(program->geo.get());
    if (!scratch) {
        return false;
    }
    const RigExecWireDomainGeometry &geo = *program->geometry;
    for (size_t c = 0; c < geo.chains.size() && c < scratch->chains.size();
         ++c) {
        const auto &revisions = geo.chains[c].revisions;
        for (size_t r = 0; r < revisions.size() &&
                           r < scratch->chains[c].revisions.size();
             ++r) {
            if (program->TextOrEmpty(revisions[r].moverPath) != moverPath) {
                continue;
            }
            const size_t id = size_t(geo.chainRevisionBegin[c] + int(r));
            const auto &topology = scratch->chains[c].revisions[r].topology;
            return topology && id < scratch->epochTopologies.size() &&
                   topology == scratch->epochTopologies[id];
        }
    }
    return false;
}

bool
RrPrologueGeometry(RrProgram *program,
                   std::vector<std::string> *poseDiagnostics,
                   std::string *error)
{
    (void)poseDiagnostics;
    if (!program || !program->geometry || !program->geo) {
        if (error) {
            *error = "geometry prologue has no program";
        }
        return false;
    }
    RrGeometryScratch *scratch = RrGeoScratch(program);
    RrStore &store = program->store;
    const RigExecWireDomainGeometry &geo = *program->geometry;
    if (scratch->chains.size() != geo.chains.size() ||
        scratch->derived.size() != geo.derivedIndex.size()) {
        if (error) {
            *error = "geometry scratch does not match its chains";
        }
        return false;
    }
    // No step runs yet: revisions with equal lattice binds share one, as
    // RigExecBakedShareLatticeBinds does.
    {
        RigExecLatticeBindSharing<RrVec3f> binds;
        for (RrGeometryScratch::Chain &chain : scratch->chains) {
            for (RrGeometryScratch::Revision &rev : chain.revisions) {
                binds.Offer(&rev.surfaceCache);
            }
        }
        for (RrGeometryScratch::Derived &derived : scratch->derived) {
            binds.Offer(&derived.revision.surfaceCache);
        }
    }

    if(!program->requiredStageFramesAdmission.admitted)return true;

    // Sample transport only. Adoption/reset belongs to the graph consumer.
    for (size_t c=0;c<geo.chains.size();++c) {
        auto &chain=scratch->chains[c];
        chain.sampleHaveBase=chain.baseSlot>=0
            ?RrInputHasValue(program,uint32_t(chain.baseSlot)):geo.chains[c].haveBase;
        // ChainInput's content version moves at publication, against the
        // base it last published (RrGeometryPublishChainInputs).
        const std::vector<RrVec3f> &base=RrGeoChainBase(program,chain);
        if(!RrGeoPointBitsEqual(base,chain.sampledBase)) {
            chain.sampledBase=base; chain.sampledMoved=true;
        }
        chain.createdCount=0; chain.scheduleCount=0;
    }
    for (size_t d=0;d<scratch->derived.size();++d) {
        auto &derived=scratch->derived[d];
        const auto &index=geo.derivedIndex[d];
        const auto &wire=geo.chains[size_t(index.first)].derived[size_t(index.second)];
        derived.sampleHaveBase=wire.baseSlot>=0
            ?RrInputHasValue(program,uint32_t(wire.baseSlot)):wire.haveBase;
        if(wire.baseSlot>=0) {
            const auto *points=RrInputArray<RrVec3f>(program,uint32_t(wire.baseSlot));
            derived.sampledBase=points?*points:std::vector<RrVec3f>();
        } else derived.sampledBase=derived.base;
        derived.createdCount=0; derived.scheduleCount=0;
    }
    return true;
}

bool RrRunChainInputs(RrProgram *program, size_t c, std::string *error)
{
    auto *scratch=RrGeoScratch(program);
    const auto &geo=*program->geometry;
    auto &store=program->store;
    if(c>=scratch->chains.size()) {
        if(error)*error="ChainInputs names no chain";
        return false;
    }
    auto &chain=scratch->chains[c];
    const bool haveBase=chain.sampleHaveBase;
    const auto &basePoints=chain.sampledBase;
    chain.haveBase=haveBase;
    store.chainHaveBase[c]=haveBase?1:0;
        if (!haveBase) {
            // Derived haveBase flags live densely by derived id; clear
            // the ones of this chain.
            for (size_t d = 0; d < geo.derivedIndex.size(); ++d) {
                if (geo.derivedIndex[d].first == int(c) &&
                    d < store.derivedHaveBase.size()) {
                    store.derivedHaveBase[d] = 0;
                }
            }
            return true;
        }
        if (chain.haveResult && basePoints.size() != chain.lastBase.size()) {
            // ChainStatus republishes in this run at the new count, so the
            // two bumps cannot return to the bytes last published.
            if (!chain.result.empty()) ++chain.resultVersion;
            chain.haveResult = false;
            chain.result.clear();
            chain.scheduleDirty = true;
            for (RrGeometryScratch::Revision &rev : chain.revisions) {
                RrGeoResetRevision(&rev);
            }
        }
        for (RrGeometryScratch::Revision &rev : chain.revisions) {
            if (rev.created) {
                ++chain.createdCount;
                rev.created = false;
            }
        }
        if (chain.scheduleDirty && !chain.revisions.empty()) {
            ++chain.scheduleCount;
        }
        chain.scheduleDirty = false;
        const bool sameBase = RrGeoPointBitsEqual(basePoints,chain.lastBase);
        const bool baseDirty = !chain.haveResult || !sameBase;
        if (c < store.chainBaseDirty.size()) {
            store.chainBaseDirty[c] = baseDirty ? 1 : 0;
        }
        if (baseDirty) {
            // ChainBase's content version moves exactly when the bytes do.
            if (!sameBase) ++chain.baseVersion;
            chain.lastBase = basePoints;
        }
    return true;
}

namespace {

// A surface projector target, as RigExecRunProjectorTarget runs it on the
// USD side: the providers' world frames out of the rest frames and the
// base and final matrix tables, the recorded settings and dials, and the
// shared kernel on the chain's authored and final points.
bool
RrGeoRunProjectorTarget(RrProgram *program, RrGeometryScratch *scratch,
                        size_t chainIndex, const RigExecWireRevision &wire,
                        const RrGeometryScratch::Revision &rev,
                        size_t object, RrStepOutput *output)
{
    RrStore &store = program->store;
    if (object >= store.derivedPublish.size()) {
        return false;
    }
    RrDerivedPublish &publish = store.derivedPublish[object];
    publish.matrixTarget = true;
    publish.haveMatrix = false;
    publish.haveBase = true;
    output->counters.revisionsBuilt = 1;
    output->counters.chainsBuilt = 1;
    if (wire.op == RrGeoOpShaderDials) {
        std::vector<double> dials;
        dials.reserve(wire.shaderDials.size());
        for (size_t k = 0; k < wire.shaderDials.size(); ++k) {
            RrGeoRead dial;
            dial.owner = &rev;
            dial.path = wire.shaderDials[k];
            dial.row = k < rev.dialRead.size() ? rev.dialRead[k] : -1;
            dials.push_back(RrGeoReadDouble(program, scratch, dial, 0.0));
        }
        publish.matrix = RigExecPackShaderDialsT<RrMat4d>(dials);
        publish.haveMatrix = true;
        return true;
    }
    const auto vec = [&](RrGeoAttr attr, const RrVec3d &fallback) {
        const auto read =
            RrGeoPathRead(scratch, RrGeoAttrRead(rev, attr));
        if (read && read->tag == RrPathValue::Tag::Vec3d) {
            return RrVec3d(read->vec[0], read->vec[1], read->vec[2]);
        }
        return fallback;
    };
    RigExecSurfaceProjectorInputs<RrMat4d, RrVec3d> in;
    in.rayOrigin = vec(RrGeoAttrRayOrigin, RrVec3d(0.0, 0.0, 0.0));
    in.rayDirection = vec(RrGeoAttrRayDirection, RrVec3d(0.0, 0.0, 1.0));
    in.rayUp = vec(RrGeoAttrRayUp, RrVec3d(0.0, 1.0, 0.0));
    in.shaderOffset.SetIdentity();
    if (const auto read = RrGeoPathRead(
            scratch, RrGeoAttrRead(rev, RrGeoAttrShaderOffset))) {
        if (read->tag == RrPathValue::Tag::Matrix4d) {
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    in.shaderOffset[i][j] = read->matrix[size_t(i * 4 + j)];
                }
            }
        }
    }
    in.reproject =
        RrGeoReadToken(program, scratch,
                       RrGeoAttrRead(rev, RrGeoAttrProjectionMode),
                       "material") == "reproject";
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            in.worldToMesh[i][j] = wire.meshWorldInverse[size_t(i * 4 + j)];
        }
    }
    // Providers: transform, transformSpace and carry slots, each a world
    // frame as rest * computeMatrix at base and at final.
    const RrPoseScratch *pose = runtimePoseDetail::_RrScratch(program);
    const int32_t slots[3] = {wire.transformSlot, wire.transformSpaceSlot,
                              wire.carrySpaceSlot};
    // Named means slotted: the bake refuses a named provider it cannot
    // give a slot, so the two are the same set here.
    const bool named[3] = {slots[0] >= 0, slots[1] >= 0, slots[2] >= 0};
    bool resolved[3] = {false, false, false};
    RrMat4d base[3], fin[3];
    for (int k = 0; k < 3; ++k) {
        base[k].SetIdentity();
        fin[k].SetIdentity();
        const int32_t slot = slots[k];
        if (!named[k] || !pose ||
            size_t(slot) >= pose->restFrames.size() ||
            size_t(slot) >= store.baseMatrix.size() ||
            size_t(slot) >= store.finalMatrix.size() ||
            !pose->restFrames[size_t(slot)].IsValid()) {
            continue;
        }
        RrMat4d rest;
        rest.SetIdentity();
        RrPointsToMatrix(RrIdentityLandmarks(),
                         pose->restFrames[size_t(slot)].points, &rest);
        base[k] = rest * store.baseMatrix[size_t(slot)];
        fin[k] = rest * store.finalMatrix[size_t(slot)];
        resolved[k] = true;
    }
    in.hasSource = named[0] && resolved[0];
    in.sourceBase = base[0];
    in.sourceFinal = fin[0];
    in.sourceSpaceNamed = named[1];
    in.hasSourceSpace = named[1] && resolved[1];
    in.sourceSpaceBase = base[1];
    in.sourceSpaceFinal = fin[1];
    in.spaceNamed = named[2];
    in.hasSpace = named[2] && resolved[2];
    in.spaceFinal = fin[2];
    std::vector<int> counts, indices;
    RrGeoReadIntArray(program, scratch,
                      RrGeoBindingRead(rev, RrGeoBindTopologyCounts),
                      &counts);
    RrGeoReadIntArray(program, scratch,
                      RrGeoBindingRead(rev, RrGeoBindTopologyIndices),
                      &indices);
    const std::vector<RrVec3f> &basePoints =
        RrGeoChainBase(program, scratch->chains[chainIndex]);
    const std::vector<RrVec3f> &finalPoints = scratch->chains[chainIndex].result.Read();
    publish.haveMatrix = RigExecSolveSurfaceProjectorT(
        in, basePoints, finalPoints, counts, indices,
        [](const std::vector<RrVec3f> &points, const std::vector<int> &c,
           const std::vector<int> &i) {
            return RrGeoComputeVertexNormals(points, c, i);
        },
        program->TextOrEmpty(wire.moverPath), &publish.matrix,
        &output->diagnostics,&rev.surfaceCache);
    return true;
}

bool
RrGeoRunDerivedStep(RrProgram *program, RrGeometryScratch *scratch,
                    size_t step, std::string *error)
{
    RrStore &store = program->store;
    const RigExecWireDomainGeometry &geo = *program->geometry;
    const int object = (*program->steps)[step].object;
    RrStepOutput &output = store.stepOutputs[step];
    if (object < 0 || size_t(object) >= geo.derivedIndex.size() ||
        size_t(object) >= scratch->derived.size()) {
        if (error) {
            *error = RrGeoStepLabel(program, step) + " names no target";
        }
        return false;
    }
    const size_t chainIndex = size_t(geo.derivedIndex[size_t(object)].first);
    const size_t derivedIndex =
        size_t(geo.derivedIndex[size_t(object)].second);
    if (chainIndex >= scratch->chains.size() ||
        derivedIndex >= geo.chains[chainIndex].derived.size()) {
        if (error) {
            *error = RrGeoStepLabel(program, step) + " names no target";
        }
        return false;
    }
    RrGeometryScratch::Chain &chain = scratch->chains[chainIndex];
    RrGeometryScratch::Derived &derived = scratch->derived[size_t(object)];
    derived.haveBase=derived.sampleHaveBase;
    const auto &derivedBase=derived.sampledBase;
    store.derivedHaveBase[size_t(object)]=derived.haveBase?1:0;
    // DerivedOut's content version moves exactly when this step leaves
    // `result` holding other bytes than it last published; a size reset
    // keeps the old array alive here to compare against.
    RrRetainedArray<RrVec3f> resetFrom;
    const auto noteReset=[&] {
        if(!resetFrom.empty()) ++derived.resultVersion;
    };
    if(derived.haveBase) {
        auto &rev=derived.revision;
        if(derived.haveResult && derivedBase.size()!=derived.lastBase.size()) {
            resetFrom=derived.result;
            derived.haveResult=false; derived.result.clear(); RrGeoResetRevision(&rev);
        }
        if(rev.created) { ++derived.createdCount; ++derived.scheduleCount; rev.created=false; }
        derived.baseDirty=!derived.haveResult || !RrGeoPointBitsEqual(derivedBase,derived.lastBase);
        if(derived.baseDirty) derived.lastBase=derivedBase;
        if(geo.chains[chainIndex].derived[derivedIndex].revision->op==uint8_t(RrGeoOpSkin)) {
            rev.topology=rev.layoutHandle; rev.topologyResolved=true;
        }
    }

    const bool haveBase = chainIndex < store.chainHaveBase.size() &&
                          store.chainHaveBase[chainIndex] != 0;
    const bool derivedHaveBase = size_t(object) < store.derivedHaveBase.size() &&
                                 store.derivedHaveBase[size_t(object)] != 0;
    if (size_t(object) < store.derivedPublish.size()) {
        store.derivedPublish[size_t(object)].haveBase = false;
    }
    if (!haveBase || !derivedHaveBase) {
        noteReset();
        return true;
    }
    const RigExecWireRevision &wire =
        *geo.chains[chainIndex].derived[derivedIndex].revision;
    if (!RrGeoOpName(wire.op)) {
        if (error) {
            *error = RrGeoStepLabel(program, step) +
                     " references unknown revision op " +
                     std::to_string(wire.op);
        }
        noteReset();
        return false;
    }
    if (wire.op == RrGeoOpSurfaceProjector ||
        wire.op == RrGeoOpShaderDials) {
        noteReset();
        derived.haveResult = true;
        return RrGeoRunProjectorTarget(program, scratch, chainIndex, wire,
                                       derived.revision, size_t(object),
                                       &output);
    }
    RrGeometryScratch::Revision &rev = derived.revision;
    // The fold's answer is dropped for a derived revision, exactly as
    // the baked step drops it -- but the tables are still folded.
    bool dropped = false;
    if (!RrGeoFoldInfluences(program, scratch, wire, &rev, &dropped,
                             error)) {
        if (error) {
            *error = RrGeoStepLabel(program, step) + ": " + *error;
        }
        return false;
    }
    RrGeoAssembleInputs in;
    in.program = program;
    in.scratch = scratch;
    in.wire = &wire;
    in.rev = &rev;
    in.basePoints = chain.result.empty() ? nullptr : chain.result.data();
    in.basePointCount = chain.result.size();
    in.chain = chainIndex;
    in.revision = derivedIndex;
    in.derived = true;
    in.diagnostics = &output.diagnostics;
    RrGeoMoverParameters parameters = RrGeoAssembleRevision(in);
    const RrGeoMoverStatus status = RrGeoStatusForParameters(
        parameters, program->TextOrEmpty(wire.moverPath));
    output.counters.revisionsBuilt = 1;
    // A selected derived operation computes its declared typed result.
    {
        output.counters.revisionsExecuted = 1;
        const bool unseededDerived=(wire.op==RrGeoOpRecomputeNormals || wire.op==RrGeoOpRecomputeExtent) && RrGeoEnvelopeIsFullStrength(parameters.weights);
        std::vector<RrVec3f> values;
        if(!unseededDerived)values.assign(derived.lastBase.begin(),derived.lastBase.end());
        const bool applied = status.AllowsApply() &&
            (unseededDerived
                ? parameters.valid && parameters.kind==RrGeoKindToken(int(wire.op)) &&
                  RrGeoApplyDerivedKernel(int(wire.op),parameters,&values,derived.lastBase.size())
                : RrGeoRunRevisionKernel(int(wire.op), parameters, &values,
                    &scratch->wireBasis,program->geoSettings.useSimd,
                    &rev.surfaceCache,&rev.wireRestCache,&rev.lastWireBasis));
        rev.resultStatus = status.state;
        if (!applied) {
            values.assign(derived.lastBase.begin(), derived.lastBase.end());
            if (status.AllowsApply()) {
                rev.resultStatus = "moverFailed";
            }
        }
        rev.output = std::move(values);
        parameters.auxPoints.clear();
        rev.lastParameters = std::move(parameters);
        // Selected graph body does not retain a duplicate auxiliary snapshot.
        rev.lastStatus = status;
        rev.ran = true;
    }
    if (rev.resultStatus == "moverFailed") {
        output.diagnostics.push_back(
            "MoverFailed " +
            program->TextOrEmpty(
                geo.chains[chainIndex].derived[derivedIndex].target) +
            ": derived geometry input/cardinality validation failed");
    }
    derived.spare.resize(rev.output.size());
    std::copy(rev.output.begin(), rev.output.end(), derived.spare.data());
    {
        // Before the swap, against what was last published: the array a
        // size reset set aside, else `result` itself (empty either way when
        // the reset found nothing).
        const std::vector<RrVec3f> &published =
            resetFrom.empty() ? derived.result.Read() : resetFrom.Read();
        if (!RrGeoPointBitsEqual(derived.spare, published)) {
            ++derived.resultVersion;
        }
    }
    if(object>=0 && size_t(object)<store.derivedPublish.size())
        store.derivedPublish[size_t(object)].result.clear();
    derived.result.swap(derived.spare);
    derived.haveResult = true;
    if (size_t(object) < store.derivedPublish.size()) {
        store.derivedPublish[size_t(object)].haveBase = true;
        store.derivedPublish[size_t(object)].result = derived.result;
    }
    output.counters.chainsBuilt = 1;
    output.counters.revisionsCreated += derived.createdCount;
    output.counters.schedulesBuilt += derived.scheduleCount;
    return true;
}

bool
RrGeoRunChainStatusStep(RrProgram *program, RrGeometryScratch *scratch,
                        size_t step, std::string *error)
{
    RrStore &store = program->store;
    const RigExecWireDomainGeometry &geo = *program->geometry;
    const int object = (*program->steps)[step].object;
    RrStepOutput &output = store.stepOutputs[step];
    if (object < 0 || size_t(object) >= scratch->chains.size() ||
        size_t(object) >= geo.chains.size()) {
        if (error) {
            *error = RrGeoStepLabel(program, step) + " names no chain";
        }
        return false;
    }
    RrGeometryScratch::Chain &chain = scratch->chains[size_t(object)];
    const bool haveBase = size_t(object) < store.chainHaveBase.size() &&
                          store.chainHaveBase[size_t(object)] != 0;
    if (size_t(object) < store.chainPublish.size()) {
        store.chainPublish[size_t(object)].haveBase = false;
    }
    if (!haveBase) {
        // Publish unavailable at the ChainPoints owner frontier; retain the
        // cached numerical vector without exposing it as this frame's Final.
        chain.haveResult = false;
        return true;
    }
    for (const RrGeometryScratch::Revision &rev : chain.revisions) {
        if (rev.resultStatus == "moverFailed") {
            const size_t r = size_t(&rev - chain.revisions.data());
            output.diagnostics.push_back(
                "MoverFailed " +
                program->TextOrEmpty(
                    geo.chains[size_t(object)].revisions[r].moverPath) +
                ": execution rejected its inputs; revision passed through");
        }
    }
    const RrVec3f *points =
        chain.lastBase.empty() ? nullptr : chain.lastBase.data();
    size_t count = chain.lastBase.size();
    if (!chain.revisions.empty()) {
        RrGeoPointsAfter(chain, chain.revisions.size() - 1, &points,
                         &count);
    }
    chain.spare.resize(count);
    if (count > 0 && points) {
        std::copy(points, points + count, chain.spare.data());
    }
    // ChainPoints' content version moves exactly when the bytes do.
    if (!RrGeoPointBitsEqual(chain.spare, chain.result.Read())) {
        ++chain.resultVersion;
    }
    if(object>=0 && size_t(object)<store.chainPublish.size())
        store.chainPublish[size_t(object)].result.clear();
    chain.result.swap(chain.spare);
    chain.haveResult = true;
    if (size_t(object) < store.chainPublish.size()) {
        store.chainPublish[size_t(object)].haveBase = true;
        store.chainPublish[size_t(object)].result = chain.result;
    }
    output.counters.chainsBuilt = 1;
    output.counters.revisionsCreated += chain.createdCount;
    output.counters.schedulesBuilt += chain.scheduleCount;
    return true;
}

bool
RrGeoRunInfluenceFoldStep(RrProgram *program, RrGeometryScratch *scratch,
                          size_t chainIndex, size_t revisionIndex,
                          size_t step, std::string *error)
{
    RrGeometryScratch::Chain &chain = scratch->chains[chainIndex];
    RrGeometryScratch::Revision &rev = chain.revisions[revisionIndex];
    const RigExecWireRevision &wire = (*program->geometry)
                                          .chains[chainIndex]
                                          .revisions[revisionIndex];
    const bool skin = wire.op == uint8_t(RrGeoOpSkin);
    bool changed = false;
    if (!RrGeoFoldInfluences(program, scratch, wire, &rev, &changed,
                             error)) {
        if (error) {
            *error = RrGeoStepLabel(program, step) + ": " + *error;
        }
        return false;
    }
    rev.influencesChanged = changed;
    rev.influencesValid =
        !skin || RrGeoSkinTransformsAreUsable(rev.influences.data(),
                                             rev.influences.size());
    if (skin && rev.influencesValid) {
        RrGeoFoldTransformForms(&rev, program->geoSettings.useSimd);
    }
    return true;
}

bool
RrGeoRunRevisionStaticStep(RrProgram *program, RrGeometryScratch *scratch,
                           size_t chainIndex, size_t revisionIndex,
                           size_t step, std::string *error)
{
    RrStore &store = program->store;
    const RigExecWireDomainGeometry &geo = *program->geometry;
    RrGeometryScratch::Chain &chain = scratch->chains[chainIndex];
    RrGeometryScratch::Revision &rev = chain.revisions[revisionIndex];
    const RigExecWireRevision &wire =
        geo.chains[chainIndex].revisions[revisionIndex];
    const int id = (*program->steps)[step].object;
    RrStepOutput &output = store.stepOutputs[step];
    const bool skin = wire.op == uint8_t(RrGeoOpSkin);
    if(wire.op==uint8_t(RrGeoOpSkin)) {
        rev.topology=rev.layoutHandle; rev.topologyResolved=true;
        RrGeoAdoptPartition(&rev,size_t(id)<scratch->epochPartitionTopologies.size()
            ?scratch->epochPartitionTopologies[size_t(id)].get():nullptr);
    }

    if (wire.weightCurrentPhase && wire.weightObject >= 0) {
        // The oracle measures the volume against the points entering this
        // revision and patches the exec packet with the field, leaving its
        // rangePolicy as the exec packet had it (bakedGeometry.cpp,
        // RevisionStatic). A failed resolve is the kernel's atomic
        // pass-through, said with the oracle's words.
        const size_t object = size_t(wire.weightObject);
        if (object >= store.weightPackets.size()) {
            if (error) {
                *error = RrGeoStepLabel(program, step) +
                         " measures a weight object the program does not "
                         "hold";
            }
            return false;
        }
        rev.currentPhasePacket =
            RrGeoStorePacket(program, store.weightPackets[object],&rev.nativePacketCache);
        const auto &field = store.weightFieldResults[size_t(wire.weightField)];
        if (field.ok) {
            rev.currentPhasePacket.representation = "dense";
            const bool same = rev.currentPhaseArrays &&
                rev.currentPhaseArrays->values.size() == field.values.size() &&
                (field.values.empty() || std::memcmp(
                    rev.currentPhaseArrays->values.data(), field.values.data(),
                    field.values.size() * sizeof(float)) == 0);
            if (!same) {
                auto arrays = std::make_shared<RrGeoWeightPacket::Arrays>();
                arrays->values = field.values;
                rev.currentPhaseArrays = std::move(arrays);
            }
            rev.currentPhasePacket.arrays = rev.currentPhaseArrays;
            rev.currentPhasePacket.values.clear();
            rev.currentPhasePacket.indices.clear();
            rev.currentPhasePacket.defaultWeight = 0.0f;
            rev.currentPhasePacket.valid = true;
        } else {
            rev.currentPhasePacket = RrGeoWeightPacket();
            output.diagnostics.push_back("current-phase weight failed: " +
                                         field.error);
        }
    }
    RrGeoAssembleInputs in;
    in.program = program;
    in.scratch = scratch;
    in.wire = &wire;
    in.rev = &rev;
    in.basePoints =
        chain.lastBase.empty() ? nullptr : chain.lastBase.data();
    in.basePointCount = chain.lastBase.size();
    in.chain = chainIndex;
    in.revision = revisionIndex;
    in.derived = false;
    in.diagnostics = &output.diagnostics;
    rev.parameters = RrGeoAssembleRevision(in);
    rev.status = RrGeoStatusForParameters(
        rev.parameters, program->TextOrEmpty(wire.moverPath));
    output.counters.revisionsBuilt = 1;
    rev.defaultWeight = RrGeoAssembleDefaultWeight(in);
    rev.staticDirty = true; // Selected graph body owns numerical evaluation.
    rev.lastDefaultWeight = rev.defaultWeight;
    if (id >= 0 && size_t(id) < store.revisionStaticDirty.size()) {
        store.revisionStaticDirty[size_t(id)] = rev.staticDirty ? 1 : 0;
    }
    rev.weightFieldPublished = false;
    if (wire.weightObject >= 0 &&
        size_t(wire.weightObject) < store.weightPackets.size()) {
        const RrGeoWeightPacket packet =
            wire.weightCurrentPhase
                ? rev.currentPhasePacket
                : RrGeoStorePacket(
                      program, store.weightPackets[size_t(wire.weightObject)],
                      &rev.nativePacketCache);
        if (packet.valid) {
            const size_t logicalCount = wire.weightOperationDomain
                ? size_t(1)
                : chain.lastBase.size();
            if(id>=0 && size_t(id)<store.revisionPublish.size())
                store.revisionPublish[size_t(id)].weightField.clear();
            auto &fieldValues=rev.weightField.Write();
            if (!packet.ResolveAll(logicalCount, &fieldValues)) {
                fieldValues.assign(logicalCount, 0.0f);
                for (size_t i = 0; i < logicalCount; ++i) {
                    const float w = packet.Resolve(i, logicalCount);
                    fieldValues[i] = w < 0.0f ? 0.0f : w;
                }
            }
            rev.weightFieldPublished = true;
        }
    }
    if (id >= 0 && size_t(id) < store.revisionPublish.size()) {
        store.revisionPublish[size_t(id)].weightFieldPublished =
            rev.weightFieldPublished;
        store.revisionPublish[size_t(id)].weightField = rev.weightField;
    }
    const size_t count = chain.lastBase.size();
    if (rev.stagingOutput.size() != count) {
        rev.stagingOutput.resize(count);
        rev.staticDirty = true;
        if (id >= 0 && size_t(id) < store.revisionStaticDirty.size()) {
            store.revisionStaticDirty[size_t(id)] = 1;
        }
    }
    rev.precedingCount = count;
    rev.layoutUsable = false;
    rev.envelopeOk = true;
    rev.fullStrength = true;
    rev.partitionStale = false;
    if (!skin) {
        // A wire's dense walk blends a separate envelope after its kernel;
        // resolved here at the full count, as a skin's is, and its chunk
        // blends with this array (bakedGeometry.cpp, RevisionStatic).
        if (wire.op == uint8_t(RrGeoOpWire) &&
            RrGeoRevisionTakesSeparateBlend(int(wire.op),
                                            rev.parameters.weights)) {
            rev.fullStrength =
                RrGeoEnvelopeIsFullStrength(rev.parameters.weights);
            if (!rev.fullStrength) {
                rev.envelopeOk =
                    rev.parameters.weights.ResolveAll(count, &rev.envelope);
            }
        }
        // From the validation the chunk's kernel runs first, over the count
        // it is applied to.
        rev.acceptance = RrGeoRevisionKernelAcceptance(int(wire.op),
                                                       rev.parameters, count);
        return true;
    }
    rev.layoutUsable = RrGeoSkinLayoutIsUsable(rev.parameters, count);
    rev.fullStrength = RrGeoEnvelopeIsFullStrength(rev.parameters.weights);
    if (!rev.fullStrength) {
        rev.envelopeOk =
            rev.parameters.weights.ResolveAll(count, &rev.envelope);
    }
    rev.acceptance = RrGeoSkinAcceptance(rev);
    if (rev.chunked) {
        const RrGeoSkinTopology *const topology =
            rev.parameters.skinTopology.get();
        const size_t indexCount = topology ? topology->indices.size()
                                           : rev.parameters.skinIndices.size();
        const int elementSize = topology ? topology->elementSize
                                         : rev.parameters.skinElementSize;
        rev.partitionStale =
            !rev.parameters.skinTopology ||
            rev.parameters.skinTopology != rev.partitionTopology ||
            indexCount != rev.partitionIndexCount ||
            elementSize != rev.partitionElementSize ||
            count != rev.partitionPointCount;
    }
    return true;
}

bool
RrGeoRunRevisionChunkStep(RrProgram *program, RrGeometryScratch *scratch,
                          size_t chainIndex, size_t revisionIndex,
                          size_t step, std::string *error)
{
    RrStore &store = program->store;
    const RigExecWireStep &wireStep = (*program->steps)[step];
    RrGeometryScratch::Chain &chain = scratch->chains[chainIndex];
    RrGeometryScratch::Revision &rev = chain.revisions[revisionIndex];
    const RigExecWireRevision &wire = (*program->geometry)
                                          .chains[chainIndex]
                                          .revisions[revisionIndex];
    const bool skin = wire.op == uint8_t(RrGeoOpSkin);
    const bool useSimd = program->geoSettings.useSimd;
    if (wireStep.part < 0 ||
        size_t(wireStep.part) >= rev.chunks.size()) {
        if (error) {
            *error = RrGeoStepLabel(program, step) + " names no chunk";
        }
        return false;
    }
    RrGeometryScratch::Chunk &chunk = rev.chunks[size_t(wireStep.part)];
    const RrVec3f *points = nullptr;
    size_t count = 0;
    RrGeoPointsBefore(chain, revisionIndex, &points, &count);
    const bool sized = count == rev.precedingCount &&
                       rev.stagingOutput.size() == count;

    if (!rev.chunked) {
        chunk.ok = false;
        if (!rev.status.AllowsApply()) {
            return true;
        }
        if (!skin) {
            if (!points && count > 0) {
                if (error) {
                    *error = RrGeoStepLabel(program, step) +
                             " reads points no buffer holds";
                }
                return false;
            }
            // Out of place into the revision's unpublished buffer, which the
            // fuse swaps in, as the program's chunk does. A wire blends with
            // the envelope RevisionStatic resolved.
            rev.stagingOutput.resize(count);
            rev.stagingFresh = true;
            chunk.ok = RrGeoRunRevisionKernel(
                int(wire.op), rev.parameters, &rev.stagingOutput,
                &scratch->wireBasis, useSimd, &rev.surfaceCache, &rev.wireRestCache, &rev.lastWireBasis,
                !rev.fullStrength && rev.envelopeOk ? &rev.envelope : nullptr,
                count > 0 ? points : nullptr, count);
            return true;
        }
        if (!RrGeoSkinPacketIsUsable(rev) || !rev.influencesValid || !sized) {
            return true;
        }
        rev.stagingFresh = true;
        chunk.ok = RrGeoSkinRange(&rev, points,
                                  RrGeoWholeTransformsView(&rev), 0, count,
                                  true, useSimd, &rev.stagingOutput);
        return true;
    }

    chunk.keyChanged = false;
    if (!rev.status.AllowsApply() || !RrGeoSkinPacketIsUsable(rev) ||
        rev.partitionStale || !sized) {
        chunk.ok = false;
        return true;
    }
    if (!RrGeoGatherChunkTransforms(program, wire, rev, &chunk, error)) {
        if (error) {
            *error = RrGeoStepLabel(program, step) + ": " + *error;
        }
        return false;
    }
    if (chunk.begin < 0 || chunk.end < chunk.begin ||
        size_t(chunk.end) > count) {
        if (error) {
            *error =
                RrGeoStepLabel(program, step) + " names no vertex range";
        }
        return false;
    }
    chunk.ok = RrGeoSkinRange(&rev, points,
                              RrGeoChunkTransformsView(chunk, useSimd),
                              size_t(chunk.begin), size_t(chunk.end), false,
                              useSimd, &rev.stagingOutput);
    return true;
}

bool
RrGeoRunRevisionFuseStep(RrProgram *program, RrGeometryScratch *scratch,
                         size_t chainIndex, size_t revisionIndex,
                         size_t step, std::string *error)
{
    RrStore &store = program->store;
    RrGeometryScratch::Chain &chain = scratch->chains[chainIndex];
    RrGeometryScratch::Revision &rev = chain.revisions[revisionIndex];
    const RigExecWireRevision &wire = (*program->geometry)
                                          .chains[chainIndex]
                                          .revisions[revisionIndex];
    const int id = (*program->steps)[step].object;
    RrStepOutput &output = store.stepOutputs[step];
    const bool skin = wire.op == uint8_t(RrGeoOpSkin);
    const bool packetValid =
        rev.parameters.valid && (!skin || rev.influencesValid);
    if (rev.parameters.enabled && !packetValid && wire.weightObject < 0) {
        const float scalar = rev.defaultWeight;
        if (!std::isfinite(scalar) || scalar < 0.0f || scalar > 1.0f) {
            output.diagnostics.push_back(
                "MoverFailed " + program->TextOrEmpty(wire.moverPath) +
                ": inputs:defaultWeight must be finite and in [0, 1]; "
                "revision passed through");
        }
    } else if (rev.parameters.enabled && !packetValid &&
               wire.weightObject >= 0) {
        bool boundValid = false;
        if (wire.weightCurrentPhase) {
            boundValid = rev.currentPhasePacket.valid;
        } else if (size_t(wire.weightObject) <
                   store.weightPackets.size()) {
            boundValid =
                store.weightPackets[size_t(wire.weightObject)].valid;
        }
        if (!boundValid) {
            output.diagnostics.push_back(
                "MoverFailed " + program->TextOrEmpty(wire.moverPath) +
                ": rigExec:weightObject produced an invalid common "
                "envelope; revision passed through");
        }
    }
    // The common graph selected this semantic completion body.
    rev.executed = true;
    output.counters.revisionsExecuted = rev.executed ? 1 : 0;
    if (rev.executed) {
        // The baseline the content version is decided against, as the
        // program's fuse: its own output when it applied last, the copy of
        // its entering points when it passed through, none before its first.
        const bool hadOwn =
            rev.ran && rev.currentSource == int(revisionIndex);
        bool moved = !rev.ran;
        rev.resultStatus = rev.status.state;
        bool applied = packetValid && rev.status.AllowsApply();
        const bool whole = applied && skin && rev.partitionStale;
        // The whole-revision skin's points; a stale partition is rare.
        std::vector<RrVec3f> fused;
        if (whole) {
            applied = RrGeoFuseWholeRevision(chain, &rev, revisionIndex,
                                             program->geoSettings.useSimd,
                                             &fused);
        } else if (applied && rev.acceptance != RrGeoAcceptance::Deferred) {
            // RevisionStatic's decision, from the validation each chunk's
            // kernel runs first, so every chunk's `ok` is this answer.
            applied = rev.acceptance == RrGeoAcceptance::Applies;
        } else if (applied) {
            for (const RrGeometryScratch::Chunk &chunk : rev.chunks) {
                if (!chunk.ok) {
                    applied = false;
                    break;
                }
            }
        }
        if (applied) {
            if (rev.chunked || whole) {
                // Sticky chunk ranges stay in staging: copy moved blocks.
                const std::vector<RrVec3f> &result =
                    whole ? fused : rev.stagingOutput;
                if (!hadOwn) {
                    moved = moved ||
                            !RrGeoPointBitsEqual(result, rev.passedPoints);
                }
                const bool copied = RrGeoCopyMovedPoints(result, &rev.output);
                moved = moved || (hadOwn && copied);
            } else if (rev.stagingFresh) {
                moved = moved ||
                        !RrGeoPointBitsEqual(rev.stagingOutput,
                                             hadOwn ? rev.output : rev.passedPoints);
                // Ownership flips instead of a copy; the replaced buffer is
                // the next chunk's, at the size the chunk left (RevisionOut
                // keys the staging size).
                rev.output.swap(rev.stagingOutput);
                rev.stagingOutput.resize(rev.output.size());
                rev.stagingFresh = false;
            } else if (!hadOwn) {
                moved = moved || !RrGeoPointBitsEqual(rev.output, rev.passedPoints);
            }
            rev.currentSource = int(revisionIndex);
        } else {
            // A pass-through publishes the entering points; `output` keeps
            // the last applied points a later apply may republish.
            const std::vector<RrVec3f> *entering =
                RrGeoPointsVersion(chain, revisionIndex);
            const std::vector<RrVec3f> &previous =
                hadOwn ? rev.output : rev.passedPoints;
            const bool same = entering ? RrGeoPointBitsEqual(*entering, previous)
                                       : previous.empty();
            moved = moved || !same;
            if (hadOwn || !same) {
                if (entering) rev.passedPoints = *entering;
                else rev.passedPoints.clear();
            }
            rev.currentSource =
                revisionIndex == 0
                    ? -1
                    : chain.revisions[revisionIndex - 1].currentSource;
            if (rev.status.AllowsApply()) {
                rev.resultStatus = "moverFailed";
            }
        }
        if (moved) {
            ++rev.doneVersion;
        }
        // No deep packet snapshot: graph input keys own scheduling.
        rev.lastStatus = rev.status;
        rev.ran = true;
    }
    if (id >= 0 && size_t(id) < store.revisionRan.size()) {
        store.revisionRan[size_t(id)] = rev.ran ? 1 : 0;
    }
    if (id >= 0 && size_t(id) < store.revisionPublish.size()) {
        store.revisionPublish[size_t(id)].resultStatus = rev.resultStatus;
    }
    (void)error;
    return true;
}

}  // namespace

bool RrRevisionWeightFieldInput(const RrProgram *program,size_t fieldIndex,
                                const RrVec3f **points,size_t *count)
{
    const auto &field=program->geometry->weightFields[fieldIndex];
    const auto &entry=program->geometry->revisionIndex[size_t(field.consumer)];
    const auto *scratch=static_cast<const RrGeometryScratch *>(program->geo.get());
    const auto &chain=scratch->chains[size_t(entry.first)];
    *points=nullptr; *count=0;
    if(!chain.haveBase) return false;
    RrGeoPointsBefore(chain,size_t(entry.second),points,count);
    return true;
}

bool
RrRunRevisionWeightField(RrProgram *program, size_t fieldIndex)
{
    const auto &field = program->geometry->weightFields[fieldIndex];
    const auto &entry = program->geometry->revisionIndex[size_t(field.consumer)];
    const auto &chain = RrGeoScratch(program)->chains[size_t(entry.first)];
    if (!chain.haveBase) {
        auto &result = program->store.weightFieldResults[fieldIndex];
        result.count = 0;
        result.ok = false;
        result.values.clear();
        result.error.clear();
        return false;
    }
    const RrVec3f *points = nullptr;
    size_t count = 0;
    RrGeoPointsBefore(chain, size_t(entry.second), &points, &count);
    std::vector<RrVec3f> current;
    if (count) current.assign(points, points + count);
    return RrResolveDeclaredWeightField(program, fieldIndex, count, &current);
}

bool
RrRunGeometryStep(RrProgram *program, size_t step, std::string *error)
{
    if (!program || !program->steps || !program->geometry ||
        !program->geo || step >= program->steps->size() ||
        step >= program->store.stepOutputs.size()) {
        if (error) {
            *error = "geometry step names no step";
        }
        return false;
    }
    RrGeometryScratch *scratch = RrGeoScratch(program);
    const RigExecWireStep &wireStep = (*program->steps)[step];
    const RigExecWireStepKind kind = wireStep.kind;
    if (kind != RigExecWireStepKind::InfluenceFold &&
        kind != RigExecWireStepKind::RevisionStatic &&
        kind != RigExecWireStepKind::RevisionChunk &&
        kind != RigExecWireStepKind::RevisionFuse &&
        kind != RigExecWireStepKind::ChainStatus &&
        kind != RigExecWireStepKind::Derived &&
        kind != RigExecWireStepKind::ChainInputs) {
        if (error) {
            *error = RrGeoStepLabel(program, step) +
                     " is not a geometry step";
        }
        return false;
    }
    if (kind == RigExecWireStepKind::ChainInputs)
        return RrRunChainInputs(program,size_t(wireStep.object),error);
    if (kind == RigExecWireStepKind::Derived) {
        return RrGeoRunDerivedStep(program, scratch, step, error);
    }
    if (kind == RigExecWireStepKind::ChainStatus) {
        return RrGeoRunChainStatusStep(program, scratch, step, error);
    }
    const int object = wireStep.object;
    const RigExecWireDomainGeometry &geo = *program->geometry;
    if (object < 0 || size_t(object) >= geo.revisionIndex.size()) {
        if (error) {
            *error =
                RrGeoStepLabel(program, step) + " names no revision";
        }
        return false;
    }
    const size_t chainIndex = size_t(geo.revisionIndex[size_t(object)].first);
    const size_t revisionIndex =
        size_t(geo.revisionIndex[size_t(object)].second);
    if (chainIndex >= scratch->chains.size() ||
        chainIndex >= geo.chains.size() ||
        revisionIndex >= scratch->chains[chainIndex].revisions.size() ||
        revisionIndex >= geo.chains[chainIndex].revisions.size()) {
        if (error) {
            *error =
                RrGeoStepLabel(program, step) + " names no revision";
        }
        return false;
    }
    if (!RrGeoOpName(geo.chains[chainIndex].revisions[revisionIndex].op)) {
        if (error) {
            *error = RrGeoStepLabel(program, step) +
                     " references unknown revision op " +
                     std::to_string(
                         geo.chains[chainIndex].revisions[revisionIndex].op);
        }
        return false;
    }
    const bool haveBase = chainIndex < program->store.chainHaveBase.size() &&
                          program->store.chainHaveBase[chainIndex] != 0;
    if (!haveBase) {
        return true;
    }
    switch (kind) {
    case RigExecWireStepKind::InfluenceFold:
        return RrGeoRunInfluenceFoldStep(program, scratch, chainIndex,
                                         revisionIndex, step, error);
    case RigExecWireStepKind::RevisionStatic:
        return RrGeoRunRevisionStaticStep(program, scratch, chainIndex,
                                          revisionIndex, step, error);
    case RigExecWireStepKind::RevisionChunk:
        return RrGeoRunRevisionChunkStep(program, scratch, chainIndex,
                                         revisionIndex, step, error);
    case RigExecWireStepKind::RevisionFuse:
        return RrGeoRunRevisionFuseStep(program, scratch, chainIndex,
                                        revisionIndex, step, error);
    default:
        break;
    }
    if (error) {
        *error =
            RrGeoStepLabel(program, step) + " is not a geometry step";
    }
    return false;
}

void
RrSkipGeometryStep(RrProgram *program, size_t step)
{
    if (!program || !program->steps || !program->geometry ||
        !program->geo || step >= program->steps->size()) {
        return;
    }
    RrGeometryScratch *scratch = RrGeoScratch(program);
    const RigExecWireStep &wireStep = (*program->steps)[step];
    const RigExecWireDomainGeometry &geo = *program->geometry;
    if (wireStep.kind == RigExecWireStepKind::WeightField) {
        // Field IDs do not index revision bookkeeping.
        return;
    }
    if (wireStep.kind == RigExecWireStepKind::ChainInputs) {
        if (wireStep.object >= 0 &&
            size_t(wireStep.object) < program->store.chainBaseDirty.size()) {
            program->store.chainBaseDirty[size_t(wireStep.object)] = 0;
        }
        return;
    }
    if (wireStep.kind == RigExecWireStepKind::Derived) {
        if (wireStep.object < 0 ||
            size_t(wireStep.object) >= scratch->derived.size()) {
            return;
        }
        scratch->derived[size_t(wireStep.object)]
            .revision.influencesChanged = false;
        return;
    }
    if (wireStep.kind == RigExecWireStepKind::ChainStatus) {
        return;
    }
    if (wireStep.object < 0 ||
        size_t(wireStep.object) >= geo.revisionIndex.size()) {
        return;
    }
    const size_t chainIndex =
        size_t(geo.revisionIndex[size_t(wireStep.object)].first);
    const size_t revisionIndex =
        size_t(geo.revisionIndex[size_t(wireStep.object)].second);
    if (chainIndex >= scratch->chains.size() ||
        revisionIndex >= scratch->chains[chainIndex].revisions.size()) {
        return;
    }
    RrGeometryScratch::Revision &rev =
        scratch->chains[chainIndex].revisions[revisionIndex];
    switch (wireStep.kind) {
    case RigExecWireStepKind::InfluenceFold:
        rev.influencesChanged = false;
        return;
    case RigExecWireStepKind::RevisionStatic:
        rev.staticDirty = false;
        if (size_t(wireStep.object) <
            program->store.revisionStaticDirty.size()) {
            program->store.revisionStaticDirty[size_t(wireStep.object)] = 0;
        }
        return;
    case RigExecWireStepKind::RevisionChunk:
        if (wireStep.part < 0 ||
            size_t(wireStep.part) >= rev.chunks.size()) {
            return;
        }
        rev.chunks[size_t(wireStep.part)].keyChanged = false;
        return;
    case RigExecWireStepKind::RevisionFuse:
        rev.executed = false;
        return;
    default:
        return;
    }
}


namespace {
template <class T> void RrOpBytes(std::string *key, const T &v)
{ key->append(reinterpret_cast<const char *>(&v), sizeof(v)); }
void RrOpBytes(std::string *key, const std::string &v)
{ RrOpBytes(key, v.size()); key->append(v); }
void RrOpBytes(std::string *key, const RrPointFrame &v)
{ RrOpBytes(key,v.points); RrOpBytes(key,v.flags); }
template <class T> void RrOpBytes(std::string *key, const std::vector<T> &v)
{
    RrOpBytes(key, v.size());
    if constexpr (RigExecOpKeyBulkElement<T>::value &&
                  RigExecOpKeyContiguous<std::vector<T>>::value)
        RigExecOpKeyAppendRun(key, v.data(), v.size());
    else for (const auto &x : v) RrOpBytes(key, x);
}
void RrOpParametersKey(std::string *key, const RrGeoMoverParameters &p)
{
    RrOpBytes(key, p.kind);
    RrOpBytes(key, p.enabled);
    RrOpBytes(key, p.valid);
    RrOpBytes(key, p.transform);
    RrOpBytes(key, p.radialWeight);
    RrOpBytes(key, p.blendDeltas);
    RrOpBytes(key, p.blendSurfaceFrame);
    RrOpBytes(key, p.referenceVolume);
    RrOpBytes(key, p.strength);
    RrOpBytes(key, p.mushIterations);
    RrOpBytes(key, p.mushStep);
    RrOpBytes(key, p.mushPinBorders);
    RrOpBytes(key, p.mushDistanceWeight);
    RrOpBytes(key, p.mushDisplacement);
    RrOpBytes(key, p.topologyCounts);
    RrOpBytes(key, p.topologyIndices);
    RrOpBytes(key, p.auxPoints);
    RrOpBytes(key, p.auxPointsB);
    RrOpBytes(key, p.restPoints);
    RrOpBytes(key, p.divisions);
    RrOpBytes(key, p.bindCoords);
    RrOpBytes(key, p.wireBindCoords);
    RrOpBytes(key, p.curveOrder);
    RrOpBytes(key, p.curveKnots);
    RrOpBytes(key, p.dropoffDistance);
    RrOpBytes(key, p.widths);
    RrOpBytes(key, p.skinTransforms);
    RrOpBytes(key, p.skinIndices);
    RrOpBytes(key, p.skinWeights);
    RrOpBytes(key, p.skinElementSize);
    RrOpBytes(key, p.skinningMethod);
    RrOpBytes(key, p.externalState);
    RrOpBytes(key,p.externalInputs.size());
    for(const auto &input:p.externalInputs) {
        RrOpBytes(key,input.type); RrOpBytes(key,input.have); RrOpBytes(key,input.count);
        RrOpBytes(key,input.bytes); RrOpBytes(key,input.token); RrOpBytes(key,input.tokens);
    }
    RrOpBytes(key, p.externalPhasedPaths);
    RrOpBytes(key, p.externalPhasedHave);
    RrOpBytes(key, p.externalPhased);
    RrOpBytes(key, p.weights.representation); RrOpBytes(key, p.weights.rangePolicy);
    RrOpBytes(key, p.weights.Values()); RrOpBytes(key, p.weights.Indices());
    RrOpBytes(key, p.weights.defaultWeight); RrOpBytes(key, p.weights.valid);
    RrOpBytes(key, p.frames.frames); RrOpBytes(key, p.frames.rests);
    RrOpBytes(key,p.wrinkleSettings.iterations);
    RrOpBytes(key,p.wrinkleSettings.topology);
    RrOpBytes(key,p.wrinkleSettings.neighborDistance);
    RrOpBytes(key,p.wrinkleSettings.restLengthScale);
    RrOpBytes(key,p.wrinkleSettings.stretchStiffness);
    RrOpBytes(key,p.wrinkleSettings.compressionStiffness);
    RrOpBytes(key,p.wrinkleSettings.bendStiffness);
    RrOpBytes(key,p.wrinkleSettings.maxDisplacement);
    RrOpBytes(key,p.wrinkleSettings.pinBorders);
    RrOpBytes(key,p.wrinkleSettings.tangentPlaneCollisions);
    RrOpBytes(key,p.wrinkleSettings.tangentPlaneInset);
    RrOpBytes(key,p.wrinkleSettings.wrinkleScale);
    RrOpBytes(key,p.wrinkleSettings.smoothingIterations);
    RrOpBytes(key,p.wrinkleSettings.pinPoints);

    const bool topology = bool(p.skinTopology); RrOpBytes(key, topology);
    if (topology) { RrOpBytes(key,p.skinTopology->indices); RrOpBytes(key,p.skinTopology->weights);
        RrOpBytes(key,p.skinTopology->elementSize); RrOpBytes(key,p.skinTopology->pointCount);
        RrOpBytes(key,p.skinTopology->influenceCount); RrOpBytes(key,p.skinTopology->validated); }
    const bool frame = bool(p.externalFrame); RrOpBytes(key, frame);
    if (frame) RrOpBytes(key, *p.externalFrame);
}
} // namespace

bool RrGeometryChainContentKey(const RrProgram *program, RigExecWireSlotDomain domain,
    uint32_t slot, std::string *key)
{
    const auto *scratch = static_cast<const RrGeometryScratch *>(program->geo.get());
    key->clear();
    if (!scratch) return false;
    using D = RigExecWireSlotDomain;
    if (domain == D::ChainInput) {
        const auto &chain=scratch->chains[slot];
        RrOpBytes(key,chain.sampleHaveBase); RrOpBytes(key,chain.sampledBase); return true;
    }
    if (domain == D::ChainBase || domain == D::ChainPoints) {
        const auto &c = scratch->chains[slot];
        RrOpBytes(key, domain == D::ChainBase ? c.haveBase : c.haveResult);
        RrOpBytes(key, domain == D::ChainBase ? c.lastBase : c.result.Read()); return true;
    }
    if (domain == D::DerivedOut) {
        const auto &d = scratch->derived[slot]; RrOpBytes(key,d.haveResult); RrOpBytes(key,d.result.Read());
        const auto &published = program->store.derivedPublish[slot];
        RrOpBytes(key,published.haveMatrix); RrOpBytes(key,published.matrix); return true;
    }
    if ((domain == D::RevisionDone || domain == D::ChainDirty) &&
        slot < program->geometry->revisionIndex.size()) {
        const auto &index=program->geometry->revisionIndex[slot];
        const auto &chain=scratch->chains[size_t(index.first)];
        const auto &rev=chain.revisions[size_t(index.second)];
        RrOpBytes(key,rev.currentSource); RrOpBytes(key,rev.resultStatus);
        RrOpBytes(key,rev.status.state); RrOpBytes(key,rev.status.firstBadAddress);
        if(rev.currentSource<0) RrOpBytes(key,chain.lastBase);
        else if(size_t(rev.currentSource)<chain.revisions.size())
            RrOpBytes(key,chain.revisions[size_t(rev.currentSource)].output);
        return true;
    }
    return false;
}

void RrGeometryPublishChainInputs(RrProgram *program)
{
    auto *scratch = static_cast<RrGeometryScratch *>(program->geo.get());
    if (!scratch) return;
    // As the native PublishLeaves: the version moves exactly when the
    // sampled bytes differ from the ones last published. A base no prologue
    // wrote since is the one last published.
    for (auto &chain : scratch->chains) {
        if (!chain.sampledMoved) continue;
        chain.sampledMoved = false;
        if (!RrGeoPointBitsEqual(chain.sampledBase, chain.publishedInput)) {
            ++chain.inputVersion; chain.publishedInput = chain.sampledBase;
        }
    }
}

void RrGeometryOpValueKey(const RrProgram *program, RigExecWireSlotDomain domain,
    uint32_t slot, std::string *key)
{
    const auto *scratch = static_cast<const RrGeometryScratch *>(program->geo.get());
    if (!scratch) return;
    // The six point-carrying domains key their points by a content version
    // its writer bumps exactly when the bytes move; RrGeometryChainContentKey
    // is the same key over the bytes.
    if(domain==RigExecWireSlotDomain::ChainInput) {
        const auto &chain=scratch->chains[slot];
        RrOpBytes(key,chain.sampleHaveBase); RrOpBytes(key,chain.inputVersion); return;
    }
    if(domain==RigExecWireSlotDomain::DerivedBase) {
        const auto &derived=scratch->derived[slot];
        RrOpBytes(key,derived.sampleHaveBase); RrOpBytes(key,derived.sampledBase); return;
    }
    if (domain == RigExecWireSlotDomain::ChainBase || domain == RigExecWireSlotDomain::ChainPoints) {
        const auto &c = scratch->chains[slot];
        RrOpBytes(key, domain == RigExecWireSlotDomain::ChainBase ? c.haveBase : c.haveResult);
        RrOpBytes(key, domain == RigExecWireSlotDomain::ChainBase ? c.baseVersion : c.resultVersion); return;
    }
    if (domain == RigExecWireSlotDomain::DerivedOut) {
        const auto &d = scratch->derived[slot]; RrOpBytes(key,d.haveResult); RrOpBytes(key,d.resultVersion);
        const auto &published = program->store.derivedPublish[slot];
        RrOpBytes(key,published.haveMatrix); RrOpBytes(key,published.matrix); return;
    }
    if (domain == RigExecWireSlotDomain::RevisionOut) {
        for (const auto &chain : program->geometry->chains) for (const auto &wire : chain.revisions) {
            if (slot < uint32_t(wire.chunkBase) || slot >= uint32_t(wire.chunkBase)+wire.chunks.size()) continue;
            const size_t c = size_t(&chain-program->geometry->chains.data());
            const size_t r = size_t(&wire-chain.revisions.data());
            const auto &rev = scratch->chains[c].revisions[r];
            const auto &chunk = rev.chunks[slot-uint32_t(wire.chunkBase)];
            const size_t begin=std::min(size_t(chunk.begin),rev.stagingOutput.size());
            const size_t end=std::min(size_t(chunk.end),rev.stagingOutput.size());
            RrOpBytes(key,chunk.ok); RrOpBytes(key,rev.stagingOutput.size()); RrOpBytes(key,end-begin);
            if (begin < end) RigExecOpKeyAppendRun(key,rev.stagingOutput.data()+begin,end-begin);
            return;
        }
        return;
    }
    const RrGeometryScratch::Revision *rev = nullptr;
    if (slot < program->geometry->revisionIndex.size()) {
        const auto &id=program->geometry->revisionIndex[slot]; rev=&scratch->chains[size_t(id.first)].revisions[size_t(id.second)];
    } else if (domain == RigExecWireSlotDomain::SkinTopology) {
        rev=&scratch->derived[slot-program->geometry->revisionIndex.size()].revision;
    }
    if (!rev) return;
    switch(domain) {
    case RigExecWireSlotDomain::RevisionPacket:
        RrOpParametersKey(key,rev->parameters); RrOpBytes(key,rev->status.state);
        RrOpBytes(key,rev->status.firstBadAddress);
        RrOpBytes(key,rev->layoutUsable); RrOpBytes(key,rev->envelopeOk); RrOpBytes(key,rev->envelope);
        RrOpBytes(key,rev->fullStrength); RrOpBytes(key,rev->precedingCount); RrOpBytes(key,rev->partitionStale);
        RrOpBytes(key,rev->weightFieldPublished); RrOpBytes(key,rev->weightField.Read());
        RrOpBytes(key,uint8_t(rev->acceptance)); break;
    case RigExecWireSlotDomain::RevisionTransforms:
        RrOpBytes(key,rev->influences); RrOpBytes(key,rev->influencesValid);
        RrOpBytes(key,rev->transform); RrOpBytes(key,rev->haveTransform);
        RrOpBytes(key,rev->carry); RrOpBytes(key,rev->haveCarry); break;
    case RigExecWireSlotDomain::RevisionDone:
    case RigExecWireSlotDomain::ChainDirty:
        RrOpBytes(key,rev->currentSource); RrOpBytes(key,rev->resultStatus);
        RrOpBytes(key,rev->status.state); RrOpBytes(key,rev->status.firstBadAddress);
        RrOpBytes(key,rev->doneVersion);
        break;
    case RigExecWireSlotDomain::SkinTopology:
        RrOpBytes(key,bool(rev->layoutHandle));
        if (rev->layoutHandle) { const auto &t=*rev->layoutHandle;
            RrOpBytes(key,t.indices); RrOpBytes(key,t.weights); RrOpBytes(key,t.elementSize);
            RrOpBytes(key,t.pointCount); RrOpBytes(key,t.influenceCount); RrOpBytes(key,t.validated); }
        break;
    default: break;
    }
}

void RrResetExcludedGeometryValue(RrProgram *program,
    RigExecWireSlotDomain domain, uint32_t slot)
{
    auto *scratch=static_cast<RrGeometryScratch *>(program->geo.get());
    if (!scratch) return;
    // An excluded writer never runs, so clearing is the only way its points
    // move: the content version moves with them.
    if (domain==RigExecWireSlotDomain::ChainBase) {
        auto &chain=scratch->chains[slot];
        if(!chain.lastBase.empty()) ++chain.baseVersion;
        chain.haveBase=false; chain.lastBase.clear(); return;
    }
    if (domain==RigExecWireSlotDomain::ChainPoints) {
        auto &chain=scratch->chains[slot];
        if(!chain.result.empty()) ++chain.resultVersion;
        chain.haveResult=false; chain.result.clear(); chain.resultStatus.clear(); return;
    }
    if (domain==RigExecWireSlotDomain::DerivedOut) {
        auto &derived=scratch->derived[slot];
        if(!derived.result.empty()) ++derived.resultVersion;
        derived.haveResult=false; derived.result.clear();
        derived.revision.output.clear(); derived.revision.ran=false;
        program->store.derivedPublish[slot].haveMatrix=false; return;
    }
    if (domain==RigExecWireSlotDomain::RevisionOut) {
        for(size_t c=0;c<program->geometry->chains.size();++c) {
            const auto &chain=program->geometry->chains[c];
            for(size_t r=0;r<chain.revisions.size();++r) {
                const auto &wire=chain.revisions[r];
                if(slot<uint32_t(wire.chunkBase) ||
                   slot>=uint32_t(wire.chunkBase)+wire.chunks.size()) continue;
                auto &revision=scratch->chains[c].revisions[r];
                revision.stagingOutput.clear();
                revision.stagingFresh=false;
                for(auto &chunk:revision.chunks) chunk.ok=false;
                return;
            }
        }
        return;
    }
    RrGeometryScratch::Revision *revision=nullptr;
    const RrGeometryScratch::Chain *owner=nullptr;
    if(slot<program->geometry->revisionIndex.size()) {
        const auto &index=program->geometry->revisionIndex[slot];
        owner=&scratch->chains[size_t(index.first)];
        revision=&scratch->chains[size_t(index.first)].revisions[size_t(index.second)];
    } else if(domain==RigExecWireSlotDomain::SkinTopology) {
        revision=&scratch->derived[slot-program->geometry->revisionIndex.size()].revision;
    }
    if(!revision) return;
    switch(domain) {
    case RigExecWireSlotDomain::RevisionPacket:
        revision->parameters.valid=false;
        revision->acceptance=RrGeoAcceptance::Refuses; break;
    case RigExecWireSlotDomain::RevisionTransforms:
        revision->influencesValid=false; break;
    case RigExecWireSlotDomain::RevisionDone:
    case RigExecWireSlotDomain::ChainDirty:
        revision->currentSource=-1; revision->resultStatus="operation cycle";
        revision->ran=false; revision->executed=false;
        // An excluded fuse never runs: the version follows the base it
        // passes through, against the copy of the base it last carried.
        if(owner && !RrGeoPointBitsEqual(owner->lastBase,revision->passedPoints)) {
            ++revision->doneVersion; revision->passedPoints=owner->lastBase;
        }
        break;
    case RigExecWireSlotDomain::SkinTopology:
        revision->layoutHandle.reset(); revision->topology.reset();
        revision->topologyResolved=false; revision->layoutUsable=false; break;
    default: break;
    }
}

const std::vector<RrVec3f> *RrReadCrossDomainPoints(const RrProgram *program,
    const fb::RigExecWireCrossDomainRead &read)
{
    const auto *scratch=static_cast<const RrGeometryScratch *>(program->geo.get());
    for (const auto &candidate:read.points) {
        const auto &chain=scratch->chains[size_t(candidate.first)];
        if (read.finalPoints ? !chain.haveResult : !chain.haveBase) continue;
        const auto *points=read.finalPoints?&chain.result.Read():RrGeoPointsVersion(chain,size_t(candidate.second));
        if (points) return points;
    }
    if(read.points.empty() && read.rawSlot>=0 &&
       program->inputState.slotHasValue[size_t(read.rawSlot)])
        return RrInputArray<RrVec3f>(program,uint32_t(read.rawSlot));
    return nullptr;
}

bool RrReadCrossDomainPoint(const RrProgram *program,
    const fb::RigExecWireCrossDomainRead &read, RrVec3f *out)
{
    const auto *points=RrReadCrossDomainPoints(program,read);
    if(!points || read.element<0 || size_t(read.element)>=points->size()) return false;
    *out=(*points)[size_t(read.element)]; return true;
}

const std::vector<RrVec3f> *RrResolveDeclaredPoints(const RrProgram *program,
    const RigExecWirePointsBinding &binding)
{
    const auto *scratch=static_cast<const RrGeometryScratch *>(program->geo.get());
    return scratch?RrGeoResolvePoints(*scratch,binding):nullptr;
}

}  // namespace rigExec
