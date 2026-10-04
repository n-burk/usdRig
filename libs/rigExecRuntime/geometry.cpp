// rigExecRuntime geometry family (M2): RevisionStatic, InfluenceFold,
// RevisionChunk, RevisionFuse, ChainStatus, Derived.
// A zero-USD port of the baked geometry loop (libs/rigExec/bakedGeometry.cpp
// driven by the movers in libs/rigExec/moverGraph.cpp and the kernels in
// libs/rigExecMath). The scalars an assembler reads through their
// connections, blend channel weights and default weights are read over the
// input slots; the remaining stage reads replay from the frame record's
// pathReads; epoch state (skin layouts, blend shapes, partitions)
// replays from the wire geometry tables. Bitwise: same ops in the same
// order, float stays float.

#include "rigExecRuntime/store.h"
#include "rigExecMath/deltaMushKernel.h"
#include "rigExecMath/wrinkleKernel.h"
#include "rigExecMath/surfaceProjectorKernel.h"
#include "poseInternal.h"

#include <algorithm>
#include <array>
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

bool
RrGeoGetenvBool(const char *name, bool fallback)
{
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    const char *value = std::getenv(name);
#ifdef _MSC_VER
#pragma warning(pop)
#endif
    if (!value || !*value) {
        return fallback;
    }
    std::string lower(value);
    for (char &c : lower) {
        if (c >= 'A' && c <= 'Z') {
            c = char(c - 'A' + 'a');
        }
    }
    if (lower == "0" || lower == "false" || lower == "no" ||
        lower == "off") {
        return false;
    }
    if (lower == "1" || lower == "true" || lower == "yes" ||
        lower == "on") {
        return true;
    }
    return fallback;
}

// Packet mirrors: the per-mover parameter packet and its shared layouts,
// compared exactly the way RigExecMoverParameters::operator== compares
// (layouts and bindings by pointer, everything else by value).

struct RrGeoWeightPacket {
    std::string representation;
    std::string rangePolicy;
    std::vector<float> values;
    std::vector<int> indices;
    float defaultWeight = 0.0f;
    bool valid = false;

    bool operator==(const RrGeoWeightPacket &o) const
    {
        return representation == o.representation &&
               rangePolicy == o.rangePolicy && values == o.values &&
               indices == o.indices && defaultWeight == o.defaultWeight &&
               valid == o.valid;
    }
    bool operator!=(const RrGeoWeightPacket &o) const
    {
        return !(*this == o);
    }

    float Resolve(size_t i, size_t count) const
    {
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

    bool ResolveAll(size_t count, std::vector<float> *resolved) const
    {
        if (!resolved || !valid) {
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
            if (!usable(defaultWeight)) {
                return false;
            }
            resolved->assign(count, defaultWeight);
            return true;
        }

        if (representation == "dense") {
            for (size_t i = 0; i < count; ++i) {
                if (!usable(values[i])) {
                    return false;
                }
            }
            resolved->assign(values.begin(), values.begin() + count);
            return true;
        }

        // Sparse: the default everywhere, then the authored entries
        // scattered over it. The default is only checked when some point
        // can actually read it.
        if (indices.size() < count && !usable(defaultWeight)) {
            return false;
        }
        for (const float value : values) {
            if (!usable(value)) {
                return false;
            }
        }
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
               weights == o.weights;
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
           envelope.values.empty() && envelope.indices.empty() &&
           envelope.defaultWeight == 1.0f &&
           (envelope.rangePolicy.empty() ||
            envelope.rangePolicy == "strict" ||
            envelope.rangePolicy == "clamp");
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
            RrGeoToVec3d(in[i]), transform, weights[i]));
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
        if (weights[i] <= 0.0f) {
            blended = q;
        } else if (weights[i] >= 1.0f) {
            blended = moved;
        } else {
            const __m128 w = _mm_set1_ps(weights[i]);
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

void
RrGeoApplyMatrixKernelRange(const RrGeoMoverParameters &p,
                           const float *envelope, size_t begin, size_t end,
                           RrVec3f *pts)
{
    static const bool useSimd = RrGeoGetenvBool("RIGEXEC_ENABLE_SIMD", true);
    if (p.radialWeight) {
        // Factored per WEIGHT rather than per point: a painted falloff is
        // mostly a handful of distinct values. Mirrors
        // RigExecApplyMatrixKernelRange exactly.
        double cachedWeight = -1.0;
        RrMat4d partial(1.0);
        for (size_t i = begin; i < end; ++i) {
            const double w = envelope[i];
            if (w != cachedWeight) {
                partial = RrGeoPartialTransform(p.transform, w);
                cachedWeight = w;
            }
            pts[i] = RrGeoToVec3f(partial.TransformAffine(
                RrGeoToVec3d(pts[i])));
        }
        return;
    }
    if (useSimd) {
        RrGeoApplyWeightedMatrixSimd(
            pts + begin, pts + begin, envelope + begin, end - begin,
            p.transform);
    } else {
        // Element i is written only after it is read, so in-place is safe.
        for (size_t i = begin; i < end; ++i) {
            pts[i] = RrGeoToVec3f(RrGeoApplyWeightedMatrix(
                RrGeoToVec3d(pts[i]), p.transform, envelope[i]));
        }
    }
}

bool
RrGeoApplyMatrixKernel(const RrGeoMoverParameters &p, std::vector<RrVec3f> *pts)
{
    const size_t count = pts->size();
    const RrGeoWeightPacket &w = p.weights;
    if (w.valid && w.representation == "sparse" && w.defaultWeight == 0.0f &&
        w.indices.size() == w.values.size() &&
        (w.rangePolicy.empty() || w.rangePolicy == "strict" ||
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
        if (p.transform == RrGeoIdentity()) {
            return true;  // at rest every weighted point maps to itself
        }
        RrVec3f *data = pts->data();
        if (p.radialWeight) {
            // The sparse walk takes the same arc the dense kernel does; a
            // painted cluster falloff is almost always sparse, so this is
            // the branch every radial cluster on a real rig reaches.
            double cachedWeight = -1.0;
            RrMat4d partial(1.0);
            for (size_t k = 0; k < w.indices.size(); ++k) {
                const double value = w.values[k];
                if (value != cachedWeight) {
                    partial = RrGeoPartialTransform(p.transform, value);
                    cachedWeight = value;
                }
                RrVec3f &point = data[size_t(w.indices[k])];
                point = RrGeoToVec3f(
                    partial.TransformAffine(RrGeoToVec3d(point)));
            }
            return true;
        }
        for (size_t k = 0; k < w.indices.size(); ++k) {
            RrVec3f &point = data[size_t(w.indices[k])];
            point = RrGeoToVec3f(RrGeoApplyWeightedMatrix(
                RrGeoToVec3d(point), p.transform, w.values[k]));
        }
        return true;
    }
    std::vector<float> weights(count);
    if (!p.weights.ResolveAll(count, &weights)) {
        return false;  // cardinality mismatch fails atomically
    }
    RrGeoApplyMatrixKernelRange(p, weights.data(), 0, count, pts->data());
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

    static const std::array<RrVec3d, 4> unitRest = {
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
                         double strength)
{
    const double s = std::min(std::max(strength, 0.0), 1.0);
    if (points->empty() || s <= 0.0) {
        return;
    }
    const std::vector<std::vector<int>> adjacency = RrGeoBuildAdjacency(
        points->size(), faceVertexCounts, faceVertexIndices);
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

double
RrGeoBernstein(int degree, int index, double t)
{
    double coefficient = 1.0;
    for (int k = 0; k < index; ++k) {
        coefficient *= double(degree - k) / double(index - k);
    }
    return coefficient * std::pow(t, index) *
           std::pow(1.0 - t, degree - index);
}

void
RrGeoApplyLattice(std::vector<RrVec3f> *points,
                 const std::vector<RrVec3f> &restPoints,
                 const std::vector<RrVec3f> &restCage,
                 const std::vector<RrVec3f> &posedCage,
                 const RrVec3i &divisions)
{
    const size_t cageCount = size_t(divisions[0]) * size_t(divisions[1]) *
                             size_t(divisions[2]);
    if (points->empty() || restPoints.size() != points->size() ||
        restCage.size() != cageCount || posedCage.size() != cageCount ||
        divisions[0] < 2 || divisions[1] < 2 || divisions[2] < 2) {
        return;  // invalid cage description: pass through
    }

    RrVec3f lo = restCage[0], hi = restCage[0];
    for (const RrVec3f &c : restCage) {
        for (int a = 0; a < 3; ++a) {
            lo[a] = std::min(lo[a], c[a]);
            hi[a] = std::max(hi[a], c[a]);
        }
    }
    const RrVec3f size = hi - lo;
    if (size[0] <= 0 || size[1] <= 0 || size[2] <= 0) {
        return;
    }

    std::vector<RrVec3f> cageDeltas(cageCount);
    for (size_t i = 0; i < cageCount; ++i) {
        cageDeltas[i] = posedCage[i] - restCage[i];
    }

    const int dx = divisions[0], dy = divisions[1], dz = divisions[2];
    for (size_t i = 0; i < points->size(); ++i) {
        RrVec3f uvw;
        for (int a = 0; a < 3; ++a) {
            uvw[a] = std::min(
                1.0f, std::max(0.0f, (restPoints[i][a] - lo[a]) / size[a]));
        }
        RrVec3f delta(0.0f);
        for (int c = 0; c < dz; ++c) {
            const double bc = RrGeoBernstein(dz - 1, c, uvw[2]);
            for (int b = 0; b < dy; ++b) {
                const double bb = RrGeoBernstein(dy - 1, b, uvw[1]);
                for (int a = 0; a < dx; ++a) {
                    const double ba = RrGeoBernstein(dx - 1, a, uvw[0]);
                    delta += cageDeltas[(c * dy + b) * dx + a] *
                             float(ba * bb * bc);
                }
            }
        }
        (*points)[i] += delta;
    }
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

bool
RrGeoBuildWireBasis(const RrVec2f *bindCoords, size_t bindCount,
                   size_t meshPointCount, const std::vector<int> &indices,
                   int order, const std::vector<double> &knots,
                   size_t controlPointCount, double dropoffDistance,
                   RrGeoWireBasis *basis)
{
    const size_t n = controlPointCount;
    if (!basis || !bindCoords || order < 1 || order > 16 || n < size_t(order) ||
        knots.size() != n + size_t(order) ||
        !(knots[size_t(order - 1)] < knots[n]) ||
        (bindCount != meshPointCount && bindCount != indices.size())) {
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

bool
RrGeoApplyWire(std::vector<RrVec3f> *points,
              const RrGeoNurbsCurve &restCurve,
              const RrGeoNurbsCurve &posedCurve,
              const RrVec2f *bindCoords, size_t bindCount,
              double dropoffDistance, size_t begin, size_t end)
{
    if (!points || !bindCoords || !restCurve.IsValid() ||
        !posedCurve.IsValid() ||
        restCurve.order != posedCurve.order ||
        restCurve.points->size() != posedCurve.points->size() ||
        *restCurve.knots != *posedCurve.knots ||
        bindCount != points->size()) {
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
        const RrVec3f delta = posedCurve.Evaluate(u) - restCurve.Evaluate(u);
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

bool
RrGeoApplySkinKernelRange(const RrGeoMoverParameters &p,
                         const RrGeoSkinTransformsView &transforms,
                         size_t begin, size_t end, std::vector<RrVec3f> *pts)
{
    const RrGeoSkinLayout layout =
        RrGeoSkinLayoutForPacket(p, transforms, pts->size());
    RrGeoSkinLayout part = layout;
    part.indices = layout.indices + begin * layout.elementSize;
    part.weights = layout.weights + begin * layout.elementSize;
    part.indexCount = (end - begin) * layout.elementSize;
    part.pointCount = end - begin;
    RrVec3f *const points = pts->data();
    static const bool useSimd = RrGeoGetenvBool("RIGEXEC_ENABLE_SIMD", true);

    if (p.skinningMethod == "classicLinear") {
        if (useSimd) {
            RrGeoApplyLinearBlendSkinSimd(
                points + begin, points + begin, part, transforms.rows);
        } else {
            RrGeoApplyLinearBlendSkin(
                points + begin, points + begin, part);
        }
        return true;
    }
    if (p.skinningMethod == "dualQuaternion") {
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
    const RrGeoSkinTransformsView &transforms, std::vector<RrVec3f> *pts)
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
    return RrGeoApplySkinKernelRange(p, transforms, 0, count, pts);
}

struct RrGeoBlendSampleData {
    float activation = 1.0f;
    std::shared_ptr<const RrGeoBlendLayout> layout;
    std::vector<RrVec3f> points;
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
        (*deltas)[i] += (sample.points[i] - base[i]) * scale;
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
                       sample.points.empty())
                    : sample.points.size() == base.size();
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
                loSample ? &loSample->points : nullptr;
            const std::vector<RrVec3f> &hiPts = hiSample.points;
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

bool
RrGeoApplyBlendShapeKernel(const RrGeoMoverParameters &p,
                           std::vector<RrVec3f> *pts)
{
    const size_t count = pts->size();
    if (p.blendDeltas.size() != count) {
        return false;
    }
    std::vector<float> envelope;
    if (!p.weights.ResolveAll(count, &envelope)) {
        return false;
    }

    std::vector<RrVec3f> transported;
    const std::vector<RrVec3f> *deltas = &p.blendDeltas;
    if (p.blendSurfaceFrame) {
        if (!RrGeoTransportSurfaceOffsets(p.restPoints, *pts,
                                          p.topologyCounts, p.topologyIndices,
                                          p.blendDeltas, &transported)) {
            return false;
        }
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
            preceding, preceding + delta[i], weight[i]);
    }
    return true;
}

bool
RrGeoApplyDerivedKernel(int op, const RrGeoMoverParameters &p,
                       std::vector<RrVec3f> *pts)
{
    const bool extent = op == RrGeoOpRecomputeExtent;
    const std::vector<RrVec3f> values =
        extent ? RrGeoComputeExtent(p.auxPoints, p.widths)
               : RrGeoComputeVertexNormals(p.auxPoints, p.topologyCounts,
                                           p.topologyIndices);
    if (values.empty() || (extent && values.size() != 2)) {
        return false;
    }

    if (pts->size() != values.size()) {
        return false;
    }
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
    return w.valid && w.representation == "sparse" &&
           w.defaultWeight == 0.0f && w.indices.size() == w.values.size() &&
           (w.rangePolicy.empty() || w.rangePolicy == "strict" ||
            w.rangePolicy == "clamp");
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
                     std::unordered_map<uint64_t, RrGeoWireBasisEntry> *cache)
{
    uint64_t h = 1469598103934665603ull;
    h = RrGeoHashBytes(h, p.wireBindCoords.data(),
                       p.wireBindCoords.size() * sizeof(RrVec2f));
    h = RrGeoHashBytes(h, indices.data(), indices.size() * sizeof(int));
    h = RrGeoHashBytes(h, p.curveKnots.data(),
                       p.curveKnots.size() * sizeof(double));
    const size_t controlPoints = p.restPoints.size();
    h = RrGeoHashBytes(h, &p.curveOrder, sizeof(p.curveOrder));
    h = RrGeoHashBytes(h, &controlPoints, sizeof(controlPoints));
    h = RrGeoHashBytes(h, &meshPoints, sizeof(meshPoints));
    h = RrGeoHashBytes(h, &p.dropoffDistance, sizeof(p.dropoffDistance));

    const auto matches = [&](const RrGeoWireBasisEntry &e) {
        return e.order == p.curveOrder && e.controlPoints == controlPoints &&
               e.meshPoints == meshPoints && e.dropoff == p.dropoffDistance &&
               e.knots == p.curveKnots && e.indices == indices &&
               e.binds.size() == p.wireBindCoords.size() &&
               std::equal(e.binds.begin(), e.binds.end(),
                          p.wireBindCoords.cbegin());
    };
    {
        const auto it = cache->find(h);
        if (it != cache->end() && matches(it->second)) {
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
    if (cache->size() > 512) {
        cache->clear();
    }
    (*cache)[h] = std::move(entry);
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
    if (!p.externalFrame) {
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
    const std::vector<uint8_t> &frame = *p.externalFrame;
    if (!p.external->kernel.apply(p.external->state.get(), frame.data(),
                                  frame.size(), phased.data(), phased.size(),
                                  xyz.data(), pts->size())) {
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
    std::unordered_map<uint64_t, RrGeoWireBasisEntry> *wireCache)
{
    switch (op) {
    case RrGeoOpExternal:
        return RrGeoApplyExternal(p, pts);
    case RrGeoOpMatrix:
        return RrGeoApplyMatrixKernel(p, pts);
    case RrGeoOpSkin:
        return RrGeoApplySkinKernelWithTransforms(
            p, RrGeoSkinTransformsOf(p), pts);
    case RrGeoOpBlendShape:
        return RrGeoApplyBlendShapeKernel(p, pts);
    case RrGeoOpVolumeCorrect:
        RrGeoApplyVolumeCorrect(pts, p.referenceVolume, p.strength);
        return true;
    case RrGeoOpSmooth:
        RrGeoApplyLaplacianSmooth(
            pts, p.topologyCounts, p.topologyIndices, p.strength);
        return true;
    case RrGeoOpDeltaMush:
        return RigExecApplyDeltaMushKernel<RrVec3f, RrVec3d>(
            pts, p.restPoints, p.topologyCounts, p.topologyIndices,
            p.mushIterations, p.mushStep, p.mushPinBorders,
            p.mushDistanceWeight, p.mushDisplacement);
    case RrGeoOpWrinkle:
        return RigExecApplyWrinkleKernel<RrVec3f, RrVec3d>(
            pts, p.restPoints, p.topologyCounts, p.topologyIndices,
            p.wrinkleSettings);
    case RrGeoOpLattice:
        if (p.restPoints.size() != pts->size()) {
            return false;  // cardinality mismatch fails atomically
        }
        RrGeoApplyLattice(
            pts, p.restPoints, p.auxPoints, p.auxPointsB, p.divisions);
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
        if (p.auxPoints.size() != p.restPoints.size()) {
            return false;
        }
        const RrGeoNurbsCurve rest{&p.restPoints, p.curveOrder,
                                  &p.curveKnots};
        const RrGeoNurbsCurve posed{&p.auxPoints, p.curveOrder,
                                   &p.curveKnots};
        if (!rest.IsValid() || !posed.IsValid()) {
            return false;
        }
        if (RrGeoWireTakesSparseEnvelope(p.weights)) {
            const RrGeoWeightPacket &w = p.weights;
            for (size_t k = 0; k < w.indices.size(); ++k) {
                if (w.indices[k] < 0 || size_t(w.indices[k]) >= pts->size() ||
                    (k > 0 && w.indices[k] <= w.indices[k - 1]) ||
                    !std::isfinite(w.values[k]) || w.values[k] < 0.0f ||
                    w.values[k] > 1.0f) {
                    return false;
                }
            }
            const std::shared_ptr<const RrGeoWireBasis> basis =
                RrGeoCachedWireBasis(p, w.indices, pts->size(), wireCache);
            if (!basis) {
                return false;
            }
            return RrGeoApplyWireBasis(pts, *basis, w.indices, w.values,
                                       p.restPoints, p.auxPoints);
        }
        if (p.wireBindCoords.size() != pts->size()) {
            return false;  // a sparse bind table needs a sparse envelope
        }
        // The runtime runs serially; a point range is an independent
        // sub-problem, so the serial call is the parallel loop's answer.
        return RrGeoApplyWire(pts, rest, posed, p.wireBindCoords.data(),
                              p.wireBindCoords.size(),
                              p.dropoffDistance, 0, pts->size());
    }
    case RrGeoOpRecomputeNormals:
    case RrGeoOpRecomputeExtent:
        return RrGeoApplyDerivedKernel(op, p, pts);
    }
    return false;
}

bool
RrGeoRunRevisionKernel(
    int op, const RrGeoMoverParameters &p, std::vector<RrVec3f> *pts,
    std::unordered_map<uint64_t, RrGeoWireBasisEntry> *wireCache)
{
    if (!p.valid || p.kind != RrGeoKindToken(op)) {
        return false;
    }
    if (op == RrGeoOpMatrix ||
        op == RrGeoOpBlendShape ||
        op == RrGeoOpRecomputeNormals ||
        op == RrGeoOpRecomputeExtent ||
        (op == RrGeoOpWire &&
         RrGeoWireTakesSparseEnvelope(p.weights))) {
        return RrGeoApplyRevisionKernel(op, p, pts, wireCache);
    }

    const bool fullStrengthEnvelope = RrGeoEnvelopeIsFullStrength(p.weights);
    const size_t precedingSize = pts->size();
    std::vector<RrVec3f> preceding;
    if (!fullStrengthEnvelope) {
        preceding = *pts;
    }
    if (!RrGeoApplyRevisionKernel(op, p, pts, wireCache)) {
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
        RrGeoBlendEnvelopeAll(preceding.data(), envelope.data(),
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
// topologies, blend layouts, the pathReads lookup and the
// per-frame record pointer. Mirrors GeomChain/GeomRevision/GeomChunk.

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
        RrGeoMoverParameters parameters;
        RrGeoMoverParameters lastParameters;
        RrGeoMoverStatus status;
        RrGeoMoverStatus lastStatus;
        bool ran = false;
        bool created = true;
        bool executed = false;
        bool staticDirty = false;
        std::vector<RrVec3f> output;
        int currentSource = -1;
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
        std::shared_ptr<const RrGeoSkinTopology> partitionTopology;
        std::vector<float> envelope;
        bool envelopeOk = false;
        bool fullStrength = false;
        bool layoutUsable = false;
        float defaultWeight = 0.0f;
        float lastDefaultWeight = 0.0f;
        bool partitionStale = false;
        size_t precedingCount = 0;
        int partitionElementSize = 0;
        size_t partitionIndexCount = 0;
        size_t partitionPointCount = 0;
        RrGeoWeightPacket currentPhasePacket;
        std::vector<float> weightField;
        bool weightFieldPublished = false;
        std::vector<RrVec3f> lastAuxPoints;
        std::string resultStatus;
        Revision() { transform.SetIdentity(); }
    };
    struct Chain {
        std::vector<RrVec3f> lastBase;
        bool haveResult = false;
        std::vector<RrVec3f> result;
        std::string resultStatus;
        std::vector<RrVec3f> spare;
        bool scheduleDirty = true;
        std::vector<Revision> revisions;
        uint32_t createdCount = 0;
        uint32_t scheduleCount = 0;
    };
    struct Derived {
        std::vector<RrVec3f> lastBase;
        Revision revision;
        std::vector<RrVec3f> result;
        std::vector<RrVec3f> spare;
        bool haveResult = false;
        bool baseDirty = false;
        uint32_t createdCount = 0;
        uint32_t scheduleCount = 0;
    };

    bool stringsIndexed = false;
    std::unordered_map<std::string, uint32_t> stringToId;
    std::unordered_map<uint32_t, std::string> idToText;
    const RigExecWireFrameInputs *frame = nullptr;
    std::map<std::pair<uint32_t, uint8_t>, const RigExecWirePathValue *>
        pathReads;
    // The connection-following scalars this run, in the Computed section's
    // pathScalarReads order; pathReads points at them in place of the
    // recorded values.
    std::vector<RigExecWirePathValue> pathReadValues;
    std::map<std::pair<uint32_t, uint32_t>, const RigExecWireRefusedLayout *>
        refusedLayouts;
    std::vector<Chain> chains;
    std::vector<Derived> derived;
    std::unordered_map<uint64_t, RrGeoWireBasisEntry> wireBasis;
    std::vector<std::shared_ptr<const RrGeoSkinTopology>> epochTopologies;
    std::vector<std::shared_ptr<const RrGeoSkinTopology>>
        epochPartitionTopologies;
};

namespace {

RrGeometryScratch *
RrGeoScratch(RrProgram *program)
{
    return static_cast<RrGeometryScratch *>(program->geo.get());
}

const RigExecWirePathValue *
RrGeoPathRead(const RrGeometryScratch *scratch, uint32_t path, bool wasDefault)
{
    if (path == 0) {
        return nullptr;
    }
    const auto it = scratch->pathReads.find(
        std::make_pair(path, uint8_t(wasDefault ? 1 : 0)));
    return it == scratch->pathReads.end() ? nullptr : it->second;
}

uint32_t
RrGeoStringId(RrProgram *program, RrGeometryScratch *scratch,
              const std::string &text)
{
    if (!scratch->stringsIndexed) {
        for (uint32_t id = 0;; ++id) {
            std::string entry;
            if (!program->strings ||
                !program->strings->GetString(id, &entry)) {
                break;
            }
            scratch->stringToId.emplace(entry, id);
        }
        scratch->stringsIndexed = true;
    }
    const auto it = scratch->stringToId.find(text);
    return it == scratch->stringToId.end() ? 0 : it->second;
}

struct RrGeoTextContext {
    RrProgram *program = nullptr;
    RrGeometryScratch *scratch = nullptr;
};

const std::string *
RrGeoSnapshotText(uint32_t id, void *context)
{
    RrGeoTextContext *ctx = static_cast<RrGeoTextContext *>(context);
    auto it = ctx->scratch->idToText.find(id);
    if (it != ctx->scratch->idToText.end()) {
        return &it->second;
    }
    std::string text;
    if (!ctx->program->GetText(id, &text)) {
        return nullptr;
    }
    return &ctx->scratch->idToText.emplace(id, text).first->second;
}

RrGeoWeightPacket
RrGeoStorePacket(const RrProgram *program, const RrWeightPacket &packet)
{
    RrGeoWeightPacket out;
    out.representation = program->TextOrEmpty(packet.representation);
    out.rangePolicy = program->TextOrEmpty(packet.rangePolicy);
    out.values = packet.values;
    out.indices = packet.indices;
    out.defaultWeight = packet.defaultWeight;
    out.valid = packet.valid;
    return out;
}

// The cross-check (RrProgram::CrossCheckThisRun) of a computed current-phase
// packet against the frame record's: every field bit for bit, the token
// fields as text. False with the text naming the first field that
// differs.
bool
RrGeoCrossCheckPhasePacket(const RrProgram *program,
                           const RigExecWireFrameInputs *record,
                           size_t chain, size_t revision,
                           const RrGeoWeightPacket &computed,
                           RrStepOutput *output, std::string *error)
{
    const std::string field = "revisionPhasePackets[" +
                              std::to_string(chain) + "][" +
                              std::to_string(revision) + "]";
    const auto fail = [error](const std::string &why) {
        if (error) {
            *error = why;
        }
        return false;
    };
    if (!record || chain >= record->revisionPhasePackets.size() ||
        revision >= record->revisionPhasePackets[chain].size()) {
        return fail("cross-check: the frame record carries no " + field);
    }
    const RigExecWireWeightPacket &recorded =
        record->revisionPhasePackets[chain][revision];
    const std::string representation =
        program->TextOrEmpty(recorded.representation);
    if (computed.representation != representation) {
        return fail(RrCrossCheckMismatch(field + ".representation",
                                         computed.representation,
                                         representation));
    }
    const std::string rangePolicy =
        program->TextOrEmpty(recorded.rangePolicy);
    if (computed.rangePolicy != rangePolicy) {
        return fail(RrCrossCheckMismatch(field + ".rangePolicy",
                                         computed.rangePolicy, rangePolicy));
    }
    if (computed.values.size() != recorded.values.size()) {
        return fail("cross-check mismatch at " + field +
                    ".values: computed " +
                    std::to_string(computed.values.size()) +
                    " value(s), recorded " +
                    std::to_string(recorded.values.size()));
    }
    for (size_t i = 0; i < computed.values.size(); ++i) {
        if (!RrSameFloatBits(computed.values[i], recorded.values[i])) {
            return fail(RrCrossCheckMismatch(
                field + ".values[" + std::to_string(i) + "]",
                computed.values[i], recorded.values[i]));
        }
    }
    if (computed.indices.size() != recorded.indices.size()) {
        return fail("cross-check mismatch at " + field +
                    ".indices: computed " +
                    std::to_string(computed.indices.size()) +
                    " index(es), recorded " +
                    std::to_string(recorded.indices.size()));
    }
    for (size_t i = 0; i < computed.indices.size(); ++i) {
        if (int64_t(computed.indices[i]) != int64_t(recorded.indices[i])) {
            return fail("cross-check mismatch at " + field + ".indices[" +
                        std::to_string(i) + "]: computed " +
                        std::to_string(computed.indices[i]) +
                        ", recorded " + std::to_string(recorded.indices[i]));
        }
    }
    if (!RrSameFloatBits(computed.defaultWeight, recorded.defaultWeight)) {
        return fail(RrCrossCheckMismatch(field + ".defaultWeight",
                                         computed.defaultWeight,
                                         recorded.defaultWeight));
    }
    if (computed.valid != recorded.valid) {
        return fail("cross-check mismatch at " + field + ".valid: computed " +
                    (computed.valid ? "true" : "false") + ", recorded " +
                    (recorded.valid ? "true" : "false"));
    }
    ++output->crossChecked[RrCrossCheckPhasePacket];
    return true;
}

std::shared_ptr<const RrGeoSkinTopology>
RrGeoWireTopology(const RigExecWireSkinTopology &wire)
{
    if (!wire.hasTopology) {
        return nullptr;
    }
    auto topology = std::make_shared<RrGeoSkinTopology>();
    topology->indices.assign(wire.indices.begin(), wire.indices.end());
    topology->weights = wire.weights;
    topology->elementSize = wire.elementSize;
    topology->pointCount = size_t(wire.pointCount);
    topology->influenceCount = size_t(wire.influenceCount);
    topology->validated = wire.validated;
    return topology;
}

std::shared_ptr<const RrGeoBlendLayout>
RrGeoWireLayout(const RigExecWireBlendChannel::Sample &wire)
{
    if (!wire.hasLayout) {
        return nullptr;
    }
    auto layout = std::make_shared<RrGeoBlendLayout>();
    layout->offsets.reserve(wire.offsets.size());
    for (const RigExecWireVec3f &p : wire.offsets) {
        layout->offsets.push_back(RrVec3f(p[0], p[1], p[2]));
    }
    layout->indices.assign(wire.indices.begin(), wire.indices.end());
    layout->pointCount = size_t(wire.pointCount);
    layout->valid = wire.layoutValid;
    return layout;
}

std::shared_ptr<const RrGeoBlendLayout>
RrGeoRefusedLayout(const RigExecWireRefusedLayout &wire)
{
    auto layout = std::make_shared<RrGeoBlendLayout>();
    layout->offsets.reserve(wire.offsets.size());
    for (const RigExecWireVec3f &p : wire.offsets) {
        layout->offsets.push_back(RrVec3f(p[0], p[1], p[2]));
    }
    layout->indices.assign(wire.indices.begin(), wire.indices.end());
    layout->pointCount = size_t(wire.pointCount);
    layout->valid = wire.valid;
    return layout;
}

// A typed stage read: the resolved-inputs overlay first (RrInputsOverlay:
// a standing override or a property-chain result, exact type, as
// RigExecResolvedInputs::GetAttribute checks it), then this run's stage
// value (a connection-following scalar evaluated over the input slots, else
// the recorded one), else the fallback. Array sites take the phase
// overlay's points instead of the property results, which carry no arrays.
bool
RrGeoReadBool(const RrProgram *program,
              const RrGeometryScratch *scratch, uint32_t path, bool wasDefault,
              bool fallback)
{
    const RigExecWirePathValue *read = RrGeoPathRead(scratch, path, wasDefault);
    if (read && read->tag == RigExecWirePathValue::Tag::Bool) {
        return read->boolean;
    }
    (void)program;
    return fallback;
}

float
RrGeoReadFloat(const RrProgram *program,
               const RrGeometryScratch *scratch, uint32_t path, bool wasDefault,
               float fallback)
{
    v4::RigExecWireValue held;
    if (path != 0 &&
        RrInputsOverlay(program, path, v4::InputTag::Float, &held)) {
        return RrWireValueFloat(held);
    }
    const RigExecWirePathValue *read = RrGeoPathRead(scratch, path, wasDefault);
    if (read && read->tag == RigExecWirePathValue::Tag::Float) {
        return read->f32;
    }
    return fallback;
}

double
RrGeoReadDouble(const RrProgram *program,
                const RrGeometryScratch *scratch, uint32_t path,
                bool wasDefault, double fallback)
{
    v4::RigExecWireValue held;
    if (path != 0 &&
        RrInputsOverlay(program, path, v4::InputTag::Double, &held)) {
        double value = 0.0;
        std::memcpy(&value, &held.bits, sizeof(value));
        return value;
    }
    const RigExecWirePathValue *read = RrGeoPathRead(scratch, path, wasDefault);
    if (read && read->tag == RigExecWirePathValue::Tag::Double) {
        return read->f64;
    }
    return fallback;
}

std::string
RrGeoReadToken(const RrProgram *program,
               const RrGeometryScratch *scratch, uint32_t path, bool wasDefault,
               const std::string &fallback)
{
    const RigExecWirePathValue *read = RrGeoPathRead(scratch, path, wasDefault);
    if (read && read->tag == RigExecWirePathValue::Tag::Token) {
        return program->TextOrEmpty(read->token);
    }
    return fallback;
}

void
RrGeoReadVec3fArray(const RrProgram *program, RrGeometryScratch *scratch,
                    uint32_t path, bool wasDefault,
                    const RrSnapshotValue *overlay, std::vector<RrVec3f> *out)
{
    if (overlay && overlay->tag == RrSnapshotValue::Tag::Points) {
        *out = overlay->points;
        return;
    }
    const RigExecWirePathValue *read = RrGeoPathRead(scratch, path, wasDefault);
    if (read && read->tag == RigExecWirePathValue::Tag::Vec3fArray) {
        out->clear();
        out->reserve(read->vec3s.size());
        for (const RigExecWireVec3f &p : read->vec3s) {
            out->push_back(RrVec3f(p[0], p[1], p[2]));
        }
        (void)program;
        return;
    }
    out->clear();
}

void
RrGeoReadIntArray(const RrGeometryScratch *scratch, uint32_t path,
                 bool wasDefault, std::vector<int> *out)
{
    const RigExecWirePathValue *read = RrGeoPathRead(scratch, path, wasDefault);
    if (read && read->tag == RigExecWirePathValue::Tag::IntArray) {
        out->assign(read->ints.begin(), read->ints.end());
        return;
    }
    out->clear();
}

void
RrGeoReadFloatArray(const RrProgram *program,
                   const RrGeometryScratch *scratch, uint32_t path,
                   bool wasDefault, bool overlay, std::vector<float> *out)
{
    (void)program;
    (void)overlay;
    const RigExecWirePathValue *read = RrGeoPathRead(scratch, path, wasDefault);
    if (read && read->tag == RigExecWirePathValue::Tag::FloatArray) {
        *out = read->floats;
        return;
    }
    out->clear();
}

void
RrGeoReadVec2fArray(const RrGeometryScratch *scratch, uint32_t path,
                   bool wasDefault, std::vector<RrVec2f> *out)
{
    const RigExecWirePathValue *read = RrGeoPathRead(scratch, path, wasDefault);
    if (read && read->tag == RigExecWirePathValue::Tag::Vec2fArray) {
        out->clear();
        out->reserve(read->vec2s.size());
        for (const RigExecWireVec2f &p : read->vec2s) {
            out->push_back(RrVec2f(p[0], p[1]));
        }
        return;
    }
    out->clear();
}

void
RrGeoReadDoubleArray(const RrGeometryScratch *scratch, uint32_t path,
                    bool wasDefault, std::vector<double> *out)
{
    const RigExecWirePathValue *read = RrGeoPathRead(scratch, path, wasDefault);
    if (read && read->tag == RigExecWirePathValue::Tag::DoubleArray) {
        *out = read->doubles;
        return;
    }
    out->clear();
}

RrVec3i
RrGeoReadVec3i(const RrGeometryScratch *scratch, uint32_t path, bool wasDefault,
               const RrVec3i &fallback)
{
    const RigExecWirePathValue *read = RrGeoPathRead(scratch, path, wasDefault);
    if (read && read->tag == RigExecWirePathValue::Tag::Vec3i) {
        return RrVec3i(read->vec3i[0], read->vec3i[1], read->vec3i[2]);
    }
    return fallback;
}

// The revision's phase overlay: snapshot values for the binding's declared
// phase inputs, looked up when the assembler reaches a phased site.
const RrSnapshotValue *
RrGeoPhaseOverlay(RrProgram *program, RrGeometryScratch *scratch,
                  const RigExecWireRevisionBinding &binding, uint32_t path)
{
    for (size_t i = 0; i < binding.phaseInputs.size() &&
                       i < binding.phases.size();
         ++i) {
        if (binding.phaseInputs[i] != path) {
            continue;
        }
        const RigExecWireReadPhase &phase = binding.phases[i];
        if (phase.kind == 0) {
            return nullptr;  // base: the stage value, no overlay
        }
        RrGeoTextContext context{program, scratch};
        return program->store.runSnapshots.Lookup(
            path, phase.kind, phase.prim, binding.moverPath,
            RrGeoSnapshotText, &context);
    }
    return nullptr;
}

}  // namespace

bool
RrGeometrySizeScratch(RrProgram *program, std::string *error)
{
    auto scratch = std::make_shared<RrGeometryScratch>();
    const RigExecWireDomainGeometry &geo = *program->geometry;
    RrStore &store = program->store;

    scratch->chains.resize(geo.chains.size());
    scratch->epochTopologies.resize(geo.revisionIndex.size());
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
            if (wire.topologyResolved) {
                rev.topology = RrGeoWireTopology(wire.topology);
                scratch->epochTopologies[id] = rev.topology;
            }
            rev.partitionTopology =
                RrGeoWireTopology(wire.partitionTopology);
            scratch->epochPartitionTopologies[id] = rev.partitionTopology;
            rev.partitionElementSize = wire.partitionElementSize;
            rev.partitionIndexCount = size_t(wire.partitionIndexCount);
            rev.partitionPointCount = size_t(wire.partitionPointCount);
            store.revisionPublish[id].weightFieldTarget =
                wire.weightFieldTarget;
        }
    }
    scratch->derived.resize(geo.derivedIndex.size());
    for (size_t d = 0; d < geo.derivedIndex.size(); ++d) {
        if (!RrGeoOpName(
                geo.chains[size_t(geo.derivedIndex[d].first)]
                    .derived[size_t(geo.derivedIndex[d].second)]
                    .revision.op)) {
            if (error) {
                *error = "geometry references unknown revision op " +
                         std::to_string(
                             geo.chains[size_t(geo.derivedIndex[d].first)]
                                 .derived[size_t(geo.derivedIndex[d].second)]
                                 .revision.op);
            }
            return false;
        }
    }

    scratch->pathReadValues.resize(
        program->inputState.computed->pathScalarReads.size());
    program->geo = std::move(scratch);
    return true;
}

namespace {

// Step plumbing: the frame record, step labels, mover-attribute paths.

const RigExecWireFrameInputs *
RrGeoFindRecord(const RrProgram *program, double time)
{
    if (!program || !program->inputs) {
        return nullptr;
    }
    // SetFrame matched exactly, so the step's time is one of these.
    for (const RigExecWireFrameInputs &record : program->inputs->frames) {
        if (record.frame == time) {
            return &record;
        }
    }
    return nullptr;
}

std::string
RrGeoStepLabel(const RrProgram *program, size_t step)
{
    if (!program || !program->steps ||
        step >= program->steps->size()) {
        return "step " + std::to_string(step);
    }
    return "step " + program->TextOrEmpty((*program->steps)[step].label);
}

// A mover-relative attribute's path id: the prim path the bake recorded
// the read under, composed the way SdfPath prints a property path. Zero
// when the table holds no such path, which the typed readers below take
// as "read the fallback" -- the same answer a missing attribute gives.
uint32_t
RrGeoMoverAttrId(RrProgram *program, RrGeometryScratch *scratch,
                 const RigExecWireRevision &wire, const char *attr)
{
    const uint32_t prim =
        wire.moverPrim != 0 ? wire.moverPrim : wire.moverPath;
    const std::string text = program->TextOrEmpty(prim);
    if (text.empty() || wire.moverPrim == 0) {
        return 0;
    }
    return RrGeoStringId(program, scratch, text + "." + attr);
}

int
RrGeoReadInt(const RrGeometryScratch *scratch, uint32_t path,
             bool wasDefault, int fallback)
{
    const RigExecWirePathValue *read = RrGeoPathRead(scratch, path, wasDefault);
    if (read && read->tag == RigExecWirePathValue::Tag::Int) {
        return int(read->i32);
    }
    return fallback;
}

// The points entering (r == 0: the base; else after r - 1) and after
// revision r of a scratch chain. Ports of PointsBefore/PointsAfter.
void
RrGeoPointsBefore(const RrGeometryScratch::Chain &chain, size_t r,
                  const RrVec3f **points, size_t *count)
{
    static const RrVec3f *empty = nullptr;
    if (!points || !count) {
        return;
    }
    const int source =
        r == 0 ? -1 : chain.revisions[r - 1].currentSource;
    if (source < 0) {
        *points = chain.lastBase.empty() ? empty : chain.lastBase.data();
        *count = chain.lastBase.size();
        return;
    }
    const std::vector<RrVec3f> &out =
        chain.revisions[size_t(source)].output;
    *points = out.empty() ? empty : out.data();
    *count = out.size();
}

void
RrGeoPointsAfter(const RrGeometryScratch::Chain &chain, size_t r,
                 const RrVec3f **points, size_t *count)
{
    static const RrVec3f *empty = nullptr;
    if (!points || !count) {
        return;
    }
    const int source = chain.revisions[r].currentSource;
    if (source < 0) {
        *points = chain.lastBase.empty() ? empty : chain.lastBase.data();
        *count = chain.lastBase.size();
        return;
    }
    const std::vector<RrVec3f> &out =
        chain.revisions[size_t(source)].output;
    *points = out.empty() ? empty : out.data();
    *count = out.size();
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

// The revision's overlay loop: every declared phase is looked up, a hit
// overrides that input path for the assembly below, and a miss the author
// did not mean as "preceding" is diagnosed. The overlay only ever affects
// Vec3fArray and Matrix sites -- the snapshot store holds nothing else,
// so every scalar read misses it by type exactly as the baked one does.
void
RrGeoResolveRevisionPhases(RrProgram *program, RrGeometryScratch *scratch,
                           const RigExecWireRevision &wire,
                           std::vector<std::string> *diagnostics)
{
    const RigExecWireRevisionBinding &binding = wire.binding;
    const size_t count =
        std::min(binding.phaseInputs.size(), binding.phases.size());
    for (size_t i = 0; i < count; ++i) {
        const uint32_t path = binding.phaseInputs[i];
        const RigExecWireReadPhase &phase = binding.phases[i];
        if (phase.kind == 0) {
            // Base: the stage value, which nothing is recorded for --
            // and which the baked loop therefore reports as unresolvable.
            if (diagnostics) {
                diagnostics->push_back(
                    "diag " + program->TextOrEmpty(wire.moverPath) +
                    ": read phase 'base' for " +
                    program->TextOrEmpty(path) +
                    " resolved to nothing; read the authored base");
            }
            continue;
        }
        RrGeoTextContext context{program, scratch};
        const RrSnapshotValue *recorded = program->store.runSnapshots.Lookup(
            path, phase.kind, phase.prim, wire.moverPath,
            RrGeoSnapshotText, &context);
        if (!recorded && phase.kind != 1 && diagnostics) {
            diagnostics->push_back(
                "diag " + program->TextOrEmpty(wire.moverPath) +
                ": read phase '" + RrGeoPhaseName(program, phase) + "' for " +
                program->TextOrEmpty(path) +
                " resolved to nothing; read the authored base");
        }
    }
}

// Assembly: one revision's packet out of the record, the path reads and the
// epoch tables. A port of AssembleRevision plus the RigExecAssemble*
// per-operation bodies it ends in.

struct RrGeoAssembleInputs {
    RrProgram *program = nullptr;
    RrGeometryScratch *scratch = nullptr;
    const RigExecWireRevision *wire = nullptr;
    RrGeometryScratch::Revision *rev = nullptr;
    const RrVec3f *basePoints = nullptr;
    size_t basePointCount = 0;
    const RigExecWireFrameInputs *record = nullptr;
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
    const uint32_t path = RrGeoMoverAttrId(
        in.program, in.scratch, *in.wire, "inputs:enabled");
    return RrGeoReadBool(in.program, in.scratch, path, false, true);
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
            in.program->store.weightPackets[size_t(in.wire->weightObject)]);
    }
    return RrGeoWeightPacket::Constant(RrGeoAssembleDefaultWeight(in));
}

void
RrGeoReadBindingVec3fArray(const RrGeoAssembleInputs &in, uint32_t path,
                           bool wasDefault, std::vector<RrVec3f> *out)
{
    // Rest reads pass resolved=nullptr on the baked path: the phase
    // overlay only ever serves a live (time) read. A Default read
    // always replays the authored value.
    const RrSnapshotValue *overlay =
        wasDefault ? nullptr
                   : RrGeoPhaseOverlay(in.program, in.scratch,
                                        in.wire->binding, path);
    RrGeoReadVec3fArray(in.program, in.scratch, path, wasDefault, overlay,
                        out);
}

// The blend channels: each channel's weight and each sample's activation
// read over the slots, and the dense points the baked gather consumed, out
// of the record, in binding order, each channel's samples stable-sorted by
// activation. Sparse samples take the refused layout when this frame
// refused the cache and the epoch one otherwise.
bool
RrGeoAssembleBlendDeltas(const RrGeoAssembleInputs &in,
                         const std::vector<RrVec3f> &base,
                         std::vector<RrVec3f> *deltas)
{
    const RigExecWireFrameInputs *record = in.record;
    std::vector<RrGeoBlendChannel> channels;
    channels.reserve(in.wire->blendChannels.size());
    for (size_t c = 0; c < in.wire->blendChannels.size(); ++c) {
        const RigExecWireBlendChannel &bound = in.wire->blendChannels[c];
        RrGeoBlendChannel channel;
        channel.weight = RrReadBlendWeight(in.program, in.chain, in.revision,
                                           in.derived, c);
        for (size_t s = 0; s < bound.samples.size(); ++s) {
            const RigExecWireBlendChannel::Sample &boundSample =
                bound.samples[s];
            RrGeoBlendSampleData sample;
            sample.activation = RrReadBlendActivation(
                in.program, in.chain, in.revision, in.derived, c, s);
            if (record) {
                const std::vector<std::vector<std::vector<std::vector<
                    std::vector<RigExecWireVec3f>>>>> *pts =
                    in.derived ? &record->derivedBlendPoints
                               : &record->blendPoints;
                if (!boundSample.blendShape && in.chain < pts->size() &&
                    in.revision < (*pts)[in.chain].size() &&
                    c < (*pts)[in.chain][in.revision].size() &&
                    s < (*pts)[in.chain][in.revision][c].size()) {
                    const std::vector<RigExecWireVec3f> &consumed =
                        (*pts)[in.chain][in.revision][c][s];
                    sample.points.reserve(consumed.size());
                    for (const RigExecWireVec3f &p : consumed) {
                        sample.points.push_back(
                            RrVec3f(p[0], p[1], p[2]));
                    }
                }
            }
            if (boundSample.blendShape) {
                const auto refused = in.scratch->refusedLayouts.find(
                    std::make_pair(in.wire->moverPath,
                                   boundSample.samplePath));
                if (refused != in.scratch->refusedLayouts.end()) {
                    sample.layout =
                        RrGeoRefusedLayout(*refused->second);
                } else {
                    sample.layout = RrGeoWireLayout(boundSample);
                }
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
                       RrGeoMoverAttrId(in.program, in.scratch, *in.wire,
                                        "rigExec:weightBlend"),
                       false, "linear") == "radial";
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
        in.program, in.scratch,
        RrGeoMoverAttrId(in.program, in.scratch, *in.wire,
                         "rigExec:skinningMethod"),
        false, "classicLinear");
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
    RrGeoReadIntArray(in.scratch,
                      RrGeoMoverAttrId(in.program, in.scratch, *in.wire,
                                       "rigExec:jointIndices"),
                      false, &params->skinIndices);
    RrGeoReadFloatArray(in.program, in.scratch,
                        RrGeoMoverAttrId(in.program, in.scratch, *in.wire,
                                         "rigExec:jointWeights"),
                        false, false, &params->skinWeights);
    params->skinElementSize = RrGeoReadInt(
        in.scratch,
        RrGeoMoverAttrId(in.program, in.scratch, *in.wire,
                         "rigExec:elementSize"),
        false, 1);
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
                     const std::vector<RrVec3f> &base,
                     RrGeoMoverParameters *params)
{
    params->restPoints = base;
    RrGeoReadBindingVec3fArray(in, in.wire->binding.cagePoints, true,
                               &params->auxPoints);
    RrGeoReadBindingVec3fArray(in, in.wire->binding.cagePoints, false,
                               &params->auxPointsB);
    params->divisions = RrGeoReadVec3i(
        in.scratch,
        RrGeoMoverAttrId(in.program, in.scratch, *in.wire,
                         "rigExec:divisions"),
        false, RrVec3i(0, 0, 0));
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
    RrGeoReadBindingVec3fArray(in, in.wire->binding.driverCurvePoints, true,
                               &params->restPoints);
    if (in.wire->binding.driverTransformCount > 0) {
        const size_t t = size_t(in.wire->binding.driverTransformCount);
        const size_t s = size_t(in.wire->binding.driverSpaceCount);
        const size_t bt =
            size_t(in.wire->binding.driverBaseTransformCount);
        // Every operation but a skin assembles against the folded table.
        const std::vector<RrMat4d> &table = in.rev->influences;
        if (table.size() < t + s + bt) {
            return;
        }
        const size_t bs = table.size() - t - s - bt;
        std::vector<float> weights, baseWeights;
        RrGeoReadFloatArray(in.program, in.scratch,
                            RrGeoMoverAttrId(in.program, in.scratch, *in.wire,
                                             "inputs:driverWeights"),
                            false, false, &weights);
        RrGeoReadFloatArray(in.program, in.scratch,
                            RrGeoMoverAttrId(in.program, in.scratch, *in.wire,
                                             "inputs:driverBaseWeights"),
                            false, false, &baseWeights);
        const auto pick = [](size_t count, size_t j) {
            return count <= 1 ? size_t(0) : j % count;
        };
        // Which frame the points this wire moves are already in, exactly as
        // RigExecAssembleParameters reads it: "posed" is a wire that runs
        // after the skin on its target, "rest" -- the fallback -- one that
        // runs before it. See rigExec:pointFrame in the schema.
        const bool posedPoints =
            RrGeoReadToken(in.program, in.scratch,
                           RrGeoMoverAttrId(in.program, in.scratch, *in.wire,
                                            "rigExec:pointFrame"),
                           false, "rest") == "posed";
        // Which frame the driver's offset from its space is applied in,
        // exactly as RigExecAssembleParameters reads it: "local" (the
        // fallback) is T * S^-1, "posed" conjugates that into the space's
        // current frame. Both tokens are recorded reads, so the runtime
        // sees what the bake saw.
        const bool posedDelta =
            RrGeoReadToken(in.program, in.scratch,
                           RrGeoMoverAttrId(in.program, in.scratch, *in.wire,
                                            "rigExec:driverDeltaFrame"),
                           false, "local") == "posed";
        // MEASURE THE OFFSET IN THE ASSET'S OWN UNITS, and carry only the
        // finished displacement out of them. The curve, its bind distances
        // and the mesh rest points are authored at the asset's scale, while
        // the driver's offset from its space arrives in whatever scale that
        // space is in: under a master scaled to two the offset is twice as
        // long while the curve it moves has not grown. A space nothing has
        // scaled measures one, so both corrections vanish and an unscaled
        // rig is untouched, and so is a wire that asks for neither posed
        // frame.
        const auto measured = [&](size_t first, size_t count,
                                  size_t spaceFirst, size_t spaceCount,
                                  size_t j, RrVec3d *spaceScale) {
            RrMat4d m = table[first + pick(count, j)];
            if (spaceCount > 0) {
                RrMat4d space = table[spaceFirst + pick(spaceCount, j)];
                const RrVec3d k = RrGeoFrameScale(space);
                if (spaceScale) {
                    *spaceScale = k;
                }
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
            }
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
        RrGeoReadBindingVec3fArray(in, in.wire->binding.driverCurvePoints,
                                   false, &params->auxPoints);
    }
    {
        std::vector<int> order;
        RrGeoReadIntArray(in.scratch, in.wire->binding.driverCurveOrder,
                          true, &order);
        if (!order.empty()) {
            params->curveOrder = order[0];
        }
    }
    RrGeoReadDoubleArray(in.scratch, in.wire->binding.driverCurveKnots,
                         true, &params->curveKnots);
    params->dropoffDistance = double(RrGeoReadFloat(
        in.program, in.scratch,
        RrGeoMoverAttrId(in.program, in.scratch, *in.wire,
                         "inputs:dropoffDistance"),
        false, 0.0f));
    if (in.wire->binding.bindCoords != 0) {
        RrGeoReadVec2fArray(in.scratch, in.wire->binding.bindCoords, false,
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
        RrGeoReadVec2fArray(in.scratch, in.wire->binding.bindCoords, false,
                            &params->bindCoords);
        params->valid = !params->bindCoords.empty();
    } else {
        params->valid = true;
    }
}

void
RrGeoAssembleDerived(const RrGeoAssembleInputs &in,
                     const std::vector<RrVec3f> &base, bool recomputeExtent,
                     RrGeoMoverParameters *params)
{
    params->auxPoints = base;
    RrGeoReadIntArray(in.scratch, in.wire->binding.topologyCounts, false,
                      &params->topologyCounts);
    RrGeoReadIntArray(in.scratch, in.wire->binding.topologyIndices, false,
                      &params->topologyIndices);
    if (recomputeExtent) {
        RrGeoReadFloatArray(in.program, in.scratch,
                            in.wire->binding.widths, false, false,
                            &params->widths);
    }
    params->valid =
        !params->auxPoints.empty() &&
        (recomputeExtent || !params->topologyCounts.empty());
}

// A plugin mover's packet: its playback state, the frame bytes the export
// recorded for the selected frame, and its phased point inputs out of this
// run's snapshots. No kernel here makes it a valid no-op; no bytes on this
// frame -- the export's assembly failed -- leave it invalid, as it was.
void
RrGeoAssembleExternal(const RrGeoAssembleInputs &in,
                      RrGeoMoverParameters *params)
{
    RrProgram &program = *in.program;
    const auto found = program.externalIndex.find(
        std::make_pair(uint32_t(in.chain), uint32_t(in.revision)));
    if (in.derived || !program.external ||
        found == program.externalIndex.end()) {
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
    const uint32_t blob =
        program.external->frames[program.frameIndex][found->second];
    if (blob == RigExecWireExternalNoFrame) {
        return;
    }
    params->externalFrame = &program.external->blobs[blob];
    const RigExecWireRevisionBinding &binding = in.wire->binding;
    for (size_t i = 0;
         i < binding.phaseInputs.size() && i < binding.phases.size(); ++i) {
        const uint32_t path = binding.phaseInputs[i];
        const RrSnapshotValue *overlay =
            RrGeoPhaseOverlay(in.program, in.scratch, binding, path);
        const bool have =
            overlay && overlay->tag == RrSnapshotValue::Tag::Points;
        params->externalPhasedPaths.push_back(program.TextOrEmpty(path));
        params->externalPhasedHave.push_back(have ? 1 : 0);
        params->externalPhased.push_back(have ? overlay->points
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
    RrGeoResolveRevisionPhases(in.program, in.scratch, *in.wire,
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
        RrGeoReadIntArray(in.scratch, in.wire->binding.topologyCounts,
                          false, &params.topologyCounts);
        RrGeoReadIntArray(in.scratch, in.wire->binding.topologyIndices,
                          false, &params.topologyIndices);
        params.valid = !params.topologyCounts.empty();
        break;
    case RrGeoOpDeltaMush: {
        const auto attr = [&](const char *name) {
            return RrGeoMoverAttrId(in.program, in.scratch, *in.wire, name);
        };
        RrGeoReadBindingVec3fArray(in, attr("inputs:restPoints"), true,
                                 &params.restPoints);
        if (params.restPoints.empty()) params.restPoints = base;
        RrGeoReadIntArray(in.scratch, in.wire->binding.topologyCounts,
                         false, &params.topologyCounts);
        RrGeoReadIntArray(in.scratch, in.wire->binding.topologyIndices,
                         false, &params.topologyIndices);
        params.mushIterations = RrGeoReadInt(in.scratch,
            attr("inputs:iterations"), false, 10);
        params.mushStep = RrGeoReadFloat(in.program, in.scratch,
            attr("inputs:step"), false, 0.5f);
        params.mushPinBorders = RrGeoReadBool(in.program, in.scratch,
            attr("inputs:pinBorders"), false, true);
        params.mushDistanceWeight = RrGeoReadFloat(in.program, in.scratch,
            attr("inputs:distanceWeight"), false, 0.0f);
        params.mushDisplacement = RrGeoReadFloat(in.program, in.scratch,
            attr("inputs:displacement"), false, 1.0f);
        params.valid = !params.restPoints.empty() && !params.topologyCounts.empty();
        break;
    }
    case RrGeoOpWrinkle: {
        const auto attr = [&](const char *name) {
            return RrGeoMoverAttrId(in.program, in.scratch, *in.wire, name);
        };
        RrGeoReadBindingVec3fArray(in, attr("inputs:restPoints"), true,
                                 &params.restPoints);
        if (params.restPoints.empty()) params.restPoints = base;
        RrGeoReadIntArray(in.scratch, in.wire->binding.topologyCounts,
                         false, &params.topologyCounts);
        RrGeoReadIntArray(in.scratch, in.wire->binding.topologyIndices,
                         false, &params.topologyIndices);
        auto &settings = params.wrinkleSettings;
        settings.iterations = RrGeoReadInt(in.scratch,
            attr("inputs:iterations"), false, 80);
        const std::string topology = RrGeoReadToken(in.program, in.scratch,
            attr("inputs:topology"), true, "cloth");
        if (topology == "cloth") {
            settings.topology = RigExecWrinkleTopology::Cloth;
        } else if (topology == "surfaceStruts") {
            settings.topology = RigExecWrinkleTopology::SurfaceStruts;
        } else {
            params.valid = false;
            break;
        }
        settings.neighborDistance = RrGeoReadInt(in.scratch,
            attr("inputs:neighborDistance"), false, 2);
        settings.restLengthScale = RrGeoReadFloat(in.program, in.scratch,
            attr("inputs:restLengthScale"), false, 1.0f);
        settings.stretchStiffness = RrGeoReadFloat(in.program, in.scratch,
            attr("inputs:stretchStiffness"), false, 1.0f);
        settings.compressionStiffness = RrGeoReadFloat(in.program, in.scratch,
            attr("inputs:compressionStiffness"), false, 1.0f);
        settings.bendStiffness = RrGeoReadFloat(in.program, in.scratch,
            attr("inputs:bendStiffness"), false, 0.1f);
        settings.maxDisplacement = RrGeoReadFloat(in.program, in.scratch,
            attr("inputs:maxDisplacement"), false, 0.2f);
        settings.pinBorders = RrGeoReadBool(in.program, in.scratch,
            attr("inputs:pinBorders"), false, true);
        RrGeoReadIntArray(in.scratch, attr("inputs:pinPoints"), true,
                         &settings.pinPoints);
        settings.tangentPlaneCollisions = RrGeoReadBool(in.program, in.scratch,
            attr("inputs:tangentPlaneCollisions"), false, true);
        settings.tangentPlaneInset = RrGeoReadFloat(in.program, in.scratch,
            attr("inputs:tangentPlaneInset"), false, 0.0f);
        settings.wrinkleScale = RrGeoReadFloat(in.program, in.scratch,
            attr("inputs:wrinkleScale"), false, 1.0f);
        settings.smoothingIterations = RrGeoReadInt(in.scratch,
            attr("inputs:smoothingIterations"), false, 0);
        params.valid = !params.restPoints.empty() && !params.topologyCounts.empty();
        break;
    }
    case RrGeoOpLattice:
        RrGeoAssembleLattice(in, base, &params);
        break;
    case RrGeoOpSurfaceProject:
        params.strength = 1.0f;
        RrGeoReadBindingVec3fArray(in, in.wire->binding.surfacePoints,
                                   false, &params.auxPoints);
        RrGeoReadIntArray(in.scratch, in.wire->binding.topologyCounts,
                          false, &params.topologyCounts);
        RrGeoReadIntArray(in.scratch, in.wire->binding.topologyIndices,
                          false, &params.topologyIndices);
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
        RrGeoAssembleDerived(in, base, false, &params);
        break;
    case RrGeoOpRecomputeExtent:
        RrGeoAssembleDerived(in, base, true, &params);
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
    const bool atPrim = wire.binding.transformPhase.kind == 3;
    if (atPrim) {
        RrGeoTextContext context{program, scratch};
        const RrSnapshotValue *recorded = store.runSnapshots.Lookup(
            wire.binding.transform, wire.binding.transformPhase.kind,
            wire.binding.transformPhase.prim, wire.moverPath,
            RrGeoSnapshotText, &context);
        if (recorded && recorded->tag == RrSnapshotValue::Tag::Matrix) {
            rev->transform = recorded->matrix;
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
                           RrGeoMoverAttrId(program, scratch, wire,
                                            "rigExec:pointFrame"),
                           false, "rest") == "posed";
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
        if (atPrim && k < wire.binding.influences.size()) {
            RrGeoTextContext context{program, scratch};
            const RrSnapshotValue *recorded = store.runSnapshots.Lookup(
                wire.binding.influences[k],
                wire.binding.transformPhase.kind,
                wire.binding.transformPhase.prim, wire.moverPath,
                RrGeoSnapshotText, &context);
            if (recorded &&
                recorded->tag == RrSnapshotValue::Tag::Matrix) {
                entry = recorded->matrix;
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
RrGeoFoldTransformForms(RrGeometryScratch::Revision *rev)
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
    if (!RrGeoGetenvBool("RIGEXEC_ENABLE_SIMD", true)) {
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
RrGeoChunkTransformsView(const RrGeometryScratch::Chunk &chunk)
{
    RrGeoSkinTransformsView view;
    view.transforms = chunk.transforms.data();
    view.transformCount = chunk.transforms.size();
    if (RrGeoGetenvBool("RIGEXEC_ENABLE_SIMD", true) &&
        !chunk.rows.empty()) {
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

bool
RrGeoSkinRange(RrGeometryScratch::Revision *rev, const RrVec3f *preceding,
               const RrGeoSkinTransformsView &view, size_t begin,
               size_t end, bool whole)
{
    std::vector<RrVec3f> &out = rev->output;
    if (end > begin && preceding) {
        std::copy(preceding + begin, preceding + end,
                  out.begin() + long(begin));
    }
    if (whole) {
        if (!RrGeoApplySkinKernelWithTransforms(rev->parameters, view,
                                                &out)) {
            return false;
        }
    } else if (!RrGeoApplySkinKernelRange(rev->parameters, view, begin,
                                           end, &out)) {
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

bool
RrGeoFuseWholeRevision(const RrGeometryScratch::Chain &chain,
                       RrGeometryScratch::Revision *rev, size_t revisionIndex)
{
    const RrVec3f *points = nullptr;
    size_t count = 0;
    RrGeoPointsBefore(chain, revisionIndex, &points, &count);
    if (!rev->layoutUsable || !rev->envelopeOk ||
        count != rev->precedingCount || rev->output.size() != count) {
        return false;
    }
    return RrGeoSkinRange(rev, points, RrGeoWholeTransformsView(rev), 0,
                          count, true);
}

// The partition re-cut, for the frame an epoch-fixed layout moved under.
// A port of RigExecBakedPartitionRevision.

int
RrGeoGetenvInt(const char *name, int fallback)
{
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    const char *value = std::getenv(name);
#ifdef _MSC_VER
#pragma warning(pop)
#endif
    return value && *value ? std::atoi(value) : fallback;
}

size_t
RrGeoChunkVertexTarget()
{
    static const size_t target = [] {
        const int authored = RrGeoGetenvInt("RIGEXEC_BAKED_CHUNK_VERTS",
                                            4096);
        return authored < 1 ? size_t(1) : size_t(authored);
    }();
    return target;
}

size_t
RrGeoChunkCap()
{
    static const size_t cap = [] {
        const int authored =
            RrGeoGetenvInt("RIGEXEC_BAKED_MAX_CHUNKS", 32);
        return authored < 1 ? size_t(1) : size_t(authored);
    }();
    return cap;
}

bool
RrGeoChunkIsSubset(const std::vector<int32_t> &a,
                   const std::vector<int32_t> &b)
{
    size_t j = 0;
    for (const int32_t value : a) {
        while (j < b.size() && b[j] < value) {
            ++j;
        }
        if (j >= b.size() || b[j] != value) {
            return false;
        }
    }
    return true;
}

void
RrGeoPartitionRevision(RrGeometryScratch::Revision *rev, size_t influences,
                       const int *indices, size_t indexCount,
                       int elementSize, int chunkCount)
{
    const size_t slots = elementSize < 1 ? 0 : size_t(elementSize);
    const size_t points = slots ? indexCount / slots : 0;
    rev->partitionElementSize = elementSize;
    rev->partitionIndexCount = indexCount;
    rev->partitionPointCount = points;

    size_t target = RrGeoChunkVertexTarget();
    size_t count = std::max<size_t>(1, (points + target - 1) / target);
    if (count > RrGeoChunkCap()) {
        count = RrGeoChunkCap();
        target = std::max<size_t>(1, (points + count - 1) / count);
        count = std::max<size_t>(1, (points + target - 1) / target);
    }

    std::vector<RrGeometryScratch::Chunk> chunks;
    chunks.reserve(count);
    std::vector<char> claimed(influences, 0);
    for (size_t c = 0; c < count; ++c) {
        RrGeometryScratch::Chunk chunk;
        chunk.begin = int(std::min(points, c * target));
        chunk.end = int(c + 1 == count
                            ? points
                            : std::min(points, (c + 1) * target));
        for (size_t point = size_t(chunk.begin);
             point < size_t(chunk.end); ++point) {
            for (size_t slot = 0; slot < slots; ++slot) {
                const int index = indices[point * slots + slot];
                if (index < 0 || size_t(index) >= influences ||
                    claimed[size_t(index)]) {
                    continue;
                }
                claimed[size_t(index)] = 1;
                chunk.key.push_back(index);
            }
        }
        std::sort(chunk.key.begin(), chunk.key.end());
        for (const int32_t index : chunk.key) {
            claimed[size_t(index)] = 0;
        }
        chunks.push_back(std::move(chunk));
    }

    for (size_t i = 0; i + 1 < chunks.size();) {
        const size_t merged = size_t(chunks[i + 1].end - chunks[i].begin);
        const bool contains =
            RrGeoChunkIsSubset(chunks[i].key, chunks[i + 1].key);
        const bool contained =
            RrGeoChunkIsSubset(chunks[i + 1].key, chunks[i].key);
        if (merged <= target && (contains || contained)) {
            chunks[i].end = chunks[i + 1].end;
            if (contains) {
                chunks[i].key = chunks[i + 1].key;
            }
            chunks.erase(chunks.begin() + long(i) + 1);
        } else {
            ++i;
        }
    }

    if (chunkCount > 0) {
        while (chunks.size() > size_t(chunkCount)) {
            size_t best = 0;
            for (size_t i = 1; i + 1 < chunks.size(); ++i) {
                if (chunks[i + 1].end - chunks[i].begin <
                    chunks[best + 1].end - chunks[best].begin) {
                    best = i;
                }
            }
            std::vector<int32_t> key;
            std::set_union(chunks[best].key.begin(),
                           chunks[best].key.end(),
                           chunks[best + 1].key.begin(),
                           chunks[best + 1].key.end(),
                           std::back_inserter(key));
            chunks[best].key = std::move(key);
            chunks[best].end = chunks[best + 1].end;
            chunks.erase(chunks.begin() + long(best) + 1);
        }
        while (chunks.size() < size_t(chunkCount)) {
            RrGeometryScratch::Chunk chunk;
            chunk.begin = int(points);
            chunk.end = int(points);
            chunks.push_back(std::move(chunk));
        }
    }

    const bool keyed = chunks.size() > 1;
    const RrMat4d identity = RrGeoIdentity();
    for (RrGeometryScratch::Chunk &chunk : chunks) {
        chunk.ok = false;
        chunk.keyChanged = false;
        chunk.palette.clear();
        if (!keyed) {
            chunk.key.clear();
            chunk.transforms.clear();
            chunk.rows.clear();
            continue;
        }
        chunk.transforms.assign(influences, identity);
        chunk.rows.assign(influences * size_t(RrGeoSkinRowStride), 0.0f);
        for (size_t t = 0; t < influences; ++t) {
            RrGeoNarrowSkinRows(chunk.transforms[t],
                                &chunk.rows[t * size_t(RrGeoSkinRowStride)]);
        }
    }
    rev->chunks = std::move(chunks);
    rev->chunked = keyed;
}

void
RrGeoResetRevision(RrGeometryScratch::Revision *rev)
{
    rev->created = true;
    rev->ran = false;
    rev->output.clear();
    rev->currentSource = -1;
    rev->lastParameters = RrGeoMoverParameters();
    rev->lastAuxPoints.clear();
    rev->lastStatus = RrGeoMoverStatus();
}

}  // namespace

bool
RrPrologueGeometry(RrProgram *program, double time,
                   const RigExecWireFrameInputs &record,
                   std::vector<std::string> *poseDiagnostics,
                   std::string *error)
{
    (void)time;
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

    // This frame's lookups: the stage-sourced reads by (path, Default)
    // and the refused blend layouts by (mover, sample).
    scratch->frame = &record;
    scratch->pathReads.clear();
    for (const RigExecWirePathRead &read : record.pathReads) {
        scratch->pathReads[std::make_pair(read.path, read.wasDefault)] =
            &read.value;
    }
    // The scalars the assemblers read through their connections at the
    // evaluation time, evaluated over the slots in place of the recorded
    // ones.
    const RigExecWireComputed &computed = *program->inputState.computed;
    for (size_t k = 0; k < computed.pathScalarReads.size() &&
                       k < scratch->pathReadValues.size();
         ++k) {
        const RigExecWirePathScalarRead &entry = computed.pathScalarReads[k];
        RigExecWirePathValue &value = scratch->pathReadValues[k];
        if (RrPathValueFromRead(entry, RrReadPathScalar(program, entry),
                                &value)) {
            scratch->pathReads[std::make_pair(entry.path, uint8_t(0))] =
                &value;
        }
    }
    scratch->refusedLayouts.clear();
    for (const RigExecWireRefusedLayout &refused : record.refusedLayouts) {
        scratch->refusedLayouts[std::make_pair(refused.mover,
                                               refused.sample)] = &refused;
    }

    for (size_t c = 0; c < geo.chains.size(); ++c) {
        const RigExecWireChain &wireChain = geo.chains[c];
        RrGeometryScratch::Chain &chain = scratch->chains[c];
        const bool haveBase = c < record.chainHaveBase.size() &&
                              record.chainHaveBase[c] != 0 &&
                              c < record.chainBases.size();
        std::vector<RrVec3f> basePoints;
        if (haveBase) {
            basePoints.reserve(record.chainBases[c].size());
            for (const RigExecWireVec3f &p : record.chainBases[c]) {
                basePoints.push_back(RrVec3f(p[0], p[1], p[2]));
            }
        }
        if (c < store.chainHaveBase.size()) {
            store.chainHaveBase[c] = haveBase ? 1 : 0;
        }
        if (c < store.chainBases.size()) {
            store.chainBases[c] = basePoints;
        }
        chain.createdCount = 0;
        chain.scheduleCount = 0;
        if (!haveBase) {
            // Derived haveBase flags live densely by derived id; clear
            // the ones of this chain.
            for (size_t d = 0; d < geo.derivedIndex.size(); ++d) {
                if (geo.derivedIndex[d].first == int(c) &&
                    d < store.derivedHaveBase.size()) {
                    store.derivedHaveBase[d] = 0;
                }
            }
            continue;
        }
        if (chain.haveResult && basePoints.size() != chain.lastBase.size()) {
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
        const bool baseDirty =
            !chain.haveResult || basePoints != chain.lastBase;
        if (c < store.chainBaseDirty.size()) {
            store.chainBaseDirty[c] = baseDirty ? 1 : 0;
        }
        chain.lastBase = std::move(basePoints);
        for (size_t r = 0; r < chain.revisions.size(); ++r) {
            const RigExecWireRevision &wire = wireChain.revisions[r];
            RrGeometryScratch::Revision &rev = chain.revisions[r];
            const size_t id =
                size_t(geo.chainRevisionBegin[c] + int(r));
            // Epoch-fixed skin layouts: the same pointer while the
            // binding did not move. Compared by value here because the
            // wire carries two copies of one layout; adopting the
            // resolved handle reproduces the baked pointer identity.
            if (wire.skinTopologyFixed && id < scratch->epochTopologies.size()) {
                rev.topology = scratch->epochTopologies[id];
                rev.topologyResolved = true;
                const bool sameLayout =
                    (rev.topology == rev.partitionTopology) ||
                    (rev.topology && rev.partitionTopology &&
                     *rev.topology == *rev.partitionTopology);
                if (rev.chunked && !sameLayout) {
                    if (rev.topology) {
                        RrGeoPartitionRevision(
                            &rev, wire.influenceSlots.size(),
                            rev.topology->indices.data(),
                            rev.topology->indices.size(),
                            rev.topology->elementSize,
                            int(rev.chunks.size()));
                        rev.partitionTopology = rev.topology;
                    }
                } else if (rev.topology) {
                    rev.partitionTopology = rev.topology;
                }
            }
        }
        for (size_t d = 0; d < wireChain.derived.size(); ++d) {
            // The dense derived id of (c, d).
            size_t id = geo.derivedIndex.size();
            for (size_t k = 0; k < geo.derivedIndex.size(); ++k) {
                if (geo.derivedIndex[k].first == int(c) &&
                    geo.derivedIndex[k].second == int(d)) {
                    id = k;
                    break;
                }
            }
            if (id >= scratch->derived.size()) {
                if (error) {
                    *error = "derived revision names no derived target";
                }
                return false;
            }
            RrGeometryScratch::Derived &derived = scratch->derived[id];
            const bool derivedHaveBase =
                id < record.derivedHaveBase.size() &&
                record.derivedHaveBase[id] != 0 &&
                id < record.derivedBases.size();
            std::vector<RrVec3f> derivedBase;
            if (derivedHaveBase) {
                derivedBase.reserve(record.derivedBases[id].size());
                for (const RigExecWireVec3f &p : record.derivedBases[id]) {
                    derivedBase.push_back(RrVec3f(p[0], p[1], p[2]));
                }
            }
            if (id < store.derivedHaveBase.size()) {
                store.derivedHaveBase[id] = derivedHaveBase ? 1 : 0;
            }
            if (id < store.derivedBases.size()) {
                store.derivedBases[id] = derivedBase;
            }
            derived.createdCount = 0;
            derived.scheduleCount = 0;
            if (!derivedHaveBase) {
                continue;
            }
            RrGeometryScratch::Revision &rev = derived.revision;
            if (derived.haveResult &&
                derivedBase.size() != derived.lastBase.size()) {
                derived.haveResult = false;
                derived.result.clear();
                RrGeoResetRevision(&rev);
            }
            if (rev.created) {
                ++derived.createdCount;
                ++derived.scheduleCount;
                rev.created = false;
            }
            derived.baseDirty =
                !derived.haveResult || derivedBase != derived.lastBase;
            derived.lastBase = std::move(derivedBase);
            const RigExecWireRevision &wire =
                wireChain.derived[d].revision;
            if (wire.skinTopologyFixed) {
                // Derived revisions keep no epoch table of their own;
                // the resolve answers null, which routes the packet
                // through the per-frame arrays.
                rev.topologyResolved = true;
                rev.topology.reset();
            }
        }
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
        for (const uint32_t dial : wire.shaderDials) {
            dials.push_back(
                RrGeoReadDouble(program, scratch, dial, false, 0.0));
        }
        publish.matrix = RigExecPackShaderDialsT<RrMat4d>(dials);
        publish.haveMatrix = true;
        return true;
    }
    const auto vec = [&](const char *attr, const RrVec3d &fallback) {
        const RigExecWirePathValue *read = RrGeoPathRead(
            scratch, RrGeoMoverAttrId(program, scratch, wire, attr), false);
        if (read && read->tag == RigExecWirePathValue::Tag::Vec3d) {
            return RrVec3d(read->vec[0], read->vec[1], read->vec[2]);
        }
        return fallback;
    };
    RigExecSurfaceProjectorInputs<RrMat4d, RrVec3d> in;
    in.rayOrigin = vec("rigExec:rayOrigin", RrVec3d(0.0, 0.0, 0.0));
    in.rayDirection = vec("rigExec:rayDirection", RrVec3d(0.0, 0.0, 1.0));
    in.rayUp = vec("rigExec:rayUp", RrVec3d(0.0, 1.0, 0.0));
    in.shaderOffset.SetIdentity();
    if (const RigExecWirePathValue *read = RrGeoPathRead(
            scratch,
            RrGeoMoverAttrId(program, scratch, wire, "rigExec:shaderOffset"),
            false)) {
        if (read->tag == RigExecWirePathValue::Tag::Matrix4d) {
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    in.shaderOffset[i][j] = read->matrix[size_t(i * 4 + j)];
                }
            }
        }
    }
    in.reproject =
        RrGeoReadToken(program, scratch,
                       RrGeoMoverAttrId(program, scratch, wire,
                                        "rigExec:projectionMode"),
                       false, "material") == "reproject";
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
    RrGeoReadIntArray(scratch, wire.binding.topologyCounts, false, &counts);
    RrGeoReadIntArray(scratch, wire.binding.topologyIndices, false, &indices);
    const std::vector<RrVec3f> &basePoints =
        chainIndex < store.chainBases.size()
            ? store.chainBases[chainIndex]
            : std::vector<RrVec3f>();
    const std::vector<RrVec3f> &finalPoints = scratch->chains[chainIndex].result;
    publish.haveMatrix = RigExecSolveSurfaceProjectorT(
        in, basePoints, finalPoints, counts, indices,
        [](const std::vector<RrVec3f> &points, const std::vector<int> &c,
           const std::vector<int> &i) {
            return RrGeoComputeVertexNormals(points, c, i);
        },
        program->TextOrEmpty(wire.moverPath), &publish.matrix,
        &output->diagnostics);
    return true;
}

bool
RrGeoRunDerivedStep(RrProgram *program, RrGeometryScratch *scratch,
                    size_t step, const RigExecWireFrameInputs *record,
                    std::string *error)
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
    const bool haveBase = chainIndex < store.chainHaveBase.size() &&
                          store.chainHaveBase[chainIndex] != 0;
    const bool derivedHaveBase = size_t(object) < store.derivedHaveBase.size() &&
                                 store.derivedHaveBase[size_t(object)] != 0;
    if (size_t(object) < store.derivedPublish.size()) {
        store.derivedPublish[size_t(object)].haveBase = false;
    }
    if (!haveBase || !derivedHaveBase) {
        return true;
    }
    const RigExecWireRevision &wire =
        geo.chains[chainIndex].derived[derivedIndex].revision;
    if (!RrGeoOpName(wire.op)) {
        if (error) {
            *error = RrGeoStepLabel(program, step) +
                     " references unknown revision op " +
                     std::to_string(wire.op);
        }
        return false;
    }
    if (wire.op == RrGeoOpSurfaceProjector ||
        wire.op == RrGeoOpShaderDials) {
        derived.haveResult = true;
        return RrGeoRunProjectorTarget(program, scratch, chainIndex, wire,
                                       size_t(object), &output);
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
    in.record = record;
    in.chain = chainIndex;
    in.revision = derivedIndex;
    in.derived = true;
    in.diagnostics = &output.diagnostics;
    RrGeoMoverParameters parameters = RrGeoAssembleRevision(in);
    if (program->CrossCheckThisRun() &&
        !RrCrossCheckRevisionReads(program, record, chainIndex, derivedIndex,
                                   true, &output, error)) {
        if (error) {
            *error = RrGeoStepLabel(program, step) + ": " + *error;
        }
        return false;
    }
    const RrGeoMoverStatus status = RrGeoStatusForParameters(
        parameters, program->TextOrEmpty(wire.moverPath));
    output.counters.revisionsBuilt = 1;
    // The chain's points beside the packet, not in it: the run
    // remembers the packet with auxPoints emptied.
    std::vector<RrVec3f> aux;
    aux.swap(parameters.auxPoints);
    const bool moved = chain.result != rev.lastAuxPoints ||
                       parameters != rev.lastParameters;
    parameters.auxPoints.swap(aux);
    if (derived.baseDirty || !rev.ran || moved || status != rev.lastStatus) {
        output.counters.revisionsExecuted = 1;
        std::vector<RrVec3f> values(derived.lastBase.begin(),
                                    derived.lastBase.end());
        const bool applied =
            status.AllowsApply() &&
            RrGeoRunRevisionKernel(int(wire.op), parameters, &values,
                                   &scratch->wireBasis);
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
        rev.lastAuxPoints = chain.result;
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
    chain.result.swap(chain.spare);
    chain.haveResult = true;
    RrSnapshotValue final;
    final.tag = RrSnapshotValue::Tag::Points;
    final.points = chain.result;
    output.snapshots.RecordFinal(geo.chains[size_t(object)].target, final);
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
        RrGeoFoldTransformForms(&rev);
    }
    return true;
}

bool
RrGeoRunRevisionStaticStep(RrProgram *program, RrGeometryScratch *scratch,
                           size_t chainIndex, size_t revisionIndex,
                           size_t step,
                           const RigExecWireFrameInputs *record,
                           std::string *error)
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
            RrGeoStorePacket(program, store.weightPackets[object]);
        const RrVec3f *entering = nullptr;
        size_t enteringCount = 0;
        RrGeoPointsBefore(chain, revisionIndex, &entering, &enteringCount);
        const std::vector<RrVec3f> current(entering,
                                           entering + enteringCount);
        std::vector<float> field;
        std::string why;
        if (RrResolveWeightOracle(program, object, current.size(), &current,
                                  &field, &why)) {
            rev.currentPhasePacket.representation = "dense";
            rev.currentPhasePacket.values = std::move(field);
            rev.currentPhasePacket.indices.clear();
            rev.currentPhasePacket.defaultWeight = 0.0f;
            rev.currentPhasePacket.valid = true;
        } else {
            rev.currentPhasePacket = RrGeoWeightPacket();
            output.diagnostics.push_back("current-phase weight failed: " +
                                         why);
        }
        if (program->CrossCheckThisRun() &&
            !RrGeoCrossCheckPhasePacket(program, record, chainIndex,
                                        revisionIndex, rev.currentPhasePacket,
                                        &output, error)) {
            if (error) {
                *error = RrGeoStepLabel(program, step) + ": " + *error;
            }
            return false;
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
    in.record = record;
    in.chain = chainIndex;
    in.revision = revisionIndex;
    in.derived = false;
    in.diagnostics = &output.diagnostics;
    rev.parameters = RrGeoAssembleRevision(in);
    rev.status = RrGeoStatusForParameters(
        rev.parameters, program->TextOrEmpty(wire.moverPath));
    output.counters.revisionsBuilt = 1;
    rev.defaultWeight = RrGeoAssembleDefaultWeight(in);
    if (program->CrossCheckThisRun() &&
        !RrCrossCheckRevisionReads(program, record, chainIndex, revisionIndex,
                                   false, &output, error)) {
        if (error) {
            *error = RrGeoStepLabel(program, step) + ": " + *error;
        }
        return false;
    }
    rev.staticDirty = !rev.ran || rev.parameters != rev.lastParameters ||
                      rev.status != rev.lastStatus ||
                      rev.defaultWeight != rev.lastDefaultWeight;
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
                      program, store.weightPackets[size_t(wire.weightObject)]);
        if (packet.valid) {
            const size_t logicalCount = wire.weightOperationDomain
                ? size_t(1)
                : chain.lastBase.size();
            if (!packet.ResolveAll(logicalCount, &rev.weightField)) {
                rev.weightField.assign(logicalCount, 0.0f);
                for (size_t i = 0; i < logicalCount; ++i) {
                    const float w = packet.Resolve(i, logicalCount);
                    rev.weightField[i] = w < 0.0f ? 0.0f : w;
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
    if (rev.output.size() != count) {
        rev.output.resize(count);
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
        return true;
    }
    rev.layoutUsable = RrGeoSkinLayoutIsUsable(rev.parameters, count);
    rev.fullStrength = RrGeoEnvelopeIsFullStrength(rev.parameters.weights);
    if (!rev.fullStrength) {
        rev.envelopeOk =
            rev.parameters.weights.ResolveAll(count, &rev.envelope);
    }
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
                       rev.output.size() == count;
    const bool chainDirty =
        revisionIndex == 0
            ? (chainIndex < store.chainBaseDirty.size() &&
               store.chainBaseDirty[chainIndex] != 0)
            : chain.revisions[revisionIndex - 1].executed;

    if (!rev.chunked) {
        chunk.ok = false;
        const bool executed = chainDirty || rev.staticDirty ||
                              (skin && rev.influencesChanged);
        if (!executed || !rev.status.AllowsApply()) {
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
            if (count > 0) {
                rev.output.assign(points, points + count);
            } else {
                rev.output.clear();
            }
            chunk.ok = RrGeoRunRevisionKernel(
                int(wire.op), rev.parameters, &rev.output,
                &scratch->wireBasis);
            return true;
        }
        if (!rev.parameters.valid ||
            rev.parameters.kind != RrGeoKindToken(RrGeoOpSkin) ||
            !rev.layoutUsable || !rev.envelopeOk ||
            !rev.influencesValid || !sized) {
            return true;
        }
        chunk.ok = RrGeoSkinRange(&rev, points,
                                  RrGeoWholeTransformsView(&rev), 0, count,
                                  true);
        return true;
    }

    chunk.keyChanged = false;
    if (!rev.status.AllowsApply() || !rev.parameters.valid ||
        rev.parameters.kind != RrGeoKindToken(RrGeoOpSkin) ||
        !rev.layoutUsable || !rev.envelopeOk || rev.partitionStale ||
        !sized) {
        chunk.ok = false;
        return true;
    }
    if (!RrGeoGatherChunkTransforms(program, wire, rev, &chunk, error)) {
        if (error) {
            *error = RrGeoStepLabel(program, step) + ": " + *error;
        }
        return false;
    }
    if (!(chainDirty || rev.staticDirty || chunk.keyChanged) && chunk.ok) {
        return true;
    }
    if (chunk.begin < 0 || chunk.end < chunk.begin ||
        size_t(chunk.end) > count) {
        if (error) {
            *error =
                RrGeoStepLabel(program, step) + " names no vertex range";
        }
        return false;
    }
    chunk.ok = RrGeoSkinRange(&rev, points, RrGeoChunkTransformsView(chunk),
                              size_t(chunk.begin), size_t(chunk.end),
                              false);
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
    const bool chainDirty =
        revisionIndex == 0
            ? (chainIndex < store.chainBaseDirty.size() &&
               store.chainBaseDirty[chainIndex] != 0)
            : chain.revisions[revisionIndex - 1].executed;
    rev.executed = chainDirty || rev.staticDirty ||
                   (skin && rev.influencesChanged);
    output.counters.revisionsExecuted = rev.executed ? 1 : 0;
    if (rev.executed) {
        rev.resultStatus = rev.status.state;
        bool applied = packetValid && rev.status.AllowsApply();
        if (applied && skin && rev.partitionStale) {
            applied =
                RrGeoFuseWholeRevision(chain, &rev, revisionIndex);
        } else if (applied) {
            for (const RrGeometryScratch::Chunk &chunk : rev.chunks) {
                if (!chunk.ok) {
                    applied = false;
                    break;
                }
            }
        }
        if (applied) {
            rev.currentSource = int(revisionIndex);
        } else {
            rev.currentSource =
                revisionIndex == 0
                    ? -1
                    : chain.revisions[revisionIndex - 1].currentSource;
            if (rev.status.AllowsApply()) {
                rev.resultStatus = "moverFailed";
            }
        }
        rev.lastParameters = rev.parameters;
        rev.lastStatus = rev.status;
        rev.ran = true;
    }
    if (id >= 0 && size_t(id) < store.revisionRan.size()) {
        store.revisionRan[size_t(id)] = rev.ran ? 1 : 0;
    }
    if (id >= 0 && size_t(id) < store.revisionPublish.size()) {
        store.revisionPublish[size_t(id)].resultStatus = rev.resultStatus;
    }
    if (wire.snapshotAfter) {
        const RrVec3f *points = nullptr;
        size_t count = 0;
        RrGeoPointsAfter(chain, revisionIndex, &points, &count);
        RrSnapshotValue value;
        value.tag = RrSnapshotValue::Tag::Points;
        if (count > 0 && points) {
            value.points.assign(points, points + count);
        }
        output.snapshots.Record(wire.target, wire.moverPath, value);
    }
    (void)error;
    return true;
}

}  // namespace

bool
RrRunGeometryStep(RrProgram *program, size_t step, double time,
                  std::string *error)
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
        kind != RigExecWireStepKind::Derived) {
        if (error) {
            *error = RrGeoStepLabel(program, step) +
                     " is not a geometry step";
        }
        return false;
    }
    const RigExecWireFrameInputs *record = scratch->frame;
    if (!record || record->frame != time) {
        record = RrGeoFindRecord(program, time);
    }
    if (kind == RigExecWireStepKind::Derived) {
        return RrGeoRunDerivedStep(program, scratch, step, record, error);
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
                                          revisionIndex, step, record,
                                          error);
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
        if (!rev.chunked) {
            rev.chunks[size_t(wireStep.part)].ok = false;
        }
        return;
    case RigExecWireStepKind::RevisionFuse:
        rev.executed = false;
        return;
    default:
        return;
    }
}

}  // namespace rigExec
