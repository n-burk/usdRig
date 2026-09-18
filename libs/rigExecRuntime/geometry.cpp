//
// rigExecRuntime geometry family (M2): RevisionStatic, InfluenceFold,
// RevisionChunk, RevisionFuse, ChainStatus, Derived.
//
// A zero-USD port of the baked geometry loop (libs/rigExec/bakedGeometry.cpp
// driven by the movers in libs/rigExec/moverGraph.cpp and the kernels in
// libs/rigExecMath). Stage reads replay from the frame record's pathReads;
// epoch state (skin layouts, blend shapes, curvenet binds, partitions)
// replays from the wire geometry tables. Bitwise: same ops in the same
// order, float stays float.

#include "rigExecRuntime/store.h"

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
// ordinal; anything outside [0, 13] is a corrupt program and fails the
// step naming it.
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
    RrGeoOpCurvenet = 10,
    RrGeoOpCurvenetAdjuster = 11,
    RrGeoOpRecomputeNormals = 12,
    RrGeoOpRecomputeExtent = 13,
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
    case RrGeoOpLattice: return "Lattice";
    case RrGeoOpSurfaceProject: return "SurfaceProject";
    case RrGeoOpRibbon: return "Ribbon";
    case RrGeoOpWire: return "Wire";
    case RrGeoOpEmitGuidePoints: return "EmitGuidePoints";
    case RrGeoOpCurvenet: return "Curvenet";
    case RrGeoOpCurvenetAdjuster: return "CurvenetAdjuster";
    case RrGeoOpRecomputeNormals: return "RecomputeNormals";
    case RrGeoOpRecomputeExtent: return "RecomputeExtent";
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
    case RrGeoOpLattice: return "lattice";
    case RrGeoOpSurfaceProject: return "surfaceProject";
    case RrGeoOpRibbon: return "ribbon";
    case RrGeoOpWire: return "wire";
    case RrGeoOpEmitGuidePoints: return "emitGuidePoints";
    case RrGeoOpCurvenet: return "curvenet";
    case RrGeoOpCurvenetAdjuster: return "curvenetAdjuster";
    case RrGeoOpRecomputeNormals: return "recomputeNormals";
    case RrGeoOpRecomputeExtent: return "recomputeExtent";
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

// ---------------------------------------------------------------------------
// Packet mirrors: the per-mover parameter packet and its shared layouts,
// compared exactly the way RigExecMoverParameters::operator== compares
// (layouts and bindings by pointer, everything else by value).
// ---------------------------------------------------------------------------

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

struct RrGeoAdjustmentCommand {
    int pointIndex = -1;
    int parentCommand = -1;
    bool includeTangents = true;
    RrMat4d localTransform;

    RrGeoAdjustmentCommand()
    {
        localTransform.SetIdentity();
    }

    bool operator==(const RrGeoAdjustmentCommand &o) const
    {
        return pointIndex == o.pointIndex &&
               parentCommand == o.parentCommand &&
               includeTangents == o.includeTangents &&
               localTransform == o.localTransform;
    }
    bool operator!=(const RrGeoAdjustmentCommand &o) const
    {
        return !(*this == o);
    }
};

struct RrGeoProfileBinding;

enum RrGeoCurvenetBasis {
    RrGeoCurvenetBasisBezier = 0,
    RrGeoCurvenetBasisCatmullRom = 1,
};

struct RrGeoMoverParameters {
    std::string kind;
    bool enabled = true;
    bool valid = false;
    RrMat4d transform;
    RrGeoWeightPacket weights;
    std::vector<RrVec3f> blendDeltas;
    bool blendSurfaceFrame = false;
    double referenceVolume = 0.0;
    float strength = 0.0f;
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
    std::shared_ptr<const RrGeoProfileBinding> curvenetBinding;
    RrGeoCurvenetBasis curvenetAdjustmentBasis = RrGeoCurvenetBasisBezier;
    std::vector<RrGeoAdjustmentCommand> curvenetAdjustments;

    RrGeoMoverParameters()
    {
        transform.SetIdentity();
    }

    bool operator==(const RrGeoMoverParameters &o) const
    {
        return kind == o.kind && enabled == o.enabled && valid == o.valid &&
               transform == o.transform && weights == o.weights &&
               blendDeltas == o.blendDeltas &&
               blendSurfaceFrame == o.blendSurfaceFrame &&
               referenceVolume == o.referenceVolume &&
               strength == o.strength &&
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
               curvenetBinding == o.curvenetBinding &&
               curvenetAdjustmentBasis == o.curvenetAdjustmentBasis &&
               curvenetAdjustments == o.curvenetAdjustments;
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

// ---------------------------------------------------------------------------
// Envelope (envelope.h, solvers.cpp, moverGraph.cpp): the common blend,
// the weighted-matrix rule, and the full-strength predicate.
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// Skin tables (simdKernels.*, dualQuat.*, solvers.cpp, pointFrame.cpp).
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// Dual-quaternion skinning (dualQuat.*, pointFrame.cpp): the per-matrix
// stretch/rotation split, the weighted palette blend, and the Kabsch SVD
// the non-rigid split fits through.
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// Geometry kernels (geometryKernels.cpp): volume, smooth, normals, extent,
// lattice, surface projection, wire/NURBS. RMF sampling, ribbon transport,
// wire binding and the sparse wire apply are unreachable from the revision
// path and stay out.
// ---------------------------------------------------------------------------

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
    if (!out || rest.size() != posed.size() || rest.size() != deltas.size()) return false;
    for (size_t i = 0; i < rest.size(); ++i) {
        for (int axis = 0; axis < 3; ++axis) {
            if (!std::isfinite(rest[i][axis]) || !std::isfinite(posed[i][axis]) ||
                !std::isfinite(deltas[i][axis])) return false;
        }
    }
    size_t offset = 0;
    std::vector<RrVec3d> restNormals(rest.size(), RrVec3d(0.0));
    std::vector<RrVec3d> posedNormals(rest.size(), RrVec3d(0.0));
    for (int count : faceCounts) {
        if (count < 3 || static_cast<size_t>(count) > faceIndices.size() - offset) return false;
        for (int corner = 0; corner < count; ++corner) {
            const int index = faceIndices[offset + corner];
            if (index < 0 || static_cast<size_t>(index) >= rest.size()) return false;
        }
        const int origin = faceIndices[offset];
        RrVec3d restArea(0.0), posedArea(0.0);
        for (int corner = 1; corner + 1 < count; ++corner) {
            const int a = faceIndices[offset + corner];
            const int b = faceIndices[offset + corner + 1];
            restArea += RrCross(RrGeoToVec3d(rest[a]) - RrGeoToVec3d(rest[origin]),
                                RrGeoToVec3d(rest[b]) - RrGeoToVec3d(rest[origin]));
            posedArea += RrCross(RrGeoToVec3d(posed[a]) - RrGeoToVec3d(posed[origin]),
                                 RrGeoToVec3d(posed[b]) - RrGeoToVec3d(posed[origin]));
        }
        for (int corner = 0; corner < count; ++corner) {
            const int index = faceIndices[offset + corner];
            restNormals[index] += restArea;
            posedNormals[index] += posedArea;
        }
        offset += static_cast<size_t>(count);
    }
    if (offset != faceIndices.size()) return false;
    const auto adjacency = RrGeoBuildAdjacency(rest.size(), faceCounts, faceIndices);
    auto normalize = [](RrVec3d *value) {
        const double length = value->GetLength();
        if (!(length > 0) || !std::isfinite(length)) return false;
        *value /= length;
        return true;
    };
    std::vector<RrVec3f> result = deltas;
    for (size_t i = 0; i < rest.size(); ++i) {
        if (deltas[i] == RrVec3f(0.0f)) continue;
        RrVec3d nr = restNormals[i], np = posedNormals[i];
        if (!normalize(&nr) || !normalize(&np)) return false;
        int neighbor = -1;
        double longest = 0;
        RrVec3d tr(0.0);
        for (int candidate : adjacency[i]) {
            const RrVec3d edge = RrGeoToVec3d(rest[candidate]) - RrGeoToVec3d(rest[i]);
            const RrVec3d projected = edge - RrDot(edge, nr) * nr;
            const double length2 = projected.GetLengthSq();
            if (length2 > longest) {
                neighbor = candidate;
                longest = length2;
                tr = projected;
            }
        }
        if (neighbor < 0 || !normalize(&tr)) return false;
        const RrVec3d edge = RrGeoToVec3d(posed[neighbor]) - RrGeoToVec3d(posed[i]);
        RrVec3d tp = edge - RrDot(edge, np) * np;
        if (tp.GetLengthSq() <= edge.GetLengthSq() * 1e-24 || !normalize(&tp)) return false;
        const RrVec3d br = RrCross(nr, tr), bp = RrCross(np, tp);
        const RrVec3d delta = RrGeoToVec3d(deltas[i]);
        const RrVec3d rotated = RrDot(delta, tr) * tp + RrDot(delta, br) * bp + RrDot(delta, nr) * np;
        result[i] = RrGeoToVec3f(rotated);
        for (int axis = 0; axis < 3; ++axis) {
            if (!std::isfinite(result[i][axis])) return false;
        }
    }
    *out = std::move(result);
    return true;
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

// ---------------------------------------------------------------------------
// Sparse Cholesky (sparseSolve.*): minimum-degree ordering, elimination
// tree, up-looking numeric factorization.
// ---------------------------------------------------------------------------

struct RrGeoSparseBuilder {
    RrGeoSparseBuilder() = default;
    explicit RrGeoSparseBuilder(int size) : _size(size) {}

    void Add(int row, int column, double value)
    {
        if (row < 0 || column < 0 || row >= _size || column >= _size) {
            return;
        }
        if (value == 0.0) {
            return;
        }
        if (row < column) {
            return;
        }
        _rows.push_back(row);
        _columns.push_back(column);
        _values.push_back(value);
    }

    int _size = 0;
    std::vector<int> _rows;
    std::vector<int> _columns;
    std::vector<double> _values;
};

void
RrGeoSparseCompress(int n, const std::vector<int> &rows,
                   const std::vector<int> &columns,
                   const std::vector<double> &values,
                   std::vector<int> *columnStart,
                   std::vector<int> *rowIndex, std::vector<double> *out)
{
    std::vector<int> counts(n + 1, 0);
    for (int c : columns) {
        ++counts[c + 1];
    }
    for (int i = 0; i < n; ++i) {
        counts[i + 1] += counts[i];
    }
    std::vector<int> cursor(counts.begin(), counts.end() - 1);
    std::vector<int> scratchRow(rows.size());
    std::vector<double> scratchValue(rows.size());
    for (size_t k = 0; k < rows.size(); ++k) {
        const int slot = cursor[columns[k]]++;
        scratchRow[slot] = rows[k];
        scratchValue[slot] = values[k];
    }

    columnStart->assign(n + 1, 0);
    rowIndex->clear();
    out->clear();
    rowIndex->reserve(rows.size());
    out->reserve(rows.size());
    std::vector<std::pair<int, double>> column;
    for (int c = 0; c < n; ++c) {
        column.clear();
        for (int k = counts[c]; k < counts[c + 1]; ++k) {
            column.push_back({scratchRow[k], scratchValue[k]});
        }
        std::sort(column.begin(), column.end(),
                  [](const std::pair<int, double> &a,
                     const std::pair<int, double> &b) {
                      return a.first < b.first;
                  });
        for (size_t k = 0; k < column.size();) {
            size_t j = k;
            double sum = 0.0;
            while (j < column.size() && column[j].first == column[k].first) {
                sum += column[j].second;
                ++j;
            }
            rowIndex->push_back(column[k].first);
            out->push_back(sum);
            k = j;
        }
        (*columnStart)[c + 1] = int(rowIndex->size());
    }
}

std::vector<int>
RrGeoEliminationTree(int n, const std::vector<int> &columnStart,
                    const std::vector<int> &rowIndex)
{
    std::vector<int> parent(n, -1);
    std::vector<int> ancestor(n, -1);
    std::vector<std::vector<int>> rowsOf(n);
    for (int j = 0; j < n; ++j) {
        for (int p = columnStart[j]; p < columnStart[j + 1]; ++p) {
            const int i = rowIndex[p];
            if (i > j) {
                rowsOf[i].push_back(j);
            }
        }
    }
    for (int k = 0; k < n; ++k) {
        for (int j : rowsOf[k]) {
            int i = j;
            while (i != -1 && i < k) {
                const int next = ancestor[i];
                ancestor[i] = k;
                if (next == -1) {
                    parent[i] = k;
                    break;
                }
                i = next;
            }
        }
    }
    return parent;
}

struct RrGeoRowView {
    std::vector<int> start;
    std::vector<int> column;
    std::vector<double> value;
};

RrGeoRowView
RrGeoBuildRowView(int n, const std::vector<int> &columnStart,
                 const std::vector<int> &rowIndex,
                 const std::vector<double> &values)
{
    RrGeoRowView view;
    view.start.assign(n + 1, 0);
    for (int j = 0; j < n; ++j) {
        for (int p = columnStart[j]; p < columnStart[j + 1]; ++p) {
            if (rowIndex[p] > j) {
                ++view.start[rowIndex[p] + 1];
            }
        }
    }
    for (int i = 0; i < n; ++i) {
        view.start[i + 1] += view.start[i];
    }
    view.column.assign(view.start[n], 0);
    view.value.assign(view.start[n], 0.0);
    std::vector<int> cursor(view.start.begin(), view.start.end() - 1);
    for (int j = 0; j < n; ++j) {
        for (int p = columnStart[j]; p < columnStart[j + 1]; ++p) {
            const int i = rowIndex[p];
            if (i > j) {
                const int slot = cursor[i]++;
                view.column[slot] = j;
                view.value[slot] = values[p];
            }
        }
    }
    return view;
}

int
RrGeoEReach(int k, const RrGeoRowView &rows,
           const std::vector<int> &parent, std::vector<int> *stack,
           std::vector<char> *marked)
{
    const int n = int(stack->size());
    int top = n;
    (*marked)[k] = 1;
    for (int p = rows.start[k]; p < rows.start[k + 1]; ++p) {
        int i = rows.column[p];
        int length = 0;
        std::vector<int> &s = *stack;
        while (i != -1 && !(*marked)[i]) {
            s[length++] = i;
            (*marked)[i] = 1;
            i = parent[i];
        }
        while (length > 0) {
            s[--top] = s[--length];
        }
    }
    for (int p = top; p < n; ++p) {
        (*marked)[(*stack)[p]] = 0;
    }
    (*marked)[k] = 0;
    return top;
}

std::vector<int>
RrGeoMinimumDegreeOrder(const std::vector<std::vector<int>> &adjacency)
{
    const int n = int(adjacency.size());
    std::vector<std::vector<int>> neighbors(n);
    for (int i = 0; i < n; ++i) {
        neighbors[i] = adjacency[i];
        std::sort(neighbors[i].begin(), neighbors[i].end());
        neighbors[i].erase(
            std::unique(neighbors[i].begin(), neighbors[i].end()),
            neighbors[i].end());
        neighbors[i].erase(
            std::remove(neighbors[i].begin(), neighbors[i].end(), i),
            neighbors[i].end());
    }

    std::vector<char> eliminated(n, 0);
    using Entry = std::pair<int, int>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> queue;
    for (int i = 0; i < n; ++i) {
        queue.push({int(neighbors[i].size()), i});
    }

    std::vector<int> order;
    order.reserve(n);
    std::vector<int> merged;
    while (!queue.empty()) {
        const Entry top = queue.top();
        queue.pop();
        const int v = top.second;
        if (eliminated[v] || top.first != int(neighbors[v].size())) {
            continue;
        }
        eliminated[v] = 1;
        order.push_back(v);

        std::vector<int> live;
        live.reserve(neighbors[v].size());
        for (int u : neighbors[v]) {
            if (!eliminated[u]) {
                live.push_back(u);
            }
        }
        for (int u : live) {
            merged.clear();
            std::set_union(neighbors[u].begin(), neighbors[u].end(),
                           live.begin(), live.end(),
                           std::back_inserter(merged));
            merged.erase(std::remove_if(merged.begin(), merged.end(),
                                        [&](int x) {
                                            return x == u || eliminated[x];
                                        }),
                         merged.end());
            merged.erase(std::unique(merged.begin(), merged.end()),
                         merged.end());
            neighbors[u].swap(merged);
            queue.push({int(neighbors[u].size()), u});
        }
        neighbors[v].clear();
        neighbors[v].shrink_to_fit();
    }
    for (int i = 0; i < n; ++i) {
        if (!eliminated[i]) {
            order.push_back(i);
        }
    }
    return order;
}

struct RrGeoSparseCholesky {
    bool Factorize(const RrGeoSparseBuilder &matrix, double regularization,
                   std::string *error)
    {
        _factorized = false;
        _size = matrix._size;
        const int n = _size;
        if (n <= 0) {
            _columnStart.assign(1, 0);
            _rowIndex.clear();
            _values.clear();
            _diagonal.clear();
            _permutation.clear();
            _inversePermutation.clear();
            _factorized = true;
            return true;
        }

        std::vector<int> inputStart, inputRow;
        std::vector<double> inputValue;
        RrGeoSparseCompress(n, matrix._rows, matrix._columns,
                            matrix._values, &inputStart, &inputRow,
                            &inputValue);

        std::vector<std::vector<int>> adjacency(n);
        for (int j = 0; j < n; ++j) {
            for (int p = inputStart[j]; p < inputStart[j + 1]; ++p) {
                const int i = inputRow[p];
                if (i != j) {
                    adjacency[i].push_back(j);
                    adjacency[j].push_back(i);
                }
            }
        }
        _permutation = RrGeoMinimumDegreeOrder(adjacency);
        _inversePermutation.assign(n, 0);
        for (int k = 0; k < n; ++k) {
            _inversePermutation[_permutation[k]] = k;
        }

        RrGeoSparseBuilder permuted(n);
        for (int j = 0; j < n; ++j) {
            for (int p = inputStart[j]; p < inputStart[j + 1]; ++p) {
                const int i = inputRow[p];
                double value = inputValue[p];
                if (i == j) {
                    value += regularization;
                }
                int row = _inversePermutation[i];
                int column = _inversePermutation[j];
                if (row < column) {
                    std::swap(row, column);
                }
                permuted.Add(row, column, value);
            }
        }
        std::vector<int> start, row;
        std::vector<double> value;
        RrGeoSparseCompress(n, permuted._rows, permuted._columns,
                            permuted._values, &start, &row, &value);

        _parent = RrGeoEliminationTree(n, start, row);
        const RrGeoRowView rows = RrGeoBuildRowView(n, start, row, value);
        std::vector<double> inputDiagonal(n, 0.0);
        for (int j = 0; j < n; ++j) {
            for (int p = start[j]; p < start[j + 1]; ++p) {
                if (row[p] == j) {
                    inputDiagonal[j] += value[p];
                }
            }
        }

        _columnStart.assign(n + 1, 0);
        _rowIndex.clear();
        _values.clear();
        _diagonal.assign(n, 0.0);

        std::vector<std::vector<std::pair<int, double>>> columns(n);
        std::vector<double> work(n, 0.0);
        std::vector<int> stack(n, 0);
        std::vector<char> marked(n, 0);
        std::vector<std::vector<std::pair<int, double>>> &lower = columns;

        for (int k = 0; k < n; ++k) {
            double diagonalValue = inputDiagonal[k];
            for (int p = rows.start[k]; p < rows.start[k + 1]; ++p) {
                work[rows.column[p]] += rows.value[p];
            }

            const int top = RrGeoEReach(k, rows, _parent, &stack, &marked);

            for (int s = top; s < n; ++s) {
                const int j = stack[s];
                const double y = work[j] / _diagonal[j];
                work[j] = 0.0;
                for (const auto &entry : lower[j]) {
                    if (entry.first > j) {
                        work[entry.first] -= entry.second * y;
                    }
                }
                columns[j].push_back({k, y});
                diagonalValue -= y * y;
            }

            if (!(diagonalValue > 0.0) || !std::isfinite(diagonalValue)) {
                if (error) {
                    char buffer[224];
                    std::snprintf(
                        buffer, sizeof(buffer),
                        "matrix is not positive definite: pivot %d (original index "
                        "%d) is %g. For the Profile Mover this means a set of mesh "
                        "vertices no curvenet constrains.",
                        k, _permutation[k], diagonalValue);
                    *error = buffer;
                }
                return false;
            }
            _diagonal[k] = std::sqrt(diagonalValue);
        }

        _columnStart[0] = 0;
        for (int j = 0; j < n; ++j) {
            std::sort(columns[j].begin(), columns[j].end());
            for (const auto &entry : columns[j]) {
                _rowIndex.push_back(entry.first);
                _values.push_back(entry.second);
            }
            _columnStart[j + 1] = int(_rowIndex.size());
        }

        _factorized = true;
        return true;
    }

    void Solve(const std::vector<double> &rhs, int columns,
               std::vector<double> *out) const
    {
        const int n = _size;
        out->assign(size_t(n) * size_t(std::max(columns, 0)), 0.0);
        if (!_factorized || n == 0 || columns <= 0) {
            return;
        }
        std::vector<double> x(n, 0.0);
        for (int c = 0; c < columns; ++c) {
            const double *b = rhs.data() + size_t(c) * size_t(n);
            for (int k = 0; k < n; ++k) {
                x[k] = b[_permutation[k]];
            }
            for (int j = 0; j < n; ++j) {
                x[j] /= _diagonal[j];
                const double y = x[j];
                for (int p = _columnStart[j]; p < _columnStart[j + 1]; ++p) {
                    x[_rowIndex[p]] -= _values[p] * y;
                }
            }
            for (int j = n - 1; j >= 0; --j) {
                double sum = x[j];
                for (int p = _columnStart[j]; p < _columnStart[j + 1]; ++p) {
                    sum -= _values[p] * x[_rowIndex[p]];
                }
                x[j] = sum / _diagonal[j];
            }
            double *result = out->data() + size_t(c) * size_t(n);
            for (int k = 0; k < n; ++k) {
                result[_permutation[k]] = x[k];
            }
        }
    }

    bool IsFactored() const { return _factorized; }

    size_t GetFactorNonzeros() const
    {
        // Off-diagonals only, like the source: the diagonal lives in
        // _diagonal on both sides and the source's tally never added it
        // (its "Nonzeros in L" comment overclaims; the bind line's number
        // is the 480-entry behavior, not the comment).
        return _values.size();
    }

    int _size = 0;
    std::vector<int> _columnStart{0};
    std::vector<int> _rowIndex;
    std::vector<double> _values;
    std::vector<double> _diagonal;
    std::vector<int> _permutation;
    std::vector<int> _inversePermutation;
    std::vector<int> _parent;
    bool _factorized = false;
};

// ---------------------------------------------------------------------------
// Curvenets (curvenet.*): topology, sampling, scaled frames, gradients.
// Matrices here are MATH (column-vector) convention, as in the source.
// ---------------------------------------------------------------------------

constexpr double RrGeoCurvenetEps = 1e-12;

enum RrGeoCurvenetKnotKind {
    RrGeoKnotUnused = 0,
    RrGeoKnotHandle = 1,
    RrGeoKnotAnchor = 2,
    RrGeoKnotInterior = 3,
    RrGeoKnotIntersection = 4,
};

struct RrGeoCurvenetCurve {
    std::vector<int> splines;
    std::vector<bool> reversed;
    int startKnot = -1;
    int endKnot = -1;
    bool closed = false;
    bool startIsIntersection = false;
    bool endIsIntersection = false;

    bool IsIsolated() const
    {
        return !startIsIntersection && !endIsIntersection;
    }
};

struct RrGeoCurvenetSpoke {
    int curve = -1;
    bool atCurveEnd = false;
};

struct RrGeoCurvenetIntersection {
    int knot = -1;
    std::vector<RrGeoCurvenetSpoke> spokes;
    RrVec3d referenceNormal{0.0, 0.0, 1.0};
};

struct RrGeoCurvenetTopology {
    RrGeoCurvenetBasis basis = RrGeoCurvenetBasisBezier;
    int pointCount = 0;
    std::vector<int> splineIndices;
    std::vector<RrGeoCurvenetKnotKind> knotKinds;
    std::vector<int> knotValence;
    std::vector<RrGeoCurvenetCurve> curves;
    std::vector<RrGeoCurvenetIntersection> intersections;
    std::vector<int> intersectionOfKnot;

    size_t GetSplineCount() const { return splineIndices.size() / 4; }

    int GetSplineStartKnot(size_t s) const
    {
        return splineIndices[4 * s +
                             (basis == RrGeoCurvenetBasisBezier ? 0 : 1)];
    }
    int GetSplineEndKnot(size_t s) const
    {
        return splineIndices[4 * s +
                             (basis == RrGeoCurvenetBasisBezier ? 3 : 2)];
    }
};

struct RrGeoCurvenetSampling {
    std::vector<RrVec3d> positions;
    std::vector<int> curveBegin;
    std::vector<int> knotOfSample;
    std::vector<std::array<int, 4>> stencilIndices;
    std::vector<std::array<double, 4>> stencilWeights;

    size_t GetSampleCount() const { return positions.size(); }
    size_t GetCurveCount() const
    {
        return curveBegin.empty() ? 0 : curveBegin.size() - 1;
    }
    int GetCurveSampleCount(size_t c) const
    {
        return curveBegin[c + 1] - curveBegin[c];
    }
};

struct RrGeoCurvenetFrames {
    std::vector<int> segmentBegin;
    std::vector<RrVec3d> tangent;
    std::vector<double> length;
    std::vector<RrVec3d> normalLeft;
    std::vector<RrVec3d> normalRight;
    std::vector<double> widthLeft;
    std::vector<double> widthRight;
    std::vector<bool> curveFramed;

    size_t GetSegmentCount() const { return tangent.size(); }
    size_t GetCurveCount() const
    {
        return segmentBegin.empty() ? 0 : segmentBegin.size() - 1;
    }

    RrMat3d GetScaledFrame(size_t segment, bool left) const
    {
        const RrVec3d &t = tangent[segment];
        const RrVec3d &n =
            left ? normalLeft[segment] : normalRight[segment];
        const double l = length[segment];
        const double w = left ? widthLeft[segment] : widthRight[segment];
        const double h = std::sqrt(std::max(l * w, 0.0));
        const RrVec3d b = RrCross(n, t);
        RrMat3d m;
        for (int i = 0; i < 3; ++i) {
            m[i][0] = t[i] * l;
            m[i][1] = b[i] * w;
            m[i][2] = n[i] * h;
        }
        return m;
    }
};

struct RrGeoCurvenetGradients {
    std::vector<RrMat3d> left;
    std::vector<RrMat3d> right;
};

struct RrGeoCurvenetSampleGradients {
    std::vector<RrMat3d> left;
    std::vector<RrMat3d> right;
};

double
RrGeoCurvenetSafeNormalize(RrVec3d *v)
{
    const double n = v->GetLength();
    if (n <= RrGeoCurvenetEps) {
        return 0.0;
    }
    *v /= n;
    return n;
}

RrVec3d
RrGeoCurvenetAnyPerpendicular(const RrVec3d &v)
{
    const RrVec3d axis = (std::abs(v[0]) < 0.9) ? RrVec3d(1.0, 0.0, 0.0)
                                                : RrVec3d(0.0, 1.0, 0.0);
    RrVec3d p = RrCross(v, axis);
    if (RrGeoCurvenetSafeNormalize(&p) == 0.0) {
        return RrVec3d(0.0, 0.0, 1.0);
    }
    return p;
}

bool
RrGeoCurvenetOrthogonalize(RrVec3d *n, const RrVec3d &t)
{
    *n -= t * RrDot(*n, t);
    return RrGeoCurvenetSafeNormalize(n) > 0.0;
}

RrMat3d
RrGeoCurvenetOuter(const RrVec3d &a, const RrVec3d &b)
{
    RrMat3d m;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            m[i][j] = a[i] * b[j];
        }
    }
    return m;
}

RrVec3d
RrGeoCurvenetSmallestEigenvector(const RrMat3d &m)
{
    const double trace = m[0][0] + m[1][1] + m[2][2];
    if (trace <= RrGeoCurvenetEps) {
        return RrVec3d(0.0);
    }
    RrMat3d shifted = RrMat3d(1.0) * trace - m;
    RrVec3d v(0.31622776601683794, 0.5477225575051661, 0.7745966692414834);
    for (int i = 0; i < 48; ++i) {
        RrVec3d next = shifted * v;
        if (RrGeoCurvenetSafeNormalize(&next) == 0.0) {
            break;
        }
        v = next;
    }
    return v;
}

RrMat3d
RrGeoCurvenetAxisAngle(const RrVec3d &axis, double angle)
{
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    RrMat3d k(0.0, -axis[2], axis[1],
                 axis[2], 0.0, -axis[0],
                 -axis[1], axis[0], 0.0);
    RrMat3d out(1.0);
    out = out + k * s + (k * k) * (1.0 - c);
    return out;
}

RrVec3d
RrGeoEvalBezier(const RrVec3d p[4], double t)
{
    const double u = 1.0 - t;
    return p[0] * (u * u * u) + p[1] * (3.0 * u * u * t) +
           p[2] * (3.0 * u * t * t) + p[3] * (t * t * t);
}

RrVec3d
RrGeoEvalCatmullRom(const RrVec3d p[4], double t)
{
    auto knot = [](double ti, const RrVec3d &a, const RrVec3d &b) {
        const double d = (b - a).GetLength();
        return ti + std::pow(std::max(d, RrGeoCurvenetEps), 0.5);
    };
    const double t0 = 0.0;
    const double t1 = knot(t0, p[0], p[1]);
    const double t2 = knot(t1, p[1], p[2]);
    const double t3 = knot(t2, p[2], p[3]);
    if (t2 - t1 <= RrGeoCurvenetEps) {
        return p[1];
    }
    const double tt = t1 + t * (t2 - t1);

    auto lerpKnot = [&](const RrVec3d &a, const RrVec3d &b, double ta,
                        double tb) {
        if (tb - ta <= RrGeoCurvenetEps) {
            return a;
        }
        const double w = (tb - tt) / (tb - ta);
        return a * w + b * (1.0 - w);
    };
    const RrVec3d a1 = lerpKnot(p[0], p[1], t0, t1);
    const RrVec3d a2 = lerpKnot(p[1], p[2], t1, t2);
    const RrVec3d a3 = lerpKnot(p[2], p[3], t2, t3);
    const RrVec3d b1 = lerpKnot(a1, a2, t0, t2);
    const RrVec3d b2 = lerpKnot(a2, a3, t1, t3);
    return lerpKnot(b1, b2, t1, t2);
}

RrVec3d
RrGeoEvalSpline(RrGeoCurvenetBasis basis, const RrVec3d p[4], double t)
{
    return (basis == RrGeoCurvenetBasisBezier) ? RrGeoEvalBezier(p, t)
                                               : RrGeoEvalCatmullRom(p, t);
}

void
RrGeoGatherSpline(const RrGeoCurvenetTopology &topology,
                 const std::vector<RrVec3f> &points, size_t spline,
                 bool reversed, RrVec3d out[4])
{
    for (int i = 0; i < 4; ++i) {
        const int src = reversed ? 3 - i : i;
        out[i] = RrGeoToVec3d(points[topology.splineIndices[4 * spline + src]]);
    }
}

RrVec4d
RrGeoSplineWeights(RrGeoCurvenetBasis basis, const RrVec3d p[4], double t)
{
    if (basis == RrGeoCurvenetBasisBezier) {
        const double u = 1.0 - t;
        return RrVec4d(u*u*u, 3*u*u*t, 3*u*t*t, t*t*t);
    }
    double knots[4] = {0, 0, 0, 0};
    for (int i = 1; i < 4; ++i)
        knots[i] = knots[i-1] + std::sqrt(std::max((p[i]-p[i-1]).GetLength(), RrGeoCurvenetEps));
    const double at = knots[1] + t * (knots[2] - knots[1]);
    auto mix = [&](const RrVec4d &a, const RrVec4d &b, int lo, int hi) {
        const double w = (at-knots[lo])/(knots[hi]-knots[lo]);
        return a*(1-w) + b*w;
    };
    const RrVec4d a = mix(RrVec4d(1,0,0,0), RrVec4d(0,1,0,0), 0,1);
    const RrVec4d b = mix(RrVec4d(0,1,0,0), RrVec4d(0,0,1,0), 1,2);
    const RrVec4d c = mix(RrVec4d(0,0,1,0), RrVec4d(0,0,0,1), 2,3);
    return mix(mix(a,b,0,2), mix(b,c,1,3), 1,2);
}

void
RrGeoSampleSplineSpan(RrGeoCurvenetBasis basis, const RrVec3d p[4],
                     int subdivisions, std::vector<RrVec3d> *out,
                     std::vector<RrVec4d> *stencils)
{
    const int refine = std::max(8 * subdivisions, 32);
    std::vector<RrVec3d> dense(refine + 1);
    std::vector<RrVec4d> coefficients(refine + 1);
    std::vector<double> arc(refine + 1, 0.0);
    for (int i = 0; i <= refine; ++i) {
        dense[i] = RrGeoEvalSpline(basis, p, double(i) / double(refine));
        coefficients[i] = RrGeoSplineWeights(basis, p, double(i) / double(refine));
        if (i > 0) {
            arc[i] = arc[i - 1] + (dense[i] - dense[i - 1]).GetLength();
        }
    }
    const double total = arc[refine];
    out->clear();
    stencils->clear();
    out->reserve(subdivisions + 1);
    if (total <= RrGeoCurvenetEps) {
        for (int j = 0; j <= subdivisions; ++j) {
            out->push_back(dense[0]);
            stencils->push_back(coefficients[0]);
        }
        return;
    }
    int cursor = 0;
    for (int j = 0; j <= subdivisions; ++j) {
        const double target = total * double(j) / double(subdivisions);
        while (cursor + 1 < refine && arc[cursor + 1] < target) {
            ++cursor;
        }
        const double span = arc[cursor + 1] - arc[cursor];
        const double w = (span <= RrGeoCurvenetEps) ? 0.0 : (target - arc[cursor]) / span;
        out->push_back(dense[cursor] * (1.0 - w) + dense[cursor + 1] * w);
        stencils->push_back(coefficients[cursor]*(1-w) + coefficients[cursor+1]*w);
    }
}

RrMat3d
RrGeoSmallestRotation(const RrVec3d &from, const RrVec3d &to)
{
    RrVec3d a = from, b = to;
    if (RrGeoCurvenetSafeNormalize(&a) == 0.0 ||
        RrGeoCurvenetSafeNormalize(&b) == 0.0) {
        return RrMat3d(1.0);
    }
    const double c = RrClamp(RrDot(a, b), -1.0, 1.0);
    if (c > 1.0 - 1e-15) {
        return RrMat3d(1.0);
    }
    if (c < -1.0 + 1e-12) {
        return RrGeoCurvenetAxisAngle(
            RrGeoCurvenetAnyPerpendicular(a), std::acos(-1));
    }
    RrVec3d axis = RrCross(a, b);
    RrGeoCurvenetSafeNormalize(&axis);
    return RrGeoCurvenetAxisAngle(axis, std::acos(c));
}

bool
RrGeoBuildCurvenetTopology(
    const std::vector<int> &splineIndices, size_t pointCount,
    RrGeoCurvenetBasis basis, const std::vector<RrVec3f> &neutralPoints,
    const std::function<RrVec3d(const RrVec3d &)> *normalAt,
    RrGeoCurvenetTopology *topology, std::string *error);

void
RrGeoOrientCurvenetIntersections(
    const std::vector<RrVec3f> &neutralPoints,
    const std::function<RrVec3d(const RrVec3d &)> &normalAt,
    RrGeoCurvenetTopology *topology)
{
    for (RrGeoCurvenetIntersection &intersection : topology->intersections) {
        const RrVec3d origin = RrGeoToVec3d(neutralPoints[intersection.knot]);
        std::vector<RrVec3d> directions;
        directions.reserve(intersection.spokes.size());
        for (const RrGeoCurvenetSpoke &spoke : intersection.spokes) {
            const RrGeoCurvenetCurve &curve = topology->curves[spoke.curve];
            const size_t which =
                spoke.atCurveEnd ? curve.splines.size() - 1 : 0;
            const bool reversed = spoke.atCurveEnd
                                      ? !curve.reversed[which]
                                      : bool(curve.reversed[which]);
            RrVec3d cp[4];
            RrGeoGatherSpline(*topology, neutralPoints, curve.splines[which],
                              reversed, cp);
            RrVec3d dir(0.0);
            for (int i = 1; i < 4; ++i) {
                dir = cp[i] - cp[0];
                if (dir.GetLength() > RrGeoCurvenetEps) {
                    break;
                }
            }
            RrGeoCurvenetSafeNormalize(&dir);
            directions.push_back(dir);
        }

        RrVec3d normal(0.0);
        if (normalAt) {
            normal = normalAt(origin);
        }
        if (RrGeoCurvenetSafeNormalize(&normal) == 0.0) {
            RrMat3d covariance(0.0);
            for (const RrVec3d &d : directions) {
                covariance += RrGeoCurvenetOuter(d, d);
            }
            normal = RrGeoCurvenetSmallestEigenvector(covariance);
            if (RrGeoCurvenetSafeNormalize(&normal) == 0.0) {
                normal = RrGeoCurvenetAnyPerpendicular(
                    directions.empty() ? RrVec3d(0, 0, 1) : directions[0]);
            }
        }
        intersection.referenceNormal = normal;

        RrVec3d basisX = directions.empty() ? RrGeoCurvenetAnyPerpendicular(normal)
                                            : directions[0];
        if (!RrGeoCurvenetOrthogonalize(&basisX, normal)) {
            basisX = RrGeoCurvenetAnyPerpendicular(normal);
        }
        const RrVec3d basisY = RrCross(normal, basisX);

        std::vector<size_t> order(intersection.spokes.size());
        for (size_t i = 0; i < order.size(); ++i) {
            order[i] = i;
        }
        std::vector<double> angle(order.size(), 0.0);
        for (size_t i = 0; i < order.size(); ++i) {
            const RrVec3d &d = directions[i];
            angle[i] = std::atan2(RrDot(d, basisY), RrDot(d, basisX));
        }
        std::stable_sort(order.begin(), order.end(),
                         [&](size_t a, size_t b) {
                             return angle[a] < angle[b];
                         });
        std::vector<RrGeoCurvenetSpoke> sorted;
        sorted.reserve(order.size());
        for (size_t i : order) {
            sorted.push_back(intersection.spokes[i]);
        }
        intersection.spokes = std::move(sorted);
    }
}

bool
RrGeoBuildCurvenetTopology(
    const std::vector<int> &splineIndices, size_t pointCount,
    RrGeoCurvenetBasis basis, const std::vector<RrVec3f> &neutralPoints,
    const std::function<RrVec3d(const RrVec3d &)> *normalAt,
    RrGeoCurvenetTopology *topology, std::string *error)
{
    auto fail = [&](const std::string &message) {
        if (error) {
            *error = message;
        }
        return false;
    };

    if (splineIndices.size() % 4 != 0) {
        return fail("rigExec:splineIndices length " +
                    std::to_string(splineIndices.size()) +
                    " is not a multiple of four (one cubic spline is four "
                    "control point indices)");
    }
    for (size_t i = 0; i < splineIndices.size(); ++i) {
        if (splineIndices[i] < 0 ||
            size_t(splineIndices[i]) >= pointCount) {
            return fail("rigExec:splineIndices[" + std::to_string(i) +
                        "] = " + std::to_string(splineIndices[i]) +
                        " is out of range for " + std::to_string(pointCount) +
                        " control points");
        }
    }
    if (neutralPoints.size() != pointCount) {
        return fail("neutral point count " +
                    std::to_string(neutralPoints.size()) +
                    " disagrees with the declared pool size " +
                    std::to_string(pointCount));
    }

    *topology = RrGeoCurvenetTopology();
    topology->basis = basis;
    topology->pointCount = int(pointCount);
    topology->splineIndices = splineIndices;

    const size_t splineCount = splineIndices.size() / 4;

    std::vector<std::vector<std::pair<int, bool>>> incident(pointCount);
    std::vector<int> valence(pointCount, 0);
    std::vector<bool> isHandle(pointCount, false);
    for (size_t s = 0; s < splineCount; ++s) {
        const int a = topology->GetSplineStartKnot(s);
        const int b = topology->GetSplineEndKnot(s);
        incident[a].push_back({int(s), true});
        incident[b].push_back({int(s), false});
        ++valence[a];
        ++valence[b];
        isHandle[splineIndices[4 * s + (basis == RrGeoCurvenetBasisBezier ? 1 : 0)]] = true;
        isHandle[splineIndices[4 * s + (basis == RrGeoCurvenetBasisBezier ? 2 : 3)]] = true;
    }

    topology->knotValence.assign(pointCount, 0);
    topology->knotKinds.assign(pointCount, RrGeoKnotUnused);
    for (size_t k = 0; k < pointCount; ++k) {
        topology->knotValence[k] = valence[k];
        if (valence[k] >= 3) {
            topology->knotKinds[k] = RrGeoKnotIntersection;
        } else if (valence[k] == 2) {
            topology->knotKinds[k] = RrGeoKnotInterior;
        } else if (valence[k] == 1) {
            topology->knotKinds[k] = RrGeoKnotAnchor;
        } else if (isHandle[k]) {
            topology->knotKinds[k] = RrGeoKnotHandle;
        }
    }

    auto isLabelled = [&](int knot) {
        return topology->knotKinds[knot] == RrGeoKnotIntersection ||
               topology->knotKinds[knot] == RrGeoKnotAnchor;
    };
    auto otherEnd = [&](int spline, int knot) {
        const int a = topology->GetSplineStartKnot(spline);
        const int b = topology->GetSplineEndKnot(spline);
        return (knot == a) ? b : a;
    };

    std::vector<bool> consumed(splineCount, false);
    for (size_t k = 0; k < pointCount; ++k) {
        if (!isLabelled(int(k))) {
            continue;
        }
        for (const auto &start : incident[k]) {
            if (consumed[start.first]) {
                continue;
            }
            RrGeoCurvenetCurve curve;
            curve.startKnot = int(k);
            curve.startIsIntersection =
                topology->knotKinds[k] == RrGeoKnotIntersection;

            int currentKnot = int(k);
            int currentSpline = start.first;
            for (;;) {
                consumed[currentSpline] = true;
                const bool reversed =
                    topology->GetSplineStartKnot(currentSpline) != currentKnot;
                curve.splines.push_back(currentSpline);
                curve.reversed.push_back(reversed);
                const int next = otherEnd(currentSpline, currentKnot);
                currentKnot = next;
                if (isLabelled(next)) {
                    break;
                }
                int following = -1;
                for (const auto &inc : incident[next]) {
                    if (inc.first != currentSpline && !consumed[inc.first]) {
                        following = inc.first;
                        break;
                    }
                }
                if (following < 0) {
                    break;  // closed back on itself
                }
                currentSpline = following;
            }
            curve.endKnot = currentKnot;
            curve.endIsIntersection =
                topology->knotKinds[currentKnot] ==
                RrGeoKnotIntersection;
            topology->curves.push_back(std::move(curve));
        }
    }

    for (size_t s = 0; s < splineCount; ++s) {
        if (consumed[s]) {
            continue;
        }
        RrGeoCurvenetCurve curve;
        curve.closed = true;
        const int origin = topology->GetSplineStartKnot(s);
        curve.startKnot = origin;
        int currentKnot = origin;
        int currentSpline = int(s);
        for (;;) {
            consumed[currentSpline] = true;
            const bool reversed =
                topology->GetSplineStartKnot(currentSpline) != currentKnot;
            curve.splines.push_back(currentSpline);
            curve.reversed.push_back(reversed);
            const int next = otherEnd(currentSpline, currentKnot);
            currentKnot = next;
            if (next == origin) {
                break;
            }
            int following = -1;
            for (const auto &inc : incident[next]) {
                if (inc.first != currentSpline && !consumed[inc.first]) {
                    following = inc.first;
                    break;
                }
            }
            if (following < 0) {
                break;
            }
            currentSpline = following;
        }
        curve.endKnot = currentKnot;
        topology->curves.push_back(std::move(curve));
    }

    topology->intersectionOfKnot.assign(pointCount, -1);
    for (size_t c = 0; c < topology->curves.size(); ++c) {
        const RrGeoCurvenetCurve &curve = topology->curves[c];
        for (int end = 0; end < 2; ++end) {
            const bool atEnd = (end == 1);
            const bool isIntersection =
                atEnd ? curve.endIsIntersection : curve.startIsIntersection;
            if (!isIntersection) {
                continue;
            }
            const int knot = atEnd ? curve.endKnot : curve.startKnot;
            int slot = topology->intersectionOfKnot[knot];
            if (slot < 0) {
                slot = int(topology->intersections.size());
                topology->intersectionOfKnot[knot] = slot;
                RrGeoCurvenetIntersection created;
                created.knot = knot;
                topology->intersections.push_back(created);
            }
            RrGeoCurvenetSpoke spoke;
            spoke.curve = int(c);
            spoke.atCurveEnd = atEnd;
            topology->intersections[slot].spokes.push_back(spoke);
        }
    }

    RrGeoOrientCurvenetIntersections(
        neutralPoints,
        normalAt ? *normalAt : std::function<RrVec3d(const RrVec3d &)>(),
        topology);
    return true;
}

std::vector<int>
RrGeoPlanCurvenetSamples(const RrGeoCurvenetTopology &topology,
                        const std::vector<RrVec3f> &neutralPoints,
                        double meshMeanEdgeLength, int samplesPerSpline)
{
    const size_t splineCount = topology.GetSplineCount();
    std::vector<int> counts(splineCount, 1);
    const double perSpline = std::max(1, samplesPerSpline);
    const double edge = (meshMeanEdgeLength > RrGeoCurvenetEps) ? meshMeanEdgeLength : 0.0;
    for (size_t s = 0; s < splineCount; ++s) {
        double polygon = 0.0;
        for (int i = 0; i < 3; ++i) {
            polygon += (RrGeoToVec3d(neutralPoints[topology.splineIndices[4 * s + i + 1]]) -
                        RrGeoToVec3d(neutralPoints[topology.splineIndices[4 * s + i]]))
                           .GetLength();
        }
        const double ratio = (edge > 0.0) ? polygon / edge : 1.0;
        const long n = std::lround(perSpline * ratio);
        counts[s] = int(std::max<long>(1, std::min<long>(n, 4096)));
    }
    return counts;
}

RrGeoCurvenetSampling
RrGeoSampleCurvenet(const RrGeoCurvenetTopology &topology,
                   const std::vector<RrVec3f> &points,
                   const std::vector<int> &samplesPerSpline)
{
    RrGeoCurvenetSampling sampling;
    sampling.curveBegin.reserve(topology.curves.size() + 1);
    sampling.curveBegin.push_back(0);

    std::vector<RrVec3d> span;
    std::vector<RrVec4d> stencils;
    for (const RrGeoCurvenetCurve &curve : topology.curves) {
        for (size_t i = 0; i < curve.splines.size(); ++i) {
            const size_t spline = curve.splines[i];
            RrVec3d cp[4];
            RrGeoGatherSpline(topology, points, spline, curve.reversed[i], cp);
            const int subdivisions =
                (spline < samplesPerSpline.size())
                    ? std::max(1, samplesPerSpline[spline])
                    : 1;
            RrGeoSampleSplineSpan(topology.basis, cp, subdivisions, &span, &stencils);

            const size_t first = (i == 0) ? 0 : 1;
            const int startKnot =
                curve.reversed[i] ? topology.GetSplineEndKnot(spline)
                                  : topology.GetSplineStartKnot(spline);
            const int endKnot =
                curve.reversed[i] ? topology.GetSplineStartKnot(spline)
                                  : topology.GetSplineEndKnot(spline);
            for (size_t j = first; j < span.size(); ++j) {
                sampling.positions.push_back(span[j]);
                std::array<int,4> indices;
                std::array<double,4> weights;
                for (int k = 0; k < 4; ++k) {
                    indices[k] = topology.splineIndices[4*spline + (curve.reversed[i] ? 3-k : k)];
                    weights[k] = stencils[j][k];
                }
                sampling.stencilIndices.push_back(indices);
                sampling.stencilWeights.push_back(weights);
                int knot = -1;
                if (j == 0) {
                    knot = startKnot;
                } else if (j + 1 == span.size()) {
                    knot = endKnot;
                }
                sampling.knotOfSample.push_back(knot);
            }
        }
        if (curve.closed && sampling.positions.size() >
                                size_t(sampling.curveBegin.back())) {
            sampling.positions.pop_back();
            sampling.knotOfSample.pop_back();
            sampling.stencilIndices.pop_back();
            sampling.stencilWeights.pop_back();
        }
        sampling.curveBegin.push_back(int(sampling.positions.size()));
    }
    return sampling;
}

RrGeoCurvenetFrames
RrGeoComputeCurvenetFrames(const RrGeoCurvenetTopology &topology,
                          const RrGeoCurvenetSampling &sampling)
{
    RrGeoCurvenetFrames frames;
    const size_t curveCount = topology.curves.size();
    frames.segmentBegin.assign(curveCount + 1, 0);
    frames.curveFramed.assign(curveCount, false);

    for (size_t c = 0; c < curveCount; ++c) {
        const int n = sampling.GetCurveSampleCount(c);
        const bool closed = topology.curves[c].closed;
        const int segments = closed ? std::max(n, 0) : std::max(n - 1, 0);
        frames.segmentBegin[c + 1] = frames.segmentBegin[c] + segments;
    }
    const size_t segmentCount = size_t(frames.segmentBegin.back());
    frames.tangent.assign(segmentCount, RrVec3d(0.0));
    frames.length.assign(segmentCount, 0.0);
    frames.normalLeft.assign(segmentCount, RrVec3d(0.0));
    frames.normalRight.assign(segmentCount, RrVec3d(0.0));
    frames.widthLeft.assign(segmentCount, 0.0);
    frames.widthRight.assign(segmentCount, 0.0);

    for (size_t c = 0; c < curveCount; ++c) {
        const int base = sampling.curveBegin[c];
        const int n = sampling.GetCurveSampleCount(c);
        const bool closed = topology.curves[c].closed;
        const int segments = frames.segmentBegin[c + 1] - frames.segmentBegin[c];
        for (int i = 0; i < segments; ++i) {
            const RrVec3d &a = sampling.positions[base + i];
            const RrVec3d &b = sampling.positions[base + ((i + 1) % n)];
            RrVec3d d = b - a;
            const double len = d.GetLength();
            frames.length[frames.segmentBegin[c] + i] = len;
            if (len > RrGeoCurvenetEps) {
                d /= len;
                frames.tangent[frames.segmentBegin[c] + i] = d;
            }
        }
    }

    struct SpokeFrame {
        RrVec3d normalLeft{0.0, 0.0, 0.0};
        RrVec3d normalRight{0.0, 0.0, 0.0};
        double widthLeft = 0.0;
        double widthRight = 0.0;
        bool valid = false;
    };
    std::vector<std::vector<SpokeFrame>> spokeFrames(
        topology.intersections.size());

    for (size_t x = 0; x < topology.intersections.size(); ++x) {
        const RrGeoCurvenetIntersection &intersection =
            topology.intersections[x];
        const size_t k = intersection.spokes.size();
        spokeFrames[x].assign(k, SpokeFrame());
        if (k == 0) {
            continue;
        }

        std::vector<RrVec3d> t(k);
        std::vector<double> l(k, 0.0);
        std::vector<int> segmentOf(k, -1);
        for (size_t i = 0; i < k; ++i) {
            const RrGeoCurvenetSpoke &spoke = intersection.spokes[i];
            const int begin = frames.segmentBegin[spoke.curve];
            const int end = frames.segmentBegin[spoke.curve + 1];
            if (end <= begin) {
                continue;
            }
            const int segment = spoke.atCurveEnd ? end - 1 : begin;
            segmentOf[i] = segment;
            l[i] = frames.length[segment];
            t[i] = spoke.atCurveEnd ? -frames.tangent[segment]
                                    : frames.tangent[segment];
        }

        std::vector<RrVec3d> corner(k, RrVec3d(0.0));
        std::vector<double> cornerLen(k, 0.0);
        for (size_t i = 0; i < k; ++i) {
            corner[i] = RrCross(t[i], t[(i + 1) % k]);
            cornerLen[i] = corner[i].GetLength();
        }
        std::vector<RrVec3d> m(k, RrVec3d(0.0));
        for (size_t i = 0; i < k; ++i) {
            if (cornerLen[i] > 1e-9) {
                m[i] = corner[i] / cornerLen[i];
                continue;
            }
            RrVec3d blended = corner[(i + 1) % k] + corner[(i + k - 1) % k];
            if (RrGeoCurvenetSafeNormalize(&blended) > 0.0) {
                m[i] = blended;
            } else {
                m[i] = intersection.referenceNormal;
            }
        }
        RrVec3d fanNormal(0.0);
        for (size_t i = 0; i < k; ++i) {
            fanNormal += m[i];
        }
        if (RrGeoCurvenetSafeNormalize(&fanNormal) == 0.0) {
            fanNormal = intersection.referenceNormal;
        }
        for (size_t i = 0; i < k; ++i) {
            if (RrDot(m[i], fanNormal) < 0.0) {
                m[i] = -m[i];
            }
        }

        for (size_t i = 0; i < k; ++i) {
            const size_t prev = (i + k - 1) % k;
            const size_t next = (i + 1) % k;
            SpokeFrame &out = spokeFrames[x][i];
            out.normalLeft = m[i];
            out.normalRight = m[prev];
            out.widthLeft = l[i] + cornerLen[i] * (l[next] - l[i]);
            out.widthRight = l[i] + cornerLen[prev] * (l[prev] - l[i]);
            if (!(out.widthLeft > RrGeoCurvenetEps)) {
                out.widthLeft = std::max(l[i], RrGeoCurvenetEps);
            }
            if (!(out.widthRight > RrGeoCurvenetEps)) {
                out.widthRight = std::max(l[i], RrGeoCurvenetEps);
            }
            out.valid = (segmentOf[i] >= 0);
        }
    }

    for (size_t c = 0; c < curveCount; ++c) {
        const RrGeoCurvenetCurve &curve = topology.curves[c];
        const int begin = frames.segmentBegin[c];
        const int end = frames.segmentBegin[c + 1];
        const int count = end - begin;
        if (count <= 0 || curve.IsIsolated()) {
            continue;
        }

        auto boundary = [&](bool atCurveEnd, RrVec3d *nLeft, RrVec3d *nRight,
                            double *wLeft, double *wRight) -> bool {
            const int knot = atCurveEnd ? curve.endKnot : curve.startKnot;
            const int slot = topology.intersectionOfKnot[knot];
            if (slot < 0) {
                return false;
            }
            const auto &spokes = topology.intersections[slot].spokes;
            for (size_t i = 0; i < spokes.size(); ++i) {
                if (spokes[i].curve != int(c) ||
                    spokes[i].atCurveEnd != atCurveEnd) {
                    continue;
                }
                const SpokeFrame &sf = spokeFrames[slot][i];
                if (!sf.valid) {
                    return false;
                }
                if (atCurveEnd) {
                    *nLeft = sf.normalRight;
                    *nRight = sf.normalLeft;
                    *wLeft = sf.widthRight;
                    *wRight = sf.widthLeft;
                } else {
                    *nLeft = sf.normalLeft;
                    *nRight = sf.normalRight;
                    *wLeft = sf.widthLeft;
                    *wRight = sf.widthRight;
                }
                return true;
            }
            return false;
        };

        RrVec3d startLeft, startRight, endLeft, endRight;
        double startWLeft = 0, startWRight = 0, endWLeft = 0, endWRight = 0;
        const bool haveStart =
            curve.startIsIntersection &&
            boundary(false, &startLeft, &startRight, &startWLeft, &startWRight);
        const bool haveEnd =
            curve.endIsIntersection &&
            boundary(true, &endLeft, &endRight, &endWLeft, &endWRight);
        if (!haveStart && !haveEnd) {
            continue;
        }

        std::vector<double> alpha(count, 0.0);
        double total = 0.0;
        for (int i = 0; i < count; ++i) {
            alpha[i] = total;
            total += frames.length[begin + i];
        }
        if (total > RrGeoCurvenetEps) {
            for (int i = 0; i < count; ++i) {
                alpha[i] /= total;
            }
        }

        for (int side = 0; side < 2; ++side) {
            const bool left = (side == 0);
            std::vector<RrVec3d> &normals =
                left ? frames.normalLeft : frames.normalRight;
            std::vector<double> &widths =
                left ? frames.widthLeft : frames.widthRight;

            const bool forward = haveStart;
            RrVec3d seed = forward ? (left ? startLeft : startRight)
                                   : (left ? endLeft : endRight);
            const double seedWidth = forward
                                         ? (left ? startWLeft : startWRight)
                                         : (left ? endWLeft : endWRight);

            std::vector<RrVec3d> transported(count, RrVec3d(0.0));
            const int firstIndex = forward ? 0 : count - 1;
            const int step = forward ? 1 : -1;

            RrVec3d n = seed;
            if (!RrGeoCurvenetOrthogonalize(&n, frames.tangent[begin + firstIndex])) {
                n = RrGeoCurvenetAnyPerpendicular(frames.tangent[begin + firstIndex]);
            }
            transported[firstIndex] = n;
            for (int i = firstIndex + step; i >= 0 && i < count; i += step) {
                const RrVec3d &prevT = frames.tangent[begin + i - step];
                const RrVec3d &curT = frames.tangent[begin + i];
                const RrMat3d r = RrGeoSmallestRotation(prevT, curT);
                RrVec3d next = r * transported[i - step];
                if (!RrGeoCurvenetOrthogonalize(&next, curT)) {
                    next = RrGeoCurvenetAnyPerpendicular(curT);
                }
                transported[i] = next;
            }

            double torsion = 0.0;
            const bool bothEnds = haveStart && haveEnd;
            if (bothEnds) {
                const int lastIndex = forward ? count - 1 : 0;
                RrVec3d target = forward ? (left ? endLeft : endRight)
                                         : (left ? startLeft : startRight);
                const RrVec3d &tk = frames.tangent[begin + lastIndex];
                if (RrGeoCurvenetOrthogonalize(&target, tk)) {
                    const RrVec3d &arrived = transported[lastIndex];
                    torsion = std::atan2(RrDot(arrived, RrCross(target, tk)),
                                         RrDot(arrived, target));
                    torsion = -torsion;
                }
            }

            for (int i = 0; i < count; ++i) {
                const double a = forward ? alpha[i] : (1.0 - alpha[i]);
                RrVec3d n_i = transported[i];
                if (bothEnds && std::abs(torsion) > 0.0) {
                    n_i = RrGeoCurvenetAxisAngle(frames.tangent[begin + i], a * torsion) *
                          n_i;
                    if (!RrGeoCurvenetOrthogonalize(&n_i, frames.tangent[begin + i])) {
                        n_i = transported[i];
                    }
                }
                normals[begin + i] = n_i;

                if (bothEnds) {
                    const double w0 = forward
                                          ? (left ? startWLeft : startWRight)
                                          : (left ? endWLeft : endWRight);
                    const double w1 = forward
                                          ? (left ? endWLeft : endWRight)
                                          : (left ? startWLeft : startWRight);
                    widths[begin + i] = (1.0 - a) * w0 + a * w1;
                } else {
                    widths[begin + i] = seedWidth;
                }
            }
        }
        frames.curveFramed[c] = true;
    }

    return frames;
}

bool
RrGeoCurvenetFramesAreValid(const RrGeoCurvenetFrames &frames,
                           std::string *error)
{
    for (size_t c = 0; c + 1 < frames.segmentBegin.size(); ++c) {
        for (int i = frames.segmentBegin[c]; i < frames.segmentBegin[c + 1];
             ++i) {
            if (frames.length[i] <= RrGeoCurvenetEps) {
                if (error) {
                    char buffer[192];
                    std::snprintf(
                        buffer, sizeof(buffer),
                        "curve %zu segment %d has zero length: two curvenet "
                        "samples coincide, which leaves the segment tangent "
                        "undefined",
                        c, i - frames.segmentBegin[c]);
                    *error = buffer;
                }
                return false;
            }
        }
    }
    return true;
}

RrGeoCurvenetGradients
RrGeoComputeCurvenetGradients(const RrGeoCurvenetFrames &restFrames,
                             const RrGeoCurvenetFrames &posedFrames)
{
    RrGeoCurvenetGradients gradients;
    const size_t count =
        std::min(restFrames.GetSegmentCount(), posedFrames.GetSegmentCount());
    gradients.left.assign(count, RrMat3d(1.0));
    gradients.right.assign(count, RrMat3d(1.0));

    for (size_t c = 0; c + 1 < posedFrames.segmentBegin.size(); ++c) {
        const bool framed =
            c < posedFrames.curveFramed.size() && posedFrames.curveFramed[c] &&
            c < restFrames.curveFramed.size() && restFrames.curveFramed[c];
        const int begin = posedFrames.segmentBegin[c];
        const int end = posedFrames.segmentBegin[c + 1];
        for (int i = begin; i < end && size_t(i) < count; ++i) {
            if (!framed) {
                const double restLen = restFrames.length[i];
                const double scale =
                    (restLen > RrGeoCurvenetEps) ? posedFrames.length[i] / restLen : 1.0;
                const RrMat3d rotation = RrGeoSmallestRotation(
                    restFrames.tangent[i], posedFrames.tangent[i]);
                gradients.left[i] = rotation * scale;
                gradients.right[i] = gradients.left[i];
                continue;
            }
            for (int side = 0; side < 2; ++side) {
                const bool left = (side == 0);
                const RrMat3d posed = posedFrames.GetScaledFrame(i, left);
                const RrMat3d rest = restFrames.GetScaledFrame(i, left);
                const RrMat3d f = posed * rest.GetInverse();
                (left ? gradients.left : gradients.right)[i] = f;
            }
        }
    }
    return gradients;
}

RrGeoCurvenetSampleGradients
RrGeoRemapGradientsToSamples(const RrGeoCurvenetTopology &topology,
                            const RrGeoCurvenetSampling &sampling,
                            const RrGeoCurvenetFrames &frames,
                            const RrGeoCurvenetGradients &gradients)
{
    RrGeoCurvenetSampleGradients out;
    const size_t sampleCount = sampling.GetSampleCount();
    out.left.assign(sampleCount, RrMat3d(1.0));
    out.right.assign(sampleCount, RrMat3d(1.0));

    for (size_t c = 0; c < topology.curves.size(); ++c) {
        const int base = sampling.curveBegin[c];
        const int n = sampling.GetCurveSampleCount(c);
        const int segBase = frames.segmentBegin[c];
        const int segCount = frames.segmentBegin[c + 1] - segBase;
        if (n <= 0 || segCount <= 0) {
            continue;
        }
        const bool closed = topology.curves[c].closed;
        for (int i = 0; i < n; ++i) {
            int before = i - 1;
            int after = i;
            if (closed) {
                before = (i - 1 + segCount) % segCount;
                after = i % segCount;
            } else {
                if (after >= segCount) {
                    after = -1;
                }
            }
            auto pick = [&](const std::vector<RrMat3d> &source) {
                if (before >= 0 && after >= 0) {
                    return (source[segBase + before] + source[segBase + after]) *
                           0.5;
                }
                if (before >= 0) {
                    return source[segBase + before];
                }
                if (after >= 0) {
                    return source[segBase + after];
                }
                return RrMat3d(1.0);
            };
            out.left[base + i] = pick(gradients.left);
            out.right[base + i] = pick(gradients.right);
        }
    }
    return out;
}

// ---------------------------------------------------------------------------
// Cut meshes (cutMesh.*): projection, tracing, per-face arrangement, the
// polygonal Laplacian and system assembly.
// ---------------------------------------------------------------------------

struct RrGeoMeshPoint {
    int face = -1;
    int indexOffset = 0;
    int cornerBegin = 0;
    int cornerCount = 0;
};

struct RrGeoCutCurveRef {
    int sampleA = -1;
    int sampleB = -1;
    double tA = 0.0;
    double tB = 1.0;
};

struct RrGeoCutMesh {
    std::vector<RrVec3d> nodePosition;
    std::vector<RrGeoMeshPoint> nodeBinding;
    std::vector<float> bindingWeights;
    std::vector<int> nodeMeshVertex;
    std::vector<int> nodeSample;

    std::vector<int> faceBegin;
    std::vector<int> cornerNode;
    std::vector<int> cornerVertexUnknown;
    std::vector<int> cornerConstraintBegin;
    std::vector<int> cornerConstraintIndex;
    std::vector<double> cornerConstraintWeight;
    std::vector<int> faceSource;

    std::vector<int> vertexUnknown;
    std::vector<int> unknownVertex;
    int unknownCount = 0;
    std::vector<int> vertexConstraintNode;
    std::vector<int> unreachedVertices;

    int constraintCount = 0;

    std::vector<RrGeoMeshPoint> sampleBinding;
    std::vector<RrVec3d> sampleResidual;

    size_t GetCutFaceCount() const
    {
        return faceBegin.empty() ? 0 : faceBegin.size() - 1;
    }
    size_t GetNodeCount() const { return nodePosition.size(); }

    RrVec3d EvaluateBinding(
        const RrGeoMeshPoint &point, const std::vector<RrVec3f> &meshPoints,
        const std::vector<int> &faceVertexCounts,
        const std::vector<int> &faceVertexIndices) const
    {
        (void)faceVertexCounts;
        RrVec3d out(0.0);
        if (point.face < 0 || point.cornerCount <= 0) {
            return out;
        }
        for (int i = 0; i < point.cornerCount; ++i) {
            const int vertex = faceVertexIndices[point.indexOffset + i];
            const double w = bindingWeights[point.cornerBegin + i];
            out += RrGeoToVec3d(meshPoints[vertex]) * w;
        }
        return out;
    }
};

struct RrGeoCutMeshReport {
    int sampleCount = 0;
    int cutFaceCount = 0;
    int crackCount = 0;
    int tracedSegments = 0;
    int failedTraces = 0;
    int lostFaces = 0;
    double maxResidual = 0.0;
    double meanEdgeLength = 0.0;
    std::vector<std::string> warnings;
};

struct RrGeoMeshView {
    std::vector<RrVec3d> points;
    std::vector<int> faceBegin;
    std::vector<int> faceCorner;
    std::vector<RrVec3d> faceNormal;
    std::vector<RrVec3d> faceCentroid;
    std::vector<int> triangleFace;
    std::vector<int> triangleVertex;
    std::map<std::pair<int, int>, std::vector<std::pair<int, int>>> edgeFaces;
    double bboxDiagonal = 0.0;
    double meanEdge = 0.0;

    size_t GetFaceCount() const { return faceBegin.size() - 1; }
    int GetFaceSize(int f) const { return faceBegin[f + 1] - faceBegin[f]; }
    int GetFaceVertex(int f, int corner) const {
        return faceCorner[faceBegin[f] + corner];
    }
};

RrGeoMeshView
RrGeoBuildMeshView(const std::vector<RrVec3f> &meshPoints,
                  const std::vector<int> &counts,
                  const std::vector<int> &indices)
{
    RrGeoMeshView view;
    view.points.reserve(meshPoints.size());
    for (const RrVec3f &p : meshPoints) {
        view.points.push_back(RrGeoToVec3d(p));
    }
    view.faceBegin.push_back(0);
    int cursor = 0;
    double edgeSum = 0.0;
    size_t edgeCount = 0;
    for (size_t f = 0; f < counts.size(); ++f) {
        const int n = counts[f];
        for (int i = 0; i < n; ++i) {
            view.faceCorner.push_back(indices[cursor + i]);
        }
        view.faceBegin.push_back(int(view.faceCorner.size()));

        RrVec3d normal(0.0), centroid(0.0);
        for (int i = 0; i < n; ++i) {
            const RrVec3d &a = view.points[indices[cursor + i]];
            const RrVec3d &b = view.points[indices[cursor + (i + 1) % n]];
            normal[0] += (a[1] - b[1]) * (a[2] + b[2]);
            normal[1] += (a[2] - b[2]) * (a[0] + b[0]);
            normal[2] += (a[0] - b[0]) * (a[1] + b[1]);
            centroid += a;
            edgeSum += (b - a).GetLength();
            ++edgeCount;
            const int v0 = indices[cursor + i];
            const int v1 = indices[cursor + (i + 1) % n];
            view.edgeFaces[{std::min(v0, v1), std::max(v0, v1)}].push_back(
                {int(f), i});
        }
        RrGeoCurvenetSafeNormalize(&normal);
        view.faceNormal.push_back(normal);
        view.faceCentroid.push_back(n > 0 ? centroid / double(n) : centroid);

        for (int i = 1; i + 1 < n; ++i) {
            view.triangleFace.push_back(int(f));
            view.triangleVertex.push_back(indices[cursor]);
            view.triangleVertex.push_back(indices[cursor + i]);
            view.triangleVertex.push_back(indices[cursor + i + 1]);
        }
        cursor += n;
    }
    view.meanEdge = (edgeCount > 0) ? edgeSum / double(edgeCount) : 0.0;

    if (!view.points.empty()) {
        RrVec3d lo = view.points[0], hi = view.points[0];
        for (const RrVec3d &p : view.points) {
            for (int k = 0; k < 3; ++k) {
                lo[k] = std::min(lo[k], p[k]);
                hi[k] = std::max(hi[k], p[k]);
            }
        }
        view.bboxDiagonal = (hi - lo).GetLength();
    }
    return view;
}

struct RrGeoTriangleGrid {
    RrVec3d origin{0, 0, 0};
    double cell = 1.0;
    int dim[3] = {1, 1, 1};
    std::vector<int> bucketStart;
    std::vector<int> bucketItem;

    int Index(int x, int y, int z) const {
        return (z * dim[1] + y) * dim[0] + x;
    }
    void Clamp(int c[3]) const {
        for (int k = 0; k < 3; ++k) {
            c[k] = std::max(0, std::min(dim[k] - 1, c[k]));
        }
    }
    void Locate(const RrVec3d &p, int c[3]) const {
        for (int k = 0; k < 3; ++k) {
            c[k] = int(std::floor((p[k] - origin[k]) / cell));
        }
        Clamp(c);
    }
};

RrGeoTriangleGrid
RrGeoBuildTriangleGrid(const RrGeoMeshView &view)
{
    RrGeoTriangleGrid grid;
    const size_t triangles = view.triangleFace.size();
    if (triangles == 0 || view.points.empty()) {
        grid.bucketStart.assign(2, 0);
        return grid;
    }
    RrVec3d lo = view.points[0], hi = view.points[0];
    for (const RrVec3d &p : view.points) {
        for (int k = 0; k < 3; ++k) {
            lo[k] = std::min(lo[k], p[k]);
            hi[k] = std::max(hi[k], p[k]);
        }
    }
    const RrVec3d span = hi - lo;
    const double volume = std::max(span[0], RrGeoCurvenetEps) * std::max(span[1], RrGeoCurvenetEps) *
                          std::max(span[2], RrGeoCurvenetEps);
    grid.cell = std::max(std::cbrt(volume * 4.0 / double(triangles)),
                         view.meanEdge * 0.5);
    if (!(grid.cell > 0.0)) {
        grid.cell = 1.0;
    }
    grid.origin = lo - RrVec3d(grid.cell, grid.cell, grid.cell);
    for (int k = 0; k < 3; ++k) {
        grid.dim[k] = std::max(
            1, std::min(256, int(std::ceil(span[k] / grid.cell)) + 3));
    }

    const int cells = grid.dim[0] * grid.dim[1] * grid.dim[2];
    std::vector<int> counts(cells + 1, 0);
    auto forEachCell = [&](size_t t, const std::function<void(int)> &fn) {
        RrVec3d tlo = view.points[view.triangleVertex[3 * t]];
        RrVec3d thi = tlo;
        for (int i = 1; i < 3; ++i) {
            const RrVec3d &p = view.points[view.triangleVertex[3 * t + i]];
            for (int k = 0; k < 3; ++k) {
                tlo[k] = std::min(tlo[k], p[k]);
                thi[k] = std::max(thi[k], p[k]);
            }
        }
        int a[3], b[3];
        grid.Locate(tlo, a);
        grid.Locate(thi, b);
        for (int z = a[2]; z <= b[2]; ++z) {
            for (int y = a[1]; y <= b[1]; ++y) {
                for (int x = a[0]; x <= b[0]; ++x) {
                    fn(grid.Index(x, y, z));
                }
            }
        }
    };
    for (size_t t = 0; t < triangles; ++t) {
        forEachCell(t, [&](int c) { ++counts[c + 1]; });
    }
    for (int i = 0; i < cells; ++i) {
        counts[i + 1] += counts[i];
    }
    grid.bucketStart = counts;
    grid.bucketItem.assign(counts[cells], 0);
    std::vector<int> cursor(counts.begin(), counts.end() - 1);
    for (size_t t = 0; t < triangles; ++t) {
        forEachCell(t, [&](int c) { grid.bucketItem[cursor[c]++] = int(t); });
    }
    return grid;
}

RrVec3d
RrGeoClosestOnSegment(const RrVec3d &p, const RrVec3d &a, const RrVec3d &b,
                     double *parameter)
{
    const RrVec3d d = b - a;
    const double dd = RrDot(d, d);
    double t = (dd <= RrGeoCurvenetEps) ? 0.0 : RrDot(p - a, d) / dd;
    t = RrClamp(t, 0.0, 1.0);
    *parameter = t;
    return a + d * t;
}

RrVec3d
RrGeoClosestOnTriangle(const RrVec3d &p, const RrVec3d &a, const RrVec3d &b,
                      const RrVec3d &c, double bary[3])
{
    const RrVec3d ab = b - a, ac = c - a, ap = p - a;
    const double d1 = RrDot(ab, ap), d2 = RrDot(ac, ap);
    if (d1 <= 0.0 && d2 <= 0.0) {
        bary[0] = 1.0; bary[1] = 0.0; bary[2] = 0.0;
        return a;
    }
    const RrVec3d bp = p - b;
    const double d3 = RrDot(ab, bp), d4 = RrDot(ac, bp);
    if (d3 >= 0.0 && d4 <= d3) {
        bary[0] = 0.0; bary[1] = 1.0; bary[2] = 0.0;
        return b;
    }
    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
        const double v = (d1 - d3 != 0.0) ? d1 / (d1 - d3) : 0.0;
        bary[0] = 1.0 - v; bary[1] = v; bary[2] = 0.0;
        return a + ab * v;
    }
    const RrVec3d cp = p - c;
    const double d5 = RrDot(ab, cp), d6 = RrDot(ac, cp);
    if (d6 >= 0.0 && d5 <= d6) {
        bary[0] = 0.0; bary[1] = 0.0; bary[2] = 1.0;
        return c;
    }
    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
        const double w = (d2 - d6 != 0.0) ? d2 / (d2 - d6) : 0.0;
        bary[0] = 1.0 - w; bary[1] = 0.0; bary[2] = w;
        return a + ac * w;
    }
    const double va = d3 * d6 - d5 * d4;
    if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
        const double denom = (d4 - d3) + (d5 - d6);
        const double w = (denom != 0.0) ? (d4 - d3) / denom : 0.0;
        bary[0] = 0.0; bary[1] = 1.0 - w; bary[2] = w;
        return b + (c - b) * w;
    }
    const double denom = va + vb + vc;
    const double v = (denom != 0.0) ? vb / denom : 0.0;
    const double w = (denom != 0.0) ? vc / denom : 0.0;
    bary[0] = 1.0 - v - w; bary[1] = v; bary[2] = w;
    return a + ab * v + ac * w;
}

struct RrGeoProjection {
    int face = -1;
    int triangle = -1;
    double weights[3] = {0,0,0};
    RrVec3d position{0, 0, 0};
    double distance = 0.0;
};

RrGeoProjection
RrGeoClosestPoint(const RrGeoMeshView &view, const RrGeoTriangleGrid &grid,
                 const RrVec3d &query)
{
    RrGeoProjection best;
    best.distance = std::numeric_limits<double>::max();
    auto consider = [&](int t) {
        double bary[3];
        const RrVec3d p = RrGeoClosestOnTriangle(
            query, view.points[view.triangleVertex[3 * t]],
            view.points[view.triangleVertex[3 * t + 1]],
            view.points[view.triangleVertex[3 * t + 2]], bary);
        const double d = (p - query).GetLength();
        if (d < best.distance) {
            best.distance = d;
            best.position = p;
            best.face = view.triangleFace[t];
            best.triangle = t;
            for (int i = 0; i < 3; ++i) best.weights[i] = bary[i];
        }
    };

    if (grid.bucketItem.empty()) {
        for (size_t t = 0; t < view.triangleFace.size(); ++t) {
            consider(int(t));
        }
        return best;
    }
    int centre[3];
    grid.Locate(query, centre);
    const int maxRing =
        std::max({grid.dim[0], grid.dim[1], grid.dim[2]});
    for (int ring = 0; ring <= maxRing; ++ring) {
        for (int z = centre[2] - ring; z <= centre[2] + ring; ++z) {
            if (z < 0 || z >= grid.dim[2]) continue;
            for (int y = centre[1] - ring; y <= centre[1] + ring; ++y) {
                if (y < 0 || y >= grid.dim[1]) continue;
                for (int x = centre[0] - ring; x <= centre[0] + ring; ++x) {
                    if (x < 0 || x >= grid.dim[0]) continue;
                    const bool onShell =
                        (std::abs(x - centre[0]) == ring ||
                         std::abs(y - centre[1]) == ring ||
                         std::abs(z - centre[2]) == ring);
                    if (ring > 0 && !onShell) continue;
                    const int c = grid.Index(x, y, z);
                    for (int p = grid.bucketStart[c];
                         p < grid.bucketStart[c + 1]; ++p) {
                        consider(grid.bucketItem[p]);
                    }
                }
            }
        }
        if (best.face >= 0 && best.distance <= double(ring) * grid.cell) {
            break;
        }
    }
    if (best.face < 0) {
        for (size_t t = 0; t < view.triangleFace.size(); ++t) {
            consider(int(t));
        }
    }
    return best;
}

struct RrGeoFaceFrame {
    RrVec3d origin{0, 0, 0};
    RrVec3d axisX{1, 0, 0};
    RrVec3d axisY{0, 1, 0};
    RrVec3d normal{0, 0, 1};

    RrVec2d To2D(const RrVec3d &p) const {
        const RrVec3d d = p - origin;
        return RrVec2d(RrDot(d, axisX), RrDot(d, axisY));
    }
};

RrGeoFaceFrame
RrGeoMakeFaceFrame(const RrGeoMeshView &view, int face)
{
    RrGeoFaceFrame frame;
    frame.origin = view.faceCentroid[face];
    frame.normal = view.faceNormal[face];
    if (frame.normal.GetLength() <= RrGeoCurvenetEps) {
        frame.normal = RrVec3d(0, 0, 1);
    }
    RrVec3d x = view.points[view.GetFaceVertex(face, 0)] - frame.origin;
    x -= frame.normal * RrDot(x, frame.normal);
    if (RrGeoCurvenetSafeNormalize(&x) == 0.0) {
        x = (std::abs(frame.normal[0]) < 0.9) ? RrVec3d(1, 0, 0)
                                              : RrVec3d(0, 1, 0);
        x -= frame.normal * RrDot(x, frame.normal);
        RrGeoCurvenetSafeNormalize(&x);
    }
    frame.axisX = x;
    frame.axisY = RrCross(frame.normal, x);
    return frame;
}

std::vector<double>
RrGeoFaceWeights(const RrGeoMeshView &view, int face, const RrVec3d &point)
{
    const int n = view.GetFaceSize(face);
    std::vector<double> weights(n, 0.0);
    if (n <= 0) {
        return weights;
    }
    if (n == 1) {
        weights[0] = 1.0;
        return weights;
    }
    double bestDistance = std::numeric_limits<double>::max();
    int bestCorner = 1;
    double bestBary[3] = {1.0, 0.0, 0.0};
    for (int i = 1; i + 1 < n; ++i) {
        double bary[3];
        const RrVec3d p = RrGeoClosestOnTriangle(
            point, view.points[view.GetFaceVertex(face, 0)],
            view.points[view.GetFaceVertex(face, i)],
            view.points[view.GetFaceVertex(face, i + 1)], bary);
        const double d = (p - point).GetLength();
        if (d < bestDistance) {
            bestDistance = d;
            bestCorner = i;
            bestBary[0] = bary[0];
            bestBary[1] = bary[1];
            bestBary[2] = bary[2];
        }
    }
    weights[0] += bestBary[0];
    weights[bestCorner] += bestBary[1];
    weights[bestCorner + 1] += bestBary[2];
    return weights;
}

enum class RrGeoSampleKind { Vertex, Edge, Face };

struct RrGeoNodeDraft {
    RrVec3d position{0, 0, 0};
    int face = -1;
    std::vector<double> weights;
    int meshVertex = -1;
    int sample = -1;
};

struct RrGeoSubEdge {
    int face = -1;
    int node0 = -1;
    int node1 = -1;
    int sampleA = -1;
    int sampleB = -1;
    double t0 = 0.0;
    double t1 = 1.0;
};

struct RrGeoDisjointSet {
    std::vector<int> parent;
    explicit RrGeoDisjointSet(int n) : parent(n) {
        for (int i = 0; i < n; ++i) {
            parent[i] = i;
        }
    }
    int Find(int a) {
        while (parent[a] != a) {
            parent[a] = parent[parent[a]];
            a = parent[a];
        }
        return a;
    }
    void Union(int a, int b) {
        a = Find(a);
        b = Find(b);
        if (a != b) {
            parent[b] = a;
        }
    }
};

double
RrGeoMeshMeanEdgeLength(const std::vector<RrVec3f> &meshPoints,
                       const std::vector<int> &faceVertexCounts,
                       const std::vector<int> &faceVertexIndices)
{
    double sum = 0.0;
    size_t count = 0;
    int cursor = 0;
    for (int n : faceVertexCounts) {
        for (int i = 0; i < n; ++i) {
            const int a = faceVertexIndices[cursor + i];
            const int b = faceVertexIndices[cursor + (i + 1) % n];
            if (a < 0 || b < 0 || size_t(a) >= meshPoints.size() ||
                size_t(b) >= meshPoints.size()) {
                continue;
            }
            sum += (RrGeoToVec3d(meshPoints[b]) - RrGeoToVec3d(meshPoints[a])).GetLength();
            ++count;
        }
        cursor += n;
    }
    return (count > 0) ? sum / double(count) : 0.0;
}

bool
RrGeoBuildCutMesh(const RrGeoCurvenetTopology &topology,
                 const RrGeoCurvenetSampling &sampling,
                 const std::vector<RrVec3f> &meshPoints,
                 const std::vector<int> &faceVertexCounts,
                 const std::vector<int> &faceVertexIndices,
                 RrGeoCutMesh *cutMesh, RrGeoCutMeshReport *report,
                 std::string *error)
{
    auto fail = [&](const std::string &message) {
        if (error) {
            *error = message;
        }
        return false;
    };
    if (meshPoints.empty() || faceVertexCounts.empty()) {
        return fail("the curvenet target has no geometry to cut");
    }
    {
        long total = 0;
        for (int n : faceVertexCounts) {
            if (n < 3) {
                return fail("the curvenet target has a face with " +
                            std::to_string(n) +
                            " vertices; the cut needs polygons");
            }
            total += n;
        }
        if (total != long(faceVertexIndices.size())) {
            return fail("faceVertexCounts sums to " + std::to_string(total) +
                        " but faceVertexIndices holds " +
                        std::to_string(faceVertexIndices.size()));
        }
        for (int v : faceVertexIndices) {
            if (v < 0 || size_t(v) >= meshPoints.size()) {
                return fail("faceVertexIndices names vertex " +
                            std::to_string(v) + " outside the point array");
            }
        }
    }

    *cutMesh = RrGeoCutMesh();
    *report = RrGeoCutMeshReport();

    const RrGeoMeshView view =
        RrGeoBuildMeshView(meshPoints, faceVertexCounts, faceVertexIndices);
    const RrGeoTriangleGrid grid = RrGeoBuildTriangleGrid(view);
    report->meanEdgeLength = view.meanEdge;
    report->sampleCount = int(sampling.GetSampleCount());
    const double tolerance = std::max(view.bboxDiagonal * 1e-5, 1e-9);

    std::vector<RrGeoNodeDraft> nodes;
    std::vector<int> nodeOfVertex(meshPoints.size(), -1);
    std::map<std::pair<int, int>, std::vector<std::pair<double, int>>> edgeNodes;

    {
        std::vector<int> anyFace(meshPoints.size(), -1);
        std::vector<int> anyCorner(meshPoints.size(), -1);
        for (size_t f = 0; f < view.GetFaceCount(); ++f) {
            const int n = view.GetFaceSize(int(f));
            for (int i = 0; i < n; ++i) {
                const int v = view.GetFaceVertex(int(f), i);
                if (anyFace[v] < 0) {
                    anyFace[v] = int(f);
                    anyCorner[v] = i;
                }
            }
        }
        for (size_t v = 0; v < meshPoints.size(); ++v) {
            if (anyFace[v] < 0) {
                continue;  // unreferenced point
            }
            RrGeoNodeDraft draft;
            draft.position = view.points[v];
            draft.face = anyFace[v];
            draft.weights.assign(view.GetFaceSize(anyFace[v]), 0.0);
            draft.weights[anyCorner[v]] = 1.0;
            draft.meshVertex = int(v);
            nodeOfVertex[v] = int(nodes.size());
            nodes.push_back(std::move(draft));
        }
    }

    auto edgeKey = [](int a, int b) {
        return std::make_pair(std::min(a, b), std::max(a, b));
    };
    auto edgeNode = [&](int v0, int v1, double u, const RrVec3d &position,
                        int sample) {
        const auto key = edgeKey(v0, v1);
        const double canonical = (v0 == key.first) ? u : 1.0 - u;
        auto &bucket = edgeNodes[key];
        const double snap =
            (view.points[key.second] - view.points[key.first]).GetLength();
        const double parameterTolerance =
            (snap > RrGeoCurvenetEps) ? tolerance / snap : 1e-9;

        if (canonical <= parameterTolerance && nodeOfVertex[key.first] >= 0) {
            const int node = nodeOfVertex[key.first];
            if (sample >= 0 && nodes[node].sample < 0) {
                nodes[node].sample = sample;
            }
            return node;
        }
        if (canonical >= 1.0 - parameterTolerance &&
            nodeOfVertex[key.second] >= 0) {
            const int node = nodeOfVertex[key.second];
            if (sample >= 0 && nodes[node].sample < 0) {
                nodes[node].sample = sample;
            }
            return node;
        }

        for (auto &entry : bucket) {
            if (std::abs(entry.first - canonical) <= parameterTolerance) {
                if (sample >= 0 && nodes[entry.second].sample < 0) {
                    nodes[entry.second].sample = sample;
                }
                return entry.second;
            }
        }
        RrGeoNodeDraft draft;
        draft.position = position;
        const auto it = view.edgeFaces.find(key);
        const int face = (it != view.edgeFaces.end() && !it->second.empty())
                             ? it->second.front().first
                             : 0;
        draft.face = face;
        draft.weights.assign(view.GetFaceSize(face), 0.0);
        for (int i = 0; i < view.GetFaceSize(face); ++i) {
            const int v = view.GetFaceVertex(face, i);
            if (v == key.first) {
                draft.weights[i] += 1.0 - canonical;
            } else if (v == key.second) {
                draft.weights[i] += canonical;
            }
        }
        draft.sample = sample;
        const int index = int(nodes.size());
        nodes.push_back(std::move(draft));
        bucket.push_back({canonical, index});
        return index;
    };

    std::vector<std::vector<int>> facesOfVertex(meshPoints.size());
    for (size_t f = 0; f < view.GetFaceCount(); ++f) {
        const int n = view.GetFaceSize(int(f));
        for (int i = 0; i < n; ++i) {
            facesOfVertex[view.GetFaceVertex(int(f), i)].push_back(int(f));
        }
    }

    const size_t sampleCount = sampling.GetSampleCount();
    std::vector<int> nodeOfSample(sampleCount, -1);
    std::vector<int> faceOfSample(sampleCount, -1);
    struct Site {
        RrGeoSampleKind kind = RrGeoSampleKind::Face;
        int vertex = -1;
        int edgeA = -1, edgeB = -1;
        double edgeU = 0.0;
        std::vector<int> faces;
    };
    std::vector<Site> sites(sampleCount);
    std::vector<RrVec3d> projectedSample(sampleCount, RrVec3d(0.0));
    cutMesh->sampleBinding.assign(sampleCount, RrGeoMeshPoint());
    cutMesh->sampleResidual.assign(sampleCount, RrVec3d(0.0));
    std::vector<std::vector<double>> sampleWeights(sampleCount);

    for (size_t s = 0; s < sampleCount; ++s) {
        const RrVec3d query = sampling.positions[s];
        const RrGeoProjection hit = RrGeoClosestPoint(view, grid, query);
        if (hit.face < 0) {
            return fail("a curvenet sample could not be projected onto the "
                        "target surface");
        }
        faceOfSample[s] = hit.face;
        projectedSample[s] = hit.position;
        cutMesh->sampleResidual[s] = query - hit.position;
        report->maxResidual =
            std::max(report->maxResidual, hit.distance);
        sampleWeights[s] = RrGeoFaceWeights(view, hit.face, hit.position);

        const int n = view.GetFaceSize(hit.face);
        RrGeoSampleKind kind = RrGeoSampleKind::Face;
        int vertexHit = -1;
        int edgeCorner = -1;
        double edgeParameter = 0.0;
        for (int i = 0; i < n; ++i) {
            const int v = view.GetFaceVertex(hit.face, i);
            if ((view.points[v] - hit.position).GetLength() <= tolerance) {
                kind = RrGeoSampleKind::Vertex;
                vertexHit = v;
                break;
            }
        }
        if (kind == RrGeoSampleKind::Face) {
            for (int i = 0; i < n; ++i) {
                const int a = view.GetFaceVertex(hit.face, i);
                const int b = view.GetFaceVertex(hit.face, (i + 1) % n);
                double u = 0.0;
                const RrVec3d p = RrGeoClosestOnSegment(
                    hit.position, view.points[a], view.points[b], &u);
                if ((p - hit.position).GetLength() <= tolerance) {
                    kind = RrGeoSampleKind::Edge;
                    edgeCorner = i;
                    edgeParameter = u;
                    break;
                }
            }
        }

        sites[s].kind = kind;
        if (kind == RrGeoSampleKind::Vertex) {
            nodeOfSample[s] = nodeOfVertex[vertexHit];
            if (nodeOfSample[s] >= 0) {
                nodes[nodeOfSample[s]].sample = int(s);
            }
            sites[s].vertex = vertexHit;
            sites[s].faces = facesOfVertex[vertexHit];
        } else if (kind == RrGeoSampleKind::Edge) {
            const int a = view.GetFaceVertex(hit.face, edgeCorner);
            const int b = view.GetFaceVertex(hit.face, (edgeCorner + 1) % n);
            nodeOfSample[s] =
                edgeNode(a, b, edgeParameter, hit.position, int(s));
            sites[s].edgeA = a;
            sites[s].edgeB = b;
            sites[s].edgeU = edgeParameter;
            const auto it = view.edgeFaces.find(edgeKey(a, b));
            if (it != view.edgeFaces.end()) {
                for (const auto &entry : it->second) {
                    sites[s].faces.push_back(entry.first);
                }
            }
        } else {
            sites[s].faces.push_back(hit.face);
            RrGeoNodeDraft draft;
            draft.position = hit.position;
            draft.face = hit.face;
            draft.weights = sampleWeights[s];
            draft.sample = int(s);
            nodeOfSample[s] = int(nodes.size());
            nodes.push_back(std::move(draft));
        }
    }

    std::vector<RrGeoSubEdge> subEdges;
    struct AlongEdge {
        int sampleA;
        int sampleB;
        std::pair<int, int> edge;
        double uFrom = 0.0;
        double uTo = 1.0;
        double tFrom = 0.0;
        double tTo = 1.0;
    };
    std::vector<AlongEdge> alongEdge;

    std::vector<std::vector<int>> edgesOfVertexA(meshPoints.size());
    for (const auto &entry : view.edgeFaces) {
        edgesOfVertexA[entry.first.first].push_back(entry.first.second);
        edgesOfVertexA[entry.first.second].push_back(entry.first.first);
    }

    auto siteParameter = [&](const Site &site,
                             const std::pair<int, int> &key) {
        if (site.kind == RrGeoSampleKind::Vertex) {
            if (site.vertex == key.first) return 0.0;
            if (site.vertex == key.second) return 1.0;
            return -1.0;
        }
        if (site.kind == RrGeoSampleKind::Edge &&
            edgeKey(site.edgeA, site.edgeB) == key) {
            return (site.edgeA == key.first) ? site.edgeU : 1.0 - site.edgeU;
        }
        return -1.0;
    };

    auto siteEdges = [&](const Site &site) {
        std::vector<std::pair<int, int>> out;
        if (site.kind == RrGeoSampleKind::Edge) {
            out.push_back(edgeKey(site.edgeA, site.edgeB));
        } else if (site.kind == RrGeoSampleKind::Vertex) {
            for (int other : edgesOfVertexA[site.vertex]) {
                out.push_back(edgeKey(site.vertex, other));
            }
        }
        return out;
    };

    auto sharedMeshEdge = [&](const Site &a,
                              const Site &b) -> std::pair<int, int> {
        const auto none = std::make_pair(-1, -1);
        auto isEdge = [&](int v0, int v1) {
            return view.edgeFaces.count(edgeKey(v0, v1)) > 0;
        };
        if (a.kind == RrGeoSampleKind::Edge && b.kind == RrGeoSampleKind::Edge) {
            if (edgeKey(a.edgeA, a.edgeB) == edgeKey(b.edgeA, b.edgeB)) {
                return edgeKey(a.edgeA, a.edgeB);
            }
            return none;
        }
        if (a.kind == RrGeoSampleKind::Vertex && b.kind == RrGeoSampleKind::Edge) {
            if (a.vertex == b.edgeA || a.vertex == b.edgeB) {
                return edgeKey(b.edgeA, b.edgeB);
            }
            return none;
        }
        if (a.kind == RrGeoSampleKind::Edge && b.kind == RrGeoSampleKind::Vertex) {
            if (b.vertex == a.edgeA || b.vertex == a.edgeB) {
                return edgeKey(a.edgeA, a.edgeB);
            }
            return none;
        }
        if (a.kind == RrGeoSampleKind::Vertex && b.kind == RrGeoSampleKind::Vertex) {
            if (a.vertex != b.vertex && isEdge(a.vertex, b.vertex)) {
                return edgeKey(a.vertex, b.vertex);
            }
            return none;
        }
        return none;
    };

    auto adjacentFace = [&](int v0, int v1, int from) {
        const auto it = view.edgeFaces.find(edgeKey(v0, v1));
        if (it == view.edgeFaces.end()) {
            return -1;
        }
        for (const auto &entry : it->second) {
            if (entry.first != from) {
                return entry.first;
            }
        }
        return -1;
    };

    for (size_t c = 0; c < topology.curves.size(); ++c) {
        const int begin = sampling.curveBegin[c];
        const int count = sampling.GetCurveSampleCount(c);
        if (count <= 1) {
            continue;
        }
        const bool closed = topology.curves[c].closed;
        const int segments = closed ? count : count - 1;
        for (int i = 0; i < segments; ++i) {
            const int sa = begin + i;
            const int sb = begin + ((i + 1) % count);
            const int fa = faceOfSample[sa];
            const int fb = faceOfSample[sb];
            if (fa < 0 || fb < 0) {
                continue;
            }
            const std::pair<int, int> shared =
                sharedMeshEdge(sites[sa], sites[sb]);
            if (shared.first >= 0) {
                AlongEdge hop;
                hop.sampleA = sa;
                hop.sampleB = sb;
                hop.edge = shared;
                hop.uFrom = siteParameter(sites[sa], shared);
                hop.uTo = siteParameter(sites[sb], shared);
                alongEdge.push_back(hop);
                continue;
            }

            {
                const RrVec3d from = projectedSample[sa];
                const RrVec3d to = projectedSample[sb];
                RrVec3d direction = to - from;
                const double total = RrGeoCurvenetSafeNormalize(&direction);
                bool routed = false;
                if (total > RrGeoCurvenetEps) {
                    const auto edgesA = siteEdges(sites[sa]);
                    const auto edgesB = siteEdges(sites[sb]);
                    for (const auto &ea : edgesA) {
                        if (routed) break;
                        for (const auto &eb : edgesB) {
                            if (ea == eb) continue;
                            int via = -1;
                            for (int candidate : {ea.first, ea.second}) {
                                if (candidate == eb.first ||
                                    candidate == eb.second) {
                                    via = candidate;
                                    break;
                                }
                            }
                            if (via < 0) continue;
                            const RrVec3d pivot = view.points[via];
                            const double along = RrDot(pivot - from, direction);
                            if (along < -tolerance ||
                                along > total + tolerance) {
                                continue;
                            }
                            if ((pivot - (from + direction * along))
                                    .GetLength() > tolerance) {
                                continue;
                            }
                            const double t = RrClamp(along / total, 0.0, 1.0);
                            AlongEdge first;
                            first.sampleA = sa;
                            first.sampleB = sb;
                            first.edge = ea;
                            first.uFrom = siteParameter(sites[sa], ea);
                            first.uTo = (via == ea.first) ? 0.0 : 1.0;
                            first.tFrom = 0.0;
                            first.tTo = t;
                            AlongEdge second;
                            second.sampleA = sa;
                            second.sampleB = sb;
                            second.edge = eb;
                            second.uFrom = (via == eb.first) ? 0.0 : 1.0;
                            second.uTo = siteParameter(sites[sb], eb);
                            second.tFrom = t;
                            second.tTo = 1.0;
                            if (first.uFrom >= 0.0 && second.uTo >= 0.0) {
                                alongEdge.push_back(first);
                                alongEdge.push_back(second);
                                routed = true;
                                break;
                            }
                        }
                    }
                }
                if (routed) {
                    continue;
                }
            }

            int commonFace = -1;
            for (int candidate : sites[sa].faces) {
                if (std::find(sites[sb].faces.begin(), sites[sb].faces.end(),
                              candidate) != sites[sb].faces.end()) {
                    commonFace = candidate;
                    break;
                }
            }
            if (commonFace >= 0) {
                RrGeoSubEdge sub;
                sub.face = commonFace;
                sub.node0 = nodeOfSample[sa];
                sub.node1 = nodeOfSample[sb];
                sub.sampleA = sa;
                sub.sampleB = sb;
                sub.t0 = 0.0;
                sub.t1 = 1.0;
                if (sub.node0 != sub.node1) {
                    subEdges.push_back(sub);
                }
                continue;
            }

            ++report->tracedSegments;
            const RrVec3d start = projectedSample[sa];
            const RrVec3d target = projectedSample[sb];
            const double total = (target - start).GetLength();
            std::vector<RrGeoSubEdge> pending;
            int face = fa;
            int node = nodeOfSample[sa];
            RrVec3d position = start;
            double parameter = 0.0;
            bool arrived = false;
            for (int step = 0; step < 64; ++step) {
                if (face == fb) {
                    RrGeoSubEdge sub;
                    sub.face = face;
                    sub.node0 = node;
                    sub.node1 = nodeOfSample[sb];
                    sub.sampleA = sa;
                    sub.sampleB = sb;
                    sub.t0 = parameter;
                    sub.t1 = 1.0;
                    if (sub.node0 != sub.node1) {
                        pending.push_back(sub);
                    }
                    arrived = true;
                    break;
                }
                const RrGeoFaceFrame frame = RrGeoMakeFaceFrame(view, face);
                const RrVec2d from = frame.To2D(position);
                RrVec2d to = frame.To2D(target);
                RrVec2d direction = to - from;
                const double directionLength = direction.GetLength();
                if (directionLength <= RrGeoCurvenetEps) {
                    break;
                }
                direction /= directionLength;

                const int n = view.GetFaceSize(face);
                double bestT = std::numeric_limits<double>::max();
                int bestCorner = -1;
                double bestU = 0.0;
                for (int e = 0; e < n; ++e) {
                    const int va = view.GetFaceVertex(face, e);
                    const int vb = view.GetFaceVertex(face, (e + 1) % n);
                    const RrVec2d a = frame.To2D(view.points[va]);
                    const RrVec2d b = frame.To2D(view.points[vb]);
                    const RrVec2d edge = b - a;
                    const double denominator =
                        direction[0] * edge[1] - direction[1] * edge[0];
                    if (std::abs(denominator) <= 1e-14) {
                        continue;
                    }
                    const RrVec2d delta = a - from;
                    const double t =
                        (delta[0] * edge[1] - delta[1] * edge[0]) / denominator;
                    const double u = (delta[0] * direction[1] -
                                      delta[1] * direction[0]) /
                                     denominator;
                    if (t > 1e-9 && u >= -1e-9 && u <= 1.0 + 1e-9 &&
                        t < bestT) {
                        bestT = t;
                        bestCorner = e;
                        bestU = RrClamp(u, 0.0, 1.0);
                    }
                }
                if (bestCorner < 0) {
                    break;
                }
                const int va = view.GetFaceVertex(face, bestCorner);
                const int vb = view.GetFaceVertex(face, (bestCorner + 1) % n);
                const RrVec3d crossing =
                    view.points[va] * (1.0 - bestU) + view.points[vb] * bestU;
                const int next = adjacentFace(va, vb, face);
                if (next < 0) {
                    break;  // ran off an open boundary
                }
                const int crossingNode =
                    edgeNode(va, vb, bestU, crossing, -1);
                const double nextParameter =
                    (total > RrGeoCurvenetEps) ? RrClamp((crossing - start).GetLength() /
                                                 total,
                                             parameter, 1.0)
                                   : parameter;
                RrGeoSubEdge sub;
                sub.face = face;
                sub.node0 = node;
                sub.node1 = crossingNode;
                sub.sampleA = sa;
                sub.sampleB = sb;
                sub.t0 = parameter;
                sub.t1 = nextParameter;
                if (sub.node0 != sub.node1) {
                    pending.push_back(sub);
                }
                face = next;
                node = crossingNode;
                position = crossing;
                parameter = nextParameter;
            }
            if (arrived) {
                subEdges.insert(subEdges.end(), pending.begin(), pending.end());
            } else {
                ++report->failedTraces;
            }
        }
    }
    for (const AlongEdge &entry : alongEdge) {
        const std::pair<int, int> key = entry.edge;
        const double uA = entry.uFrom;
        const double uB = entry.uTo;
        if (uA < 0.0 || uB < 0.0 || std::abs(uB - uA) <= 1e-15) {
            continue;
        }
        const double lo = std::min(uA, uB);
        const double hi = std::max(uA, uB);

        std::vector<std::pair<double, int>> chain;
        if (nodeOfVertex[key.first] >= 0) {
            chain.push_back({0.0, nodeOfVertex[key.first]});
        }
        if (nodeOfVertex[key.second] >= 0) {
            chain.push_back({1.0, nodeOfVertex[key.second]});
        }
        const auto found = edgeNodes.find(key);
        if (found != edgeNodes.end()) {
            for (const auto &node : found->second) {
                chain.push_back(node);
            }
        }
        std::vector<std::pair<double, int>> inside;
        for (const auto &node : chain) {
            if (node.first >= lo - 1e-12 && node.first <= hi + 1e-12) {
                inside.push_back(node);
            }
        }
        std::sort(inside.begin(), inside.end(),
                  [](const std::pair<double, int> &a,
                     const std::pair<double, int> &b) {
                      return a.first < b.first;
                  });
        inside.erase(std::unique(inside.begin(), inside.end(),
                                 [](const std::pair<double, int> &a,
                                    const std::pair<double, int> &b) {
                                     return a.second == b.second;
                                 }),
                     inside.end());
        if (uB < uA) {
            std::reverse(inside.begin(), inside.end());
        }

        const auto faces = view.edgeFaces.find(key);
        if (faces == view.edgeFaces.end()) {
            continue;
        }
        const double tSpan = entry.tTo - entry.tFrom;
        for (size_t i = 0; i + 1 < inside.size(); ++i) {
            const double t0 =
                entry.tFrom + tSpan * (inside[i].first - uA) / (uB - uA);
            const double t1 =
                entry.tFrom + tSpan * (inside[i + 1].first - uA) / (uB - uA);
            for (const auto &incident : faces->second) {
                RrGeoSubEdge sub;
                sub.face = incident.first;
                sub.node0 = inside[i].second;
                sub.node1 = inside[i + 1].second;
                sub.sampleA = entry.sampleA;
                sub.sampleB = entry.sampleB;
                sub.t0 = RrClamp(t0, 0.0, 1.0);
                sub.t1 = RrClamp(t1, 0.0, 1.0);
                if (sub.node0 != sub.node1) {
                    subEdges.push_back(sub);
                }
            }
        }
    }

    if (report->failedTraces > 0) {
        char buffer[224];
        std::snprintf(buffer, sizeof(buffer),
                      "%d curvenet segment(s) could not be traced across the "
                      "surface and were not cut; the curve may leave the mesh "
                      "or cross an open boundary",
                      report->failedTraces);
        report->warnings.push_back(buffer);
    }

    std::vector<std::vector<int>> subEdgesOfFace(view.GetFaceCount());
    for (size_t i = 0; i < subEdges.size(); ++i) {
        subEdgesOfFace[subEdges[i].face].push_back(int(i));
    }

    cutMesh->faceBegin.push_back(0);
    int crackCount = 0;

    for (size_t f = 0; f < view.GetFaceCount(); ++f) {
        const int n = view.GetFaceSize(int(f));
        const RrGeoFaceFrame frame = RrGeoMakeFaceFrame(view, int(f));
        const size_t facesBefore = cutMesh->faceSource.size();

        std::unordered_map<int, int> localOf;
        std::vector<int> globalOf;
        auto local = [&](int global) {
            auto it = localOf.find(global);
            if (it != localOf.end()) {
                return it->second;
            }
            const int index = int(globalOf.size());
            localOf[global] = index;
            globalOf.push_back(global);
            return index;
        };

        struct LocalEdge {
            int a = -1;
            int b = -1;
            bool hasCurve = false;
            RrGeoCutCurveRef curve;
        };
        std::vector<LocalEdge> edges;
        std::map<std::pair<int, int>, int> edgeOf;
        auto addEdge = [&](int a, int b, const RrGeoCutCurveRef *curve) {
            if (a == b) {
                return;
            }
            const auto key = std::make_pair(std::min(a, b), std::max(a, b));
            auto it = edgeOf.find(key);
            if (it != edgeOf.end()) {
                if (curve && !edges[it->second].hasCurve) {
                    LocalEdge &existing = edges[it->second];
                    existing.hasCurve = true;
                    existing.curve = *curve;
                    if (existing.a != a) {
                        std::swap(existing.curve.tA, existing.curve.tB);
                    }
                }
                return;
            }
            LocalEdge edge;
            edge.a = a;
            edge.b = b;
            if (curve) {
                edge.hasCurve = true;
                edge.curve = *curve;
            }
            edgeOf[key] = int(edges.size());
            edges.push_back(edge);
        };

        for (int i = 0; i < n; ++i) {
            const int va = view.GetFaceVertex(int(f), i);
            const int vb = view.GetFaceVertex(int(f), (i + 1) % n);
            const auto key = edgeKey(va, vb);
            std::vector<std::pair<double, int>> chain;
            chain.push_back({0.0, nodeOfVertex[va]});
            chain.push_back({1.0, nodeOfVertex[vb]});
            const auto it = edgeNodes.find(key);
            if (it != edgeNodes.end()) {
                for (const auto &entry : it->second) {
                    const double u =
                        (va == key.first) ? entry.first : 1.0 - entry.first;
                    chain.push_back({u, entry.second});
                }
            }
            std::sort(chain.begin(), chain.end(),
                      [](const std::pair<double, int> &a,
                         const std::pair<double, int> &b) {
                          return a.first < b.first;
                      });
            for (size_t k = 0; k + 1 < chain.size(); ++k) {
                if (chain[k].second < 0 || chain[k + 1].second < 0) {
                    continue;
                }
                addEdge(local(chain[k].second), local(chain[k + 1].second),
                        nullptr);
            }
        }

        for (int index : subEdgesOfFace[f]) {
            const RrGeoSubEdge &sub = subEdges[index];
            RrGeoCutCurveRef ref;
            ref.sampleA = sub.sampleA;
            ref.sampleB = sub.sampleB;
            ref.tA = sub.t0;
            ref.tB = sub.t1;
            addEdge(local(sub.node0), local(sub.node1), &ref);
        }

        if (edges.empty()) {
            ++report->lostFaces;
            continue;
        }

        {
            const int localCount = int(globalOf.size());
            RrGeoDisjointSet sets(localCount);
            for (const LocalEdge &edge : edges) {
                sets.Union(edge.a, edge.b);
            }
            std::set<int> boundaryRoots;
            for (int i = 0; i < n; ++i) {
                const int node = nodeOfVertex[view.GetFaceVertex(int(f), i)];
                if (node >= 0 && localOf.count(node)) {
                    boundaryRoots.insert(sets.Find(localOf[node]));
                }
            }
            std::vector<LocalEdge> kept;
            kept.reserve(edges.size());
            for (const LocalEdge &edge : edges) {
                if (boundaryRoots.count(sets.Find(edge.a))) {
                    kept.push_back(edge);
                }
            }
            if (kept.size() != edges.size()) {
                edges.swap(kept);
                edgeOf.clear();
                for (size_t i = 0; i < edges.size(); ++i) {
                    edgeOf[{std::min(edges[i].a, edges[i].b),
                            std::max(edges[i].a, edges[i].b)}] = int(i);
                }
            }
        }

        const int halfCount = int(edges.size()) * 2;
        std::vector<int> origin(halfCount), destination(halfCount);
        for (size_t e = 0; e < edges.size(); ++e) {
            origin[2 * e] = edges[e].a;
            destination[2 * e] = edges[e].b;
            origin[2 * e + 1] = edges[e].b;
            destination[2 * e + 1] = edges[e].a;
        }
        std::vector<RrVec2d> planar(globalOf.size());
        for (size_t i = 0; i < globalOf.size(); ++i) {
            planar[i] = frame.To2D(nodes[globalOf[i]].position);
        }
        std::vector<std::vector<int>> outgoing(globalOf.size());
        for (int h = 0; h < halfCount; ++h) {
            outgoing[origin[h]].push_back(h);
        }
        std::vector<int> slotOf(halfCount, 0);
        for (size_t v = 0; v < outgoing.size(); ++v) {
            std::vector<int> &fan = outgoing[v];
            std::sort(fan.begin(), fan.end(), [&](int a, int b) {
                const RrVec2d da = planar[destination[a]] - planar[v];
                const RrVec2d db = planar[destination[b]] - planar[v];
                return std::atan2(da[1], da[0]) < std::atan2(db[1], db[0]);
            });
            for (size_t i = 0; i < fan.size(); ++i) {
                slotOf[fan[i]] = int(i);
            }
        }

        auto next = [&](int h) {
            const int twin = h ^ 1;
            const int v = origin[twin];
            const std::vector<int> &fan = outgoing[v];
            const int slot = slotOf[twin];
            return fan[(slot + fan.size() - 1) % fan.size()];
        };

        std::vector<char> visited(halfCount, 0);
        for (int start = 0; start < halfCount; ++start) {
            if (visited[start]) {
                continue;
            }
            std::vector<int> loop;
            int h = start;
            bool overrun = false;
            for (int guard = 0; guard <= halfCount + 1; ++guard) {
                visited[h] = 1;
                loop.push_back(h);
                h = next(h);
                if (h == start) {
                    break;
                }
                if (guard == halfCount) {
                    overrun = true;
                }
            }
            if (overrun || loop.size() < 3) {
                continue;
            }
            double area = 0.0;
            for (size_t i = 0; i < loop.size(); ++i) {
                const RrVec2d &a = planar[origin[loop[i]]];
                const RrVec2d &b = planar[destination[loop[i]]];
                area += a[0] * b[1] - b[0] * a[1];
            }
            if (area <= 0.0) {
                continue;
            }

            for (size_t i = 0; i < loop.size(); ++i) {
                const int corner = loop[i];
                const int previous = loop[(i + loop.size() - 1) % loop.size()];
                const int node = globalOf[origin[corner]];
                cutMesh->cornerNode.push_back(node);

                const LocalEdge &ownEdge = edges[corner / 2];
                const LocalEdge &previousEdge = edges[previous / 2];
                const bool ownForward = (corner % 2) == 0;
                const bool previousForward = (previous % 2) == 0;

                int constraintBegin = int(cutMesh->cornerConstraintIndex.size());
                cutMesh->cornerConstraintBegin.push_back(constraintBegin);

                auto emit = [&](const RrGeoCutCurveRef &ref, double t,
                                bool leftSide, double scale) {
                    const int sideOffset = leftSide ? 0 : 1;
                    if (ref.sampleA >= 0) {
                        cutMesh->cornerConstraintIndex.push_back(
                            2 * ref.sampleA + sideOffset);
                        cutMesh->cornerConstraintWeight.push_back(
                            (1.0 - t) * scale);
                    }
                    if (ref.sampleB >= 0) {
                        cutMesh->cornerConstraintIndex.push_back(
                            2 * ref.sampleB + sideOffset);
                        cutMesh->cornerConstraintWeight.push_back(t * scale);
                    }
                };

                if (ownEdge.hasCurve) {
                    const double t = ownForward ? ownEdge.curve.tA
                                                : ownEdge.curve.tB;
                    emit(ownEdge.curve, t, ownForward, 1.0);
                } else if (previousEdge.hasCurve) {
                    const double t = previousForward ? previousEdge.curve.tB
                                                     : previousEdge.curve.tA;
                    emit(previousEdge.curve, t, previousForward, 1.0);
                } else if (nodes[node].sample >= 0) {
                    RrGeoCutCurveRef ref;
                    ref.sampleA = nodes[node].sample;
                    ref.sampleB = -1;
                    emit(ref, 0.0, true, 0.5);
                    emit(ref, 0.0, false, 0.5);
                }
            }
            cutMesh->faceBegin.push_back(int(cutMesh->cornerNode.size()));
            cutMesh->faceSource.push_back(int(f));
            for (int hh : loop) {
                if (std::find(loop.begin(), loop.end(), hh ^ 1) != loop.end()) {
                    ++crackCount;
                }
            }
        }
        if (cutMesh->faceSource.size() == facesBefore) {
            ++report->lostFaces;
        }
    }
    if (report->lostFaces > 0) {
        char buffer[224];
        std::snprintf(buffer, sizeof(buffer),
                      "%d input face(s) produced no cut-face; the surface "
                      "loses its Laplacian support there",
                      report->lostFaces);
        report->warnings.push_back(buffer);
    }
    if (view.meanEdge > 0.0 && report->maxResidual > 5.0 * view.meanEdge) {
        char buffer[256];
        std::snprintf(
            buffer, sizeof(buffer),
            "the curvenet floats up to %.4g from the surface, %.1fx the mean "
            "edge length %.4g -- profiles are meant to lie near the surface, "
            "and a net this far off cuts degenerately",
            report->maxResidual, report->maxResidual / view.meanEdge,
            view.meanEdge);
        report->warnings.push_back(buffer);
    }
    cutMesh->cornerConstraintBegin.push_back(
        int(cutMesh->cornerConstraintIndex.size()));
    report->crackCount = crackCount / 2;
    report->cutFaceCount = int(cutMesh->GetCutFaceCount());
    if (cutMesh->GetCutFaceCount() == 0) {
        return fail("cutting the target by this curvenet produced no faces");
    }

    cutMesh->nodePosition.resize(nodes.size());
    cutMesh->nodeBinding.resize(nodes.size());
    cutMesh->nodeMeshVertex.resize(nodes.size());
    cutMesh->nodeSample.resize(nodes.size());
    for (size_t i = 0; i < nodes.size(); ++i) {
        cutMesh->nodePosition[i] = nodes[i].position;
        cutMesh->nodeMeshVertex[i] = nodes[i].meshVertex;
        cutMesh->nodeSample[i] = nodes[i].sample;
        RrGeoMeshPoint binding;
        binding.face = nodes[i].face;
        binding.indexOffset =
            (nodes[i].face >= 0) ? view.faceBegin[nodes[i].face] : 0;
        binding.cornerBegin = int(cutMesh->bindingWeights.size());
        binding.cornerCount = int(nodes[i].weights.size());
        for (double w : nodes[i].weights) {
            cutMesh->bindingWeights.push_back(float(w));
        }
        cutMesh->nodeBinding[i] = binding;
    }
    for (size_t s = 0; s < sampleCount; ++s) {
        RrGeoMeshPoint binding;
        binding.face = faceOfSample[s];
        binding.indexOffset =
            (faceOfSample[s] >= 0) ? view.faceBegin[faceOfSample[s]] : 0;
        binding.cornerBegin = int(cutMesh->bindingWeights.size());
        binding.cornerCount = int(sampleWeights[s].size());
        for (double w : sampleWeights[s]) {
            cutMesh->bindingWeights.push_back(float(w));
        }
        cutMesh->sampleBinding[s] = binding;
    }
    cutMesh->constraintCount = int(2 * sampleCount);

    cutMesh->vertexUnknown.assign(meshPoints.size(), -1);
    cutMesh->vertexConstraintNode.assign(meshPoints.size(), -1);
    std::vector<char> candidate(meshPoints.size(), 0);
    for (size_t c = 0; c < cutMesh->cornerNode.size(); ++c) {
        if (cutMesh->cornerConstraintBegin[c] !=
            cutMesh->cornerConstraintBegin[c + 1]) {
            continue;
        }
        const int vertex = cutMesh->nodeMeshVertex[cutMesh->cornerNode[c]];
        if (vertex >= 0) {
            candidate[vertex] = 1;
        }
    }
    for (size_t i = 0; i < nodes.size(); ++i) {
        const int vertex = nodes[i].meshVertex;
        if (vertex >= 0 && nodes[i].sample >= 0) {
            cutMesh->vertexConstraintNode[vertex] = int(i);
        }
    }

    {
        RrGeoDisjointSet sets(int(meshPoints.size()));
        std::vector<char> constrained(meshPoints.size(), 0);
        for (size_t f = 0; f < cutMesh->GetCutFaceCount(); ++f) {
            const int begin = cutMesh->faceBegin[f];
            const int end = cutMesh->faceBegin[f + 1];
            int first = -1;
            bool touchesCurve = false;
            for (int c = begin; c < end; ++c) {
                if (cutMesh->cornerConstraintBegin[c] !=
                    cutMesh->cornerConstraintBegin[c + 1]) {
                    touchesCurve = true;
                }
                const int vertex = cutMesh->nodeMeshVertex[cutMesh->cornerNode[c]];
                if (vertex < 0 || !candidate[vertex]) {
                    continue;
                }
                if (first < 0) {
                    first = vertex;
                } else {
                    sets.Union(first, vertex);
                }
            }
            if (touchesCurve && first >= 0) {
                constrained[sets.Find(first)] = 1;
            }
        }
        std::vector<char> rootConstrained(meshPoints.size(), 0);
        for (size_t v = 0; v < meshPoints.size(); ++v) {
            if (constrained[v]) {
                rootConstrained[sets.Find(int(v))] = 1;
            }
        }
        for (size_t v = 0; v < meshPoints.size(); ++v) {
            if (candidate[v] && !rootConstrained[sets.Find(int(v))]) {
                candidate[v] = 0;
                cutMesh->unreachedVertices.push_back(int(v));
            }
        }
    }

    for (size_t v = 0; v < meshPoints.size(); ++v) {
        if (candidate[v]) {
            cutMesh->vertexUnknown[v] = int(cutMesh->unknownVertex.size());
            cutMesh->unknownVertex.push_back(int(v));
        }
    }
    cutMesh->unknownCount = int(cutMesh->unknownVertex.size());
    if (!cutMesh->unreachedVertices.empty()) {
        char buffer[224];
        std::snprintf(buffer, sizeof(buffer),
                      "%zu vertices lie in a mesh component this curvenet "
                      "does not reach; they are held at rest",
                      cutMesh->unreachedVertices.size());
        report->warnings.push_back(buffer);
    }

    cutMesh->cornerVertexUnknown.assign(cutMesh->cornerNode.size(), -1);
    for (size_t c = 0; c < cutMesh->cornerNode.size(); ++c) {
        if (cutMesh->cornerConstraintBegin[c] !=
            cutMesh->cornerConstraintBegin[c + 1]) {
            continue;  // curvenet takes precedence
        }
        const int vertex = cutMesh->nodeMeshVertex[cutMesh->cornerNode[c]];
        if (vertex >= 0) {
            cutMesh->cornerVertexUnknown[c] = cutMesh->vertexUnknown[vertex];
        }
    }

    return true;
}

void
RrGeoPolygonLaplacian(const std::vector<RrVec3d> &corners,
                     std::vector<double> *out)
{
    const int n = int(corners.size());
    out->assign(size_t(n) * size_t(n), 0.0);
    if (n < 3) {
        return;
    }

    RrVec3d area(0.0);
    for (int i = 0; i < n; ++i) {
        area += RrCross(corners[i], corners[(i + 1) % n]);
    }
    area *= 0.5;
    const double magnitude = area.GetLength();
    if (magnitude <= RrGeoCurvenetEps) {
        return;
    }
    const RrVec3d normal = area / magnitude;

    std::vector<RrVec3d> eta(n, RrVec3d(0.0));
    for (int i = 0; i < n; ++i) {
        const RrVec3d edge = corners[(i + 1) % n] - corners[i];
        eta[i] += edge * 0.5;
        eta[(i + 1) % n] += edge * 0.5;
    }
    std::vector<RrVec3d> gradient(n, RrVec3d(0.0));
    for (int j = 0; j < n; ++j) {
        gradient[j] = -RrCross(normal, eta[j]) / magnitude;
    }

    std::vector<double> q(size_t(n) * size_t(n), 0.0);
    for (int i = 0; i < n; ++i) {
        const RrVec3d edge = corners[(i + 1) % n] - corners[i];
        for (int j = 0; j < n; ++j) {
            double value = -RrDot(edge, gradient[j]);
            if (j == i) {
                value += -1.0;
            }
            if (j == (i + 1) % n) {
                value += 1.0;
            }
            q[size_t(i) * n + j] = value;
        }
    }

    const double lambda = 1.0;
    for (int r = 0; r < n; ++r) {
        for (int c = 0; c < n; ++c) {
            double value = magnitude * RrDot(gradient[r], gradient[c]);
            double qq = 0.0;
            for (int i = 0; i < n; ++i) {
                qq += q[size_t(i) * n + r] * q[size_t(i) * n + c];
            }
            (*out)[size_t(r) * n + c] = value + lambda * qq;
        }
    }
}

void
RrGeoAssembleCutSystem(const RrGeoCutMesh &cutMesh,
                      const std::vector<RrVec3d> &cornerPositions,
                      RrGeoSparseBuilder *matrix,
                      std::vector<double> *faceLaplacians,
                      std::vector<int> *faceLaplacianBegin)
{
    faceLaplacians->clear();
    faceLaplacianBegin->assign(1, 0);
    std::vector<RrVec3d> polygon;
    std::vector<double> local;
    for (size_t f = 0; f < cutMesh.GetCutFaceCount(); ++f) {
        const int begin = cutMesh.faceBegin[f];
        const int n = cutMesh.faceBegin[f + 1] - begin;
        polygon.clear();
        for (int i = 0; i < n; ++i) {
            polygon.push_back(cornerPositions[begin + i]);
        }
        RrGeoPolygonLaplacian(polygon, &local);
        faceLaplacians->insert(faceLaplacians->end(), local.begin(),
                               local.end());
        faceLaplacianBegin->push_back(int(faceLaplacians->size()));

        for (int a = 0; a < n; ++a) {
            const int rowVertex = cutMesh.cornerVertexUnknown[begin + a];
            if (rowVertex < 0) {
                continue;
            }
            for (int b = 0; b < n; ++b) {
                const int columnVertex =
                    cutMesh.cornerVertexUnknown[begin + b];
                if (columnVertex < 0) {
                    continue;
                }
                const double value = local[size_t(a) * n + b];
                if (value != 0.0) {
                    matrix->Add(rowVertex, columnVertex, value);
                }
            }
        }
    }
}

void
RrGeoAssembleCutRhs(const RrGeoCutMesh &cutMesh,
                   const std::vector<double> &faceLaplacians,
                   const std::vector<int> &faceLaplacianBegin,
                   const std::vector<double> &constraintValues,
                   const std::vector<double> &cornerOffsets,
                   int columns, std::vector<double> *rhs)
{
    const int unknowns = cutMesh.unknownCount;
    rhs->assign(size_t(unknowns) * size_t(std::max(columns, 0)), 0.0);
    if (columns <= 0) {
        return;
    }
    const size_t cornerCount = cutMesh.cornerNode.size();
    const bool haveOffsets = !cornerOffsets.empty();

    std::vector<double> known(cornerCount * size_t(columns), 0.0);
    for (size_t c = 0; c < cornerCount; ++c) {
        for (int col = 0; col < columns; ++col) {
            double value = 0.0;
            for (int p = cutMesh.cornerConstraintBegin[c];
                 p < cutMesh.cornerConstraintBegin[c + 1]; ++p) {
                const int index = cutMesh.cornerConstraintIndex[p];
                const double weight = cutMesh.cornerConstraintWeight[p];
                value += weight *
                         constraintValues[size_t(col) *
                                              size_t(cutMesh.constraintCount) +
                                          size_t(index)];
            }
            if (haveOffsets) {
                value -= cornerOffsets[size_t(col) * cornerCount + c];
            }
            known[c * size_t(columns) + size_t(col)] = value;
        }
    }

    for (size_t f = 0; f < cutMesh.GetCutFaceCount(); ++f) {
        const int begin = cutMesh.faceBegin[f];
        const int n = cutMesh.faceBegin[f + 1] - begin;
        const double *local = faceLaplacians.data() + faceLaplacianBegin[f];
        for (int a = 0; a < n; ++a) {
            const int row = cutMesh.cornerVertexUnknown[begin + a];
            if (row < 0) {
                continue;
            }
            for (int b = 0; b < n; ++b) {
                const double value = local[size_t(a) * n + b];
                if (value == 0.0) {
                    continue;
                }
                const size_t corner = size_t(begin + b);
                for (int col = 0; col < columns; ++col) {
                    (*rhs)[size_t(col) * size_t(unknowns) + size_t(row)] -=
                        value * known[corner * size_t(columns) + size_t(col)];
                }
            }
        }
    }
}

struct RrGeoMeshSurfaceQuery {
    RrGeoMeshView view;
    RrGeoTriangleGrid grid;

    RrGeoMeshSurfaceQuery(const std::vector<RrVec3f> &meshPoints,
                         const std::vector<int> &faceVertexCounts,
                         const std::vector<int> &faceVertexIndices)
    {
        view = RrGeoBuildMeshView(meshPoints, faceVertexCounts,
                                  faceVertexIndices);
        grid = RrGeoBuildTriangleGrid(view);
    }

    RrVec3d Normal(const RrVec3d &point) const
    {
        const RrGeoProjection hit = RrGeoClosestPoint(view, grid, point);
        if (hit.face < 0) {
            return RrVec3d(0.0);
        }
        return view.faceNormal[hit.face];
    }
};

struct RrGeoProfileBinding {
    RrGeoCurvenetTopology topology;
    std::vector<RrVec3f> projectionMeshPoints;
    std::vector<int> faceVertexCounts;
    std::vector<int> faceVertexIndices;
    std::vector<int> samplesPerSpline;
    RrGeoCurvenetSampling projectionSampling;
    RrGeoCutMesh cutMesh;
    RrGeoCutMeshReport report;
    std::vector<double> faceLaplacians;
    std::vector<int> faceLaplacianBegin;
    std::shared_ptr<RrGeoSparseCholesky> solver;

    bool IsValid() const
    {
        return solver && solver->IsFactored();
    }
};

void
RrGeoFlattenMat3(const RrMat3d &m, double *out, size_t stride)
{
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            out[size_t(r * 3 + c) * stride] = m[r][c];
        }
    }
}

RrMat3d
RrGeoUnflattenMat3(const double *values)
{
    RrMat3d m;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            m[r][c] = values[r * 3 + c];
        }
    }
    return m;
}

bool
RrGeoBindProfileMover(const RrGeoCurvenetTopology &topology,
                      const std::vector<RrVec3f> &curvenetPoints,
                      const std::vector<RrVec3f> &meshPoints,
                      const std::vector<int> &faceVertexCounts,
                      const std::vector<int> &faceVertexIndices,
                      int samplesPerSpline, RrGeoProfileBinding *binding,
                      std::string *error)
{
    *binding = RrGeoProfileBinding();
    binding->topology = topology;
    binding->projectionMeshPoints = meshPoints;
    binding->faceVertexCounts = faceVertexCounts;
    binding->faceVertexIndices = faceVertexIndices;

    {
        const RrGeoMeshSurfaceQuery query(meshPoints, faceVertexCounts,
                                          faceVertexIndices);
        RrGeoOrientCurvenetIntersections(
            curvenetPoints,
            [&query](const RrVec3d &p) { return query.Normal(p); },
            &binding->topology);
    }
    const RrGeoCurvenetTopology &oriented = binding->topology;

    const double meanEdge = RrGeoMeshMeanEdgeLength(
        meshPoints, faceVertexCounts, faceVertexIndices);
    binding->samplesPerSpline = RrGeoPlanCurvenetSamples(
        oriented, curvenetPoints, meanEdge, samplesPerSpline);
    binding->projectionSampling = RrGeoSampleCurvenet(
        oriented, curvenetPoints, binding->samplesPerSpline);
    if (binding->projectionSampling.GetSampleCount() < 2) {
        if (error) {
            *error = "the curvenet produced fewer than two samples; it has no "
                     "usable curves";
        }
        return false;
    }

    {
        const RrGeoCurvenetFrames frames = RrGeoComputeCurvenetFrames(
            oriented, binding->projectionSampling);
        std::string reason;
        if (!RrGeoCurvenetFramesAreValid(frames, &reason)) {
            if (error) {
                *error = "curvenet is degenerate in the projection pose: " +
                         reason;
            }
            return false;
        }
    }

    if (!RrGeoBuildCutMesh(oriented, binding->projectionSampling, meshPoints,
                           faceVertexCounts, faceVertexIndices,
                           &binding->cutMesh, &binding->report, error)) {
        return false;
    }

    std::vector<RrVec3d> cornerPositions(binding->cutMesh.cornerNode.size());
    for (size_t c = 0; c < cornerPositions.size(); ++c) {
        cornerPositions[c] =
            binding->cutMesh.nodePosition[binding->cutMesh.cornerNode[c]];
    }
    RrGeoSparseBuilder matrix(binding->cutMesh.unknownCount);
    RrGeoAssembleCutSystem(binding->cutMesh, cornerPositions, &matrix,
                           &binding->faceLaplacians,
                           &binding->faceLaplacianBegin);

    binding->solver = std::make_shared<RrGeoSparseCholesky>();
    if (!binding->solver->Factorize(matrix, 0.0, error)) {
        binding->solver.reset();
        return false;
    }
    return true;
}

bool
RrGeoEvaluateProfileMover(
    const RrGeoProfileBinding &binding,
    const std::vector<RrVec3f> &posedCurvenetPoints,
    const std::vector<RrVec3f> &restMeshPoints, double strength,
    std::vector<RrVec3f> *outPoints, std::string *error)
{
    if (!binding.IsValid()) {
        if (error) {
            *error = "the curvenet binding was never built";
        }
        return false;
    }
    const RrGeoCutMesh &cut = binding.cutMesh;
    const size_t vertexCount = binding.projectionMeshPoints.size();
    if (restMeshPoints.size() != vertexCount) {
        if (error) {
            *error = "the incoming points array has " +
                     std::to_string(restMeshPoints.size()) +
                     " entries but the curvenet was bound against " +
                     std::to_string(vertexCount);
        }
        return false;
    }

    bool layered = false;
    for (size_t v = 0; v < vertexCount && !layered; ++v) {
        layered = (restMeshPoints[v] != binding.projectionMeshPoints[v]);
    }

    RrGeoCurvenetSampling restSampling = binding.projectionSampling;
    std::vector<RrVec3d> restCorner(cut.cornerNode.size());

    if (!layered) {
        for (size_t c = 0; c < restCorner.size(); ++c) {
            restCorner[c] = cut.nodePosition[cut.cornerNode[c]];
        }
    } else {
        std::vector<RrVec3d> warpedNode(cut.GetNodeCount());
        for (size_t n = 0; n < warpedNode.size(); ++n) {
            warpedNode[n] = cut.EvaluateBinding(
                cut.nodeBinding[n], restMeshPoints, binding.faceVertexCounts,
                binding.faceVertexIndices);
        }
        for (size_t c = 0; c < restCorner.size(); ++c) {
            restCorner[c] = warpedNode[cut.cornerNode[c]];
        }
        for (size_t s = 0; s < cut.sampleBinding.size(); ++s) {
            const RrVec3d projected = cut.EvaluateBinding(
                cut.sampleBinding[s], restMeshPoints, binding.faceVertexCounts,
                binding.faceVertexIndices);
            restSampling.positions[s] = projected + cut.sampleResidual[s];
        }
    }

    const RrGeoCurvenetSampling posedSampling = RrGeoSampleCurvenet(
        binding.topology, posedCurvenetPoints, binding.samplesPerSpline);
    if (posedSampling.GetSampleCount() != restSampling.GetSampleCount()) {
        if (error) {
            *error = "the posed curvenet sampled to a different number of "
                     "points than the bind did";
        }
        return false;
    }
    const RrGeoCurvenetFrames restFrames =
        RrGeoComputeCurvenetFrames(binding.topology, restSampling);
    const RrGeoCurvenetFrames posedFrames =
        RrGeoComputeCurvenetFrames(binding.topology, posedSampling);
    std::string reason;
    if (!RrGeoCurvenetFramesAreValid(posedFrames, &reason)) {
        if (error) {
            *error = "the posed curvenet is degenerate: " + reason;
        }
        return false;
    }
    const RrGeoCurvenetGradients gradients =
        RrGeoComputeCurvenetGradients(restFrames, posedFrames);
    const RrGeoCurvenetSampleGradients sampleGradients =
        RrGeoRemapGradientsToSamples(binding.topology, posedSampling,
                                     posedFrames, gradients);

    const int constraintCount = cut.constraintCount;
    const int unknowns = cut.unknownCount;
    std::vector<double> gradientConstraints(size_t(constraintCount) * 9, 0.0);
    for (size_t s = 0; s < sampleGradients.left.size(); ++s) {
        RrGeoFlattenMat3(sampleGradients.left[s],
                         gradientConstraints.data() + size_t(2 * s),
                         size_t(constraintCount));
        RrGeoFlattenMat3(sampleGradients.right[s],
                         gradientConstraints.data() + size_t(2 * s + 1),
                         size_t(constraintCount));
    }

    std::vector<double> rhs, gradientUnknowns;
    RrGeoAssembleCutRhs(cut, binding.faceLaplacians,
                        binding.faceLaplacianBegin, gradientConstraints,
                        std::vector<double>(), 9, &rhs);
    binding.solver->Solve(rhs, 9, &gradientUnknowns);

    const size_t cornerCount = cut.cornerNode.size();
    std::vector<double> cornerGradient(cornerCount * 9, 0.0);
    for (size_t c = 0; c < cornerCount; ++c) {
        const int unknown = cut.cornerVertexUnknown[c];
        if (unknown >= 0) {
            for (int k = 0; k < 9; ++k) {
                cornerGradient[c * 9 + k] =
                    gradientUnknowns[size_t(k) * size_t(unknowns) +
                                     size_t(unknown)];
            }
            continue;
        }
        for (int p = cut.cornerConstraintBegin[c];
             p < cut.cornerConstraintBegin[c + 1]; ++p) {
            const int index = cut.cornerConstraintIndex[p];
            const double weight = cut.cornerConstraintWeight[p];
            for (int k = 0; k < 9; ++k) {
                cornerGradient[c * 9 + k] +=
                    weight * gradientConstraints[size_t(k) *
                                                     size_t(constraintCount) +
                                                 size_t(index)];
            }
        }
    }

    std::vector<double> cornerOffsets(cornerCount * 3, 0.0);
    std::vector<RrVec3d> cornerTarget(cornerCount, RrVec3d(0.0));
    for (size_t f = 0; f < cut.GetCutFaceCount(); ++f) {
        const int begin = cut.faceBegin[f];
        const int end = cut.faceBegin[f + 1];
        const int n = end - begin;
        if (n <= 0) {
            continue;
        }
        double average[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
        for (int c = begin; c < end; ++c) {
            for (int k = 0; k < 9; ++k) {
                average[k] += cornerGradient[size_t(c) * 9 + k];
            }
        }
        RrMat3d faceGradient;
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                faceGradient[r][c] = average[r * 3 + c] / double(n);
            }
        }
        for (int c = begin; c < end; ++c) {
            const RrVec3d transformed = faceGradient * restCorner[c];
            for (int k = 0; k < 3; ++k) {
                cornerOffsets[size_t(k) * cornerCount + size_t(c)] =
                    transformed[k];
            }
        }
    }

    std::vector<double> positionConstraints(size_t(constraintCount) * 3, 0.0);
    for (size_t s = 0; s < posedSampling.GetSampleCount(); ++s) {
        for (int side = 0; side < 2; ++side) {
            const size_t index = 2 * s + size_t(side);
            for (int k = 0; k < 3; ++k) {
                positionConstraints[size_t(k) * size_t(constraintCount) +
                                    index] = posedSampling.positions[s][k];
            }
        }
    }

    for (size_t c = 0; c < cornerCount; ++c) {
        if (cut.cornerVertexUnknown[c] >= 0 ||
            cut.cornerConstraintBegin[c] == cut.cornerConstraintBegin[c + 1]) {
            continue;
        }
        RrVec3d posed(0.0), rest(0.0);
        for (int p = cut.cornerConstraintBegin[c];
             p < cut.cornerConstraintBegin[c + 1]; ++p) {
            const int index = cut.cornerConstraintIndex[p];
            const double weight = cut.cornerConstraintWeight[p];
            const size_t sample = size_t(index) / 2;
            posed += posedSampling.positions[sample] * weight;
            rest += restSampling.positions[sample] * weight;
        }
        const RrMat3d gradient =
            RrGeoUnflattenMat3(cornerGradient.data() + c * 9);
        cornerTarget[c] = posed - gradient * (rest - restCorner[c]);
        for (int k = 0; k < 3; ++k) {
            cornerOffsets[size_t(k) * cornerCount + c] +=
                posed[k] - cornerTarget[c][k];
        }
    }

    std::vector<double> positionUnknowns;
    RrGeoAssembleCutRhs(cut, binding.faceLaplacians,
                        binding.faceLaplacianBegin, positionConstraints,
                        cornerOffsets, 3, &rhs);
    binding.solver->Solve(rhs, 3, &positionUnknowns);

    outPoints->assign(restMeshPoints.begin(), restMeshPoints.end());
    for (int u = 0; u < unknowns; ++u) {
        const int vertex = cut.unknownVertex[u];
        RrVec3d p;
        for (int k = 0; k < 3; ++k) {
            p[k] = positionUnknowns[size_t(k) * size_t(unknowns) + size_t(u)];
        }
        if (!std::isfinite(p[0]) || !std::isfinite(p[1]) ||
            !std::isfinite(p[2])) {
            if (error) {
                *error = "the curvenet solve produced a non-finite position "
                         "for mesh vertex " + std::to_string(vertex) +
                         "; the cut left that region unsupported";
            }
            return false;
        }
        (*outPoints)[vertex] = RrGeoToVec3f(p);
    }
    {
        std::vector<RrVec3d> accumulated(vertexCount, RrVec3d(0.0));
        std::vector<int> counted(vertexCount, 0);
        for (size_t c = 0; c < cut.cornerNode.size(); ++c) {
            const int vertex = cut.nodeMeshVertex[cut.cornerNode[c]];
            if (vertex < 0 || cut.vertexUnknown[vertex] >= 0 ||
                cut.cornerConstraintBegin[c] ==
                    cut.cornerConstraintBegin[c + 1]) {
                continue;
            }
            accumulated[vertex] += cornerTarget[c];
            ++counted[vertex];
        }
        for (size_t v = 0; v < vertexCount; ++v) {
            if (counted[v] > 0) {
                (*outPoints)[v] = RrGeoToVec3f(accumulated[v] / double(counted[v]));
            }
        }
    }

    if (strength != 1.0) {
        const double s = strength;
        for (size_t v = 0; v < vertexCount; ++v) {
            const RrVec3d rest = RrGeoToVec3d(restMeshPoints[v]);
            const RrVec3d moved = RrGeoToVec3d((*outPoints)[v]);
            (*outPoints)[v] = RrGeoToVec3f(rest + (moved - rest) * s);
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// Curvenet adjustments (curvenetAdjustments.*): adjustment frames from a
// rest/posed pair, then each command's local transform about its frame.
// ---------------------------------------------------------------------------

constexpr double RrGeoAdjustEps = 1e-12;

bool
RrGeoAdjustFail(std::string *error, const char *why) {
    if (error) *error = why;
    return false;
}

bool
RrGeoAdjustFinite(const RrVec3f &p) {
    return std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]);
}

RrQuatd
RrGeoAdjustBetween(const RrVec3d &a, const RrVec3d &b) {
    if (a.GetLengthSq() < RrGeoAdjustEps || b.GetLengthSq() < RrGeoAdjustEps)
        return RrQuatd::GetIdentity();
    return RrRotation(a.GetNormalized(), b.GetNormalized()).GetQuat();
}

RrMat4d
RrGeoAdjustFrame(const RrQuatd &q, const RrVec3f &p) {
    RrMat4d frame;
    frame.SetIdentity();
    frame.SetRotate(q);
    frame.SetTranslateOnly(RrGeoToVec3d(p));
    return frame;
}

RrQuatd
RrGeoAdjustBestFit(const std::vector<RrVec3d> &from,
                   const std::vector<RrVec3d> &to) {
    double b[3][3]{};
    int count = 0;
    size_t firstValid = 0;
    for (size_t i = 0; i < from.size(); ++i) {
        if (from[i].GetLengthSq() < RrGeoAdjustEps || to[i].GetLengthSq() < RrGeoAdjustEps) continue;
        const auto a = from[i].GetNormalized(), c = to[i].GetNormalized();
        if (!count) firstValid = i;
        for (int r = 0; r < 3; ++r)
            for (int s = 0; s < 3; ++s) b[r][s] += a[r] * c[s];
        ++count;
    }
    if (!count) return RrQuatd::GetIdentity();
    const double tr = b[0][0] + b[1][1] + b[2][2];
    double n[4][4] = {
        {tr, b[1][2]-b[2][1], b[2][0]-b[0][2], b[0][1]-b[1][0]},
        {b[1][2]-b[2][1], 2*b[0][0]-tr, b[0][1]+b[1][0], b[0][2]+b[2][0]},
        {b[2][0]-b[0][2], b[0][1]+b[1][0], 2*b[1][1]-tr, b[1][2]+b[2][1]},
        {b[0][1]-b[1][0], b[0][2]+b[2][0], b[1][2]+b[2][1], 2*b[2][2]-tr}};
    double v[4][4]{};
    for (int i = 0; i < 4; ++i) v[i][i] = 1;
    for (int sweep = 0; sweep < 32; ++sweep) {
        double residual = 0;
        for (int p = 0; p < 4; ++p) for (int q = p+1; q < 4; ++q) {
            residual += std::abs(n[p][q]);
            if (std::abs(n[p][q]) < 1e-15) continue;
            const double tau = (n[q][q]-n[p][p])/(2*n[p][q]);
            const double t = std::copysign(1.0, tau)/
                (std::abs(tau) + std::sqrt(1+tau*tau));
            const double c = 1/std::sqrt(1+t*t), s = t*c;
            const double off = n[p][q];
            n[p][p] -= t*off;
            n[q][q] += t*off;
            n[p][q] = n[q][p] = 0;
            for (int k = 0; k < 4; ++k) {
                if (k != p && k != q) {
                    const double x=n[k][p], y=n[k][q];
                    n[k][p]=n[p][k]=c*x-s*y;
                    n[k][q]=n[q][k]=s*x+c*y;
                }
                const double x=v[k][p], y=v[k][q];
                v[k][p]=c*x-s*y; v[k][q]=s*x+c*y;
            }
        }
        if (residual < 1e-13) break;
    }
    int largest = 0;
    for (int i = 1; i < 4; ++i) if (n[i][i] > n[largest][largest]) largest=i;
    RrQuatd result(v[0][largest], RrVec3d(v[1][largest],v[2][largest],v[3][largest]));
    result.Normalize();
    bool collinear = true;
    for (size_t i=0; i<from.size(); ++i)
        if (RrCross(from[firstValid],from[i]).GetLengthSq() > RrGeoAdjustEps) collinear=false;
    return collinear ? RrGeoAdjustBetween(from[firstValid],to[firstValid]) : result;
}

std::vector<RrVec3d>
RrGeoAdjustTangents(const RrGeoCurvenetSampling &samples, size_t c) {
    const int begin=samples.curveBegin[c], count=samples.GetCurveSampleCount(c);
    std::vector<RrVec3d> tangents(count);
    for (int i=0; i<count; ++i) {
        tangents[i] = samples.positions[begin+std::min(i+1,count-1)] -
                      samples.positions[begin+std::max(i-1,0)];
    }
    return tangents;
}

std::vector<RrQuatd>
RrGeoAdjustTransport(const std::vector<RrVec3d> &rest,
                    const std::vector<RrVec3d> &posed, RrQuatd seed,
                    bool reverse) {
    std::vector<RrQuatd> result(rest.size());
    int i=reverse ? int(rest.size())-1 : 0;
    const int step=reverse ? -1 : 1;
    result[i]=seed;
    for (int j=i+step; j>=0 && j<int(rest.size()); j+=step) {
        seed=(RrGeoAdjustBetween(posed[i],posed[j])*seed*
              RrGeoAdjustBetween(rest[i],rest[j]).GetInverse()).GetNormalized();
        result[j]=seed; i=j;
    }
    return result;
}

std::set<int>
RrGeoAdjustHandles(const RrGeoCurvenetTopology &topology, int knot) {
    std::set<int> handles;
    if (topology.basis != RrGeoCurvenetBasisBezier) return handles;
    for (size_t s=0; s<topology.GetSplineCount(); ++s) {
        if (topology.GetSplineStartKnot(s)==knot) handles.insert(topology.splineIndices[4*s+1]);
        if (topology.GetSplineEndKnot(s)==knot) handles.insert(topology.splineIndices[4*s+2]);
    }
    return handles;
}

bool
RrGeoComputeCurvenetAdjustmentFrames(
    const RrGeoCurvenetTopology &topology,
    const std::vector<RrVec3f> &rest, const std::vector<RrVec3f> &posed,
    std::vector<RrMat4d> *frames, std::string *error) {
    if (!frames || rest.size()!=posed.size() || rest.size()!=size_t(topology.pointCount))
        return RrGeoAdjustFail(error,"curvenet adjustment point cardinality mismatch");
    for (size_t i=0; i<rest.size(); ++i)
        if (!RrGeoAdjustFinite(rest[i]) || !RrGeoAdjustFinite(posed[i])) return RrGeoAdjustFail(error,"nonfinite curvenet point");
    const std::vector<int> density(topology.GetSplineCount(), 16);
    const auto a=RrGeoSampleCurvenet(topology,rest,density);
    const auto b=RrGeoSampleCurvenet(topology,posed,density);
    std::vector<std::vector<RrVec3d>> tangentsA(a.GetCurveCount()), tangentsB(b.GetCurveCount());
    for (size_t c=0; c<a.GetCurveCount(); ++c) {
        if (a.GetCurveSampleCount(c)<2 || a.GetCurveSampleCount(c)!=b.GetCurveSampleCount(c))
            return RrGeoAdjustFail(error,"degenerate curvenet adjustment curve");
        tangentsA[c]=RrGeoAdjustTangents(a,c); tangentsB[c]=RrGeoAdjustTangents(b,c);
    }
    std::vector<RrQuatd> rotations(rest.size(),RrQuatd::GetIdentity());
    for (const auto &intersection:topology.intersections) {
        std::vector<RrVec3d> from,to;
        for (const auto &spoke:intersection.spokes) {
            if (topology.basis == RrGeoCurvenetBasisBezier) {
                const auto &curve = topology.curves[spoke.curve];
                const int spline = spoke.atCurveEnd ? curve.splines.back() : curve.splines.front();
                const int handle = topology.splineIndices[4*spline +
                    (topology.GetSplineStartKnot(spline)==intersection.knot ? 1 : 2)];
                from.push_back(RrGeoToVec3d(rest[handle]-rest[intersection.knot]));
                to.push_back(RrGeoToVec3d(posed[handle]-posed[intersection.knot]));
                continue;
            }
            const auto &x=tangentsA[spoke.curve], &y=tangentsB[spoke.curve];
            from.push_back(spoke.atCurveEnd ? -x.back() : x.front());
            to.push_back(spoke.atCurveEnd ? -y.back() : y.front());
        }
        rotations[intersection.knot]=RrGeoAdjustBestFit(from,to);
    }
    for (size_t c=0; c<a.GetCurveCount(); ++c) {
        const auto &curve=topology.curves[c];
        const auto fallback=RrGeoAdjustBestFit(tangentsA[c],tangentsB[c]);
        const auto left=RrGeoAdjustTransport(tangentsA[c],tangentsB[c],
            curve.startIsIntersection ? rotations[curve.startKnot] : fallback,false);
        const auto right=RrGeoAdjustTransport(tangentsA[c],tangentsB[c],
            curve.endIsIntersection ? rotations[curve.endKnot] : fallback,true);
        std::vector<double> distance(left.size(),0.0);
        const int begin=b.curveBegin[c];
        for (size_t i=1; i<distance.size(); ++i)
            distance[i]=distance[i-1]+(b.positions[begin+i]-b.positions[begin+i-1]).GetLength();
        if (distance.back()<RrGeoAdjustEps) return RrGeoAdjustFail(error,"collapsed curvenet adjustment curve");
        for (size_t i=0; i<distance.size(); ++i) {
            const int knot=a.knotOfSample[begin+i];
            if (knot<0 || topology.knotKinds[knot]==RrGeoKnotIntersection) continue;
            rotations[knot]=curve.startIsIntersection && curve.endIsIntersection
                ? RrSlerp(distance[i]/distance.back(),left[i],right[i]).GetNormalized()
                : (curve.endIsIntersection ? right[i] : left[i]);
        }
    }
    std::vector<RrMat4d> result(rest.size());
    for (size_t i=0; i<rest.size(); ++i) result[i]=RrGeoAdjustFrame(rotations[i],posed[i]);
    *frames=std::move(result);
    return true;
}

bool
RrGeoApplyCurvenetAdjustments(
    std::vector<RrVec3f> *points, const std::vector<RrVec3f> &rest,
    const std::vector<int> &indices, RrGeoCurvenetBasis basis,
    const std::vector<RrGeoAdjustmentCommand> &commands,
    std::vector<RrMat4d> *commandFrames, std::string *error) {
    if (!points) return RrGeoAdjustFail(error,"missing curvenet adjustment points");
    RrGeoCurvenetTopology topology;
    if (!RrGeoBuildCurvenetTopology(indices,rest.size(),basis,rest,nullptr,&topology,error)) return false;
    std::vector<RrMat4d> frames;
    if (!RrGeoComputeCurvenetAdjustmentFrames(topology,rest,*points,&frames,error)) return false;
    auto output=*points;
    std::vector<RrMat4d> adjustedFrames;
    std::set<int> commanded;
    for (size_t i=0; i<commands.size(); ++i) {
        const auto &command=commands[i];
        const int index=command.pointIndex;
        if (index<0 || size_t(index)>=rest.size() || !commanded.insert(index).second)
            return RrGeoAdjustFail(error,"invalid or duplicate adjustment point index");
        for (int r=0; r<4; ++r) for (int c=0; c<4; ++c)
            if (!std::isfinite(command.localTransform[r][c])) return RrGeoAdjustFail(error,"nonfinite adjustment transform");
        if (command.localTransform[0][3]!=0 || command.localTransform[1][3]!=0 ||
            command.localTransform[2][3]!=0 || command.localTransform[3][3]!=1)
            return RrGeoAdjustFail(error,"adjustment transform must be affine");
        RrMat4d frame=frames[index];
        std::set<int> affected{index};
        if (command.parentCommand>=0) {
            if (size_t(command.parentCommand)>=i || commands[command.parentCommand].parentCommand>=0 ||
                !RrGeoAdjustHandles(topology,commands[command.parentCommand].pointIndex).count(index))
                return RrGeoAdjustFail(error,"tangent adjustment must follow its incident knot parent");
            frame=adjustedFrames[command.parentCommand];
            frame.SetTranslateOnly(RrGeoToVec3d(output[index]));
        } else {
            if (topology.knotValence[index]==0)
                return RrGeoAdjustFail(error,"knot adjustment must name a curve endpoint");
            if (command.includeTangents) {
                const auto handles=RrGeoAdjustHandles(topology,index);
                affected.insert(handles.begin(),handles.end());
            }
        }
        const RrMat4d transform=frame.GetInverse()*command.localTransform*frame;
        for (const int point:affected) {
            if (command.localTransform==RrGeoIdentity()) continue;
            output[point]=RrGeoToVec3f(transform.Transform(RrGeoToVec3d(output[point])));
            if (!RrGeoAdjustFinite(output[point])) return RrGeoAdjustFail(error,"adjustment produced a nonfinite point");
        }
        adjustedFrames.push_back(command.localTransform*frame);
    }
    points->swap(output);
    if (commandFrames) *commandFrames=std::move(adjustedFrames);
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
    RrMat4d m = transform * space.GetInverse();
    m[0][3] = 0.0;
    m[1][3] = 0.0;
    m[2][3] = 0.0;
    m[3][3] = 1.0;
    return m;
}

bool
RrGeoApplyRevisionKernel(
    int op, const RrGeoMoverParameters &p, std::vector<RrVec3f> *pts,
    std::vector<RrMat4d> *controlFrames,
    std::unordered_map<uint64_t, RrGeoWireBasisEntry> *wireCache)
{
    switch (op) {
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
    case RrGeoOpCurvenetAdjuster: {
        if (!controlFrames) {
            return false;  // the adjuster's whole second output
        }
        const auto preceding = *pts;
        if (!RrGeoApplyCurvenetAdjustments(pts, p.restPoints,
                                           p.topologyIndices,
                                           p.curvenetAdjustmentBasis,
                                           p.curvenetAdjustments,
                                           controlFrames, nullptr)) {
            return false;
        }
        for (size_t i = 0; i < controlFrames->size(); ++i) {
            const int point = p.curvenetAdjustments[i].pointIndex;
            const float weight = p.weights.Resolve(point, pts->size());
            (*controlFrames)[i].SetTranslateOnly(RrGeoToVec3d(
                preceding[point] + ((*pts)[point]-preceding[point])*weight));
        }
        return true;
    }
    case RrGeoOpCurvenet: {
        if (!p.curvenetBinding) {
            return false;
        }
        std::vector<RrVec3f> solved;
        std::string error;
        if (!RrGeoEvaluateProfileMover(*p.curvenetBinding,
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
    case RrGeoOpRecomputeNormals:
    case RrGeoOpRecomputeExtent:
        return RrGeoApplyDerivedKernel(op, p, pts);
    }
    return false;
}

bool
RrGeoRunRevisionKernel(
    int op, const RrGeoMoverParameters &p, std::vector<RrVec3f> *pts,
    std::vector<RrMat4d> *controlFrames,
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
        return RrGeoApplyRevisionKernel(op, p, pts, controlFrames, wireCache);
    }

    const bool fullStrengthEnvelope = RrGeoEnvelopeIsFullStrength(p.weights);
    const size_t precedingSize = pts->size();
    std::vector<RrVec3f> preceding;
    if (!fullStrengthEnvelope) {
        preceding = *pts;
    }
    if (!RrGeoApplyRevisionKernel(op, p, pts, controlFrames, wireCache)) {
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

// ---------------------------------------------------------------------------
// Scratch: revision packets, influence tables, output buffers, chunks, skin
// topologies, blend layouts, curvenet binds, the pathReads lookup and the
// per-frame record pointer. Mirrors GeomChain/GeomRevision/GeomChunk.
// ---------------------------------------------------------------------------

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
        std::vector<RrMat4d> controlFrames;
        std::shared_ptr<const RrGeoProfileBinding> curvenetBind;
        bool curvenetBindResolved = false;
        std::string bindDigest;
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

RrGeoWeightPacket
RrGeoWirePacket(const RrProgram *program,
                const RigExecWireWeightPacket &packet)
{
    RrGeoWeightPacket out;
    out.representation = program->TextOrEmpty(packet.representation);
    out.rangePolicy = program->TextOrEmpty(packet.rangePolicy);
    out.values = packet.values;
    out.indices.assign(packet.indices.begin(), packet.indices.end());
    out.defaultWeight = packet.defaultWeight;
    out.valid = packet.valid;
    return out;
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

// A typed stage read, replayed: the property-chain overlay first (exact
// type, as RigExecResolvedInputs::GetAttribute checks it), then the
// recorded stage value, else the fallback. Array sites take the phase
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
    if (path != 0) {
        const auto it = program->store.propertyResults.find(path);
        if (it != program->store.propertyResults.end() &&
            it->second.tag == RrPropertyValue::Tag::Float) {
            return it->second.f32;
        }
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
    if (path != 0) {
        const auto it = program->store.propertyResults.find(path);
        if (it != program->store.propertyResults.end() &&
            it->second.tag == RrPropertyValue::Tag::Double) {
            return it->second.f64;
        }
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

    program->geo = std::move(scratch);
    return true;
}

namespace {

// ---------------------------------------------------------------------------
// Step plumbing: the frame record, step labels, mover-attribute paths.
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// Assembly: one revision's packet out of the record, the path reads and the
// epoch tables. A port of AssembleRevision plus the RigExecAssemble*
// per-operation bodies it ends in.
// ---------------------------------------------------------------------------

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

// inputs:defaultWeight as RevisionStatic read it, which is what the record
// carries for every main revision -- and the same value the assembly's own
// resolved read answers, because a phase overlay holds no scalars.
float
RrGeoAssembleDefaultWeight(const RrGeoAssembleInputs &in)
{
    if (!in.record || in.derived ||
        in.chain >= in.record->revisionDefaultWeights.size() ||
        in.revision >= in.record->revisionDefaultWeights[in.chain].size()) {
        return 1.0f;
    }
    return in.record->revisionDefaultWeights[in.chain][in.revision];
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

// The blend channels out of the record: the weights, activations and dense
// points the baked gather consumed, in binding order, each channel's
// samples stable-sorted by activation. Sparse samples take the refused
// layout when this frame refused the cache and the epoch one otherwise.
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
        if (record) {
            if (!in.derived && in.chain < record->blendWeights.size() &&
                in.revision < record->blendWeights[in.chain].size() &&
                c < record->blendWeights[in.chain][in.revision].size()) {
                channel.weight =
                    record->blendWeights[in.chain][in.revision][c];
            } else if (in.derived &&
                       in.chain < record->derivedBlendWeights.size() &&
                       in.revision <
                           record->derivedBlendWeights[in.chain].size() &&
                       c < record->derivedBlendWeights[in.chain][in.revision]
                               .size()) {
                channel.weight = record->derivedBlendWeights[in.chain]
                                                             [in.revision][c];
            }
        }
        for (size_t s = 0; s < bound.samples.size(); ++s) {
            const RigExecWireBlendChannel::Sample &boundSample =
                bound.samples[s];
            RrGeoBlendSampleData sample;
            if (record) {
                const std::vector<std::vector<std::vector<
                    std::vector<float>>>> *acts = nullptr;
                const std::vector<std::vector<std::vector<std::vector<
                    std::vector<RigExecWireVec3f>>>>> *pts = nullptr;
                if (!in.derived) {
                    acts = &record->blendActivations;
                    pts = &record->blendPoints;
                } else {
                    acts = &record->derivedBlendActivations;
                    pts = &record->derivedBlendPoints;
                }
                if (in.chain < acts->size() &&
                    in.revision < (*acts)[in.chain].size() &&
                    c < (*acts)[in.chain][in.revision].size() &&
                    s < (*acts)[in.chain][in.revision][c].size()) {
                    sample.activation =
                        (*acts)[in.chain][in.revision][c][s];
                }
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
        params->auxPoints.resize(params->restPoints.size());
        for (size_t j = 0; j < params->restPoints.size(); ++j) {
            RrVec3f &rest = params->restPoints[j];
            if (bt > 0) {
                RrMat4d b = table[t + s + pick(bt, j)];
                if (bs > 0) {
                    b = RrGeoMeasureInSpace(
                        b, table[t + s + bt + pick(bs, j)]);
                }
                const float wb = baseWeights.empty()
                    ? 1.0f
                    : baseWeights[pick(baseWeights.size(), j)];
                const RrVec3f moved = RrGeoToVec3f(
                    b.TransformAffine(RrGeoToVec3d(rest)));
                rest = rest + (moved - rest) * wb;
            }
            RrMat4d m = table[pick(t, j)];
            if (s > 0) {
                m = RrGeoMeasureInSpace(m, table[t + pick(s, j)]);
            }
            const float w = weights.empty()
                ? 1.0f
                : weights[pick(weights.size(), j)];
            const RrVec3f moved = RrGeoToVec3f(
                m.TransformAffine(RrGeoToVec3d(rest)));
            params->auxPoints[j] = rest + (moved - rest) * w;
        }
    } else {
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
RrGeoAssembleCurvenet(const RrGeoAssembleInputs &in,
                       RrGeoMoverParameters *params)
{
    params->strength = 1.0f;
    if (in.wire->binding.curvenet == 0) {
        return;
    }
    std::vector<RrVec3f> restNet;
    RrGeoReadBindingVec3fArray(in, in.wire->binding.curvenetPoints, true,
                               &restNet);
    RrGeoReadIntArray(in.scratch, in.wire->binding.topologyCounts, true,
                      &params->topologyCounts);
    RrGeoReadIntArray(in.scratch, in.wire->binding.topologyIndices, true,
                      &params->topologyIndices);
    RrGeoReadBindingVec3fArray(in, in.wire->binding.base, true,
                               &params->restPoints);
    if (restNet.empty() || params->topologyCounts.empty() ||
        params->restPoints.empty()) {
        return;
    }
    // The posed net: this run's net-chain result when the curvenet has a
    // chain of its own. The authored-at-time fallback the baked path
    // reads off the stage is never recorded, so a net with no chain
    // cannot replay here. (Framework need if a rig ever hits it.)
    params->auxPoints.clear();
    if (in.wire->curvenetChain >= 0 &&
        size_t(in.wire->curvenetChain) < in.scratch->chains.size()) {
        const RrGeometryScratch::Chain &net =
            in.scratch->chains[size_t(in.wire->curvenetChain)];
        if (!net.lastBase.empty() && net.haveResult) {
            params->auxPoints = net.result;
        }
    }
    if (params->auxPoints.size() != restNet.size()) {
        return;
    }
    // The bind the prologue cut; resolved there because the bind takes
    // no lock where a step assembles.
    if (in.rev->curvenetBindResolved) {
        params->curvenetBinding = in.rev->curvenetBind;
        params->valid = params->curvenetBinding != nullptr;
    }
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
    if (op == RrGeoOpCurvenetAdjuster) {
        // The adjuster's commands, paths and target type are not on the
        // wire, so no packet for one can assemble here; a disabled one
        // still passes through silently. (Framework need: serialize the
        // adjustment prims' commands.)
        params.kind = RrGeoKindToken(RrGeoOpCurvenetAdjuster);
        params.enabled = RrGeoAssembleEnabled(in);
        params.weights = RrGeoAssembleEnvelope(in, false);
        if (!params.enabled) {
            params.valid = true;
        }
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
    case RrGeoOpCurvenet:
        RrGeoAssembleCurvenet(in, &params);
        break;
    case RrGeoOpRecomputeNormals:
        RrGeoAssembleDerived(in, base, false, &params);
        break;
    case RrGeoOpRecomputeExtent:
        RrGeoAssembleDerived(in, base, true, &params);
        break;
    default:
        break;
    }
    return params;
}

// ---------------------------------------------------------------------------
// The fold: influence tables out of the pose half's matrices. A port of
// FoldInfluences, FoldTransformForms, GatherChunkTransforms and SkinRange.
// ---------------------------------------------------------------------------

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
    if (rev->haveTransform && wire.transformSpaceSlot >= 0) {
        const RrMat4d *space = nullptr;
        if (!matrixAt(wire.transformSpaceSlot, &space)) {
            if (error) {
                *error = "transform space slot names no provider matrix";
            }
            return false;
        }
        rev->transform = RrGeoMeasureInSpace(rev->transform, *space);
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

// ---------------------------------------------------------------------------
// The partition re-cut, for the frame an epoch-fixed layout moved under.
// A port of RigExecBakedPartitionRevision.
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// The Profile Mover re-bind: the same build the baked prologue runs, from
// the retained bind inputs, reporting the same line on a (re)bind.
// ---------------------------------------------------------------------------

std::string
RrGeoBindDigest(const RigExecWireRevision &wire)
{
    // The baked digest hashes every bind input; the runtime only needs to
    // know whether they moved, so the inputs themselves are the digest.
    // (All epoch state, so after the first frame this never changes.)
    std::string digest;
    digest.append(reinterpret_cast<const char *>(wire.curvenetRestNet.data()),
                 wire.curvenetRestNet.size() * sizeof(RigExecWireVec3f));
    digest.append(
        reinterpret_cast<const char *>(wire.curvenetSplineIndices.data()),
        wire.curvenetSplineIndices.size() * sizeof(int32_t));
    digest.append(
        reinterpret_cast<const char *>(&wire.curvenetSamplesPerSpline),
        sizeof(wire.curvenetSamplesPerSpline));
    digest.append(reinterpret_cast<const char *>(&wire.curvenetBasis),
                  sizeof(wire.curvenetBasis));
    digest.append(reinterpret_cast<const char *>(wire.curvenetMeshPoints.data()),
                 wire.curvenetMeshPoints.size() * sizeof(RigExecWireVec3f));
    digest.append(reinterpret_cast<const char *>(wire.curvenetMeshCounts.data()),
                 wire.curvenetMeshCounts.size() * sizeof(int32_t));
    digest.append(
        reinterpret_cast<const char *>(wire.curvenetMeshIndices.data()),
        wire.curvenetMeshIndices.size() * sizeof(int32_t));
    return digest;
}

void
RrGeoRebindCurvenet(RrProgram *program, const RigExecWireRevision &wire,
                    RrGeometryScratch::Revision *rev)
{
    const std::string digest = RrGeoBindDigest(wire);
    if (rev->curvenetBindResolved && rev->bindDigest == digest) {
        // The cache still applies: same inputs, same handle. A fresh
        // bind every frame would move the packet's pointer and mark
        // every frame executed.
        return;
    }
    rev->curvenetBindResolved = false;
    rev->curvenetBind.reset();
    const bool firstBind = rev->bindDigest.empty() || rev->bindDigest != digest;
    rev->bindDigest = digest;
    if (!wire.curvenetBindInputsHeld) {
        // The bake never held this revision's inputs: a remembered
        // failure, exactly as the baked cache remembers one.
        rev->curvenetBindResolved = true;
        return;
    }
    std::vector<RrVec3f> restNet;
    restNet.reserve(wire.curvenetRestNet.size());
    for (const RigExecWireVec3f &p : wire.curvenetRestNet) {
        restNet.push_back(RrVec3f(p[0], p[1], p[2]));
    }
    std::vector<int> splineIndices(wire.curvenetSplineIndices.begin(),
                                   wire.curvenetSplineIndices.end());
    std::vector<RrVec3f> meshPoints;
    meshPoints.reserve(wire.curvenetMeshPoints.size());
    for (const RigExecWireVec3f &p : wire.curvenetMeshPoints) {
        meshPoints.push_back(RrVec3f(p[0], p[1], p[2]));
    }
    std::vector<int> meshCounts(wire.curvenetMeshCounts.begin(),
                                wire.curvenetMeshCounts.end());
    std::vector<int> meshIndices(wire.curvenetMeshIndices.begin(),
                                 wire.curvenetMeshIndices.end());
    const RrGeoCurvenetBasis basis =
        program->TokenEquals(wire.curvenetBasis, "catmullRom")
            ? RrGeoCurvenetBasisCatmullRom
            : RrGeoCurvenetBasisBezier;
    auto bound = std::make_shared<RrGeoProfileBinding>();
    std::string why;
    {
        RrGeoCurvenetTopology topology;
        if (!RrGeoBuildCurvenetTopology(splineIndices, restNet.size(),
                                       basis, restNet, nullptr, &topology,
                                       &why) ||
            !RrGeoBindProfileMover(topology, restNet, meshPoints,
                                   meshCounts, meshIndices,
                                   int(wire.curvenetSamplesPerSpline),
                                   bound.get(), &why)) {
            bound.reset();
        }
    }
    rev->curvenetBind = bound;
    rev->curvenetBindResolved = true;
    if (bound && firstBind) {
        char summary[320];
        std::snprintf(
            summary, sizeof(summary),
            "curvenet bind %s -> %s: %d cut faces, %d samples, %d cracks, "
            "%d traced, %d unknowns, %zu factor nonzeros",
            program->TextOrEmpty(wire.moverPath).c_str(),
            program->TextOrEmpty(wire.target).c_str(),
            bound->report.cutFaceCount, bound->report.sampleCount,
            bound->report.crackCount, bound->report.tracedSegments,
            bound->cutMesh.unknownCount,
            bound->solver ? bound->solver->GetFactorNonzeros() : size_t(0));
        program->store.curvenetBindDiagnostics.push_back(summary);
        for (const std::string &warning : bound->report.warnings) {
            program->store.curvenetBindDiagnostics.push_back(
                "curvenet bind " + program->TextOrEmpty(wire.moverPath) +
                ": " + warning);
        }
    }
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
            if (wire.op == uint8_t(RrGeoOpCurvenet)) {
                RrGeoRebindCurvenet(program, wire, &rev);
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
                                   nullptr, &scratch->wireBasis);
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
    if (wire.weightCurrentPhase && wire.weightObject >= 0 &&
        record && chainIndex < record->revisionPhasePackets.size() &&
        revisionIndex <
            record->revisionPhasePackets[chainIndex].size()) {
        // The current-phase field the bake measured; a packet that came
        // back invalid failed its resolve, which the baked step reports
        // with the oracle's own words -- words the wire does not carry,
        // so a failing resolve replays its packet but not its line.
        rev.currentPhasePacket = RrGeoWirePacket(
            program,
            record->revisionPhasePackets[chainIndex][revisionIndex]);
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
                &rev.controlFrames, &scratch->wireBasis);
            const int id = wireStep.object;
            if (id >= 0 && size_t(id) < store.revisionPublish.size()) {
                store.revisionPublish[size_t(id)].controlFrames =
                    rev.controlFrames;
            }
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
