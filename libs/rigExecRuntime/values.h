// rigExecRuntime shared value types (M2 framework).
// Zero-USD mirrors of the pose values the step bodies pass around:
// point frames, input values, weight packets and per-step outputs.
// Family .cpps build their private state from these; the store owns the
// framework-visible instances.
#ifndef RIGEXEC_RUNTIME_VALUES_H
#define RIGEXEC_RUNTIME_VALUES_H

#include "rigExecBinary/wireTypes.h"
#include "rigExecRuntime/runtimeMath.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <map>
#include <string>
#include <vector>

namespace rigExec {

// The four identity landmarks a frame is measured against. Initialised at
// load rather than as a function-local static, whose first-use guard every
// call would test.
inline const std::array<RrVec3d, 4> kRrIdentityLandmarks = {
    RrVec3d(0.0), RrVec3d(1.0, 0.0, 0.0), RrVec3d(0.0, 1.0, 0.0),
    RrVec3d(0.0, 0.0, 1.0)};

inline const std::array<RrVec3d, 4> &
RrIdentityLandmarks()
{
    return kRrIdentityLandmarks;
}

// Mirrors RigExecPointFrameFlags bit for bit.
enum RrPointFrameFlags : uint32_t {
    RrPointFrameValid = 1u << 0,
    RrPointFrameDegenerate = 1u << 1,
    RrPointFrameReflected = 1u << 2,
    RrPointFrameAffine = 1u << 3,
    RrPointFrameLiveRest = 1u << 4,
};

// Mirrors RigExecPointFrame: points[0] = O, [1] = X, [2] = Y, [3] = Z.
struct RrPointFrame {
    std::array<RrVec3d, 4> points{RrVec3d(0.0),
                                  RrVec3d(1.0, 0.0, 0.0),
                                  RrVec3d(0.0, 1.0, 0.0),
                                  RrVec3d(0.0, 0.0, 1.0)};
    uint32_t flags = RrPointFrameValid;

    bool operator==(const RrPointFrame &o) const
    {
        return points == o.points && flags == o.flags;
    }
    bool operator!=(const RrPointFrame &o) const { return !(*this == o); }

    bool IsValid() const { return (flags & RrPointFrameValid) != 0; }
    bool IsDegenerate() const
    {
        return (flags & RrPointFrameDegenerate) != 0;
    }
};

inline RrPointFrame
RrWireToFrame(const RigExecWireFrame &frame)
{
    RrPointFrame out;
    for (size_t i = 0; i < 4; ++i) {
        out.points[i] = RrVec3d(frame.points[i][0], frame.points[i][1],
                                frame.points[i][2]);
    }
    out.flags = frame.flags;
    return out;
}

// Mirrors RigExecBakedUsable: the commit gate of the pose walk.
inline bool
RrFrameUsable(const RrPointFrame &frame)
{
    if (!frame.IsValid() || frame.IsDegenerate()) {
        return false;
    }
    for (const RrVec3d &point : frame.points) {
        if (!std::isfinite(point[0]) || !std::isfinite(point[1]) ||
            !std::isfinite(point[2])) {
            return false;
        }
    }
    return true;
}

// Mirrors RigExecPointFrameArray: one solver's aggregate boundary.
struct RrPointFrameArray {
    std::vector<RrPointFrame> frames;
    std::vector<std::array<RrVec3d, 4>> rests;

    bool operator==(const RrPointFrameArray &o) const
    {
        return frames == o.frames && rests == o.rests;
    }
    bool operator!=(const RrPointFrameArray &o) const
    {
        return !(*this == o);
    }
};

// The value type of an input, a read or a constant, numbered as the
// file's InputTag. The array tags hold int32_t, float, double, float[2]
// and float[3] elements.
enum class RrInputTag : uint8_t {
    Double = 0,
    Float = 1,
    Bool = 2,
    Int = 3,
    Matrix4d = 4,
    Token = 5,
    Vec3d = 6,
    Vec3f = 7,
    IntArray = 8,
    FloatArray = 9,
    DoubleArray = 10,
    Vec2fArray = 11,
    Vec3fArray = 12,
    Vec3dArray = 13, Matrix4dArray = 14, TokenArray = 15, BoolArray = 16, Vec3i = 17,
};

inline bool
RrInputTagIsArray(RrInputTag tag)
{
    return uint8_t(tag) >= uint8_t(RrInputTag::IntArray) &&
           uint8_t(tag) <= uint8_t(RrInputTag::BoolArray);
}

// A view of an array input's elements: `count` elements of the type `tag`
// names (int32_t, float, double, float[2] or float[3]) at `data`. A set
// copies them; the reader never keeps the caller's pointer.
struct RigExecRuntimeArray {
    RrInputTag tag = RrInputTag::FloatArray;
    const void *data = nullptr;
    size_t count = 0;
};

// A resolved input value: a read the slots answered, a wire constant, or
// an input's value. The member `tag` names holds the value; Token is a
// path id of the file (or an id the reader interned for unknown text).
struct RrInputValue {
    RrInputTag tag = RrInputTag::Double;
    double f64 = 0;
    float f32 = 0;
    bool boolean = false;
    int32_t i32 = 0;
    RrMat4d matrix;
    uint32_t token = 0;
    RrVec3d vec = RrVec3d(0.0);
    RrVec3f vec3f = RrVec3f(0.0f);
    RrVec3i vec3i = RrVec3i(0);

    bool operator==(const RrInputValue &o) const
    {
        return tag == o.tag && f64 == o.f64 && f32 == o.f32 &&
               boolean == o.boolean && i32 == o.i32 && matrix == o.matrix &&
               token == o.token && vec == o.vec && vec3f == o.vec3f && vec3i == o.vec3i;
    }
    bool operator!=(const RrInputValue &o) const { return !(*this == o); }
};

// One input of a .rigexec: an attribute whose authored value the rig reads.
struct RigExecRuntimeInputInfo {
    /// The attribute path, e.g. "/Rig/Controls/Hips.avars:tx".
    std::string name;
    /// The attribute's value type.
    RrInputTag type = RrInputTag::Double;
    /// Whether the attribute is time-varying in the baked stage.
    bool animated = false;
    /// Its value at the bake time; the type's zero when that read failed.
    /// An array input's carries the tag alone.
    RrInputValue defaultValue;
    /// An array input's default element count; 0 for a scalar.
    size_t defaultCount = 0;
};

// Mirrors RigExecWeightPacket with token path ids.
struct RrWeightPacket {
    uint32_t representation = 0;
    uint32_t rangePolicy = 0;
    std::vector<float> values;
    std::vector<int32_t> indices;
    float defaultWeight = 0;
    bool valid = false;

    bool operator==(const RrWeightPacket &o) const
    {
        return representation == o.representation &&
               rangePolicy == o.rangePolicy && values == o.values &&
               indices == o.indices &&
               defaultWeight == o.defaultWeight && valid == o.valid;
    }
    bool operator!=(const RrWeightPacket &o) const
    {
        return !(*this == o);
    }
};

// One property-chain result: the four types _EvaluatePropertyChains
// instantiates, keyed by path id.
struct RrPropertyValue {
    enum class Tag : uint8_t {
        Float = 0,
        Double = 1,
        Matrix4d = 2,
        Vec3f = 3,
    };
    Tag tag = Tag::Float;
    float f32 = 0;
    double f64 = 0;
    RrMat4d matrix;
    RrVec3f vec = RrVec3f(0.0f);

    bool operator==(const RrPropertyValue &o) const
    {
        return tag == o.tag && f32 == o.f32 && f64 == o.f64 &&
               matrix == o.matrix && vec == o.vec;
    }
    bool operator!=(const RrPropertyValue &o) const
    {
        return !(*this == o);
    }
};

// Mirrors RigExecBakedStepCounters.
struct RrStepCounters {
    uint32_t revisionsExecuted = 0;
    uint32_t revisionsCreated = 0;
    uint32_t schedulesBuilt = 0;
    uint32_t chainsBuilt = 0;
    uint32_t revisionsBuilt = 0;

    void Clear() { *this = RrStepCounters(); }
};

// One step's run output: diagnostics, counters, bail.
struct RrStepOutput {
    std::vector<std::string> diagnostics;
    RrStepCounters counters;

    void BeginRun()
    {
        diagnostics.clear();
        counters.Clear();
    }

    void MarkSkipped()
    {
        counters.revisionsExecuted = 0;
        counters.revisionsCreated = 0;
        counters.schedulesBuilt = 0;
    }
};

// A constraint's authored arrays, as the bake read them.
struct RrConstraintArraysLive {
    std::vector<double> weights;
    std::vector<RrVec3d> translationOffsets, rotationOffsets;
    std::vector<std::string> diagnostics;
    bool ok = true;
    bool readPole = false;
    std::vector<double> poleWeights;
    std::vector<std::string> poleDiagnostics;
    bool poleOk = true;
};

// Mirrors RigExecConstraintSource: one resolved constraint source.
struct RrConstraintSource {
    RrPointFrame frame;
    double normalizedWeight = 1.0;
    RrVec3d translationOffset = RrVec3d(0.0);
    RrVec3d rotationOffsetDegrees = RrVec3d(0.0);
};

// One commit's per-run scratch, mirroring RigExecBakedCommit's run state.
struct RrCommitScratch {
    std::vector<char> present;
    std::vector<RrPointFrame> frames;
    std::vector<RrMat4d> deltas;
    std::vector<char> deltaOk;
    std::vector<RrPointFrame> staged;
    std::vector<uint8_t> outcome;
    bool abandoned = true;
    std::vector<RrConstraintSource> sources;
    std::vector<RrPointFrame> ikChain, ikPrepared, ikRest, ikSolved;
};

}  // namespace rigExec

#endif  // RIGEXEC_RUNTIME_VALUES_H
