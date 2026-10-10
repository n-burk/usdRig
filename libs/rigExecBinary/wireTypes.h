// Plain value types of the .rigexec format: the math aliases and small PODs
// the wire structs hold, and the native types the FlatBuffers schema's
// structs map onto (rigExecBinary/format.h). USD-free.
#ifndef RIGEXEC_BINARY_WIRE_TYPES_H
#define RIGEXEC_BINARY_WIRE_TYPES_H

#include <array>
#include <cstdint>
#include <utility>

namespace rigExec {

// Wire math (mirrors GfVec3d/GfVec3f/GfMatrix4d/RigExecPointFrame fieldwise).

using RigExecWireVec3d = std::array<double, 3>;
using RigExecWireVec3f = std::array<float, 3>;
using RigExecWireVec2f = std::array<float, 2>;
using RigExecWireVec3i = std::array<int32_t, 3>;
/// Row-major, m[r][c] at [r * 4 + c].
using RigExecWireMatrix4d = std::array<double, 16>;
/// Point-frame landmarks: the origin, then the X, Y and Z tips.
using RigExecWireLandmarks = std::array<RigExecWireVec3d, 4>;
using RigExecWireIntPair = std::pair<int32_t, int32_t>;
using RigExecWireUIntPair = std::pair<uint32_t, uint32_t>;
using RigExecWireBool3 = std::array<bool, 3>;

struct RigExecWirePropertyInputCandidate {
    uint32_t slot = 0;
    uint8_t kind = 0;
    int32_t version = -1;
    bool raw = false;
    int32_t poseWeight = -1;
    int32_t crossDomain = -1;
    bool operator==(const RigExecWirePropertyInputCandidate &other) const {
        return slot == other.slot && kind == other.kind &&
               version == other.version && raw == other.raw && poseWeight == other.poseWeight && crossDomain == other.crossDomain;
    }
};

struct RigExecWireFrame {
    RigExecWireLandmarks points{};
    uint32_t flags = 0;
};

/// A resolved read phase: the kind plus the prim AtPrim names (0 else).
struct RigExecWireReadPhase {
    /// Base/Preceding/Final/AtPrim, in RigExecReadPhaseKind order.
    uint8_t kind = 0;
    uint32_t prim = 0;
};

/// One slot above a native Xformable source, and the versions live where
/// the commit runs.
struct RigExecWireAncestorRead {
    int32_t slot = -1;
    uint32_t fin = 0;
    uint32_t base = 0;
};

struct RigExecWireConstraintSource {
    RigExecWireFrame frame;
    double normalizedWeight = 1;
    RigExecWireVec3d translationOffset{};
    RigExecWireVec3d rotationOffsetDegrees{};
};

struct RigExecWireTwoBoneIkParams {
    double upperLength = 1;
    double lowerLength = 1;
    double stretch = 1;
    double softness = 0;
    double preferredBendRadians = 0;
};

struct RigExecWireSplineIkParams {
    double preserveVolume = 1;
    double midFollowWeight = 0.5;
    double roll = 0;
    double twist = 0;
    double minLengthRatio = 0;
    bool aimRootTangent = false;
};

}  // namespace rigExec

#endif  // RIGEXEC_BINARY_WIRE_TYPES_H
