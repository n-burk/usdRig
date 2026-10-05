// The .rigexec file format: one FlatBuffer, schema libs/rigExecBinary/
// rigexec.fbs, file identifier "REXB". The generated object API types live
// in namespace rigExec::fb with the RigExecWire prefix (RigExecWireFile,
// RigExecWireSolver, ...) and are aliased into namespace rigExec under the
// same names; the schema's math and POD structs map onto the value types
// of rigExecBinary/wireTypes.h through the Pack/UnPack pairs declared
// here. Open is the decoder and Write the encoder: there is no
// hand-written codec. USD-free.
#ifndef RIGEXEC_BINARY_FORMAT_H
#define RIGEXEC_BINARY_FORMAT_H

#include "rigExecBinary/wireTypes.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace rigExec {
namespace fb {
struct Vec3d;
struct Vec3f;
struct Vec2f;
struct Vec3i;
struct Matrix4d;
struct Landmarks;
struct Frame;
struct IntPair;
struct Bool3;
struct F64;
struct F32;
struct ReadPhase;
struct AncestorRead;
struct ConstraintSource;
struct TwoBoneIkParams;
struct SplineIkParams;
}  // namespace fb
}  // namespace rigExec

// The native-type conversions the generated code calls by name: Pack<Name>
// and UnPack<Name> per struct, plus an unsuffixed Pack for the structs a
// table holds as an optional field (Value, PathValue). Each copies the
// stored bits unchanged.
namespace flatbuffers {
inline rigExec::fb::Vec3d PackVec3d(const rigExec::RigExecWireVec3d &value);
inline rigExec::RigExecWireVec3d UnPackVec3d(const rigExec::fb::Vec3d &value);
inline rigExec::fb::Vec3f PackVec3f(const rigExec::RigExecWireVec3f &value);
inline rigExec::RigExecWireVec3f UnPackVec3f(const rigExec::fb::Vec3f &value);
inline rigExec::fb::Vec2f PackVec2f(const rigExec::RigExecWireVec2f &value);
inline rigExec::RigExecWireVec2f UnPackVec2f(const rigExec::fb::Vec2f &value);
inline rigExec::fb::Vec3i PackVec3i(const rigExec::RigExecWireVec3i &value);
inline rigExec::RigExecWireVec3i UnPackVec3i(const rigExec::fb::Vec3i &value);
inline rigExec::fb::Matrix4d
PackMatrix4d(const rigExec::RigExecWireMatrix4d &value);
inline rigExec::RigExecWireMatrix4d
UnPackMatrix4d(const rigExec::fb::Matrix4d &value);
inline rigExec::fb::Landmarks
PackLandmarks(const rigExec::RigExecWireLandmarks &value);
inline rigExec::RigExecWireLandmarks
UnPackLandmarks(const rigExec::fb::Landmarks &value);
inline rigExec::fb::Frame PackFrame(const rigExec::RigExecWireFrame &value);
inline rigExec::RigExecWireFrame UnPackFrame(const rigExec::fb::Frame &value);
inline rigExec::fb::IntPair
PackIntPair(const rigExec::RigExecWireIntPair &value);
inline rigExec::RigExecWireIntPair
UnPackIntPair(const rigExec::fb::IntPair &value);
inline rigExec::fb::Bool3 PackBool3(const rigExec::RigExecWireBool3 &value);
inline rigExec::RigExecWireBool3 UnPackBool3(const rigExec::fb::Bool3 &value);
inline rigExec::fb::F64 PackF64(const double &value);
inline double UnPackF64(const rigExec::fb::F64 &value);
inline rigExec::fb::F32 PackF32(const float &value);
inline float UnPackF32(const rigExec::fb::F32 &value);
inline rigExec::fb::ReadPhase
PackReadPhase(const rigExec::RigExecWireReadPhase &value);
inline rigExec::RigExecWireReadPhase
UnPackReadPhase(const rigExec::fb::ReadPhase &value);
inline rigExec::fb::AncestorRead
PackAncestorRead(const rigExec::RigExecWireAncestorRead &value);
inline rigExec::RigExecWireAncestorRead
UnPackAncestorRead(const rigExec::fb::AncestorRead &value);
inline rigExec::fb::ConstraintSource
PackConstraintSource(const rigExec::RigExecWireConstraintSource &value);
inline rigExec::RigExecWireConstraintSource
UnPackConstraintSource(const rigExec::fb::ConstraintSource &value);
inline rigExec::fb::TwoBoneIkParams
PackTwoBoneIkParams(const rigExec::RigExecWireTwoBoneIkParams &value);
inline rigExec::RigExecWireTwoBoneIkParams
UnPackTwoBoneIkParams(const rigExec::fb::TwoBoneIkParams &value);
inline rigExec::fb::SplineIkParams
PackSplineIkParams(const rigExec::RigExecWireSplineIkParams &value);
inline rigExec::RigExecWireSplineIkParams
UnPackSplineIkParams(const rigExec::fb::SplineIkParams &value);

inline rigExec::fb::Vec3d Pack(const rigExec::RigExecWireVec3d &value);
inline rigExec::fb::Vec3f Pack(const rigExec::RigExecWireVec3f &value);
inline rigExec::fb::Vec3i Pack(const rigExec::RigExecWireVec3i &value);
inline rigExec::fb::Matrix4d Pack(const rigExec::RigExecWireMatrix4d &value);
}  // namespace flatbuffers

#include "rigExecBinary/generated/rigexec_generated.h"

namespace flatbuffers {

inline rigExec::fb::Vec3d
PackVec3d(const rigExec::RigExecWireVec3d &value)
{
    return rigExec::fb::Vec3d(value[0], value[1], value[2]);
}

inline rigExec::RigExecWireVec3d
UnPackVec3d(const rigExec::fb::Vec3d &value)
{
    return {{value.x(), value.y(), value.z()}};
}

inline rigExec::fb::Vec3f
PackVec3f(const rigExec::RigExecWireVec3f &value)
{
    return rigExec::fb::Vec3f(value[0], value[1], value[2]);
}

inline rigExec::RigExecWireVec3f
UnPackVec3f(const rigExec::fb::Vec3f &value)
{
    return {{value.x(), value.y(), value.z()}};
}

inline rigExec::fb::Vec2f
PackVec2f(const rigExec::RigExecWireVec2f &value)
{
    return rigExec::fb::Vec2f(value[0], value[1]);
}

inline rigExec::RigExecWireVec2f
UnPackVec2f(const rigExec::fb::Vec2f &value)
{
    return {{value.x(), value.y()}};
}

inline rigExec::fb::Vec3i
PackVec3i(const rigExec::RigExecWireVec3i &value)
{
    return rigExec::fb::Vec3i(value[0], value[1], value[2]);
}

inline rigExec::RigExecWireVec3i
UnPackVec3i(const rigExec::fb::Vec3i &value)
{
    return {{value.x(), value.y(), value.z()}};
}

inline rigExec::fb::Matrix4d
PackMatrix4d(const rigExec::RigExecWireMatrix4d &value)
{
    return rigExec::fb::Matrix4d(
        ::flatbuffers::span<const double, 16>(value.data(), 16));
}

inline rigExec::RigExecWireMatrix4d
UnPackMatrix4d(const rigExec::fb::Matrix4d &value)
{
    rigExec::RigExecWireMatrix4d out;
    for (::flatbuffers::uoffset_t i = 0; i < 16; ++i) {
        out[i] = value.m()->Get(i);
    }
    return out;
}

inline rigExec::fb::Landmarks
PackLandmarks(const rigExec::RigExecWireLandmarks &value)
{
    const rigExec::fb::Vec3d points[4] = {
        PackVec3d(value[0]), PackVec3d(value[1]), PackVec3d(value[2]),
        PackVec3d(value[3])};
    return rigExec::fb::Landmarks(
        ::flatbuffers::span<const rigExec::fb::Vec3d, 4>(points, 4));
}

inline rigExec::RigExecWireLandmarks
UnPackLandmarks(const rigExec::fb::Landmarks &value)
{
    rigExec::RigExecWireLandmarks out;
    for (::flatbuffers::uoffset_t i = 0; i < 4; ++i) {
        out[i] = UnPackVec3d(*value.p()->Get(i));
    }
    return out;
}

inline rigExec::fb::Frame
PackFrame(const rigExec::RigExecWireFrame &value)
{
    const rigExec::fb::Vec3d points[4] = {
        PackVec3d(value.points[0]), PackVec3d(value.points[1]),
        PackVec3d(value.points[2]), PackVec3d(value.points[3])};
    return rigExec::fb::Frame(
        ::flatbuffers::span<const rigExec::fb::Vec3d, 4>(points, 4),
        value.flags);
}

inline rigExec::RigExecWireFrame
UnPackFrame(const rigExec::fb::Frame &value)
{
    rigExec::RigExecWireFrame out;
    for (::flatbuffers::uoffset_t i = 0; i < 4; ++i) {
        out.points[i] = UnPackVec3d(*value.p()->Get(i));
    }
    out.flags = value.flags();
    return out;
}

inline rigExec::fb::IntPair
PackIntPair(const rigExec::RigExecWireIntPair &value)
{
    return rigExec::fb::IntPair(value.first, value.second);
}

inline rigExec::RigExecWireIntPair
UnPackIntPair(const rigExec::fb::IntPair &value)
{
    return {value.first(), value.second()};
}

inline rigExec::fb::Bool3
PackBool3(const rigExec::RigExecWireBool3 &value)
{
    return rigExec::fb::Bool3(value[0], value[1], value[2]);
}

inline rigExec::RigExecWireBool3
UnPackBool3(const rigExec::fb::Bool3 &value)
{
    return {{value.x(), value.y(), value.z()}};
}

inline rigExec::fb::F64
PackF64(const double &value)
{
    return rigExec::fb::F64(value);
}

inline double
UnPackF64(const rigExec::fb::F64 &value)
{
    return value.v();
}

inline rigExec::fb::F32
PackF32(const float &value)
{
    return rigExec::fb::F32(value);
}

inline float
UnPackF32(const rigExec::fb::F32 &value)
{
    return value.v();
}

inline rigExec::fb::ReadPhase
PackReadPhase(const rigExec::RigExecWireReadPhase &value)
{
    return rigExec::fb::ReadPhase(value.kind, value.prim);
}

inline rigExec::RigExecWireReadPhase
UnPackReadPhase(const rigExec::fb::ReadPhase &value)
{
    rigExec::RigExecWireReadPhase out;
    out.kind = value.kind();
    out.prim = value.prim();
    return out;
}

inline rigExec::fb::AncestorRead
PackAncestorRead(const rigExec::RigExecWireAncestorRead &value)
{
    return rigExec::fb::AncestorRead(value.slot, value.fin, value.base);
}

inline rigExec::RigExecWireAncestorRead
UnPackAncestorRead(const rigExec::fb::AncestorRead &value)
{
    rigExec::RigExecWireAncestorRead out;
    out.slot = value.slot();
    out.fin = value.fin();
    out.base = value.base();
    return out;
}

inline rigExec::fb::ConstraintSource
PackConstraintSource(const rigExec::RigExecWireConstraintSource &value)
{
    return rigExec::fb::ConstraintSource(
        PackFrame(value.frame), value.normalizedWeight,
        PackVec3d(value.translationOffset),
        PackVec3d(value.rotationOffsetDegrees));
}

inline rigExec::RigExecWireConstraintSource
UnPackConstraintSource(const rigExec::fb::ConstraintSource &value)
{
    rigExec::RigExecWireConstraintSource out;
    out.frame = UnPackFrame(value.frame());
    out.normalizedWeight = value.normalizedWeight();
    out.translationOffset = UnPackVec3d(value.translationOffset());
    out.rotationOffsetDegrees = UnPackVec3d(value.rotationOffsetDegrees());
    return out;
}

inline rigExec::fb::TwoBoneIkParams
PackTwoBoneIkParams(const rigExec::RigExecWireTwoBoneIkParams &value)
{
    return rigExec::fb::TwoBoneIkParams(
        value.upperLength, value.lowerLength, value.stretch, value.softness,
        value.preferredBendRadians);
}

inline rigExec::RigExecWireTwoBoneIkParams
UnPackTwoBoneIkParams(const rigExec::fb::TwoBoneIkParams &value)
{
    rigExec::RigExecWireTwoBoneIkParams out;
    out.upperLength = value.upperLength();
    out.lowerLength = value.lowerLength();
    out.stretch = value.stretch();
    out.softness = value.softness();
    out.preferredBendRadians = value.preferredBendRadians();
    return out;
}

inline rigExec::fb::SplineIkParams
PackSplineIkParams(const rigExec::RigExecWireSplineIkParams &value)
{
    return rigExec::fb::SplineIkParams(
        value.preserveVolume, value.midFollowWeight, value.roll, value.twist,
        value.minLengthRatio, value.aimRootTangent);
}

inline rigExec::RigExecWireSplineIkParams
UnPackSplineIkParams(const rigExec::fb::SplineIkParams &value)
{
    rigExec::RigExecWireSplineIkParams out;
    out.preserveVolume = value.preserveVolume();
    out.midFollowWeight = value.midFollowWeight();
    out.roll = value.roll();
    out.twist = value.twist();
    out.minLengthRatio = value.minLengthRatio();
    out.aimRootTangent = value.aimRootTangent();
    return out;
}

inline rigExec::fb::Vec3d
Pack(const rigExec::RigExecWireVec3d &value)
{
    return PackVec3d(value);
}

inline rigExec::fb::Vec3f
Pack(const rigExec::RigExecWireVec3f &value)
{
    return PackVec3f(value);
}

inline rigExec::fb::Vec3i
Pack(const rigExec::RigExecWireVec3i &value)
{
    return PackVec3i(value);
}

inline rigExec::fb::Matrix4d
Pack(const rigExec::RigExecWireMatrix4d &value)
{
    return PackMatrix4d(value);
}

}  // namespace flatbuffers

namespace rigExec {

// The generated object types under the RigExecWire names the runtime, the
// bake and the tests use.
using RigExecWireFile = fb::RigExecWireFile;
using RigExecWireValue = fb::RigExecWireValue;
using RigExecWireIntArray = fb::RigExecWireIntArray;
using RigExecWireFloatArray = fb::RigExecWireFloatArray;
using RigExecWireDoubleArray = fb::RigExecWireDoubleArray;
using RigExecWireVec2fArray = fb::RigExecWireVec2fArray;
using RigExecWireVec3fArray = fb::RigExecWireVec3fArray;
using RigExecWireIntList = fb::RigExecWireIntList;
using RigExecWireUintList = fb::RigExecWireUintList;
using RigExecWireFloatList = fb::RigExecWireFloatList;
using RigExecWireDoubleList = fb::RigExecWireDoubleList;
using RigExecWireAncestorReadList = fb::RigExecWireAncestorReadList;
using RigExecWireInput = fb::RigExecWireInput;
using RigExecWireSlotMeta = fb::RigExecWireSlotMeta;
using RigExecWireConstants = fb::RigExecWireConstants;
using RigExecWireStep = fb::RigExecWireStep;
using RigExecWireCluster = fb::RigExecWireCluster;
using RigExecWireClustering = fb::RigExecWireClustering;
using RigExecWireClusterSet = fb::RigExecWireClusterSet;
using RigExecWireCones = fb::RigExecWireCones;
using RigExecWireLadder = fb::RigExecWireLadder;
using RigExecWireRbf = fb::RigExecWireRbf;
using RigExecWirePoseInterpolator = fb::RigExecWirePoseInterpolator;
using RigExecWireSplineIkRest = fb::RigExecWireSplineIkRest;
using RigExecWireSolver = fb::RigExecWireSolver;
using RigExecWireConstraint = fb::RigExecWireConstraint;
using RigExecWireWalkStep = fb::RigExecWireWalkStep;
using RigExecWireCommit = fb::RigExecWireCommit;
using RigExecWireConstraintArrays = fb::RigExecWireConstraintArrays;
using RigExecWireNativeSource = fb::RigExecWireNativeSource;
using RigExecWireComposeGroup = fb::RigExecWireComposeGroup;
using RigExecWireFrameVersion = fb::RigExecWireFrameVersion;
using RigExecWireSpaceSwitch = fb::RigExecWireSpaceSwitch;
using RigExecWireAvarBinding = fb::RigExecWireAvarBinding;
using RigExecWireDomainPose = fb::RigExecWireDomainPose;
using RigExecWireBlendSampleBinding = fb::RigExecWireBlendSampleBinding;
using RigExecWireBlendSampleBindingList =
    fb::RigExecWireBlendSampleBindingList;
using RigExecWireRevisionBinding = fb::RigExecWireRevisionBinding;
using RigExecWireBlendSample = fb::RigExecWireBlendSample;
using RigExecWireBlendChannel = fb::RigExecWireBlendChannel;
using RigExecWireChunk = fb::RigExecWireChunk;
using RigExecWireSkinTopology = fb::RigExecWireSkinTopology;
using RigExecWirePathValue = fb::RigExecWirePathValue;
using RigExecWirePathRead = fb::RigExecWirePathRead;
using RigExecWireRevision = fb::RigExecWireRevision;
using RigExecWireDerived = fb::RigExecWireDerived;
using RigExecWireChain = fb::RigExecWireChain;
using RigExecWireWeightObject = fb::RigExecWireWeightObject;
using RigExecWireDomainGeometry = fb::RigExecWireDomainGeometry;
using RigExecWirePropertyRevision = fb::RigExecWirePropertyRevision;
using RigExecWirePropertyChain = fb::RigExecWirePropertyChain;
using RigExecWirePhasedConsumer = fb::RigExecWirePhasedConsumer;
using RigExecWireExternalMover = fb::RigExecWireExternalMover;

// The schema's index structs, read through accessors (domain(), name()).
using RigExecWireSlotRange = fb::SlotRange;
using RigExecWirePathNode = fb::PathNode;
using RigExecWireInputSlot = fb::InputSlot;

// The schema's enums, whose enumerators keep their names.
using RigExecWireStepKind = fb::StepKind;
using RigExecWireSlotDomain = fb::SlotDomain;
using RigExecWireSlotKind = fb::SlotKind;
using RigExecWireInputTag = fb::InputTag;
using RigExecWireReadMode = fb::ReadMode;
using RigExecWireInputReadFlags = fb::InputReadFlags;
using RigExecWireInputSlotFlags = fb::InputSlotFlags;
using RigExecWirePathKind = fb::PathKind;
using RigExecWirePathTag = fb::PathTag;
using RigExecWirePropertyValueType = fb::PropertyValueType;
using RigExecWirePropertyOp = fb::PropertyOp;

/// Bits of RigExecWireConstraint::flags (fb::ConstraintFlags).
inline constexpr uint8_t RigExecWireConstraintBlendShear =
    uint8_t(fb::ConstraintFlags::BlendShear);
inline constexpr uint8_t RigExecWireConstraintWorldUpRotationOnly =
    uint8_t(fb::ConstraintFlags::WorldUpRotationOnly);
inline constexpr uint8_t RigExecWireConstraintRadialBlend =
    uint8_t(fb::ConstraintFlags::RadialBlend);

/// The format version this code reads and writes. Any other value is
/// refused with a rebake message, so every change to rigexec.fbs bumps it.
inline constexpr uint32_t RigExecFormatVersion = 5;

/// The file identifier, bytes 4-7 of every .rigexec file.
inline constexpr char RigExecFormatIdentifier[] = "REXB";

/// Every rule a file must satisfy that the file alone can decide: path tree
/// and pools, value and slot typing, every read's walk, constant and
/// override number, table shapes and indices, step and cone ranges, skin
/// topologies, path reads, property chains, external movers and the nested
/// presentation (bounded like Open's buffer, then verified with its REXP
/// identifier). False with a reason naming the table, index and field.
bool RigExecFormatValidate(const fb::RigExecWireFile &file,
                           std::string *error);

/// Decodes \p bytes: refuses a buffer shorter than 8 bytes, the old REXB
/// container (with a rebake message), a wrong file identifier or format
/// version, a buffer whose tables, vectors and strings, counted once per
/// reference, add up past its size (shared or overlapping objects, which
/// Write never produces), and any buffer the FlatBuffers verifier rejects
/// (it reads from a 16-byte-aligned copy, so the caller's alignment never
/// matters); then unpacks and validates, so the memory it takes grows
/// linearly with \p size. Null, with the reason, on failure, including an
/// allocation failure.
bool RigExecFormatOpen(const uint8_t *bytes, size_t size,
                       std::unique_ptr<fb::RigExecWireFile> *file,
                       std::string *error);

/// Validates \p file, then packs it with the file identifier. The bytes are
/// a pure function of \p file.
bool RigExecFormatWrite(const fb::RigExecWireFile &file,
                        std::vector<uint8_t> *bytes, std::string *error);

/// The text of path id \p id: "/A/B" for a prim, "/A/B.attr" for a
/// property, the token itself for a token; "" for 0 or an id out of range.
std::string RigExecFormatPathText(const fb::RigExecWireFile &file,
                                  uint32_t id);

/// The canonical sparse form of a dense skin layout: \p indices and
/// \p weights hold \p pointCount rows of \p elementSize entries each. Every
/// entry is kept, in order, except one whose index is 0 and whose weight
/// is zero of either sign, which reads exactly as padding; the counts and
/// indices go into the narrowest vectors that hold them. False with the
/// reason, and \p out untouched, for an element size outside [0, 65535],
/// more points than a file can hold, or a layout that is not
/// \p pointCount rows of \p elementSize.
bool RigExecFormatSparseTopology(const std::vector<int32_t> &indices,
                                 const std::vector<float> &weights,
                                 int32_t elementSize, uint64_t pointCount,
                                 uint64_t influenceCount, bool validated,
                                 fb::RigExecWireSkinTopology *out,
                                 std::string *error);

/// The dense rows a sparse layout the validator accepts stands for: each
/// point's kept entries in order, then (0, +0.0f) up to element_size.
void RigExecFormatExpandTopology(const fb::RigExecWireSkinTopology &topology,
                                 std::vector<int32_t> *indices,
                                 std::vector<float> *weights);

}  // namespace rigExec

#endif  // RIGEXEC_BINARY_FORMAT_H
