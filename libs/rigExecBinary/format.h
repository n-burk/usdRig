// The .rigexec file format: one FlatBuffer, schema libs/rigExecBinary/
// rigexec.fbs, file identifier "REXB". The generated object API types live
// in namespace rigExec::fb with the RigExecWire prefix (RigExecWireFile,
// RigExecWireSolver, ...) and are aliased into namespace rigExec under the
// same names; the schema's math and POD structs map onto the value types
// of rigExecBinary/wireTypes.h through the Pack/UnPack pairs declared
// here. Open is the decoder and Write the encoder: there is no
// hand-written schema codec. An optional versioned lossless transport wraps
// the same validated FlatBuffer; raw files remain accepted. USD-free.
#ifndef RIGEXEC_BINARY_FORMAT_H
#define RIGEXEC_BINARY_FORMAT_H

#include "rigExecBinary/wireTypes.h"

#include <cmath>
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
struct UIntPair;
struct Bool3;
struct F64;
struct F32;
struct ReadPhase;
struct PropertyInputCandidate;
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
inline rigExec::fb::UIntPair
PackUIntPair(const rigExec::RigExecWireUIntPair &value);
inline rigExec::RigExecWireUIntPair
UnPackUIntPair(const rigExec::fb::UIntPair &value);
inline rigExec::fb::Bool3 PackBool3(const rigExec::RigExecWireBool3 &value);
inline rigExec::RigExecWireBool3 UnPackBool3(const rigExec::fb::Bool3 &value);
inline rigExec::fb::F64 PackF64(const double &value);
inline double UnPackF64(const rigExec::fb::F64 &value);
inline rigExec::fb::F32 PackF32(const float &value);
inline float UnPackF32(const rigExec::fb::F32 &value);
inline rigExec::fb::PropertyInputCandidate
PackPropertyInputCandidate(const rigExec::RigExecWirePropertyInputCandidate &value);
inline rigExec::RigExecWirePropertyInputCandidate
UnPackPropertyInputCandidate(const rigExec::fb::PropertyInputCandidate &value);
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

inline rigExec::fb::F64 Pack(const double &value);
inline rigExec::fb::F32 Pack(const float &value);
inline rigExec::fb::Vec2f Pack(const rigExec::RigExecWireVec2f &value);
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

inline rigExec::fb::UIntPair
PackUIntPair(const rigExec::RigExecWireUIntPair &value)
{
    return rigExec::fb::UIntPair(value.first, value.second);
}

inline rigExec::RigExecWireUIntPair
UnPackUIntPair(const rigExec::fb::UIntPair &value)
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

inline rigExec::fb::F64 Pack(const double &value) { return PackF64(value); }
inline rigExec::fb::F32 Pack(const float &value) { return PackF32(value); }
inline rigExec::fb::Vec2f Pack(const rigExec::RigExecWireVec2f &value) { return PackVec2f(value); }

inline rigExec::fb::PropertyInputCandidate
PackPropertyInputCandidate(const rigExec::RigExecWirePropertyInputCandidate &value)
{
    return rigExec::fb::PropertyInputCandidate(value.slot,
        rigExec::fb::PropertyCandidateKind(value.kind), value.version, value.raw, value.poseWeight, value.crossDomain);
}

inline rigExec::RigExecWirePropertyInputCandidate
UnPackPropertyInputCandidate(const rigExec::fb::PropertyInputCandidate &value)
{
    return {value.slot(), uint8_t(value.kind()), value.version(), value.raw(), value.poseWeight(), value.crossDomain()};
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
using RigExecWireProviderFrameInput = fb::RigExecWireProviderFrameInput;
using RigExecWireProviderRefreshCarry = fb::RigExecWireProviderRefreshCarry;
using RigExecWireProviderRefresh = fb::RigExecWireProviderRefresh;
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
using RigExecWireWeightField = fb::RigExecWireWeightField;
using RigExecWireDomainGeometry = fb::RigExecWireDomainGeometry;
using RigExecWirePropertyRevision = fb::RigExecWirePropertyRevision;
using RigExecWirePropertyChain = fb::RigExecWirePropertyChain;
using RigExecWirePhasedConsumer = fb::RigExecWirePhasedConsumer;
using RigExecWireExternalMover = fb::RigExecWireExternalMover;
using RigExecWirePointsBinding = fb::RigExecWirePointsBinding;

// The schema's index structs, read through accessors (domain(), name()).
using RigExecWireSlotRange = fb::SlotRange;
using RigExecWirePathNode = fb::PathNode;
using RigExecWireInputSlot = fb::InputSlot;
using RigExecWireFrameRecord = fb::FrameRecord;
using RigExecWirePointVersion = fb::PointVersion;

// The schema's enums, whose enumerators keep their names.
using RigExecWireStepKind = fb::StepKind;
using RigExecWireSlotDomain = fb::SlotDomain;
using RigExecWireSlotKind = fb::SlotKind;
using RigExecWireInputTag = fb::InputTag;
using RigExecWireArraySource = fb::ArraySource;
using RigExecWireReadMode = fb::ReadMode;
using RigExecWireExternalDeclaredInput = fb::RigExecWireExternalDeclaredInput;
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

/// The format version every writer writes; every change to rigexec.fbs or
/// to what its records mean bumps it. Open reads it and every version from
/// RigExecFormatOldestReadable on; it refuses older versions with an S3
/// re-export message and unknown future versions with a rebake message.
///
/// Format 21 adds the extended Delta Mush, lattice and surface settings
/// (RigExecFormatExtendedSettingNames): mover attributes the revision reads
/// through its path reads and leaf sites, and frame providers held as its
/// influences. A format-20 file holds none of them, so its Delta Mush,
/// lattice and surface revisions play the legacy deformers.
inline constexpr uint32_t RigExecFormatVersion = 21;

/// The oldest format version Open reads.
inline constexpr uint32_t RigExecFormatOldestReadable = 20;

/// Whether Open reads format version \p version.
inline constexpr bool
RigExecFormatReads(uint32_t version)
{
    return version >= RigExecFormatOldestReadable &&
           version <= RigExecFormatVersion;
}

/// The first format version whose Delta Mush, lattice and surface
/// revisions may carry the extended settings.
inline constexpr uint32_t RigExecFormatExtendedSettingsVersion = 21;

/// The mover attributes format 21 adds to revision op \p op (DeltaMush,
/// Lattice or SurfaceProject), in the order the assembly reads them, with
/// their count in \p count; null with 0 for every other op.
const char *const *RigExecFormatExtendedSettingNames(uint8_t op,
                                                     size_t *count);

/// The frame providers a revision of op \p op with extended settings may
/// hold as influences: a Delta Mush at most one (rigExec:frame), a lattice
/// or a surface none or two (rigExec:frames: cage or surface, then
/// target). True for every other op.
bool RigExecFormatExtendedInfluencesValid(uint8_t op, size_t influences);

/// A Range revision (formats 19 and 20): unchunked with two or more chunks,
/// which are its chain's group partition. Keys are empty except on a Range
/// Skin (format 20), whose chunk g carries group g's influence key.
inline bool
RigExecFormatIsRangeRevision(const fb::RigExecWireRevision &r)
{
    return !r.chunked && r.chunks.size() >= 2;
}

/// The ops a Range revision may have in format 20: Matrix, Wire, Lattice,
/// BlendShape and Skin (a Skin only with a private classicLinear method).
inline bool
RigExecFormatIsRangeOp(uint8_t op)
{
    return op == uint8_t(fb::RevisionOp::Matrix) ||
           op == uint8_t(fb::RevisionOp::Wire) ||
           op == uint8_t(fb::RevisionOp::Lattice) ||
           op == uint8_t(fb::RevisionOp::BlendShape) ||
           op == uint8_t(fb::RevisionOp::Skin);
}

/// Whether \p r's chunks carry influence keys: a chunked skin (Legacy, or
/// Whole in a chain with groups) or a Range Skin.
inline bool
RigExecFormatIsKeyedRevision(const fb::RigExecWireRevision &r)
{
    return r.chunked || (RigExecFormatIsRangeRevision(r) &&
                         r.op == uint8_t(fb::RevisionOp::Skin));
}

/// A chain's group count in format 20: the chunk count of its first Range
/// revision (all Range revisions share the bounds), or 0 when it has none
/// (every revision Legacy). A revision of a chain with groups that is not
/// Range is Whole: its fuse writes RevisionOut[chunk_base + chunks.size(),
/// + groups).
size_t RigExecFormatChainGroups(const fb::RigExecWireFile &file, size_t chain);

/// Format 20 group writers of chain \p chain, from its revisions and the
/// file's RevisionChunk steps: (*written)[r][g] whether revision r writes
/// group g (Range: a step for part g; Whole: always; Legacy: never), and
/// (*enteringWriter)[r][g] the chain index of the last earlier revision that
/// writes g, or -1 (the base). Empty when the chain has no groups. The
/// validator and runtime Open share it.
void RigExecFormatGroupWriters(const fb::RigExecWireFile &file, size_t chain,
                               std::vector<std::vector<char>> *written,
                               std::vector<std::vector<int>> *enteringWriter);

/// Whether \p tag is one of the array tags (IntArray and after).
inline constexpr bool
RigExecFormatIsArrayTag(fb::InputTag tag)
{
    return uint8_t(tag) >= uint8_t(fb::InputTag::IntArray) &&
           uint8_t(tag) <= uint8_t(fb::InputTag::BoolArray);
}

/// The file identifier, bytes 4-7 of every .rigexec file.
inline constexpr char RigExecFormatIdentifier[] = "REXB";

/// Every rule a file must satisfy that the file alone can decide: path tree
/// and pools, value and slot typing (array values' pools and stored
/// layouts among them), every read's walk (no scalar read walks an array
/// slot, an array read walks slots of its own tag), constant and override
/// number, table shapes and indices, step and cone ranges, skin topologies
/// and the layout, chains with groups (their partitions, group steps'
/// reads and writes, and the private constants their Range skins, Range
/// lattices and gates rest on), chain base, painted and oracle slots, path
/// reads, property chains, external movers, the extended deformer settings
/// of its format version (none in format 20; each op's frame provider
/// count in 21) and the nested presentation (bounded like Open's buffer,
/// then verified with its REXP identifier). Any readable format version
/// validates. False with a reason naming the table, index and field.
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

/// Validates \p file, refuses any format version but RigExecFormatVersion,
/// then packs it with the file identifier. The bytes are a pure function of
/// \p file.
bool RigExecFormatWrite(const fb::RigExecWireFile &file,
                        std::vector<uint8_t> *bytes, std::string *error);

/// The text of path id \p id: "/A/B" for a prim, "/A/B.attr" for a
/// property, the token itself for a token; "" for 0 or an id out of range.
std::string RigExecFormatPathText(const fb::RigExecWireFile &file,
                                  uint32_t id);

/// Step \p step's label, as the baked program spells it
/// (RigExecBakedStepKindName and StepLabel): the kind, a space, then the
/// path of the object the step works on. Built from the tables, which it
/// range-checks, so it is safe on a file the validator has not accepted: a
/// step past the steps is its number, and an object past its table is the
/// kind and the object's number.
std::string RigExecFormatStepLabel(const fb::RigExecWireFile &file,
                                   size_t step);

/// The evaluator's shape rule for a skin layout (RigExecBuildSkinTopology):
/// \p indexCount indices and \p weightCount weights are rows of an
/// \p elementSize of at least 1, over at least one influence.
inline bool
RigExecFormatSkinShapeValidates(size_t indexCount, size_t weightCount,
                                int32_t elementSize, uint64_t influenceCount)
{
    return elementSize >= 1 && indexCount == weightCount &&
           indexCount % size_t(elementSize) == 0 && influenceCount >= 1;
}

/// The evaluator's entry rule: \p index names one of \p influenceCount
/// influences, and \p weight is finite and not negative (-0 passes).
inline bool
RigExecFormatSkinEntryValidates(int32_t index, float weight,
                                uint64_t influenceCount)
{
    return index >= 0 && uint64_t(index) < influenceCount &&
           std::isfinite(weight) && !(weight < 0.0f);
}

/// Whether the evaluator validates the dense skin layout \p indices x
/// \p weights at \p elementSize over \p influenceCount influences: the
/// shape rule, then the entry rule on every entry. Exactly
/// RigExecBuildSkinTopology's `validated`; the validator holds a stored
/// layout's flag to it, and playback computes it for a layout it builds.
inline bool
RigExecFormatSkinLayoutValidates(const int32_t *indices, size_t indexCount,
                                 const float *weights, size_t weightCount,
                                 int32_t elementSize, uint64_t influenceCount)
{
    if (!RigExecFormatSkinShapeValidates(indexCount, weightCount,
                                         elementSize, influenceCount)) {
        return false;
    }
    for (size_t k = 0; k < indexCount; ++k) {
        if (!RigExecFormatSkinEntryValidates(indices[k], weights[k],
                                             influenceCount)) {
            return false;
        }
    }
    return true;
}

/// The sparse form of a dense skin layout: \p indices and \p weights hold
/// \p pointCount rows of \p elementSize entries each. Each row keeps its
/// entries, in order, up to its trailing run of (index 0, weight +0.0f)
/// entries, which the expansion's padding restores, so the form is
/// lossless; the counts and indices go into the narrowest vectors that
/// hold them. False with the reason, and \p out untouched, for an element
/// size outside [0, 65535], more points than a file can hold, or a layout
/// that is not \p pointCount rows of \p elementSize.
bool RigExecFormatSparseTopology(const std::vector<int32_t> &indices,
                                 const std::vector<float> &weights,
                                 int32_t elementSize, uint64_t pointCount,
                                 uint64_t influenceCount, bool validated,
                                 fb::RigExecWireSkinTopology *out,
                                 std::string *error);

/// The canonical form of a skin layout the evaluators resolved: the sparse
/// form (RigExecFormatSparseTopology) when \p indices and \p weights are
/// rows of an element size in [0, 65535] (none at element size 0), else
/// the raw form, which holds the arrays verbatim with \p validated as
/// given. False with the reason, and \p out untouched, when the sparse form
/// refuses, or when \p pointCount is not what the evaluators give a raw
/// layout: its rows when it is rows of an element size of at least 1, else
/// 0. RigExecFormatValidate holds validated to the evaluator's layout
/// rules in either form.
bool RigExecFormatTopology(const std::vector<int32_t> &indices,
                           const std::vector<float> &weights,
                           int32_t elementSize, uint64_t pointCount,
                           uint64_t influenceCount, bool validated,
                           fb::RigExecWireSkinTopology *out,
                           std::string *error);

/// The arrays a layout the validator accepts stands for: a raw layout's
/// own, verbatim; a sparse layout's dense rows, each point's kept entries
/// in order, then (0, +0.0f) up to element_size. Either way, the arrays
/// the layout was written from, bit for bit.
void RigExecFormatExpandTopology(const fb::RigExecWireSkinTopology &topology,
                                 std::vector<int32_t> *indices,
                                 std::vector<float> *weights);

}  // namespace rigExec

#endif  // RIGEXEC_BINARY_FORMAT_H
