// .rigexec Computed section (tag 18): the input slot model, the weight
// oracle facts, the property chains and the registered reads that cross
// them, carried beside the old frame records while the runtime computes
// the results those records replay.
// TEMPORARY. This section, its codec and the per-frame slot values are
// removed when the container moves to the single-FlatBuffer layout. The
// types in namespace v4 are shaped like that layout's object types
// (InputSlot, Value, Input, WeightObject, PropertyChain, PropertyRevision,
// PhasedConsumer), so the move replaces their
// definitions rather than their uses; RigExecWireComputed, the read tables
// keyed by frame-record fields (RigExecWireChainRead,
// RigExecWireRegisteredRead, RigExecWireBlendWeightRead,
// RigExecWireDefaultWeightRead, RigExecWirePathScalarRead) and
// RigExecWireComputedFrame have no v4 counterpart and go with the section.
// Index conventions are the v4 ones with one exception: what v4 calls a path
// id is a string-table index here (0 = none), for slot names, weight object
// paths, tokens and Token values alike.
// USD-free, like every header in this library.
#ifndef RIGEXEC_BINARY_COMPUTED_H
#define RIGEXEC_BINARY_COMPUTED_H

#include "rigExecBinary/inputTable.h"

#include <cstdint>
#include <string>
#include <vector>

namespace rigExec {
namespace v4 {

/// Value type of a slot, a read or a constant. 0..6 are the old
/// RigExecWireInput::Tag numbers. Frozen: append only.
enum class InputTag : uint8_t {
    Double = 0,
    Float = 1,
    Bool = 2,
    Int = 3,
    Matrix4d = 4,
    Token = 5,
    Vec3d = 6,
    Vec3f = 7,
};
inline constexpr uint8_t InputTagLast = 7;

/// How a read resolves.
enum class ReadMode : uint8_t {
    Baked = 0,     ///< RigExecBakedRead (bakedProgramImpl.h)
    Resolved = 1,  ///< RigExecResolvedInputs::GetAttribute (moverGraph.h)
    Pinned = 2,    ///< _PinnedRead (rigEvaluatorProperties.cpp)
    Raw = 3,       ///< attr.Get at walk[0], exact type, no walk
};
inline constexpr uint8_t ReadModeLast = 3;

/// Bits of RigExecWireInput::flags. NONE and ANY mirror the members
/// FlatBuffers generates for a bit_flags enum.
enum class InputReadFlags : uint8_t {
    NONE = 0,
    /// Baked: the input is read per run (RigExecBakedInput::varying).
    /// Other modes: some hop of the walk is time-varying (informational).
    Varying = 1,
    /// Baked: the program reads the long way (RigExecBakedInput::
    /// resolvedAttr). Unset in the other modes.
    LongWay = 2,
    /// A property-chain target lies on the walk.
    ViaChain = 4,
    ANY = 7,
};

/// Bits of InputSlot::flags (NONE and ANY as above).
enum class InputSlotFlags : uint8_t {
    NONE = 0,
    Listed = 1,    ///< addressable by name through the input API
    Animated = 2,  ///< ValueMightBeTimeVarying() || GetNumTimeSamples() > 0
    HasValue = 4,  ///< the typed Get at the bake time succeeded
    ANY = 7,
};

/// A tagged constant. Only the member `tag` names is meaningful: `bits`
/// holds a Double's IEEE-754 pattern, a Float's in the low 32 bits, a Bool
/// as 0/1, an Int as its uint32 two's-complement form, a Token as its
/// string index; Matrix4d, Vec3d and Vec3f use their members.
struct RigExecWireValue {
    InputTag tag = InputTag::Double;
    uint64_t bits = 0;
    RigExecWireMatrix4d matrix{};
    RigExecWireVec3d vec3d{};
    RigExecWireVec3f vec3f{};
};

/// One entry of the input list: an attribute whose authored value the
/// runtime reads, keyed by its path.
struct InputSlot {
    uint32_t name = 0;    ///< attribute path (string index)
    uint32_t value = 0;   ///< values[]: the attribute's value at the bake time
    int32_t chain = -1;   ///< propertyChains[] whose target this is, else -1
    int32_t phased = -1;  ///< phasedConsumers[] publishing at it, else -1
    InputTag type = InputTag::Double;  ///< the attribute's value type
    uint8_t flags = 0;    ///< InputSlotFlags
};

/// One bound input READ: the read's type, how it resolves, and the walk of
/// slots GetAttribute would visit (head first; empty = no attribute, which
/// reads `constant`).
struct RigExecWireInput {
    InputTag tag = InputTag::Double;
    ReadMode mode = ReadMode::Baked;
    uint8_t flags = 0;           ///< InputReadFlags
    int32_t overrideIndex = -1;  ///< Baked: program override number
    uint32_t constant = 0;       ///< values[]: input.constant / the fallback
    std::vector<uint32_t> walk;  ///< slot ids, head first
    /// Baked && Varying && !LongWay: index into `walk` of the attribute the
    /// program's query is pinned to; -1 otherwise.
    int16_t selected = -1;
};

/// The value type of a property chain, which selects the evaluator's
/// runChain arm (rigEvaluatorProperties.cpp): the target's type name is
/// float, double or matrix4d, and every other admitted type is
/// GfVec3f-backed. Same order as RigExecWirePropertyValue::Tag. Frozen.
enum class PropertyValueType : uint8_t {
    Float = 0,
    Double = 1,
    Matrix4d = 2,
    Vec3f = 3,
};
inline constexpr uint8_t PropertyValueTypeLast = 3;

/// RigExecParsePropertyOp of the mover's rigExec:operation, read at
/// Default as _ReadOperation reads it. Invalid: the token did not parse, so
/// the revision's inputs are unusable. Frozen: append only.
enum class PropertyOp : uint8_t {
    Add = 0,
    Multiply = 1,
    Clamp = 2,
    Remap = 3,
    Blend = 4,
    Curve = 5,
    Invalid = 6,
};
inline constexpr uint8_t PropertyOpLast = 6;

/// One property-mover revision (runChain, rigEvaluatorProperties.cpp).
/// Each read is _PinnedRead of the mover's input: Pinned mode for an
/// unconnected attribute (walk = the attribute alone), Resolved mode for a
/// connected one (GetAttribute's walk), an empty Pinned walk when the mover
/// has no such attribute. Every read's constant is _PinnedRead's fallback.
struct PropertyRevision {
    uint32_t mover = 0;  ///< prim path (string index), for diagnostics
    PropertyOp op = PropertyOp::Invalid;
    RigExecWireInput enabled;  ///< Bool; fallback true
    /// weightObjects[] index of rigExec:weightObject's first target, whose
    /// one-element oracle field is the envelope; -1 reads defaultWeight.
    int32_t envelope = -1;
    RigExecWireInput defaultWeight;  ///< Float; fallback 1
    /// The operation's inputs. Float for float and double chains (a double
    /// chain is computed in float), Vec3f for vec3f chains, both with a zero
    /// fallback; a Matrix4d value with an identity fallback for matrix
    /// chains, whose min and max are empty reads nothing evaluates.
    RigExecWireInput value;
    RigExecWireInput min;
    RigExecWireInput max;
    /// inputs:keys and inputs:tangents at Default, for a float or double
    /// chain whose operation is curve; empty otherwise. The bake refuses
    /// keys or tangents that are animated or connected.
    std::vector<RigExecWireVec2f> keys;
    bool hasTangentsAttr = false;
    std::vector<RigExecWireVec2f> tangents;
};

/// One property chain: a target attribute and the revisions every property
/// mover moving it applies, in order. Chains are in the evaluator's
/// dependency order (_propertyChainOrder).
struct PropertyChain {
    /// Slot id of the target; the chain's base is its raw value, read with
    /// the chain's type (a missing or mistyped value skips the chain).
    uint32_t target = 0;
    PropertyValueType valueType = PropertyValueType::Float;
    std::vector<PropertyRevision> revisions;
};

/// A connection that reads a chain at its phase, the base unless it
/// declares another (RigExecPhasedConnection): after the chain runs, the
/// value after its first `applied` revisions is published at the consumer,
/// converted per
/// RigExecPhasedConsumerValue. Grouped by chain, in the evaluator's order
/// within each chain.
struct PhasedConsumer {
    uint32_t chain = 0;     ///< propertyChains[]
    uint32_t consumer = 0;  ///< slot id of the consuming attribute
    /// Float or Double when the consumer has that type (a float/double pair
    /// converts), else the chain's own type (no conversion).
    PropertyValueType consumerType = PropertyValueType::Float;
    uint32_t applied = 0;   ///< 0 = the base; at most the revision count
    /// Slot ids of the consumer, then of each attribute its connection walk
    /// passes before the chain target (RigExecPhasedConnection::hops). An
    /// interactive override on any of them stands the reader aside: nothing
    /// is published at the consumer, and its walk meets the override.
    std::vector<uint32_t> hops;
};

/// Wrapper for a pooled points array (v4 vec3f_arrays entry).
struct RigExecWireVec3fArray {
    std::vector<RigExecWireVec3f> v;
};

/// One weight object: its composition by value, its scalar reads, and the
/// facts the runtime port of RigExecRigEvaluator::_ResolveWeights needs.
/// `base` and `inputs` index the same table and are always lower than the
/// entry's own index (dependency order).
struct RigExecWireWeightObject {
    uint32_t path = 0;
    uint32_t type = 0;            ///< token
    uint32_t representation = 0;  ///< token
    uint32_t rangePolicy = 0;     ///< token
    std::vector<float> values;
    std::vector<int32_t> indices;
    RigExecWireInput defaultWeight;
    int32_t base = -1;
    std::vector<int32_t> inputs;
    RigExecWireInput driver;
    RigExecWireInput scale;
    RigExecWireInput bias;
    uint32_t combineMode = 0;  ///< token
    RigExecWireInput strength;
    RigExecWireInput invert;
    std::vector<uint32_t> combineTargetPoints;
    std::vector<uint8_t> combineTargetValid;
    uint64_t costElements = 1;
    int32_t providerSlot = -1;
    RigExecWireInput falloffMin;
    RigExecWireInput falloffMax;
    RigExecWireInput scaleXPos;
    RigExecWireInput scaleYPos;
    RigExecWireInput scaleZPos;
    RigExecWireInput scaleXNeg;
    RigExecWireInput scaleYNeg;
    RigExecWireInput scaleZNeg;
    RigExecWireInput scaleX;
    RigExecWireInput scaleY;
    RigExecWireInput scaleZ;
    RigExecWireInput extentU;
    RigExecWireInput extentV;
    uint32_t planeAxis = 0;    ///< token
    uint32_t planeBounds = 0;  ///< token
    std::vector<uint32_t> targetPoints;
    std::vector<uint8_t> targetValid;
    std::vector<uint32_t> samplePoints;
    std::vector<uint8_t> sampleValid;
    std::vector<uint32_t> curvePoints;
    std::vector<uint8_t> curveValid;
    std::vector<float> falloffCurve;
    // The oracle facts (rigEvaluatorGeometry.cpp _ResolveWeights,
    // _ResolveVolumeWeights, _VolumeWeightSamplesInFlight, _ReadTargetPoints).
    /// Appended for a constraint or property-mover envelope; no WeightPacket
    /// step builds it.
    bool envelopeOnly = false;
    /// A volume whose rigExec:weightTarget reads `preceding`: it samples
    /// the points entering the revision that binds it.
    bool samplesInFlight = false;
    /// vec3fArrays[]: the points a volume not in flight samples
    /// (rigExec:sampleSource, else rigExec:weightTarget, at the bake time);
    /// -1 when neither reads.
    int32_t oracleSamples = -1;
    /// vec3fArrays[]: a curve volume's rigExec:curve points at the bake
    /// time; -1 when it does not read or is empty.
    int32_t oracleCurve = -1;
    /// A plane volume's rigExec:planeAxis and rigExec:planeBounds tokens as
    /// the oracle reads them at the bake time: the fallback only when the
    /// attribute is absent, so an authored empty token stays empty (the
    /// composition tokens above substitute the fallback for it). 0 for
    /// every other type.
    uint32_t oraclePlaneAxis = 0;
    uint32_t oraclePlaneBounds = 0;
    /// The oracle's whole error text when the read-phase check fails;
    /// empty otherwise.
    std::string oraclePhaseError;
    /// The oracle's whole error text for this object's first structural
    /// failure, which precedes every per-run read of the object; empty
    /// otherwise.
    std::string oracleStaticError;
};

}  // namespace v4

/// A program-registered read whose walk crosses a property-chain target
/// (Baked mode, Varying, LongWay and ViaChain), with the InputTable uid its
/// per-frame value is recorded under: a restatement of the registered read
/// with that uid, which the cross-check evaluates and compares with the
/// frame record's value under the uid (temporary: the uids go with the
/// frame records).
struct RigExecWireChainRead {
    uint32_t uid = 0;
    v4::RigExecWireInput read;
};

/// The program tables a registered read belongs to, in the order the
/// frozen context's patchable-input walk visits them.
enum class RigExecWireRegisteredFamily : uint8_t {
    AvarBinding = 0,
    AvarConstantBinding = 1,
    Ladder = 2,
    SpaceSwitch = 3,
    Interpolator = 4,
    Solver = 5,
    Constraint = 6,
    WeightObject = 7,
};
inline constexpr uint8_t RigExecWireRegisteredFamilyLast = 7;

/// One program-registered read (a RigExecBakedInput the program holds), in
/// Baked mode, with where the frame-record runtime reads its value today:
/// `object` indexes the family's table and `field` is the runtime's field
/// number (rigExecRuntime/store.h; an interpolator's enable is 0 and its
/// dial k is 1 + k; an avar binding's is 0). `uid` is its InputTable uid,
/// -1 when the directory gives it none; `avar` is an avar binding's flat
/// avar index (slot * 11 + channel), -1 for every other family.
struct RigExecWireRegisteredRead {
    RigExecWireRegisteredFamily family = RigExecWireRegisteredFamily::Ladder;
    uint32_t object = 0;
    uint32_t field = 0;
    int32_t uid = -1;
    int32_t avar = -1;
    v4::RigExecWireInput read;
};

/// A blend channel's inputs:weight (Resolved, Float, fallback 0): geometry
/// chain `chain`, its revision `revision` (or its derived target `revision`
/// when `derived`), binding channel `channel`, one entry per channel. A
/// pose-driven channel (its binding's poseWeight >= 0) takes the pose
/// weight slot instead, unless a property-chain result or phased value is
/// published at the channel's own weight path; only then does the
/// assembly use this read.
/// `activations`: each of the channel's samples' rigExec:activation, in
/// binding order (Resolved, Float, fallback 1), which the assembly reads
/// every run before it sorts the samples by them.
struct RigExecWireBlendWeightRead {
    uint32_t chain = 0;
    uint32_t revision = 0;
    bool derived = false;
    uint32_t channel = 0;
    v4::RigExecWireInput read;
    std::vector<v4::RigExecWireInput> activations;
};

/// A geometry revision's inputs:defaultWeight as the RevisionStatic step
/// reads it (Resolved, Float, fallback 1).
struct RigExecWireDefaultWeightRead {
    uint32_t chain = 0;
    uint32_t revision = 0;
    v4::RigExecWireInput read;
};

/// A scalar mover input the geometry assembly reads through its
/// connections and records under the head attribute's path (the
/// connection-following stage reads): Resolved, with the site's fallback;
/// `read.walk[0]` is the head. One entry per head path.
/// `widen`: the site reads a float and records it as a double.
/// `headFallback`: when the walk yields nothing, the site reads the head's
/// own typed value before the fallback (`if (!GetAttribute) attr.Get`).
/// That differs from the walk alone only for a float read stopped by a
/// double hop with no value; every assembler scalar site sets it.
struct RigExecWirePathScalarRead {
    uint32_t path = 0;  ///< the head attribute (string index)
    bool widen = false;
    bool headFallback = false;
    v4::RigExecWireInput read;
};

/// One baked frame's slot values (temporary: a v4 file has none). Aligned
/// with RigExecWireInputTable::frames; one entry per slot in each vector.
struct RigExecWireComputedFrame {
    double frame = 0;
    std::vector<uint32_t> values;   ///< values[] per slot: typed Get at frame
    std::vector<uint8_t> hasValue;  ///< 0/1 per slot: that Get succeeded
};

/// The Computed section.
struct RigExecWireComputed {
    /// The time slot defaults and the oracle's static points were read at.
    double bakeTime = 0;
    /// Constants; values[0] is Double +0.0.
    std::vector<v4::RigExecWireValue> values;
    /// Pooled points; vec3fArrays[0] is empty.
    std::vector<v4::RigExecWireVec3fArray> vec3fArrays;
    /// Listed slots first, sorted by path text, then unlisted ones.
    std::vector<v4::InputSlot> inputs;
    uint32_t listedInputs = 0;
    /// Every weight object the program bakes, in DomainGeometry order (the
    /// WeightPacket table), then the envelope-only ones.
    std::vector<v4::RigExecWireWeightObject> weightObjects;
    /// Per DomainPose constraint: its envelope in weightObjects (weightObject
    /// set and no points target), else -1. The runtime copies it into
    /// RigExecWireConstraint::weightObjectIndex.
    std::vector<int32_t> constraintWeightObjectIndex;
    /// The property chains, in dependency order, and their phased
    /// consumers.
    std::vector<v4::PropertyChain> propertyChains;
    std::vector<v4::PhasedConsumer> phasedConsumers;
    /// The program's registered reads that cross a chain target, in
    /// ascending uid.
    std::vector<RigExecWireChainRead> chainReads;
    /// Every program-registered read, in the patchable-input walk's order.
    std::vector<RigExecWireRegisteredRead> registeredReads;
    /// The geometry assembly's reads outside the registered tables.
    std::vector<RigExecWireBlendWeightRead> blendWeightReads;
    std::vector<RigExecWireDefaultWeightRead> defaultWeightReads;
    std::vector<RigExecWirePathScalarRead> pathScalarReads;
    /// Per-frame slot values, aligned with the InputTable frames.
    std::vector<RigExecWireComputedFrame> frames;
};

/// Encodes \p computed after checking it as the decoder does.
bool RigExecWireEncodeComputed(const RigExecWireComputed &computed,
                               std::vector<uint8_t> *out,
                               std::string *error = nullptr);
/// Decodes and checks every index the section holds against its own tables:
/// enums in range, walks and constants inside the slot and value tables,
/// `selected` inside its walk, base/inputs below the entry (dependency
/// order), envelope-only entries last, array refs inside the pool, chain
/// targets and phased consumers agreeing with their slots' back references,
/// revision reads typed and moded as their revision reads them, chain-
/// crossing registered reads in ascending uid with a chain target on their
/// walk, registered reads in Baked mode and the geometry reads Resolved,
/// one blend read per channel (its activation reads Resolved Floats too),
/// one default weight read per revision, path
/// reads headed by the attribute they are keyed by and unique by path,
/// phased consumers' hops headed by the consumer and short of the chain
/// target, frame vectors one per slot. Rejects trailing bytes.
bool RigExecWireDecodeComputed(RigExecWireReader *reader,
                               RigExecWireComputed *computed,
                               std::string *error);

/// Checks \p computed against the sections it extends -- one step-backed
/// weight object per geometry weight object, one index per constraint, one
/// frame per InputTable record at the same time, property chains exactly
/// when the pose tables say the rig has them, each chain-crossing or
/// registered read with a uid naming a directory entry of its tag,
/// override number and head, each registered read naming a table row of
/// the pose or geometry section, each geometry read naming a revision (and
/// a blend channel, with one activation read per sample of it) the
/// geometry section holds -- and copies each
/// constraint's index into \p pose.
bool RigExecWireApplyComputed(const RigExecWireComputed &computed,
                              const RigExecWireDomainGeometry &geometry,
                              const RigExecWireInputTable &inputs,
                              RigExecWireDomainPose *pose,
                              std::string *error);

}  // namespace rigExec

#endif  // RIGEXEC_BINARY_COMPUTED_H
