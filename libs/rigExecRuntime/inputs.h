// rigExecRuntime input slot model and read evaluation.
// The slots, their bake-time defaults and every read come from the opened
// file (File.inputs, File.values and the Input tables in place). A slot's
// value is mutable: an input set through the reader is the attribute's
// authored value. A read is a pure function of the slot values, the
// override flags the set inputs raise and the property-chain results the
// run published, so it is safe wherever a step body may run. USD-free.
#ifndef RIGEXEC_RUNTIME_INPUTS_H
#define RIGEXEC_RUNTIME_INPUTS_H

#include "rigExecBinary/format.h"
#include "rigExecRuntime/values.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <map>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

namespace rigExec {

struct RrProgram;

// The runtime numbers the input tags as the file does.
static_assert(uint8_t(RrInputTag::IntArray) ==
                      uint8_t(RigExecWireInputTag::IntArray) &&
                  uint8_t(RrInputTag::FloatArray) ==
                      uint8_t(RigExecWireInputTag::FloatArray) &&
                  uint8_t(RrInputTag::DoubleArray) ==
                      uint8_t(RigExecWireInputTag::DoubleArray) &&
                  uint8_t(RrInputTag::Vec2fArray) ==
                      uint8_t(RigExecWireInputTag::Vec2fArray) &&
                  uint8_t(RrInputTag::Vec3fArray) ==
                      uint8_t(RigExecWireInputTag::Vec3fArray),
              "RrInputTag mirrors the file's InputTag");
// Array elements are compared and copied as bytes, and an int array is
// the layouts' std::vector<int>.
static_assert(sizeof(RrVec2f) == 2 * sizeof(float) &&
                  sizeof(RrVec3f) == 3 * sizeof(float) &&
                  std::is_same<int, int32_t>::value,
              "array elements are their components, unpadded");

/// The element type of an array input tag.
template <class T>
struct RrArrayTag;
template <>
struct RrArrayTag<int32_t> {
    static constexpr RigExecWireInputTag value = RigExecWireInputTag::IntArray;
};
template <>
struct RrArrayTag<float> {
    static constexpr RigExecWireInputTag value =
        RigExecWireInputTag::FloatArray;
};
template <>
struct RrArrayTag<double> {
    static constexpr RigExecWireInputTag value =
        RigExecWireInputTag::DoubleArray;
};
template <>
struct RrArrayTag<RrVec2f> {
    static constexpr RigExecWireInputTag value =
        RigExecWireInputTag::Vec2fArray;
};
template <>
struct RrArrayTag<RrVec3f> {
    static constexpr RigExecWireInputTag value =
        RigExecWireInputTag::Vec3fArray;
};

template <> struct RrArrayTag<RigExecWireVec3d> {
    static constexpr RigExecWireInputTag value=RigExecWireInputTag::Vec3dArray;
};
template <> struct RrArrayTag<RigExecWireMatrix4d> {
    static constexpr RigExecWireInputTag value=RigExecWireInputTag::Matrix4dArray;
};
template <> struct RrArrayTag<uint32_t> {
    static constexpr RigExecWireInputTag value=RigExecWireInputTag::TokenArray;
};
template <> struct RrArrayTag<uint8_t> {
    static constexpr RigExecWireInputTag value=RigExecWireInputTag::BoolArray;
};

/// Typed element storage for an array input, one vector per element type;
/// only the vector of the slot's tag is used.
struct RrArrayBuffer {
    std::vector<int32_t> ints;
    std::vector<float> floats;
    std::vector<double> doubles;
    std::vector<RrVec2f> vec2s;
    std::vector<RrVec3f> vec3s;
    std::vector<RigExecWireVec3d> vec3ds;
    std::vector<RigExecWireMatrix4d> matrices;
    std::vector<uint32_t> tokens;
    std::vector<uint8_t> bools;
};

/// One array input slot. Its default is borrowed, never copied: a pool
/// entry of the file, a float2 or float3 pool entry converted once at Open,
/// or the expansion of a stored skin layout. A set's elements are held
/// beside it; a set equal to the default holds nothing, so "at its
/// default" is "holds no set".
struct RrArraySlot {
    RigExecWireInputTag tag = RigExecWireInputTag::IntArray;
    /// A const std::vector of the tag's element type.
    const void *defaultVector = nullptr;
    RrArrayBuffer held;
    bool holdsSet = false;
    /// The held value came from an authored set (SetInputArray(At)):
    /// Default-time reads take it too, as they take an authored value.
    bool authored = false;
    /// What the last run read, kept from this run's first set or reset:
    /// `ran` when ranHeld, else the default.
    RrArrayBuffer ran;
    bool ranHeld = false;
    /// The elements' content version (held set, else default): two runs
    /// read the same version exactly when they read the same elements.
    /// Moved only by RrInputsApplyTouched against `ran`.
    uint64_t version = 0;
};

/// A tagged value by value: the file's Value with its optional members
/// inline, so a slot, a read and the overlay copy it without allocating.
/// Only the member `tag` names is meaningful: `bits` holds a Double's
/// IEEE-754 pattern, a Float's in the low 32 bits, a Bool as 0/1, an Int as
/// its uint32 two's-complement form, a Token as its path id.
struct RrWireValue {
    RigExecWireInputTag tag = RigExecWireInputTag::Double;
    uint64_t bits = 0;
    RigExecWireMatrix4d matrix{};
    RigExecWireVec3d vec3d{};
    RigExecWireVec3f vec3f{};
    RigExecWireVec3i vec3i{};
};

// A slot-model value in the steps' form: the member its tag names holds
// the value's bits, every other member stays zero. No step input reads a
// Vec3f; an input's value can be one.
inline RrInputValue
RrValueFromWire(const RrWireValue &value)
{
    RrInputValue out;
    out.tag = RrInputTag(uint8_t(value.tag));
    out.matrix.SetDiagonal(0.0);
    switch (value.tag) {
    case RigExecWireInputTag::Double:
        std::memcpy(&out.f64, &value.bits, sizeof(out.f64));
        break;
    case RigExecWireInputTag::Float: {
        const uint32_t bits = uint32_t(value.bits);
        std::memcpy(&out.f32, &bits, sizeof(out.f32));
        break;
    }
    case RigExecWireInputTag::Bool:
        out.boolean = value.bits != 0;
        break;
    case RigExecWireInputTag::Int:
        out.i32 = int32_t(uint32_t(value.bits));
        break;
    case RigExecWireInputTag::Matrix4d:
        for (size_t r = 0; r < 4; ++r) {
            for (size_t c = 0; c < 4; ++c) {
                out.matrix[r][c] = value.matrix[r * 4 + c];
            }
        }
        break;
    case RigExecWireInputTag::Token:
        out.token = uint32_t(value.bits);
        break;
    case RigExecWireInputTag::Vec3d:
        out.vec = RrVec3d(value.vec3d[0], value.vec3d[1], value.vec3d[2]);
        break;
    case RigExecWireInputTag::Vec3i:
        out.vec3i=RrVec3i(value.vec3i[0],value.vec3i[1],value.vec3i[2]);
        break;
    case RigExecWireInputTag::Vec3f:
        out.vec3f = RrVec3f(value.vec3f[0], value.vec3f[1], value.vec3f[2]);
        break;
    default:
        // An array value carries its tag alone.
        break;
    }
    return out;
}

/// The file's Value \p value inline: its tag and bits, and the optional
/// member its tag names.
RrWireValue RrWireValueOf(const fb::RigExecWireValue &value);

/// A stage value an assembler reads: what the bake captured, or what a
/// connection-following read evaluated this run. Absent marks a site that
/// read its fallback; every other tag carries the value in the member it
/// names.
struct RrPathValue {
    enum class Tag : uint8_t {
        Absent = 0,
        Bool = 1,
        Int = 2,
        Float = 3,
        Double = 4,
        Token = 5,
        Matrix4d = 6,
        Vec3d = 7,
        IntArray = 8,
        FloatArray = 9,
        Vec2fArray = 10,
        Vec3fArray = 11,
        Vec3i = 12,
        DoubleArray = 13,
    };
    Tag tag = Tag::Absent;
    bool boolean = false;
    int32_t i32 = 0;
    float f32 = 0;
    double f64 = 0;
    uint32_t token = 0;
    RigExecWireMatrix4d matrix{};
    RigExecWireVec3d vec{};
    std::vector<int32_t> ints;
    std::vector<float> floats;
    std::vector<RrVec2f> vec2s;
    std::vector<RrVec3f> vec3s;
    RigExecWireVec3i vec3i{{0, 0, 0}};
    std::vector<double> doubles;
};

/// One row of the path-read table (RrProgram::pathReads), keyed by the
/// attribute's path id and whether the site read its rest (Default) value.
/// A value row holds what the bake captured. A scalar read row holds the
/// head of a connection-following read, whose value every geometry
/// prologue evaluates over the slots. An array read row is read in place
/// (RrPathArray); at rest it holds the attribute's Default-time value.
struct RrPathRead {
    uint32_t path = 0;
    bool rest = false;
    /// The file's row of a read row; null on a value row.
    const RigExecWirePathRead *read = nullptr;
    RrPathValue value;
};

/// The index of row (\p path, \p rest) of path-read table \p table, or -1
/// when it holds no such row. A binary search: the table is sorted by
/// (path, rest), one row per key. The families resolve their sites' rows
/// with it at Open; no run searches the table.
int32_t RrFindPathReadRow(const std::vector<RrPathRead> &table,
                          uint32_t path, bool rest);

/// Builds the path-read table at Open from the file's path_reads: a value
/// row per static value, decoded out of the pools, a read row per
/// connection-following scalar read, and an array read row, with a rest
/// row's Default-time value decoded. False with the reason for a scalar
/// read whose type no assembler site reads.
bool RrInputsBuildPathReads(RrProgram *program, std::string *error);

/// The slot model over the opened file.
struct RrInputState {
    const RigExecWireFile *file = nullptr;
    /// Animated arrays with actual AtTime consumers, computed once at Open.
    std::vector<size_t> stageArraySlots;
    std::vector<size_t> stageTokenSlots;
    /// File.values inline, entry for entry.
    std::vector<RrWireValue> values;
    /// The first id SetInputToken gives text no Token node holds: the path
    /// table's size, so every id below it is the file's.
    uint32_t extraTokenBase = 0;
    /// Per slot: the value every read sees this run, and whether it holds
    /// one (the attribute's typed read succeeded, or a value was set).
    /// Seeded with the bake-time defaults (values[slot.value] and the
    /// HasValue flag, kept in slotDefaultHasValue).
    std::vector<RrWireValue> slotCurrent;
    std::vector<uint8_t> slotHasValue;
    std::vector<uint8_t> slotBlocked, slotProviderSource;
    std::vector<uint8_t> slotAuthored, slotRanAuthored;
    std::vector<uint8_t> slotDefaultHasValue;
    /// Per slot: whether a topology read walks it (RrInputsMarkTopology,
    /// at Open), which fixes its element count for the reader.
    std::vector<uint8_t> slotTopology;
    /// Per slot: the value and HasValue the last run read, which a set is
    /// compared with to tell a change from a repeat. Seeded with the
    /// defaults; kept for slots that are not Animated.
    std::vector<RrWireValue> slotRan;
    std::vector<uint8_t> slotRanHasValue;
    /// Per slot: whether the inputs the last run applied changed it against
    /// the run before (bits or elements, or HasValue; an Animated scalar
    /// always), listed in changedSlots until the next run applies its own.
    std::vector<char> slotChangedSinceRun;
    std::vector<uint32_t> changedSlots;
    /// The array slots: their storage, and per slot its entry (-1 for a
    /// scalar slot).
    std::vector<RrArraySlot> arrays;
    std::vector<int32_t> arrayOf;
    /// The float2 and float3 pool entries an array value names (a default
    /// or a read's constant), in the runtime's vector types, by pool id;
    /// converted once at Open.
    std::map<uint32_t, std::vector<RrVec2f>> vec2Pool;
    std::map<uint32_t, std::vector<RrVec3f>> vec3Pool;
    std::map<uint32_t, std::vector<uint32_t>> tokenArrayPool;

    /// The avar table's reads (RigExecBakedProgramImpl::avarBindings and
    /// avarConstantBindings), as RrProgram::registeredReads indices,
    /// bucketed by provider slot (avar / 11): provider p's reads are
    /// avarReads[avarReadBegin[p], avarReadBegin[p + 1]), its varying ones
    /// then its constant ones, each in file order -- the order an
    /// AvarInputs step walks them in. Read through RrAvarReadRange.
    std::vector<uint32_t> avarReadBegin;
    std::vector<uint32_t> avarReads;
    /// The override numbers of the provider ladders' inputs, sorted
    /// (RigExecBakedProgramImpl::ladderOverrides).
    std::vector<int32_t> ladderOverrides;
    /// Per slot, the override numbers of the program reads whose walk
    /// passes it (RigExecBakedProgramImpl::overridableInputs over slots):
    /// slotOverrideNumbers[slotOverrideBegin[s], slotOverrideBegin[s + 1]).
    std::vector<uint32_t> slotOverrideBegin;
    std::vector<int32_t> slotOverrideNumbers;
    /// The inverse: per override number, the slots of the walk of every
    /// registered read carrying it, sorted:
    /// overrideSlotList[overrideSlotBegin[n], overrideSlotBegin[n + 1]).
    std::vector<uint32_t> overrideSlotBegin;
    std::vector<uint32_t> overrideSlotList;
    /// Per override number: whether its walk holds an Animated slot or a
    /// slot a property result is published at, where the long-way read of
    /// a standing override can move with time or with the chains while no
    /// slot is set.
    std::vector<char> overrideWalkMoves;
    /// Per override number: whether a non-Animated slot of its walk holds a
    /// value other than its default, which the next run copies into
    /// RrStore::overridden. Sized at Open with the store's flags.
    std::vector<char> valueOverridden;
    /// Slots set or reset since the last run, each listed once.
    std::vector<uint32_t> touched;
    std::vector<char> touchedFlag;
    /// Slots whose value, presence, authorship or blocked flag any call
    /// wrote, moved or not, since the op graph's leaf publication last
    /// consumed them (RrInputsMarkWritten), each listed once.
    std::vector<uint32_t> written;
    std::vector<char> writtenFlag;
    /// Per slot: the value and the keyed bits (bit 0 HasValue, 1 authored,
    /// 2 blocked) the slot's readers were last keyed from, which
    /// RrInputsFilterWritten compares a written scalar slot with. Sized at
    /// Open with the slots.
    std::vector<RrWireValue> keyedValue;
    std::vector<uint8_t> keyedBits;
    /// RIGEXEC_RUNTIME_WRITTEN_FILTER, read at Open: off, the filter only
    /// records and drops nothing.
    bool filterWritten = true;
    /// Per listed input (index = slot id < file->listedInputs): what
    /// GetInputInfo reports, and the value GetInputValue reports.
    std::vector<RigExecRuntimeInputInfo> inputInfo;
    std::vector<RrInputValue> inputValues;
    /// Token text SetInputToken met that the file does not hold, at ids
    /// extraTokenBase + k.
    std::vector<std::string> extraTokens;
    /// Token text -> id: the empty token at 0 and each Token node's text
    /// (the first node wins), built at Open, then each text SetInputToken
    /// interned.
    std::unordered_map<std::string, uint32_t> tokenIds;
    /// Attribute path text -> slot id, for the input API.
    std::unordered_map<std::string, uint32_t> nameIndex;
};

/// Provider slot \p provider's entries of RrInputState::avarReads, as
/// [first, second) positions: exactly the reads whose avar / 11 is
/// \p provider. Empty for a negative provider or one past every avar.
inline std::pair<uint32_t, uint32_t>
RrAvarReadRange(const RrInputState &state, int64_t provider)
{
    if (provider < 0 || size_t(provider) + 1 >= state.avarReadBegin.size()) {
        return {0, 0};
    }
    return {state.avarReadBegin[size_t(provider)],
            state.avarReadBegin[size_t(provider) + 1]};
}

/// Attaches \p file's slots to the program: the value pool inline, each
/// slot at its bake-time default, the token index; and checks what the
/// tables need of the weight objects: for every constraint whose envelope
/// arm resolves a weight object, the entry of that same object; a weight
/// object for every current-phase revision; float reads only. Runs at
/// Open after the pose and geometry tables are attached.
bool RrInputsOpen(RrProgram *program, const RigExecWireFile *file,
                  std::string *error);

/// Binds the tables' reads in place: every Input of the ladders, the space
/// switches, the interpolators, the solvers, the constraints and the
/// step-backed weight objects, and the avar bindings split by their
/// Varying flag, each named by its table field. Indexes the ladder's
/// override numbers, the override numbers by slot and the slots by
/// override number, the slot names and the listed inputs. Runs after
/// RrInputsOpen attached the slots; every read below relies on that.
bool RrInputsBindReads(RrProgram *program, std::string *error);

/// Input \p index (a listed slot) becomes \p value, as authored: the slot
/// holds it, and the next run applies it (RrInputsApplyTouched). A Double
/// value sets a Float input through static_cast<float>. False with the
/// reason, nothing changed, for an index past the listed inputs, an array
/// input, any other type mismatch, a non-finite component unless
/// \p acceptNonFinite (a stage's own value, which a bake-time default can
/// hold too), or a Token id that names no text. Setting the value the input
/// holds still marks it.
bool RrInputsSet(RrProgram *program, size_t index, const RrInputValue &value,
                 std::string *error, bool acceptNonFinite = false);

/// Array input \p index (a listed slot) holds a copy of \p value's
/// elements: an \p authored set (SetInputArray), which every read takes and
/// which keeps the default's element count, or a sampled one, of any
/// count but a topology input's, which only the reads at the evaluation
/// time take; each reader judges the elements as the evaluators judge the
/// stage's. A set whose elements equal the held ones only takes the set's
/// kind; one equal to the default holds nothing. False with the reason,
/// nothing changed, for an index past the listed inputs, a scalar input,
/// another tag, elements without data, an authored count other than the
/// default's, or a topology input's count other than the default's.
bool RrInputsSetArray(RrProgram *program, size_t index,
                      const RigExecRuntimeArray &value, bool authored,
                      std::string *error);

/// Internal sampled-array transport over actual AtTime consumers.
std::vector<size_t> RrStageArraySlots(const RrProgram *program);
bool RrStageArraySet(RrProgram *program, size_t slot,
                     const RigExecRuntimeArray &value, std::string *error);
bool RrStageTokenArraySet(RrProgram *, size_t, const std::vector<std::string> &, std::string *);
bool RrStageInputBlockedSet(RrProgram *, size_t, bool, std::string *);
bool RrStageScalarSet(RrProgram *,size_t,const RrInputValue &,std::string *);
bool RrStageScalarClear(RrProgram *,size_t,std::string *);
bool RrStageArrayClear(RrProgram *program, size_t slot, std::string *error);

/// Array input \p index's elements as the reads see them: its held set,
/// else its default; valid until the next set or reset of that input.
/// False for an index past the listed inputs or a scalar input.
bool RrInputsGetArray(const RrProgram *program, size_t index,
                      RigExecRuntimeArray *out);

/// Array slot \p slot's default is the const std::vector \p vector, of
/// its tag's element type: a stored skin layout's expansion, which the
/// geometry family builds at Open.
void RrInputsBindArrayDefault(RrProgram *program, uint32_t slot,
                              const void *vector);

/// Array slot \p slot carries topology: a skin's joint indices, a mesh's
/// face counts or indices, a curve's order or knots, or a sparse blend
/// shape's offsets or point indices. Topology is epoch state and the reader
/// never recompiles, so its element count stays the default's: a sampled
/// set of another count is refused (RrInputsSetArray) as an authored one
/// is. The geometry family marks its topology reads' slots at Open; a
/// scalar slot is left alone.
void RrInputsMarkTopology(RrProgram *program, uint32_t slot);

/// After the families bound the layout defaults: every array slot has a
/// default, and the listed ones report its element count. False with the
/// reason otherwise.
bool RrInputsFinishArrays(RrProgram *program, std::string *error);

/// Array slot \p slot's elements this run (its held set, else its default)
/// as a const std::vector of \p tag's element type; null for a slot that
/// is not an array of that tag.
const void *RrInputArrayValue(const RrProgram *program, uint32_t slot,
                              RigExecWireInputTag tag);

/// The same for array slot \p slot's default.
const void *RrInputArrayDefault(const RrProgram *program, uint32_t slot,
                                RigExecWireInputTag tag);

template <class T>
const std::vector<T> *
RrInputArray(const RrProgram *program, uint32_t slot)
{
    return static_cast<const std::vector<T> *>(
        RrInputArrayValue(program, slot, RrArrayTag<T>::value));
}

template <class T>
const std::vector<T> *
RrInputArrayDefaultOf(const RrProgram *program, uint32_t slot)
{
    return static_cast<const std::vector<T> *>(
        RrInputArrayDefault(program, slot, RrArrayTag<T>::value));
}

/// Whether array slot \p slot holds an authored set, which Default-time
/// reads take.
bool RrInputArrayAuthored(const RrProgram *program, uint32_t slot);

/// Array slot \p slot's content version (RrArraySlot::version); 0 for a
/// slot that is not an array.
uint64_t RrInputArrayVersion(const RrProgram *program, uint32_t slot);

/// Whether slot \p slot holds a value this run (HasValue).
bool RrInputHasValue(const RrProgram *program, uint32_t slot);

/// Whether slot \p slot holds other than its default: another HasValue, or
/// other bits (a scalar) or a held set (an array).
bool RrInputDiffersFromDefault(const RrProgram *program, uint32_t slot);

/// Whether the inputs this run applied changed slot \p slot
/// (RrInputState::slotChangedSinceRun).
bool RrInputChangedSinceRun(const RrProgram *program, uint32_t slot);

/// The slot array read \p read answers from this run: Raw, its head when
/// that holds a value; Resolved, the most upstream hop that does (no
/// overlay holds arrays). -1 when none does, and the read takes its
/// constant.
int32_t RrArrayReadSlot(const RrProgram *program,
                        const RigExecWireInput &read);

/// Array read \p read's elements this run: RrArrayReadSlot's, else its
/// constant's, as a const std::vector of \p tag's element type; null when
/// \p read is not an array read of that tag.
const void *RrArrayReadValue(const RrProgram *program,
                             const RigExecWireInput &read,
                             RigExecWireInputTag tag);

template <class T>
const std::vector<T> *
RrArrayRead(const RrProgram *program, const RigExecWireInput &read)
{
    return static_cast<const std::vector<T> *>(
        RrArrayReadValue(program, read, RrArrayTag<T>::value));
}

/// Path-read row \p row's elements this run, as a const std::vector of
/// \p tag's element type; null when the row holds no array of that tag.
/// A value row's static array; a live array read's RrArrayReadValue,
/// which takes any value its slots hold; a rest one's held value only
/// while an authored set stands on its slot, else its Default-time value.
const void *RrPathArrayValue(const RrProgram *program, const RrPathRead &row,
                             RigExecWireInputTag tag);

template <class T>
const std::vector<T> *
RrPathArray(const RrProgram *program, const RrPathRead &row)
{
    return static_cast<const std::vector<T> *>(
        RrPathArrayValue(program, row, RrArrayTag<T>::value));
}

/// Token input \p index becomes \p text: text the file holds (the empty
/// token, or a Token node's text) sets its id; other text is interned at
/// extraTokenBase + k first, once. False with the reason, nothing changed,
/// for an index past the listed inputs or an input that is not a Token.
bool RrInputsSetToken(RrProgram *program, size_t index,
                      const std::string &text, std::string *error);
// Internal sampling only; private tokens remain outside authored APIs.
bool RrStageTokenSet(RrProgram *program, size_t slot,
                     const std::string &text, bool hasValue,
                     std::string *error);

/// Input \p index (a listed slot) returns to its bake-time default, value
/// and HasValue alike (an array input also drops its authored mark), and
/// is marked for the next run when it held anything else.
void RrInputsReset(RrProgram *program, size_t index);

/// Input \p index (a listed slot) holds no value, as an attribute whose
/// typed read fails (a blocked sample, or Default on an attribute keyed
/// alone): every read falls back as it does over such an attribute. The
/// slot keeps its default's bits, and is marked for the next run. False
/// with the reason for an index past the listed inputs or an array input,
/// which ResetInput restores instead.
bool RrInputsClear(RrProgram *program, size_t index, std::string *error);

/// What the inputs set or reset since the last run change for this one,
/// at the start of a run: an Animated slot sets RrStore::animatedTouched,
/// which dirties what time dirties; any other scalar slot recomputes, for
/// each override number its walk carries, whether a non-Animated slot of
/// that walk differs bitwise from its default (HasValue included), so a
/// value written back to its default clears the flag. Then
/// RrStore::overridden takes those flags and the marks are cleared. A slot
/// whose value or HasValue differs bitwise from what the last run read
/// also sets RrStore::changedSinceRun for each of its override numbers, and
/// this run becomes the one it is compared with; a set that repeats the
/// value sets nothing. Every slot that changed so is listed in
/// slotChangedSinceRun; an array slot does nothing else, since every
/// reader of an array runs on every Execute or compares by value.
void RrInputsApplyTouched(RrProgram *program);

/// Lists slot \p slot in RrInputState::written. Every write to a slot's
/// value, HasValue, authored mark or blocked flag calls it, whether or not
/// anything moved, so a leaf keyed from input slots alone keeps its key
/// exactly until a slot it reads is written.
inline void
RrInputsMarkWritten(RrInputState &state, size_t slot)
{
    if (slot < state.writtenFlag.size() && !state.writtenFlag[slot]) {
        state.writtenFlag[slot] = 1;
        state.written.push_back(uint32_t(slot));
    }
}

/// Drops from `written` every scalar slot whose value bits, HasValue,
/// authored mark and blocked flag equal what its readers were last keyed
/// from, and records the rest as keyed. Array slots always stay. Called
/// where a run consumes `written`; \p all records every slot (a full
/// publication) and drops nothing.
void RrInputsFilterWritten(RrInputState *state, bool all);

/// Whether the resolved inputs hold a value at attribute \p path this run:
/// a property-chain result (RigExecResolvedInputs::Find).
bool RrInputsPublished(const RrProgram *program, uint32_t path);

/// RigExecResolvedInputs::Get<T> at attribute \p path: the property-chain
/// result published there this run, when it is exactly \p tag. \p out is
/// written only on a hit. Every walk's hop and every geometry scalar read
/// meets this overlay.
bool RrInputsOverlay(const RrProgram *program, uint32_t path,
                     RigExecWireInputTag tag, RrWireValue *out);

/// The value property chain \p chain starts from this run, in the chain's
/// type \p tag: the target's own typed value, which an input set there
/// authors, as in the USD evaluators; false when it holds none of that
/// type. The chain run (properties.cpp) revises it and publishes its own
/// result at the target, and each phased reader reads the chain's history
/// from it, so every reader sees what it sees once that value is authored
/// and rebaked.
bool RrChainBase(const RrProgram *program, size_t chain,
                 RigExecWireInputTag tag, RrWireValue *base);

/// \p input as the program resolves it in its mode, tagged with the read's
/// tag: Baked is RigExecBakedRead (bakedProgramImpl.h), Resolved is
/// RigExecResolvedInputs::GetAttribute (moverGraph.h), Pinned is
/// _PinnedRead (rigEvaluatorProperties.cpp), Raw is the head's own typed
/// value. GetAttribute's overlay at each hop is RrInputsOverlay: the
/// property-chain result published there (RrStore::propertyResults),
/// answering when it holds exactly the walk's type.
RrWireValue RrReadInput(const RrProgram *program,
                        const RigExecWireInput &input);

// API4 scalar declaration evaluation preserves failed reads separately from fallback.
bool RrEffectiveInputMemo(const RrProgram *program,uint32_t step,std::string *key,
                         std::vector<uint32_t> *coveredPropertyVersions,
                         std::vector<std::pair<uint32_t,uint32_t>> *coveredTyped = nullptr);
void RrEffectiveReadMemo(const RrProgram *program,const RigExecWireInput &read,std::string *key);
/// \p versioned keys a numeric array slot's current elements by their
/// content version and its immutable default by nothing, for a memo that
/// every run rebuilds and compares with the previous run's only.
void RrSourceReadMemo(const RrProgram *program,const RigExecWireInput &read,std::string *key,
                      bool versioned=false);
bool RrReadExternalScalar(const RrProgram *, const RigExecWireExternalDeclaredInput &,
                          RrWireValue *);
const void *RrReadExternalArray(const RrProgram *, const RigExecWireExternalDeclaredInput &);

/// A connection-following assembler read (a read row of path_reads):
/// RrReadInput, and for a site that reads the head's own value when the
/// walk yields nothing (`head_fallback`), that value before the read's
/// constant.
RrWireValue RrReadPathScalar(const RrProgram *program,
                             const RigExecWirePathRead &row);

/// \p value of read row \p row in the form the assemblers read stage
/// values in: the read's own type. A double site widens a Float value
/// itself. False for a type no scalar site reads.
bool RrPathValueFromWire(RigExecWireInputTag tag,const RrWireValue &value,RrPathValue *out);
bool RrPathValueFromRead(const RigExecWirePathRead &row,
                         const RrWireValue &value, RrPathValue *out);

/// A blend channel's weight as the assembly reads it: a pose-driven
/// channel's pose weight unless a value is published at the channel's own
/// weight (bakedGeometry.cpp's `!R.Find(weightPath)`), else its
/// inputs:weight read. Geometry chain \p chain, revision \p revision (or
/// derived target \p revision when \p derived), binding channel
/// \p channel; 0 when the file holds no such channel.
float RrReadBlendWeight(const RrProgram *program, size_t chain,
                        size_t revision, bool derived, size_t channel);

/// The rigExec:activation of sample \p sample of that blend channel, as
/// the gather reads it through the resolved inputs; 1 when the file holds
/// no such sample.
float RrReadBlendActivation(const RrProgram *program, size_t chain,
                            size_t revision, bool derived, size_t channel,
                            size_t sample);

/// A chain revision's inputs:defaultWeight as RevisionStatic reads it; 1
/// when the revision carries no read for it.
float RrReadDefaultWeight(const RrProgram *program, size_t chain,
                          size_t revision);

/// The weight oracle's _ResolvedRead<float> (rigEvaluatorInternal.h):
/// RigExecResolvedInputs::GetAttribute<float> over the read's walk,
/// whatever the read's mode, with the overlay above, and \p fallback (the
/// oracle's own fallback at that read site) when the walk yields nothing.
/// A double hop is read as a double walk from that hop and cast with
/// static_cast<float>.
struct RrOracleReadContext;
float RrReadOracleFloat(const RrProgram *program, const RigExecWireInput &input,
                       const RrOracleReadContext &context, float fallback);

float RrReadResolvedFloat(const RrProgram *program,
                          const RigExecWireInput &input, float fallback);

/// The float a Float-tagged value holds (bits in the low 32).
float RrWireValueFloat(const RrWireValue &value);

}  // namespace rigExec

#endif  // RIGEXEC_RUNTIME_INPUTS_H
