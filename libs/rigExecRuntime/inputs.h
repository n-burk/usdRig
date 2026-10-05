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
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace rigExec {

struct RrProgram;

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
    case RigExecWireInputTag::Vec3f:
        out.vec3f = RrVec3f(value.vec3f[0], value.vec3f[1], value.vec3f[2]);
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
    std::vector<RigExecWireVec2f> vec2s;
    std::vector<RigExecWireVec3f> vec3s;
    RigExecWireVec3i vec3i{{0, 0, 0}};
    std::vector<double> doubles;
};

/// One row of the path-read table (RrProgram::pathReads), keyed by the
/// attribute's path id and whether the site read its rest (Default) value.
/// A value row holds what the bake captured. A read row holds the head of
/// a connection-following read, whose value every geometry prologue
/// evaluates over the slots.
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
/// row per static value, decoded out of the pools, and a read row per
/// connection-following read. False with the reason for a read whose type
/// no assembler site reads.
bool RrInputsBuildPathReads(RrProgram *program, std::string *error);

/// The slot model over the opened file.
struct RrInputState {
    const RigExecWireFile *file = nullptr;
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
    std::vector<uint8_t> slotDefaultHasValue;

    /// The avar table's reads (RigExecBakedProgramImpl::avarBindings and
    /// avarConstantBindings), as RrProgram::registeredReads indices, in
    /// file order.
    std::vector<uint32_t> avarBindingReads;
    std::vector<uint32_t> avarConstantReads;
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
    /// Per override number: whether a non-Animated slot of its walk holds a
    /// value other than its default, which the next run copies into
    /// RrStore::overridden. Sized at Open with the store's flags.
    std::vector<char> valueOverridden;
    /// Slots set or reset since the last run, each listed once.
    std::vector<uint32_t> touched;
    std::vector<char> touchedFlag;
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
/// reason, nothing changed, for an index past the listed inputs, any other
/// type mismatch, a non-finite component unless \p acceptNonFinite (a
/// stage's own value, which a bake-time default can hold too), or a Token
/// id that names no text. Setting the value the input holds still marks
/// it.
bool RrInputsSet(RrProgram *program, size_t index, const RrInputValue &value,
                 std::string *error, bool acceptNonFinite = false);

/// Token input \p index becomes \p text: text the file holds (the empty
/// token, or a Token node's text) sets its id; other text is interned at
/// extraTokenBase + k first, once. False with the reason, nothing changed,
/// for an index past the listed inputs or an input that is not a Token.
bool RrInputsSetToken(RrProgram *program, size_t index,
                      const std::string &text, std::string *error);

/// Input \p index (a listed slot) returns to its bake-time default, value
/// and HasValue alike, and is marked for the next run.
void RrInputsReset(RrProgram *program, size_t index);

/// Input \p index (a listed slot) holds no value, as an attribute whose
/// typed read fails (a blocked sample, or Default on an attribute keyed
/// alone): every read falls back as it does over such an attribute. The
/// slot keeps its default's bits, and is marked for the next run. False
/// with the reason for an index past the listed inputs.
bool RrInputsClear(RrProgram *program, size_t index, std::string *error);

/// What the inputs set or reset since the last run change for this one,
/// at the start of a run: an Animated slot sets RrStore::animatedTouched,
/// which dirties what time dirties; any other slot recomputes, for each
/// override number its walk carries, whether a non-Animated slot of that
/// walk differs bitwise from its default (HasValue included), so a value
/// written back to its default clears the flag. Then RrStore::overridden
/// takes those flags and the marks are cleared.
void RrInputsApplyTouched(RrProgram *program);

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

/// A connection-following assembler read (a read row of path_reads):
/// RrReadInput, and for a site that reads the head's own value when the
/// walk yields nothing (`head_fallback`), that value before the read's
/// constant.
RrWireValue RrReadPathScalar(const RrProgram *program,
                             const RigExecWirePathRead &row);

/// \p value of read row \p row in the form the assemblers read stage
/// values in: the read's own type. A double site widens a Float value
/// itself. False for a type no scalar site reads.
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
float RrReadResolvedFloat(const RrProgram *program,
                          const RigExecWireInput &input, float fallback);

/// The float a Float-tagged value holds (bits in the low 32).
float RrWireValueFloat(const RrWireValue &value);

}  // namespace rigExec

#endif  // RIGEXEC_RUNTIME_INPUTS_H
