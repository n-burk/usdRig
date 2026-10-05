// rigExecRuntime input slot model and read evaluation.
// The slots, their bake-time defaults and every computed read come from
// the temporary Computed section (rigExecBinary/computed.h). A slot's
// value is mutable: an input set through the reader is the attribute's
// authored value. A read is a pure function of the slot values, the
// override flags the set inputs raise and the property-chain results the
// run published, so it is safe wherever a step body may run. USD-free.
#ifndef RIGEXEC_RUNTIME_INPUTS_H
#define RIGEXEC_RUNTIME_INPUTS_H

#include "rigExecBinary/computed.h"
#include "rigExecRuntime/values.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace rigExec {

struct RrProgram;

/// The first token id SetInputToken gives text the string table lacks;
/// RrProgram::GetText resolves ids from here through
/// RrInputState::extraTokens. Above every string-table index.
inline constexpr uint32_t RrExtraTokenBase = 0x80000000u;

/// The slot model, over the section `computed` borrows from the reader.
struct RrInputState {
    const RigExecWireComputed *computed = nullptr;
    /// Per slot: the value every read sees this run, and whether it holds
    /// one (the attribute's typed read succeeded, or a value was set).
    /// Seeded with the bake-time defaults (values[slot.value] and the
    /// HasValue flag, kept in slotDefaultHasValue).
    std::vector<v4::RigExecWireValue> slotCurrent;
    std::vector<uint8_t> slotHasValue;
    std::vector<uint8_t> slotDefaultHasValue;

    /// The avar table's registered reads (RigExecBakedProgramImpl::
    /// avarBindings and avarConstantBindings), in section order.
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
    /// Per listed input (index = slot id < computed->listedInputs): what
    /// GetInputInfo reports, and the value GetInputValue reports.
    std::vector<RigExecRuntimeInputInfo> inputInfo;
    std::vector<RrInputValue> inputValues;
    /// Token text SetInputToken met that the string table lacks, at ids
    /// RrExtraTokenBase + k.
    std::vector<std::string> extraTokens;
    /// Token text -> id over the string table and extraTokens, built on
    /// the first SetInputToken.
    std::unordered_map<std::string, uint32_t> tokenIds;
    bool tokenIdsBuilt = false;
    /// Attribute path text -> slot id, for the input API.
    std::unordered_map<std::string, uint32_t> nameIndex;

    /// Per geometry chain and revision, the index of its default weight
    /// read, or -1; per chain and revision (and per chain and derived
    /// target), per binding channel, the index of its blend weight read.
    std::vector<std::vector<int32_t>> defaultWeightRead;
    std::vector<std::vector<std::vector<int32_t>>> blendReads;
    std::vector<std::vector<std::vector<int32_t>>> derivedBlendReads;
    /// The path reads by head path: (path id, pathScalarReads index),
    /// sorted by path id.
    std::vector<std::pair<uint32_t, uint32_t>> pathReadIndex;
};

/// Attaches \p computed to the program and checks it covers what the
/// tables need: for every constraint whose envelope arm resolves a weight
/// object, the entry of that same object; a weight object for every
/// current-phase revision; float reads only. Refuses a file without the
/// section: every input the steps read is evaluated through it. Runs at
/// Open after the pose, geometry and input tables are attached.
bool RrInputsOpen(RrProgram *program, const RigExecWireComputed *computed,
                  std::string *error);

/// Binds the section's registered reads to the runtime's tables: each
/// names a table field the runtime reads, with that input's tag, override
/// number and per-run liveness, and every field of every table row is
/// named exactly once; the override numbers the reads reach at each
/// attribute are the ones the program registered there. Indexes the avar
/// bindings, the ladder's override numbers, the override numbers by slot
/// and the slots by override number, the slot names, the listed inputs and
/// the geometry reads by revision and channel. Runs after RrInputsOpen
/// attached the section; every read below relies on that.
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

/// Token input \p index becomes \p text: text the string table holds sets
/// its id; other text is interned at RrExtraTokenBase + k first, once.
/// False with the reason, nothing changed, for an index past the listed
/// inputs or an input that is not a Token.
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
                     v4::InputTag tag, v4::RigExecWireValue *out);

/// The value property chain \p chain starts from this run, in the chain's
/// type \p tag: the target's own typed value, which an input set there
/// authors, as in the USD evaluators; false when it holds none of that
/// type. The chain run (properties.cpp) revises it and publishes its own
/// result at the target, and each phased reader reads the chain's history
/// from it, so every reader sees what it sees once that value is authored
/// and rebaked.
bool RrChainBase(const RrProgram *program, size_t chain, v4::InputTag tag,
                 v4::RigExecWireValue *base);

/// \p input as the program resolves it in its mode, tagged with the read's
/// tag: Baked is RigExecBakedRead (bakedProgramImpl.h), Resolved is
/// RigExecResolvedInputs::GetAttribute (moverGraph.h), Pinned is
/// _PinnedRead (rigEvaluatorProperties.cpp), Raw is the head's own typed
/// value. GetAttribute's overlay at each hop is RrInputsOverlay: the
/// property-chain result published there (RrStore::propertyResults),
/// answering when it holds exactly the walk's type.
v4::RigExecWireValue RrReadInput(const RrProgram *program,
                                 const v4::RigExecWireInput &input);

/// A connection-following assembler read: RrReadInput, and for a site
/// that reads the head's own value when the walk yields nothing
/// (`headFallback`), that value before the read's constant.
v4::RigExecWireValue RrReadPathScalar(const RrProgram *program,
                                      const RigExecWirePathScalarRead &entry);

/// \p value of path read \p entry in the form the assemblers read stage
/// values in: its own type, or a double for a site that widens a float.
/// False for a type no scalar site reads.
bool RrPathValueFromRead(const RigExecWirePathScalarRead &entry,
                         const v4::RigExecWireValue &value,
                         RigExecWirePathValue *out);

/// A blend channel's weight as the assembly reads it: a pose-driven
/// channel's pose weight unless a value is published at the channel's own
/// weight (bakedGeometry.cpp's `!R.Find(weightPath)`), else its
/// inputs:weight read. Geometry chain \p chain, revision \p revision (or
/// derived target \p revision when \p derived), binding channel
/// \p channel; 0 when the section carries no read for it.
float RrReadBlendWeight(const RrProgram *program, size_t chain,
                        size_t revision, bool derived, size_t channel);

/// The rigExec:activation of sample \p sample of that blend channel, as
/// the gather reads it through the resolved inputs; 1 when the section
/// carries no read for it.
float RrReadBlendActivation(const RrProgram *program, size_t chain,
                            size_t revision, bool derived, size_t channel,
                            size_t sample);

/// A chain revision's inputs:defaultWeight as RevisionStatic reads it; 1
/// when the section carries no read for it.
float RrReadDefaultWeight(const RrProgram *program, size_t chain,
                          size_t revision);

/// The weight oracle's _ResolvedRead<float> (rigEvaluatorInternal.h):
/// RigExecResolvedInputs::GetAttribute<float> over the read's walk,
/// whatever the read's mode, with the overlay above, and \p fallback (the
/// oracle's own fallback at that read site) when the walk yields nothing.
/// A double hop is read as a double walk from that hop and cast with
/// static_cast<float>.
float RrReadResolvedFloat(const RrProgram *program,
                          const v4::RigExecWireInput &input, float fallback);

/// The float a Float-tagged wire value holds (bits in the low 32).
float RrWireValueFloat(const v4::RigExecWireValue &value);

}  // namespace rigExec

#endif  // RIGEXEC_RUNTIME_INPUTS_H
