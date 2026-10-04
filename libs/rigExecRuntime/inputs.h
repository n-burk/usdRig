// rigExecRuntime input slot model and read evaluation.
// The slots, their per-frame values and every computed read come from the
// temporary Computed section (rigExecBinary/computed.h). A read is a pure
// function of the slot values, the standing interactive overrides and the
// property-chain results the run selected, so it is safe wherever a step
// body may run. USD-free.
#ifndef RIGEXEC_RUNTIME_INPUTS_H
#define RIGEXEC_RUNTIME_INPUTS_H

#include "rigExecBinary/computed.h"

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace rigExec {

struct RrProgram;

/// The slot model, over the section `computed` borrows from the reader.
struct RrInputState {
    const RigExecWireComputed *computed = nullptr;
    /// Per slot: the values[] id this run reads, and whether the
    /// attribute's typed read succeeded. Seeded with the bake-time
    /// defaults; each Execute loads its frame's row while the section
    /// carries per-frame values.
    std::vector<uint32_t> slotValue;
    std::vector<uint8_t> slotHasValue;

    /// What the cross-check compares each registered read with: the frame
    /// record's value under `uid` where the record holds one, else the
    /// table input's constant (`wire`; null for an avar binding, whose
    /// constant is the avar table's at `avar`). `live` reads per run, so
    /// the record holds its value only at the frames a step read it;
    /// `chainRead` is compared as a chain read instead.
    struct BoundRead {
        const RigExecWireInput *wire = nullptr;
        int32_t uid = -1;
        int32_t avar = -1;
        bool live = false;
        bool chainRead = false;
    };
    std::vector<BoundRead> registered;
    /// The avar table's registered reads (RigExecBakedProgramImpl::
    /// avarBindings and avarConstantBindings), in section order.
    std::vector<uint32_t> avarBindingReads;
    std::vector<uint32_t> avarConstantReads;
    /// The override numbers of the provider ladders' inputs, sorted
    /// (RigExecBakedProgramImpl::ladderOverrides).
    std::vector<int32_t> ladderOverrides;
    /// Per slot: the flat avar index (slot * 11 + channel) of the avar
    /// binding the slot heads, or -1.
    std::vector<int32_t> slotAvar;
    /// Per slot, the override numbers of the program reads whose walk
    /// passes it (RigExecBakedProgramImpl::overridableInputs over slots):
    /// slotOverrideNumbers[slotOverrideBegin[s], slotOverrideBegin[s + 1]).
    std::vector<uint32_t> slotOverrideBegin;
    std::vector<int32_t> slotOverrideNumbers;
    /// Attribute path text -> slot id, for the override API.
    std::unordered_map<std::string, uint32_t> nameIndex;
    /// The interactive overrides standing this run, keyed by attribute path
    /// id: half of the overlay GetAttribute meets at each hop
    /// (RrInputsOverlay).
    std::map<uint32_t, v4::RigExecWireValue> overrides;

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
/// named exactly once. Indexes the avar bindings, the ladder's override
/// numbers, the override numbers by slot, the slot names and the geometry
/// reads by revision and channel. Runs after RrInputsOpen attached the
/// section; every read below relies on that.
bool RrInputsBindReads(RrProgram *program, std::string *error);

/// Loads InputTable frame \p index's slot values (the section's per-frame
/// rows, aligned with the InputTable frames).
bool RrInputsSelectFrame(RrProgram *program, size_t index,
                         std::string *error);

/// Places \p overrides (slot id -> value) as the program places
/// interactive overrides (RigExecBakedProgram::SetOverrides): each slot's
/// value becomes the overlay at its attribute, and every program read
/// whose walk passes the slot is flagged overridden, so it reads the long
/// way. Clears the previous run's overrides first.
void RrInputsSetOverrides(RrProgram *program,
                          const std::map<uint32_t, double> &overrides);

/// Whether an interactive override stands on one of \p slots: a phased
/// reader whose consumer or hop holds one stands aside.
bool RrInputsAnyOverridden(const RrProgram *program,
                           const std::vector<uint32_t> &slots);

/// Whether the resolved inputs hold a value at attribute \p path this run:
/// an interactive override or a property-chain result
/// (RigExecResolvedInputs::Find).
bool RrInputsPublished(const RrProgram *program, uint32_t path);

/// RigExecResolvedInputs::Get<T> at attribute \p path: the value the
/// resolved inputs hold there this run (a standing interactive override or
/// a property-chain result, ranked by RrOverrideOutranksResult), when it
/// is exactly \p tag. \p out is written only on a hit. Every walk's hop and
/// every geometry scalar read meets this overlay.
bool RrInputsOverlay(const RrProgram *program, uint32_t path,
                     v4::InputTag tag, v4::RigExecWireValue *out);

/// The runtime's rule for an interactive override on a property-chain
/// target (a slot whose InputSlot::chain >= 0). Every site that meets such
/// an override asks one of these three, so a change of rule stays here.
/// Until the semantics of a target drag are settled the runtime refuses
/// one (the USD evaluators let it replace the chain's final value):
/// - RrTargetDragAccepted: whether SetAvar may place an override on
///   \p slot; false for a chain target.
/// - RrChainBase: the value property chain \p chain starts from this run,
///   its target's own typed value (\p tag); false when that read fails.
///   No override stands on a target, so none is consulted.
/// - RrOverrideOutranksResult: whether an override standing at attribute
///   \p path answers there before a chain result published at the same
///   attribute, as the program's post-chain placement of its overrides
///   makes it. With target drags refused the two never meet at one
///   attribute: an override on a phased reader's consumer stands the
///   reader aside, so it publishes nothing there.
/// A rule that accepts target drags also decides what a phased reader of
/// a dragged chain publishes; the chain run (properties.cpp) publishes the
/// chain's own history today, and the wire carries no `final` flag.
bool RrTargetDragAccepted(const RrProgram *program, uint32_t slot);
bool RrChainBase(const RrProgram *program, size_t chain, v4::InputTag tag,
                 v4::RigExecWireValue *base);
bool RrOverrideOutranksResult(const RrProgram *program, uint32_t path);

/// \p input as the program resolves it in its mode, tagged with the read's
/// tag: Baked is RigExecBakedRead (bakedProgramImpl.h), Resolved is
/// RigExecResolvedInputs::GetAttribute (moverGraph.h), Pinned is
/// _PinnedRead (rigEvaluatorProperties.cpp), Raw is the head's own typed
/// value. GetAttribute's overlay at each hop is RrInputsOverlay: the
/// interactive override standing there or the property-chain result
/// published there (RrStore::propertyResults), answering when it holds
/// exactly the walk's type.
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

/// Cross-check helpers (RrProgram::CrossCheckThisRun): bitwise float
/// equality, which tells -0 from +0 and compares NaN payloads, and the
/// failure text naming the record field, e.g. "constraintWeights[3]".
bool RrSameFloatBits(float a, float b);
std::string RrCrossCheckMismatch(const std::string &field, float computed,
                                 float recorded);
std::string RrCrossCheckMismatch(const std::string &field, double computed,
                                 double recorded);
std::string RrCrossCheckMismatch(const std::string &field,
                                 const std::string &computed,
                                 const std::string &recorded);

}  // namespace rigExec

#endif  // RIGEXEC_RUNTIME_INPUTS_H
