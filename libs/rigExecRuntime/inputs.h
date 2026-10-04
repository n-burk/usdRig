// rigExecRuntime input slot model and read evaluation.
// The slots, their per-frame values and every computed read come from the
// temporary Computed section (rigExecBinary/computed.h). A read is a pure
// function of the slot values and the property-chain results the run
// selected, so it is safe wherever a step body may run. USD-free.
#ifndef RIGEXEC_RUNTIME_INPUTS_H
#define RIGEXEC_RUNTIME_INPUTS_H

#include "rigExecBinary/computed.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace rigExec {

struct RrProgram;

/// The slot model. `computed` borrows the reader's decoded section; null
/// when the file carries none, in which case nothing reads a slot (Open
/// refuses a file whose envelopes or current-phase fields need one).
struct RrInputState {
    const RigExecWireComputed *computed = nullptr;
    /// Per slot: the values[] id this run reads, and whether the
    /// attribute's typed read succeeded. Seeded with the bake-time
    /// defaults; each Execute loads its frame's row while the section
    /// carries per-frame values.
    std::vector<uint32_t> slotValue;
    std::vector<uint8_t> slotHasValue;

    /// One registered read bound to what the record-driven steps consume
    /// for it: `wire` is its table input (null for an avar binding, whose
    /// value is the avar table's), `uid` its holder (-1: the constant),
    /// `avar` an avar binding's flat index. `live` reads per run, so the
    /// record holds its value only at the frames a step read it;
    /// `chainRead` is compared by the chain-read hand-off instead.
    struct BoundRead {
        const RigExecWireInput *wire = nullptr;
        int32_t uid = -1;
        int32_t avar = -1;
        bool live = false;
        bool chainRead = false;
    };
    std::vector<BoundRead> registered;
    /// Per geometry chain and revision, the index of its default weight
    /// read, or -1; per chain and revision (and per chain and derived
    /// target) the indices of its blend weight reads.
    std::vector<std::vector<int32_t>> defaultWeightRead;
    std::vector<std::vector<std::vector<uint32_t>>> blendReads;
    std::vector<std::vector<std::vector<uint32_t>>> derivedBlendReads;
    /// The path reads by head path: (path id, pathScalarReads index),
    /// sorted by path id.
    std::vector<std::pair<uint32_t, uint32_t>> pathReadIndex;
};

/// Attaches \p computed (or nothing) to the program and checks it covers
/// what the tables need: for every constraint whose envelope arm resolves
/// a weight object, the entry of that same object; a weight object for
/// every current-phase revision; float reads only. Runs at Open after the
/// pose, geometry and input tables are attached.
bool RrInputsOpen(RrProgram *program, const RigExecWireComputed *computed,
                  std::string *error);

/// Binds the section's registered reads to the runtime's tables after the
/// uid replay: each names a field the runtime reads, with the uid the
/// replay routed there, the same tag and override number and the same
/// per-run liveness; an avar binding's uid routes to its avar. Indexes the
/// geometry reads by revision. A no-op without the section.
bool RrInputsBindReads(RrProgram *program, std::string *error);

/// Loads InputTable frame \p index's slot values (the section's per-frame
/// rows, aligned with the InputTable frames). A no-op without the section.
bool RrInputsSelectFrame(RrProgram *program, size_t index,
                         std::string *error);

/// \p input as the program resolves it in its mode, tagged with the read's
/// tag: Baked is RigExecBakedRead (bakedProgramImpl.h), Resolved is
/// RigExecResolvedInputs::GetAttribute (moverGraph.h), Pinned is
/// _PinnedRead (rigEvaluatorProperties.cpp), Raw is the head's own typed
/// value. GetAttribute's overlay is RrStore::propertyResults: a
/// property-chain result published at a hop answers there when it holds
/// exactly the walk's type.
v4::RigExecWireValue RrReadInput(const RrProgram *program,
                                 const v4::RigExecWireInput &input);

/// A connection-following assembler read: RrReadInput, and for a site
/// that reads the head's own value when the walk yields nothing
/// (`headFallback`), that value before the read's constant.
v4::RigExecWireValue RrReadPathScalar(const RrProgram *program,
                                      const RigExecWirePathScalarRead &entry);

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

/// Cross-check helpers (RrProgram::crossCheck): bitwise float equality,
/// which tells -0 from +0 and compares NaN payloads, and the failure text
/// naming the record field, e.g. "constraintWeights[3]".
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
