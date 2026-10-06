// .rigexec PropertyChains section: the property chains as programs, so the
// runtime computes them from live inputs instead of replaying the values a
// bake recorded.
//
// Optional. A file without it replays every chain's recorded value per
// frame, which is what every binary written before the section does. The
// bake writes it only for a poseable bake (overridable inputs), because
// only a client that sets inputs needs chains that respond to them.
//
// A chain is its target property, its value type, the target's authored
// base at each baked frame, and its revisions in order. Each revision input
// is folded to a constant, or kept as a WALK: the attribute paths its read
// follows through authored connections, head first, plus the value the read
// falls back to at each baked frame when no step of the walk has a live
// value. That is the read RigExecResolvedInputs::GetAttribute performs, so
// the runtime resolves an input exactly as the evaluator does: the first
// walk step holding a live value (an input the client set, or a chain
// result computed earlier in the same run) wins, else the recorded
// fallback.
//
// Consumers name the input holders a chain's result feeds -- every
// directory entry whose head is a chain target, or whose head's walk
// reaches one -- so a computed result reaches the pose program and the
// constraint envelopes the same way a recorded value would.
#ifndef RIGEXEC_BINARY_PROPERTY_CHAINS_H
#define RIGEXEC_BINARY_PROPERTY_CHAINS_H

#include "rigExecBinary/program.h"

#include <cstdint>
#include <string>
#include <vector>

namespace rigExec {

/// One revision input. Scalars travel as f64: every value an input can
/// hold here (bool, float, double) round-trips through it exactly.
struct RigExecWirePropertyChainInput {
    enum class Kind : uint8_t {
        Absent = 0,    ///< the mover has no such attribute: the default
        Constant = 1,  ///< folded at bake
        Walk = 2,      ///< resolved per run from `hops`, then `frameValues`
    };
    Kind kind = Kind::Absent;
    double constant = 0;
    /// String-table paths the read visits, the input's own path first.
    std::vector<uint32_t> hops;
    /// Per baked frame: whether the walk's authored fallback exists, and
    /// its value.
    std::vector<uint8_t> frameHave;
    std::vector<double> frameValues;
};

/// One revision: a float math mover's folded operation and its inputs.
struct RigExecWirePropertyChainRevision {
    uint32_t mover = 0;
    /// RigExecPropertyOp, folded at bake.
    uint8_t op = 0;
    RigExecWirePropertyChainInput enabled;
    RigExecWirePropertyChainInput defaultWeight;
    RigExecWirePropertyChainInput value;
    RigExecWirePropertyChainInput minimum;
    RigExecWirePropertyChainInput maximum;
    /// The curve operation's keys and optional tangents, folded at bake.
    std::vector<RigExecWireVec2f> keys;
    std::vector<RigExecWireVec2f> tangents;
};

/// One chain, in dependency order within the section.
struct RigExecWirePropertyChain {
    enum class ValueType : uint8_t { Float = 0, Double = 1 };
    uint32_t target = 0;
    ValueType valueType = ValueType::Float;
    /// Per baked frame: whether the target has an authored base, and it.
    std::vector<uint8_t> frameBaseHave;
    std::vector<double> frameBase;
    std::vector<RigExecWirePropertyChainRevision> revisions;
};

/// An input holder a chain's result feeds.
struct RigExecWirePropertyChainConsumer {
    uint32_t uid = 0;
    uint32_t chain = 0;
};

struct RigExecWirePropertyChains {
    std::vector<RigExecWirePropertyChain> chains;
    std::vector<RigExecWirePropertyChainConsumer> consumers;
};

bool RigExecWireEncodePropertyChains(const RigExecWirePropertyChains &chains,
                                     std::vector<uint8_t> *out);
bool RigExecWireDecodePropertyChains(RigExecWireReader *reader,
                                     RigExecWirePropertyChains *chains,
                                     std::string *error);

}  // namespace rigExec

#endif  // RIGEXEC_BINARY_PROPERTY_CHAINS_H
