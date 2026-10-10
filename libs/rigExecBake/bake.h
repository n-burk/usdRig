// .rigexec baking: a compiled rigExec character as a USD-independent binary.
// BakeToBinary runs the evaluator's baked program once, in full, at one time
// T and serializes the epoch: the static data that run read, the input slots
// with their values at T as the defaults, the steps and schedule, into one
// FlatBuffer (rigExecBinary/format.h). A binary holds no animation; a client
// drives it by setting inputs.
// A bake is of the PROGRAM, never of a fallback: the run must be answered by
// the baked program, and an epoch the program cannot express fails naming
// the feature (RigExecBakedProgram::IsBakeable reasons), the same contract
// --require-baked gives rigExecPose. There is no quiet dynamic bake, because
// a binary whose numbers came from two different paths is a binary no
// parity check can hold to account.
#ifndef RIGEXEC_BAKE_H
#define RIGEXEC_BAKE_H

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace rigExec {

class RigExecRigEvaluator;

/// What to bake.
struct RigExecBakeOpts {
    /// The time the static data and every input default are read at. NaN
    /// (the default) is the program's probe time: the stage's start time
    /// code when it authors a time-code range, else 0. Must be finite.
    double time = std::numeric_limits<double>::quiet_NaN();
    /// A REXP presentation buffer to embed, or empty. It must verify, and
    /// every control's input must name a listed input of the file.
    std::vector<uint8_t> presentation;
};

/// What a bake produced.
struct RigExecBakeResult {
    /// The .rigexec file bytes, ready to write: the FlatBuffer file,
    /// written through RigExecFormatWrite and opened again with
    /// RigExecFormatOpen before the bake returns.
    std::vector<uint8_t> bytes;
    /// Sorted names of admitted upstream inputs standing on entry. Their
    /// file defaults remain stage-authored; callers retain the values.
    std::vector<std::string> upstreamInputs;
    /// The file's path-read rows (a read row per connection-following
    /// scalar read, a value row per other key), and the distinct (path,
    /// rest) keys of the assembly's enumeration they come from
    /// (RigExecBakeEnumerateProgramReads).
    size_t pathReadsWritten = 0;
    size_t pathReadsEnumerated = 0;
};

/// Bakes \p evaluator's epoch to a .rigexec binary, or returns false with
/// the reason: a non-finite time, a compile failure, an epoch the program
/// cannot express (with the IsBakeable reasons), interactive overrides
/// standing, an invalid generation, a run that did not come from the
/// program or rebuilt it, a plugin mover that cannot encode its payload, a
/// presentation that does not verify or names an input the file does not
/// list, or a program the FlatBuffer file cannot hold or whose file does
/// not open again.
///
/// Compiles, checks export admission, builds the export program
/// (RigExecBakedRoleMode::Export, keeping every admitted upstream input and
/// every presentation input listed) with one generation at the bake time,
/// evaluates once more there with every operation forced to run, and
/// serializes with upstream inputs suspended. The reads the export
/// program's Range skins and group gates rest on are private slots holding
/// their bake-time values. The complete requested input list and the
/// evaluator's role mode are restored on every return; its next Evaluate
/// rebuilds the program.
bool RigExecBakeToBinary(RigExecRigEvaluator &evaluator,
                         const RigExecBakeOpts &opts,
                         RigExecBakeResult *result, std::string *error);

}  // namespace rigExec

#endif  // RIGEXEC_BAKE_H
