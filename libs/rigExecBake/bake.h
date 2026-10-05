// .rigexec baking: a compiled rigExec character as a USD-independent binary.
// BakeToBinary runs the evaluator's baked program once, in full, at one time
// T and serializes the epoch: the static data that run read, the input slots
// with their values at T as the defaults, the steps and schedule, into the
// sectioned container libs/rigExecBinary owns. A binary holds no animation;
// a client drives it by setting inputs.
// A bake is of the PROGRAM, never of a fallback: the run must be answered by
// the baked program, and an epoch the program cannot express fails naming
// the feature (RigExecBakedProgram::IsBakeable reasons), the same contract
// --require-baked gives rigExecPose. There is no quiet dynamic bake, because
// a binary whose numbers came from two different paths is a binary no
// parity check can hold to account.
#ifndef RIGEXEC_BAKE_H
#define RIGEXEC_BAKE_H

#include "rigExecBinary/container.h"

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
    /// The .rigexec file bytes, ready to write.
    std::vector<uint8_t> bytes;
    /// The bake's path reads: the entries its run recorded; the entries its
    /// record holds, those plus every enumerated key the run's branches did
    /// not reach; and the distinct keys of the assembly's enumeration. Every
    /// recorded entry is an enumerated key with the same value, except the
    /// ones a plugin mover's assembly records of its own (unenumerated).
    size_t pathReadsRecorded = 0;
    size_t pathReadsWritten = 0;
    size_t pathReadsEnumerated = 0;
    size_t pathReadsUnenumerated = 0;
};

/// Bakes \p evaluator's epoch to a .rigexec binary, or returns false with
/// the reason: a non-finite time, a compile failure, an epoch the program
/// cannot express (with the IsBakeable reasons), interactive overrides
/// standing, an invalid generation, a run that did not come from the
/// program or rebuilt it, a presentation that does not verify or names an
/// input the file does not list, or a path read the run recorded that the
/// assembly's enumeration (RigExecBakeEnumerateProgramReads) does not list
/// with the same value, which names the read (a key no core assembly reads
/// is accepted when the program holds plugin movers).
///
/// The caller sets the evaluation mode BEFORE calling -- Baked, the way
/// rigExecPose honors --mode -- and this compiles, checks bakeability,
/// evaluates once at the bake time with every step forced to run, and
/// serializes. The evaluator is left compiled in the caller's mode.
bool RigExecBakeToBinary(RigExecRigEvaluator &evaluator,
                         const RigExecBakeOpts &opts,
                         RigExecBakeResult *result, std::string *error);

}  // namespace rigExec

#endif  // RIGEXEC_BAKE_H
