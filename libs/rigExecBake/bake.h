//
// .rigexec baking: a compiled rigExec character as a USD-independent binary.
//
// BakeToBinary drives a RigExecRigEvaluator through the baked program at the
// requested frames and serializes the epoch -- manifest first (this slice),
// then the slot tables, steps and schedule (later M1 slices) -- into the
// sectioned container libs/rigExecBinary owns.
//
// A bake is of the PROGRAM, never of a fallback: every requested frame must
// be answered by the baked program, and an epoch the program cannot express
// fails naming the feature (RigExecBakedProgram::IsBakeable reasons), the
// same contract --require-baked gives rigExecPose. There is no quiet dynamic
// bake, because a binary whose numbers came from two different paths is a
// binary no parity check can hold to account.
//
#ifndef RIGEXEC_BAKE_H
#define RIGEXEC_BAKE_H

#include <cstdint>
#include <string>
#include <vector>

namespace rigExec {

class RigExecRigEvaluator;

/// What to bake.
struct RigExecBakeOpts {
    /// Frames to bake, in the order given. Required and non-empty; the
    /// runtime evaluates exactly these frames and nothing else.
    std::vector<double> frames;
    /// The reader major version the binary must load under. The writer only
    /// knows its own (see RigExecBinaryVersion), so anything else fails.
    uint32_t targetReaderVersion = 1;
};

/// What a bake produced.
struct RigExecBakeResult {
    /// The .rigexec file bytes, ready to write.
    std::vector<uint8_t> bytes;
    /// The manifest section as text, for logs and goldens.
    std::string manifestJson;
};

/// Bakes \p evaluator's epoch to a .rigexec binary, or returns false with
/// the reason: no frames, a compile failure, an epoch the program cannot
/// express (with the IsBakeable reasons), an invalid generation, or a
/// generation that did not come from the program.
///
/// The caller sets the evaluation mode BEFORE calling -- Baked, the way
/// rigExecPose honors --mode -- and this compiles, checks bakeability,
/// evaluates every frame, and serializes. The evaluator is left compiled in
/// the caller's mode.
bool RigExecBakeToBinary(RigExecRigEvaluator &evaluator,
                         const RigExecBakeOpts &opts,
                         RigExecBakeResult *result, std::string *error);

}  // namespace rigExec

#endif  // RIGEXEC_BAKE_H
