// .rigexec Computed section producer (temporary, see
// rigExecBinary/computed.h): the input slots the weight oracle and the
// property chains read, their value per baked frame, the weight object
// table with its reads and oracle facts, each constraint's envelope index,
// the property chains with their phased consumers, and the program's
// registered reads whose walk crosses a chain target, each with the uid
// the frame records give it.
// The slots are the attributes on the walks of the weight objects' scalar
// reads -- the registered reads of every object a WeightPacket step bakes
// (Baked mode, as RigExecBakedRead resolves them) and the reads of every
// envelope-only object (Resolved mode, as the oracle's _ResolvedRead does)
// -- of the property movers' inputs (Pinned or Resolved, as _PinnedRead
// resolves them) and of the chain-crossing registered reads (Baked), plus
// every chain target (its raw value is the chain's base) and every phased
// consumer (where a phased value is published).
// Per frame each slot is read raw -- a typed Get at the frame's time, no
// walk, no overlay, no recorder -- because the runtime performs the walks.
// Internal to rigExecBake.
#ifndef RIGEXEC_BAKE_COMPUTED_CAPTURE_H
#define RIGEXEC_BAKE_COMPUTED_CAPTURE_H

#include "rigExecBinary/computed.h"
#include "rigExecBinary/container.h"

#include <memory>
#include <string>
#include <vector>

namespace rigExec {

class RigExecRigEvaluator;
class RigExecBakeCapture;

/// A weight object the runtime resolves whose oracle facts hold one time's
/// answer of an animated attribute: the facts are what the file holds, the
/// attribute is what the stage animates.
struct RigExecBakeTimeVaryingFact {
    std::string object;
    std::string attribute;
};

class RigExecBakeComputedCapture {
public:
    RigExecBakeComputedCapture(const RigExecBakeComputedCapture &) = delete;
    RigExecBakeComputedCapture &operator=(
        const RigExecBakeComputedCapture &) = delete;

    /// Builds everything but the frames from \p evaluator's STANDING
    /// program (the caller compiles first), interning names into
    /// \p writer. \p records is the frame-record capture of the same
    /// program, whose directory numbers the chain-crossing reads. Static
    /// data and slot defaults are read at \p time, the bake time. Never
    /// evaluates. Check Valid before recording.
    RigExecBakeComputedCapture(RigExecRigEvaluator &evaluator,
                               const RigExecBakeCapture &records,
                               double time, RigExecBinaryWriter *writer,
                               std::string *error);
    ~RigExecBakeComputedCapture();

    bool Valid() const { return _valid; }

    /// Appends every slot's value at \p frame. Call once per InputTable
    /// record, in the same order, after that frame's Evaluate.
    bool RecordFrame(double frame, std::string *error);

    const RigExecWireComputed &GetComputed() const;

    /// Every resolved weight object whose facts are of an animated
    /// attribute, in the order the composition pass met them (composing
    /// objects before the ones they compose).
    const std::vector<RigExecBakeTimeVaryingFact> &GetTimeVaryingFacts() const;

    /// The listed inputs' names, ascending: the names a client sets.
    const std::vector<std::string> &GetListedInputNames() const;

private:
    struct _State;
    std::unique_ptr<_State> _state;
    bool _valid = false;
};

}  // namespace rigExec

#endif  // RIGEXEC_BAKE_COMPUTED_CAPTURE_H
