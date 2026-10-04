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

namespace rigExec {

class RigExecRigEvaluator;
class RigExecBakeCapture;

class RigExecBakeComputedCapture {
public:
    RigExecBakeComputedCapture(const RigExecBakeComputedCapture &) = delete;
    RigExecBakeComputedCapture &operator=(
        const RigExecBakeComputedCapture &) = delete;

    /// Builds everything but the frames from \p evaluator's STANDING
    /// program (the caller compiles first), interning names into
    /// \p writer. \p records is the frame-record capture of the same
    /// program, whose directory numbers the chain-crossing reads. Static
    /// data and slot defaults are read at the program's probe time. Check
    /// Valid before recording.
    RigExecBakeComputedCapture(RigExecRigEvaluator &evaluator,
                               const RigExecBakeCapture &records,
                               RigExecBinaryWriter *writer,
                               std::string *error);
    ~RigExecBakeComputedCapture();

    bool Valid() const { return _valid; }

    /// Appends every slot's value at \p frame. Call once per InputTable
    /// record, in the same order, after that frame's Evaluate.
    bool RecordFrame(double frame, std::string *error);

    const RigExecWireComputed &GetComputed() const;

private:
    struct _State;
    std::unique_ptr<_State> _state;
    bool _valid = false;
};

}  // namespace rigExec

#endif  // RIGEXEC_BAKE_COMPUTED_CAPTURE_H
