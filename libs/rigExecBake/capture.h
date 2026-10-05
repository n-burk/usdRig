// .rigexec capture: the values the program consumed in a run.
// RigExecBakeCapture walks the standing program's bound varying inputs once,
// assigning uids in the traversal order the InputTable section documents,
// arms the bake recorder on the resolved inputs, and then captures one
// record per Evaluate: the recorded reads plus the prologue's retained
// arrays for that run. A bake captures one record, from a run at the bake
// time with every step forced (RigExecBakedProgram::RequestFullRun), so the
// record holds every static datum a step reads. The destructor disarms, so
// the evaluator a bake leaves behind reads exactly as it did before.
// A capture refuses to run with interactive overrides standing: a bake is
// of the authored epoch, and a held drag would print its values into the
// record. It also refuses when the program rebuilds mid-capture, because a
// rebuild reallocates the inputs the uid map keys off.
//
// The directory holds the inputs that vary, the ones a record can hold a
// value for. The runtime reads every input over the Computed section's
// slots (computedCapture.h), which a client sets, and reads only the
// record's static data, never its uids or recorded input values.
//
#ifndef RIGEXEC_BAKE_CAPTURE_H
#define RIGEXEC_BAKE_CAPTURE_H

#include "rigExecBinary/container.h"
#include "rigExecBinary/inputTable.h"

#include <pxr/base/vt/value.h>

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace rigExec {

class RigExecRigEvaluator;
class RigExecBakedProgram;
struct RigExecBakedProgramImpl;
struct RigExecRigPose;
struct RigExecBakeReadRecorder;

/// Encodes one stage read's value the way a frame record holds it: an empty
/// value is Absent, a token goes through \p intern for its string id. False,
/// with \p out untouched past its tag, for a type no record can carry.
bool RigExecBakeEncodePathValue(
    const PXR_NS::VtValue &held,
    const std::function<uint32_t(const std::string &)> &intern,
    RigExecWirePathValue *out);

/// Whether \p a and \p b hold the same value bit for bit: the same tag, and
/// for that tag the same payload, floats and doubles compared by their bytes
/// (so -0 differs from 0 and a NaN equals only its own bits), tokens by id.
bool RigExecBakeSamePathValue(const RigExecWirePathValue &a,
                              const RigExecWirePathValue &b);

class RigExecBakeCapture {
public:
    RigExecBakeCapture(const RigExecBakeCapture &) = delete;
    RigExecBakeCapture &operator=(const RigExecBakeCapture &) = delete;

    /// Builds the directory from \p evaluator's STANDING program -- the
    /// caller compiles first -- interning head paths into \p writer, and
    /// arms the recorder. Check Valid before capturing.
    RigExecBakeCapture(RigExecRigEvaluator &evaluator,
                       RigExecBinaryWriter *writer, std::string *error);
    ~RigExecBakeCapture();

    bool Valid() const { return _valid; }

    /// The table so far: the directory and override routing from the
    /// constructor, one frame record per CaptureFrame call.
    const RigExecWireInputTable &GetTable() const { return _table; }

    /// The uid the directory gave the program input at \p input (the
    /// address of a RigExecBakedInput<T> in the standing program), or -1
    /// when it has none.
    int64_t FindUid(const void *input) const
    {
        const auto found = _uids.find(input);
        return found == _uids.end() ? -1 : int64_t(found->second);
    }

    /// Evaluates \p frame and appends its record, or returns false with
    /// the reason: an invalid generation, a rebuild since the capture was
    /// armed, an unmapped record, or an unencodable value.
    bool CaptureFrame(double frame, RigExecRigPose *pose,
                      std::string *error);

private:
    bool _Drain(const RigExecBakedProgramImpl &program, double frame,
                std::string *error);

    RigExecRigEvaluator *_evaluator = nullptr;
    RigExecBinaryWriter *_writer = nullptr;
    const RigExecBakedProgram *_program = nullptr;
    std::unique_ptr<RigExecBakeReadRecorder> _recorder;
    std::map<const void *, uint32_t> _uids;
    RigExecWireInputTable _table;
    bool _valid = false;
};

}  // namespace rigExec

#endif  // RIGEXEC_BAKE_CAPTURE_H
