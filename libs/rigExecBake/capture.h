//
// .rigexec capture: the per-frame values the program consumed.
//
// RigExecBakeCapture walks the standing program's bound varying inputs once,
// assigning uids in the traversal order the InputTable section documents,
// arms the bake recorder on the resolved inputs, and then captures one
// record per Evaluate: the recorded reads plus the prologue's retained
// arrays for that frame. The destructor disarms, so the evaluator a bake
// leaves behind reads exactly as it did before.
//
// A capture refuses to run with interactive overrides standing: a bake is
// of the authored epoch, and a held drag would print its values into every
// stream. It also refuses when the program rebuilds mid-loop, because a
// rebuild reallocates the inputs the uid map keys off.
//
#ifndef RIGEXEC_BAKE_CAPTURE_H
#define RIGEXEC_BAKE_CAPTURE_H

#include "rigExecBinary/container.h"
#include "rigExecBinary/inputTable.h"

#include <map>
#include <memory>
#include <string>

namespace rigExec {

class RigExecRigEvaluator;
class RigExecBakedProgram;
struct RigExecBakedProgramImpl;
struct RigExecRigPose;
struct RigExecBakeReadRecorder;

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

    /// Evaluates \p frame and appends its record, or returns false with
    /// the reason: an invalid generation, a mid-loop rebuild, an
    /// unmapped record, or an unencodable value.
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
