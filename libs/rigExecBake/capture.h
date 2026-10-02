// .rigexec capture: the per-frame values the program consumed.
// RigExecBakeCapture walks the standing program's bound varying inputs once,
// assigning uids in the traversal order the InputTable section documents,
// arms the bake recorder on the resolved inputs, and then captures one
// record per Evaluate: the recorded reads plus the prologue's retained
// arrays for that frame. The destructor disarms, so the evaluator a bake
// leaves behind reads exactly as it did before.
// A capture refuses to run with interactive overrides standing: a bake is
// of the authored epoch, and a held drag would print its values into every
// stream. It also refuses when the program rebuilds mid-loop, because a
// rebuild reallocates the inputs the uid map keys off.
//
// OVERRIDABLE INPUTS are off by default and are what a client needs to POSE
// a baked rig rather than replay it. The default directory holds only the
// inputs that vary, because those are the only ones the runtime must read
// per frame -- a plainly authored `hips_ctl.avars:tx` folds into a constant
// and gets no uid, so nothing outside can address it. Every such avar is
// already registered as overridable (RigExecBakedProgramImpl::Register
// hands an overrideIndex to any input with a head, varying or not), which
// is how the live service drags a baked rig; the bake simply never wrote
// them down. With this on they get uids too, and their authored value is
// seeded into the FIRST frame record so an unposed playback is unchanged.
//
// It costs directory entries and one seeding record, and nothing per frame:
// a constant input still records nothing, because the runtime holds the
// last value it saw.
//
#ifndef RIGEXEC_BAKE_CAPTURE_H
#define RIGEXEC_BAKE_CAPTURE_H

#include "rigExecBinary/container.h"
#include "rigExecBinary/inputTable.h"

#include <pxr/base/vt/value.h>

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

class RigExecBakeCapture {
public:
    RigExecBakeCapture(const RigExecBakeCapture &) = delete;
    RigExecBakeCapture &operator=(const RigExecBakeCapture &) = delete;

    /// Builds the directory from \p evaluator's STANDING program -- the
    /// caller compiles first -- interning head paths into \p writer, and
    /// arms the recorder. Check Valid before capturing.
    /// \p overridableInputs widens the directory to every input an
    /// override can be placed on, not only the ones that vary. See the
    /// header comment.
    RigExecBakeCapture(RigExecRigEvaluator &evaluator,
                       RigExecBinaryWriter *writer, std::string *error,
                       bool overridableInputs = false);
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
    /// (uid, authored value) for the constant inputs the override option
    /// added, written into the first frame record so the runtime's holder
    /// starts where the rig does.
    std::vector<std::pair<uint32_t, PXR_NS::VtValue>> _seeds;
    bool _overridableInputs = false;
    bool _seeded = false;
    bool _valid = false;
};

}  // namespace rigExec

#endif  // RIGEXEC_BAKE_CAPTURE_H
