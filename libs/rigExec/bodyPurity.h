// Step-body purity: a baked or frozen step body reads its declared slots and
// its sampled leaves, never the stage and never the resolved-input overlay.
//
// RunStepBody (bakedSchedule.cpp), shared by native and frozen execution,
// mark the running thread for the length of the body. The read funnels --
// RigExecBakedRead, RigExecResolvedInputs::GetAttribute,
// RigExecAssembleParameters, RigExecResolveSkinTopology and
// RigExecRigEvaluator::_FrameFromXformRelativeToAsset -- start with
// RIGEXEC_PURITY_CHECK(), which reports a read made under the mark.
//
// Production bodies consume copied leaves and typed operation values.
// Source adapters sample before dispatch; declared API4 assembly is stage-free.

#ifndef RIGEXEC_BODY_PURITY_H
#define RIGEXEC_BODY_PURITY_H

#include <atomic>
#include <cstdint>

namespace rigExec {

/// Whether the calling thread is inside a step body and outside any
/// RigExecVolatileRead. One thread-local load.
bool RigExecInOpBody();

/// Reports one read made inside a step body: a TF_VERIFY in debug builds,
/// and one relaxed increment of the body's program counter when that
/// program was built with RIGEXEC_PURITY_AUDIT.
void RigExecReportBodyRead();

/// Marks the calling thread as running one step body for the scope's
/// lifetime. The previous mark is saved and restored rather than cleared,
/// because a body waiting on a parallel loop can run another body on the
/// same thread. \p violations is the program's audit counter, or null when
/// the program does not audit.
class RigExecOpBodyScope
{
public:
    explicit RigExecOpBodyScope(std::atomic<uint64_t> *violations);
    ~RigExecOpBodyScope();
    RigExecOpBodyScope(const RigExecOpBodyScope &) = delete;
    RigExecOpBodyScope &operator=(const RigExecOpBodyScope &) = delete;

private:
    bool _wasInBody;
    std::atomic<uint64_t> *_wasViolations;
};

/// Lifts the body mark for one of the volatile reads listed above.
class RigExecVolatileRead
{
public:
    RigExecVolatileRead();
    ~RigExecVolatileRead();
    RigExecVolatileRead(const RigExecVolatileRead &) = delete;
    RigExecVolatileRead &operator=(const RigExecVolatileRead &) = delete;

private:
    bool _wasInBody;
};

}  // namespace rigExec

#define RIGEXEC_PURITY_CHECK()                                             \
    do {                                                                   \
        if (::rigExec::RigExecInOpBody()) {                                \
            ::rigExec::RigExecReportBodyRead();                            \
        }                                                                  \
    } while (0)

#endif  // RIGEXEC_BODY_PURITY_H
