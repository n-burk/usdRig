#ifndef RIGEXEC_GOLDEN_SUITE_H
#define RIGEXEC_GOLDEN_SUITE_H

#include "observerHost.h"
#include "pxr/usd/sdf/path.h"
#include <memory>
#include <string>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {
struct RigExecRigPose;
struct RigExecGoldenSuiteNode;

/// Optional owning-thread suite capture. Each evaluator owns its file and
/// buffer; the disabled path allocates nothing and records nothing.
/// A process-completion index checks total evaluator/generation counts and
/// capture inventory; each evaluator's file checks its own exact visit order.
class RigExecGoldenSuiteObserver {
public:
    static std::unique_ptr<RigExecGoldenSuiteObserver> Create(const SdfPath &rigPath);
    ~RigExecGoldenSuiteObserver();
    void Record(const RigExecRigPose &pose);
private:
    RigExecGoldenSuiteObserver(const SdfPath &rigPath, unsigned ordinal);
    RigExecGoldenSuiteNode *_node = nullptr;
    unsigned _ordinal = 0;
    size_t _generation = 0;
    std::string _bytes;
};
}
#endif
