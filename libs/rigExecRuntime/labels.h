// rigExecRuntime step labels: the text error messages name a step by.
// USD-free.
#ifndef RIGEXEC_RUNTIME_LABELS_H
#define RIGEXEC_RUNTIME_LABELS_H

#include <cstddef>
#include <string>

namespace rigExec {

struct RrProgram;

/// Step \p step's label, as the baked program spells it: the opened file's
/// RigExecFormatStepLabel, built on demand for error text. A program
/// assembled without a file labels a step by its number.
std::string RrStepLabel(const RrProgram &program, size_t step);

}  // namespace rigExec

#endif  // RIGEXEC_RUNTIME_LABELS_H
