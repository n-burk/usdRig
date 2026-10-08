// rigExecRuntime step labels: the file's own (RigExecFormatStepLabel), so
// the runtime and the validator name a step in the same words.
#include "rigExecRuntime/labels.h"
#include "rigExecRuntime/store.h"

#include "rigExecBinary/format.h"

namespace rigExec {

std::string
RrStepLabel(const RrProgram &program, size_t step)
{
    // A program a test assembles by hand has no file to label from.
    return program.file ? RigExecFormatStepLabel(*program.file, step)
                        : std::to_string(step);
}

}  // namespace rigExec
