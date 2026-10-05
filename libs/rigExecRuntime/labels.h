// rigExecRuntime step labels: the text error messages name a step by.
// USD-free.
#ifndef RIGEXEC_RUNTIME_LABELS_H
#define RIGEXEC_RUNTIME_LABELS_H

#include <cstddef>
#include <string>

namespace rigExec {

struct RrProgram;

/// Step \p step's label, as the baked program spells it
/// (RigExecBakedStepKindName and StepLabel, bakedSchedule.cpp): the kind,
/// a space, then the path of the object the step works on. Built from the
/// tables on demand, for error text; a step past the steps is its number,
/// and an object past its table is the kind and the object's number.
std::string RrStepLabel(const RrProgram &program, size_t step);

}  // namespace rigExec

#endif  // RIGEXEC_RUNTIME_LABELS_H
