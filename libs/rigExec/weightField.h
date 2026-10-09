#ifndef RIGEXEC_WEIGHT_FIELD_H
#define RIGEXEC_WEIGHT_FIELD_H
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>
namespace rigExec {
class RigExecRigEvaluator;
struct RigExecBakedProgramImpl;
struct RigExecBakedStep;
struct RigExecWeightFieldInputs;
struct RigExecWeightPointView;
bool RigExecBakedBuildWeightFields(const RigExecRigEvaluator &, RigExecBakedProgramImpl *, std::string *error);
void RigExecBakedRunWeightField(RigExecBakedProgramImpl *, RigExecBakedStep *);
void RigExecBakedDeclareWeightField(RigExecBakedProgramImpl *, RigExecBakedStep *);
/// Fresh source-free numerical input packet, shared by body and exact key capture.
/// All views remain immutable and live with B through the consuming call.
bool RigExecBakedCaptureWeightFieldInputs(const RigExecBakedProgramImpl &,int field,
    std::vector<RigExecWeightFieldInputs> *,RigExecWeightPointView *entering,size_t *count);
void RigExecBakedWeightFieldCoveredVersions(const RigExecBakedProgramImpl &,int field,
    std::vector<uint32_t> *versions);
/// Materialize only the canonical compiler's SCC exclusion outcome.
void RigExecBakedBindWeightCycleState(RigExecBakedProgramImpl *);
}
#endif
