#ifndef RIGEXEC_RUNTIME_OP_GRAPH_H
#define RIGEXEC_RUNTIME_OP_GRAPH_H
#include "store.h"
namespace rigExec {
bool RrCompileOpGraph(RrProgram *, std::string *);
void RrGeometryOpValueKey(const RrProgram *, RigExecWireSlotDomain, uint32_t, std::string *);
void RrResetExcludedGeometryValue(RrProgram *, RigExecWireSlotDomain, uint32_t);
bool RrExecuteOpGraph(RrProgram *, bool, std::string *);
}
#endif
