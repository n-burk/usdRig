#ifndef RIGEXEC_RUNTIME_OP_GRAPH_H
#define RIGEXEC_RUNTIME_OP_GRAPH_H
#include "store.h"
namespace rigExec {
bool RrCompileOpGraph(RrProgram *, std::string *);
void RrGeometryOpValueKey(const RrProgram *, RigExecWireSlotDomain, uint32_t, std::string *);
/// The key a point-carrying value (RevisionDone, ChainDirty, ChainPoints,
/// ChainBase, ChainInput, DerivedOut) had over its points' bytes before it
/// was keyed by content version; false for any other domain.
bool RrGeometryChainContentKey(const RrProgram *, RigExecWireSlotDomain, uint32_t, std::string *);
void RrResetExcludedGeometryValue(RrProgram *, RigExecWireSlotDomain, uint32_t);
bool RrExecuteOpGraph(RrProgram *, bool, std::string *);
}
#endif
