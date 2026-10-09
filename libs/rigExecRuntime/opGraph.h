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
/// The key the executor publishes value (\p domain, \p slot) by.
void RrOpValueKey(const RrProgram *, uint32_t domain, uint32_t slot, std::string *);
/// Publishes a fixed-size value -- pose frames, matrices, staged frames,
/// unboxed space values -- by building RrOpValueKey's bytes on the stack
/// and comparing them in place with the stored key: the flag, revision and
/// key the string publication leaves. False, with \p value untouched, for
/// any other value; the caller publishes it through its key.
bool RrPublishSmallValue(const RrProgram *, RigExecOpValueState *value);
bool RrExecuteOpGraph(RrProgram *, bool, std::string *);
}
#endif
