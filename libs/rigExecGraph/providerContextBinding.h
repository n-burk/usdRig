#ifndef RIGEXEC_GRAPH_PROVIDER_CONTEXT_BINDING_H
#define RIGEXEC_GRAPH_PROVIDER_CONTEXT_BINDING_H
#include "providerProgram.h"
#include <functional>
namespace rigExec {
/// Binds one expression/rest closure to explicitly selected typed frame versions.
/// The dense value namespace may include other domains. Unaffected subtrees
/// retain their original producers; the result has a stable consumer identity.
/// Cycles are preserved for the single final graph compiler.
bool RigExecBindProviderContext(RigExecProviderProgram *,
    const std::vector<std::string> &fullValueKeys,RigExecValueId sourceResult,
    const SdfPath &consumer,const std::string &phase,
    const std::map<RigExecValueId,RigExecValueId> &selectedFrames,
    RigExecValueId *result,size_t *beginOp,std::string *error=nullptr,
    const std::map<std::string,std::string> &externalNames={});
/// A native ladder selects an unconnected parent matrix only when authored.
/// Its namespace fallback is supplied by the compose's exact parent version.
/// Keep the full expression identity available to other consumers.
bool RigExecBindProviderParentDelivery(RigExecProviderProgram *,const SdfPath &consumer,
    RigExecValueId expression,RigExecValueId *output,std::string *error=nullptr);
/// Compile-local overload: firstProducers must index the FIRST op for every
/// current output. Existing output identities stay immutable; append suffixes
/// must be indexed before each call. This trusted index is not a general cache.
/// The legacy overload retains its original linear-search behavior.
bool RigExecBindProviderParentDelivery(RigExecProviderProgram *,const SdfPath &consumer,
    RigExecValueId expression,RigExecValueId *output,std::string *error,
    const std::map<RigExecValueId,size_t> *firstProducers);
/// Rebinds the original provider math operands after all typed versions exist.
/// Matrix consumers without explicit connections return their original ID.
/// Scalar consumers resolve the exact normal typed input traversal. Callbacks
/// may append contextual ops; descriptors retain every returned producer read.
using RigExecProviderInputBinding=std::function<bool(const SdfPath &,RigExecValueId,
    RigExecValueId *,std::string *)>;
bool RigExecRebindProviderInputs(RigExecProviderProgram *,size_t initialOpCount,
    const RigExecProviderInputBinding &,std::string *error=nullptr);

}
#endif
