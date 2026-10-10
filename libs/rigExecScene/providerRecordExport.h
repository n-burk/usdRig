#ifndef RIGEXEC_GRAPH_PROVIDER_RECORD_EXPORT_H
#define RIGEXEC_GRAPH_PROVIDER_RECORD_EXPORT_H
#include "providerProgram.h"
namespace rigExec {
/// Translates native/DB provider kernels and current typed defaults for wire
/// export. Local xform evaluation is a source frontend operation, not a frozen
/// expression record; native exports its already bound interveningSpace leaf.
bool RigExecExportProviderRecords(const RigExecProviderProgram &program,
    const RigExecTypedValueStore &store,RigExecProviderPlainProgram *records,
    std::string *error=nullptr);
}
#endif
