#ifndef RIGEXEC_RUNTIME_SPACES_H
#define RIGEXEC_RUNTIME_SPACES_H
#include "store.h"
namespace rigExec {
bool RrReadCrossDomain(const RrProgram *, int32_t, RrWireValue *);
const std::vector<RrVec3f> *RrResolveDeclaredPoints(const RrProgram *, const RigExecWirePointsBinding &);
const std::vector<RrVec3f> *RrReadCrossDomainPoints(const RrProgram *, const fb::RigExecWireCrossDomainRead &);
bool RrReadCrossDomainPoint(const RrProgram *, const fb::RigExecWireCrossDomainRead &, RrVec3f *);
bool RrOpenProviderProgram(RrProgram *, std::string *);
bool RrProviderEffectiveInputMemo(const RrProgram *, const RigExecWireStep &, std::string *, std::vector<uint32_t> *);
RrWireValue RrReadProviderRefreshBlocker(const RrProgram *, int);
bool RrRunProviderRefresh(RrProgram *, const RigExecWireStep &, std::string *);
bool RrRunSpaceExpression(RrProgram *, const RigExecWireStep &, std::string *);
}
#endif
