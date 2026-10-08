#ifndef RIGEXEC_BAKED_OP_GRAPH_H

#define RIGEXEC_BAKED_OP_GRAPH_H

#include "bakedProgramImpl.h"

namespace rigExec {

struct RigExecBakedOpIdentityRemap;
bool RigExecBakedEffectiveMemo(const RigExecBakedProgramImpl &,uint32_t,
    std::string *,std::vector<uint32_t> *,std::vector<std::pair<uint32_t,uint32_t>> *,
    const RigExecBakedOpIdentityRemap *remap=nullptr);
void RigExecBakedAdoptSkinOpState(RigExecBakedProgramImpl *,const RigExecBakedProgramImpl &);
bool RigExecBakedCompileOpGraph(RigExecBakedProgramImpl *, std::string *);

bool RigExecBakedLowerOpGraph(RigExecBakedProgramImpl *, std::string *);

bool RigExecBakedExecuteOpGraph(RigExecBakedProgramImpl *, UsdTimeCode, bool);

}

#endif
