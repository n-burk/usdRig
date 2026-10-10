#include "scalarReferenceAdapter.h"
#include "oracleDispatch.h"
#include "bakedProgramImpl.h"
#include <algorithm>
#include <cstdio>
#include <cstdlib>

namespace rigExec {
void RigExecOraclePublicationContext::Begin(uint64_t generation,RigExecOracleScene source,
    size_t chainCount,const std::set<SdfPath> &producedPaths,const std::set<SdfPath> &protectedPaths) {
    source.SealCapturedFacts();
    _generation=generation;_source=std::move(source);_protected=protectedPaths;
    _chains.clear();_chains.resize(chainCount);
    for(const auto &path:producedPaths)if(!_protected.count(path))_source.overlay.erase(path);
    _source.resolveFromFacts=true;
}
void RigExecOraclePublicationContext::Restart(uint64_t generation) {
    _generation=generation;
    for(auto &chain:_chains) { chain.finished=0;chain.completed=false;chain.writes.clear(); }
}
bool RigExecOraclePublicationContext::Finish(size_t chain,const Writes &writes) {
    if(chain>=_chains.size())return false;
    _chains[chain].writes=writes;_chains[chain].finished=_generation;_chains[chain].completed=true;
    return true;
}
RigExecOracleScene RigExecOraclePublicationContext::ReaderScene(const std::vector<int> &availableChains) const {
    auto reader=_source;
    for(size_t chain=0;chain<_chains.size();++chain) {
        const auto &publication=_chains[chain];
        if(!publication.completed || publication.finished!=_generation ||
            std::find(availableChains.begin(),availableChains.end(),int(chain))==availableChains.end())continue;
        for(const auto &write:publication.writes)if(!_protected.count(write.first))reader.overlay[write.first]=write.second;
    }
    return reader;
}

void RigExecBakedPrepareOracleReference(RigExecBakedProgramImpl *program,UsdTimeCode time) {
    auto &B=*program;
    if(!B.oraclePublications)return;
    const auto generation=B.oraclePublications->Generation()+1;
    if(B.stage)RigExecBakedBeginOracleReference(program,generation,time);
    else B.oraclePublications->Restart(generation);
}

void RigExecBakedAppendCycleDiagnostics(const RigExecBakedProgramImpl &program,RigExecRigPose *pose) {
    for(const auto &loop:program.opGraph.cycles) {
        std::string message="operation cycle: ";
        for(size_t i=0;i<loop.size();++i) { if(i)message+=" -> ";message+=loop[i]; }
        if(std::find(pose->diagnostics.begin(),pose->diagnostics.end(),message)==pose->diagnostics.end())
            pose->diagnostics.push_back(std::move(message));
    }
}

void RigExecBakedBeginOracleReference(RigExecBakedProgramImpl *program, uint64_t generation, UsdTimeCode time)
{
    if (!program || !program->oraclePublications) return;
    const auto begin = RigExecOracleInstalledHooks().begin;
    if (!begin) RigExecOracleMissing("cpu reference");
    begin(program, generation, time);
}

bool RigExecBakedRunScalarReference(RigExecBakedProgramImpl *program, UsdTimeCode time, RigExecRigPose *pose)
{
    if (!program || !program->oraclePublications) return true;
    const auto run = RigExecOracleInstalledHooks().run;
    if (!run) {
        RigExecOracleMissing("cpu reference");
        return false;
    }
    return run(program, time, pose);
}
} // namespace rigExec
