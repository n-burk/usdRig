#include "poseCommit.h"
#include "constraintProgram.h"
namespace rigExec {
bool RigExecRunPoseCommit(const RigExecPoseCommitBinding &binding,RigExecTypedValueStore *store,
    RigExecPoseCommitWorkspace *workspace,std::string *error) {
    if(!store || !workspace)return false;
    const auto count=binding.candidates.size();workspace->deltas.resize(count);workspace->deltaOk.resize(count);
    workspace->staged.resize(binding.descendants.size());std::string failure;
    for(size_t i=0;i<count;++i) {
        const auto &candidate=binding.candidates[i];const auto *before=store->Read<RigExecPointFrame>(candidate.before),*after=store->Read<RigExecPointFrame>(candidate.candidate);
        workspace->deltaOk[i]=before && after && RigExecPreparePoseDelta(*before,*after,&workspace->deltas[i]);
        if(!after || !RigExecConstraintFrameUsable(*after))failure="pose commit candidate unavailable";
    }
    for(size_t i=0;i<binding.descendants.size() && failure.empty();++i) {
        const auto &descendant=binding.descendants[i];
        if(descendant.candidate>=count)return false;
        const auto &candidate=binding.candidates[descendant.candidate];
        const auto *current=store->Read<RigExecPointFrame>(descendant.current),*before=store->Read<RigExecPointFrame>(candidate.before),*after=store->Read<RigExecPointFrame>(candidate.candidate);
        RigExecPointFrame empty;empty.flags=0;
        const auto status=RigExecPropagatePoseFrame(current?*current:empty,before?*before:empty,after?*after:empty,
            workspace->deltas[descendant.candidate],workspace->deltaOk[descendant.candidate],descendant.solverOutput,
            descendant.parentBlocked,after!=nullptr,&workspace->staged[i]);
        if(status==RigExecPosePropagateOutcome::Skipped)workspace->staged[i]=current?*current:empty;
        else if(status!=RigExecPosePropagateOutcome::Staged)failure="pose commit descendant propagation failed";
    }
    if(!failure.empty()) {
        auto passthrough=[&](RigExecValueId output,RigExecValueId input) {
            const auto &source=store->values.at(size_t(input));
            std::visit([&](const auto &value){store->Publish(output,value,source.blocked,false,source.count,failure);},source.value);
            store->values[size_t(output)].raw=source.raw;
        };
        for(const auto &candidate:binding.candidates)passthrough(candidate.output,candidate.before);
        for(const auto &descendant:binding.descendants)passthrough(descendant.output,descendant.current);
        if(error)*error=failure;return true;
    }
    for(const auto &candidate:binding.candidates)store->Copy(candidate.output,candidate.candidate,false);
    for(size_t i=0;i<binding.descendants.size();++i)store->Publish(binding.descendants[i].output,workspace->staged[i]);
    return true;
}
}
