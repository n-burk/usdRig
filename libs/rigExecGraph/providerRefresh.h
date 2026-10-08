#ifndef RIGEXEC_GRAPH_PROVIDER_REFRESH_H
#define RIGEXEC_GRAPH_PROVIDER_REFRESH_H
#include <cstddef>
#include <cstdint>
namespace rigExec {
enum class RigExecProviderRefreshOutcome : uint8_t {
    Kept,Published,MissingBase,MissingCurrent,InvalidCurrent,SingularCurrent,InvalidDescendant,InvalidTransformedDescendant
};
/// The caller supplies retained, distinct output rows. Math uses the same
/// frame measurement/classification primitives as ordinary pose commits.
template<class Math>
RigExecProviderRefreshOutcome RigExecRefreshProviderFrames(
    const typename Math::Frame &beforeBase,const typename Math::Frame &beforeCurrent,
    const typename Math::Frame *raw,const typename Math::Frame *current,bool constrained,
    const typename Math::Frame *baseInputs,const typename Math::Frame *currentInputs,
    const uint8_t *blocked,size_t count,typename Math::Frame *baseOutput,
    typename Math::Frame *currentOutput,typename Math::Frame *baseOutputs,
    typename Math::Frame *currentOutputs,size_t *failed)
{
    *baseOutput=beforeBase;*currentOutput=beforeCurrent;
    for(size_t k=0;k<count;++k) {baseOutputs[k]=baseInputs[k];currentOutputs[k]=currentInputs[k];}
    if(constrained)return RigExecProviderRefreshOutcome::Kept;
    if(!raw)return RigExecProviderRefreshOutcome::MissingBase;
    if(!current)return RigExecProviderRefreshOutcome::MissingCurrent;
    *baseOutput=*raw;
    typename Math::Matrix baseDelta(1.0);
    if(Math::Usable(*raw) && Math::PointsToMatrix(beforeBase,*raw,&baseDelta))
        for(size_t k=0;k<count;++k)if(!blocked[k])baseOutputs[k]=Math::Transform(baseInputs[k],baseDelta);
    if(!Math::Usable(*current))return RigExecProviderRefreshOutcome::InvalidCurrent;
    bool carries=false;
    for(size_t k=0;k<count;++k)carries=carries || !blocked[k];
    if(!carries) {*currentOutput=*current;return RigExecProviderRefreshOutcome::Published;}
    typename Math::Matrix delta(1.0);
    if(!Math::PointsToMatrix(beforeCurrent,*current,&delta))return RigExecProviderRefreshOutcome::SingularCurrent;
    const auto fail=[&](size_t k,RigExecProviderRefreshOutcome reason) {
        *failed=k;
        for(size_t j=0;j<count;++j)currentOutputs[j]=currentInputs[j];
        return reason;
    };
    for(size_t k=0;k<count;++k)if(!blocked[k]) {
        if(!Math::Usable(currentInputs[k]))return fail(k,RigExecProviderRefreshOutcome::InvalidDescendant);
        currentOutputs[k]=Math::Transform(currentInputs[k],delta);
        if(!Math::Usable(currentOutputs[k]))return fail(k,RigExecProviderRefreshOutcome::InvalidTransformedDescendant);
    }
    *currentOutput=*current;
    return RigExecProviderRefreshOutcome::Published;
}
}
#endif
