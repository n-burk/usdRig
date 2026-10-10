#include "poseGraphBinding.h"
#include "rigExec/frameExtraction.h"
#include <algorithm>
namespace rigExec {
namespace {
bool Fail(std::string *error,const std::string &message){if(error)*error=message;return false;}
void TypedInputs(const RigExecGraphTypedRead &input,std::vector<RigExecValueId> *reads) {
    for(const auto *hops:{&input.hops,&input.doubleHops})for(const auto &hop:*hops)
        for(auto value:{hop.raw,hop.overlay})if(value!=UINT64_MAX)reads->push_back(value);
}
void Unique(std::vector<RigExecValueId> *reads) {
    std::sort(reads->begin(),reads->end());reads->erase(std::unique(reads->begin(),reads->end()),reads->end());
}
bool Matrix(const RigExecTypedValueStore &values,RigExecValueId id,GfMatrix4d *out) {
    if(id==UINT64_MAX) {*out=GfMatrix4d(1.0);return true;}
    VtValue value;if(!RigExecReadSceneGraphValue(values,id,&value,true) || !value.IsHolding<GfMatrix4d>())return false;
    *out=value.UncheckedGet<GfMatrix4d>();return true;
}
}
bool RigExecBindSceneSpaceSwitch(const RigExecSceneDescriptors &scene,
    const RigExecSceneSpaceSwitchDescriptor &source,const RigExecSceneGraphBindingContext &context,
    RigExecBoundSpaceSwitch *out,std::string *error,RigExecValueId parentContext,RigExecValueId spaceContext) {
    if(!out || !context.resolve)return Fail(error,"missing space switch graph linker");
    RigExecBoundSpaceSwitch packet;packet.record=source.record;
    packet.labels=source.labels;packet.tokenIndex=source.tokenIndex;packet.activeFallback=source.activeFallback;
    packet.unavailable="space switch "+source.path.GetString()+" has an unavailable current input";
    if(!RigExecBindGraphTypedRead(source.active,context,&packet.active,error))return false;
    TypedInputs(packet.active,&packet.reads);
    const auto named=[&](const SdfPath &provider,const std::string &computation,RigExecValueId *value) {
        if(provider.IsEmpty())return true;
        RigExecSceneGraphReadRequest request;request.source=provider;request.reader=source.path;
        request.consumer=source.target;request.domain=RigExecSceneValueDomain::ProviderSpace;
        request.phase=TfToken("base");request.computation=computation;
        if(!context.resolve(request,value,error))return false;packet.reads.push_back(*value);return true;
    };
    const auto attribute=[&](const char *name,RigExecValueId *value) {
        RigExecSceneGraphReadRequest request;request.source=source.target.AppendProperty(TfToken(name));
        request.consumer=request.source;request.reader=source.path;request.phase=TfToken("base");
        request.domain=RigExecSceneValueDomain::ProviderSpace;
        if(!context.resolve(request,value,error))return false;packet.reads.push_back(*value);return true;
    };
    SdfPath parent;
    for(auto path=source.target.GetParentPath();!path.IsEmpty() && !path.IsAbsoluteRootPath();path=path.GetParentPath()) {
        const auto node=scene.nodes.find(path);if(node==scene.nodes.end())continue;
        const auto type=node->second.fact.type;
        if(node->second.fact.active && (type=="RigExecControl" || type=="RigExecJoint" ||
           type=="RigExecSphereWeight" || type=="RigExecPlaneWeight" || type=="RigExecCurveWeight")) {parent=path;break;}
    }
    if(!named(source.target,"computeAvarMatrix",&packet.avars) ||
       !attribute("posed:defaultSpace",&packet.posedDefault) ||
       !attribute("parent:defaultSpace",&packet.parentExpression) ||
       !(parentContext!=UINT64_MAX?(packet.parentPosed=parentContext,
            packet.reads.push_back(parentContext),true):
            named(parent,"computePointFrame",&packet.parentPosed)) ||
       !named(parent,"computedDefaultSpace",&packet.parentDefault) ||
       !(spaceContext!=UINT64_MAX?(packet.spacePosed=spaceContext,
            packet.reads.push_back(spaceContext),true):
            named(source.space,"computePointFrame",&packet.spacePosed)) ||
       !named(source.space,"computedDefaultSpace",&packet.spaceDefault))return false;
    packet.inputs.hasCarry=!source.space.IsEmpty();
    packet.inputs.sources.resize(source.sources.size());
    packet.sourceDefault.assign(source.sources.size(),UINT64_MAX);packet.sourcePosed=packet.sourceDefault;
    for(size_t k=0;k<source.sources.size();++k) {
        packet.inputs.sources[k].world=source.sources[k].IsEmpty();
        if(!named(source.sources[k],"computedDefaultSpace",&packet.sourceDefault[k]) ||
           !named(source.sources[k],"computePointFrame",&packet.sourcePosed[k]))return false;
    }
    Unique(&packet.reads);*out=std::move(packet);return true;
}
bool RigExecBindScenePoseInterpolator(const RigExecScenePoseInterpolatorDescriptor &source,
    const RigExecSceneGraphBindingContext &context,const std::vector<RigExecValueId> &weights,
    const std::vector<RigExecValueId> &disabledWeights,RigExecBoundPoseInterpolator *out,std::string *error) {
    if(!out || !context.resolve)return Fail(error,"missing pose interpolator graph linker");
    if(weights.size()!=source.poseWeights.size() || disabledWeights.size()!=source.disabledPoseWeights.size())
        return Fail(error,"pose interpolator output identity count mismatch");
    RigExecBoundPoseInterpolator packet;packet.record=source.record;packet.weights=weights;packet.disabledWeights=disabledWeights;
    packet.unusableRotation="pose interpolator "+source.path.GetString()+" has no usable frame for its driver "+
        source.driver.GetString()+" after the pose walk; its weights are zero this generation";
    packet.unusableTranslation="pose interpolator "+source.path.GetString()+
        " could not measure its driver's translation; its weights are zero this generation";
    packet.countMismatch="pose interpolator "+source.path.GetString()+" solved an unexpected number of weights";
    if(!RigExecBindGraphTypedRead(source.enabled,context,&packet.enabled,error))return false;
    TypedInputs(packet.enabled,&packet.reads);
    for(const auto &read:source.numericReads) {
        RigExecGraphTypedRead binding;if(!RigExecBindGraphTypedRead(read,context,&binding,error))return false;
        TypedInputs(binding,&packet.reads);packet.numeric.push_back(std::move(binding));
    }
    packet.inputs.numeric.resize(packet.numeric.size());
    const auto frame=[&](const SdfPath &provider,bool rest,RigExecValueId *id) {
        if(provider.IsEmpty())return true;
        RigExecSceneGraphReadRequest request;request.consumer=source.path;request.reader=source.path;
        request.source=provider;request.domain=RigExecSceneValueDomain::Pose;
        request.phase=TfToken("final");if(rest)request.computation="computeRestFrame";
        if(!context.resolve(request,id,error))return false;packet.reads.push_back(*id);return true;
    };
    if(packet.numeric.empty() && (!frame(source.driver,false,&packet.driverFinal) ||
       !frame(source.driver,true,&packet.driverRest) || !frame(source.parent,false,&packet.parentFinal) ||
       !frame(source.parent,true,&packet.parentRest)))return false;
    Unique(&packet.reads);*out=std::move(packet);return true;
}
bool RigExecRunBoundSpaceSwitch(RigExecBoundSpaceSwitch *packet,RigExecTypedValueStore *values,
    RigExecValueId output,std::string *error) {
    if(!packet || !values)return Fail(error,"null bound space switch");
    auto &input=packet->inputs;VtValue active;
    input.active=packet->tokenIndex?0.0:packet->activeFallback;
    if(RigExecReadGraphTypedRead(packet->active,*values,&active,nullptr)) {
        if(packet->tokenIndex && active.IsHolding<TfToken>()) {
            const auto found=std::find(packet->labels.begin(),packet->labels.end(),active.UncheckedGet<TfToken>());
            if(found!=packet->labels.end())input.active=double(found-packet->labels.begin());
        } else if(active.IsHolding<double>())input.active=active.UncheckedGet<double>();
    }
    GfMatrix4d parentExpression(1.0);
    bool usable=Matrix(*values,packet->avars,&input.avars) && Matrix(*values,packet->posedDefault,&input.posedDefault) &&
        Matrix(*values,packet->parentExpression,&parentExpression) && Matrix(*values,packet->parentPosed,&input.parentPosed) &&
        Matrix(*values,packet->parentDefault,&input.parentDefault);
    input.parentDefaultInverse=parentExpression.GetInverse();
    input.parentDefault=RigExecPoseRoundTrip(input.parentDefault);
    if(input.hasCarry) {
        usable=usable && Matrix(*values,packet->spaceDefault,&input.spaceDefault) && Matrix(*values,packet->spacePosed,&input.spacePosed);
        input.spaceDefault=RigExecPoseRoundTrip(input.spaceDefault);
    }
    for(size_t k=0;k<input.sources.size();++k)if(!input.sources[k].world) {
        usable=usable && Matrix(*values,packet->sourceDefault[k],&input.sources[k].defaultSpace) &&
            Matrix(*values,packet->sourcePosed[k],&input.sources[k].posedSpace);
        input.sources[k].defaultSpace=RigExecPoseRoundTrip(input.sources[k].defaultSpace);
    }
    RigExecPointFrame frame;
    if(!usable || !RigExecRunSpaceSwitch(packet->record,input,&frame)) {
        frame.flags=0;values->Publish(output,frame);return Fail(error,packet->unavailable);
    }
    values->Publish(output,frame);return true;
}
bool RigExecRunBoundPoseInterpolator(RigExecBoundPoseInterpolator *packet,
    RigExecTypedValueStore *values,std::string *error) {
    if(!packet || !values)return Fail(error,"null bound pose interpolator");
    const auto publish=[&](bool solved) {
        for(size_t i=0;i<packet->weights.size();++i)
            values->Publish(packet->weights[i],solved?float(packet->scratch[i]):0.0f);
        for(auto output:packet->disabledWeights)values->Publish(output,0.0f);
    };
    auto &input=packet->inputs;VtValue value;input.enabled=true;
    if(RigExecReadGraphTypedRead(packet->enabled,*values,&value,nullptr) && value.IsHolding<bool>())input.enabled=value.UncheckedGet<bool>();
    for(size_t k=0;k<packet->numeric.size();++k) {
        input.numeric[k]=0;
        if(RigExecReadGraphTypedRead(packet->numeric[k],*values,&value,nullptr) && value.IsHolding<double>())input.numeric[k]=value.UncheckedGet<double>();
    }
    input.driverFinal=values->Read<RigExecPointFrame>(packet->driverFinal);
    input.driverRest=values->Read<RigExecPointFrame>(packet->driverRest);
    input.parentFinal=values->Read<RigExecPointFrame>(packet->parentFinal);
    input.parentRest=values->Read<RigExecPointFrame>(packet->parentRest);
    if(input.enabled && packet->record.poseCount && packet->numeric.empty() &&
       ((packet->parentFinal!=UINT64_MAX && !input.parentFinal) ||
        (packet->parentRest!=UINT64_MAX && !input.parentRest)))
    {publish(false);return Fail(error,packet->unusableRotation);}
    const auto status=RigExecRunPoseInterpolator(packet->record,input,&packet->scratch);
    using S=RigExecPoseInterpolatorStatus;
    publish(status==S::Success);
    if(status==S::Disabled)return true;
    if(status==S::UnusableRotation)return Fail(error,packet->unusableRotation);
    if(status==S::UnusableTranslation)return Fail(error,packet->unusableTranslation);
    if(status==S::CountMismatch)return Fail(error,packet->countMismatch);
    return true;
}
}
