#include "providerContextBinding.h"
#include <functional>
#include <algorithm>
namespace rigExec {
namespace {bool Fail(std::string *error,const std::string &message){if(error)*error=message;return false;}}
bool RigExecBindProviderContext(RigExecProviderProgram *program,
    const std::vector<std::string> &fullValueKeys,RigExecValueId sourceResult,
    const SdfPath &consumer,const std::string &phase,
    const std::map<RigExecValueId,RigExecValueId> &selectedFrames,
    RigExecValueId *result,size_t *beginOp,std::string *error,
    const std::map<std::string,std::string> &externalNames) {
    if(!program || !result || !beginOp || consumer.IsEmpty() || phase.empty() ||
       program->valueKeys.size()>fullValueKeys.size() || sourceResult>=program->valueKeys.size() ||
       program->ops.size()!=program->descriptors.size())
        return Fail(error,"invalid provider consumer context");
    for(size_t i=0;i<program->valueKeys.size();++i)if(program->valueKeys[i]!=fullValueKeys[i])
        return Fail(error,"provider context changed an existing typed value identity");
    std::map<std::string,RigExecValueId> ids;
    for(size_t i=0;i<fullValueKeys.size();++i)if(!ids.emplace(fullValueKeys[i],RigExecValueId(i)).second)
        return Fail(error,"duplicate typed value key in provider consumer context: "+fullValueKeys[i]);
    for(const auto &[before,after]:selectedFrames)if(before>=program->valueKeys.size() || after>=fullValueKeys.size())
        return Fail(error,"provider consumer context references an unknown frame version");
    std::map<RigExecValueId,size_t> producers;
    for(size_t i=0;i<program->ops.size();++i)producers.emplace(program->ops[i].output,i);
    std::map<RigExecValueId,std::string> externals;
    for(const auto &input:program->externalInputs)externals.emplace(input.value,input.computation);
    // A back edge is conservatively affected. It is cloned, never normalized
    // away or rejected before cross-domain SCC compilation.
    std::map<RigExecValueId,unsigned char> state;
    std::function<bool(RigExecValueId)> affected=[&](RigExecValueId value) {
        const auto replacement=selectedFrames.find(value);
        if(replacement!=selectedFrames.end())return replacement->second!=value;
        const auto external=externals.find(value);
        if(external!=externals.end())return externalNames.count(external->second)!=0;
        auto &mark=state[value];if(mark)return mark!=2;
        const auto producer=producers.find(value);if(producer==producers.end()){mark=2;return false;}
        mark=1;bool changed=false;
        const auto &op=program->ops[producer->second];
        for(auto input:op.inputs)if(input!=RigExecNoProviderValue)changed=affected(input)||changed;
        for(const auto &input:op.xforms)if(input.raw!=RigExecNoProviderValue)changed=affected(input.raw)||changed;
        mark=changed?3:2;return changed;
    };
    std::map<RigExecValueId,RigExecValueId> replacements=selectedFrames;
    if(!affected(sourceResult)) {
        // Only the root is contextualized when its entire closure is unaffected.
        // Fence its immediate producer inputs just as the full sweep would.
        const auto root=producers.find(sourceResult);
        if(root!=producers.end()) {
            const auto share=[&](RigExecValueId value) {
                if(value!=RigExecNoProviderValue && value!=sourceResult &&
                   producers.count(value) && !replacements.count(value))
                    replacements.emplace(value,value);
            };
            const auto &op=program->ops[root->second];
            for(auto input:op.inputs)share(input);
            for(const auto &input:op.xforms)share(input.raw);
        }
    } else {
        // Preserve the original sweep's visitation order, including back edges.
        state.clear();
        for(const auto &[value,index]:producers)
            if(value!=sourceResult && !replacements.count(value) && !affected(value))replacements.emplace(value,value);
    }
    const std::string prefix="context:"+consumer.GetString()+":"+phase+":"+
        program->valueKeys[size_t(sourceResult)]+":";
    program->valueKeys=fullValueKeys;program->valueIds=std::move(ids);
    *beginOp=program->ops.size();
    if(!RigExecCloneProviderContext(program,sourceResult,prefix,externalNames,replacements,result,error))return false;
    for(size_t i=*beginOp;i<program->descriptors.size();++i) {
        auto &descriptor=program->descriptors[i];
        for(const auto &xform:program->ops[i].xforms)
            if(xform.raw!=RigExecNoProviderValue)descriptor.reads.push_back(xform.raw);
        std::sort(descriptor.reads.begin(),descriptor.reads.end());
        descriptor.reads.erase(std::unique(descriptor.reads.begin(),descriptor.reads.end()),descriptor.reads.end());
    }
    return true;
}
bool RigExecBindProviderParentDelivery(RigExecProviderProgram *program,
    const SdfPath &consumer,RigExecValueId expression,RigExecValueId *output,std::string *error) {
    return RigExecBindProviderParentDelivery(program,consumer,expression,output,error,nullptr);
}
bool RigExecBindProviderParentDelivery(RigExecProviderProgram *program,
    const SdfPath &consumer,RigExecValueId expression,RigExecValueId *output,std::string *error,
    const std::map<RigExecValueId,size_t> *firstProducers) {
    if(!program || !output || consumer.IsEmpty() || expression>=program->valueKeys.size())
        return Fail(error,"invalid parent-space delivery binding");
    size_t producer=program->ops.size();
    if(firstProducers) {
        const auto found=firstProducers->find(expression);
        if(found!=firstProducers->end()) {
            producer=found->second;
            if(producer>=program->ops.size() || program->ops[producer].output!=expression)
                return Fail(error,"invalid parent-space first-producer index");
        }
    } else {
        const auto found=std::find_if(program->ops.begin(),program->ops.end(),
            [&](const auto &op){return op.output==expression;});
        if(found!=program->ops.end()) producer=size_t(found-program->ops.begin());
    }
    if(producer==program->ops.size() || program->ops[producer].kind!=RigExecProviderOpKind::SpaceExpression ||
       program->ops[producer].inputs.size()!=3 || program->ops[producer].inputs[1]!=RigExecNoProviderValue)
        return Fail(error,"parent-space delivery requires an unconnected expression: "+consumer.GetString());
    const std::string key="parentDelivery:"+consumer.GetString();
    const auto existing=program->valueIds.find(key);
    if(existing!=program->valueIds.end()){*output=existing->second;return true;}
    auto op=program->ops[producer];op.output=RigExecValueId(program->valueKeys.size());op.inputs[2]=RigExecNoProviderValue;
    RigExecOpDescriptor descriptor;descriptor.key=key;
    descriptor.reads={op.inputs[0]};descriptor.writes={op.output};
    *output=op.output;program->valueIds.emplace(key,op.output);program->valueKeys.push_back(key);
    program->ops.push_back(std::move(op));program->descriptors.push_back(std::move(descriptor));
    return true;
}
bool RigExecRebindProviderInputs(RigExecProviderProgram *program,size_t initialOpCount,
    const RigExecProviderInputBinding &bind,std::string *error) {
    if(!program || !bind || initialOpCount>program->ops.size() ||
       program->ops.size()!=program->descriptors.size())return Fail(error,"invalid original provider input binding");
    const char *const channels[]={"avars:unitScaleFactor","avars:tx","avars:ty","avars:tz",
        "avars:sx","avars:sy","avars:sz","avars:rx","avars:ry","avars:rz","avars:rspin",
        "avars:rotationOrder","avars:rotationSign"};
    const char *const spaces[]={"posed:space","posed:defaultSpace","parent:defaultSpace","parent:space"};
    const char *const restChannels[]={"rest:tx","rest:ty","rest:tz","rest:rx","rest:ry","rest:rz"};
    const char *const defaultChannels[]={"default:tx","default:ty","default:tz","default:rx","default:ry","default:rz"};
    for(size_t i=0;i<initialOpCount;++i) {
        auto op=program->ops[i];bool changed=false;
        const auto input=[&](size_t k,const char *name) {
            if(k>=op.inputs.size())return Fail(error,"provider operation has an incomplete input record");
            const auto original=op.inputs[k];RigExecValueId value=original;
            if(!bind(op.owner.AppendProperty(TfToken(name)),original,&value,error))return false;
            if((value==RigExecNoProviderValue && original!=RigExecNoProviderValue) ||
               (value!=RigExecNoProviderValue && value>=program->valueKeys.size()))
                return Fail(error,"provider input binding has no produced value");
            changed|=value!=original;op.inputs[k]=value;return true;
        };
        if(op.kind==RigExecProviderOpKind::RestFrame) {
            if(!input(0,"rest:space"))return false;
            for(size_t k=0;k<6;++k)if(!input(k+1,restChannels[k]))return false;
        } else if(op.kind==RigExecProviderOpKind::DefaultSpace) {
            if(!input(2,"parent:defaultSpace"))return false;
            for(size_t k=0;k<6;++k)if(!input(k+3,defaultChannels[k]))return false;
        } else if(op.kind==RigExecProviderOpKind::PosedFrame) {
            for(size_t k=0;k<4;++k)if(!input(k,spaces[k]))return false;
            for(size_t k=0;k<13;++k)if(!input(k+4,channels[k]))return false;
        } else if(op.kind==RigExecProviderOpKind::AvarMatrix) {
            for(size_t k=0;k<13;++k)if(!input(k,channels[k]))return false;
        }
        if(!changed)continue;
        program->ops[i]=std::move(op);auto &descriptor=program->descriptors[i];descriptor.reads.clear();
        for(auto value:program->ops[i].inputs)if(value!=RigExecNoProviderValue)descriptor.reads.push_back(value);
        for(const auto &xform:program->ops[i].xforms)if(xform.raw!=RigExecNoProviderValue)descriptor.reads.push_back(xform.raw);
        std::sort(descriptor.reads.begin(),descriptor.reads.end());
        descriptor.reads.erase(std::unique(descriptor.reads.begin(),descriptor.reads.end()),descriptor.reads.end());
    }
    return true;
}

}
