#include "geometryGraphBinding.h"
#include <algorithm>
namespace rigExec {
bool RigExecBindGeometryGraph(const RigExecSceneGeometryDescriptor &descriptor,
    const RigExecSceneGraphBindingContext &context,RigExecGeometryGraphBinding *out,std::string *error)
{
    if(!out || !context.resolve) {if(error)*error="missing geometry graph linker";return false;}
    RigExecGeometryGraphBinding result;result.descriptor=descriptor;
    auto resolve=[&](const SdfPath &consumer,const SdfPath &source,const TfToken &phase,
        RigExecSceneValueDomain domain,bool atDefault,bool raw,RigExecValueId *id) {
        if(source.IsEmpty())return true;
        RigExecSceneGraphReadRequest request; request.consumer=consumer;request.source=source;
        request.reader=descriptor.mover;request.phase=phase;request.domain=domain;
        request.atDefault=atDefault;request.raw=raw;
        if(!context.resolve(request,id,error))return false;
        result.reads.push_back(*id);return true;
    };
    result.leaves.resize(descriptor.record.leaves.keys.size(),UINT64_MAX);
    result.typedLeaves.resize(result.leaves.size());
    for(size_t k=0;k<result.leaves.size();++k) {
        if(k>=descriptor.bound.size() || !descriptor.bound[k])continue;
        const auto &key=descriptor.record.leaves.keys[k];const auto &input=descriptor.inputs[k];
        if(key.flavour!=RigExecRevisionLeafFlavour::Raw) {
            if(k>=descriptor.typedLeaves.size()) {if(error)*error="missing ordered geometry read";return false;}
            auto reader=context;
            reader.resolve=[&](RigExecSceneGraphReadRequest request,RigExecValueId *id,std::string *diagnostic) {
                request.reader=descriptor.mover;request.atDefault=key.time==RigExecRevisionLeafTime::AtDefault;
                if(key.type==RigExecRevisionLeafType::Vec3fArray && request.source.GetName()=="points")
                    request.domain=RigExecSceneValueDomain::Points;
                if(!context.resolve(request,id,diagnostic))return false;
                result.reads.push_back(*id);return true;
            };
            if(!RigExecBindGraphTypedRead(descriptor.typedLeaves[k],reader,&result.typedLeaves[k],error))return false;
            continue;
        }
        const auto domain=key.type==RigExecRevisionLeafType::Vec3fArray && key.path.GetNameToken()==TfToken("points")?
            RigExecSceneValueDomain::Points:RigExecSceneValueDomain::Property;
        if(!resolve(input.consumer,input.source,input.readPhase,domain,
            key.time==RigExecRevisionLeafTime::AtDefault,key.flavour==RigExecRevisionLeafFlavour::Raw,
            &result.leaves[k]))return false;
    }
    const auto &binding=descriptor.record.binding;
    const TfToken phase(binding.transformPhase.GetAsString());
    if(!resolve(binding.transform,binding.transform,phase,RigExecSceneValueDomain::Pose,false,false,&result.transform) ||
       !resolve(binding.transformSpace,binding.transformSpace,phase,RigExecSceneValueDomain::Pose,false,false,&result.transformSpace) ||
       !resolve(binding.carrySpace,binding.carrySpace,phase,RigExecSceneValueDomain::Pose,false,false,&result.carry))return false;
    result.influences.resize(binding.influences.size(),UINT64_MAX);
    for(size_t i=0;i<binding.influences.size();++i)
        if(!resolve(binding.influences[i],binding.influences[i],phase,RigExecSceneValueDomain::Pose,false,false,&result.influences[i]))return false;
    if(!binding.weightObject.IsEmpty()) {
        if(!context.weightPacket || !context.weightPacket(binding.weightObject,&result.weights,error))return false;
        result.reads.push_back(result.weights);
    }
    if(!resolve(binding.driverFrames,binding.driverFrames,phase,RigExecSceneValueDomain::SolverAggregate,false,false,&result.driverFrames))return false;
    auto rest=[&](const SdfPath &path,RigExecValueId *id) {
        if(path.IsEmpty())return true;
        RigExecSceneGraphReadRequest request;request.consumer=path;request.source=path;
        request.reader=descriptor.mover;request.domain=RigExecSceneValueDomain::Pose;
        request.computation="computeRestFrame";request.atDefault=true;
        if(!context.resolve(request,id,error))return false;
        result.reads.push_back(*id);return true;
    };
    if(!rest(binding.transform,&result.transformRest) || !rest(binding.transformSpace,&result.transformSpaceRest) ||
       !rest(binding.carrySpace,&result.carryRest))return false;
    result.influenceRest.resize(binding.influences.size(),UINT64_MAX);
    for(size_t i=0;i<binding.influences.size();++i)
        if(!rest(binding.influences[i],&result.influenceRest[i]))return false;
    if(RigExecIsDerivedMatrixOp(descriptor.record.op)) {
        const SdfPath providers[3]={binding.transform,binding.transformSpace,binding.carrySpace};
        for(int k=0;k<3;++k) {
            if(!resolve(providers[k],providers[k],TfToken("base"),RigExecSceneValueDomain::Pose,false,false,&result.projectorBase[k]) ||
               !resolve(providers[k],providers[k],TfToken("final"),RigExecSceneValueDomain::Pose,false,false,&result.projectorFinal[k]))return false;
        }
    }
    std::sort(result.reads.begin(),result.reads.end());result.reads.erase(std::unique(result.reads.begin(),result.reads.end()),result.reads.end());
    *out=std::move(result);return true;
}
bool RigExecReadGeometryGraphLeaves(const RigExecGeometryGraphBinding &binding,
    const RigExecTypedValueStore &store,std::vector<VtValue> *out,std::string *error)
{
    if(!out) {if(error)*error="null geometry graph leaf buffer";return false;}
    out->resize(binding.leaves.size());
    for(size_t k=0;k<out->size();++k) {
        const auto &key=binding.descriptor.record.leaves.keys[k];const auto id=binding.leaves[k];
        if(key.flavour==RigExecRevisionLeafFlavour::Present) {
            bool present=false;
            for(const auto &hop:binding.typedLeaves[k].hops)
                if(hop.overlay<store.values.size() && store.values[size_t(hop.overlay)].initialized &&
                   store.values[size_t(hop.overlay)].authoritative) {present=true;break;}
            (*out)[k]=VtValue(present);continue;
        }
        if(key.flavour!=RigExecRevisionLeafFlavour::Raw) {
            VtValue value;
            const bool have=RigExecReadGraphTypedRead(binding.typedLeaves[k],store,&value);
            if(have && key.type==RigExecRevisionLeafType::Dial && value.IsHolding<float>())
                value=VtValue(double(value.UncheckedGet<float>()));
            (*out)[k]=have?value:key.fallback;continue;
        }
        VtValue value;
        const bool have=id!=UINT64_MAX && RigExecReadSceneGraphValue(store,id,&value);
        (*out)[k]=have && !value.IsEmpty()?value:key.fallback;
    }
    return true;
}
bool RigExecReadGeometryGraphInputs(const RigExecGeometryGraphBinding &binding,
    const RigExecTypedValueStore &store,bool posedPoints,RigExecGeometryGraphWorkspace *workspace,
    RigExecGeometryInputs *out,std::string *error)
{
    if(!workspace || !out) {if(error)*error="missing geometry graph workspace";return false;}
    if(!RigExecReadGeometryGraphLeaves(binding,store,&workspace->leaves,error))return false;
    auto matrix=[&](RigExecValueId current,RigExecValueId rest,GfMatrix4d *value) {
        *value=GfMatrix4d(1);if(current==UINT64_MAX)return true;
        if(current>=store.values.size() || rest>=store.values.size() ||
           store.values[size_t(current)].blocked || store.values[size_t(rest)].blocked)return false;
        const auto *r=store.Read<RigExecPointFrame>(rest),*p=store.Read<RigExecPointFrame>(current);
        return r && r->IsValid() && p && RigExecPointsToMatrix(r->points,*p,value);
    };
    GfMatrix4d transform,space,carry;
    if(!matrix(binding.transform,binding.transformRest,&transform) ||
       !matrix(binding.transformSpace,binding.transformSpaceRest,&space) ||
       !matrix(binding.carry,binding.carryRest,&carry)) {
        if(error)*error="geometry provider frame unavailable";return false;
    }
    workspace->influences.resize(binding.influences.size());
    for(size_t i=0;i<binding.influences.size();++i)
        if(!matrix(binding.influences[i],binding.influenceRest[i],&workspace->influences[i])) {
            if(error)*error="geometry influence frame unavailable";return false;
        }
    const auto &record=binding.descriptor.record;
    const bool matrixOp=record.op==RigExecRevisionOp::Matrix;
    const GfMatrix4d *reference=matrixOp && !workspace->influences.empty()?&workspace->influences[0]:nullptr;
    const GfMatrix4d *referenceSpace=matrixOp && workspace->influences.size()>1?&workspace->influences[1]:nullptr;
    workspace->transform=RigExecGeometryMatrixInPointFrame(transform,
        binding.transformSpace==UINT64_MAX?nullptr:&space,reference,referenceSpace,
        posedPoints,binding.carry==UINT64_MAX?nullptr:&carry);
    workspace->carry=carry;out->leaves=&workspace->leaves;
    out->transform=&workspace->transform;out->carry=&workspace->carry;
    out->influenceTransforms=&workspace->influences;out->weights=nullptr;out->driverFrames=nullptr;
    workspace->weights=VtValue();workspace->driverFrames=VtValue();
    if(binding.weights!=UINT64_MAX && RigExecReadSceneGraphValue(store,binding.weights,&workspace->weights) &&
       workspace->weights.IsHolding<RigExecWeightPacket>())
        out->weights=&workspace->weights.UncheckedGet<RigExecWeightPacket>();
    if(binding.driverFrames!=UINT64_MAX && RigExecReadSceneGraphValue(store,binding.driverFrames,&workspace->driverFrames) &&
       workspace->driverFrames.IsHolding<RigExecPointFrameArray>())
        out->driverFrames=&workspace->driverFrames.UncheckedGet<RigExecPointFrameArray>();
    if(record.op==RigExecRevisionOp::Skin) {
        auto value=[&](RigExecRevisionLeafRole role)->const VtValue * {
            const int key=record.leaves.Role(role);
            return key>=0 && size_t(key)<workspace->leaves.size()?&workspace->leaves[size_t(key)]:nullptr;
        };
        const auto *iv=value(RigExecRevisionLeafRole::JointIndices),*wv=value(RigExecRevisionLeafRole::JointWeights),
            *sv=value(RigExecRevisionLeafRole::ElementSize);
        const VtIntArray emptyIndices;const VtFloatArray emptyWeights;
        const auto &indices=iv && iv->IsHolding<VtIntArray>()?iv->UncheckedGet<VtIntArray>():emptyIndices;
        const auto &weights=wv && wv->IsHolding<VtFloatArray>()?wv->UncheckedGet<VtFloatArray>():emptyWeights;
        const int size=sv && sv->IsHolding<int>()?sv->UncheckedGet<int>():1;
        auto topology=std::make_shared<RigExecSkinTopology>();
        RigExecBuildSkinTopology(TfSpan<const int>(indices.cdata(),indices.size()),
            TfSpan<const float>(weights.cdata(),weights.size()),size,binding.influences.size(),topology.get());
        if(!workspace->skinTopology || !(*workspace->skinTopology==*topology))workspace->skinTopology=std::move(topology);
        out->skinTopology=&workspace->skinTopology;
    }
    return true;
}

bool RigExecReadGeometryGraphProjector(const RigExecGeometryGraphBinding &binding,
    const RigExecTypedValueStore &store,RigExecGeometryGraphWorkspace *workspace,
    RigExecSurfaceProjectorFrames *frames,RigExecProjectorReads *reads,std::string *error)
{
    if(!workspace || !frames || !reads)return false;
    if(!RigExecReadGeometryGraphLeaves(binding,store,&workspace->leaves,error))return false;
    RigExecRevisionLeafView view;view.decl=&binding.descriptor.record.leaves;view.values=&workspace->leaves;
    RigExecReadProjectorTargetFromLeaves(binding.descriptor.record.op,binding.descriptor.record.binding,view,reads);
    *frames=RigExecSurfaceProjectorFrames();
    const RigExecValueId rests[3]={binding.transformRest,binding.transformSpaceRest,binding.carryRest};
    for(int k=0;k<3;++k) {
        if(binding.projectorBase[k]==UINT64_MAX)continue;
        frames->named[k]=true;
        const auto *rest=store.Read<RigExecPointFrame>(rests[k]);
        const auto *base=store.Read<RigExecPointFrame>(binding.projectorBase[k]);
        const auto *final=store.Read<RigExecPointFrame>(binding.projectorFinal[k]);
        if(rests[k]>=store.values.size() || binding.projectorBase[k]>=store.values.size() ||
           binding.projectorFinal[k]>=store.values.size() || store.values[size_t(rests[k])].blocked ||
           store.values[size_t(binding.projectorBase[k])].blocked || store.values[size_t(binding.projectorFinal[k])].blocked)continue;
        if(!rest || !rest->IsValid() || !base || !final)continue;
        GfMatrix4d baseMatrix,finalMatrix;
        if(!RigExecPointsToMatrix(rest->points,*base,&baseMatrix) ||
           !RigExecPointsToMatrix(rest->points,*final,&finalMatrix))continue;
        frames->base[k]=RigExecWorldFromRest(rest->points,baseMatrix);
        frames->final[k]=RigExecWorldFromRest(rest->points,finalMatrix);
        frames->resolved[k]=true;
    }
    return true;
}

}
