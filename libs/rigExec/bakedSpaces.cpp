#include "bakedProgramImpl.h"
#include "rigEvaluator.h"
#include "rigEvaluatorDependencies.h"
#include "crossDomainInputs.h"
#include "rigExecGraph/usdSceneAccess.h"
#include "rigExecGraph/providerContextBinding.h"
#include "rigExecGraph/providerRefresh.h"
#include "parallel.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/resolveInfo.h"
#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace rigExec {
namespace {
RigExecRevisionLeafType _Type(const SdfValueTypeName &type)
{
    using T=RigExecRevisionLeafType;
    if(type==SdfValueTypeNames->Matrix4d) return T::Matrix4d;
    if(type==SdfValueTypeNames->Double3) return T::Vec3d;
    if(type==SdfValueTypeNames->Float3) return T::Vec3f;
    if(type==SdfValueTypeNames->Token) return T::Token;
    if(type==SdfValueTypeNames->Float) return T::Float;
    if(type==SdfValueTypeNames->Int) return T::Int;
    if(type==SdfValueTypeNames->Bool) return T::Bool;
    return T::Double;
}
}
bool RigExecBakedBuildSpaces(RigExecBakedProgramImpl *program,UsdTimeCode capture,
    std::string *error,const RigExecSceneDescriptors *captured,
    const UsdStageWeakPtr &capturedStage,uint64_t capturedSerial)
{
    auto &B=*program;
    auto sceneOwner=std::make_shared<RigExecSceneDescriptors>();
    auto &scene=*sceneOwner;
    const RigExecUsdSceneAccess source(B.stage);
    std::set<SdfPath> selected;
    for(const auto &entry:B.index) selected.insert(entry.first);
    const std::vector<UsdTimeCode> identities={capture,UsdTimeCode::Default()};
    bool reused=false;
    if(captured && capturedStage==UsdStageWeakPtr(B.stage) &&
       captured->rigRoot==B.evaluator->GetRigPath() &&
       capturedSerial==B.evaluator->GetStageEditSerial()) {
        // Duplicate Default identities select duplicate exact input rows.
        // A future capture time absent from this snapshot takes the raw
        // source path; it never substitutes a Default value for AtTime.
        std::string ignored;
        reused=RigExecSelectSceneDescriptorIdentities(*captured,identities,&scene,&ignored) &&
               capturedSerial==B.evaluator->GetStageEditSerial();
    }
    if(!reused && !RigExecCaptureSceneDescriptors(source,B.evaluator->GetRigPath(),identities,&scene,error)) return false;
    if(!RigExecBuildProviderProgram(scene,false,&B.providerProgram,error,&selected)) return false;
    B.sceneDescriptors=std::move(sceneOwner);
    B.providerActive.assign(B.paths.size(),0);
    for(size_t slot=0;slot<B.paths.size();++slot) {
        const auto node=scene.nodes.find(B.paths[slot]);
        B.providerActive[slot]=node!=scene.nodes.end() && node->second.fact.active;
    }
    B.providerValues=RigExecTypedValueStore(B.providerProgram.valueKeys.size());
    std::vector<RigExecValueId> changed;
    if(!RigExecSampleProviderProgram(B.providerProgram,scene,0,{},&B.providerValues,&changed,error)) return false;
    B.providerLeaves=RigExecBakedPathLeaves();
    B.providerLeafValues.clear();
    B.providerLeafBlocked.clear();
    B.providerFrozenKeys.clear();
    for(const auto &input:B.providerProgram.sampled) {
        const auto attribute=scene.attributes.find(input.attribute);
        if(attribute==scene.attributes.end()) continue;
        const auto &fact=attribute->second;
        const VtValue fallback=fact.inputs.empty()?VtValue():fact.inputs[0].raw;
        B.providerLeaves.decl.Add({input.attribute,_Type(fact.fact.type),
            RigExecRevisionLeafTime::AtTime,RigExecRevisionLeafFlavour::Raw,fallback});
        B.providerLeafValues.push_back(input.value);
        B.providerLeafBlocked.push_back(!fact.inputs.empty() && fact.inputs[0].rawBlocked);
        B.providerFrozenKeys.push_back(input.attribute.GetPrimPath().AppendProperty(
            TfToken("rigExec:providerRaw:"+input.attribute.GetName())));
        B.named.insert(input.attribute); B.prims.insert(input.attribute.GetPrimPath());
    }
    RigExecBakedBindPathLeaves(B.stage,&B.providerLeaves);
    const char *names[]={"rest:space","default:space","posed:space","parent:space",
        "parent:defaultSpace","avars:defaultSpace","posed:defaultSpace"};
    // Connected consumers refresh an expression closure against native base
    // versions. Unconnected namespace expressions keep their initial context.
    // These binding calls append ops without changing existing output IDs.
    // Preserve the first producer, including duplicates, and extend through
    // every context/delivery suffix before the next indexed delivery lookup.
    std::map<RigExecValueId,size_t> firstProducers;
    size_t indexedOps=0;
    const auto indexAppendedProducers=[&] {
        for(;indexedOps<B.providerProgram.ops.size();++indexedOps)
            firstProducers.emplace(B.providerProgram.ops[indexedOps].output,indexedOps);
    };
    for(size_t slot=0;slot<B.paths.size();++slot) {
        if(B.slotKind[slot]!=RigExecBakedSlotKind::FirstFramePose) continue;
        for(size_t channel=0;channel<7;++channel) {
            const SdfPath consumer=B.paths[slot].AppendProperty(TfToken(names[channel]));
            const auto found=B.providerProgram.attributeValues.find(consumer);
            if(found==B.providerProgram.attributeValues.end()) continue;
            RigExecValueId value=found->second;
            const auto attribute=scene.attributes.find(consumer);
            if(attribute!=scene.attributes.end() && attribute->second.fact.connections.size()==1 &&
               (attribute->second.readPhase.IsEmpty() || attribute->second.readPhase=="base"))
            {
                size_t beginOp=0;
                if(!RigExecBindProviderContext(&B.providerProgram,B.providerProgram.valueKeys,
                    value,consumer,"base",{},&value,&beginOp,error,
                    {{"computePointFrame","computeBasePointFrame"}}))return false;
            }
            if(channel==3 && attribute!=scene.attributes.end() &&
               attribute->second.fact.connections.empty()) {
                // The ladder uses this matrix only for authored/override
                // selection. Compose and checkpoint operations supply the
                // actual namespace fallback through their parent reads.
                indexAppendedProducers();
                if(!RigExecBindProviderParentDelivery(&B.providerProgram,consumer,value,&value,error,&firstProducers))return false;
            }
            B.ladders[slot].spaceValues[channel]=int(value);
        }
    }
    B.providerExternalSlots.clear();
    for(const auto &input:B.providerProgram.externalInputs) {
        const auto found=B.index.find(input.owner);
        if(found==B.index.end()) {
            if(error) *error="provider expression frame is outside native storage: "+input.owner.GetString();
            return false;
        }
        B.providerExternalSlots.push_back(found==B.index.end()?-1:found->second);
    }
    B.providerValues.values.resize(B.providerProgram.valueKeys.size());
    B.poseProviderInputs.resize(B.paths.size());
    B.providerParentRawLeaves.assign(B.paths.size(),-1);
    for(size_t k=0;k<B.providerProgram.sampled.size();++k) {
        const auto &path=B.providerProgram.sampled[k].attribute;
        if(path.GetName()!="parent:space")continue;
        const auto found=B.index.find(path.GetPrimPath());
        if(found!=B.index.end())B.providerParentRawLeaves[size_t(found->second)]=int(k);
    }
    B.connectedPoseProviders.assign(B.paths.size(),0);
    B.providerRefreshTemplates.assign(B.paths.size(),RigExecNoProviderValue);
    const char *poseInputs[]={"posed:space","posed:defaultSpace","parent:defaultSpace","parent:space",
        "avars:unitScaleFactor","avars:tx","avars:ty","avars:tz","avars:sx","avars:sy","avars:sz",
        "avars:rx","avars:ry","avars:rz","avars:rspin","avars:rotationOrder","avars:rotationSign"};
    // Every slot's pose-input closure from one _PoseInputGraph batch, as the
    // compile computes its own: an attribute many closures reach is read
    // once. Providers and connectedPose equal _CollectPoseInputInfo's for
    // each prim; RIGEXEC_VERIFY_POSEINFO re-walks every closure to check.
    // Nothing is skipped, so a joint-bound slot still lists its providers;
    // only an empty path gets no entry, and the walker's answer for it is
    // empty too. Owning thread; the graph spreads its stage reads as the
    // other Build-time regions do.
    std::unordered_map<SdfPath,evaluatorDetail::_PoseInputInfo,SdfPath::Hash> poseInfos;
    {
        evaluatorDetail::_PoseInputGraph graph;
        graph.Extend(B.stage,B.paths,[](const SdfPath &){return false;},
            RigExecParallelEvaluationEnabled() && !RigExecFrozenSerialActive(),*B.profiler,&poseInfos);
    }
    const evaluatorDetail::_PoseInputInfo noPoseInputs;
    for(size_t slot=0;slot<B.paths.size();++slot) {
        const auto computed=poseInfos.find(B.paths[slot]);
        const auto &info=computed!=poseInfos.end()?computed->second:noPoseInputs;
        for(const auto &path:info.providers) {
            const auto found=B.index.find(path);
            if(found!=B.index.end()) B.poseProviderInputs[slot].push_back(found->second);
        }
        if(!info.connectedPose || B.slotKind[slot]!=RigExecBakedSlotKind::FirstFramePose ||
           B.jointSolverBinding->count(B.paths[slot])) continue;
        B.connectedPoseProviders[slot]=1;
        auto &p=B.providerProgram;
        const auto output=RigExecValueId(p.valueKeys.size());
        const std::string key="refreshTemplate:"+B.paths[slot].GetString();
        p.valueKeys.push_back(key);p.valueIds.emplace(key,output);
        RigExecProviderOp op{RigExecProviderOpKind::PosedFrame,B.paths[slot],output,{}};
        for(const auto *name:poseInputs)
            op.inputs.push_back(p.FindValue(RigExecProviderAttributeKey(B.paths[slot].AppendProperty(TfToken(name)))));
        const auto type=scene.nodes.at(B.paths[slot]).fact.type;
        op.scaleAvars=type=="RigExecControl" || type=="RigExecJoint";
        RigExecOpDescriptor descriptor;descriptor.key=key;descriptor.writes={output};
        for(auto input:op.inputs)if(input!=RigExecNoProviderValue)descriptor.reads.push_back(input);
        std::sort(descriptor.reads.begin(),descriptor.reads.end());
        descriptor.reads.erase(std::unique(descriptor.reads.begin(),descriptor.reads.end()),descriptor.reads.end());
        B.providerRefreshTemplates[slot]=output;
        B.providerRefreshTemplateOps.push_back(p.ops.size());
        p.ops.push_back(std::move(op));p.descriptors.push_back(std::move(descriptor));
    }
    RigExecSpellProviderOwners(&B.providerProgram);
    B.providerValues.values.resize(B.providerProgram.valueKeys.size());
    RigExecBakedSampleSpaces(&B,capture,true);
    return true;
}
const VtValue *RigExecBakedSpaceLeafOverlay(const RigExecBakedProgramImpl &B,size_t k)
{
    const auto &path=B.providerProgram.sampled[k].attribute;
    if(!B.routedOverrides.empty()) {
        const auto drag=B.routedOverrides.find(path);
        if(drag!=B.routedOverrides.end()) return &drag->second;
    }
    if(!B.upstream.empty()) {
        const auto upstream=B.upstream.find(path);
        if(upstream!=B.upstream.end()) return &upstream->second;
    }
    // The index holds the two Build-only lookups below, per leaf.
    if(const auto *index=B.spaceLeafIndex.get(); index && k<index->headSlot.size()) {
        const int slot=index->headSlot[k];
        if(slot>=0 && size_t(slot)<B.headOverrides.size() && !B.headOverrides[size_t(slot)].IsEmpty())
            return &B.headOverrides[size_t(slot)];
        for(uint32_t i=index->numberBegin[k];i<index->numberBegin[k+1];++i) {
            const int number=index->numbers[i];
            if(number>=0 && size_t(number)<B.overridden.size() && B.overridden[size_t(number)])
                return B.resolvedInputs?B.resolvedInputs->Find(path):nullptr;
        }
        return nullptr;
    }
    const auto head=B.headOverrideSlots.find(path);
    if(head!=B.headOverrideSlots.end() && head->second<B.headOverrides.size() &&
       !B.headOverrides[head->second].IsEmpty()) return &B.headOverrides[head->second];
    const auto indices=B.overridableInputs.find(path);
    if(indices!=B.overridableInputs.end()) for(const int index:indices->second)
        if(index>=0 && size_t(index)<B.overridden.size() && B.overridden[size_t(index)])
            return B.resolvedInputs?B.resolvedInputs->Find(path):nullptr;
    return nullptr;
}
GfMatrix4d RigExecBakedProviderParentRaw(const RigExecBakedProgramImpl &B,int slot)
{
    const int index=B.providerParentRawLeaves[size_t(slot)];
    if(index<0 || B.providerLeafBlocked[size_t(index)])return GfMatrix4d(1.0);
    const auto &value=B.providerLeaves.values[size_t(index)];
    return value.IsHolding<GfMatrix4d>()?value.UncheckedGet<GfMatrix4d>():GfMatrix4d(1.0);
}
void RigExecBakedSampleSpaces(RigExecBakedProgramImpl *program,UsdTimeCode time,bool all)
{
    auto &B=*program;
    auto &leaves=B.providerLeaves;
    const bool full=all || !leaves.sampled || leaves.stamp!=B.programStamp;
    const bool timeMoved=!leaves.sampled || time!=leaves.time;
    const bool defaultMoved=timeMoved && (time.IsDefault() || leaves.time.IsDefault());
    for(size_t k=0;k<leaves.attributes.size();++k) {
        leaves.changed[k]=0;
        if(!full && !leaves.mustSample[k] &&
           !(timeMoved && (leaves.varying[k] || defaultMoved))) continue;
        leaves.mustSample[k]=0;
        ++B.pathLeafSamples;
        const auto &attribute=leaves.attributes[k];
        VtValue value;
        const bool blocked=attribute && attribute.GetResolveInfo(time).ValueIsBlocked();
        if(attribute) attribute.Get(&value,time);
        leaves.changed[k]=!RigExecBakedHeadValueSame(value,leaves.values[k]) ||
            bool(B.providerLeafBlocked[k])!=blocked;
        if(leaves.changed[k]) RigExecBakedNoteSpaceLeafSampled(&B,k);
        leaves.values[k]=std::move(value);
        B.providerLeafBlocked[k]=blocked;
    }
    leaves.sampled=true;
    leaves.time=time;
    leaves.stamp=B.programStamp;

}
void RigExecBakedPlanProviderRefreshes(RigExecBakedProgramImpl *program)
{
    auto &B=*program;
    B.providerRefreshes.clear();B.providerFrameInputs.clear();
    B.providerRefreshBefore.assign(B.walkSteps.size()+1,{});
    if(std::find(B.connectedPoseProviders.begin(),B.connectedPoseProviders.end(),char(1))==B.connectedPoseProviders.end())return;
    std::vector<std::vector<std::pair<uint32_t,uint32_t>>> constrained(B.paths.size());
    for(size_t checkpoint=0;checkpoint<=B.walkSteps.size();++checkpoint) {
        std::vector<int> requested;
        SdfPath reader=B.evaluator->GetRigPath();
        const auto add=[&](int slot){if(slot>=0 && size_t(slot)<B.paths.size())requested.push_back(slot);};
        const auto native=[&](int index) {
            if(index>=0)for(int slot:B.nativeSources[size_t(index)].ancestorSlots)add(slot);
        };
        if(checkpoint==B.walkSteps.size()) {
            for(size_t slot=0;slot<B.paths.size();++slot)add(int(slot));
        } else {
            const auto &walk=B.walkSteps[checkpoint];
            if(walk.solverBatch) {
                for(int index:walk.batchSolvers) {
                    const auto &solver=B.solvers[size_t(index)];reader=solver.path;
                    for(int slot:solver.controls)add(slot);
                    add(solver.start);add(solver.root);add(solver.mid);add(solver.end);add(solver.pole);add(solver.spaceSlot);
                    for(const auto &rest:solver.restRefs)add(rest.first);
                }
            } else {
                const auto &c=B.constraints[size_t(walk.index)];reader=c.path;
                add(c.target);for(int slot:c.targetSlots)add(slot);
                for(int slot:c.sources)add(slot);
                for(int index:c.sourceNatives)native(index);
                add(c.worldUpObject);native(c.worldUpNative);add(c.effector);native(c.effectorNative);
                for(int slot:c.poleObjects)add(slot);
                for(int index:c.poleObjectNatives)native(index);
                const auto weight=B.index.find(c.weightObject);if(weight!=B.index.end())add(weight->second);
            }
        }
        std::vector<uint8_t> state(B.paths.size(),0);
        const std::function<void(int)> visit=[&](int slot) {
            if(state[size_t(slot)] || B.jointSolverBinding->count(B.paths[size_t(slot)]))return;
            state[size_t(slot)]=1;
            const auto &dependencies=B.poseProviderInputs[size_t(slot)];
            for(auto it=dependencies.rbegin();it!=dependencies.rend();++it)visit(*it);
            state[size_t(slot)]=2;
            if(!B.connectedPoseProviders[size_t(slot)])return;
            RigExecBakedProgramImpl::ProviderRefresh refresh;
            refresh.slot=slot;refresh.checkpoint=checkpoint;refresh.reader=reader;
            refresh.key="providerRefresh:"+reader.GetString()+":"+std::to_string(checkpoint)+":"+B.paths[size_t(slot)].GetString();
            refresh.priorConstraints=constrained[size_t(slot)];
            for(size_t descendant=size_t(slot)+1;descendant<B.paths.size() &&
                B.paths[descendant].HasPrefix(B.paths[size_t(slot)]);++descendant) {
                if(B.slotKind[descendant]!=RigExecBakedSlotKind::FirstFramePose)continue;
                RigExecBakedProgramImpl::ProviderRefresh::Carry carry;carry.slot=int(descendant);
                bool blocked=false;
                for(int at=int(descendant);at>=0 && at!=slot;at=B.propParent[size_t(at)]) {
                    if(B.jointSolverBinding->count(B.paths[size_t(at)]) || B.ladders[size_t(at)].parentSpaceConnected) {
                        blocked=true;break;
                    }
                    if(B.slotKind[size_t(at)]==RigExecBakedSlotKind::FirstFramePose)carry.blockingSlots.push_back(at);
                }
                if(!blocked)refresh.carries.push_back(std::move(carry));
            }
            const size_t count=refresh.carries.size();
            refresh.baseInputs.resize(count);refresh.finInputs.resize(count);
            refresh.baseOutputs.resize(count);refresh.finOutputs.resize(count);refresh.blocked.resize(count);
            B.providerRefreshBefore[checkpoint].push_back(uint32_t(B.providerRefreshes.size()));
            B.providerRefreshes.push_back(std::move(refresh));
        };
        for(int slot:requested)visit(slot);
        if(checkpoint<B.walkSteps.size() && !B.walkSteps[checkpoint].solverBatch) {
            const auto &commit=B.commits[checkpoint];
            for(size_t k=0;k<commit.slots.size();++k)
                constrained[size_t(commit.slots[k])].emplace_back(uint32_t(checkpoint),uint32_t(k));
        }
    }
}
bool RigExecBakedBindProviderRefresh(RigExecBakedProgramImpl *program,uint32_t index,
    std::vector<uint32_t> *base,std::vector<uint32_t> *fin,
    const std::function<uint32_t(bool,int)> &allocate)
{
    auto &B=*program;auto &refresh=B.providerRefreshes[index];auto &p=B.providerProgram;
    const auto context=[&](bool basePhase,RigExecValueId *result) {
        std::map<RigExecValueId,RigExecValueId> replacements;
        const auto externalCount=p.externalInputs.size();
        for(size_t k=0;k<externalCount;++k) {
            const auto input=p.externalInputs[k];
            if(input.computation!="computePointFrame")continue;
            const auto found=B.index.find(input.owner);
            if(found==B.index.end() ||
               std::find(B.poseProviderInputs[size_t(refresh.slot)].begin(),B.poseProviderInputs[size_t(refresh.slot)].end(),found->second)==
                   B.poseProviderInputs[size_t(refresh.slot)].end())continue;
            const auto value=RigExecValueId(p.valueKeys.size());
            const std::string key=refresh.key+(basePhase?":base:":":current:")+input.owner.GetString();
            p.valueKeys.push_back(key);p.valueIds.emplace(key,value);
            B.providerFrameInputs.push_back({value,found->second,basePhase,
                basePhase?(*base)[size_t(found->second)]:(*fin)[size_t(found->second)],refresh.reader});
            replacements.emplace(input.value,value);
        }
        size_t begin=0;
        return RigExecBindProviderContext(&p,p.valueKeys,B.providerRefreshTemplates[size_t(refresh.slot)],
            B.paths[size_t(refresh.slot)],refresh.key+(basePhase?":base":":current"),replacements,result,&begin,&B.providerRefreshError);
    };
    if(!context(true,&refresh.baseValue) || !context(false,&refresh.currentValue))return false;
    refresh.baseRead=(*base)[size_t(refresh.slot)];refresh.finRead=(*fin)[size_t(refresh.slot)];
    refresh.baseWrite=allocate(true,refresh.slot);refresh.finWrite=allocate(false,refresh.slot);
    (*base)[size_t(refresh.slot)]=refresh.baseWrite;(*fin)[size_t(refresh.slot)]=refresh.finWrite;
    for(auto &carry:refresh.carries) {
        const size_t slot=size_t(carry.slot);carry.baseRead=(*base)[slot];carry.finRead=(*fin)[slot];
        carry.baseWrite=allocate(true,carry.slot);carry.finWrite=allocate(false,carry.slot);
        (*base)[slot]=carry.baseWrite;(*fin)[slot]=carry.finWrite;
    }
    B.providerValues.values.resize(p.valueKeys.size());
    return true;
}
namespace {
struct RefreshMath {
    using Frame=RigExecPointFrame;using Matrix=GfMatrix4d;
    static bool Usable(const Frame &frame) {
        if(!frame.IsValid() || frame.IsDegenerate())return false;
        for(const auto &point:frame.points)for(int axis=0;axis<3;++axis)if(!std::isfinite(point[axis]))return false;
        return true;
    }
    static bool PointsToMatrix(const Frame &before,const Frame &after,Matrix *delta) {
        return RigExecPointsToMatrix(before.points,after.points,delta);
    }
    static Frame Transform(const Frame &frame,const Matrix &delta) {return RigExecMatrixToPoints(frame.points,delta);}
};
}
void RigExecBakedRunProviderRefresh(RigExecBakedProgramImpl *program,RigExecBakedStep *step)
{
    auto &B=*program;auto &r=B.providerRefreshes[size_t(step->object)];
    bool constrained=false;
    for(const auto &[index,pos]:r.priorConstraints) {
        const auto &commit=B.commits[index];
        constrained=constrained || (!commit.abandoned && commit.present[pos]);
    }
    for(size_t k=0;k<r.carries.size();++k) {
        const auto &carry=r.carries[k];r.baseInputs[k]=B.base[carry.baseRead];r.finInputs[k]=B.fin[carry.finRead];
        r.blocked[k]=0;
        for(int slot:carry.blockingSlots)if(RigExecBakedProviderParentRaw(B,slot)!=GfMatrix4d(1.0))r.blocked[k]=1;
    }
    size_t failed=0;
    const auto outcome=RigExecRefreshProviderFrames<RefreshMath>(B.base[r.baseRead],B.fin[r.finRead],
        B.providerValues.Read<RigExecPointFrame>(r.baseValue),B.providerValues.Read<RigExecPointFrame>(r.currentValue),constrained,
        r.baseInputs.data(),r.finInputs.data(),r.blocked.data(),r.carries.size(),&B.base[r.baseWrite],&B.fin[r.finWrite],
        r.baseOutputs.data(),r.finOutputs.data(),&failed);
    for(size_t k=0;k<r.carries.size();++k) {
        B.base[r.carries[k].baseWrite]=r.baseOutputs[k];B.fin[r.carries[k].finWrite]=r.finOutputs[k];
    }
    // Build spelled the paths. `failed` names a carry only on the two
    // descendant outcomes, so that text is looked up there alone.
    const auto &texts=*B.pathTexts;
    const std::string &path=texts[size_t(r.slot)];
    const auto carry=[&]() -> const std::string & {return texts[size_t(r.carries[failed].slot)];};
    if(outcome==RigExecProviderRefreshOutcome::MissingBase)
        step->diagnostics.push_back("connected base pose input incomplete: "+path);
    else if(outcome==RigExecProviderRefreshOutcome::MissingCurrent)
        step->diagnostics.push_back("connected final pose input incomplete: "+path);
    else if(outcome==RigExecProviderRefreshOutcome::InvalidCurrent)
        step->diagnostics.push_back(path+" produced an invalid or degenerate frame for "+path+"; constraint passed through");
    else if(outcome==RigExecProviderRefreshOutcome::SingularCurrent)
        step->diagnostics.push_back(path+" produced a singular hierarchy delta; constraint passed through");
    else if(outcome==RigExecProviderRefreshOutcome::InvalidDescendant)
        step->diagnostics.push_back(path+" could not propagate its pose revision through "+carry()+"; constraint passed through");
    else if(outcome==RigExecProviderRefreshOutcome::InvalidTransformedDescendant)
        step->diagnostics.push_back(path+" produced an invalid descendant frame for "+carry()+"; constraint passed through");
}
// A ladder reads its own property revision before following connections,
// while other consumers keep
// the shared expression and their independently selected connection phase.
bool RigExecBakedBindOwnPropertySpaces(RigExecBakedProgramImpl *program,
    std::string *error)
{
    auto &B=*program;
    const char *names[]={"rest:space","default:space","posed:space","parent:space",
        "parent:defaultSpace","avars:defaultSpace","posed:defaultSpace"};
    for(size_t slot=0;slot<B.ladders.size();++slot) {
        if(B.slotKind[slot]!=RigExecBakedSlotKind::FirstFramePose) continue;
        for(size_t channel=0;channel<7;++channel) {
            const SdfPath consumer=B.paths[slot].AppendProperty(TfToken(names[channel]));
            const auto chain=std::find_if(B.propertyChains.begin(),B.propertyChains.end(),
                [&](const auto &item){return item.target==consumer &&
                    item.arm==RigExecBakedPropertyChain::Arm::Matrix4d && !item.revisions.empty();});
            if(chain==B.propertyChains.end()) continue;
            const int prior=B.ladders[slot].spaceValues[channel];
            if(prior<0 || size_t(prior)>=B.providerValues.values.size()) {
                if(error) *error="own property space has no captured expression: "+consumer.GetString();
                return false;
            }
            const std::string key="provider:ownPropertyFinal:"+consumer.GetString();
            if(B.providerProgram.valueIds.count(key)) {
                if(error) *error="duplicate own property space delivery: "+consumer.GetString();
                return false;
            }
            const RigExecValueId value=RigExecValueId(B.providerProgram.valueKeys.size());
            B.providerProgram.valueKeys.push_back(key);
            B.providerProgram.valueIds.emplace(key,value);
            const auto seed=B.providerValues.values[size_t(prior)];
            B.providerValues.values.push_back(seed);
            B.providerProgram.routedInputs.push_back({value,consumer,consumer,TfToken("final")});
            B.ladders[slot].spaceValues[channel]=int(value);
        }
    }
    // A provider consumes the final revisions of its own default channels.
    // Keep each property's raw value available to the mover that produces it.
    const char *defaults[]={"default:tx","default:ty","default:tz",
        "default:rx","default:ry","default:rz"};
    for (size_t index=0; index<B.providerProgram.ops.size(); ++index) {
        auto &op=B.providerProgram.ops[index];
        if (op.kind != RigExecProviderOpKind::DefaultSpace) continue;
        for (size_t channel=0; channel<6; ++channel) {
            const SdfPath consumer=op.owner.AppendProperty(TfToken(defaults[channel]));
            const auto chain=std::find_if(B.propertyChains.begin(),B.propertyChains.end(),
                [&](const auto &item){return item.target==consumer && !item.revisions.empty();});
            if (chain==B.propertyChains.end()) continue;
            const auto prior=op.inputs[3+channel];
            const std::string key="provider:ownDefaultFinal:"+consumer.GetString();
            const auto existing=B.providerProgram.valueIds.find(key);
            if (existing!=B.providerProgram.valueIds.end()) {
                op.inputs[3+channel]=existing->second;
                continue;
            }
            const auto value=RigExecValueId(B.providerProgram.valueKeys.size());
            B.providerProgram.valueKeys.push_back(key);
            B.providerProgram.valueIds.emplace(key,value);
            const auto seed=B.providerValues.values[size_t(prior)];
            B.providerValues.values.push_back(seed);
            B.providerProgram.routedInputs.push_back({value,consumer,consumer,TfToken("final")});
            op.inputs[3+channel]=value;
        }
        auto &reads=B.providerProgram.descriptors[index].reads;
        reads.clear();
        for (auto value : op.inputs) if (value!=UINT64_MAX) reads.push_back(value);
        std::sort(reads.begin(),reads.end());
        reads.erase(std::unique(reads.begin(),reads.end()),reads.end());
    }
    return true;
}

void RigExecBakedBuildSpaceSteps(RigExecBakedProgramImpl *program)
{
    auto &B=*program;
    for(auto it=B.providerRefreshTemplateOps.rbegin();it!=B.providerRefreshTemplateOps.rend();++it) {
        B.providerProgram.ops.erase(B.providerProgram.ops.begin()+*it);
        B.providerProgram.descriptors.erase(B.providerProgram.descriptors.begin()+*it);
    }
    B.providerRefreshTemplateOps.clear();
    B.providerLeafChains.assign(B.providerProgram.sampled.size(),-1);
    for(size_t k=0;k<B.providerProgram.sampled.size();++k) {
        const auto &input=B.providerProgram.sampled[k];
        RigExecBakedStep step; step.kind=RigExecBakedStepKind::SpaceExpression;
        step.object=int(k); step.part=2;
        step.label="SpaceRaw "+input.attribute.GetString();
        step.reads.push_back(RigExecBakedOne(RigExecBakedSlotDomain::SpaceLeaf,int(k)));
        step.writes.push_back(RigExecBakedOne(RigExecBakedSlotDomain::SpaceValue,int(input.value)));
        B.steps.push_back(std::move(step));
    }
    B.providerRoutedReads.clear();
    for(size_t k=0;k<B.providerProgram.routedInputs.size();++k) {
        const auto &input=B.providerProgram.routedInputs[k];
        RigExecReadPhase phase; std::string error;
        RigExecCrossDomainRead read;
        if(!RigExecParseReadPhase(input.readPhase.GetString(),&phase,&error) ||
           !RigExecBakedBindConnectionValue(B,input.consumer,input.source,phase,&read)) {
            read.unavailable=error.empty()?"connected provider phase has no native value":error;
        }
        B.providerRoutedReads.push_back(int(B.crossDomainReads.size()));
        B.crossDomainReads.push_back(std::move(read));
        RigExecBakedStep step; step.kind=RigExecBakedStepKind::SpaceExpression;
        step.object=int(k); step.part=3;
        step.label="SpaceConnection "+B.providerProgram.valueKeys[size_t(input.value)];
        step.writes.push_back(RigExecBakedOne(RigExecBakedSlotDomain::SpaceValue,int(input.value)));
        B.steps.push_back(std::move(step));
    }
    for(size_t k=0;k<B.providerProgram.externalInputs.size();++k) {
        const auto &input=B.providerProgram.externalInputs[k];
        RigExecBakedStep step; step.kind=RigExecBakedStepKind::SpaceExpression;
        step.object=int(k); step.part=1;
        step.label="SpaceInput "+B.providerProgram.valueKeys[size_t(input.value)];
        const int slot=B.providerExternalSlots[k];
        if(slot>=0 && input.computation=="computeRestFrame")
            step.reads.push_back(RigExecBakedOne(RigExecBakedSlotDomain::Rest,slot));
        else if(slot>=0 && (input.computation=="computePointFrame" || input.computation=="computeBasePointFrame"))
            step.reads.push_back(RigExecBakedOne(RigExecBakedSlotDomain::PoseBase,
                uint32_t(slot)));
        else if(slot>=0) {
            const auto &leaf=B.ladders[size_t(slot)].interveningSpace;
            if(leaf.leaf>=0) step.bindingLeaves.push_back(B.leaves.Of<GfMatrix4d>().id[size_t(leaf.leaf)]);
            step.varyingLeaves=leaf.varying;
        }
        step.writes.push_back(RigExecBakedOne(RigExecBakedSlotDomain::SpaceValue,int(input.value)));
        B.steps.push_back(std::move(step));
    }
    for(size_t k=0;k<B.providerProgram.ops.size();++k) {
        const auto &descriptor=B.providerProgram.descriptors[k];
        RigExecBakedStep step; step.kind=RigExecBakedStepKind::SpaceExpression;
        step.object=int(k); step.part=0; step.label=descriptor.key;
        for(auto value:descriptor.reads) step.reads.push_back(RigExecBakedOne(RigExecBakedSlotDomain::SpaceValue,int(value)));
        for(auto value:descriptor.writes) step.writes.push_back(RigExecBakedOne(RigExecBakedSlotDomain::SpaceValue,int(value)));
        B.steps.push_back(std::move(step));
    }
    for(size_t k=0;k<B.providerFrameInputs.size();++k) {
        const auto &input=B.providerFrameInputs[k];
        RigExecBakedStep step;step.kind=RigExecBakedStepKind::SpaceExpression;step.object=int(k);step.part=4;
        step.reads.push_back(RigExecBakedOne(input.base?RigExecBakedSlotDomain::PoseBase:RigExecBakedSlotDomain::PoseFin,input.version));
        step.writes.push_back(RigExecBakedOne(RigExecBakedSlotDomain::SpaceValue,input.value));
        B.steps.push_back(std::move(step));
    }
}
void RigExecBakedRunSpaceOp(RigExecBakedProgramImpl *program,RigExecBakedStep *step)
{
    auto &B=*program;
    if(step->part==4) {
        const auto &input=B.providerFrameInputs[size_t(step->object)];
        B.providerValues.Publish(input.value,input.base?B.base[input.version]:B.fin[input.version]);
        return;
    }
    if(step->part==2) {
        const size_t k=size_t(step->object);
        const auto &input=B.providerProgram.sampled[k];
        const auto *overlay=RigExecBakedSpaceLeafOverlay(B,k);
        const VtValue &value=overlay?*overlay:B.providerLeaves.values[k];
        const bool authoritative=overlay!=nullptr;
        B.providerValues.PublishSource(input.value,value,
            !authoritative && B.providerLeafBlocked[k],authoritative);
        return;
    }
    if(step->part==3) {
        const size_t k=size_t(step->object);
        VtValue value; std::string diagnostic;
        const bool available=RigExecBakedReadCrossDomain(B,B.providerRoutedReads[k],&value,&diagnostic);
        B.providerValues.PublishSource(B.providerProgram.routedInputs[k].value,value,!available,true);
        if(!diagnostic.empty()) step->diagnostics.push_back(diagnostic);
        return;
    }
    if(step->part==1) {
        const size_t k=size_t(step->object);
        const auto &input=B.providerProgram.externalInputs[k];
        const int slot=B.providerExternalSlots[k];
        if(input.computation=="computeRestFrame")
            B.providerValues.Publish(input.value,slot>=0?B.restFrames[size_t(slot)]:RigExecFrameFromMatrix(GfMatrix4d(1.0)));
        else if(input.computation=="computePointFrame" || input.computation=="computeBasePointFrame")
            B.providerValues.Publish(input.value,slot>=0?B.base[
                size_t(slot)] :
                RigExecFrameFromMatrix(GfMatrix4d(1.0)));
        else B.providerValues.Publish(input.value,slot>=0?RigExecBakedLeafRead(B,B.ladders[size_t(slot)].interveningSpace):GfMatrix4d(1.0));
        return;
    }
    std::string error;
    if(!RigExecRunProviderOp(B.providerProgram,uint32_t(step->object),&B.providerValues,&error)) {
        const auto output=B.providerProgram.ops[size_t(step->object)].output;
        B.providerValues.Publish(output,std::monostate(),true,false,0,error);
        B.providerValues.values[size_t(output)].raw=VtValue();
        if(!error.empty()) step->diagnostics.push_back(error);
    }
}
}
