#include "bakedOpGraph.h"
#include "bakedOpValues.h"
#include "bakedSchedule.h"
#include "weightField.h"
#include "scalarReferenceAdapter.h"
#include "parallel.h"
#include "profiler.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/base/work/dispatcher.h"
#include "pxr/base/work/withScopedParallelism.h"
#include <algorithm>
#include <cstring>
#include <limits>
#include <map>
#include <memory>
#include <tuple>
#include <type_traits>

namespace rigExec {
namespace {
bool IsShadowed(const RigExecBakedProgramImpl &B,const RigExecBakedStep &step,uint32_t v)
{
    bool found=false;
    for(const auto &pair:step.shadowedReads) if(pair.first==v) {
        if(pair.second<B.recordStoodAside.size() && B.recordStoodAside[pair.second]) return false;
        found=true;
    }
    return found;
}

void ResetExcludedValue(RigExecBakedProgramImpl *program,uint32_t domain,uint32_t slot)
{
    auto &B=*program; using D=RigExecBakedSlotDomain;
    RigExecPointFrame invalidFrame; invalidFrame.flags=0;
    GfMatrix4d invalidMatrix(1);
    invalidMatrix[3][0]=std::numeric_limits<double>::quiet_NaN();
    switch(D(domain)) {
    case D::Avars:
        std::fill(B.avars.begin()+size_t(slot)*11,B.avars.begin()+(size_t(slot)+1)*11,0.0); break;
    case D::PoseBase: B.base[slot]=invalidFrame; break;
    case D::PoseFin: B.fin[slot]=invalidFrame; break;
    case D::PosedM: B.posedM[slot]=invalidMatrix; break;
    case D::FinalMatrix: B.finalMatrix[slot]=invalidMatrix; break;
    case D::BaseMatrix: B.baseMatrix[slot]=invalidMatrix; break;
    case D::SwitchFrame: B.switchFrames[slot]=invalidMatrix; break;
    case D::Aggregate: B.aggregates[slot]={}; break;
    case D::Candidates:
        std::fill(B.solvers[slot].outPresent.begin(),B.solvers[slot].outPresent.end(),char(0));
        std::fill(B.solvers[slot].outFrames.begin(),B.solvers[slot].outFrames.end(),invalidFrame); break;
    case D::CommitTable: {
        auto &commit=B.commits[slot]; commit.abandoned=true;
        std::fill(commit.present.begin(),commit.present.end(),char(0));
        std::fill(commit.frames.begin(),commit.frames.end(),invalidFrame); break;
    }
    case D::CommitDelta:
        std::fill(B.commits[slot].deltaOk.begin(),B.commits[slot].deltaOk.end(),char(0)); break;
    case D::CommitStaging:
        for(auto &commit:B.commits) if(commit.split && slot>=uint32_t(commit.stagingBase) &&
            uint64_t(slot)-uint32_t(commit.stagingBase)<commit.staged.size()) {
            const auto i=slot-uint32_t(commit.stagingBase); commit.staged[i]=invalidFrame;
            commit.outcome[i]=uint8_t(RigExecBakedPropagateOutcome::NoCandidate); break;
        } break;
    case D::ConstraintDelta: B.deltaPresent[slot]=0; B.deltaValues[slot]=GfMatrix4d(1); break;
    case D::PropertyResult:
        B.propertyVersionValid[slot]=0; B.propertyValues[slot]={};
        if(slot<B.propertyRecordById.size() && B.propertyRecordById[slot]>=0) {
            const size_t record=size_t(B.propertyRecordById[slot]);
            B.recordValues[record]=VtValue(); B.recordStoodAside[record]=1;
        }
        break;
    case D::WeightPacket: B.weightPackets[slot]={}; break;
    case D::WeightFrames: B.volumePlacement[slot]=invalidMatrix; break;
    case D::WeightFramesBase: B.volumePlacementBase[slot]=invalidMatrix; break;
    case D::PoseWeight: B.poseWeights[slot]=0; break;
    case D::FrameMatrix: B.frameMatrixValid[slot]=0; B.frameMatrix[slot]=GfMatrix4d(1); break;
    case D::Rest: B.restFrames[slot]=invalidFrame; B.restM[slot]=GfMatrix4d(1); B.restPts[slot]={}; break;
    case D::Ladder:
        B.selfD[slot]=B.parentDinv[slot]=B.restRoundTrip[slot]=B.defaultRoundTrip[slot]=GfMatrix4d(1);
        B.posedAuthored[slot]=B.parentSpaceAuthored[slot]=0;
        B.posedAuthoredM[slot]=B.posedD[slot]=B.parentSpaceM[slot]=GfMatrix4d(1);
        B.rotOrder[slot]=TfToken();
        B.rotationSign[slot]=0;
        break;
    case D::WeightField: RigExecBakedResetWeightField(&B,int(slot)); break;
    case D::SpaceValue: {
        auto &value=B.providerValues.values[slot]; value.value=std::monostate();
        value.initialized=false; value.authoritative=false; value.blocked=true;
        value.count=0; value.error="operation cycle"; break;
    }
    default: RigExecBakedResetSetAsideGeometryValue(&B,D(domain),slot); break;
    }
}
}

bool RigExecBakedEffectiveMemo(const RigExecBakedProgramImpl &B,uint32_t c,
    std::string *key,std::vector<uint32_t> *property,
    std::vector<std::pair<uint32_t,uint32_t>> *typed,
    const RigExecBakedOpIdentityRemap *remap,bool contentLeaves)
{
    if(c>=B.opGraph.ops.size()) return false;
    const auto &step=B.steps[B.opGraph.ops[c].originalIndex];
    auto &scratch=*key;
    auto &covered=*property; covered.clear();
    auto &coveredTyped=*typed; coveredTyped.clear();
    const bool projectedConsumer=step.kind==RigExecBakedStepKind::PropertyRevision ||
        step.kind==RigExecBakedStepKind::AvarInputs ||
        step.kind==RigExecBakedStepKind::ComposeSubtree;
    const bool exact=RigExecBakedOpEffectiveInputKey(B,step,&scratch,&covered,
        projectedConsumer?&coveredTyped:nullptr,remap,contentLeaves);
    std::sort(covered.begin(),covered.end());
    std::sort(coveredTyped.begin(),coveredTyped.end());
    const auto *reads=&B.opGraph.ops[c].descriptor.reads;
    std::vector<RigExecValueId> remappedReads;
    if(remap) {
        for(auto id:*reads) if(id>=remap->values.size() || remap->values[size_t(id)]<0) return false;
        remappedReads=*reads;
        std::sort(remappedReads.begin(),remappedReads.end(),[&](auto a,auto b) {
            return remap->values[size_t(a)]<remap->values[size_t(b)];
        });
        reads=&remappedReads;
    }
    for(auto id:*reads) {
        const auto &value=B.opAdapter.values[size_t(id)];
        if(exact && step.kind==RigExecBakedStepKind::WeightField) continue;
        if(exact && step.kind==RigExecBakedStepKind::ProviderRefresh && value.domain==uint32_t(RigExecBakedSlotDomain::SpaceLeaf))continue;
        const bool incoming=step.kind==RigExecBakedStepKind::PropertyRevision && step.part>0 &&
        value.domain==uint32_t(RigExecBakedSlotDomain::PropertyResult) &&
        value.slot==B.propertyChains[size_t(step.object)].versionBase+uint32_t(step.part-1);
        if(exact && !incoming && std::binary_search(coveredTyped.begin(),coveredTyped.end(),
        std::make_pair(value.domain,value.slot))) continue;
        if(exact && step.kind==RigExecBakedStepKind::SpaceExpression &&
           ((step.part==0 && value.domain==uint32_t(RigExecBakedSlotDomain::SpaceValue)) ||
        (step.part==2 && value.domain==uint32_t(RigExecBakedSlotDomain::SpaceLeaf)))) continue;
        if(!incoming && value.domain==uint32_t(RigExecBakedSlotDomain::PropertyResult) &&
           (std::binary_search(covered.begin(),covered.end(),value.slot) ||
        IsShadowed(B,step,value.slot))) continue;
        RigExecOpKeyAppend(&scratch,remap?RigExecValueId(remap->values[size_t(id)]):id);
        RigExecOpKeyAppend(&scratch,value.revision);
    }
    return exact;
}

namespace {
std::string SkinValueIdentity(const RigExecBakedProgramImpl &B,const RigExecOpValueState &v)
{
    using D=RigExecBakedSlotDomain;
    std::string result=std::to_string(v.domain)+":";
    switch(D(v.domain)) {
    case D::RequiredStageFramesAdmission:
        if (v.slot != 0) return {};
        result += "request";
        for (const int slot : B.xformSlots) {
            if (slot < 0 || size_t(slot) >= B.paths.size()) return {};
            const auto path = B.paths[size_t(slot)].GetString();
            result += ":" + std::to_string(path.size()) + ":" + path;
        }
        return result;
    case D::PoseBase: case D::PoseFin: case D::PosedM:
    case D::FinalMatrix: case D::BaseMatrix:
        return v.slot<B.paths.size()?result+B.paths[v.slot].GetString():std::string();
    case D::ChainBase: case D::ChainPoints: case D::ChainInput:
        return v.slot<B.chains.size()?result+B.chains[v.slot].target.GetString():std::string();
    case D::SkinTopology: case D::RevisionPacket: case D::RevisionTransforms:
    case D::RevisionDone: case D::ChainDirty:
        if(v.slot<B.revisionIndex.size()) {
            const auto &index=B.revisionIndex[v.slot];
            const auto &chain=B.chains[size_t(index.first)];
            const auto &revision=chain.revisions[size_t(index.second)];
            return result+chain.target.GetString()+":"+revision.moverPath.GetString()+":"+std::to_string(index.second);
        }
        return {};
    case D::RevisionOut:
        for(size_t r=0;r<B.revisionChunkBase.size();++r) {
            const int base=B.revisionChunkBase[r],count=B.revisionChunkCount[r];
            if(base<0 || count<0 || v.slot<uint32_t(base) || uint64_t(v.slot)-uint32_t(base)>=uint32_t(count))continue;
            const auto &index=B.revisionIndex[r];const auto &chain=B.chains[size_t(index.first)];
            const auto &revision=chain.revisions[size_t(index.second)];
            const auto &chunk=revision.chunks[size_t(v.slot-uint32_t(base))];
            return result+chain.target.GetString()+":"+revision.moverPath.GetString()+":"+
                std::to_string(index.second)+":"+std::to_string(chunk.begin)+":"+std::to_string(chunk.end);
        }
        return {};
    default:return {};
    }
}

bool SameSimpleWalk(const RigExecBakedReaderWalk &a,const RigExecBakedReaderWalk &b,
    const RigExecBakedOpIdentityRemap &map)
{
    if(a.head!=b.head || a.walk.type!=b.walk.type || a.walk.flavour!=b.walk.flavour ||
       !a.versions.empty() || !b.versions.empty() || !a.shadowed.empty() || !b.shadowed.empty())return false;
    const auto mapped=[](int a,int b,const std::vector<int> &m) {
        return a<0?b==a:size_t(a)<m.size() && m[size_t(a)]==b;
    };
    if(!mapped(a.rawLeaf,b.rawLeaf,map.headLeaves))return false;
    const auto hops=[&](const auto &x,const auto &y) {
        if(x.size()!=y.size())return false;
        for(size_t i=0;i<x.size();++i) {
            if(x[i].path!=y[i].path || x[i].chain>=0 || y[i].chain>=0 ||
               x[i].record>=0 || y[i].record>=0 || x[i].poseWeight>=0 || y[i].poseWeight>=0 ||
               x[i].crossDomain>=0 || y[i].crossDomain>=0 ||
               !mapped(x[i].leaf,y[i].leaf,map.headLeaves) ||
               !mapped(x[i].overrideSlot,y[i].overrideSlot,map.overrides))return false;
        }
        return true;
    };
    const auto list=[&](const auto &x,const auto &y,const std::vector<int> &m) {
        if(x.size()!=y.size())return false;
        std::vector<uint32_t> mappedIds;
        for(auto id:x) {if(id>=m.size() || m[id]<0)return false;mappedIds.push_back(uint32_t(m[id]));}
        std::sort(mappedIds.begin(),mappedIds.end());return mappedIds==y;
    };
    return hops(a.walk.hops,b.walk.hops) && hops(a.walk.doubleHops,b.walk.doubleHops) &&
        list(a.leaves,b.leaves,map.headLeaves) && list(a.slots,b.slots,map.overrides);
}

bool SkinSourceMap(const RigExecBakedProgramImpl &B,const RigExecBakedProgramImpl &P,
    RigExecBakedOpIdentityRemap *map)
{
    map->headLeaves.assign(P.headLeaves.size(),-1);
    std::map<std::tuple<SdfPath,RigExecBakedHeadValueType,SdfPath>,int> heads;
    for(size_t j=0;j<B.headLeaves.size();++j) {
        const auto &b=B.headLeaves[j];
        if(!heads.emplace(std::make_tuple(b.path,b.type,b.frozenKey),int(j)).second)return false;
    }
    for(size_t i=0;i<P.headLeaves.size();++i) {
        const auto &a=P.headLeaves[i];const auto found=heads.find(std::make_tuple(a.path,a.type,a.frozenKey));
        if(found!=heads.end())map->headLeaves[i]=found->second;
    }
    map->overrides.assign(P.headOverrides.size(),-1);
    for(const auto &entry:P.headOverrideSlots) {
        const auto found=B.headOverrideSlots.find(entry.first);
        if(entry.second<map->overrides.size() && found!=B.headOverrideSlots.end())
            map->overrides[entry.second]=int(found->second);
    }
    map->readerWalks.assign(P.readerWalks.size(),-1);
    std::map<std::pair<SdfPath,RigExecBakedHeadValueType>,std::vector<int>> walks;
    for(size_t j=0;j<B.readerWalks.size();++j)
        walks[{B.readerWalks[j].head,B.readerWalks[j].walk.type}].push_back(int(j));
    for(size_t i=0;i<P.readerWalks.size();++i) {
        int found=-1;
        const auto candidates=walks.find({P.readerWalks[i].head,P.readerWalks[i].walk.type});
        if(candidates==walks.end())continue;
        for(int j:candidates->second)
            if(SameSimpleWalk(P.readerWalks[i],B.readerWalks[size_t(j)],*map)) {
                if(found>=0){found=-1;break;}found=int(j);
            }
        map->readerWalks[i]=found;
    }
    // Initial admission covers path declarations only. Ordinary typed pool
    // references require additional binding provenance and stay cold.
    map->bindingLeaves.assign(P.leafRefs.size()+P.pathLeafRefs.size()+P.headLeaves.size(),-1);
    for(size_t i=0;i<P.headLeaves.size();++i)if(map->headLeaves[i]>=0)
        map->bindingLeaves[P.leafRefs.size()+P.pathLeafRefs.size()+i]=
            int(B.leafRefs.size()+B.pathLeafRefs.size())+map->headLeaves[i];
    map->bindingRefIndices.assign(P.leafRefs.size(),-1);
    using PathKey=std::tuple<int,int,int,int,SdfPath>;
    const auto pathKey=[](const auto &ref,const auto &key) {
        return PathKey(int(ref.owner),int(key.type),int(key.time),int(key.flavour),key.path);
    };
    std::map<PathKey,std::vector<int>> pathLeaves;
    for(size_t j=0;j<B.pathLeafRefs.size();++j) {
        const auto &ref=B.pathLeafRefs[j];const auto *leaves=RigExecBakedPathLeavesOf(B,ref);
        if(leaves && ref.key<leaves->decl.keys.size())
            pathLeaves[pathKey(ref,leaves->decl.keys[ref.key])].push_back(int(j));
    }
    for(size_t i=0;i<P.pathLeafRefs.size();++i) {
        const auto &ref=P.pathLeafRefs[i];const auto *a=RigExecBakedPathLeavesOf(P,ref);
        if(!a || ref.key>=a->decl.keys.size())continue;
        int found=-1;
        const auto candidates=pathLeaves.find(pathKey(ref,a->decl.keys[ref.key]));
        if(candidates==pathLeaves.end())continue;
        for(int j:candidates->second) {
            const auto &other=B.pathLeafRefs[size_t(j)];const auto *b=RigExecBakedPathLeavesOf(B,other);
            if(!b || other.key>=b->decl.keys.size())continue;
            const auto &x=a->decl.keys[ref.key],&y=b->decl.keys[other.key];
            if(ref.owner==other.owner && x.path==y.path && x.type==y.type &&
               x.time==y.time && x.flavour==y.flavour &&
               RigExecExactSourceValueEqual(x.fallback,y.fallback) &&
               a->hops[ref.key]==b->hops[other.key]) {
                const int aw=ref.key<a->walks.size()?a->walks[ref.key]:-1;
                const int bw=other.key<b->walks.size()?b->walks[other.key]:-1;
                if(aw<0?bw>=0:size_t(aw)>=map->readerWalks.size() || map->readerWalks[size_t(aw)]!=bw)continue;
                if(found>=0){found=-1;break;}found=int(B.leafRefs.size()+j);
            }
        }
        map->bindingLeaves[P.leafRefs.size()+i]=found;
    }
    return true;
}

bool SameSkinShape(const RigExecBakedProgramImpl &B,const RigExecBakedProgramImpl &P,
    size_t r,size_t old)
{
    if(r>=B.revisionIndex.size() || old>=P.revisionIndex.size())return false;
    const auto &index=B.revisionIndex[r],&prior=P.revisionIndex[old];
    const auto &chain=B.chains[size_t(index.first)],&oldChain=P.chains[size_t(prior.first)];
    const auto &a=chain.revisions[size_t(index.second)],&b=oldChain.revisions[size_t(prior.second)];
    if(index.second!=prior.second || chain.target!=oldChain.target || a.op!=RigExecRevisionOp::Skin ||
       b.op!=a.op || !(a.binding==b.binding) || B.useSimd!=P.useSimd ||
       a.weightObject>=0 || b.weightObject>=0 || !a.pointBindings.empty() || !b.pointBindings.empty() ||
       !a.transformRecords.empty() || !b.transformRecords.empty() || a.skinTopologyFixed!=b.skinTopologyFixed ||
       a.chunked || b.chunked || a.chunks.size()!=b.chunks.size() || a.influenceSlots.size()!=b.influenceSlots.size() ||
       a.leaves.decl.roles!=b.leaves.decl.roles || a.leaves.decl.keys.size()!=b.leaves.decl.keys.size() ||
       !b.ran || b.created || !b.parameters.valid || !b.parameters.enabled ||
       !b.status.AllowsApply() || b.resultStatus!=b.status.state)return false;
    for(const auto &chunk:b.chunks)if(!chunk.ok)return false;
    for(size_t i=0;i<=size_t(index.second);++i)
        if(chain.revisions[i].moverPath!=oldChain.revisions[i].moverPath ||
           chain.revisions[i].op!=oldChain.revisions[i].op ||
           !(chain.revisions[i].binding==oldChain.revisions[i].binding))return false;
    for(size_t i=0;i<a.influenceSlots.size();++i) {
        const int x=a.influenceSlots[i],y=b.influenceSlots[i];
        if(x<0 || y<0 || size_t(x)>=B.paths.size() || size_t(y)>=P.paths.size() ||
           B.paths[size_t(x)]!=P.paths[size_t(y)] ||
           (i<a.influenceRecords.size() && !a.influenceRecords[i].empty()) ||
           (i<b.influenceRecords.size() && !b.influenceRecords[i].empty()))return false;
    }
    for(size_t i=0;i<a.chunks.size();++i)
        if(a.chunks[i].begin!=b.chunks[i].begin || a.chunks[i].end!=b.chunks[i].end || a.chunks[i].key!=b.chunks[i].key)return false;
    return true;
}

void CopySkinBodyOutputs(RigExecBakedProgramImpl::GeomRevision *a,
    const RigExecBakedProgramImpl::GeomRevision &b)
{
    // Only the three retained bodies' outputs. Cold Fold and SkinTopology
    // recompute their own table/layout opinions against the new bindings.
    a->parameters=b.parameters;a->status=b.status;a->precedingCount=b.precedingCount;
    a->defaultWeight=b.defaultWeight;a->lastDefaultWeight=b.lastDefaultWeight;
    a->layoutUsable=b.layoutUsable;a->envelope=b.envelope;a->envelopeOk=b.envelopeOk;
    a->fullStrength=b.fullStrength;a->partitionStale=b.partitionStale;
    a->currentPhasePacket=b.currentPhasePacket;a->publishedWeightValues=b.publishedWeightValues;
    a->weightFieldPublished=b.weightFieldPublished;a->topology=b.topology;a->topologyResolved=b.topologyResolved;
    a->stagingOutput=b.stagingOutput;a->output=b.output;a->resultStatus=b.resultStatus;
    a->currentSource=b.currentSource;a->ran=b.ran;a->lastStatus=b.lastStatus;
    // The buffers' roles and the published points' version, with the
    // baseline that version is next decided against.
    a->stagingFresh=b.stagingFresh;a->passedPoints=b.passedPoints;a->doneVersion=b.doneVersion;
    a->staticDirty=false;a->executed=false;
    for(size_t i=0;i<a->chunks.size();++i) {
        a->chunks[i].ok=b.chunks[i].ok;a->chunks[i].transforms=b.chunks[i].transforms;
        a->chunks[i].rows=b.chunks[i].rows;a->chunks[i].palette=b.chunks[i].palette;
        a->chunks[i].keyChanged=false;
    }
}
}

void RigExecBakedAdoptSkinOpState(RigExecBakedProgramImpl *program,const RigExecBakedProgramImpl &P)
{
    auto &B=*program;
    if(!P.everRan || !P.opAdapter.everRan || !P.opAdapter.compiled || !B.opAdapter.compiled)return;
    RigExecBakedOpIdentityRemap map;
    if(!SkinSourceMap(B,P,&map))return;
    std::map<std::string,int> newValues;
    for(size_t i=0;i<B.opAdapter.values.size();++i) {
        const auto key=SkinValueIdentity(B,B.opAdapter.values[i]);
        if(!key.empty() && !newValues.emplace(key,int(i)).second)return;
    }
    map.values.assign(P.opAdapter.values.size(),-1);
    for(size_t i=0;i<P.opAdapter.values.size();++i) {
        const auto found=newValues.find(SkinValueIdentity(P,P.opAdapter.values[i]));
        if(found!=newValues.end())map.values[i]=found->second;
    }
    std::map<std::string,uint32_t> oldOps;
    for(uint32_t i=0;i<P.opGraph.ops.size();++i)oldOps.emplace(P.opGraph.ops[i].descriptor.key,i);
    B.opAdapter.retainedFirst.assign(B.opGraph.ops.size(),0);
    const auto mappedList=[](const auto &old,const auto &current,const std::vector<int> &m,bool sorted) {
        std::decay_t<decltype(current)> values;for(auto id:old) {
            if(id>=m.size() || m[id]<0)return false;values.push_back(typename std::decay_t<decltype(current)>::value_type(m[id]));
        }
        if(sorted)std::sort(values.begin(),values.end());return values==current;
    };
    for(uint32_t c=0;c<B.opGraph.ops.size();++c) {
        const auto &op=B.opGraph.ops[c];const auto &step=B.steps[op.originalIndex];
        if(step.kind!=RigExecBakedStepKind::RevisionStatic && step.kind!=RigExecBakedStepKind::RevisionChunk &&
           step.kind!=RigExecBakedStepKind::RevisionFuse)continue;
        const auto found=oldOps.find(op.descriptor.key);if(found==oldOps.end())continue;
        const uint32_t old=found->second;const auto &prior=P.opGraph.ops[old];const auto &oldStep=P.steps[prior.originalIndex];
        if(op.descriptor.kind!=prior.descriptor.kind || op.descriptor.volatileInput || prior.descriptor.volatileInput ||
           step.alwaysRuns || oldStep.alwaysRuns || !oldStep.diagnostics.empty() ||
           step.maxDiagnostics!=oldStep.maxDiagnostics || step.object<0 || oldStep.object<0 ||
           !SameSkinShape(B,P,size_t(step.object),size_t(oldStep.object)) ||
           !mappedList(prior.descriptor.reads,op.descriptor.reads,map.values,true) ||
           !mappedList(prior.descriptor.writes,op.descriptor.writes,map.values,true) ||
           !mappedList(oldStep.bindingLeaves,step.bindingLeaves,map.bindingLeaves,false) ||
           !mappedList(oldStep.leaves,step.leaves,map.headLeaves,false) ||
           !mappedList(oldStep.overrideSlots,step.overrideSlots,map.overrides,false) ||
           !mappedList(oldStep.readerWalks,step.readerWalks,map.readerWalks,false) ||
           old>=P.opAdapter.inputExact.size() || !P.opAdapter.inputExact[old])continue;
        std::vector<std::string> beforePreds,afterPreds;
        for(auto i:prior.predecessors)beforePreds.push_back(P.opGraph.ops[i].descriptor.key);
        for(auto i:op.predecessors)afterPreds.push_back(B.opGraph.ops[i].descriptor.key);
        std::sort(beforePreds.begin(),beforePreds.end());std::sort(afterPreds.begin(),afterPreds.end());
        if(beforePreds!=afterPreds)continue;
        bool exact=true;
        for(const auto *ids:{&prior.descriptor.reads,&prior.descriptor.writes})for(auto id:*ids) {
            const auto &value=P.opAdapter.values[size_t(id)];
            if(!value.initialized || !RigExecBakedOpValueKeyIsExact(P,RigExecBakedSlotDomain(value.domain),value.slot)) {
                exact=false;continue;
            }
            std::string actual;
            RigExecBakedOpValueKey(P,RigExecBakedSlotDomain(value.domain),value.slot,&actual);
            if(actual!=value.key)exact=false;
        }
        if(!exact)continue;
        std::string ordinary,remapped,source;std::vector<uint32_t> covered;
        std::vector<std::pair<uint32_t,uint32_t>> typed;
        if(!RigExecBakedEffectiveMemo(P,old,&ordinary,&covered,&typed) ||
           old>=P.opAdapter.inputKeys.size() || ordinary!=P.opAdapter.inputKeys[old] ||
           !RigExecBakedEffectiveMemo(P,old,&remapped,&covered,&typed,&map) ||
           !RigExecBakedOpInputKey(P,oldStep,&source,&map))continue;
        const auto &index=B.revisionIndex[size_t(step.object)];const auto &oldIndex=P.revisionIndex[size_t(oldStep.object)];
        auto &revision=B.chains[size_t(index.first)].revisions[size_t(index.second)];
        const auto &oldRevision=P.chains[size_t(oldIndex.first)].revisions[size_t(oldIndex.second)];
        CopySkinBodyOutputs(&revision,oldRevision);
        for(const auto *ids:{&prior.descriptor.reads,&prior.descriptor.writes})for(auto id:*ids) {
            const size_t mapped=size_t(map.values[size_t(id)]);
            auto &destination=B.opAdapter.values[mapped];
            const auto &previous=P.opAdapter.values[size_t(id)];
            destination.key=previous.key;destination.revision=previous.revision;
            destination.initialized=true;destination.changed=0;
            if(mapped<B.chainContentKeys.size() && size_t(id)<P.chainContentKeys.size())
                B.chainContentKeys[mapped]=P.chainContentKeys[size_t(id)];
        }
        // MarkSkipped preserves program-size facts on the retained body.
        auto &retainedStep=B.steps[op.originalIndex];
        retainedStep.counters.chainsBuilt=oldStep.counters.chainsBuilt;
        retainedStep.counters.revisionsBuilt=oldStep.counters.revisionsBuilt;
        B.opAdapter.inputKeys[c]=std::move(remapped);B.opAdapter.sourceKeys[c]=std::move(source);
        B.opAdapter.inputExact[c]=1;B.opAdapter.retainedFirst[c]=1;
    }
}

namespace {
// Indexes the provider leaves for sparse publication, from the compiled
// leaves and the Build-only override tables; owner thread, once per compile.
void BuildSpaceLeafIndex(RigExecBakedProgramImpl *program)
{
    auto &B=*program; const auto &state=B.opAdapter;
    B.spaceLeafIndex.reset(); B.spaceLeafRekey.clear(); B.spaceLeafKeys=0;
    const size_t count=B.providerProgram.sampled.size();
    if(!count || B.providerLeaves.values.size()!=count || B.providerLeafBlocked.size()!=count) return;
    auto index=std::make_shared<RigExecBakedSpaceLeafIndex>();
    size_t found=0;
    for(const auto id:state.leaves) {
        const auto &value=state.values[size_t(id)];
        if(value.domain!=uint32_t(RigExecBakedSlotDomain::SpaceLeaf)) {
            (found?index->after:index->before).push_back(id); continue;
        }
        if(!found) index->first=id;
        // Leaf k must be value first + k for publication to walk them in id order.
        if(value.slot!=found || id!=index->first+found) return;
        ++found;
    }
    if(found!=count) return;
    index->headSlot.assign(count,-1); index->numberBegin.assign(count+1,0);
    for(size_t k=0;k<count;++k) {
        const SdfPath &path=B.providerProgram.sampled[k].attribute;
        index->byPath.emplace_back(path,uint32_t(k));
        const auto head=B.headOverrideSlots.find(path);
        if(head!=B.headOverrideSlots.end()) {
            if(head->second>uint32_t(std::numeric_limits<int>::max())) return;
            index->headSlot[k]=int(head->second);
            index->byHeadSlot.emplace_back(head->second,uint32_t(k));
        }
        const auto numbers=B.overridableInputs.find(path);
        if(numbers!=B.overridableInputs.end()) for(const int number:numbers->second) {
            index->numbers.push_back(number);
            if(number>=0) index->byNumber.emplace_back(uint32_t(number),uint32_t(k));
        }
        index->numberBegin[k+1]=uint32_t(index->numbers.size());
    }
    std::sort(index->byPath.begin(),index->byPath.end());
    index->verify=TfGetenvBool("RIGEXEC_VERIFY_SPARSE_LEAVES",false);
    B.spaceLeafIndex=std::move(index);
    B.spaceLeafRekey.assign(count,0);
}

// RIGEXEC_VERIFY_CHAIN_VERSIONS, owner thread, after \p id published: a value
// keyed by a point content version must tell the change the points' bytes
// tell. The first publication a program sees has nothing to compare with.
void VerifyChainVersion(RigExecBakedProgramImpl *program,RigExecValueId id)
{
    auto &B=*program;
    if(size_t(id)>=B.chainContentKeys.size()) return;
    const auto &value=B.opAdapter.values[size_t(id)];
    std::string content;
    if(!RigExecBakedChainContentKey(B,RigExecBakedSlotDomain(value.domain),value.slot,&content)) return;
    auto &last=B.chainContentKeys[size_t(id)];
    if(!last.empty() && (last!=content)!=(value.changed!=0)) {
        ++B.chainVersionMismatches;
        TF_VERIFY(false,"domain %u slot %u: the point content version and the "
                  "points' bytes disagree on a change",value.domain,value.slot);
    }
    last.swap(content);
}

// Re-keys the sampled leaves into changedLeaves, in id order. Unless \p all,
// a provider leaf is re-keyed only for a kRigExecSpaceLeaf* reason; any other
// keeps the key an equal compare would keep, unchanged. Owner thread, before
// the region.
void PublishLeaves(RigExecBakedProgramImpl *program,bool all)
{
    auto &B=*program; auto &state=B.opAdapter;
    // A chain input keys its sampled base by a content version, moved here,
    // before it publishes, exactly when the bytes differ from the base it
    // last published (held by handle, so immutable).
    for(auto &chain:B.chains)
        if(!RigExecBakedSamePoints(chain.sampledBase.cdata(),chain.sampledBase.size(),
                                   chain.publishedInput.cdata(),chain.publishedInput.size())) {
            ++chain.inputVersion; chain.publishedInput=chain.sampledBase;
        }
    const auto publish=[&](RigExecValueId id) {
        auto &value=state.values[size_t(id)];
        RigExecOpPublishValue(&value,[&](uint32_t d,uint32_t slot,std::string *key) {
            RigExecBakedOpValueKey(B,RigExecBakedSlotDomain(d),slot,key);
            // An unsupported sampled source has no equality proof.
            if(!RigExecBakedOpValueKeyIsExact(B,RigExecBakedSlotDomain(d),slot))
                RigExecOpKeyAppend(key,value.revision+1);
        });
        if(value.changed) state.changedLeaves.push_back(id);
        if(B.verifyChainVersions) VerifyChainVersion(&B,id);
    };
    const auto *index=B.spaceLeafIndex.get();
    auto &rekey=B.spaceLeafRekey;
    if(!index || rekey.size()+1!=index->numberBegin.size()) {
        for(const auto id:state.leaves) publish(id);
        return;
    }
    // Every leaf an overlay can stand on now: RigExecBakedSpaceLeafOverlay
    // answers only from these four tables.
    const auto overlaid=[&](uint32_t k) { rekey[k]|=kRigExecSpaceLeafOverlaid; };
    const auto atPath=[&](const SdfPath &path) {
        auto at=std::lower_bound(index->byPath.begin(),index->byPath.end(),path,
            [](const std::pair<SdfPath,uint32_t> &entry,const SdfPath &p) { return entry.first<p; });
        for(;at!=index->byPath.end() && at->first==path;++at) overlaid(at->second);
    };
    for(const auto &entry:B.routedOverrides) atPath(entry.first);
    for(const auto &entry:B.upstream) atPath(entry.first);
    for(const auto &[slot,k]:index->byHeadSlot)
        if(slot<B.headOverrides.size() && !B.headOverrides[slot].IsEmpty()) overlaid(k);
    // anyOverridden is false exactly when no override flag is set.
    if(B.anyOverridden) for(const auto &[number,k]:index->byNumber)
        if(number<B.overridden.size() && B.overridden[number]) overlaid(k);
    size_t keyed=0;
    const auto publishSpace=[&](size_t k) {
        const RigExecValueId id=index->first+k;
        auto &value=state.values[size_t(id)];
        bool exact=true;
        RigExecOpPublishValue(&value,[&](uint32_t,uint32_t slot,std::string *key) {
            exact=RigExecBakedSpaceLeafKey(B,slot,key);
            if(!exact) RigExecOpKeyAppend(key,value.revision+1);
        });
        if(value.changed) state.changedLeaves.push_back(id);
        rekey[k]=(rekey[k]&kRigExecSpaceLeafOverlaid) || !exact ? kRigExecSpaceLeafHeld : 0;
        ++keyed;
    };
    for(const auto id:index->before) publish(id);
    const size_t count=rekey.size();
    if(all) for(size_t k=0;k<count;++k) publishSpace(k);
    else if(index->verify) {
        std::string scratch;
        for(size_t k=0;k<count;++k) {
            if(rekey[k]) { publishSpace(k); continue; }
            const auto &value=state.values[size_t(index->first+k)];
            const bool exact=RigExecBakedSpaceLeafKey(B,uint32_t(k),&scratch);
            TF_VERIFY(value.initialized && exact && scratch==value.key,
                      "sparse publication skipped provider leaf %zu, whose key moved",k);
        }
    } else for(size_t k=0;k<count;) {
        // Most bytes are clear: step over eight at a time.
        uint64_t word=0;
        if(k+sizeof(word)<=count) {
            std::memcpy(&word,rekey.data()+k,sizeof(word));
            if(!word) { k+=sizeof(word); continue; }
        }
        if(rekey[k]) publishSpace(k);
        ++k;
    }
    for(const auto id:index->after) publish(id);
    B.spaceLeafKeys=keyed;
}
}

bool RigExecBakedCompileOpGraph(RigExecBakedProgramImpl *B,std::string *error)
{
    RigExecBakedDeclareStageFramesAdmission(B);
    // Compile-local membership of the exact native xform source slots.
    std::vector<char> xformSource(B->paths.size(),0);
    for(const int slot:B->xformSlots)
        if(slot>=0 && size_t(slot)<xformSource.size()) xformSource[size_t(slot)]=1;
    if(!RigExecOpCompileAdapter(B->steps,
        [](const auto &r){return uint32_t(r.domain);},
        [](const auto &r){return r.begin;},[](const auto &r){return r.end;},
        [](const auto &step){return step.descriptorKey;},
        [B,&xformSource](uint32_t domain,uint32_t slot) {
            using D=RigExecBakedSlotDomain;
            switch(D(domain)) {
            case D::RequiredStageFramesAdmission: return slot == 0;
            case D::SolverPoints: return slot<B->solvers.size();
            case D::SpaceLeaf: return slot<B->providerLeaves.values.size();
            case D::DerivedBase: return slot<B->derivedIndex.size();
            case D::ChainInput: return slot<B->chains.size();
            case D::ConstraintInputs: return slot<B->constraintArrays.size();
            case D::PoseBase: case D::PoseFin:
                return slot<B->paths.size() &&
                    (slot<=uint32_t(std::numeric_limits<int>::max())
                        ? xformSource[size_t(slot)]!=0
                        : std::find(B->xformSlots.begin(),B->xformSlots.end(),int(slot))!=B->xformSlots.end());
            default: return false;
            }
        },
        &B->opGraph,&B->opAdapter,error,RigExecCyclePolicy::SetAside,
        [B](uint32_t i) -> const std::vector<std::string> & {
            return B->steps[i].semanticPredecessorKeys;
        }, &B->cycleExclusionProof)) return false;
    RigExecBakedBindWeightCycleState(B);
    B->excludedSteps.clear();
    for(size_t i=0;i<B->steps.size();++i) if(B->opGraph.canonicalIndex[i]<0)
        B->excludedSteps.push_back(B->steps[i]);
    std::vector<RigExecBakedStep> canonical;
    canonical.reserve(B->opGraph.ops.size());
    for(uint32_t i=0;i<B->opGraph.ops.size();++i) {
        auto &op=B->opGraph.ops[i];
        canonical.push_back(std::move(B->steps[op.originalIndex]));
        auto &step=canonical.back();
        step.preds.assign(op.predecessors.begin(),op.predecessors.end());
        step.succs.assign(op.successors.begin(),op.successors.end());
        op.descriptor.predecessors=op.predecessors;
        op.originalIndex=i;
    }
    B->steps=std::move(canonical);
    B->opGraph.canonicalIndex.resize(B->steps.size());
    for(uint32_t i=0;i<B->steps.size();++i) B->opGraph.canonicalIndex[i]=int32_t(i);
    std::fill(B->revisionFuseStep.begin(),B->revisionFuseStep.end(),-1);
    for(uint32_t i=0;i<B->steps.size();++i)
        if(B->steps[i].kind==RigExecBakedStepKind::RevisionFuse && B->steps[i].object>=0 &&
           size_t(B->steps[i].object)<B->revisionFuseStep.size())
            B->revisionFuseStep[size_t(B->steps[i].object)]=int(i);
    B->opAdapter.parallel =
        RigExecParallelEvaluationEnabled() &&
        RigExecBakedScheduleModeFromEnvironment()==RigExecBakedScheduleMode::Parallel &&
        !RigExecBakedScheduleCalibrationRequested();
    // Calibration fits what a measurement accumulates, so it measures too.
    B->opAdapter.measuring=RigExecBakedStepTimingRequested() ||
        RigExecBakedScheduleCalibrationRequested();
    B->opAdapter.inputRevisions.assign(B->steps.size(),0);
    B->opAdapter.inputKeys.resize(B->steps.size());
    B->opAdapter.inputScratch.resize(B->steps.size());
    B->opAdapter.sourceKeys.resize(B->steps.size());
    B->opAdapter.sourceScratch.resize(B->steps.size());
    B->opAdapter.coveredPropertyInputs.resize(B->steps.size());
    B->opAdapter.coveredTypedInputs.resize(B->steps.size());
    B->opAdapter.inputExact.assign(B->steps.size(),1);
    // Steps, their lists and revision weight objects are fixed for the
    // program's life, so one classification serves every run and clone.
    auto &state=B->opAdapter;
    state.constantSource.assign(B->opGraph.ops.size(),0);
    state.sourceVisits.clear();
    for(uint32_t c=0;c<B->opGraph.ops.size();++c) {
        const auto &step=B->steps[B->opGraph.ops[c].originalIndex];
        state.constantSource[c]=RigExecBakedOpInputKeyIsConstant(*B,step)?1:0;
        if(!state.constantSource[c] || step.alwaysRuns) state.sourceVisits.push_back(c);
    }
    state.verifyConstantSources=TfGetenvBool("RIGEXEC_VERIFY_CONSTANT_KEYS",false);
    state.verifyLeafVersions=TfGetenvBool("RIGEXEC_VERIFY_LEAF_VERSIONS",false);
    B->verifyChainVersions=TfGetenvBool("RIGEXEC_VERIFY_CHAIN_VERSIONS",false);
    B->chainContentKeys.assign(B->verifyChainVersions?state.values.size():0,std::string());
    B->chainVersionMismatches=0;
    BuildSpaceLeafIndex(B);
    // The skip callback below changes state only through MarkSkipped, whose
    // counters only geometry bodies set, SkipGeometryStep on geometry kinds
    // and FinishHeadOp on property revisions; every other skip is a no-op.
    B->opAdapter.skipEffects.clear();
    B->skinTopologyLayouts.clear();
    for(uint32_t i=0;i<B->steps.size();++i) {
        const auto kind=B->steps[i].kind;
        if(RigExecBakedIsGeometryStep(kind) || kind==RigExecBakedStepKind::PropertyRevision)
            B->opAdapter.skipEffects.push_back(i);
        if(kind==RigExecBakedStepKind::SkinTopology)
            B->skinTopologyLayouts.push_back(B->steps[i].object);
    }
    return true;
}

bool RigExecBakedExecuteOpGraph(RigExecBakedProgramImpl *program,UsdTimeCode time,bool force)
{
    auto &B=*program; auto &state=B.opAdapter;
    std::string error;
    if(!state.compiled && !RigExecBakedCompileOpGraph(&B,&error)) return false;
    // A notice without a declared source route conservatively invalidates
    // every operation once. Tracked value edits retain ordinary cutoff.
    force=force || B.programStamp!=B.lastProgramStamp;
    const bool first=!state.everRan;
    const bool profiling=B.profiler && B.profiler->IsEnabled();
    // Op stamps are plain clock reads into op-owned fields by the thread
    // running the op; the owner reads them only after the join. Decided
    // once here, so every op of the run agrees and runStamped covers them.
    const bool measuring=state.measuring && !B.measurementSuspended;
    const bool timing=B.recordOpTimings || profiling;
    const bool stamping=measuring || timing;
    if(stamping) B.runStamped=true;
    // Only a profiled run folds cluster times (below), so only one leaves
    // any to clear.
    if(B.clustering.lastRunTimed) for(auto &cluster:B.clustering.clusters) {
        cluster.readyUs=cluster.startUs=cluster.endUs=0;
        cluster.runner=std::thread::id();
    }
    B.clustering.lastRunTimed=profiling;
    B.closureFull=force || B.programStamp!=B.lastProgramStamp;
    RigExecBakedPrepareHeadOps(&B);
    RigExecBakedPlaceHeadOverrides(&B);
    RigExecBakedPrepareOracleReference(&B,time);
    // Provider values' own change flags have no reader: op values below
    // carry every change decision.
    for(auto id:state.excludedValues) {
        const auto &value=state.values[size_t(id)];
        ResetExcludedValue(&B,value.domain,value.slot);
    }
    for(const auto &step:B.excludedSteps) if(step.kind==RigExecBakedStepKind::PropertyRevision) {
        const size_t chain=size_t(step.object);
        // Excluded outputs are invalid individually. A later excluded revision
        // cannot invalidate a surviving retained Base/AtPrim producer.
        if(step.part==0) B.chainValid[chain]=0;
        if(size_t(step.part)==B.propertyChains[chain].revisions.size())
            B.chainFinal[chain]=VtValue();
    }
    state.changedLeaves.clear(); state.seeds.clear(); state.candidateOps.clear();
    RigExecOpClearChanges(&state);
    const auto sample=[&](uint32_t d,uint32_t slot,std::string *key) {
        RigExecBakedOpValueKey(B,RigExecBakedSlotDomain(d),slot,key);
    };
    // A constant source key keeps the bytes and exactness of its last build,
    // so it yields what an equal-key compare does: exact and unchanged. Keys
    // are rebuilt on a first run, an adoption or a new program stamp; a run
    // that rebuilds none visits only the ops that can seed or change.
    const bool rebuildSources=first || !state.retainedFirst.empty() || B.programStamp!=B.lastProgramStamp;
    // Provider leaves trust last run's keys on the same terms, and not under force.
    PublishLeaves(&B,rebuildSources || force);
    // The keys below read the path leaves' versions: a write from here on
    // compares against what this run read.
    ++B.pathLeafRun;
    const bool verifyVersions=state.verifyLeafVersions;
    if(verifyVersions) {
        state.contentSourceKeys.resize(B.opGraph.ops.size());
        state.contentInputKeys.resize(B.opGraph.ops.size());
        state.leafVersionMismatch.assign(B.opGraph.ops.size(),0);
    }
    const auto source=[&](uint32_t c) {
        const auto &step=B.steps[B.opGraph.ops[c].originalIndex];
        bool seed=(first && (c>=state.retainedFirst.size() || !state.retainedFirst[c])) || step.alwaysRuns;
        auto &input=state.sourceKeys[c]; auto &scratch=state.sourceScratch[c];
        bool exact=true, directChanged=false;
        if(rebuildSources || c>=state.constantSource.size() || !state.constantSource[c]) {
            exact=RigExecBakedOpInputKey(B,step,&scratch);
            directChanged=!exact || input!=scratch;
            if(verifyVersions) {
                std::string content;
                const bool contentExact=RigExecBakedOpInputKey(B,step,&content,nullptr,true);
                // A first run's stored keys may come from another program.
                if(!first && (contentExact!=exact ||
                   (exact && directChanged!=(state.contentSourceKeys[c]!=content))))
                    state.leafVersionMismatch[c]=1;
                state.contentSourceKeys[c].swap(content);
            }
            input.swap(scratch);
        } else if(state.verifyConstantSources) {
            const bool built=RigExecBakedOpInputKey(B,step,&scratch);
            TF_VERIFY(built && scratch==input,"constant source key of op %u changed",c);
        }
        seed=seed || !exact;
        if(seed) state.seeds.push_back(c);
        else if(directChanged || (first && c<state.retainedFirst.size() && state.retainedFirst[c])) state.candidateOps.push_back(c);
    };
    if(rebuildSources || state.verifyConstantSources || state.constantSource.size()!=B.opGraph.ops.size())
        for(uint32_t c=0;c<B.opGraph.ops.size();++c) source(c);
    else for(const uint32_t c:state.sourceVisits) source(c);
    RigExecOpCallbacks callbacks;
    callbacks.changed=[&](RigExecValueId id){return state.values[size_t(id)].changed!=0;};
    callbacks.inputChanged=[&](uint32_t c,RigExecValueId id) {
        const auto &v=state.values[size_t(id)]; const auto &step=B.steps[B.opGraph.ops[c].originalIndex];
        return v.changed && (v.domain!=uint32_t(RigExecBakedSlotDomain::PropertyResult) || !IsShadowed(B,step,v.slot));
    };
    callbacks.inputsChanged=[&](uint32_t c) {
        auto &step=B.steps[B.opGraph.ops[c].originalIndex];
        if(stamping) step.memoStartNs=RigExecBakedNowNs();
        auto &input=state.inputKeys[c]; auto &scratch=state.inputScratch[c];
        const bool retained=first && c<state.retainedFirst.size() && state.retainedFirst[c];
        bool changed=false;
        if(retained) {
            // Adoption kept the outgoing program's key over leaf contents;
            // compare like with like, then keep this program's versions.
            const bool adopted=RigExecBakedEffectiveMemo(B,c,&scratch,
                &state.coveredPropertyInputs[c],&state.coveredTypedInputs[c],nullptr,true);
            changed=!adopted || input!=scratch;
            if(verifyVersions) state.contentInputKeys[c]=scratch;
        }
        const bool exact=RigExecBakedEffectiveMemo(B,c,&scratch,
            &state.coveredPropertyInputs[c],&state.coveredTypedInputs[c]);
        if(!retained) changed=!exact || input!=scratch;
        if(verifyVersions && !retained) {
            std::string content; std::vector<uint32_t> property;
            std::vector<std::pair<uint32_t,uint32_t>> typed;
            const bool contentExact=RigExecBakedEffectiveMemo(B,c,&content,&property,&typed,nullptr,true);
            if(!first && (contentExact!=exact ||
               (exact && changed!=(state.contentInputKeys[c]!=content))))
                state.leafVersionMismatch[c]=1;
            state.contentInputKeys[c].swap(content);
        }
        state.inputExact[c]=exact?1:0;
        if(!exact) ++state.inputRevisions[c];
        input.swap(scratch);
        if(measuring) step.memoEndNs=RigExecBakedNowNs();
        return changed;
    };
    callbacks.run=[&](uint32_t c) {
        auto &step=B.steps[B.opGraph.ops[c].originalIndex];
        if(B.opBeforeBody) B.opBeforeBody(B.opGraph.ops[c].originalIndex);
        RigExecBakedRunStepBody(&B,&step,time,timing);
        RigExecBakedFinishHeadOp(&B,step);
        // ChainDirty's key is its revision's RevisionDone key. Writes run in
        // (domain, slot) order, so the fuse has built that exact key already.
        const RigExecOpValueState *done=nullptr;
        for(auto id:B.opGraph.ops[c].descriptor.writes) {
            auto &v=state.values[size_t(id)];
            const RigExecOpValueState *reuse=done && v.slot==done->slot &&
                v.domain==uint32_t(RigExecBakedSlotDomain::ChainDirty) ? done : nullptr;
            bool exact=true;
            RigExecOpPublishValue(&v,[&](uint32_t d,uint32_t slot,std::string *key) {
                if(reuse) key->append(reuse->key); else sample(d,slot,key);
                // API4 opaque packets are deterministic in their declared
                // inputs and immutable manifest. Retain their identity
                // while every bound input version remains the same.
                if(!RigExecBakedOpValueKeyIsExact(B,RigExecBakedSlotDomain(d),slot)) {
                    exact=false;
                    RigExecOpKeyAppend(key,state.inputKeys[c]);
                    if(!state.inputExact[c]) RigExecOpKeyAppend(key,state.inputRevisions[c]);
                }
            });
            if(v.domain==uint32_t(RigExecBakedSlotDomain::RevisionDone)) done=exact?&v:nullptr;
        }
        if(stamping) step.publishEndNs=RigExecBakedNowNs();
        if(B.opAfterBody) B.opAfterBody(B.opGraph.ops[c].originalIndex);
        return true;
    };
    callbacks.skip=[&](uint32_t c) {
        auto &step=B.steps[B.opGraph.ops[c].originalIndex]; step.MarkSkipped();
        if(RigExecBakedIsGeometryStep(step.kind)) RigExecBakedSkipGeometryStep(&B,&step);
        RigExecBakedFinishHeadOp(&B,step);
    };
    callbacks.skipEffects=&state.skipEffects;
    bool ok=false;
    const auto execute=[&] { ok=RigExecExecuteOpGraph(B.opGraph,state.changedLeaves,state.seeds,force,
        callbacks,&B.opExecution,&error,&B.opWorkspace,&state.candidateOps); };
    if(state.parallel && !RigExecFrozenSerialActive()) {
        pxr::WorkWithScopedParallelism([&] {
            pxr::WorkDispatcher dispatcher;
            callbacks.dispatch=[&](std::function<void()> task){dispatcher.Run(std::move(task));};
            callbacks.wait=[&]{dispatcher.Wait();}; execute();
        });
    } else execute();
    RigExecOpGatherChanges(&state,B.opGraph,B.opExecution.ran);
    if(verifyVersions)
        for(uint32_t c=0;c<state.leafVersionMismatch.size();++c)
            TF_VERIFY(!state.leafVersionMismatch[c],
                "op %u: a path-leaf version key and its content key disagree on a change",c);
    // After the join, so the workers do no verification: nothing a run
    // writes later moves the points a published value describes.
    if(B.verifyChainVersions)
        for(uint32_t c=0;c<B.opGraph.ops.size() && c<B.opExecution.ran.size();++c)
            if(B.opExecution.ran[c])
                for(const auto id:B.opGraph.ops[c].descriptor.writes) VerifyChainVersion(&B,id);
    if(!ok) {
        state.retainedFirst.clear();
        B.everRan=false;
        state.everRan=false;
        return false;
    }
    // Worker writes are confined to each operation. Reduce coarse timing
    // after join so profiling never adds synchronization to readiness.
    if(profiling) for(uint32_t c=0;c<B.opGraph.ops.size() && c<B.opExecution.ran.size();++c) {
        if(!B.opExecution.ran[c]) continue;
        const auto &step=B.steps[B.opGraph.ops[c].originalIndex];
        auto &cluster=B.clustering.clusters[B.opGraph.opClusters[c]];
        if(!cluster.startUs || step.startUs<cluster.startUs) {
            cluster.startUs=step.startUs; cluster.runner=step.runner;
        }
        cluster.endUs=std::max(cluster.endUs,step.endUs);
    }
    // Every candidate evaluated its memo this run (a failed run returned
    // above) and only an op that ran published, so no stamp of another run
    // is folded.
    if(measuring) for(uint32_t c=0;c<B.opGraph.ops.size() && c<B.opExecution.candidates.size();++c) {
        if(!B.opExecution.candidates[c]) continue;
        auto &step=B.steps[B.opGraph.ops[c].originalIndex];
        if(step.memoStartNs && step.memoEndNs>=step.memoStartNs) {
            step.measuredMemoUs+=double(step.memoEndNs-step.memoStartNs)/1000.0;
            ++step.measuredMemoRuns;
        }
        if(B.opExecution.ran[c] && step.bodyEndNs && step.publishEndNs>=step.bodyEndNs)
            step.measuredPublishUs+=double(step.publishEndNs-step.bodyEndNs)/1000.0;
    }
    RigExecBakedPublishPropertyChains(&B);
    RigExecBakedNoteReaderWalks(&B);
    B.closed.Clear();
    B.closedSteps.Clear();
    for(uint32_t c=0;c<B.opGraph.ops.size();++c)
        if(B.opExecution.ran[c]) {
            B.closed.Set(int(B.opGraph.opClusters[c]));
            B.closedSteps.Set(int(c));
        }
    B.lastClosedClusters=B.closed.Count();
    B.lastClosedSteps=B.opExecution.executed;
    B.restMoved.clear(); B.ladderMoved.clear();
    for(size_t slot=0;slot<B.restChanged.size();++slot)
        if(B.restChanged[slot]) B.restMoved.push_back(int(slot));
    for(size_t slot=0;slot<B.ladderChanged.size();++slot)
        if(B.ladderChanged[slot]) B.ladderMoved.push_back(int(slot));
    B.headOpsRun=0;
    for(uint32_t c=0;c<B.opGraph.ops.size();++c)
        if(B.opExecution.ran[c] && B.steps[B.opGraph.ops[c].originalIndex].isHead) ++B.headOpsRun;
    state.retainedFirst.clear();
    // Pure preparation and admission outcomes completed, but a refused
    // public generation does not accept the pending epoch/edit debt.
    state.everRan=ok;
    if (B.requiredStageFramesAdmission.admitted) {
        B.lastTime=time; B.everRan=ok;
        B.lastProgramStamp=B.programStamp;
        B.lastOverridden=B.overridden;
        B.lastHeadOverrides=B.headOverrides;
        std::fill(B.edited.begin(),B.edited.end(),char(0)); B.anyEdited=false;
    }
    return ok;
}

bool RigExecBakedLowerOpGraph(RigExecBakedProgramImpl *B,std::string *error)
{
    std::vector<double> costs;
    costs.reserve(B->opGraph.ops.size());
    double total=0;
    for(const auto &op:B->opGraph.ops) {
        const double cost=B->steps[op.originalIndex].cost;
        costs.push_back(cost); total+=cost;
    }
    const double grain=RigExecBakedScheduleGrainUs(total);
    if(!RigExecLowerOpClusters(&B->opGraph,costs,grain,error)) return false;
    // Compatibility reports and cone views reflect the common artifact.
    auto &view=B->clustering;
    view={}; view.grainUs=grain; view.serialCost=total;
    view.clusterOf.assign(B->opGraph.opClusters.begin(),B->opGraph.opClusters.end());
    view.clusters.resize(B->opGraph.clusters.size());
    std::vector<double> path(view.clusters.size(),0);
    for(size_t i=0;i<view.clusters.size();++i) {
        const auto &compiled=B->opGraph.clusters[i]; auto &cluster=view.clusters[i];
        cluster.members.assign(compiled.members.begin(),compiled.members.end());
        cluster.preds.assign(compiled.predecessors.begin(),compiled.predecessors.end());
        cluster.succs.assign(compiled.successors.begin(),compiled.successors.end());
        for(uint32_t member:compiled.members) {
            cluster.cost+=costs[member]; cluster.level=std::max(cluster.level,B->steps[member].level);
            B->steps[member].cluster=int(i);
        }
        for(uint32_t predecessor:compiled.predecessors) path[i]=std::max(path[i],path[predecessor]);
        path[i]+=cluster.cost; view.criticalPathCost=std::max(view.criticalPathCost,path[i]);
        view.topologicalOrder.push_back(int(i));
    }
    return true;
}
} // namespace rigExec
