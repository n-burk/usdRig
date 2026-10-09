// Runtime adapter for the shared readiness and value propagation loop.
#include "rigExecRuntime/labels.h"
#include "rigExecRuntime/store.h"
#include "opGraph.h"
#include "poseInternal.h"
#include "spaces.h"

#include <algorithm>
#include <cstring>
#include <limits>
#include <map>
#include <set>

namespace rigExec {

namespace {

bool
_RrIsGeometryKind(RigExecWireStepKind kind)
{
    switch (kind) {
    case RigExecWireStepKind::InfluenceFold:
    case RigExecWireStepKind::RevisionStatic:
    case RigExecWireStepKind::RevisionChunk:
    case RigExecWireStepKind::RevisionFuse:
    case RigExecWireStepKind::ChainStatus:
    case RigExecWireStepKind::Derived:
    case RigExecWireStepKind::ChainInputs:
        return true;
    default:
        return false;
    }
}

bool
_RrIsWeightKind(RigExecWireStepKind kind)
{
    return kind == RigExecWireStepKind::WeightField ||
           kind == RigExecWireStepKind::WeightPacket ||
           kind == RigExecWireStepKind::VolumePlacements;
}

// The skip callback changes state only through MarkSkipped, whose counters
// only geometry bodies set, and RrSkipGeometryStep; other skips are no-ops.
void
_RrIndexSkipEffects(RrProgram *p)
{
    auto &skipEffects = p->store.opAdapter.skipEffects;
    skipEffects.clear();
    for (uint32_t c = 0; c < p->opGraph.ops.size(); ++c)
        if (_RrIsGeometryKind((*p->steps)[p->opGraph.ops[c].originalIndex].kind))
            skipEffects.push_back(c);
}

}  // namespace

namespace {
template <class T> void _RrAppend(std::string *key, const T &value)
{
    key->append(reinterpret_cast<const char *>(&value), sizeof(value));
}
void _RrMemoValue(std::string *key, const RrWireValue &value)
{
    _RrAppend(key, value.tag); _RrAppend(key, value.bits);
    _RrAppend(key, value.matrix); _RrAppend(key, value.vec3d);
    _RrAppend(key, value.vec3f);
    _RrAppend(key, value.vec3i);
}
template <class T> void _RrMemoArray(const RrProgram *program, uint32_t slot,
                                   std::string *key)
{
    const auto *values = RrInputArray<T>(program, slot);
    const size_t count = values ? values->size() : 0;
    _RrAppend(key, count);
    if (count) key->append(reinterpret_cast<const char *>(values->data()),
                           sizeof(T) * count);
}
// \p versioned keys an array's elements by their content version, for a
// memo every run rebuilds and compares with the previous run's only.
void _RrRawSlotMemo(const RrProgram *program, int slot, std::string *key,
                    bool versioned=false)
{
    const auto &state = program->inputState;
    const bool declared = slot >= 0 && size_t(slot) < state.slotHasValue.size();
    _RrAppend(key, declared);
    if (!declared) return;
    _RrAppend(key, state.slotHasValue[size_t(slot)]);
    const auto tag = state.file->inputs[size_t(slot)].type();
    const bool array = RrInputTagIsArray(RrInputTag(uint8_t(tag)));
    // Arrays carry authorship beside their retained payload. An authored
    // set of the same bytes changes a Default read after a sampled set.
    _RrAppend(key, array ? RrInputArrayAuthored(program,uint32_t(slot))
                         : state.slotAuthored[size_t(slot)]!=0);
    _RrAppend(key, state.slotBlocked[size_t(slot)]);
    _RrAppend(key, tag);
    if (array && versioned) {
        // The elements are read only while the slot holds a value.
        if (state.slotHasValue[size_t(slot)])
            _RrAppend(key, RrInputArrayVersion(program, uint32_t(slot)));
    } else if (array) {
        switch (tag) {
        case RigExecWireInputTag::IntArray: _RrMemoArray<int32_t>(program,slot,key); break;
        case RigExecWireInputTag::FloatArray: _RrMemoArray<float>(program,slot,key); break;
        case RigExecWireInputTag::DoubleArray: _RrMemoArray<double>(program,slot,key); break;
        case RigExecWireInputTag::Vec2fArray: _RrMemoArray<RrVec2f>(program,slot,key); break;
        case RigExecWireInputTag::Vec3fArray: _RrMemoArray<RrVec3f>(program,slot,key); break;
        case RigExecWireInputTag::Vec3dArray: _RrMemoArray<RigExecWireVec3d>(program,slot,key); break;
        case RigExecWireInputTag::Matrix4dArray: _RrMemoArray<RigExecWireMatrix4d>(program,slot,key); break;
        case RigExecWireInputTag::TokenArray: _RrMemoArray<uint32_t>(program,slot,key); break;
        case RigExecWireInputTag::BoolArray: _RrMemoArray<uint8_t>(program,slot,key); break;
        default: break;
        }
    } else {
        _RrMemoValue(key,state.slotCurrent[size_t(slot)]);
        if (tag == RigExecWireInputTag::Token)
            RigExecOpKeyAppend(key,program->TextOrEmpty(uint32_t(state.slotCurrent[size_t(slot)].bits)));
    }
}
// \p contentArrays keys every array by its elements, as the
// verifyLeafVersions cross-check does.
void _RrInputMemo(const RrProgram *program, const RigExecWireStep &step, std::string *key,
                  bool contentArrays=false)
{
    const auto &state=program->inputState;
    // Geometry and weight inputs key array elements by content version.
    const bool versioned=!contentArrays && (_RrIsGeometryKind(step.kind) ||
        _RrIsWeightKind(step.kind) || step.kind==RigExecWireStepKind::SkinTopology);
    const auto appendSlot=[&](uint32_t slot) { _RrRawSlotMemo(program,int(slot),key,versioned); };
    for(uint32_t slot:step.headInputSlots) appendSlot(slot);
    for(const auto &read:step.headInputReads) RrSourceReadMemo(program,read,key,versioned);
    if(step.kind==RigExecWireStepKind::AvarInputs) {
        const auto range=RrAvarReadRange(state,step.object);
        for(uint32_t i=range.first;i<range.second;++i) {
            const auto &binding=program->registeredReads[state.avarReads[i]];
            const auto &read=*binding.read;
            _RrAppend(key,binding.avar); _RrMemoValue(key,state.values[read.constant]);
            for(uint32_t slot:read.walk) appendSlot(slot);
            for(const auto &candidate:read.propertyCandidates) if(candidate.raw) appendSlot(candidate.slot);
            for(const auto &candidate:read.doubleCandidates) if(candidate.raw) appendSlot(candidate.slot);
            if(read.rawFallbackSlot>=0) appendSlot(uint32_t(read.rawFallbackSlot));
        }
    }
    for(int number:step.overrideInputs) {
        if(number<0 || size_t(number)+1>=state.overrideSlotBegin.size()) continue;
        for(uint32_t i=state.overrideSlotBegin[size_t(number)];i<state.overrideSlotBegin[size_t(number)+1];++i)
            appendSlot(state.overrideSlotList[i]);
    }
}
// The gates of every input read in _RrInputMemo; a step with none memoizes
// the empty key. Mirrors RigExecBakedOpInputKeyIsConstant.
bool _RrInputMemoIsConstant(const RigExecWireStep &step)
{
    return step.kind!=RigExecWireStepKind::AvarInputs && step.headInputSlots.empty() &&
        step.headInputReads.empty() && step.overrideInputs.empty();
}
void _RrPropertyVersion(const RrProgram *program, uint32_t v,std::string *key)
{
    const auto &value = program->store.propertyVersions[v];
    _RrAppend(key, program->store.propertyVersionValid[v]);
    _RrAppend(key, value.tag); _RrAppend(key, value.f32);
    _RrAppend(key, value.f64);
    for (size_t row = 0; row < 4; ++row)
        for (size_t col = 0; col < 4; ++col)
            _RrAppend(key, value.matrix[row][col]);
    for (size_t k = 0; k < 3; ++k) _RrAppend(key, value.vec[k]);
}
bool _RrShadowed(const RrStore &store, const RigExecWireStep &step, uint32_t v)
{
    bool found = false;
    for (const auto &pair : step.shadowedReads) {
        if (pair.first != int32_t(v)) continue;
        if (store.propertyRecordStoodAside[size_t(pair.second)]) return false;
        found = true;
    }
    return found;
}
} // namespace


namespace {
void RrOpFrame(std::string *key, const RrPointFrame &v)
{ RigExecOpKeyAppend(key,v.points); RigExecOpKeyAppend(key,v.flags); }
template<class T> void RrOpArray(std::string *key, const std::vector<T> &v)
{ RigExecOpKeyArray(key,v); }
void RrOpArray(std::string *key, const std::vector<RrPointFrame> &v)
{ RigExecOpKeyAppend(key,v.size()); for(const auto &x:v) RrOpFrame(key,x); }
void RrResetExcludedValue(RrProgram *p,uint32_t domain,uint32_t slot)
{
    auto &s=p->store; auto *pose=static_cast<RrPoseScratch *>(p->pose.get());
    RrPointFrame invalid; invalid.flags=0; RrMat4d identity; identity.SetIdentity();
    RrMat4d invalidMatrix=identity;
    invalidMatrix[3][0]=std::numeric_limits<double>::quiet_NaN();
    using D=RigExecWireSlotDomain;
    switch(D(domain)) {
    case D::Avars: std::fill(s.avars.begin()+size_t(slot)*11,s.avars.begin()+(size_t(slot)+1)*11,0); break;
    case D::PoseBase: s.base[slot]=invalid; break;
    case D::PoseFin: s.fin[slot]=invalid; break;
    case D::PosedM: s.posedM[slot]=invalidMatrix; break;
    case D::FinalMatrix: s.finalMatrix[slot]=invalidMatrix; break;
    case D::BaseMatrix: s.baseMatrix[slot]=invalidMatrix; break;
    case D::SwitchFrame: s.switchFrames[slot]=invalidMatrix; break;
    case D::Aggregate: s.aggregates[slot]={}; break;
    case D::Candidates:
        std::fill(s.solverOutPresent[slot].begin(),s.solverOutPresent[slot].end(),char(0));
        std::fill(s.solverOutFrames[slot].begin(),s.solverOutFrames[slot].end(),invalid); break;
    case D::CommitTable: {
        auto &c=s.commits[slot]; c.abandoned=true;
        std::fill(c.present.begin(),c.present.end(),char(0)); std::fill(c.frames.begin(),c.frames.end(),invalid); break;
    }
    case D::CommitDelta: std::fill(s.commits[slot].deltaOk.begin(),s.commits[slot].deltaOk.end(),char(0)); break;
    case D::CommitStaging:
        for(size_t c=0;c<p->poses->commits.size();++c) {
            const auto base=p->poses->commits[c].stagingBase;
            if(base<0 || slot<uint32_t(base) || uint64_t(slot)-uint32_t(base)>=s.commits[c].staged.size()) continue;
            const auto k=slot-uint32_t(base); s.commits[c].staged[k]=invalid; s.commits[c].outcome[k]=2; break;
        } break;
    case D::ConstraintDelta: s.deltaPresent[slot]=0; s.deltaValues[slot]=identity; break;
    case D::PropertyResult:
        s.propertyVersionValid[slot]=0; s.propertyVersions[slot]={};
        for(size_t r=0;r<p->file->phasedConsumers.size();++r)
            if(p->file->phasedConsumers[r].version==slot) s.propertyRecordStoodAside[r]=1;
        break;
    case D::WeightPacket: s.weightPackets[slot]={}; break;
    case D::WeightFrames:
        s.volumePlaced[slot]=slot<p->constants->noScaleAvars.size() && p->constants->noScaleAvars[slot];
        s.volumePlacement[slot]=invalidMatrix; break;
    case D::WeightFramesBase:
        s.volumePlacedBase[slot]=slot<p->constants->noScaleAvars.size() && p->constants->noScaleAvars[slot];
        s.volumePlacementBase[slot]=invalidMatrix; break;
    case D::PoseWeight: s.poseWeights[slot]=0; break;
    case D::FrameMatrix: s.frameMatrixValid[slot]=0; s.frameMatrix[slot]=identity; break;
    case D::WeightField: {
        auto &v=s.weightFieldResults[slot]; v.values.clear(); v.count=0; v.ok=false; v.error="operation cycle"; break;
    }
    case D::SpaceValue: {
        auto &v=s.providerValues[slot]; v.value=std::monostate(); v.initialized=false;
        v.authoritative=false; v.blocked=true; v.count=0; v.error="operation cycle"; break;
    }
    case D::Rest:
        if(pose) {
            pose->restFrames[slot]=invalid; pose->restM[slot]=identity;
            for(auto &point:pose->restPts[slot]) point=RrVec3d(0);
        } break;
    case D::Ladder:
        if(pose) {
            pose->selfD[slot]=pose->parentDinv[slot]=pose->restRoundTrip[slot]=pose->defaultRoundTrip[slot]=identity;
            pose->posedAuthoredM[slot]=pose->posedD[slot]=pose->parentSpaceM[slot]=identity;
            pose->posedAuthored[slot]=pose->parentSpaceAuthored[slot]=0;
            pose->rotOrder[slot]=0; pose->rotationSign[slot]=0;
        } break;
    default: RrResetExcludedGeometryValue(p,D(domain),slot); break;
    }
}
void RrOpValue(const RrProgram *p, uint32_t domain, uint32_t slot, std::string *key)
{
    const auto d=RigExecWireSlotDomain(domain); const auto &s=p->store;
    const auto *pose=static_cast<const RrPoseScratch *>(p->pose.get());
    switch(d) {
    case RigExecWireSlotDomain::Avars:
        for(size_t i=size_t(slot)*11;i<size_t(slot)*11+11 && i<s.avars.size();++i) RigExecOpKeyAppend(key,s.avars[i]);
        for(size_t i=0;i<p->slotMeta->xformSlots.size();++i) if(p->slotMeta->xformSlots[i]==int32_t(slot)) RigExecOpKeyAppend(key,s.xformBase[i]);
        break;
    case RigExecWireSlotDomain::PoseBase: RrOpFrame(key,s.base[slot]); break;
    case RigExecWireSlotDomain::PoseFin: RrOpFrame(key,s.fin[slot]); break;
    case RigExecWireSlotDomain::PosedM: RigExecOpKeyAppend(key,s.posedM[slot]); break;
    case RigExecWireSlotDomain::FinalMatrix: RigExecOpKeyAppend(key,s.finalMatrix[slot]); break;
    case RigExecWireSlotDomain::BaseMatrix: RigExecOpKeyAppend(key,s.baseMatrix[slot]); break;
    case RigExecWireSlotDomain::SwitchFrame: RigExecOpKeyAppend(key,s.switchFrames[slot]); break;
    case RigExecWireSlotDomain::RequiredStageFramesAdmission:
        RigExecOpKeyAppend(key,uint8_t(p->requiredStageFramesAdmission.admitted));
        RigExecOpKeyAppend(key,p->requiredStageFramesAdmission.firstBadTarget); break;
    case RigExecWireSlotDomain::Aggregate: RrOpArray(key,s.aggregates[slot].frames); RrOpArray(key,s.aggregates[slot].rests); break;
    case RigExecWireSlotDomain::SolverPoints: RrOpArray(key,s.ribbonConstant[slot]); break;
    case RigExecWireSlotDomain::Candidates: RrOpArray(key,s.solverOutFrames[slot]); RrOpArray(key,s.solverOutPresent[slot]); break;
    case RigExecWireSlotDomain::CommitTable: {
        const auto &c=s.commits[slot]; RigExecOpKeyAppend(key,c.abandoned);
        RrOpArray(key,c.frames); RrOpArray(key,c.present);
        if(pose) { RigExecOpKeyAppend(key,pose->recordAfter[slot]); RigExecOpKeyAppend(key,pose->recordEveryTarget[slot]); } break;
    }
    case RigExecWireSlotDomain::CommitDelta: RrOpArray(key,s.commits[slot].deltas); RrOpArray(key,s.commits[slot].deltaOk); break;
    case RigExecWireSlotDomain::CommitStaging:
        for(size_t c=0;c<p->poses->commits.size();++c) {
            const auto &wire=p->poses->commits[c];
            if(slot<uint32_t(wire.stagingBase) || slot>=uint32_t(wire.stagingBase)+wire.propagate.size()) continue;
            const auto k=slot-uint32_t(wire.stagingBase); RrOpFrame(key,s.commits[c].staged[k]); RigExecOpKeyAppend(key,s.commits[c].outcome[k]); break;
        } break;
    case RigExecWireSlotDomain::ConstraintDelta: RigExecOpKeyAppend(key,s.deltaPresent[slot]); RigExecOpKeyAppend(key,s.deltaValues[slot]); break;
    case RigExecWireSlotDomain::PropertyResult: _RrPropertyVersion(p,slot,key); break;
    case RigExecWireSlotDomain::WeightPacket: {
        const auto &w=s.weightPackets[slot]; RigExecOpKeyAppend(key,w.representation); RigExecOpKeyAppend(key,w.rangePolicy);
        RigExecOpKeyAppend(key,w.defaultWeight); RigExecOpKeyAppend(key,w.valid); RrOpArray(key,w.values); RrOpArray(key,w.indices); break;
    }
    case RigExecWireSlotDomain::WeightFrames: RigExecOpKeyAppend(key,s.volumePlaced[slot]); RigExecOpKeyAppend(key,s.volumePlacement[slot]); break;
    case RigExecWireSlotDomain::WeightFramesBase: RigExecOpKeyAppend(key,s.volumePlacedBase[slot]); RigExecOpKeyAppend(key,s.volumePlacementBase[slot]); break;
    case RigExecWireSlotDomain::PoseWeight: RigExecOpKeyAppend(key,s.poseWeights[slot]); break;
    case RigExecWireSlotDomain::FrameMatrix: RigExecOpKeyAppend(key,s.frameMatrixValid[slot]); RigExecOpKeyAppend(key,s.frameMatrix[slot]); break;
    case RigExecWireSlotDomain::WeightField: {
        const auto &w=s.weightFieldResults[slot]; RigExecOpKeyAppend(key,w.count); RigExecOpKeyAppend(key,w.ok); RigExecOpKeyAppend(key,w.error); RrOpArray(key,w.values); break;
    }
    case RigExecWireSlotDomain::Rest:
        if(pose) {
            RigExecOpKeyAppend(key,pose->restM[slot]); RrOpFrame(key,pose->restFrames[slot]);
            RigExecOpKeyAppendRun(key,pose->restPts[slot].data(),pose->restPts[slot].size());
        } break;
    case RigExecWireSlotDomain::Ladder:
        if(pose) { RigExecOpKeyAppend(key,pose->selfD[slot]); RigExecOpKeyAppend(key,pose->parentDinv[slot]);
            RigExecOpKeyAppend(key,pose->restRoundTrip[slot]); RigExecOpKeyAppend(key,pose->defaultRoundTrip[slot]); RigExecOpKeyAppend(key,pose->rotOrder[slot]);
            RigExecOpKeyAppend(key,pose->posedAuthored[slot]); RigExecOpKeyAppend(key,pose->posedAuthoredM[slot]);
            RigExecOpKeyAppend(key,pose->posedD[slot]); RigExecOpKeyAppend(key,pose->parentSpaceM[slot]);
            RigExecOpKeyAppend(key,pose->parentSpaceAuthored[slot]); RigExecOpKeyAppend(key,pose->rotationSign[slot]); } break;
    case RigExecWireSlotDomain::SpaceValue: {
        const auto &v=s.providerValues[slot];
        RigExecOpKeyAppend(key,v.initialized); RigExecOpKeyAppend(key,v.authoritative);
        RigExecOpKeyAppend(key,v.blocked); RigExecOpKeyAppend(key,v.count);
        RigExecOpKeyAppend(key,v.error); RigExecOpKeyAppend(key,v.value.index());
        RigExecOpKeyPlainValue(key,v.value);
        break;
    }
    case RigExecWireSlotDomain::ConstraintInputs:
        for (int input : p->file->pose->constraintArrays[slot].rawSlots)
            _RrRawSlotMemo(p,input,key);
        break;
    case RigExecWireSlotDomain::SpaceLeaf: {
        const auto &leaf=p->file->providerProgram->sampled[slot];
        const int input=leaf.inputSlot;
        RigExecOpKeyAppend(key,input);
        if(input<0) { RigExecOpKeyAppend(key,uint8_t(0)); break; }
        _RrRawSlotMemo(p,input,key);
        break;
    }
    default: RrGeometryOpValueKey(p,d,slot,key); break;
    }
}
bool RrRunOpBody(RrProgram *p,size_t i,std::string *error)
{
    const auto &step=(*p->steps)[i]; auto &s=p->store;
    s.stepOutputs[i].BeginRun();
    if(!p->requiredStageFramesAdmission.admitted &&
       std::any_of(step.reads.begin(),step.reads.end(),[](const auto &range) {
           return range.domain()==RigExecWireSlotDomain::RequiredStageFramesAdmission;
       })) return true;
    bool ok=true;
    if(step.kind==RigExecWireStepKind::PropertyRevision) {
        auto &lines=s.headLines[i]; lines.clear();
        ok=RrRunPropertyPart(p,size_t(step.object),size_t(step.part),&lines);
        if(!ok && error) *error="step "+RrStepLabel(*p,i)+" has no property part";
    } else if(step.kind==RigExecWireStepKind::RestCompose || step.kind==RigExecWireStepKind::LadderCompose)
        RrRunRestHead(p,size_t(step.object),step.kind==RigExecWireStepKind::LadderCompose);
    else if(step.kind==RigExecWireStepKind::SkinTopology) RrRunTopologyHead(p,size_t(step.object));
    else if(step.kind==RigExecWireStepKind::SpaceExpression) ok=RrRunSpaceExpression(p,step,error);
    else if(_RrIsWeightKind(step.kind)) ok=RrRunWeightStep(p,i,error);
    else if(_RrIsGeometryKind(step.kind)) ok=RrRunGeometryStep(p,i,error);
    else ok=RrRunPoseStep(p,i,error);
    return ok;
}
}

static bool RrCompileCommonGraph(RrProgram *p,std::string *error)
{
    if(!p->file) {
        // In-memory kernel fixtures bypass strict Open. Validate opted-in
        // authored relationships before compiling their declared semantic keys.
        const auto invalid = [&](const char *message) { if (error) *error = message; return false; };
        for (const auto &step : *p->steps) {
            if (step.kind != RigExecWireStepKind::Solve) {
                if (!step.semanticPredecessorKeys.empty()) return invalid("semantic prerequisites require a Solve body");
                continue;
            }
            if (step.semanticPredecessorKeys.empty() && (!p->poses || step.object < 0 ||
                size_t(step.object) >= p->poses->solvers.size())) continue;
            if (!p->poses || step.object < 0 || size_t(step.object) >= p->poses->solvers.size())
                return invalid("semantic solver owner has no record");
            const auto &solver = p->poses->solvers[size_t(step.object)];
            if (solver.relationshipRequirements.empty() && solver.solveDescriptorKey.empty() &&
                step.semanticPredecessorKeys.empty()) continue;
            if (solver.solveDescriptorKey.empty() || solver.solveDescriptorKey != step.descriptorKey)
                return invalid("semantic solver descriptor identity differs from body");
            const auto type = p->TextOrEmpty(solver.type);
            std::vector<std::string> ports;
            if (type == "RigExecFkChain") ports = {"rigExec:controls","rigExec:startFrame"};
            else if (type == "RigExecTwoBoneIk") ports = {"rigExec:rootControl","rigExec:effectorControl","rigExec:poleControl","rigExec:space"};
            else if (type == "RigExecBlendPointFrames") ports = {"rigExec:inputA","rigExec:inputB"};
            else if (type == "RigExecTwistDistribution") ports = {"rigExec:start","rigExec:end"};
            else if (type == "RigExecRibbon") ports =
                {"rigExec:driverCurve", "rigExec:startFrame", "rigExec:endFrame", "rigExec:twistFrames"};
            else if (type == "RigExecSplineIk") ports = {"rigExec:rootControl","rigExec:midControl","rigExec:endControl","rigExec:space"};
            std::set<std::pair<std::string,int>> seen;
            std::map<std::string,size_t> counts;
            std::vector<std::string> keys;
            for (const auto &requirement : solver.relationshipRequirements) {
                if (std::find(ports.begin(),ports.end(),requirement.port) == ports.end() ||
                    !seen.emplace(requirement.port,requirement.solver).second ||
                    (requirement.port != "rigExec:controls" &&
                     !(type == "RigExecRibbon" && requirement.port != "rigExec:driverCurve") &&
                     ++counts[requirement.port] > 1) ||
                    requirement.solver < 0 || size_t(requirement.solver) >= p->poses->solvers.size())
                    return invalid("invalid solver relationship requirement");
                if (type == "RigExecBlendPointFrames" &&
                    ((requirement.port == "rigExec:inputA" && requirement.solver != solver.inA) ||
                     (requirement.port == "rigExec:inputB" && requirement.solver != solver.inB)))
                    return invalid("solver relationship differs from aggregate binding");
                const auto &source = p->poses->solvers[size_t(requirement.solver)];
                if (source.solveDescriptorKey.empty()) return invalid("semantic source has no descriptor identity");
                const auto found = std::find_if(p->steps->begin(),p->steps->end(),[&](const auto &candidate) {
                    return candidate.descriptorKey == source.solveDescriptorKey;
                });
                if (found == p->steps->end() || found->kind != RigExecWireStepKind::Solve || found->object != requirement.solver)
                    return invalid("semantic source key names no matching Solve body");
                keys.push_back(source.solveDescriptorKey);
            }
            if (type == "RigExecBlendPointFrames" &&
                ((solver.inA >= 0 && !seen.count({"rigExec:inputA",solver.inA})) ||
                 (solver.inB >= 0 && !seen.count({"rigExec:inputB",solver.inB}))))
                return invalid("solver omits aggregate relationship requirement");
            std::sort(keys.begin(),keys.end()); keys.erase(std::unique(keys.begin(),keys.end()),keys.end());
            if (keys != step.semanticPredecessorKeys) return invalid("semantic keys differ from solver requirements");
        }
        const bool compiled = RigExecOpCompileAdapter(*p->steps,
            [](const auto &r){return uint32_t(r.domain());},
            [](const auto &r){return r.begin();},[](const auto &r){return r.end();},
            [&](const auto &step){ const auto i=size_t(&step-p->steps->data());
                return step.descriptorKey.empty() ?
                    RrStepLabel(*p,i)+"/"+std::to_string(uint32_t(step.kind))+"/"+std::to_string(step.part) :
                    step.descriptorKey; },
            [p](uint32_t domain,uint32_t slot) {
                using D=RigExecWireSlotDomain;
                switch(D(domain)) {
                case D::SolverPoints: case D::SpaceLeaf: case D::DerivedBase:
                case D::ChainInput: case D::ConstraintInputs: return true;
                case D::RequiredStageFramesAdmission: return slot==0;
                case D::PoseBase: case D::PoseFin:
                    return p->slotMeta && std::find(p->slotMeta->xformSlots.begin(),
                        p->slotMeta->xformSlots.end(),int32_t(slot))!=p->slotMeta->xformSlots.end();
                default: return false;
                }
            },
            &p->opGraph,&p->store.opAdapter,error,RigExecCyclePolicy::Reject,
            [p](uint32_t i) -> const std::vector<std::string> & {
                return (*p->steps)[i].semanticPredecessorKeys;
            });
        if (compiled) _RrIndexSkipEffects(p);
        return compiled;
    }
    if(!p->file->commonGraph) { if(error) *error="missing common operation graph"; return false; }
    const auto &wire=*p->file->commonGraph;
    if(wire.ops.size()!=p->steps->size()) { if(error) *error="common graph omits operation bodies"; return false; }
    RigExecCompiledGraph graph; RigExecOpAdapterState state;
    for(const auto &spec:wire.valueSpecs) state.values.push_back({spec.domain,spec.slot});
    state.leaves=wire.leaves; graph.canonicalIndex=wire.canonicalIndex;
    state.excludedValues=wire.excludedValues;
    graph.longestPath=size_t(wire.longestPath);
    std::map<std::string, uint32_t> byKey;
    for (uint32_t i = 0; i < wire.ops.size(); ++i)
        if (!byKey.emplace(wire.ops[i].key, i).second) {
            if (error) *error = "duplicate common operation key"; return false;
        }
    const auto excludedSolverValue = [&](uint32_t domain, uint32_t slot) {
        for (auto id : state.excludedValues)
            if (id < state.values.size() && state.values[size_t(id)].domain == domain &&
                state.values[size_t(id)].slot == slot) return true;
        return false;
    };
    std::vector<RigExecOpDescriptor> descriptors;
    for(const auto &row:wire.ops) {
        RigExecCompiledOp op; op.originalIndex=row.originalIndex;
        if(op.originalIndex>=p->steps->size()) { if(error) *error="common operation names no body"; return false; }
        op.descriptor.key=row.key; op.descriptor.kind=row.kind;
        if(op.descriptor.kind!=uint32_t((*p->steps)[op.originalIndex].kind)) {
            if(error) *error="common operation kind differs from its body"; return false;
        }
        op.descriptor.reads=row.reads; op.descriptor.writes=row.writes;
        // Serialized generated edges are checked below, not imported as
        // semantic authority. Reconstruct only declared solver prerequisites.
        op.descriptor.volatileInput=row.volatileInput;
        op.predecessors=row.predecessors; op.successors=row.successors;
        for(auto id:op.descriptor.reads) if(id>=state.values.size()) { if(error) *error="common read names no value"; return false; }
        for(auto id:op.descriptor.writes) if(id>=state.values.size()) { if(error) *error="common write names no value"; return false; }
        const auto &body=(*p->steps)[op.originalIndex];
        std::vector<std::string> requiredKeys;
        if (body.kind == RigExecWireStepKind::Solve) {
            if (!p->poses || body.object < 0 || size_t(body.object) >= p->poses->solvers.size()) {
                if (error) *error = "semantic solver owner has no record"; return false;
            }
            const auto &solver = p->poses->solvers[size_t(body.object)];
            for (const auto &requirement : solver.relationshipRequirements) {
                if (requirement.solver < 0 || size_t(requirement.solver) >= p->poses->solvers.size()) {
                    if (error) *error = "semantic relationship target has no solver"; return false;
                }
                const auto &source = p->poses->solvers[size_t(requirement.solver)];
                requiredKeys.push_back(source.solveDescriptorKey);
                const auto found = byKey.find(source.solveDescriptorKey);
                if (found != byKey.end()) {
                    const auto &sourceOp = wire.ops[found->second];
                    if (sourceOp.kind != uint32_t(RigExecWireStepKind::Solve) ||
                        sourceOp.originalIndex >= p->steps->size() ||
                        (*p->steps)[sourceOp.originalIndex].object != requirement.solver) {
                        if (error) *error = "semantic relationship key names another body"; return false;
                    }
                    op.descriptor.predecessors.push_back(found->second);
                } else if (!excludedSolverValue(uint32_t(RigExecWireSlotDomain::Aggregate), uint32_t(requirement.solver)) ||
                           !excludedSolverValue(uint32_t(RigExecWireSlotDomain::Candidates), uint32_t(requirement.solver))) {
                    if (error) *error = "semantic relationship target is neither active nor excluded"; return false;
                }
            }
        }
        std::sort(requiredKeys.begin(), requiredKeys.end());
        requiredKeys.erase(std::unique(requiredKeys.begin(), requiredKeys.end()), requiredKeys.end());
        if (body.semanticPredecessorKeys != requiredKeys) {
            if (error) *error = "operation semantic prerequisites differ from solver requirements"; return false;
        }
        const auto projection=[&](const auto &ids) {
            std::vector<std::pair<uint32_t,uint32_t>> values;
            for(auto id:ids) {
                const auto &value=state.values[size_t(id)];
                values.emplace_back(value.domain,value.slot);
            }
            std::sort(values.begin(),values.end());
            values.erase(std::unique(values.begin(),values.end()),values.end());
            return values;
        };
        const auto declaration=[](const auto &ranges) {
            std::vector<std::pair<uint32_t,uint32_t>> values;
            for(const auto &range:ranges)
                for(uint32_t slot=range.begin();slot<range.end();++slot)
                    values.emplace_back(uint32_t(range.domain()),slot);
            std::sort(values.begin(),values.end());
            values.erase(std::unique(values.begin(),values.end()),values.end());
            return values;
        };
        if(projection(op.descriptor.reads)!=declaration(body.reads) ||
           projection(op.descriptor.writes)!=declaration(body.writes) ||
           op.descriptor.volatileInput!=body.externalReads) {
            if(error) *error="common operation values differ from its body declarations";
            return false;
        }
        std::vector<uint32_t> bodyPredecessors(body.preds.begin(),body.preds.end());
        std::sort(bodyPredecessors.begin(),bodyPredecessors.end());
        bodyPredecessors.erase(std::unique(bodyPredecessors.begin(),bodyPredecessors.end()),bodyPredecessors.end());
        if(bodyPredecessors!=op.predecessors) {
            if(error) *error="common operation predecessors differ from its body declarations";
            return false;
        }
        descriptors.push_back(op.descriptor); graph.ops.push_back(std::move(op));
    }
    for(auto id:state.leaves) if(id>=state.values.size()) { if(error) *error="common leaf names no value"; return false; }
    for(auto id:state.excludedValues) {
        if(id>=state.values.size() || std::find(state.leaves.begin(),state.leaves.end(),id)==state.leaves.end()) {
            if(error) *error="excluded output is not a declared fallback leaf"; return false;
        }
        for(const auto &op:graph.ops) if(std::find(op.descriptor.writes.begin(),op.descriptor.writes.end(),id)!=op.descriptor.writes.end()) {
            if(error) *error="excluded output has an active producer"; return false;
        }
    }
    for(const auto &row:wire.readers) graph.readers.emplace(row.value,row.ops);
    for(const auto &row:wire.cycles) graph.cycles.push_back(row.keys);
    graph.opClusters=wire.opClusters;
    for(const auto &row:wire.clusters)
        graph.clusters.push_back({row.members,row.predecessors,row.successors});
    if(!RigExecValidateOpClusters(graph,error)) return false;
    RigExecCompiledGraph checked;
    if(!RigExecCompileOpGraph(descriptors,state.leaves,RigExecCyclePolicy::Reject,&checked,error)) return false;
    if(checked.ops.size()!=graph.ops.size() || checked.longestPath!=graph.longestPath ||
       checked.readers!=graph.readers || checked.canonicalIndex!=graph.canonicalIndex) {
        if(error) *error="common graph differs from its producer declarations"; return false;
    }
    for(size_t i=0;i<graph.ops.size();++i) {
        if(checked.ops[i].originalIndex!=i || checked.ops[i].predecessors!=graph.ops[i].predecessors ||
           checked.ops[i].successors!=graph.ops[i].successors || graph.ops[i].originalIndex!=i) {
            if(error) *error="common graph is not the canonical producer order"; return false;
        }
    }
    // The checked compile has these exact canonical ops and descriptors.
    graph.volatileOps=checked.volatileOps;
    state.compiled=true; p->opGraph=std::move(graph); p->store.opAdapter=std::move(state);
    _RrIndexSkipEffects(p);
    return true;
}

bool RrCompileOpGraph(RrProgram *p,std::string *error)
{
    if(!RrCompileCommonGraph(p,error)) return false;
    // Wire steps are immutable after Open: one classification serves every run.
    auto &state=p->store.opAdapter;
    state.constantSource.assign(p->opGraph.ops.size(),0);
    state.sourceVisits.clear();
    for(uint32_t c=0;c<p->opGraph.ops.size();++c) {
        const auto &step=(*p->steps)[p->opGraph.ops[c].originalIndex];
        state.constantSource[c]=_RrInputMemoIsConstant(step)?1:0;
        if(!state.constantSource[c] || step.headAlwaysRuns) state.sourceVisits.push_back(c);
    }
    return true;
}

bool RrExecuteOpGraph(RrProgram *p,bool force,std::string *error)
{
    auto &s=p->store; auto &state=s.opAdapter;
    s.lastClosedClusters=0;
    if(!state.compiled && !RrCompileOpGraph(p,error)) return false;
    force=force || p->requiredStageFramesFullOwed;
    const bool first=!state.everRan;
    state.changedLeaves.clear(); state.seeds.clear(); state.candidateOps.clear();
    state.inputKeys.resize(p->opGraph.ops.size()); state.inputScratch.resize(p->opGraph.ops.size());
    state.coveredPropertyInputs.resize(p->opGraph.ops.size());
    state.coveredTypedInputs.resize(p->opGraph.ops.size());
    s.headLines.resize(p->steps->size()); s.headMemoKeys.resize(p->steps->size());
    s.opInputScratch.resize(p->steps->size());
    s.weightFieldChanged.assign(p->geometry->weightFields.size(),0);
    s.restChanged.assign(p->slotMeta->paths.size(),0); s.ladderChanged.assign(p->slotMeta->paths.size(),0);
    s.topologyChanged.assign(p->geometry->revisionIndex.size()+p->geometry->derivedIndex.size(),0);
    RrPropertyBegin(p);
    for(auto id:state.excludedValues) {
        const auto &value=state.values[size_t(id)]; RrResetExcludedValue(p,value.domain,value.slot);
    }
    RigExecOpClearChanges(&state);
    const auto sample=[&](uint32_t d,uint32_t slot,std::string *key){RrOpValue(p,d,slot,key);};
    for(const auto id:state.leaves) { RigExecOpPublishValue(&state.values[size_t(id)],sample);
        if(state.values[size_t(id)].changed) state.changedLeaves.push_back(id); }
    // A constant source memo is empty on every run, which an equal compare
    // reports unchanged. It is built on a first run; a later run visits only
    // the steps that can seed or change.
    const bool verify=p->verifyConstantSources;
    const bool verifyVersions=p->verifyLeafVersions;
    if(verifyVersions) s.headContentKeys.resize(p->steps->size());
    int64_t moved=-1, disagreed=-1;
    const auto source=[&](uint32_t c) {
        const auto i=p->opGraph.ops[c].originalIndex; const auto &step=(*p->steps)[i];
        bool seed=first || step.headAlwaysRuns, changed=false;
        auto &input=s.headMemoKeys[i]; auto &scratch=s.opInputScratch[i];
        if(first || c>=state.constantSource.size() || !state.constantSource[c]) {
            scratch.clear(); _RrInputMemo(p,step,&scratch); changed=input!=scratch; input.swap(scratch);
            if(verifyVersions) {
                std::string content; _RrInputMemo(p,step,&content,true);
                if(!first && changed!=(s.headContentKeys[i]!=content) && disagreed<0) disagreed=int64_t(i);
                s.headContentKeys[i].swap(content);
            }
        } else if(verify) {
            scratch.clear(); _RrInputMemo(p,step,&scratch);
            if(scratch!=input && moved<0) moved=int64_t(i);
        }
        if(seed) state.seeds.push_back(c);
        else if(changed) state.candidateOps.push_back(c);
    };
    if(first || verify || state.constantSource.size()!=p->opGraph.ops.size())
        for(uint32_t c=0;c<p->opGraph.ops.size();++c) source(c);
    else for(const uint32_t c:state.sourceVisits) source(c);
    if(moved>=0) {
        if(error) *error="constant source memo of step "+RrStepLabel(*p,size_t(moved))+" changed";
        state.everRan=false; s.everRan=false;
        return false;
    }
    if(disagreed>=0) {
        if(error) *error="source memo of step "+RrStepLabel(*p,size_t(disagreed))+
            " moved otherwise than its array contents";
        state.everRan=false; s.everRan=false;
        return false;
    }
    RigExecOpCallbacks callbacks;
    callbacks.changed=[&](RigExecValueId id){return state.values[size_t(id)].changed!=0;};
    callbacks.inputChanged=[&](uint32_t c,RigExecValueId id){
        const auto &v=state.values[size_t(id)]; const auto &step=(*p->steps)[p->opGraph.ops[c].originalIndex];
        return v.changed && (v.domain!=uint32_t(RigExecWireSlotDomain::PropertyResult) || !_RrShadowed(s,step,v.slot));
    };
    callbacks.inputsChanged=[&](uint32_t c) {
        const auto i=p->opGraph.ops[c].originalIndex; const auto &step=(*p->steps)[i];
        auto &input=state.inputKeys[c]; auto &scratch=state.inputScratch[c]; scratch.clear();
        auto &covered=state.coveredPropertyInputs[c]; covered.clear();
        auto &coveredTyped=state.coveredTypedInputs[c]; coveredTyped.clear();
        const bool scalarConsumer=step.kind==RigExecWireStepKind::PropertyRevision ||
            step.kind==RigExecWireStepKind::AvarInputs ||
            step.kind==RigExecWireStepKind::ComposeSubtree;
        const bool exact=RrEffectiveInputMemo(p,i,&scratch,&covered,
            scalarConsumer?&coveredTyped:nullptr);
        std::sort(covered.begin(),covered.end());
        std::sort(coveredTyped.begin(),coveredTyped.end());
        for(auto id:p->opGraph.ops[c].descriptor.reads) {
            const auto &value=state.values[size_t(id)];
            if(exact && step.kind==RigExecWireStepKind::WeightField) continue;
            if(exact && step.kind==RigExecWireStepKind::ProviderRefresh &&
               value.domain==uint32_t(RigExecWireSlotDomain::SpaceLeaf)) continue;
            const bool incoming=step.kind==RigExecWireStepKind::PropertyRevision && step.part>0 &&
                value.domain==uint32_t(RigExecWireSlotDomain::PropertyResult) &&
                value.slot==uint32_t(p->file->propertyChains[size_t(step.object)].versionBase)+uint32_t(step.part-1);
            if(exact && !incoming && std::binary_search(coveredTyped.begin(),coveredTyped.end(),
                std::make_pair(value.domain,value.slot))) continue;
            if(exact && step.kind==RigExecWireStepKind::SpaceExpression &&
               ((step.part==0 && value.domain==uint32_t(RigExecWireSlotDomain::SpaceValue)) ||
                (step.part==2 && value.domain==uint32_t(RigExecWireSlotDomain::SpaceLeaf)))) continue;
            if(!incoming && value.domain==uint32_t(RigExecWireSlotDomain::PropertyResult) &&
               (std::binary_search(covered.begin(),covered.end(),value.slot) ||
                _RrShadowed(s,step,value.slot))) continue;
            RigExecOpKeyAppend(&scratch,id); RigExecOpKeyAppend(&scratch,value.revision);
        }
        const bool changed=!exact || input!=scratch; input.swap(scratch);
        return changed;
    };
    // The runtime executor is serial (no dispatch), so bodies run in
    // completion order and a cluster's ran ops are consecutive.
    s.runTrace.clear();
    size_t closedClusters=0; uint32_t lastCluster=UINT32_MAX;
    callbacks.run=[&](uint32_t c){
        const auto i=p->opGraph.ops[c].originalIndex;
        s.runTrace.push_back(int32_t(i));
        if(p->opGraph.opClusters[c]!=lastCluster) { lastCluster=p->opGraph.opClusters[c]; ++closedClusters; }
        if(!RrRunOpBody(p,i,error)) return false;
        // ChainDirty's key is its revision's RevisionDone key. Writes run in
        // (domain, slot) order, so the fuse has built that key already.
        const RigExecOpValueState *done=nullptr;
        for(auto id:p->opGraph.ops[c].descriptor.writes) {
            auto &v=state.values[size_t(id)];
            if(done && v.slot==done->slot && v.domain==uint32_t(RigExecWireSlotDomain::ChainDirty))
                RigExecOpPublishValue(&v,[&](uint32_t,uint32_t,std::string *key){key->append(done->key);});
            else RigExecOpPublishValue(&v,sample);
            if(v.domain==uint32_t(RigExecWireSlotDomain::RevisionDone)) done=&v;
            if(v.domain==uint32_t(RigExecWireSlotDomain::PropertyResult)) s.propertyVersionChanged[v.slot]=v.changed;
            if(v.domain==uint32_t(RigExecWireSlotDomain::WeightField)) s.weightFieldChanged[v.slot]=v.changed;
        }
        return true;
    };
    callbacks.skip=[&](uint32_t c){const auto i=p->opGraph.ops[c].originalIndex;
        s.stepOutputs[i].MarkSkipped(); if(_RrIsGeometryKind((*p->steps)[i].kind)) RrSkipGeometryStep(p,i);};
    callbacks.skipEffects=&state.skipEffects;
    std::string graphError;
    const bool ok=RigExecExecuteOpGraph(p->opGraph,state.changedLeaves,state.seeds,force,callbacks,&s.opExecution,&graphError,&s.opWorkspace,&state.candidateOps);
    RigExecOpGatherChanges(&state,p->opGraph,s.opExecution.ran);
    if(!ok) {
        s.runTrace.clear();
        state.everRan=false; s.everRan=false;
        if(error && error->empty()) *error=graphError;
        return false;
    }
    s.lastClosedClusters=closedClusters;
    RrPropertyPublish(p);
    // Refusal finishes preparation but owes a full pass on the next Execute.
    if(!p->requiredStageFramesAdmission.admitted) {
        state.everRan=true;
        p->requiredStageFramesFullOwed=true;
        return true;
    }
    p->requiredStageFramesFullOwed=false;
    state.everRan=true; s.everRan=true; s.animatedTouched=false;
    std::fill(s.changedSinceRun.begin(),s.changedSinceRun.end(),char(0)); s.anyChangedSinceRun=false;
    return true;
}

bool RrRunSteps(RrProgram *p,bool force,std::string *error)
{ return RrExecuteOpGraph(p,force,error); }

} // namespace rigExec
