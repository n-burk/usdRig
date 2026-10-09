#include "rigExec/weightField.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/crossDomainInputs.h"
#include "rigExecGraph/weightProgram.h"
#include <pxr/usd/usdGeom/pointBased.h>
#include <pxr/usd/sdf/types.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <set>

PXR_NAMESPACE_USING_DIRECTIVE
namespace rigExec {
namespace {
using Baked = RigExecBakedProgramImpl;
using Field = Baked::WeightField;
constexpr const char *ScalarNames[] = {
    "rigExec:defaultWeight", "inputs:driver", "inputs:scale", "inputs:bias",
    "inputs:strength", "inputs:invert", "inputs:falloffMin", "inputs:falloffMax",
    "inputs:scaleX", "inputs:scaleY", "inputs:scaleZ", "inputs:scaleXPos",
    "inputs:scaleYPos", "inputs:scaleZPos", "inputs:scaleXNeg", "inputs:scaleYNeg",
    "inputs:scaleZNeg", "inputs:extentU", "inputs:extentV"
};
SdfPath PointsPath(const UsdStageRefPtr &stage, const UsdPrim &prim, const char *name)
{
    SdfPathVector paths;
    const auto rel = prim.GetRelationship(TfToken(name));
    if (!rel || !rel.GetTargets(&paths) || paths.size() != 1) return SdfPath();
    if (paths[0].IsPrimPath()) {
        const auto target = stage->GetPrimAtPath(paths[0]);
        return UsdGeomPointBased(target) ? paths[0].AppendProperty(TfToken("points")) : SdfPath();
    }
    return paths[0];
}
float Scalar(const Baked &B, const Field &field, int object, int member, float fallback)
{
    const size_t key = size_t(object) * 19 + size_t(member);
    if (object >= 0 && member >= 0 && member < 19 && key < field.scalarReadIndex.size()) {
        const int read = field.scalarReadIndex[key];
        if (read >= 0) return RigExecBakedReadOracleScalar(B,field.scalarReads[size_t(read)],
                                                       field.availableChains,fallback);
    }
    return fallback;
}

}
bool RigExecBakedBuildWeightFields(const RigExecRigEvaluator &evaluator, Baked *program, std::string *error)
{
    auto &B = *program;
    B.weightFields.clear();
    B.weightProgram.clear();
    for (auto &object : B.weightObjects) {
        RigExecBakedDescribeWeightOracle(evaluator,object.path,RigExecBakedProbeTime(B.stage),&object.oracleFacts);
        object.oracleName = object.path.GetString();
        const std::string type = object.type.GetString();
        object.oracleKind = type == "RigExecStaticWeight" ? 0 : type == "RigExecDynamicWeight" ? 1 :
            type == "RigExecCombineWeight" ? 2 : type == "RigExecSphereWeight" ? 3 :
            type == "RigExecPlaneWeight" ? 4 : type == "RigExecCurveWeight" ? 5 : -1;
        object.oracleRepresentation = object.representation == TfToken("constant") ? 0 :
            object.representation == TfToken("dense") ? 1 : object.representation == TfToken("sparse") ? 2 : -1;
        object.oracleClamp = object.rangePolicy == TfToken("clamp");
        const std::string modes[] = {"multiply","add","subtract","max","min","average","overlay"};
        for (int i=0;i<7;++i) if (object.combineMode.GetString() == modes[i]) object.oracleCombineMode = i;
        object.oracleAxisName = object.oracleFacts.planeAxis.GetString();
        object.oracleBoundsName = object.oracleFacts.planeBounds.GetString();
        object.oracleAxis = object.oracleAxisName == "x" ? 0 : object.oracleAxisName == "y" ? 1 :
            object.oracleAxisName == "z" ? 2 : -1;
        object.oracleBounded = object.oracleBoundsName == "bounded";
        object.oracleUnbounded = object.oracleBoundsName == "unbounded";
        const auto prim = B.stage->GetPrimAtPath(object.path);
        object.oracleAxisAttribute = bool(prim.GetAttribute(TfToken("rigExec:planeAxis")));
        object.oracleBoundsAttribute = bool(prim.GetAttribute(TfToken("rigExec:planeBounds")));
        object.oracleLeaves.decl.keys.clear();
        for (const char *name : {"rigExec:sampleSource","rigExec:weightTarget","rigExec:curve"}) {
            object.oracleLeaves.decl.Add({PointsPath(B.stage,prim,name),
                RigExecRevisionLeafType::Vec3fArray,RigExecRevisionLeafTime::AtTime,
                RigExecRevisionLeafFlavour::Raw,VtValue()});
        }
        for (const char *name : {"rigExec:planeAxis","rigExec:planeBounds"}) {
            object.oracleLeaves.decl.Add({object.path.AppendProperty(TfToken(name)),
                RigExecRevisionLeafType::Token,RigExecRevisionLeafTime::AtTime,
                RigExecRevisionLeafFlavour::Raw,VtValue()});
        }
        object.oracleFrozenKeys.clear();
        for (size_t k = 0; k < object.oracleLeaves.decl.keys.size(); ++k)
            object.oracleFrozenKeys.push_back(object.path.AppendProperty(
                TfToken("rigExecFrozen:oracle:leaf" + std::to_string(k))));
        RigExecBakedBindPathLeaves(B.stage,&object.oracleLeaves);
        RigExecWeightRecord record;
        record.name=object.oracleName;record.kind=object.oracleKind;
        record.representation=object.oracleRepresentation;record.combineMode=object.oracleCombineMode;
        record.type=object.type;record.representationToken=object.representation;
        record.rangePolicy=object.rangePolicy;record.combineToken=object.combineMode;
        record.planeAxis=object.planeAxis;record.planeBounds=object.planeBounds;
        record.clamp=object.oracleClamp;record.base=object.base;record.inputs=object.inputs;
        record.values=object.values;record.indices=object.indices;record.falloffCurve=object.falloffCurve;
        record.staticError=object.oracleFacts.staticError;record.phaseError=object.oracleFacts.phaseError;
        record.samplesInFlight=object.oracleFacts.samplesInFlight;
        B.weightProgram.push_back(std::move(record));
    }
    std::string placementError;
    const auto add = [&](const SdfPath &path, Field::Form form, int consumer, int part) {
        const auto found = B.weightIndex.find(path);
        if (found == B.weightIndex.end() || found->second < 0) return -1;
        Field field; field.object = found->second; field.form = form;
        field.currentInputs.resize(B.weightObjects.size());
        field.scalarReadIndex.assign(B.weightObjects.size() * 19,-1);
        field.consumer = consumer; field.part = part;
        SdfPath reader;
        if(form==Field::Form::Revision) {
            const auto [chain,revision]=B.revisionIndex[size_t(consumer)];
            reader=B.chains[size_t(chain)].revisions[size_t(revision)].moverPath;
        } else if(form==Field::Form::EnvelopeProperty)
            reader=B.propertyChains[size_t(consumer)].revisions[size_t(part-1)].mover;
        else reader=B.constraints[size_t(B.walkSteps[size_t(consumer)].index)].path;
        RigExecReadPhase placement;std::string why;
        const auto relation=B.stage->GetPrimAtPath(reader).GetRelationship(TfToken("rigExec:weightObject"));
        if(!RigExecResolveReadPhase(relation,&placement,&why) ||
            (placement.kind!=RigExecReadPhaseKind::Base && placement.kind!=RigExecReadPhaseKind::Final)) {
            placementError=reader.GetString()+": weight placement must read base or final"+
                (why.empty()?std::string():": "+why);return -1;
        }
        field.placementPhase=placement.kind==RigExecReadPhaseKind::Final?
            Field::PlacementPhase::Final:Field::PlacementPhase::Base;
        // Oracle reads observe the original generation publication boundary,
        // not every retained final in the canonical graph. Property envelopes
        // run before their own chain finishes; only earlier semantic chains
        // have published. Rest-tier consumers observe the completed head tier.
        const size_t publishedEnd = form == Field::Form::EnvelopeProperty
            ? size_t(consumer) : B.propertyChains.size();
        for (size_t c=0;c<publishedEnd;++c)
            field.availableChains.push_back(int(c));
        std::set<int> objects;
        const auto visit = [&](const auto &self,int object) -> void {
            if (object < 0 || !objects.insert(object).second) return;
            const auto &weight = B.weightObjects[size_t(object)];
            self(self,weight.base);
            for (int input : weight.inputs) self(self,input);
            if (weight.providerSlot >= 0) field.volumes.push_back(weight.providerSlot);
            const auto prim = B.stage->GetPrimAtPath(weight.path);
            for (int member=0;member<19;++member) {
                const bool used = member == 0 ||
                    (weight.oracleKind == 1 && member >= 1 && member <= 3) ||
                    (weight.oracleKind == 2 && (member == 4 || member == 5)) ||
                    (weight.oracleKind == 3 && member >= 4 && member <= 16) ||
                    (weight.oracleKind == 4 && ((member >= 4 && member <= 7) || member >= 17)) ||
                    (weight.oracleKind == 5 && member >= 4 && member <= 10);
                if (!used) continue;
                field.scalarReadIndex[size_t(object) * 19 + size_t(member)] = int(field.scalarReads.size());
                field.scalarObjects.push_back(object); field.scalarMembers.push_back(member);
                field.scalarReads.push_back(RigExecBakedBuildOracleRead(&B,
                    prim.GetAttribute(TfToken(ScalarNames[member])),field.availableChains));
            }
        };
        visit(visit,field.object);
        field.objects.assign(objects.begin(),objects.end());
        for (auto &read : field.scalarReads)
            for (auto &hop : read.walk.hops) {
                const auto pose = B.poseWeightIndex.find(hop.path);
                if (pose != B.poseWeightIndex.end()) hop.poseWeight = pose->second;
            }
        std::sort(field.volumes.begin(),field.volumes.end());
        field.volumes.erase(std::unique(field.volumes.begin(),field.volumes.end()),field.volumes.end());
        B.weightFields.push_back(std::move(field));
        return int(B.weightFields.size())-1;
    };
    for (size_t c=0;c<B.propertyChains.size();++c)
        for (size_t k=0;k<B.propertyChains[c].revisions.size();++k) {
            auto &revision = B.propertyChains[c].revisions[k];
            if (!revision.weightObject.IsEmpty())
                revision.weightField = add(revision.weightObject,Field::Form::EnvelopeProperty,int(c),int(k+1));
        }
    for (size_t w=0;w<B.walkSteps.size();++w) {
        const auto &walk = B.walkSteps[w];
        if (walk.solverBatch || walk.index < 0) continue;
        auto &constraint = B.constraints[size_t(walk.index)];
        if (!constraint.weightObject.IsEmpty() && constraint.pointsTarget.IsEmpty())
            constraint.weightField = add(constraint.weightObject,Field::Form::EnvelopeConstraint,int(w),0);
    }
    for (size_t r=0;r<B.revisionIndex.size();++r) {
        const auto [c,k] = B.revisionIndex[r];
        auto &revision = B.chains[size_t(c)].revisions[size_t(k)];
        if (revision.weightObject >= 0 && revision.weightCurrentPhase) {
            revision.weightField = add(B.weightObjects[size_t(revision.weightObject)].path,
                                       Field::Form::Revision,int(r),0);

        }
    }
    if(!placementError.empty()){if(error)*error=placementError;return false;}
    for (const auto &field : B.weightFields) {
        if (field.scalarReads.size() != field.scalarObjects.size() ||
            field.scalarReads.size() != field.scalarMembers.size()) {
            if (error) *error = "weight field scalar provenance is incomplete";
            return false;
        }
    }
    B.headOverrides.resize(B.headOverrideSlots.size());
    B.lastHeadOverrides.resize(B.headOverrideSlots.size());
    B.headOverrideMoved.resize(B.headOverrideSlots.size(),0);
    B.volumePlacementBase.assign(B.paths.size(),GfMatrix4d(1.0));
    for (size_t f=0;f<B.weightFields.size();++f) {
        const auto &field = B.weightFields[f];
        if (field.form == Field::Form::Revision) continue;
        RigExecBakedStep step;
        step.kind = RigExecBakedStepKind::WeightField; step.object = int(f);
        step.isHead = true; step.alwaysRuns = false; step.maxDiagnostics = 0;
        RigExecBakedDeclareWeightField(&B,&step);
        B.steps.push_back(std::move(step));
    }
    for (auto &step : B.steps) {
        if (step.kind != RigExecBakedStepKind::PropertyRevision || step.part <= 0) continue;
        auto &revision = B.propertyChains[size_t(step.object)].revisions[size_t(step.part-1)];
        if (revision.weightField >= 0) {
            step.reads.push_back(RigExecBakedOne(RigExecBakedSlotDomain::WeightField,revision.weightField));
            step.alwaysRuns = false;
        }
    }
    return true;
}
void RigExecBakedDeclareWeightField(Baked *program, RigExecBakedStep *step)
{
    const auto &field = program->weightFields[size_t(step->object)];
    step->writes.push_back(RigExecBakedOne(RigExecBakedSlotDomain::WeightField,step->object));
    for (const auto &read : field.scalarReads) {
        for (uint32_t version : read.versions)
            step->reads.push_back(RigExecBakedOne(RigExecBakedSlotDomain::PropertyResult,int(version)));
        step->leaves.insert(step->leaves.end(),read.leaves.begin(),read.leaves.end());
        step->overrideSlots.insert(step->overrideSlots.end(),read.slots.begin(),read.slots.end());
        for (const auto &hop : read.walk.hops)
            if (hop.poseWeight >= 0)
                step->reads.push_back(RigExecBakedOne(RigExecBakedSlotDomain::PoseWeight,hop.poseWeight));
        for (uint32_t leaf : read.leaves) if (program->headLeaves[leaf].varying) step->varyingInputs = true;
    }
    // Raw oracle arrays participate in the same numbered leaf census as
    // revision inputs, including private sampled arrays and failed reads.
    for (size_t i = 0; i < program->pathLeafRefs.size(); ++i) {
        const auto &ref = program->pathLeafRefs[i];
        if (ref.owner != RigExecBakedPathLeafOwner::WeightOracle ||
            std::find(field.scalarObjects.begin(),field.scalarObjects.end(),
                      int(ref.a)) == field.scalarObjects.end()) continue;
        const auto &leaves = program->weightObjects[ref.a].oracleLeaves;
        step->bindingLeaves.push_back(uint32_t(program->leafRefs.size() + i));
        if (ref.key < leaves.varying.size() && leaves.varying[ref.key]) {
            step->varyingInputs = true;
            step->varyingLeaves = true;
        }
    }
    std::sort(step->bindingLeaves.begin(),step->bindingLeaves.end());
    step->bindingLeaves.erase(std::unique(step->bindingLeaves.begin(),step->bindingLeaves.end()),step->bindingLeaves.end());
    if (field.form == Field::Form::Revision) {
        const auto [c,k] = program->revisionIndex[size_t(field.consumer)];
        if (k > 0) {
            const int previous = field.consumer - 1;
            step->reads.push_back(RigExecBakedOne(RigExecBakedSlotDomain::RevisionDone,previous));
            step->reads.push_back(RigExecBakedOne(RigExecBakedSlotDomain::ChainDirty,previous));
            for (int earlier = 0; earlier < k; ++earlier) {
                const int revision = program->chainRevisionBegin[size_t(c)] + earlier;
                step->reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::RevisionDone, revision));
            }
            step->reads.push_back(RigExecBakedOne(RigExecBakedSlotDomain::ChainBase,int(c)));
        } else step->reads.push_back(RigExecBakedOne(RigExecBakedSlotDomain::ChainBase,int(c)));
    }
    // Every field form consumes the selected analytic-volume placement.
    for (int slot : field.volumes)
        step->reads.push_back(RigExecBakedOne(field.placementPhase == Field::PlacementPhase::Base
            ? RigExecBakedSlotDomain::WeightFramesBase : RigExecBakedSlotDomain::WeightFrames,slot));
    // Declaration is repeated after input numbering; keep producer ranges unique.
    std::sort(step->reads.begin(),step->reads.end());
    step->reads.erase(std::unique(step->reads.begin(),step->reads.end()),step->reads.end());
    std::sort(step->writes.begin(),step->writes.end());
    step->writes.erase(std::unique(step->writes.begin(),step->writes.end()),step->writes.end());
    std::sort(step->leaves.begin(),step->leaves.end());
    step->leaves.erase(std::unique(step->leaves.begin(),step->leaves.end()),step->leaves.end());
    std::sort(step->overrideSlots.begin(),step->overrideSlots.end());
    step->overrideSlots.erase(std::unique(step->overrideSlots.begin(),step->overrideSlots.end()),step->overrideSlots.end());
}
bool RigExecBakedCaptureWeightFieldInputs(const Baked &B,int index,
    std::vector<RigExecWeightFieldInputs> *inputs,RigExecWeightPointView *entering,size_t *count)
{
    if(!inputs || !entering || !count || index<0 || size_t(index)>=B.weightFields.size())return false;
    const auto &field=B.weightFields[size_t(index)];
    *entering={};*count=1;
    bool haveContext = true;
    if (field.form == Field::Form::Revision) {
        const auto [c,k] = B.revisionIndex[size_t(field.consumer)];
        const auto &chain = B.chains[size_t(c)];
        haveContext = chain.haveBase;
        if (haveContext) {
            // The version entering the consumer as one array: in place, or
            // gathered into the field's own scratch, which only its step and
            // that step's key read, one after the other.
            const GfVec3f *points = nullptr;
            size_t pointCount = 0;
            RigExecBakedVersionPoints(chain, size_t(k),
                                      &const_cast<Field &>(field).enteringGather,
                                      &points, &pointCount);
            *entering = {points, pointCount};
        }
        *count = entering->count;
    }
    inputs->resize(B.weightObjects.size());
    // All borrowed views are from this generation's leaves or selected graph versions.
    for (int objectIndex:field.objects) {
        const size_t id=size_t(objectIndex);
        const auto &object=B.weightObjects[id];auto &input=(*inputs)[id];
        input=RigExecWeightFieldInputs{};
        input.blocked=id<B.weightCycleBlocked.size() && B.weightCycleBlocked[id];
        for (int member=0;member<19;++member)
            input.scalars[size_t(member)]=Scalar(B,field,int(id),member,input.scalars[size_t(member)]);
        for (int key=0;key<3;++key) {
            const auto chosen=RigExecBakedReadWeightPointInput(B,index,int(id),key);
            input.phasedPoints[size_t(key)]={chosen.declared,chosen.available,chosen.data,chosen.count};
            if (size_t(key)<object.oracleLeaves.values.size() &&
                object.oracleLeaves.values[size_t(key)].IsHolding<VtVec3fArray>()) {
                const auto &points=object.oracleLeaves.values[size_t(key)].UncheckedGet<VtVec3fArray>();
                input.rawPoints[size_t(key)]={false,true,points.data(),points.size()};
            }
        }
        const auto token=[&](size_t key,bool exists,const char *fallback) {
            const auto &values=object.oracleLeaves.values;
            return key<values.size() && values[key].IsHolding<TfToken>()
                ? values[key].UncheckedGet<TfToken>().GetString() : exists?std::string():std::string(fallback);
        };
        input.axis=token(3,object.oracleAxisAttribute,"x");
        input.bounds=token(4,object.oracleBoundsAttribute,"unbounded");
        const auto &placements=field.placementPhase==Field::PlacementPhase::Base
            ? B.volumePlacementBase:B.volumePlacement;
        const int slot=object.providerSlot;
        input.hasPlacement=slot>=0 && size_t(slot)<B.placedVolumes.size() &&
            B.placedVolumes[size_t(slot)] && size_t(slot)<placements.size();
        if(input.hasPlacement)input.placement=placements[size_t(slot)];
    }
    return haveContext;
}
void RigExecBakedWeightFieldCoveredVersions(const Baked &B,int index,std::vector<uint32_t> *versions)
{
    if(!versions || index<0 || size_t(index)>=B.weightFields.size())return;
    for(const auto &read:B.weightFields[size_t(index)].scalarReads)
        versions->insert(versions->end(),read.versions.begin(),read.versions.end());
}
void RigExecBakedRunWeightField(Baked *program, RigExecBakedStep *step)
{
    auto &B = *program; auto &field = B.weightFields[size_t(step->object)];
    const auto &before = field.values; const bool beforeOk = field.ok; const auto beforeError = field.error;
    const size_t beforeCount = field.count;
    RigExecWeightPointView entering;size_t count=1;
    auto &inputs=field.currentInputs;
    const bool haveContext=RigExecBakedCaptureWeightFieldInputs(B,step->object,&inputs,&entering,&count);
    const auto *current=field.form==Field::Form::Revision?&entering:nullptr;
    field.count=count;field.error.clear();
    field.ok = haveContext && RigExecRunWeightField(B.weightProgram,field.object,inputs,
        count,current,&field.workspace,&field.nextValues,&field.error);
    // Failed fields publish no partial values. Equality preserves signed-zero bits.
    if (!field.ok) field.nextValues.clear();
    const bool sameValues = before.size() == field.nextValues.size() &&
        (before.empty() || std::memcmp(before.data(),field.nextValues.data(),
                                     before.size() * sizeof(float)) == 0);
    field.changed = beforeCount != field.count || beforeOk != field.ok || beforeError != field.error || !sameValues;
    field.values.swap(field.nextValues);
}

void RigExecBakedResetWeightField(Baked *program, int index)
{
    if (index < 0 || size_t(index) >= program->weightFields.size()) return;
    auto &field = program->weightFields[size_t(index)];
    field.values.clear();
    field.nextValues.clear();
    field.count = 0;
    field.ok = false;
    field.error = "operation cycle";
    field.changed = true;
}
void RigExecBakedBindWeightCycleState(Baked *program)
{
    auto &B=*program;B.weightCycleBlocked.assign(B.weightObjects.size(),0);
    for(auto id:B.opAdapter.excludedValues) {
        if(id>=B.opAdapter.values.size())continue;
        const auto &value=B.opAdapter.values[size_t(id)];
        if(value.domain==uint32_t(RigExecBakedSlotDomain::WeightPacket) &&
            value.slot<B.weightCycleBlocked.size())B.weightCycleBlocked[value.slot]=1;
    }
}
}
