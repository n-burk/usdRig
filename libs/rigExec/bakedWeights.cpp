// Weight objects publish one shared packet per generation. Ordinary mover
// packets use the BASE pose placement and packet validity policy. Current
// point fields and scalar envelopes use the diagnostic field policy and their
// explicitly bound placement phase. Both source adapters use weightProgram;
// weightReference retains independent scalar reference arithmetic.
#include "bakedProgramImpl.h"

#include "types.h"
#include "weightPackets.h"

#include "pxr/base/tf/staticTokens.h"
#include "pxr/base/tf/token.h"
#include "pxr/base/tf/type.h"
#include "pxr/base/vt/array.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/stage.h"

#include <string>
#include <vector>

// The weight object types, spelled once. The per-frame dispatch runs a
// step body down this list, and TfToken(const char *) takes the token
// registry's spin lock on every construction -- which a step body may not
// take (docs/specs/baked-step-graph.md section 2) -- so the comparison is against
// interned tokens rather than against literals.
TF_DEFINE_PRIVATE_TOKENS(
    _tokens,
    ((staticWeight, "RigExecStaticWeight"))
    ((dynamicWeight, "RigExecDynamicWeight"))
    ((combineWeight, "RigExecCombineWeight"))
    ((sphereWeight, "RigExecSphereWeight"))
    ((planeWeight, "RigExecPlaneWeight"))
    ((curveWeight, "RigExecCurveWeight"))
);

namespace rigExec {

void
RigExecBakedWeightsTouchTokens()
{
    (void)_tokens.Get();
}

int
RigExecBakedBakeWeightObject(RigExecBakedBuildContext *ctx,
                             const SdfPath &path)
{
    RigExecBakedProgramImpl &B = *ctx->program;
    if (path.IsEmpty()) {
        return -1;
    }
    // Allocate identity on entry so authored back-edges survive discovery.
    // Canonical graph compilation owns SCC diagnosis and exclusion.
    const auto seen=B.weightIndex.find(path);
    if(seen!=B.weightIndex.end())return seen->second;
    const UsdPrim prim = B.stage->GetPrimAtPath(path);
    if (!prim) {
        // No marker left behind: there is nothing to recurse into, so a
        // second bind of the same missing prim should say so again rather
        // than be reported as a cycle.
        ctx->Refuse("weight object prim is missing", path);
        return -1;
    }
    const int index=int(B.weightObjects.size());
    B.weightIndex[path]=index;
    B.weightObjects.emplace_back();

    RigExecBakedProgramImpl::WeightObject object;
    object.path = path;
    object.type = prim.GetTypeName();

    // Composition first, and depth first: the table is in dependency order,
    // so everything this object reads has to be in it already. The authored
    // order of rigExec:inputWeights is kept -- subtract and overlay are
    // order dependent by design.
    object.base =
        RigExecBakedBakeWeightObject(ctx, [&]() {
            const SdfPathVector targets =
                ctx->Targets(prim, "rigExec:baseWeight");
            return targets.empty() ? SdfPath() : targets[0];
        }());
    for (const SdfPath &input : ctx->Targets(prim, "rigExec:inputWeights")) {
        const int index = RigExecBakedBakeWeightObject(ctx, input);
        if (index >= 0) {
            object.inputs.push_back(index);
        }
    }

    // The structural tokens are read the way the dynamic path reads them --
    // plainly, with no time and no resolution walk -- and land in `folded`,
    // so an edit to one rebuilds the program rather than being missed.
    object.representation = ctx->ReadToken(prim, "rigExec:representation",
                                           "constant");
    object.rangePolicy = ctx->ReadToken(prim, "rigExec:rangePolicy", "strict");
    object.combineMode = ctx->ReadToken(prim, "rigExec:combineMode", "");

    // The painted arrays. Uniform by schema, so they fold; still named, so a
    // resync that authors one is seen.
    ctx->Fold(prim, "rigExec:values");
    if (const UsdAttribute a = prim.GetAttribute(TfToken("rigExec:values"))) {
        VtFloatArray values;
        a.Get(&values);
        object.values.assign(values.begin(), values.end());
    }
    ctx->Fold(prim, "rigExec:indices");
    if (const UsdAttribute a = prim.GetAttribute(TfToken("rigExec:indices"))) {
        VtIntArray indices;
        a.Get(&indices);
        object.indices.assign(indices.begin(), indices.end());
    }

    // The per-frame inputs. Every one goes through Bind, which registers it
    // for interactive-override placement and records what a notice would
    // have to touch -- that is what makes IsInvalidatedBy and SetOverrides
    // correct here with no code of their own.
    object.defaultWeight = ctx->Bind(prim, "rigExec:defaultWeight", 0.0f);
    object.driver = ctx->Bind(prim, "inputs:driver", 1.0f);
    object.scale = ctx->Bind(prim, "inputs:scale", 1.0f);
    object.bias = ctx->Bind(prim, "inputs:bias", 0.0f);
    object.strength = ctx->Bind(prim, "inputs:strength", 1.0f);
    object.invert = ctx->Bind(prim, "inputs:invert", 0.0f);

    // The volumetric three, which are the only weight objects with a
    // PLACEMENT: they are RigExecXformables, so the pose walk composes them
    // into a provider slot like a joint, and the field is generated about
    // that frame.
    // The array-bearing relationships, as the properties exec would have
    // reached: authored target order, nothing inferred. See
    // WeightObject::targetPoints for why authored and not canonicalized.
    const auto attributesOf = [&](const char *name) {
        std::vector<UsdAttribute> out;
        for (const SdfPath &target : ctx->Targets(prim, name)) {
            if (!target.IsPropertyPath()) {
                continue;
            }
            B.named.insert(target);
            B.prims.insert(target.GetPrimPath());
            if (const UsdAttribute a = B.stage->GetAttributeAtPath(target)) {
                // A terminal opinion of another type cannot contribute to a
                // typed point gather. Keep connected walks and missing values
                // on actual point-array attributes; raw oracle relationship
                // facts retain the target and its unavailable typed value.
                if (a.GetTypeName().GetType() != TfType::Find<VtVec3fArray>() &&
                    !a.HasAuthoredConnections()) {
                    continue;
                }
                out.push_back(a);
            }
        }
        return out;
    };

    const bool volumetric = object.type == _tokens->sphereWeight ||
                            object.type == _tokens->planeWeight ||
                            object.type == _tokens->curveWeight;
    if (volumetric) {
        object.providerSlot = ctx->SlotOf(path);
        if (object.providerSlot < 0) {
            ctx->Refuse("volume weight is not a pose provider", path);
        }
        object.falloffMin = ctx->Bind(prim, "inputs:falloffMin", 0.0f);
        object.falloffMax = ctx->Bind(prim, "inputs:falloffMax", 1.0f);
        object.scaleXPos = ctx->Bind(prim, "inputs:scaleXPos", 1.0f);
        object.scaleYPos = ctx->Bind(prim, "inputs:scaleYPos", 1.0f);
        object.scaleZPos = ctx->Bind(prim, "inputs:scaleZPos", 1.0f);
        object.scaleXNeg = ctx->Bind(prim, "inputs:scaleXNeg", 1.0f);
        object.scaleYNeg = ctx->Bind(prim, "inputs:scaleYNeg", 1.0f);
        object.scaleZNeg = ctx->Bind(prim, "inputs:scaleZNeg", 1.0f);
        object.scaleX = ctx->Bind(prim, "inputs:scaleX", 1.0f);
        object.scaleY = ctx->Bind(prim, "inputs:scaleY", 1.0f);
        object.scaleZ = ctx->Bind(prim, "inputs:scaleZ", 1.0f);
        object.extentU = ctx->Bind(prim, "inputs:extentU", 1.0f);
        object.extentV = ctx->Bind(prim, "inputs:extentV", 1.0f);
        object.planeAxis = ctx->ReadToken(prim, "rigExec:planeAxis", "y");
        object.planeBounds =
            ctx->ReadToken(prim, "rigExec:planeBounds", "unbounded");
        // The curve the remap was resampled from. Folded, not bound: there
        // is no rebaking a lookup table mid-drag, so an override on it
        // correctly reports NOT placeable and forces the dynamic path.
        ctx->Fold(prim, "rigExec:falloffProfile");
        ctx->Fold(prim, "rigExec:falloffCurve");
        const auto lut = B.falloffLuts.find(path);
        if (lut != B.falloffLuts.end()) {
            object.falloffCurve = lut->second;
        }
        object.targetPoints = attributesOf("rigExec:weightTarget");
        object.samplePoints = attributesOf("rigExec:sampleSource");
        object.curvePoints = attributesOf("rigExec:curve");
    } else if (object.type == _tokens->combineWeight) {
        // Read for its SIZE and never for its points: the combine consults
        // its own weight target only when no input is dense enough to carry
        // the cardinality itself.
        for (const SdfPath &target :
                 ctx->Targets(prim, "rigExec:weightTarget")) {
            if (!target.IsPropertyPath()) {
                continue;
            }
            B.named.insert(target);
            B.prims.insert(target.GetPrimPath());
            if (const UsdAttribute a = B.stage->GetAttributeAtPath(target)) {
                object.combineTargetPoints.push_back(a);
            }
        }
    }

    // What the schedule sizes this step by. The points if the object reads
    // any, else the painted table, else one element.
    object.costElements = std::max<size_t>(object.values.size(), 1);
    for (const std::vector<UsdAttribute> *points :
             {&object.targetPoints, &object.combineTargetPoints}) {
        for (const UsdAttribute &a : *points) {
            VtVec3fArray value;
            if (a.Get(&value, UsdTimeCode::EarliestTime())) {
                object.costElements =
                    std::max(object.costElements, value.size());
            }
        }
    }

    // The point gathers' reads as path leaves, one per attribute in gather
    // order, sampled in the prologue through the generation's resolved
    // inputs: the packet step reads them instead of the stage.
    const auto declare = [&object](const std::vector<UsdAttribute> &points) {
        const size_t begin = object.pointLeaves.decl.keys.size();
        for (const UsdAttribute &a : points) {
            object.pointLeaves.decl.Add(
                {a.GetPath(), RigExecRevisionLeafType::Vec3fArray,
                 RigExecRevisionLeafTime::AtTime,
                 RigExecRevisionLeafFlavour::ResolvedOnly,
                 VtValue(VtVec3fArray())});
        }
        return begin;
    };
    object.targetLeaves = declare(object.targetPoints);
    object.sampleLeaves = declare(object.samplePoints);
    object.curveLeaves = declare(object.curvePoints);
    object.combineLeaves = declare(object.combineTargetPoints);
    RigExecBakedBindPathLeaves(B.stage, &object.pointLeaves);

    B.weightObjects[size_t(index)]=std::move(object);
    return index;
}

RigExecWeightPacket
RigExecBakedWeightPacket(const RigExecBakedProgramImpl &program,
                         RigExecBakedProgramImpl::WeightObject *objectPtr,
                         const std::vector<RigExecWeightPacket> &packets,
                         UsdTimeCode /*time: the leaves are this run's*/)
{
    const RigExecBakedProgramImpl &B = program;
    RigExecBakedProgramImpl::WeightObject &object = *objectPtr;
    const auto &record=B.weightProgram[size_t(objectPtr-B.weightObjects.data())];
    // The object's inputs as the prologue sampled them.
    const auto rd = [&B](const auto &input) {
        return RigExecBakedLeafRead(B, input);
    };
    if (object.type == _tokens->staticWeight) {
        RigExecWeightPacketInputs inputs;inputs.workspace=&object.packetWorkspace;
        inputs.painted={object.representation,object.rangePolicy,object.values.data(),object.values.size(),
            object.indices.data(),object.indices.size(),rd(object.defaultWeight)};
        return RigExecRunWeightPacket(record,inputs);
    }
    if (object.type == _tokens->dynamicWeight) {
        RigExecDynamicWeightInputs inputs;
        inputs.representation = object.representation;
        inputs.rangePolicy = object.rangePolicy;
        inputs.driver = rd(object.driver);
        inputs.scale = rd(object.scale);
        inputs.bias = rd(object.bias);
        // Null where rigExec:baseWeight targets nothing, which the kernel
        // accepts only for a constant packet -- passing a default-constructed
        // packet instead would be a different answer.
        const RigExecWeightPacket *base =
            object.base >= 0 ? &packets[size_t(object.base)] : nullptr;
        RigExecWeightPacketInputs packet;packet.dynamic=std::move(inputs);packet.base=base;
        return RigExecRunWeightPacket(record,packet);
    }
    if (object.type == _tokens->combineWeight) {
        auto &inputs=object.packetInputRefs;inputs.clear();
        inputs.reserve(object.inputs.size());
        for (const int input : object.inputs) {
            inputs.push_back(&packets[size_t(input)]);
        }
        // The cardinality fallback, and the reason it is a SIZE and not an
        // array: a combine consults its own weight target only when every
        // input is constant and none of them can say how many elements the
        // field has.
        // A read that found nothing is an empty leaf, and adds nothing.
        size_t targetCount = 0;
        for (size_t i = 0; i < object.combineTargetPoints.size(); ++i) {
            targetCount += object.pointLeaves
                               .Value<VtVec3fArray>(
                                   int(object.combineLeaves + i),
                                   VtVec3fArray())
                               .size();
        }
        RigExecWeightPacketInputs packet;packet.borrowedInputs=&inputs;packet.targetCount=targetCount;packet.workspace=&object.packetWorkspace;
        packet.strength=rd(object.strength);packet.invert=rd(object.invert);
        return RigExecRunWeightPacket(record,packet);
    }
    if (object.type == _tokens->sphereWeight ||
        object.type == _tokens->planeWeight ||
        object.type == _tokens->curveWeight) {
        RigExecVolumeWeightInputs inputs;
        inputs.representation = object.representation;
        inputs.rangePolicy = object.rangePolicy;
        // The volume's BASE frame, which is what exec's computePointFrame
        // carries for every seeded provider in the authoritative snapshot.
        // NOT B.fin: a volume some constraint revises is placed for a mover
        // where it was composed and for pose.weightFrames where it ended up,
        // and reproducing both is the contract (see the head of this file).
        if (object.providerSlot >= 0) {
            // The LAST base version of the slot, which is what
            // `baseFrames.at(provider)` holds when the authoritative
            // snapshot is taken: a solver commit writes a provider's base as
            // well as its final, and the packet is built after the walk.
            inputs.placement =
                B.base[size_t(B.baseLast[size_t(object.providerSlot)])];
            inputs.hasPlacement = true;
        }
        inputs.params.falloffMin = rd(object.falloffMin);
        inputs.params.falloffMax = rd(object.falloffMax);
        inputs.params.invert = rd(object.invert);
        inputs.params.strength = rd(object.strength);
        inputs.params.curveData = object.falloffCurve.data();
        inputs.params.curveCount = object.falloffCurve.size();
        if (object.type != _tokens->planeWeight) {
            inputs.positiveScales = GfVec3f(
                rd(object.scaleXPos),
                rd(object.scaleYPos),
                rd(object.scaleZPos));
            inputs.negativeScales = GfVec3f(
                rd(object.scaleXNeg),
                rd(object.scaleYNeg),
                rd(object.scaleZNeg));
            inputs.scales = GfVec3f(rd(object.scaleX), rd(object.scaleY),
                                    rd(object.scaleZ));
        } else {
            inputs.planeAxis = object.planeAxis;
            inputs.planeBounds = object.planeBounds;
            // Only the bounded arm consults them: an unbounded plane is an
            // infinite half-space gradient, and a bad extent on one is a
            // legibility problem rather than a reason to invalidate the rig.
            if (object.planeBounds == "bounded") {
                inputs.extentU = rd(object.extentU);
                inputs.extentV = rd(object.extentV);
            }
        }
        // The points are a whole mesh, so they are gathered only once the
        // structural half has said the volume can produce a field at all --
        // which is the order the exec adapters gather them in, and the
        // reason RigExecVolumeWeightCanBuild exists.
        // A read that found nothing is an empty leaf, and contributes
        // nothing to the concatenation.
        inputs.usePointViews=true;inputs.localCurveScratch=&object.packetLocalCurve;
        const auto gather = [&object](size_t key,size_t begin,size_t count,
                                      RigExecWeightPointView *view) {
            auto &held=object.packetPointViews[key];auto &scratch=object.packetPointScratch[key];
            *view={};scratch.clear();held.clear();
            if(count==1) {
                held=object.pointLeaves.Value<VtVec3fArray>(int(begin),VtVec3fArray());
                const auto &points=held;*view={points.data(),points.size()};
            } else {
                for(size_t i=0;i<count;++i) {
                    const auto points=object.pointLeaves.Value<VtVec3fArray>(int(begin+i),VtVec3fArray());
                    scratch.insert(scratch.end(),points.begin(),points.end());
                }
                *view={scratch.data(),scratch.size()};
            }
        };
        if (RigExecVolumeWeightCanBuild(object.type, inputs)) {
            gather(1,object.targetLeaves,object.targetPoints.size(),&inputs.targetView);
            gather(0,object.sampleLeaves,object.samplePoints.size(),&inputs.sampleView);
            if(object.type==_tokens->curveWeight)
                gather(2,object.curveLeaves,object.curvePoints.size(),&inputs.curveView);
        }
        RigExecWeightPacketInputs packet;packet.volume=std::move(inputs);
        return RigExecRunWeightPacket(record,packet);
    }
    // Every weight object type the epoch can hold has an arm above.
    // IsBakeable refuses any other by name, so nothing reaches this -- and
    // an unknown type is an invalid packet, which is a MoverFailed
    // pass-through rather than a plausible wrong deformation.
    return RigExecWeightPacket();
}

void
RigExecBakedNoteWeightInputs(
    const RigExecBakedProgramImpl::WeightObject &weight,
    RigExecBakedDependencySink *sink)
{
    // This object's OWN inputs only. Every object it composes has a step of
    // its own, and that step declares its own inputs; a packet moves when
    // any of them does, and the cone carries it forward along the
    // WeightPacket edges between them.
    RigExecBakedNoteInput(weight.defaultWeight, sink);
    RigExecBakedNoteInput(weight.driver, sink);
    RigExecBakedNoteInput(weight.scale, sink);
    RigExecBakedNoteInput(weight.bias, sink);
    RigExecBakedNoteInput(weight.strength, sink);
    RigExecBakedNoteInput(weight.invert, sink);
    RigExecBakedNoteInput(weight.falloffMin, sink);
    RigExecBakedNoteInput(weight.falloffMax, sink);
    RigExecBakedNoteInput(weight.scaleXPos, sink);
    RigExecBakedNoteInput(weight.scaleYPos, sink);
    RigExecBakedNoteInput(weight.scaleZPos, sink);
    RigExecBakedNoteInput(weight.scaleXNeg, sink);
    RigExecBakedNoteInput(weight.scaleYNeg, sink);
    RigExecBakedNoteInput(weight.scaleZNeg, sink);
    RigExecBakedNoteInput(weight.scaleX, sink);
    RigExecBakedNoteInput(weight.scaleY, sink);
    RigExecBakedNoteInput(weight.scaleZ, sink);
    RigExecBakedNoteInput(weight.extentU, sink);
    RigExecBakedNoteInput(weight.extentV, sink);
    // The point arrays a volume measures, which no RigExecBakedInput covers:
    // they are read through the generation's resolved inputs every frame, so
    // a property chain or a drag on the weighted mesh reaches this step the
    // same way it reaches a geometry mover.
    for (const std::vector<UsdAttribute> *points :
             {&weight.targetPoints, &weight.samplePoints, &weight.curvePoints,
              &weight.combineTargetPoints}) {
        if (!points->empty()) {
            sink->resolvedReads = true;
            for (const UsdAttribute &a : *points) {
                sink->step->varyingInputs =
                    sink->step->varyingInputs || a.ValueMightBeTimeVarying();
            }
        }
    }
}

void
RigExecBakedBuildWeightSteps(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    // Where each volume weight ended up: one step per volume slot, writing
    // that slot of the program's volumePlacement table (WeightFrames[slot]).
    // The table has two readers. pose.weightFrames is one (the epilogue
    // publishes it); the other is the ORACLE, handed the table as a
    // RigExecVolumePlacementView, which a current-phase field resolves
    // through and which declares WeightFrames for exactly the volumes of its
    // object's closure. The geometry walk runs after every pose commit, so a
    // placement from the slot's last pose version is what every reader
    // sees. Every volume slot gets a step, bound to a mover or not: an
    // unbound volume still publishes a weightFrames entry.
    // part = 1 marks the per-volume form (object = provider slot); the
    // whole-map form (object 0, part -1) is no longer emitted.
    for (size_t i = 0; i < B.noScaleAvars.size(); ++i) {
        if (B.noScaleAvars[i] == 0) {
            continue;
        }
        RigExecBakedStep step;
        step.kind = RigExecBakedStepKind::VolumePlacements;
        step.object = int(i);
        step.part = 1;
        step.maxDiagnostics = 0;
        step.reads.push_back(
            RigExecBakedOne(RigExecBakedSlotDomain::PoseFin, B.finLast[i]));
        step.writes.push_back(
            RigExecBakedOne(RigExecBakedSlotDomain::WeightFrames, int(i)));
        B.steps.push_back(std::move(step));
        RigExecBakedStep base;
        base.kind = RigExecBakedStepKind::VolumePlacements;
        base.object = int(i);
        base.part = 2;
        base.maxDiagnostics = 0;
        base.reads.push_back(RigExecBakedOne(RigExecBakedSlotDomain::PoseBase,B.baseLast[i]));
        base.writes.push_back(RigExecBakedOne(RigExecBakedSlotDomain::WeightFramesBase,int(i)));
        B.steps.push_back(std::move(base));
    }
    // The slot storage, sized once. A consumer holds a pointer into it for
    // the whole region, so it is never resized inside one.
    B.weightPackets.assign(B.weightObjects.size(), RigExecWeightPacket());
    for (size_t i = 0; i < B.weightObjects.size(); ++i) {
        const RigExecBakedProgramImpl::WeightObject &weight =
            B.weightObjects[i];
        RigExecBakedStep step;
        step.kind = RigExecBakedStepKind::WeightPacket;
        step.object = int(i);
        step.maxDiagnostics = 0;
        // The composition, as edges. The table is in dependency order, so
        // every one of these names a lower index and the edge sweep sees a
        // forward edge like any other.
        if (weight.base >= 0) {
            step.reads.push_back(RigExecBakedOne(
                RigExecBakedSlotDomain::WeightPacket, weight.base));
        }
        for (const int input : weight.inputs) {
            step.reads.push_back(RigExecBakedOne(
                RigExecBakedSlotDomain::WeightPacket, input));
        }
        // A volume's placement is a pose frame, which makes its step a
        // reader of the pose half and so neither a source nor a pure
        // function of the stage.
        if (weight.providerSlot >= 0) {
            step.reads.push_back(RigExecBakedOne(
                RigExecBakedSlotDomain::PoseBase, B.baseLast[size_t(weight.providerSlot)]));
        }
        step.writes.push_back(
            RigExecBakedOne(RigExecBakedSlotDomain::WeightPacket, int(i)));
        B.steps.push_back(std::move(step));
    }
}

void
RigExecBakedRunWeightStep(RigExecBakedProgramImpl *program,
                          RigExecBakedStep *step, UsdTimeCode time)
{
    RigExecBakedProgramImpl &B = *program;
    if (step->kind == RigExecBakedStepKind::VolumePlacements) {
        // This step's volume slot, from the frame the walk ended with, as
        // the dynamic refresh places it. Readers mask by placedVolumes.
        const size_t slot = size_t(step->object);
        if (step->part == 2) {
            B.volumePlacementBase[slot] = RigExecVolumePlacement(B.base[size_t(B.baseLast[slot])]);
        } else {
            B.volumePlacement[slot] = RigExecVolumePlacement(B.fin[size_t(B.finLast[slot])]);
        }
        return;
    }
    RigExecBakedProgramImpl::WeightObject &weight =
        B.weightObjects[size_t(step->object)];
    // Rebuilt every frame, with NO packet carried over from the last one,
    // and that is a decision rather than an omission: the predicate is the
    // hard part. The obvious one -- "no input of this closure is a function
    // of TIME" -- is not "nothing moved": an interactive override standing
    // on a painted weight moves it, so does the frame AFTER one is lifted,
    // and a volume's placement moves with the rig whatever its own inputs
    // do. A wrong predicate here is a silently stale field, while a packet
    // rebuilt from values that did not move is the same packet, so the only
    // thing at stake is the work -- and the only object for which that work
    // is more than a few floats is a volume field over a large mesh, which
    // is also the object the tempting predicate cannot cover. The sharing
    // that did matter is what this step IS: one packet per object per
    // frame rather than per consumer, which is what exec's node cache buys.
    // If a large painted weight ever measures, the predicate to write is
    // "no override index in this closure is set now or was set last run",
    // and it is sound only for a SOURCE weight step, because
    // RigExecBakedComputeClosure rewrites lastOverridden between the source
    // pass and everything else.
    B.weightPackets[size_t(step->object)] =
        RigExecBakedWeightPacket(B, &weight, B.weightPackets, time);
}

void
RigExecBakedPublishVolumePlacements(const RigExecBakedProgramImpl &program,
                                    std::map<SdfPath, GfMatrix4d> *frames)
{
    frames->clear();
    const std::vector<char> &slots = program.placedVolumes;
    for (size_t i = 0; i < slots.size() && i < program.paths.size() &&
                       i < program.volumePlacement.size(); ++i) {
        if (slots[i]) {
            // Slots are in path order, so each entry lands at the end.
            frames->emplace_hint(frames->end(), program.paths[i],
                                 program.volumePlacement[i]);
        }
    }
}

}  // namespace rigExec
