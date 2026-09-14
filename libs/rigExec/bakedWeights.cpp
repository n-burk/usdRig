//
// The baked program's weight objects: the table, at bake, and the packet
// each one publishes, per frame.
//
// A weight object is the one piece of the epoch neither domain owns -- a
// mover binds one, so does a constraint, and the same object may be bound by
// several of each -- so it lives here rather than in bakedPose.cpp or
// bakedGeometry.cpp, and both call in.
//
// The arithmetic is NOT restated here. RigExecBuildStaticWeightPacket and its
// peers in weightPackets.h are the same functions the exec computeWeightPacket
// callbacks call, so a packet cannot mean one thing on the dynamic path and
// another on this one; what this file owns is only WHICH values are handed to
// them, and when.
//
// It is deliberately NOT RigExecRigEvaluator::_ResolveWeights. That is an
// independent second implementation with its own failure modes -- it rejects
// a static field carrying time samples, which exec simply reads, and it
// reports errors as strings where exec publishes an invalid packet -- and its
// value is exactly that it was written separately.
//
// WHICH SIDE EACH CALL SITE COPIES. The dynamic path resolves a weight in
// two different ways, and every parity bug in this domain is a call site
// copying the wrong one. So it is written down once, here, and obeyed:
//
//   * a MOVER copies EXEC. Its packet is the one the computeWeightPacket
//     callbacks publish -- these builders -- placed against the volume's
//     BASE frame, with exec's validity ladder, and an invalid packet is a
//     MoverFailed pass-through rather than a diagnostic string.
//   * a CONSTRAINT copies the ORACLE. The dynamic constraint path does not
//     go through exec at all: it calls RigExecRigEvaluator::_ResolveWeights
//     and takes its error string, against the FINAL-frame placement in
//     _volumeWeightMatrices. So does the baked one, for the same value.
//   * a CURRENT-PHASE field copies the ORACLE, for the same reason: the
//     dynamic path patches the tapped packet with what _ResolveWeights
//     measured against the in-flight points.
//   * pose.weightFrames copies the walk's FINAL frames, which is what
//     _UpdateVolumePlacements publishes and what the oracle then reads.
//
// The two placements genuinely differ for a volume some constraint revises,
// and reproducing BOTH is the contract: parity is with the dynamic path as
// it stands, not with the dynamic path as it might be tidied.
//
#include "bakedProgramImpl.h"

#include "types.h"
#include "weightPackets.h"

#include "pxr/base/tf/token.h"
#include "pxr/base/vt/array.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/stage.h"

#include <string>
#include <vector>

namespace rigExec {

int
RigExecBakedBakeWeightObject(RigExecBakedBuildContext *ctx,
                             const SdfPath &path)
{
    RigExecBakedProgramImpl &B = *ctx->program;
    if (path.IsEmpty()) {
        return -1;
    }
    // Baked once however many movers and constraints bind it -- the sharing
    // exec's targeted-objects accessor gives the dynamic path for free -- and
    // the same map answers re-entrancy. The table is in dependency order, so
    // an object cannot be entered in it until its inputs are; what marks it
    // as under way meanwhile is a NEGATIVE index, and meeting one on the way
    // down is a cycle. The epoch compile diagnoses weight cycles before a
    // program is ever built, but this is the entry point Phase 3 calls and a
    // recursion that only terminates because somebody else checked first is
    // not one to leave in place.
    const auto seen = B.weightIndex.find(path);
    if (seen != B.weightIndex.end()) {
        if (seen->second < 0) {
            ctx->Refuse("weight object composition contains a cycle", path);
        }
        return seen->second;
    }
    const UsdPrim prim = B.stage->GetPrimAtPath(path);
    if (!prim) {
        // No marker left behind: there is nothing to recurse into, so a
        // second bind of the same missing prim should say so again rather
        // than be reported as a cycle.
        ctx->Refuse("weight object prim is missing", path);
        return -1;
    }
    B.weightIndex[path] = -1;

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
    object.operation = ctx->ReadToken(prim, "rigExec:operation", "");
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
    static const TfToken sphereType("RigExecSphereWeight");
    static const TfToken planeType("RigExecPlaneWeight");
    static const TfToken curveType("RigExecCurveWeight");
    const bool volumetric = object.type == sphereType ||
                            object.type == planeType ||
                            object.type == curveType;
    if (volumetric) {
        object.providerSlot = ctx->SlotOf(path);
        if (object.providerSlot < 0) {
            ctx->Refuse("volume weight is not a pose provider", path);
        }
        object.falloffMin = ctx->Bind(prim, "inputs:falloffMin", 0.0f);
        object.falloffMax = ctx->Bind(prim, "inputs:falloffMax", 1.0f);
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
        // The points, as the properties exec would have reached: authored
        // target order, nothing inferred. See WeightObject::targetPoints.
        const auto pointsOf = [&](const char *name) {
            std::vector<UsdAttribute> out;
            for (const SdfPath &target : ctx->Targets(prim, name)) {
                if (!target.IsPropertyPath()) {
                    continue;
                }
                B.named.insert(target);
                B.prims.insert(target.GetPrimPath());
                if (const UsdAttribute a =
                        B.stage->GetAttributeAtPath(target)) {
                    out.push_back(a);
                }
            }
            return out;
        };
        object.targetPoints = pointsOf("rigExec:weightTarget");
        object.samplePoints = pointsOf("rigExec:sampleSource");
        object.curvePoints = pointsOf("rigExec:curve");
    } else if (object.type == "RigExecCombineWeight") {
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

    object.varying = object.defaultWeight.varying || object.driver.varying ||
                     object.scale.varying || object.bias.varying ||
                     object.strength.varying || object.invert.varying ||
                     object.falloffMin.varying || object.falloffMax.varying ||
                     object.scaleX.varying || object.scaleY.varying ||
                     object.scaleZ.varying || object.extentU.varying ||
                     object.extentV.varying;
    // A composed object moves when anything it composes moves, and the
    // dependency order above guarantees those are already decided.
    if (object.base >= 0 && B.weightObjects[size_t(object.base)].varying) {
        object.varying = true;
    }
    for (const int input : object.inputs) {
        if (B.weightObjects[size_t(input)].varying) {
            object.varying = true;
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

    const int index = int(B.weightObjects.size());
    B.weightIndex[path] = index;  // replaces the under-way marker
    B.weightObjects.push_back(std::move(object));
    return index;
}

RigExecWeightPacket
RigExecBakedWeightPacket(const RigExecBakedProgramImpl &program,
                         const RigExecBakedProgramImpl::WeightObject &object,
                         const std::vector<RigExecWeightPacket> &packets,
                         UsdTimeCode time)
{
    const RigExecBakedProgramImpl &B = program;
    const auto rd = [&](const auto &input) {
        return RigExecBakedRead(input, *B.resolvedInputs, time, &B.overridden);
    };
    if (object.type == "RigExecStaticWeight") {
        RigExecStaticWeightInputs inputs;
        inputs.representation = object.representation;
        inputs.rangePolicy = object.rangePolicy;
        inputs.values = object.values;
        inputs.indices = object.indices;
        inputs.defaultWeight = rd(object.defaultWeight);
        return RigExecBuildStaticWeightPacket(inputs);
    }
    if (object.type == "RigExecDynamicWeight") {
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
        return RigExecBuildDynamicWeightPacket(inputs, base);
    }
    if (object.type == "RigExecCombineWeight") {
        std::vector<RigExecWeightPacket> inputs;
        inputs.reserve(object.inputs.size());
        for (const int input : object.inputs) {
            inputs.push_back(packets[size_t(input)]);
        }
        // The cardinality fallback, and the reason it is a SIZE and not an
        // array: a combine consults its own weight target only when every
        // input is constant and none of them can say how many elements the
        // field has.
        size_t targetCount = 0;
        for (const UsdAttribute &a : object.combineTargetPoints) {
            VtVec3fArray value;
            if (B.resolvedInputs->GetAttribute(a, time, &value)) {
                targetCount += value.size();
            }
        }
        return RigExecBuildCombineWeightPacket(
            object.representation, object.rangePolicy, object.combineMode,
            inputs, targetCount, rd(object.strength), rd(object.invert));
    }
    static const TfToken sphereType("RigExecSphereWeight");
    static const TfToken planeType("RigExecPlaneWeight");
    static const TfToken curveType("RigExecCurveWeight");
    if (object.type == sphereType || object.type == planeType ||
        object.type == curveType) {
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
        inputs.params.curve = object.falloffCurve;
        if (object.type != planeType) {
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
        const auto gather = [&](const std::vector<UsdAttribute> &attributes,
                                std::vector<GfVec3f> *out) {
            for (const UsdAttribute &a : attributes) {
                VtVec3fArray value;
                if (B.resolvedInputs->GetAttribute(a, time, &value)) {
                    out->insert(out->end(), value.begin(), value.end());
                }
            }
        };
        if (RigExecVolumeWeightCanBuild(object.type, inputs)) {
            gather(object.targetPoints, &inputs.targetPoints);
            gather(object.samplePoints, &inputs.samplePoints);
            if (object.type == curveType) {
                gather(object.curvePoints, &inputs.curvePoints);
            }
        }
        return RigExecBuildVolumeWeightPacket(object.type, inputs);
    }
    // RigExecCurvenetWeight is the one weight object left: its field comes
    // off a curvenet bind, which the program does not hold. IsBakeable
    // refuses it by name, so nothing reaches this -- and an unknown type is
    // an invalid packet, which is a MoverFailed pass-through rather than a
    // plausible wrong deformation.
    return RigExecWeightPacket();
}

void
RigExecBakedNoteWeightInputs(
    const RigExecBakedProgramImpl::WeightObject &weight,
    RigExecBakedStep *step)
{
    // This object's OWN inputs only. Every object it composes has a step of
    // its own, and that step declares its own inputs; a packet moves when
    // any of them does, and the cone carries it forward along the
    // WeightPacket edges between them.
    RigExecBakedNoteInput(weight.defaultWeight, step);
    RigExecBakedNoteInput(weight.driver, step);
    RigExecBakedNoteInput(weight.scale, step);
    RigExecBakedNoteInput(weight.bias, step);
    RigExecBakedNoteInput(weight.strength, step);
    RigExecBakedNoteInput(weight.invert, step);
    RigExecBakedNoteInput(weight.falloffMin, step);
    RigExecBakedNoteInput(weight.falloffMax, step);
    RigExecBakedNoteInput(weight.scaleX, step);
    RigExecBakedNoteInput(weight.scaleY, step);
    RigExecBakedNoteInput(weight.scaleZ, step);
    RigExecBakedNoteInput(weight.extentU, step);
    RigExecBakedNoteInput(weight.extentV, step);
    // The point arrays a volume measures, which no RigExecBakedInput covers:
    // they are read through the generation's resolved inputs every frame, so
    // a property chain or a drag on the weighted mesh reaches this step the
    // same way it reaches a geometry mover.
    for (const std::vector<UsdAttribute> *points :
             {&weight.targetPoints, &weight.samplePoints, &weight.curvePoints,
              &weight.combineTargetPoints}) {
        if (!points->empty()) {
            step->resolvedInputReads = true;
            for (const UsdAttribute &a : *points) {
                step->varyingInputs =
                    step->varyingInputs || a.ValueMightBeTimeVarying();
            }
        }
    }
}

void
RigExecBakedBuildWeightSteps(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
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
                RigExecBakedSlotDomain::PoseBase, weight.providerSlot));
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
    const RigExecBakedProgramImpl::WeightObject &weight =
        B.weightObjects[size_t(step->object)];
    // Rebuilt every frame rather than replayed out of `cached`.
    //
    // The table carries a cache and the frame path does not use it yet, and
    // that is a decision rather than an omission: the predicate is the hard
    // part. `varying` says no input of the closure is a function of TIME,
    // which is not the same as "nothing can have moved" -- an interactive
    // override standing on a painted weight moves it, and so does the frame
    // AFTER one is lifted, and a volume's placement moves with the rig
    // whatever its own inputs do. A packet built from values that did not
    // move is the same packet, so the only thing at stake here is the work,
    // and the only object for which that work is more than a few floats is a
    // volume field over a large mesh -- which is also the object the
    // predicate cannot cover. Sharing the packet between the movers that
    // bind it, which is what exec's node cache buys and what this step IS,
    // is the part that mattered.
    B.weightPackets[size_t(step->object)] =
        RigExecBakedWeightPacket(B, weight, B.weightPackets, time);
}

}  // namespace rigExec
