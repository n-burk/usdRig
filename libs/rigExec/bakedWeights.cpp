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

    object.varying = object.defaultWeight.varying || object.driver.varying ||
                     object.scale.varying || object.bias.varying ||
                     object.strength.varying || object.invert.varying;
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
        return RigExecBuildCombineWeightPacket(
            object.representation, object.rangePolicy, object.combineMode,
            inputs, object.weightTargetCount, rd(object.strength),
            rd(object.invert));
    }
    // Volumetric and curvenet weights read point arrays and a posed frame
    // that the table does not carry yet; their arms arrive with the features
    // that remove their refusals, through RigExecBuildVolumeWeightPacket and
    // RigExecComputeCurvenetWeightPacket. Until then an unknown type is an
    // invalid packet, which is a MoverFailed pass-through rather than a
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
