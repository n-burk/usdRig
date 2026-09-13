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
// Nothing here runs yet: IsBakeable still refuses "weight object on mover"
// and "weight object on constraint", so no rig reaches the table. It is here
// because the operator groups that remove those refusals would otherwise each
// write it.
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
    // Memoized BEFORE the composition walk below, so an object bound by ten
    // movers is baked once -- which is the sharing exec's targeted-objects
    // accessor gives the dynamic path for free.
    const auto seen = B.weightIndex.find(path);
    if (seen != B.weightIndex.end()) {
        return seen->second;
    }
    const UsdPrim prim = B.stage->GetPrimAtPath(path);
    if (!prim) {
        ctx->Refuse("weight object prim is missing", path);
        return -1;
    }

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
    B.weightIndex[path] = index;
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

}  // namespace rigExec
