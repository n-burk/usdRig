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
//   * a CURVENET weight copies EXEC, like any other mover binding, and is
//     the one object whose kernel had to be CUT to be copied: the exec
//     computation resolves its bind out of a process-wide LRU behind a
//     mutex, and a step body may take no lock. So the kernel is split into
//     a bind half and a field half (curvenetWeightComputations.h) and exec
//     calls both in order; the program holds a bind of its own, one per
//     weight object, and calls the field half with the same arguments.
//
// The two placements genuinely differ for a volume some constraint revises,
// and reproducing BOTH is the contract: parity is with the dynamic path as
// it stands, not with the dynamic path as it might be tidied.
//
#include "bakedProgramImpl.h"

#include "types.h"
#include "weightPackets.h"

#include "pxr/base/tf/staticTokens.h"
#include "pxr/base/tf/token.h"
#include "pxr/base/vt/array.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/stage.h"

#include <string>
#include <vector>

// The seven weight object types, spelled once. The per-frame dispatch runs a
// step body down this list, and TfToken(const char *) takes the token
// registry's spin lock on every construction -- which a step body may not
// take (docs/baked-step-graph.md section 2) -- so the comparison is against
// interned tokens rather than against literals.
TF_DEFINE_PRIVATE_TOKENS(
    _tokens,
    ((staticWeight, "RigExecStaticWeight"))
    ((dynamicWeight, "RigExecDynamicWeight"))
    ((combineWeight, "RigExecCombineWeight"))
    ((sphereWeight, "RigExecSphereWeight"))
    ((planeWeight, "RigExecPlaneWeight"))
    ((curveWeight, "RigExecCurveWeight"))
    ((curvenetWeight, "RigExecCurvenetWeight"))
);

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
    // Hoisted out of the volumetric branch because a curvenet weight reaches
    // five of its inputs the same way.
    const auto attributesOf = [&](const char *name) {
        std::vector<UsdAttribute> out;
        for (const SdfPath &target : ctx->Targets(prim, name)) {
            if (!target.IsPropertyPath()) {
                continue;
            }
            B.named.insert(target);
            B.prims.insert(target.GetPrimPath());
            if (const UsdAttribute a = B.stage->GetAttributeAtPath(target)) {
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
    } else if (object.type == _tokens->curvenetWeight) {
        object.curvenetMeshPoints = attributesOf("rigExec:weightTarget");
        object.curvenetPoints = attributesOf("rigExec:curvenetPoints");
        object.curvenetCounts = attributesOf("rigExec:meshFaceCounts");
        object.curvenetIndices = attributesOf("rigExec:meshFaceIndices");
        object.curvenetSplines = attributesOf("rigExec:curvenetSplineIndices");
        // The two ARRAYS exec reads off the prim itself. No RigExecBakedInput
        // covers an array, so they are read through the generation's resolved
        // inputs every frame -- which is also what lets an animated
        // inputs:weights move the field without re-cutting the mesh, and
        // what puts a drag on it in front of the same read.
        for (const char *name : {"inputs:weights", "rigExec:autoSmooth"}) {
            B.named.insert(path.AppendProperty(TfToken(name)));
        }
        B.prims.insert(path);
        B.resolvedRoutedPrims.insert(path);
        object.curvenetWeights = prim.GetAttribute(TfToken("inputs:weights"));
        object.curvenetAutoSmooth =
            prim.GetAttribute(TfToken("rigExec:autoSmooth"));
        // uniform by schema, so it folds; the sample count and the unreached
        // value are bound, because the schema lets both CONNECT to the
        // curvenet's own attributes and a connection is not an epoch value.
        object.curvenetBasis = ctx->ReadToken(prim, "rigExec:basis",
                                              "catmullRom");
        object.curvenetSamples = ctx->Bind(prim, "rigExec:samplesPerSpline", 5);
        object.curvenetUnreached =
            ctx->Bind(prim, "rigExec:unreachedValue", 0.0f);
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
             {&object.targetPoints, &object.combineTargetPoints,
              &object.curvenetMeshPoints}) {
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
                         RigExecBakedProgramImpl::WeightObject *objectPtr,
                         const std::vector<RigExecWeightPacket> &packets,
                         UsdTimeCode time)
{
    const RigExecBakedProgramImpl &B = program;
    RigExecBakedProgramImpl::WeightObject &object = *objectPtr;
    const auto rd = [&](const auto &input) {
        return RigExecBakedRead(input, *B.resolvedInputs, time, &B.overridden);
    };
    if (object.type == _tokens->staticWeight) {
        RigExecStaticWeightInputs inputs;
        inputs.representation = object.representation;
        inputs.rangePolicy = object.rangePolicy;
        inputs.values = object.values;
        inputs.indices = object.indices;
        inputs.defaultWeight = rd(object.defaultWeight);
        return RigExecBuildStaticWeightPacket(inputs);
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
        return RigExecBuildDynamicWeightPacket(inputs, base);
    }
    if (object.type == _tokens->combineWeight) {
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
        inputs.params.curve = object.falloffCurve;
        if (object.type != _tokens->planeWeight) {
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
            if (object.type == _tokens->curveWeight) {
                gather(object.curvePoints, &inputs.curvePoints);
            }
        }
        return RigExecBuildVolumeWeightPacket(object.type, inputs);
    }
    if (object.type == _tokens->curvenetWeight) {
        // The layout, gathered the way exec's Relationship().TargetedObjects
        // inputs are: every targeted property, concatenated in authored
        // order.
        std::vector<GfVec3f> mesh, net;
        std::vector<int> counts, indices, splines, autoSmooth;
        std::vector<float> weights;
        const auto gather = [&](const std::vector<UsdAttribute> &attributes,
                                auto *out) {
            for (const UsdAttribute &a : attributes) {
                VtArray<typename std::decay_t<decltype(*out)>::value_type>
                    value;
                if (B.resolvedInputs->GetAttribute(a, time, &value)) {
                    out->insert(out->end(), value.begin(), value.end());
                }
            }
        };
        gather(object.curvenetMeshPoints, &mesh);
        gather(object.curvenetPoints, &net);
        gather(object.curvenetCounts, &counts);
        gather(object.curvenetIndices, &indices);
        gather(object.curvenetSplines, &splines);
        const auto array = [&](const UsdAttribute &a, auto *out) {
            VtArray<typename std::decay_t<decltype(*out)>::value_type> value;
            if (a && B.resolvedInputs->GetAttribute(a, time, &value)) {
                out->assign(value.begin(), value.end());
            }
        };
        array(object.curvenetWeights, &weights);
        array(object.curvenetAutoSmooth, &autoSmooth);
        const int samples = RigExecBakedRead(object.curvenetSamples,
                                             *B.resolvedInputs, time,
                                             &B.overridden);
        // The structural check first, and the same one, so a bad token is
        // an invalid packet here exactly as it is there.
        if (!RigExecCurvenetWeightTokensAreValid(object.curvenetBasis,
                                                 object.rangePolicy)) {
            RigExecWeightPacket packet;
            packet.representation = object.representation;
            packet.rangePolicy = object.rangePolicy;
            return packet;
        }
        // The BIND, and the reason this object carries one: cutting the mesh
        // and factorizing its Laplacian is the expensive half and depends on
        // nothing but the layout. Exec keeps those in a process-wide LRU
        // behind a mutex, which a step body may not take; this is the same
        // cache with one entry, owned by this object and written by this
        // object's own step, so no two steps can be inside it at once.
        if (!object.bound || object.boundMesh != mesh ||
            object.boundNet != net ||
            object.boundCounts != counts ||
            object.boundIndices != indices ||
            object.boundSplines != splines ||
            object.boundSmooth != autoSmooth ||
            object.boundSamples != samples) {
            object.curvenetBinding = RigExecBindCurvenetWeightPacket(
                mesh, counts, indices, net, splines, object.curvenetBasis,
                samples, autoSmooth, nullptr);
            object.boundMesh = mesh;
            object.boundNet = net;
            object.boundCounts = counts;
            object.boundIndices = indices;
            object.boundSplines = splines;
            object.boundSmooth = autoSmooth;
            object.boundSamples = samples;
            object.bound = true;
        }
        if (!object.curvenetBinding) {
            // A bind that failed is an invalid packet, which is the kernel's
            // MoverFailed pass-through -- the same answer exec publishes
            // when its own bind fails.
            RigExecWeightPacket packet;
            packet.representation = object.representation;
            packet.rangePolicy = object.rangePolicy;
            return packet;
        }
        return RigExecCurvenetWeightPacketFromBinding(
            *object.curvenetBinding, weights, object.rangePolicy,
            RigExecBakedRead(object.curvenetUnreached, *B.resolvedInputs,
                             time, &B.overridden),
            nullptr);
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
    RigExecBakedNoteInput(weight.curvenetSamples, step);
    RigExecBakedNoteInput(weight.curvenetUnreached, step);
    // The point arrays a volume measures, which no RigExecBakedInput covers:
    // they are read through the generation's resolved inputs every frame, so
    // a property chain or a drag on the weighted mesh reaches this step the
    // same way it reaches a geometry mover.
    for (const std::vector<UsdAttribute> *points :
             {&weight.targetPoints, &weight.samplePoints, &weight.curvePoints,
              &weight.combineTargetPoints, &weight.curvenetMeshPoints,
              &weight.curvenetPoints, &weight.curvenetCounts,
              &weight.curvenetIndices, &weight.curvenetSplines}) {
        if (!points->empty()) {
            step->resolvedInputReads = true;
            for (const UsdAttribute &a : *points) {
                step->varyingInputs =
                    step->varyingInputs || a.ValueMightBeTimeVarying();
            }
        }
    }
    // The two arrays a curvenet weight reads off its own prim. Same rule:
    // no RigExecBakedInput covers an array, so the step reads them through
    // the generation's resolved inputs and must say so.
    for (const UsdAttribute *a :
             {&weight.curvenetWeights, &weight.curvenetAutoSmooth}) {
        if (*a) {
            step->resolvedInputReads = true;
            step->varyingInputs =
                step->varyingInputs || a->ValueMightBeTimeVarying();
        }
    }
}

void
RigExecBakedBuildWeightSteps(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    // Where every volume weight ended up, as one step and one slot.
    //
    // It exists for two readers. pose.weightFrames is one; the other is the
    // ORACLE, which places a volume from this map and is what a constraint's
    // envelope and a current-phase field resolve through. The dynamic path
    // refreshes it after every commit and the geometry walk runs after all of
    // them, so one refresh at the end of the pose half is the same map every
    // reader of it sees -- and making it a step is what orders those readers
    // against it instead of leaving the refresh somewhere in the epilogue
    // where a parallel schedule could have read it already.
    bool anyVolume = false;
    for (const RigExecBakedProgramImpl::WeightObject &weight :
             B.weightObjects) {
        anyVolume = anyVolume || weight.providerSlot >= 0;
    }
    for (size_t i = 0; i < B.noScaleAvars.size() && !anyVolume; ++i) {
        // A volume bound to no mover still has a placement and still
        // publishes a weightFrames entry, so the provider table decides this
        // and not the weight-object table.
        anyVolume = B.noScaleAvars[i] != 0;
    }
    if (anyVolume) {
        RigExecBakedStep step;
        step.kind = RigExecBakedStepKind::VolumePlacements;
        step.object = 0;
        step.maxDiagnostics = 0;
        for (size_t i = 0; i < B.noScaleAvars.size(); ++i) {
            if (B.noScaleAvars[i] != 0) {
                step.reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::PoseFin, int(i)));
            }
        }
        step.writes.push_back(
            RigExecBakedOne(RigExecBakedSlotDomain::WeightFrames, 0));
        B.steps.push_back(std::move(step));
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
    if (step->kind == RigExecBakedStepKind::VolumePlacements) {
        // The evaluator's own routine, over the frames this walk ended with.
        // Not a second copy of it: a frame no matrix can be built from leaves
        // whatever the failed decomposition wrote rather than the identity,
        // and that is exactly the kind of detail a second copy loses.
        B.updateVolumePlacements(
            [&B](const SdfPath &provider, RigExecPointFrame *frame) {
                const auto slot = B.index.find(provider);
                if (slot == B.index.end()) {
                    return false;
                }
                *frame = B.fin[size_t(B.finLast[size_t(slot->second)])];
                return true;
            });
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

}  // namespace rigExec
