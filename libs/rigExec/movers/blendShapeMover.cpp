//
// RigExecBlendShapeMover: everything about the blend-shape mover
// (spec §4.1, §7.3).
//
// A blend-shape mover accumulates channel-weighted delta samples onto
// the points: p'_i = p_i + sum_k alpha_k(w_k) d_{k,i}, with deltas
// derived against the authored base. This TU owns its exec-side
// computeMoverParameters registration and builder, its revision binder,
// its compile validator, and its parity-oracle branch, and registers
// the row that points at them.
//

#include "moverRegistry.h"
#include "moverExecCommon.h"

#include "rigExecMath/geometryKernels.h"

#include "pxr/exec/exec/builtinComputations.h"
#include "pxr/exec/exec/registerSchema.h"
#include "pxr/exec/vdf/context.h"
#include "pxr/exec/vdf/readIterator.h"
#include "pxr/usd/usdGeom/mesh.h"
#include "pxr/usd/usdSkel/blendShape.h"

#include <algorithm>

using rigExec::RigExecBlendChannel;
using rigExec::RigExecBlendSampleData;
using rigExec::RigExecMoverParameters;
using rigExec::RigExecMoverExecTokens;

PXR_NAMESPACE_USING_DIRECTIVE

namespace {

RigExecMoverParameters
_BuildBlendMoverParameters(const VdfContext &ctx)
{
    RigExecMoverParameters params;
    params.kind = TfToken("blendShape");
    const bool *enabled =
        ctx.GetInputValuePtr<bool>(RigExecMoverExecTokens->enabled);
    params.enabled = enabled ? *enabled : true;
    if (!params.enabled) {
        params.valid = true;
        return params;
    }

    // Base points for delta derivation (spec §7.3): deltas derive against
    // the authored base, never the preceding revision.
    std::vector<GfVec3f> base;
    {
        VdfReadIterator<GfVec3f> it(
            ctx, RigExecMoverExecTokens->basePoints);
        base.reserve(it.ComputeSize());
        for (; !it.IsAtEnd(); ++it) {
            base.push_back(*it);
        }
    }
    if (base.empty()) {
        return params;
    }
    const TfToken *deltaSpace = ctx.GetInputValuePtr<TfToken>(
        RigExecMoverExecTokens->deltaSpace);
    if (deltaSpace && *deltaSpace != "target" &&
        *deltaSpace != "surfaceFrame") {
        return params;
    }
    params.blendSurfaceFrame = deltaSpace && *deltaSpace == "surfaceFrame";
    if (params.blendSurfaceFrame) {
        params.restPoints = base;
        for (VdfReadIterator<int> it(
                 ctx, RigExecMoverExecTokens->topologyCounts);
             !it.IsAtEnd(); ++it) {
            params.topologyCounts.push_back(*it);
        }
        for (VdfReadIterator<int> it(
                 ctx, RigExecMoverExecTokens->topologyIndices);
             !it.IsAtEnd(); ++it) {
            params.topologyIndices.push_back(*it);
        }
        if (params.topologyCounts.empty()) {
            return params;
        }
    }
    // Summed by the shared kernel (moverGraph.h): RigExecRigEvaluator builds
    // the same packet from tapped channels with no derived stage, and the two
    // must agree exactly.
    std::vector<RigExecBlendChannel> channelValues;
    {
        VdfReadIterator<RigExecBlendChannel> channels(
            ctx, RigExecMoverExecTokens->blendChannels);
        channelValues.reserve(channels.ComputeSize());
        for (; !channels.IsAtEnd(); ++channels) {
            channelValues.push_back(*channels);
        }
    }
    if (!rigExec::RigExecSumBlendChannels(channelValues, base,
                                          &params.blendDeltas)) {
        return params;  // structural error: fails atomically
    }
    // Common MoverAPI envelope. A bound object supersedes the scalar.
    if (!rigExec::RigExecMoverSetCommonEnvelope(ctx, &params)) {
        return params;
    }
    params.valid = true;
    return params;
}

void
_BindBlendShapeMover(const rigExec::RigExecMoverBindContext &ctx)
{
    rigExec::RigExecRevisionBinding &binding = *ctx.binding;
    const UsdPrim &moverPrim = ctx.moverPrim;
    const SdfPath &target = ctx.target;
    const SdfPath &ownerPath = ctx.ownerPath;
    if (moverPrim.GetStage()->GetPrimAtPath(ownerPath).IsA<UsdGeomMesh>()) {
        binding.topologyCounts = ownerPath.AppendProperty(
            TfToken("faceVertexCounts"));
        binding.topologyIndices = ownerPath.AppendProperty(
            TfToken("faceVertexIndices"));
    }
    binding.blendInputs = rigExec::RigExecRelationshipTargets(
        moverPrim, "rigExec:blendInputs");
    std::sort(binding.blendInputs.begin(), binding.blendInputs.end());
    for (const SdfPath &input : binding.blendInputs) {
        const UsdPrim channel = moverPrim.GetStage()->GetPrimAtPath(input);
        for (const SdfPath &samplePath :
             rigExec::RigExecRelationshipTargets(
                 channel, "rigExec:samples")) {
            const UsdPrim sample =
                moverPrim.GetStage()->GetPrimAtPath(samplePath);
            const SdfPathVector points =
                rigExec::RigExecRelationshipTargets(
                    sample, "rigExec:targetPoints");
            const SdfPathVector shapes =
                rigExec::RigExecRelationshipTargets(
                    sample, "rigExec:blendShape");
            // Exactly one of the two. Both authored is not a precedence
            // question: two shapes that disagree with a silent winner is
            // the worst of the three outcomes, so the sample is dropped
            // here and compile reports it.
            if (points.size() + shapes.size() != 1) {
                continue;
            }
            rigExec::RigExecReadPhase phase;
            std::string error;
            if (shapes.size() == 1) {
                // A sparse sample has no phased points property to read:
                // its offsets are authored data on a UsdSkelBlendShape,
                // not a chain result, so there is no "preceding" or
                // "final" revision of them to select.
                binding.blendSamples[input].push_back(
                    {samplePath, SdfPath(), phase, shapes[0]});
                continue;
            }
            rigExec::RigExecResolveReadPhase(
                sample.GetRelationship(TfToken("rigExec:targetPoints")),
                "rigExec:pointsReadPhase", &phase, &error);
            binding.blendSamples[input].push_back(
                {samplePath, rigExec::RigExecPointsOf(points[0]), phase,
                 SdfPath()});
        }
    }
    binding.base = target;
}

bool
_ValidateBlendShapeMover(
    const rigExec::RigExecMoverValidateContext &ctx, std::string *error)
{
    const UsdStageRefPtr &stage = ctx.stage;
    const UsdPrim &prim = ctx.prim;
    TfToken deltaSpace("target");
    prim.GetAttribute(TfToken("rigExec:deltaSpace")).Get(&deltaSpace);
    if (deltaSpace != "target" && deltaSpace != "surfaceFrame") {
        *error = prim.GetPath().GetString() + ": invalid blend deltaSpace";
        return false;
    }
    if (deltaSpace == "surfaceFrame") {
        SdfPathVector targets;
        prim.GetRelationship(TfToken("rigExec:moves")).GetTargets(&targets);
        for (const auto &target : targets) {
            if (!stage->GetPrimAtPath(target.GetPrimPath())
                     .IsA<UsdGeomMesh>()) {
                *error = prim.GetPath().GetString() +
                         ": surfaceFrame blend requires mesh targets";
                return false;
            }
        }
    }
    SdfPathVector inputs;
    prim.GetRelationship(TfToken("rigExec:blendInputs"))
        .GetTargets(&inputs);
    for (const SdfPath &inputPath : inputs) {
        const UsdPrim input = stage->GetPrimAtPath(inputPath);
        if (!input) {
            continue;
        }
        SdfPathVector samples;
        input.GetRelationship(TfToken("rigExec:samples"))
            .GetTargets(&samples);
        for (const SdfPath &samplePath : samples) {
            const UsdPrim sample = stage->GetPrimAtPath(samplePath);
            if (!sample) {
                continue;
            }
            // A sample states its shape exactly one way. Both
            // authored is refused rather than resolved by
            // precedence: two shapes that disagree with a silent
            // winner is worse than either shape being wrong.
            SdfPathVector densePoints, sparseShape;
            if (UsdRelationship rel = sample.GetRelationship(
                    TfToken("rigExec:targetPoints"))) {
                rel.GetTargets(&densePoints);
            }
            if (UsdRelationship rel = sample.GetRelationship(
                    TfToken("rigExec:blendShape"))) {
                rel.GetTargets(&sparseShape);
            }
            if (!densePoints.empty() && !sparseShape.empty()) {
                *error = samplePath.GetString() +
                         ": blend sample authors both rigExec:targetPoints"
                         " and rigExec:blendShape; exactly one is allowed";
                return false;
            }
            if (!sparseShape.empty()) {
                if (sparseShape.size() != 1 ||
                    !stage->GetPrimAtPath(
                         sparseShape[0].GetPrimPath())
                         .IsA<UsdSkelBlendShape>()) {
                    *error = samplePath.GetString() +
                             ": rigExec:blendShape must name exactly one"
                             " UsdSkelBlendShape prim";
                    return false;
                }
                // No phased read to validate: the offsets are
                // authored data, not a chain revision, so there
                // is no preceding or final version of them.
                continue;
            }
            rigExec::RigExecReadPhase phase;
            std::string phaseError;
            const UsdAttribute legacy = sample.GetAttribute(
                TfToken("rigExec:pointsReadPhase"));
            if (!rigExec::RigExecResolveReadPhase(
                    sample.GetRelationship(
                        TfToken("rigExec:targetPoints")),
                    "rigExec:pointsReadPhase", &phase, &phaseError) ||
                (legacy && legacy.GetNumTimeSamples() != 0)) {
                *error = samplePath.GetString() +
                         ": blend sample points read phase must be a valid"
                         " static phase" +
                         (phaseError.empty() ? std::string()
                                             : ": " + phaseError);
                return false;
            }
        }
    }
    return true;
}

rigExec::RigExecOracleResult
_OracleBlendShapeMover(const rigExec::RigExecMoverOracleContext &ctx)
{
    using rigExec::RigExecOracleResult;
    const UsdStageRefPtr &stage = ctx.stage;
    const UsdPrim &prim = ctx.prim;
    const SdfPath &moverPath = ctx.moverPath;
    const SdfPath &target = ctx.target;
    const UsdTimeCode time = ctx.time;
    const rigExec::RigExecResolvedInputs &resolved = ctx.resolved;
    std::vector<std::string> *diagnostics = ctx.diagnostics;
    VtVec3fArray &points = *ctx.points;
    const VtVec3fArray &basePoints = ctx.basePoints;
    // p'_i = p_i + sum_k alpha_k(w_k) d_{k,i} (spec §7.3).
    SdfPathVector inputs;
    if (UsdRelationship rel =
            prim.GetRelationship(TfToken("rigExec:blendInputs"))) {
        rel.GetTargets(&inputs);
    }
    // Active inputs accumulate in canonical input-path order
    // (spec §7.3); authored relationship order is non-semantic.
    std::sort(inputs.begin(), inputs.end());

    VtVec3fArray next = points;
    bool failed = false;
    for (const SdfPath &inputPath : inputs) {
        const UsdPrim input = stage->GetPrimAtPath(inputPath);
        if (!input) {
            diagnostics->push_back(
                "MoverFailed " + moverPath.GetString() +
                ": missing blend input " + inputPath.GetString());
            failed = true;
            break;
        }
        float channel = 0;
        // Through the resolver, NOT straight off the stage.
        //
        // An authored `inputs:weight` and a DRIVEN one are the same
        // attribute; a plain Get() sees only the first. The moment
        // anything connects a weight -- which is the entire point of
        // a pose-space rig, where an interpolator drives every
        // corrective -- this oracle read 0 while the real path read
        // the driven value, and then reported the disagreement as a
        // parity mismatch in the blend kernel. The packet assembly
        // has always used _resolvedInputs here; this is the oracle
        // catching up to it.
        resolved.GetAttribute(
            input.GetAttribute(TfToken("inputs:weight")), time,
            &channel);
        if (!std::isfinite(channel)) {
            diagnostics->push_back(
                "MoverFailed " + moverPath.GetString() +
                ": non-finite channel weight on " +
                inputPath.GetString());
            failed = true;
            break;
        }
        SdfPathVector samplePaths;
        if (UsdRelationship rel = input.GetRelationship(
                TfToken("rigExec:samples"))) {
            rel.GetTargets(&samplePaths);
        }

        // Collect samples: activation plus target-shape points.
        // Activations must be finite, strictly positive, and
        // unique; samples compile in (activation, canonicalPath)
        // order with an implicit zero-delta sample at activation 0
        // and the channel weight clamped to [0, lastActivation]
        // (spec §7.3).
        struct _Sample {
            float activation;
            SdfPath path;
            VtVec3fArray shape;
        };
        std::vector<_Sample> samples;
        if (samplePaths.empty()) {
            diagnostics->push_back(
                "MoverFailed " + moverPath.GetString() +
                ": blend input has no samples: " +
                inputPath.GetString());
            failed = true;
            break;
        }
        for (const SdfPath &samplePath : samplePaths) {
            const UsdPrim sample =
                stage->GetPrimAtPath(samplePath.GetPrimPath());
            if (!sample) {
                diagnostics->push_back(
                    "MoverFailed " + moverPath.GetString() +
                    ": missing blend sample " +
                    samplePath.GetString());
                failed = true;
                break;
            }
            SdfPathVector shapeTargets, sparseTargets;
            if (UsdRelationship rel = sample.GetRelationship(
                    TfToken("rigExec:targetPoints"))) {
                rel.GetTargets(&shapeTargets);
            }
            if (UsdRelationship rel = sample.GetRelationship(
                    TfToken("rigExec:blendShape"))) {
                rel.GetTargets(&sparseTargets);
            }
            if (shapeTargets.size() + sparseTargets.size() != 1) {
                diagnostics->push_back(
                    "MoverFailed " + moverPath.GetString() +
                    ": blend sample must name exactly one of "
                    "rigExec:targetPoints or rigExec:blendShape: " +
                    samplePath.GetString());
                failed = true;
                break;
            }
            _Sample s;
            s.path = sample.GetPath();
            if (sparseTargets.size() == 1) {
                // The oracle is a SCALAR REFERENCE, so it reads the
                // blend shape straight off the stage and expands it
                // to a full moved-points array -- deliberately not
                // through RigExecBlendSampleCache. An oracle that
                // shared the fast path's cache would agree with it
                // about a stale layout, which is precisely the class
                // of bug this exists to catch.
                rigExec::RigExecBlendSampleLayout layout;
                rigExec::RigExecResolveBlendSampleLayout(
                    stage, sparseTargets[0], basePoints.size(), &layout);
                if (!layout.valid) {
                    diagnostics->push_back(
                        "MoverFailed " + moverPath.GetString() +
                        ": unusable blend shape at " +
                        sparseTargets[0].GetString());
                    failed = true;
                    break;
                }
                s.shape = VtVec3fArray(basePoints.begin(),
                                       basePoints.end());
                if (layout.indices.empty()) {
                    for (size_t i = 0; i < layout.offsets.size(); ++i) {
                        s.shape[i] += layout.offsets[i];
                    }
                } else {
                    for (size_t k = 0; k < layout.indices.size(); ++k) {
                        s.shape[size_t(layout.indices[k])] +=
                            layout.offsets[k];
                    }
                }
                s.activation = 1;
                if (UsdAttribute a = sample.GetAttribute(
                        TfToken("rigExec:activation"))) {
                    a.Get(&s.activation, time);
                }
                if (!std::isfinite(s.activation) ||
                    s.activation <= 0) {
                    diagnostics->push_back(
                        "MoverFailed " + moverPath.GetString() +
                        ": non-positive activation at " +
                        s.path.GetString());
                    failed = true;
                    break;
                }
                samples.push_back(std::move(s));
                continue;
            }
            const UsdAttribute shapeAttr =
                stage->GetAttributeAtPath(shapeTargets[0]);
            bool gotShape = shapeAttr && shapeAttr.Get(&s.shape, time);
            // A phased sample read answers from the recorded
            // snapshot, resolved through the compiled chain exactly
            // as the in-evaluator oracle did before this branch
            // moved here.
            if (const VtValue *value =
                    ctx.sampleSnapshot(inputPath, samplePath)) {
                if (value->IsHolding<VtVec3fArray>()) {
                    s.shape = value->UncheckedGet<VtVec3fArray>();
                    gotShape = true;
                }
            }
            if (!gotShape ||
                s.shape.size() != basePoints.size()) {
                diagnostics->push_back(
                    "MoverFailed " + moverPath.GetString() +
                    ": sample cardinality mismatch at " +
                    shapeTargets[0].GetString());
                failed = true;
                break;
            }
            s.activation = 1;
            if (UsdAttribute a = sample.GetAttribute(
                    TfToken("rigExec:activation"))) {
                a.Get(&s.activation, time);
            }
            if (!std::isfinite(s.activation) || s.activation <= 0) {
                diagnostics->push_back(
                    "MoverFailed " + moverPath.GetString() +
                    ": non-positive activation at " +
                    s.path.GetString());
                failed = true;
                break;
            }
            samples.push_back(std::move(s));
        }
        if (failed) {
            break;
        }
        if (samples.empty()) {
            continue;
        }
        std::sort(samples.begin(), samples.end(),
                  [](const _Sample &x, const _Sample &y) {
                      return x.activation != y.activation
                          ? x.activation < y.activation
                          : x.path < y.path;
                  });
        for (size_t k = 1; k < samples.size(); ++k) {
            if (samples[k].activation == samples[k - 1].activation) {
                diagnostics->push_back(
                    "MoverFailed " + moverPath.GetString() +
                    ": duplicate activation at " +
                    samples[k].path.GetString());
                failed = true;
                break;
            }
        }
        if (failed) {
            break;
        }

        const float w = std::min(
            std::max(channel, 0.0f), samples.back().activation);
        if (w == 0.0f) {
            continue;
        }
        // Piecewise-linear interpolation between the bracketing
        // activation samples; the lower bracket may be the
        // implicit zero-delta sample at activation 0. Deltas
        // derive against the destination BASE points (spec §7.3),
        // then accumulate onto the preceding revision.
        size_t hi = 0;
        while (hi < samples.size() &&
               samples[hi].activation < w) {
            ++hi;
        }
        if (hi >= samples.size()) {
            hi = samples.size() - 1;
        }
        const float aHi = samples[hi].activation;
        const float aLo = hi > 0 ? samples[hi - 1].activation : 0.0f;
        const float t = aHi > aLo ? (w - aLo) / (aHi - aLo) : 1.0f;
        const VtVec3fArray *shapeLo =
            hi > 0 ? &samples[hi - 1].shape : nullptr;
        const VtVec3fArray &shapeHi = samples[hi].shape;
        for (size_t i = 0; i < next.size(); ++i) {
            const GfVec3f deltaHi = shapeHi[i] - basePoints[i];
            const GfVec3f deltaLo = shapeLo
                ? (*shapeLo)[i] - basePoints[i] : GfVec3f(0);
            const GfVec3f delta =
                deltaLo + (deltaHi - deltaLo) * t;
            next[i] += delta;
        }
    }
    if (!failed) {
        TfToken space("target");
        prim.GetAttribute(TfToken("rigExec:deltaSpace")).Get(&space);
        if (space == "surfaceFrame") {
            VtIntArray counts, indices;
            stage->GetPrimAtPath(target.GetPrimPath()).GetAttribute(
                TfToken("faceVertexCounts")).Get(&counts, time);
            stage->GetPrimAtPath(target.GetPrimPath()).GetAttribute(
                TfToken("faceVertexIndices")).Get(&indices, time);
            std::vector<GfVec3f> deltas(next.size()), transported;
            for (size_t i = 0; i < next.size(); ++i) {
                deltas[i] = next[i] - points[i];
            }
            if (rigExec::RigExecTransportSurfaceOffsets(
                    std::vector<GfVec3f>(basePoints.begin(),
                                         basePoints.end()),
                    std::vector<GfVec3f>(points.begin(), points.end()),
                    std::vector<int>(counts.begin(), counts.end()),
                    std::vector<int>(indices.begin(), indices.end()),
                    deltas, &transported)) {
                for (size_t i = 0; i < next.size(); ++i) {
                    next[i] = points[i] + transported[i];
                }
            } else {
                failed = true;
                diagnostics->push_back(
                    "MoverFailed " + moverPath.GetString() +
                    ": degenerate blend surface frame");
            }
        }
        if (!failed) {
            points = next;
        }
    }
    // The blend branch never passes through: even on failure it falls
    // to the shared envelope blend over the unmodified points, exactly
    // as the in-evaluator branch did.
    return RigExecOracleResult::Blend;
}

rigExec::RigExecMoverHandler
_MakeHandler()
{
    rigExec::RigExecMoverHandler handler(
        "RigExecBlendShapeMover",
        &rigExec::RigExecFixedMoverOp<
            rigExec::RigExecRevisionOp::BlendShape>,
        rigExec::RigExecMoverDomain::Points);
    handler.bind = &_BindBlendShapeMover;
    handler.validate = &_ValidateBlendShapeMover;
    handler.oracle = &_OracleBlendShapeMover;
    return handler;
}

}  // namespace

// EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA opens/closes the pxr namespace
// itself, so the registration block stays at global scope.
EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(RigExecBlendShapeMover)
{
    self.PrimComputation(RigExecMoverExecTokens->computeMoverParameters)
        .Callback<RigExecMoverParameters>(&_BuildBlendMoverParameters)
        .Inputs(
            RIGEXEC_MOVER_COMMON_INPUTS,
            AttributeValue<TfToken>(RigExecMoverExecTokens->deltaSpace),
            Relationship(RigExecMoverExecTokens->resolvedTopologyCounts)
                .TargetedObjects<int>(ExecBuiltinComputations->computeValue)
                .InputName(RigExecMoverExecTokens->topologyCounts),
            Relationship(RigExecMoverExecTokens->resolvedTopologyIndices)
                .TargetedObjects<int>(ExecBuiltinComputations->computeValue)
                .InputName(RigExecMoverExecTokens->topologyIndices),
            Relationship(RigExecMoverExecTokens->resolvedBlendInputs)
                .TargetedObjects<RigExecBlendChannel>(
                    RigExecMoverExecTokens->computeBlendChannel)
                .InputName(RigExecMoverExecTokens->blendChannels),
            Relationship(RigExecMoverExecTokens->resolvedBase)
                .TargetedObjects<GfVec3f>(
                    ExecBuiltinComputations->computeValue)
                .InputName(RigExecMoverExecTokens->basePoints));

    self.PrimComputation(RigExecMoverExecTokens->computeMoverStatus)
        .Callback<rigExec::RigExecMoverStatus>(
            &rigExec::RigExecMoverBuildStatus)
        .Inputs(
            Computation<RigExecMoverParameters>(
                RigExecMoverExecTokens->computeMoverParameters)
                .Required(),
            Computation<SdfPath>(ExecBuiltinComputations->computePath)
                .InputName(RigExecMoverExecTokens->moverPath));
}

RIGEXEC_REGISTER_MOVER(_MakeHandler());
