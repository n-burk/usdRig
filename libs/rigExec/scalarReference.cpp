#include "scalarReference.h"
#include "rigEvaluatorInternal.h"
#include "rigEvaluatorConstraints.h"
#include "rigExecMath/envelope.h"
#include <algorithm>
#include <cmath>
namespace rigExec {
using namespace evaluatorDetail;
namespace {
const TfToken _kGeoWeightObject("rigExec:weightObject");
const TfToken _kGeoInputsDefaultWeight("inputs:defaultWeight");
bool _IsEnabled(const RigExecOraclePrim &mover, UsdTimeCode time) {
    bool enabled = true;
    if (const auto a = mover.GetAttribute(_enabledAttr)) a.Get(&enabled,time);
    return enabled;
}
}
VtVec3fArray
RigExecScalarChainReference(
    const RigExecScalarReferenceContext &context,
    const SdfPath &target,
    const std::vector<const RigExecMoverRecord *> &chain,
    const std::unordered_map<SdfPath, GfMatrix4d, SdfPath::Hash> &baseProviderMatrices,
    const std::unordered_map<SdfPath, GfMatrix4d, SdfPath::Hash> &finalProviderMatrices,
    UsdTimeCode time,
    std::vector<std::string> *diagnostics,
    const std::unordered_map<SdfPath, GfMatrix4d, SdfPath::Hash> &geometryConstraintDeltas)
{
    // The oracle resolves a read phase ITSELF, from the authored metadata,
    // and reads the same recorded snapshots. That keeps it independent of
    // RigExecResolveRevisionBinding and RigExecAssembleParameters -- which is
    // what makes parity a real check -- while sharing the authored INTENT,
    // which it must, or the two paths are evaluating different rigs.

    // Base: the stock resolved value of the exact native property
    // (spec §7.2). The base revision is retained: blend-shape deltas
    // derive against base points, not the preceding revision (spec §7.3).
    VtVec3fArray points;
    const RigExecOracleAttribute baseAttr = context.stage->GetAttributeAtPath(target);
    const auto upstream = context.upstream.find(target);
    const bool upstreamRead = upstream != context.upstream.end() && upstream->second.IsHolding<VtVec3fArray>();
    if (upstreamRead) points = upstream->second.UncheckedGet<VtVec3fArray>();
    if (!upstreamRead && (!baseAttr || !baseAttr.Get(&points, time))) {
        diagnostics->push_back("no base value for " + target.GetString());
        return points;
    }
    const VtVec3fArray basePoints = points;

    for (size_t index = 0; index < chain.size(); ++index) {
        const RigExecMoverRecord *mover = chain[index];
        const RigExecOraclePrim prim = context.stage->GetPrimAtPath(mover->moverPath);
        if (!prim || !_IsEnabled(prim, time)) {
            continue;  // pass-through (spec §4.2)
        }

        // Resolve the common envelope against the PRECEDING revision. A
        // current-phase volume therefore measures exactly the points that
        // enter this mover. A bound object supersedes inputs:defaultWeight.
        const VtVec3fArray preceding = points;
        std::vector<float> envelope(points.size(), 1.0f);
        SdfPathVector weightObjects;
        if (const RigExecOracleRelationship rel =
                prim.GetRelationship(_kGeoWeightObject)) {
            rel.GetTargets(&weightObjects);
        }
        if (!weightObjects.empty()) {
            const std::vector<GfVec3f> currentPoints(
                preceding.begin(), preceding.end());
            std::string error;
            if (!context.weights) error="no weight reference adapter";
            if (!context.weights || !context.weights(mover->moverPath, weightObjects[0], points.size(),
                                 &envelope, &error, &currentPoints)) {
                if (context.currentPhaseWeight &&
                    context.currentPhaseWeight(mover->moverPath)) {
                    // The producer-level refusal already has this canonical route.
                    // Form it from the independent error; a different reason remains
                    // a distinct diagnostic and cannot be hidden by deduplication.
                    const std::string message = "current-phase weight failed: " + error;
                    if (std::find(diagnostics->begin(), diagnostics->end(), message) ==
                        diagnostics->end()) diagnostics->push_back(message);
                } else {
                    diagnostics->push_back(
                        "MoverFailed " + mover->moverPath.GetString() + ": " +
                        error);
                }
                continue;
            }
        } else {
            const float scalar = [&]() { float v=1.0f;context.resolved.GetAttribute(prim.GetAttribute(_kGeoInputsDefaultWeight),time,&v);return v; }();
            if (!std::isfinite(scalar) || scalar < 0.0f || scalar > 1.0f) {
                diagnostics->push_back(
                    "MoverFailed " + mover->moverPath.GetString() +
                    ": inputs:defaultWeight must be finite and in [0, 1]");
                continue;
            }
            std::fill(envelope.begin(), envelope.end(), scalar);
        }

        bool envelopeApplied = false;
        if (_IsSourceFrameConstraintType(prim.GetTypeName())) {
            const auto delta = geometryConstraintDeltas.find(mover->moverPath);
            if (delta == geometryConstraintDeltas.end()) continue;
            for (auto &point : points)
                point = GfVec3f(delta->second.TransformAffine(GfVec3d(point)));
        } else if (const RigExecMoverHandler *oracleHandler =
                       mover->handler) {
            // The parity-oracle branch, from the mover's own row (see movers/).
            // A type with no row, or a row with no oracle, falls through to
            // the shared envelope blend over the unmodified points.
            if (oracleHandler->oracle) {
                // A dense blend sample phased read answers from the
                // recorded snapshot, resolved through this revision
                // compiled binding -- the one piece of evaluator
                // compile state the oracle needs.
                const auto sampleSnapshot = [&context, mover](const SdfPath &input, const SdfPath &sample) {
                    return context.sampleSnapshot ? context.sampleSnapshot(mover->moverPath, input, sample) : nullptr;
                };
                const RigExecMoverOracleContext oracleCtx{
                    context.stage,
                    prim,
                    mover->moverPath,
                    target,
                    time,
                    context.resolved,
                    context.phasedPoints,
                    context.phasedMatrix,
                    baseProviderMatrices,
                    finalProviderMatrices,
                    diagnostics,
                    &points,
                    basePoints,
                    sampleSnapshot,
                    index > 0 ? &preceding : nullptr,
                    &envelope,
                    context.boundFrames};
                const auto result = oracleHandler->oracle(oracleCtx);
                if (result == RigExecOracleResult::PassThrough) {
                    continue;
                }
                envelopeApplied = result == RigExecOracleResult::Applied;
            }
        }

        // Ordinary candidates blend once over the preceding revision.
        // Nonlinear weighting reports Applied after consuming this same
        // independently resolved envelope.
        if (points.size() != preceding.size() ||
            envelope.size() != points.size()) {
            diagnostics->push_back(
                "MoverFailed " + mover->moverPath.GetString() +
                ": result cardinality changed; revision passed through");
            points = preceding;
            continue;
        }
        if (!envelopeApplied) for (size_t i = 0; i < points.size(); ++i) {
            points[i] = RigExecBlendEnvelope(
                preceding[i], points[i], envelope[i]);
        }
        if (context.revisionObserver)
            context.revisionObserver(index, mover->moverPath, points);
    }
    return points;
}

} // namespace rigExec
