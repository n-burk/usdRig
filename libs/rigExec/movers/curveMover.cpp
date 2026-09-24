//
// RigExecCurveMover: everything about the curve mover (spec §4.1).
//
// A curve mover deforms points from a driver curve in one of three
// modes: ribbon (rotation-minimizing frame transport), wire (NURBS
// displacement at the bind parameter), or emitGuidePoints. This TU owns
// its exec-side computeMoverParameters registration and builder, its
// revision binder, and its parity-oracle branch, and registers the row
// that points at them. Compile validation is the generic points-target
// rules.
//

#include "moverRegistry.h"
#include "moverExecCommon.h"

#include "rigExecMath/geometryKernels.h"

#include "pxr/exec/exec/builtinComputations.h"
#include "pxr/exec/exec/registerSchema.h"
#include "pxr/exec/vdf/context.h"
#include "pxr/exec/vdf/readIterator.h"

using rigExec::RigExecMoverParameters;
using rigExec::RigExecMoverExecTokens;
using rigExec::RigExecPointFrameArray;

PXR_NAMESPACE_USING_DIRECTIVE

namespace {

RigExecMoverParameters
_BuildCurveMoverParameters(const VdfContext &ctx)
{
    RigExecMoverParameters params;
    static const TfToken ribbon("ribbon");
    const TfToken *mode =
        ctx.GetInputValuePtr<TfToken>(RigExecMoverExecTokens->modeAttr);
    params.kind = mode ? *mode : ribbon;
    const bool *enabled =
        ctx.GetInputValuePtr<bool>(RigExecMoverExecTokens->enabled);
    params.enabled = enabled ? *enabled : true;
    if (!params.enabled) {
        params.valid = true;
        return params;
    }
    if (!rigExec::RigExecMoverSetCommonEnvelope(ctx, &params)) {
        return params;
    }
    const RigExecPointFrameArray *frames =
        ctx.GetInputValuePtr<RigExecPointFrameArray>(
            RigExecMoverExecTokens->driverFrames);
    if (!frames || frames->IsEmpty() ||
        frames->rests.size() != frames->GetSize()) {
        return params;
    }
    params.frames = *frames;
    if (params.kind == "ribbon") {
        params.bindCoords = rigExec::RigExecMoverCollect<GfVec2f>(
            ctx, RigExecMoverExecTokens->bindCoords);
        params.valid = !params.bindCoords.empty();
    } else if (params.kind == "emitGuidePoints") {
        params.valid = true;
    }
    return params;
}

std::optional<rigExec::RigExecRevisionOp>
_ResolveCurveOp(const TfToken &curveMode)
{
    // The curve mover's frozen signature branches on its authored mode.
    if (curveMode == "emitGuidePoints") {
        return rigExec::RigExecRevisionOp::EmitGuidePoints;
    }
    if (curveMode == "wire") {
        return rigExec::RigExecRevisionOp::Wire;
    }
    return rigExec::RigExecRevisionOp::Ribbon;
}

void
_BindCurveMover(const rigExec::RigExecMoverBindContext &ctx)
{
    rigExec::RigExecRevisionBinding &binding = *ctx.binding;
    const UsdPrim &moverPrim = ctx.moverPrim;
    const std::map<SdfPath, SdfPath> &frameChainHeads = ctx.frameChainHeads;
    const SdfPathVector binds = rigExec::RigExecRelationshipTargets(
        moverPrim, "rigExec:bindCoordinates");
    if (!binds.empty()) {
        binding.bindCoords = binds[0];
    }
    // The wire's driver: a NURBS curve prim, read at its declared phase
    // (a curve deformed by its own chain reads "final").
    const SdfPathVector curves = rigExec::RigExecRelationshipTargets(
        moverPrim, "rigExec:driverCurve");
    if (!curves.empty()) {
        const SdfPath curvePrim = curves[0].GetPrimPath();
        binding.driverCurvePoints =
            curvePrim.AppendProperty(TfToken("points"));
        binding.driverCurveOrder =
            curvePrim.AppendProperty(TfToken("order"));
        binding.driverCurveKnots =
            curvePrim.AppendProperty(TfToken("knots"));
        const rigExec::RigExecReadPhase phase =
            rigExec::RigExecPhaseForInput(
                moverPrim, "rigExec:driverCurve",
                "rigExec:driverCurveReadPhase");
        if (!phase.IsBase()) {
            binding.phases[binding.driverCurvePoints] = phase;
        }
    }
    // Or the wire's control points moved by matrix providers directly:
    // one transform (or one for all) per unique control point, measured
    // against its space. Carried as influences -- transforms first, then
    // spaces -- so every path delivers them the way it delivers a skin's.
    const SdfPathVector driverTransforms =
        rigExec::RigExecRelationshipTargets(
            moverPrim, "rigExec:driverTransforms");
    if (!driverTransforms.empty()) {
        binding.transformPhase = rigExec::RigExecPhaseForInput(
            moverPrim, "rigExec:driverTransforms",
            "rigExec:transformReadPhase");
        const auto provider = [&](SdfPath path) {
            if (binding.transformPhase.kind ==
                rigExec::RigExecReadPhaseKind::Final) {
                const auto it = frameChainHeads.find(path);
                if (it != frameChainHeads.end()) {
                    path = it->second;
                }
            }
            return path;
        };
        for (const SdfPath &t : driverTransforms) {
            binding.influences.push_back(provider(t));
        }
        const SdfPathVector spaces = rigExec::RigExecRelationshipTargets(
            moverPrim, "rigExec:driverTransformSpaces");
        for (const SdfPath &s : spaces) {
            binding.influences.push_back(provider(s));
        }
        const SdfPathVector baseTransforms =
            rigExec::RigExecRelationshipTargets(
                moverPrim, "rigExec:driverBaseTransforms");
        for (const SdfPath &b : baseTransforms) {
            binding.influences.push_back(provider(b));
        }
        for (const SdfPath &b :
             rigExec::RigExecRelationshipTargets(
                 moverPrim, "rigExec:driverBaseTransformSpaces")) {
            binding.influences.push_back(provider(b));
        }
        binding.driverTransformCount = int(driverTransforms.size());
        binding.driverSpaceCount = int(spaces.size());
        binding.driverBaseTransformCount = int(baseTransforms.size());
    }
    const SdfPathVector frames = rigExec::RigExecRelationshipTargets(
        moverPrim, "rigExec:driverFrames");
    if (!frames.empty()) {
        binding.driverFrames = frames[0];
    }
    if (!binding.bindCoords.IsEmpty()) {
        const rigExec::RigExecReadPhase phase =
            rigExec::RigExecPhaseForInput(
                moverPrim, "rigExec:bindCoordinates", nullptr);
        if (!phase.IsBase()) {
            binding.phases[binding.bindCoords] = phase;
        }
    }
}

rigExec::RigExecOracleResult
_OracleCurveMover(const rigExec::RigExecMoverOracleContext &ctx)
{
    using rigExec::RigExecOracleResult;
    const UsdStageRefPtr &stage = ctx.stage;
    const UsdPrim &prim = ctx.prim;
    const SdfPath &moverPath = ctx.moverPath;
    const UsdTimeCode time = ctx.time;
    std::vector<std::string> *diagnostics = ctx.diagnostics;
    VtVec3fArray &points = *ctx.points;
    TfToken mode("ribbon");
    if (UsdAttribute a = prim.GetAttribute(TfToken("rigExec:mode"))) {
        a.Get(&mode, time);
    }
    if (mode == "wire") {
        // The wire reads its NURBS driver directly: posed control
        // points at the declared phase, rest control points, order
        // and knots as authored, and the authored bind coordinates.
        SdfPathVector curves, binds;
        if (UsdRelationship rel = prim.GetRelationship(
                TfToken("rigExec:driverCurve"))) {
            rel.GetTargets(&curves);
        }
        if (UsdRelationship rel = prim.GetRelationship(
                TfToken("rigExec:bindCoordinates"))) {
            rel.GetTargets(&binds);
        }
        if (curves.empty() || binds.empty()) {
            diagnostics->push_back(
                "MoverFailed " + moverPath.GetString() +
                ": wire needs rigExec:driverCurve and "
                "rigExec:bindCoordinates");
            return RigExecOracleResult::PassThrough;
        }
        const SdfPath curvePrim = curves[0].GetPrimPath();
        VtVec3fArray posedCvs, restCvs;
        SdfPathVector driverTransforms, driverSpaces;
        if (UsdRelationship rel = prim.GetRelationship(
                TfToken("rigExec:driverTransforms"))) {
            rel.GetTargets(&driverTransforms);
        }
        if (UsdRelationship rel = prim.GetRelationship(
                TfToken("rigExec:driverTransformSpaces"))) {
            rel.GetTargets(&driverSpaces);
        }
        if (driverTransforms.empty()) {
            rigExec::RigExecReadPhasedPoints(
                stage, ctx.snapshots, time, prim, "rigExec:driverCurve",
                "rigExec:driverCurveReadPhase",
                curvePrim.AppendProperty(TfToken("points")), moverPath,
                &posedCvs);
        }
        VtIntArray order;
        VtDoubleArray knots;
        VtVec2fArray sts;
        float dropoff = 0.0f;
        if (const UsdPrim curve = stage->GetPrimAtPath(curvePrim)) {
            curve.GetAttribute(TfToken("points"))
                .Get(&restCvs, UsdTimeCode::Default());
            curve.GetAttribute(TfToken("order"))
                .Get(&order, UsdTimeCode::Default());
            curve.GetAttribute(TfToken("knots"))
                .Get(&knots, UsdTimeCode::Default());
        }
        if (UsdAttribute a = stage->GetAttributeAtPath(binds[0])) {
            a.Get(&sts, time);
        }
        if (UsdAttribute a =
                prim.GetAttribute(TfToken("inputs:dropoffDistance"))) {
            a.Get(&dropoff, time);
        }
        if (!driverTransforms.empty()) {
            // Independently of the assembler: the providers' own
            // matrices, measured and weighted per control point.
            TfToken phase("base");
            if (const UsdAttribute a = prim.GetAttribute(
                    TfToken("rigExec:transformReadPhase"))) {
                a.Get(&phase);
            }
            const auto &matrices = phase == "final"
                                       ? ctx.finalProviderMatrices
                                       : ctx.baseProviderMatrices;
            VtFloatArray weights, baseWeights;
            if (const UsdAttribute a = prim.GetAttribute(
                    TfToken("inputs:driverWeights"))) {
                a.Get(&weights, time);
            }
            if (const UsdAttribute a = prim.GetAttribute(
                    TfToken("inputs:driverBaseWeights"))) {
                a.Get(&baseWeights, time);
            }
            SdfPathVector baseTransforms, baseSpaces;
            if (UsdRelationship rel = prim.GetRelationship(
                    TfToken("rigExec:driverBaseTransforms"))) {
                rel.GetTargets(&baseTransforms);
            }
            if (UsdRelationship rel = prim.GetRelationship(
                    TfToken("rigExec:driverBaseTransformSpaces"))) {
                rel.GetTargets(&baseSpaces);
            }
            const auto pick = [](size_t count, size_t j) {
                return count <= 1 ? size_t(0) : j % count;
            };
            bool missing = false;
            const auto measured = [&](const SdfPathVector &ts,
                                      const SdfPathVector &ss,
                                      size_t j) {
                GfMatrix4d m(1.0);
                const auto t = matrices.find(ts[pick(ts.size(), j)]);
                if (t == matrices.end()) {
                    missing = true;
                    return m;
                }
                m = t->second;
                if (!ss.empty()) {
                    const auto sp =
                        matrices.find(ss[pick(ss.size(), j)]);
                    if (sp == matrices.end()) {
                        missing = true;
                        return m;
                    }
                    m = rigExec::RigExecMeasureInSpace(m, sp->second);
                }
                return m;
            };
            posedCvs = restCvs;
            for (size_t j = 0; j < restCvs.size() && !missing; ++j) {
                if (!baseTransforms.empty()) {
                    const GfMatrix4d b =
                        measured(baseTransforms, baseSpaces, j);
                    const float wb = baseWeights.empty()
                        ? 1.0f
                        : baseWeights[pick(baseWeights.size(), j)];
                    const GfVec3f moved(
                        b.TransformAffine(GfVec3d(restCvs[j])));
                    restCvs[j] = restCvs[j] + (moved - restCvs[j]) * wb;
                }
                const GfMatrix4d m =
                    measured(driverTransforms, driverSpaces, j);
                const float w = weights.empty()
                    ? 1.0f : weights[pick(weights.size(), j)];
                const GfVec3f moved(
                    m.TransformAffine(GfVec3d(restCvs[j])));
                posedCvs[j] = restCvs[j] + (moved - restCvs[j]) * w;
            }
            if (missing) {
                diagnostics->push_back(
                    "MoverFailed " + moverPath.GetString() +
                    ": no matrix for a wire driver transform");
                return RigExecOracleResult::PassThrough;
            }
        }
        const std::vector<GfVec3f> rest(restCvs.begin(), restCvs.end());
        const std::vector<GfVec3f> posed(posedCvs.begin(),
                                         posedCvs.end());
        const std::vector<double> knotVec(knots.begin(), knots.end());
        const int curveOrder = order.empty() ? 0 : order[0];
        std::vector<GfVec3f> scratch(points.begin(), points.end());
        // A sparse bind table is parallel to the weight object's
        // indices; spread over the whole mesh here, where the
        // envelope below zeroes every point it does not name.
        std::vector<GfVec2f> bindAll(sts.begin(), sts.end());
        if (sts.size() != scratch.size()) {
            SdfPathVector weightTargets;
            if (UsdRelationship rel = prim.GetRelationship(
                    TfToken("rigExec:weightObject"))) {
                rel.GetTargets(&weightTargets);
            }
            VtIntArray indices;
            if (!weightTargets.empty()) {
                if (const UsdPrim w =
                        stage->GetPrimAtPath(weightTargets[0])) {
                    w.GetAttribute(TfToken("rigExec:indices"))
                        .Get(&indices);
                }
            }
            if (indices.size() == sts.size()) {
                bindAll.assign(scratch.size(), GfVec2f(0.0f));
                for (size_t k = 0; k < indices.size(); ++k) {
                    if (indices[k] >= 0 &&
                        size_t(indices[k]) < bindAll.size()) {
                        bindAll[size_t(indices[k])] = sts[k];
                    }
                }
            }
        }
        if (!rigExec::RigExecApplyWire(
                &scratch, rigExec::RigExecNurbsCurve{&rest, curveOrder,
                                                    &knotVec},
                rigExec::RigExecNurbsCurve{&posed, curveOrder, &knotVec},
                bindAll.data(), bindAll.size(), dropoff,
                0, scratch.size())) {
            diagnostics->push_back(
                "MoverFailed " + moverPath.GetString() +
                ": wire driver curve or bind coordinates do not "
                "match the deformed points");
            return RigExecOracleResult::PassThrough;
        }
        std::copy(scratch.begin(), scratch.end(), points.begin());
        return RigExecOracleResult::Blend;
    }
    // Parity path samples the driver curve directly: rest from
    // the bind-time authored value, posed from the timed value.
    SdfPathVector frameTargets;
    if (UsdRelationship rel = prim.GetRelationship(
            TfToken("rigExec:driverFrames"))) {
        rel.GetTargets(&frameTargets);
    }
    SdfPath curvePoints;
    int sampleCount = 5;
    if (!frameTargets.empty()) {
        if (const UsdPrim ribbon =
                stage->GetPrimAtPath(frameTargets[0])) {
            SdfPathVector curves;
            if (UsdRelationship rel = ribbon.GetRelationship(
                    TfToken("rigExec:driverCurve"))) {
                rel.GetTargets(&curves);
            }
            if (!curves.empty()) {
                curvePoints = curves[0].IsPrimPath()
                    ? curves[0].AppendProperty(TfToken("points"))
                    : curves[0];
            }
            if (UsdAttribute a = ribbon.GetAttribute(
                    TfToken("rigExec:sampleCount"))) {
                a.Get(&sampleCount, time);
            }
        }
    }
    VtVec3fArray posedCvs, restCvs;
    if (UsdAttribute a = stage->GetAttributeAtPath(curvePoints)) {
        a.Get(&posedCvs, time);
        a.Get(&restCvs, UsdTimeCode::Default());
    }
    const auto posedSamples = rigExec::RigExecSampleCurveRMF(
        std::vector<GfVec3f>(posedCvs.begin(), posedCvs.end()),
        sampleCount);
    const auto restSamples = rigExec::RigExecSampleCurveRMF(
        std::vector<GfVec3f>(restCvs.begin(), restCvs.end()),
        sampleCount);
    if (mode == "emitGuidePoints") {
        if (posedSamples.GetSize() == points.size()) {
            for (size_t i = 0; i < points.size(); ++i) {
                points[i] = posedSamples.positions[i];
            }
        } else {
            diagnostics->push_back(
                "MoverFailed " + moverPath.GetString() +
                ": guide cardinality mismatch");
        }
    } else {
        SdfPathVector binds;
        if (UsdRelationship rel = prim.GetRelationship(
                TfToken("rigExec:bindCoordinates"))) {
            rel.GetTargets(&binds);
        }
        VtVec2fArray sts;
        if (!binds.empty()) {
            if (UsdAttribute a =
                    stage->GetAttributeAtPath(binds[0])) {
                a.Get(&sts, time);
            }
        }
        std::vector<GfVec3f> scratch(points.begin(), points.end());
        rigExec::RigExecApplyRibbonTransport(
            &scratch,
            std::vector<GfVec2f>(sts.begin(), sts.end()),
            restSamples, posedSamples);
        std::copy(scratch.begin(), scratch.end(), points.begin());
    }
    return RigExecOracleResult::Blend;
}

rigExec::RigExecMoverHandler
_MakeHandler()
{
    rigExec::RigExecMoverHandler handler(
        "RigExecCurveMover", &_ResolveCurveOp,
        rigExec::RigExecMoverDomain::Points);
    handler.frameRelationships = {
        "rigExec:driverTransforms", "rigExec:driverTransformSpaces",
        "rigExec:driverBaseTransforms",
        "rigExec:driverBaseTransformSpaces"};
    handler.bind = &_BindCurveMover;
    handler.oracle = &_OracleCurveMover;
    return handler;
}

}  // namespace

// EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA opens/closes the pxr namespace
// itself, so the registration block stays at global scope.
EXEC_REGISTER_COMPUTATIONS_FOR_SCHEMA(RigExecCurveMover)
{
    self.PrimComputation(RigExecMoverExecTokens->computeMoverParameters)
        .Callback<RigExecMoverParameters>(&_BuildCurveMoverParameters)
        .Inputs(
            RIGEXEC_MOVER_COMMON_INPUTS,
            AttributeValue<TfToken>(RigExecMoverExecTokens->modeAttr),
            Relationship(RigExecMoverExecTokens->resolvedBindCoords)
                .TargetedObjects<GfVec2f>(
                    ExecBuiltinComputations->computeValue)
                .InputName(RigExecMoverExecTokens->bindCoords),
            Relationship(RigExecMoverExecTokens->resolvedDriverFrames)
                .TargetedObjects<rigExec::RigExecPointFrameArray>(
                    RigExecMoverExecTokens->computePointFrameArray)
                .InputName(RigExecMoverExecTokens->driverFrames));

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
