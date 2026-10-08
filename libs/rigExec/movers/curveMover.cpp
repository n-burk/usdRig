// RigExecCurveMover: everything about the curve mover (spec §4.1).
// A curve mover deforms points from a driver curve in one of three
// modes: ribbon (rotation-minimizing frame transport), wire (NURBS
// displacement at the bind parameter), or emitGuidePoints. This TU owns
// its exec-side computeMoverParameters registration and builder, its
// revision binder, and its parity-oracle branch, and registers the row
// that points at them. Compile validation is the generic points-target
// rules.
#include "moverRegistry.h"
#include "moverExecCommon.h"

#include "rigExecMath/geometryKernels.h"

#include "pxr/exec/exec/builtinComputations.h"
#include "pxr/exec/exec/registerSchema.h"
#include "pxr/exec/vdf/context.h"
#include "pxr/exec/vdf/readIterator.h"

#include <algorithm>
#include <cmath>

using rigExec::RigExecMoverParameters;
using rigExec::RigExecMoverExecTokens;
using rigExec::RigExecPointFrameArray;

PXR_NAMESPACE_USING_DIRECTIVE

namespace {
const TfToken _oracleLocalFrame("local");
const TfToken _oracleToken0("rigExec:mode");
const TfToken _oracleToken1("rigExec:driverCurve");
const TfToken _oracleToken2("rigExec:bindCoordinates");
const TfToken _oracleToken3("rigExec:driverTransforms");
const TfToken _oracleToken4("rigExec:driverTransformSpaces");
const TfToken _oracleToken5("points");
const TfToken _oracleToken6("order");
const TfToken _oracleToken7("knots");
const TfToken _oracleToken8("inputs:dropoffDistance");
const TfToken _oracleToken9("inputs:driverWeights");
const TfToken _oracleToken10("inputs:driverBaseWeights");
const TfToken _oracleToken11("rigExec:driverBaseTransforms");
const TfToken _oracleToken12("rigExec:driverBaseTransformSpaces");
const TfToken _oracleToken13("rigExec:pointFrame");
const TfToken _oracleToken14("rigExec:driverDeltaFrame");
const TfToken _oracleToken15("rigExec:space");
const TfToken _oracleToken16("rigExec:weightObject");
const TfToken _oracleToken17("rigExec:indices");
const TfToken _oracleToken18("rigExec:driverFrames");
const TfToken _oracleToken19("rigExec:sampleCount");
const TfToken _oracleToken20("ribbon");
const TfToken _oracleToken21("rest");


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
            rigExec::RigExecPhaseForInput(moverPrim, "rigExec:driverCurve");
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
            moverPrim, "rigExec:driverTransforms");
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
        // rigExec:space, the rig's carry, read at the drivers' phase. Only
        // a wire whose points are posed applies it (RigExecCarryWireCurves).
        const SdfPathVector carries =
            rigExec::RigExecRelationshipTargets(moverPrim, "rigExec:space");
        if (!carries.empty()) {
            binding.carrySpace = provider(carries[0]);
        }
    }
    const SdfPathVector frames = rigExec::RigExecRelationshipTargets(
        moverPrim, "rigExec:driverFrames");
    if (!frames.empty()) {
        binding.driverFrames = frames[0];
    }
    if (!binding.bindCoords.IsEmpty()) {
        const rigExec::RigExecReadPhase phase =
            rigExec::RigExecPhaseForInput(
                moverPrim, "rigExec:bindCoordinates");
        if (!phase.IsBase()) {
            binding.phases[binding.bindCoords] = phase;
        }
    }
}

rigExec::RigExecOracleResult
_OracleCurveMover(const rigExec::RigExecMoverOracleContext &ctx)
{
    using rigExec::RigExecOracleResult;
    const rigExec::RigExecOracleScene &stage = ctx.stage;
    const rigExec::RigExecOraclePrim &prim = ctx.prim;
    const SdfPath &moverPath = ctx.moverPath;
    const UsdTimeCode time = ctx.time;
    std::vector<std::string> *diagnostics = ctx.diagnostics;
    VtVec3fArray &points = *ctx.points;
    TfToken mode = _oracleToken20;
    if (rigExec::RigExecOracleAttribute a = prim.GetAttribute(_oracleToken0)) {
        a.Get(&mode, time);
    }
    if (mode == "wire") {
        // The wire reads its NURBS driver directly: posed control
        // points at the declared phase, rest control points, order
        // and knots as authored, and the authored bind coordinates.
        SdfPathVector curves, binds;
        if (rigExec::RigExecOracleRelationship rel = prim.GetRelationship(
                _oracleToken1)) {
            rel.GetTargets(&curves);
        }
        if (rigExec::RigExecOracleRelationship rel = prim.GetRelationship(
                _oracleToken2)) {
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
        if (rigExec::RigExecOracleRelationship rel = prim.GetRelationship(
                _oracleToken3)) {
            rel.GetTargets(&driverTransforms);
        }
        if (rigExec::RigExecOracleRelationship rel = prim.GetRelationship(
                _oracleToken4)) {
            rel.GetTargets(&driverSpaces);
        }
        if (driverTransforms.empty()) {
            rigExec::RigExecReadPhasedPoints(
                ctx, "rigExec:driverCurve",
                curvePrim.AppendProperty(_oracleToken5), &posedCvs);
        }
        VtIntArray order;
        VtDoubleArray knots;
        VtVec2fArray sts;
        float dropoff = 0.0f;
        if (const rigExec::RigExecOraclePrim curve = stage->GetPrimAtPath(curvePrim)) {
            curve.GetAttribute(_oracleToken5)
                .Get(&restCvs, UsdTimeCode::Default());
            curve.GetAttribute(_oracleToken6)
                .Get(&order, UsdTimeCode::Default());
            curve.GetAttribute(_oracleToken7)
                .Get(&knots, UsdTimeCode::Default());
        }
        if (rigExec::RigExecOracleAttribute a = stage->GetAttributeAtPath(binds[0])) {
            a.Get(&sts, time);
        }
        if (rigExec::RigExecOracleAttribute a =
                prim.GetAttribute(_oracleToken8)) {
            a.Get(&dropoff, time);
        }
        if (!driverTransforms.empty()) {
            // Independently of the assembler: the providers' own
            // matrices, measured and weighted per control point.
            const auto phase = rigExec::RigExecPhaseForInput(prim, "rigExec:driverTransforms");
            std::unordered_map<SdfPath,GfMatrix4d,SdfPath::Hash> matrices;
            for (const char *name : {"rigExec:driverTransforms", "rigExec:driverTransformSpaces", "rigExec:driverBaseTransforms", "rigExec:driverBaseTransformSpaces", "rigExec:space"}) {
                for (const auto &path : rigExec::RigExecRelationshipTargets(prim,name)) {
                    const auto *value = ctx.phasedMatrix ? ctx.phasedMatrix(path,phase,moverPath) : nullptr;
                    if (value) matrices[path] = *value;
                }
            }
            VtFloatArray weights, baseWeights;
            if (const rigExec::RigExecOracleAttribute a = prim.GetAttribute(
                    _oracleToken9)) {
                a.Get(&weights, time);
            }
            if (const rigExec::RigExecOracleAttribute a = prim.GetAttribute(
                    _oracleToken10)) {
                a.Get(&baseWeights, time);
            }
            SdfPathVector baseTransforms, baseSpaces;
            if (rigExec::RigExecOracleRelationship rel = prim.GetRelationship(
                    _oracleToken11)) {
                rel.GetTargets(&baseTransforms);
            }
            if (rigExec::RigExecOracleRelationship rel = prim.GetRelationship(
                    _oracleToken12)) {
                rel.GetTargets(&baseSpaces);
            }
            const auto pick = [](size_t count, size_t j) {
                return count <= 1 ? size_t(0) : j % count;
            };
            bool missing = false;
            // The frame options and the carry, read independently of the
            // assembler; the arithmetic is the shared per-driver measure.
            rigExec::RigExecWireDriverFrame frame;
            TfToken pointFrame = _oracleToken21, deltaFrame = _oracleLocalFrame;
            if (const rigExec::RigExecOracleAttribute a =
                    prim.GetAttribute(_oracleToken13)) {
                a.Get(&pointFrame, time);
            }
            if (const rigExec::RigExecOracleAttribute a = prim.GetAttribute(
                    _oracleToken14)) {
                a.Get(&deltaFrame, time);
            }
            frame.posedPoints = pointFrame == "posed";
            frame.posedDelta = deltaFrame == "posed";
            SdfPathVector carries;
            if (rigExec::RigExecOracleRelationship rel =
                    prim.GetRelationship(_oracleToken15)) {
                rel.GetTargets(&carries);
            }
            GfMatrix4d carry(1.0);
            if (!carries.empty()) {
                const auto found = matrices.find(carries[0]);
                if (found == matrices.end()) {
                    missing = true;
                } else {
                    carry = found->second;
                    frame.carry = &carry;
                }
            }
            const auto measured = [&](const SdfPathVector &ts,
                                      const SdfPathVector &ss,
                                      size_t j, GfVec3d *spaceScale) {
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
                    m = rigExec::RigExecMeasureWireDriver(
                        m, sp->second, frame, spaceScale);
                }
                return m;
            };
            posedCvs = restCvs;
            for (size_t j = 0; j < restCvs.size() && !missing; ++j) {
                if (!baseTransforms.empty()) {
                    const GfMatrix4d b =
                        measured(baseTransforms, baseSpaces, j, nullptr);
                    const float wb = baseWeights.empty()
                        ? 1.0f
                        : baseWeights[pick(baseWeights.size(), j)];
                    const GfVec3f moved(
                        b.TransformAffine(GfVec3d(restCvs[j])));
                    restCvs[j] = restCvs[j] + (moved - restCvs[j]) * wb;
                }
                GfVec3d scale(1.0, 1.0, 1.0);
                const GfMatrix4d m =
                    measured(driverTransforms, driverSpaces, j, &scale);
                const float w = weights.empty()
                    ? 1.0f : weights[pick(weights.size(), j)];
                const GfVec3f moved(
                    m.TransformAffine(GfVec3d(restCvs[j])));
                GfVec3f displacement = (moved - restCvs[j]) * w;
                if (frame.posedPoints && !frame.carry) {
                    displacement = GfVec3f(
                        displacement[0] * float(scale[0]),
                        displacement[1] * float(scale[1]),
                        displacement[2] * float(scale[2]));
                }
                posedCvs[j] = restCvs[j] + displacement;
            }
            if (!missing && frame.posedPoints && frame.carry) {
                std::vector<GfVec3f> restVec(restCvs.begin(), restCvs.end());
                std::vector<GfVec3f> posedVec(posedCvs.begin(),
                                              posedCvs.end());
                rigExec::RigExecCarryWireCurves(&restVec, &posedVec, carry);
                restCvs.assign(restVec.begin(), restVec.end());
                posedCvs.assign(posedVec.begin(), posedVec.end());
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
            if (rigExec::RigExecOracleRelationship rel = prim.GetRelationship(
                    _oracleToken16)) {
                rel.GetTargets(&weightTargets);
            }
            VtIntArray indices;
            if (!weightTargets.empty()) {
                if (const rigExec::RigExecOraclePrim w =
                        stage->GetPrimAtPath(weightTargets[0])) {
                    w.GetAttribute(_oracleToken17)
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
    // The bound frame input can come from any solver kind. A source-only
    // oracle without that adapter retains the raw Ribbon sampling path.
    SdfPathVector frameTargets;
    if (rigExec::RigExecOracleRelationship rel = prim.GetRelationship(
            _oracleToken18)) {
        rel.GetTargets(&frameTargets);
    }
    if (ctx.boundFrames) {
        const auto *input = frameTargets.size() == 1
            ? ctx.boundFrames(moverPath, frameTargets[0], time) : nullptr;
        if (!input || !input->available || !input->generation ||
            input->owner != frameTargets[0] || input->time != time) {
            diagnostics->push_back("MoverFailed " + moverPath.GetString() +
                                   ": driver frame input unavailable");
            return RigExecOracleResult::PassThrough;
        }
        const auto &frames = input->value;
        const size_t count = frames.GetSize();
        if (!count || frames.rests.size() != count) {
            diagnostics->push_back("MoverFailed " + moverPath.GetString() +
                                   ": driver frame cardinality mismatch");
            return RigExecOracleResult::PassThrough;
        }
        if (mode == "emitGuidePoints") {
            if (count != points.size()) {
                diagnostics->push_back("MoverFailed " + moverPath.GetString() +
                                       ": guide cardinality mismatch");
                return RigExecOracleResult::PassThrough;
            }
            for (size_t i = 0; i < count; ++i)
                points[i] = GfVec3f(frames.frames[i].points[0]);
        } else {
            SdfPathVector binds;
            if (const auto rel = prim.GetRelationship(_oracleToken2))
                rel.GetTargets(&binds);
            VtVec2fArray coordinates;
            if (binds.size() == 1)
                stage->GetAttributeAtPath(binds[0]).Get(&coordinates, time);
            if (count < 2 || coordinates.size() != points.size()) {
                diagnostics->push_back("MoverFailed " + moverPath.GetString() +
                                       ": ribbon bind cardinality mismatch");
                return RigExecOracleResult::PassThrough;
            }
            // Evaluate each affine landmark map by scalar coordinates in
            // its rest basis. This does not call the production geometry
            // kernel or its matrix-map builder.
            const auto transport = [&](size_t sample, const GfVec3d &point) {
                const auto &rest = frames.rests[sample];
                const auto &posed = frames.frames[sample].points;
                const GfVec3d x = rest[1] - rest[0];
                const GfVec3d y = rest[2] - rest[0];
                const GfVec3d z = rest[3] - rest[0];
                const double determinant = GfDot(x, GfCross(y, z));
                const double epsilon = 1e-10 * std::max(
                    {1.0, x.GetLength(), y.GetLength(), z.GetLength()});
                if (std::abs(determinant) < epsilon * epsilon * epsilon)
                    return point;
                const GfVec3d delta = point - rest[0];
                const double a = GfDot(delta, GfCross(y, z)) / determinant;
                const double b = GfDot(x, GfCross(delta, z)) / determinant;
                const double c = GfDot(x, GfCross(y, delta)) / determinant;
                return posed[0] + (posed[1] - posed[0]) * a +
                    (posed[2] - posed[0]) * b + (posed[3] - posed[0]) * c;
            };
            for (size_t i = 0; i < points.size(); ++i) {
                const float u = std::min(1.0f, std::max(0.0f, coordinates[i][0]));
                const float position = u * float(count - 1);
                const size_t sample = std::min(count - 2, size_t(position));
                const float fraction = position - float(sample);
                const GfVec3d point(points[i]);
                const GfVec3d a = transport(sample, point);
                const GfVec3d b = transport(sample + 1, point);
                points[i] = GfVec3f(a + (b - a) * double(fraction));
            }
        }
        return RigExecOracleResult::Blend;
    }
    SdfPath curvePoints;
    int sampleCount = 5;
    if (!frameTargets.empty()) {
        if (const rigExec::RigExecOraclePrim ribbon =
                stage->GetPrimAtPath(frameTargets[0])) {
            SdfPathVector curves;
            if (rigExec::RigExecOracleRelationship rel = ribbon.GetRelationship(
                    _oracleToken1)) {
                rel.GetTargets(&curves);
            }
            if (!curves.empty()) {
                curvePoints = curves[0].IsPrimPath()
                    ? curves[0].AppendProperty(_oracleToken5)
                    : curves[0];
            }
            if (rigExec::RigExecOracleAttribute a = ribbon.GetAttribute(
                    _oracleToken19)) {
                a.Get(&sampleCount, time);
            }
        }
    }
    VtVec3fArray posedCvs, restCvs;
    if (rigExec::RigExecOracleAttribute a = stage->GetAttributeAtPath(curvePoints)) {
        a.Get(&posedCvs, time);
        a.Get(&restCvs, UsdTimeCode::Default());
    }
    if (!frameTargets.empty()) {
        const auto ribbon = stage->GetPrimAtPath(frameTargets[0]);
        const auto phase = rigExec::RigExecPhaseForInput(ribbon,"rigExec:driverCurve");
        if (!phase.IsBase() && ctx.phasedPoints) {
            const auto *value = ctx.phasedPoints(curvePoints,phase,moverPath);
            if (value && value->IsHolding<VtVec3fArray>()) posedCvs = value->UncheckedGet<VtVec3fArray>();
        }
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
        if (rigExec::RigExecOracleRelationship rel = prim.GetRelationship(
                _oracleToken2)) {
            rel.GetTargets(&binds);
        }
        VtVec2fArray sts;
        if (!binds.empty()) {
            if (rigExec::RigExecOracleAttribute a =
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

// rigExec:space, the wire's carry: at most one target, and a catalogued
// matrix provider. Refused rather than dropped, as the matrix mover does,
// because a carry that silently resolves to nothing looks exactly like
// the shear it was named to remove.
bool
_ValidateCurveMover(
    const rigExec::RigExecMoverValidateContext &ctx, std::string *error)
{
    SdfPathVector carries;
    if (UsdRelationship rel =
            ctx.prim.GetRelationship(TfToken("rigExec:space"))) {
        rel.GetTargets(&carries);
    }
    const std::string who = "CurveMover " + ctx.prim.GetPath().GetString();
    if (carries.size() > 1) {
        *error = who + ": rigExec:space takes at most one target";
        return false;
    }
    if (!carries.empty()) {
        const UsdPrim carryPrim = ctx.stage->GetPrimAtPath(carries[0]);
        const TfToken type = carryPrim ? carryPrim.GetTypeName() : TfToken();
        if (type != "RigExecControl" && type != "RigExecJoint") {
            *error = who + ": rigExec:space target is not a catalogued "
                           "matrix provider";
            return false;
        }
    }
    return true;
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
        "rigExec:driverBaseTransformSpaces", "rigExec:space"};
    handler.bind = &_BindCurveMover;
    handler.validate = &_ValidateCurveMover;
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
