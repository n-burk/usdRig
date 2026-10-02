// Geometry chains, weight fields, and skin-layout inputs.

#include "rigEvaluatorInternal.h"
#include "frameExtraction.h"
#include "rigExecMath/geometryKernels.h"
#include "rigEvaluatorConstraints.h"
#include "curvenetWeightComputations.h"
#include "movers/moverRegistry.h"
#include "rigExecMath/envelope.h"
#include "rigExecMath/weightFields.h"

#include "pxr/base/ts/spline.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usdGeom/pointBased.h"

#include <algorithm>
#include <cmath>
#include <set>
#include <unordered_map>

namespace rigExec {

using namespace evaluatorDetail;

namespace {

bool
_GetLandmarks(
    const UsdPrim &prim, const TfToken &attrName, UsdTimeCode time,
    std::array<GfVec3d, 4> *out)
{
    VtVec3dArray points;
    const UsdAttribute attr = prim.GetAttribute(attrName);
    if (!attr || !attr.Get(&points, time) || points.size() != 4) {
        return false;
    }
    std::copy(points.begin(), points.end(), out->begin());
    return true;
}

bool
_IsEnabled(const UsdPrim &mover, UsdTimeCode time)
{
    bool enabled = true;
    if (UsdAttribute a = mover.GetAttribute(_enabledAttr)) {
        a.Get(&enabled, time);
    }
    return enabled;
}

} // namespace

namespace evaluatorDetail {

/// Bakes a volumetric weight's distance-to-weight remap into the lookup
/// table its exec kernel consumes.
///
/// The named profiles bake analytically; `curve` resamples the Ts spline
/// authored on rigExec:falloffCurve. Both land in the same table, so the
/// kernel has one remap path and an author switching between a preset and
/// a hand-drawn curve changes only the numbers.
std::vector<float>
_BakeFalloffLut(const UsdPrim &prim)
{
    TfToken profile("smooth");
    if (UsdAttribute a = prim.GetAttribute(_falloffProfileAttr)) {
        a.Get(&profile);
    }
    if (profile == "linear") {
        return RigExecBuildFalloffLut(RigExecFalloffProfile::Linear);
    }
    if (profile == "smooth") {
        return RigExecBuildFalloffLut(RigExecFalloffProfile::Smooth);
    }
    if (profile == "easeIn") {
        return RigExecBuildFalloffLut(RigExecFalloffProfile::EaseIn);
    }
    if (profile == "easeOut") {
        return RigExecBuildFalloffLut(RigExecFalloffProfile::EaseOut);
    }
    if (profile == "constant") {
        return RigExecBuildFalloffLut(RigExecFalloffProfile::Constant);
    }
    if (profile != "curve") {
        return {};  // unknown token: linear, never coerced to a preset
    }

    const UsdAttribute curve = prim.GetAttribute(_falloffCurveAttr);
    if (!curve || !curve.HasSpline()) {
        // `curve` with nothing drawn is linear, not empty: the profile
        // token is a promise about SHAPE, and an author who selects it
        // before touching the editor should see the identity ramp.
        return RigExecBuildFalloffLut(RigExecFalloffProfile::Linear);
    }
    const TsSpline spline = curve.GetSpline();
    std::vector<float> lut(RigExecFalloffLutSize);
    for (size_t i = 0; i < RigExecFalloffLutSize; ++i) {
        const double x = double(i) / double(RigExecFalloffLutSize - 1);
        float value = 0.0f;
        // Ts extrapolates HELD outside the authored knot range, so a
        // curve drawn over a shorter span still yields a total field.
        if (!spline.Eval(x, &value) || !std::isfinite(value)) {
            value = float(x);
        }
        lut[i] = value;
    }
    return lut;
}

// Resolves a READ-side geometry input: naming a PointBased prim means its
// .points property, because a geometry input has exactly one thing to read.
// Property paths stay exact (spec §4.2).
// This rule is deliberately NOT applied to write targets. On the write side a
// bare prim path names the transform domain and <prim>.points names the
// geometry domain -- two different write sets on the same prim -- so inferring
// between them is what made a constraint unable to target a Mesh at all.
SdfPath
_ResolveGeometryInput(const UsdStageRefPtr &stage, const SdfPath &target)
{
    if (target.IsPrimPath()) {
        const UsdPrim prim = stage->GetPrimAtPath(target);
        if (prim && prim.IsA<UsdGeomPointBased>()) {
            return target.AppendProperty(TfToken("points"));
        }
    }
    return target;
}

} // namespace evaluatorDetail

bool
RigExecRigEvaluator::_ReadTargetPoints(
    const UsdPrim &prim, const char *relationshipName, UsdTimeCode time,
    std::vector<GfVec3f> *points) const
{
    points->clear();
    SdfPathVector targets;
    if (UsdRelationship rel = prim.GetRelationship(TfToken(relationshipName))) {
        rel.GetTargets(&targets);
    }
    if (targets.size() != 1) {
        return false;
    }
    const SdfPath canonical = _ResolveGeometryInput(_stage, targets[0]);
    const UsdAttribute attr = _stage->GetAttributeAtPath(canonical);
    VtVec3fArray value;
    if (!attr || !attr.Get(&value, time)) {
        return false;
    }
    points->assign(value.begin(), value.end());
    return true;
}

bool
RigExecRigEvaluator::_ResolveVolumeWeights(
    const UsdPrim &prim, size_t count, UsdTimeCode time,
    std::vector<float> *weights, std::string *error,
    const std::vector<GfVec3f> *currentPoints) const
{
    // Spelled only when an error needs it: this runs every time a volume
    // weight is resolved, and a resolve that succeeds reports nothing.
    const auto who = [&prim]() { return prim.GetPath().GetAsString(); };
    const TfToken typeName = prim.GetTypeName();

    // The composed field folds its inputs; it measures nothing itself.
    if (typeName == "RigExecCombineWeight") {
        SdfPathVector inputs;
        if (UsdRelationship rel =
                prim.GetRelationship(TfToken("rigExec:inputWeights"))) {
            rel.GetTargets(&inputs);
        }
        TfToken modeName("multiply");
        if (UsdAttribute a =
                prim.GetAttribute(TfToken("rigExec:combineMode"))) {
            a.Get(&modeName, time);
        }
        RigExecWeightCombine mode;
        if (modeName == "multiply") {
            mode = RigExecWeightCombine::Multiply;
        } else if (modeName == "add") {
            mode = RigExecWeightCombine::Add;
        } else if (modeName == "subtract") {
            mode = RigExecWeightCombine::Subtract;
        } else if (modeName == "max") {
            mode = RigExecWeightCombine::Max;
        } else if (modeName == "min") {
            mode = RigExecWeightCombine::Min;
        } else if (modeName == "average") {
            mode = RigExecWeightCombine::Average;
        } else if (modeName == "overlay") {
            mode = RigExecWeightCombine::Overlay;
        } else {
            *error = who() + ": unknown rigExec:combineMode " +
                     modeName.GetString();
            return false;
        }

        // Authored order, unsorted: subtract and overlay are order
        // dependent by design (see the schema doc).
        std::vector<std::vector<float>> fields;
        fields.reserve(inputs.size());
        for (const SdfPath &input : inputs) {
            std::vector<float> field;
            if (!_ResolveWeights(input, count, time, &field, error,
                                 currentPoints)) {
                return false;
            }
            fields.push_back(std::move(field));
        }
        if (!RigExecCombineWeightFields(mode, fields, count, weights)) {
            *error = who() + ": combine inputs disagree on element count";
            return false;
        }
        const float strength = _ResolvedRead(
            _resolvedInputs, prim, "inputs:strength", 1.0f, time);
        const float invert = _ResolvedRead(
            _resolvedInputs, prim, "inputs:invert", 0.0f, time);
        for (float &w : *weights) {
            w = (w + (1.0f - 2.0f * w) * invert) * strength;
        }
        return true;
    }

    // Placement.
    // Taken from the volume's own exec computeMatrix rather than
    // recomputed here. The oracle exists to check the WEIGHT FIELD math
    // independently, not the xformable frame chain -- that already has
    // its own parity coverage, and a second hand-rolled implementation
    // of posed:space + rest offsets + avars + rotation order is exactly
    // the drift frameExtraction.h was created to prevent.
    const auto matrixIt = _volumeWeightMatrices.find(prim.GetPath());
    if (matrixIt == _volumeWeightMatrices.end()) {
        *error = who() + ": no resolved placement for this volume weight";
        return false;
    }
    // Scale and shear are removed so the field matches the rigid guide a
    // viewer draws; inputs:scaleX/Y/Z is the sole authority on
    // anisotropy (see the RigExecVolumeWeight schema doc).
    GfMatrix4d rigid = matrixIt->second.RemoveScaleShear();
    const double det = rigid.GetDeterminant();
    if (!std::isfinite(det) || std::abs(det) < 1e-12) {
        *error = who() + ": degenerate volume placement";
        return false;
    }
    GfMatrix4d worldToLocal = rigid.GetInverse();

    // Which points the distance function measures.
    std::vector<GfVec3f> samplePoints;
    TfToken samplePhase("reference");
    if (UsdAttribute a = prim.GetAttribute(_samplePhaseAttr)) {
        a.Get(&samplePhase, time);
    }
    if (samplePhase == "current") {
        if (!currentPoints) {
            *error = who() +
                     ": rigExec:samplePhase is `current` but no in-flight "
                     "points were supplied";
            return false;
        }
        samplePoints = *currentPoints;
    } else if (samplePhase == "reference") {
        // An explicit sampleSource wins over the weighted domain, which
        // is how one mesh is weighted by another mesh's shape.
        if (!_ReadTargetPoints(prim, "rigExec:sampleSource", time,
                               &samplePoints) &&
            !_ReadTargetPoints(prim, "rigExec:weightTarget", time,
                               &samplePoints)) {
            *error = who() + ": could not read the points to sample";
            return false;
        }
    } else {
        *error = who() + ": unknown rigExec:samplePhase " +
                 samplePhase.GetString();
        return false;
    }
    if (samplePoints.size() != count) {
        *error = who() + ": sampled point count does not match the target";
        return false;
    }

    RigExecFalloffParams params;
    auto readFloat = [this, &prim, time](const char *name, float fallback) {
        return _ResolvedRead(
            _resolvedInputs, prim, name, fallback, time);
    };
    params.falloffMin = readFloat("inputs:falloffMin", 0.0f);
    params.falloffMax = readFloat("inputs:falloffMax", 1.0f);
    params.invert = readFloat("inputs:invert", 0.0f);
    params.strength = readFloat("inputs:strength", 1.0f);
    params.curve = _BakeFalloffLut(prim);

    if (typeName == "RigExecPlaneWeight") {
        TfToken axis("y");
        if (UsdAttribute a = prim.GetAttribute(TfToken("rigExec:planeAxis"))) {
            a.Get(&axis, time);
        }
        const int axisIndex =
            axis == "x" ? 0 : (axis == "y" ? 1 : (axis == "z" ? 2 : -1));
        if (axisIndex < 0) {
            *error = who() + ": unknown rigExec:planeAxis " + axis.GetString();
            return false;
        }
        // Bounded clips the field to the in-plane rectangle. Mirrors
        // _BuildPlaneWeightPacket exactly, including reading the extents
        // only in the bounded arm -- the two paths have to agree value
        // for value or the parity harness fires.
        TfToken boundsMode("unbounded");
        if (UsdAttribute a =
                prim.GetAttribute(TfToken("rigExec:planeBounds"))) {
            a.Get(&boundsMode, time);
        }
        RigExecPlaneBounds extent;
        const RigExecPlaneBounds *extentPtr = nullptr;
        if (boundsMode == "bounded") {
            extent.extentU = readFloat("inputs:extentU", 1.0f);
            extent.extentV = readFloat("inputs:extentV", 1.0f);
            for (const float e : {extent.extentU, extent.extentV}) {
                if (!std::isfinite(e) || e <= 0.0f) {
                    *error = who() +
                             ": inputs:extentU/V must be finite and positive "
                             "when rigExec:planeBounds is `bounded`";
                    return false;
                }
            }
            extentPtr = &extent;
        } else if (boundsMode != "unbounded") {
            *error = who() + ": unknown rigExec:planeBounds " +
                     boundsMode.GetString();
            return false;
        }
        RigExecPlaneWeightField(
            samplePoints, worldToLocal, axisIndex, params, weights, extentPtr);
        return true;
    }

    // Sphere and curve both take the per-axis divisors, folded into the
    // matrix so the hot loop stays one transform.
    const float sx = readFloat("inputs:scaleX", 1.0f);
    const float sy = readFloat("inputs:scaleY", 1.0f);
    const float sz = readFloat("inputs:scaleZ", 1.0f);
    for (float s : {sx, sy, sz}) {
        if (!std::isfinite(s) || s <= 0.0f) {
            *error = who() + ": inputs:scaleX/Y/Z must be finite and positive";
            return false;
        }
    }
    GfMatrix4d divide(1.0);
    divide.SetScale(GfVec3d(1.0 / double(sx), 1.0 / double(sy),
                            1.0 / double(sz)));
    worldToLocal = worldToLocal * divide;

    if (typeName == "RigExecSphereWeight") {
        const GfVec3f positiveScales(
            readFloat("inputs:scaleXPos", 1.0f),
            readFloat("inputs:scaleYPos", 1.0f),
            readFloat("inputs:scaleZPos", 1.0f));
        const GfVec3f negativeScales(
            readFloat("inputs:scaleXNeg", 1.0f),
            readFloat("inputs:scaleYNeg", 1.0f),
            readFloat("inputs:scaleZNeg", 1.0f));
        for (int axis = 0; axis < 3; ++axis) {
            if (!std::isfinite(positiveScales[axis]) || positiveScales[axis] <= 0 ||
                !std::isfinite(negativeScales[axis]) || negativeScales[axis] <= 0) {
                *error = who() + ": signed axis scales must be finite and positive";
                return false;
            }
        }
        RigExecSphereWeightField(samplePoints, worldToLocal, params, weights,
                                 positiveScales, negativeScales);
        return true;
    }
    if (typeName == "RigExecCurveWeight") {
        std::vector<GfVec3f> curvePoints;
        if (!_ReadTargetPoints(prim, "rigExec:curve", time, &curvePoints) ||
            curvePoints.empty()) {
            *error = who() + ": rigExec:curve must name exactly one points source";
            return false;
        }
        RigExecCurveWeightField(
            samplePoints, curvePoints, worldToLocal, params, weights);
        return true;
    }
    *error = who() + ": not a volumetric weight object";
    return false;
}

bool
RigExecRigEvaluator::_ResolveWeights(
    const SdfPath &weightPrimPath, size_t count, UsdTimeCode time,
    std::vector<float> *weights, std::string *error,
    const std::vector<GfVec3f> *currentPoints) const
{
    weights->assign(count, 1.0f);
    const UsdPrim prim = _stage->GetPrimAtPath(weightPrimPath);
    if (!prim) {
        *error = "missing weight object " + weightPrimPath.GetString();
        return false;
    }

    // A type this oracle does not understand must FAIL, never fall
    // through to the authored-table path. That path reads no
    // representation and no defaultWeight off a volumetric prim and so
    // returns an all-zero field and `true` -- a silently wrong answer,
    // and the one shape of bug the parity harness cannot catch because
    // both sides would agree on nothing.
    const TfToken typeName = prim.GetTypeName();
    if (!_IsWeightObjectType(typeName)) {
        *error = "unknown weight object type " + typeName.GetString() +
                 " on " + weightPrimPath.GetString();
        return false;
    }
    if (typeName == "RigExecCurvenetWeight") {
        auto array = [&](const char *relationship, auto *out) {
            SdfPathVector paths;
            prim.GetRelationship(TfToken(relationship)).GetTargets(&paths);
            return paths.size() == 1 && _resolvedInputs.GetAttribute(
                _stage->GetAttributeAtPath(paths[0]), time, out);
        };
        VtVec3fArray mesh, net;
        VtIntArray counts, indices, splines, smooth;
        VtFloatArray authored;
        if (!array("rigExec:weightTarget", &mesh) || !array("rigExec:curvenetPoints", &net) ||
            !array("rigExec:meshFaceCounts", &counts) || !array("rigExec:meshFaceIndices", &indices) ||
            !array("rigExec:curvenetSplineIndices", &splines)) {
            *error = "unresolved curvenet weight geometry"; return false;
        }
        _resolvedInputs.GetAttribute(prim.GetAttribute(TfToken("inputs:weights")), time, &authored);
        _resolvedInputs.GetAttribute(prim.GetAttribute(TfToken("rigExec:autoSmooth")), time, &smooth);
        const auto packet = RigExecComputeCurvenetWeightPacket(
            {mesh.begin(),mesh.end()}, {counts.begin(),counts.end()}, {indices.begin(),indices.end()},
            {net.begin(),net.end()}, {splines.begin(),splines.end()},
            _ResolvedRead(_resolvedInputs,prim,"rigExec:basis",TfToken("catmullRom"),time),
            _ResolvedRead(_resolvedInputs,prim,"rigExec:samplesPerSpline",5,time),
            {smooth.begin(),smooth.end()}, {authored.begin(),authored.end()},
            _ResolvedRead(_resolvedInputs,prim,"rigExec:rangePolicy",TfToken("clamp"),time),
            _ResolvedRead(_resolvedInputs,prim,"rigExec:unreachedValue",0.0f,time),error);
        return packet.ResolveAll(count, weights);
    }
    if (_IsVolumeWeightType(typeName) || typeName == "RigExecCombineWeight") {
        std::vector<float> resolved;
        if (!_ResolveVolumeWeights(prim, count, time, &resolved, error,
                                   currentPoints)) {
            return false;
        }
        TfToken volumePolicy("clamp");
        if (UsdAttribute a =
                prim.GetAttribute(TfToken("rigExec:rangePolicy"))) {
            a.Get(&volumePolicy, time);
        }
        for (float &w : resolved) {
            if (!std::isfinite(w)) {
                *error = "non-finite weight on " + weightPrimPath.GetString();
                return false;
            }
            if (w < 0.0f || w > 1.0f) {
                if (volumePolicy != "clamp") {
                    *error = "strict range violation on " +
                             weightPrimPath.GetString();
                    return false;
                }
                w = std::min(std::max(w, 0.0f), 1.0f);
            }
        }
        *weights = std::move(resolved);
        return true;
    }

    TfToken representation("constant");
    if (UsdAttribute a = prim.GetAttribute(
            TfToken("rigExec:representation"))) {
        a.Get(&representation, time);
    }
    const float defaultWeight = _ResolvedRead(
        _resolvedInputs, prim, "rigExec:defaultWeight", 0.0f, time);
    TfToken rangePolicy("strict");
    if (UsdAttribute a = prim.GetAttribute(TfToken("rigExec:rangePolicy"))) {
        a.Get(&rangePolicy, time);
    }
    if (rangePolicy != "strict" && rangePolicy != "clamp") {
        *error = "unknown rangePolicy on " + weightPrimPath.GetString();
        return false;
    }

    const bool isDynamic = prim.GetTypeName() == "RigExecDynamicWeight";
    if (isDynamic) {
        TfToken operation("multiply");
        if (UsdAttribute a =
                prim.GetAttribute(TfToken("rigExec:operation"))) {
            a.Get(&operation, time);
        }
        if (operation != "multiply") {
            *error = "unknown dynamic-weight operation on " +
                     weightPrimPath.GetString();
            return false;
        }

        // Base field first, then r_i = (b_i * d) * s + a (spec §4.1).
        std::vector<float> base(count, 1.0f);
        SdfPathVector baseTargets;
        if (UsdRelationship rel =
                prim.GetRelationship(TfToken("rigExec:baseWeight"))) {
            rel.GetTargets(&baseTargets);
        }
        if (baseTargets.size() > 1) {
            *error = "rigExec:baseWeight must have at most one target on " +
                     weightPrimPath.GetString();
            return false;
        }
        if (baseTargets.empty()) {
            // Without a base, only constant representation is legal and
            // b_i = 1 everywhere (spec §4.1).
            if (representation != "constant") {
                *error = "no-base dynamic weight must be constant on " +
                         weightPrimPath.GetString();
                return false;
            }
        } else {
            const UsdPrim basePrim = _stage->GetPrimAtPath(baseTargets[0]);
            if (!basePrim) {
                *error = "missing base weight object on " +
                         weightPrimPath.GetString();
                return false;
            }
            // The base descriptor must exactly match the dynamic
            // descriptor: canonical target, representation, and sparse
            // support (spec §4.1).
            auto canonicalWeightTarget =
                [this, &time](const UsdPrim &p) -> SdfPath {
                SdfPathVector t;
                if (UsdRelationship rel = p.GetRelationship(
                        TfToken("rigExec:weightTarget"))) {
                    rel.GetTargets(&t);
                }
                return t.size() == 1 ? _ResolveGeometryInput(_stage, t[0])
                                     : SdfPath();
            };
            if (canonicalWeightTarget(prim) !=
                    canonicalWeightTarget(basePrim) ||
                canonicalWeightTarget(prim).IsEmpty()) {
                *error = "dynamic/base weight target mismatch on " +
                         weightPrimPath.GetString();
                return false;
            }
            TfToken baseRepresentation("constant");
            if (UsdAttribute a = basePrim.GetAttribute(
                    TfToken("rigExec:representation"))) {
                a.Get(&baseRepresentation, time);
            }
            if (baseRepresentation != representation) {
                *error = "dynamic/base representation mismatch on " +
                         weightPrimPath.GetString();
                return false;
            }
            if (representation == "sparse") {
                // The dynamic descriptor's sparse support is inherited
                // from the base; a dynamic prim that authors its own
                // support must match the base exactly (spec §4.1).
                VtIntArray mine;
                if (UsdAttribute a =
                        prim.GetAttribute(TfToken("rigExec:indices"))) {
                    a.Get(&mine, time);
                }
                if (!mine.empty()) {
                    VtIntArray theirs;
                    if (UsdAttribute a = basePrim.GetAttribute(
                            TfToken("rigExec:indices"))) {
                        a.Get(&theirs, time);
                    }
                    const std::set<int> mySupport(mine.begin(), mine.end());
                    const std::set<int> baseSupport(
                        theirs.begin(), theirs.end());
                    if (mySupport != baseSupport) {
                        *error = "dynamic/base sparse support mismatch on " +
                                 weightPrimPath.GetString();
                        return false;
                    }
                }
            }
            // currentPoints is forwarded: a dynamic weight modulating a
            // current-phase volume must still measure against the
            // in-flight points, or the base silently reverts to the
            // reference field.
            if (!_ResolveWeights(baseTargets[0], count, time, &base, error,
                                 currentPoints)) {
                return false;
            }
        }
        const float driver = _ResolvedRead(
            _resolvedInputs, prim, "inputs:driver", 1.0f, time);
        const float scale = _ResolvedRead(
            _resolvedInputs, prim, "inputs:scale", 1.0f, time);
        const float bias = _ResolvedRead(
            _resolvedInputs, prim, "inputs:bias", 0.0f, time);
        for (size_t i = 0; i < count; ++i) {
            float r = (base[i] * driver) * scale + bias;
            if (!std::isfinite(r)) {
                *error = "non-finite dynamic weight on " +
                         weightPrimPath.GetString();
                return false;
            }
            if (r < 0.0f || r > 1.0f) {
                if (rangePolicy == "clamp") {
                    r = std::min(std::max(r, 0.0f), 1.0f);
                } else {
                    *error = "strict range violation on " +
                             weightPrimPath.GetString();
                    return false;
                }
            }
            (*weights)[i] = r;
        }
        return true;
    }

    // Static weights are time-invariant by contract: reject time samples
    // and value connections on every field (spec §4.1).
    static const TfToken staticFields[] = {
        TfToken("rigExec:values"), TfToken("rigExec:indices"),
        TfToken("rigExec:defaultWeight"), TfToken("rigExec:representation"),
        TfToken("rigExec:rangePolicy")};
    for (const TfToken &field : staticFields) {
        const UsdAttribute a = prim.GetAttribute(field);
        if (a && (a.GetNumTimeSamples() > 0 || a.HasAuthoredConnections())) {
            *error = "static weight field " + field.GetString() +
                     " has time samples or connections on " +
                     weightPrimPath.GetString();
            return false;
        }
    }
    VtFloatArray values;
    if (UsdAttribute a = prim.GetAttribute(TfToken("rigExec:values"))) {
        a.Get(&values, time);
    }
    if (representation == "constant") {
        if (!values.empty()) {
            *error = "constant weight must not author values on " +
                     weightPrimPath.GetString();
            return false;
        }
        weights->assign(count, defaultWeight);
    } else if (representation == "dense") {
        if (values.size() != count) {
            *error = "dense weight cardinality mismatch on " +
                     weightPrimPath.GetString();
            return false;
        }
        // Canonical encoding requires dense defaultWeight = 0 (spec §4.1).
        if (defaultWeight != 0.0f) {
            *error = "dense weight requires canonical defaultWeight 0 on " +
                     weightPrimPath.GetString();
            return false;
        }
        weights->assign(values.begin(), values.end());
    } else if (representation == "sparse") {
        VtIntArray indices;
        if (UsdAttribute a = prim.GetAttribute(TfToken("rigExec:indices"))) {
            a.Get(&indices, time);
        }
        if (indices.size() != values.size()) {
            *error = "sparse index/value size mismatch on " +
                     weightPrimPath.GetString();
            return false;
        }
        weights->assign(count, defaultWeight);
        std::set<int> seen;
        for (size_t i = 0; i < indices.size(); ++i) {
            if (indices[i] < 0 || static_cast<size_t>(indices[i]) >= count) {
                *error = "sparse index out of range on " +
                         weightPrimPath.GetString();
                return false;
            }
            // Indices are unique logical element indices; authored pair
            // order is non-semantic (spec §4.1).
            if (!seen.insert(indices[i]).second) {
                *error = "duplicate sparse index on " +
                         weightPrimPath.GetString();
                return false;
            }
            (*weights)[indices[i]] = values[i];
        }
    } else {
        *error = "unknown weight representation on " +
                 weightPrimPath.GetString();
        return false;
    }

    for (float w : *weights) {
        if (!std::isfinite(w) ||
            (rangePolicy == "strict" && (w < 0.0f || w > 1.0f))) {
            *error = "weight range violation on " +
                     weightPrimPath.GetString();
            return false;
        }
    }
    if (rangePolicy == "clamp") {
        for (float &w : *weights) {
            w = std::min(std::max(w, 0.0f), 1.0f);
        }
    }
    return true;
}

VtVec3fArray
RigExecRigEvaluator::_EvaluateChain(
    const SdfPath &target,
    const std::vector<const RigExecMoverRecord *> &chain,
    const RigExecRigPose &pose,
    const std::unordered_map<SdfPath, GfMatrix4d, SdfPath::Hash> &baseProviderMatrices,
    const std::unordered_map<SdfPath, GfMatrix4d, SdfPath::Hash> &finalProviderMatrices,
    UsdTimeCode time,
    std::vector<std::string> *diagnostics,
    const std::unordered_map<SdfPath, GfMatrix4d, SdfPath::Hash> &geometryConstraintDeltas) const
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
    const UsdAttribute baseAttr = _stage->GetAttributeAtPath(target);
    if (!baseAttr || !baseAttr.Get(&points, time)) {
        diagnostics->push_back("no base value for " + target.GetString());
        return points;
    }
    const VtVec3fArray basePoints = points;

    for (const RigExecMoverRecord *mover : chain) {
        const UsdPrim prim = _stage->GetPrimAtPath(mover->moverPath);
        if (!prim || !_IsEnabled(prim, time)) {
            continue;  // pass-through (spec §4.2)
        }

        // Resolve the common envelope against the PRECEDING revision. A
        // current-phase volume therefore measures exactly the points that
        // enter this mover. A bound object supersedes inputs:defaultWeight.
        const VtVec3fArray preceding = points;
        std::vector<float> envelope(points.size(), 1.0f);
        SdfPathVector weightObjects;
        if (const UsdRelationship rel =
                prim.GetRelationship(TfToken("rigExec:weightObject"))) {
            rel.GetTargets(&weightObjects);
        }
        if (!weightObjects.empty()) {
            const std::vector<GfVec3f> currentPoints(
                preceding.begin(), preceding.end());
            std::string error;
            if (!_ResolveWeights(weightObjects[0], points.size(), time,
                                 &envelope, &error, &currentPoints)) {
                diagnostics->push_back(
                    "MoverFailed " + mover->moverPath.GetString() + ": " +
                    error);
                continue;
            }
        } else {
            const float scalar = _ResolvedRead(
                _resolvedInputs, prim, "inputs:defaultWeight", 1.0f, time);
            if (!std::isfinite(scalar) || scalar < 0.0f || scalar > 1.0f) {
                diagnostics->push_back(
                    "MoverFailed " + mover->moverPath.GetString() +
                    ": inputs:defaultWeight must be finite and in [0, 1]");
                continue;
            }
            std::fill(envelope.begin(), envelope.end(), scalar);
        }

        if (_IsSourceFrameConstraintType(prim.GetTypeName())) {
            const auto delta = geometryConstraintDeltas.find(mover->moverPath);
            if (delta == geometryConstraintDeltas.end()) continue;
            for (auto &point : points)
                point = GfVec3f(delta->second.TransformAffine(GfVec3d(point)));
        } else if (const RigExecMoverHandler *oracleHandler =
                       RigExecFindMoverHandler(mover->schemaType)) {
            // The parity-oracle branch, from the mover's own row (see movers/).
            // A type with no row, or a row with no oracle, falls through to
            // the shared envelope blend over the unmodified points.
            if (oracleHandler->oracle) {
                // A dense blend sample phased read answers from the
                // recorded snapshot, resolved through this revision
                // compiled binding -- the one piece of evaluator
                // compile state the oracle needs.
                auto sampleSnapshot = [this, &target, mover](
                    const SdfPath &inputPath,
                    const SdfPath &samplePath) -> const VtValue * {
                    const auto compiledChain =
                        _graphChains.find(target);
                    if (compiledChain == _graphChains.end()) {
                        return nullptr;
                    }
                    for (const auto &revision :
                         compiledChain->second) {
                        if (revision.moverPath != mover->moverPath) {
                            continue;
                        }
                        const auto channel =
                            revision.binding.blendSamples.find(inputPath);
                        if (channel ==
                            revision.binding.blendSamples.end()) {
                            continue;
                        }
                        for (const auto &binding : channel->second) {
                            if (binding.sample != samplePath) {
                                continue;
                            }
                            return _chainSnapshots.Lookup(
                                binding.points, binding.phase,
                                mover->moverPath);
                        }
                    }
                    return nullptr;
                };
                const RigExecMoverOracleContext oracleCtx{
                    _stage,
                    prim,
                    mover->moverPath,
                    target,
                    time,
                    _resolvedInputs,
                    _chainSnapshots,
                    baseProviderMatrices,
                    finalProviderMatrices,
                    diagnostics,
                    &points,
                    basePoints,
                    sampleSnapshot};
                if (oracleHandler->oracle(oracleCtx) ==
                    RigExecOracleResult::PassThrough) {
                    continue;
                }
            }
        }

        // Every branch above computes the operation's full-strength
        // candidate. The universal envelope is the one and only blend back
        // over the preceding revision.
        if (points.size() != preceding.size() ||
            envelope.size() != points.size()) {
            diagnostics->push_back(
                "MoverFailed " + mover->moverPath.GetString() +
                ": result cardinality changed; revision passed through");
            points = preceding;
            continue;
        }
        for (size_t i = 0; i < points.size(); ++i) {
            points[i] = RigExecBlendEnvelope(
                preceding[i], points[i], envelope[i]);
        }
    }
    return points;
}

void
RigExecRigEvaluator::_ResolveSkinLayoutInputs() const
{
    _skinLayoutInputs.clear();
    _skinLayoutInputsValid = true;
    for (const RigExecMoverRecord &record : _movers) {
        const RigExecMoverHandler *layoutHandler =
            RigExecFindMoverHandler(record.schemaType);
        if (!layoutHandler || layoutHandler->layoutAttributes.empty()) {
            continue;
        }
        const UsdPrim prim = _stage->GetPrimAtPath(record.moverPath);
        if (!prim) {
            continue;
        }
        for (const TfToken &name : layoutHandler->layoutAttributes) {
            // The SAME walk the value is read through
            // (RigExecResolvedInputs::GetAttribute): a single authored
            // connection per hop, the resolved map consulted at every hop,
            // cycles refused. Every path along it is a path an override can
            // stand on and be seen by the read, so every path along it
            // belongs in this set -- an override one hop upstream of a
            // connected rigExec:jointIndices is the case that makes the
            // difference between a re-read and a silently stale deformation.
            UsdAttribute attribute = prim.GetAttribute(name);
            if (!attribute) {
                // Not authored and not in the schema: an override could
                // still create the opinion the read would find, so the
                // property itself is named even where the attribute is not.
                _skinLayoutInputs.insert(
                    record.moverPath.AppendProperty(name));
                continue;
            }
            while (attribute &&
                   _skinLayoutInputs.insert(attribute.GetPath()).second) {
                SdfPathVector connections;
                if (attribute.HasAuthoredConnections()) {
                    attribute.GetConnections(&connections);
                }
                if (connections.size() != 1) {
                    break;
                }
                attribute = _stage->GetAttributeAtPath(connections[0]);
            }
        }
    }
}

bool
RigExecRigEvaluator::_OverridesReachSkinLayout(
    const std::vector<RigExecValueOverride> &overrides) const
{
    if (overrides.empty()) {
        return false;
    }
    if (!_skinLayoutInputsValid) {
        _ResolveSkinLayoutInputs();
    }
    for (const RigExecValueOverride &o : overrides) {
        // A computation override names no property, so there is nothing to
        // compare it against: it is taken to reach everything.
        if (o.attribute.IsEmpty()) {
            return true;
        }
        if (_skinLayoutInputs.count(o.prim.AppendProperty(o.attribute))) {
            return true;
        }
    }
    return false;
}

size_t
RigExecRigEvaluator::GetSkinTopologyCacheSize() const
{
    return _skinTopologies.GetSize();
}

size_t
RigExecRigEvaluator::GetBlendSampleCacheSize() const
{
    return _blendSampleShapes.GetSize();
}

void
RigExecRigEvaluator::_ApplySurfaceProjectors(RigExecRigPose *pose,
                                             UsdTimeCode time) const
{
    for (const _SurfaceProjectorRecord &record : _surfaceProjectors) {
        const SdfPath meshPath = record.target.GetPrimPath();
        const UsdPrim mesh = _stage->GetPrimAtPath(meshPath);
        if (!mesh) {
            pose->diagnostics.push_back(
                "SurfaceProjector " + record.path.GetString() +
                ": no prim at " + meshPath.GetString());
            continue;
        }
        VtVec3fArray base;
        if (const UsdAttribute a = _stage->GetAttributeAtPath(record.target)) {
            a.Get(&base, time);
        }
        // The POSED points, if a deformer produced any. A projector on an
        // undeformed surface is not an error -- the delta is simply
        // identity, which is what an unposed rig should publish.
        VtVec3fArray posed = base;
        const auto it = pose->movedProperties.find(record.target);
        if (it != pose->movedProperties.end() &&
            it->second.IsHolding<VtVec3fArray>()) {
            posed = it->second.UncheckedGet<VtVec3fArray>();
        }
        VtIntArray counts, indices;
        mesh.GetAttribute(TfToken("faceVertexCounts")).Get(&counts, time);
        mesh.GetAttribute(TfToken("faceVertexIndices")).Get(&indices, time);
        if (base.empty() || counts.empty() || posed.size() != base.size()) {
            pose->diagnostics.push_back(
                "SurfaceProjector " + record.path.GetString() +
                ": surface has no usable points or topology");
            continue;
        }
        const std::vector<int> faceCounts(counts.begin(), counts.end());
        const std::vector<int> faceIndices(indices.begin(), indices.end());
        const std::vector<GfVec3f> basePoints(base.begin(), base.end());
        const std::vector<GfVec3f> posedPoints(posed.begin(), posed.end());

        // The ray is cast at the BASE points, and what it finds is a
        // MATERIAL point: a triangle and a place inside it. The posed
        // points then say where that point went and which way the surface
        // faces there. Re-casting at the posed surface would find where
        // the deformed surface happens to cross the same fixed line, which
        // is a different material point whenever the surface slides across
        // the ray -- and the socket stretch slides the whole cornea up it.
        // Read that way the iris centre's motion was invisible: the hit
        // crept back along the ray and tilted, while the material under
        // the projector had moved a third of an eye radius.
        // The ray comes from the SOURCE frame when there is one -- the
        // eye's bind joint -- expressed in the surface's own space. The
        // authored rayOrigin/rayDirection are the fallback for a
        // projector with no source.
        //
        // Not optional once the surface carries its points in asset
        // space: an origin of (0,0,0) in object space is then the world
        // origin, metres from the eyeball, and the ray misses entirely.
        GfVec3d rayOrigin = record.rayOrigin;
        GfVec3d rayDirection = record.rayDirection;
        GfVec3d rayUp = record.rayUp;
        const auto frameOf = [&](const std::map<SdfPath,
                                                RigExecPointFrame> &frames,
                                 const SdfPath &path, GfMatrix4d *out) {
            const auto it = frames.find(path);
            return it != frames.end() &&
                   RigExecPointsToMatrix(RigExecIdentityLandmarks(),
                                         it->second.points, out);
        };
        if (!record.source.IsEmpty() && !record.sourceSpace.IsEmpty()) {
            // THE RAY IN THE SOURCE'S SIBLING SPACE.
            //
            // The old route mapped the source's WORLD frame into the
            // mesh's object space through UsdGeomXformCache. That cache
            // reads the STAGE, and the eye mesh's stage transform is the
            // identity -- the rig moves its points, not its xform -- so
            // the ray origin came out as the bind's world position, 164
            // units from a unit eyeball authored at the origin. It hit
            // only because the direction happened to point back at the
            // ball, and past about 20 degrees of head turn it stopped
            // hitting at all: measured, the projector published no matrix
            // for either eye at head ry 30, in BOTH projection modes.
            //
            // Measuring the source against a sibling space instead --
            // eye_?_bind against eyeSocket_?_bind -- gives the eye's own
            // motion with the head's taken out, which is the space the
            // points are authored in. Head and face_upper then cancel
            // exactly rather than being carried in and missing.
            GfMatrix4d sourceRest(1.0), sourceFinal(1.0);
            GfMatrix4d spaceRest(1.0), spaceFinal(1.0);
            if (frameOf(pose->jointFramesBase, record.source, &sourceRest) &&
                frameOf(pose->jointFramesFinal, record.source, &sourceFinal) &&
                frameOf(pose->jointFramesBase, record.sourceSpace,
                        &spaceRest) &&
                frameOf(pose->jointFramesFinal, record.sourceSpace,
                        &spaceFinal)) {
                const GfMatrix4d restLocal =
                    sourceRest * spaceRest.GetInverse();
                const GfMatrix4d posedLocal =
                    sourceFinal * spaceFinal.GetInverse();
                // The source's motion within its space, applied to the
                // authored rest ray. At rest this is the identity, so the
                // ray is exactly rigExec:rayOrigin / rayDirection.
                const GfMatrix4d motion =
                    restLocal.GetInverse() * posedLocal;
                rayOrigin = motion.Transform(record.rayOrigin);
                rayDirection = motion.TransformDir(record.rayDirection);
                rayUp = motion.TransformDir(record.rayUp);
            }
        } else if (!record.source.IsEmpty()) {
            GfMatrix4d sourceFrame(1.0);
            if (frameOf(pose->jointFramesFinal, record.source, &sourceFrame)) {
                UsdGeomXformCache rayCache(time);
                const GfMatrix4d toMesh =
                    rayCache.GetLocalToWorldTransform(mesh).GetInverse();
                const GfMatrix4d local = sourceFrame * toMesh;
                rayOrigin = local.ExtractTranslation();
                rayDirection = GfVec3d(local.GetRow3(2));
                rayUp = GfVec3d(local.GetRow3(1));
            }
        }

        // A RAY PER CONFIGURATION. The one above is the POSED ray: it
        // comes from the source's final frame, so it travels with the
        // head. The first cast, though, is against the BASE points, and
        // those do not travel with anything -- so casting the posed ray
        // at them walks the origin out of the rest surface as soon as the
        // head turns far enough. Measured on the biped: at head ry 30 the
        // origin sat at (5.369, 161.956, 1.379), 3.07 from a rest eyeball
        // of radius 2.48, and the cast simply missed. Both eyes, every
        // projection mode, and the projector published nothing at all.
        //
        // So the rest cast gets a REST ray, built the same way from the
        // source's base frame. Each cast then meets the surface it is
        // aimed at, and the head cancels because it moves both.
        GfVec3d restRayOrigin = rayOrigin;
        GfVec3d restRayDirection = rayDirection;
        GfVec3d restRayUp = rayUp;
        if (!record.source.IsEmpty() && record.sourceSpace.IsEmpty()) {
            GfMatrix4d sourceBase(1.0);
            if (frameOf(pose->jointFramesBase, record.source, &sourceBase)) {
                UsdGeomXformCache rayCache(time);
                const GfMatrix4d toMesh =
                    rayCache.GetLocalToWorldTransform(mesh).GetInverse();
                const GfMatrix4d local = sourceBase * toMesh;
                restRayOrigin = local.ExtractTranslation();
                restRayDirection = GfVec3d(local.GetRow3(2));
                restRayUp = GfVec3d(local.GetRow3(1));
            }
        }

        // AND THE MASTERS COME BACK OFF IT. The rest ray above is built
        // from the source's BASE frame, and a base frame still composes
        // through the masters' posed avars -- while the base POINTS it is
        // about to be cast at are the mesh exactly as authored, which
        // carry nothing. At the origin the two agree and the cast lands.
        // Move Main forty units and the ray starts forty units away from
        // the rest eyeball: measured at o(43.5628 164.3010 24.1881)
        // against a ball of radius 2.48, cast=MISS, restFrame=FAILED, and
        // the projector published NOTHING at all -- both eyes lost their
        // iris the moment the character left the origin.
        //
        // The carry is rest-inverse-times-posed on whatever rigExec:space
        // names, so a master sitting at its rest contributes identity and
        // a rig that names no space is bit for bit what it was.
        // The masters' uniform scale, taken off the rest ray below and
        // put back on the shader matrix further down. 1.0 when the rig
        // names no space or the masters stand at their rest.
        double spaceScale = 1.0;
        if (!record.space.IsEmpty()) {
            // THE SPACE IS NORMALLY A CONTROL, AND CONTROLS ARE NOT JOINTS.
            //
            // This looked only in the joint frame maps, and a rig names the
            // innermost TRS MASTER here -- /Biped/Rig/Main/Shot/Aux, a
            // RigExecControl. The lookup missed every time, the correction
            // never ran, and the failure was silent: deleting the
            // relationship altogether changed nothing, which is how it was
            // found. Measured on the biped, both eyes lost their iris the
            // moment the character left the origin -- ray
            // o(43.3842 166.3989 22.1251) against a rest eyeball at
            // (-3.3828 164.3012 1.3808), cast=MISS, and the projector
            // published no shader matrix at all.
            //
            // AND THE QUANTITY WAS THE WRONG ONE. base^-1 * final is the
            // PRE- to POST-CONSTRAINT delta, not rest to pose. A master is
            // not constrained, so that product is the identity and the
            // correction was a no-op even where the lookup succeeded:
            // pointing rigExec:space at a joint instead did not help, which
            // is what ruled the lookup out as the whole story.
            //
            // The carry is the space's POSED frame, exactly as
            // RigExecClusterInPointFrame takes it, and it carries that
            // function's contract with it: a space standing at its rest
            // must measure the identity. The biped's masters do, measured.
            GfMatrix4d carry(1.0);
            bool haveCarry =
                frameOf(pose->jointFramesFinal, record.space, &carry);
            if (!haveCarry) {
                const auto it = pose->controlFrames.find(record.space);
                haveCarry = it != pose->controlFrames.end() &&
                            RigExecPointsToMatrix(RigExecIdentityLandmarks(),
                                                  it->second.points, &carry);
            }
            if (haveCarry) {
                const GfMatrix4d back = carry.GetInverse();
                restRayOrigin = back.Transform(restRayOrigin);
                restRayDirection = back.TransformDir(restRayDirection);
                restRayUp = back.TransformDir(restRayUp);
                // THE MASTERS' SCALE LIVES HERE AND NOWHERE ELSE.
                // A base frame already composes through the masters'
                // posed avars -- this file says so a few lines up -- so
                // the source's own rest-to-posed ratio sees the master
                // scale in BOTH frames and cancels it. Measured: with
                // only the source ratio, Head x2 scaled the projector
                // correctly and Main x2 left it at 2.4829. The carry is
                // the master's posed frame and measures identity at
                // rest, so its scale is the missing factor and costs
                // nothing when no master is scaled.
                double carried = 0.0;
                for (int i = 0; i < 3; ++i) {
                    carried += GfVec3d(carry.GetRow3(i)).GetLength();
                }
                if (carried > 1e-9 && std::isfinite(carried)) {
                    spaceScale = carried / 3.0;
                }
            } else {
                pose->diagnostics.push_back(
                    "SurfaceProjector " + record.path.GetString() +
                    ": rigExec:space names " + record.space.GetString() +
                    ", which is neither a joint nor a control frame; the "
                    "rest ray keeps the masters and will miss once the rig "
                    "leaves its rest");
            }
        }

        // WHICH SURFACE THE POSED FRAME IS READ FROM. See
        // rigExec:projectionMode in the schema.
        //
        //   material   one ray, cast at the base surface; the posed frame
        //              is that same material point after deformation. The
        //              projector then rides the skin, and the socket
        //              stretch carries the iris with it.
        //   reproject  the ray is cast AGAIN at the posed surface, so the
        //              frame is wherever the look meets the deformed
        //              eyeball now. This is what the Maya rig does:
        //              eye_?_pupil_bindplane is skinned to eye_?_bind for
        //              the look and then SHRINKWRAPPED onto
        //              eye_?_projection_sphere, and a shrink wrap
        //              re-projects every frame rather than following a
        //              material point.
        TfToken projection("material");
        if (const UsdPrim projector = _stage->GetPrimAtPath(record.path)) {
            if (const UsdAttribute a = projector.GetAttribute(
                    TfToken("rigExec:projectionMode"))) {
                a.Get(&projection);
            }
        }

        RigExecSurfaceHit hit;
        GfMatrix4d restFrame(1.0), posedFrame(1.0);
        bool ok = RigExecRaycastSurface(basePoints, faceCounts, faceIndices,
                                        restRayOrigin, restRayDirection,
                                        &hit) &&
                  RigExecSurfaceFrameAtHit(basePoints, faceCounts,
                                           faceIndices, hit, restRayUp,
                                           &restFrame);
        bool reprojected = false;
        if (ok && projection == "reproject") {
            // A second cast, against the deformed surface. The rest frame
            // stays the first cast's: it is the projector's own placement,
            // and re-deriving it per frame would move the thing the offset
            // is quoted against.
            RigExecSurfaceHit posedHit;
            reprojected =
                RigExecRaycastSurface(posedPoints, faceCounts, faceIndices,
                                      rayOrigin, rayDirection, &posedHit) &&
                RigExecSurfaceFrameAtHit(posedPoints, faceCounts,
                                         faceIndices, posedHit, rayUp,
                                         &posedFrame);
            if (!reprojected) {
                // THE MATERIAL POINT IS THE FALLBACK, not nothing.
                //
                // The ray is built from the eye bind, and the bind does
                // not follow the head wire, so pushing the wire far
                // enough walks the deformed eyeball out from under the
                // ray: measured on the biped, M_HeadwireMid tx = 2.5 is
                // the last value that re-casts and tx = 3 misses. Until
                // then the drift is 1.4-2.2% of the ball's radius.
                //
                // Dropping the projector there took BOTH irises off the
                // face at once -- a hard cliff mid-drag, with the
                // material answer sitting right there and no worse than
                // the frame before it. So the miss degrades to the
                // material point and says so, rather than publishing
                // nothing.
                ok = RigExecSurfaceFrameAtHit(posedPoints, faceCounts,
                                              faceIndices, hit, rayUp,
                                              &posedFrame);
                if (ok) {
                    pose->diagnostics.push_back(
                        "SurfaceProjector " + record.path.GetString() +
                        ": the re-cast missed the deformed surface; "
                        "falling back to the material point");
                }
            }
        } else if (ok) {
            ok = RigExecSurfaceFrameAtHit(posedPoints, faceCounts,
                                          faceIndices, hit, rayUp,
                                          &posedFrame);
        }
        if (!ok) {
            // Which step failed, and the ray it failed with. The single
            // message covered three different faults -- no hit, no rest
            // frame, no posed frame -- and they need different fixes.
            RigExecSurfaceHit probe;
            const bool castHit =
                RigExecRaycastSurface(basePoints, faceCounts, faceIndices,
                                      restRayOrigin, restRayDirection,
                                      &probe);
            // The re-cast is a SEPARATE cast at the posed surface, and
            // reporting only the rest one said "cast=hit" while the
            // failure was the other cast missing entirely.
            RigExecSurfaceHit posedProbe;
            const bool posedCast =
                projection != "reproject" ||
                RigExecRaycastSurface(posedPoints, faceCounts, faceIndices,
                                      rayOrigin, rayDirection, &posedProbe);
            GfMatrix4d scratch(1.0);
            const bool restOk =
                castHit && RigExecSurfaceFrameAtHit(basePoints, faceCounts,
                                                    faceIndices, probe,
                                                    rayUp, &scratch);
            const bool posedOk =
                castHit && RigExecSurfaceFrameAtHit(posedPoints, faceCounts,
                                                    faceIndices, probe,
                                                    rayUp, &scratch);
            pose->diagnostics.push_back(
                "SurfaceProjector " + record.path.GetString() +
                ": ray o(" + TfStringPrintf("%.4f %.4f %.4f", rayOrigin[0],
                                            rayOrigin[1], rayOrigin[2]) +
                ") d(" + TfStringPrintf("%.4f %.4f %.4f", rayDirection[0],
                                        rayDirection[1], rayDirection[2]) +
                ") up(" + TfStringPrintf("%.4f %.4f %.4f", rayUp[0],
                                         rayUp[1], rayUp[2]) +
                ") cast=" + (castHit ? "hit" : "MISS") +
                " recast=" + (posedCast ? "hit" : "MISS") +
                " restFrame=" + (restOk ? "ok" : "FAILED") +
                " posedFrame=" + (posedOk ? "ok" : "FAILED"));
            continue;
        }

        // How the material at the hit MOVED: the rigid motion that carries
        // the rest frame onto the posed frame, in the mesh's object space
        // (row vectors, so rest * delta == posed). The projector keeps its
        // own placement and takes only this, exactly as a parent constraint
        // with an offset does -- the projector does not sit at the hit.
        const GfMatrix4d delta = restFrame.GetInverse() * posedFrame;

        // THE HIT FRAME IS THE PROJECTOR. The eye bind supplies only the
        // ray; where that ray meets the DEFORMED surface, and the surface
        // normal there, is the frame -- and driving the projector with it
        // is what translates and rotates it to keep the projection
        // spherical while the eyeball itself stretches.
        //
        // delta is that frame's motion from the rest surface to the posed
        // one, so a projector placed at its rest frame and carried by
        // delta stays on the surface, turns with it, and keeps the iris
        // round. The anchor the bind joint used to supply is gone: the
        // ray already carries the look-at, because rotating the bind
        // sweeps the hit around the ball.
        // TWO CONTRIBUTIONS, and they are not the same thing.
        //
        // delta is DEFORMATION only. The ray is cast once and the frame
        // evaluated on the rest and posed surfaces, so it follows one
        // material point and the look cancels straight out of it -- which
        // is why dropping the anchor left the projector deaf to a look-at
        // even though the ray had turned.
        //
        // The look is the source's own rotation about the eye centre, and
        // it has to be put back beside the deformation. The projector then
        // turns to face where the eye looks AND rides the surface where
        // that lands.
        // THE LOOK IS ONLY SEPARATE IN MATERIAL MODE.
        //
        // material casts once and follows one material point, so the
        // source's own rotation cancels out of delta and has to be put
        // back beside it. reproject casts again with the POSED ray, so
        // delta already carries both the look and the head -- and
        // multiplying by look as well counted them twice. Measured before
        // this: a 30-degree head turn moved the bind 3.07 and the
        // projector 5.94, and a socket stretch of 2.00 moved it 4.00.
        GfMatrix4d look(1.0);
        if (!record.source.IsEmpty() && projection != "reproject") {
            GfMatrix4d sourceRest(1.0), sourcePosed(1.0);
            if (frameOf(pose->jointFramesBase, record.source, &sourceRest) &&
                frameOf(pose->jointFramesFinal, record.source,
                        &sourcePosed)) {
                if (!record.sourceSpace.IsEmpty()) {
                    // Measured in the SAME sibling space as the ray. In
                    // world the look carries the head's rotation, so
                    // turning the head swung the iris as though the eye
                    // had looked -- which is the other half of what
                    // "moving the head control breaks it" looks like.
                    GfMatrix4d spaceRest(1.0), spacePosed(1.0);
                    if (frameOf(pose->jointFramesBase, record.sourceSpace,
                                &spaceRest) &&
                        frameOf(pose->jointFramesFinal, record.sourceSpace,
                                &spacePosed)) {
                        sourceRest = sourceRest * spaceRest.GetInverse();
                        sourcePosed = sourcePosed * spacePosed.GetInverse();
                    }
                }
                look = sourceRest.GetInverse() * sourcePosed;
            }
        }
        // THE HIT FRAME IS ORTHONORMAL, SO delta CANNOT CARRY A SCALE.
        //
        // RigExecSurfaceFrameAtHit normalizes side, up and normal -- it
        // has to, the normal is a direction -- so restFrame^-1 * posedFrame
        // is a RIGID motion however the surface was resized. Scale the rig
        // and the eyeball grows while the projector does not, and worse:
        // the hit's outward motion is recorded as pure translation, which
        // pushes the projector off the ball by exactly one radius.
        // Measured 2026-09-29 on l_eye_geo, before this:
        //
        //     case                ball r   projector   off-centre
        //     rest                2.4846     2.4829       0.0051
        //     Main x2             4.9692     2.4829       2.4904
        //     Main x2 + Head x2   9.9386     2.4829       7.4614
        //
        // The scale is taken from the SOURCE's own world frame and not
        // from the surface. The surface's local scale would also pick up
        // every deformer that squashes the ball -- the socket stretch and
        // the blink both do -- and start resizing the iris on a blink,
        // which is a behaviour change nobody asked for. The source's world
        // frame moves only with the rig: the head controls, the TRS
        // masters, and their product. It is exactly "the scale of the head
        // and the world" and nothing else.
        //
        // Mean row length, so a non-uniform scale is taken as its uniform
        // part rather than refused; the biped only ever scales uniformly
        // here, and a projector is a sphere's worth of placement anyway.
        double sourceScale = 1.0;
        if (!record.source.IsEmpty()) {
            GfMatrix4d worldRest(1.0), worldPosed(1.0);
            if (frameOf(pose->jointFramesBase, record.source, &worldRest) &&
                frameOf(pose->jointFramesFinal, record.source, &worldPosed)) {
                double rest = 0.0, posed = 0.0;
                for (int i = 0; i < 3; ++i) {
                    rest += GfVec3d(worldRest.GetRow3(i)).GetLength();
                    posed += GfVec3d(worldPosed.GetRow3(i)).GetLength();
                }
                if (rest > 1e-9 && std::isfinite(rest) &&
                    std::isfinite(posed)) {
                    sourceScale = posed / rest;
                }
            }
        }
        // The head's scale and the world's are independent and multiply:
        // Main x2 with Head x2 is a x4 eyeball, and the projector has to
        // be a x4 projector.
        sourceScale *= spaceScale;
        GfMatrix4d carried = delta;
        if (std::abs(sourceScale - 1.0) > 1e-9 &&
            std::isfinite(sourceScale) && sourceScale > 0.0) {
            // Anchored at the rest hit, which is restFrame's own origin:
            // the material point must still map to where it ended up, and
            // only the size around it changes. p' = p (s L) + t with
            // t = t0 + (1 - s) (p_rest L), so a pure scale about any
            // centre comes out as that same scale and no stray offset.
            const GfVec3d restHit(restFrame.ExtractTranslation());
            GfMatrix4d linear = delta;
            linear.SetTranslateOnly(GfVec3d(0));
            const GfVec3d mapped = linear.TransformDir(restHit);
            for (int i = 0; i < 3; ++i) {
                carried.SetRow3(i, GfVec3d(delta.GetRow3(i)) * sourceScale);
            }
            carried.SetTranslateOnly(GfVec3d(delta.ExtractTranslation())
                                     + (1.0 - sourceScale) * mapped);
        }
        if (!record.shaderPrimvar.IsEmpty()) {
            pose->shaderMatrices[meshPath][record.shaderPrimvar] =
                record.shaderOffset * look * carried;
        }
        // The animator's shader dials, packed into one matrix primvar.
        //
        // Read off the stage here rather than cached on the record: this
        // runs twice per frame on a biped and reads sixteen scalars at
        // most, which is nothing beside the pose it rides on, and the
        // alternative was another field on RigExecRigPose -- which is a
        // struct that, for reasons not yet understood, breaks four
        // constraint tests when anything is added to it.
        if (const UsdPrim projector = _stage->GetPrimAtPath(record.path)) {
            TfToken dialPrimvar;
            if (const UsdAttribute a = projector.GetAttribute(
                    TfToken("rigExec:shaderDialPrimvar"))) {
                a.Get(&dialPrimvar);
            }
            if (!dialPrimvar.IsEmpty()) {
                SdfPathVector dials;
                if (const UsdRelationship rel = projector.GetRelationship(
                        TfToken("rigExec:shaderDialSources"))) {
                    rel.GetTargets(&dials);
                }
                GfMatrix4d packed(0.0);
                const size_t slots =
                    std::min<size_t>(dials.size(), 16);
                for (size_t i = 0; i < slots; ++i) {
                    double value = 0.0;
                    // A mover may have revised the dial; that revision is
                    // the value the animator sees, so it wins over the
                    // authored one.
                    const auto moved = pose->movedProperties.find(dials[i]);
                    if (moved != pose->movedProperties.end() &&
                        moved->second.IsHolding<double>()) {
                        value = moved->second.UncheckedGet<double>();
                    } else if (const UsdAttribute attr =
                                   _stage->GetAttributeAtPath(dials[i])) {
                        attr.Get(&value, time);
                    }
                    if (std::isfinite(value)) {
                        packed[int(i / 4)][int(i % 4)] = value;
                    }
                }
                pose->shaderMatrices[meshPath][dialPrimvar] = packed;
            }
        }

        // The bone, so anything parented to it rides along. The surface
        // it rides is moved by its own deformers, so nothing here touches
        // the mesh: rewriting its points would fight the skin that moves
        // it, which is what the old arrangement did when the transform
        // was the projector rather than the placement.
        if (!record.source.IsEmpty()) {
            const auto boneBase = pose->jointFramesBase.find(record.target);
            const auto bonePosed = pose->jointFramesFinal.find(record.target);
            GfMatrix4d base(1.0), posedBone(1.0);
            if (boneBase != pose->jointFramesBase.end() &&
                bonePosed != pose->jointFramesFinal.end() &&
                RigExecPointsToMatrix(RigExecIdentityLandmarks(),
                                      boneBase->second.points, &base) &&
                RigExecPointsToMatrix(RigExecIdentityLandmarks(),
                                      bonePosed->second.points, &posedBone)) {
                pose->providerBaseXforms[record.target] = base;
                pose->providerXforms[record.target] =
                    posedBone * look * delta;
            }
        }

    }
}

RigExecRigPose
RigExecRigEvaluator::_WithSurfaceProjectors(RigExecRigPose pose,
                                            UsdTimeCode time)
{
    if (!_surfaceProjectors.empty()) {
        RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "SurfaceProjectors", "publish");
        _ApplySurfaceProjectors(&pose, time);
    }
    return pose;
}

} // namespace rigExec
