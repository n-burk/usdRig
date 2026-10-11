// Reference arithmetic retains the original scalar statement order.
#include "rigExec/weightReference.h"
#include "rigExec/rigEvaluatorInternal.h"
#include "rigExec/moverGraph.h"
#include "rigExecMath/volumeFieldCheck.h"
#include "rigExecMath/weightFields.h"
#include "pxr/usd/usdGeom/pointBased.h"
#include <algorithm>
#include <cmath>
#include <set>
namespace rigExec {
namespace {
const TfToken _kGeoInputWeights("rigExec:inputWeights");
const TfToken _kGeoCombineMode("rigExec:combineMode");
const TfToken _kGeoPlaneAxis("rigExec:planeAxis");
const TfToken _kGeoPlaneBounds("rigExec:planeBounds");
const TfToken _kGeoRangePolicy("rigExec:rangePolicy");
const TfToken _kGeoRepresentation("rigExec:representation");
const TfToken _kGeoDefaultWeight("rigExec:defaultWeight");
const TfToken _kGeoOperation("rigExec:operation");
const TfToken _kGeoBaseWeight("rigExec:baseWeight");
const TfToken _kGeoWeightTarget("rigExec:weightTarget");
const TfToken _kGeoIndices("rigExec:indices");
const TfToken _kGeoValues("rigExec:values");
const TfToken _kGeoSampleSource("rigExec:sampleSource");
const TfToken _kGeoCurve("rigExec:curve");
const TfToken _kGeoStrength("inputs:strength");
const TfToken _kGeoInvert("inputs:invert");
const TfToken _kGeoFalloffMin("inputs:falloffMin");
const TfToken _kGeoFalloffMax("inputs:falloffMax");
const TfToken _kGeoExtentU("inputs:extentU");
const TfToken _kGeoExtentV("inputs:extentV");
const TfToken _kGeoScaleX("inputs:scaleX");
const TfToken _kGeoScaleY("inputs:scaleY");
const TfToken _kGeoScaleZ("inputs:scaleZ");
const TfToken _kGeoScaleXPos("inputs:scaleXPos");
const TfToken _kGeoScaleYPos("inputs:scaleYPos");
const TfToken _kGeoScaleZPos("inputs:scaleZPos");
const TfToken _kGeoScaleXNeg("inputs:scaleXNeg");
const TfToken _kGeoScaleYNeg("inputs:scaleYNeg");
const TfToken _kGeoScaleZNeg("inputs:scaleZNeg");
const TfToken _kGeoDriver("inputs:driver");
const TfToken _kGeoScale("inputs:scale");
const TfToken _kGeoBias("inputs:bias");
const TfToken _kGeoMultiply("multiply");
const TfToken _kGeoAdd("add");
const TfToken _kGeoSubtract("subtract");
const TfToken _kGeoMax("max");
const TfToken _kGeoMin("min");
const TfToken _kGeoAverage("average");
const TfToken _kGeoOverlay("overlay");
const TfToken _kGeoAxisX("x");
const TfToken _kGeoAxisY("y");
const TfToken _kGeoAxisZ("z");
const TfToken _kGeoUnbounded("unbounded");
const TfToken _kGeoBounded("bounded");
const TfToken _kGeoConstant("constant");
const TfToken _kGeoStrict("strict");
const TfToken _kGeoClamp("clamp");
const TfToken _kGeoDense("dense");
const TfToken _kGeoSparse("sparse");
const TfToken _kGeoCombineWeight("RigExecCombineWeight");
const TfToken _kGeoPlaneWeight("RigExecPlaneWeight");
const TfToken _kGeoSphereWeight("RigExecSphereWeight");
const TfToken _kGeoCurveWeight("RigExecCurveWeight");
const TfToken _kGeoDynamicWeight("RigExecDynamicWeight");
const TfToken _kGeoWeightObject("rigExec:weightObject");
const TfToken _kGeoPoints("points");
// The mover envelope default, in the inputs: namespace -- distinct from a
// weight object's rigExec:defaultWeight above.
const TfToken _kGeoInputsDefaultWeight("inputs:defaultWeight");
// The falloff profile's fallback. Namespace scope like the rest: the oracle
// runs inside step bodies on workers and builds no token from text.
const TfToken _kGeoSmoothProfile("smooth");


bool IsVolume(const TfToken &t) { return t == _kGeoPlaneWeight || t == _kGeoSphereWeight || t == _kGeoCurveWeight; }
const TfToken _kGeoStaticWeight("RigExecStaticWeight");
bool IsWeightObject(const TfToken &t) { return IsVolume(t) || t == _kGeoCombineWeight || t == _kGeoDynamicWeight || t == _kGeoStaticWeight; }
float ReadResolved(const RigExecWeightReferencePrim &p, const TfToken &name, float fallback, UsdTimeCode) {
    const auto a = p.GetAttribute(name);
    return a.resolvedFloat.IsHolding<float>() ? a.resolvedFloat.UncheckedGet<float>() : fallback;
}
SdfPath Canonical(const RigExecWeightReferenceContext &context, const SdfPath &path) {
    const auto it = context.canonicalPoints.find(path);
    return it == context.canonicalPoints.end() ? path : it->second;
}
bool ReadTarget(const RigExecWeightReferenceContext &context, const RigExecWeightReferencePrim &p,
                const TfToken &name, UsdTimeCode, std::vector<GfVec3f> *out) {
    out->clear(); const auto rel = p.GetRelationship(name);
    if (rel.targets.size() != 1) return false;
    const auto phase=context.phasedPoints.find({p.path,name});
    if(phase!=context.phasedPoints.end()) {
        if(!phase->second.IsHolding<VtVec3fArray>())return false;
        const auto &points=phase->second.UncheckedGet<VtVec3fArray>();
        out->assign(points.begin(),points.end());return true;
    }
    const auto it = context.points.find(Canonical(context, rel.targets[0]));
    if (it == context.points.end()) return false;
    out->assign(it->second.begin(), it->second.end()); return true;
}
bool SamplesInFlight(const RigExecWeightReferencePrim &p, bool *value, std::string *error) {
    *value = false;
    if (!p.phaseError.empty()) { *error = p.phaseError; return false; }
    const auto &phase = p.weightTargetPhase;
    if (phase.kind == RigExecReadPhaseKind::Preceding) *value = true;
    return true;
}
bool ResolveWeight(const RigExecWeightReferenceContext &, const SdfPath &, size_t, UsdTimeCode,
                   std::vector<float> *, std::string *, const std::vector<GfVec3f> *, std::set<SdfPath> *);
bool
ResolveVolume(
    const RigExecWeightReferenceContext &context,
    const RigExecWeightReferencePrim &prim, size_t count, UsdTimeCode time,
    std::vector<float> *weights, std::string *error,
    const std::vector<GfVec3f> *currentPoints, std::set<SdfPath> *active)
{
    // Spelled only when an error needs it: this runs every time a volume
    // weight is resolved, and a resolve that succeeds reports nothing.
    const auto who = [&prim]() { return prim.GetPath().GetAsString(); };
    const TfToken typeName = prim.GetTypeName();

    // The composed field folds its inputs; it measures nothing itself.
    if (typeName == _kGeoCombineWeight) {
        SdfPathVector inputs;
        if (RigExecWeightReferenceRelationship rel =
                prim.GetRelationship(_kGeoInputWeights)) {
            rel.GetTargets(&inputs);
        }
        TfToken modeName = _kGeoMultiply;
        if (RigExecWeightReferenceAttribute a =
                prim.GetAttribute(_kGeoCombineMode)) {
            a.Get(&modeName, time);
        }
        RigExecWeightCombine mode;
        if (modeName == _kGeoMultiply) {
            mode = RigExecWeightCombine::Multiply;
        } else if (modeName == _kGeoAdd) {
            mode = RigExecWeightCombine::Add;
        } else if (modeName == _kGeoSubtract) {
            mode = RigExecWeightCombine::Subtract;
        } else if (modeName == _kGeoMax) {
            mode = RigExecWeightCombine::Max;
        } else if (modeName == _kGeoMin) {
            mode = RigExecWeightCombine::Min;
        } else if (modeName == _kGeoAverage) {
            mode = RigExecWeightCombine::Average;
        } else if (modeName == _kGeoOverlay) {
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
            if (!ResolveWeight(context, input, count, time, &field, error,
                                 currentPoints, active)) {
                return false;
            }
            fields.push_back(std::move(field));
        }
        if (!RigExecCombineWeightFields(mode, fields, count, weights)) {
            *error = who() + ": combine inputs disagree on element count";
            return false;
        }
        const float strength = ReadResolved(
            prim, _kGeoStrength, 1.0f, time);
        const float invert = ReadResolved(
            prim, _kGeoInvert, 0.0f, time);
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
    const GfMatrix4d *placement = nullptr;
    const auto matrixIt = context.placements.find(prim.GetPath());
    if (matrixIt != context.placements.end()) placement = &matrixIt->second;
    if (!placement) {
        *error = who() + ": no resolved placement for this volume weight";
        return false;
    }
    // Scale and shear are removed so the field matches the rigid guide a
    // viewer draws; inputs:scaleX/Y/Z is the sole authority on
    // anisotropy (see the RigExecVolumeWeight schema doc).
    GfMatrix4d rigid = placement->RemoveScaleShear();
    const double det = rigid.GetDeterminant();
    if (!std::isfinite(det) || std::abs(det) < 1e-12) {
        *error = who() + ": degenerate volume placement";
        return false;
    }
    GfMatrix4d worldToLocal = rigid.GetInverse();

    // Which points the distance function measures.
    std::vector<GfVec3f> samplePoints;
    bool inFlight = false;
    std::string phaseError;
    if (!SamplesInFlight(prim, &inFlight, &phaseError)) {
        *error = who() + ": " + phaseError;
        return false;
    }
    if (inFlight) {
        if (!currentPoints) {
            *error = who() +
                     ": rigExec:weightTarget reads `preceding` but no "
                     "in-flight points were supplied";
            return false;
        }
        samplePoints = *currentPoints;
    } else {
        // An explicit sampleSource wins over the weighted domain, which
        // is how one mesh is weighted by another mesh's shape.
        if (!ReadTarget(context, prim, _kGeoSampleSource, time,
                               &samplePoints) &&
            !ReadTarget(context, prim, _kGeoWeightTarget, time,
                               &samplePoints)) {
            *error = who() + ": could not read the points to sample";
            return false;
        }
    }
    if (samplePoints.size() != count) {
        *error = who() + ": sampled point count does not match the target";
        return false;
    }

    RigExecFalloffParams params;
    auto readFloat = [&prim, time](const TfToken &name, float fallback) {
        return ReadResolved(
            prim, name, fallback, time);
    };
    params.falloffMin = readFloat(_kGeoFalloffMin, 0.0f);
    params.falloffMax = readFloat(_kGeoFalloffMax, 1.0f);
    params.invert = readFloat(_kGeoInvert, 0.0f);
    params.strength = readFloat(_kGeoStrength, 1.0f);
    params.curve = prim.falloff;

    if (typeName == _kGeoPlaneWeight) {
        TfToken axis = _kGeoAxisY;
        if (RigExecWeightReferenceAttribute a = prim.GetAttribute(_kGeoPlaneAxis)) {
            a.Get(&axis, time);
        }
        const int axisIndex =
            axis == _kGeoAxisX ? 0
            : (axis == _kGeoAxisY ? 1 : (axis == _kGeoAxisZ ? 2 : -1));
        if (axisIndex < 0) {
            *error = who() + ": unknown rigExec:planeAxis " + axis.GetString();
            return false;
        }
        // Bounded clips the field to the in-plane rectangle. Mirrors
        // _BuildPlaneWeightPacket exactly, including reading the extents
        // only in the bounded arm -- the two paths have to agree value
        // for value or the parity harness fires.
        TfToken boundsMode = _kGeoUnbounded;
        if (RigExecWeightReferenceAttribute a =
                prim.GetAttribute(_kGeoPlaneBounds)) {
            a.Get(&boundsMode, time);
        }
        RigExecPlaneBounds extent;
        const RigExecPlaneBounds *extentPtr = nullptr;
        if (boundsMode == _kGeoBounded) {
            extent.extentU = readFloat(_kGeoExtentU, 1.0f);
            extent.extentV = readFloat(_kGeoExtentV, 1.0f);
            if (!RigExecVolumeExtentsOk(who(), extent.extentU, extent.extentV,
                                        error)) {
                return false;
            }
            extentPtr = &extent;
        } else if (boundsMode != _kGeoUnbounded) {
            *error = RigExecUnknownPlaneBoundsMessage(
                who(), boundsMode.GetString());
            return false;
        }
        RigExecPlaneWeightField(
            samplePoints, worldToLocal, axisIndex, params, weights, extentPtr);
        return true;
    }

    // Sphere and curve both take the per-axis divisors, folded into the
    // matrix so the hot loop stays one transform.
    const float sx = readFloat(_kGeoScaleX, 1.0f);
    const float sy = readFloat(_kGeoScaleY, 1.0f);
    const float sz = readFloat(_kGeoScaleZ, 1.0f);
    if (!RigExecVolumeAxisScalesOk(who(), sx, sy, sz, error)) {
        return false;
    }
    GfMatrix4d divide(1.0);
    divide.SetScale(GfVec3d(1.0 / double(sx), 1.0 / double(sy),
                            1.0 / double(sz)));
    worldToLocal = worldToLocal * divide;

    if (typeName == _kGeoSphereWeight) {
        const GfVec3f positiveScales(
            readFloat(_kGeoScaleXPos, 1.0f),
            readFloat(_kGeoScaleYPos, 1.0f),
            readFloat(_kGeoScaleZPos, 1.0f));
        const GfVec3f negativeScales(
            readFloat(_kGeoScaleXNeg, 1.0f),
            readFloat(_kGeoScaleYNeg, 1.0f),
            readFloat(_kGeoScaleZNeg, 1.0f));
        if (!RigExecSignedAxisScalesOk(who(), positiveScales, negativeScales,
                                       error)) {
            return false;
        }
        RigExecSphereWeightField(samplePoints, worldToLocal, params, weights,
                                 positiveScales, negativeScales);
        return true;
    }
    if (typeName == _kGeoCurveWeight) {
        std::vector<GfVec3f> curvePoints;
        if (!ReadTarget(context, prim, _kGeoCurve, time, &curvePoints) ||
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
ResolveWeight(
    const RigExecWeightReferenceContext &context,
    const SdfPath &weightPrimPath, size_t count, UsdTimeCode time,
    std::vector<float> *weights, std::string *error,
    const std::vector<GfVec3f> *currentPoints, std::set<SdfPath> *active)
{
    if (!active->insert(weightPrimPath).second) {
        *error = "cyclic weight object closure at " + weightPrimPath.GetString();
        return false;
    }
    struct Pop { std::set<SdfPath> *paths; SdfPath path;
        ~Pop() { paths->erase(path); } } pop{active,weightPrimPath};
    weights->assign(count, 1.0f);
    const RigExecWeightReferencePrim prim = context.GetPrimAtPath(weightPrimPath);
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
    if (!IsWeightObject(typeName)) {
        *error = "unknown weight object type " + typeName.GetString() +
                 " on " + weightPrimPath.GetString();
        return false;
    }
    if (IsVolume(typeName) || typeName == _kGeoCombineWeight) {
        std::vector<float> resolved;
        if (!ResolveVolume(context, prim, count, time, &resolved, error,
                                   currentPoints, active)) {
            return false;
        }
        TfToken volumePolicy = _kGeoClamp;
        if (RigExecWeightReferenceAttribute a =
                prim.GetAttribute(_kGeoRangePolicy)) {
            a.Get(&volumePolicy, time);
        }
        for (float &w : resolved) {
            if (!std::isfinite(w)) {
                *error = "non-finite weight on " + weightPrimPath.GetString();
                return false;
            }
            if (w < 0.0f || w > 1.0f) {
                if (volumePolicy != _kGeoClamp) {
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

    TfToken representation = _kGeoConstant;
    if (RigExecWeightReferenceAttribute a = prim.GetAttribute(
            _kGeoRepresentation)) {
        a.Get(&representation, time);
    }
    const float defaultWeight = ReadResolved(
        prim, _kGeoDefaultWeight, 0.0f, time);
    TfToken rangePolicy = _kGeoStrict;
    if (RigExecWeightReferenceAttribute a = prim.GetAttribute(_kGeoRangePolicy)) {
        a.Get(&rangePolicy, time);
    }
    if (rangePolicy != _kGeoStrict && rangePolicy != _kGeoClamp) {
        *error = "unknown rangePolicy on " + weightPrimPath.GetString();
        return false;
    }

    const bool isDynamic = typeName == _kGeoDynamicWeight;
    if (isDynamic) {
        TfToken operation = _kGeoMultiply;
        if (RigExecWeightReferenceAttribute a =
                prim.GetAttribute(_kGeoOperation)) {
            a.Get(&operation, time);
        }
        if (operation != _kGeoMultiply) {
            *error = "unknown dynamic-weight operation on " +
                     weightPrimPath.GetString();
            return false;
        }

        // Base field first, then r_i = (b_i * d) * s + a (spec §4.1).
        std::vector<float> base(count, 1.0f);
        SdfPathVector baseTargets;
        if (RigExecWeightReferenceRelationship rel =
                prim.GetRelationship(_kGeoBaseWeight)) {
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
            if (representation != _kGeoConstant) {
                *error = "no-base dynamic weight must be constant on " +
                         weightPrimPath.GetString();
                return false;
            }
        } else {
            const RigExecWeightReferencePrim basePrim = context.GetPrimAtPath(baseTargets[0]);
            if (!basePrim) {
                *error = "missing base weight object on " +
                         weightPrimPath.GetString();
                return false;
            }
            // The base descriptor must exactly match the dynamic
            // descriptor: canonical target, representation, and sparse
            // support (spec §4.1).
            auto canonicalWeightTarget =
                [&context, &time](const RigExecWeightReferencePrim &p) -> SdfPath {
                SdfPathVector t;
                if (RigExecWeightReferenceRelationship rel = p.GetRelationship(
                        _kGeoWeightTarget)) {
                    rel.GetTargets(&t);
                }
                return t.size() == 1 ? Canonical(context, t[0])
                                     : SdfPath();
            };
            if (canonicalWeightTarget(prim) !=
                    canonicalWeightTarget(basePrim) ||
                canonicalWeightTarget(prim).IsEmpty()) {
                *error = "dynamic/base weight target mismatch on " +
                         weightPrimPath.GetString();
                return false;
            }
            TfToken baseRepresentation = _kGeoConstant;
            if (RigExecWeightReferenceAttribute a = basePrim.GetAttribute(
                    _kGeoRepresentation)) {
                a.Get(&baseRepresentation, time);
            }
            if (baseRepresentation != representation) {
                *error = "dynamic/base representation mismatch on " +
                         weightPrimPath.GetString();
                return false;
            }
            if (representation == _kGeoSparse) {
                // The dynamic descriptor's sparse support is inherited
                // from the base; a dynamic prim that authors its own
                // support must match the base exactly (spec §4.1).
                VtIntArray mine;
                if (RigExecWeightReferenceAttribute a =
                        prim.GetAttribute(_kGeoIndices)) {
                    a.Get(&mine, time);
                }
                if (!mine.empty()) {
                    VtIntArray theirs;
                    if (RigExecWeightReferenceAttribute a = basePrim.GetAttribute(
                            _kGeoIndices)) {
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
            if (!ResolveWeight(context, baseTargets[0], count, time, &base, error,
                                 currentPoints, active)) {
                return false;
            }
        }
        const float driver = ReadResolved(
            prim, _kGeoDriver, 1.0f, time);
        const float scale = ReadResolved(
            prim, _kGeoScale, 1.0f, time);
        const float bias = ReadResolved(
            prim, _kGeoBias, 0.0f, time);
        for (size_t i = 0; i < count; ++i) {
            float r = (base[i] * driver) * scale + bias;
            if (!std::isfinite(r)) {
                *error = "non-finite dynamic weight on " +
                         weightPrimPath.GetString();
                return false;
            }
            if (r < 0.0f || r > 1.0f) {
                if (rangePolicy == _kGeoClamp) {
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
    const TfToken *const staticFields[] = {
        &_kGeoValues, &_kGeoIndices,
        &_kGeoDefaultWeight, &_kGeoRepresentation,
        &_kGeoRangePolicy};
    for (const TfToken *field : staticFields) {
        const RigExecWeightReferenceAttribute a = prim.GetAttribute(*field);
        if (a && (a.GetNumTimeSamples() > 0 || a.HasAuthoredConnections())) {
            *error = "static weight field " + field->GetString() +
                     " has time samples or connections on " +
                     weightPrimPath.GetString();
            return false;
        }
    }
    VtFloatArray values;
    if (RigExecWeightReferenceAttribute a = prim.GetAttribute(_kGeoValues)) {
        a.Get(&values, time);
    }
    if (representation == _kGeoConstant) {
        if (!values.empty()) {
            *error = "constant weight must not author values on " +
                     weightPrimPath.GetString();
            return false;
        }
        weights->assign(count, defaultWeight);
    } else if (representation == _kGeoDense) {
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
    } else if (representation == _kGeoSparse) {
        VtIntArray indices;
        if (RigExecWeightReferenceAttribute a = prim.GetAttribute(_kGeoIndices)) {
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
            (rangePolicy == _kGeoStrict && (w < 0.0f || w > 1.0f))) {
            *error = "weight range violation on " +
                     weightPrimPath.GetString();
            return false;
        }
    }
    if (rangePolicy == _kGeoClamp) {
        for (float &w : *weights) {
            w = std::min(std::max(w, 0.0f), 1.0f);
        }
    }
    return true;
}

} // namespace
bool RigExecResolveWeightReference(const RigExecWeightReferenceContext &context, const SdfPath &path,
    size_t count, std::vector<float> *weights, std::string *error, const std::vector<GfVec3f> *currentPoints) {
    std::set<SdfPath> active;
    return ResolveWeight(context, path, count, UsdTimeCode::Default(), weights, error, currentPoints, &active);
}
RigExecWeightReferenceContext RigExecCaptureWeightReference(
    const UsdStageRefPtr &stage, const SdfPath &root, const RigExecResolvedInputs &resolved,
    const std::map<SdfPath,VtValue> &upstream, UsdTimeCode time,
    const std::function<const GfMatrix4d *(const SdfPath &)> &placement) {
    RigExecWeightReferenceContext context;
    std::set<SdfPath> seen;
    std::function<void(const SdfPath &)> visit = [&](const SdfPath &path) {
        if (!seen.insert(path).second) return;
        RigExecWeightReferencePrim record;
        record.path = path;
        const auto prim = stage->GetPrimAtPath(path);
        record.exists = bool(prim);
        if (!prim) { context.prims[path] = record; return; }
        record.type = prim.GetTypeName();
        for (const auto &a : prim.GetAttributes()) {
            RigExecWeightReferenceAttribute value;
            value.exists = true;
            value.connected = a.HasAuthoredConnections();
            value.timeSamples = a.GetNumTimeSamples();
            a.Get(&value.raw, time);
            const auto &name = a.GetName();
            const bool scalarRead = name == _kGeoDefaultWeight || name == _kGeoStrength ||
                name == _kGeoInvert || name == _kGeoFalloffMin || name == _kGeoFalloffMax ||
                name == _kGeoExtentU || name == _kGeoExtentV || name == _kGeoScaleX ||
                name == _kGeoScaleY || name == _kGeoScaleZ || name == _kGeoScaleXPos ||
                name == _kGeoScaleYPos || name == _kGeoScaleZPos || name == _kGeoScaleXNeg ||
                name == _kGeoScaleYNeg || name == _kGeoScaleZNeg || name == _kGeoDriver ||
                name == _kGeoScale || name == _kGeoBias;
            value.resolvedFloatSite = scalarRead;
            float scalar = 0.0f;
            if (scalarRead && resolved.GetAttribute(a, time, &scalar)) value.resolvedFloat = VtValue(scalar);
            record.attributes.emplace(a.GetName(), std::move(value));
        }
        for (const auto &rel : prim.GetRelationships()) {
            RigExecWeightReferenceRelationship value;
            value.exists = true; rel.GetTargets(&value.targets);
            RigExecResolveReadPhase(rel,&value.phase,&value.phaseError);
            record.relationships.emplace(rel.GetName(), std::move(value));
        }
        RigExecResolveReadPhase(prim.GetRelationship(_kGeoWeightTarget), &record.weightTargetPhase, &record.phaseError);
        record.falloff = evaluatorDetail::_BakeFalloffLut(prim);
        if (const auto *matrix = placement(path)) context.placements[path] = *matrix;
        context.prims[path] = record;
        for (const auto &entry : record.relationships) {
            for (const auto &target : entry.second.targets) {
                if (entry.first == _kGeoInputWeights || entry.first == _kGeoBaseWeight) { visit(target); continue; }
                SdfPath canonical = target;
                if (target.IsPrimPath() && stage->GetPrimAtPath(target).IsA<UsdGeomPointBased>())
                    canonical = target.AppendProperty(_kGeoPoints);
                context.canonicalPoints[target] = canonical;
                VtVec3fArray points;
                const auto authored = upstream.find(canonical);
                bool read = false;
                if (authored != upstream.end() && authored->second.IsHolding<VtVec3fArray>()) {
                    points = authored->second.UncheckedGet<VtVec3fArray>(); read = true;
                } else if (const auto a = stage->GetAttributeAtPath(canonical)) read = a.Get(&points,time);
                if (read) context.points[canonical] = points;
            }
        }
    };
    visit(root); return context;
}
} // namespace rigExec
