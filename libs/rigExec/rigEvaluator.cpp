//
// RigExec rig evaluator implementation.
//
#include "rigEvaluator.h"

#include "frameExtraction.h"
#include "rigExecMath/geometryKernels.h"
#include "rigExecMath/solvers.h"
#include "rigExecMath/weightFields.h"

#include "pxr/base/gf/rotation.h"
#include "pxr/base/ts/spline.h"
#include "pxr/base/tf/diagnostic.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usdGeom/basisCurves.h"
#include "pxr/usd/usdGeom/curves.h"
#include "pxr/usd/usdGeom/gprim.h"
#include "pxr/usd/usdGeom/imageable.h"
#include "pxr/usd/usdGeom/mesh.h"
#include "pxr/usd/usdGeom/xformCache.h"
#include "pxr/usd/usdGeom/xformable.h"
#include "pxr/usd/usdGeom/pointBased.h"
#include "pxr/usd/usdGeom/points.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <functional>
#include <set>

namespace rigExec {

namespace {

const TfToken _computePointFrame("computePointFrame");
const TfToken _computePointFrameArray("computePointFrameArray");
const TfToken _movesRel("rigExec:moves");
const TfToken _enabledAttr("inputs:enabled");
const TfToken _restPointsAttr("rigExec:restPoints");
const TfToken _computeFalloffLut("computeFalloffLut");
const TfToken _falloffProfileAttr("rigExec:falloffProfile");
const TfToken _falloffCurveAttr("rigExec:falloffCurve");
const TfToken _samplePhaseAttr("rigExec:samplePhase");

/// True for the schema types that GENERATE a weight field from a placed
/// volume, as opposed to storing or modulating one.
bool
_IsVolumeWeightType(const TfToken &typeName)
{
    return typeName == "RigExecSphereWeight" ||
           typeName == "RigExecPlaneWeight" ||
           typeName == "RigExecCurveWeight";
}

/// True for every schema that publishes computeWeightPacket.
///
/// The volumetric types are NOT RigExecWeightObject subclasses -- a typed
/// schema gets exactly one base and they spend it on RigExecXformable, to
/// be placeable (see the RigExecVolumeWeight schema doc) -- so weight-object
/// identity is a type-name question here rather than an IsA one. That is
/// what the rest of this file already does for RigExecDynamicWeight.
bool
_IsWeightObjectType(const TfToken &typeName)
{
    return typeName == "RigExecStaticWeight" ||
           typeName == "RigExecDynamicWeight" ||
           typeName == "RigExecCombineWeight" ||
           _IsVolumeWeightType(typeName);
}

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

// Canonicalizes a moves target: a PointBased prim maps to its .points
// property by the standard UsdGeomPointBased rule; a RigExec transform
// provider prim keeps the prim path (its writable value is the point frame);
// property paths stay exact (spec §4.2).
SdfPath
_CanonicalizeTarget(const UsdStageRefPtr &stage, const SdfPath &target)
{
    if (target.IsPrimPath()) {
        const UsdPrim prim = stage->GetPrimAtPath(target);
        if (prim && prim.IsA<UsdGeomPointBased>()) {
            return target.AppendProperty(TfToken("points"));
        }
    }
    return target;
}

// Discovers the rig's joint output set implicitly (spec §4.1: the rig is a
// namespace root, not a manifest). Movers are already found this way -- a
// post-order walk where carrying rigExec:moves is what makes a prim a mover --
// and joints now follow the same rule: being a RigExecJoint under the rig is
// what makes a prim a joint output. Returned in namespace pre-order, which
// reproduces the parent-before-child ordering the authored lists used and keeps
// the binding-epoch digest stable against unrelated edits.
//
// Operator-declared joints are unioned in afterwards. Solver rigExec:joints
// targets are validated to be RigExecJoint prims later in Compile, so in a
// valid rig they are already a subset of the namespace walk; including them
// means a rig that is midway through an edit still compiles the joints its
// operators actually drive, instead of failing on a set that disagrees with the
// graph. The schema is codeless (skipCodeGeneration), so type identity is a
// type-name comparison -- the same idiom the imaging registry uses to find the
// rig itself. RigExecJoint has no derived types.
std::vector<SdfPath>
_DiscoverJointOutputs(const UsdStageRefPtr &stage, const SdfPath &rigPath)
{
    static const TfToken kJointType("RigExecJoint");
    static const TfToken kJointsRel("rigExec:joints");

    std::vector<SdfPath> joints;
    std::set<SdfPath> seen;

    const UsdPrim rig = stage->GetPrimAtPath(rigPath);
    if (!rig) {
        return joints;
    }

    for (const UsdPrim &prim : UsdPrimRange(rig)) {
        if (prim.GetTypeName() == kJointType && seen.insert(prim.GetPath()).second) {
            joints.push_back(prim.GetPath());
        }
    }

    // Union in whatever the operators name, in solver namespace order.
    for (const UsdPrim &prim : UsdPrimRange(rig)) {
        const UsdRelationship jointsRel = prim.GetRelationship(kJointsRel);
        if (!jointsRel) {
            continue;
        }
        SdfPathVector targets;
        jointsRel.GetTargets(&targets);
        for (const SdfPath &target : targets) {
            const UsdPrim joint = stage->GetPrimAtPath(target);
            if (joint && joint.GetTypeName() == kJointType &&
                seen.insert(target).second) {
                joints.push_back(target);
            }
        }
    }

    return joints;
}

// Discovers the rig's controls the same implicit way (spec §4.1): being a
// RigExecControl under the rig is what makes a prim a control. Returned in
// namespace pre-order so the discovered order -- and with it the epoch
// digest -- is stable against unrelated edits.
//
// No union pass over operator wiring, unlike the joints. A solver's
// rigExec:controls names inputs it READS, and reading a control does not
// make it one; the type does. And no emptiness rule either: a rig whose
// joints are animated directly has no control prims, which is a legal rig
// that simply draws no control guides.
std::vector<SdfPath>
_DiscoverControls(const UsdStageRefPtr &stage, const SdfPath &rigPath)
{
    static const TfToken kControlType("RigExecControl");

    std::vector<SdfPath> controls;
    const UsdPrim rig = stage->GetPrimAtPath(rigPath);
    if (!rig) {
        return controls;
    }
    for (const UsdPrim &prim : UsdPrimRange(rig)) {
        if (prim.GetTypeName() == kControlType) {
            controls.push_back(prim.GetPath());
        }
    }
    return controls;
}

}  // namespace

RigExecRigEvaluator::RigExecRigEvaluator(
    const UsdStageRefPtr &stage, const SdfPath &rigPath)
    : _stage(stage)
    , _rigPath(rigPath)
{
    // The rig evaluates directly against the source stage. There used to be
    // a private derived stage here, holding an anonymous session sublayer
    // for the compiler's generated property applications; the engine authors
    // nothing now, so there is nothing to hold and no stage to derive.
}

RigExecRigEvaluator::~RigExecRigEvaluator()
{
    _guideTaps.reset();
    _taps.reset();
}

size_t
RigExecRigEvaluator::_ComputeStructureDigest() const
{
    // The v0.1 binding-epoch identity: canonical mover paths, schema
    // types, targets, post-order ordinals, and the structural dependency
    // wiring each operation declares — relationship identities, read
    // phases, weight-descriptor shape, and blend membership/activations
    // (spec §4.2, §6.3). Structural edits change it; numeric values and
    // shape-preserving enables do not.
    std::string digest;

    auto appendRelTargets =
        [this, &digest](const UsdPrim &prim, const char *name,
                        bool sorted) {
        SdfPathVector targets;
        if (UsdRelationship rel = prim.GetRelationship(TfToken(name))) {
            rel.GetTargets(&targets);
        }
        if (sorted) {
            std::sort(targets.begin(), targets.end());
        }
        digest += name;
        digest += '=';
        for (const SdfPath &t : targets) {
            digest += t.GetString();
            digest += ',';
        }
        digest += '|';
        return targets;
    };
    auto appendToken = [&digest](const UsdPrim &prim, const char *name) {
        TfToken value;
        if (UsdAttribute a = prim.GetAttribute(TfToken(name))) {
            a.Get(&value);
        }
        digest += name;
        digest += '=';
        digest += value.GetString();
        digest += '|';
    };

    // Weight-object descriptor shape is epoch identity (spec §4.1):
    // target, representation, policy, and canonical sparse support.
    //
    // Recursive, because a combine's field shape is its inputs' shapes:
    // an edit inside a composed input has to re-epoch the combine that
    // folds it, or the baked falloff tables replay stale.
    //
    // Cycle-TRACKED rather than depth-limited. A depth cap terminates,
    // but it terminates by silently dropping everything below it, so a
    // legitimately deep composition stops contributing to the epoch
    // identity and edits down there stop triggering a recompile. Marking
    // the path being walked costs the same and is exact; a genuine cycle
    // is caught and reported by the compile-time walk instead.
    std::set<SdfPath> digestVisiting;
    std::function<void(const SdfPath &)> appendWeightObject =
        [&](const SdfPath &weightPath) {
        const UsdPrim w = _stage->GetPrimAtPath(weightPath);
        if (!w || !digestVisiting.insert(weightPath).second) {
            return;
        }
        struct Pop {
            std::set<SdfPath> &s;
            const SdfPath &p;
            ~Pop() { s.erase(p); }
        } pop{digestVisiting, weightPath};
        digest += w.GetTypeName().GetString();
        digest += '|';
        appendRelTargets(w, "rigExec:weightTarget", true);
        appendRelTargets(w, "rigExec:baseWeight", true);
        appendToken(w, "rigExec:representation");
        appendToken(w, "rigExec:rangePolicy");
        appendToken(w, "rigExec:operation");
        VtIntArray indices;
        if (UsdAttribute a = w.GetAttribute(TfToken("rigExec:indices"))) {
            a.Get(&indices);
        }
        std::vector<int> support(indices.begin(), indices.end());
        std::sort(support.begin(), support.end());
        for (int i : support) {
            digest += std::to_string(i);
            digest += ',';
        }
        digest += '|';

        // Volumetric extension. Only STRUCTURAL properties belong here:
        // the shape family, which axis it measures, where it samples,
        // and the baked remap. inputs:falloffMin/Max, invert, strength,
        // scaleX/Y/Z and extentU/V are deliberately absent -- they are
        // per-frame exec values, and hashing them would recompile every
        // frame an artist scrubs one. rigExec:planeBounds IS here
        // because it selects which field function runs, exactly as
        // rigExec:planeAxis selects which coordinate it measures.
        appendToken(w, "rigExec:falloffProfile");
        appendToken(w, "rigExec:samplePhase");
        appendToken(w, "rigExec:planeAxis");
        appendToken(w, "rigExec:planeBounds");
        appendToken(w, "rigExec:combineMode");
        appendRelTargets(w, "rigExec:curve", true);
        appendRelTargets(w, "rigExec:sampleSource", true);
        // The falloff curve is structural: it is resampled to a table
        // once per epoch, so an edit to it has to begin a new one.
        //
        // What gets hashed is the BAKED TABLE, not the knots. Hashing
        // knot times and values misses everything else that changes the
        // curve's shape -- interpolation mode, tangent slopes and widths,
        // dual values, extrapolation, loops -- so flipping a knot from
        // curve to held left the digest unchanged, the epoch unrebuilt,
        // and exec replaying a stale LUT while the CPU oracle rebaked the
        // live spline. Hashing the table is exact by construction: it is
        // precisely the bytes exec consumes, so anything that changes
        // them re-epochs and nothing that does not, does.
        //
        // This also folds in rigExec:falloffProfile, which is why that
        // token is not hashed separately.
        if (_IsVolumeWeightType(w.GetTypeName())) {
            const std::vector<float> lut = _BakeFalloffLut(w);
            digest += "lut=";
            digest.append(reinterpret_cast<const char *>(lut.data()),
                          lut.size() * sizeof(float));
            digest += '|';
        }

        // Composition order is semantic for subtract and overlay, so the
        // input list is hashed UNSORTED -- unlike every other
        // relationship here, whose permutation is explicitly not.
        SdfPathVector inputs;
        if (UsdRelationship rel =
                w.GetRelationship(TfToken("rigExec:inputWeights"))) {
            rel.GetTargets(&inputs);
        }
        digest += "rigExec:inputWeights=";
        for (const SdfPath &t : inputs) {
            digest += t.GetString();
            digest += ',';
        }
        digest += '|';
        for (const SdfPath &t : inputs) {
            appendWeightObject(t);
        }
        // A dynamic weight's base is composed the same way.
        SdfPathVector bases;
        if (UsdRelationship rel =
                w.GetRelationship(TfToken("rigExec:baseWeight"))) {
            rel.GetTargets(&bases);
        }
        for (const SdfPath &t : bases) {
            appendWeightObject(t);
        }
    };

    // Rig output set. Discovered rather than authored, so the digest hashes the
    // discovered paths -- adding, removing, or renaming a joint prim changes
    // the epoch exactly as editing the old manifest relationship did.
    // Derived-property maintenance (spec §7.6 revised) is unconditional now, so
    // there is no policy token left to hash: what the compiler synthesizes depends
    // only on which gprims author normals/extent, and that is already epoch
    // identity through the points-chain targets below.
    for (const SdfPath &jointPath : _DiscoverJointOutputs(_stage, _rigPath)) {
        digest += jointPath.GetString();
        digest += ',';
    }
    digest += '|';

    // The discovered control set, for the same reason: it decides which
    // computePointFrame taps the epoch's prepared request carries. A control
    // that no solver reads is otherwise invisible to this digest -- adding
    // one purely to draw a guide would leave the compiled tap set behind and
    // the guide would never appear.
    for (const SdfPath &controlPath : _DiscoverControls(_stage, _rigPath)) {
        digest += controlPath.GetString();
        digest += ',';
    }
    digest += '|';

    // Solver->joint wiring is epoch identity (view-free extraction,
    // user-directed 2026-07-25, replaces RigExecPointFrameView): each
    // solver's ORDERED rigExec:joints list decides which joint
    // self-extracts which aggregate element, so adding, removing, or
    // reordering joints changes what compile Pass 0 synthesizes. Order is
    // semantic (position = element index), so this list is never sorted.
    if (const UsdPrim solvers = _stage->GetPrimAtPath(
            _rigPath.AppendChild(TfToken("Solvers")))) {
        // Recursive over the composed Solvers subtree (not GetChildren):
        // nested solver scopes must contribute to epoch identity too
        // (consistent with mover discovery and compile Pass 0).
        static const std::set<TfToken> kAggregateSolverTypes = {
            TfToken("RigExecFkChain"), TfToken("RigExecTwoBoneIk"),
            TfToken("RigExecBlendPointFrames"),
            TfToken("RigExecTwistDistribution"),
            TfToken("RigExecRibbon")};
        for (const UsdPrim &solver : UsdPrimRange(solvers)) {
            SdfPathVector joints;
            if (const UsdRelationship rel =
                    solver.GetRelationship(TfToken("rigExec:joints"))) {
                rel.GetTargets(&joints);
            }
            // Emit for joint-bearing prims (their bindings) AND for every
            // aggregate solver even without joints: its cardinality feeds
            // Phase A element checks, possibly indirectly through a Blend
            // input, so a cardinality edit must begin a new epoch
            // (codex round-4).
            const bool isAggregate =
                kAggregateSolverTypes.count(solver.GetTypeName()) > 0;
            if (joints.empty() && !isAggregate) {
                continue;
            }
            digest += solver.GetPath().GetString();
            digest += '|';
            digest += solver.GetTypeName().GetString();
            digest += '|';
            for (const SdfPath &j : joints) {
                digest += j.GetString();
                digest += ',';
            }
            // Element remap is structural: it changes which frame each
            // joint self-extracts. Parallel to joints, so not sorted.
            digest += '|';
            const UsdAttribute jeAttr =
                solver.GetAttribute(TfToken("rigExec:jointElements"));
            VtIntArray jointElements;
            if (jeAttr) {
                jeAttr.Get(&jointElements);
            }
            for (int e : jointElements) {
                digest += std::to_string(e);
                digest += ',';
            }
            // jointElements is a static input; record its time-sample count
            // so adding a sample post-compile re-runs Compile()'s rejection
            // rather than silently keeping the captured default (round-6).
            digest += "s" + std::to_string(
                                jeAttr ? jeAttr.GetNumTimeSamples() : 0);
            // Cardinality-determining inputs: an edit that changes how many
            // frames the solver produces must recompile so Phase A
            // re-validates every element binding (codex round-3). Value-only
            // edits that don't change frame count stay value-only.
            digest += "|card=";
            const TfToken stype = solver.GetTypeName();
            if (stype == "RigExecFkChain") {
                appendRelTargets(solver, "rigExec:controls", false);
            } else if (stype == "RigExecTwistDistribution") {
                const UsdAttribute ca =
                    solver.GetAttribute(TfToken("rigExec:count"));
                const UsdAttribute wa =
                    solver.GetAttribute(TfToken("rigExec:weights"));
                int cnt = 1;
                if (ca) {
                    ca.Get(&cnt);
                }
                VtFloatArray w;
                if (wa) {
                    wa.Get(&w);
                }
                // Effective cardinality (weights wins; ignored count value
                // does not churn the epoch, codex round-5 LOW) plus the
                // time-sample presence of BOTH attrs so that ADDING a
                // sample without changing the default still changes the
                // digest, forcing the recompile that re-runs the pre-pass
                // sample rejection (codex round-5 MAJOR).
                const size_t effective =
                    !w.empty() ? w.size()
                               : static_cast<size_t>(std::max(cnt, 1));
                digest += std::to_string(effective) + "/" +
                          std::to_string(ca ? ca.GetNumTimeSamples() : 0) +
                          "/" +
                          std::to_string(wa ? wa.GetNumTimeSamples() : 0) +
                          ",";
            } else if (stype == "RigExecRibbon") {
                const UsdAttribute a =
                    solver.GetAttribute(TfToken("rigExec:sampleCount"));
                int sc = 5;
                if (a) {
                    a.Get(&sc);
                }
                digest += std::to_string(sc) + "/" +
                          std::to_string(a ? a.GetNumTimeSamples() : 0) + ",";
                // The driver curve lowers (Pass 1.5) to generated
                // resolvedDriverPoints + a bind-time restDriverPoints
                // capture, so rewiring it (or a layer-mute/variant switch
                // that retargets it) is structural (codex round-6).
                appendRelTargets(solver, "rigExec:driverCurve", false);
            } else if (stype == "RigExecBlendPointFrames") {
                appendRelTargets(solver, "rigExec:inputA", false);
                appendRelTargets(solver, "rigExec:inputB", false);
            }
            digest += ';';
        }
    }

    const UsdPrim movers =
        _stage->GetPrimAtPath(_rigPath.AppendChild(TfToken("Movers")));
    if (movers) {
        UsdPrimRange range = UsdPrimRange::PreAndPostVisit(movers);
        for (auto it = range.begin(); it != range.end(); ++it) {
            if (!it.IsPostVisit()) {
                continue;
            }
            const UsdPrim prim = *it;
            const UsdRelationship moves = prim.GetRelationship(_movesRel);
            if (!moves) {
                continue;
            }
            digest += prim.GetPath().GetString();
            digest += '|';
            digest += prim.GetTypeName().GetString();
            digest += '|';
            SdfPathVector targets;
            moves.GetTargets(&targets);
            // Target-list order is non-semantic (spec §4.2): sort before
            // hashing so a permutation does not change the epoch.
            std::sort(targets.begin(), targets.end());
            for (const SdfPath &t : targets) {
                const SdfPath canonical = _CanonicalizeTarget(_stage, t);
                digest += canonical.GetString();
                digest += ',';
                // Derived synthesis identity (spec §7.6 revised): whether
                // a written points target's gprim authors the derived
                // properties decides what the compiler synthesizes, so
                // authoring or removing them is a structural edit.
                if (canonical.IsPropertyPath() &&
                    canonical.GetNameToken() == "points") {
                    const SdfPath owner = canonical.GetPrimPath();
                    auto authored = [this, &owner](const char *name) {
                        const UsdAttribute a = _stage->GetAttributeAtPath(
                            owner.AppendProperty(TfToken(name)));
                        return a && a.HasAuthoredValue();
                    };
                    digest += "derived=";
                    // Owner schema type gates mesh-only normal synthesis.
                    if (const UsdPrim ownerPrim =
                            _stage->GetPrimAtPath(owner)) {
                        digest += ownerPrim.GetTypeName().GetString();
                    }
                    digest += authored("normals") ? 'n' : '-';
                    digest += authored("extent") ? 'e' : '-';
                    digest += authored("widths") ? 'w' : '-';
                    digest += ',';
                }
            }
            digest += ';';

            // Declared dependency wiring and read phases.
            appendRelTargets(prim, "rigExec:transform", true);
            appendToken(prim, "rigExec:transformReadPhase");
            appendToken(prim, "rigExec:operation");
            appendToken(prim, "rigExec:mode");
            // Authored order: aim compilation consumes aims[0] and there is no
            // exactly-one aimTarget validation, so a reorder that changes
            // the selected provider must change the digest (codex round-9).
            appendRelTargets(prim, "rigExec:aimTarget", false);
            // Static-input relationships captured at compile into generated
            // resolved*/rest* wiring (lattice cage, surface, curve bind/
            // driver): retargeting any of these must recompile so the
            // captured bind-time values are refreshed (codex round-7,
            // pre-existing general mover-digest gap). Hashed in AUTHORED
            // order (sorted=false) because the compiler consumes targets[0], so
            // a reorder that changes the selected input must change the
            // digest (codex round-8). An absent rel appends a constant
            // empty marker (harmless, invariant per mover type).
            appendRelTargets(prim, "rigExec:cage", false);
            appendRelTargets(prim, "rigExec:surface", false);
            appendRelTargets(prim, "rigExec:bindCoordinates", false);
            appendRelTargets(prim, "rigExec:driverFrames", false);
            appendRelTargets(prim, "rigExec:driverCurve", false);
            for (const SdfPath &w :
                 appendRelTargets(prim, "rigExec:weightObject", true)) {
                appendWeightObject(w);
            }
            for (const SdfPath &inputPath :
                 appendRelTargets(prim, "rigExec:blendInputs", true)) {
                const UsdPrim input = _stage->GetPrimAtPath(inputPath);
                if (!input) {
                    continue;
                }
                for (const SdfPath &samplePath :
                     appendRelTargets(input, "rigExec:samples", true)) {
                    const UsdPrim sample =
                        _stage->GetPrimAtPath(samplePath.GetPrimPath());
                    if (!sample) {
                        continue;
                    }
                    appendRelTargets(sample, "rigExec:targetPoints", true);
                    float activation = 1;
                    if (UsdAttribute a = sample.GetAttribute(
                            TfToken("rigExec:activation"))) {
                        a.Get(&activation);
                    }
                    // Activation edits are structural (spec §7.3).
                    digest += std::to_string(activation);
                    digest += '|';
                }
            }
            digest += ';';
        }
    }
    return std::hash<std::string>{}(digest);
}

bool
RigExecRigEvaluator::Compile(std::vector<std::string> *errors)
{
    auto reportError = [errors](const std::string &message) {
        if (errors) {
            errors->push_back(message);
        }
    };

    if (!_stage) {
        reportError("no stage; nothing to compile");
        return false;
    }

    const UsdPrim rig = _stage->GetPrimAtPath(_rigPath);
    if (!rig) {
        reportError("Rig prim not found: " + _rigPath.GetString());
        return false;
    }
    // Prototype-hosted rigs fail validation (spec §4.1: a rig in a
    // prototype or otherwise unable to deinstance is rejected).
    if (rig.IsInstanceProxy() || rig.IsInPrototype()) {
        reportError("Rig is instance-proxy/prototype hosted: " +
                    _rigPath.GetString());
        return false;
    }

    // Phase A: validation into locals. Nothing below mutates evaluator
    // state until every check passes, so a failed structural edit keeps
    // the previous epoch publishable (spec §4.1 atomic transactions).
    std::vector<SdfPath> newJointPaths =
        _DiscoverJointOutputs(_stage, _rigPath);
    if (newJointPaths.empty()) {
        reportError("Rig has no RigExecJoint prims to publish: " +
                    _rigPath.GetString());
        return false;
    }
    // Controls are discovered alongside the joints but never gate the
    // compile: zero controls is an ordinary rig, not a broken one.
    std::vector<SdfPath> newControlPaths =
        _DiscoverControls(_stage, _rigPath);

    // Transform-authority validation (host-durability redesign).
    //
    // Neither condition can FAIL a compile, and both are reported rather
    // than fixed: the rig still evaluates exactly right, because the
    // evaluator reads rest:space and the avars and nothing else. What
    // breaks is the BOUNDS -- a provider's computed extent bakes its posed
    // frame into asset-relative space, which is only the whole story while
    // nothing else contributes a transform between the asset root and the
    // provider. Refusing to compile over a framing inaccuracy would be
    // wildly out of proportion; saying nothing would leave an author
    // wondering why one control frames to the wrong place.
    {
        const SdfPath assetRoot = _rigPath.GetParentPath();
        auto warn = [errors](const std::string &message) {
            // Both channels on purpose: TF_WARN is what a host surfaces to
            // the author, and the errors vector is what a test can read.
            // Compile still returns true.
            if (errors) {
                errors->push_back("warning: " + message);
            }
            TF_WARN("%s", message.c_str());
        };
        // Every Boundable provider, aggregate solvers included: they
        // inherit Boundable/Xformable too, so an authored op on one is
        // applied by BBoxCache to an already-baked extent while the guide
        // it draws ignores it entirely.
        std::vector<SdfPath> providers = newJointPaths;
        providers.insert(providers.end(), newControlPaths.begin(),
                         newControlPaths.end());
        {
            static const std::set<TfToken> kBoundableSolverTypes = {
                TfToken("RigExecFkChain"), TfToken("RigExecTwoBoneIk"),
                TfToken("RigExecBlendPointFrames"),
                TfToken("RigExecTwistDistribution"),
                TfToken("RigExecRibbon")};
            if (const UsdPrim solverRoot = _stage->GetPrimAtPath(
                    _rigPath.AppendChild(TfToken("Solvers")))) {
                for (const UsdPrim &solver : UsdPrimRange(solverRoot)) {
                    if (kBoundableSolverTypes.count(solver.GetTypeName())) {
                        providers.push_back(solver.GetPath());
                    }
                }
            }
        }
        for (const SdfPath &providerPath : providers) {
            const UsdPrim prim = _stage->GetPrimAtPath(providerPath);
            if (!prim) {
                continue;
            }
            // xformOps arrive on every provider now that RigExecXformable
            // inherits UsdGeomBoundable, but they are NOT a transform
            // authority: rest:space plus the avars are the only one (the
            // Ir alignment). An authored op is a second one that nothing
            // reads, so the prim moves in a stock UsdGeom traversal while
            // the rig ignores it entirely.
            if (const UsdGeomXformable xformable = UsdGeomXformable(prim)) {
                bool resetsStack = false;
                if (!xformable.GetOrderedXformOps(&resetsStack).empty()) {
                    warn(prim.GetTypeName().GetString() + " " +
                         providerPath.GetString() +
                         " authors xformOps, which are not a transform "
                         "authority for a RigExec provider (rest:space and "
                         "the avars are); the ops are ignored by evaluation "
                         "and are not in the computed extent");
                }
            }
            // ...and nothing between the provider and the asset root may
            // contribute one either. RigExec's own types are skipped: a
            // joint nested under a joint is the ordinary shape of a rig,
            // and the loop above already polices ops authored on those.
            for (SdfPath ancestorPath = providerPath.GetParentPath();
                 ancestorPath != assetRoot &&
                     !ancestorPath.IsAbsoluteRootPath() &&
                     !ancestorPath.IsEmpty();
                 ancestorPath = ancestorPath.GetParentPath()) {
                const UsdPrim ancestor = _stage->GetPrimAtPath(ancestorPath);
                if (!ancestor) {
                    break;
                }
                if (TfStringStartsWith(ancestor.GetTypeName().GetString(),
                                       "RigExec")) {
                    continue;
                }
                if (UsdGeomXformable(ancestor)) {
                    warn("Xformable " + ancestorPath.GetString() +
                         " sits between the asset root and provider " +
                         providerPath.GetString() +
                         "; its transform is not composed into the "
                         "provider's frames, so the computed extent places "
                         "the guide as if it were identity");
                }
            }

            // A provider's extent covers the guides beneath it, and only
            // those. Authored geometry parented under one is invisible to
            // it -- and to every ancestor, because UsdGeomBBoxCache stops
            // descending at a Boundable -- so the gprim silently drops out
            // of every bounding box in the scene.
            for (const UsdPrim &descendant : UsdPrimRange(prim)) {
                if (descendant == prim) {
                    continue;
                }
                // A provider nested under a provider with a DIFFERENT
                // resolved purpose is dropped from the ancestor's extent
                // on purpose: one extent carries one purpose, and the
                // bounding-box cache files it under the ancestor's. Nobody
                // reading the namespace would guess that, so say it.
                if (descendant.IsA<UsdGeomImageable>()) {
                    const UsdGeomImageable descendantImageable(descendant);
                    const UsdGeomImageable providerImageable(prim);
                    const TfToken descendantPurpose =
                        descendantImageable.ComputePurpose();
                    const TfToken providerPurpose =
                        providerImageable.ComputePurpose();
                    if (!descendantPurpose.IsEmpty() &&
                        !providerPurpose.IsEmpty() &&
                        descendantPurpose != providerPurpose &&
                        TfStringStartsWith(
                            descendant.GetTypeName().GetString(),
                            "RigExec")) {
                        warn(descendant.GetTypeName().GetString() + " " +
                             descendant.GetPath().GetString() +
                             " has purpose '" +
                             descendantPurpose.GetString() +
                             "' but is nested under " +
                             providerPath.GetString() + " whose purpose is '" +
                             providerPurpose.GetString() +
                             "'; one extent carries one purpose, so this "
                             "provider is excluded from its ancestor's "
                             "bounds");
                    }
                }
                if (descendant.IsA<UsdGeomGprim>()) {
                    warn("gprim " + descendant.GetPath().GetString() +
                         " is parented under RigExec provider " +
                         providerPath.GetString() +
                         "; a provider's computed extent covers only the "
                         "guides beneath it, and bounds stop descending at "
                         "a Boundable, so this geometry is absent from "
                         "every bounding box that should contain it");
                }
            }
        }
    }

    std::vector<SdfPath> solverArrayPaths;
    std::map<SdfPath, SdfPath> newRibbonDriverPoints;
    const UsdPrim solvers =
        _stage->GetPrimAtPath(_rigPath.AppendChild(TfToken("Solvers")));
    if (solvers) {
        // Every aggregate frame provider: their computePointFrameArray
        // results are published (and drawn as guides by the imaging
        // chain, like OpenExec's IrJointScope guides).
        static const std::set<TfToken> aggregateSolverTypes = {
            TfToken("RigExecFkChain"),
            TfToken("RigExecTwoBoneIk"),
            TfToken("RigExecBlendPointFrames"),
            TfToken("RigExecTwistDistribution"),
            TfToken("RigExecRibbon")};
        for (const UsdPrim &child : UsdPrimRange(solvers)) {
            if (aggregateSolverTypes.count(child.GetTypeName())) {
                solverArrayPaths.push_back(child.GetPath());
            }
            // A ribbon's driver-curve points, resolved to the exact native
            // attribute. This replaces the compiler's last authoring pass:
            // the resolution is compiled state (rewiring the relationship is
            // structural, and the epoch digest already treats it that way),
            // and the values ride in as exec overrides at evaluation time.
            if (child.GetTypeName() == "RigExecRibbon") {
                SdfPathVector curves;
                if (const UsdRelationship rel = child.GetRelationship(
                        TfToken("rigExec:driverCurve"))) {
                    rel.GetTargets(&curves);
                }
                if (!curves.empty()) {
                    newRibbonDriverPoints[child.GetPath()] =
                        curves[0].IsPrimPath()
                            ? curves[0].AppendProperty(TfToken("points"))
                            : curves[0];
                }
            }
        }
    }

    // Mover discovery: post-order depth-first walk of the composed Movers
    // namespace, descendants first, branches in composed child order
    // (spec §4.2, UsdPrimRange::PreAndPostVisit).
    std::vector<RigExecMoverRecord> newMovers;
    const UsdPrim movers =
        _stage->GetPrimAtPath(_rigPath.AppendChild(TfToken("Movers")));
    int ordinal = 0;
    if (movers) {
        UsdPrimRange range = UsdPrimRange::PreAndPostVisit(movers);
        for (auto it = range.begin(); it != range.end(); ++it) {
            if (!it.IsPostVisit()) {
                continue;
            }
            const UsdPrim prim = *it;
            const UsdRelationship moves = prim.GetRelationship(_movesRel);
            if (!moves) {
                continue;  // grouping scope
            }
            SdfPathVector targets;
            moves.GetTargets(&targets);
            if (targets.empty()) {
                reportError("Mover has no moves targets: " +
                            prim.GetPath().GetString());
                return false;
            }

            RigExecMoverRecord record;
            record.moverPath = prim.GetPath();
            record.schemaType = prim.GetTypeName();
            record.ordinal = ordinal++;

            // Structural/topology properties are never writable move
            // targets (spec §4.2, §7.7).
            static const std::set<TfToken> structuralProperties = {
                TfToken("faceVertexCounts"), TfToken("faceVertexIndices"),
                TfToken("holeIndices"), TfToken("curveVertexCounts"),
                TfToken("cornerIndices"), TfToken("cornerSharpnesses"),
                TfToken("creaseIndices"), TfToken("creaseLengths"),
                TfToken("creaseSharpnesses"), TfToken("subdivisionScheme"),
                TfToken("type"), TfToken("basis"), TfToken("wrap"),
                TfToken("orientation"), TfToken("doubleSided")};
            const SdfPath assetRoot = _rigPath.GetParentPath();

            for (const SdfPath &t : targets) {
                const SdfPath canonical = _CanonicalizeTarget(_stage, t);
                const SdfPath primPath = canonical.GetPrimPath();
                // Reject dangling targets (spec §4.2).
                if (!_stage->GetPrimAtPath(primPath)) {
                    reportError("Mover " + prim.GetPath().GetString() +
                                " targets missing prim " +
                                primPath.GetString());
                    return false;
                }
                // Cross-rig writes are rejected in v1 (spec §4.2).
                if (!primPath.HasPrefix(assetRoot)) {
                    reportError("Mover " + prim.GetPath().GetString() +
                                " targets outside the rig asset: " +
                                canonical.GetString());
                    return false;
                }
                if (canonical.IsPropertyPath()) {
                    if (structuralProperties.count(canonical.GetNameToken())) {
                        reportError(
                            "Mover " + prim.GetPath().GetString() +
                            " targets structural property " +
                            canonical.GetString());
                        return false;
                    }
                    // Derived properties are compiler-maintained (spec
                    // §7.6 revised): no authored mover writes them.
                    if (canonical.GetNameToken() == "normals" ||
                        canonical.GetNameToken() == "extent") {
                        reportError(
                            "Mover " + prim.GetPath().GetString() +
                            " targets derived property " +
                            canonical.GetString() +
                            "; normals/extent maintenance is synthesized "
                            "by the compiler");
                        return false;
                    }
                    if (!_stage->GetAttributeAtPath(canonical)) {
                        reportError(
                            "Mover " + prim.GetPath().GetString() +
                            " targets missing property " +
                            canonical.GetString());
                        return false;
                    }
                }
                // Duplicate targets after canonicalization are rejected
                // (spec §6.1).
                if (std::find(record.targets.begin(), record.targets.end(),
                              canonical) != record.targets.end()) {
                    reportError("Mover " + prim.GetPath().GetString() +
                                " has duplicate canonical target " +
                                canonical.GetString());
                    return false;
                }
                record.targets.push_back(canonical);
            }

            // Matrix movers narrow the general rule (spec §4.2): moves,
            // transform, and weightObject each have cardinality one; the
            // move target is a native PointBased points property; the
            // weight object canonicalizes to that same target.
            if (record.schemaType == "RigExecMatrixMover") {
                std::string error;
                if (!_ValidateMatrixMover(prim, record, &error)) {
                    reportError(error);
                    return false;
                }
            }
            // Blend movers: the mover-owned parameter packet derives
            // deltas against one declared base, so v0.1 supports exactly
            // one canonical points target per blend mover. Multi-target
            // fan-out requires per-target parameter specialization and
            // is rejected rather than silently computed wrong.
            if (record.schemaType == "RigExecBlendShapeMover" &&
                record.targets.size() != 1) {
                reportError("BlendShapeMover " +
                            prim.GetPath().GetString() +
                            " must have exactly one canonical target in "
                            "v0.1 (multi-target fan-out unsupported)");
                return false;
            }
            // Typed geometry movers move points: every canonical target
            // must be a native points property — anything else would
            // pass validation yet silently produce no application.
            {
                static const std::set<TfToken> pointsMoverTypes = {
                    TfToken("RigExecSmoothMover"),
                    TfToken("RigExecVolumeCorrectMover"),
                    TfToken("RigExecLatticeMover"),
                    TfToken("RigExecSurfaceMover"),
                    TfToken("RigExecCurveMover"),
                    TfToken("RigExecBlendShapeMover")};
                if (pointsMoverTypes.count(record.schemaType)) {
                    for (const SdfPath &t : record.targets) {
                        const UsdPrim owner = t.IsPropertyPath()
                            ? _stage->GetPrimAtPath(t.GetPrimPath())
                            : UsdPrim();
                        const UsdAttribute attr = t.IsPropertyPath()
                            ? _stage->GetAttributeAtPath(t)
                            : UsdAttribute();
                        if (!t.IsPropertyPath() ||
                            t.GetNameToken() != "points" ||
                            !owner || !owner.IsA<UsdGeomPointBased>() ||
                            !attr ||
                            attr.GetTypeName() !=
                                SdfValueTypeNames->Point3fArray) {
                            reportError(
                                record.schemaType.GetString() + " " +
                                prim.GetPath().GetString() +
                                " target " + t.GetString() +
                                " is not a native UsdGeomPointBased "
                                "point3f[] points attribute");
                            return false;
                        }
                    }
                }
            }
            // Smooth/volume/lattice movers own one mover-level parameter
            // packet resolved against their target, so v0.1 supports
            // exactly one canonical points target each (like blend
            // movers); multi-target fan-out would alias the last
            // target's parameters onto every application.
            if ((record.schemaType == "RigExecSmoothMover" ||
                 record.schemaType == "RigExecVolumeCorrectMover" ||
                 record.schemaType == "RigExecLatticeMover") &&
                record.targets.size() != 1) {
                reportError(record.schemaType.GetString() + " " +
                            prim.GetPath().GetString() +
                            " must have exactly one canonical points "
                            "target in v0.1 (multi-target fan-out would "
                            "alias mover-level parameters)");
                return false;
            }
            // Structure-determining tokens must be static. `uniform` is a
            // convention, not an enforcement: USD permits time samples on a
            // uniform attribute, and these tokens select the compiled
            // operation and read phase. Sampling them per-frame would let the
            // op or binding change under a compiled epoch without changing
            // the binding-epoch digest, so reject samples here rather than
            // resolving them at evaluation time (codex round-3). Same rule
            // the aggregate cardinality attributes already follow.
            for (const char *name : {"rigExec:mode",
                                     "rigExec:transformReadPhase",
                                     "rigExec:cageReadPhase",
                                     "rigExec:surfaceReadPhase"}) {
                const UsdAttribute a = prim.GetAttribute(TfToken(name));
                if (a && a.GetNumTimeSamples() > 0) {
                    reportError(record.schemaType.GetString() + " " +
                                prim.GetPath().GetString() + ": " + name +
                                " must not be time-sampled (it selects the "
                                "compiled operation/read phase)");
                    return false;
                }
            }
            newMovers.push_back(std::move(record));
        }
    }

    // Same-target writers that are not nested (neither is a namespace
    // ancestor of the other) compete through composed child order at
    // their common ancestor; release validation requires an authored
    // child-reorder opinion there covering both branches (spec §4.2).
    {
        std::map<SdfPath, std::vector<const RigExecMoverRecord *>> byTarget;
        for (const RigExecMoverRecord &m : newMovers) {
            for (const SdfPath &t : m.targets) {
                byTarget[t].push_back(&m);
            }
        }
        for (const auto &[target, writers] : byTarget) {
            for (size_t i = 0; i < writers.size(); ++i) {
                for (size_t j = i + 1; j < writers.size(); ++j) {
                    const SdfPath &a = writers[i]->moverPath;
                    const SdfPath &b = writers[j]->moverPath;
                    if (a.HasPrefix(b) || b.HasPrefix(a)) {
                        continue;  // nested: hierarchy order rules
                    }
                    const SdfPath ancestor =
                        a.GetCommonPrefix(b);
                    // The two competing direct-child branch names at the
                    // common ancestor.
                    SdfPath branchA = a, branchB = b;
                    while (branchA.GetParentPath() != ancestor) {
                        branchA = branchA.GetParentPath();
                    }
                    while (branchB.GetParentPath() != ancestor) {
                        branchB = branchB.GetParentPath();
                    }
                    const UsdPrim parent = _stage->GetPrimAtPath(ancestor);
                    const TfTokenVector reorder =
                        parent ? parent.GetChildrenReorder()
                               : TfTokenVector();
                    const bool covered =
                        std::find(reorder.begin(), reorder.end(),
                                  branchA.GetNameToken()) != reorder.end() &&
                        std::find(reorder.begin(), reorder.end(),
                                  branchB.GetNameToken()) != reorder.end();
                    if (!covered) {
                        reportError(
                            "Ambiguous competing writers of " +
                            target.GetString() + ": " + a.GetString() +
                            " and " + b.GetString() +
                            " need nesting or an authored child reorder "
                            "at " + ancestor.GetString() +
                            " covering both branches (spec §4.2)");
                        return false;
                    }
                }
            }
        }
    }

    // final transform reads are legal only when every writer of that
    // provider precedes the reader in logical order (spec §4.2): a
    // frame mover with a later ordinal than a consuming matrix mover is
    // an unsatisfied final read.
    {
        std::map<SdfPath, int> lastFrameWriterOrdinal;
        for (const RigExecMoverRecord &m : newMovers) {
            if (m.schemaType != "RigExecAimConstraint") {
                continue;
            }
            for (const SdfPath &t : m.targets) {
                if (t.IsPrimPath()) {
                    lastFrameWriterOrdinal[t] = std::max(
                        lastFrameWriterOrdinal.count(t)
                            ? lastFrameWriterOrdinal[t] : -1,
                        m.ordinal);
                }
            }
        }
        for (const RigExecMoverRecord &m : newMovers) {
            if (m.schemaType != "RigExecMatrixMover") {
                continue;
            }
            const UsdPrim prim = _stage->GetPrimAtPath(m.moverPath);
            if (!prim) {
                continue;
            }
            TfToken phase("base");
            if (UsdAttribute a = prim.GetAttribute(
                    TfToken("rigExec:transformReadPhase"))) {
                a.Get(&phase);
            }
            if (phase != "final") {
                continue;
            }
            SdfPathVector transforms;
            if (UsdRelationship rel =
                    prim.GetRelationship(TfToken("rigExec:transform"))) {
                rel.GetTargets(&transforms);
            }
            for (const SdfPath &provider : transforms) {
                const auto it = lastFrameWriterOrdinal.find(provider);
                if (it != lastFrameWriterOrdinal.end() &&
                    it->second > m.ordinal) {
                    reportError(
                        "Unsatisfied final read: " +
                        m.moverPath.GetString() + " reads final of " +
                        provider.GetString() +
                        " but a writer with a later ordinal exists "
                        "(spec §4.2)");
                    return false;
                }
            }
        }
    }

    // View-free solver->joint binding validation (codex round-2). This
    // runs in Phase A, BEFORE any epoch teardown, so an invalid binding
    // rejects the compile while the previous epoch stays publishable (the
    // BLOCKER fix: compile Pass 0 must never be the first place a bad
    // binding is discovered, because by then the old layer is gone and
    // restoration would re-hit the same invalid state). Recursive over the
    // composed Solvers subtree. Authored-conflict checks read the SOURCE
    // stage (_stage).
    //
    // joint -> (posing solver, element). The validation below already
    // resolves and bounds-checks exactly this pair; keeping it is what lets
    // Evaluate extract each bound joint's frame from its solver's aggregate
    // and supply it as a value override, so the binding never has to be
    // authored anywhere (it used to become rigExec:frameSource /
    // rigExec:frameElement on the joint, in the derived layer).
    std::map<SdfPath, std::pair<SdfPath, int>> newJointBinding;
    {
        std::map<SdfPath, SdfPath> jointClaim;  // joint -> claiming solver
        const UsdPrim solverRoot = _stage->GetPrimAtPath(
            _rigPath.AppendChild(TfToken("Solvers")));
        if (solverRoot) {
            // Cardinality attributes must be static (compile-time
            // structural) for EVERY aggregate solver in the subtree, not
            // only joint-bearing ones: a non-joint Twist/Ribbon feeding a
            // joint-bearing Blend still determines that Blend's element
            // count, so a time-sampled cardinality would silently shift a
            // blend-bound joint's frame (codex round-4). `uniform` is only
            // a hint; reject samples explicitly.
            for (const UsdPrim &solver : UsdPrimRange(solverRoot)) {
                const TfToken t = solver.GetTypeName();
                std::vector<const char *> cardinalityAttrs;
                if (t == "RigExecTwistDistribution") {
                    cardinalityAttrs = {"rigExec:count", "rigExec:weights"};
                } else if (t == "RigExecRibbon") {
                    cardinalityAttrs = {"rigExec:sampleCount"};
                }
                for (const char *name : cardinalityAttrs) {
                    const UsdAttribute a = solver.GetAttribute(TfToken(name));
                    if (a && a.GetNumTimeSamples() > 0) {
                        reportError(
                            solver.GetTypeName().GetString() + " " +
                            solver.GetPath().GetString() + ": " + name +
                            " must not be time-sampled (it defines frame "
                            "cardinality)");
                        return false;
                    }
                }
            }
            for (const UsdPrim &solver : UsdPrimRange(solverRoot)) {
                const UsdRelationship jointsRel =
                    solver.GetRelationship(TfToken("rigExec:joints"));
                if (!jointsRel) {
                    continue;
                }
                SdfPathVector jointTargets;
                jointsRel.GetTargets(&jointTargets);
                if (jointTargets.empty()) {
                    continue;
                }
                const std::string who = solver.GetTypeName().GetString() +
                                        " " + solver.GetPath().GetString();

                // The claimant must be a real aggregate solver (it must
                // publish computePointFrameArray for the joint to extract).
                // A Scope or arbitrary prim carrying rigExec:joints would
                // otherwise pass, and the binding would name a prim with no
                // aggregate computation to extract an element from.
                static const std::set<TfToken> kAggregateSolverTypes = {
                    TfToken("RigExecFkChain"),
                    TfToken("RigExecTwoBoneIk"),
                    TfToken("RigExecBlendPointFrames"),
                    TfToken("RigExecTwistDistribution"),
                    TfToken("RigExecRibbon")};
                if (!kAggregateSolverTypes.count(solver.GetTypeName())) {
                    reportError(who + " authors rigExec:joints but is not a "
                                      "recognized aggregate solver type");
                    return false;
                }

                // jointElements parallel-array shape: empty, or exactly one
                // entry per joint (no partial remap / silent truncation).
                VtIntArray jointElements;
                if (const UsdAttribute a = solver.GetAttribute(
                        TfToken("rigExec:jointElements"))) {
                    std::vector<double> times;
                    if (a.GetTimeSamples(&times) && !times.empty()) {
                        reportError(who + ": rigExec:jointElements must not "
                                          "carry time samples");
                        return false;
                    }
                    a.Get(&jointElements);
                }
                if (!jointElements.empty() &&
                    jointElements.size() != jointTargets.size()) {
                    reportError(
                        who + ": rigExec:jointElements length " +
                        std::to_string(jointElements.size()) +
                        " must equal rigExec:joints length " +
                        std::to_string(jointTargets.size()));
                    return false;
                }

                // Knowable aggregate element count per solver type
                // (-1 = not cheaply knowable, e.g. a blend of another
                // aggregate: element bounds are enforced only at runtime).
                int knownCount = -1;
                const TfToken type = solver.GetTypeName();
                if (type == "RigExecTwoBoneIk") {
                    knownCount = 3;
                } else if (type == "RigExecFkChain") {
                    SdfPathVector controls;
                    if (const UsdRelationship c = solver.GetRelationship(
                            TfToken("rigExec:controls"))) {
                        c.GetTargets(&controls);
                    }
                    knownCount = static_cast<int>(controls.size());
                } else if (type == "RigExecTwistDistribution") {
                    // count/weights time samples already rejected above.
                    VtFloatArray weights;
                    if (const UsdAttribute a = solver.GetAttribute(
                            TfToken("rigExec:weights"))) {
                        a.Get(&weights);
                    }
                    if (!weights.empty()) {
                        knownCount = static_cast<int>(weights.size());
                    } else if (const UsdAttribute a = solver.GetAttribute(
                                   TfToken("rigExec:count"))) {
                        int c = 1;
                        a.Get(&c);
                        knownCount = std::max(c, 1);
                    }
                } else if (type == "RigExecRibbon") {
                    if (const UsdAttribute a = solver.GetAttribute(
                            TfToken("rigExec:sampleCount"))) {
                        int c = 5;
                        a.Get(&c);
                        // The transported-frame ribbon needs >= 2 samples;
                        // below that it produces no frames, so no element
                        // is bindable (matches the runtime cardinality).
                        knownCount = c >= 2 ? c : 0;
                    }
                }

                for (size_t i = 0; i < jointTargets.size(); ++i) {
                    const SdfPath &jointPath = jointTargets[i];
                    const int element = i < jointElements.size()
                        ? jointElements[i] : static_cast<int>(i);
                    if (element < 0) {
                        reportError(who + ": negative element index " +
                                    std::to_string(element) + " for " +
                                    jointPath.GetString());
                        return false;
                    }
                    if (knownCount >= 0 && element >= knownCount) {
                        reportError(
                            who + ": element " + std::to_string(element) +
                            " for " + jointPath.GetString() +
                            " is out of range (solver produces " +
                            std::to_string(knownCount) + " frames)");
                        return false;
                    }
                    const UsdPrim jointPrim =
                        _stage->GetPrimAtPath(jointPath);
                    if (!jointPrim) {
                        reportError(who + " rigExec:joints targets missing "
                                          "prim " + jointPath.GetString());
                        return false;
                    }
                    if (jointPrim.GetTypeName() != "RigExecJoint") {
                        reportError(
                            who + " rigExec:joints target " +
                            jointPath.GetString() + " is a " +
                            jointPrim.GetTypeName().GetString() +
                            ", not a RigExecJoint");
                        return false;
                    }
                    // Every joint is tapped individually and a bound one also
                    // carries a per-prim value override, both of which key on
                    // a real prim; only the rig's nearest instanceable
                    // ancestor is deinstanced. Reject an instance-proxy /
                    // prototype-hosted joint up front.
                    if (jointPrim.IsInstanceProxy() ||
                        jointPrim.IsInPrototype()) {
                        reportError(
                            who + " rigExec:joints target " +
                            jointPath.GetString() +
                            " is instance-proxy/prototype hosted and cannot "
                            "receive a solver binding");
                        return false;
                    }
                    // Exclusive ownership: a solver-posed joint must not
                    // also author its own posed:space connection (the solver
                    // pose is supplied as an override and would silently win
                    // over the connection the raw stage shows).
                    //
                    // There is no longer a companion check for a legacy
                    // authored rigExec:frameSource. Nothing reads that name
                    // now -- it is neither a schema property nor a
                    // registered computation input -- so a leftover opinion
                    // from an asset saved against the old schema is inert,
                    // and failing the compile over it would reject a rig
                    // that evaluates correctly.
                    if (const UsdPrim srcJoint =
                            _stage->GetPrimAtPath(jointPath)) {
                        if (const UsdAttribute posed = srcJoint.GetAttribute(
                                TfToken("posed:space"))) {
                            SdfPathVector conns;
                            posed.GetConnections(&conns);
                            if (!conns.empty()) {
                                reportError(who + ": joint " +
                                            jointPath.GetString() +
                                            " also connects posed:space");
                                return false;
                            }
                        }
                    }
                    const auto claimed = jointClaim.find(jointPath);
                    if (claimed != jointClaim.end()) {
                        reportError("joint " + jointPath.GetString() +
                                    " is posed by two solvers (" +
                                    claimed->second.GetString() + " and " +
                                    solver.GetPath().GetString() + ")");
                        return false;
                    }
                    jointClaim[jointPath] = solver.GetPath();
                    newJointBinding[jointPath] = {solver.GetPath(), element};
                }
            }
        }
    }

    // Solver->solver acyclicity. Unique joint ownership does NOT imply this:
    // two solvers can each uniquely pose their own joints while reading each
    // other's, which is a genuine cycle (codex round-3). Evaluate resolves the
    // solver->joint overrides by iterating to a fixed point, and a cycle has
    // no fixed point to reach -- so reject it here, where the author gets a
    // path, instead of discovering it as a non-converging generation.
    //
    // Edges are derived generically: any relationship on a solver whose target
    // is a joint that another solver poses makes this solver depend on that
    // one. That needs no per-solver-type table of which relationships carry
    // frames, so a new solver type cannot quietly escape the check.
    {
        std::map<SdfPath, std::set<SdfPath>> dependsOn;
        const UsdPrim solverRoot = _stage->GetPrimAtPath(
            _rigPath.AppendChild(TfToken("Solvers")));
        if (solverRoot) {
            for (const UsdPrim &solver : UsdPrimRange(solverRoot)) {
                for (const UsdRelationship &rel :
                     solver.GetRelationships()) {
                    if (rel.GetName() == "rigExec:joints") {
                        continue;  // what it poses, not what it reads
                    }
                    SdfPathVector targets;
                    rel.GetTargets(&targets);
                    for (const SdfPath &target : targets) {
                        const SdfPath targetPrim = target.GetPrimPath();
                        // Indirect: reading a joint that another solver poses.
                        const auto it = newJointBinding.find(targetPrim);
                        if (it != newJointBinding.end() &&
                            it->second.first != solver.GetPath()) {
                            dependsOn[solver.GetPath()].insert(
                                it->second.first);
                        }
                        // Direct: reading another solver's aggregate.
                        // RigExecBlendPointFrames takes rigExec:inputA /
                        // inputB as solver paths, so a cycle can run through
                        // a solver->solver edge without touching a joint at
                        // all; deriving only the joint edges would miss it.
                        if (targetPrim != solver.GetPath() &&
                            targetPrim.HasPrefix(solverRoot.GetPath())) {
                            const UsdPrim other =
                                _stage->GetPrimAtPath(targetPrim);
                            if (other && other.GetTypeName() != "Scope") {
                                dependsOn[solver.GetPath()].insert(targetPrim);
                            }
                        }
                    }
                }
            }
        }
        // Iterative DFS with a colour map: grey means on the current stack.
        std::map<SdfPath, int> colour;  // 0 unvisited, 1 grey, 2 black
        std::vector<SdfPath> stack;
        std::function<bool(const SdfPath &)> visit =
            [&](const SdfPath &node) -> bool {
            colour[node] = 1;
            stack.push_back(node);
            for (const SdfPath &next : dependsOn[node]) {
                if (colour[next] == 1) {
                    std::string cycle;
                    bool started = false;
                    for (const SdfPath &p : stack) {
                        if (p == next) {
                            started = true;
                        }
                        if (started) {
                            cycle += p.GetString() + " -> ";
                        }
                    }
                    cycle += next.GetString();
                    reportError("solver dependency cycle: " + cycle +
                                " (each solver reads a joint the next one "
                                "poses, so no pose can be resolved)");
                    return false;
                }
                if (colour[next] == 0 && !visit(next)) {
                    return false;
                }
            }
            stack.pop_back();
            colour[node] = 2;
            return true;
        };
        for (const auto &[solverPath, _] : dependsOn) {
            if (colour[solverPath] == 0 && !visit(solverPath)) {
                return false;
            }
        }
    }

    const size_t newDigest = _ComputeStructureDigest();

    // Phase B: epoch replacement. This used to remove and re-author an
    // owned sublayer of generated prims, with a restore path for when the
    // authoring failed. Nothing is authored now, so the whole epoch swap is
    // just tearing down the previous exec system before building the next.
    _guideTaps.reset();
    _taps.reset();

    auto restorePreviousEpoch = [&]() {
        // The previous request was torn down with its taps above, so a
        // failed compile leaves the evaluator uncompiled either way.
        _compiled = false;
    };

    // Phase C: build and prepare the new epoch's taps before committing
    // any evaluator state; a request that cannot be built valid must not
    // become a "successful" epoch. Public tap addresses stay canonical
    // (native property or provider prim plus computation/phase);
    // generated chain heads are private resolutions (spec §9.1).
    static const TfToken basePhase("base");
    static const TfToken finalPhase("final");
    auto newTaps = std::make_unique<RigExecTapSet>(_stage);
    std::vector<RigExecTapId> newJointFrameTaps;
    std::vector<RigExecTapId> newJointFinalFrameTaps;
    std::vector<RigExecTapId> newJointFinalMatrixTaps;
    std::map<SdfPath, RigExecTapId> newSolverArrayTaps;
    std::map<SdfPath, RigExecTapId> newJointSolverArrayTaps;

    // Volumetric weight epoch state (spec §4.1 volumetric extension).
    std::vector<RigExecValueOverride> newFalloffLutOverrides;
    std::set<SdfPath> newCurrentPhaseWeights;
    std::map<SdfPath, RigExecTapId> newVolumeWeightMatrixTaps;

    // Walks a weight object and everything it composes, gathering what
    // the volumetric types need beyond their computeWeightPacket tap.
    // Depth-limited for the same reason the structure digest is: a cycle
    // is authoring error, and the bound only has to keep this
    // terminating.
    // Structural authoring errors on a volume weight, collected during
    // the walk below and reported before the epoch commits.
    //
    // These are cardinality rules on the points-bearing relationships,
    // and they exist because the two evaluation paths CANNOT disagree
    // about them safely: the exec kernel receives a relationship's
    // targets as one flattened value stream, so two targets on
    // rigExec:curve silently concatenate into one polyline with a
    // spurious segment joining them, while the CPU oracle reads targets
    // explicitly and rejects the pair. Catching it here means neither
    // path ever sees the ambiguous authoring.
    std::string volumeWeightError;

    // Weight objects currently being visited, for cycle detection. A
    // cycle is an authoring error and must be DIAGNOSED, not survived:
    // the CPU resolver recurses through the same edges with no depth
    // guard of its own, so an undetected cycle exhausts the stack rather
    // than producing a bad answer.
    std::set<SdfPath> visiting;

    // Returns true when this weight object, or anything it composes,
    // samples the in-flight points.
    //
    // The answer has to propagate UP: the graph build loop tests the
    // weight object a mover actually binds, which for a composed field is
    // the combine, not the sphere inside it. Recording only the leaf left
    // exec applying the reference-phase packet while the CPU oracle
    // reached a leaf with no in-flight points and failed.
    std::function<bool(const SdfPath &)> registerVolumeWeights =
        [&](const SdfPath &weightPath) -> bool {
        const UsdPrim w = _stage->GetPrimAtPath(weightPath);
        if (!w) {
            return false;
        }
        if (!visiting.insert(weightPath).second) {
            volumeWeightError =
                weightPath.GetString() +
                ": weight object composition contains a cycle";
            return false;
        }
        struct Pop {
            std::set<SdfPath> &s;
            const SdfPath &p;
            ~Pop() { s.erase(p); }
        } pop{visiting, weightPath};

        if (newVolumeWeightMatrixTaps.count(weightPath) ||
            newCurrentPhaseWeights.count(weightPath)) {
            // Already walked through another consumer; its answer stands.
            return newCurrentPhaseWeights.count(weightPath) != 0;
        }
        bool isCurrent = false;
        if (_IsVolumeWeightType(w.GetTypeName())) {
            auto requireTargets = [&](const char *rel, size_t exact,
                                      const char *what) {
                SdfPathVector targets;
                if (UsdRelationship r = w.GetRelationship(TfToken(rel))) {
                    r.GetTargets(&targets);
                }
                if (targets.size() > exact) {
                    volumeWeightError =
                        weightPath.GetString() + ": " + rel + " must name " +
                        what;
                }
                return targets.size();
            };
            // At most one sampling override; exactly one curve for a
            // curve weight.
            requireTargets("rigExec:sampleSource", 1,
                           "at most one points source");
            if (w.GetTypeName() == "RigExecCurveWeight" &&
                requireTargets("rigExec:curve", 1,
                               "exactly one points source") != 1) {
                volumeWeightError =
                    weightPath.GetString() +
                    ": rigExec:curve must name exactly one points source";
            }
        }
        if (_IsVolumeWeightType(w.GetTypeName())) {
            // computePointFrame, NOT computeMatrix: the latter is the
            // rest->posed map, so an unanimated volume's is the identity
            // and its field would land at the origin however the prim is
            // placed. See _RigidWorldToLocal in moverKernels.cpp.
            newVolumeWeightMatrixTaps[weightPath] =
                newTaps->Add(RigExecValueAddress::Prim(
                    weightPath, _computePointFrame));

            RigExecValueOverride lutOverride;
            lutOverride.prim = weightPath;
            lutOverride.computation = _computeFalloffLut;
            RigExecFalloffLut lut;
            lut.samples = _BakeFalloffLut(w);
            lutOverride.value = VtValue(lut);
            newFalloffLutOverrides.push_back(std::move(lutOverride));

            TfToken phase("reference");
            if (UsdAttribute a = w.GetAttribute(_samplePhaseAttr)) {
                a.Get(&phase);
            }
            if (phase == "current") {
                newCurrentPhaseWeights.insert(weightPath);
                isCurrent = true;
            }
        }
        for (const char *rel :
             {"rigExec:inputWeights", "rigExec:baseWeight"}) {
            SdfPathVector targets;
            if (UsdRelationship r = w.GetRelationship(TfToken(rel))) {
                r.GetTargets(&targets);
            }
            for (const SdfPath &t : targets) {
                // Not short-circuited: every reachable weight object
                // still needs its matrix tap and LUT override, so the
                // walk must complete even once the answer is known.
                if (registerVolumeWeights(t)) {
                    isCurrent = true;
                }
            }
        }
        // A composed field is current-phase if anything inside it is, so
        // that the combine a mover actually binds tests true.
        if (isCurrent) {
            newCurrentPhaseWeights.insert(weightPath);
        }
        return isCurrent;
    };

    // The aggregate frame array of every solver that poses a joint, in its
    // own request so it can be evaluated first: the authoritative request
    // below is computed with each bound joint's frame overridden by an
    // element of these. Solvers read controls, never joints, so there is no
    // cycle between the two requests.
    auto newSolverFrameTaps = std::make_unique<RigExecTapSet>(_stage);
    for (const auto &[jointPath, binding] : newJointBinding) {
        const SdfPath &solverPath = binding.first;
        if (newJointSolverArrayTaps.count(solverPath)) {
            continue;
        }
        newJointSolverArrayTaps[solverPath] = newSolverFrameTaps->Add(
            RigExecValueAddress::Prim(solverPath, _computePointFrameArray));
    }
    if (!newJointSolverArrayTaps.empty() && !newSolverFrameTaps->Prepare()) {
        reportError("failed to build a valid prepared request for the "
                    "solver frame aggregates");
        restorePreviousEpoch();
        return false;
    }

    // Pose-domain frame revisions (aim constraints), in memory. These used to
    // become generated RigExecPointFrameMoverApplication prims whose chain
    // head the taps below resolved to; now the revision is applied
    // evaluator-side from the mover's own authored inputs, so nothing needs
    // to be authored to express it.
    std::map<SdfPath, std::vector<_FrameRevision>> newFrameChains;
    std::map<SdfPath, RigExecTapId> newProviderRestFrameTaps;
    std::map<SdfPath, RigExecTapId> newProviderBaseFrameTaps;
    for (const RigExecMoverRecord &mover : newMovers) {
        if (mover.schemaType != "RigExecAimConstraint") {
            continue;
        }
        const UsdPrim moverPrim = _stage->GetPrimAtPath(mover.moverPath);
        if (!moverPrim) {
            continue;
        }
        SdfPathVector aims;
        if (const UsdRelationship rel =
                moverPrim.GetRelationship(TfToken("rigExec:aimTarget"))) {
            rel.GetTargets(&aims);
        }
        for (const SdfPath &target : mover.targets) {
            if (!target.IsPrimPath()) {
                continue;
            }
            _FrameRevision revision;
            revision.moverPath = mover.moverPath;
            if (!aims.empty()) {
                revision.aimTargetFrameTap = newTaps->Add(
                    RigExecValueAddress::Prim(aims[0].GetPrimPath(),
                                              _computePointFrame, basePhase));
            }
            newFrameChains[target].push_back(revision);
        }
    }
    // A provider carrying aim revisions that is not a joint needs a base
    // frame from somewhere. A Control has computePointFrame like a joint; a
    // plain UsdGeomXformable has no exec computation at all, so its frame
    // comes from its own USD transform and its revised matrix is published
    // back onto the prim for Hydra to inherit.
    std::set<SdfPath> newXformDerivedProviders;
    for (const auto &[provider, revisions] : newFrameChains) {
        if (std::find(newJointPaths.begin(), newJointPaths.end(), provider) !=
            newJointPaths.end()) {
            continue;  // joints are tapped below
        }
        const UsdPrim providerPrim = _stage->GetPrimAtPath(provider);
        const TfToken type = providerPrim ? providerPrim.GetTypeName()
                                          : TfToken();
        if (type == "RigExecControl" || type == "RigExecJoint") {
            newProviderBaseFrameTaps[provider] =
                newTaps->Add(RigExecValueAddress::Prim(
                    provider, _computePointFrame, basePhase));
        } else if (providerPrim && UsdGeomXformable(providerPrim)) {
            newXformDerivedProviders.insert(provider);
        } else {
            reportError(
                "aim constraint target " + provider.GetString() +
                " is neither a RigExec transform provider nor a "
                "UsdGeomXformable; nothing can carry the revised frame");
                restorePreviousEpoch();
            return false;
        }
    }

    // Rest frames, only for providers that actually publish one. A plain
    // UsdGeomXformable has no computeRestFrame -- requesting it is a hard exec
    // failure, not a missing value -- and needs none: its rest is identity,
    // which is what the published revision is measured against.
    for (const auto &[provider, revisions] : newFrameChains) {
        if (newXformDerivedProviders.count(provider)) {
            continue;
        }
        newProviderRestFrameTaps[provider] =
            newTaps->Add(RigExecValueAddress::Prim(
                provider, TfToken("computeRestFrame")));
    }

    for (const SdfPath &jointPath : newJointPaths) {
        newJointFrameTaps.push_back(newTaps->Add(RigExecValueAddress::Prim(
            jointPath, _computePointFrame, basePhase)));
        // The final-phase taps no longer resolve to a generated prim. When a
        // joint carries aim revisions its final frame is computed in memory
        // and overwrites these; when it does not, final IS base and these
        // resolve to the joint itself, which is what the empty resolution
        // already meant.
        newJointFinalFrameTaps.push_back(newTaps->Add(
            RigExecValueAddress::Prim(
                jointPath, _computePointFrame, finalPhase)));
        newJointFinalMatrixTaps.push_back(newTaps->Add(
            RigExecValueAddress::Prim(
                jointPath, TfToken("computeMatrix"), finalPhase)));
    }

    // Control frames, base phase only: a control is an input, so nothing in
    // the pose domain revises it and its base frame IS its posed frame.
    // Same request as the joints -- a control that cannot produce a frame
    // means the animator's own channel failed to evaluate, which is not a
    // condition to publish a generation under.
    std::vector<RigExecTapId> newControlFrameTaps;
    for (const SdfPath &controlPath : newControlPaths) {
        newControlFrameTaps.push_back(newTaps->Add(RigExecValueAddress::Prim(
            controlPath, _computePointFrame, basePhase)));
    }

    // The compiled mover graph, built from the same composed post-order walk
    // the generated prims come from. It runs alongside them for now: Evaluate
    // compares the two and diagnoses any disagreement, so the graph can be
    // proven equal before it becomes what publishes (see
    // docs/mover-graph-cutover.md). Bindings resolve off the authored stage
    // and author nothing.
    std::map<SdfPath, std::vector<_GraphRevision>> newGraphChains;
    for (const RigExecMoverRecord &mover : newMovers) {
        const UsdPrim moverPrim = _stage->GetPrimAtPath(mover.moverPath);
        if (!moverPrim) {
            continue;
        }
        TfToken curveMode;
        if (const UsdAttribute a =
                moverPrim.GetAttribute(TfToken("rigExec:mode"))) {
            a.Get(&curveMode);
        }
        const std::optional<RigExecRevisionOp> op =
            RigExecRevisionOpForSchema(mover.schemaType, curveMode);
        if (!op) {
            continue;
        }
        for (const SdfPath &target : mover.targets) {
            if (!target.IsPropertyPath() ||
                target.GetNameToken() != "points") {
                continue;
            }
            _GraphRevision revision;
            revision.moverPath = mover.moverPath;
            revision.target = target;
            revision.op = *op;
            revision.binding =
                RigExecResolveRevisionBinding(moverPrim, target, {});
            TfToken phase("base");
            if (const UsdAttribute a = moverPrim.GetAttribute(
                    TfToken("rigExec:transformReadPhase"))) {
                a.Get(&phase);
            }
            revision.transformFinalPhase = phase == "final";
            if (!revision.binding.transform.IsEmpty()) {
                revision.transformTap = newTaps->Add(
                    RigExecValueAddress::Prim(revision.binding.transform,
                                              TfToken("computeMatrix")));
            }
            if (!revision.binding.weightObject.IsEmpty()) {
                revision.weightTap = newTaps->Add(RigExecValueAddress::Prim(
                    revision.binding.weightObject,
                    TfToken("computeWeightPacket")));
                // Volumetric weights need two things exec cannot supply
                // on its own: a baked falloff table (no spline accessor
                // exists -- see RigExecFalloffLut) and, for the CPU
                // oracle, their resolved placement. Both are gathered
                // once here, following composition into combines.
                registerVolumeWeights(revision.binding.weightObject);
            }
            if (!revision.binding.driverFrames.IsEmpty()) {
                revision.driverFramesTap =
                    newTaps->Add(RigExecValueAddress::Prim(
                        revision.binding.driverFrames,
                        _computePointFrameArray));
            }
            // binding.blendInputs is already in canonical sorted order, which
            // is the order the accumulation is defined in.
            for (const SdfPath &input : revision.binding.blendInputs) {
                revision.blendChannelTaps.push_back(
                    newTaps->Add(RigExecValueAddress::Prim(
                        input, TfToken("computeBlendChannel"))));
            }
            newGraphChains[target].push_back(revision);
        }
    }

    // Derived maintenance (spec §7.6 revised), mirroring Pass 3: for every
    // moved points target whose gprim authors normals or extent, synthesize
    // the recompute revision. There is no authored mover, so the gprim itself
    // stands in as the parameter source -- it supplies the stage for the
    // static topology reads and has no inputs:enabled, so the revision is
    // enabled. Vertex-normal recomputation is mesh-only; the compiler already
    // rejects authored normals on a non-mesh points target.
    std::map<SdfPath, std::vector<_GraphRevision>> newGraphDerivedChains;
    for (const auto &[pointsTarget, revisions] : newGraphChains) {
        const SdfPath ownerPath = pointsTarget.GetPrimPath();
        const UsdPrim owner = _stage->GetPrimAtPath(ownerPath);
        if (!owner) {
            continue;
        }
        for (const bool isNormals : {true, false}) {
            const TfToken property(isNormals ? "normals" : "extent");
            const SdfPath derivedTarget =
                ownerPath.AppendProperty(property);
            const UsdAttribute authored =
                _stage->GetAttributeAtPath(derivedTarget);
            if (!authored || !authored.HasAuthoredValue()) {
                continue;
            }
            // Vertex-normal recomputation is mesh-only. Silently leaving
            // authored normals stale on a moved Points/BasisCurves target
            // would break the automatic-maintenance contract (spec §7.6
            // revised), so reject the configuration rather than skip it. This
            // check used to live in the compiler's Pass 3.
            if (isNormals && !UsdGeomMesh(owner)) {
                reportError(
                    "authored normals on non-mesh points target " +
                    ownerPath.GetString() +
                    " cannot be maintained (vertex-normal recomputation is "
                    "mesh-only, spec §7.6 revised); remove the authored "
                    "normals");
                return false;
            }
            _GraphRevision derived;
            derived.moverPath = ownerPath;
            derived.target = derivedTarget;
            derived.op = isNormals ? RigExecRevisionOp::RecomputeNormals
                                   : RigExecRevisionOp::RecomputeExtent;
            derived.binding.moverPath = ownerPath;
            derived.binding.target = derivedTarget;
            derived.binding.topologyCounts =
                ownerPath.AppendProperty(TfToken("faceVertexCounts"));
            derived.binding.topologyIndices =
                ownerPath.AppendProperty(TfToken("faceVertexIndices"));
            if (!isNormals) {
                // The authoritative winning widths, when authored: they
                // widen the extent bound (Pass 3 wires resolvedWidths).
                const SdfPath widthsPath =
                    ownerPath.AppendProperty(TfToken("widths"));
                if (const UsdAttribute w =
                        _stage->GetAttributeAtPath(widthsPath)) {
                    if (w.HasAuthoredValue()) {
                        derived.binding.widths = widthsPath;
                    }
                }
            }
            newGraphDerivedChains[pointsTarget].push_back(derived);
        }
    }

    if (!newTaps->Prepare()) {
        reportError("failed to build a valid prepared request for the "
                    "new epoch");
        restorePreviousEpoch();
        return false;
    }

    // Observational solver-guide taps prepare separately so a failing or
    // unused aggregate solver never gates the authoritative rig request;
    // preparation failure simply drops solver guide drawing.
    auto newGuideTaps = std::make_unique<RigExecTapSet>(_stage);
    for (const SdfPath &solverPath : solverArrayPaths) {
        newSolverArrayTaps[solverPath] = newGuideTaps->Add(
            RigExecValueAddress::Prim(solverPath, _computePointFrameArray));
    }
    if (newSolverArrayTaps.empty() || !newGuideTaps->Prepare()) {
        newGuideTaps.reset();
        newSolverArrayTaps.clear();
    }

    // Commit the new epoch atomically with respect to evaluator state.
    _movers = std::move(newMovers);
    _jointPaths = std::move(newJointPaths);
    _controlPaths = std::move(newControlPaths);
    _controlFrameTaps = std::move(newControlFrameTaps);
    _frameChains = std::move(newFrameChains);
    _providerRestFrameTaps = std::move(newProviderRestFrameTaps);
    _providerBaseFrameTaps = std::move(newProviderBaseFrameTaps);
    _xformDerivedProviders = std::move(newXformDerivedProviders);
    _ribbonDriverPoints = std::move(newRibbonDriverPoints);
    if (!volumeWeightError.empty()) {
        reportError(volumeWeightError);
        restorePreviousEpoch();
        return false;
    }

    _falloffLutOverrides = std::move(newFalloffLutOverrides);
    _currentPhaseWeights = std::move(newCurrentPhaseWeights);
    _volumeWeightMatrixTaps = std::move(newVolumeWeightMatrixTaps);
    _volumeWeightMatrices.clear();
    _structureDigest = newDigest;
    _taps = std::move(newTaps);
    _guideTaps = std::move(newGuideTaps);
    _jointFrameTaps = std::move(newJointFrameTaps);
    _jointFinalFrameTaps = std::move(newJointFinalFrameTaps);
    _jointFinalMatrixTaps = std::move(newJointFinalMatrixTaps);
    _jointSolverBinding = std::move(newJointBinding);
    _jointSolverArrayTaps = std::move(newJointSolverArrayTaps);
    _solverFrameTaps = std::move(newSolverFrameTaps);
    _solverArrayTaps = std::move(newSolverArrayTaps);
    _graphChains = std::move(newGraphChains);
    _graphDerivedChains = std::move(newGraphDerivedChains);
    _compiled = true;
    return true;
}

bool
RigExecRigEvaluator::_ValidateMatrixMover(
    const UsdPrim &prim,
    const RigExecMoverRecord &record,
    std::string *error) const
{
    const std::string who = "MatrixMover " + prim.GetPath().GetString();
    if (record.targets.size() != 1 ||
        !record.targets[0].IsPropertyPath() ||
        record.targets[0].GetNameToken() != "points") {
        *error = who + ": moves must resolve to exactly one native "
                       "PointBased points property";
        return false;
    }
    const UsdPrim owner =
        _stage->GetPrimAtPath(record.targets[0].GetPrimPath());
    if (!owner || !owner.IsA<UsdGeomPointBased>()) {
        *error = who + ": move target owner is not a stock PointBased prim";
        return false;
    }
    // Exact Sdf type/role check: equal C++ element types never infer
    // compatibility (spec §7.2).
    const UsdAttribute targetAttr =
        _stage->GetAttributeAtPath(record.targets[0]);
    if (!targetAttr ||
        targetAttr.GetTypeName() != SdfValueTypeNames->Point3fArray) {
        *error = who + ": move target is not an exact point3f[] property";
        return false;
    }

    SdfPathVector transforms, weightObjects;
    if (UsdRelationship rel =
            prim.GetRelationship(TfToken("rigExec:transform"))) {
        rel.GetTargets(&transforms);
    }
    if (UsdRelationship rel =
            prim.GetRelationship(TfToken("rigExec:weightObject"))) {
        rel.GetTargets(&weightObjects);
    }
    if (transforms.size() != 1) {
        *error = who + ": rigExec:transform must have exactly one target";
        return false;
    }
    // preceding is legal only for a dependency specialized to one
    // consuming application ordinal (spec §4.2); the v0.1 compiler
    // supports base and acyclic final.
    TfToken phase("base");
    if (UsdAttribute a =
            prim.GetAttribute(TfToken("rigExec:transformReadPhase"))) {
        a.Get(&phase);
    }
    if (phase != "base" && phase != "final") {
        *error = who + ": unsupported transformReadPhase '" +
                 phase.GetString() + "' (v0.1 supports base and final)";
        return false;
    }
    // The transform target must be a catalogued computeMatrix provider.
    const UsdPrim transformPrim = _stage->GetPrimAtPath(transforms[0]);
    static const std::set<TfToken> frameProviderTypes = {
        TfToken("RigExecControl"), TfToken("RigExecJoint")};
    // The applied-API arm is gone with RigExecPointTransformAPI: that was the
    // pre-alignment landmark transform model, superseded by RigExecXformable
    // (matrix rest/posed spaces plus avars) and applied by nothing.
    const bool isProvider =
        transformPrim && frameProviderTypes.count(transformPrim.GetTypeName());
    if (!isProvider) {
        *error = who + ": rigExec:transform target is not a catalogued "
                       "matrix provider";
        return false;
    }
    if (weightObjects.size() != 1) {
        *error = who + ": rigExec:weightObject must have exactly one target";
        return false;
    }
    const UsdPrim weightPrim = _stage->GetPrimAtPath(weightObjects[0]);
    if (!weightPrim) {
        *error = who + ": missing weight object " +
                 weightObjects[0].GetString();
        return false;
    }
    // The weight object's canonical target must match the mover's exact
    // points target (spec §4.2, §7.4).
    SdfPathVector weightTargets;
    if (UsdRelationship rel =
            weightPrim.GetRelationship(TfToken("rigExec:weightTarget"))) {
        rel.GetTargets(&weightTargets);
    }
    if (weightTargets.size() != 1 ||
        _CanonicalizeTarget(_stage, weightTargets[0]) != record.targets[0]) {
        *error = who + ": weight object target does not canonicalize to "
                       "the mover's points target";
        return false;
    }
    return true;
}

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
    const SdfPath canonical = _CanonicalizeTarget(_stage, targets[0]);
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
    const std::string who = prim.GetPath().GetString();
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
            *error = who + ": unknown rigExec:combineMode " +
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
            *error = who + ": combine inputs disagree on element count";
            return false;
        }
        float strength = 1.0f, invert = 0.0f;
        if (UsdAttribute a = prim.GetAttribute(TfToken("inputs:strength"))) {
            a.Get(&strength, time);
        }
        if (UsdAttribute a = prim.GetAttribute(TfToken("inputs:invert"))) {
            a.Get(&invert, time);
        }
        for (float &w : *weights) {
            w = (w + (1.0f - 2.0f * w) * invert) * strength;
        }
        return true;
    }

    // Placement.
    //
    // Taken from the volume's own exec computeMatrix rather than
    // recomputed here. The oracle exists to check the WEIGHT FIELD math
    // independently, not the xformable frame chain -- that already has
    // its own parity coverage, and a second hand-rolled implementation
    // of posed:space + rest offsets + avars + rotation order is exactly
    // the drift frameExtraction.h was created to prevent.
    const auto matrixIt = _volumeWeightMatrices.find(prim.GetPath());
    if (matrixIt == _volumeWeightMatrices.end()) {
        *error = who + ": no resolved placement for this volume weight";
        return false;
    }
    // Scale and shear are removed so the field matches the rigid guide a
    // viewer draws; inputs:scaleX/Y/Z is the sole authority on
    // anisotropy (see the RigExecVolumeWeight schema doc).
    GfMatrix4d rigid = matrixIt->second.RemoveScaleShear();
    const double det = rigid.GetDeterminant();
    if (!std::isfinite(det) || std::abs(det) < 1e-12) {
        *error = who + ": degenerate volume placement";
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
            *error = who +
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
            *error = who + ": could not read the points to sample";
            return false;
        }
    } else {
        *error = who + ": unknown rigExec:samplePhase " +
                 samplePhase.GetString();
        return false;
    }
    if (samplePoints.size() != count) {
        *error = who + ": sampled point count does not match the target";
        return false;
    }

    RigExecFalloffParams params;
    auto readFloat = [&prim, time](const char *name, float fallback) {
        float v = fallback;
        if (UsdAttribute a = prim.GetAttribute(TfToken(name))) {
            a.Get(&v, time);
        }
        return v;
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
            *error = who + ": unknown rigExec:planeAxis " + axis.GetString();
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
                    *error = who +
                             ": inputs:extentU/V must be finite and positive "
                             "when rigExec:planeBounds is `bounded`";
                    return false;
                }
            }
            extentPtr = &extent;
        } else if (boundsMode != "unbounded") {
            *error = who + ": unknown rigExec:planeBounds " +
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
            *error = who + ": inputs:scaleX/Y/Z must be finite and positive";
            return false;
        }
    }
    GfMatrix4d divide(1.0);
    divide.SetScale(GfVec3d(1.0 / double(sx), 1.0 / double(sy),
                            1.0 / double(sz)));
    worldToLocal = worldToLocal * divide;

    if (typeName == "RigExecSphereWeight") {
        RigExecSphereWeightField(samplePoints, worldToLocal, params, weights);
        return true;
    }
    if (typeName == "RigExecCurveWeight") {
        std::vector<GfVec3f> curvePoints;
        if (!_ReadTargetPoints(prim, "rigExec:curve", time, &curvePoints) ||
            curvePoints.empty()) {
            *error = who + ": rigExec:curve must name exactly one points source";
            return false;
        }
        RigExecCurveWeightField(
            samplePoints, curvePoints, worldToLocal, params, weights);
        return true;
    }
    *error = who + ": not a volumetric weight object";
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
    float defaultWeight = 0.0f;
    if (UsdAttribute a = prim.GetAttribute(TfToken("rigExec:defaultWeight"))) {
        a.Get(&defaultWeight, time);
    }
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
                return t.size() == 1 ? _CanonicalizeTarget(_stage, t[0])
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
        float driver = 1, scale = 1, bias = 0;
        if (UsdAttribute a = prim.GetAttribute(TfToken("inputs:driver"))) {
            a.Get(&driver, time);
        }
        if (UsdAttribute a = prim.GetAttribute(TfToken("inputs:scale"))) {
            a.Get(&scale, time);
        }
        if (UsdAttribute a = prim.GetAttribute(TfToken("inputs:bias"))) {
            a.Get(&bias, time);
        }
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

namespace {

bool
_IsEnabled(const UsdPrim &mover, UsdTimeCode time)
{
    bool enabled = true;
    if (UsdAttribute a = mover.GetAttribute(_enabledAttr)) {
        a.Get(&enabled, time);
    }
    return enabled;
}

}  // namespace

VtVec3fArray
RigExecRigEvaluator::_EvaluateChain(
    const SdfPath &target,
    const std::vector<const RigExecMoverRecord *> &chain,
    const RigExecRigPose &pose,
    UsdTimeCode time,
    std::vector<std::string> *diagnostics) const
{
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
        const std::string type = mover->schemaType.GetString();

        if (type == "RigExecBlendShapeMover") {
            // p'_i = p_i + sum_k alpha_k(w_k) d_{k,i} (spec §7.3).
            SdfPathVector inputs;
            if (UsdRelationship rel =
                    prim.GetRelationship(TfToken("rigExec:blendInputs"))) {
                rel.GetTargets(&inputs);
            }
            std::vector<float> mask(points.size(), 1.0f);
            SdfPathVector weightObj;
            if (UsdRelationship rel =
                    prim.GetRelationship(TfToken("rigExec:weightObject"))) {
                rel.GetTargets(&weightObj);
            }
            std::string error;
            // `points` IS the in-flight buffer here, so a current-phase
            // volume measures the same thing the graph measures (see the
            // graph build loop in Evaluate). The copy is taken only when
            // one is actually authored.
            std::vector<GfVec3f> inFlight;
            const std::vector<GfVec3f> *currentPoints = nullptr;
            if (!weightObj.empty() && _currentPhaseWeights.count(weightObj[0])) {
                inFlight.assign(points.begin(), points.end());
                currentPoints = &inFlight;
            }
            if (!weightObj.empty() &&
                !_ResolveWeights(weightObj[0], points.size(), time, &mask,
                                 &error, currentPoints)) {
                diagnostics->push_back(
                    "MoverFailed " + mover->moverPath.GetString() + ": " +
                    error);
                continue;  // atomic failure returns preceding revision
            }
            // Active inputs accumulate in canonical input-path order
            // (spec §7.3); authored relationship order is non-semantic.
            std::sort(inputs.begin(), inputs.end());

            VtVec3fArray next = points;
            bool failed = false;
            for (const SdfPath &inputPath : inputs) {
                const UsdPrim input = _stage->GetPrimAtPath(inputPath);
                if (!input) {
                    diagnostics->push_back(
                        "MoverFailed " + mover->moverPath.GetString() +
                        ": missing blend input " + inputPath.GetString());
                    failed = true;
                    break;
                }
                float channel = 0;
                if (UsdAttribute a =
                        input.GetAttribute(TfToken("inputs:weight"))) {
                    a.Get(&channel, time);
                }
                if (!std::isfinite(channel)) {
                    diagnostics->push_back(
                        "MoverFailed " + mover->moverPath.GetString() +
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
                        "MoverFailed " + mover->moverPath.GetString() +
                        ": blend input has no samples: " +
                        inputPath.GetString());
                    failed = true;
                    break;
                }
                for (const SdfPath &samplePath : samplePaths) {
                    const UsdPrim sample =
                        _stage->GetPrimAtPath(samplePath.GetPrimPath());
                    if (!sample) {
                        diagnostics->push_back(
                            "MoverFailed " + mover->moverPath.GetString() +
                            ": missing blend sample " +
                            samplePath.GetString());
                        failed = true;
                        break;
                    }
                    SdfPathVector shapeTargets;
                    if (UsdRelationship rel = sample.GetRelationship(
                            TfToken("rigExec:targetPoints"))) {
                        rel.GetTargets(&shapeTargets);
                    }
                    if (shapeTargets.size() != 1) {
                        diagnostics->push_back(
                            "MoverFailed " + mover->moverPath.GetString() +
                            ": blend sample must target exactly one native "
                            "points property: " + samplePath.GetString());
                        failed = true;
                        break;
                    }
                    _Sample s;
                    s.path = sample.GetPath();
                    const UsdAttribute shapeAttr =
                        _stage->GetAttributeAtPath(shapeTargets[0]);
                    if (!shapeAttr || !shapeAttr.Get(&s.shape, time) ||
                        s.shape.size() != basePoints.size()) {
                        diagnostics->push_back(
                            "MoverFailed " + mover->moverPath.GetString() +
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
                            "MoverFailed " + mover->moverPath.GetString() +
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
                            "MoverFailed " + mover->moverPath.GetString() +
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
                    next[i] += delta * mask[i];
                }
            }
            if (!failed) {
                points = next;
            }
        } else if (type == "RigExecMatrixMover") {
            // p' = q + w (T q - q) (spec §7.4).
            SdfPathVector transforms, weightObj;
            if (UsdRelationship rel =
                    prim.GetRelationship(TfToken("rigExec:transform"))) {
                rel.GetTargets(&transforms);
            }
            if (UsdRelationship rel =
                    prim.GetRelationship(TfToken("rigExec:weightObject"))) {
                rel.GetTargets(&weightObj);
            }
            if (transforms.size() != 1 || weightObj.size() != 1) {
                diagnostics->push_back(
                    "MoverFailed " + mover->moverPath.GetString() +
                    ": transform and weightObject must each have exactly "
                    "one target");
                continue;
            }
            const auto matrixIt = pose.jointMatricesFinal.find(transforms[0]);
            if (matrixIt == pose.jointMatricesFinal.end()) {
                diagnostics->push_back(
                    "MoverFailed " + mover->moverPath.GetString() +
                    ": no matrix provider at " + transforms[0].GetString());
                continue;
            }
            std::vector<float> weights;
            std::string error;
            // See the blend branch above: the in-flight buffer is copied
            // only for a volume that asked to measure against it.
            std::vector<GfVec3f> inFlight;
            const std::vector<GfVec3f> *currentPoints = nullptr;
            if (_currentPhaseWeights.count(weightObj[0])) {
                inFlight.assign(points.begin(), points.end());
                currentPoints = &inFlight;
            }
            if (!_ResolveWeights(weightObj[0], points.size(), time, &weights,
                                 &error, currentPoints)) {
                diagnostics->push_back(
                    "MoverFailed " + mover->moverPath.GetString() + ": " +
                    error);
                continue;
            }
            const GfMatrix4d &m = matrixIt->second;
            for (size_t i = 0; i < points.size(); ++i) {
                const GfVec3d moved = RigExecApplyWeightedMatrix(
                    GfVec3d(points[i]), m, weights[i]);
                points[i] = GfVec3f(moved);
            }
        } else if (type == "RigExecCurveMover") {
            TfToken mode("ribbon");
            if (UsdAttribute a = prim.GetAttribute(TfToken("rigExec:mode"))) {
                a.Get(&mode, time);
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
                        _stage->GetPrimAtPath(frameTargets[0])) {
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
            if (UsdAttribute a = _stage->GetAttributeAtPath(curvePoints)) {
                a.Get(&posedCvs, time);
                a.Get(&restCvs, UsdTimeCode::Default());
            }
            const auto posedSamples = RigExecSampleCurveRMF(
                std::vector<GfVec3f>(posedCvs.begin(), posedCvs.end()),
                sampleCount);
            const auto restSamples = RigExecSampleCurveRMF(
                std::vector<GfVec3f>(restCvs.begin(), restCvs.end()),
                sampleCount);
            if (mode == "emitGuidePoints") {
                if (posedSamples.GetSize() == points.size()) {
                    for (size_t i = 0; i < points.size(); ++i) {
                        points[i] = posedSamples.positions[i];
                    }
                } else {
                    diagnostics->push_back(
                        "MoverFailed " + mover->moverPath.GetString() +
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
                            _stage->GetAttributeAtPath(binds[0])) {
                        a.Get(&sts, time);
                    }
                }
                std::vector<GfVec3f> scratch(points.begin(), points.end());
                RigExecApplyRibbonTransport(
                    &scratch,
                    std::vector<GfVec2f>(sts.begin(), sts.end()),
                    restSamples, posedSamples);
                std::copy(scratch.begin(), scratch.end(), points.begin());
            }
        } else if (type == "RigExecVolumeCorrectMover" ||
                   type == "RigExecSmoothMover") {
            const bool isVolume = type == "RigExecVolumeCorrectMover";
            float strength = isVolume ? 0.0f : 0.5f;
            if (UsdAttribute a =
                    prim.GetAttribute(TfToken("inputs:strength"))) {
                a.Get(&strength, time);
            }
            std::vector<GfVec3f> scratch(points.begin(), points.end());
            if (isVolume) {
                VtVec3fArray base;
                if (UsdAttribute a = _stage->GetAttributeAtPath(target)) {
                    a.Get(&base, time);
                }
                const double reference = RigExecBoundVolume(
                    base.cdata(), base.size());
                RigExecApplyVolumeCorrect(&scratch, reference, strength);
            } else {
                const UsdPrim owner =
                    _stage->GetPrimAtPath(target.GetPrimPath());
                VtIntArray counts, indices;
                if (owner) {
                    owner.GetAttribute(TfToken("faceVertexCounts"))
                        .Get(&counts, time);
                    owner.GetAttribute(TfToken("faceVertexIndices"))
                        .Get(&indices, time);
                }
                RigExecApplyLaplacianSmooth(
                    &scratch,
                    std::vector<int>(counts.begin(), counts.end()),
                    std::vector<int>(indices.begin(), indices.end()),
                    strength);
            }
            std::copy(scratch.begin(), scratch.end(), points.begin());
        } else if (type == "RigExecLatticeMover") {
            // Independent of RigExecAssembleParameters on purpose: this is the
            // parity oracle, so it resolves its own inputs off the stage. A
            // reference that called the assembler would have agreed with the
            // SurfaceProject strength bug instead of catching it.
            //
            // The rest cage is the cage at Default time. That is exactly
            // what the deleted compiler captured into
            // rigExec:restCagePoints -- a Default-time read and nothing
            // more, which is why the authored capture was removable.
            SdfPathVector cages;
            if (UsdRelationship rel =
                    prim.GetRelationship(TfToken("rigExec:cage"))) {
                rel.GetTargets(&cages);
            }
            if (cages.empty()) {
                continue;
            }
            SdfPath cagePoints = cages[0];
            if (cagePoints.IsPrimPath()) {
                cagePoints = cagePoints.AppendProperty(TfToken("points"));
            }
            VtVec3fArray restCage, posedCage, base;
            if (UsdAttribute a = _stage->GetAttributeAtPath(cagePoints)) {
                a.Get(&restCage, UsdTimeCode::Default());
                a.Get(&posedCage, time);
            }
            if (UsdAttribute a = _stage->GetAttributeAtPath(target)) {
                a.Get(&base, time);
            }
            GfVec3i divisions(0);
            if (UsdAttribute a =
                    prim.GetAttribute(TfToken("rigExec:divisions"))) {
                a.Get(&divisions, time);
            }
            const size_t cageCount = size_t(divisions[0]) *
                                     size_t(divisions[1]) *
                                     size_t(divisions[2]);
            if (divisions[0] < 2 || divisions[1] < 2 || divisions[2] < 2 ||
                restCage.size() != cageCount ||
                posedCage.size() != cageCount || base.size() != points.size()) {
                diagnostics->push_back(
                    "MoverFailed " + mover->moverPath.GetString() +
                    ": lattice cage/divisions mismatch");
                continue;
            }
            std::vector<GfVec3f> scratch(points.begin(), points.end());
            RigExecApplyLattice(
                &scratch, std::vector<GfVec3f>(base.begin(), base.end()),
                std::vector<GfVec3f>(restCage.begin(), restCage.end()),
                std::vector<GfVec3f>(posedCage.begin(), posedCage.end()),
                divisions);
            std::copy(scratch.begin(), scratch.end(), points.begin());
        } else if (type == "RigExecSurfaceMover") {
            SdfPathVector surfaces;
            if (UsdRelationship rel =
                    prim.GetRelationship(TfToken("rigExec:surface"))) {
                rel.GetTargets(&surfaces);
            }
            if (surfaces.empty()) {
                continue;
            }
            const SdfPath surfacePrim = surfaces[0].GetPrimPath();
            VtVec3fArray surfacePoints;
            VtIntArray counts, indices;
            if (UsdAttribute a = _stage->GetAttributeAtPath(
                    surfacePrim.AppendProperty(TfToken("points")))) {
                a.Get(&surfacePoints, time);
            }
            if (const UsdPrim s = _stage->GetPrimAtPath(surfacePrim)) {
                s.GetAttribute(TfToken("faceVertexCounts")).Get(&counts, time);
                s.GetAttribute(TfToken("faceVertexIndices"))
                    .Get(&indices, time);
            }
            if (surfacePoints.empty() || counts.empty()) {
                diagnostics->push_back(
                    "MoverFailed " + mover->moverPath.GetString() +
                    ": surface has no points/topology");
                continue;
            }
            std::vector<GfVec3f> scratch(points.begin(), points.end());
            // v0.1 attach/project both map fully; the mode token selects no
            // numeric difference yet (_BuildSurfaceMoverParameters pins 1.0).
            RigExecApplySurfaceProject(
                &scratch,
                std::vector<GfVec3f>(surfacePoints.begin(),
                                     surfacePoints.end()),
                std::vector<int>(counts.begin(), counts.end()),
                std::vector<int>(indices.begin(), indices.end()), 1.0f);
            std::copy(scratch.begin(), scratch.end(), points.begin());
        }
    }
    return points;
}

RigExecRigPose
RigExecRigEvaluator::Evaluate(UsdTimeCode time)
{
    RigExecRigPose pose;
    pose.time = time;
    if (!_compiled && !Compile(nullptr)) {
        return pose;
    }
    // Structural edits begin a new epoch: recompile when the composed
    // mover topology digest changed (spec §4.2, §6.3).
    if (_ComputeStructureDigest() != _structureDigest) {
        if (!Compile(nullptr)) {
            pose.diagnostics.push_back(
                "structural recompilation failed");
            return pose;
        }
        pose.diagnostics.push_back("structural edit: epoch rebuilt");
    }

    // 0. Solver aggregates first, then each solver-posed joint's frame as an
    // override on the authoritative request. This is the whole of the
    // solver->joint binding: the element choice is compiled state, and exec
    // sees the result as if the joint had computed it, so computeMatrix,
    // frame-chain applications, and the NamespaceAncestor fallback that
    // unbound descendants follow all stay correct with nothing authored.
    // Ribbon driver-curve points, live and at bind time. Constant for a given
    // time, so they are built once and prefixed to every override vector --
    // including the guide taps, which evaluate the same ribbon solvers.
    // Bind-time values read at Default(), which is exactly what the deleted
    // compiler pass captured.
    std::vector<RigExecValueOverride> baseOverrides;
    for (const auto &[ribbonPath, pointsPath] : _ribbonDriverPoints) {
        const UsdAttribute a = _stage->GetAttributeAtPath(pointsPath);
        if (!a) {
            continue;
        }
        VtVec3fArray live, rest;
        a.Get(&live, time);
        a.Get(&rest, UsdTimeCode::Default());
        RigExecPointsPacket livePacket, restPacket;
        livePacket.points.assign(live.begin(), live.end());
        restPacket.points.assign(rest.begin(), rest.end());
        baseOverrides.push_back(RigExecValueOverride{
            ribbonPath, TfToken("rigExec:computeDriverPoints"), TfToken(),
            VtValue(livePacket)});
        baseOverrides.push_back(RigExecValueOverride{
            ribbonPath, TfToken("rigExec:computeRestDriverPoints"), TfToken(),
            VtValue(restPacket)});
    }

    std::vector<RigExecValueOverride> jointOverrides = baseOverrides;
    if (_solverFrameTaps && !_jointSolverBinding.empty()) {
        // Iterated to a fixed point, NOT computed once.
        //
        // Solvers are not downstream-only of controls: RigExecTwistDistribution
        // takes rigExec:start / rigExec:end as computePointFrame inputs, and
        // those endpoints are routinely joints that ANOTHER solver poses
        // (05_TwistRibbonSpine: SpineFK poses Root and Chest, which SpineTwist
        // reads, and SpineTwist in turn poses TwistMid). Evaluating the
        // aggregates once with no overrides would compute such a solver from
        // its endpoints' unposed namespace-parent fallback, and the joint it
        // poses would get an override derived from stale input.
        //
        // Graph/lowered parity cannot see this: both paths consume the same
        // override, so both are wrong together and agree perfectly.
        //
        // Each round feeds the previous round's overrides back in, so a solver
        // that depends on another solver's joints converges one level per
        // round. The binding graph is a DAG (a joint claimed by two solvers is
        // rejected at compile), so depth is bounded by the number of posing
        // solvers; the cap is a backstop, and stability is the real exit.
        const size_t maxRounds = _jointSolverArrayTaps.size() + 1;
        for (size_t round = 0; round < maxRounds; ++round) {
            const RigExecSnapshot solverSnapshot =
                _solverFrameTaps->Evaluate(time, jointOverrides);
            if (!solverSnapshot.IsValid() || !solverSnapshot.IsComplete()) {
                pose.diagnostics.push_back(
                    solverSnapshot.IsValid()
                        ? "solver aggregate snapshot incomplete: missing frames"
                        : "solver aggregate evaluation failed");
                return pose;
            }
            std::map<SdfPath, RigExecPointFrameArray> aggregates;
            for (const auto &[solverPath, tap] : _jointSolverArrayTaps) {
                aggregates[solverPath] =
                    solverSnapshot.Get<RigExecPointFrameArray>(tap);
            }

            // Seeded with the ribbon driver points, which are constant for
            // this time: the loop rebuilds this vector every round, so a bare
            // `next` would drop them after round 0 and the ribbon would
            // sample an empty curve.
            std::vector<RigExecValueOverride> next = baseOverrides;
            next.reserve(baseOverrides.size() + _jointSolverBinding.size());
            for (const auto &[jointPath, binding] : _jointSolverBinding) {
                const auto it = aggregates.find(binding.first);
                if (it == aggregates.end()) {
                    continue;
                }
                next.push_back(RigExecValueOverride{
                    jointPath, _computePointFrame, TfToken(),
                    VtValue(RigExecExtractElementFrame(
                        &it->second, static_cast<size_t>(binding.second)))});
            }

            bool changed = next.size() != jointOverrides.size();
            for (size_t i = 0; !changed && i < next.size(); ++i) {
                changed = next[i].prim != jointOverrides[i].prim ||
                          next[i].value != jointOverrides[i].value;
            }
            jointOverrides.swap(next);
            if (!changed) {
                // Rounds beyond the first that still moved a value are the
                // solver-depends-on-solver-posed-joint case; reporting the
                // count is what makes that dependency visible instead of
                // silently absorbed.
                pose.solverOverrideRounds = round;
                pose.diagnostics.push_back(
                    "solver->joint overrides converged after " +
                    std::to_string(round) + " refinement round(s)");
                break;  // fixed point
            }
            if (round + 1 == maxRounds) {
                // Refusing to publish, not publishing the last iterate.
                // A non-converged override set means some joint is posed
                // from a frame that is still moving, and every consumer
                // downstream -- geometry, matrices, imaging -- would take it
                // as settled. An unevaluated generation is recoverable; a
                // plausible wrong one is not (spec §6.6).
                pose.solverOverrideRounds = maxRounds;
                pose.solverOverridesConverged = false;
                pose.diagnostics.push_back(
                    "solver->joint overrides did not converge in " +
                    std::to_string(maxRounds) +
                    " rounds; refusing to publish this generation");
                return pose;  // pose.valid stays false
            }
        }
    }

    // Baked falloff tables ride along with the joint overrides. They are
    // epoch-constant, so this replays the same values Compile produced
    // until the next epoch -- exec has no accessor for an attribute's
    // spline, and a falloff curve is the whole function rather than one
    // resolved value (see RigExecFalloffLut in types.h).
    jointOverrides.insert(jointOverrides.end(), _falloffLutOverrides.begin(),
                          _falloffLutOverrides.end());

    // 1. Transforms and solvers through OpenExec. An incomplete snapshot
    // means some computation failed to compile or evaluate; refusing to
    // continue prevents default-constructed values from masquerading as
    // results (spec §6.6).
    const RigExecSnapshot snapshot = _taps->Evaluate(time, jointOverrides);
    if (!snapshot.IsValid() || !snapshot.IsComplete()) {
        pose.diagnostics.push_back(
            snapshot.IsValid() ? "snapshot incomplete: missing tap values"
                               : "snapshot evaluation failed");
        return pose;
    }

    // Resolved volume placements, for the CPU oracle and for any
    // `current`-phase field recomputed against the in-flight points.
    _volumeWeightMatrices.clear();
    for (const auto &[weightPath, tap] : _volumeWeightMatrixTaps) {
        // The tap is the ABSOLUTE posed frame; the placement is the map
        // taking the identity landmarks to it.
        const RigExecPointFrame posed = snapshot.Get<RigExecPointFrame>(tap);
        GfMatrix4d placement(1.0);
        if (posed.IsValid() && !posed.IsDegenerate()) {
            RigExecPointsToMatrix(
                RigExecIdentityLandmarks(), posed.points, &placement);
        }
        _volumeWeightMatrices[weightPath] = placement;
    }
    // Published as-is: the imaging bridge draws each volume's falloff
    // iso-surfaces in exactly the space its field was measured in.
    pose.weightFrames = _volumeWeightMatrices;
    // 2. Pose-domain frame revisions (aim constraints), applied in memory.
    //
    // These used to be generated RigExecPointFrameMoverApplication prims that
    // the final-phase taps resolved to. The revision is two lines of math over
    // the preceding frame, and every input is authored on the mover itself, so
    // the generated prim was carrying only the wiring that named those inputs
    // -- wiring the evaluator can resolve directly.
    std::map<SdfPath, RigExecPointFrame> finalFrames;
    std::map<SdfPath, GfMatrix4d> finalMatrices;
    for (const auto &[provider, revisions] : _frameChains) {
        RigExecPointFrame frame;
        const bool xformDerived = _xformDerivedProviders.count(provider) != 0;
        const auto baseTapIt = _providerBaseFrameTaps.find(provider);
        // Kept for the publication below: the revision is only meaningful to a
        // downstream consumer alongside the matrix it revised.
        GfMatrix4d xformDerivedBase(1.0);
        bool xformDerivedValid = false;
        if (xformDerived) {
            // The prim's own composed transform IS its base frame, expressed
            // RELATIVE TO THE ASSET ROOT -- not local-to-parent, and not
            // local-to-world either.
            //
            // The constraint solves this frame against an aim target frame,
            // and the solver subtracts the two origins directly, so both must
            // live in the same space. Joint and control frames come from
            // authored rest:space composed with avars (_JointRestSpace in
            // computations.cpp) and never acquire the asset's placement on
            // the stage: they are asset-common space. So:
            //
            //   local-to-parent  -- wrong. Omits every transform between the
            //     provider and the asset root. With Geom at (6,1,-3) and the
            //     provider at local (0,2,0), the aim is computed from (0,2,0)
            //     instead of (6,3,-3).
            //   local-to-world   -- also wrong, in the other direction. Adds
            //     the asset's own placement to one side of the subtraction
            //     only. Place the asset at x=+100 and the turret aims at -X
            //     while the target is physically at +X.
            //
            // Identity ancestors collapse all three, which is how the first
            // error hid and how the over-correction hid after it.
            const UsdPrim providerPrim = _stage->GetPrimAtPath(provider);
            const UsdPrim assetRoot =
                _stage->GetPrimAtPath(_rigPath.GetParentPath());
            const UsdGeomXformable xformable(providerPrim);
            if (xformable && assetRoot) {
                UsdGeomXformCache cache(time);
                // The resetXformStack out-param is documented as required to
                // be valid -- it is not optional and must not be null.
                bool resetsBelowAsset = false;
                xformDerivedBase = cache.ComputeRelativeTransform(
                    providerPrim, assetRoot, &resetsBelowAsset);
                xformDerivedValid = true;
            }
            // A provider we cannot read is skipped below rather than
            // published as an identity-derived revision.
            frame = RigExecFrameFromMatrix(xformDerivedBase);
        } else if (baseTapIt != _providerBaseFrameTaps.end()) {
            frame = snapshot.Get<RigExecPointFrame>(baseTapIt->second);
        } else {
            const auto jointIt = std::find(_jointPaths.begin(),
                                           _jointPaths.end(), provider);
            if (jointIt == _jointPaths.end()) {
                continue;
            }
            frame = snapshot.Get<RigExecPointFrame>(
                _jointFrameTaps[jointIt - _jointPaths.begin()]);
        }
        if (!frame.IsValid()) {
            continue;
        }

        for (const _FrameRevision &revision : revisions) {
            const UsdPrim moverPrim =
                _stage->GetPrimAtPath(revision.moverPath);
            if (!moverPrim) {
                continue;
            }
            bool enabled = true;
            if (const UsdAttribute a =
                    moverPrim.GetAttribute(TfToken("inputs:enabled"))) {
                a.Get(&enabled, time);
            }
            if (!enabled) {
                continue;  // ordinary pass-through (spec §6.6)
            }
            if (revision.aimTargetFrameTap < 0) {
                continue;
            }
            const RigExecPointFrame aimTarget =
                snapshot.Get<RigExecPointFrame>(revision.aimTargetFrameTap);
            if (!aimTarget.IsValid()) {
                continue;  // an unresolvable aim leaves the frame unrevised
            }
            float weight = 1.0f;
            if (const UsdAttribute a =
                    moverPrim.GetAttribute(TfToken("inputs:weight"))) {
                a.Get(&weight, time);
            }
            TfToken aimAxis("x");
            if (const UsdAttribute a =
                    moverPrim.GetAttribute(TfToken("rigExec:aimAxis"))) {
                a.Get(&aimAxis);
            }
            const int aimIdx =
                static_cast<int>(RigExecParseAxis(aimAxis, RigExecAxis::X)) + 1;
            frame = RigExecApplyAimConstraint(
                frame, aimTarget.Origin(), weight, aimIdx);
        }
        finalFrames[provider] = frame;

        // An xform-derived provider publishes its revised transform back onto
        // the prim, so geometry parented underneath rides along -- one matrix,
        // no point deformation. Base and revision are published together;
        // consumers downstream of Hydra's flatten need both to build the
        // world-space delta (see RigExecRigPose::providerBaseXforms).
        if (xformDerived && xformDerivedValid) {
            GfMatrix4d revised(1.0);
            const std::array<GfVec3d, 4> &identity =
                RigExecIdentityLandmarks();
            if (RigExecPointsToMatrix(identity, frame.points, &revised)) {
                pose.providerXforms[provider] = revised;
                pose.providerBaseXforms[provider] = xformDerivedBase;
            }
        }

        // The paired matrix is the provider's rest->final map, exactly as
        // _EvaluateFrameApplicationMatrix computed it.
        const auto restIt = _providerRestFrameTaps.find(provider);
        if (restIt != _providerRestFrameTaps.end() && frame.IsValid()) {
            const RigExecPointFrame rest =
                snapshot.Get<RigExecPointFrame>(restIt->second);
            GfMatrix4d m(1.0);
            if (rest.IsValid() &&
                RigExecPointsToMatrix(rest.points, frame.points, &m)) {
                finalMatrices[provider] = m;
            }
        }
    }

    // 3. Base and final transform revisions plus paired matrices. A
    // solver-posed joint's base value was supplied as an override above, so
    // exec published it as that joint's computePointFrame; the final value is
    // the in-memory frame revision when the joint carries one, and otherwise
    // is the base (which is what the final-phase tap already resolves to).
    for (size_t i = 0; i < _jointPaths.size(); ++i) {
        const RigExecPointFrame baseFrame =
            snapshot.Get<RigExecPointFrame>(_jointFrameTaps[i]);
        const auto revisedIt = finalFrames.find(_jointPaths[i]);
        const RigExecPointFrame finalFrame =
            revisedIt != finalFrames.end()
                ? revisedIt->second
                : snapshot.Get<RigExecPointFrame>(_jointFinalFrameTaps[i]);
        pose.jointFramesBase[_jointPaths[i]] = baseFrame;
        pose.jointFramesFinal[_jointPaths[i]] = finalFrame;
        // The point frame is the status bearer; the matrix result carries
        // no status and _ComputeJointMatrix returns identity for a
        // degenerate/invalid frame. Publishing that identity would let a
        // matrix-only consumer deform with a plausible-but-wrong transform
        // (codex round-2). Omit the matrix and diagnose so absence — not a
        // false identity — signals the failure; consumers already handle a
        // missing jointMatricesFinal entry. The degenerate frame is still
        // published so imaging can omit its guide.
        if (finalFrame.IsValid() && !finalFrame.IsDegenerate()) {
            const auto revisedMatrix = finalMatrices.find(_jointPaths[i]);
            pose.jointMatricesFinal[_jointPaths[i]] =
                revisedMatrix != finalMatrices.end()
                    ? revisedMatrix->second
                    : snapshot.Get<GfMatrix4d>(_jointFinalMatrixTaps[i]);
        } else {
            pose.diagnostics.push_back(
                "joint " + _jointPaths[i].GetString() +
                " has a degenerate final frame; matrix omitted");
        }
    }
    // 3a. Control frames, straight from the base taps. A degenerate or
    // invalid frame is published as it stands, exactly like a joint's: the
    // imaging bridge is what decides a guide cannot be drawn from it, and it
    // already makes that judgement for every joint frame it sees.
    for (size_t i = 0; i < _controlPaths.size(); ++i) {
        pose.controlFrames[_controlPaths[i]] =
            snapshot.Get<RigExecPointFrame>(_controlFrameTaps[i]);
    }

    // Observational solver guides never gate the rig snapshot: an
    // incomplete guide evaluation degrades to a diagnostic.
    if (_guideTaps) {
        const RigExecSnapshot guideSnapshot =
            _guideTaps->Evaluate(time, baseOverrides);
        if (guideSnapshot.IsComplete()) {
            for (const auto &[solverPath, tap] : _solverArrayTaps) {
                pose.solverFrames[solverPath] =
                    guideSnapshot.Get<RigExecPointFrameArray>(tap).frames;
            }
        } else {
            pose.diagnostics.push_back(
                "solver guide taps incomplete: guides omitted this "
                "generation");
        }
    }

    for (const RigExecMoverRecord &mover : _movers) {
        if (mover.schemaType == "RigExecFloatMathMover") {
            // Property movers over solver inputs are lowered into the
            // consuming computation in v0.1-alpha (the blend clamps its
            // weight); recorded for the trace.
            pose.diagnostics.push_back(
                "diag " + mover.moverPath.GetString() +
                ": float property mover lowered into consumer");
        }
    }

    // 3. Geometry point chains from the generated applications: the chain
    // head's passive outputs:value bridge extracts the exact native
    // VtVec3fArray (spec §7.2). Unsupported v0.1-alpha operations were
    // skipped by the compiler and pass through with diagnostics.
    // 3b. The compiled mover graph, which is the ONLY producer of geometry
    // (spec §7.2): no generated prim and no derived-stage read stands between
    // the authored mover chain and the value written to movedProperties.
    //
    // Every op reachable in a point chain has its provider values: matrix
    // (computeMatrix + computeWeightPacket), blendShape (summed
    // computeBlendChannel), ribbon / emitGuidePoints
    // (computePointFrameArray), and volumeCorrect / smooth / lattice /
    // surfaceProject, whose inputs are static reads through the binding plus
    // the authored base.
    //
    // Correctness is policed by the scalar CPU reference below, which resolves
    // its own inputs off the authored stage and authors nothing. A
    // disagreement is counted on the pose, never silently substituted:
    // grepping a diagnostic string is what let a packet-assembly drift reach
    // usdview once already (see docs/mover-graph-cutover.md).
    size_t graphChainsChecked = 0;
    size_t graphRevisionsBuilt = 0;
    for (const auto &[target, revisions] : _graphChains) {
        VtVec3fArray basePoints;
        const UsdAttribute baseAttr = _stage->GetAttributeAtPath(target);
        if (!baseAttr || !baseAttr.Get(&basePoints, time)) {
            continue;
        }

        RigExecMoverGraph graph;
        VdfMaskedOutput head = graph.AddPointSource(target, basePoints);
        bool built = true;
        for (const _GraphRevision &revision : revisions) {
            const UsdPrim moverPrim =
                _stage->GetPrimAtPath(revision.moverPath);
            if (!moverPrim) {
                built = false;
                break;
            }
            RigExecProviderValues values;
            GfMatrix4d transform(1.0);
            RigExecWeightPacket weights;
            RigExecPointFrameArray driverFrames;
            if (revision.driverFramesTap >= 0) {
                driverFrames = snapshot.Get<RigExecPointFrameArray>(
                    revision.driverFramesTap);
                values.driverFrames = &driverFrames;
            }
            if (revision.transformTap >= 0) {
                transform = snapshot.Get<GfMatrix4d>(revision.transformTap);
                values.transform = &transform;
            }
            // A "final" read phase takes the provider's aim-revised matrix.
            // That used to be expressed by binding the generated frame-chain
            // head as the transform provider; now the binding names the
            // provider itself and the revised value is substituted here.
            if (revision.transformFinalPhase) {
                const auto revisedIt =
                    finalMatrices.find(revision.binding.transform);
                if (revisedIt != finalMatrices.end()) {
                    transform = revisedIt->second;
                    values.transform = &transform;
                }
            }
            if (revision.weightTap >= 0) {
                weights =
                    snapshot.Get<RigExecWeightPacket>(revision.weightTap);
                values.weights = &weights;

                // rigExec:samplePhase = "current": the field is measured
                // against the points AS THEY STAND HERE, not the
                // authored base, so the volume grabs whatever is inside
                // it right now.
                //
                // It cannot come from exec. The revision node's only
                // inputs are its parameters, its status, and the
                // read-write point buffer, and the parameters are baked
                // as a VDF constant when the graph is built -- nothing
                // in the packet can depend on a value the graph has not
                // computed yet. What CAN be done is evaluate the chain
                // built SO FAR (RigExecMoverGraph::Evaluate is const and
                // takes any masked output), measure against that, and
                // bake the result into this revision's parameters. One
                // extra graph evaluation per current-phase revision,
                // paid only by rigs that ask for it.
                if (_currentPhaseWeights.count(
                        revision.binding.weightObject)) {
                    const VtVec3fArray inFlight = graph.Evaluate(head);
                    const std::vector<GfVec3f> currentPoints(
                        inFlight.begin(), inFlight.end());
                    std::vector<float> field;
                    std::string weightError;
                    if (_ResolveWeights(revision.binding.weightObject,
                                        currentPoints.size(), time, &field,
                                        &weightError, &currentPoints)) {
                        weights.representation = TfToken("dense");
                        weights.values = std::move(field);
                        weights.indices.clear();
                        weights.defaultWeight = 0.0f;
                        weights.valid = true;
                    } else {
                        // An invalid packet is the kernel's atomic
                        // MoverFailed pass-through, which is the right
                        // answer here: publishing the reference-phase
                        // field instead would silently be a different
                        // deformation.
                        weights = RigExecWeightPacket();
                        pose.diagnostics.push_back(
                            "current-phase weight failed: " + weightError);
                    }
                }
            }
            // Publish the field an authoring tool paints as an influence
            // overlay. Taken from the packet the mover is about to
            // consume, so what a rigger sees is exactly what deformed
            // the geometry -- not a re-derivation that could drift.
            if (revision.weightTap >= 0 && weights.valid) {
                RigExecResolvedWeightField &field =
                    pose.weightFields[revision.binding.weightObject];
                field.target = target;
                field.weights.assign(basePoints.size(), 0.0f);
                for (size_t i = 0; i < basePoints.size(); ++i) {
                    const float w = weights.Resolve(i, basePoints.size());
                    field.weights[i] = w < 0.0f ? 0.0f : w;
                }
            }
            values.basePoints.assign(basePoints.begin(), basePoints.end());
            if (!revision.blendChannelTaps.empty()) {
                std::vector<RigExecBlendChannel> channels;
                channels.reserve(revision.blendChannelTaps.size());
                for (const RigExecTapId tap : revision.blendChannelTaps) {
                    channels.push_back(
                        snapshot.Get<RigExecBlendChannel>(tap));
                }
                // A structural failure leaves blendDeltas empty, which is
                // what makes the assembled packet invalid -- the same atomic
                // MoverFailed pass-through the kernel produces.
                if (!RigExecSumBlendChannels(channels, values.basePoints,
                                             &values.blendDeltas)) {
                    values.blendDeltas.clear();
                }
            }

            const RigExecMoverParameters parameters =
                RigExecAssembleParameters(moverPrim, revision.op,
                                          revision.binding, values, time);
            head = graph.AddRevision(
                revision.op, head, parameters,
                RigExecStatusForParameters(parameters, revision.moverPath));
            ++graphRevisionsBuilt;
        }
        if (!built) {
            continue;
        }

        const VtVec3fArray graphPoints = graph.Evaluate(head);
        pose.movedProperties[target] = VtValue(graphPoints);
        ++graphChainsChecked;

        // Derived maintenance reads this chain's final points, which is why it
        // runs here rather than as another entry in _graphChains.
        const auto derivedIt = _graphDerivedChains.find(target);
        if (derivedIt == _graphDerivedChains.end()) {
            continue;
        }
        for (const _GraphRevision &derived : derivedIt->second) {
            VtVec3fArray derivedBase;
            const UsdAttribute derivedAttr =
                _stage->GetAttributeAtPath(derived.target);
            if (!derivedAttr || !derivedAttr.Get(&derivedBase, time)) {
                continue;
            }
            RigExecProviderValues values;
            values.basePoints.assign(graphPoints.begin(), graphPoints.end());

            RigExecMoverGraph derivedGraph;
            const VdfMaskedOutput derivedSource =
                derivedGraph.AddPointSource(derived.target, derivedBase);
            const RigExecMoverParameters parameters =
                RigExecAssembleParameters(
                    _stage->GetPrimAtPath(derived.moverPath), derived.op,
                    derived.binding, values, time);
            const VdfMaskedOutput derivedHead = derivedGraph.AddRevision(
                derived.op, derivedSource, parameters,
                RigExecStatusForParameters(parameters, derived.moverPath));
            ++graphRevisionsBuilt;

            const VtVec3fArray derivedResult =
                derivedGraph.Evaluate(derivedHead);
            pose.movedProperties[derived.target] = VtValue(derivedResult);
            ++graphChainsChecked;
        }
    }
    // The same chains against the scalar CPU reference (spec §7.4). This is
    // the oracle that OUTLIVES the generated-prim chains: it resolves every
    // input off the authored stage itself and authors nothing, so it survives
    // the compiler's deletion, and it is a genuinely independent
    // implementation -- it does not call RigExecAssembleParameters, which is
    // why it can catch a packet-assembly drift rather than share one.
    {
        std::map<SdfPath, std::vector<const RigExecMoverRecord *>> chains;
        for (const RigExecMoverRecord &mover : _movers) {
            for (const SdfPath &target : mover.targets) {
                if (target.IsPropertyPath() &&
                    target.GetNameToken() == "points") {
                    chains[target].push_back(&mover);
                }
            }
        }
        for (const auto &[target, chain] : chains) {
            const auto graphIt = pose.movedProperties.find(target);
            if (graphIt == pose.movedProperties.end() ||
                !graphIt->second.IsHolding<VtVec3fArray>()) {
                continue;
            }
            std::vector<std::string> quiet;
            const VtVec3fArray reference =
                _EvaluateChain(target, chain, pose, time, &quiet);
            const VtVec3fArray &graphValue =
                graphIt->second.UncheckedGet<VtVec3fArray>();
            if (reference.size() != graphValue.size()) {
                pose.diagnostics.push_back(
                    "cpu reference parity: size mismatch on " +
                    target.GetString());
                ++pose.moverGraphParityMismatches;
                continue;
            }
            for (size_t i = 0; i < reference.size(); ++i) {
                if (!GfIsClose(reference[i], graphValue[i], 1e-4)) {
                    pose.diagnostics.push_back(
                        "cpu reference parity: value mismatch on " +
                        target.GetString() + " at element " +
                        std::to_string(i));
                    ++pose.moverGraphParityMismatches;
                    break;
                }
            }
        }
    }

    // Reported unconditionally, including the zero case: a parity pass that
    // silently checked nothing is indistinguishable from one that passed, and
    // that is exactly how a no-op hides (spec §7.4 parity reporting).
    pose.moverGraphParityAgreements = graphChainsChecked;
    pose.diagnostics.push_back(
        "mover graph parity: " + std::to_string(graphChainsChecked) +
        " chain(s) agreed over " + std::to_string(graphRevisionsBuilt) +
        " revision(s)");
    // Optional CPU reference-kernel parity for the lowered chains
    // (scalar-reference goldens, spec §7.4).
    if (cpuParityMode) {
        std::map<SdfPath, std::vector<const RigExecMoverRecord *>> chains;
        for (const RigExecMoverRecord &mover : _movers) {
            for (const SdfPath &target : mover.targets) {
                if (target.IsPropertyPath() &&
                    target.GetNameToken() == "points") {
                    chains[target].push_back(&mover);
                }
            }
        }
        for (const auto &[target, chain] : chains) {
            pose.movedPropertiesCpu[target] = VtValue(_EvaluateChain(
                target, chain, pose, time, &pose.diagnostics));
        }
    }

    pose.valid = true;
    return pose;
}

}  // namespace rigExec
