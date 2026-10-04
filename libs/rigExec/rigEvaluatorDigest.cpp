// Structural fingerprints and digest settlement.

#include "rigEvaluatorInternal.h"
#include "rigEvaluatorDependencies.h"
#include "rigEvaluatorConstraints.h"
#include "parallel.h"
#include "pathText.h"
#include "movers/moverRegistry.h"

#include "pxr/base/work/dispatcher.h"
#include "pxr/base/work/withScopedParallelism.h"
#include "pxr/base/work/threadLimits.h"
#include "pxr/base/tf/diagnostic.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usdGeom/xformable.h"

#include <algorithm>
#include <functional>
#include <set>
#include <unordered_map>

namespace rigExec {

using namespace evaluatorDetail;

namespace {

// Every path the structure digest writes is spelled through one of these
// (see pathText.h).
using _PathText = RigExecPathText;

// The Solvers digest's prefetch (see _ComputeStructureDigest) runs on more
// than one lane only with at least this many prims to read, and gives every
// lane at least this many. One prim is a few tens of microseconds of reads,
// so below the floor the fork and join cost more than the reads they spread.
constexpr size_t _kDigestPrefetchParallelMin = 16;

constexpr size_t _kDigestPrefetchPerLane = 4;

// Its width beside compile (_DigestBesideCompile). MEASURED, cold compiles,
// ms of prefetch by lanes:
//   puppetA  1 / 2 / 3 / 4 / pool:       14.6 / 8.2 / 7.0 / 4.9 / 3.4
//   biped    1 / 2 / 4 / 8 / 16 / 32:    24.6 / 20.0 / 12.2 / 12.5 / 12.7 / 12.9
// DiscoverValidate on the compiling thread stays flat up to 4 lanes and is
// ~1 ms slower with the whole pool. The biped stops scaling at 4: USD reads
// beside compile's own share something that serializes them (one lane runs
// at half the speed it does with nothing beside it).
constexpr size_t _kDigestPrefetchLanesBesideCompile = 4;

// RIGEXEC_VERIFY_DIGEST_MEMO=1: every join also recomputes the Solvers
// segment with its read memo off and fails fatally unless the two texts are
// equal.
static bool
_DigestMemoVerifyRequested()
{
    static const bool requested =
        TfGetenvBool("RIGEXEC_VERIFY_DIGEST_MEMO", false);
    return requested;
}

// The index of the first byte at which two digest texts differ.
static size_t
_FirstDigestDifference(const std::string &a, const std::string &b)
{
    size_t at = 0;
    while (at < a.size() && at < b.size() && a[at] == b[at]) {
        ++at;
    }
    return at;
}

} // namespace

namespace evaluatorDetail {

// RIGEXEC_VERIFY_DIGEST_SPLIT=1: every join of the three digest segments
// also computes the whole digest serially and fails fatally unless the two
// texts are equal.
bool
_DigestSplitVerifyRequested()
{
    static const bool requested =
        TfGetenvBool("RIGEXEC_VERIFY_DIGEST_SPLIT", false);
    return requested;
}

} // namespace evaluatorDetail

std::string
RigExecRigEvaluator::_ComputeStructureDigest(
    unsigned segments, _DigestFootprint *footprint) const
{
    // The v0.1 binding-epoch identity: canonical mover paths, schema
    // types, targets, mover execution ordinals, and the structural dependency
    // wiring each operation declares — relationship identities, read
    // phases, weight-descriptor shape, and blend membership/activations
    // (spec §4.2, §6.3). Structural edits change it; numeric values and
    // shape-preserving enables do not.
    // Returned as TEXT, of only the segments \p segments selects, so the
    // three can be computed on three tasks and hashed once at the join
    // (_JoinStructureDigest). Everything a segment appends is local to this
    // call -- the string, the weight-object visiting set (empty again after
    // every top-level walk), the solver closure caches, the profile region
    // -- so a segment emits the same bytes whether it runs alone or beside
    // the other two in one call.
    std::string digest;
    // Every path this call writes is spelled through here (see _PathText).
    _PathText pathText;
    // Made once here, never per read: constructing a token from text takes
    // the token registry's lock, and readOf below runs on the prefetch's
    // parallel lanes.
    const TfToken phaseField(RigExecReadPhaseMetadataName);

    const bool profileDigest = _profiler.IsEnabled();
    uint64_t digestRegionStart =
        profileDigest ? RigExecProfiler::NowUs() : 0;
    auto stampDigestRegion = [&](const char *name) {
        if (!profileDigest) {
            return;
        }
        const uint64_t now = RigExecProfiler::NowUs();
        _profiler.Record(name, "compile", digestRegionStart, now);
        digestRegionStart = now;
    };

    // THE FOOTPRINT: the prims outside the rig this call reads, which is
    // what the notice gate asks an edit about (_NoticeIsDigestSuspect).
    // Recorded where a walk REACHES a prim -- a relationship or connection
    // target, missing or not, and the ancestors a chain walks -- rather than
    // at every read of it: everything read afterwards is read off a prim
    // already recorded. Prims in the rig are not recorded, because every
    // notice there is suspect anyway.
    const SdfPath &rigPath = _rigPath;
    const auto noteRead = [footprint, &rigPath](const SdfPath &path) {
        if (!footprint || path.IsEmpty()) {
            return;
        }
        SdfPath prim = path.GetPrimPath();
        if (!prim.IsAbsoluteRootPath() && !prim.HasPrefix(rigPath)) {
            footprint->read.insert(std::move(prim));
        }
    };
    const auto noteAncestor = [footprint, &rigPath](const SdfPath &prim) {
        if (footprint && !prim.IsEmpty() && !prim.IsAbsoluteRootPath() &&
            !prim.HasPrefix(rigPath)) {
            footprint->ancestors.insert(prim);
        }
    };
    // THE CERTAIN FOOTPRINT: what this call wrote that an edit can be seen
    // to change without recomputing it (_EditIsCertainlyStructural). A
    // relationship is recorded wherever its targets are written, with the
    // targets as composed, and stays "sorted" only while every place that
    // wrote it sorted them; a prim is recorded where its path is written
    // because of its type.
    // Only relationships that exist. The digest also writes an empty list
    // for each one a prim's type could carry and does not -- most of the
    // mover segment's names, on any one mover -- and recording those too
    // was measured at ~1.5 ms more on the Movers task on the biped. A
    // relationship authored for the first time is left to the digest.
    const auto noteTargets = [footprint](const UsdPrim &prim,
                                         const TfToken &name,
                                         const SdfPathVector &targets,
                                         bool sorted) {
        if (!footprint || !prim) {
            return;
        }
        const auto [entry, inserted] = footprint->certain.targets.try_emplace(
            _CertainFootprint::PropertyKey{prim.GetPath(), name});
        if (inserted) {
            entry->second.targets = targets;
        }
        entry->second.sorted = entry->second.sorted && sorted;
    };
    const auto notePrim = [footprint, &rigPath](const SdfPath &path,
                                                const TfToken &type) {
        if (footprint && path != rigPath && path.HasPrefix(rigPath)) {
            footprint->certain.prims.emplace(path, type);
        }
    };

    // Named by token where the caller already holds one -- the solver loop
    // walks a prim's relationships and has each one's name -- and by string
    // literal everywhere else, which interns the token on every call.
    auto appendRelTargetsNamed =
        [this, &digest, &noteRead, &noteTargets, &pathText](
            const UsdPrim &prim, const TfToken &nameToken, bool sorted) {
        SdfPathVector targets;
        if (UsdRelationship rel = prim.GetRelationship(nameToken)) {
            rel.GetTargets(&targets);
            noteTargets(prim, nameToken, targets, sorted);
        }
        for (const SdfPath &target : targets) {
            noteRead(target);
        }
        if (sorted) {
            std::sort(targets.begin(), targets.end());
        }
        digest += nameToken.GetString();
        digest += '=';
        for (const SdfPath &t : targets) {
            digest += pathText(t);
            digest += ',';
        }
        digest += '|';
        return targets;
    };
    auto appendRelTargets = [&appendRelTargetsNamed](const UsdPrim &prim,
                                                     const char *name,
                                                     bool sorted) {
        return appendRelTargetsNamed(prim, TfToken(name), sorted);
    };
    // A read phase decides WHICH revision of an input a mover consumes, which
    // is compiled wiring, not a value -- so editing one has to re-epoch
    // exactly the way retargeting the relationship does. Authored on the
    // property, so it is hashed alongside that property's targets rather than
    // as another prim-level token.
    auto appendPhaseNamed = [&digest, &phaseField](const UsdPrim &prim,
                                                   const TfToken &name) {
        std::string authored;
        if (const UsdRelationship rel = prim.GetRelationship(name)) {
            rel.GetMetadata(phaseField, &authored);
        } else if (const UsdAttribute a = prim.GetAttribute(name)) {
            a.GetMetadata(phaseField, &authored);
        }
        digest += name.GetString();
        digest += "@phase=";
        digest += authored;
        digest += '|';
    };
    auto appendPhase = [&appendPhaseNamed](const UsdPrim &prim,
                                           const char *name) {
        appendPhaseNamed(prim, TfToken(name));
    };
    auto appendToken = [&digest](const UsdPrim &prim, const char *name) {
        TfToken value;
        if (UsdAttribute a = prim.GetAttribute(TfToken(name))) {
            a.Get(&value);
            digest += name;
            digest += "#samples=";
            digest += std::to_string(a.GetNumTimeSamples());
            digest += '|';
        }
        digest += name;
        digest += '=';
        digest += value.GetString();
        digest += '|';
    };
    // Any scalar attribute as text. VtValue's stream operator rather than a
    // type switch: what the digest needs is that two different authored
    // values produce two different strings, not that the string is pretty.
    auto appendScalar = [&digest](const UsdPrim &prim, const char *name) {
        digest += name;
        digest += '=';
        VtValue value;
        if (const UsdAttribute a = prim.GetAttribute(TfToken(name))) {
            a.Get(&value);
        }
        digest += TfStringify(value);
        digest += '|';
    };
    auto appendAttributeBinding =
        [this, &digest, &noteRead, &pathText, &appendRelTargets,
         &phaseField](const UsdPrim &prim, const char *name) {
        const UsdAttribute attr = prim.GetAttribute(TfToken(name));
        digest += name;
        digest += "@sources=";
        std::set<SdfPath> visiting;
        std::function<void(const UsdAttribute &)> append =
            [&](const UsdAttribute &a) {
            if (!a || !visiting.insert(a.GetPath()).second) {
                if (a) {
                    digest += "cycle:";
                    digest += pathText(a.GetPath());
                } else {
                    digest += "missing";
                }
                digest += ',';
                return;
            }
            digest += pathText(a.GetPath());
            const auto frameInputs = appendRelTargets(a.GetPrim(), "rigExec:poseInputs", false);
            for (const SdfPath &input : frameInputs) noteRead(input);
            digest += ":" +
                      a.GetTypeName().GetAsToken().GetString() + ":samples:" +
                      std::to_string(a.GetNumTimeSamples()) + "->";
            const SdfPathVector sources = _AuthoredConnections(a);
            for (const SdfPath &source : sources) {
                noteRead(source);
                const UsdAttribute sourceAttr =
                    _stage->GetAttributeAtPath(source);
                if (!sourceAttr) {
                    digest += "missing:";
                    digest += pathText(source);
                    digest += ',';
                } else {
                    append(sourceAttr);
                }
            }
            visiting.erase(a.GetPath());
        };
        append(attr);
        // The read phase a connected input declares selects which revision
        // of a property chain its connection reads, which Compile resolves.
        // An unconnected input has nothing to choose, so it is not read.
        if (attr && attr.HasAuthoredConnections()) {
            std::string phase;
            attr.GetMetadata(phaseField, &phase);
            digest += "@phase=";
            digest += phase;
        }
        digest += '|';
    };
    auto appendFrameBindingIdentityNamed =
        [this, &digest, &noteRead, &noteTargets, &pathText](
            const UsdPrim &prim, const TfToken &nameToken) {
        SdfPathVector targets;
        if (const UsdRelationship rel = prim.GetRelationship(nameToken)) {
            rel.GetTargets(&targets);
            noteTargets(prim, nameToken, targets, false);
        }
        digest += nameToken.GetString();
        digest += "@bindings=";
        for (const SdfPath &target : targets) {
            noteRead(target);
            const UsdPrim provider =
                _stage->GetPrimAtPath(target.GetPrimPath());
            const TfToken type = provider ? provider.GetTypeName() : TfToken();
            digest += pathText(target);
            digest += ':';
            digest += type.GetString();
            digest += ':';
            if (type == "RigExecControl" || type == "RigExecJoint") {
                digest += "frameTap";
            } else if (provider && UsdGeomXformable(provider)) {
                digest += "nativeXform";
            } else {
                digest += "invalid";
            }
            digest += ',';
        }
        digest += '|';
    };
    auto appendFrameBindingIdentity =
        [&appendFrameBindingIdentityNamed](const UsdPrim &prim,
                                           const char *name) {
        appendFrameBindingIdentityNamed(prim, TfToken(name));
    };

    // Weight-object descriptor shape is epoch identity (spec §4.1):
    // target, representation, policy, and canonical sparse support.
    // Recursive, because a combine's field shape is its inputs' shapes:
    // an edit inside a composed input has to re-epoch the combine that
    // folds it, or the baked falloff tables replay stale.
    // Cycle-TRACKED rather than depth-limited. A depth cap terminates,
    // but it terminates by silently dropping everything below it, so a
    // legitimately deep composition stops contributing to the epoch
    // identity and edits down there stop triggering a recompile. Marking
    // the path being walked costs the same and is exact; a genuine cycle
    // is caught and reported by the compile-time walk instead.
    std::set<SdfPath> digestVisiting;
    std::function<void(const SdfPath &)> appendWeightObject =
        [&](const SdfPath &weightPath) {
        noteRead(weightPath);
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
        VtFloatArray values;
        if (UsdAttribute a = w.GetAttribute(TfToken("rigExec:values"))) {
            a.Get(&values);
        }
        digest += "rigExec:values#size=" +
                  std::to_string(values.size()) + '|';

        // Attribute connection identity is compiled Exec wiring. A rewire
        // must rebuild the prepared request even when the two sources happen
        // to carry the same value at the current time. Static fields include
        // these markers too so adding an illegal source re-enters Compile and
        // is rejected instead of replaying the old request.
        for (const char *field : kDigestWeightObjectFields) {
            appendAttributeBinding(w, field);
        }

        // Volumetric extension. Only STRUCTURAL properties belong here:
        // the shape family, which axis it measures, where it samples,
        // and the baked remap. inputs:falloffMin/Max, invert, strength,
        // scaleX/Y/Z and extentU/V are deliberately absent -- they are
        // per-frame exec values, and hashing them would recompile every
        // frame an artist scrubs one. rigExec:planeBounds IS here
        // because it selects which field function runs, exactly as
        // rigExec:planeAxis selects which coordinate it measures.
        appendToken(w, "rigExec:falloffProfile");
        appendPhase(w, "rigExec:weightTarget");
        appendToken(w, "rigExec:planeAxis");
        appendToken(w, "rigExec:planeBounds");
        appendToken(w, "rigExec:combineMode");
        const SdfPathVector curveTargets =
            appendRelTargets(w, "rigExec:curve", true);
        // A CurveWeight's relationship alone is not the complete structural
        // binding: the target must still resolve to a point3f[] attribute.
        // Hash that resolution so removing/retyping the source, or repairing
        // it in place without changing the relationship path, re-enters
        // Compile and applies the same validation as the original authoring.
        for (const SdfPath &target : curveTargets) {
            const SdfPath pointsPath = target.IsPropertyPath()
                ? target
                : target.AppendProperty(TfToken("points"));
            const UsdAttribute points =
                _stage->GetAttributeAtPath(pointsPath);
            digest += "rigExec:curveSource=";
            digest += pathText(pointsPath);
            digest += ':';
            digest += points
                ? points.GetTypeName().GetAsToken().GetString()
                : std::string("missing");
            digest += '|';
        }
        appendRelTargets(w, "rigExec:sampleSource", true);
        // The falloff curve is structural: it is resampled to a table
        // once per epoch, so an edit to it has to begin a new one.
        // What gets hashed is the BAKED TABLE, not the knots. Hashing
        // knot times and values misses everything else that changes the
        // curve's shape -- interpolation mode, tangent slopes and widths,
        // dual values, extrapolation, loops -- so flipping a knot from
        // curve to held left the digest unchanged, the epoch unrebuilt,
        // and exec replaying a stale LUT while the CPU oracle rebaked the
        // live spline. Hashing the table is exact by construction: it is
        // precisely the bytes exec consumes, so anything that changes
        // them re-epochs and nothing that does not, does.
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
        static const TfToken kInputWeights("rigExec:inputWeights");
        SdfPathVector inputs;
        if (UsdRelationship rel = w.GetRelationship(kInputWeights)) {
            rel.GetTargets(&inputs);
            noteTargets(w, kInputWeights, inputs, false);
        }
        digest += "rigExec:inputWeights=";
        for (const SdfPath &t : inputs) {
            noteRead(t);
            digest += pathText(t);
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

    if (segments & _DigestOutputSets) {
        // Rig output set. Discovered rather than authored, so the digest hashes the
        // discovered paths -- adding, removing, or renaming a joint prim changes
        // the epoch exactly as editing the old manifest relationship did.
        // Derived-property maintenance (spec §7.6 revised) is unconditional now, so
        // there is no policy token left to hash: what the compiler synthesizes depends
        // only on which gprims author normals/extent, and that is already epoch
        // identity through the points-chain targets below.
        static const TfToken kJointType("RigExecJoint");
        static const TfToken kControlType("RigExecControl");
        static const TfToken kInterpolatorType("RigExecPoseInterpolator");
        for (const SdfPath &jointPath : _DiscoverJointOutputs(_stage, _rigPath)) {
            notePrim(jointPath, kJointType);
            digest += pathText(jointPath);
            digest += ',';
        }
        digest += '|';

        // The discovered control set, for the same reason: it decides which
        // computePointFrame taps the epoch's prepared request carries. A control
        // that no solver reads is otherwise invisible to this digest -- adding
        // one purely to draw a guide would leave the compiled tap set behind and
        // the guide would never appear.
        for (const SdfPath &controlPath : _DiscoverControls(_stage, _rigPath)) {
            notePrim(controlPath, kControlType);
            digest += pathText(controlPath);
            digest += ',';
        }
        digest += '|';

        // Standalone volume guides carry placement taps and baked falloff state
        // even when no mover consumes their field. Their discovered paths and
        // structural properties therefore belong to the epoch just as standalone
        // controls do; otherwise adding or repairing one would replay a tap set
        // that can never publish it.
        for (const SdfPath &volumePath :
             _DiscoverVolumeWeights(_stage, _rigPath)) {
            if (footprint) {
                notePrim(volumePath,
                         _stage->GetPrimAtPath(volumePath).GetTypeName());
            }
            digest += pathText(volumePath);
            digest += '|';
            appendWeightObject(volumePath);
        }
        digest += '|';

        // Pose interpolators. The RBF solve is a compile-time CONSTANT -- the
        // inverted matrix of every pose's kernel value at every other pose -- so
        // everything that constant is a function of is epoch identity: the
        // driver, every pose's rotation, translation, type and radii, the kernel,
        // the twist axis, the regularization and which channels are enabled.
        // Numeric though most of those are, an edit to one has to reach a solve
        // that has already happened, and nothing else in this digest hashes an
        // interpolator.
        // inputs:enabled on a POSE is hashed and inputs:enabled on the
        // INTERPOLATOR is not, which is the same distinction the schema draws: a
        // disabled pose is left out of the solve entirely (leaving it in would
        // keep it in every other pose's matrix row, so switching one off would
        // quietly change all the others), while a disabled interpolator just
        // publishes zeros and needs no recompile to do it.
        for (const SdfPath &interpolatorPath :
             _DiscoverPoseInterpolators(_stage, _rigPath)) {
            const UsdPrim interpolator = _stage->GetPrimAtPath(interpolatorPath);
            notePrim(interpolatorPath, kInterpolatorType);
            digest += pathText(interpolatorPath);
            digest += '|';
            appendRelTargets(interpolator, "rigExec:driver", false);
            appendRelTargets(interpolator, "rigExec:driverAttributes", false);
            for (const char *name : {"rigExec:kernel", "rigExec:twistAxis",
                                     "rigExec:regularization",
                                     "rigExec:normalize",
                                     "rigExec:enableRotation",
                                     "rigExec:enableTranslation",
                                     "rigExec:allowNegativeWeights"}) {
                appendScalar(interpolator, name);
            }
            for (const UsdPrim &pose : interpolator.GetChildren()) {
                digest += pose.GetName().GetString();
                digest += '=';
                for (const char *name : {"rigExec:poseType", "rigExec:rotation",
                                         "rigExec:translation",
                                         "rigExec:rotationRadius",
                                         "rigExec:translationRadius",
                                         "inputs:enabled"}) {
                    appendScalar(pose, name);
                }
                digest += ';';
            }
            digest += '|';
        }
        digest += '|';

    // Space switches. Everything the compile resolves once is epoch
    // identity: which prim is switched, the ordered source list, the labels
    // parallel to it, where the active index is read from, and the masks --
    // all of which decide the shape of the taps and the dependency order
    // between switches. inputs:activeSpace is deliberately NOT hashed: it is
    // the animated channel, read per frame like any avar, and an edit to it
    // must not rebuild the epoch.
    for (const UsdPrim &prim :
         UsdPrimRange(_stage->GetPrimAtPath(_rigPath))) {
        if (prim.GetTypeName() != "RigExecSpaceSwitch") continue;
        digest += prim.GetPath().GetString();
        digest += '|';
        appendRelTargets(prim, "rigExec:target", false);
        appendRelTargets(prim, "rigExec:sources", false);
        appendRelTargets(prim, "rigExec:activeSpaceAttribute", false);
        // The space decides a tap pair and the dependency order between
        // switches, so it is epoch identity like the sources it joins.
        appendRelTargets(prim, "rigExec:space", false);
        for (const char *name : {"rigExec:spaceLabels",
                                 "rigExec:rotationFilters",
                                 "rigExec:twistAxis",
                                 "inputs:affectTranslationX",
                                 "inputs:affectTranslationY",
                                 "inputs:affectTranslationZ",
                                 "inputs:affectRotationX",
                                 "inputs:affectRotationY",
                                 "inputs:affectRotationZ",
                                 "inputs:affectScaleX",
                                 "inputs:affectScaleY",
                                 "inputs:affectScaleZ",
                                 "inputs:sourceWeights"}) {
            appendScalar(prim, name);
        }
        digest += '|';
    }
    digest += '|';
    // RigExecAutoClavicle: its prims and constants decide taps and solved
    // pose tables. The blend and amount PROPERTIES it reads are per-frame
    // channels and stay out, as a switch's active index does.
    for (const UsdPrim &prim :
         UsdPrimRange(_stage->GetPrimAtPath(_rigPath))) {
        if (prim.GetTypeName() != "RigExecAutoClavicle") continue;
        digest += prim.GetPath().GetString();
        digest += '|';
        for (const char *rel :
             {"rigExec:target", "rigExec:pivot", "rigExec:anchor",
              "rigExec:fkControls", "rigExec:ikTarget", "rigExec:poleControl",
              "rigExec:ikBlendAttribute", "rigExec:amountAttribute"}) {
            appendRelTargets(prim, rel, false);
        }
        for (const char *name :
             {"rigExec:ikValue", "inputs:gain", "rigExec:basis",
              "rigExec:poseRotations", "rigExec:poseFalloffs",
              "rigExec:poseGains", "rigExec:kernel",
              "rigExec:regularization"}) {
            appendScalar(prim, name);
        }
        digest += '|';
    }
    digest += '|';

    // Read phases on connections: every connected operator input, the set
    // Compile resolves (_ForEachConnectedInput). Its record is a function of
    // its type, its authored phase (absent, or the text), and the
    // single-source walk up to the first property-chain target -- the
    // chain targets themselves are the movers segment's -- so the walk is
    // written to its end, hop by hop with each hop's type, and editing a
    // phase or rewiring any hop re-epochs the rig.
    _ForEachConnectedInput(
        _stage->GetPrimAtPath(_rigPath),
        [&](const UsdAttribute &input) {
            digest += pathText(input.GetPath());
            digest += ':';
            digest += input.GetTypeName().GetAsToken().GetString();
            if (input.HasAuthoredMetadata(phaseField)) {
                VtValue phase;
                input.GetMetadata(phaseField, &phase);
                digest += "@phase=";
                digest += TfStringify(phase);
            }
            SdfPathVector walked{input.GetPath()};
            SdfPathVector sources = _AuthoredConnections(input);
            for (;;) {
                digest += "->";
                for (const SdfPath &source : sources) {
                    noteRead(source);
                    digest += pathText(source);
                    digest += ',';
                }
                if (sources.size() != 1 ||
                    std::find(walked.begin(), walked.end(), sources[0]) !=
                        walked.end()) {
                    break;
                }
                const UsdAttribute next =
                    _stage->GetAttributeAtPath(sources[0]);
                if (!next) {
                    digest += "missing";
                    break;
                }
                digest += ':';
                digest += next.GetTypeName().GetAsToken().GetString();
                walked.push_back(sources[0]);
                sources = _AuthoredConnections(next);
            }
            digest += '|';
        },
        [&noteRead](const SdfPath &prim) { noteRead(prim); });
    digest += '|';

    stampDigestRegion("Digest.OutputSets");
        stampDigestRegion("Digest.OutputSets");
    }
    // Solver->joint wiring is epoch identity (view-free extraction,
    // user-directed 2026-07-25, replaces RigExecPointFrameView): each
    // solver's ORDERED rigExec:joints list decides which joint
    // self-extracts which aggregate element, so adding, removing, or
    // reordering joints changes what compile Pass 0 synthesizes. Order is
    // semantic (position = element index), so this list is never sorted.
    // The block a prim contributes is a pure function of (prim, composed
    // stage) -- it reads connections and namespace-frame providers and
    // nothing else -- and the stage cannot change underneath a const digest
    // computation, so memoizing it for the duration of THIS digest emits
    // exactly the bytes the uncached walk emits.
    // MEASURED 2026-09-13, biped (24 solvers): uncached, this emission was
    // 1.15 MILLION UsdAttribute::GetConnections calls per digest -- 2.33 s --
    // and one digest is computed on EVERY evaluate that follows ANY stage
    // edit, which is what made letting go of a gizmo take 2.4 s. It runs once
    // per ancestor, per relationship target, per relationship, per aggregate
    // solver, and each run re-walked the same pose-input closure from
    // scratch, so a few hundred prims were re-closed tens of thousands of
    // times: O(n^2) in solver count, the 455 + 51n + 6.2n^2 ms measured.
    // The caches are function-local: nothing survives the call, so a stage
    // edit between two digests is still seen. They are deliberately NOT
    // evaluator members -- a member cache would have to be invalidated by
    // _OnObjectsChanged, and the whole point of the digest is to be the
    // thing that does not trust incremental invalidation.
    std::unordered_map<SdfPath, std::string, SdfPath::Hash> solverInputTokens;
    // One attribute can sit in many blocks (every joint's parent:space chain
    // republishes its ancestors' attributes), so resolve each path's
    // "path:type->sources|" text once too.
    std::unordered_map<SdfPath, std::string, SdfPath::Hash> connectionText;
    // Each provider's transitive pose-input closure, as one token, shared by
    // every closure that reaches it.
    std::unordered_map<SdfPath, std::string, SdfPath::Hash>
        providerClosureTokens;
    struct _DigestHop {
        std::vector<SdfPath> own;     // this prim's attributes, sorted
        std::set<SdfPath> depends;    // prims this one reads
        /// Set by the prefetch below and nowhere else: the attributeText of
        /// every path in \c own, concatenated in \c own's order, and whether
        /// any of them has a connection source.
        bool prefetched = false;
        bool anyConnected = false;
        std::string ownText;
    };
    std::unordered_map<SdfPath, _DigestHop, SdfPath::Hash> digestHops;
    // THE READ MEMO: what this segment knows about one attribute path.
    // Every attribute the walks below touch is read for the same three
    // facts -- whether it exists, its type name, its authored connection
    // sources -- and most are read by three walks: hopFor lists them,
    // attributeText writes them out, and a prim's own connection-input
    // closure follows them. Each fact is a pure function of the composed
    // stage, so the first read of a path answers every later one. hopFor
    // fills the memo from the attributes GetAttributes hands it, which is
    // where nearly every path is first seen; a path first reached through a
    // connection (a source on another prim, or a missing one) is read with
    // GetAttributeAtPath on first use. Function-local for the same reason as
    // the caches above.
    // _DigestUnmemoizedReads turns the memo off -- every read goes back to
    // the stage, as it did before the memo existed -- which is what
    // RIGEXEC_VERIFY_DIGEST_MEMO compares this segment's bytes against.
    struct _DigestAttribute {
        bool exists = false;
        TfToken typeName;
        SdfPathVector sources;
        /// A connected attribute's declared read phase, which selects the
        /// revision of a property chain its connection reads.
        std::string phase;
    };
    const bool memoizeReads = !(segments & _DigestUnmemoizedReads);
    std::unordered_map<SdfPath, _DigestAttribute, SdfPath::Hash>
        digestAttributes;
    // The three facts of one attribute, and the text an attribute is written
    // out as. One definition of each, shared by the walk and by the prefetch
    // below, so the two cannot spell an attribute differently.
    const auto readOf = [&phaseField](const UsdAttribute &attribute) {
        _DigestAttribute read;
        read.exists = static_cast<bool>(attribute);
        if (read.exists) {
            read.typeName = attribute.GetTypeName().GetAsToken();
        }
        read.sources = _AuthoredConnections(attribute);
        if (!read.sources.empty()) {
            attribute.GetMetadata(phaseField, &read.phase);
        }
        return read;
    };
    const auto entryOf = [](_PathText &text, const SdfPath &path,
                            const _DigestAttribute &attribute) {
        std::string entry = text(path);
        entry += ':';
        entry += attribute.exists ? attribute.typeName.GetString()
                                  : std::string("missing");
        entry += "->";
        for (const SdfPath &source : attribute.sources) {
            entry += text(source);
            entry += ',';
        }
        if (!attribute.phase.empty()) {
            entry += "@phase=";
            entry += attribute.phase;
        }
        entry += '|';
        return entry;
    };
    // THE PREFETCH: one hop's worth of reads for each prim of the rig and
    // each of its ancestors, made in parallel before the walk starts and
    // adopted by hopFor the first time the walk asks for that prim. See the
    // prefetch itself, above the solver loop.
    struct _DigestPrefetched {
        _DigestHop hop;
        /// Parallel to hop.own.
        std::vector<_DigestAttribute> reads;
        std::vector<std::string> texts;
        bool isProvider = false;
        SdfPath frameParent;
    };
    std::vector<_DigestPrefetched> prefetched;
    std::unordered_map<SdfPath, size_t, SdfPath::Hash> prefetchedIndex;
    const auto rememberAttribute =
        [memoizeReads, &digestAttributes, footprint, &readOf](
            SdfPath path, const UsdAttribute &attribute)
            -> const _DigestAttribute & {
        if (memoizeReads) {
            const auto found = digestAttributes.find(path);
            if (found != digestAttributes.end()) {
                return found->second;
            }
        }
        _DigestAttribute read = readOf(attribute);
        // Every attribute read here is written out with its sources, so a
        // connected one goes into the certain footprint as it stands.
        if (footprint && !read.sources.empty()) {
            footprint->certain.closureConnections.insert_or_assign(
                path, read.sources);
        }
        return digestAttributes.insert_or_assign(std::move(path),
                                                 std::move(read))
            .first->second;
    };
    const auto readAttribute =
        [this, memoizeReads, &digestAttributes, &rememberAttribute](
            const SdfPath &path) -> const _DigestAttribute & {
        if (memoizeReads) {
            const auto found = digestAttributes.find(path);
            if (found != digestAttributes.end()) {
                return found->second;
            }
        }
        return rememberAttribute(path, _stage->GetAttributeAtPath(path));
    };
    const auto appendSolverInputConnections =
        [this, &digest, &solverInputTokens, &connectionText,
         &providerClosureTokens, &digestHops, memoizeReads,
         &rememberAttribute, &readAttribute, footprint, &noteRead,
         &noteAncestor, &pathText, &entryOf, &prefetched, &prefetchedIndex,
         &digestAttributes](const UsdPrim &prim) {
        const SdfPath primPath = prim ? prim.GetPath() : SdfPath();
        const auto cached = solverInputTokens.find(primPath);
        if (cached != solverInputTokens.end()) {
            digest += cached->second;
            return;
        }
        // THE TRANSITIVE POSE-INPUT CLOSURE, AS A MERKLE TOKEN.
        // This used to flatten a prim's whole provider closure into one set
        // of attribute paths and hash the text of all of them -- rebuilt
        // from scratch per prim, because every prim's closure is a
        // different set and so the per-prim cache below could not share
        // anything between them. On a nested chain a prim at depth d reads
        // d providers, so its block was O(d) entries of O(d)-long paths,
        // and the digest over N such prims was CUBIC. Measured on one
        // RigExecFkChain over a nested chain: 2.0 s at 100 joints, 191 s at
        // 400, all of it in Digest.Solvers.
        // Now each provider's closure is a token computed once: the text of
        // its OWN attributes plus the tokens of the providers it reads, in
        // path order. A provider is shared by every closure that reaches
        // it, so the work is linear in providers. Identity is at least as
        // strong as the flattened set: any change to any attribute, type or
        // connection anywhere in a closure changes that provider's token
        // and therefore every token that folds it in -- and it also
        // distinguishes WHICH provider an attribute arrived through, which
        // the flat set merged.
        // Iterative post-order, not recursion: closures run hundreds deep.
        // A cycle cannot be closed over, so the edge that closes one
        // contributes a marker naming the path instead; Compile reports the
        // cycle itself, and the marker still makes introducing or removing
        // one change the digest.
        const auto attributeText = [&connectionText, &readAttribute,
                                    &pathText, &entryOf](
                                       const SdfPath &path)
            -> const std::string & {
            auto text = connectionText.find(path);
            if (text == connectionText.end()) {
                text = connectionText
                           .emplace(path, entryOf(pathText, path,
                                                  readAttribute(path)))
                           .first;
            }
            return text->second;
        };
        // ONE HOP of a prim's pose inputs: every attribute it owns, and the
        // prims those attributes lead to. The same rules as
        // _CollectPoseInputInfo -- connections, parent:space to the
        // namespace frame provider, the default-space fallback -- but it
        // stops at the first foreign prim instead of following it, because
        // that prim's own token already covers everything past it.
        // _CollectPoseInputInfo follows the default-space fallback to the
        // root, so a prim at depth d returns ~13 attributes per ancestor in
        // a std::set whose SdfPath comparisons also walk depth. Built for
        // every prim, that stayed super-quadratic after the token work
        // above (10.4 s of 11 at 400 joints). The pose schedule still uses
        // it, unchanged; only the digest reads this instead.
        const auto hopFor = [&digestHops, &rememberAttribute, footprint,
                             &noteRead, &noteAncestor, &prefetched,
                             &prefetchedIndex, &digestAttributes,
                             &connectionText](
                                const UsdPrim &provider)
            -> const _DigestHop & {
            const SdfPath key = provider.GetPath();
            auto found = digestHops.find(key);
            if (found != digestHops.end()) {
                return found->second;
            }
            _DigestHop hop;
            if (footprint) {
                footprint->certain.closurePrims.insert(key);
            }
            bool isProvider = false;
            SdfPath frameParent;
            const auto ready = prefetchedIndex.find(key);
            if (ready != prefetchedIndex.end()) {
                // Adopted rather than read: the facts and texts the branch
                // below and attributeText would have made, already made. A
                // path the walk reached earlier through a connection keeps
                // the entry it has, which is the same one; a path it had not
                // goes into the certain footprint here, as rememberAttribute
                // would have put it there on its first read.
                _DigestPrefetched &made = prefetched[ready->second];
                for (size_t i = 0; i < made.hop.own.size(); ++i) {
                    const SdfPath &path = made.hop.own[i];
                    const auto [read, inserted] = digestAttributes.try_emplace(
                        path, std::move(made.reads[i]));
                    if (inserted && footprint &&
                        !read->second.sources.empty()) {
                        footprint->certain.closureConnections
                            .insert_or_assign(path, read->second.sources);
                    }
                    connectionText.try_emplace(path, std::move(made.texts[i]));
                }
                hop = std::move(made.hop);
                isProvider = made.isProvider;
                frameParent = made.frameParent;
            } else {
                const TfToken type = provider.GetTypeName();
                isProvider = _IsFrameProviderType(type);
                if (isProvider) {
                    if (const UsdPrim above =
                            _NamespaceFrameProvider(provider)) {
                        frameParent = above.GetPath();
                    }
                }
                for (const UsdAttribute &attribute :
                     provider.GetAttributes()) {
                    hop.own.push_back(attribute.GetPath());
                    const _DigestAttribute &read =
                        rememberAttribute(hop.own.back(), attribute);
                    for (const SdfPath &source : read.sources) {
                        if (source.GetPrimPath() != key) {
                            hop.depends.insert(source.GetPrimPath());
                        }
                    }
                    if (!isProvider || frameParent.IsEmpty()) {
                        continue;
                    }
                    // The fallback rules that leave this prim: each of them
                    // reads the namespace frame provider, whose token
                    // carries its own continuation of the chain.
                    const TfToken &name = attribute.GetName();
                    if (name == "parent:space" ||
                        name == "parent:defaultSpace" ||
                        name == "default:space") {
                        hop.depends.insert(frameParent);
                    }
                }
                std::sort(hop.own.begin(), hop.own.end());
            }
            // Footprint: every prim this hop leads to, and the ancestors the
            // namespace frame search walked through to find the provider it
            // falls back to (all of them, to the root, when it found none).
            if (footprint) {
                for (const SdfPath &input : hop.depends) {
                    noteRead(input);
                }
                if (isProvider) {
                    for (SdfPath above = key.GetParentPath();
                         !above.IsEmpty() && above != frameParent &&
                         !above.IsAbsoluteRootPath();
                         above = above.GetParentPath()) {
                        noteAncestor(above);
                    }
                }
            }
            return digestHops.emplace(key, std::move(hop)).first->second;
        };
        const auto tokenOf = [](const std::string &kind,
                                const std::string &text) {
            return kind + "#" + std::to_string(std::hash<std::string>{}(text)) +
                   ":" + std::to_string(text.size()) + "|";
        };

        std::string providerToken;
        if (prim) {
            // 1 = on the walk, 2 = token ready (in providerClosureTokens).
            std::unordered_map<SdfPath, int, SdfPath::Hash> state;
            std::vector<std::pair<SdfPath, bool>> walk{{prim.GetPath(), false}};
            while (!walk.empty()) {
                const auto [path, expanded] = walk.back();
                walk.pop_back();
                if (providerClosureTokens.count(path)) {
                    continue;
                }
                const UsdPrim provider = _stage->GetPrimAtPath(path);
                if (!provider) {
                    providerClosureTokens.emplace(
                        path, "missingProvider:" + pathText(path) + "|");
                    continue;
                }
                const _DigestHop &hop = hopFor(provider);
                // WHAT THIS TOKEN FOLDS IN: this prim's own attributes by
                // text, and every prim it reads one hop away by token. A
                // foreign attribute is never written out here -- its owner's
                // token already hashes all of that owner's own attributes,
                // so any edit to it changes this token too. That is at least
                // as strong an identity as the flattened set this replaced,
                // and marginally stronger: a structural edit to another
                // attribute of a connection-source prim now also re-epochs,
                // which is a recompile, never a missed one.
                const std::set<SdfPath> &depends = hop.depends;
                if (!expanded) {
                    if (state[path] == 1) {
                        continue;  // already scheduled on this walk
                    }
                    state[path] = 1;
                    walk.push_back({path, true});
                    for (const SdfPath &input : depends) {
                        if (!providerClosureTokens.count(input) &&
                            state[input] != 1) {
                            walk.push_back({input, false});
                        }
                    }
                    continue;
                }
                // std::sets: already in path order and unique, which is
                // what makes the token independent of the order anything
                // was discovered in.
                std::string text = pathText(path);
                text += '{';
                if (hop.prefetched) {
                    // The same entries attributeText hands back for these
                    // paths, concatenated in the same order, once.
                    text += hop.ownText;
                } else {
                    for (const SdfPath &attribute : hop.own) {
                        text += attributeText(attribute);
                    }
                }
                for (const SdfPath &input : depends) {
                    const auto ready = providerClosureTokens.find(input);
                    if (ready != providerClosureTokens.end()) {
                        text += ready->second;
                    } else {
                        // Still on the walk: this edge closes a cycle.
                        text += "cycle@";
                        text += pathText(input);
                        text += '|';
                    }
                }
                text += '}';
                state[path] = 2;
                providerClosureTokens.emplace(path, tokenOf("provider", text));
            }
            providerToken = providerClosureTokens[prim.GetPath()];
        }

        // The prim's own authored connection inputs sit beside its provider
        // closure, exactly as the flattened set used to add them. The same
        // iterative closure as _CollectAttributeConnectionInputs -- every
        // attribute the prim owns, then every source reached from one,
        // missing sources and cycles included -- but read through the memo,
        // which by now already holds the prim's own attributes (the walk
        // above ran hopFor on it) and most of what they connect to. The
        // solver cache keeps the stage-reading original.
        std::string block;
        std::set<SdfPath> connectionInputs;
        const _DigestHop *ownHop =
            prim && memoizeReads ? &hopFor(prim) : nullptr;
        if (ownHop && ownHop->prefetched && !ownHop->anyConnected) {
            // No attribute of the prim has a source, so the closure below
            // would be exactly its own attributes, in the order the set
            // keeps them -- which is the order hop.own is sorted in, and so
            // the order its concatenated text is already in.
            block = ownHop->ownText;
        } else if (prim) {
            std::vector<SdfPath> pending;
            if (memoizeReads) {
                pending = hopFor(prim).own;
            } else {
                for (const UsdAttribute &attribute : prim.GetAttributes()) {
                    pending.push_back(attribute.GetPath());
                }
            }
            while (!pending.empty()) {
                const SdfPath path = pending.back();
                pending.pop_back();
                if (!connectionInputs.insert(path).second) {
                    continue;
                }
                const SdfPathVector &sources = readAttribute(path).sources;
                for (const SdfPath &source : sources) {
                    noteRead(source);
                }
                pending.insert(pending.end(), sources.begin(), sources.end());
            }
        }
        for (const SdfPath &path : connectionInputs) {
            block += attributeText(path);
        }
        block += providerToken;
        // What lands in the digest is a TOKEN for this prim's closure, not
        // the closure text. The same prim's block is emitted once per
        // ancestor per target per relationship per solver, so appending the
        // text leaves the digest STRING quadratic in rig size even with the
        // walking cached away -- tens of MB of std::string concatenation per
        // evaluate. The digest as a whole is already collapsed to one size_t
        // by std::hash<std::string> on the way out, so folding a fixed-size
        // hash of the block in here is the same kind of identity it already
        // was: different closures give different tokens, and the surrounding
        // structure (which prim, which relationship, which ancestor) stays in
        // plain text around it. The length goes in beside the hash, so two
        // blocks have to collide in both to be confused.
        std::string token = "closure#" +
            std::to_string(std::hash<std::string>{}(block)) + ":" +
            std::to_string(block.size()) + "|";
        digest += token;
        solverInputTokens.emplace(primPath, std::move(token));
    };
    // THE ANCESTOR CHAIN OF A RELATIONSHIP TARGET, AS ONE TOKEN PER PATH.
    // Every aggregate-solver target contributes its whole ancestor chain --
    // each ancestor's path, type and input closure -- because a rewire
    // anywhere above a joint changes the frame it resolves against. That
    // used to be emitted inline: per target, walk every ancestor and append
    // its full path string. On a nested chain that is N targets x N
    // ancestors x a path N components long, and it was cubic. Measured on
    // one RigExecFkChain over a nested chain: 287 ms at 50 joints, 2.0 s at
    // 100, 188 s at 400 -- and 224 of the biped's 372 ms compile, the
    // largest single cost left after the pose-schedule work.
    // Siblings share every ancestor above them, and a chain's joints share
    // all of theirs, so each path's chain is computed once and folds in its
    // parent's token. The identity is the same kind the closure tokens above
    // already use: a hash of the exact text plus its length, so any change
    // to any ancestor's path, type or closure still changes every token
    // below it. Function-local for the same reason as the caches above.
    // Iterative, not recursive: a chain can be hundreds of joints deep.
    std::unordered_map<SdfPath, std::string, SdfPath::Hash> ancestorChainTokens;
    const auto ancestorChainToken =
        [this, &digest, &ancestorChainTokens, &appendSolverInputConnections,
         &noteAncestor, &pathText](const SdfPath &start)
            -> const std::string & {
        static const std::string kRoot;
        // Walk up to the first path already known (or the root), noting the
        // ones that are not; then build them top-down so each finds its
        // parent's token ready.
        std::vector<SdfPath> missing;
        for (SdfPath path = start;
             !path.IsEmpty() && path != SdfPath::AbsoluteRootPath();
             path = path.GetParentPath()) {
            if (ancestorChainTokens.count(path)) {
                break;
            }
            missing.push_back(path);
        }
        for (auto it = missing.rbegin(); it != missing.rend(); ++it) {
            const SdfPath &path = *it;
            noteAncestor(path);
            const UsdPrim ancestor = _stage->GetPrimAtPath(path);
            std::string block = pathText(path);
            block += ':';
            block += ancestor ? ancestor.GetTypeName().GetString()
                              : std::string("missing");
            block += ',';
            // The input closure's token, captured rather than appended: the
            // helper writes into the digest, and here it belongs inside this
            // path's block instead.
            const size_t mark = digest.size();
            appendSolverInputConnections(ancestor);
            block.append(digest, mark, std::string::npos);
            digest.resize(mark);
            const SdfPath parent = path.GetParentPath();
            const auto above = ancestorChainTokens.find(parent);
            if (above != ancestorChainTokens.end()) {
                block += above->second;
            }
            ancestorChainTokens.emplace(
                path, "chain#" +
                    std::to_string(std::hash<std::string>{}(block)) + ":" +
                    std::to_string(block.size()) + "|");
        }
        const auto found = ancestorChainTokens.find(start);
        return found != ancestorChainTokens.end() ? found->second : kRoot;
    };
    const UsdPrim rig = (segments & _DigestSolvers)
        ? _stage->GetPrimAtPath(_rigPath) : UsdPrim();
    // THE PREFETCH, which is how this segment is split. The walk below stays
    // one serial walk, and must: a cycle marker names the prim the walk was
    // on when it closed the cycle, so the bytes depend on its order. What it
    // spends its time on is not ordered, though. Most of the segment was one
    // hop's worth of reads per prim -- its attributes, their type names and
    // connection sources, its namespace frame provider -- and the text those
    // attributes are written out as, and every piece of that is a pure
    // function of one prim and the composed stage.
    // So it is all made here, up front, in parallel, for exactly the prims
    // the walk will ask hopFor about. The walk runs a pose-input closure from
    // every aggregate solver and from every ancestor of every target of one
    // of its relationships (ancestorChainToken), and a closure goes wherever
    // a hop's depends lead. So the prefetch seeds with those, then follows
    // depends a level at a time: each level is read in parallel, and what
    // its hops depend on and nobody has read yet is the next level. A prim
    // the walk never reaches is never read -- which matters on a rig like
    // the biped, whose walk reaches 227 of its 611 prims. Each result sits in
    // its own slot until hopFor adopts it, in walk order; a path that names
    // no prim is left to the walk, which writes it as a missing provider.
    // Off with the read memo: the unmemoized walk exists to check the memo
    // against, and a prefetch is a memo.
    if (rig && memoizeReads) {
        // Recorded by hand, nested inside the Digest.Solvers region: this
        // is a const method, and the scope macro wants a mutable profiler.
        const uint64_t prefetchStart =
            profileDigest ? RigExecProfiler::NowUs() : 0;
        std::vector<UsdPrim> prims;
        std::vector<UsdPrim> level;
        const auto enqueue = [&prefetchedIndex, &prims,
                              &level](const UsdPrim &prim) {
            if (prim &&
                prefetchedIndex.emplace(prim.GetPath(), prims.size() +
                                                            level.size())
                    .second) {
                level.push_back(prim);
            }
        };
        // The seeds, in the walk's own traversal: the same predicate, the
        // same relationships, the same composed targets.
        for (const UsdPrim &solver : UsdPrimRange(rig)) {
            if (!_IsAggregateSolverType(solver.GetTypeName())) {
                continue;
            }
            enqueue(solver);
            for (const UsdRelationship &rel : solver.GetRelationships()) {
                SdfPathVector targets;
                rel.GetTargets(&targets);
                for (const SdfPath &target : targets) {
                    for (SdfPath path = target.GetPrimPath();
                         !path.IsEmpty() && !path.IsAbsoluteRootPath();
                         path = path.GetParentPath()) {
                        if (!prefetchedIndex.count(path)) {
                            enqueue(_stage->GetPrimAtPath(path));
                        }
                    }
                }
            }
        }
        const auto prefetch = [&prims, &prefetched, &readOf, &entryOf](
                                  size_t begin, size_t lane, size_t lanes) {
            // A memo per lane: _PathText is never shared between threads.
            _PathText laneText;
            std::vector<std::pair<SdfPath, _DigestAttribute>> reads;
            for (size_t i = begin + lane; i < prims.size(); i += lanes) {
                const UsdPrim &provider = prims[i];
                const SdfPath &key = provider.GetPath();
                _DigestPrefetched &made = prefetched[i];
                // hopFor's rules, verbatim: see its uncached branch.
                const TfToken type = provider.GetTypeName();
                made.isProvider = _IsFrameProviderType(type);
                if (made.isProvider) {
                    if (const UsdPrim above =
                            _NamespaceFrameProvider(provider)) {
                        made.frameParent = above.GetPath();
                    }
                }
                reads.clear();
                for (const UsdAttribute &attribute :
                     provider.GetAttributes()) {
                    _DigestAttribute read = readOf(attribute);
                    for (const SdfPath &source : read.sources) {
                        if (source.GetPrimPath() != key) {
                            made.hop.depends.insert(source.GetPrimPath());
                        }
                    }
                    if (made.isProvider && !made.frameParent.IsEmpty()) {
                        const TfToken &name = attribute.GetName();
                        if (name == "parent:space" ||
                            name == "parent:defaultSpace" ||
                            name == "default:space") {
                            made.hop.depends.insert(made.frameParent);
                        }
                    }
                    reads.emplace_back(attribute.GetPath(), std::move(read));
                }
                std::sort(reads.begin(), reads.end(),
                          [](const auto &a, const auto &b) {
                              return a.first < b.first;
                          });
                made.hop.own.reserve(reads.size());
                made.reads.reserve(reads.size());
                made.texts.reserve(reads.size());
                for (auto &[path, read] : reads) {
                    std::string entry = entryOf(laneText, path, read);
                    made.hop.ownText += entry;
                    made.hop.anyConnected =
                        made.hop.anyConnected || !read.sources.empty();
                    made.hop.own.push_back(path);
                    made.reads.push_back(std::move(read));
                    made.texts.push_back(std::move(entry));
                }
                made.hop.prefetched = true;
            }
        };
        // How wide. Beside compile, a few lanes: compile's own thread is
        // reading the same prims at the same time, and USD reads slow each
        // other down -- MEASURED on puppetA, the whole pool took the prefetch
        // from 4.9 ms to 3.4 ms and added 1.1 ms to DiscoverValidate on the
        // compiling thread, which is on compile's critical path where the
        // digest is not. Alone, as the settle path runs it, it is the whole
        // wait, so the whole pool. Strided rather than blocked, because the
        // heavy prims (joints and controls, with their dozens of attributes)
        // sit together in the traversal.
        const size_t widest =
            !RigExecParallelEvaluationEnabled()
                ? 1
                : (segments & _DigestBesideCompile)
                      ? _kDigestPrefetchLanesBesideCompile
                      : WorkGetConcurrencyLimit();
        while (!level.empty()) {
            const size_t begin = prims.size();
            prims.insert(prims.end(), level.begin(), level.end());
            level.clear();
            prefetched.resize(prims.size());
            const size_t count = prims.size() - begin;
            const size_t lanes =
                count < _kDigestPrefetchParallelMin
                    ? 1
                    : std::max<size_t>(
                          1, std::min(widest,
                                      count / _kDigestPrefetchPerLane));
            if (lanes > 1) {
                WorkDispatcher dispatcher;
                for (size_t lane = 1; lane < lanes; ++lane) {
                    dispatcher.Run([&prefetch, begin, lane, lanes]() {
                        prefetch(begin, lane, lanes);
                    });
                }
                prefetch(begin, 0, lanes);
                dispatcher.Wait();
            } else {
                prefetch(begin, 0, 1);
            }
            // The next level: whatever this one depends on that nothing has
            // read yet, in the order its hops name them.
            for (size_t i = begin; i < prims.size(); ++i) {
                for (const SdfPath &input : prefetched[i].hop.depends) {
                    if (!prefetchedIndex.count(input)) {
                        enqueue(_stage->GetPrimAtPath(input));
                    }
                }
            }
        }
        if (profileDigest) {
            _profiler.Record("Digest.Solvers.Prefetch", "compile",
                             prefetchStart, RigExecProfiler::NowUs());
        }
    }
    if (rig) {
        // Recursive over the composed rig subtree (not GetChildren):
        // solvers live wherever the author put them, so every scope's
        // wiring must contribute to epoch identity
        // (consistent with mover discovery and compile Pass 0).
        // This walk's ORDER is now evaluation semantics, not only identity:
        // the solver stack ordinal is the reverse of exactly this composed
        // pre-order, so two solvers writing one joint commit in the order
        // this loop visits them, reversed. Each segment leads with the
        // solver's full path, so a sibling reorder (or a layer-strength
        // change that composes a different order) changes the concatenation
        // and starts a new epoch -- which is the only reason a reordered
        // stack cannot keep a stale schedule. Do not "optimize" this into a
        // sorted set or a path-keyed map.
        static const TfToken kJointsRel("rigExec:joints");
        static const TfToken kJointElements("rigExec:jointElements");
        static const TfToken kCount("rigExec:count");
        static const TfToken kWeights("rigExec:weights");
        static const TfToken kSampleCount("rigExec:sampleCount");
        static const TfToken kVolumeWeights("rigExec:volumeWeights");
        for (const UsdPrim &solver : UsdPrimRange(rig)) {
            SdfPathVector joints;
            if (const UsdRelationship rel =
                    solver.GetRelationship(kJointsRel)) {
                rel.GetTargets(&joints);
            }
            // The joint discovery of the OutputSets segment reads these same
            // targets, so they are its footprint too.
            for (const SdfPath &joint : joints) {
                noteRead(joint);
            }
            // Emit for joint-bearing prims (their bindings) AND for every
            // aggregate solver even without joints: its cardinality feeds
            // Phase A element checks, possibly indirectly through a Blend
            // input, so a cardinality edit must begin a new epoch.
            const bool isAggregate =
                _IsAggregateSolverType(solver.GetTypeName());
            if (joints.empty() && !isAggregate) {
                continue;
            }
            if (isAggregate) {
                notePrim(solver.GetPath(), solver.GetTypeName());
            }
            if (!joints.empty()) {
                noteTargets(solver, kJointsRel, joints, false);
            }
            digest += pathText(solver.GetPath());
            digest += '|';
            digest += solver.GetTypeName().GetString();
            digest += '|';
            // The compiled solver DAG depends on input wiring as well as
            // output bindings. A Twist endpoint or IK control rewire must
            // replace the schedule even when cardinality is unchanged.
            if (isAggregate) {
                appendSolverInputConnections(solver);
                for (const UsdRelationship &rel : solver.GetRelationships()) {
                    const TfToken &name = rel.GetName();
                    const SdfPathVector targets =
                        appendRelTargetsNamed(solver, name, false);
                    appendFrameBindingIdentityNamed(solver, name);
                    // A read phase on a solver input orders it against the
                    // constraints above it, so it is schedule identity.
                    appendPhaseNamed(solver, name);
                    for (const SdfPath &target : targets) {
                        digest += ancestorChainToken(target.GetPrimPath());
                    }
                }
            }
            for (const SdfPath &j : joints) {
                digest += pathText(j);
                digest += ',';
            }
            // Element remap is structural: it changes which frame each
            // joint self-extracts. Parallel to joints, so not sorted.
            digest += '|';
            const UsdAttribute jeAttr = solver.GetAttribute(kJointElements);
            VtIntArray jointElements;
            if (jeAttr) {
                jeAttr.Get(&jointElements);
            }
            for (int e : jointElements) {
                digest += std::to_string(e);
                digest += ',';
            }
            if (footprint) {
                footprint->certain.jointElements.insert_or_assign(
                    solver.GetPath(),
                    _CertainFootprint::JointElements{
                        jointElements,
                        jeAttr ? jeAttr.GetNumTimeSamples() : 0});
            }
            // jointElements is a static input; record its time-sample count
            // so adding a sample post-compile re-runs Compile()'s rejection
            // rather than silently keeping the captured default (round-6).
            digest += "s" + std::to_string(
                                jeAttr ? jeAttr.GetNumTimeSamples() : 0);
            // Cardinality-determining inputs: an edit that changes how many
            // frames the solver produces must recompile so Phase A
            // re-validates every element binding. Value-only
            // edits that don't change frame count stay value-only.
            digest += "|card=";
            const TfToken stype = solver.GetTypeName();
            if (stype == "RigExecFkChain") {
                appendRelTargets(solver, "rigExec:controls", false);
                // The start-frame inference switch: flipping it rewrites
                // the derived session targets the rel loop above hashed,
                // so the token joins the digest or no recompile follows.
                appendToken(solver, "rigExec:startFramePolicy");
            } else if (stype == "RigExecTwistDistribution") {
                const UsdAttribute ca = solver.GetAttribute(kCount);
                const UsdAttribute wa = solver.GetAttribute(kWeights);
                int cnt = 1;
                if (ca) {
                    ca.Get(&cnt);
                }
                VtFloatArray w;
                if (wa) {
                    wa.Get(&w);
                }
                // Effective cardinality (weights wins, so an ignored count
                // value does not churn the epoch) plus the
                // time-sample presence of BOTH attrs so that ADDING a
                // sample without changing the default still changes the
                // digest, forcing the recompile that re-runs the pre-pass
                // sample rejection.
                const size_t effective =
                    !w.empty() ? w.size()
                               : static_cast<size_t>(std::max(cnt, 1));
                digest += std::to_string(effective) + "/" +
                          std::to_string(ca ? ca.GetNumTimeSamples() : 0) +
                          "/" +
                          std::to_string(wa ? wa.GetNumTimeSamples() : 0) +
                          ",";
            } else if (stype == "RigExecRibbon") {
                const UsdAttribute a = solver.GetAttribute(kSampleCount);
                int sc = 5;
                if (a) {
                    a.Get(&sc);
                }
                digest += std::to_string(sc) + "/" +
                          std::to_string(a ? a.GetNumTimeSamples() : 0) + ",";
                // The driver curve lowers (Pass 1.5) to generated
                // resolvedDriverPoints + a bind-time restDriverPoints
                // capture, so rewiring it (or a layer-mute/variant switch
                // that retargets it) is structural.
                appendRelTargets(solver, "rigExec:driverCurve", false);
            } else if (stype == "RigExecBlendPointFrames") {
                appendRelTargets(solver, "rigExec:inputA", false);
                appendRelTargets(solver, "rigExec:inputB", false);
            } else if (stype == "RigExecSplineIk") {
                // Cardinality is the joints list, hashed above. The
                // per-joint volume weights are a static parallel array
                // whose length Compile() validates; hash the length and
                // the sample presence so an edit that breaks the parallel
                // shape (or samples the attribute) re-runs that check.
                const UsdAttribute wa = solver.GetAttribute(kVolumeWeights);
                VtFloatArray w;
                if (wa) {
                    wa.Get(&w);
                }
                digest += std::to_string(w.size()) + "/" +
                          std::to_string(wa ? wa.GetNumTimeSamples() : 0) +
                          ",";
            }
            digest += ';';
        }
    }

    if (segments & _DigestSolvers) {
        // Native expression schemas can read posed frames via Exec
        // relationships. Retargeting those reads changes the pose schedule.
        if (rig) for (const UsdPrim &expression : UsdPrimRange(rig)) {
            if (!expression.GetRelationship(TfToken("rigExec:poseInputs"))) continue;
            digest += pathText(expression.GetPath());
            const auto inputs = appendRelTargets(expression, "rigExec:poseInputs", false);
            appendSolverInputConnections(expression);
            for (const SdfPath &input : inputs) {
                digest += ancestorChainToken(input);
            }
        }
        stampDigestRegion("Digest.Solvers");
    }
    // The whole rig, the same walk Compile's mover discovery takes: a mover
    // is found by carrying rigExec:moves, wherever it sits. (Its own
    // handle rather than the solver segment's `rig`, which is only set when
    // that segment is being computed.)
    const UsdPrim moverRig = (segments & _DigestMovers)
        ? _stage->GetPrimAtPath(_rigPath)
        : UsdPrim();
    if (moverRig) {
        for (const UsdPrim &prim : _GetMoverExecutionOrder(moverRig)) {
            const UsdRelationship moves = prim.GetRelationship(_movesRel);
            if (!moves) {
                continue;
            }
            digest += pathText(prim.GetPath());
            digest += '|';
            digest += prim.GetTypeName().GetString();
            digest += '|';
            SdfPathVector targets;
            moves.GetTargets(&targets);
            noteTargets(prim, _movesRel, targets, true);
            // Target-list order is non-semantic (spec §4.2): sort before
            // hashing so a permutation does not change the epoch.
            std::sort(targets.begin(), targets.end());
            for (const SdfPath &t : targets) {
                noteRead(t);
                const SdfPath canonical = t;
                digest += pathText(canonical);
                digest += ',';
                // Derived synthesis identity (spec §7.6 revised): whether
                // a written points target's gprim authors the derived
                // properties decides what the compiler synthesizes, so
                // authoring or removing them is a structural edit.
                if (canonical.IsPropertyPath() &&
                    canonical.GetNameToken() == "points") {
                    const SdfPath owner = canonical.GetPrimPath();
                    // Point-domain cardinality is frozen descriptor shape.
                    // Hash every authored cardinality (not the point values)
                    // so a 3 -> 2 target edit rebuilds the epoch and lets the
                    // compile validator reject a now-mismatched dense field.
                    // A set avoids recompiling merely because another sample
                    // with the same frozen cardinality was authored.
                    std::set<size_t> cardinalities;
                    if (const UsdAttribute points =
                            _stage->GetAttributeAtPath(canonical)) {
                        VtVec3fArray value;
                        bool resolved = false;
                        if (points.GetResolveInfo(UsdTimeCode::Default())
                                .GetSource() ==
                                UsdResolveInfoSourceDefault &&
                            points.Get(&value, UsdTimeCode::Default())) {
                            cardinalities.insert(value.size());
                            resolved = true;
                        }
                        std::vector<double> times;
                        points.GetTimeSamples(&times);
                        for (double sampleTime : times) {
                            if (points.Get(
                                    &value, UsdTimeCode(sampleTime))) {
                                cardinalities.insert(value.size());
                                resolved = true;
                            }
                        }
                        if (!resolved &&
                            points.Get(&value, UsdTimeCode::Default())) {
                            cardinalities.insert(value.size());
                        }
                    }
                    digest += "pointCardinalities=";
                    for (size_t cardinality : cardinalities) {
                        digest += std::to_string(cardinality);
                        digest += ',';
                    }
                    digest += '|';
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
            appendRelTargets(prim, "rigExec:transformSpace", true);
            // The carry decides a tap and a slot, so it is epoch
            // identity exactly as the measuring space is.
            appendRelTargets(prim, "rigExec:space", true);
            appendRelTargets(prim, "rigExec:referenceTransform", true);
            appendRelTargets(prim, "rigExec:referenceTransformSpace", true);
            appendRelTargets(prim, "rigExec:driverTransforms", false);
            appendRelTargets(prim, "rigExec:driverTransformSpaces", false);
            appendRelTargets(prim, "rigExec:driverBaseTransforms", false);
            appendRelTargets(prim, "rigExec:driverBaseTransformSpaces", false);
            // Influence order is semantic: jointIndices index into it.
            appendRelTargets(prim, "rigExec:influences", false);
            appendToken(prim, "rigExec:skinningMethod");
            appendToken(prim, "rigExec:operation");
            appendToken(prim, "rigExec:mode");
            appendToken(prim, "rigExec:deltaSpace");
            // Compiled into the revision: it selects the cluster's
            // point-frame correction in the fold.
            appendToken(prim, "rigExec:pointFrame");
            for (const char *input : kDigestMoverInputs) {
                appendAttributeBinding(prim, input);
            }
            // Pose-constraint wiring. Source order is semantic because every
            // source has a parallel weight (and Parent has parallel offsets),
            // so it must never be sorted. aimTarget remains the legacy
            // single-source spelling and is hashed for existing assets.
            appendRelTargets(prim, "rigExec:aimTarget", false);
            appendRelTargets(prim, "rigExec:sources", false);
            appendRelTargets(prim, "rigExec:worldUpObject", false);
            appendRelTargets(prim, "rigExec:firstJoint", false);
            appendRelTargets(prim, "rigExec:endJoint", false);
            appendRelTargets(prim, "rigExec:effector", false);
            appendRelTargets(prim, "rigExec:poleVectorObjects", false);
            if (_IsFrameConstraintType(prim.GetTypeName())) {
                appendFrameBindingIdentity(prim, "rigExec:moves");
                appendFrameBindingIdentity(prim, "rigExec:aimTarget");
                appendFrameBindingIdentity(prim, "rigExec:sources");
                appendFrameBindingIdentity(prim, "rigExec:worldUpObject");
                appendFrameBindingIdentity(prim, "rigExec:firstJoint");
                appendFrameBindingIdentity(prim, "rigExec:endJoint");
                appendFrameBindingIdentity(prim, "rigExec:effector");
                appendFrameBindingIdentity(
                    prim, "rigExec:poleVectorObjects");
                appendToken(prim, "rigExec:rotationOrder");
                appendToken(prim, "rigExec:worldUpType");
                appendToken(prim, "rigExec:aimAxis");
                appendToken(prim, "rigExec:upPolicy");
                appendToken(prim, "rigExec:solverMode");
                appendToken(prim, "rigExec:poleVectorMode");
                appendToken(prim, "rigExec:evaluationMode");
                appendToken(prim, "rigExec:orientationMode");
                // Uniform opt-ins compiled into the constraint record.
                appendScalar(prim, "rigExec:blendShear");
                appendScalar(prim, "rigExec:worldUpRotationOnly");
            }
            // Static-input relationships captured at compile into generated
            // resolved*/rest* wiring (lattice cage, surface, curve bind/
            // driver): retargeting any of these must recompile so the
            // captured bind-time values are refreshed. Hashed in AUTHORED
            // order (sorted=false) because the compiler consumes targets[0], so
            // a reorder that changes the selected input must change the
            // digest. An absent rel appends a constant
            // empty marker (harmless, invariant per mover type).
            appendRelTargets(prim, "rigExec:cage", false);
            appendRelTargets(prim, "rigExec:surface", false);
            appendRelTargets(prim, "rigExec:bindCoordinates", false);
            for (const char *phased : {"rigExec:transform",
                                       "rigExec:influences",
                                       "rigExec:driverTransforms",
                                       "rigExec:cage", "rigExec:surface",
                                       "rigExec:bindCoordinates",
                                       "rigExec:driverCurve"}) {
                appendPhase(prim, phased);
            }
            appendRelTargets(prim, "rigExec:driverFrames", false);
            appendRelTargets(prim, "rigExec:driverCurve", false);
            if (const RigExecMoverHandler *handler =
                    RigExecFindMoverHandler(prim.GetTypeName());
                handler && handler->assembleExternal) {
                // Plugin bindings may use relationships unknown to core.
                // Their target order and read phases are compiled wiring.
                for (const UsdRelationship &rel : prim.GetRelationships()) {
                    if (rel.GetName() == _movesRel) continue;
                    appendRelTargetsNamed(prim, rel.GetName(), false);
                    appendPhaseNamed(prim, rel.GetName());
                }
            }
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
                for (const char *field : kDigestBlendInputFields) {
                    appendAttributeBinding(input, field);
                }
                for (const SdfPath &samplePath :
                     appendRelTargets(input, "rigExec:samples", true)) {
                    const UsdPrim sample =
                        _stage->GetPrimAtPath(samplePath.GetPrimPath());
                    if (!sample) {
                        continue;
                    }
                    appendRelTargets(sample, "rigExec:targetPoints", true);
                    appendPhase(sample, "rigExec:targetPoints");
                    // WHICH blend shape a sparse sample names is structure,
                    // so it belongs in the epoch digest. What the shape
                    // CONTAINS deliberately does not: offsets and
                    // pointIndices are uniform, so they cannot be time
                    // samples, and RigExecBlendSampleCache is cleared by
                    // every notice -- a sculpt edit is caught by the cache's
                    // array compare on the next frame without forcing a
                    // recompile of the whole rig.
                    appendRelTargets(sample, "rigExec:blendShape", true);
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
    if (segments & _DigestMovers) {
        stampDigestRegion("Digest.Movers");
    }
    return digest;
}

size_t
RigExecRigEvaluator::_JoinStructureDigest(
    const std::string (&parts)[_DigestSegmentCount]) const
{
    // Segment order is the order one serial walk emits them in, so the
    // concatenation is that walk's text and hashes to the digest it did.
    std::string digest;
    digest.reserve(parts[0].size() + parts[1].size() + parts[2].size());
    for (const std::string &part : parts) {
        digest += part;
    }
    if (_DigestMemoVerifyRequested() && _stage) {
        const std::string unmemoized =
            _ComputeStructureDigest(_DigestSolvers | _DigestUnmemoizedReads);
        if (unmemoized != parts[1]) {
            TF_FATAL_ERROR(
                "RIGEXEC_VERIFY_DIGEST_MEMO: the memoized Solvers digest "
                "segment of <%s> differs from the unmemoized one at byte %zu "
                "(memoized %zu bytes, unmemoized %zu)",
                _rigPath.GetText(),
                _FirstDigestDifference(unmemoized, parts[1]),
                parts[1].size(), unmemoized.size());
        }
    }
    if (_DigestSplitVerifyRequested() && _stage) {
        const std::string whole = _ComputeStructureDigest(_DigestAllSegments);
        if (whole != digest) {
            const size_t at = _FirstDigestDifference(whole, digest);
            TF_FATAL_ERROR(
                "RIGEXEC_VERIFY_DIGEST_SPLIT: the split structure digest of "
                "<%s> differs from the whole one at byte %zu (whole %zu "
                "bytes; segments %zu + %zu + %zu)",
                _rigPath.GetText(), at, whole.size(), parts[0].size(),
                parts[1].size(), parts[2].size());
        }
    }
    return std::hash<std::string>{}(digest);
}

size_t
RigExecRigEvaluator::_SettleStructureDigest(_DigestGate *gate) const
{
    // Called on the thread that evaluates, which can be a Python caller
    // holding the GIL or a worker inside someone else's parallel region:
    // the scoped dispatcher drops the GIL and isolates the wait, so this
    // thread helps with its own three tasks and with nothing else.
    std::string parts[_DigestSegmentCount];
    _DigestFootprint footprints[_DigestSegmentCount];
    const auto computePart = [this, &parts, &footprints, gate](size_t i) {
        parts[i] = _ComputeStructureDigest(1u << i,
                                           gate ? &footprints[i] : nullptr);
    };
    if (RigExecParallelEvaluationEnabled()) {
        WorkWithScopedDispatcher([&computePart](WorkDispatcher &dispatcher) {
            // Solvers first: it is the long one.
            for (const size_t i : {size_t(1), size_t(2), size_t(0)}) {
                dispatcher.Run([&computePart, i]() { computePart(i); });
            }
        });
    } else {
        for (size_t i = 0; i < _DigestSegmentCount; ++i) {
            computePart(i);
        }
    }
    if (gate) {
        *gate = _MakeDigestGate(footprints);
    }
    return _JoinStructureDigest(parts);
}

} // namespace rigExec
