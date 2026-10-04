// Dynamic evaluation: pose revisions followed by geometry revisions.

#include "rigEvaluatorInternal.h"
#include "rigExecMath/geometryKernels.h"
#include "rigEvaluatorConstraints.h"
#include "parallel.h"
#include "frameExtraction.h"
#include "solverKernels.h"
#include "rigExecMath/singleChainIk.h"
#include "rigExecMath/solvers.h"

#include "pxr/base/gf/rotation.h"
#include "pxr/base/work/dispatcher.h"
#include "pxr/base/tf/pyLock.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usdGeom/metrics.h"
#include "pxr/usd/usdGeom/xformCache.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <set>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

namespace rigExec {

using namespace evaluatorDetail;

namespace {

// Attribute names and values the pose walk reads per constraint per frame,
// interned once. A per-frame TfToken construction takes the token registry
// lock, so the walk spells each of these once rather than per call.
const TfToken _kDynEnabled("inputs:enabled");
const TfToken _kDynDefaultWeight("inputs:defaultWeight");
const TfToken _kDynSourceWeights("inputs:sourceWeights");
const TfToken _kDynTranslationOffsets("inputs:translationOffsets");
const TfToken _kDynRotationOffsets("inputs:rotationOffsets");
const TfToken _kDynPoleVectorWeights("inputs:poleVectorWeights");
const TfToken _kDynPoleVector("inputs:poleVector");
const TfToken _kDynTwistDegrees("inputs:twistDegrees");
const TfToken _kDynAimVector("inputs:aimVector");
const TfToken _kDynUpVector("inputs:upVector");
const TfToken _kDynRotationOffset("inputs:rotationOffset");
const TfToken _kDynWorldUpVector("inputs:worldUpVector");
const TfToken _kDynRotationOrder("rigExec:rotationOrder");
const TfToken _kDynOrientationMode("rigExec:orientationMode");
const TfToken _kDynSolverMode("rigExec:solverMode");
const TfToken _kDynPoleVectorMode("rigExec:poleVectorMode");
const TfToken _kDynEvaluationMode("rigExec:evaluationMode");
const TfToken _kDynAimAxis("rigExec:aimAxis");
const TfToken _kDynWorldUpType("rigExec:worldUpType");
const TfToken _kDynSources("rigExec:sources");
const TfToken _kDynComputeDriverPoints("rigExec:computeDriverPoints");
const TfToken _kDynComputeRestDriverPoints("rigExec:computeRestDriverPoints");
const TfToken _kDynXyz("XYZ");
const TfToken _kDynAimX("aimX");
const TfToken _kDynPreserve("preserve");
const TfToken _kDynRotatePlane("rotatePlane");
const TfToken _kDynSingleChain("singleChain");
const TfToken _kDynVector("vector");
const TfToken _kDynObject("object");
const TfToken _kDynNeverTs("neverTS");
const TfToken _kDynAlwaysTs("alwaysTS");
const TfToken _kDynAutoDetect("autoDetect");
const TfToken _kDynAxisX("x");
const TfToken _kDynAxisY("y");
const TfToken _kDynAxisZ("z");
const TfToken _kDynWorldUpNone("none");
const TfToken _kDynWorldUpSceneUp("sceneUp");
const TfToken _kDynWorldUpObjectUp("objectUp");
const TfToken _kDynWorldUpObjectRotationUp("objectRotationUp");
const TfToken _kDynParentConstraint("RigExecParentConstraint");
const TfToken _kDynSingleChainIkConstraint("RigExecSingleChainIkConstraint");
const TfToken _kDynParentSpace("parent:space");
const TfToken _kDynComputeMatrix("computeMatrix");

} // namespace

RigExecRigPose
RigExecRigEvaluator::_EvaluateDynamic(UsdTimeCode time,
                                      std::vector<std::string> diagnostics)
{
    RigExecRigPose pose;
    pose.time = time;
    pose.diagnostics = std::move(diagnostics);
    if (!_SettleEpoch(&pose.diagnostics)) {
        return pose;
    }
    // This is the one path that pulls the deferred requests, and the first
    // line of it that could: everything below reads a snapshot from one of
    // them. After _SettleEpoch, because settling may compile a new epoch --
    // and it is that epoch's requests, not the retired one's, that are owed
    // preparing.
    if (_execPrepDeferred && !_RealizeDeferredExecPrep(&pose)) {
        return pose;
    }
    // The epoch's rest frames, when an edit reached a rest path and the
    // settle left the re-pull to the walk that reads them (see _SettleEpoch).
    // A failed re-pull leaves them stale, so the next dynamic frame asks
    // again.
    if (_epochRestFramesStale && !_RefreshEpochRestFrames()) {
        pose.diagnostics.push_back("rest frame evaluation incomplete");
        return pose;
    }

    // Region stamps for the stretches that cannot take an RAII scope,
    // because a scope needs a block and the block would scope out the
    // lambdas the rest of the walk calls.
    // MEASURED 2026-09-13, biped: 3.3-3.9 ms/frame of this function sat
    // inside no profiler scope at all -- 24% of a no-change evaluate, more
    // than AuthoritativeSnapshot. The publish loops turned out to be only
    // 1.2 ms of that; the rest is here, in the per-provider frame maps that
    // are rebuilt from scratch every generation. Instrumenting first is what
    // kept a design from being written against the wrong 2.5 ms.
    const bool profileRegions = _profiler.IsEnabled();
    uint64_t regionStart = profileRegions ? RigExecProfiler::NowUs() : 0;
    auto stampRegion = [&](const char *name) {
        if (!profileRegions) {
            return;
        }
        const uint64_t now = RigExecProfiler::NowUs();
        _profiler.Record(name, "evaluate", regionStart, now);
        regionStart = now;
    };

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

    // Property chains resolve FIRST, and their results ride in as attribute
    // overrides on every request below.
    // This is the whole point of evaluating them off the authored stage: a
    // math mover's inputs are all authored on itself, so its chain owes exec
    // nothing and can be computed before exec runs -- which means the value
    // it produces can be handed to exec as the attribute's value. A clamped
    // IK/FK weight then reaches RigExecBlendPointFrames as the weight it
    // reads, instead of that kernel reimplementing the author's clamp
    // internally and the authored mover meaning nothing.
    // No cycle is possible: nothing in a property chain reads a computation.
    _resolvedInputs.Clear();
    _chainSnapshots.Clear();

    // Interactive overrides are applied once, before the property chains,
    // and after the clear above. One on a value a chain READS -- a control
    // avar feeding a math mover -- is the value the chain computes from.
    // One on a property a chain WRITES is that chain's base: the chain
    // revises it as it would the value authored there and publishes its own
    // result in the override's place, so every reader sees during the drag
    // what it sees once the value is authored (_EvaluatePropertyChains).
    if (!_interactiveOverrides.empty()) {
        _ApplyInteractiveOverrides(
            _interactiveOverrides, &baseOverrides, &_resolvedInputs);
    }
    if (!_propertyChains.empty()) {
        RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "PropertyChains", "property");
        _EvaluatePropertyChains(time, &pose.movedProperties, &baseOverrides,
                                &pose.diagnostics);
        // Two delivery routes for one value, and they must not disagree:
        // baseOverrides carries it to every exec consumer; _resolvedInputs
        // carries it to the static reads exec never touches (packet
        // assembly, CPU oracle). _EvaluatePropertyChains writes both routes
        // for every published chain, so no re-sync loop is needed here.
    }

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
            ribbonPath, _kDynComputeDriverPoints, TfToken(),
            VtValue(livePacket)});
        baseOverrides.push_back(RigExecValueOverride{
            ribbonPath, _kDynComputeRestDriverPoints, TfToken(),
            VtValue(restPacket)});
    }

    // Joints whose solver published no element for them (an incomplete
    // solver: its required inputs are unwired, so the kernel returned an
    // empty aggregate). They keep their natural rest-chain frame below, so
    // a rig mid-edit stays visible instead of vanishing.
    // (joint, (solver, element)) in POSE-WALK order, because under a stack
    // the joint is not enough: several solvers may write it and the one that
    // failed is not necessarily the one a joint->solver lookup would name.
    // The verdict beside it is the other half -- a joint another writer DID
    // publish kept that writer's frame and fell back to nothing at all, so
    // the rest-chain sentence would simply be false. The baked path builds
    // both the same way, from the same walk order, and sorts them by the
    // same key, because the two diagnostic streams are compared verbatim.
    std::vector<std::pair<SdfPath, std::pair<SdfPath, int>>> fallbackJoints;
    std::map<SdfPath, SdfPath> lastPublishingWriter;
    std::map<SdfPath, RigExecPointFrameArray> solvedAggregates;
    RigExecSnapshot seedSnapshot;
    bool seedFromCache = false;
    if (_firstFramePoseTaps) {
        RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "FirstFramePose", "pose");
        // Warm the seed executor: override pulls use a throwaway sub-executor
        // seeded from its main cache. Each connected-request partition warms
        // its own cache in Evaluate; a cold cache repeats upstream work.
        _firstFramePoseTaps->Warm(time);
        // The tap-level dirty flag is drained, never consulted: exec's
        // time/value callbacks fire across sibling frames sharing this
        // system and would spuriously veto fresh entries. Genuine stage
        // edits set _firstFramePoseDirty through the notice handler.
        _firstFramePoseTaps->ConsumeDirty();
        _SnapshotCache::Entry *seed =
            !_firstFramePoseDirty
                ? _firstFramePoseCache.Find(baseOverrides, time)
                : nullptr;
        if (seed != nullptr) {
            seedSnapshot = seed->snapshot;
            seedFromCache = true;
        } else {
            seedSnapshot = _firstFramePoseTaps->Evaluate(time, baseOverrides);
            if (!seedSnapshot.IsValid() || !seedSnapshot.IsComplete()) {
                _firstFramePoseDirty = true;
                pose.diagnostics.push_back("pose provider input evaluation incomplete");
                return pose;
            }
            _firstFramePoseCache.Store(baseOverrides, time, seedSnapshot);
            _firstFramePoseDirty = false;
        }
    }

    // 1b. Space switches. Each one replaces its target's composed frame with
    // the same compose taken against another parent, and republishes it as a
    // value override so that every later reader -- the target's namespace
    // children through exec, the solvers through their frame inputs, the
    // manipulator through the frames this function returns -- sees one
    // answer. See _SpaceSwitch for the algebra.
    if (!_spaceSwitches.empty() && _firstFramePoseTaps &&
        seedSnapshot.IsValid()) {
        RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "SpaceSwitches", "pose");
        const auto matrixOf = [&](RigExecTapId tap, bool *ok) {
            GfMatrix4d m(1.0);
            if (tap < 0) {
                if (ok) *ok = true;
                return m;
            }
            const RigExecPointFrame frame =
                seedSnapshot.Get<RigExecPointFrame>(tap);
            const bool valid = frame.IsValid() && !frame.IsDegenerate() &&
                RigExecPointsToMatrix(RigExecIdentityLandmarks(),
                                      frame.points, &m);
            if (ok) *ok = valid;
            return valid ? m : GfMatrix4d(1.0);
        };
        std::vector<RigExecValueOverride> switchOverrides;
        // Where the base overrides end and this pass's begin. Each band
        // REPLACES the switch overrides rather than appending to them, so a
        // band that re-resolves an earlier band's switch cannot leave two
        // opinions about one target standing.
        const size_t baseCount = baseOverrides.size();
        size_t published = 0;
        // Band by band: everything in a band resolves from the seed standing
        // now, and the seed is pulled again before the next one. The
        // re-evaluation is what makes a descendant of a switched control
        // read correctly here -- exec recomposes it from the override,
        // which is the same arithmetic the baked compose performs.
        size_t begin = 0;
        while (begin < _spaceSwitches.size()) {
            size_t end = begin;
            while (end < _spaceSwitches.size() &&
                   _spaceSwitches[end].band == _spaceSwitches[begin].band) {
                ++end;
            }
        for (size_t i = begin; i < end; ++i) {
            const _SpaceSwitch &sw = _spaceSwitches[i];
            bool ok = true, partOk = true;
            const GfMatrix4d targetWorld =
                matrixOf(sw.targetPosedTap, &partOk);
            ok = ok && partOk;
            const GfMatrix4d parentPosed =
                matrixOf(sw.parentPosedTap, &partOk);
            ok = ok && partOk;
            const GfMatrix4d parentDefault =
                matrixOf(sw.parentDefaultTap, &partOk);
            ok = ok && partOk;
            if (!ok) {
                pose.diagnostics.push_back(
                    sw.switchPath.GetString() +
                    " has no usable frame for its target " +
                    sw.target.GetString() + "; the space is not applied");
                continue;
            }
            // avars * default:space, with exec's own answer for both.
            const GfMatrix4d local =
                targetWorld * parentPosed.GetInverse() * parentDefault;

            // Read per frame, not captured at compile: the active index is
            // the animated channel, and the digest deliberately does not
            // hash it so that keying a space never rebuilds the epoch.
            double active = sw.activeFallback;
            if (const UsdPrim switchPrim =
                    _stage->GetPrimAtPath(sw.switchPath)) {
                active = _ResolvedRead(_resolvedInputs, switchPrim,
                                       "inputs:activeSpace",
                                       sw.activeFallback, time);
            }
            if (!sw.activeAttribute.IsEmpty()) {
                const UsdAttribute attribute =
                    _stage->GetAttributeAtPath(sw.activeAttribute);
                double value = 0.0;
                float asFloat = 0.0f;
                TfToken asToken;
                // THE TOKEN IS TRIED FIRST, and deliberately. The numeric
                // reads answer TRUE for a token-valued attribute -- with
                // a zero -- so asking them first pins every named space
                // to source 0 and the switch stops responding at all,
                // silently. The declared type decides, not the order a
                // reader happens to guess in.
                const bool isToken =
                    attribute.GetTypeName() == SdfValueTypeNames->Token;
                if (isToken &&
                    (_resolvedInputs.GetAttribute(attribute, time, &asToken) ||
                     attribute.Get(&asToken, time))) {
                    // A SPACE NAMED RATHER THAN NUMBERED. The labels are
                    // already the switch's own, so the name an animator
                    // sets is the name the rig published -- no index to
                    // keep in step with the source order, and a source
                    // inserted in the middle cannot silently repoint
                    // every control that named a space after it.
                    //
                    // An index still reads as one, so a rig that never
                    // migrates keeps working, and a numeric channel can
                    // still blend between neighbours. A name is exact by
                    // construction: there is no halfway between two of
                    // them.
                    bool matched = false;
                    for (size_t i = 0; i < sw.labels.size(); ++i) {
                        if (sw.labels[i] == asToken.GetString()) {
                            active = double(i);
                            matched = true;
                            break;
                        }
                    }
                    if (!matched && !asToken.IsEmpty()) {
                        pose.diagnostics.push_back(
                            sw.switchPath.GetString() + " has no space named " +
                            asToken.GetString() + "; holding " +
                            (sw.labels.empty()
                                 ? std::string("source 0")
                                 : sw.labels[0]));
                    }
                } else if (_resolvedInputs.GetAttribute(attribute, time,
                                                        &value) ||
                           attribute.Get(&value, time)) {
                    active = value;
                } else if (_resolvedInputs.GetAttribute(attribute, time,
                                                        &asFloat) ||
                           attribute.Get(&asFloat, time)) {
                    active = double(asFloat);
                }
            }
            const int count = int(sw.sources.size());
            if (!std::isfinite(active)) active = 0.0;
            active = GfClamp(active, 0.0, double(count - 1));
            const int lower = int(std::floor(active));
            const int upper = std::min(lower + 1, count - 1);
            const double blend = active - double(lower);

            // The space's own delta, expressed in the TARGET's local frame:
            //     L = (avars * D) * inverse(source default) * source posed
            //         * inverse(avars * D)
            // so that world = L * (avars * D). Masking L is therefore masking
            // in the control's own axes, which is what makes a translation
            // mask read as "orient only about my own pivot".

            // rigExec:space. `local` above is composed over the target's
            // DEFAULT ancestors, so a master's motion reaches a switched
            // control only inside the source's motion -- and a twist or
            // swing filter throws that carry away along with the part it
            // was asked to drop. carryInverse strips the master map off
            // before the filter runs and carry puts it back after, so the
            // filter only ever sees the source's own master-free motion.
            // Identity when no space is named: the pre-masters behaviour.
            // hasCarry, not "carry == identity": a rig that names no space
            // must take the branches below UNTOUCHED, not multiplied by an
            // identity. `local * identity * local.GetInverse()` is identity
            // in exact arithmetic and a few ulps off it in doubles, and
            // that difference reaches verify_binary as a failure.
            GfMatrix4d carry(1.0), carryInverse(1.0);
            bool hasCarry = false;
            if (sw.spacePosedTap >= 0 && sw.spaceDefaultTap >= 0) {
                bool spacePosedOk = true, spaceDefaultOk = true;
                const GfMatrix4d spacePosed =
                    matrixOf(sw.spacePosedTap, &spacePosedOk);
                const GfMatrix4d spaceDefault =
                    matrixOf(sw.spaceDefaultTap, &spaceDefaultOk);
                if (spacePosedOk && spaceDefaultOk) {
                    carry = spaceDefault.GetInverse() * spacePosed;
                    carryInverse = carry.GetInverse();
                    hasCarry = true;
                }
            }
            const auto rawDeltaOf = [&](int index, bool *valid) {
                const _SpaceSwitch::Source &source = sw.sources[size_t(index)];
                *valid = true;
                if (source.path.IsEmpty()) {
                    // World: the source never moves, so the only motion
                    // left is the space's own carry -- and with no space
                    // named there is none, which is the pin-to-zero-pose
                    // this branch has always meant.
                    return hasCarry ? local * carry * local.GetInverse()
                                    : GfMatrix4d(1.0);
                }
                // Both come from the seed standing NOW, which is why bands
                // exist: a source that another switch moves was already
                // republished into this seed before this band was reached.
                bool frameOk = true;
                const GfMatrix4d posed = matrixOf(source.posedTap, &frameOk);
                const GfMatrix4d sourceDefault =
                    matrixOf(source.defaultTap, valid);
                *valid = *valid && frameOk;
                // The source's own motion from its rest, in world, filtered
                // before it is expressed in the target's frame: the twist
                // axis is the SOURCE's, so the split has to happen while the
                // motion is still measured against the source's rest.
                const GfMatrix4d moved = sourceDefault.GetInverse() * posed;
                const GfMatrix4d motion =
                    hasCarry
                        ? RigExecFilterSpaceRotation(
                              moved * carryInverse,
                              sourceDefault.TransformDir(sw.twistAxis),
                              source.filter) * carry
                        : RigExecFilterSpaceRotation(
                              moved,
                              sourceDefault.TransformDir(sw.twistAxis),
                              source.filter);
                return local * motion * local.GetInverse();
            };
            // An `orient` source turns the control with the space and keeps
            // it where its namespace parent carries it.
            const auto deltaOf = [&](int index, bool *valid) {
                const GfMatrix4d d = rawDeltaOf(index, valid);
                if (sw.sources[size_t(index)].filter !=
                    RigExecRotationFilter::Orient) {
                    return d;
                }
                return RigExecOrientSpaceDelta(d, local, local.GetInverse(),
                                               targetWorld);
            };
            bool lowerOk = true, upperOk = true;
            GfMatrix4d delta = deltaOf(lower, &lowerOk);
            if (upper != lower && blend > 0.0) {
                const GfMatrix4d other = deltaOf(upper, &upperOk);
                delta = RigExecBlendTransforms(delta, other, blend);
            }
            if (!lowerOk || !upperOk) {
                pose.diagnostics.push_back(
                    sw.switchPath.GetString() +
                    " has no usable frame for a space source; the space is "
                    "not applied");
                continue;
            }
            delta = RigExecMaskTransform(delta, sw.affectTranslation,
                                         sw.affectRotation, sw.affectScale);
            switchOverrides.push_back(RigExecValueOverride{
                sw.target, _computePointFrame, TfToken(),
                VtValue(RigExecFrameFromMatrix(delta * local))});
        }
            // The seed is pulled again between bands so that the next band
            // reads the answers this one published, and once more after the
            // last band so that every later reader -- the target's namespace
            // children, the solvers, the frames this function returns --
            // sees one answer. A rig with one band therefore pays exactly
            // one extra pull, which is the common case.
            begin = end;
            if (switchOverrides.size() == published) {
                // Nothing new this band: every switch in it declined.
                continue;
            }
            baseOverrides.resize(baseCount);
            baseOverrides.insert(baseOverrides.end(), switchOverrides.begin(),
                                 switchOverrides.end());
            published = switchOverrides.size();
            const bool sameSwitch = seedFromCache && end >= _spaceSwitches.size() &&
                _spaceSwitchSnapshot.IsValid() &&
                switchOverrides.size() == _lastSpaceSwitchOverrides.size() &&
                std::equal(switchOverrides.begin(), switchOverrides.end(),
                           _lastSpaceSwitchOverrides.begin(),
                           [](const RigExecValueOverride &a,
                              const RigExecValueOverride &b) {
                               return a.prim == b.prim && a.value == b.value;
                           });
            if (sameSwitch) {
                seedSnapshot = _spaceSwitchSnapshot;
                continue;
            }
            // A switched leaf with no pose consumers cannot affect any
            // other seed value. Replace its direct frame taps in a copy;
            // keep all overrides for subsequent requests as usual.
            std::unordered_map<SdfPath, VtValue, SdfPath::Hash> leafValues;
            bool leavesOnly = true;
            for (const auto &value : switchOverrides) {
                const auto provider = _providerIndex.find(value.prim);
                if (provider == _providerIndex.end() ||
                    !_poseRefreshDependents[provider->second].empty() ||
                    !_hierDescendants[provider->second].empty()) {
                    leavesOnly = false;
                    break;
                }
                leafValues[value.prim] = value.value;
            }
            if (leavesOnly) {
                for (size_t tap = 0; tap < seedSnapshot._values.size(); ++tap) {
                    const auto &address = _firstFramePoseTaps->GetAddress(tap);
                    if (leafValues.count(address.target.GetPrimPath()) &&
                        (address.target.IsPropertyPath() ||
                         address.publicComputation != _computePointFrame)) {
                        leavesOnly = false;
                        break;
                    }
                }
            }
            RigExecSnapshot switched;
            if (leavesOnly) {
                RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "SpaceSwitches.LeafSnapshot", "pose");
                switched = seedSnapshot;
                for (size_t tap = 0; tap < switched._values.size(); ++tap) {
                    const auto &address = _firstFramePoseTaps->GetAddress(tap);
                    const auto value = leafValues.find(address.target);
                    if (value != leafValues.end()) switched._values[tap] = value->second;
                }
            } else {
                switched = _firstFramePoseTaps->Evaluate(time, baseOverrides);
            }
            if (!switched.IsValid() || !switched.IsComplete()) {
                pose.diagnostics.push_back(
                    "space switch evaluation incomplete");
                return pose;
            }
            seedSnapshot = switched;
            if (begin >= _spaceSwitches.size()) {
                _spaceSwitchSnapshot = switched;
                _lastSpaceSwitchOverrides = switchOverrides;
            }
        }
    }

    // 1c. Auto clavicles. After every switch, so each reads its limb's
    // controls in the spaces they are in, and republished as an override on
    // the target's computePointFrame -- the switch's own override for that
    // target when there is one, translated -- so the target's descendants
    // and the solvers read the moved frame, as they read a switched one.
    if (!_autoClavicles.empty() && _firstFramePoseTaps &&
        seedSnapshot.IsValid()) {
        RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "AutoClavicles", "pose");
        const auto frameOf = [&](RigExecTapId tap, GfMatrix4d *m) {
            if (tap < 0) return false;
            const RigExecPointFrame frame =
                seedSnapshot.Get<RigExecPointFrame>(tap);
            return frame.IsValid() && !frame.IsDegenerate() &&
                   RigExecPointsToMatrix(RigExecIdentityLandmarks(),
                                         frame.points, m);
        };
        const auto readScalar = [&](const SdfPath &attrPath,
                                    double fallback) {
            if (attrPath.IsEmpty()) return fallback;
            const UsdAttribute attribute =
                _stage->GetAttributeAtPath(attrPath);
            double value = 0.0;
            float asFloat = 0.0f;
            if (!attribute) return fallback;
            if (_resolvedInputs.GetAttribute(attribute, time, &value) ||
                attribute.Get(&value, time)) {
                return value;
            }
            if (_resolvedInputs.GetAttribute(attribute, time, &asFloat) ||
                attribute.Get(&asFloat, time)) {
                return double(asFloat);
            }
            return fallback;
        };
        std::vector<RigExecValueOverride> moved;
        for (const _AutoClavicle &ac : _autoClavicles) {
            GfMatrix4d target, pivot, anchor, anchorRest, fk, fkRest[3],
                ikTarget, pole;
            bool ok = frameOf(ac.targetPosedTap, &target) &&
                      frameOf(ac.pivotPosedTap, &pivot) &&
                      frameOf(ac.anchorPosedTap, &anchor) &&
                      frameOf(ac.anchorDefaultTap, &anchorRest) &&
                      frameOf(ac.fkPosedTap, &fk);
            for (int i = 0; i < 3; ++i) {
                ok = ok && frameOf(ac.fkDefaultTap[i], &fkRest[i]);
            }
            if (!ok) {
                pose.diagnostics.push_back(
                    ac.nodePath.GetString() +
                    " has no usable frame for its limb; the clavicle is not "
                    "carried");
                continue;
            }
            RigExecAutoClavicleFrames f;
            f.anchorPosed = anchor.data();
            f.anchorDefault = anchorRest.data();
            f.pivotPosed = pivot.data();
            f.targetPosed = target.data();
            f.fkPosed = fk.data();
            for (int i = 0; i < 3; ++i) f.fkDefault[i] = fkRest[i].data();
            if (frameOf(ac.ikTargetPosedTap, &ikTarget)) {
                f.ikTargetPosed = ikTarget.data();
            }
            if (frameOf(ac.polePosedTap, &pole)) f.polePosed = pole.data();
            f.ikBlend =
                readScalar(ac.ikBlendAttribute, 1.0 - ac.constants.ikValue);
            f.amount = readScalar(ac.amountAttribute, 1.0);
            if (!ac.limbSolver.IsEmpty()) {
                const auto input = [&](const char *name, double fallback) {
                    return readScalar(
                        ac.limbSolver.AppendProperty(TfToken(name)), fallback);
                };
                f.hasLimb = true;
                f.limb.stretch = input("inputs:stretch", 1.0);
                f.limb.pin = input("inputs:pin", 0.0);
                f.limb.upperScale = input("inputs:upperScale", 1.0);
                f.limb.lowerScale = input("inputs:lowerScale", 1.0);
                f.limb.softDistance = input("inputs:softDistance", 0.0);
                f.limb.scaleCalibration =
                    input("rigExec:scaleCalibration", 0.0);
                f.twistRadians =
                    input("inputs:twist", 0.0) * std::acos(-1.0) / 180.0;
                GfMatrix4d rests[3];
                if (frameOf(ac.limbJointRestTap[0], &rests[0]) &&
                    frameOf(ac.limbJointRestTap[1], &rests[1]) &&
                    frameOf(ac.limbJointRestTap[2], &rests[2])) {
                    f.limbRestUpper = (rests[1].ExtractTranslation() -
                                       rests[0].ExtractTranslation())
                                          .GetLength() +
                                      input("rigExec:upperLengthOffset", 0.0);
                    f.limbRestLower = (rests[2].ExtractTranslation() -
                                       rests[1].ExtractTranslation())
                                          .GetLength() +
                                      input("rigExec:lowerLengthOffset", 0.0);
                }
            }
            double delta[3];
            RigExecAutoClavicleShift(ac.constants, f, delta);
            if (delta[0] == 0.0 && delta[1] == 0.0 && delta[2] == 0.0) {
                continue;
            }
            GfMatrix4d shifted = target;
            shifted.SetTranslateOnly(target.ExtractTranslation() +
                                     GfVec3d(delta[0], delta[1], delta[2]));
            moved.push_back(RigExecValueOverride{
                ac.target, _computePointFrame, TfToken(),
                VtValue(RigExecFrameFromMatrix(shifted))});
        }
        if (!moved.empty()) {
            const bool same = seedFromCache && _autoClavicleSnapshot.IsValid() &&
                moved == _lastAutoClavicleOverrides;
            for (const RigExecValueOverride &o : moved) {
                const auto it = std::find_if(
                    baseOverrides.begin(), baseOverrides.end(),
                    [&](const RigExecValueOverride &b) {
                        return b.prim == o.prim &&
                               b.computation == o.computation &&
                               b.attribute.IsEmpty();
                    });
                if (it != baseOverrides.end()) {
                    it->value = o.value;
                } else {
                    baseOverrides.push_back(o);
                }
            }
            if (same) {
                seedSnapshot = _autoClavicleSnapshot;
            } else {
                const RigExecSnapshot carried =
                    _firstFramePoseTaps->Evaluate(time, baseOverrides);
                if (!carried.IsValid() || !carried.IsComplete()) {
                    pose.diagnostics.push_back(
                        "auto clavicle evaluation incomplete");
                    return pose;
                }
                seedSnapshot = carried;
                _autoClavicleSnapshot = carried;
                _lastAutoClavicleOverrides = moved;
            }
        }
    }

    // 2. Pose-domain FBX-style constraints, applied in the single composed
    // mover walk. A global walk is essential for SingleChainIK: all joints in
    // its write set must be solved and committed atomically, while ordinary
    // one-provider constraints still chain in exactly the same order as every
    // points/property revision.
    std::map<SdfPath, RigExecPointFrame> baseFrames;
    std::vector<RigExecPointFrame> finalFrames(_providerPaths.size());
    std::vector<char> finalLive(_providerPaths.size(), 0);
    std::vector<RigExecPointFrame> restFrames(_providerPaths.size());
    std::vector<char> restLive(_providerPaths.size(), 0);
    std::unordered_map<SdfPath, GfMatrix4d, SdfPath::Hash> xformDerivedBases;
    std::unordered_map<SdfPath, GfMatrix4d, SdfPath::Hash> finalMatrices;
    /// Geometry-domain constraint results: the delta each one produced, the
    /// envelope it carries, and its optional per-element weight field.
    /// Produced by the pose walk below and consumed after it, the same
    /// in-memory hand-off finalMatrices performs for a "final" read phase.
    std::unordered_map<SdfPath, GfMatrix4d, SdfPath::Hash> constraintDeltas;
    const UsdPrim assetRoot =
        _stage->GetPrimAtPath(_rigPath.GetParentPath());
    UsdGeomXformCache constraintXformCache(time);

    auto frameFromXform = [&](const SdfPath &path,
                              RigExecPointFrame *out,
                              GfMatrix4d *matrix) {
        return _FrameFromXformRelativeToAsset(
            assetRoot, &constraintXformCache, path, out, matrix);
    };

    // Seed every reachable RigExec frame provider before any pose operation.
    // Solvers will replace their owned joint frames when their inputs are ready.
    // The three maps are empty here and _firstFramePoseFrames hands its keys over
    // already ordered, so each insertion goes straight to the end instead of
    // searching the tree it is building.
    // The rests are the epoch's, pulled once at Compile -- unless this epoch
    // has a rest channel that can move with time, in which case they rode in
    // on the seed snapshot with the frames.
    if (_firstFramePoseRests.empty()) {
        // ... or unless a drag is standing on a rest channel. An interactive
        // override is a value that stands in for an authored one, so it has
        // to move a rest frame exactly as authoring it would: the drag and
        // the commit of that same drag must agree, and jointMatricesFinal is
        // the rest->pose map, so a pose that moved against a rest that did
        // not is not a pose of this rig at all. The epoch pull carries no
        // overrides, so for as long as one stands the rests are pulled
        // per frame with them, through the same request and at the frame's
        // own time code -- which is exactly what the per-frame rest taps
        // used to do.
        // Only a rest input can do it: computeRestFrame reads the seven
        // names below on the provider and on its RigExec ancestors and
        // nothing else (see _RestInputNames), and this epoch has no rest
        // channel with an authored connection -- _ProviderRestMightVary
        // refuses the epoch path outright when one does -- so no override on
        // any other attribute can reach a rest frame. A computation override
        // names a computation this cannot inspect, so it counts.
        const bool restOverridden = [this]() {
            for (const RigExecValueOverride &o : _interactiveOverrides) {
                if (o.attribute.IsEmpty() || _IsRestInputName(o.attribute)) {
                    return true;
                }
            }
            return false;
        }();
        if (restOverridden && _restTaps && !_restTapIds.empty()) {
            RIGEXEC_PROFILE_SCOPE_CAT(
                _profiler, "RestFramesOverridden", "pose");
            const RigExecSnapshot rests =
                _restTaps->Evaluate(time, baseOverrides);
            if (!rests.IsValid() || !rests.IsComplete()) {
                pose.diagnostics.push_back(
                    "rest frame evaluation incomplete under an override");
                return pose;
            }
            for (const auto &[provider, tap] : _restTapIds) {
                const int fi = _providerIndex.at(provider);
                restFrames[fi] = rests.Get<RigExecPointFrame>(tap);
                restLive[fi] = 1;
            }
        } else {
            for (const auto &kv : _epochRestFrames) {
                const int fi = _providerIndex.at(kv.first);
                restFrames[fi] = kv.second;
                restLive[fi] = 1;
            }
        }
    }
    // Everything from here to the pose walk rebuilds the per-provider frame
    // maps from scratch, and it is the largest cost in this function that
    // carried no profiler scope: 1.7 ms with nothing changed, 3.2 ms on a
    // root drag. Stamped rather than scoped because the lambdas the pose
    // walk calls are declared in the middle of it, and a block would scope
    // them out.
    regionStart = profileRegions ? RigExecProfiler::NowUs() : 0;
    for (const auto &[provider, tap] : _firstFramePoseFrames) {
        const RigExecPointFrame frame = seedSnapshot.Get<RigExecPointFrame>(tap);
        baseFrames.emplace_hint(baseFrames.end(), provider, frame);
        const int fi = _providerIndex.at(provider);
        finalFrames[fi] = frame;
        finalLive[fi] = 1;
        if (!_firstFramePoseRests.empty()) {
            restFrames[fi] = seedSnapshot.Get<RigExecPointFrame>(
                _firstFramePoseRests.at(provider));
            restLive[fi] = 1;
        }
    }
    // Compose the transform of any plain Xformable lying between the asset
    // root and a provider, which exec resolves as identity and therefore
    // drops (docs/superpowers/specs/2026-09-09-intervening-xform-design.md).
    // At evaluation, from the stage, into the frames in memory. Nothing is
    // authored: the rig follows the Xform the author wrote, wherever they
    // wrote it, and no layer is rewritten.
    const auto authRestFrames = restFrames;
    const auto authRestLive = restLive;
    if (!_ComposeInterveningXforms(assetRoot, &constraintXformCache,
                                   &restFrames, &restLive, &baseFrames,
                                   &finalFrames, &finalLive, &pose)) {
        return pose;
    }
    for (const SdfPath &provider : _xformDerivedProviders) {
        RigExecPointFrame base;
        GfMatrix4d matrix(1.0);
        if (!frameFromXform(provider, &base, &matrix)) {
            pose.diagnostics.push_back("could not resolve constraint target " +
                provider.GetString() + " relative to the asset root");
            return pose;
        }
        xformDerivedBases[provider] = matrix;
        const int fi = _providerIndex.at(provider);
        restFrames[fi] = RigExecFrameFromMatrix(GfMatrix4d(1.0));
        restLive[fi] = 1;
        baseFrames[provider] = base;
        finalFrames[fi] = base;
        finalLive[fi] = 1;
    }
    stampRegion("FrameSeed");
    // The walk's frame store, in the two shapes the routines it shares with
    // the baked program ask for it: one provider by path, and every provider
    // it holds a base frame for. Both answer straight out of the maps as the
    // walk has left them -- the store is never copied into a third shape in
    // order to be read.
    auto finalFrameOf = [&](const SdfPath &path, RigExecPointFrame *frame) {
        const auto fi = _providerIndex.find(path);
        if (fi == _providerIndex.end() || !finalLive[fi->second]) {
            return false;
        }
        *frame = finalFrames[fi->second];
        return true;
    };
    auto enumerateProviderFrames = [&](const RigExecPoseFrameVisitor &visit) {
        for (std::size_t i = 0; i < _providerPaths.size(); ++i) {
            if (!finalLive[i]) continue;
            const SdfPath &provider = _providerPaths[i];
            const auto base = baseFrames.find(provider);
            if (base == baseFrames.end()) {
                continue;
            }
            visit(provider, base->second, finalFrames[i]);
        }
    };
    auto updateVolumePlacements = [&]() {
        _UpdateVolumePlacements(finalFrameOf, &pose);
    };
    updateVolumePlacements();

    std::function<bool(const SdfPath &)> refreshPoseProvider;
    auto resolveBinding = [&](const _FrameSourceBinding &binding,
                              RigExecPointFrame *out) {
        if (!refreshPoseProvider(binding.sourcePath)) return false;
        // Constraint relationships have implicit `preceding` semantics: the
        // single composed mover walk is the authority, so a later constraint
        // observes every earlier revision of the provider while a reference to
        // a provider written later still sees its current (normally base)
        // value. This is deterministic and cannot create an evaluation cycle.
        if (const auto revised = _providerIndex.find(binding.sourcePath);
            revised != _providerIndex.end() && finalLive[revised->second]) {
            *out = finalFrames[revised->second];
            return out->IsValid();
        }
        if (!binding.xformPath.IsEmpty()) {
            return _ResolveNativeXformSource(
                assetRoot, &constraintXformCache, binding.xformPath,
                enumerateProviderFrames, out);
        }
        return false;
    };

    auto readWeights = [&](const UsdPrim &prim, const TfToken &name,
                           size_t count, std::vector<double> *weights) {
        return _ReadConstraintSourceWeights(prim, name, count, time,
                                            &pose.diagnostics, weights);
    };

    auto readOffsets = [&](const UsdPrim &prim, const TfToken &name,
                           size_t count, std::vector<GfVec3d> *offsets) {
        return _ReadConstraintSourceOffsets(prim, name, count, time,
                                            &pose.diagnostics, offsets);
    };

    auto buildSources = [&](const _FrameConstraint &constraint,
                            const UsdPrim &prim,
                            std::vector<RigExecConstraintSource> *sources) {
        std::vector<double> weights;
        if (!readWeights(prim, _kDynSourceWeights,
                         constraint.sources.size(), &weights)) {
            return false;
        }
        std::vector<GfVec3d> translationOffsets, rotationOffsets;
        if (constraint.schemaType == _kDynParentConstraint) {
            if (!readOffsets(prim, _kDynTranslationOffsets,
                             constraint.sources.size(),
                             &translationOffsets) ||
                !readOffsets(prim, _kDynRotationOffsets,
                             constraint.sources.size(), &rotationOffsets)) {
                return false;
            }
        } else {
            translationOffsets.assign(constraint.sources.size(), GfVec3d(0));
            rotationOffsets.assign(constraint.sources.size(), GfVec3d(0));
        }
        sources->clear();
        sources->reserve(constraint.sources.size());
        for (size_t i = 0; i < constraint.sources.size(); ++i) {
            RigExecPointFrame sourceFrame;
            if (!resolveBinding(constraint.sources[i], &sourceFrame)) {
                pose.diagnostics.push_back(
                    prim.GetPath().GetString() +
                    " could not resolve source " +
                    constraint.sources[i].sourcePath.GetString());
                return false;
            }
            RigExecConstraintSource source;
            source.frame = sourceFrame;
            source.normalizedWeight = weights[i];
            source.translationOffset = translationOffsets[i];
            source.rotationOffsetDegrees = rotationOffsets[i];
            sources->push_back(source);
        }
        return true;
    };

    auto recordFrame = [&](const SdfPath &provider,
                           const SdfPath &afterMover) {
        const auto wanted = _chainPlan.snapshots.find(provider);
        const auto fi = _providerIndex.find(provider);
        const bool haveFrame =
            fi != _providerIndex.end() && finalLive[fi->second];
        if (wanted == _chainPlan.snapshots.end() ||
            !wanted->second.count(afterMover) || !haveFrame ||
            !finalFrames[fi->second].IsValid()) {
            return;
        }
        const auto &restFrame = restFrames[fi->second];
        const auto &landmarks =
            restLive[fi->second] && restFrame.IsValid()
                ? restFrame.points
                : RigExecIdentityLandmarks();
        GfMatrix4d matrix(1.0);
        if (RigExecPointsToMatrix(landmarks, finalFrames[fi->second].points, &matrix)) {
            _chainSnapshots.Record(provider, afterMover, VtValue(matrix));
        }
    };

    const std::unordered_set<SdfPath, SdfPath::Hash> &hierarchicalProviders =
        _hierarchicalProviderSet;

    // MEASURED 2026-09-13, biped: 3003 calls per frame, 8.1 us each -- 24 ms
    // of a 49 ms evaluate -- because commitConstraintFrames asks this once per
    // descendant joint per constraint, and the same handful of joints are
    // walked again for every constraint in the rig.
    // The predicate reads parent:space AT A TIME, and parent:space may carry
    // time samples, so a compile-time answer would be wrong on any frame but
    // the one it was baked at -- and this predicate decides whether a
    // constraint's pose propagates through a joint, so a wrong answer moves
    // joints by centimetres (see verify_spine.py). Within one Evaluate,
    // `time` is fixed and the stage cannot change, so a memo is byte-identical
    // to recomputing.
    // Two tiers: _namespaceInheritsCache is a persistent member, cleared on
    // epoch change, that records only stage-constant answers (parent:space
    // with no time samples). Time-sampled attributes stay in the per-Evaluate
    // namespacePoseCache and are re-read every frame.
    std::unordered_map<SdfPath, bool, SdfPath::Hash> namespacePoseCache;
    const auto inheritsNamespacePose = [&](const SdfPath &path) {
        const auto persistCached = _namespaceInheritsCache.find(path);
        if (persistCached != _namespaceInheritsCache.end()) {
            return persistCached->second;
        }
        const auto cached = namespacePoseCache.find(path);
        if (cached != namespacePoseCache.end()) {
            return cached->second;
        }
        const UsdPrim prim = _stage->GetPrimAtPath(path);
        bool inherits = true;
        bool stageConstant = true;
        for (const TfToken *name : {&_kDynParentSpace}) {
            const UsdAttribute attribute = prim.GetAttribute(*name);
            SdfPathVector connections;
            // HasAuthoredConnections first: see _AuthoredConnections.
            // `inherits = false; break;` rather than an early return, so the
            // answer still reaches the memo below.
            if (attribute && attribute.HasAuthoredConnections() &&
                attribute.GetConnections(&connections) &&
                !connections.empty()) { inherits = false; break; }
            GfMatrix4d authored(1.0);
            if (attribute) {
                if (attribute.GetNumTimeSamples() > 0) {
                    stageConstant = false;
                }
                if (attribute.Get(&authored, time) &&
                    authored != GfMatrix4d(1.0)) { inherits = false; break; }
            }
        }
        namespacePoseCache.emplace(path, inherits);
        if (stageConstant) {
            _namespaceInheritsCache.emplace(path, inherits);
        }
        return inherits;
    };

    // Nearest pose-owning ancestor-or-self of a path, memoized for this
    // evaluation.
    // Namespace propagation stops at a path that owns its own pose -- a joint
    // a solver writes, or a provider whose parent:space is authored rather
    // than inherited. Both walks below need that answer for a provider
    // relative to some ancestor, and both used to rediscover it by climbing
    // the namespace and re-reading parent:space off the stage at every step,
    // once per propagated descendant: quadratic along a joint chain, and the
    // reason a long spine costs more at its root than at its tip. The nearest
    // owner depends only on the path, so it is computed once and shared.
    // The climb closes over every path element, not only the known providers:
    // a provider's parent need not itself be a provider.
    const auto ownsItsPose = [&](const SdfPath &path) {
        return _jointSolverBinding.count(path) ||
            (hierarchicalProviders.count(path) && !inheritsNamespacePose(path));
    };
    const auto nearestBlocking = [&](const SdfPath &path) {
        std::vector<SdfPath> pending;
        SdfPath walk = path;
        SdfPath owner;
        for (; !walk.IsEmpty(); walk = walk.GetParentPath()) {
            const auto cached = _nearestBlockingCache.find(walk);
            if (cached != _nearestBlockingCache.end()) {
                owner = cached->second;
                break;
            }
            if (ownsItsPose(walk)) {
                _nearestBlockingCache[walk] = walk;
                owner = walk;
                break;
            }
            pending.push_back(walk);
        }
        for (const SdfPath &seen : pending) {
            _nearestBlockingCache[seen] = owner;
        }
        return owner;
    };

    std::set<SdfPath> constrainedProviders;
    // A consumer may ask for the same connected hierarchy hundreds of times
    // in one generation. Keep a completed refresh until an intervening frame
    // write reaches it through the compiled expression dependencies.
    std::vector<unsigned char> refreshComplete(_providerPaths.size(), 0);
    // Bits: known, base equals seed, final equals seed. Every pose write
    // invalidates the affected dependency closure below.
    std::vector<unsigned char> seedMatches(_providerPaths.size(), 0);
    std::vector<uint64_t> refreshInvalidated(_providerPaths.size(), 0);
    uint64_t refreshWrite = 0;
    const auto invalidateRefreshes = [&](std::vector<int> pending) {
        ++refreshWrite;
        while (!pending.empty()) {
            const int index = pending.back();pending.pop_back();
            if (refreshInvalidated[index] == refreshWrite) continue;
            refreshInvalidated[index] = refreshWrite;
            refreshComplete[index] = 0;
            seedMatches[index] = 0;
            const auto &dependents = _poseRefreshDependents[index];
            pending.insert(pending.end(), dependents.begin(), dependents.end());
        }
    };
    // Validate and commit one constraint's complete write bundle. Descendant
    // RigExec providers are updated from the nearest changed ancestor in the
    // same transaction; native Xform descendants ride the published ancestor
    // delta in Hydra and therefore must not be duplicated here.
    auto commitConstraintFrames =
        [&](const SdfPath &moverPath,
            const std::map<SdfPath, RigExecPointFrame> &candidates,
            bool solverOutput = false, bool derivedRefresh = false) {
        for (const auto &[path, frame] : candidates) {
            if (!solverOutput && !_IsUsableConstraintFrame(frame)) {
                pose.diagnostics.push_back(
                    moverPath.GetString() +
                    " produced an invalid or degenerate frame for " +
                    path.GetString() + "; constraint passed through");
                return false;
            }
        }

        std::vector<std::pair<int, RigExecPointFrame>> propagated;
        // The hierarchy delta for a descendant depends only on its nearest
        // candidate ancestor ("closest"): that ancestor's old (before) and new
        // (candidate) frames are shared by every descendant under it, so the
        // delta is computed once per closest and reused. It is ALWAYS
        // applied, even when before and candidate are exactly equal: the
        // baked program applies unconditionally, and skipping the multiply
        // differs from applying an almost-identity by rounding dust --
        // which is exactly what the parity comparisons forbid.
        std::map<SdfPath, GfMatrix4d> closestDelta;
        // Only descendants can change. Enumerate disjoint changed subtrees,
        // rather than scanning every provider for each solver dependency level.
        std::vector<int> descendants;
        SdfPath coveredRoot;
        {
            RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "ccfDesc", "pose");
        for (const auto &[target, candidate] : candidates) {
            if (!coveredRoot.IsEmpty() && target.HasPrefix(coveredRoot)) continue;
            coveredRoot = target;
            const auto ti = _providerIndex.find(target);
            if (ti == _providerIndex.end()) continue;
            if (!_hierDescendants[ti->second].empty()) {
                for (int di : _hierDescendants[ti->second]) {
                    if (!candidates.count(_providerPaths[di]))
                        descendants.push_back(di);
                }
            } else {
                for (std::size_t j = static_cast<std::size_t>(ti->second) + 1;
                     j < _providerPaths.size() &&
                     _providerPaths[j].HasPrefix(target); ++j) {
                    if (candidates.count(_providerPaths[j])) continue;
                    if (!hierarchicalProviders.count(_providerPaths[j])) continue;
                    descendants.push_back(static_cast<int>(j));
                }
            }
        }
        }
        if (candidates.size() == 1) {
            const SdfPath &closest = candidates.begin()->first;
            const auto beforeIt = _providerIndex.find(closest);
            const bool haveBefore = beforeIt != _providerIndex.end() &&
                                    finalLive[beforeIt->second];
            const RigExecPointFrame &candFrame = candidates.begin()->second;
            // Single candidate: every descendant maps through the same
            // closest, so one shared hierarchy delta serves them all.
            GfMatrix4d delta(1.0);
            bool sharedSingular = false;
            if (haveBefore &&
                !RigExecPointsToMatrix(finalFrames[beforeIt->second].points,
                                       candFrame.points, &delta)) {
                sharedSingular = true;
            }
            {
                RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "ccfSingle", "pose");
            for (int di : descendants) {
                const SdfPath &provider = _providerPaths[di];
                const RigExecPointFrame &current = finalFrames[di];
                // An independently solved joint is an absolute posed
                // override; namespace propagation cannot pass through it.
                const SdfPath blocker = nearestBlocking(provider);
                if (!blocker.IsEmpty() && blocker != closest &&
                    blocker.HasPrefix(closest)) {
                    continue;
                }
                if (solverOutput && (!haveBefore ||
                    !_IsUsableConstraintFrame(current) ||
                    !_IsUsableConstraintFrame(finalFrames[beforeIt->second]) ||
                    !_IsUsableConstraintFrame(candFrame))) continue;
                if (!haveBefore ||
                    !_IsUsableConstraintFrame(current)) {
                    pose.diagnostics.push_back(
                        moverPath.GetString() +
                        " could not propagate its pose revision through " +
                        provider.GetString() + "; constraint passed through");
                    return false;
                }
                if (sharedSingular) {
                    pose.diagnostics.push_back(
                        moverPath.GetString() +
                        " produced a singular hierarchy delta; constraint "
                        "passed through");
                    return false;
                }
                RigExecPointFrame frame =
                    RigExecMatrixToPoints(current.points, delta);
                if (!_IsUsableConstraintFrame(frame)) {
                    pose.diagnostics.push_back(
                        moverPath.GetString() +
                        " produced an invalid descendant frame for " +
                        provider.GetString() + "; constraint passed through");
                    return false;
                }
                propagated.emplace_back(di, frame);
            }
            }
        } else {
            for (int di : descendants) {
                const SdfPath &provider = _providerPaths[di];
                const RigExecPointFrame &current = finalFrames[di];
                SdfPath closest = provider.GetParentPath();
                for (; !closest.IsEmpty() && !candidates.count(closest);
                     closest = closest.GetParentPath()) {}
                if (closest.IsEmpty()) {
                    continue;
                }
                const SdfPath blocker = nearestBlocking(provider);
                if (!blocker.IsEmpty() && blocker != closest &&
                    blocker.HasPrefix(closest)) {
                    continue;
                }
                const auto beforeIt = _providerIndex.find(closest);
                const bool haveBefore = beforeIt != _providerIndex.end() &&
                                        finalLive[beforeIt->second];
                if (solverOutput && (!haveBefore ||
                    !_IsUsableConstraintFrame(current) ||
                    !_IsUsableConstraintFrame(finalFrames[beforeIt->second]) ||
                    !_IsUsableConstraintFrame(candidates.at(closest)))) continue;
                if (!haveBefore ||
                    !_IsUsableConstraintFrame(current)) {
                    pose.diagnostics.push_back(
                        moverPath.GetString() +
                        " could not propagate its pose revision through " +
                        provider.GetString() + "; constraint passed through");
                    return false;
                }
                auto cdIt = closestDelta.find(closest);
                if (cdIt == closestDelta.end()) {
                    GfMatrix4d delta(1.0);
                    if (!RigExecPointsToMatrix(finalFrames[beforeIt->second].points,
                                               candidates.at(closest).points,
                                               &delta)) {
                        pose.diagnostics.push_back(
                            moverPath.GetString() +
                            " produced a singular hierarchy delta; constraint "
                            "passed through");
                        return false;
                    }
                    cdIt = closestDelta.emplace(closest, delta).first;
                }
                RigExecPointFrame frame =
                    RigExecMatrixToPoints(current.points, cdIt->second);
                if (!_IsUsableConstraintFrame(frame)) {
                    pose.diagnostics.push_back(
                        moverPath.GetString() +
                        " produced an invalid descendant frame for " +
                        provider.GetString() + "; constraint passed through");
                    return false;
                }
                propagated.emplace_back(di, frame);
            }
        }

        {
            RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "ccfFinal", "pose");
        std::vector<int> changed;
        for (const auto &[path, frame] : candidates) {
            const int fi = _providerIndex.at(path);
            if (!finalLive[fi] || finalFrames[fi].points != frame.points ||
                (solverOutput && baseFrames[path].points != frame.points)) changed.push_back(fi);
            finalFrames[fi] = frame;
            finalLive[fi] = 1;
            if (!solverOutput && !derivedRefresh) constrainedProviders.insert(path);
            if (solverOutput) baseFrames[path] = frame;
        }
        for (const auto &[idx, frame] : propagated) {
            if (!finalLive[idx] || finalFrames[idx].points != frame.points ||
                (solverOutput && baseFrames[_providerPaths[idx]].points != frame.points)) changed.push_back(idx);
            finalFrames[idx] = frame;
            finalLive[idx] = 1;
            if (solverOutput) baseFrames[_providerPaths[idx]] = frame;
        }
        if (!changed.empty()) invalidateRefreshes(std::move(changed));
        updateVolumePlacements();
        }
        return true;
    };

    // Connected matrix expressions cannot be updated by a namespace delta:
    // their posed input may live in another subtree. Refresh only those
    // providers, with explicit current/base input phases, before a consumer
    // reads them. Within a generation an unchanged input tuple reuses the
    // previous refresh, even when many operators consume the same control.
    std::map<SdfPath, std::vector<RigExecValueOverride>> connectedInputCache;
    struct ConnectedBatchResult {
        std::vector<RigExecValueOverride> baseInputs, finalInputs;
        RigExecSnapshot base, final;
        bool ready = false;
    };
    std::map<size_t, ConnectedBatchResult> connectedBatchResults;
    // Importers may opt in after validating their declared frame and
    // attribute input closures. Other rigs retain complete override reads.
    const VtValue seedReuse = _stage->GetPrimAtPath(_rigPath).GetCustomDataByKey(
        TfToken("rigExec:connectedPoseSeedReuse"));
    const bool seedReuseDefault = seedReuse.IsHolding<bool>() && seedReuse.UncheckedGet<bool>();
    const bool reuseConnectedSeeds = TfGetenv("RIGEXEC_CONNECTED_POSE_SEED_REUSE",
        seedReuseDefault ? "1" : "0") != "0";
    // baseOverrides is complete for this generation. Index its frame pins once
    // instead of rebuilding the complement of each provider's direct inputs.
    // Keep original indices so duplicate overrides retain their precedence.
    std::unordered_map<SdfPath, std::vector<size_t>, SdfPath::Hash> frameOverrideIndices;
    std::vector<size_t> otherOverrideIndices;
    if (reuseConnectedSeeds) {
        for (size_t i = 0; i < baseOverrides.size(); ++i) {
            const auto &input = baseOverrides[i];
            if (input.attribute.IsEmpty() && input.computation == _computePointFrame) {
                frameOverrideIndices[input.prim].push_back(i);
            } else {
                otherOverrideIndices.push_back(i);
            }
        }
    }
    const auto sameOverrides = [](const auto &a, const auto &b) {
        return a.size() == b.size() && std::equal(a.begin(), a.end(), b.begin(),
            [](const RigExecValueOverride &x, const RigExecValueOverride &y) {
                return x.prim == y.prim && x.computation == y.computation &&
                    x.attribute == y.attribute && x.value == y.value;
            });
    };
    const auto seedMatch = [&](const SdfPath &input) -> unsigned char {
        const auto index = _providerIndex.find(input);
        if (index == _providerIndex.end()) return 4;
        auto &match = seedMatches[index->second];
        if (match) return match;
        match = 4;
        const auto seed = _firstFramePoseFrames.find(input);
        const auto base = baseFrames.find(input);
        if (seed == _firstFramePoseFrames.end() || base == baseFrames.end() ||
            !finalLive[index->second]) return match;
        const auto &value = seedSnapshot.Get(seed->second);
        if (!value.IsHolding<RigExecPointFrame>()) return match;
        const auto &original = value.UncheckedGet<RigExecPointFrame>();
        if (base->second.points == original.points) match |= 1;
        if (finalFrames[index->second].points == original.points) match |= 2;
        return match;
    };
    std::unordered_map<SdfPath, std::vector<SdfPath>, SdfPath::Hash> seedClosures;
    refreshPoseProvider = [&](const SdfPath &requested) {
        // Nothing to refresh when no provider has a connected tap: the walk
        // below would visit the whole pose DAG to discover that at every
        // constraint target of every frame. Its only other effect is the
        // cycle diagnostic, and Compile already rejects a cyclic pose DAG.
        if (_connectedPoseTaps.empty()) {
            return true;
        }
        std::vector<std::pair<SdfPath, bool>> pending{{requested, false}};
        // These sets only test membership; traversal order comes from pending.
        std::unordered_set<SdfPath, SdfPath::Hash> active, complete;
        while (!pending.empty()) {
            const SdfPath path = pending.back().first;
            const bool ready = pending.back().second;
            pending.pop_back();
            if (path.IsEmpty() || complete.count(path) ||
                _jointSolverBinding.count(path) || constrainedProviders.count(path)) continue;
            const auto provider = _providerIndex.find(path);
            if (provider != _providerIndex.end() && refreshComplete[provider->second]) continue;
            const auto dependencies = _poseProviderInputs.find(path);
            if (!ready) {
                if (!active.insert(path).second) {
                    pose.diagnostics.push_back("connected pose provider cycle at " + path.GetString());
                    return false;
                }
                pending.emplace_back(path, true);
                if (dependencies != _poseProviderInputs.end()) {
                    for (const SdfPath &input : dependencies->second) {
                        pending.emplace_back(input, false);
                    }
                }
                continue;
            }
            active.erase(path);
            complete.insert(path);
            const auto taps = _connectedPoseTaps.find(path);
            if (taps == _connectedPoseTaps.end()) {
                if (provider != _providerIndex.end()) refreshComplete[provider->second] = 1;
                continue;
            }
            // The initial batch already evaluated every connected expression
            // with this generation's authored and interactive overrides. Its
            // answer remains authoritative until a dependency (or the output
            // itself, through namespace propagation) changes in either phase.
            // Leave the individual tap's dirty/cache state intact: a later
            // phase revision must still invalidate a previous-generation hit.
            const auto unchangedFromSeed = [&](const SdfPath &input) {
                return seedMatch(input) == 7;
            };
            if (unchangedFromSeed(path) &&
                (dependencies == _poseProviderInputs.end() ||
                 std::all_of(dependencies->second.begin(), dependencies->second.end(), unchangedFromSeed))) {
                refreshComplete[provider->second] = 1;
                continue;
            }
            const auto &attributes = _connectedPoseOverrideInputs.at(path);
            const auto &localAttributes = _connectedPoseLocalOverrideInputs.at(path);
            std::vector<RigExecValueOverride> baseInputs;
            std::vector<SdfPath> removedAttributes;
            if (!reuseConnectedSeeds) {
                baseInputs = baseOverrides;
            } else {
                std::vector<size_t> included;
                for (size_t i : otherOverrideIndices) {
                    const auto &input = baseOverrides[i];
                    if (input.attribute.IsEmpty()) {
                        included.push_back(i);
                    } else {
                        const SdfPath attribute = input.prim.AppendProperty(input.attribute);
                        if (std::binary_search(localAttributes.begin(), localAttributes.end(), attribute))
                            included.push_back(i);
                        else removedAttributes.push_back(attribute);
                    }
                }
                const auto includeFrames = [&](const SdfPath &input) {
                    const auto found = frameOverrideIndices.find(input);
                    if (found != frameOverrideIndices.end())
                        included.insert(included.end(), found->second.begin(), found->second.end());
                };
                includeFrames(path);
                if (dependencies != _poseProviderInputs.end()) {
                    for (const SdfPath &input : dependencies->second)
                        if (input != path) includeFrames(input);
                }
                std::sort(included.begin(), included.end());
                baseInputs.reserve(included.size());
                for (size_t i : included) baseInputs.push_back(baseOverrides[i]);
            }
            std::vector<RigExecValueOverride> finalInputs = baseInputs;
            // An override invalidates the whole downstream Exec network,
            // even outside this request. The shared seed already provides
            // unchanged inputs. Keep a pin when any dependency has changed:
            // recomputing an unchanged child against a revised ancestor would
            // otherwise change the pose phase that this request must read.
            const auto unchangedSeedClosure = [&](const SdfPath &input) -> unsigned char {
                const auto closure = _connectedPoseOverrideInputs.find(input);
                if (closure == _connectedPoseOverrideInputs.end()) return 0;
                for (const auto &attribute : removedAttributes) {
                    if (std::binary_search(closure->second.begin(), closure->second.end(), attribute))
                        return 0;
                }
                auto cachedClosure = seedClosures.find(input);
                if (cachedClosure == seedClosures.end()) {
                    std::vector<SdfPath> pending{input}, paths;
                    std::unordered_set<SdfPath, SdfPath::Hash> seen;
                    while (!pending.empty()) {
                        const auto current = pending.back();
                        pending.pop_back();
                        if (!seen.insert(current).second) continue;
                        paths.push_back(current);
                        const auto upstream = _poseProviderInputs.find(current);
                        if (upstream != _poseProviderInputs.end())
                            pending.insert(pending.end(), upstream->second.begin(), upstream->second.end());
                    }
                    cachedClosure = seedClosures.emplace(input, std::move(paths)).first;
                }
                unsigned char match = 3;
                for (const SdfPath &current : cachedClosure->second) {
                    if (frameOverrideIndices.count(current) && current != path &&
                        (dependencies == _poseProviderInputs.end() || !dependencies->second.count(current)))
                        return 0;
                    match &= seedMatch(current);
                    if (!match) return 0;
                }
                return match;
            };
            if (dependencies != _poseProviderInputs.end()) {
                for (const SdfPath &input : dependencies->second) {
                    const auto base = baseFrames.find(input);
                    const auto currentIt = _providerIndex.find(input);
                    if (base == baseFrames.end() || currentIt == _providerIndex.end() ||
                        !finalLive[currentIt->second]) continue;
                    const unsigned char seedPhases = reuseConnectedSeeds ? unchangedSeedClosure(input) : 0;
                    if (!(seedPhases & 1))
                        baseInputs.push_back({input, _computePointFrame, TfToken(), VtValue(base->second)});
                    if (!(seedPhases & 2))
                        finalInputs.push_back({input, _computePointFrame, TfToken(), VtValue(finalFrames[currentIt->second])});
                }
            }
            // Direct attribute inputs and pinned dependency frames replace
            // upstream overrides. The fingerprint covers the requested
            // expression's inputs in both pose phases.
            std::vector<RigExecValueOverride> identity;
            for (const auto &inputs : {&baseInputs, &finalInputs}) {
                for (const auto &input : *inputs) {
                    if (input.attribute.IsEmpty() || std::binary_search(
                            attributes.begin(), attributes.end(),
                            input.prim.AppendProperty(input.attribute))) {
                        identity.push_back(input);
                    }
                }
            }
            const auto cached = connectedInputCache.find(path);
            if (cached != connectedInputCache.end() && sameOverrides(cached->second, identity)) {
                if (provider != _providerIndex.end()) refreshComplete[provider->second] = 1;
                continue;
            }
            auto &stored = _connectedPoseCache[path];
            const auto batchIndex = _connectedPoseBatchIndex.find(path);
            // A batch changes how a miss is computed, not the identity of
            // this output. Authored edits retire _connectedPoseCache, and
            // the key below includes time and both phases' relevant inputs.
            const bool reuseStored =
                stored.cached && stored.time == time &&
                (batchIndex != _connectedPoseBatchIndex.end() ||
                 !taps->second->ConsumeDirty()) &&
                sameOverrides(stored.inputs, identity);
            RigExecPointFrame raw;
            RigExecPointFrame current;
            if (reuseStored) {
                raw = stored.base;
                current = stored.current;
            } else if (batchIndex != _connectedPoseBatchIndex.end()) {
                auto &batch = _connectedPoseBatches[batchIndex->second];
                auto &result = connectedBatchResults[batchIndex->second];
                if (!result.ready || !sameOverrides(result.baseInputs, baseInputs) ||
                    !sameOverrides(result.finalInputs, finalInputs)) {
                    RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "ConnectedPose.Batch", "pose");
                    result.base = batch.taps->Evaluate(time, baseInputs);
                    result.final = sameOverrides(baseInputs, finalInputs)
                        ? result.base : batch.taps->Evaluate(time, finalInputs);
                    result.ready = result.base.IsValid() && result.base.IsComplete() &&
                        result.final.IsValid() && result.final.IsComplete();
                    result.baseInputs = baseInputs;
                    result.finalInputs = finalInputs;
                }
                if (!result.ready) {
                    stored.cached = false;
                    pose.diagnostics.push_back("connected pose batch incomplete: " + path.GetString());
                    return false;
                }
                raw = result.base.Get<RigExecPointFrame>(batch.outputs.at(path));
                current = result.final.Get<RigExecPointFrame>(batch.outputs.at(path));
            } else {
                const RigExecSnapshot base = taps->second->Evaluate(time, baseInputs);
                if (!base.IsValid() || !base.IsComplete()) {
                    stored.cached = false;
                    pose.diagnostics.push_back("connected base pose input incomplete: " + path.GetString());
                    return false;
                }
                raw = base.Get<RigExecPointFrame>(0);
                current = raw;
                if (!sameOverrides(baseInputs, finalInputs)) {
                    const RigExecSnapshot revised = taps->second->Evaluate(time, finalInputs);
                    if (!revised.IsValid() || !revised.IsComplete()) {
                        stored.cached = false;
                        pose.diagnostics.push_back("connected final pose input incomplete: " + path.GetString());
                        return false;
                    }
                    current = revised.Get<RigExecPointFrame>(0);
                }
            }
            // Maintain the base phase for namespace descendants independently
            // from the current constraint phase. Solver-provided sources are
            // already present in baseFrames; constraint revisions are not.
            const auto previous = baseFrames.find(path);
            std::vector<int> baseChanged;
            if (provider != _providerIndex.end() &&
                (previous == baseFrames.end() || previous->second.points != raw.points)) baseChanged.push_back(provider->second);
            GfMatrix4d delta(1.0);
            if (previous != baseFrames.end() && _IsUsableConstraintFrame(raw) &&
                RigExecPointsToMatrix(previous->second.points, raw.points, &delta)) {
                for (auto it = baseFrames.upper_bound(path);
                     it != baseFrames.end() && it->first.HasPrefix(path); ++it) {
                    if (!hierarchicalProviders.count(it->first)) continue;
                    const SdfPath blocker = nearestBlocking(it->first);
                    const bool blocked = !blocker.IsEmpty() &&
                        blocker != path && blocker.HasPrefix(path);
                    if (!blocked) {
                        const auto revised = RigExecMatrixToPoints(it->second.points, delta);
                        if (it->second.points != revised.points) baseChanged.push_back(_providerIndex.at(it->first));
                        it->second = revised;
                    }
                }
            }
            baseFrames[path] = raw;
            if (!baseChanged.empty()) invalidateRefreshes(std::move(baseChanged));
            if (!commitConstraintFrames(path, {{path, current}}, false, true)) {
                stored.cached = false;
                return false;
            }
            stored.inputs = identity;
            stored.time = time;
            stored.base = raw;
            stored.current = current;
            stored.cached = true;
            connectedInputCache[path] = std::move(identity);
            if (provider != _providerIndex.end()) refreshComplete[provider->second] = 1;
        }
        return true;
    };

    std::set<size_t> visitedSolverLevels;
    // One batch's tail: the overrides its solver's request needs beyond
    // baseOverrides. Only direct prerequisites enter it. Copying all previous
    // joint overrides into every level would itself be quadratic for a deep
    // chain, even if the kernels each executed only once. False when a
    // connected refresh failed, which ends the generation.
    // A shared request (see "one exec request per run" in Compile) builds
    // every member's tail here, back to back, before any member commits.
    // Compile only groups solvers for which that is the same tail the walk
    // would have built at each one's own step.
    const auto appendBatchTail =
        [&](const _SolverBatch &batch,
            std::vector<RigExecValueOverride> *tail) -> bool {
        for (const SdfPath &dependency : batch.dependencies) {
            const auto aggregate = solvedAggregates.find(dependency);
            if (aggregate != solvedAggregates.end()) {
                tail->push_back({dependency, _computePointFrameArray, TfToken(),
                                 VtValue(aggregate->second)});
            }
        }
        for (const SdfPath &input : batch.frameInputs) {
            if (!refreshPoseProvider(input)) return false;
            const auto fi = _providerIndex.find(input);
            if (fi != _providerIndex.end() && finalLive[fi->second]) {
                tail->push_back({input, _computePointFrame, TfToken(),
                                 VtValue(finalFrames[fi->second])});
            }
        }
        // "The incoming frame replaces the authored rest" (spec §4.2).
        // The joint prim publishes computePointFrame AND computeRestFrame,
        // and RigExecTwoBoneIk / RigExecSplineIk request the latter by
        // name off rigExec:joints -- so the whole of the dynamic-side
        // change is one more override loop on a DIFFERENT computation.
        // rigExec:joints stays skipped in solverFrameInputs: overriding a
        // solver's own output joints as computePointFrame would feed the
        // solver back into itself, which is a different thing entirely.
        // A live entry takes the frame the preceding step left; an entry
        // with no predecessor pins the AUTHORED rest, because
        // computeRestFrame reads its namespace ancestor's and an override
        // on the hip would otherwise move the knee's rest too. The map is
        // empty unless something is live, so a rig with no pose step below
        // a solver pushes nothing here at all.
        for (const auto &[joint, predecessor] : batch.restInputs) {
            const bool live = !predecessor.IsEmpty();
            RigExecPointFrame rest;
            bool haveRest = false;
            if (live) {
                const auto fi = _providerIndex.find(joint);
                if (fi != _providerIndex.end() && finalLive[fi->second]) {
                    rest = finalFrames[fi->second];
                    haveRest = true;
                }
            } else {
                const auto fi = _providerIndex.find(joint);
                if (fi != _providerIndex.end() && restLive[fi->second]) {
                    rest = restFrames[fi->second];
                    haveRest = true;
                }
            }
            if (haveRest) {
                // A solver with a basis of its own -- an FK chain
                // composing control deltas -- switches to the joint's
                // rest reference only where this flag says a step below
                // it really wrote the joint. Everywhere else it keeps the
                // basis it always had, which is what makes the rule free
                // on every rig that does not stack.
                if (live) rest.flags |= RigExecPointFrameLiveRest;
                tail->push_back({joint, _computeRestFrame, TfToken(),
                                VtValue(rest)});
            }
        }
        return true;
    };

    // Its overrides -- every solver aggregate and every provider's BASE
    // frame -- are final at the last solver commit: only a solver commit
    // writes baseFrames or solvedAggregates, apart from the connected-pose
    // refresh. So once the last solver has committed, the request can run on
    // a worker while this thread walks the constraints after it (on the
    // biped, everything past L30), instead of after them.
    // Only where that is the whole story:
    //  - No connected pose provider. refreshPoseProvider is then a no-op, so
    //    the constraint tail makes no exec call and moves no base frame --
    //    exec stays single-lane, and the overrides really are final.
    //  - Parallel evaluation on, and not inside a frozen run, whose thread
    //    must dispatch nothing.
    //  - Not a time-keyed cache hit, which needs no exec at all.
    // The JOIN sits where the snapshot used to be computed: after the
    // constraint tail and the fallback-joint diagnostics, and before
    // PublishProviders. So the snapshot's failure check and early return
    // happen exactly where they did, after exactly the same diagnostics, and
    // nothing the publish loops emit can precede them. The solver guides,
    // exec too, stay after the join.
    const auto assembleAuthOverrides = [&]() {
        std::vector<RigExecValueOverride> jointOverrides = baseOverrides;
        for (const auto &[solver, aggregate] : solvedAggregates) {
            jointOverrides.push_back({solver, _computePointFrameArray,
                                      TfToken(), VtValue(aggregate)});
        }
        for (const auto &[provider, tap] : _firstFramePoseFrames) {
            jointOverrides.push_back({provider, _computePointFrame,
                                      TfToken(),
                                      VtValue(baseFrames.at(provider))});
        }
        jointOverrides.insert(jointOverrides.end(),
                              _falloffLutOverrides.begin(),
                              _falloffLutOverrides.end());
        return jointOverrides;
    };
    const auto evaluateAuthSnapshot = [&](const std::vector<RigExecValueOverride> &overrides) {
        std::vector<VtValue> supplied(_taps->GetTapCount());
        std::unordered_set<SdfPath, SdfPath::Hash> matrixProviders(
            _jointPaths.begin(), _jointPaths.end());
        matrixProviders.insert(_controlPaths.begin(), _controlPaths.end());
        std::unordered_map<SdfPath, VtValue, SdfPath::Hash> matrices;
        std::unordered_map<SdfPath, const VtValue *, SdfPath::Hash> matrixOverrides;
        for (const auto &value : overrides)
            if (value.attribute.IsEmpty() && value.computation == _kDynComputeMatrix)
                matrixOverrides[value.prim] = &value.value;
        for (size_t tap = 0; tap < supplied.size(); ++tap) {
            const auto &address = _taps->GetAddress(tap);
            if (address.target.IsPropertyPath()) continue;
            const auto base = baseFrames.find(address.target);
            if (base == baseFrames.end() || !_firstFramePoseFrames.count(address.target)) continue;
            if (address.publicComputation == _computePointFrame) {
                // assembleAuthOverrides pins exactly this frame.
                supplied[tap] = VtValue(base->second);
            } else if (address.publicComputation == _kDynComputeMatrix &&
                       matrixProviders.count(address.target)) {
                const auto explicitValue = matrixOverrides.find(address.target);
                if (explicitValue != matrixOverrides.end()) {
                    if (explicitValue->second->IsHolding<GfMatrix4d>())
                        supplied[tap] = *explicitValue->second;
                    continue;
                }
                const int index = _providerIndex.at(address.target);
                if (!authRestLive[index] || !authRestFrames[index].IsValid() ||
                    !base->second.IsValid()) continue;
                auto found = matrices.find(address.target);
                if (found == matrices.end()) {
                    // The same operation as the native computeMatrix kernel.
                    // Use the Exec rest frame, before plain Xform ancestors
                    // are composed into the pose walk's world-space frame.
                    GfMatrix4d matrix(1.0);
                    RigExecPointsToMatrix(authRestFrames[index].points, base->second.points, &matrix);
                    found = matrices.emplace(address.target, VtValue(matrix)).first;
                }
                supplied[tap] = found->second;
            }
        }
        return _taps->EvaluateWithSuppliedResults(time, overrides, std::move(supplied));
    };
    const bool authCacheHit = !_authSnapshotDirty &&
                              _interactiveOverrides.empty() &&
                              _authSnapTimeKeyed.count(time) > 0;
    const bool overlapSnapshot = _connectedPoseTaps.empty() &&
                                 RigExecParallelEvaluationEnabled() &&
                                 !RigExecFrozenSerialActive() &&
                                 !authCacheHit;
    const _PoseStep *lastSolverStep = nullptr;
    for (const _PoseStep &step : _poseSteps) {
        if (step.solverBatch) lastSolverStep = &step;
    }
    // Declared in this order so that an early return joins the worker
    // before anything it touches is freed (a WorkDispatcher waits in its
    // destructor), and with the GIL released across that wait: the worker's
    // Evaluate can re-prepare a request, and the first prepare in a process
    // loads the exec plugins through TfScriptModuleLoader, which needs the
    // GIL a Python caller holds across this call (see Compile).
    TF_PY_ALLOW_THREADS_IN_SCOPE();
    std::vector<RigExecValueOverride> overlappedOverrides;
    RigExecSnapshot overlappedSnapshot;
    WorkDispatcher snapshotDispatcher;
    bool snapshotInFlight = false;
    const auto launchSnapshot = [&]() {
        if (!overlapSnapshot || snapshotInFlight) return;
        _taps->ConsumeDirty();
        overlappedOverrides = assembleAuthOverrides();
        snapshotDispatcher.Run(
            [this, time, &overlappedOverrides, &overlappedSnapshot]() {
                RIGEXEC_PROFILE_SCOPE_CAT(
                    _profiler, "AuthoritativeSnapshot", "exec");
                overlappedSnapshot = _taps->Evaluate(time, overlappedOverrides);
            });
        snapshotInFlight = true;
    };
    // 2026-09-13: this stamp was briefly believed to be load-bearing -- moving
    // it one line earlier appeared to turn testRigExecNoAuthoring into an
    // access violation and testRigExecWeightOverlay into 0xC0000409. It is
    // not. The crashes were an ABI mismatch: rigExecImaging/bridge.h includes
    // rigEvaluator.h, hence types.h, and types.h was being edited in another
    // window (RigExecBlendSampleData gained a member, so its size changed).
    // A partial rebuild left rigExec.dll and rigExecImaging.dll disagreeing
    // about that layout, which is exactly an access violation in one imaging
    // test and a stack-cookie failure in another -- and why BOTH failures
    // were imaging tests, the clue the "statement placement" theory never
    // explained. Every bisect step rebuilt a different subset, so the
    // pass/fail pattern tracked which DLLs happened to resynchronise, not the
    // edit under test. Placement is irrelevant; verified green in both
    // positions at /O2 and /Od once the tree settled.
    // Left as a note because the methodological error is worth more than the
    // finding was: a single non-reproduced observation was treated as a
    // controlled experiment. Re-run the failing state before believing any
    // bisect taken while another agent is writing shared headers.
    stampRegion("PoseWalkSetup");
    // A walk with no solver step has its snapshot inputs final already.
    if (lastSolverStep == nullptr) {
        launchSnapshot();
    }
    // The scene-up direction, resolved at most once per generation: every
    // sceneUp aim used to read stage metadata for itself, and metadata
    // cannot change mid-generation.
    GfVec3d sceneUpDirection(0, 1, 0);
    bool sceneUpDirectionRead = false;
    for (const _PoseStep &step : _poseSteps) {
        if (step.solverBatch) {
            _SolverBatch &batch = _solverBatches[step.index];
            RIGEXEC_PROFILE_SCOPE_CAT(
                _profiler,
                "SolverBatch L" + std::to_string(batch.level) + " B" +
                std::to_string(step.index), "pose");
            std::map<SdfPath, RigExecPointFrame> candidates;
            // The request this solver's aggregate comes from. A follower's
            // leader evaluated it at the leader's step, just before this one.
            _SolverBatch &request = _solverBatches[batch.leader];
            if (batch.leader == step.index && batch.followers.empty()) {
                // The batch-specific overrides only. baseOverrides is a pure
                // function of `time` -- and (time, tail) uniquely determines
                // the full exec input -- only while no interactive overrides
                // are held: a held drag rides into baseOverrides beside the
                // chains, so a tail hit under a drag would answer with another
                // override set's base. The Find and the Store below both stand
                // down then; the repeat-pass frames the cache exists for carry
                // no overrides.
                std::vector<RigExecValueOverride> tail;
                {
                    RIGEXEC_PROFILE_SCOPE_CAT(
                        _profiler, "SolverBatch.Inputs", "pose");
                    tail.reserve(batch.dependencies.size() +
                                 batch.frameInputs.size() +
                                 batch.restInputs.size());
                }
                if (!appendBatchTail(batch, &tail)) return pose;
                // A batch snapshot is a pure function of (time, inputs).
                // Inputs are themselves assembled from the seed snapshot,
                // prior-level aggregates, and rest frames -- all deterministic
                // for a given frame -- so a recently-computed entry is
                // reusable. The tap-level dirty flag is drained, not consulted
                // (it fires across sibling frames sharing the system); genuine
                // edits ride batch.dirty.
                batch.taps->ConsumeDirty();
                _SnapshotCache::Entry *hit = nullptr;
                {
                    RIGEXEC_PROFILE_SCOPE_CAT(
                        _profiler, "SolverBatch.Find", "pose");
                    // No cache under a held drag: the key is (tail, time) but
                    // the result depends on baseOverrides too, which the drag
                    // is part of. See the note where the tail is assembled.
                    hit = (!batch.dirty && _interactiveOverrides.empty())
                        ? batch.cache.Find(tail, time)
                        : nullptr;
                }
                if (hit != nullptr) {
                    batch.snapshot = hit->snapshot;
                } else {
                    RIGEXEC_PROFILE_SCOPE_CAT(
                        _profiler, "ExecEvaluate", "exec");
                    std::vector<RigExecValueOverride> inputs;
                    inputs.reserve(baseOverrides.size() + tail.size());
                    inputs.insert(inputs.end(), baseOverrides.begin(),
                                  baseOverrides.end());
                    inputs.insert(inputs.end(), tail.begin(), tail.end());
                    const RigExecSnapshot refreshed =
                        batch.taps->Evaluate(time, inputs);
                    if (!refreshed.IsValid() || !refreshed.IsComplete()) {
                        batch.dirty = true;
                        pose.solverOverridesConverged = false;
                        pose.diagnostics.push_back(
                            "solver dependency level evaluation incomplete");
                        return pose;
                    }
                    if (_interactiveOverrides.empty()) {
                        batch.cache.Store(tail, time, refreshed);
                        // Cleared only beside the store: an edit that dirtied
                        // the batch ahead of a drag must still force a
                        // re-resolve once the drag releases.
                        batch.dirty = false;
                    }
                    batch.snapshot = refreshed;
                    // ChangeTime/compilation can notify while Evaluate runs.
                    // The successful snapshot already includes those
                    // invalidations.
                    batch.taps->ConsumeDirty();
                    // Count the exec pulls actually performed, not the batches
                    // the schedule visited: a cache hit reuses the stored
                    // snapshot without evaluating, and the counter is how the
                    // suite proves it (an unchanged pull evaluates nothing).
                    pose.solverEvaluations += batch.solvers.size();
                }
            } else if (batch.leader == step.index) {
                // A SHARED request (see "one exec request per run" in
                // Compile): the same steps as above, over every member.
                // Each member's own tail is built and kept apart first, in
                // walk order, because it is that member's input fingerprint;
                // the request's tail is their union, in the same order, with
                // an override two members both push taken once (Compile groups
                // them only where both push the same value).
                std::vector<size_t> members{step.index};
                members.insert(members.end(), batch.followers.begin(),
                               batch.followers.end());
                std::vector<std::vector<RigExecValueOverride>> own(
                    members.size());
                std::vector<RigExecValueOverride> tail;
                {
                    RIGEXEC_PROFILE_SCOPE_CAT(
                        _profiler, "SolverBatch.Inputs", "pose");
                    for (size_t m = 0; m < members.size(); ++m) {
                        const _SolverBatch &member = _solverBatches[members[m]];
                        own[m].reserve(member.dependencies.size() +
                                       member.frameInputs.size() +
                                       member.restInputs.size());
                    }
                }
                std::set<std::tuple<SdfPath, TfToken, TfToken>> pushed;
                for (size_t m = 0; m < members.size(); ++m) {
                    if (!appendBatchTail(_solverBatches[members[m]], &own[m])) {
                        return pose;
                    }
                    for (const RigExecValueOverride &o : own[m]) {
                        if (pushed.emplace(o.prim, o.computation, o.attribute)
                                .second) {
                            tail.push_back(o);
                        }
                    }
                }
                batch.taps->ConsumeDirty();
                const bool dragging = !_interactiveOverrides.empty();
                bool anyDirty = false;
                for (size_t index : members) {
                    anyDirty = anyDirty || _solverBatches[index].dirty;
                }
                _SnapshotCache::Entry *hit = nullptr;
                {
                    RIGEXEC_PROFILE_SCOPE_CAT(
                        _profiler, "SolverBatch.Find", "pose");
                    hit = (!anyDirty && !dragging)
                        ? batch.requestCache.Find(tail, time)
                        : nullptr;
                }
                if (hit != nullptr) {
                    batch.snapshot = hit->snapshot;
                    // Keep each fingerprint's recency where a request of its
                    // own would have left it: that was a hit too.
                    for (size_t m = 0; m < members.size(); ++m) {
                        _solverBatches[members[m]].cache.Find(own[m], time);
                    }
                } else {
                    RIGEXEC_PROFILE_SCOPE_CAT(
                        _profiler, "ExecEvaluate", "exec");
                    std::vector<RigExecValueOverride> inputs;
                    inputs.reserve(baseOverrides.size() + tail.size());
                    inputs.insert(inputs.end(), baseOverrides.begin(),
                                  baseOverrides.end());
                    inputs.insert(inputs.end(), tail.begin(), tail.end());
                    const RigExecSnapshot refreshed =
                        batch.taps->Evaluate(time, inputs);
                    if (!refreshed.IsValid() || !refreshed.IsComplete()) {
                        for (size_t index : members) {
                            _solverBatches[index].dirty = true;
                        }
                        pose.solverOverridesConverged = false;
                        pose.diagnostics.push_back(
                            "solver dependency level evaluation incomplete");
                        return pose;
                    }
                    // The counter, per member, as a request of its own would
                    // have kept it: a member re-solves when it was dirtied, when
                    // a drag is held, or when its own tail misses its own
                    // fingerprint. The others ride along unchanged.
                    for (size_t m = 0; m < members.size(); ++m) {
                        _SolverBatch &member = _solverBatches[members[m]];
                        const bool resolved =
                            dragging || member.dirty ||
                            member.cache.Find(own[m], time) == nullptr;
                        if (resolved) {
                            pose.solverEvaluations += member.solvers.size();
                        }
                        if (!dragging) {
                            member.cache.Store(std::move(own[m]), time,
                                               RigExecSnapshot());
                            member.dirty = false;
                        }
                    }
                    if (!dragging) {
                        batch.requestCache.Store(tail, time, refreshed);
                    }
                    batch.snapshot = refreshed;
                    batch.taps->ConsumeDirty();
                }
            }
            const RigExecSnapshot &values = request.snapshot;
            if (visitedSolverLevels.insert(batch.level).second) {
                ++pose.solverOverrideRounds;
            }
            for (const auto &[solver, tap] : batch.solvers) {
                const RigExecPointFrameArray aggregate =
                    values.Get<RigExecPointFrameArray>(tap);
                solvedAggregates[solver] = aggregate;
                const auto joints = _solverJoints.find(solver);
                if (joints == _solverJoints.end()) {
                    continue;
                }
                for (const auto &[joint, element] : joints->second) {
                    if (element < 0 || size_t(element) >= aggregate.GetSize()) {
                        fallbackJoints.push_back({joint, {solver, element}});
                        continue;
                    }
                    const RigExecPointFrame frame =
                        RigExecExtractElementFrame(&aggregate, size_t(element));
                    candidates[joint] = frame;
                    lastPublishingWriter[joint] = solver;
                }
            }
            if (!candidates.empty()) {
                commitConstraintFrames(SdfPath(), candidates, true);
            }
            if (!candidates.empty() && !_chainPlan.snapshots.empty()) {
                // A solver checkpoint: the joint as THIS writer left it, for
                // a reader whose read phase names it. Skipped whole unless
                // SOMETHING on the rig asked for a checkpoint -- recordFrame
                // early-outs per pair anyway, but that is still two map
                // lookups per bound joint per frame on a rig that never names
                // a solver, which is every rig that does not stack.
                for (const auto &[solver, tap] : batch.solvers) {
                    const auto joints = _solverJoints.find(solver);
                    if (joints == _solverJoints.end()) {
                        continue;
                    }
                    for (const auto &[joint, element] : joints->second) {
                        if (candidates.count(joint)) {
                            recordFrame(joint, solver);
                        }
                    }
                }
            }
            // The last solver commit: every snapshot override is final.
            if (&step == lastSolverStep) {
                launchSnapshot();
            }
            continue;
        }
        const _FrameConstraint &constraint = _frameConstraints[step.index];
        RIGEXEC_PROFILE_SCOPE_CAT(
            _profiler,
            constraint.schemaType.GetString() + " " +
                constraint.moverPath.GetString(),
            "pose");
        for (const SdfPath &target : constraint.targets) {
            if (!refreshPoseProvider(target)) return pose;
        }
        if (!refreshPoseProvider(constraint.weightObject)) return pose;
        const UsdPrim prim = _stage->GetPrimAtPath(constraint.moverPath);
        if (!prim) {
            continue;
        }
        const bool enabled = _ResolvedRead(
            _resolvedInputs, prim, _kDynEnabled, true, time);
        if (!enabled) {
            for (const SdfPath &target : constraint.targets) {
                recordFrame(target, constraint.moverPath);
            }
            continue;
        }
        double weight = 1.0;
        if (!constraint.weightObject.IsEmpty() &&
            constraint.pointsTarget.IsEmpty()) {
            std::vector<float> resolvedWeight;
            std::string error;
            if (!_ResolveWeights(constraint.weightObject, 1, time,
                                 &resolvedWeight, &error) ||
                resolvedWeight.size() != 1) {
                pose.diagnostics.push_back(
                    constraint.moverPath.GetString() + ": " + error +
                    "; constraint passed through");
                for (const SdfPath &target : constraint.targets) {
                    recordFrame(target, constraint.moverPath);
                }
                continue;
            }
            weight = resolvedWeight[0];
        } else if (constraint.weightObject.IsEmpty()) {
            weight = _ResolvedRead(
                _resolvedInputs, prim, _kDynDefaultWeight, 1.0f, time);
            if (!std::isfinite(weight) || weight < 0.0 || weight > 1.0) {
                pose.diagnostics.push_back(
                    constraint.moverPath.GetString() +
                    " has inputs:defaultWeight outside finite [0, 1]; "
                    "constraint passed through");
                for (const SdfPath &target : constraint.targets) {
                    recordFrame(target, constraint.moverPath);
                }
                continue;
            }
        }
        // A zero/negative envelope is an exact dormant pass-through. Do this
        // before resolving sources, effectors, or poles so malformed
        // disconnected inputs cannot make a disabled constraint fail.
        // A geometry-domain object is per point and resolves after the solve,
        // so it cannot short-circuit here. A transform object has already
        // resolved its one element and may use the ordinary dormant path.
        if (weight <= 0.0 &&
            (constraint.pointsTarget.IsEmpty() ||
             constraint.weightObject.IsEmpty())) {
            if (!constraint.pointsTarget.IsEmpty())
                constraintDeltas[constraint.moverPath] = GfMatrix4d(1.0);
            for (const SdfPath &target : constraint.targets) {
                recordFrame(target, constraint.moverPath);
            }
            continue;
        }

        // The envelope is applied exactly ONCE. In the transform domain the
        // kernel's per-channel blend carries it. In the geometry domain the
        // per-point lerp does, so the solve must run UNWEIGHTED and hand back
        // the full-strength delta -- passing the envelope to both would
        // square it, and 0.5 would come out as 0.25 on points.
        const double solveWeight =
            constraint.pointsTarget.IsEmpty() ? weight : 1.0;

        if (constraint.schemaType == _kDynSingleChainIkConstraint) {
            std::vector<RigExecPointFrame> chain;
            chain.reserve(constraint.ikChain.size());
            bool inputsValid = true;
            for (const SdfPath &joint : constraint.ikChain) {
                const auto fi = _providerIndex.find(joint);
                if (fi == _providerIndex.end() || !finalLive[fi->second]) {
                    pose.diagnostics.push_back(
                        constraint.moverPath.GetString() +
                        " has no current frame for " + joint.GetString());
                    inputsValid = false;
                    break;
                }
                chain.push_back(finalFrames[fi->second]);
            }
            RigExecPointFrame effector;
            if (inputsValid &&
                !resolveBinding(constraint.effector, &effector)) {
                pose.diagnostics.push_back(
                    constraint.moverPath.GetString() +
                    " could not resolve its effector; constraint passed "
                    "through");
                inputsValid = false;
            }

            RigExecSingleChainIkParams params;
            TfToken orientationMode = _kDynAimX;
            if (auto a = prim.GetAttribute(_kDynOrientationMode))
                a.Get(&orientationMode);
            params.preserveJointOrientation = orientationMode == _kDynPreserve;
            TfToken solverMode = _kDynRotatePlane;
            if (const UsdAttribute a =
                    prim.GetAttribute(_kDynSolverMode)) {
                a.Get(&solverMode);
            }
            params.mode = solverMode == _kDynSingleChain
                ? RigExecSingleChainIkMode::SingleChain
                : RigExecSingleChainIkMode::RotatePlane;
            TfToken poleMode = _kDynVector;
            if (const UsdAttribute a =
                    prim.GetAttribute(_kDynPoleVectorMode)) {
                a.Get(&poleMode);
            }
            params.weight = weight;
            params.stretch = _ResolvedRead(_resolvedInputs, prim, TfToken("inputs:stretch"), 0.0f, time);
            if (params.mode == RigExecSingleChainIkMode::RotatePlane) {
                params.pole = _ResolvedRead(
                    _resolvedInputs, prim, _kDynPoleVector,
                    GfVec3d(0, 1, 0), time);
                params.twistDegrees = _ResolvedRead(
                    _resolvedInputs, prim, _kDynTwistDegrees, 0.0,
                    time);
            }
            if (inputsValid &&
                params.mode == RigExecSingleChainIkMode::RotatePlane &&
                poleMode == _kDynObject) {
                if (constraint.poleObjects.empty()) {
                    pose.diagnostics.push_back(
                        constraint.moverPath.GetString() +
                        " uses object pole mode with no pole-vector objects; "
                        "constraint passed through");
                    inputsValid = false;
                }
                std::vector<double> poleWeights;
                if (inputsValid &&
                    !readWeights(prim, _kDynPoleVectorWeights,
                                 constraint.poleObjects.size(), &poleWeights)) {
                    inputsValid = false;
                }
                GfVec3d polePoint(0);
                double total = 0;
                for (size_t i = 0;
                     inputsValid && i < constraint.poleObjects.size(); ++i) {
                    RigExecPointFrame poleFrame;
                    if (!resolveBinding(constraint.poleObjects[i],
                                        &poleFrame) ||
                        !std::isfinite(poleWeights[i]) ||
                        poleWeights[i] < 0) {
                        pose.diagnostics.push_back(
                            constraint.moverPath.GetString() +
                            " has an invalid pole-vector source or weight; "
                            "constraint passed through");
                        inputsValid = false;
                        break;
                    }
                    polePoint += poleFrame.Origin() * poleWeights[i];
                    total += poleWeights[i];
                }
                if (inputsValid && total <= 0) {
                    pose.diagnostics.push_back(
                        constraint.moverPath.GetString() +
                        " has zero total pole-vector weight; constraint passed "
                        "through");
                    inputsValid = false;
                }
                if (inputsValid) {
                    params.pole = polePoint / total;
                }
            }

            TfToken evaluationMode = _kDynNeverTs;
            if (const UsdAttribute a =
                    prim.GetAttribute(_kDynEvaluationMode)) {
                a.Get(&evaluationMode);
            }
            std::vector<RigExecPointFrame> solveChain = chain;
            const bool useAnimatedTs =
                evaluationMode == _kDynAlwaysTs ||
                (evaluationMode == _kDynAutoDetect &&
                 _IkUsesAnimatedTs(constraint.ikChain));
            if (inputsValid && !useAnimatedTs) {
                std::vector<RigExecPointFrame> rest;
                rest.reserve(constraint.ikChain.size());
                for (size_t i = 0; i < constraint.ikChain.size(); ++i) {
                    const SdfPath &joint = constraint.ikChain[i];
                    // The incoming frame IS the rest reference where a step
                    // below this constraint wrote the joint; the authored
                    // rest everywhere else.
                    if (i < constraint.ikRestLive.size() &&
                        constraint.ikRestLive[i] && i < chain.size()) {
                        rest.push_back(chain[i]);
                        continue;
                    }
                    const auto fi = _providerIndex.find(joint);
                    if (fi == _providerIndex.end() || !restLive[fi->second]) {
                        inputsValid = false;
                        break;
                    }
                    rest.push_back(restFrames[fi->second]);
                }
                if (!inputsValid ||
                    !RigExecPrepareRestDerivedIkChain(
                        chain, rest, &solveChain)) {
                    pose.diagnostics.push_back(
                        constraint.moverPath.GetString() +
                        " could not prepare rest-derived IK inputs; "
                        "constraint passed through");
                    inputsValid = false;
                }
            }

            std::vector<RigExecPointFrame> solved;
            if (inputsValid) {
                solved = RigExecSolveSingleChainIk(
                    solveChain, effector, params);
            }
            if (inputsValid &&
                (solved.size() != constraint.ikChain.size() ||
                 std::any_of(
                     solved.begin(), solved.end(),
                     [](const RigExecPointFrame &frame) {
                         return !_IsUsableConstraintFrame(frame);
                     }))) {
                pose.diagnostics.push_back(
                    constraint.moverPath.GetString() +
                    " failed to solve its joint chain; constraint passed "
                    "through atomically");
                inputsValid = false;
            }
            if (inputsValid) {
                std::map<SdfPath, RigExecPointFrame> candidates;
                for (size_t i = 0; i < solved.size(); ++i) {
                    candidates[constraint.ikChain[i]] = solved[i];
                }
                commitConstraintFrames(constraint.moverPath, candidates);
            }
            for (size_t i = 0; i < constraint.ikChain.size(); ++i) {
                recordFrame(constraint.ikChain[i], constraint.moverPath);
            }
            continue;
        }

        std::vector<RigExecConstraintSource> sources;
        if (!buildSources(constraint, prim, &sources)) {
            pose.diagnostics.push_back(
                constraint.moverPath.GetString() +
                " has unusable constraint inputs; constraint passed through");
            recordFrame(constraint.targets[0], constraint.moverPath);
            continue;
        }
        const auto inputIt = _providerIndex.find(constraint.targets[0]);
        const RigExecPointFrame inputFrame =
            (inputIt != _providerIndex.end() && finalLive[inputIt->second])
                ? finalFrames[inputIt->second]
                : RigExecPointFrame();
        RigExecPointFrame candidate = inputFrame;
        bool candidateReady = true;
        // Which mask triple this operator reads is a table property: masks
        // are addressed by (group, axis), so Position reads the translation
        // triple and Rotation and Aim read the rotation one, rather than all
        // three sharing an inputs:affectX that means something different in
        // each.
        const _ConstraintHandler *solveHandler =
            _FindConstraintHandler(constraint.schemaType);
        RigExecConstraintAxisMask affect;
        if (constraint.masksStatic) {
            switch (solveHandler ? solveHandler->maskGroup
                                 : _ChannelGroup::None) {
            case _ChannelGroup::Translation:
                affect = constraint.precompTranslation;
                break;
            case _ChannelGroup::Rotation:
                affect = constraint.precompRotation;
                break;
            case _ChannelGroup::Scale:
                affect = constraint.precompScale;
                break;
            case _ChannelGroup::All:
            case _ChannelGroup::None:
            default:
                affect = RigExecConstraintAxisMask();
                break;
            }
        } else {
            affect = _ReadGroupMask(
                _resolvedInputs, prim,
                solveHandler ? solveHandler->maskGroup : _ChannelGroup::None,
                time);
        }
        TfToken orderToken = _kDynXyz;
        if (const UsdAttribute a =
                prim.GetAttribute(_kDynRotationOrder)) {
            a.Get(&orderToken);
        }
        const RigExecEulerOrder order =
            _ParseConstraintEulerOrder(orderToken);


        // The kernel-backed operators solve through the registry: one row
        // per operator, so adding an operator is a table entry rather than
        // another arm here. Aim falls through to the inline branch below,
        // which resolves a world-up binding the uniform context cannot carry.
        // A matrix mover in the transform domain: M' = T_w M, where T is
        // the same rest->posed map its geometry twin uses, measured in
        // rigExec:transformSpace by the shared helper.
        //
        // The falloff is applied to the frame's four LANDMARK POINTS with
        // the identical rule the point kernel uses, rather than to a matrix
        // built from it. That is what makes a transform placed where a point
        // is land exactly where that point lands, for linear and radial
        // alike, instead of merely close.
        if (constraint.schemaType == "RigExecMatrixMover") {
            auto mapOf = [&](const _FrameSourceBinding &binding,
                             GfMatrix4d *out) {
                RigExecPointFrame posed;
                if (!resolveBinding(binding, &posed)) return false;
                const auto ri = _providerIndex.find(binding.sourcePath);
                const bool haveRest = ri != _providerIndex.end() &&
                                      restLive[ri->second] &&
                                      restFrames[ri->second].IsValid();
                const auto &landmarks = haveRest
                    ? restFrames[ri->second].points
                    : RigExecIdentityLandmarks();
                return RigExecPointsToMatrix(landmarks, posed, out);
            };
            GfMatrix4d transform(1.0);
            if (constraint.sources.empty() ||
                !mapOf(constraint.sources[0], &transform)) {
                pose.diagnostics.push_back(
                    constraint.moverPath.GetString() +
                    " could not resolve rigExec:transform; mover passed "
                    "through");
                for (const SdfPath &target : constraint.targets) {
                    recordFrame(target, constraint.moverPath);
                }
                continue;
            }
            if (constraint.sources.size() > 1) {
                GfMatrix4d space(1.0);
                if (!mapOf(constraint.sources[1], &space)) {
                    pose.diagnostics.push_back(
                        constraint.moverPath.GetString() +
                        " could not resolve rigExec:transformSpace; mover "
                        "passed through");
                    for (const SdfPath &target : constraint.targets) {
                        recordFrame(target, constraint.moverPath);
                    }
                    continue;
                }
                transform = RigExecMeasureInSpace(transform, space);
            }
            RigExecPointFrame solved = inputFrame;
            if (constraint.radialBlend) {
                const GfMatrix4d partial =
                    RigExecPartialTransform(transform, weight);
                for (GfVec3d &q : solved.points) {
                    q = partial.TransformAffine(q);
                }
            } else {
                for (GfVec3d &q : solved.points) {
                    q = q + weight * (transform.TransformAffine(q) - q);
                }
            }
            candidate = solved;
            candidateReady = true;
        } else if (solveHandler && solveHandler->solve) {
            _ConstraintSolveContext solveContext;
            solveContext.resolved = &_resolvedInputs;
            solveContext.prim = prim;
            solveContext.time = time;
            solveContext.inputFrame = inputFrame;
            solveContext.sources = &sources;
            solveContext.affect = affect;
            solveContext.order = order;
            solveContext.weight = solveWeight;
            solveContext.masksStatic = constraint.masksStatic;
            solveContext.precompTranslation = constraint.precompTranslation;
            solveContext.precompRotation = constraint.precompRotation;
            solveContext.precompScale = constraint.precompScale;
            solveContext.blendShear = constraint.blendShear;
            // rigExec:space. The carry is the SEED's answer for the space,
            // default^-1 * posed, exactly as the space switch takes it --
            // and hasCarry, not "carry == identity": a constraint naming
            // no space hands the kernel a null carry and takes its
            // untouched branch, which is bit for bit what it did before.
            GfMatrix4d carry(1.0);
            bool hasCarry = false;
            if (constraint.spacePosedTap >= 0 &&
                constraint.spaceDefaultTap >= 0 && seedSnapshot.IsValid()) {
                const auto matrixOfTap = [&](RigExecTapId tap,
                                             GfMatrix4d *m) {
                    const RigExecPointFrame frame =
                        seedSnapshot.Get<RigExecPointFrame>(tap);
                    return frame.IsValid() && !frame.IsDegenerate() &&
                           RigExecPointsToMatrix(RigExecIdentityLandmarks(),
                                                 frame.points, m);
                };
                GfMatrix4d spacePosed(1.0), spaceDefault(1.0);
                if (matrixOfTap(constraint.spacePosedTap, &spacePosed) &&
                    matrixOfTap(constraint.spaceDefaultTap, &spaceDefault)) {
                    carry = spaceDefault.GetInverse() * spacePosed;
                    hasCarry = true;
                } else {
                    pose.diagnostics.push_back(
                        constraint.moverPath.GetString() +
                        " could not resolve rigExec:space; the axis mask "
                        "runs without the carry");
                }
            }
            solveContext.carry = hasCarry ? &carry : nullptr;
            candidate = solveHandler->solve(solveContext);
        } else {
            // Aim uses the same weighted source set, reduced to the target
            // point specified by FBX's AimAtObjects contract.
            GfVec3d target(0);
            double total = 0;
            for (const RigExecConstraintSource &source : sources) {
                if (!std::isfinite(source.normalizedWeight) ||
                    source.normalizedWeight < 0) {
                    pose.diagnostics.push_back(
                        constraint.moverPath.GetString() +
                        " has an invalid source weight; constraint passed "
                        "through");
                    candidateReady = false;
                    break;
                }
                target += source.frame.Origin() * source.normalizedWeight;
                total += source.normalizedWeight;
            }
            if (candidateReady && total > 0) {
                target /= total;
                RigExecAimConstraintParams params;
                const UsdAttribute aimVectorAttr =
                    prim.GetAttribute(_kDynAimVector);
                params.localAimVector = _ResolvedRead(
                    _resolvedInputs, prim, _kDynAimVector,
                    GfVec3d(1, 0, 0), time);
                // Existing assets author aimAxis but predate aimVector. Keep
                // that authored meaning until they opt into the vector form.
                if (!aimVectorAttr ||
                    !aimVectorAttr.HasAuthoredValueOpinion()) {
                    TfToken axis = _kDynAxisX;
                    if (const UsdAttribute a = prim.GetAttribute(
                            _kDynAimAxis)) {
                        a.Get(&axis);
                    }
                    params.localAimVector =
                        axis == _kDynAxisY ? GfVec3d(0, 1, 0)
                                    : axis == _kDynAxisZ ? GfVec3d(0, 0, 1)
                                                  : GfVec3d(1, 0, 0);
                }
                params.localUpVector = _ResolvedRead(
                    _resolvedInputs, prim, _kDynUpVector,
                    GfVec3d(0, 1, 0), time);
                params.rotationOffsetDegrees = _ResolvedRead(
                    _resolvedInputs, prim, _kDynRotationOffset,
                    GfVec3d(0), time);
                params.affectRotation = affect;
                params.rotationOrder = order;
                params.weight = solveWeight;

                TfToken worldUpType = _kDynWorldUpNone;
                if (const UsdAttribute a = prim.GetAttribute(
                        _kDynWorldUpType)) {
                    a.Get(&worldUpType);
                }
                SdfPathVector authoredSources;
                if (const UsdRelationship rel = prim.GetRelationship(
                        _kDynSources)) {
                    rel.GetTargets(&authoredSources);
                }
                // The legacy aimTarget/aimAxis contract preserves input up.
                // FBX WorldUpType=None is the distinct minimum-swing mode.
                params.preserveInputUp = authoredSources.empty();
                const GfVec3d authoredWorldUp = _ResolvedRead(
                    _resolvedInputs, prim, _kDynWorldUpVector,
                    GfVec3d(0, 1, 0), time);
                if (worldUpType == _kDynWorldUpSceneUp) {
                    if (!sceneUpDirectionRead) {
                        const std::string up =
                            UsdGeomGetStageUpAxis(_stage).GetString();
                        sceneUpDirection =
                            (up == "Z" || up == "z")
                                ? GfVec3d(0, 0, 1)
                                : GfVec3d(0, 1, 0);
                        sceneUpDirectionRead = true;
                    }
                    params.worldUpDirection = sceneUpDirection;
                } else if (worldUpType == _kDynVector) {
                    params.worldUpDirection = authoredWorldUp;
                } else if (worldUpType == _kDynWorldUpObjectUp) {
                    // FBX ObjectUp without a reference object uses the
                    // world origin as the object point.
                    if (constraint.worldUpObject.sourcePath.IsEmpty()) {
                        params.worldUpDirection = -inputFrame.Origin();
                    } else {
                        RigExecPointFrame upObject;
                        if (!resolveBinding(constraint.worldUpObject,
                                            &upObject)) {
                            pose.diagnostics.push_back(
                                constraint.moverPath.GetString() +
                                " could not resolve its world-up object; "
                                "constraint passed through");
                            candidateReady = false;
                        } else {
                            params.worldUpDirection =
                                upObject.Origin() - inputFrame.Origin();
                        }
                    }
                } else if (worldUpType == _kDynWorldUpObjectRotationUp) {
                    // With no object, FBX applies WorldUpVector directly in
                    // world space rather than treating a missing binding as
                    // a failed constraint.
                    if (constraint.worldUpObject.sourcePath.IsEmpty()) {
                        params.worldUpDirection = authoredWorldUp;
                    } else {
                        RigExecPointFrame upObject;
                        if (!resolveBinding(constraint.worldUpObject,
                                            &upObject)) {
                            pose.diagnostics.push_back(
                                constraint.moverPath.GetString() +
                                " could not resolve its world-up object; "
                                "constraint passed through");
                            candidateReady = false;
                        }
                        if (candidateReady) {
                            GfMatrix4d upMatrix(1.0);
                            if (!RigExecPointsToMatrix(
                                    RigExecIdentityLandmarks(),
                                    upObject.points, &upMatrix)) {
                                pose.diagnostics.push_back(
                                    constraint.moverPath.GetString() +
                                    " has a degenerate world-up object");
                                candidateReady = false;
                            } else {
                                // rigExec:worldUpRotationOnly takes the up
                                // direction from the up object's rotation
                                // alone: ExtractRotation on a scaled frame
                                // does not return its rotation, and under a
                                // scaled rig root the derived up can swing
                                // far enough to turn an aim-constrained
                                // foot. Off is the original extraction.
                                params.worldUpDirection =
                                    (constraint.worldUpRotationOnly
                                         ? upMatrix.GetOrthonormalized(false)
                                         : upMatrix)
                                        .ExtractRotation()
                                        .TransformDir(authoredWorldUp);
                            }
                        }
                    }
                }
                if (candidateReady) {
                    candidate = RigExecApplyAimConstraint(
                        inputFrame, target, params);
                }
            }
        }
        if (candidateReady && !constraint.pointsTarget.IsEmpty()) {
            // The GEOMETRY domain. The solve produced the same full-strength
            // frame the transform domain would publish; the delta against the
            // prim's own base frame is what the points ride.
            //     D = F_solved * F_base^-1
            // Stashed here and consumed after the pose walk, the same
            // in-memory hand-off finalMatrices performs for a "final" read
            // phase. The prim's transform is NOT revised: a geometry-domain
            // constraint writes points and nothing else.
            GfMatrix4d baseMatrix(1.0);
            GfMatrix4d solvedMatrix(1.0);
            if (_IsUsableConstraintFrame(candidate) &&
                frameFromXform(constraint.targets[0], nullptr, &baseMatrix) &&
                std::isfinite(baseMatrix.GetDeterminant()) &&
                baseMatrix.GetDeterminant() != 0.0 &&
                RigExecPointsToMatrix(RigExecIdentityLandmarks(),
                                      candidate.points, &solvedMatrix)) {
                constraintDeltas[constraint.moverPath] =
                    solvedMatrix * baseMatrix.GetInverse();
            } else {
                pose.diagnostics.push_back(
                    constraint.moverPath.GetString() +
                    " could not measure its delta against " +
                    constraint.targets[0].GetString() +
                    "; constraint passed through");
            }
        } else if (candidateReady) {
            commitConstraintFrames(
                constraint.moverPath,
                {{constraint.targets[0], candidate}});
        }
        if (constraint.pointsTarget.IsEmpty()) {
            recordFrame(constraint.targets[0], constraint.moverPath);
        }
    }

    for (const auto &[provider, tap] : _firstFramePoseFrames) {
        if (!refreshPoseProvider(provider)) return pose;
    }

    // An incomplete solver is an authoring gap, not a silent one: name every
    // writer that published nothing so a rig mid-edit explains itself.
    // Accumulated in walk order, stable-sorted by joint path, so the lines
    // are in the same order the baked epilogue produces them in.
    std::stable_sort(fallbackJoints.begin(), fallbackJoints.end(),
                     [](const std::pair<SdfPath, std::pair<SdfPath, int>> &a,
                        const std::pair<SdfPath, std::pair<SdfPath, int>> &b) {
                         return a.first < b.first;
                     });
    for (const auto &[jointPath, writer] : fallbackJoints) {
        const auto kept = lastPublishingWriter.find(jointPath);
        pose.diagnostics.push_back(
            "solver " + writer.first.GetString() + " published no element " +
            std::to_string(writer.second) + " for joint " +
            jointPath.GetString() + "; " +
            (kept == lastPublishingWriter.end()
                 ? std::string("joint fell back to its rest chain")
                 : "the joint keeps the frame " + kept->second.GetString() +
                       " left"));
    }

    // 1. Transforms and solvers through OpenExec. An incomplete snapshot
    // means some computation failed to compile or evaluate; refusing to
    // continue prevents default-constructed values from masquerading as
    // results (spec §6.6).
    RigExecSnapshot snapshot;
    // Stored only on success and only with no drag held, by whichever
    // thread computed it -- this one, after the join.
    const auto storeAuthSnapshot = [&]() {
        if (snapshot.IsValid() && snapshot.IsComplete() &&
            _interactiveOverrides.empty()) {
            if (_authSnapTimeKeyed.size() >= 4)
                _authSnapTimeKeyed.erase(_authSnapTimeKeyed.begin());
            _authSnapTimeKeyed.emplace(time, snapshot);
            _authSnapshotDirty = false;
        }
    };
    if (snapshotInFlight) {
        // The overlapped request (see the snapshot overlap above the walk).
        {
            RIGEXEC_PROFILE_SCOPE_CAT(
                _profiler, "AuthoritativeSnapshot.Join", "exec");
            snapshotDispatcher.Wait();
        }
        snapshotInFlight = false;
        snapshot = std::move(overlappedSnapshot);
        storeAuthSnapshot();
    } else {
        RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "AuthoritativeSnapshot", "exec");
        _taps->ConsumeDirty();
        // jointOverrides is a pure function of (epoch, time) -- the solver
        // aggregates, base frames, and falloff LUTs are all deterministic
        // given the time -- while no interactive overrides are held. A held
        // drag rides into baseOverrides beside the chains, so a time-keyed
        // hit under a drag would answer with another override set's
        // snapshot. The find and the store below both stand down then.
        // Genuine stage edits set _authSnapshotDirty; the tap-level dirty
        // flag is drained, not consulted (it fires across sibling frames
        // sharing the system).
        bool authResolved = false;
        if (!_authSnapshotDirty && _interactiveOverrides.empty()) {
            const auto tk = _authSnapTimeKeyed.find(time);
            if (tk != _authSnapTimeKeyed.end()) {
                snapshot = tk->second;
                authResolved = true;
            }
        }
        if (!authResolved) {
            snapshot = evaluateAuthSnapshot(assembleAuthOverrides());
            storeAuthSnapshot();
        }
    }
    if (!snapshot.IsValid() || !snapshot.IsComplete()) {
        pose.diagnostics.push_back(
            snapshot.IsValid() ? "snapshot incomplete: missing tap values"
                               : "snapshot evaluation failed");
        return pose;
    }

    // Publish final provider matrices after the atomic pose walk.
    // MEASURED 2026-09-13, biped: the three publish loops below cost 3.3-3.9
    // ms/frame -- 24% of a no-change evaluate, more than AuthoritativeSnapshot
    // -- and until these scopes existed NONE of it appeared in the profile.
    // Summing the profiler's rows accounted for only 76% of the floor and
    // nobody had asked where the rest went. The cost is 252 x 3 std::map
    // inserts into jointFramesBase/Final/MatricesFinal, done unconditionally
    // for joints that did not move, plus this provider walk over every
    // finalFrames entry. Scoped separately from the exec snapshot above it
    // because the two want opposite fixes: that one is exec, this one is a
    // container choice (see docs/superpowers/specs/
    // 2026-09-13-sparse-evaluation-design.md, Gate 3).
    {
    RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "PublishProviders", "publish");
    for (std::size_t i = 0; i < _providerPaths.size(); ++i) {
        if (!finalLive[i]) continue;
        const SdfPath &provider = _providerPaths[i];
        const RigExecPointFrame &frame = finalFrames[i];
        if (_xformDerivedProviders.count(provider)) {
            if (!_IsUsableConstraintFrame(frame)) {
                pose.diagnostics.push_back(
                    "constraint target " + provider.GetString() +
                    " has an invalid final frame; transform omitted");
                continue;
            }
            GfMatrix4d revised(1.0);
            if (RigExecPointsToMatrix(RigExecIdentityLandmarks(),
                                      frame.points, &revised)) {
                pose.providerXforms[provider] = revised;
                pose.providerBaseXforms[provider] =
                    xformDerivedBases[provider];
            }
        }
        if (restLive[i] &&
            _IsUsableConstraintFrame(restFrames[i]) &&
            _IsUsableConstraintFrame(frame)) {
            GfMatrix4d matrix(1.0);
            if (RigExecPointsToMatrix(restFrames[i].points, frame.points,
                                      &matrix)) {
                finalMatrices[provider] = matrix;
                _chainSnapshots.RecordFinal(provider, VtValue(matrix));
            }
        }
    }
    }

    // 3. Base and final transform revisions plus paired matrices. A
    // solver-posed joint's base value was supplied as an override above, so
    // exec published it as that joint's computePointFrame; the final value is
    // the in-memory frame revision when the joint carries one, and otherwise
    // is the base (which is what the final-phase tap already resolves to).
    {
    RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "PublishJoints", "publish");
    for (size_t i = 0; i < _jointPaths.size(); ++i) {
        const RigExecPointFrame baseFrame =
            snapshot.Get<RigExecPointFrame>(_jointFrameTaps[i]);
        const auto revisedIt = _providerIndex.find(_jointPaths[i]);
        const bool haveRevised = revisedIt != _providerIndex.end() &&
                                 finalLive[revisedIt->second];
        const RigExecPointFrame finalFrame =
            haveRevised
                ? finalFrames[revisedIt->second]
                : snapshot.Get<RigExecPointFrame>(_jointFinalFrameTaps[i]);
        pose.jointFramesBase[_jointPaths[i]] = baseFrame;
        pose.jointFramesFinal[_jointPaths[i]] = finalFrame;
        // The point frame is the status bearer; the matrix result carries
        // no status and _ComputeJointMatrix returns identity for a
        // degenerate/invalid frame. Publishing that identity would let a
        // matrix-only consumer deform with a plausible-but-wrong transform.
        // Omit the matrix and diagnose so absence — not a
        // false identity — signals the failure; consumers already handle a
        // missing jointMatricesFinal entry. The degenerate frame is still
        // published so imaging can omit its guide.
        if (finalFrame.IsValid() && !finalFrame.IsDegenerate()) {
            auto revisedMatrix = finalMatrices.find(_jointPaths[i]);
            if (revisedMatrix == finalMatrices.end()) {
                // Untargeted descendants carried by a constrained ancestor do
                // not have a dedicated rest-frame tap. Compose their
                // base->revised delta onto the authoritative pre-constraint
                // rest->base matrix from the snapshot.
                GfMatrix4d delta(1.0);
                if (RigExecPointsToMatrix(
                        baseFrame.points, finalFrame.points, &delta)) {
                    finalMatrices[_jointPaths[i]] =
                        snapshot.Get<GfMatrix4d>(
                            _jointFinalMatrixTaps[i]) * delta;
                    revisedMatrix = finalMatrices.find(_jointPaths[i]);
                }
            }
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
    }
    // 3a. Control frames. Most are animator-authored inputs and therefore
    // publish their base tap directly; a control explicitly named as a
    // constraint write target publishes the revised frame, matching FBX's
    // ability to constrain any transform object. A degenerate/invalid frame
    // remains the status bearer and lets imaging omit the guide.
    {
    RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "PublishControls", "publish");
    for (size_t i = 0; i < _controlPaths.size(); ++i) {
        const auto revised = _providerIndex.find(_controlPaths[i]);
        const bool haveRevised = revised != _providerIndex.end() &&
                                 finalLive[revised->second];
        pose.controlFrames[_controlPaths[i]] =
            haveRevised
                ? finalFrames[revised->second]
                : snapshot.Get<RigExecPointFrame>(_controlFrameTaps[i]);
    }
    }

    // Observational solver guides never gate the rig snapshot: an
    // incomplete guide evaluation degrades to a diagnostic. A consumer that
    // never reads pose.solverFrames disables them outright (see
    // SetSolverGuidesEnabled) and skips the request entirely.
    if (_guideTaps && _solverGuidesEnabled) {
        std::vector<RigExecValueOverride> guideOverrides = baseOverrides;
        guideOverrides.insert(guideOverrides.end(), _falloffLutOverrides.begin(),
                              _falloffLutOverrides.end());
        for (const auto &[solver, aggregate] : solvedAggregates) {
            guideOverrides.push_back({solver, _computePointFrameArray, TfToken(),
                                      VtValue(aggregate)});
        }
        for (const auto &[provider, tap] : _firstFramePoseFrames) {
            guideOverrides.push_back({provider, _computePointFrame, TfToken(),
                                      VtValue(finalFrames[_providerIndex.at(provider)])});
        }
        _guideDirty = _guideTaps->ConsumeDirty() || _guideDirty;
        const bool sameGuideInputs =
            !_guideDirty && _guideTime == time &&
            _guideSnapshot.IsValid() && _guideSnapshot.IsComplete() &&
            guideOverrides.size() == _guideInputs.size() &&
            std::equal(guideOverrides.begin(), guideOverrides.end(),
                       _guideInputs.begin(),
                       [](const RigExecValueOverride &a,
                          const RigExecValueOverride &b) {
                           return a.prim == b.prim &&
                                  a.computation == b.computation &&
                                  a.attribute == b.attribute &&
                                  a.value == b.value;
                       });
        RigExecSnapshot guideSnapshot;
        if (sameGuideInputs) {
            guideSnapshot = _guideSnapshot;
        } else {
            guideSnapshot = [&]() {
                RIGEXEC_PROFILE_SCOPE_CAT(_profiler, "SolverGuides", "exec");
                return _guideTaps->Evaluate(time, guideOverrides);
            }();
            if (guideSnapshot.IsComplete()) {
                _guideSnapshot = guideSnapshot;
                _guideInputs = guideOverrides;
                _guideTime = time;
                _guideDirty = false;
                _guideTaps->ConsumeDirty();
            } else {
                _guideDirty = true;
            }
        }
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

    // Property-domain results are published directly into
    // pose.movedProperties by _EvaluatePropertyChains, so no separate copy
    // is needed.
    // They share the map with the point chains below; a consumer distinguishes
    // them by the type the VtValue holds, not by which mover domain produced
    // them.

    // 3c. THE POSE-INTERPOLATOR PHASE.
    // Here and nowhere else. It reads the FINAL pose -- so it runs after the
    // whole pose walk, every constraint included and the driver constraints
    // in particular -- and it writes floats that the geometry chains below
    // consume, so it runs before them. It is not a mover and cannot be one:
    // a mover's inputs are resolved by the property chains, which run before
    // exec does and therefore cannot see the pose at all.
    _EvaluatePoseInterpolators(time, restFrames, restLive, finalFrames, finalLive, &pose);

    // Geometry consumes the completed pose, including interpolator weights.
    pose.valid = _EvaluateGeometry(
        time, snapshot, finalMatrices, constraintDeltas, constraintXformCache, pose);
    return pose;
}

} // namespace rigExec
