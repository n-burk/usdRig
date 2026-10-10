// Constraint operators, pose interpolators, and rest-frame composition.

#include "rigEvaluatorInternal.h"
#include "rigExecScene/poseSceneLowering.h"
#include "sceneDispatch.h"
#include "rigEvaluatorDependencies.h"
#include "rigEvaluatorConstraints.h"
#include "movers/moverRegistry.h"
#include "frameExtraction.h"
#include "rigExecMath/rbf.h"
#include "solverKernels.h"
#include "weightPackets.h"
#include "bodyPurity.h"
#include "rigExecMath/singleChainIk.h"
#include "rigExecMath/solvers.h"

#include "pxr/base/tf/notice.h"
#include "pxr/usd/sdf/changeBlock.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/editContext.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usdGeom/xformCache.h"
#include "pxr/usd/usdGeom/xformable.h"

#include <algorithm>
#include <cmath>
#include <mutex>
#include <set>

namespace rigExec {

using namespace evaluatorDetail;

namespace {

// Bodies sit next to the dispatch they replaced.
RigExecPointFrame _SolvePositionConstraint(const _ConstraintSolveContext &);

RigExecPointFrame _SolveRotationConstraint(const _ConstraintSolveContext &);

RigExecPointFrame _SolveScaleConstraint(const _ConstraintSolveContext &);

RigExecPointFrame _SolveParentConstraint(const _ConstraintSolveContext &);

const std::vector<_ConstraintHandler> &
_ConstraintHandlers()
{
    static const std::vector<_ConstraintHandler> handlers = {
        {TfToken("RigExecAimConstraint"), true, true, true, true,
         _ChannelGroup::Rotation, _ChannelGroup::Rotation, nullptr},
        {TfToken("RigExecPositionConstraint"), true, true, false, false,
         _ChannelGroup::Translation, _ChannelGroup::Translation,
         _SolvePositionConstraint},
        {TfToken("RigExecRotationConstraint"), true, true, true, false,
         _ChannelGroup::Rotation, _ChannelGroup::Rotation,
         _SolveRotationConstraint},
        {TfToken("RigExecScaleConstraint"), true, true, false, false,
         _ChannelGroup::Scale, _ChannelGroup::Scale,
         _SolveScaleConstraint},
        {TfToken("RigExecParentConstraint"), true, true, true, false,
         _ChannelGroup::All, _ChannelGroup::None,
         _SolveParentConstraint},
        {TfToken("RigExecSingleChainIkConstraint"), false, true, false, true,
         _ChannelGroup::None, _ChannelGroup::None, nullptr},
    };
    return handlers;
}

// Attribute names read per constraint per frame, interned once. Each is
// spelled here rather than at its call site because the parent solve and the
// group-mask reader name the same nine.
const TfToken _kAffectTranslationX("inputs:affectTranslationX");
const TfToken _kAffectTranslationY("inputs:affectTranslationY");
const TfToken _kAffectTranslationZ("inputs:affectTranslationZ");
const TfToken _kAffectRotationX("inputs:affectRotationX");
const TfToken _kAffectRotationY("inputs:affectRotationY");
const TfToken _kAffectRotationZ("inputs:affectRotationZ");
const TfToken _kAffectScaleX("inputs:affectScaleX");
const TfToken _kAffectScaleY("inputs:affectScaleY");
const TfToken _kAffectScaleZ("inputs:affectScaleZ");
const TfToken _kTranslationOffset("inputs:translationOffset");
const TfToken _kRotationOffset("inputs:rotationOffset");
const TfToken _kScaleOffset("inputs:scaleOffset");
const TfToken _kInputsEnabled("inputs:enabled");

RigExecConstraintAxisMask
_ReadConstraintAxisMask(
    const RigExecResolvedInputs &resolved, const UsdPrim &prim,
    const TfToken &x, const TfToken &y, const TfToken &z, UsdTimeCode time,
    bool fallback = true)
{
    RigExecConstraintAxisMask mask;
    mask.x = _ResolvedRead(resolved, prim, x, fallback, time);
    mask.y = _ResolvedRead(resolved, prim, y, fallback, time);
    mask.z = _ResolvedRead(resolved, prim, z, fallback, time);
    return mask;
}

RigExecPointFrame
_SolvePositionConstraint(const _ConstraintSolveContext &c)
{
    RigExecPositionConstraintParams params;
    params.offset = _ResolvedRead(
        *c.resolved, c.prim, _kTranslationOffset, GfVec3d(0), c.time);
    params.affect = c.affect;
    params.weight = c.weight;
    return RigExecApplyPositionConstraint(c.inputFrame, *c.sources, params);
}

RigExecPointFrame
_SolveRotationConstraint(const _ConstraintSolveContext &c)
{
    RigExecRotationConstraintParams params;
    params.offsetDegrees = _ResolvedRead(
        *c.resolved, c.prim, _kRotationOffset, GfVec3d(0), c.time);
    params.affect = c.affect;
    params.rotationOrder = c.order;
    params.weight = c.weight;
    params.carry = c.carry;
    return RigExecApplyRotationConstraint(c.inputFrame, *c.sources, params);
}

RigExecPointFrame
_SolveScaleConstraint(const _ConstraintSolveContext &c)
{
    RigExecScaleConstraintParams params;
    params.offset = _ResolvedRead(
        *c.resolved, c.prim, _kScaleOffset, GfVec3d(0), c.time);
    params.affect = c.affect;
    params.weight = c.weight;
    params.blendShear = c.blendShear;
    return RigExecApplyScaleConstraint(c.inputFrame, *c.sources, params);
}

RigExecPointFrame
_SolveParentConstraint(const _ConstraintSolveContext &c)
{
    RigExecParentConstraintParams params;
    if (c.masksStatic) {
        params.translationAxes = c.precompTranslation;
        params.rotationAxes = c.precompRotation;
        params.scaleAxes = c.precompScale;
    } else {
        params.translationAxes = _ReadConstraintAxisMask(
            *c.resolved, c.prim, _kAffectTranslationX,
            _kAffectTranslationY, _kAffectTranslationZ, c.time);
        params.rotationAxes = _ReadConstraintAxisMask(
            *c.resolved, c.prim, _kAffectRotationX,
            _kAffectRotationY, _kAffectRotationZ, c.time);
        // FBX disables scale by default; the explicit false fallback is the
        // authored contract, not an oversight (schema.usda:769-771).
        params.scaleAxes = _ReadConstraintAxisMask(
            *c.resolved, c.prim, _kAffectScaleX,
            _kAffectScaleY, _kAffectScaleZ, c.time, false);
    }
    params.rotationOrder = c.order;
    params.weight = c.weight;
    params.carry = c.carry;
    params.blendShear = c.blendShear;
    return RigExecApplyParentConstraint(c.inputFrame, *c.sources, params);
}

} // namespace

namespace evaluatorDetail {

const _ConstraintHandler *
_FindConstraintHandler(const TfToken &typeName)
{
    for (const _ConstraintHandler &handler : _ConstraintHandlers()) {
        if (typeName == handler.schemaType) {
            return &handler;
        }
    }
    return nullptr;
}

/// Source-blending constraints that revise one transform provider.
bool
_IsSourceFrameConstraintType(const TfToken &typeName)
{
    const _ConstraintHandler *handler = _FindConstraintHandler(typeName);
    return handler && handler->sourceFrame;
}

/// Every built-in constraint with fixed evaluator semantics.
bool
_IsFrameConstraintType(const TfToken &typeName)
{
    const _ConstraintHandler *handler = _FindConstraintHandler(typeName);
    return handler && handler->frameConstraint;
}

// Interned once at load: the parse runs per constraint per frame, and token
// comparison is a pointer comparison while literal comparison is not.
const TfToken _kOrderXzy("XZY");
const TfToken _kOrderYxz("YXZ");
const TfToken _kOrderYzx("YZX");
const TfToken _kOrderZxy("ZXY");
const TfToken _kOrderZyx("ZYX");

RigExecEulerOrder
_ParseConstraintEulerOrder(const TfToken &token)
{
    if (token == _kOrderXzy) return RigExecEulerOrder::XZY;
    if (token == _kOrderYxz) return RigExecEulerOrder::YXZ;
    if (token == _kOrderYzx) return RigExecEulerOrder::YZX;
    if (token == _kOrderZxy) return RigExecEulerOrder::ZXY;
    if (token == _kOrderZyx) return RigExecEulerOrder::ZYX;
    return RigExecEulerOrder::XYZ;
}

bool
_TokenIsOneOf(const TfToken &value,
              std::initializer_list<const char *> allowed)
{
    return std::any_of(
        allowed.begin(), allowed.end(),
        [&value](const char *candidate) { return value == candidate; });
}

/// Reads the per-axis mask for an operator's own channel group. The masks
/// are spelled by (group, axis) on the base class, so which triple to read is
/// a property of the operator, not of the call site.
RigExecConstraintAxisMask
_ReadGroupMask(const RigExecResolvedInputs &resolved, const UsdPrim &prim,
               _ChannelGroup group, UsdTimeCode time)
{
    switch (group) {
    case _ChannelGroup::Translation:
        return _ReadConstraintAxisMask(
            resolved, prim, _kAffectTranslationX,
            _kAffectTranslationY, _kAffectTranslationZ, time);
    case _ChannelGroup::Rotation:
        return _ReadConstraintAxisMask(
            resolved, prim, _kAffectRotationX, _kAffectRotationY,
            _kAffectRotationZ, time);
    case _ChannelGroup::Scale:
        return _ReadConstraintAxisMask(
            resolved, prim, _kAffectScaleX, _kAffectScaleY,
            _kAffectScaleZ, time);
    case _ChannelGroup::All:
    case _ChannelGroup::None:
        break;
    }
    // Parent reads all three groups itself; the operators that honor no mask
    // are unmasked. Both want the all-true identity.
    return RigExecConstraintAxisMask();
}

} // namespace evaluatorDetail

// Pose interpolators

bool
RigExecRigEvaluator::_CompilePoseInterpolators(
    const std::vector<SdfPath> &joints,
    const std::vector<SdfPath> &controls,
    std::vector<_PoseInterpolator> *out,
    std::vector<std::string> *notes,
    std::string *error, SdfPath *operation,
    RigExecSceneDescriptors *capturedScene,RigExecProfiler *profile) const
{
    RigExecProfilePhases phases(profile,"compile");
    phases.Next("Compile.PoseInterpolators.Discovery");
    out->clear();
    auto paths=_DiscoverPoseInterpolators(_stage,_rigPath);
    paths.erase(std::remove_if(paths.begin(),paths.end(),[this](const SdfPath &path) {
        return _IsSkippedOperation(path);
    }),paths.end());
    if(paths.empty())return true;
    phases.Next("Compile.PoseInterpolators.SceneCapture");
    RigExecSceneDescriptors localScene;
    RigExecSceneDescriptors &scene = capturedScene ? *capturedScene : localScene;
    if(!RigExecDispatchCaptureSceneDescriptors(_stage,_rigPath,{UsdTimeCode::Default()},&scene,error))return false;
    phases.Next("Compile.PoseInterpolators.Lowering");
    std::set<SdfPath> providers(joints.begin(),joints.end());
    providers.insert(controls.begin(),controls.end());
    for(const auto &path:paths) {
        *operation=path;
        auto descriptor=std::make_shared<RigExecScenePoseInterpolatorDescriptor>();
        if(!RigExecDispatchLowerScenePoseInterpolator(scene,path,descriptor.get(),error))return false;
        if(!descriptor->driver.IsEmpty() && !providers.count(descriptor->driver)) {
            *error="pose interpolator "+path.GetString()+" driver is not a provider of this rig";
            return false;
        }
        if(!descriptor->parent.IsEmpty() && !providers.count(descriptor->parent)) {
            *error="pose interpolator "+path.GetString()+" driver parent is not a provider of this rig";
            return false;
        }
        _PoseInterpolator record;record.prim=path;record.descriptor=descriptor;
        record.driver=descriptor->driver;record.driverParent=descriptor->parent;
        record.driverAttributes=descriptor->driverAttributes;
        record.poseWeights=descriptor->poseWeights;record.disabledPoseWeights=descriptor->disabledPoseWeights;
        if(notes)notes->insert(notes->end(),descriptor->notes.begin(),descriptor->notes.end());
        out->push_back(std::move(record));
    }
    return true;
}

std::vector<std::string>
RigExecRigEvaluator::_ApplyDerivedStartFrames()
{
    // Process-wide: two evaluators compiling concurrently (two characters
    // sharing one stage) must not interleave session-layer writes, and USD
    // layers are not thread-safe. Microseconds per compile.
    static std::mutex derivedOpinionsMutex;
    std::lock_guard<std::mutex> lock(derivedOpinionsMutex);

    static const TfToken startFrameRel("rigExec:startFrame");
    static const TfToken startFramePolicy("rigExec:startFramePolicy");
    static const TfToken jointsRel("rigExec:joints");
    static const TfToken jointType("RigExecJoint");
    static const TfToken controlType("RigExecControl");
    static const TfToken policyNone("none");
    static const TfToken policyParent("parent");

    SdfLayerHandle session = _stage->GetSessionLayer();
    // Collected, not emitted: nothing that can Send -- TF_WARN included --
    // may run inside the notice block below, so the caller emits these
    // after this returns (both blocks then closed).
    std::vector<std::string> warnings;
    auto warn = [&warnings](const std::string &message) {
        warnings.push_back(message);
    };

    // NO NOTICE may leave this function. It runs inside Compile, and the
    // imaging registry registers its ObjectsChanged listener BEFORE
    // Compile and holds a non-recursive mutex across the whole call, so a
    // notice fired here re-enters the registry on its own held mutex
    // (measured: an access violation in _OnObjectsChanged on the first
    // usdview activation of a policy-carrying rig). TfNotice::Block
    // swallows the send while the ChangeBlock still batches the Sdf-side
    // work. Sound because the in-flight compile is the only reader of
    // these opinions: the structure digest runs after this returns, and
    // every Compile rebuilds the epoch (fresh taps) rather than
    // invalidating the old one, so no cached exec value can strand on the
    // swallowed send. Block is thread-scoped, and this runs
    // single-threaded, before the digest dispatch.
    TfNotice::Block noticeBlock;
    SdfChangeBlock block;
    // Retract this evaluator's previous opinions first, surgically: only
    // OUR tracked provider leaves each session list, so a hand-authored
    // session opinion on the same relationship survives. Anything still
    // composed afterwards is not ours, which is exactly the "authored
    // wins" test the derivation below applies.
    for (const auto &[solverPath, ourProvider] : _derivedStartFrames) {
        SdfRelationshipSpecHandle spec = session->GetRelationshipAtPath(
            solverPath.AppendProperty(startFrameRel));
        if (!spec) {
            continue;
        }
        bool present = false;
        // Explicit items only: our writes are SetTargets (explicit), and
        // a user's prepended/appended opinions are not ours to inspect.
        const auto explicitItems =
            spec->GetTargetPathList().GetExplicitItems();
        for (size_t i = 0, n = explicitItems.size(); i < n; ++i) {
            if (explicitItems[i] == ourProvider) {
                present = true;
                break;
            }
        }
        if (!present) {
            continue;
        }
        spec->RemoveTargetPath(ourProvider);
        if (!spec->HasTargetPathList()) {
            if (SdfPrimSpecHandle primSpec =
                    session->GetPrimAtPath(solverPath)) {
                primSpec->RemoveProperty(spec);
            }
        }
    }
    _derivedStartFrames.clear();

    UsdPrim rig = _stage->GetPrimAtPath(_rigPath);
    if (rig) {
        UsdEditContext sessionCtx(_stage, session);
        for (const UsdPrim &prim : UsdPrimRange(rig)) {
            if (prim.GetTypeName() != "RigExecFkChain" ||
                _IsSkippedOperation(prim.GetPath())) {
                continue;
            }
            TfToken policy;
            prim.GetAttribute(startFramePolicy).Get(&policy);
            if (policy.IsEmpty()) {
                policy = policyNone;
            }
            if (policy == policyNone) {
                continue;
            }
            const SdfPath solverPath = prim.GetPath();
            if (policy != policyParent) {
                warn(solverPath.GetString() +
                     " has rigExec:startFramePolicy '" +
                     policy.GetString() +
                     "', expected 'none' or 'parent'; solving absolute.");
                continue;
            }
            SdfPathVector composed;
            prim.GetRelationship(startFrameRel).GetTargets(&composed);
            if (!composed.empty()) {
                continue;  // Authored (asset or session) always wins.
            }
            SdfPathVector joints;
            prim.GetRelationship(jointsRel).GetTargets(&joints);
            if (joints.empty()) {
                continue;  // A jointless chain hangs from nothing.
            }
            // The inference: nearest namespace ancestor of the chain's
            // joints that is a joint or control. Structural ancestry
            // only -- GetParentPath, never a name -- so a reparented
            // chain follows its new parent with no authoring change.
            UsdPrim first =
                _stage->GetPrimAtPath(joints[0].GetPrimPath());
            SdfPath provider;
            for (SdfPath a = first ? first.GetPath().GetParentPath()
                                   : SdfPath::EmptyPath();
                 !a.IsEmpty() && a != SdfPath::AbsoluteRootPath();
                 a = a.GetParentPath()) {
                if (a == _rigPath) {
                    break;
                }
                const UsdPrim ancestor = _stage->GetPrimAtPath(a);
                if (!ancestor) {
                    continue;
                }
                const TfToken type = ancestor.GetTypeName();
                if (type == jointType || type == controlType) {
                    provider = a;
                    break;
                }
            }
            if (provider.IsEmpty()) {
                warn(solverPath.GetString() +
                     " has rigExec:startFramePolicy 'parent' but no "
                     "RigExecJoint/RigExecControl ancestor; solving "
                     "absolute.");
                continue;
            }
            bool shared = true;
            for (const SdfPath &j : joints) {
                const SdfPath jp = j.GetPrimPath();
                if (jp == provider || !jp.HasPrefix(provider)) {
                    shared = false;
                    break;
                }
            }
            if (!shared) {
                warn(solverPath.GetString() +
                     " has rigExec:startFramePolicy 'parent' but its "
                     "joints span providers; solving absolute.");
                continue;
            }
            prim.GetRelationship(startFrameRel)
                .SetTargets(SdfPathVector{provider});
            _derivedStartFrames[solverPath] = provider;
        }
    }
    return warnings;
}

// Whether any of \p providers has a rest channel that can no longer be held
// as an epoch constant.
// Asked only for the providers the rest gate saw a notice reach, never per
// frame: the answer is a function of the provider's own seven rest
// attributes and of the epoch's property chains, the chains change only with
// a recompile, and the attributes change only through a notice on one of the
// paths the gate watches.
void
RigExecRigEvaluator::_NoteRestEdits(const UsdNotice::ObjectsChanged &notice)
{
    if (!_epochRestsConstant || _restEpochProviderPaths.empty()) {
        return;
    }
    bool reached = false;
    const auto noteProperty = [&](const SdfPath &path) {
        if (!path.IsPrimPropertyPath() ||
            !_IsRestInputName(_restInputNames, path.GetNameToken())) {
            return;
        }
        const SdfPath prim = path.GetPrimPath();
        // A subtree is one contiguous run of the ordered map, starting at
        // the prim itself when the prim is a key.
        const auto first = _restEpochProviderPaths.lower_bound(prim);
        if (first == _restEpochProviderPaths.end() || !first->HasPrefix(prim)) {
            return;
        }
        if (*first == prim) {
            _restEditedProviders.insert(prim);
        }
        reached = true;
    };
    const auto noteResync = [&](const SdfPath &path) {
        if (!path.IsAbsoluteRootOrPrimPath()) {
            noteProperty(path);
            return;
        }
        // Every provider at or under the resynced prim. A subtree is one
        // contiguous run of the ordered map, starting at the prim itself.
        for (auto provider = _restEpochProviderPaths.lower_bound(path);
             provider != _restEpochProviderPaths.end() &&
             provider->HasPrefix(path); ++provider) {
            _restEditedProviders.insert(*provider);
            reached = true;
        }
    };
    for (const SdfPath &path : notice.GetResyncedPaths()) {
        noteResync(path);
    }
    for (const SdfPath &path : notice.GetResolvedAssetPathsResyncedPaths()) {
        noteResync(path);
    }
    for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
        noteProperty(path);
    }
    if (reached) {
        _epochRestFramesStale = true;
    }
}

// Source-boundary sampling of ordinary Xformable inputs. Pure operation
// bodies consume the captured frames through typed graph values.

bool
RigExecRigEvaluator::_FrameFromXformRelativeToAsset(
    const UsdPrim &assetRoot,
    UsdGeomXformCache *xformCache,
    const SdfPath &path,
    RigExecPointFrame *outFrame,
    GfMatrix4d *outMatrix) const
{
    RIGEXEC_PURITY_CHECK();
    const UsdPrim prim = _stage->GetPrimAtPath(path);
    if (!prim || !assetRoot || !UsdGeomXformable(prim)) {
        return false;
    }
    bool resetsBelowAsset = false;
    const GfMatrix4d relative =
        xformCache->ComputeRelativeTransform(prim, assetRoot,
                                             &resetsBelowAsset);
    if (outFrame) {
        *outFrame = RigExecFrameFromMatrix(relative);
    }
    if (outMatrix) {
        *outMatrix = relative;
    }
    return true;
}

bool
RigExecApplyRevisedAncestorDelta(
    const SdfPath &xformPath,
    const RigExecPoseFrameEnumerator &providers,
    RigExecPointFrame *frame)
{
    // A native source that is not itself a written provider may still
    // sit beneath a constrained transform provider. The closest
    // revised ancestor contains all higher ancestor deltas, so apply
    // it once to the stage-derived source frame.
    // The comparison is over POINTS and not whole frames: a provider whose
    // flags differ from its base while its points do not has not moved, and
    // comparing the frames would make it the closest revised ancestor and
    // ride the source on an identity that is not one.
    SdfPath closest;
    RigExecPointFrame closestBase, closestCurrent;
    auto select = [&](const SdfPath &provider,
                      const RigExecPointFrame &base,
                      const RigExecPointFrame &current) {
        if (provider == xformPath || !xformPath.HasPrefix(provider) ||
            current.points == base.points) {
            return;
        }
        if (closest.IsEmpty() ||
            provider.GetPathElementCount() >
                closest.GetPathElementCount()) {
            closest = provider;
            closestBase = base;
            closestCurrent = current;
        }
    };
    providers(select);
    if (!closest.IsEmpty()) {
        GfMatrix4d delta(1.0);
        if (!RigExecPointsToMatrix(
                closestBase.points, closestCurrent.points, &delta)) {
            return false;
        }
        *frame = RigExecMatrixToPoints(frame->points, delta);
    }
    return frame->IsValid();
}

bool
RigExecRigEvaluator::_IkUsesAnimatedTs(const std::vector<SdfPath> &chain) const
{
    for (const SdfPath &path : chain) {
        if (_jointSolverBinding.count(path)) {
            return true;
        }
        const UsdPrim joint = _stage->GetPrimAtPath(path);
        for (const char *name : {
                 "posed:space", "avars:tx", "avars:ty", "avars:tz",
                 "avars:sx", "avars:sy", "avars:sz"}) {
            const UsdAttribute attr = joint.GetAttribute(TfToken(name));
            SdfPathVector connections;
            // HasAuthoredConnections first: see _AuthoredConnections.
            if (attr &&
                (attr.GetNumTimeSamples() > 0 ||
                 (attr.HasAuthoredConnections() &&
                  attr.GetConnections(&connections) &&
                  !connections.empty()))) {
                return true;
            }
        }
    }
    return false;
}

const std::vector<TfToken> &
RigExecRigEvaluator::GetConstraintOperatorTypeNames()
{
    // Built from the table itself rather than written out again, so an
    // operator added there is named here without anyone remembering to.
    static const std::vector<TfToken> names = [] {
        std::vector<TfToken> types;
        types.reserve(_ConstraintHandlers().size());
        for (const _ConstraintHandler &handler : _ConstraintHandlers()) {
            types.push_back(handler.schemaType);
        }
        return types;
    }();
    return names;
}

bool
RigExecRigEvaluator::IsConstraintOperatorType(const TfToken &schemaType)
{
    return _FindConstraintHandler(schemaType) != nullptr;
}

size_t
RigExecConstraintHandlerCount(const TfToken &schemaType)
{
    return _FindConstraintHandler(schemaType) ? 1 : 0;
}

size_t
RigExecConstraintHandlerTotal()
{
    return _ConstraintHandlers().size();
}

bool
RigExecConstraintUsesRotationOrder(const TfToken &schemaType)
{
    const _ConstraintHandler *handler = _FindConstraintHandler(schemaType);
    return handler && handler->usesRotationOrder;
}

} // namespace rigExec
