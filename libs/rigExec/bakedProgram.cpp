// The baked program: build and run. See bakedProgram.h for what it is and
// why it is a request rather than a promise.
// Everything here is a second expression of semantics that live elsewhere --
// the provider compose and the default-space ladder in computations.cpp, the
// pose walk in rigEvaluator.cpp, the geometry revision in moverGraph.cpp --
// so every piece is written against the one it mirrors and nothing is
// re-derived from first principles. Where a kernel can simply be CALLED
// instead of mirrored (the constraint operators, the skin kernel, the
// extent/normal kernels, the packet assembler, the property chains) it is,
// because a shared call cannot drift and a copy can.
// Phase 2 split this file by domain: the pose walk lives in bakedPose.cpp and
// the geometry chains in bakedGeometry.cpp, both over the state declared in
// bakedProgramImpl.h. What stays here is the public surface, the bakeability
// judgement, the Build skeleton that hands each domain its share, and the
// parity comparator -- plus the one thing neither half may do, which is read
// the evaluator's private state. This is the only translation unit the
// evaluator declares a friend, so Build captures what a frame needs of it
// once and the halves read the program instead.
#include "bakedProgram.h"

#include "bakedProgramImpl.h"
#include "bakedSchedule.h"
#include "frameExtraction.h"
#include "frozenContextInternal.h"
#include "moverGraph.h"
#include "movers/moverRegistry.h"
#include "parallel.h"
#include "rigEvaluator.h"
#include "rigEvaluatorInternal.h"
#include "types.h"

#include "rigExecMath/pointFrame.h"

#include "pxr/base/gf/math.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/base/work/dispatcher.h"
#include "pxr/base/work/loops.h"
#include "pxr/base/gf/rotation.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/attributeQuery.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/metrics.h"
#include "pxr/usd/usdGeom/xformCache.h"
#include "pxr/usd/usdGeom/xformable.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace rigExec {

namespace {

// A plain attribute read with no time and no resolution walk, for the
// uniform tokens the dynamic path reads exactly this way
// (rigExec:rotationOrder, rigExec:worldUpType, rigExec:aimAxis, ...).
TfToken
_ReadToken(const UsdPrim &prim, const char *name, const char *fallback)
{
    TfToken value(fallback);
    if (prim) {
        if (const UsdAttribute a = prim.GetAttribute(TfToken(name))) {
            TfToken read;
            if (a.Get(&read) && !read.IsEmpty()) {
                value = read;
            }
        }
    }
    return value;
}

SdfPathVector
_Targets(const UsdPrim &prim, const char *name)
{
    SdfPathVector targets;
    if (prim) {
        if (const UsdRelationship rel = prim.GetRelationship(TfToken(name))) {
            rel.GetTargets(&targets);
        }
    }
    return targets;
}

// The constraint operators the program expresses. A type outside this set
// makes the epoch unbakeable rather than silently passing through.
bool
_IsBakedConstraintType(const TfToken &type)
{
    const bool baked = type == "RigExecPositionConstraint" ||
           type == "RigExecRotationConstraint" ||
           type == "RigExecScaleConstraint" ||
           type == "RigExecParentConstraint" ||
           type == "RigExecAimConstraint" ||
           type == "RigExecSingleChainIkConstraint";
    // RigExecRigEvaluator::GetConstraintOperatorTypeNames() is the authority
    // on which operators exist; this is the subset the program can express,
    // and it must stay a subset. A name here the evaluator does not register
    // is a typo that refuses nothing and bakes nothing, and it would read as
    // an operator that simply never takes the baked path.
    TF_VERIFY(!baked || RigExecRigEvaluator::IsConstraintOperatorType(type),
              "rigExec: %s is baked but is not a registered constraint "
              "operator", type.GetText());
    return baked;
}

bool
_IsBakedSolverType(const TfToken &type)
{
    return type == "RigExecFkChain" || type == "RigExecTwoBoneIk" ||
           type == "RigExecBlendPointFrames" || type == "RigExecSplineIk" ||
           type == "RigExecTwistDistribution" || type == "RigExecRibbon";
}

// The three placeable weight shapes, which the evaluator spells the same way
// in its own file-static _IsVolumeWeightType. Three tokens, and duplicating
// them here is cheaper than widening a header for a predicate neither side
// would ever change without the other.
bool
_IsVolumeWeightTypeName(const TfToken &type)
{
    return type == "RigExecSphereWeight" || type == "RigExecPlaneWeight" ||
           type == "RigExecCurveWeight";
}

// The volumes the walk places (_volumeWeightMatrixTaps) that have no
// volume-typed provider slot, so the program could not place or publish
// them. Expected empty: the compile seeds every tapped volume as a provider
// (seedProvider over the taps, rigEvaluatorCompile.cpp). The converse is
// not checked because it does not hold: a constraint can name a volume
// outside the rig as a source, which makes it a provider without a tap,
// and the program then holds a slot the walk never places (placedVolumes).
std::vector<SdfPath>
_WalkVolumesWithoutASlot(const std::set<SdfPath> &programVolumes,
                         const std::map<SdfPath, RigExecTapId> &walkVolumes)
{
    std::vector<SdfPath> missing;
    for (const auto &[path, tap] : walkVolumes) {
        if (!programVolumes.count(path)) {
            missing.push_back(path);
        }
    }
    return missing;
}

// The solvers the walk runs in a batch. A template, because the batch type
// is private to the evaluator and only this file's friend code can hand it in.
template <class Batches>
std::set<SdfPath>
_BatchedSolvers(const Batches &batches)
{
    std::set<SdfPath> batched;
    for (const auto &batch : batches) {
        for (const auto &[solverPath, tap] : batch.solvers) {
            batched.insert(solverPath);
        }
    }
    return batched;
}

// The read-phase solver checkpoints no batched solver commits: a pair
// (joint, solver) the compile named whose solver is not batched or does not
// bind the joint. "Is an aggregate solver" is _solverDependencies membership:
// every discovered solver is a key there.
std::vector<SdfPath>
_SolverCheckpointsWithoutAnOutput(
    const std::map<SdfPath, std::set<SdfPath>> &snapshots,
    const std::map<SdfPath, std::set<SdfPath>> &solverDependencies,
    const std::map<SdfPath, std::vector<std::pair<SdfPath, int>>>
        &solverJoints,
    const std::set<SdfPath> &batched)
{
    std::vector<SdfPath> missing;
    for (const auto &[joint, movers] : snapshots) {
        for (const SdfPath &mover : movers) {
            if (!solverDependencies.count(mover)) {
                continue;
            }
            bool bound = false;
            const auto joints = solverJoints.find(mover);
            if (batched.count(mover) && joints != solverJoints.end()) {
                for (const auto &[path, element] : joints->second) {
                    bound = bound || path == joint;
                }
            }
            if (!bound) {
                missing.push_back(mover);
            }
        }
    }
    return missing;
}

// The weight-object schemas the program can build a packet for.
// It is about the BUILDER and nothing else. The volumetric three are here
// because RigExecBuildVolumeWeightPacket is one of the builders; volumes
// bake, and the program places them itself (VolumePlacements). The only
// tap-related check is the one-way guard above (_WalkVolumesWithoutASlot):
// every volume the walk places needs a provider slot.
bool
_IsBakedWeightType(const TfToken &type)
{
    return type == "RigExecStaticWeight" || type == "RigExecDynamicWeight" ||
           type == "RigExecCombineWeight" || _IsVolumeWeightTypeName(type);
}

// A numeric probe time. Selection along a connection chain must not depend on
// which time code the caller asks for, and Default cannot read an attribute
// that carries only time samples, so stability is checked against both.
UsdTimeCode
_ProbeTime(const UsdStageRefPtr &stage)
{
    return stage && stage->HasAuthoredTimeCodeRange()
               ? UsdTimeCode(stage->GetStartTimeCode())
               : UsdTimeCode(0.0);
}

bool
_AttributeIsIdentity(const UsdPrim &prim, const char *name, bool *connected)
{
    const UsdAttribute a = prim.GetAttribute(TfToken(name));
    if (!a) {
        return true;
    }
    SdfPathVector connections;
    if (a.HasAuthoredConnections() && a.GetConnections(&connections) &&
        !connections.empty()) {
        *connected = true;
        return false;
    }
    GfMatrix4d value(1.0);
    return !a.Get(&value) || value == GfMatrix4d(1.0);
}

// The provider space attributes exec does NOT read off the stage. Five of
// them carry a registered AttributeExpression (computations.cpp's
// RIGEXEC_SPACE_EXPRESSION), and posed:space is fed to computePointFrame
// through an explicit Connections input; on any of the six, the builtin
// computeValue a connection resolves through is a COMPUTATION -- the space
// expression that follows the namespace parent, or a posed frame -- not the
// attribute's authored value. A connection that ENDS at one of them is
// therefore outside the single-connection walk the binding table performs.
bool
_ConnectionReachesComputedSpace(const UsdAttribute &attribute)
{
    static const char *const kComputedSpaces[] = {
        "default:space", "avars:defaultSpace", "posed:defaultSpace",
        "parent:space", "parent:defaultSpace", "posed:space"};
    // Mirrors RigExecBakedClassifyInput's loop, and stops on the same two
    // conditions it does: a cycle, or a fan-out the walk cannot follow.
    std::set<SdfPath> visiting;
    UsdAttribute a = attribute;
    bool first = true;
    while (a && visiting.insert(a.GetPath()).second) {
        if (!first) {
            const std::string name = a.GetName().GetString();
            for (const char *computed : kComputedSpaces) {
                if (name == computed) {
                    return true;
                }
            }
        }
        first = false;
        SdfPathVector connections;
        a.GetConnections(&connections);
        if (connections.size() != 1) {
            break;
        }
        a = a.GetPrim().GetStage()->GetAttributeAtPath(connections[0]);
    }
    return false;
}

}  // namespace

// Bakeability.

bool
RigExecBakedProgram::IsBakeable(const RigExecRigEvaluator &evaluator,
                               std::vector<std::string> *reasons)
{
    // The refusal list IS the answer here -- the walk below is the same
    // whether or not anybody wants to read the sentences -- so a caller
    // that passes nothing is given a scratch vector and its refusals are
    // dropped on return; see _WantsBakeRefusalReasons for who keeps them.
    std::vector<std::string> local;
    std::vector<std::string> &out = reasons ? *reasons : local;
    const size_t before = out.size();
    const RigExecRigEvaluator &E = evaluator;
    auto say = [&out](const std::string &what, const SdfPath &where) {
        out.push_back(what + ": " + where.GetString());
    };

    if (!E._compiled) {
        out.push_back("the rig is not compiled");
        return false;
    }

    for (const auto &[path, taps] : E._connectedPoseTaps) {
        say("connected-space provider", path);
    }
    // _interveningXformProviders is a CANDIDATE list: it holds every provider
    // whose parent is not its anchor, which on an ordinary rig means a
    // grouping Scope with no transform at all. The dynamic path composes
    // nothing unless one of them resolves to a non-identity X(P), so that --
    // not the candidacy -- is what the program cannot express.
    if (!E._interveningXformProviders.empty()) {
        const UsdPrim assetRoot =
            E._stage->GetPrimAtPath(E._rigPath.GetParentPath());
        UsdGeomXformCache cache(UsdTimeCode::Default());
        for (const SdfPath &path : E._interveningXformProviders) {
            const auto anchorIt = E._poseProviderAnchors.find(path);
            const SdfPath anchorPath = anchorIt == E._poseProviderAnchors.end()
                                           ? SdfPath()
                                           : anchorIt->second;
            const UsdPrim anchor = anchorPath.IsEmpty()
                                       ? assetRoot
                                       : E._stage->GetPrimAtPath(anchorPath);
            const UsdPrim parent =
                E._stage->GetPrimAtPath(path.GetParentPath());
            if (!parent || !anchor || parent == anchor) {
                continue;
            }
            bool resets = false;
            const GfMatrix4d x =
                cache.ComputeRelativeTransform(parent, anchor, &resets);
            if (resets || x != GfMatrix4d(1.0)) {
                say("intervening Xform above provider", path);
                continue;
            }
            // Identity today is not identity for the epoch if it animates.
            for (UsdPrim walk = parent; walk && walk != anchor;
                 walk = walk.GetParent()) {
                const UsdGeomXformable xformable(walk);
                if (xformable && xformable.TransformMightBeTimeVarying()) {
                    say("animated Xform above provider", path);
                    break;
                }
            }
        }
    }

    // The step reads the driver's frame, and its parent's, out of the slots,
    // so both have to BE slots. Compile already refused a driver that is not a
    // joint or control of the rig; this is the same fact stated against the
    // program's own table.
    for (const RigExecRigEvaluator::_PoseInterpolator &record :
             E._poseInterpolators) {
        // A NUMERIC driver reads dials rather than a frame: it names no
        // driver prim, so there is no slot for one to be.
        if (!record.driverAttributes.empty()) {
            continue;
        }
        if (!E._firstFramePoseFrames.count(record.driver)) {
            say("pose interpolator driver is not a pose provider",
                record.driver);
        }
        if (!record.driverParent.IsEmpty() &&
            !E._firstFramePoseFrames.count(record.driverParent)) {
            say("pose interpolator driver parent is not a pose provider",
                record.driverParent);
        }
    }

    const UsdTimeCode probe = _ProbeTime(E._stage);
    // A property chain RECOMPUTES its target every generation, so a value
    // read once at bake time is not that target's value -- it is whatever
    // the previous generation happened to leave behind. Inputs the program
    // re-reads per frame resolve through the chain and are fine; the ones
    // resolved once into the rest chain and the default-space ladder are
    // not, so a chain aimed at one of those refuses the bake.
    std::set<SdfPath> chainTargets;
    for (const auto &[target, revisions] : E._propertyChains) {
        chainTargets.insert(target);
    }
    // A space switch replaces a provider's parent with one the ladder does
    // not know about. The compose reads every frame it needs at a version
    // bound at Build (RigExecBakedProgramImpl::SpaceSwitch::FrameVersion)
    // and the compose groups are emitted in dependency order, so all it
    // needs here is a slot for the target and for each source.
    for (const RigExecRigEvaluator::_SpaceSwitch &sw : E._spaceSwitches) {
        if (!E._firstFramePoseFrames.count(sw.target)) {
            say("space switch on a provider with no slot", sw.target);
            continue;
        }
        for (const auto &source : sw.sources) {
            if (source.path.IsEmpty()) {
                continue;   // world: identity, no slot needed
            }
            if (!E._firstFramePoseFrames.count(source.path)) {
                say("space switch source has no slot", source.path);
            }
        }
    }
    // An auto clavicle reads its frames out of slots, and composes its first
    // FK control inline from the target's frame, so that control must hang
    // directly from the target with no space of its own.
    std::set<SdfPath> switchedTargets;
    for (const RigExecRigEvaluator::_SpaceSwitch &sw : E._spaceSwitches) {
        switchedTargets.insert(sw.target);
    }
    for (const RigExecRigEvaluator::_AutoClavicle &ac : E._autoClavicles) {
        for (const SdfPath &path :
             {ac.target, ac.pivot, ac.anchor, ac.fk[0], ac.fk[1], ac.fk[2],
              ac.ikTarget, ac.pole}) {
            if (!path.IsEmpty() && !E._firstFramePoseFrames.count(path)) {
                say("auto clavicle frame has no slot", path);
            }
        }
        if (ac.fk[0].GetParentPath() != ac.target) {
            say("auto clavicle's first FK control is not a child of its "
                "target", ac.fk[0]);
        }
        if (switchedTargets.count(ac.fk[0])) {
            say("auto clavicle's first FK control has a space switch",
                ac.fk[0]);
        }
    }
    const TfToken rotationSignName("avars:rotationSign");
    for (const auto &[path, tap] : E._firstFramePoseFrames) {
        const UsdPrim prim = E._stage->GetPrimAtPath(path);
        if (!prim) {
            say("pose provider has no prim", path);
            continue;
        }
        const TfToken type = prim.GetTypeName();
        // The volumetric three are RigExecXformables, so the pose walk
        // composes them into a slot exactly as it composes a joint -- with
        // one difference the compose has to know about, which is that their
        // scale avars are read and discarded (RigExecBakedProgramImpl::
        // noScaleAvars).
        if (type != "RigExecJoint" && type != "RigExecControl" &&
            !_IsVolumeWeightTypeName(type)) {
            say("provider type not baked (" + type.GetString() + ")", path);
        }
        // A non-identity AUTHORED posed:space is not a ladder at all: exec
        // uses it directly and reads neither the avars nor the parent
        // (computations.cpp's _ComputeXformablePointFrame, step 2). The
        // compose expresses that branch and re-takes the "is it identity"
        // test on every frame the ladder is recomposed on, so an animated
        // or chain-written one bakes. Only the CONNECTED case -- an
        // arbitrary exec computation, which is what step 1 of the same
        // function reads -- still refuses.
        {
            const UsdAttribute posed =
                prim.GetAttribute(TfToken("posed:space"));
            SdfPathVector connections;
            if (posed && posed.HasAuthoredConnections() &&
                posed.GetConnections(&connections) && !connections.empty()) {
                say("connected posed:space on provider", path);
            }
        }
        // The other four ARE the ladder: the program builds the
        // default-space chain from rest + default avars and follows the
        // namespace parent, which is what exec does only while these stay
        // unauthored and unconnected.
        for (const char *name : {"parent:space",
                                 "parent:defaultSpace", "avars:defaultSpace",
                                 "posed:defaultSpace"}) {
            bool connected = false;
            if (!_AttributeIsIdentity(prim, name, &connected)) {
                say(std::string(connected ? "connected " : "authored ") +
                        name + " on provider",
                    path);
            }
            if (chainTargets.count(path.AppendProperty(TfToken(name)))) {
                say(std::string("property chain writes ") + name +
                        " on provider",
                    path);
            }
        }
        // The ladder is a per-frame input now, so an animated, connected or
        // chain-written channel bakes wherever exec reads the channel the
        // way the binding table reads it: a plain AttributeValue, which
        // resolves a connection by the same single-connection walk
        // RigExecBakedClassifyInput performs. That is every rest and
        // default AVAR, avars:rotationOrder -- and rest:space, which is the
        // one MATRIX channel of the ladder with no registered
        // AttributeExpression behind it (computations.cpp declares
        // AttributeValue<GfMatrix4d>(restSpace) beside
        // AttributeValue<double>(restTx), in the same computeRestFrame, and
        // registers its five space expressions elsewhere). Measured
        // against a rig whose rest:space MOVES the joint --
        // testRigExecEpochRests' TestAConnectedRestSpaceIsPulledPerFrame,
        // which compares the connection against the same matrix authored
        // plainly and against the unedited rig, because the tail already
        // authors a rest:space here and a case that re-authored the value
        // it found would have agreed with everything.
        // What rest:space cannot carry is a connection that ENDS at one of
        // the six computed spaces, where exec's computeValue is a
        // computation and the walk would read a raw authored value instead.
        {
            const UsdAttribute a = prim.GetAttribute(TfToken("rest:space"));
            if (a && _ConnectionReachesComputedSpace(a)) {
                say("connected rest:space on provider", path);
            }
        }
        // default:space IS one of the five expressions, and the expression
        // is not a value read at all: a connection is authoritative even
        // when it resolves to the identity, a raw identity selects the
        // COMPUTED fallback instead, and only a non-identity raw value is
        // taken verbatim (_ComputeSpaceExpression). The walk's rule --
        // deepest attribute on the chain that has a value -- is a different
        // rule, so a connected one still refuses. Unconnected it is the
        // ladder proper, and bakes per frame like the rest.
        {
            const UsdAttribute a =
                prim.GetAttribute(TfToken("default:space"));
            SdfPathVector connections;
            if (a && a.HasAuthoredConnections() &&
                a.GetConnections(&connections) && !connections.empty()) {
                say("connected default:space on provider", path);
            }
        }
        // avars:rotationSign is captured once per slot (Build folds it, so
        // an edit rebuilds), while exec reads it every frame: a sign that
        // varies, is connected or is written by a chain is not one value.
        {
            const UsdAttribute a = prim.GetAttribute(rotationSignName);
            if (a && (a.ValueMightBeTimeVarying() ||
                      a.HasAuthoredConnections())) {
                say("animated or connected avars:rotationSign on provider",
                    path);
            }
            if (chainTargets.count(path.AppendProperty(rotationSignName))) {
                say("property chain writes avars:rotationSign on provider",
                    path);
            }
        }
        (void)probe;
    }
    for (const SdfPath &joint : E._jointPaths) {
        if (!E._firstFramePoseFrames.count(joint)) {
            say("joint is not a seeded pose provider", joint);
        }
    }
    for (const SdfPath &control : E._controlPaths) {
        if (!E._firstFramePoseFrames.count(control)) {
            say("control is not a seeded pose provider", control);
        }
    }

    std::set<SdfPath> batched;
    for (const auto &batch : E._solverBatches) {
        for (const auto &[solverPath, tap] : batch.solvers) {
            batched.insert(solverPath);
            const UsdPrim prim = E._stage->GetPrimAtPath(solverPath);
            const TfToken type = prim ? prim.GetTypeName() : TfToken();
            if (!_IsBakedSolverType(type)) {
                say("solver type not baked (" + type.GetString() + ")",
                    solverPath);
            }
            // A TwoBoneIk that does not bind three joint rests used to be
            // refused here, resolved a second time by a copy of bakeSolver's
            // rule. It is not a rig the program cannot express: the
            // computation warns and publishes an empty aggregate, and
            // bakeSolver reproduces that through Solver::degenerate. One
            // expression of the rule, in the place that needs its answer.
        }
    }
    // An aggregate solver no batch runs is not a rig the program cannot
    // express: the dynamic path computes it inside the guide request,
    // against the walk's FINAL frames, and the program runs it as a Solve
    // step after the walk for the same reason. Its TYPE still has to be one
    // the program expresses, which the batch loop above never asked about
    // because it never saw it.
    for (const auto &[solverPath, tap] : E._solverArrayTaps) {
        if (batched.count(solverPath)) {
            continue;
        }
        const UsdPrim prim = E._stage->GetPrimAtPath(solverPath);
        const TfToken type = prim ? prim.GetTypeName() : TfToken();
        if (!_IsBakedSolverType(type)) {
            say("solver type not baked (" + type.GetString() + ")",
                solverPath);
        }
    }

    // A weight object is a COMPOSITION -- a dynamic weight remaps a base, a
    // combine folds a list -- so every question about one is a question about
    // its closure and not about the object a mover or a constraint happens to
    // name. Both walks below carry a visited set of their own rather than
    // trusting the compile pass to have rejected a cycle first: a
    // bakeability check that only terminates because somebody else checked is
    // not one to leave in place.
    const auto walkWeights = [&E](const SdfPath &root,
                                  const std::function<bool(const UsdPrim &)>
                                      &visit) {
        std::set<SdfPath> seen;
        std::function<void(const SdfPath &)> walk = [&](const SdfPath &path) {
            if (path.IsEmpty() || !seen.insert(path).second) {
                return;
            }
            const UsdPrim prim = E._stage->GetPrimAtPath(path);
            if (!prim || !visit(prim)) {
                return;
            }
            for (const char *name : {"rigExec:baseWeight",
                                     "rigExec:inputWeights"}) {
                for (const SdfPath &input : _Targets(prim, name)) {
                    walk(input);
                }
            }
        };
        walk(root);
    };
    const auto sayUnbakedWeights = [&](const SdfPath &root) {
        walkWeights(root, [&](const UsdPrim &prim) {
            const TfToken type = prim.GetTypeName();
            if (!_IsBakedWeightType(type)) {
                say("weight object type not baked (" + type.GetString() + ")",
                    prim.GetPath());
                return false;
            }
            return true;
        });
    };

    for (const auto &constraint : E._frameConstraints) {
        // A transform-domain matrix mover rides the constraint walk without
        // being a constraint operator, so it is admitted by name here
        // rather than through the operator registry's subset check.
        if (constraint.schemaType != "RigExecMatrixMover" &&
            !_IsBakedConstraintType(constraint.schemaType)) {
            say("constraint type not baked (" +
                    constraint.schemaType.GetString() + ")",
                constraint.moverPath);
            continue;
        }
        // A volume anywhere in the closure used to be refused here, on the
        // grounds that a constraint's envelope is resolved by the CPU oracle
        // against the volume placements AS THEY STAND AT THE CONSTRAINT'S
        // POINT IN THE WALK, while the program places volumes once, after
        // the walk. That reasoning describes a constraint
        // the epoch cannot hold: a volumetric field requires a POINT domain
        // (_ValidateWeightObjectDomain, composed inputs included), so the
        // only constraint that can bind one is a geometry-domain constraint
        // -- and a geometry-domain constraint resolves NO envelope in the
        // walk at all. Its weight is per point and resolves after the solve,
        // on the revision its delta feeds, where the placement is the same
        // one every other mover's packet uses. So no placement is read
        // mid-walk, and the per-volume VolumePlacements steps after the walk
        // are what every reader sees.
        sayUnbakedWeights(constraint.weightObject);
        if (constraint.targets.empty()) {
            say("constraint names no target", constraint.moverPath);
            continue;
        }
        // Either provider family: an exec-seeded one, or a plain Xformable
        // the program seeds from the stage in its prologue. Both take a slot
        // in the same table, which is what the dynamic walk's one frame map
        // holds. EVERY target, not only the first: a SingleChainIK revises
        // its whole chain atomically, and the dynamic walk records every
        // target of every constraint whatever it revises.
        for (const SdfPath &target : constraint.targets) {
            if (!E._firstFramePoseFrames.count(target) &&
                !E._xformDerivedProviders.count(target)) {
                say("constraint target is not a seeded pose provider",
                    target);
            } else if (E._firstFramePoseFrames.count(target) &&
                       E._xformDerivedProviders.count(target)) {
                // The two families are documented as disjoint, and they are
                // for every provider but one: a VOLUME WEIGHT is exec-seeded
                // like a joint AND, because its type is neither
                // RigExecControl nor RigExecJoint, is catalogued as a plain
                // Xformable the moment a constraint targets it. The dynamic
                // walk then resolves the collision by LAST WRITER: the
                // xform-derived pass runs after the compose and overwrites
                // the volume's rest, base and final with an identity rest
                // and a transform read off the stage, discarding the avar
                // composition entirely -- while exec's computeWeightPacket
                // goes on placing that same volume from its avars. Two
                // placements, and the program has one slot to hold them in.
                // Refused rather than guessed, and refused narrowly: a
                // volume a constraint does NOT target bakes, which is what
                // lifting "volume weight object on constraint" above was
                // about. The dynamic path's own parity mode declines to
                // publish a reference-phase field on such a volume
                // ("mover graph parity failed"), so which of the two
                // placements is intended is not a question the program can
                // answer by reading either side.
                say("constraint target is both exec-seeded and "
                    "xform-derived",
                    target);
            }
        }
    }

    // Every volume the walk places needs a program slot; Build checks the
    // same set from its own table.
    {
        std::set<SdfPath> programVolumes;
        const auto noteVolume = [&](const SdfPath &path) {
            const UsdPrim prim = E._stage->GetPrimAtPath(path);
            if (prim && _IsVolumeWeightTypeName(prim.GetTypeName())) {
                programVolumes.insert(path);
            }
        };
        for (const auto &[path, tap] : E._firstFramePoseFrames) {
            noteVolume(path);
        }
        for (const SdfPath &path : E._xformDerivedProviders) {
            noteVolume(path);
        }
        for (const SdfPath &path : _WalkVolumesWithoutASlot(
                 programVolumes, E._volumeWeightMatrixTaps)) {
            say("a volume weight the walk places has no program slot", path);
        }
    }

    // A read phase that names a SOLVER checkpoint reads a frame record on the
    // solver's commit, which exists only for a joint the solver's batch
    // commits. The compile names a writer of the joint, so this is a
    // consistency guard; Build checks the same condition.
    for (const SdfPath &solver : _SolverCheckpointsWithoutAnOutput(
             E._chainPlan.snapshots, E._solverDependencies, E._solverJoints,
             _BatchedSolvers(E._solverBatches))) {
        say("read phase names a solver checkpoint the walk does not commit "
            "for that joint",
            solver);
    }

    auto checkRevision = [&](const RigExecRigEvaluator::_GraphRevision &r,
                             bool derived) {
        const bool supported =
            derived ? (r.op == RigExecRevisionOp::RecomputeExtent ||
                       r.op == RigExecRevisionOp::RecomputeNormals ||
                       RigExecIsDerivedMatrixOp(r.op))
                    : (r.op == RigExecRevisionOp::Skin ||
                       r.op == RigExecRevisionOp::Matrix ||
                       // Every operation whose whole packet the per-frame
                       // assembler reads off the stage: the program hands
                       // RigExecAssembleParameters the same binding and the
                       // same resolved inputs the dynamic walk hands it, and
                       // RigExecRunRevisionKernel is the same kernel. What
                       // separates these from the ones still refused below is
                       // that none of them needs a value the pose walk has
                       // not already produced.
                       r.op == RigExecRevisionOp::BlendShape ||
                       r.op == RigExecRevisionOp::External ||
                       r.op == RigExecRevisionOp::EmitGuidePoints ||
                       r.op == RigExecRevisionOp::Ribbon ||
                       r.op == RigExecRevisionOp::Wire ||
                       r.op == RigExecRevisionOp::VolumeCorrect ||
                       r.op == RigExecRevisionOp::Smooth ||
                       r.op == RigExecRevisionOp::DeltaMush ||
                       r.op == RigExecRevisionOp::Wrinkle ||
                       r.op == RigExecRevisionOp::Lattice ||
                       r.op == RigExecRevisionOp::SurfaceProject);
        if (!supported) {
            say(std::string("mover operation not baked (") +
                    RigExecBakedOpName(r.op) +
                    ")",
                r.moverPath);
            return;
        }
        sayUnbakedWeights(r.binding.weightObject);
        // The frames the walk hands a curve mover are the aggregate a
        // BATCHED solver publishes -- the dynamic path taps the solver's
        // computePointFrameArray and then overrides the tap with the walk's
        // own solve, so the program's aggregate table is the same number.
        // A solver in no batch has no aggregate for the program to point at,
        // and one whose type the bake declines has already said so above.
        if (!r.binding.driverFrames.IsEmpty() &&
            !batched.count(r.binding.driverFrames)) {
            say("driver frames solver is in no batch", r.binding.driverFrames);
        }
        if (r.op == RigExecRevisionOp::Skin) {
            const UsdPrim prim = E._stage->GetPrimAtPath(r.moverPath);
            // The layout is captured once; the kernel then does an O(1) shape
            // check per frame instead of re-reading three arrays.
            for (const char *name : {"rigExec:jointIndices",
                                     "rigExec:jointWeights",
                                     "rigExec:elementSize",
                                     "rigExec:skinningMethod"}) {
                if (const UsdAttribute a = prim.GetAttribute(TfToken(name))) {
                    if (RigExecBakedAnimatedOrConnected(a)) {
                        say(std::string("time-varying or connected ") + name,
                            r.moverPath);
                    }
                }
            }
            for (const SdfPath &influence : r.binding.influences) {
                if (!E._firstFramePoseFrames.count(influence)) {
                    say("skin influence is not a seeded pose provider",
                        influence);
                }
            }
        }
    };
    for (const auto &[target, revisions] : E._graphChains) {
        for (const auto &revision : revisions) {
            checkRevision(revision, /* derived = */ false);
        }
    }
    for (const auto &[target, revisions] : E._graphDerivedChains) {
        for (const auto &revision : revisions) {
            checkRevision(revision, /* derived = */ true);
        }
    }

    // The reasons are a set in spirit: one line per feature, not per mention.
    std::sort(out.begin() + long(before), out.end());
    out.erase(std::unique(out.begin() + long(before), out.end()), out.end());
    return out.size() == before;
}

// The bake.

RigExecBakedProgram::RigExecBakedProgram(
    std::unique_ptr<RigExecBakedProgramImpl> impl)
    : _impl(std::move(impl))
{
}

RigExecBakedProgram::~RigExecBakedProgram() = default;

size_t RigExecBakedProgram::GetProviderCount() const {
    return _impl->paths.size();
}
size_t RigExecBakedProgram::GetBoundInputCount() const {
    return _impl->boundInputs;
}
size_t RigExecBakedProgram::GetVaryingInputCount() const {
    return _impl->varyingInputs;
}

void RigExecBakedProgram::ReleaseStageReferences() {
    _impl->stage.Reset();
    _impl->resolveBlendSample = nullptr;
}

void RigExecBakedProgram::AdoptGeometryStateFrom(
    RigExecBakedProgram &previous) {
    RigExecBakedProgramImpl &B = *_impl;
    RigExecBakedProgramImpl &P = *previous._impl;
    // One revision's run state, moved from the node the outgoing program
    // held. Everything here is a CACHE of the last run; the compiled
    // description around it comes from the epoch the new program was built
    // from and is left alone.
    // \p keepRun says whether the cached RESULT may be kept as well as the
    // node: a revision spliced into or out of a chain changes the point
    // stream every revision after it reads, which is why the dynamic path's
    // VdfNetwork re-executes them, so from the first divergence on the node
    // survives but its result does not.
    const auto adopt = [](RigExecBakedProgramImpl::GeomRevision *destination,
                          RigExecBakedProgramImpl::GeomRevision *source,
                          bool keepRun) {
        destination->created = false;
        // The layout the outgoing SkinTopology op held, which the new op's
        // first build of the same layout hands back: the packet the kept
        // run compares against carries that object, as the evaluator's
        // cache handed it to both programs.
        destination->layoutCandidate = source->layoutHandle
                                           ? source->layoutHandle
                                           : source->layoutCandidate;
        if (!keepRun) {
            return;
        }
        destination->resultStatus = source->resultStatus;
        destination->output = std::move(source->output);
        destination->lastParameters = std::move(source->lastParameters);
        // A DERIVED revision's remembered input lives beside the packet
        // rather than inside it -- the chain's 315KB point buffer, held by
        // handle with `lastParameters.auxPoints` left empty -- so it has to
        // travel with the packet it was split from. Leaving it behind gives
        // the adopted node a `ran` that says "compare against what I last
        // saw" and an empty array to compare against, so every derived
        // revision of the rig re-executes on the first generation after any
        // edit, bumps `revisionsExecuted` and emits its diagnostic, while
        // the dynamic path -- whose graphs stood through the same edit --
        // reports none of it. Exactly the failure lastDefaultWeight below
        // describes, one field further along.
        destination->lastAuxPoints = std::move(source->lastAuxPoints);
        destination->lastStatus = source->lastStatus;
        destination->ran = source->ran;
        // The FOLDED influence table, which is the other half of the
        // comparison `ran` promises. A skin revision re-executes when one of
        // its matrices moved, and `influencesChanged` is decided by the fold
        // against the table it last wrote -- so a node that kept its `ran`
        // and lost its table compares this run's matrices against a table
        // that has never held one, reports every entry changed and runs.
        // That is the same divergence the packet fields above describe,
        // reached through the fold instead of through the packet: the
        // dynamic path's VdfNetwork keeps the buffers of the nodes it
        // reconnects, so it reports no such work.
        // Carried only where the two tables are the same shape. keepRun says
        // the revision is the same mover at the same place in the chain; it
        // does not say its binding still names the same joints, and the fold
        // writes one entry per influence slot of the NEW binding.
        if (source->influences.size() == destination->influences.size()) {
            destination->influences = std::move(source->influences);
            // And the fold's verdict on that table, which is a function of
            // the table alone. The fold is a step like any other, so the
            // first run of the new program skips it when nothing it reads
            // moved -- a blend shape's fold reads nothing at all -- and a
            // table that came across without its verdict would leave the
            // fuse reading the `false` a fresh revision starts at.
            destination->influencesValid = source->influencesValid;
        }
        // The last run's envelope scalar belongs to the cached result the
        // same way the packet does: a node whose `ran` survives a rebuild
        // must not then compare this against the zero a fresh revision
        // starts at, or every revision of the rig re-executes on the first
        // generation after any edit.
        destination->lastDefaultWeight = source->lastDefaultWeight;
        // WHICH buffer the chain's running value was in, which is as much a
        // part of the cached result as the buffer itself: a revision that
        // does not execute publishes through this indirection, and a kept
        // result with a lost source would publish the authored base. Valid
        // in the new chain because keepRun is only true below the first
        // divergence, where the two identity sequences agree position by
        // position.
        destination->currentSource = source->currentSource;
    };
    std::map<SdfPath, RigExecBakedProgramImpl::GeomChain *> outgoing;
    for (RigExecBakedProgramImpl::GeomChain &chain : P.chains) {
        outgoing.emplace(chain.target, &chain);
    }
    for (RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        const auto found = outgoing.find(chain.target);
        if (found == outgoing.end()) {
            // A target the outgoing program did not drive: every revision of
            // it is genuinely new, which is also what the dynamic path says
            // about a live graph it has just created.
            continue;
        }
        RigExecBakedProgramImpl::GeomChain &old = *found->second;
        std::map<std::pair<SdfPath, RigExecRevisionOp>,
                 RigExecBakedProgramImpl::GeomRevision *> retained;
        for (RigExecBakedProgramImpl::GeomRevision &revision : old.revisions) {
            retained.emplace(std::make_pair(revision.moverPath, revision.op),
                             &revision);
        }
        // The first position at which the two identity sequences disagree:
        // everything before it reads the same point stream as before and
        // everything from it on does not.
        size_t divergence = 0;
        while (divergence < chain.revisions.size() &&
               divergence < old.revisions.size() &&
               old.revisions[divergence].moverPath ==
                   chain.revisions[divergence].moverPath &&
               old.revisions[divergence].op == chain.revisions[divergence].op) {
            ++divergence;
        }
        const bool sameSequence = divergence == chain.revisions.size() &&
                                  divergence == old.revisions.size();
        for (size_t i = 0; i < chain.revisions.size(); ++i) {
            RigExecBakedProgramImpl::GeomRevision &revision =
                chain.revisions[i];
            const auto node =
                retained.find(std::make_pair(revision.moverPath, revision.op));
            if (node == retained.end()) {
                continue;
            }
            adopt(&revision, node->second, /* keepRun = */ i < divergence);
            retained.erase(node);
        }
        // Insertion, removal and reordering rebuild the schedule; a rebind
        // only updates packets. Same rule, same words, as the dynamic walk.
        chain.scheduleDirty = !sameSequence;
        chain.lastBase = std::move(old.lastBase);
        chain.result = std::move(old.result);
        chain.haveResult = old.haveResult;

        std::map<SdfPath, RigExecBakedProgramImpl::GeomChain::Derived *>
            outgoingDerived;
        for (RigExecBakedProgramImpl::GeomChain::Derived &derived :
                 old.derived) {
            outgoingDerived.emplace(derived.target, &derived);
        }
        for (RigExecBakedProgramImpl::GeomChain::Derived &derived :
                 chain.derived) {
            const auto match = outgoingDerived.find(derived.target);
            if (match == outgoingDerived.end() ||
                match->second->revision.moverPath !=
                    derived.revision.moverPath ||
                match->second->revision.op != derived.revision.op) {
                continue;
            }
            adopt(&derived.revision, &match->second->revision,
                  /* keepRun = */ true);
            derived.lastBase = std::move(match->second->lastBase);
            derived.result = std::move(match->second->result);
            derived.haveResult = match->second->haveResult;
            derived.matrix = match->second->matrix;
            derived.haveMatrix = match->second->haveMatrix;
        }
    }
}

// Invalidation.

namespace {

// A live input's own spec appearing or going away -- the first value a
// layer authors for an animator's channel, or the undo that removes it.
// The frame reads that input the long way every run, so what it composes to
// afterwards needs no rebuild: it routes like a value edit. A spec that
// brings anything but a value (a connection, metadata) is still structure.
bool
_IsLiveInputResync(const RigExecBakedProgramImpl &B,
                   const UsdNotice::ObjectsChanged &notice,
                   const SdfPath &path)
{
    if (!path.IsPropertyPath() || B.rebuild.count(path) ||
        B.xformPrims.count(path.GetPrimPath())) {
        return false;
    }
    const auto found = B.overridableInputs.find(path);
    if (found == B.overridableInputs.end()) {
        return false;
    }
    for (const int index : found->second) {
        if (index < 0 || size_t(index) >= B.cones.editRoute.size() ||
            !B.cones.editRoute[size_t(index)]) {
            return false;
        }
    }
    static const TfToken kDefault("default");
    static const TfToken kTimeSamples("timeSamples");
    static const TfToken kSpline("spline");
    static const TfToken kTypeName("typeName");
    static const TfToken kCustom("custom");
    for (const TfToken &field : notice.GetChangedFields(path)) {
        if (field != kDefault && field != kTimeSamples && field != kSpline &&
            field != kTypeName && field != kCustom) {
            return false;
        }
    }
    return true;
}

}  // namespace

bool
RigExecBakedProgram::IsInvalidatedBy(
    const UsdNotice::ObjectsChanged &notice) const
{
    const RigExecBakedProgramImpl &B = *_impl;
    // A changed-info notice is a VALUE edit on a property that already
    // existed. It matters only where the bake read that value -- and, for
    // the Xforms bakeability accepted for composing to the identity, on any
    // property of theirs at all, since it is their composed transform and
    // not one named attribute that was judged. The prim-level info that
    // arrives for every ancestor of an edit (an `over` being created above
    // it) is not that, and is exactly what must not rebuild.
    static const TfToken kDefault("default");
    static const TfToken kTimeSamples("timeSamples");
    static const TfToken kSpline("spline");
    for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
        if (B.rebuild.count(path) ||
            (path.IsPropertyPath() &&
             B.xformPrims.count(path.GetPrimPath()))) {
            return true;
        }
        // Anything but a value on a property the bake asked about by name:
        // a connection or a target retargeted, or metadata that changes how
        // the value is read. The walk the bake recorded for that property
        // -- and with it which upstream hops a later value edit is routed
        // through (ApplyValueEdits) -- may no longer be the walk the frame
        // takes, and only a rebuild records the new one.
        if (path.IsPropertyPath() && B.named.count(path)) {
            for (const TfToken &field : notice.GetChangedFields(path)) {
                if (field != kDefault && field != kTimeSamples &&
                    field != kSpline) {
                    return true;
                }
            }
        }
    }
    // A resync names a subtree whose composition changed: properties can have
    // appeared, disappeared or been retargeted anywhere inside it, so the
    // question is whether the subtree and the index overlap in EITHER
    // direction -- the resync above something the bake read, or at a property
    // of a prim it read from.
    const auto overlaps = [&B](const SdfPath &path) {
        for (const std::set<SdfPath> *index :
                 {&B.named, &B.prims, &B.xformPrims}) {
            // Descendants of `path` are contiguous from lower_bound: SdfPath
            // sorts in namespace order, which _OnObjectsChanged already
            // relies on for the solver-batch subtree walk.
            const auto it = index->lower_bound(path);
            if (it != index->end() && it->HasPrefix(path)) {
                return true;
            }
        }
        // A resync that names a PROPERTY is that property appearing,
        // disappearing or being retargeted. It matters where the bake asked
        // about that property by name -- not merely somewhere on the prim,
        // or every rig would rebuild for a property it never reads.
        const SdfPath prim = path.GetPrimPath();
        if (path.IsPropertyPath()) {
            return B.xformPrims.count(prim) > 0;
        }
        return B.prims.count(prim) > 0 || B.xformPrims.count(prim) > 0;
    };
    for (const SdfPath &path : notice.GetResyncedPaths()) {
        if (!_IsLiveInputResync(B, notice, path) && overlaps(path)) {
            return true;
        }
    }
    for (const SdfPath &path : notice.GetResolvedAssetPathsResyncedPaths()) {
        if (overlaps(path)) {
            return true;
        }
    }
    return false;
}

// Exact equality, not a tolerance: the program exists to produce the same
// numbers, so "close" is a failure with extra steps.
void
RigExecComparePoses(const RigExecRigPose &reference,
                    const RigExecRigPose &baked, RigExecRigPose *out)
{
    // Read FIRST, before anything below appends: `out` is allowed to be the
    // reference pose -- the evaluator passes it that way, so a disagreement
    // is reported on the generation it publishes -- and a diagnostic appended
    // by an earlier domain would otherwise make this domain disagree about
    // a pose that agreed.
    const bool sameDiagnostics = reference.diagnostics == baked.diagnostics;
    const auto sameFrame = [](const RigExecPointFrame &a,
                              const RigExecPointFrame &b) {
        return a.flags == b.flags && a.points == b.points;
    };
    const auto compare = [&out](const auto &referenceMap, const auto &bakedMap,
                                const char *what, auto equal) {
        for (const auto &[path, value] : referenceMap) {
            const auto found = bakedMap.find(path);
            if (found == bakedMap.end()) {
                out->diagnostics.push_back(
                    std::string("baked parity: no ") + what + " for " +
                    path.GetString());
                ++out->bakedParityMismatches;
            } else if (!equal(value, found->second)) {
                out->diagnostics.push_back(
                    std::string("baked parity: ") + what + " differs at " +
                    path.GetString());
                ++out->bakedParityMismatches;
            }
        }
        for (const auto &[path, value] : bakedMap) {
            if (!referenceMap.count(path)) {
                out->diagnostics.push_back(
                    std::string("baked parity: unexpected ") + what + " at " +
                    path.GetString());
                ++out->bakedParityMismatches;
            }
        }
    };
    const auto sameMatrix = [](const GfMatrix4d &a, const GfMatrix4d &b) {
        return a == b;
    };
    compare(reference.jointFramesBase, baked.jointFramesBase,
            "base joint frame", sameFrame);
    compare(reference.jointFramesFinal, baked.jointFramesFinal,
            "final joint frame", sameFrame);
    compare(reference.jointMatricesFinal, baked.jointMatricesFinal,
            "joint matrix", sameMatrix);
    compare(reference.controlFrames, baked.controlFrames, "control frame",
            sameFrame);
    compare(reference.providerXforms, baked.providerXforms,
            "provider transform", sameMatrix);
    compare(reference.providerBaseXforms, baked.providerBaseXforms,
            "provider base transform", sameMatrix);
    compare(reference.movedProperties, baked.movedProperties,
            "moved property",
            [](const VtValue &a, const VtValue &b) { return a == b; });
    // The observational guides too: they are a published domain like any
    // other, and a solver aggregate that drifts shows up here first.
    compare(reference.solverFrames, baked.solverFrames, "solver frames",
            [&sameFrame](const std::vector<RigExecPointFrame> &a,
                         const std::vector<RigExecPointFrame> &b) {
                return a.size() == b.size() &&
                       std::equal(a.begin(), a.end(), b.begin(), sameFrame);
            });
    // The weight domains. They were compared here before anything could
    // fill them, which is why a weight object's very first baked generation
    // was measured against the dynamic path instead of against a comparator
    // that had never been taught to look -- and they are both filled now: a
    // mover's resolved field, and where every volume weight ended the walk.
    // A field is equal iff it weights the same property with the same
    // floats, bit for bit -- a resolved field is what a mover actually
    // consumed, and an element one path clamped and the other did not is
    // exactly the difference a size check cannot see.
    compare(reference.weightFields, baked.weightFields, "weight field",
            [](const RigExecResolvedWeightField &a,
               const RigExecResolvedWeightField &b) {
                return a.target == b.target && a.weights == b.weights;
            });
    compare(reference.weightFrames, baked.weightFrames, "weight frame",
            sameMatrix);

    // The SCALARS of the generation. They are published state a consumer
    // reads -- an editor shows the work counters, a test asserts on them --
    // so a program that lands on the right points while reporting different
    // work is still a second rig, and the difference is exactly the shape a
    // map comparison cannot see. Counted separately from the maps: each is
    // its own domain, so a mismatch of one names which one moved. The
    // diagnostics are compared in ORDER, because the order is the walk order
    // and a consumer reading "MoverFailed X" after "constraint Y passed
    // through" is being told a sequence.
    const auto compareCount = [&out](size_t referenceValue, size_t bakedValue,
                                     const char *what) {
        if (referenceValue != bakedValue) {
            out->diagnostics.push_back(
                std::string("baked parity: ") + what + " differs (" +
                std::to_string(referenceValue) + " vs " +
                std::to_string(bakedValue) + ")");
            ++out->bakedParityMismatches;
        }
    };
    compareCount(reference.moverGraphRevisionsCreated,
                 baked.moverGraphRevisionsCreated,
                 "mover graph revisions created");
    compareCount(reference.moverGraphRevisionsExecuted,
                 baked.moverGraphRevisionsExecuted,
                 "mover graph revisions executed");
    compareCount(reference.moverGraphSchedulesBuilt,
                 baked.moverGraphSchedulesBuilt,
                 "mover graph schedules built");
    compareCount(reference.solverOverrideRounds, baked.solverOverrideRounds,
                 "solver override rounds");
    // Convergence is a published scalar of the same kind, and the one whose
    // wrong answer is the quietest: a consumer that reads it sees "the
    // overrides settled" and reads the points below it as final. The baked
    // path says true unconditionally today because only an incomplete exec
    // snapshot clears it and there is no exec there -- which is an agreement
    // exactly as long as the dynamic path's snapshot keeps completing.
    if (reference.solverOverridesConverged != baked.solverOverridesConverged) {
        out->diagnostics.push_back(
            std::string("baked parity: solver overrides converged differs (") +
            (reference.solverOverridesConverged ? "true" : "false") + " vs " +
            (baked.solverOverridesConverged ? "true" : "false") + ")");
        ++out->bakedParityMismatches;
    }
    // solverEvaluations is deliberately NOT compared. It counts the solver
    // computations the dependency schedule actually REQUESTED, and the
    // dynamic path's per-batch exec cache lets it skip a batch whose time and
    // inputs are the ones it already answered -- so re-evaluating the same
    // frame twice costs it nothing and costs the program, which holds no such
    // cache and re-solves, its whole schedule. Both numbers are true of the
    // path that reported them; they are not two answers to one question, and
    // making them agree would mean either the program inventing a cache or
    // the dynamic path giving one up.
    // movedPropertiesCpu, moverGraphParityMismatches and
    // moverGraphParityAgreements are left out for the opposite reason: they
    // are filled only by cpuParityMode, and cpuParityMode turns the baked
    // path OFF (the independent CPU oracle is what that mode asks for, and
    // the program is not it -- it shares the kernels). So in every
    // generation this function ever sees, all three are empty or zero on
    // both sides, and comparing them would assert a tautology. `valid` and
    // `time` likewise: the parity generation publishes the DYNAMIC pose, so
    // its own are the only ones a consumer reads.
    if (!sameDiagnostics) {
        out->diagnostics.push_back(
            "baked parity: diagnostics differ (" +
            std::to_string(reference.diagnostics.size()) + " vs " +
            std::to_string(baked.diagnostics.size()) + " line(s))");
        ++out->bakedParityMismatches;
    }
}

// Interactive overrides.

void
RigExecBakedProgram::BumpProgramStamp()
{
    ++_impl->programStamp;
}

namespace {

// The attributes a resolved-input reader can reach through connections; see
// `connectedSources`. Every authored attribute of every resolvedRoutedPrims
// prim is a walk head, and every connection target is followed on -- all of
// them, where the reader follows only a single one: a reader that stops at
// two connections reads the attribute's own value instead, which is on a
// routed prim already, so the extra targets only report more.
const std::set<SdfPath> &
_ConnectedSources(const RigExecBakedProgramImpl &B)
{
    if (B.connectedSourcesFilled &&
        B.connectedSourcesStamp == B.programStamp) {
        return B.connectedSources;
    }
    B.connectedSources.clear();
    B.connectedSourcesStamp = B.programStamp;
    B.connectedSourcesFilled = true;
    if (!B.stage) {
        return B.connectedSources;
    }
    std::vector<UsdAttribute> pending;
    for (const SdfPath &path : B.resolvedRoutedPrims) {
        const UsdPrim prim = B.stage->GetPrimAtPath(path);
        if (!prim) {
            continue;
        }
        for (const UsdAttribute &attribute : prim.GetAuthoredAttributes()) {
            if (attribute.HasAuthoredConnections()) {
                pending.push_back(attribute);
            }
        }
    }
    SdfPathVector connections;
    while (!pending.empty()) {
        const UsdAttribute attribute = std::move(pending.back());
        pending.pop_back();
        connections.clear();
        attribute.GetConnections(&connections);
        for (const SdfPath &target : connections) {
            if (!B.connectedSources.insert(target).second ||
                !target.IsPropertyPath()) {
                continue;
            }
            const UsdAttribute next = B.stage->GetAttributeAtPath(target);
            if (next && next.HasAuthoredConnections()) {
                pending.push_back(next);
            }
        }
    }
    return B.connectedSources;
}

// Decides where a stage VALUE edit lands in the program without writing
// anything, so the dry run and ApplyValueEdits share every check by
// construction. The caller has already asked IsInvalidatedBy, so no path
// here is one whose value the bake captured.
// Fills \p indices with the override numbers of the per-frame inputs the
// notice reached, and \p readPaths with every property path it names that
// something in the program can read -- the edited inputs, the properties of
// a prim read through the generation's resolved inputs, the sources those
// reads reach through connections, and the properties of a prim a
// value-compared source reads whole. A property that is none of these is
// read by nothing, and is in neither list. Returns false when the
// notice holds anything that can only be answered by running the whole
// program once (a stamp bump).
bool
_RouteValueEdits(const RigExecBakedProgramImpl &B,
                 const UsdNotice::ObjectsChanged &notice,
                 std::vector<int> *indices, std::vector<SdfPath> *readPaths,
                 std::vector<SdfPath> *leafPaths = nullptr,
                 bool skipPatchableAvars = false)
{
    // The transform channels the in-place patch moves, when the caller
    // applies that patch beside this routing.
    const auto patchedElsewhere = [&B, skipPatchableAvars](
                                      const SdfPath &path) {
        return skipPatchableAvars && B.patchableAvars.count(path) > 0;
    };
    // Every resync still runs the program whole -- which retained query or
    // cached binding a spec appearing or going away leaves stale is not
    // indexed per path -- except a live input's own spec, which the frame
    // re-reads anyway (_IsLiveInputResync).
    if (!notice.GetResolvedAssetPathsResyncedPaths().empty()) {
        return false;
    }
    std::vector<int> resyncDecided;
    std::vector<SdfPath> resyncRead;
    for (const SdfPath &path : notice.GetResyncedPaths()) {
        if (patchedElsewhere(path)) {
            continue;
        }
        if (!_IsLiveInputResync(B, notice, path)) {
            return false;
        }
        for (const int index : B.overridableInputs.find(path)->second) {
            resyncDecided.push_back(index);
        }
        resyncRead.push_back(path);
    }
    static const TfToken kDefault("default");
    static const TfToken kTimeSamples("timeSamples");
    static const TfToken kSpline("spline");
    // A prim some source of the program reads as a whole rather than by
    // property name: the Xforms accepted for composing to the identity, a
    // prim whose reads all go through the resolved inputs (a property-chain
    // mover, a geometry mover, a weight object), and every prim the bake
    // read from or an ancestor of one -- a transform read relative to the
    // asset root sees every Xform above the prim.
    const auto readWhole = [&B](const SdfPath &prim) {
        if (B.xformPrims.count(prim) || B.resolvedRoutedPrims.count(prim)) {
            return true;
        }
        const auto below = B.prims.lower_bound(prim);
        return below != B.prims.end() && below->HasPrefix(prim);
    };
    std::vector<int> decided = std::move(resyncDecided);
    std::vector<SdfPath> read = std::move(resyncRead);
    for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
        const std::vector<TfToken> fields = notice.GetChangedFields(path);
        // Layer metadata: the time codes, the frame rate -- read by every
        // time conversion and indexed nowhere.
        if (path.IsAbsoluteRootPath()) {
            return false;
        }
        if (patchedElsewhere(path)) {
            continue;
        }
        if (!path.IsPropertyPath()) {
            // The empty info every ancestor of an edited spec reports moves
            // nothing. Prim metadata on a prim nothing reads moves nothing
            // either; on one the program reads, it is not a value the index
            // can place.
            if (fields.empty() || !readWhole(path.GetPrimPath())) {
                continue;
            }
            return false;
        }
        for (const TfToken &field : fields) {
            if (field != kDefault && field != kTimeSamples &&
                field != kSpline) {
                return false;
            }
        }
        const auto found = B.overridableInputs.find(path);
        if (found != B.overridableInputs.end()) {
            // A per-frame input, on its head or on an upstream hop of its
            // walk: routed exactly where a drag on it would be, provided
            // every input the path feeds has a reader that re-reads it.
            for (const int index : found->second) {
                if (index < 0 || size_t(index) >= B.cones.editRoute.size() ||
                    !B.cones.editRoute[size_t(index)]) {
                    return false;
                }
                decided.push_back(index);
            }
            read.push_back(path);
            continue;
        }
        const SdfPath prim = path.GetPrimPath();
        // Read through the resolved inputs, which the prologue rebuilds
        // every run: the property chains compare their results, and the
        // geometry movers and weight objects are assembled by sources that
        // compare what they assembled.
        if (B.resolvedRoutedPrims.count(prim)) {
            read.push_back(path);
            continue;
        }
        // Asked about by name and not an input a step declares: a reader
        // the index cannot name, so the whole program answers it.
        if (B.named.count(path)) {
            return false;
        }
        // Not read by name at all. A source that reads its prim whole -- a
        // stage transform, a chain's base points -- re-reads it every run
        // and compares by value, and so does a resolved-input reader whose
        // connection reaches the property, so the program owes either
        // nothing; the frame cache still does, and the path is reported for
        // that.
        if (readWhole(prim) || (!B.resolvedRoutedPrims.empty() &&
                                _ConnectedSources(B).count(path))) {
            read.push_back(path);
        } else if (leafPaths && B.leafByPath.count(path)) {
            // A hop only a leaf reads (a shader dial on a prim nothing else
            // reads): the leaf re-reads it, which is all the program owes.
            leafPaths->push_back(path);
        }
    }
    if (indices) {
        *indices = std::move(decided);
    }
    if (readPaths) {
        *readPaths = std::move(read);
    }
    return true;
}

}  // namespace

bool
RigExecBakedProgram::DryRunValueEdits(
    const UsdNotice::ObjectsChanged &notice,
    std::vector<SdfPath> *readPaths, bool skipPatchableAvars) const
{
    return _RouteValueEdits(*_impl, notice, nullptr, readPaths, nullptr,
                            skipPatchableAvars);
}

bool
RigExecBakedProgram::ApplyValueEdits(const UsdNotice::ObjectsChanged &notice,
                                     bool skipPatchableAvars)
{
    RigExecBakedProgramImpl &B = *_impl;
    std::vector<int> indices;
    std::vector<SdfPath> read, leafOnly;
    if (!_RouteValueEdits(B, notice, &indices, &read, &leafOnly,
                          skipPatchableAvars)) {
        return false;
    }
    // The leaves the edit reached, re-read by the next sample: every leaf
    // filed under an edited path, and on a routed prim -- whose edits carry
    // no number -- every leaf with a path on that prim.
    const auto mark = [&B](const std::vector<uint32_t> &ids) {
        for (const uint32_t id : ids) {
            RigExecBakedMarkLeaf(&B, id);
        }
    };
    for (const SdfPath &path : read) {
        const auto found = B.leafByPath.find(path);
        if (found != B.leafByPath.end()) {
            mark(found->second);
            continue;
        }
        const SdfPath prim = path.GetPrimPath();
        if (!B.resolvedRoutedPrims.count(prim)) {
            continue;
        }
        for (auto it = B.leafByPath.lower_bound(prim);
             it != B.leafByPath.end() && it->first.HasPrefix(prim); ++it) {
            if (it->first.GetPrimPath() == prim) {
                mark(it->second);
            }
        }
    }
    for (const SdfPath &path : leafOnly) {
        mark(B.leafByPath.at(path));
    }
    if (indices.empty()) {
        return true;
    }
    B.edited.resize(B.overridden.size(), 0);
    B.editSerial.resize(B.overridden.size(), 0);
    ++B.valueEditSerial;
    for (const int index : indices) {
        B.edited[size_t(index)] = 1;
        B.editSerial[size_t(index)] = B.valueEditSerial;
    }
    B.anyEdited = true;
    return true;
}

namespace {

// One captured avar, moved to a new constant.
// Three copies of the value exist and all three move: the binding (what a
// drag's release writes back from), the constant table (what a rebuild would
// have captured), and the working table the frame reads. The working slot is
// left alone while an override stands on it -- the drag owns that slot, and
// the constant pass that runs when it is released writes this new constant
// back, which is exactly the value an authored edit under a drag should land
// on. Nothing is marked dirty: the frame path compares every avar slot with
// last run's by VALUE, which is the whole of what the cone needs.
// \p animated moves the binding the other way instead: the avar has become a
// function of time (Animation mode keys a released drag as a spline knot),
// so from now on the frame reads it the long way, exactly as a rebuild would
// have bound it. And an animated binding that is given a plain value again
// (the undo of that key) comes back to being a constant here.
void
RigExecProgramAvarPatch(RigExecBakedProgramImpl *B, size_t bindingIndex,
                        double value, bool animated)
{
    RigExecBakedProgramImpl::AvarBinding &binding =
        B->avarConstantBindings[bindingIndex];
    // The binding's leaf is re-read by the next sample, whichever way the
    // patch moves the binding below.
    if (binding.input.leaf >= 0) {
        B->leaves.Of<double>().mustSample[size_t(binding.input.leaf)] = 1;
    }
    const auto promoted = std::find(B->promotedAvars.begin(),
                                    B->promotedAvars.end(), bindingIndex);
    if (animated) {
        if (promoted == B->promotedAvars.end()) {
            binding.input.varying = true;
            binding.input.query = UsdAttributeQuery();
            binding.input.resolvedAttr = binding.input.head;
            B->promotedAvars.push_back(bindingIndex);
            ++B->varyingInputs;
        }
        // The frame's read decides the working value; it compares by value
        // against the last run like any other avar.
        return;
    }
    if (promoted != B->promotedAvars.end()) {
        binding.input.varying = false;
        binding.input.resolvedAttr = UsdAttribute();
        B->promotedAvars.erase(promoted);
        --B->varyingInputs;
    }
    binding.input.constant = value;
    B->avarConstants[binding.slot] = value;
    const int overrideIndex = binding.input.overrideIndex;
    const bool dragged = overrideIndex >= 0 &&
                         size_t(overrideIndex) < B->overridden.size() &&
                         B->overridden[size_t(overrideIndex)];
    if (!dragged) {
        B->avars[binding.slot] = value;
    }
}

}  // namespace

bool
RigExecBakedProgram::IsPatchableAvarPath(const SdfPath &path) const
{
    return _impl->patchableAvars.find(path) != _impl->patchableAvars.end();
}

namespace {

// One decided patch: the binding to write, the value to write, and the
// property path the notice named. Decided by _DryRunAvarPatches without
// writing anything, so the dry-run query and ApplyAvarValueEdits share
// every check by construction.
struct _AvarPatch {
    size_t binding = 0;
    double value = 0.0;
    bool animated = false;
    SdfPath path;
};

// Decide EVERYTHING before writing anything: a notice this cannot fully
// absorb must leave the table exactly as the rebuild path expects it.
bool
_DryRunAvarPatches(const RigExecBakedProgramImpl &B,
                   const UsdNotice::ObjectsChanged &notice,
                   std::vector<_AvarPatch> *patches)
{
    if (!notice.GetResolvedAssetPathsResyncedPaths().empty()) {
        return false;
    }
    static const TfToken kDefault("default");
    static const TfToken kTypeName("typeName");
    static const TfToken kSpline("spline");

    std::vector<_AvarPatch> decided;
    // A property resync is a spec appearing or going away under a value:
    // the first value a layer authors for an avar, and the undo that
    // removes it. Only the property itself may resync (a prim resync is
    // structure), and it may name only its type besides the value. What the
    // attribute composes to afterwards is checked below exactly as for a
    // plain value edit, so a spec that brings a connection or a spline, or
    // takes the last value away, still declines.
    std::vector<std::pair<SdfPath, bool>> edited;
    for (const SdfPath &path : notice.GetResyncedPaths()) {
        if (!path.IsPropertyPath()) {
            return false;
        }
        edited.emplace_back(path, true);
    }
    for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
        edited.emplace_back(path, false);
    }
    for (const auto &[path, resynced] : edited) {
        const bool captured = B.rebuild.count(path) ||
                              (path.IsPropertyPath() &&
                               B.xformPrims.count(path.GetPrimPath()));
        if (!captured) {
            // Prim-level info arrives for the ancestors of an edit (an
            // `over` being touched above it). It moves no value the frame
            // reads, and IsInvalidatedBy ignores it for the same reason.
            if (path.IsPrimPath()) {
                continue;
            }
            // A live input's value in the same edit -- an IK/FK blend keyed
            // with the channels it switches -- is not a patch: the caller
            // routes it (DryRunValueEdits) alongside these.
            if (_IsLiveInputResync(B, notice, path)) {
                continue;
            }
            // A PROPERTY the bake did not fold: a value the frame path
            // re-reads, or something on a prim it never looked at. That
            // still owes the program a stamp bump, which this entry point
            // promises the caller it does not need -- so it is declined.
            return false;
        }
        const auto found = B.patchableAvars.find(path);
        if (found == B.patchableAvars.end()) {
            return false;
        }
        for (const TfToken &field : notice.GetChangedFields(path)) {
            if (field != kDefault && field != kSpline &&
                !(resynced && field == kTypeName)) {
                return false;  // time samples, a connection...
            }
        }
        const RigExecBakedProgramImpl::AvarBinding &binding =
            B.avarConstantBindings[found->second];
        const UsdAttribute attribute = binding.input.head;
        if (!attribute) {
            return false;
        }
        // A new spec can restate the type, and the slot holds a double.
        if (attribute.GetTypeName() != SdfValueTypeNames->Double) {
            return false;
        }
        // A connection makes the value come from somewhere else, which only
        // a rebuild can walk to.
        SdfPathVector connections;
        if (attribute.HasAuthoredConnections() &&
            attribute.GetConnections(&connections) && !connections.empty()) {
            return false;
        }
        // Animated by a spline: promoted to a per-frame read. Time SAMPLES
        // still rebuild, because the bake reads their count for more than
        // this slot (a SingleChainIK's autoDetect mode is decided by whether
        // its chain's translates are keyed that way, and splines do not
        // count there).
        if (attribute.GetNumTimeSamples() > 0) {
            return false;
        }
        if (attribute.HasSpline()) {
            decided.push_back({found->second, 0.0, true, path});
            continue;
        }
        if (attribute.ValueMightBeTimeVarying()) {
            return false;  // varies some way this does not know
        }
        // Cleared back to nothing reads as the channel's default, which is
        // the fallback the bake captures for an attribute with no value.
        double value = RigExecBakedAvarDefaults[binding.slot % 11];
        attribute.Get(&value, UsdTimeCode::Default());
        decided.push_back({found->second, value, false, path});
    }
    if (decided.empty()) {
        return false;
    }
    if (patches) {
        *patches = std::move(decided);
    }
    return true;
}

}  // namespace

bool
RigExecBakedProgram::DryRunAvarValueEdits(
    const UsdNotice::ObjectsChanged &notice,
    std::vector<SdfPath> *patchedPaths) const
{
    std::vector<_AvarPatch> patches;
    if (!_DryRunAvarPatches(*_impl, notice, &patches)) {
        return false;
    }
    if (patchedPaths) {
        patchedPaths->clear();
        for (const _AvarPatch &patch : patches) {
            patchedPaths->push_back(patch.path);
        }
    }
    return true;
}

bool
RigExecBakedProgram::ApplyAvarValueEdits(
    const UsdNotice::ObjectsChanged &notice)
{
    RigExecBakedProgramImpl &B = *_impl;
    std::vector<_AvarPatch> patches;
    if (!_DryRunAvarPatches(B, notice, &patches)) {
        return false;
    }
    for (const _AvarPatch &patch : patches) {
        RigExecProgramAvarPatch(&B, patch.binding, patch.value,
                                patch.animated);
        // Every other leaf whose read reaches the avar (a shader dial, a
        // mover input connected to it) re-reads it too.
        const auto found = B.leafByPath.find(patch.path);
        if (found != B.leafByPath.end()) {
            for (const uint32_t id : found->second) {
                RigExecBakedMarkLeaf(&B, id);
            }
        }
    }
    return true;
}

void
RigExecBakedProgram::SetPublishWeightFields(bool publish)
{
    _impl->publishWeightFields = publish;
}

bool
RigExecBakedProgram::SetOverrides(
    const std::vector<RigExecValueOverride> &overrides)
{
    RigExecBakedProgramImpl &B = *_impl;
    B.routedOverrides.clear();
    if (overrides.empty()) {
        if (B.anyOverridden) {
            std::fill(B.overridden.begin(), B.overridden.end(), 0);
            B.anyOverridden = false;
        }
        return true;
    }
    if (B.anyOverridden) {
        std::fill(B.overridden.begin(), B.overridden.end(), 0);
        B.anyOverridden = false;
    }
    bool placeable = true;
    for (const RigExecValueOverride &o : overrides) {
        // A computation override names an exec computation, and there is no
        // exec here to hold it against.
        if (o.attribute.IsEmpty()) {
            placeable = false;
            continue;
        }
        const SdfPath path = o.prim.AppendProperty(o.attribute);
        // Folded first: a property can be both a per-frame input and the
        // source of something resolved once, and the once wins.
        if (B.folded.count(path)) {
            placeable = false;
            continue;
        }
        const auto found = B.overridableInputs.find(path);
        if (found != B.overridableInputs.end()) {
            for (int index : found->second) {
                B.overridden[size_t(index)] = 1;
            }
            B.anyOverridden = true;
            continue;
        }
        // Routed readers consult resolved inputs at every connection hop,
        // so an override on their source is placed by that same overlay.
        // It carries no number, so the leaf sampler compares its value.
        if (B.resolvedRoutedPrims.count(o.prim) ||
            _ConnectedSources(B).count(path)) {
            B.routedOverrides[path] = o.value;
            continue;
        }
        placeable = false;
    }
    return placeable;
}

namespace {

// RIGEXEC_BAKED_PROGRAM_DIGEST: a fingerprint of what Build produced.
// The parts of Build that resolve in parallel promise a program identical to
// a serial Build's, byte for byte -- the same override numbering, the same
// invalidation index, the same bound constants. Nothing in a frame's OUTPUT
// can check that promise: two numberings that differ still pose the rig the
// same, and a path missing from `named` shows up only on the edit that
// should have rebuilt. So the tables are hashed here, table by table, and a
// run with RIGEXEC_ENABLE_PARALLEL_EVAL=0 (or a build from before a change)
// must print the same lines. One line per table rather than one hash, so a
// mismatch names the table it is in.

bool
_ProgramDigestRequested()
{
    static const bool requested =
        TfGetenvBool("RIGEXEC_BAKED_PROGRAM_DIGEST", false);
    return requested;
}

// FNV-1a over the serialized bytes, plus a count of what was hashed.
struct _DigestTable {
    uint64_t hash = 1469598103934665603ull;
    size_t count = 0;

    void Bytes(const void *data, size_t size)
    {
        const unsigned char *p = static_cast<const unsigned char *>(data);
        for (size_t i = 0; i < size; ++i) {
            hash ^= p[i];
            hash *= 1099511628211ull;
        }
    }
    void Str(const std::string &s)
    {
        const uint64_t n = s.size();
        Bytes(&n, sizeof(n));
        Bytes(s.data(), s.size());
    }
    void Path(const SdfPath &p) { Str(p.GetString()); }
    void Int(int64_t v) { Bytes(&v, sizeof(v)); }
    void Double(double v) { Bytes(&v, sizeof(v)); }
    void Matrix(const GfMatrix4d &m)
    {
        Bytes(m.GetArray(), sizeof(double) * 16);
    }
    void Value(double v) { Double(v); }
    void Value(bool v) { Int(v ? 1 : 0); }
    void Value(const TfToken &v) { Str(v.GetString()); }
    void Value(const GfMatrix4d &v) { Matrix(v); }
    template <class T>
    void Input(const RigExecBakedInput<T> &input)
    {
        Value(input.constant);
        Path(input.query.IsValid() ? input.query.GetAttribute().GetPath()
                                   : SdfPath());
        Path(input.resolvedAttr ? input.resolvedAttr.GetPath() : SdfPath());
        Path(input.head ? input.head.GetPath() : SdfPath());
        Int(input.varying ? 1 : 0);
        Int(input.overrideIndex);
    }
    void PathSet(const std::set<SdfPath> &paths)
    {
        for (const SdfPath &p : paths) {
            Path(p);
        }
        count = paths.size();
    }
};

void
_PrintProgramDigest(const RigExecBakedProgramImpl &B)
{
    std::vector<std::pair<const char *, _DigestTable>> tables;
    const auto table = [&tables](const char *name) -> _DigestTable & {
        tables.emplace_back(name, _DigestTable());
        return tables.back().second;
    };
    table("rebuild").PathSet(B.rebuild);
    table("named").PathSet(B.named);
    table("prims").PathSet(B.prims);
    table("xformPrims").PathSet(B.xformPrims);
    table("folded").PathSet(B.folded);
    table("resolvedRoutedPrims").PathSet(B.resolvedRoutedPrims);
    {
        _DigestTable &t = table("overridden");
        for (char c : B.overridden) {
            t.Int(c);
        }
        t.count = B.overridden.size();
    }
    {
        _DigestTable &t = table("overridableInputs");
        for (const auto &[path, indices] : B.overridableInputs) {
            t.Path(path);
            t.Int(int64_t(indices.size()));
            for (int index : indices) {
                t.Int(index);
            }
        }
        t.count = B.overridableInputs.size();
    }
    {
        _DigestTable &t = table("inputCounts");
        t.Int(int64_t(B.boundInputs));
        t.Int(int64_t(B.varyingInputs));
        t.count = 2;
    }
    {
        _DigestTable &t = table("slots");
        for (size_t i = 0; i < B.paths.size(); ++i) {
            t.Path(B.paths[i]);
            t.Int(int64_t(B.slotKind[i]));
            t.Int(B.parent[i]);
            t.Int(B.propParent[i]);
        }
        for (int slot : B.xformSlots) {
            t.Int(slot);
        }
        t.count = B.paths.size();
    }
    {
        _DigestTable &t = table("ladders");
        for (const RigExecBakedProgramImpl::Ladder &ladder : B.ladders) {
            t.Input(ladder.posedSpace);
            t.Input(ladder.restSpace);
            t.Input(ladder.defaultSpace);
            t.Input(ladder.rotationOrder);
            for (int c = 0; c < 6; ++c) {
                t.Input(ladder.restAvars[c]);
                t.Input(ladder.defaultAvars[c]);
            }
        }
        t.Int(B.ladderVarying ? 1 : 0);
        for (int index : B.ladderOverrides) {
            t.Int(index);
        }
        for (char c : B.restChainVaries) {
            t.Int(c);
        }
        t.count = B.ladders.size();
    }
    {
        _DigestTable &t = table("ladderComposed");
        for (size_t i = 0; i < B.restM.size(); ++i) {
            t.Matrix(B.restM[i]);
            t.Matrix(B.selfD[i]);
            t.Matrix(B.parentDinv[i]);
            t.Matrix(B.posedAuthoredM[i]);
            t.Int(B.posedAuthored[i]);
            t.Str(B.rotOrder[i].GetString());
            t.Int(i < B.rotationSign.size() ? B.rotationSign[i] : 0);
        }
        t.count = B.restM.size();
    }
    {
        _DigestTable &t = table("avarTable");
        for (double v : B.avarConstants) {
            t.Double(v);
        }
        for (const auto *bindings :
             {&B.avarBindings, &B.avarConstantBindings}) {
            t.Int(int64_t(bindings->size()));
            for (const RigExecBakedProgramImpl::AvarBinding &b : *bindings) {
                t.Int(int64_t(b.slot));
                t.Input(b.input);
            }
        }
        for (const auto &[path, index] : B.patchableAvars) {
            t.Path(path);
            t.Int(int64_t(index));
        }
        t.count = B.avarConstants.size();
    }
    {
        // Per leaf, in id order: its pool, its owner's head and the paths
        // it is filed under.
        _DigestTable &t = table("leaves");
        std::vector<std::vector<const SdfPath *>> filed(B.leafRefs.size());
        for (const auto &[path, ids] : B.leafByPath) {
            for (const uint32_t id : ids) {
                if (id < filed.size()) {
                    filed[id].push_back(&path);
                }
            }
        }
        size_t id = 0;
        frozenDetail::_ForEachPatchableInput(B, [&](const auto &input) {
            const RigExecBakedLeafRef ref =
                id < B.leafRefs.size() ? B.leafRefs[id]
                                       : RigExecBakedLeafRef();
            t.Int(int64_t(ref.type));
            t.Int(int64_t(ref.index));
            t.Int(input.leaf);
            t.Path(input.head ? input.head.GetPath() : SdfPath());
            if (id < filed.size()) {
                t.Int(int64_t(filed[id].size()));
                for (const SdfPath *path : filed[id]) {
                    t.Path(*path);
                }
            }
            ++id;
        });
        t.count = B.leafRefs.size();
    }
    {
        // Per path leaf, in id order: its owner, its key and every path its
        // read can reach.
        _DigestTable &t = table("pathLeaves");
        for (const RigExecBakedPathLeafRef &ref : B.pathLeafRefs) {
            t.Int(int64_t(ref.owner));
            t.Int(int64_t(ref.a));
            t.Int(int64_t(ref.b));
            t.Int(int64_t(ref.key));
            const RigExecBakedPathLeaves *leaves =
                RigExecBakedPathLeavesOf(B, ref);
            if (!leaves || ref.key >= leaves->decl.keys.size()) {
                continue;
            }
            const RigExecRevisionLeafKey &key = leaves->decl.keys[ref.key];
            t.Path(key.path);
            t.Int(int64_t(key.type));
            t.Int(int64_t(key.time));
            t.Int(int64_t(key.flavour));
            if (ref.key < leaves->hops.size()) {
                t.Int(int64_t(leaves->hops[ref.key].size()));
                for (const SdfPath &hop : leaves->hops[ref.key]) {
                    t.Path(hop);
                }
            }
        }
        t.count = B.pathLeafRefs.size();
    }
    {
        // Per property chain: its target, arm and version base, each
        // revision's operation, weight object and walks (every hop with its
        // override slot, candidates and leaf), and its records.
        _DigestTable &t = table("propertyChains");
        const auto walk = [&t](const RigExecBakedWalk &w) {
            t.Int(int64_t(w.flavour));
            t.Int(int64_t(w.type));
            for (const std::vector<RigExecBakedWalkHop> *hops :
                 {&w.hops, &w.doubleHops}) {
                t.Int(int64_t(hops->size()));
                for (const RigExecBakedWalkHop &hop : *hops) {
                    t.Path(hop.path);
                    t.Int(hop.overrideSlot);
                    t.Int(hop.chain);
                    t.Int(hop.record);
                    t.Int(hop.leaf);
                }
            }
        };
        for (const RigExecBakedPropertyChain &chain : B.propertyChains) {
            t.Path(chain.target);
            t.Int(chain.targetExists ? 1 : 0);
            t.Int(int64_t(chain.arm));
            t.Int(chain.ownLeaf);
            t.Int(chain.targetSlot);
            t.Int(int64_t(chain.versionBase));
            for (const RigExecBakedPropertyChain::Revision &r :
                 chain.revisions) {
                t.Path(r.mover);
                t.Int(r.moverExists ? 1 : 0);
                t.Int(r.opValid ? int64_t(r.op) : -1);
                t.Path(r.weightObject);
                t.Int(r.hasTangents ? 1 : 0);
                for (const RigExecBakedWalk *w :
                     {&r.enabled, &r.defaultWeight, &r.value, &r.minimum,
                      &r.maximum, &r.keys, &r.tangents}) {
                    walk(*w);
                }
            }
            for (const uint32_t index : chain.records) {
                const RigExecBakedPropertyRecord &record =
                    B.propertyRecords[index];
                t.Path(record.consumer);
                t.Str(record.consumerType.GetAsToken().GetString());
                t.Int(int64_t(record.applied));
                t.Int(int64_t(record.id));
                for (const int slot : record.hopSlots) {
                    t.Int(slot);
                }
            }
        }
        for (const RigExecBakedHeadLeaf &leaf : B.headLeaves) {
            t.Path(leaf.path);
            t.Int(int64_t(leaf.type));
            t.Int(leaf.typeMatches ? 1 : 0);
            t.Path(leaf.frozenKey);
        }
        for (const auto &[path, slot] : B.headOverrideSlots) {
            t.Path(path);
            t.Int(int64_t(slot));
        }
        t.count = B.propertyChains.size();
    }
    {
        // The head steps in head order: kind, object, part, reads, writes,
        // what they always run on, and their predecessors.
        _DigestTable &t = table("headSteps");
        const auto ranges =
            [&t](const std::vector<RigExecBakedHeadRange> &list) {
                t.Int(int64_t(list.size()));
                for (const RigExecBakedHeadRange &range : list) {
                    t.Int(int64_t(range.domain));
                    t.Int(int64_t(range.begin));
                    t.Int(int64_t(range.end));
                }
            };
        for (const uint32_t index : B.headOrder) {
            const RigExecBakedHeadStep &step = B.headSteps[index];
            t.Int(int64_t(index));
            t.Int(int64_t(step.kind));
            t.Int(step.object);
            t.Int(step.part);
            ranges(step.reads);
            ranges(step.writes);
            t.Int(step.alwaysRuns ? 1 : 0);
            for (const uint32_t leaf : step.leaves) {
                t.Int(int64_t(leaf));
            }
            for (const uint32_t slot : step.overrideSlots) {
                t.Int(int64_t(slot));
            }
            for (const uint32_t leaf : step.bindingLeaves) {
                t.Int(int64_t(leaf));
            }
            t.Int(step.varyingLeaves ? 1 : 0);
            for (const auto &[version, record] : step.shadowedReads) {
                t.Int(int64_t(version));
                t.Int(int64_t(record));
            }
            for (const uint32_t pred : step.preds) {
                t.Int(int64_t(pred));
            }
            t.Str(step.label);
        }
        t.count = B.headSteps.size();
    }
    {
        // Per SkinTopology index: whether the revision's layout is fixed at
        // compile and at Build, and its layout leaves' paths (the leaves'
        // hops are in pathLeaves).
        _DigestTable &t = table("skinLayouts");
        const size_t count = B.revisionIndex.size() + B.derivedIndex.size();
        for (size_t r = 0; r < count; ++r) {
            const RigExecBakedProgramImpl::GeomRevision *revision =
                RigExecBakedLayoutRevision(B, r);
            t.Path(revision->moverPath);
            t.Int(revision->skinTopologyFixed ? 1 : 0);
            t.Int(revision->layoutFixed ? 1 : 0);
            for (const RigExecRevisionLeafKey &key :
                 revision->layoutLeaves.decl.keys) {
                t.Path(key.path);
                t.Int(int64_t(key.type));
                t.Int(int64_t(key.flavour));
            }
        }
        t.count = count;
    }
    {
        // Per chunked revision: the arrays Build cut its keys from, which
        // decide whether a frame adopts a layout handle.
        _DigestTable &t = table("partitions");
        size_t count = 0;
        for (const RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
            for (const RigExecBakedProgramImpl::GeomRevision &revision :
                 chain.revisions) {
                if (!revision.chunked) {
                    continue;
                }
                t.Path(revision.moverPath);
                t.Int(revision.partitionElementSize);
                t.Int(int64_t(revision.partitionIndices.size()));
                t.Bytes(revision.partitionIndices.cdata(),
                        sizeof(int) * revision.partitionIndices.size());
                ++count;
            }
        }
        t.count = count;
    }
    {
        // The reads after the head tier that resolve walks: each walk's head,
        // hops (slot, candidates, leaf), raw leaf, declared versions and
        // shadows; which path leaves and steps read them; what each step
        // and avar slot declares.
        _DigestTable &t = table("readerWalks");
        for (const RigExecBakedReaderWalk &reader : B.readerWalks) {
            t.Path(reader.head);
            t.Int(int64_t(reader.walk.flavour));
            t.Int(int64_t(reader.walk.type));
            for (const std::vector<RigExecBakedWalkHop> *hops :
                 {&reader.walk.hops, &reader.walk.doubleHops}) {
                t.Int(int64_t(hops->size()));
                for (const RigExecBakedWalkHop &hop : *hops) {
                    t.Path(hop.path);
                    t.Int(hop.overrideSlot);
                    t.Int(hop.chain);
                    t.Int(hop.record);
                    t.Int(hop.leaf);
                }
            }
            t.Int(reader.rawLeaf);
            for (const uint32_t id : reader.versions) {
                t.Int(int64_t(id));
            }
            for (const auto &[version, record] : reader.shadowed) {
                t.Int(int64_t(version));
                t.Int(int64_t(record));
            }
        }
        for (const RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
            for (const RigExecBakedProgramImpl::GeomRevision &revision :
                 chain.revisions) {
                for (const int walk : revision.leaves.walks) {
                    t.Int(walk);
                }
            }
            for (const RigExecBakedProgramImpl::GeomChain::Derived &derived :
                 chain.derived) {
                for (const int walk : derived.revision.leaves.walks) {
                    t.Int(walk);
                }
            }
        }
        frozenDetail::_ForEachPatchableInput(B, [&t](const auto &input) {
            t.Int(input.walk);
        });
        for (size_t i = 0; i < B.steps.size(); ++i) {
            const RigExecBakedStep &step = B.steps[i];
            if (step.readerWalks.empty()) {
                continue;
            }
            t.Int(int64_t(i));
            for (const int walk : step.readerWalks) {
                t.Int(walk);
            }
            for (const RigExecBakedHeadRange &range : step.headReads) {
                t.Int(int64_t(range.begin));
                t.Int(int64_t(range.end));
            }
            for (const auto &[version, record] : step.shadowedReads) {
                t.Int(int64_t(version));
                t.Int(int64_t(record));
            }
        }
        for (size_t slot = 0; slot < B.avarHeadReads.size(); ++slot) {
            for (const uint32_t id : B.avarHeadReads[slot]) {
                t.Int(int64_t(slot));
                t.Int(int64_t(id));
            }
        }
        t.count = B.readerWalks.size();
    }
    {
        _DigestTable &t = table("schedule");
        t.Str(RigExecBakedScheduleReport(B));
        t.count = B.steps.size();
    }
    _DigestTable total;
    std::string out;
    for (const auto &[name, t] : tables) {
        total.Str(name);
        total.Int(int64_t(t.count));
        total.Bytes(&t.hash, sizeof(t.hash));
        out += TfStringPrintf(
            "RIGEXEC_BAKED_PROGRAM_DIGEST %-22s n=%-6zu %016llx\n", name,
            t.count, static_cast<unsigned long long>(t.hash));
    }
    out += TfStringPrintf(
        "RIGEXEC_BAKED_PROGRAM_DIGEST %-22s        %016llx\n", "total",
        static_cast<unsigned long long>(total.hash));
    std::fwrite(out.data(), 1, out.size(), stderr);
}

}  // namespace

// Build.

void
RigExecBakedBuildContext::Refuse(const std::string &what, const SdfPath &where)
{
    if (reasons) reasons->push_back(what + ": " + where.GetString());
    ok = false;
}

void
RigExecBakedBuildContext::Fold(const UsdPrim &prim, const char *name)
{
    RigExecBakedProgramSink sink{program};
    RigExecBakedRecordFold(&sink, prim, name);
}

void
RigExecBakedBuildContext::Fold(const UsdPrim &prim, const TfToken &name)
{
    RigExecBakedProgramSink sink{program};
    RigExecBakedRecordFold(&sink, prim, name);
}

void
RigExecBakedCommitShard::Seal()
{
    for (SdfPathVector *paths : {&prims, &named, &rebuild, &folded}) {
        std::sort(paths->begin(), paths->end());
        paths->erase(std::unique(paths->begin(), paths->end()),
                     paths->end());
    }
    // Not deduplicated: one input names each walk step once (the walk
    // stops at the first attribute it revisits), so an equal pair is two
    // inputs and both numbers belong in the entry.
    std::sort(overridable.begin(), overridable.end());
}

std::vector<int>
RigExecBakedMergeCommitShards(RigExecBakedProgramImpl *program,
                              std::vector<RigExecBakedCommitShard> *shards)
{
    RigExecBakedProgramImpl &B = *program;
    std::vector<int> first(shards->size(), 0);
    int next = int(B.overridden.size());
    for (size_t k = 0; k < shards->size(); ++k) {
        first[k] = next;
        next += (*shards)[k].numbered;
        B.boundInputs += (*shards)[k].boundInputs;
        B.varyingInputs += (*shards)[k].varyingInputs;
    }
    // Every flag starts clear, so growing the vector is the same as the
    // serial commit's one push_back per number.
    B.overridden.resize(size_t(next), 0);
    // Each shard's paths are sorted, so each insert is hinted at the
    // successor of the one before it -- constant time wherever the set
    // holds nothing in between, a plain insert wherever it does. The hint
    // only decides the cost, never the contents.
    const auto insertSorted = [](std::set<SdfPath> *set,
                                 const SdfPathVector &paths) {
        auto hint = set->end();
        for (const SdfPath &path : paths) {
            hint = std::next(set->insert(hint, path));
        }
    };
    for (RigExecBakedCommitShard &shard : *shards) {
        insertSorted(&B.prims, shard.prims);
        insertSorted(&B.named, shard.named);
        insertSorted(&B.rebuild, shard.rebuild);
        insertSorted(&B.folded, shard.folded);
    }
    // In chunk order and, inside a shard, by (path, number): every number
    // shard k hands out is below every number shard k+1 does, and every
    // number the phase hands out is above whatever an earlier phase put in
    // an entry, so each entry grows in exactly the ascending order the
    // serial commit appends in.
    for (size_t k = 0; k < shards->size(); ++k) {
        auto hint = B.overridableInputs.end();
        for (const auto &[path, relative] : (*shards)[k].overridable) {
            const auto it = B.overridableInputs.try_emplace(hint, path);
            it->second.push_back(first[k] + relative);
            hint = std::next(it);
        }
    }
    return first;
}

void
RigExecBakedBuildContext::FoldShape(const UsdPrim &prim, const char *name)
{
    if (prim) {
        const SdfPath path = prim.GetPath().AppendProperty(TfToken(name));
        program->rebuild.insert(path);
        program->named.insert(path);
        program->prims.insert(prim.GetPath());
    }
}

TfToken
RigExecBakedBuildContext::ReadToken(const UsdPrim &prim, const char *name,
                                    const char *fallback)
{
    Fold(prim, name);
    return _ReadToken(prim, name, fallback);
}

SdfPathVector
RigExecBakedBuildContext::Targets(const UsdPrim &prim, const char *name)
{
    Fold(prim, name);
    return _Targets(prim, name);
}

int
RigExecBakedBuildContext::SlotOf(const SdfPath &path) const
{
    const auto it = program->index.find(path);
    return it == program->index.end() ? -1 : it->second;
}

std::unique_ptr<RigExecBakedProgram>
RigExecBakedProgram::Build(RigExecRigEvaluator *evaluator,
                           std::vector<std::string> *reasons)
{
    if (!evaluator) {
        return nullptr;
    }
    RigExecRigEvaluator &E = *evaluator;
    // Bound here, ahead of IsBakeable, so the profiler exists before the
    // refusal walk runs. A rig that refuses still costs whatever
    // IsBakeable spends deciding why, and with `phases` opened only after
    // that call the cost had nowhere to attribute to -- it showed up as a
    // gap between Compile.Bake opening and the first Bake.* mark. Marking
    // it here costs the one branch per mark every other phase already pays.
    RigExecProfilePhases phases(&E._profiler, "compile");
    // The bakeability walk runs as a task beside the start of the bake --
    // the entry block, the dense provider slots and the ladder's parallel
    // resolve -- and is joined before the first thing that COMMITS anything
    // the refusal would have prevented: the ladder commit, which hands out
    // override indices. It can, because it reads only the evaluator's
    // compiled tables and the stage, and so do those three: none of them
    // writes anything but the program under construction and `ctx`, which
    // are discarded whole on a refusal.
    // The two write their refusals to separate vectors, merged in program
    // order at the join. `ctx.reasons` is what `refuse()` appends to, and the
    // dense slots refuse; sharing one vector with the walk would interleave
    // the two lists and race. On a refusal the walk's reasons alone are
    // handed back, which is what running it first used to return.
    // Inline where a task is not allowed or not worth it: with parallel
    // evaluation off, inside a frozen run, and on an evaluator that has not
    // compiled, whose tables the slots below have no business reading.
    const bool bakeableBeside = E._compiled &&
                                RigExecParallelEvaluationEnabled() &&
                                !RigExecFrozenSerialActive();
    bool bakeable = true;
    std::vector<std::string> bakeableReasons;
    std::vector<std::string> buildReasons;
    // Declared after everything the task writes, so an exit from anywhere
    // below destroys the dispatcher, which joins it, before those die.
    WorkDispatcher bakeableLane;
    if (bakeableBeside) {
        bakeableLane.Run([&E, &bakeable, &bakeableReasons] {
            RigExecProfileScope scope(&E._profiler, "Bake.IsBakeable",
                                      "compile");
            bakeable = IsBakeable(E, &bakeableReasons);
        });
    } else {
        phases.Next("Bake.IsBakeable");
        if (!IsBakeable(E, reasons)) {
            return nullptr;
        }
    }
    auto impl = std::make_unique<RigExecBakedProgramImpl>();
    RigExecBakedProgramImpl &B = *impl;
    B.evaluator = evaluator;
    B.stage = E._stage;
    // What the bodies would otherwise read from the environment or a
    // function-local static, read once here (bodyPurity.h).
    B.chunkVertexTarget = RigExecBakedChunkVertexTargetFromEnvironment();
    B.chunkCap = RigExecBakedChunkCapFromEnvironment();
    B.useSimd = RigExecSimdEnabled();
    B.purityAudit = TfGetenvBool("RIGEXEC_PURITY_AUDIT", false);
    RigExecMoverGraphTouchTokens();
    RigExecWeightPacketsTouchTokens();
    RigExecBakedGeometryTouchTokens();
    RigExecBakedWeightsTouchTokens();
    RigExecFrozenGeometryTouchTokens();
    B.assetRootPath = E._rigPath.GetParentPath();
    // The evaluator state a frame reads, captured here because this is the
    // only translation unit its friendship reaches; bakedProgramImpl.h says
    // why each one is a pointer rather than a copy.
    B.resolvedInputs = &E._resolvedInputs;
    B.chainSnapshots = &E._chainSnapshots;
    B.blendSampleShapes = &E._blendSampleShapes;
    B.profiler = &E._profiler;
    // Where Build spends its time, part by part. The parts run one
    // after another in this block rather than nested, so they are
    // marked rather than scoped; see RigExecProfilePhases.
    phases.Next("Bake.entry");
    B.interactiveOverrides = &E._interactiveOverrides;
    B.jointSolverBinding = &E._jointSolverBinding;
    B.guideTaps = &E._guideTaps;
    B.solverGuidesEnabled = &E._solverGuidesEnabled;
    B.hasPropertyChains = !E._propertyChains.empty();
    // Every volumetric weight's baked falloff remap, copied rather than
    // recomputed: these are the same bytes the authoritative snapshot hands
    // exec, and resampling the curve again here would risk a different
    // answer for a spline edited between Compile and Build.
    B.currentPhaseWeights = E._currentPhaseWeights;
    // The oracle a constraint's envelope resolves through, bound here
    // because this is the only translation unit the evaluator's friendship
    // reaches. `evaluator` outlives the program -- the evaluator owns it and
    // drops it on any epoch change -- so capturing the pointer is safe in
    // exactly the way every other capture in this block is; `program` is
    // the impl that stores this function, and a frozen clone drops it.
    // A volume is placed from the program's own table, never the
    // evaluator's map, which only the dynamic walk refreshes. Every caller
    // shares this one function, so none can reach that map by accident.
    B.resolveWeights = [evaluator, program = &B](
                           const SdfPath &weightPath, size_t count,
                           UsdTimeCode time, std::vector<float> *weights,
                           std::string *error,
                           const std::vector<GfVec3f> *current) {
        // placedVolumes, not noScaleAvars: the oracle sees exactly the
        // volumes the walk places, so an untapped volume stays a miss.
        RigExecVolumePlacementView placements;
        placements.index = &program->index;
        placements.placed = program->placedVolumes.data();
        placements.placements = program->volumePlacement.data();
        placements.count = std::min(program->placedVolumes.size(),
                                    program->volumePlacement.size());
        return evaluator->_ResolveWeights(weightPath, count, time, weights,
                                          error, current, &placements);
    };
    // The reader a sparse blend sample shape resolves through, on a cache
    // miss: the shared resolver, so the program and the dynamic walk admit
    // and refuse exactly the same shapes.
    B.resolveBlendSample = [stage = B.stage](
                                 const SdfPath &blendShape,
                                 size_t pointCount,
                                 RigExecBlendSampleLayout *layout) {
        return RigExecResolveBlendSampleLayout(stage, blendShape, pointCount,
                                               layout);
    };
    for (const RigExecValueOverride &override : E._falloffLutOverrides) {
        if (override.value.IsHolding<RigExecFalloffLut>()) {
            B.falloffLuts[override.prim] =
                override.value.UncheckedGet<RigExecFalloffLut>().samples;
        }
    }

    RigExecBakedBuildContext ctx;
    ctx.program = &B;
    ctx.stage = B.stage;
    ctx.capture = UsdTimeCode::Default();
    ctx.probe = _ProbeTime(B.stage);
    // Until the bakeability task is joined, this build's own refusals go to
    // a vector of their own; see the head of Build.
    ctx.reasons = bakeableBeside && reasons ? &buildReasons : reasons;
    // Every property a chain writes. An input resolving through one of these
    // cannot be captured, because the chain recomputes it every generation.
    for (const auto &[target, revisions] : E._propertyChains) {
        ctx.chainTargets.insert(target);
    }
    B.chainTargets = ctx.chainTargets;
    // Each ribbon's driver-points attribute, resolved once by the compiler.
    // Restated here because bakedPose.cpp cannot name the evaluator's
    // private map, and re-deriving the resolution (a prim target becomes its
    // .points, a property target is taken verbatim) would be a second
    // expression of a rule the compiler already applied.
    ctx.ribbonDriverPoints = E._ribbonDriverPoints;
    // _solverDependencies holds every discovered solver as a key, which is
    // the test "is an aggregate solver" uses below too.
    for (const auto &[solverPath, dependencies] : E._solverDependencies) {
        ctx.aggregateSolvers.insert(ctx.aggregateSolvers.end(), solverPath);
    }
    const UsdTimeCode capture = ctx.capture;
    const std::set<SdfPath> &chainTargets = ctx.chainTargets;
    // The bodies below were written against these as lambdas of this
    // function, and the bake functions in the other two files were split out
    // of the same bodies; naming them keeps both sides reading alike.
    auto refuse = [&](const std::string &what, const SdfPath &where) {
        ctx.Refuse(what, where);
    };
    auto slotOf = [&](const SdfPath &path) { return ctx.SlotOf(path); };

    // Stage metadata, which arrives as a changed-info notice on the
    // pseudo-root: the up axis an aim constraint resolves against, and the
    // time-code range the input classification probes at.
    B.rebuild.insert(SdfPath::AbsoluteRootPath());

    // What a plain Xformable's transform is measured relative to, which is
    // the one prim rigEvaluator.cpp's pose walk measures against too.
    B.assetRoot = B.stage->GetPrimAtPath(B.assetRootPath);

    phases.Next("Bake.dense_provider_slots");
    // The ordered union of the two families, which is the set the dynamic
    // walk's frame maps hold: the exec-seeded providers and the plain
    // Xformables a constraint targets. Both are already in SdfPath order, so
    // merging them through one ordered map keeps namespace DFS pre-order and
    // with it the "a parent has a lower slot" invariant the compose relies
    // on. The two sets are disjoint by construction (only what isFrameProvider
    // accepts is seeded); FirstFramePose wins a collision, because that is the kind
    // the compose can actually write.
    {
        const TfToken rotationSignName("avars:rotationSign");
        std::map<SdfPath, RigExecBakedSlotKind> ordered;
        for (const auto &[path, tap] : E._firstFramePoseFrames) {
            ordered.emplace(path, RigExecBakedSlotKind::FirstFramePose);
        }
        for (const SdfPath &path : E._xformDerivedProviders) {
            ordered.emplace(path, RigExecBakedSlotKind::XformDerived);
        }
        for (const auto &[path, kind] : ordered) {
            B.index[path] = int(B.paths.size());
            B.paths.push_back(path);
            B.slotKind.push_back(kind);
            // Read here, with the type already in hand, rather than in the
            // compose: a volume weight's placement is rigid, and the
            // transform-scale avars it would otherwise compose with are
            // exactly the ones exec never binds.
            const UsdPrim prim = B.stage->GetPrimAtPath(path);
            B.noScaleAvars.push_back(
                prim && _IsVolumeWeightTypeName(prim.GetTypeName()) ? 1 : 0);
            // avars:rotationSign is rig structure, not animation: it is read
            // once here beside the type, and the compose applies it to
            // whatever the avar inputs carry that frame. Folded, so an edit
            // rebuilds the program and a drag on it falls back.
            GfVec3d sign(1, 1, 1);
            if (prim) {
                ctx.Fold(prim, rotationSignName);
                if (const UsdAttribute attr =
                        prim.GetAttribute(rotationSignName)) {
                    attr.Get(&sign);
                }
            }
            B.rotationSign.push_back(
                RigExecRotationSignMask(sign[0], sign[1], sign[2]));
        }
    }
    const int N = int(B.paths.size());
    B.parent.assign(N, -1);
    B.propParent.assign(N, -1);
    for (int i = 0; i < N; ++i) {
        // One climb fills both: the first slot found is the propagation
        // parent whatever its kind, and the climb continues past an
        // xform-derived one because the compose ladder can only inherit from
        // a provider exec composed.
        for (SdfPath p = B.paths[i].GetParentPath();
             !p.IsEmpty() && p != SdfPath::AbsoluteRootPath();
             p = p.GetParentPath()) {
            const auto it = B.index.find(p);
            if (it == B.index.end()) {
                continue;
            }
            if (B.propParent[i] < 0) {
                B.propParent[i] = it->second;
            }
            if (B.slotKind[size_t(it->second)] ==
                RigExecBakedSlotKind::FirstFramePose) {
                B.parent[i] = it->second;
                break;
            }
        }
        if (B.parent[i] >= i) {
            refuse("provider slots are not in namespace DFS order",
                   B.paths[i]);
        }
    }

    phases.Next("Bake.the_rest_chain_and_the_default-space_ladder");
    // computations.cpp resolves both per provider per evaluation through
    // eight computations. Every channel feeding them is BOUND here, not
    // folded: an ordinary rig binds fifteen constants per provider and the
    // frame path never reads one again, while a rig that animates, connects
    // or chain-writes a rest keeps the same fifteen bindings and recomposes
    // the ladder in the prologue. One resolution either way --
    // RigExecBakedComposeRestRange and RigExecBakedComposeLadderRange,
    // called once below over every slot and per compose group by the rest
    // and ladder head ops -- so the two cannot drift.
    B.xyzToken = TfToken("XYZ");
    B.posedAuthored.assign(size_t(N), 0);
    B.posedAuthoredM.assign(size_t(N), GfMatrix4d(1.0));
    B.restM.assign(N, GfMatrix4d(1.0));
    B.restPts.resize(N);
    B.restFrames.resize(N);
    B.selfD.assign(N, GfMatrix4d(1.0));
    B.parentDinv.assign(N, GfMatrix4d(1.0));
    B.rotOrder.assign(N, B.xyzToken);
    B.restRoundTrip.assign(N, GfMatrix4d(1.0));
    B.defaultRoundTrip.assign(N, GfMatrix4d(1.0));
    B.ladders.resize(size_t(N));
    static const char *const kRestAvars[6] = {
        "rest:tx", "rest:ty", "rest:tz",
        "rest:rx", "rest:ry", "rest:rz"};
    static const char *const kDefaultAvars[6] = {
        "default:tx", "default:ty", "default:tz",
        "default:rx", "default:ry", "default:rz"};
    // Fifteen composed-stage resolutions per provider slot, and on a
    // character this loop is the other of the two places Build spends most
    // of its time -- see the input binding table below, which is the same
    // shape of problem at eleven channels instead of fifteen and was split
    // the same way. ResolveBind only READS the stage and `chainTargets`, so
    // the fifteen calls for one slot don't depend on any other slot's, and
    // the resolve half moves to a parallel pass. The three space bindings
    // are matrices, the rotation order a token and the twelve avars
    // doubles, so ResolveBind's typed result is kept typed rather than
    // forced into one array -- each paired with its walk, same as
    // _AvarResolution.
    // The commit's order-free half runs in the same pass: each chunk of
    // slots records its folds and bindings into a shard of its own (see
    // RigExecBakedRecordBind), in the slot-then-channel order the serial
    // commit used, and the shards are merged in chunk order after the
    // bakeability join. Recording before the join is safe because a shard
    // is scratch -- nothing reaches the program until the merge, and a
    // refused rig never merges.
    RigExecProfilePhases bindPhases(&E._profiler, "compile");
    bindPhases.Next("Bake.ladder.resolve");
    struct _LadderResolution {
        RigExecBakedInput<GfMatrix4d> posedSpace;
        SdfPathVector posedSpaceWalk;
        RigExecBakedInput<GfMatrix4d> restSpace;
        SdfPathVector restSpaceWalk;
        RigExecBakedInput<GfMatrix4d> defaultSpace;
        SdfPathVector defaultSpaceWalk;
        RigExecBakedInput<TfToken> rotationOrder;
        SdfPathVector rotationOrderWalk;
        RigExecBakedInput<double> restAvars[6];
        SdfPathVector restAvarWalks[6];
        RigExecBakedInput<double> defaultAvars[6];
        SdfPathVector defaultAvarWalks[6];
    };
    std::vector<_LadderResolution> ladderResolved(static_cast<size_t>(N));
    // A chunk is a run of consecutive slots. Its size decides only how the
    // work is cut, never what the merge produces, so it is tuned for cost
    // alone: large enough that a shard's sort has something to amortize,
    // small enough to spread a character's few hundred slots over the pool.
    const size_t commitChunkSlots = 8;
    const size_t commitChunks =
        (size_t(N) + commitChunkSlots - 1) / commitChunkSlots;
    std::vector<RigExecBakedCommitShard> ladderShards(commitChunks);
    const auto resolveLadderChunks = [&](size_t beginChunk,
                                         size_t endChunk) {
        for (size_t k = beginChunk; k < endChunk; ++k) {
            RigExecBakedCommitShard &shard = ladderShards[k];
            const size_t end =
                std::min(size_t(N), (k + 1) * commitChunkSlots);
            for (size_t i = k * commitChunkSlots; i < end; ++i) {
                // An xform-derived slot binds no ladder -- its rest frame
                // is the identity outright, below -- so there is nothing
                // here for it to resolve.
                if (B.slotKind[i] != RigExecBakedSlotKind::FirstFramePose) {
                    continue;
                }
                const UsdPrim prim = B.stage->GetPrimAtPath(B.paths[i]);
                _LadderResolution &out = ladderResolved[i];
                out.posedSpace = ctx.ResolveBind(prim, "posed:space",
                                                 GfMatrix4d(1.0),
                                                 &out.posedSpaceWalk);
                out.restSpace = ctx.ResolveBind(prim, "rest:space",
                                                GfMatrix4d(1.0),
                                                &out.restSpaceWalk);
                out.defaultSpace = ctx.ResolveBind(prim, "default:space",
                                                   GfMatrix4d(1.0),
                                                   &out.defaultSpaceWalk);
                out.rotationOrder = ctx.ResolveBind(
                    prim, "avars:rotationOrder", TfToken("XYZ"),
                    &out.rotationOrderWalk);
                for (int c = 0; c < 6; ++c) {
                    out.restAvars[c] = ctx.ResolveBind(
                        prim, kRestAvars[c], 0.0, &out.restAvarWalks[c]);
                    out.defaultAvars[c] = ctx.ResolveBind(
                        prim, kDefaultAvars[c], 0.0,
                        &out.defaultAvarWalks[c]);
                }
                shard.Prim(B.paths[i]);
                // The space EXPRESSIONS stay FOLDED. Bakeability accepted
                // them because they are unauthored and unconnected, which
                // is a statement about the SHAPE of the compose --
                // authoring one replaces the ladder rather than moving a
                // value in it -- so an edit to one must rebuild the program
                // and a drag on one must fall back.
                for (const char *name : {"parent:space", "parent:defaultSpace",
                                         "avars:defaultSpace",
                                         "posed:defaultSpace"}) {
                    RigExecBakedRecordFold(&shard, prim, name);
                }
                // Recorded in the posedSpace/restSpace/defaultSpace/
                // rotationOrder/avar sequence the serial commit bound them
                // in: the shard numbers in recording order, so this is the
                // order the override indices come out in.
                RigExecBakedRecordBind(&shard, prim, "posed:space",
                                       &out.posedSpace, out.posedSpaceWalk);
                RigExecBakedRecordBind(&shard, prim, "rest:space",
                                       &out.restSpace, out.restSpaceWalk);
                RigExecBakedRecordBind(&shard, prim, "default:space",
                                       &out.defaultSpace,
                                       out.defaultSpaceWalk);
                RigExecBakedRecordBind(&shard, prim, "avars:rotationOrder",
                                       &out.rotationOrder,
                                       out.rotationOrderWalk);
                for (int c = 0; c < 6; ++c) {
                    RigExecBakedRecordBind(&shard, prim, kRestAvars[c],
                                           &out.restAvars[c],
                                           out.restAvarWalks[c]);
                    RigExecBakedRecordBind(&shard, prim, kDefaultAvars[c],
                                           &out.defaultAvars[c],
                                           out.defaultAvarWalks[c]);
                }
            }
            shard.Seal();
        }
    };
    if (RigExecParallelEvaluationEnabled() && !RigExecFrozenSerialActive() &&
        commitChunks > 1) {
        WorkParallelForN(commitChunks, resolveLadderChunks);
    } else {
        resolveLadderChunks(0, commitChunks);
    }
    // The bakeability join, here because everything above only read, and
    // the commit below is the first step a refused rig must not take (see
    // the head of Build).
    if (bakeableBeside) {
        {
            RigExecProfileScope join(&E._profiler, "Bake.IsBakeableJoin",
                                     "compile");
            bakeableLane.Wait();
        }
        if (!bakeable) {
            if (reasons) {
                reasons->insert(reasons->end(), bakeableReasons.begin(),
                                bakeableReasons.end());
            }
            return nullptr;
        }
        // The walk passed, so it added nothing; what the slots refused
        // follows it, in the order a serial Build appends them.
        if (reasons) {
            reasons->insert(reasons->end(), buildReasons.begin(),
                            buildReasons.end());
        }
        ctx.reasons = reasons;
    }
    bindPhases.Next("Bake.ladder.commit");
    // The order-bearing half, serial: the merge hands each chunk its first
    // override number, which every input the chunk numbered is moved up by.
    const std::vector<int> ladderFirst =
        RigExecBakedMergeCommitShards(&B, &ladderShards);
    const auto renumber = [](auto *input, int first) {
        if (input->overrideIndex >= 0) {
            input->overrideIndex += first;
        }
    };
    for (int i = 0; i < N; ++i) {
        if (B.slotKind[size_t(i)] != RigExecBakedSlotKind::FirstFramePose) {
            // An xform-derived slot has no rest chain and no default-space
            // ladder: the dynamic path gives it the identity rest frame
            // outright and reads its pose off the stage.
            // SI-6, invalidation-index coverage: the prim and every ancestor
            // whose transform the prologue composes go into B.prims, so a
            // RESYNC that retypes or reparents one rebuilds the program. They
            // go nowhere else on purpose. Nothing about their VALUE is
            // captured -- the prologue re-reads the whole ladder every frame
            // -- so a changed-info notice on one must not rebuild, or an
            // animated Xform above a constraint target would rebuild the
            // program once per frame.
            B.restFrames[i] = RigExecFrameFromMatrix(GfMatrix4d(1.0));
            B.restPts[i] = B.restFrames[i].points;
            B.xformSlots.push_back(i);
            B.xformPrimsBySlot.push_back(B.stage->GetPrimAtPath(B.paths[i]));
            for (SdfPath p = B.paths[i];
                 !p.IsEmpty() && p != SdfPath::AbsoluteRootPath();
                 p = p.GetParentPath()) {
                B.prims.insert(p);
                if (p == B.assetRootPath) {
                    break;
                }
            }
            continue;
        }
        _LadderResolution &out = ladderResolved[size_t(i)];
        const int first = ladderFirst[size_t(i) / commitChunkSlots];
        RigExecBakedProgramImpl::Ladder &ladder = B.ladders[size_t(i)];
        renumber(&out.posedSpace, first);
        ladder.posedSpace = std::move(out.posedSpace);
        renumber(&out.restSpace, first);
        ladder.restSpace = std::move(out.restSpace);
        renumber(&out.defaultSpace, first);
        ladder.defaultSpace = std::move(out.defaultSpace);
        renumber(&out.rotationOrder, first);
        ladder.rotationOrder = std::move(out.rotationOrder);
        for (int c = 0; c < 6; ++c) {
            renumber(&out.restAvars[c], first);
            ladder.restAvars[c] = std::move(out.restAvars[c]);
            renumber(&out.defaultAvars[c], first);
            ladder.defaultAvars[c] = std::move(out.defaultAvars[c]);
        }
    }
    // What a frame may have to re-resolve, and where a drag on one lands.
    // Both are asked of the WHOLE ladder rather than per provider: exec
    // re-pulls every rest the moment any one of them can move (rigEvaluator
    // .cpp's newRestsMightVary), and a chain's answer depends on its
    // ancestors in any case.
    bindPhases.Next("Bake.ladder.varies_and_compose");
    const auto noteLadderInput = [&B](const auto &input) {
        if (input.varying) {
            B.ladderVarying = true;
        }
        if (input.overrideIndex >= 0) {
            B.ladderOverrides.push_back(input.overrideIndex);
        }
    };
    for (int i = 0; i < N; ++i) {
        if (B.slotKind[size_t(i)] != RigExecBakedSlotKind::FirstFramePose) {
            continue;
        }
        const RigExecBakedProgramImpl::Ladder &ladder = B.ladders[size_t(i)];
        noteLadderInput(ladder.posedSpace);
        noteLadderInput(ladder.restSpace);
        noteLadderInput(ladder.defaultSpace);
        noteLadderInput(ladder.rotationOrder);
        for (int c = 0; c < 6; ++c) {
            noteLadderInput(ladder.restAvars[c]);
            noteLadderInput(ladder.defaultAvars[c]);
        }
    }
    // Per slot, whether the rest chain reaching it can move. A rest is its
    // ancestors' rests as well, so this propagates down the chain -- slots
    // are in namespace DFS pre-order and a parent's slot is always lower,
    // so one forward pass settles it.
    B.restChainVaries.assign(size_t(N), 0);
    for (int i = 0; i < N; ++i) {
        if (B.slotKind[size_t(i)] != RigExecBakedSlotKind::FirstFramePose) {
            continue;
        }
        const RigExecBakedProgramImpl::Ladder &ladder = B.ladders[size_t(i)];
        bool varies = ladder.restSpace.varying;
        for (int c = 0; c < 6; ++c) {
            varies = varies || ladder.restAvars[c].varying;
        }
        if (B.parent[size_t(i)] >= 0 &&
            B.restChainVaries[size_t(B.parent[size_t(i)])]) {
            varies = true;
        }
        B.restChainVaries[size_t(i)] = varies ? 1 : 0;
    }
    std::sort(B.ladderOverrides.begin(), B.ladderOverrides.end());
    B.ladderOverrides.erase(
        std::unique(B.ladderOverrides.begin(), B.ladderOverrides.end()),
        B.ladderOverrides.end());
    // And the ladder itself, at the capture time, through the same function
    // the prologue calls. Nothing is captured that the frame path cannot
    // re-resolve, which is what makes the two agree by construction. It
    // reads leaves, as the prologue does, so the ones bound so far are
    // numbered and sampled first; the end of Build numbers them all again.
    RigExecBakedNumberLeaves(&B);
    RigExecBakedSampleLeaves(&B, capture, /* all = */ true);
    // Every rest before any ladder: a ladder reads its own slot's rest and
    // its parent's, and a slot's parent is below it.
    RigExecBakedComposeRestRange(&B, 0, N, /* trackMoves = */ false);
    RigExecBakedComposeLadderRange(&B, 0, N, /* trackMoves = */ false);
    bindPhases.Close();
    // The comparison buffers the rest and ladder ops compare against,
    // seeded with what Build just composed: a later compose that lands on
    // the same numbers finds nothing moved.
    B.lastRestM = B.restM;
    B.lastSelfD = B.selfD;
    B.lastParentDinv = B.parentDinv;
    B.lastPosedAuthored = B.posedAuthored;
    B.lastPosedAuthoredM = B.posedAuthoredM;
    B.lastRotOrder = B.rotOrder;

    // The seed's two comparison buffers, sized once and never resized in a
    // run. Their first-run values are never read: the first run of a program
    // dirties every pose cluster outright, because it has nothing to compare
    // against.
    B.xformBase.assign(B.xformSlots.size(), GfMatrix4d(1.0));
    B.lastXformBase.assign(B.xformSlots.size(), GfMatrix4d(1.0));

    phases.Next("Bake.the_input_binding_table");
    B.avarConstants.assign(size_t(N) * 11, 0.0);
    // Eleven channels on every provider, each an independent read of the
    // composed stage, and on a character this is one of the two loops Build
    // spends most of its time in. Resolved across threads, and the commit's
    // order-free half recorded there too, into one shard per chunk of slots
    // exactly as the ladder above does; what stays on this thread is what
    // only the program order can decide -- the override numbers, by the
    // merge's prefix sum, and the positions in the two binding lists.
    struct _AvarResolution {
        RigExecBakedInput<double> input;
        SdfPathVector walk;
        /// The avar's own property, and whether a registered constant was
        /// captured from it and nowhere else -- decided in the parallel
        /// pass, which already holds the prim, so the serial one only
        /// files it.
        SdfPath property;
        bool patchable = false;
    };
    bindPhases.Next("Bake.input_table.resolve");
    std::vector<_AvarResolution> resolved(size_t(N) * 11);
    std::vector<RigExecBakedCommitShard> avarShards(commitChunks);
    const auto resolveChunks = [&](size_t beginChunk, size_t endChunk) {
        for (size_t k = beginChunk; k < endChunk; ++k) {
            RigExecBakedCommitShard &shard = avarShards[k];
            const size_t end =
                std::min(size_t(N), (k + 1) * commitChunkSlots);
            for (size_t i = k * commitChunkSlots; i < end; ++i) {
                // An xform-derived slot binds no avars -- its pose is the
                // stage's -- so the bound/varying counts and the
                // overridable set stay exactly what the RigExec providers
                // alone make them.
                if (B.slotKind[i] != RigExecBakedSlotKind::FirstFramePose) {
                    continue;
                }
                const UsdPrim prim = B.stage->GetPrimAtPath(B.paths[i]);
                for (int c = 0; c < 11; ++c) {
                    _AvarResolution &out = resolved[i * 11 + size_t(c)];
                    out.input = ctx.ResolveBind(
                        prim, RigExecBakedAvarNames[c],
                        RigExecBakedAvarDefaults[c], &out.walk);
                    RigExecBakedRecordBind(&shard, prim,
                                           RigExecBakedAvarNames[c],
                                           &out.input, out.walk);
                    if (out.input.varying || out.input.overrideIndex < 0) {
                        continue;
                    }
                    // Patchable only when the value came from the attribute
                    // itself. A walk longer than one captured it upstream,
                    // through a connection, and an edit on the head would
                    // not be the value the slot holds.
                    out.property = prim.GetPath().AppendProperty(
                        TfToken(RigExecBakedAvarNames[c]));
                    out.patchable = out.walk.size() <= 1 && out.input.head &&
                                    out.input.head.GetPath() == out.property;
                }
            }
            shard.Seal();
        }
    };
    if (RigExecParallelEvaluationEnabled() && !RigExecFrozenSerialActive() &&
        commitChunks > 1) {
        WorkParallelForN(commitChunks, resolveChunks);
    } else {
        resolveChunks(0, commitChunks);
    }
    bindPhases.Next("Bake.input_table.commit");
    const std::vector<int> avarFirst =
        RigExecBakedMergeCommitShards(&B, &avarShards);
    for (int i = 0; i < N; ++i) {
        if (B.slotKind[size_t(i)] != RigExecBakedSlotKind::FirstFramePose) {
            continue;
        }
        const int first = avarFirst[size_t(i) / commitChunkSlots];
        for (int c = 0; c < 11; ++c) {
            const size_t slot = size_t(i) * 11 + size_t(c);
            _AvarResolution &out = resolved[slot];
            renumber(&out.input, first);
            B.avarConstants[slot] = out.input.constant;
            if (out.input.varying) {
                B.avarBindings.push_back({slot, std::move(out.input)});
            } else if (out.input.overrideIndex >= 0) {
                if (out.patchable) {
                    B.patchableAvars[out.property] =
                        B.avarConstantBindings.size();
                }
                B.avarConstantBindings.push_back({slot, std::move(out.input)});
            }
        }
    }
    B.avars = B.avarConstants;
    bindPhases.Close();

    phases.Next("Bake.the_walk");
    // Restated as plain records, because _PoseStep, _SolverBatch and
    // _FrameConstraint are private to the evaluator and bakedPose.cpp is not
    // its friend. The restatement is a copy of the structure only: every
    // VALUE the bake reads still comes off the stage, through the context.
    // Whether a read phase asked for \p target's value as of \p mover. The
    // dynamic path asks this per call, inside recordFrame and beside every
    // chain revision; the program asks it once, here.
    const auto snapshotAfter = [&E](const SdfPath &target,
                                    const SdfPath &mover) {
        const auto wanted = E._chainPlan.snapshots.find(target);
        return wanted != E._chainPlan.snapshots.end() &&
               wanted->second.count(mover) > 0;
    };
    std::vector<RigExecBakedWalkEntry> walk;
    for (const RigExecRigEvaluator::_PoseStep &step : E._poseSteps) {
        RigExecBakedWalkEntry entry;
        entry.solverBatch = step.solverBatch;
        if (step.solverBatch) {
            const auto &batch = E._solverBatches[step.index];
            entry.level = batch.level;
            for (const auto &[solverPath, tap] : batch.solvers) {
                entry.batchSolvers.push_back(solverPath);
                const auto joints = E._solverJoints.find(solverPath);
                entry.solverJoints.push_back(
                    joints == E._solverJoints.end()
                        ? std::vector<std::pair<SdfPath, int>>()
                        : joints->second);
                std::vector<char> snapshotJoints;
                for (const auto &[joint, element] :
                         entry.solverJoints.back()) {
                    snapshotJoints.push_back(
                        snapshotAfter(joint, solverPath) ? 1 : 0);
                }
                entry.solverSnapshotJoints.push_back(
                    std::move(snapshotJoints));
                // The live-rest half of the same binding, carried across so
                // the baked description measures from the same frames the
                // dynamic path's computeRestFrame overrides do. A restInputs
                // entry with an EMPTY predecessor is a joint whose authored
                // rest was merely pinned, not a live one.
                SdfPathVector live;
                for (const auto &[joint, predecessor] : batch.restInputs) {
                    if (predecessor.IsEmpty()) continue;
                    bool named = false;
                    if (joints != E._solverJoints.end()) {
                        for (const auto &[named_, element] : joints->second) {
                            named = named || named_ == joint;
                        }
                    }
                    if (named) live.push_back(joint);
                }
                entry.solverLiveRestJoints.push_back(std::move(live));
            }
        } else {
            const RigExecRigEvaluator::_FrameConstraint &fc =
                E._frameConstraints[step.index];
            entry.constraint.moverPath = fc.moverPath;
            entry.constraint.schemaType = fc.schemaType;
            entry.constraint.targets = fc.targets;
            for (const SdfPath &target : fc.targets) {
                entry.constraint.snapshotTargets.push_back(
                    snapshotAfter(target, fc.moverPath) ? 1 : 0);
            }
            entry.constraint.pointsTarget = fc.pointsTarget;
            entry.constraint.ikChain = fc.ikChain;
            entry.constraint.ikRestLive = fc.ikRestLive;
            entry.constraint.ikUsesAnimatedTs =
                !fc.ikChain.empty() && E._IkUsesAnimatedTs(fc.ikChain);
            entry.constraint.effector = fc.effector.sourcePath;
            entry.constraint.effectorXform = fc.effector.xformPath;
            for (const auto &pole : fc.poleObjects) {
                entry.constraint.poleObjects.push_back(pole.sourcePath);
                entry.constraint.poleObjectXforms.push_back(pole.xformPath);
            }
            for (const auto &source : fc.sources) {
                entry.constraint.sources.push_back(source.sourcePath);
                entry.constraint.sourceXforms.push_back(source.xformPath);
            }
            entry.constraint.worldUpObject = fc.worldUpObject.sourcePath;
            entry.constraint.weightObject = fc.weightObject;
            entry.constraint.worldUpXform = fc.worldUpObject.xformPath;
            entry.constraint.space = fc.spacePath;
            entry.constraint.blendShear = fc.blendShear;
            entry.constraint.worldUpRotationOnly = fc.worldUpRotationOnly;
            entry.constraint.radialBlend = fc.radialBlend;
        }
        walk.push_back(std::move(entry));
    }
    // The aggregate solvers no batch runs, in DEPENDENCY order. A guide-only
    // RigExecBlendPointFrames reads another solver's aggregate, so the
    // reader has to be baked -- and later, run -- second; a dependency that
    // is itself batched is satisfied by the walk and drops out of the
    // ordering. The edges are the evaluator's own (_solverDependencies),
    // because re-deriving them here would be a second expression of the rule
    // that decides the batch levels.
    {
        std::set<SdfPath> batched;
        for (const auto &batch : E._solverBatches) {
            for (const auto &[solverPath, tap] : batch.solvers) {
                batched.insert(solverPath);
            }
        }
        std::set<SdfPath> pending;
        for (const auto &[solverPath, tap] : E._solverArrayTaps) {
            if (!batched.count(solverPath)) {
                pending.insert(solverPath);
            }
        }
        while (!pending.empty()) {
            bool progressed = false;
            for (auto it = pending.begin(); it != pending.end();) {
                const auto edges = E._solverDependencies.find(*it);
                bool ready = true;
                if (edges != E._solverDependencies.end()) {
                    for (const SdfPath &dependency : edges->second) {
                        ready = ready && !pending.count(dependency);
                    }
                }
                if (!ready) {
                    ++it;
                    continue;
                }
                ctx.guideOnlySolvers.push_back(*it);
                it = pending.erase(it);
                progressed = true;
            }
            if (!progressed) {
                // A cycle among unbatched solvers, which the compile refuses
                // first. Refused here too: a reader baked before its input
                // would find that input unbaked. The rest are still baked,
                // in path order, so the tables below stay whole.
                refuse("guide-only aggregate solvers form a dependency cycle",
                       *pending.begin());
                for (const SdfPath &solverPath : pending) {
                    ctx.guideOnlySolvers.push_back(solverPath);
                }
                pending.clear();
            }
        }
    }
    RigExecBakedBuildWalk(&ctx, walk);
    // The native sources the walk registered, and the two buffers the
    // prologue fills and the cone compares. Sized here, once, and never
    // resized in a run.
    B.deltaValues.assign(B.deltaBasePaths.size(), GfMatrix4d(1.0));
    B.deltaPresent.assign(B.deltaBasePaths.size(), 0);
    B.deltaBaseMatrix.assign(B.deltaBasePaths.size(), GfMatrix4d(1.0));
    B.lastDeltaBaseMatrix = B.deltaBaseMatrix;
    B.deltaBaseOk.assign(B.deltaBasePaths.size(), 0);
    B.lastDeltaBaseOk = B.deltaBaseOk;
    B.nativeFrames.assign(B.nativeSources.size(), RigExecPointFrame());
    B.lastNativeFrames = B.nativeFrames;
    B.nativeFrameOk.assign(B.nativeSources.size(), 0);
    B.lastNativeFrameOk = B.nativeFrameOk;

    phases.Next("Bake.publication");
    for (const SdfPath &joint : E._jointPaths) {
        B.jointSlots.push_back(slotOf(joint));
        B.jointPaths.push_back(joint);
    }
    for (const SdfPath &control : E._controlPaths) {
        B.controlSlots.push_back(slotOf(control));
        B.controlPaths.push_back(control);
    }
    for (const auto &[solverPath, tap] : E._solverArrayTaps) {
        const auto it = B.solverIndex.find(solverPath);
        if (it == B.solverIndex.end()) {
            // An assertion now rather than a feature: the walk bakes every
            // batched solver and the guide pass above bakes the rest of
            // _solverArrayTaps, so the two together cover this map exactly.
            refuse("solver publishes guides but was not baked", solverPath);
            continue;
        }
        B.solverArrays.emplace_back(solverPath, it->second);
    }
    // Sorted once, here, so the epilogue can fill each published map from
    // its end instead of searching it for every key. The lists themselves
    // are NOT in path order -- the biped's 252 joints and 74 controls both
    // arrive in binding order -- so the permutation is the whole point.
    const auto publishOrder = [](const std::vector<SdfPath> &paths,
                                 std::vector<int> *order) {
        order->resize(paths.size());
        std::iota(order->begin(), order->end(), 0);
        // STABLE, because of the fallback below: when a list does name one
        // path twice the permutation is still what both fill loops walk,
        // and only a stable sort leaves two equal paths in the order the
        // publication list had them -- which is what makes the assigning
        // fallback publish the same value the assignment it replaces did.
        std::stable_sort(order->begin(), order->end(), [&](int a, int b) {
            return paths[size_t(a)] < paths[size_t(b)];
        });
        // Strictly ascending, not merely sorted: a repeated path would make
        // an emplace keep the first value where the assignment it replaces
        // kept the last, so a list that names one twice is published the
        // old way.
        for (size_t k = 1; k < order->size(); ++k) {
            if (!(paths[size_t((*order)[k - 1])] <
                  paths[size_t((*order)[k])])) {
                return false;
            }
        }
        return true;
    };
    B.jointPathsAscending = publishOrder(B.jointPaths, &B.jointPublishOrder);
    B.controlPathsAscending =
        publishOrder(B.controlPaths, &B.controlPublishOrder);
    std::vector<SdfPath> solverPaths;
    solverPaths.reserve(B.solverArrays.size());
    for (const auto &entry : B.solverArrays) {
        solverPaths.push_back(entry.first);
    }
    B.solverArraysAscending = publishOrder(solverPaths, &B.solverPublishOrder);
    // Sized at Build so the epilogue's two passes allocate nothing.
    B.jointMatrixPublished.assign(B.jointPaths.size(), 0);

    phases.Next("Bake.geometry");
    // Restated for the same reason the walk was: _GraphRevision is private to
    // the evaluator.
    phases.Next("Bake.pose_interpolators");
    // Before the geometry, which looks the weights up by property path to
    // wire a connected blend channel to its slot. The solved table is COPIED
    // from the compiled record: it is a constant of the epoch, and the
    // program is dropped with the epoch.
    for (const RigExecRigEvaluator::_PoseInterpolator &record :
             E._poseInterpolators) {
        RigExecBakedProgramImpl::PoseInterpolator out;
        out.path = record.prim;
        // A numeric driver reads dials, not a frame: it names no driver prim
        // and needs no slot.
        const bool numeric = !record.driverAttributes.empty();
        out.driverSlot = numeric ? -1 : ctx.SlotOf(record.driver);
        if (!numeric && out.driverSlot < 0) {
            ctx.Refuse("pose interpolator driver has no provider slot",
                       record.driver);
            continue;
        }
        if (!record.driverParent.IsEmpty()) {
            out.parentSlot = ctx.SlotOf(record.driverParent);
            if (out.parentSlot < 0) {
                ctx.Refuse("pose interpolator driver parent has no provider "
                           "slot", record.driverParent);
                continue;
            }
        }
        out.allowNegativeWeights = record.allowNegativeWeights;
        out.enableTranslation = record.enableTranslation;
        const UsdPrim prim = B.stage->GetPrimAtPath(record.prim);
        // The interpolator's structure -- its driver, its poses, their
        // rotations and radii -- is epoch identity and recompiles; what a
        // frame reads is the one enable, bound like any other input so an
        // animated or dragged one is honoured and a value edit needs no
        // rebuild.
        B.prims.insert(record.prim);
        B.resolvedRoutedPrims.insert(record.prim);
        out.enabled = ctx.Bind<bool>(prim, "inputs:enabled", true);
        for (const SdfPath &value : record.driverAttributes) {
            const UsdPrim owner =
                B.stage->GetPrimAtPath(value.GetPrimPath());
            // NAMED, for invalidation, but NOT routed: a routed prim tells
            // SetOverrides that every read of it already goes through the
            // resolved inputs, and a drag on the dial would then be placed
            // nowhere -- the bound input below is what reads it, and it is
            // reached through its own override index.
            B.prims.insert(value.GetPrimPath());
            const std::string valueName = value.GetName();
            out.valueInputs.push_back(
                ctx.Bind<double>(owner, valueName.c_str(), 0.0));
        }
        out.values.assign(out.valueInputs.size(), 0.0);
        const auto addWeight = [&B](const SdfPath &weight) {
            const int slot = int(B.poseWeightPaths.size());
            B.poseWeightPaths.push_back(weight);
            B.poseWeightIndex[weight] = slot;
            return slot;
        };
        out.weightBegin = int(B.poseWeightPaths.size());
        for (const SdfPath &weight : record.disabledPoseWeights) {
            out.disabledSlots.push_back(addWeight(weight));
        }
        for (const SdfPath &weight : record.poseWeights) {
            out.poseSlots.push_back(addWeight(weight));
        }
        out.weightEnd = int(B.poseWeightPaths.size());
        out.solver = record.solver;
        B.poseInterpolators.push_back(std::move(out));
    }
    B.poseWeights.assign(B.poseWeightPaths.size(), 0.0f);

    // ---- space switches ------------------------------------------------------
    //
    // Everything structural was settled at compile: which prim is switched,
    // the ordered sources, the masks, and the dependency order between
    // switches. All that is left is to turn paths into slots and to bind the
    // one value that moves -- the active index -- so that keying a space
    // dirties the compose step that reads it and nothing else.
    B.spaceSwitchBySlot.assign(B.paths.size(), -1);
    // Parallel to B.spaceSwitches: the compile's resolution round.
    std::vector<int> switchResolveOrder;
    for (const RigExecRigEvaluator::_SpaceSwitch &sw : E._spaceSwitches) {
        RigExecBakedProgramImpl::SpaceSwitch out;
        out.slot = slotOf(sw.target);
        if (out.slot < 0) continue;
        for (const auto &source : sw.sources) {
            // A source may sit anywhere in the slot order, above or below.
            // Slots are in SdfPath order, which orders a slot after its
            // PARENT and says nothing about a space source in another
            // branch -- so the compose partition cuts the switched subtree
            // into groups of its own and the source frames are declared as
            // reads of that group (see the partition in bakedPose.cpp). The
            // schedule orders the rest.
            out.sourceSlots.push_back(
                source.path.IsEmpty() ? -1 : slotOf(source.path));
            out.filters.push_back(source.filter);
        }
        out.twistAxis = sw.twistAxis;
        // The space is read the same way a source is, and like a source it
        // may sit anywhere in the slot order.
        out.spaceSlot =
            sw.spacePath.IsEmpty() ? -1 : slotOf(sw.spacePath);
        for (int axis = 0; axis < 3; ++axis) {
            out.affectTranslation[axis] = sw.affectTranslation[axis];
            out.affectRotation[axis] = sw.affectRotation[axis];
            out.affectScale[axis] = sw.affectScale[axis];
        }
        // The index comes either from a property on the control -- where the
        // animator keys it -- or from the switch's own inputs:activeSpace.
        // Both are ordinary per-frame inputs, so both bind the same way and
        // an override lands on whichever one the rig actually reads.
        if (!sw.activeAttribute.IsEmpty()) {
            const UsdPrim owner =
                B.stage->GetPrimAtPath(sw.activeAttribute.GetPrimPath());
            if (owner) {
                out.activeInput = ctx.Bind<double>(
                    owner, sw.activeAttribute.GetName().c_str(),
                    sw.activeFallback);
            }
        } else if (const UsdPrim switchPrim =
                       B.stage->GetPrimAtPath(sw.switchPath)) {
            out.activeInput = ctx.Bind<double>(switchPrim,
                                               "inputs:activeSpace",
                                               sw.activeFallback);
        }
        B.spaceSwitchBySlot[size_t(out.slot)] = int(B.spaceSwitches.size());
        B.spaceSwitches.push_back(std::move(out));
        switchResolveOrder.push_back(sw.band);
    }
    // Once every switched slot is known: which version of each frame a
    // switch reads depends on which controls above it are switched.
    RigExecBakedBindSpaceSwitchVersions(&B, switchResolveOrder);

    // ---- auto clavicles ----------------------------------------------------
    //
    // Slots for the frames, and the two per-frame channels bound like a
    // switch's index, so keying either dirties the compose step reading it.
    B.autoClavicleBySlot.assign(B.paths.size(), -1);
    for (const RigExecRigEvaluator::_AutoClavicle &ac : E._autoClavicles) {
        RigExecBakedProgramImpl::AutoClavicle out;
        out.slot = slotOf(ac.target);
        out.pivotSlot = slotOf(ac.pivot);
        out.anchorSlot = slotOf(ac.anchor);
        for (int i = 0; i < 3; ++i) out.fkSlot[i] = slotOf(ac.fk[i]);
        out.ikTargetSlot = ac.ikTarget.IsEmpty() ? -1 : slotOf(ac.ikTarget);
        out.poleSlot = ac.pole.IsEmpty() ? -1 : slotOf(ac.pole);
        if (out.slot < 0 || out.pivotSlot < 0 || out.anchorSlot < 0 ||
            out.fkSlot[0] < 0 || out.fkSlot[1] < 0 || out.fkSlot[2] < 0) {
            continue;
        }
        out.constants = ac.constants;
        const auto bind = [&](const SdfPath &attr, double fallback,
                              RigExecBakedInput<double> *input,
                              RigExecBakedInput<float> *asFloat,
                              bool *isFloat) {
            input->constant = fallback;
            if (attr.IsEmpty()) return;
            const UsdPrim owner = B.stage->GetPrimAtPath(attr.GetPrimPath());
            if (!owner) return;
            const UsdAttribute a = owner.GetAttribute(attr.GetNameToken());
            if (a && a.GetTypeName() == SdfValueTypeNames->Float) {
                *isFloat = true;
                *asFloat = ctx.Bind<float>(owner, attr.GetName().c_str(),
                                           float(fallback));
            } else {
                *input = ctx.Bind<double>(owner, attr.GetName().c_str(),
                                          fallback);
            }
        };
        bind(ac.ikBlendAttribute, 1.0 - ac.constants.ikValue,
             &out.ikBlendInput, &out.ikBlendFloat, &out.ikBlendIsFloat);
        bind(ac.amountAttribute, 1.0, &out.amountInput, &out.amountFloat,
             &out.amountIsFloat);
        for (size_t k = 0; k < B.solvers.size(); ++k) {
            const RigExecBakedProgramImpl::Solver &s = B.solvers[k];
            if (s.type == "RigExecTwoBoneIk" &&
                s.ikParams.softDistancePolicy && s.end >= 0 &&
                s.end == out.ikTargetSlot && s.pole >= 0 &&
                s.pole == out.poleSlot) {
                out.limbSolver = int(k);
                break;
            }
        }
        B.autoClavicleBySlot[size_t(out.slot)] =
            int(B.autoClavicles.size());
        B.autoClavicles.push_back(std::move(out));
    }

    std::vector<RigExecBakedChainSpec> chainSpecs;
    const auto revisionSpec =
        [&snapshotAfter](const RigExecRigEvaluator::_GraphRevision &r) {
            RigExecBakedRevisionSpec spec;
            spec.moverPath = r.moverPath;
            spec.target = r.target;
            spec.op = r.op;
            spec.binding = r.binding;
            spec.transformFinalPhase = r.transformFinalPhase;
            spec.transformPosedPoints = r.transformPosedPoints;
            spec.skinTopologyFixed = r.skinTopologyFixed;
            spec.snapshotAfter = snapshotAfter(r.target, r.moverPath);
            return spec;
        };
    for (const SdfPath &target : E._chainPlan.order) {
        const auto chainIt = E._graphChains.find(target);
        if (chainIt == E._graphChains.end()) continue;
        RigExecBakedChainSpec spec;
        spec.target = target;
        for (const auto &revision : chainIt->second) {
            spec.revisions.push_back(revisionSpec(revision));
        }
        const auto derivedIt = E._graphDerivedChains.find(target);
        if (derivedIt != E._graphDerivedChains.end()) {
            for (const auto &derived : derivedIt->second) {
                spec.derived.push_back(revisionSpec(derived));
            }
        }
        chainSpecs.push_back(std::move(spec));
    }
    RigExecBakedBuildGeometry(&ctx, chainSpecs);

    phases.Next("Bake.the_rest_of_the_invalidation_index");
    // Property chains run INSIDE the program, off the authored stage through
    // the generation's resolved inputs, so their movers are read live like a
    // geometry mover: nothing captured, and an override on one places itself.
    for (const auto &[target, revisions] : E._propertyChains) {
        B.prims.insert(target.GetPrimPath());
        B.resolvedRoutedPrims.insert(target.GetPrimPath());
        // Named: the head tier binds the target's existence and type, which
        // the epoch digest does not hash, so a resync or a type edit of it
        // rebuilds. Its value edits still route through the routed prim.
        B.named.insert(target);
        for (const RigExecRigEvaluator::_PropertyRevision &revision :
                 revisions) {
            B.prims.insert(revision.moverPath);
            B.resolvedRoutedPrims.insert(revision.moverPath);
        }
    }
    // The property revisions as head-tier ops, ordered by what they read and
    // refused when that order cannot be trusted.
    _BindPropertyChains(E, &B);
    RigExecBakedBuildPropertySteps(&B);
    {
        std::string invalid;
        if (!RigExecBakedSortHeadTier(&B, &invalid) ||
            !RigExecBakedValidateHeadTier(B, &invalid)) {
            refuse("the baked head tier is invalid: " + invalid, E._rigPath);
            return nullptr;
        }
    }
    // Bakeability accepted every intervening-Xform candidate BECAUSE the
    // Xforms between it and its anchor compose to the identity and do not
    // animate. That is a value judgement about prims the rest of the index
    // never looks at, so it is recorded here or moving one of them would
    // silently change what the dynamic path composes and not the program.
    {
        const UsdPrim assetRoot =
            B.stage->GetPrimAtPath(E._rigPath.GetParentPath());
        for (const SdfPath &path : E._interveningXformProviders) {
            const auto anchorIt = E._poseProviderAnchors.find(path);
            const UsdPrim anchor =
                anchorIt == E._poseProviderAnchors.end() ||
                        anchorIt->second.IsEmpty()
                    ? assetRoot
                    : B.stage->GetPrimAtPath(anchorIt->second);
            for (UsdPrim walk = B.stage->GetPrimAtPath(path.GetParentPath());
                 walk && walk != anchor; walk = walk.GetParent()) {
                B.xformPrims.insert(walk.GetPath());
            }
        }
    }

    B.posedM.assign(N, GfMatrix4d(1.0));
    B.volumePlacement.assign(N, GfMatrix4d(1.0));
    B.base.resize(N);
    B.fin.resize(N);
    if (!ctx.ok) {
        return nullptr;
    }

    // The volumes this program places and publishes: the walk's, at their
    // slots. Then the mirror of IsBakeable's refusal of a tapped volume
    // with no slot. Same sentence.
    {
        std::set<SdfPath> programVolumes;
        B.placedVolumes.assign(B.noScaleAvars.size(), 0);
        for (size_t i = 0; i < B.noScaleAvars.size(); ++i) {
            if (B.noScaleAvars[i]) {
                programVolumes.insert(B.paths[i]);
                B.placedVolumes[i] =
                    E._volumeWeightMatrixTaps.count(B.paths[i]) ? 1 : 0;
            }
        }
        for (const SdfPath &path : _WalkVolumesWithoutASlot(
                 programVolumes, E._volumeWeightMatrixTaps)) {
            refuse("a volume weight the walk places has no program slot",
                   path);
        }
    }
    // The mirror of IsBakeable's solver-checkpoint guard, so a forced bake
    // cannot slip past it. Same condition, same sentence.
    for (const SdfPath &solver : _SolverCheckpointsWithoutAnOutput(
             E._chainPlan.snapshots, E._solverDependencies, E._solverJoints,
             _BatchedSolvers(E._solverBatches))) {
        refuse("read phase names a solver checkpoint the walk does not commit "
               "for that joint",
               solver);
    }
    // One solver per commit is what makes a stack expressible at all; refuse
    // a batch that would collapse two writers of one slot into one.
    if (!RigExecBakedRefuseBatchedStackWrites(&ctx) || !ctx.ok) {
        return nullptr;
    }

    phases.Next("Bake.the_step_graph");
    // Last, because it is derived from everything above: the steps are the
    // straight line's pieces in the straight line's order, and the edges
    // between them follow from the slot ranges each piece declares. Built
    // once here, so that a frame costs the graph nothing.
    // Its parts are marked one level down, under the phase this one opened,
    // so the trace says which of them the graph's time goes to.
    RigExecProfilePhases graphPhases(&E._profiler, "compile");
    graphPhases.Next("Bake.the_step_graph.pose_steps");
    RigExecBakedBuildPoseSteps(&B);
    if (B.composeCycle) {
        // A guard, not a path: the compile refuses every switch cycle, and
        // versioned switch reads leave none for the groups to close. An
        // unsorted program would read frames no step has written yet.
        refuse("space switches form a compose cycle: two of them each need "
               "the other's space composed first", E._rigPath);
        return nullptr;
    }
    // The rest and ladder composes as head ops, one pair per compose group,
    // ordered with the property revisions by what they read.
    RigExecBakedBuildRestSteps(&B);
    {
        std::string invalid;
        if (!RigExecBakedSortHeadTier(&B, &invalid) ||
            !RigExecBakedValidateHeadTier(B, &invalid)) {
            refuse("the baked head tier is invalid: " + invalid, E._rigPath);
            return nullptr;
        }
    }
    // Between the two halves, which is where a weight object belongs in
    // program order: its placement comes from the pose walk and its packet is
    // what a revision assembles against.
    graphPhases.Next("Bake.the_step_graph.weight_steps");
    RigExecBakedBuildWeightSteps(&B);
    // The pose half's edges and levels, settled BEFORE the geometry half is
    // built. A skin revision is cut into chunks only where the chunks' joints
    // land at different levels (§6), and that question cannot be asked until
    // the ProviderMatrix steps have levels -- so the sweep starts here, over
    // the pose steps alone, and RigExecBakedBuildSchedule extends the same
    // sweep over the geometry steps once they exist. The extension never
    // revisits a step it has passed (RigExecBakedEdgeSweep), so the levels
    // this pass assigns are the levels the final graph holds by
    // construction, and the partition cannot have been cut from a level the
    // graph no longer has -- which is what used to have to be checked after
    // a second, full sweep.
    graphPhases.Next("Bake.the_step_graph.pose_edges");
    RigExecBakedEdgeSweep sweep;
    RigExecBakedBuildStepEdges(&B, &sweep);
    RigExecBakedAssignStepCosts(&B);
    graphPhases.Next("Bake.the_step_graph.geometry_steps");
    RigExecBakedBuildGeometrySteps(&B);
    // RigExecBakedBuildSchedule marks its own parts.
    graphPhases.Close();
    RigExecBakedBuildSchedule(&B, &sweep);
    // The skin layouts as head ops, one per fixed skin revision, now that
    // the geometry steps number the revisions; ordered after the rest and
    // ladder ops, and declared by the steps that read the handle.
    RigExecBakedBuildLayoutSteps(&B);
    {
        std::string invalid;
        if (!RigExecBakedSortHeadTier(&B, &invalid) ||
            !RigExecBakedValidateHeadTier(B, &invalid)) {
            refuse("the baked head tier is invalid: " + invalid, E._rigPath);
            return nullptr;
        }
    }
    RigExecBakedDeclareLayoutReads(&B);
    // The edges cannot show a read whose producer is later or missing, so a
    // program with one would run on a stale value; refuse it instead.
    {
        std::string invalid;
        if (!RigExecBakedValidateStepGraph(B, &invalid)) {
            refuse("the baked step graph is invalid: " + invalid,
                   E._rigPath);
            return nullptr;
        }
        // The readers' declared versions, now that the steps hold them.
        if (!RigExecBakedValidateHeadReads(B, &invalid)) {
            refuse("the baked head tier is invalid: " + invalid, E._rigPath);
            return nullptr;
        }
    }
    // One leaf per binding, now that every binding exists. The first run
    // samples them all (RigExecBakedSampleLeaves, rule 1).
    RigExecBakedNumberLeaves(&B);
    RigExecBakedNoteRestLeaves(&B);
    if (RigExecBakedScheduleReportRequested()) {
        const std::string report = RigExecBakedScheduleReport(B);
        std::fwrite(report.data(), 1, report.size(), stderr);
    }
    if (_ProgramDigestRequested()) {
        _PrintProgramDigest(B);
    }
    return std::unique_ptr<RigExecBakedProgram>(
        new RigExecBakedProgram(std::move(impl)));
}

// One frame.
// The prologue is here rather than in bakedPose.cpp because it is the one
// part of a frame that calls the evaluator's own private routines; everything
// it produces reaches the two halves through the program.

namespace {

// The per-source constraint tables Run reads every frame, interned once at
// load: interning takes the token registry's lock.
const TfToken kSourceWeights("inputs:sourceWeights");
const TfToken kTranslationOffsets("inputs:translationOffsets");
const TfToken kRotationOffsets("inputs:rotationOffsets");
const TfToken kPoleVectorWeights("inputs:poleVectorWeights");

} // namespace

bool
RigExecBakedProgram::Run(UsdTimeCode time, RigExecRigPose *pose)
{
    RigExecBakedProgramImpl &B = *_impl;
    RigExecRigEvaluator &E = *B.evaluator;
    RIGEXEC_PROFILE_SCOPE_CAT(*B.profiler, "Baked", "baked");
    _lastBail = RigExecBakedBail::None;
    // Consumed by this run whatever becomes of it.
    const bool fullRunRequested = _fullRunRequested;
    _fullRunRequested = false;

    // Two clock reads per phase, and only when asked: the profiler's scopes
    // take three mutexes apiece and cost more than the prologue they would
    // be measuring, which is why the phase a frame spends its time in was
    // never readable without the tracer distorting it. See
    // RigExecBakedStepTimingRequested.
    const bool measuring = RigExecBakedStepTimingRequested();
    const auto now = [] {
        return std::chrono::duration<double, std::micro>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    };
    const double frameBegan = measuring ? now() : 0;
    double phaseMark = frameBegan;
    // This frame's three phases, held here and folded into the program's
    // sums at the one exit that counts a frame: the divisor is incremented
    // there too, and a generation a step hands back or the publication
    // declines must not put a prologue and a region into a numerator whose
    // denominator stood still.
    double prologueUs = 0, regionUs = 0;
    bool stageFramesOk = true;

    // Every frame the run reads off the STAGE, read once, here.
    // A step may not touch USD, so the stage-derived frames the pose walk
    // uses are settled in the prologue: the plain Xformables a constraint
    // targets, seeded into their slots' FIRST version, which is where the
    // compose would have left one. The cache is built FRESH for this frame,
    // exactly as _EvaluateDynamic builds its own: a retained
    // UsdGeomXformCache keeps a UsdGeomXformQuery per prim across SetTime,
    // so an xformOp added or reordered would leave a stale query where the
    // dynamic path has none.
    // False means a target's transform could not be resolved at all, which
    // is the one thing the dynamic walk gives the generation back for here
    // -- and gives it back invalid, so a false here ends the frame with the
    // pose as the walk would leave it (see the bail below).
    const auto stageFrames = [&B, &E, time, pose]() {
        if (B.xformSlots.empty() && B.nativeSources.empty() &&
            B.deltaBasePaths.empty()) {
            return true;
        }
        UsdGeomXformCache cache(time);
        for (size_t k = 0; k < B.xformSlots.size(); ++k) {
            const size_t slot = size_t(B.xformSlots[k]);
            RigExecPointFrame frame;
            GfMatrix4d matrix(1.0);
            if (!E._FrameFromXformRelativeToAsset(B.assetRoot, &cache,
                                                  B.paths[slot], &frame,
                                                  &matrix)) {
                pose->diagnostics.push_back(
                    "could not resolve constraint target " +
                    B.paths[slot].GetString() +
                    " relative to the asset root");
                return false;
            }
            B.xformBase[k] = matrix;
            B.base[slot] = frame;
            B.fin[slot] = frame;
        }
        // And the target transform each geometry-domain constraint measures
        // its delta against. A stage read even when the target is a
        // RigExecJoint, because RigExecXformable inherits Xformable and the
        // dynamic walk measures against the authored transform rather than
        // the rig frame. Do not "improve" this: parity says mirror it.
        for (size_t k = 0; k < B.deltaBasePaths.size(); ++k) {
            GfMatrix4d matrix(1.0);
            B.deltaBaseOk[k] = E._FrameFromXformRelativeToAsset(
                B.assetRoot, &cache, B.deltaBasePaths[k], nullptr, &matrix);
            B.deltaBaseMatrix[k] = matrix;
        }
        // And the plain Xformables a constraint reads as a SOURCE. Only the
        // stage half is settled here: the delta such a source rides is the
        // deepest revision above it, which is a frame the walk has not
        // produced yet, so the constraint step performs the ride out of the
        // slots it declared. A source the stage cannot answer for is not a
        // bail -- the dynamic walk diagnoses it per constraint and passes
        // that constraint through -- so the failure is recorded and carried
        // into the step.
        for (size_t k = 0; k < B.nativeSources.size(); ++k) {
            RigExecPointFrame frame;
            B.nativeFrameOk[k] = E._FrameFromXformRelativeToAsset(
                B.assetRoot, &cache, B.nativeSources[k].path, &frame,
                nullptr);
            B.nativeFrames[k] = B.nativeFrameOk[k] ? frame
                                                   : RigExecPointFrame();
        }
        return true;
    };

    // A constraint's own authored tables, read RAW at the frame's time.
    // Not through the resolved inputs and not through a bound query: the
    // dynamic walk reads these straight off the attribute, so a property
    // chain or an interactive override on one is deliberately honoured by
    // neither path. The cardinality line each read can produce is kept
    // beside the values and replayed by the constraint step, which is where
    // the dynamic walk emits it.
    const auto constraintArrays = [&B, time]() {
        for (RigExecBakedProgramImpl::ConstraintArrays &arrays :
                 B.constraintArrays) {
            arrays.diagnostics.clear();
            arrays.ok = RigExecRigEvaluator::_ReadConstraintSourceWeights(
                arrays.prim, kSourceWeights, arrays.sourceCount, time,
                &arrays.diagnostics, &arrays.weights);
            // The dynamic walk stops at the first table it cannot use, so
            // the offsets are not read when the weights were malformed --
            // and a run that read them anyway could produce a second line
            // the reference generation never produced.
            if (arrays.parentOffsets) {
                arrays.ok =
                    arrays.ok &&
                    RigExecRigEvaluator::_ReadConstraintSourceOffsets(
                        arrays.prim, kTranslationOffsets,
                        arrays.sourceCount, time, &arrays.diagnostics,
                        &arrays.translationOffsets) &&
                    RigExecRigEvaluator::_ReadConstraintSourceOffsets(
                        arrays.prim, kRotationOffsets,
                        arrays.sourceCount, time, &arrays.diagnostics,
                        &arrays.rotationOffsets);
            } else {
                arrays.translationOffsets.assign(arrays.sourceCount,
                                                 GfVec3d(0));
                arrays.rotationOffsets.assign(arrays.sourceCount,
                                              GfVec3d(0));
            }
            if (arrays.readPole) {
                arrays.poleDiagnostics.clear();
                arrays.poleOk =
                    RigExecRigEvaluator::_ReadConstraintSourceWeights(
                        arrays.prim, kPoleVectorWeights,
                        arrays.poleCount, time, &arrays.poleDiagnostics,
                        &arrays.poleWeights);
            }
        }
    };

    // Serial, always run, and the only part of a frame that may take a lock,
    // read the stage through anything but a pinned query, or touch the pose
    // as it goes. Everything the region needs from outside itself is settled
    // here.
    {
        RIGEXEC_PROFILE_SCOPE_CAT(*B.profiler, "BakedPrologue", "baked");
        // A math mover's inputs are all authored on itself, so its chain owes
        // exec nothing and resolves first: the head tier runs the chains
        // before anything else reads a result.
        B.propertyResults.clear();
        // The geometry-domain constraint deltas are NOT emptied here. They
        // are slots: what a skipped constraint step left is what it would
        // have measured again, and clearing them would make a cone that
        // skips one deform as though no constraint had ever measured a
        // delta. That was the hazard §7 named; keeping them across runs the
        // way a slot is kept is the answer it offered.
        B.resolvedInputs->Clear();
        B.chainSnapshots->Clear();
        // Interactive overrides are applied before the property chains, as
        // _EvaluateDynamic applies them: a drag on a chain's target is its
        // base, and the chain publishes its own result in the drag's place.
        if (!B.interactiveOverrides->empty()) {
            E._ApplyInteractiveOverridesToResolved(B.resolvedInputs);
        }
        // Every bound input's value at this time, read here, on this
        // thread: nothing after this point reads a binding any other way.
        // A forced run trusts no leaf it holds. The bindings whose walks
        // reach a chain result are read once the chains are published.
        RigExecBakedSampleLeaves(&B, time, fullRunRequested,
                                 RigExecBakedLeafPass::BeforeHead);
        if (B.hasPropertyChains) {
            RIGEXEC_PROFILE_SCOPE_CAT(*B.profiler, "PropertyChains",
                                      "property");
            // The property revisions, from leaves sampled now. The bake's
            // forced run executes every op, so what it exports is computed
            // at its time; every other run re-runs only what moved, and the
            // publication refills the results and the overlay either way.
            RigExecBakedSampleHeadLeaves(&B, time, fullRunRequested);
            RigExecBakedRunHeadTier(&B, time, pose, fullRunRequested,
                                    RigExecBakedVerifyConesRequested());
            RigExecBakedPublishPropertyChains(&B);
            RigExecBakedNoteReaderWalks(&B);
        }
        RigExecBakedSampleLeaves(&B, time, fullRunRequested,
                                 RigExecBakedLeafPass::ChainRouted);
        // The rest and ladder ops, from leaves sampled now (a chain-written
        // channel among them): each re-runs only when what it reads moved,
        // and the bake's forced run composes them all at its time.
        RigExecBakedRunRestTier(&B, pose, fullRunRequested,
                                RigExecBakedVerifyConesRequested());
        // The skin layouts, from leaves sampled now: an op builds a new
        // handle only when its leaves moved, on every generation, and the
        // geometry prologue adopts it where it resolves a topology.
        RigExecBakedRunLayoutTier(&B, time, pose, fullRunRequested,
                                  /*sample=*/true,
                                  RigExecBakedVerifyConesRequested());
        RigExecBakedRunInputs(&B, time);
        RigExecBakedRunSolverSources(&B, time);
        stageFramesOk = stageFrames();
        if (stageFramesOk) {
            constraintArrays();
            RigExecBakedRunGeometryPrologue(&B, time, pose,
                                            fullRunRequested);
        }
    }
    if (!stageFramesOk) {
        // The dynamic walk gives the generation back at the same point, with
        // the same line, having published nothing but what the property
        // chains moved. So this pose is its answer as it stands: the
        // settle's lines, the chains' lines and the target's, the chains'
        // results, and valid left false. The geometry prologue did not run,
        // and that is the walk's state too -- it builds no mover graph, and
        // reports none as created, for a generation it gave back.
        for (const auto &[target, value] : B.propertyResults) {
            pose->movedProperties.emplace_hint(pose->movedProperties.end(),
                                               target, value);
        }
        // The region did not run, but the prologue above has already moved
        // the per-run state the next closure compares against -- the ribbon
        // points it swapped, the ladder moves it recorded -- so the next run
        // cannot trust a cone. It is owed one run of everything, which is
        // what a notice the index could not place asks for. Owed once: a
        // stamp already ahead of the last completed run's is still owed.
        if (B.programStamp == B.lastProgramStamp) {
            ++B.programStamp;
        }
        // No step ran, so the last run's stamps must not answer for this
        // one in GetLastOpTrace or the interval replay.
        RigExecBakedClearRunStamps(&B);
        _lastBail = RigExecBakedBail::StageFrames;
        return false;
    }
    if (measuring) {
        const double mark = now();
        prologueUs = mark - phaseMark;
        phaseMark = mark;
    }

    // A run executes the CLOSURE of what the sources say moved, not the whole
    // program (§7). Under RIGEXEC_BAKED_VERIFY_CONES it does both: the cone
    // run's whole answer is shadowed, the state the prologue left is put
    // back, every step runs, and the two are compared. A difference is
    // reported rather than fatal, so a test can assert on the count.
    const bool verifying = RigExecBakedVerifyConesRequested();
    RigExecBakedRunShadow before, after;
    if (verifying) {
        before.Capture(B);
    }
    bool bailed = false;
    {
        RIGEXEC_PROFILE_SCOPE_CAT(*B.profiler, "BakedRegion", "baked");
        bailed = !RigExecBakedRunSteps(&B, time, fullRunRequested);
    }
    if (measuring) {
        const double mark = now();
        regionUs = mark - phaseMark;
        phaseMark = mark;
    }
    if (verifying) {
        after.Capture(B);
        // What the generation DID, taken before the second pass overwrites
        // it. A verification pass runs everything by construction, so
        // leaving its numbers in place would make every observer -- the run
        // report, GetClustersRunLastGeneration, and so the tests that prove
        // a cone skips anything -- describe the instrument instead of the
        // frame.
        const RigExecBakedRunStatistics coneRun(B);
        const size_t coneClusters = coneRun.closedClusters;
        before.Restore(&B);
        // The second pass is the verifier, not the frame: it is kept out of
        // the per-step accumulators for the same reason the phase marks
        // below skip over it, so that a verified frame's table still says
        // what one frame costs.
        B.measurementSuspended = true;
        const bool bailedFull = !RigExecBakedRunSteps(&B, time, true);
        B.measurementSuspended = false;
        coneRun.Restore(&B);
        std::vector<std::string> differences;
        size_t mismatches = after.Compare(B, &differences);
        if (bailedFull != bailed) {
            ++mismatches;
            differences.push_back(
                "baked cone mismatch: the cone run and the whole program "
                "disagree about giving the generation back");
        }
        bailed = bailedFull;
        pose->bakedParityMismatches += mismatches;
        for (std::string &difference : differences) {
            pose->diagnostics.push_back(std::move(difference));
        }
        if (mismatches > 0) {
            // The same wording a real parity disagreement uses, on the same
            // stream, so one regular expression catches both.
            TF_WARN("rigExec: baked parity mismatch: %zu difference(s) "
                    "between the cone run (%zu of %zu cluster(s)) and the "
                    "whole program",
                    mismatches, coneClusters,
                    B.clustering.clusters.size());
        }
        if (measuring) {
            // Charged to no phase at all. The block above ran the whole
            // program a second time and compared two copies of the state;
            // adding that to the epilogue would make the epilogue of a
            // verified frame read several times the epilogue a caller gets.
            phaseMark = now();
        }
    }

    RIGEXEC_PROFILE_SCOPE_CAT(*B.profiler, "BakedEpilogue", "baked");
    RigExecBakedReplayStepTimings(B);
    if (RigExecBakedScheduleCalibrationRequested()) {
        RigExecBakedScheduleCalibrate(&B);
    }
    if (RigExecBakedScheduleReportRequested()) {
        // What the schedule DID, as against what Build predicted it would:
        // the structural half was printed once, at Build, and this is the
        // only half that needs a frame to have happened.
        const std::string report = RigExecBakedScheduleRunReport(B);
        std::fwrite(report.data(), 1, report.size(), stderr);
    }
    if (bailed) {
        // A failed step leaves publication to the dynamic fallback.
        _lastBail = RigExecBakedBail::Step;
        return false;
    }
    if (!RigExecBakedPublishPose(&B, pose)) {
        _lastBail = RigExecBakedBail::Publish;
        return false;  // the dynamic fallback needs exec
    }
    // Where every volume the walk places ended up: each slot's placement is
    // as its own VolumePlacements step left it. A slot whose step the cone
    // skipped kept a placement whose final frame did not move, so it is
    // this generation's.
    RigExecBakedPublishVolumePlacements(B, &pose->weightFrames);
    RigExecBakedPublishGeometry(&B, pose);

    // The generation's work counters. Two of them are PROGRAM CONSTANTS
    // rather than observations -- the dynamic path's override rounds are the
    // number of distinct batch levels the schedule holds, and the baked
    // meaning of solverEvaluations is the number of solver computations the
    // program requests -- and the rest are the sum of what the steps did.
    pose->solverOverrideRounds += B.solverOverrideRounds;
    pose->solverEvaluations += B.solverEvaluations;
    size_t chainsBuilt = 0, revisionsBuilt = 0;
    for (const RigExecBakedStep &step : B.steps) {
        pose->moverGraphRevisionsExecuted += step.counters.revisionsExecuted;
        pose->moverGraphRevisionsCreated += step.counters.revisionsCreated;
        pose->moverGraphSchedulesBuilt += step.counters.schedulesBuilt;
        chainsBuilt += step.counters.chainsBuilt;
        revisionsBuilt += step.counters.revisionsBuilt;
    }

    pose->diagnostics.push_back(
        "mover graph: " + std::to_string(chainsBuilt) + " chain(s), " +
        std::to_string(revisionsBuilt) + " revision(s); " +
        std::to_string(pose->moverGraphRevisionsCreated) + " created, " +
        std::to_string(pose->moverGraphRevisionsExecuted) + " executed, " +
        std::to_string(pose->moverGraphSchedulesBuilt) +
        " schedule(s) built");
    pose->valid = true;
    if (measuring) {
        // Counted here, at the one exit that published a pose: a frame that
        // bailed or fell back did not run the epilogue this is measuring,
        // and its prologue and region are dropped with it so that every
        // term of the table is divided by the frames that produced it.
        B.timedPrologueUs += prologueUs;
        B.timedRegionUs += regionUs;
        B.timedEpilogueUs += now() - phaseMark;
        ++B.timedFrames;
        RigExecBakedStepTimingReport(&B);
    }
    return true;
}

bool
RigExecBakedProgram::SampleStageFrameSeeds(
    UsdTimeCode time, RigExecStageFrameSeeds *seeds,
    std::string *error) const
{
    if (!seeds) {
        if (error) {
            *error = "no seeds to sample into";
        }
        return false;
    }
    const RigExecBakedProgramImpl &B = *_impl;
    const RigExecRigEvaluator &E = *B.evaluator;
    // The stageFrames of Run, read-only: one cache fresh for the frame (a
    // retained cache would keep a UsdGeomXformQuery per prim across
    // SetTime, stale where the prologue it mirrors has none), the same
    // reader over the same slots and paths, the same failure semantics --
    // an unresolvable target declines the sample as Run declines the run,
    // while native and delta misses record per entry.
    UsdGeomXformCache cache(time);
    RigExecStageFrameSeeds sampled;
    sampled.xformBase.reserve(B.xformSlots.size());
    sampled.xformFrames.reserve(B.xformSlots.size());
    for (size_t k = 0; k < B.xformSlots.size(); ++k) {
        const size_t slot = size_t(B.xformSlots[k]);
        RigExecPointFrame frame;
        GfMatrix4d matrix(1.0);
        if (!E._FrameFromXformRelativeToAsset(B.assetRoot, &cache,
                                              B.paths[slot], &frame,
                                              &matrix)) {
            if (error) {
                *error = "could not resolve constraint target " +
                         B.paths[slot].GetString() +
                         " relative to the asset root";
            }
            return false;
        }
        sampled.xformBase.push_back(matrix);
        sampled.xformFrames.push_back(frame);
    }
    sampled.deltaOk.reserve(B.deltaBasePaths.size());
    sampled.deltaBase.reserve(B.deltaBasePaths.size());
    for (size_t k = 0; k < B.deltaBasePaths.size(); ++k) {
        GfMatrix4d matrix(1.0);
        sampled.deltaOk.push_back(E._FrameFromXformRelativeToAsset(
            B.assetRoot, &cache, B.deltaBasePaths[k], nullptr, &matrix));
        sampled.deltaBase.push_back(matrix);
    }
    sampled.nativeOk.reserve(B.nativeSources.size());
    sampled.nativeFrames.reserve(B.nativeSources.size());
    for (size_t k = 0; k < B.nativeSources.size(); ++k) {
        RigExecPointFrame frame;
        const bool ok = E._FrameFromXformRelativeToAsset(
            B.assetRoot, &cache, B.nativeSources[k].path, &frame, nullptr);
        sampled.nativeOk.push_back(ok);
        sampled.nativeFrames.push_back(ok ? frame : RigExecPointFrame());
    }
    *seeds = sampled;
    return true;
}

size_t
RigExecBakedProgram::GetClusterCount() const
{
    return _impl->clustering.clusters.size();
}

size_t
RigExecBakedProgram::GetClustersRunLastGeneration() const
{
    return _impl->lastClosedClusters;
}

const RigExecBakedProgramImpl &
RigExecBakedProgram::GetStepGraph() const
{
    return *_impl;
}

UsdTimeCode
RigExecBakedProbeTime(const UsdStageRefPtr &stage)
{
    return _ProbeTime(stage);
}

void
RigExecBakedProgram::_SetWalkVolumePlacements(RigExecRigEvaluator *evaluator,
                                              const GfMatrix4d &matrix)
{
    for (auto &[path, placement] : evaluator->_volumeWeightMatrices) {
        placement = matrix;
    }
}

void
RigExecBakedProgramTesting::SetWalkVolumePlacements(
    RigExecRigEvaluator *evaluator, const GfMatrix4d &matrix)
{
    RigExecBakedProgram::_SetWalkVolumePlacements(evaluator, matrix);
}

bool
RigExecBakedProgramTesting::CapturePointReads(
    const RigExecBakedProgram &program)
{
    RigExecBakedProgramImpl &B = *program._impl;
    B.pointCaptures.assign(size_t(B.pointBindingCount),
                           RigExecBakedPointCapture());
    B.capturePointReads = RigExecBakedScheduleModeFromEnvironment() ==
                          RigExecBakedScheduleMode::Serial;
    return B.capturePointReads;
}

std::vector<SdfPath>
RigExecBakedProgramTesting::SolverCheckpointsWithoutAnOutput(
    const std::map<SdfPath, std::set<SdfPath>> &snapshots,
    const std::map<SdfPath, std::set<SdfPath>> &solverDependencies,
    const std::map<SdfPath, std::vector<std::pair<SdfPath, int>>>
        &solverJoints,
    const std::set<SdfPath> &batched)
{
    return _SolverCheckpointsWithoutAnOutput(snapshots, solverDependencies,
                                             solverJoints, batched);
}

// The structural half of RigExecRigEvaluator::_ResolveWeights and
// _ResolveVolumeWeights (rigEvaluatorGeometry.cpp), statement for
// statement: the same reads, the same fallbacks, the same error text. What
// depends on a run -- placement, counts, scalar values, recursion into the
// objects this one composes -- is left to the port.
void
RigExecBakedProgram::DescribeWeightOracle(
    const RigExecRigEvaluator &evaluator, const SdfPath &path,
    UsdTimeCode time, RigExecWeightOracleFacts *facts)
{
    const RigExecRigEvaluator &E = evaluator;
    *facts = RigExecWeightOracleFacts();
    const UsdPrim prim = E._stage->GetPrimAtPath(path);
    if (!prim) {
        facts->staticError = "missing weight object " + path.GetString();
        return;
    }
    const TfToken typeName = prim.GetTypeName();
    if (!evaluatorDetail::_IsWeightObjectType(typeName)) {
        facts->staticError = "unknown weight object type " +
                             typeName.GetString() + " on " +
                             path.GetString();
        return;
    }
    const auto readToken = [&](const char *name, const char *fallback) {
        TfToken value(fallback);
        if (const UsdAttribute a = prim.GetAttribute(TfToken(name))) {
            a.Get(&value, time);
        }
        return value;
    };
    const std::string who = prim.GetPath().GetAsString();
    if (typeName == TfToken("RigExecCombineWeight")) {
        const TfToken mode = readToken("rigExec:combineMode", "multiply");
        if (mode != "multiply" && mode != "add" && mode != "subtract" &&
            mode != "max" && mode != "min" && mode != "average" &&
            mode != "overlay") {
            facts->staticError =
                who + ": unknown rigExec:combineMode " + mode.GetString();
        }
        return;
    }
    if (evaluatorDetail::_IsVolumeWeightType(typeName)) {
        bool inFlight = false;
        std::string why;
        if (!evaluatorDetail::_VolumeWeightSamplesInFlight(prim, &inFlight,
                                                           &why)) {
            facts->phaseError = who + ": " + why;
            return;
        }
        facts->samplesInFlight = inFlight;
        // What the facts below hold at `time` only, while the oracle reads
        // it at every evaluation time.
        const auto noteAnimated = [&](const UsdAttribute &a) {
            if (facts->timeVarying.IsEmpty() && a &&
                (a.ValueMightBeTimeVarying() || a.GetNumTimeSamples() > 0)) {
                facts->timeVarying = a.GetPath();
            }
        };
        // The attribute _ReadTargetPoints reads through a relationship.
        const auto pointsSource = [&](const char *relationship) {
            SdfPathVector targets;
            if (const UsdRelationship rel =
                    prim.GetRelationship(TfToken(relationship))) {
                rel.GetTargets(&targets);
            }
            return targets.size() == 1
                       ? E._stage->GetAttributeAtPath(
                             evaluatorDetail::_ResolveGeometryInput(
                                 E._stage, targets[0]))
                       : UsdAttribute();
        };
        if (!inFlight) {
            const bool fromSampleSource =
                E._ReadTargetPoints(prim, TfToken("rigExec:sampleSource"),
                                    time, &facts->samples);
            facts->haveSamples =
                fromSampleSource ||
                E._ReadTargetPoints(prim, TfToken("rigExec:weightTarget"),
                                    time, &facts->samples);
            if (!facts->haveSamples) {
                facts->samples.clear();
            }
            noteAnimated(pointsSource("rigExec:sampleSource"));
            if (!fromSampleSource) {
                noteAnimated(pointsSource("rigExec:weightTarget"));
            }
        }
        if (typeName == TfToken("RigExecCurveWeight")) {
            facts->haveCurve =
                E._ReadTargetPoints(prim, TfToken("rigExec:curve"), time,
                                    &facts->curve) &&
                !facts->curve.empty();
            if (!facts->haveCurve) {
                facts->curve.clear();
            }
            noteAnimated(pointsSource("rigExec:curve"));
        }
        if (typeName == TfToken("RigExecPlaneWeight")) {
            facts->planeAxis = readToken("rigExec:planeAxis", "y");
            facts->planeBounds = readToken("rigExec:planeBounds", "unbounded");
            noteAnimated(prim.GetAttribute(TfToken("rigExec:planeAxis")));
            noteAnimated(prim.GetAttribute(TfToken("rigExec:planeBounds")));
        }
        return;
    }

    const TfToken representation =
        readToken("rigExec:representation", "constant");
    const TfToken rangePolicy = readToken("rigExec:rangePolicy", "strict");
    if (rangePolicy != "strict" && rangePolicy != "clamp") {
        facts->staticError = "unknown rangePolicy on " + path.GetString();
        return;
    }
    if (typeName == TfToken("RigExecDynamicWeight")) {
        if (readToken("rigExec:operation", "multiply") != "multiply") {
            facts->staticError =
                "unknown dynamic-weight operation on " + path.GetString();
            return;
        }
        SdfPathVector baseTargets;
        if (const UsdRelationship rel =
                prim.GetRelationship(TfToken("rigExec:baseWeight"))) {
            rel.GetTargets(&baseTargets);
        }
        if (baseTargets.size() > 1) {
            facts->staticError =
                "rigExec:baseWeight must have at most one target on " +
                path.GetString();
            return;
        }
        if (baseTargets.empty()) {
            if (representation != "constant") {
                facts->staticError =
                    "no-base dynamic weight must be constant on " +
                    path.GetString();
            }
            return;
        }
        const UsdPrim basePrim = E._stage->GetPrimAtPath(baseTargets[0]);
        if (!basePrim) {
            facts->staticError =
                "missing base weight object on " + path.GetString();
            return;
        }
        const auto canonicalWeightTarget = [&](const UsdPrim &p) {
            SdfPathVector t;
            if (const UsdRelationship rel =
                    p.GetRelationship(TfToken("rigExec:weightTarget"))) {
                rel.GetTargets(&t);
            }
            return t.size() == 1
                       ? evaluatorDetail::_ResolveGeometryInput(E._stage,
                                                                t[0])
                       : SdfPath();
        };
        if (canonicalWeightTarget(prim) != canonicalWeightTarget(basePrim) ||
            canonicalWeightTarget(prim).IsEmpty()) {
            facts->staticError =
                "dynamic/base weight target mismatch on " + path.GetString();
            return;
        }
        TfToken baseRepresentation("constant");
        if (const UsdAttribute a =
                basePrim.GetAttribute(TfToken("rigExec:representation"))) {
            a.Get(&baseRepresentation, time);
        }
        if (baseRepresentation != representation) {
            facts->staticError = "dynamic/base representation mismatch on " +
                                 path.GetString();
            return;
        }
        if (representation == "sparse") {
            VtIntArray mine;
            if (const UsdAttribute a =
                    prim.GetAttribute(TfToken("rigExec:indices"))) {
                a.Get(&mine, time);
            }
            if (!mine.empty()) {
                VtIntArray theirs;
                if (const UsdAttribute a =
                        basePrim.GetAttribute(TfToken("rigExec:indices"))) {
                    a.Get(&theirs, time);
                }
                const std::set<int> mySupport(mine.begin(), mine.end());
                const std::set<int> baseSupport(theirs.begin(),
                                                theirs.end());
                if (mySupport != baseSupport) {
                    facts->staticError =
                        "dynamic/base sparse support mismatch on " +
                        path.GetString();
                }
            }
        }
        return;
    }
    for (const char *field :
         {"rigExec:values", "rigExec:indices", "rigExec:defaultWeight",
          "rigExec:representation", "rigExec:rangePolicy"}) {
        const UsdAttribute a = prim.GetAttribute(TfToken(field));
        if (a && (a.GetNumTimeSamples() > 0 || a.HasAuthoredConnections())) {
            facts->staticError = std::string("static weight field ") + field +
                                 " has time samples or connections on " +
                                 path.GetString();
            return;
        }
    }
}

bool
RigExecBakedProgram::ComposeEnvelopeObjects(
    const RigExecRigEvaluator &evaluator,
    const RigExecBakedProgramImpl &program,
    std::vector<RigExecBakedEnvelopeObject> *objects,
    std::map<SdfPath, int> *index, std::string *error)
{
    const RigExecRigEvaluator &E = evaluator;
    const RigExecBakedProgramImpl &B = program;
    objects->clear();
    index->clear();
    const int stepBacked = int(B.weightObjects.size());
    std::string failure;
    const auto refuse = [&](const std::string &what, const SdfPath &where) {
        if (failure.empty()) {
            failure = what + ": " + where.GetString();
        }
        return -1;
    };
    // Depth first and entered on the way out, like
    // RigExecBakedBakeWeightObject: -1 in `index` marks an object under way.
    std::function<int(const SdfPath &)> compose =
        [&](const SdfPath &path) -> int {
        if (!failure.empty()) {
            return -1;
        }
        const auto baked = B.weightIndex.find(path);
        if (baked != B.weightIndex.end() && baked->second >= 0) {
            return baked->second;
        }
        const auto seen = index->find(path);
        if (seen != index->end()) {
            return seen->second < 0
                       ? refuse("weight object composition contains a cycle",
                                path)
                       : seen->second;
        }
        const UsdPrim prim = B.stage->GetPrimAtPath(path);
        if (!prim) {
            return refuse("weight object prim is missing", path);
        }
        const TfToken type = prim.GetTypeName();
        if (!evaluatorDetail::_IsWeightObjectType(type)) {
            return refuse("not a weight object type", path);
        }
        if (evaluatorDetail::_IsVolumeWeightType(type)) {
            return refuse("a volume weight cannot be an envelope", path);
        }
        (*index)[path] = -1;
        RigExecBakedEnvelopeObject object;
        object.path = path;
        object.type = type;
        const SdfPathVector bases = _Targets(prim, "rigExec:baseWeight");
        if (type == TfToken("RigExecDynamicWeight") && bases.size() > 1) {
            return refuse("more than one rigExec:baseWeight target", path);
        }
        if (!bases.empty()) {
            object.base = compose(bases[0]);
        }
        for (const SdfPath &input : _Targets(prim, "rigExec:inputWeights")) {
            const int composed = compose(input);
            if (composed >= 0) {
                object.inputs.push_back(composed);
            }
        }
        if (!failure.empty()) {
            return -1;
        }
        object.representation =
            _ReadToken(prim, "rigExec:representation", "constant");
        object.rangePolicy = _ReadToken(prim, "rigExec:rangePolicy", "strict");
        object.combineMode = _ReadToken(prim, "rigExec:combineMode", "");
        if (type == TfToken("RigExecCombineWeight")) {
            const TfToken &mode = object.combineMode;
            if (mode != "multiply" && mode != "add" && mode != "subtract" &&
                mode != "max" && mode != "min" && mode != "average" &&
                mode != "overlay") {
                return refuse("unknown rigExec:combineMode", path);
            }
        } else if (object.rangePolicy != "strict" &&
                   object.rangePolicy != "clamp") {
            return refuse("unknown rigExec:rangePolicy", path);
        }
        if (const UsdAttribute a =
                prim.GetAttribute(TfToken("rigExec:values"))) {
            VtFloatArray values;
            a.Get(&values);
            object.values.assign(values.begin(), values.end());
        }
        if (const UsdAttribute a =
                prim.GetAttribute(TfToken("rigExec:indices"))) {
            VtIntArray indices;
            a.Get(&indices);
            object.indices.assign(indices.begin(), indices.end());
        }
        const auto read = [&](const char *name, float fallback) {
            RigExecBakedEnvelopeObject::Read r;
            r.head = prim.GetAttribute(TfToken(name));
            r.fallback = fallback;
            return r;
        };
        // The oracle's fallbacks (rigEvaluatorGeometry.cpp _ResolveWeights).
        object.defaultWeight = read("rigExec:defaultWeight", 0.0f);
        object.driver = read("inputs:driver", 1.0f);
        object.scale = read("inputs:scale", 1.0f);
        object.bias = read("inputs:bias", 0.0f);
        object.strength = read("inputs:strength", 1.0f);
        object.invert = read("inputs:invert", 0.0f);
        const int composedIndex = stepBacked + int(objects->size());
        (*index)[path] = composedIndex;
        objects->push_back(std::move(object));
        return composedIndex;
    };
    for (const RigExecBakedProgramImpl::Constraint &c : B.constraints) {
        if (!c.weightObject.IsEmpty() && c.pointsTarget.IsEmpty()) {
            compose(c.weightObject);
        }
    }
    for (const SdfPath &target : E._propertyChainOrder) {
        const auto chain = E._propertyChains.find(target);
        if (chain == E._propertyChains.end()) {
            continue;
        }
        for (const RigExecRigEvaluator::_PropertyRevision &revision :
             chain->second) {
            const SdfPathVector weightObjects =
                _Targets(B.stage->GetPrimAtPath(revision.moverPath),
                         "rigExec:weightObject");
            if (!weightObjects.empty()) {
                compose(weightObjects[0]);
            }
        }
    }
    if (!failure.empty()) {
        if (error) {
            *error = failure;
        }
        return false;
    }
    return true;
}

// _CompilePropertyChains' results as _EvaluatePropertyChains binds them:
// the chain list, each revision's mover inputs (_BindInput), operation
// (_ReadOperation) and weight object, and the phased connections.
bool
RigExecBakedProgram::DescribePropertyChains(
    const RigExecRigEvaluator &evaluator,
    std::vector<RigExecBakedPropertyChainDesc> *chains, std::string *error)
{
    using Desc = RigExecBakedPropertyChainDesc;
    const RigExecRigEvaluator &E = evaluator;
    chains->clear();
    const auto fail = [&](const std::string &what) {
        if (error) {
            *error = what;
        }
        return false;
    };
    // The runChain arm and RigExecPhasedConsumerValue compare type names.
    const auto valueType = [](const SdfValueTypeName &type) {
        return type == SdfValueTypeNames->Float      ? Desc::ValueType::Float
               : type == SdfValueTypeNames->Double   ? Desc::ValueType::Double
               : type == SdfValueTypeNames->Matrix4d ? Desc::ValueType::Matrix4d
                                                     : Desc::ValueType::Vec3f;
    };
    const TfToken enabledName("inputs:enabled");
    const TfToken defaultWeightName("inputs:defaultWeight");
    const TfToken operationName("rigExec:operation");
    const TfToken valueName("inputs:value");
    const TfToken minName("inputs:min");
    const TfToken maxName("inputs:max");
    const TfToken keysName("inputs:keys");
    const TfToken tangentsName("inputs:tangents");
    for (const SdfPath &orderedTarget : E._propertyChainOrder) {
        const auto found = E._propertyChains.find(orderedTarget);
        if (found == E._propertyChains.end()) {
            continue;
        }
        Desc chain;
        chain.target = found->first;
        chain.targetAttr = E._stage->GetAttributeAtPath(chain.target);
        if (!chain.targetAttr) {
            return fail("property chain " + chain.target.GetString() +
                        ": target attribute disappeared");
        }
        chain.valueType = valueType(chain.targetAttr.GetTypeName());
        const bool floatArm = chain.valueType == Desc::ValueType::Float ||
                              chain.valueType == Desc::ValueType::Double;
        for (const RigExecRigEvaluator::_PropertyRevision &revision :
             found->second) {
            Desc::Revision r;
            r.mover = revision.moverPath;
            const UsdPrim mover = E._stage->GetPrimAtPath(r.mover);
            if (!mover) {
                return fail("property mover " + r.mover.GetString() +
                            " is missing");
            }
            const SdfPathVector weightObjects =
                _Targets(mover, "rigExec:weightObject");
            if (!weightObjects.empty()) {
                r.weightObject = weightObjects[0];
            }
            r.enabled = mover.GetAttribute(enabledName);
            r.defaultWeight = mover.GetAttribute(defaultWeightName);
            r.value = mover.GetAttribute(valueName);
            r.minimum = mover.GetAttribute(minName);
            r.maximum = mover.GetAttribute(maxName);
            TfToken operation;
            if (const UsdAttribute a = mover.GetAttribute(operationName)) {
                a.Get(&operation);
            }
            r.opValid = RigExecParsePropertyOp(operation, &r.op);
            if (floatArm && r.opValid && r.op == RigExecPropertyOp::Curve) {
                const UsdAttribute keys = mover.GetAttribute(keysName);
                const UsdAttribute tangents = mover.GetAttribute(tangentsName);
                for (const UsdAttribute &a : {keys, tangents}) {
                    if (RigExecBakedAnimatedOrConnected(a)) {
                        return fail("property mover " + r.mover.GetString() +
                                    " reads " + a.GetPath().GetString() +
                                    ", which is animated or connected; a "
                                    "baked curve holds its keys at one time "
                                    "only");
                    }
                }
                VtArray<GfVec2f> values;
                if (keys && keys.Get(&values)) {
                    r.keys.assign(values.begin(), values.end());
                }
                r.hasTangentsAttr = bool(tangents);
                values.clear();
                if (tangents && tangents.Get(&values)) {
                    r.tangents.assign(values.begin(), values.end());
                }
            }
            chain.revisions.push_back(std::move(r));
        }
        for (const RigExecPhasedConnection &connection :
             E._phasedConnections) {
            if (connection.target != chain.target) {
                continue;
            }
            Desc::Phased phased;
            phased.consumer = connection.consumer;
            const Desc::ValueType consumer =
                valueType(connection.consumerType);
            phased.consumerType = consumer == Desc::ValueType::Float ||
                                          consumer == Desc::ValueType::Double
                                      ? consumer
                                      : chain.valueType;
            phased.applied = connection.applied;
            phased.hops = connection.hops;
            chain.phased.push_back(std::move(phased));
        }
        chains->push_back(std::move(chain));
    }
    return true;
}

}  // namespace rigExec
