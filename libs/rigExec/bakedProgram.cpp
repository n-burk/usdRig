//
// The baked program: build and run. See bakedProgram.h for what it is and
// why it is a request rather than a promise.
//
// Everything here is a second expression of semantics that live elsewhere --
// the provider compose and the default-space ladder in computations.cpp, the
// pose walk in rigEvaluator.cpp, the geometry revision in moverGraph.cpp --
// so every piece is written against the one it mirrors and nothing is
// re-derived from first principles. Where a kernel can simply be CALLED
// instead of mirrored (the constraint operators, the skin kernel, the
// extent/normal kernels, the packet assembler, the property chains) it is,
// because a shared call cannot drift and a copy can.
//
//
// Phase 2 split this file by domain: the pose walk lives in bakedPose.cpp and
// the geometry chains in bakedGeometry.cpp, both over the state declared in
// bakedProgramImpl.h. What stays here is the public surface, the bakeability
// judgement, the Build skeleton that hands each domain its share, and the
// parity comparator -- plus the one thing neither half may do, which is read
// the evaluator's private state. This is the only translation unit the
// evaluator declares a friend, so Build captures what a frame needs of it
// once and the halves read the program instead.
//
#include "bakedProgram.h"

#include "bakedProgramImpl.h"
#include "bakedSchedule.h"
#include "frameExtraction.h"
#include "moverGraph.h"
#include "parallel.h"
#include "rigEvaluator.h"
#include "types.h"

#include "rigExecMath/pointFrame.h"

#include "pxr/base/gf/math.h"
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

// The weight-object schemas the program can build a packet for.
//
// It is about the BUILDER and nothing else. The volumetric three are here
// because RigExecBuildVolumeWeightPacket is one of the builders, and they are
// still refused above by the _volumeWeightMatrixTaps loop, which is about the
// PLACEMENT a volume's field needs -- which the pose walk now composes, so
// the two questions have one answer again.
// RigExecCurvenetWeight is here too, now that the program holds a bind of its
// own: the cut and the factorization are resolved into the weight object and
// only the right-hand side is solved per frame.
bool
_IsBakedWeightType(const TfToken &type)
{
    return type == "RigExecStaticWeight" || type == "RigExecDynamicWeight" ||
           type == "RigExecCombineWeight" ||
           type == "RigExecCurvenetWeight" || _IsVolumeWeightTypeName(type);
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

// ---------------------------------------------------------------------------
// Bakeability.
// ---------------------------------------------------------------------------

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

    // ---- pose providers ---------------------------------------------------
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

    // ---- pose interpolators -------------------------------------------------
    // The step reads the driver's frame, and its parent's, out of the slots,
    // so both have to BE slots. Compile already refused a driver that is not a
    // joint or control of the rig; this is the same fact stated against the
    // program's own table.
    for (const RigExecRigEvaluator::_PoseInterpolator &record :
             E._poseInterpolators) {
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
        //
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

    // ---- solvers ----------------------------------------------------------
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

    // ---- weight objects -----------------------------------------------------
    //
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

    // ---- constraints -------------------------------------------------------
    for (const auto &constraint : E._frameConstraints) {
        if (!_IsBakedConstraintType(constraint.schemaType)) {
            say("constraint type not baked (" +
                    constraint.schemaType.GetString() + ")",
                constraint.moverPath);
            continue;
        }
        // A volume anywhere in the closure used to be refused here, on the
        // grounds that a constraint's envelope is resolved by the CPU oracle
        // out of _volumeWeightMatrices AS THAT MAP STANDS AT THE
        // CONSTRAINT'S POINT IN THE WALK, while the program republishes the
        // map once, after the walk. That reasoning describes a constraint
        // the epoch cannot hold: a volumetric field requires a POINT domain
        // (_ValidateWeightObjectDomain, composed inputs included), so the
        // only constraint that can bind one is a geometry-domain constraint
        // -- and a geometry-domain constraint resolves NO envelope in the
        // walk at all. Its weight is per point and resolves after the solve,
        // on the revision its delta feeds, where the placement is the same
        // one every other mover's packet uses. So the map is never read
        // mid-walk by anything, and one republication after it is what every
        // reader of it sees.
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
                //
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

    // ---- geometry ----------------------------------------------------------
    auto checkRevision = [&](const RigExecRigEvaluator::_GraphRevision &r,
                             bool derived) {
        const bool supported =
            derived ? (r.op == RigExecRevisionOp::RecomputeExtent ||
                       r.op == RigExecRevisionOp::RecomputeNormals)
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
                       r.op == RigExecRevisionOp::Curvenet ||
                       r.op == RigExecRevisionOp::CurvenetAdjuster ||
                       r.op == RigExecRevisionOp::EmitGuidePoints ||
                       r.op == RigExecRevisionOp::Ribbon ||
                       r.op == RigExecRevisionOp::Wire ||
                       r.op == RigExecRevisionOp::VolumeCorrect ||
                       r.op == RigExecRevisionOp::Smooth ||
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

// ---------------------------------------------------------------------------
// The bake.
// ---------------------------------------------------------------------------

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

void RigExecBakedProgram::AdoptGeometryStateFrom(
    RigExecBakedProgram &previous) {
    RigExecBakedProgramImpl &B = *_impl;
    RigExecBakedProgramImpl &P = *previous._impl;
    // One revision's run state, moved from the node the outgoing program
    // held. Everything here is a CACHE of the last run; the compiled
    // description around it comes from the epoch the new program was built
    // from and is left alone.
    //
    // \p keepRun says whether the cached RESULT may be kept as well as the
    // node: a revision spliced into or out of a chain changes the point
    // stream every revision after it reads, which is why the dynamic path's
    // VdfNetwork re-executes them, so from the first divergence on the node
    // survives but its result does not.
    const auto adopt = [](RigExecBakedProgramImpl::GeomRevision *destination,
                          RigExecBakedProgramImpl::GeomRevision *source,
                          bool keepRun) {
        destination->created = false;
        // The adjuster's control frames are the node's MUTABLE member state,
        // which survives a Compute the node did not run -- so they survive a
        // rebuild the node survived, whatever happens to its result. A
        // revision below the first divergence re-executes and overwrites
        // them; one that does not would otherwise publish nothing where the
        // dynamic path publishes its last answer.
        destination->controlFrames = std::move(source->controlFrames);
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
        //
        // Carried only where the two tables are the same shape. keepRun says
        // the revision is the same mover at the same place in the chain; it
        // does not say its binding still names the same joints, and the fold
        // writes one entry per influence slot of the NEW binding.
        if (source->influences.size() == destination->influences.size()) {
            destination->influences = std::move(source->influences);
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
    // A curvenet bind is cached against the layout it was cut from, which an
    // edit that rebuilds the program need not have touched; carrying the
    // cache is the same judgement the revision results below get, and it also
    // carries any bind diagnostic the outgoing program had not drained yet.
    B.curvenetBindings = std::move(P.curvenetBindings);
    // A curvenet WEIGHT object's bind is the same judgement and the
    // expensive one -- it cuts the mesh and factorizes its Laplacian, which
    // is the costliest step a rig can carry on its first frame -- so an edit
    // that rebuilt the program for some unrelated reason must not pay for it
    // again. Matched by PATH, because a rebuild can add or drop objects and
    // the indices need not line up, and only where the TYPE still agrees.
    // Correctness does not rest on this: the step compares the layout it
    // holds against the frame's own before it uses the bind, and re-cuts on
    // any difference, so a carried bind is a hint and never an answer.
    {
        std::map<SdfPath, RigExecBakedProgramImpl::WeightObject *> outgoing;
        for (RigExecBakedProgramImpl::WeightObject &object : P.weightObjects) {
            if (object.bound) {
                outgoing.emplace(object.path, &object);
            }
        }
        for (RigExecBakedProgramImpl::WeightObject &object : B.weightObjects) {
            const auto found = outgoing.find(object.path);
            if (found == outgoing.end() || found->second->type != object.type) {
                continue;
            }
            RigExecBakedProgramImpl::WeightObject &source = *found->second;
            object.curvenetBinding = std::move(source.curvenetBinding);
            object.boundMesh = std::move(source.boundMesh);
            object.boundNet = std::move(source.boundNet);
            object.boundCounts = std::move(source.boundCounts);
            object.boundIndices = std::move(source.boundIndices);
            object.boundSplines = std::move(source.boundSplines);
            object.boundSmooth = std::move(source.boundSmooth);
            object.boundSamples = source.boundSamples;
            object.bound = true;
        }
    }
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
        }
    }
}

// ---------------------------------------------------------------------------
// Invalidation.
// ---------------------------------------------------------------------------

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
    for (const SdfPath &path : notice.GetChangedInfoOnlyPaths()) {
        if (B.rebuild.count(path) ||
            (path.IsPropertyPath() &&
             B.xformPrims.count(path.GetPrimPath()))) {
            return true;
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
        if (overlaps(path)) {
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
    //
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

// ---------------------------------------------------------------------------
// Interactive overrides.
// ---------------------------------------------------------------------------

void
RigExecBakedProgram::BumpProgramStamp()
{
    ++_impl->programStamp;
}

namespace {

// One captured avar, moved to a new constant.
//
// Three copies of the value exist and all three move: the binding (what a
// drag's release writes back from), the constant table (what a rebuild would
// have captured), and the working table the frame reads. The working slot is
// left alone while an override stands on it -- the drag owns that slot, and
// the constant pass that runs when it is released writes this new constant
// back, which is exactly the value an authored edit under a drag should land
// on. Nothing is marked dirty: the frame path compares every avar slot with
// last run's by VALUE, which is the whole of what the cone needs.
//
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
RigExecBakedProgram::ApplyAvarValueEdits(
    const UsdNotice::ObjectsChanged &notice)
{
    RigExecBakedProgramImpl &B = *_impl;
    if (!notice.GetResolvedAssetPathsResyncedPaths().empty()) {
        return false;
    }
    static const TfToken kDefault("default");
    static const TfToken kTypeName("typeName");
    static const TfToken kSpline("spline");

    // Decide EVERYTHING before writing anything: a notice this cannot fully
    // absorb must leave the table exactly as the rebuild path expects it.
    struct _Patch {
        size_t binding;
        double value;
        bool animated;
    };
    std::vector<_Patch> patches;
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
            patches.push_back({found->second, 0.0, true});
            continue;
        }
        if (attribute.ValueMightBeTimeVarying()) {
            return false;  // varies some way this does not know
        }
        // Cleared back to nothing reads as the channel's default, which is
        // the fallback the bake captures for an attribute with no value.
        double value = RigExecBakedAvarDefaults[binding.slot % 11];
        attribute.Get(&value, UsdTimeCode::Default());
        patches.push_back({found->second, value, false});
    }
    if (patches.empty()) {
        return false;
    }
    for (const _Patch &patch : patches) {
        RigExecProgramAvarPatch(&B, patch.binding, patch.value,
                                patch.animated);
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
        // Unplaceable for the opposite reason to `folded`: the value IS
        // re-read every frame, and the program would honour the override
        // while exec type-rejects it. See execTypedArrayInputs. Ahead of
        // the routed-prim test below, which would otherwise say "already
        // routed, nothing to do" for the prim these properties live on.
        if (B.execTypedArrayInputs.count(path)) {
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
        // Nothing to place: every read of this prim already goes through the
        // resolved inputs the override was written into.
        if (B.resolvedRoutedPrims.count(o.prim)) {
            continue;
        }
        placeable = false;
    }
    return placeable;
}

// ---------------------------------------------------------------------------
// Build.
// ---------------------------------------------------------------------------

void
RigExecBakedBuildContext::Refuse(const std::string &what, const SdfPath &where)
{
    if (reasons) reasons->push_back(what + ": " + where.GetString());
    ok = false;
}

void
RigExecBakedBuildContext::Fold(const UsdPrim &prim, const char *name)
{
    if (!prim) {
        return;
    }
    const SdfPath path = prim.GetPath().AppendProperty(TfToken(name));
    program->rebuild.insert(path);
    program->folded.insert(path);
    program->named.insert(path);
    program->prims.insert(prim.GetPath());
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
    phases.Next("Bake.IsBakeable");
    if (!IsBakeable(E, reasons)) {
        return nullptr;
    }
    auto impl = std::make_unique<RigExecBakedProgramImpl>();
    RigExecBakedProgramImpl &B = *impl;
    B.evaluator = evaluator;
    B.stage = E._stage;
    B.assetRootPath = E._rigPath.GetParentPath();
    // The evaluator state a frame reads, captured here because this is the
    // only translation unit its friendship reaches; bakedProgramImpl.h says
    // why each one is a pointer rather than a copy.
    B.resolvedInputs = &E._resolvedInputs;
    B.chainSnapshots = &E._chainSnapshots;
    B.skinTopologies = &E._skinTopologies;
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
    B.volumeWeightMatrices = &E._volumeWeightMatrices;
    B.currentPhaseWeights = E._currentPhaseWeights;
    B.updateVolumePlacements =
        [evaluator](const std::function<
                        bool(const SdfPath &, RigExecPointFrame *)> &lookup) {
            // The pose is the routine's second output and this call wants
            // only its first: what the program publishes is the MAP, which
            // the epilogue copies where the dynamic walk assigns it from.
            // One empty pose per frame, and nothing in it but the same
            // handful of matrices.
            RigExecRigPose unused;
            evaluator->_UpdateVolumePlacements(
                RigExecPoseFrameLookup(lookup), &unused);
        };
    // The oracle a constraint's envelope resolves through, bound here
    // because this is the only translation unit the evaluator's friendship
    // reaches. `evaluator` outlives the program -- the evaluator owns it and
    // drops it on any epoch change -- so capturing the pointer is safe in
    // exactly the way every other capture in this block is.
    B.resolveWeights = [evaluator](const SdfPath &weightPath, size_t count,
                                   UsdTimeCode time,
                                   std::vector<float> *weights,
                                   std::string *error,
                                   const std::vector<GfVec3f> *current) {
        return evaluator->_ResolveWeights(weightPath, count, time, weights,
                                          error, current);
    };
    // The reader a sparse blend sample's shape resolves through, on a cache
    // miss: the evaluator's own, so the program and the dynamic walk admit
    // and refuse exactly the same shapes.
    B.resolveBlendSample = [evaluator](const SdfPath &blendShape,
                                       size_t pointCount,
                                       RigExecBlendSampleLayout *layout) {
        return evaluator->_ResolveBlendSampleLayout(blendShape, pointCount,
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
    ctx.reasons = reasons;
    // Every property a chain writes. An input resolving through one of these
    // cannot be captured, because the chain recomputes it every generation.
    for (const auto &[target, revisions] : E._propertyChains) {
        ctx.chainTargets.insert(target);
    }
    // Each ribbon's driver-points attribute, resolved once by the compiler.
    // Restated here because bakedPose.cpp cannot name the evaluator's
    // private map, and re-deriving the resolution (a prim target becomes its
    // .points, a property target is taken verbatim) would be a second
    // expression of a rule the compiler already applied.
    ctx.ribbonDriverPoints = E._ribbonDriverPoints;
    const UsdTimeCode capture = ctx.capture;
    const std::set<SdfPath> &chainTargets = ctx.chainTargets;
    // The bodies below were written against these as lambdas of this
    // function, and the bake functions in the other two files were split out
    // of the same bodies; naming them keeps both sides reading alike.
    auto refuse = [&](const std::string &what, const SdfPath &where) {
        ctx.Refuse(what, where);
    };
    auto fold = [&](const UsdPrim &prim, const char *name) {
        ctx.Fold(prim, name);
    };
    auto slotOf = [&](const SdfPath &path) { return ctx.SlotOf(path); };

    // Stage metadata, which arrives as a changed-info notice on the
    // pseudo-root: the up axis an aim constraint resolves against, and the
    // time-code range the input classification probes at.
    B.rebuild.insert(SdfPath::AbsoluteRootPath());

    // What a plain Xformable's transform is measured relative to, which is
    // the one prim rigEvaluator.cpp's pose walk measures against too -- and
    // the same asset root the adjuster publication composes against, set
    // once at the head of Build.
    B.assetRoot = B.stage->GetPrimAtPath(B.assetRootPath);

    phases.Next("Bake.dense_provider_slots");
    // ---- dense provider slots ---------------------------------------------
    //
    // The ordered union of the two families, which is the set the dynamic
    // walk's frame maps hold: the exec-seeded providers and the plain
    // Xformables a constraint targets. Both are already in SdfPath order, so
    // merging them through one ordered map keeps namespace DFS pre-order and
    // with it the "a parent has a lower slot" invariant the compose relies
    // on. The two sets are disjoint by construction (only what isFrameProvider
    // accepts is seeded); FirstFramePose wins a collision, because that is the kind
    // the compose can actually write.
    {
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
    // ---- the rest chain and the default-space ladder -----------------------
    //
    // computations.cpp resolves both per provider per evaluation through
    // eight computations. Every channel feeding them is BOUND here, not
    // folded: an ordinary rig binds fifteen constants per provider and the
    // frame path never reads one again, while a rig that animates, connects
    // or chain-writes a rest keeps the same fifteen bindings and recomposes
    // the ladder in the prologue. One resolution either way --
    // RigExecBakedComposeLadder, called once below and again per frame --
    // so the two cannot drift.
    B.posedAuthored.assign(size_t(N), 0);
    B.posedAuthoredM.assign(size_t(N), GfMatrix4d(1.0));
    B.restM.assign(N, GfMatrix4d(1.0));
    B.restPts.resize(N);
    B.restFrames.resize(N);
    B.selfD.assign(N, GfMatrix4d(1.0));
    B.parentDinv.assign(N, GfMatrix4d(1.0));
    B.rotOrder.assign(N, TfToken("XYZ"));
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
    // forced into one array -- each paired with the walk CommitBind needs,
    // same as _AvarResolution.
    struct _LadderResolution {
        UsdPrim prim;
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
    const auto resolveLadderSlots = [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            // An xform-derived slot binds no ladder -- its rest frame is
            // the identity outright, below -- so there is nothing here for
            // it to resolve.
            if (B.slotKind[i] != RigExecBakedSlotKind::FirstFramePose) {
                continue;
            }
            const UsdPrim prim = B.stage->GetPrimAtPath(B.paths[i]);
            _LadderResolution &out = ladderResolved[i];
            out.prim = prim;
            out.posedSpace = ctx.ResolveBind(prim, "posed:space",
                                             GfMatrix4d(1.0),
                                             &out.posedSpaceWalk);
            out.restSpace = ctx.ResolveBind(prim, "rest:space",
                                            GfMatrix4d(1.0),
                                            &out.restSpaceWalk);
            out.defaultSpace = ctx.ResolveBind(prim, "default:space",
                                               GfMatrix4d(1.0),
                                               &out.defaultSpaceWalk);
            out.rotationOrder = ctx.ResolveBind(prim, "avars:rotationOrder",
                                                TfToken("XYZ"),
                                                &out.rotationOrderWalk);
            for (int c = 0; c < 6; ++c) {
                out.restAvars[c] = ctx.ResolveBind(
                    prim, kRestAvars[c], 0.0, &out.restAvarWalks[c]);
                out.defaultAvars[c] = ctx.ResolveBind(
                    prim, kDefaultAvars[c], 0.0, &out.defaultAvarWalks[c]);
            }
        }
    };
    if (RigExecParallelEvaluationEnabled() && N > 1) {
        WorkParallelForN(size_t(N), resolveLadderSlots);
    } else {
        resolveLadderSlots(0, size_t(N));
    }
    for (int i = 0; i < N; ++i) {
        if (B.slotKind[size_t(i)] != RigExecBakedSlotKind::FirstFramePose) {
            // An xform-derived slot has no rest chain and no default-space
            // ladder: the dynamic path gives it the identity rest frame
            // outright and reads its pose off the stage.
            //
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
        const UsdPrim &prim = out.prim;
        B.prims.insert(B.paths[i]);
        // The space EXPRESSIONS stay FOLDED. Bakeability accepted them
        // because they are unauthored and unconnected, which is a statement
        // about the SHAPE of the compose -- authoring one replaces the
        // ladder rather than moving a value in it -- so an edit to one must
        // rebuild the program and a drag on one must fall back. Fold
        // mutates `program->rebuild`/`folded`/`named`/`prims`, so it stays
        // here, serial, rather than moving into the resolve pass above.
        for (const char *name : {"parent:space", "parent:defaultSpace",
                                 "avars:defaultSpace",
                                 "posed:defaultSpace"}) {
            fold(prim, name);
        }
        RigExecBakedProgramImpl::Ladder &ladder = B.ladders[size_t(i)];
        // Committing in slot order, and within a slot in the same
        // posedSpace/restSpace/defaultSpace/rotationOrder/avar sequence the
        // un-parallelized loop bound them in, is what makes a parallel
        // Build hand out the same override indices a serial one would --
        // CommitBind is where Register numbers them, see ResolveBind
        // /CommitBind above.
        ctx.CommitBind(prim, "posed:space", &out.posedSpace,
                       out.posedSpaceWalk);
        ladder.posedSpace = out.posedSpace;
        ctx.CommitBind(prim, "rest:space", &out.restSpace, out.restSpaceWalk);
        ladder.restSpace = out.restSpace;
        ctx.CommitBind(prim, "default:space", &out.defaultSpace,
                       out.defaultSpaceWalk);
        ladder.defaultSpace = out.defaultSpace;
        ctx.CommitBind(prim, "avars:rotationOrder", &out.rotationOrder,
                       out.rotationOrderWalk);
        ladder.rotationOrder = out.rotationOrder;
        for (int c = 0; c < 6; ++c) {
            ctx.CommitBind(prim, kRestAvars[c], &out.restAvars[c],
                           out.restAvarWalks[c]);
            ladder.restAvars[c] = out.restAvars[c];
            ctx.CommitBind(prim, kDefaultAvars[c], &out.defaultAvars[c],
                           out.defaultAvarWalks[c]);
            ladder.defaultAvars[c] = out.defaultAvars[c];
        }
    }
    // What a frame may have to re-resolve, and where a drag on one lands.
    // Both are asked of the WHOLE ladder rather than per provider: exec
    // re-pulls every rest the moment any one of them can move (rigEvaluator
    // .cpp's newRestsMightVary), and a chain's answer depends on its
    // ancestors in any case.
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
    // re-resolve, which is what makes the two agree by construction.
    RigExecBakedComposeLadder(&B, capture, /* trackMoves = */ false);
    // The comparison buffers the per-frame recompose dirties against, sized
    // only for a ladder that can actually move. Seeded with what Build just
    // composed: the first run dirties every pose cluster outright, so what
    // they hold on it is never read, and a later run that recomposes to the
    // same numbers correctly finds nothing moved.
    if (B.ladderVarying || !B.ladderOverrides.empty()) {
        B.lastRestM = B.restM;
        B.lastSelfD = B.selfD;
        B.lastParentDinv = B.parentDinv;
        B.lastPosedAuthored = B.posedAuthored;
        B.lastPosedAuthoredM = B.posedAuthoredM;
        B.lastRotOrder = B.rotOrder;
    }

    // The seed's two comparison buffers, sized once and never resized in a
    // run. Their first-run values are never read: the first run of a program
    // dirties every pose cluster outright, because it has nothing to compare
    // against.
    B.xformBase.assign(B.xformSlots.size(), GfMatrix4d(1.0));
    B.lastXformBase.assign(B.xformSlots.size(), GfMatrix4d(1.0));

    phases.Next("Bake.the_input_binding_table");
    // ---- the input binding table -------------------------------------------
    B.avarConstants.assign(size_t(N) * 11, 0.0);
    // Eleven channels on every provider, each an independent read of the
    // composed stage, and on a character this is one of the two loops Build
    // spends most of its time in. Resolved across threads and committed on
    // this one: the commit is what hands out override indices, so doing it
    // here in slot-then-channel order is what keeps the program a parallel
    // Build produces identical to a serial one. See ResolveBind/CommitBind.
    struct _AvarResolution {
        UsdPrim prim;
        RigExecBakedInput<double> input;
        SdfPathVector walk;
    };
    std::vector<_AvarResolution> resolved(size_t(N) * 11);
    const auto resolveSlots = [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            // An xform-derived slot binds no avars -- its pose is the
            // stage's -- so the bound/varying counts and the overridable set
            // stay exactly what the RigExec providers alone make them.
            if (B.slotKind[i] != RigExecBakedSlotKind::FirstFramePose) {
                continue;
            }
            const UsdPrim prim = B.stage->GetPrimAtPath(B.paths[i]);
            for (int c = 0; c < 11; ++c) {
                _AvarResolution &out = resolved[i * 11 + size_t(c)];
                out.prim = prim;
                out.input = ctx.ResolveBind(prim, RigExecBakedAvarNames[c],
                                            RigExecBakedAvarDefaults[c],
                                            &out.walk);
            }
        }
    };
    if (RigExecParallelEvaluationEnabled() && N > 1) {
        WorkParallelForN(size_t(N), resolveSlots);
    } else {
        resolveSlots(0, size_t(N));
    }
    for (int i = 0; i < N; ++i) {
        if (B.slotKind[size_t(i)] != RigExecBakedSlotKind::FirstFramePose) {
            continue;
        }
        for (int c = 0; c < 11; ++c) {
            const size_t slot = size_t(i) * 11 + size_t(c);
            _AvarResolution &out = resolved[slot];
            ctx.CommitBind(out.prim, RigExecBakedAvarNames[c], &out.input,
                           out.walk);
            B.avarConstants[slot] = out.input.constant;
            if (out.input.varying) {
                B.avarBindings.push_back({slot, std::move(out.input)});
            } else if (out.input.overrideIndex >= 0) {
                // Patchable only when the value came from the attribute
                // itself. A walk longer than one captured it upstream,
                // through a connection, and an edit on the head would not be
                // the value the slot holds.
                const SdfPath property = out.prim.GetPath().AppendProperty(
                    TfToken(RigExecBakedAvarNames[c]));
                if (out.walk.size() <= 1 && out.input.head &&
                    out.input.head.GetPath() == property) {
                    B.patchableAvars[property] =
                        B.avarConstantBindings.size();
                }
                B.avarConstantBindings.push_back({slot, std::move(out.input)});
            }
        }
    }
    B.avars = B.avarConstants;

    phases.Next("Bake.the_walk");
    // ---- the walk ------------------------------------------------------------
    //
    // Restated as plain records, because _PoseStep, _SolverBatch and
    // _FrameConstraint are private to the evaluator and bakedPose.cpp is not
    // its friend. The restatement is a copy of the structure only: every
    // VALUE the bake reads still comes off the stage, through the context.
    // Whether a read phase asked for \p target's value as of \p mover. The
    // dynamic path asks this per call, inside recordFrame and beside every
    // chain revision; the program asks it once, here.
    const auto snapshotAfter = [&E](const SdfPath &target,
                                    const SdfPath &mover) {
        const auto wanted = E._snapshotPoints.find(target);
        return wanted != E._snapshotPoints.end() &&
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
                // A cycle among unbatched solvers. The compile's own Kahn
                // pass would have refused to schedule one, so this cannot
                // happen; taking the rest in path order rather than looping
                // for ever is what an assertion would cost anyway.
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
    // ---- publication ---------------------------------------------------------
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
    // ---- geometry ------------------------------------------------------------
    //
    // Restated for the same reason the walk was: _GraphRevision is private to
    // the evaluator.
    phases.Next("Bake.pose_interpolators");
    // ---- pose interpolators -------------------------------------------------
    //
    // Before the geometry, which looks the weights up by property path to
    // wire a connected blend channel to its slot. The solved table is COPIED
    // from the compiled record: it is a constant of the epoch, and the
    // program is dropped with the epoch.
    for (const RigExecRigEvaluator::_PoseInterpolator &record :
             E._poseInterpolators) {
        RigExecBakedProgramImpl::PoseInterpolator out;
        out.path = record.prim;
        out.driverSlot = ctx.SlotOf(record.driver);
        if (out.driverSlot < 0) {
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
        const UsdPrim prim = B.stage->GetPrimAtPath(record.prim);
        // The interpolator's structure -- its driver, its poses, their
        // rotations and radii -- is epoch identity and recompiles; what a
        // frame reads is the one enable, bound like any other input so an
        // animated or dragged one is honoured and a value edit needs no
        // rebuild.
        B.prims.insert(record.prim);
        B.resolvedRoutedPrims.insert(record.prim);
        out.enabled = ctx.Bind<bool>(prim, "inputs:enabled", true);
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

    std::vector<RigExecBakedChainSpec> chainSpecs;
    const auto revisionSpec =
        [&snapshotAfter](const RigExecRigEvaluator::_GraphRevision &r) {
            RigExecBakedRevisionSpec spec;
            spec.moverPath = r.moverPath;
            spec.target = r.target;
            spec.op = r.op;
            spec.binding = r.binding;
            spec.transformFinalPhase = r.transformFinalPhase;
            spec.skinTopologyFixed = r.skinTopologyFixed;
            spec.snapshotAfter = snapshotAfter(r.target, r.moverPath);
            return spec;
        };
    for (const SdfPath &target : E._chainOrder) {
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
    // ---- the rest of the invalidation index ---------------------------------
    //
    // Property chains run INSIDE the program, off the authored stage through
    // the generation's resolved inputs, so their movers are read live like a
    // geometry mover: nothing captured, and an override on one places itself.
    for (const auto &[target, revisions] : E._propertyChains) {
        B.prims.insert(target.GetPrimPath());
        B.resolvedRoutedPrims.insert(target.GetPrimPath());
        for (const RigExecRigEvaluator::_PropertyRevision &revision :
                 revisions) {
            B.prims.insert(revision.moverPath);
            B.resolvedRoutedPrims.insert(revision.moverPath);
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
    B.base.resize(N);
    B.fin.resize(N);
    if (!ctx.ok) {
        return nullptr;
    }

    phases.Next("Bake.the_step_graph");
    // ---- the step graph -----------------------------------------------------
    //
    // Last, because it is derived from everything above: the steps are the
    // straight line's pieces in the straight line's order, and the edges
    // between them follow from the slot ranges each piece declares. Built
    // once here, so that a frame costs the graph nothing.
    RigExecBakedBuildPoseSteps(&B);
    // Between the two halves, which is where a weight object belongs in
    // program order: its placement comes from the pose walk and its packet is
    // what a revision assembles against.
    RigExecBakedBuildWeightSteps(&B);
    // The pose half's edges and levels, settled BEFORE the geometry half is
    // built. A skin revision is cut into chunks only where the chunks' joints
    // land at different levels (§6), and that question cannot be asked until
    // the ProviderMatrix steps have levels -- so the sweep runs here, over
    // the pose steps alone, and again inside RigExecBakedBuildSchedule once
    // the geometry steps exist. A geometry step never precedes a pose step,
    // so the levels this pass assigns are the levels the final graph holds.
    RigExecBakedBuildStepEdges(&B);
    RigExecBakedAssignStepCosts(&B);
    // The levels the partition is about to read, kept so that the sentence
    // above is CHECKED and not merely written down. Nothing else can catch
    // its violation: the cut decision and the record of the cut decision
    // (GeomRevision::partitionReady*) are derived from one another, so a
    // partition cut from levels the final graph no longer holds still agrees
    // with itself and tests/testRigExecBakedSchedule still passes. What
    // would have moved is this vector -- a pose step given a geometry
    // predecessor is a pose step pushed down a level -- so it is compared
    // after the second sweep instead.
    std::vector<int> poseLevels;
    poseLevels.reserve(B.steps.size());
    for (const RigExecBakedStep &step : B.steps) {
        poseLevels.push_back(step.level);
    }
    RigExecBakedBuildGeometrySteps(&B);
    RigExecBakedBuildSchedule(&B);
    for (size_t i = 0; i < poseLevels.size(); ++i) {
        if (!TF_VERIFY(B.steps[i].level == poseLevels[i],
                       "rigExec: pose step %zu (%s) moved from level %d to "
                       "%d when the geometry steps were added; the vertex "
                       "partition was cut from the level it no longer has",
                       i, RigExecBakedStepKindName(B.steps[i].kind),
                       poseLevels[i], B.steps[i].level)) {
            break;
        }
    }
    if (RigExecBakedScheduleReportRequested()) {
        const std::string report = RigExecBakedScheduleReport(B);
        std::fwrite(report.data(), 1, report.size(), stderr);
    }
    return std::unique_ptr<RigExecBakedProgram>(
        new RigExecBakedProgram(std::move(impl)));
}

// ---------------------------------------------------------------------------
// One frame.
//
// The prologue is here rather than in bakedPose.cpp because it is the one
// part of a frame that calls the evaluator's own private routines; everything
// it produces reaches the two halves through the program.
// ---------------------------------------------------------------------------

bool
RigExecBakedProgram::Run(UsdTimeCode time, RigExecRigPose *pose)
{
    RigExecBakedProgramImpl &B = *_impl;
    RigExecRigEvaluator &E = *B.evaluator;
    RIGEXEC_PROFILE_SCOPE_CAT(*B.profiler, "Baked", "baked");

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
    //
    // A step may not touch USD, so the stage-derived frames the pose walk
    // uses are settled in the prologue: the plain Xformables a constraint
    // targets, seeded into their slots' FIRST version, which is where the
    // compose would have left one. The cache is built FRESH for this frame,
    // exactly as _EvaluateDynamic builds its own: a retained
    // UsdGeomXformCache keeps a UsdGeomXformQuery per prim across SetTime,
    // so an xformOp added or reordered would leave a stale query where the
    // dynamic path has none.
    //
    // False means a target's transform could not be resolved at all, which
    // is the one thing the dynamic walk gives the generation back for here.
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
    //
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
                arrays.prim, "inputs:sourceWeights", arrays.sourceCount, time,
                &arrays.diagnostics, &arrays.weights);
            // The dynamic walk stops at the first table it cannot use, so
            // the offsets are not read when the weights were malformed --
            // and a run that read them anyway could produce a second line
            // the reference generation never produced.
            if (arrays.parentOffsets) {
                arrays.ok =
                    arrays.ok &&
                    RigExecRigEvaluator::_ReadConstraintSourceOffsets(
                        arrays.prim, "inputs:translationOffsets",
                        arrays.sourceCount, time, &arrays.diagnostics,
                        &arrays.translationOffsets) &&
                    RigExecRigEvaluator::_ReadConstraintSourceOffsets(
                        arrays.prim, "inputs:rotationOffsets",
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
                        arrays.prim, "inputs:poleVectorWeights",
                        arrays.poleCount, time, &arrays.poleDiagnostics,
                        &arrays.poleWeights);
            }
        }
    };

    // ---- prologue -----------------------------------------------------------
    //
    // Serial, always run, and the only part of a frame that may take a lock,
    // read the stage through anything but a pinned query, or touch the pose
    // as it goes. Everything the region needs from outside itself is settled
    // here.
    {
        RIGEXEC_PROFILE_SCOPE_CAT(*B.profiler, "BakedPrologue", "baked");
        // A math mover's inputs are all authored on itself, so its chain owes
        // exec nothing and resolves first -- which is why this op can be the
        // same routine the dynamic path runs rather than a second copy of it.
        B.propertyResults.clear();
        // The geometry-domain constraint deltas are NOT emptied here. They
        // are slots: what a skipped constraint step left is what it would
        // have measured again, and clearing them would make a cone that
        // skips one deform as though no constraint had ever measured a
        // delta. That was the hazard §7 named; keeping them across runs the
        // way a slot is kept is the answer it offered.
        B.resolvedInputs->Clear();
        B.runSnapshots.Clear();
        B.chainSnapshots->Clear();
        // Interactive overrides are applied on BOTH sides of the property
        // chains, for the reason _EvaluateDynamic gives at the same two
        // points: an override can be either end of a chain and the two ends
        // want opposite orderings, and which end a given one is at is not
        // knowable here.
        const bool dragging = !B.interactiveOverrides->empty();
        if (dragging) {
            E._ApplyInteractiveOverridesToResolved(B.resolvedInputs, nullptr);
        }
        if (B.hasPropertyChains) {
            RIGEXEC_PROFILE_SCOPE_CAT(*B.profiler, "PropertyChains",
                                      "property");
            std::vector<RigExecValueOverride> overrides;
            // Straight into the published diagnostics: the chains run before
            // anything else on both paths, so their lines are the first of
            // the generation and nothing has to replay them.
            E._EvaluatePropertyChains(time, &B.propertyResults, &overrides,
                                      &pose->diagnostics);
            for (const auto &[path, value] : B.propertyResults) {
                B.resolvedInputs->SetProperty(path, value);
            }
        }
        if (dragging) {
            E._ApplyInteractiveOverridesToResolved(B.resolvedInputs,
                                                   &B.propertyResults);
        }
        RigExecBakedRunInputs(&B, time);
        RigExecBakedRunSolverSources(&B, time);
        stageFramesOk = stageFrames();
        constraintArrays();
        RigExecBakedRunGeometryPrologue(&B, time, pose);
    }
    if (!stageFramesOk) {
        // The dynamic walk gives the generation back at the same point and
        // pushes the same line; the caller drops this pose and re-runs it
        // there, so the diagnostic above is its rehearsal and not a second
        // copy.
        return false;
    }
    if (measuring) {
        const double mark = now();
        prologueUs = mark - phaseMark;
        phaseMark = mark;
    }

    // ---- the region ---------------------------------------------------------
    //
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
        bailed = !RigExecBakedRunSteps(&B, time);
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

    // ---- epilogue -----------------------------------------------------------
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
        // A step gave the generation back. Before the curvenet drain, which
        // is destructive: a frame that gave up must not spend the pending
        // bind lines into a pose nobody publishes. The caller drops the
        // program, cache and all, and the dynamic fallback emits its own
        // lines because it re-resolves every bind against the evaluator's
        // still-empty cache.
        return false;
    }
    if (!RigExecBakedPublishPose(&B, pose)) {
        return false;  // the dynamic fallback needs exec
    }
    // Where every volume weight ended up, as the VolumePlacements step left
    // it. The dynamic walk assigns the pose from the same map at the same
    // point -- the last refresh before anything reads it -- and a run whose
    // cone skipped the step is a run in which no volume's final frame moved,
    // so the map it kept is still this generation's.
    if (B.volumeWeightMatrices) {
        pose->weightFrames = *B.volumeWeightMatrices;
    }
    RigExecBakedPublishGeometry(&B, time, pose);

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

    // Whatever the Profile Mover binds reported; drained so a cached bind
    // stays silent on every later frame. Empty on a bakeable rig, drained
    // anyway so a rig that starts binding reports the same lines the dynamic
    // path does. It sits here, at the tail of the run and past every return
    // above it: the position is the one the dynamic walk drains from --
    // immediately before its summary line -- so a completed frame's
    // diagnostics interleave identically.
    for (std::string &message : B.curvenetBindings.TakeDiagnostics()) {
        pose->diagnostics.push_back(std::move(message));
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

}  // namespace rigExec
