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
#include "rigEvaluator.h"
#include "types.h"

#include "rigExecMath/pointFrame.h"

#include "pxr/base/gf/math.h"
#include "pxr/base/gf/rotation.h"
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
#include <cmath>
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
           type == "RigExecAimConstraint";
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
           type == "RigExecBlendPointFrames" || type == "RigExecSplineIk";
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

}  // namespace

// ---------------------------------------------------------------------------
// Bakeability.
// ---------------------------------------------------------------------------

bool
RigExecBakedProgram::IsBakeable(const RigExecRigEvaluator &evaluator,
                               std::vector<std::string> *reasons)
{
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
    for (const SdfPath &path : E._xformDerivedProviders) {
        say("constraint target is a plain Xformable", path);
    }
    for (const auto &[path, tap] : E._volumeWeightMatrixTaps) {
        say("volume weight object", path);
    }
    for (const SdfPath &path : E._currentPhaseWeights) {
        say("current-phase volume weight", path);
    }
    for (const auto &[path, movers] : E._snapshotPoints) {
        say("read-phase snapshot required on", path);
    }
    for (const auto &[path, points] : E._ribbonDriverPoints) {
        say("ribbon driver curve", path);
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
    for (const auto &[path, tap] : E._poseSeedFrames) {
        const UsdPrim prim = E._stage->GetPrimAtPath(path);
        if (!prim) {
            say("pose provider has no prim", path);
            continue;
        }
        const TfToken type = prim.GetTypeName();
        if (type != "RigExecJoint" && type != "RigExecControl") {
            say("provider type not baked (" + type.GetString() + ")", path);
        }
        // A space expression the compose cannot express: the program builds
        // the default-space ladder from rest + default avars and follows the
        // namespace parent, which is exactly what exec does only while these
        // stay unauthored and unconnected.
        for (const char *name : {"posed:space", "parent:space",
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
        // The ladder is resolved once, so anything feeding it must be still.
        for (const char *name : {"rest:space", "rest:tx", "rest:ty", "rest:tz",
                                 "rest:rx", "rest:ry", "rest:rz",
                                 "default:space", "default:tx", "default:ty",
                                 "default:tz", "default:rx", "default:ry",
                                 "default:rz", "avars:rotationOrder"}) {
            if (const UsdAttribute a = prim.GetAttribute(TfToken(name))) {
                if (RigExecBakedAnimatedOrConnected(a)) {
                    say(std::string("animated or connected ") + name +
                            " on provider",
                        path);
                }
            }
            if (chainTargets.count(path.AppendProperty(TfToken(name)))) {
                say(std::string("property chain writes ") + name +
                        " on provider",
                    path);
            }
        }
        (void)probe;
    }
    for (const SdfPath &joint : E._jointPaths) {
        if (!E._poseSeedFrames.count(joint)) {
            say("joint is not a seeded pose provider", joint);
        }
    }
    for (const SdfPath &control : E._controlPaths) {
        if (!E._poseSeedFrames.count(control)) {
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
                continue;
            }
            if (type == "RigExecTwoBoneIk") {
                // Bone lengths are MEASURED from the rests of the joints the
                // solver names; without all three there is nothing to bake.
                //
                // Resolved the way bakeSolver resolves it, so the two do not
                // drift: one rest per rigExec:joints target that publishes
                // computeRestFrame, and the remap indexed by position within
                // THAT list. A cardinality mismatch is deliberately not a
                // refusal here -- exec's computation warns and publishes an
                // empty aggregate, and the bake reproduces that through
                // Solver::degenerate rather than declining the rig.
                std::array<bool, 3> seen{false, false, false};
                std::vector<SdfPath> rests;
                for (const SdfPath &joint : _Targets(prim, "rigExec:joints")) {
                    if (E._poseSeedFrames.count(joint)) {
                        rests.push_back(joint);
                    }
                }
                VtIntArray elements;
                if (const UsdAttribute a = prim.GetAttribute(
                        TfToken("rigExec:jointElements"))) {
                    a.Get(&elements);
                }
                for (size_t k = 0; k < rests.size(); ++k) {
                    const int element =
                        elements.size() == rests.size() ? elements[k] : int(k);
                    if (element >= 0 && element < 3) seen[element] = true;
                }
                if (!(seen[0] && seen[1] && seen[2])) {
                    say("TwoBoneIk does not bind three joint rests",
                        solverPath);
                }
            }
        }
    }
    for (const auto &[solverPath, tap] : E._solverArrayTaps) {
        if (!batched.count(solverPath)) {
            say("solver publishes guides but is in no batch", solverPath);
        }
    }

    // ---- constraints -------------------------------------------------------
    for (const auto &constraint : E._frameConstraints) {
        if (!constraint.pointsTarget.IsEmpty()) {
            say("geometry-domain constraint", constraint.moverPath);
        }
        if (constraint.schemaType == "RigExecSingleChainIkConstraint") {
            say("SingleChainIK constraint", constraint.moverPath);
            continue;
        }
        if (!_IsBakedConstraintType(constraint.schemaType)) {
            say("constraint type not baked (" +
                    constraint.schemaType.GetString() + ")",
                constraint.moverPath);
            continue;
        }
        if (!constraint.weightObject.IsEmpty()) {
            say("weight object on constraint", constraint.moverPath);
        }
        if (constraint.targets.size() != 1) {
            say("constraint does not write exactly one target",
                constraint.moverPath);
            continue;
        }
        if (!E._poseSeedFrames.count(constraint.targets[0])) {
            say("constraint target is not a seeded pose provider",
                constraint.targets[0]);
        }
        for (const auto &source : constraint.sources) {
            if (!source.xformPath.IsEmpty()) {
                say("native Xformable constraint source", source.sourcePath);
            }
        }
        if (!constraint.worldUpObject.xformPath.IsEmpty()) {
            say("native Xformable world-up object",
                constraint.worldUpObject.sourcePath);
        }
    }

    // ---- geometry ----------------------------------------------------------
    auto checkRevision = [&](const RigExecRigEvaluator::_GraphRevision &r,
                             bool derived) {
        const bool supported =
            derived ? (r.op == RigExecRevisionOp::RecomputeExtent ||
                       r.op == RigExecRevisionOp::RecomputeNormals)
                    : (r.op == RigExecRevisionOp::Skin ||
                       r.op == RigExecRevisionOp::Matrix);
        if (!supported) {
            say(std::string("mover operation not baked (") +
                    RigExecBakedOpName(r.op) +
                    ")",
                r.moverPath);
            return;
        }
        if (!r.binding.weightObject.IsEmpty()) {
            say("weight object on mover", r.moverPath);
        }
        if (!r.binding.blendInputs.empty()) {
            say("blend shape inputs on mover", r.moverPath);
        }
        if (!r.binding.phases.empty()) {
            say("read phase on a mover input", r.moverPath);
        }
        if (!r.binding.curvenetPoints.IsEmpty() ||
            !r.binding.curvenet.IsEmpty()) {
            say("curvenet on mover", r.moverPath);
        }
        if (r.driverFramesTap >= 0) {
            say("driver frames on mover", r.moverPath);
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
                if (!E._poseSeedFrames.count(influence)) {
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
        if (!keepRun) {
            return;
        }
        destination->resultStatus = source->resultStatus;
        destination->output = std::move(source->output);
        destination->lastParameters = std::move(source->lastParameters);
        destination->lastStatus = source->lastStatus;
        destination->ran = source->ran;
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
    // The weight domains, which nothing bakeable fills TODAY -- every
    // feature that publishes one still refuses the bake, so both maps are
    // empty on both paths and these two calls cost a pair of empty walks.
    // They are here anyway, and ahead of the operators that will fill them:
    // the day a weight object bakes, its very first generation is measured
    // against the dynamic path instead of against a comparator that was
    // never taught to look. A field is equal iff it weights the same
    // property with the same floats, bit for bit -- a resolved field is what
    // a mover actually consumed, and an element one path clamped and the
    // other did not is exactly the difference a size check cannot see.
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
    if (!evaluator || !IsBakeable(*evaluator, reasons)) {
        return nullptr;
    }
    RigExecRigEvaluator &E = *evaluator;
    auto impl = std::make_unique<RigExecBakedProgramImpl>();
    RigExecBakedProgramImpl &B = *impl;
    B.evaluator = evaluator;
    B.stage = E._stage;
    // The evaluator state a frame reads, captured here because this is the
    // only translation unit its friendship reaches; bakedProgramImpl.h says
    // why each one is a pointer rather than a copy.
    B.resolvedInputs = &E._resolvedInputs;
    B.chainSnapshots = &E._chainSnapshots;
    B.skinTopologies = &E._skinTopologies;
    B.profiler = &E._profiler;
    B.interactiveOverrides = &E._interactiveOverrides;
    B.jointSolverBinding = &E._jointSolverBinding;
    B.guideTaps = &E._guideTaps;
    B.solverGuidesEnabled = &E._solverGuidesEnabled;
    B.hasPropertyChains = !E._propertyChains.empty();

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
    auto bind = [&](const UsdPrim &prim, const char *name, auto fallback) {
        return ctx.Bind(prim, name, fallback);
    };
    auto slotOf = [&](const SdfPath &path) { return ctx.SlotOf(path); };

    // Stage metadata, which arrives as a changed-info notice on the
    // pseudo-root: the up axis an aim constraint resolves against, and the
    // time-code range the input classification probes at.
    B.rebuild.insert(SdfPath::AbsoluteRootPath());

    // ---- dense provider slots ---------------------------------------------
    //
    // The ordered union of the two families, which is the set the dynamic
    // walk's frame maps hold: the exec-seeded providers and the plain
    // Xformables a constraint targets. Both are already in SdfPath order, so
    // merging them through one ordered map keeps namespace DFS pre-order and
    // with it the "a parent has a lower slot" invariant the compose relies
    // on. The two sets are disjoint by construction (only what isFrameProvider
    // accepts is seeded); PoseSeed wins a collision, because that is the kind
    // the compose can actually write.
    {
        std::map<SdfPath, RigExecBakedSlotKind> ordered;
        for (const auto &[path, tap] : E._poseSeedFrames) {
            ordered.emplace(path, RigExecBakedSlotKind::PoseSeed);
        }
        for (const SdfPath &path : E._xformDerivedProviders) {
            ordered.emplace(path, RigExecBakedSlotKind::XformDerived);
        }
        for (const auto &[path, kind] : ordered) {
            B.index[path] = int(B.paths.size());
            B.paths.push_back(path);
            B.slotKind.push_back(kind);
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
                RigExecBakedSlotKind::PoseSeed) {
                B.parent[i] = it->second;
                break;
            }
        }
        if (B.parent[i] >= i) {
            refuse("provider slots are not in namespace DFS order",
                   B.paths[i]);
        }
    }

    // ---- the rest chain and the default-space ladder -----------------------
    //
    // computations.cpp resolves both per provider per evaluation through
    // eight computations; every input to them is epoch-constant on a bakeable
    // rig, so both resolve once here. The frame round trips exec performs
    // between computations are reproduced, not simplified away.
    B.restM.assign(N, GfMatrix4d(1.0));
    B.restPts.resize(N);
    B.restFrames.resize(N);
    B.selfD.assign(N, GfMatrix4d(1.0));
    B.parentDinv.assign(N, GfMatrix4d(1.0));
    B.rotOrder.assign(N, TfToken("XYZ"));
    std::vector<GfMatrix4d> restRoundTrip(N, GfMatrix4d(1.0));
    std::vector<GfMatrix4d> defaultRoundTrip(N, GfMatrix4d(1.0));
    for (int i = 0; i < N; ++i) {
        if (B.slotKind[size_t(i)] != RigExecBakedSlotKind::PoseSeed) {
            // An xform-derived slot has no rest chain and no default-space
            // ladder: the dynamic path gives it the identity rest frame
            // outright and reads its pose off the stage.
            //
            // SI-6, invalidation-index coverage: skipping the block below
            // also skips its B.prims.insert, so an xform-derived prim is NOT
            // in the invalidation index and IsInvalidatedBy cannot answer a
            // notice on it. That is right only while nothing reads such a
            // slot's value -- today nothing writes one either, and IsBakeable
            // refuses the rig outright. The feature that starts seeding them
            // from the stage must insert the prim (and the ancestors whose
            // transforms it composes) here, or an edit to the Xform will not
            // rebuild the program.
            B.restFrames[i] = RigExecFrameFromMatrix(GfMatrix4d(1.0));
            B.restPts[i] = B.restFrames[i].points;
            continue;
        }
        const UsdPrim prim = B.stage->GetPrimAtPath(B.paths[i]);
        B.prims.insert(B.paths[i]);
        // The ladder is resolved once and folded into restM/selfD, so every
        // attribute feeding it is a captured constant -- including the space
        // EXPRESSIONS, which bakeability accepted because they are unauthored
        // and would change the compose if they stopped being so.
        for (const char *name : {"posed:space", "parent:space",
                                 "parent:defaultSpace", "avars:defaultSpace",
                                 "posed:defaultSpace", "avars:rotationOrder"}) {
            fold(prim, name);
        }
        auto number = [&](const char *name, double fallback) {
            SdfPathVector walk;
            const RigExecBakedInput<double> input = RigExecBakedBindInput(
                prim, name, fallback, capture, chainTargets, &walk);
            fold(prim, name);
            B.rebuild.insert(walk.begin(), walk.end());
            B.folded.insert(walk.begin(), walk.end());
            return RigExecBakedRead(input, E._resolvedInputs, capture);
        };
        GfMatrix4d space(1.0);
        fold(prim, "rest:space");
        if (const UsdAttribute a = prim.GetAttribute(TfToken("rest:space"))) {
            a.Get(&space);
        }
        GfMatrix4d rest = RigExecBakedComposeAvars(
            number("rest:tx", 0), number("rest:ty", 0), number("rest:tz", 0),
            1, 1, 1, number("rest:rx", 0), number("rest:ry", 0),
            number("rest:rz", 0), 0, TfToken("XYZ")) * space;
        rest.Orthonormalize(/* issueWarning = */ false);
        const GfMatrix4d parentRest =
            B.parent[i] >= 0 ? restRoundTrip[B.parent[i]] : GfMatrix4d(1.0);
        B.restM[i] = rest * parentRest;
        B.restFrames[i] = RigExecFrameFromMatrix(B.restM[i]);
        B.restPts[i] = B.restFrames[i].points;
        restRoundTrip[i] = RigExecBakedRoundTrip(B.restM[i]);

        // default:space is a space EXPRESSION: a non-identity authored value
        // wins, otherwise the computed ladder.
        GfMatrix4d authoredDefault(1.0);
        bool haveAuthored = false;
        fold(prim, "default:space");
        if (const UsdAttribute a =
                prim.GetAttribute(TfToken("default:space"))) {
            a.Get(&authoredDefault);
            haveAuthored = authoredDefault != GfMatrix4d(1.0);
        }
        const GfMatrix4d parentDefault =
            B.parent[i] >= 0 ? defaultRoundTrip[B.parent[i]]
                             : GfMatrix4d(1.0);
        if (haveAuthored) {
            B.selfD[i] = authoredDefault;
        } else {
            const GfMatrix4d offset = RigExecBakedComposeAvars(
                number("default:tx", 0), number("default:ty", 0),
                number("default:tz", 0), 1, 1, 1, number("default:rx", 0),
                number("default:ry", 0), number("default:rz", 0), 0,
                TfToken("XYZ"));
            B.selfD[i] = offset * restRoundTrip[i] * parentRest.GetInverse() *
                         parentDefault;
        }
        defaultRoundTrip[i] = RigExecBakedRoundTrip(B.selfD[i]);
        B.parentDinv[i] = parentDefault.GetInverse();
        if (const UsdAttribute a =
                prim.GetAttribute(TfToken("avars:rotationOrder"))) {
            TfToken order;
            if (a.Get(&order) && !order.IsEmpty()) {
                B.rotOrder[i] = order;
            }
        }
    }

    // ---- the input binding table -------------------------------------------
    B.avarConstants.assign(size_t(N) * 11, 0.0);
    for (int i = 0; i < N; ++i) {
        // An xform-derived slot binds no avars -- its pose is the stage's --
        // so the bound/varying counts and the overridable set stay exactly
        // what the RigExec providers alone make them.
        if (B.slotKind[size_t(i)] != RigExecBakedSlotKind::PoseSeed) {
            continue;
        }
        const UsdPrim prim = B.stage->GetPrimAtPath(B.paths[i]);
        for (int c = 0; c < 11; ++c) {
            const size_t slot = size_t(i) * 11 + size_t(c);
            RigExecBakedInput<double> input =
                bind(prim, RigExecBakedAvarNames[c],
                     RigExecBakedAvarDefaults[c]);
            B.avarConstants[slot] = input.constant;
            if (input.varying) {
                B.avarBindings.push_back({slot, std::move(input)});
            } else if (input.overrideIndex >= 0) {
                B.avarConstantBindings.push_back({slot, std::move(input)});
            }
        }
    }
    B.avars = B.avarConstants;

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
            entry.constraint.snapshotAfter =
                !fc.targets.empty() &&
                snapshotAfter(fc.targets[0], fc.moverPath);
            for (const auto &source : fc.sources) {
                entry.constraint.sources.push_back(source.sourcePath);
            }
            entry.constraint.worldUpObject = fc.worldUpObject.sourcePath;
        }
        walk.push_back(std::move(entry));
    }
    RigExecBakedBuildWalk(&ctx, walk);

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
        std::sort(order->begin(), order->end(), [&](int a, int b) {
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

    // ---- geometry ------------------------------------------------------------
    //
    // Restated for the same reason the walk was: _GraphRevision is private to
    // the evaluator.
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

    // ---- the step graph -----------------------------------------------------
    //
    // Last, because it is derived from everything above: the steps are the
    // straight line's pieces in the straight line's order, and the edges
    // between them follow from the slot ranges each piece declares. Built
    // once here, so that a frame costs the graph nothing.
    RigExecBakedBuildPoseSteps(&B);
    RigExecBakedBuildGeometrySteps(&B);
    RigExecBakedBuildSchedule(&B);
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
        // RUN-LOCAL, and therefore a cone hazard the group that lands the
        // writer has to answer (§7): this map is emptied here and filled by
        // the pose walk's geometry-domain constraint, which does not exist
        // yet. A run that SKIPS that constraint would leave the entry
        // missing rather than leaving last run's value in it, and the
        // assemble that find-guards it would deform as though no constraint
        // had ever measured a delta. The two answers are to keep the map
        // across runs the way a slot is kept, or to force a full run the way
        // `phasedReads` does for the snapshot store. Whichever, it is a
        // decision, not something to leave to the first frame that skips.
        B.constraintDeltas.clear();
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
        RigExecBakedRunGeometryPrologue(&B, time, pose);
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
        const bool bailedFull = !RigExecBakedRunSteps(&B, time, true);
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
