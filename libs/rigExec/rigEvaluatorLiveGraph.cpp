// RigExecRigEvaluator::DescribeLiveOperations: the compiled epoch as one
// diagnostic operation graph. Reads only; evaluation never calls it.
#include "rigExec/rigEvaluator.h"

#include "rigExec/bakedProgramImpl.h"

#include <algorithm>
#include <set>
#include <tuple>
#include <utility>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {
namespace {

std::string
_PathStr(const SdfPath &path)
{
    return path.GetString();
}

std::string
_BatchScope(size_t index, size_t level)
{
    return "SolverBatch L" + std::to_string(level) + " B" +
        std::to_string(index);
}

std::string
_ConstraintScope(const TfToken &type, const SdfPath &mover)
{
    return type.GetString() + " " + mover.GetString();
}

std::string
_AssembleScope(const SdfPath &mover, const SdfPath &target)
{
    return "Assemble " + mover.GetString() + " " + target.GetString();
}

std::string
_PropChainScope(const SdfPath &target)
{
    return "PropertyChain " + target.GetString();
}

void
_NotePath(std::map<std::string, std::string> *details, const char *key,
          const SdfPath &path)
{
    if (!path.IsEmpty()) {
        (*details)[key] = path.GetString();
    }
}

}  // namespace

RigExecLiveOperationGraph
RigExecRigEvaluator::DescribeLiveOperations() const
{
    RigExecLiveOperationGraph graph;
    std::set<std::tuple<std::string, std::string, std::string>> seenEdges;
    auto edge = [&](const std::string &src, const std::string &dst,
                    const char *kind) {
        if (src.empty() || dst.empty() || src == dst) {
            return;
        }
        if (seenEdges.emplace(src, dst, kind).second) {
            graph.edges.push_back({src, dst, kind});
        }
    };

    // A tap node id, or "" when the owning set does not stand or holds
    // no such tap: edges never dangle at a deferred or leader-less set.
    auto tapNode = [&](const std::string &set, RigExecTapId tap) {
        if (tap < 0) {
            return std::string();
        }
        const RigExecTapSet *taps = nullptr;
        if (set == "main") {
            taps = _taps.get();
        } else if (set == "rest") {
            taps = _restTaps.get();
        } else if (set == "first") {
            taps = _firstFramePoseTaps.get();
        } else if (set == "guide") {
            taps = _guideTaps.get();
        } else if (set.rfind("batch", 0) == 0) {
            size_t b = 0;
            try {
                b = std::stoul(set.substr(5));
            } catch (const std::exception &) {
                return std::string();
            }
            if (b < _solverBatches.size()) {
                taps = _solverBatches[b].taps.get();
            }
        }
        if (!taps || size_t(tap) >= taps->GetTapCount()) {
            return std::string();
        }
        return "tap:" + set + ":" + std::to_string(tap);
    };

    // Indexable lookups, built first so edges resolve against nodes that
    // are appended later.
    std::set<std::string> providers, joints, controls;
    for (const SdfPath &p : _providerPaths) {
        providers.insert(p.GetString());
    }
    for (const SdfPath &p : _jointPaths) {
        joints.insert(p.GetString());
    }
    for (const SdfPath &p : _controlPaths) {
        controls.insert(p.GetString());
    }
    auto providerId = [&](const SdfPath &p) {
        return providers.count(p.GetString()) ? "provider:" + p.GetString()
                                              : std::string();
    };
    std::map<std::string, std::string> moverConstraint;
    for (const _FrameConstraint &c : _frameConstraints) {
        moverConstraint[c.moverPath.GetString()] =
            "constraint:" + c.moverPath.GetString();
    }
    std::map<std::string, std::vector<std::string>> moverRevisions;
    // A derived chain (extent and the like) shares its points chain's
    // target, so it takes its own prefixes.
    auto revId = [&](const SdfPath &target, size_t i,
                     const char *prefix = "rev:") {
        return prefix + target.GetString() + "@" + std::to_string(i);
    };
    for (const auto &[target, revisions] : _graphChains) {
        for (size_t i = 0; i < revisions.size(); ++i) {
            moverRevisions[revisions[i].moverPath.GetString()].push_back(
                revId(target, i));
        }
    }
    for (const auto &[target, revisions] : _graphDerivedChains) {
        for (size_t i = 0; i < revisions.size(); ++i) {
            moverRevisions[revisions[i].moverPath.GetString()].push_back(
                revId(target, i, "drev:"));
        }
    }
    // A property mover is a revision of its own chain, and a phased reader
    // like any other.
    for (const auto &[target, revisions] : _propertyChains) {
        for (size_t i = 0; i < revisions.size(); ++i) {
            moverRevisions[revisions[i].moverPath.GetString()].push_back(
                "proprev:" + target.GetString() + "@" + std::to_string(i));
        }
    }
    std::map<std::string, std::string> solverNode;
    for (const auto &[solver, batches] : _solverJoints) {
        solverNode[solver.GetString()] = "solver:" + solver.GetString();
    }
    for (const auto &[solver, deps] : _solverDependencies) {
        solverNode[solver.GetString()] = "solver:" + solver.GetString();
    }
    // A consumer property resolves to the op node of its owning prim: a
    // constraint, a revision (possibly several, across chains), or a solver.
    auto consumerNodes = [&](const SdfPath &consumer) {
        std::vector<std::string> out;
        const std::string prim = consumer.GetPrimPath().GetString();
        const auto c = moverConstraint.find(prim);
        if (c != moverConstraint.end()) {
            out.push_back(c->second);
        }
        const auto r = moverRevisions.find(prim);
        if (r != moverRevisions.end()) {
            out.insert(out.end(), r->second.begin(), r->second.end());
        }
        const auto s = solverNode.find(prim);
        if (s != solverNode.end()) {
            out.push_back(s->second);
        }
        return out;
    };

    // -- providers ------------------------------------------------------
    for (const SdfPath &p : _providerPaths) {
        RigExecLiveOpNode node;
        node.id = "provider:" + p.GetString();
        node.domain = "provider";
        node.kind = joints.count(p.GetString())
            ? "joint"
            : (controls.count(p.GetString()) ? "control" : "xform");
        node.label = p.GetString();
        const auto anchor = _poseProviderAnchors.find(p);
        if (anchor != _poseProviderAnchors.end()) {
            _NotePath(&node.details, "anchor", anchor->second);
        }
        graph.nodes.push_back(std::move(node));
    }

    // -- solver batches and solvers -------------------------------------
    for (size_t b = 0; b < _solverBatches.size(); ++b) {
        const _SolverBatch &batch = _solverBatches[b];
        RigExecLiveOpNode node;
        node.id = "batch:" + std::to_string(b);
        node.domain = "batch";
        node.kind = batch.leader == b
            ? (batch.followers.empty() ? "leader" : "leader+followers")
            : "follower";
        node.label = "batch " + std::to_string(b) + " (" +
            std::to_string(batch.solvers.size()) + " solvers)";
        node.profile = _BatchScope(b, batch.level);
        node.level = int(batch.level);
        node.details["dirty"] = batch.dirty ? "true" : "false";
        node.details["leader"] = std::to_string(batch.leader);
        for (const auto &[solver, tap] : batch.solvers) {
            node.lists["solvers"].push_back(solver.GetString());
        }
        for (const SdfPath &dep : batch.dependencies) {
            node.lists["depends"].push_back(dep.GetString());
        }
        for (const SdfPath &input : batch.frameInputs) {
            node.lists["frame_inputs"].push_back(input.GetString());
        }
        for (const auto &[joint, writer] : batch.restInputs) {
            node.details["rest:" + joint.GetString()] = writer.GetString();
        }
        graph.nodes.push_back(std::move(node));
    }
    for (size_t b = 0; b < _solverBatches.size(); ++b) {
        const _SolverBatch &batch = _solverBatches[b];
        const std::string id = "batch:" + std::to_string(b);
        if (batch.leader != b) {
            edge(id, "batch:" + std::to_string(batch.leader), "rides");
        }
        for (const auto &[solver, tap] : batch.solvers) {
            edge("solver:" + solver.GetString(), id, "member");
        }
        for (const SdfPath &dep : batch.dependencies) {
            edge(id, "solver:" + dep.GetString(), "dep");
        }
        for (const SdfPath &input : batch.frameInputs) {
            edge(id, providerId(input), "read");
        }
        for (const auto &[joint, writer] : batch.restInputs) {
            edge(id, providerId(joint), "read");
        }
    }
    for (const auto &[solver, boundJoints] : _solverJoints) {
        RigExecLiveOpNode node;
        node.id = "solver:" + solver.GetString();
        node.domain = "solver";
        node.kind = "aggregate";
        node.label = solver.GetString();
        for (const auto &[joint, element] : boundJoints) {
            node.lists["joints"].push_back(
                joint.GetString() + "#" + std::to_string(element));
            edge(node.id, providerId(joint), "write");
        }
        graph.nodes.push_back(std::move(node));
    }
    // Solvers with no joints (guide-only) still get nodes.
    for (const auto &[solver, deps] : _solverDependencies) {
        const std::string id = "solver:" + solver.GetString();
        bool known = false;
        for (const RigExecLiveOpNode &n : graph.nodes) {
            if (n.id == id) {
                known = true;
                break;
            }
        }
        if (!known) {
            RigExecLiveOpNode node;
            node.id = id;
            node.domain = "solver";
            node.kind = "guide-only";
            node.label = solver.GetString();
            graph.nodes.push_back(std::move(node));
        }
        for (const SdfPath &dep : deps) {
            edge(id, "solver:" + dep.GetString(), "dep");
        }
    }
    // A batch member in neither map above still gets a node, so member
    // edges always land on one.
    for (size_t b = 0; b < _solverBatches.size(); ++b) {
        for (const auto &[solver, tap] : _solverBatches[b].solvers) {
            const std::string id = "solver:" + solver.GetString();
            bool known = false;
            for (const RigExecLiveOpNode &n : graph.nodes) {
                if (n.id == id) {
                    known = true;
                    break;
                }
            }
            if (!known) {
                RigExecLiveOpNode node;
                node.id = id;
                node.domain = "solver";
                node.kind = "aggregate";
                node.label = solver.GetString();
                graph.nodes.push_back(std::move(node));
            }
        }
    }
    // Each solver fires with its batch.
    for (size_t b = 0; b < _solverBatches.size(); ++b) {
        const _SolverBatch &batch = _solverBatches[b];
        for (const auto &[solver, tap] : batch.solvers) {
            const std::string id = "solver:" + solver.GetString();
            for (RigExecLiveOpNode &n : graph.nodes) {
                if (n.id == id && n.profile.empty()) {
                    n.profile = _BatchScope(b, batch.level);
                    n.level = int(batch.level);
                    n.details["batch"] = std::to_string(b);
                }
            }
        }
    }

    // -- frame constraints ----------------------------------------------
    for (size_t c = 0; c < _frameConstraints.size(); ++c) {
        const _FrameConstraint &constraint = _frameConstraints[c];
        RigExecLiveOpNode node;
        node.id = "constraint:" + constraint.moverPath.GetString();
        node.domain = "constraint";
        node.kind = constraint.schemaType.GetString();
        node.label = constraint.moverPath.GetString();
        node.profile =
            _ConstraintScope(constraint.schemaType, constraint.moverPath);
        node.order = int(c);
        _NotePath(&node.details, "weight_object", constraint.weightObject);
        _NotePath(&node.details, "points_target", constraint.pointsTarget);
        _NotePath(&node.details, "space", constraint.spacePath);
        for (const SdfPath &target : constraint.targets) {
            node.lists["targets"].push_back(target.GetString());
            edge(node.id, providerId(target), "write");
        }
        auto sourceEdges = [&](const _FrameSourceBinding &source) {
            if (source.sourcePath.IsEmpty()) {
                return;
            }
            node.lists["sources"].push_back(source.sourcePath.GetString());
            edge(node.id, providerId(source.sourcePath), "read");
            if (!source.xformPath.IsEmpty()) {
                edge(node.id, providerId(source.xformPath), "read");
            }
            edge(node.id, tapNode("main", source.frameTap), "read");
        };
        for (const _FrameSourceBinding &source : constraint.sources) {
            sourceEdges(source);
        }
        sourceEdges(constraint.worldUpObject);
        sourceEdges(constraint.effector);
        for (const _FrameSourceBinding &pole : constraint.poleObjects) {
            sourceEdges(pole);
        }
        for (const SdfPath &joint : constraint.ikChain) {
            node.lists["ik_chain"].push_back(joint.GetString());
            edge(node.id, providerId(joint), "read");
        }
        edge(node.id, tapNode("first", constraint.spacePosedTap), "read");
        edge(node.id, tapNode("first", constraint.spaceDefaultTap), "read");
        graph.nodes.push_back(std::move(node));
    }

    // -- pose walk order --------------------------------------------------
    auto poseOpId = [&](const _PoseStep &step) {
        if (step.solverBatch) {
            return "batch:" + std::to_string(step.index);
        }
        if (step.index < _frameConstraints.size()) {
            return "constraint:" +
                _frameConstraints[step.index].moverPath.GetString();
        }
        return std::string();
    };
    for (size_t i = 1; i < _poseSteps.size(); ++i) {
        edge(poseOpId(_poseSteps[i - 1]), poseOpId(_poseSteps[i]), "order");
    }
    // Provider write chains: each writer revises the provider, in order.
    for (const auto &[provider, writers] : _frameChains) {
        const std::string dst = providerId(provider);
        std::string prev;
        for (const SdfPath &writer : writers) {
            std::string src;
            const std::string path = writer.GetString();
            if (moverConstraint.count(path)) {
                src = moverConstraint[path];
            } else if (solverNode.count(path)) {
                src = solverNode[path];
            }
            if (src.empty()) {
                continue;
            }
            edge(src, dst, "write");
            edge(prev, src, "order");
            prev = src;
        }
    }

    // -- geometry chains and revisions ------------------------------------
    std::map<std::string, int> chainLevel;
    for (size_t l = 0; l < _chainPlan.levels.size(); ++l) {
        for (const SdfPath &target : _chainPlan.levels[l].targets) {
            chainLevel[target.GetString()] = int(l);
        }
    }
    auto describeChains =
        [&](const std::map<SdfPath, std::vector<_GraphRevision>> &chains,
            const char *kind, const char *chainPrefix,
            const char *revPrefix) {
            for (const auto &[target, revisions] : chains) {
                const std::string chainId = chainPrefix + target.GetString();
                RigExecLiveOpNode node;
                node.id = chainId;
                node.domain = "chain";
                node.kind = kind;
                node.label = target.GetString();
                const auto level = chainLevel.find(target.GetString());
                node.level = level == chainLevel.end() ? -1 : level->second;
                node.details["revisions"] = std::to_string(revisions.size());
                const auto live = _liveGraphs.find(target);
                node.details["live"] = live == _liveGraphs.end() ? "false"
                                                                 : "true";
                if (live != _liveGraphs.end() && live->second) {
                    node.details["live_identities"] = std::to_string(
                        live->second->identities.size());
                    node.details["base_static"] =
                        live->second->basePointsStatic ? "true" : "false";
                }
                const auto snaps = _chainPlan.snapshots.find(target);
                if (snaps != _chainPlan.snapshots.end()) {
                    for (const SdfPath &mover : snaps->second) {
                        node.lists["snapshots"].push_back(mover.GetString());
                    }
                }
                graph.nodes.push_back(std::move(node));
                for (size_t i = 0; i < revisions.size(); ++i) {
                    const _GraphRevision &rev = revisions[i];
                    const std::string id = revId(target, i, revPrefix);
                    RigExecLiveOpNode revNode;
                    revNode.id = id;
                    revNode.domain = "revision";
                    revNode.kind = RigExecBakedOpName(rev.op);
                    revNode.label =
                        rev.moverPath.GetString() + " (" + revNode.kind + ")";
                    // A derived revision assembles inside its chain's scope,
                    // which is the one that names the target.
                    revNode.profile = std::string(revPrefix) == "drev:"
                        ? "Derived " + target.GetString()
                        : _AssembleScope(rev.moverPath, target);
                    revNode.order = int(i);
                    revNode.details["mover"] = rev.moverPath.GetString();
                    revNode.details["target"] = target.GetString();
                    revNode.details["transform_final_phase"] =
                        rev.transformFinalPhase ? "true" : "false";
                    revNode.details["transform_posed_points"] =
                        rev.transformPosedPoints ? "true" : "false";
                    revNode.details["skin_topology_fixed"] =
                        rev.skinTopologyFixed ? "true" : "false";
                    const RigExecRevisionBinding &bind = rev.binding;
                    _NotePath(&revNode.details, "transform", bind.transform);
                    _NotePath(&revNode.details, "transform_space",
                              bind.transformSpace);
                    _NotePath(&revNode.details, "carry_space", bind.carrySpace);
                    _NotePath(&revNode.details, "weight_object",
                              bind.weightObject);
                    _NotePath(&revNode.details, "base", bind.base);
                    _NotePath(&revNode.details, "cage", bind.cagePoints);
                    _NotePath(&revNode.details, "surface", bind.surfacePoints);
                    _NotePath(&revNode.details, "driver_frames",
                              bind.driverFrames);
                    for (const SdfPath &influence : bind.influences) {
                        revNode.lists["influences"].push_back(
                            influence.GetString());
                    }
                    for (const SdfPath &input : bind.blendInputs) {
                        revNode.lists["blend_inputs"].push_back(
                            input.GetString());
                    }
                    for (const auto &[input, phase] : bind.phases) {
                        revNode.lists["phase_inputs"].push_back(
                            input.GetString() + " (" + phase.GetAsString() +
                            ")");
                        const std::string inputChain =
                            "chain:" + input.GetString();
                        bool known = _graphChains.count(input) > 0;
                        if (known) {
                            edge(id, inputChain, "phase");
                        }
                    }
                    if (!bind.transformPhase.IsBase()) {
                        revNode.details["transform_phase"] =
                            bind.transformPhase.GetAsString();
                    }
                    graph.nodes.push_back(std::move(revNode));
                    edge(id, chainId, "member");
                    if (i > 0) {
                        edge(revId(target, i - 1, revPrefix), id, "order");
                    }
                    auto tapEdge = [&](RigExecTapId tap) {
                        edge(id, tapNode("main", tap), "read");
                    };
                    tapEdge(rev.transformTap);
                    tapEdge(rev.transformSpaceTap);
                    tapEdge(rev.carrySpaceTap);
                    tapEdge(rev.weightTap);
                    tapEdge(rev.driverFramesTap);
                    for (RigExecTapId tap : rev.influenceTaps) {
                        tapEdge(tap);
                    }
                    for (RigExecTapId tap : rev.projectorRestTaps) {
                        tapEdge(tap);
                    }
                }
            }
        };
    describeChains(_graphChains, "points", "chain:", "rev:");
    describeChains(_graphDerivedChains, "derived", "derived:", "drev:");
    for (const auto &[target, revisions] : _graphDerivedChains) {
        edge("chain:" + target.GetString(), "derived:" + target.GetString(),
             "order");
    }
    for (size_t i = 1; i < _chainPlan.order.size(); ++i) {
        edge("chain:" + _chainPlan.order[i - 1].GetString(),
             "chain:" + _chainPlan.order[i].GetString(), "order");
    }

    // -- property chains --------------------------------------------------
    for (const auto &[target, revisions] : _propertyChains) {
        const std::string chainId = "prop:" + target.GetString();
        RigExecLiveOpNode node;
        node.id = chainId;
        node.domain = "propchain";
        node.kind = "scalar";
        node.label = target.GetString();
        node.profile = _PropChainScope(target);
        node.details["revisions"] = std::to_string(revisions.size());
        graph.nodes.push_back(std::move(node));
        for (size_t i = 0; i < revisions.size(); ++i) {
            const std::string id = "proprev:" + target.GetString() + "@" +
                std::to_string(i);
            RigExecLiveOpNode revNode;
            revNode.id = id;
            revNode.domain = "proprev";
            revNode.kind = revisions[i].schemaType.GetString();
            revNode.label = revisions[i].moverPath.GetString();
            revNode.profile = _PropChainScope(target);
            revNode.order = int(i);
            graph.nodes.push_back(std::move(revNode));
            edge(id, chainId, "member");
            if (i > 0) {
                edge("proprev:" + target.GetString() + "@" +
                         std::to_string(i - 1),
                     id, "order");
            }
        }
    }
    for (size_t i = 1; i < _propertyChainOrder.size(); ++i) {
        edge("prop:" + _propertyChainOrder[i - 1].GetString(),
             "prop:" + _propertyChainOrder[i].GetString(), "order");
    }
    // Each phased read is an edge from its consumer's op to the chain, and
    // a line on the chain naming where in the history it reads: an edge has
    // no room for the revision, and a consumer with no op of its own (a
    // control avar) has no edge at all.
    std::map<std::string, std::vector<std::string>> phasedReaders;
    for (const RigExecPhasedConnection &phased : _phasedConnections) {
        // A target with no property chain is a geometry chain's.
        const auto prop = _propertyChains.find(phased.target);
        const std::string dst =
            (prop != _propertyChains.end() ? "prop:" : "chain:") +
            phased.target.GetString();
        for (const std::string &src : consumerNodes(phased.consumer)) {
            edge(src, dst, "phase");
        }
        std::string line = phased.consumer.GetString() + ": ";
        if (phased.final) {
            line += "final";
        } else if (phased.applied == 0) {
            line += "base";
        } else {
            line += "after " + std::to_string(phased.applied);
            if (prop != _propertyChains.end()) {
                line += " of " + std::to_string(prop->second.size());
            }
        }
        for (size_t h = 1; h < phased.hops.size(); ++h) {
            line += (h == 1 ? " via " : ", ") + phased.hops[h].GetString();
        }
        phasedReaders[dst].push_back(std::move(line));
    }

    // -- exec taps ----------------------------------------------------------
    auto describeTapSet = [&](const char *set, const RigExecTapSet *taps) {
        if (!taps) {
            return;
        }
        for (size_t t = 0; t < taps->GetTapCount(); ++t) {
            const RigExecValueAddress &address =
                taps->GetAddress(RigExecTapId(t));
            const std::string id =
                "tap:" + std::string(set) + ":" + std::to_string(t);
            RigExecLiveOpNode node;
            node.id = id;
            node.domain = "tap";
            node.kind = address.publicComputation.IsEmpty()
                ? "value"
                : address.publicComputation.GetString();
            node.label = address.target.GetString() +
                (address.publicComputation.IsEmpty()
                     ? ""
                     : " " + address.publicComputation.GetString());
            node.details["set"] = set;
            node.details["phase"] = address.phase.GetString();
            graph.nodes.push_back(std::move(node));
            SdfPath prim = address.target.GetPrimPath();
            if (!prim.IsEmpty()) {
                edge(id, providerId(prim), "provides");
            }
        }
    };
    describeTapSet("main", _taps.get());
    describeTapSet("rest", _restTaps.get());
    describeTapSet("first", _firstFramePoseTaps.get());
    describeTapSet("guide", _guideTaps.get());
    for (size_t b = 0; b < _solverBatches.size(); ++b) {
        if (_solverBatches[b].taps) {
            describeTapSet(("batch" + std::to_string(b)).c_str(),
                           _solverBatches[b].taps.get());
        }
    }
    for (const auto &[path, taps] : _connectedPoseTaps) {
        describeTapSet(("conn:" + path.GetString()).c_str(), taps.get());
    }
    // Solver taps live in the owning (leader's) set.
    for (size_t b = 0; b < _solverBatches.size(); ++b) {
        const _SolverBatch &batch = _solverBatches[b];
        const size_t owner = batch.taps ? b : batch.leader;
        for (const auto &[solver, tap] : batch.solvers) {
            edge("solver:" + solver.GetString(),
                 tapNode("batch" + std::to_string(owner), tap), "read");
        }
    }

    // -- space switches and interpolators -----------------------------------
    for (const _SpaceSwitch &sw : _spaceSwitches) {
        const std::string id = "switch:" + sw.switchPath.GetString();
        RigExecLiveOpNode node;
        node.id = id;
        node.domain = "switch";
        node.kind = "space";
        node.label = sw.switchPath.GetString();
        node.level = sw.band;
        _NotePath(&node.details, "target", sw.target);
        _NotePath(&node.details, "space", sw.spacePath);
        for (const auto &source : sw.sources) {
            if (!source.path.IsEmpty()) {
                node.lists["sources"].push_back(source.path.GetString());
                edge(id, providerId(source.path), "read");
            }
            edge(id, tapNode("first", source.posedTap), "read");
            edge(id, tapNode("first", source.defaultTap), "read");
        }
        graph.nodes.push_back(std::move(node));
        edge(id, providerId(sw.target), "write");
    }
    for (const _PoseInterpolator &interp : _poseInterpolators) {
        const std::string id = "interp:" + interp.prim.GetString();
        RigExecLiveOpNode node;
        node.id = id;
        node.domain = "interp";
        node.kind = "rbf";
        node.label = interp.prim.GetString();
        _NotePath(&node.details, "driver", interp.driver);
        _NotePath(&node.details, "driver_parent", interp.driverParent);
        for (const SdfPath &attr : interp.driverAttributes) {
            node.lists["driver_attributes"].push_back(attr.GetString());
        }
        for (const SdfPath &w : interp.poseWeights) {
            node.lists["pose_weights"].push_back(w.GetString());
        }
        graph.nodes.push_back(std::move(node));
        edge(id, providerId(interp.driver), "read");
    }

    // -- set-aside operations (present, but not executing) --------------------
    for (const auto &[path, reason] : _skippedOperations) {
        RigExecLiveOpNode node;
        node.id = "skipped:" + path.GetString();
        node.domain = "skipped";
        node.kind = "set-aside";
        node.label = path.GetString();
        node.details["reason"] = reason;
        graph.nodes.push_back(std::move(node));
    }

    std::set<std::string> ids;
    for (RigExecLiveOpNode &node : graph.nodes) {
        ids.insert(node.id);
        const auto readers = phasedReaders.find(node.id);
        if (readers != phasedReaders.end()) {
            node.lists["phased_readers"] = readers->second;
        }
    }
    // An edge was added by id before every node stood; drop any whose end
    // never became a node, so edges name only nodes of this graph.
    graph.edges.erase(
        std::remove_if(graph.edges.begin(), graph.edges.end(),
                       [&ids](const RigExecLiveOpEdge &e) {
                           return !ids.count(e.src) || !ids.count(e.dst);
                       }),
        graph.edges.end());
    return graph;
}

}  // namespace rigExec
