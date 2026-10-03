// Dependency ordering, parallel levels, and revision snapshots.

#include "rigEvaluatorInternal.h"
#include "rigEvaluatorDependencies.h"
#include "movers/moverRegistry.h"

#include <algorithm>
#include <set>

namespace rigExec {

using namespace evaluatorDetail;

std::vector<SdfPath>
RigExecRigEvaluator::GetChainLevelTargets(size_t level) const
{
    return level < _chainPlan.levels.size() ? _chainPlan.levels[level].targets
                                       : std::vector<SdfPath>();
}

bool
RigExecRigEvaluator::IsChainLevelParallel(size_t level) const
{
    return level < _chainPlan.levels.size() && _chainPlan.levels[level].parallel;
}

bool
RigExecRigEvaluator::_CompileChainPlan(
    const std::map<SdfPath, std::vector<_GraphRevision>> &graphChains,
    const std::map<SdfPath, std::vector<SdfPath>> &frameChains,
    const std::vector<RigExecMoverRecord> &movers,
    const std::set<SdfPath> &currentPhaseWeights,
    _ChainPlan *plan, _CompileFailure *failure)
{
    *plan = {};
    const auto fail = [failure](const std::string &message,
                               SdfPathVector operations) {
        *failure = {message, std::move(operations)};
        return false;
    };
    // Chain evaluation order.
    // A chain that reads another chain's target at a non-base phase cannot
    // run until that chain has. Collect those edges and sort; a cycle is a
    // compile error, because there is no order that satisfies it and the
    // alternative -- picking one and reading a stale or authored value -- is
    // the silent-wrong-answer failure this engine refuses everywhere else.
    {
        std::map<SdfPath, std::set<SdfPath>> dependsOn;  // target -> producers
        for (const auto &[target, revisions] : graphChains) {
            dependsOn[target];  // every chain is a node, even with no edges
        }
        for (const auto &chain : graphChains) {
            const SdfPath &target = chain.first;
            const std::vector<_GraphRevision> &revisions = chain.second;
            for (const _GraphRevision &revision : revisions) {
                auto addEdge = [&](const SdfPath &producer) {
                    if (producer.IsEmpty() || producer == target ||
                        !graphChains.count(producer)) {
                        return;
                    }
                    dependsOn[target].insert(producer);
                };
                for (const auto &[inputPath, phase] : revision.binding.GetPhasedInputs()) {
                    addEdge(inputPath);
                }
                // The Profile Mover's implicit dependency: its net's knots
                // are posed by ordinary movers and it must see them posed.
                // Stated as an edge now rather than as a pass ordering, so
                // one mechanism carries both kinds.
                addEdge(revision.binding.curvenetPoints);
            }
        }

        auto schedule = _OrderDependencies(dependsOn);
        if (!schedule.blocked.empty()) {
            std::string cycle;
            SdfPathVector operations;
            for (const SdfPath &target : schedule.blocked) {
                if (!cycle.empty()) cycle += ", ";
                cycle += target.GetString();
                for (const auto &revision : graphChains.at(target)) {
                    operations.push_back(revision.moverPath);
                }
            }
            return fail("Cyclic read-phase dependency between chains: " +
                        cycle + " (a phased read cannot be satisfied in any "
                        "evaluation order)", std::move(operations));
        }
        plan->order = std::move(schedule.ordered);

        // Dependency levels over that order: a new level begins where a chain
        // reads one already in the current level. Greedy over plan->order
        // rather than "longest path from a root", so every level is a
        // CONTIGUOUS run of the order the walk takes anyway -- which is what
        // lets a level be spread over tasks without moving anything the walk
        // publishes.
        plan->levels.clear();
        std::set<SdfPath> inCurrentLevel;
        for (const SdfPath &target : plan->order) {
            bool readsCurrentLevel = false;
            const auto producers = dependsOn.find(target);
            if (producers != dependsOn.end()) {
                for (const SdfPath &producer : producers->second) {
                    if (inCurrentLevel.count(producer)) {
                        readsCurrentLevel = true;
                        break;
                    }
                }
            }
            if (plan->levels.empty() || readsCurrentLevel) {
                plan->levels.emplace_back();
                inCurrentLevel.clear();
            }
            plan->levels.back().targets.push_back(target);
            inCurrentLevel.insert(target);
        }
        for (_ChainLevel &level : plan->levels) {
            level.parallel = _IsChainLevelParallelSafe(level.targets, graphChains, currentPhaseWeights);
        }
    }

    // Validate every declared phase, and reduce it to the one revision it
    // names.
    {
        std::map<SdfPath, std::vector<SdfPath>> chainMovers;
        for (const auto &[target, revisions] : graphChains) {
            for (const _GraphRevision &revision : revisions) {
                chainMovers[target].push_back(revision.moverPath);
            }
        }
        std::map<SdfPath, int> ordinalOf;
        for (const RigExecMoverRecord &m : movers) {
            ordinalOf[m.moverPath] = m.ordinal;
        }

        plan->snapshots.clear();
        for (const auto &[target, revisions] : graphChains) {
            for (const _GraphRevision &revision : revisions) {
                for (const auto &[inputPath, phase] : revision.binding.GetPhasedInputs()) {
                    const std::string who =
                        revision.moverPath.GetString() + ": read phase '" +
                        phase.GetAsString() + "' on " + inputPath.GetString();

                    // A phase on an input nothing writes is a no-op that
                    // reads as intent. Reject it: the author asked for a
                    // revision of something that has none, and silently
                    // handing back the authored value is how a rig ends up
                    // deforming against the wrong pose with no signal.
                    const auto moversIt = chainMovers.find(inputPath);
                    if (moversIt == chainMovers.end()) {
                        return fail(who + " names a property no mover writes; "
                                          "only `base` is meaningful there", {revision.moverPath});
                    }
                    const std::vector<SdfPath> &movers = moversIt->second;

                    if (phase.kind == RigExecReadPhaseKind::Final) {
                        if (inputPath == target) {
                            return fail(who + " creates a self dependency on the final output", {revision.moverPath});
                        }
                        continue;  // the chain's published result
                    }
                    if (phase.kind == RigExecReadPhaseKind::Preceding) {
                        // Only meaningful when the reader is itself in that
                        // chain; otherwise there is no position to precede.
                        const auto at = std::find(movers.begin(), movers.end(),
                                                  revision.moverPath);
                        if (at == movers.end()) {
                            return fail(
                                who + " is `preceding`, but " +
                                revision.moverPath.GetString() +
                                " does not write " + inputPath.GetString() +
                                "; there is no preceding revision to name", {revision.moverPath});
                        }
                        if (at != movers.begin()) {
                            plan->snapshots[inputPath].insert(*(at - 1));
                        }
                        continue;
                    }

                    // AtPrim: the last revision at or beneath the named prim.
                    SdfPath found;
                    for (const SdfPath &mover : movers) {
                        if (mover == phase.prim || mover.HasPrefix(phase.prim)) {
                            found = mover;
                        }
                    }
                    if (found.IsEmpty()) {
                        return fail(who + " names " + phase.prim.GetString() +
                                    ", which writes nothing to " +
                                    inputPath.GetString(), {revision.moverPath});
                    }
                    // Within one chain the named revision must already have
                    // run when the reader runs. Across chains the topological
                    // order above guarantees it, so only the self-read case
                    // can be unsatisfiable.
                    if (inputPath == target) {
                        const auto namedOrdinal = ordinalOf.find(found);
                        const auto readerOrdinal =
                            ordinalOf.find(revision.moverPath);
                        if (namedOrdinal != ordinalOf.end() &&
                            readerOrdinal != ordinalOf.end() &&
                            namedOrdinal->second >= readerOrdinal->second) {
                            return fail(
                                who + " names " + found.GetString() +
                                ", which runs at or after the reader in the "
                                "composed walk (spec §4.2)", {revision.moverPath});
                        }
                    }
                    plan->snapshots[inputPath].insert(found);
                }
            }
        }

        // The transform provider's phase is answered from the FRAME chains,
        // so it validates against those rather than against chainMovers.
        for (const auto &[target, revisions] : graphChains) {
            for (const _GraphRevision &revision : revisions) {
                const RigExecReadPhase &phase = revision.binding.transformPhase;
                if (phase.kind != RigExecReadPhaseKind::AtPrim) {
                    continue;
                }
                const std::string who =
                    revision.moverPath.GetString() + ": read phase '" +
                    phase.GetAsString() + "' on rigExec:transform";
                const auto frameIt =
                    frameChains.find(revision.binding.transform);
                if (frameIt == frameChains.end()) {
                    return fail(who + " names a point in the pose walk, but " +
                                revision.binding.transform.GetString() +
                                " is revised by no pose mover", {revision.moverPath});
                }
                SdfPath found;
                for (const SdfPath &frameMover : frameIt->second) {
                    if (frameMover == phase.prim ||
                        frameMover.HasPrefix(phase.prim)) {
                        found = frameMover;
                    }
                }
                if (found.IsEmpty()) {
                    return fail(who + " names " + phase.prim.GetString() +
                                ", which revises nothing on " +
                                revision.binding.transform.GetString(), {revision.moverPath});
                }
                plan->snapshots[revision.binding.transform].insert(found);
            }
        }
    }

    return true;
}

bool
RigExecRigEvaluator::_IsChainLevelParallelSafe(
    const std::vector<SdfPath> &targets,
    const std::map<SdfPath, std::vector<_GraphRevision>> &graphChains,
    const std::set<SdfPath> &currentPhaseWeights)
{
    // Two chains are not enough work to pay for the dispatch and the join,
    // and a rig with two deformed meshes is far more common than one with
    // twenty.
    if (targets.size() < 3) {
        return false;
    }
    std::set<SdfPath> levelWeightObjects;
    for (const SdfPath &target : targets) {
        const auto chain = graphChains.find(target);
        if (chain == graphChains.end()) {
            return false;
        }
        std::set<SdfPath> chainWeightObjects;
        for (const _GraphRevision &revision : chain->second) {
            // A Profile Mover reads its curvenet's posed points out of the
            // generation being built and binds through a cut/factorization
            // cache every profile chain shares. Both are things a serial walk
            // has finished with before the next chain asks.
            if (revision.op == RigExecRevisionOp::Curvenet) {
                return false;
            }
            // Defence in depth against a future edge type, not a hazard the
            // dependency graph can currently produce: a phased read is an
            // edge addEdge already records, so a phased reader and the chain
            // that produces what it reads land in different levels, and a
            // phase read WITHIN a chain is served from that task's own
            // snapshots. No level the partition builds today holds a phased
            // read across its own chains. It stays because the cost is one
            // level's parallelism on a rig that has any phased read at all,
            // and the alternative is that a new edge kind -- one addEdge does
            // not know to record -- would make a level silently read the
            // chain-snapshot store mid-level, where what this level's chains
            // have recorded has not arrived yet.
            if (!revision.binding.phases.empty()) {
                return false;
            }
            for (const auto &blendInput : revision.binding.blendSamples) {
                for (const RigExecBlendSampleBinding &sample :
                         blendInput.second) {
                    if (!sample.phase.IsBase()) {
                        return false;
                    }
                }
            }
            if (revision.binding.weightObject.IsEmpty()) {
                continue;
            }
            // A `current` sample phase measures the field against the points
            // as they stand mid-chain, which re-enters weight resolution --
            // the one part of assembling a packet that is not a pure read of
            // the stage and this generation's results.
            if (currentPhaseWeights.count(revision.binding.weightObject)) {
                return false;
            }
            chainWeightObjects.insert(revision.binding.weightObject);
        }
        // One weight object driving two chains in the level: they publish the
        // same pose.weightFields entry, so which field a rigger is shown
        // would become a question about the walk rather than about the rig.
        // Schema validation reaches that case first today -- a weight
        // object's target has to be the mover's own -- so this is the walk
        // declining to depend on a rule enforced somewhere else.
        for (const SdfPath &weightObject : chainWeightObjects) {
            if (!levelWeightObjects.insert(weightObject).second) {
                return false;
            }
        }
    }
    return true;
}

} // namespace rigExec
