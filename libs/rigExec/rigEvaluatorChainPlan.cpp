// Dependency ordering, parallel levels, and revision snapshots.

#include "rigEvaluatorInternal.h"
#include "rigEvaluatorDependencies.h"
#include "movers/moverRegistry.h"

#include <algorithm>
#include <set>

namespace rigExec {

using namespace evaluatorDetail;

bool
RigExecRigEvaluator::_ValidateNativePhases(
    const std::map<SdfPath, std::vector<_GraphRevision>> &graphChains,
    const std::map<SdfPath, std::vector<SdfPath>> &frameChains,
    const std::vector<RigExecMoverRecord> &movers,
    std::map<SdfPath,std::set<SdfPath>> *checkpoints, _CompileFailure *failure)
{
    checkpoints->clear();
    const auto fail = [failure](const std::string &message,
                               SdfPathVector operations) {
        *failure = {message, std::move(operations)};
        return false;
    };
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

        (*checkpoints).clear();
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
                        continue;  // the chain's published result
                    }
                    if (phase.kind == RigExecReadPhaseKind::Preceding) {
                        const int readerOrdinal=ordinalOf[revision.moverPath];
                        SdfPath preceding;
                        for(const auto &writer:movers) {
                            const auto position=ordinalOf.find(writer);
                            if(position!=ordinalOf.end() && position->second<readerOrdinal)
                                preceding=writer;
                        }
                        if(!preceding.IsEmpty()) (*checkpoints)[inputPath].insert(preceding);
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
                    (*checkpoints)[inputPath].insert(found);
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
                (*checkpoints)[revision.binding.transform].insert(found);
            }
        }
    }

    return true;
}

} // namespace rigExec
