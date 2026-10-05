// rigExecRuntime executor (M2 framework).
// A port of the serial half of bakedSchedule.cpp: RigExecBakedComputeClosure
// and the serial step walk of RigExecBakedRunSteps. The runtime never
// rebuilds, so revision `ran` starts false and the program stamp never
// moves, and it holds no time: where the program compares the time with
// the last run's, the closure reads RrStore::animatedTouched. An input set
// is an authored value, so its readers re-run once per change, as the
// program re-runs them for a value edit (RrStore::changedSinceRun), not on
// every run a drag stands. Everything else is the same value comparisons
// in the same order.
#include "rigExecRuntime/labels.h"
#include "rigExecRuntime/store.h"

#include <algorithm>

namespace rigExec {

namespace {

bool
_RrIsGeometryKind(RigExecWireStepKind kind)
{
    switch (kind) {
    case RigExecWireStepKind::InfluenceFold:
    case RigExecWireStepKind::RevisionStatic:
    case RigExecWireStepKind::RevisionChunk:
    case RigExecWireStepKind::RevisionFuse:
    case RigExecWireStepKind::ChainStatus:
    case RigExecWireStepKind::Derived:
        return true;
    default:
        return false;
    }
}

bool
_RrIsWeightKind(RigExecWireStepKind kind)
{
    return kind == RigExecWireStepKind::WeightPacket ||
           kind == RigExecWireStepKind::VolumePlacements;
}

unsigned
_RrFamilyBit(RigExecWireStepKind kind)
{
    if (_RrIsWeightKind(kind)) {
        return 0x2u;
    }
    if (_RrIsGeometryKind(kind)) {
        return 0x4u;
    }
    return 0x1u;
}

bool
_RrTest(const std::vector<uint64_t> &words, size_t cluster)
{
    return (words[cluster / 64] & (uint64_t(1) << (cluster % 64))) != 0;
}

void
_RrSet(std::vector<uint64_t> *words, size_t cluster)
{
    (*words)[cluster / 64] |= uint64_t(1) << (cluster % 64);
}

void
_RrUnionWords(std::vector<uint64_t> *words,
              const std::vector<uint64_t> &other)
{
    for (size_t i = 0; i < words->size(); ++i) {
        (*words)[i] |= other[i];
    }
}

size_t
_RrCount(const std::vector<uint64_t> &words)
{
    size_t count = 0;
    for (uint64_t word : words) {
        while (word) {
            count += size_t(word & 1);
            word >>= 1;
        }
    }
    return count;
}

}  // namespace

void
RrComputeClosure(RrProgram *program, bool force)
{
    RrStore &store = program->store;
    const RigExecWireCones &cones = *program->cones;
    const size_t count = program->clustering->clusters.size();
    store.closedWords.assign((count + 63) / 64, 0);
    if (count == 0) {
        store.animatedTouched = false;
        std::fill(store.changedSinceRun.begin(), store.changedSinceRun.end(),
                  char(0));
        store.anyChangedSinceRun = false;
        return;
    }
    std::vector<uint64_t> dirty((count + 63) / 64, 0);

    // The runtime never rebuilds and carries no evaluator stamp, so the
    // stamp never moves; a rig that can look up a phased read still runs
    // everything every run, for the same emptied-store reason.
    const bool full = force || program->poses->phasedReads;
    // Equal to comparing the published maps: each entry is one attribute,
    // and an unpublished entry holds the same zero value every run.
    const bool chainResultsMoved =
        !full && store.everRan && program->poses->hasPropertyChains &&
        (store.propertyPublished != store.lastPropertyPublished ||
         store.propertyValues != store.lastPropertyValues);
    if (full) {
        for (size_t c = 0; c < count; ++c) {
            _RrSet(&dirty, c);
        }
    } else if (!store.everRan) {
        _RrUnionWords(&dirty, cones.poseClusters->words);
        _RrUnionWords(&dirty, cones.always->words);
        for (int index : cones.varyingSteps) {
            _RrSet(&dirty, size_t((*program->steps)[size_t(index)].cluster));
        }
        for (int index : cones.overrideSteps) {
            _RrSet(&dirty, size_t((*program->steps)[size_t(index)].cluster));
        }
        for (size_t r = 0; r < program->geometry->revisionIndex.size();
             ++r) {
            if (store.revisionStaticDirty[r]) {
                _RrSet(&dirty, size_t(cones.revisionStaticCluster[r]));
            }
            if (store.revisionRan[r]) {
                continue;
            }
            for (int cluster : cones.revisionClusters[r].v) {
                _RrSet(&dirty, size_t(cluster));
            }
        }
        for (size_t c = 0; c < program->geometry->chains.size(); ++c) {
            if (store.chainHaveBase[c] && !store.chainBaseDirty[c]) {
                continue;
            }
            for (int cluster : cones.chainBaseClusters[c].v) {
                _RrSet(&dirty, size_t(cluster));
            }
        }
    } else {
        _RrUnionWords(&dirty, cones.always->words);
        // A provider's own compose cluster, and every cluster that
        // recomposes an earlier version of it from the same avars and
        // ladder.
        const auto dirtyAvarReaders = [&](size_t slot) {
            _RrSet(&dirty, size_t(cones.avarCluster[slot]));
            if (slot < program->avarVersionClusters.size()) {
                for (const int32_t cluster :
                     program->avarVersionClusters[slot]) {
                    _RrSet(&dirty, size_t(cluster));
                }
            }
        };
        const size_t slots = program->slotMeta->paths.size();
        for (size_t i = 0; i < slots; ++i) {
            const size_t base = i * 11;
            bool moved = false;
            for (size_t k = 0; k < 11 && !moved; ++k) {
                moved = store.avars[base + k] != store.lastAvars[base + k];
            }
            if (moved) {
                dirtyAvarReaders(i);
            }
        }
        for (size_t k = 0; k < program->slotMeta->xformSlots.size(); ++k) {
            if (store.xformBase[k] != store.lastXformBase[k]) {
                dirtyAvarReaders(size_t(program->slotMeta->xformSlots[k]));
            }
        }
        for (int slot : store.ladderMovedSlots) {
            dirtyAvarReaders(size_t(slot));
        }
        // Constraint arrays, delta bases, native frames and ribbon driver
        // points are static: they never differ from the last run's, so no
        // diff is taken.
        for (size_t c = 0; c < program->geometry->chains.size(); ++c) {
            if (!store.chainBaseDirty[c] &&
                store.chainHaveBase[c] == store.lastHaveBase[c]) {
                continue;
            }
            for (int cluster : cones.chainBaseClusters[c].v) {
                _RrSet(&dirty, size_t(cluster));
            }
        }
        for (size_t r = 0; r < program->geometry->revisionIndex.size();
             ++r) {
            if (store.revisionStaticDirty[r]) {
                _RrSet(&dirty, size_t(cones.revisionStaticCluster[r]));
            }
            if (!store.revisionRan[r]) {
                for (int cluster : cones.revisionClusters[r].v) {
                    _RrSet(&dirty, size_t(cluster));
                }
            }
        }
        // An Animated input set, or time said to move: what the program
        // dirties when time moves.
        if (store.animatedTouched) {
            for (int index : cones.varyingSteps) {
                _RrSet(&dirty,
                       size_t((*program->steps)[size_t(index)].cluster));
            }
        } else if (chainResultsMoved) {
            for (int index : cones.varyingSteps) {
                const RigExecWireStep &step =
                    (*program->steps)[size_t(index)];
                if (step.resolvedInputReads) {
                    _RrSet(&dirty, size_t(step.cluster));
                }
            }
        }
        // A step that declares an input re-runs once when a set changed a
        // slot of its walk since the last run, a released drag included.
        // A standing override re-runs it again only where its long-way
        // read can move with time or with the chains while no slot is set.
        const std::vector<char> &walkMoves =
            program->inputState.overrideWalkMoves;
        const bool runMoved = store.animatedTouched || chainResultsMoved;
        if (store.anyChangedSinceRun || (runMoved && store.anyOverridden)) {
            for (int index : cones.overrideSteps) {
                const RigExecWireStep &step =
                    (*program->steps)[size_t(index)];
                for (int input : step.overrideInputs) {
                    const size_t number = size_t(input);
                    if (store.changedSinceRun[number] ||
                        (runMoved && store.overridden[number] &&
                         number < walkMoves.size() && walkMoves[number])) {
                        _RrSet(&dirty, size_t(step.cluster));
                        break;
                    }
                }
            }
        }
    }

    for (size_t c = 0; c < count; ++c) {
        if ((dirty[c / 64] & (uint64_t(1) << (c % 64))) != 0) {
            _RrUnionWords(&store.closedWords, cones.cone[c].words);
        }
    }

    // What the next run compares against.
    store.lastAvars = store.avars;
    store.lastXformBase = store.xformBase;
    store.lastPropertyValues = store.propertyValues;
    store.lastPropertyPublished = store.propertyPublished;
    // Consumed by this closure whichever branch took it: a first or a
    // forced run re-runs every reader the flags could name.
    if (store.anyChangedSinceRun) {
        std::fill(store.changedSinceRun.begin(), store.changedSinceRun.end(),
                  char(0));
        store.anyChangedSinceRun = false;
    }
    for (size_t c = 0; c < program->geometry->chains.size(); ++c) {
        store.lastHaveBase[c] = store.chainHaveBase[c];
    }
    store.animatedTouched = false;
    store.everRan = true;
    store.lastClosedClusters = _RrCount(store.closedWords);
}

bool
RrRunSteps(RrProgram *program, bool force, std::string *error)
{
    RrStore &store = program->store;
    const std::vector<RigExecWireStep> &steps = *program->steps;
    // Sources first, serial and in program order: a source weight packet
    // is composed from other source packets.
    for (size_t i = 0; i < steps.size(); ++i) {
        if (!steps[i].isSource) {
            continue;
        }
        if ((_RrFamilyBit(steps[i].kind) & program->runMask) == 0) {
            continue;
        }
        store.runTrace.push_back(int32_t(i));
        store.stepOutputs[i].BeginRun();
        const RigExecWireStepKind kind = steps[i].kind;
        bool ok = true;
        if (_RrIsWeightKind(kind)) {
            ok = RrRunWeightStep(program, i, error);
        } else if (_RrIsGeometryKind(kind)) {
            ok = RrRunGeometryStep(program, i, error);
        } else {
            ok = RrRunPoseStep(program, i, error);
        }
        if (!ok) {
            return false;
        }
    }
    RrComputeClosure(program, force);
    for (size_t i = 0; i < steps.size(); ++i) {
        if (steps[i].isSource ||
            _RrTest(store.closedWords, size_t(steps[i].cluster))) {
            continue;
        }
        store.stepOutputs[i].MarkSkipped();
        if (_RrIsGeometryKind(steps[i].kind)) {
            RrSkipGeometryStep(program, i);
        }
    }
    // Serial walk in program order. A masked family's steps are skipped
    // the same way the cone skips: values stand, deltas reset.
    for (size_t i = 0; i < steps.size(); ++i) {
        if (steps[i].isSource ||
            !_RrTest(store.closedWords, size_t(steps[i].cluster))) {
            continue;
        }
        if ((_RrFamilyBit(steps[i].kind) & program->runMask) == 0) {
            store.stepOutputs[i].MarkSkipped();
            if (_RrIsGeometryKind(steps[i].kind)) {
                RrSkipGeometryStep(program, i);
            }
            continue;
        }
        store.runTrace.push_back(int32_t(i));
        store.stepOutputs[i].BeginRun();
        const RigExecWireStepKind kind = steps[i].kind;
        bool ok = true;
        if (_RrIsWeightKind(kind)) {
            ok = RrRunWeightStep(program, i, error);
        } else if (_RrIsGeometryKind(kind)) {
            ok = RrRunGeometryStep(program, i, error);
        } else {
            ok = RrRunPoseStep(program, i, error);
        }
        if (!ok) {
            return false;
        }
        if (!store.stepOutputs[i].snapshots.IsEmpty()) {
            store.runSnapshots.Merge(
                std::move(store.stepOutputs[i].snapshots));
        }
        if (store.stepOutputs[i].bail) {
            if (error) {
                *error = "step " + RrStepLabel(*program, i) +
                         " gave the generation back";
            }
            return false;
        }
    }
    return true;
}

}  // namespace rigExec
