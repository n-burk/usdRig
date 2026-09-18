//
// rigExecRuntime executor (M2 framework).
//
// A port of the serial half of bakedSchedule.cpp: RigExecBakedComputeClosure
// and the serial step walk of RigExecBakedRunSteps. The runtime never
// rebuilds, so revision `ran` starts false and the program stamp never
// moves; everything else is the same value comparisons in the same order.
//

#include "rigExecRuntime/store.h"

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
RrComputeClosure(RrProgram *program, double time, bool force)
{
    RrStore &store = program->store;
    const RigExecWireCones &cones = *program->cones;
    const size_t count = program->clustering->clusters.size();
    store.closedWords.assign((count + 63) / 64, 0);
    if (count == 0) {
        return;
    }
    std::vector<uint64_t> dirty((count + 63) / 64, 0);

    // The runtime never rebuilds and carries no evaluator stamp, so the
    // stamp never moves; a rig that can look up a phased read still runs
    // everything every run, for the same emptied-store reason.
    const bool full = force || program->poses->phasedReads;
    const bool chainResultsMoved =
        !full && store.everRan && program->poses->hasPropertyChains &&
        store.propertyResults != store.lastPropertyResults;
    if (full) {
        for (size_t c = 0; c < count; ++c) {
            _RrSet(&dirty, c);
        }
    } else if (!store.everRan) {
        _RrUnionWords(&dirty, cones.poseClusters.words);
        _RrUnionWords(&dirty, cones.always.words);
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
            for (int cluster : cones.revisionClusters[r]) {
                _RrSet(&dirty, size_t(cluster));
            }
        }
        for (size_t c = 0; c < program->geometry->chains.size(); ++c) {
            if (store.chainHaveBase[c] && !store.chainBaseDirty[c]) {
                continue;
            }
            for (int cluster : cones.chainBaseClusters[c]) {
                _RrSet(&dirty, size_t(cluster));
            }
        }
    } else {
        _RrUnionWords(&dirty, cones.always.words);
        const size_t slots = program->slotMeta->paths.size();
        for (size_t i = 0; i < slots; ++i) {
            const size_t base = i * 11;
            bool moved = false;
            for (size_t k = 0; k < 11 && !moved; ++k) {
                moved = store.avars[base + k] != store.lastAvars[base + k];
            }
            if (moved) {
                _RrSet(&dirty, size_t(cones.avarCluster[i]));
            }
        }
        for (size_t k = 0; k < program->slotMeta->xformSlots.size(); ++k) {
            if (store.xformBase[k] != store.lastXformBase[k]) {
                _RrSet(&dirty, size_t(cones.avarCluster[size_t(
                                           program->slotMeta->xformSlots[k])]));
            }
        }
        for (int slot : store.ladderMovedSlots) {
            _RrSet(&dirty, size_t(cones.avarCluster[size_t(slot)]));
        }
        for (size_t k = 0; k < store.arrays.size(); ++k) {
            const RrConstraintArraysLive &arrays = store.arrays[k];
            if (arrays.ok == arrays.lastOk &&
                arrays.weights == arrays.lastWeights &&
                arrays.translationOffsets ==
                    arrays.lastTranslationOffsets &&
                arrays.rotationOffsets == arrays.lastRotationOffsets &&
                arrays.diagnostics == arrays.lastDiagnostics &&
                arrays.poleOk == arrays.lastPoleOk &&
                arrays.poleWeights == arrays.lastPoleWeights &&
                arrays.poleDiagnostics == arrays.lastPoleDiagnostics) {
                continue;
            }
            for (int cluster : cones.constraintArrayClusters[k]) {
                _RrSet(&dirty, size_t(cluster));
            }
        }
        for (size_t k = 0; k < store.deltaBaseMatrix.size(); ++k) {
            if (store.deltaBaseOk[k] != store.lastDeltaBaseOk[k] ||
                store.deltaBaseMatrix[k] != store.lastDeltaBaseMatrix[k]) {
                for (int cluster : cones.deltaBaseClusters[k]) {
                    _RrSet(&dirty, size_t(cluster));
                }
            }
        }
        for (size_t k = 0; k < store.nativeFrames.size(); ++k) {
            if (store.nativeFrameOk[k] != store.lastNativeFrameOk[k] ||
                store.nativeFrames[k].points !=
                    store.lastNativeFrames[k].points) {
                for (int cluster : cones.nativeSourceClusters[k]) {
                    _RrSet(&dirty, size_t(cluster));
                }
            }
        }
        for (size_t c = 0; c < program->geometry->chains.size(); ++c) {
            if (!store.chainBaseDirty[c] &&
                store.chainHaveBase[c] == store.lastHaveBase[c]) {
                continue;
            }
            for (int cluster : cones.chainBaseClusters[c]) {
                _RrSet(&dirty, size_t(cluster));
            }
        }
        for (size_t si = 0; si < store.ribbonDirty.size(); ++si) {
            if (!store.ribbonDirty[si]) {
                continue;
            }
            for (int cluster : cones.solverPointsClusters[si]) {
                _RrSet(&dirty, size_t(cluster));
            }
        }
        for (size_t r = 0; r < program->geometry->revisionIndex.size();
             ++r) {
            if (store.revisionStaticDirty[r]) {
                _RrSet(&dirty, size_t(cones.revisionStaticCluster[r]));
            }
            if (!store.revisionRan[r]) {
                for (int cluster : cones.revisionClusters[r]) {
                    _RrSet(&dirty, size_t(cluster));
                }
            }
        }
        if (time != store.lastTime) {
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
        for (int index : cones.overrideSteps) {
            const RigExecWireStep &step = (*program->steps)[size_t(index)];
            for (int input : step.overrideInputs) {
                if (store.overridden[size_t(input)] ||
                    store.lastOverridden[size_t(input)]) {
                    _RrSet(&dirty, size_t(step.cluster));
                    break;
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
    store.lastNativeFrames = store.nativeFrames;
    store.lastNativeFrameOk = store.nativeFrameOk;
    store.lastDeltaBaseMatrix = store.deltaBaseMatrix;
    store.lastDeltaBaseOk = store.deltaBaseOk;
    for (RrConstraintArraysLive &arrays : store.arrays) {
        arrays.lastOk = arrays.ok;
        arrays.lastWeights = arrays.weights;
        arrays.lastTranslationOffsets = arrays.translationOffsets;
        arrays.lastRotationOffsets = arrays.rotationOffsets;
        arrays.lastDiagnostics = arrays.diagnostics;
        arrays.lastPoleOk = arrays.poleOk;
        arrays.lastPoleWeights = arrays.poleWeights;
        arrays.lastPoleDiagnostics = arrays.poleDiagnostics;
    }
    store.lastPropertyResults = store.propertyResults;
    store.lastOverridden = store.overridden;
    for (size_t c = 0; c < program->geometry->chains.size(); ++c) {
        store.lastHaveBase[c] = store.chainHaveBase[c];
    }
    store.lastTime = time;
    store.everRan = true;
    store.lastClosedClusters = _RrCount(store.closedWords);
}

bool
RrRunSteps(RrProgram *program, double time, bool force, std::string *error)
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
        store.stepOutputs[i].BeginRun();
        const RigExecWireStepKind kind = steps[i].kind;
        bool ok = true;
        if (_RrIsWeightKind(kind)) {
            ok = RrRunWeightStep(program, i, time, error);
        } else if (_RrIsGeometryKind(kind)) {
            ok = RrRunGeometryStep(program, i, time, error);
        } else {
            ok = RrRunPoseStep(program, i, time, error);
        }
        if (!ok) {
            return false;
        }
    }
    RrComputeClosure(program, time, force);
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
        store.stepOutputs[i].BeginRun();
        const RigExecWireStepKind kind = steps[i].kind;
        bool ok = true;
        if (_RrIsWeightKind(kind)) {
            ok = RrRunWeightStep(program, i, time, error);
        } else if (_RrIsGeometryKind(kind)) {
            ok = RrRunGeometryStep(program, i, time, error);
        } else {
            ok = RrRunPoseStep(program, i, time, error);
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
                *error = "step " + program->TextOrEmpty(steps[i].label) +
                         " gave the generation back";
            }
            return false;
        }
    }
    return true;
}

}  // namespace rigExec
