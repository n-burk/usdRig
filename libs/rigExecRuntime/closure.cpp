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
#include <cstring>

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

namespace {
template <class T> void _RrAppend(std::string *key, const T &value)
{
    key->append(reinterpret_cast<const char *>(&value), sizeof(value));
}
void _RrMemoValue(std::string *key, const RrWireValue &value)
{
    _RrAppend(key, value.tag); _RrAppend(key, value.bits);
    _RrAppend(key, value.matrix); _RrAppend(key, value.vec3d);
    _RrAppend(key, value.vec3f);
}
template <class T> void _RrMemoArray(const RrProgram *program, uint32_t slot,
                                   std::string *key)
{
    const auto *values = RrInputArray<T>(program, slot);
    const size_t count = values ? values->size() : 0;
    _RrAppend(key, count);
    if (count) key->append(reinterpret_cast<const char *>(values->data()),
                           sizeof(T) * count);
}
std::string _RrHeadMemo(const RrProgram *program, const RigExecWireStep &step)
{
    std::string key;
    const auto &state = program->inputState;
    for (const auto slot : step.headInputSlots) {
        _RrAppend(&key, state.slotHasValue[slot]);
        if (RrInputTagIsArray(RrInputTag(uint8_t(state.file->inputs[slot].type())))) {
            switch (state.file->inputs[slot].type()) {
            case RigExecWireInputTag::IntArray: _RrMemoArray<int32_t>(program,slot,&key); break;
            case RigExecWireInputTag::FloatArray: _RrMemoArray<float>(program,slot,&key); break;
            case RigExecWireInputTag::DoubleArray: _RrMemoArray<double>(program,slot,&key); break;
            case RigExecWireInputTag::Vec2fArray: _RrMemoArray<RrVec2f>(program,slot,&key); break;
            case RigExecWireInputTag::Vec3fArray: _RrMemoArray<RrVec3f>(program,slot,&key); break;
            default: break;
            }
        } else _RrMemoValue(&key, state.slotCurrent[slot]);
    }
    for (const auto &read : step.headInputReads)
        _RrMemoValue(&key, RrReadInput(program, read));
    return key;
}
std::string _RrPropertyVersion(const RrProgram *program, uint32_t v)
{
    std::string key;
    const auto &value = program->store.propertyVersions[v];
    _RrAppend(&key, program->store.propertyVersionValid[v]);
    _RrAppend(&key, value.tag); _RrAppend(&key, value.f32);
    _RrAppend(&key, value.f64);
    for (size_t row = 0; row < 4; ++row)
        for (size_t col = 0; col < 4; ++col)
            _RrAppend(&key, value.matrix[row][col]);
    for (size_t k = 0; k < 3; ++k) _RrAppend(&key, value.vec[k]);
    return key;
}
bool _RrRangeMoved(const RrStore &store, const fb::SlotRange &range)
{
    const std::vector<char> *moved =
        range.domain() == RigExecWireSlotDomain::Rest ? &store.restChanged :
        range.domain() == RigExecWireSlotDomain::Ladder ? &store.ladderChanged :
        range.domain() == RigExecWireSlotDomain::SkinTopology ? &store.topologyChanged : nullptr;
    if (!moved) return false;
    for (uint32_t v = range.begin(); v < range.end(); ++v)
        if ((*moved)[v]) return true;
    return false;
}
bool _RrShadowed(const RrStore &store, const RigExecWireStep &step, uint32_t v)
{
    bool found = false;
    for (const auto &pair : step.shadowedReads) {
        if (pair.first != int32_t(v)) continue;
        if (store.propertyRecordStoodAside[size_t(pair.second)]) return false;
        found = true;
    }
    return found;
}
} // namespace

bool
RrRunHeadSteps(RrProgram *program, std::vector<std::string> *diagnostics,
               std::string *error)
{
    auto &store = program->store;
    const auto &steps = *program->steps;
    if (store.headRan.size() != steps.size()) {
        store.headRan.assign(steps.size(), 0);
        store.headLines.resize(steps.size());
        store.headMemoKeys.resize(steps.size());
    }
    store.headOutputChanged.assign(steps.size(), 0);
    store.restChanged.assign(program->slotMeta->paths.size(), 0);
    store.ladderChanged.assign(program->slotMeta->paths.size(), 0);
    store.topologyChanged.assign(program->geometry->revisionIndex.size() +
                                  program->geometry->derivedIndex.size(), 0);
    RrPropertyBegin(program);
    std::vector<char> finished(program->inputState.file->propertyChains.size(), 0);
    for (size_t i = 0; i < steps.size() && steps[i].isHead; ++i) {
        const auto &step = steps[i];
        if (step.kind == RigExecWireStepKind::PropertyRevision && step.headAlwaysRuns)
            RrPropertyPublishFinished(program, finished);
        const auto key = _RrHeadMemo(program, step);
        const bool first = !store.headRan[i];
        bool initialBindingMoved = false;
        if (first) {
            for (const auto &read : step.headInputReads) {
                std::string current, captured;
                _RrMemoValue(&current, RrReadInput(program, read));
                _RrMemoValue(&captured, program->inputState.values[read.constant]);
                initialBindingMoved = initialBindingMoved || current != captured;
            }
        }
        bool dirty = initialBindingMoved || step.headAlwaysRuns ||
            (first ? (step.kind == RigExecWireStepKind::PropertyRevision ||
                      step.kind == RigExecWireStepKind::SkinTopology ||
                      step.headVaryingLeaves) : key != store.headMemoKeys[i]);
        for (const auto &range : step.reads) {
            if (range.domain() == RigExecWireSlotDomain::PropertyResult) {
                for (uint32_t v = range.begin(); v < range.end(); ++v)
                    dirty = dirty || (store.propertyVersionChanged[v] &&
                                       !_RrShadowed(store, step, v));
            } else {
                dirty = dirty || _RrRangeMoved(store, range);
            }
        }
        store.headMemoKeys[i] = key;
        store.headRan[i] = 1;
        if (dirty) {
            store.runTrace.push_back(int32_t(i));
            store.stepOutputs[i].BeginRun();
            auto &lines = store.headLines[i]; lines.clear();
            bool changed = false;
            if (step.kind == RigExecWireStepKind::PropertyRevision) {
                std::vector<std::pair<uint32_t, std::string>> before;
                for (const auto &range : step.writes)
                    for (uint32_t v = range.begin(); v < range.end(); ++v)
                        before.emplace_back(v, _RrPropertyVersion(program, v));
                if (!RrRunPropertyPart(program, size_t(step.object), size_t(step.part), &lines)) {
                    if (error) *error = "step " + RrStepLabel(*program, i) + " has no property part";
                    return false;
                }
                for (const auto &[v, previous] : before) {
                    if (previous != _RrPropertyVersion(program, v)) {
                        store.propertyVersionChanged[v] = 1;
                        changed = true;
                    }
                }
                const auto &chain = program->inputState.file->propertyChains[size_t(step.object)];
                const uint32_t backing = chain.versionBase + uint32_t(step.part);
                if (store.propertyVersionChanged[backing])
                    for (const auto &record : program->inputState.file->phasedConsumers)
                        if (record.chain == uint32_t(step.object) && record.applied == uint32_t(step.part))
                            store.propertyVersionChanged[record.version] = 1;
            } else if (step.kind == RigExecWireStepKind::RestCompose ||
                       step.kind == RigExecWireStepKind::LadderCompose) {
                changed = RrRunRestHead(program, size_t(step.object),
                                        step.kind == RigExecWireStepKind::LadderCompose);
            } else if (step.kind == RigExecWireStepKind::SkinTopology) {
                changed = RrRunTopologyHead(program, size_t(step.object));
                store.topologyChanged[size_t(step.object)] = changed ? 1 : 0;
            }
            store.headOutputChanged[i] = changed ? 1 : 0;
        } else store.stepOutputs[i].MarkSkipped();
        if (step.kind == RigExecWireStepKind::PropertyRevision &&
            size_t(step.part) == program->inputState.file->propertyChains[size_t(step.object)].revisions.size())
            finished[size_t(step.object)] = 1;
        diagnostics->insert(diagnostics->end(), store.headLines[i].begin(), store.headLines[i].end());
    }
    RrPropertyPublish(program);
    return true;
}

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
    // stamp never moves and only the caller forces a whole run. A phased
    // read is no reason either: it reads frame records and chain versions
    // that persist across runs like every other slot.
    const bool full = force;
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
        }
        // A step that declares an input re-runs once when a set changed a
        // slot of its walk since the last run, a released drag included.
        // A standing override re-runs it again only where its long-way
        // read can move with time or with the chains while no slot is set.
        const std::vector<char> &walkMoves =
            program->inputState.overrideWalkMoves;
        const bool runMoved = store.animatedTouched;
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

        for (size_t i = 0; i < program->steps->size(); ++i) {
        const auto &step = (*program->steps)[i];
        if (step.isHead) continue;
        bool moved = false;
        for (const auto &range : step.reads) {
            if (range.domain() == RigExecWireSlotDomain::PropertyResult)
                for (uint32_t v = range.begin(); v < range.end(); ++v)
                    moved = moved || (store.propertyVersionChanged[v] &&
                                      !_RrShadowed(store, step, v));
        }
        for (const auto &read : step.reads)
            moved = moved || _RrRangeMoved(store, read);
        if (moved) _RrSet(&dirty, size_t(step.cluster));
    }
    for (size_t c = 0; c < count; ++c) {
        if ((dirty[c / 64] & (uint64_t(1) << (c % 64))) != 0) {
            _RrUnionWords(&store.closedWords, cones.cone[c].words);
        }
    }

    // This census describes ordinary closure work. Head-only clusters
    // contain no ordinary closed step; mixed clusters remain selected.
    // Actual head execution is represented by the prologue run trace.
    for (size_t c = 0; c < count; ++c) {
        bool onlyHeads = true;
        for (const auto index : program->clustering->clusters[c].members)
            onlyHeads = onlyHeads && (*program->steps)[size_t(index)].isHead;
        if (onlyHeads) store.closedWords[c / 64] &= ~(uint64_t(1) << (c % 64));
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
        if (steps[i].isHead || !steps[i].isSource) {
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
        if (steps[i].isHead || steps[i].isSource ||
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
        if (steps[i].isHead || steps[i].isSource ||
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
