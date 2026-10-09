// rigExecRuntime Open (M2 framework).
// Opens the file through RigExecFormatOpen, whose validator holds every
// rule the file alone decides (the step and cluster graph the index walk
// follows among them), then derives what the file implies but does not
// store: each path node's text, the input reads each table field binds,
// the path-read table, the ribbon driver points, the SSA version pool
// sizes and last-version maps (scanning the commit writes), and the store
// sizing; the families build their own tables (blend layouts and points,
// expanded skin topologies, base points, and the path-read rows each site
// reads) as they size their scratch, so no run builds any of it. The
// geometry kernels' environment settings are read here too, so no run
// reads the environment.
// Cross-references that do not close are Open errors naming the table.
#include "rigExecRuntime/runtime.h"

#include "rigExecRuntime/store.h"
#include "rigExecRuntime/spaces.h"
#include "rigExecRuntime/opGraph.h"

#include <algorithm>
#include <cstdlib>
#include <set>
#include <string>

namespace rigExec {

namespace {

bool
RrGeoGetenvBool(const char *name, bool fallback)
{
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4996)
#endif
    const char *value = std::getenv(name);
#ifdef _MSC_VER
#pragma warning(pop)
#endif
    if (!value || !*value) {
        return fallback;
    }
    std::string lower(value);
    for (char &c : lower) {
        if (c >= 'A' && c <= 'Z') {
            c = char(c - 'A' + 'a');
        }
    }
    if (lower == "0" || lower == "false" || lower == "no" ||
        lower == "off") {
        return false;
    }
    if (lower == "1" || lower == "true" || lower == "yes" ||
        lower == "on") {
        return true;
    }
    return fallback;
}

// The baked program's default: SIMD on.
RrGeoSettings
RrGeoSettingsFromEnvironment()
{
    RrGeoSettings settings;
    settings.useSimd = RrGeoGetenvBool("RIGEXEC_ENABLE_SIMD", true);
    return settings;
}

}  // namespace

RigExecRuntimeReader::RigExecRuntimeReader() : _program(new RrProgram())
{
    _noValue.matrix.SetDiagonal(0.0);
    _noInput.defaultValue = _noValue;
}

RigExecRuntimeReader::~RigExecRuntimeReader() = default;

std::unique_ptr<RigExecRuntimeReader>
RigExecRuntimeReader::Open(const uint8_t *bytes, size_t size,
                           std::string *error)
{
    auto fail = [&](const std::string &why) {
        if (error) {
            *error = why;
        }
        return std::unique_ptr<RigExecRuntimeReader>();
    };
    std::unique_ptr<RigExecRuntimeReader> self(new RigExecRuntimeReader());
    RrProgram &program = *self->_program;
    {
        std::string why;
        if (!RigExecFormatOpen(bytes, size, &program.file, &why)) {
            return fail(why);
        }
    }
    const RigExecWireFile &file = *program.file;
    program.steps = &file.steps;
    program.clustering = file.clustering.get();
    program.cones = file.cones.get();
    program.slotMeta = file.slotMeta.get();
    program.constants = file.constants.get();
    program.poses = file.pose.get();
    program.requiredStageFramesAdmission = *file.pose->requiredStageFramesAdmission;
    program.geometry = file.geometry.get();
    program.statics.file = &file;
    program.geoSettings = RrGeoSettingsFromEnvironment();
    program.verifyConstantSources =
        RrGeoGetenvBool("RIGEXEC_VERIFY_CONSTANT_KEYS", false);
    program.verifyLeafVersions =
        RrGeoGetenvBool("RIGEXEC_VERIFY_LEAF_VERSIONS", false);
    program.verifySparseLeaves =
        RrGeoGetenvBool("RIGEXEC_VERIFY_SPARSE_LEAVES", false);
    program.verifyChainVersions =
        RrGeoGetenvBool("RIGEXEC_VERIFY_CHAIN_VERSIONS", false);
    program.compileDiagnostics = file.compileDiagnostics;
    while (program.stepWeightObjects < file.geometry->weightObjects.size() &&
           !file.geometry->weightObjects[program.stepWeightObjects]
                .envelopeOnly) {
        ++program.stepWeightObjects;
    }

    // Each node's text, a parent before its child: a prim is its parent's
    // text, "/" and its name; a property its prim's, "." and its name; a
    // token its name.
    program.nodeText.assign(file.paths.size(), std::string());
    for (size_t i = 1; i < file.paths.size(); ++i) {
        const RigExecWirePathNode &node = file.paths[i];
        const std::string &name = file.names[node.name()];
        switch (node.kind()) {
        case RigExecWirePathKind::Prim:
            program.nodeText[i] = program.nodeText[node.parent()] + "/" + name;
            break;
        case RigExecWirePathKind::Property:
            program.nodeText[i] = program.nodeText[node.parent()] + "." + name;
            break;
        default:
            program.nodeText[i] = name;
            break;
        }
    }

    // Plugin movers: one playback state per entry, keyed by the revision
    // it names; every op-16 revision has its entry.
    const RigExecWireDomainGeometry &geometry = *file.geometry;
    for (const RigExecWireExternalMover &entry : file.externalMovers) {
        const auto &chains = geometry.chains;
        if (size_t(entry.chain) >= chains.size() ||
            size_t(entry.revision) >= chains[entry.chain].revisions.size() ||
            chains[entry.chain].revisions[entry.revision].op !=
                uint8_t(RigExecWireExternalRevisionOp)) {
            return fail("an external mover entry names no plugin revision");
        }
        RrProgram::ExternalRevision state;
        state.type = program.TextOrEmpty(entry.type);
        if (state.type.empty()) {
            return fail("an external mover names no type");
        }
        if (!program.externalIndex
                 .emplace(std::make_pair(entry.chain, entry.revision),
                          program.externals.size())
                 .second) {
            return fail("an external mover entry is duplicated");
        }
        program.externals.push_back(std::move(state));
    }
    for (size_t c = 0; c < geometry.chains.size(); ++c) {
        const auto &revisions = geometry.chains[c].revisions;
        for (size_t r = 0; r < revisions.size(); ++r) {
            if (revisions[r].op == uint8_t(RigExecWireExternalRevisionOp) &&
                !program.externalIndex.count(
                    std::make_pair(uint32_t(c), uint32_t(r)))) {
                return fail("a plugin revision carries no external mover "
                            "entry");
            }
        }
    }

    const size_t slots = file.slotMeta->paths.size();
    const size_t clusters = file.clustering->clusters.size();
    const size_t revisions = geometry.revisionIndex.size();
    if (file.constants->avarConstants.size() != slots * 11) {
        return fail("avar constants do not cover the slots");
    }
    // The input reads: each table field the steps read binds its Input in
    // place; the path reads come out of the pools.
    {
        std::string why;
        if (!RrInputsOpen(&program, &file, &why) ||
            !RrInputsBindReads(&program, &why) ||
            !RrInputsBuildPathReads(&program, &why)) {
            return fail(why);
        }
    }

    const RigExecWireDomainPose &poses = *file.pose;
    // Switch index by provider slot, dense, so the compose reads one int
    // per slot. Left empty when the rig has no switch, which is the
    // branch the compose tests first.
    if (!poses.spaceSwitches.empty()) {
        const RigExecWireSlotMeta &meta = *file.slotMeta;
        // A version the compose reads: its anchor's posedM (or identity),
        // then each recompose slot composed from its avars and ladder.
        const auto versionInRange = [&](const RigExecWireFrameVersion &read) {
            if (read.anchor < -1 ||
                (read.anchor >= 0 && size_t(read.anchor) >= slots)) {
                return false;
            }
            for (const int32_t at : read.recompose) {
                if (at < 0 || size_t(at) >= slots ||
                    size_t(at) >= meta.slotKind.size() ||
                    meta.slotKind[size_t(at)] !=
                        RigExecWireSlotKind::FirstFramePose) {
                    return false;
                }
            }
            return true;
        };
        program.spaceSwitchBySlot.assign(slots, -1);
        for (size_t i = 0; i < poses.spaceSwitches.size(); ++i) {
            const RigExecWireSpaceSwitch &sw =
                poses.spaceSwitches[i];
            if (sw.slot < 0 || size_t(sw.slot) >= slots) {
                return fail("space switch names no slot");
            }
            for (const int32_t source : sw.sourceSlots) {
                if (source >= 0 && size_t(source) >= slots) {
                    return fail("space switch names no source slot");
                }
            }
            if (sw.spaceSlot >= 0 && size_t(sw.spaceSlot) >= slots) {
                return fail("space switch names no space slot");
            }
            if (sw.sourceReads.size() != sw.sourceSlots.size() ||
                !versionInRange(*sw.parentRead) ||
                !versionInRange(*sw.spaceRead)) {
                return fail("space switch reads a version of no slot");
            }
            for (const RigExecWireFrameVersion &read : sw.sourceReads) {
                if (!versionInRange(read)) {
                    return fail("space switch reads a version of no slot");
                }
            }
            // A world source and a missing space are never read; the
            // program leaves their versions at {-1, {}}.
            const auto unread = [](const RigExecWireFrameVersion &read) {
                return read.anchor == -1 && read.recompose.empty();
            };
            bool versionWithoutSlot =
                sw.spaceSlot < 0 && !unread(*sw.spaceRead);
            for (size_t k = 0; k < sw.sourceSlots.size(); ++k) {
                versionWithoutSlot =
                    versionWithoutSlot ||
                    (sw.sourceSlots[k] < 0 && !unread(sw.sourceReads[k]));
            }
            if (versionWithoutSlot) {
                return fail("space switch reads a version of a world source "
                            "or a missing space");
            }
            program.spaceSwitchBySlot[size_t(sw.slot)] = int32_t(i);
        }
    }
    RrStore &store = program.store;
    store.avars = file.constants->avarConstants;
    store.lastAvars = store.avars;

    // SSA versions, replicating BindPoseVersions: the seeds are the
    // slots; a slot's LAST write takes the dense entry at [n, 2n) and
    // only the versions in between go to the arena past 2n.
    store.finLast.assign(slots, 0);
    store.baseLast.assign(slots, 0);
    for (size_t i = 0; i < slots; ++i) {
        store.finLast[i] = uint32_t(i);
        store.baseLast[i] = uint32_t(i);
    }
    std::vector<uint32_t> finWriteCount(slots, 0);
    std::vector<uint32_t> baseWriteCount(slots, 0);
    uint32_t finMax = 0, baseMax = 0;
    bool haveFinReference = false, haveBaseReference = false;
    auto noteFin = [&](uint32_t v) {
        haveFinReference = true; finMax = std::max(finMax, v);
    };
    auto noteBase = [&](uint32_t v) {
        haveBaseReference = true; baseMax = std::max(baseMax, v);
    };
    for (const RigExecWireCommit &commit : poses.commits) {
        for (size_t p = 0; p < commit.slots.size(); ++p) {
            const size_t slot = size_t(commit.slots[p]);
            if (slot >= slots ||
                p >= commit.slotWrites.size()) {
                return fail("a commit write names no slot");
            }
            ++finWriteCount[slot];
            noteFin(commit.slotWrites[p]);
            if (p < commit.slotReads.size()) {
                noteFin(commit.slotReads[p]);
            }
            if (p < commit.slotCarry.size()) {
                noteFin(commit.slotCarry[p]);
            }
            if (commit.solverOutput && p < commit.slotBaseWrites.size()) {
                ++baseWriteCount[slot];
                noteBase(commit.slotBaseWrites[p]);
            }
            if (commit.solverOutput && p < commit.slotBaseCarry.size()) {
                noteBase(commit.slotBaseCarry[p]);
            }
        }
        for (size_t k = 0; k < commit.propagate.size(); ++k) {
            const size_t slot = size_t(commit.propagate[k].first);
            if (slot >= slots ||
                k >= commit.descendantWrites.size()) {
                return fail("a propagation write names no slot");
            }
            ++finWriteCount[slot];
            noteFin(commit.descendantWrites[k]);
            if (k < commit.descendantReads.size()) {
                noteFin(commit.descendantReads[k]);
            }
            if (k < commit.closestReads.size()) {
                noteFin(commit.closestReads[k]);
            }
            if (k < commit.descendantCarry.size()) {
                noteFin(commit.descendantCarry[k]);
            }
            if (commit.solverOutput &&
                k < commit.descendantBaseWrites.size()) {
                ++baseWriteCount[slot];
                noteBase(commit.descendantBaseWrites[k]);
            }
            if (commit.solverOutput &&
                k < commit.descendantBaseCarry.size()) {
                noteBase(commit.descendantBaseCarry[k]);
            }
        }
        for (uint32_t v : commit.sourceReads) {
            noteFin(v);
        }
        noteFin(commit.worldUpRead);
        noteFin(commit.targetRead);
        for (uint32_t v : commit.targetReads) {
            noteFin(v);
        }
        noteFin(commit.effectorRead);
        for (uint32_t v : commit.poleReads) {
            noteFin(v);
        }
        for (const RigExecWireAncestorRead &read :
             commit.effectorAncestors) {
            noteFin(read.fin);
            noteBase(read.base);
        }
        for (const auto &list : commit.poleAncestors) {
            for (const RigExecWireAncestorRead &read : list.v) {
                noteFin(read.fin);
                noteBase(read.base);
            }
        }
        for (const auto &list : commit.sourceAncestors) {
            for (const RigExecWireAncestorRead &read : list.v) {
                noteFin(read.fin);
                noteBase(read.base);
            }
        }
        for (const RigExecWireAncestorRead &read :
             commit.worldUpAncestors) {
            noteFin(read.fin);
            noteBase(read.base);
        }
    }
    for (const RigExecWireSolver &solver : poses.solvers) {
        for (uint32_t v : solver.restReads) {
            noteFin(v);
        }
        for (uint32_t v : solver.controlReads) {
            noteFin(v);
        }
        noteFin(solver.rootRead);
        noteFin(solver.midRead);
        noteFin(solver.endRead);
        noteFin(solver.poleRead);
        if (solver.spaceSlot >= 0) {
            if (size_t(solver.spaceSlot) >= slots) {
                return fail("a solver's space names no slot");
            }
            noteFin(solver.spaceRead);
        }
    }
    // Refresh is a real Base/Fin writer family, just like a commit. Its
    // target and inherited carries need arena slots even when the value
    // later passes through; admission has already proved their ownership.
    const auto refreshFrames = [&](int32_t slot, uint32_t baseRead,
                                   uint32_t finRead, uint32_t baseWrite,
                                   uint32_t finWrite) {
        if (slot < 0 || size_t(slot) >= slots) return false;
        ++baseWriteCount[size_t(slot)];
        ++finWriteCount[size_t(slot)];
        noteBase(baseRead); noteBase(baseWrite);
        noteFin(finRead); noteFin(finWrite);
        return true;
    };
    for (const auto &refresh : poses.providerRefreshes) {
        if (!refreshFrames(refresh.slot, refresh.baseRead, refresh.finRead,
                           refresh.baseWrite, refresh.finWrite)) {
            return fail("a provider refresh write names no slot");
        }
        for (const auto &carry : refresh.carries) {
            if (!refreshFrames(carry.slot, carry.baseRead, carry.finRead,
                               carry.baseWrite, carry.finWrite)) {
                return fail("a provider refresh carry write names no slot");
            }
        }
    }
    for (const auto &frame : poses.providerFrameInputs) {
        if (frame.base) noteBase(frame.version);
        else noteFin(frame.version);
    }
    size_t finArena = 0, baseArena = 0;
    for (size_t i = 0; i < slots; ++i) {
        if (finWriteCount[i] > 0) {
            store.finLast[i] = uint32_t(slots + i);
            finArena += size_t(finWriteCount[i]) - 1;
        }
        if (baseWriteCount[i] > 0) {
            store.baseLast[i] = uint32_t(slots + i);
            baseArena += size_t(baseWriteCount[i]) - 1;
        }
    }
    const size_t finPool = 2 * slots + finArena;
    const size_t basePool = 2 * slots + baseArena;
    // A mesh-only rig has no pose slots and no version references. An empty
    // pool is valid there; the initial max of zero is not a reference to it.
    if ((haveFinReference && size_t(finMax) >= finPool) ||
        (haveBaseReference && size_t(baseMax) >= basePool)) {
        return fail("a version reference exceeds the version pools");
    }
    store.fin.assign(finPool, RrPointFrame());
    store.base.assign(basePool, RrPointFrame());

    // Retained producer lookup metadata must match the admitted program.
    const RigExecWireCones &cones = *file.cones;
    const size_t words = (clusters + 63) / 64;
    if (cones.always->words.size() != words ||
        cones.poseClusters->words.size() != words ||
        cones.avarCluster.size() != slots ||
        cones.chainBaseClusters.size() != geometry.chains.size() ||
        cones.revisionClusters.size() != revisions ||
        cones.revisionStaticCluster.size() != revisions) {
        return fail("cone lookup tables do not match the program");
    }
    // Preserve admission checks for producer lookup cluster identities.
    const auto clusterInRange = [clusters](int32_t cluster) {
        return cluster >= 0 && size_t(cluster) < clusters;
    };
    bool coneClustersInRange =
        std::all_of(cones.avarCluster.begin(), cones.avarCluster.end(),
                    clusterInRange) &&
        std::all_of(cones.revisionStaticCluster.begin(),
                    cones.revisionStaticCluster.end(), clusterInRange);
    for (const std::vector<fb::RigExecWireIntList> *lists :
         {&cones.chainBaseClusters, &cones.revisionClusters}) {
        for (const fb::RigExecWireIntList &list : *lists) {
            coneClustersInRange =
                coneClustersInRange &&
                std::all_of(list.v.begin(), list.v.end(), clusterInRange);
        }
    }
    if (!coneClustersInRange) {
        return fail("a cone lookup table names no cluster");
    }
    // Exact native declaration associations, independent of cluster packing.
    program.avarReaderSteps.assign(slots, {});
    program.chainBaseSteps.assign(geometry.chains.size(), {});
    program.revisionSteps.assign(geometry.revisionIndex.size(), {});
    program.revisionStaticStep.assign(geometry.revisionIndex.size(), -1);
    for (size_t i = 0; i < file.steps.size(); ++i) {
        const auto &step = file.steps[i];
        if (step.isHead) continue;
        if (step.kind == RigExecWireStepKind::ComposeSubtree) {
            const auto &group = poses.composeGroups[size_t(step.object)];
            for (int slot = group.begin; slot < group.end; ++slot)
                program.avarReaderSteps[size_t(slot)].push_back(int32_t(i));
        }
        for (const auto &read : step.reads) {
            if (step.kind == RigExecWireStepKind::ComposeSubtree &&
                read.domain() == RigExecWireSlotDomain::Avars) {
                for (uint32_t slot = read.begin() / 11;
                     slot < (uint64_t(read.end()) + 10) / 11 && slot < slots; ++slot)
                    if (int(slot) < poses.composeGroups[size_t(step.object)].begin ||
                        int(slot) >= poses.composeGroups[size_t(step.object)].end)
                        program.avarReaderSteps[slot].push_back(int32_t(i));
            }
            if (read.domain() == RigExecWireSlotDomain::ChainBase) {
                for (uint32_t c = read.begin(); c < read.end(); ++c)
                    program.chainBaseSteps[c].push_back(int32_t(i));
            }
        }
        switch (step.kind) {
        case RigExecWireStepKind::RevisionStatic:
            program.revisionStaticStep[size_t(step.object)] = int32_t(i);
            [[fallthrough]];
        case RigExecWireStepKind::InfluenceFold:
        case RigExecWireStepKind::RevisionChunk:
        case RigExecWireStepKind::RevisionFuse:
            program.revisionSteps[size_t(step.object)].push_back(int32_t(i));
            break;
        default: break;
        }
    }
    for(size_t revision=0;revision<program.revisionStaticStep.size();++revision) {
        if(program.revisionStaticStep[revision]>=0) continue;
        bool excluded=false;
        if(file.commonGraph) for(uint64_t id:file.commonGraph->excludedValues) {
            if(id>=file.commonGraph->valueSpecs.size()) continue;
            const auto &spec=file.commonGraph->valueSpecs[size_t(id)];
            if(spec.domain==uint32_t(RigExecWireSlotDomain::RevisionPacket) && spec.slot==revision)
                excluded=true;
        }
        if(!excluded) return fail("a revision has no RevisionStatic step or explicit excluded packet");
    }
    for (int index : cones.varyingSteps) {
        if (index < 0 || size_t(index) >= file.steps.size()) {
            return fail("a varying step names no step");
        }
    }
    for (int index : cones.overrideSteps) {
        if (index < 0 || size_t(index) >= file.steps.size()) {
            return fail("an override step names no step");
        }
    }

    // Store sizing.
    RrMat4d identity;
    identity.SetIdentity();
    store.posedM.assign(slots, identity);
    store.finalMatrix.assign(slots, identity);
    store.baseMatrix.assign(slots, identity);
    store.aggregates.assign(poses.solvers.size(),
                            RrPointFrameArray());
    store.ladders.assign(slots, RrLadderLive());
    // Seeded with the reads' folded constants until a prologue composes.
    for (size_t i = 0;
         i < std::min(slots, poses.ladders.size()); ++i) {
        RrLadderLive &live = store.ladders[i];
        const auto constant = [&](int field) {
            return program.RegisteredConstant(
                program.ladderRead[i][size_t(field)]);
        };
        live.restSpace = constant(RrLadderRestSpace).matrix;
        live.defaultSpace = constant(RrLadderDefaultSpace).matrix;
        live.posedSpace = constant(RrLadderPosedSpace).matrix;
        for (int k = 0; k < 6; ++k) {
            live.restAvars[k] = constant(RrLadderRestAvar0 + k).f64;
            live.defaultAvars[k] = constant(RrLadderDefaultAvar0 + k).f64;
        }
        live.rotationOrder = constant(RrLadderRotationOrder).token;
    }
    store.solverOutFrames.assign(poses.solvers.size(), {});
    store.solverOutPresent.assign(poses.solvers.size(), {});
    for (size_t i = 0; i < poses.solvers.size(); ++i) {
        store.solverOutFrames[i].assign(
            poses.solvers[i].outputs.size(), RrPointFrame());
        store.solverOutPresent[i].assign(
            poses.solvers[i].outputs.size(), 0);
    }
    // A ribbon's driver points: the ones the bake read, constant from
    // here on.
    store.ribbonConstant.assign(poses.solvers.size(), {});
    for (size_t i = 0; i < poses.solvers.size(); ++i) {
        const std::vector<RigExecWireVec3f> &driver =
            poses.solvers[i].ribbonConstantPoints;
        store.ribbonConstant[i].reserve(driver.size());
        for (const RigExecWireVec3f &p : driver) {
            store.ribbonConstant[i].push_back(
                RrVec3f(p[0], p[1], p[2]));
        }
    }
    store.commits.assign(poses.commits.size(), RrCommitScratch());
    for (size_t i = 0; i < poses.commits.size(); ++i) {
        const RigExecWireCommit &commit = poses.commits[i];
        RrCommitScratch &scratch = store.commits[i];
        scratch.present.assign(commit.slots.size(), 0);
        scratch.frames.assign(commit.slots.size(), RrPointFrame());
        scratch.deltas.assign(commit.slots.size(), identity);
        scratch.deltaOk.assign(commit.slots.size(), 0);
        scratch.staged.assign(commit.propagate.size(), RrPointFrame());
        scratch.outcome.assign(commit.propagate.size(), 0);
        scratch.sources.assign(commit.sources.size(),
                               RrConstraintSource());
    }
    store.arrays.assign(poses.constraintArrays.size(),
                        RrConstraintArraysLive());
    for (size_t i = 0; i < poses.constraintArrays.size(); ++i) {
        store.arrays[i].readPole =
            poses.constraintArrays[i].readPole;
    }
    store.chainHaveBase.assign(geometry.chains.size(), 0);
    store.chainBaseDirty.assign(geometry.chains.size(), 0);
    store.lastHaveBase.assign(geometry.chains.size(), 0);
    store.derivedHaveBase.assign(geometry.derivedIndex.size(), 0);
    store.nativeFrames.assign(poses.nativeSources.size(),
                              RrPointFrame());
    store.nativeFrameOk.assign(poses.nativeSources.size(), 0);
    store.switchFrames.assign(poses.spaceCheckpoints.size(), identity);
    store.xformBase.assign(file.slotMeta->xformSlots.size(), identity);
    store.lastXformBase.assign(file.slotMeta->xformSlots.size(),
                               identity);
    store.deltaBaseMatrix.assign(geometry.deltaBasePaths.size(),
                                 identity);
    store.deltaBaseOk.assign(geometry.deltaBasePaths.size(), 0);
    store.deltaValues.assign(geometry.deltaBasePaths.size(),
                             identity);
    store.deltaPresent.assign(geometry.deltaBasePaths.size(), 0);
    store.revisionRan.assign(revisions, 0);
    store.revisionStaticDirty.assign(revisions, 0);
    store.revisionPublish.assign(revisions, RrRevisionPublish());
    for (size_t r = 0; r < revisions; ++r) {
        const auto &entry = geometry.revisionIndex[r];
        store.revisionPublish[r].target =
            geometry.chains[size_t(entry.first)]
                .revisions[size_t(entry.second)]
                .target;
    }
    store.chainPublish.assign(geometry.chains.size(),
                              RrChainPublish());
    for (size_t c = 0; c < geometry.chains.size(); ++c) {
        store.chainPublish[c].target = geometry.chains[c].target;
    }
    store.derivedPublish.assign(geometry.derivedIndex.size(),
                                RrDerivedPublish());
    for (size_t d = 0; d < geometry.derivedIndex.size(); ++d) {
        const auto &entry = geometry.derivedIndex[d];
        store.derivedPublish[d].target =
            geometry.chains[size_t(entry.first)]
                .derived[size_t(entry.second)]
                .target;
    }
    store.weightPackets.assign(program.stepWeightObjects, RrWeightPacket());
    store.volumePlacement.assign(slots, identity);
    store.volumePlaced.assign(slots, 0);
    store.volumePlacementBase.assign(slots, identity);
    store.volumePlacedBase.assign(slots, 0);
    store.frameMatrix.assign(poses.frameRecords.size(), identity);
    store.frameMatrixValid.assign(poses.frameRecords.size(), 0);
    store.poseWeights.assign(poses.poseWeightPaths.size(), 0.0f);
    store.stepOutputs.assign(file.steps.size(), RrStepOutput());
    store.runTrace.reserve(file.steps.size());
    for (size_t j = 0; j < poses.jointBindingJoints.size(); ++j) {
        program.jointBindingIndex[poses.jointBindingJoints[j]] = j;
    }
    int32_t maxOverride = -1;
    for (const RigExecWireStep &step : file.steps) {
        for (int input : step.overrideInputs) {
            maxOverride = std::max(maxOverride, input);
        }
    }
    for (const RrRegisteredRead &entry : program.registeredReads) {
        maxOverride = std::max(maxOverride, entry.read->overrideIndex);
    }
    store.overridden.assign(size_t(maxOverride + 1), 0);
    store.anyOverridden = false;
    store.changedSinceRun.assign(store.overridden.size(), 0);
    store.anyChangedSinceRun = false;
    program.inputState.valueOverridden.assign(store.overridden.size(), 0);

    if (!RrPoseSizeScratch(&program, error)) {
        return fail(error ? *error : std::string("pose sizing failed"));
    }
    if (!RrGeometrySizeScratch(&program, error)) {
        return fail(error ? *error
                          : std::string("geometry sizing failed"));
    }
    // The layout defaults exist once the geometry family expanded them.
    if (!RrInputsFinishArrays(&program, error)) {
        return fail(error ? *error
                          : std::string("an array input has no default"));
    }
    if (!RrWeightSizeScratch(&program, error)) {
        return fail(error ? *error
                          : std::string("weight sizing failed"));
    }
    if (!RrPropertySizeScratch(&program, error)) {
        return fail(error ? *error
                          : std::string("property chain sizing failed"));
    }
    if (!RrOpenProviderProgram(&program, error) ||
        !RrCompileOpGraph(&program, error)) {
        return fail(error ? *error : std::string("common graph initialization failed"));
    }
    return self;
}

std::vector<std::string>
RigExecRuntimeReader::GetExternalMoverTypes() const
{
    std::set<std::string> types;
    for (const RrProgram::ExternalRevision &state : _program->externals) {
        types.insert(state.type);
    }
    return std::vector<std::string>(types.begin(), types.end());
}

bool
RigExecRuntimeReader::SetExternalKernel(const std::string &type,
                                        const RigExecExternalKernel &kernel,
                                        std::string *error)
{
    const auto fail = [error](const std::string &why) {
        if (error) {
            *error = why;
        }
        return false;
    };
    if (!kernel.IsSet()) {
        return fail("a playback kernel needs prepare and apply");
    }
    bool found = false;
    bool prepared = true;
    std::string why;
    for (size_t k = 0; k < _program->externals.size(); ++k) {
        RrProgram::ExternalRevision &state = _program->externals[k];
        if (state.type != type) {
            continue;
        }
        found = true;
        const std::vector<uint8_t> &epoch =
            _program->file->externalMovers[k].epoch;
        std::string reason;
        state.kernel = kernel;
        state.state = kernel.prepare(epoch.data(), epoch.size(), &reason);
        if (!state.state) {
            state.kernel = RigExecExternalKernel();
            if (prepared) {
                why = type + " could not prepare its epoch data" +
                      (reason.empty() ? std::string() : ": " + reason);
            }
            prepared = false;
        }
    }
    if (!found) {
        return fail("the file holds no " + type + " mover");
    }
    return prepared || fail(why);
}

std::vector<std::string>
RigExecRuntimeReader::GetMissingExternalKernels() const
{
    std::set<std::string> types;
    for (const RrProgram::ExternalRevision &state : _program->externals) {
        if (!state.state) {
            types.insert(state.type);
        }
    }
    return std::vector<std::string>(types.begin(), types.end());
}

std::vector<RigExecRuntimeProviderFrames>
RigExecRuntimeReader::GetProviderFramePublications() const
{
    std::vector<RigExecRuntimeProviderFrames> result;
    if(!_program || !_program->slotMeta) return result;
    const auto &meta=*_program->slotMeta;
    const auto &store=_program->store;
    result.reserve(meta.paths.size());
    for(size_t slot=0;slot<meta.paths.size();++slot) {
        RigExecRuntimeProviderFrames value;
        value.path=_program->TextOrEmpty(meta.paths[slot]);
        value.kind=uint8_t(meta.slotKind[slot]);
        value.publicationRole=meta.publicationRoles[slot];
        value.baseVersion=store.baseLast[slot]; value.finalVersion=store.finLast[slot];
        value.base=store.base[value.baseVersion]; value.final=store.fin[value.finalVersion];
        result.push_back(std::move(value));
    }
    return result;
}

std::vector<RigExecRuntimeSolverFrames>
RigExecRuntimeReader::GetSolverFramePublications() const
{
    std::vector<RigExecRuntimeSolverFrames> result;
    if (!_program || !_program->slotMeta || !_program->slotMeta->solverGuidesEnabled) return result;
    const auto &meta = *_program->slotMeta;
    std::vector<char> aliveSolver(_program->poses->solvers.size(), 0);
    for (const RigExecWireStep &step : *_program->steps)
        if (step.kind == RigExecWireStepKind::Solve)
            aliveSolver[size_t(step.object)] = 1;
    for (int index : meta.solverPublishOrder) {
        if (!aliveSolver[size_t(meta.solverArrayElements[size_t(index)])]) continue;
        RigExecRuntimeSolverFrames value;
        value.path = _program->TextOrEmpty(meta.solverArrayPaths[size_t(index)]);
        value.aggregate = uint32_t(meta.solverArrayElements[size_t(index)]);
        value.frames = _program->store.aggregates[value.aggregate].frames;
        result.push_back(std::move(value));
    }
    return result;
}

}  // namespace rigExec
