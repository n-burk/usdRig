// rigExecRuntime Open (M2 framework).
// Decodes every section, then derives what the file implies but does not
// store: the input reads each table field binds, the SSA version pool
// sizes and last-version maps (scanning the commit writes), and the store
// sizing. Cross-references that do not close are Open errors naming the
// table, and so is a step or cluster graph the index walk cannot follow.
#include "rigExecRuntime/runtime.h"

#include "rigExecBinary/stepGraph.h"

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <set>

namespace rigExec {

namespace {

/// RIGEXEC_RUNTIME_CROSSCHECK, read once per Open: set to anything but
/// empty or "0" turns the record cross-check on.
bool
_CrossCheckFromEnvironment()
{
    const char *name = "RIGEXEC_RUNTIME_CROSSCHECK";
#ifdef _MSC_VER
    char *value = nullptr;
    size_t length = 0;
    if (_dupenv_s(&value, &length, name) != 0 || !value) {
        return false;
    }
    const bool on = value[0] != '\0' && std::strcmp(value, "0") != 0;
    std::free(value);
    return on;
#else
    const char *value = std::getenv(name);
    return value && value[0] != '\0' && std::strcmp(value, "0") != 0;
#endif
}

// Minimal manifest reader for the "compileDiagnostics" string array. The
// writer is _EscapeJson in rigExecBake/bake.cpp; this mirrors its escapes.
// A manifest without the key (binaries baked before it) reads as empty;
// a present-but-malformed array fails the open.
bool
_ParseManifestDiagnostics(const char *begin, const char *end,
                          std::vector<std::string> *out,
                          std::string *error)
{
    auto fail = [&](const std::string &why) {
        if (error) {
            *error = why;
        }
        return false;
    };
    static const char key[] = "\"compileDiagnostics\"";
    const char *at =
        std::search(begin, end, key, key + sizeof(key) - 1);
    if (at == end) {
        return true;
    }
    at += sizeof(key) - 1;
    auto skipWs = [&]() {
        while (at != end && (*at == ' ' || *at == '\t' || *at == '\n' ||
                             *at == '\r')) {
            ++at;
        }
    };
    skipWs();
    if (at == end || *at != ':') {
        return fail("malformed manifest section");
    }
    ++at;
    skipWs();
    if (at == end || *at != '[') {
        return fail("malformed manifest section");
    }
    ++at;
    for (;;) {
        skipWs();
        if (at != end && *at == ']') {
            return true;
        }
        if (at == end || *at != '"') {
            return fail("malformed manifest section");
        }
        ++at;
        std::string line;
        while (at != end && *at != '"') {
            if (*at != '\\') {
                line.push_back(*at++);
                continue;
            }
            ++at;
            if (at == end) {
                return fail("malformed manifest section");
            }
            switch (*at) {
            case '"':
                line.push_back('"');
                ++at;
                break;
            case '\\':
                line.push_back('\\');
                ++at;
                break;
            case 'n':
                line.push_back('\n');
                ++at;
                break;
            case 'r':
                line.push_back('\r');
                ++at;
                break;
            case 't':
                line.push_back('\t');
                ++at;
                break;
            case 'u': {
                // _EscapeJson only ever emits \u00XX for C0 controls.
                unsigned code = 0;
                for (int i = 0; i < 4; ++i) {
                    ++at;
                    if (at == end) {
                        return fail("malformed manifest section");
                    }
                    code <<= 4;
                    if (*at >= '0' && *at <= '9') {
                        code |= unsigned(*at - '0');
                    } else if (*at >= 'a' && *at <= 'f') {
                        code |= unsigned(*at - 'a' + 10);
                    } else if (*at >= 'A' && *at <= 'F') {
                        code |= unsigned(*at - 'A' + 10);
                    } else {
                        return fail("malformed manifest section");
                    }
                }
                ++at;
                if (code < 0x80) {
                    line.push_back(char(code));
                } else if (code < 0x800) {
                    line.push_back(char(0xC0 | (code >> 6)));
                    line.push_back(char(0x80 | (code & 0x3F)));
                } else {
                    line.push_back(char(0xE0 | (code >> 12)));
                    line.push_back(char(0x80 | ((code >> 6) & 0x3F)));
                    line.push_back(char(0x80 | (code & 0x3F)));
                }
                break;
            }
            default:
                return fail("malformed manifest section");
            }
        }
        if (at == end) {
            return fail("malformed manifest section");
        }
        ++at;  // closing quote
        out->push_back(std::move(line));
        skipWs();
        if (at != end && *at == ',') {
            ++at;
            continue;
        }
        if (at != end && *at == ']') {
            return true;
        }
        return fail("malformed manifest section");
    }
}

}  // namespace

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
    self->_reader = RigExecBinaryReader::Open(bytes, size, error);
    if (!self->_reader) {
        return fail(error ? *error : std::string("unreadable file"));
    }
    const RigExecBinaryReader &reader = *self->_reader;
    auto section = [&](RigExecBinarySection tag, const char *name,
                       const uint8_t **data, size_t *bytesOut) {
        if (!reader.FindSection(tag, data, bytesOut)) {
            if (error) {
                *error = std::string("missing ") + name + " section";
            }
            return false;
        }
        return true;
    };
    const uint8_t *data = nullptr;
    size_t bytesOut = 0;
    if (!section(RigExecBinarySection::Steps, "steps", &data, &bytesOut)) {
        return fail(*error);
    }
    {
        RigExecWireReader cursor(data, bytesOut);
        if (!RigExecWireDecodeSteps(&cursor, &self->_steps, error) ||
            !cursor.Exhausted()) {
            return fail("malformed steps section");
        }
        self->_hasSteps = true;
    }
    if (section(RigExecBinarySection::Clusters, "clusters", &data,
                &bytesOut)) {
        RigExecWireReader cursor(data, bytesOut);
        if (!RigExecWireDecodeClustering(&cursor, &self->_clustering,
                                         error) ||
            !cursor.Exhausted()) {
            return fail("malformed clusters section");
        }
        self->_hasClusters = true;
    }
    if (section(RigExecBinarySection::Cones, "cones", &data, &bytesOut)) {
        RigExecWireReader cursor(data, bytesOut);
        if (!RigExecWireDecodeCones(&cursor, &self->_cones, error) ||
            !cursor.Exhausted()) {
            return fail("malformed cones section");
        }
        self->_hasCones = true;
    }
    if (section(RigExecBinarySection::SlotMeta, "slot inventory", &data,
                &bytesOut)) {
        RigExecWireReader cursor(data, bytesOut);
        if (!RigExecWireDecodeSlotMeta(&cursor, &self->_slotMeta, error) ||
            !cursor.Exhausted()) {
            return fail("malformed slot inventory section");
        }
        self->_hasSlotMeta = true;
    }
    if (section(RigExecBinarySection::Constants, "constants", &data,
                &bytesOut)) {
        RigExecWireReader cursor(data, bytesOut);
        if (!RigExecWireDecodeConstants(&cursor, &self->_constants,
                                        error) ||
            !cursor.Exhausted()) {
            return fail("malformed constants section");
        }
        self->_hasConstants = true;
    }
    if (section(RigExecBinarySection::DomainPose, "pose domain", &data,
                &bytesOut)) {
        RigExecWireReader cursor(data, bytesOut);
        if (!RigExecWireDecodeDomainPose(&cursor, &self->_poses, error) ||
            !cursor.Exhausted()) {
            return fail("malformed pose-domain section");
        }
        self->_hasPoses = true;
    }
    // Optional since major 1 minor 1: absent in older binaries, which load with
    // every chain absolute. Present but dangling or malformed: fail.
    if (section(RigExecBinarySection::SolverStart, "solver starts", &data,
                &bytesOut)) {
        RigExecWireReader cursor(data, bytesOut);
        std::vector<RigExecWireSolverStart> starts;
        if (!RigExecWireDecodeSolverStarts(&cursor, &starts, error) ||
            !cursor.Exhausted()) {
            return fail("malformed solver-start section");
        }
        for (const RigExecWireSolverStart &entry : starts) {
            if (size_t(entry.solver) >= self->_poses.solvers.size() ||
                entry.start < 0) {
                return fail("malformed solver-start section");
            }
            RigExecWireSolver &solver =
                self->_poses.solvers[size_t(entry.solver)];
            solver.start = entry.start;
            solver.startRest = entry.rest;
            solver.startRead = entry.read;
        }
    }
    // Optional since minor 1: absent in older binaries, which load with
    // every interpolator solving a transform's rotation alone. Present
    // but dangling or malformed: fail.
    if (section(RigExecBinarySection::PoseNumeric, "pose numerics", &data,
                &bytesOut)) {
        RigExecWireReader cursor(data, bytesOut);
        std::vector<RigExecWirePoseNumeric> numerics;
        if (!RigExecWireDecodePoseNumerics(&cursor, &numerics, error) ||
            !cursor.Exhausted()) {
            return fail("malformed pose-numeric section");
        }
        for (const RigExecWirePoseNumeric &entry : numerics) {
            if (size_t(entry.interpolator) >=
                self->_poses.poseInterpolators.size()) {
                return fail("malformed pose-numeric section");
            }
            RigExecWirePoseInterpolator &interp =
                self->_poses.poseInterpolators[size_t(entry.interpolator)];
            // A numeric driver names no driver prim; a transform-driven
            // one names a slot. A file that says both is corrupt.
            if (!entry.values.empty() && interp.driverSlot >= 0) {
                return fail("malformed pose-numeric section");
            }
            interp.enableTranslation = entry.enableTranslation;
            interp.valueInputs = entry.values;
        }
    }
    // Optional since minor 1, in the same way: absent means every
    // provider composes against its authored parent.
    if (section(RigExecBinarySection::SpaceSwitch, "space switches", &data,
                &bytesOut)) {
        RigExecWireReader cursor(data, bytesOut);
        if (!RigExecWireDecodeSpaceSwitches(&cursor,
                                            self->_slotMeta.parent,
                                            &self->_poses.spaceSwitches,
                                            error) ||
            !cursor.Exhausted()) {
            return fail("malformed space-switch section");
        }
    }
    if (section(RigExecBinarySection::DomainGeometry, "geometry domain",
                &data, &bytesOut)) {
        RigExecWireReader cursor(data, bytesOut);
        if (!RigExecWireDecodeDomainGeometry(&cursor, &self->_geometry,
                                             error) ||
            !cursor.Exhausted()) {
            return fail("malformed geometry-domain section");
        }
        self->_hasGeometry = true;
    }
    if (!section(RigExecBinarySection::InputTable, "input table", &data,
                 &bytesOut)) {
        return fail(*error);
    }
    {
        RigExecWireReader cursor(data, bytesOut);
        if (!RigExecWireDecodeInputTable(&cursor, &self->_inputs, error) ||
            !cursor.Exhausted()) {
            return fail("malformed input-table section");
        }
        self->_hasInputTable = true;
    }
    // The Computed section (temporary): the input slots every read the
    // steps make is evaluated over, and the weight objects and property
    // chains the runtime computes from them. Absent, malformed, or not
    // matching the tables it extends: fail (RrInputsOpen below refuses a
    // file without it).
    bool hasComputed = false;
    if (reader.FindSection(RigExecBinarySection::Computed, &data,
                           &bytesOut)) {
        RigExecWireReader cursor(data, bytesOut);
        std::string why;
        if (!RigExecWireDecodeComputed(&cursor, &self->_computed, &why)) {
            return fail("malformed computed section: " + why);
        }
        if (!RigExecWireApplyComputed(self->_computed, self->_geometry,
                                      self->_inputs, &self->_poses, &why)) {
            return fail(why);
        }
        hasComputed = true;
    }
    // Plugin movers' bytes, since minor 3. Every op-16 revision needs its
    // entry and every entry an op-16 revision, with one frame row per
    // InputTable frame; anything else is a file that does not close.
    bool hasExternal = false;
    if (reader.FindSection(RigExecBinarySection::ExternalMovers, &data,
                           &bytesOut)) {
        RigExecWireReader cursor(data, bytesOut);
        if (!RigExecWireDecodeExternalMovers(&cursor, &self->_external,
                                             error)) {
            return fail(*error);
        }
        if (self->_external.frames.size() != self->_inputs.frames.size()) {
            return fail("external movers carry a frame count the input "
                        "table does not");
        }
        hasExternal = true;
    }
    {
        RrProgram &program = self->_program;
        for (const RigExecWireExternalRevision &entry :
             self->_external.revisions) {
            const auto &chains = self->_geometry.chains;
            if (size_t(entry.chain) >= chains.size() ||
                size_t(entry.revision) >=
                    chains[entry.chain].revisions.size() ||
                chains[entry.chain].revisions[entry.revision].op !=
                    uint8_t(RigExecWireExternalRevisionOp)) {
                return fail("an external mover entry names no plugin "
                            "revision");
            }
            RrProgram::ExternalRevision state;
            if (!reader.GetString(entry.type, &state.type) ||
                state.type.empty()) {
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
        for (size_t c = 0; c < self->_geometry.chains.size(); ++c) {
            const auto &revisions = self->_geometry.chains[c].revisions;
            for (size_t r = 0; r < revisions.size(); ++r) {
                if (revisions[r].op ==
                        uint8_t(RigExecWireExternalRevisionOp) &&
                    !program.externalIndex.count(
                        std::make_pair(uint32_t(c), uint32_t(r)))) {
                    return fail("a plugin revision carries no external "
                                "mover entry");
                }
            }
        }
        program.external = hasExternal ? &self->_external : nullptr;
    }
    // The manifest is advisory except for its compile-diagnostics seed,
    // which the first Execute replays. Absent (or seedless) in binaries
    // baked before the key: not an error. Present but malformed: fail.
    if (reader.FindSection(RigExecBinarySection::Manifest, &data,
                           &bytesOut)) {
        const char *text = reinterpret_cast<const char *>(data);
        if (!_ParseManifestDiagnostics(
                text, text + bytesOut,
                &self->_program.compileDiagnostics, error)) {
            return fail(*error);
        }
    }
    // Playback walks the steps in index order and dirties whole clusters
    // without reading an edge, so the file's own graph must make that walk
    // a topological order, with every read produced before it, before
    // anything is sized from it.
    {
        const std::string why = RigExecStepGraphError(
            self->_steps, self->_clustering,
            [](const RigExecWireSlotRange &range) {
                return RigExecStepGraphRange{uint8_t(range.domain),
                                             range.begin, range.end};
            });
        if (!why.empty()) {
            return fail(why);
        }
    }

    RrProgram &program = self->_program;
    program.steps = &self->_steps;
    program.clustering = &self->_clustering;
    program.cones = &self->_cones;
    program.slotMeta = &self->_slotMeta;
    program.constants = &self->_constants;
    program.poses = &self->_poses;
    program.geometry = &self->_geometry;
    program.inputs = &self->_inputs;
    program.strings = self->_reader.get();
    program.crossCheck = _CrossCheckFromEnvironment();
    const size_t slots = self->_slotMeta.paths.size();
    const size_t clusters = self->_clustering.clusters.size();
    const size_t revisions = self->_geometry.revisionIndex.size();
    if (self->_constants.avarConstants.size() != slots * 11) {
        return fail("avar constants do not cover the slots");
    }
    // The input reads: each table field the steps read binds one of the
    // section's registered reads.
    {
        std::string why;
        if (!RrInputsOpen(&program, hasComputed ? &self->_computed : nullptr,
                          &why) ||
            !RrInputsBindReads(&program, &why)) {
            return fail(why);
        }
    }

    // Switch index by provider slot, dense, so the compose reads one int
    // per slot. Left empty when the rig has no switch, which is the
    // branch the compose tests first.
    if (!self->_poses.spaceSwitches.empty()) {
        const RigExecWireSlotMeta &meta = self->_slotMeta;
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
        for (size_t i = 0; i < self->_poses.spaceSwitches.size(); ++i) {
            const RigExecWireSpaceSwitch &sw =
                self->_poses.spaceSwitches[i];
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
                !versionInRange(sw.parentRead) ||
                !versionInRange(sw.spaceRead)) {
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
                sw.spaceSlot < 0 && !unread(sw.spaceRead);
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
    store.avars = self->_constants.avarConstants;
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
    for (const RigExecWireCommit &commit : self->_poses.commits) {
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
            for (const RigExecWireAncestorRead &read : list) {
                noteFin(read.fin);
                noteBase(read.base);
            }
        }
        for (const auto &list : commit.sourceAncestors) {
            for (const RigExecWireAncestorRead &read : list) {
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
    for (const RigExecWireSolver &solver : self->_poses.solvers) {
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

    // Cone table shapes, indexed blindly by the closure.
    const RigExecWireCones &cones = self->_cones;
    const size_t words = (clusters + 63) / 64;
    if (cones.cone.size() != clusters ||
        cones.always.words.size() != words ||
        cones.poseClusters.words.size() != words ||
        cones.avarCluster.size() != slots ||
        cones.chainBaseClusters.size() != self->_geometry.chains.size() ||
        cones.solverPointsClusters.size() != self->_poses.solvers.size() ||
        cones.revisionClusters.size() != revisions ||
        cones.revisionStaticCluster.size() != revisions ||
        cones.nativeSourceClusters.size() !=
            self->_poses.nativeSources.size() ||
        cones.deltaBaseClusters.size() !=
            self->_geometry.deltaBasePaths.size() ||
        cones.constraintArrayClusters.size() !=
            self->_poses.constraintArrays.size()) {
        return fail("cone lookup tables do not match the program");
    }
    for (const RigExecWireClusterSet &set : cones.cone) {
        if (set.words.size() != words) {
            return fail("a cone closure has the wrong word count");
        }
    }
    // The clusters the dirty sources name, which the closure sets blindly.
    // The program fills every one from a step's cluster.
    const auto clusterInRange = [clusters](int32_t cluster) {
        return cluster >= 0 && size_t(cluster) < clusters;
    };
    bool coneClustersInRange =
        std::all_of(cones.avarCluster.begin(), cones.avarCluster.end(),
                    clusterInRange) &&
        std::all_of(cones.revisionStaticCluster.begin(),
                    cones.revisionStaticCluster.end(), clusterInRange);
    for (const std::vector<std::vector<int32_t>> *lists :
         {&cones.chainBaseClusters, &cones.solverPointsClusters,
          &cones.revisionClusters, &cones.nativeSourceClusters,
          &cones.deltaBaseClusters, &cones.constraintArrayClusters}) {
        for (const std::vector<int32_t> &list : *lists) {
            coneClustersInRange =
                coneClustersInRange &&
                std::all_of(list.begin(), list.end(), clusterInRange);
        }
    }
    if (!coneClustersInRange) {
        return fail("a cone lookup table names no cluster");
    }
    // A compose step's Avars reads outside its own group: the slots a
    // switch recomposes an earlier version of (RigExecBakedCones::
    // avarVersionSteps, built from the same reads).
    for (const RigExecWireStep &step : self->_steps) {
        if (step.kind != RigExecWireStepKind::ComposeSubtree ||
            step.object < 0 ||
            size_t(step.object) >= self->_poses.composeGroups.size()) {
            continue;
        }
        const RigExecWireComposeGroup &group =
            self->_poses.composeGroups[size_t(step.object)];
        for (const RigExecWireSlotRange &range : step.reads) {
            if (range.domain != RigExecWireSlotDomain::Avars) {
                continue;
            }
            for (uint64_t slot = range.begin / 11;
                 slot < (uint64_t(range.end) + 10) / 11 && slot < slots;
                 ++slot) {
                if (int64_t(slot) >= group.begin &&
                    int64_t(slot) < group.end) {
                    continue;
                }
                if (program.avarVersionClusters.empty()) {
                    program.avarVersionClusters.assign(slots, {});
                }
                program.avarVersionClusters[size_t(slot)].push_back(
                    step.cluster);
            }
        }
    }
    for (int index : cones.varyingSteps) {
        if (index < 0 || size_t(index) >= self->_steps.size()) {
            return fail("a varying step names no step");
        }
    }
    for (int index : cones.overrideSteps) {
        if (index < 0 || size_t(index) >= self->_steps.size()) {
            return fail("an override step names no step");
        }
    }

    // Store sizing.
    RrMat4d identity;
    identity.SetIdentity();
    store.posedM.assign(slots, identity);
    store.finalMatrix.assign(slots, identity);
    store.baseMatrix.assign(slots, identity);
    store.aggregates.assign(self->_poses.solvers.size(),
                            RrPointFrameArray());
    store.ladders.assign(slots, RrLadderLive());
    // Seeded with the reads' folded constants until a prologue composes.
    for (size_t i = 0;
         i < std::min(slots, self->_poses.ladders.size()); ++i) {
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
    store.solverOutFrames.assign(self->_poses.solvers.size(), {});
    store.solverOutPresent.assign(self->_poses.solvers.size(), {});
    for (size_t i = 0; i < self->_poses.solvers.size(); ++i) {
        store.solverOutFrames[i].assign(
            self->_poses.solvers[i].outputs.size(), RrPointFrame());
        store.solverOutPresent[i].assign(
            self->_poses.solvers[i].outputs.size(), 0);
    }
    store.ribbonPoints.assign(self->_poses.solvers.size(), {});
    store.ribbonLast.assign(self->_poses.solvers.size(), {});
    store.ribbonConstant.assign(self->_poses.solvers.size(), {});
    store.ribbonVarying.assign(self->_poses.solvers.size(), 0);
    store.ribbonDirty.assign(self->_poses.solvers.size(), 0);
    for (size_t i = 0; i < self->_poses.solvers.size(); ++i) {
        const RigExecWireSolver &solver = self->_poses.solvers[i];
        for (const RigExecWireVec3f &p : solver.ribbonConstantPoints) {
            store.ribbonConstant[i].push_back(
                RrVec3f(p[0], p[1], p[2]));
        }
        store.ribbonVarying[i] = solver.ribbonPointsVarying ? 1 : 0;
    }
    store.commits.assign(self->_poses.commits.size(), RrCommitScratch());
    for (size_t i = 0; i < self->_poses.commits.size(); ++i) {
        const RigExecWireCommit &commit = self->_poses.commits[i];
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
    store.arrays.assign(self->_poses.constraintArrays.size(),
                        RrConstraintArraysLive());
    for (size_t i = 0; i < self->_poses.constraintArrays.size(); ++i) {
        store.arrays[i].readPole =
            self->_poses.constraintArrays[i].readPole;
    }
    store.chainHaveBase.assign(self->_geometry.chains.size(), 0);
    store.chainBaseDirty.assign(self->_geometry.chains.size(), 0);
    store.lastHaveBase.assign(self->_geometry.chains.size(), 0);
    store.chainBases.assign(self->_geometry.chains.size(), {});
    store.derivedHaveBase.assign(self->_geometry.derivedIndex.size(), 0);
    store.derivedBases.assign(self->_geometry.derivedIndex.size(), {});
    store.nativeFrames.assign(self->_poses.nativeSources.size(),
                              RrPointFrame());
    store.lastNativeFrames.assign(self->_poses.nativeSources.size(),
                                  RrPointFrame());
    store.nativeFrameOk.assign(self->_poses.nativeSources.size(), 0);
    store.lastNativeFrameOk.assign(self->_poses.nativeSources.size(), 0);
    store.xformBase.assign(self->_slotMeta.xformSlots.size(), identity);
    store.lastXformBase.assign(self->_slotMeta.xformSlots.size(),
                               identity);
    store.deltaBaseMatrix.assign(self->_geometry.deltaBasePaths.size(),
                                 identity);
    store.lastDeltaBaseMatrix.assign(self->_geometry.deltaBasePaths.size(),
                                     identity);
    store.deltaBaseOk.assign(self->_geometry.deltaBasePaths.size(), 0);
    store.lastDeltaBaseOk.assign(self->_geometry.deltaBasePaths.size(),
                                 0);
    store.deltaValues.assign(self->_geometry.deltaBasePaths.size(),
                             identity);
    store.deltaPresent.assign(self->_geometry.deltaBasePaths.size(), 0);
    store.revisionRan.assign(revisions, 0);
    store.revisionStaticDirty.assign(revisions, 0);
    store.revisionPublish.assign(revisions, RrRevisionPublish());
    for (size_t r = 0; r < revisions; ++r) {
        const auto &entry = self->_geometry.revisionIndex[r];
        store.revisionPublish[r].target =
            self->_geometry.chains[size_t(entry.first)]
                .revisions[size_t(entry.second)]
                .target;
    }
    store.chainPublish.assign(self->_geometry.chains.size(),
                              RrChainPublish());
    for (size_t c = 0; c < self->_geometry.chains.size(); ++c) {
        store.chainPublish[c].target = self->_geometry.chains[c].target;
    }
    store.derivedPublish.assign(self->_geometry.derivedIndex.size(),
                                RrDerivedPublish());
    for (size_t d = 0; d < self->_geometry.derivedIndex.size(); ++d) {
        const auto &entry = self->_geometry.derivedIndex[d];
        store.derivedPublish[d].target =
            self->_geometry.chains[size_t(entry.first)]
                .derived[size_t(entry.second)]
                .target;
    }
    store.weightPackets.assign(self->_geometry.weightObjects.size(),
                               RrWeightPacket());
    store.poseWeights.assign(self->_poses.poseWeightPaths.size(), 0.0f);
    store.stepOutputs.assign(self->_steps.size(), RrStepOutput());
    for (size_t j = 0; j < self->_poses.jointBindingJoints.size(); ++j) {
        program.jointBindingIndex[self->_poses.jointBindingJoints[j]] = j;
    }
    int32_t maxOverride = -1;
    for (const RigExecWireStep &step : self->_steps) {
        for (int input : step.overrideInputs) {
            maxOverride = std::max(maxOverride, input);
        }
    }
    for (const RigExecWireRegisteredRead &entry :
         self->_computed.registeredReads) {
        maxOverride = std::max(maxOverride, entry.read.overrideIndex);
    }
    store.overridden.assign(size_t(maxOverride + 1), 0);
    store.lastOverridden.assign(size_t(maxOverride + 1), 0);
    store.anyOverridden = false;

    if (!RrPoseSizeScratch(&program, error)) {
        return fail(error ? *error : std::string("pose sizing failed"));
    }
    if (!RrGeometrySizeScratch(&program, error)) {
        return fail(error ? *error
                          : std::string("geometry sizing failed"));
    }
    if (!RrWeightSizeScratch(&program, error)) {
        return fail(error ? *error
                          : std::string("weight sizing failed"));
    }
    if (!RrPropertySizeScratch(&program, error)) {
        return fail(error ? *error
                          : std::string("property chain sizing failed"));
    }
    return self;
}

std::vector<std::string>
RigExecRuntimeReader::GetExternalMoverTypes() const
{
    std::set<std::string> types;
    for (const RrProgram::ExternalRevision &state : _program.externals) {
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
    for (size_t k = 0; k < _program.externals.size(); ++k) {
        RrProgram::ExternalRevision &state = _program.externals[k];
        if (state.type != type) {
            continue;
        }
        found = true;
        const std::vector<uint8_t> &epoch = _external.revisions[k].epoch;
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
    for (const RrProgram::ExternalRevision &state : _program.externals) {
        if (!state.state) {
            types.insert(state.type);
        }
    }
    return std::vector<std::string>(types.begin(), types.end());
}

}  // namespace rigExec
