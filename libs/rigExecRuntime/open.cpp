//
// rigExecRuntime Open (M2 framework).
//
// Decodes every section, then derives what the file implies but does not
// store: the uid routing (replaying the capture traversal), the SSA
// version pool sizes and last-version maps (scanning the commit writes),
// and the store sizing. Cross-references that do not close are Open
// errors naming the table.
//

#include "rigExecRuntime/runtime.h"

#include <algorithm>

namespace rigExec {

namespace {

bool
_RouteField(const RigExecWireInput &input,
            const std::vector<RigExecWireInputDirectoryEntry> &directory,
            size_t *nextUid, int32_t *uid, std::string *error)
{
    if (!input.varying || !input.bound) {
        *uid = -1;
        return true;
    }
    if (*nextUid >= directory.size()) {
        if (error) {
            *error = "input directory ends before the uid replay does";
        }
        return false;
    }
    if (directory[*nextUid].tag != input.tag) {
        if (error) {
            *error = "input directory tag drifted from the tables";
        }
        return false;
    }
    *uid = int32_t(*nextUid);
    ++(*nextUid);
    return true;
}

int
_AvarChannel(const std::string &property)
{
    for (int c = 0; c < 11; ++c) {
        if (property == RrAvarNames[c]) {
            return c;
        }
    }
    return -1;
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
    // Optional since minor 1: absent in older binaries, which load with
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

    const size_t slots = self->_slotMeta.paths.size();
    const size_t clusters = self->_clustering.clusters.size();
    const size_t revisions = self->_geometry.revisionIndex.size();

    // Slot path -> slot index.
    for (size_t i = 0; i < slots; ++i) {
        std::string path;
        if (!reader.GetString(self->_slotMeta.paths[i], &path)) {
            return fail("slot inventory names a missing string");
        }
        program.pathIndex[path] = int(i);
    }

    // Uid routing, replaying the capture traversal in order.
    const std::vector<RigExecWireInputDirectoryEntry> &directory =
        self->_inputs.directory;
    size_t nextUid = 0;
    program.ladderUid.resize(self->_poses.ladders.size());
    for (size_t i = 0; i < self->_poses.ladders.size(); ++i) {
        for (int f = 0; f < RrLadderFieldCount; ++f) {
            if (!_RouteField(program.LadderInput(i, f), directory,
                             &nextUid,
                             &program.ladderUid[i][size_t(f)], error)) {
                return fail(*error);
            }
        }
    }
    program.interpUid.assign(self->_poses.poseInterpolators.size(), -1);
    for (size_t i = 0; i < self->_poses.poseInterpolators.size(); ++i) {
        if (!_RouteField(self->_poses.poseInterpolators[i].enabled,
                         directory, &nextUid, &program.interpUid[i],
                         error)) {
            return fail(*error);
        }
    }
    program.solverUid.resize(self->_poses.solvers.size());
    for (size_t i = 0; i < self->_poses.solvers.size(); ++i) {
        for (int f = 0; f < RrSolverFieldCount; ++f) {
            if (!_RouteField(program.SolverInput(i, f), directory,
                             &nextUid,
                             &program.solverUid[i][size_t(f)], error)) {
                return fail(*error);
            }
        }
    }
    program.constraintUid.resize(self->_poses.constraints.size());
    for (size_t i = 0; i < self->_poses.constraints.size(); ++i) {
        for (int f = 0; f < RrConstraintFieldCount; ++f) {
            if (!_RouteField(program.ConstraintInput(i, f), directory,
                             &nextUid,
                             &program.constraintUid[i][size_t(f)],
                             error)) {
                return fail(*error);
            }
        }
    }
    program.weightUid.resize(self->_geometry.weightObjects.size());
    for (size_t i = 0; i < self->_geometry.weightObjects.size(); ++i) {
        for (int f = 0; f < RrWeightFieldCount; ++f) {
            if (!_RouteField(program.WeightInput(i, f), directory,
                             &nextUid,
                             &program.weightUid[i][size_t(f)], error)) {
                return fail(*error);
            }
        }
    }
    // The directory tail is the avar bindings, routed by head path:
    // "primpath.avars:xx" -> (slot, channel).
    program.avarUidTarget.assign(directory.size(), -1);
    for (size_t uid = nextUid; uid < directory.size(); ++uid) {
        std::string head;
        if (directory[uid].head == 0 ||
            !reader.GetString(directory[uid].head, &head)) {
            return fail("avar directory entry has no routable head");
        }
        const size_t dot = head.find_last_of('.');
        if (dot == std::string::npos) {
            return fail("avar head is not a property path: " + head);
        }
        const auto slot = program.pathIndex.find(head.substr(0, dot));
        const int channel = _AvarChannel(head.substr(dot + 1));
        if (slot == program.pathIndex.end() || channel < 0) {
            return fail("avar head routes nowhere: " + head);
        }
        program.avarUidTarget[uid] = int32_t(slot->second * 11 + channel);
    }

    // Holder seeds: wire constants positionally, avar constants by target.
    RrStore &store = program.store;
    store.inputHolders.assign(directory.size(), RrInputValue());
    for (size_t i = 0; i < program.ladderUid.size(); ++i) {
        for (int f = 0; f < RrLadderFieldCount; ++f) {
            const int32_t uid = program.ladderUid[i][size_t(f)];
            if (uid >= 0) {
                store.inputHolders[size_t(uid)] =
                    RrWireInputConstant(program.LadderInput(i, f));
            }
        }
    }
    for (size_t i = 0; i < program.interpUid.size(); ++i) {
        if (program.interpUid[i] >= 0) {
            store.inputHolders[size_t(program.interpUid[i])] =
                RrWireInputConstant(
                    self->_poses.poseInterpolators[i].enabled);
        }
    }
    for (size_t i = 0; i < program.solverUid.size(); ++i) {
        for (int f = 0; f < RrSolverFieldCount; ++f) {
            const int32_t uid = program.solverUid[i][size_t(f)];
            if (uid >= 0) {
                store.inputHolders[size_t(uid)] =
                    RrWireInputConstant(program.SolverInput(i, f));
            }
        }
    }
    for (size_t i = 0; i < program.constraintUid.size(); ++i) {
        for (int f = 0; f < RrConstraintFieldCount; ++f) {
            const int32_t uid = program.constraintUid[i][size_t(f)];
            if (uid >= 0) {
                store.inputHolders[size_t(uid)] =
                    RrWireInputConstant(program.ConstraintInput(i, f));
            }
        }
    }
    for (size_t i = 0; i < program.weightUid.size(); ++i) {
        for (int f = 0; f < RrWeightFieldCount; ++f) {
            const int32_t uid = program.weightUid[i][size_t(f)];
            if (uid >= 0) {
                store.inputHolders[size_t(uid)] =
                    RrWireInputConstant(program.WeightInput(i, f));
            }
        }
    }
    if (self->_constants.avarConstants.size() != slots * 11) {
        return fail("avar constants do not cover the slots");
    }
    for (size_t uid = 0; uid < directory.size(); ++uid) {
        const int32_t target = program.avarUidTarget[uid];
        if (target >= 0) {
            store.inputHolders[uid].tag =
                RigExecWireInput::Tag::Double;
            store.inputHolders[uid].f64 =
                self->_constants.avarConstants[size_t(target)];
        }
    }
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
    auto noteFin = [&](uint32_t v) { finMax = std::max(finMax, v); };
    auto noteBase = [&](uint32_t v) { baseMax = std::max(baseMax, v); };
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
    if (size_t(finMax) >= finPool || size_t(baseMax) >= basePool) {
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
    for (const RigExecWireStep &step : self->_steps) {
        if (step.cluster < 0 || size_t(step.cluster) >= clusters) {
            return fail("a step names no cluster");
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
    for (size_t i = 0;
         i < std::min(slots, self->_poses.ladders.size()); ++i) {
        const RigExecWireLadder &ladder = self->_poses.ladders[i];
        RrLadderLive &live = store.ladders[i];
        live.restSpace = RrWireInputConstant(ladder.restSpace).matrix;
        live.defaultSpace =
            RrWireInputConstant(ladder.defaultSpace).matrix;
        live.posedSpace = RrWireInputConstant(ladder.posedSpace).matrix;
        for (int k = 0; k < 6; ++k) {
            live.restAvars[k] = ladder.restAvars[k].f64;
            live.defaultAvars[k] = ladder.defaultAvars[k].f64;
        }
        live.rotationOrder = ladder.rotationOrder.token;
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
    for (const auto &entry : directory) {
        maxOverride = std::max(maxOverride, entry.overrideIndex);
    }
    store.overridden.assign(size_t(maxOverride + 1), 0);
    store.lastOverridden.assign(size_t(maxOverride + 1), 0);

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
    return self;
}

}  // namespace rigExec
