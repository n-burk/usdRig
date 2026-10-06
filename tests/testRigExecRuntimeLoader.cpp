// rigExecRuntime loader conformance: bake every baking fixture in-process
// at the probe time, open the bytes with the zero-USD reader, and check
// every step's label as the reader names it in error text against the
// program's, and malformed-input refusal, including a truncated file, the
// old container, another file identifier or format version, a step graph
// with a predecessor after its step, step and cluster indices past their
// tables, a cluster graph with a cycle, static tables that do not match
// the program, and point bindings out of range, of another count or input,
// or read by a step that does not declare them. The phase fixtures under
// tests/fixtures (frame records, solver records, per-volume placements)
// bake at time 1 with every label matching, and refuse frame records and
// record lists out of range, out of order or undeclared, the retired
// snapshot domain and step, the whole-map placement, a placement of a slot
// that is no volume, and the previous format version with its re-export
// message. Skin layouts the sparse form cannot hold are stored raw, with
// their authored arrays, and play back as live baked runs them; Open
// refuses layouts and chunk tables edited out of their forms. Then: a
// fresh reader's defaults against a fresh evaluator at the bake time
// (every output, the diagnostics and the counters), twice,
// and the input API -- the inputs' names, lookup and defaults, the table's
// control avar and operator input found as inputs, refusals, a set that
// moves the outputs, a reset, and token text the file lacks or holds as
// the empty token. Last, on a bake at the table's first frame:
// the input sampler drives a fresh reader along the table's frames (an
// `inputs` row) or at the bake time (a `static` row), each run against a
// fresh evaluator's generation at that time, counters and summary line
// included.
#include "rigExecBake/bake.h"
#include "rigExec/bakedProgram.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/moverGraph.h"
#include "rigExec/rigEvaluator.h"
#include "rigExecBinary/format.h"
#include "rigExecRuntime/runtime.h"
#include "rigExecExampleFixtures.h"

#include "pxr/base/plug/registry.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE

static int failures = 0;

#define CHECK(cond)                                                     \
    do {                                                                \
        if (!(cond)) {                                                  \
            ++failures;                                                 \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);  \
        }                                                               \
    } while (0)

#include "rigExecFileEdit.h"
#include "rigExecRuntimeDrive.h"

static SdfPath
_FindRig(const UsdStageRefPtr &stage)
{
    for (const UsdPrim &prim : stage->TraverseAll()) {
        if (prim.GetTypeName() == TfToken("RigExecRoot")) {
            return prim.GetPath();
        }
    }
    return SdfPath();
}

static std::vector<double>
_ParseFrames(const std::string &text)
{
    std::vector<double> frames;
    size_t begin = 0;
    while (begin <= text.size()) {
        const size_t end = text.find(",", begin);
        const std::string piece =
            text.substr(begin, end == std::string::npos
                        ? std::string::npos : end - begin);
        if (!piece.empty()) {
            frames.push_back(std::stod(piece));
        }
        if (end == std::string::npos) {
            break;
        }
        begin = end + 1;
    }
    return frames;
}

static void
_TestMalformed()
{
    std::string error;
    // Empty input.
    CHECK(!RigExecRuntimeReader::Open(nullptr, 0, &error));
    CHECK(!error.empty());
    // Truncated garbage.
    const std::vector<uint8_t> garbage = {'R', 'E', 'X', 'B', 1, 2, 3};
    CHECK(!RigExecRuntimeReader::Open(garbage.data(), garbage.size(),
                                      &error));
    CHECK(!error.empty());
    // Wrong magic, plausible size.
    const std::vector<uint8_t> wrong(256, 0);
    CHECK(!RigExecRuntimeReader::Open(wrong.data(), wrong.size(), &error));
    CHECK(!error.empty());
    // The old sectioned container's magic leads the file.
    std::vector<uint8_t> old(64, 0);
    old[0] = 'R';
    old[1] = 'E';
    old[2] = 'X';
    old[3] = 'B';
    old[4] = 3;
    old[6] = 3;
    error.clear();
    CHECK(!RigExecRuntimeReader::Open(old.data(), old.size(), &error));
    CHECK(error == "not a v4 .rigexec (old REXB container); rebake");
}

// What Open says about \p bytes, or "(opened)".
static std::string
_OpenError(const std::vector<uint8_t> &bytes)
{
    std::string error;
    const std::unique_ptr<RigExecRuntimeReader> reader =
        RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &error);
    return reader ? std::string("(opened)") : error;
}

static int fileRefusalRows = 0;

// A real bake with another file identifier, and with another format
// version, each refused with its exact message.
static void
_TestFileRefusals(const std::vector<uint8_t> &bytes)
{
    std::vector<uint8_t> identifier = bytes;
    CHECK(identifier.size() > 8);
    if (identifier.size() <= 8) {
        return;
    }
    identifier[4] = 'R';
    identifier[5] = 'I';
    identifier[6] = 'G';
    identifier[7] = 'X';
    std::string got = _OpenError(identifier);
    CHECK(got == "not a .rigexec file (file identifier is not REXB)");
    const std::vector<uint8_t> version =
        RigExecTestEdited(bytes, [](fb::RigExecWireFile *file) {
            file->formatVersion = 4;
        });
    got = _OpenError(version);
    CHECK(got == "unsupported .rigexec format version 4 (this reader reads " +
                     std::to_string(RigExecFormatVersion) + "); rebake");
    if (got.find("format version 4") == std::string::npos) {
        std::printf("version 4: open said '%s'\n", got.c_str());
    }
    ++fileRefusalRows;
}

static int stepGraphFlips = 0;
static int stepGraphRanges = 0;
static int stepGraphCycles = 0;
static int stepGraphProducers = 0;

// Open refuses \p bytes changed by \p edit, saying exactly \p expected
// after the validator's prefix.
template <class Edit>
static void
_ExpectRefusal(const std::string &name, const char *what,
               const std::vector<uint8_t> &bytes, const Edit &edit,
               const std::string &expected)
{
    const std::string want = "invalid .rigexec: " + expected;
    const std::string got = _OpenError(RigExecTestEdited(bytes, edit));
    CHECK(got == want);
    if (got != want) {
        std::printf("%s, %s: open said '%s', expected '%s'\n", name.c_str(),
                    what, got.c_str(), want.c_str());
    }
}

// Step and cluster graphs playback could not walk, edited into a fixture's
// bake, each refused by Open with the step-graph check's exact message:
// the first step with a predecessor names the last step instead; indices
// past the end in a step's predecessors, a step's cluster, a cluster's
// members and a cluster's predecessors, and a clustering that places one
// step too few; and two clusters joined by an edge given the reverse edge
// too.
static void
_TestStepGraphRefusals(const std::string &name,
                       const std::vector<uint8_t> &bytes)
{
    const std::unique_ptr<fb::RigExecWireFile> file = RigExecTestUnpack(bytes);
    if (!file) {
        return;
    }
    const std::vector<fb::RigExecWireStep> &steps = file->steps;
    const fb::RigExecWireClustering &clustering = *file->clustering;
    const size_t count = steps.size();
    const auto &clusters = clustering.clusters;
    const std::string stepCount = std::to_string(count);
    const std::string clusterCount = std::to_string(clusters.size());
    for (size_t s = 0; s + 1 < count; ++s) {
        if (steps[s].preds.empty()) {
            continue;
        }
        _ExpectRefusal(name, "flipped predecessor", bytes,
                       [&](fb::RigExecWireFile *edited) {
                           edited->steps[s].preds[0] = int32_t(count - 1);
                       },
                       "step " + std::to_string(s) +
                           " depends on later step " +
                           std::to_string(count - 1));
        _ExpectRefusal(name, "predecessor past the steps", bytes,
                       [&](fb::RigExecWireFile *edited) {
                           edited->steps[s].preds[0] = int32_t(count);
                       },
                       "step " + std::to_string(s) + " names predecessor " +
                           stepCount + ", which is no step");
        ++stepGraphFlips;
        ++stepGraphRanges;
        break;
    }
    // A read past every slot, in a pose domain and in Aggregate, which
    // the blend reads from the same run: nothing writes either.
    if (count > 0) {
        const uint32_t past = uint32_t(1) << 30;
        const std::string last = std::to_string(count - 1);
        const std::pair<fb::SlotDomain, const char *> domains[] = {
            {fb::SlotDomain::PosedM, "PosedM"},
            {fb::SlotDomain::Aggregate, "Aggregate"}};
        for (const auto &[domain, text] : domains) {
            const fb::SlotDomain read = domain;
            _ExpectRefusal(name, "read nothing writes", bytes,
                           [&](fb::RigExecWireFile *edited) {
                               edited->steps.back().reads.push_back(
                                   fb::SlotRange(read, past, past + 1));
                           },
                           "step " + last + " reads " + text + " slots [" +
                               std::to_string(past) + ", " +
                               std::to_string(past + 1) +
                               "), which no earlier step writes");
            ++stepGraphProducers;
        }
    }
    if (count > 0 && !clusters.empty()) {
        _ExpectRefusal(name, "cluster past the clusters", bytes,
                       [&](fb::RigExecWireFile *edited) {
                           edited->steps[0].cluster =
                               int32_t(clusters.size());
                       },
                       "step 0 names cluster " + clusterCount +
                           ", which is no cluster");
        _ExpectRefusal(name, "member past the steps", bytes,
                       [&](fb::RigExecWireFile *edited) {
                           edited->clustering->clusters[0].members.push_back(
                               int32_t(count));
                       },
                       "cluster 0 names member " + stepCount +
                           ", which is no step");
        _ExpectRefusal(name, "cluster predecessor past the clusters", bytes,
                       [&](fb::RigExecWireFile *edited) {
                           edited->clustering->clusters[0].preds.push_back(
                               int32_t(clusters.size()));
                       },
                       "cluster 0 names predecessor " + clusterCount +
                           ", which is no cluster");
        _ExpectRefusal(name, "clustering one step short", bytes,
                       [&](fb::RigExecWireFile *edited) {
                           edited->clustering->clusterOf.pop_back();
                       },
                       "the clustering places " + std::to_string(count - 1) +
                           " steps; the file has " + stepCount);
        stepGraphRanges += 4;
    }
    // Every cluster \p from reaches along succs, itself included.
    const auto reached = [&](int32_t from) {
        std::vector<char> seen(clusters.size(), 0);
        std::vector<int32_t> stack = {from};
        seen[size_t(from)] = 1;
        while (!stack.empty()) {
            const int32_t at = stack.back();
            stack.pop_back();
            for (const int32_t next : clusters[size_t(at)].succs) {
                if (!seen[size_t(next)]) {
                    seen[size_t(next)] = 1;
                    stack.push_back(next);
                }
            }
        }
        return seen;
    };
    // The edge a -> b to reverse: a reaches no other predecessor of b, so
    // the only cycle is a and b, and a is the lowest cluster it reaches.
    // The sort then leaves waiting exactly the clusters a reaches; a is the
    // lowest of them, its one waiting predecessor is b, and b's is a, so
    // the walk from the lowest waiting cluster names a.
    for (size_t b = 0; b < clusters.size(); ++b) {
        for (const int32_t a : clusters[b].preds) {
            const std::vector<char> seen = reached(a);
            const bool otherPath = std::any_of(
                clusters[b].preds.begin(), clusters[b].preds.end(),
                [&](int32_t pred) { return pred != a && seen[size_t(pred)]; });
            const bool lowest =
                std::find(seen.begin(), seen.begin() + a, 1) ==
                seen.begin() + a;
            if (otherPath || !lowest) {
                continue;
            }
            _ExpectRefusal(
                name, "two-cluster cycle", bytes,
                [&](fb::RigExecWireFile *edited) {
                    std::vector<int32_t> &preds =
                        edited->clustering->clusters[size_t(a)].preds;
                    preds.insert(std::lower_bound(preds.begin(), preds.end(),
                                                  int32_t(b)),
                                 int32_t(b));
                    std::vector<int32_t> &succs =
                        edited->clustering->clusters[b].succs;
                    succs.insert(
                        std::lower_bound(succs.begin(), succs.end(), a), a);
                },
                "the cluster graph has a cycle through cluster " +
                    std::to_string(a));
            ++stepGraphCycles;
            return;
        }
    }
}

static int staticTableRows = 0;
static int staticChainRows = 0;
static int staticArrayRows = 0;

// Open refuses static tables that do not match the program, each with the
// validator's exact message naming the field: an xform base past the
// xform slots, a delta-base flag past the delta bases, a chain base
// without its have_base flag, and a constraint's array weights of another
// count than its sources.
static void
_TestStaticTableRefusals(const std::string &name,
                         const std::vector<uint8_t> &bytes)
{
    const std::unique_ptr<fb::RigExecWireFile> file = RigExecTestUnpack(bytes);
    if (!file) {
        return;
    }
    const size_t xforms = file->pose->xformBase.size();
    _ExpectRefusal(name, "xform base past the slots", bytes,
                   [](fb::RigExecWireFile *edited) {
                       edited->pose->xformBase.push_back(
                           RigExecWireMatrix4d{});
                   },
                   "pose.xform_base: " + std::to_string(xforms + 1) +
                       " entries, expected " + std::to_string(xforms));
    const size_t deltas = file->geometry->deltaBasePaths.size();
    _ExpectRefusal(name, "delta-base flag past the delta bases", bytes,
                   [](fb::RigExecWireFile *edited) {
                       edited->geometry->deltaBaseOk.push_back(0);
                   },
                   "geometry.delta_base_ok: " + std::to_string(deltas + 1) +
                       " entries, expected " + std::to_string(deltas));
    ++staticTableRows;
    // A chain holding base points, which keeps them without the flag.
    const auto &chains = file->geometry->chains;
    for (size_t c = 0; c < chains.size(); ++c) {
        if (!chains[c].haveBase || chains[c].base == 0) {
            continue;
        }
        _ExpectRefusal(name, "chain base without have_base", bytes,
                       [c](fb::RigExecWireFile *edited) {
                           edited->geometry->chains[c].haveBase = false;
                       },
                       "geometry.chains[" + std::to_string(c) +
                           "]: a base without have_base");
        ++staticChainRows;
        break;
    }
    // A constraint's arrays that resolved, one weight more than sources.
    const auto &arrays = file->pose->constraintArrays;
    for (size_t k = 0; k < arrays.size(); ++k) {
        if (!arrays[k].ok) {
            continue;
        }
        const uint64_t sources = arrays[k].sourceCount;
        _ExpectRefusal(name, "array weights past the sources", bytes,
                       [k](fb::RigExecWireFile *edited) {
                           edited->pose->constraintArrays[k].weights.push_back(
                               1.0);
                       },
                       "pose.constraint_arrays[" + std::to_string(k) +
                           "].weights: " + std::to_string(sources + 1) +
                           " entries, expected " + std::to_string(sources));
        ++staticArrayRows;
        break;
    }
}

static int stepLabelRows = 0;

// Every step's label, as the reader builds it from the tables for error
// text, is the live program's label; a step past the steps is its number.
// True when every label matches.
static bool
_TestStepLabels(const std::string &stage,
                const RigExecRigEvaluator &evaluator,
                const std::vector<uint8_t> &bytes)
{
    std::string error;
    const std::unique_ptr<RigExecRuntimeReader> reader =
        RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &error);
    const RigExecBakedProgram *baked = evaluator.GetBakedProgram();
    CHECK(reader);
    CHECK(baked);
    if (!reader || !baked) {
        if (!reader) {
            std::printf("%s: open: %s\n", stage.c_str(), error.c_str());
        }
        return false;
    }
    const RigExecBakedProgramImpl &program = baked->GetStepGraph();
    const std::unique_ptr<fb::RigExecWireFile> file = RigExecTestUnpack(bytes);
    CHECK(file && file->steps.size() == program.steps.size());
    size_t matched = 0;
    size_t printed = 0;
    for (size_t i = 0; i < program.steps.size(); ++i) {
        const std::string label = reader->GetStepLabelForTesting(i);
        const bool same = label == program.steps[i].label;
        CHECK(same);
        if (same) {
            ++matched;
        } else if (printed++ < 5) {
            std::printf("%s step %zu: label '%s', program '%s'\n",
                        stage.c_str(), i, label.c_str(),
                        program.steps[i].label.c_str());
        }
    }
    const size_t count = program.steps.size();
    CHECK(reader->GetStepLabelForTesting(count) == std::to_string(count));
    std::printf("step labels %s: %zu of %zu match\n", stage.c_str(), matched,
                count);
    return matched == count;
}

// The label RigExecFormatStepLabel gives step \p step of \p bytes edited by
// \p edit, in the validator's "step N (<label>)" form: what a refusal of
// the edited file names.
template <class Edit>
static std::string
_EditedStepName(const std::vector<uint8_t> &bytes, const Edit &edit,
                size_t step)
{
    const std::unique_ptr<fb::RigExecWireFile> file = RigExecTestUnpack(bytes);
    if (!file) {
        return std::string();
    }
    edit(file.get());
    return "step " + std::to_string(step) + " (" +
           RigExecFormatStepLabel(*file, step) + ")";
}

// The index of the first step of \p kind whose object is \p object, or -1.
static int64_t
_FindStep(const fb::RigExecWireFile &file, fb::StepKind kind, int64_t object)
{
    for (size_t s = 0; s < file.steps.size(); ++s) {
        if (file.steps[s].kind == kind && file.steps[s].object == object) {
            return int64_t(s);
        }
    }
    return -1;
}

// Removes from step \p step's reads every range of \p domain covering
// \p slot.
static void
_DropRead(fb::RigExecWireFile *file, size_t step, fb::SlotDomain domain,
          uint32_t slot)
{
    std::vector<fb::SlotRange> &reads = file->steps[step].reads;
    reads.erase(std::remove_if(reads.begin(), reads.end(),
                               [&](const fb::SlotRange &range) {
                                   return range.domain() == domain &&
                                          range.begin() <= slot &&
                                          slot < range.end();
                               }),
                reads.end());
}

static bool
_IsCommitStep(fb::StepKind kind)
{
    return kind == fb::StepKind::SolverCommit ||
           kind == fb::StepKind::Constraint ||
           kind == fb::StepKind::CommitDelta ||
           kind == fb::StepKind::PropagateChunk ||
           kind == fb::StepKind::CommitApply;
}

// _ExpectRefusal, printing the expected refusal, so the log shows each
// phase-table rule's words on a real bake.
template <class Edit>
static void
_ExpectShownRefusal(const std::string &name, const char *what,
                    const std::vector<uint8_t> &bytes, const Edit &edit,
                    const std::string &expected)
{
    _ExpectRefusal(name, what, bytes, edit, expected);
    std::printf("  refused, %s: %s\n", what, expected.c_str());
}

static int recordRefusals = 0;
static int recordListRefusals = 0;

// Frame records edited into a real bake, each refused by Open naming the
// FrameMatrix step or the fold: a record rebound to a version a later
// commit writes, a solver record's position moved to another provider, a
// FrameMatrix step naming a record past the table, a transform list naming
// one, and a fold stripped of a FrameMatrix read.
static void
_TestFrameRecordRefusals(const std::string &name,
                         const std::vector<uint8_t> &bytes)
{
    const std::unique_ptr<fb::RigExecWireFile> file = RigExecTestUnpack(bytes);
    if (!file || file->pose->frameRecords.empty()) {
        return;
    }
    const fb::RigExecWireDomainPose &pose = *file->pose;
    const std::string records = std::to_string(pose.frameRecords.size());
    const auto slotText = [&](uint32_t slot) {
        return RigExecFormatPathText(*file, file->slotMeta->paths[slot]);
    };
    // The last step of each commit, by step index.
    std::vector<int64_t> commitLast(pose.commits.size(), -1);
    for (size_t s = 0; s < file->steps.size(); ++s) {
        const fb::RigExecWireStep &step = file->steps[s];
        if (_IsCommitStep(step.kind) && step.object >= 0 &&
            size_t(step.object) < commitLast.size()) {
            commitLast[size_t(step.object)] = int64_t(s);
        }
    }
    // A record bound to the version of its provider that a commit whose
    // last step runs after the record's step writes.
    bool rebound = false;
    for (size_t r = 0; r < pose.frameRecords.size() && !rebound; ++r) {
        const fb::FrameRecord &record = pose.frameRecords[r];
        const int64_t step =
            _FindStep(*file, fb::StepKind::FrameMatrix, int64_t(r));
        for (size_t w = 0; w < pose.commits.size() && step >= 0; ++w) {
            const fb::RigExecWireCommit &commit = pose.commits[w];
            if (commitLast[w] <= step) {
                continue;
            }
            const auto at = std::find(commit.slots.begin(),
                                      commit.slots.end(),
                                      int32_t(record.slot()));
            if (at == commit.slots.end()) {
                continue;
            }
            const uint32_t version =
                commit.slotWrites[size_t(at - commit.slots.begin())];
            const auto edit = [&](fb::RigExecWireFile *edited) {
                const fb::FrameRecord &was = edited->pose->frameRecords[r];
                edited->pose->frameRecords[r] = fb::FrameRecord(
                    was.slot(), was.commit(), was.target(), was.position(),
                    version, was.moverLabel());
            };
            _ExpectShownRefusal(name, "record rebound to a later commit",
                                bytes, edit,
                                _EditedStepName(bytes, edit, size_t(step)) +
                                    " is bound to PoseFin version " +
                                    std::to_string(version) + " of " +
                                    slotText(record.slot()) + ", which " +
                                    _EditedStepName(bytes, edit,
                                                    size_t(commitLast[w])) +
                                    " writes at or after it");
            ++recordRefusals;
            rebound = true;
            break;
        }
    }
    // A solver record's position moved to another provider of its commit,
    // or past them.
    for (size_t r = 0; r < pose.frameRecords.size(); ++r) {
        const fb::FrameRecord &record = pose.frameRecords[r];
        const fb::RigExecWireCommit &commit = pose.commits[record.commit()];
        const int64_t step =
            _FindStep(*file, fb::StepKind::FrameMatrix, int64_t(r));
        if (!commit.solverOutput || step < 0) {
            continue;
        }
        int32_t position = int32_t(commit.slots.size());
        for (size_t k = 0; k < commit.slots.size(); ++k) {
            if (commit.slots[k] != int32_t(record.slot())) {
                position = int32_t(k);
                break;
            }
        }
        const auto edit = [&](fb::RigExecWireFile *edited) {
            const fb::FrameRecord &was = edited->pose->frameRecords[r];
            edited->pose->frameRecords[r] =
                fb::FrameRecord(was.slot(), was.commit(), was.target(),
                                position, was.version(), was.moverLabel());
        };
        _ExpectShownRefusal(name, "solver record of another provider", bytes,
                            edit,
                            _EditedStepName(bytes, edit, size_t(step)) +
                                " reads position " +
                                std::to_string(position) + " of commit " +
                                std::to_string(record.commit()) +
                                ", which is not the provider it records");
        ++recordRefusals;
        break;
    }
    // A FrameMatrix step naming a record past the table.
    const int64_t first = _FindStep(*file, fb::StepKind::FrameMatrix, 0);
    if (first >= 0) {
        const auto edit = [&](fb::RigExecWireFile *edited) {
            edited->steps[size_t(first)].object =
                int32_t(edited->pose->frameRecords.size());
        };
        _ExpectShownRefusal(name, "FrameMatrix past the records", bytes, edit,
                            _EditedStepName(bytes, edit, size_t(first)) +
                                " names frame record " + records + " of " +
                                records);
        ++recordRefusals;
    }
    // The first revision with a transform list: an entry past the table,
    // and its fold stripped of the first entry's FrameMatrix read.
    const auto &chains = file->geometry->chains;
    size_t id = 0;
    for (size_t c = 0; c < chains.size(); ++c) {
        for (size_t r = 0; r < chains[c].revisions.size(); ++r, ++id) {
            const fb::RigExecWireRevision &revision = chains[c].revisions[r];
            const int64_t fold =
                _FindStep(*file, fb::StepKind::InfluenceFold, int64_t(id));
            if (revision.transformRecords.empty() || fold < 0) {
                continue;
            }
            const auto past = [&](fb::RigExecWireFile *edited) {
                edited->geometry->chains[c].revisions[r].transformRecords[0] =
                    uint32_t(edited->pose->frameRecords.size());
            };
            const std::string folder =
                _EditedStepName(bytes, past, size_t(fold));
            _ExpectShownRefusal(name, "transform record past the records",
                                bytes, past,
                                folder + " reads frame record " + records +
                                    " of " + records);
            const uint32_t k = revision.transformRecords[0];
            const auto stripped = [&](fb::RigExecWireFile *edited) {
                _DropRead(edited, size_t(fold), fb::SlotDomain::FrameMatrix,
                          k);
            };
            _ExpectShownRefusal(name, "fold without a FrameMatrix read", bytes,
                                stripped,
                                folder + " does not declare FrameMatrix[" +
                                    std::to_string(k) + "]");
            recordListRefusals += 2;
            return;
        }
    }
}

static int reservedRefusals = 0;
static int volumeRefusals = 0;
static int versionRefusals = 0;

// The retired values and forms edited into a real bake, each refused by
// Open in the step graph's or the step rules' words: a Snapshots read; a
// step of the SnapshotFinals kind, named as retired even where it stands
// in for a commit's first step or a VolumePlacements step, whose own rules
// would otherwise refuse the file first; a whole-map VolumePlacements step
// and one placing a slot that is no volume; and the previous format
// version, refused with its re-export message.
static void
_TestRetiredRefusals(const std::string &name,
                     const std::vector<uint8_t> &bytes)
{
    const std::unique_ptr<fb::RigExecWireFile> file = RigExecTestUnpack(bytes);
    if (!file || file->steps.empty()) {
        return;
    }
    const size_t last = file->steps.size() - 1;
    const auto snapshots = [](fb::RigExecWireFile *edited) {
        edited->steps.back().reads.push_back(
            fb::SlotRange(fb::SlotDomain::Snapshots, 0, 1));
    };
    _ExpectShownRefusal(name, "Snapshots read", bytes, snapshots,
                        "step " + std::to_string(last) +
                            " declares the retired Snapshots domain");
    // A chain's status step: no other rule reads its kind.
    int64_t status = -1;
    for (size_t s = 0; s < file->steps.size() && status < 0; ++s) {
        if (file->steps[s].kind == fb::StepKind::ChainStatus) {
            status = int64_t(s);
        }
    }
    CHECK(status >= 0);
    if (status >= 0) {
        _ExpectShownRefusal(name, "SnapshotFinals step", bytes,
                            [&](fb::RigExecWireFile *edited) {
                                edited->steps[size_t(status)].kind =
                                    fb::StepKind::SnapshotFinals;
                            },
                            "step " + std::to_string(status) +
                                " is a retired SnapshotFinals step");
    }
    reservedRefusals += status >= 0 ? 2 : 1;
    // The first step of the first commit and the first VolumePlacements
    // step, relabelled.
    for (const bool commit : {true, false}) {
        int64_t at = -1;
        for (size_t s = 0; s < file->steps.size() && at < 0; ++s) {
            const fb::StepKind kind = file->steps[s].kind;
            if (commit ? _IsCommitStep(kind)
                       : kind == fb::StepKind::VolumePlacements) {
                at = int64_t(s);
            }
        }
        if (at < 0) {
            continue;
        }
        _ExpectShownRefusal(name,
                            commit ? "SnapshotFinals for a commit step"
                                   : "SnapshotFinals for a placement step",
                            bytes,
                            [&](fb::RigExecWireFile *edited) {
                                edited->steps[size_t(at)].kind =
                                    fb::StepKind::SnapshotFinals;
                            },
                            "step " + std::to_string(at) +
                                " is a retired SnapshotFinals step");
        ++reservedRefusals;
    }

    // The previous format version.
    const uint32_t previous = RigExecFormatVersion - 1;
    const std::string got =
        _OpenError(RigExecTestEdited(bytes, [&](fb::RigExecWireFile *edited) {
            edited->formatVersion = previous;
        }));
    const std::string want =
        "unsupported .rigexec format version " + std::to_string(previous) +
        " (this reader reads " + std::to_string(RigExecFormatVersion) +
        "); re-export: array inputs";
    CHECK(got == want);
    if (got != want) {
        std::printf("%s, previous version: open said '%s', expected '%s'\n",
                    name.c_str(), got.c_str(), want.c_str());
    }
    ++versionRefusals;

    // The first VolumePlacements step, in the whole-map form and placing a
    // slot that is no volume.
    int64_t place = -1;
    for (size_t s = 0; s < file->steps.size() && place < 0; ++s) {
        if (file->steps[s].kind == fb::StepKind::VolumePlacements) {
            place = int64_t(s);
        }
    }
    const std::vector<uint8_t> &volumes = file->constants->noScaleAvars;
    const auto plain = std::find(volumes.begin(), volumes.end(), 0);
    if (place < 0 || plain == volumes.end()) {
        return;
    }
    const auto wholeMap = [&](fb::RigExecWireFile *edited) {
        edited->steps[size_t(place)].part = -1;
    };
    _ExpectShownRefusal(name, "whole-map VolumePlacements", bytes, wholeMap,
                        _EditedStepName(bytes, wholeMap, size_t(place)) +
                            " is the retired whole-map placement (part -1)");
    const int32_t slot = int32_t(plain - volumes.begin());
    const auto notVolume = [&](fb::RigExecWireFile *edited) {
        edited->steps[size_t(place)].object = slot;
    };
    _ExpectShownRefusal(name, "VolumePlacements of no volume", bytes,
                        notVolume,
                        _EditedStepName(bytes, notVolume, size_t(place)) +
                            " places slot " + std::to_string(slot) +
                            ", which is no volume slot");
    volumeRefusals += 2;
}

static int pointBindingRows = 0;
static int pointBindingRefusals = 0;

// Point bindings edited into a real bake, each refused by Open naming the
// revision's RevisionStatic step: a candidate's chain past the chains, its
// version past the chain's last, one binding more than the phased inputs,
// a binding naming another input, and the reader stripped of the read its
// candidate needs.
static void
_TestPointBindingRefusals(const std::string &name,
                          const std::vector<uint8_t> &bytes)
{
    const std::unique_ptr<fb::RigExecWireFile> file = RigExecTestUnpack(bytes);
    if (!file) {
        return;
    }
    const auto &chains = file->geometry->chains;
    size_t id = 0;
    for (size_t c = 0; c < chains.size(); ++c) {
        for (size_t r = 0; r < chains[c].revisions.size(); ++r, ++id) {
            const fb::RigExecWireRevision &revision = chains[c].revisions[r];
            const int64_t reader =
                _FindStep(*file, fb::StepKind::RevisionStatic, int64_t(id));
            if (revision.pointBindings.empty() ||
                revision.pointBindings[0].candidates.empty() || reader < 0) {
                continue;
            }
            const RigExecWirePointsBinding &binding =
                revision.pointBindings[0];
            const std::string input =
                RigExecFormatPathText(*file, binding.inputPath);
            const std::string who = _EditedStepName(
                bytes, [](fb::RigExecWireFile *) {}, size_t(reader));
            const fb::PointVersion candidate = binding.candidates[0];
            const auto bound = [&](fb::RigExecWireFile *edited)
                -> RigExecWirePointsBinding & {
                return edited->geometry->chains[c]
                    .revisions[r]
                    .pointBindings[0];
            };
            const std::string chainCount = std::to_string(chains.size());
            const auto pastChains = [&](fb::RigExecWireFile *edited) {
                bound(edited).candidates[0] = fb::PointVersion(
                    int32_t(chains.size()), candidate.version());
            };
            _ExpectShownRefusal(name, "candidate chain past the chains", bytes,
                                pastChains,
                                who + " binds " + input + " to chain " +
                                    chainCount + " of " + chainCount);
            const size_t versions =
                chains[size_t(candidate.chain())].revisions.size();
            const auto pastVersions = [&](fb::RigExecWireFile *edited) {
                bound(edited).candidates[0] = fb::PointVersion(
                    candidate.chain(), int32_t(versions + 1));
            };
            _ExpectShownRefusal(name, "candidate version past the revisions",
                                bytes, pastVersions,
                                who + " binds " + input + " to version " +
                                    std::to_string(versions + 1) +
                                    " of chain " +
                                    std::to_string(candidate.chain()) +
                                    ", past its last version " +
                                    std::to_string(versions));
            const size_t inputs = revision.binding->phaseInputs.size();
            const auto oneMore = [&](fb::RigExecWireFile *edited) {
                auto &bindings =
                    edited->geometry->chains[c].revisions[r].pointBindings;
                bindings.push_back(bindings[0]);
            };
            _ExpectShownRefusal(name, "one binding more than the inputs",
                                bytes, oneMore,
                                who + " holds " + std::to_string(inputs + 1) +
                                    " point bindings for " +
                                    std::to_string(inputs) + " phased inputs");
            const auto otherInput = [&](fb::RigExecWireFile *edited) {
                bound(edited).inputPath = edited->rig;
            };
            _ExpectShownRefusal(name, "binding of another input", bytes,
                                otherInput,
                                who + " binds " +
                                    RigExecFormatPathText(*file, file->rig) +
                                    " for phased input 0, which is " + input);
            // The read the first candidate needs: the published points of
            // a final read, else the fuse's done slot (or the base).
            fb::SlotDomain domain = fb::SlotDomain::ChainPoints;
            uint32_t slot = uint32_t(candidate.chain());
            if (!binding.finalRead && candidate.version() == 0) {
                domain = fb::SlotDomain::ChainBase;
            } else if (!binding.finalRead) {
                domain = fb::SlotDomain::RevisionDone;
                slot = uint32_t(file->geometry->chainRevisionBegin[size_t(
                                    candidate.chain())] +
                                candidate.version() - 1);
            }
            const char *domainName =
                domain == fb::SlotDomain::ChainPoints  ? "ChainPoints"
                : domain == fb::SlotDomain::ChainBase ? "ChainBase"
                                                      : "RevisionDone";
            const auto undeclared = [&](fb::RigExecWireFile *edited) {
                _DropRead(edited, size_t(reader), domain, slot);
            };
            _ExpectShownRefusal(name, "reader without its version read", bytes,
                                undeclared,
                                who + " does not declare " + domainName +
                                    "[" + std::to_string(slot) + "] for " +
                                    input);
            pointBindingRefusals += 5;
            ++pointBindingRows;
            return;
        }
    }
}

static int ownChainRuns = 0;
static int ownChainRefusals = 0;

// tests/fixtures/preceding_own_chain.usda: Echo, a lattice whose cage is
// its own target read at `preceding`, binds that read to the one version
// entering it on its own chain. Baked where the lift moves the box, the
// binary equals live baked, and differs from the same rig read at `base`.
// The candidate moved one version later, with the reader declaring the
// slot that version needs, names what the reader's own fuse writes, so
// Open refuses the read as unproduced.
static void
_TestOwnChainPreceding()
{
    const std::string stagePath =
        std::string(RIGEXEC_TEST_FIXTURES_DIR) + "/preceding_own_chain.usda";
    std::string text;
    {
        std::ifstream in(stagePath, std::ios::binary);
        std::ostringstream all;
        all << in.rdbuf();
        text = all.str();
    }
    const std::string preceding = "rigExecReadPhase = \"preceding\"";
    CHECK(text.find(preceding) != std::string::npos);
    std::string baseText = text;
    baseText.replace(baseText.find(preceding), preceding.size(),
                     "rigExecReadPhase = \"base\"");

    const double time = 5.0;
    // Bakes \p usda at time, plays the defaults back and compares them with
    // live baked there; returns the binary's points and its bytes.
    const auto run = [&](const std::string &usda, const char *label,
                         std::vector<RigExecRuntimePoints> *points,
                         std::vector<uint8_t> *bytes) {
        const SdfLayerRefPtr layer = SdfLayer::CreateAnonymous(".usda");
        CHECK(layer->ImportFromString(usda));
        const UsdStageRefPtr stage = UsdStage::Open(layer);
        const SdfPath rigPath = _FindRig(stage);
        CHECK(!rigPath.IsEmpty());
        if (rigPath.IsEmpty()) {
            return false;
        }
        RigExecRigEvaluator evaluator(stage, rigPath);
        evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
        std::string error;
        CHECK(RigExecTestBakeAt(evaluator, time, bytes, &error));
        std::unique_ptr<RigExecRuntimeReader> reader =
            RigExecRuntimeReader::Open(bytes->data(), bytes->size(), &error);
        CHECK(reader);
        if (!reader || !reader->Execute(&error)) {
            std::printf("own-chain preceding, %s: %s\n", label,
                        error.c_str());
            return false;
        }
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(time));
        std::vector<std::string> diffs;
        const bool same = RigExecCompareRuntimeOutputs(pose, *reader, &diffs);
        CHECK(same);
        for (const std::string &line : diffs) {
            std::printf("own-chain preceding, %s: %s\n", label, line.c_str());
        }
        *points = reader->GetPoints();
        return same;
    };
    std::vector<RigExecRuntimePoints> atPreceding, atBase;
    std::vector<uint8_t> bytes, baseBytes;
    if (!run(text, "preceding", &atPreceding, &bytes) ||
        !run(baseText, "base", &atBase, &baseBytes)) {
        return;
    }
    // The entering version moved Echo's posed cage off its rest, so the two
    // reads publish different points.
    bool differs = atPreceding.size() != atBase.size();
    for (size_t i = 0; !differs && i < atPreceding.size(); ++i) {
        differs = atPreceding[i].points.size() != atBase[i].points.size() ||
                  std::memcmp(atPreceding[i].points.data(),
                              atBase[i].points.data(),
                              sizeof(RrVec3f) * atPreceding[i].points.size()) !=
                      0;
    }
    CHECK(differs);
    ++ownChainRuns;

    const std::unique_ptr<fb::RigExecWireFile> file = RigExecTestUnpack(bytes);
    if (!file) {
        return;
    }
    const auto &chains = file->geometry->chains;
    size_t id = 0;
    for (size_t c = 0; c < chains.size(); ++c) {
        for (size_t r = 0; r < chains[c].revisions.size(); ++r, ++id) {
            const auto &bindings = chains[c].revisions[r].pointBindings;
            const int64_t reader =
                _FindStep(*file, fb::StepKind::RevisionStatic, int64_t(id));
            if (bindings.empty() || bindings[0].finalRead ||
                bindings[0].candidates.size() != 1 || reader < 0) {
                continue;
            }
            const fb::PointVersion own = bindings[0].candidates[0];
            if (own.chain() != int32_t(c) || own.version() != int32_t(r)) {
                continue;
            }
            const uint32_t later =
                uint32_t(file->geometry->chainRevisionBegin[c] + r);
            const auto edit = [&](fb::RigExecWireFile *edited) {
                edited->geometry->chains[c].revisions[r].pointBindings[0]
                    .candidates[0] =
                    fb::PointVersion(int32_t(c), int32_t(r + 1));
                for (const fb::SlotDomain domain :
                     {fb::SlotDomain::RevisionDone,
                      fb::SlotDomain::ChainDirty}) {
                    edited->steps[size_t(reader)].reads.push_back(
                        fb::SlotRange(domain, later, later + 1));
                }
            };
            _ExpectShownRefusal(stagePath, "own-chain candidate past the reader",
                                bytes, edit,
                                "step " + std::to_string(reader) +
                                    " reads RevisionDone slots [" +
                                    std::to_string(later) + ", " +
                                    std::to_string(later + 1) +
                                    "), which no earlier step writes");
            ++ownChainRefusals;
        }
    }
}

static int rawLayoutRuns = 0;
static int rawLayoutRefusals = 0;

// Bakes \p stage at \p time into \p bytes and plays a fresh reader twice
// against a fresh evaluator's generations there: the outputs, the
// diagnostics and the work counters. True when both runs match; \p failed
// counts the first run's MoverFailed lines, \p counters holds both runs'
// counters and \p points the first run's points.
static bool
_PlayRawLayouts(const UsdStageRefPtr &stage, const SdfPath &rigPath,
                double time, const char *label, size_t *failed,
                RigExecRuntimeCounters counters[2],
                std::vector<RigExecRuntimePoints> *points,
                std::vector<uint8_t> *bytesOut)
{
    std::vector<uint8_t> &bytes = *bytesOut;
    std::string error;
    {
        RigExecRigEvaluator evaluator(stage, rigPath);
        evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
        CHECK(RigExecTestBakeAt(evaluator, time, &bytes, &error));
    }
    const std::unique_ptr<RigExecRuntimeReader> reader =
        bytes.empty()
            ? nullptr
            : RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &error);
    CHECK(reader);
    if (!reader) {
        std::printf("%s: %s\n", label, error.c_str());
        return false;
    }
    RigExecRigEvaluator evaluator(stage, rigPath);
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    bool all = true;
    for (int run = 0; run < 2; ++run) {
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(time));
        CHECK(pose.valid);
        CHECK(reader->Execute(&error));
        std::vector<std::string> diffs;
        const bool same =
            RigExecCompareRuntime(pose, *reader, true, {}, &diffs);
        CHECK(same);
        size_t lines = 0;
        for (const std::string &line : reader->GetDiagnostics()) {
            lines += line.rfind("MoverFailed ", 0) == 0 ? 1 : 0;
        }
        counters[run] = reader->GetCounters();
        std::printf("%s t=%g, run %d: %s (%zu diagnostics, %zu "
                    "MoverFailed, counters %u/%u/%u)\n",
                    label, time, run + 1, same ? "== Evaluate(T)" : "MISMATCH",
                    reader->GetDiagnostics().size(), lines,
                    counters[run].revisionsExecuted,
                    counters[run].revisionsCreated,
                    counters[run].schedulesBuilt);
        for (const std::string &line : diffs) {
            std::printf("    %s\n", line.c_str());
        }
        if (run == 0) {
            *failed = lines;
            *points = reader->GetPoints();
        }
        all = all && same;
    }
    return all;
}

// tests/fixtures/raw_skin_layouts.usda: skin layouts the evaluators resolve
// but cannot use. The layout cache leaves OddSkin and WideOddSkin
// unvalidated with no point count, their entries not rows of their element
// size, and validates WideEmptySkin at an element size past the sparse
// form's; the bake stores those three raw, their authored arrays bit for
// bit, and ShortSkin and RowSkin sparse. A fresh reader's runs are a fresh
// evaluator's generations at the bake time, twice, failure lines and work
// counters included; so are they on the same rig with a one-point mesh
// skinned by a validated row of element size 65536, which the file holds
// raw and whose stored arrays skin the point. Open refuses a raw layout
// and a sparse one edited out of their forms or off the evaluator's rules,
// and RowSkin's chunk table edited out of the shape the bake cuts.
static void
_TestRawSkinLayouts()
{
    const std::string stagePath =
        std::string(RIGEXEC_TEST_FIXTURES_DIR) + "/raw_skin_layouts.usda";
    const UsdStageRefPtr stage = UsdStage::Open(stagePath);
    CHECK(stage);
    if (!stage) {
        return;
    }
    const SdfPath rigPath = _FindRig(stage);
    CHECK(!rigPath.IsEmpty());
    if (rigPath.IsEmpty()) {
        return;
    }
    struct Skin {
        const char *path;
        int32_t elementSize;
        uint64_t points;
        bool validated;
        bool raw;
    };
    const Skin skins[] = {
        {"OddSmooth/OddSkin", 2, 0, false, true},
        {"WideOddSkin", 70000, 0, false, true},
        {"WideEmptySkin", 70000, 0, true, true},
        {"ShortSkin", 2, 3, true, false},
        {"RowSkin", 2, 10, true, false},
    };
    const size_t skinCount = sizeof(skins) / sizeof(skins[0]);
    const std::string movers = rigPath.GetString() + "/Movers/";

    // The evaluators' own answer, from the layout cache every evaluator
    // resolves through.
    for (const Skin &skin : skins) {
        const UsdPrim prim =
            stage->GetPrimAtPath(SdfPath(movers + skin.path));
        CHECK(prim);
        RigExecSkinTopologyCache cache;
        const std::shared_ptr<const RigExecSkinTopology> topology =
            RigExecResolveSkinTopology(prim, 2, UsdTimeCode(1.0), nullptr,
                                       &cache);
        CHECK(topology);
        if (!topology) {
            continue;
        }
        CHECK(topology->elementSize == skin.elementSize &&
              topology->pointCount == skin.points &&
              topology->influenceCount == 2 &&
              topology->validated == skin.validated);
        if (skin.raw && !skin.validated) {
            CHECK(topology->indices.size() == 5 &&
                  topology->weights.size() == 5);
        }
    }

    std::vector<uint8_t> bytes;
    {
        RigExecRigEvaluator evaluator(stage, rigPath);
        evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
        std::string error;
        CHECK(RigExecTestBakeAt(evaluator, 1.0, &bytes, &error));
        if (bytes.empty()) {
            std::printf("raw skin layouts: bake: %s\n", error.c_str());
            return;
        }
    }
    const std::unique_ptr<fb::RigExecWireFile> file = RigExecTestUnpack(bytes);
    if (!file) {
        return;
    }
    std::vector<std::pair<size_t, size_t>> at(skinCount,
                                              {SIZE_MAX, SIZE_MAX});
    const auto &chains = file->geometry->chains;
    for (size_t c = 0; c < chains.size(); ++c) {
        for (size_t r = 0; r < chains[c].revisions.size(); ++r) {
            const std::string mover =
                RigExecFormatPathText(*file, chains[c].revisions[r].moverPath);
            for (size_t k = 0; k < skinCount; ++k) {
                if (mover == movers + skins[k].path) {
                    at[k] = {c, r};
                }
            }
        }
    }
    for (size_t k = 0; k < skinCount; ++k) {
        CHECK(at[k].first != SIZE_MAX);
        if (at[k].first == SIZE_MAX) {
            return;
        }
        const fb::RigExecWireRevision &revision =
            chains[at[k].first].revisions[at[k].second];
        CHECK(revision.topology && !revision.chunked &&
              revision.chunks.size() == 1);
        if (!revision.topology) {
            return;
        }
        const fb::RigExecWireSkinTopology &t = *revision.topology;
        CHECK(t.raw == skins[k].raw && t.validated == skins[k].validated &&
              t.elementSize == skins[k].elementSize &&
              t.pointCount == skins[k].points && t.influenceCount == 2);
        if (!skins[k].raw) {
            CHECK(t.rawIndices.empty() && t.rawWeights.empty());
            continue;
        }
        // The stored arrays are the authored ones, bit for bit.
        const UsdPrim prim =
            stage->GetPrimAtPath(SdfPath(movers + skins[k].path));
        VtIntArray indices;
        VtFloatArray weights;
        CHECK(prim.GetAttribute(TfToken("rigExec:jointIndices"))
                  .Get(&indices));
        CHECK(prim.GetAttribute(TfToken("rigExec:jointWeights"))
                  .Get(&weights));
        CHECK(t.indexWidth == 0 && t.counts8.empty() && t.counts16.empty() &&
              t.indices8.empty() && t.indices16.empty() &&
              t.indices32.empty() && t.weights.empty());
        CHECK(t.rawIndices.size() == indices.size() &&
              std::equal(indices.begin(), indices.end(),
                         t.rawIndices.begin()));
        CHECK(t.rawWeights.size() == weights.size() &&
              (weights.empty() ||
               std::memcmp(weights.cdata(), t.rawWeights.data(),
                           sizeof(float) * weights.size()) == 0));
    }
    // OddSkin's last weight is authored -0, which the raw form keeps bit
    // for bit.
    {
        const fb::RigExecWireSkinTopology &odd =
            *chains[at[0].first].revisions[at[0].second].topology;
        uint32_t bits = 0;
        CHECK(odd.rawWeights.size() == 5);
        if (odd.rawWeights.size() == 5) {
            std::memcpy(&bits, &odd.rawWeights[4], sizeof bits);
        }
        CHECK(bits == 0x80000000u);
    }

    // Playback against live baked: the outputs, the four skins' failure
    // lines and the mover graph's line, and the work counters; the first
    // generation builds the graph, the second redoes nothing.
    {
        size_t failed = 0;
        RigExecRuntimeCounters counters[2];
        std::vector<RigExecRuntimePoints> points;
        std::vector<uint8_t> played;
        rawLayoutRuns += _PlayRawLayouts(stage, rigPath, 1.0,
                                         "raw skin layouts", &failed,
                                         counters, &points, &played)
                             ? 2
                             : 0;
        // A bake is deterministic: the played file is the one read above.
        CHECK(played == bytes);
        CHECK(failed == 4);
        CHECK(counters[0].revisionsExecuted == 11 &&
              counters[0].revisionsCreated == 11 &&
              counters[0].schedulesBuilt == 10);
        CHECK(counters[1].revisionsExecuted == 0 &&
              counters[1].revisionsCreated == 0 &&
              counters[1].schedulesBuilt == 0);
    }

    // A one-point mesh skinned by a validated row of element size 65536,
    // which only the raw form holds, baked at time 7, where the joints have
    // left their rests: the stored arrays skin the point, beside the
    // fixture's four failures.
    {
        std::string text;
        {
            std::ifstream in(stagePath, std::ios::binary);
            std::ostringstream all;
            all << in.rdbuf();
            text = all.str();
        }
        const SdfLayerRefPtr layer = SdfLayer::CreateAnonymous(".usda");
        CHECK(layer->ImportFromString(text));
        const UsdStageRefPtr wide = UsdStage::Open(layer);
        const GfVec3f authored(3.0f, 0.5f, 10.0f);
        const SdfPath meshPath("/RawSkinAsset/Geom/WideRowMesh");
        const UsdPrim mesh = wide->DefinePrim(meshPath, TfToken("Mesh"));
        mesh.GetAttribute(TfToken("points")).Set(VtVec3fArray{authored});
        const std::string skinPath = movers + "WideRowSkin";
        const UsdPrim skin = wide->DefinePrim(SdfPath(skinPath),
                                              TfToken("RigExecSkinMover"));
        skin.ApplyAPI(TfToken("RigExecMoverAPI"));
        skin.GetRelationship(TfToken("rigExec:moves"))
            .SetTargets({meshPath.AppendProperty(TfToken("points"))});
        skin.CreateRelationship(TfToken("rigExec:influences"))
            .SetTargets({rigPath.AppendPath(SdfPath("Joints/J0")),
                         rigPath.AppendPath(SdfPath("Joints/J0/J1"))});
        skin.CreateAttribute(TfToken("rigExec:elementSize"),
                             SdfValueTypeNames->Int)
            .Set(65536);
        skin.CreateAttribute(TfToken("rigExec:skinningMethod"),
                             SdfValueTypeNames->Token)
            .Set(TfToken("classicLinear"));
        // J1 at 0.75 and J0 at 0.25, then padding.
        VtIntArray indices(65536, 0);
        VtFloatArray weights(65536, 0.0f);
        indices[0] = 1;
        weights[0] = 0.75f;
        weights[1] = 0.25f;
        skin.CreateAttribute(TfToken("rigExec:jointIndices"),
                             SdfValueTypeNames->IntArray)
            .Set(indices);
        skin.CreateAttribute(TfToken("rigExec:jointWeights"),
                             SdfValueTypeNames->FloatArray)
            .Set(weights);

        size_t failed = 0;
        RigExecRuntimeCounters counters[2];
        std::vector<RigExecRuntimePoints> points;
        std::vector<uint8_t> wideBytes;
        rawLayoutRuns += _PlayRawLayouts(wide, rigPath, 7.0,
                                         "raw skin layouts, a wide row",
                                         &failed, counters, &points,
                                         &wideBytes)
                             ? 2
                             : 0;
        CHECK(failed == 4);
        // The file holds the row raw and validated, its arrays bit for bit.
        bool stored = false;
        if (const std::unique_ptr<fb::RigExecWireFile> wideFile =
                RigExecTestUnpack(wideBytes)) {
            for (const fb::RigExecWireChain &chain :
                 wideFile->geometry->chains) {
                for (const fb::RigExecWireRevision &revision :
                     chain.revisions) {
                    if (RigExecFormatPathText(*wideFile,
                                              revision.moverPath) !=
                            skinPath ||
                        !revision.topology) {
                        continue;
                    }
                    const fb::RigExecWireSkinTopology &t = *revision.topology;
                    stored = t.raw && t.validated && t.pointCount == 1 &&
                             t.elementSize == 65536 &&
                             t.rawIndices.size() == indices.size() &&
                             std::equal(indices.begin(), indices.end(),
                                        t.rawIndices.begin()) &&
                             t.rawWeights.size() == weights.size() &&
                             std::memcmp(weights.cdata(),
                                         t.rawWeights.data(),
                                         sizeof(float) * weights.size()) ==
                                 0;
                }
            }
        }
        CHECK(stored);
        // And playback skinned the point off the authored one, as live
        // baked did.
        bool skinned = false;
        for (const RigExecRuntimePoints &moved : points) {
            if (moved.path == meshPath.GetString() + ".points" &&
                moved.points.size() == 1) {
                const RrVec3f &p = moved.points[0];
                skinned = p[0] != authored[0] || p[1] != authored[1] ||
                          p[2] != authored[2];
            }
        }
        CHECK(skinned);
    }

    // Edits Open refuses: OddSkin's raw layout out of its form or off the
    // rules, WideEmptySkin's raw row and RowSkin's sparse layout off the
    // evaluator's rules, the out-of-range read playback would make among
    // them, and RowSkin's chunk table out of the shape the bake cuts.
    const auto rowOf = [&](size_t k) {
        return "geometry.chains[" + std::to_string(at[k].first) +
               "].revisions[" + std::to_string(at[k].second) + "]";
    };
    const auto revisionOf = [&](fb::RigExecWireFile *edited,
                                size_t k) -> fb::RigExecWireRevision & {
        return edited->geometry->chains[at[k].first].revisions[at[k].second];
    };
    const size_t odd = 0, wideEmpty = 2, row = 4;
    const std::string oddTopology = rowOf(odd) + ".topology";
    const auto refuse = [&](const char *what, size_t k,
                            const std::function<void(
                                fb::RigExecWireRevision &)> &edit,
                            const std::string &expected) {
        _ExpectShownRefusal(stagePath, what, bytes,
                            [&](fb::RigExecWireFile *edited) {
                                edit(revisionOf(edited, k));
                            },
                            expected);
        ++rawLayoutRefusals;
    };
    using Rev = fb::RigExecWireRevision;
    refuse("raw layout marked validated", odd,
           [](Rev &r) { r.topology->validated = true; },
           oddTopology +
               ": validated, but its raw layout is not rows of element_size");
    refuse("raw layout the sparse form holds", odd,
           [](Rev &r) {
               r.topology->rawIndices.pop_back();
               r.topology->rawWeights.pop_back();
           },
           oddTopology + ": a raw layout of 4 indices and 4 weights at "
                         "element_size 2, which the sparse form holds");
    refuse("unvalidated raw row the rules pass", wideEmpty,
           [](Rev &r) { r.topology->validated = false; },
           rowOf(wideEmpty) + ".topology: not validated, but its layout "
                              "passes the evaluator's rules");
    refuse("raw layout point count", odd,
           [](Rev &r) { r.topology->pointCount = 2; },
           oddTopology + ": point_count 2, but its raw layout holds 0 "
                         "point(s)");
    refuse("raw arrays on a sparse layout", odd,
           [](Rev &r) { r.topology->raw = false; },
           oddTopology + ": raw_indices or raw_weights on a sparse layout");
    refuse("raw layout with an index width", odd,
           [](Rev &r) { r.topology->indexWidth = 1; },
           oddTopology + ": a raw layout with sparse vectors or an "
                         "index_width");
    const std::string rowTopology = rowOf(row) + ".topology";
    const Rev &stored = chains[at[row].first].revisions[at[row].second];
    CHECK(stored.topology->indexWidth == 1 &&
          !stored.topology->indices8.empty() &&
          stored.topology->indices8[0] == 0);
    refuse("index at the influence count", row,
           [](Rev &r) { r.topology->indices8[0] = 2; },
           rowTopology + ": validated, but kept entry 0 indexes influence 2 "
                         "of 2");
    refuse("negative index", row,
           [](Rev &r) {
               fb::RigExecWireSkinTopology &t = *r.topology;
               t.indexWidth = 4;
               t.indices32.assign(t.indices8.begin(), t.indices8.end());
               t.indices8.clear();
               t.indices32[0] = -1;
           },
           rowTopology + ": validated, but kept entry 0 indexes influence -1 "
                         "of 2");
    refuse("unvalidated layout the rules pass", row,
           [](Rev &r) { r.topology->validated = false; },
           rowTopology + ": not validated, but its layout passes the "
                         "evaluator's rules");
    refuse("NaN weight", row,
           [](Rev &r) {
               r.topology->weights[0] =
                   std::numeric_limits<float>::quiet_NaN();
           },
           rowTopology + ": validated, but kept entry 0 has a negative or "
                         "non-finite weight");
    refuse("chunked with one chunk", row, [](Rev &r) { r.chunked = true; },
           rowOf(row) + ": chunked with 1 chunk(s)");
    refuse("key on an unchunked revision", row,
           [](Rev &r) { r.chunks[0].key = {0}; },
           rowOf(row) + ".chunks[0]: a key on an unchunked revision");
    refuse("partition point count", row,
           [](Rev &r) { r.partitionPointCount = 11; },
           rowOf(row) + ": partition_point_count 11, but "
                        "partition_index_count " +
               std::to_string(stored.partitionIndexCount) + " is " +
               std::to_string(stored.partitionPointCount) +
               " row(s) of partition_element_size " +
               std::to_string(stored.partitionElementSize));
    CHECK(stored.partitionPointCount == 10 &&
          stored.partitionIndexCount == 20 &&
          stored.partitionElementSize == 2);
}

static int phaseFixtureRows = 0;

// The phase fixtures under tests/fixtures, baked at time 1: every step
// label, the FrameMatrix and per-volume VolumePlacements ones among them,
// against the live program's, and the refusals above on each bake.
static void
_TestPhaseFixtures()
{
    for (const char *fixture :
         {"frame_record_fallbacks.usda", "solver_checkpoint.usda",
          "volume_placements.usda"}) {
        const std::string stagePath =
            std::string(RIGEXEC_TEST_FIXTURES_DIR) + "/" + fixture;
        const UsdStageRefPtr stage = UsdStage::Open(stagePath);
        CHECK(stage);
        if (!stage) {
            continue;
        }
        const SdfPath rigPath = _FindRig(stage);
        CHECK(!rigPath.IsEmpty());
        if (rigPath.IsEmpty()) {
            continue;
        }
        RigExecRigEvaluator evaluator(stage, rigPath);
        evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
        std::vector<uint8_t> bytes;
        std::string error;
        CHECK(RigExecTestBakeAt(evaluator, 1.0, &bytes, &error));
        if (bytes.empty()) {
            std::printf("%s: bake: %s\n", fixture, error.c_str());
            continue;
        }
        if (_TestStepLabels(fixture, evaluator, bytes)) {
            ++phaseFixtureRows;
        }
        _TestFrameRecordRefusals(stagePath, bytes);
        _TestRetiredRefusals(stagePath, bytes);
    }
}

// A fresh reader executes its defaults: Execute has no precondition, and
// the run publishes its generation's lines.
static void
_TestFreshExecute(const std::vector<uint8_t> &bytes)
{
    std::string error;
    std::unique_ptr<RigExecRuntimeReader> reader =
        RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &error);
    CHECK(reader);
    if (!reader) {
        std::printf("open diagnostic: %s\n", error.c_str());
        return;
    }
    CHECK(reader->Execute(&error));
    if (!error.empty()) {
        std::printf("fresh execute: %s\n", error.c_str());
    }
    CHECK(!reader->GetDiagnostics().empty());
}

template <class T>
static bool
_SameBits(const T &a, const T &b)
{
    return std::memcmp(&a, &b, sizeof(T)) == 0;
}

template <class T>
static bool
_SameBits(const std::vector<T> &a, const std::vector<T> &b)
{
    return a.size() == b.size() &&
           (a.empty() ||
            std::memcmp(a.data(), b.data(), a.size() * sizeof(T)) == 0);
}

// Bitwise equality on the member the tag names.
static bool
_SameInput(const RrInputValue &a, const RrInputValue &b)
{
    if (a.tag != b.tag) {
        return false;
    }
    switch (a.tag) {
    case RrInputTag::Double:
        return _SameBits(a.f64, b.f64);
    case RrInputTag::Float:
        return _SameBits(a.f32, b.f32);
    case RrInputTag::Bool:
        return a.boolean == b.boolean;
    case RrInputTag::Int:
        return a.i32 == b.i32;
    case RrInputTag::Matrix4d:
        return _SameBits(a.matrix, b.matrix);
    case RrInputTag::Token:
        return a.token == b.token;
    case RrInputTag::Vec3d:
        return _SameBits(a.vec, b.vec);
    case RrInputTag::Vec3f:
        return _SameBits(a.vec3f, b.vec3f);
    case RrInputTag::IntArray:
    case RrInputTag::FloatArray:
    case RrInputTag::DoubleArray:
    case RrInputTag::Vec2fArray:
    case RrInputTag::Vec3fArray:
        // An array input's value carries its tag alone.
        return true;
    }
    return false;
}

static bool
_SameProperty(const RrPropertyValue &a, const RrPropertyValue &b)
{
    if (a.tag != b.tag) {
        return false;
    }
    switch (a.tag) {
    case RrPropertyValue::Tag::Float:
        return _SameBits(a.f32, b.f32);
    case RrPropertyValue::Tag::Double:
        return _SameBits(a.f64, b.f64);
    case RrPropertyValue::Tag::Matrix4d:
        return _SameBits(a.matrix, b.matrix);
    case RrPropertyValue::Tag::Vec3f:
        return _SameBits(a.vec, b.vec);
    }
    return false;
}

// The first output domain two readers' last runs differ in, bit for bit,
// or "": joints, points, matrix primvars, weight frames and fields,
// provider transforms and property values, and with \p everything the
// diagnostics and counters too.
static std::string
_CompareRuns(const RigExecRuntimeReader &a, const RigExecRuntimeReader &b,
             bool everything)
{
    {
        const auto &x = a.GetJointMatrices();
        const auto &y = b.GetJointMatrices();
        if (x.size() != y.size()) {
            return "joint matrix count";
        }
        for (size_t i = 0; i < x.size(); ++i) {
            if (x[i].path != y[i].path || !_SameBits(x[i].matrix, y[i].matrix)) {
                return "joint matrix " + x[i].path;
            }
        }
    }
    {
        const auto &x = a.GetPoints();
        const auto &y = b.GetPoints();
        if (x.size() != y.size()) {
            return "points count";
        }
        for (size_t i = 0; i < x.size(); ++i) {
            if (x[i].path != y[i].path || !_SameBits(x[i].points, y[i].points)) {
                return "points " + x[i].path;
            }
        }
    }
    {
        const auto &x = a.GetMatrixPrimvars();
        const auto &y = b.GetMatrixPrimvars();
        if (x.size() != y.size()) {
            return "matrix primvar count";
        }
        for (size_t i = 0; i < x.size(); ++i) {
            if (x[i].path != y[i].path || !_SameBits(x[i].matrix, y[i].matrix)) {
                return "matrix primvar " + x[i].path;
            }
        }
    }
    {
        const auto &x = a.GetWeightFrames();
        const auto &y = b.GetWeightFrames();
        if (x.size() != y.size()) {
            return "weight frame count";
        }
        for (size_t i = 0; i < x.size(); ++i) {
            if (x[i].path != y[i].path || !_SameBits(x[i].matrix, y[i].matrix)) {
                return "weight frame " + x[i].path;
            }
        }
    }
    {
        const auto &x = a.GetWeightFields();
        const auto &y = b.GetWeightFields();
        if (x.size() != y.size()) {
            return "weight field count";
        }
        for (size_t i = 0; i < x.size(); ++i) {
            if (x[i].path != y[i].path || x[i].target != y[i].target ||
                !_SameBits(x[i].weights, y[i].weights)) {
                return "weight field " + x[i].path;
            }
        }
    }
    {
        const auto &x = a.GetProviderXforms();
        const auto &y = b.GetProviderXforms();
        if (x.size() != y.size()) {
            return "provider transform count";
        }
        for (size_t i = 0; i < x.size(); ++i) {
            if (x[i].path != y[i].path || !_SameBits(x[i].matrix, y[i].matrix) ||
                !_SameBits(x[i].base, y[i].base)) {
                return "provider transform " + x[i].path;
            }
        }
    }
    {
        const auto x = a.GetPropertyValues();
        const auto y = b.GetPropertyValues();
        if (x.size() != y.size()) {
            return "property value count";
        }
        for (size_t i = 0; i < x.size(); ++i) {
            if (x[i].path != y[i].path || !_SameProperty(x[i].value, y[i].value)) {
                return "property value " + x[i].path;
            }
        }
    }
    if (!everything) {
        return std::string();
    }
    if (a.GetDiagnostics() != b.GetDiagnostics()) {
        return "diagnostics";
    }
    const RigExecRuntimeCounters p = a.GetCounters();
    const RigExecRuntimeCounters q = b.GetCounters();
    if (p.revisionsExecuted != q.revisionsExecuted ||
        p.revisionsCreated != q.revisionsCreated ||
        p.schedulesBuilt != q.schedulesBuilt ||
        p.chainsBuilt != q.chainsBuilt ||
        p.revisionsBuilt != q.revisionsBuilt) {
        return "counters";
    }
    return std::string();
}

static int inputApiRows = 0;
static int inputApiFields = 0;
static int inputApiMoved = 0;
static int inputApiResets = 0;
static int inputApiTokens = 0;

static int defaultsRows = 0;

// A fresh reader executes the defaults, and they are the bake: a fresh
// evaluator's generation at the bake time (no explicit Compile, so its
// compile notices land in that generation, where the reader replays them)
// matches the reader's first run in every output, the diagnostics and the
// work counters. A second run with nothing set matches the evaluator's
// second generation at the same time, counters included.
static void
_TestDefaults(const RigExecExampleFixture &fixture,
              const UsdStageRefPtr &stage, const SdfPath &rigPath,
              const std::vector<uint8_t> &bytes)
{
    std::string error;
    const std::unique_ptr<RigExecRuntimeReader> reader =
        RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &error);
    CHECK(reader);
    if (!reader) {
        std::printf("%s: open: %s\n", fixture.stage, error.c_str());
        return;
    }
    const double t = reader->GetBakeTime();
    CHECK(t == RigExecBakedProbeTime(stage).GetValue());
    RigExecRigEvaluator evaluator(stage, rigPath);
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(t));
    CHECK(pose.valid);
    CHECK(evaluator.GetBakedGenerationCount() == 1);
    CHECK(reader->Execute(&error));
    if (!error.empty()) {
        std::printf("%s: defaults execute: %s\n", fixture.stage,
                    error.c_str());
    }
    std::vector<std::string> diffs;
    bool same = RigExecCompareRuntime(pose, *reader, true, {}, &diffs);
    CHECK(same);
    {
        const RigExecRigPose again = evaluator.Evaluate(UsdTimeCode(t));
        CHECK(again.valid);
        CHECK(reader->Execute(&error));
        const bool sameAgain =
            RigExecCompareRuntime(again, *reader, true, {}, &diffs);
        CHECK(sameAgain);
        if (!sameAgain) {
            std::printf("defaults %s t=%g: the second run differs\n",
                        fixture.stage, t);
        }
        same = same && sameAgain;
    }
    if (same) {
        std::printf("defaults %s t=%g: == Evaluate(T), twice (%zu "
                    "joints, %zu moved, %zu weight frames, %zu fields, %zu "
                    "provider xforms, %zu diagnostics, counters "
                    "%zu/%zu/%zu)\n",
                    fixture.stage, t, reader->GetJointMatrices().size(),
                    reader->GetPoints().size(),
                    reader->GetWeightFrames().size(),
                    reader->GetWeightFields().size(),
                    reader->GetProviderXforms().size(),
                    reader->GetDiagnostics().size(),
                    size_t(reader->GetCounters().revisionsExecuted),
                    size_t(reader->GetCounters().revisionsCreated),
                    size_t(reader->GetCounters().schedulesBuilt));
        ++defaultsRows;
    } else {
        std::printf("defaults %s t=%g: MISMATCH (%zu differences)\n",
                    fixture.stage, t, diffs.size());
        for (const std::string &line : diffs) {
            std::printf("    %s\n", line.c_str());
        }
    }
}

static int samplerRows = 0;
static int samplerFrames = 0;
static int samplerUnanimatedRows = 0;

// Playback along the stage's timeline: the sampler hands a fresh reader the
// stage's values of its Animated inputs at each time, and each run is a
// fresh evaluator's generation at that time in every output, the
// diagnostics (summary line included) and the work counters. A row whose
// file has no Animated input still moves the counters with time: the
// sampler's TouchAnimatedInputs is what dirties the steps a change of time
// dirties in the program.
static void
_TestSamplerDrive(const RigExecExampleFixture &fixture,
                  const UsdStageRefPtr &stage, const SdfPath &rigPath,
                  const std::vector<uint8_t> &bytes,
                  const std::vector<double> &frames)
{
    std::string error;
    const std::unique_ptr<RigExecRuntimeReader> reader =
        RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &error);
    CHECK(reader);
    if (!reader) {
        std::printf("%s: open: %s\n", fixture.stage, error.c_str());
        return;
    }
    CHECK(reader->GetBakeTime() == frames.front());
    RigExecInputSampler sampler;
    CHECK(sampler.Bind(stage, *reader, &error));
    CHECK(sampler.GetWarnings().empty());
    for (const std::string &warning : sampler.GetWarnings()) {
        std::printf("%s: sampler: %s\n", fixture.stage, warning.c_str());
    }
    const bool isStatic = std::string(fixture.animation) == "static";
    const std::vector<double> times =
        isStatic ? std::vector<double>{frames.front()} : frames;
    RigExecRigEvaluator evaluator(stage, rigPath);
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    size_t matched = 0;
    for (const double t : times) {
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(t));
        CHECK(pose.valid);
        const bool ran = RigExecTestDrive(reader.get(), &sampler, t, &error);
        CHECK(ran);
        if (!ran) {
            std::printf("%s t=%g: drive: %s\n", fixture.stage, t,
                        error.c_str());
            continue;
        }
        std::vector<std::string> diffs;
        const bool same =
            RigExecCompareRuntime(pose, *reader, true, {}, &diffs);
        CHECK(same);
        if (same) {
            ++matched;
        } else {
            std::printf("sampler %s t=%g: MISMATCH (%zu differences)\n",
                        fixture.stage, t, diffs.size());
            for (const std::string &line : diffs) {
                std::printf("    %s\n", line.c_str());
            }
        }
    }
    CHECK(evaluator.GetBakedGenerationCount() == times.size());
    std::printf("sampler %s: %zu of %zu frame(s) driven == Evaluate(t) "
                "(%s; %zu input(s), %zu sampled per frame)\n",
                fixture.stage, matched, times.size(), fixture.animation,
                reader->GetInputCount(), sampler.GetAnimatedCount());
    if (matched == times.size()) {
        ++samplerRows;
        if (!isStatic && times.size() > 1 &&
            sampler.GetAnimatedCount() == 0) {
            ++samplerUnanimatedRows;
        }
    }
    samplerFrames += int(matched);
}

// The input API over a bake at the probe time, whose slot values are the
// inputs' defaults.
static void
_TestInputApi(const RigExecExampleFixture &fixture,
              const UsdStageRefPtr &stage, const std::vector<uint8_t> &bytes)
{
    const double probe = RigExecBakedProbeTime(stage).GetValue();
    std::string error;
    const auto open = [&] {
        std::string why;
        std::unique_ptr<RigExecRuntimeReader> reader =
            RigExecRuntimeReader::Open(bytes.data(), bytes.size(), &why);
        CHECK(reader);
        if (!reader) {
            std::printf("%s: open: %s\n", fixture.stage, why.c_str());
        }
        return reader;
    };
    const std::unique_ptr<RigExecRuntimeReader> fresh = open();
    if (!fresh) {
        return;
    }
    CHECK(fresh->GetBakeTime() == probe);

    // Names unique and sorted, each found at its own index, and a fresh
    // reader holding every default.
    const size_t count = fresh->GetInputCount();
    CHECK(count > 0);
    size_t animated = 0;
    for (size_t i = 0; i < count; ++i) {
        const RigExecRuntimeInputInfo &info = fresh->GetInputInfo(i);
        CHECK(!info.name.empty());
        if (i > 0) {
            CHECK(fresh->GetInputInfo(i - 1).name < info.name);
        }
        size_t at = count;
        CHECK(fresh->FindInput(info.name, &at) && at == i);
        CHECK(_SameInput(fresh->GetInputValue(i), info.defaultValue));
        CHECK(fresh->GetInputValue(i).tag == info.type);
        animated += info.animated ? 1 : 0;
    }
    CHECK(!fresh->FindInput("/RigExecTest/NoSuch.attr", nullptr));
    CHECK(fresh->GetInputInfo(count).name.empty());

    // Refusals change nothing: an unknown name, a value of another type,
    // non-finite values, an index past the count.
    std::string why;
    CHECK(!fresh->SetInput("/RigExecTest/NoSuch.attr", 1.0, &why));
    CHECK(why == "no input named /RigExecTest/NoSuch.attr");
    {
        const RigExecRuntimeInputInfo &info = fresh->GetInputInfo(0);
        RrInputValue wrong = info.defaultValue;
        wrong.tag = info.type == RrInputTag::Bool ? RrInputTag::Int
                                                  : RrInputTag::Bool;
        why.clear();
        CHECK(!fresh->SetInputAt(0, wrong, &why));
        CHECK(!why.empty());
        CHECK(_SameInput(fresh->GetInputValue(0), info.defaultValue));
    }
    for (size_t i = 0; i < count; ++i) {
        const RigExecRuntimeInputInfo &info = fresh->GetInputInfo(i);
        if (info.type != RrInputTag::Double &&
            info.type != RrInputTag::Float) {
            continue;
        }
        // A Float input also refuses a finite double that narrows to inf.
        std::vector<double> bad = {std::numeric_limits<double>::quiet_NaN(),
                                   -std::numeric_limits<double>::infinity()};
        if (info.type == RrInputTag::Float) {
            bad.push_back(1e300);
        }
        for (const double value : bad) {
            why.clear();
            CHECK(!fresh->SetInput(info.name, value, &why));
            CHECK(why == info.name + " takes finite values only");
        }
        CHECK(_SameInput(fresh->GetInputValue(i), info.defaultValue));
        break;
    }
    why.clear();
    CHECK(!fresh->SetInputAt(count, fresh->GetInputInfo(0).defaultValue,
                             &why));
    CHECK(why == "no input at index " + std::to_string(count) +
                     "; the file lists " + std::to_string(count));

    // The table's control avar and operator input are inputs of the file:
    // the fields every drag of the example lands on. The first is set to
    // its default + 0.25 and the run moves the outputs or not (it moves
    // them on most rows); reset, the run plays the defaults again.
    CHECK(fresh->Execute(&error));
    std::vector<std::string> fields;
    if (fixture.controlPrim[0] != '\0') {
        fields.push_back(std::string(fixture.controlPrim) + "." +
                         fixture.controlAvar);
    }
    if (fixture.operatorPrim[0] != '\0') {
        fields.push_back(std::string(fixture.operatorPrim) + "." +
                         fixture.operatorInput);
    }
    for (const std::string &field : fields) {
        size_t at = count;
        const bool found = fresh->FindInput(field, &at);
        CHECK(found);
        if (found) {
            ++inputApiFields;
        } else {
            std::printf("%s: %s is no input\n", fixture.stage,
                        field.c_str());
        }
    }
    const std::string dragName = fields.empty() ? std::string() : fields[0];
    size_t drag = count;
    if (!dragName.empty()) {
        fresh->FindInput(dragName, &drag);
    }
    std::string moveNote = "n/a";
    std::string resetNote = "n/a";
    if (drag < count) {
        const RigExecRuntimeInputInfo &info = fresh->GetInputInfo(drag);
        const double base = info.type == RrInputTag::Float
                                ? double(info.defaultValue.f32)
                                : info.defaultValue.f64;
        const double value = base + 0.25;
        const std::unique_ptr<RigExecRuntimeReader> input = open();
        if (!input) {
            return;
        }
        CHECK(input->SetInput(dragName, value, &why));
        CHECK(input->Execute(&error));
        const std::string moved = _CompareRuns(*input, *fresh, false);
        if (!moved.empty()) {
            ++inputApiMoved;
            moveNote = "moves " + moved;
        } else {
            moveNote = "outputs unmoved";
        }
        input->ResetInputs();
        CHECK(_SameInput(input->GetInputValue(drag), info.defaultValue));
        CHECK(input->Execute(&error));
        const std::string diff = _CompareRuns(*input, *fresh, false);
        CHECK(diff.empty());
        if (!diff.empty()) {
            std::printf("%s: reset run and fresh run differ at %s\n",
                        fixture.stage, diff.c_str());
        }
        resetNote = "reset == fresh";
        ++inputApiResets;
    }

    // Token text the file lacks is interned once, past the file's path
    // ids, and round-trips; known text resolves to the file's own id, the
    // empty token to id 0.
    std::string tokenNote = "n/a";
    const std::unique_ptr<fb::RigExecWireFile> unpacked =
        RigExecTestUnpack(bytes);
    const size_t paths = unpacked ? unpacked->paths.size() : 0;
    for (size_t i = 0; i < count; ++i) {
        const RigExecRuntimeInputInfo &info = fresh->GetInputInfo(i);
        if (info.type != RrInputTag::Token) {
            continue;
        }
        const std::unique_ptr<RigExecRuntimeReader> tokens = open();
        if (!tokens) {
            return;
        }
        const std::string text = "rigExecTestUnlistedToken";
        CHECK(tokens->SetInputToken(info.name, text, &why));
        const uint32_t id = tokens->GetInputValue(i).token;
        CHECK(tokens->GetInputValue(i).tag == RrInputTag::Token);
        CHECK(paths > 0 && id >= paths);
        CHECK(tokens->GetTokenText(id) == text);
        CHECK(tokens->SetInputToken(info.name, text, &why));
        CHECK(tokens->GetInputValue(i).token == id);
        const std::string known =
            tokens->GetTokenText(info.defaultValue.token);
        CHECK(tokens->SetInputToken(info.name, known, &why));
        CHECK(tokens->GetInputValue(i).token < paths);
        CHECK(tokens->GetTokenText(tokens->GetInputValue(i).token) == known);
        CHECK(tokens->SetInputToken(info.name, "", &why));
        CHECK(tokens->GetInputValue(i).token == 0);
        CHECK(tokens->GetTokenText(0).empty());
        CHECK(tokens->SetInputToken(info.name, text, &why));
        CHECK(tokens->Execute(&error));
        tokenNote = info.name;
        ++inputApiTokens;
        break;
    }
    if (tokenNote == "n/a" && count > 0) {
        // No Token input: SetInputToken refuses every input as not a token.
        const RigExecRuntimeInputInfo &info = fresh->GetInputInfo(0);
        why.clear();
        CHECK(!fresh->SetInputToken(info.name, "x", &why));
        CHECK(!why.empty());
    }
    std::printf("input api %s: %zu input(s), %zu animated; defaults, "
                "lookup and refusals ok; %zu table field(s) found; %s set: "
                "%s; %s; token: %s\n",
                fixture.stage, count, animated, fields.size(),
                dragName.empty() ? "-" : dragName.c_str(), moveNote.c_str(),
                resetNote.c_str(), tokenNote.c_str());
    ++inputApiRows;
}

static void
_TestFixture(const RigExecExampleFixture &fixture,
             const std::string &stagePath,
             const std::vector<double> &bakeFrames)
{
    const UsdStageRefPtr stage = UsdStage::Open(stagePath);
    CHECK(stage);
    if (!stage) {
        return;
    }
    const SdfPath rigPath = _FindRig(stage);
    CHECK(!rigPath.IsEmpty());
    if (rigPath.IsEmpty()) {
        return;
    }
    RigExecRigEvaluator evaluator(stage, rigPath);
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::string error;

    // One record at the probe time: the defaults, then the input API.
    std::vector<uint8_t> atProbe;
    CHECK(RigExecTestBakeAt(evaluator,
                            std::numeric_limits<double>::quiet_NaN(),
                            &atProbe, &error));
    if (atProbe.empty()) {
        std::printf("%s: probe bake: %s\n", fixture.stage, error.c_str());
        return;
    }
    // The bake writes a file the format's validator accepts, so Open
    // takes it.
    CHECK(RigExecRuntimeReader::Open(atProbe.data(), atProbe.size(),
                                     &error));

    if (_TestStepLabels(fixture.stage, evaluator, atProbe)) {
        ++stepLabelRows;
    }
    _TestStepGraphRefusals(stagePath, atProbe);
    _TestStaticTableRefusals(stagePath, atProbe);
    _TestPointBindingRefusals(stagePath, atProbe);

    // Truncating the tail breaks the buffer, so Open refuses.
    if (atProbe.size() > 64) {
        std::string truncError;
        CHECK(!RigExecRuntimeReader::Open(
            atProbe.data(), atProbe.size() - 32, &truncError));
        CHECK(!truncError.empty());
    }

    _TestDefaults(fixture, stage, rigPath, atProbe);
    _TestInputApi(fixture, stage, atProbe);

    // One record at the first table frame, played along the table.
    std::vector<uint8_t> atFirst;
    if (RigExecBakedProbeTime(stage).GetValue() == bakeFrames.front()) {
        atFirst = atProbe;
    } else {
        CHECK(RigExecTestBakeAt(evaluator, bakeFrames.front(), &atFirst,
                                &error));
    }
    if (atFirst.empty()) {
        std::printf("%s: first-frame bake: %s\n", fixture.stage,
                    error.c_str());
        return;
    }
    _TestSamplerDrive(fixture, stage, rigPath, atFirst, bakeFrames);
}

int
main(int argc, char **argv)
{
    PlugRegistry::GetInstance().RegisterPlugins(
        RIGEXEC_SCHEMA_RESOURCE_DIR);
    _TestMalformed();

    std::string examplesDir = RIGEXEC_EXAMPLES_DIR;
    if (argc > 1) {
        examplesDir = argv[1];
    }
    bool sawBaking = false;
    bool sawBytes = false;
    int bakingRows = 0;
    for (const RigExecExampleFixture &fixture : kRigExecExampleFixtures) {
        if (!fixture.bakesToday) {
            continue;
        }
        sawBaking = true;
        ++bakingRows;
        const std::string stagePath =
            examplesDir + "/" + fixture.stage;
        const std::vector<double> frames = _ParseFrames(fixture.frames);
        CHECK(!frames.empty());
        if (frames.empty()) {
            continue;
        }
        _TestFixture(fixture, stagePath, frames);
        if (!sawBytes) {
            // One fresh-reader Execute on the first baking rig.
            const UsdStageRefPtr stage = UsdStage::Open(stagePath);
            if (stage && !_FindRig(stage).IsEmpty()) {
                RigExecRigEvaluator evaluator(stage, _FindRig(stage));
                evaluator.SetEvaluationMode(
                    RigExecEvaluationMode::Baked);
                std::vector<uint8_t> bytes;
                std::string error;
                if (RigExecTestBakeAt(evaluator, frames.front(), &bytes,
                                      &error) &&
                    !bytes.empty()) {
                    _TestFreshExecute(bytes);
                    _TestFileRefusals(bytes);
                    sawBytes = true;
                }
            }
        }
    }
    CHECK(sawBaking);
    CHECK(sawBytes);
    _TestPhaseFixtures();
    std::printf("phase fixtures: %d of 3 with every step label matching; "
                "%d frame record refusal(s), %d record list refusal(s), %d "
                "retired value refusal(s), %d per-volume refusal(s), %d "
                "previous version refusal(s)\n",
                phaseFixtureRows, recordRefusals, recordListRefusals,
                reservedRefusals, volumeRefusals, versionRefusals);
    CHECK(phaseFixtureRows == 3);
    CHECK(recordRefusals > 0);
    CHECK(recordListRefusals > 0);
    CHECK(reservedRefusals > 0);
    CHECK(volumeRefusals > 0);
    CHECK(versionRefusals > 0);
    _TestOwnChainPreceding();
    std::printf("own-chain preceding: %d run(s) equal to live baked and off "
                "the base read; %d refusal(s)\n",
                ownChainRuns, ownChainRefusals);
    CHECK(ownChainRuns == 1);
    CHECK(ownChainRefusals == 1);
    _TestRawSkinLayouts();
    std::printf("raw skin layouts: %d run(s) equal to live baked; %d "
                "refusal(s)\n",
                rawLayoutRuns, rawLayoutRefusals);
    CHECK(rawLayoutRuns == 4);
    CHECK(rawLayoutRefusals == 13);
    std::printf("point binding refusals: %d on %d baking row(s) with a bound "
                "phased read\n",
                pointBindingRefusals, pointBindingRows);
    CHECK(pointBindingRows > 0);
    CHECK(pointBindingRefusals > 0);
    // Every baking fixture opened above; the refusals need a step with a
    // predecessor, a cluster and a cluster edge, which some fixture must
    // have.
    std::printf("step graph refusals: %d flipped predecessor(s), %d index "
                "range(s), %d cluster cycle(s), %d unproduced read(s)\n",
                stepGraphFlips, stepGraphRanges, stepGraphCycles,
                stepGraphProducers);
    CHECK(stepGraphProducers > 0);
    CHECK(stepGraphFlips > 0);
    CHECK(stepGraphRanges > 0);
    CHECK(stepGraphCycles > 0);
    std::printf("file refusals: %d row(s) (identifier, format "
                "version)\n",
                fileRefusalRows);
    CHECK(fileRefusalRows == 1);
    std::printf("static table refusals: %d of %d baking row(s); chain base "
                "on %d, constraint arrays on %d\n",
                staticTableRows, bakingRows, staticChainRows,
                staticArrayRows);
    CHECK(staticTableRows == bakingRows);
    CHECK(staticChainRows > 0);
    CHECK(staticArrayRows > 0);
    std::printf("step labels: %d of %d baking row(s) match the "
                "program's\n",
                stepLabelRows, bakingRows);
    CHECK(stepLabelRows == bakingRows);
    // Every baking row ran the input API; the set, the reset and the token
    // cases need a fixture that has each.
    std::printf("input api: %d of %d baking row(s), %d table field(s) "
                "found as inputs, %d set(s) moving the outputs, %d "
                "reset(s), %d token input(s)\n",
                inputApiRows, bakingRows, inputApiFields, inputApiMoved,
                inputApiResets, inputApiTokens);
    CHECK(inputApiRows == bakingRows);
    std::printf("defaults: %d of %d baking row(s) execute their defaults "
                "as the bake time's generation\n",
                defaultsRows, bakingRows);
    CHECK(defaultsRows == bakingRows);
    // The rows with no Animated input are the ones only the sampler's
    // change of time keeps in step with the program.
    std::printf("sampler: %d of %d baking row(s) driven along their frames "
                "(%d frame(s); %d inputs row(s) with no Animated input)\n",
                samplerRows, bakingRows, samplerFrames,
                samplerUnanimatedRows);
    CHECK(samplerRows == bakingRows);
    CHECK(samplerUnanimatedRows > 0);
    CHECK(inputApiFields > 0);
    CHECK(inputApiMoved > 0);
    CHECK(inputApiResets > 0);

    if (failures == 0) {
        std::printf("testRigExecRuntimeLoader: all tests passed\n");
        return 0;
    }
    std::printf("testRigExecRuntimeLoader: %d failures\n", failures);
    return 1;
}
