//
// The baked program's step graph must describe the straight line it replaced.
//
// Every other suite asserts on what a frame PUBLISHES, which the serial
// executor gets right by running the steps in program order whatever the
// edges say. That is exactly why the edges need a test of their own: they are
// the part of this design nothing else exercises yet, and the first thing the
// parallel executor will trust. So this suite asks the four questions whose
// wrong answer makes a parallel run wrong while leaving a serial run
// perfect:
//
//   * every edge points FORWARD in program order, so running the steps in
//     index order is always a topological order;
//   * every declared read either has a writer earlier in the program or names
//     a domain the prologue fills, so nothing reads a slot the graph never
//     produced;
//   * two steps that declare overlapping WRITES are ordered with respect to
//     each other, so no schedule can run them at once;
//   * the report is deterministic, so a schedule can be diffed between two
//     builds of the same stage and a change in it is a change someone made.
//
// The geometry section below asks the two questions the vertex partition
// adds: that the chunks of a skin revision cover every vertex exactly once
// and that no chunk is missing an influence one of its own vertices names --
// a chunk that skins a vertex against an identity it never noticed is a
// silently wrong deformation, not a crash -- and that a revision whose packet
// the frame rejects passes its preceding points through exactly as the
// dynamic path does, which is the decision the fuse took over from the
// kernel.
//
// argv[1] = path to the examples directory (containing biped/Biped.usda).
//
#include "rigExec/bakedProgram.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/bakedSchedule.h"
#include "rigExec/parallel.h"
#include "rigExec/moverGraph.h"
#include "rigExecMath/simdKernels.h"
#include "rigExecMath/solvers.h"
#include "rigExec/rigEvaluator.h"
#include "rigExec/tapSet.h"

#include "pxr/base/plug/registry.h"
#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/vt/value.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"

#include <algorithm>
#include <cmath>
#include <set>

#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

using namespace rigExec;

PXR_NAMESPACE_USING_DIRECTIVE

static int failures = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            ++failures;                                                    \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
        }                                                                  \
    } while (0)

static SdfPath
FindRig(const UsdStageRefPtr &stage)
{
    for (const UsdPrim &prim : stage->Traverse()) {
        if (prim.GetTypeName() == "RigExecRoot") {
            return prim.GetPath();
        }
    }
    return SdfPath();
}

namespace {

/// One compiled, baked rig, kept alive together with the stage and evaluator
/// the program points into.
struct BuiltProgram {
    UsdStageRefPtr stage;
    std::unique_ptr<RigExecRigEvaluator> evaluator;
    std::unique_ptr<RigExecBakedProgram> program;
};

BuiltProgram
Build(const std::string &stagePath)
{
    BuiltProgram built;
    built.stage = UsdStage::Open(stagePath);
    if (!built.stage) {
        return built;
    }
    const SdfPath rigPath = FindRig(built.stage);
    if (rigPath.IsEmpty()) {
        return built;
    }
    built.evaluator =
        std::make_unique<RigExecRigEvaluator>(built.stage, rigPath);
    std::vector<std::string> errors;
    if (!built.evaluator->Compile(&errors)) {
        return built;
    }
    std::vector<std::string> reasons;
    built.program = RigExecBakedProgram::Build(built.evaluator.get(),
                                               &reasons);
    if (!built.program) {
        for (const std::string &reason : reasons) {
            std::printf("    not bakeable: %s\n", reason.c_str());
        }
    }
    return built;
}

/// A dense bitset over steps, for the reachability the write-conflict check
/// needs: two steps that write the same slot must be ordered, and the edge
/// that orders them is often two hops away because a third step wrote
/// between them.
class Reachability {
public:
    explicit Reachability(const std::vector<RigExecBakedStep> &steps)
        : _words((steps.size() + 63) / 64), _bits(steps.size() * _words, 0)
    {
        // Program order is a topological order, so one forward pass closes
        // the relation: a step reaches whatever its predecessors reached,
        // plus the predecessors themselves.
        for (size_t index = 0; index < steps.size(); ++index) {
            uint64_t *row = &_bits[index * _words];
            for (const int pred : steps[index].preds) {
                row[size_t(pred) / 64] |= uint64_t(1) << (size_t(pred) % 64);
                const uint64_t *predecessor = &_bits[size_t(pred) * _words];
                for (size_t w = 0; w < _words; ++w) {
                    row[w] |= predecessor[w];
                }
            }
        }
    }

    bool Ordered(size_t earlier, size_t later) const
    {
        return (_bits[later * _words + earlier / 64] >>
                (earlier % 64)) & uint64_t(1);
    }

private:
    size_t _words;
    std::vector<uint64_t> _bits;
};

void
TestTheGraphDescribesTheProgram(const std::string &stagePath,
                                const char *name)
{
    const BuiltProgram built = Build(stagePath);
    CHECK(built.program != nullptr);
    if (!built.program) {
        return;
    }
    const RigExecBakedProgramImpl &B = built.program->GetStepGraph();
    CHECK(!B.steps.empty());

    // (1) Every edge points forward, and the identity permutation is
    // therefore a topological order -- which is what "the serial executor
    // runs the steps in program order" is allowed to mean.
    size_t edges = 0;
    for (size_t index = 0; index < B.steps.size(); ++index) {
        for (const int pred : B.steps[index].preds) {
            ++edges;
            if (pred < 0 || size_t(pred) >= index) {
                ++failures;
                std::printf("FAIL %s: step %zu (%s) depends on step %d, "
                            "which is not earlier in program order\n",
                            name, index, B.steps[index].label.c_str(), pred);
            }
        }
        // preds and succs describe the same relation.
        for (const int succ : B.steps[index].succs) {
            if (succ < 0 || size_t(succ) <= index) {
                ++failures;
                std::printf("FAIL %s: step %zu names successor %d\n", name,
                            index, succ);
            }
        }
    }
    CHECK(edges > 0);

    // (2) Every declared read is produced by something: an earlier step, or
    // the prologue.
    for (size_t index = 0; index < B.steps.size(); ++index) {
        for (const RigExecBakedSlotRange &read : B.steps[index].reads) {
            if (RigExecBakedIsSourceDomain(read.domain) || read.IsEmpty()) {
                continue;
            }
            bool written = false;
            for (size_t earlier = 0; earlier < index && !written; ++earlier) {
                for (const RigExecBakedSlotRange &write :
                         B.steps[earlier].writes) {
                    if (write.Overlaps(read)) {
                        written = true;
                        break;
                    }
                }
            }
            if (!written) {
                ++failures;
                std::printf("FAIL %s: step %zu (%s) reads %s[%u,%u) which no "
                            "earlier step writes\n", name, index,
                            B.steps[index].label.c_str(),
                            RigExecBakedSlotDomainName(read.domain),
                            read.begin, read.end);
            }
        }
    }

    // (3) Two steps whose declared writes overlap are ordered. A declared
    // write is an upper bound, so this is stricter than what any run does --
    // which is the point: a scheduler may only look at the declarations.
    const Reachability reachable(B.steps);
    size_t conflicts = 0;
    for (size_t earlier = 0; earlier < B.steps.size(); ++earlier) {
        for (size_t later = earlier + 1; later < B.steps.size(); ++later) {
            if (reachable.Ordered(earlier, later)) {
                continue;
            }
            for (const RigExecBakedSlotRange &a : B.steps[earlier].writes) {
                for (const RigExecBakedSlotRange &b : B.steps[later].writes) {
                    if (!a.Overlaps(b)) {
                        continue;
                    }
                    if (++conflicts <= 4) {
                        std::printf("FAIL %s: steps %zu (%s) and %zu (%s) "
                                    "both write %s[%u,%u) with no edge "
                                    "between them\n", name, earlier,
                                    B.steps[earlier].label.c_str(), later,
                                    B.steps[later].label.c_str(),
                                    RigExecBakedSlotDomainName(a.domain),
                                    b.begin, b.end);
                    }
                }
            }
        }
    }
    failures += int(conflicts);

    // (4) And a step that reads a slot an unordered step writes is the same
    // race seen from the other side.
    size_t races = 0;
    for (size_t earlier = 0; earlier < B.steps.size(); ++earlier) {
        for (size_t later = earlier + 1; later < B.steps.size(); ++later) {
            if (reachable.Ordered(earlier, later)) {
                continue;
            }
            for (const RigExecBakedSlotRange &write :
                     B.steps[earlier].writes) {
                for (const RigExecBakedSlotRange &read :
                         B.steps[later].reads) {
                    if (write.Overlaps(read)) {
                        ++races;
                    }
                }
            }
            for (const RigExecBakedSlotRange &read : B.steps[earlier].reads) {
                for (const RigExecBakedSlotRange &write :
                         B.steps[later].writes) {
                    if (write.Overlaps(read)) {
                        ++races;
                    }
                }
            }
        }
    }
    if (races) {
        ++failures;
        std::printf("FAIL %s: %zu unordered read/write slot overlap(s)\n",
                    name, races);
    }
    std::printf("  %s: %zu step(s), %zu edge(s)\n", name, B.steps.size(),
                edges);
}

/// Two builds of one stage produce the same schedule, character for
/// character. Without this the report is a debugging aid nobody can diff;
/// with it, a schedule change shows up in a review.
void
TestTheReportIsDeterministic(const std::string &stagePath)
{
    const BuiltProgram first = Build(stagePath);
    const BuiltProgram second = Build(stagePath);
    CHECK(first.program != nullptr);
    CHECK(second.program != nullptr);
    if (!first.program || !second.program) {
        return;
    }
    const std::string a =
        RigExecBakedScheduleReport(first.program->GetStepGraph());
    const std::string b =
        RigExecBakedScheduleReport(second.program->GetStepGraph());
    CHECK(!a.empty());
    if (a == b) {
        return;
    }
    ++failures;
    std::printf("FAIL: the schedule report is not deterministic\n");
    size_t line = 0, offset = 0;
    while (offset < a.size() && offset < b.size() && a[offset] == b[offset]) {
        if (a[offset] == '\n') ++line;
        ++offset;
    }
    std::printf("    first difference at line %zu\n", line);
}

/// The mode the whole parity argument rests on: anything but an explicit
/// RIGEXEC_BAKED_SCHEDULE=parallel is the reference order, and asking for
/// parallel with rigExec's own threading switched off is too. The suite runs
/// under all three of those environments (verify_sched.sh), so the assertion
/// is on the MAPPING rather than on one answer.
void
TestTheModeIsTheOneTheEnvironmentAsked()
{
    const bool asked = TfGetenv("RIGEXEC_BAKED_SCHEDULE", "serial") ==
                       "parallel";
    const RigExecBakedScheduleMode expected =
        asked && RigExecParallelEvaluationEnabled()
            ? RigExecBakedScheduleMode::Parallel
            : RigExecBakedScheduleMode::Serial;
    CHECK(RigExecBakedScheduleModeFromEnvironment() == expected);
}

/// A synthetic skin packet: \p influences joints, \p points vertices,
/// elementSize slots per vertex, weights and indices that vary per vertex.
RigExecMoverParameters
SyntheticSkinPacket(size_t influences, size_t points, size_t elementSize,
                    const TfToken &method)
{
    RigExecMoverParameters packet;
    packet.kind = TfToken("skin");
    packet.enabled = true;
    packet.valid = true;
    packet.skinningMethod = method;
    packet.weights = RigExecWeightPacket::Constant(1.0f);
    packet.skinElementSize = int(elementSize);
    for (size_t t = 0; t < influences; ++t) {
        GfMatrix4d matrix(1.0);
        matrix.SetTranslate(GfVec3d(double(t) * 0.25, double(t) * -0.125,
                                    double(t) * 0.5));
        matrix[0][0] = 1.0 + 0.01 * double(t);
        matrix[1][2] = 0.02 * double(t);
        packet.skinTransforms.push_back(matrix);
    }
    for (size_t i = 0; i < points; ++i) {
        float total = 0.0f;
        for (size_t k = 0; k < elementSize; ++k) {
            packet.skinIndices.push_back(int((i * 7 + k * 3) % influences));
            const float weight = float((i + k + 1) % 5) / 10.0f;
            packet.skinWeights.push_back(weight);
            total += weight;
        }
        // A shortfall on some vertices and a full set on others, so the
        // complement rule is exercised on both sides.
        if (total > 0.9f) {
            packet.skinWeights[i * elementSize] +=
                std::max(0.0f, 1.0f - total);
        }
    }
    return packet;
}

/// The range form deforms a vertex exactly as the whole-array form does.
///
/// This is the shared-kernel rule as an assertion: the chunked path and the
/// unchunked one run the same per-vertex body, so a mesh cut into ranges is
/// bit-identical to the same mesh deformed whole -- not equal to tolerance,
/// bit-identical -- for both skinning methods and whether the caller narrows
/// and splits the influence table itself or lets the kernel do it. The rigs
/// that bake today are all classicLinear, so without this the
/// dual-quaternion range form has no fixture at all.
void
TestTheRangeFormDeformsLikeTheWholeArray(const char *method)
{
    const size_t influences = 11, points = 9000, elementSize = 4;
    const RigExecMoverParameters packet = SyntheticSkinPacket(
        influences, points, elementSize, TfToken(method));
    std::vector<GfVec3f> rest(points);
    for (size_t i = 0; i < points; ++i) {
        rest[i] = GfVec3f(float(i % 37) * 0.5f, float(i % 11) * 1.25f,
                          float(i % 5) * -2.0f);
    }

    std::vector<GfVec3f> whole = rest;
    CHECK(RigExecApplySkinKernel(packet, &whole));

    // The same influence table, narrowed and split once by the caller -- the
    // forms a chunk keeps beside its own matrices.
    std::vector<float> rows(influences * RigExecSkinRowStride);
    for (size_t t = 0; t < influences; ++t) {
        RigExecNarrowSkinRows(packet.skinTransforms[t],
                              &rows[t * RigExecSkinRowStride]);
    }
    RigExecSkinLayout layout;
    layout.transforms = packet.skinTransforms.data();
    layout.transformCount = influences;
    const std::vector<RigExecScaledDualQuat> palette =
        RigExecSkinDualQuatPalette(layout);

    for (const bool supplied : {false, true}) {
        for (const size_t chunk : {size_t(1), size_t(512), size_t(4096)}) {
            RigExecSkinTransformsView view;
            view.transforms = packet.skinTransforms.data();
            view.transformCount = influences;
            if (supplied) {
                view.rows = rows.data();
                view.palette = palette.data();
                view.paletteSize = palette.size();
            }
            std::vector<GfVec3f> ranged = rest;
            bool ok = true;
            for (size_t begin = 0; begin < points; begin += chunk) {
                ok = ok && RigExecApplySkinKernelRange(
                               packet, view, begin,
                               std::min(points, begin + chunk), &ranged);
            }
            CHECK(ok);
            if (ranged != whole) {
                ++failures;
                std::printf("FAIL %s: ranges of %zu%s differ from the whole "
                            "array\n", method, chunk,
                            supplied ? " (caller's tables)" : "");
            }
        }
    }
    std::printf("  %s: range forms are bit-identical to the whole array\n",
                method);
}

/// The skin mover of \p stage, or an empty path.
SdfPath
FindSkinMover(const UsdStageRefPtr &stage)
{
    for (const UsdPrim &prim : stage->Traverse()) {
        if (prim.GetTypeName() == "RigExecSkinMover") {
            return prim.GetPath();
        }
    }
    return SdfPath();
}

/// Every skin revision's partition covers its vertices exactly once, stays
/// under the cap, and gives each chunk a key that contains every influence
/// its own vertices name.
///
/// The last one is the invariant the whole design rests on: a chunk fills the
/// entries of its key and leaves the rest of the table identity, so an
/// influence missing from a key is a vertex quietly skinned against the
/// identity. Checked against the authored arrays rather than against the
/// partition's own bookkeeping, so the check cannot agree with a bug by
/// reading it back.
void
TestTheVertexPartitionCoversEveryVertexOnce(const std::string &stagePath,
                                            const char *name, bool report)
{
    const BuiltProgram built = Build(stagePath);
    CHECK(built.program != nullptr);
    if (!built.program) {
        return;
    }
    // One frame, so the partition can be asked whether it SURVIVED contact
    // with the layout the packet carries: a stale one is correct -- the fuse
    // runs the revision whole -- and would make every assertion below true
    // of chunks nothing ran.
    RigExecRigPose pose;
    CHECK(built.program->Run(UsdTimeCode::Default(), &pose));
    const RigExecBakedProgramImpl &B = built.program->GetStepGraph();
    const size_t cap =
        size_t(std::max(1, TfGetenvInt("RIGEXEC_BAKED_MAX_CHUNKS", 32)));
    size_t skinRevisions = 0;
    for (const RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        for (const RigExecBakedProgramImpl::GeomRevision &revision :
                 chain.revisions) {
            if (revision.op != RigExecRevisionOp::Skin) {
                continue;
            }
            ++skinRevisions;
            CHECK(!revision.chunks.empty());
            CHECK(revision.chunks.size() <= cap);
            // The packet's table is identity and sized to the influences:
            // that is what lets the assemble run before the fold.
            CHECK(revision.packetInfluences.size() ==
                  revision.influenceSlots.size());
            for (const GfMatrix4d &matrix : revision.packetInfluences) {
                CHECK(matrix == GfMatrix4d(1.0));
            }
            if (!revision.chunked) {
                continue;
            }
            // The frame agreed that the keys still describe the vertices,
            // and every chunk produced its range.
            CHECK(!revision.partitionStale);
            for (const RigExecBakedProgramImpl::GeomChunk &chunk :
                     revision.chunks) {
                CHECK(chunk.ok);
            }
            // (1) The ranges are contiguous, ascending and cover exactly
            // [0, pointCount).
            int expected = 0;
            for (const RigExecBakedProgramImpl::GeomChunk &chunk :
                     revision.chunks) {
                CHECK(chunk.begin == expected);
                CHECK(chunk.end >= chunk.begin);
                expected = chunk.end;
            }
            CHECK(size_t(expected) == revision.partitionPointCount);

            // (2) Every vertex's influences are in its chunk's key, read
            // back out of the authored layout.
            const UsdPrim mover =
                built.stage->GetPrimAtPath(revision.moverPath);
            VtIntArray indices;
            int elementSize = 0;
            if (const UsdAttribute a =
                    mover.GetAttribute(TfToken("rigExec:jointIndices"))) {
                a.Get(&indices, UsdTimeCode::Default());
            }
            if (const UsdAttribute a =
                    mover.GetAttribute(TfToken("rigExec:elementSize"))) {
                a.Get(&elementSize, UsdTimeCode::Default());
            }
            CHECK(elementSize >= 1);
            CHECK(indices.size() ==
                  revision.partitionPointCount * size_t(elementSize));
            if (elementSize < 1) {
                continue;
            }
            size_t missing = 0;
            for (const RigExecBakedProgramImpl::GeomChunk &chunk :
                     revision.chunks) {
                const std::set<int> key(chunk.key.begin(), chunk.key.end());
                CHECK(key.size() == chunk.key.size());  // sorted, unique
                for (size_t point = size_t(chunk.begin);
                     point < size_t(chunk.end); ++point) {
                    for (size_t slot = 0; slot < size_t(elementSize); ++slot) {
                        const int index =
                            indices[point * size_t(elementSize) + slot];
                        if (index < 0 ||
                            size_t(index) >= revision.influenceSlots.size()) {
                            continue;  // the packet will reject the layout
                        }
                        if (!key.count(index)) {
                            ++missing;
                        }
                    }
                }
            }
            if (missing) {
                ++failures;
                std::printf("FAIL %s: %s has %zu vertex influence(s) outside "
                            "their chunk's key\n", name,
                            revision.moverPath.GetString().c_str(), missing);
            }
        }
    }
    std::printf("  %s: %zu skin revision(s)\n", name, skinRevisions);
    if (report) {
        std::printf("%s", RigExecBakedGeometryReport(B).c_str());
    }
}

/// A skin revision the frame rejects publishes what the dynamic path
/// publishes: the preceding points, and the MoverFailed line.
///
/// The decision moved: the kernel used to make it, and now the fuse does,
/// out of a packet that no longer carries the influence matrices and a fold
/// that checks them separately. So the fixture drives a rejection through an
/// interactive override -- the route a manipulator uses, and the one that
/// reaches the packet without rebuilding the program -- and asks the parity
/// mode, which runs both paths in one generation and compares every
/// published map, whether they agreed.
void
TestARejectedSkinPacketPassesThroughLikeTheDynamicPath(
    const std::string &stagePath, const char *what, const TfToken &attribute,
    const VtValue &value)
{
    const UsdStageRefPtr stage = UsdStage::Open(stagePath);
    CHECK(stage != nullptr);
    if (!stage) {
        return;
    }
    const SdfPath rigPath = FindRig(stage);
    const SdfPath moverPath = FindSkinMover(stage);
    CHECK(!rigPath.IsEmpty());
    CHECK(!moverPath.IsEmpty());
    if (rigPath.IsEmpty() || moverPath.IsEmpty()) {
        return;
    }
    RigExecRigEvaluator evaluator(stage, rigPath);
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    CHECK(evaluator.IsBakeable());
    evaluator.SetEvaluationMode(RigExecEvaluationMode::BakedWithParityCheck);

    const RigExecRigPose clean = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(clean.valid);
    CHECK(clean.bakedParityMismatches == 0);

    RigExecValueOverride override;
    override.prim = moverPath;
    override.attribute = attribute;
    override.value = value;
    evaluator.SetInteractiveOverrides({override});
    const RigExecRigPose rejected = evaluator.Evaluate(UsdTimeCode::Default());
    evaluator.ClearInteractiveOverrides();

    CHECK(rejected.valid);
    if (rejected.bakedParityMismatches != 0) {
        ++failures;
        std::printf("FAIL %s: %zu baked/dynamic mismatch(es)\n", what,
                    rejected.bakedParityMismatches);
        for (const std::string &diagnostic : rejected.diagnostics) {
            std::printf("    %s\n", diagnostic.c_str());
        }
        return;
    }
    // And the fixture really did reject the revision, rather than agree
    // about a generation in which nothing happened.
    bool failed = false;
    for (const std::string &diagnostic : rejected.diagnostics) {
        failed = failed ||
                 diagnostic.find("MoverFailed " + moverPath.GetString()) !=
                     std::string::npos;
    }
    if (!failed) {
        ++failures;
        std::printf("FAIL %s: the revision was not rejected at all\n", what);
        return;
    }
    // Rejected means the preceding points pass through, so the mesh is no
    // longer where the skin put it.
    size_t moved = 0;
    for (const auto &[target, points] : clean.movedProperties) {
        const auto found = rejected.movedProperties.find(target);
        if (found != rejected.movedProperties.end() &&
            found->second != points) {
            ++moved;
        }
    }
    if (!moved) {
        ++failures;
        std::printf("FAIL %s: the rejected revision published the same "
                    "points as the applied one\n", what);
    }
    std::printf("  %s: passed through, %zu published target(s) moved\n", what,
                moved);
}

/// The predicate the fold hands the fuse, on the one input the pose walk
/// cannot produce.
///
/// A provider matrix is always finite -- RigExecPointsToMatrix leaves the
/// identity where it cannot solve -- so no rig fixture can put a non-finite
/// matrix in front of the fold. The check is still the one the dynamic path's
/// assembler makes over the same table, and a fold that stopped making it
/// would hand a NaN to the kernel, so it is asserted directly.
void
TestTheInfluenceValidityCheckRejectsWhatTheAssemblerRejects()
{
    std::vector<GfMatrix4d> table(3, GfMatrix4d(1.0));
    CHECK(RigExecSkinTransformsAreUsable(table.data(), table.size()));
    table[1][2][0] = std::nan("");
    CHECK(!RigExecSkinTransformsAreUsable(table.data(), table.size()));
    table[1][2][0] = 0.0;
    table[1][1][3] = 0.5;  // not affine
    CHECK(!RigExecSkinTransformsAreUsable(table.data(), table.size()));
    // An empty table is what a skin mover with no influences assembles, and
    // the assembler rejects that too.
    CHECK(!RigExecSkinTransformsAreUsable(table.data(), 0));
}

std::string
SchemaResourceDir(const std::string &examplesDir)
{
#ifdef RIGEXEC_SCHEMA_RESOURCE_DIR
    (void)examplesDir;
    return TfAbsPath(RIGEXEC_SCHEMA_RESOURCE_DIR);
#else
    return TfAbsPath(examplesDir + "/../plugin/rigExecSchema/resources");
#endif
}

}  // namespace

int
main(int argc, char **argv)
{
    if (argc < 2) {
        std::printf("usage: testRigExecBakedSchedule <examplesDir>\n");
        return 2;
    }
    const std::string examplesDir = argv[1];
    const std::string resources = SchemaResourceDir(examplesDir);
    if (PlugRegistry::GetInstance().RegisterPlugins(resources).empty()) {
        std::printf("FATAL: no schema plugin found at %s\n",
                    resources.c_str());
        return 2;
    }
    TestTheModeIsTheOneTheEnvironmentAsked();
    TestTheGraphDescribesTheProgram(examplesDir + "/biped/Biped.usda",
                                    "Biped");
    TestTheGraphDescribesTheProgram(examplesDir + "/biped/Biped_anim.usda",
                                    "Biped_anim");
    TestTheGraphDescribesTheProgram(
        examplesDir + "/spider_legs_assembly_ref.usda", "spider_legs");
    TestTheReportIsDeterministic(examplesDir + "/biped/Biped.usda");
    TestTheInfluenceValidityCheckRejectsWhatTheAssemblerRejects();
    TestTheRangeFormDeformsLikeTheWholeArray("classicLinear");
    TestTheRangeFormDeformsLikeTheWholeArray("dualQuaternion");
    TestTheVertexPartitionCoversEveryVertexOnce(
        examplesDir + "/biped/Biped.usda", "Biped", /*report=*/true);
    TestTheVertexPartitionCoversEveryVertexOnce(
        examplesDir + "/spider_legs_assembly_ref.usda", "spider_legs",
        /*report=*/false);
    TestARejectedSkinPacketPassesThroughLikeTheDynamicPath(
        examplesDir + "/biped/Biped.usda", "a non-finite defaultWeight",
        TfToken("inputs:defaultWeight"), VtValue(std::nanf("")));
    TestARejectedSkinPacketPassesThroughLikeTheDynamicPath(
        examplesDir + "/biped/Biped.usda", "a non-finite jointWeight",
        TfToken("rigExec:jointWeights"), VtValue(VtFloatArray()));
    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecBakedSchedule: all tests passed\n");
    return 0;
}
