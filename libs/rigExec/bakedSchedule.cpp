//
// The baked program's scheduler: edges, executors and the report.
//
// See bakedSchedule.h for what belongs here and what belongs with a domain.
//
#include "bakedSchedule.h"

#include "parallel.h"
#include "profiler.h"

#include "pxr/base/tf/diagnostic.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/token.h"
#include "pxr/base/vt/value.h"
#include "pxr/base/work/dispatcher.h"
#include "pxr/base/work/threadLimits.h"
#include "pxr/base/work/withScopedParallelism.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <memory>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

namespace rigExec {

const char *
RigExecBakedSlotDomainName(RigExecBakedSlotDomain domain)
{
    switch (domain) {
    case RigExecBakedSlotDomain::Avars: return "Avars";
    case RigExecBakedSlotDomain::PoseBase: return "PoseBase";
    case RigExecBakedSlotDomain::PoseFin: return "PoseFin";
    case RigExecBakedSlotDomain::PosedM: return "PosedM";
    case RigExecBakedSlotDomain::FinalMatrix: return "FinalMatrix";
    case RigExecBakedSlotDomain::BaseMatrix: return "BaseMatrix";
    case RigExecBakedSlotDomain::Aggregate: return "Aggregate";
    case RigExecBakedSlotDomain::SolverPoints: return "SolverPoints";
    case RigExecBakedSlotDomain::Candidates: return "Candidates";
    case RigExecBakedSlotDomain::CommitTable: return "CommitTable";
    case RigExecBakedSlotDomain::CommitDelta: return "CommitDelta";
    case RigExecBakedSlotDomain::CommitStaging: return "CommitStaging";
    case RigExecBakedSlotDomain::ConstraintDelta: return "ConstraintDelta";
    case RigExecBakedSlotDomain::PropertyResult: return "PropertyResult";
    case RigExecBakedSlotDomain::ChainBase: return "ChainBase";
    case RigExecBakedSlotDomain::RevisionPacket: return "RevisionPacket";
    case RigExecBakedSlotDomain::RevisionTransforms:
        return "RevisionTransforms";
    case RigExecBakedSlotDomain::RevisionOut: return "RevisionOut";
    case RigExecBakedSlotDomain::RevisionDone: return "RevisionDone";
    case RigExecBakedSlotDomain::ChainDirty: return "ChainDirty";
    case RigExecBakedSlotDomain::ChainPoints: return "ChainPoints";
    case RigExecBakedSlotDomain::DerivedOut: return "DerivedOut";
    case RigExecBakedSlotDomain::WeightPacket: return "WeightPacket";
    case RigExecBakedSlotDomain::WeightFrames: return "WeightFrames";
    case RigExecBakedSlotDomain::Snapshots: return "Snapshots";
    }
    return "unknown";
}

const char *
RigExecBakedStepKindName(RigExecBakedStepKind kind)
{
    switch (kind) {
    case RigExecBakedStepKind::ComposeSubtree: return "ComposeSubtree";
    case RigExecBakedStepKind::Solve: return "Solve";
    case RigExecBakedStepKind::SolverCommit: return "SolverCommit";
    case RigExecBakedStepKind::Constraint: return "Constraint";
    case RigExecBakedStepKind::CommitDelta: return "CommitDelta";
    case RigExecBakedStepKind::PropagateChunk: return "PropagateChunk";
    case RigExecBakedStepKind::CommitApply: return "CommitApply";
    case RigExecBakedStepKind::ProviderMatrix: return "ProviderMatrix";
    case RigExecBakedStepKind::SnapshotFinals: return "SnapshotFinals";
    case RigExecBakedStepKind::VolumePlacements: return "VolumePlacements";
    case RigExecBakedStepKind::WeightPacket: return "WeightPacket";
    case RigExecBakedStepKind::InfluenceFold: return "InfluenceFold";
    case RigExecBakedStepKind::RevisionStatic: return "RevisionStatic";
    case RigExecBakedStepKind::RevisionChunk: return "RevisionChunk";
    case RigExecBakedStepKind::RevisionFuse: return "RevisionFuse";
    case RigExecBakedStepKind::ChainStatus: return "ChainStatus";
    case RigExecBakedStepKind::Derived: return "Derived";
    }
    return "unknown";
}

// ---------------------------------------------------------------------------
// Execution mode.
// ---------------------------------------------------------------------------

RigExecBakedScheduleMode
RigExecBakedScheduleModeFromEnvironment()
{
    // Read once. A mode that could change between two frames of one session
    // would make "the same program produced two different traces" a question
    // about when the variable was read rather than about the schedule.
    static const RigExecBakedScheduleMode mode = [] {
        if (!RigExecParallelEvaluationEnabled()) {
            return RigExecBakedScheduleMode::Serial;
        }
        return TfGetenv("RIGEXEC_BAKED_SCHEDULE", "serial") == "parallel"
                   ? RigExecBakedScheduleMode::Parallel
                   : RigExecBakedScheduleMode::Serial;
    }();
    return mode;
}

bool
RigExecBakedScheduleReportRequested()
{
    static const bool requested =
        TfGetenvBool("RIGEXEC_BAKED_SCHEDULE_REPORT", false);
    return requested;
}

// ---------------------------------------------------------------------------
// The cost model.
//
// cost = a[kind] + b[kind] x size(step), in microseconds. Two constants per
// step kind and one size per step, which is as much model as a scheduler can
// use: the packing only ever asks "is this bin about a grain yet", so what it
// needs is the RATIO between a skin chunk and a constraint, not either one's
// absolute time.
//
// The size of a step is the count that its body's inner loop runs over:
//
//   ComposeSubtree   provider slots in the group
//   Solve            controls for an FK chain, joints for a spline IK,
//                    published elements otherwise
//   SolverCommit     candidate slots + propagation pairs -- BOTH halves of
//   Constraint       the commit walk the two of them, and a batch with forty
//                    candidates and no descendants is not free
//   CommitDelta      candidate slots
//   PropagateChunk   the staging slots this chunk writes, which IS its share
//                    of the propagation
//   CommitApply      candidate slots + propagation pairs
//   ProviderMatrix   one matrix; the whole step is its fixed term
//   SnapshotFinals   provider slots
//   InfluenceFold    influences -- NOT vertices; the fold is O(joints)
//   RevisionStatic   the target's vertices (ResolveAll of the envelope)
//   RevisionChunk    vertices x elementSize, NOT x influences: elementSize IS
//                    the influences per vertex the layout stores, and
//                    multiplying by the revision's whole influence count
//                    would make a 90-joint skin look thirty times the work a
//                    3-influence gather does
//   RevisionFuse     one decision
//   ChainStatus      the CHAIN's vertices -- the sweep ends by COPYING
//                    the chain's whole point array into its spare
//                    buffer, so its time is the mesh's and not the
//                    handful of MoverFailed lines it may also emit.
//                    Sizing it by the revision count made one 26k-point
//                    copy read as 7.8us of cost 'per revision'
//   Derived          the CHAIN's vertices, which is what recomputeNormals
//                    and recomputeExtent walk -- an extent is two vectors
//                    however large the mesh behind it is
//
// The constants below were fitted by RIGEXEC_BAKED_SCHEDULE_CALIBRATE=1 over
// eight frames of examples/biped/Biped_anim.usda on a 20-core box (see
// RigExecBakedScheduleCalibrationRequested). They are a machine's
// numbers, so they will be wrong on another machine by some factor -- which
// costs a schedule that is packed a little coarse or a little fine, and never
// an answer. Build must not measure: a schedule that depended on what the box
// was doing while the program was built could not be tested for producing the
// same values at every grain.
// ---------------------------------------------------------------------------

namespace {

/// The two constants of one step kind, in microseconds.
struct StepCostConstants {
    double fixedUs = 0;
    double perUnitUs = 0;
};

constexpr size_t kStepKindCount =
    size_t(RigExecBakedStepKind::Derived) + 1;

// Indexed by RigExecBakedStepKind, in the enum's order. Deliberately a
// deduced-extent array with the assertion below it: a std::array with a
// stated size accepts too few rows and zero-fills the rest, which would make
// a newly added step kind free and the schedule silently wrong.
constexpr StepCostConstants kStepCosts[] = {
    {0.0401, 0.087594},   // ComposeSubtree   93 samples
    {0.0000, 0.385597},   // Solve            14
    {0.0862, 0.024661},   // SolverCommit     14
    {1.2290, 0.014504},   // Constraint       65
    {0.0000, 0.066000},   // CommitDelta       2
    {0.0324, 0.016869},   // PropagateChunk    4
    {0.0000, 0.004088},   // CommitApply       2
    {0.0000, 0.051952},   // ProviderMatrix  252
    {0.0000, 0.077140},   // SnapshotFinals    1 -- 13_ReadPhases
    {0.0000, 0.143880},   // VolumePlacements  1 -- 11_VolumeWeights
    {0.0000, 0.066481},   // WeightPacket      5 -- 11_VolumeWeights
    {0.0000, 0.003781},   // InfluenceFold     1
    {0.0000, 0.000487},   // RevisionStatic    1
    {11.3239, 0.000714},  // RevisionChunk     7 -- see below
    {0.0000, 0.594000},   // RevisionFuse      1
    {0.0000, 0.000336},   // ChainStatus       1 -- see below
    {0.0000, 0.003744},   // Derived           1
};
static_assert(sizeof(kStepCosts) / sizeof(kStepCosts[0]) == kStepKindCount,
              "rigExec: every baked step kind needs a cost row");

// Four rows are worth reading twice before they are trusted:
//
//  * SnapshotFinals, VolumePlacements and WeightPacket were guesses until
//    Phase 3 made the rigs that exercise them bake; each is now the MEDIAN of
//    NINE calibration runs of the one rig that has it, at
//    RIGEXEC_BAKED_SCHEDULE_CALIBRATE=200 rather than at the default 8
//    frames, and each carries a caveat the next fitter should know. They are
//    fitted through the ORIGIN because the sample count could not separate
//    the two terms (see FitKind):
//    SnapshotFinals 0.0754..0.0814 over one step of 13_ReadPhases;
//    VolumePlacements 0.1377..0.1652 over one step of 11_VolumeWeights;
//    WeightPacket 0.0658..0.0674 over five steps of the same rig, which is
//    the only one of the three with enough steps for the fit to mean
//    anything. One run of each printed roughly double the rest and is not in
//    the spread; the medians are what nine runs agree on.
//
//    The frame COUNT is the part worth copying, because the first fit of
//    these three rows did not do it and was not reproducible. A rig with one
//    step of a kind gives the fit one sample per run, so the cold frame --
//    first touch of the arrays, cold caches, the arena still waking -- is an
//    eighth of the average at the default 8 frames and reads as about 1.8x
//    the steady-state cost, differently every run. The biped's rows do not
//    have that problem and do not need the longer run: 93 ComposeSubtree
//    samples or 252 ProviderMatrix samples move by 3-5% between 8 frames and
//    200 (0.0914 -> 0.0880, 0.0530 -> 0.0515), which is what puts every row
//    of this table on one scale. A row fitted on one step must be fitted
//    warm to join them.
//
//    All three sizes are small -- a handful of providers, one volume, a few
//    packet elements -- so the per-unit terms are honest at that scale and
//    extrapolate on trust. The guesses they replace were {1.0, 0.2},
//    {0.5, 0.01} and {0, 0.05}: SnapshotFinals was over-costed by nearly
//    three, VolumePlacements under-costed by fourteen, WeightPacket by a
//    third.
//  * ChainStatus is sized by the chain's vertices, and its row was
//    re-fitted after that correction: the median per-unit of four
//    calibration runs of the biped (0.000318 .. 0.000351), which
//    predicts 8.8us for the 26276 points the step copies. The row it
//    replaced read 7.79us per REVISION -- the same measurement with the
//    mesh hidden in it, which would have predicted 23us for a
//    three-revision chain whatever its size.
//  * RevisionChunk was re-fitted after the vertex partition landed, which is
//    what the row it replaced ({0.0000, 0.000914}) asked for: that one
//    measured ONE chunk over a whole skin, a range RigExecApplySkinKernel
//    spread over the arena itself, so it described the wall time of an
//    internally parallel step rather than the work in it. A chunk body is
//    unconditionally serial, so the new row is honest work. It is the median
//    of three calibration runs of the biped (fixed 10.34 .. 11.42, per-unit
//    0.000691 .. 0.000743). The FIXED term is the large one in this table for
//    a reason worth stating: a chunk gathers and narrows its key's matrices
//    before it deforms a vertex, so its real cost carries a |key| term this
//    model has no size for -- the biped's chunks range over 30 to 58
//    influences -- and the intercept is where that work lands. Giving
//    RevisionChunk a second size dimension would express it properly; until
//    then the row predicts the biped's chunks within a microsecond or two,
//    and a cost row can only pack a bin badly, never change an answer.


/// What a task costs to hand to the arena and pick up again, in the same
/// microseconds. The absorb rule compares a cluster against twice this: a
/// cluster that does not cost two dispatches is cheaper to run inside its
/// predecessor than to schedule.
constexpr double kSpawnCostUs = 1.0;

/// The vertex counts the geometry cost terms need, one entry per dense
/// revision id and per dense derived id.
///
/// Read from the stage ONCE, at Build, at the earliest time the attribute
/// answers to. A points array whose length moves with time makes this an
/// estimate rather than a fact -- which is all a cost model is, and the
/// program resets such a chain in its prologue anyway.
struct GeometrySizes {
    std::vector<double> revisionUnits;   ///< vertices x elementSize
    std::vector<double> revisionPoints;  ///< vertices
    /// The influence slots one vertex stores, per revision id: the factor a
    /// chunk's own vertex range is multiplied by.
    std::vector<double> revisionElementSize;
    /// The chain's vertices, per chain index: what the chain-wide steps
    /// walk and copy.
    std::vector<double> chainPoints;
    /// The CHAIN's vertices, per derived id: recomputeNormals and
    /// recomputeExtent loop over the points they are maintained FROM, and
    /// an extent's own array is two vectors however large the mesh is.
    std::vector<double> derivedPoints;
};

size_t
PointCountOf(const UsdAttributeQuery &query)
{
    VtValue value;
    if (!query.IsValid() || !query.Get(&value, UsdTimeCode::EarliestTime()) ||
        !value.IsArrayValued()) {
        return 0;
    }
    return value.GetArraySize();
}

/// The influence slots one vertex of \p revision stores, which is what the
/// skin gather loops over per point. One is the schema's own fallback (see
/// moverGraph.cpp's layout resolution), so an unauthored elementSize costs a
/// per-vertex term of one rather than nothing.
double
ElementSizeOf(const RigExecBakedProgramImpl::GeomRevision &revision)
{
    if (revision.op != RigExecRevisionOp::Skin || !revision.moverPrim) {
        return 1;
    }
    static const TfToken elementSize("rigExec:elementSize");
    int size = 1;
    if (const UsdAttribute attribute =
            revision.moverPrim.GetAttribute(elementSize)) {
        attribute.Get(&size, UsdTimeCode::EarliestTime());
    }
    return size < 1 ? 1 : double(size);
}

GeometrySizes
MeasureGeometry(const RigExecBakedProgramImpl &B)
{
    GeometrySizes sizes;
    sizes.revisionUnits.assign(B.revisionIndex.size(), 0);
    sizes.revisionPoints.assign(B.revisionIndex.size(), 0);
    sizes.revisionElementSize.assign(B.revisionIndex.size(), 1);
    sizes.derivedPoints.assign(B.derivedIndex.size(), 0);
    std::vector<double> &chainPoints = sizes.chainPoints;
    chainPoints.assign(B.chains.size(), 0);
    for (size_t c = 0; c < B.chains.size(); ++c) {
        chainPoints[c] = double(PointCountOf(B.chains[c].baseQuery));
    }
    for (size_t id = 0; id < B.revisionIndex.size(); ++id) {
        const auto &[chain, revision] = B.revisionIndex[id];
        sizes.revisionPoints[id] = chainPoints[size_t(chain)];
        sizes.revisionElementSize[id] =
            ElementSizeOf(B.chains[size_t(chain)].revisions[size_t(revision)]);
        sizes.revisionUnits[id] =
            chainPoints[size_t(chain)] * sizes.revisionElementSize[id];
    }
    for (size_t id = 0; id < B.derivedIndex.size(); ++id) {
        sizes.derivedPoints[id] = chainPoints[size_t(B.derivedIndex[id].first)];
    }
    return sizes;
}

/// How many slots of \p domain \p step declares it writes.
double
WrittenSlots(const RigExecBakedStep &step, RigExecBakedSlotDomain domain)
{
    double count = 0;
    for (const RigExecBakedSlotRange &range : step.writes) {
        if (range.domain == domain) {
            count += double(range.end) - double(range.begin);
        }
    }
    return count;
}

double
StepSize(const RigExecBakedProgramImpl &B, const GeometrySizes &geometry,
         const RigExecBakedStep &step)
{
    const size_t object = size_t(step.object);
    switch (step.kind) {
    case RigExecBakedStepKind::ComposeSubtree: {
        const RigExecBakedComposeGroup &group = B.composeGroups[object];
        return double(group.end - group.begin);
    }
    case RigExecBakedStepKind::Solve: {
        const RigExecBakedProgramImpl::Solver &solver = B.solvers[object];
        if (!solver.controls.empty()) {
            return double(solver.controls.size());
        }
        if (solver.splineCount) {
            return double(solver.splineCount);
        }
        return double(std::max<size_t>(solver.outputs.size(), 1));
    }
    case RigExecBakedStepKind::SolverCommit:
    case RigExecBakedStepKind::Constraint:
    case RigExecBakedStepKind::CommitApply: {
        const RigExecBakedCommit &commit = B.commits[object];
        return double(commit.slots.size() + commit.propagate.size());
    }
    case RigExecBakedStepKind::CommitDelta:
        return double(B.commits[object].slots.size());
    case RigExecBakedStepKind::PropagateChunk:
        return WrittenSlots(step, RigExecBakedSlotDomain::CommitStaging);
    case RigExecBakedStepKind::ProviderMatrix:
    case RigExecBakedStepKind::RevisionFuse:
        return 1;
    case RigExecBakedStepKind::SnapshotFinals:
        return double(B.paths.size());
    case RigExecBakedStepKind::VolumePlacements:
        // One decomposition per volume, and there are never many.
        return WrittenSlots(step, RigExecBakedSlotDomain::WeightFrames) *
               double(std::max<size_t>(step.reads.size(), 1));
    case RigExecBakedStepKind::WeightPacket: {
        // The ELEMENTS the packet carries, which is what every one of the
        // builders costs per unit: a painted table's values, a volume's
        // weighted points, a combine's cardinality. A constant packet is one
        // element and is very nearly free, which is the common case and the
        // reason the fixed term is zero.
        return double(B.weightObjects[object].costElements);
    }
    case RigExecBakedStepKind::InfluenceFold: {
        const auto &[chain, revision] = B.revisionIndex[object];
        return double(B.chains[size_t(chain)]
                          .revisions[size_t(revision)]
                          .influenceSlots.size());
    }
    case RigExecBakedStepKind::RevisionStatic:
        return geometry.revisionPoints[object];
    case RigExecBakedStepKind::RevisionChunk: {
        // The chunk's OWN vertex range, not the revision's: the partition
        // divides the work between the chunks, so sizing each of them by the
        // whole mesh counts the mesh once per chunk. On the biped that made
        // the modelled serial cost 1972us for a program that runs in 531us,
        // and the packing binned every chunk as if it were the largest step
        // in the frame. An unpartitioned revision is one chunk over
        // everything, which is exactly the revision's units.
        const auto &[chain, revision] = B.revisionIndex[object];
        const RigExecBakedProgramImpl::GeomRevision &geom =
            B.chains[size_t(chain)].revisions[size_t(revision)];
        if (geom.chunked && step.part >= 0 &&
            size_t(step.part) < geom.chunks.size()) {
            const RigExecBakedProgramImpl::GeomChunk &chunk =
                geom.chunks[size_t(step.part)];
            return double(chunk.end - chunk.begin) *
                   geometry.revisionElementSize[object];
        }
        return geometry.revisionUnits[object];
    }
    case RigExecBakedStepKind::ChainStatus:
        // The chain's points, not its revisions: the body copies the
        // published array whole (bakedGeometry.cpp), and the MoverFailed
        // sweep over the revisions costs a branch each.
        return geometry.chainPoints[object];
    case RigExecBakedStepKind::Derived:
        return geometry.derivedPoints[object];
    }
    return 1;
}

}  // namespace

void
RigExecBakedAssignStepCosts(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    const GeometrySizes geometry = MeasureGeometry(B);
    for (int index = 0; index < int(B.steps.size()); ++index) {
        RigExecBakedStep &step = B.steps[size_t(index)];
        const StepCostConstants &constants = kStepCosts[size_t(step.kind)];
        step.sizeUnits = StepSize(B, geometry, step);
        step.cost = constants.fixedUs + constants.perUnitUs * step.sizeUnits;
        // Longest path from a source, which is the level the packing groups
        // by. One forward pass, because program order is a topological order.
        int level = 0;
        for (const int pred : step.preds) {
            level = std::max(level, B.steps[size_t(pred)].level);
        }
        step.level = level + 1;
    }
}

// ---------------------------------------------------------------------------
// Clustering.
// ---------------------------------------------------------------------------

double
RigExecBakedScheduleGrainUs(double totalCost)
{
    // Read once: a grain that could move between two frames of one session
    // would make "the program produced two schedules" a question about when
    // the variable was read.
    static const double override = [] {
        const std::string text = TfGetenv("RIGEXEC_BAKED_GRAIN_US", "");
        if (text.empty()) {
            return -1.0;
        }
        const double value = std::strtod(text.c_str(), nullptr);
        return value < 0 ? 0.0 : value;
    }();
    if (override >= 0) {
        return override;
    }
    const double concurrency =
        double(std::max<size_t>(WorkGetConcurrencyLimit(), 1));
    // Enough work per task that the dispatch is noise, few enough tasks that
    // no thread is left holding the only remaining one: four bins per thread
    // is the usual compromise, and the clamp keeps a tiny rig from packing
    // everything into one cluster and a huge one from making ten thousand.
    return std::min(50.0, std::max(5.0, totalCost / (4.0 * concurrency)));
}

namespace {

/// The cluster edges implied by \p clusterOf, with the members collected.
///
/// Recomputed from the step graph after every merge rather than patched:
/// the merge rules are stated on the CURRENT quotient graph (§5.1), and a
/// patched adjacency that had drifted would let a merge fire on a
/// predecessor set that no longer exists -- which is the one way this
/// algorithm could produce a cycle.
void
BuildQuotient(const RigExecBakedProgramImpl &B, RigExecBakedClustering *out)
{
    const size_t count = out->clusters.size();
    for (RigExecBakedCluster &cluster : out->clusters) {
        cluster.members.clear();
        cluster.preds.clear();
        cluster.succs.clear();
        cluster.cost = 0;
        cluster.level = 0;
    }
    for (int index = 0; index < int(B.steps.size()); ++index) {
        const RigExecBakedStep &step = B.steps[size_t(index)];
        const int owner = out->clusterOf[size_t(index)];
        RigExecBakedCluster &cluster = out->clusters[size_t(owner)];
        cluster.members.push_back(index);
        cluster.cost += step.cost;
        cluster.level = std::max(cluster.level, step.level);
        for (const int pred : step.preds) {
            const int from = out->clusterOf[size_t(pred)];
            if (from != owner) {
                cluster.preds.push_back(from);
                out->clusters[size_t(from)].succs.push_back(owner);
            }
        }
    }
    for (size_t c = 0; c < count; ++c) {
        RigExecBakedCluster &cluster = out->clusters[c];
        std::sort(cluster.preds.begin(), cluster.preds.end());
        cluster.preds.erase(
            std::unique(cluster.preds.begin(), cluster.preds.end()),
            cluster.preds.end());
        std::sort(cluster.succs.begin(), cluster.succs.end());
        cluster.succs.erase(
            std::unique(cluster.succs.begin(), cluster.succs.end()),
            cluster.succs.end());
    }
}

/// Drops empty clusters and renumbers what is left by first member, so that
/// the partition a caller sees does not depend on how many merges produced
/// it.
void
Compact(const RigExecBakedProgramImpl &B, RigExecBakedClustering *out)
{
    std::vector<int> order;
    for (size_t c = 0; c < out->clusters.size(); ++c) {
        if (!out->clusters[c].members.empty()) {
            order.push_back(int(c));
        }
    }
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        return out->clusters[size_t(a)].members.front() <
               out->clusters[size_t(b)].members.front();
    });
    std::vector<int> renumbered(out->clusters.size(), -1);
    for (size_t position = 0; position < order.size(); ++position) {
        renumbered[size_t(order[position])] = int(position);
    }
    for (int &owner : out->clusterOf) {
        owner = renumbered[size_t(owner)];
    }
    std::vector<RigExecBakedCluster> kept(order.size());
    out->clusters.swap(kept);
    BuildQuotient(B, out);
}

/// The longest path through the cluster graph by cost -- the wall time the
/// schedule could not beat with any number of threads.
double
CriticalPath(const RigExecBakedClustering &clustering)
{
    std::vector<double> finish(clustering.clusters.size(), 0);
    double longest = 0;
    // Kahn, not a scan over cluster ids: a merge can leave a cluster whose
    // first member precedes its predecessor's, so id order is not a
    // topological order of the quotient graph even though program order is
    // one of the step graph.
    std::vector<int> ready;
    std::vector<int> remaining(clustering.clusters.size(), 0);
    for (size_t c = 0; c < clustering.clusters.size(); ++c) {
        remaining[c] = int(clustering.clusters[c].preds.size());
        if (!remaining[c]) {
            ready.push_back(int(c));
        }
    }
    for (size_t head = 0; head < ready.size(); ++head) {
        const int c = ready[head];
        const RigExecBakedCluster &cluster = clustering.clusters[size_t(c)];
        finish[size_t(c)] += cluster.cost;
        longest = std::max(longest, finish[size_t(c)]);
        for (const int succ : cluster.succs) {
            finish[size_t(succ)] =
                std::max(finish[size_t(succ)], finish[size_t(c)]);
            if (--remaining[size_t(succ)] == 0) {
                ready.push_back(succ);
            }
        }
    }
    return longest;
}

}  // namespace

RigExecBakedClustering
RigExecBakedBuildClusters(const RigExecBakedProgramImpl &B, double grainUs)
{
    RigExecBakedClustering out;
    out.grainUs = grainUs;
    out.clusterOf.assign(B.steps.size(), 0);
    for (const RigExecBakedStep &step : B.steps) {
        out.serialCost += step.cost;
    }
    if (B.steps.empty()) {
        return out;
    }
    if (grainUs <= 0) {
        // One step per cluster: the finest schedule the edges admit, and the
        // one that says most about them. Neither fusion nor the absorb rule
        // runs, because both exist to make clusters coarser and this grain
        // asked for the opposite.
        out.clusters.resize(B.steps.size());
        std::iota(out.clusterOf.begin(), out.clusterOf.end(), 0);
        BuildQuotient(B, &out);
        out.criticalPathCost = CriticalPath(out);
        return out;
    }

    // ---- level packing ------------------------------------------------------
    //
    // With longest-path levels no edge joins two steps of ONE level, so any
    // grouping within a level is acyclic however the bins fall. That is the
    // whole correctness argument for the packing, and it is why the levels
    // are longest-path and not depth-first depths.
    int levels = 0;
    for (const RigExecBakedStep &step : B.steps) {
        levels = std::max(levels, step.level);
    }
    std::vector<std::vector<int>> byLevel(size_t(levels) + 1);
    for (int index = 0; index < int(B.steps.size()); ++index) {
        byLevel[size_t(B.steps[size_t(index)].level)].push_back(index);
    }
    const int concurrency = std::max(1, int(WorkGetConcurrencyLimit()));
    int next = 0;
    for (const std::vector<int> &level : byLevel) {
        if (level.empty()) {
            continue;
        }
        double total = 0;
        for (const int index : level) {
            total += B.steps[size_t(index)].cost;
        }
        const int bins = std::max(
            1, std::min(concurrency, int(std::ceil(total / grainUs))));
        // Contiguous bins in PROGRAM order, cut where the running cost
        // crosses each bin's share. Contiguity is not cosmetic: it keeps a
        // cluster's members adjacent in the program, which is what makes the
        // slots they touch adjacent too.
        const double share = total / double(bins);
        const int first = next;
        next += bins;
        double running = 0;
        int bin = 0;
        for (const int index : level) {
            out.clusterOf[size_t(index)] = first + bin;
            running += B.steps[size_t(index)].cost;
            while (bin + 1 < bins && running >= share * double(bin + 1)) {
                ++bin;
            }
        }
    }
    out.clusters.resize(size_t(next));
    BuildQuotient(B, &out);
    Compact(B, &out);

    // ---- chain fusion -------------------------------------------------------
    //
    // Contract (A, B) when B is A's only successor and A is B's only
    // predecessor: nothing else can run while A holds B up, so the edge buys
    // no parallelism and costs a dispatch. Contracting such an edge cannot
    // close a cycle -- every path out of A starts with A -> B, so there is no
    // second A ~> B path to close one with.
    bool merged = true;
    while (merged) {
        merged = false;
        for (size_t a = 0; a < out.clusters.size() && !merged; ++a) {
            const RigExecBakedCluster &from = out.clusters[a];
            if (from.members.empty() || from.succs.size() != 1) {
                continue;
            }
            const int b = from.succs.front();
            if (out.clusters[size_t(b)].preds.size() != 1) {
                continue;
            }
            for (int &owner : out.clusterOf) {
                if (owner == b) {
                    owner = int(a);
                }
            }
            BuildQuotient(B, &out);
            merged = true;
        }
    }

    // ---- absorb -------------------------------------------------------------
    //
    // A cluster too small to be worth a task joins its predecessor when it
    // has exactly one. The invariant is evaluated on the CURRENT quotient
    // graph after every merge, which is what makes it sound: with
    // pred(s) = {C}, a second C ~> s path would have to pass through another
    // predecessor of s, and s has none.
    merged = true;
    while (merged) {
        merged = false;
        for (size_t s = 0; s < out.clusters.size() && !merged; ++s) {
            const RigExecBakedCluster &cluster = out.clusters[s];
            if (cluster.members.empty() ||
                cluster.cost >= 2.0 * kSpawnCostUs ||
                cluster.preds.size() != 1) {
                continue;
            }
            const int owner = cluster.preds.front();
            for (int &entry : out.clusterOf) {
                if (entry == int(s)) {
                    entry = owner;
                }
            }
            BuildQuotient(B, &out);
            merged = true;
        }
    }
    Compact(B, &out);
    out.criticalPathCost = CriticalPath(out);
    return out;
}

// ---------------------------------------------------------------------------
// Edges.
// ---------------------------------------------------------------------------

namespace {

/// A half-open run of one domain's slots, and the step it belongs to.
struct SlotInterval {
    uint32_t begin = 0, end = 0;
    int step = 0;
};

/// Removes [\p begin, \p end) from every interval of \p intervals, splitting
/// one that straddles it. This is what "update lastWriter for the range"
/// means when the bookkeeping is per interval rather than per slot: the
/// parts of an older writer's range the new one did not cover are still that
/// writer's.
void
SubtractRange(std::vector<SlotInterval> *intervals, uint32_t begin,
              uint32_t end)
{
    std::vector<SlotInterval> kept;
    kept.reserve(intervals->size() + 1);
    for (const SlotInterval &interval : *intervals) {
        if (interval.end <= begin || end <= interval.begin) {
            kept.push_back(interval);
            continue;
        }
        if (interval.begin < begin) {
            kept.push_back({interval.begin, begin, interval.step});
        }
        if (end < interval.end) {
            kept.push_back({end, interval.end, interval.step});
        }
    }
    intervals->swap(kept);
}

void
SortRanges(std::vector<RigExecBakedSlotRange> *ranges)
{
    std::sort(ranges->begin(), ranges->end());
    ranges->erase(std::unique(ranges->begin(), ranges->end()), ranges->end());
}

}  // namespace

namespace {
std::string StepLabel(const RigExecBakedProgramImpl &B,
                      const RigExecBakedStep &step);
}  // namespace

void
RigExecBakedBuildStepEdges(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    if (B.phasedReads) {
        // The run's phased-read store is ONE container, and a step's records
        // reach it through a fold the executor performs. Slot-per-step
        // declarations order each record against the steps that read it, but
        // they leave two RECORDERS free to run at once -- and two folds into
        // one store at once is a race whatever the slots say. So on a rig
        // that can look a record up, every recorder declares the whole store
        // up to its own point, which makes the recorders a chain in program
        // order and leaves the readers where they were. Declared writes are
        // an upper bound, so widening one is always sound; it is only ever
        // paid for by a rig that declares a read phase, and nothing else
        // about the schedule changes.
        for (int index = 0; index < int(B.steps.size()); ++index) {
            for (RigExecBakedSlotRange &range :
                     B.steps[size_t(index)].writes) {
                if (range.domain == RigExecBakedSlotDomain::Snapshots) {
                    range.begin = 0;
                    range.end = uint32_t(index) + 1;
                }
            }
        }
    }
    std::array<std::vector<SlotInterval>, RigExecBakedSlotDomainCount>
        writers, readers;
    for (int index = 0; index < int(B.steps.size()); ++index) {
        RigExecBakedStep &step = B.steps[size_t(index)];
        SortRanges(&step.reads);
        SortRanges(&step.writes);
        step.preds.clear();
        const auto overlapping = [&](const std::vector<SlotInterval> &table,
                                     const RigExecBakedSlotRange &range) {
            for (const SlotInterval &interval : table) {
                if (interval.begin < range.end && range.begin < interval.end &&
                    interval.step != index) {
                    step.preds.push_back(interval.step);
                }
            }
        };
        for (const RigExecBakedSlotRange &range : step.reads) {
            overlapping(writers[size_t(range.domain)], range);
        }
        // Registered before the write pass, so that a read-modify-write step
        // -- every constraint is one -- does not raise a write-after-read
        // edge against itself; `overlapping` skips its own index.
        for (const RigExecBakedSlotRange &range : step.reads) {
            readers[size_t(range.domain)].push_back(
                {range.begin, range.end, index});
        }
        for (const RigExecBakedSlotRange &range : step.writes) {
            // Write after write, always: two writers of one slot are ordered
            // by the program, and the later one CARRIES the earlier one's
            // version where it does not write (§3.1), which makes this a
            // true data edge and not only an ordering one.
            overlapping(writers[size_t(range.domain)], range);
            // Write after read, only where the two would share storage.
            // They do not in the versioned pose domains: a writer there
            // writes storage of its own, so a reader of an earlier version
            // and a later writer touch different memory and may run in
            // either order or at once. Dropping the edge is not a
            // scheduling nicety -- it is what keeps a dragged constraint's
            // cone from reaching every commit that happens to revise a slot
            // it read.
            if (!RigExecBakedIsVersionedDomain(range.domain)) {
                overlapping(readers[size_t(range.domain)], range);
            }
        }
        for (const RigExecBakedSlotRange &range : step.writes) {
            std::vector<SlotInterval> &writerTable =
                writers[size_t(range.domain)];
            SubtractRange(&writerTable, range.begin, range.end);
            SubtractRange(&readers[size_t(range.domain)], range.begin,
                          range.end);
            writerTable.push_back({range.begin, range.end, index});
        }
        std::sort(step.preds.begin(), step.preds.end());
        step.preds.erase(std::unique(step.preds.begin(), step.preds.end()),
                         step.preds.end());
        // The invariant the whole design rests on: an edge only ever leaves a
        // step the sweep has already passed, so a cluster running its members
        // in increasing program index is always in topological order.
        for (const int pred : step.preds) {
            TF_VERIFY(pred < index,
                      "rigExec: baked step %d depends on later step %d",
                      index, pred);
        }
    }
    for (RigExecBakedStep &step : B.steps) {
        step.succs.clear();
        step.label = std::string(RigExecBakedStepKindName(step.kind)) + " " +
                     StepLabel(B, step);
    }
    for (int index = 0; index < int(B.steps.size()); ++index) {
        for (const int pred : B.steps[size_t(index)].preds) {
            B.steps[size_t(pred)].succs.push_back(index);
        }
    }
}

void
RigExecBakedBuildSchedule(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    // Re-run rather than assumed done: Build calls the sweep once over the
    // pose half alone, so that the partition rule (§6) can read the pose
    // steps' levels before it decides where to cut, and the geometry steps
    // it then appends need edges of their own.
    RigExecBakedBuildStepEdges(&B);

    // The schedule the parallel executor runs, chosen once here so that a
    // frame costs it nothing: the cost model, the packing, and one padded
    // counter per cluster.
    RigExecBakedAssignStepCosts(&B);
    B.clustering = RigExecBakedBuildClusters(
        B, RigExecBakedScheduleGrainUs(
               std::accumulate(B.steps.begin(), B.steps.end(), 0.0,
                               [](double sum, const RigExecBakedStep &step) {
                                   return sum + step.cost;
                               })));
    for (int index = 0; index < int(B.steps.size()); ++index) {
        B.steps[size_t(index)].cluster = B.clustering.clusterOf[size_t(index)];
    }
    B.clusterCounters = std::make_unique<RigExecBakedClusterCounter[]>(
        std::max<size_t>(B.clustering.clusters.size(), 1));
    RigExecBakedDeclareInputDependencies(&B);
    RigExecBakedBuildCones(&B);
}

// ---------------------------------------------------------------------------
// Cone re-execution (§7).
//
// What a frame may skip, and why skipping it is not an approximation. ONE
// closure decides it, computed once at Build:
//
//   cone[c]    -- run c and you have to run all of this
//
// There used to be a second, the restore closure: "run c and all of THIS had
// to have run first, because c reads a slot version the end of a run does not
// hold". Versioned pose storage (§3.1) retired it. Every writer writes its
// own entry, so the version a clean reader wants is exactly where its writer
// left it however often the SLOT was revised afterwards, and nothing ever has
// to be re-run to put a value back.
//
// One rule decides what starts the closure: a SOURCE -- the avar table, a
// chain's base points, a skin revision's static packet, the property-chain
// results -- always runs, and its output is compared with the last run's by
// VALUE. Never "the time changed", never "an override stands": those two
// predicates each miss a case that reaches the graph anyway (an override on
// a routed prim authors no flag, a released drag leaves the table disturbed,
// a cleared layout cache changes a packet), and a value comparison misses
// none of them.
// ---------------------------------------------------------------------------

namespace {

/// The clusters of \p program, in an order in which every cluster follows
/// its predecessors.
///
/// Cluster ids come out of the level packing, which numbers bins and not
/// dependencies -- cluster 6 can perfectly well have cluster 10 among its
/// predecessors -- so a closure over the cluster graph needs its own order.
std::vector<int>
ClusterTopologicalOrder(const RigExecBakedClustering &clustering)
{
    const size_t count = clustering.clusters.size();
    std::vector<int> remaining(count, 0), order;
    order.reserve(count);
    for (size_t c = 0; c < count; ++c) {
        remaining[c] = int(clustering.clusters[c].preds.size());
    }
    std::vector<int> ready;
    for (size_t c = 0; c < count; ++c) {
        if (remaining[c] == 0) ready.push_back(int(c));
    }
    while (!ready.empty()) {
        const int cluster = ready.back();
        ready.pop_back();
        order.push_back(cluster);
        for (const int succ : clustering.clusters[size_t(cluster)].succs) {
            if (--remaining[size_t(succ)] == 0) {
                ready.push_back(succ);
            }
        }
    }
    TF_VERIFY(order.size() == count,
              "rigExec: the baked cluster graph has a cycle (%zu of %zu "
              "clusters ordered)", order.size(), count);
    return order;
}

}  // namespace

void
RigExecBakedBuildCones(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    RigExecBakedCones &cones = B.cones;
    const size_t count = B.clustering.clusters.size();
    cones = RigExecBakedCones();
    B.closed.Resize(count);
    if (count == 0) {
        return;
    }
    cones.cone.resize(count);
    cones.always.Resize(count);
    cones.poseClusters.Resize(count);
    for (size_t c = 0; c < count; ++c) {
        cones.cone[c].Resize(count);
    }

    // ---- which steps read outside the graph ---------------------------------
    //
    // A step whose every read is a source slot can be run BEFORE the dirty
    // set is computed -- it has no predecessor to wait for -- and that is
    // what "sources always run" comes to in an executor. A step that reads
    // outside the graph and does have predecessors cannot be, so its cluster
    // is simply dirty every run; its own value comparison is what keeps the
    // counters saying what the dynamic path says.
    // Which weight objects are built from the stage alone. A painted or
    // driven weight reads no slot, so its step is a source like any other --
    // it runs in the source pass, ahead of the dirty set, and a consumer of
    // its packet can still be a source itself. A VOLUME reads the frame its
    // provider was posed into, so it is a step with predecessors and its
    // consumers are not sources either. Filled in program order, which is
    // dependency order for the weight steps and puts every one of them ahead
    // of the RevisionStatic that reads it.
    std::vector<char> sourcePacket(B.weightObjects.size(), 0);
    for (RigExecBakedStep &step : B.steps) {
        step.isSource = false;
        step.externalReads = false;
        if (step.kind == RigExecBakedStepKind::WeightPacket) {
            bool pure = true;
            for (const RigExecBakedSlotRange &range : step.reads) {
                pure = pure &&
                       range.domain == RigExecBakedSlotDomain::WeightPacket &&
                       [&] {
                           for (uint32_t w = range.begin; w < range.end; ++w) {
                               if (!sourcePacket[w]) return false;
                           }
                           return true;
                       }();
            }
            sourcePacket[size_t(step.object)] = pure ? 1 : 0;
            step.isSource = pure;
            step.externalReads = !pure;
        } else if (step.kind == RigExecBakedStepKind::RevisionStatic) {
            bool pure = true;
            for (const RigExecBakedSlotRange &range : step.reads) {
                if (range.domain == RigExecBakedSlotDomain::WeightPacket) {
                    for (uint32_t w = range.begin; w < range.end && pure; ++w) {
                        pure = sourcePacket[w] != 0;
                    }
                    continue;
                }
                pure = pure &&
                       (range.domain == RigExecBakedSlotDomain::Avars ||
                        range.domain ==
                            RigExecBakedSlotDomain::PropertyResult ||
                        range.domain == RigExecBakedSlotDomain::ChainBase);
            }
            step.isSource = pure;
            step.externalReads = !pure;
        } else if (step.kind == RigExecBakedStepKind::Constraint) {
            // A constraint's envelope object is resolved from the stage by
            // the oracle, at the constraint's own point in the walk, and no
            // slot names it. That is a read outside the program, so the
            // cluster is dirty every run and the resolve's own answer is
            // what decides the rest.
            const RigExecBakedProgramImpl::WalkStep &walk =
                B.walkSteps[size_t(step.object)];
            step.externalReads =
                !walk.solverBatch && walk.index >= 0 &&
                !B.constraints[size_t(walk.index)].weightObject.IsEmpty();
        } else if (step.kind == RigExecBakedStepKind::Derived) {
            // A derived target assembles its own packet against its own
            // inputs, and does it after the chain it maintains has published
            // -- so it can be neither a source nor a pure function of slots.
            step.externalReads = true;
        }
        if (step.externalReads) {
            cones.always.Set(step.cluster);
        }
        if (!RigExecBakedIsGeometryStep(step.kind)) {
            cones.poseClusters.Set(step.cluster);
        }
        if (step.varyingInputs || step.resolvedInputReads) {
            cones.varyingSteps.push_back(int(&step - B.steps.data()));
        }
        if (!step.overrideInputs.empty()) {
            cones.overrideSteps.push_back(int(&step - B.steps.data()));
        }
    }

    // ---- what a changed source makes dirty ----------------------------------
    cones.avarCluster.assign(B.paths.size(), -1);
    cones.chainBaseClusters.assign(B.chains.size(), {});
    cones.solverPointsClusters.assign(B.solvers.size(), {});
    cones.revisionClusters.assign(B.revisionIndex.size(), {});
    cones.revisionStaticCluster.assign(B.revisionIndex.size(), -1);
    for (const RigExecBakedStep &step : B.steps) {
        if (step.kind == RigExecBakedStepKind::ComposeSubtree) {
            const RigExecBakedComposeGroup &group =
                B.composeGroups[size_t(step.object)];
            for (int slot = group.begin; slot < group.end; ++slot) {
                cones.avarCluster[size_t(slot)] = step.cluster;
            }
        }
        for (const RigExecBakedSlotRange &range : step.reads) {
            if (range.domain == RigExecBakedSlotDomain::ChainBase) {
                for (uint32_t c = range.begin;
                     c < range.end && c < cones.chainBaseClusters.size();
                     ++c) {
                    cones.chainBaseClusters[c].push_back(step.cluster);
                }
            } else if (range.domain ==
                       RigExecBakedSlotDomain::SolverPoints) {
                for (uint32_t si = range.begin;
                     si < range.end && si < cones.solverPointsClusters.size();
                     ++si) {
                    cones.solverPointsClusters[si].push_back(step.cluster);
                }
            }
        }
        switch (step.kind) {
        case RigExecBakedStepKind::RevisionStatic:
            cones.revisionStaticCluster[size_t(step.object)] = step.cluster;
            [[fallthrough]];
        case RigExecBakedStepKind::InfluenceFold:
        case RigExecBakedStepKind::RevisionChunk:
        case RigExecBakedStepKind::RevisionFuse:
            cones.revisionClusters[size_t(step.object)].push_back(
                step.cluster);
            break;
        default:
            break;
        }
    }
    // Which constraint steps a native source's stage transform reaches, and
    // which one each geometry-domain delta base reaches.
    cones.nativeSourceClusters.assign(B.nativeSources.size(), {});
    cones.deltaBaseClusters.assign(B.deltaBasePaths.size(), {});
    cones.constraintArrayClusters.assign(B.constraintArrays.size(), {});
    for (const RigExecBakedStep &step : B.steps) {
        if (step.kind != RigExecBakedStepKind::Constraint) {
            continue;
        }
        const RigExecBakedProgramImpl::WalkStep &walk =
            B.walkSteps[size_t(step.object)];
        if (walk.solverBatch || walk.index < 0) {
            continue;
        }
        const RigExecBakedProgramImpl::Constraint &constraint =
            B.constraints[size_t(walk.index)];
        for (const int native : constraint.sourceNatives) {
            if (native >= 0) {
                cones.nativeSourceClusters[size_t(native)].push_back(
                    step.cluster);
            }
        }
        if (constraint.worldUpNative >= 0) {
            cones.nativeSourceClusters[size_t(constraint.worldUpNative)]
                .push_back(step.cluster);
        }
        if (constraint.deltaBase >= 0) {
            cones.deltaBaseClusters[size_t(constraint.deltaBase)].push_back(
                step.cluster);
        }
        if (constraint.arrays >= 0) {
            cones.constraintArrayClusters[size_t(constraint.arrays)]
                .push_back(step.cluster);
        }
    }
    for (std::vector<int> &clusters : cones.nativeSourceClusters) {
        std::sort(clusters.begin(), clusters.end());
        clusters.erase(std::unique(clusters.begin(), clusters.end()),
                       clusters.end());
    }
    for (std::vector<int> &clusters : cones.chainBaseClusters) {
        std::sort(clusters.begin(), clusters.end());
        clusters.erase(std::unique(clusters.begin(), clusters.end()),
                       clusters.end());
    }
    for (std::vector<int> &clusters : cones.solverPointsClusters) {
        std::sort(clusters.begin(), clusters.end());
        clusters.erase(std::unique(clusters.begin(), clusters.end()),
                       clusters.end());
    }
    for (std::vector<int> &clusters : cones.revisionClusters) {
        std::sort(clusters.begin(), clusters.end());
        clusters.erase(std::unique(clusters.begin(), clusters.end()),
                       clusters.end());
    }

    // ---- the forward closure ------------------------------------------------
    const std::vector<int> order = ClusterTopologicalOrder(B.clustering);
    for (size_t k = order.size(); k-- > 0;) {
        const int cluster = order[k];
        RigExecBakedClusterSet &cone = cones.cone[size_t(cluster)];
        cone.Set(cluster);
        for (const int succ : B.clustering.clusters[size_t(cluster)].succs) {
            cone.Union(cones.cone[size_t(succ)]);
        }
    }
}

void
RigExecBakedComputeClosure(RigExecBakedProgramImpl *program, UsdTimeCode time,
                           bool force)
{
    RigExecBakedProgramImpl &B = *program;
    RigExecBakedCones &cones = B.cones;
    const size_t count = B.clustering.clusters.size();
    B.closed.Resize(count);
    if (count == 0) {
        return;
    }
    RigExecBakedClusterSet dirty;
    dirty.Resize(count);

    // What makes a run trust NOTHING it holds: the caller forcing one (the
    // cone verifier's second pass); a program stamp that moved, which is the
    // evaluator saying a notice changed something the index does not name;
    // and a rig that can LOOK UP a phased read, because the store the
    // records land in is emptied at the head of every run and a skipped
    // recorder leaves a hole in it rather than last run's answer. The first
    // run of a program is NOT one of them -- it has its own dirty set below.
    bool full = force || B.phasedReads ||
                B.programStamp != B.lastProgramStamp;
    if (!full && B.everRan && B.hasPropertyChains &&
        B.propertyResults != B.lastPropertyResults) {
        // A chain's result reaches a step through the generation's resolved
        // inputs, which no slot names: every step that reads one has to run.
        full = true;
    }
    if (full) {
        dirty.SetAll(count);
    } else if (!B.everRan) {
        // The FIRST run of this program -- §7's other dirty set [S28].
        //
        // There is nothing to compare against, so every step that could have
        // moved for any reason is dirty: the whole pose half, the steps that
        // read outside the graph, and the steps a time or a standing
        // override reaches. The one thing this run DOES know is which
        // revisions came across a rebuild with their answer intact:
        // AdoptGeometryStateFrom carries the cached result of every revision
        // whose position in its chain did not move, and re-deforming those
        // would spend a whole skin on an edit the dynamic path's VdfNetwork
        // reconnects without re-executing a node.
        //
        // Nothing downstream of them is at risk of running on half a state:
        // a revision's steps are dirtied together or not at all, because
        // every source whose cone reaches one of them -- its own
        // RevisionStatic, its chain's base, the matrices of its influences
        // -- reaches the rest of the revision as well.
        dirty.Union(cones.poseClusters);
        dirty.Union(cones.always);
        for (const int index : cones.varyingSteps) {
            dirty.Set(B.steps[size_t(index)].cluster);
        }
        for (const int index : cones.overrideSteps) {
            dirty.Set(B.steps[size_t(index)].cluster);
        }
        for (size_t r = 0; r < B.revisionIndex.size(); ++r) {
            const auto &[chainIndex, revisionIndex] = B.revisionIndex[r];
            const RigExecBakedProgramImpl::GeomRevision &revision =
                B.chains[size_t(chainIndex)]
                    .revisions[size_t(revisionIndex)];
            // The mover's own half of the same sentence the chain guard
            // below makes: `ran` came across the rebuild on the revision's
            // IDENTITY, so an edit that moved a mover's parameters while it
            // was at it leaves a revision saying it holds an answer to a
            // packet the stage no longer has. Its RevisionStatic step has
            // already compared the two this run -- it is a source, so it
            // runs before this set is computed -- and `staticDirty` is that
            // comparison. Read the same way the steady-state branch reads
            // it, and dirtied the same way: the static cluster, whose cone
            // carries the rest of the revision.
            if (revision.staticDirty) {
                dirty.Set(cones.revisionStaticCluster[r]);
            }
            if (revision.ran) {
                continue;
            }
            for (const int cluster : cones.revisionClusters[r]) {
                dirty.Set(cluster);
            }
        }
        // And the chains whose AUTHORED points moved with the same edit, or
        // which do not read at this time at all. `ran` says a revision holds
        // an answer; it does not say the points that answer was computed
        // from are still the ones the stage has. Without this a rebuild that
        // also repainted a mesh would publish the old deformation on any
        // chain no joint drives.
        for (size_t c = 0; c < B.chains.size(); ++c) {
            const RigExecBakedProgramImpl::GeomChain &chain = B.chains[c];
            if (chain.haveBase && !chain.baseDirty) {
                continue;
            }
            for (const int cluster : cones.chainBaseClusters[c]) {
                dirty.Set(cluster);
            }
        }
    } else {
        dirty.Union(cones.always);
        // Avars, per provider: eleven doubles compared, not a flag consulted.
        for (size_t i = 0; i < B.paths.size(); ++i) {
            const size_t base = i * 11;
            bool moved = false;
            for (size_t k = 0; k < 11 && !moved; ++k) {
                moved = B.avars[base + k] != B.lastAvars[base + k];
            }
            if (moved) {
                dirty.Set(cones.avarCluster[i]);
            }
        }
        // The transforms the prologue read off the stage for the plain
        // Xformables a constraint targets: sixteen numbers per slot compared
        // by VALUE, exactly as the avars above are, because "the time moved"
        // is never the predicate for a source. The compose group covering
        // the slot is what declares a write of its frame, so its cluster is
        // the one every reader of that frame hangs off.
        for (size_t k = 0; k < B.xformSlots.size(); ++k) {
            if (B.xformBase[k] != B.lastXformBase[k]) {
                dirty.Set(cones.avarCluster[size_t(B.xformSlots[k])]);
            }
        }
        // The provider ladder, where the prologue recomposed it and found a
        // value moved: a rest, a default space or an authored posed:space
        // that moved recomposes its provider, and the compose group is what
        // declares the write every reader of that frame -- and of the
        // rest -> pose matrices beside it -- hangs off.
        for (const int slot : B.ladderMovedSlots) {
            dirty.Set(cones.avarCluster[size_t(slot)]);
        }
        // A constraint's own authored tables, which the prologue re-reads
        // at the frame's time. Compared by value, values and diagnostic
        // together, because a cardinality line that changed is a published
        // difference as much as a weight that changed.
        for (size_t k = 0; k < B.constraintArrays.size(); ++k) {
            const RigExecBakedProgramImpl::ConstraintArrays &arrays =
                B.constraintArrays[k];
            if (arrays.ok == arrays.lastOk &&
                arrays.weights == arrays.lastWeights &&
                arrays.translationOffsets == arrays.lastTranslationOffsets &&
                arrays.rotationOffsets == arrays.lastRotationOffsets &&
                arrays.diagnostics == arrays.lastDiagnostics &&
                // The pole half is read at a different point in the walk and
                // owns its own diagnostic, but it is the same kind of thing:
                // a table the prologue re-read, so a table this run must be
                // compared by value against. Leaving it out let an animated
                // inputs:poleVectorWeights hold a stale solve.
                arrays.poleOk == arrays.lastPoleOk &&
                arrays.poleWeights == arrays.lastPoleWeights &&
                arrays.poleDiagnostics == arrays.lastPoleDiagnostics) {
                continue;
            }
            for (const int cluster : cones.constraintArrayClusters[k]) {
                dirty.Set(cluster);
            }
        }
        // And the transform each geometry-domain constraint measures its
        // delta against, which is its target prim's own authored one.
        for (size_t k = 0; k < B.deltaBasePaths.size(); ++k) {
            if (B.deltaBaseOk[k] != B.lastDeltaBaseOk[k] ||
                B.deltaBaseMatrix[k] != B.lastDeltaBaseMatrix[k]) {
                for (const int cluster : cones.deltaBaseClusters[k]) {
                    dirty.Set(cluster);
                }
            }
        }
        // And the transforms of the plain Xformables a constraint names as
        // a SOURCE, compared the same way -- frame and read-or-not together,
        // because a source that stopped resolving has moved as surely as one
        // that moved.
        for (size_t k = 0; k < B.nativeSources.size(); ++k) {
            if (B.nativeFrameOk[k] != B.lastNativeFrameOk[k] ||
                B.nativeFrames[k].points != B.lastNativeFrames[k].points) {
                for (const int cluster : cones.nativeSourceClusters[k]) {
                    dirty.Set(cluster);
                }
            }
        }
        // Each chain's authored base, and whether it read at all.
        for (size_t c = 0; c < B.chains.size(); ++c) {
            const RigExecBakedProgramImpl::GeomChain &chain = B.chains[c];
            if (!chain.baseDirty &&
                chain.haveBase == (B.lastHaveBase[c] != 0)) {
                continue;
            }
            for (const int cluster : cones.chainBaseClusters[c]) {
                dirty.Set(cluster);
            }
        }
        // Each ribbon's driver curve, which the prologue has already read
        // and compared this run. Scene data, not rig state: nothing in the
        // program writes it, so nothing else can say that it moved.
        for (size_t si = 0; si < B.solvers.size(); ++si) {
            if (!B.solvers[si].ribbonPointsDirty) {
                continue;
            }
            for (const int cluster : cones.solverPointsClusters[si]) {
                dirty.Set(cluster);
            }
        }
        // Each skin revision's static packet, which its own source step has
        // already assembled and compared this run.
        for (size_t r = 0; r < B.revisionIndex.size(); ++r) {
            const auto &[chainIndex, revisionIndex] = B.revisionIndex[r];
            const RigExecBakedProgramImpl::GeomRevision &revision =
                B.chains[size_t(chainIndex)]
                    .revisions[size_t(revisionIndex)];
            if (revision.staticDirty) {
                dirty.Set(cones.revisionStaticCluster[r]);
            }
            if (!revision.ran) {
                // Geometry state that a rebuild did not carry over: the
                // revision holds no answer to re-publish, so it is not a
                // candidate for skipping whatever its packet says.
                for (const int cluster : cones.revisionClusters[r]) {
                    dirty.Set(cluster);
                }
            }
        }
        // The inputs a Solve and a Constraint read off the stage per frame.
        // Two tests, and both are about the VALUE that reaches the step: an
        // input that is a function of time can only have moved if the time
        // did, and an input an override stands on moved when the override
        // was placed and again when it was lifted.
        if (time != B.lastTime) {
            for (const int index : cones.varyingSteps) {
                dirty.Set(B.steps[size_t(index)].cluster);
            }
        }
        for (const int index : cones.overrideSteps) {
            const RigExecBakedStep &step = B.steps[size_t(index)];
            for (const int input : step.overrideInputs) {
                if (B.overridden[size_t(input)] ||
                    B.lastOverridden[size_t(input)]) {
                    dirty.Set(step.cluster);
                    break;
                }
            }
        }
    }

    B.closed.Clear();
    for (size_t c = 0; c < count; ++c) {
        if (dirty.Test(int(c))) {
            B.closed.Union(cones.cone[c]);
        }
    }
    // And that is the whole closure. Nothing is added to it to RESTORE a
    // version a later writer took over, because no later writer takes one
    // over (§3.1): what a skipped step left in its own storage is what the
    // readers bound to it are still entitled to read.

    // What the next run compares against. Updated here, once, whether or not
    // this run skipped anything: the comparison is always with the values the
    // last run SAW, and a forced run saw them too.
    B.lastAvars = B.avars;
    B.lastXformBase = B.xformBase;
    B.lastNativeFrames = B.nativeFrames;
    B.lastNativeFrameOk = B.nativeFrameOk;
    B.lastDeltaBaseMatrix = B.deltaBaseMatrix;
    B.lastDeltaBaseOk = B.deltaBaseOk;
    for (RigExecBakedProgramImpl::ConstraintArrays &arrays :
             B.constraintArrays) {
        arrays.lastOk = arrays.ok;
        arrays.lastWeights = arrays.weights;
        arrays.lastTranslationOffsets = arrays.translationOffsets;
        arrays.lastRotationOffsets = arrays.rotationOffsets;
        arrays.lastDiagnostics = arrays.diagnostics;
        arrays.lastPoleOk = arrays.poleOk;
        arrays.lastPoleWeights = arrays.poleWeights;
        arrays.lastPoleDiagnostics = arrays.poleDiagnostics;
    }
    B.lastOverridden = B.overridden;
    B.lastPropertyResults = B.propertyResults;
    B.lastHaveBase.resize(B.chains.size());
    for (size_t c = 0; c < B.chains.size(); ++c) {
        B.lastHaveBase[c] = B.chains[c].haveBase ? 1 : 0;
    }
    B.lastTime = time;
    B.lastProgramStamp = B.programStamp;
    B.everRan = true;
    B.lastClosedClusters = B.closed.Count();
}

// ---------------------------------------------------------------------------
// The executors.
// ---------------------------------------------------------------------------

namespace {

/// Runs one step's body and checks what it produced against what it declared.
void
RunStepBody(RigExecBakedProgramImpl *B, RigExecBakedStep *step,
            UsdTimeCode time)
{
    // A run's output is cleared HERE rather than in the body, so that the
    // clearing is the executor's promise and not something fifteen bodies
    // each have to remember.
    step->BeginRun();
    if (step->kind == RigExecBakedStepKind::WeightPacket ||
        step->kind == RigExecBakedStepKind::VolumePlacements) {
        // Neither half's: a weight object is bound by movers and by
        // constraints, so its packet is built between the two rather than
        // inside either (bakedWeights.cpp).
        RigExecBakedRunWeightStep(B, step, time);
    } else if (RigExecBakedIsGeometryStep(step->kind)) {
        RigExecBakedRunGeometryStep(B, step, time);
    } else {
        RigExecBakedRunPoseStep(B, step, time);
    }
    // Bodies size their own lists at Build; a body that grew past what it
    // declared is a step allocating inside the region, which is the thing
    // the declaration exists to prevent.
    TF_VERIFY(step->diagnostics.size() <= step->maxDiagnostics,
              "rigExec: baked step emitted %zu diagnostics, at most %zu "
              "declared", step->diagnostics.size(), step->maxDiagnostics);
}

/// The same clock RigExecProfiler::NowUs reads, in NANOseconds.
///
/// The profiler's microseconds are the right unit for a trace and the wrong
/// one for a cost model: most steps of a biped frame are under a microsecond,
/// and a table fitted from integer microseconds would call all of them free.
uint64_t
NowNs()
{
    return uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::steady_clock::now().time_since_epoch())
                        .count());
}

bool
RunStepsSerial(RigExecBakedProgramImpl *program, UsdTimeCode time)
{
    RigExecBakedProgramImpl &B = *program;
    const bool timing = B.profiler && B.profiler->IsEnabled();
    // Nothing here looks at a cluster, so the run report must not pretend
    // this frame measured any.
    B.clustering.lastRunTimed = false;
    // The calibration and the step timing measure a step BODY on a finer
    // clock, into that step's own accumulator: no lock, no shared counter,
    // and nothing that survives the frame but a sum. Both stop for the cone
    // verifier's second pass, which is not this frame.
    const bool calibrating = (RigExecBakedScheduleCalibrationRequested() ||
                              RigExecBakedStepTimingRequested()) &&
                             !B.measurementSuspended;
    uint64_t mark = timing ? RigExecProfiler::NowUs() : 0;
    for (RigExecBakedStep &step : B.steps) {
        // A source already ran, before the dirty set that decided the rest
        // could be computed; a step outside the closed set is this run's
        // skip, and its slots, its lines and its structural counters stand.
        if (step.isSource || !B.closed.Test(step.cluster)) {
            continue;
        }
        // A PAIR around the body, the same interval the parallel executor
        // takes -- not the rolling boundary this loop uses for the trace.
        // The two modes' step times are read against each other (it is the
        // whole reason a step time is interesting), so they have to measure
        // the same thing: a boundary that rolled from the last executed
        // step would charge serial for the snapshot merge below and for the
        // scan over every step skipped since, and make the comparison
        // flatter than the frame is. Affordable because nothing reads a
        // clock here unless a calibration or a step timing asked.
        const uint64_t beganNs = calibrating ? NowNs() : 0;
        RunStepBody(&B, &step, time);
        if (calibrating) {
            step.measuredUs += double(NowNs() - beganNs) / 1000.0;
            ++step.measuredRuns;
        }
        if (timing) {
            // ONE clock read per step boundary, not two per step: a biped's
            // graph is several hundred steps and a thousand reads of a
            // vDSO clock is a measurable part of the frame being measured.
            // What each TRACE interval then covers is the step plus the few
            // instructions of bookkeeping below it, which is where the time
            // went -- the trace is a picture of the frame and wants the
            // whole of it. The step times above are the other question,
            // "what does this body cost", and take their own pair.
            const uint64_t now = RigExecProfiler::NowUs();
            step.startUs = mark;
            step.endUs = now;
            mark = now;
        }
        // The phased-read records this step made, folded into the run's store
        // in step order -- which is what lets a step record into a store
        // nobody else can see and still leave the walk's order deciding what
        // the store ends up holding.
        if (!step.snapshots.IsEmpty()) {
            B.runSnapshots.Merge(std::move(step.snapshots));
        }
        if (step.bail) {
            // The generation is going back to the dynamic path, so nothing
            // after this step is worth running -- and nothing it would have
            // written is ever read: the caller drops the program.
            return false;
        }
    }
    return true;
}

/// What one parallel region's tasks share.
///
/// Everything mutable in here is either an atomic or a slot the graph says
/// one step owns. There is no mutex, no spin lock and no condition variable:
/// the remaining-predecessor counters and WorkDispatcher's own task queue
/// are the whole of the synchronisation (§2.2).
struct ParallelRun {
    RigExecBakedProgramImpl *program = nullptr;
    UsdTimeCode time;
    WorkDispatcher *dispatcher = nullptr;
    RigExecBakedClusterCounter *counters = nullptr;
    /// A step gave the generation back. Nothing after it is worth running,
    /// so clusters still to start give up -- but a step already running is
    /// never interrupted, because a half-written slot is a different kind of
    /// wrong from a slot nobody publishes.
    std::atomic<bool> bailed{false};
    bool profiling = false;
    bool timing = false;
    /// Whether each step adds its own nanoseconds to its own accumulator.
    /// See RigExecBakedStepTimingRequested for why this is off by default.
    bool measuring = false;
    /// Whether a step's phased-read records are folded into the run's store
    /// as the region goes. See RunStepsParallel for why that is safe only
    /// when the program has a reader for them.
    bool mergeInline = false;

    void RunFrom(int cluster);
};

void
ParallelRun::RunFrom(int start)
{
    RigExecBakedProgramImpl &B = *program;
    int current = start;
    while (current >= 0) {
        RigExecBakedCluster &cluster =
            B.clustering.clusters[size_t(current)];
        if (timing) {
            cluster.startUs = RigExecProfiler::NowUs();
        }
        for (const int index : cluster.members) {
            if (bailed.load(std::memory_order_relaxed)) {
                break;
            }
            RigExecBakedStep &step = B.steps[size_t(index)];
            if (step.isSource) {
                continue;  // ran before the region, with every other source
            }
            const uint64_t began = profiling ? RigExecProfiler::NowUs() : 0;
            const uint64_t beganNs = measuring ? NowNs() : 0;
            RunStepBody(&B, &step, time);
            if (measuring) {
                // Two stores into this step's own accumulators, which no
                // other task touches -- the same rule the interval pair
                // below follows, and the reason neither needs a lock.
                step.measuredUs += double(NowNs() - beganNs) / 1000.0;
                ++step.measuredRuns;
            }
            if (profiling) {
                // Two stores into storage this step alone owns. No profile
                // scope: RIGEXEC_PROFILE_SCOPE takes three mutexes even with
                // recording off, and the epilogue replays these intervals in
                // step order, which makes the trace deterministic as well as
                // lock-free.
                step.startUs = began;
                step.endUs = RigExecProfiler::NowUs();
            }
            if (mergeInline && !step.snapshots.IsEmpty()) {
                B.runSnapshots.Merge(std::move(step.snapshots));
            }
            if (step.bail) {
                bailed.store(true, std::memory_order_relaxed);
                break;
            }
        }
        if (timing) {
            cluster.endUs = RigExecProfiler::NowUs();
        }
        // Release what this cluster wrote to whoever picks its successors
        // up, and acquire it on the thread that sees the last decrement.
        int next = -1;
        for (const int succ : cluster.succs) {
            if (!B.closed.Test(succ)) {
                // A skipped cluster is never seeded and never counted, so a
                // predecessor of one has nothing to hand it.
                continue;
            }
            if (counters[succ].remaining.fetch_sub(
                    1, std::memory_order_acq_rel) != 1) {
                continue;
            }
            if (timing) {
                // Written by the thread that made the cluster runnable,
                // which is the same thread that then spawns or runs it --
                // so there is no second writer and no race with startUs.
                B.clustering.clusters[size_t(succ)].readyUs =
                    RigExecProfiler::NowUs();
            }
            if (next >= 0) {
                const int spawn = next;
                dispatcher->Run([this, spawn]() { RunFrom(spawn); });
            }
            next = succ;
        }
        // All but one spawned; the last runs here, on the thread that has
        // this cluster's writes in its cache already.
        current = next;
    }
}

bool
RunStepsParallel(RigExecBakedProgramImpl *program, UsdTimeCode time)
{
    RigExecBakedProgramImpl &B = *program;
    if (B.clustering.clusters.empty() || !B.clusterCounters) {
        return RunStepsSerial(program, time);
    }
    ParallelRun run;
    run.program = &B;
    run.time = time;
    run.counters = B.clusterCounters.get();
    run.profiling = B.profiler && B.profiler->IsEnabled();
    run.measuring = RigExecBakedStepTimingRequested() &&
                    !B.measurementSuspended;
    run.timing = run.profiling || RigExecBakedScheduleReportRequested();
    B.clustering.lastRunTimed = run.timing;
    // The run's phased-read store is one container, and folding a step's
    // records into it is a write to it. When some revision declares a read
    // phase the graph orders every writer and reader of the store against
    // each other (see the widening in RigExecBakedBuildSchedule), so the
    // fold may happen where the serial executor does it. When nothing
    // declares one, nothing can LOOK a record up before the epilogue, so the
    // folds all wait until after the region -- in step order, which is the
    // order that decides what the store ends up holding.
    run.mergeInline = B.phasedReads;

    std::vector<int> seeds;
    const uint64_t opened = run.timing ? RigExecProfiler::NowUs() : 0;
    for (size_t c = 0; c < B.clustering.clusters.size(); ++c) {
        RigExecBakedCluster &cluster = B.clustering.clusters[c];
        cluster.readyUs = cluster.startUs = cluster.endUs = opened;
        if (!B.closed.Test(int(c))) {
            // Skipped: nothing decrements it and it seeds nothing. Its
            // counter is left where a skipped cluster's belongs, at the
            // number of predecessors it will never be handed.
            run.counters[c].remaining.store(int(cluster.preds.size()),
                                            std::memory_order_relaxed);
            continue;
        }
        int waiting = 0;
        for (const int pred : cluster.preds) {
            waiting += B.closed.Test(pred) ? 1 : 0;
        }
        run.counters[c].remaining.store(waiting, std::memory_order_relaxed);
        if (waiting == 0) {
            seeds.push_back(int(c));
        }
    }

    // Isolated: Run is never entered from an exec callback, but a client may
    // call Evaluate from a TBB task, and without isolation this dispatcher's
    // Wait could pick up that outer task's work and re-enter the region.
    WorkWithScopedParallelism([&run, &seeds]() {
        WorkDispatcher dispatcher;
        run.dispatcher = &dispatcher;
        for (size_t i = 0; i + 1 < seeds.size(); ++i) {
            const int seed = seeds[i];
            dispatcher.Run([&run, seed]() { run.RunFrom(seed); });
        }
        if (!seeds.empty()) {
            run.RunFrom(seeds.back());
        }
        dispatcher.Wait();
    });

    if (!run.mergeInline) {
        for (RigExecBakedStep &step : B.steps) {
            if (!step.snapshots.IsEmpty()) {
                B.runSnapshots.Merge(std::move(step.snapshots));
            }
        }
    }
    return !run.bailed.load(std::memory_order_relaxed);
}

}  // namespace

bool
RigExecBakedRunSteps(RigExecBakedProgramImpl *program, UsdTimeCode time,
                     bool force)
{
    // Last frame's intervals must not survive into this one. A step's own
    // BeginRun cannot do this: a run that bails never reaches the steps
    // after it, so BeginRun is exactly what those steps do not get, and the
    // epilogue would replay their previous intervals into the trace of the
    // frame that gave up -- the one frame a reader takes at face value.
    // Cleared here, once, so that neither executor has to remember it.
    for (RigExecBakedStep &step : program->steps) {
        step.startUs = step.endUs = 0;
    }
    // The sources, before anything that could be skipped: they are what the
    // dirty set is computed FROM. They read nothing a step OUTSIDE this pass
    // writes -- which is not the same as reading nothing at all, and stopped
    // being the same when weight objects started baking: a source weight
    // packet is composed from other source packets, and a source assemble
    // reads them. So this loop stays SERIAL and stays in program order,
    // which for the weight steps is dependency order; running it in
    // parallel, or reordering it, would read a packet before its own step
    // built it. Anything added here that needs a different order needs its
    // own edges instead.
    for (RigExecBakedStep &step : program->steps) {
        if (step.isSource) {
            RunStepBody(program, &step, time);
        }
    }
    RigExecBakedComputeClosure(program, time, force);
    for (RigExecBakedStep &step : program->steps) {
        if (step.isSource || program->closed.Test(step.cluster)) {
            continue;
        }
        step.MarkSkipped();
        if (RigExecBakedIsGeometryStep(step.kind)) {
            // A geometry step writes DELTAS beside its values -- "the
            // influence table moved", "the packet moved", "the revision
            // executed" -- and a delta is the one thing last run's answer is
            // never this run's. The values stand; the deltas are reset to
            // what a step that did not run means by them, which is "nothing
            // moved", and is the truth: the step was skipped precisely
            // because nothing it reads did.
            RigExecBakedSkipGeometryStep(program, &step);
        }
    }
    // Calibration times the steps to fit the cost table, so it runs the
    // reference order whatever the mode asks for: a step's interval in a
    // parallel frame includes the memory traffic of every other step that
    // happened to be running beside it.
    if (RigExecBakedScheduleModeFromEnvironment() ==
            RigExecBakedScheduleMode::Parallel &&
        !RigExecBakedScheduleCalibrationRequested()) {
        return RunStepsParallel(program, time);
    }
    return RunStepsSerial(program, time);
}

// ---------------------------------------------------------------------------
// Calibration.
// ---------------------------------------------------------------------------

bool
RigExecBakedStepTimingRequested()
{
    static const bool requested =
        TfGetenvInt("RIGEXEC_BAKED_STEP_TIMING", 0) > 0;
    return requested;
}

bool
RigExecBakedScheduleCalibrationRequested()
{
    static const bool requested =
        TfGetenvInt("RIGEXEC_BAKED_SCHEDULE_CALIBRATE", 0) > 0;
    return requested;
}

namespace {

/// How many frames the calibration watches before it prints.
///
/// RIGEXEC_BAKED_SCHEDULE_CALIBRATE=1 means "the usual number", because 1 is
/// what an opt-in flag is usually set to; any larger value is taken as the
/// count. The first frame is watched like the rest: it is the cold one, and
/// a cost model that ignored cold frames would under-count exactly the work
/// a first frame does.
int
CalibrationFrames()
{
    static const int frames = [] {
        const int value = TfGetenvInt("RIGEXEC_BAKED_SCHEDULE_CALIBRATE", 0);
        return value > 1 ? value : 8;
    }();
    return frames;
}

/// Least squares of t = a + b x size over one kind's steps.
StepCostConstants
FitKind(const std::vector<std::pair<double, double>> &samples,
        const StepCostConstants &fallback)
{
    const double n = double(samples.size());
    if (samples.empty()) {
        return fallback;
    }
    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (const auto &[size, microseconds] : samples) {
        sx += size;
        sy += microseconds;
        sxx += size * size;
        sxy += size * microseconds;
    }
    StepCostConstants fitted;
    const double denominator = n * sxx - sx * sx;
    if (denominator > 1e-9) {
        fitted.perUnitUs = (n * sxy - sx * sy) / denominator;
        fitted.fixedUs = (sy - fitted.perUnitUs * sx) / n;
    }
    if (denominator > 1e-9 && fitted.perUnitUs >= 0 && fitted.fixedUs >= 0) {
        return fitted;
    }
    // The samples could not separate the two terms -- one sample, or every
    // sample the same size -- or the separation came out negative. Then fit
    // through the ORIGIN and give the whole of the measurement to the
    // per-unit term, because that is the term that extrapolates: a rig whose
    // mesh is ten times this one's should be predicted to cost about ten
    // times as much, not the same.
    fitted.fixedUs = 0;
    fitted.perUnitUs = sx > 0 ? sy / sx : 0;
    if (sx <= 0) {
        fitted.fixedUs = sy / n;
    }
    return fitted;
}

}  // namespace

void
RigExecBakedScheduleCalibrate(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    static int framesSeen = 0;
    if (framesSeen >= CalibrationFrames()) {
        return;
    }
    if (++framesSeen < CalibrationFrames()) {
        return;
    }
    std::array<std::vector<std::pair<double, double>>, kStepKindCount>
        samples;
    for (const RigExecBakedStep &step : B.steps) {
        if (!step.measuredRuns) {
            continue;
        }
        samples[size_t(step.kind)].emplace_back(
            step.sizeUnits, step.measuredUs / double(step.measuredRuns));
    }
    std::string table =
        "rigExec baked schedule: cost table fitted over " +
        std::to_string(CalibrationFrames()) +
        " frame(s); paste over kStepCosts in bakedSchedule.cpp\n"
        "constexpr StepCostConstants kStepCosts[] = {\n";
    char line[256];
    for (size_t kind = 0; kind < kStepKindCount; ++kind) {
        const StepCostConstants fitted =
            FitKind(samples[kind], kStepCosts[kind]);
        std::snprintf(line, sizeof(line),
                      "    {%.4f, %.6f},   // %-16s %zu sample(s)\n",
                      fitted.fixedUs, fitted.perUnitUs,
                      RigExecBakedStepKindName(RigExecBakedStepKind(kind)),
                      samples[kind].size());
        table += line;
    }
    table += "};\n";
    std::fwrite(table.data(), 1, table.size(), stderr);
}

namespace {

/// How many frames the step timing watches before it prints, on the same
/// rule the calibration uses: 1 means "the usual number", anything larger is
/// the count.
int
StepTimingFrames()
{
    static const int frames = [] {
        const int value = TfGetenvInt("RIGEXEC_BAKED_STEP_TIMING", 0);
        return value > 1 ? value : 8;
    }();
    return frames;
}

}  // namespace

void
RigExecBakedStepTimingReport(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    static int framesSeen = 0;
    if (framesSeen >= StepTimingFrames()) {
        return;
    }
    if (++framesSeen < StepTimingFrames()) {
        return;
    }
    const double frames = double(std::max<size_t>(B.timedFrames, 1));
    std::array<double, kStepKindCount> byKind{};
    std::array<size_t, kStepKindCount> runsByKind{};
    double steps = 0;
    for (const RigExecBakedStep &step : B.steps) {
        if (!step.measuredRuns) {
            continue;
        }
        byKind[size_t(step.kind)] += step.measuredUs;
        runsByKind[size_t(step.kind)] += step.measuredRuns;
        steps += step.measuredUs;
    }
    char line[256];
    std::string out = "rigExec baked step timing over " +
                      std::to_string(B.timedFrames) + " frame(s), us/frame\n";
    std::snprintf(line, sizeof(line),
                  "  prologue %8.1f  region %8.1f  epilogue %8.1f  "
                  "frame %8.1f\n",
                  B.timedPrologueUs / frames, B.timedRegionUs / frames,
                  B.timedEpilogueUs / frames,
                  (B.timedPrologueUs + B.timedRegionUs + B.timedEpilogueUs) /
                      frames);
    out += line;
    // The steps sum to less than the region: the region also computes the
    // dirty closure, runs the sources and, in parallel mode, waits.
    std::snprintf(line, sizeof(line),
                  "  step bodies %8.1f of the region\n", steps / frames);
    out += line;
    std::vector<std::pair<double, size_t>> order;
    for (size_t kind = 0; kind < kStepKindCount; ++kind) {
        if (runsByKind[kind]) {
            order.emplace_back(byKind[kind], kind);
        }
    }
    std::sort(order.begin(), order.end(),
              [](const auto &a, const auto &b) { return a.first > b.first; });
    for (const auto &[total, kind] : order) {
        std::snprintf(
            line, sizeof(line), "  %-16s %8.2f  %6.2f per run, %zu run(s)\n",
            RigExecBakedStepKindName(RigExecBakedStepKind(kind)),
            total / frames, total / double(runsByKind[kind]),
            size_t(double(runsByKind[kind]) / frames + 0.5));
        out += line;
    }
    std::fwrite(out.data(), 1, out.size(), stderr);
}

// ---------------------------------------------------------------------------
// The report.
// ---------------------------------------------------------------------------

namespace {

std::string
StepLabel(const RigExecBakedProgramImpl &B, const RigExecBakedStep &step)
{
    const auto revisionOf = [&B](int id) -> const
        RigExecBakedProgramImpl::GeomRevision & {
        const auto &[chain, revision] = B.revisionIndex[size_t(id)];
        return B.chains[size_t(chain)].revisions[size_t(revision)];
    };
    switch (step.kind) {
    case RigExecBakedStepKind::ComposeSubtree:
        return B.paths[size_t(B.composeGroups[size_t(step.object)].begin)]
            .GetString();
    case RigExecBakedStepKind::Solve:
        return B.solvers[size_t(step.object)].path.GetString();
    case RigExecBakedStepKind::SolverCommit:
    case RigExecBakedStepKind::Constraint:
    case RigExecBakedStepKind::CommitDelta:
    case RigExecBakedStepKind::PropagateChunk:
    case RigExecBakedStepKind::CommitApply: {
        const RigExecBakedCommit &commit = B.commits[size_t(step.object)];
        return commit.moverPath.IsEmpty()
                   ? "batch " + std::to_string(step.object)
                   : commit.moverPath.GetString();
    }
    case RigExecBakedStepKind::ProviderMatrix:
        return B.paths[size_t(step.object)].GetString() +
               (step.part ? " final" : " base");
    case RigExecBakedStepKind::SnapshotFinals:
        return "every provider";
    case RigExecBakedStepKind::VolumePlacements:
        return "every volume weight";
    case RigExecBakedStepKind::WeightPacket:
        return B.weightObjects[size_t(step.object)].path.GetString();
    case RigExecBakedStepKind::InfluenceFold:
    case RigExecBakedStepKind::RevisionStatic:
    case RigExecBakedStepKind::RevisionChunk:
    case RigExecBakedStepKind::RevisionFuse:
        return revisionOf(step.object).moverPath.GetString();
    case RigExecBakedStepKind::ChainStatus:
        return B.chains[size_t(step.object)].target.GetString();
    case RigExecBakedStepKind::Derived: {
        const auto &[chain, derived] = B.derivedIndex[size_t(step.object)];
        return B.chains[size_t(chain)]
            .derived[size_t(derived)]
            .target.GetString();
    }
    }
    return std::string();
}

void
AppendRanges(std::string *out, const char *what,
              const std::vector<RigExecBakedSlotRange> &ranges)
{
    *out += "        ";
    *out += what;
    if (ranges.empty()) {
        *out += " -\n";
        return;
    }
    for (const RigExecBakedSlotRange &range : ranges) {
        *out += " ";
        *out += RigExecBakedSlotDomainName(range.domain);
        *out += "[" + std::to_string(range.begin) + "," +
                std::to_string(range.end) + ")";
    }
    *out += "\n";
}

/// A number with two decimals, because std::to_string gives six and the
/// report is read by people.
std::string
Fixed(double value)
{
    char text[32];
    std::snprintf(text, sizeof(text), "%.2f", value);
    return text;
}

}  // namespace

std::string
RigExecBakedScheduleReport(const RigExecBakedProgramImpl &B)
{
    size_t edges = 0;
    std::map<std::string, size_t> byKind;
    for (const RigExecBakedStep &step : B.steps) {
        edges += step.preds.size();
        ++byKind[RigExecBakedStepKindName(step.kind)];
    }
    const RigExecBakedClustering &schedule = B.clustering;
    size_t clusterEdges = 0;
    for (const RigExecBakedCluster &cluster : schedule.clusters) {
        clusterEdges += cluster.preds.size();
    }
    std::string out = "rigExec baked schedule: " +
                      std::to_string(B.steps.size()) + " step(s), " +
                      std::to_string(edges) + " edge(s), " +
                      std::to_string(B.paths.size()) + " provider slot(s)\n";
    out += "  mode=";
    out += RigExecBakedScheduleModeFromEnvironment() ==
                   RigExecBakedScheduleMode::Parallel
               ? "parallel"
               : "serial";
    out += " grain=" + Fixed(schedule.grainUs) + "us concurrency=" +
           std::to_string(WorkGetConcurrencyLimit()) + "\n";
    out += "  clusters=" + std::to_string(schedule.clusters.size()) + " (" +
           std::to_string(clusterEdges) + " edge(s)) serial=" +
           Fixed(schedule.serialCost) + "us criticalPath=" +
           Fixed(schedule.criticalPathCost) + "us";
    if (schedule.criticalPathCost > 0) {
        out += " (" +
               Fixed(schedule.serialCost / schedule.criticalPathCost) + "x)";
    }
    out += "\n";
    for (const auto &[kind, count] : byKind) {
        out += "  " + kind + " " + std::to_string(count) + "\n";
    }
    // Per skin revision: the vertex partition's shape, and for every chunk
    // the level its own influences are final at against the frame's deepest
    // level. The geometry half owns the chunk keys, so it writes these lines
    // -- it reads the levels and costs this file just filled in.
    const std::string geometry = RigExecBakedGeometryReport(B);
    if (!geometry.empty()) {
        out += "  skin revisions:\n" + geometry;
    }
    for (size_t c = 0; c < schedule.clusters.size(); ++c) {
        const RigExecBakedCluster &cluster = schedule.clusters[c];
        out += "  cluster [" + std::to_string(c) + "] level " +
               std::to_string(cluster.level) + " cost " +
               Fixed(cluster.cost) + "us " +
               std::to_string(cluster.members.size()) + " step(s)\n";
        out += "        preds";
        if (cluster.preds.empty()) {
            out += " -";
        }
        for (const int pred : cluster.preds) {
            out += " " + std::to_string(pred);
        }
        out += "\n        members";
        for (const int member : cluster.members) {
            out += " " + std::to_string(member);
        }
        out += "\n";
    }
    for (int index = 0; index < int(B.steps.size()); ++index) {
        const RigExecBakedStep &step = B.steps[size_t(index)];
        out += "  [" + std::to_string(index) + "] " + step.label;
        if (step.part >= 0) {
            out += " #" + std::to_string(step.part);
        }
        out += " cluster " + std::to_string(step.cluster) + " level " +
               std::to_string(step.level) + " size " +
               Fixed(step.sizeUnits) + " cost " + Fixed(step.cost) + "us";
        out += "\n";
        AppendRanges(&out, "reads ", step.reads);
        AppendRanges(&out, "writes", step.writes);
        out += "        preds ";
        if (step.preds.empty()) {
            out += "-";
        }
        for (const int pred : step.preds) {
            out += " " + std::to_string(pred);
        }
        out += "\n";
    }
    return out;
}


std::string
RigExecBakedScheduleRunReport(const RigExecBakedProgramImpl &B)
{
    const RigExecBakedClustering &schedule = B.clustering;
    std::string out = "rigExec baked schedule, last run: " +
                      std::to_string(B.lastClosedClusters) + " of " +
                      std::to_string(schedule.clusters.size()) +
                      " cluster(s) run\n";
    if (!schedule.lastRunTimed) {
        // Say which it is. A serial frame leaves every ready/wait/run at
        // zero, and a table of zeros does not read as "unmeasured" -- it
        // reads as "free", which is the one thing it never means.
        out += "  the last run was serial: clusters are not timed. Run with "
               "RIGEXEC_BAKED_SCHEDULE=parallel\n  for the per-cluster "
               "ready/wait/run table; the structural half of the report is "
               "printed at Build.\n";
        return out;
    }
    // The region opened when the earliest cluster became ready, which is
    // when the seeds were stamped. Times are relative to it, so two runs of
    // one frame can be laid beside each other.
    uint64_t opened = 0;
    bool haveOpened = false;
    for (const RigExecBakedCluster &cluster : schedule.clusters) {
        if (!haveOpened || cluster.readyUs < opened) {
            opened = cluster.readyUs;
            haveOpened = true;
        }
    }
    for (size_t c = 0; c < schedule.clusters.size(); ++c) {
        const RigExecBakedCluster &cluster = schedule.clusters[c];
        const double ready = double(cluster.readyUs - opened);
        const double started = double(cluster.startUs - opened);
        out += "  cluster [" + std::to_string(c) + "] ready " +
               Fixed(ready) + "us wait " + Fixed(started - ready) +
               "us run " +
               Fixed(double(cluster.endUs) - double(cluster.startUs)) +
               "us cost " + Fixed(cluster.cost) + "us " +
               std::to_string(cluster.members.size()) + " step(s)\n";
    }
    return out;
}

// ---------------------------------------------------------------------------
// Profiling.
// ---------------------------------------------------------------------------

void
RigExecBakedReplayStepTimings(const RigExecBakedProgramImpl &B)
{
    if (!B.profiler || !B.profiler->IsEnabled()) {
        return;
    }
    for (const RigExecBakedStep &step : B.steps) {
        // A step that did not take a whole microsecond is on nobody's
        // critical path, and a biped's graph holds several hundred of them
        // -- so recording each would cost more than the steps did and would
        // bury the events that matter under zero-length ones. The interval
        // is still measured; what is dropped is the report of it.
        if (step.endUs <= step.startUs) {
            continue;
        }
        B.profiler->Record(step.label, "step", step.startUs, step.endUs);
    }
}

}  // namespace rigExec
