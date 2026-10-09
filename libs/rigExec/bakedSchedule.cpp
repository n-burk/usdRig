#include "rigExec/weightField.h"
// The baked program's scheduler: edges, executors and the report.
// See bakedSchedule.h for what belongs here and what belongs with a domain.
#include "bakedSchedule.h"
#include "bakedOpGraph.h"
#include "rigExecGraph/solverProgram.h"
#include "bakedExecCrossCheckRows.h"

#include "bakedTrace.h"
#include "bodyPurity.h"
#include "parallel.h"
#include "pathText.h"
#include "profiler.h"
#include "rigEvaluator.h"

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
#include <functional>
#include <map>
#include <memory>
#include <numeric>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

namespace rigExec {

namespace {
std::string StepLabel(const RigExecBakedProgramImpl &, const RigExecBakedStep &,
                      RigExecPathText &);
}

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
    case RigExecBakedSlotDomain::PoseWeight: return "PoseWeight";
    case RigExecBakedSlotDomain::Snapshots: return "Snapshots";
    case RigExecBakedSlotDomain::FrameMatrix: return "FrameMatrix";
    case RigExecBakedSlotDomain::Rest: return "Rest";
    case RigExecBakedSlotDomain::Ladder: return "Ladder";
    case RigExecBakedSlotDomain::SkinTopology: return "SkinTopology";
    case RigExecBakedSlotDomain::WeightField: return "WeightField";
    case RigExecBakedSlotDomain::WeightFramesBase: return "WeightFramesBase";
    case RigExecBakedSlotDomain::SpaceValue: return "SpaceValue";
    case RigExecBakedSlotDomain::SpaceLeaf: return "SpaceLeaf";
    case RigExecBakedSlotDomain::DerivedBase: return "DerivedBase";
    case RigExecBakedSlotDomain::ChainInput: return "ChainInput";
    case RigExecBakedSlotDomain::ConstraintInputs: return "ConstraintInputs";
    case RigExecBakedSlotDomain::SwitchFrame: return "SwitchFrame";
    case RigExecBakedSlotDomain::RequiredStageFramesAdmission: return "RequiredStageFramesAdmission";
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
    case RigExecBakedStepKind::PoseInterpolator: return "PoseInterpolator";
    case RigExecBakedStepKind::VolumePlacements: return "VolumePlacements";
    case RigExecBakedStepKind::WeightPacket: return "WeightPacket";
    case RigExecBakedStepKind::InfluenceFold: return "InfluenceFold";
    case RigExecBakedStepKind::RevisionStatic: return "RevisionStatic";
    case RigExecBakedStepKind::RevisionChunk: return "RevisionChunk";
    case RigExecBakedStepKind::RevisionFuse: return "RevisionFuse";
    case RigExecBakedStepKind::ChainStatus: return "ChainStatus";
    case RigExecBakedStepKind::Derived: return "Derived";
    case RigExecBakedStepKind::FrameMatrix: return "FrameMatrix";
    case RigExecBakedStepKind::PropertyRevision: return "PropertyRevision";
    case RigExecBakedStepKind::RestCompose: return "RestCompose";
    case RigExecBakedStepKind::LadderCompose: return "LadderCompose";
    case RigExecBakedStepKind::SkinTopology: return "SkinTopology";
    case RigExecBakedStepKind::WeightField: return "WeightField";
    case RigExecBakedStepKind::SpaceExpression: return "SpaceExpression";
    case RigExecBakedStepKind::ChainInputs: return "ChainInputs";
    case RigExecBakedStepKind::AvarInputs: return "AvarInputs";
    case RigExecBakedStepKind::SpaceCheckpoint: return "SpaceCheckpoint";
    case RigExecBakedStepKind::ProviderRefresh: return "ProviderRefresh";
    }
    return "unknown";
}

// Execution mode.

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
        return TfGetenv("RIGEXEC_BAKED_SCHEDULE", "parallel") == "parallel"
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

// The cost model.
// cost = a[kind] + b[kind] x size(step), in microseconds. Two constants per
// step kind and one size per step, which is as much model as a scheduler can
// use: the packing only ever asks "is this bin about a grain yet", so what it
// needs is the RATIO between a skin chunk and a constraint, not either one's
// absolute time.
// The size of a step is the count that its body's inner loop runs over:
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
//   FrameMatrix      one matrix, like ProviderMatrix
//   SnapshotFinals   provider slots (reserved: no longer emitted)
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
// The constants below were fitted by RIGEXEC_BAKED_SCHEDULE_CALIBRATE=1 over
// eight frames of examples/biped/Biped_anim.usda on a 20-core box (see
// RigExecBakedScheduleCalibrationRequested). They are a machine's
// numbers, so they will be wrong on another machine by some factor -- which
// costs a schedule that is packed a little coarse or a little fine, and never
// an answer. Build must not measure: a schedule that depended on what the box
// was doing while the program was built could not be tested for producing the
// same values at every grain.

namespace {

/// The two constants of one step kind, in microseconds.
struct StepCostConstants {
    double fixedUs = 0;
    double perUnitUs = 0;
};

constexpr size_t kStepKindCount =
    size_t(RigExecBakedStepKind::ProviderRefresh) + 1;

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
    {0.0000, 0.077140},   // SnapshotFinals    1 -- 13_ReadPhases; retired
    {0.2000, 0.020000},   // PoseInterpolator  unfitted -- see below
    {0.0000, 0.143880},   // VolumePlacements  1 -- 11_VolumeWeights
    {0.0000, 0.066481},   // WeightPacket      5 -- 11_VolumeWeights
    {0.0000, 0.003781},   // InfluenceFold     1
    {0.0000, 0.000487},   // RevisionStatic    1
    {11.3239, 0.000714},  // RevisionChunk     7 -- see below
    {0.0000, 0.594000},   // RevisionFuse      1
    {0.0000, 0.000336},   // ChainStatus       1 -- see below
    {0.0000, 0.003744},   // Derived           1
    {0.0000, 0.051952},   // FrameMatrix       unfitted -- ProviderMatrix's row
    {0.0, 0.0},          // PropertyRevision: serial prologue
    {0.0, 0.0},          // RestCompose: serial prologue
    {0.0, 0.0},          // LadderCompose: serial prologue
    {0.0, 0.0},          // SkinTopology: serial prologue
    {0.25, 0.04},        // WeightField
    {0.04, 0.0},         // SpaceExpression
    {0.04, 0.0},         // ChainInputs
    {0.04, 0.0},         // AvarInputs
    {0.04, 0.087594},    // SpaceCheckpoint: ordinary compose per carried slot
    {0.04, 0.016869},    // ProviderRefresh: one provider and namespace carries
};
static_assert(sizeof(kStepCosts) / sizeof(kStepCosts[0]) == kStepKindCount,
              "rigExec: every baked step kind needs a cost row");

// Four rows are worth reading twice before they are trusted:
//  * PoseInterpolator is a GUESS, not a fit: two quaternion extractions, a
//    quaternion delta and one RBF kernel row per pose. No calibration run
//    has covered a rig with interpolators yet; the first one to do so should
//    replace the row the way the three below were replaced.
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

/// MeasureGeometry, deferred to the first step whose size asks for it.
///
/// Measuring reads every chain's points off the stage, and Build costs the
/// pose half before the geometry half exists: none of those steps asks, so
/// the read happens once, when the geometry steps are costed, and not once
/// per pass.
class LazyGeometrySizes {
public:
    explicit LazyGeometrySizes(const RigExecBakedProgramImpl &program)
        : _program(program)
    {
    }

    const GeometrySizes &Get()
    {
        if (!_sizes) {
            _sizes = MeasureGeometry(_program);
        }
        return *_sizes;
    }

private:
    const RigExecBakedProgramImpl &_program;
    std::optional<GeometrySizes> _sizes;
};

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
StepSize(const RigExecBakedProgramImpl &B, LazyGeometrySizes &geometry,
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
    case RigExecBakedStepKind::FrameMatrix:
    case RigExecBakedStepKind::RevisionFuse:
    case RigExecBakedStepKind::SpaceExpression:
    case RigExecBakedStepKind::ChainInputs:
    case RigExecBakedStepKind::AvarInputs:
        return 1;
    case RigExecBakedStepKind::SpaceCheckpoint:
        return double(B.switchFrameContexts[object].recompose.size());
    case RigExecBakedStepKind::ProviderRefresh:
        return double(1+B.providerRefreshes[object].carries.size());
    case RigExecBakedStepKind::SnapshotFinals:
        return double(B.paths.size());
    case RigExecBakedStepKind::PoseInterpolator:
        // One kernel row per pose in the solve.
        return double(std::max<size_t>(
            B.poseInterpolators[object].poseSlots.size(), 1));
    case RigExecBakedStepKind::VolumePlacements:
        // One decomposition: the step places one volume; the fitted row is
        // per volume.
        return 1;
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
        return geometry.Get().revisionPoints[object];
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
        if ((geom.role == RigExecBakedRevisionRole::Range ||
             (geom.role == RigExecBakedRevisionRole::Whole && geom.chunked)) &&
            step.part >= 0 && size_t(step.part) < geom.chunks.size()) {
            // A vertex group of a range chain -- a Range revision's group
            // step or a Whole keyed skin's speculative chunk: the group's
            // share of the revision's units.
            const RigExecBakedProgramImpl::GeomChunk &chunk =
                geom.chunks[size_t(step.part)];
            const double points =
                double(B.chains[size_t(chain)].groupPointCount);
            return points > 0.0
                       ? geometry.Get().revisionUnits[object] *
                             double(chunk.end - chunk.begin) / points
                       : 0.0;
        }
        if (geom.chunked && step.part >= 0 &&
            size_t(step.part) < geom.chunks.size()) {
            const RigExecBakedProgramImpl::GeomChunk &chunk =
                geom.chunks[size_t(step.part)];
            return double(chunk.end - chunk.begin) *
                   geometry.Get().revisionElementSize[object];
        }
        return geometry.Get().revisionUnits[object];
    }
    case RigExecBakedStepKind::ChainStatus:
        // The chain's points, not its revisions: the body copies the
        // published array whole (bakedGeometry.cpp), and the MoverFailed
        // sweep over the revisions costs a branch each.
        return geometry.Get().chainPoints[object];
    case RigExecBakedStepKind::Derived:
        return geometry.Get().derivedPoints[object];
    }
    return 1;
}

/// \p fixedUs for \p step, divided among the group steps of a range chain's
/// revision -- a Range revision's written groups, a Whole keyed skin's G
/// chunks -- so the revision's modelled cost is the one chunk's it replaced.
double
RangeFixedUs(const RigExecBakedProgramImpl &B, const RigExecBakedStep &step,
             double fixedUs)
{
    if (step.kind != RigExecBakedStepKind::RevisionChunk || step.object < 0 ||
        size_t(step.object) >= B.revisionIndex.size()) {
        return fixedUs;
    }
    const auto &[chain, revision] = B.revisionIndex[size_t(step.object)];
    const RigExecBakedProgramImpl::GeomRevision &geom =
        B.chains[size_t(chain)].revisions[size_t(revision)];
    size_t steps = 0;
    if (geom.role == RigExecBakedRevisionRole::Range) {
        steps = size_t(std::count(geom.groupWritten.begin(),
                                  geom.groupWritten.end(), char(1)));
    } else if (geom.role == RigExecBakedRevisionRole::Whole && geom.chunked) {
        steps = geom.chunks.size();
    }
    return steps > 0 ? fixedUs / double(steps) : fixedUs;
}

}  // namespace

void
RigExecBakedAssignStepCosts(RigExecBakedProgramImpl *program,
                            size_t firstStep)
{
    RigExecBakedProgramImpl &B = *program;
    LazyGeometrySizes geometry(B);
    RigExecPathText labels;
    for (int index = int(firstStep); index < int(B.steps.size()); ++index) {
        RigExecBakedStep &step = B.steps[size_t(index)];
        // Compiled identity owns the label; handwritten diagnostics may
        // still derive a label before the final descriptor is assigned.
        const auto category = step.descriptorKey.rfind("/category:");
        step.label = category == std::string::npos
            ? StepLabel(B, step, labels) : step.descriptorKey.substr(0, category);
        const StepCostConstants &constants = kStepCosts[size_t(step.kind)];
        step.sizeUnits = StepSize(B, geometry, step);
        step.cost = RangeFixedUs(B, step, constants.fixedUs) +
                    constants.perUnitUs * step.sizeUnits;
        // Longest path from a source, which is the level the packing groups
        // by. One forward pass, because program order is a topological order.
        int level = 0;
        for (const int pred : step.preds) {
            level = std::max(level, B.steps[size_t(pred)].level);
        }
        step.level = level + 1;
    }
}

// Clustering.

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

// Validation.
// The sweep sees only writers it has already passed, so a read whose
// producer comes later, or never, leaves no trace in the edges. Build asks
// here instead, once, before any executor is handed the program.

namespace {

/// What the validator found: the first violation in full, and how many.
struct GraphViolations {
    std::string first;
    size_t count = 0;

    void Add(std::string line) {
        if (count++ == 0) {
            first = std::move(line);
        }
    }
};

std::string
NameStep(const RigExecBakedProgramImpl &B, int index)
{
    std::string out = "step " + std::to_string(index);
    if (index >= 0 && size_t(index) < B.steps.size() &&
        !B.steps[size_t(index)].label.empty()) {
        out += " (" + B.steps[size_t(index)].label + ")";
    }
    return out;
}

bool
SortedUnique(const std::vector<int> &values)
{
    return std::adjacent_find(values.begin(), values.end(),
                              std::greater_equal<int>()) == values.end();
}

bool
SortedContains(const std::vector<int> &values, int value)
{
    return std::binary_search(values.begin(), values.end(), value);
}

bool
IsCommitStep(RigExecBakedStepKind kind)
{
    switch (kind) {
    case RigExecBakedStepKind::SolverCommit:
    case RigExecBakedStepKind::Constraint:
    case RigExecBakedStepKind::CommitDelta:
    case RigExecBakedStepKind::PropagateChunk:
    case RigExecBakedStepKind::CommitApply:
        return true;
    default:
        return false;
    }
}

/// preds point backward and succs forward, each sorted, unique and the
/// inverse of the other.
void
ValidateStepEdges(const RigExecBakedProgramImpl &B, GraphViolations *out)
{
    const int count = int(B.steps.size());
    for (int index = 0; index < count; ++index) {
        const RigExecBakedStep &step = B.steps[size_t(index)];
        if (!SortedUnique(step.preds) || !SortedUnique(step.succs)) {
            out->Add(NameStep(B, index) +
                     " lists an edge out of order or twice");
            continue;
        }
        for (const int pred : step.preds) {
            if (pred < 0 || pred >= index) {
                out->Add(NameStep(B, index) + " depends on " +
                         NameStep(B, pred) +
                         ", which is not earlier in program order");
            } else if (!SortedContains(B.steps[size_t(pred)].succs, index)) {
                out->Add(NameStep(B, index) + " depends on " +
                         NameStep(B, pred) +
                         ", which does not list it as a successor");
            }
        }
        for (const int succ : step.succs) {
            if (succ <= index || succ >= count) {
                out->Add(NameStep(B, index) + " names successor " +
                         NameStep(B, succ) +
                         ", which is not later in program order");
            } else if (!SortedContains(B.steps[size_t(succ)].preds, index)) {
                out->Add(NameStep(B, index) + " names successor " +
                         NameStep(B, succ) + ", which does not depend on it");
            }
        }
    }
}

bool
IsDeclaredSeedOrExcluded(const RigExecBakedProgramImpl &B,
                     RigExecBakedSlotDomain domain, uint32_t entry)
{
    if ((domain == RigExecBakedSlotDomain::PoseFin ||
         domain == RigExecBakedSlotDomain::PoseBase) && entry < B.paths.size() &&
        std::find(B.xformSlots.begin(), B.xformSlots.end(), int(entry)) != B.xformSlots.end())
        return true;
    for (const auto id : B.opAdapter.excludedValues) {
        if (id >= B.opAdapter.values.size()) continue;
        const auto &value = B.opAdapter.values[size_t(id)];
        if (value.domain == uint32_t(domain) && value.slot == entry) return true;
    }
    return false;
}

/// Every slot read in a domain the prologue does not fill has a writer at a
/// strictly lower index. No step may name the retired Snapshots domain: its
/// store is gone, so a read of it would order against nothing.
// Solver input requirements are declared semantic edges, independent of typed
// numerical reads. Validate their captured identities before inspecting the
// generated dependency graph; an arbitrary historical pred is not authority.
void
ValidateSemanticRequirements(const RigExecBakedProgramImpl &B, GraphViolations *out)
{
    std::map<std::string,int> active;
    for (size_t i = 0; i < B.steps.size(); ++i)
        if (!B.steps[i].descriptorKey.empty() && !active.emplace(B.steps[i].descriptorKey,int(i)).second)
            out->Add(NameStep(B,int(i)) + " repeats a descriptor key");
    for (size_t i = 0; i < B.steps.size(); ++i) {
        const auto &step = B.steps[i];
        if (step.kind != RigExecBakedStepKind::Solve) {
            if (!step.semanticPredecessorKeys.empty())
                out->Add(NameStep(B,int(i)) + " declares solver requirements on another body");
            continue;
        }
        if (step.object < 0 || size_t(step.object) >= B.solvers.size()) continue;
        const auto &solver = B.solvers[size_t(step.object)];
        // Pure numerical fixtures have no authored relationship metadata.
        if (solver.solveDescriptorKey.empty() && solver.relationshipRequirements.empty() &&
            step.semanticPredecessorKeys.empty()) continue;
        if (!solver.solveDescriptorKey.empty() && solver.solveDescriptorKey != step.descriptorKey)
            out->Add(NameStep(B,int(i)) + " differs from its solver descriptor identity");
        const auto ports = RigExecSolverRelationshipPorts(solver.kind);
        std::set<std::pair<std::string,int>> seen;
        std::map<std::string,size_t> portCounts;
        std::vector<std::string> required;
        for (const auto &requirement : solver.relationshipRequirements) {
            if (std::find(ports.begin(),ports.end(),requirement.first) == ports.end() ||
                !seen.insert(requirement).second ||
                (requirement.first != "rigExec:controls" &&
                 !(solver.kind == RigExecSolverKind::Ribbon && requirement.first != "rigExec:driverCurve") &&
                 ++portCounts[requirement.first] > 1) ||
                requirement.second < 0 || size_t(requirement.second) >= B.solvers.size()) {
                out->Add(NameStep(B,int(i)) + " has an invalid solver relationship requirement");
                continue;
            }
            if (solver.kind == RigExecSolverKind::BlendPointFrames &&
                ((requirement.first == "rigExec:inputA" && requirement.second != solver.inA) ||
                 (requirement.first == "rigExec:inputB" && requirement.second != solver.inB)))
                out->Add(NameStep(B,int(i)) + " has a relationship differing from its aggregate binding");
            const auto &source = B.solvers[size_t(requirement.second)];
            if (source.solveDescriptorKey.empty()) {
                out->Add(NameStep(B,int(i)) + " requires a solver with no descriptor identity");
                continue;
            }
            required.push_back(source.solveDescriptorKey);
            const auto found = active.find(source.solveDescriptorKey);
            if (found != active.end()) {
                const auto &producer = B.steps[size_t(found->second)];
                if (producer.kind != RigExecBakedStepKind::Solve || producer.object != requirement.second ||
                    !SortedContains(step.preds,found->second))
                    out->Add(NameStep(B,int(i)) + " omits its declared solver prerequisite");
            } else if (!IsDeclaredSeedOrExcluded(B,RigExecBakedSlotDomain::Aggregate,uint32_t(requirement.second)) ||
                       !IsDeclaredSeedOrExcluded(B,RigExecBakedSlotDomain::Candidates,uint32_t(requirement.second)))
                out->Add(NameStep(B,int(i)) + " requires a solver that is neither active nor excluded");
        }
        if (solver.kind == RigExecSolverKind::BlendPointFrames) {
            if (solver.inA >= 0 && !seen.count({"rigExec:inputA",solver.inA}))
                out->Add(NameStep(B,int(i)) + " omits its inputA relationship requirement");
            if (solver.inB >= 0 && !seen.count({"rigExec:inputB",solver.inB}))
                out->Add(NameStep(B,int(i)) + " omits its inputB relationship requirement");
        }
        std::sort(required.begin(),required.end());
        required.erase(std::unique(required.begin(),required.end()),required.end());
        if (step.semanticPredecessorKeys != required)
            out->Add(NameStep(B,int(i)) + " differs from its declared solver prerequisites");
    }
}

void
ValidateSlotProducers(const RigExecBakedProgramImpl &B, GraphViolations *out)
{
    const auto checked = [](RigExecBakedSlotDomain domain) {
        return !RigExecBakedIsSourceDomain(domain);
    };
    for (int index = 0; index < int(B.steps.size()); ++index) {
        const RigExecBakedStep &step = B.steps[size_t(index)];
        for (const auto *ranges : {&step.reads, &step.writes}) {
            for (const RigExecBakedSlotRange &range : *ranges) {
                if (range.domain == RigExecBakedSlotDomain::RequiredStageFramesAdmission &&
                    (range.begin != 0 || range.end != 1 || ranges == &step.writes))
                    out->Add(NameStep(B, index) + " has an invalid source-only stage-frame admission range");
                if (range.domain == RigExecBakedSlotDomain::Snapshots) {
                    out->Add(NameStep(B, index) +
                             " declares the retired Snapshots domain");
                }
            }
        }
    }
    // The lowest step whose declared writes cover each slot; -1 for none.
    std::array<std::vector<int>, RigExecBakedSlotDomainCount> firstWriter;
    for (int index = 0; index < int(B.steps.size()); ++index) {
        for (const RigExecBakedSlotRange &write :
                 B.steps[size_t(index)].writes) {
            if (!checked(write.domain) || write.IsEmpty()) {
                continue;
            }
            std::vector<int> &table = firstWriter[size_t(write.domain)];
            if (table.size() < write.end) {
                table.resize(write.end, -1);
            }
            for (uint32_t slot = write.begin; slot < write.end; ++slot) {
                if (table[slot] < 0) {
                    table[slot] = index;
                }
            }
        }
    }
    for (int index = 0; index < int(B.steps.size()); ++index) {
        for (const RigExecBakedSlotRange &read :
                 B.steps[size_t(index)].reads) {
            if (read.IsEmpty()) {
                continue;
            }
            // Formatted only for a violation: this runs over every read of
            // every step at each Build.
            const auto range = [&read]() {
                return std::string(RigExecBakedSlotDomainName(read.domain)) +
                       "[" + std::to_string(read.begin) + "," +
                       std::to_string(read.end) + ")";
            };
            if (!checked(read.domain) ||
                read.domain == RigExecBakedSlotDomain::Snapshots) {
                continue;
            }
            const std::vector<int> &table = firstWriter[size_t(read.domain)];
            uint32_t bad = 0, firstBad = 0;
            int badWriter = -1;
            for (uint32_t slot = read.begin; slot < read.end; ++slot) {
                const int writer = slot < table.size() ? table[slot] : -1;
                if (writer >= 0 && writer < index) {
                    if (!SortedContains(B.steps[size_t(index)].preds, writer))
                        out->Add(NameStep(B, index) + " reads " +
                            RigExecBakedSlotDomainName(read.domain) + "[" +
                            std::to_string(slot) + "] without an edge from its producer " +
                            NameStep(B, writer));
                    continue;
                }
                if (writer < 0 && IsDeclaredSeedOrExcluded(B, read.domain, slot)) continue;
                if (bad++ == 0) {
                    firstBad = slot;
                    badWriter = writer;
                }
            }
            if (bad == 0) {
                continue;
            }
            std::string line = NameStep(B, index) + " reads " +
                               RigExecBakedSlotDomainName(read.domain) + "[" +
                               std::to_string(firstBad) + "]";
            line += badWriter < 0
                        ? ", which no step writes"
                        : ", which no step before it writes (the first "
                          "writer is " + NameStep(B, badWriter) + ")";
            if (bad > 1) {
                line += "; " + std::to_string(bad) + " slots of " + range() +
                        " are unproduced";
            }
            out->Add(line);
        }
    }
}

/// Every pose version a reader is bound to (BindPoseVersions) was written by
/// an earlier step. The slot check cannot see this: a pose slot's first
/// writer is its compose, ahead of every commit, so a binding to a version a
/// LATER commit writes passes it.
void
ValidatePoseVersions(const RigExecBakedProgramImpl &B, GraphViolations *out)
{
    const size_t n = B.paths.size();
    if (n == 0) {
        return;  // No pose slots, so nothing is bound.
    }
    if (B.finLast.size() != n || B.baseLast.size() != n) {
        out->Add("the last-version table holds " +
                 std::to_string(B.finLast.size()) + " PoseFin and " +
                 std::to_string(B.baseLast.size()) + " PoseBase entries for " +
                 std::to_string(n) + " pose slots");
        return;
    }
    constexpr RigExecBakedSlotDomain kFin = RigExecBakedSlotDomain::PoseFin;
    constexpr RigExecBakedSlotDomain kBase = RigExecBakedSlotDomain::PoseBase;
    // Which step writes each entry, and which slot it is a version of. Entry
    // i < n is slot i's compose; the others are commit write-backs, each
    // written by its commit's last step. BindPoseVersions hands every entry
    // to one writer at most.
    std::vector<int> finWriter(B.fin.size(), -1), baseWriter(B.base.size(), -1);
    std::vector<int> finSlot(B.fin.size(), -1), baseSlot(B.base.size(), -1);
    std::vector<int> commitFirst(B.commits.size(), -1),
        commitLast(B.commits.size(), -1);
    std::vector<int> frameRecordStep(B.frameRecords.size(), -1);
    const auto record = [&](RigExecBakedSlotDomain domain, uint32_t entry,
                            int step, int slot) {
        std::vector<int> &writer = domain == kFin ? finWriter : baseWriter;
        std::vector<int> &slotOf = domain == kFin ? finSlot : baseSlot;
        const auto what = [&]() {
            return NameStep(B, step) + " writes " +
                   RigExecBakedSlotDomainName(domain) + " version " +
                   std::to_string(entry);
        };
        if (entry >= writer.size()) {
            out->Add(what() + ", past the table's " +
                     std::to_string(writer.size()) + " entries");
        } else if (writer[entry] >= 0) {
            out->Add(what() + ", which " + NameStep(B, writer[entry]) +
                     " writes too");
        } else {
            writer[entry] = step;
            slotOf[entry] = slot;
        }
    };
    for (int index = 0; index < int(B.steps.size()); ++index) {
        const RigExecBakedStep &step = B.steps[size_t(index)];
        // A FrameMatrix step indexes the record table with no "no object"
        // value, so a negative object is out of range like a too-large one.
        if (step.kind == RigExecBakedStepKind::FrameMatrix) {
            if (step.object < 0 ||
                size_t(step.object) >= B.frameRecords.size()) {
                out->Add(NameStep(B, index) + " names frame record " +
                         std::to_string(step.object) + " of " +
                         std::to_string(B.frameRecords.size()));
            } else if (frameRecordStep[size_t(step.object)] >= 0) {
                out->Add(NameStep(B, index) +
                         " evaluates the same frame record as " +
                         NameStep(B, frameRecordStep[size_t(step.object)]));
            } else {
                frameRecordStep[size_t(step.object)] = index;
            }
            continue;
        }
        if (step.object < 0) {
            continue;
        }
        const size_t object = size_t(step.object);
        if (step.kind == RigExecBakedStepKind::ComposeSubtree &&
            object < B.composeGroups.size()) {
            const RigExecBakedComposeGroup &group = B.composeGroups[object];
            for (int slot = group.begin; slot < group.end; ++slot) {
                record(kFin, uint32_t(slot), index, slot);
                record(kBase, uint32_t(slot), index, slot);
            }
        } else if (IsCommitStep(step.kind) && object < B.commits.size()) {
            if (commitFirst[object] < 0) {
                commitFirst[object] = index;
            }
            commitLast[object] = index;
        }
    }
    for (size_t w = 0; w < B.commits.size(); ++w) {
        const RigExecBakedCommit &commit = B.commits[w];
        const int writer = commitLast[w];
        if (writer < 0) {
            continue;
        }
        for (size_t k = 0; k < commit.slots.size(); ++k) {
            if (k < commit.slotWrites.size()) {
                record(kFin, commit.slotWrites[k], writer, commit.slots[k]);
            }
            if (k < commit.slotBaseWrites.size()) {
                record(kBase, commit.slotBaseWrites[k], writer,
                       commit.slots[k]);
            }
        }
        for (size_t k = 0; k < commit.propagate.size(); ++k) {
            if (k < commit.descendantWrites.size()) {
                record(kFin, commit.descendantWrites[k], writer,
                       commit.propagate[k].first);
            }
            if (k < commit.descendantBaseWrites.size()) {
                record(kBase, commit.descendantBaseWrites[k], writer,
                       commit.propagate[k].first);
            }
        }
    }

    for (int index = 0; index < int(B.steps.size()); ++index) {
        const auto &step = B.steps[size_t(index)];
        if (step.kind == RigExecBakedStepKind::ComposeSubtree || IsCommitStep(step.kind)) continue;
        for (const auto &write : step.writes) {
            if (write.domain != kFin && write.domain != kBase) continue;
            for (uint32_t entry = write.begin; entry < write.end; ++entry)
                record(write.domain, entry, index, -1);
        }
    }
    const auto covers = [](const auto &ranges, RigExecBakedSlotDomain domain,
                           uint32_t entry) {
        return std::any_of(ranges.begin(), ranges.end(), [&](const auto &range) {
            return range.domain == domain && range.begin <= entry && entry < range.end;
        });
    };
    // A carry may name a value written within the same apply body. Every
    // other binding must name its declared SSA input and its unique producer.
    const auto check = [&](RigExecBakedSlotDomain domain, uint32_t entry,
                           int reader, int own) {
        if (reader < 0 || size_t(reader) >= B.steps.size()) return;
        const auto &step = B.steps[size_t(reader)];
        const std::vector<int> &writers = domain == kFin ? finWriter : baseWriter;
        const int writer = entry < writers.size() ? writers[entry] : -1;
        const bool internal = writer == own && own == reader &&
                              covers(step.writes, domain, entry);
        const bool available = writer >= 0 ? writer < reader || internal
                                          : IsDeclaredSeedOrExcluded(B, domain, entry);
        std::string line = NameStep(B, reader) + " is bound to " +
                           RigExecBakedSlotDomainName(domain) + " version " +
                           std::to_string(entry);
        const std::vector<int> &slots = domain == kFin ? finSlot : baseSlot;
        const int slot = entry < slots.size() ? slots[entry] : -1;
        if (slot >= 0 && size_t(slot) < n)
            line += " of " + B.paths[size_t(slot)].GetString();
        if (!available) {
            out->Add(line + (writer < 0 ? std::string(", which no step writes")
                : ", which " + NameStep(B, writer) + " writes at or after it"));
            return;
        }
        if (internal) return;
        if (!covers(step.reads, domain, entry)) {
            out->Add(line + " without declaring it");
            return;
        }
        if (writer >= 0 && !SortedContains(step.preds, writer))
            out->Add(line + " without an edge from its producer " + NameStep(B, writer));
    };

    for (int reader = 0; reader < int(B.steps.size()); ++reader) {
        const auto &step = B.steps[size_t(reader)];
        const auto all = [&](RigExecBakedSlotDomain domain, const auto &values, int own = -1) {
            for (const auto value : values) check(domain, value, reader, own);
        };
        if (step.kind == RigExecBakedStepKind::Solve && step.object >= 0 &&
            size_t(step.object) < B.solvers.size()) {
            const auto &solver = B.solvers[size_t(step.object)];
            all(kFin, solver.controlReads);
            for (const auto binding : {std::make_pair(solver.start, solver.startRead),
                 std::make_pair(solver.root, solver.rootRead),
                 std::make_pair(solver.mid, solver.midRead),
                 std::make_pair(solver.end, solver.endRead),
                 std::make_pair(solver.pole, solver.poleRead),
                 std::make_pair(solver.spaceSlot, uint32_t(solver.spaceRead))})
                if (binding.first >= 0) check(kFin, binding.second, reader, -1);
            for (size_t k = 0; k < solver.restReads.size(); ++k)
                if (k < solver.restIsLive.size() && solver.restIsLive[k])
                    check(kFin, solver.restReads[k], reader, -1);
        } else if (IsCommitStep(step.kind) && step.object >= 0 &&
                   size_t(step.object) < B.commits.size()) {
            const auto &commit = B.commits[size_t(step.object)];
            if (step.kind == RigExecBakedStepKind::Constraint &&
                size_t(step.object) < B.walkSteps.size()) {
                const int constraint = B.walkSteps[size_t(step.object)].index;
                if (constraint >= 0 && size_t(constraint) < B.constraints.size()) {
                    const auto &c = B.constraints[size_t(constraint)];
                    for (size_t k = 0; k < c.sources.size() && k < commit.sourceReads.size(); ++k)
                        if (c.sources[k] >= 0) check(kFin, commit.sourceReads[k], reader, -1);
                    if (c.target >= 0) check(kFin, commit.targetRead, reader, -1);
                    all(kFin, commit.targetReads);
                    if (c.worldUpObject >= 0) check(kFin, commit.worldUpRead, reader, -1);
                    if (c.effector >= 0) check(kFin, commit.effectorRead, reader, -1);
                    for (size_t k = 0; k < c.poleObjects.size() && k < commit.poleReads.size(); ++k)
                        if (c.poleObjects[k] >= 0) check(kFin, commit.poleReads[k], reader, -1);
                    const auto ancestors = [&](const auto &values) {
                        for (const auto &value : values) {
                            check(kFin, value.fin, reader, -1);
                            check(kBase, value.base, reader, -1);
                        }
                    };
                    for (const auto &values : commit.sourceAncestors) ancestors(values);
                    for (const auto &values : commit.poleAncestors) ancestors(values);
                    ancestors(commit.worldUpAncestors); ancestors(commit.effectorAncestors);
                }
            }
            const bool apply = step.kind == RigExecBakedStepKind::CommitApply ||
                (!commit.split && (step.kind == RigExecBakedStepKind::Constraint ||
                                   step.kind == RigExecBakedStepKind::SolverCommit));
            if (step.kind == RigExecBakedStepKind::CommitDelta || (apply && !commit.split))
                all(kFin, commit.slotReads);
            if (step.kind == RigExecBakedStepKind::PropagateChunk || (apply && !commit.split)) {
                const size_t begin = step.kind == RigExecBakedStepKind::PropagateChunk
                    ? size_t(step.part) * 64 : 0;
                const size_t end = step.kind == RigExecBakedStepKind::PropagateChunk
                    ? std::min(begin + 64, commit.propagate.size()) : commit.propagate.size();
                for (size_t k = begin; k < end; ++k) {
                    if (k < commit.descendantReads.size()) check(kFin, commit.descendantReads[k], reader, -1);
                    if (k < commit.closestReads.size()) check(kFin, commit.closestReads[k], reader, -1);
                }
            }
            if (apply) {
                all(kFin, commit.slotCarry); all(kBase, commit.slotBaseCarry);
                const auto descendantCarries = [&](RigExecBakedSlotDomain domain,
                                                   const auto &values, const auto &candidateWrites) {
                    for (size_t k = 0; k < values.size(); ++k) {
                        bool earlierCandidate = false;
                        for (size_t pos = 0; pos < commit.slots.size() && pos < candidateWrites.size(); ++pos)
                            if (k < commit.propagate.size() &&
                                commit.slots[pos] == commit.propagate[k].first &&
                                candidateWrites[pos] == values[k]) earlierCandidate = true;
                        check(domain, values[k], reader, earlierCandidate ? reader : -1);
                    }
                };
                // FinishCommit writes all candidate outcomes, including their
                // fallback copies, before a descendant can carry one of them.
                descendantCarries(kFin, commit.descendantCarry, commit.slotWrites);
                descendantCarries(kBase, commit.descendantBaseCarry, commit.slotBaseWrites);
            }
        } else if (step.kind == RigExecBakedStepKind::ProviderRefresh && step.object >= 0 &&
                   size_t(step.object) < B.providerRefreshes.size()) {
            const auto &refresh = B.providerRefreshes[size_t(step.object)];
            check(kBase, refresh.baseRead, reader, -1);
            check(kFin, refresh.finRead, reader, -1);
            for (const auto &carry : refresh.carries) {
                check(kBase, carry.baseRead, reader, -1);
                check(kFin, carry.finRead, reader, -1);
            }
            for (const auto value : {refresh.baseValue, refresh.currentValue}) {
                if (value >= B.providerValues.values.size() ||
                    !covers(step.reads, RigExecBakedSlotDomain::SpaceValue, uint32_t(value)))
                    out->Add(NameStep(B, reader) + " consumes SpaceValue[" +
                             std::to_string(value) + "] without declaring it");
            }
            for (const auto &prior : refresh.priorConstraints)
                if (!covers(step.reads, RigExecBakedSlotDomain::CommitTable, prior.first))
                    out->Add(NameStep(B, reader) + " inspects commit " +
                             std::to_string(prior.first) + " without declaring it");
        } else if (step.kind == RigExecBakedStepKind::ProviderMatrix && step.object >= 0 &&
                   size_t(step.object) < n) {
            const size_t slot = size_t(step.object);
            if (step.part) check(kFin, B.finLast[slot], reader, -1);
            check(kBase, B.baseLast[slot], reader, -1);
        } else if (step.kind == RigExecBakedStepKind::PoseInterpolator && step.object >= 0 &&
                   size_t(step.object) < B.poseInterpolators.size()) {
            const auto &interp = B.poseInterpolators[size_t(step.object)];
            if (interp.driverSlot >= 0 && size_t(interp.driverSlot) < n)
                check(kFin, B.finLast[size_t(interp.driverSlot)], reader, -1);
            if (interp.parentSlot >= 0 && size_t(interp.parentSlot) < n)
                check(kFin, B.finLast[size_t(interp.parentSlot)], reader, -1);
        }
    }
    // A frame record reads the version its writer left the provider in,
    // which is usually not the slot's last, and its commit's table -- a
    // constraint's exit flags, a solver batch's `present` bytes -- which the
    // commit's first step writes.
    for (size_t r = 0; r < B.frameRecords.size(); ++r) {
        const RigExecBakedFrameRecord &frameRecord = B.frameRecords[r];
        const int reader = frameRecordStep[r];
        if (reader < 0) {
            out->Add("frame record " + std::to_string(r) + " of " +
                     (frameRecord.slot >= 0 && size_t(frameRecord.slot) < n
                          ? B.paths[size_t(frameRecord.slot)].GetString()
                          : std::string("slot ") +
                                std::to_string(frameRecord.slot)) +
                     " after " + frameRecord.mover.GetString() +
                     " has no FrameMatrix step");
            continue;
        }
        if (frameRecord.commit < 0 ||
            size_t(frameRecord.commit) >= B.commits.size()) {
            out->Add(NameStep(B, reader) + " names commit " +
                     std::to_string(frameRecord.commit) + " of " +
                     std::to_string(B.commits.size()));
            continue;
        }
        check(kFin, frameRecord.version, reader, -1);
        const int slot = frameRecord.version < finSlot.size()
                             ? finSlot[frameRecord.version]
                             : -1;
        if (slot >= 0 && slot != frameRecord.slot) {
            out->Add(NameStep(B, reader) + " is bound to PoseFin version " +
                     std::to_string(frameRecord.version) + " of " +
                     B.paths[size_t(slot)].GetString() +
                     ", not of the provider it records");
        }
        const RigExecBakedCommit &recordCommit =
            B.commits[size_t(frameRecord.commit)];
        if (recordCommit.solverOutput &&
            (frameRecord.position < 0 ||
             size_t(frameRecord.position) >= recordCommit.slots.size() ||
             recordCommit.slots[size_t(frameRecord.position)] !=
                 frameRecord.slot)) {
            out->Add(NameStep(B, reader) + " reads position " +
                     std::to_string(frameRecord.position) + " of commit " +
                     std::to_string(frameRecord.commit) +
                     ", which is not the provider it records");
        }
        const int head = commitFirst[size_t(frameRecord.commit)];
        if (head < 0 && IsDeclaredSeedOrExcluded(B, RigExecBakedSlotDomain::CommitTable,
                                              uint32_t(frameRecord.commit))) continue;
        if (head < 0 || head >= reader) {
            out->Add(NameStep(B, reader) + " reads the exit of commit " +
                     std::to_string(frameRecord.commit) + ", which " +
                     (head < 0 ? std::string("no step writes")
                               : NameStep(B, head) + " writes at or after it"));
        } else if (!covers(B.steps[size_t(reader)].reads,
                           RigExecBakedSlotDomain::CommitTable, uint32_t(frameRecord.commit)) ||
                   !SortedContains(B.steps[size_t(reader)].preds, head)) {
            out->Add(NameStep(B, reader) + " reads the exit of commit " +
                     std::to_string(frameRecord.commit) +
                     " without its declared value and producer edge");
        }
    }
    // Declarations already contain exact SSA identities. A version number
    // must never be reinterpreted as a provider slot and mapped through last.
    for (int index = 0; index < int(B.steps.size()); ++index) {
        for (const auto &read : B.steps[size_t(index)].reads) {
            if (read.domain != kFin && read.domain != kBase) continue;
            for (uint32_t entry = read.begin; entry < read.end; ++entry)
                check(read.domain, entry, index, -1);
        }
    }

}

/// The clusters partition the steps with members in program order, their
/// edges are mutual inverses covering every step edge between two clusters,
/// and the topological order names each cluster once, after its
/// predecessors.
void
ValidateClusters(const RigExecBakedProgramImpl &B, GraphViolations *out)
{
    const RigExecBakedClustering &C = B.clustering;
    const int steps = int(B.steps.size());
    const int clusters = int(C.clusters.size());
    const auto name = [](int c) { return "cluster " + std::to_string(c); };
    if (C.clusterOf.size() != B.steps.size()) {
        out->Add("the clustering assigns " +
                 std::to_string(C.clusterOf.size()) + " of " +
                 std::to_string(steps) + " steps");
        return;
    }
    for (int index = 0; index < steps; ++index) {
        const int cluster = C.clusterOf[size_t(index)];
        if (cluster < 0 || cluster >= clusters ||
            B.steps[size_t(index)].cluster != cluster) {
            out->Add(NameStep(B, index) + " is in " +
                     name(B.steps[size_t(index)].cluster) +
                     " but the clustering puts it in " + name(cluster));
            return;
        }
    }
    size_t members = 0;
    for (int c = 0; c < clusters; ++c) {
        const RigExecBakedCluster &cluster = C.clusters[size_t(c)];
        members += cluster.members.size();
        if (!SortedUnique(cluster.members)) {
            out->Add(name(c) + " does not hold its members in program order");
        }
        for (const int member : cluster.members) {
            if (member < 0 || member >= steps ||
                C.clusterOf[size_t(member)] != c) {
                out->Add(name(c) + " holds " + NameStep(B, member) +
                         ", which the clustering puts elsewhere");
            }
        }
        if (!SortedUnique(cluster.preds) || !SortedUnique(cluster.succs)) {
            out->Add(name(c) + " lists an edge out of order or twice");
            continue;
        }
        for (const int pred : cluster.preds) {
            if (pred < 0 || pred >= clusters || pred == c ||
                !SortedContains(C.clusters[size_t(pred)].succs, c)) {
                out->Add(name(c) + " depends on " + name(pred) +
                         ", which does not list it as a successor");
            }
        }
        for (const int succ : cluster.succs) {
            if (succ < 0 || succ >= clusters || succ == c ||
                !SortedContains(C.clusters[size_t(succ)].preds, c)) {
                out->Add(name(c) + " names successor " + name(succ) +
                         ", which does not depend on it");
            }
        }
    }
    if (members != B.steps.size()) {
        out->Add("the clusters hold " + std::to_string(members) +
                 " members for " + std::to_string(steps) + " steps");
    }
    for (int index = 0; index < steps; ++index) {
        const int to = C.clusterOf[size_t(index)];
        for (const int pred : B.steps[size_t(index)].preds) {
            if (pred < 0 || pred >= steps) {
                continue;  // ValidateStepEdges reports it.
            }
            const int from = C.clusterOf[size_t(pred)];
            if (from != to &&
                !SortedContains(C.clusters[size_t(to)].preds, from)) {
                out->Add(NameStep(B, index) + " depends on " +
                         NameStep(B, pred) + ", but " + name(to) +
                         " does not depend on " + name(from));
            }
        }
    }
    std::vector<int> position(size_t(clusters), -1);
    for (size_t k = 0; k < C.topologicalOrder.size(); ++k) {
        const int c = C.topologicalOrder[k];
        if (c < 0 || c >= clusters || position[size_t(c)] >= 0) {
            out->Add("the cluster order names " + name(c) +
                     " out of range or twice");
            return;
        }
        position[size_t(c)] = int(k);
    }
    if (C.topologicalOrder.size() != C.clusters.size()) {
        out->Add("the cluster order holds " +
                 std::to_string(C.topologicalOrder.size()) + " of " +
                 std::to_string(clusters) + " clusters");
        return;
    }
    for (int c = 0; c < clusters; ++c) {
        for (const int pred : C.clusters[size_t(c)].preds) {
            if (pred >= 0 && pred < clusters &&
                position[size_t(pred)] > position[size_t(c)]) {
                out->Add("the cluster order puts " + name(c) +
                         " before its predecessor " + name(pred));
            }
        }
    }
}

/// Every point-chain reader declares the version it reads, and that
/// version's producer precedes it. Version v > 0 of a chain is addressed by
/// RevisionDone and ChainDirty of revision first + v - 1, and the only writer
/// of either is that revision's fuse (revisionFuseStep). The declaration is
/// checked from the revision tables rather than from the sweep, so a version
/// read the builder dropped is reported even though no edge is missing for it.
/// In a range chain a group step reads exactly group `part` of the entering
/// version (its writer's RevisionOut slot, or ChainBase) and no version; a
/// join reads its written groups and the version; a Whole fuse publishes and
/// reads every group besides the version. Each RevisionOut slot has one
/// writer, and a slot nobody writes (a gated group) has no reader.
void
ValidatePointVersions(const RigExecBakedProgramImpl &B, GraphViolations *out)
{
    const size_t revisions = B.revisionIndex.size();
    if (B.revisionFuseStep.size() != revisions) {
        out->Add("the program records " +
                 std::to_string(B.revisionFuseStep.size()) +
                 " fuse steps for " + std::to_string(revisions) +
                 " revisions");
        return;
    }
    const int count = int(B.steps.size());
    for (size_t r = 0; r < revisions; ++r) {
        const int fuse = B.revisionFuseStep[r];
        if (fuse < 0 || fuse >= count ||
            B.steps[size_t(fuse)].kind != RigExecBakedStepKind::RevisionFuse ||
            B.steps[size_t(fuse)].object != int(r)) {
            out->Add("revision " + std::to_string(r) + "'s fuse is recorded "
                     "as " + NameStep(B, fuse) + ", which is not its fuse");
            return;
        }
    }
    const auto chainDomain = [](RigExecBakedSlotDomain domain) {
        return domain == RigExecBakedSlotDomain::RevisionDone ||
               domain == RigExecBakedSlotDomain::ChainDirty;
    };
    const auto covers = [](const std::vector<RigExecBakedSlotRange> &ranges,
                           RigExecBakedSlotDomain domain, int slot) {
        for (const RigExecBakedSlotRange &range : ranges) {
            if (range.domain == domain && range.begin <= uint32_t(slot) &&
                uint32_t(slot) < range.end) {
                return true;
            }
        }
        return false;
    };
    for (int index = 0; index < count; ++index) {
        const RigExecBakedStep &step = B.steps[size_t(index)];
        // The producer: one writer per version slot, the revision's fuse.
        for (const RigExecBakedSlotRange &write : step.writes) {
            if (!chainDomain(write.domain)) {
                continue;
            }
            if (!write.IsEmpty() && write.end > revisions) {
                out->Add(NameStep(B, index) + " writes " +
                         RigExecBakedSlotDomainName(write.domain) + "[" +
                         std::to_string(write.end - 1) + "], past the " +
                         "table's " + std::to_string(revisions) +
                         " revisions");
            }
            for (uint32_t slot = write.begin;
                 slot < write.end && slot < revisions; ++slot) {
                if (B.revisionFuseStep[slot] != index) {
                    out->Add(NameStep(B, index) + " writes " +
                             RigExecBakedSlotDomainName(write.domain) + "[" +
                             std::to_string(slot) + "], which only " +
                             NameStep(B, B.revisionFuseStep[slot]) +
                             " may write");
                }
            }
        }
        // Every reader of a version slot is ordered after its fuse.
        for (const RigExecBakedSlotRange &read : step.reads) {
            if (!chainDomain(read.domain)) {
                continue;
            }
            if (!read.IsEmpty() && read.end > revisions) {
                out->Add(NameStep(B, index) + " reads " +
                         RigExecBakedSlotDomainName(read.domain) + "[" +
                         std::to_string(read.end - 1) + "], past the " +
                         "table's " + std::to_string(revisions) +
                         " revisions");
            }
            for (uint32_t slot = read.begin;
                 slot < read.end && slot < revisions; ++slot) {
                const int fuse = B.revisionFuseStep[slot];
                if (!SortedContains(step.preds, fuse)) {
                    out->Add(NameStep(B, index) + " reads " +
                             RigExecBakedSlotDomainName(read.domain) + "[" +
                             std::to_string(slot) + "] without an edge from "
                             "its producer " + NameStep(B, fuse));
                }
            }
        }
        // Every point-binding candidate of the revision a step assembles (or
        // of a derived step's revision) is a slot that step declares: the
        // version's RevisionDone and ChainDirty, ChainBase for version 0, and
        // ChainPoints for a final read. That covers an own-chain `preceding`
        // binding, which reads version r without a current-phase field.
        const RigExecBakedProgramImpl::GeomRevision *bound = nullptr;
        if (step.kind == RigExecBakedStepKind::RevisionStatic &&
            step.object >= 0 && size_t(step.object) < revisions) {
            const auto &[chain, r] = B.revisionIndex[size_t(step.object)];
            bound = &B.chains[size_t(chain)].revisions[size_t(r)];
        } else if (step.kind == RigExecBakedStepKind::Derived &&
                   step.object >= 0 &&
                   size_t(step.object) < B.derivedIndex.size()) {
            const auto &[chain, d] = B.derivedIndex[size_t(step.object)];
            bound = &B.chains[size_t(chain)].derived[size_t(d)].revision;
        }
        if (bound) {
            const auto checkBinding =
                [&](const RigExecBakedPointsBinding &binding) {
                for (const RigExecBakedPointVersion &candidate :
                         binding.candidates) {
                    const std::string what =
                        " point version " + std::to_string(candidate.version) +
                        " of chain " + std::to_string(candidate.chain);
                    if (candidate.chain < 0 ||
                        size_t(candidate.chain) >=
                            B.chainRevisionBegin.size()) {
                        out->Add(NameStep(B, index) + " binds" + what +
                                 ", which is not a chain");
                        continue;
                    }
                    const int first =
                        B.chainRevisionBegin[size_t(candidate.chain)];
                    const int last =
                        B.chainRevisionEnd[size_t(candidate.chain)];
                    bool declared = false;
                    if (binding.finalRead) {
                        declared = covers(step.reads,
                                          RigExecBakedSlotDomain::ChainPoints,
                                          candidate.chain);
                    } else if (candidate.version == 0) {
                        declared = covers(step.reads,
                                          RigExecBakedSlotDomain::ChainBase,
                                          candidate.chain);
                    } else if (candidate.version < 0 ||
                               first + candidate.version - 1 >= last) {
                        out->Add(NameStep(B, index) + " binds" + what +
                                 ", past the chain's " +
                                 std::to_string(last - first) +
                                 " revisions");
                        continue;
                    } else {
                        const int slot = first + candidate.version - 1;
                        declared =
                            covers(step.reads,
                                   RigExecBakedSlotDomain::RevisionDone,
                                   slot) &&
                            covers(step.reads,
                                   RigExecBakedSlotDomain::ChainDirty, slot);
                    }
                    if (!declared) {
                        out->Add(NameStep(B, index) + " binds" + what +
                                 " without declaring it");
                    }
                }
            };
            for (const RigExecBakedPointsBinding &binding :
                     bound->pointBindings) {
                checkBinding(binding);
            }
            for (const RigExecBakedProgramImpl::GeomBlendChannel &channel :
                     bound->blendChannels) {
                for (const auto &sample : channel.samples) {
                    checkBinding(sample.pointBinding);
                }
            }
        }
        // The declaration: the steps that read the points entering revision
        // r name version r.
        int enteringRevision = -1;
        if (step.kind == RigExecBakedStepKind::RevisionChunk ||
            step.kind == RigExecBakedStepKind::RevisionFuse)
            enteringRevision = step.object;
        else if (step.kind == RigExecBakedStepKind::WeightField && step.object >= 0 &&
                 size_t(step.object) < B.weightFields.size() &&
                 B.weightFields[size_t(step.object)].form == RigExecBakedProgramImpl::WeightField::Form::Revision)
            enteringRevision = B.weightFields[size_t(step.object)].consumer;
        else if (step.kind == RigExecBakedStepKind::RevisionStatic && step.object >= 0 &&
                 size_t(step.object) < revisions) {
            const auto [chain,r] = B.revisionIndex[size_t(step.object)];
            const auto &revision = B.chains[size_t(chain)].revisions[size_t(r)];
            // Assembly adopts the field; the field owns the entering points.
            if (revision.weightCurrentPhase && revision.weightObject >= 0) {
                const int field = revision.weightField;
                if (field < 0 || size_t(field) >= B.weightFields.size() ||
                    B.weightFields[size_t(field)].form != RigExecBakedProgramImpl::WeightField::Form::Revision ||
                    B.weightFields[size_t(field)].consumer != step.object ||
                    B.weightFields[size_t(field)].object != revision.weightObject)
                    out->Add(NameStep(B,index) + " has no matching current-phase WeightField");
            }
            if (revision.weightField >= 0 &&
                !covers(step.reads,RigExecBakedSlotDomain::WeightField,revision.weightField))
                out->Add(NameStep(B,index) + " adopts its WeightField without declaring it");
        }
        if (enteringRevision < 0 || size_t(enteringRevision) >= revisions) continue;
        const auto &[chain, r] = B.revisionIndex[size_t(enteringRevision)];
        const RigExecBakedProgramImpl::GeomRevision &own =
            B.chains[size_t(chain)].revisions[size_t(r)];
        const bool groupStep =
            own.role != RigExecBakedRevisionRole::Legacy &&
            (step.kind == RigExecBakedStepKind::RevisionChunk ||
             step.kind == RigExecBakedStepKind::RevisionFuse);
        if (groupStep) {
            if (B.revisionChunkBase.size() != revisions ||
                B.revisionChunkCount.size() != revisions) {
                out->Add(NameStep(B, index) + " belongs to a range-chain "
                         "revision of a program without its chunk tables");
                continue;
            }
            const int id = enteringRevision;
            const int base = B.revisionChunkBase[size_t(id)];
            const size_t groups = own.groupWritten.size();
            const bool range = own.role == RigExecBakedRevisionRole::Range;
            const bool fuse = step.kind == RigExecBakedStepKind::RevisionFuse;
            const std::string version = " of point version " +
                                        std::to_string(r) + " of chain " +
                                        std::to_string(chain);
            // Group g of the version entering the revision: its last
            // writer's slot, or -1 for the base. (A lambda may not capture
            // a structured binding in C++17, so the chain is copied.)
            const int chainIndex = chain;
            const auto groupSlot = [&](size_t g) {
                const int writer = g < own.enteringWriter.size()
                                       ? own.enteringWriter[g]
                                       : -1;
                return writer < 0
                           ? -1
                           : RigExecBakedGroupSlot(
                                 B,
                                 B.chainRevisionBegin[size_t(chainIndex)] +
                                     writer,
                                 g);
            };
            const auto readsGroup = [&](size_t g) {
                const int slot = groupSlot(g);
                return slot < 0
                           ? covers(step.reads, RigExecBakedSlotDomain::ChainBase,
                                    chainIndex)
                           : covers(step.reads,
                                    RigExecBakedSlotDomain::RevisionOut, slot);
            };
            if (!fuse && (range || own.chunked)) {
                // A group step: group `part` of the entering version and
                // nothing else of it.
                const size_t g = size_t(step.part);
                if (step.part < 0 || g >= groups ||
                    (range && !own.groupWritten[g])) {
                    out->Add(NameStep(B, index) + " is group " +
                             std::to_string(step.part) + " of revision " +
                             std::to_string(id) + ", which writes no such "
                             "group");
                    continue;
                }
                if (!covers(step.writes, RigExecBakedSlotDomain::RevisionOut,
                            base + int(g))) {
                    out->Add(NameStep(B, index) + " does not write group " +
                             std::to_string(g) + " of revision " +
                             std::to_string(id) + " (RevisionOut[" +
                             std::to_string(base + int(g)) + "])");
                }
                if (!readsGroup(g)) {
                    out->Add(NameStep(B, index) + " reads group " +
                             std::to_string(g) + version +
                             " without declaring its writer's slot");
                }
                const int expected = groupSlot(g);
                for (const RigExecBakedSlotRange &read : step.reads) {
                    if (read.domain != RigExecBakedSlotDomain::RevisionOut) {
                        continue;
                    }
                    for (uint32_t slot = read.begin;
                         slot < read.end && slot < B.chunkRevision.size();
                         ++slot) {
                        if (int(slot) != expected) {
                            out->Add(NameStep(B, index) + " reads RevisionOut[" +
                                     std::to_string(slot) + "], which is not "
                                     "group " + std::to_string(g) + version);
                        }
                    }
                }
                if (r > 0 &&
                    (covers(step.reads, RigExecBakedSlotDomain::RevisionDone,
                            id - 1) ||
                     covers(step.reads, RigExecBakedSlotDomain::ChainDirty,
                            id - 1))) {
                    out->Add(NameStep(B, index) + " reads point version " +
                             std::to_string(r) + " of chain " +
                             std::to_string(chain) + ", which a group step "
                             "must not wait for");
                }
                continue;
            }
            if (fuse && range) {
                // A join: its written groups, and below, like a fuse, the
                // entering version, which keeps the joins in chain order.
                for (size_t g = 0; g < groups; ++g) {
                    if (own.groupWritten[g] &&
                        !covers(step.reads, RigExecBakedSlotDomain::RevisionOut,
                                base + int(g))) {
                        out->Add(NameStep(B, index) + " joins revision " +
                                 std::to_string(id) + " without declaring "
                                 "RevisionOut[" + std::to_string(base + int(g)) +
                                 "]");
                    }
                }
                for (const RigExecBakedSlotRange &write : step.writes) {
                    if (write.domain == RigExecBakedSlotDomain::RevisionOut &&
                        !write.IsEmpty()) {
                        out->Add(NameStep(B, index) + " joins revision " +
                                 std::to_string(id) + " and writes "
                                 "RevisionOut[" + std::to_string(write.begin) +
                                 "]");
                    }
                }
            } else if (fuse) {
                // A Whole fuse publishes every group and reads every
                // entering group it may pass through.
                const int published = base + int(own.chunks.size());
                for (size_t g = 0; g < groups; ++g) {
                    if (!covers(step.writes, RigExecBakedSlotDomain::RevisionOut,
                                published + int(g))) {
                        out->Add(NameStep(B, index) + " does not publish group " +
                                 std::to_string(g) + " of revision " +
                                 std::to_string(id) + " (RevisionOut[" +
                                 std::to_string(published + int(g)) + "])");
                    }
                    if (!readsGroup(g)) {
                        out->Add(NameStep(B, index) + " reads group " +
                                 std::to_string(g) + version +
                                 " without declaring its writer's slot");
                    }
                }
            }
            // A join, a Whole fuse and a Whole one-chunk op read the version.
        }
        // Version 0 is the source ChainBase, which every chain step reads
        // whether or not it reads points, so there is nothing to check.
        if (r == 0) {
            continue;
        }
        const bool declared =
            covers(step.reads, RigExecBakedSlotDomain::RevisionDone,
                   enteringRevision - 1) &&
            covers(step.reads, RigExecBakedSlotDomain::ChainDirty,
                   enteringRevision - 1);
        if (!declared) {
            out->Add(NameStep(B, index) + " reads point version " +
                     std::to_string(r) + " of chain " +
                     std::to_string(chain) + " without declaring it");
        }
    }
    // One writer per RevisionOut slot, and a reader only of a written one.
    const size_t outSlots = B.chunkRevision.size();
    std::vector<int> outWriter(outSlots, -1);
    for (int index = 0; index < count; ++index) {
        for (const RigExecBakedSlotRange &write : B.steps[size_t(index)].writes) {
            if (write.domain != RigExecBakedSlotDomain::RevisionOut) {
                continue;
            }
            for (uint32_t slot = write.begin; slot < write.end && slot < outSlots;
                 ++slot) {
                if (outWriter[slot] >= 0 && outWriter[slot] != index) {
                    out->Add(NameStep(B, index) + " writes RevisionOut[" +
                             std::to_string(slot) + "], which " +
                             NameStep(B, outWriter[slot]) + " writes too");
                }
                outWriter[slot] = index;
            }
        }
    }
    for (int index = 0; index < count; ++index) {
        for (const RigExecBakedSlotRange &read : B.steps[size_t(index)].reads) {
            if (read.domain != RigExecBakedSlotDomain::RevisionOut) {
                continue;
            }
            for (uint32_t slot = read.begin; slot < read.end && slot < outSlots;
                 ++slot) {
                if (outWriter[slot] < 0) {
                    out->Add(NameStep(B, index) + " reads RevisionOut[" +
                             std::to_string(slot) + "], which no step writes");
                }
            }
        }
    }
}

}  // namespace

bool
RigExecBakedValidateStepGraph(const RigExecBakedProgramImpl &program,
                              std::string *error)
{
    GraphViolations violations;
    ValidateStepEdges(program, &violations);
    ValidateSemanticRequirements(program, &violations);
    ValidateSlotProducers(program, &violations);
    const auto admissionReads = RigExecBakedExpectedStageFramesAdmissionReads(program);
    for (size_t i = 0; i < program.steps.size(); ++i) {
        const auto count = std::count_if(program.steps[i].reads.begin(), program.steps[i].reads.end(),
            [](const auto &range) { return range.domain == RigExecBakedSlotDomain::RequiredStageFramesAdmission; });
        if (size_t(count) != size_t(bool(admissionReads[i])))
            violations.Add(NameStep(program, int(i)) + " differs from its required stage-frame admission role");
    }
    std::string headError;
    if (!RigExecBakedValidateHeadTier(program,&headError)) violations.Add(headError);
    if (!RigExecBakedValidateHeadReads(program,&headError)) violations.Add(headError);
    ValidatePoseVersions(program, &violations);
    ValidatePointVersions(program, &violations);
    ValidateClusters(program, &violations);
    if (violations.count == 0) {
        return true;
    }
    if (error) {
        *error = violations.first;
        if (violations.count > 1) {
            *error += " (and " + std::to_string(violations.count - 1) +
                      " more)";
        }
    }
    return false;
}

// Common execution follows operation successors when values change. The
// retained cluster closure serves output queries and sparse cache planning;
// clusters containing clean operations do not make those operations dirty.
// There used to be a second, the restore closure: "run c and all of THIS had
// to have run first, because c reads a slot version the end of a run does not
// hold". Versioned pose storage (§3.1) retired it. Every writer writes its
// own entry, so the version a clean reader wants is exactly where its writer
// left it however often the SLOT was revised afterwards, and nothing ever has
// to be re-run to put a value back.
// One rule decides what starts the closure: a SOURCE -- the avar table, a
// chain's base points, a skin revision's static packet, the property-chain
// results -- always runs, and its output is compared with the last run's by
// VALUE. Never "the time changed", never "an override stands": those two
// predicates each miss a case that reaches the graph anyway (an override on
// a routed prim authors no flag, a released drag leaves the table disturbed,
// a cleared layout cache changes a packet), and a value comparison misses
// none of them.

std::vector<int>
RigExecBakedClusterTopologicalOrder(const RigExecBakedClustering &clustering)
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

void
RigExecBakedBuildCones(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    RigExecBakedCones &cones = B.cones;
    const auto resolvedReaders = RigExecBakedResolvedReaders(B);
    const size_t count = B.clustering.clusters.size();
    const size_t stepCount = B.steps.size();
    cones = RigExecBakedCones();
    B.closed.Resize(count);
    B.closedSteps.Resize(stepCount);
    // Cached for cone-report traversal over the common cluster artifact.
    B.clustering.topologicalOrder =
        RigExecBakedClusterTopologicalOrder(B.clustering);
    if (count == 0) {
        return;
    }
    cones.cone.resize(count);
    cones.always.Resize(count);
    cones.poseClusters.Resize(count);
    for (size_t c = 0; c < count; ++c) {
        cones.cone[c].Resize(count);
    }
    cones.alwaysSteps.Resize(stepCount);
    cones.poseSteps.Resize(stepCount);

    // Cone reports mirror the compiled graph. They do not classify or
    // execute a separate source pass.
    for (RigExecBakedStep &step : B.steps) {
        const int index = int(&step - B.steps.data());
        if (step.externalReads) {
            cones.always.Set(step.cluster);
            cones.alwaysSteps.Set(index);
        }
        if (!RigExecBakedIsGeometryStep(step.kind)) {
            cones.poseClusters.Set(step.cluster);
            cones.poseSteps.Set(index);
        }
        if (step.varyingInputs || resolvedReaders[size_t(&step - B.steps.data())]) {
            cones.varyingSteps.push_back(int(&step - B.steps.data()));
        }
        if (!step.overrideInputs.empty()) {
            cones.overrideSteps.push_back(int(&step - B.steps.data()));
        }
    }

    // An edit on a per-frame input is routed like the run after a drag is
    // lifted (unified-program spec rule S2), so it is only as good as the
    // reader it reaches. Three readers qualify: a step that declares the
    // input (the closure's override rule re-runs it), the avar table (the
    // prologue re-reads every varying binding and compares the table by
    // value), and the rest and ladder ops, which re-run when the channel's
    // leaf moved and seed the readers of what they moved.
    // An index with none of the three is answered by a stamp bump.
    cones.editRoute.assign(B.overridden.size(), 0);
    const auto route = [&cones](int index, uint8_t reader) {
        if (index >= 0 && size_t(index) < cones.editRoute.size()) {
            cones.editRoute[size_t(index)] |= reader;
        }
    };
    for (const RigExecBakedStep &step : B.steps) {
        for (const int input : step.overrideInputs) {
            route(input, kEditRouteStep);
        }
    }
    for (const auto *bindings : {&B.avarBindings, &B.avarConstantBindings}) {
        for (const RigExecBakedProgramImpl::AvarBinding &binding : *bindings) {
            route(binding.input.overrideIndex, kEditRouteAvar);
        }
    }
    for (const int input : B.ladderOverrides) {
        route(input, kEditRouteHead);
    }

    cones.avarCluster.assign(B.paths.size(), -1);
    cones.chainBaseClusters.assign(B.chains.size(), {});
    cones.solverPointsClusters.assign(B.solvers.size(), {});
    cones.revisionClusters.assign(B.revisionIndex.size(), {});
    cones.revisionStaticCluster.assign(B.revisionIndex.size(), -1);
    cones.avarStep.assign(B.paths.size(), -1);
    cones.avarVersionSteps.assign(B.paths.size(), {});
    cones.chainBaseSteps.assign(B.chains.size(), {});
    cones.solverPointsSteps.assign(B.solvers.size(), {});
    cones.revisionSteps.assign(B.revisionIndex.size(), {});
    cones.revisionStaticStep.assign(B.revisionIndex.size(), -1);
    for (const RigExecBakedStep &step : B.steps) {
        const int index = int(&step - B.steps.data());
        if (step.kind == RigExecBakedStepKind::AvarInputs) {
            cones.avarCluster[size_t(step.object)] = step.cluster;
            cones.avarStep[size_t(step.object)] = index;
        }
        if (step.kind == RigExecBakedStepKind::ComposeSubtree) {
            const RigExecBakedComposeGroup &group =
                B.composeGroups[size_t(step.object)];
            // Avars this step declares OUTSIDE its group: the slots a switch
            // recomposes an earlier version of.
            for (const RigExecBakedSlotRange &range : step.reads) {
                if (range.domain != RigExecBakedSlotDomain::Avars) {
                    continue;
                }
                for (uint32_t slot = range.begin;
                     slot < range.end &&
                     slot < cones.avarVersionSteps.size();
                     ++slot) {
                    if (int(slot) < group.begin || int(slot) >= group.end) {
                        cones.avarVersionSteps[slot].push_back(index);
                    }
                }
            }
        }
        for (const RigExecBakedSlotRange &range : step.reads) {
            if (range.domain == RigExecBakedSlotDomain::ChainBase) {
                for (uint32_t c = range.begin;
                     c < range.end && c < cones.chainBaseClusters.size();
                     ++c) {
                    cones.chainBaseClusters[c].push_back(step.cluster);
                    cones.chainBaseSteps[c].push_back(index);
                }
            } else if (range.domain ==
                       RigExecBakedSlotDomain::SolverPoints) {
                for (uint32_t si = range.begin;
                     si < range.end && si < cones.solverPointsClusters.size();
                     ++si) {
                    cones.solverPointsClusters[si].push_back(step.cluster);
                    cones.solverPointsSteps[si].push_back(index);
                }
            }
        }
        switch (step.kind) {
        case RigExecBakedStepKind::RevisionStatic:
            cones.revisionStaticCluster[size_t(step.object)] = step.cluster;
            cones.revisionStaticStep[size_t(step.object)] = index;
            [[fallthrough]];
        case RigExecBakedStepKind::InfluenceFold:
        case RigExecBakedStepKind::RevisionChunk:
        case RigExecBakedStepKind::RevisionFuse:
            cones.revisionClusters[size_t(step.object)].push_back(
                step.cluster);
            cones.revisionSteps[size_t(step.object)].push_back(index);
            break;
        default:
            break;
        }
    }
    // The steps a moved property version, a moved rest or ladder slot, or
    // a moved reader walk reaches.
    cones.headReaders.assign(B.propertyVersionCount, {});
    cones.fieldReaders.assign(B.weightFields.size(), {});
    cones.fieldSteps.assign(B.weightFields.size(), -1);
    cones.restReaders.assign(B.paths.size(), {});
    cones.ladderReaders.assign(B.paths.size(), {});
    cones.walkReaders.assign(B.readerWalks.size(), {});
    for (const RigExecBakedStep &step : B.steps) {
        if (step.isHead) continue;
        const int index = int(&step - B.steps.data());
        if (step.kind == RigExecBakedStepKind::WeightField)
            cones.fieldSteps[size_t(step.object)] = index;
        for (const RigExecBakedSlotRange &range : step.reads) {
            std::vector<std::vector<int>> *readers =
                range.domain == RigExecBakedSlotDomain::PropertyResult
                    ? &cones.headReaders
                : range.domain == RigExecBakedSlotDomain::WeightField
                    ? &cones.fieldReaders
                : range.domain == RigExecBakedSlotDomain::Rest
                    ? &cones.restReaders
                : range.domain == RigExecBakedSlotDomain::Ladder
                    ? &cones.ladderReaders
                    : nullptr;
            if (!readers) {
                continue;
            }
            for (uint32_t id = range.begin;
                 id < range.end && id < readers->size(); ++id) {
                (*readers)[id].push_back(index);
            }
        }
        for (const int walk : step.readerWalks) {
            cones.walkReaders[size_t(walk)].push_back(index);
        }
    }
    // Which constraint steps a native source's stage transform reaches, and
    // which one each geometry-domain delta base reaches.
    cones.nativeSourceClusters.assign(B.nativeSources.size(), {});
    cones.deltaBaseClusters.assign(B.deltaBasePaths.size(), {});
    cones.constraintArrayClusters.assign(B.constraintArrays.size(), {});
    cones.nativeSourceSteps.assign(B.nativeSources.size(), {});
    cones.deltaBaseSteps.assign(B.deltaBasePaths.size(), {});
    cones.constraintArraySteps.assign(B.constraintArrays.size(), {});
    for (const RigExecBakedStep &step : B.steps) {
        if (step.kind != RigExecBakedStepKind::Constraint) {
            continue;
        }
        const int index = int(&step - B.steps.data());
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
                cones.nativeSourceSteps[size_t(native)].push_back(index);
            }
        }
        if (constraint.worldUpNative >= 0) {
            cones.nativeSourceClusters[size_t(constraint.worldUpNative)]
                .push_back(step.cluster);
            cones.nativeSourceSteps[size_t(constraint.worldUpNative)]
                .push_back(index);
        }
        if (constraint.deltaBase >= 0) {
            cones.deltaBaseClusters[size_t(constraint.deltaBase)].push_back(
                step.cluster);
            cones.deltaBaseSteps[size_t(constraint.deltaBase)].push_back(
                index);
        }
        if (constraint.arrays >= 0) {
            cones.constraintArrayClusters[size_t(constraint.arrays)]
                .push_back(step.cluster);
            cones.constraintArraySteps[size_t(constraint.arrays)].push_back(
                index);
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

    const std::vector<int> &order = B.clustering.topologicalOrder;
    for (size_t k = order.size(); k-- > 0;) {
        const int cluster = order[k];
        RigExecBakedClusterSet &cone = cones.cone[size_t(cluster)];
        cone.Set(cluster);
        for (const int succ : B.clustering.clusters[size_t(cluster)].succs) {
            cone.Union(cones.cone[size_t(succ)]);
        }
    }


}

// Step-body purity (bodyPurity.h). Namespace-scope and constant-initialised,
// so reading them takes no guard and no lock on any thread.

namespace {

thread_local bool tRigExecInOpBody = false;
thread_local std::atomic<uint64_t> *tRigExecPurityViolations = nullptr;

}  // namespace

bool
RigExecInOpBody()
{
    return tRigExecInOpBody;
}

void
RigExecReportBodyRead()
{
    if (std::atomic<uint64_t> *violations = tRigExecPurityViolations) {
        violations->fetch_add(1, std::memory_order_relaxed);
    }
#ifndef NDEBUG
    TF_VERIFY(false, "rigExec: a step body read the stage or the resolved "
                     "inputs outside a listed volatile read");
#endif
}

RigExecOpBodyScope::RigExecOpBodyScope(std::atomic<uint64_t> *violations)
    : _wasInBody(tRigExecInOpBody)
    , _wasViolations(tRigExecPurityViolations)
{
    tRigExecInOpBody = true;
    tRigExecPurityViolations = violations;
}

RigExecOpBodyScope::~RigExecOpBodyScope()
{
    tRigExecInOpBody = _wasInBody;
    tRigExecPurityViolations = _wasViolations;
}

RigExecVolatileRead::RigExecVolatileRead()
    : _wasInBody(tRigExecInOpBody)
{
    tRigExecInOpBody = false;
}

RigExecVolatileRead::~RigExecVolatileRead()
{
    tRigExecInOpBody = _wasInBody;
}

// The executors.

namespace {

/// Runs one step's body and checks what it produced against what it declared.
void
RunStepBody(RigExecBakedProgramImpl *B, RigExecBakedStep *step,
            UsdTimeCode time)
{
    // Every domain body is dispatched through the shared executor.
    const RigExecOpBodyScope body(
        B->purityAudit ? &B->purityViolations.count : nullptr);
    // A run's output is cleared HERE rather than in the body, so that the
    // clearing is the executor's promise and not something fifteen bodies
    // each have to remember.
    step->BeginRun();
    // A declared semantic input controls this operation's admission. The
    // shared memo/scheduler still executes its ordinary pure outcome; no
    // arithmetic or owned post-refusal state is changed on refusal.
    if (RigExecBakedRequiresStageFramesAdmission(*step) &&
        !B->requiredStageFramesAdmission.admitted) return;
    if (step->isHead) {
        switch (step->kind) {
        case RigExecBakedStepKind::WeightField:
            RigExecBakedRunWeightField(B,step); break;
        case RigExecBakedStepKind::PropertyRevision:
            RigExecBakedRunPropertyStep(B,step,time); break;
        case RigExecBakedStepKind::RestCompose: {
            const auto &g = B->composeGroups[size_t(step->object)];
            RigExecBakedComposeRestRange(B,g.begin,g.end,true); break;
        }
        case RigExecBakedStepKind::LadderCompose: {
            const auto &g = B->composeGroups[size_t(step->object)];
            RigExecBakedComposeLadderRange(B,g.begin,g.end,true); break;
        }
        case RigExecBakedStepKind::SkinTopology: {
            auto *revision = RigExecBakedLayoutRevision(B,size_t(step->object));
            if (revision) {
                const auto before = revision->layoutHandle;
                RigExecBakedRunLayoutOp(*B,revision);
                revision->layoutRan = true;
                revision->layoutOutputChanged = before != revision->layoutHandle;
            }
            break;
        }
        default: TF_VERIFY(false,"invalid head operation"); break;
        }
    } else if (step->kind == RigExecBakedStepKind::SpaceExpression) {
        RigExecBakedRunSpaceOp(B,step);
    } else if (step->kind == RigExecBakedStepKind::SpaceCheckpoint) {
        RigExecBakedRunSpaceCheckpoint(B,step);
    } else if (step->kind == RigExecBakedStepKind::ProviderRefresh) {
        RigExecBakedRunProviderRefresh(B,step);
    } else if (step->kind == RigExecBakedStepKind::AvarInputs) {
        RigExecBakedRunAvarOp(B,step);
    } else if (step->kind == RigExecBakedStepKind::WeightField) {
        RigExecBakedRunWeightField(B,step);
    } else if (step->kind == RigExecBakedStepKind::WeightPacket ||
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

}  // namespace

/// The profiler's microseconds are the right unit for a trace and the wrong
/// one for a cost model: most steps of a biped frame are under a microsecond,
/// and a table fitted from integer microseconds would call all of them free.
uint64_t
RigExecBakedNowNs()
{
    return uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::steady_clock::now().time_since_epoch())
                        .count());
}

namespace {
bool OriginalPreparationKind(RigExecBakedStepKind kind)
{
    using K = RigExecBakedStepKind;
    return kind == K::PropertyRevision || kind == K::RestCompose ||
           kind == K::LadderCompose || kind == K::SkinTopology;
}
}

bool RigExecBakedRequiresStageFramesAdmission(const RigExecBakedStep &step)
{
    for (const auto &r : step.reads)
        if (r.domain == RigExecBakedSlotDomain::RequiredStageFramesAdmission &&
            r.begin == 0 && r.end == 1) return true;
    return false;
}

std::vector<char> RigExecBakedExpectedStageFramesAdmissionReads(const RigExecBakedProgramImpl &program)
{
    const auto *B = &program;
    // Genuine compiler-emitted value edges, never vector order or extra
    // scheduler dependencies. Only pure mixed-role helpers may inherit prep.
    std::map<uint64_t, std::vector<size_t>> writers;
    for (size_t i = 0; i < B->steps.size(); ++i)
        for (const auto &r : B->steps[i].writes)
            for (uint32_t slot = r.begin; slot < r.end; ++slot)
                writers[(uint64_t(r.domain) << 32) | slot].push_back(i);
    std::vector<char> ancestry(B->steps.size(), 0);
    std::vector<size_t> pending;
    for (size_t i = 0; i < B->steps.size(); ++i)
        if (OriginalPreparationKind(B->steps[i].kind)) {
            ancestry[i] = 1; pending.push_back(i);
        }
    while (!pending.empty()) {
        const size_t i = pending.back(); pending.pop_back();
        for (const auto &r : B->steps[i].reads)
            for (uint32_t slot = r.begin; slot < r.end; ++slot) {
                const auto at = writers.find((uint64_t(r.domain) << 32) | slot);
                if (at == writers.end()) continue;
                for (const size_t producer : at->second)
                    if (!ancestry[producer]) {
                        ancestry[producer] = 1; pending.push_back(producer);
                    }
            }
    }
    using K = RigExecBakedStepKind;
    std::vector<char> required(B->steps.size(), 0);
    for (size_t i = 0; i < B->steps.size(); ++i) {
        const auto &step = B->steps[i];
        const bool preparation = OriginalPreparationKind(step.kind) ||
            step.kind == K::AvarInputs ||
            (ancestry[i] && (step.kind == K::SpaceExpression || step.kind == K::WeightField));
        required[i] = !preparation && step.kind != K::SnapshotFinals;
    }
    return required;
}

void RigExecBakedDeclareStageFramesAdmission(RigExecBakedProgramImpl *B)
{
    const auto required = RigExecBakedExpectedStageFramesAdmissionReads(*B);
    for (size_t i = 0; i < B->steps.size(); ++i)
        if (required[i] && !RigExecBakedRequiresStageFramesAdmission(B->steps[i]))
            B->steps[i].reads.push_back({RigExecBakedSlotDomain::RequiredStageFramesAdmission, 0, 1});
}

bool RigExecBakedPublishStageFramesRefusal(RigExecBakedProgramImpl *B,
                                         RigExecRigPose *pose)
{
    const auto &a = B->requiredStageFramesAdmission;
    if (a.admitted) return false;
    if (a.firstBadTarget < 0 || size_t(a.firstBadTarget) >= B->xformSlots.size())
        return false;
    std::vector<const RigExecBakedStep *> lines;
    for (const auto &step : B->steps)
        if (step.kind == RigExecBakedStepKind::PropertyRevision)
            lines.push_back(&step);
    std::sort(lines.begin(), lines.end(), [](const auto *a, const auto *b) {
        return std::make_pair(a->object,a->part) < std::make_pair(b->object,b->part);
    });
    for (const auto *step : lines) {
        // ORIGINAL's pre-refusal property tier replays cached lines;
        // later body diagnostics never become this refused publication.
        pose->diagnostics.insert(pose->diagnostics.end(), step->lines.begin(), step->lines.end());
    }
    for (const auto &entry : B->propertyResults)
        pose->movedProperties.emplace_hint(pose->movedProperties.end(), entry.first, entry.second);
    const int slot = B->xformSlots[size_t(a.firstBadTarget)];
    if (slot < 0 || size_t(slot) >= B->paths.size()) return false;
    pose->diagnostics.push_back("could not resolve constraint target " +
        B->paths[size_t(slot)].GetString() + " relative to the asset root");
    // Caller-owned validity/maps/time remain as supplied, exactly as direct
    // ORIGINAL Run; a fresh output is already invalid.
    if (B->programStamp == B->lastProgramStamp) ++B->programStamp;
    return true;
}

void
RigExecBakedRunStepBody(RigExecBakedProgramImpl *B,
                                RigExecBakedStep *step, UsdTimeCode time,
                                bool profiling)
{
    const bool measuring=B->opAdapter.measuring && !B->measurementSuspended &&
        !B->coldRunExcluded;
    const uint64_t began=profiling ? RigExecProfiler::NowUs() : 0;
    const uint64_t beganNs=measuring ? RigExecBakedNowNs() : 0;
    const size_t index=size_t(step-B->steps.data());
    if(profiling) step->runner=std::this_thread::get_id();
    if(B->execCheckRows) B->execCheckRows->BeforeStep(*B,index);
    RunStepBody(B,step,time);
    if(B->execCheckRows) B->execCheckRows->AfterStep(*B,index);
    if(measuring) {
        step->bodyEndNs=RigExecBakedNowNs();
        step->measuredUs+=double(step->bodyEndNs-beganNs)/1000.0; ++step->measuredRuns;
    }
    if(profiling) { step->startUs=began; step->endUs=RigExecProfiler::NowUs(); }

}

void
RigExecBakedClearRunStamps(RigExecBakedProgramImpl *program)
{
    // Only a listed step can hold a stamp, so a frame's clear costs its
    // candidates rather than the whole program.
    for (const uint32_t index : program->stampedSteps) {
        if (index >= program->steps.size()) continue;
        RigExecBakedStep &step = program->steps[index];
        step.startUs = step.endUs = 0;
        step.memoStartNs = step.memoEndNs = step.bodyEndNs = step.publishEndNs = 0;
    }
    program->stampedSteps.clear();
    auto &execution = program->opExecution;
    std::fill(execution.ran.begin(), execution.ran.end(), char(0));
    std::fill(execution.candidates.begin(), execution.candidates.end(), char(0));
    std::fill(execution.completion.begin(), execution.completion.end(), uint64_t(0));
    execution.executed = execution.skipped = 0;
}

bool
RigExecBakedRunSteps(RigExecBakedProgramImpl *program, UsdTimeCode time, bool force)
{
    return RigExecBakedExecuteOpGraph(program,time,force);
}

// Calibration.

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
    static int framesSeen = 0;
    if (framesSeen >= StepTimingFrames()) {
        return;
    }
    if (++framesSeen < StepTimingFrames()) {
        return;
    }
    const std::string out = RigExecBakedStepTimingTable(*program);
    std::fwrite(out.data(), 1, out.size(), stderr);
}

std::string
RigExecBakedStepTimingTable(const RigExecBakedProgramImpl &B)
{
    const double frames = double(std::max<size_t>(B.timedFrames, 1));
    std::array<double, kStepKindCount> byKind{};
    std::array<size_t, kStepKindCount> runsByKind{};
    std::array<double, kStepKindCount> memoByKind{}, publishByKind{};
    std::array<size_t, kStepKindCount> memoRunsByKind{};
    double steps = 0, memos = 0, publications = 0;
    for (const RigExecBakedStep &step : B.steps) {
        memoByKind[size_t(step.kind)] += step.measuredMemoUs;
        memoRunsByKind[size_t(step.kind)] += step.measuredMemoRuns;
        publishByKind[size_t(step.kind)] += step.measuredPublishUs;
        memos += step.measuredMemoUs;
        publications += step.measuredPublishUs;
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
    // Then what an op holds its thread for around its body: the memo,
    // evaluated for every candidate including one that then skips its body,
    // and the value publication. Summed over threads, so in parallel mode
    // the whole can exceed the region. "per run" marks a body-table line, so
    // no line below contains it.
    std::snprintf(line, sizeof(line),
                  "  op phases: memos %8.1f  publications %8.1f  "
                  "memo+body+publication %8.1f\n",
                  memos / frames, publications / frames,
                  (memos + steps + publications) / frames);
    out += line;
    order.clear();
    for (size_t kind = 0; kind < kStepKindCount; ++kind) {
        if (memoRunsByKind[kind]) {
            order.emplace_back(memoByKind[kind] + publishByKind[kind], kind);
        }
    }
    std::sort(order.begin(), order.end(),
              [](const auto &a, const auto &b) { return a.first > b.first; });
    for (const auto &entry : order) {
        const size_t kind = entry.second;
        std::snprintf(
            line, sizeof(line), "  %-16s memo %8.2f over %zu memo(s)  "
            "publish %8.2f\n",
            RigExecBakedStepKindName(RigExecBakedStepKind(kind)),
            memoByKind[kind] / frames,
            size_t(double(memoRunsByKind[kind]) / frames + 0.5),
            publishByKind[kind] / frames);
        out += line;
    }
    // Last, so that every line above keeps its position: the cold runs the
    // sums leave out.
    if (B.coldFrames) {
        const double cold = double(B.coldFrames);
        std::snprintf(line, sizeof(line),
                      "  cold runs left out: %zu (prologue %.1f  region %.1f  "
                      "epilogue %.1f us/run)\n",
                      B.coldFrames, B.coldPrologueUs / cold,
                      B.coldRegionUs / cold, B.coldEpilogueUs / cold);
        out += line;
    }
    return out;
}

// The report.

namespace {

std::string
StepLabel(const RigExecBakedProgramImpl &B, const RigExecBakedStep &step,
          RigExecPathText &text)
{
    const auto revisionOf = [&B](int id) -> const
        RigExecBakedProgramImpl::GeomRevision & {
        const auto &[chain, revision] = B.revisionIndex[size_t(id)];
        return B.chains[size_t(chain)].revisions[size_t(revision)];
    };
    switch (step.kind) {
    case RigExecBakedStepKind::SpaceCheckpoint:
        return step.object>=0 && size_t(step.object)<B.switchFrameContexts.size()
            ? B.switchFrameContexts[size_t(step.object)].key : "invalid space checkpoint";
    case RigExecBakedStepKind::ProviderRefresh:
        return step.object>=0 && size_t(step.object)<B.providerRefreshes.size()
            ? B.providerRefreshes[size_t(step.object)].key : "invalid provider refresh";
    case RigExecBakedStepKind::AvarInputs:
        return text(B.paths[size_t(step.object)]);
    case RigExecBakedStepKind::ChainInputs:
        return text(B.chains[size_t(step.object)].target);

    case RigExecBakedStepKind::SpaceExpression: {
        RigExecValueId value=RigExecNoProviderValue;
        if(step.object>=0) {
            const size_t object=size_t(step.object);
            if(step.part==0 && object<B.providerProgram.ops.size()) value=B.providerProgram.ops[object].output;
            else if(step.part==1 && object<B.providerProgram.externalInputs.size()) value=B.providerProgram.externalInputs[object].value;
            else if(step.part==2 && object<B.providerProgram.sampled.size()) value=B.providerProgram.sampled[object].value;
            else if(step.part==3 && object<B.providerProgram.routedInputs.size()) value=B.providerProgram.routedInputs[object].value;
            else if(step.part==4 && object<B.providerFrameInputs.size()) value=B.providerFrameInputs[object].value;
        }
        return value<B.providerProgram.valueKeys.size() ? B.providerProgram.valueKeys[size_t(value)] : "invalid provider value";
    }

    case RigExecBakedStepKind::ComposeSubtree:
        return text(
            B.paths[size_t(B.composeGroups[size_t(step.object)].begin)]);
    case RigExecBakedStepKind::Solve:
        return text(B.solvers[size_t(step.object)].path);
    case RigExecBakedStepKind::SolverCommit:
    case RigExecBakedStepKind::Constraint:
    case RigExecBakedStepKind::CommitDelta:
    case RigExecBakedStepKind::PropagateChunk:
    case RigExecBakedStepKind::CommitApply: {
        const RigExecBakedCommit &commit = B.commits[size_t(step.object)];
        return commit.moverPath.IsEmpty()
                   ? "batch " + std::to_string(step.object)
                   : text(commit.moverPath);
    }
    case RigExecBakedStepKind::ProviderMatrix:
        return text(B.paths[size_t(step.object)]) +
               (step.part ? " final" : " base");
    case RigExecBakedStepKind::SnapshotFinals:
        return "every provider";
    case RigExecBakedStepKind::FrameMatrix: {
        // Range-checked: the validator names a step whose record is out of
        // the table.
        if (step.object < 0 ||
            size_t(step.object) >= B.frameRecords.size()) {
            return "record " + std::to_string(step.object);
        }
        const RigExecBakedFrameRecord &record =
            B.frameRecords[size_t(step.object)];
        const std::string provider =
            record.slot >= 0 && size_t(record.slot) < B.paths.size()
                ? text(B.paths[size_t(record.slot)])
                : "slot " + std::to_string(record.slot);
        return provider + " after " + text(record.mover);
    }
    case RigExecBakedStepKind::PoseInterpolator:
        return text(B.poseInterpolators[size_t(step.object)].path);
    case RigExecBakedStepKind::VolumePlacements:
        // part 1 is the per-volume form Build emits; a hand-built whole-map
        // step (part -1) has no slot to name.
        if ((step.part == 1 || step.part == 2) && step.object >= 0 &&
            size_t(step.object) < B.paths.size()) {
            return text(B.paths[size_t(step.object)]) + (step.part == 2 ? " base" : "");
        }
        return "every volume weight";
    case RigExecBakedStepKind::WeightField:
        return text(B.weightObjects[size_t(B.weightFields[size_t(step.object)].object)].path) +
               " form " + std::to_string(int(B.weightFields[size_t(step.object)].form)) +
               " consumer " + std::to_string(B.weightFields[size_t(step.object)].consumer) +
               " part " + std::to_string(B.weightFields[size_t(step.object)].part);
    case RigExecBakedStepKind::WeightPacket:
        return text(B.weightObjects[size_t(step.object)].path);
    case RigExecBakedStepKind::InfluenceFold:
    case RigExecBakedStepKind::RevisionStatic:
    case RigExecBakedStepKind::RevisionChunk:
    case RigExecBakedStepKind::RevisionFuse:
        return text(revisionOf(step.object).moverPath);
    case RigExecBakedStepKind::ChainStatus:
        return text(B.chains[size_t(step.object)].target);
    case RigExecBakedStepKind::Derived: {
        const auto &[chain, derived] = B.derivedIndex[size_t(step.object)];
        return text(B.chains[size_t(chain)].derived[size_t(derived)].target);
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
        out += "  operation body timings were not enabled\n";
        return out;
    }
    uint64_t opened=UINT64_MAX;
    for(const auto &cluster:schedule.clusters)
        if(cluster.startUs) opened=std::min(opened,cluster.startUs);
    for(size_t c=0;c<schedule.clusters.size();++c) {
        const auto &cluster=schedule.clusters[c];
        if(!cluster.startUs) continue;
        size_t executed=0;
        for(int member:cluster.members) executed+=B.opExecution.ran[size_t(member)]!=0;
        out += "  cluster ["+std::to_string(c)+"] start "+
            Fixed(double(cluster.startUs-opened))+"us run "+
            Fixed(double(cluster.endUs-cluster.startUs))+"us "+
            std::to_string(executed)+" operation(s)\n";
    }
    return out;
}

// Profiling.

namespace {

/// Whether RIGEXEC_TRACE_ALL_STEPS asks the replay to keep steps shorter
/// than a microsecond. Read once, on the replaying thread.
bool
TraceAllStepsRequested()
{
    static const bool requested =
        TfGetenvBool("RIGEXEC_TRACE_ALL_STEPS", false);
    return requested;
}

}  // namespace

void
RigExecBakedReplayStepTimings(const RigExecBakedProgramImpl &B)
{
    if (!B.profiler || !B.profiler->IsEnabled()) {
        return;
    }
    const std::vector<RigExecBakedCluster> &clusters = B.clustering.clusters;
    const bool traceAll = TraceAllStepsRequested();
    for (const RigExecBakedStep &step : B.steps) {
        // A step that did not take a whole microsecond is on nobody's
        // critical path, and a biped's graph holds several hundred of them
        // -- so recording each would cost more than the steps did and would
        // bury the events that matter under zero-length ones. The interval
        // is still measured; what is dropped is the report of it, unless
        // RIGEXEC_TRACE_ALL_STEPS asks for every timed step that ran.
        const bool timedRun = B.opExecution.ran[size_t(&step-B.steps.data())] && step.startUs != 0;
        if (step.endUs <= step.startUs && !(traceAll && timedRun)) {
            continue;
        }
        B.profiler->RecordOn(
            step.runner, step.label, "op", step.startUs, step.endUs,
            {{"kind", RigExecBakedStepKindName(step.kind)},
             {"domain", RigExecBakedStepDomainName(step.kind)},
             {"seq", std::to_string(B.opExecution.completion[size_t(&step-B.steps.data())])}});
    }

    // The clusters themselves, one span each, on the row that ran them.
    // Each span covers the actual bodies run by one common cluster task.
    // Skipped clusters carry no interval.
    for (size_t c = 0; c < clusters.size(); ++c) {
        const RigExecBakedCluster &cluster = clusters[c];
        if (cluster.endUs <= cluster.startUs) {
            continue;
        }
        size_t executed=0;
        for(int member:cluster.members) executed+=B.opExecution.ran[size_t(member)]!=0;
        B.profiler->RecordOn(
            cluster.runner, "cluster " + std::to_string(c), "cluster",
            cluster.startUs, cluster.endUs,
            {{"operations", std::to_string(executed)}});
    }
}

// The head tier.

namespace {

constexpr size_t kHeadDomainCount =
    RigExecBakedSlotDomainCount;

// Per head domain, the head step writing each slot (-1 for none); a slot
// with two writers keeps the first, and `*duplicate` names the slot.
std::array<std::vector<int>, kHeadDomainCount>
HeadWriters(const RigExecBakedProgramImpl &B, std::string *duplicate)
{
    std::array<std::vector<int>, kHeadDomainCount> writers;
    for (size_t i = 0; i < B.steps.size(); ++i) {
        if (!B.steps[i].isHead) continue;
        for (const RigExecBakedSlotRange &range : B.steps[i].writes) {
            std::vector<int> &table = writers[size_t(range.domain)];
            if (table.size() < range.end) {
                table.resize(range.end, -1);
            }
            for (uint32_t slot = range.begin; slot < range.end; ++slot) {
                if (table[slot] >= 0) {
                    if (duplicate && duplicate->empty()) {
                        *duplicate =
                            std::string("head ") +
                            RigExecBakedSlotDomainName(range.domain) +
                            " slot " + std::to_string(slot) +
                            " has more than one head producer";
                    }
                    continue;
                }
                table[slot] = int(i);
            }
        }
    }
    return writers;
}

}  // namespace

bool
RigExecBakedValidateHeadTier(const RigExecBakedProgramImpl &B,
                             std::string *error)
{
    const auto heads = RigExecBakedHeadIndices(B);
    size_t count = 0;
    std::string first;
    const auto add = [&](const std::string &violation) {
        if (count++ == 0) {
            first = violation;
        }
    };
    for (const auto &step : B.steps) {
        if (step.kind == RigExecBakedStepKind::WeightField) {
            if (step.object < 0 || size_t(step.object) >= B.weightFields.size()) {
                add("WeightField step " + step.label + " names an invalid field");
                continue;
            }
            const auto &field = B.weightFields[size_t(step.object)];
            if (field.object < 0 || size_t(field.object) >= B.weightObjects.size())
                add("WeightField step " + step.label + " names an invalid weight object");
            if (field.scalarReads.size() != field.scalarObjects.size() ||
                field.scalarReads.size() != field.scalarMembers.size())
                add("WeightField step " + step.label + " has inconsistent scalar provenance");
            const auto own = RigExecBakedOne(RigExecBakedSlotDomain::WeightField,step.object);
            if (step.writes.size() != 1 || !(step.writes[0] == own))
                add("WeightField step " + step.label + " does not write exactly its field");
            if (field.form == RigExecBakedProgramImpl::WeightField::Form::EnvelopeProperty) {
                if (field.consumer < 0 || size_t(field.consumer) >= B.propertyChains.size() ||
                    field.part <= 0 || size_t(field.part) > B.propertyChains[size_t(field.consumer)].revisions.size() ||
                    B.propertyChains[size_t(field.consumer)].revisions[size_t(field.part-1)].weightField != step.object)
                    add("WeightField step " + step.label + " has no matching property consumer");
            } else if (field.form == RigExecBakedProgramImpl::WeightField::Form::EnvelopeConstraint) {
                if (field.consumer < 0 || size_t(field.consumer) >= B.walkSteps.size()) {
                    add("WeightField step " + step.label + " names an invalid constraint consumer");
                } else {
                    const auto &walk = B.walkSteps[size_t(field.consumer)];
                    if (walk.solverBatch || walk.index < 0 || size_t(walk.index) >= B.constraints.size() ||
                        B.constraints[size_t(walk.index)].weightField != step.object)
                        add("WeightField step " + step.label + " has no matching constraint consumer");
                }
            } else if (field.form == RigExecBakedProgramImpl::WeightField::Form::Revision) {
                if (field.consumer < 0 || size_t(field.consumer) >= B.revisionIndex.size()) {
                    add("WeightField step " + step.label + " names an invalid revision consumer");
                } else {
                    const auto [chain,part] = B.revisionIndex[size_t(field.consumer)];
                    const auto covers = [&](RigExecBakedSlotDomain domain, uint32_t slot) {
                        return std::any_of(step.reads.begin(),step.reads.end(),
                            [&](const RigExecBakedSlotRange &range) {
                                return range.domain == domain && slot >= range.begin &&
                                       slot < range.end;
                            });
                    };
                    if (!covers(RigExecBakedSlotDomain::ChainBase,uint32_t(chain)))
                        add("WeightField step " + step.label + " omits its entering base");
                    for (int earlier = 0; earlier < part; ++earlier) {
                        const int revision = B.chainRevisionBegin[size_t(chain)] + earlier;
                        if (!covers(RigExecBakedSlotDomain::RevisionDone,uint32_t(revision)))
                            add("WeightField step " + step.label + " omits an entering completed revision");
                    }                    if (B.chains[chain].revisions[part].weightField != step.object)
                        add("WeightField step " + step.label + " has no matching revision consumer");
                }
            } else add("WeightField step " + step.label + " has an unknown consumer form");
            for (const auto &read : field.scalarReads) {
                for (uint32_t version : read.versions) {
                    const auto declared = RigExecBakedOne(RigExecBakedSlotDomain::PropertyResult,int(version));
                    if (std::find(step.reads.begin(),step.reads.end(),declared) == step.reads.end())
                        add("WeightField step " + step.label + " omits an oracle property version");
                }
            }
            for (int slot : field.volumes) {
                if (slot < 0 || size_t(slot) >= B.paths.size()) {
                    add("WeightField step " + step.label + " names an invalid placement slot");
                    continue;
                }
                const auto domain = field.placementPhase == RigExecBakedProgramImpl::WeightField::PlacementPhase::Base
                    ? RigExecBakedSlotDomain::WeightFramesBase : RigExecBakedSlotDomain::WeightFrames;
                const auto declared = RigExecBakedOne(domain,slot);
                if (std::find(step.reads.begin(),step.reads.end(),declared) == step.reads.end())
                    add("WeightField step " + step.label + " omits its placement read");
            }
        }
        const bool headKind = step.kind == RigExecBakedStepKind::PropertyRevision ||
            step.kind == RigExecBakedStepKind::RestCompose ||
            step.kind == RigExecBakedStepKind::LadderCompose ||
            step.kind == RigExecBakedStepKind::SkinTopology ||
            (step.kind == RigExecBakedStepKind::WeightField && step.object >= 0 &&
             size_t(step.object) < B.weightFields.size() &&
             B.weightFields[size_t(step.object)].form != RigExecBakedProgramImpl::WeightField::Form::Revision);
        if (step.isHead != headKind) add("step " + step.label + " has inconsistent head kind");
        if (step.isHead && (step.isSource || step.externalReads))
            add("head step " + step.label + " is a source or always step");
    }
    std::string duplicate;
    const auto writers = HeadWriters(B, &duplicate);
    if (!duplicate.empty()) {
        add(duplicate);
    }
    std::vector<size_t> position(B.steps.size(), B.steps.size());
    for (size_t p = 0; p < heads.size(); ++p) {
        if (heads[p] < position.size()) {
            position[heads[p]] = p;
        }
    }
    for (size_t p = 0; p < heads.size(); ++p) {
        const uint32_t index = heads[p];
        if (index >= B.steps.size()) {
            add("head order names step " + std::to_string(index) +
                ", which does not exist");
            continue;
        }
        const RigExecBakedStep &step = B.steps[index];
        const auto contains = [](const auto &ranges, RigExecBakedSlotDomain domain, uint32_t id) {
            return std::any_of(ranges.begin(),ranges.end(),[&](const auto &r) {
                return r.domain == domain && r.begin <= id && id < r.end;
            });
        };
        if (step.kind == RigExecBakedStepKind::PropertyRevision) {
            if (step.object < 0 || size_t(step.object) >= B.propertyChains.size() ||
                step.part < 0 || size_t(step.part) > B.propertyChains[size_t(step.object)].revisions.size()) {
                add("head step " + step.label + " has an invalid chain or revision part");
                continue;
            }
            const auto &chain = B.propertyChains[size_t(step.object)];
            const uint32_t own = chain.versionBase + uint32_t(step.part);
            std::set<uint32_t> expected{own};
            for (const uint32_t r : chain.records) {
                if (r >= B.propertyRecords.size()) { add("head step " + step.label + " has an invalid phased record"); continue; }
                const auto &record = B.propertyRecords[r];
                if (std::min(record.applied,chain.revisions.size()) == size_t(step.part)) expected.insert(record.id);
            }
            std::set<uint32_t> actual;
            for (const auto &range : step.writes) {
                if (range.domain != RigExecBakedSlotDomain::PropertyResult) {
                    add("head step " + step.label + " writes a non-property domain"); continue;
                }
                for (uint32_t id=range.begin;id<range.end;++id) actual.insert(id);
            }
            if (actual != expected) add("head step " + step.label + " does not write exactly its version and owned records");
            if (step.part > 0 && !contains(step.reads,RigExecBakedSlotDomain::PropertyResult,own-1))
                add("head step " + step.label + " omits its predecessor property version");
            if (step.part == 0 && !step.reads.empty()) add("head step " + step.label + " base reads a produced value");
        } else if (step.kind == RigExecBakedStepKind::RestCompose || step.kind == RigExecBakedStepKind::LadderCompose) {
            if (step.object < 0 || size_t(step.object) >= B.composeGroups.size()) {
                add("head step " + step.label + " has an invalid compose group"); continue;
            }
            const auto &group = B.composeGroups[size_t(step.object)];
            const auto domain = step.kind == RigExecBakedStepKind::RestCompose ? RigExecBakedSlotDomain::Rest : RigExecBakedSlotDomain::Ladder;
            if (step.part != (domain == RigExecBakedSlotDomain::Rest ? 0 : 1) || step.writes.size()!=1 ||
                step.writes[0].domain != domain || step.writes[0].begin != uint32_t(group.begin) || step.writes[0].end != uint32_t(group.end))
                add("head step " + step.label + " does not write its complete compose group");
            const auto require = [&](RigExecBakedSlotDomain input, int slot) {
                if (slot >= 0 && size_t(slot) < B.paths.size() &&
                    !contains(step.reads, input, uint32_t(slot)))
                    add("head step " + step.label + " omits required " +
                        RigExecBakedSlotDomainName(input) + " slot " + std::to_string(slot));
            };
            for (int slot = std::max(0, group.begin);
                 slot < std::min(int(B.paths.size()), group.end); ++slot) {
                if (domain == RigExecBakedSlotDomain::Ladder)
                    require(RigExecBakedSlotDomain::Rest, slot);
                if (size_t(slot) >= B.slotKind.size() ||
                    B.slotKind[size_t(slot)] != RigExecBakedSlotKind::FirstFramePose ||
                    size_t(slot) >= B.parent.size()) continue;
                const int parent = B.parent[size_t(slot)];
                if (parent < group.begin || parent >= group.end) {
                    require(RigExecBakedSlotDomain::Rest, parent);
                    if (domain == RigExecBakedSlotDomain::Ladder)
                        require(RigExecBakedSlotDomain::Ladder, parent);
                }
            }
        } else if (step.kind == RigExecBakedStepKind::SkinTopology) {
            const auto *revision = step.object < 0 ? nullptr : RigExecBakedLayoutRevision(B,size_t(step.object));
            if (!revision || revision->op != RigExecRevisionOp::Skin || step.part != 0 || step.writes.size()!=1 ||
                step.writes[0].domain!=RigExecBakedSlotDomain::SkinTopology || step.writes[0].begin!=uint32_t(step.object) || step.writes[0].end!=uint32_t(step.object)+1)
                add("head step " + step.label + " does not write its fixed layout revision");
        }
        // Completeness: every chain target or record consumer a walk of a
        // property revision meets is declared.
        if (step.kind != RigExecBakedStepKind::PropertyRevision ||
            step.part == 0 || step.object < 0 ||
            size_t(step.object) >= B.propertyChains.size()) {
            continue;
        }
        const RigExecBakedPropertyChain &chain =
            B.propertyChains[size_t(step.object)];
        if (size_t(step.part) > chain.revisions.size()) {
            continue;
        }
        const auto declares = [&step](uint32_t id) {
            for (const RigExecBakedSlotRange &range : step.reads) {
                if (range.domain == RigExecBakedSlotDomain::PropertyResult &&
                    range.begin <= id && id < range.end) {
                    return true;
                }
            }
            return false;
        };
        const RigExecBakedPropertyChain::Revision &r =
            chain.revisions[size_t(step.part) - 1];
        for (const RigExecBakedWalk *walk :
             {&r.enabled, &r.defaultWeight, &r.value, &r.minimum,
              &r.maximum, &r.keys, &r.tangents}) {
            for (const std::vector<RigExecBakedWalkHop> *hops :
                 {&walk->hops, &walk->doubleHops}) {
                for (const RigExecBakedWalkHop &hop : *hops) {
                    if (hop.chain >= 0 &&
                        size_t(hop.chain) < B.propertyChains.size()) {
                        const RigExecBakedPropertyChain &other =
                            B.propertyChains[size_t(hop.chain)];
                        if (!declares(other.versionBase +
                                      uint32_t(other.revisions.size()))) {
                            add("walk " + hop.path.GetString() +
                                " meets chain target without declaring it");
                        }
                    }
                    if (hop.record >= 0 &&
                        size_t(hop.record) < B.propertyRecords.size() &&
                        !declares(
                            B.propertyRecords[size_t(hop.record)].id)) {
                        add("walk " + hop.path.GetString() +
                            " meets record without declaring it");
                    }
                }
            }
        }
    }
    const auto &propertyWriters = writers[size_t(RigExecBakedSlotDomain::PropertyResult)];
    for (uint32_t id=0;id<B.propertyVersionCount;++id)
        if (id>=propertyWriters.size() || propertyWriters[id]<0)
            add("property version " + std::to_string(id) + " has no head producer");
    if (heads.size() != std::count_if(B.steps.begin(), B.steps.end(), [](const auto &s) { return s.isHead; })) {
        add("the head order holds " + std::to_string(heads.size()) +
            " of " + std::to_string(B.steps.size()) + " head step(s)");
    }
    if (count == 0) {
        return true;
    }
    if (error) {
        *error = first;
        if (count > 1) {
            *error += " (and " + std::to_string(count - 1) + " more)";
        }
    }
    return false;
}

bool
RigExecBakedValidateHeadReads(const RigExecBakedProgramImpl &B,
                              std::string *error)
{
    size_t count = 0;
    std::string first;
    const auto add = [&](const std::string &violation) {
        if (count++ == 0) {
            first = violation;
        }
    };
    const auto writers = HeadWriters(B, nullptr);
    const auto producedIn = [&writers](RigExecBakedSlotDomain domain,
                                       uint32_t id) {
        const std::vector<int> &table = writers[size_t(domain)];
        return id < table.size() && table[id] >= 0;
    };
    const auto produced = [&producedIn](uint32_t id) {
        return producedIn(RigExecBakedSlotDomain::PropertyResult, id);
    };
    for (const RigExecBakedStep &step : B.steps) {
        if (step.isHead) continue;
        const auto declares = [&step](uint32_t id) {
            for (const RigExecBakedSlotRange &range : step.reads) {
                if (range.domain == RigExecBakedSlotDomain::PropertyResult &&
                    range.begin <= id && id < range.end) {
                    return true;
                }
            }
            return false;
        };
        for (const RigExecBakedSlotRange &range : step.reads) {
            if (!RigExecBakedIsHeadDomain(range.domain)) continue;
            for (uint32_t id = range.begin; id < range.end; ++id) {
                if (!producedIn(range.domain, id)) {
                    add("step " + step.label + " reads " +
                        RigExecBakedSlotDomainName(range.domain) + " slot " +
                        std::to_string(id) + ", which no head step writes");
                }
            }
        }
        for (const auto &required : RigExecBakedRequiredRestReads(B,step)) {
            for (uint32_t id=required.begin;id<required.end;++id) {
                const bool declared = std::any_of(step.reads.begin(),step.reads.end(),[&](const auto &r) {
                    return r.domain==required.domain && r.begin<=id && id<r.end;
                });
                if (!declared) add("step " + step.label + " omits required " +
                    RigExecBakedSlotDomainName(required.domain) + " slot " + std::to_string(id));
            }
        }
        size_t layout = 0;
        bool layoutReader = true;
        switch (step.kind) {
        case RigExecBakedStepKind::RevisionStatic:
        case RigExecBakedStepKind::RevisionChunk:
        case RigExecBakedStepKind::RevisionFuse: layout=size_t(step.object); break;
        case RigExecBakedStepKind::Derived: layout=B.revisionIndex.size()+size_t(step.object); break;
        default: layoutReader=false; break;
        }
        const auto *revision = layoutReader ? RigExecBakedLayoutRevision(B,layout) : nullptr;
        if (revision && revision->op == RigExecRevisionOp::Skin && !std::any_of(step.reads.begin(),step.reads.end(),[&](const auto &r) {
            return r.domain==RigExecBakedSlotDomain::SkinTopology && r.begin<=layout && layout<r.end;
        })) add("step " + step.label + " omits its required SkinTopology slot " + std::to_string(layout));
        // Completeness: every chain target or record consumer a walk the
        // step reads meets is declared.
        for (const int walk : step.readerWalks) {
            if (walk < 0 || size_t(walk) >= B.readerWalks.size()) {
                add("step " + step.label + " names reader walk " +
                    std::to_string(walk) + ", which does not exist");
                continue;
            }
            const RigExecBakedReaderWalk &reader = B.readerWalks[size_t(walk)];
            for (const std::vector<RigExecBakedWalkHop> *hops :
                 {&reader.walk.hops, &reader.walk.doubleHops}) {
                for (const RigExecBakedWalkHop &hop : *hops) {
                    if (hop.chain >= 0 &&
                        size_t(hop.chain) < B.propertyChains.size()) {
                        const RigExecBakedPropertyChain &chain =
                            B.propertyChains[size_t(hop.chain)];
                        if (!declares(chain.versionBase +
                                      uint32_t(chain.revisions.size()))) {
                            add("walk " + hop.path.GetString() +
                                " meets chain target without declaring it");
                        }
                    }
                    if (hop.record >= 0 &&
                        size_t(hop.record) < B.propertyRecords.size() &&
                        !declares(B.propertyRecords[size_t(hop.record)].id)) {
                        add("walk " + hop.path.GetString() +
                            " meets record without declaring it");
                    }
                }
            }
        }
    }
    for (size_t slot = 0; slot < B.avarHeadReads.size(); ++slot) {
        for (const uint32_t id : B.avarHeadReads[slot]) {
            if (!produced(id)) {
                add("avar slot " + std::to_string(slot) +
                    " reads PropertyVersion slot " + std::to_string(id) +
                    ", which no head step writes");
            }
        }
    }
    if (count == 0) {
        return true;
    }
    if (error) {
        *error = first;
        if (count > 1) {
            *error += " (and " + std::to_string(count - 1) + " more)";
        }
    }
    return false;
}

void RigExecBakedPrepareHeadOps(RigExecBakedProgramImpl *program)
{
    auto &B = *program;
    std::fill(B.propertyChanged.begin(),B.propertyChanged.end(),uint8_t(0));
    for (int slot : B.restMoved) B.restChanged[size_t(slot)] = 0;
    for (int slot : B.ladderMoved) B.ladderChanged[size_t(slot)] = 0;
    B.restMoved.clear(); B.ladderMoved.clear();
    for (auto &field : B.weightFields) field.changed = false;
    for (int layout : B.skinTopologyLayouts)
        if (auto *revision = RigExecBakedLayoutRevision(&B,size_t(layout))) revision->layoutOutputChanged = false;
}
void RigExecBakedFinishHeadOp(RigExecBakedProgramImpl *B,const RigExecBakedStep &step)
{
    if (step.kind == RigExecBakedStepKind::PropertyRevision) RigExecBakedFinishPropertyStep(B,step);
}
bool RigExecBakedHeadValueChanged(const RigExecBakedProgramImpl &B,
                                  RigExecBakedSlotDomain domain,uint32_t slot)
{
    switch (domain) {
    case RigExecBakedSlotDomain::PropertyResult: return slot < B.propertyChanged.size() && B.propertyChanged[slot];
    case RigExecBakedSlotDomain::Rest: return slot < B.restChanged.size() && B.restChanged[slot];
    case RigExecBakedSlotDomain::Ladder: return slot < B.ladderChanged.size() && B.ladderChanged[slot];
    case RigExecBakedSlotDomain::WeightField: return slot < B.weightFields.size() && B.weightFields[slot].changed;
    case RigExecBakedSlotDomain::SkinTopology: {
        const auto *revision = RigExecBakedLayoutRevision(B,slot);
        return revision && revision->layoutOutputChanged;
    }
    default: return false;
    }
}

std::vector<RigExecBakedClusterSet>
RigExecBakedHeadOpSeeds(const RigExecBakedProgramImpl &B)
{
    const auto heads = RigExecBakedHeadIndices(B);
    const size_t clusters = B.clustering.clusters.size();
    std::vector<RigExecBakedClusterSet> seeds(B.steps.size());
    // Reverse head order: every successor's set is final before a
    // predecessor unions it.
    for (size_t k = heads.size(); k-- > 0;) {
        const uint32_t i = heads[k];
        const RigExecBakedStep &step = B.steps[i];
        RigExecBakedClusterSet &set = seeds[i];
        set.Resize(clusters);
        const auto readers = [&](const std::vector<std::vector<int>> &table,
                                 uint32_t id) {
            if (id >= table.size()) {
                return;
            }
            for (const int reader : table[id]) {
                if (reader >= 0 && size_t(reader) < B.steps.size()) {
                    set.Set(B.steps[size_t(reader)].cluster);
                }
            }
        };
        for (const RigExecBakedSlotRange &range : step.writes) {
            for (uint32_t id = range.begin; id < range.end; ++id) {
                switch (range.domain) {
                case RigExecBakedSlotDomain::PropertyResult:
                    readers(B.cones.headReaders, id);
                    break;
                case RigExecBakedSlotDomain::Rest:
                    readers(B.cones.restReaders, id);
                    break;
                case RigExecBakedSlotDomain::Ladder:
                    readers(B.cones.ladderReaders, id);
                    break;
                case RigExecBakedSlotDomain::WeightField:
                    readers(B.cones.fieldReaders,id);
                    break;
                case RigExecBakedSlotDomain::SkinTopology:
                    break;
                }
            }
        }
        for (const uint32_t succ : step.succs) {
            set.Union(seeds[succ]);
        }
    }
    return seeds;
}

std::vector<int>
RigExecBakedHeadSeedsFrom(const RigExecBakedProgramImpl &B,
                          const std::vector<RigExecBakedClusterSet> &opSeeds,
                          const std::vector<uint32_t> &ops)
{
    RigExecBakedClusterSet set;
    set.Resize(B.clustering.clusters.size());
    for (const uint32_t op : ops) {
        if (op < opSeeds.size()) {
            set.Union(opSeeds[op]);
        }
    }
    std::vector<int> out;
    for (size_t c = 0; c < B.clustering.clusters.size(); ++c) {
        if (set.Test(int(c))) {
            out.push_back(int(c));
        }
    }
    return out;
}

std::vector<uint32_t>
RigExecBakedHeadOpsReading(const RigExecBakedProgramImpl &B, int index)
{
    std::vector<uint32_t> ops;
    if (index < 0 || size_t(index) >= B.leafOfOverride.size() ||
        B.leafOfOverride[size_t(index)] < 0) {
        return ops;
    }
    const uint32_t leaf = uint32_t(B.leafOfOverride[size_t(index)]);
    for (size_t i = 0; i < B.steps.size(); ++i) {
        if (!B.steps[i].isHead) continue;
        const std::vector<uint32_t> &leaves = B.steps[i].bindingLeaves;
        if (std::find(leaves.begin(), leaves.end(), leaf) != leaves.end()) {
            ops.push_back(uint32_t(i));
        }
    }
    return ops;
}

std::vector<int>
RigExecBakedHeadSeeds(const RigExecBakedProgramImpl &B, int index)
{
    return RigExecBakedHeadSeedsFrom(B, RigExecBakedHeadOpSeeds(B),
                                     RigExecBakedHeadOpsReading(B, index));
}

}  // namespace rigExec
