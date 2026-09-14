//
// RigExec baked program: the compiled epoch as a flat op list over dense
// slots, with no exec round trip on the per-frame path.
//
// The dynamic path re-derives the same numbers every frame through OpenExec
// requests, SdfPath-keyed maps and VtValue copies. Almost none of that work
// depends on the frame: on a rig whose shape is fixed for the epoch, the
// provider hierarchy, the default-space ladder, the solver rest descriptions,
// the constraint wiring and the descendant propagation pairs are all decided
// once. This class decides them once -- at Compile -- and leaves a per-frame
// program that reads only the inputs that actually vary, runs the same
// rigExecMath kernels in the same order over dense arrays, and publishes the
// same RigExecRigPose.
//
// It is a SECOND implementation of the evaluation semantics, so it is a
// request rather than a promise: a rig using any feature the program cannot
// express stays on the dynamic path, with a reason per feature. See
// IsBakeable.
//
#ifndef RIGEXEC_BAKED_PROGRAM_H
#define RIGEXEC_BAKED_PROGRAM_H

#include "tapSet.h"

#include "pxr/usd/usd/notice.h"
#include "pxr/usd/usd/timeCode.h"

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

class RigExecRigEvaluator;
struct RigExecRigPose;

/// Which path RigExecRigEvaluator::Evaluate takes.
///
/// `Baked` is a REQUEST, not a guarantee: Compile builds the program only
/// when the epoch is bakeable and Evaluate falls back to the dynamic path
/// whenever it is not, so setting it can never change an answer -- only how
/// fast it arrives.
enum class RigExecEvaluationMode {
    /// OpenExec plus the in-memory pose walk. The reference path.
    Dynamic,
    /// The baked program when the epoch allows it, Dynamic otherwise.
    Baked,
    /// Both, in one generation, compared with exact equality. Every
    /// disagreement is a diagnostic and a count on the published pose.
    BakedWithParityCheck,
    /// The dynamic path with the constraint walk gated on a dirty set: a
    /// constraint whose input closure AND output set are both clean replays
    /// the outputs it recorded last generation instead of solving again
    /// (docs/superpowers/specs/2026-09-13-sparse-evaluation-design.md,
    /// Gate 1). A REQUEST like Baked: whenever the generation cannot be
    /// answered sparsely -- a new epoch, a notice, a different time -- it
    /// falls back to the full walk, so asking for it can never change an
    /// answer.
    Sparse,
    /// Sparse and then Dynamic, in that order, compared with exact equality.
    /// Sparse runs FIRST so it is validated against the cache state it sees
    /// in production rather than one the reference just warmed, and the
    /// DYNAMIC generation is the one published -- a disagreement must not
    /// also change what consumers see. The same shape, and the same
    /// reasoning, as BakedWithParityCheck.
    SparseWithParityCheck,
};

/// Appends one diagnostic per exact-equality disagreement between \p baked
/// and \p reference, counting them on \p out->bakedParityMismatches.
///
/// The comparison BakedWithParityCheck performs, as a function of two poses
/// and nothing else. It is the only thing in the suite that catches several
/// classes of bake defect -- a solver aggregate that drifts in its last bits,
/// a rest that moved on one path and not the other -- and every assertion
/// made through the parity mode is that it found NOTHING, which is an
/// assertion a dead comparator also satisfies. Declared here so a test can
/// hand it two poses it built itself and check that it finds what is there.
///
/// Compares the seven published maps plus the solver guides, in both
/// directions: a key present only in \p reference and a key present only in
/// \p baked are each one mismatch. It also compares the SCALARS of the
/// generation -- the mover-graph work counters, the solver override rounds
/// and the diagnostics, the last order-sensitively -- because those are
/// published state a consumer reads, and a program that arrives at the right
/// numbers while claiming different work is still a second rig. Each is its
/// own mismatch domain with its own text, so a count of one names which.
///
/// RigExecRigPose::solverEvaluations is the one published scalar left out,
/// and on purpose: it counts the solver computations the schedule requested,
/// and the dynamic path's per-batch exec cache answers a repeated time with
/// the same inputs for free while the program, which holds no such cache,
/// re-solves. The two numbers are each true of the path that reported them.
/// compareWork false omits the generation SCALARS -- the mover-graph work
/// counters, the solver override rounds, and the diagnostics. They describe
/// what a generation DID rather than what the rig looks like, and two
/// generations run back to back in one parity call cannot agree about that:
/// the second reuses every cache the first warmed. Measured on the biped as a
/// constant 2 mismatches on every sparse probe, INCLUDING the ones that
/// replayed nothing -- which is the tell that the disagreement was never
/// about the gate under test. Sparse parity passes false and compares only
/// values; baked parity keeps the default and compares everything, exactly as
/// before.
void RigExecComparePoses(const RigExecRigPose &reference,
                         const RigExecRigPose &baked, RigExecRigPose *out,
                         bool compareWork = true);

/// One compiled epoch, flattened.
class RigExecBakedProgram {
public:
    ~RigExecBakedProgram();

    RigExecBakedProgram(const RigExecBakedProgram &) = delete;
    RigExecBakedProgram &operator=(const RigExecBakedProgram &) = delete;

    /// Whether \p evaluator's compiled epoch can be expressed as a program,
    /// appending one reason per feature that cannot.
    ///
    /// The reasons are the point: a rig that will not bake should say which
    /// of its features stopped it, because "it fell back" is not actionable
    /// and a silent fallback reads as the mode not working.
    static bool IsBakeable(const RigExecRigEvaluator &evaluator,
                           std::vector<std::string> *reasons);

    /// Builds the program from \p evaluator's compiled epoch, or returns
    /// null with the reasons when the epoch is not bakeable.
    ///
    /// \p evaluator must outlive the program; the evaluator owns it and drops
    /// it whenever the epoch or the scene changes underneath.
    static std::unique_ptr<RigExecBakedProgram> Build(
        RigExecRigEvaluator *evaluator, std::vector<std::string> *reasons);

    /// Runs the whole program at \p time and publishes into \p pose.
    ///
    /// Returns false having published diagnostics when the program could not
    /// complete; the caller is expected to fall back to the dynamic path.
    bool Run(UsdTimeCode time, RigExecRigPose *pose);

    /// Whether \p notice can have moved anything the bake captured.
    ///
    /// The epoch digest is deliberately blind to values, so it cannot answer
    /// this: it is unchanged by exactly the edits that make a captured
    /// constant wrong. The bake therefore records WHICH properties it read
    /// and which prims it read them from, and this asks that index. An edit
    /// that misses it -- a value on an input the program re-reads every
    /// frame, or anything on a prim the bake never looked at -- leaves the
    /// program standing, which is the whole point of having an index rather
    /// than dropping the program on every notice.
    bool IsInvalidatedBy(const UsdNotice::ObjectsChanged &notice) const;

    /// Places the standing interactive overrides for the generations that
    /// follow, returning false when one of them names something the program
    /// cannot place.
    ///
    /// A placeable override is one the program can route the way the dynamic
    /// path does: an input the frame path reads (it is read the long way,
    /// through the generation's resolved inputs, while the override stands),
    /// or a property whose only reader already goes through those resolved
    /// inputs -- a property-chain mover, a geometry mover. An override on a
    /// value folded into bake state (a rest, a default-space ladder, a
    /// solver rest description) cannot be placed without rebaking, and a
    /// computation override names something that only exec can answer; both
    /// return false so the caller runs the generation dynamically. A wrong
    /// baked answer is never one of the outcomes.
    bool SetOverrides(const std::vector<RigExecValueOverride> &overrides);

    /// Dense provider slots in namespace DFS order.
    size_t GetProviderCount() const;
    /// Input channels the program reads from USD at all.
    size_t GetBoundInputCount() const;
    /// Of those, the ones re-read every frame; the rest are epoch constants.
    size_t GetVaryingInputCount() const;

    /// Takes over \p previous's persistent geometry state.
    ///
    /// The program's geometry revisions are the dynamic path's `_liveGraphs`
    /// with the VdfNetwork baked away: each holds the packet it last ran with
    /// and the points it produced, so an unchanged input re-publishes instead
    /// of re-running the kernel. That state belongs to the RIG, not to one
    /// program -- the dynamic path keeps its nodes across a value edit and
    /// across a recompile, reconnecting whichever survive -- and a program
    /// that started over would re-run every kernel and publish
    /// `moverGraphRevisionsCreated` / `SchedulesBuilt` and the mover graph
    /// diagnostic for nodes that were never rebuilt.
    ///
    /// Matched exactly the way the dynamic walk matches VdfNetwork nodes: by
    /// chain target, then by (mover, operation) identity, with the schedule
    /// counted as rebuilt only when the identity SEQUENCE of a chain changed.
    /// A revision with no match is new and is reported as created.
    ///
    /// \p previous is left empty of the state it handed over.
    void AdoptGeometryStateFrom(RigExecBakedProgram &previous);

private:
    struct _Impl;
    explicit RigExecBakedProgram(std::unique_ptr<_Impl> impl);
    std::unique_ptr<_Impl> _impl;
};

}  // namespace rigExec

#endif  // RIGEXEC_BAKED_PROGRAM_H
