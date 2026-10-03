// Runtime pose-step dispatch, commit deltas, and propagation.

#include "poseInternal.h"
#include <algorithm>
#include <limits>
#include <utility>

namespace rigExec {

using namespace runtimePoseDetail;

namespace {

// Propagation outcomes, in RigExecBakedPropagateOutcome order.
enum _RrPropagateOutcome : uint8_t {
    _RrStaged = 0,
    _RrSkipped = 1,
    _RrNoCandidate = 2,
    _RrUnusableDescendant = 3,
    _RrSingularDelta = 4,
    _RrInvalidResult = 5,
};

// The split constants of the commit walk (bakedPose.cpp).
enum : size_t {
    _RrPropagateChunkSize = 64,
};

std::string
_RrKindName(RigExecWireStepKind kind)
{
    switch (kind) {
    case RigExecWireStepKind::ComposeSubtree:
        return "ComposeSubtree";
    case RigExecWireStepKind::Solve:
        return "Solve";
    case RigExecWireStepKind::SolverCommit:
        return "SolverCommit";
    case RigExecWireStepKind::Constraint:
        return "Constraint";
    case RigExecWireStepKind::CommitDelta:
        return "CommitDelta";
    case RigExecWireStepKind::PropagateChunk:
        return "PropagateChunk";
    case RigExecWireStepKind::CommitApply:
        return "CommitApply";
    case RigExecWireStepKind::ProviderMatrix:
        return "ProviderMatrix";
    case RigExecWireStepKind::SnapshotFinals:
        return "SnapshotFinals";
    case RigExecWireStepKind::PoseInterpolator:
        return "PoseInterpolator";
    default:
        return "Unknown";
    }
}

// The phased-read store's pose-half record (RecordFrame, bakedPose.cpp).
bool
_RrRecordFrame(RrProgram *program, size_t step,
               const RigExecWireConstraint &constraint,
               const RigExecWireCommit &commit, RrStepOutput *output,
               std::string *error)
{
    RrStore &store = program->store;
    RrPoseScratch *scratch = _RrScratch(program);
    if (!constraint.snapshotAfter) {
        return true;
    }
    const size_t commitIndex = size_t((*program->steps)[step].object);
    const bool everyTarget =
        commitIndex < scratch->recordEveryTarget.size() &&
        scratch->recordEveryTarget[commitIndex];
    const size_t count =
        everyTarget ? constraint.targetSlots.size()
                    : std::min<size_t>(1, constraint.targetSlots.size());
    for (size_t k = 0; k < count; ++k) {
        if (k >= constraint.snapshotTargets.size() ||
            !constraint.snapshotTargets[k]) {
            continue;
        }
        if (k >= constraint.targetSlots.size() ||
            k >= commit.targetReads.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no record target";
            }
            return false;
        }
        const int32_t target = constraint.targetSlots[k];
        const size_t slot = size_t(target);
        const auto found = std::lower_bound(commit.slots.begin(),
                                            commit.slots.end(), target);
        const uint32_t version =
            found != commit.slots.end() && *found == target
                ? (size_t(found - commit.slots.begin()) <
                           commit.slotWrites.size()
                       ? commit.slotWrites[size_t(found -
                                                  commit.slots.begin())]
                       : commit.targetReads[k])
                : commit.targetReads[k];
        if (size_t(version) >= store.fin.size() ||
            slot >= scratch->restFrames.size() ||
            slot >= program->slotMeta->paths.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no record target";
            }
            return false;
        }
        const RrPointFrame &frame = store.fin[size_t(version)];
        if (!frame.IsValid()) {
            continue;
        }
        const RrPointFrame &rest = scratch->restFrames[slot];
        const std::array<RrVec3d, 4> landmarks =
            rest.IsValid() ? rest.points : RrIdentityLandmarks();
        RrMat4d matrix = _RrIdentity();
        if (RrPointsToMatrix(landmarks, frame.points, &matrix)) {
            RrSnapshotValue value;
            value.tag = RrSnapshotValue::Tag::Matrix;
            value.matrix = matrix;
            output->snapshots.Record(
                program->slotMeta->paths[slot], constraint.path, value);
        }
    }
    return true;
}

} // namespace

namespace runtimePoseDetail {

std::string
_RrStepHead(const RrProgram *program, size_t step)
{
    const RigExecWireStep &wire = (*program->steps)[step];
    return "pose step " + _RrKindName(wire.kind) + " object " +
           std::to_string(wire.object) + " part " +
           std::to_string(wire.part);
}

// The hierarchy delta each candidate of the commit carries, once
// (ComputeCommitDeltas, bakedPose.cpp).
bool
_RrComputeCommitDeltas(RrProgram *program, size_t step,
                       const RigExecWireCommit &commit,
                       RrCommitScratch *scratch, std::string *error)
{
    RrStore &store = program->store;
    for (size_t pos = 0; pos < commit.slots.size(); ++pos) {
        if (pos >= scratch->present.size() ||
            pos >= scratch->frames.size() ||
            pos >= scratch->deltas.size() ||
            pos >= scratch->deltaOk.size() ||
            pos >= commit.slotReads.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no candidate slot";
            }
            return false;
        }
        if (!scratch->present[pos]) {
            scratch->deltaOk[pos] = 0;
            continue;
        }
        scratch->deltas[pos] = _RrIdentity();
        const uint32_t read = commit.slotReads[pos];
        if (size_t(read) >= store.fin.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no fin version";
            }
            return false;
        }
        scratch->deltaOk[pos] =
            RrPointsToMatrix(store.fin[size_t(read)].points,
                             scratch->frames[pos].points,
                             &scratch->deltas[pos])
                ? 1
                : 2;
    }
    return true;
}

// Stages descendants [begin, end) of the commit (StageCommitPairs,
// bakedPose.cpp): each pair decided exactly where the walk decides it.
bool
_RrStageCommitPairs(RrProgram *program, size_t step,
                    const RigExecWireCommit &commit,
                    RrCommitScratch *scratch, size_t begin, size_t end,
                    std::string *error)
{
    RrStore &store = program->store;
    for (size_t k = begin;
         k < end && k < commit.propagate.size(); ++k) {
        if (k >= commit.closestPos.size() ||
            k >= commit.descendantReads.size() ||
            k >= commit.closestReads.size() ||
            k >= scratch->staged.size() ||
            k >= scratch->outcome.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no propagation pair";
            }
            return false;
        }
        const int pos = commit.closestPos[k];
        if (pos < 0 || size_t(pos) >= scratch->present.size() ||
            !scratch->present[size_t(pos)]) {
            scratch->outcome[k] = _RrNoCandidate;
            continue;
        }
        const uint32_t descendantRead = commit.descendantReads[k];
        const uint32_t closestRead = commit.closestReads[k];
        if (size_t(descendantRead) >= store.fin.size() ||
            size_t(closestRead) >= store.fin.size() ||
            size_t(pos) >= scratch->frames.size() ||
            size_t(pos) >= scratch->deltaOk.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no fin version";
            }
            return false;
        }
        const RrPointFrame &current =
            store.fin[size_t(descendantRead)];
        const RrPointFrame &before = store.fin[size_t(closestRead)];
        if (commit.solverOutput &&
            (!RrFrameUsable(current) || !RrFrameUsable(before) ||
             !RrFrameUsable(scratch->frames[size_t(pos)]))) {
            scratch->outcome[k] = _RrSkipped;
            continue;
        }
        if (!RrFrameUsable(current)) {
            scratch->outcome[k] = _RrUnusableDescendant;
            continue;
        }
        if (scratch->deltaOk[size_t(pos)] != 1) {
            scratch->outcome[k] = _RrSingularDelta;
            continue;
        }
        const RrPointFrame frame = RrMatrixToPoints(
            current.points, scratch->deltas[size_t(pos)]);
        if (!RrFrameUsable(frame)) {
            scratch->outcome[k] = _RrInvalidResult;
            continue;
        }
        scratch->staged[k] = frame;
        scratch->outcome[k] = _RrStaged;
    }
    return true;
}

// Decides the commit and writes it back, or says why it passed through
// (FinishCommit, bakedPose.cpp).
bool
_RrFinishCommit(RrProgram *program, size_t step, size_t commitIndex,
                std::string *error)
{
    RrStore &store = program->store;
    RrPoseScratch *scratch = _RrScratch(program);
    const RigExecWireCommit &commit =
        program->poses->commits[commitIndex];
    RrCommitScratch &live = store.commits[commitIndex];
    RrStepOutput &output = store.stepOutputs[step];
    const RigExecWireConstraint *constraint = nullptr;
    if (!commit.solverOutput) {
        const size_t walk = size_t((*program->steps)[step].object);
        if (walk >= program->poses->walkSteps.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no walk step";
            }
            return false;
        }
        const int index = program->poses->walkSteps[walk].index;
        if (index < 0 ||
            size_t(index) >= program->poses->constraints.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no constraint";
            }
            return false;
        }
        constraint = &program->poses->constraints[size_t(index)];
    }
    const auto carry = [&](size_t pos, size_t k, bool candidates,
                           bool descendants, std::string *fail) {
        if (candidates) {
            if (pos >= commit.slotWrites.size() ||
                pos >= commit.slotCarry.size() ||
                size_t(commit.slotWrites[pos]) >= store.fin.size() ||
                size_t(commit.slotCarry[pos]) >= store.fin.size()) {
                if (fail) {
                    *fail = _RrStepHead(program, step) +
                            " names no fin version";
                }
                return false;
            }
            store.fin[size_t(commit.slotWrites[pos])] =
                store.fin[size_t(commit.slotCarry[pos])];
            if (commit.solverOutput) {
                if (pos >= commit.slotBaseWrites.size() ||
                    pos >= commit.slotBaseCarry.size() ||
                    size_t(commit.slotBaseWrites[pos]) >=
                        store.base.size() ||
                    size_t(commit.slotBaseCarry[pos]) >=
                        store.base.size()) {
                    if (fail) {
                        *fail = _RrStepHead(program, step) +
                                " names no base version";
                    }
                    return false;
                }
                store.base[size_t(commit.slotBaseWrites[pos])] =
                    store.base[size_t(commit.slotBaseCarry[pos])];
            }
        }
        if (descendants) {
            if (k >= commit.descendantWrites.size() ||
                k >= commit.descendantCarry.size() ||
                size_t(commit.descendantWrites[k]) >=
                    store.fin.size() ||
                size_t(commit.descendantCarry[k]) >= store.fin.size()) {
                if (fail) {
                    *fail = _RrStepHead(program, step) +
                            " names no fin version";
                }
                return false;
            }
            store.fin[size_t(commit.descendantWrites[k])] =
                store.fin[size_t(commit.descendantCarry[k])];
            if (commit.solverOutput) {
                if (k >= commit.descendantBaseWrites.size() ||
                    k >= commit.descendantBaseCarry.size() ||
                    size_t(commit.descendantBaseWrites[k]) >=
                        store.base.size() ||
                    size_t(commit.descendantBaseCarry[k]) >=
                        store.base.size()) {
                    if (fail) {
                        *fail = _RrStepHead(program, step) +
                                " names no base version";
                    }
                    return false;
                }
                store.base[size_t(commit.descendantBaseWrites[k])] =
                    store.base[size_t(commit.descendantBaseCarry[k])];
            }
        }
        return true;
    };
    const auto carryEverything = [&](std::string *fail) {
        for (size_t pos = 0; pos < commit.slots.size(); ++pos) {
            if (!carry(pos, 0, true, false, fail)) {
                return false;
            }
        }
        for (size_t k = 0; k < commit.propagate.size(); ++k) {
            if (!carry(0, k, false, true, fail)) {
                return false;
            }
        }
        return true;
    };
    const auto record = [&]() {
        if (constraint && commitIndex < scratch->recordAfter.size() &&
            scratch->recordAfter[commitIndex]) {
            return _RrRecordFrame(program, step, *constraint, commit,
                                  &output, error);
        }
        return true;
    };
    if (live.abandoned) {
        if (!carryEverything(error)) {
            return false;
        }
        return record();
    }
    const std::string mover = program->TextOrEmpty(commit.moverPath);
    for (size_t k = 0; k < commit.propagate.size(); ++k) {
        if (k >= live.outcome.size() ||
            k >= commit.propagate.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no propagation pair";
            }
            return false;
        }
        const _RrPropagateOutcome outcome =
            _RrPropagateOutcome(live.outcome[k]);
        if (outcome == _RrStaged || outcome == _RrSkipped) {
            continue;
        }
        switch (outcome) {
        case _RrNoCandidate:
            if (!carryEverything(error)) {
                return false;
            }
            output.bail = true;
            return true;
        case _RrUnusableDescendant:
            output.diagnostics.push_back(
                mover + " could not propagate its pose revision through " +
                program->TextOrEmpty(
                    program->slotMeta->paths[size_t(
                        commit.propagate[k].first)]) +
                "; constraint passed through");
            break;
        case _RrSingularDelta:
            output.diagnostics.push_back(
                mover + " produced a singular hierarchy delta; constraint "
                "passed through");
            break;
        default:
            output.diagnostics.push_back(
                mover + " produced an invalid descendant frame for " +
                program->TextOrEmpty(
                    program->slotMeta->paths[size_t(
                        commit.propagate[k].first)]) +
                "; constraint passed through");
            break;
        }
        if (!carryEverything(error)) {
            return false;
        }
        return record();
    }
    for (size_t pos = 0; pos < commit.slots.size(); ++pos) {
        if (pos >= live.present.size() || pos >= live.frames.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no candidate slot";
            }
            return false;
        }
        if (!live.present[pos]) {
            if (!carry(pos, 0, true, false, error)) {
                return false;
            }
            continue;
        }
        if (pos >= commit.slotWrites.size() ||
            size_t(commit.slotWrites[pos]) >= store.fin.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no fin version";
            }
            return false;
        }
        store.fin[size_t(commit.slotWrites[pos])] = live.frames[pos];
        if (commit.solverOutput) {
            if (pos >= commit.slotBaseWrites.size() ||
                size_t(commit.slotBaseWrites[pos]) >=
                    store.base.size()) {
                if (error) {
                    *error = _RrStepHead(program, step) +
                             " names no base version";
                }
                return false;
            }
            store.base[size_t(commit.slotBaseWrites[pos])] =
                live.frames[pos];
        }
    }
    for (size_t k = 0; k < commit.propagate.size(); ++k) {
        if (k >= live.outcome.size() || k >= live.staged.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no propagation pair";
            }
            return false;
        }
        if (_RrPropagateOutcome(live.outcome[k]) != _RrStaged) {
            if (!carry(0, k, false, true, error)) {
                return false;
            }
            continue;
        }
        if (k >= commit.descendantWrites.size() ||
            size_t(commit.descendantWrites[k]) >= store.fin.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no fin version";
            }
            return false;
        }
        store.fin[size_t(commit.descendantWrites[k])] = live.staged[k];
        if (commit.solverOutput) {
            if (k >= commit.descendantBaseWrites.size() ||
                size_t(commit.descendantBaseWrites[k]) >=
                    store.base.size()) {
                if (error) {
                    *error = _RrStepHead(program, step) +
                             " names no base version";
                }
                return false;
            }
            store.base[size_t(commit.descendantBaseWrites[k])] =
                live.staged[k];
        }
    }
    return record();
}

} // namespace runtimePoseDetail

bool
RrRunPoseStep(RrProgram *program, size_t step, double time,
              std::string *error)
{
    (void)time;
    RrStore &store = program->store;
    RrPoseScratch *scratch = _RrScratch(program);
    if (step >= program->steps->size() ||
        step >= store.stepOutputs.size()) {
        if (error) {
            *error = "pose step " + std::to_string(step) +
                     " names no step";
        }
        return false;
    }
    const RigExecWireStep &wire = (*program->steps)[step];
    RrStepOutput &output = store.stepOutputs[step];
    (void)output;

    switch (wire.kind) {
    case RigExecWireStepKind::ComposeSubtree: {
        if (wire.object < 0 ||
            size_t(wire.object) >= program->poses->composeGroups.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no compose group";
            }
            return false;
        }
        const RigExecWireComposeGroup &group =
            program->poses->composeGroups[size_t(wire.object)];
        const RigExecWireSlotMeta &meta = *program->slotMeta;
        for (int i = group.begin; i < group.end; ++i) {
            if (i < 0 || size_t(i) >= meta.slotKind.size() ||
                size_t(i) >= store.base.size() ||
                size_t(i) >= store.fin.size() ||
                size_t(i) >= store.posedM.size() ||
                size_t(i) >= scratch->restM.size()) {
                if (error) {
                    *error = _RrStepHead(program, step) +
                             " names no slot";
                }
                return false;
            }
            if (meta.slotKind[size_t(i)] !=
                RigExecWireSlotKind::FirstFramePose) {
                continue;
            }
            if (scratch->posedAuthored[size_t(i)]) {
                store.base[size_t(i)] = RrFrameFromMatrix(
                    scratch->posedAuthoredM[size_t(i)]);
            } else {
                if (size_t(i) * 11 + 11 > store.avars.size() ||
                    size_t(i) >= scratch->noScaleAvars.size() ||
                    size_t(i) >= scratch->rotOrder.size() ||
                    size_t(i) >= scratch->selfD.size() ||
                    size_t(i) >= scratch->parentDinv.size() ||
                    size_t(i) >= meta.parent.size()) {
                    if (error) {
                        *error = _RrStepHead(program, step) +
                                 " names no slot";
                    }
                    return false;
                }
                const double *a = &store.avars[size_t(i) * 11];
                const double units = a[10];
                const bool noScale =
                    scratch->noScaleAvars[size_t(i)] != 0;
                const RrMat4d avars = _RrComposeAvars(
                    a[0] * units, a[1] * units, a[2] * units,
                    noScale ? 1.0 : a[3], noScale ? 1.0 : a[4],
                    noScale ? 1.0 : a[5], a[6], a[7], a[8], a[9],
                    program->TextOrEmpty(
                        scratch->rotOrder[size_t(i)]));
                const int parent = meta.parent[size_t(i)];
                if (parent >= 0 &&
                    size_t(parent) >= store.posedM.size()) {
                    if (error) {
                        *error = _RrStepHead(program, step) +
                                 " names no slot";
                    }
                    return false;
                }
                const RrMat4d parentPosed =
                    parent >= 0 ? store.posedM[size_t(parent)]
                                : _RrIdentity();
                store.base[size_t(i)] = RrFrameFromMatrix(
                    avars * scratch->selfD[size_t(i)] *
                    scratch->parentDinv[size_t(i)] * parentPosed);
            }
            store.fin[size_t(i)] = store.base[size_t(i)];
            RrMat4d space = _RrIdentity();
            if (!store.base[size_t(i)].IsValid() ||
                store.base[size_t(i)].IsDegenerate() ||
                !RrPointsToMatrix(RrIdentityLandmarks(),
                                  store.base[size_t(i)].points,
                                  &space)) {
                space = _RrIdentity();
                space[3][0] = std::numeric_limits<double>::quiet_NaN();
            }
            store.posedM[size_t(i)] = space;
        }
        return true;
    }

    case RigExecWireStepKind::Solve: {
        return runtimePoseDetail::_RrRunSolveStep(program, step, error);
    }

    case RigExecWireStepKind::SolverCommit: {
        return runtimePoseDetail::_RrRunSolverCommitStep(program, step,
                                                    error);
    }

    case RigExecWireStepKind::Constraint: {
        return runtimePoseDetail::_RrRunConstraintStep(program, step, error);
    }

    case RigExecWireStepKind::CommitDelta: {
        if (wire.object < 0 ||
            size_t(wire.object) >= program->poses->commits.size() ||
            size_t(wire.object) >= store.commits.size()) {
            if (error) {
                *error = _RrStepHead(program, step) + " names no commit";
            }
            return false;
        }
        RrCommitScratch &commit = store.commits[size_t(wire.object)];
        if (!commit.abandoned) {
            return _RrComputeCommitDeltas(
                program, step,
                program->poses->commits[size_t(wire.object)],
                &commit, error);
        }
        return true;
    }

    case RigExecWireStepKind::PropagateChunk: {
        if (wire.object < 0 ||
            size_t(wire.object) >= program->poses->commits.size() ||
            size_t(wire.object) >= store.commits.size()) {
            if (error) {
                *error = _RrStepHead(program, step) + " names no commit";
            }
            return false;
        }
        RrCommitScratch &commit = store.commits[size_t(wire.object)];
        if (commit.abandoned) {
            return true;
        }
        if (wire.part < 0) {
            if (error) {
                *error = _RrStepHead(program, step) + " names no chunk";
            }
            return false;
        }
        const RigExecWireCommit &wireCommit =
            program->poses->commits[size_t(wire.object)];
        const size_t begin =
            size_t(wire.part) * _RrPropagateChunkSize;
        const size_t end =
            std::min(begin + _RrPropagateChunkSize,
                     wireCommit.propagate.size());
        return _RrStageCommitPairs(program, step, wireCommit, &commit,
                                   begin, end, error);
    }

    case RigExecWireStepKind::CommitApply: {
        if (wire.object < 0 ||
            size_t(wire.object) >= program->poses->commits.size() ||
            size_t(wire.object) >= store.commits.size()) {
            if (error) {
                *error = _RrStepHead(program, step) + " names no commit";
            }
            return false;
        }
        return _RrFinishCommit(program, step, size_t(wire.object),
                               error);
    }

    case RigExecWireStepKind::ProviderMatrix: {
        if (wire.object < 0 ||
            size_t(wire.object) >= store.finalMatrix.size() ||
            size_t(wire.object) >= store.baseMatrix.size() ||
            size_t(wire.object) >= store.finLast.size() ||
            size_t(wire.object) >= store.baseLast.size() ||
            size_t(wire.object) >= scratch->restFrames.size()) {
            if (error) {
                *error = _RrStepHead(program, step) + " names no slot";
            }
            return false;
        }
        const size_t slot = size_t(wire.object);
        RrMat4d matrix = _RrIdentity();
        if (wire.part) {
            if (size_t(store.finLast[slot]) >= store.fin.size() ||
                slot >= scratch->restPts.size()) {
                if (error) {
                    *error = _RrStepHead(program, step) +
                             " names no fin version";
                }
                return false;
            }
            const RrPointFrame &frame =
                store.fin[size_t(store.finLast[slot])];
            if (RrFrameUsable(scratch->restFrames[slot]) &&
                RrFrameUsable(frame)) {
                RrPointsToMatrix(scratch->restPts[slot], frame.points,
                                 &matrix);
            }
            store.finalMatrix[slot] = matrix;
        } else {
            if (size_t(store.baseLast[slot]) >= store.base.size() ||
                slot >= scratch->restPts.size()) {
                if (error) {
                    *error = _RrStepHead(program, step) +
                             " names no base version";
                }
                return false;
            }
            const RrPointFrame &frame =
                store.base[size_t(store.baseLast[slot])];
            if (RrFrameUsable(scratch->restFrames[slot]) &&
                RrFrameUsable(frame)) {
                RrPointsToMatrix(scratch->restPts[slot], frame.points,
                                 &matrix);
            }
            store.baseMatrix[slot] = matrix;
        }
        return true;
    }

    case RigExecWireStepKind::PoseInterpolator: {
        if (wire.object < 0 ||
            size_t(wire.object) >= program->poses->poseInterpolators
                                        .size() ||
            size_t(wire.object) >= scratch->interpEnabled.size() ||
            size_t(wire.object) >= scratch->interpSolvers.size() ||
            size_t(wire.object) >= scratch->interpScratch.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no pose interpolator";
            }
            return false;
        }
        const RigExecWirePoseInterpolator &interp =
            program->poses->poseInterpolators[size_t(wire.object)];
        for (int slot = interp.weightBegin; slot < interp.weightEnd;
             ++slot) {
            if (slot < 0 ||
                size_t(slot) >= store.poseWeights.size()) {
                if (error) {
                    *error = _RrStepHead(program, step) +
                             " names no pose weight";
                }
                return false;
            }
            store.poseWeights[size_t(slot)] = 0.0f;
        }
        if (!scratch->interpEnabled[size_t(wire.object)] ||
            interp.poseSlots.empty()) {
            return true;
        }
        if (interp.driverSlot < 0 ||
            size_t(interp.driverSlot) >= store.finLast.size() ||
            size_t(interp.driverSlot) >= scratch->restFrames.size() ||
            size_t(interp.driverSlot) >=
                program->slotMeta->paths.size() ||
            (interp.parentSlot >= 0 &&
             (size_t(interp.parentSlot) >= store.finLast.size() ||
              size_t(interp.parentSlot) >=
                  scratch->restFrames.size()))) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no driver slot";
            }
            return false;
        }
        const size_t d = size_t(interp.driverSlot);
        if (size_t(store.finLast[d]) >= store.fin.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no fin version";
            }
            return false;
        }
        RrQuatd driverFinal(1.0), driverRest(1.0);
        RrQuatd parentFinal(1.0), parentRest(1.0);
        bool usable =
            RrFrameRotation(store.fin[size_t(store.finLast[d])],
                            &driverFinal) &&
            RrFrameRotation(scratch->restFrames[d], &driverRest);
        if (usable && interp.parentSlot >= 0) {
            const size_t p = size_t(interp.parentSlot);
            if (size_t(store.finLast[p]) >= store.fin.size()) {
                if (error) {
                    *error = _RrStepHead(program, step) +
                             " names no fin version";
                }
                return false;
            }
            usable = RrFrameRotation(store.fin[size_t(store.finLast[p])],
                                     &parentFinal) &&
                     RrFrameRotation(scratch->restFrames[p], &parentRest);
        }
        if (!usable) {
            output.diagnostics.push_back(
                "pose interpolator " +
                program->TextOrEmpty(interp.path) +
                " has no usable frame for its driver " +
                program->TextOrEmpty(program->slotMeta->paths[d]) +
                " after the pose walk; its weights are zero this "
                "generation");
            return true;
        }
        const RrQuatd local = parentFinal.GetInverse() * driverFinal;
        const RrQuatd restLocal = parentRest.GetInverse() * driverRest;
        const RrQuatd delta =
            (restLocal.GetInverse() * local).GetNormalized();
        std::vector<double> &interpScratch =
            scratch->interpScratch[size_t(wire.object)];
        scratch->interpSolvers[size_t(wire.object)].Evaluate(
            _RrRbfEulerFromQuaternion(delta), nullptr, &interpScratch,
            interp.allowNegativeWeights);
        if (interpScratch.size() != interp.poseSlots.size()) {
            output.diagnostics.push_back(
                "pose interpolator " +
                program->TextOrEmpty(interp.path) + " solved " +
                std::to_string(interpScratch.size()) + " weights for " +
                std::to_string(interp.poseSlots.size()) + " poses");
            return true;
        }
        for (size_t i = 0; i < interp.poseSlots.size(); ++i) {
            if (interp.poseSlots[i] < 0 ||
                size_t(interp.poseSlots[i]) >= store.poseWeights.size()) {
                if (error) {
                    *error = _RrStepHead(program, step) +
                             " names no pose weight";
                }
                return false;
            }
            store.poseWeights[size_t(interp.poseSlots[i])] =
                static_cast<float>(interpScratch[i]);
        }
        return true;
    }

    case RigExecWireStepKind::SnapshotFinals: {
        const size_t slots = program->slotMeta->paths.size();
        for (size_t i = 0; i < slots; ++i) {
            if (i >= store.finLast.size() ||
                size_t(store.finLast[i]) >= store.fin.size() ||
                i >= scratch->restFrames.size() ||
                i >= scratch->restPts.size()) {
                if (error) {
                    *error = _RrStepHead(program, step) +
                             " names no slot";
                }
                return false;
            }
            const RrPointFrame &frame =
                store.fin[size_t(store.finLast[i])];
            if (!RrFrameUsable(scratch->restFrames[i]) ||
                !RrFrameUsable(frame)) {
                continue;
            }
            RrMat4d matrix = _RrIdentity();
            if (RrPointsToMatrix(scratch->restPts[i], frame.points,
                                 &matrix)) {
                RrSnapshotValue value;
                value.tag = RrSnapshotValue::Tag::Matrix;
                value.matrix = matrix;
                output.snapshots.RecordFinal(
                    program->slotMeta->paths[i], value);
            }
        }
        return true;
    }

    default:
        break;
    }

    if (error) {
        *error = _RrStepHead(program, step) + " not implemented yet";
    }
    return false;
}

} // namespace rigExec
