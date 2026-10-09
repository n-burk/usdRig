// Runtime pose-step dispatch, commit deltas, and propagation.

#include "poseInternal.h"
#include "spaces.h"
#include "rigExecGraph/poseArithmetic.h"
#include <algorithm>
#include <limits>
#include <utility>

namespace rigExec {

using namespace runtimePoseDetail;

namespace {

struct RrSpaceSwitchMath {
    using Matrix=RrMat4d;
    static Matrix RoundTrip(const Matrix &m) { return RrRoundTrip(m); }
    static Matrix Filter(const Matrix &m,const RrVec3d &axis,int filter) {
        return RrFilterSpaceRotation(m,axis,RrRotationFilter(filter));
    }
    static Matrix Blend(const Matrix &a,const Matrix &b,double t) { return RrBlendTransforms(a,b,t); }
    static Matrix Mask(const Matrix &m,const bool *t,const bool *r,const bool *scale) {
        return RrMaskTransform(m,t,r,scale);
    }
};

struct RrPoseInterpolatorMath {
    using Quaternion=RrQuatd; using Vector=RrVec3d;
    static bool FrameRotation(const RrPointFrame &f,Quaternion *q) { return RrFrameRotation(f,q); }
    static bool FrameTranslation(const RrPointFrame &f,const RrPointFrame &rest,
        const RrPointFrame *parent,const RrPointFrame *parentRest,Vector *v) {
        return RrFrameTranslation(f,rest,parent,parentRest,v);
    }
    static Vector EulerFromQuaternion(const Quaternion &q) { return _RrRbfEulerFromQuaternion(q); }
};

struct RrPosePropagateMath {
    static bool Usable(const RrPointFrame &frame) { return RrFrameUsable(frame); }
    static RrPointFrame Carry(const RrPointFrame &frame,const RrMat4d &delta) {
        return RrMatrixToPoints(frame.points,delta);
    }
};

// Propagation outcomes, in RigExecBakedPropagateOutcome order.
enum _RrPropagateOutcome : uint8_t {
    _RrStaged = 0,
    _RrSkipped = 1,
    _RrUnusableDescendant = 3,
    _RrSingularDelta = 4,
    _RrInvalidResult = 5,
};

// RigExecRotationSignFromMask, verbatim: bit N set negates axis N. The
// runtime carries its own copy of the math helpers rather than reaching
// into rigExecMath, which keeps it free of every other project header.
double
_RrRotationSign(unsigned mask, int axis)
{
    return (mask & (1u << axis)) ? -1.0 : 1.0;
}

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
    case RigExecWireStepKind::FrameMatrix:
        return "FrameMatrix";
    default:
        return "Unknown";
    }
}

// Whether a frame record's writer recorded its provider this run
// (RigExecBakedEvalFrameRecord, bakedPose.cpp). A solver commit records a
// slot only when its batch published an element for it, which is that
// position's present byte; a constraint commit records behind the exit
// flags its constraint step set this run, a target past the first only
// while the every-target flag is set.
bool
_RrFrameRecorded(const RrProgram &program, const RrPoseScratch &scratch,
                 const RigExecWireFrameRecord &record)
{
    const size_t commitIndex = size_t(record.commit());
    if (program.poses->commits[commitIndex].solverOutput) {
        const std::vector<char> &present =
            program.store.commits[commitIndex].present;
        return record.position() >= 0 &&
               size_t(record.position()) < present.size() &&
               present[size_t(record.position())];
    }
    return scratch.recordAfter[commitIndex] &&
           !(record.target() > 0 && !scratch.recordEveryTarget[commitIndex]);
}

// Whether slot \p i's avars and ladder rows are there to compose.
bool
_RrComposable(const RrProgram &program, const RrPoseScratch &scratch,
              size_t i)
{
    return i * 11 + 11 <= program.store.avars.size() &&
           i < scratch.noScaleAvars.size() && i < scratch.rotOrder.size() &&
           i < scratch.posedD.size() && i < scratch.parentSpaceM.size() &&
           i < scratch.parentSpaceAuthored.size() && i < scratch.parentDinv.size() &&
           i < scratch.posedAuthored.size() &&
           i < scratch.posedAuthoredM.size();
}

// Slot \p i's avars as one matrix, as the program composes them
// (_ComposeAvarsOf, bakedPose.cpp).
RrMat4d
_RrComposeAvarsOf(const RrProgram &program, const RrPoseScratch &scratch,
                  size_t i)
{
    const double *a = &program.store.avars[i * 11];
    const double units = a[10];
    const bool noScale = scratch.noScaleAvars[i] != 0;
    // avars:rotationSign, applied to the avar where the other two paths
    // apply it (computations.cpp, bakedPose.cpp).
    const unsigned sign =
        i < scratch.rotationSign.size() ? scratch.rotationSign[i] : 0u;
    const double signX = _RrRotationSign(sign, 0);
    return _RrComposeAvars(
        a[0] * units, a[1] * units, a[2] * units, noScale ? 1.0 : a[3],
        noScale ? 1.0 : a[4], noScale ? 1.0 : a[5], a[6] * signX,
        a[7] * _RrRotationSign(sign, 1), a[8] * _RrRotationSign(sign, 2),
        a[9] * signX, program.TextOrEmpty(scratch.rotOrder[i]));
}

// _SpaceOfFrame: an unusable frame selects the NaN sentinel, so the
// failure survives into every descendant instead of being scrubbed into a
// plausible identity.
RrMat4d
_RrSpaceOfFrame(const RrPointFrame &frame)
{
    RrMat4d space = _RrIdentity();
    if (!frame.IsValid() || frame.IsDegenerate() ||
        !RrPointsToMatrix(RrIdentityLandmarks(), frame.points, &space)) {
        space = _RrIdentity();
        space[3][0] = std::numeric_limits<double>::quiet_NaN();
    }
    return space;
}

// Whether every slot \p read names is there to read or recompose.
bool
_RrVersionReadable(const RrProgram &program, const RrPoseScratch &scratch,
                   const RigExecWireFrameVersion &read)
{
    if (read.context >= 0) {
        return size_t(read.context) < program.store.switchFrames.size();
    }
    if (read.anchor >= 0 &&
        size_t(read.anchor) >= program.store.posedM.size()) {
        return false;
    }
    return read.recompose.empty();
}

// Read an explicit checkpoint output or the bound current provider matrix.
RrMat4d
_RrReadFrameVersion(const RrProgram &program, const RrPoseScratch &scratch,
                    const RigExecWireFrameVersion &read)
{
    if (read.context >= 0) {
        return program.store.switchFrames[size_t(read.context)];
    }
    RrMat4d posed = read.anchor >= 0
                        ? program.store.posedM[size_t(read.anchor)]
                        : _RrIdentity();
    return posed;
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
        bool parentBlocked = false;
        const auto *pose = _RrScratch(program);
        for (int slot = commit.propagate[k].first;
             slot >= 0 && slot != commit.propagate[k].second;
             slot = program->slotMeta->propParent[size_t(slot)]) {
            if (pose->parentSpaceAuthored[size_t(slot)]) {
                parentBlocked = true;
                break;
            }
        }
        if (parentBlocked || pos < 0 || size_t(pos) >= scratch->present.size() ||
            !scratch->present[size_t(pos)]) {
            scratch->outcome[k] = _RrSkipped;
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
        RrPointFrame frame;
        const auto outcome=RigExecPropagatePoseArithmetic<RrPosePropagateMath>(
            current,before,scratch->frames[size_t(pos)],scratch->deltas[size_t(pos)],
            scratch->deltaOk[size_t(pos)]==1,commit.solverOutput,false,true,&frame);
        scratch->outcome[k]=uint8_t(outcome);
        if(outcome==RigExecPosePropagateOutcome::Staged) scratch->staged[k]=frame;
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
    const RigExecWireCommit &commit =
        program->poses->commits[commitIndex];
    RrCommitScratch &live = store.commits[commitIndex];
    RrStepOutput &output = store.stepOutputs[step];
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
    if (live.abandoned) {
        return carryEverything(error);
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
        return carryEverything(error);
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
    return true;
}

} // namespace runtimePoseDetail

bool
RrRunPoseStep(RrProgram *program, size_t step, std::string *error)
{
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
        const auto noSlot = [&]() {
            if (error) {
                *error = _RrStepHead(program, step) + " names no slot";
            }
            return false;
        };
        for (int i = group.begin; i < group.end; ++i) {
            if (i < 0 || size_t(i) >= meta.slotKind.size() ||
                size_t(i) >= store.base.size() ||
                size_t(i) >= store.fin.size() ||
                size_t(i) >= store.posedM.size() ||
                size_t(i) >= scratch->restM.size()) {
                return noSlot();
            }
            if (meta.slotKind[size_t(i)] !=
                RigExecWireSlotKind::FirstFramePose) {
                continue;
            }
            if (!meta.providerActive.empty() && !meta.providerActive[size_t(i)]) {
                RrMat4d unavailable(1);
                unavailable[3][0] = std::numeric_limits<double>::quiet_NaN();
                store.base[size_t(i)] = RrFrameFromMatrix(RrMat4d(1));
                store.base[size_t(i)].flags = 0;

            store.fin[size_t(i)] = store.base[size_t(i)];
                store.posedM[size_t(i)] = unavailable;
                continue;
            }
            if (scratch->posedAuthored[size_t(i)]) {
                store.base[size_t(i)] = RrFrameFromMatrix(
                    scratch->posedAuthoredM[size_t(i)]);
            } else {
                if (!_RrComposable(*program, *scratch, size_t(i)) ||
                    size_t(i) >= scratch->defaultRoundTrip.size() ||
                    size_t(i) >= meta.parent.size()) {
                    return noSlot();
                }
                const RrMat4d avars =
                    _RrComposeAvarsOf(*program, *scratch, size_t(i));
                const int parent = meta.parent[size_t(i)];
                // One int per slot, and only on a rig that has a switch
                // at all: the ordinary compose falls straight through.
                const int switchIndex =
                    program->spaceSwitchBySlot.empty()
                        ? -1
                        : program->spaceSwitchBySlot[size_t(i)];
                if (switchIndex < 0) {
                    if (parent >= 0 &&
                        size_t(parent) >= store.posedM.size()) {
                        return noSlot();
                    }
                    const RrMat4d parentPosed =
                        parent >= 0 ? store.posedM[size_t(parent)]
                                    : _RrIdentity();
                    store.base[size_t(i)] = RrFrameFromMatrix(
                        avars * scratch->posedD[size_t(i)] *
                        scratch->parentDinv[size_t(i)] *
                        (scratch->parentSpaceAuthored[size_t(i)]
                            ? scratch->parentSpaceM[size_t(i)] : parentPosed));
                } else {
                    // A switched slot composes against the SELECTED
                    // source's pair of spaces instead of its namespace
                    // parent's. Both halves come from one source, so at
                    // rest they cancel and no space moves the rig
                    // standing still. Every provider frame it reads is
                    // read at the version the program bound at Build,
                    // never as whatever the slot holds now: the step
                    // order does not place the namespace parent's
                    // posedM before this step.
                    const RigExecWireSpaceSwitch &sw =
                        program->poses->spaceSwitches[size_t(switchIndex)];
                    bool readable =
                        sw.sourceReads.size() == sw.sourceSlots.size() &&
                        _RrVersionReadable(*program, *scratch,
                                           *sw.parentRead) &&
                        _RrVersionReadable(*program, *scratch,
                                           *sw.spaceRead);
                    for (const RigExecWireFrameVersion &read :
                         sw.sourceReads) {
                        readable = readable &&
                                   _RrVersionReadable(*program, *scratch,
                                                      read);
                    }
                    if (!readable ||
                        (parent >= 0 &&
                         size_t(parent) >=
                             scratch->defaultRoundTrip.size())) {
                        return noSlot();
                    }
                    // `local` is avars * default:space reached the long
                    // way round -- compose the UNSWITCHED world, then
                    // divide the namespace parent back out -- rather than
                    // as `avars * selfD`, which is the same quantity in
                    // exact arithmetic and NOT the same in doubles. The
                    // program can only reach it this way, the two answers
                    // are compared bit for bit, and 4e-15 of disagreement
                    // here propagates to every descendant. The parent is
                    // read before its own switch when that resolves in
                    // this switch's round or later.
                    const RrMat4d parentPosed =
                        _RrReadFrameVersion(*program, *scratch,
                                            *sw.parentRead);
                    const RrMat4d parentDefault =
                        parent >= 0
                            ? scratch->defaultRoundTrip[size_t(parent)]
                            : _RrIdentity();
                    RigExecSpaceSwitchRecordT<RrVec3d> record;
                    record.filters.assign(sw.filters.begin(),sw.filters.end());
                    record.twistAxis=RrVec3d(sw.twistAxis[0],sw.twistAxis[1],sw.twistAxis[2]);
                    std::copy(sw.affectTranslation.begin(),sw.affectTranslation.end(),record.affectTranslation.begin());
                    std::copy(sw.affectRotation.begin(),sw.affectRotation.end(),record.affectRotation.begin());
                    std::copy(sw.affectScale.begin(),sw.affectScale.end(),record.affectScale.begin());
                    RigExecSpaceSwitchInputsT<RrMat4d> input;
                    input.avars=avars; input.posedDefault=scratch->posedD[size_t(i)];
                    input.parentDefaultInverse=scratch->parentDinv[size_t(i)];
                    input.parentPosed=parentPosed; input.parentDefault=parentDefault;
                    const auto selector=program->ReadSpaceSwitch(size_t(switchIndex));
                    input.active=selector.f64;
                    if(sw.tokenIndex) {
                        input.active=0.0;
                        const auto text=program->TextOrEmpty(selector.token);
                        const auto found=std::find(sw.labels.begin(),sw.labels.end(),text);
                        if(found!=sw.labels.end()) input.active=double(found-sw.labels.begin());
                    }
                    input.hasCarry=sw.spaceSlot>=0;
                    if(input.hasCarry) {
                        input.spaceDefault=scratch->defaultRoundTrip[size_t(sw.spaceSlot)];
                        input.spacePosed=_RrReadFrameVersion(*program,*scratch,*sw.spaceRead);
                    }
                    for(size_t source=0;source<sw.sourceSlots.size();++source) {
                        RigExecSpaceSwitchSourceT<RrMat4d> value;
                        const int slot=sw.sourceSlots[source]; value.world=slot<0;
                        if(slot>=0) {
                            value.defaultSpace=scratch->defaultRoundTrip[size_t(slot)];
                            value.posedSpace=_RrReadFrameVersion(*program,*scratch,sw.sourceReads[source]);
                        }
                        input.sources.push_back(value);
                    }
                    RrMat4d output;
                    if(!RigExecRunSpaceSwitchArithmetic<RrSpaceSwitchMath>(record,input,&output)) {
                        if(error) *error=_RrStepHead(program,step)+" names no space";
                        return false;
                    }
                    store.base[size_t(i)]=RrFrameFromMatrix(output);
                }
            }
            for(size_t acIndex=0;acIndex<program->poses->autoClavicles.size();++acIndex) {
                const auto &ac=program->poses->autoClavicles[acIndex];if(ac.slot!=i)continue;
                std::array<RrMat4d,13> matrices;matrices[0]=_RrSpaceOfFrame(store.base[size_t(i)]);
                bool usable=(store.base[size_t(i)].flags&RrPointFrameValid) && !(store.base[size_t(i)].flags&RrPointFrameDegenerate);
                for(size_t k=1;k<13;++k) {
                    const auto &read=ac.frames[k];if(read.slot<0)continue;
                    if(read.computation==1)matrices[k]=scratch->defaultRoundTrip[size_t(read.slot)];
                    else if(read.computation==2)matrices[k]=_RrSpaceOfFrame(scratch->restFrames[size_t(read.slot)]);
                    else if(!read.recompose.empty() || read.slot==i) {
                        matrices[k]=matrices[0];
                        for(int child:read.recompose) {
                            if(scratch->posedAuthored[size_t(child)])matrices[k]=scratch->posedAuthoredM[size_t(child)];
                            else matrices[k]=RrRoundTrip(_RrComposeAvarsOf(*program,*scratch,size_t(child))*scratch->posedD[size_t(child)]*
                                scratch->parentDinv[size_t(child)]*(scratch->parentSpaceAuthored[size_t(child)]?scratch->parentSpaceM[size_t(child)]:matrices[k]));
                        }
                    } else matrices[k]=_RrSpaceOfFrame(store.fin[store.finLast[size_t(read.slot)]]);
                    for(int row=0;row<4;++row)for(int col=0;col<4;++col)usable=usable && std::isfinite(matrices[k][row][col]);
                }
                if(!usable)continue;
                RigExecAutoClavicleConstants constants;
                std::copy(ac.basis.begin(),ac.basis.end(),constants.basis);
                constants.ikValue=ac.ikValue;constants.gain=ac.gain;constants.kernel=ac.kernel;
                constants.normalize=ac.normalize;constants.swings=ac.swings;constants.widths=ac.widths;
                constants.gains=ac.gains;constants.weights=ac.weights;
                const auto scalar=[&](size_t k,double fallback) {
                    const int index=ac.scalarIndices[k];if(index<0)return fallback;
                    const auto value=program->ReadRegistered(program->autoClavicleRead[acIndex][size_t(index)]);
                    return value.tag==RrInputTag::Float?double(value.f32):value.f64;
                };
                RigExecAutoClavicleFrames frames;
                frames.targetPosed=&matrices[0]._mtx[0][0];frames.pivotPosed=&matrices[1]._mtx[0][0];
                frames.anchorPosed=&matrices[2]._mtx[0][0];frames.anchorDefault=&matrices[3]._mtx[0][0];
                frames.fkPosed=&matrices[4]._mtx[0][0];
                for(size_t k=0;k<3;++k)frames.fkDefault[k]=&matrices[5+k]._mtx[0][0];
                if(ac.frames[8].slot>=0)frames.ikTargetPosed=&matrices[8]._mtx[0][0];
                if(ac.frames[9].slot>=0)frames.polePosed=&matrices[9]._mtx[0][0];
                frames.ikBlend=scalar(0,1-constants.ikValue);frames.amount=scalar(1,1);frames.hasLimb=ac.hasLimb;
                if(frames.hasLimb) {
                    frames.limb.stretch=scalar(2,1);frames.limb.pin=scalar(3,0);
                    frames.limb.upperScale=scalar(4,1);frames.limb.lowerScale=scalar(5,1);
                    frames.limb.softDistance=scalar(6,0);frames.limb.scaleCalibration=scalar(7,0);
                    frames.twistRadians=scalar(8,0)*_RrPi/180.0;
                    frames.limbRestUpper=(matrices[11].ExtractTranslation()-matrices[10].ExtractTranslation()).GetLength()+scalar(9,0);
                    frames.limbRestLower=(matrices[12].ExtractTranslation()-matrices[11].ExtractTranslation()).GetLength()+scalar(10,0);
                }
                double delta[3];RigExecAutoClavicleShift(constants,frames,delta);
                if(delta[0]!=0 || delta[1]!=0 || delta[2]!=0) {
                    auto shifted=matrices[0];shifted.SetTranslateOnly(shifted.ExtractTranslation()+RrVec3d(delta[0],delta[1],delta[2]));
                    store.base[size_t(i)]=RrFrameFromMatrix(shifted);
                }
            }
            store.fin[size_t(i)] = store.base[size_t(i)];
            store.posedM[size_t(i)] = _RrSpaceOfFrame(store.base[size_t(i)]);
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
                size_t(store.baseLast[slot]) >= store.base.size() ||
                slot >= scratch->restPts.size()) {
                if (error) {
                    *error = _RrStepHead(program, step) +
                             " names no fin version";
                }
                return false;
            }
            const RrPointFrame &frame =
                store.fin[size_t(store.finLast[slot])];
            const bool converted = RrFrameUsable(scratch->restFrames[slot]) &&
                RrFrameUsable(frame) &&
                RrPointsToMatrix(scratch->restPts[slot], frame.points, &matrix);
            // Mirror the native provider admission: an unavailable final
            // frame retains identity rather than entering the base fallback.
            if (!converted && RrFrameUsable(frame)) {
                const RrPointFrame &base = store.base[size_t(store.baseLast[slot])];
                RrMat4d execM = _RrIdentity();
                if (scratch->restFrames[slot].IsValid() && base.IsValid())
                    RrPointsToMatrix(scratch->restPts[slot], base.points, &execM);
                matrix = execM;
                const auto &jointSlots = program->slotMeta->jointSlots;
                if (std::find(jointSlots.begin(), jointSlots.end(), int32_t(slot)) != jointSlots.end()) {
                    RrMat4d delta = _RrIdentity();
                    if (RrPointsToMatrix(base.points, frame.points, &delta))
                        matrix = execM * delta;
                }
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

    case RigExecWireStepKind::FrameMatrix: {
        // RigExecBakedEvalFrameRecord: the record's provider frame, as its
        // writer left it, converted against the slot's rest landmarks (the
        // identity landmarks where the rest is unusable). Identity and 0
        // where the writer did not record this run or its frame does not
        // convert. Both fields every run, so a reader never sees a valid
        // byte from another run beside this run's matrix.
        const std::vector<RigExecWireFrameRecord> &records =
            program->poses->frameRecords;
        if (wire.object < 0 || size_t(wire.object) >= records.size() ||
            size_t(wire.object) >= store.frameMatrix.size() ||
            size_t(wire.object) >= store.frameMatrixValid.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no frame record";
            }
            return false;
        }
        const size_t object = size_t(wire.object);
        const RigExecWireFrameRecord &record = records[object];
        const size_t commitIndex = size_t(record.commit());
        if (commitIndex >= program->poses->commits.size() ||
            commitIndex >= store.commits.size() ||
            commitIndex >= scratch->recordAfter.size() ||
            commitIndex >= scratch->recordEveryTarget.size()) {
            if (error) {
                *error = _RrStepHead(program, step) + " names no commit";
            }
            return false;
        }
        if (size_t(record.version()) >= store.fin.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no fin version";
            }
            return false;
        }
        if (size_t(record.slot()) >= scratch->restFrames.size()) {
            if (error) {
                *error = _RrStepHead(program, step) + " names no slot";
            }
            return false;
        }
        RrMat4d matrix = _RrIdentity();
        bool valid = false;
        const RrPointFrame &frame = store.fin[size_t(record.version())];
        if (_RrFrameRecorded(*program, *scratch, record) &&
            frame.IsValid()) {
            const RrPointFrame &rest =
                scratch->restFrames[size_t(record.slot())];
            const std::array<RrVec3d, 4> landmarks =
                rest.IsValid() ? rest.points : RrIdentityLandmarks();
            valid = RrPointsToMatrix(landmarks, frame.points, &matrix);
        }
        store.frameMatrix[object] = matrix;
        store.frameMatrixValid[object] = valid ? 1 : 0;
        return true;
    }

    case RigExecWireStepKind::SpaceCheckpoint: {
        if(wire.object<0 || size_t(wire.object)>=program->poses->spaceCheckpoints.size()) return false;
        const auto &context=program->poses->spaceCheckpoints[size_t(wire.object)];
        struct CheckpointMath {
            using Matrix=RrMat4d;
            static Matrix RoundTripCheckpoint(const Matrix &matrix) {
                return _RrSpaceOfFrame(RrFrameFromMatrix(matrix));
            }
        };
        auto &inputs=scratch->checkpointInputs[size_t(wire.object)];
        for(size_t position=0;position<context.recompose.size();++position) {
            const size_t i=size_t(context.recompose[position]);
            auto &input=inputs[position];
            input.avars=_RrComposeAvarsOf(*program,*scratch,i);
            input.posedDefault=scratch->posedD[i];
            input.parentDefaultInverse=scratch->parentDinv[i];
            input.parentExpression=scratch->parentSpaceM[i];
            input.posedAuthoredMatrix=scratch->posedAuthoredM[i];
            input.parentExpressionAuthored=scratch->parentSpaceAuthored[i]!=0;
            input.posedAuthored=scratch->posedAuthored[i]!=0;
        }
        const RrMat4d ancestor=context.anchor>=0 ? program->store.posedM[size_t(context.anchor)] : _RrIdentity();
        program->store.switchFrames[size_t(wire.object)]=
            RigExecRunSpaceCheckpointArithmetic<CheckpointMath>(ancestor,inputs);
        return true;
    }
    case RigExecWireStepKind::AvarInputs: {
        const auto &state=program->inputState;
        const auto range=RrAvarReadRange(state,wire.object);
        for(uint32_t i=range.first;i<range.second;++i) {
            const uint32_t read=state.avarReads[i];
            const int avar=program->registeredReads[read].avar;
            store.avars[size_t(avar)]=program->ReadRegistered(int32_t(read)).f64;
        }
        return true;
    }

    case RigExecWireStepKind::ProviderRefresh:
        return RrRunProviderRefresh(program,wire,error);

    case RigExecWireStepKind::SpaceExpression:
        return RrRunSpaceExpression(program, wire, error);

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
        // Resolve current producer outputs in the consuming graph body.
        const size_t object = size_t(wire.object);
        scratch->interpEnabled[object] =
            program->ReadInterp(object).boolean ? 1 : 0;
        for (size_t v = 0; v < interp.valueInputs.size() && v < 3; ++v) {
            scratch->interpValues[object][v] =
                program->ReadInterpValue(object, v).f64;
        }
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
        // A NUMERIC driver reads dials, not a frame: it names no driver
        // prim and takes no slot, and its rotation channel is the
        // identity, so every frame lookup below is skipped.
        const bool numeric = !interp.valueInputs.empty();
        if (!numeric &&
            (interp.driverSlot < 0 ||
             size_t(interp.driverSlot) >= store.finLast.size() ||
             size_t(interp.driverSlot) >= scratch->restFrames.size() ||
             size_t(interp.driverSlot) >=
                 program->slotMeta->paths.size() ||
             (interp.parentSlot >= 0 &&
              (size_t(interp.parentSlot) >= store.finLast.size() ||
               size_t(interp.parentSlot) >=
                   scratch->restFrames.size())))) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no driver slot";
            }
            return false;
        }
        const size_t d = size_t(numeric ? 0 : interp.driverSlot);
        if (!numeric && size_t(store.finLast[d]) >= store.fin.size()) {
            if (error) {
                *error = _RrStepHead(program, step) +
                         " names no fin version";
            }
            return false;
        }
        RigExecPoseInterpolatorInputsT<RrPointFrame> input;
        input.enabled=true;
        if (numeric) {
            for(size_t k=0;k<interp.valueInputs.size() && k<3;++k)
                input.numeric.push_back(scratch->interpValues[object][k]);
        } else {
            input.driverFinal=&store.fin[size_t(store.finLast[d])];
            input.driverRest=&scratch->restFrames[d];
            if(interp.parentSlot>=0) {
                const size_t p=size_t(interp.parentSlot);
                if(size_t(store.finLast[p])>=store.fin.size()) {
                    if(error) *error=_RrStepHead(program,step)+" names no fin version";
                    return false;
                }
                input.parentFinal=&store.fin[size_t(store.finLast[p])];
                input.parentRest=&scratch->restFrames[p];
            }
        }
        auto &solver=scratch->interpSolvers[object];
        RigExecPoseInterpolatorRecordT<decltype(solver)> record{
            solver,interp.enableTranslation,interp.allowNegativeWeights,interp.poseSlots.size()};
        std::vector<double> &interpScratch=scratch->interpScratch[object];
        const auto status=RigExecRunPoseInterpolatorArithmetic<RrPoseInterpolatorMath>(
            record,input,&interpScratch);
        if(status==RigExecPoseInterpolatorStatus::UnusableRotation) {
            output.diagnostics.push_back("pose interpolator "+program->TextOrEmpty(interp.path)+
                " has no usable frame for its driver "+program->TextOrEmpty(program->slotMeta->paths[d])+
                " after the pose walk; its weights are zero this generation");
            return true;
        }
        if(status==RigExecPoseInterpolatorStatus::UnusableTranslation) {
            output.diagnostics.push_back("pose interpolator "+program->TextOrEmpty(interp.path)+
                " could not measure its driver's translation; its weights are zero this generation");
            return true;
        }
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

    default:
        break;
    }

    if (error) {
        *error = _RrStepHead(program, step) + " not implemented yet";
    }
    return false;
}

} // namespace rigExec
