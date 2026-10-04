// Runtime pose scratch allocation, input prologue, and ladder composition.

#include "poseInternal.h"
#include <algorithm>
#include <limits>
#include <utility>

namespace rigExec {

using namespace runtimePoseDetail;

namespace {

// Smallest supported avar scale magnitude, in avarScale.h terms.
constexpr double _RrAvarScaleFloor = 1e-4;

// RigExecNormalizeAvarScale, verbatim: non-finite resolves to identity,
// magnitudes below the floor are raised to it with the sign intact.
double
_RrNormalizeAvarScale(double value)
{
    if (!std::isfinite(value)) {
        return 1.0;
    }
    return std::abs(value) < _RrAvarScaleFloor
        ? std::copysign(_RrAvarScaleFloor, value)
        : value;
}

RrMat4d
_RrWireMatrix(const RigExecWireMatrix4d &wire)
{
    RrMat4d m;
    for (size_t r = 0; r < 4; ++r) {
        for (size_t c = 0; c < 4; ++c) {
            m[r][c] = wire[r * 4 + c];
        }
    }
    return m;
}

std::array<RrVec3d, 4>
_RrWireLandmarks(const std::array<RigExecWireVec3d, 4> &wire)
{
    std::array<RrVec3d, 4> out;
    for (size_t i = 0; i < 4; ++i) {
        out[i] = RrVec3d(wire[i][0], wire[i][1], wire[i][2]);
    }
    return out;
}

// RigExecBakedComposeLadder (bakedPose.cpp), over the scratch tables.
// Reads through ReadLadder, the runtime form of RigExecBakedRead.
void
_RrComposeLadder(RrProgram *program, bool trackMoves)
{
    RrStore &store = program->store;
    RrPoseScratch *scratch = _RrScratch(program);
    const RigExecWireSlotMeta &meta = *program->slotMeta;
    const size_t slots = meta.paths.size();
    const RrMat4d identity = _RrIdentity();
    if (trackMoves) {
        store.ladderMovedSlots.clear();
    }
    for (size_t i = 0; i < slots; ++i) {
        if (i >= meta.slotKind.size() ||
            meta.slotKind[i] != RigExecWireSlotKind::FirstFramePose) {
            continue;
        }
        const RrMat4d posed =
            program->ReadLadder(i, RrLadderPosedSpace).matrix;
        scratch->posedAuthored[i] = posed != identity ? 1 : 0;
        scratch->posedAuthoredM[i] = posed;

        const RrInputValue restAvar0 =
            program->ReadLadder(i, RrLadderRestAvar0 + 0);
        const RrInputValue restAvar1 =
            program->ReadLadder(i, RrLadderRestAvar0 + 1);
        const RrInputValue restAvar2 =
            program->ReadLadder(i, RrLadderRestAvar0 + 2);
        const RrInputValue restAvar3 =
            program->ReadLadder(i, RrLadderRestAvar0 + 3);
        const RrInputValue restAvar4 =
            program->ReadLadder(i, RrLadderRestAvar0 + 4);
        const RrInputValue restAvar5 =
            program->ReadLadder(i, RrLadderRestAvar0 + 5);
        RrMat4d rest =
            _RrComposeAvars(restAvar0.f64, restAvar1.f64, restAvar2.f64,
                            1, 1, 1, restAvar3.f64, restAvar4.f64,
                            restAvar5.f64, 0, "XYZ") *
            program->ReadLadder(i, RrLadderRestSpace).matrix;
        rest.Orthonormalize(false);
        const int parent =
            i < meta.parent.size() ? meta.parent[i] : -1;
        const RrMat4d parentRest =
            parent >= 0 ? scratch->restRoundTrip[size_t(parent)]
                        : identity;
        scratch->restM[i] = rest * parentRest;
        scratch->restFrames[i] = RrFrameFromMatrix(scratch->restM[i]);
        scratch->restPts[i] = scratch->restFrames[i].points;
        scratch->restRoundTrip[i] = RrRoundTrip(scratch->restM[i]);

        const RrMat4d authoredDefault =
            program->ReadLadder(i, RrLadderDefaultSpace).matrix;
        const RrMat4d parentDefault =
            parent >= 0 ? scratch->defaultRoundTrip[size_t(parent)]
                        : identity;
        if (authoredDefault != identity) {
            scratch->selfD[i] = authoredDefault;
        } else {
            const RrInputValue defAvar0 =
                program->ReadLadder(i, RrLadderDefaultAvar0 + 0);
            const RrInputValue defAvar1 =
                program->ReadLadder(i, RrLadderDefaultAvar0 + 1);
            const RrInputValue defAvar2 =
                program->ReadLadder(i, RrLadderDefaultAvar0 + 2);
            const RrInputValue defAvar3 =
                program->ReadLadder(i, RrLadderDefaultAvar0 + 3);
            const RrInputValue defAvar4 =
                program->ReadLadder(i, RrLadderDefaultAvar0 + 4);
            const RrInputValue defAvar5 =
                program->ReadLadder(i, RrLadderDefaultAvar0 + 5);
            const RrMat4d offset = _RrComposeAvars(
                defAvar0.f64, defAvar1.f64, defAvar2.f64, 1, 1, 1,
                defAvar3.f64, defAvar4.f64, defAvar5.f64, 0, "XYZ");
            scratch->selfD[i] = offset * scratch->restRoundTrip[i] *
                                parentRest.GetInverse() * parentDefault;
        }
        scratch->defaultRoundTrip[i] = RrRoundTrip(scratch->selfD[i]);
        scratch->parentDinv[i] = parentDefault.GetInverse();
        const uint32_t orderId =
            program->ReadLadder(i, RrLadderRotationOrder).token;
        scratch->rotOrder[i] = orderId;

        if (i < store.ladders.size()) {
            RrLadderLive &live = store.ladders[i];
            live.restSpace =
                program->ReadLadder(i, RrLadderRestSpace).matrix;
            live.defaultSpace =
                program->ReadLadder(i, RrLadderDefaultSpace).matrix;
            live.posedSpace = posed;
            live.restAvars[0] = restAvar0.f64;
            live.restAvars[1] = restAvar1.f64;
            live.restAvars[2] = restAvar2.f64;
            live.restAvars[3] = restAvar3.f64;
            live.restAvars[4] = restAvar4.f64;
            live.restAvars[5] = restAvar5.f64;
            live.defaultAvars[0] =
                program->ReadLadder(i, RrLadderDefaultAvar0 + 0).f64;
            live.defaultAvars[1] =
                program->ReadLadder(i, RrLadderDefaultAvar0 + 1).f64;
            live.defaultAvars[2] =
                program->ReadLadder(i, RrLadderDefaultAvar0 + 2).f64;
            live.defaultAvars[3] =
                program->ReadLadder(i, RrLadderDefaultAvar0 + 3).f64;
            live.defaultAvars[4] =
                program->ReadLadder(i, RrLadderDefaultAvar0 + 4).f64;
            live.defaultAvars[5] =
                program->ReadLadder(i, RrLadderDefaultAvar0 + 5).f64;
            live.rotationOrder = orderId;
        }

        if (!trackMoves) {
            continue;
        }
        if (scratch->restM[i] != scratch->lastRestM[i] ||
            scratch->selfD[i] != scratch->lastSelfD[i] ||
            scratch->parentDinv[i] != scratch->lastParentDinv[i] ||
            scratch->posedAuthored[i] != scratch->lastPosedAuthored[i] ||
            scratch->posedAuthoredM[i] != scratch->lastPosedAuthoredM[i] ||
            scratch->rotOrder[i] != scratch->lastRotOrder[i]) {
            store.ladderMovedSlots.push_back(int(i));
            scratch->lastRestM[i] = scratch->restM[i];
            scratch->lastSelfD[i] = scratch->selfD[i];
            scratch->lastParentDinv[i] = scratch->parentDinv[i];
            scratch->lastPosedAuthored[i] = scratch->posedAuthored[i];
            scratch->lastPosedAuthoredM[i] = scratch->posedAuthoredM[i];
            scratch->lastRotOrder[i] = scratch->rotOrder[i];
        }
    }
}

} // namespace

namespace runtimePoseDetail {

RrMat4d
_RrIdentity()
{
    RrMat4d m;
    m.SetIdentity();
    return m;
}

// RigExecBakedComposeAvars (bakedProgramImpl.h), written the same way
// rather than algebraically simplified. `order` is the resolved token
// text; anything but a 3-letter sequence composes XYZ.
RrMat4d
_RrComposeAvars(double tx, double ty, double tz, double sx, double sy,
                double sz, double rx, double ry, double rz, double rspin,
                const std::string &order)
{
    static const RrVec3d axes[3] = {
        RrVec3d(1, 0, 0), RrVec3d(0, 1, 0), RrVec3d(0, 0, 1)};
    const double angles[3] = {rx, ry, rz};
    std::string sequence = order;
    if (sequence.size() != 3) {
        sequence = "XYZ";
    }
    RrMat4d m;
    m.SetIdentity();
    m[0][0] = _RrNormalizeAvarScale(sx);
    m[1][1] = _RrNormalizeAvarScale(sy);
    m[2][2] = _RrNormalizeAvarScale(sz);
    for (const char axis : sequence) {
        const int index = axis == 'X' ? 0 : axis == 'Y' ? 1 : 2;
        if (angles[index] != 0.0) {
            m = m * RrMat4d(RrRotation(axes[index], angles[index]),
                            RrVec3d(0));
        }
    }
    if (rspin != 0.0) {
        m = m * RrMat4d(RrRotation(axes[0], rspin), RrVec3d(0));
    }
    RrMat4d t;
    t.SetIdentity();
    t.SetTranslateOnly(RrVec3d(tx, ty, tz));
    return m * t;
}

// The step body's `live`: whether a baked parameter has to be re-read
// at all (bakedPose.cpp): read per run, or an override stands on its walk.
bool
_RrLive(const RrProgram *program, const v4::RigExecWireInput &read)
{
    if ((read.flags & uint8_t(v4::InputReadFlags::Varying)) != 0) {
        return true;
    }
    return read.overrideIndex >= 0 &&
           size_t(read.overrideIndex) < program->store.overridden.size() &&
           program->store.overridden[size_t(read.overrideIndex)];
}

bool
_RrLiveSolver(const RrProgram *program, size_t solver, int field)
{
    return _RrLive(program, program->RegisteredInput(
                                program->solverRead[solver][size_t(field)]));
}

RrPoseScratch *
_RrScratch(RrProgram *program)
{
    return static_cast<RrPoseScratch *>(program->pose.get());
}

} // namespace runtimePoseDetail

bool
RrPoseSizeScratch(RrProgram *program, std::string *error)
{
    if (!program || !program->slotMeta || !program->constants ||
        !program->poses || !program->geometry) {
        if (error) {
            *error = "pose scratch needs the program tables";
        }
        return false;
    }
    const RigExecWireSlotMeta &meta = *program->slotMeta;
    const RigExecWireConstants &constants = *program->constants;
    const RigExecWireDomainPose &poses = *program->poses;
    const size_t slots = meta.paths.size();
    if (constants.restM.size() != slots ||
        constants.restPts.size() != slots ||
        constants.restFrames.size() != slots ||
        constants.selfD.size() != slots ||
        constants.parentDinv.size() != slots ||
        constants.rotOrder.size() != slots ||
        constants.restRoundTrip.size() != slots ||
        constants.defaultRoundTrip.size() != slots ||
        constants.posedAuthored.size() != slots ||
        constants.posedAuthoredM.size() != slots ||
        constants.noScaleAvars.size() != slots) {
        if (error) {
            *error = "pose constants do not cover the slots";
        }
        return false;
    }
    RrPoseScratch *scratch = new RrPoseScratch();
    const RrMat4d identity = _RrIdentity();
    scratch->restM.assign(slots, identity);
    scratch->selfD.assign(slots, identity);
    scratch->parentDinv.assign(slots, identity);
    scratch->posedAuthoredM.assign(slots, identity);
    scratch->restRoundTrip.assign(slots, identity);
    scratch->defaultRoundTrip.assign(slots, identity);
    scratch->restPts.assign(slots, RrIdentityLandmarks());
    scratch->restFrames.assign(slots, RrPointFrame());
    scratch->rotOrder.assign(slots, 0);
    scratch->posedAuthored.assign(slots, 0);
    scratch->noScaleAvars.assign(slots, 0);
    // Empty on a minor-1 binary, which is every axis +1.
    scratch->rotationSign.assign(slots, 0);
    for (size_t i = 0; i < slots; ++i) {
        scratch->restM[i] = _RrWireMatrix(constants.restM[i]);
        scratch->selfD[i] = _RrWireMatrix(constants.selfD[i]);
        scratch->parentDinv[i] = _RrWireMatrix(constants.parentDinv[i]);
        scratch->posedAuthoredM[i] =
            _RrWireMatrix(constants.posedAuthoredM[i]);
        scratch->restRoundTrip[i] =
            _RrWireMatrix(constants.restRoundTrip[i]);
        scratch->defaultRoundTrip[i] =
            _RrWireMatrix(constants.defaultRoundTrip[i]);
        scratch->restPts[i] = _RrWireLandmarks(constants.restPts[i]);
        scratch->restFrames[i] = RrWireToFrame(constants.restFrames[i]);
        scratch->rotOrder[i] = constants.rotOrder[i];
        scratch->posedAuthored[i] =
            constants.posedAuthored[i] ? 1 : 0;
        scratch->noScaleAvars[i] =
            constants.noScaleAvars[i] ? 1 : 0;
        scratch->rotationSign[i] = i < constants.rotationSign.size()
            ? (unsigned char)(constants.rotationSign[i] & 7u) : 0;
    }
    scratch->lastRestM = scratch->restM;
    scratch->lastSelfD = scratch->selfD;
    scratch->lastParentDinv = scratch->parentDinv;
    scratch->lastPosedAuthoredM = scratch->posedAuthoredM;
    scratch->lastPosedAuthored = scratch->posedAuthored;
    scratch->lastRotOrder = scratch->rotOrder;
    scratch->interpEnabled.assign(poses.poseInterpolators.size(), 0);
    scratch->interpValues.assign(poses.poseInterpolators.size(),
                                 {0.0, 0.0, 0.0});
    scratch->interpSolvers.assign(poses.poseInterpolators.size(),
                                  _RrRbfSolver());
    scratch->interpScratch.assign(poses.poseInterpolators.size(), {});
    for (size_t i = 0; i < poses.poseInterpolators.size(); ++i) {
        const RigExecWireRbf &wire =
            poses.poseInterpolators[i].solver;
        std::vector<RrVec3d> rbfPoses;
        for (const RigExecWireVec3d &p : wire.poses) {
            rbfPoses.push_back(RrVec3d(p[0], p[1], p[2]));
        }
        std::vector<RrVec3d> rbfTranslations;
        for (const RigExecWireVec3d &p : wire.translations) {
            rbfTranslations.push_back(RrVec3d(p[0], p[1], p[2]));
        }
        std::vector<_RrRbfPoseType> rbfPoseTypes;
        for (uint8_t type : wire.poseTypes) {
            rbfPoseTypes.push_back(_RrRbfPoseType(type));
        }
        scratch->interpSolvers[i] = _RrRbfSolver(
            rbfPoses, rbfTranslations, rbfPoseTypes,
            RrVec3d(wire.twistAxis[0], wire.twistAxis[1],
                    wire.twistAxis[2]),
            _RrRbfKernel(wire.kernel), wire.radius,
            wire.translationRadius,
            wire.normalize, wire.enableRotation,
            wire.enableTranslation);
        scratch->interpSolvers[i].SetSolvedTable(
            wire.radii, wire.translationRadii, wire.weights);
    }
    scratch->solvers.resize(poses.solvers.size());
    for (size_t s = 0; s < poses.solvers.size(); ++s) {
        const RigExecWireSolver &wire = poses.solvers[s];
        RrPoseSolverState &state = scratch->solvers[s];
        for (const auto &rest : wire.jointRests) {
            state.jointRests.push_back(_RrWireLandmarks(rest));
        }
        for (const auto &rest : wire.controlRests) {
            state.controlRests.push_back(_RrWireLandmarks(rest));
        }
        state.startRest = _RrWireLandmarks(wire.startRest);
        const size_t base = wire.start >= 0 ? 1 : 0;
        state.elements.resize(wire.controls.size() + base);
        for (size_t k = 0; k < wire.controls.size(); ++k) {
            const size_t e = k + base;
            if (k < state.controlRests.size()) {
                state.elements[e].restPoints = state.controlRests[k];
            } else {
                state.elements[e].restPoints = RrIdentityLandmarks();
            }
            state.elements[e].posePoints = RrIdentityLandmarks();
            state.elements[e].outRestPoints = RrIdentityLandmarks();
        }
        for (size_t k = 0; k < 3; ++k) {
            state.ikRests[k] = _RrWireLandmarks(wire.ikRests[k]);
        }
        state.ikParams.upperLength = wire.ikParams.upperLength;
        state.ikParams.lowerLength = wire.ikParams.lowerLength;
        state.ikParams.stretch = wire.ikParams.stretch;
        state.ikParams.softness = wire.ikParams.softness;
        state.ikParams.preferredBendRadians =
            wire.ikParams.preferredBendRadians;
        state.upperLengthBase = wire.upperLengthBase;
        state.lowerLengthBase = wire.lowerLengthBase;
        state.spaceRest = _RrWireLandmarks(wire.spaceRest);
        state.splineRest.cvs = _RrWireLandmarks(wire.splineRest.cvs);
        state.splineRest.rootControl =
            RrWireToFrame(wire.splineRest.rootControl);
        state.splineRest.midControl =
            RrWireToFrame(wire.splineRest.midControl);
        state.splineRest.endControl =
            RrWireToFrame(wire.splineRest.endControl);
        for (const RigExecWireFrame &joint : wire.splineRest.joints) {
            state.splineRest.joints.push_back(RrWireToFrame(joint));
        }
        state.splineRest.segmentLengths = wire.splineRest.segmentLengths;
        state.splineRest.restArcLength = wire.splineRest.restArcLength;
        state.splineRest.volumeWeights = wire.splineRest.volumeWeights;
        for (const auto &rest : wire.splineJointRests) {
            state.splineJointRests.push_back(_RrWireLandmarks(rest));
        }
        state.twistStartRest = _RrWireLandmarks(wire.twistStartRest);
        state.twistEndRest = _RrWireLandmarks(wire.twistEndRest);
        state.twistWeights = wire.twistWeights;
        for (const RigExecWireVec3f &p : wire.ribbonRestPoints) {
            state.ribbonRestPoints.push_back(
                RrVec3f(p[0], p[1], p[2]));
        }
    }
    scratch->recordAfter.assign(poses.commits.size(), 1);
    scratch->recordEveryTarget.assign(poses.commits.size(), 1);
    scratch->constraintWeights.assign(poses.constraints.size(), 0.0f);
    scratch->constraintHaveWeight.assign(poses.constraints.size(), 0);
    scratch->weightScratch.assign(poses.constraints.size(), {});
    scratch->weightError.assign(poses.constraints.size(), {});
    scratch->deltaValues.assign(
        program->geometry->deltaBasePaths.size(), identity);
    scratch->deltaPresent.assign(
        program->geometry->deltaBasePaths.size(), 0);
    // Per-commit IK chains, sized from the wire commit the way Build
    // sizes them: the IK write-back and the atomic check walk these.
    RrStore &store = program->store;
    for (size_t c = 0; c < poses.commits.size() &&
         c < store.commits.size(); ++c) {
        const size_t targets =
            poses.commits[c].targetReads.size();
        store.commits[c].ikChain.assign(targets, RrPointFrame());
        store.commits[c].ikPrepared.assign(targets, RrPointFrame());
        store.commits[c].ikRest.assign(targets, RrPointFrame());
        store.commits[c].ikSolved.clear();
    }
    program->pose = std::shared_ptr<void>(scratch);
    return true;
}

bool
RrProloguePose(RrProgram *program,
               const RigExecWireFrameInputs &record,
               std::vector<std::string> *poseDiagnostics,
               std::string *error)
{
    (void)poseDiagnostics;
    RrStore &store = program->store;
    RrPoseScratch *scratch = _RrScratch(program);
    const RigExecWireSlotMeta &meta = *program->slotMeta;
    const RigExecWireDomainPose &poses = *program->poses;
    const RigExecWireDomainGeometry &geometry = *program->geometry;
    // Every record table is validated before anything is mutated.
    if (record.solverRibbonPoints.size() != poses.solvers.size()) {
        if (error) {
            *error = "frame record ribbon tables do not match the solvers";
        }
        return false;
    }
    if (record.xformBase.size() != meta.xformSlots.size()) {
        if (error) {
            *error = "frame record xform tables do not match the slots";
        }
        return false;
    }
    if (record.nativeFrames.size() != poses.nativeSources.size()) {
        if (error) {
            *error =
                "frame record native frames do not match the sources";
        }
        return false;
    }
    if (record.deltaBaseMatrix.size() != geometry.deltaBasePaths.size() ||
        record.deltaBaseOk.size() != geometry.deltaBasePaths.size()) {
        if (error) {
            *error = "frame record delta tables do not match the paths";
        }
        return false;
    }
    if (record.arrayWeights.size() != poses.constraintArrays.size() ||
        record.arrayTranslationOffsets.size() !=
            poses.constraintArrays.size() ||
        record.arrayRotationOffsets.size() !=
            poses.constraintArrays.size() ||
        record.arrayOk.size() != poses.constraintArrays.size() ||
        record.arrayPoleWeights.size() !=
            poses.constraintArrays.size() ||
        record.arrayPoleOk.size() != poses.constraintArrays.size()) {
        if (error) {
            *error =
                "frame record array tables do not match the constraints";
        }
        return false;
    }
    if (record.constraintWeights.size() != poses.constraints.size() ||
        record.constraintHaveWeight.size() != poses.constraints.size()) {
        if (error) {
            *error =
                "frame record envelope tables do not match the constraints";
        }
        return false;
    }
    // RigExecBakedRunInputs. The provider ladder, before the avars that
    // compose against it: it recomposes when a channel varies, while a drag
    // stands on one of its channels, and once more after that drag is
    // released, which writes the authored values back over the dragged.
    const RrInputState &inputs = program->inputState;
    bool ladderDragged = false;
    if (store.anyOverridden) {
        for (const int32_t index : inputs.ladderOverrides) {
            if (size_t(index) < store.overridden.size() &&
                store.overridden[size_t(index)]) {
                ladderDragged = true;
                break;
            }
        }
    }
    scratch->ladderRecomputed =
        poses.ladderVarying || ladderDragged || scratch->ladderDisturbed;
    if (scratch->ladderRecomputed) {
        _RrComposeLadder(program, /* trackMoves = */ true);
        scratch->ladderDisturbed = ladderDragged;
    } else if (!store.ladderMovedSlots.empty()) {
        store.ladderMovedSlots.clear();
    }
    // The avar table: each binding read per run. A drag lands on avars
    // the bake captured as constants, so while one stands, and once more
    // after it is released, every constant binding is walked too: the
    // released avar holds the dragged value until its constant is written
    // back over it.
    const RigExecWireComputed &computed = *inputs.computed;
    const auto avarOf = [&](uint32_t read) {
        return size_t(computed.registeredReads[read].avar);
    };
    for (const uint32_t read : inputs.avarBindingReads) {
        store.avars[avarOf(read)] =
            program->ReadRegistered(int32_t(read)).f64;
    }
    for (const uint32_t read : inputs.avarConstantReads) {
        // A constant binding an edit animated reads per run as well.
        if ((program->RegisteredInput(int32_t(read)).flags &
             uint8_t(v4::InputReadFlags::Varying)) != 0) {
            store.avars[avarOf(read)] =
                program->ReadRegistered(int32_t(read)).f64;
        }
    }
    if (store.anyOverridden || scratch->avarsDisturbed) {
        for (const uint32_t read : inputs.avarConstantReads) {
            store.avars[avarOf(read)] =
                _RrLive(program, program->RegisteredInput(int32_t(read)))
                    ? program->ReadRegistered(int32_t(read)).f64
                    : program->RegisteredConstant(int32_t(read)).f64;
        }
        scratch->avarsDisturbed = store.anyOverridden;
    }
    // The pose interpolators' enables, read here so their step reads
    // no input table.
    for (size_t i = 0; i < poses.poseInterpolators.size() &&
         i < scratch->interpEnabled.size(); ++i) {
        scratch->interpEnabled[i] =
            program->ReadInterp(i).boolean ? 1 : 0;
        // A numeric driver's dials, read here for the same reason.
        const size_t dials =
            poses.poseInterpolators[i].valueInputs.size();
        for (size_t v = 0; v < dials && v < 3; ++v) {
            scratch->interpValues[i][v] =
                program->ReadInterpValue(i, v).f64;
        }
    }
    // Live ribbon driver points, swapped against last run's by value.
    for (size_t s = 0; s < poses.solvers.size(); ++s) {
        if (!store.ribbonVarying[s]) {
            continue;
        }
        store.ribbonLast[s].swap(store.ribbonPoints[s]);
        std::vector<RrVec3f> &points = store.ribbonPoints[s];
        points.clear();
        points.reserve(record.solverRibbonPoints[s].size());
        for (const RigExecWireVec3f &p : record.solverRibbonPoints[s]) {
            points.push_back(RrVec3f(p[0], p[1], p[2]));
        }
        store.ribbonDirty[s] = points != store.ribbonLast[s] ? 1 : 0;
    }
    // Xform-derived slots, seeded from the stage through the record.
    for (size_t k = 0; k < meta.xformSlots.size(); ++k) {
        const int slot = meta.xformSlots[k];
        if (slot < 0 || size_t(slot) >= store.fin.size() ||
            size_t(slot) >= store.base.size() ||
            k >= store.xformBase.size()) {
            if (error) {
                *error = "frame record xform tables do not match the slots";
            }
            return false;
        }
        const RrMat4d matrix = _RrWireMatrix(record.xformBase[k]);
        store.xformBase[k] = matrix;
        const RrPointFrame frame = RrFrameFromMatrix(matrix);
        store.base[size_t(slot)] = frame;
        store.fin[size_t(slot)] = frame;
    }
    // The target transform each geometry-domain constraint measures
    // its delta against.
    for (size_t k = 0; k < geometry.deltaBasePaths.size(); ++k) {
        store.deltaBaseOk[k] = record.deltaBaseOk[k] ? 1 : 0;
        store.deltaBaseMatrix[k] = _RrWireMatrix(record.deltaBaseMatrix[k]);
    }
    // The plain Xformables a constraint reads as a source. The record
    // carries frames only, no ok array, so ok is the frame's own
    // usability: a store read that failed has no frame to seed.
    for (size_t k = 0; k < poses.nativeSources.size(); ++k) {
        const RrPointFrame frame = RrWireToFrame(record.nativeFrames[k]);
        store.nativeFrames[k] = frame;
        store.nativeFrameOk[k] = RrFrameUsable(frame) ? 1 : 0;
    }
    // The recorded envelopes, which only the cross-check reads: the
    // Constraint step computes its own. Sizes were validated against the
    // constraints at the head.
    for (size_t k = 0;
         program->CrossCheckThisRun() && k < poses.constraints.size();
         ++k) {
        scratch->constraintHaveWeight[k] =
            k < record.constraintHaveWeight.size() &&
                    record.constraintHaveWeight[k]
                ? 1
                : 0;
        scratch->constraintWeights[k] =
            k < record.constraintWeights.size()
                ? record.constraintWeights[k]
                : 0.0f;
    }
    // A constraint's own authored tables, as the prologue read them at
    // this frame's time. Capture folded every read semantic into the
    // record; the cardinality lines it cannot carry stay empty.
    for (size_t k = 0; k < poses.constraintArrays.size(); ++k) {
        RrConstraintArraysLive &live = store.arrays[k];
        live.weights = record.arrayWeights[k];
        live.translationOffsets.clear();
        live.translationOffsets.reserve(
            record.arrayTranslationOffsets[k].size());
        for (const RigExecWireVec3d &v :
             record.arrayTranslationOffsets[k]) {
            live.translationOffsets.push_back(
                RrVec3d(v[0], v[1], v[2]));
        }
        live.rotationOffsets.clear();
        live.rotationOffsets.reserve(
            record.arrayRotationOffsets[k].size());
        for (const RigExecWireVec3d &v : record.arrayRotationOffsets[k]) {
            live.rotationOffsets.push_back(
                RrVec3d(v[0], v[1], v[2]));
        }
        live.ok = record.arrayOk[k] != 0;
        live.poleWeights = record.arrayPoleWeights[k];
        live.poleOk = record.arrayPoleOk[k] != 0;
    }
    return true;
}

} // namespace rigExec
