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
_RrComposeLadder(RrProgram *program, bool trackMoves,
                 size_t begin, size_t end, bool restOnly, bool ladderOnly)
{
    RrStore &store = program->store;
    RrPoseScratch *scratch = _RrScratch(program);
    const RigExecWireSlotMeta &meta = *program->slotMeta;
    const size_t slots = meta.paths.size();
    const RrMat4d identity = _RrIdentity();
    if (trackMoves) {
        store.ladderMovedSlots.clear();
    }
    for (size_t i = begin; i < std::min(end, slots); ++i) {
        if (i >= meta.slotKind.size() ||
            meta.slotKind[i] != RigExecWireSlotKind::FirstFramePose) {
            continue;
        }
        // Immutable provider availability is distinct from per-attribute presence.
        if (!meta.providerActive.empty() && !meta.providerActive[i]) {
            RrMat4d unavailable(1);
            unavailable[3][0] = std::numeric_limits<double>::quiet_NaN();
            if (!ladderOnly) {
                scratch->restM[i] = unavailable;
                scratch->restFrames[i] = RrFrameFromMatrix(RrMat4d(1));
                scratch->restFrames[i].flags = 0;
                scratch->restPts[i] = scratch->restFrames[i].points;
                scratch->restRoundTrip[i] = unavailable;
            }
            if (!restOnly) {
                scratch->restRoundTrip[i] = unavailable;
                scratch->defaultRoundTrip[i] = unavailable;
                scratch->selfD[i] = unavailable;
                scratch->parentDinv[i] = unavailable;
                scratch->posedD[i] = unavailable;
                scratch->posedAuthoredM[i] = unavailable;
                scratch->parentSpaceM[i] = unavailable;
                scratch->posedAuthored[i] = 0;
                scratch->parentSpaceAuthored[i] = 0;
                scratch->rotationSign[i] = 0;
                scratch->rotOrder[i] = 0; // Empty token selects the existing XYZ fallback.
            }
            continue;
        }
        const auto &ladder = program->poses->ladders[i];
        const auto authoritative = [&](const auto &input, bool connected) {
            const auto ready=[&](const auto &field,int channel) {
                if(input.get()!=field.get() || size_t(channel)>=ladder.spaceValues.size()) return false;
                const int id=ladder.spaceValues[size_t(channel)];
                if(id<0 || size_t(id)>=store.providerValues.size()) return false;
                const auto &value=store.providerValues[size_t(id)];
                return value.initialized && !value.blocked &&
                    std::holds_alternative<std::array<double,16>>(value.value);
            };
            // Ready default expressions supply defaults; readiness alone does not
            // make identity posed or parent space an authored replacement.
            if(ready(ladder.defaultSpace,1) || ready(ladder.parentDefaultSpace,4) ||
               ready(ladder.avarDefaultSpace,5) || ready(ladder.posedDefaultSpace,6)) return true;
            const int index = input->overrideIndex;
            return connected || (index >= 0 &&
                size_t(index) < program->inputState.valueOverridden.size() &&
                program->inputState.valueOverridden[size_t(index)]);
        };
        const RrMat4d posed =
            program->ReadLadder(i, RrLadderPosedSpace).matrix;
        if (!restOnly) {
            scratch->posedAuthored[i] =
                (authoritative(ladder.posedSpace, ladder.posedSpaceConnected) || posed != identity) ? 1 : 0;
            scratch->posedAuthoredM[i] = posed;
        }

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
        if (!ladderOnly) {
            const RrMat4d intervening =
                program->ReadLadder(i, RrLadderInterveningSpace).matrix;
            scratch->restM[i] = (intervening == identity ? rest : rest * intervening) * parentRest;
            scratch->restFrames[i] = RrFrameFromMatrix(scratch->restM[i]);
            scratch->restPts[i] = scratch->restFrames[i].points;
            scratch->restRoundTrip[i] = RrRoundTrip(scratch->restM[i]);
        }
        if (!restOnly) {

        const RrMat4d authoredDefault =
            program->ReadLadder(i, RrLadderDefaultSpace).matrix;
        const RrMat4d authoredParentDefault =
            program->ReadLadder(i, RrLadderParentDefaultSpace).matrix;
        const RrMat4d parentDefault =
            authoritative(ladder.parentDefaultSpace, ladder.parentDefaultSpaceConnected) ||
            authoredParentDefault != identity ? authoredParentDefault :
            (parent >= 0 ? scratch->defaultRoundTrip[size_t(parent)] : identity);
        if (authoritative(ladder.defaultSpace, ladder.defaultSpaceConnected) ||
            authoredDefault != identity) {
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
        const RrMat4d authoredAvarDefault =
            program->ReadLadder(i, RrLadderAvarDefaultSpace).matrix;
        const RrMat4d avarDefault =
            authoritative(ladder.avarDefaultSpace, ladder.avarDefaultSpaceConnected) ||
            authoredAvarDefault != identity ? authoredAvarDefault : scratch->selfD[i];
        const RrMat4d authoredPosedDefault =
            program->ReadLadder(i, RrLadderPosedDefaultSpace).matrix;
        scratch->posedD[i] =
            authoritative(ladder.posedDefaultSpace, ladder.posedDefaultSpaceConnected) ||
            authoredPosedDefault != identity ? authoredPosedDefault : avarDefault;
        scratch->parentSpaceM[i] = program->ReadLadder(i, RrLadderParentSpace).matrix;
        scratch->parentSpaceAuthored[i] =
            authoritative(ladder.parentSpace, ladder.parentSpaceConnected) ||
            scratch->parentSpaceM[i] != identity;
        const auto sign = program->ReadLadder(i, RrLadderRotationSign).vec;
        scratch->rotationSign[i] = (sign[0] < 0 ? 1u : 0u) |
                                   (sign[1] < 0 ? 2u : 0u) |
                                   (sign[2] < 0 ? 4u : 0u);

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

        }
        if (!trackMoves) {
            continue;
        }
        if (scratch->restM[i] != scratch->lastRestM[i] ||
            scratch->selfD[i] != scratch->lastSelfD[i] ||
            scratch->parentDinv[i] != scratch->lastParentDinv[i] ||
            scratch->posedAuthored[i] != scratch->lastPosedAuthored[i] ||
            scratch->posedAuthoredM[i] != scratch->lastPosedAuthoredM[i] ||
            scratch->rotOrder[i] != scratch->lastRotOrder[i] ||
            scratch->posedD[i] != scratch->lastPosedD[i] ||
            scratch->parentSpaceM[i] != scratch->lastParentSpaceM[i] ||
            scratch->parentSpaceAuthored[i] != scratch->lastParentSpaceAuthored[i] ||
            scratch->rotationSign[i] != scratch->lastRotationSign[i]) {
            store.ladderMovedSlots.push_back(int(i));
            scratch->lastRestM[i] = scratch->restM[i];
            scratch->lastSelfD[i] = scratch->selfD[i];
            scratch->lastParentDinv[i] = scratch->parentDinv[i];
            scratch->lastPosedAuthored[i] = scratch->posedAuthored[i];
            scratch->lastPosedAuthoredM[i] = scratch->posedAuthoredM[i];
            scratch->lastRotOrder[i] = scratch->rotOrder[i];
            scratch->lastPosedD[i] = scratch->posedD[i];
            scratch->lastParentSpaceM[i] = scratch->parentSpaceM[i];
            scratch->lastParentSpaceAuthored[i] = scratch->parentSpaceAuthored[i];
            scratch->lastRotationSign[i] = scratch->rotationSign[i];
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
    const RrVec3d axes[3] = {
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
_RrLive(const RrProgram *program, const RigExecWireInput &read)
{
    if ((read.flags & (uint8_t(RigExecWireInputReadFlags::Varying) |
                       uint8_t(RigExecWireInputReadFlags::SourceBacked))) != 0) {
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

const std::vector<RrPointFrame> &
RrPoseRestFrames(const RrProgram *program)
{
    return static_cast<const RrPoseScratch *>(program->pose.get())
        ->restFrames;
}

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
        constants.posedD.size() != slots ||
        constants.parentSpaceM.size() != slots ||
        constants.parentSpaceAuthored.size() != slots ||
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
    scratch->checkpointInputs.resize(poses.spaceCheckpoints.size());
    for(size_t i=0;i<poses.spaceCheckpoints.size();++i)
        scratch->checkpointInputs[i].resize(poses.spaceCheckpoints[i].recompose.size());
    scratch->restM.assign(slots, identity);
    scratch->selfD.assign(slots, identity);
    scratch->posedD.assign(slots, identity);
    scratch->parentSpaceM.assign(slots, identity);
    scratch->parentSpaceAuthored.assign(slots, 0);
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
        scratch->posedD[i] = _RrWireMatrix(constants.posedD[i]);
        scratch->parentSpaceM[i] = _RrWireMatrix(constants.parentSpaceM[i]);
        scratch->parentSpaceAuthored[i] = constants.parentSpaceAuthored[i] ? 1 : 0;
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
    scratch->lastPosedD = scratch->posedD;
    scratch->lastParentSpaceM = scratch->parentSpaceM;
    scratch->lastParentSpaceAuthored = scratch->parentSpaceAuthored;
    scratch->lastRotationSign = scratch->rotationSign;
    scratch->interpEnabled.assign(poses.poseInterpolators.size(), 0);
    scratch->interpValues.assign(poses.poseInterpolators.size(),
                                 {0.0, 0.0, 0.0});
    scratch->interpSolvers.assign(poses.poseInterpolators.size(),
                                  _RrRbfSolver());
    scratch->interpScratch.assign(poses.poseInterpolators.size(), {});
    for (size_t i = 0; i < poses.poseInterpolators.size(); ++i) {
        const RigExecWireRbf &wire =
            *poses.poseInterpolators[i].solver;
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
        std::vector<std::vector<double>> weights;
        weights.reserve(wire.weights.size());
        for (const fb::RigExecWireDoubleList &row : wire.weights) {
            weights.push_back(row.v);
        }
        scratch->interpSolvers[i].SetSolvedTable(
            wire.radii, wire.translationRadii, weights);
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
        state.splineRest.cvs = _RrWireLandmarks(wire.splineRest->cvs);
        state.splineRest.rootControl =
            RrWireToFrame(wire.splineRest->rootControl);
        state.splineRest.midControl =
            RrWireToFrame(wire.splineRest->midControl);
        state.splineRest.endControl =
            RrWireToFrame(wire.splineRest->endControl);
        for (const RigExecWireFrame &joint : wire.splineRest->joints) {
            state.splineRest.joints.push_back(RrWireToFrame(joint));
        }
        state.splineRest.segmentLengths = wire.splineRest->segmentLengths;
        state.splineRest.restArcLength = wire.splineRest->restArcLength;
        state.splineRest.volumeWeights = wire.splineRest->volumeWeights;
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
RrRunRestHead(RrProgram *program, size_t group, bool ladder)
{
    const auto &range = program->poses->composeGroups[group];
    _RrComposeLadder(program, true, size_t(range.begin), size_t(range.end),
                     !ladder, ladder);
    auto &changed = ladder ? program->store.ladderChanged : program->store.restChanged;
    for (const int slot : program->store.ladderMovedSlots) changed[size_t(slot)] = 1;
    return !program->store.ladderMovedSlots.empty();
}

bool
RrProloguePose(RrProgram *program,
               std::vector<std::string> *poseDiagnostics,
               std::string *error)
{
    (void)poseDiagnostics;
    (void)error;
    RrStore &store = program->store;
    RrPoseScratch *scratch = _RrScratch(program);
    const RigExecWireSlotMeta &meta = *program->slotMeta;
    const RigExecWireDomainPose &poses = *program->poses;
    const RigExecWireDomainGeometry &geometry = *program->geometry;
    // The validator checked the static tables' sizes against the program.
    const RrStatic &statics = program->statics;
    const RrInputState &inputs = program->inputState;
    // Xform-derived slots, seeded from the stage values the bake captured.
    const size_t captured=program->requiredStageFramesAdmission.admitted
        ?meta.xformSlots.size():size_t(program->requiredStageFramesAdmission.firstBadTarget);
    for (size_t k = 0; k < captured; ++k) {
        const int slot = meta.xformSlots[k];
        const RrMat4d matrix = _RrWireMatrix(statics.XformBase(k));
        store.xformBase[k] = matrix;
        const RrPointFrame frame = RrWireToFrame(poses.xformFrames[k]);
        store.base[size_t(slot)] = frame;
        store.fin[size_t(slot)] = frame;
    }
    if(!program->requiredStageFramesAdmission.admitted)return true;
    // The target transform each geometry-domain constraint measures
    // its delta against.
    for (size_t k = 0; k < geometry.deltaBasePaths.size(); ++k) {
        store.deltaBaseOk[k] = statics.DeltaBaseOk(k) ? 1 : 0;
        store.deltaBaseMatrix[k] = _RrWireMatrix(statics.DeltaBase(k));
    }
    // The plain Xformables a constraint reads as a source. ok is derived
    // from the frame's own usability and the file's NativeSource.ok is not
    // read: a store read that failed has no frame to seed.
    for (size_t k = 0; k < poses.nativeSources.size(); ++k) {
        const RrPointFrame frame = RrWireToFrame(statics.NativeFrame(k));
        store.nativeFrames[k] = frame;
        store.nativeFrameOk[k] = RrFrameUsable(frame) ? 1 : 0;
    }
    // Constraint array inputs are copied by the input sampler. The owning
    // Constraint operation validates cardinality and expands neutral defaults.
    return true;
}

} // namespace rigExec
