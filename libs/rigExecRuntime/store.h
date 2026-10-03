// rigExecRuntime program state and pipeline contracts (M2 framework).
// RrStore owns every framework-visible slot domain: the avar table, the
// SSA fin/base version pools, matrices, aggregates, commit scratch, the
// prologue's retained arrays and their lasts, snapshots, step outputs,
// and the input-value holders. Family .cpps own their private scratch
// (solver live rests, revision packets, weight oracles) behind the three
// extension points on RrProgram, and implement the pipeline functions
// declared here. The framework implements Open/closure/walk/publish.
#ifndef RIGEXEC_RUNTIME_STORE_H
#define RIGEXEC_RUNTIME_STORE_H

#include "rigExecBinary/container.h"
#include "rigExecBinary/geometry.h"
#include "rigExecBinary/inputTable.h"
#include "rigExecBinary/pose.h"
#include "rigExecBinary/program.h"
#include "rigExecRuntime/values.h"

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace rigExec {

// Capture-order field indices for the uid replay. Each list is in the
// order RigExecBakeCapture::RigExecBakeCapture walks it; a field whose
// wire input is not (varying && bound) takes no uid.
enum RrLadderField : int {
    RrLadderRestSpace = 0,
    RrLadderDefaultSpace = 1,
    RrLadderPosedSpace = 2,
    RrLadderRestAvar0 = 3,
    // RrLadderRestAvar0 + k, k in 0..5.
    RrLadderDefaultAvar0 = 9,
    // RrLadderDefaultAvar0 + k, k in 0..5.
    RrLadderRotationOrder = 15,
    RrLadderFieldCount = 16,
};

enum RrSolverField : int {
    RrSolverBend = 0,
    RrSolverUpperOffset = 1,
    RrSolverLowerOffset = 2,
    RrSolverStretch = 3,
    RrSolverSoftness = 4,
    RrSolverBlendWeight = 5,
    RrSolverPreserveVolume = 6,
    RrSolverMidFollowWeight = 7,
    RrSolverRoll = 8,
    RrSolverTwist = 9,
    RrSolverMinLengthRatio = 10,
    RrSolverTwistTurns = 11,
    RrSolverRibbonSampleCount = 12,
    RrSolverFieldCount = 13,
};

enum RrConstraintField : int {
    RrConstraintEnabled = 0,
    RrConstraintDefaultWeight = 1,
    RrConstraintOffset = 2,
    RrConstraintAffectX = 3,
    RrConstraintAffectY = 4,
    RrConstraintAffectZ = 5,
    RrConstraintTX = 6,
    RrConstraintTY = 7,
    RrConstraintTZ = 8,
    RrConstraintRX = 9,
    RrConstraintRY = 10,
    RrConstraintRZ = 11,
    RrConstraintSX = 12,
    RrConstraintSY = 13,
    RrConstraintSZ = 14,
    RrConstraintAimVector = 15,
    RrConstraintUpVector = 16,
    RrConstraintRotationOffset = 17,
    RrConstraintWorldUpVector = 18,
    RrConstraintPoleVector = 19,
    RrConstraintTwistDegrees = 20,
    RrConstraintFieldCount = 21,
};

enum RrWeightField : int {
    RrWeightDefaultWeight = 0,
    RrWeightDriver = 1,
    RrWeightScale = 2,
    RrWeightBias = 3,
    RrWeightStrength = 4,
    RrWeightInvert = 5,
    RrWeightFalloffMin = 6,
    RrWeightFalloffMax = 7,
    RrWeightScaleXPos = 8,
    RrWeightScaleYPos = 9,
    RrWeightScaleZPos = 10,
    RrWeightScaleXNeg = 11,
    RrWeightScaleYNeg = 12,
    RrWeightScaleZNeg = 13,
    RrWeightScaleX = 14,
    RrWeightScaleY = 15,
    RrWeightScaleZ = 16,
    RrWeightExtentU = 17,
    RrWeightExtentV = 18,
    RrWeightFieldCount = 19,
};

// One slot's live ladder values, recomputed by the pose prologue when
// the ladders vary and seeded from the constants otherwise.
struct RrLadderLive {
    RrMat4d restSpace;
    RrMat4d defaultSpace;
    RrMat4d posedSpace;
    double restAvars[6] = {0, 0, 0, 0, 0, 0};
    double defaultAvars[6] = {0, 0, 0, 0, 0, 0};
    uint32_t rotationOrder = 0;
};

// One revision's publish row, filled by the geometry steps for the
// epilogue's weight-field and moved-property publication.
struct RrRevisionPublish {
    bool weightFieldPublished = false;
    uint32_t weightFieldTarget = 0;
    std::vector<float> weightField;
    std::string resultStatus;
    uint32_t target = 0;
};

// One chain's publish row: the deformed points the ChainStatus step
// left, or nothing when the chain has no base this run.
struct RrChainPublish {
    bool haveBase = false;
    uint32_t target = 0;
    std::vector<RrVec3f> result;
};

// One resolved weight field publication.
struct RrWeightFieldPublish {
    uint32_t target = 0;
    std::vector<float> weights;
};

// One derived target's publish row.
struct RrDerivedPublish {
    bool haveBase = false;
    uint32_t target = 0;
    std::vector<RrVec3f> result;
};

// Family-private scratch, defined in the family's own .cpp.
struct RrPoseScratch;
struct RrGeometryScratch;
struct RrWeightScratch;

// The frame's working state. Sized once at Open from the decoded tables;
// a run mutates values, never sizes.
struct RrStore {
    std::vector<double> avars, lastAvars;
    std::vector<RrPointFrame> base, fin;
    std::vector<uint32_t> finLast, baseLast;
    std::vector<RrMat4d> posedM, finalMatrix, baseMatrix;
    std::vector<RrPointFrameArray> aggregates;
    std::vector<RrLadderLive> ladders;
    std::vector<int> ladderMovedSlots;
    std::vector<std::vector<RrPointFrame>> solverOutFrames;
    std::vector<std::vector<char>> solverOutPresent;
    std::vector<std::vector<RrVec3f>> ribbonPoints, ribbonLast;
    std::vector<std::vector<RrVec3f>> ribbonConstant;
    std::vector<char> ribbonVarying, ribbonDirty;
    std::vector<RrCommitScratch> commits;
    std::vector<RrConstraintArraysLive> arrays;
    std::map<uint32_t, RrPropertyValue> propertyResults, lastPropertyResults;
    std::vector<char> chainHaveBase, chainBaseDirty, lastHaveBase;
    std::vector<std::vector<RrVec3f>> chainBases;
    std::vector<char> derivedHaveBase;
    std::vector<std::vector<RrVec3f>> derivedBases;
    std::vector<RrPointFrame> nativeFrames, lastNativeFrames;
    std::vector<char> nativeFrameOk, lastNativeFrameOk;
    std::vector<RrMat4d> xformBase, lastXformBase;
    std::vector<RrMat4d> deltaBaseMatrix, lastDeltaBaseMatrix;
    std::vector<char> deltaBaseOk, lastDeltaBaseOk;
    // Per-frame solved constraint deltas, published by the pose family for
    // the geometry fold (the baked B.deltaValues/deltaPresent). Indexed by
    // constraint deltaBase, like deltaBaseMatrix; the pose step clears each
    // entry it owns before solving, exactly as the baked step does, so no
    // last-frame copy is retained.
    std::vector<RrMat4d> deltaValues;
    std::vector<char> deltaPresent;
    std::vector<char> revisionRan, revisionStaticDirty;
    std::vector<RrRevisionPublish> revisionPublish;
    std::vector<RrChainPublish> chainPublish;
    std::vector<RrDerivedPublish> derivedPublish;
    std::vector<RrWeightPacket> weightPackets;
    std::map<uint32_t, RrMat4d> weightFrames;
    std::vector<float> poseWeights;
    std::vector<char> overridden, lastOverridden;
    std::map<uint32_t, RrMat4d> providerXforms, providerBaseXforms;
    std::map<uint32_t, RrMat4d> jointMatricesFinal;
    std::map<uint32_t, RrPointFrame> jointFramesBase, jointFramesFinal;
    std::map<uint32_t, RrPointFrame> controlFrames;
    std::map<uint32_t, std::vector<RrVec3f>> movedProperties;
    std::map<uint32_t, RrWeightFieldPublish> weightFields;
    std::vector<char> jointMatrixPublished;
    std::vector<RrInputValue> inputHolders;
    RrSnapshots runSnapshots;
    std::vector<RrStepOutput> stepOutputs;
    std::vector<uint64_t> closedWords;
    bool everRan = false;
    double lastTime = 0;
    size_t lastClosedClusters = 0;
};

// The decoded program plus its working state. The wire tables borrow
// from the reader; the store is sized at Open.
struct RrProgram {
    const std::vector<RigExecWireStep> *steps = nullptr;
    const RigExecWireClustering *clustering = nullptr;
    const RigExecWireCones *cones = nullptr;
    const RigExecWireSlotMeta *slotMeta = nullptr;
    const RigExecWireConstants *constants = nullptr;
    const RigExecWireDomainPose *poses = nullptr;
    const RigExecWireDomainGeometry *geometry = nullptr;
    const RigExecWireInputTable *inputs = nullptr;
    const RigExecBinaryReader *strings = nullptr;

    RrStore store;

    // Slot path -> slot index, for avar-head routing at Open.
    std::unordered_map<std::string, int> pathIndex;

    // Uid routing, replayed at Open in capture order; -1 takes no uid.
    std::vector<std::array<int32_t, RrLadderFieldCount>> ladderUid;
    std::vector<int32_t> spaceSwitchUid;
    std::vector<int32_t> interpUid;
    // Per interpolator, one uid per numeric dial. Three because the
    // compile reads at most three, one per axis, and refuses a fourth.
    std::vector<std::array<int32_t, 3>> interpValueUid;
    // Per provider slot: its space switch's index, or -1. Built once at
    // Open so the compose pays one array lookup per slot and a rig with
    // no switch pays nothing at all. Empty when the binary carries none.
    std::vector<int32_t> spaceSwitchBySlot;
    std::vector<std::array<int32_t, RrSolverFieldCount>> solverUid;
    std::vector<std::array<int32_t, RrConstraintFieldCount>> constraintUid;
    std::vector<std::array<int32_t, RrWeightFieldCount>> weightUid;
    // Per uid: the avar flat index, or -1 when the uid is not an avar.
    std::vector<int32_t> avarUidTarget;

    // Joint path id -> row in the jointBinding* tables.
    std::map<uint32_t, size_t> jointBindingIndex;

    // Type-erased: each family defines its struct in its own .cpp
    // and casts. shared_ptr's deleter is captured where the type is
    // complete, so no TU needs all three definitions.
    std::shared_ptr<void> pose;
    std::shared_ptr<void> geo;
    std::shared_ptr<void> weights;

    // Test-only family mask: bit 0 pose, 1 weights, 2 geometry. All set
    // outside tests; a masked family's steps are skipped, which is how
    // one family's outputs are compared while another is still landing.
    unsigned runMask = 0x7u;

    // Compile notices from the manifest ("compileDiagnostics"), replayed
    // ahead of the program lines on the first Execute, then drained.
    // Empty for binaries baked before the key existed.
    std::vector<std::string> compileDiagnostics;

    bool GetText(uint32_t id, std::string *out) const
    {
        return strings && strings->GetString(id, out);
    }

    std::string TextOrEmpty(uint32_t id) const
    {
        std::string out;
        if (strings) {
            strings->GetString(id, &out);
        }
        return out;
    }

    bool TokenEquals(uint32_t id, const char *literal) const
    {
        return TextOrEmpty(id) == literal;
    }

    // The holder when the input takes a uid, else its constant: the
    // runtime form of RigExecBakedRead.
    RrInputValue ReadUid(const RigExecWireInput &input, int32_t uid) const
    {
        if (uid >= 0 && size_t(uid) < store.inputHolders.size()) {
            return store.inputHolders[size_t(uid)];
        }
        return RrWireInputConstant(input);
    }

    const RigExecWireInput &LadderInput(size_t slot, int field) const;
    const RigExecWireInput &SolverInput(size_t solver, int field) const;
    const RigExecWireInput &ConstraintInput(size_t constraint,
                                            int field) const;
    const RigExecWireInput &WeightInput(size_t object, int field) const;

    RrInputValue ReadLadder(size_t slot, int field) const
    {
        return ReadUid(LadderInput(slot, field),
                       ladderUid[slot][size_t(field)]);
    }
    RrInputValue ReadSolver(size_t solver, int field) const
    {
        return ReadUid(SolverInput(solver, field),
                       solverUid[solver][size_t(field)]);
    }
    RrInputValue ReadConstraint(size_t constraint, int field) const
    {
        return ReadUid(ConstraintInput(constraint, field),
                       constraintUid[constraint][size_t(field)]);
    }
    RrInputValue ReadWeight(size_t object, int field) const
    {
        return ReadUid(WeightInput(object, field),
                       weightUid[object][size_t(field)]);
    }
    RrInputValue ReadInterp(size_t interp) const
    {
        const RigExecWirePoseInterpolator &in =
            poses->poseInterpolators[interp];
        return ReadUid(in.enabled, interpUid[interp]);
    }
    RrInputValue ReadInterpValue(size_t interp, size_t axis) const
    {
        const RigExecWirePoseInterpolator &in =
            poses->poseInterpolators[interp];
        return ReadUid(in.valueInputs[axis],
                       interpValueUid[interp][axis]);
    }
    RrInputValue ReadSpaceSwitch(size_t index) const
    {
        return ReadUid(poses->spaceSwitches[index].active,
                       spaceSwitchUid[index]);
    }
};

// The framework (closure.cpp, publish.cpp, runtime.cpp) implements the
// executor and the epilogue; each family .cpp implements its own
// prologue, steps and scratch sizing. A step body reports a bail
// through its RrStepOutput; false with `error` is a fatal that names
// the step.

// Family scratch sizing at Open.
bool RrPoseSizeScratch(RrProgram *program, std::string *error);
bool RrGeometrySizeScratch(RrProgram *program, std::string *error);
bool RrWeightSizeScratch(RrProgram *program, std::string *error);

// Prologues. `poseDiagnostics` carries lines published straight into
// the generation (property chains); false names the failure.
bool RrProloguePose(RrProgram *program,
                    const RigExecWireFrameInputs &record,
                    std::vector<std::string> *poseDiagnostics,
                    std::string *error);
bool RrPrologueGeometry(RrProgram *program, double time,
                        const RigExecWireFrameInputs &record,
                        std::vector<std::string> *poseDiagnostics,
                        std::string *error);

// Step bodies by family.
bool RrRunPoseStep(RrProgram *program, size_t step, double time,
                   std::string *error);
bool RrRunGeometryStep(RrProgram *program, size_t step, double time,
                       std::string *error);
bool RrRunWeightStep(RrProgram *program, size_t step, double time,
                     std::string *error);
void RrSkipGeometryStep(RrProgram *program, size_t step);

// Executor and epilogue (framework).
void RrComputeClosure(RrProgram *program, double time, bool force);
bool RrRunSteps(RrProgram *program, double time, bool force,
                std::string *error);
bool RrPublishPose(RrProgram *program,
                   std::vector<std::string> *poseDiagnostics,
                   std::string *error);
void RrPublishGeometry(RrProgram *program,
                       const RigExecWireFrameInputs &record,
                       std::vector<std::string> *poseDiagnostics);

// Bit-identical ports of the point-frame kernels every family reads:
// FrameFromMatrix, PointsToMatrix, MatrixToPoints, FrameRotation,
// ElementOutSpace, ExtractElementFrame, RoundTrip.

RrPointFrame RrFrameFromMatrix(const RrMat4d &m);
bool RrPointsToMatrix(const std::array<RrVec3d, 4> &restPoints,
                      const std::array<RrVec3d, 4> &posePoints,
                      RrMat4d *matrix);
bool RrPointsToMatrix(const std::array<RrVec3d, 4> &restPoints,
                      const RrPointFrame &frame, RrMat4d *matrix);
RrPointFrame RrMatrixToPoints(const std::array<RrVec3d, 4> &restPoints,
                              const RrMat4d &matrix);
double RrFrameEpsilon(const std::array<RrVec3d, 4> &restPoints);
bool RrFrameRotation(const RrPointFrame &frame, RrQuatd *out);
RrMat4d RrElementOutSpace(const RrPointFrameArray *source, size_t index);
RrPointFrame RrExtractElementFrame(const RrPointFrameArray *source,
                                   size_t index);
RrMat4d RrRoundTrip(const RrMat4d &m);

/// RigExecFrameTranslation: the driver's local translation measured against
/// its own rest, which is what a pose interpolator's translation channel
/// solves on. Null parents mean the driver's local frame is its world one.
bool RrFrameTranslation(const RrPointFrame &driverFinal,
                        const RrPointFrame &driverRest,
                        const RrPointFrame *parentFinal,
                        const RrPointFrame *parentRest, RrVec3d *out);

/// RigExecBlendTransforms: decomposed lerp, with both endpoints returning
/// their operand untouched so a space switch on a whole number is
/// bit-identical to selecting that space.
RrMat4d RrBlendTransforms(const RrMat4d &a, const RrMat4d &b, double weight);

/// RigExecMaskTransform: zero the masked channels per axis, in the
/// transform's own decomposition. An all-true mask returns \p m untouched.
RrMat4d RrMaskTransform(const RrMat4d &m, const bool translation[3],
                        const bool rotation[3], const bool scale[3]);

/// Mirrors RigExecRotationFilter bit for bit.
enum class RrRotationFilter : uint8_t { All = 0, Twist = 1, Swing = 2 };

/// RigExecFilterSpaceRotation: keep only the twist of \p m's rotation about
/// \p axis, or only the swing. `All` returns \p m untouched.
RrMat4d RrFilterSpaceRotation(const RrMat4d &m, const RrVec3d &axis,
                              RrRotationFilter filter);

}  // namespace rigExec

#endif  // RIGEXEC_RUNTIME_STORE_H
