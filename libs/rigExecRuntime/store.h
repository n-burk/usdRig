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
#include "rigExecBinary/external.h"
#include "rigExecBinary/geometry.h"
#include "rigExecBinary/inputTable.h"
#include "rigExecBinary/pose.h"
#include "rigExecBinary/program.h"
#include "rigExecRuntime/inputs.h"
#include "rigExecRuntime/values.h"
#include "rigExecMath/autoClavicleKernel.h"

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
    RrSolverIkSpace = 13,
    RrSolverFieldCount = 14,
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
    /// A surface projector target publishes a matrix primvar instead,
    /// when this run measured one.
    bool matrixTarget = false;
    bool haveMatrix = false;
    RrMat4d matrix;
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
    // The property chains' results this run (properties.cpp), keyed by the
    // attribute path id they are published at: chain targets and phased
    // consumers. Every read of such an attribute sees them.
    std::map<uint32_t, RrPropertyValue> propertyResults;
    // The same results by publish entry (one per attribute a chain or a
    // phased consumer publishes at), with whether this run published it;
    // an unpublished entry holds a zero value. The closure compares them
    // with the last run's.
    std::vector<RrPropertyValue> propertyValues, lastPropertyValues;
    std::vector<char> propertyPublished, lastPropertyPublished;
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
    /// Matrix primvars a surface projector published, by property.
    std::map<uint32_t, RrMat4d> movedMatrices;
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
    /// The ExternalMovers section, or null when the file has no plugin
    /// mover.
    const RigExecWireExternalMovers *external = nullptr;
    /// The InputTable frame Execute is running, which also selects each
    /// plugin mover's frame bytes.
    size_t frameIndex = 0;

    /// One plugin revision's playback state, in ExternalMovers order. No
    /// prepared state means no kernel here: the revision passes through.
    struct ExternalRevision {
        std::string type;
        RigExecExternalKernel kernel;
        std::shared_ptr<const void> state;
    };
    std::vector<ExternalRevision> externals;
    /// (chain, revision) -> index into `externals`.
    std::map<std::pair<uint32_t, uint32_t>, size_t> externalIndex;

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
    // Per auto clavicle: the IK blend and amount uids, its constants, and
    // per provider slot its index (empty when the binary carries none).
    std::vector<std::array<int32_t, 2>> autoClavicleUid;
    std::vector<RigExecAutoClavicleConstants> autoClavicleConstants;
    std::vector<int32_t> autoClavicleBySlot;
    // Per limb record: the pin, upper/lower scale, soft distance and twist
    // uids, and per solver its record's index (empty when the binary
    // carries no LimbSolvers section).
    std::vector<std::array<int32_t, 5>> limbUid;
    std::vector<int32_t> limbBySolver;
    // Per auto clavicle: the limb record of the two-bone IK whose end and
    // pole are its IK target and pole, or -1.
    std::vector<int32_t> autoClavicleLimb;
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
    std::shared_ptr<void> properties;

    // Test-only family mask: bit 0 pose, 1 weights, 2 geometry. All set
    // outside tests; a masked family's steps are skipped, which is how
    // one family's outputs are compared while another is still landing.
    unsigned runMask = 0x7u;

    // Compile notices from the manifest ("compileDiagnostics"), replayed
    // ahead of the program lines on the first Execute, then drained.
    // Empty for binaries baked before the key existed.
    std::vector<std::string> compileDiagnostics;

    // The input slots and their values this run (inputs.h).
    RrInputState inputState;

    // Test-only: a step that computes what the frame record also carries
    // (constraint envelopes, current-phase weight packets, the reads a
    // revision's assembly consumes) compares the two bit for bit, fails
    // naming the field and index on a mismatch, and counts each comparison
    // in its RrStepOutput::crossChecked. Execute compares the property
    // chains' results and the registered and mover-scalar reads the same
    // way.
    bool crossCheck = false;

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
    RrInputValue ReadAutoClavicle(size_t index, size_t which) const
    {
        const RigExecWireAutoClavicle &ac = poses->autoClavicles[index];
        return ReadUid(which == 0 ? ac.ikBlend : ac.amount,
                       autoClavicleUid[index][which]);
    }
    RrInputValue ReadLimb(size_t index, size_t which) const
    {
        const RigExecWireLimbSolver &l = poses->limbSolvers[index];
        const RigExecWireInput *inputs[5] = {&l.pin, &l.upperScale,
                                             &l.lowerScale, &l.softDistance,
                                             &l.twist};
        return ReadUid(*inputs[which], limbUid[index][which]);
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
/// Classifies the Computed section's property chains and sizes the
/// publish entries; checks that each chain read's uid routes to an input
/// that reads the long way. Refuses a rig with property chains and no
/// section. Runs after the uid routing and the holders are set up.
bool RrPropertySizeScratch(RrProgram *program, std::string *error);

/// The property chains (RigExecRigEvaluator::_EvaluatePropertyChains,
/// rigEvaluatorProperties.cpp) over this run's slot values, in dependency
/// order, before every prologue: each published result lands in
/// RrStore::propertyResults at once, so a later chain or any read crossing
/// the target sees it. Diagnostics are appended to \p poseDiagnostics in
/// chain order. False only when the section and the classified chains
/// disagree.
bool RrRunPropertyChains(RrProgram *program,
                         std::vector<std::string> *poseDiagnostics);

/// Cross-check (RrProgram::crossCheck): this run's property results
/// against \p record's, path set, tag and bits. False naming the field,
/// e.g. "propertyValues[/Rig/Channels/Dial.rigExec:amount]"; otherwise adds
/// the number of values compared to \p compared.
bool RrCrossCheckPropertyResults(const RrProgram *program,
                                 const RigExecWireFrameInputs &record,
                                 uint64_t *compared, std::string *error);

/// The program's registered reads that cross a property-chain target (the
/// Computed section's chainReads), after the chains and before every
/// prologue: each is evaluated by RrReadInput over this run's slot values
/// and chain results and handed to its uid's holder, which is what the
/// consuming step reads. Under RrProgram::crossCheck each is first compared
/// bit for bit with the value \p record holds for that uid, when it holds
/// one; a mismatch fails naming the field, e.g. "values[uid 12,
/// /Rig/Movers/Follow.inputs:defaultWeight]", otherwise the number compared
/// is added to \p compared.
bool RrRunChainReads(RrProgram *program, const RigExecWireFrameInputs &record,
                     uint64_t *compared, std::string *error);

/// Cross-check (RrProgram::crossCheck), after the chain-read hand-off and
/// before every prologue: each registered read the chain hand-off does not
/// cover, evaluated by RrReadInput, against what the record-driven steps
/// consume for it -- its holder or constant, or for an avar binding the
/// avar table's value -- wherever that is this frame's value (a read
/// made per run only at the frames \p record holds its uid); and each
/// connection-following mover scalar against the value \p record holds
/// under its head's path, where a forced live read put one. A mismatch
/// fails naming the field, e.g. "registered reads: ... at solvers[0].bend
/// (uid 3, /Rig/Solvers/Ik.rigExec:preferredBendRadians)"; otherwise
/// each comparison is added to \p counts under its kind.
bool RrCrossCheckReads(const RrProgram *program,
                       const RigExecWireFrameInputs &record,
                       RrCrossCheckCounts *counts, std::string *error);

/// Cross-check of the reads geometry chain \p chain's revision \p revision
/// (derived target \p revision when \p derived) consumed from \p record
/// this run: its blend channel weights where no pose weight drives the
/// channel and, for a chain revision, its inputs:defaultWeight, each
/// evaluated by RrReadInput. Counted in \p output; false naming the field.
bool RrCrossCheckRevisionReads(const RrProgram *program,
                               const RigExecWireFrameInputs *record,
                               size_t chain, size_t revision, bool derived,
                               RrStepOutput *output, std::string *error);

// Prologues. `poseDiagnostics` carries lines published straight into
// the generation, after the property chains' own; false names the
// failure.
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

/// RigExecRigEvaluator::_ResolveWeights (rigEvaluatorGeometry.cpp) over the
/// Computed section's weight objects: the field a constraint envelope
/// (count 1) and a current-phase revision (count = the entering points,
/// passed as \p current) copy, as the baked program does
/// (bakedWeights.cpp). \p object indexes the section's weight objects.
/// False with the oracle's own error text, checked in the oracle's order;
/// \p weights is then unspecified. Pure: reads slot values, the run's
/// property-chain results, the weight placements and static tables only.
bool RrResolveWeightOracle(const RrProgram *program, size_t object,
                           size_t count, const std::vector<RrVec3f> *current,
                           std::vector<float> *weights, std::string *error);

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
enum class RrRotationFilter : uint8_t { All = 0, Twist = 1, Swing = 2, Orient = 3 };

/// RigExecFilterSpaceRotation: keep only the twist of \p m's rotation about
/// \p axis, or only the swing. `All` returns \p m untouched.
RrMat4d RrFilterSpaceRotation(const RrMat4d &m, const RrVec3d &axis,
                              RrRotationFilter filter);

/// RigExecOrientSpaceDelta: \p delta with the switched frame's origin taken
/// from \p unswitched -- a rotation-only space.
RrMat4d RrOrientSpaceDelta(const RrMat4d &delta, const RrMat4d &local,
                           const RrMat4d &localInverse,
                           const RrMat4d &unswitched);

}  // namespace rigExec

#endif  // RIGEXEC_RUNTIME_STORE_H
