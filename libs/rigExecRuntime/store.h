// rigExecRuntime program state and pipeline contracts (M2 framework).
// RrProgram owns the opened file and the tables Open derives from it;
// RrStore owns every framework-visible slot domain: the avar table, the
// SSA fin/base version pools, matrices, aggregates, commit scratch, the
// prologue's retained arrays and their lasts, the frame records and volume
// placements the steps keep across runs, step outputs and the override
// flags. Every input a step reads is evaluated over the
// input slots (inputs.h); every other stage value it reads is static data
// the bake captured into the file, read through RrStatic. Family .cpps own
// their private scratch (solver live rests, revision packets, weight
// oracles) behind the extension points on RrProgram, and implement the
// pipeline functions declared here. The framework implements
// Open/closure/walk/publish.
#ifndef RIGEXEC_RUNTIME_STORE_H
#define RIGEXEC_RUNTIME_STORE_H

#include "rigExecBinary/external.h"
#include "rigExecBinary/format.h"
#include "rigExecRuntime/inputs.h"
#include "rigExecRuntime/values.h"
#include "rigExecGraph/opValues.h"
#include "rigExecGraph/providerRecords.h"

#include <array>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>
#include <unordered_map>

namespace rigExec {

// Runtime vectors and matrices are bare scalar arrays, so their key bytes
// are their object bytes and contiguous runs of them key with one append.
static_assert(sizeof(RrVec2f) == 2 * sizeof(float) &&
              sizeof(RrVec3f) == 3 * sizeof(float) &&
              sizeof(RrVec3d) == 3 * sizeof(double) &&
              sizeof(RrMat4d) == 16 * sizeof(double) &&
              std::is_trivially_copyable<RrVec2f>::value &&
              std::is_trivially_copyable<RrVec3f>::value &&
              std::is_trivially_copyable<RrVec3d>::value &&
              std::is_trivially_copyable<RrMat4d>::value,
              "key runs need padding-free, trivially copyable elements");
template <> struct RigExecOpKeyBulkElement<RrVec2f> : std::true_type {};
template <> struct RigExecOpKeyBulkElement<RrVec3f> : std::true_type {};
template <> struct RigExecOpKeyBulkElement<RrVec3d> : std::true_type {};
template <> struct RigExecOpKeyBulkElement<RrMat4d> : std::true_type {};

// The runtime's field numbers of each table's inputs, in the tables'
// field order. A registered read (RrRegisteredRead) names the table field
// it binds by them.
enum RrLadderField : int {
    RrLadderRestSpace = 0,
    RrLadderDefaultSpace = 1,
    RrLadderPosedSpace = 2,
    RrLadderRestAvar0 = 3,
    // RrLadderRestAvar0 + k, k in 0..5.
    RrLadderDefaultAvar0 = 9,
    // RrLadderDefaultAvar0 + k, k in 0..5.
    RrLadderRotationOrder = 15,
    RrLadderParentSpace = 16,
    RrLadderParentDefaultSpace = 17,
    RrLadderAvarDefaultSpace = 18,
    RrLadderPosedDefaultSpace = 19,
    RrLadderRotationSign = 20,
    RrLadderInterveningSpace = 21,
    RrLadderFieldCount = 22,
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
    RrSolverPin=14,RrSolverUpperScale=15,RrSolverLowerScale=16,RrSolverSoftDistance=17,RrSolverLimbTwist=18,
    RrSolverFieldCount = 19,
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
    RrConstraintStretch=21,
    RrConstraintFieldCount = 22,
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
// Completed arrays are immutable snapshots. A writable spare becomes the next
// snapshot; an unshared old buffer may be recycled, while retained publications
// keep their old payload intact across later executions and workspace clones.
template <class T> class RrRetainedArray {
    std::shared_ptr<std::vector<T>> _data;
public:
    const std::vector<T> &Read() const {
        return _data ? *_data : Empty();
    }
    static const std::vector<T> &Empty();
    bool empty() const { return Read().empty(); }
    size_t size() const { return Read().size(); }
    const T *data() const { return Read().data(); }
    void clear() { _data.reset(); }
    std::vector<T> &Write() {
        if (!_data) _data=std::make_shared<std::vector<T>>();
        else if (!_data.unique()) _data=std::make_shared<std::vector<T>>(*_data);
        return *_data;
    }
    void swap(std::vector<T> &spare) {
        if (_data && _data.unique()) { _data->swap(spare); return; }
        auto next = std::make_shared<std::vector<T>>();
        next->swap(spare);
        if (_data && _data.unique()) spare.swap(*_data);
        _data = std::move(next);
    }
};
// Namespace-owned empty storage avoids worker-local static initialization.
inline const std::vector<RrVec3f> RrEmptyRetainedPoints;
template <> inline const std::vector<RrVec3f> &
RrRetainedArray<RrVec3f>::Empty() { return RrEmptyRetainedPoints; }

inline const std::vector<float> RrEmptyRetainedWeights;
template <> inline const std::vector<float> &
RrRetainedArray<float>::Empty() { return RrEmptyRetainedWeights; }

struct RrRevisionPublish {
    bool weightFieldPublished = false;
    uint32_t weightFieldTarget = 0;
    RrRetainedArray<float> weightField;
    std::string resultStatus;
    uint32_t target = 0;
};

// One chain's publish row: the deformed points the ChainStatus step
// left, or nothing when the chain has no base this run.
struct RrChainPublish {
    bool haveBase = false;
    uint32_t target = 0;
    RrRetainedArray<RrVec3f> result;
};

// One resolved weight field publication.
// Per-consumer pure weight field result, preserved when its producer skips.
struct RrOracleReadContext {
    std::map<uint32_t, std::vector<int32_t>> versionsBySlot;
    std::map<std::pair<int32_t, int32_t>, size_t> scalarReadIndices;
    std::map<std::pair<int32_t, int32_t>, size_t> pointReadIndices;
};

struct RrWeightFieldResult {
    size_t count = 0;
    std::vector<float> values;
    bool ok = false;
    std::string error;
};

struct RrWeightFieldPublish {
    uint32_t target = 0;
    std::vector<float> weights;
};

// One derived target's publish row.
struct RrDerivedPublish {
    bool haveBase = false;
    uint32_t target = 0;
    RrRetainedArray<RrVec3f> result;
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
// Conversion storage belongs to one provider op. All slots are sized at Open,
// so Read* pointers stay stable while that op evaluates, including repeated IDs.
struct RrProviderConversionScratch {
    std::vector<RrMat4d> matrices;
    std::vector<RrPointFrame> frames;
    std::vector<RrVec3d> vectors;
    std::vector<double> scalars;
};

struct RrProviderRefreshScratch {
    std::vector<RrPointFrame> baseInputs,finInputs,baseOutputs,finOutputs;
    std::vector<uint8_t> blocked;
};

struct RrStore {
    RigExecOpWorkspace opWorkspace;
    RigExecOpExecution opExecution;
    RigExecOpAdapterState opAdapter;
    std::vector<double> avars, lastAvars;
    std::vector<RrPointFrame> base, fin;
    std::vector<uint32_t> finLast, baseLast;
    std::vector<RrMat4d> posedM, finalMatrix, baseMatrix;
    std::vector<RrPointFrameArray> aggregates;
    std::vector<RrLadderLive> ladders;
    std::vector<int> ladderMovedSlots;
    std::vector<std::vector<RrPointFrame>> solverOutFrames;
    std::vector<std::vector<char>> solverOutPresent;
    // Per solver, the ribbon driver points the solve samples: the points
    // the bake read, set at Open.
    std::vector<std::vector<RrVec3f>> ribbonConstant;
    std::vector<RrCommitScratch> commits;
    std::vector<RrConstraintArraysLive> arrays;
    // The property chains' results this run (properties.cpp), keyed by the
    // attribute path id they are published at: chain targets and phased
    // consumers. Every read of such an attribute sees them.
    std::map<uint32_t, RrPropertyValue> propertyResults;
    // The same results by publish entry (one per attribute a chain or a
    // phased consumer publishes at), with whether this run published it;
    // an unpublished entry holds a zero value. These are current
    // publication values, separate from indexed version change tracking.
    std::vector<RrPropertyValue> propertyValues;
    std::vector<RrPropertyValue> propertyVersions;
    std::vector<char> propertyVersionValid, propertyVersionChanged;
    std::vector<char> propertyChainValid, propertyRecordStoodAside;
    std::vector<int32_t> propertyPublishedVersions;
    std::unordered_map<uint32_t, uint32_t> propertyPathSlots;
    std::vector<char> headOutputChanged, headRan;
    std::vector<char> restChanged, ladderChanged, topologyChanged;
    std::vector<std::vector<std::string>> headLines;
    std::vector<std::string> headMemoKeys;
    std::vector<std::string> opInputScratch;
    /// verifyLeafVersions: each step's source memo over array contents.
    std::vector<std::string> headContentKeys;
    /// verifyChainVersions: per value id, the last published key of a
    /// point-carrying value over its points' bytes.
    std::vector<std::string> chainContentKeys;
    std::vector<RigExecProviderPlainState> providerValues;
    std::vector<RrProviderConversionScratch> providerConversionScratch;
    std::vector<RrProviderRefreshScratch> providerRefreshScratch;
    std::vector<char> propertyPublished;
    std::vector<char> chainHaveBase, chainBaseDirty, lastHaveBase;
    std::vector<char> derivedHaveBase;
    std::vector<RrPointFrame> nativeFrames;
    std::vector<char> nativeFrameOk;
    std::vector<RrMat4d> xformBase, lastXformBase;
    std::vector<RrMat4d> switchFrames;
    std::vector<RrMat4d> deltaBaseMatrix;
    std::vector<char> deltaBaseOk;
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
    // Per provider slot, what that volume's VolumePlacements step last
    // wrote (the program's slot-indexed volumePlacement) and whether it has
    // run. Sized at Open and never cleared, so a step the closure skipped
    // keeps its placement. The oracle and the published weight frames
    // read them.
    std::vector<RrMat4d> volumePlacement;
    std::vector<char> volumePlaced;
    // Per pose.frame_records entry, what its FrameMatrix step last wrote:
    // the provider matrix as the record's writer left it, and whether the
    // writer recorded it (identity and 0 where it did not). Sized at Open
    // and never cleared, so a step the closure skipped keeps its record.
    // An AtPrim transform phase reads them in the fold.
    std::vector<RrMat4d> frameMatrix;
    std::vector<char> frameMatrixValid;
    std::vector<float> poseWeights;
    // One flag per override number (RigExecBakedProgramImpl::overridden),
    // set while an input on the read's walk that is not Animated holds
    // other than its default, so the read walks the slots.
    std::vector<char> overridden;
    bool anyOverridden = false;
    // One flag per override number, set when an input on the read's walk
    // that is not Animated took a value (bits or HasValue) other than the
    // one the last run read. The closure re-runs the number's readers once
    // from it, and clears it.
    std::vector<char> changedSinceRun;
    bool anyChangedSinceRun = false;
    std::map<uint32_t, RrMat4d> providerXforms, providerBaseXforms;
    std::map<uint32_t, RrMat4d> jointMatricesFinal;
    std::map<uint32_t, RrPointFrame> jointFramesBase, jointFramesFinal;
    std::map<uint32_t, RrPointFrame> controlFrames;
    std::map<uint32_t, std::vector<RrVec3f>> movedProperties;
    /// Matrix primvars a surface projector published, by property.
    std::map<uint32_t, RrMat4d> movedMatrices;
    std::map<uint32_t, RrWeightFieldPublish> weightFields;
    std::vector<RrWeightFieldResult> weightFieldResults;
    std::vector<char> weightFieldChanged;
    std::vector<RrOracleReadContext> weightFieldContexts;
    std::vector<RrMat4d> volumePlacementBase;
    std::vector<char> volumePlacedBase;
    std::vector<char> jointMatrixPublished;
    std::vector<RrStepOutput> stepOutputs;
    std::vector<uint64_t> closedWords, closedSteps;
    // The steps the last Execute ran, by index, in the order it ran them.
    // Reserved at Open to the step count.
    std::vector<int32_t> runTrace;
    bool everRan = false;
    // An Animated input was set, or the caller said time moved
    // (RigExecRuntimeReader::TouchAnimatedInputs): the next closure dirties
    // what a change of time dirties in the program, then clears it.
    bool animatedTouched = false;
    size_t lastClosedClusters = 0;
    // The slot-keyed leaves (RrProgram::slotKeyedLeaves) a run re-keys:
    // scratch, and how many the last run re-keyed.
    std::vector<RigExecValueId> leafQueue;
    size_t slotLeafKeys = 0;
};

// The static data the steps read besides the tables and the input slots:
// stage values the bake captured once, at the bake time, into the file's
// tables (xform bases, native sources, constraint arrays, chain and
// derived bases, plugin movers' frame bytes). Every read of them goes
// through these accessors; the validator checked the sizes of the tables
// they index blindly. The geometry family converts the chain and derived
// bases and the dense blend sample points once, at Open.
struct RrStatic {
    const RigExecWireFile *file = nullptr;

    /// The base matrix of xform-derived slot entry \p k.
    const RigExecWireMatrix4d &XformBase(size_t k) const
    {
        return file->pose->xformBase[k];
    }
    /// Native source \p k's frame.
    const RigExecWireFrame &NativeFrame(size_t k) const
    {
        return file->pose->nativeSources[k].frame;
    }
    /// Geometry-domain constraint delta base \p k and whether it resolved.
    const RigExecWireMatrix4d &DeltaBase(size_t k) const
    {
        return file->geometry->deltaBaseMatrix[k];
    }
    bool DeltaBaseOk(size_t k) const
    {
        return file->geometry->deltaBaseOk[k] != 0;
    }
    /// Constraint array \p k's authored tables.
    const std::vector<double> &ArrayWeights(size_t k) const
    {
        return file->pose->constraintArrays[k].weights;
    }
    const std::vector<RigExecWireVec3d> &ArrayTranslationOffsets(
        size_t k) const
    {
        return file->pose->constraintArrays[k].translationOffsets;
    }
    const std::vector<RigExecWireVec3d> &ArrayRotationOffsets(size_t k) const
    {
        return file->pose->constraintArrays[k].rotationOffsets;
    }
    bool ArrayOk(size_t k) const { return file->pose->constraintArrays[k].ok; }
    const std::vector<double> &ArrayPoleWeights(size_t k) const
    {
        return file->pose->constraintArrays[k].poleWeights;
    }
    bool ArrayPoleOk(size_t k) const
    {
        return file->pose->constraintArrays[k].poleOk;
    }
    /// The lines constraint array \p k's source and pole table reads
    /// reported, which the constraint step replays.
    const std::vector<std::string> &ArrayDiagnostics(size_t k) const
    {
        return file->pose->constraintArrays[k].diagnostics;
    }
    const std::vector<std::string> &ArrayPoleDiagnostics(size_t k) const
    {
        return file->pose->constraintArrays[k].poleDiagnostics;
    }
    /// Geometry chain \p chain's base points as the bake captured them, or
    /// null when it has none. A chain with a base slot reads that slot
    /// instead (RrGeoChainBase, the weight gathers).
    const std::vector<RigExecWireVec3f> *ChainBase(size_t chain) const
    {
        const RigExecWireChain &wire = file->geometry->chains[chain];
        return wire.haveBase ? &file->vec3fArrays[wire.base].v : nullptr;
    }
    /// Derived target \p id's base points, or null when it has none.
    const std::vector<RigExecWireVec3f> *DerivedBase(size_t id) const
    {
        const auto &entry = file->geometry->derivedIndex[id];
        const RigExecWireDerived &wire =
            file->geometry->chains[size_t(entry.first)]
                .derived[size_t(entry.second)];
        return wire.haveBase ? &file->vec3fArrays[wire.base].v : nullptr;
    }
    /// Plugin revision entry \p k's frame bytes, or null when the
    /// plugin's assembly failed at the bake time.
    const std::vector<uint8_t> *ExternalFrame(size_t k) const
    {
        const RigExecWireExternalMover &mover = file->externalMovers[k];
        return mover.v2FrameValid ? &mover.v2Frame : nullptr;
    }
};

// The tables a registered read belongs to, in the order the frozen
// context's patchable-input walk visits them.
enum class RrReadFamily : uint8_t {
    AvarBinding = 0,
    AvarConstantBinding = 1,
    Ladder = 2,
    SpaceSwitch = 3,
    Interpolator = 4,
    Solver = 5,
    Constraint = 6,
    WeightObject = 7,
    AutoClavicle = 8,
};

// One program-registered read (a RigExecBakedInput the program holds),
// Baked mode, bound in place: the Input of table field `field` of row
// `object` of its family's table (an interpolator's enable is field 0 and
// its dial k is 1 + k; an avar binding's is 0). `avar` is an avar
// binding's flat avar index (slot * 11 + channel), -1 for every other
// family.
struct RrRegisteredRead {
    const RigExecWireInput *read = nullptr;
    RrReadFamily family = RrReadFamily::Ladder;
    uint32_t object = 0;
    uint32_t field = 0;
    int32_t avar = -1;
};

// The geometry kernels' environment settings. Open reads them once, so no
// run reads the environment; a program a test assembles by hand keeps
// these defaults.
struct RrGeoSettings {
    /// RIGEXEC_ENABLE_SIMD: the skin and matrix kernels take the SSE2 path.
    bool useSimd = true;
};

// The opened program plus its working state. The table pointers borrow
// from `file`; the store is sized at Open.
struct RrProgram {
    // Explicit captured source state; no runtime Stage lookup or inferred frame validity.
    fb::RigExecWireRequiredStageFramesAdmission requiredStageFramesAdmission;
    bool requiredStageFramesFullOwed=false;
    RigExecCompiledGraph opGraph;
    RigExecProviderPlainProgram providerProgram;
    /// What Open decoded. A program a test assembles by hand leaves it
    /// null and points the tables below at its own.
    std::unique_ptr<RigExecWireFile> file;

    const std::vector<RigExecWireStep> *steps = nullptr;
    const RigExecWireClustering *clustering = nullptr;
    const RigExecWireCones *cones = nullptr;
    const RigExecWireSlotMeta *slotMeta = nullptr;
    const RigExecWireConstants *constants = nullptr;
    const RigExecWireDomainPose *poses = nullptr;
    const RigExecWireDomainGeometry *geometry = nullptr;
    /// The weight objects WeightPacket steps build: the leading entries of
    /// geometry->weightObjects, before the envelope-only ones.
    size_t stepWeightObjects = 0;
    /// Each path node's text, by id: one pass, a parent before its child.
    std::vector<std::string> nodeText;
    /// The static data every run reads, plugin movers' bytes included.
    RrStatic statics;
    /// The assemblers' stage reads (RrPathRead), sorted by (path, rest),
    /// built at Open; and the indices of its read rows, which every
    /// geometry prologue evaluates. The families resolve the rows their
    /// sites read at Open and index the table by them.
    std::vector<RrPathRead> pathReads;
    std::vector<uint32_t> pathReadRows;
    /// Read from the environment at Open; the geometry steps pass them to
    /// the kernels.
    RrGeoSettings geoSettings;
    /// RIGEXEC_VERIFY_CONSTANT_KEYS, read at Open: every run rebuilds the
    /// constant source memos the closure skips and fails if one moved.
    bool verifyConstantSources = false;
    /// RIGEXEC_VERIFY_LEAF_VERSIONS, read at Open: every run also builds
    /// each source memo over array contents and fails if the two disagree
    /// on a change.
    bool verifyLeafVersions = false;
    /// RIGEXEC_VERIFY_SPARSE_LEAVES, read at Open: every run re-keys the
    /// slot-keyed leaves it skips and fails if one moved.
    bool verifySparseLeaves = false;
    /// RIGEXEC_VERIFY_CHAIN_VERSIONS, read at Open: every run also keys each
    /// published point content version over the points' bytes and fails if
    /// the two disagree on a change.
    bool verifyChainVersions = false;
    /// What the epilogue visits instead of every step, as the native
    /// RigExecBakedProgramImpl::EpilogueIndex: indexed with the op graph
    /// (RrIndexEpilogue) and folded after each run (RrFoldHeldSteps).
    struct EpilogueIndex {
        /// Per solver, 1 where a Solve step publishes it.
        std::vector<char> aliveSolver;
        /// The RevisionStatic, ChainStatus and Derived steps, ascending.
        std::vector<uint32_t> geometrySteps;
        /// The steps holding diagnostics or head lines now.
        RigExecHeldSteps held;
        /// RIGEXEC_VERIFY_EPILOGUE_LISTS, read at Open: every block is
        /// also swept from all steps, and a run whose blocks differ fails.
        bool verify = false;
        size_t mismatches = 0;
    } epilogue;
    /// The leaves whose keys read input slots alone (provider leaves and
    /// constraint input arrays), by the slots they read:
    /// slotLeaves[slotLeafBegin[s], slotLeafBegin[s + 1]). slotKeyedLeaves
    /// lists them and otherLeaves every other leaf, each in id order. Built
    /// with the op graph; an empty slotLeafBegin re-keys every leaf per run.
    std::vector<uint32_t> slotLeafBegin;
    std::vector<RigExecValueId> slotLeaves, slotKeyedLeaves, otherLeaves;

    /// One plugin revision's playback state, in external_movers order. No
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

    /// Every registered read, bound in place, in family order;
    /// RrInputsBindReads fills it and the tables below.
    std::vector<RrRegisteredRead> registeredReads;
    // The registered read (an index into registeredReads) each table field
    // binds, by row and field number.
    std::vector<std::array<int32_t, RrLadderFieldCount>> ladderRead;
    std::vector<int32_t> spaceSwitchRead;
    std::vector<std::vector<int32_t>> autoClavicleRead;
    std::vector<int32_t> interpRead;
    // Per interpolator, one read per numeric dial. Three because the
    // compile reads at most three, one per axis, and refuses a fourth.
    std::vector<std::array<int32_t, 3>> interpValueRead;
    // Per provider slot: its space switch's index, or -1. Built once at
    // Open so the compose pays one array lookup per slot and a rig with
    // no switch pays nothing at all. Empty when the binary carries none.
    std::vector<int32_t> spaceSwitchBySlot;
    // Exact native declaration-derived seed steps, independent of clustering.
    std::vector<std::vector<int32_t>> avarReaderSteps, chainBaseSteps, revisionSteps;
    std::vector<int32_t> revisionStaticStep;
    std::vector<std::array<int32_t, RrSolverFieldCount>> solverRead;
    std::vector<std::array<int32_t, RrConstraintFieldCount>> constraintRead;
    std::vector<std::array<int32_t, RrWeightFieldCount>> weightRead;

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

    // The file's compile notices, replayed ahead of the program lines on
    // the first Execute, then drained.
    std::vector<std::string> compileDiagnostics;

    // The input slots and their values this run (inputs.h).
    RrInputState inputState;

    // A path node's text, or token text an input set interned
    // (inputState.extraTokenBase and up).
    bool GetText(uint32_t id, std::string *out) const
    {
        if (id < nodeText.size()) {
            *out = nodeText[id];
            return true;
        }
        if (id < inputState.extraTokenBase) {
            return false;
        }
        const size_t k = size_t(id - inputState.extraTokenBase);
        if (k >= inputState.extraTokens.size()) {
            return false;
        }
        *out = inputState.extraTokens[k];
        return true;
    }

    std::string TextOrEmpty(uint32_t id) const
    {
        std::string out;
        GetText(id, &out);
        return out;
    }

    bool TokenEquals(uint32_t id, const char *literal) const
    {
        return TextOrEmpty(id) == literal;
    }

    // Registered read \p read, bound in place.
    const RigExecWireInput &RegisteredInput(int32_t read) const
    {
        return *registeredReads[size_t(read)].read;
    }
    // The runtime form of RigExecBakedRead: the registered read evaluated
    // over this run's slots.
    RrInputValue ReadRegistered(int32_t read) const
    {
        return RrValueFromWire(RrReadInput(this, RegisteredInput(read)));
    }
    // The read's folded constant (RigExecBakedInput::constant).
    RrInputValue RegisteredConstant(int32_t read) const
    {
        return RrValueFromWire(
            inputState.values[RegisteredInput(read).constant]);
    }

    RrInputValue ReadLadder(size_t slot, int field) const
    {
        const auto &ladder=poses->ladders[slot];
        const int channel=field<=2?field:field==RrLadderParentSpace?3:
            field==RrLadderParentDefaultSpace?4:field==RrLadderAvarDefaultSpace?5:
            field==RrLadderPosedDefaultSpace?6:-1;
        if(channel>=0 && size_t(channel)<ladder.spaceValues.size()) {
            const int id=ladder.spaceValues[size_t(channel)];
            if(id>=0 && size_t(id)<store.providerValues.size()) {
                const auto &state=store.providerValues[size_t(id)];
                const auto *matrix=std::get_if<std::array<double,16>>(&state.value);
                if(state.initialized && !state.blocked && matrix) {
                    RrInputValue value; value.tag=RrInputTag::Matrix4d;
                    std::memcpy(value.matrix._mtx,matrix->data(),sizeof(double)*16);
                    return value;
                }
            }
        }
        return ReadRegistered(ladderRead[slot][size_t(field)]);
    }
    RrInputValue ReadSolver(size_t solver, int field) const
    {
        return ReadRegistered(solverRead[solver][size_t(field)]);
    }
    RrInputValue ReadConstraint(size_t constraint, int field) const
    {
        return ReadRegistered(constraintRead[constraint][size_t(field)]);
    }
    RrInputValue ReadWeight(size_t object, int field) const
    {
        return ReadRegistered(weightRead[object][size_t(field)]);
    }
    RrInputValue ReadInterp(size_t interp) const
    {
        return ReadRegistered(interpRead[interp]);
    }
    RrInputValue ReadInterpValue(size_t interp, size_t axis) const
    {
        return ReadRegistered(interpValueRead[interp][axis]);
    }
    RrInputValue ReadSpaceSwitch(size_t index) const
    {
        return ReadRegistered(spaceSwitchRead[index]);
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
/// Test-only: whether the last Execute found mover \p moverPath's chain
/// revision partition stale, so the revision ran whole. False when no chain
/// revision moves for it.
bool RrGeometryPartitionStaleForTesting(const RrProgram *program,
                                        const std::string &moverPath);
/// Test-only: mover \p moverPath's chain revision's apply-or-fail decision
/// as the last RevisionStatic made it (0 refuses, 1 applies, 2 deferred),
/// and whether every one of its chunks reported ok. False when no chain
/// revision moves for it.
bool RrGeometryRevisionDecisionForTesting(const RrProgram *program,
                                          const std::string &moverPath,
                                          int *acceptance, bool *chunksOk);
/// Test-only: whether the last Execute's layout of mover \p moverPath's
/// chain revision is the one Open expanded from the file. False when no
/// chain revision moves for it, or it has no layout.
bool RrGeometrySkinLayoutIsOpenForTesting(const RrProgram *program,
                                          const std::string &moverPath);
bool RrWeightSizeScratch(RrProgram *program, std::string *error);
/// Classifies the file's property chains and sizes the publish entries.
/// Runs after the reads are bound.
bool RrPropertySizeScratch(RrProgram *program, std::string *error);

/// The property chains (RigExecRigEvaluator::_EvaluatePropertyChains,
/// rigEvaluatorProperties.cpp) over this run's slot values, in dependency
/// order, before every prologue: each published result lands in
/// RrStore::propertyResults at once, so a later chain or any read crossing
/// the target sees it. Diagnostics are appended to \p poseDiagnostics in
/// chain order. False only when the file and the classified chains
/// disagree.
bool RrRunPropertyPart(RrProgram *program, size_t chain, size_t part,
                       std::vector<std::string> *diagnostics);
void RrPropertyBegin(RrProgram *program);
void RrPropertyPublish(RrProgram *program);
void RrPropertyPublishFinished(RrProgram *program, const std::vector<char> &finished);
bool RrRunRestHead(RrProgram *program, size_t group, bool ladder);
bool RrRunTopologyHead(RrProgram *program, size_t revision);
bool RrRunPropertyChains(RrProgram *program,
                         std::vector<std::string> *poseDiagnostics);

// Prologues, over this run's slots and RrProgram::statics.
// `poseDiagnostics` carries lines published straight into the generation,
// after the property chains' own; false names the failure.
bool RrProloguePose(RrProgram *program,
                    std::vector<std::string> *poseDiagnostics,
                    std::string *error);
bool RrPrologueGeometry(RrProgram *program,
                        std::vector<std::string> *poseDiagnostics,
                        std::string *error);

// Step bodies by family.
bool RrRunPoseStep(RrProgram *program, size_t step, std::string *error);
bool RrRunGeometryStep(RrProgram *program, size_t step, std::string *error);
bool RrWeightFieldEffectiveInputMemo(const RrProgram *, uint32_t, std::string *, std::vector<uint32_t> *);
bool RrRunWeightStep(RrProgram *program, size_t step, std::string *error);
bool RrRevisionWeightFieldInput(const RrProgram *, size_t, const RrVec3f **, size_t *);
bool RrRunRevisionWeightField(RrProgram *program, size_t fieldIndex);
// Evaluate one compiled per-consumer field without stage or publication-map
// lookup. The graph producer supplies the actual entering point count.
bool RrResolveDeclaredWeightField(RrProgram *program, size_t fieldIndex,
                                  size_t count,
                                  const std::vector<RrVec3f> *current);
void RrSkipGeometryStep(RrProgram *program, size_t step);

/// RigExecRigEvaluator::_ResolveWeights (rigEvaluatorGeometry.cpp) over the
/// file's weight objects: the field a constraint envelope
/// (count 1) and a current-phase revision (count = the entering points,
/// passed as \p current) copy, as the baked program does
/// (bakedWeights.cpp). \p object indexes geometry.weight_objects.
/// False with the oracle's own error text, checked in the oracle's order;
/// \p weights is then unspecified. Pure: reads slot values, the run's
/// property-chain results, the weight placements and static tables only.
bool RrResolveWeightOracle(const RrProgram *program, size_t object,
                           size_t count, const std::vector<RrVec3f> *current,
                           std::vector<float> *weights, std::string *error);

// Executor and epilogue (framework). The closure dirties what the inputs
// set since the last run reach; RrStore::animatedTouched stands for a
// change of time.
bool RrRunSteps(RrProgram *program, bool force, std::string *error);
/// Builds RrProgram::epilogue from the steps and their output as they
/// stand, keeping `verify` and `mismatches`.
void RrIndexEpilogue(RrProgram *program);
/// Notes the output of every op the last run ran; after the executor,
/// failed run or not.
void RrFoldHeldSteps(RrProgram *program);
bool RrPublishPose(RrProgram *program,
                   std::vector<std::string> *poseDiagnostics,
                   std::string *error);
void RrPublishGeometry(RrProgram *program,
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
