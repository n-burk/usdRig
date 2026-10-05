// .rigexec DomainPose section: the pose-domain build tables.
// Ladders, pose interpolators (+ solved RBF tables), solvers, constraints,
// walk steps and commits: everything the pose-half steps index into. Bound
// inputs travel as tagged constants plus their read-route flags; the USD
// handles behind them (queries, attributes) do not -- per-frame value
// streams arrive with the InputTable section (M1 slice 4), which keys off
// the head paths and override indices recorded here.
#ifndef RIGEXEC_BINARY_POSE_H
#define RIGEXEC_BINARY_POSE_H

#include "rigExecBinary/program.h"

namespace rigExec {

/// A bound input's constant, tagged. Mirrors RigExecBakedInput<T> for the
/// seven T the program instantiates; the USD handles are replaced by the
/// read-route facts slice 4 needs: whether USD answers per frame (bound),
/// whether the route goes through the resolved inputs (viaResolved), and
/// where the walk starts (head) for override matching.
struct RigExecWireInput {
    enum class Tag : uint8_t {
        Double = 0,
        Float = 1,
        Bool = 2,
        Int = 3,
        Matrix4d = 4,
        Token = 5,
        Vec3d = 6,
    };
    Tag tag = Tag::Double;
    double f64 = 0;
    float f32 = 0;
    bool boolean = false;
    int32_t i32 = 0;
    RigExecWireMatrix4d matrix{};
    uint32_t token = 0;
    RigExecWireVec3d vec{};
    bool varying = false;
    bool bound = false;
    bool viaResolved = false;
    int32_t overrideIndex = -1;
    uint32_t head = 0;
};

/// An unbound Matrix4d input holding the identity.
inline RigExecWireInput
RigExecWireIdentityMatrixInput()
{
    RigExecWireInput input;
    input.tag = RigExecWireInput::Tag::Matrix4d;
    for (size_t i = 0; i < 4; ++i) {
        input.matrix[i * 4 + i] = 1.0;
    }
    return input;
}

/// One provider slot's rest/default ladder. Xform-derived slots carry a
/// default one, exactly as the program holds.
struct RigExecWireLadder {
    RigExecWireInput restSpace;
    RigExecWireInput defaultSpace;
    RigExecWireInput posedSpace;
    RigExecWireInput restAvars[6];
    RigExecWireInput defaultAvars[6];
    RigExecWireInput rotationOrder;
};

/// A solved RBF table: the desc-equivalent inputs plus the solved widths
/// and inverted matrix, which is what SetSolvedTable reconstitutes. The
/// falloff vector is NOT carried: painted post-fit widths have no falloff
/// vector that reproduces them, so the widths travel explicitly.
struct RigExecWireRbf {
    std::vector<RigExecWireVec3d> poses;
    std::vector<RigExecWireVec3d> translations;
    /// Whole/Swing/Twist per pose, in RigExecRbfPoseType order. Empty to
    /// measure whole rotations.
    std::vector<uint8_t> poseTypes;
    RigExecWireVec3d twistAxis{};
    /// Gaussian/Linear, in RigExecRbfKernel order.
    uint8_t kernel = 0;
    double radius = 0;
    double translationRadius = 0;
    std::vector<double> radii;
    std::vector<double> translationRadii;
    double regularization = 0;
    bool normalize = true;
    bool enableRotation = true;
    bool enableTranslation = false;
    bool regularizedSingular = false;
    std::vector<std::vector<double>> weights;
};

struct RigExecWirePoseInterpolator {
    uint32_t path = 0;
    /// -1 on a NUMERIC driver, which reads dials and no frame at all.
    int32_t driverSlot = -1;
    int32_t parentSlot = -1;
    bool allowNegativeWeights = true;
    RigExecWireInput enabled;
    int32_t weightBegin = 0;
    int32_t weightEnd = 0;
    std::vector<int32_t> poseSlots;
    std::vector<int32_t> disabledSlots;
    RigExecWireRbf solver;
    /// Whether the solve measures the driver's translation as well.
    /// Memory only: the interpolator record layout is frozen, so this and
    /// the numeric dials travel in the optional PoseNumeric section and
    /// the loader overlays them after decoding the pose domain.
    bool enableTranslation = false;
    /// A numeric driver's dials, one per axis, at most three. Empty on a
    /// transform-driven interpolator. Memory only, as above.
    std::vector<RigExecWireInput> valueInputs;
};

struct RigExecWireSplineIkRest {
    std::array<RigExecWireVec3d, 4> cvs{};
    RigExecWireFrame rootControl;
    RigExecWireFrame midControl;
    RigExecWireFrame endControl;
    std::vector<RigExecWireFrame> joints;
    std::vector<double> segmentLengths;
    double restArcLength = 0;
    std::vector<double> volumeWeights;
};

/// One solver's epoch description. Per-frame scratch (outputs, fallback
/// joints, live ribbon points, FK elements) is NOT on the wire.
struct RigExecWireSolver {
    uint32_t path = 0;
    uint32_t type = 0;
    std::vector<int32_t> restSlots;
    std::vector<std::pair<int32_t, int32_t>> restRefs;
    std::vector<uint8_t> restIsLive;
    std::vector<uint32_t> restReads;
    bool hasLiveRest = false;
    std::vector<std::array<RigExecWireVec3d, 4>> jointRests;
    bool restsVary = false;
    std::vector<int32_t> restOverrides;
    std::vector<double> splineRestWeights;
    /// Curve/Chain, in RigExecSplineIkRestLength order.
    uint8_t splineRestMode = 0;
    bool degenerate = false;
    // FkChain
    std::vector<int32_t> controls;
    bool parentRelative = false;
    std::vector<std::array<RigExecWireVec3d, 4>> controlRests;
    /// rigExec:startFrame provider slot (-1: absolute), its rest, and the
    /// bound `fin` version. Memory only: the solver record layout is
    /// frozen, so these travel in the optional SolverStart section and
    /// the loader overlays them after decoding the pose domain.
    int32_t start = -1;
    std::array<RigExecWireVec3d, 4> startRest{};
    uint32_t startRead = 0;
    // IK / spline controls
    int32_t root = -1;
    int32_t mid = -1;
    int32_t end = -1;
    int32_t pole = -1;
    std::array<std::array<RigExecWireVec3d, 4>, 3> ikRests{};
    RigExecWireTwoBoneIkParams ikParams;
    RigExecWireInput bend;
    RigExecWireInput upperOffset;
    RigExecWireInput lowerOffset;
    RigExecWireInput stretch;
    RigExecWireInput softness;
    double upperLengthBase = 0;
    double lowerLengthBase = 0;
    /// rigExec:spaceMatrix, the explicit factor of the space a TwoBoneIk
    /// measures its chain in, and rigExec:space: the provider slot whose
    /// rest -> pose map composes ahead of it (-1: none), that slot's rest
    /// landmarks and the bound `fin` version. Memory only on the solver
    /// record: a trailing block after every pose table carries them, so a
    /// binary that ends before it decodes with the identity and no space.
    RigExecWireInput ikSpace = RigExecWireIdentityMatrixInput();
    int32_t spaceSlot = -1;
    std::array<RigExecWireVec3d, 4> spaceRest{};
    uint32_t spaceRead = 0;
    // BlendPointFrames
    int32_t inA = -1;
    int32_t inB = -1;
    RigExecWireInput blendWeight;
    /// Log/Linear, in RigExecScaleBlend order.
    uint8_t scaleMode = 0;
    bool blendRotationRejected = false;
    // SplineIk
    RigExecWireSplineIkRest splineRest;
    RigExecWireSplineIkParams splineParams;
    std::vector<std::array<RigExecWireVec3d, 4>> splineJointRests;
    uint64_t splineCount = 0;
    RigExecWireInput preserveVolume;
    RigExecWireInput midFollowWeight;
    RigExecWireInput roll;
    RigExecWireInput twist;
    RigExecWireInput minLengthRatio;
    bool splineParamsVary = false;
    // TwistDistribution
    std::array<RigExecWireVec3d, 4> twistStartRest{};
    std::array<RigExecWireVec3d, 4> twistEndRest{};
    std::vector<double> twistWeights;
    RigExecWireInput twistTurns;
    // Ribbon
    uint32_t ribbonPointsPath = 0;
    std::vector<RigExecWireVec3f> ribbonRestPoints;
    std::vector<RigExecWireVec3f> ribbonConstantPoints;
    bool ribbonPointsVarying = false;
    RigExecWireInput ribbonSampleCount;
    // Outputs
    std::vector<std::pair<int32_t, int32_t>> outputs;
    std::vector<int32_t> outPosition;
    std::vector<uint32_t> controlReads;
    uint32_t rootRead = 0;
    uint32_t midRead = 0;
    uint32_t endRead = 0;
    uint32_t poleRead = 0;
};

/// One constraint's epoch description. Weight-oracle scratch is NOT on the
/// wire: the runtime re-resolves per frame like the program does.
/// Bits of RigExecWireConstraint::flags.
enum RigExecWireConstraintFlag : uint8_t {
    /// rigExec:blendShear on a Scale or Parent constraint.
    RigExecWireConstraintBlendShear = 1u << 0,
    /// rigExec:worldUpRotationOnly on an Aim constraint.
    RigExecWireConstraintWorldUpRotationOnly = 1u << 1,
    /// rigExec:weightBlend = "radial" on a transform-domain matrix mover.
    RigExecWireConstraintRadialBlend = 1u << 2,
};
/// Every bit this reader understands; any other bit is refused.
inline constexpr uint8_t RigExecWireConstraintKnownFlags =
    RigExecWireConstraintBlendShear |
    RigExecWireConstraintWorldUpRotationOnly |
    RigExecWireConstraintRadialBlend;

struct RigExecWireConstraint {
    uint32_t path = 0;
    uint32_t type = 0;
    uint32_t weightObject = 0;
    /// The envelope's entry in the Computed section's weight objects (a
    /// weight object and no points target), else -1. Memory only: the
    /// pose section does not encode it; RigExecWireApplyComputed fills it.
    int32_t weightObjectIndex = -1;
    int32_t target = -1;
    std::vector<int32_t> targetSlots;
    std::vector<uint8_t> snapshotTargets;
    std::vector<int32_t> sources;
    std::vector<int32_t> sourceNatives;
    std::vector<uint32_t> sourcePaths;
    int32_t arrays = -1;
    RigExecWireInput enabled;
    RigExecWireInput defaultWeight;
    RigExecWireInput offset;
    RigExecWireInput affectX;
    RigExecWireInput affectY;
    RigExecWireInput affectZ;
    RigExecWireInput tX;
    RigExecWireInput tY;
    RigExecWireInput tZ;
    RigExecWireInput rX;
    RigExecWireInput rY;
    RigExecWireInput rZ;
    RigExecWireInput sX;
    RigExecWireInput sY;
    RigExecWireInput sZ;
    /// XYZ..ZYX, in RigExecEulerOrder order.
    uint8_t order = 0;
    RigExecWireInput aimVector;
    RigExecWireInput upVector;
    RigExecWireInput rotationOffset;
    RigExecWireInput worldUpVector;
    RigExecWireVec3d aimAxisFallback{};
    bool aimVectorAuthored = false;
    bool preserveInputUp = false;
    uint32_t worldUpType = 0;
    RigExecWireVec3d sceneUp{};
    uint32_t pointsTarget = 0;
    uint32_t deltaBasePath = 0;
    int32_t deltaBase = -1;
    int32_t worldUpObject = -1;
    int32_t worldUpNative = -1;
    uint32_t worldUpPath = 0;
    bool worldUpObjectNamed = false;
    /// rigExec:space on a rotation constraint, or -1 for none. Encoded as
    /// a trailing block at the END of the pose section, after every table,
    /// so a binary baked before it decodes with -1 on every constraint --
    /// the behaviour those files were baked from. See
    /// RigExecWireDecodeDomainPose.
    int32_t spaceSlot = -1;
    /// RigExecWireConstraintFlag bits: opt-in behaviours that change the
    /// operator's arithmetic. A second trailing block after the space
    /// slots; a binary without it decodes as 0, the original arithmetic.
    uint8_t flags = 0;
    bool snapshotAfter = false;
    // SingleChainIK
    bool singleChainIk = false;
    /// RotatePlane/SingleChain, in RigExecSingleChainIkMode order.
    uint8_t ikMode = 0;
    bool poleModeObject = false;
    bool useAnimatedTs = false;
    std::vector<uint8_t> ikRestLive;
    int32_t effector = -1;
    int32_t effectorNative = -1;
    uint32_t effectorPath = 0;
    std::vector<int32_t> poleObjects;
    std::vector<int32_t> poleObjectNatives;
    RigExecWireInput poleVector;
    RigExecWireInput twistDegrees;
};

struct RigExecWireWalkStep {
    bool solverBatch = false;
    uint64_t level = 0;
    int32_t index = 0;
    std::vector<int32_t> batchSolvers;
    std::vector<std::pair<int32_t, int32_t>> propagate;
};

/// One commit's epoch decisions: merge shape, propagation pairs, and every
/// version index. Per-run candidate frames, deltas, staging and flags are
/// NOT on the wire.
struct RigExecWireCommit {
    uint32_t moverPath = 0;
    bool solverOutput = false;
    std::vector<int32_t> slots;
    std::vector<std::pair<int32_t, int32_t>> propagate;
    std::vector<int32_t> closestPos;
    bool split = false;
    int32_t stagingBase = 0;
    std::vector<RigExecWireConstraintSource> sources;
    std::vector<uint32_t> slotReads;
    std::vector<uint32_t> slotWrites;
    std::vector<uint32_t> slotBaseWrites;
    std::vector<uint32_t> descendantReads;
    std::vector<uint32_t> closestReads;
    std::vector<uint32_t> descendantWrites;
    std::vector<uint32_t> descendantBaseWrites;
    std::vector<uint32_t> slotCarry;
    std::vector<uint32_t> slotBaseCarry;
    std::vector<uint32_t> descendantCarry;
    std::vector<uint32_t> descendantBaseCarry;
    std::vector<uint32_t> sourceReads;
    uint32_t worldUpRead = 0;
    uint32_t targetRead = 0;
    std::vector<uint32_t> targetReads;
    uint32_t effectorRead = 0;
    std::vector<RigExecWireAncestorRead> effectorAncestors;
    std::vector<uint32_t> poleReads;
    std::vector<std::vector<RigExecWireAncestorRead>> poleAncestors;
    bool recordAfter = true;
    bool recordEveryTarget = true;
    std::vector<std::vector<RigExecWireAncestorRead>> sourceAncestors;
    std::vector<RigExecWireAncestorRead> worldUpAncestors;
};

/// A constraint's per-frame array reads: the static shape only. The values
/// the prologue reads are per-frame inputs (slice 4), not constants.
struct RigExecWireConstraintArrays {
    uint32_t prim = 0;
    uint64_t sourceCount = 0;
    bool parentOffsets = false;
    bool readPole = false;
    uint64_t poleCount = 0;
};

/// A native Xformable constraint source's namespace topology.
struct RigExecWireNativeSource {
    uint32_t path = 0;
    std::vector<int32_t> ancestorSlots;
};

/// One contiguous provider-slot group the compose pass runs as one step.
struct RigExecWireComposeGroup {
    int32_t begin = 0;
    int32_t end = 0;
    std::vector<int32_t> parentSlots;
};

/// Which version of one provider frame a space switch reads
/// (RigExecBakedProgramImpl::SpaceSwitch::FrameVersion): `anchor`'s last
/// version (identity at -1), composed unswitched through `recompose`, top
/// down. An empty `recompose` reads the anchor as it stands.
struct RigExecWireFrameVersion {
    int32_t anchor = -1;
    std::vector<int32_t> recompose;
};

/// One RigExecSpaceSwitch: the switched provider slot, the slots of its
/// labelled sources (-1 is world), and the channels the resulting delta is
/// allowed to reach. `active` is the fractional selector, read per frame.
///
/// Memory only on RigExecWireDomainPose: these travel in the optional
/// SpaceSwitch section, so a binary baked before minor 1 loads with every
/// provider composed against its authored parent, exactly as before.
struct RigExecWireSpaceSwitch {
    int32_t slot = -1;
    std::vector<int32_t> sourceSlots;
    /// Parallel to sourceSlots, in RigExecRotationFilter order: which part
    /// of that source's rotation reaches the target. Empty means All.
    std::vector<uint8_t> filters;
    RigExecWireVec3d twistAxis{};
    /// rigExec:space -- the provider whose own rest->pose map carries the
    /// whole rig, or -1 for none. Encoded as a trailing block AFTER every
    /// switch's fixed fields, so a binary baked before it decodes with -1
    /// on every switch, which is the behaviour those files were baked
    /// from. See RigExecWireDecodeSpaceSwitches.
    int32_t spaceSlot = -1;
    /// The versions the compose reads: the namespace parent's, which
    /// `local` is divided back out of; each source's, parallel to
    /// sourceSlots ({-1, {}} at a world source); and spaceSlot's, for the
    /// carry ({-1, {}} when none is named). A second trailing block after
    /// the space slots: a binary without it decodes with every read at
    /// that slot's last version, which is what it was baked from.
    RigExecWireFrameVersion parentRead;
    std::vector<RigExecWireFrameVersion> sourceReads;
    RigExecWireFrameVersion spaceRead;
    RigExecWireInput active;
    bool affectTranslation[3] = {true, true, true};
    bool affectRotation[3] = {true, true, true};
    bool affectScale[3] = {true, true, true};
};

/// The DomainPose section: every table the pose-half steps index, plus the
/// publication inputs the epilogue reads.
struct RigExecWireDomainPose {
    std::vector<RigExecWireLadder> ladders;
    bool ladderVarying = false;
    std::vector<int32_t> ladderOverrides;
    std::vector<uint8_t> restChainVaries;
    std::vector<RigExecWirePoseInterpolator> poseInterpolators;
    std::vector<uint32_t> poseWeightPaths;
    std::vector<RigExecWireSolver> solvers;
    std::vector<int32_t> guideSolvers;
    std::vector<RigExecWireConstraint> constraints;
    std::vector<RigExecWireConstraintArrays> constraintArrays;
    std::vector<RigExecWireNativeSource> nativeSources;
    std::vector<RigExecWireWalkStep> walkSteps;
    std::vector<RigExecWireComposeGroup> composeGroups;
    std::vector<RigExecWireCommit> commits;
    /// Joint path -> ordered (solver path, element) stack, for fallback
    /// publication.
    std::vector<uint32_t> jointBindingJoints;
    std::vector<std::vector<uint32_t>> jointBindingSolvers;
    std::vector<std::vector<int32_t>> jointBindingElements;
    bool hasPropertyChains = false;
    bool phasedReads = false;
    bool publishWeightFields = true;
    /// Memory only: the SpaceSwitch section carries these (see bake.cpp).
    /// Sparse -- one entry per switched provider, in program order
    /// (resolution round, then discovery).
    std::vector<RigExecWireSpaceSwitch> spaceSwitches;
};

void RigExecWirePutInput(std::vector<uint8_t> *out,
                        const RigExecWireInput &input);
bool RigExecWireReadInput(RigExecWireReader *reader,
                          RigExecWireInput *input);

bool RigExecWireEncodeDomainPose(const RigExecWireDomainPose &pose,
                                 std::vector<uint8_t> *out);
bool RigExecWireDecodeDomainPose(RigExecWireReader *reader,
                                 RigExecWireDomainPose *pose,
                                 std::string *error);

/// One FkChain start provider: sparse, only solvers with start >= 0.
struct RigExecWireSolverStart {
    uint32_t solver = 0;
    int32_t start = -1;
    std::array<RigExecWireVec3d, 4> rest{};
    uint32_t read = 0;
};

bool RigExecWireEncodeSolverStarts(
    const std::vector<RigExecWireSolverStart> &starts,
    std::vector<uint8_t> *out);
bool RigExecWireDecodeSolverStarts(RigExecWireReader *reader,
                                   std::vector<RigExecWireSolverStart> *starts,
                                   std::string *error);

/// One pose interpolator's numeric dials and translation channel: sparse,
/// only interpolators that read one or the other.
struct RigExecWirePoseNumeric {
    uint32_t interpolator = 0;
    bool enableTranslation = false;
    std::vector<RigExecWireInput> values;
};

bool RigExecWireEncodePoseNumerics(
    const std::vector<RigExecWirePoseNumeric> &numerics,
    std::vector<uint8_t> *out);
bool RigExecWireDecodePoseNumerics(
    RigExecWireReader *reader,
    std::vector<RigExecWirePoseNumeric> *numerics, std::string *error);

/// False when a switch's sourceReads is not parallel to its sourceSlots.
bool RigExecWireEncodeSpaceSwitches(
    const std::vector<RigExecWireSpaceSwitch> &switches,
    std::vector<uint8_t> *out);
/// \p slotParent is the slot inventory's namespace parent per slot
/// (RigExecWireSlotMeta::parent). A section without the read-version
/// block gives each switch {slotParent[slot], {}} as its parent read, each
/// source and the space their own slot with no recompose; a switch slot
/// outside \p slotParent then fails the decode.
bool RigExecWireDecodeSpaceSwitches(
    RigExecWireReader *reader, const std::vector<int32_t> &slotParent,
    std::vector<RigExecWireSpaceSwitch> *switches, std::string *error);

}  // namespace rigExec

#endif  // RIGEXEC_BINARY_POSE_H
