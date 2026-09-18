//
// .rigexec DomainPose section: the pose-domain build tables.
//
// Ladders, pose interpolators (+ solved RBF tables), solvers, constraints,
// walk steps and commits: everything the pose-half steps index into. Bound
// inputs travel as tagged constants plus their read-route flags; the USD
// handles behind them (queries, attributes) do not -- per-frame value
// streams arrive with the InputTable section (M1 slice 4), which keys off
// the head paths and override indices recorded here.
//
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
    int32_t driverSlot = -1;
    int32_t parentSlot = -1;
    bool allowNegativeWeights = true;
    RigExecWireInput enabled;
    int32_t weightBegin = 0;
    int32_t weightEnd = 0;
    std::vector<int32_t> poseSlots;
    std::vector<int32_t> disabledSlots;
    RigExecWireRbf solver;
};

struct RigExecWireTwoBoneIkParams {
    double upperLength = 1;
    double lowerLength = 1;
    double stretch = 1;
    double softness = 0;
    double preferredBendRadians = 0;
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

struct RigExecWireSplineIkParams {
    double preserveVolume = 1;
    double midFollowWeight = 0.5;
    double roll = 0;
    double twist = 0;
    double minLengthRatio = 0;
    bool aimRootTangent = false;
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
struct RigExecWireConstraint {
    uint32_t path = 0;
    uint32_t type = 0;
    uint32_t weightObject = 0;
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

struct RigExecWireConstraintSource {
    RigExecWireFrame frame;
    double normalizedWeight = 1;
    RigExecWireVec3d translationOffset{};
    RigExecWireVec3d rotationOffsetDegrees{};
};

/// One slot above a native Xformable source, and the versions live where
/// the commit runs.
struct RigExecWireAncestorRead {
    int32_t slot = -1;
    uint32_t fin = 0;
    uint32_t base = 0;
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

}  // namespace rigExec

#endif  // RIGEXEC_BINARY_POSE_H
