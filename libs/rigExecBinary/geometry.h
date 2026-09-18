//
// .rigexec DomainGeometry section: chains, revisions, weight objects.
//
// The geometry-half build tables: revision bindings (+ read phases), chunk
// partitions, epoch layouts (skin topologies, blend-sample shapes), weight
// objects, falloff LUTs, delta descriptors and the dense revision index.
// Per-frame values -- packets, influence tables, envelopes, published
// points -- are NOT on the wire: the runtime re-derives them by running.
//
// Two deliberate deferrals, both documented where they land:
//   * Curvenet profile binds own a factorization, which has no by-value
//     form; M2 re-binds from the bind inputs. The weight side already
//     retains its inputs (bound*), the profile side needs retention state
//     (M1 slice 3b).
//   * Blend layouts and skin topologies the epoch cache refused vary per
//     frame; their streams arrive with the InputTable (slice 4), keyed by
//     the sample/mover paths recorded here.
//
#ifndef RIGEXEC_BINARY_GEOMETRY_H
#define RIGEXEC_BINARY_GEOMETRY_H

#include "rigExecBinary/pose.h"

namespace rigExec {

/// A resolved read phase: the kind plus the prim AtPrim names (0 else).
struct RigExecWireReadPhase {
    /// Base/Preceding/Final/AtPrim, in RigExecReadPhaseKind order.
    uint8_t kind = 0;
    uint32_t prim = 0;
};

struct RigExecWireBlendSampleBinding {
    uint32_t sample = 0;
    uint32_t points = 0;
    RigExecWireReadPhase phase;
    uint32_t blendShape = 0;
};

/// The revision's epoch-static binding: every provider path, in the roles
/// the assembler reads them under.
struct RigExecWireRevisionBinding {
    uint32_t moverPath = 0;
    uint32_t target = 0;
    uint32_t transform = 0;
    uint32_t transformSpace = 0;
    std::vector<uint32_t> influences;
    uint32_t weightObject = 0;
    uint32_t base = 0;
    uint32_t topologyCounts = 0;
    uint32_t topologyIndices = 0;
    uint32_t cagePoints = 0;
    uint32_t surfacePoints = 0;
    uint32_t bindCoords = 0;
    uint32_t driverCurvePoints = 0;
    uint32_t driverCurveOrder = 0;
    uint32_t driverCurveKnots = 0;
    int32_t driverTransformCount = 0;
    int32_t driverSpaceCount = 0;
    int32_t driverBaseTransformCount = 0;
    uint32_t driverFrames = 0;
    uint32_t widths = 0;
    uint32_t curvenet = 0;
    uint32_t curvenetPoints = 0;
    std::vector<uint32_t> blendInputs;
    std::vector<uint32_t> blendSampleInputs;
    std::vector<std::vector<RigExecWireBlendSampleBinding>> blendSamples;
    std::vector<uint32_t> phaseInputs;
    std::vector<RigExecWireReadPhase> phases;
    RigExecWireReadPhase transformPhase;
};

/// One blend channel's stage handles: which attribute to ask, in
/// accumulation order. Invalid handles read as the channel default, so the
/// validity rides along; the default VALUE is slice-4 business (it is a
/// code constant in the accumulation, not stage data).
struct RigExecWireBlendChannel {
    uint32_t weight = 0;
    bool weightValid = false;
    uint32_t weightPath = 0;
    int32_t poseWeight = -1;
    struct Sample {
        uint32_t samplePath = 0;
        uint32_t activation = 0;
        bool activationValid = false;
        uint32_t points = 0;
        bool pointsValid = false;
        uint32_t pointsPath = 0;
        RigExecWireReadPhase phase;
        uint32_t blendShape = 0;
        bool hasLayout = false;
        std::vector<RigExecWireVec3f> offsets;
        std::vector<int32_t> indices;
        uint64_t pointCount = 0;
        bool layoutValid = false;
    };
    std::vector<Sample> samples;
};

/// One contiguous vertex range and the influence positions its vertices
/// index. Per-run tables (transforms, rows, palette) are NOT on the wire.
struct RigExecWireChunk {
    int32_t begin = 0;
    int32_t end = 0;
    std::vector<int32_t> key;
};

/// The per-point influence layout of one skin mover, by value.
struct RigExecWireSkinTopology {
    bool hasTopology = false;
    std::vector<int32_t> indices;
    std::vector<float> weights;
    int32_t elementSize = 0;
    uint64_t pointCount = 0;
    uint64_t influenceCount = 0;
    bool validated = false;
};

/// One geometry revision's epoch decisions. Everything RevisionStatic,
/// InfluenceFold and the fuse write per run is NOT on the wire.
struct RigExecWireRevision {
    uint32_t moverPath = 0;
    uint32_t target = 0;
    uint32_t moverPrim = 0;
    /// Matrix..RecomputeExtent, in RigExecRevisionOp order.
    uint8_t op = 1;
    RigExecWireRevisionBinding binding;
    int32_t curvenetChain = -1;
    bool curvenetBindResolved = false;
    /// Whether the prologue's bind produced a binding (as opposed to a
    /// remembered failure): M2's re-bind must agree.
    bool hasCurvenetBind = false;
    /// The bind's retained inputs, for M2's re-bind.
    std::vector<RigExecWireVec3f> curvenetRestNet;
    std::vector<int32_t> curvenetSplineIndices;
    int32_t curvenetSamplesPerSpline = 5;
    uint32_t curvenetBasis = 0;
    std::vector<RigExecWireVec3f> curvenetMeshPoints;
    std::vector<int32_t> curvenetMeshCounts;
    std::vector<int32_t> curvenetMeshIndices;
    bool curvenetBindInputsHeld = false;
    std::vector<RigExecWireBlendChannel> blendChannels;
    std::vector<int32_t> influenceSlots;
    int32_t transformSlot = -1;
    int32_t transformSpaceSlot = -1;
    int32_t constraintDelta = -1;
    int32_t driverFramesSolver = -1;
    bool finalPhase = false;
    bool skinTopologyFixed = false;
    bool snapshotAfter = false;
    bool readsSnapshots = false;
    std::vector<RigExecWireMatrix4d> packetInfluences;
    std::vector<RigExecWireChunk> chunks;
    int32_t chunkBase = 0;
    RigExecWireSkinTopology partitionTopology;
    int32_t partitionElementSize = 0;
    uint64_t partitionIndexCount = 0;
    uint64_t partitionPointCount = 0;
    bool chunked = false;
    uint64_t partitionCandidates = 0;
    int32_t partitionReadyMin = 0;
    int32_t partitionReadyMax = 0;
    int32_t weightObject = -1;
    bool weightOperationDomain = false;
    uint32_t weightFieldTarget = 0;
    bool weightCurrentPhase = false;
    RigExecWireSkinTopology topology;
    bool topologyResolved = false;
};

struct RigExecWireDerived {
    uint32_t target = 0;
    RigExecWireRevision revision;
};

struct RigExecWireChain {
    uint32_t target = 0;
    std::vector<RigExecWireRevision> revisions;
    std::vector<RigExecWireDerived> derived;
};

/// One weight object: the composed formula, by value. Packets are per-frame
/// and NOT on the wire; the curvenet binding is re-cut from the retained
/// bound* inputs.
struct RigExecWireWeightObject {
    uint32_t path = 0;
    uint32_t type = 0;
    uint32_t representation = 0;
    uint32_t rangePolicy = 0;
    std::vector<float> values;
    std::vector<int32_t> indices;
    RigExecWireInput defaultWeight;
    int32_t base = -1;
    std::vector<int32_t> inputs;
    RigExecWireInput driver;
    RigExecWireInput scale;
    RigExecWireInput bias;
    uint32_t combineMode = 0;
    RigExecWireInput strength;
    RigExecWireInput invert;
    std::vector<uint32_t> combineTargetPoints;
    std::vector<uint8_t> combineTargetValid;
    uint64_t costElements = 1;
    int32_t providerSlot = -1;
    RigExecWireInput falloffMin;
    RigExecWireInput falloffMax;
    RigExecWireInput scaleX;
    RigExecWireInput scaleY;
    RigExecWireInput scaleZ;
    RigExecWireInput extentU;
    RigExecWireInput extentV;
    uint32_t planeAxis = 0;
    uint32_t planeBounds = 0;
    std::vector<uint32_t> targetPoints;
    std::vector<uint8_t> targetValid;
    std::vector<uint32_t> samplePoints;
    std::vector<uint8_t> sampleValid;
    std::vector<uint32_t> curvePoints;
    std::vector<uint8_t> curveValid;
    std::vector<float> falloffCurve;
    std::vector<uint32_t> curvenetMeshPoints;
    std::vector<uint8_t> curvenetMeshValid;
    std::vector<uint32_t> curvenetPoints;
    std::vector<uint8_t> curvenetPointsValid;
    std::vector<uint32_t> curvenetCounts;
    std::vector<uint8_t> curvenetCountsValid;
    std::vector<uint32_t> curvenetIndices;
    std::vector<uint8_t> curvenetIndicesValid;
    std::vector<uint32_t> curvenetSplines;
    std::vector<uint8_t> curvenetSplinesValid;
    uint32_t curvenetWeights = 0;
    bool curvenetWeightsValid = false;
    uint32_t curvenetAutoSmooth = 0;
    bool curvenetAutoSmoothValid = false;
    uint32_t curvenetBasis = 0;
    RigExecWireInput curvenetSamples;
    RigExecWireInput curvenetUnreached;
    std::vector<RigExecWireVec3f> boundMesh;
    std::vector<RigExecWireVec3f> boundNet;
    std::vector<int32_t> boundCounts;
    std::vector<int32_t> boundIndices;
    std::vector<int32_t> boundSplines;
    std::vector<int32_t> boundSmooth;
    int32_t boundSamples = -1;
    bool bound = false;
};

/// The DomainGeometry section.
struct RigExecWireDomainGeometry {
    std::vector<RigExecWireChain> chains;
    std::vector<std::pair<int32_t, int32_t>> revisionIndex;
    std::vector<std::pair<int32_t, int32_t>> derivedIndex;
    std::vector<int32_t> chainRevisionBegin;
    std::vector<int32_t> chainRevisionEnd;
    std::vector<int32_t> revisionChunkBase;
    std::vector<int32_t> revisionChunkCount;
    std::vector<int32_t> chainChunkBegin;
    std::vector<int32_t> chainChunkEnd;
    std::vector<RigExecWireWeightObject> weightObjects;
    std::vector<uint32_t> falloffPaths;
    std::vector<std::vector<float>> falloffLuts;
    std::vector<uint32_t> currentPhaseWeights;
    std::vector<uint32_t> deltaBasePaths;
};

bool RigExecWireEncodeDomainGeometry(const RigExecWireDomainGeometry &geometry,
                                     std::vector<uint8_t> *out);
bool RigExecWireDecodeDomainGeometry(RigExecWireReader *reader,
                                     RigExecWireDomainGeometry *geometry,
                                     std::string *error);

}  // namespace rigExec

#endif  // RIGEXEC_BINARY_GEOMETRY_H
