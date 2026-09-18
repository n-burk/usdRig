//
// .rigexec program sections: the wire encoding of the baked program's
// skeleton -- slot inventory, epoch constants, steps, clusters, cones.
//
// The wire structs here are plain data (no USD): the bake side converts the
// live RigExecBakedProgramImpl into them, and the M2 runtime interprets them
// directly. Encode and Decode are strict inverses -- Decode rejects trailing
// bytes, so an encoder/decoder skew fails loudly rather than reading stale
// fields -- and every enum is range-checked on the way in.
//
// Layout notes (D1): everything is little-endian; counts are u32; vectors
// are count-prefixed and dense; matrices are 16xf64 row-major; frames are
// 4xvec3d landmarks plus a flags u32. String references are u32 indices
// into the container's string table.
//
#ifndef RIGEXEC_BINARY_PROGRAM_H
#define RIGEXEC_BINARY_PROGRAM_H

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace rigExec {

// ---------------------------------------------------------------------------
// Wire primitives.
// ---------------------------------------------------------------------------

/// Appends little-endian scalars to a section payload.
void RigExecWirePutU8(std::vector<uint8_t> *out, uint8_t value);
void RigExecWirePutU32(std::vector<uint8_t> *out, uint32_t value);
void RigExecWirePutI32(std::vector<uint8_t> *out, int32_t value);
void RigExecWirePutU64(std::vector<uint8_t> *out, uint64_t value);
void RigExecWirePutF32(std::vector<uint8_t> *out, float value);
void RigExecWirePutF64(std::vector<uint8_t> *out, double value);

/// Bounds-checked section cursor. Every read past the end fails rather than
/// wrapping, and Exhausted reports trailing bytes.
class RigExecWireReader {
public:
    RigExecWireReader(const uint8_t *data, size_t size)
        : _data(data), _size(size)
    {
    }

    bool ReadU8(uint8_t *out);
    bool ReadU32(uint32_t *out);
    bool ReadI32(int32_t *out);
    bool ReadU64(uint64_t *out);
    bool ReadF32(float *out);
    bool ReadF64(double *out);
    bool Exhausted() const { return _at == _size; }

private:
    const uint8_t *_data = nullptr;
    size_t _size = 0;
    size_t _at = 0;
};

// ---------------------------------------------------------------------------
// Wire math (mirrors GfVec3d/GfVec3f/GfMatrix4d/RigExecPointFrame fieldwise).
// ---------------------------------------------------------------------------

using RigExecWireVec3d = std::array<double, 3>;
using RigExecWireVec3f = std::array<float, 3>;
using RigExecWireVec2f = std::array<float, 2>;
using RigExecWireVec3i = std::array<int32_t, 3>;
/// Row-major, m[r][c] at [r * 4 + c].
using RigExecWireMatrix4d = std::array<double, 16>;

struct RigExecWireFrame {
    std::array<RigExecWireVec3d, 4> points{};
    uint32_t flags = 0;
};

void RigExecWirePutVec3d(std::vector<uint8_t> *out,
                         const RigExecWireVec3d &value);
void RigExecWirePutVec3f(std::vector<uint8_t> *out,
                         const RigExecWireVec3f &value);
void RigExecWirePutVec2f(std::vector<uint8_t> *out,
                         const RigExecWireVec2f &value);
void RigExecWirePutMatrix4d(std::vector<uint8_t> *out,
                            const RigExecWireMatrix4d &value);
void RigExecWirePutFrame(std::vector<uint8_t> *out,
                         const RigExecWireFrame &value);
bool RigExecWireReadVec3d(RigExecWireReader *reader,
                          RigExecWireVec3d *value);
bool RigExecWireReadVec3f(RigExecWireReader *reader,
                          RigExecWireVec3f *value);
bool RigExecWireReadVec2f(RigExecWireReader *reader,
                          RigExecWireVec2f *value);
bool RigExecWireReadMatrix4d(RigExecWireReader *reader,
                             RigExecWireMatrix4d *value);
bool RigExecWireReadFrame(RigExecWireReader *reader,
                          RigExecWireFrame *value);

// ---------------------------------------------------------------------------
// Steps, clusters, cones (SlotMeta section companions).
// ---------------------------------------------------------------------------

/// Mirrors RigExecBakedSlotDomain, in the same order. The step graph's
/// domain ids are part of the format, so this enum is frozen: append only,
/// never reorder.
enum class RigExecWireSlotDomain : uint8_t {
    Avars = 0,
    PoseBase = 1,
    PoseFin = 2,
    PosedM = 3,
    FinalMatrix = 4,
    BaseMatrix = 5,
    Aggregate = 6,
    SolverPoints = 7,
    Candidates = 8,
    CommitTable = 9,
    CommitDelta = 10,
    CommitStaging = 11,
    ConstraintDelta = 12,
    PropertyResult = 13,
    ChainBase = 14,
    RevisionPacket = 15,
    RevisionTransforms = 16,
    RevisionOut = 17,
    RevisionDone = 18,
    ChainDirty = 19,
    ChainPoints = 20,
    DerivedOut = 21,
    WeightPacket = 22,
    WeightFrames = 23,
    PoseWeight = 24,
    Snapshots = 25,
};

/// Mirrors RigExecBakedStepKind, in the same order. Frozen like the domains.
enum class RigExecWireStepKind : uint8_t {
    ComposeSubtree = 0,
    Solve = 1,
    SolverCommit = 2,
    Constraint = 3,
    CommitDelta = 4,
    PropagateChunk = 5,
    CommitApply = 6,
    ProviderMatrix = 7,
    SnapshotFinals = 8,
    PoseInterpolator = 9,
    VolumePlacements = 10,
    WeightPacket = 11,
    InfluenceFold = 12,
    RevisionStatic = 13,
    RevisionChunk = 14,
    RevisionFuse = 15,
    ChainStatus = 16,
    Derived = 17,
};

/// Mirrors RigExecBakedSlotKind. Frozen.
enum class RigExecWireSlotKind : uint8_t {
    FirstFramePose = 0,
    XformDerived = 1,
};

/// One half-open run of slots of one domain.
struct RigExecWireSlotRange {
    RigExecWireSlotDomain domain = RigExecWireSlotDomain::Avars;
    uint32_t begin = 0;
    uint32_t end = 0;
};

/// One step: identity, declared ranges, edges, dirtiness, schedule. The
/// per-run outputs (diagnostics, counters, snapshots, bail, timestamps)
/// are NOT on the wire -- a step's output belongs to a run, not a file.
/// The label string reference rides along for reports and traces.
struct RigExecWireStep {
    RigExecWireStepKind kind = RigExecWireStepKind::ComposeSubtree;
    int32_t object = -1;
    int32_t part = -1;
    std::vector<RigExecWireSlotRange> reads;
    std::vector<RigExecWireSlotRange> writes;
    std::vector<int32_t> preds;
    std::vector<int32_t> succs;
    bool isSource = false;
    bool externalReads = false;
    bool varyingInputs = false;
    bool resolvedInputReads = false;
    std::vector<int32_t> overrideInputs;
    int32_t cluster = -1;
    int32_t level = 0;
    double sizeUnits = 0;
    double cost = 0;
    uint32_t maxDiagnostics = 0;
    uint32_t label = 0;
};

/// One cluster: members in increasing program index, edges, cost, level.
/// The run-timing fields are NOT on the wire.
struct RigExecWireCluster {
    std::vector<int32_t> members;
    std::vector<int32_t> preds;
    std::vector<int32_t> succs;
    double cost = 0;
    int32_t level = 0;
};

struct RigExecWireClustering {
    std::vector<RigExecWireCluster> clusters;
    std::vector<int32_t> clusterOf;
    double grainUs = 0;
    double serialCost = 0;
    double criticalPathCost = 0;
};

/// A bitset over clusters: the cluster count plus ceil(count/64) words.
struct RigExecWireClusterSet {
    uint32_t clusters = 0;
    std::vector<uint64_t> words;
};

/// Build-time cone closures and dirty-source lookup tables.
struct RigExecWireCones {
    std::vector<RigExecWireClusterSet> cone;
    RigExecWireClusterSet always;
    RigExecWireClusterSet poseClusters;
    std::vector<int32_t> avarCluster;
    std::vector<std::vector<int32_t>> chainBaseClusters;
    std::vector<std::vector<int32_t>> solverPointsClusters;
    std::vector<std::vector<int32_t>> revisionClusters;
    std::vector<int32_t> revisionStaticCluster;
    std::vector<std::vector<int32_t>> nativeSourceClusters;
    std::vector<std::vector<int32_t>> deltaBaseClusters;
    std::vector<std::vector<int32_t>> constraintArrayClusters;
    std::vector<int32_t> varyingSteps;
    std::vector<int32_t> overrideSteps;
};

bool RigExecWireEncodeSteps(const std::vector<RigExecWireStep> &steps,
                            std::vector<uint8_t> *out);
bool RigExecWireDecodeSteps(RigExecWireReader *reader,
                            std::vector<RigExecWireStep> *steps,
                            std::string *error);
bool RigExecWireEncodeClustering(const RigExecWireClustering &clustering,
                                 std::vector<uint8_t> *out);
bool RigExecWireDecodeClustering(RigExecWireReader *reader,
                                 RigExecWireClustering *clustering,
                                 std::string *error);
bool RigExecWireEncodeCones(const RigExecWireCones &cones,
                            std::vector<uint8_t> *out);
bool RigExecWireDecodeCones(RigExecWireReader *reader,
                            RigExecWireCones *cones, std::string *error);

// ---------------------------------------------------------------------------
// Slot inventory (SlotMeta section) and epoch constants (Constants section).
// ---------------------------------------------------------------------------

/// Dense provider slots in namespace DFS order, plus the publication
/// tables the epilogue reads. Paths are string-table references; the
/// SdfPath-keyed lookup maps are rebuilt by the reader and NOT on the wire.
struct RigExecWireSlotMeta {
    std::vector<uint32_t> paths;
    std::vector<RigExecWireSlotKind> slotKind;
    std::vector<int32_t> parent;
    std::vector<int32_t> propParent;
    std::vector<int32_t> xformSlots;
    std::vector<uint32_t> xformPaths;
    std::vector<int32_t> jointSlots;
    std::vector<uint32_t> jointPaths;
    std::vector<int32_t> controlSlots;
    std::vector<uint32_t> controlPaths;
    std::vector<uint32_t> solverArrayPaths;
    std::vector<int32_t> solverArrayElements;
    std::vector<int32_t> jointPublishOrder;
    std::vector<int32_t> controlPublishOrder;
    std::vector<int32_t> solverPublishOrder;
    bool jointPathsAscending = false;
    bool controlPathsAscending = false;
    bool solverArraysAscending = false;
    std::vector<uint8_t> needFinal;
    std::vector<uint8_t> needBase;
};

/// The Build-time answers the frame path reads: rest chain, default-space
/// ladder, rotation orders, composed avar constants. Per-run doubles
/// (last*, moved slots, disturbed flags) are NOT on the wire.
struct RigExecWireConstants {
    std::vector<RigExecWireMatrix4d> restM;
    std::vector<std::array<RigExecWireVec3d, 4>> restPts;
    std::vector<RigExecWireFrame> restFrames;
    std::vector<RigExecWireMatrix4d> selfD;
    std::vector<RigExecWireMatrix4d> parentDinv;
    std::vector<uint32_t> rotOrder;
    std::vector<RigExecWireMatrix4d> restRoundTrip;
    std::vector<RigExecWireMatrix4d> defaultRoundTrip;
    std::vector<uint8_t> posedAuthored;
    std::vector<RigExecWireMatrix4d> posedAuthoredM;
    std::vector<uint8_t> noScaleAvars;
    /// providers * 11 doubles, in the avar-name order.
    std::vector<double> avarConstants;
};

bool RigExecWireEncodeSlotMeta(const RigExecWireSlotMeta &meta,
                               std::vector<uint8_t> *out);
bool RigExecWireDecodeSlotMeta(RigExecWireReader *reader,
                               RigExecWireSlotMeta *meta,
                               std::string *error);
bool RigExecWireEncodeConstants(const RigExecWireConstants &constants,
                                std::vector<uint8_t> *out);
bool RigExecWireDecodeConstants(RigExecWireReader *reader,
                                RigExecWireConstants *constants,
                                std::string *error);

}  // namespace rigExec

#endif  // RIGEXEC_BINARY_PROGRAM_H
