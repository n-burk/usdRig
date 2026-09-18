//
// .rigexec InputTable section: the per-frame values the program consumed.
//
// Two halves. The static DIRECTORY assigns every bound varying input a uid
// in one documented traversal order (see RigExecBakeCapture in
// libs/rigExecBake/capture.h): ladders, interpolators, solvers,
// constraints, weight objects, then the avar bindings, each input in field
// order. The runtime allocates one slot per uid in the same order, so a
// uid IS the slot -- no lookup tables on the hot path.
//
// The dynamic half is one record per baked frame: the values the inputs
// resolved (sparse -- an input records only when its step ran, and the
// runtime holds the last value, which is exactly the program's cone
// semantics), plus the prologue's retained arrays for that frame. Inputs
// the directory skips (constant or unbound) provably read as their
// constant -- see RigExecBakedRead -- so the runtime never consults them.
//
#ifndef RIGEXEC_BINARY_INPUT_TABLE_H
#define RIGEXEC_BINARY_INPUT_TABLE_H

#include "rigExecBinary/geometry.h"

namespace rigExec {

/// One directory entry: the input's value tag, its override ordinal (-1
/// for none), and the head path override matching keys off (0 for none).
struct RigExecWireInputDirectoryEntry {
    RigExecWireInput::Tag tag = RigExecWireInput::Tag::Double;
    int32_t overrideIndex = -1;
    uint32_t head = 0;
};

/// A resolved scalar/matrix/token/landmark value, tagged like the inputs.
struct RigExecWireValue {
    RigExecWireInput::Tag tag = RigExecWireInput::Tag::Double;
    double f64 = 0;
    float f32 = 0;
    bool boolean = false;
    int32_t i32 = 0;
    RigExecWireMatrix4d matrix{};
    uint32_t token = 0;
    RigExecWireVec3d vec{};
};

/// A property chain's published value: float, double, Matrix4d or Vec3f,
/// the four types _EvaluatePropertyChains instantiates.
struct RigExecWirePropertyValue {
    enum class Tag : uint8_t {
        Float = 0,
        Double = 1,
        Matrix4d = 2,
        Vec3f = 3,
    };
    Tag tag = Tag::Float;
    float f32 = 0;
    double f64 = 0;
    RigExecWireMatrix4d matrix{};
    RigExecWireVec3f vec{};
};

/// One weight packet, as the WeightPacket steps left it.
struct RigExecWireWeightPacket {
    uint32_t representation = 0;
    uint32_t rangePolicy = 0;
    std::vector<float> values;
    std::vector<int32_t> indices;
    float defaultWeight = 0;
    bool valid = false;
};

/// One cache-refused sparse sample's layout, path-keyed by the
/// sample/mover paths the geometry section records (slice-3a design):
/// the runtime uses this when present and the epoch layout otherwise.
struct RigExecWireRefusedLayout {
    uint32_t mover = 0;
    uint32_t sample = 0;
    uint64_t pointCount = 0;
    bool valid = false;
    std::vector<RigExecWireVec3f> offsets;
    std::vector<int32_t> indices;
};

/// One stage-sourced value the shared assemblers consumed, by value: the
/// path-keyed half of the recorder (slice 5). Absent marks KNOWN-ABSENT
/// (the site read its fallback); every other tag carries the value.
struct RigExecWirePathValue {
    enum class Tag : uint8_t {
        Absent = 0,
        Bool = 1,
        Int = 2,
        Float = 3,
        Double = 4,
        Token = 5,
        Matrix4d = 6,
        Vec3d = 7,
        IntArray = 8,
        FloatArray = 9,
        Vec2fArray = 10,
        Vec3fArray = 11,
        Vec3i = 12,
        DoubleArray = 13,
    };
    Tag tag = Tag::Absent;
    bool boolean = false;
    int32_t i32 = 0;
    float f32 = 0;
    double f64 = 0;
    uint32_t token = 0;
    RigExecWireMatrix4d matrix{};
    RigExecWireVec3d vec{};
    std::vector<int32_t> ints;
    std::vector<float> floats;
    std::vector<RigExecWireVec2f> vec2s;
    std::vector<RigExecWireVec3f> vec3s;
    RigExecWireVec3i vec3i{{0, 0, 0}};
    std::vector<double> doubles;
};

/// One recorded stage read: the (attribute path, was-Default) key plus
/// the consumed value. forceFrame marks connection-following reads,
/// whose variance the drain cannot judge from the attribute.
struct RigExecWirePathRead {
    uint32_t path = 0;
    uint8_t wasDefault = 0;
    uint8_t forceFrame = 0;
    RigExecWirePathValue value;
};

/// Everything frame f consumed that is not computed from in-graph state.
struct RigExecWireFrameInputs {
    double frame = 0;
    /// Sparse resolved inputs: (uid, value) for the inputs read this
    /// frame, in ascending uid.
    std::vector<uint32_t> uids;
    std::vector<RigExecWireValue> values;
    /// Blend channel weights, activations and dense points as the last
    /// assembly consumed them: [chain][revision][channel]([sample]).
    /// Derived revisions ride beside their chain's, in the same nesting.
    /// Sparse samples carry no points here; their shape rides either the
    /// epoch layouts or refusedLayouts below.
    std::vector<std::vector<std::vector<float>>> blendWeights;
    std::vector<std::vector<std::vector<std::vector<float>>>>
        blendActivations;
    std::vector<std::vector<std::vector<std::vector<std::vector<
        RigExecWireVec3f>>>>>
        blendPoints;
    std::vector<std::vector<std::vector<float>>> derivedBlendWeights;
    std::vector<std::vector<std::vector<std::vector<float>>>>
        derivedBlendActivations;
    std::vector<std::vector<std::vector<std::vector<std::vector<
        RigExecWireVec3f>>>>>
        derivedBlendPoints;
    std::vector<RigExecWireRefusedLayout> refusedLayouts;
    /// inputs:defaultWeight per revision, as RevisionStatic read it.
    /// Main revisions only: derived ones never fill it.
    std::vector<std::vector<float>> revisionDefaultWeights;
    /// Current-phase weight packets per revision, for the movers whose
    /// weight object measures against the points as they stand.
    std::vector<std::vector<RigExecWireWeightPacket>> revisionPhasePackets;
    std::vector<std::vector<RigExecWireWeightPacket>> derivedPhasePackets;
    /// The adjuster ladder per revision, fresh where the have-flag
    /// says. Main revisions only: the publish visits no derived ones.
    std::vector<std::vector<RigExecWireMatrix4d>> revisionAdjusters;
    std::vector<std::vector<uint8_t>> revisionAdjusterHave;
    /// Live ribbon driver points per solver, in program order. Empty
    /// where the solver binds no varying driver.
    std::vector<std::vector<RigExecWireVec3f>> solverRibbonPoints;
    /// The resolved envelope per constraint, fresh where the have-flag
    /// says (a weight object on a non-point constraint).
    std::vector<float> constraintWeights;
    std::vector<uint8_t> constraintHaveWeight;
    /// Chain and derived bases, in chain order.
    std::vector<uint8_t> chainHaveBase;
    std::vector<std::vector<RigExecWireVec3f>> chainBases;
    std::vector<uint8_t> derivedHaveBase;
    std::vector<std::vector<RigExecWireVec3f>> derivedBases;
    std::vector<RigExecWireMatrix4d> xformBase;
    std::vector<RigExecWireFrame> nativeFrames;
    std::vector<uint32_t> propertyPaths;
    std::vector<RigExecWirePropertyValue> propertyValues;
    std::vector<std::vector<double>> arrayWeights;
    std::vector<std::vector<RigExecWireVec3d>> arrayTranslationOffsets;
    std::vector<std::vector<RigExecWireVec3d>> arrayRotationOffsets;
    std::vector<uint8_t> arrayOk;
    std::vector<std::vector<double>> arrayPoleWeights;
    std::vector<uint8_t> arrayPoleOk;
    std::vector<RigExecWireMatrix4d> deltaBaseMatrix;
    std::vector<uint8_t> deltaBaseOk;
    std::vector<RigExecWireWeightPacket> weightPackets;
    std::vector<uint32_t> currentPhaseWeights;
    /// Stage-sourced reads the assemblers consumed this frame, in
    /// (path, was-Default) order: the recorder path half, drained.
    std::vector<RigExecWirePathRead> pathReads;
};

/// The InputTable section: the static directory and override routing, then
/// one record per baked frame in bake order.
struct RigExecWireInputTable {
    std::vector<RigExecWireInputDirectoryEntry> directory;
    std::vector<uint32_t> overridablePaths;
    std::vector<std::vector<int32_t>> overridableIndices;
    std::vector<RigExecWireFrameInputs> frames;
};

bool RigExecWireEncodeInputTable(const RigExecWireInputTable &table,
                                 std::vector<uint8_t> *out);
bool RigExecWireDecodeInputTable(RigExecWireReader *reader,
                                 RigExecWireInputTable *table,
                                 std::string *error);

}  // namespace rigExec

#endif  // RIGEXEC_BINARY_INPUT_TABLE_H
