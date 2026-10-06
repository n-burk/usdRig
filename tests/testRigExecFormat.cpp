// testRigExecFormat: the single-FlatBuffer .rigexec format on hand-built
// files. Write -> Open -> Write is byte-identical and keeps every stored bit
// (signed zeros, NaN payloads, denormals) of every floating-point field
// kind, value, pool and struct; Open refuses short buffers, the old
// container, a wrong file identifier, buffers the verifier rejects and
// other format versions, and buffers whose objects are shared or overlap,
// takes bytes at any address and keeps none of them, refuses every
// truncated tail and the named defects, and survives every single-byte
// flip, vtable flips and many random multi-byte corruptions; writing is
// deterministic; the validator refuses a smoke set of rule violations, a
// step or cluster graph playback could not walk, the retired snapshot
// step and domain, space switches out of resolution order, VolumePlacements
// steps that do not place one volume each, a failed plugin assembly
// holding frame bytes, and a presentation without its identifier, orders
// listed inputs as their composed texts, and handles a path tree 100000
// prims deep; step labels match the program's; sparse skin topologies
// encode losslessly and expand back to the layout bit for bit, every layout
// the sparse form cannot hold is stored raw and verbatim, and the validated
// flag holds to the evaluator's rules in both directions, in either form;
// chunk tables hold to the shape the bake cuts; array inputs (slots of each
// array tag, their pool and layout defaults, the reads bound to them, and
// the chain base, layout, painted and oracle slots) round-trip bit for bit,
// and each rule of theirs refuses its violation with its exact message.
// USD-free, like the format.
#include "rigExecBinary/format.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <initializer_list>
#include <memory>
#include <string>
#include <utility>
#include <vector>

using namespace rigExec;

namespace {

int _failures = 0;
std::string _context;

#define CHECK(cond)                                                         \
    do {                                                                    \
        if (!(cond)) {                                                      \
            ++_failures;                                                    \
            std::printf("FAIL %s:%d [%s]: %s\n", __FILE__, __LINE__,         \
                        _context.c_str(), #cond);                           \
        }                                                                   \
    } while (0)

using fb::InputTag;
using fb::PathKind;
using fb::ReadMode;
using fb::RigExecWireFile;
using fb::RigExecWireInput;

// ------------------------------------------------------------ bit helpers

uint64_t
_Bits(double value)
{
    uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof bits);
    return bits;
}

uint32_t
_Bits(float value)
{
    uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof bits);
    return bits;
}

double
_D(uint64_t bits)
{
    double value = 0;
    std::memcpy(&value, &bits, sizeof value);
    return value;
}

float
_F(uint32_t bits)
{
    float value = 0;
    std::memcpy(&value, &bits, sizeof value);
    return value;
}

/// -0.0, a quiet NaN with a payload, a negative signaling NaN, the
/// smallest denormal, the largest negative denormal, -inf.
const uint64_t _specialD[] = {0x8000000000000000ull, 0x7ff80000deadbeefull,
                              0xfff0000000000123ull, 0x0000000000000001ull,
                              0x800fffffffffffffull, 0xfff0000000000000ull};
const uint32_t _specialF[] = {0x80000000u, 0x7fc12345u, 0xff800001u,
                              0x00000001u, 0x807fffffu, 0xff800000u};
constexpr size_t _specialCount = 6;

double
_SD(size_t k)
{
    return _D(_specialD[k % _specialCount]);
}

float
_SF(size_t k)
{
    return _F(_specialF[k % _specialCount]);
}

RigExecWireVec3d
_V3d(size_t k)
{
    return {{_SD(k), _SD(k + 1), _SD(k + 2)}};
}

RigExecWireVec3f
_V3f(size_t k)
{
    return {{_SF(k), _SF(k + 1), _SF(k + 2)}};
}

RigExecWireVec2f
_V2f(size_t k)
{
    return {{_SF(k), _SF(k + 1)}};
}

RigExecWireMatrix4d
_M(size_t k)
{
    RigExecWireMatrix4d m;
    for (size_t i = 0; i < 16; ++i) {
        m[i] = _SD(k + i);
    }
    return m;
}

RigExecWireLandmarks
_L(size_t k)
{
    return {{_V3d(k), _V3d(k + 1), _V3d(k + 2), _V3d(k + 3)}};
}

RigExecWireFrame
_Frame(size_t k)
{
    RigExecWireFrame frame;
    frame.points = _L(k);
    frame.flags = uint32_t(0x80000001u + k);
    return frame;
}

std::vector<double>
_Ds(size_t count, size_t k = 0)
{
    std::vector<double> out;
    for (size_t i = 0; i < count; ++i) {
        out.push_back(_SD(k + i));
    }
    return out;
}

std::vector<float>
_Fs(size_t count, size_t k = 0)
{
    std::vector<float> out;
    for (size_t i = 0; i < count; ++i) {
        out.push_back(_SF(k + i));
    }
    return out;
}

// --------------------------------------------------------- bitwise compare

bool
_Same(double a, double b)
{
    return _Bits(a) == _Bits(b);
}

bool
_Same(float a, float b)
{
    return _Bits(a) == _Bits(b);
}

template <class T, size_t N>
bool
_Same(const std::array<T, N> &a, const std::array<T, N> &b)
{
    for (size_t i = 0; i < N; ++i) {
        if (!_Same(a[i], b[i])) {
            return false;
        }
    }
    return true;
}

bool
_Same(const RigExecWireFrame &a, const RigExecWireFrame &b)
{
    return _Same(a.points, b.points) && a.flags == b.flags;
}

template <class T>
bool
_Same(const std::vector<T> &a, const std::vector<T> &b)
{
    if (a.size() != b.size()) {
        return false;
    }
    for (size_t i = 0; i < a.size(); ++i) {
        if (!_Same(a[i], b[i])) {
            return false;
        }
    }
    return true;
}

// ------------------------------------------------------------- file parts

// Value ids of the rich file, one per tag plus the specials.
enum : uint32_t {
    _vZero = 0,
    _vNegZero = 1,
    _vNaN = 2,
    _vSNaN = 3,
    _vDenormal = 4,
    _vFloatNegZero = 5,
    _vFloatNaN = 6,
    _vFloatDenormal = 7,
    _vBool = 8,
    _vInt = 9,
    _vToken = 10,
    _vMatrix = 11,
    _vVec3d = 12,
    _vVec3f = 13,
    _vCount = 14,
};

uint32_t
_ConstantFor(InputTag tag)
{
    switch (tag) {
    case InputTag::Double:
        return _vNegZero;
    case InputTag::Float:
        return _vFloatNaN;
    case InputTag::Bool:
        return _vBool;
    case InputTag::Int:
        return _vInt;
    case InputTag::Token:
        return _vToken;
    case InputTag::Matrix4d:
        return _vMatrix;
    case InputTag::Vec3d:
        return _vVec3d;
    default:
        return _vVec3f;
    }
}

/// A read of \p tag with that tag's constant: Baked and constant unless
/// told otherwise.
RigExecWireInput
_InValue(InputTag tag, ReadMode mode = ReadMode::Baked,
         std::vector<uint32_t> walk = {})
{
    RigExecWireInput input;
    input.tag = tag;
    input.mode = mode;
    input.constant = _ConstantFor(tag);
    input.walk = std::move(walk);
    return input;
}

std::unique_ptr<RigExecWireInput>
_In(InputTag tag, ReadMode mode = ReadMode::Baked,
    std::vector<uint32_t> walk = {})
{
    return std::make_unique<RigExecWireInput>(
        _InValue(tag, mode, std::move(walk)));
}

/// The smallest file the validator accepts: no slots, no steps, the
/// required tables present and empty.
RigExecWireFile
_MinimalFile()
{
    RigExecWireFile f;
    f.formatVersion = RigExecFormatVersion;
    f.names = {"", "Rig"};
    f.paths = {fb::PathNode(0, 0, PathKind::None),
               fb::PathNode(0, 1, PathKind::Prim)};
    f.rig = 1;
    f.values.resize(1);
    f.intArrays.resize(1);
    f.floatArrays.resize(1);
    f.doubleArrays.resize(1);
    f.vec2fArrays.resize(1);
    f.vec3fArrays.resize(1);
    f.slotMeta = std::make_unique<fb::RigExecWireSlotMeta>();
    f.constants = std::make_unique<fb::RigExecWireConstants>();
    f.clustering = std::make_unique<fb::RigExecWireClustering>();
    f.cones = std::make_unique<fb::RigExecWireCones>();
    f.cones->always = std::make_unique<fb::RigExecWireClusterSet>();
    f.cones->poseClusters = std::make_unique<fb::RigExecWireClusterSet>();
    f.pose = std::make_unique<fb::RigExecWireDomainPose>();
    f.geometry = std::make_unique<fb::RigExecWireDomainGeometry>();
    return f;
}

// Path ids of the rich file.
enum : uint32_t {
    _pRig = 1,
    _pCtl = 2,
    _pCtlTx = 3,
    _pTokenXyz = 4,
    _pWeight = 5,
    _pMesh = 6,
    _pMeshPoints = 7,
    _pCtlW = 8,
    _pMover = 9,
    _pMoverDw = 10,
    _pTokenStatic = 11,
    _pTokenPlugin = 12,
};

std::unique_ptr<fb::RigExecWireRevisionBinding>
_Binding()
{
    auto binding = std::make_unique<fb::RigExecWireRevisionBinding>();
    binding->moverPath = _pMover;
    binding->target = _pMeshPoints;
    binding->transformPhase.kind = 1;
    binding->transformPhase.prim = _pMesh;
    return binding;
}

/// A presentation with one control, finished with \p identifier (none when
/// null). \p references > 1 makes its controls vector name the same control
/// table that many times, which Write never does.
std::vector<uint8_t>
_PresentationBytes(const char *identifier = fb::PresentationIdentifier(),
                   size_t references = 1)
{
    flatbuffers::FlatBufferBuilder builder;
    const auto control = fb::CreatePresentationControl(
        builder, builder.CreateString("Move.tx"),
        builder.CreateString("/Rig/Ctl.tx"), fb::PresentationUnit::Degrees);
    const std::vector<flatbuffers::Offset<fb::PresentationControl>> controls(
        references, control);
    const auto presentation = fb::CreatePresentation(
        builder, 1, builder.CreateVector(controls), 0,
        builder.CreateString("{}"));
    builder.Finish(presentation, identifier);
    return std::vector<uint8_t>(builder.GetBufferPointer(),
                                builder.GetBufferPointer() +
                                    builder.GetSize());
}

/// One of everything: a slot, a step and a cluster, every pose and
/// geometry table, a property chain with a phased consumer, path reads of
/// both forms, a presentation. Every floating-point field, value, pool and
/// struct holds signed zeros, NaN payloads or denormals.
RigExecWireFile
_RichFileBody()
{
    RigExecWireFile f = _MinimalFile();
    f.bakeTime = _SD(0);
    f.names = {"",  "Rig",   "Ctl",  "tx", "XYZ",  "W",    "Mesh",
               "points", "w", "Mover", "dw", "StaticWeight", "Plugin"};
    f.paths = {fb::PathNode(0, 0, PathKind::None),
               fb::PathNode(0, 1, PathKind::Prim),          // /Rig
               fb::PathNode(_pRig, 2, PathKind::Prim),      // /Rig/Ctl
               fb::PathNode(_pCtl, 3, PathKind::Property),  // /Rig/Ctl.tx
               fb::PathNode(0, 4, PathKind::Token),         // XYZ
               fb::PathNode(_pRig, 5, PathKind::Prim),      // /Rig/W
               fb::PathNode(_pRig, 6, PathKind::Prim),      // /Rig/Mesh
               fb::PathNode(_pMesh, 7, PathKind::Property),  // .points
               fb::PathNode(_pCtl, 8, PathKind::Property),  // /Rig/Ctl.w
               fb::PathNode(_pRig, 9, PathKind::Prim),      // /Rig/Mover
               fb::PathNode(_pMover, 10, PathKind::Property),  // .dw
               fb::PathNode(0, 11, PathKind::Token),        // StaticWeight
               fb::PathNode(0, 12, PathKind::Token)};       // Plugin

    // Values: one per tag, and the specials.
    f.values.resize(_vCount);
    const auto scalar = [&](uint32_t id, InputTag tag, uint64_t bits) {
        f.values[id].tag = tag;
        f.values[id].bits = bits;
    };
    scalar(_vNegZero, InputTag::Double, _specialD[0]);
    scalar(_vNaN, InputTag::Double, _specialD[1]);
    scalar(_vSNaN, InputTag::Double, _specialD[2]);
    scalar(_vDenormal, InputTag::Double, _specialD[3]);
    scalar(_vFloatNegZero, InputTag::Float, _specialF[0]);
    scalar(_vFloatNaN, InputTag::Float, _specialF[1]);
    scalar(_vFloatDenormal, InputTag::Float, _specialF[3]);
    scalar(_vBool, InputTag::Bool, 1);
    scalar(_vInt, InputTag::Int, 0xffffffffull);
    scalar(_vToken, InputTag::Token, _pTokenXyz);
    f.values[_vMatrix].tag = InputTag::Matrix4d;
    f.values[_vMatrix].matrix = std::make_unique<RigExecWireMatrix4d>(_M(1));
    f.values[_vVec3d].tag = InputTag::Vec3d;
    f.values[_vVec3d].vec3d = std::make_unique<RigExecWireVec3d>(_V3d(2));
    f.values[_vVec3f].tag = InputTag::Vec3f;
    f.values[_vVec3f].vec3f = std::make_unique<RigExecWireVec3f>(_V3f(3));

    // Pools, each with its empty [0].
    f.intArrays.resize(2);
    f.intArrays[1].v = {-1, 0, 2147483647};
    f.floatArrays.resize(2);
    f.floatArrays[1].v = _Fs(6);
    f.doubleArrays.resize(2);
    f.doubleArrays[1].v = _Ds(6);
    f.vec2fArrays.resize(2);
    f.vec2fArrays[1].v = {_V2f(0), _V2f(3)};
    f.vec3fArrays.resize(2);
    f.vec3fArrays[1].v = {_V3f(0), _V3f(4)};

    // The input list: three listed slots, sorted by path text.
    const uint8_t listed = uint8_t(fb::InputSlotFlags::Listed);
    const uint8_t all = uint8_t(fb::InputSlotFlags::ANY);
    f.inputs = {fb::InputSlot(_pCtlTx, _vNegZero, -1, -1, InputTag::Double,
                              all),
                fb::InputSlot(_pCtlW, _vFloatNaN, -1, 0, InputTag::Float,
                              listed),
                fb::InputSlot(_pMoverDw, _vFloatDenormal, 0, -1,
                              InputTag::Float, listed)};
    f.listedInputs = 3;

    // One provider slot.
    fb::RigExecWireSlotMeta &meta = *f.slotMeta;
    meta.paths = {_pCtl};
    meta.slotKind = {fb::SlotKind::FirstFramePose};
    meta.parent = {-1};
    meta.propParent = {-1};
    meta.needFinal = {1};
    meta.needBase = {0};

    fb::RigExecWireConstants &c = *f.constants;
    c.restM = {_M(0)};
    c.restPts = {_L(1)};
    c.restFrames = {_Frame(2)};
    c.selfD = {_M(3)};
    c.parentDinv = {_M(4)};
    c.rotOrder = {_pTokenXyz};
    c.restRoundTrip = {_M(5)};
    c.defaultRoundTrip = {_M(0)};
    c.posedAuthored = {1};
    c.posedAuthoredM = {_M(1)};
    c.noScaleAvars = {0};
    c.avarConstants = _Ds(11);
    c.rotationSign = {5};

    // One step in one cluster.
    fb::RigExecWireStep step;
    step.kind = fb::StepKind::ComposeSubtree;
    step.object = 0;
    step.cluster = 0;
    step.reads = {fb::SlotRange(fb::SlotDomain::Avars, 0, 11)};
    step.writes = {fb::SlotRange(fb::SlotDomain::PoseFin, 0, 1)};
    step.overrideInputs = {0};
    step.sizeUnits = _SD(1);
    step.cost = _SD(2);
    f.steps = {step};
    fb::RigExecWireCluster cluster;
    cluster.members = {0};
    cluster.cost = _SD(3);
    f.clustering->clusters = {cluster};
    f.clustering->clusterOf = {0};
    f.clustering->grainUs = _SD(4);
    f.clustering->serialCost = _SD(5);
    f.clustering->criticalPathCost = _SD(0);
    fb::RigExecWireClusterSet one;
    one.clusters = 1;
    one.words = {1};
    f.cones->cone = {one};
    f.cones->always->clusters = 1;
    f.cones->always->words = {0};
    f.cones->poseClusters->clusters = 1;
    f.cones->poseClusters->words = {1};
    f.cones->avarCluster = {0};
    f.cones->chainBaseClusters.resize(1);
    f.cones->chainBaseClusters[0].v = {0};
    f.cones->revisionClusters.resize(1);
    f.cones->revisionStaticCluster = {0};
    f.cones->varyingSteps = {0};

    // Pose domain.
    fb::RigExecWireDomainPose &pose = *f.pose;
    pose.overrideCount = 1;
    fb::RigExecWireLadder ladder;
    ladder.restSpace = _In(InputTag::Matrix4d);
    ladder.defaultSpace = _In(InputTag::Matrix4d);
    ladder.posedSpace = _In(InputTag::Matrix4d);
    for (size_t k = 0; k < 6; ++k) {
        ladder.restAvars.push_back(_InValue(InputTag::Double));
        ladder.defaultAvars.push_back(_InValue(InputTag::Double));
    }
    // The one override number: a varying avar read pinned to its slot.
    ladder.restAvars[0].walk = {0};
    ladder.restAvars[0].flags = uint8_t(fb::InputReadFlags::Varying);
    ladder.restAvars[0].selected = 0;
    ladder.restAvars[0].overrideIndex = 0;
    ladder.rotationOrder = _In(InputTag::Token);
    pose.ladders.push_back(std::move(ladder));
    pose.ladderOverrides = {0};
    pose.restChainVaries = {1};

    fb::RigExecWirePoseInterpolator interp;
    interp.path = _pCtl;
    interp.enabled = _In(InputTag::Bool);
    interp.valueInputs.push_back(_InValue(InputTag::Double));
    interp.solver = std::make_unique<fb::RigExecWireRbf>();
    interp.solver->poses = {_V3d(0), _V3d(1)};
    interp.solver->translations = {_V3d(2)};
    interp.solver->poseTypes = {0, 2};
    interp.solver->twistAxis = _V3d(3);
    interp.solver->radius = _SD(1);
    interp.solver->translationRadius = _SD(2);
    interp.solver->radii = _Ds(2, 3);
    interp.solver->translationRadii = _Ds(2, 4);
    interp.solver->regularization = _SD(5);
    interp.solver->weights.resize(2);
    interp.solver->weights[0].v = _Ds(2, 0);
    interp.solver->weights[1].v = _Ds(2, 2);
    pose.poseInterpolators.push_back(std::move(interp));

    fb::RigExecWireSolver solver;
    solver.path = _pCtl;
    solver.type = _pTokenXyz;
    solver.jointRests = {_L(0)};
    solver.splineRestWeights = _Ds(3);
    solver.controlRests = {_L(1)};
    solver.startRest = _L(2);
    solver.ikRests = {_L(3), _L(4), _L(5)};
    solver.ikParams = {_SD(0), _SD(1), _SD(2), _SD(3), _SD(4)};
    solver.bend = _In(InputTag::Double);
    solver.upperOffset = _In(InputTag::Double);
    solver.lowerOffset = _In(InputTag::Double);
    solver.stretch = _In(InputTag::Float);
    solver.softness = _In(InputTag::Float);
    solver.upperLengthBase = _SD(1);
    solver.lowerLengthBase = _SD(2);
    solver.ikSpace = _In(InputTag::Matrix4d);
    solver.spaceSlot = 0;
    solver.spaceRest = _L(0);
    solver.blendWeight = _In(InputTag::Float);
    solver.splineRest = std::make_unique<fb::RigExecWireSplineIkRest>();
    solver.splineRest->cvs = _L(1);
    solver.splineRest->rootControl = _Frame(0);
    solver.splineRest->midControl = _Frame(1);
    solver.splineRest->endControl = _Frame(2);
    solver.splineRest->joints = {_Frame(3)};
    solver.splineRest->segmentLengths = _Ds(2, 4);
    solver.splineRest->restArcLength = _SD(3);
    solver.splineRest->volumeWeights = _Ds(2, 5);
    solver.splineParams.preserveVolume = _SD(0);
    solver.splineParams.midFollowWeight = _SD(1);
    solver.splineParams.roll = _SD(2);
    solver.splineParams.twist = _SD(3);
    solver.splineParams.minLengthRatio = _SD(4);
    solver.splineParams.aimRootTangent = true;
    solver.splineJointRests = {_L(2)};
    solver.preserveVolume = _In(InputTag::Double);
    solver.midFollowWeight = _In(InputTag::Double);
    solver.roll = _In(InputTag::Double);
    solver.twist = _In(InputTag::Double);
    solver.minLengthRatio = _In(InputTag::Double);
    solver.twistStartRest = _L(3);
    solver.twistEndRest = _L(4);
    solver.twistWeights = _Ds(2, 1);
    solver.twistTurns = _In(InputTag::Double);
    solver.ribbonRestPoints = {_V3f(0)};
    solver.ribbonConstantPoints = {_V3f(1), _V3f(2)};
    solver.ribbonSampleCount = _In(InputTag::Int);
    solver.outputs = {{0, 1}};
    pose.solvers.push_back(std::move(solver));

    fb::RigExecWireConstraint constraint;
    constraint.path = _pCtl;
    constraint.type = _pTokenXyz;
    constraint.target = 0;
    constraint.enabled = _In(InputTag::Bool);
    constraint.defaultWeight = _In(InputTag::Float);
    constraint.offset = _In(InputTag::Vec3d);
    for (auto *bit : {&constraint.affectX, &constraint.affectY,
                      &constraint.affectZ, &constraint.tX, &constraint.tY,
                      &constraint.tZ, &constraint.rX, &constraint.rY,
                      &constraint.rZ, &constraint.sX, &constraint.sY,
                      &constraint.sZ}) {
        *bit = _In(InputTag::Bool);
    }
    constraint.aimVector = _In(InputTag::Vec3d);
    constraint.upVector = _In(InputTag::Vec3d);
    constraint.rotationOffset = _In(InputTag::Vec3d);
    constraint.worldUpVector = _In(InputTag::Vec3d);
    constraint.aimAxisFallback = _V3d(1);
    constraint.sceneUp = _V3d(4);
    constraint.flags = uint8_t(fb::ConstraintFlags::ANY);
    constraint.arrays = 0;
    constraint.poleVector = _In(InputTag::Vec3d);
    constraint.twistDegrees = _In(InputTag::Double);
    pose.constraints.push_back(std::move(constraint));

    fb::RigExecWireConstraintArrays arrays;
    arrays.prim = _pCtl;
    arrays.sourceCount = 2;
    arrays.parentOffsets = true;
    arrays.readPole = true;
    arrays.poleCount = 1;
    arrays.weights = _Ds(2);
    arrays.translationOffsets = {_V3d(0), _V3d(1)};
    arrays.rotationOffsets = {_V3d(2), _V3d(3)};
    arrays.poleWeights = _Ds(1, 5);
    arrays.diagnostics = {"array note"};
    arrays.poleDiagnostics = {"pole note"};
    pose.constraintArrays.push_back(std::move(arrays));

    fb::RigExecWireNativeSource native;
    native.path = _pMesh;
    native.ancestorSlots = {0};
    native.frame = _Frame(4);
    native.ok = true;
    pose.nativeSources.push_back(std::move(native));

    fb::RigExecWireCommit commit;
    commit.moverPath = _pCtl;
    commit.slots = {0};
    commit.slotWrites = {1};
    RigExecWireConstraintSource source;
    source.frame = _Frame(5);
    source.normalizedWeight = _SD(1);
    source.translationOffset = _V3d(2);
    source.rotationOffsetDegrees = _V3d(3);
    commit.sources = {source};
    pose.commits.push_back(std::move(commit));

    fb::RigExecWireSpaceSwitch sw;
    sw.slot = 0;
    sw.sourceSlots = {-1};
    sw.filters = {2};
    sw.twistAxis = _V3d(5);
    // A recomposed parent version, a world source, and a recomposed space:
    // this switch switches the one slot, so no read of it may anchor on
    // that slot (TestSwitchOrder covers anchored reads).
    sw.spaceSlot = 0;
    sw.parentRead = std::make_unique<fb::RigExecWireFrameVersion>();
    sw.parentRead->recompose = {0};
    sw.sourceReads.resize(1);
    sw.spaceRead = std::make_unique<fb::RigExecWireFrameVersion>();
    sw.spaceRead->recompose = {0};
    sw.active = _In(InputTag::Double);
    sw.affectTranslation = {{true, false, true}};
    sw.affectRotation = {{false, true, false}};
    sw.affectScale = {{true, true, false}};
    pose.spaceSwitches.push_back(std::move(sw));

    fb::RigExecWireAvarBinding avar;
    avar.flat = 0;
    avar.read = _In(InputTag::Double);
    pose.avarBindings.push_back(std::move(avar));
    pose.hasPropertyChains = true;

    // Geometry domain: one chain, one revision, one derived target.
    fb::RigExecWireDomainGeometry &g = *f.geometry;
    fb::RigExecWireRevision revision;
    revision.moverPath = _pMover;
    revision.target = _pMeshPoints;
    revision.moverPrim = _pMover;
    revision.op = 0;
    revision.binding = _Binding();
    fb::RigExecWireBlendChannel channel;
    channel.weight = _pCtlW;
    channel.weightValid = true;
    channel.weightRead = _In(InputTag::Float, ReadMode::Resolved, {1});
    fb::RigExecWireBlendSample sample;
    sample.samplePath = _pMesh;
    sample.offsets = {_V3f(1), _V3f(5)};
    sample.indices = {0, 3};
    sample.pointsValue = 1;
    sample.activationRead = _In(InputTag::Float, ReadMode::Resolved, {1});
    channel.samples.push_back(std::move(sample));
    revision.blendChannels.push_back(std::move(channel));
    revision.packetInfluences = {_M(2)};
    revision.meshWorldInverse = _M(3);
    revision.weightObject = 0;
    revision.topology = std::make_unique<fb::RigExecWireSkinTopology>();
    revision.topology->elementSize = 2;
    revision.topology->pointCount = 2;
    revision.topology->counts8 = {1, 2};
    revision.topology->indexWidth = 1;
    revision.topology->indices8 = {3, 0, 1};
    revision.topology->weights = _Fs(3);
    revision.partitionSameAsTopology = true;
    revision.defaultWeight = _In(InputTag::Float, ReadMode::Resolved, {2});
    fb::RigExecWireChain chain;
    chain.target = _pMeshPoints;
    chain.revisions.push_back(std::move(revision));
    fb::RigExecWireDerived derived;
    derived.target = _pMeshPoints;
    derived.revision = std::make_unique<fb::RigExecWireRevision>();
    derived.revision->op = uint8_t(fb::RevisionOp::RecomputeNormals);
    derived.revision->binding = _Binding();
    derived.revision->meshWorldInverse = _M(4);
    chain.derived.push_back(std::move(derived));
    chain.haveBase = true;
    chain.base = 1;
    g.chains.push_back(std::move(chain));
    g.revisionIndex = {{0, 0}};
    g.derivedIndex = {{0, 0}};
    g.chainRevisionBegin = {0};
    g.chainRevisionEnd = {1};
    g.revisionChunkBase = {0};
    g.revisionChunkCount = {0};
    g.chainChunkBegin = {0};
    g.chainChunkEnd = {0};

    fb::RigExecWireWeightObject weight;
    weight.path = _pWeight;
    weight.type = _pTokenStatic;
    weight.falloffCurve = _Fs(3, 1);
    for (auto *read :
         {&weight.defaultWeight, &weight.driver, &weight.scale, &weight.bias,
          &weight.strength, &weight.invert, &weight.falloffMin,
          &weight.falloffMax, &weight.scaleXPos, &weight.scaleYPos,
          &weight.scaleZPos, &weight.scaleXNeg, &weight.scaleYNeg,
          &weight.scaleZNeg, &weight.scaleX, &weight.scaleY, &weight.scaleZ,
          &weight.extentU, &weight.extentV}) {
        *read = _In(InputTag::Float);
    }
    weight.oracleStaticError = "static note";
    g.weightObjects.push_back(std::move(weight));
    g.falloffPaths = {_pWeight};
    g.falloffLuts.resize(1);
    g.falloffLuts[0].v = _Fs(5, 3);
    g.deltaBasePaths = {_pCtl};
    g.deltaBaseMatrix = {_M(5)};
    g.deltaBaseOk = {1};

    // Path reads, sorted by (path, rest): static values of each pooled
    // and struct kind, and one live read with the head fallback.
    const auto value = [&](uint32_t path, bool rest, fb::PathTag tag) {
        fb::RigExecWirePathRead read;
        read.path = path;
        read.rest = rest;
        read.value = std::make_unique<fb::RigExecWirePathValue>();
        read.value->tag = tag;
        return read;
    };
    fb::RigExecWirePathRead read = value(_pCtlTx, false, fb::PathTag::Double);
    read.value->bits = _specialD[1];
    g.pathReads.push_back(std::move(read));
    read = value(_pCtlTx, true, fb::PathTag::DoubleArray);
    read.value->array = 1;
    g.pathReads.push_back(std::move(read));
    read = value(_pMeshPoints, false, fb::PathTag::Vec3fArray);
    read.value->array = 1;
    g.pathReads.push_back(std::move(read));
    read = fb::RigExecWirePathRead();
    read.path = _pCtlW;
    read.read = _In(InputTag::Float, ReadMode::Resolved, {1});
    read.headFallback = true;
    g.pathReads.push_back(std::move(read));
    read = value(_pMoverDw, false, fb::PathTag::Matrix4d);
    read.value->matrix = std::make_unique<RigExecWireMatrix4d>(_M(2));
    g.pathReads.push_back(std::move(read));
    read = value(_pMoverDw, true, fb::PathTag::Float);
    read.value->bits = _specialF[2];
    g.pathReads.push_back(std::move(read));

    // A float chain on /Rig/Mover.dw, one curve revision, and a phased
    // consumer at /Rig/Ctl.w.
    fb::RigExecWirePropertyChain property;
    property.target = 2;
    property.valueType = fb::PropertyValueType::Float;
    fb::RigExecWirePropertyRevision step0;
    step0.mover = _pMover;
    step0.op = fb::PropertyOp::Curve;
    step0.enabled = _In(InputTag::Bool, ReadMode::Pinned);
    step0.defaultWeight = _In(InputTag::Float, ReadMode::Pinned, {2});
    step0.value = _In(InputTag::Float, ReadMode::Resolved, {1, 0});
    step0.min = _In(InputTag::Float, ReadMode::Pinned);
    step0.max = _In(InputTag::Float, ReadMode::Pinned);
    step0.keys = {_V2f(0), _V2f(2)};
    step0.hasTangentsAttr = true;
    step0.tangents = {_V2f(4), _V2f(1)};
    property.revisions.push_back(std::move(step0));
    f.propertyChains.push_back(std::move(property));
    fb::RigExecWirePhasedConsumer phased;
    phased.chain = 0;
    phased.consumer = 1;
    phased.consumerType = fb::PropertyValueType::Float;
    phased.applied = 1;
    phased.hops = {1};
    f.phasedConsumers.push_back(std::move(phased));

    f.compileDiagnostics = {"compile note", ""};
    f.presentation = _PresentationBytes();
    return f;
}

// Legacy fixture region indices remain relative to this explicitly declared
// format-9 head prefix. These helpers do not modify validation inputs.
size_t
_RegionBegin(const RigExecWireFile &f)
{
    size_t n = 0;
    while (n < f.steps.size() && f.steps[n].isHead) ++n;
    return n;
}

fb::RigExecWireStep &_RegionStep(RigExecWireFile &f, size_t n)
{ return f.steps[_RegionBegin(f) + n]; }
const fb::RigExecWireStep &_RegionStep(const RigExecWireFile &f, size_t n)
{ return f.steps[_RegionBegin(f) + n]; }

std::vector<int32_t>
_RegionIds(const RigExecWireFile &f, std::initializer_list<int32_t> ids)
{
    std::vector<int32_t> result(ids);
    for (auto &id : result) if (id >= 0) id += int32_t(_RegionBegin(f));
    return result;
}

std::string
_RegionDiagnostic(const RigExecWireFile &f, std::string text)
{
    const auto shift = [&](const std::string &marker) {
        size_t at = 0;
        while ((at = text.find(marker, at)) != std::string::npos) {
            const size_t begin = at + marker.size();
            size_t end = begin;
            while (end < text.size() && text[end] >= '0' && text[end] <= '9') ++end;
            if (end == begin) { at = begin; continue; }
            const std::string number = std::to_string(std::stoul(text.substr(begin, end - begin)) + _RegionBegin(f));
            text.replace(begin, end - begin, number);
            at = begin + number.size();
        }
    };
    shift("step "); shift("steps[");
    return text;
}

void
_AddFixtureHeads(RigExecWireFile &f)
{
    std::vector<fb::RigExecWireStep> heads;
    // The rich fixture has exactly one float chain and one record.
    f.propertyChains[0].versionBase = 0;
    f.phasedConsumers[0].version = 2;
    fb::RigExecWireStep base;
    base.kind = fb::StepKind::PropertyRevision;
    base.object = 0;
    base.part = 0;
    base.isHead = true;
    base.headInputSlots = {2};
    base.writes = {fb::SlotRange(fb::SlotDomain::PropertyResult, 0, 1)};
    heads.push_back(base);
    fb::RigExecWireStep revision = base;
    revision.part = 1;
    revision.headInputSlots = {0, 1, 2};
    revision.reads = {fb::SlotRange(fb::SlotDomain::PropertyResult, 0, 1)};
    revision.writes = {fb::SlotRange(fb::SlotDomain::PropertyResult, 1, 3)};
    revision.preds = {0};
    heads[0].succs = {1};
    for (auto *read : {f.propertyChains[0].revisions[0].defaultWeight.get(),
                       f.propertyChains[0].revisions[0].value.get()}) {
        for (uint32_t slot : read->walk) {
            RigExecWirePropertyInputCandidate candidate;
            candidate.slot = slot;
            candidate.raw = true;
            read->propertyCandidates.push_back(candidate);
        }
    }
    heads.push_back(revision);
    if (!f.pose->composeGroups.empty()) {
        const auto &group = f.pose->composeGroups[0];
        fb::RigExecWireStep rest;
        rest.isHead = true;
        rest.kind = fb::StepKind::RestCompose;
        rest.object = 0;
        rest.part = 0;
        rest.writes = {fb::SlotRange(fb::SlotDomain::Rest, group.begin, group.end)};
        fb::RigExecWireStep ladder = rest;
        ladder.kind = fb::StepKind::LadderCompose;
        ladder.part = 1;
        ladder.reads = rest.writes;
        ladder.writes = {fb::SlotRange(fb::SlotDomain::Ladder, group.begin, group.end)};
        for (int slot = group.begin; slot < group.end; ++slot) {
            const auto &body = f.pose->ladders[size_t(slot)];
            rest.headInputReads.insert(rest.headInputReads.end(), body.restAvars.begin(), body.restAvars.end());
            rest.headInputReads.push_back(*body.restSpace);
            ladder.headInputReads.push_back(*body.posedSpace);
            ladder.headInputReads.push_back(*body.defaultSpace);
            ladder.headInputReads.insert(ladder.headInputReads.end(), body.defaultAvars.begin(), body.defaultAvars.end());
            ladder.headInputReads.push_back(*body.rotationOrder);
        }
        for (const auto &read : rest.headInputReads)
            rest.headVaryingLeaves |= bool(read.flags & uint16_t(fb::InputReadFlags::Varying));
        for (const auto &read : ladder.headInputReads)
            ladder.headVaryingLeaves |= bool(read.flags & uint16_t(fb::InputReadFlags::Varying));
        rest.succs = {3}; ladder.preds = {2};
        heads.push_back(std::move(rest)); heads.push_back(std::move(ladder));
        for (auto &step : f.steps) {
            if (step.kind == fb::StepKind::ComposeSubtree)
                step.reads.push_back(fb::SlotRange(fb::SlotDomain::Ladder, group.begin, group.end));
            if (step.kind == fb::StepKind::Constraint || step.kind == fb::StepKind::FrameMatrix)
                step.reads.push_back(fb::SlotRange(fb::SlotDomain::Rest, 0, 1));
        }
    }
    const int32_t h = int32_t(heads.size());
    for (auto &step : f.steps) {
        for (auto &id : step.preds) if (id >= 0) id += h;
        for (auto &id : step.succs) if (id >= 0) id += h;
    }
    for (auto &cluster : f.clustering->clusters)
        for (auto &id : cluster.members) id += h;
    for (auto &id : f.cones->varyingSteps) id += h;
    for (auto &id : f.cones->overrideSteps) id += h;
    for (size_t i = 0; i < heads.size(); ++i) heads[i].cluster = 0;
    f.clustering->clusters[0].members.insert(f.clustering->clusters[0].members.begin(), heads.size(), 0);
    for (size_t i = 0; i < heads.size(); ++i) f.clustering->clusters[0].members[i] = int32_t(i);
    f.clustering->clusterOf.insert(f.clustering->clusterOf.begin(), heads.size(), 0);
    heads.insert(heads.end(), f.steps.begin(), f.steps.end());
    f.steps = std::move(heads);
    const auto crossing = [&](RigExecWireInput *read) {
        if (!read) return;
        for (uint32_t slot : read->walk) {
            if (slot != 1 && slot != 2) continue;
            RigExecWirePropertyInputCandidate candidate;
            candidate.slot = slot;
            candidate.kind = uint8_t(slot == 1 ? fb::PropertyCandidateKind::PhasedRecord : fb::PropertyCandidateKind::ChainFinal);
            candidate.version = slot == 1 ? 2 : 1;
            candidate.raw = true;
            read->propertyCandidates.push_back(candidate);
        }
    };
    crossing(f.geometry->chains[0].revisions[0].defaultWeight.get());
    crossing(f.geometry->chains[0].revisions[0].blendChannels[0].weightRead.get());
    crossing(f.geometry->chains[0].revisions[0].blendChannels[0].samples[0].activationRead.get());
    crossing(f.geometry->pathReads[3].read.get());
}

// A fixed skin layout is an explicit head producer even in fixture-only
// exports. Called while constructing the valid fixture, before mutations.
void
_AddFixtureTopologyHead(RigExecWireFile &f)
{
    const size_t h = _RegionBegin(f);
    for (auto &step : f.steps) {
        for (auto &id : step.preds) if (id >= int32_t(h)) ++id;
        for (auto &id : step.succs) if (id >= int32_t(h)) ++id;
    }
    for (auto &cluster : f.clustering->clusters)
        for (auto &id : cluster.members) if (id >= int32_t(h)) ++id;
    for (auto &id : f.cones->varyingSteps) if (id >= int32_t(h)) ++id;
    for (auto &id : f.cones->overrideSteps) if (id >= int32_t(h)) ++id;
    fb::RigExecWireStep topology;
    topology.isHead = true;
    topology.kind = fb::StepKind::SkinTopology;
    topology.object = 0;
    topology.part = 0;
    topology.cluster = 0;
    topology.writes = {fb::SlotRange(fb::SlotDomain::SkinTopology, 0, 1)};
    const auto &body = f.geometry->chains[0].revisions[0];
    for (int32_t slot : {body.jointIndicesSlot, body.jointWeightsSlot})
        if (slot >= 0) topology.headInputSlots.push_back(uint32_t(slot));
    std::sort(topology.headInputSlots.begin(), topology.headInputSlots.end());
    f.steps.insert(f.steps.begin() + std::ptrdiff_t(h), topology);
    f.clustering->clusterOf.insert(f.clustering->clusterOf.begin() + std::ptrdiff_t(h), 0);
    auto &members = f.clustering->clusters[0].members;
    members.insert(std::lower_bound(members.begin(), members.end(), int32_t(h)), int32_t(h));
}

RigExecWireFile
_RichFile()
{
    RigExecWireFile f = _RichFileBody();
    _AddFixtureHeads(f);
    return f;
}

/// Makes the rich file's revision a plugin mover, with its entry and the
/// frame bytes of a successful assembly.
void
_AddPlugin(RigExecWireFile &file)
{
    file.geometry->chains[0].revisions[0].op =
        uint8_t(fb::RevisionOp::External);
    fb::RigExecWireExternalMover mover;
    mover.type = _pTokenPlugin;
    mover.epoch = {1, 2, 3};
    mover.inputs.push_back(_InValue(InputTag::Float, ReadMode::Baked));
    mover.v2Frame = {0, 0xff, 7, 0x80};
    mover.v2FrameValid = true;
    file.externalMovers.push_back(std::move(mover));
}

/// Appends a second revision to the rich file's chain, a plugin mover
/// whose assembly failed: its entry holds no frame bytes.
void
_AddFailedPlugin(RigExecWireFile &file)
{
    fb::RigExecWireDomainGeometry &g = *file.geometry;
    fb::RigExecWireRevision revision;
    revision.moverPath = _pMover;
    revision.target = _pMeshPoints;
    revision.moverPrim = _pMover;
    revision.op = uint8_t(fb::RevisionOp::External);
    revision.binding = _Binding();
    revision.defaultWeight = _In(InputTag::Float, ReadMode::Resolved, {2});
    RigExecWirePropertyInputCandidate candidate;
    candidate.slot = 2;
    candidate.kind = uint8_t(fb::PropertyCandidateKind::ChainFinal);
    candidate.version = 1;
    candidate.raw = true;
    revision.defaultWeight->propertyCandidates.push_back(candidate);
    g.chains[0].revisions.push_back(std::move(revision));
    g.revisionIndex.push_back({0, 1});
    g.chainRevisionEnd[0] = 2;
    g.revisionChunkBase.push_back(0);
    g.revisionChunkCount.push_back(0);
    file.cones->revisionClusters.resize(2);
    file.cones->revisionStaticCluster.push_back(0);
    fb::RigExecWireExternalMover mover;
    mover.revision = 1;
    mover.type = _pTokenPlugin;
    mover.epoch = {9};
    mover.v2FrameValid = false;
    file.externalMovers.push_back(std::move(mover));
}

bool
_Write(const RigExecWireFile &file, std::vector<uint8_t> *bytes,
       std::string *why = nullptr)
{
    std::string error;
    const bool ok = RigExecFormatWrite(file, bytes, &error);
    if (why) {
        *why = error;
    }
    return ok;
}

std::unique_ptr<RigExecWireFile>
_Open(const std::vector<uint8_t> &bytes, std::string *why = nullptr)
{
    std::unique_ptr<RigExecWireFile> file;
    std::string error;
    RigExecFormatOpen(bytes.data(), bytes.size(), &file, &error);
    if (why) {
        *why = error;
    }
    return file;
}

/// A deep copy through the generated copy constructors.
RigExecWireFile
_Copy(const RigExecWireFile &file)
{
    return RigExecWireFile(file);
}

/// Packs \p file as Write does, but without validating it: the way to build
/// buffers Open must refuse.
std::vector<uint8_t>
_PackUnchecked(const RigExecWireFile &file)
{
    flatbuffers::FlatBufferBuilder builder;
    fb::FinishFileBuffer(builder, fb::File::Pack(builder, &file));
    return std::vector<uint8_t>(builder.GetBufferPointer(),
                                builder.GetBufferPointer() +
                                    builder.GetSize());
}

bool
_Contains(const std::string &text, const std::string &part)
{
    return text.find(part) != std::string::npos;
}

// ------------------------------------------------------------------ tests

void
TestMinimalRoundTrip()
{
    _context = "minimal";
    const RigExecWireFile file = _MinimalFile();
    std::string why;
    CHECK(RigExecFormatValidate(file, &why));
    if (!why.empty()) {
        std::printf("  %s\n", why.c_str());
    }
    std::vector<uint8_t> bytes;
    CHECK(_Write(file, &bytes, &why));
    CHECK(bytes.size() >= 8 && std::memcmp(bytes.data() + 4, "REXB", 4) == 0);
    const auto opened = _Open(bytes, &why);
    CHECK(opened != nullptr);
    if (!opened) {
        std::printf("  %s\n", why.c_str());
        return;
    }
    std::vector<uint8_t> again;
    CHECK(_Write(*opened, &again));
    CHECK(again == bytes);
    CHECK(RigExecFormatPathText(*opened, 1) == "/Rig");
    std::printf("minimal file: %zu bytes\n", bytes.size());
}

/// Every stored bit of every floating-point field kind survives
/// Write -> Open -> Write, and the second write is byte-identical.
void
TestBitExactness()
{
    _context = "bit exactness";
    const RigExecWireFile file = _RichFile();
    std::string why;
    const bool valid = RigExecFormatValidate(file, &why);
    CHECK(valid);
    if (!valid) {
        std::printf("  rich file refused: %s\n", why.c_str());
        return;
    }
    std::vector<uint8_t> bytes;
    CHECK(_Write(file, &bytes));
    const auto o = _Open(bytes, &why);
    CHECK(o != nullptr);
    if (!o) {
        std::printf("  %s\n", why.c_str());
        return;
    }
    std::vector<uint8_t> again;
    CHECK(_Write(*o, &again));
    CHECK(again == bytes);

    // F64 and F32 table fields.
    CHECK(_Same(o->bakeTime, file.bakeTime));
    CHECK(_Same(_RegionStep(*o, 0).sizeUnits, _RegionStep(file, 0).sizeUnits));
    CHECK(_Same(_RegionStep(*o, 0).cost, _RegionStep(file, 0).cost));
    CHECK(_Same(o->clustering->clusters[0].cost,
                file.clustering->clusters[0].cost));
    CHECK(_Same(o->clustering->grainUs, file.clustering->grainUs));
    CHECK(_Same(o->clustering->serialCost, file.clustering->serialCost));
    CHECK(_Same(o->clustering->criticalPathCost,
                file.clustering->criticalPathCost));
    const fb::RigExecWireRbf &rbf = *o->pose->poseInterpolators[0].solver;
    const fb::RigExecWireRbf &rbf0 = *file.pose->poseInterpolators[0].solver;
    CHECK(_Same(rbf.radius, rbf0.radius));
    CHECK(_Same(rbf.translationRadius, rbf0.translationRadius));
    CHECK(_Same(rbf.regularization, rbf0.regularization));
    CHECK(_Same(rbf.radii, rbf0.radii));
    CHECK(_Same(rbf.translationRadii, rbf0.translationRadii));
    CHECK(_Same(rbf.poses, rbf0.poses));
    CHECK(_Same(rbf.translations, rbf0.translations));
    CHECK(_Same(rbf.twistAxis, rbf0.twistAxis));
    CHECK(rbf.weights.size() == 2 &&
          _Same(rbf.weights[1].v, rbf0.weights[1].v));
    const fb::RigExecWireSolver &s = o->pose->solvers[0];
    const fb::RigExecWireSolver &s0 = file.pose->solvers[0];
    CHECK(_Same(s.upperLengthBase, s0.upperLengthBase));
    CHECK(_Same(s.lowerLengthBase, s0.lowerLengthBase));
    CHECK(_Same(s.splineRest->restArcLength, s0.splineRest->restArcLength));
    const auto &samples =
        o->geometry->chains[0].revisions[0].blendChannels[0].samples;
    const auto &samples0 =
        file.geometry->chains[0].revisions[0].blendChannels[0].samples;
    CHECK(samples.size() == 1 && samples0.size() == 1);
    if (samples.empty() || samples0.empty()) {
        return;
    }
    const fb::RigExecWireBlendSample &bs = samples[0];
    CHECK(_Same(bs.offsets, samples0[0].offsets));
    CHECK(bs.activationRead && samples0[0].activationRead &&
          bs.activationRead->mode == ReadMode::Resolved &&
          bs.activationRead->walk == samples0[0].activationRead->walk);

    // Structs.
    CHECK(_Same(s.ikRests, s0.ikRests));
    CHECK(_Same(s.ikParams.upperLength, s0.ikParams.upperLength) &&
          _Same(s.ikParams.lowerLength, s0.ikParams.lowerLength) &&
          _Same(s.ikParams.stretch, s0.ikParams.stretch) &&
          _Same(s.ikParams.softness, s0.ikParams.softness) &&
          _Same(s.ikParams.preferredBendRadians,
                s0.ikParams.preferredBendRadians));
    CHECK(_Same(s.splineParams.preserveVolume,
                s0.splineParams.preserveVolume) &&
          _Same(s.splineParams.midFollowWeight,
                s0.splineParams.midFollowWeight) &&
          _Same(s.splineParams.roll, s0.splineParams.roll) &&
          _Same(s.splineParams.twist, s0.splineParams.twist) &&
          _Same(s.splineParams.minLengthRatio,
                s0.splineParams.minLengthRatio) &&
          s.splineParams.aimRootTangent);
    CHECK(_Same(s.splineRest->cvs, s0.splineRest->cvs));
    CHECK(_Same(s.splineRest->rootControl, s0.splineRest->rootControl));
    CHECK(_Same(s.splineRest->joints, s0.splineRest->joints));
    CHECK(_Same(s.splineRest->segmentLengths,
                s0.splineRest->segmentLengths));
    CHECK(_Same(s.splineRest->volumeWeights, s0.splineRest->volumeWeights));
    CHECK(_Same(s.startRest, s0.startRest));
    CHECK(_Same(s.spaceRest, s0.spaceRest));
    CHECK(s.spaceSlot == 0);
    CHECK(_Same(s.twistStartRest, s0.twistStartRest));
    CHECK(_Same(s.twistWeights, s0.twistWeights));
    CHECK(_Same(s.splineRestWeights, s0.splineRestWeights));
    CHECK(_Same(s.ribbonConstantPoints, s0.ribbonConstantPoints));
    CHECK(_Same(s.jointRests, s0.jointRests));
    CHECK(s.outputs == s0.outputs);
    const fb::RigExecWireConstants &c = *o->constants;
    const fb::RigExecWireConstants &c0 = *file.constants;
    CHECK(_Same(c.restM, c0.restM));
    CHECK(_Same(c.restPts, c0.restPts));
    CHECK(_Same(c.restFrames, c0.restFrames));
    CHECK(_Same(c.selfD, c0.selfD) && _Same(c.parentDinv, c0.parentDinv));
    CHECK(_Same(c.restRoundTrip, c0.restRoundTrip) &&
          _Same(c.defaultRoundTrip, c0.defaultRoundTrip) &&
          _Same(c.posedAuthoredM, c0.posedAuthoredM));
    CHECK(_Same(c.avarConstants, c0.avarConstants));
    CHECK(c.rotationSign == c0.rotationSign);
    const fb::RigExecWireConstraint &k = o->pose->constraints[0];
    CHECK(_Same(k.aimAxisFallback,
                file.pose->constraints[0].aimAxisFallback));
    CHECK(_Same(k.sceneUp, file.pose->constraints[0].sceneUp));
    CHECK(k.flags == uint8_t(fb::ConstraintFlags::ANY));
    const fb::RigExecWireConstraintArrays &a = o->pose->constraintArrays[0];
    const fb::RigExecWireConstraintArrays &a0 =
        file.pose->constraintArrays[0];
    CHECK(_Same(a.weights, a0.weights) &&
          _Same(a.translationOffsets, a0.translationOffsets) &&
          _Same(a.rotationOffsets, a0.rotationOffsets) &&
          _Same(a.poleWeights, a0.poleWeights));
    CHECK(a.diagnostics == a0.diagnostics &&
          a.poleDiagnostics == a0.poleDiagnostics);
    CHECK(_Same(o->pose->nativeSources[0].frame,
                file.pose->nativeSources[0].frame));
    const RigExecWireConstraintSource &cs = o->pose->commits[0].sources[0];
    const RigExecWireConstraintSource &cs0 =
        file.pose->commits[0].sources[0];
    CHECK(_Same(cs.frame, cs0.frame) &&
          _Same(cs.normalizedWeight, cs0.normalizedWeight) &&
          _Same(cs.translationOffset, cs0.translationOffset) &&
          _Same(cs.rotationOffsetDegrees, cs0.rotationOffsetDegrees));
    const fb::RigExecWireSpaceSwitch &sw = o->pose->spaceSwitches[0];
    CHECK(_Same(sw.twistAxis, file.pose->spaceSwitches[0].twistAxis));
    CHECK(sw.affectTranslation == file.pose->spaceSwitches[0].affectTranslation &&
          sw.affectRotation == file.pose->spaceSwitches[0].affectRotation &&
          sw.affectScale == file.pose->spaceSwitches[0].affectScale);
    CHECK(sw.parentRead && sw.parentRead->anchor == -1 &&
          sw.parentRead->recompose == std::vector<int32_t>{0});
    CHECK(sw.sourceReads.size() == 1 && sw.sourceReads[0].anchor == -1 &&
          sw.sourceReads[0].recompose.empty());
    CHECK(sw.spaceRead && sw.spaceRead->anchor == -1 &&
          sw.spaceRead->recompose == std::vector<int32_t>{0});
    const fb::RigExecWireRevision &r = o->geometry->chains[0].revisions[0];
    const fb::RigExecWireRevision &r0 = file.geometry->chains[0].revisions[0];
    CHECK(_Same(r.packetInfluences, r0.packetInfluences));
    CHECK(_Same(r.meshWorldInverse, r0.meshWorldInverse));
    CHECK(r.binding->transformPhase.kind == 1 &&
          r.binding->transformPhase.prim == _pMesh);
    CHECK(r.topology && _Same(r.topology->weights, r0.topology->weights) &&
          r.topology->indices8 == r0.topology->indices8 &&
          r.topology->counts8 == r0.topology->counts8);
    CHECK(r.partitionSameAsTopology && !r.partitionTopology);
    CHECK(_Same(o->geometry->deltaBaseMatrix, file.geometry->deltaBaseMatrix));
    const fb::RigExecWireWeightObject &w = o->geometry->weightObjects[0];
    CHECK(_Same(w.falloffCurve, file.geometry->weightObjects[0].falloffCurve));
    CHECK(w.oracleStaticError == "static note");
    CHECK(_Same(o->geometry->falloffLuts[0].v,
                file.geometry->falloffLuts[0].v));
    const fb::RigExecWirePropertyRevision &pr =
        o->propertyChains[0].revisions[0];
    CHECK(_Same(pr.keys, file.propertyChains[0].revisions[0].keys) &&
          _Same(pr.tangents, file.propertyChains[0].revisions[0].tangents));

    // Value bits and their structs.
    CHECK(o->values.size() == file.values.size());
    for (size_t i = 0; i < o->values.size() && i < file.values.size(); ++i) {
        CHECK(o->values[i].tag == file.values[i].tag &&
              o->values[i].bits == file.values[i].bits);
    }
    CHECK(o->values[_vMatrix].matrix &&
          _Same(*o->values[_vMatrix].matrix, *file.values[_vMatrix].matrix));
    CHECK(o->values[_vVec3d].vec3d &&
          _Same(*o->values[_vVec3d].vec3d, *file.values[_vVec3d].vec3d));
    CHECK(o->values[_vVec3f].vec3f &&
          _Same(*o->values[_vVec3f].vec3f, *file.values[_vVec3f].vec3f));
    CHECK(!o->values[_vNegZero].matrix && !o->values[_vNegZero].vec3d &&
          !o->values[_vNegZero].vec3f);

    // Pools.
    CHECK(o->intArrays[1].v == file.intArrays[1].v);
    CHECK(_Same(o->floatArrays[1].v, file.floatArrays[1].v));
    CHECK(_Same(o->doubleArrays[1].v, file.doubleArrays[1].v));
    CHECK(_Same(o->vec2fArrays[1].v, file.vec2fArrays[1].v));
    CHECK(_Same(o->vec3fArrays[1].v, file.vec3fArrays[1].v));

    // Path reads of both forms.
    const auto &reads = o->geometry->pathReads;
    CHECK(reads.size() == 6);
    if (reads.size() == 6) {
        CHECK(reads[0].value && reads[0].value->bits == _specialD[1]);
        CHECK(reads[3].read && reads[3].headFallback &&
              reads[3].read->walk == std::vector<uint32_t>{1});
        CHECK(reads[4].value && reads[4].value->matrix &&
              _Same(*reads[4].value->matrix, _M(2)));
        CHECK(reads[5].value && reads[5].value->bits == _specialF[2]);
    }

    // The read every slot's default and walk survive.
    CHECK(o->inputs.size() == 3 && o->inputs[1].phased() == 0 &&
          o->inputs[2].chain() == 0 && o->listedInputs == 3);
    const RigExecWireInput &pinned = o->pose->ladders[0].restAvars[0];
    CHECK(pinned.overrideIndex == 0 && pinned.selected == 0 &&
          pinned.walk == std::vector<uint32_t>{0});
    CHECK(o->compileDiagnostics == file.compileDiagnostics);
    CHECK(o->presentation == file.presentation);

    // The specials really are what the test means.
    CHECK(std::signbit(_SD(0)) && _SD(0) == 0.0);
    CHECK(_Bits(o->bakeTime) == 0x8000000000000000ull);
    CHECK(_Bits(o->clustering->grainUs) == _specialD[4]);
    CHECK(_Bits(bs.offsets[0][0]) == _specialF[1]);
    std::printf("rich file: %zu bytes, bit-exact through Write -> Open -> "
                "Write\n",
                bytes.size());
}

void
TestDeterminism()
{
    _context = "determinism";
    std::vector<uint8_t> first, second, third, copied;
    CHECK(_Write(_RichFile(), &first));
    CHECK(_Write(_RichFile(), &second));
    const RigExecWireFile file = _RichFile();
    CHECK(_Write(file, &third));
    CHECK(_Write(_Copy(file), &copied));
    CHECK(!first.empty() && first == second && first == third &&
          first == copied);
}

/// The refusals before and during verification, and the copy Open makes.
void
TestOpenRefusals()
{
    _context = "open refusals";
    std::vector<uint8_t> bytes;
    CHECK(_Write(_RichFile(), &bytes));
    std::string why;

    // Short buffers and the old container.
    CHECK(!_Open({}, &why) && _Contains(why, "not a .rigexec"));
    CHECK(!_Open(std::vector<uint8_t>(bytes.begin(), bytes.begin() + 7),
                 &why) &&
          _Contains(why, "not a .rigexec"));
    std::vector<uint8_t> old = {'R', 'E', 'X', 'B', 3, 0, 3, 0,
                                0,   0,   0,   0,   0, 0, 0, 0};
    CHECK(!_Open(old, &why) &&
          why == "not a v4 .rigexec (old REXB container); rebake");
    std::unique_ptr<RigExecWireFile> out;
    CHECK(!RigExecFormatOpen(nullptr, 64, &out, &why) && !out);

    // The file identifier.
    std::vector<uint8_t> wrongId = bytes;
    wrongId[7] = 'P';
    CHECK(!_Open(wrongId, &why) && _Contains(why, "identifier"));

    // The verifier.
    std::vector<uint8_t> truncated(bytes.begin(),
                                   bytes.begin() + bytes.size() / 2);
    CHECK(!_Open(truncated, &why) && _Contains(why, "malformed"));
    std::vector<uint8_t> wildRoot = bytes;
    wildRoot[0] = 0xf0;
    wildRoot[1] = 0xff;
    wildRoot[2] = 0xff;
    wildRoot[3] = 0x0f;
    CHECK(!_Open(wildRoot, &why) && _Contains(why, "malformed"));

    // Every prior version lacks the promoted shared head tier and must be
    // re-exported. Future versions still require a supported exporter.
    RigExecWireFile versioned = _RichFile();
    for (uint32_t version = 0; version <= RigExecFormatVersion + 1; ++version) {
        if (version == RigExecFormatVersion) continue;
        _context = "open refusals: version " + std::to_string(version);
        versioned.formatVersion = version;
        const std::string expected = "unsupported .rigexec format version " +
            std::to_string(version) + " (this reader reads 9); " +
            (version < RigExecFormatVersion ? "re-export: S3 head tier" : "rebake");
        CHECK(!_Open(_PackUnchecked(versioned), &why) && why == expected);
    }
    _context = "open refusals";
    versioned.formatVersion = RigExecFormatVersion;
    std::vector<uint8_t> current;
    CHECK(_Write(versioned, &current) && _Open(current, &why) != nullptr);
    std::printf("format versions: 0 through 8 refused with a re-export, "
                "%u with a rebake; %u writes and opens\n",
                RigExecFormatVersion + 1, RigExecFormatVersion);

    // A verified buffer that breaks a rule.
    RigExecWireFile broken = _RichFile();
    broken.pose->ladders[0].restSpace->constant = _vNegZero;
    CHECK(!_Open(_PackUnchecked(broken), &why) &&
          _Contains(why, "invalid .rigexec") &&
          _Contains(why, "pose.ladders[0].rest_space"));

    // A nested presentation that is not a REXP buffer.
    RigExecWireFile badNested = _RichFile();
    badNested.presentation.assign(32, 0x5a);
    CHECK(!_Open(_PackUnchecked(badNested), &why) &&
          _Contains(why, "presentation"));

    // Open takes the bytes at any address and keeps nothing of them: the
    // opened file survives the caller overwriting its buffer.
    std::vector<uint8_t> shifted(bytes.size() + 3);
    std::memcpy(shifted.data() + 3, bytes.data(), bytes.size());
    std::unique_ptr<RigExecWireFile> fromShifted;
    CHECK(RigExecFormatOpen(shifted.data() + 3, bytes.size(), &fromShifted,
                            &why) &&
          fromShifted);
    std::fill(shifted.begin(), shifted.end(), uint8_t(0xa5));
    std::vector<uint8_t> rewrittenShifted;
    CHECK(fromShifted && _Write(*fromShifted, &rewrittenShifted) &&
          rewrittenShifted == bytes);

    // Every single-byte corruption is refused or opens cleanly; none
    // crashes. What opens must write back to a buffer Open accepts.
    size_t refused = 0, opened = 0;
    for (size_t i = 0; i < bytes.size(); ++i) {
        std::vector<uint8_t> flipped = bytes;
        flipped[i] ^= 0xff;
        std::string reason;
        const auto file = _Open(flipped, &reason);
        if (!file) {
            ++refused;
            CHECK(!reason.empty());
            continue;
        }
        ++opened;
        std::vector<uint8_t> rewritten;
        CHECK(_Write(*file, &rewritten) && _Open(rewritten) != nullptr);
    }
    std::printf("byte flips: %zu refused, %zu opened (of %zu bytes)\n",
                refused, opened, bytes.size());
}

void
TestWriteRefuses()
{
    _context = "write refuses";
    RigExecWireFile file = _RichFile();
    file.slotMeta.reset();
    std::vector<uint8_t> bytes = {1, 2, 3};
    std::string why;
    CHECK(!_Write(file, &bytes, &why) && bytes.empty() &&
          _Contains(why, "required root table"));
    RigExecWireFile unversioned = _RichFile();
    unversioned.formatVersion = 0;
    CHECK(!_Write(unversioned, &bytes, &why) &&
          _Contains(why, "format_version"));
}

void
TestPathText()
{
    _context = "path text";
    const RigExecWireFile file = _RichFile();
    CHECK(RigExecFormatPathText(file, 0).empty());
    CHECK(RigExecFormatPathText(file, _pRig) == "/Rig");
    CHECK(RigExecFormatPathText(file, _pCtl) == "/Rig/Ctl");
    CHECK(RigExecFormatPathText(file, _pCtlTx) == "/Rig/Ctl.tx");
    CHECK(RigExecFormatPathText(file, _pMeshPoints) == "/Rig/Mesh.points");
    CHECK(RigExecFormatPathText(file, _pTokenXyz) == "XYZ");
    CHECK(RigExecFormatPathText(file, 9999).empty());
}

/// The validator refuses one violation of each rule family; each mutation
/// of the rich file must fail and name \p part.
void
TestValidationSmoke()
{
    _context = "validation";
    int cases = 0;
    const auto expect = [&](const char *name, const char *part,
                            const std::function<void(RigExecWireFile &)>
                                &mutate) {
        _context = std::string("validation: ") + name;
        RigExecWireFile file = _RichFile();
        mutate(file);
        std::string why;
        const bool ok = RigExecFormatValidate(file, &why);
        CHECK(!ok);
        CHECK(_Contains(why, _RegionDiagnostic(file, part)));
        if (ok || !_Contains(why, _RegionDiagnostic(file, part))) {
            std::printf("  got: %s\n", ok ? "(accepted)" : why.c_str());
        }
        ++cases;
    };
    using F = RigExecWireFile;
    // The path tree.
    expect("names[0]", "names[0]", [](F &f) { f.names[0] = "x"; });
    expect("repeated name", "names[", [](F &f) { f.names[3] = "Rig"; });
    expect("parent after node", "malformed prim node",
           [](F &f) { f.paths[_pCtl] = fb::PathNode(5, 2, PathKind::Prim); });
    expect("property under property", "malformed property node", [](F &f) {
        f.paths[_pCtlW] = fb::PathNode(_pCtlTx, 8, PathKind::Property);
    });
    expect("duplicate node", "two nodes", [](F &f) {
        f.paths[_pMover] = fb::PathNode(_pRig, 5, PathKind::Prim);
    });
    expect("rig not a prim", "file.rig", [](F &f) { f.rig = _pCtlTx; });
    // Values.
    expect("values[0]", "values[0]", [](F &f) { f.values[0].bits = 1; });
    expect("matrix missing", "values[11]",
           [](F &f) { f.values[_vMatrix].matrix.reset(); });
    expect("bool bits", "values[8]", [](F &f) { f.values[_vBool].bits = 2; });
    expect("token not a token", "values[10]",
           [](F &f) { f.values[_vToken].bits = _pCtl; });
    // Pools.
    expect("pool [0]", "float_arrays[0]",
           [](F &f) { f.floatArrays[0].v = {1.0f}; });
    // The input list.
    expect("unsorted listed", "sorted", [](F &f) {
        std::swap(f.inputs[0], f.inputs[1]);
        f.phasedConsumers[0].consumer = 0;
        f.phasedConsumers[0].hops = {0};
    });
    expect("slot default tag", "inputs[0]", [](F &f) {
        f.inputs[0] = fb::InputSlot(_pCtlTx, _vFloatNaN, -1, -1,
                                    InputTag::Double,
                                    uint8_t(fb::InputSlotFlags::Listed));
    });
    expect("listed prefix", "inputs[2]", [](F &f) { f.listedInputs = 2; });
    // Reads.
    expect("walk out of range", "walk[0]", [](F &f) {
        f.pose->solvers[0].bend->walk = {7};
    });
    expect("selected on a constant read", "selected", [](F &f) {
        f.pose->solvers[0].bend->walk = {0};
        f.pose->solvers[0].bend->selected = 0;
    });
    expect("constant tag", "constant", [](F &f) {
        f.pose->solvers[0].roll->constant = _vFloatNaN;
    });
    expect("raw walk", "Raw", [](F &f) {
        _AddPlugin(f);
        f.externalMovers[0].inputs[0].mode = ReadMode::Raw;
    });
    // Override numbers and field types.
    expect("override uncovered", "override_count",
           [](F &f) { f.pose->overrideCount = 2; });
    expect("override twice", "override number", [](F &f) {
        f.pose->solvers[0].bend->walk = {0};
        f.pose->solvers[0].bend->overrideIndex = 0;
    });
    expect("field tag", "stretch", [](F &f) {
        f.pose->solvers[0].stretch = _In(InputTag::Double);
    });
    expect("registered read mode", "read mode", [](F &f) {
        f.pose->constraints[0].offset->mode = ReadMode::Resolved;
    });
    expect("missing required read", "twist_turns: missing",
           [](F &f) { f.pose->solvers[0].twistTurns.reset(); });
    expect("activation read tag", "activation_read", [](F &f) {
        f.geometry->chains[0].revisions[0].blendChannels[0].samples[0]
            .activationRead = _In(InputTag::Double, ReadMode::Resolved);
    });
    // Blend sample layouts: offsets without indices run over every point,
    // and an applied layout indexes only its points.
    expect("blend offsets without indices", "samples[0].offsets", [](F &f) {
        f.geometry->chains[0].revisions[0].blendChannels[0].samples[0]
            .indices.clear();
    });
    expect("blend layout index", "samples[0].indices[1]: 3 outside", [](F &f) {
        fb::RigExecWireBlendSample &sample =
            f.geometry->chains[0].revisions[0].blendChannels[0].samples[0];
        sample.hasLayout = true;
        sample.layoutValid = true;
        sample.pointCount = 2;
    });
    expect("blend layout negative index", "samples[0].indices[0]: -1",
           [](F &f) {
               fb::RigExecWireBlendSample &sample =
                   f.geometry->chains[0].revisions[0].blendChannels[0]
                       .samples[0];
               sample.layoutValid = true;
               sample.pointCount = 4;
               sample.indices[0] = -1;
           });
    // Binding attribute roles name properties.
    expect("binding attribute role", "binding.base: path id", [](F &f) {
        f.geometry->chains[0].revisions[0].binding->base = _pMesh;
    });
    expect("binding curve role", "binding.driver_curve_knots", [](F &f) {
        f.geometry->chains[0].revisions[0].binding->driverCurveKnots =
            _pTokenXyz;
    });
    // Avar bindings.
    expect("avar flat", "flat", [](F &f) { f.pose->avarBindings[0].flat = 11; });
    // Slot tables and constants.
    expect("slot kind size", "slot_kind",
           [](F &f) { f.slotMeta->slotKind.clear(); });
    expect("rotation sign", "rotation_sign",
           [](F &f) { f.constants->rotationSign = {8}; });
    // Steps, clusters, cones.
    expect("step cluster", "step 0 names cluster 1, which is no cluster",
           [](F &f) { _RegionStep(f, 0).cluster = 1; });
    expect("cone list cluster", "chain_base_clusters[0]",
           [](F &f) { f.cones->chainBaseClusters[0].v = {1}; });
    expect("cluster set words", "cones.always",
           [](F &f) { f.cones->always->words = {0, 0}; });
    // Every slot's avars and every revision's static step have a cluster.
    expect("avar cluster none", "cones.avar_cluster[0]: -1 out of range",
           [](F &f) { f.cones->avarCluster = {-1}; });
    expect("revision static cluster none",
           "cones.revision_static_cluster[0]: -1 out of range",
           [](F &f) { f.cones->revisionStaticCluster = {-1}; });
    // Solvers and interpolators.
    expect("ik rests", "ik_rests",
           [](F &f) { f.pose->solvers[0].ikRests.pop_back(); });
    expect("rbf weights", "weights[0]", [](F &f) {
        f.pose->poseInterpolators[0].solver->weights[0].v.pop_back();
    });
    // Constraints.
    expect("envelope without weight object", "weight_object_index",
           [](F &f) { f.pose->constraints[0].weightObjectIndex = 0; });
    // Space switches.
    expect("switch filters", "filters", [](F &f) {
        f.pose->spaceSwitches[0].filters = {0, 1};
    });
    expect("switch parent read missing", "parent_read: missing",
           [](F &f) { f.pose->spaceSwitches[0].parentRead.reset(); });
    expect("switch space read missing", "space_read: missing",
           [](F &f) { f.pose->spaceSwitches[0].spaceRead.reset(); });
    expect("switch source reads", "source_reads",
           [](F &f) { f.pose->spaceSwitches[0].sourceReads.clear(); });
    expect("switch read anchor", "parent_read.anchor",
           [](F &f) { f.pose->spaceSwitches[0].parentRead->anchor = 1; });
    expect("switch recompose slot", "source_reads[0].recompose[0]",
           [](F &f) {
               f.pose->spaceSwitches[0].sourceReads[0].recompose = {1};
           });
    expect("switch recompose kind", "not a FirstFramePose slot", [](F &f) {
        f.slotMeta->slotKind[0] = fb::SlotKind::XformDerived;
    });
    // A world source and a missing space carry the unread version.
    expect("switch world source anchor", "source_reads[0]: reads no slot",
           [](F &f) { f.pose->spaceSwitches[0].sourceReads[0].anchor = 0; });
    expect("switch world source recompose",
           "source_reads[0]: reads no slot", [](F &f) {
               f.pose->spaceSwitches[0].sourceReads[0].recompose = {0};
           });
    expect("switch missing space read", "space_read: reads no slot",
           [](F &f) { f.pose->spaceSwitches[0].spaceSlot = -1; });
    // Commits and constraint arrays.
    expect("version pools", "version pools",
           [](F &f) { f.pose->commits[0].slotWrites = {2}; });
    expect("array weights", "weights",
           [](F &f) { f.pose->constraintArrays[0].weights.pop_back(); });
    // Revisions and the dense indices.
    expect("reserved op", "reserved",
           [](F &f) { f.geometry->chains[0].revisions[0].op = 10; });
    expect("unknown op", "reserved or unknown",
           [](F &f) { f.geometry->chains[0].revisions[0].op = 19; });
    expect("revision index", "revision_index",
           [](F &f) { f.geometry->revisionIndex[0].second = 1; });
    expect("derived default weight", "default_weight", [](F &f) {
        f.geometry->chains[0].derived[0].revision->defaultWeight =
            _In(InputTag::Float, ReadMode::Resolved);
    });
    // Sparse skin topology.
    expect("topology sum", "kept entries", [](F &f) {
        f.geometry->chains[0].revisions[0].topology->counts8 = {2, 2};
    });
    expect("index width", "index_width", [](F &f) {
        f.geometry->chains[0].revisions[0].topology->indexWidth = 3;
    });
    expect("partition identity", "partition_same_as_topology", [](F &f) {
        f.geometry->chains[0].revisions[0].partitionTopology =
            std::make_unique<fb::RigExecWireSkinTopology>();
    });
    expect("validated index range",
           "topology: validated, but kept entry 0 indexes influence 3 of 3",
           [](F &f) {
               fb::RigExecWireSkinTopology &t =
                   *f.geometry->chains[0].revisions[0].topology;
               t.validated = true;
               t.influenceCount = 3;
           });
    expect("raw arrays on a sparse layout", "raw_indices or raw_weights",
           [](F &f) {
               f.geometry->chains[0].revisions[0].topology->rawWeights = {
                   1.0f};
           });
    expect("chunked with one chunk", "chunked with 1 chunk(s)", [](F &f) {
        f.geometry->chains[0].revisions[0].chunked = true;
        f.geometry->chains[0].revisions[0].chunks.resize(1);
        f.geometry->revisionChunkCount = {1};
    });
    // Weight objects.
    expect("weight dependency order", "base",
           [](F &f) { f.geometry->weightObjects[0].base = 0; });
    expect("validity mask", "target_valid", [](F &f) {
        f.geometry->weightObjects[0].targetValid = {1};
    });
    // Path reads and chain bases.
    expect("path reads order", "sorted",
           [](F &f) { std::swap(f.geometry->pathReads[0],
                                f.geometry->pathReads[1]); });
    expect("value and read", "both or neither", [](F &f) {
        f.geometry->pathReads[3].value =
            std::make_unique<fb::RigExecWirePathValue>();
    });
    expect("array pool id", "array", [](F &f) {
        f.geometry->pathReads[1].value->array = 2;
    });
    expect("base without have_base", "have_base",
           [](F &f) { f.geometry->chains[0].haveBase = false; });
    // Property chains.
    expect("phased applied", "phased_consumers[0]",
           [](F &f) { f.phasedConsumers[0].applied = 2; });
    expect("phased hops", "malformed hops",
           [](F &f) { f.phasedConsumers[0].hops = {0, 1}; });
    expect("phased hop at the target", "malformed hops",
           [](F &f) { f.phasedConsumers[0].hops = {1, 2}; });
    expect("chain flag", "has_property_chains",
           [](F &f) { f.pose->hasPropertyChains = false; });
    expect("chain target type", "property_chains[0]: target slot 2", [](F &f) {
        f.propertyChains[0].valueType = fb::PropertyValueType::Double;
    });
    expect("keys off a curve", "keys", [](F &f) {
        f.propertyChains[0].revisions[0].op = fb::PropertyOp::Add;
    });
    // Plugin movers.
    expect("plugin mover without entry", "external_movers", [](F &f) {
        f.geometry->chains[0].revisions[0].op =
            uint8_t(fb::RevisionOp::External);
    });
    // The nested presentation.
    expect("presentation", "presentation",
           [](F &f) { f.presentation.assign(16, 0); });
    std::printf("validation smoke: %d rule violations refused\n", cases);

    // And a plugin mover with its entry is accepted.
    _context = "validation: plugin mover";
    RigExecWireFile plugin = _RichFile();
    _AddPlugin(plugin);
    std::string why;
    CHECK(RigExecFormatValidate(plugin, &why));
    if (!why.empty()) {
        std::printf("  %s\n", why.c_str());
    }
    plugin.externalMovers.push_back(plugin.externalMovers[0]);
    CHECK(!RigExecFormatValidate(plugin, &why) &&
          _Contains(why, "external_movers[1]"));

    // A dense blend layout (offsets for every point, no indices) and an
    // applied sparse one inside its points are accepted.
    _context = "validation: blend layouts";
    for (const bool dense : {true, false}) {
        RigExecWireFile layouts = _RichFile();
        fb::RigExecWireBlendSample &sample =
            layouts.geometry->chains[0].revisions[0].blendChannels[0]
                .samples[0];
        sample.hasLayout = true;
        sample.layoutValid = true;
        sample.pointCount = dense ? sample.offsets.size() : 4;
        if (dense) {
            sample.indices.clear();
        }
        why.clear();
        CHECK(RigExecFormatValidate(layouts, &why));
        if (!why.empty()) {
            std::printf("  %s\n", why.c_str());
        }
    }
}

/// The rich file with four steps in two clusters: 0 -> 1 -> 2 -> 3 and
/// 0 -> 3, steps 0 and 1 in cluster 0, steps 2 and 3 in cluster 1, so
/// cluster 0 -> cluster 1. Steps 1 and 2 write PosedM slots 0 and 1, and
/// step 3 reads both.
RigExecWireFile
_GraphFile()
{
    RigExecWireFile f = _RichFileBody();
    f.propertyChains.clear();
    f.phasedConsumers.clear();
    f.pose->hasPropertyChains = false;
    for (auto &slot : f.inputs) slot = fb::InputSlot(slot.name(), slot.value(), -1, -1, slot.type(), slot.flags());
    fb::RigExecWireStep later = f.steps[0];
    later.reads.clear();
    later.writes.clear();
    later.overrideInputs.clear();
    f.steps.resize(4, later);
    f.steps[1].writes = {fb::SlotRange(fb::SlotDomain::PosedM, 0, 1)};
    f.steps[2].writes = {fb::SlotRange(fb::SlotDomain::PosedM, 1, 2)};
    f.steps[3].reads = {fb::SlotRange(fb::SlotDomain::PosedM, 0, 2)};
    const std::vector<std::vector<int32_t>> preds = {{}, {0}, {1}, {0, 2}};
    const std::vector<std::vector<int32_t>> succs = {{1, 3}, {2}, {3}, {}};
    f.clustering->clusterOf = {0, 0, 1, 1};
    for (size_t s = 0; s < 4; ++s) {
        f.steps[s].preds = preds[s];
        f.steps[s].succs = succs[s];
        f.steps[s].cluster = f.clustering->clusterOf[s];
    }
    f.clustering->clusters.resize(2, f.clustering->clusters[0]);
    f.clustering->clusters[0].members = {0, 1};
    f.clustering->clusters[0].succs = {1};
    f.clustering->clusters[1].members = {2, 3};
    f.clustering->clusters[1].preds = {0};
    fb::RigExecWireClusterSet set;
    set.clusters = 2;
    set.words = {3};
    f.cones->cone = {set, set};
    f.cones->cone[1].words = {2};
    f.cones->always->clusters = 2;
    f.cones->poseClusters->clusters = 2;
    f.cones->poseClusters->words = {3};
    return f;
}

/// The step and cluster graph: each rule of the shared step-graph check
/// refuses its violation of the graph file with its exact message, both
/// from the validator and, for the two the runtime's index walk and
/// cluster sort depend on most, from Open.
void
TestStepGraph()
{
    _context = "step graph";
    std::string why;
    const RigExecWireFile valid = _GraphFile();
    CHECK(RigExecFormatValidate(valid, &why));
    if (!why.empty()) {
        std::printf("  graph file refused: %s\n", why.c_str());
    }
    std::vector<uint8_t> bytes;
    CHECK(_Write(valid, &bytes) && _Open(bytes) != nullptr);

    using F = RigExecWireFile;
    int cases = 0;
    const auto expect = [&](const char *name, const std::string &text,
                            const std::function<void(F &)> &mutate) {
        _context = std::string("step graph: ") + name;
        ++cases;
        F file = _GraphFile();
        mutate(file);
        const bool ok = RigExecFormatValidate(file, &why);
        CHECK(!ok && why == _RegionDiagnostic(file, text));
        if (ok || why != _RegionDiagnostic(file, text)) {
            std::printf("  got '%s', expected '%s'\n",
                        ok ? "(accepted)" : why.c_str(), text.c_str());
        }
    };
    const auto expectOpen = [&](const char *name, const std::string &text,
                                const std::function<void(F &)> &mutate) {
        _context = std::string("step graph open: ") + name;
        ++cases;
        F file = _GraphFile();
        mutate(file);
        const bool opened = _Open(_PackUnchecked(file), &why) != nullptr;
        CHECK(!opened && why == "invalid .rigexec: " + _RegionDiagnostic(file, text));
        if (opened || why != "invalid .rigexec: " + _RegionDiagnostic(file, text)) {
            std::printf("  got '%s', expected 'invalid .rigexec: %s'\n",
                        opened ? "(opened)" : why.c_str(), text.c_str());
        }
    };
    // The two the runtime relies on most: a predecessor flipped to a later
    // step, and a two-cluster cycle.
    const auto flipped = [](F &f) { _RegionStep(f, 1).preds = {2}; };
    const auto cycle = [](F &f) {
        f.clustering->clusters[0].preds = {1};
        f.clustering->clusters[1].succs = {0};
    };
    expect("pred flipped later", "step 1 depends on later step 2", flipped);
    expectOpen("pred flipped later", "step 1 depends on later step 2",
               flipped);
    expect("two-cluster cycle",
           "the cluster graph has a cycle through cluster 0", cycle);
    expectOpen("two-cluster cycle",
               "the cluster graph has a cycle through cluster 0", cycle);
    // Step edges.
    expect("pred on itself", "step 1 depends on itself",
           [](F &f) { _RegionStep(f, 1).preds = {1}; });
    expect("preds unsorted", "step 3 lists its predecessors out of order "
                             "or twice",
           [](F &f) { _RegionStep(f, 3).preds = {2, 0}; });
    expect("preds repeated", "step 3 lists its predecessors out of order "
                             "or twice",
           [](F &f) { _RegionStep(f, 3).preds = {0, 0}; });
    expect("succ earlier", "step 2 names earlier step 1 as a successor",
           [](F &f) { _RegionStep(f, 2).succs = {1, 3}; });
    expect("pred without succ",
           "step 3 names predecessor 0, which does not name it as a "
           "successor",
           [](F &f) { _RegionStep(f, 0).succs = {1}; });
    expect("succ without pred",
           "step 1 names successor 3, which does not name it as a "
           "predecessor",
           [](F &f) { _RegionStep(f, 1).succs = {2, 3}; });
    expect("source after a step",
           "source step 1 depends on step 0, which is not a source",
           [](F &f) { _RegionStep(f, 1).isSource = true; });
    // Indices past their tables, refused before they are followed.
    expect("pred past the steps",
           "step 3 names predecessor 4, which is no step",
           [](F &f) { _RegionStep(f, 3).preds = {0, 4}; });
    expect("negative pred", "step 1 names predecessor -1, which is no step",
           [](F &f) { _RegionStep(f, 1).preds = {-1}; });
    expect("succ past the steps", "step 2 names successor 4, which is no step",
           [](F &f) { _RegionStep(f, 2).succs = {3, 4}; });
    expect("cluster past the clusters",
           "step 2 names cluster 2, which is no cluster",
           [](F &f) { _RegionStep(f, 2).cluster = 2; });
    expect("cluster_of short", "the clustering places 3 steps; the file has 4",
           [](F &f) { f.clustering->clusterOf.pop_back(); });
    expect("member past the steps",
           "cluster 1 names member 4, which is no step",
           [](F &f) { f.clustering->clusters[1].members = {2, 3, 4}; });
    expect("cluster pred past the clusters",
           "cluster 1 names predecessor 2, which is no cluster",
           [](F &f) { f.clustering->clusters[1].preds = {0, 2}; });
    // Membership.
    expect("cluster_of disagrees",
           "step 1 names cluster 0, but the clustering places it in "
           "cluster 1",
           [](F &f) { f.clustering->clusterOf[1] = 1; });
    expect("members unsorted", "cluster 0 lists its members out of order "
                               "or twice",
           [](F &f) { f.clustering->clusters[0].members = {1, 0}; });
    expect("member of another cluster",
           "cluster 0 names member 2, which the clustering places in "
           "cluster 1",
           [](F &f) { f.clustering->clusters[0].members = {0, 1, 2}; });
    expect("member missing", "step 3 is not a member of cluster 1",
           [](F &f) { f.clustering->clusters[1].members = {2}; });
    // Cluster edges.
    expect("cluster on itself", "cluster 1 depends on itself",
           [](F &f) { f.clustering->clusters[1].preds = {0, 1}; });
    expect("cluster pred without succ",
           "cluster 1 names predecessor 0, which does not name it as a "
           "successor",
           [](F &f) { f.clustering->clusters[0].succs.clear(); });
    expect("cluster succ without pred",
           "cluster 0 names successor 1, which does not name it as a "
           "predecessor",
           [](F &f) { f.clustering->clusters[1].preds.clear(); });
    expect("step edge across no cluster edge",
           "step 2 in cluster 1 depends on step 1 in cluster 0, which "
           "cluster 1 does not depend on",
           [](F &f) {
               f.clustering->clusters[0].succs.clear();
               f.clustering->clusters[1].preds.clear();
           });
    // Producers, in playback order.
    const auto unproduced = [](F &f) {
        _RegionStep(f, 3).reads = {fb::SlotRange(fb::SlotDomain::PosedM, 0, 3)};
    };
    expect("read nothing writes",
           "step 3 reads PosedM slots [0, 3), which no earlier step writes",
           unproduced);
    expectOpen("read nothing writes",
               "step 3 reads PosedM slots [0, 3), which no earlier step "
               "writes",
               unproduced);
    expect("read before its writer",
           "step 1 reads PosedM slots [1, 2), which no earlier step writes",
           [](F &f) {
               _RegionStep(f, 1).reads = {fb::SlotRange(fb::SlotDomain::PosedM, 1, 2)};
           });
    expect("aggregate nothing writes",
           "step 3 reads Aggregate slots [0, 1), which no earlier step writes",
           [](F &f) {
               _RegionStep(f, 3).reads.push_back(
                   fb::SlotRange(fb::SlotDomain::Aggregate, 0, 1));
           });
    // Step 2 becomes a source with no predecessor. It still reads step 1's
    // write, which index order would have produced, but the source pass
    // runs it before step 1.
    expect("source reads a later-running write",
           "step 2 reads PosedM slots [0, 1), which no earlier step writes",
           [](F &f) {
               _RegionStep(f, 1).succs.clear();
               _RegionStep(f, 2).preds.clear();
               _RegionStep(f, 2).isSource = true;
               _RegionStep(f, 2).reads = {fb::SlotRange(fb::SlotDomain::PosedM, 0, 1)};
           });
    // The domains a run holds before any step writes them need no producer.
    {
        _context = "step graph: unproduced source reads";
        F file = _GraphFile();
        _RegionStep(file, 3).reads.push_back(
            fb::SlotRange(fb::SlotDomain::Avars, 0, 11));
        _RegionStep(file, 3).reads.push_back(
            fb::SlotRange(fb::SlotDomain::ChainBase, 0, 1));
        why.clear();
        CHECK(RigExecFormatValidate(file, &why));
        if (!why.empty()) {
            std::printf("  source reads refused: %s\n", why.c_str());
        }
    }
    // The reserved values: the retired snapshot store is no source any
    // more, and neither its domain nor its step may appear at all.
    const auto finals = [](F &f) {
        _RegionStep(f, 2).kind = fb::StepKind::SnapshotFinals;
    };
    expect("Snapshots read", "step 3 declares the retired Snapshots domain",
           [](F &f) {
               _RegionStep(f, 3).reads.push_back(
                   fb::SlotRange(fb::SlotDomain::Snapshots, 0, 3));
           });
    expect("Snapshots write", "step 1 declares the retired Snapshots domain",
           [](F &f) {
               _RegionStep(f, 1).writes.push_back(
                   fb::SlotRange(fb::SlotDomain::Snapshots, 0, 1));
           });
    expect("empty Snapshots range",
           "step 0 declares the retired Snapshots domain", [](F &f) {
               _RegionStep(f, 0).reads.push_back(
                   fb::SlotRange(fb::SlotDomain::Snapshots, 0, 0));
           });
    expect("SnapshotFinals step", "step 2 is a retired SnapshotFinals step",
           finals);
    expectOpen("SnapshotFinals step",
               "step 2 is a retired SnapshotFinals step", finals);
    std::printf("step graph: %d violations refused\n", cases);
}

/// The nested presentation must carry the REXP identifier: a presentation
/// that is otherwise sound (it fits its bytes and verifies without an
/// identifier) is refused with none or with another one.
void
TestPresentationIdentifier()
{
    _context = "presentation identifier";
    std::string why;
    const std::pair<const char *, const char *> cases[] = {
        {"none", nullptr}, {"XXXX", "XXXX"}};
    for (const auto &[label, identifier] : cases) {
        _context = std::string("presentation identifier: ") + label;
        const std::vector<uint8_t> nested = _PresentationBytes(identifier);
        flatbuffers::Verifier plain(nested.data(), nested.size());
        CHECK(plain.VerifyBuffer<fb::Presentation>(nullptr));
        RigExecWireFile file = _RichFile();
        file.presentation = nested;
        CHECK(!RigExecFormatValidate(file, &why) &&
              _Contains(why, "presentation") && _Contains(why, "REXP"));
        CHECK(!_Open(_PackUnchecked(file), &why) &&
              _Contains(why, "invalid .rigexec") &&
              _Contains(why, "presentation"));
    }
}

/// A plugin mover whose assembly succeeded and one whose assembly failed
/// survive Write -> Open; a failed one that still holds frame bytes is
/// refused by the validator, Write and Open.
void
TestExternalMovers()
{
    _context = "external movers";
    RigExecWireFile file = _RichFile();
    _AddPlugin(file);
    _AddFailedPlugin(file);
    std::string why;
    CHECK(RigExecFormatValidate(file, &why));
    if (!why.empty()) {
        std::printf("  plugin file refused: %s\n", why.c_str());
    }
    std::vector<uint8_t> bytes;
    CHECK(_Write(file, &bytes));
    const auto o = _Open(bytes, &why);
    CHECK(o && o->externalMovers.size() == 2);
    if (o && o->externalMovers.size() == 2) {
        const fb::RigExecWireExternalMover &done = o->externalMovers[0];
        const fb::RigExecWireExternalMover &failed = o->externalMovers[1];
        CHECK(done.v2FrameValid &&
              done.v2Frame == std::vector<uint8_t>({0, 0xff, 7, 0x80}) &&
              done.epoch == std::vector<uint8_t>({1, 2, 3}));
        CHECK(!failed.v2FrameValid && failed.v2Frame.empty() &&
              failed.revision == 1 &&
              failed.epoch == std::vector<uint8_t>({9}));
        std::vector<uint8_t> again;
        CHECK(_Write(*o, &again) && again == bytes);
    }

    // A successful assembly may encode no frame bytes at all.
    RigExecWireFile empty = _Copy(file);
    empty.externalMovers[0].v2Frame.clear();
    CHECK(RigExecFormatValidate(empty, &why));

    const std::string stale =
        "external_movers[1].v2_frame: bytes of an assembly that failed "
        "(v2_frame_valid is false)";
    RigExecWireFile holding = _Copy(file);
    holding.externalMovers[1].v2Frame = {1};
    CHECK(!RigExecFormatValidate(holding, &why) && why == stale);
    std::vector<uint8_t> refused = {1};
    CHECK(!_Write(holding, &refused, &why) && refused.empty() &&
          why == "invalid .rigexec: " + stale);
    CHECK(!_Open(_PackUnchecked(holding), &why) &&
          why == "invalid .rigexec: " + stale);
    std::printf("external movers: a successful and a failed assembly "
                "round-trip; a failed one holding frame bytes is refused\n");
}

/// Appends \p count provider slots under /Rig to \p f, with their slot
/// tables, constants, avar clusters and ladders of constant reads.
/// Returns the first new slot.
int32_t
_AddSlots(RigExecWireFile &f, size_t count)
{
    const int32_t first = int32_t(f.slotMeta->paths.size());
    for (size_t k = 0; k < count; ++k) {
        f.names.push_back("Slot" + std::to_string(k));
        f.paths.push_back(fb::PathNode(
            _pRig, uint32_t(f.names.size() - 1), PathKind::Prim));
        fb::RigExecWireSlotMeta &meta = *f.slotMeta;
        meta.paths.push_back(uint32_t(f.paths.size() - 1));
        meta.slotKind.push_back(fb::SlotKind::FirstFramePose);
        meta.parent.push_back(-1);
        meta.propParent.push_back(-1);
        meta.needFinal.push_back(1);
        meta.needBase.push_back(0);
        fb::RigExecWireConstants &c = *f.constants;
        c.restM.push_back(_M(k));
        c.restPts.push_back(_L(k));
        c.restFrames.push_back(_Frame(k));
        c.selfD.push_back(_M(k + 1));
        c.parentDinv.push_back(_M(k + 2));
        c.rotOrder.push_back(_pTokenXyz);
        c.restRoundTrip.push_back(_M(k + 3));
        c.defaultRoundTrip.push_back(_M(k + 4));
        c.posedAuthored.push_back(0);
        c.posedAuthoredM.push_back(_M(k + 5));
        c.noScaleAvars.push_back(0);
        const std::vector<double> avars = _Ds(11, k);
        c.avarConstants.insert(c.avarConstants.end(), avars.begin(),
                               avars.end());
        c.rotationSign.push_back(0);
        fb::RigExecWireLadder ladder;
        ladder.restSpace = _In(InputTag::Matrix4d);
        ladder.defaultSpace = _In(InputTag::Matrix4d);
        ladder.posedSpace = _In(InputTag::Matrix4d);
        for (size_t a = 0; a < 6; ++a) {
            ladder.restAvars.push_back(_InValue(InputTag::Double));
            ladder.defaultAvars.push_back(_InValue(InputTag::Double));
        }
        ladder.rotationOrder = _In(InputTag::Token);
        f.pose->ladders.push_back(std::move(ladder));
        f.pose->restChainVaries.push_back(0);
        f.cones->avarCluster.push_back(0);
    }
    return first;
}

/// A switch of \p slot with one world source, reading its parent and space
/// as they stand (anchor -1, nothing recomposed).
fb::RigExecWireSpaceSwitch
_Switch(int32_t slot)
{
    fb::RigExecWireSpaceSwitch sw;
    sw.slot = slot;
    sw.sourceSlots = {-1};
    sw.sourceReads.resize(1);
    sw.parentRead = std::make_unique<fb::RigExecWireFrameVersion>();
    sw.spaceRead = std::make_unique<fb::RigExecWireFrameVersion>();
    sw.active = _In(InputTag::Double);
    sw.affectTranslation = {{true, true, true}};
    sw.affectRotation = {{true, true, true}};
    sw.affectScale = {{true, true, true}};
    return sw;
}

/// The rich file with two more slots, 1 and 2, switched in resolution
/// order against slot order: space_switches[0] switches slot 2, and
/// space_switches[1] switches slot 1 with slot 2 as a source and as its
/// space, both read at slot 2's switched version; slot 0 is not switched,
/// and space_switches[0] reads it as its parent.
RigExecWireFile
_SwitchFile()
{
    RigExecWireFile f = _RichFile();
    const int32_t lo = _AddSlots(f, 2);
    const int32_t hi = lo + 1;
    f.pose->spaceSwitches.clear();
    fb::RigExecWireSpaceSwitch first = _Switch(hi);
    first.parentRead->anchor = 0;
    fb::RigExecWireSpaceSwitch second = _Switch(lo);
    second.sourceSlots = {-1, hi};
    second.filters = {0, 1};
    second.sourceReads.resize(2);
    second.sourceReads[1].anchor = hi;
    second.spaceSlot = hi;
    second.spaceRead->anchor = hi;
    f.pose->spaceSwitches.push_back(std::move(first));
    f.pose->spaceSwitches.push_back(std::move(second));
    return f;
}

/// Space switches are stored in resolution order, not slot order: a slot is
/// switched at most once, and a read anchored on a switched slot names a
/// switch stored before the reading one.
void
TestSwitchOrder()
{
    _context = "switch order";
    std::string why;
    const RigExecWireFile valid = _SwitchFile();
    CHECK(RigExecFormatValidate(valid, &why));
    if (!why.empty()) {
        std::printf("  switch file refused: %s\n", why.c_str());
    }
    std::vector<uint8_t> bytes;
    CHECK(_Write(valid, &bytes));
    const auto o = _Open(bytes, &why);
    CHECK(o && o->pose->spaceSwitches.size() == 2);
    if (o && o->pose->spaceSwitches.size() == 2) {
        const auto &switches = o->pose->spaceSwitches;
        CHECK(switches[0].slot == 2 && switches[1].slot == 1);
        CHECK(switches[0].parentRead->anchor == 0);
        CHECK(switches[1].sourceReads.size() == 2 &&
              switches[1].sourceReads[1].anchor == 2 &&
              switches[1].spaceRead->anchor == 2);
    }

    using F = RigExecWireFile;
    int cases = 0;
    const auto expect = [&](const char *name, const std::string &text,
                            const std::function<void(F &)> &mutate) {
        _context = std::string("switch order: ") + name;
        ++cases;
        F file = _SwitchFile();
        mutate(file);
        const bool ok = RigExecFormatValidate(file, &why);
        CHECK(!ok && why == _RegionDiagnostic(file, text));
        if (ok || why != _RegionDiagnostic(file, text)) {
            std::printf("  got '%s', expected '%s'\n",
                        ok ? "(accepted)" : why.c_str(), text.c_str());
        }
        const bool opened = _Open(_PackUnchecked(file), &why) != nullptr;
        CHECK(!opened && why == "invalid .rigexec: " + _RegionDiagnostic(file, text));
    };
    const std::string later =
        "is switched by pose.space_switches[1], which is not stored before "
        "this switch";
    expect("slot switched twice",
           "pose.space_switches[1].slot: slot 2 is already switched by "
           "pose.space_switches[0]",
           [](F &f) { f.pose->spaceSwitches[1].slot = 2; });
    expect("source anchored on a later switch",
           "pose.space_switches[0].source_reads[0].anchor: slot 1 " + later,
           [](F &f) {
               f.pose->spaceSwitches[0].sourceSlots = {1};
               f.pose->spaceSwitches[0].sourceReads[0].anchor = 1;
           });
    expect("parent anchored on a later switch",
           "pose.space_switches[0].parent_read.anchor: slot 1 " + later,
           [](F &f) { f.pose->spaceSwitches[0].parentRead->anchor = 1; });
    expect("space anchored on a later switch",
           "pose.space_switches[0].space_read.anchor: slot 1 " + later,
           [](F &f) {
               f.pose->spaceSwitches[0].spaceSlot = 1;
               f.pose->spaceSwitches[0].spaceRead->anchor = 1;
           });
    expect("anchored on its own switch",
           "pose.space_switches[1].source_reads[1].anchor: slot 1 " + later,
           [](F &f) { f.pose->spaceSwitches[1].sourceReads[1].anchor = 1; });

    // Recomposing a switched slot reads it unswitched: not an anchor.
    _context = "switch order: recompose of a later switch's slot";
    F recomposed = _SwitchFile();
    recomposed.pose->spaceSwitches[0].parentRead->anchor = -1;
    recomposed.pose->spaceSwitches[0].parentRead->recompose = {1};
    CHECK(RigExecFormatValidate(recomposed, &why));
    std::printf("switch order: stored against slot order and anchored on "
                "an earlier switch accepted, %d violations refused\n",
                cases);
}

/// The rich file with slot 0 a volume and slot 1 (/Rig/Slot0) not, and the
/// volume's VolumePlacements step after the compose: step 1, part 1,
/// reading PoseFin[0] and writing WeightFrames[0], in cluster 0.
RigExecWireFile
_VolumeFile()
{
    RigExecWireFile f = _RichFile();
    _AddSlots(f, 1);
    f.constants->noScaleAvars = {1, 0};
    fb::RigExecWireStep place;
    place.kind = fb::StepKind::VolumePlacements;
    place.object = 0;
    place.part = 1;
    place.cluster = 0;
    place.reads = {fb::SlotRange(fb::SlotDomain::PoseFin, 0, 1)};
    place.writes = {fb::SlotRange(fb::SlotDomain::WeightFrames, 0, 1)};
    place.preds = _RegionIds(f, {0});
    _RegionStep(f, 0).succs = _RegionIds(f, {1});
    f.steps.push_back(std::move(place));
    f.clustering->clusters[0].members.push_back(int32_t(f.steps.size() - 1));
    f.clustering->clusterOf.push_back(0);
    return f;
}

/// Appends \p step to \p f in cluster 0, with no edges.
void
_AppendStep(RigExecWireFile &f, fb::RigExecWireStep step)
{
    step.cluster = 0;
    step.preds.clear();
    step.succs.clear();
    f.clustering->clusters[0].members.push_back(int32_t(f.steps.size()));
    f.clustering->clusterOf.push_back(0);
    f.steps.push_back(std::move(step));
}

/// One VolumePlacements step per volume slot, each placing its own slot
/// alone: the per-volume form is accepted and round-trips; the whole-map
/// form, a step of a slot that is no volume, a write other than its own
/// WeightFrames slot, a second step of one volume, a volume with no step,
/// another kind writing WeightFrames and a reader of a placement ahead of
/// its step are refused, each naming the step.
void
TestVolumePlacementSteps()
{
    _context = "volume placements";
    std::string why;
    const RigExecWireFile valid = _VolumeFile();
    CHECK(RigExecFormatValidate(valid, &why));
    if (!why.empty()) {
        std::printf("  volume file refused: %s\n", why.c_str());
    }
    std::vector<uint8_t> bytes;
    CHECK(_Write(valid, &bytes));
    const auto opened = _Open(bytes, &why);
    CHECK(opened && opened->steps.size() == _RegionBegin(*opened) + 2 &&
          _RegionStep(*opened, 1).kind == fb::StepKind::VolumePlacements &&
          _RegionStep(*opened, 1).object == 0 && _RegionStep(*opened, 1).part == 1);
    std::vector<uint8_t> again;
    CHECK(opened && _Write(*opened, &again) && again == bytes);

    using F = RigExecWireFile;
    int cases = 0;
    const auto expect = [&](const char *name, const std::string &text,
                            const std::function<void(F &)> &mutate) {
        _context = std::string("volume placements: ") + name;
        ++cases;
        F file = _VolumeFile();
        mutate(file);
        const bool ok = RigExecFormatValidate(file, &why);
        CHECK(!ok && why == _RegionDiagnostic(file, text));
        if (ok || why != _RegionDiagnostic(file, text)) {
            std::printf("  got '%s', expected '%s'\n",
                        ok ? "(accepted)" : why.c_str(), text.c_str());
        }
    };
    const auto wholeMap = [](F &f) { _RegionStep(f, 1).part = -1; };
    const std::string retired = "step 1 (VolumePlacements every volume "
                                "weight) is the retired whole-map placement "
                                "(part -1)";
    expect("whole-map form", retired, wholeMap);
    {
        _context = "volume placements open: whole-map form";
        ++cases;
        F file = _VolumeFile();
        wholeMap(file);
        CHECK(!_Open(_PackUnchecked(file), &why) &&
              why == "invalid .rigexec: " + _RegionDiagnostic(file, retired));
    }
    expect("another part",
           "step 1 (VolumePlacements every volume weight) has part 0, "
           "which is no VolumePlacements form",
           [](F &f) { _RegionStep(f, 1).part = 0; });
    expect("no volume slot",
           "step 1 (VolumePlacements /Rig/Slot0) places slot 1, which is no "
           "volume slot",
           [](F &f) {
               _RegionStep(f, 1).object = 1;
               _RegionStep(f, 1).reads = {
                   fb::SlotRange(fb::SlotDomain::PoseFin, 1, 2)};
               _RegionStep(f, 1).writes = {
                   fb::SlotRange(fb::SlotDomain::WeightFrames, 1, 2)};
           });
    expect("slot past the slots",
           "step 1 (VolumePlacements every volume weight) places slot 2, "
           "which is no volume slot",
           [](F &f) { _RegionStep(f, 1).object = 2; });
    expect("another volume's slot",
           "step 1 (VolumePlacements /Rig/Ctl) writes other than "
           "WeightFrames[0]",
           [](F &f) {
               _RegionStep(f, 1).writes = {
                   fb::SlotRange(fb::SlotDomain::WeightFrames, 1, 2)};
           });
    expect("more than its slot",
           "step 1 (VolumePlacements /Rig/Ctl) writes other than "
           "WeightFrames[0]",
           [](F &f) {
               _RegionStep(f, 1).writes = {
                   fb::SlotRange(fb::SlotDomain::WeightFrames, 0, 2)};
           });
    expect("no write",
           "step 1 (VolumePlacements /Rig/Ctl) writes other than "
           "WeightFrames[0]",
           [](F &f) { _RegionStep(f, 1).writes.clear(); });
    expect("a second step of one volume",
           "step 2 (VolumePlacements /Rig/Ctl) places volume slot 0 again; "
           "step 1 (VolumePlacements /Rig/Ctl) already does",
           [](F &f) { _AppendStep(f, _RegionStep(f, 1)); });
    expect("a volume with no step",
           "volume slot 1 (/Rig/Slot0) has no VolumePlacements step",
           [](F &f) { f.constants->noScaleAvars[1] = 1; });
    // A retired kind is named as one ahead of the WeightFrames rule its
    // write would break.
    expect("a placement step of the retired kind",
           "step 1 is a retired SnapshotFinals step",
           [](F &f) { _RegionStep(f, 1).kind = fb::StepKind::SnapshotFinals; });
    expect("a weight packet writing WeightFrames",
           "step 2 (WeightPacket /Rig/W) writes WeightFrames, which only a "
           "VolumePlacements step writes",
           [](F &f) {
               fb::RigExecWireStep packet;
               packet.kind = fb::StepKind::WeightPacket;
               packet.object = 0;
               packet.writes = {
                   fb::SlotRange(fb::SlotDomain::WeightPacket, 0, 1),
                   fb::SlotRange(fb::SlotDomain::WeightFrames, 0, 1)};
               _AppendStep(f, std::move(packet));
           });

    // A reader of a volume's placement: after its step it reads what the
    // step wrote; ahead of it, the producer check refuses it.
    fb::RigExecWireStep reader;
    reader.kind = fb::StepKind::WeightPacket;
    reader.object = 0;
    reader.reads = {fb::SlotRange(fb::SlotDomain::WeightFrames, 0, 1)};
    reader.writes = {fb::SlotRange(fb::SlotDomain::WeightPacket, 0, 1)};
    {
        _context = "volume placements: reader after its step";
        F file = _VolumeFile();
        _AppendStep(file, reader);
        why.clear();
        CHECK(RigExecFormatValidate(file, &why));
        if (!why.empty()) {
            std::printf("  reader after its step refused: %s\n", why.c_str());
        }
    }
    expect("reader ahead of its step",
           "step 1 reads WeightFrames slots [0, 1), which no earlier step "
           "writes",
           [&reader](F &f) {
               _AppendStep(f, reader);
               // Swap the two last steps: the reader runs before the
               // placement it reads.
               std::swap(_RegionStep(f, 1), _RegionStep(f, 2));
               _RegionStep(f, 0).succs = _RegionIds(f, {2});
               _RegionStep(f, 1).cluster = _RegionStep(f, 2).cluster = 0;
               _RegionStep(f, 1).preds.clear();
               _RegionStep(f, 2).preds = _RegionIds(f, {0});
           });
    std::printf("volume placements: the per-volume form accepted, %d "
                "violations refused\n",
                cases);
}

/// The label of each step kind, as the baked program spells it, from the
/// tables alone, including the out-of-range forms.
void
TestStepLabels()
{
    _context = "step labels";
    // The rich file's compose step names a group the file does not hold.
    const RigExecWireFile rich = _RichFile();
    CHECK(RigExecFormatStepLabel(rich, _RegionBegin(rich)) == "ComposeSubtree 0");
    CHECK(RigExecFormatStepLabel(rich, _RegionBegin(rich) + 1) == std::to_string(_RegionBegin(rich) + 1));
    const RigExecWireFile volume = _VolumeFile();
    CHECK(RigExecFormatStepLabel(volume, _RegionBegin(volume) + 1) == "VolumePlacements /Rig/Ctl");

    size_t checked = 3;
    const auto label = [&](const char *want,
                           const std::function<void(RigExecWireFile &)>
                               &mutate) {
        RigExecWireFile file = _Copy(rich);
        mutate(file);
        const std::string got = RigExecFormatStepLabel(file, _RegionBegin(file));
        ++checked;
        CHECK(got == want);
        if (got != want) {
            std::printf("  label '%s', expected '%s'\n", got.c_str(), want);
        }
    };
    using fb::StepKind;
    label("VolumePlacements every volume weight", [](RigExecWireFile &f) {
        _RegionStep(f, 0).kind = StepKind::VolumePlacements;
        _RegionStep(f, 0).part = -1;
    });
    const auto group = [](RigExecWireFile &f) {
        f.pose->composeGroups.resize(1);
        f.pose->composeGroups[0].begin = 0;
        f.pose->composeGroups[0].end = 1;
    };
    label("ComposeSubtree /Rig/Ctl", group);
    label("ComposeSubtree 7", [&group](RigExecWireFile &f) {
        group(f);
        _RegionStep(f, 0).object = 7;
    });
    label("Solve /Rig/Ctl",
          [](RigExecWireFile &f) { _RegionStep(f, 0).kind = StepKind::Solve; });
    label("Constraint /Rig/Ctl", [](RigExecWireFile &f) {
        _RegionStep(f, 0).kind = StepKind::Constraint;
    });
    label("CommitApply batch 0", [](RigExecWireFile &f) {
        _RegionStep(f, 0).kind = StepKind::CommitApply;
        f.pose->commits[0].moverPath = 0;
    });
    label("ProviderMatrix /Rig/Ctl final", [](RigExecWireFile &f) {
        _RegionStep(f, 0).kind = StepKind::ProviderMatrix;
        _RegionStep(f, 0).part = 1;
    });
    label("ProviderMatrix /Rig/Ctl base", [](RigExecWireFile &f) {
        _RegionStep(f, 0).kind = StepKind::ProviderMatrix;
        _RegionStep(f, 0).part = 0;
    });
    label("SnapshotFinals every provider", [](RigExecWireFile &f) {
        _RegionStep(f, 0).kind = StepKind::SnapshotFinals;
    });
    label("PoseInterpolator /Rig/Ctl", [](RigExecWireFile &f) {
        _RegionStep(f, 0).kind = StepKind::PoseInterpolator;
    });
    label("WeightPacket /Rig/W", [](RigExecWireFile &f) {
        _RegionStep(f, 0).kind = StepKind::WeightPacket;
    });
    label("RevisionFuse /Rig/Mover", [](RigExecWireFile &f) {
        _RegionStep(f, 0).kind = StepKind::RevisionFuse;
    });
    label("ChainStatus /Rig/Mesh.points", [](RigExecWireFile &f) {
        _RegionStep(f, 0).kind = StepKind::ChainStatus;
    });
    label("Derived /Rig/Mesh.points", [](RigExecWireFile &f) {
        _RegionStep(f, 0).kind = StepKind::Derived;
    });
    label("InfluenceFold 3", [](RigExecWireFile &f) {
        _RegionStep(f, 0).kind = StepKind::InfluenceFold;
        _RegionStep(f, 0).object = 3;
    });
    // A frame record names its provider and the writer it records after;
    // an object past the records and a provider past the slots keep their
    // numbers.
    const auto record = [](RigExecWireFile &f) {
        _RegionStep(f, 0).kind = StepKind::FrameMatrix;
        f.pose->frameRecords = {fb::FrameRecord(0, 0, 0, -1, 1, _pMover)};
    };
    label("FrameMatrix /Rig/Ctl after /Rig/Mover", record);
    label("FrameMatrix record 1", [&record](RigExecWireFile &f) {
        record(f);
        _RegionStep(f, 0).object = 1;
    });
    label("FrameMatrix record -1", [&record](RigExecWireFile &f) {
        record(f);
        _RegionStep(f, 0).object = -1;
    });
    label("FrameMatrix slot 4 after /Rig/Mover",
          [](RigExecWireFile &f) {
              _RegionStep(f, 0).kind = StepKind::FrameMatrix;
              f.pose->frameRecords = {
                  fb::FrameRecord(4, 0, -1, 0, 1, _pMover)};
          });
    std::printf("step labels: %zu checked\n", checked);
}

/// The rich file with a second slot, /Rig/Slot0, both composed by step 0
/// (compose group [0, 2), so PoseFin versions 0 and 1 are the composes);
/// commit 0 writes slot 0's last version, 2, in step 1; and frame record 0
/// of slot 0 after commit 0 reads version 2 in step 2, which declares
/// PoseFin[0] and CommitTable[0] and writes FrameMatrix[0]. \p solver makes
/// commit 0 a solver batch over both slots (versions 2 and 3), step 1 its
/// SolverCommit, and the record a solver's at position 0.
RigExecWireFile
_RecordFile(bool solver = false)
{
    RigExecWireFile f = _RichFileBody();
    _AddSlots(f, 1);
    f.pose->composeGroups.resize(1);
    f.pose->composeGroups[0].begin = 0;
    f.pose->composeGroups[0].end = 2;
    f.steps[0].writes = {fb::SlotRange(fb::SlotDomain::PoseFin, 0, 2)};
    fb::RigExecWireCommit &commit = f.pose->commits[0];
    commit.solverOutput = solver;
    commit.slots = solver ? std::vector<int32_t>{0, 1}
                          : std::vector<int32_t>{0};
    commit.slotWrites = solver ? std::vector<uint32_t>{2, 3}
                               : std::vector<uint32_t>{2};
    fb::RigExecWireStep writer;
    writer.kind = solver ? fb::StepKind::SolverCommit
                         : fb::StepKind::Constraint;
    writer.object = 0;
    writer.reads = {fb::SlotRange(fb::SlotDomain::PoseFin, 0, 1)};
    writer.writes = {fb::SlotRange(fb::SlotDomain::CommitTable, 0, 1)};
    _AppendStep(f, std::move(writer));
    fb::RigExecWireStep frame;
    frame.kind = fb::StepKind::FrameMatrix;
    frame.object = 0;
    frame.reads = {fb::SlotRange(fb::SlotDomain::PoseFin, 0, 1),
                   fb::SlotRange(fb::SlotDomain::CommitTable, 0, 1)};
    frame.writes = {fb::SlotRange(fb::SlotDomain::FrameMatrix, 0, 1)};
    _AppendStep(f, std::move(frame));
    f.pose->frameRecords = {
        fb::FrameRecord(0, 0, solver ? -1 : 0, solver ? 0 : -1, 2, _pCtl)};
    _AddFixtureHeads(f);
    return f;
}

/// Removes step \p step from \p f, which has one cluster and no edges.
void
_DropStep(RigExecWireFile &f, size_t step)
{
    step += _RegionBegin(f);
    f.steps.erase(f.steps.begin() + std::ptrdiff_t(step));
    f.clustering->clusterOf.pop_back();
    std::vector<int32_t> &members = f.clustering->clusters[0].members;
    members.pop_back();
}

/// The record file with the rich revision's transform phase AtPrim at
/// /Rig/Ctl, provider slot 0, record 0 listed for its transform and its one
/// influence (slot 0); its phased input /Rig/Mesh.points and its blend
/// sample both read chain 0's published points (Final). Step 3 folds
/// revision 0, declaring FrameMatrix[0]; step 4, chain 0's status, writes
/// ChainPoints[0]; step 5, the revision's RevisionStatic, declares it.
RigExecWireFile
_PhaseFile()
{
    RigExecWireFile f = _RecordFile();
    fb::RigExecWireRevision &r = f.geometry->chains[0].revisions[0];
    r.binding->transformPhase.kind = uint8_t(fb::ReadPhaseKind::AtPrim);
    r.binding->transformPhase.prim = _pCtl;
    r.transformSlot = 0;
    r.transformRecords = {0};
    r.influenceSlots = {0};
    r.influenceRecords.resize(1);
    r.influenceRecords[0].v = {0};
    RigExecWireReadPhase final;
    final.kind = uint8_t(fb::ReadPhaseKind::Final);
    r.binding->phaseInputs = {_pMeshPoints};
    r.binding->phases = {final};
    RigExecWirePointsBinding published;
    published.inputPath = _pMeshPoints;
    published.phase = final;
    published.candidates = {fb::PointVersion(0, 1)};
    published.finalRead = true;
    published.diagnoseMiss = true;
    r.pointBindings = {published};
    fb::RigExecWireBlendSample &sample = r.blendChannels[0].samples[0];
    sample.pointsPath = _pMeshPoints;
    sample.phase = final;
    published.diagnoseMiss = false;
    sample.pointBinding =
        std::make_unique<RigExecWirePointsBinding>(published);
    fb::RigExecWireStep fold;
    fold.kind = fb::StepKind::InfluenceFold;
    fold.object = 0;
    fold.reads = {fb::SlotRange(fb::SlotDomain::FrameMatrix, 0, 1)};
    fold.writes = {fb::SlotRange(fb::SlotDomain::RevisionTransforms, 0, 1)};
    _AppendStep(f, std::move(fold));
    fb::RigExecWireStep status;
    status.kind = fb::StepKind::ChainStatus;
    status.object = 0;
    status.reads = {fb::SlotRange(fb::SlotDomain::ChainBase, 0, 1)};
    status.writes = {fb::SlotRange(fb::SlotDomain::ChainPoints, 0, 1)};
    _AppendStep(f, std::move(status));
    fb::RigExecWireStep reader;
    reader.kind = fb::StepKind::RevisionStatic;
    reader.object = 0;
    reader.reads = {fb::SlotRange(fb::SlotDomain::RevisionTransforms, 0, 1),
                    fb::SlotRange(fb::SlotDomain::ChainPoints, 0, 1)};
    reader.writes = {fb::SlotRange(fb::SlotDomain::RevisionPacket, 0, 1)};
    _AppendStep(f, std::move(reader));
    return f;
}

bool
_Same(const fb::FrameRecord &a, const fb::FrameRecord &b)
{
    return a.slot() == b.slot() && a.commit() == b.commit() &&
           a.target() == b.target() && a.position() == b.position() &&
           a.version() == b.version() && a.moverLabel() == b.moverLabel();
}

bool
_Same(const RigExecWirePointsBinding &a, const RigExecWirePointsBinding &b)
{
    if (a.inputPath != b.inputPath || a.phase.kind != b.phase.kind ||
        a.phase.prim != b.phase.prim || a.finalRead != b.finalRead ||
        a.diagnoseMiss != b.diagnoseMiss ||
        a.candidates.size() != b.candidates.size()) {
        return false;
    }
    for (size_t k = 0; k < a.candidates.size(); ++k) {
        if (a.candidates[k].chain() != b.candidates[k].chain() ||
            a.candidates[k].version() != b.candidates[k].version()) {
            return false;
        }
    }
    return true;
}

/// Frame records, the record lists of an AtPrim transform phase and point
/// bindings: a constraint's and a solver's record, a revision's lists, a
/// phased input's and a blend sample's binding, a version read and a
/// derived reader are accepted and survive Write -> Open -> Write field for
/// field; each rule refuses its violation with its exact text, naming the
/// step, ahead of the step graph's producer check.
void
TestPhaseTables()
{
    _context = "phase tables";
    std::string why;
    const auto accepted = [&](const char *name, const RigExecWireFile &file) {
        _context = std::string("phase tables: ") + name;
        const bool ok = RigExecFormatValidate(file, &why);
        CHECK(ok);
        if (!ok) {
            std::printf("  %s refused: %s\n", name, why.c_str());
        }
        return ok;
    };
    const RigExecWireFile phase = _PhaseFile();
    if (accepted("phase file", phase)) {
        std::vector<uint8_t> bytes;
        CHECK(_Write(phase, &bytes));
        const auto o = _Open(bytes, &why);
        CHECK(o != nullptr);
        if (o) {
            std::vector<uint8_t> again;
            CHECK(_Write(*o, &again) && again == bytes);
            CHECK(o->pose->frameRecords.size() == 1 &&
                  _Same(o->pose->frameRecords[0],
                        phase.pose->frameRecords[0]));
            const fb::RigExecWireRevision &want =
                phase.geometry->chains[0].revisions[0];
            const fb::RigExecWireRevision &got =
                o->geometry->chains[0].revisions[0];
            CHECK(got.transformRecords == std::vector<uint32_t>({0}));
            CHECK(got.influenceRecords.size() == 1 &&
                  got.influenceRecords[0].v == std::vector<uint32_t>({0}));
            CHECK(got.pointBindings.size() == 1 &&
                  _Same(got.pointBindings[0], want.pointBindings[0]));
            const auto &sample = got.blendChannels[0].samples[0];
            CHECK(sample.pointBinding &&
                  _Same(*sample.pointBinding,
                        *want.blendChannels[0].samples[0].pointBinding));
            CHECK(!o->geometry->chains[0].derived[0].revision->pointBindings
                       .size());
        }
    }
    const RigExecWireFile solver = _RecordFile(true);
    if (accepted("solver record", solver)) {
        std::vector<uint8_t> bytes;
        const auto o = _Write(solver, &bytes) ? _Open(bytes, &why) : nullptr;
        CHECK(o && o->pose->frameRecords.size() == 1 &&
              _Same(o->pose->frameRecords[0], solver.pose->frameRecords[0]));
    }

    using F = RigExecWireFile;
    // A record reading a version its commit read rather than wrote: the
    // compose's, written before the record's step.
    const auto composeVersion = [](F &f) {
        f.pose->frameRecords[0] = fb::FrameRecord(0, 0, 0, -1, 0, _pCtl);
    };
    {
        F file = _RecordFile();
        composeVersion(file);
        accepted("record of a read version", file);
    }
    // A second commit, /Rig/Mover, writing slot 0's last version (2) in
    // step 3, after the record's step; commit 0 writes version 4 before it.
    const auto laterCommit = [](F &f) {
        fb::RigExecWireCommit later;
        later.moverPath = _pMover;
        later.slots = {0};
        later.slotWrites = {2};
        f.pose->commits[0].slotWrites = {4};
        f.pose->commits.push_back(std::move(later));
        fb::RigExecWireStep step;
        step.kind = fb::StepKind::Constraint;
        step.object = 1;
        step.reads = {fb::SlotRange(fb::SlotDomain::PoseFin, 0, 1),
                      fb::SlotRange(fb::SlotDomain::Rest, 0, 1)};
        step.writes = {fb::SlotRange(fb::SlotDomain::CommitTable, 1, 2)};
        _AppendStep(f, std::move(step));
        f.pose->frameRecords[0] = fb::FrameRecord(0, 0, 0, -1, 4, _pCtl);
    };
    {
        F file = _RecordFile();
        laterCommit(file);
        accepted("record before a later commit", file);
    }
    // A non-final version: the binding reads what revision 0's fuse, step
    // 6, left (version 1), through RevisionDone[0] and ChainDirty[0].
    const auto version = [](F &f) {
        fb::RigExecWireRevision &r = f.geometry->chains[0].revisions[0];
        RigExecWireReadPhase atPrim;
        atPrim.kind = uint8_t(fb::ReadPhaseKind::AtPrim);
        atPrim.prim = _pMover;
        r.binding->phases = {atPrim};
        r.pointBindings[0].phase = atPrim;
        r.pointBindings[0].finalRead = false;
        fb::RigExecWireStep fuse;
        fuse.kind = fb::StepKind::RevisionFuse;
        fuse.object = 0;
        fuse.writes = {fb::SlotRange(fb::SlotDomain::RevisionDone, 0, 1),
                       fb::SlotRange(fb::SlotDomain::ChainDirty, 0, 1)};
        _AppendStep(f, std::move(fuse));
        fb::RigExecWireStep reader = _RegionStep(f, 5);
        reader.reads.push_back(
            fb::SlotRange(fb::SlotDomain::RevisionDone, 0, 1));
        reader.reads.push_back(
            fb::SlotRange(fb::SlotDomain::ChainDirty, 0, 1));
        _DropStep(f, 5);
        _AppendStep(f, std::move(reader));
    };
    {
        F file = _PhaseFile();
        version(file);
        accepted("version read", file);
    }
    // A derived target reading its chain's published points.
    const auto derivedReader = [](F &f) {
        fb::RigExecWireRevision &d = *f.geometry->chains[0].derived[0].revision;
        RigExecWireReadPhase final;
        final.kind = uint8_t(fb::ReadPhaseKind::Final);
        d.binding->phaseInputs = {_pMeshPoints};
        d.binding->phases = {final};
        RigExecWirePointsBinding published;
        published.inputPath = _pMeshPoints;
        published.phase = final;
        published.candidates = {fb::PointVersion(0, 1)};
        published.finalRead = true;
        published.diagnoseMiss = true;
        d.pointBindings = {published};
        fb::RigExecWireStep step;
        step.kind = fb::StepKind::Derived;
        step.object = 0;
        step.reads = {fb::SlotRange(fb::SlotDomain::ChainPoints, 0, 1)};
        step.writes = {fb::SlotRange(fb::SlotDomain::DerivedOut, 0, 1)};
        _AppendStep(f, std::move(step));
    };
    {
        F file = _PhaseFile();
        derivedReader(file);
        accepted("derived reader", file);
    }

    int cases = 0;
    const auto expect = [&](const char *name, const std::string &text,
                            const std::function<F()> &make) {
        _context = std::string("phase tables: ") + name;
        ++cases;
        const F file = make();
        const bool ok = RigExecFormatValidate(file, &why);
        CHECK(!ok && why == _RegionDiagnostic(file, text));
        if (ok || why != _RegionDiagnostic(file, text)) {
            std::printf("  got '%s', expected '%s'\n",
                        ok ? "(accepted)" : why.c_str(), text.c_str());
        }
    };
    const auto record = [](const std::function<void(F &)> &mutate,
                           bool solverForm = false) {
        return [mutate, solverForm] {
            F f = _RecordFile(solverForm);
            mutate(f);
            return f;
        };
    };
    const auto phased = [](const std::function<void(F &)> &mutate) {
        return [mutate] {
            F f = _PhaseFile();
            mutate(f);
            return f;
        };
    };
    const std::string frame = "step 2 (FrameMatrix /Rig/Ctl after /Rig/Ctl)";
    const std::string fold = "step 3 (InfluenceFold /Rig/Mover)";
    const std::string reader = "step 5 (RevisionStatic /Rig/Mover)";

    // The records and their FrameMatrix steps.
    expect("version a later commit writes",
           frame + " is bound to PoseFin version 2 of /Rig/Ctl, which step 3 "
                   "(Constraint /Rig/Mover) writes at or after it",
           record([&](F &f) {
               laterCommit(f);
               f.pose->frameRecords[0] =
                   fb::FrameRecord(0, 0, 0, -1, 2, _pCtl);
           }));
    expect("version of another slot",
           frame + " is bound to PoseFin version 1 of /Rig/Slot0, not of the "
                   "provider it records",
           record([](F &f) {
               f.pose->frameRecords[0] =
                   fb::FrameRecord(0, 0, 0, -1, 1, _pCtl);
           }));
    expect("version no step writes",
           frame + " is bound to PoseFin version 3, which no step writes",
           record([](F &f) {
               f.pose->frameRecords[0] =
                   fb::FrameRecord(0, 0, 0, -1, 3, _pCtl);
           }));
    expect("solver position of another slot",
           frame + " reads position 1 of commit 0, which is not the provider "
                   "it records",
           record(
               [](F &f) {
                   f.pose->frameRecords[0] =
                       fb::FrameRecord(0, 0, -1, 1, 2, _pCtl);
               },
               true));
    expect("solver position past the slots",
           frame + " reads position 2 of commit 0, which is not the provider "
                   "it records",
           record(
               [](F &f) {
                   f.pose->frameRecords[0] =
                       fb::FrameRecord(0, 0, -1, 2, 2, _pCtl);
               },
               true));
    expect("solver record with a target",
           frame + " records commit 0, a solver's, at target 0; a solver's "
                   "record has target -1",
           record(
               [](F &f) {
                   f.pose->frameRecords[0] =
                       fb::FrameRecord(0, 0, 0, 0, 2, _pCtl);
               },
               true));
    expect("constraint record with a position",
           frame + " records commit 0, a constraint's, at target 0 and "
                   "position 0; a constraint's record has a target and "
                   "position -1",
           record([](F &f) {
               f.pose->frameRecords[0] = fb::FrameRecord(0, 0, 0, 0, 2, _pCtl);
           }));
    expect("record with no step",
           "frame record 0 of /Rig/Ctl after /Rig/Ctl has no FrameMatrix step",
           record([](F &f) { _DropStep(f, 2); }));
    expect("two steps for one record",
           "step 3 (FrameMatrix /Rig/Ctl after /Rig/Ctl) evaluates the same "
           "frame record as " + frame,
           record([](F &f) { _AppendStep(f, _RegionStep(f, 2)); }));
    expect("object past the records",
           "step 2 (FrameMatrix record 1) names frame record 1 of 1",
           record([](F &f) { _RegionStep(f, 2).object = 1; }));
    // A record's own fields out of range: named by its FrameMatrix step,
    // and by the record's row only when no step evaluates it.
    expect("record past the slots",
           "step 2 (FrameMatrix slot 2 after /Rig/Ctl) names slot 2 of 2",
           record([](F &f) {
               f.pose->frameRecords[0] = fb::FrameRecord(2, 0, 0, -1, 2, _pCtl);
           }));
    expect("record commit past the commits",
           frame + " names commit 5 of " +
               std::to_string(_RecordFile(false).pose->commits.size()),
           record([](F &f) {
               f.pose->frameRecords[0] = fb::FrameRecord(0, 5, 0, -1, 2, _pCtl);
           }));
    expect("record version past the pool",
           frame + " is bound to PoseFin version 4, which no step writes",
           record([](F &f) {
               f.pose->frameRecords[0] = fb::FrameRecord(0, 0, 0, -1, 4, _pCtl);
           }));
    expect("record mover label past the paths",
           "step 2 (FrameMatrix /Rig/Ctl after ) names mover label path id "
           "999 of " +
               std::to_string(_RecordFile(false).paths.size()),
           record([](F &f) {
               f.pose->frameRecords[0] = fb::FrameRecord(0, 0, 0, -1, 2, 999);
           }));
    expect("record with no step past the slots",
           "pose.frame_records[0].slot: 2 out of range (2)", record([](F &f) {
               _DropStep(f, 2);
               f.pose->frameRecords[0] = fb::FrameRecord(2, 0, 0, -1, 2, _pCtl);
           }));
    expect("no CommitTable read",
           frame + " does not declare CommitTable[0]", record([](F &f) {
               _RegionStep(f, 2).reads = {
                   fb::SlotRange(fb::SlotDomain::PoseFin, 0, 1)};
           }));
    expect("no PoseFin read", frame + " does not declare PoseFin[0]",
           record([](F &f) {
               _RegionStep(f, 2).reads = {
                   fb::SlotRange(fb::SlotDomain::CommitTable, 0, 1)};
           }));
    // Ahead of its commit: the version is the compose's, so only the
    // commit's table is unwritten; with the commit's step gone, no step
    // writes it. Either way the producer check would name an unproduced
    // CommitTable read instead.
    expect("ahead of its commit",
           "step 1 (FrameMatrix /Rig/Ctl after /Rig/Ctl) reads the exit of "
           "commit 0, which step 2 (Constraint /Rig/Ctl) writes at or after "
           "it",
           record([&](F &f) {
               composeVersion(f);
               std::swap(_RegionStep(f, 1), _RegionStep(f, 2));
           }));
    expect("commit with no step",
           "step 1 (FrameMatrix /Rig/Ctl after /Rig/Ctl) reads the exit of "
           "commit 0, which no step writes",
           record([&](F &f) {
               composeVersion(f);
               _DropStep(f, 1);
           }));
    // A retired kind is named as one ahead of the frame-record rule a
    // commit missing its first step would break.
    expect("a commit step of the retired kind",
           "step 1 is a retired SnapshotFinals step", record([](F &f) {
               _RegionStep(f, 1).kind = fb::StepKind::SnapshotFinals;
           }));
    expect("another kind writing FrameMatrix",
           "step 1 (Constraint /Rig/Ctl) writes FrameMatrix, which only a "
           "FrameMatrix step writes",
           record([](F &f) {
               _RegionStep(f, 1).writes.push_back(
                   fb::SlotRange(fb::SlotDomain::FrameMatrix, 0, 1));
           }));
    expect("FrameMatrix writing more than its record",
           frame + " writes other than FrameMatrix[0]", record([](F &f) {
               _RegionStep(f, 2).writes = {
                   fb::SlotRange(fb::SlotDomain::FrameMatrix, 0, 2)};
           }));
    expect("two composes of one version",
           "step 3 (ComposeSubtree /Rig/Ctl) writes PoseFin version 0, which "
           "step 0 (ComposeSubtree /Rig/Ctl) writes too",
           record([](F &f) {
               fb::RigExecWireStep again = _RegionStep(f, 0);
               again.overrideInputs.clear();
               _AppendStep(f, std::move(again));
           }));

    // The record lists an AtPrim transform phase folds.
    expect("transform record past the records",
           fold + " reads frame record 1 of 1", phased([](F &f) {
               f.geometry->chains[0].revisions[0].transformRecords = {1};
           }));
    expect("influence lists of another count",
           fold + " holds 2 influence record lists for 1 influence slots",
           phased([](F &f) {
               f.geometry->chains[0].revisions[0].influenceRecords.resize(2);
           }));
    expect("record of another provider",
           fold + " reads frame record 0 of /Rig/Ctl for its transform, "
                  "whose provider is /Rig/Slot0",
           phased([](F &f) {
               f.geometry->chains[0].revisions[0].transformSlot = 1;
           }));
    expect("influence record of another provider",
           fold + " reads frame record 0 of /Rig/Ctl for influence 0, whose "
                  "provider is /Rig/Slot0",
           phased([](F &f) {
               f.geometry->chains[0].revisions[0].influenceSlots = {1};
           }));
    expect("records without an AtPrim phase",
           fold + " holds frame records for a transform phase that is not "
                  "AtPrim",
           phased([](F &f) {
               f.geometry->chains[0].revisions[0].binding->transformPhase.kind =
                   uint8_t(fb::ReadPhaseKind::Preceding);
           }));
    expect("fold without its FrameMatrix read",
           fold + " does not declare FrameMatrix[0]",
           phased([](F &f) { _RegionStep(f, 3).reads.clear(); }));
    expect("records with no fold",
           "geometry.chains[0].revisions[0]: frame records with no "
           "InfluenceFold step to read them",
           phased([](F &f) { _RegionStep(f, 3).kind = fb::StepKind::RevisionChunk; }));

    // Point bindings.
    expect("candidate chain past the chains",
           reader + " binds /Rig/Mesh.points to chain 1 of 1",
           phased([](F &f) {
               f.geometry->chains[0].revisions[0].pointBindings[0].candidates =
                   {fb::PointVersion(1, 1)};
           }));
    expect("candidate version past the revisions",
           reader + " binds /Rig/Mesh.points to version 2 of chain 0, past "
                    "its last version 1",
           phased([](F &f) {
               f.geometry->chains[0].revisions[0].pointBindings[0].candidates =
                   {fb::PointVersion(0, 2)};
           }));
    expect("negative candidate version",
           reader + " binds /Rig/Mesh.points to version -1 of chain 0, which "
                    "is no version",
           phased([](F &f) {
               f.geometry->chains[0].revisions[0].pointBindings[0].candidates =
                   {fb::PointVersion(0, -1)};
           }));
    expect("final read of an earlier version",
           reader + " binds /Rig/Mesh.points as a final read of version 0 of "
                    "chain 0, not its last version 1",
           phased([](F &f) {
               f.geometry->chains[0].revisions[0].pointBindings[0].candidates =
                   {fb::PointVersion(0, 0)};
           }));
    expect("final read of two candidates",
           reader + " binds /Rig/Mesh.points as a final read of 2 "
                    "candidates; a final read has one",
           phased([](F &f) {
               f.geometry->chains[0].revisions[0].pointBindings[0].candidates =
                   {fb::PointVersion(0, 1), fb::PointVersion(0, 1)};
           }));
    expect("bindings of another count",
           reader + " holds 2 point bindings for 1 phased inputs",
           phased([](F &f) {
               auto &bindings =
                   f.geometry->chains[0].revisions[0].pointBindings;
               bindings.push_back(bindings[0]);
           }));
    expect("binding of another input",
           reader + " binds /Rig/Ctl.tx for phased input 0, which is "
                    "/Rig/Mesh.points",
           phased([](F &f) {
               f.geometry->chains[0].revisions[0].pointBindings[0].inputPath =
                   _pCtlTx;
           }));
    expect("binding at another phase",
           reader + " binds /Rig/Mesh.points at a phase other than the one "
                    "it declares",
           phased([](F &f) {
               f.geometry->chains[0].revisions[0].pointBindings[0].phase.kind =
                   uint8_t(fb::ReadPhaseKind::Preceding);
           }));
    expect("reader without its ChainPoints read",
           reader + " does not declare ChainPoints[0] for /Rig/Mesh.points",
           phased([](F &f) {
               _RegionStep(f, 5).reads = {fb::SlotRange(
                   fb::SlotDomain::RevisionTransforms, 0, 1)};
           }));
    expect("version reader without its ChainDirty read",
           "step 6 (RevisionStatic /Rig/Mover) does not declare "
           "ChainDirty[0] for /Rig/Mesh.points",
           phased([&](F &f) {
               version(f);
               _RegionStep(f, 6).reads.pop_back();
           }));
    expect("derived reader without its ChainPoints read",
           "step 6 (Derived /Rig/Mesh.points) does not declare ChainPoints[0] "
           "for /Rig/Mesh.points",
           phased([&](F &f) {
               derivedReader(f);
               _RegionStep(f, 6).reads.clear();
           }));
    expect("bindings with no reader",
           "geometry.chains[0].revisions[0]: point bindings with no "
           "RevisionStatic step to read them",
           phased([](F &f) { _RegionStep(f, 5).kind = fb::StepKind::RevisionChunk; }));
    // A blend sample's binding.
    const std::string sample = "blend_channels[0].samples[0]";
    expect("sample binding on a base-phase sample",
           reader + " binds " + sample + ", which reads no phased points",
           phased([](F &f) {
               f.geometry->chains[0].revisions[0].blendChannels[0].samples[0]
                   .phase = RigExecWireReadPhase();
           }));
    expect("sample binding on a blend shape",
           reader + " binds " + sample + ", which reads no phased points",
           phased([](F &f) {
               f.geometry->chains[0].revisions[0].blendChannels[0].samples[0]
                   .blendShape = _pMesh;
           }));
    expect("phased sample without its binding",
           reader + " leaves " + sample + " unbound, though it reads phased "
                                          "points",
           phased([](F &f) {
               f.geometry->chains[0].revisions[0].blendChannels[0].samples[0]
                   .pointBinding.reset();
           }));
    expect("sample binding of another input",
           reader + " binds " + sample + " to /Rig/Ctl.tx, not to its points "
                                         "/Rig/Mesh.points",
           phased([](F &f) {
               f.geometry->chains[0].revisions[0].blendChannels[0].samples[0]
                   .pointBinding->inputPath = _pCtlTx;
           }));
    expect("sample binding with a miss diagnostic",
           reader + " binds " + sample + " with a miss diagnostic, which a "
                                         "blend sample never emits",
           phased([](F &f) {
               f.geometry->chains[0].revisions[0].blendChannels[0].samples[0]
                   .pointBinding->diagnoseMiss = true;
           }));
    expect("sample candidate past the revisions",
           reader + " binds /Rig/Mesh.points to version 3 of chain 0, past "
                    "its last version 1",
           phased([](F &f) {
               f.geometry->chains[0].revisions[0].blendChannels[0].samples[0]
                   .pointBinding->candidates = {fb::PointVersion(0, 3)};
           }));

    // Open refuses with the same words.
    {
        _context = "phase tables open: version a later commit writes";
        ++cases;
        F file = _RecordFile();
        laterCommit(file);
        file.pose->frameRecords[0] = fb::FrameRecord(0, 0, 0, -1, 2, _pCtl);
        CHECK(!_Open(_PackUnchecked(file), &why) &&
              why == "invalid .rigexec: " + _RegionDiagnostic(file, frame +
                         " is bound to PoseFin version 2 of /Rig/Ctl, which "
                         "step 3 (Constraint /Rig/Mover) writes at or after "
                         "it"));
    }
    std::printf("phase tables: records, lists and bindings accepted and "
                "round-tripped, %d violations refused\n",
                cases);
}

/// Every field of two skin layouts, weights bit for bit.
bool
_Same(const fb::RigExecWireSkinTopology &a,
      const fb::RigExecWireSkinTopology &b)
{
    return a.elementSize == b.elementSize && a.pointCount == b.pointCount &&
           a.influenceCount == b.influenceCount &&
           a.validated == b.validated && a.counts8 == b.counts8 &&
           a.counts16 == b.counts16 && a.indexWidth == b.indexWidth &&
           a.indices8 == b.indices8 && a.indices16 == b.indices16 &&
           a.indices32 == b.indices32 && _Same(a.weights, b.weights) &&
           a.raw == b.raw && a.rawIndices == b.rawIndices &&
           _Same(a.rawWeights, b.rawWeights);
}

/// The evaluator's layout rules (RigExecResolveSkinTopology), from their
/// definition: rows of an element size of at least 1, at least one
/// influence, every index in [0, influenceCount) and every weight finite
/// and not negative.
bool
_EvaluatorValidates(const std::vector<int32_t> &indices,
                    const std::vector<float> &weights, int32_t elementSize,
                    uint64_t influenceCount)
{
    if (elementSize < 1 || indices.size() != weights.size() ||
        indices.size() % size_t(elementSize) != 0 || influenceCount == 0) {
        return false;
    }
    for (size_t k = 0; k < indices.size(); ++k) {
        if (indices[k] < 0 || uint64_t(indices[k]) >= influenceCount ||
            !std::isfinite(weights[k]) || weights[k] < 0.0f) {
            return false;
        }
    }
    return true;
}

/// Sparse skin topologies: lossless encoding of dense rows with interior
/// zeros of either sign, negative weights and non-finite weights, each row
/// up to its trailing (0, +0) run; the counts and index vectors at their
/// size boundaries; refusal of layouts that are not rectangular; every
/// encoded layout validates, survives Write -> Open and expands to the
/// layout it was written from, bit for bit; a kept trailing (0, +0) entry
/// is refused, and a kept trailing (0, -0), a mid-row (0, +0) and a
/// (0, NaN) are accepted.
void
TestSparseTopology()
{
    _context = "sparse topology";
    std::string why;
    size_t encoded = 0, unvalidated = 0, passing = 0;

    // Encodes \p indices x \p weights over 7 influences, validated as the
    // evaluator would leave it, checks the encoding validates inside the
    // rich file, survives Write -> Open, re-encodes identically from its
    // expansion, and expands to the layout itself bit for bit. The same
    // encoding with its validated flag flipped is refused.
    const auto encode = [&](const std::string &label,
                            const std::vector<int32_t> &indices,
                            const std::vector<float> &weights,
                            int32_t elementSize, uint64_t pointCount) {
        _context = "sparse topology: " + label;
        const bool validated =
            _EvaluatorValidates(indices, weights, elementSize, 7);
        fb::RigExecWireSkinTopology sparse;
        const bool ok = RigExecFormatSparseTopology(
            indices, weights, elementSize, pointCount, 7, validated, &sparse,
            &why);
        CHECK(ok);
        if (!ok) {
            std::printf("  refused: %s\n", why.c_str());
            return sparse;
        }
        ++encoded;
        CHECK(sparse.elementSize == elementSize &&
              sparse.pointCount == pointCount &&
              sparse.influenceCount == 7 && sparse.validated == validated &&
              !sparse.raw && sparse.rawIndices.empty() &&
              sparse.rawWeights.empty());
        const std::vector<int32_t> &wantIndices = indices;
        const std::vector<float> &wantWeights = weights;
        std::vector<int32_t> gotIndices;
        std::vector<float> gotWeights;
        RigExecFormatExpandTopology(sparse, &gotIndices, &gotWeights);
        CHECK(gotIndices == wantIndices && _Same(gotWeights, wantWeights));

        fb::RigExecWireSkinTopology again;
        CHECK(RigExecFormatSparseTopology(gotIndices, gotWeights,
                                          elementSize, pointCount, 7,
                                          validated, &again, &why) &&
              _Same(again, sparse));
        // The writer picks the sparse form for every layout it holds.
        CHECK(RigExecFormatTopology(indices, weights, elementSize,
                                    pointCount, 7, validated, &again,
                                    &why) &&
              _Same(again, sparse));

        RigExecWireFile file = _RichFile();
        file.geometry->chains[0].revisions[0].topology =
            std::make_unique<fb::RigExecWireSkinTopology>(sparse);
        why.clear();
        CHECK(RigExecFormatValidate(file, &why));
        if (!why.empty()) {
            std::printf("  file refused: %s\n", why.c_str());
        }
        {
            RigExecWireFile flipped = _RichFile();
            flipped.geometry->chains[0].revisions[0].topology =
                std::make_unique<fb::RigExecWireSkinTopology>(sparse);
            flipped.geometry->chains[0].revisions[0].topology->validated =
                !validated;
            CHECK(!RigExecFormatValidate(flipped, &why) &&
                  _Contains(why,
                            std::string("geometry.chains[0].revisions[0]."
                                        "topology: ") +
                                (validated ? "not validated, but its layout "
                                             "passes the evaluator's rules"
                                           : "validated")));
            ++(validated ? passing : unvalidated);
        }
        std::vector<uint8_t> bytes;
        std::unique_ptr<RigExecWireFile> o;
        if (_Write(file, &bytes)) {
            o = _Open(bytes, &why);
        }
        CHECK(o && o->geometry->chains[0].revisions[0].topology);
        if (o && o->geometry->chains[0].revisions[0].topology) {
            const fb::RigExecWireSkinTopology &read =
                *o->geometry->chains[0].revisions[0].topology;
            CHECK(_Same(read, sparse));
            RigExecFormatExpandTopology(read, &gotIndices, &gotWeights);
            CHECK(gotIndices == wantIndices &&
                  _Same(gotWeights, wantWeights));
        }
        return sparse;
    };

    // Each row keeps its entries up to its trailing (0, +0) run: interior
    // (0, +0) and (0, -0) entries, a trailing (0, -0), a zero weight at
    // another index, a negative weight, a denormal and a NaN payload at
    // index 0 are kept, in order.
    const float denormal = _F(0x00000001u), nan = _F(0x7fc12345u);
    const std::vector<int32_t> indices = {2, 0, 3, 0, 0,   //
                                          0, 5, 0, 6, 0,   //
                                          0, 0, 0, 0, 0,   //
                                          1, 0, 0, 0, 0};
    const std::vector<float> weights = {
        0.5f,  0.0f, -0.25f, -0.0f, denormal,  //
        0.0f,  0.0f, 1.0f,   -0.0f, nan,       //
        -0.0f, 0.0f, -0.0f,  0.0f,  0.0f,      //
        0.75f, 0.0f, 0.0f,   0.0f,  0.0f};
    const fb::RigExecWireSkinTopology rows =
        encode("interior zeros", indices, weights, 5, 4);
    CHECK(rows.counts8 == std::vector<uint8_t>({5, 5, 3, 1}) &&
          rows.counts16.empty());
    CHECK(rows.indexWidth == 1 &&
          rows.indices8 == std::vector<uint8_t>({2, 0, 3, 0, 0,  //
                                                 0, 5, 0, 6, 0,  //
                                                 0, 0, 0,        //
                                                 1}) &&
          rows.indices16.empty() && rows.indices32.empty());
    CHECK(_Same(rows.weights,
                std::vector<float>({0.5f, 0.0f, -0.25f, -0.0f, denormal,  //
                                    0.0f, 0.0f, 1.0f, -0.0f, nan,         //
                                    -0.0f, 0.0f, -0.0f,                   //
                                    0.75f})));
    std::vector<int32_t> denseIndices;
    std::vector<float> denseWeights;
    RigExecFormatExpandTopology(rows, &denseIndices, &denseWeights);
    _context = "sparse topology: interior zeros";
    CHECK(denseIndices == indices && _Same(denseWeights, weights));

    // The counts vector by element size.
    for (const int32_t width : {255, 256}) {
        std::vector<int32_t> full(static_cast<size_t>(width), 0);
        for (int32_t e = 0; e < width; ++e) {
            full[size_t(e)] = e % 200 + 1;
        }
        const fb::RigExecWireSkinTopology counted = encode(
            "element size " + std::to_string(width), full,
            std::vector<float>(size_t(width), 0.25f), width, 1);
        CHECK(width == 255
                  ? counted.counts8 == std::vector<uint8_t>({255}) &&
                        counted.counts16.empty()
                  : counted.counts16 == std::vector<uint16_t>({256}) &&
                        counted.counts8.empty());
    }

    // The index vector by the largest kept index, which a zero weight at
    // that index still decides; any negative index takes 32 bits.
    const std::pair<int32_t, uint8_t> widths[] = {
        {255, 1}, {256, 2}, {65535, 2}, {65536, 4}};
    for (const auto &[largest, width] : widths) {
        const fb::RigExecWireSkinTopology wide =
            encode("largest index " + std::to_string(largest),
                   {1, largest, 0, 0}, {0.5f, 0.0f, 0.0f, 0.0f}, 2, 2);
        CHECK(wide.indexWidth == width &&
              wide.indices8.size() == (width == 1 ? 2u : 0u) &&
              wide.indices16.size() == (width == 2 ? 2u : 0u) &&
              wide.indices32.size() == (width == 4 ? 2u : 0u));
    }
    const fb::RigExecWireSkinTopology negative = encode(
        "negative index", {3, -1, -2, 0}, {0.25f, 0.5f, 0.0f, 0.0f}, 2, 2);
    CHECK(negative.indexWidth == 4 &&
          negative.indices32 == std::vector<int32_t>({3, -1, -2}) &&
          negative.indices8.empty() && negative.indices16.empty());

    // Zero-width rows and no rows.
    const fb::RigExecWireSkinTopology zeroWidth =
        encode("element size 0", {}, {}, 0, 3);
    CHECK(zeroWidth.counts8 == std::vector<uint8_t>({0, 0, 0}) &&
          zeroWidth.weights.empty());
    encode("no points", {}, {}, 4, 0);

    // Layouts that are not point_count rows of element_size: refused with a
    // reason, the output untouched.
    const auto refuse = [&](const char *label,
                            const std::vector<int32_t> &badIndices,
                            const std::vector<float> &badWeights,
                            int32_t elementSize, uint64_t pointCount,
                            const char *part) {
        _context = std::string("sparse topology refuses: ") + label;
        fb::RigExecWireSkinTopology out;
        out.elementSize = 12345;
        CHECK(!RigExecFormatSparseTopology(badIndices, badWeights,
                                           elementSize, pointCount, 0, false,
                                           &out, &why) &&
              _Contains(why, part) && out.elementSize == 12345);
    };
    refuse("ragged rows", std::vector<int32_t>(7, 1),
           std::vector<float>(7, 1.0f), 2, 3, "not 3 points of 2 entries");
    refuse("weights short", std::vector<int32_t>(8, 1),
           std::vector<float>(7, 1.0f), 2, 4, "not 4 points of 2 entries");
    refuse("point count", std::vector<int32_t>(8, 1),
           std::vector<float>(8, 1.0f), 2, 3, "not 3 points of 2 entries");
    refuse("entries of zero width", {1}, {1.0f}, 0, 0,
           "not 0 points of 0 entries");
    refuse("negative element size", {}, {}, -1, 0, "element size -1");
    refuse("element size past 65535", {}, {}, 65536, 0,
           "element size 65536");

    // The validator refuses a hand-made row whose last kept entry is the
    // (0, +0) the sparse form drops, at any index width, and accepts a
    // trailing (0, -0) and (0, NaN), which the writer keeps, and a (0, +0)
    // before another entry.
    const auto handMade = [&](uint8_t width, std::vector<uint32_t> kept,
                              float weight) {
        fb::RigExecWireSkinTopology t;
        t.elementSize = 2;
        t.pointCount = 1;
        t.counts8 = {2};
        t.indexWidth = width;
        for (const uint32_t index : kept) {
            if (width == 1) {
                t.indices8.push_back(uint8_t(index));
            } else if (width == 2) {
                t.indices16.push_back(uint16_t(index));
            } else {
                t.indices32.push_back(int32_t(index));
            }
        }
        t.weights = kept[0] == 0 ? std::vector<float>{weight, 0.5f}
                                 : std::vector<float>{0.5f, weight};
        RigExecWireFile file = _RichFile();
        file.geometry->chains[0].revisions[0].topology =
            std::make_unique<fb::RigExecWireSkinTopology>(t);
        return file;
    };
    size_t refusedTrailing = 0, acceptedZeros = 0;
    for (const uint8_t width : {uint8_t(1), uint8_t(2), uint8_t(4)}) {
        _context = "sparse topology: kept trailing (0, +0) at width " +
                   std::to_string(width);
        CHECK(!RigExecFormatValidate(handMade(width, {1, 0}, 0.0f), &why) &&
              why == "geometry.chains[0].revisions[0].topology: point 0 "
                     "keeps a trailing (0, +0) entry, which the sparse "
                     "form drops");
        ++refusedTrailing;
        for (const float weight : {-0.0f, nan}) {
            _context = "sparse topology: kept trailing (0, " +
                       std::string(std::isnan(weight) ? "NaN" : "-0") +
                       ") at width " + std::to_string(width);
            CHECK(RigExecFormatValidate(handMade(width, {1, 0}, weight),
                                        &why));
            ++acceptedZeros;
        }
        _context = "sparse topology: kept mid-row (0, +0) at width " +
                   std::to_string(width);
        CHECK(RigExecFormatValidate(handMade(width, {0, 1}, 0.0f), &why));
        ++acceptedZeros;
    }
    std::printf("sparse topology: %zu layouts encoded losslessly, "
                "validated and expanded, each refused with its validated "
                "flag flipped (%zu the evaluator's rules fail, %zu they "
                "pass); 6 malformed layouts and %zu kept trailing (0, +0) "
                "entries refused, %zu kept (0, -0), (0, NaN) and mid-row "
                "(0, +0) entries accepted\n",
                encoded, unvalidated, passing, refusedTrailing,
                acceptedZeros);
    CHECK(encoded == 10 && unvalidated == 9 && passing == 1 &&
          refusedTrailing == 3 && acceptedZeros == 9);
}

/// Raw skin layouts: RigExecFormatTopology stores verbatim every layout the
/// sparse form cannot hold -- indices and weights of different lengths,
/// entries short of a whole row, entries at element size 0, a negative
/// element size with and without entries, and an element size past 65535
/// over a partial row, no entries and one row, the last both passing and
/// failing the evaluator's rules -- with the evaluator's point count and
/// validation. Each validates inside the rich file, survives Write -> Open
/// bit for bit (a -0 and a NaN payload among the weights), expands to its
/// own arrays and is refused with its validated flag flipped. The
/// validator refuses each way a layout breaks its form or carries a
/// validated flag the evaluator's rules would not give it, in either form
/// and as a partition layout.
void
TestRawTopology()
{
    _context = "raw topology";
    std::string why;
    size_t stored = 0, unvalidated = 0, passing = 0;
    const float nan = _F(0x7fc12345u);
    const std::string row = "geometry.chains[0].revisions[0].topology";

    // The point count the evaluator gives a layout, from its definition.
    const auto pointsOf = [](const std::vector<int32_t> &indices,
                             const std::vector<float> &weights,
                             int32_t elementSize) -> uint64_t {
        return elementSize >= 1 && indices.size() == weights.size() &&
                       indices.size() % size_t(elementSize) == 0
                   ? indices.size() / size_t(elementSize)
                   : 0;
    };
    const auto store = [&](const std::string &label,
                           const std::vector<int32_t> &indices,
                           const std::vector<float> &weights,
                           int32_t elementSize, uint64_t influences) {
        _context = "raw topology: " + label;
        const uint64_t points = pointsOf(indices, weights, elementSize);
        const bool validated =
            _EvaluatorValidates(indices, weights, elementSize, influences);
        fb::RigExecWireSkinTopology raw;
        const bool ok =
            RigExecFormatTopology(indices, weights, elementSize, points,
                                  influences, validated, &raw, &why);
        CHECK(ok);
        if (!ok) {
            std::printf("  refused: %s\n", why.c_str());
            return raw;
        }
        CHECK(raw.raw && raw.rawIndices == indices &&
              _Same(raw.rawWeights, weights) &&
              raw.elementSize == elementSize && raw.pointCount == points &&
              raw.influenceCount == influences &&
              raw.validated == validated);
        CHECK(raw.indexWidth == 0 && raw.counts8.empty() &&
              raw.counts16.empty() && raw.indices8.empty() &&
              raw.indices16.empty() && raw.indices32.empty() &&
              raw.weights.empty());
        // The sparse form cannot hold it, which is why it is raw.
        fb::RigExecWireSkinTopology sparse;
        CHECK(!RigExecFormatSparseTopology(indices, weights, elementSize,
                                           points, influences, validated,
                                           &sparse, &why));
        std::vector<int32_t> gotIndices;
        std::vector<float> gotWeights;
        RigExecFormatExpandTopology(raw, &gotIndices, &gotWeights);
        CHECK(gotIndices == indices && _Same(gotWeights, weights));
        fb::RigExecWireSkinTopology again;
        CHECK(RigExecFormatTopology(gotIndices, gotWeights, elementSize,
                                    points, influences, validated, &again,
                                    &why) &&
              _Same(again, raw));
        // A point count the evaluator would not give it is refused, the
        // output untouched.
        again.elementSize = 12345;
        CHECK(!RigExecFormatTopology(indices, weights, elementSize,
                                     points + 1, influences, validated,
                                     &again, &why) &&
              _Contains(why, "hold " + std::to_string(points) +
                                 " point(s), not " +
                                 std::to_string(points + 1)) &&
              again.elementSize == 12345);

        RigExecWireFile file = _RichFile();
        file.geometry->chains[0].revisions[0].topology =
            std::make_unique<fb::RigExecWireSkinTopology>(raw);
        why.clear();
        CHECK(RigExecFormatValidate(file, &why));
        if (!why.empty()) {
            std::printf("  file refused: %s\n", why.c_str());
        }
        {
            RigExecWireFile flipped = _RichFile();
            flipped.geometry->chains[0].revisions[0].topology =
                std::make_unique<fb::RigExecWireSkinTopology>(raw);
            flipped.geometry->chains[0].revisions[0].topology->validated =
                !validated;
            CHECK(!RigExecFormatValidate(flipped, &why) &&
                  _Contains(why, row + ": " +
                                     (validated
                                          ? "not validated, but its layout "
                                            "passes the evaluator's rules"
                                          : "validated")));
            ++(validated ? passing : unvalidated);
        }
        std::vector<uint8_t> bytes;
        std::unique_ptr<RigExecWireFile> o;
        if (_Write(file, &bytes, &why)) {
            o = _Open(bytes, &why);
        }
        CHECK(o && o->geometry->chains[0].revisions[0].topology);
        if (o && o->geometry->chains[0].revisions[0].topology) {
            const fb::RigExecWireSkinTopology &read =
                *o->geometry->chains[0].revisions[0].topology;
            CHECK(_Same(read, raw));
            RigExecFormatExpandTopology(read, &gotIndices, &gotWeights);
            CHECK(gotIndices == indices && _Same(gotWeights, weights));
            std::vector<uint8_t> rewritten;
            CHECK(_Write(*o, &rewritten) && rewritten == bytes);
        }
        ++stored;
        return raw;
    };

    const fb::RigExecWireSkinTopology odd =
        store("not rows", {1, 0, 0, 1, 0}, {0.5f, 0.0f, 1.0f, nan, -0.0f}, 2,
              2);
    store("weights short", {1, 0, 2}, {0.5f, -0.0f}, 1, 3);
    store("entries at element size 0", {2}, {0.25f}, 0, 3);
    store("negative element size", {0}, {1.0f}, -1, 1);
    store("negative element size, no entries", {}, {}, -1, 1);
    store("element size 70000, a partial row", {0, 1, 1, 0, 1},
          {1.0f, 0.5f, 0.25f, 0.0f, 1.0f}, 70000, 2);
    const fb::RigExecWireSkinTopology wideEmpty =
        store("element size 65536, no entries", {}, {}, 65536, 2);
    CHECK(wideEmpty.validated && wideEmpty.pointCount == 0);
    std::vector<int32_t> wideIndices(70000, 1);
    std::vector<float> wideWeights(70000, 0.5f);
    wideWeights[0] = -0.0f;
    const fb::RigExecWireSkinTopology wideRow = store(
        "element size 70000, one row", wideIndices, wideWeights, 70000, 2);
    CHECK(wideRow.validated && wideRow.pointCount == 1);
    std::vector<int32_t> pastIndices = wideIndices;
    pastIndices[0] = 2;
    const fb::RigExecWireSkinTopology wideRowPast =
        store("element size 70000, one row past the influences",
              pastIndices, wideWeights, 70000, 2);
    CHECK(!wideRowPast.validated && wideRowPast.pointCount == 1);
    CHECK(!odd.validated && odd.pointCount == 0);

    // Refusals, each with its exact message, on the rich file's topology,
    // or on a partition layout beside it.
    int refused = 0;
    const auto refuse = [&](const char *label,
                            const fb::RigExecWireSkinTopology &t,
                            const std::string &expected,
                            bool partition = false) {
        _context = std::string("raw topology refuses: ") + label;
        RigExecWireFile file = _RichFile();
        fb::RigExecWireRevision &revision =
            file.geometry->chains[0].revisions[0];
        if (partition) {
            revision.partitionSameAsTopology = false;
            revision.partitionTopology =
                std::make_unique<fb::RigExecWireSkinTopology>(t);
        } else {
            revision.topology =
                std::make_unique<fb::RigExecWireSkinTopology>(t);
        }
        why.clear();
        const bool ok = RigExecFormatValidate(file, &why);
        CHECK(!ok && why == expected);
        if (ok || why != expected) {
            std::printf("  got '%s', expected '%s'\n",
                        ok ? "(accepted)" : why.c_str(), expected.c_str());
        }
        // Open says the same through the bytes.
        std::string opened;
        CHECK(!_Open(_PackUnchecked(file), &opened) &&
              opened == "invalid .rigexec: " + expected);
        ++refused;
    };
    const auto edited = [](fb::RigExecWireSkinTopology t,
                           const std::function<void(
                               fb::RigExecWireSkinTopology &)> &edit) {
        edit(t);
        return t;
    };
    using T = fb::RigExecWireSkinTopology;
    const std::string sparseVectors =
        row + ": a raw layout with sparse vectors or an index_width";
    refuse("raw with counts",
           edited(odd, [](T &t) { t.counts8 = {1}; }), sparseVectors);
    refuse("raw with kept weights",
           edited(odd, [](T &t) { t.weights = {0.5f}; }), sparseVectors);
    refuse("raw with indices",
           edited(odd, [](T &t) { t.indices32 = {1}; }), sparseVectors);
    refuse("raw with an index width",
           edited(odd, [](T &t) { t.indexWidth = 1; }), sparseVectors);
    refuse("raw rows the sparse form holds", edited(odd, [](T &t) {
               t.rawIndices.pop_back();
               t.rawWeights.pop_back();
           }),
           row + ": a raw layout of 4 indices and 4 weights at element_size "
                 "2, which the sparse form holds");
    refuse("raw and empty at element size 0", edited(odd, [](T &t) {
               t.elementSize = 0;
               t.rawIndices.clear();
               t.rawWeights.clear();
           }),
           row + ": a raw layout of 0 indices and 0 weights at element_size "
                 "0, which the sparse form holds");
    refuse("raw point count",
           edited(odd, [](T &t) { t.pointCount = 2; }),
           row + ": point_count 2, but its raw layout holds 0 point(s)");
    refuse("raw rows, point count",
           edited(wideRow, [](T &t) { t.pointCount = 2; }),
           row + ": point_count 2, but its raw layout holds 1 point(s)");
    refuse("raw validated, not rows",
           edited(odd, [](T &t) { t.validated = true; }),
           row + ": validated, but its raw layout is not rows of "
                 "element_size");
    refuse("raw validated without influences",
           edited(wideEmpty, [](T &t) { t.influenceCount = 0; }),
           row + ": validated with element_size 65536 and 0 influence(s)");
    refuse("raw validated, index past the influences",
           edited(wideRow, [](T &t) { t.rawIndices[69999] = 2; }),
           row + ": validated, but raw entry 69999 indexes influence 2 of 2");
    refuse("raw validated, negative index",
           edited(wideRow, [](T &t) { t.rawIndices[3] = -1; }),
           row + ": validated, but raw entry 3 indexes influence -1 of 2");
    refuse("raw validated, NaN weight",
           edited(wideRow, [nan](T &t) { t.rawWeights[5] = nan; }),
           row + ": validated, but raw entry 5 has a negative or non-finite "
                 "weight");
    refuse("raw validated, negative weight",
           edited(wideRow, [](T &t) { t.rawWeights[6] = -0.25f; }),
           row + ": validated, but raw entry 6 has a negative or non-finite "
                 "weight");
    refuse("raw rows not validated, the rules pass",
           edited(wideRow, [](T &t) { t.validated = false; }),
           row + ": not validated, but its layout passes the evaluator's "
                 "rules");

    // The sparse form: no raw arrays, and validated exactly when the
    // evaluator's rules pass.
    fb::RigExecWireSkinTopology sparse;
    CHECK(RigExecFormatSparseTopology({1, 2}, {0.5f, 0.5f}, 2, 1, 3, true,
                                      &sparse, &why));
    refuse("sparse with raw indices",
           edited(sparse, [](T &t) { t.rawIndices = {1, 2}; }),
           row + ": raw_indices or raw_weights on a sparse layout");
    refuse("sparse with raw weights",
           edited(sparse, [](T &t) { t.rawWeights = {0.5f}; }),
           row + ": raw_indices or raw_weights on a sparse layout");
    refuse("sparse validated at element size 0", edited(sparse, [](T &t) {
               t.elementSize = 0;
               t.counts8 = {0};
               t.indices8.clear();
               t.weights.clear();
           }),
           row + ": validated with element_size 0 and 3 influence(s)");
    refuse("sparse validated without influences",
           edited(sparse, [](T &t) { t.influenceCount = 0; }),
           row + ": validated with element_size 2 and 0 influence(s)");
    for (const uint8_t width : {uint8_t(1), uint8_t(2), uint8_t(4)}) {
        const T past = edited(sparse, [width](T &t) {
            t.indexWidth = width;
            t.indices8.clear();
            if (width == 1) {
                t.indices8 = {1, 3};
            } else if (width == 2) {
                t.indices16 = {1, 3};
            } else {
                t.indices32 = {1, 3};
            }
        });
        refuse(("sparse validated, index past the influences at width " +
                std::to_string(width))
                   .c_str(),
               past,
               row + ": validated, but kept entry 1 indexes influence 3 of "
                     "3");
    }
    refuse("sparse validated, negative index", edited(sparse, [](T &t) {
               t.indexWidth = 4;
               t.indices8.clear();
               t.indices32 = {1, -1};
           }),
           row + ": validated, but kept entry 1 indexes influence -1 of 3");
    refuse("sparse validated, NaN weight",
           edited(sparse, [nan](T &t) { t.weights[1] = nan; }),
           row + ": validated, but kept entry 1 has a negative or non-finite "
                 "weight");
    refuse("sparse validated, negative weight",
           edited(sparse, [](T &t) { t.weights[0] = -0.5f; }),
           row + ": validated, but kept entry 0 has a negative or non-finite "
                 "weight");
    refuse("sparse not validated, the rules pass",
           edited(sparse, [](T &t) { t.validated = false; }),
           row + ": not validated, but its layout passes the evaluator's "
                 "rules");

    // The same rules on a partition layout.
    const std::string partition =
        "geometry.chains[0].revisions[0].partition_topology";
    refuse("partition raw validated, not rows",
           edited(odd, [](T &t) { t.validated = true; }),
           partition + ": validated, but its raw layout is not rows of "
                       "element_size",
           true);
    refuse("partition sparse validated, index past the influences",
           edited(sparse, [](T &t) { t.indices8 = {1, 3}; }),
           partition + ": validated, but kept entry 1 indexes influence 3 "
                       "of 3",
           true);
    refuse("partition sparse with raw weights",
           edited(sparse, [](T &t) { t.rawWeights = {0.5f}; }),
           partition + ": raw_indices or raw_weights on a sparse layout",
           true);
    refuse("partition sparse not validated, the rules pass",
           edited(sparse, [](T &t) { t.validated = false; }),
           partition + ": not validated, but its layout passes the "
                       "evaluator's rules",
           true);
    refuse("partition raw rows not validated, the rules pass",
           edited(wideEmpty, [](T &t) { t.validated = false; }),
           partition + ": not validated, but its layout passes the "
                       "evaluator's rules",
           true);

    // A validated layout's -0 weight is a zero weight, which the rules
    // admit, and a raw layout serves as a partition layout.
    _context = "raw topology: accepted";
    {
        RigExecWireFile file = _RichFile();
        fb::RigExecWireRevision &revision =
            file.geometry->chains[0].revisions[0];
        revision.topology = std::make_unique<T>(edited(
            sparse, [](T &t) { t.weights[0] = -0.0f; }));
        revision.partitionSameAsTopology = false;
        revision.partitionTopology = std::make_unique<T>(odd);
        why.clear();
        CHECK(RigExecFormatValidate(file, &why));
        if (!why.empty()) {
            std::printf("  refused: %s\n", why.c_str());
        }
    }
    std::printf("raw topology: %zu layouts stored raw, validated, "
                "round-tripped and expanded bit for bit, each refused with "
                "its validated flag flipped (%zu the evaluator's rules fail, "
                "%zu they pass); %d layouts refused by the validator\n",
                stored, unvalidated, passing, refused);
    CHECK(stored == 9 && unvalidated == 7 && passing == 2 && refused == 31);
}

/// The vertex partition of a main revision: a chunked skin revision whose
/// ranges tile its partition's points, keyed by ascending influence
/// positions, over a partition layout of its element size and index count,
/// is accepted, also over a raw partition layout; each way a table breaks
/// the shape the bake cuts is refused with its exact message.
void
TestChunkTables()
{
    _context = "chunk tables";
    std::string why;
    const std::string row = "geometry.chains[0].revisions[0]";
    // The rich revision cut into two chunks over its 2-point topology of
    // element size 2.
    const auto chunked = [] {
        RigExecWireFile f = _RichFile();
        fb::RigExecWireRevision &r = f.geometry->chains[0].revisions[0];
        r.op = uint8_t(fb::RevisionOp::Skin);
        r.skinTopologyFixed = true;
        r.influenceSlots = {0, 0};
        r.chunked = true;
        r.chunks.resize(2);
        r.chunks[0].begin = 0;
        r.chunks[0].end = 1;
        r.chunks[0].key = {0};
        r.chunks[1].begin = 1;
        r.chunks[1].end = 2;
        r.chunks[1].key = {0, 1};
        r.partitionElementSize = 2;
        r.partitionIndexCount = 4;
        r.partitionPointCount = 2;
        f.geometry->revisionChunkCount = {2};
        _AddFixtureTopologyHead(f);
        return f;
    };
    {
        _context = "chunk tables: accepted";
        const RigExecWireFile f = chunked();
        why.clear();
        CHECK(RigExecFormatValidate(f, &why));
        if (!why.empty()) {
            std::printf("  refused: %s\n", why.c_str());
        }
        std::vector<uint8_t> bytes;
        CHECK(_Write(f, &bytes, &why) && _Open(bytes, &why) != nullptr);
    }
    {
        // A raw partition layout of the partition's element size and
        // index count: 5 entries at element size 2 hold 2 rows.
        _context = "chunk tables: raw partition accepted";
        RigExecWireFile f = chunked();
        fb::RigExecWireRevision &r = f.geometry->chains[0].revisions[0];
        fb::RigExecWireSkinTopology raw;
        CHECK(RigExecFormatTopology({1, 0, 0, 1, 0},
                                    {0.5f, 0.0f, 1.0f, 0.25f, -0.0f}, 2, 0,
                                    2, false, &raw, &why));
        r.partitionSameAsTopology = false;
        r.partitionTopology =
            std::make_unique<fb::RigExecWireSkinTopology>(raw);
        r.partitionIndexCount = 5;
        r.chunks[0].key = {0, 1};
        why.clear();
        CHECK(RigExecFormatValidate(f, &why));
        if (!why.empty()) {
            std::printf("  refused: %s\n", why.c_str());
        }
    }

    int refused = 0;
    using F = RigExecWireFile;
    using R = fb::RigExecWireRevision;
    const auto refuse = [&](const char *label,
                            const std::function<void(F &, R &)> &edit,
                            const std::string &expected) {
        _context = std::string("chunk tables refuse: ") + label;
        F f = chunked();
        edit(f, f.geometry->chains[0].revisions[0]);
        f.geometry->revisionChunkCount = {
            int32_t(f.geometry->chains[0].revisions[0].chunks.size())};
        why.clear();
        const bool ok = RigExecFormatValidate(f, &why);
        CHECK(!ok && why == expected);
        if (ok || why != expected) {
            std::printf("  got '%s', expected '%s'\n",
                        ok ? "(accepted)" : why.c_str(), expected.c_str());
        }
        ++refused;
    };
    refuse("one chunk", [](F &, R &r) {
        r.chunks.pop_back();
        r.chunks[0].end = 2;
    }, row + ": chunked with 1 chunk(s)");
    refuse("a matrix revision",
           [](F &, R &r) { r.op = uint8_t(fb::RevisionOp::Matrix); },
           row + ": chunked, but not a skin whose layout the epoch fixes");
    refuse("a layout the epoch does not fix",
           [](F &, R &r) { r.skinTopologyFixed = false; },
           row + ": chunked, but not a skin whose layout the epoch fixes");
    refuse("two chunks, unchunked", [](F &, R &r) {
        r.chunked = false;
        r.chunks[0].key.clear();
        r.chunks[1].key.clear();
    }, row + ": 2 chunks, but not chunked");
    refuse("a key on an unchunked revision", [](F &, R &r) {
        r.chunked = false;
        r.chunks.pop_back();
    }, row + ".chunks[0]: a key on an unchunked revision");
    refuse("a gap", [](F &, R &r) { r.chunks[1].begin = 2; },
           row + ".chunks[1]: vertex range [2, 2) does not continue the "
                 "partition at 1");
    refuse("an overlap", [](F &, R &r) { r.chunks[1].begin = 0; },
           row + ".chunks[1]: vertex range [0, 2) does not continue the "
                 "partition at 1");
    refuse("a first range off 0", [](F &, R &r) { r.chunks[0].begin = 1; },
           row + ".chunks[0]: vertex range [1, 1) does not continue the "
                 "partition at 0");
    refuse("an end before its begin", [](F &, R &r) { r.chunks[0].end = -1; },
           row + ".chunks[0]: vertex range [0, -1) ends before it begins");
    refuse("the last range short", [](F &, R &r) { r.chunks[1].end = 1; },
           row + ".chunks[1]: the last range ends at 1, not at the "
                 "partition's 2 point(s)");
    refuse("a descending key", [](F &, R &r) { r.chunks[1].key = {1, 0}; },
           row + ".chunks[1].key[1]: 0 is not an ascending influence "
                 "position below 2");
    refuse("a repeated key", [](F &, R &r) { r.chunks[1].key = {1, 1}; },
           row + ".chunks[1].key[1]: 1 is not an ascending influence "
                 "position below 2");
    refuse("a key past the influences",
           [](F &, R &r) { r.chunks[1].key = {0, 2}; },
           row + ".chunks[1].key[1]: 2 is not an ascending influence "
                 "position below 2");
    refuse("a negative key", [](F &, R &r) { r.chunks[0].key = {-1}; },
           row + ".chunks[0].key[0]: -1 is not an ascending influence "
                 "position below 2");
    refuse("a partition point count off by one",
           [](F &, R &r) { r.partitionPointCount = 3; },
           row + ": partition_point_count 3, but partition_index_count 4 is "
                 "2 row(s) of partition_element_size 2");
    refuse("a partition point count at element size 0", [](F &, R &r) {
        r.partitionElementSize = 0;
    }, row + ": partition_point_count 2, but partition_index_count 4 is 0 "
             "row(s) of partition_element_size 0");
    refuse("a partition point count on an unchunked revision",
           [](F &, R &r) {
               r.chunked = false;
               r.chunks.assign(1, fb::RigExecWireChunk());
               r.partitionPointCount = 1;
           },
           row + ": partition_point_count 1, but partition_index_count 4 is "
                 "2 row(s) of partition_element_size 2");
    refuse("the partition's element size", [](F &, R &r) {
        r.partitionElementSize = 1;
        r.partitionPointCount = 4;
        r.chunks[1].end = 4;
    }, row + ": the partition's element_size and index_count are not its "
             "layout's");
    refuse("the partition's index count", [](F &, R &r) {
        r.partitionIndexCount = 6;
        r.partitionPointCount = 3;
        r.chunks[1].end = 3;
    }, row + ": the partition's element_size and index_count are not its "
             "layout's");
    refuse("a raw partition's index count", [&why](F &, R &r) {
        fb::RigExecWireSkinTopology raw;
        RigExecFormatTopology({1, 0, 0, 1, 0},
                              {0.5f, 0.0f, 1.0f, 0.25f, -0.0f}, 2, 0, 2,
                              false, &raw, &why);
        r.partitionSameAsTopology = false;
        r.partitionTopology =
            std::make_unique<fb::RigExecWireSkinTopology>(raw);
    }, row + ": the partition's element_size and index_count are not its "
             "layout's");
    std::printf("chunk tables: a chunked skin revision accepted over a "
                "sparse and a raw partition layout; %d chunk table "
                "violations refused\n",
                refused);
    CHECK(refused == 20);
}

// ----------------------------------------------------------------- arrays

// Path ids the array file adds to the rich file's; each is also the id of
// its name.
enum : uint32_t {
    _pJointIndices = 13,  // /Rig/Mover.rigExec:jointIndices
    _pJointWeights = 14,  // /Rig/Mover.rigExec:jointWeights
    _pValues = 15,        // /Rig/W.rigExec:values
    _pIndices = 16,       // /Rig/W.rigExec:indices
    _pSamples = 17,       // /Rig/Mesh.samples
    _pCurve = 18,         // /Rig/Mesh.curve
    _pKnots = 19,         // /Rig/Mover.knots
    _pSt = 20,            // /Rig/Mover.st
};

// The unlisted slots the array file appends after the rich file's three.
enum : uint32_t {
    _sJointIndices = 3,
    _sJointWeights = 4,
    _sBase = 5,
    _sValues = 6,
    _sIndices = 7,
    _sSamples = 8,
    _sCurve = 9,
    _sKnots = 10,
    _sSt = 11,
};

// The array values the array file appends: revision 0's layout, each
// pool's entry 1, and each pool's empty array.
enum : uint32_t {
    _vSkinIndices = _vCount,
    _vSkinWeights,
    _vPoints,
    _vFloats,
    _vInts,
    _vDoubles,
    _vVec2fs,
    _vNoInts,
    _vNoFloats,
    _vNoDoubles,
    _vNoVec2fs,
    _vNoVec3fs,
    _vArrayCount,
};

/// The rich file with an array input of each tag: revision 0 a fixed skin
/// whose layout slots default to its stored layout, the chain's base slot,
/// the weight object's painted and oracle slots, a blend sample's points
/// read through a slot, and path reads bound to slots: a live Raw read, a
/// rest Raw read holding its Default-time value, and a live Resolved read.
RigExecWireFile
_ArrayFile()
{
    RigExecWireFile f = _RichFile();
    for (const char *name :
         {"rigExec:jointIndices", "rigExec:jointWeights", "rigExec:values",
          "rigExec:indices", "samples", "curve", "knots", "st"}) {
        f.names.push_back(name);
    }
    const std::pair<uint32_t, uint32_t> properties[] = {
        {_pMover, _pJointIndices}, {_pMover, _pJointWeights},
        {_pWeight, _pValues},      {_pWeight, _pIndices},
        {_pMesh, _pSamples},       {_pMesh, _pCurve},
        {_pMover, _pKnots},        {_pMover, _pSt}};
    for (const auto &[prim, id] : properties) {
        f.paths.push_back(fb::PathNode(prim, id, PathKind::Property));
    }

    f.values.resize(_vArrayCount);
    const auto array = [&](uint32_t id, InputTag tag, fb::ArraySource source,
                           uint32_t at) {
        f.values[id].tag = tag;
        f.values[id].arraySource = source;
        f.values[id].array = at;
    };
    using Source = fb::ArraySource;
    array(_vSkinIndices, InputTag::IntArray, Source::SkinIndices, 0);
    array(_vSkinWeights, InputTag::FloatArray, Source::SkinWeights, 0);
    array(_vPoints, InputTag::Vec3fArray, Source::Pool, 1);
    array(_vFloats, InputTag::FloatArray, Source::Pool, 1);
    array(_vInts, InputTag::IntArray, Source::Pool, 1);
    array(_vDoubles, InputTag::DoubleArray, Source::Pool, 1);
    array(_vVec2fs, InputTag::Vec2fArray, Source::Pool, 1);
    array(_vNoInts, InputTag::IntArray, Source::Pool, 0);
    array(_vNoFloats, InputTag::FloatArray, Source::Pool, 0);
    array(_vNoDoubles, InputTag::DoubleArray, Source::Pool, 0);
    array(_vNoVec2fs, InputTag::Vec2fArray, Source::Pool, 0);
    array(_vNoVec3fs, InputTag::Vec3fArray, Source::Pool, 0);

    const uint8_t has = uint8_t(fb::InputSlotFlags::HasValue);
    const auto slot = [&](uint32_t path, uint32_t value, InputTag tag) {
        f.inputs.push_back(fb::InputSlot(path, value, -1, -1, tag, has));
    };
    slot(_pJointIndices, _vSkinIndices, InputTag::IntArray);
    slot(_pJointWeights, _vSkinWeights, InputTag::FloatArray);
    slot(_pMeshPoints, _vPoints, InputTag::Vec3fArray);
    slot(_pValues, _vFloats, InputTag::FloatArray);
    slot(_pIndices, _vInts, InputTag::IntArray);
    slot(_pSamples, _vPoints, InputTag::Vec3fArray);
    slot(_pCurve, _vPoints, InputTag::Vec3fArray);
    slot(_pKnots, _vDoubles, InputTag::DoubleArray);
    slot(_pSt, _vVec2fs, InputTag::Vec2fArray);

    fb::RigExecWireDomainGeometry &g = *f.geometry;
    fb::RigExecWireChain &chain = g.chains[0];
    chain.baseSlot = int32_t(_sBase);
    fb::RigExecWireRevision &revision = chain.revisions[0];
    revision.op = uint8_t(fb::RevisionOp::Skin);
    revision.skinTopologyFixed = true;
    revision.topologyResolved = true;
    revision.jointIndicesSlot = int32_t(_sJointIndices);
    revision.jointWeightsSlot = int32_t(_sJointWeights);
    fb::RigExecWireBlendSample &sample =
        revision.blendChannels[0].samples[0];
    sample.pointsPath = _pSamples;
    sample.pointsRead =
        _In(InputTag::Vec3fArray, ReadMode::Raw, {uint32_t(_sSamples)});
    sample.pointsRead->constant = _vNoVec3fs;

    fb::RigExecWireWeightObject &weight = g.weightObjects[0];
    weight.valuesSlot = int32_t(_sValues);
    weight.indicesSlot = int32_t(_sIndices);
    weight.oracleSamplesSlot = int32_t(_sSamples);
    weight.oracleCurveSlot = int32_t(_sCurve);

    const auto read = [&](uint32_t path, bool rest, InputTag tag,
                          ReadMode mode, uint32_t at, uint32_t constant) {
        fb::RigExecWirePathRead row;
        row.path = path;
        row.rest = rest;
        row.read = _In(tag, mode, {at});
        row.read->constant = constant;
        return row;
    };
    g.pathReads.push_back(read(_pValues, false, InputTag::FloatArray,
                               ReadMode::Raw, _sValues, _vNoFloats));
    g.pathReads.push_back(read(_pKnots, true, InputTag::DoubleArray,
                               ReadMode::Raw, _sKnots, _vNoDoubles));
    g.pathReads.back().value = std::make_unique<fb::RigExecWirePathValue>();
    g.pathReads.back().value->tag = fb::PathTag::DoubleArray;
    g.pathReads.back().value->array = 1;
    g.pathReads.push_back(read(_pSt, false, InputTag::Vec2fArray,
                               ReadMode::Resolved, _sSt, _vNoVec2fs));
    _AddFixtureTopologyHead(f);
    return f;
}

/// Array inputs: the array file validates and survives Write -> Open ->
/// Write bit for bit with every array field it holds; a slot whose read
/// failed (no HasValue) defaults to the empty array; each rule of the array
/// tags, sources, slots and reads refuses its violation with its exact
/// message.
void
TestArrayInputs()
{
    _context = "array inputs";
    std::string why;
    const RigExecWireFile file = _ArrayFile();
    CHECK(RigExecFormatValidate(file, &why));
    if (!why.empty()) {
        std::printf("  %s\n", why.c_str());
    }
    std::vector<uint8_t> bytes, again;
    std::unique_ptr<RigExecWireFile> o;
    CHECK(_Write(file, &bytes, &why));
    if (!bytes.empty()) {
        o = _Open(bytes, &why);
    }
    CHECK(o != nullptr);
    if (o) {
        CHECK(_Write(*o, &again) && again == bytes);
        CHECK(o->values.size() == size_t(_vArrayCount));
        for (size_t i = _vCount; i < o->values.size(); ++i) {
            CHECK(o->values[i].tag == file.values[i].tag &&
                  o->values[i].arraySource == file.values[i].arraySource &&
                  o->values[i].array == file.values[i].array &&
                  o->values[i].bits == 0);
        }
        CHECK(o->values[_vSkinWeights].arraySource ==
                  fb::ArraySource::SkinWeights &&
              o->values[_vPoints].array == 1);
        CHECK(o->inputs.size() == 12 && o->listedInputs == 3 &&
              o->inputs[_sSt].type() == InputTag::Vec2fArray &&
              o->inputs[_sKnots].value() == _vDoubles);
        const fb::RigExecWireChain &chain = o->geometry->chains[0];
        const fb::RigExecWireRevision &r = chain.revisions[0];
        CHECK(chain.baseSlot == int32_t(_sBase) &&
              r.jointIndicesSlot == int32_t(_sJointIndices) &&
              r.jointWeightsSlot == int32_t(_sJointWeights) &&
              chain.derived[0].revision->jointIndicesSlot == -1);
        const fb::RigExecWireBlendSample &sample =
            r.blendChannels[0].samples[0];
        CHECK(sample.pointsRead &&
              sample.pointsRead->tag == InputTag::Vec3fArray &&
              sample.pointsRead->mode == ReadMode::Raw &&
              sample.pointsRead->walk ==
                  std::vector<uint32_t>{uint32_t(_sSamples)});
        const fb::RigExecWireWeightObject &w = o->geometry->weightObjects[0];
        CHECK(w.valuesSlot == int32_t(_sValues) &&
              w.indicesSlot == int32_t(_sIndices) &&
              w.oracleSamplesSlot == int32_t(_sSamples) &&
              w.oracleCurveSlot == int32_t(_sCurve));
        const auto &rows = o->geometry->pathReads;
        CHECK(rows.size() == 9);
        if (rows.size() == 9) {
            CHECK(rows[6].read && !rows[6].value &&
                  rows[6].read->tag == InputTag::FloatArray);
            CHECK(rows[7].rest && rows[7].read && rows[7].value &&
                  rows[7].value->tag == fb::PathTag::DoubleArray &&
                  rows[7].read->mode == ReadMode::Raw);
            CHECK(rows[8].read && rows[8].read->mode == ReadMode::Resolved);
        }
    }
    // An array whose read failed defaults to the empty array, unflagged.
    {
        RigExecWireFile empty = _ArrayFile();
        empty.inputs[_sSt] = fb::InputSlot(_pSt, _vNoVec2fs, -1, -1,
                                           InputTag::Vec2fArray, 0);
        CHECK(RigExecFormatValidate(empty, &why));
    }

    int refused = 0;
    const auto expect = [&](const char *name, const std::string &want,
                            const std::function<void(RigExecWireFile &)>
                                &mutate) {
        _context = std::string("array inputs: ") + name;
        RigExecWireFile f = _ArrayFile();
        mutate(f);
        std::string got;
        const bool ok = RigExecFormatValidate(f, &got);
        CHECK(!ok && got == want);
        if (ok || got != want) {
            std::printf("  got: %s\n  want: %s\n",
                        ok ? "(accepted)" : got.c_str(), want.c_str());
        }
        ++refused;
    };
    using F = RigExecWireFile;
    using Source = fb::ArraySource;
    const auto has = uint8_t(fb::InputSlotFlags::HasValue);
    const auto at = [](uint32_t value) { return std::to_string(value); };
    const std::string revision = "geometry.chains[0].revisions[0]";
    const std::string sample =
        revision + ".blend_channels[0].samples[0].points_read";
    const std::string weight = "geometry.weight_objects[0]";

    // Values.
    expect("an array source on a scalar",
           "values[8]: an array source on a scalar value", [](F &f) {
               f.values[_vBool].arraySource = Source::SkinIndices;
           });
    expect("an array id on a scalar",
           "values[8]: an array source on a scalar value",
           [](F &f) { f.values[_vBool].array = 1; });
    expect("bits on an array",
           "values[" + at(_vDoubles) + "]: bits or members on an array value",
           [](F &f) { f.values[_vDoubles].bits = 1; });
    expect("a member on an array",
           "values[" + at(_vDoubles) + "]: its members do not match its tag",
           [](F &f) {
               f.values[_vDoubles].vec3f =
                   std::make_unique<RigExecWireVec3f>(_V3f(0));
           });
    expect("an array source past the enum",
           "values[" + at(_vInts) + "]: array source out of range",
           [](F &f) { f.values[_vInts].arraySource = Source(3); });
    for (const uint32_t id :
         {uint32_t(_vInts), uint32_t(_vFloats), uint32_t(_vDoubles),
          uint32_t(_vVec2fs), uint32_t(_vPoints)}) {
        expect("a pool id past its pool",
               "values[" + at(id) + "].array: pool id 2 out of range (2)",
               [id](F &f) { f.values[id].array = 2; });
    }
    expect("skin weights on an int array",
           "values[" + at(_vSkinIndices) +
               "]: a skin layout source on a int[] value",
           [](F &f) {
               f.values[_vSkinIndices].arraySource = Source::SkinWeights;
           });
    expect("skin indices on a float array",
           "values[" + at(_vSkinWeights) +
               "]: a skin layout source on a float[] value",
           [](F &f) {
               f.values[_vSkinWeights].arraySource = Source::SkinIndices;
           });
    expect("a layout past the revisions",
           "values[" + at(_vSkinIndices) + "].array: 1 out of range (1)",
           [](F &f) { f.values[_vSkinIndices].array = 1; });
    expect("a layout of a revision the epoch does not fix",
           "values[" + at(_vSkinIndices) +
               "]: revision 0 stores no fixed layout",
           [](F &f) {
               fb::RigExecWireRevision &r = f.geometry->chains[0].revisions[0];
               r.jointIndicesSlot = -1;
               r.jointWeightsSlot = -1;
               r.skinTopologyFixed = false;
           });
    expect("a layout of a revision with no stored layout",
           "values[" + at(_vSkinIndices) +
               "]: revision 0 stores no fixed layout",
           [](F &f) {
               fb::RigExecWireRevision &r = f.geometry->chains[0].revisions[0];
               r.jointIndicesSlot = -1;
               r.jointWeightsSlot = -1;
               r.topology.reset();
               r.partitionSameAsTopology = false;
           });

    // Slots.
    expect("an array slot naming a chain",
           "inputs[" + at(_sKnots) +
               "]: an array slot names a chain or a phased consumer",
           [has](F &f) {
               f.inputs[_sKnots] = fb::InputSlot(_pKnots, _vDoubles, 0, -1,
                                                 InputTag::DoubleArray, has);
           });
    expect("an array slot naming a phased consumer",
           "inputs[" + at(_sKnots) +
               "]: an array slot names a chain or a phased consumer",
           [has](F &f) {
               f.inputs[_sKnots] = fb::InputSlot(_pKnots, _vDoubles, -1, 0,
                                                 InputTag::DoubleArray, has);
           });
    expect("an array slot defaulting to another tag",
           "inputs[" + at(_sKnots) + "]: malformed type, flags or default",
           [has](F &f) {
               f.inputs[_sKnots] = fb::InputSlot(_pKnots, _vFloats, -1, -1,
                                                 InputTag::DoubleArray, has);
           });

    // Reads.
    expect("a scalar read walking an array slot",
           "pose.solvers[0].bend.walk[0]: slot " + at(_sKnots) +
               " is an array input, which a scalar read never walks",
           [](F &f) { f.pose->solvers[0].bend->walk = {_sKnots}; });
    expect("an array read over a slot of another tag",
           "geometry.path_reads[8].read.walk[0]: slot " + at(_sKnots) +
               " holds double[], not the read's float2[]",
           [](F &f) { f.geometry->pathReads[8].read->walk = {_sKnots}; });
    expect("an array read walking on to another tag",
           "geometry.path_reads[8].read.walk[1]: slot " + at(_sSamples) +
               " holds float3[], not the read's float2[]",
           [](F &f) {
               f.geometry->pathReads[8].read->walk = {_sSt, _sSamples};
           });
    expect("a Baked array read",
           "geometry.path_reads[8].read: read mode 0 is not admitted here",
           [](F &f) {
               f.geometry->pathReads[8].read->mode = ReadMode::Baked;
           });
    expect("an array read through a chain",
           "geometry.path_reads[8].read: an array read crosses no chain "
           "and reads no long way",
           [](F &f) {
               f.geometry->pathReads[8].read->flags =
                   uint8_t(fb::InputReadFlags::ViaChain);
           });
    expect("an array read whose constant is a layout",
           "geometry.path_reads[6].read: constant " + at(_vSkinWeights) +
               " is not a pool array",
           [](F &f) {
               f.geometry->pathReads[6].read->constant = _vSkinWeights;
           });
    expect("an array read of a plugin mover",
           "external_movers[0].inputs[0]: tag 9 is an array tag, which "
           "this site does not read",
           [](F &f) {
               fb::RigExecWireRevision &r = f.geometry->chains[0].revisions[0];
               r.jointIndicesSlot = -1;
               r.jointWeightsSlot = -1;
               _AddPlugin(f);
               f.externalMovers[0].inputs[0] =
                   _InValue(InputTag::FloatArray, ReadMode::Raw, {_sValues});
               f.externalMovers[0].inputs[0].constant = _vNoFloats;
           });
    expect("a phased hop on an array slot",
           "phased_consumers[0].hops[1]: slot " + at(_sKnots) +
               " is an array input",
           [](F &f) { f.phasedConsumers[0].hops = {1, _sKnots}; });

    // Path reads bound to array slots.
    expect("a rest array read without its value",
           "geometry.path_reads[7]: an array read holds a value exactly "
           "when it reads at rest",
           [](F &f) { f.geometry->pathReads[7].value.reset(); });
    expect("a live array read with a value",
           "geometry.path_reads[8]: an array read holds a value exactly "
           "when it reads at rest",
           [](F &f) {
               auto &row = f.geometry->pathReads[8];
               row.value = std::make_unique<fb::RigExecWirePathValue>();
               row.value->tag = fb::PathTag::Vec2fArray;
           });
    expect("a Resolved array read at rest",
           "geometry.path_reads[7].read: an array read at rest is Raw",
           [](F &f) {
               f.geometry->pathReads[7].read->mode = ReadMode::Resolved;
           });
    expect("an array read headed by another attribute",
           "geometry.path_reads[6].read: an array read headed by its own "
           "attribute expected",
           [](F &f) {
               f.geometry->pathReads[6].read->walk = {_sJointWeights};
           });
    expect("an array read with the head fallback",
           "geometry.path_reads[8]: head_fallback on an array read",
           [](F &f) { f.geometry->pathReads[8].headFallback = true; });
    expect("a rest value of another tag",
           "geometry.path_reads[7].value: tag 9 is not the read's double[]",
           [](F &f) {
               f.geometry->pathReads[7].value->tag = fb::PathTag::FloatArray;
           });

    // Blend sample points.
    expect("points read on a blend shape sample",
           sample + ": on a sample with a blend shape", [](F &f) {
               f.geometry->chains[0].revisions[0].blendChannels[0]
                   .samples[0]
                   .blendShape = _pMesh;
           });
    expect("points read of another tag", sample + ": tag 1, expected 12",
           [](F &f) {
               auto &read = f.geometry->chains[0].revisions[0]
                                .blendChannels[0]
                                .samples[0]
                                .pointsRead;
               read->tag = InputTag::Float;
               read->constant = _vFloatNaN;
           });
    expect("points read off points_path",
           sample + ": a read headed by points_path expected", [](F &f) {
               f.geometry->chains[0].revisions[0].blendChannels[0]
                   .samples[0]
                   .pointsPath = _pCtlTx;
           });

    // The chain base.
    expect("a base slot of another tag",
           "geometry.chains[0].base_slot: slot " + at(_sValues) +
               " is not a float3[] input",
           [](F &f) { f.geometry->chains[0].baseSlot = _sValues; });
    expect("a base slot past the slots",
           "geometry.chains[0].base_slot: 12 out of range (12)",
           [](F &f) { f.geometry->chains[0].baseSlot = 12; });
    expect("a base slot without a base",
           "geometry.chains[0].base_slot: a base slot on a chain without a "
           "base",
           [](F &f) {
               f.geometry->chains[0].haveBase = false;
               f.geometry->chains[0].base = 0;
           });
    expect("a base slot of another attribute",
           "geometry.chains[0].base_slot: slot " + at(_sSamples) +
               " is not the chain target's input",
           [](F &f) { f.geometry->chains[0].baseSlot = _sSamples; });
    expect("a base slot defaulting to another base",
           "geometry.chains[0].base_slot: slot " + at(_sBase) +
               "'s default is not the chain's base",
           [](F &f) { f.geometry->chains[0].base = 0; });

    // Layout slots.
    expect("one layout slot alone",
           revision + ": joint_indices_slot and joint_weights_slot are set "
                      "together",
           [](F &f) {
               f.geometry->chains[0].revisions[0].jointWeightsSlot = -1;
           });
    expect("layout slots on a derived revision",
           "geometry.chains[0].derived[0].revision: layout slots on a "
           "revision that is not a main skin whose layout the epoch fixes",
           [](F &f) {
               fb::RigExecWireRevision &r =
                   *f.geometry->chains[0].derived[0].revision;
               r.jointIndicesSlot = int32_t(_sJointIndices);
               r.jointWeightsSlot = int32_t(_sJointWeights);
           });
    expect("layout slots on a revision the epoch does not fix",
           revision + ": layout slots on a revision that is not a main "
                      "skin whose layout the epoch fixes",
           [](F &f) {
               f.geometry->chains[0].revisions[0].skinTopologyFixed = false;
           });
    expect("a layout slot of another tag",
           revision + ".joint_indices_slot: slot " + at(_sValues) +
               " is not a int[] input",
           [](F &f) {
               f.geometry->chains[0].revisions[0].jointIndicesSlot =
                   int32_t(_sValues);
           });
    expect("a layout indices slot of another attribute",
           revision + ".joint_indices_slot: slot " + at(_sIndices) +
               " is not the mover's rigExec:jointIndices input",
           [](F &f) {
               f.geometry->chains[0].revisions[0].jointIndicesSlot =
                   int32_t(_sIndices);
           });
    expect("a layout weights slot of another attribute",
           revision + ".joint_weights_slot: slot " + at(_sValues) +
               " is not the mover's rigExec:jointWeights input",
           [](F &f) {
               f.geometry->chains[0].revisions[0].jointWeightsSlot =
                   int32_t(_sValues);
           });
    expect("a layout default naming another revision",
           revision + ".joint_indices_slot: its default does not name this "
                      "revision's layout",
           [](F &f) {
               _AddFailedPlugin(f);
               f.values[_vSkinIndices].array = 1;
           });
    expect("a pool layout default beside a stored layout",
           revision + ".joint_weights_slot: its default does not name this "
                      "revision's layout",
           [has](F &f) {
               f.inputs[_sJointWeights] =
                   fb::InputSlot(_pJointWeights, _vFloats, -1, -1,
                                 InputTag::FloatArray, has);
           });
    expect("a stored-layout default with no stored layout",
           revision + ".joint_indices_slot: its default names a layout this "
                      "revision does not store",
           [](F &f) {
               f.geometry->chains[0].revisions[0].topologyResolved = false;
           });

    // Painted and oracle slots.
    expect("a values slot of another tag",
           weight + ".values_slot: slot " + at(_sIndices) +
               " is not a float[] input",
           [](F &f) {
               f.geometry->weightObjects[0].valuesSlot = int32_t(_sIndices);
           });
    expect("an indices slot past the slots",
           weight + ".indices_slot: 99 out of range (12)",
           [](F &f) { f.geometry->weightObjects[0].indicesSlot = 99; });
    expect("a values slot of another attribute",
           weight + ".values_slot: slot " + at(_sJointWeights) +
               " is not the object's rigExec:values input",
           [](F &f) {
               f.geometry->weightObjects[0].valuesSlot =
                   int32_t(_sJointWeights);
           });
    expect("an indices slot of another attribute",
           weight + ".indices_slot: slot " + at(_sJointIndices) +
               " is not the object's rigExec:indices input",
           [](F &f) {
               f.geometry->weightObjects[0].indicesSlot =
                   int32_t(_sJointIndices);
           });
    expect("an oracle curve slot of another tag",
           weight + ".oracle_curve_slot: slot " + at(_sValues) +
               " is not a float3[] input",
           [](F &f) {
               f.geometry->weightObjects[0].oracleCurveSlot =
                   int32_t(_sValues);
           });
    expect("a painted default that is a layout",
           weight + ".indices_slot: its default is not a pool array",
           [has](F &f) {
               f.inputs[_sIndices] = fb::InputSlot(
                   _pIndices, _vSkinIndices, -1, -1, InputTag::IntArray, has);
           });
    std::printf("array inputs: the array file round-trips bit for bit; %d "
                "array rule violations refused with their messages\n",
                refused);
    CHECK(refused == 55);
}

/// Offsets a test builds by hand in place of what Pack would write.
struct _Parts {
    flatbuffers::Offset<
        flatbuffers::Vector<flatbuffers::Offset<flatbuffers::String>>>
        names;
    flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<fb::IntArray>>>
        intArrays;
};

/// The minimal file packed field by field, with \p build's names or
/// int_arrays in place of Pack's: the way to build buffers whose objects
/// are shared or overlap, which Write never produces.
std::vector<uint8_t>
_PackMinimalWith(
    const std::function<_Parts(flatbuffers::FlatBufferBuilder &)> &build)
{
    const RigExecWireFile f = _MinimalFile();
    flatbuffers::FlatBufferBuilder b;
    _Parts parts = build(b);
    if (parts.names.IsNull()) {
        parts.names = b.CreateVectorOfStrings(f.names);
    }
    if (parts.intArrays.IsNull()) {
        const auto empty = fb::CreateIntArray(b, &f.intArrays[0]);
        parts.intArrays = b.CreateVector(&empty, 1);
    }
    const auto paths = b.CreateVectorOfStructs(f.paths);
    const auto value = fb::CreateValue(b, &f.values[0]);
    const auto values = b.CreateVector(&value, 1);
    const auto floats = fb::CreateFloatArray(b, &f.floatArrays[0]);
    const auto floatArrays = b.CreateVector(&floats, 1);
    const auto doubles = fb::CreateDoubleArray(b, &f.doubleArrays[0]);
    const auto doubleArrays = b.CreateVector(&doubles, 1);
    const auto vec2fs = fb::CreateVec2fArray(b, &f.vec2fArrays[0]);
    const auto vec2fArrays = b.CreateVector(&vec2fs, 1);
    const auto vec3fs = fb::CreateVec3fArray(b, &f.vec3fArrays[0]);
    const auto vec3fArrays = b.CreateVector(&vec3fs, 1);
    const auto slotMeta = fb::CreateSlotMeta(b, f.slotMeta.get());
    const auto constants = fb::CreateConstants(b, f.constants.get());
    const auto clustering = fb::CreateClustering(b, f.clustering.get());
    const auto cones = fb::CreateCones(b, f.cones.get());
    const auto pose = fb::CreateDomainPose(b, f.pose.get());
    const auto geometry = fb::CreateDomainGeometry(b, f.geometry.get());
    fb::FileBuilder file(b);
    file.add_formatVersion(f.formatVersion);
    file.add_rig(f.rig);
    file.add_names(parts.names);
    file.add_paths(paths);
    file.add_values(values);
    file.add_intArrays(parts.intArrays);
    file.add_floatArrays(floatArrays);
    file.add_doubleArrays(doubleArrays);
    file.add_vec2fArrays(vec2fArrays);
    file.add_vec3fArrays(vec3fArrays);
    file.add_slotMeta(slotMeta);
    file.add_constants(constants);
    file.add_clustering(clustering);
    file.add_cones(cones);
    file.add_pose(pose);
    file.add_geometry(geometry);
    fb::FinishFileBuffer(b, file.Finish());
    return std::vector<uint8_t>(b.GetBufferPointer(),
                                b.GetBufferPointer() + b.GetSize());
}

bool
_Verifies(const std::vector<uint8_t> &bytes)
{
    flatbuffers::Verifier verifier(bytes.data(), bytes.size());
    return fb::VerifyFileBuffer(verifier);
}

/// Buffers the FlatBuffers verifier accepts but whose offsets share or
/// overlap objects, so UnPack would copy them once per reference: Open
/// refuses each before unpacking, and accepts large unshared ones.
void
TestBoundedOpen()
{
    _context = "bounded open";
    std::string why;
    using Builder = flatbuffers::FlatBufferBuilder;
    constexpr size_t refs = 4096;

    // The hand-packed minimal file opens; so do sixteen distinct 64 KiB
    // pool arrays.
    const auto plain = _PackMinimalWith([](Builder &) { return _Parts(); });
    CHECK(_Open(plain, &why) != nullptr);
    const auto distinct = _PackMinimalWith([](Builder &b) {
        std::vector<flatbuffers::Offset<fb::IntArray>> arrays = {
            fb::CreateIntArray(b)};
        for (int k = 0; k < 16; ++k) {
            arrays.push_back(fb::CreateIntArray(
                b, b.CreateVector(std::vector<int32_t>(1 << 14, k))));
        }
        _Parts parts;
        parts.intArrays = b.CreateVector(arrays);
        return parts;
    });
    const auto opened = _Open(distinct, &why);
    CHECK(opened && opened->intArrays.size() == 17 &&
          opened->intArrays[16].v.size() == (1u << 14));

    // names: 4096 entries naming one 64 KiB string (256 MiB unpacked).
    const auto sharedString = _PackMinimalWith([](Builder &b) {
        std::vector<flatbuffers::Offset<flatbuffers::String>> names = {
            b.CreateString(""), b.CreateString("Rig")};
        names.resize(2 + refs, b.CreateString(std::string(1 << 16, 'n')));
        _Parts parts;
        parts.names = b.CreateVector(names);
        return parts;
    });
    CHECK(_Verifies(sharedString));
    CHECK(!_Open(sharedString, &why) && _Contains(why, "malformed") &&
          _Contains(why, "do not fit"));

    // int_arrays: 4096 entries naming one table with a 64 KiB vector. The
    // validator has no rule against it: only the bound refuses it.
    const auto sharedTable = _PackMinimalWith([](Builder &b) {
        const auto big = fb::CreateIntArray(
            b, b.CreateVector(std::vector<int32_t>(1 << 14, 7)));
        std::vector<flatbuffers::Offset<fb::IntArray>> arrays(1 + refs, big);
        arrays[0] = fb::CreateIntArray(b);
        _Parts parts;
        parts.intArrays = b.CreateVector(arrays);
        return parts;
    });
    CHECK(_Verifies(sharedTable));
    CHECK(!_Open(sharedTable, &why) && _Contains(why, "do not fit"));

    // int_arrays whose vectors overlap: entry j + 1 reads the suffix of one
    // 2048-int vector starting at its element j, which holds the suffix's
    // length. No two start alike; together they hold 8 MiB.
    const auto overlapping = _PackMinimalWith([](Builder &b) {
        constexpr int32_t count = 2048;
        std::vector<int32_t> lengths(count);
        for (int32_t j = 0; j < count; ++j) {
            lengths[size_t(j)] = count - j - 1;
        }
        const auto vector = b.CreateVector(lengths);
        std::vector<flatbuffers::Offset<fb::IntArray>> arrays = {
            fb::CreateIntArray(b)};
        for (int32_t j = 0; j < count; ++j) {
            arrays.push_back(fb::CreateIntArray(
                b, flatbuffers::Offset<flatbuffers::Vector<int32_t>>(
                       vector.o - 4 - 4 * uint32_t(j))));
        }
        _Parts parts;
        parts.intArrays = b.CreateVector(arrays);
        return parts;
    });
    CHECK(_Verifies(overlapping));
    CHECK(!_Open(overlapping, &why) && _Contains(why, "do not fit"));

    // A nested presentation naming one control 4096 times.
    const std::vector<uint8_t> nested =
        _PresentationBytes(fb::PresentationIdentifier(), refs);
    flatbuffers::Verifier nestedVerifier(nested.data(), nested.size());
    CHECK(fb::VerifyPresentationBuffer(nestedVerifier));
    RigExecWireFile sharedNested = _RichFile();
    sharedNested.presentation = nested;
    CHECK(!RigExecFormatValidate(sharedNested, &why) &&
          _Contains(why, "presentation") && _Contains(why, "do not fit"));
    std::printf("bounded open: shared strings, shared tables, overlapping "
                "vectors and a shared nested control refused\n");
}

std::string
_Hex(const std::string &text)
{
    std::string out;
    char digits[4];
    for (const unsigned char c : text) {
        std::snprintf(digits, sizeof digits, "%02x", c);
        out += digits;
    }
    return out;
}

/// The listed-input order against composed texts: names with bytes below
/// '.', between the separators and above 0x7f, prefixes of each other, on
/// three levels. For every ordered pair of properties a listed pair is
/// accepted exactly when std::string orders their texts, and the whole
/// set sorted that way is accepted.
void
TestPathOrder()
{
    _context = "path order";
    RigExecWireFile f = _MinimalFile();
    const std::vector<std::string> words = {"a",  "a-", "a0", "ab",
                                            "a\x80", " ",  "b",  "x",
                                            "x-", "a\x01"};
    f.names = {"", "Rig"};
    f.names.insert(f.names.end(), words.begin(), words.end());
    const auto nameId = [&](const std::string &name) {
        return uint32_t(std::find(f.names.begin(), f.names.end(), name) -
                        f.names.begin());
    };
    std::vector<uint32_t> prims;
    const auto addPrim = [&](uint32_t parent, const std::string &name) {
        f.paths.push_back(fb::PathNode(parent, nameId(name), PathKind::Prim));
        prims.push_back(uint32_t(f.paths.size() - 1));
        return prims.back();
    };
    for (const char *top : {"a", "a-", "a0", "ab", "a\x80", " ", "b"}) {
        const uint32_t level1 = addPrim(0, top);
        for (const char *child : {"a", "ab", "a-"}) {
            const uint32_t level2 = addPrim(level1, child);
            if (std::string(child) == "a") {
                addPrim(level2, "a");
            }
        }
    }
    std::vector<uint32_t> properties;
    for (const uint32_t prim : prims) {
        for (const char *name : {"x", "x-", "a", "a\x01"}) {
            f.paths.push_back(
                fb::PathNode(prim, nameId(name), PathKind::Property));
            properties.push_back(uint32_t(f.paths.size() - 1));
        }
    }
    const auto slot = [](uint32_t name) {
        return fb::InputSlot(name, 0, -1, -1, InputTag::Double,
                             uint8_t(fb::InputSlotFlags::Listed));
    };
    std::string why;
    size_t pairs = 0, mismatches = 0;
    for (const uint32_t a : properties) {
        for (const uint32_t b : properties) {
            f.inputs = {slot(a), slot(b)};
            f.listedInputs = 2;
            const std::string textA = RigExecFormatPathText(f, a);
            const std::string textB = RigExecFormatPathText(f, b);
            const bool accepted = RigExecFormatValidate(f, &why);
            ++pairs;
            if (a == b) {
                CHECK(!accepted && _Contains(why, "a second slot for " +
                                                      textA));
            } else if (accepted != (textA < textB)) {
                ++mismatches;
                if (mismatches <= 5) {
                    std::printf("  %s vs %s: %s\n", _Hex(textA).c_str(),
                                _Hex(textB).c_str(),
                                accepted ? "accepted" : why.c_str());
                }
            }
        }
    }
    CHECK(mismatches == 0);

    std::vector<uint32_t> sorted = properties;
    std::sort(sorted.begin(), sorted.end(), [&](uint32_t a, uint32_t b) {
        return RigExecFormatPathText(f, a) < RigExecFormatPathText(f, b);
    });
    f.inputs.clear();
    for (const uint32_t id : sorted) {
        f.inputs.push_back(slot(id));
    }
    f.listedInputs = uint32_t(f.inputs.size());
    CHECK(RigExecFormatValidate(f, &why));
    std::swap(f.inputs[10], f.inputs[11]);
    CHECK(!RigExecFormatValidate(f, &why) && _Contains(why, "sorted"));
    std::printf("path order: %zu property pairs agree with composed text "
                "order\n",
                pairs);
}

/// A path tree 100000 prims deep validates and opens in time and memory
/// linear in its depth (composing every node's text would take about
/// 10 GB), and the listed order is decided at any depth.
void
TestDeepPaths()
{
    _context = "deep paths";
    constexpr uint32_t depth = 100000;
    RigExecWireFile f = _MinimalFile();
    f.names = {"", "Rig", "a", "p", "q"};
    uint32_t deepest = 1;  // /Rig
    for (uint32_t k = 0; k < depth; ++k) {
        f.paths.push_back(fb::PathNode(deepest, 2, PathKind::Prim));
        deepest = uint32_t(f.paths.size() - 1);
    }
    const uint32_t middle = 1 + depth / 2;
    f.paths.push_back(fb::PathNode(middle, 3, PathKind::Property));
    const uint32_t middleP = uint32_t(f.paths.size() - 1);
    f.paths.push_back(fb::PathNode(deepest, 3, PathKind::Property));
    const uint32_t deepP = uint32_t(f.paths.size() - 1);
    f.paths.push_back(fb::PathNode(deepest, 4, PathKind::Property));
    const uint32_t deepQ = uint32_t(f.paths.size() - 1);
    const auto slot = [](uint32_t name) {
        return fb::InputSlot(name, 0, -1, -1, InputTag::Double,
                             uint8_t(fb::InputSlotFlags::Listed));
    };
    // .../a.p < .../a/a/.../a.p < .../a.q
    f.inputs = {slot(middleP), slot(deepP), slot(deepQ)};
    f.listedInputs = 3;
    std::string why;
    CHECK(RigExecFormatValidate(f, &why));
    std::vector<uint8_t> bytes;
    CHECK(_Write(f, &bytes, &why));
    const auto opened = _Open(bytes, &why);
    CHECK(opened != nullptr);
    CHECK(RigExecFormatPathText(f, deepQ).size() == 4 + 2 * depth + 2);
    std::swap(f.inputs[1], f.inputs[2]);
    CHECK(!RigExecFormatValidate(f, &why) && _Contains(why, "sorted"));
    std::swap(f.inputs[0], f.inputs[2]);
    CHECK(!RigExecFormatValidate(f, &why) && _Contains(why, "sorted"));
    f.inputs = {slot(middleP), slot(deepP), slot(deepP)};
    CHECK(!RigExecFormatValidate(f, &why) &&
          _Contains(why, "a second slot for /Rig/a/a/a"));
    std::printf("deep paths: %u prims deep, %zu bytes\n", depth,
                bytes.size());
}

/// Every byte of the vtable of the table at \p table, as offsets into the
/// buffer at \p base: the two header entries and each field's.
std::vector<size_t>
_VtableBytes(const uint8_t *base, const uint8_t *table)
{
    const uint8_t *vtable =
        table - flatbuffers::ReadScalar<flatbuffers::soffset_t>(table);
    const uint16_t size = flatbuffers::ReadScalar<uint16_t>(vtable);
    std::vector<size_t> out;
    for (uint16_t at = 0; at < size; ++at) {
        out.push_back(size_t(vtable - base) + at);
    }
    return out;
}

/// The corruptions the format names, each made to the rich file and handed
/// to Open (the identifier, the old container and other versions are
/// TestOpenRefusals'): a truncated tail, refused at every length that
/// cuts more than trailing padding; a reserved revision op, a walk past
/// the slots, a constant of another tag than its read, a cluster past the
/// clusters in a cone list, a path node whose parent follows it, and
/// topology counts that do not sum to the kept entries, each refused
/// naming the defect; and every byte of the root table's vtable and of
/// the first revision's flipped, each refused with a reason or opening and
/// writing back to a file Open accepts.
void
TestCorruptionList()
{
    _context = "corruption list";
    std::vector<uint8_t> bytes;
    CHECK(_Write(_RichFile(), &bytes));
    std::string why;
    // A cut that only drops the builder's trailing alignment padding holds
    // the same file: it opens and writes back to the uncut bytes. Every
    // other cut is refused.
    size_t truncations = 0, truncationsRefused = 0, paddingCuts = 0;
    for (size_t cut = 1; cut < bytes.size(); cut += cut < 64 ? 1 : 61) {
        const std::vector<uint8_t> shorter(bytes.begin(), bytes.end() - cut);
        ++truncations;
        why.clear();
        const auto file = _Open(shorter, &why);
        if (!file) {
            CHECK(!why.empty());
            ++truncationsRefused;
            continue;
        }
        std::vector<uint8_t> rewritten;
        const bool padding = _Write(*file, &rewritten) && rewritten == bytes;
        if (!padding) {
            std::printf("  a tail %zu bytes short opened as another file\n",
                        cut);
        }
        CHECK(padding);
        ++paddingCuts;
    }
    CHECK(truncations > 64 && truncationsRefused > 64 &&
          truncationsRefused + paddingCuts == truncations);

    int cases = 0;
    using F = RigExecWireFile;
    const auto refused = [&](const char *name, const char *part,
                             const std::function<void(F &)> &mutate) {
        _context = std::string("corruption list: ") + name;
        F file = _RichFile();
        mutate(file);
        why.clear();
        const bool named =
            !_Open(_PackUnchecked(file), &why) && _Contains(why, _RegionDiagnostic(file, part));
        if (!named) {
            std::printf("  got: %s\n", why.empty() ? "(accepted)" : why.c_str());
        }
        CHECK(named);
        ++cases;
    };
    refused("op 10", "reserved",
            [](F &f) { f.geometry->chains[0].revisions[0].op = 10; });
    refused("walk out of range", "walk[0]",
            [](F &f) { f.pose->solvers[0].bend->walk = {7}; });
    refused("constant tag", "constant",
            [](F &f) { f.pose->solvers[0].roll->constant = _vFloatNaN; });
    refused("cone list cluster", "chain_base_clusters[0]",
            [](F &f) { f.cones->chainBaseClusters[0].v = {1}; });
    refused("parent after node", "malformed prim node",
            [](F &f) { f.paths[_pCtl] = fb::PathNode(5, 2, PathKind::Prim); });
    refused("topology sum", "kept entries", [](F &f) {
        f.geometry->chains[0].revisions[0].topology->counts8 = {2, 2};
    });
    refused("validated index range",
            "topology: validated, but kept entry 0 indexes influence 3 of 3",
            [](F &f) {
                fb::RigExecWireSkinTopology &t =
                    *f.geometry->chains[0].revisions[0].topology;
                t.validated = true;
                t.influenceCount = 3;
            });
    refused("raw arrays on a sparse layout", "raw_indices or raw_weights",
            [](F &f) {
                f.geometry->chains[0].revisions[0].topology->rawIndices = {
                    0};
            });

    _context = "corruption list: vtables";
    std::vector<uint64_t> aligned((bytes.size() + 7) / 8);
    std::memcpy(aligned.data(), bytes.data(), bytes.size());
    const uint8_t *base = reinterpret_cast<const uint8_t *>(aligned.data());
    const fb::File *root = flatbuffers::GetRoot<fb::File>(base);
    const fb::Revision *revision = nullptr;
    if (root->geometry() && root->geometry()->chains() &&
        root->geometry()->chains()->size() > 0) {
        const fb::Chain *chain = root->geometry()->chains()->Get(0);
        if (chain->revisions() && chain->revisions()->size() > 0) {
            revision = chain->revisions()->Get(0);
        }
    }
    CHECK(revision);
    if (!revision) {
        return;
    }
    size_t flips = 0, flipsRefused = 0, flipsOpened = 0;
    for (const uint8_t *table : {reinterpret_cast<const uint8_t *>(root),
                                 reinterpret_cast<const uint8_t *>(revision)}) {
        for (const size_t at : _VtableBytes(base, table)) {
            std::vector<uint8_t> flipped = bytes;
            flipped[at] ^= 0xff;
            ++flips;
            why.clear();
            const auto file = _Open(flipped, &why);
            if (!file) {
                ++flipsRefused;
                CHECK(!why.empty());
                continue;
            }
            ++flipsOpened;
            std::vector<uint8_t> rewritten;
            CHECK(_Write(*file, &rewritten) && _Open(rewritten) != nullptr);
        }
    }
    CHECK(flipsRefused > 0 && flipsRefused + flipsOpened == flips);
    std::printf("corruption list: %zu truncated tails refused, %zu cutting "
                "only trailing padding open as the same file; %d named "
                "defects refused by Open; %zu vtable bytes of the root and a "
                "revision flipped, %zu refused, %zu opened\n",
                truncationsRefused, paddingCuts, cases, flips, flipsRefused,
                flipsOpened);
}

/// Random multi-byte corruptions of the rich file, including retargeted
/// offsets: each is refused with a reason or opens and writes back; none
/// crashes. Exercises the bounding walk on unverified offsets, lengths and
/// vtables that no single flip produces.
void
TestCorruptions()
{
    _context = "corruptions";
    std::vector<uint8_t> bytes;
    CHECK(_Write(_RichFile(), &bytes));
    uint64_t state = 0x9e3779b97f4a7c15ull;
    const auto next = [&state] {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        return state;
    };
    const uint8_t interesting[] = {0x00, 0x01, 0x04, 0x08,
                                   0x7f, 0x80, 0xfe, 0xff};
    size_t refused = 0, opened = 0;
    constexpr int rounds = 20000;
    for (int round = 0; round < rounds; ++round) {
        std::vector<uint8_t> mutated = bytes;
        const int edits = 1 + int(next() % 4);
        for (int e = 0; e < edits; ++e) {
            const size_t at = next() % bytes.size();
            if (at >= 4 && at < 8) {
                continue;  // keep the identifier: reach the walk
            }
            switch (next() % 3) {
            case 0:
                mutated[at] = interesting[next() % 8];
                break;
            case 1:
                mutated[at] = uint8_t(next());
                break;
            default: {
                // An aligned word set to a small offset or length.
                const size_t word = at & ~size_t(3);
                const uint32_t small = uint32_t(next() % bytes.size());
                if (word >= 8 && word + 4 <= mutated.size()) {
                    std::memcpy(mutated.data() + word, &small, 4);
                }
                break;
            }
            }
        }
        std::string reason;
        const auto file = _Open(mutated, &reason);
        if (!file) {
            ++refused;
            CHECK(!reason.empty());
            continue;
        }
        ++opened;
        std::vector<uint8_t> rewritten;
        CHECK(_Write(*file, &rewritten) && _Open(rewritten) != nullptr);
    }
    std::printf("corruptions: %zu refused, %zu opened (of %d)\n", refused,
                opened, rounds);
}

#include "rigExecHeadFormatCases.h"

}  // namespace

int
main()
{
    TestMinimalRoundTrip();
    TestBitExactness();
    TestDeterminism();
    TestOpenRefusals();
    TestWriteRefuses();
    TestPathText();
    TestValidationSmoke();
    TestStepGraph();
    TestPresentationIdentifier();
    TestExternalMovers();
    TestSwitchOrder();
    TestVolumePlacementSteps();
    TestStepLabels();
    TestPhaseTables();
    TestSparseTopology();
    TestRawTopology();
    TestChunkTables();
    TestArrayInputs();
    TestHeadFormatCases();
    TestHeadComposeFormatCases();
    TestHeadDoubleFormatCases();
    TestHeadEnvelopeFormatCases();
    TestHeadChunkFormatCases();
    TestHeadTopologyFormatCases();
    TestBoundedOpen();
    TestPathOrder();
    TestDeepPaths();
    TestCorruptionList();
    TestCorruptions();
    if (_failures) {
        std::printf("testRigExecFormat: %d failures\n", _failures);
        return 1;
    }
    std::printf("testRigExecFormat: all passed\n");
    return 0;
}
