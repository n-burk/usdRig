// testRigExecFormat: the single-FlatBuffer .rigexec format on hand-built
// files. Write -> Open -> Write is byte-identical and keeps every stored bit
// (signed zeros, NaN payloads, denormals) of every floating-point field
// kind, value, pool and struct; Open refuses short buffers, the old
// container, a wrong file identifier, buffers the verifier rejects and
// other format versions, and buffers whose objects are shared or overlap,
// takes bytes at any address and keeps none of them, and survives every
// single-byte and many random multi-byte corruptions; writing is
// deterministic; the validator refuses a smoke set of rule violations, a
// step or cluster graph playback could not walk, and a presentation
// without its identifier, orders listed inputs as their composed texts,
// and handles a path tree 100000 prims deep.
// USD-free, like the format.
#include "rigExecBinary/format.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
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
_RichFile()
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
    // A recomposed parent version, a world source, the space at its last
    // version.
    sw.spaceSlot = 0;
    sw.parentRead = std::make_unique<fb::RigExecWireFrameVersion>();
    sw.parentRead->recompose = {0};
    sw.sourceReads.resize(1);
    sw.spaceRead = std::make_unique<fb::RigExecWireFrameVersion>();
    sw.spaceRead->anchor = 0;
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
    // One sample per float special, so the one F32 field carries each:
    // samples[k] holds _SF(k + 1).
    for (size_t k = 0; k < _specialCount; ++k) {
        fb::RigExecWireBlendSample copy = sample;
        copy.activationValue = _SF(k + 1);
        channel.samples.push_back(std::move(copy));
    }
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
    weight.values = _Fs(4, 2);
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
    weight.oracleSamples = 1;
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

/// Makes the rich file's revision a plugin mover, with its entry.
void
_AddPlugin(RigExecWireFile &file)
{
    file.geometry->chains[0].revisions[0].op =
        uint8_t(fb::RevisionOp::External);
    fb::RigExecWireExternalMover mover;
    mover.type = _pTokenPlugin;
    mover.epoch = {1, 2, 3};
    mover.inputs.push_back(_InValue(InputTag::Float, ReadMode::Baked));
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
    CHECK(_Same(o->steps[0].sizeUnits, file.steps[0].sizeUnits));
    CHECK(_Same(o->steps[0].cost, file.steps[0].cost));
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
    CHECK(samples.size() == _specialCount &&
          samples0.size() == _specialCount);
    for (size_t k = 0; k < samples.size() && k < samples0.size(); ++k) {
        CHECK(_Same(samples[k].activationValue, samples0[k].activationValue));
        CHECK(_Bits(samples[k].activationValue) ==
              _specialF[(k + 1) % _specialCount]);
    }
    if (samples.empty()) {
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
    CHECK(sw.spaceRead && sw.spaceRead->anchor == 0 &&
          sw.spaceRead->recompose.empty());
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
    CHECK(_Same(w.values, file.geometry->weightObjects[0].values) &&
          _Same(w.falloffCurve, file.geometry->weightObjects[0].falloffCurve));
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
    CHECK(_Bits(bs.activationValue) == _specialF[1]);
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

    // The format version, named before any other table is read.
    RigExecWireFile older = _RichFile();
    older.formatVersion = 3;
    CHECK(!_Open(_PackUnchecked(older), &why) &&
          _Contains(why, "format version 3"));
    older.formatVersion = 5;
    CHECK(!_Open(_PackUnchecked(older), &why) &&
          _Contains(why, "format version 5"));

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
        CHECK(_Contains(why, part));
        if (ok || !_Contains(why, part)) {
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
    // Avar bindings.
    expect("avar flat", "flat", [](F &f) { f.pose->avarBindings[0].flat = 11; });
    // Slot tables and constants.
    expect("slot kind size", "slot_kind",
           [](F &f) { f.slotMeta->slotKind.clear(); });
    expect("rotation sign", "rotation_sign",
           [](F &f) { f.constants->rotationSign = {8}; });
    // Steps, clusters, cones.
    expect("step cluster", "step 0 names cluster 1, which is no cluster",
           [](F &f) { f.steps[0].cluster = 1; });
    expect("cone list cluster", "chain_base_clusters[0]",
           [](F &f) { f.cones->chainBaseClusters[0].v = {1}; });
    expect("cluster set words", "cones.always",
           [](F &f) { f.cones->always->words = {0, 0}; });
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
}

/// The rich file with four steps in two clusters: 0 -> 1 -> 2 -> 3 and
/// 0 -> 3, steps 0 and 1 in cluster 0, steps 2 and 3 in cluster 1, so
/// cluster 0 -> cluster 1. Steps 1 and 2 write PosedM slots 0 and 1, and
/// step 3 reads both.
RigExecWireFile
_GraphFile()
{
    RigExecWireFile f = _RichFile();
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
        CHECK(!ok && why == text);
        if (ok || why != text) {
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
        CHECK(!opened && why == "invalid .rigexec: " + text);
        if (opened || why != "invalid .rigexec: " + text) {
            std::printf("  got '%s', expected 'invalid .rigexec: %s'\n",
                        opened ? "(opened)" : why.c_str(), text.c_str());
        }
    };
    // The two the runtime relies on most: a predecessor flipped to a later
    // step, and a two-cluster cycle.
    const auto flipped = [](F &f) { f.steps[1].preds = {2}; };
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
           [](F &f) { f.steps[1].preds = {1}; });
    expect("preds unsorted", "step 3 lists its predecessors out of order "
                             "or twice",
           [](F &f) { f.steps[3].preds = {2, 0}; });
    expect("preds repeated", "step 3 lists its predecessors out of order "
                             "or twice",
           [](F &f) { f.steps[3].preds = {0, 0}; });
    expect("succ earlier", "step 2 names earlier step 1 as a successor",
           [](F &f) { f.steps[2].succs = {1, 3}; });
    expect("pred without succ",
           "step 3 names predecessor 0, which does not name it as a "
           "successor",
           [](F &f) { f.steps[0].succs = {1}; });
    expect("succ without pred",
           "step 1 names successor 3, which does not name it as a "
           "predecessor",
           [](F &f) { f.steps[1].succs = {2, 3}; });
    expect("source after a step",
           "source step 1 depends on step 0, which is not a source",
           [](F &f) { f.steps[1].isSource = true; });
    // Indices past their tables, refused before they are followed.
    expect("pred past the steps",
           "step 3 names predecessor 4, which is no step",
           [](F &f) { f.steps[3].preds = {0, 4}; });
    expect("negative pred", "step 1 names predecessor -1, which is no step",
           [](F &f) { f.steps[1].preds = {-1}; });
    expect("succ past the steps", "step 2 names successor 4, which is no step",
           [](F &f) { f.steps[2].succs = {3, 4}; });
    expect("cluster past the clusters",
           "step 2 names cluster 2, which is no cluster",
           [](F &f) { f.steps[2].cluster = 2; });
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
        f.steps[3].reads = {fb::SlotRange(fb::SlotDomain::PosedM, 0, 3)};
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
               f.steps[1].reads = {fb::SlotRange(fb::SlotDomain::PosedM, 1, 2)};
           });
    expect("aggregate nothing writes",
           "step 3 reads Aggregate slots [0, 1), which no earlier step writes",
           [](F &f) {
               f.steps[3].reads.push_back(
                   fb::SlotRange(fb::SlotDomain::Aggregate, 0, 1));
           });
    // Step 2 becomes a source with no predecessor. It still reads step 1's
    // write, which index order would have produced, but the source pass
    // runs it before step 1.
    expect("source reads a later-running write",
           "step 2 reads PosedM slots [0, 1), which no earlier step writes",
           [](F &f) {
               f.steps[1].succs.clear();
               f.steps[2].preds.clear();
               f.steps[2].isSource = true;
               f.steps[2].reads = {fb::SlotRange(fb::SlotDomain::PosedM, 0, 1)};
           });
    // The domains a run holds before any step writes them need no producer.
    {
        _context = "step graph: unproduced source reads";
        F file = _GraphFile();
        file.steps[3].reads.push_back(
            fb::SlotRange(fb::SlotDomain::Avars, 0, 11));
        file.steps[3].reads.push_back(
            fb::SlotRange(fb::SlotDomain::Snapshots, 0, 3));
        file.steps[3].reads.push_back(
            fb::SlotRange(fb::SlotDomain::ChainBase, 0, 1));
        CHECK(RigExecFormatValidate(file, &why));
        if (!why.empty()) {
            std::printf("  source reads refused: %s\n", why.c_str());
        }
    }
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
    TestBoundedOpen();
    TestPathOrder();
    TestDeepPaths();
    TestCorruptions();
    if (_failures) {
        std::printf("testRigExecFormat: %d failures\n", _failures);
        return 1;
    }
    std::printf("testRigExecFormat: all passed\n");
    return 0;
}
