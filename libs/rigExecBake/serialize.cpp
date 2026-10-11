// .rigexec serialization.
#include "rigExecBake/serialize.h"
#include "rigExecBake/computedCapture.h"
#include "rigExecBake/pathTable.h"
#include "rigExecBake/staticCapture.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExecScene/providerRecordExport.h"

#include <algorithm>
#include <map>
#include <memory>
#include <tuple>
#include <type_traits>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {
namespace {

// The schema's enums mirror the program enums in order; the order is part
// of the format, so every enumerator is pinned here. A reordered program
// enum fails the build, not the bake.
#define _RIGEXEC_BAKE_PIN_STEP(name)                                       \
    static_assert(uint8_t(RigExecBakedStepKind::name) ==                   \
                      uint8_t(RigExecWireStepKind::name),                  \
                  "wire step order drifted")
_RIGEXEC_BAKE_PIN_STEP(ComposeSubtree);
_RIGEXEC_BAKE_PIN_STEP(Solve);
_RIGEXEC_BAKE_PIN_STEP(SolverCommit);
_RIGEXEC_BAKE_PIN_STEP(Constraint);
_RIGEXEC_BAKE_PIN_STEP(CommitDelta);
_RIGEXEC_BAKE_PIN_STEP(PropagateChunk);
_RIGEXEC_BAKE_PIN_STEP(CommitApply);
_RIGEXEC_BAKE_PIN_STEP(ProviderMatrix);
_RIGEXEC_BAKE_PIN_STEP(SnapshotFinals);
_RIGEXEC_BAKE_PIN_STEP(PoseInterpolator);
_RIGEXEC_BAKE_PIN_STEP(VolumePlacements);
_RIGEXEC_BAKE_PIN_STEP(WeightPacket);
_RIGEXEC_BAKE_PIN_STEP(InfluenceFold);
_RIGEXEC_BAKE_PIN_STEP(RevisionStatic);
_RIGEXEC_BAKE_PIN_STEP(RevisionChunk);
_RIGEXEC_BAKE_PIN_STEP(RevisionFuse);
_RIGEXEC_BAKE_PIN_STEP(ChainStatus);
_RIGEXEC_BAKE_PIN_STEP(Derived);
_RIGEXEC_BAKE_PIN_STEP(FrameMatrix);
_RIGEXEC_BAKE_PIN_STEP(PropertyRevision);
_RIGEXEC_BAKE_PIN_STEP(RestCompose);
_RIGEXEC_BAKE_PIN_STEP(LadderCompose);
_RIGEXEC_BAKE_PIN_STEP(SkinTopology);
_RIGEXEC_BAKE_PIN_STEP(ProviderRefresh);
#undef _RIGEXEC_BAKE_PIN_STEP

#define _RIGEXEC_BAKE_PIN_DOMAIN(name)                                     \
    static_assert(uint8_t(RigExecBakedSlotDomain::name) ==                 \
                      uint8_t(RigExecWireSlotDomain::name),                \
                  "wire domain order drifted")
_RIGEXEC_BAKE_PIN_DOMAIN(Avars);
_RIGEXEC_BAKE_PIN_DOMAIN(PoseBase);
_RIGEXEC_BAKE_PIN_DOMAIN(PoseFin);
_RIGEXEC_BAKE_PIN_DOMAIN(PosedM);
_RIGEXEC_BAKE_PIN_DOMAIN(FinalMatrix);
_RIGEXEC_BAKE_PIN_DOMAIN(BaseMatrix);
_RIGEXEC_BAKE_PIN_DOMAIN(Aggregate);
_RIGEXEC_BAKE_PIN_DOMAIN(SolverPoints);
_RIGEXEC_BAKE_PIN_DOMAIN(Candidates);
_RIGEXEC_BAKE_PIN_DOMAIN(CommitTable);
_RIGEXEC_BAKE_PIN_DOMAIN(CommitDelta);
_RIGEXEC_BAKE_PIN_DOMAIN(CommitStaging);
_RIGEXEC_BAKE_PIN_DOMAIN(ConstraintDelta);
_RIGEXEC_BAKE_PIN_DOMAIN(PropertyResult);
_RIGEXEC_BAKE_PIN_DOMAIN(ChainBase);
_RIGEXEC_BAKE_PIN_DOMAIN(RevisionPacket);
_RIGEXEC_BAKE_PIN_DOMAIN(RevisionTransforms);
_RIGEXEC_BAKE_PIN_DOMAIN(RevisionOut);
_RIGEXEC_BAKE_PIN_DOMAIN(RevisionDone);
_RIGEXEC_BAKE_PIN_DOMAIN(ChainDirty);
_RIGEXEC_BAKE_PIN_DOMAIN(ChainPoints);
_RIGEXEC_BAKE_PIN_DOMAIN(DerivedOut);
_RIGEXEC_BAKE_PIN_DOMAIN(WeightPacket);
_RIGEXEC_BAKE_PIN_DOMAIN(WeightFrames);
_RIGEXEC_BAKE_PIN_DOMAIN(PoseWeight);
_RIGEXEC_BAKE_PIN_DOMAIN(Snapshots);
_RIGEXEC_BAKE_PIN_DOMAIN(FrameMatrix);
_RIGEXEC_BAKE_PIN_DOMAIN(Rest);
_RIGEXEC_BAKE_PIN_DOMAIN(Ladder);
_RIGEXEC_BAKE_PIN_DOMAIN(SkinTopology);
#undef _RIGEXEC_BAKE_PIN_DOMAIN
// The program's last enumerators are the wire's: a program kind or domain
// appended without its wire value fails the build here.
static_assert(uint8_t(RigExecBakedStepKind::ProviderRefresh) ==
                  uint8_t(RigExecWireStepKind::MAX),
              "the wire step kinds end before the program's");
static_assert(RigExecBakedSlotDomainCount ==
                  size_t(RigExecWireSlotDomain::MAX) + 1,
              "the wire slot domains end before the program's");

static_assert(uint8_t(RigExecBakedSlotKind::FirstFramePose) ==
                  uint8_t(RigExecWireSlotKind::FirstFramePose),
              "wire slot-kind order drifted");
static_assert(uint8_t(RigExecBakedSlotKind::XformDerived) ==
                  uint8_t(RigExecWireSlotKind::XformDerived),
              "wire slot-kind order drifted");

std::vector<int32_t>
_ToI32s(const std::vector<int> &values)
{
    return std::vector<int32_t>(values.begin(), values.end());
}

RigExecWireMatrix4d
_ToMatrix(const GfMatrix4d &m)
{
    RigExecWireMatrix4d out;
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            out[size_t(r * 4 + c)] = m[r][c];
        }
    }
    return out;
}

RigExecWireVec3d
_ToVec3d(const GfVec3d &v)
{
    return RigExecWireVec3d{v[0], v[1], v[2]};
}

RigExecWireFrame
_ToFrame(const RigExecPointFrame &f)
{
    RigExecWireFrame out;
    for (size_t i = 0; i < 4; ++i) {
        out.points[i] = _ToVec3d(f.points[i]);
    }
    out.flags = f.flags;
    return out;
}

// The pose-table enums the file stores as raw bytes, pinned like the step
// and domain orders above: a reordered program enum fails the build.
#define _RIGEXEC_BAKE_PIN_EULER(name, value)                               \
    static_assert(uint8_t(RigExecEulerOrder::name) == value,               \
                  "wire euler order drifted")
_RIGEXEC_BAKE_PIN_EULER(XYZ, 0);
_RIGEXEC_BAKE_PIN_EULER(XZY, 1);
_RIGEXEC_BAKE_PIN_EULER(YXZ, 2);
_RIGEXEC_BAKE_PIN_EULER(YZX, 3);
_RIGEXEC_BAKE_PIN_EULER(ZXY, 4);
_RIGEXEC_BAKE_PIN_EULER(ZYX, 5);
#undef _RIGEXEC_BAKE_PIN_EULER
static_assert(uint8_t(RigExecScaleBlend::Log) == 0 &&
                  uint8_t(RigExecScaleBlend::Linear) == 1,
              "wire scale-blend order drifted");
static_assert(uint8_t(RigExecSingleChainIkMode::RotatePlane) == 0 &&
                  uint8_t(RigExecSingleChainIkMode::SingleChain) == 1,
              "wire IK-mode order drifted");
static_assert(uint8_t(RigExecSplineIkRestLength::Curve) == 0 &&
                  uint8_t(RigExecSplineIkRestLength::Chain) == 1,
              "wire spline rest-length order drifted");
static_assert(uint8_t(RigExecRbfKernel::Gaussian) == 0 &&
                  uint8_t(RigExecRbfKernel::Linear) == 1,
              "wire RBF kernel order drifted");
static_assert(uint8_t(RigExecRbfPoseType::Whole) == 0 &&
                  uint8_t(RigExecRbfPoseType::Swing) == 1 &&
                  uint8_t(RigExecRbfPoseType::Twist) == 2,
              "wire RBF pose-type order drifted");

RigExecWireVec3f
_ToVec3f(const GfVec3f &v)
{
    return RigExecWireVec3f{v[0], v[1], v[2]};
}

RigExecWireAncestorRead
_ToAncestorRead(
    const RigExecBakedCommit::AncestorRead &read)
{
    RigExecWireAncestorRead out;
    out.slot = int32_t(read.slot);
    out.fin = read.fin;
    out.base = read.base;
    return out;
}

#define _RIGEXEC_BAKE_PIN_OP(name, value)                                  \
    static_assert(uint8_t(RigExecRevisionOp::name) == value,               \
                  "wire revision-op order drifted")
_RIGEXEC_BAKE_PIN_OP(Matrix, 0);
_RIGEXEC_BAKE_PIN_OP(Skin, 1);
_RIGEXEC_BAKE_PIN_OP(BlendShape, 2);
_RIGEXEC_BAKE_PIN_OP(VolumeCorrect, 3);
_RIGEXEC_BAKE_PIN_OP(Smooth, 4);
_RIGEXEC_BAKE_PIN_OP(Lattice, 5);
_RIGEXEC_BAKE_PIN_OP(SurfaceProject, 6);
_RIGEXEC_BAKE_PIN_OP(Ribbon, 7);
_RIGEXEC_BAKE_PIN_OP(Wire, 8);
_RIGEXEC_BAKE_PIN_OP(EmitGuidePoints, 9);
_RIGEXEC_BAKE_PIN_OP(RecomputeNormals, 12);
_RIGEXEC_BAKE_PIN_OP(RecomputeExtent, 13);
_RIGEXEC_BAKE_PIN_OP(DeltaMush, 14);
_RIGEXEC_BAKE_PIN_OP(Wrinkle, 15);
#undef _RIGEXEC_BAKE_PIN_OP
#define _RIGEXEC_BAKE_PIN_PHASE(name, value)                               \
    static_assert(uint8_t(RigExecReadPhaseKind::name) == value,            \
                  "wire read-phase order drifted")
_RIGEXEC_BAKE_PIN_PHASE(Base, 0);
_RIGEXEC_BAKE_PIN_PHASE(Preceding, 1);
_RIGEXEC_BAKE_PIN_PHASE(Final, 2);
_RIGEXEC_BAKE_PIN_PHASE(AtPrim, 3);
#undef _RIGEXEC_BAKE_PIN_PHASE

std::unique_ptr<fb::RigExecWireInput>
_FbInputPtr(const fb::RigExecWireInput &in)
{
    return std::make_unique<fb::RigExecWireInput>(in);
}

RigExecWireLandmarks
_ToLandmarks(const std::array<GfVec3d, 4> &points)
{
    RigExecWireLandmarks out;
    for (size_t i = 0; i < 4; ++i) {
        out[i] = _ToVec3d(points[i]);
    }
    return out;
}

std::vector<fb::RigExecWireIntList>
_FbIntLists(const std::vector<std::vector<int>> &rows)
{
    std::vector<fb::RigExecWireIntList> out(rows.size());
    for (size_t k = 0; k < rows.size(); ++k) {
        out[k].v = _ToI32s(rows[k]);
    }
    return out;
}

fb::RigExecWireClusterSet
_FbClusterSet(const RigExecBakedClusterSet &set, uint32_t clusters)
{
    fb::RigExecWireClusterSet out;
    out.clusters = clusters;
    out.words = set.words;
    return out;
}

std::vector<fb::SlotRange>
_FbRanges(const std::vector<RigExecBakedSlotRange> &ranges)
{
    std::vector<fb::SlotRange> out;
    out.reserve(ranges.size());
    for (const RigExecBakedSlotRange &range : ranges) {
        out.emplace_back(fb::SlotDomain(uint8_t(range.domain)), range.begin,
                         range.end);
    }
    return out;
}

std::vector<fb::RigExecWireAncestorReadList>
_FbAncestorLists(
    const std::vector<std::vector<RigExecBakedCommit::AncestorRead>> &rows)
{
    std::vector<fb::RigExecWireAncestorReadList> out(rows.size());
    for (size_t k = 0; k < rows.size(); ++k) {
        out[k].v.reserve(rows[k].size());
        for (const auto &read : rows[k]) {
            out[k].v.push_back(_ToAncestorRead(read));
        }
    }
    return out;
}

/// What a fill reads from and interns into, and its first failure.
class _FileFill {
public:
    _FileFill(const RigExecBakedProgramImpl &program,
              const RigExecBakeInputs &inputs, RigExecBakePathTable *paths, RigExecBakePools *pools)
        : _program(program), _inputs(inputs), _paths(paths), _pools(pools)
    {
    }

    bool Run(fb::RigExecWireFile *file, std::string *error);

private:
    using _Family = RigExecBakeReadFamily;

    bool _Fail(const std::string &what)
    {
        if (_error.empty()) {
            _error = what;
        }
        return false;
    }
    uint32_t _Path(const SdfPath &path) { return _paths->Path(path); }
    uint32_t _Tok(const TfToken &token) { return _paths->Token(token); }
    RigExecWireReadPhase _Phase(const RigExecReadPhase &phase)
    {
        RigExecWireReadPhase out;
        out.kind = uint8_t(phase.kind);
        out.prim = _Path(phase.prim);
        return out;
    }
    fb::RigExecWirePointsBinding
    _PointsBinding(const RigExecBakedPointsBinding &binding);

    bool _Index();
    std::unique_ptr<fb::RigExecWireInput> _Registered(_Family family,
                                                      size_t object,
                                                      size_t field);
    fb::RigExecWireInput _RegisteredValue(_Family family, size_t object,
                                          size_t field);

    void _SlotMeta(fb::RigExecWireSlotMeta *meta);
    void _Constants(fb::RigExecWireConstants *constants);
    void _Steps(std::vector<fb::RigExecWireStep> *steps);
    void _Clustering(fb::RigExecWireClustering *clustering);
    void _Cones(fb::RigExecWireCones *cones);
    void _Pose(fb::RigExecWireDomainPose *pose);
    void _Solver(size_t index, fb::RigExecWireSolver *out);
    void _Constraint(size_t index, fb::RigExecWireConstraint *out);
    void _Geometry(fb::RigExecWireDomainGeometry *geometry);
    void _Revision(const RigExecBakedProgramImpl::GeomRevision &revision,
                   size_t chain, size_t index, bool derived,
                   fb::RigExecWireRevision *out);
    std::unique_ptr<fb::RigExecWireSkinTopology>
    _Topology(const RigExecSkinTopology &topology,
              const RigExecBakedProgramImpl::GeomRevision &revision);
    void _Inputs(fb::RigExecWireFile *file);

    const RigExecBakedProgramImpl &_program;
    const RigExecBakeInputs &_inputs;
    RigExecBakePools *_pools;
    RigExecBakePathTable *_paths;
    std::string _error;
    std::map<std::tuple<uint8_t, uint32_t, uint32_t>,
             const fb::RigExecWireInput *>
        _registered;
    std::map<std::tuple<uint32_t, uint32_t, bool, uint32_t>,
             const RigExecBakeBlendRead *>
        _blendReads;
    std::map<std::pair<uint32_t, uint32_t>, const fb::RigExecWireInput *>
        _defaultWeightReads;
    std::map<std::tuple<uint32_t, uint32_t, bool, uint32_t, uint32_t>,
             const fb::RigExecWireInput *>
        _blendPoints;
    std::map<std::pair<uint32_t, uint32_t>, const RigExecBakeLayoutSlots *>
        _layoutSlots;
};

bool
_FileFill::_Index()
{
    for (const RigExecBakeRegisteredRead &entry : _inputs.registeredReads) {
        const auto key = std::make_tuple(uint8_t(entry.family), entry.object,
                                         entry.field);
        if (!_registered.emplace(key, &entry.read).second) {
            return _Fail("the inputs hold two registered reads for one table "
                         "field");
        }
    }
    for (const RigExecBakeBlendRead &entry : _inputs.blendWeightReads) {
        const auto key = std::make_tuple(entry.chain, entry.revision,
                                         entry.derived, entry.channel);
        if (!_blendReads.emplace(key, &entry).second) {
            return _Fail("the inputs hold two blend weight reads for one "
                         "channel");
        }
    }
    for (const RigExecBakeDefaultWeightRead &entry :
         _inputs.defaultWeightReads) {
        if (!_defaultWeightReads
                 .emplace(std::make_pair(entry.chain, entry.revision),
                          &entry.read)
                 .second) {
            return _Fail("the inputs hold two default weight reads for one "
                         "revision");
        }
    }
    for (const RigExecBakeBlendPointsRead &entry : _inputs.blendPoints) {
        const auto key = std::make_tuple(entry.chain, entry.revision,
                                         entry.derived, entry.channel,
                                         entry.sample);
        if (!_blendPoints.emplace(key, &entry.read).second) {
            return _Fail("the inputs hold two points reads for one blend "
                         "sample");
        }
    }
    for (const RigExecBakeLayoutSlots &entry : _inputs.layoutSlots) {
        if (!_layoutSlots
                 .emplace(std::make_pair(entry.chain, entry.revision), &entry)
                 .second) {
            return _Fail("the inputs hold two layouts for one revision");
        }
    }
    if (_inputs.chainBaseSlots.size() != _program.chains.size()) {
        return _Fail("the inputs hold no base input per chain");
    }
    return true;
}

fb::RigExecWireInput
_FileFill::_RegisteredValue(_Family family, size_t object, size_t field)
{
    const auto found = _registered.find(
        std::make_tuple(uint8_t(family), uint32_t(object), uint32_t(field)));
    if (found == _registered.end()) {
        _Fail("no registered read for family " +
              std::to_string(unsigned(family)) + " row " +
              std::to_string(object) + " field " + std::to_string(field));
        return fb::RigExecWireInput();
    }
    return *found->second;
}

std::unique_ptr<fb::RigExecWireInput>
_FileFill::_Registered(_Family family, size_t object, size_t field)
{
    return std::make_unique<fb::RigExecWireInput>(
        _RegisteredValue(family, object, field));
}

fb::RigExecWirePointsBinding
_FileFill::_PointsBinding(const RigExecBakedPointsBinding &binding)
{
    // The binding as Build made it; its id is the program's test capture
    // key and stays behind.
    fb::RigExecWirePointsBinding out;
    out.inputPath = _Path(binding.input);
    out.phase = _Phase(binding.phase);
    out.candidates.reserve(binding.candidates.size());
    for (const RigExecBakedPointVersion &candidate : binding.candidates) {
        out.candidates.emplace_back(int32_t(candidate.chain),
                                    int32_t(candidate.version));
    }
    out.finalRead = binding.finalRead;
    out.diagnoseMiss = binding.diagnoseMiss;
    return out;
}

void
_FileFill::_SlotMeta(fb::RigExecWireSlotMeta *meta)
{
    const RigExecBakedProgramImpl &program = _program;
    meta->paths.reserve(program.paths.size());
    for (const SdfPath &path : program.paths) {
        meta->paths.push_back(_Path(path));
    }
    meta->slotKind.reserve(program.slotKind.size());
    for (RigExecBakedSlotKind kind : program.slotKind) {
        meta->slotKind.push_back(fb::SlotKind(uint8_t(kind)));
    }
    meta->providerActive.reserve(program.providerActive.size());
    for (char active : program.providerActive) {
        meta->providerActive.push_back(active ? uint8_t(1) : uint8_t(0));
    }
    meta->parent = _ToI32s(program.parent);
    meta->propParent = _ToI32s(program.propParent);
    meta->xformSlots = _ToI32s(program.xformSlots);
    meta->xformPaths.reserve(program.xformPrimsBySlot.size());
    for (const UsdPrim &prim : program.xformPrimsBySlot) {
        meta->xformPaths.push_back(_Path(prim.GetPath()));
    }
    meta->publicationRoles.assign(program.paths.size(), uint8_t(0));
    for (int slot : program.jointSlots) meta->publicationRoles[size_t(slot)] |= uint8_t(1);
    for (int slot : program.controlSlots) meta->publicationRoles[size_t(slot)] |= uint8_t(2);
    meta->solverGuidesEnabled = program.solverGuidesEnabled && *program.solverGuidesEnabled;
    meta->jointSlots = _ToI32s(program.jointSlots);
    meta->jointPaths.reserve(program.jointPaths.size());
    for (const SdfPath &path : program.jointPaths) {
        meta->jointPaths.push_back(_Path(path));
    }
    meta->controlSlots = _ToI32s(program.controlSlots);
    meta->controlPaths.reserve(program.controlPaths.size());
    for (const SdfPath &path : program.controlPaths) {
        meta->controlPaths.push_back(_Path(path));
    }
    meta->solverArrayPaths.reserve(program.solverArrays.size());
    meta->solverArrayElements.reserve(program.solverArrays.size());
    for (const auto &entry : program.solverArrays) {
        meta->solverArrayPaths.push_back(_Path(entry.first));
        meta->solverArrayElements.push_back(int32_t(entry.second));
    }
    meta->jointPublishOrder = _ToI32s(program.jointPublishOrder);
    meta->controlPublishOrder = _ToI32s(program.controlPublishOrder);
    meta->solverPublishOrder = _ToI32s(program.solverPublishOrder);
    meta->jointPathsAscending = program.jointPathsAscending;
    meta->controlPathsAscending = program.controlPathsAscending;
    meta->solverArraysAscending = program.solverArraysAscending;
    meta->needFinal.reserve(program.needFinal.size());
    for (char v : program.needFinal) {
        meta->needFinal.push_back(v ? uint8_t(1) : uint8_t(0));
    }
    meta->needBase.reserve(program.needBase.size());
    for (char v : program.needBase) {
        meta->needBase.push_back(v ? uint8_t(1) : uint8_t(0));
    }
}

void
_FileFill::_Constants(fb::RigExecWireConstants *constants)
{
    const RigExecBakedProgramImpl &program = _program;
    constants->restM.reserve(program.restM.size());
    for (const GfMatrix4d &m : program.restM) {
        constants->restM.push_back(_ToMatrix(m));
    }
    constants->restPts.reserve(program.restPts.size());
    for (const std::array<GfVec3d, 4> &pts : program.restPts) {
        constants->restPts.push_back(_ToLandmarks(pts));
    }
    constants->restFrames.reserve(program.restFrames.size());
    for (const RigExecPointFrame &f : program.restFrames) {
        constants->restFrames.push_back(_ToFrame(f));
    }
    constants->selfD.reserve(program.selfD.size());
    for (const GfMatrix4d &m : program.selfD) {
        constants->selfD.push_back(_ToMatrix(m));
    }
    for (const auto &m : program.posedD) constants->posedD.push_back(_ToMatrix(m));
    for (const auto &m : program.parentSpaceM) constants->parentSpaceM.push_back(_ToMatrix(m));
    for (char value : program.parentSpaceAuthored)
        constants->parentSpaceAuthored.push_back(value ? uint8_t(1) : uint8_t(0));
    constants->parentDinv.reserve(program.parentDinv.size());
    for (const GfMatrix4d &m : program.parentDinv) {
        constants->parentDinv.push_back(_ToMatrix(m));
    }
    constants->rotOrder.reserve(program.rotOrder.size());
    for (const TfToken &token : program.rotOrder) {
        constants->rotOrder.push_back(_Tok(token));
    }
    constants->restRoundTrip.reserve(program.restRoundTrip.size());
    for (const GfMatrix4d &m : program.restRoundTrip) {
        constants->restRoundTrip.push_back(_ToMatrix(m));
    }
    constants->defaultRoundTrip.reserve(program.defaultRoundTrip.size());
    for (const GfMatrix4d &m : program.defaultRoundTrip) {
        constants->defaultRoundTrip.push_back(_ToMatrix(m));
    }
    constants->posedAuthored.reserve(program.posedAuthored.size());
    for (char v : program.posedAuthored) {
        constants->posedAuthored.push_back(v ? uint8_t(1) : uint8_t(0));
    }
    constants->posedAuthoredM.reserve(program.posedAuthoredM.size());
    for (const GfMatrix4d &m : program.posedAuthoredM) {
        constants->posedAuthoredM.push_back(_ToMatrix(m));
    }
    constants->noScaleAvars.reserve(program.noScaleAvars.size());
    for (char v : program.noScaleAvars) {
        constants->noScaleAvars.push_back(v ? uint8_t(1) : uint8_t(0));
    }
    constants->avarConstants = program.avarConstants;
    constants->rotationSign.assign(program.rotationSign.begin(),
                                   program.rotationSign.end());
}

void
_FileFill::_Steps(std::vector<fb::RigExecWireStep> *steps)
{
    steps->reserve(_program.steps.size());
    for (const RigExecBakedStep &step : _program.steps) {
        fb::RigExecWireStep out;
        out.kind = fb::StepKind(uint8_t(step.kind));
        out.object = int32_t(step.object);
        out.part = int32_t(step.part);
        out.descriptorKey=step.descriptorKey;
        out.semanticPredecessorKeys = step.semanticPredecessorKeys;
        out.reads = _FbRanges(step.reads);
        out.writes = _FbRanges(step.writes);
        out.preds = _ToI32s(step.preds);
        out.succs = _ToI32s(step.succs);
        out.isSource = step.isSource;
        out.isHead = step.isHead;
        const size_t index = steps->size();
        if (index < _inputs.headInputSlots.size())
            out.headInputSlots = _inputs.headInputSlots[index];
        if (index < _inputs.headInputReads.size())
            out.headInputReads = _inputs.headInputReads[index];
        out.headVaryingLeaves = step.varyingLeaves;
        out.headAlwaysRuns = step.alwaysRuns;
        for (const auto &[version, record] : step.shadowedReads)
            out.shadowedReads.emplace_back(int32_t(version), int32_t(record));
        out.externalReads = step.externalReads;
        out.varyingInputs = step.varyingInputs;
        out.overrideInputs = _ToI32s(step.overrideInputs);
        out.cluster = int32_t(step.cluster);
        out.level = int32_t(step.level);
        out.sizeUnits = step.sizeUnits;
        out.cost = step.cost;
        out.maxDiagnostics = uint32_t(step.maxDiagnostics);
        steps->push_back(std::move(out));
    }
}

void
_FileFill::_Clustering(fb::RigExecWireClustering *clustering)
{
    clustering->clusters.reserve(_program.clustering.clusters.size());
    for (const RigExecBakedCluster &cluster : _program.clustering.clusters) {
        fb::RigExecWireCluster out;
        out.members = _ToI32s(cluster.members);
        out.preds = _ToI32s(cluster.preds);
        out.succs = _ToI32s(cluster.succs);
        out.cost = cluster.cost;
        out.level = int32_t(cluster.level);
        clustering->clusters.push_back(std::move(out));
    }
    clustering->clusterOf = _ToI32s(_program.clustering.clusterOf);
    clustering->grainUs = _program.clustering.grainUs;
    clustering->serialCost = _program.clustering.serialCost;
    clustering->criticalPathCost = _program.clustering.criticalPathCost;
}

void
_FileFill::_Cones(fb::RigExecWireCones *cones)
{
    const uint32_t clusters = uint32_t(_program.clustering.clusters.size());
    cones->always = std::make_unique<fb::RigExecWireClusterSet>(
        _FbClusterSet(_program.cones.always, clusters));
    cones->poseClusters = std::make_unique<fb::RigExecWireClusterSet>(
        _FbClusterSet(_program.cones.poseClusters, clusters));
    cones->avarCluster = _ToI32s(_program.cones.avarCluster);
    cones->chainBaseClusters = _FbIntLists(_program.cones.chainBaseClusters);
    cones->revisionClusters = _FbIntLists(_program.cones.revisionClusters);
    cones->revisionStaticCluster = _ToI32s(_program.cones.revisionStaticCluster);
    cones->varyingSteps = _ToI32s(_program.cones.varyingSteps);
    cones->overrideSteps = _ToI32s(_program.cones.overrideSteps);
}

void
_FileFill::_Solver(size_t index, fb::RigExecWireSolver *out)
{
    const RigExecBakedProgramImpl::Solver &solver = _program.solvers[index];
    const auto read = [&](size_t field) {
        return _Registered(_Family::Solver, index, field);
    };
    out->path = _Path(solver.path);
    out->type = _Tok(solver.type);
    out->restSlots = _ToI32s(solver.restSlots);
    out->restRefs.reserve(solver.restRefs.size());
    for (const auto &ref : solver.restRefs) {
        out->restRefs.emplace_back(int32_t(ref.first), int32_t(ref.second));
    }
    out->restIsLive.reserve(solver.restIsLive.size());
    for (size_t i = 0; i < solver.restIsLive.size(); ++i) {
        out->restIsLive.push_back(solver.restIsLive[i] ? uint8_t(1)
                                                       : uint8_t(0));
    }
    out->restReads = solver.restReads;
    out->hasLiveRest = solver.hasLiveRest;
    out->jointRests.reserve(solver.jointRests.size());
    for (const std::array<GfVec3d, 4> &set : solver.jointRests) {
        out->jointRests.push_back(_ToLandmarks(set));
    }
    out->restsVary = solver.restsVary;
    out->restOverrides = _ToI32s(solver.restOverrides);
    out->splineRestWeights = solver.splineRestWeights;
    out->splineRestMode = uint8_t(solver.splineRestMode);
    out->degenerate = solver.degenerate;
    out->controls = _ToI32s(solver.controls);
    out->parentRelative = solver.parentRelative;
    out->controlRests.reserve(solver.controlRests.size());
    for (const std::array<GfVec3d, 4> &set : solver.controlRests) {
        out->controlRests.push_back(_ToLandmarks(set));
    }
    out->start = int32_t(solver.start);
    out->startRest = _ToLandmarks(solver.startRest);
    out->startRead = solver.startRead;
    out->root = int32_t(solver.root);
    out->mid = int32_t(solver.mid);
    out->end = int32_t(solver.end);
    out->pole = int32_t(solver.pole);
    out->ikRests.reserve(3);
    for (size_t k = 0; k < 3; ++k) {
        out->ikRests.push_back(_ToLandmarks(solver.ikRests[k]));
    }
    out->ikParams.upperLength = solver.ikParams.upperLength;
    out->ikParams.lowerLength = solver.ikParams.lowerLength;
    out->ikParams.stretch = solver.ikParams.stretch;
    out->ikParams.softness = solver.ikParams.softness;
    out->ikParams.preferredBendRadians = solver.ikParams.preferredBendRadians;
    // The registered fields, numbered as the collection numbers them.
    out->bend = read(0);
    out->upperOffset = read(1);
    out->lowerOffset = read(2);
    out->stretch = read(3);
    out->softness = read(4);
    out->blendWeight = read(5);
    out->preserveVolume = read(6);
    out->midFollowWeight = read(7);
    out->roll = read(8);
    out->twist = read(9);
    out->minLengthRatio = read(10);
    out->twistTurns = read(11);
    out->ribbonSampleCount = read(12);
    out->ikSpace = read(13);
    out->pin=read(14);out->upperScale=read(15);out->lowerScale=read(16);
    out->softDistance=read(17);out->limbTwist=read(18);
    out->scaleSegments=solver.scaleSegments;
    out->softDistancePolicy=solver.ikParams.softDistancePolicy;
    out->ikScaleSegments=solver.ikParams.scaleSegments;
    out->scaleCalibration=solver.ikParams.limb.scaleCalibration;
    out->upperLengthBase = solver.upperLengthBase;
    out->lowerLengthBase = solver.lowerLengthBase;
    out->spaceSlot = int32_t(solver.spaceSlot);
    out->spaceRest = _ToLandmarks(solver.spaceRest);
    out->spaceRead = uint32_t(std::max(solver.spaceRead, 0));
    out->inA = int32_t(solver.inA);
    out->inB = int32_t(solver.inB);
    out->solveDescriptorKey = solver.solveDescriptorKey;
    out->relationshipRequirements.reserve(solver.relationshipRequirements.size());
    for (const auto &requirement : solver.relationshipRequirements) {
        fb::RigExecWireSolverRelationshipRequirement wire;
        wire.port = requirement.first;
        wire.solver = int32_t(requirement.second);
        out->relationshipRequirements.push_back(std::move(wire));
    }
    out->scaleMode = uint8_t(solver.scaleMode);
    out->blendRotationRejected = solver.blendRotationRejected;
    auto rest = std::make_unique<fb::RigExecWireSplineIkRest>();
    for (size_t i = 0; i < 4; ++i) {
        rest->cvs[i] = _ToVec3d(solver.splineRest.cvs[i]);
    }
    rest->rootControl = _ToFrame(solver.splineRest.rootControl);
    rest->midControl = _ToFrame(solver.splineRest.midControl);
    rest->endControl = _ToFrame(solver.splineRest.endControl);
    rest->joints.reserve(solver.splineRest.joints.size());
    for (const RigExecPointFrame &joint : solver.splineRest.joints) {
        rest->joints.push_back(_ToFrame(joint));
    }
    rest->segmentLengths = solver.splineRest.segmentLengths;
    rest->restArcLength = solver.splineRest.restArcLength;
    rest->volumeWeights = solver.splineRest.volumeWeights;
    out->splineRest = std::move(rest);
    out->splineParams.preserveVolume = solver.splineParams.preserveVolume;
    out->splineParams.midFollowWeight = solver.splineParams.midFollowWeight;
    out->splineParams.roll = solver.splineParams.roll;
    out->splineParams.twist = solver.splineParams.twist;
    out->splineParams.minLengthRatio = solver.splineParams.minLengthRatio;
    out->splineParams.aimRootTangent = solver.splineParams.aimRootTangent;
    out->splineJointRests.reserve(solver.splineJointRests.size());
    for (const std::array<GfVec3d, 4> &set : solver.splineJointRests) {
        out->splineJointRests.push_back(_ToLandmarks(set));
    }
    out->splineCount = uint64_t(solver.splineCount);
    out->splineParamsVary = solver.splineParamsVary;
    out->twistStartRest = _ToLandmarks(solver.twistStartRest);
    out->twistEndRest = _ToLandmarks(solver.twistEndRest);
    out->twistWeights = solver.twistWeights;
    out->ribbonPointsPath = _Path(solver.ribbonPointsPath);
    out->ribbonRestPoints.reserve(solver.ribbonRestPoints.size());
    for (const GfVec3f &p : solver.ribbonRestPoints) {
        out->ribbonRestPoints.push_back(_ToVec3f(p));
    }
    // The constant; the static capture puts a varying driver's run points
    // here instead.
    out->ribbonConstantPoints.reserve(solver.ribbonConstantPoints.size());
    for (const GfVec3f &p : solver.ribbonConstantPoints) {
        out->ribbonConstantPoints.push_back(_ToVec3f(p));
    }
    out->outputs.reserve(solver.outputs.size());
    for (const auto &output : solver.outputs) {
        out->outputs.emplace_back(int32_t(output.first),
                                  int32_t(output.second));
    }
    out->outPosition = _ToI32s(solver.outPosition);
    out->controlReads = solver.controlReads;
    out->rootRead = solver.rootRead;
    out->midRead = solver.midRead;
    out->endRead = solver.endRead;
    out->poleRead = solver.poleRead;
}

void
_FileFill::_Constraint(size_t index, fb::RigExecWireConstraint *out)
{
    const RigExecBakedProgramImpl::Constraint &constraint =
        _program.constraints[index];
    const auto read = [&](size_t field) {
        return _Registered(_Family::Constraint, index, field);
    };
    out->path = _Path(constraint.path);
    out->type = _Tok(constraint.type);
    out->weightObject = _Path(constraint.weightObject);
    out->weightField = constraint.weightField;
    out->weightObjectIndex =
        index < _inputs.constraintWeightObjectIndex.size()
            ? _inputs.constraintWeightObjectIndex[index]
            : -1;
    if (index >= _inputs.constraintWeightObjectIndex.size()) {
        _Fail("the inputs hold no envelope index for constraint " +
              constraint.path.GetString());
    }
    out->target = int32_t(constraint.target);
    out->targetSlots = _ToI32s(constraint.targetSlots);
    out->sources = _ToI32s(constraint.sources);
    out->sourceNatives = _ToI32s(constraint.sourceNatives);
    out->sourcePaths.reserve(constraint.sourcePaths.size());
    for (const SdfPath &path : constraint.sourcePaths) {
        out->sourcePaths.push_back(_Path(path));
    }
    out->arrays = int32_t(constraint.arrays);
    // The registered fields, numbered as the collection numbers them.
    out->enabled = read(0);
    out->defaultWeight = read(1);
    out->offset = read(2);
    out->affectX = read(3);
    out->affectY = read(4);
    out->affectZ = read(5);
    out->tX = read(6);
    out->tY = read(7);
    out->tZ = read(8);
    out->rX = read(9);
    out->rY = read(10);
    out->rZ = read(11);
    out->sX = read(12);
    out->sY = read(13);
    out->sZ = read(14);
    out->aimVector = read(15);
    out->upVector = read(16);
    out->rotationOffset = read(17);
    out->worldUpVector = read(18);
    out->poleVector = read(19);
    out->twistDegrees = read(20);
    out->stretch=read(21);
    out->order = uint8_t(constraint.order);
    out->aimAxisFallback = _ToVec3d(constraint.aimAxisFallback);
    out->aimVectorAuthored = constraint.aimVectorAuthored;
    out->preserveInputUp = constraint.preserveInputUp;
    out->worldUpType = _Tok(constraint.worldUpType);
    out->sceneUp = _ToVec3d(constraint.sceneUp);
    out->pointsTarget = _Path(constraint.pointsTarget);
    out->deltaBasePath = _Path(constraint.deltaBasePath);
    out->deltaBase = int32_t(constraint.deltaBase);
    out->worldUpObject = int32_t(constraint.worldUpObject);
    out->worldUpNative = int32_t(constraint.worldUpNative);
    out->worldUpPath = _Path(constraint.worldUpPath);
    out->worldUpObjectNamed = constraint.worldUpObjectNamed;
    out->spaceSlot = int32_t(constraint.spaceSlot);
    out->flags = uint8_t(
        (constraint.blendShear ? RigExecWireConstraintBlendShear : 0) |
        (constraint.worldUpRotationOnly
             ? RigExecWireConstraintWorldUpRotationOnly
             : 0) |
        (constraint.radialBlend ? RigExecWireConstraintRadialBlend : 0));
    out->singleChainIk = constraint.singleChainIk;
    out->ikMode = uint8_t(constraint.ikMode);
    out->poleModeObject = constraint.poleModeObject;
    out->useAnimatedTs = constraint.useAnimatedTs;
    out->ikRestLive.reserve(constraint.ikRestLive.size());
    for (char v : constraint.ikRestLive) {
        out->ikRestLive.push_back(v ? uint8_t(1) : uint8_t(0));
    }
    out->effector = int32_t(constraint.effector);
    out->effectorNative = int32_t(constraint.effectorNative);
    out->effectorPath = _Path(constraint.effectorPath);
    out->poleObjects = _ToI32s(constraint.poleObjects);
    out->poleObjectNatives = _ToI32s(constraint.poleObjectNatives);
}

void
_FileFill::_Pose(fb::RigExecWireDomainPose *pose)
{
    const RigExecBakedProgramImpl &program = _program;
    pose->requiredStageFramesAdmission = std::make_unique<fb::RigExecWireRequiredStageFramesAdmission>();
    pose->requiredStageFramesAdmission->admitted = program.requiredStageFramesAdmission.admitted;
    pose->requiredStageFramesAdmission->firstBadTarget = program.requiredStageFramesAdmission.firstBadTarget;
    pose->ladders.resize(program.ladders.size());
    for (size_t i = 0; i < program.ladders.size(); ++i) {
        fb::RigExecWireLadder &out = pose->ladders[i];
        out.restSpace = _Registered(_Family::Ladder, i, 0);
        out.defaultSpace = _Registered(_Family::Ladder, i, 1);
        out.posedSpace = _Registered(_Family::Ladder, i, 2);
        for (size_t k = 0; k < 6; ++k) {
            out.restAvars.push_back(
                _RegisteredValue(_Family::Ladder, i, 3 + k));
        }
        for (size_t k = 0; k < 6; ++k) {
            out.defaultAvars.push_back(
                _RegisteredValue(_Family::Ladder, i, 9 + k));
        }
        out.rotationOrder = _Registered(_Family::Ladder, i, 15);
        out.parentSpace = _Registered(_Family::Ladder, i, 16);
        out.parentDefaultSpace = _Registered(_Family::Ladder, i, 17);
        out.avarDefaultSpace = _Registered(_Family::Ladder, i, 18);
        out.posedDefaultSpace = _Registered(_Family::Ladder, i, 19);
        out.rotationSign = _Registered(_Family::Ladder, i, 20);
        out.interveningSpace = _Registered(_Family::Ladder, i, 21);
        const auto &ladder = program.ladders[i];
        out.interveningReset = ladder.interveningReset;
        out.spaceValues.assign(ladder.spaceValues.begin(),ladder.spaceValues.end());
        out.posedSpaceConnected = ladder.posedSpaceConnected;
        out.defaultSpaceConnected = ladder.defaultSpaceConnected;
        out.parentSpaceConnected = ladder.parentSpaceConnected;
        out.parentDefaultSpaceConnected = ladder.parentDefaultSpaceConnected;
        out.avarDefaultSpaceConnected = ladder.avarDefaultSpaceConnected;
        out.posedDefaultSpaceConnected = ladder.posedDefaultSpaceConnected;
    }
    pose->ladderVarying = program.ladderVarying;
    pose->ladderOverrides = _ToI32s(program.ladderOverrides);
    pose->restChainVaries.reserve(program.restChainVaries.size());
    for (char v : program.restChainVaries) {
        pose->restChainVaries.push_back(v ? uint8_t(1) : uint8_t(0));
    }
    pose->poseInterpolators.resize(program.poseInterpolators.size());
    for (size_t i = 0; i < program.poseInterpolators.size(); ++i) {
        const RigExecBakedProgramImpl::PoseInterpolator &interp =
            program.poseInterpolators[i];
        fb::RigExecWirePoseInterpolator &out = pose->poseInterpolators[i];
        out.path = _Path(interp.path);
        out.driverSlot = int32_t(interp.driverSlot);
        out.parentSlot = int32_t(interp.parentSlot);
        out.allowNegativeWeights = interp.allowNegativeWeights;
        out.enabled = _Registered(_Family::Interpolator, i, 0);
        out.weightBegin = int32_t(interp.weightBegin);
        out.weightEnd = int32_t(interp.weightEnd);
        out.poseSlots = _ToI32s(interp.poseSlots);
        out.disabledSlots = _ToI32s(interp.disabledSlots);
        const RigExecRbfSolver &solver = interp.solver;
        auto rbf = std::make_unique<fb::RigExecWireRbf>();
        rbf->poses.reserve(solver.GetPoses().size());
        for (const GfVec3d &p : solver.GetPoses()) {
            rbf->poses.push_back(_ToVec3d(p));
        }
        rbf->translations.reserve(solver.GetTranslations().size());
        for (const GfVec3d &t : solver.GetTranslations()) {
            rbf->translations.push_back(_ToVec3d(t));
        }
        rbf->poseTypes.reserve(solver.GetPoseTypes().size());
        for (RigExecRbfPoseType type : solver.GetPoseTypes()) {
            rbf->poseTypes.push_back(uint8_t(type));
        }
        rbf->twistAxis = _ToVec3d(solver.GetTwistAxis());
        rbf->kernel = uint8_t(solver.GetKernel());
        rbf->radius = solver.GetRadius();
        rbf->translationRadius = solver.GetTranslationRadius();
        rbf->radii = solver.GetRadii();
        rbf->translationRadii = solver.GetTranslationRadii();
        rbf->regularization = solver.GetRegularization();
        rbf->normalize = solver.GetNormalize();
        rbf->enableRotation = solver.GetEnableRotation();
        rbf->enableTranslation = solver.GetEnableTranslation();
        rbf->regularizedSingular = solver.GetRegularizedSingular();
        rbf->weights.resize(solver.GetWeights().size());
        for (size_t k = 0; k < solver.GetWeights().size(); ++k) {
            rbf->weights[k].v = solver.GetWeights()[k];
        }
        out.solver = std::move(rbf);
        out.enableTranslation = interp.enableTranslation;
        for (size_t k = 0; k < interp.valueInputs.size(); ++k) {
            out.valueInputs.push_back(
                _RegisteredValue(_Family::Interpolator, i, 1 + k));
        }
    }
    pose->poseWeightPaths.reserve(program.poseWeightPaths.size());
    for (const SdfPath &path : program.poseWeightPaths) {
        pose->poseWeightPaths.push_back(_Path(path));
    }
    pose->solvers.resize(program.solvers.size());
    for (size_t i = 0; i < program.solvers.size(); ++i) {
        _Solver(i, &pose->solvers[i]);
    }
    pose->guideSolvers = _ToI32s(program.guideSolvers);
    pose->constraints.resize(program.constraints.size());
    for (size_t i = 0; i < program.constraints.size(); ++i) {
        _Constraint(i, &pose->constraints[i]);
    }
    // The shape; the static capture adds the values the run read.
    pose->constraintArrays.resize(program.constraintArrays.size());
    for (size_t i = 0; i < program.constraintArrays.size(); ++i) {
        const RigExecBakedProgramImpl::ConstraintArrays &arrays =
            program.constraintArrays[i];
        fb::RigExecWireConstraintArrays &out = pose->constraintArrays[i];
        out.prim = arrays.prim ? _Path(arrays.prim.GetPath()) : 0;
        out.rawSlots.assign(_inputs.constraintRawSlots[i].begin(),_inputs.constraintRawSlots[i].end());
        out.sourceCount = uint64_t(arrays.sourceCount);
        out.parentOffsets = arrays.parentOffsets;
        out.readPole = arrays.readPole;
        out.poleCount = uint64_t(arrays.poleCount);
    }
    pose->nativeSources.resize(program.nativeSources.size());
    for (size_t i = 0; i < program.nativeSources.size(); ++i) {
        const RigExecBakedProgramImpl::NativeXformSource &source =
            program.nativeSources[i];
        pose->nativeSources[i].path = _Path(source.path);
        pose->nativeSources[i].ancestorSlots = _ToI32s(source.ancestorSlots);
    }
    pose->walkSteps.resize(program.walkSteps.size());
    for (size_t i = 0; i < program.walkSteps.size(); ++i) {
        const RigExecBakedProgramImpl::WalkStep &step = program.walkSteps[i];
        fb::RigExecWireWalkStep &out = pose->walkSteps[i];
        out.solverBatch = step.solverBatch;
        out.level = uint64_t(step.level);
        out.index = int32_t(step.index);
        out.batchSolvers = _ToI32s(step.batchSolvers);
        out.propagate.reserve(step.propagate.size());
        for (const auto &pair : step.propagate) {
            out.propagate.emplace_back(int32_t(pair.first),
                                       int32_t(pair.second));
        }
    }
    pose->composeGroups.resize(program.composeGroups.size());
    for (size_t i = 0; i < program.composeGroups.size(); ++i) {
        const RigExecBakedComposeGroup &group = program.composeGroups[i];
        fb::RigExecWireComposeGroup &out = pose->composeGroups[i];
        out.begin = int32_t(group.begin);
        out.end = int32_t(group.end);
        out.parentSlots = _ToI32s(group.parentSlots);
    }
    pose->commits.resize(program.commits.size());
    for (size_t i = 0; i < program.commits.size(); ++i) {
        const RigExecBakedCommit &commit = program.commits[i];
        fb::RigExecWireCommit &out = pose->commits[i];
        out.moverPath = _Path(commit.moverPath);
        out.solverOutput = commit.solverOutput;
        out.slots = _ToI32s(commit.slots);
        out.propagate.reserve(commit.propagate.size());
        for (const auto &pair : commit.propagate) {
            out.propagate.emplace_back(int32_t(pair.first),
                                       int32_t(pair.second));
        }
        out.closestPos = _ToI32s(commit.closestPos);
        out.split = commit.split;
        out.stagingBase = int32_t(commit.stagingBase);
        out.sources.reserve(commit.sources.size());
        for (const RigExecConstraintSource &source : commit.sources) {
            RigExecWireConstraintSource wire;
            wire.frame = _ToFrame(source.frame);
            wire.normalizedWeight = source.normalizedWeight;
            wire.translationOffset = _ToVec3d(source.translationOffset);
            wire.rotationOffsetDegrees =
                _ToVec3d(source.rotationOffsetDegrees);
            out.sources.push_back(wire);
        }
        out.slotReads = commit.slotReads;
        out.slotWrites = commit.slotWrites;
        out.slotBaseWrites = commit.slotBaseWrites;
        out.descendantReads = commit.descendantReads;
        out.closestReads = commit.closestReads;
        out.descendantWrites = commit.descendantWrites;
        out.descendantBaseWrites = commit.descendantBaseWrites;
        out.slotCarry = commit.slotCarry;
        out.slotBaseCarry = commit.slotBaseCarry;
        out.descendantCarry = commit.descendantCarry;
        out.descendantBaseCarry = commit.descendantBaseCarry;
        out.sourceReads = commit.sourceReads;
        out.worldUpRead = commit.worldUpRead;
        out.targetRead = commit.targetRead;
        out.targetReads = commit.targetReads;
        out.effectorRead = commit.effectorRead;
        out.effectorAncestors.reserve(commit.effectorAncestors.size());
        for (const auto &read : commit.effectorAncestors) {
            out.effectorAncestors.push_back(_ToAncestorRead(read));
        }
        out.poleReads = commit.poleReads;
        out.poleAncestors = _FbAncestorLists(commit.poleAncestors);
        out.recordAfter = commit.recordAfter;
        out.recordEveryTarget = commit.recordEveryTarget;
        out.sourceAncestors = _FbAncestorLists(commit.sourceAncestors);
        out.worldUpAncestors.reserve(commit.worldUpAncestors.size());
        for (const auto &read : commit.worldUpAncestors) {
            out.worldUpAncestors.push_back(_ToAncestorRead(read));
        }
    }
    if (program.jointSolverBinding) {
        for (const auto &entry : *program.jointSolverBinding) {
            pose->jointBindingJoints.push_back(_Path(entry.first));
            fb::RigExecWireUintList solvers;
            fb::RigExecWireIntList elements;
            solvers.v.reserve(entry.second.size());
            elements.v.reserve(entry.second.size());
            for (const auto &stack : entry.second) {
                solvers.v.push_back(_Path(stack.first));
                elements.v.push_back(int32_t(stack.second));
            }
            pose->jointBindingSolvers.push_back(std::move(solvers));
            pose->jointBindingElements.push_back(std::move(elements));
        }
    }
    pose->hasPropertyChains = program.hasPropertyChains;
    pose->publishWeightFields = program.publishWeightFields;
    // In program order: resolution round, then discovery.
    pose->autoClavicles.resize(program.autoClavicles.size());
    for(size_t i=0;i<program.autoClavicles.size();++i) {
        const auto &ac=program.autoClavicles[i];auto &out=pose->autoClavicles[i];
        const auto &c=ac.operation.constants;
        out.slot=ac.slot;out.basis.assign(c.basis,c.basis+9);out.ikValue=c.ikValue;out.gain=c.gain;
        out.kernel=c.kernel;out.normalize=c.normalize;out.swings=c.swings;out.widths=c.widths;
        out.gains=c.gains;out.weights=c.weights;out.hasLimb=ac.operation.hasLimb;
        out.frames.resize(13);out.frames[0].slot=ac.slot;
        for(size_t k=1;k<13;++k)for(const auto &read:ac.frames)if(read.value==ac.operation.frames[k]) {
            out.frames[k].slot=read.slot;
            out.frames[k].computation=read.computation=="computeDefaultFrame"?1:read.computation=="computeRestFrame"?2:0;
            out.frames[k].recompose.assign(read.recompose.begin(),read.recompose.end());
        }
        for(size_t k=0;k<ac.scalars.size();++k)out.scalars.push_back(_RegisteredValue(_Family::AutoClavicle,i,k));
        out.scalarIndices.assign(11,-1);
        for(size_t k=0;k<11;++k)if(!ac.operation.scalars[k].hops.empty())
            for(size_t j=0;j<ac.scalars.size();++j)if(ac.scalars[j].value==ac.operation.scalars[k].hops.front().raw)
                out.scalarIndices[k]=int(j);
    }
    pose->spaceSwitches.resize(program.spaceSwitches.size());
    for (size_t i = 0; i < program.spaceSwitches.size(); ++i) {
        const RigExecBakedProgramImpl::SpaceSwitch &sw =
            program.spaceSwitches[i];
        fb::RigExecWireSpaceSwitch &out = pose->spaceSwitches[i];
        out.slot = int32_t(sw.slot);
        out.sourceSlots = _ToI32s(sw.sourceSlots);
        out.filters.reserve(sw.filters.size());
        for (const RigExecRotationFilter filter : sw.filters) {
            out.filters.push_back(uint8_t(filter));
        }
        out.twistAxis = _ToVec3d(sw.twistAxis);
        out.spaceSlot = int32_t(sw.spaceSlot);
        using Version = RigExecBakedProgramImpl::SpaceSwitch::FrameVersion;
        const auto version = [](const Version &read) {
            fb::RigExecWireFrameVersion out;
            out.anchor = int32_t(read.anchor);
            out.recompose = _ToI32s(read.recompose);
            out.context = read.context;
            return out;
        };
        out.parentRead = std::make_unique<fb::RigExecWireFrameVersion>(
            version(sw.parentRead));
        out.sourceReads.reserve(sw.sourceReads.size());
        for (const auto &read : sw.sourceReads) {
            out.sourceReads.push_back(version(read));
        }
        out.spaceRead = std::make_unique<fb::RigExecWireFrameVersion>(
            version(sw.spaceRead));
        out.active = _Registered(_Family::SpaceSwitch, i, 0);
        out.tokenIndex=sw.tokenIndex;
        for(const auto &label:sw.labels) out.labels.push_back(label.GetString());
        for (size_t axis = 0; axis < 3; ++axis) {
            out.affectTranslation[axis] = sw.affectTranslation[axis];
            out.affectRotation[axis] = sw.affectRotation[axis];
            out.affectScale[axis] = sw.affectScale[axis];
        }
    }
    // The varying bindings, then the constant ones; the reader tells them
    // apart by the read's Varying flag.
    pose->avarBindings.reserve(program.avarBindings.size() +
                               program.avarConstantBindings.size());
    for (size_t i = 0; i < program.avarBindings.size(); ++i) {
        fb::RigExecWireAvarBinding out;
        out.flat = uint32_t(program.avarBindings[i].slot);
        out.read = _Registered(_Family::AvarBinding, i, 0);
        pose->avarBindings.push_back(std::move(out));
    }
    for (size_t i = 0; i < program.avarConstantBindings.size(); ++i) {
        fb::RigExecWireAvarBinding out;
        out.flat = uint32_t(program.avarConstantBindings[i].slot);
        out.read = _Registered(_Family::AvarConstantBinding, i, 0);
        pose->avarBindings.push_back(std::move(out));
    }
    pose->overrideCount = uint32_t(program.overridden.size());
    // In the program's walk order, which the FrameMatrix steps index.
    pose->spaceCheckpoints.reserve(program.switchFrameContexts.size());
    for (const auto &checkpoint : program.switchFrameContexts) {
        fb::RigExecWireSpaceCheckpoint out;
        out.key = checkpoint.key;
        out.anchor = checkpoint.anchor;
        out.recompose = _ToI32s(checkpoint.recompose);
        pose->spaceCheckpoints.push_back(std::move(out));
    }
    pose->frameRecords.reserve(program.frameRecords.size());
    for (const RigExecBakedFrameRecord &record : program.frameRecords) {
        const uint32_t mover = _Path(record.mover);
        pose->frameRecords.emplace_back(
            uint32_t(record.slot), uint32_t(record.commit),
            int32_t(record.target), int32_t(record.position), record.version,
            mover);
    }
    pose->providerFrameInputs.reserve(program.providerFrameInputs.size());
    for (const auto &input : program.providerFrameInputs) {
        fb::RigExecWireProviderFrameInput out;
        out.value = input.value;
        out.slot = input.slot;
        out.base = input.base;
        out.version = input.version;
        out.reader = _Path(input.reader);
        pose->providerFrameInputs.push_back(std::move(out));
    }
    pose->providerRefreshes.reserve(program.providerRefreshes.size());
    for (const auto &refresh : program.providerRefreshes) {
        fb::RigExecWireProviderRefresh out;
        out.key = refresh.key;
        out.reader = _Path(refresh.reader);
        out.slot = refresh.slot;
        out.checkpoint = refresh.checkpoint;
        out.baseValue = refresh.baseValue;
        out.currentValue = refresh.currentValue;
        out.baseRead = refresh.baseRead;
        out.finRead = refresh.finRead;
        out.baseWrite = refresh.baseWrite;
        out.finWrite = refresh.finWrite;
        out.carries.reserve(refresh.carries.size());
        for (const auto &carry : refresh.carries) {
            fb::RigExecWireProviderRefreshCarry row;
            row.slot = carry.slot;
            row.baseRead = carry.baseRead;
            row.finRead = carry.finRead;
            row.baseWrite = carry.baseWrite;
            row.finWrite = carry.finWrite;
            row.blockingSlots.assign(carry.blockingSlots.begin(),
                                     carry.blockingSlots.end());
            out.carries.push_back(std::move(row));
        }
        out.priorConstraints.assign(refresh.priorConstraints.begin(),
                                    refresh.priorConstraints.end());
        pose->providerRefreshes.push_back(std::move(out));
    }
}

std::unique_ptr<fb::RigExecWireSkinTopology>
_FileFill::_Topology(const RigExecSkinTopology &topology,
                     const RigExecBakedProgramImpl::GeomRevision &revision)
{
    auto out = std::make_unique<fb::RigExecWireSkinTopology>();
    std::string why;
    // A layout the sparse form cannot hold is stored raw, so playback reads
    // the arrays the evaluators read.
    if (!RigExecFormatTopology(
            topology.indices, topology.weights, int32_t(topology.elementSize),
            uint64_t(topology.pointCount), uint64_t(topology.influenceCount),
            topology.validated, out.get(), &why)) {
        _Fail("skin mover " + revision.moverPath.GetString() +
              ": its joint layout cannot be written (" + why + ")");
        return nullptr;
    }
    return out;
}

void
_FileFill::_Revision(const RigExecBakedProgramImpl::GeomRevision &revision,
                     size_t chain, size_t index, bool derived,
                     fb::RigExecWireRevision *out)
{
    const auto fillSites = [&](bool layout, auto *destination) {
        const auto found = _inputs.leafSites.find({uint32_t(chain), uint32_t(index), derived, layout});
        if (found == _inputs.leafSites.end()) return;
        *destination = found->second.reads;
        for (size_t k = 0; k < destination->size(); ++k) {
            auto &row = (*destination)[k];
            if (!RigExecFormatIsArrayTag(row.read->tag)) continue;
            const auto &fallback = found->second.fallbacks[k];
            fb::RigExecWireValue value; value.tag = row.read->tag;
            if (fallback.IsHolding<VtIntArray>()) {
                const auto &v=fallback.UncheckedGet<VtIntArray>(); value.array=_pools->Ints(v.cdata(),v.size());
            } else if (fallback.IsHolding<VtFloatArray>()) {
                const auto &v=fallback.UncheckedGet<VtFloatArray>(); value.array=_pools->Floats(v.cdata(),v.size());
            } else if (fallback.IsHolding<VtDoubleArray>()) {
                const auto &v=fallback.UncheckedGet<VtDoubleArray>(); value.array=_pools->Doubles(v.cdata(),v.size());
            } else if (fallback.IsHolding<VtVec2fArray>()) {
                const auto &v=fallback.UncheckedGet<VtVec2fArray>(); value.array=_pools->Vec2fs(v.empty()?nullptr:v.cdata()->data(),v.size());
            } else if (fallback.IsHolding<VtVec3fArray>()) {
                const auto &v=fallback.UncheckedGet<VtVec3fArray>(); value.array=_pools->Vec3fs(v.empty()?nullptr:v.cdata()->data(),v.size());
            }
            row.read->constant=_pools->Value(value);
            if (row.bodyWalk) row.bodyWalk->constant=row.read->constant;
        }
    };
    fillSites(false, &out->leafSites);
    fillSites(true, &out->layoutLeafSites);
    out->moverPath = _Path(revision.moverPath);
    out->target = _Path(revision.target);
    out->moverPrim =
        revision.moverPrim ? _Path(revision.moverPrim.GetPath()) : 0;
    out->op = uint8_t(revision.op);
    const RigExecRevisionBinding &binding = revision.binding;
    auto bound = std::make_unique<fb::RigExecWireRevisionBinding>();
    bound->moverPath = _Path(binding.moverPath);
    bound->target = _Path(binding.target);
    bound->transform = _Path(binding.transform);
    bound->transformSpace = _Path(binding.transformSpace);
    bound->influences.reserve(binding.influences.size());
    for (const SdfPath &path : binding.influences) {
        bound->influences.push_back(_Path(path));
    }
    bound->weightObject = _Path(binding.weightObject);
    bound->base = _Path(binding.base);
    bound->topologyCounts = _Path(binding.topologyCounts);
    bound->topologyIndices = _Path(binding.topologyIndices);
    bound->cagePoints = _Path(binding.cagePoints);
    bound->surfacePoints = _Path(binding.surfacePoints);
    bound->bindCoords = _Path(binding.bindCoords);
    bound->driverCurvePoints = _Path(binding.driverCurvePoints);
    bound->driverCurveOrder = _Path(binding.driverCurveOrder);
    bound->driverCurveKnots = _Path(binding.driverCurveKnots);
    bound->driverTransformCount = int32_t(binding.driverTransformCount);
    bound->driverSpaceCount = int32_t(binding.driverSpaceCount);
    bound->driverBaseTransformCount =
        int32_t(binding.driverBaseTransformCount);
    bound->driverFrames = _Path(binding.driverFrames);
    bound->widths = _Path(binding.widths);
    bound->blendInputs.reserve(binding.blendInputs.size());
    for (const SdfPath &path : binding.blendInputs) {
        bound->blendInputs.push_back(_Path(path));
    }
    bound->blendSampleInputs.reserve(binding.blendSamples.size());
    bound->blendSamples.reserve(binding.blendSamples.size());
    for (const auto &entry : binding.blendSamples) {
        bound->blendSampleInputs.push_back(_Path(entry.first));
        fb::RigExecWireBlendSampleBindingList row;
        row.v.reserve(entry.second.size());
        for (const RigExecBlendSampleBinding &sample : entry.second) {
            fb::RigExecWireBlendSampleBinding wire;
            wire.sample = _Path(sample.sample);
            wire.points = _Path(sample.points);
            wire.phase = _Phase(sample.phase);
            wire.blendShape = _Path(sample.blendShape);
            row.v.push_back(std::move(wire));
        }
        bound->blendSamples.push_back(std::move(row));
    }
    bound->phaseInputs.reserve(binding.phases.size());
    bound->phases.reserve(binding.phases.size());
    for (const auto &entry : binding.phases) {
        bound->phaseInputs.push_back(_Path(entry.first));
        bound->phases.push_back(_Phase(entry.second));
    }
    bound->transformPhase = _Phase(binding.transformPhase);
    out->binding = std::move(bound);
    out->blendChannels.resize(revision.blendChannels.size());
    for (size_t ch = 0; ch < revision.blendChannels.size(); ++ch) {
        const RigExecBakedProgramImpl::GeomBlendChannel &channel =
            revision.blendChannels[ch];
        fb::RigExecWireBlendChannel &wire = out->blendChannels[ch];
        wire.weight = channel.weight ? _Path(channel.weight.GetPath()) : 0;
        wire.weightValid = bool(channel.weight);
        wire.weightPath = _Path(channel.weightPath);
        wire.poseWeight = int32_t(channel.poseWeight);
        const auto reads = _blendReads.find(std::make_tuple(
            uint32_t(chain), uint32_t(index), derived, uint32_t(ch)));
        if (reads == _blendReads.end() ||
            reads->second->activations.size() != channel.samples.size()) {
            _Fail("the inputs hold no blend weight read for channel " +
                  channel.weightPath.GetString());
        } else {
            wire.weightRead = _FbInputPtr(reads->second->read);
        }
        wire.samples.resize(channel.samples.size());
        for (size_t s = 0; s < channel.samples.size(); ++s) {
            const RigExecBakedProgramImpl::GeomBlendChannel::Sample &sample =
                channel.samples[s];
            fb::RigExecWireBlendSample &wireSample = wire.samples[s];
            wireSample.samplePath = _Path(sample.samplePath);
            wireSample.activation =
                sample.activation ? _Path(sample.activation.GetPath()) : 0;
            wireSample.activationValid = bool(sample.activation);
            wireSample.points =
                sample.points ? _Path(sample.points.GetPath()) : 0;
            wireSample.pointsValid = bool(sample.points);
            wireSample.pointsPath = _Path(sample.pointsPath);
            wireSample.phase = _Phase(sample.phase);
            wireSample.blendShape = _Path(sample.blendShape);
            wireSample.shapeValid = sample.shapeValid;
            wireSample.hasLayout = bool(sample.layout);
            if (sample.layout) {
                wireSample.offsets.reserve(sample.layout->offsets.size());
                for (const GfVec3f &p : sample.layout->offsets) {
                    wireSample.offsets.push_back(_ToVec3f(p));
                }
                wireSample.indices.assign(sample.layout->indices.begin(),
                                          sample.layout->indices.end());
                wireSample.pointCount = uint64_t(sample.layout->pointCount);
                wireSample.layoutValid = sample.layout->valid;
            }
            if (reads != _blendReads.end() &&
                s < reads->second->activations.size()) {
                wireSample.activationRead =
                    _FbInputPtr(reads->second->activations[s]);
            }
            // Only a dense sample with a non-base phase is bound.
            if (sample.pointBinding.id >= 0) {
                wireSample.pointBinding =
                    std::make_unique<fb::RigExecWirePointsBinding>(
                        _PointsBinding(sample.pointBinding));
            }
            const auto points = _blendPoints.find(
                std::make_tuple(uint32_t(chain), uint32_t(index), derived,
                                uint32_t(ch), uint32_t(s)));
            if (points != _blendPoints.end()) {
                wireSample.pointsRead = _FbInputPtr(*points->second);
            }
        }
    }
    out->influenceSlots = _ToI32s(revision.influenceSlots);
    out->transformSlot = int32_t(revision.transformSlot);
    out->transformSpaceSlot = int32_t(revision.transformSpaceSlot);
    out->carrySpaceSlot = int32_t(revision.carrySpaceSlot);
    out->shaderDials.reserve(binding.shaderDials.size());
    for (const SdfPath &dial : binding.shaderDials) {
        out->shaderDials.push_back(_Path(dial));
    }
    out->meshWorldInverse = _ToMatrix(binding.meshWorldInverse);
    out->constraintDelta = int32_t(revision.constraintDelta);
    out->driverFramesSolver = int32_t(revision.driverFramesSolver);
    out->finalPhase = revision.finalPhase;
    out->skinTopologyFixed = revision.skinTopologyFixed;
    if (!derived) {
        const auto layout = _layoutSlots.find(
            std::make_pair(uint32_t(chain), uint32_t(index)));
        if (layout != _layoutSlots.end()) {
            out->jointIndicesSlot = int32_t(layout->second->indices);
            out->jointWeightsSlot = int32_t(layout->second->weights);
        }
    }
    out->packetInfluences.reserve(revision.packetInfluences.size());
    for (const GfMatrix4d &m : revision.packetInfluences) {
        out->packetInfluences.push_back(_ToMatrix(m));
    }
    out->chunks.resize(revision.chunks.size());
    for (size_t k = 0; k < revision.chunks.size(); ++k) {
        const RigExecBakedProgramImpl::GeomChunk &chunk = revision.chunks[k];
        out->chunks[k].begin = int32_t(chunk.begin);
        out->chunks[k].end = int32_t(chunk.end);
        out->chunks[k].key.assign(chunk.key.begin(), chunk.key.end());
    }
    out->chunkBase = int32_t(revision.chunkBase);
    // The partition is stored once when it is the topology itself (the
    // same shared layout, not an equal one).
    out->partitionSameAsTopology =
        revision.partitionTopology &&
        revision.partitionTopology == revision.topology;
    if (revision.partitionTopology && !out->partitionSameAsTopology) {
        out->partitionTopology =
            _Topology(*revision.partitionTopology, revision);
    }
    out->partitionElementSize = int32_t(revision.partitionElementSize);
    out->partitionIndexCount = uint64_t(revision.partitionIndexCount);
    out->partitionPointCount = uint64_t(revision.partitionPointCount);
    // A Range skin is unchunked in the file, its chunks the chain's groups
    // with their influence keys (RigExecFormatIsKeyedRevision).
    out->chunked = revision.chunked &&
                   revision.role != RigExecBakedRevisionRole::Range;
    out->partitionCandidates = uint64_t(revision.partitionCandidates);
    out->partitionProducerMin = int32_t(revision.partitionProducerMin);
    out->partitionProducerMax = int32_t(revision.partitionProducerMax);
    out->partitionDistinctReads=uint64_t(revision.partitionDistinctReads);
    for(const auto &set:revision.partitionProducerSets) {
        fb::RigExecWirePartitionProducerSet row;
        for(const auto &value:set) row.values.emplace_back(
            fb::SlotDomain(value.first),uint32_t(value.second),uint32_t(value.second)+1);
        out->partitionProducerSets.push_back(std::move(row));
    }
    out->weightObject = int32_t(revision.weightObject);
    out->weightOperationDomain = revision.weightOperationDomain;
    out->weightFieldTarget = _Path(revision.weightFieldTarget);
    out->weightCurrentPhase = revision.weightCurrentPhase;
    out->weightField = revision.weightField;
    if (revision.topology) {
        out->topology = _Topology(*revision.topology, revision);
    }
    out->topologyResolved = revision.topologyResolved;
    // The phase tables, in the program's order: frame records the fold
    // reads (an AtPrim transform phase only), and one point binding per
    // binding.phases entry.
    out->transformRecords.reserve(revision.transformRecords.size());
    for (const int record : revision.transformRecords) {
        out->transformRecords.push_back(uint32_t(record));
    }
    out->influenceRecords.resize(revision.influenceRecords.size());
    for (size_t k = 0; k < revision.influenceRecords.size(); ++k) {
        std::vector<uint32_t> &records = out->influenceRecords[k].v;
        records.reserve(revision.influenceRecords[k].size());
        for (const int record : revision.influenceRecords[k]) {
            records.push_back(uint32_t(record));
        }
    }
    out->pointBindings.reserve(revision.pointBindings.size());
    for (const RigExecBakedPointsBinding &binding : revision.pointBindings) {
        out->pointBindings.push_back(_PointsBinding(binding));
    }
    // RevisionStatic reads a main revision's inputs:defaultWeight; a
    // derived one weighs 1 and holds none.
    if (!derived) {
        const auto read = _defaultWeightReads.find(
            std::make_pair(uint32_t(chain), uint32_t(index)));
        if (read == _defaultWeightReads.end()) {
            _Fail("the inputs hold no default weight read for " +
                  revision.moverPath.GetString());
        } else {
            out->defaultWeight = _FbInputPtr(*read->second);
        }
    }
}

void
_FileFill::_Geometry(fb::RigExecWireDomainGeometry *geometry)
{
    const RigExecBakedProgramImpl &program = _program;
    geometry->chains.resize(program.chains.size());
    for (size_t c = 0; c < program.chains.size(); ++c) {
        const RigExecBakedProgramImpl::GeomChain &chain = program.chains[c];
        fb::RigExecWireChain &wire = geometry->chains[c];
        wire.target = _Path(chain.target);
        wire.baseSlot = _inputs.chainBaseSlots[c];
        wire.revisions.resize(chain.revisions.size());
        for (size_t r = 0; r < chain.revisions.size(); ++r) {
            _Revision(chain.revisions[r], c, r, false, &wire.revisions[r]);
        }
        wire.derived.resize(chain.derived.size());
        for (size_t d = 0; d < chain.derived.size(); ++d) {
            wire.derived[d].target = _Path(chain.derived[d].target);
            for(size_t k=0;k<program.derivedIndex.size();++k)
                if(program.derivedIndex[k].first==int(c) && program.derivedIndex[k].second==int(d))
                    wire.derived[d].baseSlot=_inputs.derivedBaseSlots[k];
            wire.derived[d].revision =
                std::make_unique<fb::RigExecWireRevision>();
            _Revision(chain.derived[d].revision, c, d, true,
                      wire.derived[d].revision.get());
        }
    }
    geometry->revisionIndex.reserve(program.revisionIndex.size());
    for (const auto &entry : program.revisionIndex) {
        geometry->revisionIndex.emplace_back(int32_t(entry.first),
                                             int32_t(entry.second));
    }
    geometry->derivedIndex.reserve(program.derivedIndex.size());
    for (const auto &entry : program.derivedIndex) {
        geometry->derivedIndex.emplace_back(int32_t(entry.first),
                                            int32_t(entry.second));
    }
    geometry->chainRevisionBegin = _ToI32s(program.chainRevisionBegin);
    geometry->chainRevisionEnd = _ToI32s(program.chainRevisionEnd);
    geometry->revisionChunkBase = _ToI32s(program.revisionChunkBase);
    geometry->revisionChunkCount = _ToI32s(program.revisionChunkCount);
    geometry->chainChunkBegin = _ToI32s(program.chainChunkBegin);
    geometry->chainChunkEnd = _ToI32s(program.chainChunkEnd);
    // The step-backed objects in the program's order, then the envelope-
    // only ones, each with its reads and oracle facts as collected.
    geometry->weightObjects = _inputs.weightObjects;
    geometry->weightFields = _inputs.weightFields;
    for (size_t f=0;f<geometry->weightFields.size();++f) {
        for (const auto &read:_program.weightFields[f].pointReads) {
            fb::RigExecWireWeightFieldPointRead row;
            row.object=read.object; row.leaf=read.leaf;
            row.binding=std::make_unique<fb::RigExecWirePointsBinding>(_PointsBinding(read.binding));
            geometry->weightFields[f].pointReads.push_back(std::move(row));
        }
    }
    geometry->falloffPaths.reserve(program.falloffLuts.size());
    geometry->falloffLuts.reserve(program.falloffLuts.size());
    for (const auto &entry : program.falloffLuts) {
        geometry->falloffPaths.push_back(_Path(entry.first));
        fb::RigExecWireFloatList lut;
        lut.v = entry.second;
        geometry->falloffLuts.push_back(std::move(lut));
    }
    geometry->currentPhaseWeights.reserve(program.currentPhaseWeights.size());
    for (const SdfPath &path : program.currentPhaseWeights) {
        geometry->currentPhaseWeights.push_back(_Path(path));
    }
    geometry->deltaBasePaths.reserve(program.deltaBasePaths.size());
    for (const SdfPath &path : program.deltaBasePaths) {
        geometry->deltaBasePaths.push_back(_Path(path));
    }
}

void
_FileFill::_Inputs(fb::RigExecWireFile *file)
{
    file->inputs = _inputs.inputs;
    file->listedInputs = _inputs.listedInputs;
    file->propertyChains = _inputs.propertyChains;
    file->phasedConsumers = _inputs.phasedConsumers;
}

bool
_FileFill::Run(fb::RigExecWireFile *file, std::string *error)
{
    if (!_Index()) {
        *error = _error;
        return false;
    }
    file->slotMeta = std::make_unique<fb::RigExecWireSlotMeta>();
    _SlotMeta(file->slotMeta.get());
    file->constants = std::make_unique<fb::RigExecWireConstants>();
    _Constants(file->constants.get());
    _Steps(&file->steps);
    file->commonGraph = std::make_unique<fb::RigExecWireCommonGraph>();
    auto &common = *file->commonGraph;
    for (const auto &value : _program.opAdapter.values) {
        fb::RigExecWireCommonValueSpec spec;
        spec.domain = value.domain;
        spec.slot = value.slot;
        common.valueSpecs.push_back(spec);
    }
    common.leaves = _program.opAdapter.leaves;
    common.excludedValues = _program.opAdapter.excludedValues;
    common.canonicalIndex = _program.opGraph.canonicalIndex;
    common.longestPath = _program.opGraph.longestPath;
    common.opClusters = _program.opGraph.opClusters;
    for (const auto &cluster : _program.opGraph.clusters) {
        fb::RigExecWireCommonCluster row;
        row.members = cluster.members;
        row.predecessors = cluster.predecessors;
        row.successors = cluster.successors;
        common.clusters.push_back(std::move(row));
    }
    for (const auto &op : _program.opGraph.ops) {
        fb::RigExecWireCommonOp row;
        row.key = op.descriptor.key;
        row.kind = op.descriptor.kind;
        row.originalIndex = op.originalIndex;
        row.reads = op.descriptor.reads;
        row.writes = op.descriptor.writes;
        row.descriptorPredecessors = op.descriptor.predecessors;
        row.predecessors = op.predecessors;
        row.successors = op.successors;
        row.volatileInput = op.descriptor.volatileInput;
        common.ops.push_back(std::move(row));
    }
    std::map<RigExecValueId, std::vector<uint32_t>> readers(
        _program.opGraph.readers.begin(), _program.opGraph.readers.end());
    for (const auto &entry : readers) {
        fb::RigExecWireCommonReaders row;
        row.value = entry.first;
        row.ops = entry.second;
        common.readers.push_back(std::move(row));
    }
    for (const auto &cycle : _program.opGraph.cycles) {
        fb::RigExecWireCommonCycle row;
        row.keys = cycle;
        common.cycles.push_back(std::move(row));
    }
    RigExecProviderPlainProgram provider;
    if (!RigExecExportProviderRecords(_program.providerProgram,
                                     _program.providerValues, &provider, error)) {
        return false;
    }
    file->providerProgram = std::make_unique<fb::RigExecWireProviderProgram>();
    auto &portable = *file->providerProgram;
    portable.valueKeys = provider.valueKeys;
    portable.leaves = provider.leaves;
    for (const auto &state : provider.defaults) {
        fb::RigExecWireProviderValue row;
        row.kind = fb::ProviderValueKind(state.value.index());
        row.initialized = state.initialized;
        row.blocked = state.blocked;
        row.authoritative = state.authoritative;
        row.count = state.count;
        row.error = state.error;
        if (const auto *v = std::get_if<double>(&state.value)) row.scalarDouble = std::make_unique<double>(*v);
        else if (const auto *v = std::get_if<float>(&state.value)) row.scalarFloat = std::make_unique<float>(*v);
        else if (const auto *v = std::get_if<std::array<double, 3>>(&state.value))
            row.vector = std::make_unique<RigExecWireVec3d>(*v);
        else if (const auto *v = std::get_if<std::array<double, 16>>(&state.value))
            row.matrix = std::make_unique<RigExecWireMatrix4d>(*v);
        else if (const auto *v = std::get_if<std::string>(&state.value)) row.token = *v;
        else if (const auto *v = std::get_if<RigExecProviderPlainFrame>(&state.value)) {
            row.framePoints.assign(v->points.begin(), v->points.end());
            row.frameFlags = v->flags;
        }
        if(const auto *v=std::get_if<std::array<float,3>>(&state.value))
            row.vec3f=std::make_unique<RigExecWireVec3f>(*v);
        else if(const auto *v=std::get_if<bool>(&state.value)) row.boolean=*v;
        else if(const auto *v=std::get_if<int32_t>(&state.value)) row.integer=*v;
        else if(const auto *v=std::get_if<std::vector<float>>(&state.value)) row.floats=*v;
        else if(const auto *v=std::get_if<std::vector<double>>(&state.value)) row.doubles=*v;
        else if(const auto *v=std::get_if<std::vector<std::array<float,3>>>(&state.value)) row.vec3fs=*v;
        else if(const auto *v=std::get_if<std::vector<std::array<double,3>>>(&state.value)) row.vec3ds=*v;
        else if(const auto *v=std::get_if<std::vector<int32_t>>(&state.value)) row.ints=*v;
        else if(const auto *v=std::get_if<std::vector<std::array<double,16>>>(&state.value)) row.matrices=*v;
        else if(const auto *v=std::get_if<std::vector<std::string>>(&state.value)) row.tokens=*v;
        else if(const auto *v=std::get_if<std::vector<bool>>(&state.value))
            for(bool bit:*v) row.bools.push_back(bit?1:0);
        else if(const auto *v=std::get_if<std::array<float,2>>(&state.value))
            row.vec2f=std::make_unique<RigExecWireVec2f>(*v);
        else if(const auto *v=std::get_if<std::vector<std::array<float,2>>>(&state.value)) row.vec2fs=*v;
        else if(const auto *v=std::get_if<std::array<int32_t,3>>(&state.value))
            row.vec3i=std::make_unique<RigExecWireVec3i>(*v);
        portable.defaults.push_back(std::move(row));
    }
    for (const auto &op : provider.ops) {
        fb::RigExecWireProviderOp row;
        row.kind = uint32_t(op.kind);
        row.owner = op.owner;
        row.output = op.output;
        row.inputs = op.inputs;
        row.scaleAvars = op.scaleAvars;
        row.affineKind = op.affineKind;
        row.affineTargets = op.affineTargets;
        portable.ops.push_back(std::move(row));
    }
    const auto leaves = [](const auto &source, auto *destination) {
        for (const auto &leaf : source) {
            fb::RigExecWireProviderLeaf row;
            row.value = leaf.value;
            row.path = leaf.path;
            row.computation = leaf.computation;
            destination->push_back(std::move(row));
        }
    };
    leaves(provider.sampled, &portable.sampled);
    leaves(provider.externalInputs, &portable.externalInputs);
    for (size_t k = 0; k < portable.sampled.size(); ++k) {
        auto &row = portable.sampled[k];
        row.inputSlot = _inputs.providerLeafSlots[k];

    }
    for (size_t k = 0; k < portable.externalInputs.size(); ++k) {
        auto &row = portable.externalInputs[k];
        row.providerSlot = _program.providerExternalSlots[k];
        if(row.providerSlot>=0 && (row.computation=="computePointFrame" || row.computation=="computeBasePointFrame"))
            row.frameVersion=row.providerSlot;
        if (row.providerSlot >= 0 && row.computation != "computeRestFrame" &&
            row.computation != "computePointFrame" && row.computation != "computeBasePointFrame") {
            row.interveningRead = std::make_unique<fb::RigExecWireInput>(
                _RegisteredValue(_Family::Ladder, uint32_t(row.providerSlot), 21));
        }
    }

    for (size_t k=0;k<provider.routedInputs.size();++k) {
        const auto &route=provider.routedInputs[k];
        fb::RigExecWireProviderRoutedInput row;
        row.value=route.value; row.consumer=route.consumer; row.source=route.source;
        row.readPhase=route.readPhase; row.crossRead=_program.providerRoutedReads[k];
        portable.routedInputs.push_back(std::move(row));
    }
    for (size_t k=0;k<_program.crossDomainReads.size();++k) {
        const auto &read=_program.crossDomainReads[k];
        fb::RigExecWireCrossDomainRead row;
        row.kind=fb::CrossDomainReadKind(read.kind);
        row.consumer=_Path(read.consumer); row.source=_Path(read.source); row.reader=_Path(read.reader);
        row.phase=_Phase(read.phase); row.element=read.element; row.provider=read.provider;
        row.rawSlot=_inputs.crossDomainRawSlots[k]; row.spaceValue=read.spaceValue;
        row.propertyChain=read.propertyChain; row.propertyVersion=read.propertyVersion;
        row.baseFrame=read.baseFrame; row.finalPoints=read.finalPoints;
        for (const auto &point:read.points) row.points.emplace_back(point.chain,point.version);
        row.frames=read.frames; row.unavailable=read.unavailable;
        file->crossDomainReads.push_back(std::move(row));
    }
    file->clustering = std::make_unique<fb::RigExecWireClustering>();
    _Clustering(file->clustering.get());
    file->cones = std::make_unique<fb::RigExecWireCones>();
    _Cones(file->cones.get());
    file->pose = std::make_unique<fb::RigExecWireDomainPose>();
    _Pose(file->pose.get());
    file->geometry = std::make_unique<fb::RigExecWireDomainGeometry>();
    _Geometry(file->geometry.get());
    _Inputs(file);
    if (!_error.empty()) {
        *error = _error;
        return false;
    }
    return true;
}

}  // namespace

bool
RigExecBakeFillFile(const RigExecBakedProgramImpl &program,
                    const RigExecBakeInputs &inputs,
                    RigExecBakePathTable *paths, RigExecBakePools *pools,
                    fb::RigExecWireFile *file, std::string *error)
{
    std::string why;
    if (!paths || !pools || !file) {
        why = "nothing to fill";
    } else if (!pools->Seed(inputs, &why)) {
        why = "cannot pool the collected values: " + why;
    } else if (_FileFill(program, inputs, paths, pools).Run(file, &why)) {
        return true;
    }
    if (error) {
        *error = why;
    }
    return false;
}

}  // namespace rigExec
