//
// .rigexec serialization.
//

#include "rigExecBake/serialize.h"
#include "rigExec/bakedProgramImpl.h"

#include <type_traits>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {
namespace {

// The wire enums mirror the program enums in order; the order is part of the
// format, so every enumerator is pinned here. A reordered program enum fails
// the build, not the bake.
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
#undef _RIGEXEC_BAKE_PIN_DOMAIN

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

RigExecWireClusterSet
_ToClusterSet(const RigExecBakedClusterSet &set, uint32_t clusters)
{
    RigExecWireClusterSet out;
    out.clusters = clusters;
    out.words = set.words;
    return out;
}

std::vector<std::vector<int32_t>>
_ToNested(const std::vector<std::vector<int>> &rows)
{
    std::vector<std::vector<int32_t>> out;
    out.reserve(rows.size());
    for (const std::vector<int> &row : rows) {
        out.push_back(_ToI32s(row));
    }
    return out;
}

}  // namespace

RigExecWireSlotMeta
RigExecBakeConvertSlotMeta(const RigExecBakedProgramImpl &program,
                           RigExecBinaryWriter *writer)
{
    RigExecWireSlotMeta meta;
    meta.paths.reserve(program.paths.size());
    for (const SdfPath &path : program.paths) {
        meta.paths.push_back(writer->AddString(path.GetString()));
    }
    meta.slotKind.reserve(program.slotKind.size());
    for (RigExecBakedSlotKind kind : program.slotKind) {
        meta.slotKind.push_back(RigExecWireSlotKind(uint8_t(kind)));
    }
    meta.parent = _ToI32s(program.parent);
    meta.propParent = _ToI32s(program.propParent);
    meta.xformSlots = _ToI32s(program.xformSlots);
    meta.xformPaths.reserve(program.xformPrimsBySlot.size());
    for (const UsdPrim &prim : program.xformPrimsBySlot) {
        meta.xformPaths.push_back(
            writer->AddString(prim.GetPath().GetString()));
    }
    meta.jointSlots = _ToI32s(program.jointSlots);
    meta.jointPaths.reserve(program.jointPaths.size());
    for (const SdfPath &path : program.jointPaths) {
        meta.jointPaths.push_back(writer->AddString(path.GetString()));
    }
    meta.controlSlots = _ToI32s(program.controlSlots);
    meta.controlPaths.reserve(program.controlPaths.size());
    for (const SdfPath &path : program.controlPaths) {
        meta.controlPaths.push_back(writer->AddString(path.GetString()));
    }
    meta.solverArrayPaths.reserve(program.solverArrays.size());
    meta.solverArrayElements.reserve(program.solverArrays.size());
    for (const auto &entry : program.solverArrays) {
        meta.solverArrayPaths.push_back(
            writer->AddString(entry.first.GetString()));
        meta.solverArrayElements.push_back(int32_t(entry.second));
    }
    meta.jointPublishOrder = _ToI32s(program.jointPublishOrder);
    meta.controlPublishOrder = _ToI32s(program.controlPublishOrder);
    meta.solverPublishOrder = _ToI32s(program.solverPublishOrder);
    meta.jointPathsAscending = program.jointPathsAscending;
    meta.controlPathsAscending = program.controlPathsAscending;
    meta.solverArraysAscending = program.solverArraysAscending;
    meta.needFinal.reserve(program.needFinal.size());
    for (char v : program.needFinal) {
        meta.needFinal.push_back(v ? uint8_t(1) : uint8_t(0));
    }
    meta.needBase.reserve(program.needBase.size());
    for (char v : program.needBase) {
        meta.needBase.push_back(v ? uint8_t(1) : uint8_t(0));
    }
    return meta;
}

RigExecWireConstants
RigExecBakeConvertConstants(const RigExecBakedProgramImpl &program,
                           RigExecBinaryWriter *writer)
{
    RigExecWireConstants constants;
    constants.restM.reserve(program.restM.size());
    for (const GfMatrix4d &m : program.restM) {
        constants.restM.push_back(_ToMatrix(m));
    }
    constants.restPts.reserve(program.restPts.size());
    for (const std::array<GfVec3d, 4> &pts : program.restPts) {
        std::array<RigExecWireVec3d, 4> out;
        for (size_t i = 0; i < 4; ++i) {
            out[i] = _ToVec3d(pts[i]);
        }
        constants.restPts.push_back(out);
    }
    constants.restFrames.reserve(program.restFrames.size());
    for (const RigExecPointFrame &f : program.restFrames) {
        constants.restFrames.push_back(_ToFrame(f));
    }
    constants.selfD.reserve(program.selfD.size());
    for (const GfMatrix4d &m : program.selfD) {
        constants.selfD.push_back(_ToMatrix(m));
    }
    constants.parentDinv.reserve(program.parentDinv.size());
    for (const GfMatrix4d &m : program.parentDinv) {
        constants.parentDinv.push_back(_ToMatrix(m));
    }
    constants.rotOrder.reserve(program.rotOrder.size());
    for (const TfToken &token : program.rotOrder) {
        constants.rotOrder.push_back(writer->AddString(token.GetString()));
    }
    constants.restRoundTrip.reserve(program.restRoundTrip.size());
    for (const GfMatrix4d &m : program.restRoundTrip) {
        constants.restRoundTrip.push_back(_ToMatrix(m));
    }
    constants.defaultRoundTrip.reserve(program.defaultRoundTrip.size());
    for (const GfMatrix4d &m : program.defaultRoundTrip) {
        constants.defaultRoundTrip.push_back(_ToMatrix(m));
    }
    constants.posedAuthored.reserve(program.posedAuthored.size());
    for (char v : program.posedAuthored) {
        constants.posedAuthored.push_back(v ? uint8_t(1) : uint8_t(0));
    }
    constants.posedAuthoredM.reserve(program.posedAuthoredM.size());
    for (const GfMatrix4d &m : program.posedAuthoredM) {
        constants.posedAuthoredM.push_back(_ToMatrix(m));
    }
    constants.noScaleAvars.reserve(program.noScaleAvars.size());
    for (char v : program.noScaleAvars) {
        constants.noScaleAvars.push_back(v ? uint8_t(1) : uint8_t(0));
    }
    constants.avarConstants = program.avarConstants;
    return constants;
}

std::vector<RigExecWireStep>
RigExecBakeConvertSteps(const RigExecBakedProgramImpl &program,
                       RigExecBinaryWriter *writer)
{
    std::vector<RigExecWireStep> steps;
    steps.reserve(program.steps.size());
    for (const RigExecBakedStep &step : program.steps) {
        RigExecWireStep out;
        out.kind = RigExecWireStepKind(uint8_t(step.kind));
        out.object = int32_t(step.object);
        out.part = int32_t(step.part);
        out.reads.reserve(step.reads.size());
        for (const RigExecBakedSlotRange &range : step.reads) {
            RigExecWireSlotRange wire;
            wire.domain = RigExecWireSlotDomain(uint8_t(range.domain));
            wire.begin = range.begin;
            wire.end = range.end;
            out.reads.push_back(wire);
        }
        out.writes.reserve(step.writes.size());
        for (const RigExecBakedSlotRange &range : step.writes) {
            RigExecWireSlotRange wire;
            wire.domain = RigExecWireSlotDomain(uint8_t(range.domain));
            wire.begin = range.begin;
            wire.end = range.end;
            out.writes.push_back(wire);
        }
        out.preds = _ToI32s(step.preds);
        out.succs = _ToI32s(step.succs);
        out.isSource = step.isSource;
        out.externalReads = step.externalReads;
        out.varyingInputs = step.varyingInputs;
        out.resolvedInputReads = step.resolvedInputReads;
        out.overrideInputs = _ToI32s(step.overrideInputs);
        out.cluster = int32_t(step.cluster);
        out.level = int32_t(step.level);
        out.sizeUnits = step.sizeUnits;
        out.cost = step.cost;
        out.maxDiagnostics = uint32_t(step.maxDiagnostics);
        out.label = writer->AddString(step.label);
        steps.push_back(std::move(out));
    }
    return steps;
}

RigExecWireClustering
RigExecBakeConvertClustering(const RigExecBakedProgramImpl &program)
{
    RigExecWireClustering out;
    out.clusters.reserve(program.clustering.clusters.size());
    for (const RigExecBakedCluster &cluster :
         program.clustering.clusters) {
        RigExecWireCluster wire;
        wire.members = _ToI32s(cluster.members);
        wire.preds = _ToI32s(cluster.preds);
        wire.succs = _ToI32s(cluster.succs);
        wire.cost = cluster.cost;
        wire.level = int32_t(cluster.level);
        out.clusters.push_back(std::move(wire));
    }
    out.clusterOf = _ToI32s(program.clustering.clusterOf);
    out.grainUs = program.clustering.grainUs;
    out.serialCost = program.clustering.serialCost;
    out.criticalPathCost = program.clustering.criticalPathCost;
    return out;
}

RigExecWireCones
RigExecBakeConvertCones(const RigExecBakedProgramImpl &program)
{
    const uint32_t clusters =
        uint32_t(program.clustering.clusters.size());
    RigExecWireCones out;
    out.cone.reserve(program.cones.cone.size());
    for (const RigExecBakedClusterSet &set : program.cones.cone) {
        out.cone.push_back(_ToClusterSet(set, clusters));
    }
    out.always = _ToClusterSet(program.cones.always, clusters);
    out.poseClusters = _ToClusterSet(program.cones.poseClusters, clusters);
    out.avarCluster = _ToI32s(program.cones.avarCluster);
    out.chainBaseClusters = _ToNested(program.cones.chainBaseClusters);
    out.solverPointsClusters =
        _ToNested(program.cones.solverPointsClusters);
    out.revisionClusters = _ToNested(program.cones.revisionClusters);
    out.revisionStaticCluster =
        _ToI32s(program.cones.revisionStaticCluster);
    out.nativeSourceClusters =
        _ToNested(program.cones.nativeSourceClusters);
    out.deltaBaseClusters = _ToNested(program.cones.deltaBaseClusters);
    out.constraintArrayClusters =
        _ToNested(program.cones.constraintArrayClusters);
    out.varyingSteps = _ToI32s(program.cones.varyingSteps);
    out.overrideSteps = _ToI32s(program.cones.overrideSteps);
    return out;
}

namespace {

// The pose-table enums the wire stores as raw bytes, pinned like the step
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

void
_FillInputRoute(const UsdAttribute &head, bool varying, bool bound,
                bool viaResolved, int overrideIndex,
                RigExecBinaryWriter *writer, RigExecWireInput *out)
{
    out->varying = varying;
    out->bound = bound;
    out->viaResolved = viaResolved;
    out->overrideIndex = int32_t(overrideIndex);
    out->head = head ? writer->AddString(head.GetPath().GetString()) : 0;
}

template <class T>
void
_FillInputRoute(const RigExecBakedInput<T> &input,
                RigExecBinaryWriter *writer, RigExecWireInput *out)
{
    const bool viaResolved = bool(input.resolvedAttr);
    _FillInputRoute(input.head, input.varying,
                    input.query.IsValid() || viaResolved, viaResolved,
                    input.overrideIndex, writer, out);
}

RigExecWireInput
_ToInput(const RigExecBakedInput<double> &input,
         RigExecBinaryWriter *writer)
{
    RigExecWireInput out;
    out.tag = RigExecWireInput::Tag::Double;
    out.f64 = input.constant;
    _FillInputRoute(input, writer, &out);
    return out;
}

RigExecWireInput
_ToInput(const RigExecBakedInput<float> &input,
         RigExecBinaryWriter *writer)
{
    RigExecWireInput out;
    out.tag = RigExecWireInput::Tag::Float;
    out.f32 = input.constant;
    _FillInputRoute(input, writer, &out);
    return out;
}

RigExecWireInput
_ToInput(const RigExecBakedInput<bool> &input,
         RigExecBinaryWriter *writer)
{
    RigExecWireInput out;
    out.tag = RigExecWireInput::Tag::Bool;
    out.boolean = input.constant;
    _FillInputRoute(input, writer, &out);
    return out;
}

RigExecWireInput
_ToInput(const RigExecBakedInput<int> &input, RigExecBinaryWriter *writer)
{
    RigExecWireInput out;
    out.tag = RigExecWireInput::Tag::Int;
    out.i32 = int32_t(input.constant);
    _FillInputRoute(input, writer, &out);
    return out;
}

RigExecWireInput
_ToInput(const RigExecBakedInput<GfMatrix4d> &input,
         RigExecBinaryWriter *writer)
{
    RigExecWireInput out;
    out.tag = RigExecWireInput::Tag::Matrix4d;
    out.matrix = _ToMatrix(input.constant);
    _FillInputRoute(input, writer, &out);
    return out;
}

RigExecWireInput
_ToInput(const RigExecBakedInput<TfToken> &input,
         RigExecBinaryWriter *writer)
{
    RigExecWireInput out;
    out.tag = RigExecWireInput::Tag::Token;
    out.token = writer->AddString(input.constant.GetString());
    _FillInputRoute(input, writer, &out);
    return out;
}

RigExecWireInput
_ToInput(const RigExecBakedInput<GfVec3d> &input,
         RigExecBinaryWriter *writer)
{
    RigExecWireInput out;
    out.tag = RigExecWireInput::Tag::Vec3d;
    out.vec = _ToVec3d(input.constant);
    _FillInputRoute(input, writer, &out);
    return out;
}

RigExecWireVec3f
_ToVec3f(const GfVec3f &v)
{
    return RigExecWireVec3f{v[0], v[1], v[2]};
}

RigExecWireRbf
_ToRbf(const RigExecRbfSolver &solver)
{
    RigExecWireRbf out;
    out.poses.reserve(solver.GetPoses().size());
    for (const GfVec3d &pose : solver.GetPoses()) {
        out.poses.push_back(_ToVec3d(pose));
    }
    out.translations.reserve(solver.GetTranslations().size());
    for (const GfVec3d &t : solver.GetTranslations()) {
        out.translations.push_back(_ToVec3d(t));
    }
    out.poseTypes.reserve(solver.GetPoseTypes().size());
    for (RigExecRbfPoseType type : solver.GetPoseTypes()) {
        out.poseTypes.push_back(uint8_t(type));
    }
    out.twistAxis = _ToVec3d(solver.GetTwistAxis());
    out.kernel = uint8_t(solver.GetKernel());
    out.radius = solver.GetRadius();
    out.translationRadius = solver.GetTranslationRadius();
    out.radii = solver.GetRadii();
    out.translationRadii = solver.GetTranslationRadii();
    out.regularization = solver.GetRegularization();
    out.normalize = solver.GetNormalize();
    out.enableRotation = solver.GetEnableRotation();
    out.enableTranslation = solver.GetEnableTranslation();
    out.regularizedSingular = solver.GetRegularizedSingular();
    out.weights = solver.GetWeights();
    return out;
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

}  // namespace

RigExecWireDomainPose
RigExecBakeConvertDomainPose(const RigExecBakedProgramImpl &program,
                             RigExecBinaryWriter *writer)
{
    RigExecWireDomainPose pose;
    pose.ladders.reserve(program.ladders.size());
    for (const RigExecBakedProgramImpl::Ladder &ladder : program.ladders) {
        RigExecWireLadder out;
        out.restSpace = _ToInput(ladder.restSpace, writer);
        out.defaultSpace = _ToInput(ladder.defaultSpace, writer);
        out.posedSpace = _ToInput(ladder.posedSpace, writer);
        for (size_t i = 0; i < 6; ++i) {
            out.restAvars[i] = _ToInput(ladder.restAvars[i], writer);
            out.defaultAvars[i] = _ToInput(ladder.defaultAvars[i], writer);
        }
        out.rotationOrder = _ToInput(ladder.rotationOrder, writer);
        pose.ladders.push_back(std::move(out));
    }
    pose.ladderVarying = program.ladderVarying;
    pose.ladderOverrides = _ToI32s(program.ladderOverrides);
    pose.restChainVaries.reserve(program.restChainVaries.size());
    for (char v : program.restChainVaries) {
        pose.restChainVaries.push_back(v ? uint8_t(1) : uint8_t(0));
    }
    pose.poseInterpolators.reserve(program.poseInterpolators.size());
    for (const RigExecBakedProgramImpl::PoseInterpolator &interp :
         program.poseInterpolators) {
        RigExecWirePoseInterpolator out;
        out.path = writer->AddString(interp.path.GetString());
        out.driverSlot = int32_t(interp.driverSlot);
        out.parentSlot = int32_t(interp.parentSlot);
        out.allowNegativeWeights = interp.allowNegativeWeights;
        out.enabled = _ToInput(interp.enabled, writer);
        out.weightBegin = int32_t(interp.weightBegin);
        out.weightEnd = int32_t(interp.weightEnd);
        out.poseSlots = _ToI32s(interp.poseSlots);
        out.disabledSlots = _ToI32s(interp.disabledSlots);
        out.solver = _ToRbf(interp.solver);
        pose.poseInterpolators.push_back(std::move(out));
    }
    pose.poseWeightPaths.reserve(program.poseWeightPaths.size());
    for (const SdfPath &path : program.poseWeightPaths) {
        pose.poseWeightPaths.push_back(writer->AddString(path.GetString()));
    }
    pose.solvers.reserve(program.solvers.size());
    for (const RigExecBakedProgramImpl::Solver &solver : program.solvers) {
        RigExecWireSolver out;
        out.path = writer->AddString(solver.path.GetString());
        out.type = writer->AddString(solver.type.GetString());
        out.restSlots = _ToI32s(solver.restSlots);
        out.restRefs.reserve(solver.restRefs.size());
        for (const auto &ref : solver.restRefs) {
            out.restRefs.emplace_back(int32_t(ref.first),
                                      int32_t(ref.second));
        }
        out.restIsLive.reserve(solver.restIsLive.size());
        for (size_t i = 0; i < solver.restIsLive.size(); ++i) {
            out.restIsLive.push_back(solver.restIsLive[i] ? uint8_t(1)
                                                          : uint8_t(0));
        }
        out.restReads = solver.restReads;
        out.hasLiveRest = solver.hasLiveRest;
        out.jointRests.reserve(solver.jointRests.size());
        for (const std::array<GfVec3d, 4> &set : solver.jointRests) {
            std::array<RigExecWireVec3d, 4> wire;
            for (size_t i = 0; i < 4; ++i) {
                wire[i] = _ToVec3d(set[i]);
            }
            out.jointRests.push_back(wire);
        }
        out.restsVary = solver.restsVary;
        out.restOverrides = _ToI32s(solver.restOverrides);
        out.splineRestWeights = solver.splineRestWeights;
        out.splineRestMode = uint8_t(solver.splineRestMode);
        out.degenerate = solver.degenerate;
        out.controls = _ToI32s(solver.controls);
        out.parentRelative = solver.parentRelative;
        out.controlRests.reserve(solver.controlRests.size());
        for (const std::array<GfVec3d, 4> &set : solver.controlRests) {
            std::array<RigExecWireVec3d, 4> wire;
            for (size_t i = 0; i < 4; ++i) {
                wire[i] = _ToVec3d(set[i]);
            }
            out.controlRests.push_back(wire);
        }
        // Memory only: the solver record layout is frozen, so the
        // SolverStart section carries these (see bake.cpp).
        out.start = int32_t(solver.start);
        for (size_t i = 0; i < 4; ++i) {
            out.startRest[i] = _ToVec3d(solver.startRest[i]);
        }
        out.startRead = solver.startRead;
        out.root = int32_t(solver.root);
        out.mid = int32_t(solver.mid);
        out.end = int32_t(solver.end);
        out.pole = int32_t(solver.pole);
        for (size_t k = 0; k < 3; ++k) {
            for (size_t i = 0; i < 4; ++i) {
                out.ikRests[k][i] = _ToVec3d(solver.ikRests[k][i]);
            }
        }
        out.ikParams.upperLength = solver.ikParams.upperLength;
        out.ikParams.lowerLength = solver.ikParams.lowerLength;
        out.ikParams.stretch = solver.ikParams.stretch;
        out.ikParams.softness = solver.ikParams.softness;
        out.ikParams.preferredBendRadians =
            solver.ikParams.preferredBendRadians;
        out.bend = _ToInput(solver.bend, writer);
        out.upperOffset = _ToInput(solver.upperOffset, writer);
        out.lowerOffset = _ToInput(solver.lowerOffset, writer);
        out.stretch = _ToInput(solver.stretch, writer);
        out.softness = _ToInput(solver.softness, writer);
        out.upperLengthBase = solver.upperLengthBase;
        out.lowerLengthBase = solver.lowerLengthBase;
        out.inA = int32_t(solver.inA);
        out.inB = int32_t(solver.inB);
        out.blendWeight = _ToInput(solver.blendWeight, writer);
        out.scaleMode = uint8_t(solver.scaleMode);
        out.blendRotationRejected = solver.blendRotationRejected;
        for (size_t i = 0; i < 4; ++i) {
            out.splineRest.cvs[i] = _ToVec3d(solver.splineRest.cvs[i]);
        }
        out.splineRest.rootControl = _ToFrame(solver.splineRest.rootControl);
        out.splineRest.midControl = _ToFrame(solver.splineRest.midControl);
        out.splineRest.endControl = _ToFrame(solver.splineRest.endControl);
        out.splineRest.joints.reserve(solver.splineRest.joints.size());
        for (const RigExecPointFrame &joint : solver.splineRest.joints) {
            out.splineRest.joints.push_back(_ToFrame(joint));
        }
        out.splineRest.segmentLengths = solver.splineRest.segmentLengths;
        out.splineRest.restArcLength = solver.splineRest.restArcLength;
        out.splineRest.volumeWeights = solver.splineRest.volumeWeights;
        out.splineParams.preserveVolume =
            solver.splineParams.preserveVolume;
        out.splineParams.midFollowWeight =
            solver.splineParams.midFollowWeight;
        out.splineParams.roll = solver.splineParams.roll;
        out.splineParams.twist = solver.splineParams.twist;
        out.splineParams.minLengthRatio =
            solver.splineParams.minLengthRatio;
        out.splineParams.aimRootTangent =
            solver.splineParams.aimRootTangent;
        out.splineJointRests.reserve(solver.splineJointRests.size());
        for (const std::array<GfVec3d, 4> &set : solver.splineJointRests) {
            std::array<RigExecWireVec3d, 4> wire;
            for (size_t i = 0; i < 4; ++i) {
                wire[i] = _ToVec3d(set[i]);
            }
            out.splineJointRests.push_back(wire);
        }
        out.splineCount = uint64_t(solver.splineCount);
        out.preserveVolume = _ToInput(solver.preserveVolume, writer);
        out.midFollowWeight = _ToInput(solver.midFollowWeight, writer);
        out.roll = _ToInput(solver.roll, writer);
        out.twist = _ToInput(solver.twist, writer);
        out.minLengthRatio = _ToInput(solver.minLengthRatio, writer);
        out.splineParamsVary = solver.splineParamsVary;
        for (size_t i = 0; i < 4; ++i) {
            out.twistStartRest[i] = _ToVec3d(solver.twistStartRest[i]);
            out.twistEndRest[i] = _ToVec3d(solver.twistEndRest[i]);
        }
        out.twistWeights = solver.twistWeights;
        out.twistTurns = _ToInput(solver.twistTurns, writer);
        out.ribbonPointsPath =
            writer->AddString(solver.ribbonPointsPath.GetString());
        out.ribbonRestPoints.reserve(solver.ribbonRestPoints.size());
        for (const GfVec3f &p : solver.ribbonRestPoints) {
            out.ribbonRestPoints.push_back(_ToVec3f(p));
        }
        out.ribbonConstantPoints.reserve(
            solver.ribbonConstantPoints.size());
        for (const GfVec3f &p : solver.ribbonConstantPoints) {
            out.ribbonConstantPoints.push_back(_ToVec3f(p));
        }
        out.ribbonPointsVarying = solver.ribbonPointsVarying;
        out.ribbonSampleCount = _ToInput(solver.ribbonSampleCount, writer);
        out.outputs.reserve(solver.outputs.size());
        for (const auto &output : solver.outputs) {
            out.outputs.emplace_back(int32_t(output.first),
                                     int32_t(output.second));
        }
        out.outPosition = _ToI32s(solver.outPosition);
        out.controlReads = solver.controlReads;
        out.rootRead = solver.rootRead;
        out.midRead = solver.midRead;
        out.endRead = solver.endRead;
        out.poleRead = solver.poleRead;
        pose.solvers.push_back(std::move(out));
    }
    pose.guideSolvers = _ToI32s(program.guideSolvers);
    pose.constraints.reserve(program.constraints.size());
    for (const RigExecBakedProgramImpl::Constraint &constraint :
         program.constraints) {
        RigExecWireConstraint out;
        out.path = writer->AddString(constraint.path.GetString());
        out.type = writer->AddString(constraint.type.GetString());
        out.weightObject =
            writer->AddString(constraint.weightObject.GetString());
        out.target = int32_t(constraint.target);
        out.targetSlots = _ToI32s(constraint.targetSlots);
        out.snapshotTargets.reserve(constraint.snapshotTargets.size());
        for (char v : constraint.snapshotTargets) {
            out.snapshotTargets.push_back(v ? uint8_t(1) : uint8_t(0));
        }
        out.sources = _ToI32s(constraint.sources);
        out.sourceNatives = _ToI32s(constraint.sourceNatives);
        out.sourcePaths.reserve(constraint.sourcePaths.size());
        for (const SdfPath &path : constraint.sourcePaths) {
            out.sourcePaths.push_back(writer->AddString(path.GetString()));
        }
        out.arrays = int32_t(constraint.arrays);
        out.enabled = _ToInput(constraint.enabled, writer);
        out.defaultWeight = _ToInput(constraint.defaultWeight, writer);
        out.offset = _ToInput(constraint.offset, writer);
        out.affectX = _ToInput(constraint.affectX, writer);
        out.affectY = _ToInput(constraint.affectY, writer);
        out.affectZ = _ToInput(constraint.affectZ, writer);
        out.tX = _ToInput(constraint.tX, writer);
        out.tY = _ToInput(constraint.tY, writer);
        out.tZ = _ToInput(constraint.tZ, writer);
        out.rX = _ToInput(constraint.rX, writer);
        out.rY = _ToInput(constraint.rY, writer);
        out.rZ = _ToInput(constraint.rZ, writer);
        out.sX = _ToInput(constraint.sX, writer);
        out.sY = _ToInput(constraint.sY, writer);
        out.sZ = _ToInput(constraint.sZ, writer);
        out.order = uint8_t(constraint.order);
        out.aimVector = _ToInput(constraint.aimVector, writer);
        out.upVector = _ToInput(constraint.upVector, writer);
        out.rotationOffset = _ToInput(constraint.rotationOffset, writer);
        out.worldUpVector = _ToInput(constraint.worldUpVector, writer);
        out.aimAxisFallback = _ToVec3d(constraint.aimAxisFallback);
        out.aimVectorAuthored = constraint.aimVectorAuthored;
        out.preserveInputUp = constraint.preserveInputUp;
        out.worldUpType =
            writer->AddString(constraint.worldUpType.GetString());
        out.sceneUp = _ToVec3d(constraint.sceneUp);
        out.pointsTarget =
            writer->AddString(constraint.pointsTarget.GetString());
        out.deltaBasePath =
            writer->AddString(constraint.deltaBasePath.GetString());
        out.deltaBase = int32_t(constraint.deltaBase);
        out.worldUpObject = int32_t(constraint.worldUpObject);
        out.worldUpNative = int32_t(constraint.worldUpNative);
        out.worldUpPath =
            writer->AddString(constraint.worldUpPath.GetString());
        out.worldUpObjectNamed = constraint.worldUpObjectNamed;
        out.snapshotAfter = constraint.snapshotAfter;
        out.singleChainIk = constraint.singleChainIk;
        out.ikMode = uint8_t(constraint.ikMode);
        out.poleModeObject = constraint.poleModeObject;
        out.useAnimatedTs = constraint.useAnimatedTs;
        out.ikRestLive.reserve(constraint.ikRestLive.size());
        for (char v : constraint.ikRestLive) {
            out.ikRestLive.push_back(v ? uint8_t(1) : uint8_t(0));
        }
        out.effector = int32_t(constraint.effector);
        out.effectorNative = int32_t(constraint.effectorNative);
        out.effectorPath =
            writer->AddString(constraint.effectorPath.GetString());
        out.poleObjects = _ToI32s(constraint.poleObjects);
        out.poleObjectNatives = _ToI32s(constraint.poleObjectNatives);
        out.poleVector = _ToInput(constraint.poleVector, writer);
        out.twistDegrees = _ToInput(constraint.twistDegrees, writer);
        pose.constraints.push_back(std::move(out));
    }
    pose.constraintArrays.reserve(program.constraintArrays.size());
    for (const RigExecBakedProgramImpl::ConstraintArrays &arrays :
         program.constraintArrays) {
        RigExecWireConstraintArrays out;
        out.prim = arrays.prim
                       ? writer->AddString(arrays.prim.GetPath().GetString())
                       : 0;
        out.sourceCount = uint64_t(arrays.sourceCount);
        out.parentOffsets = arrays.parentOffsets;
        out.readPole = arrays.readPole;
        out.poleCount = uint64_t(arrays.poleCount);
        pose.constraintArrays.push_back(std::move(out));
    }
    pose.nativeSources.reserve(program.nativeSources.size());
    for (const RigExecBakedProgramImpl::NativeXformSource &source :
         program.nativeSources) {
        RigExecWireNativeSource out;
        out.path = writer->AddString(source.path.GetString());
        out.ancestorSlots = _ToI32s(source.ancestorSlots);
        pose.nativeSources.push_back(std::move(out));
    }
    pose.walkSteps.reserve(program.walkSteps.size());
    for (const RigExecBakedProgramImpl::WalkStep &step : program.walkSteps) {
        RigExecWireWalkStep out;
        out.solverBatch = step.solverBatch;
        out.level = uint64_t(step.level);
        out.index = int32_t(step.index);
        out.batchSolvers = _ToI32s(step.batchSolvers);
        out.propagate.reserve(step.propagate.size());
        for (const auto &pair : step.propagate) {
            out.propagate.emplace_back(int32_t(pair.first),
                                       int32_t(pair.second));
        }
        pose.walkSteps.push_back(std::move(out));
    }
    pose.composeGroups.reserve(program.composeGroups.size());
    for (const RigExecBakedComposeGroup &group : program.composeGroups) {
        RigExecWireComposeGroup out;
        out.begin = int32_t(group.begin);
        out.end = int32_t(group.end);
        out.parentSlots = _ToI32s(group.parentSlots);
        pose.composeGroups.push_back(std::move(out));
    }
    pose.commits.reserve(program.commits.size());
    for (const RigExecBakedCommit &commit : program.commits) {
        RigExecWireCommit out;
        out.moverPath = writer->AddString(commit.moverPath.GetString());
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
            out.sources.push_back(std::move(wire));
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
        out.poleAncestors.reserve(commit.poleAncestors.size());
        for (const auto &row : commit.poleAncestors) {
            std::vector<RigExecWireAncestorRead> wire;
            wire.reserve(row.size());
            for (const auto &read : row) {
                wire.push_back(_ToAncestorRead(read));
            }
            out.poleAncestors.push_back(std::move(wire));
        }
        out.recordAfter = commit.recordAfter;
        out.recordEveryTarget = commit.recordEveryTarget;
        out.sourceAncestors.reserve(commit.sourceAncestors.size());
        for (const auto &row : commit.sourceAncestors) {
            std::vector<RigExecWireAncestorRead> wire;
            wire.reserve(row.size());
            for (const auto &read : row) {
                wire.push_back(_ToAncestorRead(read));
            }
            out.sourceAncestors.push_back(std::move(wire));
        }
        out.worldUpAncestors.reserve(commit.worldUpAncestors.size());
        for (const auto &read : commit.worldUpAncestors) {
            out.worldUpAncestors.push_back(_ToAncestorRead(read));
        }
        pose.commits.push_back(std::move(out));
    }
    if (program.jointSolverBinding) {
        for (const auto &entry : *program.jointSolverBinding) {
            pose.jointBindingJoints.push_back(
                writer->AddString(entry.first.GetString()));
            std::vector<uint32_t> solvers;
            std::vector<int32_t> elements;
            solvers.reserve(entry.second.size());
            elements.reserve(entry.second.size());
            for (const auto &stack : entry.second) {
                solvers.push_back(
                    writer->AddString(stack.first.GetString()));
                elements.push_back(int32_t(stack.second));
            }
            pose.jointBindingSolvers.push_back(std::move(solvers));
            pose.jointBindingElements.push_back(std::move(elements));
        }
    }
    pose.hasPropertyChains = program.hasPropertyChains;
    pose.phasedReads = program.phasedReads;
    pose.publishWeightFields = program.publishWeightFields;
    return pose;
}

namespace {

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
_RIGEXEC_BAKE_PIN_OP(Curvenet, 10);
_RIGEXEC_BAKE_PIN_OP(CurvenetAdjuster, 11);
_RIGEXEC_BAKE_PIN_OP(RecomputeNormals, 12);
_RIGEXEC_BAKE_PIN_OP(RecomputeExtent, 13);
#undef _RIGEXEC_BAKE_PIN_OP
#define _RIGEXEC_BAKE_PIN_PHASE(name, value)                               \
    static_assert(uint8_t(RigExecReadPhaseKind::name) == value,            \
                  "wire read-phase order drifted")
_RIGEXEC_BAKE_PIN_PHASE(Base, 0);
_RIGEXEC_BAKE_PIN_PHASE(Preceding, 1);
_RIGEXEC_BAKE_PIN_PHASE(Final, 2);
_RIGEXEC_BAKE_PIN_PHASE(AtPrim, 3);
#undef _RIGEXEC_BAKE_PIN_PHASE

uint32_t
_PathRef(const SdfPath &path, RigExecBinaryWriter *writer)
{
    return writer->AddString(path.GetString());
}

uint32_t
_TokenRef(const TfToken &token, RigExecBinaryWriter *writer)
{
    return writer->AddString(token.GetString());
}

RigExecWireReadPhase
_ToPhase(const RigExecReadPhase &phase, RigExecBinaryWriter *writer)
{
    RigExecWireReadPhase out;
    out.kind = uint8_t(phase.kind);
    out.prim = _PathRef(phase.prim, writer);
    return out;
}

RigExecWireRevisionBinding
_ToBinding(const RigExecRevisionBinding &binding,
           RigExecBinaryWriter *writer)
{
    RigExecWireRevisionBinding out;
    out.moverPath = _PathRef(binding.moverPath, writer);
    out.target = _PathRef(binding.target, writer);
    out.transform = _PathRef(binding.transform, writer);
    out.transformSpace = _PathRef(binding.transformSpace, writer);
    out.influences.reserve(binding.influences.size());
    for (const SdfPath &path : binding.influences) {
        out.influences.push_back(_PathRef(path, writer));
    }
    out.weightObject = _PathRef(binding.weightObject, writer);
    out.base = _PathRef(binding.base, writer);
    out.topologyCounts = _PathRef(binding.topologyCounts, writer);
    out.topologyIndices = _PathRef(binding.topologyIndices, writer);
    out.cagePoints = _PathRef(binding.cagePoints, writer);
    out.surfacePoints = _PathRef(binding.surfacePoints, writer);
    out.bindCoords = _PathRef(binding.bindCoords, writer);
    out.driverCurvePoints = _PathRef(binding.driverCurvePoints, writer);
    out.driverCurveOrder = _PathRef(binding.driverCurveOrder, writer);
    out.driverCurveKnots = _PathRef(binding.driverCurveKnots, writer);
    out.driverTransformCount = int32_t(binding.driverTransformCount);
    out.driverSpaceCount = int32_t(binding.driverSpaceCount);
    out.driverBaseTransformCount =
        int32_t(binding.driverBaseTransformCount);
    out.driverFrames = _PathRef(binding.driverFrames, writer);
    out.widths = _PathRef(binding.widths, writer);
    out.curvenet = _PathRef(binding.curvenet, writer);
    out.curvenetPoints = _PathRef(binding.curvenetPoints, writer);
    out.blendInputs.reserve(binding.blendInputs.size());
    for (const SdfPath &path : binding.blendInputs) {
        out.blendInputs.push_back(_PathRef(path, writer));
    }
    out.blendSampleInputs.reserve(binding.blendSamples.size());
    out.blendSamples.reserve(binding.blendSamples.size());
    for (const auto &entry : binding.blendSamples) {
        out.blendSampleInputs.push_back(_PathRef(entry.first, writer));
        std::vector<RigExecWireBlendSampleBinding> row;
        row.reserve(entry.second.size());
        for (const RigExecBlendSampleBinding &sample : entry.second) {
            RigExecWireBlendSampleBinding wire;
            wire.sample = _PathRef(sample.sample, writer);
            wire.points = _PathRef(sample.points, writer);
            wire.phase = _ToPhase(sample.phase, writer);
            wire.blendShape = _PathRef(sample.blendShape, writer);
            row.push_back(std::move(wire));
        }
        out.blendSamples.push_back(std::move(row));
    }
    out.phaseInputs.reserve(binding.phases.size());
    out.phases.reserve(binding.phases.size());
    for (const auto &entry : binding.phases) {
        out.phaseInputs.push_back(_PathRef(entry.first, writer));
        out.phases.push_back(_ToPhase(entry.second, writer));
    }
    out.transformPhase = _ToPhase(binding.transformPhase, writer);
    return out;
}

RigExecWireSkinTopology
_ToTopology(const std::shared_ptr<const RigExecSkinTopology> &topology)
{
    RigExecWireSkinTopology out;
    out.hasTopology = bool(topology);
    if (!topology) {
        return out;
    }
    out.indices = topology->indices;
    out.weights = topology->weights;
    out.elementSize = int32_t(topology->elementSize);
    out.pointCount = uint64_t(topology->pointCount);
    out.influenceCount = uint64_t(topology->influenceCount);
    out.validated = topology->validated;
    return out;
}

RigExecWireRevision
_ToRevision(const RigExecBakedProgramImpl::GeomRevision &revision,
            RigExecBinaryWriter *writer)
{
    RigExecWireRevision out;
    out.moverPath = _PathRef(revision.moverPath, writer);
    out.target = _PathRef(revision.target, writer);
    out.moverPrim = revision.moverPrim
                        ? _PathRef(revision.moverPrim.GetPath(), writer)
                        : 0;
    out.op = uint8_t(revision.op);
    out.binding = _ToBinding(revision.binding, writer);
    out.curvenetChain = int32_t(revision.curvenetChain);
    out.curvenetBindResolved = revision.curvenetBindResolved;
    out.hasCurvenetBind = bool(revision.curvenetBind);
    out.curvenetRestNet.reserve(revision.curvenetBindInputs.restNet.size());
    for (const GfVec3f &p : revision.curvenetBindInputs.restNet) {
        out.curvenetRestNet.push_back(_ToVec3f(p));
    }
    out.curvenetSplineIndices =
        _ToI32s(revision.curvenetBindInputs.splineIndices);
    out.curvenetSamplesPerSpline =
        int32_t(revision.curvenetBindInputs.samplesPerSpline);
    out.curvenetBasis =
        _TokenRef(revision.curvenetBindInputs.basis, writer);
    out.curvenetMeshPoints.reserve(
        revision.curvenetBindInputs.meshPoints.size());
    for (const GfVec3f &p : revision.curvenetBindInputs.meshPoints) {
        out.curvenetMeshPoints.push_back(_ToVec3f(p));
    }
    out.curvenetMeshCounts =
        _ToI32s(revision.curvenetBindInputs.meshCounts);
    out.curvenetMeshIndices =
        _ToI32s(revision.curvenetBindInputs.meshIndices);
    out.curvenetBindInputsHeld = revision.curvenetBindInputs.held;
    out.blendChannels.reserve(revision.blendChannels.size());
    for (const RigExecBakedProgramImpl::GeomBlendChannel &channel :
         revision.blendChannels) {
        RigExecWireBlendChannel wire;
        wire.weight = channel.weight
                          ? _PathRef(channel.weight.GetPath(), writer)
                          : 0;
        wire.weightValid = bool(channel.weight);
        wire.weightPath = _PathRef(channel.weightPath, writer);
        wire.poseWeight = int32_t(channel.poseWeight);
        wire.samples.reserve(channel.samples.size());
        for (const RigExecBakedProgramImpl::GeomBlendChannel::Sample &sample :
             channel.samples) {
            RigExecWireBlendChannel::Sample wireSample;
            wireSample.samplePath = _PathRef(sample.samplePath, writer);
            wireSample.activation = sample.activation
                                        ? _PathRef(sample.activation.GetPath(),
                                                   writer)
                                        : 0;
            wireSample.activationValid = bool(sample.activation);
            wireSample.points = sample.points
                                    ? _PathRef(sample.points.GetPath(),
                                               writer)
                                    : 0;
            wireSample.pointsValid = bool(sample.points);
            wireSample.pointsPath = _PathRef(sample.pointsPath, writer);
            wireSample.phase = _ToPhase(sample.phase, writer);
            wireSample.blendShape = _PathRef(sample.blendShape, writer);
            wireSample.hasLayout = bool(sample.layout);
            if (sample.layout) {
                wireSample.offsets.reserve(sample.layout->offsets.size());
                for (const GfVec3f &p : sample.layout->offsets) {
                    wireSample.offsets.push_back(_ToVec3f(p));
                }
                wireSample.indices = sample.layout->indices;
                wireSample.pointCount =
                    uint64_t(sample.layout->pointCount);
                wireSample.layoutValid = sample.layout->valid;
            }
            wire.samples.push_back(std::move(wireSample));
        }
        out.blendChannels.push_back(std::move(wire));
    }
    out.influenceSlots = _ToI32s(revision.influenceSlots);
    out.transformSlot = int32_t(revision.transformSlot);
    out.transformSpaceSlot = int32_t(revision.transformSpaceSlot);
    out.constraintDelta = int32_t(revision.constraintDelta);
    out.driverFramesSolver = int32_t(revision.driverFramesSolver);
    out.finalPhase = revision.finalPhase;
    out.skinTopologyFixed = revision.skinTopologyFixed;
    out.snapshotAfter = revision.snapshotAfter;
    out.readsSnapshots = revision.readsSnapshots;
    out.packetInfluences.reserve(revision.packetInfluences.size());
    for (const GfMatrix4d &m : revision.packetInfluences) {
        out.packetInfluences.push_back(_ToMatrix(m));
    }
    out.chunks.reserve(revision.chunks.size());
    for (const RigExecBakedProgramImpl::GeomChunk &chunk : revision.chunks) {
        RigExecWireChunk wire;
        wire.begin = int32_t(chunk.begin);
        wire.end = int32_t(chunk.end);
        wire.key = chunk.key;
        out.chunks.push_back(std::move(wire));
    }
    out.chunkBase = int32_t(revision.chunkBase);
    out.partitionTopology = _ToTopology(revision.partitionTopology);
    out.partitionElementSize = int32_t(revision.partitionElementSize);
    out.partitionIndexCount = uint64_t(revision.partitionIndexCount);
    out.partitionPointCount = uint64_t(revision.partitionPointCount);
    out.chunked = revision.chunked;
    out.partitionCandidates = uint64_t(revision.partitionCandidates);
    out.partitionReadyMin = int32_t(revision.partitionReadyMin);
    out.partitionReadyMax = int32_t(revision.partitionReadyMax);
    out.weightObject = int32_t(revision.weightObject);
    out.weightOperationDomain = revision.weightOperationDomain;
    out.weightFieldTarget = _PathRef(revision.weightFieldTarget, writer);
    out.weightCurrentPhase = revision.weightCurrentPhase;
    out.topology = _ToTopology(revision.topology);
    out.topologyResolved = revision.topologyResolved;
    return out;
}

void
_ToAttributes(const std::vector<UsdAttribute> &attrs,
              RigExecBinaryWriter *writer, std::vector<uint32_t> *paths,
              std::vector<uint8_t> *valid)
{
    paths->reserve(attrs.size());
    valid->reserve(attrs.size());
    for (const UsdAttribute &attr : attrs) {
        paths->push_back(attr ? _PathRef(attr.GetPath(), writer) : 0);
        valid->push_back(attr ? uint8_t(1) : uint8_t(0));
    }
}

}  // namespace

RigExecWireDomainGeometry
RigExecBakeConvertDomainGeometry(const RigExecBakedProgramImpl &program,
                                 RigExecBinaryWriter *writer)
{
    RigExecWireDomainGeometry geometry;
    geometry.chains.reserve(program.chains.size());
    for (const RigExecBakedProgramImpl::GeomChain &chain : program.chains) {
        RigExecWireChain wire;
        wire.target = _PathRef(chain.target, writer);
        wire.revisions.reserve(chain.revisions.size());
        for (const RigExecBakedProgramImpl::GeomRevision &revision :
             chain.revisions) {
            wire.revisions.push_back(_ToRevision(revision, writer));
        }
        wire.derived.reserve(chain.derived.size());
        for (const RigExecBakedProgramImpl::GeomChain::Derived &derived :
             chain.derived) {
            RigExecWireDerived wireDerived;
            wireDerived.target = _PathRef(derived.target, writer);
            wireDerived.revision = _ToRevision(derived.revision, writer);
            wire.derived.push_back(std::move(wireDerived));
        }
        geometry.chains.push_back(std::move(wire));
    }
    geometry.revisionIndex.reserve(program.revisionIndex.size());
    for (const auto &entry : program.revisionIndex) {
        geometry.revisionIndex.emplace_back(int32_t(entry.first),
                                            int32_t(entry.second));
    }
    geometry.derivedIndex.reserve(program.derivedIndex.size());
    for (const auto &entry : program.derivedIndex) {
        geometry.derivedIndex.emplace_back(int32_t(entry.first),
                                           int32_t(entry.second));
    }
    geometry.chainRevisionBegin = _ToI32s(program.chainRevisionBegin);
    geometry.chainRevisionEnd = _ToI32s(program.chainRevisionEnd);
    geometry.revisionChunkBase = _ToI32s(program.revisionChunkBase);
    geometry.revisionChunkCount = _ToI32s(program.revisionChunkCount);
    geometry.chainChunkBegin = _ToI32s(program.chainChunkBegin);
    geometry.chainChunkEnd = _ToI32s(program.chainChunkEnd);
    geometry.weightObjects.reserve(program.weightObjects.size());
    for (const RigExecBakedProgramImpl::WeightObject &object :
         program.weightObjects) {
        RigExecWireWeightObject wire;
        wire.path = _PathRef(object.path, writer);
        wire.type = _TokenRef(object.type, writer);
        wire.representation = _TokenRef(object.representation, writer);
        wire.rangePolicy = _TokenRef(object.rangePolicy, writer);
        wire.values = object.values;
        wire.indices = object.indices;
        wire.defaultWeight = _ToInput(object.defaultWeight, writer);
        wire.base = int32_t(object.base);
        wire.inputs = _ToI32s(object.inputs);
        wire.driver = _ToInput(object.driver, writer);
        wire.scale = _ToInput(object.scale, writer);
        wire.bias = _ToInput(object.bias, writer);
        wire.combineMode = _TokenRef(object.combineMode, writer);
        wire.strength = _ToInput(object.strength, writer);
        wire.invert = _ToInput(object.invert, writer);
        _ToAttributes(object.combineTargetPoints, writer,
                      &wire.combineTargetPoints, &wire.combineTargetValid);
        wire.costElements = uint64_t(object.costElements);
        wire.providerSlot = int32_t(object.providerSlot);
        wire.falloffMin = _ToInput(object.falloffMin, writer);
        wire.falloffMax = _ToInput(object.falloffMax, writer);
        wire.scaleX = _ToInput(object.scaleX, writer);
        wire.scaleY = _ToInput(object.scaleY, writer);
        wire.scaleZ = _ToInput(object.scaleZ, writer);
        wire.extentU = _ToInput(object.extentU, writer);
        wire.extentV = _ToInput(object.extentV, writer);
        wire.planeAxis = _TokenRef(object.planeAxis, writer);
        wire.planeBounds = _TokenRef(object.planeBounds, writer);
        _ToAttributes(object.targetPoints, writer, &wire.targetPoints,
                      &wire.targetValid);
        _ToAttributes(object.samplePoints, writer, &wire.samplePoints,
                      &wire.sampleValid);
        _ToAttributes(object.curvePoints, writer, &wire.curvePoints,
                      &wire.curveValid);
        wire.falloffCurve = object.falloffCurve;
        _ToAttributes(object.curvenetMeshPoints, writer,
                      &wire.curvenetMeshPoints, &wire.curvenetMeshValid);
        _ToAttributes(object.curvenetPoints, writer, &wire.curvenetPoints,
                      &wire.curvenetPointsValid);
        _ToAttributes(object.curvenetCounts, writer, &wire.curvenetCounts,
                      &wire.curvenetCountsValid);
        _ToAttributes(object.curvenetIndices, writer, &wire.curvenetIndices,
                      &wire.curvenetIndicesValid);
        _ToAttributes(object.curvenetSplines, writer, &wire.curvenetSplines,
                      &wire.curvenetSplinesValid);
        wire.curvenetWeights = object.curvenetWeights
                                   ? _PathRef(object.curvenetWeights.GetPath(),
                                              writer)
                                   : 0;
        wire.curvenetWeightsValid = bool(object.curvenetWeights);
        wire.curvenetAutoSmooth = object.curvenetAutoSmooth
                                      ? _PathRef(object.curvenetAutoSmooth
                                                     .GetPath(),
                                                 writer)
                                      : 0;
        wire.curvenetAutoSmoothValid = bool(object.curvenetAutoSmooth);
        wire.curvenetBasis = _TokenRef(object.curvenetBasis, writer);
        wire.curvenetSamples = _ToInput(object.curvenetSamples, writer);
        wire.curvenetUnreached = _ToInput(object.curvenetUnreached, writer);
        wire.boundMesh.reserve(object.boundMesh.size());
        for (const GfVec3f &p : object.boundMesh) {
            wire.boundMesh.push_back(_ToVec3f(p));
        }
        wire.boundNet.reserve(object.boundNet.size());
        for (const GfVec3f &p : object.boundNet) {
            wire.boundNet.push_back(_ToVec3f(p));
        }
        wire.boundCounts = object.boundCounts;
        wire.boundIndices = object.boundIndices;
        wire.boundSmooth = object.boundSmooth;
        wire.boundSplines = object.boundSplines;
        wire.boundSamples = int32_t(object.boundSamples);
        wire.bound = object.bound;
        geometry.weightObjects.push_back(std::move(wire));
    }
    geometry.falloffPaths.reserve(program.falloffLuts.size());
    geometry.falloffLuts.reserve(program.falloffLuts.size());
    for (const auto &entry : program.falloffLuts) {
        geometry.falloffPaths.push_back(_PathRef(entry.first, writer));
        geometry.falloffLuts.push_back(entry.second);
    }
    geometry.currentPhaseWeights.reserve(program.currentPhaseWeights.size());
    for (const SdfPath &path : program.currentPhaseWeights) {
        geometry.currentPhaseWeights.push_back(_PathRef(path, writer));
    }
    geometry.deltaBasePaths.reserve(program.deltaBasePaths.size());
    for (const SdfPath &path : program.deltaBasePaths) {
        geometry.deltaBasePaths.push_back(_PathRef(path, writer));
    }
    return geometry;
}

}  // namespace rigExec
