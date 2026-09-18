//
// testRigExecBinary comparison: the decoded wire structs against the live
// program they were converted from, field by field.
//
// Included by tests/testRigExecBinary.cpp AFTER its CHECK macro: every
// comparison below reports through it. Doubles compare with exact equality,
// which is correct here because the converter copies values and the wire
// preserves bits -- no arithmetic stands between the two sides.
//

#include "rigExec/bakedProgramImpl.h"
#include "rigExecBinary/container.h"
#include "rigExecBinary/geometry.h"
#include "rigExecBinary/inputTable.h"
#include "rigExecBinary/pose.h"
#include "rigExecBinary/program.h"

#include "pxr/base/gf/vec2f.h"
#include "pxr/base/gf/vec3i.h"
#include "pxr/base/vt/array.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/timeCode.h"

#include <functional>
#include <string>

namespace {

std::string
_BinaryString(const rigExec::RigExecBinaryReader &reader, uint32_t index)
{
    std::string out;
    CHECK(reader.GetString(index, &out));
    return out;
}

template <class A, class B>
void
_BinaryCheckEqual(const std::vector<A> &a, const std::vector<B> &b)
{
    CHECK(a.size() == b.size());
    const size_t count = std::min(a.size(), b.size());
    for (size_t i = 0; i < count; ++i) {
        CHECK(a[i] == B(b[i]) && B(a[i]) == b[i]);
    }
}

void
_BinaryCheckMatrix(const GfMatrix4d &m,
                   const rigExec::RigExecWireMatrix4d &w)
{
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            CHECK(m[r][c] == w[size_t(r * 4 + c)]);
        }
    }
}

void
_BinaryCheckFrame(const rigExec::RigExecPointFrame &f,
                  const rigExec::RigExecWireFrame &w)
{
    for (size_t i = 0; i < 4; ++i) {
        CHECK(f.points[i][0] == w.points[i][0]);
        CHECK(f.points[i][1] == w.points[i][1]);
        CHECK(f.points[i][2] == w.points[i][2]);
    }
    CHECK(f.flags == w.flags);
}

void
_BinaryCompareSlotMeta(const rigExec::RigExecBakedProgramImpl &program,
                       const rigExec::RigExecWireSlotMeta &meta,
                       const rigExec::RigExecBinaryReader &reader)
{
    CHECK(program.paths.size() == meta.paths.size());
    for (size_t i = 0; i < meta.paths.size(); ++i) {
        CHECK(program.paths[i].GetString() ==
              _BinaryString(reader, meta.paths[i]));
    }
    CHECK(program.slotKind.size() == meta.slotKind.size());
    for (size_t i = 0; i < meta.slotKind.size(); ++i) {
        CHECK(uint8_t(program.slotKind[i]) == uint8_t(meta.slotKind[i]));
    }
    _BinaryCheckEqual(program.parent, meta.parent);
    _BinaryCheckEqual(program.propParent, meta.propParent);
    _BinaryCheckEqual(program.xformSlots, meta.xformSlots);
    CHECK(program.xformPrimsBySlot.size() == meta.xformPaths.size());
    for (size_t i = 0; i < meta.xformPaths.size(); ++i) {
        CHECK(program.xformPrimsBySlot[i].GetPath().GetString() ==
              _BinaryString(reader, meta.xformPaths[i]));
    }
    _BinaryCheckEqual(program.jointSlots, meta.jointSlots);
    CHECK(program.jointPaths.size() == meta.jointPaths.size());
    for (size_t i = 0; i < meta.jointPaths.size(); ++i) {
        CHECK(program.jointPaths[i].GetString() ==
              _BinaryString(reader, meta.jointPaths[i]));
    }
    _BinaryCheckEqual(program.controlSlots, meta.controlSlots);
    CHECK(program.controlPaths.size() == meta.controlPaths.size());
    for (size_t i = 0; i < meta.controlPaths.size(); ++i) {
        CHECK(program.controlPaths[i].GetString() ==
              _BinaryString(reader, meta.controlPaths[i]));
    }
    CHECK(program.solverArrays.size() == meta.solverArrayPaths.size());
    CHECK(program.solverArrays.size() == meta.solverArrayElements.size());
    for (size_t i = 0; i < meta.solverArrayPaths.size(); ++i) {
        CHECK(program.solverArrays[i].first.GetString() ==
              _BinaryString(reader, meta.solverArrayPaths[i]));
        CHECK(program.solverArrays[i].second ==
              meta.solverArrayElements[i]);
    }
    _BinaryCheckEqual(program.jointPublishOrder, meta.jointPublishOrder);
    _BinaryCheckEqual(program.controlPublishOrder, meta.controlPublishOrder);
    _BinaryCheckEqual(program.solverPublishOrder, meta.solverPublishOrder);
    CHECK(program.jointPathsAscending == meta.jointPathsAscending);
    CHECK(program.controlPathsAscending == meta.controlPathsAscending);
    CHECK(program.solverArraysAscending == meta.solverArraysAscending);
    CHECK(program.needFinal.size() == meta.needFinal.size());
    for (size_t i = 0; i < meta.needFinal.size(); ++i) {
        CHECK((program.needFinal[i] != 0) == (meta.needFinal[i] != 0));
    }
    CHECK(program.needBase.size() == meta.needBase.size());
    for (size_t i = 0; i < meta.needBase.size(); ++i) {
        CHECK((program.needBase[i] != 0) == (meta.needBase[i] != 0));
    }
}

void
_BinaryCompareConstants(const rigExec::RigExecBakedProgramImpl &program,
                        const rigExec::RigExecWireConstants &constants,
                        const rigExec::RigExecBinaryReader &reader)
{
    CHECK(program.restM.size() == constants.restM.size());
    for (size_t i = 0; i < constants.restM.size(); ++i) {
        _BinaryCheckMatrix(program.restM[i], constants.restM[i]);
    }
    CHECK(program.restPts.size() == constants.restPts.size());
    for (size_t i = 0; i < constants.restPts.size(); ++i) {
        for (size_t p = 0; p < 4; ++p) {
            CHECK(program.restPts[i][p][0] ==
                  constants.restPts[i][p][0]);
            CHECK(program.restPts[i][p][1] ==
                  constants.restPts[i][p][1]);
            CHECK(program.restPts[i][p][2] ==
                  constants.restPts[i][p][2]);
        }
    }
    CHECK(program.restFrames.size() == constants.restFrames.size());
    for (size_t i = 0; i < constants.restFrames.size(); ++i) {
        _BinaryCheckFrame(program.restFrames[i], constants.restFrames[i]);
    }
    CHECK(program.selfD.size() == constants.selfD.size());
    for (size_t i = 0; i < constants.selfD.size(); ++i) {
        _BinaryCheckMatrix(program.selfD[i], constants.selfD[i]);
    }
    CHECK(program.parentDinv.size() == constants.parentDinv.size());
    for (size_t i = 0; i < constants.parentDinv.size(); ++i) {
        _BinaryCheckMatrix(program.parentDinv[i], constants.parentDinv[i]);
    }
    CHECK(program.rotOrder.size() == constants.rotOrder.size());
    for (size_t i = 0; i < constants.rotOrder.size(); ++i) {
        CHECK(program.rotOrder[i].GetString() ==
              _BinaryString(reader, constants.rotOrder[i]));
    }
    CHECK(program.restRoundTrip.size() == constants.restRoundTrip.size());
    for (size_t i = 0; i < constants.restRoundTrip.size(); ++i) {
        _BinaryCheckMatrix(program.restRoundTrip[i],
                           constants.restRoundTrip[i]);
    }
    CHECK(program.defaultRoundTrip.size() ==
          constants.defaultRoundTrip.size());
    for (size_t i = 0; i < constants.defaultRoundTrip.size(); ++i) {
        _BinaryCheckMatrix(program.defaultRoundTrip[i],
                           constants.defaultRoundTrip[i]);
    }
    CHECK(program.posedAuthored.size() == constants.posedAuthored.size());
    for (size_t i = 0; i < constants.posedAuthored.size(); ++i) {
        CHECK((program.posedAuthored[i] != 0) ==
              (constants.posedAuthored[i] != 0));
    }
    CHECK(program.posedAuthoredM.size() == constants.posedAuthoredM.size());
    for (size_t i = 0; i < constants.posedAuthoredM.size(); ++i) {
        _BinaryCheckMatrix(program.posedAuthoredM[i],
                           constants.posedAuthoredM[i]);
    }
    CHECK(program.noScaleAvars.size() == constants.noScaleAvars.size());
    for (size_t i = 0; i < constants.noScaleAvars.size(); ++i) {
        CHECK((program.noScaleAvars[i] != 0) ==
              (constants.noScaleAvars[i] != 0));
    }
    _BinaryCheckEqual(program.avarConstants, constants.avarConstants);
}

void
_BinaryCompareSteps(const rigExec::RigExecBakedProgramImpl &program,
                    const std::vector<rigExec::RigExecWireStep> &steps,
                    const rigExec::RigExecBinaryReader &reader)
{
    CHECK(program.steps.size() == steps.size());
    for (size_t i = 0; i < steps.size(); ++i) {
        const rigExec::RigExecBakedStep &live = program.steps[i];
        const rigExec::RigExecWireStep &wire = steps[i];
        CHECK(uint8_t(live.kind) == uint8_t(wire.kind));
        CHECK(live.object == wire.object);
        CHECK(live.part == wire.part);
        CHECK(live.reads.size() == wire.reads.size());
        for (size_t r = 0; r < wire.reads.size(); ++r) {
            CHECK(uint8_t(live.reads[r].domain) ==
                  uint8_t(wire.reads[r].domain));
            CHECK(live.reads[r].begin == wire.reads[r].begin);
            CHECK(live.reads[r].end == wire.reads[r].end);
        }
        CHECK(live.writes.size() == wire.writes.size());
        for (size_t w = 0; w < wire.writes.size(); ++w) {
            CHECK(uint8_t(live.writes[w].domain) ==
                  uint8_t(wire.writes[w].domain));
            CHECK(live.writes[w].begin == wire.writes[w].begin);
            CHECK(live.writes[w].end == wire.writes[w].end);
        }
        _BinaryCheckEqual(live.preds, wire.preds);
        _BinaryCheckEqual(live.succs, wire.succs);
        CHECK(live.isSource == wire.isSource);
        CHECK(live.externalReads == wire.externalReads);
        CHECK(live.varyingInputs == wire.varyingInputs);
        CHECK(live.resolvedInputReads == wire.resolvedInputReads);
        _BinaryCheckEqual(live.overrideInputs, wire.overrideInputs);
        CHECK(live.cluster == wire.cluster);
        CHECK(live.level == wire.level);
        CHECK(live.sizeUnits == wire.sizeUnits);
        CHECK(live.cost == wire.cost);
        CHECK(live.maxDiagnostics == size_t(wire.maxDiagnostics));
        CHECK(live.label == _BinaryString(reader, wire.label));
    }
}

void
_BinaryCompareClustering(
    const rigExec::RigExecBakedProgramImpl &program,
    const rigExec::RigExecWireClustering &clustering)
{
    CHECK(program.clustering.clusters.size() ==
          clustering.clusters.size());
    for (size_t i = 0; i < clustering.clusters.size(); ++i) {
        _BinaryCheckEqual(program.clustering.clusters[i].members,
                          clustering.clusters[i].members);
        _BinaryCheckEqual(program.clustering.clusters[i].preds,
                          clustering.clusters[i].preds);
        _BinaryCheckEqual(program.clustering.clusters[i].succs,
                          clustering.clusters[i].succs);
        CHECK(program.clustering.clusters[i].cost ==
              clustering.clusters[i].cost);
        CHECK(program.clustering.clusters[i].level ==
              clustering.clusters[i].level);
    }
    _BinaryCheckEqual(program.clustering.clusterOf, clustering.clusterOf);
    CHECK(program.clustering.grainUs == clustering.grainUs);
    CHECK(program.clustering.serialCost == clustering.serialCost);
    CHECK(program.clustering.criticalPathCost ==
          clustering.criticalPathCost);
}

void
_BinaryCompareClusterSet(const rigExec::RigExecBakedClusterSet &live,
                         const rigExec::RigExecWireClusterSet &wire,
                         size_t clusters)
{
    CHECK(size_t(wire.clusters) == clusters);
    CHECK(live.words == wire.words);
}

void
_BinaryCompareCones(const rigExec::RigExecBakedProgramImpl &program,
                    const rigExec::RigExecWireCones &cones)
{
    const size_t clusters = program.clustering.clusters.size();
    CHECK(program.cones.cone.size() == cones.cone.size());
    for (size_t i = 0; i < cones.cone.size(); ++i) {
        _BinaryCompareClusterSet(program.cones.cone[i], cones.cone[i],
                                 clusters);
    }
    _BinaryCompareClusterSet(program.cones.always, cones.always, clusters);
    _BinaryCompareClusterSet(program.cones.poseClusters, cones.poseClusters,
                             clusters);
    _BinaryCheckEqual(program.cones.avarCluster, cones.avarCluster);
    CHECK(program.cones.chainBaseClusters.size() ==
          cones.chainBaseClusters.size());
    for (size_t i = 0; i < cones.chainBaseClusters.size(); ++i) {
        _BinaryCheckEqual(program.cones.chainBaseClusters[i],
                          cones.chainBaseClusters[i]);
    }
    CHECK(program.cones.solverPointsClusters.size() ==
          cones.solverPointsClusters.size());
    for (size_t i = 0; i < cones.solverPointsClusters.size(); ++i) {
        _BinaryCheckEqual(program.cones.solverPointsClusters[i],
                          cones.solverPointsClusters[i]);
    }
    CHECK(program.cones.revisionClusters.size() ==
          cones.revisionClusters.size());
    for (size_t i = 0; i < cones.revisionClusters.size(); ++i) {
        _BinaryCheckEqual(program.cones.revisionClusters[i],
                          cones.revisionClusters[i]);
    }
    _BinaryCheckEqual(program.cones.revisionStaticCluster,
                      cones.revisionStaticCluster);
    CHECK(program.cones.nativeSourceClusters.size() ==
          cones.nativeSourceClusters.size());
    for (size_t i = 0; i < cones.nativeSourceClusters.size(); ++i) {
        _BinaryCheckEqual(program.cones.nativeSourceClusters[i],
                          cones.nativeSourceClusters[i]);
    }
    CHECK(program.cones.deltaBaseClusters.size() ==
          cones.deltaBaseClusters.size());
    for (size_t i = 0; i < cones.deltaBaseClusters.size(); ++i) {
        _BinaryCheckEqual(program.cones.deltaBaseClusters[i],
                          cones.deltaBaseClusters[i]);
    }
    CHECK(program.cones.constraintArrayClusters.size() ==
          cones.constraintArrayClusters.size());
    for (size_t i = 0; i < cones.constraintArrayClusters.size(); ++i) {
        _BinaryCheckEqual(program.cones.constraintArrayClusters[i],
                          cones.constraintArrayClusters[i]);
    }
    _BinaryCheckEqual(program.cones.varyingSteps, cones.varyingSteps);
    _BinaryCheckEqual(program.cones.overrideSteps, cones.overrideSteps);
}

/// Decodes every slice-2a section of \p bytes and compares the structs
/// against the standing program of \p evaluator -- the same program object
/// the bake serialized, so the correspondence is exact rather than
/// build-to-build.
void _BinaryCompareDomainPose(
    const rigExec::RigExecBakedProgramImpl &program,
    const rigExec::RigExecWireDomainPose &pose,
    const rigExec::RigExecBinaryReader &reader);
void _BinaryCompareDomainGeometry(
    const rigExec::RigExecBakedProgramImpl &program,
    const rigExec::RigExecWireDomainGeometry &geometry,
    const rigExec::RigExecBinaryReader &reader);

struct _BinaryOracleInput;
std::vector<_BinaryOracleInput> _BinaryCollectOracle(
    const rigExec::RigExecBakedProgramImpl &program);
void _BinaryCompareTableFrame(
    const rigExec::RigExecBakedProgramImpl &program,
    const rigExec::RigExecWireFrameInputs &frame,
    const std::vector<_BinaryOracleInput> &oracle,
    const rigExec::RigExecWireInputTable &table,
    const rigExec::RigExecBinaryReader &reader);
void _BinaryCompareTableStatic(
    const rigExec::RigExecBakedProgramImpl &program,
    const rigExec::RigExecWireInputTable &table,
    const std::vector<_BinaryOracleInput> &oracle,
    const rigExec::RigExecBinaryReader &reader);

void
_BinaryCompareProgram(rigExec::RigExecRigEvaluator &evaluator,
                      const std::vector<uint8_t> &bytes)
{
    const rigExec::RigExecBakedProgram *baked =
        evaluator.GetBakedProgram();
    CHECK(baked);
    if (!baked) {
        return;
    }
    const rigExec::RigExecBakedProgramImpl &program =
        baked->GetStepGraph();
    std::string error;
    std::unique_ptr<rigExec::RigExecBinaryReader> reader =
        rigExec::RigExecBinaryReader::Open(bytes.data(), bytes.size(),
                                           &error);
    CHECK(reader);
    if (!reader) {
        return;
    }
    const uint8_t *data = nullptr;
    size_t size = 0;
    CHECK(reader->FindSection(rigExec::RigExecBinarySection::SlotMeta,
                              &data, &size));
    {
        rigExec::RigExecWireReader cursor(data, size);
        rigExec::RigExecWireSlotMeta meta;
        CHECK(rigExec::RigExecWireDecodeSlotMeta(&cursor, &meta, &error));
        _BinaryCompareSlotMeta(program, meta, *reader);
    }
    CHECK(reader->FindSection(rigExec::RigExecBinarySection::Constants,
                              &data, &size));
    {
        rigExec::RigExecWireReader cursor(data, size);
        rigExec::RigExecWireConstants constants;
        CHECK(rigExec::RigExecWireDecodeConstants(&cursor, &constants,
                                                  &error));
        _BinaryCompareConstants(program, constants, *reader);
    }
    CHECK(reader->FindSection(rigExec::RigExecBinarySection::Steps,
                              &data, &size));
    {
        rigExec::RigExecWireReader cursor(data, size);
        std::vector<rigExec::RigExecWireStep> steps;
        CHECK(rigExec::RigExecWireDecodeSteps(&cursor, &steps, &error));
        _BinaryCompareSteps(program, steps, *reader);
    }
    CHECK(reader->FindSection(rigExec::RigExecBinarySection::Clusters,
                              &data, &size));
    {
        rigExec::RigExecWireReader cursor(data, size);
        rigExec::RigExecWireClustering clustering;
        CHECK(rigExec::RigExecWireDecodeClustering(&cursor, &clustering,
                                                   &error));
        _BinaryCompareClustering(program, clustering);
    }
    CHECK(reader->FindSection(rigExec::RigExecBinarySection::Cones,
                              &data, &size));
    {
        rigExec::RigExecWireReader cursor(data, size);
        rigExec::RigExecWireCones cones;
        CHECK(rigExec::RigExecWireDecodeCones(&cursor, &cones, &error));
        _BinaryCompareCones(program, cones);
    }
    CHECK(reader->FindSection(rigExec::RigExecBinarySection::DomainPose,
                              &data, &size));
    {
        rigExec::RigExecWireReader cursor(data, size);
        rigExec::RigExecWireDomainPose pose;
        CHECK(rigExec::RigExecWireDecodeDomainPose(&cursor, &pose, &error));
        _BinaryCompareDomainPose(program, pose, *reader);
    }
    CHECK(reader->FindSection(rigExec::RigExecBinarySection::DomainGeometry,
                              &data, &size));
    {
        rigExec::RigExecWireReader cursor(data, size);
        rigExec::RigExecWireDomainGeometry geometry;
        CHECK(rigExec::RigExecWireDecodeDomainGeometry(&cursor, &geometry,
                                                       &error));
        _BinaryCompareDomainGeometry(program, geometry, *reader);
    }
    CHECK(reader->FindSection(rigExec::RigExecBinarySection::InputTable,
                              &data, &size));
    {
        rigExec::RigExecWireReader cursor(data, size);
        rigExec::RigExecWireInputTable table;
        CHECK(rigExec::RigExecWireDecodeInputTable(&cursor, &table,
                                                   &error));
        const std::vector<_BinaryOracleInput> oracle =
            _BinaryCollectOracle(program);
        _BinaryCompareTableStatic(program, table, oracle, *reader);
        // The live program holds the last baked frame; only it can be
        // compared through the bytes, and the per-frame loop above
        // covers the rest against the capture directly.
        CHECK(!table.frames.empty());
        _BinaryCompareTableFrame(program, table.frames.back(), oracle,
                                 table, *reader);
    }
}

template <class T>
void
_BinaryCompareInputRoute(
    const rigExec::RigExecBakedInput<T> &live,
    const rigExec::RigExecWireInput &wire,
    const rigExec::RigExecBinaryReader &reader)
{
    CHECK(live.varying == wire.varying);
    CHECK((live.query.IsValid() || bool(live.resolvedAttr)) == wire.bound);
    CHECK(bool(live.resolvedAttr) == wire.viaResolved);
    CHECK(live.overrideIndex == wire.overrideIndex);
    if (live.head) {
        CHECK(live.head.GetPath().GetString() ==
              _BinaryString(reader, wire.head));
    } else {
        CHECK(wire.head == 0);
    }
}

void
_BinaryCompareInput(const rigExec::RigExecBakedInput<double> &live,
                    const rigExec::RigExecWireInput &wire,
                    const rigExec::RigExecBinaryReader &reader)
{
    CHECK(uint8_t(rigExec::RigExecWireInput::Tag::Double) ==
          uint8_t(wire.tag));
    CHECK(live.constant == wire.f64);
    _BinaryCompareInputRoute(live, wire, reader);
}

void
_BinaryCompareInput(const rigExec::RigExecBakedInput<float> &live,
                    const rigExec::RigExecWireInput &wire,
                    const rigExec::RigExecBinaryReader &reader)
{
    CHECK(uint8_t(rigExec::RigExecWireInput::Tag::Float) ==
          uint8_t(wire.tag));
    CHECK(live.constant == wire.f32);
    _BinaryCompareInputRoute(live, wire, reader);
}

void
_BinaryCompareInput(const rigExec::RigExecBakedInput<bool> &live,
                    const rigExec::RigExecWireInput &wire,
                    const rigExec::RigExecBinaryReader &reader)
{
    CHECK(uint8_t(rigExec::RigExecWireInput::Tag::Bool) ==
          uint8_t(wire.tag));
    CHECK(live.constant == wire.boolean);
    _BinaryCompareInputRoute(live, wire, reader);
}

void
_BinaryCompareInput(const rigExec::RigExecBakedInput<int> &live,
                    const rigExec::RigExecWireInput &wire,
                    const rigExec::RigExecBinaryReader &reader)
{
    CHECK(uint8_t(rigExec::RigExecWireInput::Tag::Int) ==
          uint8_t(wire.tag));
    CHECK(live.constant == wire.i32);
    _BinaryCompareInputRoute(live, wire, reader);
}

void
_BinaryCompareInput(const rigExec::RigExecBakedInput<GfMatrix4d> &live,
                    const rigExec::RigExecWireInput &wire,
                    const rigExec::RigExecBinaryReader &reader)
{
    CHECK(uint8_t(rigExec::RigExecWireInput::Tag::Matrix4d) ==
          uint8_t(wire.tag));
    _BinaryCheckMatrix(live.constant, wire.matrix);
    _BinaryCompareInputRoute(live, wire, reader);
}

void
_BinaryCompareInput(const rigExec::RigExecBakedInput<TfToken> &live,
                    const rigExec::RigExecWireInput &wire,
                    const rigExec::RigExecBinaryReader &reader)
{
    CHECK(uint8_t(rigExec::RigExecWireInput::Tag::Token) ==
          uint8_t(wire.tag));
    CHECK(live.constant.GetString() == _BinaryString(reader, wire.token));
    _BinaryCompareInputRoute(live, wire, reader);
}

void
_BinaryCompareInput(const rigExec::RigExecBakedInput<GfVec3d> &live,
                    const rigExec::RigExecWireInput &wire,
                    const rigExec::RigExecBinaryReader &reader)
{
    CHECK(uint8_t(rigExec::RigExecWireInput::Tag::Vec3d) ==
          uint8_t(wire.tag));
    CHECK(live.constant[0] == wire.vec[0]);
    CHECK(live.constant[1] == wire.vec[1]);
    CHECK(live.constant[2] == wire.vec[2]);
    _BinaryCompareInputRoute(live, wire, reader);
}

void
_BinaryCompareLandmarkSet(const std::array<GfVec3d, 4> &live,
                          const std::array<rigExec::RigExecWireVec3d, 4> &wire)
{
    for (size_t i = 0; i < 4; ++i) {
        CHECK(live[i][0] == wire[i][0]);
        CHECK(live[i][1] == wire[i][1]);
        CHECK(live[i][2] == wire[i][2]);
    }
}

void
_BinaryCompareRbf(const rigExec::RigExecRbfSolver &live,
                  const rigExec::RigExecWireRbf &wire)
{
    CHECK(live.GetPoses().size() == wire.poses.size());
    for (size_t i = 0; i < wire.poses.size(); ++i) {
        CHECK(live.GetPoses()[i][0] == wire.poses[i][0]);
        CHECK(live.GetPoses()[i][1] == wire.poses[i][1]);
        CHECK(live.GetPoses()[i][2] == wire.poses[i][2]);
    }
    CHECK(live.GetTranslations().size() == wire.translations.size());
    for (size_t i = 0; i < wire.translations.size(); ++i) {
        CHECK(live.GetTranslations()[i][0] == wire.translations[i][0]);
        CHECK(live.GetTranslations()[i][1] == wire.translations[i][1]);
        CHECK(live.GetTranslations()[i][2] == wire.translations[i][2]);
    }
    CHECK(live.GetPoseTypes().size() == wire.poseTypes.size());
    for (size_t i = 0; i < wire.poseTypes.size(); ++i) {
        CHECK(uint8_t(live.GetPoseTypes()[i]) == wire.poseTypes[i]);
    }
    CHECK(live.GetTwistAxis()[0] == wire.twistAxis[0]);
    CHECK(live.GetTwistAxis()[1] == wire.twistAxis[1]);
    CHECK(live.GetTwistAxis()[2] == wire.twistAxis[2]);
    CHECK(uint8_t(live.GetKernel()) == wire.kernel);
    CHECK(live.GetRadius() == wire.radius);
    CHECK(live.GetTranslationRadius() == wire.translationRadius);
    _BinaryCheckEqual(live.GetRadii(), wire.radii);
    _BinaryCheckEqual(live.GetTranslationRadii(), wire.translationRadii);
    CHECK(live.GetRegularization() == wire.regularization);
    CHECK(live.GetNormalize() == wire.normalize);
    CHECK(live.GetEnableRotation() == wire.enableRotation);
    CHECK(live.GetEnableTranslation() == wire.enableTranslation);
    CHECK(live.GetRegularizedSingular() == wire.regularizedSingular);
    CHECK(live.GetWeights() == wire.weights);
}

void
_BinaryCompareSolver(const rigExec::RigExecBakedProgramImpl::Solver &live,
                     const rigExec::RigExecWireSolver &wire,
                     const rigExec::RigExecBinaryReader &reader)
{
    CHECK(live.path.GetString() == _BinaryString(reader, wire.path));
    CHECK(live.type.GetString() == _BinaryString(reader, wire.type));
    _BinaryCheckEqual(live.restSlots, wire.restSlots);
    CHECK(live.restRefs.size() == wire.restRefs.size());
    for (size_t i = 0; i < wire.restRefs.size(); ++i) {
        CHECK(live.restRefs[i].first == wire.restRefs[i].first);
        CHECK(live.restRefs[i].second == wire.restRefs[i].second);
    }
    CHECK(live.restIsLive.size() == wire.restIsLive.size());
    for (size_t i = 0; i < wire.restIsLive.size(); ++i) {
        CHECK(bool(live.restIsLive[i]) == (wire.restIsLive[i] != 0));
    }
    _BinaryCheckEqual(live.restReads, wire.restReads);
    CHECK(live.hasLiveRest == wire.hasLiveRest);
    CHECK(live.jointRests.size() == wire.jointRests.size());
    for (size_t i = 0; i < wire.jointRests.size(); ++i) {
        _BinaryCompareLandmarkSet(live.jointRests[i], wire.jointRests[i]);
    }
    CHECK(live.restsVary == wire.restsVary);
    _BinaryCheckEqual(live.restOverrides, wire.restOverrides);
    _BinaryCheckEqual(live.splineRestWeights, wire.splineRestWeights);
    CHECK(uint8_t(live.splineRestMode) == wire.splineRestMode);
    CHECK(live.degenerate == wire.degenerate);
    _BinaryCheckEqual(live.controls, wire.controls);
    CHECK(live.parentRelative == wire.parentRelative);
    CHECK(live.controlRests.size() == wire.controlRests.size());
    for (size_t i = 0; i < wire.controlRests.size(); ++i) {
        _BinaryCompareLandmarkSet(live.controlRests[i],
                                  wire.controlRests[i]);
    }
    CHECK(live.root == wire.root);
    CHECK(live.mid == wire.mid);
    CHECK(live.end == wire.end);
    CHECK(live.pole == wire.pole);
    for (size_t k = 0; k < 3; ++k) {
        _BinaryCompareLandmarkSet(live.ikRests[k], wire.ikRests[k]);
    }
    CHECK(live.ikParams.upperLength == wire.ikParams.upperLength);
    CHECK(live.ikParams.lowerLength == wire.ikParams.lowerLength);
    CHECK(live.ikParams.stretch == wire.ikParams.stretch);
    CHECK(live.ikParams.softness == wire.ikParams.softness);
    CHECK(live.ikParams.preferredBendRadians ==
          wire.ikParams.preferredBendRadians);
    _BinaryCompareInput(live.bend, wire.bend, reader);
    _BinaryCompareInput(live.upperOffset, wire.upperOffset, reader);
    _BinaryCompareInput(live.lowerOffset, wire.lowerOffset, reader);
    _BinaryCompareInput(live.stretch, wire.stretch, reader);
    _BinaryCompareInput(live.softness, wire.softness, reader);
    CHECK(live.upperLengthBase == wire.upperLengthBase);
    CHECK(live.lowerLengthBase == wire.lowerLengthBase);
    CHECK(live.inA == wire.inA);
    CHECK(live.inB == wire.inB);
    _BinaryCompareInput(live.blendWeight, wire.blendWeight, reader);
    CHECK(uint8_t(live.scaleMode) == wire.scaleMode);
    CHECK(live.blendRotationRejected == wire.blendRotationRejected);
    _BinaryCompareLandmarkSet(live.splineRest.cvs, wire.splineRest.cvs);
    _BinaryCheckFrame(live.splineRest.rootControl,
                      wire.splineRest.rootControl);
    _BinaryCheckFrame(live.splineRest.midControl,
                      wire.splineRest.midControl);
    _BinaryCheckFrame(live.splineRest.endControl,
                      wire.splineRest.endControl);
    CHECK(live.splineRest.joints.size() == wire.splineRest.joints.size());
    for (size_t i = 0; i < wire.splineRest.joints.size(); ++i) {
        _BinaryCheckFrame(live.splineRest.joints[i],
                          wire.splineRest.joints[i]);
    }
    _BinaryCheckEqual(live.splineRest.segmentLengths,
                      wire.splineRest.segmentLengths);
    CHECK(live.splineRest.restArcLength == wire.splineRest.restArcLength);
    _BinaryCheckEqual(live.splineRest.volumeWeights,
                      wire.splineRest.volumeWeights);
    CHECK(live.splineParams.preserveVolume ==
          wire.splineParams.preserveVolume);
    CHECK(live.splineParams.midFollowWeight ==
          wire.splineParams.midFollowWeight);
    CHECK(live.splineParams.roll == wire.splineParams.roll);
    CHECK(live.splineParams.twist == wire.splineParams.twist);
    CHECK(live.splineParams.minLengthRatio ==
          wire.splineParams.minLengthRatio);
    CHECK(live.splineParams.aimRootTangent ==
          wire.splineParams.aimRootTangent);
    CHECK(live.splineJointRests.size() == wire.splineJointRests.size());
    for (size_t i = 0; i < wire.splineJointRests.size(); ++i) {
        _BinaryCompareLandmarkSet(live.splineJointRests[i],
                                  wire.splineJointRests[i]);
    }
    CHECK(uint64_t(live.splineCount) == wire.splineCount);
    _BinaryCompareInput(live.preserveVolume, wire.preserveVolume, reader);
    _BinaryCompareInput(live.midFollowWeight, wire.midFollowWeight,
                        reader);
    _BinaryCompareInput(live.roll, wire.roll, reader);
    _BinaryCompareInput(live.twist, wire.twist, reader);
    _BinaryCompareInput(live.minLengthRatio, wire.minLengthRatio, reader);
    CHECK(live.splineParamsVary == wire.splineParamsVary);
    _BinaryCompareLandmarkSet(live.twistStartRest, wire.twistStartRest);
    _BinaryCompareLandmarkSet(live.twistEndRest, wire.twistEndRest);
    _BinaryCheckEqual(live.twistWeights, wire.twistWeights);
    _BinaryCompareInput(live.twistTurns, wire.twistTurns, reader);
    CHECK(live.ribbonPointsPath.GetString() ==
          _BinaryString(reader, wire.ribbonPointsPath));
    CHECK(live.ribbonRestPoints.size() == wire.ribbonRestPoints.size());
    for (size_t i = 0; i < wire.ribbonRestPoints.size(); ++i) {
        CHECK(live.ribbonRestPoints[i][0] == wire.ribbonRestPoints[i][0]);
        CHECK(live.ribbonRestPoints[i][1] == wire.ribbonRestPoints[i][1]);
        CHECK(live.ribbonRestPoints[i][2] == wire.ribbonRestPoints[i][2]);
    }
    CHECK(live.ribbonConstantPoints.size() ==
          wire.ribbonConstantPoints.size());
    for (size_t i = 0; i < wire.ribbonConstantPoints.size(); ++i) {
        CHECK(live.ribbonConstantPoints[i][0] ==
              wire.ribbonConstantPoints[i][0]);
        CHECK(live.ribbonConstantPoints[i][1] ==
              wire.ribbonConstantPoints[i][1]);
        CHECK(live.ribbonConstantPoints[i][2] ==
              wire.ribbonConstantPoints[i][2]);
    }
    CHECK(live.ribbonPointsVarying == wire.ribbonPointsVarying);
    _BinaryCompareInput(live.ribbonSampleCount, wire.ribbonSampleCount,
                        reader);
    CHECK(live.outputs.size() == wire.outputs.size());
    for (size_t i = 0; i < wire.outputs.size(); ++i) {
        CHECK(live.outputs[i].first == wire.outputs[i].first);
        CHECK(live.outputs[i].second == wire.outputs[i].second);
    }
    _BinaryCheckEqual(live.outPosition, wire.outPosition);
    _BinaryCheckEqual(live.controlReads, wire.controlReads);
    CHECK(live.rootRead == wire.rootRead);
    CHECK(live.midRead == wire.midRead);
    CHECK(live.endRead == wire.endRead);
    CHECK(live.poleRead == wire.poleRead);
}

void
_BinaryCompareConstraint(
    const rigExec::RigExecBakedProgramImpl::Constraint &live,
    const rigExec::RigExecWireConstraint &wire,
    const rigExec::RigExecBinaryReader &reader)
{
    CHECK(live.path.GetString() == _BinaryString(reader, wire.path));
    CHECK(live.type.GetString() == _BinaryString(reader, wire.type));
    CHECK(live.weightObject.GetString() ==
          _BinaryString(reader, wire.weightObject));
    CHECK(live.target == wire.target);
    _BinaryCheckEqual(live.targetSlots, wire.targetSlots);
    CHECK(live.snapshotTargets.size() == wire.snapshotTargets.size());
    for (size_t i = 0; i < wire.snapshotTargets.size(); ++i) {
        CHECK((live.snapshotTargets[i] != 0) ==
              (wire.snapshotTargets[i] != 0));
    }
    _BinaryCheckEqual(live.sources, wire.sources);
    _BinaryCheckEqual(live.sourceNatives, wire.sourceNatives);
    CHECK(live.sourcePaths.size() == wire.sourcePaths.size());
    for (size_t i = 0; i < wire.sourcePaths.size(); ++i) {
        CHECK(live.sourcePaths[i].GetString() ==
              _BinaryString(reader, wire.sourcePaths[i]));
    }
    CHECK(live.arrays == wire.arrays);
    _BinaryCompareInput(live.enabled, wire.enabled, reader);
    _BinaryCompareInput(live.defaultWeight, wire.defaultWeight, reader);
    _BinaryCompareInput(live.offset, wire.offset, reader);
    _BinaryCompareInput(live.affectX, wire.affectX, reader);
    _BinaryCompareInput(live.affectY, wire.affectY, reader);
    _BinaryCompareInput(live.affectZ, wire.affectZ, reader);
    _BinaryCompareInput(live.tX, wire.tX, reader);
    _BinaryCompareInput(live.tY, wire.tY, reader);
    _BinaryCompareInput(live.tZ, wire.tZ, reader);
    _BinaryCompareInput(live.rX, wire.rX, reader);
    _BinaryCompareInput(live.rY, wire.rY, reader);
    _BinaryCompareInput(live.rZ, wire.rZ, reader);
    _BinaryCompareInput(live.sX, wire.sX, reader);
    _BinaryCompareInput(live.sY, wire.sY, reader);
    _BinaryCompareInput(live.sZ, wire.sZ, reader);
    CHECK(uint8_t(live.order) == wire.order);
    _BinaryCompareInput(live.aimVector, wire.aimVector, reader);
    _BinaryCompareInput(live.upVector, wire.upVector, reader);
    _BinaryCompareInput(live.rotationOffset, wire.rotationOffset, reader);
    _BinaryCompareInput(live.worldUpVector, wire.worldUpVector, reader);
    CHECK(live.aimAxisFallback[0] == wire.aimAxisFallback[0]);
    CHECK(live.aimAxisFallback[1] == wire.aimAxisFallback[1]);
    CHECK(live.aimAxisFallback[2] == wire.aimAxisFallback[2]);
    CHECK(live.aimVectorAuthored == wire.aimVectorAuthored);
    CHECK(live.preserveInputUp == wire.preserveInputUp);
    CHECK(live.worldUpType.GetString() ==
          _BinaryString(reader, wire.worldUpType));
    CHECK(live.sceneUp[0] == wire.sceneUp[0]);
    CHECK(live.sceneUp[1] == wire.sceneUp[1]);
    CHECK(live.sceneUp[2] == wire.sceneUp[2]);
    CHECK(live.pointsTarget.GetString() ==
          _BinaryString(reader, wire.pointsTarget));
    CHECK(live.deltaBasePath.GetString() ==
          _BinaryString(reader, wire.deltaBasePath));
    CHECK(live.deltaBase == wire.deltaBase);
    CHECK(live.worldUpObject == wire.worldUpObject);
    CHECK(live.worldUpNative == wire.worldUpNative);
    CHECK(live.worldUpPath.GetString() ==
          _BinaryString(reader, wire.worldUpPath));
    CHECK(live.worldUpObjectNamed == wire.worldUpObjectNamed);
    CHECK(live.snapshotAfter == wire.snapshotAfter);
    CHECK(live.singleChainIk == wire.singleChainIk);
    CHECK(uint8_t(live.ikMode) == wire.ikMode);
    CHECK(live.poleModeObject == wire.poleModeObject);
    CHECK(live.useAnimatedTs == wire.useAnimatedTs);
    CHECK(live.ikRestLive.size() == wire.ikRestLive.size());
    for (size_t i = 0; i < wire.ikRestLive.size(); ++i) {
        CHECK((live.ikRestLive[i] != 0) == (wire.ikRestLive[i] != 0));
    }
    CHECK(live.effector == wire.effector);
    CHECK(live.effectorNative == wire.effectorNative);
    CHECK(live.effectorPath.GetString() ==
          _BinaryString(reader, wire.effectorPath));
    _BinaryCheckEqual(live.poleObjects, wire.poleObjects);
    _BinaryCheckEqual(live.poleObjectNatives, wire.poleObjectNatives);
    _BinaryCompareInput(live.poleVector, wire.poleVector, reader);
    _BinaryCompareInput(live.twistDegrees, wire.twistDegrees, reader);
}

void
_BinaryCompareAncestorRead(
    const rigExec::RigExecBakedCommit::AncestorRead
        &live,
    const rigExec::RigExecWireAncestorRead &wire)
{
    CHECK(live.slot == wire.slot);
    CHECK(live.fin == wire.fin);
    CHECK(live.base == wire.base);
}

void
_BinaryCompareCommit(const rigExec::RigExecBakedCommit &live,
                     const rigExec::RigExecWireCommit &wire,
                     const rigExec::RigExecBinaryReader &reader)
{
    CHECK(live.moverPath.GetString() ==
          _BinaryString(reader, wire.moverPath));
    CHECK(live.solverOutput == wire.solverOutput);
    _BinaryCheckEqual(live.slots, wire.slots);
    CHECK(live.propagate.size() == wire.propagate.size());
    for (size_t i = 0; i < wire.propagate.size(); ++i) {
        CHECK(live.propagate[i].first == wire.propagate[i].first);
        CHECK(live.propagate[i].second == wire.propagate[i].second);
    }
    _BinaryCheckEqual(live.closestPos, wire.closestPos);
    CHECK(live.split == wire.split);
    CHECK(live.stagingBase == wire.stagingBase);
    CHECK(live.sources.size() == wire.sources.size());
    for (size_t i = 0; i < wire.sources.size(); ++i) {
        _BinaryCheckFrame(live.sources[i].frame, wire.sources[i].frame);
        CHECK(live.sources[i].normalizedWeight ==
              wire.sources[i].normalizedWeight);
        CHECK(live.sources[i].translationOffset[0] ==
              wire.sources[i].translationOffset[0]);
        CHECK(live.sources[i].translationOffset[1] ==
              wire.sources[i].translationOffset[1]);
        CHECK(live.sources[i].translationOffset[2] ==
              wire.sources[i].translationOffset[2]);
        CHECK(live.sources[i].rotationOffsetDegrees[0] ==
              wire.sources[i].rotationOffsetDegrees[0]);
        CHECK(live.sources[i].rotationOffsetDegrees[1] ==
              wire.sources[i].rotationOffsetDegrees[1]);
        CHECK(live.sources[i].rotationOffsetDegrees[2] ==
              wire.sources[i].rotationOffsetDegrees[2]);
    }
    _BinaryCheckEqual(live.slotReads, wire.slotReads);
    _BinaryCheckEqual(live.slotWrites, wire.slotWrites);
    _BinaryCheckEqual(live.slotBaseWrites, wire.slotBaseWrites);
    _BinaryCheckEqual(live.descendantReads, wire.descendantReads);
    _BinaryCheckEqual(live.closestReads, wire.closestReads);
    _BinaryCheckEqual(live.descendantWrites, wire.descendantWrites);
    _BinaryCheckEqual(live.descendantBaseWrites, wire.descendantBaseWrites);
    _BinaryCheckEqual(live.slotCarry, wire.slotCarry);
    _BinaryCheckEqual(live.slotBaseCarry, wire.slotBaseCarry);
    _BinaryCheckEqual(live.descendantCarry, wire.descendantCarry);
    _BinaryCheckEqual(live.descendantBaseCarry, wire.descendantBaseCarry);
    _BinaryCheckEqual(live.sourceReads, wire.sourceReads);
    CHECK(live.worldUpRead == wire.worldUpRead);
    CHECK(live.targetRead == wire.targetRead);
    _BinaryCheckEqual(live.targetReads, wire.targetReads);
    CHECK(live.effectorRead == wire.effectorRead);
    CHECK(live.effectorAncestors.size() == wire.effectorAncestors.size());
    for (size_t i = 0; i < wire.effectorAncestors.size(); ++i) {
        _BinaryCompareAncestorRead(live.effectorAncestors[i],
                                   wire.effectorAncestors[i]);
    }
    _BinaryCheckEqual(live.poleReads, wire.poleReads);
    CHECK(live.poleAncestors.size() == wire.poleAncestors.size());
    for (size_t i = 0; i < wire.poleAncestors.size(); ++i) {
        CHECK(live.poleAncestors[i].size() ==
              wire.poleAncestors[i].size());
        for (size_t k = 0; k < wire.poleAncestors[i].size(); ++k) {
            _BinaryCompareAncestorRead(live.poleAncestors[i][k],
                                       wire.poleAncestors[i][k]);
        }
    }
    CHECK(live.recordAfter == wire.recordAfter);
    CHECK(live.recordEveryTarget == wire.recordEveryTarget);
    CHECK(live.sourceAncestors.size() == wire.sourceAncestors.size());
    for (size_t i = 0; i < wire.sourceAncestors.size(); ++i) {
        CHECK(live.sourceAncestors[i].size() ==
              wire.sourceAncestors[i].size());
        for (size_t k = 0; k < wire.sourceAncestors[i].size(); ++k) {
            _BinaryCompareAncestorRead(live.sourceAncestors[i][k],
                                       wire.sourceAncestors[i][k]);
        }
    }
    CHECK(live.worldUpAncestors.size() == wire.worldUpAncestors.size());
    for (size_t i = 0; i < wire.worldUpAncestors.size(); ++i) {
        _BinaryCompareAncestorRead(live.worldUpAncestors[i],
                                   wire.worldUpAncestors[i]);
    }
}

void
_BinaryCompareDomainPose(const rigExec::RigExecBakedProgramImpl &program,
                         const rigExec::RigExecWireDomainPose &pose,
                         const rigExec::RigExecBinaryReader &reader)
{
    CHECK(program.ladders.size() == pose.ladders.size());
    for (size_t i = 0; i < pose.ladders.size(); ++i) {
        _BinaryCompareInput(program.ladders[i].restSpace,
                            pose.ladders[i].restSpace, reader);
        _BinaryCompareInput(program.ladders[i].defaultSpace,
                            pose.ladders[i].defaultSpace, reader);
        _BinaryCompareInput(program.ladders[i].posedSpace,
                            pose.ladders[i].posedSpace, reader);
        for (size_t a = 0; a < 6; ++a) {
            _BinaryCompareInput(program.ladders[i].restAvars[a],
                                pose.ladders[i].restAvars[a], reader);
            _BinaryCompareInput(program.ladders[i].defaultAvars[a],
                                pose.ladders[i].defaultAvars[a], reader);
        }
        _BinaryCompareInput(program.ladders[i].rotationOrder,
                            pose.ladders[i].rotationOrder, reader);
    }
    CHECK(program.ladderVarying == pose.ladderVarying);
    _BinaryCheckEqual(program.ladderOverrides, pose.ladderOverrides);
    CHECK(program.restChainVaries.size() == pose.restChainVaries.size());
    for (size_t i = 0; i < pose.restChainVaries.size(); ++i) {
        CHECK((program.restChainVaries[i] != 0) ==
              (pose.restChainVaries[i] != 0));
    }
    CHECK(program.poseInterpolators.size() ==
          pose.poseInterpolators.size());
    for (size_t i = 0; i < pose.poseInterpolators.size(); ++i) {
        CHECK(program.poseInterpolators[i].path.GetString() ==
              _BinaryString(reader, pose.poseInterpolators[i].path));
        CHECK(program.poseInterpolators[i].driverSlot ==
              pose.poseInterpolators[i].driverSlot);
        CHECK(program.poseInterpolators[i].parentSlot ==
              pose.poseInterpolators[i].parentSlot);
        CHECK(program.poseInterpolators[i].allowNegativeWeights ==
              pose.poseInterpolators[i].allowNegativeWeights);
        _BinaryCompareInput(program.poseInterpolators[i].enabled,
                            pose.poseInterpolators[i].enabled, reader);
        CHECK(program.poseInterpolators[i].weightBegin ==
              pose.poseInterpolators[i].weightBegin);
        CHECK(program.poseInterpolators[i].weightEnd ==
              pose.poseInterpolators[i].weightEnd);
        _BinaryCheckEqual(program.poseInterpolators[i].poseSlots,
                          pose.poseInterpolators[i].poseSlots);
        _BinaryCheckEqual(program.poseInterpolators[i].disabledSlots,
                          pose.poseInterpolators[i].disabledSlots);
        _BinaryCompareRbf(program.poseInterpolators[i].solver,
                          pose.poseInterpolators[i].solver);
    }
    CHECK(program.poseWeightPaths.size() == pose.poseWeightPaths.size());
    for (size_t i = 0; i < pose.poseWeightPaths.size(); ++i) {
        CHECK(program.poseWeightPaths[i].GetString() ==
              _BinaryString(reader, pose.poseWeightPaths[i]));
    }
    CHECK(program.solvers.size() == pose.solvers.size());
    for (size_t i = 0; i < pose.solvers.size(); ++i) {
        _BinaryCompareSolver(program.solvers[i], pose.solvers[i], reader);
    }
    _BinaryCheckEqual(program.guideSolvers, pose.guideSolvers);
    CHECK(program.constraints.size() == pose.constraints.size());
    for (size_t i = 0; i < pose.constraints.size(); ++i) {
        _BinaryCompareConstraint(program.constraints[i],
                                 pose.constraints[i], reader);
    }
    CHECK(program.constraintArrays.size() == pose.constraintArrays.size());
    for (size_t i = 0; i < pose.constraintArrays.size(); ++i) {
        if (program.constraintArrays[i].prim) {
            CHECK(program.constraintArrays[i].prim.GetPath().GetString() ==
                  _BinaryString(reader, pose.constraintArrays[i].prim));
        } else {
            CHECK(pose.constraintArrays[i].prim == 0);
        }
        CHECK(uint64_t(program.constraintArrays[i].sourceCount) ==
              pose.constraintArrays[i].sourceCount);
        CHECK(program.constraintArrays[i].parentOffsets ==
              pose.constraintArrays[i].parentOffsets);
        CHECK(program.constraintArrays[i].readPole ==
              pose.constraintArrays[i].readPole);
        CHECK(uint64_t(program.constraintArrays[i].poleCount) ==
              pose.constraintArrays[i].poleCount);
    }
    CHECK(program.nativeSources.size() == pose.nativeSources.size());
    for (size_t i = 0; i < pose.nativeSources.size(); ++i) {
        CHECK(program.nativeSources[i].path.GetString() ==
              _BinaryString(reader, pose.nativeSources[i].path));
        _BinaryCheckEqual(program.nativeSources[i].ancestorSlots,
                          pose.nativeSources[i].ancestorSlots);
    }
    CHECK(program.walkSteps.size() == pose.walkSteps.size());
    for (size_t i = 0; i < pose.walkSteps.size(); ++i) {
        CHECK(program.walkSteps[i].solverBatch ==
              pose.walkSteps[i].solverBatch);
        CHECK(uint64_t(program.walkSteps[i].level) ==
              pose.walkSteps[i].level);
        CHECK(program.walkSteps[i].index == pose.walkSteps[i].index);
        _BinaryCheckEqual(program.walkSteps[i].batchSolvers,
                          pose.walkSteps[i].batchSolvers);
        CHECK(program.walkSteps[i].propagate.size() ==
              pose.walkSteps[i].propagate.size());
        for (size_t k = 0; k < pose.walkSteps[i].propagate.size(); ++k) {
            CHECK(program.walkSteps[i].propagate[k].first ==
                  pose.walkSteps[i].propagate[k].first);
            CHECK(program.walkSteps[i].propagate[k].second ==
                  pose.walkSteps[i].propagate[k].second);
        }
    }
    CHECK(program.composeGroups.size() == pose.composeGroups.size());
    for (size_t i = 0; i < pose.composeGroups.size(); ++i) {
        CHECK(program.composeGroups[i].begin ==
              pose.composeGroups[i].begin);
        CHECK(program.composeGroups[i].end == pose.composeGroups[i].end);
        _BinaryCheckEqual(program.composeGroups[i].parentSlots,
                          pose.composeGroups[i].parentSlots);
    }
    CHECK(program.commits.size() == pose.commits.size());
    for (size_t i = 0; i < pose.commits.size(); ++i) {
        _BinaryCompareCommit(program.commits[i], pose.commits[i], reader);
    }
    if (program.jointSolverBinding) {
        CHECK(program.jointSolverBinding->size() ==
              pose.jointBindingJoints.size());
        size_t i = 0;
        for (const auto &entry : *program.jointSolverBinding) {
            CHECK(entry.first.GetString() ==
                  _BinaryString(reader, pose.jointBindingJoints[i]));
            CHECK(entry.second.size() ==
                  pose.jointBindingSolvers[i].size());
            CHECK(entry.second.size() ==
                  pose.jointBindingElements[i].size());
            for (size_t k = 0; k < entry.second.size(); ++k) {
                CHECK(entry.second[k].first.GetString() ==
                      _BinaryString(reader,
                                    pose.jointBindingSolvers[i][k]));
                CHECK(entry.second[k].second ==
                      pose.jointBindingElements[i][k]);
            }
            ++i;
        }
    } else {
        CHECK(pose.jointBindingJoints.empty());
    }
    CHECK(program.hasPropertyChains == pose.hasPropertyChains);
    CHECK(program.phasedReads == pose.phasedReads);
    CHECK(program.publishWeightFields == pose.publishWeightFields);
}

void
_BinaryComparePhase(const rigExec::RigExecReadPhase &live,
                    const rigExec::RigExecWireReadPhase &wire,
                    const rigExec::RigExecBinaryReader &reader)
{
    CHECK(uint8_t(live.kind) == wire.kind);
    CHECK(live.prim.GetString() == _BinaryString(reader, wire.prim));
}

void
_BinaryCompareBinding(const rigExec::RigExecRevisionBinding &live,
                      const rigExec::RigExecWireRevisionBinding &wire,
                      const rigExec::RigExecBinaryReader &reader)
{
    CHECK(live.moverPath.GetString() ==
          _BinaryString(reader, wire.moverPath));
    CHECK(live.target.GetString() == _BinaryString(reader, wire.target));
    CHECK(live.transform.GetString() ==
          _BinaryString(reader, wire.transform));
    CHECK(live.transformSpace.GetString() ==
          _BinaryString(reader, wire.transformSpace));
    CHECK(live.influences.size() == wire.influences.size());
    for (size_t i = 0; i < wire.influences.size(); ++i) {
        CHECK(live.influences[i].GetString() ==
              _BinaryString(reader, wire.influences[i]));
    }
    CHECK(live.weightObject.GetString() ==
          _BinaryString(reader, wire.weightObject));
    CHECK(live.base.GetString() == _BinaryString(reader, wire.base));
    CHECK(live.topologyCounts.GetString() ==
          _BinaryString(reader, wire.topologyCounts));
    CHECK(live.topologyIndices.GetString() ==
          _BinaryString(reader, wire.topologyIndices));
    CHECK(live.cagePoints.GetString() ==
          _BinaryString(reader, wire.cagePoints));
    CHECK(live.surfacePoints.GetString() ==
          _BinaryString(reader, wire.surfacePoints));
    CHECK(live.bindCoords.GetString() ==
          _BinaryString(reader, wire.bindCoords));
    CHECK(live.driverCurvePoints.GetString() ==
          _BinaryString(reader, wire.driverCurvePoints));
    CHECK(live.driverCurveOrder.GetString() ==
          _BinaryString(reader, wire.driverCurveOrder));
    CHECK(live.driverCurveKnots.GetString() ==
          _BinaryString(reader, wire.driverCurveKnots));
    CHECK(live.driverTransformCount == wire.driverTransformCount);
    CHECK(live.driverSpaceCount == wire.driverSpaceCount);
    CHECK(live.driverBaseTransformCount == wire.driverBaseTransformCount);
    CHECK(live.driverFrames.GetString() ==
          _BinaryString(reader, wire.driverFrames));
    CHECK(live.widths.GetString() == _BinaryString(reader, wire.widths));
    CHECK(live.curvenet.GetString() ==
          _BinaryString(reader, wire.curvenet));
    CHECK(live.curvenetPoints.GetString() ==
          _BinaryString(reader, wire.curvenetPoints));
    CHECK(live.blendInputs.size() == wire.blendInputs.size());
    for (size_t i = 0; i < wire.blendInputs.size(); ++i) {
        CHECK(live.blendInputs[i].GetString() ==
              _BinaryString(reader, wire.blendInputs[i]));
    }
    CHECK(live.blendSamples.size() == wire.blendSampleInputs.size());
    CHECK(live.blendSamples.size() == wire.blendSamples.size());
    size_t row = 0;
    for (const auto &entry : live.blendSamples) {
        CHECK(entry.first.GetString() ==
              _BinaryString(reader, wire.blendSampleInputs[row]));
        CHECK(entry.second.size() == wire.blendSamples[row].size());
        for (size_t s = 0; s < wire.blendSamples[row].size(); ++s) {
            CHECK(entry.second[s].sample.GetString() ==
                  _BinaryString(reader, wire.blendSamples[row][s].sample));
            CHECK(entry.second[s].points.GetString() ==
                  _BinaryString(reader, wire.blendSamples[row][s].points));
            _BinaryComparePhase(entry.second[s].phase,
                                wire.blendSamples[row][s].phase, reader);
            CHECK(entry.second[s].blendShape.GetString() ==
                  _BinaryString(reader,
                                wire.blendSamples[row][s].blendShape));
        }
        ++row;
    }
    CHECK(live.phases.size() == wire.phaseInputs.size());
    CHECK(live.phases.size() == wire.phases.size());
    row = 0;
    for (const auto &entry : live.phases) {
        CHECK(entry.first.GetString() ==
              _BinaryString(reader, wire.phaseInputs[row]));
        _BinaryComparePhase(entry.second, wire.phases[row], reader);
        ++row;
    }
    _BinaryComparePhase(live.transformPhase, wire.transformPhase, reader);
}

void
_BinaryCompareTopology(
    const std::shared_ptr<const rigExec::RigExecSkinTopology> &live,
    const rigExec::RigExecWireSkinTopology &wire)
{
    CHECK(bool(live) == wire.hasTopology);
    if (!live) {
        return;
    }
    _BinaryCheckEqual(live->indices, wire.indices);
    _BinaryCheckEqual(live->weights, wire.weights);
    CHECK(live->elementSize == wire.elementSize);
    CHECK(uint64_t(live->pointCount) == wire.pointCount);
    CHECK(uint64_t(live->influenceCount) == wire.influenceCount);
    CHECK(live->validated == wire.validated);
}

void
_BinaryCompareAttributes(const std::vector<UsdAttribute> &live,
                         const std::vector<uint32_t> &paths,
                         const std::vector<uint8_t> &valid,
                         const rigExec::RigExecBinaryReader &reader)
{
    CHECK(live.size() == paths.size());
    CHECK(live.size() == valid.size());
    for (size_t i = 0; i < paths.size(); ++i) {
        CHECK(bool(live[i]) == (valid[i] != 0));
        if (live[i]) {
            CHECK(live[i].GetPath().GetString() ==
                  _BinaryString(reader, paths[i]));
        } else {
            CHECK(paths[i] == 0);
        }
    }
}

void
_BinaryCompareRevision(
    const rigExec::RigExecBakedProgramImpl::GeomRevision &live,
    const rigExec::RigExecWireRevision &wire,
    const rigExec::RigExecBinaryReader &reader)
{
    CHECK(live.moverPath.GetString() ==
          _BinaryString(reader, wire.moverPath));
    CHECK(live.target.GetString() == _BinaryString(reader, wire.target));
    if (live.moverPrim) {
        CHECK(live.moverPrim.GetPath().GetString() ==
              _BinaryString(reader, wire.moverPrim));
    } else {
        CHECK(wire.moverPrim == 0);
    }
    CHECK(uint8_t(live.op) == wire.op);
    _BinaryCompareBinding(live.binding, wire.binding, reader);
    CHECK(live.curvenetChain == wire.curvenetChain);
    CHECK(live.curvenetBindResolved == wire.curvenetBindResolved);
    CHECK(bool(live.curvenetBind) == wire.hasCurvenetBind);
    CHECK(live.curvenetBindInputs.restNet.size() ==
          wire.curvenetRestNet.size());
    for (size_t i = 0; i < wire.curvenetRestNet.size(); ++i) {
        CHECK(live.curvenetBindInputs.restNet[i][0] ==
              wire.curvenetRestNet[i][0]);
        CHECK(live.curvenetBindInputs.restNet[i][1] ==
              wire.curvenetRestNet[i][1]);
        CHECK(live.curvenetBindInputs.restNet[i][2] ==
              wire.curvenetRestNet[i][2]);
    }
    _BinaryCheckEqual(live.curvenetBindInputs.splineIndices,
                      wire.curvenetSplineIndices);
    CHECK(live.curvenetBindInputs.samplesPerSpline ==
          wire.curvenetSamplesPerSpline);
    CHECK(live.curvenetBindInputs.basis.GetString() ==
          _BinaryString(reader, wire.curvenetBasis));
    CHECK(live.curvenetBindInputs.meshPoints.size() ==
          wire.curvenetMeshPoints.size());
    for (size_t i = 0; i < wire.curvenetMeshPoints.size(); ++i) {
        CHECK(live.curvenetBindInputs.meshPoints[i][0] ==
              wire.curvenetMeshPoints[i][0]);
        CHECK(live.curvenetBindInputs.meshPoints[i][1] ==
              wire.curvenetMeshPoints[i][1]);
        CHECK(live.curvenetBindInputs.meshPoints[i][2] ==
              wire.curvenetMeshPoints[i][2]);
    }
    _BinaryCheckEqual(live.curvenetBindInputs.meshCounts,
                      wire.curvenetMeshCounts);
    _BinaryCheckEqual(live.curvenetBindInputs.meshIndices,
                      wire.curvenetMeshIndices);
    CHECK(live.curvenetBindInputs.held == wire.curvenetBindInputsHeld);
    CHECK(live.blendChannels.size() == wire.blendChannels.size());
    for (size_t i = 0; i < wire.blendChannels.size(); ++i) {
        if (live.blendChannels[i].weight) {
            CHECK(live.blendChannels[i].weight.GetPath().GetString() ==
                  _BinaryString(reader, wire.blendChannels[i].weight));
        } else {
            CHECK(wire.blendChannels[i].weight == 0);
        }
        CHECK(bool(live.blendChannels[i].weight) ==
              wire.blendChannels[i].weightValid);
        CHECK(live.blendChannels[i].weightPath.GetString() ==
              _BinaryString(reader, wire.blendChannels[i].weightPath));
        CHECK(live.blendChannels[i].poseWeight ==
              wire.blendChannels[i].poseWeight);
        CHECK(live.blendChannels[i].samples.size() ==
              wire.blendChannels[i].samples.size());
        for (size_t s = 0; s < wire.blendChannels[i].samples.size(); ++s) {
            const auto &liveSample = live.blendChannels[i].samples[s];
            const auto &wireSample = wire.blendChannels[i].samples[s];
            CHECK(liveSample.samplePath.GetString() ==
                  _BinaryString(reader, wireSample.samplePath));
            if (liveSample.activation) {
                CHECK(liveSample.activation.GetPath().GetString() ==
                      _BinaryString(reader, wireSample.activation));
            } else {
                CHECK(wireSample.activation == 0);
            }
            CHECK(bool(liveSample.activation) ==
                  wireSample.activationValid);
            if (liveSample.points) {
                CHECK(liveSample.points.GetPath().GetString() ==
                      _BinaryString(reader, wireSample.points));
            } else {
                CHECK(wireSample.points == 0);
            }
            CHECK(bool(liveSample.points) == wireSample.pointsValid);
            CHECK(liveSample.pointsPath.GetString() ==
                  _BinaryString(reader, wireSample.pointsPath));
            _BinaryComparePhase(liveSample.phase, wireSample.phase,
                                reader);
            CHECK(liveSample.blendShape.GetString() ==
                  _BinaryString(reader, wireSample.blendShape));
            CHECK(bool(liveSample.layout) == wireSample.hasLayout);
            if (liveSample.layout) {
                CHECK(liveSample.layout->offsets.size() ==
                      wireSample.offsets.size());
                for (size_t p = 0; p < wireSample.offsets.size(); ++p) {
                    CHECK(liveSample.layout->offsets[p][0] ==
                          wireSample.offsets[p][0]);
                    CHECK(liveSample.layout->offsets[p][1] ==
                          wireSample.offsets[p][1]);
                    CHECK(liveSample.layout->offsets[p][2] ==
                          wireSample.offsets[p][2]);
                }
                _BinaryCheckEqual(liveSample.layout->indices,
                                  wireSample.indices);
                CHECK(uint64_t(liveSample.layout->pointCount) ==
                      wireSample.pointCount);
                CHECK(liveSample.layout->valid == wireSample.layoutValid);
            }
        }
    }
    _BinaryCheckEqual(live.influenceSlots, wire.influenceSlots);
    CHECK(live.transformSlot == wire.transformSlot);
    CHECK(live.transformSpaceSlot == wire.transformSpaceSlot);
    CHECK(live.constraintDelta == wire.constraintDelta);
    CHECK(live.driverFramesSolver == wire.driverFramesSolver);
    CHECK(live.finalPhase == wire.finalPhase);
    CHECK(live.skinTopologyFixed == wire.skinTopologyFixed);
    CHECK(live.snapshotAfter == wire.snapshotAfter);
    CHECK(live.readsSnapshots == wire.readsSnapshots);
    CHECK(live.packetInfluences.size() == wire.packetInfluences.size());
    for (size_t i = 0; i < wire.packetInfluences.size(); ++i) {
        _BinaryCheckMatrix(live.packetInfluences[i],
                           wire.packetInfluences[i]);
    }
    CHECK(live.chunks.size() == wire.chunks.size());
    for (size_t i = 0; i < wire.chunks.size(); ++i) {
        CHECK(live.chunks[i].begin == wire.chunks[i].begin);
        CHECK(live.chunks[i].end == wire.chunks[i].end);
        _BinaryCheckEqual(live.chunks[i].key, wire.chunks[i].key);
    }
    CHECK(live.chunkBase == wire.chunkBase);
    _BinaryCompareTopology(live.partitionTopology, wire.partitionTopology);
    CHECK(live.partitionElementSize == wire.partitionElementSize);
    CHECK(uint64_t(live.partitionIndexCount) ==
          wire.partitionIndexCount);
    CHECK(uint64_t(live.partitionPointCount) ==
          wire.partitionPointCount);
    CHECK(live.chunked == wire.chunked);
    CHECK(uint64_t(live.partitionCandidates) ==
          wire.partitionCandidates);
    CHECK(live.partitionReadyMin == wire.partitionReadyMin);
    CHECK(live.partitionReadyMax == wire.partitionReadyMax);
    CHECK(live.weightObject == wire.weightObject);
    CHECK(live.weightOperationDomain == wire.weightOperationDomain);
    CHECK(live.weightFieldTarget.GetString() ==
          _BinaryString(reader, wire.weightFieldTarget));
    CHECK(live.weightCurrentPhase == wire.weightCurrentPhase);
    _BinaryCompareTopology(live.topology, wire.topology);
    CHECK(live.topologyResolved == wire.topologyResolved);
}

void
_BinaryCompareWeightObject(
    const rigExec::RigExecBakedProgramImpl::WeightObject &live,
    const rigExec::RigExecWireWeightObject &wire,
    const rigExec::RigExecBinaryReader &reader)
{
    CHECK(live.path.GetString() == _BinaryString(reader, wire.path));
    CHECK(live.type.GetString() == _BinaryString(reader, wire.type));
    CHECK(live.representation.GetString() ==
          _BinaryString(reader, wire.representation));
    CHECK(live.rangePolicy.GetString() ==
          _BinaryString(reader, wire.rangePolicy));
    _BinaryCheckEqual(live.values, wire.values);
    _BinaryCheckEqual(live.indices, wire.indices);
    _BinaryCompareInput(live.defaultWeight, wire.defaultWeight, reader);
    CHECK(live.base == wire.base);
    _BinaryCheckEqual(live.inputs, wire.inputs);
    _BinaryCompareInput(live.driver, wire.driver, reader);
    _BinaryCompareInput(live.scale, wire.scale, reader);
    _BinaryCompareInput(live.bias, wire.bias, reader);
    CHECK(live.combineMode.GetString() ==
          _BinaryString(reader, wire.combineMode));
    _BinaryCompareInput(live.strength, wire.strength, reader);
    _BinaryCompareInput(live.invert, wire.invert, reader);
    _BinaryCompareAttributes(live.combineTargetPoints,
                             wire.combineTargetPoints,
                             wire.combineTargetValid, reader);
    CHECK(uint64_t(live.costElements) == wire.costElements);
    CHECK(live.providerSlot == wire.providerSlot);
    _BinaryCompareInput(live.falloffMin, wire.falloffMin, reader);
    _BinaryCompareInput(live.falloffMax, wire.falloffMax, reader);
    _BinaryCompareInput(live.scaleX, wire.scaleX, reader);
    _BinaryCompareInput(live.scaleY, wire.scaleY, reader);
    _BinaryCompareInput(live.scaleZ, wire.scaleZ, reader);
    _BinaryCompareInput(live.extentU, wire.extentU, reader);
    _BinaryCompareInput(live.extentV, wire.extentV, reader);
    CHECK(live.planeAxis.GetString() ==
          _BinaryString(reader, wire.planeAxis));
    CHECK(live.planeBounds.GetString() ==
          _BinaryString(reader, wire.planeBounds));
    _BinaryCompareAttributes(live.targetPoints, wire.targetPoints,
                             wire.targetValid, reader);
    _BinaryCompareAttributes(live.samplePoints, wire.samplePoints,
                             wire.sampleValid, reader);
    _BinaryCompareAttributes(live.curvePoints, wire.curvePoints,
                             wire.curveValid, reader);
    _BinaryCheckEqual(live.falloffCurve, wire.falloffCurve);
    _BinaryCompareAttributes(live.curvenetMeshPoints,
                             wire.curvenetMeshPoints,
                             wire.curvenetMeshValid, reader);
    _BinaryCompareAttributes(live.curvenetPoints, wire.curvenetPoints,
                             wire.curvenetPointsValid, reader);
    _BinaryCompareAttributes(live.curvenetCounts, wire.curvenetCounts,
                             wire.curvenetCountsValid, reader);
    _BinaryCompareAttributes(live.curvenetIndices, wire.curvenetIndices,
                             wire.curvenetIndicesValid, reader);
    _BinaryCompareAttributes(live.curvenetSplines, wire.curvenetSplines,
                             wire.curvenetSplinesValid, reader);
    if (live.curvenetWeights) {
        CHECK(live.curvenetWeights.GetPath().GetString() ==
              _BinaryString(reader, wire.curvenetWeights));
    } else {
        CHECK(wire.curvenetWeights == 0);
    }
    CHECK(bool(live.curvenetWeights) == wire.curvenetWeightsValid);
    if (live.curvenetAutoSmooth) {
        CHECK(live.curvenetAutoSmooth.GetPath().GetString() ==
              _BinaryString(reader, wire.curvenetAutoSmooth));
    } else {
        CHECK(wire.curvenetAutoSmooth == 0);
    }
    CHECK(bool(live.curvenetAutoSmooth) == wire.curvenetAutoSmoothValid);
    CHECK(live.curvenetBasis.GetString() ==
          _BinaryString(reader, wire.curvenetBasis));
    _BinaryCompareInput(live.curvenetSamples, wire.curvenetSamples,
                        reader);
    _BinaryCompareInput(live.curvenetUnreached, wire.curvenetUnreached,
                        reader);
    CHECK(live.boundMesh.size() == wire.boundMesh.size());
    for (size_t i = 0; i < wire.boundMesh.size(); ++i) {
        CHECK(live.boundMesh[i][0] == wire.boundMesh[i][0]);
        CHECK(live.boundMesh[i][1] == wire.boundMesh[i][1]);
        CHECK(live.boundMesh[i][2] == wire.boundMesh[i][2]);
    }
    CHECK(live.boundNet.size() == wire.boundNet.size());
    for (size_t i = 0; i < wire.boundNet.size(); ++i) {
        CHECK(live.boundNet[i][0] == wire.boundNet[i][0]);
        CHECK(live.boundNet[i][1] == wire.boundNet[i][1]);
        CHECK(live.boundNet[i][2] == wire.boundNet[i][2]);
    }
    _BinaryCheckEqual(live.boundCounts, wire.boundCounts);
    _BinaryCheckEqual(live.boundIndices, wire.boundIndices);
    _BinaryCheckEqual(live.boundSplines, wire.boundSplines);
    _BinaryCheckEqual(live.boundSmooth, wire.boundSmooth);
    CHECK(live.boundSamples == wire.boundSamples);
    CHECK(live.bound == wire.bound);
}

void
_BinaryCompareDomainGeometry(
    const rigExec::RigExecBakedProgramImpl &program,
    const rigExec::RigExecWireDomainGeometry &geometry,
    const rigExec::RigExecBinaryReader &reader)
{
    CHECK(program.chains.size() == geometry.chains.size());
    for (size_t i = 0; i < geometry.chains.size(); ++i) {
        CHECK(program.chains[i].target.GetString() ==
              _BinaryString(reader, geometry.chains[i].target));
        CHECK(program.chains[i].revisions.size() ==
              geometry.chains[i].revisions.size());
        for (size_t r = 0; r < geometry.chains[i].revisions.size(); ++r) {
            _BinaryCompareRevision(program.chains[i].revisions[r],
                                   geometry.chains[i].revisions[r], reader);
        }
        CHECK(program.chains[i].derived.size() ==
              geometry.chains[i].derived.size());
        for (size_t d = 0; d < geometry.chains[i].derived.size(); ++d) {
            CHECK(program.chains[i].derived[d].target.GetString() ==
                  _BinaryString(reader,
                                geometry.chains[i].derived[d].target));
            _BinaryCompareRevision(program.chains[i].derived[d].revision,
                                   geometry.chains[i].derived[d].revision,
                                   reader);
        }
    }
    CHECK(program.revisionIndex.size() == geometry.revisionIndex.size());
    for (size_t i = 0; i < geometry.revisionIndex.size(); ++i) {
        CHECK(program.revisionIndex[i].first ==
              geometry.revisionIndex[i].first);
        CHECK(program.revisionIndex[i].second ==
              geometry.revisionIndex[i].second);
    }
    CHECK(program.derivedIndex.size() == geometry.derivedIndex.size());
    for (size_t i = 0; i < geometry.derivedIndex.size(); ++i) {
        CHECK(program.derivedIndex[i].first ==
              geometry.derivedIndex[i].first);
        CHECK(program.derivedIndex[i].second ==
              geometry.derivedIndex[i].second);
    }
    _BinaryCheckEqual(program.chainRevisionBegin,
                      geometry.chainRevisionBegin);
    _BinaryCheckEqual(program.chainRevisionEnd, geometry.chainRevisionEnd);
    _BinaryCheckEqual(program.revisionChunkBase,
                      geometry.revisionChunkBase);
    _BinaryCheckEqual(program.revisionChunkCount,
                      geometry.revisionChunkCount);
    _BinaryCheckEqual(program.chainChunkBegin, geometry.chainChunkBegin);
    _BinaryCheckEqual(program.chainChunkEnd, geometry.chainChunkEnd);
    CHECK(program.weightObjects.size() == geometry.weightObjects.size());
    for (size_t i = 0; i < geometry.weightObjects.size(); ++i) {
        _BinaryCompareWeightObject(program.weightObjects[i],
                                   geometry.weightObjects[i], reader);
    }
    CHECK(program.falloffLuts.size() == geometry.falloffPaths.size());
    CHECK(program.falloffLuts.size() == geometry.falloffLuts.size());
    size_t lut = 0;
    for (const auto &entry : program.falloffLuts) {
        CHECK(entry.first.GetString() ==
              _BinaryString(reader, geometry.falloffPaths[lut]));
        _BinaryCheckEqual(entry.second, geometry.falloffLuts[lut]);
        ++lut;
    }
    CHECK(program.currentPhaseWeights.size() ==
          geometry.currentPhaseWeights.size());
    lut = 0;
    for (const SdfPath &path : program.currentPhaseWeights) {
        CHECK(path.GetString() ==
              _BinaryString(reader, geometry.currentPhaseWeights[lut]));
        ++lut;
    }
    CHECK(program.deltaBasePaths.size() == geometry.deltaBasePaths.size());
    for (size_t i = 0; i < geometry.deltaBasePaths.size(); ++i) {
        CHECK(program.deltaBasePaths[i].GetString() ==
              _BinaryString(reader, geometry.deltaBasePaths[i]));
    }
}

/// One live input in traversal order: the oracle mirror of the capture's
/// directory walk (see RigExecBakeCapture's constructor). The two
/// traversals are intentionally separate code: if one drifts, the
/// directory comparison below fails rather than agreeing vacuously.
struct _BinaryOracleInput {
    rigExec::RigExecWireInput::Tag tag;
    int overrideIndex = -1;
    SdfPath headPath;
    bool varying = false;
    bool bound = false;
    /// Re-samples the query route at a frame, mirroring the funnel:
    /// constant first, then the query read with its return ignored.
    /// Null for inputs with no oracle (constant, unbound, resolved
    /// route -- the overlay has moved on by comparison time).
    std::function<bool(UsdTimeCode, VtValue *)> sample;
};

template <class T>
void
_BinaryCollectOne(const rigExec::RigExecBakedInput<T> &input,
                  rigExec::RigExecWireInput::Tag tag,
                  std::vector<_BinaryOracleInput> *out)
{
    _BinaryOracleInput entry;
    entry.tag = tag;
    entry.overrideIndex = input.overrideIndex;
    entry.headPath = input.head ? input.head.GetPath() : SdfPath();
    entry.varying = input.varying;
    entry.bound = input.query.IsValid() || bool(input.resolvedAttr);
    if (entry.varying && !input.resolvedAttr && input.query.IsValid()) {
        entry.sample = [&input](UsdTimeCode time, VtValue *value) {
            T sampled = input.constant;
            input.query.Get(&sampled, time);
            *value = VtValue(sampled);
            return true;
        };
    }
    out->push_back(std::move(entry));
}

std::vector<_BinaryOracleInput>
_BinaryCollectOracle(const rigExec::RigExecBakedProgramImpl &program)
{
    using Tag = rigExec::RigExecWireInput::Tag;
    std::vector<_BinaryOracleInput> oracle;
    for (const auto &ladder : program.ladders) {
        _BinaryCollectOne(ladder.restSpace, Tag::Matrix4d, &oracle);
        _BinaryCollectOne(ladder.defaultSpace, Tag::Matrix4d, &oracle);
        _BinaryCollectOne(ladder.posedSpace, Tag::Matrix4d, &oracle);
        for (const auto &avar : ladder.restAvars) {
            _BinaryCollectOne(avar, Tag::Double, &oracle);
        }
        for (const auto &avar : ladder.defaultAvars) {
            _BinaryCollectOne(avar, Tag::Double, &oracle);
        }
        _BinaryCollectOne(ladder.rotationOrder, Tag::Token, &oracle);
    }
    for (const auto &interp : program.poseInterpolators) {
        _BinaryCollectOne(interp.enabled, Tag::Bool, &oracle);
    }
    for (const auto &solver : program.solvers) {
        _BinaryCollectOne(solver.bend, Tag::Double, &oracle);
        _BinaryCollectOne(solver.upperOffset, Tag::Double, &oracle);
        _BinaryCollectOne(solver.lowerOffset, Tag::Double, &oracle);
        _BinaryCollectOne(solver.stretch, Tag::Float, &oracle);
        _BinaryCollectOne(solver.softness, Tag::Float, &oracle);
        _BinaryCollectOne(solver.blendWeight, Tag::Float, &oracle);
        _BinaryCollectOne(solver.preserveVolume, Tag::Double, &oracle);
        _BinaryCollectOne(solver.midFollowWeight, Tag::Double, &oracle);
        _BinaryCollectOne(solver.roll, Tag::Double, &oracle);
        _BinaryCollectOne(solver.twist, Tag::Double, &oracle);
        _BinaryCollectOne(solver.minLengthRatio, Tag::Double, &oracle);
        _BinaryCollectOne(solver.twistTurns, Tag::Double, &oracle);
        _BinaryCollectOne(solver.ribbonSampleCount, Tag::Int, &oracle);
    }
    for (const auto &constraint : program.constraints) {
        _BinaryCollectOne(constraint.enabled, Tag::Bool, &oracle);
        _BinaryCollectOne(constraint.defaultWeight, Tag::Float, &oracle);
        _BinaryCollectOne(constraint.offset, Tag::Vec3d, &oracle);
        _BinaryCollectOne(constraint.affectX, Tag::Bool, &oracle);
        _BinaryCollectOne(constraint.affectY, Tag::Bool, &oracle);
        _BinaryCollectOne(constraint.affectZ, Tag::Bool, &oracle);
        _BinaryCollectOne(constraint.tX, Tag::Bool, &oracle);
        _BinaryCollectOne(constraint.tY, Tag::Bool, &oracle);
        _BinaryCollectOne(constraint.tZ, Tag::Bool, &oracle);
        _BinaryCollectOne(constraint.rX, Tag::Bool, &oracle);
        _BinaryCollectOne(constraint.rY, Tag::Bool, &oracle);
        _BinaryCollectOne(constraint.rZ, Tag::Bool, &oracle);
        _BinaryCollectOne(constraint.sX, Tag::Bool, &oracle);
        _BinaryCollectOne(constraint.sY, Tag::Bool, &oracle);
        _BinaryCollectOne(constraint.sZ, Tag::Bool, &oracle);
        _BinaryCollectOne(constraint.aimVector, Tag::Vec3d, &oracle);
        _BinaryCollectOne(constraint.upVector, Tag::Vec3d, &oracle);
        _BinaryCollectOne(constraint.rotationOffset, Tag::Vec3d, &oracle);
        _BinaryCollectOne(constraint.worldUpVector, Tag::Vec3d, &oracle);
        _BinaryCollectOne(constraint.poleVector, Tag::Vec3d, &oracle);
        _BinaryCollectOne(constraint.twistDegrees, Tag::Double, &oracle);
    }
    for (const auto &object : program.weightObjects) {
        _BinaryCollectOne(object.defaultWeight, Tag::Float, &oracle);
        _BinaryCollectOne(object.driver, Tag::Float, &oracle);
        _BinaryCollectOne(object.scale, Tag::Float, &oracle);
        _BinaryCollectOne(object.bias, Tag::Float, &oracle);
        _BinaryCollectOne(object.strength, Tag::Float, &oracle);
        _BinaryCollectOne(object.invert, Tag::Float, &oracle);
        _BinaryCollectOne(object.falloffMin, Tag::Float, &oracle);
        _BinaryCollectOne(object.falloffMax, Tag::Float, &oracle);
        _BinaryCollectOne(object.scaleX, Tag::Float, &oracle);
        _BinaryCollectOne(object.scaleY, Tag::Float, &oracle);
        _BinaryCollectOne(object.scaleZ, Tag::Float, &oracle);
        _BinaryCollectOne(object.extentU, Tag::Float, &oracle);
        _BinaryCollectOne(object.extentV, Tag::Float, &oracle);
        _BinaryCollectOne(object.curvenetSamples, Tag::Int, &oracle);
        _BinaryCollectOne(object.curvenetUnreached, Tag::Float, &oracle);
    }
    for (const auto &binding : program.avarBindings) {
        _BinaryCollectOne(binding.input, Tag::Double, &oracle);
    }
    for (const auto &binding : program.avarConstantBindings) {
        _BinaryCollectOne(binding.input, Tag::Double, &oracle);
    }
    return oracle;
}

void
_BinaryCompareValue(const VtValue &live,
                    const rigExec::RigExecWireValue &wire,
                    const rigExec::RigExecBinaryReader &reader)
{
    switch (wire.tag) {
    case rigExec::RigExecWireInput::Tag::Double:
        CHECK(live.IsHolding<double>() &&
              live.UncheckedGet<double>() == wire.f64);
        break;
    case rigExec::RigExecWireInput::Tag::Float:
        CHECK(live.IsHolding<float>() &&
              live.UncheckedGet<float>() == wire.f32);
        break;
    case rigExec::RigExecWireInput::Tag::Bool:
        CHECK(live.IsHolding<bool>() &&
              live.UncheckedGet<bool>() == wire.boolean);
        break;
    case rigExec::RigExecWireInput::Tag::Int:
        CHECK(live.IsHolding<int>() &&
              live.UncheckedGet<int>() == wire.i32);
        break;
    case rigExec::RigExecWireInput::Tag::Matrix4d: {
        CHECK(live.IsHolding<GfMatrix4d>());
        const GfMatrix4d &m = live.UncheckedGet<GfMatrix4d>();
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                CHECK(m[r][c] == wire.matrix[size_t(r * 4 + c)]);
            }
        }
        break;
    }
    case rigExec::RigExecWireInput::Tag::Token:
        CHECK(live.IsHolding<TfToken>() &&
              live.UncheckedGet<TfToken>().GetString() ==
                  _BinaryString(reader, wire.token));
        break;
    case rigExec::RigExecWireInput::Tag::Vec3d: {
        CHECK(live.IsHolding<GfVec3d>());
        const GfVec3d &v = live.UncheckedGet<GfVec3d>();
        CHECK(v[0] == wire.vec[0]);
        CHECK(v[1] == wire.vec[1]);
        CHECK(v[2] == wire.vec[2]);
        break;
    }
    }
}

/// Compares one captured frame against the live program it was drained
/// from -- called before the next Evaluate moves the program on, so the
/// prologue state is exactly what the capture snapshotted.
void
_BinaryCompareTableFrame(const rigExec::RigExecBakedProgramImpl &program,
                         const rigExec::RigExecWireFrameInputs &frame,
                         const std::vector<_BinaryOracleInput> &oracle,
                         const rigExec::RigExecWireInputTable &table,
                         const rigExec::RigExecBinaryReader &reader)
{
    CHECK(frame.uids.size() == frame.values.size());
    for (size_t i = 0; i < frame.uids.size(); ++i) {
        CHECK(frame.uids[i] < table.directory.size());
        if (frame.uids[i] >= table.directory.size()) {
            continue;
        }
        CHECK(uint8_t(frame.values[i].tag) ==
              uint8_t(table.directory[frame.uids[i]].tag));
        // The oracle re-samples the query route independently; the
        // resolved route has no post-hoc oracle (the overlay moved on)
        // and is covered structurally here plus numerically by parity.
        size_t seen = 0;
        for (size_t k = 0; k < oracle.size(); ++k) {
            if (!oracle[k].varying || !oracle[k].bound) {
                continue;
            }
            if (seen == frame.uids[i] && oracle[k].sample) {
                VtValue sampled;
                CHECK(oracle[k].sample(UsdTimeCode(frame.frame),
                                      &sampled));
                _BinaryCompareValue(sampled, frame.values[i], reader);
                break;
            }
            ++seen;
        }
    }
    // The path oracle re-reads the stage attribute the site consumed:
    // was-Default records read Default, the rest the frame time. Absent
    // records require an invalid attribute; anything else must match the
    // live value exactly, mirroring the hook (the read return ignored,
    // so a valid-but-unreadable attribute compares its fallback).
    for (const auto &read : frame.pathReads) {
        const SdfPath path(_BinaryString(reader, read.path));
        const UsdAttribute attr =
            program.stage->GetAttributeAtPath(path);
        const UsdTimeCode when = read.wasDefault
            ? UsdTimeCode::Default()
            : UsdTimeCode(frame.frame);
        const auto &value = read.value;
        using PathTag = rigExec::RigExecWirePathValue::Tag;
        if (value.tag == PathTag::Absent) {
            CHECK(!attr);
            continue;
        }
        CHECK(attr);
        if (!attr) {
            continue;
        }
        switch (value.tag) {
        case PathTag::Absent:
            break;
        case PathTag::Bool: {
            bool sampled = false;
            attr.Get(&sampled, when);
            CHECK(sampled == value.boolean);
            break;
        }
        case PathTag::Int: {
            int sampled = 0;
            attr.Get(&sampled, when);
            CHECK(sampled == value.i32);
            break;
        }
        case PathTag::Float: {
            // A resolved-route float read coerces a double source, the
            // way GetAttribute does; a direct read does not.
            float sampled = 0;
            if (read.forceFrame &&
                attr.GetTypeName() == SdfValueTypeNames->Double) {
                double wide = 0;
                attr.Get(&wide, when);
                sampled = static_cast<float>(wide);
            } else {
                attr.Get(&sampled, when);
            }
            CHECK(sampled == value.f32);
            break;
        }
        case PathTag::Double: {
            double sampled = 0;
            attr.Get(&sampled, when);
            CHECK(sampled == value.f64);
            break;
        }
        case PathTag::Token: {
            TfToken sampled;
            attr.Get(&sampled, when);
            CHECK(sampled.GetString() ==
                  _BinaryString(reader, value.token));
            break;
        }
        case PathTag::Matrix4d: {
            GfMatrix4d sampled(1);
            attr.Get(&sampled, when);
            _BinaryCheckMatrix(sampled, value.matrix);
            break;
        }
        case PathTag::Vec3d: {
            GfVec3d sampled(0);
            attr.Get(&sampled, when);
            CHECK(sampled[0] == value.vec[0]);
            CHECK(sampled[1] == value.vec[1]);
            CHECK(sampled[2] == value.vec[2]);
            break;
        }
        case PathTag::IntArray: {
            VtIntArray sampled;
            attr.Get(&sampled, when);
            CHECK(sampled.size() == value.ints.size());
            const size_t count =
                std::min(sampled.size(), value.ints.size());
            for (size_t i = 0; i < count; ++i) {
                CHECK(sampled[i] == value.ints[i]);
            }
            break;
        }
        case PathTag::FloatArray: {
            VtFloatArray sampled;
            attr.Get(&sampled, when);
            CHECK(sampled.size() == value.floats.size());
            const size_t count =
                std::min(sampled.size(), value.floats.size());
            for (size_t i = 0; i < count; ++i) {
                CHECK(sampled[i] == value.floats[i]);
            }
            break;
        }
        case PathTag::Vec2fArray: {
            VtArray<GfVec2f> sampled;
            attr.Get(&sampled, when);
            CHECK(sampled.size() == value.vec2s.size());
            const size_t count =
                std::min(sampled.size(), value.vec2s.size());
            for (size_t i = 0; i < count; ++i) {
                CHECK(sampled[i][0] == value.vec2s[i][0]);
                CHECK(sampled[i][1] == value.vec2s[i][1]);
            }
            break;
        }
        case PathTag::Vec3fArray: {
            VtVec3fArray sampled;
            attr.Get(&sampled, when);
            CHECK(sampled.size() == value.vec3s.size());
            const size_t count =
                std::min(sampled.size(), value.vec3s.size());
            for (size_t i = 0; i < count; ++i) {
                CHECK(sampled[i][0] == value.vec3s[i][0]);
                CHECK(sampled[i][1] == value.vec3s[i][1]);
                CHECK(sampled[i][2] == value.vec3s[i][2]);
            }
            break;
        }
        case PathTag::Vec3i: {
            GfVec3i sampled(0);
            attr.Get(&sampled, when);
            CHECK(sampled[0] == value.vec3i[0]);
            CHECK(sampled[1] == value.vec3i[1]);
            CHECK(sampled[2] == value.vec3i[2]);
            break;
        }
        case PathTag::DoubleArray: {
            VtDoubleArray sampled;
            attr.Get(&sampled, when);
            CHECK(sampled.size() == value.doubles.size());
            const size_t count =
                std::min(sampled.size(), value.doubles.size());
            for (size_t i = 0; i < count; ++i) {
                CHECK(sampled[i] == value.doubles[i]);
            }
            break;
        }
        }
    }
    CHECK(program.chains.size() == frame.chainHaveBase.size());
    CHECK(program.chains.size() == frame.chainBases.size());
    for (size_t i = 0; i < frame.chainBases.size(); ++i) {
        CHECK(program.chains[i].haveBase == (frame.chainHaveBase[i] != 0));
        CHECK(program.chains[i].lastBase.size() ==
              frame.chainBases[i].size());
        for (size_t p = 0; p < frame.chainBases[i].size(); ++p) {
            CHECK(program.chains[i].lastBase[p][0] ==
                  frame.chainBases[i][p][0]);
            CHECK(program.chains[i].lastBase[p][1] ==
                  frame.chainBases[i][p][1]);
            CHECK(program.chains[i].lastBase[p][2] ==
                  frame.chainBases[i][p][2]);
        }
    }
    size_t derived = 0;
    for (const auto &chain : program.chains) {
        derived += chain.derived.size();
    }
    CHECK(derived == frame.derivedHaveBase.size());
    CHECK(derived == frame.derivedBases.size());
    size_t d = 0;
    for (const auto &chain : program.chains) {
        for (const auto &entry : chain.derived) {
            CHECK(entry.haveBase == (frame.derivedHaveBase[d] != 0));
            CHECK(entry.lastBase.size() == frame.derivedBases[d].size());
            for (size_t p = 0; p < frame.derivedBases[d].size(); ++p) {
                CHECK(entry.lastBase[p][0] ==
                      frame.derivedBases[d][p][0]);
                CHECK(entry.lastBase[p][1] ==
                      frame.derivedBases[d][p][1]);
                CHECK(entry.lastBase[p][2] ==
                      frame.derivedBases[d][p][2]);
            }
            ++d;
        }
    }
    CHECK(program.xformBase.size() == frame.xformBase.size());
    for (size_t i = 0; i < frame.xformBase.size(); ++i) {
        _BinaryCheckMatrix(program.xformBase[i], frame.xformBase[i]);
    }
    CHECK(program.nativeFrames.size() == frame.nativeFrames.size());
    for (size_t i = 0; i < frame.nativeFrames.size(); ++i) {
        _BinaryCheckFrame(program.nativeFrames[i], frame.nativeFrames[i]);
    }
    CHECK(program.propertyResults.size() == frame.propertyPaths.size());
    CHECK(program.propertyResults.size() == frame.propertyValues.size());
    size_t prop = 0;
    for (const auto &entry : program.propertyResults) {
        CHECK(entry.first.GetString() ==
              _BinaryString(reader, frame.propertyPaths[prop]));
        const VtValue &held = entry.second;
        const auto &wire = frame.propertyValues[prop];
        if (held.IsHolding<float>()) {
            CHECK(uint8_t(wire.tag) == 0);
            CHECK(held.UncheckedGet<float>() == wire.f32);
        } else if (held.IsHolding<double>()) {
            CHECK(uint8_t(wire.tag) == 1);
            CHECK(held.UncheckedGet<double>() == wire.f64);
        } else if (held.IsHolding<GfMatrix4d>()) {
            CHECK(uint8_t(wire.tag) == 2);
            _BinaryCheckMatrix(held.UncheckedGet<GfMatrix4d>(),
                               wire.matrix);
        } else if (held.IsHolding<GfVec3f>()) {
            CHECK(uint8_t(wire.tag) == 3);
            CHECK(held.UncheckedGet<GfVec3f>()[0] == wire.vec[0]);
            CHECK(held.UncheckedGet<GfVec3f>()[1] == wire.vec[1]);
            CHECK(held.UncheckedGet<GfVec3f>()[2] == wire.vec[2]);
        } else {
            CHECK(!"unencodable property result in comparison");
        }
        ++prop;
    }
    CHECK(program.constraintArrays.size() == frame.arrayWeights.size());
    CHECK(program.constraintArrays.size() ==
          frame.arrayTranslationOffsets.size());
    CHECK(program.constraintArrays.size() ==
          frame.arrayRotationOffsets.size());
    CHECK(program.constraintArrays.size() == frame.arrayOk.size());
    CHECK(program.constraintArrays.size() ==
          frame.arrayPoleWeights.size());
    CHECK(program.constraintArrays.size() == frame.arrayPoleOk.size());
    for (size_t i = 0; i < frame.arrayWeights.size(); ++i) {
        _BinaryCheckEqual(program.constraintArrays[i].weights,
                          frame.arrayWeights[i]);
        CHECK(program.constraintArrays[i].translationOffsets.size() ==
              frame.arrayTranslationOffsets[i].size());
        for (size_t k = 0; k < frame.arrayTranslationOffsets[i].size();
             ++k) {
            CHECK(program.constraintArrays[i].translationOffsets[k][0] ==
                  frame.arrayTranslationOffsets[i][k][0]);
            CHECK(program.constraintArrays[i].translationOffsets[k][1] ==
                  frame.arrayTranslationOffsets[i][k][1]);
            CHECK(program.constraintArrays[i].translationOffsets[k][2] ==
                  frame.arrayTranslationOffsets[i][k][2]);
        }
        CHECK(program.constraintArrays[i].rotationOffsets.size() ==
              frame.arrayRotationOffsets[i].size());
        for (size_t k = 0; k < frame.arrayRotationOffsets[i].size(); ++k) {
            CHECK(program.constraintArrays[i].rotationOffsets[k][0] ==
                  frame.arrayRotationOffsets[i][k][0]);
            CHECK(program.constraintArrays[i].rotationOffsets[k][1] ==
                  frame.arrayRotationOffsets[i][k][1]);
            CHECK(program.constraintArrays[i].rotationOffsets[k][2] ==
                  frame.arrayRotationOffsets[i][k][2]);
        }
        CHECK(program.constraintArrays[i].ok == (frame.arrayOk[i] != 0));
        _BinaryCheckEqual(program.constraintArrays[i].poleWeights,
                          frame.arrayPoleWeights[i]);
        CHECK(program.constraintArrays[i].poleOk ==
              (frame.arrayPoleOk[i] != 0));
    }
    CHECK(program.deltaBaseMatrix.size() == frame.deltaBaseMatrix.size());
    for (size_t i = 0; i < frame.deltaBaseMatrix.size(); ++i) {
        _BinaryCheckMatrix(program.deltaBaseMatrix[i],
                           frame.deltaBaseMatrix[i]);
    }
    CHECK(program.deltaBaseOk.size() == frame.deltaBaseOk.size());
    for (size_t i = 0; i < frame.deltaBaseOk.size(); ++i) {
        CHECK((program.deltaBaseOk[i] != 0) ==
              (frame.deltaBaseOk[i] != 0));
    }
    CHECK(program.weightPackets.size() == frame.weightPackets.size());
    for (size_t i = 0; i < frame.weightPackets.size(); ++i) {
        CHECK(program.weightPackets[i].representation.GetString() ==
              _BinaryString(reader,
                            frame.weightPackets[i].representation));
        CHECK(program.weightPackets[i].rangePolicy.GetString() ==
              _BinaryString(reader, frame.weightPackets[i].rangePolicy));
        _BinaryCheckEqual(program.weightPackets[i].values,
                          frame.weightPackets[i].values);
        _BinaryCheckEqual(program.weightPackets[i].indices,
                          frame.weightPackets[i].indices);
        CHECK(program.weightPackets[i].defaultWeight ==
              frame.weightPackets[i].defaultWeight);
        CHECK(program.weightPackets[i].valid ==
              frame.weightPackets[i].valid);
    }
    CHECK(program.currentPhaseWeights.size() ==
          frame.currentPhaseWeights.size());
    size_t w = 0;
    for (const SdfPath &path : program.currentPhaseWeights) {
        CHECK(path.GetString() ==
              _BinaryString(reader, frame.currentPhaseWeights[w]));
        ++w;
    }
    // The blend reads: record against retention, and retention against a
    // re-derivation of what the assembly consumed -- the retention is
    // capture-only (nothing downstream reads it), so only the mirror
    // proves the fill captured the read rather than a neighboring value.
    const RigExecResolvedInputs &blendReads = *program.resolvedInputs;
    const UsdTimeCode blendTime(frame.frame);
    auto CheckRevisionBlend =
        [&](const RigExecBakedProgramImpl::GeomRevision &revision,
            const std::vector<float> &wireWeights,
            const std::vector<std::vector<float>> &wireActivations,
            const std::vector<std::vector<std::vector<RigExecWireVec3f>>>
                &wirePoints) {
            CHECK(revision.blendChannels.size() == wireWeights.size());
            CHECK(revision.blendChannels.size() == wireActivations.size());
            CHECK(revision.blendChannels.size() == wirePoints.size());
            for (size_t c = 0; c < revision.blendChannels.size(); ++c) {
                if (c >= wireWeights.size() || c >= wireActivations.size() ||
                    c >= wirePoints.size()) {
                    continue;
                }
                const auto &bound = revision.blendChannels[c];
                CHECK(bound.lastWeight == wireWeights[c]);
                float expectedWeight = 0.0f;
                if (bound.poseWeight >= 0 &&
                    !blendReads.Find(bound.weightPath)) {
                    CHECK(size_t(bound.poseWeight) <
                          program.poseWeights.size());
                    if (size_t(bound.poseWeight) <
                        program.poseWeights.size()) {
                        expectedWeight =
                            program.poseWeights[size_t(bound.poseWeight)];
                    }
                } else {
                    blendReads.GetAttribute(bound.weight, blendTime,
                                            &expectedWeight);
                }
                CHECK(bound.lastWeight == expectedWeight);
                CHECK(bound.samples.size() ==
                      wireActivations[c].size());
                CHECK(bound.samples.size() == wirePoints[c].size());
                for (size_t s = 0; s < bound.samples.size(); ++s) {
                    if (s >= wireActivations[c].size() ||
                        s >= wirePoints[c].size()) {
                        continue;
                    }
                    const auto &boundSample = bound.samples[s];
                    CHECK(boundSample.lastActivation ==
                          wireActivations[c][s]);
                    float expectedActivation = 1.0f;
                    blendReads.GetAttribute(boundSample.activation,
                                            blendTime, &expectedActivation);
                    CHECK(boundSample.lastActivation == expectedActivation);
                    if (boundSample.blendShape.IsEmpty()) {
                        CHECK(boundSample.lastPoints.size() ==
                              wirePoints[c][s].size());
                        for (size_t p = 0;
                             p < wirePoints[c][s].size() &&
                             p < boundSample.lastPoints.size();
                             ++p) {
                            CHECK(boundSample.lastPoints[p][0] ==
                                  wirePoints[c][s][p][0]);
                            CHECK(boundSample.lastPoints[p][1] ==
                                  wirePoints[c][s][p][1]);
                            CHECK(boundSample.lastPoints[p][2] ==
                                  wirePoints[c][s][p][2]);
                        }
                        VtVec3fArray points;
                        const VtValue *phased =
                            program.runSnapshots.Lookup(
                                boundSample.pointsPath, boundSample.phase,
                                revision.moverPath);
                        if (phased &&
                            phased->IsHolding<VtVec3fArray>()) {
                            points = phased->UncheckedGet<VtVec3fArray>();
                        } else {
                            blendReads.GetAttribute(boundSample.points,
                                                    blendTime, &points);
                        }
                        CHECK(points.size() ==
                              boundSample.lastPoints.size());
                        for (size_t p = 0;
                             p < points.size() &&
                             p < boundSample.lastPoints.size();
                             ++p) {
                            CHECK(points[p] == boundSample.lastPoints[p]);
                        }
                    } else {
                        CHECK(boundSample.lastPoints.empty());
                        CHECK(wirePoints[c][s].empty());
                    }
                }
            }
        };
    auto CheckRevisionPacket =
        [&](const RigExecWeightPacket &packet,
            const RigExecWireWeightPacket &wire) {
            CHECK(packet.representation.GetString() ==
                  _BinaryString(reader, wire.representation));
            CHECK(packet.rangePolicy.GetString() ==
                  _BinaryString(reader, wire.rangePolicy));
            _BinaryCheckEqual(packet.values, wire.values);
            _BinaryCheckEqual(packet.indices, wire.indices);
            CHECK(packet.defaultWeight == wire.defaultWeight);
            CHECK(packet.valid == wire.valid);
        };
    auto CheckRevisionAdjuster =
        [&](const RigExecBakedProgramImpl::GeomRevision &revision,
            const RigExecWireMatrix4d &wire, uint8_t wireHave) {
            const bool published =
                revision.op == RigExecRevisionOp::CurvenetAdjuster &&
                revision.resultStatus == "ok";
            CHECK(published == (wireHave != 0));
            _BinaryCheckMatrix(revision.lastAdjusterNetToAsset, wire);
        };
    CHECK(program.chains.size() == frame.blendWeights.size());
    CHECK(program.chains.size() == frame.blendActivations.size());
    CHECK(program.chains.size() == frame.blendPoints.size());
    CHECK(program.chains.size() == frame.derivedBlendWeights.size());
    CHECK(program.chains.size() == frame.derivedBlendActivations.size());
    CHECK(program.chains.size() == frame.derivedBlendPoints.size());
    CHECK(program.chains.size() == frame.revisionDefaultWeights.size());
    CHECK(program.chains.size() == frame.revisionPhasePackets.size());
    CHECK(program.chains.size() == frame.derivedPhasePackets.size());
    CHECK(program.chains.size() == frame.revisionAdjusters.size());
    CHECK(program.chains.size() == frame.revisionAdjusterHave.size());
    size_t refusedExpected = 0;
    for (size_t i = 0; i < program.chains.size(); ++i) {
        if (i >= frame.blendWeights.size() ||
            i >= frame.blendActivations.size() ||
            i >= frame.blendPoints.size() ||
            i >= frame.derivedBlendWeights.size() ||
            i >= frame.derivedBlendActivations.size() ||
            i >= frame.derivedBlendPoints.size() ||
            i >= frame.revisionDefaultWeights.size() ||
            i >= frame.revisionPhasePackets.size() ||
            i >= frame.derivedPhasePackets.size() ||
            i >= frame.revisionAdjusters.size() ||
            i >= frame.revisionAdjusterHave.size()) {
            continue;
        }
        const auto &chain = program.chains[i];
        auto CheckOne =
            [&](const RigExecBakedProgramImpl::GeomRevision &revision,
                size_t r, bool derived) {
                const std::vector<std::vector<float>> &weights =
                    derived ? frame.derivedBlendWeights[i]
                            : frame.blendWeights[i];
                const std::vector<std::vector<std::vector<float>>>
                    &activations = derived ? frame.derivedBlendActivations[i]
                                           : frame.blendActivations[i];
                const std::vector<std::vector<std::vector<
                    std::vector<RigExecWireVec3f>>>> &points =
                    derived ? frame.derivedBlendPoints[i]
                            : frame.blendPoints[i];
                const std::vector<RigExecWireWeightPacket> &packets =
                    derived ? frame.derivedPhasePackets[i]
                            : frame.revisionPhasePackets[i];
                if (r >= weights.size() || r >= activations.size() ||
                    r >= points.size() || r >= packets.size()) {
                    return;
                }
                CheckRevisionBlend(revision, weights[r], activations[r],
                                   points[r]);
                CheckRevisionPacket(revision.currentPhasePacket, packets[r]);
                // Record against retention only, no stage mirror: the
                // fill IS the consumption site (RevisionStatic reads the
                // member for staticDirty two lines below the read), and a
                // cone-skipped step legitimately holds last frame's.
                // Main revisions only: derived ones never fill these
                // (no RevisionStatic arm, no adjuster publish), so they
                // carry no streams at all.
                if (!derived) {
                    if (r >= frame.revisionDefaultWeights[i].size() ||
                        r >= frame.revisionAdjusters[i].size() ||
                        r >= frame.revisionAdjusterHave[i].size()) {
                        return;
                    }
                    CHECK(revision.defaultWeight ==
                          frame.revisionDefaultWeights[i][r]);
                    CheckRevisionAdjuster(
                        revision, frame.revisionAdjusters[i][r],
                        frame.revisionAdjusterHave[i][r]);
                }
                for (const auto &channel : revision.blendChannels) {
                    for (const auto &sample : channel.samples) {
                        if (sample.layoutRefused) {
                            ++refusedExpected;
                        }
                    }
                }
            };
        // Every inner row the runtime indexes blindly is shape-checked
        // here: without these a short row would silently skip its
        // comparisons above instead of failing.
        CHECK(chain.revisions.size() == frame.blendWeights[i].size());
        CHECK(chain.revisions.size() ==
              frame.blendActivations[i].size());
        CHECK(chain.revisions.size() == frame.blendPoints[i].size());
        CHECK(chain.revisions.size() ==
              frame.revisionDefaultWeights[i].size());
        CHECK(chain.revisions.size() ==
              frame.revisionPhasePackets[i].size());
        CHECK(chain.revisions.size() ==
              frame.revisionAdjusters[i].size());
        CHECK(chain.revisions.size() ==
              frame.revisionAdjusterHave[i].size());
        for (size_t r = 0; r < chain.revisions.size(); ++r) {
            CheckOne(chain.revisions[r], r, false);
        }
        CHECK(chain.derived.size() ==
              frame.derivedBlendWeights[i].size());
        CHECK(chain.derived.size() ==
              frame.derivedBlendActivations[i].size());
        CHECK(chain.derived.size() == frame.derivedBlendPoints[i].size());
        CHECK(chain.derived.size() ==
              frame.derivedPhasePackets[i].size());
        for (size_t d = 0; d < chain.derived.size(); ++d) {
            CheckOne(chain.derived[d].revision, d, true);
        }
    }
    // The refused layouts compare as a SET keyed by (mover, sample): the
    // drain order carries no meaning and the runtime looks them up by
    // path, so the test must not pin the order down.
    CHECK(refusedExpected == frame.refusedLayouts.size());
    for (size_t i = 0; i < program.chains.size(); ++i) {
        auto CheckRefused =
            [&](const RigExecBakedProgramImpl::GeomRevision &revision) {
                for (const auto &channel : revision.blendChannels) {
                    for (const auto &sample : channel.samples) {
                        if (!sample.layoutRefused || !sample.layout) {
                            continue;
                        }
                        bool found = false;
                        for (const auto &wire : frame.refusedLayouts) {
                            if (_BinaryString(reader, wire.mover) !=
                                    revision.moverPath.GetString() ||
                                _BinaryString(reader, wire.sample) !=
                                    sample.samplePath.GetString()) {
                                continue;
                            }
                            found = true;
                            CHECK(uint64_t(sample.layout->pointCount) ==
                                  wire.pointCount);
                            CHECK(sample.layout->valid == wire.valid);
                            CHECK(sample.layout->offsets.size() ==
                                  wire.offsets.size());
                            for (size_t p = 0;
                                 p < wire.offsets.size() &&
                                 p < sample.layout->offsets.size();
                                 ++p) {
                                CHECK(sample.layout->offsets[p][0] ==
                                      wire.offsets[p][0]);
                                CHECK(sample.layout->offsets[p][1] ==
                                      wire.offsets[p][1]);
                                CHECK(sample.layout->offsets[p][2] ==
                                      wire.offsets[p][2]);
                            }
                            _BinaryCheckEqual(sample.layout->indices,
                                              wire.indices);
                        }
                        CHECK(found);
                    }
                }
            };
        for (const auto &revision : program.chains[i].revisions) {
            CheckRefused(revision);
        }
        for (const auto &derived : program.chains[i].derived) {
            CheckRefused(derived.revision);
        }
    }
    CHECK(program.solvers.size() == frame.solverRibbonPoints.size());
    for (size_t i = 0; i < program.solvers.size() &&
                       i < frame.solverRibbonPoints.size();
         ++i) {
        CHECK(program.solvers[i].ribbonPoints.size() ==
              frame.solverRibbonPoints[i].size());
        for (size_t p = 0;
             p < frame.solverRibbonPoints[i].size() &&
             p < program.solvers[i].ribbonPoints.size();
             ++p) {
            CHECK(program.solvers[i].ribbonPoints[p][0] ==
                  frame.solverRibbonPoints[i][p][0]);
            CHECK(program.solvers[i].ribbonPoints[p][1] ==
                  frame.solverRibbonPoints[i][p][1]);
            CHECK(program.solvers[i].ribbonPoints[p][2] ==
                  frame.solverRibbonPoints[i][p][2]);
        }
    }
    CHECK(program.constraints.size() == frame.constraintWeights.size());
    CHECK(program.constraints.size() == frame.constraintHaveWeight.size());
    for (size_t i = 0; i < program.constraints.size() &&
                       i < frame.constraintWeights.size() &&
                       i < frame.constraintHaveWeight.size();
         ++i) {
        const auto &constraint = program.constraints[i];
        const bool resolved =
            !constraint.weightObject.IsEmpty() &&
            constraint.pointsTarget.IsEmpty() &&
            constraint.weightScratch.size() == 1;
        CHECK(resolved == (frame.constraintHaveWeight[i] != 0));
        CHECK((resolved ? constraint.weightScratch[0] : 0.0f) ==
              frame.constraintWeights[i]);
    }
}

/// Compares the captured directory and override routing against the live
/// program, both ways: every varying bound input holds a uid, and every
/// uid names one.
void
_BinaryCompareTableStatic(const rigExec::RigExecBakedProgramImpl &program,
                          const rigExec::RigExecWireInputTable &table,
                          const std::vector<_BinaryOracleInput> &oracle,
                          const rigExec::RigExecBinaryReader &reader)
{
    size_t uids = 0;
    for (const _BinaryOracleInput &entry : oracle) {
        if (!entry.varying || !entry.bound) {
            continue;
        }
        CHECK(uids < table.directory.size());
        if (uids >= table.directory.size()) {
            return;
        }
        CHECK(uint8_t(entry.tag) ==
              uint8_t(table.directory[uids].tag));
        CHECK(entry.overrideIndex ==
              table.directory[uids].overrideIndex);
        CHECK(entry.headPath.GetString() ==
              _BinaryString(reader, table.directory[uids].head));
        ++uids;
    }
    CHECK(uids == table.directory.size());
    CHECK(program.overridableInputs.size() ==
          table.overridablePaths.size());
    CHECK(program.overridableInputs.size() ==
          table.overridableIndices.size());
    size_t row = 0;
    for (const auto &entry : program.overridableInputs) {
        CHECK(entry.first.GetString() ==
              _BinaryString(reader, table.overridablePaths[row]));
        _BinaryCheckEqual(entry.second, table.overridableIndices[row]);
        ++row;
    }
    // Promoted avars read per frame the long way: each one must hold a
    // directory entry, or its animation would bake as a constant.
    for (size_t promoted : program.promotedAvars) {
        CHECK(promoted < program.avarConstantBindings.size());
        if (promoted >= program.avarConstantBindings.size()) {
            continue;
        }
        const auto &input =
            program.avarConstantBindings[promoted].input;
        CHECK(input.varying &&
              (input.query.IsValid() || bool(input.resolvedAttr)));
    }
}

/// Exact equality on one captured value: the same process evaluates both
/// frames through the same deterministic kernels, so bit equality is the
/// right bar, the way the two-pass byte comparison already assumes.
bool
_BinaryWireValueEqual(const rigExec::RigExecWireValue &a,
                      const rigExec::RigExecWireValue &b)
{
    return a.tag == b.tag && a.f64 == b.f64 && a.f32 == b.f32 &&
           a.boolean == b.boolean && a.i32 == b.i32 &&
           a.matrix == b.matrix && a.token == b.token && a.vec == b.vec;
}

bool
_BinaryPropertyValueEqual(const rigExec::RigExecWirePropertyValue &a,
                          const rigExec::RigExecWirePropertyValue &b)
{
    return a.tag == b.tag && a.f32 == b.f32 && a.f64 == b.f64 &&
           a.matrix == b.matrix && a.vec == b.vec;
}

bool
_BinaryWeightPacketEqual(const rigExec::RigExecWireWeightPacket &a,
                         const rigExec::RigExecWireWeightPacket &b)
{
    return a.representation == b.representation &&
           a.rangePolicy == b.rangePolicy && a.values == b.values &&
           a.indices == b.indices && a.defaultWeight == b.defaultWeight &&
           a.valid == b.valid;
}

bool
_BinaryRefusedLayoutEqual(const rigExec::RigExecWireRefusedLayout &a,
                          const rigExec::RigExecWireRefusedLayout &b)
{
    return a.mover == b.mover && a.sample == b.sample &&
           a.pointCount == b.pointCount && a.valid == b.valid &&
           a.offsets == b.offsets && a.indices == b.indices;
}

bool
_BinaryPathValueEqual(const rigExec::RigExecWirePathValue &a,
                      const rigExec::RigExecWirePathValue &b)
{
    if (a.tag != b.tag) {
        return false;
    }
    using Tag = rigExec::RigExecWirePathValue::Tag;
    switch (a.tag) {
    case Tag::Absent:
        return true;
    case Tag::Bool:
        return a.boolean == b.boolean;
    case Tag::Int:
        return a.i32 == b.i32;
    case Tag::Float:
        return a.f32 == b.f32;
    case Tag::Double:
        return a.f64 == b.f64;
    case Tag::Token:
        return a.token == b.token;
    case Tag::Matrix4d:
        return a.matrix == b.matrix;
    case Tag::Vec3d:
        return a.vec == b.vec;
    case Tag::IntArray:
        return a.ints == b.ints;
    case Tag::FloatArray:
        return a.floats == b.floats;
    case Tag::Vec2fArray:
        return a.vec2s == b.vec2s;
    case Tag::Vec3fArray:
        return a.vec3s == b.vec3s;
    case Tag::Vec3i:
        return a.vec3i == b.vec3i;
    case Tag::DoubleArray:
        return a.doubles == b.doubles;
    }
    return false;
}

bool
_BinaryPathReadEqual(const rigExec::RigExecWirePathRead &a,
                     const rigExec::RigExecWirePathRead &b)
{
    return a.path == b.path && a.wasDefault == b.wasDefault &&
           a.forceFrame == b.forceFrame &&
           _BinaryPathValueEqual(a.value, b.value);
}

/// Names the top-level streams that differ between two frame records,
/// for the varies-guard's diagnostic: "uids" pins cone sparsity (presence
/// without value change) apart from genuine value variance.
std::string
_BinaryFrameInputsDiff(const rigExec::RigExecWireFrameInputs &a,
                       const rigExec::RigExecWireFrameInputs &b)
{
    std::string out;
    auto Note = [&](const char *stream) {
        if (!out.empty()) {
            out += ",";
        }
        out += stream;
    };
    if (a.uids != b.uids) {
        Note("uids");
    }
    bool valuesDiffer = a.values.size() != b.values.size();
    for (size_t i = 0; !valuesDiffer && i < a.values.size(); ++i) {
        valuesDiffer = !_BinaryWireValueEqual(a.values[i], b.values[i]);
    }
    if (valuesDiffer) {
        Note("values");
    }
    if (a.chainHaveBase != b.chainHaveBase) {
        Note("chainHaveBase");
    }
    if (a.chainBases != b.chainBases) {
        Note("chainBases");
    }
    if (a.derivedHaveBase != b.derivedHaveBase) {
        Note("derivedHaveBase");
    }
    if (a.derivedBases != b.derivedBases) {
        Note("derivedBases");
    }
    if (a.xformBase != b.xformBase) {
        Note("xformBase");
    }
    bool nativeDiffer = a.nativeFrames.size() != b.nativeFrames.size();
    for (size_t i = 0; !nativeDiffer && i < a.nativeFrames.size(); ++i) {
        nativeDiffer =
            a.nativeFrames[i].points != b.nativeFrames[i].points ||
            a.nativeFrames[i].flags != b.nativeFrames[i].flags;
    }
    if (nativeDiffer) {
        Note("nativeFrames");
    }
    if (a.propertyPaths != b.propertyPaths) {
        Note("propertyPaths");
    }
    bool propertyDiffer = a.propertyValues.size() != b.propertyValues.size();
    for (size_t i = 0; !propertyDiffer && i < a.propertyValues.size(); ++i) {
        propertyDiffer = !_BinaryPropertyValueEqual(a.propertyValues[i],
                                                    b.propertyValues[i]);
    }
    if (propertyDiffer) {
        Note("propertyValues");
    }
    if (a.arrayWeights != b.arrayWeights) {
        Note("arrayWeights");
    }
    if (a.arrayTranslationOffsets != b.arrayTranslationOffsets) {
        Note("arrayTranslationOffsets");
    }
    if (a.arrayRotationOffsets != b.arrayRotationOffsets) {
        Note("arrayRotationOffsets");
    }
    if (a.arrayOk != b.arrayOk) {
        Note("arrayOk");
    }
    if (a.arrayPoleWeights != b.arrayPoleWeights) {
        Note("arrayPoleWeights");
    }
    if (a.arrayPoleOk != b.arrayPoleOk) {
        Note("arrayPoleOk");
    }
    if (a.deltaBaseMatrix != b.deltaBaseMatrix) {
        Note("deltaBaseMatrix");
    }
    if (a.deltaBaseOk != b.deltaBaseOk) {
        Note("deltaBaseOk");
    }
    bool packetsDiffer = a.weightPackets.size() != b.weightPackets.size();
    for (size_t i = 0; !packetsDiffer && i < a.weightPackets.size(); ++i) {
        packetsDiffer = !_BinaryWeightPacketEqual(a.weightPackets[i],
                                                  b.weightPackets[i]);
    }
    if (packetsDiffer) {
        Note("weightPackets");
    }
    if (a.currentPhaseWeights != b.currentPhaseWeights) {
        Note("currentPhaseWeights");
    }
    if (a.blendWeights != b.blendWeights) {
        Note("blendWeights");
    }
    if (a.blendActivations != b.blendActivations) {
        Note("blendActivations");
    }
    if (a.blendPoints != b.blendPoints) {
        Note("blendPoints");
    }
    if (a.derivedBlendWeights != b.derivedBlendWeights) {
        Note("derivedBlendWeights");
    }
    if (a.derivedBlendActivations != b.derivedBlendActivations) {
        Note("derivedBlendActivations");
    }
    if (a.derivedBlendPoints != b.derivedBlendPoints) {
        Note("derivedBlendPoints");
    }
    bool refusedDiffer =
        a.refusedLayouts.size() != b.refusedLayouts.size();
    for (size_t i = 0; !refusedDiffer && i < a.refusedLayouts.size(); ++i) {
        refusedDiffer = !_BinaryRefusedLayoutEqual(a.refusedLayouts[i],
                                                   b.refusedLayouts[i]);
    }
    if (refusedDiffer) {
        Note("refusedLayouts");
    }
    if (a.revisionDefaultWeights != b.revisionDefaultWeights) {
        Note("revisionDefaultWeights");
    }
    auto Packets2Differ =
        [&](const std::vector<std::vector<rigExec::RigExecWireWeightPacket>> &x,
            const std::vector<std::vector<rigExec::RigExecWireWeightPacket>> &y) {
            if (x.size() != y.size()) {
                return true;
            }
            for (size_t i = 0; i < x.size(); ++i) {
                if (x[i].size() != y[i].size()) {
                    return true;
                }
                for (size_t k = 0; k < x[i].size(); ++k) {
                    if (!_BinaryWeightPacketEqual(x[i][k], y[i][k])) {
                        return true;
                    }
                }
            }
            return false;
        };
    if (Packets2Differ(a.revisionPhasePackets, b.revisionPhasePackets)) {
        Note("revisionPhasePackets");
    }
    if (Packets2Differ(a.derivedPhasePackets, b.derivedPhasePackets)) {
        Note("derivedPhasePackets");
    }
    if (a.revisionAdjusters != b.revisionAdjusters) {
        Note("revisionAdjusters");
    }
    if (a.revisionAdjusterHave != b.revisionAdjusterHave) {
        Note("revisionAdjusterHave");
    }
    if (a.solverRibbonPoints != b.solverRibbonPoints) {
        Note("solverRibbonPoints");
    }
    if (a.constraintWeights != b.constraintWeights) {
        Note("constraintWeights");
    }
    if (a.constraintHaveWeight != b.constraintHaveWeight) {
        Note("constraintHaveWeight");
    }
    bool pathDiffer = a.pathReads.size() != b.pathReads.size();
    for (size_t i = 0; !pathDiffer && i < a.pathReads.size(); ++i) {
        pathDiffer = !_BinaryPathReadEqual(a.pathReads[i],
                                           b.pathReads[i]);
    }
    if (pathDiffer) {
        Note("pathReads");
    }
    return out;
}

/// Exact equality on two frame records, IGNORING the frame number: whether
/// the capture varies across frames is the property the varies-guard in
/// TestBake asserts, and the frame stamp varies by construction.
bool
_BinaryFrameInputsEqual(const rigExec::RigExecWireFrameInputs &a,
                        const rigExec::RigExecWireFrameInputs &b)
{
    if (a.uids.size() != b.uids.size() ||
        a.values.size() != b.values.size()) {
        return false;
    }
    for (size_t i = 0; i < a.uids.size(); ++i) {
        if (a.uids[i] != b.uids[i] ||
            !_BinaryWireValueEqual(a.values[i], b.values[i])) {
            return false;
        }
    }
    if (a.chainHaveBase != b.chainHaveBase ||
        a.chainBases != b.chainBases ||
        a.derivedHaveBase != b.derivedHaveBase ||
        a.derivedBases != b.derivedBases ||
        a.xformBase != b.xformBase ||
        a.propertyPaths != b.propertyPaths ||
        a.arrayWeights != b.arrayWeights ||
        a.arrayTranslationOffsets != b.arrayTranslationOffsets ||
        a.arrayRotationOffsets != b.arrayRotationOffsets ||
        a.arrayOk != b.arrayOk ||
        a.arrayPoleWeights != b.arrayPoleWeights ||
        a.arrayPoleOk != b.arrayPoleOk ||
        a.deltaBaseMatrix != b.deltaBaseMatrix ||
        a.deltaBaseOk != b.deltaBaseOk ||
        a.currentPhaseWeights != b.currentPhaseWeights) {
        return false;
    }
    if (a.blendWeights != b.blendWeights ||
        a.blendActivations != b.blendActivations ||
        a.blendPoints != b.blendPoints ||
        a.derivedBlendWeights != b.derivedBlendWeights ||
        a.derivedBlendActivations != b.derivedBlendActivations ||
        a.derivedBlendPoints != b.derivedBlendPoints ||
        a.revisionDefaultWeights != b.revisionDefaultWeights ||
        a.revisionAdjusters != b.revisionAdjusters ||
        a.revisionAdjusterHave != b.revisionAdjusterHave ||
        a.solverRibbonPoints != b.solverRibbonPoints ||
        a.constraintWeights != b.constraintWeights ||
        a.constraintHaveWeight != b.constraintHaveWeight) {
        return false;
    }
    if (a.refusedLayouts.size() != b.refusedLayouts.size()) {
        return false;
    }
    for (size_t i = 0; i < a.refusedLayouts.size(); ++i) {
        if (!_BinaryRefusedLayoutEqual(a.refusedLayouts[i],
                                       b.refusedLayouts[i])) {
            return false;
        }
    }
    auto Packets2Equal =
        [&](const std::vector<std::vector<rigExec::RigExecWireWeightPacket>> &x,
            const std::vector<std::vector<rigExec::RigExecWireWeightPacket>> &y) {
            if (x.size() != y.size()) {
                return false;
            }
            for (size_t i = 0; i < x.size(); ++i) {
                if (x[i].size() != y[i].size()) {
                    return false;
                }
                for (size_t k = 0; k < x[i].size(); ++k) {
                    if (!_BinaryWeightPacketEqual(x[i][k], y[i][k])) {
                        return false;
                    }
                }
            }
            return true;
        };
    if (!Packets2Equal(a.revisionPhasePackets, b.revisionPhasePackets) ||
        !Packets2Equal(a.derivedPhasePackets, b.derivedPhasePackets)) {
        return false;
    }
    if (a.nativeFrames.size() != b.nativeFrames.size()) {
        return false;
    }
    for (size_t i = 0; i < a.nativeFrames.size(); ++i) {
        if (a.nativeFrames[i].points != b.nativeFrames[i].points ||
            a.nativeFrames[i].flags != b.nativeFrames[i].flags) {
            return false;
        }
    }
    if (a.propertyValues.size() != b.propertyValues.size()) {
        return false;
    }
    for (size_t i = 0; i < a.propertyValues.size(); ++i) {
        if (!_BinaryPropertyValueEqual(a.propertyValues[i],
                                       b.propertyValues[i])) {
            return false;
        }
    }
    if (a.weightPackets.size() != b.weightPackets.size()) {
        return false;
    }
    for (size_t i = 0; i < a.weightPackets.size(); ++i) {
        if (!_BinaryWeightPacketEqual(a.weightPackets[i],
                                      b.weightPackets[i])) {
            return false;
        }
    }
    if (a.pathReads.size() != b.pathReads.size()) {
        return false;
    }
    for (size_t i = 0; i < a.pathReads.size(); ++i) {
        if (!_BinaryPathReadEqual(a.pathReads[i], b.pathReads[i])) {
            return false;
        }
    }
    return true;
}

}  // namespace
