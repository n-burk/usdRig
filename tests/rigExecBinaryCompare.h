// testRigExecBinary comparison: a baked .rigexec file, opened as its object
// form, against the live program it was baked from, field by field, and its
// static data against what the program's run at the bake time left behind.
// Included by tests/testRigExecBinary.cpp AFTER its CHECK macro: every
// comparison below reports through it. Numbers compare bit for bit: the bake
// copies values and the file preserves bits, with no arithmetic between.
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/rigEvaluator.h"
#include "rigExecBake/revisionReads.h"
#include "rigExecBinary/format.h"

#include "pxr/base/gf/vec2f.h"
#include "pxr/base/gf/vec3i.h"
#include "pxr/base/vt/array.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/timeCode.h"

#include <cstring>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace {

using _BinaryFile = rigExec::fb::RigExecWireFile;
using _BinaryRead = rigExec::fb::RigExecWireInput;

std::string
_BinaryText(const _BinaryFile &file, uint32_t id)
{
    return rigExec::RigExecFormatPathText(file, id);
}

template <class T>
bool
_BinarySame(const T &a, const T &b)
{
    return std::memcmp(&a, &b, sizeof(T)) == 0;
}

inline bool
_BinaryEq(double a, double b)
{
    return _BinarySame(a, b);
}

inline bool
_BinaryEq(float a, float b)
{
    return _BinarySame(a, b);
}

template <class A, class B>
bool
_BinaryEq(const A &a, const B &b)
{
    return a == static_cast<A>(b) && static_cast<B>(a) == b;
}

template <class A, class B>
void
_BinaryCheckEqual(const std::vector<A> &a, const std::vector<B> &b)
{
    CHECK(a.size() == b.size());
    const size_t count = std::min(a.size(), b.size());
    for (size_t i = 0; i < count; ++i) {
        CHECK(_BinaryEq(a[i], b[i]));
    }
}

void
_BinaryCheckMatrix(const GfMatrix4d &m,
                   const rigExec::RigExecWireMatrix4d &w)
{
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            CHECK(_BinaryEq(m[r][c], w[size_t(r * 4 + c)]));
        }
    }
}

void
_BinaryCheckVec(const GfVec3d &v, const rigExec::RigExecWireVec3d &w)
{
    CHECK(_BinaryEq(v[0], w[0]) && _BinaryEq(v[1], w[1]) &&
          _BinaryEq(v[2], w[2]));
}

void
_BinaryCheckFrame(const rigExec::RigExecPointFrame &f,
                  const rigExec::RigExecWireFrame &w)
{
    for (size_t i = 0; i < 4; ++i) {
        _BinaryCheckVec(f.points[i], w.points[i]);
    }
    CHECK(f.flags == w.flags);
}

void
_BinaryCompareLandmarkSet(const std::array<GfVec3d, 4> &live,
                          const rigExec::RigExecWireLandmarks &wire)
{
    for (size_t i = 0; i < 4; ++i) {
        _BinaryCheckVec(live[i], wire[i]);
    }
}

/// \p live points against \p wire, bit for bit.
template <class Points>
bool
_BinarySamePoints(const Points &live,
                  const std::vector<rigExec::RigExecWireVec3f> &wire)
{
    if (size_t(live.size()) != wire.size()) {
        return false;
    }
    size_t i = 0;
    for (const GfVec3f &p : live) {
        if (!_BinaryEq(p[0], wire[i][0]) || !_BinaryEq(p[1], wire[i][1]) ||
            !_BinaryEq(p[2], wire[i][2])) {
            return false;
        }
        ++i;
    }
    return true;
}

/// The points of vec3f pool entry \p id, or null when out of range.
const std::vector<rigExec::RigExecWireVec3f> *
_BinaryPoolPoints(const _BinaryFile &file, uint32_t id)
{
    return id < file.vec3fArrays.size() ? &file.vec3fArrays[id].v : nullptr;
}

/// The name of slot \p slot, or "" out of range.
std::string
_BinarySlotName(const _BinaryFile &file, uint32_t slot)
{
    return slot < file.inputs.size()
               ? _BinaryText(file, file.inputs[slot].name())
               : std::string();
}

/// What one comparison counted, for the per-fixture summary line.
struct _BinaryCompareStats {
    size_t reads = 0;
    size_t topologies = 0;
    size_t droppedEntries = 0;
    size_t samples = 0;
    size_t bases = 0;
    size_t readRows = 0;
    size_t valueRows = 0;
    size_t enumeratedKeys = 0;
    size_t defaults = 0;
    size_t overrideNumbers = 0;
    size_t propertyChains = 0;
    size_t oracleFacts = 0;
};

_BinaryCompareStats *_binaryStats = nullptr;

// -- Reads -----------------------------------------------------------------

/// A registered program read against its file read: Baked mode, the
/// override number, Varying and LongWay as the program routes it, ViaChain
/// exactly when the walk crosses a chain target, the walk headed by the
/// input's head, and the hop a pinned query reads selected.
template <class T>
void
_BinaryCompareRoute(const rigExec::RigExecBakedInput<T> &live,
                    const _BinaryRead &wire, const _BinaryFile &file)
{
    using Flags = rigExec::fb::InputReadFlags;
    CHECK(wire.mode == rigExec::fb::ReadMode::Baked);
    CHECK(wire.overrideIndex == live.overrideIndex);
    const bool varying = (wire.flags & uint8_t(Flags::Varying)) != 0;
    const bool longWay = (wire.flags & uint8_t(Flags::LongWay)) != 0;
    const bool viaChain = (wire.flags & uint8_t(Flags::ViaChain)) != 0;
    CHECK(varying == live.varying);
    CHECK(longWay == bool(live.resolvedAttr));
    if (live.head) {
        CHECK(!wire.walk.empty() &&
              _BinarySlotName(file, wire.walk[0]) ==
                  live.head.GetPath().GetString());
    } else {
        CHECK(wire.walk.empty());
    }
    bool crosses = false;
    for (const uint32_t slot : wire.walk) {
        crosses = crosses ||
                  (slot < file.inputs.size() && file.inputs[slot].chain() >= 0);
    }
    CHECK(viaChain == crosses);
    int selected = -1;
    if (live.varying && !live.resolvedAttr && live.query.IsValid()) {
        const std::string pinned =
            live.query.GetAttribute().GetPath().GetString();
        for (size_t k = 0; k < wire.walk.size(); ++k) {
            if (_BinarySlotName(file, wire.walk[k]) == pinned) {
                selected = int(k);
                break;
            }
        }
        CHECK(selected >= 0);
    }
    CHECK(int(wire.selected) == selected);
}

/// The constant of \p wire, checked to be of \p tag; null when it is not.
const rigExec::fb::RigExecWireValue *
_BinaryConstant(const _BinaryFile &file, const _BinaryRead &wire,
                rigExec::fb::InputTag tag)
{
    CHECK(wire.tag == tag);
    CHECK(wire.constant < file.values.size());
    if (wire.tag != tag || wire.constant >= file.values.size()) {
        return nullptr;
    }
    const rigExec::fb::RigExecWireValue &value = file.values[wire.constant];
    CHECK(value.tag == tag);
    return value.tag == tag ? &value : nullptr;
}

uint64_t
_BinaryBits(double d)
{
    uint64_t bits = 0;
    std::memcpy(&bits, &d, sizeof(d));
    return bits;
}

uint64_t
_BinaryBits(float f)
{
    uint32_t bits = 0;
    std::memcpy(&bits, &f, sizeof(f));
    return bits;
}

bool
_BinaryConstantIs(const rigExec::fb::RigExecWireValue &c, double live,
                  const _BinaryFile &)
{
    return c.bits == _BinaryBits(live);
}

bool
_BinaryConstantIs(const rigExec::fb::RigExecWireValue &c, float live,
                  const _BinaryFile &)
{
    return c.bits == _BinaryBits(live);
}

bool
_BinaryConstantIs(const rigExec::fb::RigExecWireValue &c, bool live,
                  const _BinaryFile &)
{
    return c.bits == (live ? 1u : 0u);
}

bool
_BinaryConstantIs(const rigExec::fb::RigExecWireValue &c, int live,
                  const _BinaryFile &)
{
    return c.bits == uint64_t(uint32_t(live));
}

bool
_BinaryConstantIs(const rigExec::fb::RigExecWireValue &c,
                  const GfMatrix4d &live, const _BinaryFile &)
{
    if (!c.matrix) {
        return false;
    }
    for (int r = 0; r < 4; ++r) {
        for (int col = 0; col < 4; ++col) {
            if (!_BinaryEq(live[r][col], (*c.matrix)[size_t(r * 4 + col)])) {
                return false;
            }
        }
    }
    return true;
}

bool
_BinaryConstantIs(const rigExec::fb::RigExecWireValue &c,
                  const TfToken &live, const _BinaryFile &file)
{
    return c.bits <= 0xffffffffull &&
           _BinaryText(file, uint32_t(c.bits)) == live.GetString();
}

bool
_BinaryConstantIs(const rigExec::fb::RigExecWireValue &c,
                  const GfVec3d &live, const _BinaryFile &)
{
    return c.vec3d && _BinaryEq(live[0], (*c.vec3d)[0]) &&
           _BinaryEq(live[1], (*c.vec3d)[1]) &&
           _BinaryEq(live[2], (*c.vec3d)[2]);
}

template <class T>
constexpr rigExec::fb::InputTag
_BinaryTagOf()
{
    using Tag = rigExec::fb::InputTag;
    if constexpr (std::is_same_v<T, double>) {
        return Tag::Double;
    } else if constexpr (std::is_same_v<T, float>) {
        return Tag::Float;
    } else if constexpr (std::is_same_v<T, bool>) {
        return Tag::Bool;
    } else if constexpr (std::is_same_v<T, int>) {
        return Tag::Int;
    } else if constexpr (std::is_same_v<T, GfMatrix4d>) {
        return Tag::Matrix4d;
    } else if constexpr (std::is_same_v<T, TfToken>) {
        return Tag::Token;
    } else {
        static_assert(std::is_same_v<T, GfVec3d>, "no input tag");
        return Tag::Vec3d;
    }
}

template <class T>
void
_BinaryCompareInput(const rigExec::RigExecBakedInput<T> &live,
                    const _BinaryRead *wire, const _BinaryFile &file)
{
    CHECK(wire);
    if (!wire) {
        return;
    }
    if (_binaryStats) {
        ++_binaryStats->reads;
    }
    const rigExec::fb::RigExecWireValue *constant =
        _BinaryConstant(file, *wire, _BinaryTagOf<T>());
    if (constant) {
        CHECK(_BinaryConstantIs(*constant, live.constant, file));
    }
    _BinaryCompareRoute(live, *wire, file);
}

template <class T>
void
_BinaryCompareInput(const rigExec::RigExecBakedInput<T> &live,
                    const std::unique_ptr<_BinaryRead> &wire,
                    const _BinaryFile &file)
{
    _BinaryCompareInput(live, wire.get(), file);
}

template <class T>
void
_BinaryCompareInput(const rigExec::RigExecBakedInput<T> &live,
                    const _BinaryRead &wire, const _BinaryFile &file)
{
    _BinaryCompareInput(live, &wire, file);
}

/// A geometry read through RigExecResolvedInputs: Resolved, a Float with
/// \p fallback as its constant, its walk headed by \p head (empty without
/// one).
void
_BinaryCompareResolvedFloat(const UsdAttribute &head, float fallback,
                            const _BinaryRead *wire, const _BinaryFile &file)
{
    CHECK(wire);
    if (!wire) {
        return;
    }
    if (_binaryStats) {
        ++_binaryStats->reads;
    }
    CHECK(wire->mode == rigExec::fb::ReadMode::Resolved);
    CHECK(wire->overrideIndex == -1 && wire->selected == -1);
    const rigExec::fb::RigExecWireValue *constant =
        _BinaryConstant(file, *wire, rigExec::fb::InputTag::Float);
    if (constant) {
        CHECK(constant->bits == _BinaryBits(fallback));
    }
    if (head) {
        CHECK(!wire->walk.empty() &&
              _BinarySlotName(file, wire->walk[0]) ==
                  head.GetPath().GetString());
    } else {
        CHECK(wire->walk.empty());
    }
}

// -- Slot inventory, constants, steps, clusters, cones ---------------------

void
_BinaryCompareSlotMeta(const rigExec::RigExecBakedProgramImpl &program,
                       const rigExec::fb::RigExecWireSlotMeta &meta,
                       const _BinaryFile &file)
{
    CHECK(program.paths.size() == meta.paths.size());
    for (size_t i = 0; i < meta.paths.size() && i < program.paths.size();
         ++i) {
        CHECK(program.paths[i].GetString() ==
              _BinaryText(file, meta.paths[i]));
    }
    CHECK(program.slotKind.size() == meta.slotKind.size());
    for (size_t i = 0; i < meta.slotKind.size() && i < program.slotKind.size();
         ++i) {
        CHECK(uint8_t(program.slotKind[i]) == uint8_t(meta.slotKind[i]));
    }
    _BinaryCheckEqual(program.parent, meta.parent);
    _BinaryCheckEqual(program.propParent, meta.propParent);
    _BinaryCheckEqual(program.xformSlots, meta.xformSlots);
    CHECK(program.xformPrimsBySlot.size() == meta.xformPaths.size());
    for (size_t i = 0;
         i < meta.xformPaths.size() && i < program.xformPrimsBySlot.size();
         ++i) {
        CHECK(program.xformPrimsBySlot[i].GetPath().GetString() ==
              _BinaryText(file, meta.xformPaths[i]));
    }
    _BinaryCheckEqual(program.jointSlots, meta.jointSlots);
    CHECK(program.jointPaths.size() == meta.jointPaths.size());
    for (size_t i = 0;
         i < meta.jointPaths.size() && i < program.jointPaths.size(); ++i) {
        CHECK(program.jointPaths[i].GetString() ==
              _BinaryText(file, meta.jointPaths[i]));
    }
    _BinaryCheckEqual(program.controlSlots, meta.controlSlots);
    CHECK(program.controlPaths.size() == meta.controlPaths.size());
    for (size_t i = 0;
         i < meta.controlPaths.size() && i < program.controlPaths.size();
         ++i) {
        CHECK(program.controlPaths[i].GetString() ==
              _BinaryText(file, meta.controlPaths[i]));
    }
    CHECK(program.solverArrays.size() == meta.solverArrayPaths.size());
    CHECK(program.solverArrays.size() == meta.solverArrayElements.size());
    for (size_t i = 0; i < meta.solverArrayPaths.size() &&
                       i < meta.solverArrayElements.size() &&
                       i < program.solverArrays.size();
         ++i) {
        CHECK(program.solverArrays[i].first.GetString() ==
              _BinaryText(file, meta.solverArrayPaths[i]));
        CHECK(int64_t(program.solverArrays[i].second) ==
              int64_t(meta.solverArrayElements[i]));
    }
    _BinaryCheckEqual(program.jointPublishOrder, meta.jointPublishOrder);
    _BinaryCheckEqual(program.controlPublishOrder, meta.controlPublishOrder);
    _BinaryCheckEqual(program.solverPublishOrder, meta.solverPublishOrder);
    CHECK(program.jointPathsAscending == meta.jointPathsAscending);
    CHECK(program.controlPathsAscending == meta.controlPathsAscending);
    CHECK(program.solverArraysAscending == meta.solverArraysAscending);
    CHECK(program.needFinal.size() == meta.needFinal.size());
    for (size_t i = 0;
         i < meta.needFinal.size() && i < program.needFinal.size(); ++i) {
        CHECK((program.needFinal[i] != 0) == (meta.needFinal[i] != 0));
    }
    CHECK(program.needBase.size() == meta.needBase.size());
    for (size_t i = 0; i < meta.needBase.size() && i < program.needBase.size();
         ++i) {
        CHECK((program.needBase[i] != 0) == (meta.needBase[i] != 0));
    }
}

void
_BinaryCompareMatrices(const std::vector<GfMatrix4d> &live,
                       const std::vector<rigExec::RigExecWireMatrix4d> &wire)
{
    CHECK(live.size() == wire.size());
    for (size_t i = 0; i < live.size() && i < wire.size(); ++i) {
        _BinaryCheckMatrix(live[i], wire[i]);
    }
}

template <class Flags, class Bytes>
void
_BinaryCompareFlags(const Flags &live, const Bytes &wire)
{
    CHECK(live.size() == wire.size());
    for (size_t i = 0; i < live.size() && i < wire.size(); ++i) {
        CHECK((live[i] != 0) == (wire[i] != 0));
    }
}

void
_BinaryCompareConstants(const rigExec::RigExecBakedProgramImpl &program,
                        const rigExec::fb::RigExecWireConstants &constants,
                        const _BinaryFile &file)
{
    _BinaryCompareMatrices(program.restM, constants.restM);
    CHECK(program.restPts.size() == constants.restPts.size());
    for (size_t i = 0;
         i < constants.restPts.size() && i < program.restPts.size(); ++i) {
        _BinaryCompareLandmarkSet(program.restPts[i], constants.restPts[i]);
    }
    CHECK(program.restFrames.size() == constants.restFrames.size());
    for (size_t i = 0;
         i < constants.restFrames.size() && i < program.restFrames.size();
         ++i) {
        _BinaryCheckFrame(program.restFrames[i], constants.restFrames[i]);
    }
    _BinaryCompareMatrices(program.selfD, constants.selfD);
    _BinaryCompareMatrices(program.parentDinv, constants.parentDinv);
    CHECK(program.rotOrder.size() == constants.rotOrder.size());
    for (size_t i = 0;
         i < constants.rotOrder.size() && i < program.rotOrder.size(); ++i) {
        CHECK(program.rotOrder[i].GetString() ==
              _BinaryText(file, constants.rotOrder[i]));
    }
    _BinaryCompareMatrices(program.restRoundTrip, constants.restRoundTrip);
    _BinaryCompareMatrices(program.defaultRoundTrip,
                           constants.defaultRoundTrip);
    _BinaryCompareFlags(program.posedAuthored, constants.posedAuthored);
    _BinaryCompareMatrices(program.posedAuthoredM, constants.posedAuthoredM);
    _BinaryCompareFlags(program.noScaleAvars, constants.noScaleAvars);
    CHECK(program.rotationSign.size() == constants.rotationSign.size());
    for (size_t i = 0; i < constants.rotationSign.size() &&
                       i < program.rotationSign.size();
         ++i) {
        CHECK(uint8_t(program.rotationSign[i]) == constants.rotationSign[i]);
    }
    _BinaryCheckEqual(program.avarConstants, constants.avarConstants);
}

void
_BinaryCompareRanges(const std::vector<rigExec::RigExecBakedSlotRange> &live,
                     const std::vector<rigExec::fb::SlotRange> &wire)
{
    CHECK(live.size() == wire.size());
    for (size_t r = 0; r < live.size() && r < wire.size(); ++r) {
        CHECK(uint8_t(live[r].domain) == uint8_t(wire[r].domain()));
        CHECK(int64_t(live[r].begin) == int64_t(wire[r].begin()));
        CHECK(int64_t(live[r].end) == int64_t(wire[r].end()));
    }
}

/// Labels are not stored: the runtime rebuilds them from these tables, and
/// testRigExecRuntimeLoader holds every rebuilt label to the program's.
void
_BinaryCompareSteps(const rigExec::RigExecBakedProgramImpl &program,
                    const std::vector<rigExec::fb::RigExecWireStep> &steps)
{
    CHECK(program.steps.size() == steps.size());
    for (size_t i = 0; i < steps.size() && i < program.steps.size(); ++i) {
        const rigExec::RigExecBakedStep &live = program.steps[i];
        const rigExec::fb::RigExecWireStep &wire = steps[i];
        CHECK(uint8_t(live.kind) == uint8_t(wire.kind));
        CHECK(int64_t(live.object) == int64_t(wire.object));
        CHECK(int64_t(live.part) == int64_t(wire.part));
        _BinaryCompareRanges(live.reads, wire.reads);
        _BinaryCompareRanges(live.writes, wire.writes);
        _BinaryCheckEqual(live.preds, wire.preds);
        _BinaryCheckEqual(live.succs, wire.succs);
        CHECK(live.isSource == wire.isSource);
        CHECK(live.externalReads == wire.externalReads);
        CHECK(live.varyingInputs == wire.varyingInputs);
        CHECK(live.resolvedInputReads == wire.resolvedInputReads);
        _BinaryCheckEqual(live.overrideInputs, wire.overrideInputs);
        CHECK(int64_t(live.cluster) == int64_t(wire.cluster));
        CHECK(int64_t(live.level) == int64_t(wire.level));
        CHECK(_BinaryEq(live.sizeUnits, wire.sizeUnits));
        CHECK(_BinaryEq(live.cost, wire.cost));
        CHECK(live.maxDiagnostics == size_t(wire.maxDiagnostics));
    }
}

void
_BinaryCompareClustering(
    const rigExec::RigExecBakedProgramImpl &program,
    const rigExec::fb::RigExecWireClustering &clustering)
{
    const auto &live = program.clustering;
    CHECK(live.clusters.size() == clustering.clusters.size());
    for (size_t i = 0;
         i < clustering.clusters.size() && i < live.clusters.size(); ++i) {
        _BinaryCheckEqual(live.clusters[i].members,
                          clustering.clusters[i].members);
        _BinaryCheckEqual(live.clusters[i].preds,
                          clustering.clusters[i].preds);
        _BinaryCheckEqual(live.clusters[i].succs,
                          clustering.clusters[i].succs);
        CHECK(_BinaryEq(live.clusters[i].cost, clustering.clusters[i].cost));
        CHECK(int64_t(live.clusters[i].level) ==
              int64_t(clustering.clusters[i].level));
    }
    _BinaryCheckEqual(live.clusterOf, clustering.clusterOf);
    CHECK(_BinaryEq(live.grainUs, clustering.grainUs));
    CHECK(_BinaryEq(live.serialCost, clustering.serialCost));
    CHECK(_BinaryEq(live.criticalPathCost, clustering.criticalPathCost));
}

void
_BinaryCompareClusterSet(const rigExec::RigExecBakedClusterSet &live,
                         const rigExec::fb::RigExecWireClusterSet *wire,
                         size_t clusters)
{
    CHECK(wire);
    if (!wire) {
        return;
    }
    CHECK(size_t(wire->clusters) == clusters);
    CHECK(live.words == wire->words);
}

template <class Rows>
void
_BinaryCompareLists(const Rows &live,
                    const std::vector<rigExec::fb::RigExecWireIntList> &wire)
{
    CHECK(live.size() == wire.size());
    for (size_t i = 0; i < live.size() && i < wire.size(); ++i) {
        _BinaryCheckEqual(live[i], wire[i].v);
    }
}

/// The nine cone lists the file keeps.
void
_BinaryCompareCones(const rigExec::RigExecBakedProgramImpl &program,
                    const rigExec::fb::RigExecWireCones &cones)
{
    const size_t clusters = program.clustering.clusters.size();
    CHECK(program.cones.cone.size() == cones.cone.size());
    for (size_t i = 0; i < cones.cone.size() && i < program.cones.cone.size();
         ++i) {
        _BinaryCompareClusterSet(program.cones.cone[i], &cones.cone[i],
                                 clusters);
    }
    _BinaryCompareClusterSet(program.cones.always, cones.always.get(),
                             clusters);
    _BinaryCompareClusterSet(program.cones.poseClusters,
                             cones.poseClusters.get(), clusters);
    _BinaryCheckEqual(program.cones.avarCluster, cones.avarCluster);
    _BinaryCompareLists(program.cones.chainBaseClusters,
                        cones.chainBaseClusters);
    _BinaryCompareLists(program.cones.revisionClusters,
                        cones.revisionClusters);
    _BinaryCheckEqual(program.cones.revisionStaticCluster,
                      cones.revisionStaticCluster);
    _BinaryCheckEqual(program.cones.varyingSteps, cones.varyingSteps);
    _BinaryCheckEqual(program.cones.overrideSteps, cones.overrideSteps);
}

// -- The pose tables -------------------------------------------------------

void
_BinaryCompareRbf(const rigExec::RigExecRbfSolver &live,
                  const rigExec::fb::RigExecWireRbf *wire)
{
    CHECK(wire);
    if (!wire) {
        return;
    }
    CHECK(live.GetPoses().size() == wire->poses.size());
    for (size_t i = 0;
         i < wire->poses.size() && i < live.GetPoses().size(); ++i) {
        _BinaryCheckVec(live.GetPoses()[i], wire->poses[i]);
    }
    CHECK(live.GetTranslations().size() == wire->translations.size());
    for (size_t i = 0; i < wire->translations.size() &&
                       i < live.GetTranslations().size();
         ++i) {
        _BinaryCheckVec(live.GetTranslations()[i], wire->translations[i]);
    }
    CHECK(live.GetPoseTypes().size() == wire->poseTypes.size());
    for (size_t i = 0;
         i < wire->poseTypes.size() && i < live.GetPoseTypes().size(); ++i) {
        CHECK(uint8_t(live.GetPoseTypes()[i]) == wire->poseTypes[i]);
    }
    _BinaryCheckVec(live.GetTwistAxis(), wire->twistAxis);
    CHECK(uint8_t(live.GetKernel()) == wire->kernel);
    CHECK(_BinaryEq(live.GetRadius(), wire->radius));
    CHECK(_BinaryEq(live.GetTranslationRadius(), wire->translationRadius));
    _BinaryCheckEqual(live.GetRadii(), wire->radii);
    _BinaryCheckEqual(live.GetTranslationRadii(), wire->translationRadii);
    CHECK(_BinaryEq(live.GetRegularization(), wire->regularization));
    CHECK(live.GetNormalize() == wire->normalize);
    CHECK(live.GetEnableRotation() == wire->enableRotation);
    CHECK(live.GetEnableTranslation() == wire->enableTranslation);
    CHECK(live.GetRegularizedSingular() == wire->regularizedSingular);
    CHECK(live.GetWeights().size() == wire->weights.size());
    for (size_t i = 0;
         i < wire->weights.size() && i < live.GetWeights().size(); ++i) {
        _BinaryCheckEqual(live.GetWeights()[i], wire->weights[i].v);
    }
}

void
_BinaryCompareSolver(const rigExec::RigExecBakedProgramImpl::Solver &live,
                     const rigExec::fb::RigExecWireSolver &wire,
                     const _BinaryFile &file)
{
    CHECK(live.path.GetString() == _BinaryText(file, wire.path));
    CHECK(live.type.GetString() == _BinaryText(file, wire.type));
    _BinaryCheckEqual(live.restSlots, wire.restSlots);
    CHECK(live.restRefs.size() == wire.restRefs.size());
    for (size_t i = 0;
         i < wire.restRefs.size() && i < live.restRefs.size(); ++i) {
        CHECK(int64_t(live.restRefs[i].first) ==
              int64_t(wire.restRefs[i].first));
        CHECK(int64_t(live.restRefs[i].second) ==
              int64_t(wire.restRefs[i].second));
    }
    CHECK(live.restIsLive.size() == wire.restIsLive.size());
    for (size_t i = 0;
         i < wire.restIsLive.size() && i < live.restIsLive.size(); ++i) {
        CHECK(bool(live.restIsLive[i]) == (wire.restIsLive[i] != 0));
    }
    _BinaryCheckEqual(live.restReads, wire.restReads);
    CHECK(live.hasLiveRest == wire.hasLiveRest);
    CHECK(live.jointRests.size() == wire.jointRests.size());
    for (size_t i = 0;
         i < wire.jointRests.size() && i < live.jointRests.size(); ++i) {
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
    for (size_t i = 0;
         i < wire.controlRests.size() && i < live.controlRests.size(); ++i) {
        _BinaryCompareLandmarkSet(live.controlRests[i],
                                  wire.controlRests[i]);
    }
    CHECK(int64_t(live.start) == int64_t(wire.start));
    _BinaryCompareLandmarkSet(live.startRest, wire.startRest);
    CHECK(live.startRead == wire.startRead);
    CHECK(int64_t(live.root) == int64_t(wire.root));
    CHECK(int64_t(live.mid) == int64_t(wire.mid));
    CHECK(int64_t(live.end) == int64_t(wire.end));
    CHECK(int64_t(live.pole) == int64_t(wire.pole));
    CHECK(wire.ikRests.size() == 3);
    for (size_t k = 0; k < 3 && k < wire.ikRests.size(); ++k) {
        _BinaryCompareLandmarkSet(live.ikRests[k], wire.ikRests[k]);
    }
    CHECK(_BinaryEq(live.ikParams.upperLength, wire.ikParams.upperLength));
    CHECK(_BinaryEq(live.ikParams.lowerLength, wire.ikParams.lowerLength));
    CHECK(_BinaryEq(live.ikParams.stretch, wire.ikParams.stretch));
    CHECK(_BinaryEq(live.ikParams.softness, wire.ikParams.softness));
    CHECK(_BinaryEq(live.ikParams.preferredBendRadians,
                    wire.ikParams.preferredBendRadians));
    _BinaryCompareInput(live.bend, wire.bend, file);
    _BinaryCompareInput(live.upperOffset, wire.upperOffset, file);
    _BinaryCompareInput(live.lowerOffset, wire.lowerOffset, file);
    _BinaryCompareInput(live.stretch, wire.stretch, file);
    _BinaryCompareInput(live.softness, wire.softness, file);
    CHECK(_BinaryEq(live.upperLengthBase, wire.upperLengthBase));
    CHECK(_BinaryEq(live.lowerLengthBase, wire.lowerLengthBase));
    _BinaryCompareInput(live.ikSpace, wire.ikSpace, file);
    CHECK(int64_t(live.spaceSlot) == int64_t(wire.spaceSlot));
    _BinaryCompareLandmarkSet(live.spaceRest, wire.spaceRest);
    CHECK(wire.spaceRead == uint32_t(std::max(live.spaceRead, 0)));
    CHECK(int64_t(live.inA) == int64_t(wire.inA));
    CHECK(int64_t(live.inB) == int64_t(wire.inB));
    _BinaryCompareInput(live.blendWeight, wire.blendWeight, file);
    CHECK(uint8_t(live.scaleMode) == wire.scaleMode);
    CHECK(live.blendRotationRejected == wire.blendRotationRejected);
    CHECK(wire.splineRest);
    if (wire.splineRest) {
        const rigExec::fb::RigExecWireSplineIkRest &rest = *wire.splineRest;
        _BinaryCompareLandmarkSet(live.splineRest.cvs, rest.cvs);
        _BinaryCheckFrame(live.splineRest.rootControl, rest.rootControl);
        _BinaryCheckFrame(live.splineRest.midControl, rest.midControl);
        _BinaryCheckFrame(live.splineRest.endControl, rest.endControl);
        CHECK(live.splineRest.joints.size() == rest.joints.size());
        for (size_t i = 0;
             i < rest.joints.size() && i < live.splineRest.joints.size();
             ++i) {
            _BinaryCheckFrame(live.splineRest.joints[i], rest.joints[i]);
        }
        _BinaryCheckEqual(live.splineRest.segmentLengths,
                          rest.segmentLengths);
        CHECK(_BinaryEq(live.splineRest.restArcLength, rest.restArcLength));
        _BinaryCheckEqual(live.splineRest.volumeWeights, rest.volumeWeights);
    }
    CHECK(_BinaryEq(live.splineParams.preserveVolume,
                    wire.splineParams.preserveVolume));
    CHECK(_BinaryEq(live.splineParams.midFollowWeight,
                    wire.splineParams.midFollowWeight));
    CHECK(_BinaryEq(live.splineParams.roll, wire.splineParams.roll));
    CHECK(_BinaryEq(live.splineParams.twist, wire.splineParams.twist));
    CHECK(_BinaryEq(live.splineParams.minLengthRatio,
                    wire.splineParams.minLengthRatio));
    CHECK(live.splineParams.aimRootTangent ==
          wire.splineParams.aimRootTangent);
    CHECK(live.splineJointRests.size() == wire.splineJointRests.size());
    for (size_t i = 0; i < wire.splineJointRests.size() &&
                       i < live.splineJointRests.size();
         ++i) {
        _BinaryCompareLandmarkSet(live.splineJointRests[i],
                                  wire.splineJointRests[i]);
    }
    CHECK(uint64_t(live.splineCount) == wire.splineCount);
    _BinaryCompareInput(live.preserveVolume, wire.preserveVolume, file);
    _BinaryCompareInput(live.midFollowWeight, wire.midFollowWeight, file);
    _BinaryCompareInput(live.roll, wire.roll, file);
    _BinaryCompareInput(live.twist, wire.twist, file);
    _BinaryCompareInput(live.minLengthRatio, wire.minLengthRatio, file);
    CHECK(live.splineParamsVary == wire.splineParamsVary);
    _BinaryCompareLandmarkSet(live.twistStartRest, wire.twistStartRest);
    _BinaryCompareLandmarkSet(live.twistEndRest, wire.twistEndRest);
    _BinaryCheckEqual(live.twistWeights, wire.twistWeights);
    _BinaryCompareInput(live.twistTurns, wire.twistTurns, file);
    CHECK(live.ribbonPointsPath.GetString() ==
          _BinaryText(file, wire.ribbonPointsPath));
    CHECK(_BinarySamePoints(live.ribbonRestPoints, wire.ribbonRestPoints));
    // Static at the bake time: a varying driver's points as the run read
    // them, else the epoch constant.
    CHECK(_BinarySamePoints(live.ribbonPointsVarying
                                ? live.ribbonPoints
                                : live.ribbonConstantPoints,
                            wire.ribbonConstantPoints));
    _BinaryCompareInput(live.ribbonSampleCount, wire.ribbonSampleCount,
                        file);
    CHECK(live.outputs.size() == wire.outputs.size());
    for (size_t i = 0; i < wire.outputs.size() && i < live.outputs.size();
         ++i) {
        CHECK(int64_t(live.outputs[i].first) ==
              int64_t(wire.outputs[i].first));
        CHECK(int64_t(live.outputs[i].second) ==
              int64_t(wire.outputs[i].second));
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
    const rigExec::fb::RigExecWireConstraint &wire, const _BinaryFile &file)
{
    CHECK(live.path.GetString() == _BinaryText(file, wire.path));
    CHECK(live.type.GetString() == _BinaryText(file, wire.type));
    CHECK(live.weightObject.GetString() ==
          _BinaryText(file, wire.weightObject));
    // The envelope arm: a weight object and no points target, naming the
    // entry of that object.
    const bool envelope =
        !live.weightObject.IsEmpty() && live.pointsTarget.IsEmpty();
    CHECK(envelope == (wire.weightObjectIndex >= 0));
    if (wire.weightObjectIndex >= 0 && file.geometry &&
        size_t(wire.weightObjectIndex) < file.geometry->weightObjects.size()) {
        CHECK(_BinaryText(file, file.geometry
                                    ->weightObjects[size_t(
                                        wire.weightObjectIndex)]
                                    .path) ==
              live.weightObject.GetString());
    }
    CHECK(int64_t(live.target) == int64_t(wire.target));
    _BinaryCheckEqual(live.targetSlots, wire.targetSlots);
    _BinaryCompareFlags(live.snapshotTargets, wire.snapshotTargets);
    _BinaryCheckEqual(live.sources, wire.sources);
    _BinaryCheckEqual(live.sourceNatives, wire.sourceNatives);
    CHECK(live.sourcePaths.size() == wire.sourcePaths.size());
    for (size_t i = 0;
         i < wire.sourcePaths.size() && i < live.sourcePaths.size(); ++i) {
        CHECK(live.sourcePaths[i].GetString() ==
              _BinaryText(file, wire.sourcePaths[i]));
    }
    CHECK(int64_t(live.arrays) == int64_t(wire.arrays));
    _BinaryCompareInput(live.enabled, wire.enabled, file);
    _BinaryCompareInput(live.defaultWeight, wire.defaultWeight, file);
    _BinaryCompareInput(live.offset, wire.offset, file);
    _BinaryCompareInput(live.affectX, wire.affectX, file);
    _BinaryCompareInput(live.affectY, wire.affectY, file);
    _BinaryCompareInput(live.affectZ, wire.affectZ, file);
    _BinaryCompareInput(live.tX, wire.tX, file);
    _BinaryCompareInput(live.tY, wire.tY, file);
    _BinaryCompareInput(live.tZ, wire.tZ, file);
    _BinaryCompareInput(live.rX, wire.rX, file);
    _BinaryCompareInput(live.rY, wire.rY, file);
    _BinaryCompareInput(live.rZ, wire.rZ, file);
    _BinaryCompareInput(live.sX, wire.sX, file);
    _BinaryCompareInput(live.sY, wire.sY, file);
    _BinaryCompareInput(live.sZ, wire.sZ, file);
    CHECK(uint8_t(live.order) == wire.order);
    _BinaryCompareInput(live.aimVector, wire.aimVector, file);
    _BinaryCompareInput(live.upVector, wire.upVector, file);
    _BinaryCompareInput(live.rotationOffset, wire.rotationOffset, file);
    _BinaryCompareInput(live.worldUpVector, wire.worldUpVector, file);
    _BinaryCheckVec(live.aimAxisFallback, wire.aimAxisFallback);
    CHECK(live.aimVectorAuthored == wire.aimVectorAuthored);
    CHECK(live.preserveInputUp == wire.preserveInputUp);
    CHECK(live.worldUpType.GetString() ==
          _BinaryText(file, wire.worldUpType));
    _BinaryCheckVec(live.sceneUp, wire.sceneUp);
    CHECK(live.pointsTarget.GetString() ==
          _BinaryText(file, wire.pointsTarget));
    CHECK(live.deltaBasePath.GetString() ==
          _BinaryText(file, wire.deltaBasePath));
    CHECK(int64_t(live.deltaBase) == int64_t(wire.deltaBase));
    CHECK(int64_t(live.worldUpObject) == int64_t(wire.worldUpObject));
    CHECK(int64_t(live.worldUpNative) == int64_t(wire.worldUpNative));
    CHECK(live.worldUpPath.GetString() ==
          _BinaryText(file, wire.worldUpPath));
    CHECK(live.worldUpObjectNamed == wire.worldUpObjectNamed);
    CHECK(int64_t(live.spaceSlot) == int64_t(wire.spaceSlot));
    using Flags = rigExec::fb::ConstraintFlags;
    CHECK(live.blendShear ==
          ((wire.flags & uint8_t(Flags::BlendShear)) != 0));
    CHECK(live.worldUpRotationOnly ==
          ((wire.flags & uint8_t(Flags::WorldUpRotationOnly)) != 0));
    CHECK(live.radialBlend ==
          ((wire.flags & uint8_t(Flags::RadialBlend)) != 0));
    CHECK(live.snapshotAfter == wire.snapshotAfter);
    CHECK(live.singleChainIk == wire.singleChainIk);
    CHECK(uint8_t(live.ikMode) == wire.ikMode);
    CHECK(live.poleModeObject == wire.poleModeObject);
    CHECK(live.useAnimatedTs == wire.useAnimatedTs);
    _BinaryCompareFlags(live.ikRestLive, wire.ikRestLive);
    CHECK(int64_t(live.effector) == int64_t(wire.effector));
    CHECK(int64_t(live.effectorNative) == int64_t(wire.effectorNative));
    CHECK(live.effectorPath.GetString() ==
          _BinaryText(file, wire.effectorPath));
    _BinaryCheckEqual(live.poleObjects, wire.poleObjects);
    _BinaryCheckEqual(live.poleObjectNatives, wire.poleObjectNatives);
    _BinaryCompareInput(live.poleVector, wire.poleVector, file);
    _BinaryCompareInput(live.twistDegrees, wire.twistDegrees, file);
}

void
_BinaryCompareAncestorRead(
    const rigExec::RigExecBakedCommit::AncestorRead &live,
    const rigExec::RigExecWireAncestorRead &wire)
{
    CHECK(int64_t(live.slot) == int64_t(wire.slot));
    CHECK(live.fin == wire.fin);
    CHECK(live.base == wire.base);
}

void
_BinaryCompareAncestorLists(
    const std::vector<std::vector<rigExec::RigExecBakedCommit::AncestorRead>>
        &live,
    const std::vector<rigExec::fb::RigExecWireAncestorReadList> &wire)
{
    CHECK(live.size() == wire.size());
    for (size_t i = 0; i < live.size() && i < wire.size(); ++i) {
        CHECK(live[i].size() == wire[i].v.size());
        for (size_t k = 0; k < live[i].size() && k < wire[i].v.size(); ++k) {
            _BinaryCompareAncestorRead(live[i][k], wire[i].v[k]);
        }
    }
}

void
_BinaryCompareCommit(const rigExec::RigExecBakedCommit &live,
                     const rigExec::fb::RigExecWireCommit &wire,
                     const _BinaryFile &file)
{
    CHECK(live.moverPath.GetString() == _BinaryText(file, wire.moverPath));
    CHECK(live.solverOutput == wire.solverOutput);
    _BinaryCheckEqual(live.slots, wire.slots);
    CHECK(live.propagate.size() == wire.propagate.size());
    for (size_t i = 0;
         i < wire.propagate.size() && i < live.propagate.size(); ++i) {
        CHECK(int64_t(live.propagate[i].first) ==
              int64_t(wire.propagate[i].first));
        CHECK(int64_t(live.propagate[i].second) ==
              int64_t(wire.propagate[i].second));
    }
    _BinaryCheckEqual(live.closestPos, wire.closestPos);
    CHECK(live.split == wire.split);
    CHECK(int64_t(live.stagingBase) == int64_t(wire.stagingBase));
    CHECK(live.sources.size() == wire.sources.size());
    for (size_t i = 0; i < wire.sources.size() && i < live.sources.size();
         ++i) {
        _BinaryCheckFrame(live.sources[i].frame, wire.sources[i].frame);
        CHECK(_BinaryEq(live.sources[i].normalizedWeight,
                        wire.sources[i].normalizedWeight));
        _BinaryCheckVec(live.sources[i].translationOffset,
                        wire.sources[i].translationOffset);
        _BinaryCheckVec(live.sources[i].rotationOffsetDegrees,
                        wire.sources[i].rotationOffsetDegrees);
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
    for (size_t i = 0; i < wire.effectorAncestors.size() &&
                       i < live.effectorAncestors.size();
         ++i) {
        _BinaryCompareAncestorRead(live.effectorAncestors[i],
                                   wire.effectorAncestors[i]);
    }
    _BinaryCheckEqual(live.poleReads, wire.poleReads);
    _BinaryCompareAncestorLists(live.poleAncestors, wire.poleAncestors);
    CHECK(live.recordAfter == wire.recordAfter);
    CHECK(live.recordEveryTarget == wire.recordEveryTarget);
    _BinaryCompareAncestorLists(live.sourceAncestors, wire.sourceAncestors);
    CHECK(live.worldUpAncestors.size() == wire.worldUpAncestors.size());
    for (size_t i = 0; i < wire.worldUpAncestors.size() &&
                       i < live.worldUpAncestors.size();
         ++i) {
        _BinaryCompareAncestorRead(live.worldUpAncestors[i],
                                   wire.worldUpAncestors[i]);
    }
}

/// The switches: slots, sources, filters, space, every read version
/// exactly as Build bound it, the active read and the axis masks.
void
_BinaryCompareSpaceSwitches(
    const rigExec::RigExecBakedProgramImpl &program,
    const std::vector<rigExec::fb::RigExecWireSpaceSwitch> &switches,
    const _BinaryFile &file)
{
    using Version =
        rigExec::RigExecBakedProgramImpl::SpaceSwitch::FrameVersion;
    const auto same = [](const Version &live,
                         const rigExec::fb::RigExecWireFrameVersion *wire) {
        CHECK(wire);
        if (wire) {
            CHECK(int64_t(live.anchor) == int64_t(wire->anchor));
            _BinaryCheckEqual(live.recompose, wire->recompose);
        }
    };
    CHECK(switches.size() == program.spaceSwitches.size());
    for (size_t i = 0;
         i < switches.size() && i < program.spaceSwitches.size(); ++i) {
        const rigExec::RigExecBakedProgramImpl::SpaceSwitch &live =
            program.spaceSwitches[i];
        const rigExec::fb::RigExecWireSpaceSwitch &wire = switches[i];
        CHECK(int64_t(live.slot) == int64_t(wire.slot));
        _BinaryCheckEqual(live.sourceSlots, wire.sourceSlots);
        CHECK(live.filters.size() == wire.filters.size());
        for (size_t k = 0;
             k < live.filters.size() && k < wire.filters.size(); ++k) {
            CHECK(uint8_t(live.filters[k]) == wire.filters[k]);
        }
        _BinaryCheckVec(live.twistAxis, wire.twistAxis);
        CHECK(int64_t(live.spaceSlot) == int64_t(wire.spaceSlot));
        same(live.parentRead, wire.parentRead.get());
        same(live.spaceRead, wire.spaceRead.get());
        CHECK(live.sourceReads.size() == wire.sourceReads.size());
        for (size_t k = 0; k < live.sourceReads.size() &&
                           k < wire.sourceReads.size();
             ++k) {
            same(live.sourceReads[k], &wire.sourceReads[k]);
        }
        _BinaryCompareInput(live.activeInput, wire.active, file);
        for (size_t axis = 0; axis < 3; ++axis) {
            CHECK(live.affectTranslation[axis] ==
                  wire.affectTranslation[axis]);
            CHECK(live.affectRotation[axis] == wire.affectRotation[axis]);
            CHECK(live.affectScale[axis] == wire.affectScale[axis]);
        }
    }
}

void
_BinaryCompareDomainPose(const rigExec::RigExecBakedProgramImpl &program,
                         const rigExec::fb::RigExecWireDomainPose &pose,
                         const _BinaryFile &file)
{
    CHECK(program.ladders.size() == pose.ladders.size());
    for (size_t i = 0; i < pose.ladders.size() && i < program.ladders.size();
         ++i) {
        const auto &live = program.ladders[i];
        const rigExec::fb::RigExecWireLadder &wire = pose.ladders[i];
        _BinaryCompareInput(live.restSpace, wire.restSpace, file);
        _BinaryCompareInput(live.defaultSpace, wire.defaultSpace, file);
        _BinaryCompareInput(live.posedSpace, wire.posedSpace, file);
        CHECK(wire.restAvars.size() == 6 && wire.defaultAvars.size() == 6);
        for (size_t a = 0;
             a < 6 && a < wire.restAvars.size() && a < wire.defaultAvars.size();
             ++a) {
            _BinaryCompareInput(live.restAvars[a], wire.restAvars[a], file);
            _BinaryCompareInput(live.defaultAvars[a], wire.defaultAvars[a],
                                file);
        }
        _BinaryCompareInput(live.rotationOrder, wire.rotationOrder, file);
    }
    CHECK(program.ladderVarying == pose.ladderVarying);
    _BinaryCheckEqual(program.ladderOverrides, pose.ladderOverrides);
    _BinaryCompareFlags(program.restChainVaries, pose.restChainVaries);
    CHECK(program.poseInterpolators.size() == pose.poseInterpolators.size());
    for (size_t i = 0; i < pose.poseInterpolators.size() &&
                       i < program.poseInterpolators.size();
         ++i) {
        const auto &live = program.poseInterpolators[i];
        const rigExec::fb::RigExecWirePoseInterpolator &wire =
            pose.poseInterpolators[i];
        CHECK(live.path.GetString() == _BinaryText(file, wire.path));
        CHECK(int64_t(live.driverSlot) == int64_t(wire.driverSlot));
        CHECK(int64_t(live.parentSlot) == int64_t(wire.parentSlot));
        CHECK(live.allowNegativeWeights == wire.allowNegativeWeights);
        _BinaryCompareInput(live.enabled, wire.enabled, file);
        CHECK(int64_t(live.weightBegin) == int64_t(wire.weightBegin));
        CHECK(int64_t(live.weightEnd) == int64_t(wire.weightEnd));
        _BinaryCheckEqual(live.poseSlots, wire.poseSlots);
        _BinaryCheckEqual(live.disabledSlots, wire.disabledSlots);
        _BinaryCompareRbf(live.solver, wire.solver.get());
        CHECK(live.enableTranslation == wire.enableTranslation);
        CHECK(live.valueInputs.size() == wire.valueInputs.size());
        for (size_t k = 0; k < live.valueInputs.size() &&
                           k < wire.valueInputs.size();
             ++k) {
            _BinaryCompareInput(live.valueInputs[k], wire.valueInputs[k],
                                file);
        }
    }
    CHECK(program.poseWeightPaths.size() == pose.poseWeightPaths.size());
    for (size_t i = 0; i < pose.poseWeightPaths.size() &&
                       i < program.poseWeightPaths.size();
         ++i) {
        CHECK(program.poseWeightPaths[i].GetString() ==
              _BinaryText(file, pose.poseWeightPaths[i]));
    }
    CHECK(program.solvers.size() == pose.solvers.size());
    for (size_t i = 0; i < pose.solvers.size() && i < program.solvers.size();
         ++i) {
        _BinaryCompareSolver(program.solvers[i], pose.solvers[i], file);
    }
    _BinaryCheckEqual(program.guideSolvers, pose.guideSolvers);
    CHECK(program.constraints.size() == pose.constraints.size());
    for (size_t i = 0;
         i < pose.constraints.size() && i < program.constraints.size(); ++i) {
        _BinaryCompareConstraint(program.constraints[i],
                                 pose.constraints[i], file);
    }
    // The constraint arrays: their shape, and the values and lines the
    // run's prologue read at the bake time.
    CHECK(program.constraintArrays.size() == pose.constraintArrays.size());
    for (size_t i = 0; i < pose.constraintArrays.size() &&
                       i < program.constraintArrays.size();
         ++i) {
        const auto &live = program.constraintArrays[i];
        const rigExec::fb::RigExecWireConstraintArrays &wire =
            pose.constraintArrays[i];
        CHECK((live.prim ? live.prim.GetPath().GetString() : std::string()) ==
              _BinaryText(file, wire.prim));
        CHECK(uint64_t(live.sourceCount) == wire.sourceCount);
        CHECK(live.parentOffsets == wire.parentOffsets);
        CHECK(live.readPole == wire.readPole);
        CHECK(uint64_t(live.poleCount) == wire.poleCount);
        _BinaryCheckEqual(live.weights, wire.weights);
        CHECK(live.translationOffsets.size() ==
              wire.translationOffsets.size());
        for (size_t k = 0; k < live.translationOffsets.size() &&
                           k < wire.translationOffsets.size();
             ++k) {
            _BinaryCheckVec(live.translationOffsets[k],
                            wire.translationOffsets[k]);
        }
        CHECK(live.rotationOffsets.size() == wire.rotationOffsets.size());
        for (size_t k = 0; k < live.rotationOffsets.size() &&
                           k < wire.rotationOffsets.size();
             ++k) {
            _BinaryCheckVec(live.rotationOffsets[k], wire.rotationOffsets[k]);
        }
        CHECK(live.ok == wire.ok);
        _BinaryCheckEqual(live.poleWeights, wire.poleWeights);
        CHECK(live.poleOk == wire.poleOk);
        CHECK(live.diagnostics == wire.diagnostics);
        CHECK(live.poleDiagnostics == wire.poleDiagnostics);
    }
    // The native sources, and the frame the run's prologue read for each.
    CHECK(program.nativeSources.size() == pose.nativeSources.size());
    CHECK(program.nativeFrames.size() == pose.nativeSources.size());
    CHECK(program.nativeFrameOk.size() == pose.nativeSources.size());
    for (size_t i = 0; i < pose.nativeSources.size() &&
                       i < program.nativeSources.size() &&
                       i < program.nativeFrames.size() &&
                       i < program.nativeFrameOk.size();
         ++i) {
        CHECK(program.nativeSources[i].path.GetString() ==
              _BinaryText(file, pose.nativeSources[i].path));
        _BinaryCheckEqual(program.nativeSources[i].ancestorSlots,
                          pose.nativeSources[i].ancestorSlots);
        _BinaryCheckFrame(program.nativeFrames[i],
                          pose.nativeSources[i].frame);
        CHECK((program.nativeFrameOk[i] != 0) == pose.nativeSources[i].ok);
    }
    CHECK(program.walkSteps.size() == pose.walkSteps.size());
    for (size_t i = 0;
         i < pose.walkSteps.size() && i < program.walkSteps.size(); ++i) {
        const auto &live = program.walkSteps[i];
        const rigExec::fb::RigExecWireWalkStep &wire = pose.walkSteps[i];
        CHECK(live.solverBatch == wire.solverBatch);
        CHECK(uint64_t(live.level) == wire.level);
        CHECK(int64_t(live.index) == int64_t(wire.index));
        _BinaryCheckEqual(live.batchSolvers, wire.batchSolvers);
        CHECK(live.propagate.size() == wire.propagate.size());
        for (size_t k = 0;
             k < wire.propagate.size() && k < live.propagate.size(); ++k) {
            CHECK(int64_t(live.propagate[k].first) ==
                  int64_t(wire.propagate[k].first));
            CHECK(int64_t(live.propagate[k].second) ==
                  int64_t(wire.propagate[k].second));
        }
    }
    CHECK(program.composeGroups.size() == pose.composeGroups.size());
    for (size_t i = 0; i < pose.composeGroups.size() &&
                       i < program.composeGroups.size();
         ++i) {
        CHECK(int64_t(program.composeGroups[i].begin) ==
              int64_t(pose.composeGroups[i].begin));
        CHECK(int64_t(program.composeGroups[i].end) ==
              int64_t(pose.composeGroups[i].end));
        _BinaryCheckEqual(program.composeGroups[i].parentSlots,
                          pose.composeGroups[i].parentSlots);
    }
    CHECK(program.commits.size() == pose.commits.size());
    for (size_t i = 0; i < pose.commits.size() && i < program.commits.size();
         ++i) {
        _BinaryCompareCommit(program.commits[i], pose.commits[i], file);
    }
    if (program.jointSolverBinding) {
        CHECK(program.jointSolverBinding->size() ==
              pose.jointBindingJoints.size());
        CHECK(pose.jointBindingSolvers.size() ==
                  pose.jointBindingJoints.size() &&
              pose.jointBindingElements.size() ==
                  pose.jointBindingJoints.size());
        size_t i = 0;
        for (const auto &entry : *program.jointSolverBinding) {
            if (i >= pose.jointBindingJoints.size() ||
                i >= pose.jointBindingSolvers.size() ||
                i >= pose.jointBindingElements.size()) {
                break;
            }
            CHECK(entry.first.GetString() ==
                  _BinaryText(file, pose.jointBindingJoints[i]));
            const auto &solvers = pose.jointBindingSolvers[i].v;
            const auto &elements = pose.jointBindingElements[i].v;
            CHECK(entry.second.size() == solvers.size());
            CHECK(entry.second.size() == elements.size());
            for (size_t k = 0; k < entry.second.size() &&
                               k < solvers.size() && k < elements.size();
                 ++k) {
                CHECK(entry.second[k].first.GetString() ==
                      _BinaryText(file, solvers[k]));
                CHECK(int64_t(entry.second[k].second) ==
                      int64_t(elements[k]));
            }
            ++i;
        }
    } else {
        CHECK(pose.jointBindingJoints.empty());
    }
    CHECK(program.hasPropertyChains == pose.hasPropertyChains);
    CHECK(program.phasedReads == pose.phasedReads);
    CHECK(program.publishWeightFields == pose.publishWeightFields);
    _BinaryCompareSpaceSwitches(program, pose.spaceSwitches, file);
    // The varying avar bindings, then the constant ones.
    CHECK(pose.avarBindings.size() == program.avarBindings.size() +
                                          program.avarConstantBindings.size());
    size_t at = 0;
    for (const auto *bindings :
         {&program.avarBindings, &program.avarConstantBindings}) {
        for (const auto &binding : *bindings) {
            if (at >= pose.avarBindings.size()) {
                break;
            }
            CHECK(uint64_t(binding.slot) == pose.avarBindings[at].flat);
            _BinaryCompareInput(binding.input, pose.avarBindings[at].read,
                                file);
            ++at;
        }
    }
    // The relative transforms the run's prologue read.
    _BinaryCompareMatrices(program.xformBase, pose.xformBase);
    CHECK(pose.overrideCount == program.overridden.size());
}

// -- The geometry tables ---------------------------------------------------

void
_BinaryComparePhase(const rigExec::RigExecReadPhase &live,
                    const rigExec::RigExecWireReadPhase &wire,
                    const _BinaryFile &file)
{
    CHECK(uint8_t(live.kind) == wire.kind);
    CHECK(live.prim.GetString() == _BinaryText(file, wire.prim));
}

void
_BinaryCompareBinding(const rigExec::RigExecRevisionBinding &live,
                      const rigExec::fb::RigExecWireRevisionBinding *bound,
                      const _BinaryFile &file)
{
    CHECK(bound);
    if (!bound) {
        return;
    }
    const rigExec::fb::RigExecWireRevisionBinding &wire = *bound;
    const auto path = [&](const SdfPath &p, uint32_t id) {
        CHECK(p.GetString() == _BinaryText(file, id));
    };
    path(live.moverPath, wire.moverPath);
    path(live.target, wire.target);
    path(live.transform, wire.transform);
    path(live.transformSpace, wire.transformSpace);
    CHECK(live.influences.size() == wire.influences.size());
    for (size_t i = 0;
         i < wire.influences.size() && i < live.influences.size(); ++i) {
        path(live.influences[i], wire.influences[i]);
    }
    path(live.weightObject, wire.weightObject);
    path(live.base, wire.base);
    path(live.topologyCounts, wire.topologyCounts);
    path(live.topologyIndices, wire.topologyIndices);
    path(live.cagePoints, wire.cagePoints);
    path(live.surfacePoints, wire.surfacePoints);
    path(live.bindCoords, wire.bindCoords);
    path(live.driverCurvePoints, wire.driverCurvePoints);
    path(live.driverCurveOrder, wire.driverCurveOrder);
    path(live.driverCurveKnots, wire.driverCurveKnots);
    CHECK(int64_t(live.driverTransformCount) ==
          int64_t(wire.driverTransformCount));
    CHECK(int64_t(live.driverSpaceCount) == int64_t(wire.driverSpaceCount));
    CHECK(int64_t(live.driverBaseTransformCount) ==
          int64_t(wire.driverBaseTransformCount));
    path(live.driverFrames, wire.driverFrames);
    path(live.widths, wire.widths);
    CHECK(live.blendInputs.size() == wire.blendInputs.size());
    for (size_t i = 0;
         i < wire.blendInputs.size() && i < live.blendInputs.size(); ++i) {
        path(live.blendInputs[i], wire.blendInputs[i]);
    }
    CHECK(live.blendSamples.size() == wire.blendSampleInputs.size());
    CHECK(live.blendSamples.size() == wire.blendSamples.size());
    size_t row = 0;
    for (const auto &entry : live.blendSamples) {
        if (row >= wire.blendSampleInputs.size() ||
            row >= wire.blendSamples.size()) {
            break;
        }
        path(entry.first, wire.blendSampleInputs[row]);
        const auto &samples = wire.blendSamples[row].v;
        CHECK(entry.second.size() == samples.size());
        for (size_t s = 0; s < samples.size() && s < entry.second.size();
             ++s) {
            path(entry.second[s].sample, samples[s].sample);
            path(entry.second[s].points, samples[s].points);
            _BinaryComparePhase(entry.second[s].phase, samples[s].phase,
                                file);
            path(entry.second[s].blendShape, samples[s].blendShape);
        }
        ++row;
    }
    CHECK(live.phases.size() == wire.phaseInputs.size());
    CHECK(live.phases.size() == wire.phases.size());
    row = 0;
    for (const auto &entry : live.phases) {
        if (row >= wire.phaseInputs.size() || row >= wire.phases.size()) {
            break;
        }
        path(entry.first, wire.phaseInputs[row]);
        _BinaryComparePhase(entry.second, wire.phases[row], file);
        ++row;
    }
    _BinaryComparePhase(live.transformPhase, wire.transformPhase, file);
}

/// The canonical dense form of \p live: each row's entries but (0, +-0),
/// in order, then (0, +0.0f) padding. \p dropped counts the entries left
/// out.
void
_BinaryCanonicalTopology(const rigExec::RigExecSkinTopology &live,
                         std::vector<int32_t> *indices,
                         std::vector<float> *weights, size_t *dropped)
{
    const size_t width = live.elementSize > 0 ? size_t(live.elementSize) : 0;
    indices->assign(live.indices.size(), 0);
    weights->assign(live.weights.size(), 0.0f);
    for (size_t p = 0; width && (p + 1) * width <= live.indices.size() &&
                       (p + 1) * width <= live.weights.size();
         ++p) {
        size_t kept = 0;
        for (size_t e = p * width; e < (p + 1) * width; ++e) {
            if (live.indices[e] == 0 && live.weights[e] == 0.0f) {
                ++*dropped;
                continue;
            }
            (*indices)[p * width + kept] = int32_t(live.indices[e]);
            (*weights)[p * width + kept] = live.weights[e];
            ++kept;
        }
    }
}

/// A sparse topology against the program's dense one: its expansion is
/// the canonical form, bit for bit, with the same shape facts.
void
_BinaryCompareTopology(const rigExec::RigExecSkinTopology &live,
                       const rigExec::fb::RigExecWireSkinTopology *wire)
{
    CHECK(wire);
    if (!wire) {
        return;
    }
    size_t dropped = 0;
    std::vector<int32_t> wantIndices, gotIndices;
    std::vector<float> wantWeights, gotWeights;
    _BinaryCanonicalTopology(live, &wantIndices, &wantWeights, &dropped);
    rigExec::RigExecFormatExpandTopology(*wire, &gotIndices, &gotWeights);
    CHECK(wantIndices == gotIndices);
    CHECK(wantWeights.size() == gotWeights.size() &&
          (wantWeights.empty() ||
           std::memcmp(wantWeights.data(), gotWeights.data(),
                       wantWeights.size() * sizeof(float)) == 0));
    CHECK(int64_t(live.elementSize) == int64_t(wire->elementSize));
    CHECK(uint64_t(live.pointCount) == wire->pointCount);
    CHECK(uint64_t(live.influenceCount) == wire->influenceCount);
    CHECK(live.validated == wire->validated);
    if (_binaryStats) {
        ++_binaryStats->topologies;
        _binaryStats->droppedEntries += dropped;
    }
}

void
_BinaryCompareAttribute(const UsdAttribute &live, uint32_t path,
                        bool valid, const _BinaryFile &file)
{
    CHECK(bool(live) == valid);
    CHECK((live ? live.GetPath().GetString() : std::string()) ==
          _BinaryText(file, path));
}

void
_BinaryCompareRevision(
    const rigExec::RigExecBakedProgramImpl::GeomRevision &live,
    const rigExec::fb::RigExecWireRevision &wire, bool derived,
    const _BinaryFile &file)
{
    CHECK(live.moverPath.GetString() == _BinaryText(file, wire.moverPath));
    CHECK(live.target.GetString() == _BinaryText(file, wire.target));
    CHECK((live.moverPrim ? live.moverPrim.GetPath().GetString()
                          : std::string()) ==
          _BinaryText(file, wire.moverPrim));
    CHECK(uint8_t(live.op) == wire.op);
    _BinaryCompareBinding(live.binding, wire.binding.get(), file);
    CHECK(live.blendChannels.size() == wire.blendChannels.size());
    for (size_t i = 0; i < wire.blendChannels.size() &&
                       i < live.blendChannels.size();
         ++i) {
        const auto &channel = live.blendChannels[i];
        const rigExec::fb::RigExecWireBlendChannel &wireChannel =
            wire.blendChannels[i];
        _BinaryCompareAttribute(channel.weight, wireChannel.weight,
                                wireChannel.weightValid, file);
        CHECK(channel.weightPath.GetString() ==
              _BinaryText(file, wireChannel.weightPath));
        CHECK(int64_t(channel.poseWeight) ==
              int64_t(wireChannel.poseWeight));
        _BinaryCompareResolvedFloat(channel.weight, 0.0f,
                                    wireChannel.weightRead.get(), file);
        CHECK(channel.samples.size() == wireChannel.samples.size());
        for (size_t s = 0; s < wireChannel.samples.size() &&
                           s < channel.samples.size();
             ++s) {
            const auto &sample = channel.samples[s];
            const rigExec::fb::RigExecWireBlendSample &wireSample =
                wireChannel.samples[s];
            if (_binaryStats) {
                ++_binaryStats->samples;
            }
            CHECK(sample.samplePath.GetString() ==
                  _BinaryText(file, wireSample.samplePath));
            _BinaryCompareAttribute(sample.activation, wireSample.activation,
                                    wireSample.activationValid, file);
            _BinaryCompareAttribute(sample.points, wireSample.points,
                                    wireSample.pointsValid, file);
            CHECK(sample.pointsPath.GetString() ==
                  _BinaryText(file, wireSample.pointsPath));
            _BinaryComparePhase(sample.phase, wireSample.phase, file);
            CHECK(sample.blendShape.GetString() ==
                  _BinaryText(file, wireSample.blendShape));
            // The layout the run's prologue resolved, a cache-refused one
            // included.
            CHECK(bool(sample.layout) == wireSample.hasLayout);
            if (sample.layout) {
                CHECK(_BinarySamePoints(sample.layout->offsets,
                                        wireSample.offsets));
                _BinaryCheckEqual(sample.layout->indices, wireSample.indices);
                CHECK(uint64_t(sample.layout->pointCount) ==
                      wireSample.pointCount);
                CHECK(sample.layout->valid == wireSample.layoutValid);
            }
            // The dense points the run's assembly consumed.
            const auto *points =
                _BinaryPoolPoints(file, wireSample.pointsValue);
            CHECK(points && _BinarySamePoints(sample.lastPoints, *points));
            _BinaryCompareResolvedFloat(sample.activation, 1.0f,
                                        wireSample.activationRead.get(),
                                        file);
        }
    }
    _BinaryCheckEqual(live.influenceSlots, wire.influenceSlots);
    CHECK(int64_t(live.transformSlot) == int64_t(wire.transformSlot));
    CHECK(int64_t(live.transformSpaceSlot) ==
          int64_t(wire.transformSpaceSlot));
    CHECK(int64_t(live.carrySpaceSlot) == int64_t(wire.carrySpaceSlot));
    CHECK(live.binding.shaderDials.size() == wire.shaderDials.size());
    for (size_t i = 0; i < wire.shaderDials.size() &&
                       i < live.binding.shaderDials.size();
         ++i) {
        CHECK(live.binding.shaderDials[i].GetString() ==
              _BinaryText(file, wire.shaderDials[i]));
    }
    _BinaryCheckMatrix(live.binding.meshWorldInverse, wire.meshWorldInverse);
    CHECK(int64_t(live.constraintDelta) == int64_t(wire.constraintDelta));
    CHECK(int64_t(live.driverFramesSolver) ==
          int64_t(wire.driverFramesSolver));
    CHECK(live.finalPhase == wire.finalPhase);
    CHECK(live.skinTopologyFixed == wire.skinTopologyFixed);
    CHECK(live.snapshotAfter == wire.snapshotAfter);
    CHECK(live.readsSnapshots == wire.readsSnapshots);
    _BinaryCompareMatrices(live.packetInfluences, wire.packetInfluences);
    CHECK(live.chunks.size() == wire.chunks.size());
    for (size_t i = 0; i < wire.chunks.size() && i < live.chunks.size();
         ++i) {
        CHECK(int64_t(live.chunks[i].begin) == int64_t(wire.chunks[i].begin));
        CHECK(int64_t(live.chunks[i].end) == int64_t(wire.chunks[i].end));
        _BinaryCheckEqual(live.chunks[i].key, wire.chunks[i].key);
    }
    CHECK(int64_t(live.chunkBase) == int64_t(wire.chunkBase));
    // A partition cut from the topology itself is stored once.
    const bool same =
        live.partitionTopology && live.partitionTopology == live.topology;
    CHECK(same == wire.partitionSameAsTopology);
    if (same) {
        CHECK(!wire.partitionTopology);
    } else {
        CHECK(bool(live.partitionTopology) == bool(wire.partitionTopology));
        if (live.partitionTopology) {
            _BinaryCompareTopology(*live.partitionTopology,
                                   wire.partitionTopology.get());
        }
    }
    CHECK(int64_t(live.partitionElementSize) ==
          int64_t(wire.partitionElementSize));
    CHECK(uint64_t(live.partitionIndexCount) == wire.partitionIndexCount);
    CHECK(uint64_t(live.partitionPointCount) == wire.partitionPointCount);
    CHECK(live.chunked == wire.chunked);
    CHECK(uint64_t(live.partitionCandidates) == wire.partitionCandidates);
    CHECK(int64_t(live.partitionReadyMin) ==
          int64_t(wire.partitionReadyMin));
    CHECK(int64_t(live.partitionReadyMax) ==
          int64_t(wire.partitionReadyMax));
    CHECK(int64_t(live.weightObject) == int64_t(wire.weightObject));
    CHECK(live.weightOperationDomain == wire.weightOperationDomain);
    CHECK(live.weightFieldTarget.GetString() ==
          _BinaryText(file, wire.weightFieldTarget));
    CHECK(live.weightCurrentPhase == wire.weightCurrentPhase);
    CHECK(bool(live.topology) == bool(wire.topology));
    if (live.topology) {
        _BinaryCompareTopology(*live.topology, wire.topology.get());
    }
    CHECK(live.topologyResolved == wire.topologyResolved);
    // A main revision reads its inputs:defaultWeight; a derived one holds
    // none.
    if (derived) {
        CHECK(!wire.defaultWeight);
    } else {
        const UsdAttribute head =
            live.moverPrim
                ? live.moverPrim.GetAttribute(TfToken("inputs:defaultWeight"))
                : UsdAttribute();
        _BinaryCompareResolvedFloat(head, 1.0f, wire.defaultWeight.get(),
                                    file);
    }
}

/// A step-backed weight object against the program's, and the oracle
/// facts of every entry against the oracle's own description at the bake
/// time.
void
_BinaryCompareWeightObject(
    const rigExec::RigExecBakedProgramImpl::WeightObject &live,
    const rigExec::fb::RigExecWireWeightObject &wire,
    const _BinaryFile &file)
{
    CHECK(!wire.envelopeOnly);
    CHECK(live.path.GetString() == _BinaryText(file, wire.path));
    CHECK(live.type.GetString() == _BinaryText(file, wire.type));
    CHECK(live.representation.GetString() ==
          _BinaryText(file, wire.representation));
    CHECK(live.rangePolicy.GetString() ==
          _BinaryText(file, wire.rangePolicy));
    _BinaryCheckEqual(live.values, wire.values);
    _BinaryCheckEqual(live.indices, wire.indices);
    _BinaryCompareInput(live.defaultWeight, wire.defaultWeight, file);
    CHECK(int64_t(live.base) == int64_t(wire.base));
    _BinaryCheckEqual(live.inputs, wire.inputs);
    _BinaryCompareInput(live.driver, wire.driver, file);
    _BinaryCompareInput(live.scale, wire.scale, file);
    _BinaryCompareInput(live.bias, wire.bias, file);
    CHECK(live.combineMode.GetString() ==
          _BinaryText(file, wire.combineMode));
    _BinaryCompareInput(live.strength, wire.strength, file);
    _BinaryCompareInput(live.invert, wire.invert, file);
    const auto attributes = [&](const std::vector<UsdAttribute> &attrs,
                                const std::vector<uint32_t> &paths,
                                const std::vector<uint8_t> &valid) {
        CHECK(attrs.size() == paths.size() && attrs.size() == valid.size());
        for (size_t i = 0;
             i < attrs.size() && i < paths.size() && i < valid.size(); ++i) {
            _BinaryCompareAttribute(attrs[i], paths[i], valid[i] != 0, file);
        }
    };
    attributes(live.combineTargetPoints, wire.combineTargetPoints,
               wire.combineTargetValid);
    CHECK(uint64_t(live.costElements) == wire.costElements);
    CHECK(int64_t(live.providerSlot) == int64_t(wire.providerSlot));
    _BinaryCompareInput(live.falloffMin, wire.falloffMin, file);
    _BinaryCompareInput(live.falloffMax, wire.falloffMax, file);
    _BinaryCompareInput(live.scaleXPos, wire.scaleXPos, file);
    _BinaryCompareInput(live.scaleYPos, wire.scaleYPos, file);
    _BinaryCompareInput(live.scaleZPos, wire.scaleZPos, file);
    _BinaryCompareInput(live.scaleXNeg, wire.scaleXNeg, file);
    _BinaryCompareInput(live.scaleYNeg, wire.scaleYNeg, file);
    _BinaryCompareInput(live.scaleZNeg, wire.scaleZNeg, file);
    _BinaryCompareInput(live.scaleX, wire.scaleX, file);
    _BinaryCompareInput(live.scaleY, wire.scaleY, file);
    _BinaryCompareInput(live.scaleZ, wire.scaleZ, file);
    _BinaryCompareInput(live.extentU, wire.extentU, file);
    _BinaryCompareInput(live.extentV, wire.extentV, file);
    CHECK(live.planeAxis.GetString() == _BinaryText(file, wire.planeAxis));
    CHECK(live.planeBounds.GetString() ==
          _BinaryText(file, wire.planeBounds));
    attributes(live.targetPoints, wire.targetPoints, wire.targetValid);
    attributes(live.samplePoints, wire.samplePoints, wire.sampleValid);
    attributes(live.curvePoints, wire.curvePoints, wire.curveValid);
    _BinaryCheckEqual(live.falloffCurve, wire.falloffCurve);
}

void
_BinaryCompareOracleFacts(const rigExec::RigExecRigEvaluator &evaluator,
                          const rigExec::fb::RigExecWireWeightObject &wire,
                          double t, const _BinaryFile &file)
{
    const SdfPath path(_BinaryText(file, wire.path));
    rigExec::RigExecWeightOracleFacts facts;
    rigExec::RigExecBakedDescribeWeightOracle(evaluator, path, UsdTimeCode(t),
                                              &facts);
    CHECK(facts.samplesInFlight == wire.samplesInFlight);
    CHECK(facts.haveSamples == (wire.oracleSamples >= 0));
    if (facts.haveSamples && wire.oracleSamples >= 0) {
        const auto *points =
            _BinaryPoolPoints(file, uint32_t(wire.oracleSamples));
        CHECK(points && _BinarySamePoints(facts.samples, *points));
    }
    CHECK(facts.haveCurve == (wire.oracleCurve >= 0));
    if (facts.haveCurve && wire.oracleCurve >= 0) {
        const auto *points = _BinaryPoolPoints(file, uint32_t(wire.oracleCurve));
        CHECK(points && _BinarySamePoints(facts.curve, *points));
    }
    if (_BinaryText(file, wire.type) == "RigExecPlaneWeight") {
        CHECK(facts.planeAxis.GetString() ==
              _BinaryText(file, wire.oraclePlaneAxis));
        CHECK(facts.planeBounds.GetString() ==
              _BinaryText(file, wire.oraclePlaneBounds));
    } else {
        CHECK(wire.oraclePlaneAxis == 0 && wire.oraclePlaneBounds == 0);
    }
    CHECK(facts.phaseError == wire.oraclePhaseError);
    CHECK(facts.staticError == wire.oracleStaticError);
    if (_binaryStats) {
        ++_binaryStats->oracleFacts;
    }
}

void
_BinaryCompareDomainGeometry(const rigExec::RigExecRigEvaluator &evaluator,
                             const rigExec::RigExecBakedProgramImpl &program,
                             const rigExec::fb::RigExecWireDomainGeometry
                                 &geometry,
                             const _BinaryFile &file)
{
    // The chains, with the bases the run's prologue read.
    const auto base = [&](bool have, const VtVec3fArray &points,
                          bool fileHave, uint32_t fileBase) {
        if (_binaryStats) {
            ++_binaryStats->bases;
        }
        CHECK(have == fileHave);
        if (!have) {
            CHECK(fileBase == 0);
            return;
        }
        const auto *pooled = _BinaryPoolPoints(file, fileBase);
        CHECK(pooled && _BinarySamePoints(points, *pooled));
    };
    CHECK(program.chains.size() == geometry.chains.size());
    for (size_t i = 0;
         i < geometry.chains.size() && i < program.chains.size(); ++i) {
        const auto &chain = program.chains[i];
        const rigExec::fb::RigExecWireChain &wire = geometry.chains[i];
        CHECK(chain.target.GetString() == _BinaryText(file, wire.target));
        base(chain.haveBase, chain.lastBase, wire.haveBase, wire.base);
        CHECK(chain.revisions.size() == wire.revisions.size());
        for (size_t r = 0;
             r < wire.revisions.size() && r < chain.revisions.size(); ++r) {
            _BinaryCompareRevision(chain.revisions[r], wire.revisions[r],
                                   false, file);
        }
        CHECK(chain.derived.size() == wire.derived.size());
        for (size_t d = 0;
             d < wire.derived.size() && d < chain.derived.size(); ++d) {
            const auto &derived = chain.derived[d];
            CHECK(derived.target.GetString() ==
                  _BinaryText(file, wire.derived[d].target));
            base(derived.haveBase, derived.lastBase, wire.derived[d].haveBase,
                 wire.derived[d].base);
            CHECK(wire.derived[d].revision);
            if (wire.derived[d].revision) {
                _BinaryCompareRevision(derived.revision,
                                       *wire.derived[d].revision, true, file);
            }
        }
    }
    CHECK(program.revisionIndex.size() == geometry.revisionIndex.size());
    for (size_t i = 0; i < geometry.revisionIndex.size() &&
                       i < program.revisionIndex.size();
         ++i) {
        CHECK(int64_t(program.revisionIndex[i].first) ==
              int64_t(geometry.revisionIndex[i].first));
        CHECK(int64_t(program.revisionIndex[i].second) ==
              int64_t(geometry.revisionIndex[i].second));
    }
    CHECK(program.derivedIndex.size() == geometry.derivedIndex.size());
    for (size_t i = 0; i < geometry.derivedIndex.size() &&
                       i < program.derivedIndex.size();
         ++i) {
        CHECK(int64_t(program.derivedIndex[i].first) ==
              int64_t(geometry.derivedIndex[i].first));
        CHECK(int64_t(program.derivedIndex[i].second) ==
              int64_t(geometry.derivedIndex[i].second));
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
    // The step-backed objects in the program's order, then the envelope-
    // only ones; every entry's oracle facts.
    CHECK(geometry.weightObjects.size() >= program.weightObjects.size());
    for (size_t i = 0; i < geometry.weightObjects.size(); ++i) {
        const rigExec::fb::RigExecWireWeightObject &wire =
            geometry.weightObjects[i];
        if (i < program.weightObjects.size()) {
            _BinaryCompareWeightObject(program.weightObjects[i], wire, file);
        } else {
            CHECK(wire.envelopeOnly);
        }
        _BinaryCompareOracleFacts(evaluator, wire, file.bakeTime, file);
    }
    CHECK(program.falloffLuts.size() == geometry.falloffPaths.size());
    CHECK(program.falloffLuts.size() == geometry.falloffLuts.size());
    size_t lut = 0;
    for (const auto &entry : program.falloffLuts) {
        if (lut >= geometry.falloffPaths.size() ||
            lut >= geometry.falloffLuts.size()) {
            break;
        }
        CHECK(entry.first.GetString() ==
              _BinaryText(file, geometry.falloffPaths[lut]));
        _BinaryCheckEqual(entry.second, geometry.falloffLuts[lut].v);
        ++lut;
    }
    CHECK(program.currentPhaseWeights.size() ==
          geometry.currentPhaseWeights.size());
    lut = 0;
    for (const SdfPath &path : program.currentPhaseWeights) {
        if (lut >= geometry.currentPhaseWeights.size()) {
            break;
        }
        CHECK(path.GetString() ==
              _BinaryText(file, geometry.currentPhaseWeights[lut]));
        ++lut;
    }
    CHECK(program.deltaBasePaths.size() == geometry.deltaBasePaths.size());
    for (size_t i = 0; i < geometry.deltaBasePaths.size() &&
                       i < program.deltaBasePaths.size();
         ++i) {
        CHECK(program.deltaBasePaths[i].GetString() ==
              _BinaryText(file, geometry.deltaBasePaths[i]));
    }
    // The delta bases the run's prologue read.
    _BinaryCompareMatrices(program.deltaBaseMatrix, geometry.deltaBaseMatrix);
    _BinaryCompareFlags(program.deltaBaseOk, geometry.deltaBaseOk);
}

// -- Path reads ------------------------------------------------------------

/// \p held, a value an enumerated site consumes, against a value row.
bool
_BinarySamePathValue(const VtValue &held,
                     const rigExec::fb::RigExecWirePathValue &value,
                     const _BinaryFile &file)
{
    using Tag = rigExec::fb::PathTag;
    const auto pool = [](uint32_t id, const auto &arrays) {
        return id < arrays.size() ? &arrays[id].v : nullptr;
    };
    if (held.IsEmpty()) {
        return value.tag == Tag::Absent;
    }
    if (held.IsHolding<bool>()) {
        return value.tag == Tag::Bool &&
               value.bits == (held.UncheckedGet<bool>() ? 1u : 0u);
    }
    if (held.IsHolding<int>()) {
        return value.tag == Tag::Int &&
               value.bits == uint64_t(uint32_t(held.UncheckedGet<int>()));
    }
    if (held.IsHolding<float>()) {
        return value.tag == Tag::Float &&
               value.bits == _BinaryBits(held.UncheckedGet<float>());
    }
    if (held.IsHolding<double>()) {
        return value.tag == Tag::Double &&
               value.bits == _BinaryBits(held.UncheckedGet<double>());
    }
    if (held.IsHolding<TfToken>()) {
        return value.tag == Tag::Token && value.bits <= 0xffffffffull &&
               _BinaryText(file, uint32_t(value.bits)) ==
                   held.UncheckedGet<TfToken>().GetString();
    }
    if (held.IsHolding<GfMatrix4d>()) {
        rigExec::fb::RigExecWireValue probe;
        probe.matrix = value.matrix
                           ? std::make_unique<rigExec::RigExecWireMatrix4d>(
                                 *value.matrix)
                           : nullptr;
        return value.tag == Tag::Matrix4d &&
               _BinaryConstantIs(probe, held.UncheckedGet<GfMatrix4d>(),
                                 file);
    }
    if (held.IsHolding<GfVec3d>()) {
        const GfVec3d &v = held.UncheckedGet<GfVec3d>();
        return value.tag == Tag::Vec3d && value.vec3d &&
               _BinaryEq(v[0], (*value.vec3d)[0]) &&
               _BinaryEq(v[1], (*value.vec3d)[1]) &&
               _BinaryEq(v[2], (*value.vec3d)[2]);
    }
    if (held.IsHolding<GfVec3i>()) {
        const GfVec3i &v = held.UncheckedGet<GfVec3i>();
        return value.tag == Tag::Vec3i && value.vec3i &&
               v[0] == (*value.vec3i)[0] && v[1] == (*value.vec3i)[1] &&
               v[2] == (*value.vec3i)[2];
    }
    if (held.IsHolding<VtIntArray>()) {
        const VtIntArray &a = held.UncheckedGet<VtIntArray>();
        const auto *p = pool(value.array, file.intArrays);
        return value.tag == Tag::IntArray && p &&
               std::vector<int32_t>(a.begin(), a.end()) == *p;
    }
    if (held.IsHolding<VtFloatArray>()) {
        const VtFloatArray &a = held.UncheckedGet<VtFloatArray>();
        const auto *p = pool(value.array, file.floatArrays);
        return value.tag == Tag::FloatArray && p && p->size() == a.size() &&
               (a.empty() || std::memcmp(a.cdata(), p->data(),
                                         a.size() * sizeof(float)) == 0);
    }
    if (held.IsHolding<VtDoubleArray>()) {
        const VtDoubleArray &a = held.UncheckedGet<VtDoubleArray>();
        const auto *p = pool(value.array, file.doubleArrays);
        return value.tag == Tag::DoubleArray && p && p->size() == a.size() &&
               (a.empty() || std::memcmp(a.cdata(), p->data(),
                                         a.size() * sizeof(double)) == 0);
    }
    if (held.IsHolding<VtArray<GfVec2f>>()) {
        const VtArray<GfVec2f> &a = held.UncheckedGet<VtArray<GfVec2f>>();
        const auto *p = pool(value.array, file.vec2fArrays);
        if (value.tag != Tag::Vec2fArray || !p || p->size() != a.size()) {
            return false;
        }
        for (size_t i = 0; i < a.size(); ++i) {
            if (!_BinaryEq(a[i][0], (*p)[i][0]) ||
                !_BinaryEq(a[i][1], (*p)[i][1])) {
                return false;
            }
        }
        return true;
    }
    if (held.IsHolding<VtVec3fArray>()) {
        const auto *p = pool(value.array, file.vec3fArrays);
        return value.tag == Tag::Vec3fArray && p &&
               _BinarySamePoints(held.UncheckedGet<VtVec3fArray>(), *p);
    }
    return false;
}

/// The path reads against an enumeration of the program after its run:
/// every key no overlay stands on is a row; a value row holds the value
/// its first such site consumes; a read row is a connection-following site
/// the enumeration lists, read the way RigExecResolvedInputs reads it, from
/// its own attribute.
void
_BinaryComparePathReads(const rigExec::RigExecBakedProgramImpl &program,
                        const _BinaryFile &file, const std::string &fixture)
{
    std::vector<rigExec::RigExecBakeRevisionRead> enumerated;
    rigExec::RigExecBakeEnumerateProgramReads(program, file.bakeTime,
                                              &enumerated);
    using Key = std::pair<std::string, bool>;
    std::map<Key, const rigExec::RigExecBakeRevisionRead *> first;
    std::set<Key> resolved;
    for (const rigExec::RigExecBakeRevisionRead &candidate : enumerated) {
        const Key key(candidate.path.GetString(), candidate.rest);
        if (candidate.resolved) {
            resolved.insert(key);
        }
        if (!candidate.overlaid) {
            first.emplace(key, &candidate);
        }
    }
    CHECK(file.geometry);
    if (!file.geometry) {
        return;
    }
    std::set<Key> rows;
    size_t readRows = 0, valueRows = 0;
    for (const rigExec::fb::RigExecWirePathRead &row :
         file.geometry->pathReads) {
        const Key key(_BinaryText(file, row.path), row.rest);
        CHECK(rows.insert(key).second);
        CHECK(bool(row.value) != bool(row.read));
        if (row.read) {
            ++readRows;
            const _BinaryRead &read = *row.read;
            CHECK(!row.rest && row.headFallback);
            CHECK(read.mode == rigExec::fb::ReadMode::Resolved);
            CHECK(read.overrideIndex == -1 && read.selected == -1);
            CHECK(!read.walk.empty() &&
                  _BinarySlotName(file, read.walk[0]) == key.first);
            const bool listed = resolved.count(key) != 0;
            if (!listed) {
                std::printf("  %s: read row %s is no connection-following "
                            "site of the enumeration\n",
                            fixture.c_str(), key.first.c_str());
            }
            CHECK(listed);
            continue;
        }
        if (!row.value) {
            continue;
        }
        ++valueRows;
        const auto found = first.find(key);
        CHECK(found != first.end());
        if (found == first.end()) {
            std::printf("  %s: value row %s (%s) is not enumerated\n",
                        fixture.c_str(), key.first.c_str(),
                        key.second ? "rest" : "live");
            continue;
        }
        const bool same =
            _BinarySamePathValue(found->second->value, *row.value, file);
        if (!same) {
            std::printf("  %s: value row %s (%s) holds another value than "
                        "its site consumes\n",
                        fixture.c_str(), key.first.c_str(),
                        key.second ? "rest" : "live");
        }
        CHECK(same);
    }
    size_t missing = 0;
    for (const auto &entry : first) {
        if (!rows.count(entry.first) && missing++ == 0) {
            std::printf("  %s: enumerated %s (%s) is not a row\n",
                        fixture.c_str(), entry.first.first.c_str(),
                        entry.first.second ? "rest" : "live");
        }
    }
    CHECK(missing == 0);
    if (_binaryStats) {
        _binaryStats->readRows += readRows;
        _binaryStats->valueRows += valueRows;
        _binaryStats->enumeratedKeys += first.size();
    }
}

// -- Property chains -------------------------------------------------------

/// A property-mover read as _PinnedRead resolves it: Resolved through a
/// connection, else Pinned over the attribute alone, else an empty walk.
void
_BinaryComparePropertyRead(const UsdAttribute &head, const _BinaryRead *wire,
                           const _BinaryFile &file)
{
    CHECK(wire);
    if (!wire) {
        return;
    }
    if (_binaryStats) {
        ++_binaryStats->reads;
    }
    CHECK(wire->overrideIndex == -1 && wire->selected == -1);
    if (!head) {
        CHECK(wire->mode == rigExec::fb::ReadMode::Pinned);
        CHECK(wire->walk.empty());
        return;
    }
    CHECK(wire->mode == (head.HasAuthoredConnections()
                             ? rigExec::fb::ReadMode::Resolved
                             : rigExec::fb::ReadMode::Pinned));
    CHECK(!wire->walk.empty() && _BinarySlotName(file, wire->walk[0]) ==
                                     head.GetPath().GetString());
}

/// The chains and phased consumers against the evaluator's own statement
/// of its property chains.
void
_BinaryComparePropertyChains(const rigExec::RigExecRigEvaluator &evaluator,
                             const rigExec::RigExecBakedProgramImpl &program,
                             const _BinaryFile &file)
{
    std::vector<rigExec::RigExecBakedPropertyChainDesc> descs;
    std::string error;
    CHECK(rigExec::RigExecBakedDescribePropertyChains(evaluator, &descs,
                                                      &error));
    CHECK(descs.size() == file.propertyChains.size());
    CHECK(program.hasPropertyChains == !file.propertyChains.empty());
    std::set<std::string> targets, programTargets;
    for (const SdfPath &path : program.chainTargets) {
        programTargets.insert(path.GetString());
    }
    using Op = rigExec::fb::PropertyOp;
    const auto opOf = [](const rigExec::RigExecBakedPropertyChainDesc::
                             Revision &r) {
        if (!r.opValid) {
            return Op::Invalid;
        }
        switch (r.op) {
        case RigExecPropertyOp::Add:
            return Op::Add;
        case RigExecPropertyOp::Multiply:
            return Op::Multiply;
        case RigExecPropertyOp::Clamp:
            return Op::Clamp;
        case RigExecPropertyOp::Remap:
            return Op::Remap;
        case RigExecPropertyOp::Blend:
            return Op::Blend;
        case RigExecPropertyOp::Curve:
            return Op::Curve;
        }
        return Op::Invalid;
    };
    const auto sameKeys = [](const std::vector<GfVec2f> &live,
                             const std::vector<rigExec::RigExecWireVec2f>
                                 &wire) {
        if (live.size() != wire.size()) {
            return false;
        }
        for (size_t i = 0; i < live.size(); ++i) {
            if (!_BinaryEq(live[i][0], wire[i][0]) ||
                !_BinaryEq(live[i][1], wire[i][1])) {
                return false;
            }
        }
        return true;
    };
    size_t phased = 0;
    for (size_t c = 0; c < descs.size() && c < file.propertyChains.size();
         ++c) {
        const rigExec::RigExecBakedPropertyChainDesc &desc = descs[c];
        const rigExec::fb::RigExecWirePropertyChain &chain =
            file.propertyChains[c];
        if (_binaryStats) {
            ++_binaryStats->propertyChains;
        }
        const std::string target = _BinarySlotName(file, chain.target);
        targets.insert(target);
        CHECK(target == desc.target.GetString());
        CHECK(chain.target < file.inputs.size() &&
              file.inputs[chain.target].chain() == int32_t(c));
        CHECK(uint8_t(chain.valueType) == uint8_t(desc.valueType));
        const bool matrix =
            desc.valueType ==
            rigExec::RigExecBakedPropertyChainDesc::ValueType::Matrix4d;
        CHECK(chain.revisions.size() == desc.revisions.size());
        for (size_t r = 0;
             r < chain.revisions.size() && r < desc.revisions.size(); ++r) {
            const auto &live = desc.revisions[r];
            const rigExec::fb::RigExecWirePropertyRevision &wire =
                chain.revisions[r];
            CHECK(live.mover.GetString() == _BinaryText(file, wire.mover));
            CHECK(wire.op == opOf(live));
            _BinaryComparePropertyRead(live.enabled, wire.enabled.get(), file);
            _BinaryComparePropertyRead(live.defaultWeight,
                                       wire.defaultWeight.get(), file);
            _BinaryComparePropertyRead(live.value, wire.value.get(), file);
            _BinaryComparePropertyRead(matrix ? UsdAttribute() : live.minimum,
                                       wire.min.get(), file);
            _BinaryComparePropertyRead(matrix ? UsdAttribute() : live.maximum,
                                       wire.max.get(), file);
            if (live.weightObject.IsEmpty()) {
                CHECK(wire.envelope == -1);
            } else {
                CHECK(wire.envelope >= 0 && file.geometry &&
                      size_t(wire.envelope) <
                          file.geometry->weightObjects.size() &&
                      _BinaryText(file,
                                  file.geometry
                                      ->weightObjects[size_t(wire.envelope)]
                                      .path) ==
                          live.weightObject.GetString());
            }
            CHECK(sameKeys(live.keys, wire.keys));
            CHECK(live.hasTangentsAttr == wire.hasTangentsAttr);
            CHECK(sameKeys(live.tangents, wire.tangents));
        }
        for (const auto &p : desc.phased) {
            CHECK(phased < file.phasedConsumers.size());
            if (phased >= file.phasedConsumers.size()) {
                break;
            }
            const rigExec::fb::RigExecWirePhasedConsumer &wire =
                file.phasedConsumers[phased];
            CHECK(wire.chain == c);
            CHECK(_BinarySlotName(file, wire.consumer) ==
                  p.consumer.GetString());
            CHECK(wire.consumer < file.inputs.size() &&
                  file.inputs[wire.consumer].phased() == int32_t(phased));
            CHECK(uint8_t(wire.consumerType) == uint8_t(p.consumerType));
            CHECK(uint64_t(wire.applied) == uint64_t(p.applied));
            CHECK(wire.hops.size() == p.hops.size());
            for (size_t h = 0; h < wire.hops.size() && h < p.hops.size();
                 ++h) {
                CHECK(_BinarySlotName(file, wire.hops[h]) ==
                      p.hops[h].GetString());
            }
            ++phased;
        }
    }
    CHECK(phased == file.phasedConsumers.size());
    CHECK(targets == programTargets);
}

// -- The input list --------------------------------------------------------

/// Calls \p visit with every read the file holds, wherever it sits.
void
_BinaryForEachRead(const _BinaryFile &file,
                   const std::function<void(const _BinaryRead &)> &visit)
{
    const auto one = [&](const std::unique_ptr<_BinaryRead> &read) {
        if (read) {
            visit(*read);
        }
    };
    if (file.pose) {
        const rigExec::fb::RigExecWireDomainPose &pose = *file.pose;
        for (const auto &ladder : pose.ladders) {
            one(ladder.restSpace);
            one(ladder.defaultSpace);
            one(ladder.posedSpace);
            for (const auto &read : ladder.restAvars) {
                visit(read);
            }
            for (const auto &read : ladder.defaultAvars) {
                visit(read);
            }
            one(ladder.rotationOrder);
        }
        for (const auto &interp : pose.poseInterpolators) {
            one(interp.enabled);
            for (const auto &read : interp.valueInputs) {
                visit(read);
            }
        }
        for (const auto &s : pose.solvers) {
            for (const auto *read :
                 {&s.bend, &s.upperOffset, &s.lowerOffset, &s.stretch,
                  &s.softness, &s.blendWeight, &s.preserveVolume,
                  &s.midFollowWeight, &s.roll, &s.twist, &s.minLengthRatio,
                  &s.twistTurns, &s.ribbonSampleCount, &s.ikSpace}) {
                one(*read);
            }
        }
        for (const auto &c : pose.constraints) {
            for (const auto *read :
                 {&c.enabled, &c.defaultWeight, &c.offset, &c.affectX,
                  &c.affectY, &c.affectZ, &c.tX, &c.tY, &c.tZ, &c.rX, &c.rY,
                  &c.rZ, &c.sX, &c.sY, &c.sZ, &c.aimVector, &c.upVector,
                  &c.rotationOffset, &c.worldUpVector, &c.poleVector,
                  &c.twistDegrees}) {
                one(*read);
            }
        }
        for (const auto &sw : pose.spaceSwitches) {
            one(sw.active);
        }
        for (const auto &binding : pose.avarBindings) {
            one(binding.read);
        }
    }
    if (file.geometry) {
        const auto revision = [&](const rigExec::fb::RigExecWireRevision &r) {
            one(r.defaultWeight);
            for (const auto &channel : r.blendChannels) {
                one(channel.weightRead);
                for (const auto &sample : channel.samples) {
                    one(sample.activationRead);
                }
            }
        };
        for (const auto &chain : file.geometry->chains) {
            for (const auto &r : chain.revisions) {
                revision(r);
            }
            for (const auto &d : chain.derived) {
                if (d.revision) {
                    revision(*d.revision);
                }
            }
        }
        for (const auto &w : file.geometry->weightObjects) {
            for (const auto *read :
                 {&w.defaultWeight, &w.driver, &w.scale, &w.bias, &w.strength,
                  &w.invert, &w.falloffMin, &w.falloffMax, &w.scaleXPos,
                  &w.scaleYPos, &w.scaleZPos, &w.scaleXNeg, &w.scaleYNeg,
                  &w.scaleZNeg, &w.scaleX, &w.scaleY, &w.scaleZ, &w.extentU,
                  &w.extentV}) {
                one(*read);
            }
        }
        for (const auto &row : file.geometry->pathReads) {
            one(row.read);
        }
    }
    for (const auto &chain : file.propertyChains) {
        for (const auto &r : chain.revisions) {
            for (const auto *read :
                 {&r.enabled, &r.defaultWeight, &r.value, &r.min, &r.max}) {
                one(*read);
            }
        }
    }
    for (const auto &mover : file.externalMovers) {
        for (const auto &read : mover.inputs) {
            visit(read);
        }
    }
}

/// Every override number the program registered lies on exactly one Baked
/// read, whose walk holds every property the program lists for it; no
/// other read carries one.
void
_BinaryCheckOverrides(const rigExec::RigExecBakedProgramImpl &program,
                      const _BinaryFile &file)
{
    const size_t count = program.overridden.size();
    CHECK(file.pose && file.pose->overrideCount == count);
    std::vector<const _BinaryRead *> holder(count, nullptr);
    std::vector<size_t> held(count, 0);
    size_t stray = 0;
    _BinaryForEachRead(file, [&](const _BinaryRead &read) {
        if (read.overrideIndex < 0) {
            return;
        }
        if (read.mode != rigExec::fb::ReadMode::Baked ||
            size_t(read.overrideIndex) >= count) {
            ++stray;
            return;
        }
        ++held[size_t(read.overrideIndex)];
        holder[size_t(read.overrideIndex)] = &read;
    });
    CHECK(stray == 0);
    size_t once = 0;
    for (size_t n = 0; n < count; ++n) {
        once += held[n] == 1 ? 1 : 0;
    }
    CHECK(once == count);
    for (const auto &entry : program.overridableInputs) {
        const std::string path = entry.first.GetString();
        for (const int number : entry.second) {
            CHECK(number >= 0 && size_t(number) < count &&
                  holder[size_t(number)]);
            if (number < 0 || size_t(number) >= count ||
                !holder[size_t(number)]) {
                continue;
            }
            bool onWalk = false;
            for (const uint32_t slot : holder[size_t(number)]->walk) {
                onWalk = onWalk || _BinarySlotName(file, slot) == path;
            }
            CHECK(onWalk);
        }
    }
    if (_binaryStats) {
        _binaryStats->overrideNumbers += count;
    }
}

/// Every slot is listed, typed as its attribute, flagged as its attribute
/// reads at the bake time, and holds that typed Get's value bit for bit
/// (a token by text), or the type's zero when the Get fails.
void
_BinaryCheckDefaults(const UsdStageRefPtr &stage, const _BinaryFile &file)
{
    using Tag = rigExec::fb::InputTag;
    using Flags = rigExec::fb::InputSlotFlags;
    const UsdTimeCode time(file.bakeTime);
    CHECK(file.listedInputs == file.inputs.size());
    for (size_t s = 0; s < file.inputs.size(); ++s) {
        const rigExec::fb::InputSlot &slot = file.inputs[s];
        const std::string name = _BinaryText(file, slot.name());
        const UsdAttribute a = stage->GetAttributeAtPath(SdfPath(name));
        CHECK(a);
        CHECK(slot.value() < file.values.size());
        if (!a || slot.value() >= file.values.size()) {
            continue;
        }
        const rigExec::fb::RigExecWireValue &value = file.values[slot.value()];
        CHECK(value.tag == slot.type());
        CHECK((slot.flags() & uint8_t(Flags::Listed)) != 0);
        const bool animated =
            a.ValueMightBeTimeVarying() || a.GetNumTimeSamples() > 0;
        CHECK(animated == ((slot.flags() & uint8_t(Flags::Animated)) != 0));
        const TfType type = a.GetTypeName().GetType();
        bool has = false, same = false;
        switch (slot.type()) {
        case Tag::Double: {
            double v = 0.0;
            CHECK(type == TfType::Find<double>());
            has = a.Get(&v, time);
            same = value.bits == (has ? _BinaryBits(v) : 0);
            break;
        }
        case Tag::Float: {
            float v = 0.0f;
            CHECK(type == TfType::Find<float>());
            has = a.Get(&v, time);
            same = value.bits == (has ? _BinaryBits(v) : 0);
            break;
        }
        case Tag::Bool: {
            bool v = false;
            CHECK(type == TfType::Find<bool>());
            has = a.Get(&v, time);
            same = value.bits == (has && v ? 1u : 0u);
            break;
        }
        case Tag::Int: {
            int v = 0;
            CHECK(type == TfType::Find<int>());
            has = a.Get(&v, time);
            same = value.bits == (has ? uint64_t(uint32_t(v)) : 0);
            break;
        }
        case Tag::Token: {
            TfToken v;
            CHECK(type == TfType::Find<TfToken>());
            has = a.Get(&v, time);
            same = has ? value.bits <= 0xffffffffull &&
                             _BinaryText(file, uint32_t(value.bits)) ==
                                 v.GetString()
                       : value.bits == 0;
            break;
        }
        case Tag::Matrix4d: {
            GfMatrix4d v(1.0);
            CHECK(type == TfType::Find<GfMatrix4d>());
            has = a.Get(&v, time);
            if (has) {
                same = _BinaryConstantIs(value, v, file);
            } else {
                same = value.matrix &&
                       *value.matrix == rigExec::RigExecWireMatrix4d{};
            }
            break;
        }
        case Tag::Vec3d: {
            GfVec3d v(0.0);
            CHECK(type == TfType::Find<GfVec3d>());
            has = a.Get(&v, time);
            same = value.vec3d &&
                   (has ? _BinaryConstantIs(value, v, file)
                        : *value.vec3d == rigExec::RigExecWireVec3d{});
            break;
        }
        case Tag::Vec3f: {
            GfVec3f v(0.0f);
            CHECK(type == TfType::Find<GfVec3f>());
            has = a.Get(&v, time);
            same = value.vec3f &&
                   (has ? _BinaryEq(v[0], (*value.vec3f)[0]) &&
                              _BinaryEq(v[1], (*value.vec3f)[1]) &&
                              _BinaryEq(v[2], (*value.vec3f)[2])
                        : *value.vec3f == rigExec::RigExecWireVec3f{});
            break;
        }
        }
        CHECK(has == ((slot.flags() & uint8_t(Flags::HasValue)) != 0));
        if (!same) {
            std::printf("  default of %s differs from its value at %g\n",
                        name.c_str(), file.bakeTime);
        }
        CHECK(same);
        if (_binaryStats) {
            ++_binaryStats->defaults;
        }
    }
}

/// Opens \p bytes and compares the file with the live program of
/// \p evaluator -- the program the bake ran, still holding that run -- for
/// every table, the static data, the path reads, the property chains and
/// the input list. Returns the opened file, null when it does not open.
std::unique_ptr<_BinaryFile>
_BinaryCompareProgram(rigExec::RigExecRigEvaluator &evaluator,
                      const std::vector<uint8_t> &bytes,
                      const std::string &fixture,
                      _BinaryCompareStats *stats = nullptr)
{
    const rigExec::RigExecBakedProgram *baked = evaluator.GetBakedProgram();
    CHECK(baked);
    if (!baked) {
        return nullptr;
    }
    const rigExec::RigExecBakedProgramImpl &program = baked->GetStepGraph();
    std::unique_ptr<_BinaryFile> opened;
    std::string error;
    CHECK(rigExec::RigExecFormatOpen(bytes.data(), bytes.size(), &opened,
                                     &error));
    if (!opened) {
        std::printf("  %s does not open: %s\n", fixture.c_str(),
                    error.c_str());
        return nullptr;
    }
    const _BinaryFile &file = *opened;
    _BinaryCompareStats local;
    _binaryStats = stats ? stats : &local;
    CHECK(file.formatVersion == rigExec::RigExecFormatVersion);
    CHECK(_BinaryText(file, file.rig) == evaluator.GetRigPath().GetString());
    CHECK(file.slotMeta && file.constants && file.clustering && file.cones &&
          file.pose && file.geometry);
    if (file.slotMeta) {
        _BinaryCompareSlotMeta(program, *file.slotMeta, file);
    }
    if (file.constants) {
        _BinaryCompareConstants(program, *file.constants, file);
    }
    _BinaryCompareSteps(program, file.steps);
    if (file.clustering) {
        _BinaryCompareClustering(program, *file.clustering);
    }
    if (file.cones) {
        _BinaryCompareCones(program, *file.cones);
    }
    if (file.pose) {
        _BinaryCompareDomainPose(program, *file.pose, file);
    }
    if (file.geometry) {
        _BinaryCompareDomainGeometry(evaluator, program, *file.geometry,
                                     file);
    }
    _BinaryComparePathReads(program, file, fixture);
    _BinaryComparePropertyChains(evaluator, program, file);
    _BinaryCheckOverrides(program, file);
    _BinaryCheckDefaults(program.stage, file);
    _binaryStats = nullptr;
    return opened;
}

}  // namespace
