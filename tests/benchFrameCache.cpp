//
// benchFrameCache -- Stream 0 measurements for the per-frame caching plan.
//
// Measures, on the biped and the 9-mesh rig, the four numbers the plan's
// defaults are made from: per-frame stored bytes (pose maps and slot arenas
// reported separately), baked serial frame cost (the warming budget), and
// the UI-thread cost of sampling one frame's input vector (which bounds the
// neighbor radius). Prints human-readable breakdowns; asserts nothing, so it
// is built but deliberately NOT registered with ctest (like benchPathLookup).
//
//   benchFrameCache <examplesDir> [biped|9mesh|all]
//
// The serial numbers come from running under RIGEXEC_ENABLE_PARALLEL_EVAL=0
// (the binary prints the switch state, so a mistimed run labels itself).
// RIGEXEC_BAKED_STEP_TIMING=N adds the library's own prologue/region/epilogue
// thirds on stderr. The biped is examples/biped/Biped_anim.usda at frames
// 1..8; the 9-mesh rig is the MakeMultiMeshRig construction from
// testRigExecChainLevels with animated controls (time samples at 1..40 on
// the two control avars) and valid envelopes throughout, timed at 1..40.
//
// Method notes, so the report's numbers can be re-derived:
//   * Pose bytes walk every published map of a real evaluated pose and sum
//     payload bytes. SdfPath keys count sizeof(SdfPath); path strings are
//     interned and shared, so they are excluded and said so. std::map node
//     overhead is estimated at 32 bytes per entry (MSVC x64) and reported
//     on its own line, never folded into the payload.
//   * Arena bytes walk the baked program's per-frame working state through
//     GetStepGraph: SSA frame versions, avars, matrices, property results,
//     solver outputs and scratch, geometry outputs and packets, weight
//     packets, chain buffers, and the last-run comparison buffers. Epoch
//     structure (steps, clustering, cones, walk, ladder tables) is excluded.
//     VtArray payloads are counted per holder; copy-on-write sharing between
//     holders (lastAuxPoints, wire tables) overcounts shared buffers and is
//     noted where it applies.
//   * Sampling reads every varying input binding through its retained
//     UsdAttributeQuery, or through its resolved attribute when a property
//     chain stands on the walk -- the route the baked frame path reads -- at
//     a fresh time code, and reports the whole-vector cost plus per-read ns.
//   * Frame cost is min-of-5 pass means over 40 animated frames each, with
//     the minimum clusters-run across the 200 frames proving every timed
//     frame ran the whole program rather than an empty cone.
//
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/parallel.h"
#include "rigExec/rigEvaluator.h"
#include "rigExecMath/pointFrame.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec2f.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/gf/vec3i.h"
#include "pxr/base/gf/vec4f.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/vt/value.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

using namespace rigExec;

PXR_NAMESPACE_USING_DIRECTIVE

namespace {

constexpr size_t kMapNodeOverhead = 32;
constexpr size_t kTimedPasses = 5;
constexpr size_t kFramesPerPass = 40;

template <class T>
size_t
VecBytes(const std::vector<T> &v)
{
    return v.size() * sizeof(T);
}

size_t
StringBytes(const std::string &s)
{
    return s.size();
}

// Payload bytes of one VtValue. Unknown types count the VtValue shell and
// are tallied separately, so a new held type shows up as a count rather
// than silently understating the total.
size_t
VtValueBytes(const VtValue &value, size_t *unknownTypes,
             std::vector<std::string> *unknownNames)
{
    if (value.IsEmpty()) {
        return 0;
    }
#define _RIGEXEC_BENCH_ARRAY(type, element)                       \
    if (value.IsHolding<type>()) {                                \
        return value.UncheckedGet<type>().size() * sizeof(element); \
    }
    _RIGEXEC_BENCH_ARRAY(VtVec3fArray, GfVec3f)
    _RIGEXEC_BENCH_ARRAY(VtVec3dArray, GfVec3d)
    _RIGEXEC_BENCH_ARRAY(VtVec2fArray, GfVec2f)
    _RIGEXEC_BENCH_ARRAY(VtVec4fArray, GfVec4f)
    _RIGEXEC_BENCH_ARRAY(VtFloatArray, float)
    _RIGEXEC_BENCH_ARRAY(VtDoubleArray, double)
    _RIGEXEC_BENCH_ARRAY(VtIntArray, int)
    _RIGEXEC_BENCH_ARRAY(VtBoolArray, bool)
    _RIGEXEC_BENCH_ARRAY(VtStringArray, std::string)
#undef _RIGEXEC_BENCH_ARRAY
#define _RIGEXEC_BENCH_SCALAR(type)              \
    if (value.IsHolding<type>()) {               \
        return sizeof(type);                     \
    }
    _RIGEXEC_BENCH_SCALAR(float)
    _RIGEXEC_BENCH_SCALAR(double)
    _RIGEXEC_BENCH_SCALAR(int)
    _RIGEXEC_BENCH_SCALAR(bool)
    _RIGEXEC_BENCH_SCALAR(GfVec2f)
    _RIGEXEC_BENCH_SCALAR(GfVec3f)
    _RIGEXEC_BENCH_SCALAR(GfVec3d)
    _RIGEXEC_BENCH_SCALAR(GfVec3i)
    _RIGEXEC_BENCH_SCALAR(GfVec4f)
    _RIGEXEC_BENCH_SCALAR(GfMatrix4d)
    _RIGEXEC_BENCH_SCALAR(TfToken)
    _RIGEXEC_BENCH_SCALAR(SdfPath)
#undef _RIGEXEC_BENCH_SCALAR
    if (value.IsHolding<std::string>()) {
        return value.UncheckedGet<std::string>().size();
    }
    ++(*unknownTypes);
    if (unknownNames->size() < 8) {
        unknownNames->push_back(value.GetTypeName());
    }
    return sizeof(VtValue);
}

size_t
WeightPacketBytes(const RigExecWeightPacket &packet)
{
    return VecBytes(packet.values) + VecBytes(packet.indices);
}

size_t
PointFrameArrayBytes(const RigExecPointFrameArray &array)
{
    return VecBytes(array.frames) +
           array.rests.size() * sizeof(std::array<GfVec3d, 4>);
}

size_t
MoverParametersBytes(const RigExecMoverParameters &params)
{
    return WeightPacketBytes(params.weights) +
           VecBytes(params.blendDeltas) + VecBytes(params.topologyCounts) +
           VecBytes(params.topologyIndices) + VecBytes(params.auxPoints) +
           VecBytes(params.auxPointsB) + VecBytes(params.restPoints) +
           VecBytes(params.bindCoords) +
           PointFrameArrayBytes(params.frames) +
           params.wireBindCoords.size() * sizeof(GfVec2f) +
           VecBytes(params.curveKnots) + VecBytes(params.widths) +
           VecBytes(params.skinTransforms) + VecBytes(params.skinIndices) +
           VecBytes(params.skinWeights) +
           VecBytes(params.curvenetAdjustments);
}

// Every published map of one evaluated pose, in payload bytes. Prints the
// per-domain breakdown and returns the payload total (map node overhead is
// reported beside it, not folded in).
size_t
MeasurePoseBytes(const RigExecRigPose &pose, const char *rig)
{
    size_t nodes = 0;
    size_t unknownTypes = 0;
    std::vector<std::string> unknownNames;
    const auto keyBytes = [&nodes](size_t entries) {
        nodes += entries;
        return entries * sizeof(SdfPath);
    };

    size_t total = 0;
    const auto line = [&total](const char *domain, size_t bytes,
                               size_t entries) {
        total += bytes;
        std::printf("  %-22s %10zu bytes  %6zu entries\n", domain, bytes,
                    entries);
    };

    std::printf("[%s] pose bytes (payload; path strings interned/excluded):\n",
                rig);
    line("jointFramesBase",
         keyBytes(pose.jointFramesBase.size()) +
             pose.jointFramesBase.size() * sizeof(RigExecPointFrame),
         pose.jointFramesBase.size());
    line("jointFramesFinal",
         keyBytes(pose.jointFramesFinal.size()) +
             pose.jointFramesFinal.size() * sizeof(RigExecPointFrame),
         pose.jointFramesFinal.size());
    line("jointMatricesFinal",
         keyBytes(pose.jointMatricesFinal.size()) +
             pose.jointMatricesFinal.size() * sizeof(GfMatrix4d),
         pose.jointMatricesFinal.size());
    line("controlFrames",
         keyBytes(pose.controlFrames.size()) +
             pose.controlFrames.size() * sizeof(RigExecPointFrame),
         pose.controlFrames.size());
    line("providerXforms",
         keyBytes(pose.providerXforms.size()) +
             pose.providerXforms.size() * sizeof(GfMatrix4d),
         pose.providerXforms.size());
    line("providerBaseXforms",
         keyBytes(pose.providerBaseXforms.size()) +
             pose.providerBaseXforms.size() * sizeof(GfMatrix4d),
         pose.providerBaseXforms.size());

    size_t solverBytes = keyBytes(pose.solverFrames.size());
    for (const auto &[path, frames] : pose.solverFrames) {
        (void)path;
        solverBytes += VecBytes(frames);
    }
    line("solverFrames", solverBytes, pose.solverFrames.size());

    size_t movedBytes = keyBytes(pose.movedProperties.size());
    for (const auto &[path, value] : pose.movedProperties) {
        (void)path;
        movedBytes +=
            VtValueBytes(value, &unknownTypes, &unknownNames);
    }
    line("movedProperties", movedBytes, pose.movedProperties.size());

    size_t cpuBytes = keyBytes(pose.movedPropertiesCpu.size());
    for (const auto &[path, value] : pose.movedPropertiesCpu) {
        (void)path;
        cpuBytes += VtValueBytes(value, &unknownTypes, &unknownNames);
    }
    line("movedPropertiesCpu", cpuBytes, pose.movedPropertiesCpu.size());

    size_t fieldBytes = keyBytes(pose.weightFields.size());
    for (const auto &[path, field] : pose.weightFields) {
        (void)path;
        fieldBytes += sizeof(SdfPath) + VecBytes(field.weights);
    }
    line("weightFields", fieldBytes, pose.weightFields.size());
    line("weightFrames",
         keyBytes(pose.weightFrames.size()) +
             pose.weightFrames.size() * sizeof(GfMatrix4d),
         pose.weightFrames.size());

    size_t diagBytes = 0;
    for (const std::string &d : pose.diagnostics) {
        diagBytes += StringBytes(d);
    }
    line("diagnostics", diagBytes, pose.diagnostics.size());
    line("struct shell", sizeof(RigExecRigPose), 1);

    std::printf("  %-22s %10zu bytes\n", "POSE PAYLOAD TOTAL", total);
    std::printf("  %-22s %10zu bytes  (est. %zu nodes x %zu)\n",
                "map node overhead", nodes * kMapNodeOverhead, nodes,
                kMapNodeOverhead);
    if (unknownTypes) {
        std::printf("  !! %zu value(s) of unlisted type, counted as shells:",
                    unknownTypes);
        for (const std::string &name : unknownNames) {
            std::printf(" %s", name.c_str());
        }
        std::printf("\n");
    }
    return total;
}

size_t
VtArrayVec3fBytes(const VtVec3fArray &array)
{
    return array.size() * sizeof(GfVec3f);
}

// One geometry revision's per-frame state: outputs, packets, chunks, and the
// last-run comparison copies. Epoch structure (bindings, topology handles,
// blend channel stage handles) is excluded.
size_t
GeomRevisionBytes(
    const RigExecBakedProgramImpl::GeomRevision &revision,
    size_t *unknownTypes, std::vector<std::string> *unknownNames)
{
    (void)unknownTypes;
    (void)unknownNames;
    size_t bytes = VecBytes(revision.output);
    bytes += MoverParametersBytes(revision.lastParameters);
    bytes += MoverParametersBytes(revision.parameters);
    // COW handle to a chain-published array: shares its buffer with the
    // chain result counted below, so this double counts one buffer per
    // derived revision. Counted anyway; the report notes the sharing.
    bytes += VtArrayVec3fBytes(revision.lastAuxPoints);
    bytes += StringBytes(revision.lastStatus.firstBadAddress);
    bytes += VecBytes(revision.envelope) + VecBytes(revision.weightField);
    bytes += VecBytes(revision.influences) + VecBytes(revision.rows) +
             VecBytes(revision.palette);
    bytes += WeightPacketBytes(revision.currentPhasePacket);
    bytes += VecBytes(revision.controlFrames);
    bytes += sizeof(revision.lastAdjusterNetToAsset);
    for (const RigExecBakedProgramImpl::GeomChunk &chunk : revision.chunks) {
        bytes += VecBytes(chunk.transforms) + VecBytes(chunk.rows) +
                 VecBytes(chunk.palette) + VecBytes(chunk.key);
    }
    for (const RigExecBakedProgramImpl::GeomBlendChannel &channel :
         revision.blendChannels) {
        for (const RigExecBakedProgramImpl::GeomBlendChannel::Sample &sample :
             channel.samples) {
            bytes += VecBytes(sample.lastPoints);
            // The sparse layout is epoch-shared; only the handle is per-frame.
            bytes += sizeof(sample.layout);
        }
    }
    return bytes;
}

// The program's per-frame working state through GetStepGraph, in payload
// bytes. Prints the per-domain breakdown and returns the total. Epoch
// structure -- steps, clustering, cones, walk/commits, ladder tables,
// invalidation index, topology and bind caches -- is excluded: it is the
// program, not the arena a cached frame would retain.
size_t
MeasureArenaBytes(const RigExecBakedProgramImpl &B, const char *rig)
{
    size_t unknownTypes = 0;
    std::vector<std::string> unknownNames;
    size_t total = 0;
    const auto line = [&total](const char *domain, size_t bytes) {
        total += bytes;
        std::printf("  %-22s %10zu bytes\n", domain, bytes);
    };

    std::printf("[%s] slot-arena bytes (per-frame state only):\n", rig);
    line("avars+posedM",
         VecBytes(B.avars) + VecBytes(B.lastAvars) + VecBytes(B.posedM));
    line("base+fin versions",
         VecBytes(B.base) + VecBytes(B.fin) + VecBytes(B.finLast) +
             VecBytes(B.baseLast));
    line("rest->pose matrices",
         VecBytes(B.finalMatrix) + VecBytes(B.baseMatrix));
    line("xform seeds",
         VecBytes(B.xformBase) + VecBytes(B.lastXformBase));
    line("native sources",
         VecBytes(B.nativeFrames) + VecBytes(B.lastNativeFrames) +
             VecBytes(B.nativeFrameOk) + VecBytes(B.lastNativeFrameOk));
    line("pose weights", VecBytes(B.poseWeights));

    size_t propertyBytes = 0;
    size_t propertyEntries = 0;
    for (const auto &[path, value] : B.propertyResults) {
        (void)path;
        ++propertyEntries;
        propertyBytes +=
            VtValueBytes(value, &unknownTypes, &unknownNames);
    }
    for (const auto &[path, value] : B.lastPropertyResults) {
        (void)path;
        ++propertyEntries;
        propertyBytes +=
            VtValueBytes(value, &unknownTypes, &unknownNames);
    }
    propertyBytes +=
        propertyEntries * (sizeof(SdfPath) + kMapNodeOverhead);
    line("property results", propertyBytes);

    size_t aggregateBytes = 0;
    for (const RigExecPointFrameArray &array : B.aggregates) {
        aggregateBytes += PointFrameArrayBytes(array);
    }
    line("solver aggregates", aggregateBytes);

    size_t solverBytes = 0;
    for (const RigExecBakedProgramImpl::Solver &solver : B.solvers) {
        solverBytes += VecBytes(solver.outFrames) +
                       VecBytes(solver.outPresent) +
                       VecBytes(solver.outPosition) +
                       VecBytes(solver.elements) +
                       VecBytes(solver.controlReads) +
                       solver.fallbackJoints.size() * sizeof(SdfPath) +
                       VecBytes(solver.ribbonPoints) +
                       VecBytes(solver.lastRibbonPoints);
        // The rest description is rebuilt per frame only for a solver whose
        // rests can move within the epoch; otherwise it is epoch state.
        if (solver.restsVary || solver.hasLiveRest) {
            solverBytes +=
                VecBytes(solver.restSlots) + VecBytes(solver.restRefs) +
                VecBytes(solver.restReads) +
                VecBytes(solver.jointRests) +
                VecBytes(solver.restOverrides) +
                VecBytes(solver.splineRestWeights) +
                VecBytes(solver.controlRests) +
                VecBytes(solver.splineJointRests) +
                VecBytes(solver.twistWeights);
        }
    }
    line("solver outputs", solverBytes);

    size_t constraintBytes = 0;
    for (const RigExecBakedProgramImpl::Constraint &constraint :
         B.constraints) {
        constraintBytes += VecBytes(constraint.weightScratch) +
                           StringBytes(constraint.weightError);
    }
    for (const RigExecBakedProgramImpl::ConstraintArrays &arrays :
         B.constraintArrays) {
        constraintBytes += VecBytes(arrays.weights) +
                           VecBytes(arrays.translationOffsets) +
                           VecBytes(arrays.rotationOffsets) +
                           VecBytes(arrays.lastWeights) +
                           VecBytes(arrays.lastTranslationOffsets) +
                           VecBytes(arrays.lastRotationOffsets) +
                           VecBytes(arrays.poleWeights) +
                           VecBytes(arrays.lastPoleWeights);
        for (const std::string &d : arrays.diagnostics) {
            constraintBytes += StringBytes(d);
        }
        for (const std::string &d : arrays.lastDiagnostics) {
            constraintBytes += StringBytes(d);
        }
        for (const std::string &d : arrays.poleDiagnostics) {
            constraintBytes += StringBytes(d);
        }
        for (const std::string &d : arrays.lastPoleDiagnostics) {
            constraintBytes += StringBytes(d);
        }
    }
    line("constraint scratch", constraintBytes);

    size_t chainBytes = 0;
    size_t revisionCount = 0;
    for (const RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        chainBytes += VtArrayVec3fBytes(chain.lastBase) +
                      VtArrayVec3fBytes(chain.result) +
                      VtArrayVec3fBytes(chain.spare);
        for (const RigExecBakedProgramImpl::GeomRevision &revision :
             chain.revisions) {
            ++revisionCount;
            chainBytes += GeomRevisionBytes(revision, &unknownTypes,
                                            &unknownNames);
        }
        for (const RigExecBakedProgramImpl::GeomChain::Derived &derived :
             chain.derived) {
            ++revisionCount;
            chainBytes += VtArrayVec3fBytes(derived.lastBase) +
                          VtArrayVec3fBytes(derived.result) +
                          VtArrayVec3fBytes(derived.spare);
            chainBytes += GeomRevisionBytes(derived.revision, &unknownTypes,
                                            &unknownNames);
        }
    }
    line("geometry chains", chainBytes);

    size_t weightBytes = 0;
    for (const RigExecWeightPacket &packet : B.weightPackets) {
        weightBytes += WeightPacketBytes(packet);
    }
    for (const RigExecBakedProgramImpl::WeightObject &object :
         B.weightObjects) {
        weightBytes += VecBytes(object.boundMesh) +
                       VecBytes(object.boundNet) +
                       VecBytes(object.boundCounts) +
                       VecBytes(object.boundIndices) +
                       VecBytes(object.boundSplines) +
                       VecBytes(object.boundSmooth);
    }
    line("weight packets", weightBytes);

    line("constraint deltas",
         VecBytes(B.deltaValues) + VecBytes(B.deltaPresent) +
             VecBytes(B.deltaBaseMatrix) + VecBytes(B.lastDeltaBaseMatrix) +
             VecBytes(B.deltaBaseOk) + VecBytes(B.lastDeltaBaseOk));
    line("cone state",
         VecBytes(B.lastOverridden) + VecBytes(B.lastHaveBase));
    // The provider ladder: per-frame state only while a rest channel can
    // move within the epoch, epoch-constant tables otherwise.
    const size_t ladderBytes =
        VecBytes(B.restM) + VecBytes(B.restPts) + VecBytes(B.restFrames) +
        VecBytes(B.selfD) + VecBytes(B.parentDinv) + VecBytes(B.rotOrder) +
        VecBytes(B.posedAuthoredM) + VecBytes(B.posedAuthored) +
        VecBytes(B.restRoundTrip) + VecBytes(B.defaultRoundTrip) +
        VecBytes(B.lastRestM) + VecBytes(B.lastSelfD) +
        VecBytes(B.lastParentDinv) + VecBytes(B.lastPosedAuthoredM) +
        VecBytes(B.lastRotOrder) + VecBytes(B.lastPosedAuthored) +
        VecBytes(B.noScaleAvars) + VecBytes(B.restChainVaries);
    if (B.ladderVarying) {
        line("ladder tables", ladderBytes);
    } else {
        std::printf("  %-22s %10zu bytes  (epoch-constant, excluded)\n",
                    "ladder tables", ladderBytes);
    }
    std::printf("  (%srunSnapshots %s; revision inputs unsized: counts "
                "unavailable)\n",
                B.runSnapshots.IsEmpty() ? "" : "NON-EMPTY ",
                B.runSnapshots.IsEmpty() ? "empty" : "held");
    std::printf("  %-22s %10zu bytes  (%zu revisions)\n", "ARENA TOTAL", total,
                revisionCount);
    if (unknownTypes) {
        std::printf("  !! %zu value(s) of unlisted type, counted as shells:",
                    unknownTypes);
        for (const std::string &name : unknownNames) {
            std::printf(" %s", name.c_str());
        }
        std::printf("\n");
    }
    return total;
}

// ---------------------------------------------------------------------------
// Input sampling: the UI-thread cost of one frame's input vector.
// ---------------------------------------------------------------------------

// Collects one binding's read handle when it is a per-frame input: the
// retained query when USD alone answers, the resolved attribute when a
// property chain stands on the walk -- the same route the baked frame path
// takes. Counts override-registered inputs beside the reads: standing
// overrides are not sampled per frame, but the control digest covers them.
template <class T>
void
CollectBinding(const RigExecBakedInput<T> &input,
               std::vector<UsdAttributeQuery> *queries,
               std::vector<UsdAttribute> *chainAttrs, size_t *overrides)
{
    if (input.overrideIndex >= 0) {
        ++(*overrides);
    }
    if (!input.varying) {
        return;
    }
    if (input.query.IsValid()) {
        queries->push_back(input.query);
    } else if (input.resolvedAttr.IsValid()) {
        chainAttrs->push_back(input.resolvedAttr);
    }
}

void
CollectSolverBindings(const RigExecBakedProgramImpl::Solver &solver,
                      std::vector<UsdAttributeQuery> *queries,
                      std::vector<UsdAttribute> *chainAttrs,
                      size_t *overrides)
{
    CollectBinding(solver.bend, queries, chainAttrs, overrides);
    CollectBinding(solver.upperOffset, queries, chainAttrs, overrides);
    CollectBinding(solver.lowerOffset, queries, chainAttrs, overrides);
    CollectBinding(solver.stretch, queries, chainAttrs, overrides);
    CollectBinding(solver.softness, queries, chainAttrs, overrides);
    CollectBinding(solver.blendWeight, queries, chainAttrs, overrides);
    CollectBinding(solver.preserveVolume, queries, chainAttrs, overrides);
    CollectBinding(solver.midFollowWeight, queries, chainAttrs, overrides);
    CollectBinding(solver.roll, queries, chainAttrs, overrides);
    CollectBinding(solver.twist, queries, chainAttrs, overrides);
    CollectBinding(solver.minLengthRatio, queries, chainAttrs, overrides);
    CollectBinding(solver.twistTurns, queries, chainAttrs, overrides);
    CollectBinding(solver.ribbonSampleCount, queries, chainAttrs, overrides);
}

void
CollectConstraintBindings(
    const RigExecBakedProgramImpl::Constraint &constraint,
    std::vector<UsdAttributeQuery> *queries,
    std::vector<UsdAttribute> *chainAttrs, size_t *overrides)
{
    CollectBinding(constraint.enabled, queries, chainAttrs, overrides);
    CollectBinding(constraint.defaultWeight, queries, chainAttrs, overrides);
    CollectBinding(constraint.offset, queries, chainAttrs, overrides);
    CollectBinding(constraint.affectX, queries, chainAttrs, overrides);
    CollectBinding(constraint.affectY, queries, chainAttrs, overrides);
    CollectBinding(constraint.affectZ, queries, chainAttrs, overrides);
    CollectBinding(constraint.tX, queries, chainAttrs, overrides);
    CollectBinding(constraint.tY, queries, chainAttrs, overrides);
    CollectBinding(constraint.tZ, queries, chainAttrs, overrides);
    CollectBinding(constraint.rX, queries, chainAttrs, overrides);
    CollectBinding(constraint.rY, queries, chainAttrs, overrides);
    CollectBinding(constraint.rZ, queries, chainAttrs, overrides);
    CollectBinding(constraint.sX, queries, chainAttrs, overrides);
    CollectBinding(constraint.sY, queries, chainAttrs, overrides);
    CollectBinding(constraint.sZ, queries, chainAttrs, overrides);
    CollectBinding(constraint.aimVector, queries, chainAttrs, overrides);
    CollectBinding(constraint.upVector, queries, chainAttrs, overrides);
    CollectBinding(constraint.rotationOffset, queries, chainAttrs, overrides);
    CollectBinding(constraint.worldUpVector, queries, chainAttrs, overrides);
    CollectBinding(constraint.poleVector, queries, chainAttrs, overrides);
    CollectBinding(constraint.twistDegrees, queries, chainAttrs, overrides);
}

void
CollectWeightBindings(const RigExecBakedProgramImpl::WeightObject &object,
                      std::vector<UsdAttributeQuery> *queries,
                      std::vector<UsdAttribute> *chainAttrs,
                      size_t *overrides)
{
    CollectBinding(object.defaultWeight, queries, chainAttrs, overrides);
    CollectBinding(object.driver, queries, chainAttrs, overrides);
    CollectBinding(object.scale, queries, chainAttrs, overrides);
    CollectBinding(object.bias, queries, chainAttrs, overrides);
    CollectBinding(object.strength, queries, chainAttrs, overrides);
    CollectBinding(object.invert, queries, chainAttrs, overrides);
    CollectBinding(object.falloffMin, queries, chainAttrs, overrides);
    CollectBinding(object.falloffMax, queries, chainAttrs, overrides);
    CollectBinding(object.scaleX, queries, chainAttrs, overrides);
    CollectBinding(object.scaleY, queries, chainAttrs, overrides);
    CollectBinding(object.scaleZ, queries, chainAttrs, overrides);
    CollectBinding(object.extentU, queries, chainAttrs, overrides);
    CollectBinding(object.extentV, queries, chainAttrs, overrides);
    CollectBinding(object.curvenetSamples, queries, chainAttrs, overrides);
    CollectBinding(object.curvenetUnreached, queries, chainAttrs, overrides);
}

// Reads the whole sampled vector at \p time, the way the UI thread will at
// enqueue: every varying binding through its retained handle, plus the blend
// channel stage reads the prologue makes outside the binding table.
void
MeasureSampling(const RigExecBakedProgramImpl &B, UsdTimeCode time,
                const char *rig)
{
    std::vector<UsdAttributeQuery> queries;
    std::vector<UsdAttribute> chainAttrs;
    size_t overrides = 0;
    for (const RigExecBakedProgramImpl::AvarBinding &binding : B.avarBindings) {
        CollectBinding(binding.input, &queries, &chainAttrs, &overrides);
    }
    for (const RigExecBakedProgramImpl::AvarBinding &binding :
         B.avarConstantBindings) {
        // Constants are epoch state, but a drag can promote one: count the
        // override registration, not a read.
        if (binding.input.overrideIndex >= 0) {
            ++overrides;
        }
    }
    for (const RigExecBakedProgramImpl::Ladder &ladder : B.ladders) {
        CollectBinding(ladder.restSpace, &queries, &chainAttrs, &overrides);
        CollectBinding(ladder.defaultSpace, &queries, &chainAttrs, &overrides);
        CollectBinding(ladder.posedSpace, &queries, &chainAttrs, &overrides);
        for (int i = 0; i < 6; ++i) {
            CollectBinding(ladder.restAvars[i], &queries, &chainAttrs,
                           &overrides);
            CollectBinding(ladder.defaultAvars[i], &queries, &chainAttrs,
                           &overrides);
        }
        CollectBinding(ladder.rotationOrder, &queries, &chainAttrs,
                       &overrides);
    }
    for (const RigExecBakedProgramImpl::PoseInterpolator &interp :
         B.poseInterpolators) {
        CollectBinding(interp.enabled, &queries, &chainAttrs, &overrides);
    }
    for (const RigExecBakedProgramImpl::Solver &solver : B.solvers) {
        CollectSolverBindings(solver, &queries, &chainAttrs, &overrides);
    }
    for (const RigExecBakedProgramImpl::Constraint &constraint :
         B.constraints) {
        CollectConstraintBindings(constraint, &queries, &chainAttrs,
                                 &overrides);
    }
    for (const RigExecBakedProgramImpl::WeightObject &object :
         B.weightObjects) {
        CollectWeightBindings(object, &queries, &chainAttrs, &overrides);
    }
    // Blend channel reads outside the binding table: the channel weight
    // (unless it resolves to a pose slot) and every sample's activation and
    // dense points.
    std::vector<UsdAttribute> blendAttrs;
    for (const RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        for (const RigExecBakedProgramImpl::GeomRevision &revision :
             chain.revisions) {
            for (const RigExecBakedProgramImpl::GeomBlendChannel &channel :
                 revision.blendChannels) {
                if (channel.poseWeight < 0 && channel.weight.IsValid()) {
                    blendAttrs.push_back(channel.weight);
                }
                for (const RigExecBakedProgramImpl::GeomBlendChannel::Sample
                         &sample : channel.samples) {
                    if (sample.activation.IsValid()) {
                        blendAttrs.push_back(sample.activation);
                    }
                    if (sample.points.IsValid()) {
                        blendAttrs.push_back(sample.points);
                    }
                }
            }
        }
    }

    // Constraint operator arrays: read raw off the prim per frame by the
    // prologue, so sampled too. Same attribute names, type-erased reads.
    struct OperatorArrayRead {
        UsdPrim prim;
        TfToken name;
    };
    std::vector<OperatorArrayRead> arrayReads;
    for (const RigExecBakedProgramImpl::ConstraintArrays &arrays :
         B.constraintArrays) {
        arrayReads.push_back({arrays.prim, TfToken("inputs:sourceWeights")});
        if (arrays.parentOffsets) {
            arrayReads.push_back(
                {arrays.prim, TfToken("inputs:translationOffsets")});
            arrayReads.push_back(
                {arrays.prim, TfToken("inputs:rotationOffsets")});
        }
        if (arrays.readPole) {
            arrayReads.push_back(
                {arrays.prim, TfToken("inputs:poleVectorWeights")});
        }
    }
    // Ribbon driver points: the prologue reads the live curve per frame
    // when it can vary.
    std::vector<UsdAttributeQuery> ribbonQueries;
    for (const RigExecBakedProgramImpl::Solver &solver : B.solvers) {
        if (solver.ribbonPointsVarying &&
            solver.ribbonPointsQuery.IsValid()) {
            ribbonQueries.push_back(solver.ribbonPointsQuery);
        }
    }
    constexpr int kRepeats = 200;
    VtValue scratch;
    const auto start = std::chrono::steady_clock::now();
    for (int r = 0; r < kRepeats; ++r) {
        for (const UsdAttributeQuery &query : queries) {
            query.Get(&scratch, time);
        }
        for (const UsdAttribute &attr : chainAttrs) {
            attr.Get(&scratch, time);
        }
        for (const UsdAttribute &attr : blendAttrs) {
            attr.Get(&scratch, time);
        }
        for (const OperatorArrayRead &read : arrayReads) {
            read.prim.GetAttribute(read.name).Get(&scratch, time);
        }
        for (const UsdAttributeQuery &query : ribbonQueries) {
            query.Get(&scratch, time);
        }
    }
    const auto end = std::chrono::steady_clock::now();
    const double totalUs =
        std::chrono::duration<double, std::micro>(end - start).count() /
        kRepeats;
    const size_t reads = queries.size() + chainAttrs.size() +
                         blendAttrs.size() + arrayReads.size() +
                         ribbonQueries.size();
    std::printf("[%s] sampling one frame's input vector at t=%.1f:\n", rig,
                time.GetValue());
    std::printf("  %-22s %10zu\n", "query reads", queries.size());
    std::printf("  %-22s %10zu\n", "chain-resolved reads",
                chainAttrs.size());
    std::printf("  %-22s %10zu\n", "blend channel reads",
                blendAttrs.size());
    std::printf("  %-22s %10zu\n", "operator array reads",
                arrayReads.size());
    std::printf("  %-22s %10zu\n", "ribbon point reads",
                ribbonQueries.size());
    std::printf("  %-22s %10zu\n", "override-registered inputs",
                overrides);
    std::printf("  %-22s %10.1f us/frame\n", "sample cost", totalUs);
    std::printf("  %-22s %10.1f ns/read\n", "per read",
                reads ? totalUs * 1000.0 / double(reads) : 0.0);
    // Chain base points: the prologue reads every chain base off the stage
    // per frame, and a worker cannot -- so the UI thread samples these
    // too. Timed apart from the bindings because the cost is bytes, not
    // reads.
    std::vector<UsdAttributeQuery> baseQueries;
    for (const RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        if (chain.baseQuery.IsValid()) {
            baseQueries.push_back(chain.baseQuery);
        }
        for (const RigExecBakedProgramImpl::GeomChain::Derived &derived :
             chain.derived) {
            if (derived.baseQuery.IsValid()) {
                baseQueries.push_back(derived.baseQuery);
            }
        }
    }
    VtVec3fArray baseScratch;
    size_t baseBytes = 0;
    for (const UsdAttributeQuery &query : baseQueries) {
        if (query.Get(&baseScratch, time)) {
            baseBytes += baseScratch.size() * sizeof(GfVec3f);
        }
    }
    const auto baseStart = std::chrono::steady_clock::now();
    for (int r = 0; r < kRepeats; ++r) {
        for (const UsdAttributeQuery &query : baseQueries) {
            query.Get(&baseScratch, time);
        }
    }
    const auto baseEnd = std::chrono::steady_clock::now();
    const double baseUs =
        std::chrono::duration<double, std::micro>(baseEnd - baseStart).count() /
        kRepeats;
    std::printf("  %-22s %10zu\n", "base geometry reads",
                baseQueries.size());
    std::printf("  %-22s %10zu bytes\n", "base points", baseBytes);
    std::printf("  %-22s %10.1f us/frame\n", "base read cost", baseUs);
    std::printf("  %-22s %10.1f us/frame\n", "sample+base cost",
                totalUs + baseUs);
}

// ---------------------------------------------------------------------------
// Rigs.
// ---------------------------------------------------------------------------

SdfPath
FindRig(const UsdStageRefPtr &stage)
{
    for (const UsdPrim &prim : stage->TraverseAll()) {
        if (prim.GetTypeName() == TfToken("RigExecRoot")) {
            return prim.GetPath();
        }
    }
    return SdfPath();
}

// The MakeMultiMeshRig construction from testRigExecChainLevels: meshCount
// skinned meshes over two shared controls, distinct geometry and weights per
// mesh. Two deliberate deviations, both stated: the controls carry time
// samples (1..40) so consecutive frames genuinely re-evaluate instead of
// cone-skipping a static rig, and every envelope is valid, so the cost
// measured is the skin kernel's rather than the pass-through's.
UsdStageRefPtr
MakeAnimatedMultiMeshRig(size_t meshCount)
{
    constexpr size_t kPointCount = RigExecGeometryParallelThreshold + 37;
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const UsdPrim alongX = stage->DefinePrim(
        SdfPath("/Asset/Rig/AlongX"), TfToken("RigExecControl"));
    const UsdPrim alongY = stage->DefinePrim(
        SdfPath("/Asset/Rig/AlongY"), TfToken("RigExecControl"));
    UsdAttribute tx = alongX.GetAttribute(TfToken("avars:tx"));
    UsdAttribute ty = alongY.GetAttribute(TfToken("avars:ty"));
    for (int t = 1; t <= 40; ++t) {
        tx.Set(10.0 + 0.1 * double(t), UsdTimeCode(double(t)));
        ty.Set(20.0 - 0.05 * double(t), UsdTimeCode(double(t)));
    }
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));

    VtIntArray indices(kPointCount * 2);
    for (size_t i = 0; i < kPointCount; ++i) {
        indices[i * 2] = 0;
        indices[i * 2 + 1] = 1;
    }
    for (size_t mesh = 0; mesh < meshCount; ++mesh) {
        const SdfPath meshPath(
            TfStringPrintf("/Asset/Geom/Mesh_%zu", mesh));
        const UsdPrim prim = stage->DefinePrim(meshPath, TfToken("Mesh"));
        VtVec3fArray points(kPointCount);
        for (size_t i = 0; i < kPointCount; ++i) {
            points[i] = GfVec3f(float(i) * 0.5f + float(mesh),
                                float(i) * -0.25f,
                                float(mesh) * 3.0f - float(i) * 0.125f);
        }
        prim.GetAttribute(TfToken("points")).Set(points);

        const UsdPrim skin = stage->DefinePrim(
            SdfPath(TfStringPrintf("/Asset/Rig/Movers/Skin_%zu", mesh)),
            TfToken("RigExecSkinMover"));
        skin.ApplyAPI(TfToken("RigExecMoverAPI"));
        skin.GetRelationship(TfToken("rigExec:moves"))
            .SetTargets(
                {meshPath.AppendProperty(TfToken("points"))});
        skin.GetAttribute(TfToken("inputs:defaultWeight")).Set(1.0f);
        skin.CreateRelationship(TfToken("rigExec:influences"))
            .SetTargets({alongX.GetPath(), alongY.GetPath()});
        skin.CreateAttribute(TfToken("rigExec:elementSize"),
                             SdfValueTypeNames->Int).Set(2);
        skin.CreateAttribute(TfToken("rigExec:jointIndices"),
                             SdfValueTypeNames->IntArray).Set(indices);
        VtFloatArray weights(kPointCount * 2);
        const float xWeight = 0.1f + 0.05f * float(mesh);
        const float yWeight = 0.9f - 0.05f * float(mesh);
        for (size_t i = 0; i < kPointCount; ++i) {
            weights[i * 2] = xWeight;
            weights[i * 2 + 1] = yWeight;
        }
        skin.CreateAttribute(TfToken("rigExec:jointWeights"),
                             SdfValueTypeNames->FloatArray).Set(weights);
    }
    return stage;
}

double
NowMs()
{
    return std::chrono::duration<double, std::milli>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

// Full measurement of one rig. Returns false (having printed why) when the
// rig does not compile or does not bake: the numbers below it would be the
// dynamic path's, mislabeled.
bool
MeasureRig(const char *rig, const UsdStageRefPtr &stage,
           const std::vector<double> &frameTimes, double sampleTime)
{
    std::printf("== %s ==\n", rig);
    RigExecRigEvaluator evaluator(stage, FindRig(stage));
    std::vector<std::string> errors;
    const double compileStart = NowMs();
    if (!evaluator.Compile(&errors)) {
        std::printf("FATAL: %s did not compile\n", rig);
        for (const std::string &error : errors) {
            std::printf("  %s\n", error.c_str());
        }
        return false;
    }
    std::printf("  %-22s %10.1f ms\n", "compile",
                NowMs() - compileStart);

    std::vector<std::string> reasons;
    if (!RigExecBakedProgram::IsBakeable(evaluator, &reasons)) {
        std::printf("FATAL: %s does not bake\n", rig);
        for (const std::string &reason : reasons) {
            std::printf("  %s\n", reason.c_str());
        }
        return false;
    }
    const double bakeStart = NowMs();
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::printf("  %-22s %10.1f ms\n", "bake", NowMs() - bakeStart);
    const RigExecBakedProgram *program = evaluator.GetBakedProgram();
    if (!program) {
        std::printf("FATAL: %s built no program\n", rig);
        return false;
    }
    std::printf("  %-22s %10zu\n", "providers",
                program->GetProviderCount());
    std::printf("  %-22s %10zu\n", "bound inputs",
                program->GetBoundInputCount());
    std::printf("  %-22s %10zu\n", "varying inputs",
                program->GetVaryingInputCount());
    std::printf("  %-22s %10zu\n", "clusters",
                program->GetClusterCount());

    // The cold frame: first generation out of a fresh program.
    const double coldStart = NowMs();
    RigExecRigPose first =
        evaluator.Evaluate(UsdTimeCode(frameTimes[0]));
    const double coldMs = NowMs() - coldStart;
    if (!first.valid) {
        std::printf("FATAL: %s first frame invalid\n", rig);
        return false;
    }
    std::printf("  %-22s %10.1f us\n", "cold first frame", coldMs * 1000.0);
    std::printf("  %-22s %10zu of %zu\n", "cold clusters run",
                program->GetClustersRunLastGeneration(),
                program->GetClusterCount());

    const size_t poseBytes = MeasurePoseBytes(first, rig);
    // A second frame's total beside the first: the stored-bytes number is
    // only a number if it does not wander frame to frame.
    RigExecRigPose second =
        evaluator.Evaluate(UsdTimeCode(frameTimes[1]));
    const size_t poseBytes2 = MeasurePoseBytes(second, rig);
    std::printf("  pose bytes stable across frames: %s (%zu vs %zu)\n",
                poseBytes == poseBytes2 ? "yes" : "NO", poseBytes,
                poseBytes2);

    const size_t arenaBytes =
        MeasureArenaBytes(program->GetStepGraph(), rig);
    std::printf("  stored bytes per frame: %zu pose + %zu arena = %zu\n",
                poseBytes, arenaBytes, poseBytes + arenaBytes);

    MeasureSampling(program->GetStepGraph(), UsdTimeCode(sampleTime), rig);

    // Warm frames: min-of-5 pass means over animated frames, with the
    // minimum clusters-run proving every timed frame ran the whole program.
    std::vector<double> passMeans;
    double fastestUs = 0.0;
    size_t minClustersRun = program->GetClusterCount();
    size_t frames = 0;
    for (size_t pass = 0; pass < kTimedPasses; ++pass) {
        const auto start = std::chrono::steady_clock::now();
        for (size_t i = 0; i < kFramesPerPass; ++i) {
            const double t = frameTimes[i % frameTimes.size()];
            RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode(t));
            if (!pose.valid) {
                std::printf("FATAL: %s frame %g invalid\n", rig, t);
                return false;
            }
            minClustersRun = std::min(
                minClustersRun,
                evaluator.GetBakedProgram()
                    ->GetClustersRunLastGeneration());
            ++frames;
        }
        const auto end = std::chrono::steady_clock::now();
        const double meanUs =
            std::chrono::duration<double, std::micro>(end - start).count() /
            double(kFramesPerPass);
        passMeans.push_back(meanUs);
        fastestUs = fastestUs == 0.0 ? meanUs : std::min(fastestUs, meanUs);
    }
    std::sort(passMeans.begin(), passMeans.end());
    const double minMean = passMeans.front();
    const double medianMean = passMeans[passMeans.size() / 2];
    std::printf("[%s] baked frame cost (%zu frames x %zu passes):\n", rig,
                kFramesPerPass, kTimedPasses);
    std::printf("  %-22s %10.1f us/frame\n", "min-of-5 pass mean",
                minMean);
    std::printf("  %-22s %10.1f us/frame\n", "median pass mean",
                medianMean);
    std::printf("  %-22s %10zu of %zu\n", "min clusters run",
                minClustersRun, program->GetClusterCount());
    std::printf("  %-22s %10zu (asked %zu)\n", "baked generations",
                evaluator.GetBakedGenerationCount(), frames + 2);
    return true;
}

std::string
SchemaResourceDir()
{
#ifdef RIGEXEC_SCHEMA_RESOURCE_DIR
    return TfAbsPath(RIGEXEC_SCHEMA_RESOURCE_DIR);
#else
    return std::string();
#endif
}

}  // namespace

int
main(int argc, char **argv)
{
    if (argc < 2) {
        std::printf("usage: benchFrameCache <examplesDir> "
                    "[biped|9mesh|all]\n");
        return 2;
    }
    const std::string examplesDir = argv[1];
    const std::string which = argc > 2 ? argv[2] : "all";
    if (which != "biped" && which != "9mesh" && which != "all") {
        std::printf("usage: benchFrameCache <examplesDir> "
                    "[biped|9mesh|all]\n");
        return 2;
    }

    const std::string resources = SchemaResourceDir();
    if (!resources.empty() &&
        PlugRegistry::GetInstance().RegisterPlugins(resources).empty()) {
        std::printf("FATAL: no schema plugin found at %s\n",
                    resources.c_str());
        return 2;
    }

    std::printf("parallel eval: %s\n",
                RigExecParallelEvaluationEnabled() ? "ON" : "OFF (serial)");
    std::printf("sizeof(RigExecPointFrame)=%zu sizeof(GfMatrix4d)=%zu "
                "sizeof(SdfPath)=%zu sizeof(VtValue)=%zu\n",
                sizeof(RigExecPointFrame), sizeof(GfMatrix4d),
                sizeof(SdfPath), sizeof(VtValue));

    bool ok = true;
    if (which == "biped" || which == "all") {
        const std::string stagePath =
            examplesDir + "/biped/Biped_anim.usda";
        UsdStageRefPtr stage = UsdStage::Open(stagePath);
        if (!stage) {
            std::printf("FATAL: cannot open %s\n", stagePath.c_str());
            return 2;
        }
        ok = MeasureRig("biped", stage, {1, 2, 3, 4, 5, 6, 7, 8},
                        /*sampleTime=*/1001.0) &&
             ok;
    }
    if (which == "9mesh" || which == "all") {
        std::vector<double> times;
        for (int t = 1; t <= 40; ++t) {
            times.push_back(double(t));
        }
        ok = MeasureRig("9mesh", MakeAnimatedMultiMeshRig(9), times,
                        /*sampleTime=*/41.0) &&
             ok;
    }
    return ok ? 0 : 2;
}
