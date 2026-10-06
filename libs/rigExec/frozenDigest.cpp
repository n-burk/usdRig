// Exact frame-input and epoch-constant fingerprints.

#include "frozenContextInternal.h"
#include "pxr/base/gf/vec2f.h"
#include "pxr/base/gf/vec3i.h"
#include "pxr/base/gf/vec4f.h"
#include <cstring>
#include <algorithm>
#include <cmath>
#include <set>

namespace rigExec {

using namespace frozenDetail;

namespace frozenDetail {

uint64_t
_HashBytes(uint64_t hash, const void *data, size_t size)
{
    const unsigned char *bytes = static_cast<const unsigned char *>(data);
    while (size >= 8) {
        uint64_t word;
        std::memcpy(&word, bytes, sizeof(word));
        hash = _MixWord(hash, word);
        bytes += 8;
        size -= 8;
    }
    if (size > 0) {
        // Zero-padded, with the length mixed in so a short tail cannot
        // equal a longer stream's shared prefix. An empty hash stays a
        // no-op, as before.
        uint64_t tail = 0;
        std::memcpy(&tail, bytes, size);
        hash = _MixWord(hash, tail + size);
    }
    return hash;
}

uint64_t
_HashString(uint64_t hash, const char *text)
{
    return _HashBytes(hash, text, std::strlen(text));
}

// Mixes a VtValue's bytes. Returns false for a type this function does not
// name, having mixed only the type name: the caller reports the digest
// inexact rather than risk two different frames sharing one key.
bool
_HashVtValue(uint64_t *hash, const VtValue &value)
{
    *hash = _HashString(*hash, value.GetTypeName().c_str());
    if (value.IsEmpty()) {
        return true;
    }
#define _RIGEXEC_FROZEN_SCALAR(type)                  \
    if (value.IsHolding<type>()) {                    \
        const type &held = value.UncheckedGet<type>(); \
        *hash = _HashBytes(*hash, &held, sizeof(held)); \
        return true;                                  \
    }
    _RIGEXEC_FROZEN_SCALAR(bool)
    _RIGEXEC_FROZEN_SCALAR(int)
    _RIGEXEC_FROZEN_SCALAR(float)
    _RIGEXEC_FROZEN_SCALAR(double)
    _RIGEXEC_FROZEN_SCALAR(GfVec2f)
    _RIGEXEC_FROZEN_SCALAR(GfVec3f)
    _RIGEXEC_FROZEN_SCALAR(GfVec3d)
    _RIGEXEC_FROZEN_SCALAR(GfVec3i)
    _RIGEXEC_FROZEN_SCALAR(GfVec4f)
    _RIGEXEC_FROZEN_SCALAR(GfMatrix4d)
#undef _RIGEXEC_FROZEN_SCALAR
#define _RIGEXEC_FROZEN_ARRAY(type, element)                       \
    if (value.IsHolding<type>()) {                                 \
        const type &held = value.UncheckedGet<type>();              \
        const size_t count = held.size();                          \
        *hash = _HashBytes(*hash, &count, sizeof(count));           \
        if (!held.empty()) {                                       \
            *hash = _HashBytes(*hash, held.cdata(),                 \
                               held.size() * sizeof(element));      \
        }                                                          \
        return true;                                               \
    }
    _RIGEXEC_FROZEN_ARRAY(VtBoolArray, bool)
    _RIGEXEC_FROZEN_ARRAY(VtIntArray, int)
    _RIGEXEC_FROZEN_ARRAY(VtFloatArray, float)
    _RIGEXEC_FROZEN_ARRAY(VtDoubleArray, double)
    _RIGEXEC_FROZEN_ARRAY(VtVec2fArray, GfVec2f)
    _RIGEXEC_FROZEN_ARRAY(VtVec3fArray, GfVec3f)
    _RIGEXEC_FROZEN_ARRAY(VtVec3dArray, GfVec3d)
    _RIGEXEC_FROZEN_ARRAY(VtVec4fArray, GfVec4f)
#undef _RIGEXEC_FROZEN_ARRAY
    if (value.IsHolding<std::string>()) {
        *hash = _HashString(*hash, value.UncheckedGet<std::string>().c_str());
        return true;
    }
    if (value.IsHolding<TfToken>()) {
        *hash = _HashString(*hash, value.UncheckedGet<TfToken>().GetText());
        return true;
    }
    if (value.IsHolding<SdfPath>()) {
        *hash = _HashString(
            *hash, value.UncheckedGet<SdfPath>().GetString().c_str());
        return true;
    }
    if (value.IsHolding<VtStringArray>()) {
        const VtStringArray &held = value.UncheckedGet<VtStringArray>();
        const size_t count = held.size();
        *hash = _HashBytes(*hash, &count, sizeof(count));
        for (const std::string &entry : held) {
            *hash = _HashString(*hash, entry.c_str());
        }
        return true;
    }
    return false;
}

} // namespace frozenDetail

void
RigExecFrameInputs::Add(const SdfPath &path, const VtValue &value,
                        bool hasValue, bool viaChain)
{
    RigExecSampledInput sampled;
    sampled.path = path;
    sampled.value = value;
    sampled.hasValue = hasValue;
    sampled.viaChain = viaChain;
    values.push_back(sampled);
}

const std::vector<RigExecPurityFinding> &
RigExecFrozenPurityAudit()
{
    // One row per unit examined. "Examined" means read for static,
    // thread-local, and member state surviving a call, plus every lock and
    // every USD handle a worker could reach through it; the verdict is what
    // the frozen path may do with the unit.
    static const std::vector<RigExecPurityFinding> audit = {
        {"rigExecMath/* (avarScale, dualQuat, pointFrame, rbf, "
         "singleChainIk, solvers, splineIk, envelope, geometryKernels, "
         "propertyMath, weightFields)",
         RigExecFrozenPurity::Pure,
         "free functions over their arguments; no static, thread-local, or "
         "member state anywhere in the directory"},
        {"libs/rigExec/solverKernels.{h,cpp}",
         RigExecFrozenPurity::Pure,
         "ribbon/twist/rotation glue over caller buffers; no statics, no "
         "locks, no USD"},
        {"libs/rigExec/moverKernels.cpp exec callbacks and helpers",
         RigExecFrozenPurity::Pure,
         "only static const tokens and the SIMD switch, read once at "
         "library load (RigExecSimdEnabled); per-evaluation state lives in "
         "the VdfContext, never in the kernel"},
        {"libs/rigExec/computations.cpp frame helpers",
         RigExecFrozenPurity::Pure,
         "static free functions over frames and params; guards and packing "
         "only, no retained state"},
        {"libs/rigExec/weightPackets.cpp packet math",
         RigExecFrozenPurity::Pure,
         "one token table, touched at Build; assembly over caller "
         "buffers"},
        {"baked step bodies (bakedPose/bakedGeometry/bakedWeights/"
         "bakedVerify.cpp)",
         RigExecFrozenPurity::Pure,
         "a step reads declared slots and writes declared slots; per-step "
         "diagnostics and counters merge in step order, and BeginRun resets "
         "them, so no result survives into the next run; no body reads USD "
         "or the resolved-input overlay (enforced by RIGEXEC_PURITY_CHECK, "
         "bodyPurity.h), except the volatile oracle callers and the stage "
         "assembly of a plugin revision bound to region values; settings "
         "and token tables are read and touched at Build"},
        {"baked head tier (bakedProperties.cpp property revisions)",
         RigExecFrozenPurity::LiveOnly,
         "runs in the live prologue, serially on the owning thread, from "
         "head leaves sampled there; a body reads leaves, overrides and "
         "earlier versions and no USD (enforced by RIGEXEC_PURITY_CHECK), "
         "except a weight-object envelope's volatile oracle, which refuses "
         "the freeze"},
        {"baked head tier in a frozen job (frozenWorker.cpp _FrozenPrologue)",
         RigExecFrozenPurity::Pure,
         "runs from sampled leaves; no USD: the sampler reads every head "
         "leaf on the UI thread under its Build-time synthetic key, the "
         "worker writes them into its clone's leaves (whose attribute "
         "handles are dead and unread) and runs the same bodies serially, "
         "with no cone verifier and a disabled profiler; override slots "
         "are found by (prim, attribute), with no path built"},
        {"reader walks (bakedProperties.cpp RigExecBakedResolveReaderWalk)",
         RigExecFrozenPurity::Pure,
         "every read after the head tier that a chain result or a record "
         "can answer -- a chain-routed binding, a path leaf read through "
         "the resolved inputs -- resolves from head leaves, override slots, "
         "chain finals and records: no USD, no path built, no lock; live "
         "and a frozen job call the same function"},
        {"baked schedule serial executor",
         RigExecFrozenPurity::Pure,
         "program order on one thread; the reference every frozen run uses"},
        {"baked schedule parallel executor",
         RigExecFrozenPurity::LiveOnly,
         "WorkDispatcher + per-run atomics on the shared arena at normal "
         "priority; the frozen serial scope exists to keep workers out of "
         "it (proposed hook in RigExecBakedRunSteps)"},
        {"per-point geometry kernels (moverGraph.cpp parallel regions)",
         RigExecFrozenPurity::Pure,
         "range-independent per-point math; the per-context serial hook "
         "(D4) keeps frozen runs on the serial variants, which compute "
         "byte-identical numbers"},
        {"RigExecBakedProgramImpl structure (steps, edges, clusters, "
         "cones, walk, rest and ladder op declarations)",
         RigExecFrozenPurity::EpochPinned,
         "immutable after Build; safe as the programDigest check names it, "
         "never as a live read of per-frame working state"},
        {"ladder tables (restM through rotOrder)",
         RigExecFrozenPurity::Pure,
         "op-written: the RestCompose and LadderCompose head ops compose "
         "them in the live prologue from sampled ladder leaves, serially "
         "on the owning thread, under the body purity mark; a frozen job "
         "runs no rest or ladder op and declines any ladder that would "
         "recompose, so it reads the clone's tables as live left them"},
        {"skin/blend bindings (shared_ptr<const> topologies and layouts)",
         RigExecFrozenPurity::EpochPinned,
         "immutable snapshots resolved at Build/prologue; the worker runs "
         "from its own references, never from the live caches"},
        {"RigExecStaticInputCache VALUES",
         RigExecFrozenPurity::EpochPinned,
         "admission promises the same answer at every time code; safe as "
         "sampled values, while the cache OBJECT is the row below"},
        {"UsdStage, UsdAttribute, UsdAttributeQuery, UsdPrim, "
         "UsdGeomXformCache",
         RigExecFrozenPurity::LiveOnly,
         "every prologue read; sampled into the input vector on the UI "
         "thread, never named by the context, inputs, arena, or runner"},
        {"RigExecRigEvaluator (resolved inputs, live graphs, tap sets, "
         "overrides, profiler, epoch rests)",
         RigExecFrozenPurity::LiveOnly,
         "live state by definition; the program's captured pointers to it "
         "are why workers run a private arena, never the live program"},
        {"RigExecBakedProgramImpl live pointers (evaluator, stage, "
         "resolvedInputs, chainSnapshots, skinTopologies, "
         "blendSampleShapes, profiler, guideTaps)",
         RigExecFrozenPurity::LiveOnly,
         "read-what-the-evaluator-holds-now by design; a worker-owned "
         "program copy would still point at the live evaluator, so the "
         "region-only frozen entry is a program-side hook, not a copy"},
        {"ExecUsdSystem / TapSet / Snapshot (dynamic path)",
         RigExecFrozenPurity::LiveOnly,
         "OpenExec against the live stage; cannot run concurrently with "
         "stage edits, which is why refusal rigs take the D7 memo path"},
        {"wire-basis memo (moverGraph.cpp _CachedWireBasis, process-wide, "
         "mutex-guarded)",
         RigExecFrozenPurity::LiveOnly,
         "thread-safe and answer-preserving (full-input compare on hit), "
         "but a lock held across map insert; a baked or frozen Wire step "
         "whose envelope is sparse with a zero default still reaches it "
         "through RigExecRunRevisionKernel and takes the lock on a worker, "
         "until the program resolves the basis in its prologue"},
        {"RigExecStaticInputCache / RigExecSkinTopologyCache / "
         "RigExecBlendSampleCache OBJECTS",
         RigExecFrozenPurity::LiveOnly,
         "single-threaded or notice-invalidated live state (THREAD rule); "
         "workers use sampled values and held bindings, never the caches"},
        {"calibration/timing statics (bakedSchedule.cpp framesSeen)",
         RigExecFrozenPurity::LiveOnly,
         "unsynchronized diagnostic counters; frozen runs never enable "
         "calibration or step timing, so workers never touch them"},
        {"registry mutex / imaging bridge / snapshot store writes",
         RigExecFrozenPurity::LiveOnly,
         "Stream E's publish fence; background completions publish into "
         "the cache only, never into the snapshot, and never take the "
         "registry lock"},
    };
    return audit;
}

uint64_t
RigExecFrameCacheEpochDigest(const RigExecRigEvaluator &evaluator)
{
    uint64_t h = evaluator.GetBindingEpochDigest();
    const RigExecBakedProgram *program = evaluator.GetBakedProgram();
    if (!program) {
        return h;
    }
    const uint64_t builds = uint64_t(evaluator.GetBakedProgramBuildCount());
    h = _HashBytes(h, &builds, sizeof(builds));
    // Structure only (plan D3): the binding-table shapes, never the
    // constant values. A value patch moves the constant digest the
    // control half folds -- not this -- so it opens a new key namespace
    // instead of evicting the epoch.
    const RigExecBakedProgramImpl &B = program->GetStepGraph();
    const uint64_t constants = uint64_t(B.avarConstants.size());
    h = _HashBytes(h, &constants, sizeof(constants));
    const uint64_t bindings = uint64_t(B.avarConstantBindings.size());
    h = _HashBytes(h, &bindings, sizeof(bindings));
    for (const RigExecBakedProgramImpl::AvarBinding &binding :
         B.avarConstantBindings) {
        const uint64_t slot = uint64_t(binding.slot);
        h = _HashBytes(h, &slot, sizeof(slot));
    }
    const uint64_t varying = uint64_t(B.avarBindings.size());
    h = _HashBytes(h, &varying, sizeof(varying));
    const uint64_t table = uint64_t(B.avars.size());
    h = _HashBytes(h, &table, sizeof(table));
    return h;
}

uint64_t
RigExecEpochConstantDigest(const RigExecBakedProgram &program)
{
    return RigExecFrozenAvarRegionDigest(program);
}

std::vector<uint64_t>
RigExecConstantRegionsForPaths(const RigExecBakedProgram &program,
                               const std::vector<SdfPath> &paths)
{
    const RigExecBakedProgramImpl &B = program.GetStepGraph();
    std::vector<uint64_t> regions;
    for (const SdfPath &path : paths) {
        const auto found = B.patchableAvars.find(path);
        if (found == B.patchableAvars.end()) {
            continue;
        }
        const uint64_t region =
            RigExecConstantRegionForBinding(found->second);
        if (regions.empty() || regions.back() != region) {
            // patchableAvars iterates in path order, not binding order,
            // so contiguity is not guaranteed; the sort below fixes it.
            regions.push_back(region);
        }
    }
    std::sort(regions.begin(), regions.end());
    regions.erase(std::unique(regions.begin(), regions.end()),
                  regions.end());
    return regions;
}

bool
RigExecPatchFrozenAvarConstants(const RigExecFrozenProgram &base,
                                const RigExecBakedProgram &live,
                                std::shared_ptr<const RigExecFrozenProgram> *out,
                                std::string *error)
{
    const auto fail = [&error](const std::string &why) {
        if (error) {
            *error = why;
        }
        return false;
    };
    if (!out) {
        return fail("no snapshot to patch into");
    }
    const RigExecBakedProgramImpl &L = live.GetStepGraph();
    const RigExecBakedProgramImpl &S = base.program;
    // The same program object, or at least the same shape: a rebuild is
    // re-frozen, never patched.
    if (S.avarConstantBindings.size() != L.avarConstantBindings.size() ||
        S.avarConstants.size() != L.avarConstants.size()) {
        return fail("avar region changed shape: re-freeze, do not patch");
    }
    auto snapshot = std::make_shared<RigExecFrozenProgram>();
    _CloneImpl(S, &snapshot->program);
    RigExecBakedProgramImpl &P = snapshot->program;
    // Every field the live patch writes, carried onto the copy. The
    // consumed table for constant slots comes along too: no run recomputes
    // those slots (only a drag walks them), so the patch's write into
    // avars is the value, not history -- without it the copy would warm
    // the pre-edit constant. lastAvars is untouched on both sides, so the
    // next run's cones include the changed slots alike. Copied query and
    // resolved handles are dead by construction; the worker nulls them as
    // it patches.
    if (P.avars.size() != L.avars.size()) {
        return fail("avar table changed shape: re-freeze, do not patch");
    }
    for (size_t i = 0; i < L.avarConstantBindings.size(); ++i) {
        P.avarConstantBindings[i].input.constant =
            L.avarConstantBindings[i].input.constant;
        P.avarConstantBindings[i].input.varying =
            L.avarConstantBindings[i].input.varying;
        P.avarConstantBindings[i].input.query =
            L.avarConstantBindings[i].input.query;
        P.avarConstantBindings[i].input.resolvedAttr =
            L.avarConstantBindings[i].input.resolvedAttr;
        const size_t slot = L.avarConstantBindings[i].slot;
        if (slot >= P.avars.size()) {
            return fail("avar slot out of range: re-freeze, do not patch");
        }
        P.avars[slot] = L.avars[slot];
    }
    P.promotedAvars = L.promotedAvars;
    P.varyingInputs = L.varyingInputs;
    P.avarConstants = L.avarConstants;
    // And every value edit routed since the snapshot was taken or last
    // patched, added to the copy's own pending ones: the copy's first run
    // owes them all. Asked of the per-index edit counts, not of the live
    // program's pending flags, because a live run consumes those -- an edit
    // the live program has already answered is one the snapshot's history
    // has still never seen.
    if (L.valueEditSerial != S.valueEditSerial) {
        P.edited.resize(std::max(P.edited.size(), L.editSerial.size()), 0);
        for (size_t i = 0; i < L.editSerial.size(); ++i) {
            if (L.editSerial[i] > S.valueEditSerial) {
                P.edited[i] = 1;
                P.anyEdited = true;
            }
        }
        P.valueEditSerial = L.valueEditSerial;
        P.editSerial = L.editSerial;
    }
    // And a stamp bumped since: a notice the index could not place, which
    // the live program answered by running whole once. The copy owes the
    // same run -- its stamp moves past the one its last run recorded.
    if (L.programStamp != S.programStamp) {
        P.programStamp = L.programStamp;
    }
    // The epoch's side-tables, unchanged by a constant patch.
    snapshot->jointSolverBinding = base.jointSolverBinding;
    snapshot->guideTapsPresent = base.guideTapsPresent;
    snapshot->inputHeadPaths = base.inputHeadPaths;
    snapshot->arrayKeys = base.arrayKeys;
    snapshot->chainBaseQueryValid = base.chainBaseQueryValid;
    snapshot->derivedBaseQueryValid = base.derivedBaseQueryValid;
    snapshot->ribbonQueryValid = base.ribbonQueryValid;
    snapshot->moverHasEnabled = base.moverHasEnabled;
    snapshot->moverHasDefaultWeight = base.moverHasDefaultWeight;
    snapshot->moverHasMethod = base.moverHasMethod;
    *out = std::move(snapshot);
    return true;
}

} // namespace rigExec
