//
// The baked program's geometry half: the chain/revision bake and the frame
// path that runs those revisions and the derived maintenance behind them.
//
// Split out of bakedProgram.cpp for the reason bakedPose.cpp was: a domain's
// bake and its frame path have to agree about what was captured and what is
// re-read. The evaluator is not visible here -- the compiled chains arrive
// restated as RigExecBakedChainSpec, and the caches the frame path shares
// with the dynamic walk were captured into RigExecBakedProgramImpl at Build.
//
#include "bakedProgramImpl.h"

#include "curvenetAdjuster.h"
#include "frameExtraction.h"
#include "moverGraph.h"
#include "parallel.h"
#include "rigEvaluator.h"
#include "types.h"

#include "rigExecMath/dualQuat.h"
#include "rigExecMath/envelope.h"
#include "rigExecMath/geometryKernels.h"
#include "rigExecMath/simdKernels.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/staticTokens.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/vt/value.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/attributeQuery.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/xformCache.h"
#include "pxr/usd/usdGeom/xformable.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iterator>
#include <memory>
#include <string>
#include <utility>
#include <vector>

// The two names a step body needs a TOKEN for rather than a comparison: the
// status a failed revision publishes, and the attribute the defaultWeight
// diagnostic re-reads. Hoisted because TfToken(const char *) takes the token
// registry's spin lock on every construction, and a step body may take no
// lock (docs/specs/baked-step-graph.md §2).
TF_DEFINE_PRIVATE_TOKENS(
    _tokens,
    ((moverFailed, "moverFailed"))
    ((defaultWeight, "inputs:defaultWeight"))
    ((classicLinear, "classicLinear"))
    ((dualQuaternion, "dualQuaternion"))
    ((jointIndices, "rigExec:jointIndices"))
    ((elementSize, "rigExec:elementSize"))
    ((denseRepresentation, "dense"))
);

namespace rigExec {

// ---------------------------------------------------------------------------
// Bake.
// ---------------------------------------------------------------------------

void
RigExecBakedBuildGeometry(RigExecBakedBuildContext *ctx,
                          const std::vector<RigExecBakedChainSpec> &chains)
{
    RigExecBakedProgramImpl &B = *ctx->program;
    auto refuse = [&](const std::string &what, const SdfPath &where) {
        ctx->Refuse(what, where);
    };
    auto slotOf = [&](const SdfPath &path) { return ctx->SlotOf(path); };

    auto bakeRevision = [&](const RigExecBakedRevisionSpec &r) {
        RigExecBakedProgramImpl::GeomRevision out;
        out.moverPath = r.moverPath;
        out.target = r.target;
        out.moverPrim = B.stage->GetPrimAtPath(r.moverPath);
        out.op = r.op;
        out.binding = r.binding;
        out.finalPhase = r.transformFinalPhase;
        out.skinTopologyFixed = r.skinTopologyFixed;
        out.snapshotAfter = r.snapshotAfter;
        // The geometry-domain constraint whose measured delta IS this
        // revision's transform. The join key is the MOVER path, which is
        // what the dynamic walk's own constraintDeltas map is keyed by; the
        // pose half is baked first, so the constraint is already here.
        for (const RigExecBakedProgramImpl::Constraint &constraint :
                 B.constraints) {
            if (constraint.deltaBase >= 0 && constraint.path == r.moverPath) {
                out.constraintDelta = constraint.deltaBase;
                break;
            }
        }
        // A declared phase is the only thing that reads the snapshot store,
        // and the pose half fills its half of that store only when one
        // exists. All THREE consumers the dynamic walk has are counted: the
        // per-input phases, an AtPrim transform (answered out of the same
        // store), and a blend sample whose points carry a phase -- that last
        // one reads chain-point records, which the geometry half writes
        // ungated, but leaving it out would make this flag mean "some phases"
        // rather than "some phased read", which is what the next consumer
        // will read it as.
        bool phased = !r.binding.phases.empty() ||
                      r.binding.transformPhase.kind ==
                          RigExecReadPhaseKind::AtPrim;
        for (const auto &[input, samples] : r.binding.blendSamples) {
            for (const RigExecBlendSampleBinding &sample : samples) {
                phased = phased || !sample.phase.IsBase();
                out.readsSnapshots =
                    out.readsSnapshots || !sample.phase.IsBase();
            }
        }
        out.readsSnapshots =
            out.readsSnapshots || !r.binding.phases.empty() ||
            r.binding.transformPhase.kind == RigExecReadPhaseKind::AtPrim;
        if (phased) {
            B.phasedReads = true;
        }
        if (!out.moverPrim) {
            refuse("mover prim is missing", r.moverPath);
        }
        // The packet is assembled live from this prim every frame through
        // the generation's resolved inputs: nothing of it is captured, so a
        // value edit needs no rebuild and an override places itself. The
        // layout arrays are still NAMED, because bakeability judged them --
        // connecting one is a resync, and the judgement was that none is
        // connected. They do not belong in `rebuild` even though the layout
        // is no longer read per frame: it is held in the evaluator's skin
        // topology cache, which every notice clears, so a weight-paint edit
        // reaches the next generation through the re-read rather than
        // through a rebuild of this program.
        B.prims.insert(r.moverPath);
        B.resolvedRoutedPrims.insert(r.moverPath);
        for (const char *name : {"rigExec:jointIndices", "rigExec:jointWeights",
                                 "rigExec:elementSize",
                                 "rigExec:skinningMethod"}) {
            B.named.insert(r.moverPath.AppendProperty(TfToken(name)));
        }
        // Every path an operation's per-frame assembler reads, whatever the
        // operation is: the cage a lattice deforms through, the surface a
        // projection lands on, the topology a smooth walks, the bind
        // coordinates a ribbon rides. None of them is folded into the
        // program -- they are re-read on every frame through the
        // generation's resolved inputs -- so they belong in `named`, where a
        // RESYNC (the property appearing, disappearing or being retargeted,
        // which also invalidates any retained query) finds them, and in no
        // case in `rebuild`, which is for values the program captured.
        //
        // Named unconditionally rather than per operation: the binding
        // carries exactly the paths the operation resolved, so an empty one
        // is an operation that does not read it and an authored one is a
        // path some assembler will ask for.
        const auto name = [&](const SdfPath &path) {
            if (path.IsEmpty()) {
                return;
            }
            B.named.insert(path);
            B.prims.insert(path.GetPrimPath());
        };
        name(r.binding.base);
        name(r.binding.topologyCounts);
        name(r.binding.topologyIndices);
        name(r.binding.cagePoints);
        name(r.binding.surfacePoints);
        name(r.binding.bindCoords);
        name(r.binding.driverCurvePoints);
        name(r.binding.driverCurveOrder);
        name(r.binding.driverCurveKnots);
        name(r.binding.widths);
        name(r.binding.curvenetPoints);
        if (r.op == RigExecRevisionOp::CurvenetAdjuster) {
            // The adjuster reads the NET it writes -- its authored points at
            // Default, its spline indices and its basis -- and then every
            // adjustment prim's rest/default/parent/posed/avars ladder,
            // which is what an animator drags. All of it per frame through
            // the generation's resolved inputs, so the adjustment prims join
            // the mover in resolvedRoutedPrims and nothing is captured.
            const SdfPath netPrim = r.target.GetPrimPath();
            B.prims.insert(netPrim);
            B.named.insert(netPrim.AppendProperty(TfToken("points")));
            for (const char *attribute : {"rigExec:splineIndices",
                                          "rigExec:basis"}) {
                B.named.insert(netPrim.AppendProperty(TfToken(attribute)));
            }
            B.named.insert(
                r.moverPath.AppendProperty(TfToken("rigExec:adjustments")));
            for (const SdfPath &adjustment :
                     RigExecCurvenetAdjustmentPaths(out.moverPrim)) {
                B.prims.insert(adjustment);
                B.resolvedRoutedPrims.insert(adjustment);
                B.named.insert(
                    adjustment.AppendProperty(TfToken("rigExec:curvenet")));
                for (const char *attribute : {"rigExec:knotIndex",
                                              "rigExec:pointKind",
                                              "rigExec:includeTangents"}) {
                    B.named.insert(
                        adjustment.AppendProperty(TfToken(attribute)));
                }
            }
        }
        if (!r.binding.curvenet.IsEmpty()) {
            // The net prim itself: its rigExec:splineIndices,
            // rigExec:samplesPerSpline and rigExec:basis are the bind's
            // shape, read at Default on every frame and digested rather than
            // captured -- so a resync that retargets one has to be seen, and
            // an edit to one re-cuts through the digest with no rebuild.
            B.prims.insert(r.binding.curvenet);
            for (const char *attribute : {"rigExec:splineIndices",
                                          "rigExec:samplesPerSpline",
                                          "rigExec:basis"}) {
                B.named.insert(
                    r.binding.curvenet.AppendProperty(TfToken(attribute)));
            }
        }
        // A phased input is read out of the run's snapshot store when the
        // phase resolves and off the stage when it does not, so the path is
        // one the bake asked about either way.
        for (const auto &[input, phase] : r.binding.phases) {
            name(input);
        }
        // The blend channels, in the order the accumulation is defined over.
        // Only the HANDLES are captured: every channel weight, every
        // activation and every target shape is re-read per frame through the
        // generation's resolved inputs, which is what lets an animator drag a
        // channel weight and see it -- so the input and sample prims go into
        // `resolvedRoutedPrims` beside the mover's, where an override on one
        // of their properties places with nothing else to do.
        for (const SdfPath &input : r.binding.blendInputs) {
            RigExecBakedProgramImpl::GeomBlendChannel channel;
            B.prims.insert(input);
            B.resolvedRoutedPrims.insert(input);
            const SdfPath weightPath =
                input.AppendProperty(TfToken("inputs:weight"));
            B.named.insert(weightPath);
            channel.weight = B.stage->GetAttributeAtPath(weightPath);
            channel.weightPath = weightPath;
            // A weight connected to a pose interpolator's output reads the
            // interpolator's slot. The walk is the one RigExecResolvedInputs::
            // GetAttribute makes -- single authored connections, followed
            // until one lands on a property the program publishes -- so the
            // slot answers exactly where the dynamic walk's in-memory lookup
            // would have.
            {
                std::set<SdfPath> visiting;
                UsdAttribute a = channel.weight;
                while (a && visiting.insert(a.GetPath()).second) {
                    const auto found = B.poseWeightIndex.find(a.GetPath());
                    if (found != B.poseWeightIndex.end()) {
                        channel.poseWeight = found->second;
                        break;
                    }
                    SdfPathVector connections;
                    if (a.HasAuthoredConnections()) {
                        a.GetConnections(&connections);
                    }
                    if (connections.size() != 1) {
                        break;
                    }
                    a = B.stage->GetAttributeAtPath(connections[0]);
                }
            }
            const auto samples = r.binding.blendSamples.find(input);
            if (samples != r.binding.blendSamples.end()) {
                for (const RigExecBlendSampleBinding &binding :
                         samples->second) {
                    RigExecBakedProgramImpl::GeomBlendChannel::Sample sample;
                    B.prims.insert(binding.sample);
                    B.resolvedRoutedPrims.insert(binding.sample);
                    const SdfPath activationPath =
                        binding.sample.AppendProperty(
                            TfToken("rigExec:activation"));
                    B.named.insert(activationPath);
                    sample.activation =
                        B.stage->GetAttributeAtPath(activationPath);
                    sample.samplePath = binding.sample;
                    sample.pointsPath = binding.points;
                    sample.phase = binding.phase;
                    sample.blendShape = binding.blendShape;
                    if (binding.blendShape.IsEmpty()) {
                        name(binding.points);
                        sample.points =
                            B.stage->GetAttributeAtPath(binding.points);
                    } else {
                        // A sparse sample's shape is resolved in the
                        // prologue, through the evaluator's cache; what is
                        // named here is the shape prim, so a resync under it
                        // rebuilds. A value edit to its offsets is caught by
                        // the cache, which every notice clears.
                        B.prims.insert(binding.blendShape.GetPrimPath());
                    }
                    channel.samples.push_back(std::move(sample));
                }
            }
            out.blendChannels.push_back(std::move(channel));
        }
        if (!r.binding.transform.IsEmpty()) {
            out.transformSlot = slotOf(r.binding.transform);
            if (out.transformSlot < 0) {
                refuse("mover transform provider is not a pose provider",
                       r.binding.transform);
            }
        }
        if (!r.binding.transformSpace.IsEmpty()) {
            out.transformSpaceSlot = slotOf(r.binding.transformSpace);
            if (out.transformSpaceSlot < 0) {
                refuse("mover transform space is not a pose provider",
                       r.binding.transformSpace);
            }
        }
        // The solver whose aggregate supplies values.driverFrames, resolved
        // once here: the walk's solver table is built before the geometry
        // half, so the per-revision tap the dynamic path reads becomes an
        // index into it.
        if (!r.binding.driverFrames.IsEmpty()) {
            B.prims.insert(r.binding.driverFrames);
            const auto solver = B.solverIndex.find(r.binding.driverFrames);
            if (solver == B.solverIndex.end()) {
                refuse("driver frames solver was not baked",
                       r.binding.driverFrames);
            } else {
                out.driverFramesSolver = solver->second;
            }
        }
        for (const SdfPath &influence : r.binding.influences) {
            const int slot = slotOf(influence);
            if (slot < 0) refuse("skin influence is not a provider", influence);
            out.influenceSlots.push_back(slot);
        }
        // The weight object, baked once however many revisions bind it: what
        // this revision holds is its index in the shared table, and the
        // packet itself is built once per frame by that object's own step.
        out.weightObject =
            RigExecBakedBakeWeightObject(ctx, r.binding.weightObject);
        if (out.weightObject >= 0) {
            // WHAT the published field says it weights, decided here because
            // it is a relationship read and a relationship is epoch state.
            // The condition is the dynamic path's, verbatim: exactly one
            // declared target, and it is this mover, means the field weights
            // the OPERATION and carries one element; anything else weights
            // the chain's points.
            const UsdPrim weightPrim =
                B.stage->GetPrimAtPath(r.binding.weightObject);
            const SdfPathVector declared =
                ctx->Targets(weightPrim, "rigExec:weightTarget");
            out.weightOperationDomain =
                declared.size() == 1 && declared[0] == r.moverPath;
            out.weightFieldTarget =
                out.weightOperationDomain ? r.moverPath : r.target;
            out.weightCurrentPhase =
                B.currentPhaseWeights.count(r.binding.weightObject) > 0;
        }
        return out;
    };
    for (const RigExecBakedChainSpec &spec : chains) {
        const SdfPath &target = spec.target;
        RigExecBakedProgramImpl::GeomChain chain;
        chain.target = target;
        // The base points are read per frame through a query, so only the
        // query's own validity depends on this surviving a resync.
        B.prims.insert(target.GetPrimPath());
        B.named.insert(target);
        if (const UsdAttribute a = B.stage->GetAttributeAtPath(target)) {
            chain.baseQuery = UsdAttributeQuery(a);
        } else {
            refuse("chain target has no attribute", target);
        }
        for (const RigExecBakedRevisionSpec &revision : spec.revisions) {
            chain.revisions.push_back(bakeRevision(revision));
        }
        for (const RigExecBakedRevisionSpec &derived : spec.derived) {
            RigExecBakedProgramImpl::GeomChain::Derived d;
            d.target = derived.target;
            B.prims.insert(derived.target.GetPrimPath());
            B.named.insert(derived.target);
            if (const UsdAttribute a =
                    B.stage->GetAttributeAtPath(derived.target)) {
                d.baseQuery = UsdAttributeQuery(a);
            } else {
                refuse("derived target has no attribute", derived.target);
            }
            d.revision = bakeRevision(derived);
            chain.derived.push_back(std::move(d));
        }
        B.chains.push_back(std::move(chain));
    }
    // The posed net a curvenet mover reads is another CHAIN's published
    // points, so the link is an index into the table just built. Resolved in
    // a second pass because the chains are appended in E._chainOrder and the
    // net's own chain, while always EARLIER than its reader, is not in the
    // table yet while the reader is being baked.
    std::map<SdfPath, int> chainIndex;
    for (size_t c = 0; c < B.chains.size(); ++c) {
        chainIndex[B.chains[c].target] = int(c);
    }
    for (RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        for (RigExecBakedProgramImpl::GeomRevision &revision :
                 chain.revisions) {
            if (revision.binding.curvenetPoints.IsEmpty()) {
                continue;
            }
            const auto found = chainIndex.find(revision.binding.curvenetPoints);
            if (found != chainIndex.end()) {
                revision.curvenetChain = found->second;
            }
            // Not found is not a refusal: a net that no mover chain poses
            // has no chain of its own, and the assembler falls back to its
            // authored points at the evaluated time -- which is exactly what
            // the dynamic walk does when movedProperties has no entry.
        }
    }
}


// ---------------------------------------------------------------------------
// The vertex partition.
//
// A skin revision's per-vertex work is separable -- point i reads the indices
// and weights at i * elementSize, the influence matrices they name and its
// own incoming position, and nothing else -- so a contiguous range of
// vertices is an independent sub-problem whose result is bit-identical to the
// same vertices computed as part of the whole array. What is NOT separable is
// everything around that body, which is why the range is cut here and the
// decisions stay whole (§6 of docs/specs/baked-step-graph.md).
//
// The cut is by vertex COUNT and the key follows from it, rather than the
// other way round: the vertex order is the mesh's and is never permuted, so
// two body regions that happen to share a range simply wait for both their
// joints -- and the other ranges still start when their own joints land,
// which is what the whole exercise buys. The schedule report prints |key| per
// chunk so that trade-off is measured per asset rather than assumed.
// ---------------------------------------------------------------------------

namespace {

/// The level at which every provider's matrix is final, indexed by slot:
/// [1] the rest -> final matrices, [0] the rest -> base ones. A slot no
/// ProviderMatrix step writes stays at -1, which reads as "not scheduled"
/// rather than "ready at level 0".
struct ProviderLevels {
    std::vector<int> byPhase[2];
    int maxLevel = 0;

    int Of(int slot, bool finalPhase) const
    {
        const std::vector<int> &levels = byPhase[finalPhase ? 1 : 0];
        return slot >= 0 && size_t(slot) < levels.size()
                   ? levels[size_t(slot)]
                   : -1;
    }
};

ProviderLevels
GatherProviderLevels(const RigExecBakedProgramImpl &B)
{
    ProviderLevels levels;
    levels.byPhase[0].assign(B.paths.size(), -1);
    levels.byPhase[1].assign(B.paths.size(), -1);
    for (const RigExecBakedStep &step : B.steps) {
        levels.maxLevel = std::max(levels.maxLevel, step.level);
        if (step.kind != RigExecBakedStepKind::ProviderMatrix) {
            continue;
        }
        // part 1 is the final-phase matrix, part 0 the base one; object is
        // the provider slot.
        if (step.object >= 0 && size_t(step.object) < B.paths.size() &&
            (step.part == 0 || step.part == 1)) {
            levels.byPhase[size_t(step.part)][size_t(step.object)] =
                step.level;
        }
    }
    return levels;
}

/// The level at which chunk \p k of \p revision has every joint it reads.
///
/// The one number the cut is decided on, and the one the report prints, so
/// that the decision and the record of it cannot drift apart. An
/// unpartitioned revision is one chunk over every influence, which is the
/// degenerate case of the same maximum.
int
ChunkReadyLevel(const RigExecBakedProgramImpl::GeomRevision &revision,
                const RigExecBakedProgramImpl::GeomChunk &chunk,
                bool chunked, const ProviderLevels &levels)
{
    const size_t influences = revision.influenceSlots.size();
    const size_t keySize = chunked ? chunk.key.size() : influences;
    int ready = 0;
    for (size_t e = 0; e < keySize; ++e) {
        const size_t position = chunked ? size_t(chunk.key[e]) : e;
        if (position >= influences) {
            continue;
        }
        ready = std::max(
            ready,
            levels.Of(revision.influenceSlots[position],
                      revision.finalPhase));
    }
    return ready;
}

/// How many vertices one chunk covers before the cap takes over.
///
/// The default is the point count at which the geometry kernels start
/// splitting work themselves, because a range smaller than that is a range
/// the deformation was never worth spreading over.
size_t
ChunkVertexTarget()
{
    static const size_t target = [] {
        const int authored = TfGetenvInt(
            "RIGEXEC_BAKED_CHUNK_VERTS", int(RigExecGeometryParallelThreshold));
        return authored < 1 ? size_t(1) : size_t(authored);
    }();
    return target;
}

/// The most chunks one revision is cut into; the range grows to meet it.
///
/// A cap rather than a target: chunk count decides STEP count, and a mesh
/// with a million vertices must not add two hundred steps to the program for
/// a machine that can run twenty of them at once.
size_t
ChunkCap()
{
    static const size_t cap = [] {
        const int authored = TfGetenvInt("RIGEXEC_BAKED_MAX_CHUNKS", 32);
        return authored < 1 ? size_t(1) : size_t(authored);
    }();
    return cap;
}

/// Whether a skin revision is cut whatever its chunks' ready levels say.
///
/// RIGEXEC_BAKED_CHUNK_ALWAYS=1. The escape hatch for the rules' own
/// fixtures: the chunked path has to be exercised on a real rig, and the
/// rigs that bake today all pose every joint of a mesh at the same level, so
/// with the rule ON nothing would cut and every assertion about a chunk would
/// pass vacuously. Read on each call rather than cached in a static, so one
/// process can build a program both ways -- which is exactly what the
/// decision test does.
bool
ChunkAlways()
{
    return TfGetenvInt("RIGEXEC_BAKED_CHUNK_ALWAYS", 0) != 0;
}

/// Whether every element of ascending \p a appears in ascending \p b.
bool
IsSubset(const std::vector<int> &a, const std::vector<int> &b)
{
    size_t j = 0;
    for (const int value : a) {
        while (j < b.size() && b[j] < value) {
            ++j;
        }
        if (j >= b.size() || b[j] != value) {
            return false;
        }
    }
    return true;
}

}  // namespace

void
RigExecBakedPartitionRevision(
    RigExecBakedProgramImpl::GeomRevision *revision,
    const int *indices, size_t indexCount, int elementSize, int chunkCount)
{
    using GeomChunk = RigExecBakedProgramImpl::GeomChunk;
    const size_t slots = elementSize < 1 ? 0 : size_t(elementSize);
    const size_t points = slots ? indexCount / slots : 0;
    const size_t influences = revision->influenceSlots.size();
    revision->partitionElementSize = elementSize;
    revision->partitionIndexCount = indexCount;
    revision->partitionPointCount = points;

    // How many ranges, and how long. The same answer every time for the same
    // layout, which is what makes a re-cut against arrays that did not
    // actually move reproduce the cut it replaces.
    size_t target = ChunkVertexTarget();
    size_t count = std::max<size_t>(1, (points + target - 1) / target);
    if (count > ChunkCap()) {
        count = ChunkCap();
        target = std::max<size_t>(1, (points + count - 1) / count);
        count = std::max<size_t>(1, (points + target - 1) / target);
    }

    std::vector<GeomChunk> chunks;
    chunks.reserve(count);
    // One pass over the indices, and one byte per influence to remember what
    // this range has already claimed -- the key is a SET of the range's
    // influence positions, and the vertices arrive in no particular order.
    std::vector<char> claimed(influences, 0);
    for (size_t c = 0; c < count; ++c) {
        GeomChunk chunk;
        chunk.begin = int(std::min(points, c * target));
        chunk.end = int(c + 1 == count ? points
                                       : std::min(points, (c + 1) * target));
        for (size_t point = size_t(chunk.begin); point < size_t(chunk.end);
             ++point) {
            for (size_t slot = 0; slot < slots; ++slot) {
                const int index = indices[point * slots + slot];
                // An index outside the table is a layout the packet will
                // reject; leaving it out of the key costs nothing, because
                // the revision never applies.
                if (index < 0 || size_t(index) >= influences ||
                    claimed[size_t(index)]) {
                    continue;
                }
                claimed[size_t(index)] = 1;
                chunk.key.push_back(index);
            }
        }
        std::sort(chunk.key.begin(), chunk.key.end());
        for (const int index : chunk.key) {
            claimed[size_t(index)] = 0;
        }
        chunks.push_back(std::move(chunk));
    }

    // Adjacent ranges merge while they stay under the vertex target and one
    // key contains the other: a range waiting for a superset of its
    // neighbour's joints is not waiting any longer for holding both, and one
    // range is one step's worth of scheduling instead of two.
    for (size_t i = 0; i + 1 < chunks.size();) {
        const size_t merged = size_t(chunks[i + 1].end - chunks[i].begin);
        const bool contains = IsSubset(chunks[i].key, chunks[i + 1].key);
        const bool contained = IsSubset(chunks[i + 1].key, chunks[i].key);
        if (merged <= target && (contains || contained)) {
            chunks[i].end = chunks[i + 1].end;
            if (contains) {
                chunks[i].key = chunks[i + 1].key;  // the union
            }
            chunks.erase(chunks.begin() + long(i) + 1);
        } else {
            ++i;
        }
    }

    // A re-cut is told how many ranges it must end up with, because the
    // number of STEPS a revision is made of was fixed at Build and a layout
    // that moved may not change the program. A layout that did NOT move
    // reaches this with the count it had and nothing happens, which is what
    // makes the first frame of an epoch re-derive the cut Build made rather
    // than replace it.
    if (chunkCount > 0) {
        while (chunks.size() > size_t(chunkCount)) {
            // The adjacent pair that costs the least to join.
            size_t best = 0;
            for (size_t i = 1; i + 1 < chunks.size(); ++i) {
                if (chunks[i + 1].end - chunks[i].begin <
                    chunks[best + 1].end - chunks[best].begin) {
                    best = i;
                }
            }
            std::vector<int> key;
            std::set_union(chunks[best].key.begin(), chunks[best].key.end(),
                           chunks[best + 1].key.begin(),
                           chunks[best + 1].key.end(),
                           std::back_inserter(key));
            chunks[best].key = std::move(key);
            chunks[best].end = chunks[best + 1].end;
            chunks.erase(chunks.begin() + long(best) + 1);
        }
        // An empty range is a step that does nothing, which is the honest
        // shape of "this layout wants fewer chunks than the program has".
        while (chunks.size() < size_t(chunkCount)) {
            GeomChunk chunk;
            chunk.begin = int(points);
            chunk.end = int(points);
            chunks.push_back(std::move(chunk));
        }
    }

    // The table every chunk skins against: identity everywhere, with its own
    // influences copied in per run. Filled here so that a run writes only the
    // |key| entries that moved, and so that the rows a SIMD kernel loads are
    // never out of step with the matrices beside them.
    const bool keyed = chunks.size() > 1;
    for (GeomChunk &chunk : chunks) {
        chunk.ok = false;
        chunk.keyChanged = false;
        chunk.palette.clear();
        if (!keyed) {
            // One chunk is the whole array, and the whole array is what the
            // revision's own folded table already holds.
            chunk.key.clear();
            chunk.transforms.clear();
            chunk.rows.clear();
            continue;
        }
        chunk.transforms.assign(influences, GfMatrix4d(1.0));
        chunk.rows.assign(influences * RigExecSkinRowStride, 0.0f);
        for (size_t t = 0; t < influences; ++t) {
            RigExecNarrowSkinRows(chunk.transforms[t],
                                  &chunk.rows[t * RigExecSkinRowStride]);
        }
    }
    revision->chunks = std::move(chunks);
    revision->chunked = keyed;
}

// ---------------------------------------------------------------------------
// Build: the geometry half of the program, in program order.
// ---------------------------------------------------------------------------

namespace {

RigExecBakedStep &
AddGeometryStep(RigExecBakedProgramImpl *program, RigExecBakedStepKind kind,
                int object, int part = -1)
{
    RigExecBakedStep step;
    step.kind = kind;
    step.object = object;
    step.part = part;
    step.maxDiagnostics = 0;
    program->steps.push_back(std::move(step));
    return program->steps.back();
}

/// Declares the matrices \p revision folds, which are the slots the pose half
/// published for it: final or base per the revision's phase, exactly the set
/// the ProviderMatrix steps were created for.
void
DeclareMatrixReads(const RigExecBakedProgramImpl::GeomRevision &revision,
                   RigExecBakedStep *step)
{
    const RigExecBakedSlotDomain domain =
        revision.finalPhase ? RigExecBakedSlotDomain::FinalMatrix
                            : RigExecBakedSlotDomain::BaseMatrix;
    if (revision.transformSlot >= 0) {
        step->reads.push_back(
            RigExecBakedOne(domain, revision.transformSlot));
    }
    if (revision.transformSpaceSlot >= 0) {
        step->reads.push_back(
            RigExecBakedOne(domain, revision.transformSpaceSlot));
    }
    for (const int slot : revision.influenceSlots) {
        if (slot >= 0) {
            step->reads.push_back(RigExecBakedOne(domain, slot));
        }
    }
}

}  // namespace

namespace {

/// Cuts \p revision at Build, from the layout the stage authors.
///
/// Only a SKIN revision whose layout the epoch fixed is cut into more than
/// one chunk. Two reasons, and they are the same reason twice: a layout that
/// can move within the epoch is one whose keys a Build-time cut cannot
/// promise anything about, and a mover with such a layout already re-reads
/// and re-validates every element of it once per frame, which is far more
/// than a partition would save. Everything else -- every other operation, and
/// a skin whose arrays are animated, connected or written by a property chain
/// -- is one chunk over the whole array, which is the degenerate case of the
/// same step.
void
PartitionAtBuild(const ProviderLevels &levels,
                 RigExecBakedProgramImpl::GeomRevision *revision)
{
    revision->chunks.assign(1, RigExecBakedProgramImpl::GeomChunk());
    revision->chunked = false;
    revision->partitionCandidates = 0;
    revision->partitionReadyMin = 0;
    revision->partitionReadyMax = 0;
    if (revision->op != RigExecRevisionOp::Skin ||
        !revision->skinTopologyFixed || !revision->moverPrim) {
        return;
    }
    // Read directly rather than through the evaluator's skin topology cache:
    // resolving there would seed the cache from Build's time code with a
    // layout the dynamic path has not asked for yet. Fixed means the arrays
    // do not vary in time, so the default-time read IS the epoch's -- and the
    // prologue re-cuts against the handle the packet will actually carry the
    // first time it sees one, so a disagreement costs one extra pass and
    // never a wrong key.
    VtIntArray indices;
    int elementSize = 1;
    if (const UsdAttribute a =
            revision->moverPrim.GetAttribute(_tokens->jointIndices)) {
        a.Get(&indices, UsdTimeCode::Default());
    }
    if (const UsdAttribute a =
            revision->moverPrim.GetAttribute(_tokens->elementSize)) {
        a.Get(&elementSize, UsdTimeCode::Default());
    }
    if (elementSize < 1 || indices.empty() ||
        indices.size() % size_t(elementSize) != 0) {
        return;
    }
    RigExecBakedPartitionRevision(revision, indices.cdata(), indices.size(),
                                  elementSize, /*chunkCount=*/0);
    revision->partitionCandidates = revision->chunks.size();

    // Whether the cut pays, which is a question about LEVELS and not about
    // vertex counts.
    //
    // A chunk body is a serial loop over its range; an uncut revision is one
    // call to RigExecApplySkinKernel over the whole array, and that kernel
    // spreads itself over the arena. So cutting a revision into seven ranges
    // that all become runnable at the same level does not overlap anything
    // -- it replaces one data-parallel call with seven serial ones, measured
    // at 47us serial and 92us parallel of the biped's frame. What the cut is
    // FOR is the range whose own joints land early: it may start while the
    // rest of the rig is still being posed, and that is only possible when
    // the candidate ranges become ready at different levels.
    //
    // Equivalently, and this is how the rule reads in §6: the whole revision
    // is ready when its LAST joint is, which is the maximum below, so a
    // range readier than that maximum is exactly a range that can start
    // earlier than the revision could.
    int readyMin = -1, readyMax = -1;
    for (const RigExecBakedProgramImpl::GeomChunk &chunk : revision->chunks) {
        const int ready = ChunkReadyLevel(*revision, chunk,
                                          /*chunked=*/true, levels);
        readyMin = readyMin < 0 ? ready : std::min(readyMin, ready);
        readyMax = std::max(readyMax, ready);
    }
    revision->partitionReadyMin = readyMin < 0 ? 0 : readyMin;
    revision->partitionReadyMax = readyMax < 0 ? 0 : readyMax;
    if (revision->chunked && revision->partitionReadyMin >=
                                 revision->partitionReadyMax &&
        !ChunkAlways()) {
        // Back to the degenerate cut, which is the shape every other
        // operation already has: one range over the whole array, no keys and
        // no per-chunk tables, so the fuse's whole-array path runs the
        // revision through the self-parallelising kernel. The partition
        // FIELDS stay where the cut left them -- the point and index counts
        // are the layout's, not the cut's, and the report reads them.
        revision->chunks.assign(1, RigExecBakedProgramImpl::GeomChunk());
        revision->chunked = false;
    }
}

}  // namespace

void
RigExecBakedBuildGeometrySteps(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    // The pose half's levels, swept in Build before this function is called.
    // Every ProviderMatrix step exists and carries its final level, because a
    // geometry step never precedes a pose step -- so the partition can ask
    // when a chunk's joints land without the geometry steps being in the
    // graph yet.
    const ProviderLevels levels = GatherProviderLevels(B);
    B.chainRevisionBegin.assign(B.chains.size(), 0);
    B.chainRevisionEnd.assign(B.chains.size(), 0);
    B.chainChunkBegin.assign(B.chains.size(), 0);
    B.chainChunkEnd.assign(B.chains.size(), 0);
    int nextChunk = 0;
    for (size_t c = 0; c < B.chains.size(); ++c) {
        RigExecBakedProgramImpl::GeomChain &chain = B.chains[c];
        // Revision ids are handed out chain by chain, so one chain's
        // revisions are a contiguous RANGE: "the buffers this chain has
        // produced so far" -- which is what a revision reads its preceding
        // points from and what the status sweep reads at the end -- is then
        // one declared range rather than a list. The chunk ids inside them
        // are contiguous for the same reason and at the same grain.
        B.chainRevisionBegin[c] = int(B.revisionIndex.size());
        B.chainChunkBegin[c] = nextChunk;
        for (size_t r = 0; r < chain.revisions.size(); ++r) {
            B.revisionIndex.emplace_back(int(c), int(r));
        }
        B.chainRevisionEnd[c] = int(B.revisionIndex.size());
        const int first = B.chainRevisionBegin[c];
        const int last = B.chainRevisionEnd[c];
        const int chunkFirst = B.chainChunkBegin[c];
        for (size_t r = 0; r < chain.revisions.size(); ++r) {
            RigExecBakedProgramImpl::GeomRevision &revision =
                chain.revisions[r];
            const int id = first + int(r);
            const bool skin = revision.op == RigExecRevisionOp::Skin;
            revision.influences.assign(revision.influenceSlots.size(),
                                       GfMatrix4d(1.0));
            // The table the PACKET carries for a skin revision, and never
            // anything else: identity, sized to the influence count and
            // written once, here.
            //
            // The packet is assembled before the matrices are folded, so it
            // cannot carry them -- that is what lets a chunk start on its own
            // joints without waiting for every joint of the rig. What the
            // assembler does with the table is check its SHAPE and its
            // elements' finiteness, and identities pass both; the real
            // table's finite/affine check is the fold's, and the fuse ANDs it
            // in exactly where the assembler's would have landed.
            revision.packetInfluences.assign(revision.influenceSlots.size(),
                                             GfMatrix4d(1.0));
            PartitionAtBuild(levels, &revision);
            revision.chunkBase = nextChunk;
            nextChunk += int(revision.chunks.size());
            B.revisionChunkBase.push_back(revision.chunkBase);
            B.revisionChunkCount.push_back(int(revision.chunks.size()));

            // The fold comes FIRST for every operation but a skin, because
            // every other operation's packet carries the matrix it was folded
            // from. A skin revision's does not, so its fold comes after the
            // assemble -- which is also where it learns the skinning method,
            // and so which form of the table the chunks will want.
            const auto addFold = [&] {
                RigExecBakedStep &fold = AddGeometryStep(
                    &B, RigExecBakedStepKind::InfluenceFold, id);
                DeclareMatrixReads(revision, &fold);
                // The delta a geometry-domain constraint measured for this
                // mover IS this revision's transform, so the fold waits for
                // the constraint step that writes it.
                if (revision.constraintDelta >= 0) {
                    fold.reads.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::ConstraintDelta,
                        revision.constraintDelta));
                }
                // A pose-walk read phase is answered out of the run's
                // snapshot store, so the fold waits for every recorder
                // before it -- which is what the pose walk's own constraint
                // exits are. The range is the same one the assemble
                // declares, and it is the assemble's for the same reason:
                // the store is ONE container and a reader of it has to be
                // ordered against every writer that could still reach it.
                if (revision.binding.transformPhase.kind ==
                    RigExecReadPhaseKind::AtPrim) {
                    fold.reads.push_back(RigExecBakedRange(
                        RigExecBakedSlotDomain::Snapshots, 0,
                        int(B.steps.size()) - 1));
                }
                if (skin) {
                    fold.reads.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::RevisionPacket, id));
                }
                fold.writes.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::RevisionTransforms, id));
            };
            const auto addStatic = [&] {
                RigExecBakedStep &assemble = AddGeometryStep(
                    &B, RigExecBakedStepKind::RevisionStatic, id);
                // One line per declared phase that resolved to nothing. The
                // defaultWeight line moved to the fuse, which is the first
                // step that knows both halves of "the packet is valid".
                assemble.maxDiagnostics = revision.binding.phases.size();
                if (!skin) {
                    assemble.reads.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::RevisionTransforms, id));
                }
                assemble.reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::ChainBase, int(c)));
                if (revision.weightObject >= 0) {
                    assemble.reads.push_back(
                        RigExecBakedOne(RigExecBakedSlotDomain::WeightPacket,
                                        revision.weightObject));
                }
                if (revision.weightCurrentPhase) {
                    // A current-phase field is measured against the points
                    // ENTERING this revision, so the assemble of a revision
                    // that would otherwise read nothing but the stage now
                    // waits for every buffer this chain has filled before
                    // it -- and for the indirection that says which of them
                    // holds the running value, exactly as a chunk does.
                    // The placement the oracle reads comes from the walk, so
                    // it waits for that too.
                    assemble.maxDiagnostics += 1;
                    assemble.reads.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::WeightFrames, 0));
                    if (id > first) {
                        assemble.reads.push_back(RigExecBakedRange(
                            RigExecBakedSlotDomain::RevisionOut, chunkFirst,
                            revision.chunkBase));
                        assemble.reads.push_back(RigExecBakedRange(
                            RigExecBakedSlotDomain::RevisionDone, first, id));
                        assemble.reads.push_back(RigExecBakedOne(
                            RigExecBakedSlotDomain::ChainDirty, id - 1));
                    }
                }
                if (revision.curvenetChain >= 0) {
                    assemble.reads.push_back(
                        RigExecBakedOne(RigExecBakedSlotDomain::ChainPoints,
                                        revision.curvenetChain));
                }
                if (revision.driverFramesSolver >= 0) {
                    assemble.reads.push_back(
                        RigExecBakedOne(RigExecBakedSlotDomain::Aggregate,
                                        revision.driverFramesSolver));
                }
                // A channel driven by a pose interpolator waits for the
                // interpolator's step, which is the edge that makes a drag
                // that cannot reach the driver leave this revision alone.
                for (const RigExecBakedProgramImpl::GeomBlendChannel &channel :
                         revision.blendChannels) {
                    if (channel.poseWeight >= 0) {
                        assemble.reads.push_back(RigExecBakedOne(
                            RigExecBakedSlotDomain::PoseWeight,
                            channel.poseWeight));
                    }
                }
                // readsSnapshots and not `!binding.phases.empty()`: a
                // blend sample's phase is looked up directly by the channel
                // gather rather than through the revision's input overlay,
                // so the overlay's own predicate does not cover it. It is a
                // strict superset of the old guard.
                if (revision.readsSnapshots) {
                    assemble.reads.push_back(RigExecBakedRange(
                        RigExecBakedSlotDomain::Snapshots, 0,
                        int(B.steps.size()) - 1));
                }
                assemble.writes.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::RevisionPacket, id));
                // It sizes the buffer the chunks write into, which is a write
                // to every one of their slots even though it fills none of
                // them.
                assemble.writes.push_back(RigExecBakedRange(
                    RigExecBakedSlotDomain::RevisionOut, revision.chunkBase,
                    revision.chunkBase + int(revision.chunks.size())));
            };
            if (skin) {
                addStatic();
                addFold();
            } else {
                addFold();
                addStatic();
            }
            for (size_t k = 0; k < revision.chunks.size(); ++k) {
                RigExecBakedStep &chunk = AddGeometryStep(
                    &B, RigExecBakedStepKind::RevisionChunk, id, int(k));
                chunk.reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::RevisionPacket, id));
                chunk.reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::ChainBase, int(c)));
                if (revision.chunked) {
                    // The whole point: this chunk waits for ITS influences
                    // and for nothing else. Not the fold, not the other
                    // chunks' joints -- it starts the instant its own are
                    // final and the fuse decides afterwards whether the work
                    // was wanted.
                    const RigExecBakedSlotDomain domain =
                        revision.finalPhase
                            ? RigExecBakedSlotDomain::FinalMatrix
                            : RigExecBakedSlotDomain::BaseMatrix;
                    for (const int position : revision.chunks[k].key) {
                        const int slot =
                            revision.influenceSlots[size_t(position)];
                        if (slot >= 0) {
                            chunk.reads.push_back(
                                RigExecBakedOne(domain, slot));
                        }
                    }
                } else if (skin) {
                    // One chunk is the whole array, so there is nothing to
                    // speculate about: it skins against the folded table.
                    chunk.reads.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::RevisionTransforms, id));
                }
                // Which earlier buffer holds the preceding points is a
                // runtime indirection, so the declaration is the upper
                // bound: every chunk this chain has filled before it -- AND
                // the indirection itself, which is the `currentSource` each
                // earlier fuse published into RevisionDone. Declaring the
                // buffers alone left a chunk free to run before the fuse that
                // decides which of them to read, which a serial order hides
                // and one cluster per step finds immediately. The whole
                // RevisionDone range, not just the last one: the buffers are
                // indexed by chunk now, so the range that names them cannot
                // also name the revisions whose fuses chose among them.
                if (id > first) {
                    chunk.reads.push_back(RigExecBakedRange(
                        RigExecBakedSlotDomain::RevisionOut, chunkFirst,
                        revision.chunkBase));
                    chunk.reads.push_back(RigExecBakedRange(
                        RigExecBakedSlotDomain::RevisionDone, first, id));
                    chunk.reads.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::ChainDirty, id - 1));
                }
                chunk.writes.push_back(
                    RigExecBakedOne(RigExecBakedSlotDomain::RevisionOut,
                                    revision.chunkBase + int(k)));
            }
            {
                RigExecBakedStep &fuse = AddGeometryStep(
                    &B, RigExecBakedStepKind::RevisionFuse, id);
                // The one the defaultWeight check can emit.
                fuse.maxDiagnostics = 1;
                fuse.reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::RevisionPacket, id));
                fuse.reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::RevisionTransforms, id));
                fuse.reads.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::ChainBase, int(c)));
                if (revision.weightObject >= 0) {
                    // The packet the MoverFailed arm consults. The assemble
                    // declares it too and the fuse waits on the assemble
                    // either way, so this changes no order -- but a step
                    // that touches a slot it did not declare is a bug even
                    // when a neighbour's declaration happens to cover it,
                    // and the next reader of either builder would have to
                    // rediscover why it was safe.
                    fuse.reads.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::WeightPacket,
                        revision.weightObject));
                }
                fuse.reads.push_back(RigExecBakedRange(
                    RigExecBakedSlotDomain::RevisionOut, chunkFirst,
                    revision.chunkBase + int(revision.chunks.size())));
                if (id > first) {
                    fuse.reads.push_back(RigExecBakedRange(
                        RigExecBakedSlotDomain::RevisionDone, first, id));
                    fuse.reads.push_back(RigExecBakedOne(
                        RigExecBakedSlotDomain::ChainDirty, id - 1));
                }
                fuse.writes.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::RevisionDone, id));
                fuse.writes.push_back(RigExecBakedOne(
                    RigExecBakedSlotDomain::ChainDirty, id));
                // Only on the path where the partition no longer describes
                // the layout: the fuse then runs the revision whole rather
                // than let a chunk deform a vertex against an identity.
                fuse.writes.push_back(RigExecBakedRange(
                    RigExecBakedSlotDomain::RevisionOut, revision.chunkBase,
                    revision.chunkBase + int(revision.chunks.size())));
                if (revision.snapshotAfter) {
                    fuse.writes.push_back(
                        RigExecBakedOne(RigExecBakedSlotDomain::Snapshots,
                                        int(B.steps.size()) - 1));
                }
            }
        }
        B.chainChunkEnd[c] = nextChunk;
        {
            RigExecBakedStep &status =
                AddGeometryStep(&B, RigExecBakedStepKind::ChainStatus, int(c));
            // One MoverFailed line per revision of the chain, which is the
            // sweep's whole output.
            status.maxDiagnostics = chain.revisions.size();
            status.reads.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::ChainBase, int(c)));
            if (last > first) {
                status.reads.push_back(RigExecBakedRange(
                    RigExecBakedSlotDomain::RevisionDone, first, last));
                status.reads.push_back(RigExecBakedRange(
                    RigExecBakedSlotDomain::RevisionOut, chunkFirst,
                    nextChunk));
            }
            status.writes.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::ChainPoints, int(c)));
            status.writes.push_back(RigExecBakedOne(
                RigExecBakedSlotDomain::Snapshots, int(B.steps.size()) - 1));
        }
        for (size_t d = 0; d < chain.derived.size(); ++d) {
            RigExecBakedProgramImpl::GeomChain::Derived &derived =
                chain.derived[d];
            const int id = int(B.derivedIndex.size());
            B.derivedIndex.emplace_back(int(c), int(d));
            derived.revision.influences.assign(
                derived.revision.influenceSlots.size(), GfMatrix4d(1.0));
            RigExecBakedStep &step =
                AddGeometryStep(&B, RigExecBakedStepKind::Derived, id);
            step.maxDiagnostics =
                derived.revision.binding.phases.size() + 1;
            // The chain's FINAL points, deliberately: recomputeNormals and
            // recomputeExtent describe the geometry as published. The
            // target's own authored base is prologue state, read beside the
            // chain's.
            step.reads.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::ChainPoints, int(c)));
            step.reads.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::ChainBase, int(c)));
            // A derived revision's binding carries phases like any other, and
            // its assemble reads the run's store for them -- so it declares
            // the store the same way a chain revision does: every step before
            // it, because which of them recorded what is a runtime answer.
            if (derived.revision.readsSnapshots) {
                step.reads.push_back(RigExecBakedRange(
                    RigExecBakedSlotDomain::Snapshots, 0,
                    int(B.steps.size()) - 1));
            }
            DeclareMatrixReads(derived.revision, &step);
            step.writes.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::DerivedOut, id));
        }
    }
}

// ---------------------------------------------------------------------------
// One frame, geometry half.
//
// The packet is assembled by the same RigExecAssembleParameters the dynamic
// path calls and the kernel is the same shared kernel; only the VdfNetwork
// around them is baked away, so the accounting it performed -- a revision
// runs when its inputs changed, and not otherwise -- is performed here
// instead, by the four steps a revision is made of.
// ---------------------------------------------------------------------------

namespace {

/// The points the chain holds BEFORE revision \p r.
///
/// Not a buffer of its own: `currentSource` names the revision whose output
/// buffer last held the running value, which is -1 for the authored base. It
/// is the indirection that replaces today's `revision.output = current` copy
/// of a revision that applied nothing.
void
PointsBefore(const RigExecBakedProgramImpl::GeomChain &chain, size_t r,
             const GfVec3f **points, size_t *count)
{
    const int source =
        r == 0 ? -1 : chain.revisions[r - 1].currentSource;
    if (source < 0) {
        *points = chain.lastBase.cdata();
        *count = chain.lastBase.size();
        return;
    }
    *points = chain.revisions[size_t(source)].output.data();
    *count = chain.revisions[size_t(source)].output.size();
}

/// The points the chain holds AFTER revision \p r.
void
PointsAfter(const RigExecBakedProgramImpl::GeomChain &chain, size_t r,
            const GfVec3f **points, size_t *count)
{
    const int source = chain.revisions[r].currentSource;
    if (source < 0) {
        *points = chain.lastBase.cdata();
        *count = chain.lastBase.size();
        return;
    }
    *points = chain.revisions[size_t(source)].output.data();
    *count = chain.revisions[size_t(source)].output.size();
}

/// The influence matrices of \p revision, out of the tables the pose half's
/// ProviderMatrix steps published. Returns whether the table MOVED.
///
/// The compare is here rather than in the packet comparison because a skin
/// revision's packet no longer carries the table: the half of the executed
/// decision that used to read `skinTransforms == o.skinTransforms` reads this
/// instead, and it is the same comparison over the same values.
bool
FoldInfluences(const RigExecBakedProgramImpl &B,
               RigExecBakedProgramImpl::GeomRevision *revision)
{
    bool changed = false;
    revision->haveTransform = revision->transformSlot >= 0;
    if (revision->haveTransform) {
        revision->transform =
            revision->finalPhase
                ? B.finalMatrix[size_t(revision->transformSlot)]
                : B.baseMatrix[size_t(revision->transformSlot)];
    }
    // A geometry-domain constraint's delta, if one was measured for this
    // mover: the pose walk solved the constraint and stashed the map from the
    // target's authored transform to the solved one, and THIS is the Matrix
    // revision that same constraint contributes. Find-guarded, and here
    // rather than in the assemble because the transform table is what this
    // step declares -- the assemble only READS it, and a step that writes a
    // slot it declared as a read is the declaration the executor trusts
    // being wrong.
    //
    // The dynamic path applies it before the final-phase substitution rather
    // than after. The two orders can only disagree for a revision that has
    // both a bound transform provider and a delta, which cannot arise: a
    // geometry-domain constraint's binding.transform is empty -- that is what
    // makes the delta the only source of the matrix.
    if (revision->constraintDelta >= 0 &&
        B.deltaPresent[size_t(revision->constraintDelta)]) {
        revision->transform = B.deltaValues[size_t(revision->constraintDelta)];
        revision->haveTransform = true;
    }
    // A read phase naming a POINT IN THE POSE WALK, which is the general
    // form `base`, `preceding` and `final` abbreviate: the provider's matrix
    // as it stood immediately after one named constraint, out of the run's
    // snapshot store instead of out of the dense tables.
    //
    // AFTER the delta, exactly as the dynamic path orders the two, and the
    // two can no more both apply here than they can there: an AtPrim phase
    // needs a bound rigExec:transform to name a frame chain of, and a
    // geometry-domain constraint's binding.transform is empty -- which is
    // what makes its delta the only source of the matrix.
    //
    // A phase that resolved to NOTHING leaves the dense-table value
    // standing and says nothing, because that is what the dynamic path does
    // with it: compile has already refused a phase naming a prim that
    // revises the provider nowhere, so the only way to get here is a
    // constraint whose exit this run declined to record -- an unusable
    // frame -- and the base matrix is then the same fallback both paths
    // take.
    const auto phased = [&B, revision](const SdfPath &provider,
                                       GfMatrix4d *matrix) {
        const VtValue *recorded = B.runSnapshots.Lookup(
            provider, revision->binding.transformPhase, revision->moverPath);
        if (recorded && recorded->IsHolding<GfMatrix4d>()) {
            *matrix = recorded->UncheckedGet<GfMatrix4d>();
            return true;
        }
        return false;
    };
    const bool atPrim = revision->binding.transformPhase.kind ==
                        RigExecReadPhaseKind::AtPrim;
    if (atPrim && phased(revision->binding.transform, &revision->transform)) {
        // Set rather than left alone: the dynamic path binds its matrix
        // pointer whenever the store answered, whatever the tap held.
        revision->haveTransform = true;
    }
    if (revision->haveTransform && revision->transformSpaceSlot >= 0) {
        const GfMatrix4d &space =
            revision->finalPhase
                ? B.finalMatrix[size_t(revision->transformSpaceSlot)]
                : B.baseMatrix[size_t(revision->transformSpaceSlot)];
        revision->transform =
            RigExecMeasureInSpace(revision->transform, space);
    }
    for (size_t k = 0; k < revision->influenceSlots.size(); ++k) {
        const size_t slot = size_t(revision->influenceSlots[k]);
        GfMatrix4d matrix = revision->finalPhase ? B.finalMatrix[slot]
                                                 : B.baseMatrix[slot];
        // Every influence shares the one declared phase, so the substitution
        // is per ENTRY and the provider is the entry's own. No rig reaches
        // this today: compile validates an AtPrim phase against
        // binding.transform alone, and a skin mover's is empty, so such a
        // rig is refused before either path evaluates it. Written anyway,
        // and written the same way, because the dynamic fold does exactly
        // this with its influence entries -- the day the validator learns
        // about rigExec:influences, the two paths already agree.
        if (atPrim && k < revision->binding.influences.size()) {
            phased(revision->binding.influences[k], &matrix);
        }
        if (revision->influences[k] != matrix) {
            revision->influences[k] = matrix;
            changed = true;
        }
    }
    return changed;
}

/// The revision's own folded table, in the forms the kernels want it in.
///
/// What a chunk of a revision cut into ONE chunk skins against, and what the
/// fuse falls back to when the partition no longer describes the layout. A
/// chunked revision's chunks keep their own tables beside these and never
/// read them; the fold writes them anyway, because they are part of the
/// RevisionTransforms slot and the fuse may only READ that slot.
RigExecSkinTransformsView
WholeTransformsView(RigExecBakedProgramImpl::GeomRevision *revision)
{
    RigExecSkinTransformsView view;
    view.transforms = revision->influences.data();
    view.transformCount = revision->influences.size();
    if (!revision->rows.empty()) {
        view.rows = revision->rows.data();
    }
    if (!revision->palette.empty()) {
        view.palette = revision->palette.data();
        view.paletteSize = revision->palette.size();
    }
    return view;
}

/// Narrows and splits \p revision's folded table for the method its packet
/// names, once per run rather than once per range.
///
/// Part of what InfluenceFold writes, for every skin revision and not only
/// an unchunked one: the whole-array fallback the fuse falls back to reads
/// these, and a step may not write a slot it declared as a read.
void
FoldTransformForms(RigExecBakedProgramImpl::GeomRevision *revision)
{
    static const bool useSimd = TfGetenvBool("RIGEXEC_ENABLE_SIMD", true);
    const size_t count = revision->influences.size();
    if (revision->parameters.skinningMethod == _tokens->dualQuaternion) {
        revision->rows.clear();
        revision->palette.resize(count + 1);
        for (size_t t = 0; t < count; ++t) {
            revision->palette[t] =
                RigExecScaledDualQuatFromMatrix(revision->influences[t]);
        }
        // The trailing entry is the weight complement's identity influence,
        // exactly as RigExecSkinDualQuatPalette appends it.
        revision->palette[count] = RigExecScaledDualQuat();
        return;
    }
    revision->palette.clear();
    if (!useSimd) {
        revision->rows.clear();
        return;
    }
    revision->rows.resize(count * RigExecSkinRowStride);
    for (size_t t = 0; t < count; ++t) {
        RigExecNarrowSkinRows(revision->influences[t],
                              &revision->rows[t * RigExecSkinRowStride]);
    }
}

/// Copies \p chunk's OWN influences out of the matrix slots, keeping the
/// narrowed and split forms beside them, and records whether any moved.
///
/// Only |key| entries, never the whole table: that is what makes a chunk's
/// per-run cost proportional to the joints its vertices actually reference.
/// The entries outside the key stay identity, and linear blend skinning
/// reaches an entry only through an index of one of the chunk's own vertices,
/// so they are never read.
void
GatherChunkTransforms(const RigExecBakedProgramImpl &B,
                      const RigExecBakedProgramImpl::GeomRevision &revision,
                      RigExecBakedProgramImpl::GeomChunk *chunk)
{
    const bool dualQuat =
        revision.parameters.skinningMethod == _tokens->dualQuaternion;
    const size_t count = chunk->transforms.size();
    const bool splitAlready = chunk->palette.size() == count + 1;
    for (const int position : chunk->key) {
        const int slot = revision.influenceSlots[size_t(position)];
        if (slot < 0) {
            continue;
        }
        const GfMatrix4d &matrix = revision.finalPhase
                                       ? B.finalMatrix[size_t(slot)]
                                       : B.baseMatrix[size_t(slot)];
        if (chunk->transforms[size_t(position)] == matrix) {
            continue;
        }
        chunk->transforms[size_t(position)] = matrix;
        chunk->keyChanged = true;
        RigExecNarrowSkinRows(
            matrix, &chunk->rows[size_t(position) * RigExecSkinRowStride]);
        if (splitAlready) {
            chunk->palette[size_t(position)] =
                RigExecScaledDualQuatFromMatrix(matrix);
        }
    }
    if (dualQuat && !splitAlready) {
        // First run of a dual-quaternion revision: the whole table at once,
        // which is also what keeps the entries outside the key -- identity,
        // and never read -- in step with the matrices beside them.
        chunk->palette.resize(count + 1);
        for (size_t t = 0; t < count; ++t) {
            chunk->palette[t] =
                RigExecScaledDualQuatFromMatrix(chunk->transforms[t]);
        }
        chunk->palette[count] = RigExecScaledDualQuat();
    }
}

/// The view onto \p chunk's own tables.
RigExecSkinTransformsView
ChunkTransformsView(const RigExecBakedProgramImpl::GeomChunk &chunk)
{
    RigExecSkinTransformsView view;
    view.transforms = chunk.transforms.data();
    view.transformCount = chunk.transforms.size();
    static const bool useSimd = TfGetenvBool("RIGEXEC_ENABLE_SIMD", true);
    if (useSimd && !chunk.rows.empty()) {
        view.rows = chunk.rows.data();
    }
    if (!chunk.palette.empty()) {
        view.palette = chunk.palette.data();
        view.paletteSize = chunk.palette.size();
    }
    return view;
}

/// One vertex range of a skin revision: seed the revision's own output buffer
/// from the preceding points, deform in place, blend the envelope back.
///
/// The three pieces RigExecRunRevisionKernel performs for the whole array,
/// performed over a range -- with every whole-array decision (the packet
/// check, the layout validation, the envelope's resolution, the size check)
/// already made by the steps that own them.
///
/// \p whole runs the full-range kernel instead, which is the same body
/// inside the kernel's own parallel loop; a revision cut into one chunk uses
/// it so that an unpartitioned mesh keeps the threading it has today.
bool
SkinRange(RigExecBakedProgramImpl::GeomRevision *revision,
          const GfVec3f *preceding, const RigExecSkinTransformsView &view,
          size_t begin, size_t end, bool whole)
{
    std::vector<GfVec3f> &out = revision->output;
    std::copy(preceding + begin, preceding + end, out.begin() + long(begin));
    if (whole) {
        if (!RigExecApplySkinKernelWithTransforms(revision->parameters, view,
                                                  &out)) {
            return false;
        }
    } else if (!RigExecApplySkinKernelRange(revision->parameters, view, begin,
                                            end, &out)) {
        return false;
    }
    // "Apply once", over the same range: the envelope was resolved at the
    // full count by RevisionStatic, because resolving it is atomic over the
    // whole array, and every array here is indexed absolutely. The whole
    // array goes through the kernel's own split for the reason the skinning
    // above does: an unpartitioned mesh keeps the threading it has today.
    if (!revision->fullStrength) {
        if (whole) {
            // `whole` means the whole array -- the full-range kernel above
            // ignores the bounds -- so begin is 0 and the count is `end`.
            RigExecBlendEnvelopeAll(preceding, revision->envelope.data(), end,
                                    out.data());
        } else {
            RigExecBlendEnvelopeRange(preceding, revision->envelope.data(),
                                      begin, end, out.data());
        }
    }
    return true;
}

// WHICH points a revision is assembled against, stated once because the two
// callers below pass different ones and the difference is invisible until an
// operator reads them:
//
//  * a CHAIN revision gets the chain's AUTHORED base -- the attribute as the
//    stage holds it -- for every revision of the chain, not the running
//    value. The dynamic walk reads the base once per target and hands that
//    same array to every revision, so a lattice's rest points and a
//    volumeCorrect's reference volume are measured against the mesh as
//    authored however deep in the chain they sit.
//  * a DERIVED revision gets the chain's FINAL points, which is what it is
//    for: recomputeNormals and recomputeExtent take auxPoints = basePoints
//    and must describe the geometry as published.
//
// Taken as a range rather than a vector because the two callers hold the
// points in different containers and RigExecProviderValues copies them
// anyway.
RigExecMoverParameters
AssembleRevision(RigExecBakedProgramImpl &B,
                 RigExecBakedProgramImpl::GeomRevision *revision,
                 const GfVec3f *basePoints, size_t basePointCount,
                 UsdTimeCode time, RigExecBakedStep *step)
{
    const RigExecResolvedInputs &R = *B.resolvedInputs;
    RigExecProviderValues values;
    values.resolved = &R;
    values.curvenetBindInputs = &revision->curvenetBindInputs;
    // One overlay per REVISION: the generation-wide resolved inputs, plus
    // whatever this revision's declared phases resolve to out of the run's
    // snapshot store. The assembler reads inputs by path and never learns a
    // phase exists, which is what lets a phase apply to any input. Built only
    // for a revision that declares one, and owned by that revision, because
    // one buffer shared by the walk is state two steps could be inside at
    // once.
    if (!revision->binding.phases.empty()) {
        revision->revisionInputs = R;
        for (const auto &[inputPath, phase] : revision->binding.phases) {
            if (const VtValue *recorded = B.runSnapshots.Lookup(
                    inputPath, phase, revision->moverPath)) {
                revision->revisionInputs.SetProperty(inputPath, *recorded);
            } else if (phase.kind != RigExecReadPhaseKind::Preceding) {
                // Preceding falling through to the stage is correct: the
                // reader is the chain's first revision, so its preceding
                // value IS the base. Anything else means the phase named
                // something that produced nothing.
                step->diagnostics.push_back(
                    "diag " + revision->moverPath.GetString() +
                    ": read phase '" + phase.GetAsString() + "' for " +
                    inputPath.GetString() +
                    " resolved to nothing; read the authored base");
            }
        }
        values.resolved = &revision->revisionInputs;
    }
    // The fold decided whether there is a matrix at all -- a bound transform
    // provider, or a geometry-domain constraint's delta -- and wrote it.
    //
    // A SKIN revision is assembled without one, and that is not an omission:
    // its static step runs BEFORE its fold, precisely so its chunks wait for
    // their own joints rather than for the rig's, so the fold's matrix here
    // would be the one last run measured. Nothing reads it --
    // RigExecAssembleSkinParameters takes no transform, which is also how
    // the dynamic path treats a delta landing on a skin mover -- so the
    // packet is assembled without a matrix rather than with a stale one.
    if (revision->haveTransform && revision->op != RigExecRevisionOp::Skin) {
        values.transform = &revision->transform;
    }
    // A skin revision's packet carries the IDENTITY table and never the
    // folded one: it is assembled before the matrices are folded, which is
    // what lets its chunks start on their own joints. Every other operation's
    // packet carries what the fold wrote -- which is nothing today, because a
    // skin is the only operation that binds influences at all.
    const std::vector<GfMatrix4d> &table =
        revision->op == RigExecRevisionOp::Skin ? revision->packetInfluences
                                                : revision->influences;
    if (!table.empty()) {
        values.influenceTransforms = &table;
    }
    // The weight object's packet, shared with every other mover that binds
    // it and built by its own step earlier in the frame. A mover copies EXEC
    // here (see bakedWeights.cpp): this is the packet the tap would have
    // carried, and `inputs:defaultWeight` is deliberately NOT consulted
    // beside it -- the assembler falls back to that scalar only when no
    // packet is bound at all.
    if (revision->weightObject >= 0) {
        values.weights = revision->weightCurrentPhase
                             ? &revision->currentPhasePacket
                             : &B.weightPackets[size_t(revision->weightObject)];
    }
    // Only the operations that measure against the authored base read it; a
    // matrix, skin or wire revision never does, and copying a whole body's
    // points per revision per frame was most of what such a step cost.
    if (revision->op != RigExecRevisionOp::Matrix &&
        revision->op != RigExecRevisionOp::Skin &&
        revision->op != RigExecRevisionOp::Wire) {
        values.basePoints.assign(basePoints, basePoints + basePointCount);
    }
    // The curvenet: the bind out of the program's OWN cache, resolved in the
    // prologue because that cache has no locking and reports a diagnostic per
    // bind; and the POSED net, which is the net's own chain result. The
    // dynamic walk takes the latter out of pose.movedProperties, which is the
    // same array this chain published -- the walk runs a net's chain before
    // any mover that reads it and so does the program.
    if (revision->op == RigExecRevisionOp::Curvenet) {
        if (revision->curvenetBindResolved) {
            values.curvenetBinding = &revision->curvenetBind;
        } else {
            // The prologue's own call, which is the one that binds.
            values.curvenetCache = &B.curvenetBindings;
        }
        if (revision->curvenetChain >= 0) {
            const RigExecBakedProgramImpl::GeomChain &net =
                B.chains[size_t(revision->curvenetChain)];
            if (net.haveBase && net.haveResult) {
                values.curvenetPoints.assign(net.result.begin(),
                                             net.result.end());
            }
        }
    }
    // The driver solver's aggregate, which is what the dynamic path's
    // per-revision tap resolves to: exec computes one and the pose walk then
    // OVERRIDES the tap with its own solve, so the program's table -- the
    // same solve, by the same kernel -- is the authoritative value on both
    // paths. Declared as a read of the Aggregate slot, which is the edge
    // from the solver's Solve step to this revision.
    if (revision->driverFramesSolver >= 0) {
        values.driverFrames =
            &B.aggregates[size_t(revision->driverFramesSolver)];
    }
    // The blend channels. Gathered here rather than inside the assembler
    // because the dynamic walk gathers them here too, and the ORDER is the
    // whole contract: channels in `binding.blendInputs` order, which compile
    // sorted, and each channel's samples stable-sorted by activation. Float
    // addition is not associative, so a different order is a different last
    // bit of every point.
    //
    // The reads go through the generation-wide resolved inputs and NOT
    // through this revision's phase overlay: a sample's own phase is looked
    // up directly, which is what lets one blend sample read another chain
    // while the rest of the channel reads the stage.
    if (!revision->blendChannels.empty()) {
        std::vector<RigExecBlendChannel> channels;
        channels.reserve(revision->blendChannels.size());
        for (size_t c = 0; c < revision->blendChannels.size(); ++c) {
            RigExecBakedProgramImpl::GeomBlendChannel &bound =
                revision->blendChannels[c];
            RigExecBlendChannel channel;
            // A pose-driven weight is this run's slot -- unless an override
            // stands on the weight itself, which the dynamic walk's lookup
            // finds before it follows the connection and which the resolved
            // inputs therefore answer here too.
            if (bound.poseWeight >= 0 && !R.Find(bound.weightPath)) {
                channel.weight = B.poseWeights[size_t(bound.poseWeight)];
            } else {
                R.GetAttribute(bound.weight, time, &channel.weight);
            }
            // Retained for the bake beside the read, before the accumulate
            // below consumes the local: what the record carries is what the
            // gather gathered, whatever route it came by.
            bound.lastWeight = channel.weight;
            for (size_t s = 0; s < bound.samples.size(); ++s) {
                auto &boundSample = bound.samples[s];
                RigExecBlendSampleData sample;
                R.GetAttribute(boundSample.activation, time,
                               &sample.activation);
                boundSample.lastActivation = sample.activation;
                if (!boundSample.blendShape.IsEmpty()) {
                    // Sparse: the shape the prologue resolved, shared by
                    // pointer. The packet compares layouts by pointer, so an
                    // unchanged sample costs one compare and not an array.
                    sample.layout = boundSample.layout;
                    channel.samples.push_back(std::move(sample));
                    continue;
                }
                VtVec3fArray points;
                const VtValue *phased = B.runSnapshots.Lookup(
                    boundSample.pointsPath, boundSample.phase,
                    revision->moverPath);
                if (phased && phased->IsHolding<VtVec3fArray>()) {
                    points = phased->UncheckedGet<VtVec3fArray>();
                } else {
                    R.GetAttribute(boundSample.points, time, &points);
                }
                sample.points.assign(points.begin(), points.end());
                boundSample.lastPoints.assign(points.begin(), points.end());
                channel.samples.push_back(std::move(sample));
            }
            std::stable_sort(channel.samples.begin(), channel.samples.end(),
                             [](const RigExecBlendSampleData &a,
                                const RigExecBlendSampleData &b) {
                                 return a.activation < b.activation;
                             });
            channels.push_back(std::move(channel));
        }
        // A structural failure leaves blendDeltas empty, which is what makes
        // the assembled packet invalid -- the same atomic MoverFailed
        // pass-through the kernel produces.
        if (!RigExecSumBlendChannels(channels, values.basePoints,
                                     &values.blendDeltas)) {
            values.blendDeltas.clear();
        }
    }
    // Epoch-fixed layouts were resolved in the PROLOGUE, through the same
    // evaluator cache the dynamic path's assembly resolves through -- so the
    // two paths cannot hold different arrays, and no step body takes the
    // cache's lock.
    if (revision->topologyResolved) {
        values.skinTopology = &revision->topology;
    }
    return RigExecAssembleParameters(revision->moverPrim, revision->op,
                                     revision->binding, values, time);
}

/// The skin revision over its whole array, out of the fuse.
///
/// The path where the partition no longer describes the layout the packet
/// carries -- a weight-paint edit the epoch let through, an interactive
/// override on the indices, a cache that refused the mover after all. The
/// keys cannot be trusted, so the chunks stand down and the one step that
/// holds both the folded table and the preceding points runs the revision
/// whole. Serial, cold, and exactly the arithmetic an unchunked revision
/// would have performed.
bool
FuseWholeRevision(const RigExecBakedProgramImpl::GeomChain &chain,
                  RigExecBakedProgramImpl::GeomRevision *revision,
                  size_t revisionIndex)
{
    const GfVec3f *points = nullptr;
    size_t count = 0;
    PointsBefore(chain, revisionIndex, &points, &count);
    if (!revision->layoutUsable || !revision->envelopeOk ||
        count != revision->precedingCount ||
        revision->output.size() != count) {
        return false;
    }
    // Against the forms the FOLD wrote -- this step reads RevisionTransforms
    // and writes none of it, so the table it skins against is the one that
    // slot already holds.
    return SkinRange(revision, points, WholeTransformsView(revision), 0, count,
                     /*whole=*/true);
}

}  // namespace

void
RigExecBakedRunGeometryPrologue(RigExecBakedProgramImpl *program,
                                UsdTimeCode time, RigExecRigPose *pose)
{
    RigExecBakedProgramImpl &B = *program;
    RIGEXEC_PROFILE_SCOPE_CAT(*B.profiler, "BakedChainBase", "geometry");
    // Everything a frame reads off the stage for a chain, plus the one lock a
    // frame takes. Building the geometry state is reported once per node and
    // once per schedule, not once per program: a program rebuilt over state
    // that survived (AdoptGeometryStateFrom) has nothing to report, which is
    // what the dynamic path says about the same edit. Counted here, because a
    // chain can lose its state at the head of a frame -- a point count that
    // moved with time -- and that is a node the dynamic path adds again too.
    const auto resetRevision =
        [](RigExecBakedProgramImpl::GeomRevision *revision) {
        revision->created = true;
        revision->ran = false;
        revision->output.clear();
        revision->currentSource = -1;
        revision->lastParameters = RigExecMoverParameters();
        revision->lastAuxPoints = VtVec3fArray();
        revision->lastStatus = RigExecMoverStatus();
    };
    const auto resolveTopology =
        [&B, time](RigExecBakedProgramImpl::GeomRevision *revision) {
        if (!revision->skinTopologyFixed) {
            return;
        }
        revision->topology = RigExecResolveSkinTopology(
            revision->moverPrim, revision->influenceSlots.size(), time,
            B.resolvedInputs, B.skinTopologies);
        revision->topologyResolved = true;
        // The partition is Build state, and this is where a frame can tell
        // in O(1) whether it still describes the vertices: the cache hands
        // back the SAME layout pointer for a binding that did not move, so a
        // different one means the arrays did -- a weight-paint edit, or an
        // override placed on the indices. Re-cutting keeps the chunk COUNT,
        // because the number of steps a revision is made of was fixed at
        // Build and a value edit may not change the program. Here rather
        // than in a step because it reads the arrays, and because the first
        // frame of an epoch is the one that pays for it.
        if (!revision->chunked ||
            revision->topology == revision->partitionTopology) {
            return;
        }
        if (!revision->topology) {
            // A refusal leaves the partition AND the handle it was cut from
            // where they are: the packet then reads the arrays per frame,
            // and RevisionStatic, which has no resolved layout to compare
            // the handle of, routes the revision through the fuse's
            // whole-array path. Recording the refusal here instead would
            // make the handles agree by both being null, which is the one
            // answer this comparison must never give.
            return;
        }
        RigExecBakedPartitionRevision(
            revision, revision->topology->indices.data(),
            revision->topology->indices.size(),
            revision->topology->elementSize,
            int(revision->chunks.size()));
        revision->partitionTopology = revision->topology;
    };
    // The Profile Mover's bind, resolved HERE and never in a step.
    //
    // RigExecCurvenetBindCache mutates its entry map and appends to its
    // pending diagnostics with no synchronisation whatever, so it belongs to
    // serial code -- and a step body may take no lock to make it belong
    // anywhere else. Everything the bind depends on is read at Default with
    // the resolved inputs bypassed, so the answer cannot depend on where in
    // the frame it is taken; what the packet's own assembly then adds is the
    // POSED net, which decides no part of the cut and is compared only
    // against the rest net's cardinality. The bind is therefore this frame's
    // whichever points stand in the net chain's buffer while the prologue
    // runs, and the step is left with a pointer and no cache.
    //
    // The price is one extra curvenet packet assembly per frame per curvenet
    // mover, paid so that the bind cannot be reached from two threads. It is
    // the same assembly the dynamic walk performs once, over the same arrays.
    const auto resolveCurvenetBind =
        [&B, time](const RigExecBakedProgramImpl::GeomChain &chain,
                   RigExecBakedProgramImpl::GeomRevision *revision) {
        if (revision->op != RigExecRevisionOp::Curvenet) {
            return;
        }
        revision->curvenetBindResolved = false;
        RigExecBakedStep scratch;
        const RigExecMoverParameters bound =
            AssembleRevision(B, revision, chain.lastBase.cdata(),
                             chain.lastBase.size(), time, &scratch);
        revision->curvenetBind = bound.curvenetBinding;
        revision->curvenetBindResolved = true;
    };
    // A sparse blend sample's shape, resolved HERE and never in a step: the
    // cache takes a lock, and a refusal means the shape is read off the stage
    // per frame. Both are the prologue's. The dynamic walk resolves through
    // the same cache at its own assembly, so the two paths hold the same
    // pointer for a shape that did not move.
    const auto resolveBlendLayouts =
        [&B](RigExecBakedProgramImpl::GeomRevision *revision,
             size_t pointCount) {
        for (RigExecBakedProgramImpl::GeomBlendChannel &channel :
                 revision->blendChannels) {
            for (RigExecBakedProgramImpl::GeomBlendChannel::Sample &sample :
                     channel.samples) {
                if (sample.blendShape.IsEmpty()) {
                    continue;
                }
                sample.layout = B.blendSampleShapes->Resolve(
                    sample.samplePath,
                    [&](RigExecBlendSampleLayout *layout) {
                        return B.resolveBlendSample(sample.blendShape,
                                                    pointCount, layout);
                    });
                sample.layoutRefused = !sample.layout;
                if (!sample.layout) {
                    // Refused the cache: something about the shape can move
                    // inside this epoch, so it is read per frame instead.
                    auto perFrame = std::make_shared<RigExecBlendSampleLayout>();
                    B.resolveBlendSample(sample.blendShape, pointCount,
                                         perFrame.get());
                    sample.layout = perFrame;
                }
            }
        }
    };
    for (RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        VtVec3fArray basePoints;
        chain.haveBase =
            chain.baseQuery.IsValid() && chain.baseQuery.Get(&basePoints, time);
        if (!chain.haveBase) {
            // The target's points do not read at this time at all; the
            // dynamic path drives nothing for it and neither does the
            // program, counters included.
            for (RigExecBakedProgramImpl::GeomChain::Derived &derived :
                     chain.derived) {
                derived.haveBase = false;
            }
            continue;
        }
        if (chain.haveResult && basePoints.size() != chain.lastBase.size()) {
            // A time-varying point count replaces THIS target's graph in the
            // dynamic path and leaves every other target's standing. Same
            // here: the cached run describes a different mesh, so it is
            // dropped and the chain's nodes are reported as created -- which
            // is what the dynamic walk reports for the nodes it has to add
            // again. Handing the whole generation back instead would degrade
            // every other chain of the rig for one mesh whose vertex count is
            // keyed.
            chain.haveResult = false;
            chain.result = VtVec3fArray();
            chain.scheduleDirty = true;
            for (RigExecBakedProgramImpl::GeomRevision &revision :
                     chain.revisions) {
                resetRevision(&revision);
            }
        }
        for (RigExecBakedProgramImpl::GeomRevision &revision :
                 chain.revisions) {
            if (revision.created) {
                ++pose->moverGraphRevisionsCreated;
                revision.created = false;
            }
        }
        if (chain.scheduleDirty && !chain.revisions.empty()) {
            ++pose->moverGraphSchedulesBuilt;
        }
        chain.scheduleDirty = false;
        chain.baseDirty = !chain.haveResult || basePoints != chain.lastBase;
        chain.lastBase = basePoints;
        for (RigExecBakedProgramImpl::GeomRevision &revision :
                 chain.revisions) {
            resolveTopology(&revision);
            resolveBlendLayouts(&revision, basePoints.size());
            resolveCurvenetBind(chain, &revision);
        }
        for (RigExecBakedProgramImpl::GeomChain::Derived &derived :
                 chain.derived) {
            VtVec3fArray derivedBase;
            derived.haveBase = derived.baseQuery.IsValid() &&
                               derived.baseQuery.Get(&derivedBase, time);
            if (!derived.haveBase) {
                continue;
            }
            if (derived.haveResult &&
                derivedBase.size() != derived.lastBase.size()) {
                // As above, and for the same reason: the dynamic walk
                // replaces this derived target's graph alone.
                derived.haveResult = false;
                derived.result = VtVec3fArray();
                resetRevision(&derived.revision);
            }
            // A derived target is one node with one schedule of its own,
            // created together, exactly as the dynamic walk creates them.
            if (derived.revision.created) {
                ++pose->moverGraphRevisionsCreated;
                ++pose->moverGraphSchedulesBuilt;
                derived.revision.created = false;
            }
            derived.baseDirty =
                !derived.haveResult || derivedBase != derived.lastBase;
            derived.lastBase = derivedBase;
            resolveTopology(&derived.revision);
        }
    }
}

void
RigExecBakedSkipGeometryStep(RigExecBakedProgramImpl *program,
                             RigExecBakedStep *step)
{
    RigExecBakedProgramImpl &B = *program;
    if (step->kind == RigExecBakedStepKind::Derived) {
        RigExecBakedProgramImpl::GeomRevision &revision =
            B.chains[size_t(B.derivedIndex[size_t(step->object)].first)]
                .derived[size_t(B.derivedIndex[size_t(step->object)].second)]
                .revision;
        revision.influencesChanged = false;
        return;
    }
    if (step->kind == RigExecBakedStepKind::ChainStatus) {
        return;  // it writes no delta: the sweep is over persisted status
    }
    const auto &[chainIndex, revisionIndex] =
        B.revisionIndex[size_t(step->object)];
    RigExecBakedProgramImpl::GeomRevision &revision =
        B.chains[size_t(chainIndex)].revisions[size_t(revisionIndex)];
    switch (step->kind) {
    case RigExecBakedStepKind::InfluenceFold:
        // The table is where the fold left it, so nothing moved into it.
        revision.influencesChanged = false;
        return;
    case RigExecBakedStepKind::RevisionStatic:
        revision.staticDirty = false;
        return;
    case RigExecBakedStepKind::RevisionChunk:
        revision.chunks[size_t(step->part)].keyChanged = false;
        if (!revision.chunked) {
            // A whole-array chunk opens every run saying it has produced
            // nothing, and says otherwise only where the revision executed;
            // a run that skipped it produced nothing either. The
            // SPECULATIVE form's `ok` is the opposite kind of flag -- it
            // describes what its range of the buffer holds, across runs,
            // and the step reads it back to decide whether it may keep it
            // -- so that one is left exactly where the chunk left it.
            revision.chunks[size_t(step->part)].ok = false;
        }
        return;
    case RigExecBakedStepKind::RevisionFuse:
        // The chain's sticky dirty bit as the NEXT revision reads it: this
        // revision did not execute this run, whatever it did during the last
        // one. `currentSource`, `resultStatus` and the points stay where the
        // fuse left them -- those are the values, not the comparison.
        revision.executed = false;
        return;
    default:
        return;
    }
}

void
RigExecBakedRunGeometryStep(RigExecBakedProgramImpl *program,
                            RigExecBakedStep *step, UsdTimeCode time)
{
    RigExecBakedProgramImpl &B = *program;
    const RigExecResolvedInputs &R = *B.resolvedInputs;
    if (step->kind == RigExecBakedStepKind::Derived) {
        const auto &[chainIndex, derivedIndex] =
            B.derivedIndex[size_t(step->object)];
        RigExecBakedProgramImpl::GeomChain &chain =
            B.chains[size_t(chainIndex)];
        RigExecBakedProgramImpl::GeomChain::Derived &derived =
            chain.derived[size_t(derivedIndex)];
        if (!chain.haveBase || !derived.haveBase) {
            return;
        }
        RigExecBakedProgramImpl::GeomRevision &revision = derived.revision;
        FoldInfluences(B, &revision);
        RigExecMoverParameters parameters = AssembleRevision(
            B, &revision, chain.result.cdata(), chain.result.size(), time,
            step);
        const RigExecMoverStatus status =
            RigExecStatusForParameters(parameters, revision.moverPath);
        step->counters.revisionsBuilt = 1;
        // The 26k-point input is remembered by HANDLE, not by copy.
        //
        // A derived revision's auxPoints IS the chain's own published
        // buffer, and nothing else in the packet is that size -- so the run
        // remembers the packet with that one field emptied and the points
        // beside it as the VtArray the chain published, which costs a
        // refcount. `chain.result != lastAuxPoints` is then the same test
        // `parameters != lastParameters` performed over the same values:
        // VtArray's operator== is `IsIdentical(other) || (shape equal &&
        // std::equal(...))`, so it is the elementwise walk with the identity
        // case taken first. The identity case is the rarer one here, because
        // the status sweep publishes into a double buffer and so hands out a
        // different array whenever it runs; what this removes for certain is
        // the pass that was never a comparison at all -- copying 315KB into
        // lastParameters every time the extent was recomputed, for a
        // bounding box that reads the points once. Measured on the biped:
        // the Derived step from 95-107us to 74-76us.
        std::vector<GfVec3f> aux;
        aux.swap(parameters.auxPoints);
        const bool moved = chain.result != revision.lastAuxPoints ||
                           parameters != revision.lastParameters;
        parameters.auxPoints.swap(aux);
        if (derived.baseDirty || !revision.ran || moved ||
            status != revision.lastStatus) {
            step->counters.revisionsExecuted = 1;
            // The derived target's own authored array, which is what the
            // recomputation writes over: two vectors for an extent, the
            // whole mesh for normals. Built once and re-assigned on the
            // failure arm rather than copied into a `preceding` that exists
            // only to be copied again.
            std::vector<GfVec3f> values(derived.lastBase.begin(),
                                        derived.lastBase.end());
            // The same function the revision node calls, and not a second
            // arrangement of the same rules: the kind check, the empty and
            // size-2 guards, the derived property keeping its authored
            // cardinality (an in-place write cannot resize it) and the
            // envelope folded into the recomputation's own arithmetic all
            // live in RigExecApplyDerivedKernel. The status is the one half
            // the kernel does not own, because the packet is what it is told
            // about and the status is what the graph decided.
            const bool applied =
                status.AllowsApply() &&
                RigExecRunRevisionKernel(revision.op, parameters, &values,
                                         /*controlFrames=*/nullptr);
            revision.resultStatus = status.state;
            if (!applied) {
                values.assign(derived.lastBase.begin(),
                              derived.lastBase.end());
                if (status.AllowsApply()) {
                    revision.resultStatus = _tokens->moverFailed;
                }
            }
            revision.output = std::move(values);
            // The packet WITHOUT its points, and the points beside it. Both
            // are the run's, and both are replaced only where the revision
            // executed -- the comparison above is against the last packet
            // that produced an answer, not against last frame's inputs.
            parameters.auxPoints.clear();
            parameters.auxPoints.shrink_to_fit();
            revision.lastParameters = std::move(parameters);
            revision.lastAuxPoints = chain.result;
            revision.lastStatus = status;
            revision.ran = true;
        }
        if (revision.resultStatus == "moverFailed") {
            step->diagnostics.push_back(
                "MoverFailed " + derived.target.GetString() +
                ": derived geometry input/cardinality validation failed");
        }
        derived.spare.resize(revision.output.size());
        std::copy(revision.output.begin(), revision.output.end(),
                  derived.spare.data());
        derived.result.swap(derived.spare);
        derived.haveResult = true;
        step->counters.chainsBuilt = 1;
        return;
    }

    // A ChainStatus step is about a CHAIN and every other one about a
    // revision, so the two payloads index different tables.
    if (step->kind == RigExecBakedStepKind::ChainStatus) {
        RigExecBakedProgramImpl::GeomChain &chain =
            B.chains[size_t(step->object)];
        if (!chain.haveBase) {
            return;
        }
        // The per-chain status sweep, over the status each revision LAST
        // published -- which outlives an evaluation it did not take part in,
        // and so does its diagnostic.
        for (const RigExecBakedProgramImpl::GeomRevision &revision :
                 chain.revisions) {
            if (revision.resultStatus == "moverFailed") {
                step->diagnostics.push_back(
                    "MoverFailed " + revision.moverPath.GetString() +
                    ": execution rejected its inputs; revision passed "
                    "through");
            }
        }
        const GfVec3f *points = chain.lastBase.cdata();
        size_t count = chain.lastBase.size();
        if (!chain.revisions.empty()) {
            PointsAfter(chain, chain.revisions.size() - 1, &points, &count);
        }
        // Double-buffered: publication is a refcount bump for the consumer
        // and the array one may still hold from last frame is never the one
        // being written.
        chain.spare.resize(count);
        std::copy(points, points + count, chain.spare.data());
        chain.result.swap(chain.spare);
        chain.haveResult = true;
        // `final` costs nothing extra: this is the value the chain
        // publishes.
        step->snapshots.RecordFinal(chain.target, VtValue(chain.result));
        step->counters.chainsBuilt = 1;
        return;
    }

    const auto &[chainIndex, revisionIndex] =
        B.revisionIndex[size_t(step->object)];
    RigExecBakedProgramImpl::GeomChain &chain = B.chains[size_t(chainIndex)];
    if (!chain.haveBase) {
        return;
    }
    RigExecBakedProgramImpl::GeomRevision &revision =
        chain.revisions[size_t(revisionIndex)];
    const bool skin = revision.op == RigExecRevisionOp::Skin;
    // The chain's sticky dirty bit as of the PRECEDING revision: once a
    // revision executed, every later one does, and the chain's own base is
    // where it starts.
    const bool chainDirty =
        revisionIndex == 0
            ? chain.baseDirty
            : chain.revisions[size_t(revisionIndex) - 1].executed;
    switch (step->kind) {
    case RigExecBakedStepKind::InfluenceFold: {
        revision.influencesChanged = FoldInfluences(B, &revision);
        // The finite/affine check the dynamic path's assembler makes over the
        // same matrices. It is the fold's because the packet no longer
        // carries them, and the fuse ANDs it in where the assembler's answer
        // would have landed: a revision with one bad influence passes its
        // preceding value through and reports MoverFailed.
        revision.influencesValid =
            !skin || RigExecSkinTransformsAreUsable(revision.influences.data(),
                                                    revision.influences.size());
        // Both forms of the table, whether or not the revision is chunked:
        // a chunked one's chunks keep their own, but the fuse's whole-array
        // fallback reads these and cannot write them. O(influences) of pure
        // per-matrix arithmetic, which is the size of this step anyway.
        if (skin && revision.influencesValid) {
            FoldTransformForms(&revision);
        }
        return;
    }

    case RigExecBakedStepKind::RevisionStatic: {
        // rigExec:samplePhase = "current": the field is measured against the
        // points AS THEY STAND HERE, not the authored base, so the volume
        // grabs whatever is inside it right now.
        //
        // This one copies the ORACLE and not exec (see bakedWeights.cpp):
        // the dynamic path cannot get it from exec either -- a revision
        // node's parameters are a VDF constant, so nothing in the packet can
        // depend on a value the graph has not computed yet -- and it patches
        // the tapped packet with what _ResolveWeights measured. So the patch
        // starts from the SAME packet and touches the SAME fields:
        // representation, values, indices, defaultWeight and valid, and not
        // rangePolicy, which the dynamic path leaves at whatever the tapped
        // packet had. A freshly constructed packet would differ there, and
        // the difference would move moverGraphRevisionsExecuted through the
        // parameters comparison below rather than move a point.
        if (revision.weightCurrentPhase && revision.weightObject >= 0) {
            revision.currentPhasePacket =
                B.weightPackets[size_t(revision.weightObject)];
            const GfVec3f *entering = nullptr;
            size_t enteringCount = 0;
            PointsBefore(chain, size_t(revisionIndex), &entering,
                         &enteringCount);
            const std::vector<GfVec3f> current(entering,
                                               entering + enteringCount);
            std::vector<float> field;
            std::string error;
            if (B.resolveWeights(
                    B.weightObjects[size_t(revision.weightObject)].path,
                    current.size(), time, &field, &error, &current)) {
                revision.currentPhasePacket.representation =
                    _tokens->denseRepresentation;
                revision.currentPhasePacket.values = std::move(field);
                revision.currentPhasePacket.indices.clear();
                revision.currentPhasePacket.defaultWeight = 0.0f;
                revision.currentPhasePacket.valid = true;
            } else {
                // An invalid packet is the kernel's atomic MoverFailed
                // pass-through, which is the right answer here: publishing
                // the reference-phase field instead would silently be a
                // different deformation.
                revision.currentPhasePacket = RigExecWeightPacket();
                step->diagnostics.push_back("current-phase weight failed: " +
                                            error);
            }
        }
        revision.parameters =
            AssembleRevision(B, &revision, chain.lastBase.cdata(),
                             chain.lastBase.size(), time, step);
        // The status of the PACKET. For a skin revision that is half of the
        // answer -- the packet carries identities where the matrices would
        // be -- and the fold's `influencesValid` is the other half; the fuse
        // is where the two meet and where `moverFailed` is published.
        revision.status =
            RigExecStatusForParameters(revision.parameters,
                                       revision.moverPath);
        step->counters.revisionsBuilt = 1;
        // Whether the revision has to run at all, by VALUE and never by
        // dirtiness: a control dragged back to where it started leaves an
        // identical packet, and the VdfNetwork does not re-execute for one.
        // This is the half of the decision the packet and the status carry;
        // the chain's sticky bit and the influence table's own comparison are
        // ORed in by the fuse, which is the first step that has all three.
        //
        // The scalar the fuse's one diagnostic is about, read here because
        // this step always runs and the fuse may not. Compared like every
        // other source value: two different out-of-range weights can leave
        // the packet identical, and the line the fuse emits is about the
        // weight rather than about the packet.
        revision.defaultWeight = 1.0f;
        if (const UsdAttribute attribute =
                revision.moverPrim.GetAttribute(_tokens->defaultWeight)) {
            R.GetAttribute(attribute, time, &revision.defaultWeight);
        }
        revision.staticDirty = !revision.ran ||
                               revision.parameters != revision.lastParameters ||
                               revision.status != revision.lastStatus ||
                               revision.defaultWeight !=
                                   revision.lastDefaultWeight;
        revision.lastDefaultWeight = revision.defaultWeight;
        // The field an authoring tool paints as an influence overlay, taken
        // from the packet the mover is about to consume so that what a
        // rigger sees is what deformed the geometry. Published from HERE
        // because this is where the dynamic path publishes it -- beside the
        // assemble, whether or not the revision then executes -- and the
        // epilogue drains it in chain and revision order, which is what
        // makes two revisions sharing one weight object last-writer-wins the
        // way the dynamic walk's per-chain merge does.
        revision.weightFieldPublished = false;
        if (revision.weightObject >= 0 && B.publishWeightFields) {
            const RigExecWeightPacket &packet =
                revision.weightCurrentPhase
                    ? revision.currentPhasePacket
                    : B.weightPackets[size_t(revision.weightObject)];
            if (packet.valid) {
                const size_t logicalCount =
                    revision.weightOperationDomain ? size_t(1)
                                                   : chain.lastBase.size();
                // One scatter rather than a search per point: Resolve()
                // binary-searches a sparse packet's indices, which for a
                // face cluster on a body is tens of thousands of searches
                // to find a few hundred weights. An unresolvable packet
                // publishes what the per-point loop did: zero wherever
                // Resolve() answered out of range.
                if (!packet.ResolveAll(logicalCount, &revision.weightField)) {
                    revision.weightField.assign(logicalCount, 0.0f);
                    for (size_t i = 0; i < logicalCount; ++i) {
                        const float w = packet.Resolve(i, logicalCount);
                        revision.weightField[i] = w < 0.0f ? 0.0f : w;
                    }
                }
                revision.weightFieldPublished = true;
            }
        }
        // Every revision of a chain is applied to the same number of points:
        // a kernel that resized its output failed the application, so no
        // buffer the chain ever reads holds a different count. Sizing the
        // output here rather than in a chunk is what lets the chunks write
        // disjoint ranges of it without one of them owning its length.
        const size_t count = chain.lastBase.size();
        if (revision.output.size() != count) {
            revision.output.resize(count);
            // A resized buffer holds no answers, so every chunk of it has to
            // produce one again.
            revision.staticDirty = true;
        }
        revision.precedingCount = count;
        // The whole-array decisions that are the skin kernel's and are made
        // here because a range may not repeat them: the layout validation
        // against the points that arrived, and the envelope, which resolves
        // atomically at the FULL count or not at all.
        revision.layoutUsable = false;
        revision.envelopeOk = true;
        revision.fullStrength = true;
        revision.partitionStale = false;
        if (!skin) {
            return;
        }
        revision.layoutUsable =
            RigExecSkinLayoutIsUsable(revision.parameters, count);
        revision.fullStrength =
            RigExecEnvelopeIsFullStrength(revision.parameters.weights);
        if (!revision.fullStrength) {
            revision.envelopeOk = revision.parameters.weights.ResolveAll(
                count, &revision.envelope);
        }
        if (revision.chunked) {
            // Whether the keys still describe the vertices. The handle is
            // the identity the layout cache preserves for a binding that did
            // not move, and the prologue re-cut against it; anything else
            // here means the packet is reading arrays the partition never
            // saw, and the fuse runs the revision whole instead.
            const RigExecSkinTopology *const topology =
                revision.parameters.skinTopology.get();
            const size_t indexCount =
                topology ? topology->indices.size()
                         : revision.parameters.skinIndices.size();
            const int elementSize = topology
                                        ? topology->elementSize
                                        : revision.parameters.skinElementSize;
            revision.partitionStale =
                // No resolved layout at all is a layout the keys cannot be
                // checked against: the packet is reading the mover's arrays
                // per frame, and nothing here can say they are the arrays
                // the cut was made from.
                !revision.parameters.skinTopology ||
                revision.parameters.skinTopology !=
                    revision.partitionTopology ||
                indexCount != revision.partitionIndexCount ||
                elementSize != revision.partitionElementSize ||
                count != revision.partitionPointCount;
        }
        return;
    }

    case RigExecBakedStepKind::RevisionChunk: {
        RigExecBakedProgramImpl::GeomChunk &chunk =
            revision.chunks[size_t(step->part)];
        const GfVec3f *points = nullptr;
        size_t count = 0;
        PointsBefore(chain, size_t(revisionIndex), &points, &count);
        const bool sized = count == revision.precedingCount &&
                           revision.output.size() == count;

        if (!revision.chunked) {
            // One chunk is the whole array, so there is nothing to speculate
            // about: it knows everything the fuse knows.
            chunk.ok = false;
            const bool executed = chainDirty || revision.staticDirty ||
                                  (skin && revision.influencesChanged);
            if (!executed || !revision.status.AllowsApply()) {
                return;
            }
            if (!skin) {
                // The revision's OWN buffer, seeded from the preceding one:
                // there is no `scratch = current` and no
                // `revision.output = current` afterwards -- the fuse decides
                // which buffer the chain's running value is in rather than
                // copying one into another.
                revision.output.assign(points, points + count);
                // Not a second dispatch that mirrors _RevisionNode::Compute
                // -- the same function the node calls. The packet check, the
                // full-strength fast path, the kernel and the "apply once"
                // blend all live in RigExecRunRevisionKernel, so an
                // operation cannot mean one thing here and another there.
                // The control frames are handed over for every operation
                // and written by one, exactly as the revision node hands its
                // own buffer to the same kernel whatever it is running: the
                // adjuster's second output is the only thing that touches
                // them, and it keeps its last answer through a run it sat
                // out because the buffer outlives the run.
                chunk.ok = RigExecRunRevisionKernel(
                    revision.op, revision.parameters, &revision.output,
                    &revision.controlFrames);
                return;
            }
            if (!revision.parameters.valid ||
                revision.parameters.kind != "skin" ||
                !revision.layoutUsable || !revision.envelopeOk ||
                !revision.influencesValid || !sized) {
                return;
            }
            chunk.ok = SkinRange(&revision, points,
                                 WholeTransformsView(&revision), 0, count,
                                 /*whole=*/true);
            return;
        }

        // Chunked, and therefore SPECULATIVE: it has not waited for the
        // fold, so it cannot know whether the revision will apply at all --
        // only that its own vertices, its own joints and the packet say what
        // its range of the output buffer should hold. The fuse discards the
        // work if the revision turns out not to apply.
        chunk.keyChanged = false;
        if (!revision.status.AllowsApply() || !revision.parameters.valid ||
            revision.parameters.kind != "skin" || !revision.layoutUsable ||
            !revision.envelopeOk || revision.partitionStale || !sized) {
            chunk.ok = false;
            return;
        }
        GatherChunkTransforms(B, revision, &chunk);
        if (!(chainDirty || revision.staticDirty || chunk.keyChanged) &&
            chunk.ok) {
            // Its own joints stood still over points that stood still and a
            // packet that stood still, so its range of the buffer already
            // holds this run's answer -- and its `ok` still describes it.
            // Another chunk's joints moving makes the REVISION execute; it
            // does not make this range's vertices land anywhere else.
            //
            // `ok` is part of the gate rather than a consequence of it: a
            // chunk whose last answer was a failure, or one the partition
            // reset without the packet moving, has nothing in its range to
            // keep, and reading that off its own flag keeps the invariant
            // local to this step.
            return;
        }
        chunk.ok = SkinRange(&revision, points, ChunkTransformsView(chunk),
                             size_t(chunk.begin), size_t(chunk.end),
                             /*whole=*/false);
        return;
    }

    case RigExecBakedStepKind::RevisionFuse: {
        // Where the dynamic path's assembler would have failed the packet:
        // for a skin revision the matrices are not in it, so "the packet is
        // valid" is the packet's own answer AND the fold's.
        const bool packetValid =
            revision.parameters.valid &&
            (!skin || revision.influencesValid);
        if (revision.parameters.enabled && !packetValid &&
            revision.weightObject < 0) {
            // Read by RevisionStatic, which always runs: a step body that
            // went to the stage for a value would be a step outside its own
            // declarations, and the cone could not tell when it moved.
            const float scalar = revision.defaultWeight;
            if (!std::isfinite(scalar) || scalar < 0.0f || scalar > 1.0f) {
                step->diagnostics.push_back(
                    "MoverFailed " + revision.moverPath.GetString() +
                    ": inputs:defaultWeight must be finite and in "
                    "[0, 1]; revision passed through");
            }
        } else if (revision.parameters.enabled && !packetValid &&
                   revision.weightObject >= 0 &&
                   !(revision.weightCurrentPhase
                         ? revision.currentPhasePacket
                         : B.weightPackets[size_t(revision.weightObject)])
                        .valid) {
            // The other half of the same sentence, and the reason the arms
            // are exclusive: with a weight object bound, the assembler never
            // reads inputs:defaultWeight, so a rig whose scalar is out of
            // range and whose object is fine must stay silent.
            step->diagnostics.push_back(
                "MoverFailed " + revision.moverPath.GetString() +
                ": rigExec:weightObject produced an invalid common "
                "envelope; revision passed through");
        }
        revision.executed = chainDirty || revision.staticDirty ||
                            (skin && revision.influencesChanged);
        step->counters.revisionsExecuted = revision.executed ? 1 : 0;
        if (revision.executed) {
            revision.resultStatus = revision.status.state;
            bool applied = packetValid && revision.status.AllowsApply();
            if (applied && skin && revision.partitionStale) {
                applied = FuseWholeRevision(chain, &revision,
                                            size_t(revisionIndex));
            } else if (applied) {
                for (const RigExecBakedProgramImpl::GeomChunk &chunk :
                         revision.chunks) {
                    if (!chunk.ok) {
                        applied = false;
                        break;
                    }
                }
            }
            if (applied) {
                revision.currentSource = int(revisionIndex);
            } else {
                // Nothing was applied, so the chain's running value stays
                // where it was -- an indirection, not a copy. A chunk that
                // ran and produced points is discarded with it.
                revision.currentSource =
                    revisionIndex == 0
                        ? -1
                        : chain.revisions[size_t(revisionIndex) - 1]
                              .currentSource;
                if (revision.status.AllowsApply()) {
                    revision.resultStatus = _tokens->moverFailed;
                }
            }
            revision.lastParameters = revision.parameters;
            revision.lastStatus = revision.status;
            revision.ran = true;
        }
        // Snapshot only where a phased read named this revision, which is the
        // whole cost of the feature for a rig that uses it and nothing at all
        // for one that does not.
        if (revision.snapshotAfter) {
            const GfVec3f *points = nullptr;
            size_t count = 0;
            PointsAfter(chain, size_t(revisionIndex), &points, &count);
            step->snapshots.Record(
                chain.target, revision.moverPath,
                VtValue(VtVec3fArray(points, points + count)));
        }
        return;
    }

    default:
        return;
    }
}

// ---------------------------------------------------------------------------
// The report.
// ---------------------------------------------------------------------------

namespace {

/// Two decimals, so a ratio in the report lines up with the scheduler's.
std::string
Fixed2(double value)
{
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%.2f", value);
    return buffer;
}

/// Whether \p step is one of the four steps that make up revision \p id.
bool
IsStepOfRevision(const RigExecBakedStep &step, int id)
{
    if (step.object != id) {
        return false;
    }
    switch (step.kind) {
    case RigExecBakedStepKind::InfluenceFold:
    case RigExecBakedStepKind::RevisionStatic:
    case RigExecBakedStepKind::RevisionChunk:
    case RigExecBakedStepKind::RevisionFuse:
        return true;
    default:
        return false;
    }
}

/// The cost of revision \p id run one step at a time, and the cost of its
/// longest dependency chain -- the floor the chunks cannot go below however
/// many threads there are. The longest path is taken over the revision's own
/// steps only: an edge leaving the revision is the rest of the frame's
/// business, and what this number answers is "how much of this skin is
/// width and how much is depth".
void
SkinCosts(const RigExecBakedProgramImpl &B, int id, double *serial,
          double *criticalPath)
{
    *serial = 0.0;
    *criticalPath = 0.0;
    // Steps are in topological order, so one forward pass settles the
    // longest path into each of them.
    std::vector<double> through(B.steps.size(), 0.0);
    for (size_t index = 0; index < B.steps.size(); ++index) {
        const RigExecBakedStep &step = B.steps[index];
        if (!IsStepOfRevision(step, id)) {
            continue;
        }
        double before = 0.0;
        for (const int pred : step.preds) {
            if (IsStepOfRevision(B.steps[size_t(pred)], id)) {
                before = std::max(before, through[size_t(pred)]);
            }
        }
        through[index] = before + step.cost;
        *serial += step.cost;
        *criticalPath = std::max(*criticalPath, through[index]);
    }
}

}  // namespace

std::string
RigExecBakedGeometryReport(const RigExecBakedProgramImpl &B)
{
    std::string out;
    char line[256];
    const ProviderLevels levels = GatherProviderLevels(B);
    for (size_t c = 0; c < B.chains.size(); ++c) {
        const RigExecBakedProgramImpl::GeomChain &chain = B.chains[c];
        for (size_t r = 0; r < chain.revisions.size(); ++r) {
            const RigExecBakedProgramImpl::GeomRevision &revision =
                chain.revisions[r];
            if (revision.op != RigExecRevisionOp::Skin) {
                continue;
            }
            const int id = B.chainRevisionBegin[c] + int(r);
            const size_t influences = revision.influenceSlots.size();
            const size_t chunks = revision.chunks.size();
            size_t vertexMin = size_t(-1), vertexMax = 0, vertexSum = 0;
            size_t keyMin = size_t(-1), keyMax = 0, keySum = 0, wide = 0;
            for (const RigExecBakedProgramImpl::GeomChunk &chunk :
                     revision.chunks) {
                // An unpartitioned revision is one chunk over everything,
                // which is every influence and every vertex it has.
                const size_t vertices =
                    revision.chunked ? size_t(chunk.end - chunk.begin)
                                     : revision.partitionPointCount;
                const size_t key =
                    revision.chunked ? chunk.key.size() : influences;
                vertexMin = std::min(vertexMin, vertices);
                vertexMax = std::max(vertexMax, vertices);
                vertexSum += vertices;
                keyMin = std::min(keyMin, key);
                keyMax = std::max(keyMax, key);
                keySum += key;
                if (influences && key * 2 >= influences) {
                    ++wide;
                }
            }
            std::snprintf(
                line, sizeof(line),
                "  %s: %zu chunk(s), %zu vertex(es), %zu influence(s)%s\n",
                revision.moverPath.GetString().c_str(), chunks, vertexSum,
                influences, revision.chunked ? "" : " [not partitioned]");
            out += line;
            std::snprintf(line, sizeof(line),
                          "    vertices/chunk min %zu mean %.1f max %zu\n",
                          vertexMin == size_t(-1) ? 0 : vertexMin,
                          chunks ? double(vertexSum) / double(chunks) : 0.0,
                          vertexMax);
            out += line;
            std::snprintf(
                line, sizeof(line),
                "    |key| min %zu mean %.1f max %zu; %zu/%zu chunk(s) reach "
                "half the influences\n",
                keyMin == size_t(-1) ? 0 : keyMin,
                chunks ? double(keySum) / double(chunks) : 0.0, keyMax, wide,
                chunks);
            out += line;
            // What the partition bought, in the scheduler's own units: the
            // revision's serial cost against its longest dependency chain.
            double skinSerial = 0.0, skinPath = 0.0;
            SkinCosts(B, id, &skinSerial, &skinPath);
            std::snprintf(line, sizeof(line),
                          "    skin serial %.2fus critical path %.2fus%s\n",
                          skinSerial, skinPath,
                          skinPath > 0.0
                              ? (" (" + Fixed2(skinSerial / skinPath) +
                                 "x)").c_str()
                              : "");
            out += line;
            // A chunk is ready when the last of ITS influences is, so its
            // ready level against the frame's deepest level is the whole
            // speculation in one number: a chunk ready far below the maximum
            // is one the arena can start long before the rig is posed.
            int readyMin = -1, readyMax = -1;
            for (size_t k = 0; k < chunks; ++k) {
                const RigExecBakedProgramImpl::GeomChunk &chunk =
                    revision.chunks[k];
                const int ready = ChunkReadyLevel(revision, chunk,
                                                  revision.chunked, levels);
                readyMin = readyMin < 0 ? ready : std::min(readyMin, ready);
                readyMax = std::max(readyMax, ready);
                std::snprintf(line, sizeof(line),
                              "    [%zu] %d..%d ready level %d/%d key", k,
                              chunk.begin, chunk.end, ready, levels.maxLevel);
                out += line;
                if (!revision.chunked) {
                    out += " (every influence)";
                }
                for (const int position : chunk.key) {
                    out += " " + std::to_string(position);
                }
                out += "\n";
            }
            std::snprintf(line, sizeof(line),
                          "    ready level min %d max %d of %d\n",
                          readyMin < 0 ? 0 : readyMin,
                          readyMax < 0 ? 0 : readyMax, levels.maxLevel);
            out += line;
        }
    }
    return out;
}

void
RigExecBakedPublishGeometry(RigExecBakedProgramImpl *program,
                            UsdTimeCode time, RigExecRigPose *pose)
{
    RigExecBakedProgramImpl &B = *program;
    // The ladder from a curvenet's own prim up to the asset root, composed
    // at the evaluated time. Built LAZILY and at most once per frame: a rig
    // with no adjuster never constructs a UsdGeomXformCache, and one with
    // several composes each net's ladder through the same cache.
    //
    // Here rather than in a step because it reads the stage and writes the
    // pose, and both are the epilogue's business. The kernel's own output --
    // the adjusted frames in net-local space -- was produced in the region;
    // this only moves them into the space every rig control publishes in.
    std::unique_ptr<UsdGeomXformCache> xformCache;
    const UsdPrim assetRoot = B.stage->GetPrimAtPath(B.assetRootPath);
    const auto publishAdjuster =
        [&](RigExecBakedProgramImpl::GeomRevision &revision) {
        if (revision.op != RigExecRevisionOp::CurvenetAdjuster ||
            revision.resultStatus != "ok") {
            return;
        }
        if (!xformCache) {
            xformCache = std::make_unique<UsdGeomXformCache>(time);
        }
        const std::vector<SdfPath> paths = RigExecCurvenetAdjustmentPaths(
            B.stage->GetPrimAtPath(revision.moverPath));
        GfMatrix4d netToAsset(1.0);
        for (UsdPrim prim =
                 B.stage->GetPrimAtPath(revision.target.GetPrimPath());
             prim && !prim.IsPseudoRoot() && prim != assetRoot;
             prim = prim.GetParent()) {
            const UsdGeomXformable xform(prim);
            if (!xform) {
                continue;
            }
            GfMatrix4d local(1.0);
            bool reset = false;
            xform.GetLocalTransformation(&local, &reset, time);
            const auto driven = pose->providerXforms.find(prim.GetPath());
            if (driven != pose->providerXforms.end()) {
                local = driven->second;
            }
            netToAsset = netToAsset * local;
            if (reset) {
                netToAsset =
                    netToAsset * xformCache->GetLocalToWorldTransform(assetRoot)
                                     .GetInverse();
                break;
            }
        }
        // Retained for the bake beside the compose the publish below
        // consumes: the per-frame stage read the recorder cannot see.
        revision.lastAdjusterNetToAsset = netToAsset;
        // min(), and not a cardinality refusal: the walk publishes what both
        // sides have and says nothing about the rest.
        for (size_t j = 0;
             j < std::min(revision.controlFrames.size(), paths.size()); ++j) {
            pose->controlFrames[paths[j]] =
                RigExecFrameFromMatrix(revision.controlFrames[j] * netToAsset);
        }
    };
    // Chain by chain, in chain order, and inside a chain exactly where the
    // straight line put each line: every revision's assemble diagnostics
    // first, then the status sweep's, then the chain's points, then each
    // derived target's diagnostic and its points. Program order is that
    // order, so this is one pass over the steps.
    for (const RigExecBakedStep &step : B.steps) {
        if (!RigExecBakedIsGeometryStep(step.kind)) {
            continue;
        }
        for (const std::string &diagnostic : step.diagnostics) {
            pose->diagnostics.push_back(diagnostic);
        }
        if (step.kind == RigExecBakedStepKind::RevisionStatic) {
            const auto &[chainIndex, revisionIndex] =
                B.revisionIndex[size_t(step.object)];
            const RigExecBakedProgramImpl::GeomChain &chain =
                B.chains[size_t(chainIndex)];
            const RigExecBakedProgramImpl::GeomRevision &revision =
                chain.revisions[size_t(revisionIndex)];
            // `haveBase` for the same reason the two arms below test it: a
            // chain whose points stopped reading at this time drives
            // nothing, and the assemble that would have refreshed the field
            // returned before it ever looked at the packet. Its last answer
            // is then last FRAME's, and publishing that is the one shape of
            // staleness a value comparison cannot catch -- the dynamic path
            // publishes no field at all for such a chain.
            if (chain.haveBase && revision.weightFieldPublished) {
                RigExecResolvedWeightField &field =
                    pose->weightFields[
                        B.weightObjects[size_t(revision.weightObject)].path];
                field.target = revision.weightFieldTarget;
                field.weights = revision.weightField;
            }
        } else if (step.kind == RigExecBakedStepKind::ChainStatus) {
            RigExecBakedProgramImpl::GeomChain &chain =
                B.chains[size_t(step.object)];
            if (chain.haveBase) {
                // The adjusters of this chain, in revision order, where the
                // walk publishes them: inside the same per-revision sweep
                // that reports the chain's MoverFailed lines, and after the
                // pose half published the rig's own controls -- so an
                // adjustment path that collides with a RigExecControl path
                // wins, as it does there.
                for (RigExecBakedProgramImpl::GeomRevision &revision :
                         chain.revisions) {
                    publishAdjuster(revision);
                }
                pose->movedProperties[chain.target] = VtValue(chain.result);
            }
        } else if (step.kind == RigExecBakedStepKind::Derived) {
            const auto &[chainIndex, derivedIndex] =
                B.derivedIndex[size_t(step.object)];
            const RigExecBakedProgramImpl::GeomChain &chain =
                B.chains[size_t(chainIndex)];
            const RigExecBakedProgramImpl::GeomChain::Derived &derived =
                chain.derived[size_t(derivedIndex)];
            if (chain.haveBase && derived.haveBase) {
                pose->movedProperties[derived.target] =
                    VtValue(derived.result);
            }
        }
    }
}

}  // namespace rigExec
