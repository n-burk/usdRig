// The baked program's geometry half: the chain/revision bake and the frame
// path that runs those revisions and the derived maintenance behind them.
// Split out of bakedProgram.cpp for the reason bakedPose.cpp was: a domain's
// bake and its frame path have to agree about what was captured and what is
// re-read. The evaluator is not visible here -- the compiled chains arrive
// restated as RigExecBakedChainSpec, and the caches the frame path shares
// with the dynamic walk were captured into RigExecBakedProgramImpl at Build.
#include "bakedProgramImpl.h"
#include "bakedSchedule.h"

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
#include "pxr/base/tf/diagnostic.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/staticTokens.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/vt/value.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/attributeQuery.h"
#include "pxr/usd/usd/stage.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iterator>
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

void
RigExecBakedGeometryTouchTokens()
{
    (void)_tokens.Get();
}

// Bake.

namespace {

/// Binds every phased point read of the program to the chain versions the
/// dynamic walk answers it from (RigExecBakedPointsBinding): its phased-read
/// store, or for `preceding` on the reader's own chain the points entering
/// the reader.
///
/// "Before the reader" is program order, which RigExecBakedBuildGeometrySteps
/// emits chain by chain in `chains` order: every step of an earlier chain,
/// and on the reader's own chain the fuses of the revisions before it -- or,
/// for a derived reader, every fuse and the chain's status step. The walk
/// records a revision only where `snapshotAfter` is set and a chain's final
/// after its status sweep, and both only when the chain read a base.
void
BindPointReads(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    std::map<SdfPath, int> chainOf;
    for (size_t c = 0; c < B.chains.size(); ++c) {
        chainOf.emplace(B.chains[c].target, int(c));
    }
    int nextId = 0;
    // `before` revisions of the reader's own chain precede it, and
    // `afterStatus` says its status step does too.
    const auto bind = [&](const SdfPath &input, const RigExecReadPhase &phase,
                          const SdfPath &reader, size_t readerChain,
                          size_t before, bool afterStatus) {
        RigExecBakedPointsBinding out;
        out.input = input;
        out.phase = phase;
        out.id = nextId++;
        const auto found = chainOf.find(input);
        if (found == chainOf.end()) {
            return out;
        }
        const size_t c = size_t(found->second);
        const std::vector<RigExecBakedProgramImpl::GeomRevision> &revisions =
            B.chains[c].revisions;
        const size_t recorded = c < readerChain    ? revisions.size()
                                : c == readerChain ? before
                                                   : 0;
        switch (phase.kind) {
        case RigExecReadPhaseKind::Base:
            break;
        case RigExecReadPhaseKind::Final:
            if (c < readerChain || (c == readerChain && afterStatus)) {
                out.finalRead = true;
                out.candidates.push_back({int(c), int(revisions.size())});
            }
            break;
        case RigExecReadPhaseKind::Preceding: {
            if (c == readerChain && !afterStatus) {
                // The reader's own chain: the points entering it, version
                // `before`. A first revision reads the resolved input, as
                // the walk does.
                if (before > 0) {
                    out.candidates.push_back({int(c), int(before)});
                }
                break;
            }
            // Another chain the reader writes: the record before the
            // reader's own, among the records made so far; none when the
            // reader's own is not among them yet.
            int previous = -1;
            for (size_t r = 0; r < recorded; ++r) {
                if (!revisions[r].snapshotAfter) {
                    continue;
                }
                if (revisions[r].moverPath == reader) {
                    if (previous >= 0) {
                        out.candidates.push_back({int(c), previous + 1});
                    }
                    break;
                }
                previous = int(r);
            }
            break;
        }
        case RigExecReadPhaseKind::AtPrim:
            for (size_t r = recorded; r-- > 0;) {
                const SdfPath &mover = revisions[r].moverPath;
                if (revisions[r].snapshotAfter &&
                    (mover == phase.prim || mover.HasPrefix(phase.prim))) {
                    out.candidates.push_back({int(c), int(r) + 1});
                }
            }
            break;
        }
        return out;
    };
    const auto bindRevision =
        [&](RigExecBakedProgramImpl::GeomRevision *revision, size_t chain,
            size_t before, bool afterStatus) {
            revision->pointBindings.clear();
            for (const auto &[input, phase] : revision->binding.phases) {
                RigExecBakedPointsBinding bound =
                    bind(input, phase, revision->moverPath, chain, before,
                         afterStatus);
                bound.diagnoseMiss =
                    phase.kind != RigExecReadPhaseKind::Preceding;
                revision->pointBindings.push_back(std::move(bound));
            }
            for (RigExecBakedProgramImpl::GeomBlendChannel &channel :
                     revision->blendChannels) {
                for (auto &sample : channel.samples) {
                    sample.pointBinding = RigExecBakedPointsBinding();
                    if (sample.phase.IsBase() || !sample.blendShape.IsEmpty()) {
                        continue;
                    }
                    sample.pointBinding =
                        bind(sample.pointsPath, sample.phase,
                             revision->moverPath, chain, before, afterStatus);
                }
            }
        };
    for (size_t c = 0; c < B.chains.size(); ++c) {
        RigExecBakedProgramImpl::GeomChain &chain = B.chains[c];
        for (size_t r = 0; r < chain.revisions.size(); ++r) {
            bindRevision(&chain.revisions[r], c, r, false);
        }
        for (RigExecBakedProgramImpl::GeomChain::Derived &derived :
                 chain.derived) {
            bindRevision(&derived.revision, c, chain.revisions.size(), true);
        }
    }
    B.pointBindingCount = nextId;
}

}  // namespace

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
        out.posedPoints = r.transformPosedPoints;
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
        if (!out.moverPrim) {
            refuse("mover prim is missing", r.moverPath);
        }
        // The packet is assembled live from this prim every frame through
        // the generation's resolved inputs: nothing of it is captured, so a
        // value edit needs no rebuild and an override places itself. The
        // layout arrays are still NAMED, because bakeability judged them --
        // connecting one is a resync, and the judgement was that none is
        // connected. They do not belong in `rebuild` even though the layout
        // is no longer read per frame: the SkinTopology op holds it, and its
        // layout leaves are filed under these paths, so a weight-paint edit
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
        // A phased input reads the chain version it is bound to when a
        // candidate answers and the resolved input -- the stage -- otherwise,
        // so the path is one the bake asked about either way.
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
                    sample.activationPath = activationPath;
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
        // rigExec:space: the rig's carry. Refused on a miss for the same
        // reason the measuring space is -- a carry that quietly resolved to
        // nothing looks exactly like the bug it was named to fix.
        if (!r.binding.carrySpace.IsEmpty()) {
            out.carrySpaceSlot = slotOf(r.binding.carrySpace);
            if (out.carrySpaceSlot < 0) {
                refuse("mover carry space is not a pose provider",
                       r.binding.carrySpace);
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
        // The packet assembly's reads as path leaves, sampled in the
        // prologue: the assembler's, for an operation the leaves assemble,
        // and the blend gather's, which reads the generation's resolved
        // inputs and not the phase overlay.
        RigExecRevisionLeafDecl &decl = out.leaves.decl;
        RigExecDeclareRevisionLeaves(out.op, out.moverPath, out.binding,
                                     &decl);
        // A plugin mover is handed every provider value; the prologue holds
        // them only for one that binds none the region computes (no phase,
        // weight object, transform, carry, influence, driver frames,
        // constraint delta or blend channel). Any other keeps its plugin
        // call in RevisionStatic, through the stage assembler.
        if (out.op == RigExecRevisionOp::External &&
            !(out.binding.phases.empty() && out.weightObject < 0 &&
              out.binding.transform.IsEmpty() &&
              out.binding.carrySpace.IsEmpty() &&
              out.binding.influences.empty() &&
              out.binding.driverFrames.IsEmpty() &&
              out.constraintDelta < 0 && out.blendChannels.empty())) {
            decl = RigExecRevisionLeafDecl();
        }
        if (decl.assembles) {
            using Type = RigExecRevisionLeafType;
            using Flavour = RigExecRevisionLeafFlavour;
            const RigExecRevisionLeafTime at =
                RigExecRevisionLeafTime::AtTime;
            for (RigExecBakedProgramImpl::GeomBlendChannel &channel :
                     out.blendChannels) {
                channel.weightHeldLeaf =
                    decl.Add({channel.weightPath, Type::Bool, at,
                              Flavour::Present, VtValue(false)});
                channel.weightLeaf = decl.Add(
                    {channel.weightPath, Type::Float, at,
                     Flavour::ResolvedOnly,
                     VtValue(RigExecBlendChannel().weight)});
                for (auto &sample : channel.samples) {
                    sample.activationLeaf = decl.Add(
                        {sample.activationPath, Type::Float, at,
                         Flavour::ResolvedOnly,
                         VtValue(RigExecBlendSampleData().activation)});
                    if (sample.blendShape.IsEmpty()) {
                        sample.pointsLeaf =
                            decl.Add({sample.pointsPath, Type::Vec3fArray, at,
                                      Flavour::ResolvedOnly,
                                      VtValue(VtVec3fArray())});
                    }
                }
            }
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
            // RevisionStatic's own defaultWeight read, which the fuse's
            // diagnostic is about: the generation's resolved inputs alone.
            RigExecBakedProgramImpl::GeomRevision &baked =
                chain.revisions.back();
            baked.defaultWeightLeaf = baked.leaves.decl.Add(
                {baked.moverPath.AppendProperty(_tokens->defaultWeight),
                 RigExecRevisionLeafType::Float,
                 RigExecRevisionLeafTime::AtTime,
                 RigExecRevisionLeafFlavour::ResolvedOnly, VtValue(1.0f)});
            RigExecBakedBindPathLeaves(B.stage, &baked.leaves);
        }
        for (const RigExecBakedRevisionSpec &derived : spec.derived) {
            RigExecBakedProgramImpl::GeomChain::Derived d;
            d.target = derived.target;
            d.matrixTarget = RigExecIsDerivedMatrixOp(derived.op);
            B.prims.insert(derived.target.GetPrimPath());
            B.named.insert(derived.target);
            if (d.matrixTarget) {
                // A projector's primvar is published, never authored, so
                // there is no base to read; its inputs are the chain's.
            } else if (const UsdAttribute a =
                    B.stage->GetAttributeAtPath(derived.target)) {
                d.baseQuery = UsdAttributeQuery(a);
            } else {
                refuse("derived target has no attribute", derived.target);
            }
            d.revision = bakeRevision(derived);
            RigExecBakedBindPathLeaves(B.stage, &d.revision.leaves);
            chain.derived.push_back(std::move(d));
        }
        B.chains.push_back(std::move(chain));
    }
    BindPointReads(&B);
}


// The vertex partition.
// A skin revision's per-vertex work is separable -- point i reads the indices
// and weights at i * elementSize, the influence matrices they name and its
// own incoming position, and nothing else -- so a contiguous range of
// vertices is an independent sub-problem whose result is bit-identical to the
// same vertices computed as part of the whole array. What is NOT separable is
// everything around that body, which is why the range is cut here and the
// decisions stay whole (§6 of docs/specs/baked-step-graph.md).
// The cut is by vertex COUNT and the key follows from it, rather than the
// other way round: the vertex order is the mesh's and is never permuted, so
// two body regions that happen to share a range simply wait for both their
// joints -- and the other ranges still start when their own joints land,
// which is what the whole exercise buys. The schedule report prints |key| per
// chunk so that trade-off is measured per asset rather than assumed.

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

/// How many vertices one chunk covers before the cap takes over.
///
/// The default is the point count at which the geometry kernels start
/// splitting work themselves, because a range smaller than that is a range
/// the deformation was never worth spreading over. Read at Build into
/// chunkVertexTarget.
size_t
RigExecBakedChunkVertexTargetFromEnvironment()
{
    const int authored = TfGetenvInt("RIGEXEC_BAKED_CHUNK_VERTS",
                                     int(RigExecGeometryParallelThreshold));
    return authored < 1 ? size_t(1) : size_t(authored);
}

/// The most chunks one revision is cut into; the range grows to meet it.
///
/// A cap rather than a target: chunk count decides STEP count, and a mesh
/// with a million vertices must not add two hundred steps to the program for
/// a machine that can run twenty of them at once. Read at Build into
/// chunkCap.
size_t
RigExecBakedChunkCapFromEnvironment()
{
    const int authored = TfGetenvInt("RIGEXEC_BAKED_MAX_CHUNKS", 32);
    return authored < 1 ? size_t(1) : size_t(authored);
}

void
RigExecBakedPartitionRevision(
    const RigExecBakedProgramImpl &program,
    RigExecBakedProgramImpl::GeomRevision *revision,
    const VtIntArray &indices, int elementSize)
{
    using GeomChunk = RigExecBakedProgramImpl::GeomChunk;
    const size_t indexCount = indices.size();
    const size_t slots = elementSize < 1 ? 0 : size_t(elementSize);
    const size_t points = slots ? indexCount / slots : 0;
    const size_t influences = revision->influenceSlots.size();
    revision->partitionElementSize = elementSize;
    revision->partitionIndexCount = indexCount;
    revision->partitionPointCount = points;

    // How many ranges, and how long.
    size_t target = program.chunkVertexTarget;
    size_t count = std::max<size_t>(1, (points + target - 1) / target);
    if (count > program.chunkCap) {
        count = program.chunkCap;
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
                const int index = indices.cdata()[point * slots + slot];
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
    revision->partitionIndices = keyed ? indices : VtIntArray();
}

void
RigExecBakedAdoptPartition(RigExecBakedProgramImpl::GeomRevision *revision)
{
    // A null handle is a refusal: the packet reads the arrays per frame and
    // RevisionStatic runs the revision whole. Recording it here would make
    // the handles agree by both being null.
    if (!revision->chunked || !revision->topology ||
        revision->topology == revision->partitionTopology) {
        return;
    }
    // The keys stay Build's whatever the layout says, because each chunk
    // step's reads were declared from its key: a re-cut would let a chunk
    // read a joint it never declared, and a cone that skips it would leave
    // its range stale. So a handle is adopted only when it has the arrays
    // the keys were cut from; a weight-only edit keeps the chunks running.
    const RigExecSkinTopology &topology = *revision->topology;
    const VtIntArray &indices = revision->partitionIndices;
    if (topology.elementSize == revision->partitionElementSize &&
        topology.indices.size() == indices.size() &&
        std::equal(topology.indices.begin(), topology.indices.end(),
                   indices.cdata())) {
        revision->partitionTopology = revision->topology;
    }
}

// Build: the geometry half of the program, in program order.

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

/// Declares the matrices \p revision reads, which are the slots the pose half
/// published for it: the same enumeration the ProviderMatrix steps were
/// created from.
void
DeclareMatrixReads(const RigExecBakedProgramImpl::GeomRevision &revision,
                   RigExecBakedStep *step)
{
    RigExecBakedForEachMatrixRead(
        revision, [step](RigExecBakedSlotDomain domain, int slot) {
        step->reads.push_back(RigExecBakedOne(domain, slot));
    });
}

/// Declares a read of point version \p version: the authored base for 0, and
/// otherwise the RevisionDone and ChainDirty slots of revision
/// first + version - 1, whose only writer is that revision's fuse. The buffer
/// the version resolves to is not declared: whichever earlier chunk or fuse
/// filled it precedes that fuse.
void
DeclarePointVersionRead(const RigExecBakedProgramImpl &B,
                        RigExecBakedPointVersion version,
                        std::vector<RigExecBakedSlotRange> *reads)
{
    if (version.version == 0) {
        reads->push_back(RigExecBakedOne(RigExecBakedSlotDomain::ChainBase,
                                         version.chain));
        return;
    }
    const int revision =
        B.chainRevisionBegin[size_t(version.chain)] + version.version - 1;
    reads->push_back(
        RigExecBakedOne(RigExecBakedSlotDomain::RevisionDone, revision));
    reads->push_back(
        RigExecBakedOne(RigExecBakedSlotDomain::ChainDirty, revision));
}

/// Declares what \p binding can read: each candidate version, or the
/// chain's published points for a final read. The tail is the resolved
/// input, which is prologue state.
void
DeclarePointBindingReads(const RigExecBakedProgramImpl &B,
                         const RigExecBakedPointsBinding &binding,
                         std::vector<RigExecBakedSlotRange> *reads)
{
    for (const RigExecBakedPointVersion &candidate : binding.candidates) {
        if (binding.finalRead) {
            reads->push_back(RigExecBakedOne(
                RigExecBakedSlotDomain::ChainPoints, candidate.chain));
        } else {
            DeclarePointVersionRead(B, candidate, reads);
        }
    }
}

/// The frame records an AtPrim transform phase of \p revision can read,
/// for its transform and for each influence entry. Their FrameMatrix steps
/// are the only writers, and all of them are pose steps.
void
DeclareFrameRecordReads(const RigExecBakedProgramImpl::GeomRevision &revision,
                        std::vector<RigExecBakedSlotRange> *reads)
{
    for (const int record : revision.transformRecords) {
        reads->push_back(
            RigExecBakedOne(RigExecBakedSlotDomain::FrameMatrix, record));
    }
    for (const std::vector<int> &records : revision.influenceRecords) {
        for (const int record : records) {
            reads->push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::FrameMatrix, record));
        }
    }
}

/// The point bindings of \p revision: its declared input phases and its
/// blend samples'.
void
DeclareRevisionPointReads(const RigExecBakedProgramImpl &B,
                          const RigExecBakedProgramImpl::GeomRevision &revision,
                          std::vector<RigExecBakedSlotRange> *reads)
{
    for (const RigExecBakedPointsBinding &binding : revision.pointBindings) {
        DeclarePointBindingReads(B, binding, reads);
    }
    for (const RigExecBakedProgramImpl::GeomBlendChannel &channel :
             revision.blendChannels) {
        for (const auto &sample : channel.samples) {
            DeclarePointBindingReads(B, sample.pointBinding, reads);
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
PartitionAtBuild(const RigExecBakedProgramImpl &B, const ProviderLevels &levels,
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
    // Read directly, at Default: Build holds no layout handle yet. Fixed
    // means the arrays do not vary in time, so the default-time read IS the
    // epoch's. A handle whose arrays disagree with it (an override standing
    // on the indices at Build) is never adopted, and the revision runs whole
    // (RigExecBakedAdoptPartition).
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
    RigExecBakedPartitionRevision(B, revision, indices, elementSize);
    revision->partitionCandidates = revision->chunks.size();

    // Whether the cut pays, which is a question about LEVELS and not about
    // vertex counts.
    // A chunk body is a serial loop over its range; an uncut revision is one
    // call to RigExecApplySkinKernel over the whole array, and that kernel
    // spreads itself over the arena. So cutting a revision into seven ranges
    // that all become runnable at the same level does not overlap anything
    // -- it replaces one data-parallel call with seven serial ones, measured
    // at 47us serial and 92us parallel of the biped's frame. What the cut is
    // FOR is the range whose own joints land early: it may start while the
    // rest of the rig is still being posed, and that is only possible when
    // the candidate ranges become ready at different levels.
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
        // are the layout's, not the cut's, and the report reads them -- but
        // the indices, which only an adoption reads, are let go.
        revision->chunks.assign(1, RigExecBakedProgramImpl::GeomChunk());
        revision->chunked = false;
        revision->partitionIndices = VtIntArray();
    }
}

/// Declares WeightFrames[slot] for every volume in weight object \p object's
/// composition closure (its baseWeight and inputWeights, recursively): the
/// placements the oracle reads when it resolves that object. A closure
/// member outside the table, or a volume slot that has no placement step,
/// should be unreachable; it falls back to every volume slot.
void
DeclarePlacementReads(const RigExecBakedProgramImpl &B, int object,
                      std::vector<RigExecBakedSlotRange> *reads)
{
    std::vector<char> seen(B.weightObjects.size(), 0);
    std::vector<int> stack{object};
    std::vector<int> slots;
    bool complete = true;
    while (!stack.empty()) {
        const int o = stack.back();
        stack.pop_back();
        if (o < 0 || size_t(o) >= B.weightObjects.size()) {
            complete = false;
            continue;
        }
        if (seen[size_t(o)]) {
            continue;
        }
        seen[size_t(o)] = 1;
        const RigExecBakedProgramImpl::WeightObject &weight =
            B.weightObjects[size_t(o)];
        if (weight.providerSlot >= 0) {
            if (size_t(weight.providerSlot) < B.noScaleAvars.size() &&
                B.noScaleAvars[size_t(weight.providerSlot)]) {
                slots.push_back(weight.providerSlot);
            } else {
                complete = false;
            }
        }
        if (weight.base >= 0) {
            stack.push_back(weight.base);
        }
        stack.insert(stack.end(), weight.inputs.begin(), weight.inputs.end());
    }
    if (!TF_VERIFY(complete, "a weight object's closure is outside the "
                             "program's volume table")) {
        slots.clear();
        for (size_t i = 0; i < B.noScaleAvars.size(); ++i) {
            if (B.noScaleAvars[i]) {
                slots.push_back(int(i));
            }
        }
    }
    for (const int slot : slots) {
        reads->push_back(
            RigExecBakedOne(RigExecBakedSlotDomain::WeightFrames, slot));
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
        // revisions are a contiguous RANGE: point version v of the chain is
        // revision first + v - 1's, and the status sweep reads the whole
        // chain as one declared range. The chunk ids inside them are
        // contiguous for the same reason and at the same grain.
        B.chainRevisionBegin[c] = int(B.revisionIndex.size());
        B.chainChunkBegin[c] = nextChunk;
        for (size_t r = 0; r < chain.revisions.size(); ++r) {
            B.revisionIndex.emplace_back(int(c), int(r));
        }
        B.chainRevisionEnd[c] = int(B.revisionIndex.size());
        const int first = B.chainRevisionBegin[c];
        const int last = B.chainRevisionEnd[c];
        B.revisionFuseStep.resize(size_t(last), -1);
        for (size_t r = 0; r < chain.revisions.size(); ++r) {
            RigExecBakedProgramImpl::GeomRevision &revision =
                chain.revisions[r];
            const int id = first + int(r);
            // The points entering this revision, which its chunks, its fuse
            // and a current-phase assemble read.
            const RigExecBakedPointVersion entering{int(c), int(r)};
            const bool skin = revision.op == RigExecRevisionOp::Skin;
            revision.influences.assign(revision.influenceSlots.size(),
                                       GfMatrix4d(1.0));
            // The table the PACKET carries for a skin revision, and never
            // anything else: identity, sized to the influence count and
            // written once, here.
            // The packet is assembled before the matrices are folded, so it
            // cannot carry them -- that is what lets a chunk start on its own
            // joints without waiting for every joint of the rig. What the
            // assembler does with the table is check its SHAPE and its
            // elements' finiteness, and identities pass both; the real
            // table's finite/affine check is the fold's, and the fuse ANDs it
            // in exactly where the assembler's would have landed.
            revision.packetInfluences.assign(revision.influenceSlots.size(),
                                             GfMatrix4d(1.0));
            PartitionAtBuild(B, levels, &revision);
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
                // A pose-walk read phase reads the frame records its lists
                // name, each written by one FrameMatrix step.
                DeclareFrameRecordReads(revision, &fold.reads);
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
                    // ENTERING this revision, so the assemble reads that
                    // version exactly as a chunk does. The oracle places
                    // each volume of the field from the program's
                    // volumePlacement table, so it waits for the
                    // VolumePlacements step of each of those volumes.
                    assemble.maxDiagnostics += 1;
                    DeclarePlacementReads(B, revision.weightObject,
                                          &assemble.reads);
                    DeclarePointVersionRead(B, entering, &assemble.reads);
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
                // The versions its phased inputs and blend samples are bound
                // to. The AtPrim transform phase is the fold's read.
                DeclareRevisionPointReads(B, revision, &assemble.reads);
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
                        RigExecBakedOwnMatrixDomain(revision);
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
                // Which buffer holds the entering points is decided by the
                // previous fuse, so the read is that fuse's version.
                DeclarePointVersionRead(B, entering, &chunk.reads);
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
                // Its own chunks' ranges. The entering points, which the
                // whole-revision fallback and a pass-through read, are a
                // version like a chunk's.
                fuse.reads.push_back(RigExecBakedRange(
                    RigExecBakedSlotDomain::RevisionOut, revision.chunkBase,
                    revision.chunkBase + int(revision.chunks.size())));
                DeclarePointVersionRead(B, entering, &fuse.reads);
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
                B.revisionFuseStep[size_t(id)] = int(B.steps.size()) - 1;
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
            // Every revision's published status; the last one's
            // `currentSource` among them names version n, the final points.
            if (last > first) {
                status.reads.push_back(RigExecBakedRange(
                    RigExecBakedSlotDomain::RevisionDone, first, last));
            }
            status.writes.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::ChainPoints, int(c)));
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
            // A derived revision's binding carries phases like any other, so
            // it reads the versions they are bound to. Its fold runs inside
            // this step, so it reads the frame records an AtPrim transform
            // phase names too.
            DeclareRevisionPointReads(B, derived.revision, &step.reads);
            DeclareFrameRecordReads(derived.revision, &step.reads);
            DeclareMatrixReads(derived.revision, &step);
            step.writes.push_back(
                RigExecBakedOne(RigExecBakedSlotDomain::DerivedOut, id));
        }
    }
}

// One frame, geometry half.
// The packet is assembled by the same RigExecAssembleParameters the dynamic
// path calls and the kernel is the same shared kernel; only the VdfNetwork
// around them is baked away, so the accounting it performed -- a revision
// runs when its inputs changed, and not otherwise -- is performed here
// instead, by the four steps a revision is made of.

namespace {

/// The points of version \p version of \p chain (RigExecBakedPointVersion):
/// 0 is the authored base, v > 0 what revision v - 1's fuse left.
///
/// Not a buffer of its own: revision v - 1's `currentSource` names the
/// revision whose output buffer holds the value, so a pass-through version
/// names an earlier revision's buffer, or the base when it is -1.
void
PointsAt(const RigExecBakedProgramImpl::GeomChain &chain, size_t version,
         const GfVec3f **points, size_t *count)
{
    const int source =
        version == 0 ? -1 : chain.revisions[version - 1].currentSource;
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
    // rigExec:space, out of the same table at the same phase, so this reads
    // exactly what the dynamic path's carry tap reads. Held on the revision
    // for the assemble, which may not read the tables itself: the fold is
    // the step that declared them.
    revision->haveCarry = revision->carrySpaceSlot >= 0;
    if (revision->haveCarry) {
        const GfMatrix4d &carry =
            revision->finalPhase
                ? B.finalMatrix[size_t(revision->carrySpaceSlot)]
                : B.baseMatrix[size_t(revision->carrySpaceSlot)];
        if (revision->carry != carry) {
            revision->carry = carry;
            changed = true;
        }
    }
    // A geometry-domain constraint's delta, if one was measured for this
    // mover: the pose walk solved the constraint and stashed the map from the
    // target's authored transform to the solved one, and THIS is the Matrix
    // revision that same constraint contributes. Find-guarded, and here
    // rather than in the assemble because the transform table is what this
    // step declares -- the assemble only READS it, and a step that writes a
    // slot it declared as a read is the declaration the executor trusts
    // being wrong.
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
    // as it stood immediately after one named constraint, out of the frame
    // records bound at Build instead of out of the dense tables.
    // AFTER the delta, exactly as the dynamic path orders the two, and the
    // two can no more both apply here than they can there: an AtPrim phase
    // needs a bound rigExec:transform to name a frame chain of, and a
    // geometry-domain constraint's binding.transform is empty -- which is
    // what makes its delta the only source of the matrix.
    // The first valid record of the list answers: the last record the walk
    // made under the phase's prim. A list with none valid leaves the
    // dense-table value standing and says nothing, as the dynamic path does:
    // every named constraint under the prim declined to record this run (a
    // geometry-domain exit, an unusable frame).
    const auto phased = [&B](const std::vector<int> &records,
                             GfMatrix4d *matrix) {
        for (const int record : records) {
            if (B.frameMatrixValid[size_t(record)]) {
                *matrix = B.frameMatrix[size_t(record)];
                return true;
            }
        }
        return false;
    };
    const bool atPrim = revision->binding.transformPhase.kind ==
                        RigExecReadPhaseKind::AtPrim;
    if (atPrim && phased(revision->transformRecords, &revision->transform)) {
        // Set rather than left alone: the dynamic path binds its matrix
        // pointer whenever the store answered, whatever the tap held.
        revision->haveTransform = true;
    }
    const bool hasReference = revision->op == RigExecRevisionOp::Matrix &&
                              !revision->influenceSlots.empty();
    const auto referenceMatrix = [&B, revision](size_t index) {
        const size_t slot = size_t(revision->influenceSlots[index]);
        return revision->finalPhase ? B.finalMatrix[slot] : B.baseMatrix[slot];
    };
    if (revision->haveTransform && hasReference) {
        revision->transform = RigExecMeasureFromReference(revision->transform, referenceMatrix(0));
    }
    if (revision->haveTransform && revision->transformSpaceSlot >= 0) {
        GfMatrix4d space =
            revision->finalPhase
                ? B.finalMatrix[size_t(revision->transformSpaceSlot)]
                : B.baseMatrix[size_t(revision->transformSpaceSlot)];
        // The reference refines the SPACE first, then the carry
        // consumes it: the two are sequential, not alternatives.
        if (hasReference && revision->influenceSlots.size() > 1) {
            space = RigExecMeasureFromReference(space, referenceMatrix(1));
        }
        // rigExec:space, as read above. A POINTER, because a revision
        // naming no carry must take the untouched branch and not one
        // multiplied by an identity: see RigExecClusterInPointFrame.
        const GfMatrix4d *carry =
            revision->haveCarry ? &revision->carry : nullptr;
        revision->transform = RigExecClusterInPointFrame(
            RigExecMeasureInSpace(revision->transform, space), space,
            revision->posedPoints, carry);
    }
    for (size_t k = 0; k < revision->influenceSlots.size(); ++k) {
        const size_t slot = size_t(revision->influenceSlots[k]);
        GfMatrix4d matrix = revision->finalPhase ? B.finalMatrix[slot]
                                                 : B.baseMatrix[slot];
        // Every influence shares the one declared phase, so the substitution
        // is per ENTRY and the provider is the entry's own. A skin never
        // gets here: compile refuses an AtPrim phase whose binding.transform
        // is empty, as a skin's is. A matrix mover's influences are its
        // reference providers; the Matrix op does not read them, but its
        // packet carries this table into the executed decision, as the
        // walk's fold does.
        if (atPrim && k < revision->influenceRecords.size()) {
            phased(revision->influenceRecords[k], &matrix);
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
FoldTransformForms(RigExecBakedProgramImpl::GeomRevision *revision,
                   bool useSimd)
{
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
ChunkTransformsView(const RigExecBakedProgramImpl::GeomChunk &chunk,
                    bool useSimd)
{
    RigExecSkinTransformsView view;
    view.transforms = chunk.transforms.data();
    view.transformCount = chunk.transforms.size();
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
          size_t begin, size_t end, bool whole, bool useSimd)
{
    std::vector<GfVec3f> &out = revision->output;
    std::copy(preceding + begin, preceding + end, out.begin() + long(begin));
    if (whole) {
        if (!RigExecApplySkinKernelWithTransforms(revision->parameters, view,
                                                  &out, useSimd)) {
            return false;
        }
    } else if (!RigExecApplySkinKernelRange(revision->parameters, view, begin,
                                            end, &out, useSimd)) {
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

/// The test capture (RigExecBakedProgramTesting::CapturePointReads): the
/// binding's answer for \p binding as its reader took it this run.
void
CapturePointRead(RigExecBakedProgramImpl *program,
                 const RigExecBakedPointsBinding &binding,
                 const GfVec3f *points, size_t count, bool answered)
{
    RigExecBakedProgramImpl &B = *program;
    if (!B.capturePointReads || binding.id < 0 ||
        size_t(binding.id) >= B.pointCaptures.size()) {
        return;
    }
    RigExecBakedPointCapture &capture = B.pointCaptures[size_t(binding.id)];
    capture.read = true;
    capture.bindingAnswered = answered;
    capture.bound =
        answered ? VtVec3fArray(points, points + count) : VtVec3fArray();
}

}  // namespace

bool
RigExecBakedResolvePoints(const RigExecBakedProgramImpl &B,
                          const RigExecBakedPointsBinding &binding,
                          const GfVec3f **points, size_t *count)
{
    for (const RigExecBakedPointVersion &candidate : binding.candidates) {
        const RigExecBakedProgramImpl::GeomChain &chain =
            B.chains[size_t(candidate.chain)];
        if (!chain.haveBase) {
            continue;
        }
        if (binding.finalRead) {
            *points = chain.result.cdata();
            *count = chain.result.size();
        } else {
            PointsAt(chain, size_t(candidate.version), points, count);
        }
        return true;
    }
    return false;
}

void
RigExecBakedOverlayPointReads(RigExecBakedProgramImpl *program,
                              RigExecBakedProgramImpl::GeomRevision *revision,
                              const RigExecResolvedInputs &resolved,
                              std::vector<std::string> *diagnostics)
{
    if (revision->binding.phases.empty()) {
        return;
    }
    RigExecBakedProgramImpl &B = *program;
    revision->revisionInputs = resolved;
    for (const RigExecBakedPointsBinding &binding : revision->pointBindings) {
        const GfVec3f *points = nullptr;
        size_t count = 0;
        const bool answered =
            RigExecBakedResolvePoints(B, binding, &points, &count);
        CapturePointRead(&B, binding, points, count, answered);
        if (answered) {
            // A final read shares the chain's published buffer, as the
            // walk's record of it does.
            revision->revisionInputs.SetProperty(
                binding.input,
                binding.finalRead
                    ? VtValue(B.chains[size_t(binding.candidates[0].chain)]
                                  .result)
                    : VtValue(VtVec3fArray(points, points + count)));
        } else if (binding.diagnoseMiss) {
            // A `preceding` tail is silent, as it is in the dynamic walk.
            diagnostics->push_back(
                "diag " + revision->moverPath.GetString() +
                ": read phase '" + binding.phase.GetAsString() + "' for " +
                binding.input.GetString() +
                " resolved to nothing; read the authored base");
        }
    }
}

namespace {

// WHICH points a revision is assembled against, stated once because the two
// callers below pass different ones and the difference is invisible until an
// operator reads them:
//  * a CHAIN revision gets the chain's AUTHORED base -- the attribute as the
//    stage holds it -- for every revision of the chain, not the running
//    value. The dynamic walk reads the base once per target and hands that
//    same array to every revision, so a lattice's rest points and a
//    volumeCorrect's reference volume are measured against the mesh as
//    authored however deep in the chain they sit.
//  * a DERIVED revision gets the chain's FINAL points, which is what it is
//    for: recomputeNormals and recomputeExtent take auxPoints = basePoints
//    and must describe the geometry as published.
// Taken as a range rather than a vector because the two callers hold the
// points in different containers and RigExecProviderValues copies them
// anyway.
//
// The provider values both assemblies take: everything but the stage reads.
RigExecProviderValues
RevisionValues(RigExecBakedProgramImpl &B,
               RigExecBakedProgramImpl::GeomRevision *revision,
               const GfVec3f *basePoints, size_t basePointCount)
{
    RigExecProviderValues values;
    // The fold decided whether there is a matrix at all -- a bound transform
    // provider, or a geometry-domain constraint's delta -- and wrote it.
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
    // The carry the fold read beside the transform, for the wire assembler.
    // A skin's fold runs after its assemble, but a skin never names one.
    if (revision->haveCarry) {
        values.carry = &revision->carry;
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
    // Epoch-fixed layouts were adopted in the PROLOGUE from the SkinTopology
    // op, which builds them by the dynamic cache's own rules
    // (RigExecBuildSkinTopology) -- so the two paths cannot hold different
    // arrays, and no step body reads one off the stage.
    if (revision->topologyResolved) {
        values.skinTopology = &revision->topology;
    }
    return values;
}

// The blend channels. Gathered here rather than inside the assembler
// because the dynamic walk gathers them here too, and the ORDER is the
// whole contract: channels in `binding.blendInputs` order, which compile
// sorted, and each channel's samples stable-sorted by activation. Float
// addition is not associative, so a different order is a different last
// bit of every point.
// The reads are the generation-wide resolved inputs' and NOT this
// revision's phase overlay's: a sample's own phase is looked up directly,
// which is what lets one blend sample read another chain while the rest of
// the channel reads the stage. \p read answers each one: the stage gather
// through the resolved inputs, the leaf gather from the leaves the prologue
// sampled through them. \p record writes each dense sample's `lastPoints`.
template <class Read>
void
GatherBlendChannels(RigExecBakedProgramImpl &B,
                    RigExecBakedProgramImpl::GeomRevision *revision,
                    const Read &read, bool record,
                    RigExecProviderValues *values)
{
    if (revision->blendChannels.empty()) {
        return;
    }
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
        if (bound.poseWeight >= 0 && !read.WeightHeld(bound)) {
            channel.weight = B.poseWeights[size_t(bound.poseWeight)];
        } else {
            read.Weight(bound, &channel.weight);
        }
        for (size_t s = 0; s < bound.samples.size(); ++s) {
            auto &boundSample = bound.samples[s];
            RigExecBlendSampleData sample;
            read.Activation(boundSample, &sample.activation);
            if (!boundSample.blendShape.IsEmpty()) {
                // Sparse: the shape the prologue resolved, shared by
                // pointer. The packet compares layouts by pointer, so an
                // unchanged sample costs one compare and not an array.
                sample.layout = boundSample.layout;
                channel.samples.push_back(std::move(sample));
                continue;
            }
            // A phased sample reads the version it is bound to, and the
            // resolved input when that chain read no base: silently,
            // as the dynamic gather falls back.
            const GfVec3f *phased = nullptr;
            size_t phasedCount = 0;
            const bool answered =
                boundSample.pointBinding.id >= 0 &&
                RigExecBakedResolvePoints(B, boundSample.pointBinding,
                                          &phased, &phasedCount);
            if (boundSample.pointBinding.id >= 0) {
                CapturePointRead(&B, boundSample.pointBinding, phased,
                                 phasedCount, answered);
            }
            if (answered) {
                sample.points.assign(phased, phased + phasedCount);
            } else {
                read.Points(boundSample, &sample.points);
            }
            if (record) {
                boundSample.lastPoints = sample.points;
            }
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
    if (!RigExecSumBlendChannels(channels, values->basePoints,
                                 &values->blendDeltas)) {
        values->blendDeltas.clear();
    }
}

// The blend gather's reads, off the stage through the resolved inputs.
struct _StageBlendReads {
    const RigExecResolvedInputs &R;
    UsdTimeCode time;

    bool WeightHeld(
        const RigExecBakedProgramImpl::GeomBlendChannel &bound) const {
        return R.Find(bound.weightPath) != nullptr;
    }
    void Weight(const RigExecBakedProgramImpl::GeomBlendChannel &bound,
                float *out) const {
        R.GetAttribute(bound.weight, time, out);
    }
    void Activation(
        const RigExecBakedProgramImpl::GeomBlendChannel::Sample &sample,
        float *out) const {
        R.GetAttribute(sample.activation, time, out);
    }
    void Points(const RigExecBakedProgramImpl::GeomBlendChannel::Sample &sample,
                std::vector<GfVec3f> *out) const {
        VtVec3fArray points;
        R.GetAttribute(sample.points, time, &points);
        out->assign(points.begin(), points.end());
    }
};

// The same reads, from the revision's path leaves.
struct _LeafBlendReads {
    const RigExecBakedPathLeaves &leaves;

    bool WeightHeld(
        const RigExecBakedProgramImpl::GeomBlendChannel &bound) const {
        return leaves.Value<bool>(bound.weightHeldLeaf, false);
    }
    void Weight(const RigExecBakedProgramImpl::GeomBlendChannel &bound,
                float *out) const {
        *out = leaves.Value<float>(bound.weightLeaf, *out);
    }
    void Activation(
        const RigExecBakedProgramImpl::GeomBlendChannel::Sample &sample,
        float *out) const {
        *out = leaves.Value<float>(sample.activationLeaf, *out);
    }
    void Points(const RigExecBakedProgramImpl::GeomBlendChannel::Sample &sample,
                std::vector<GfVec3f> *out) const {
        const VtVec3fArray points =
            leaves.Value<VtVec3fArray>(sample.pointsLeaf, VtVec3fArray());
        out->assign(points.begin(), points.end());
    }
};

// The packet off the stage, through \p R (or the revision's phase overlay
// over it), with RigExecAssembleParameters: a plugin revision bound to a
// value the region computes, and the shadow test's reference for the rest.
RigExecMoverParameters
AssembleRevisionFromStage(RigExecBakedProgramImpl &B,
                          const RigExecResolvedInputs &R,
                          RigExecBakedProgramImpl::GeomRevision *revision,
                          const GfVec3f *basePoints, size_t basePointCount,
                          UsdTimeCode time,
                          std::vector<std::string> *diagnostics, bool record)
{
    RigExecProviderValues values =
        RevisionValues(B, revision, basePoints, basePointCount);
    values.resolved = &R;
    // One overlay per REVISION: the generation-wide resolved inputs, plus
    // whatever this revision's declared phases resolve to through their
    // bindings. The assembler reads inputs by path and never learns a phase
    // exists, which is what lets a phase apply to any input. Built only for
    // a revision that declares one, and owned by that revision, because one
    // buffer shared by the walk is state two steps could be inside at once.
    if (!revision->binding.phases.empty()) {
        RigExecBakedOverlayPointReads(&B, revision, R, diagnostics);
        values.resolved = &revision->revisionInputs;
    }
    GatherBlendChannels(B, revision, _StageBlendReads{R, time}, record,
                        &values);
    return RigExecAssembleParameters(revision->moverPrim, revision->op,
                                     revision->binding, values, time);
}

}  // namespace

RigExecMoverParameters
RigExecBakedAssembleFromLeaves(
    RigExecBakedProgramImpl *program,
    RigExecBakedProgramImpl::GeomRevision *revision,
    const GfVec3f *basePoints, size_t basePointCount,
    std::vector<std::string> *diagnostics, std::vector<std::string> *missing)
{
    RigExecBakedProgramImpl &B = *program;
    RigExecProviderValues values =
        RevisionValues(B, revision, basePoints, basePointCount);
    RigExecRevisionLeafView view;
    view.decl = &revision->leaves.decl;
    view.values = &revision->leaves.values;
    view.missing = missing;
    if (revision->op == RigExecRevisionOp::External) {
        view.external = &revision->externalPayload;
    }
    // The phase overlay, as the stage assembly builds it: a phased points
    // read takes the version it is bound to before its leaf.
    if (!revision->binding.phases.empty()) {
        RigExecBakedOverlayPointReads(&B, revision, *B.resolvedInputs,
                                      diagnostics);
        view.phased = &revision->revisionInputs;
    }
    GatherBlendChannels(B, revision, _LeafBlendReads{revision->leaves},
                        /*record=*/true, &values);
    return RigExecAssembleFromLeaves(revision->op, revision->binding, view,
                                     values);
}

namespace {

// The packet of \p revision this run: from its path leaves wherever they
// assemble it, off the stage for a plugin revision bound to region values.
RigExecMoverParameters
AssembleRevision(RigExecBakedProgramImpl &B,
                 RigExecBakedProgramImpl::GeomRevision *revision,
                 const GfVec3f *basePoints, size_t basePointCount,
                 UsdTimeCode time, RigExecBakedStep *step)
{
    if (revision->leaves.decl.assembles) {
        return RigExecBakedAssembleFromLeaves(&B, revision, basePoints,
                                              basePointCount,
                                              &step->diagnostics);
    }
    // Volatile until plugin API v3 (bodyPurity.h): the plugin is handed
    // values the region computes, so it cannot run in the prologue.
    const RigExecVolatileRead volatileRead;
    return AssembleRevisionFromStage(B, *B.resolvedInputs, revision,
                                     basePoints, basePointCount, time,
                                     &step->diagnostics, /*record=*/true);
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
                  size_t revisionIndex, bool useSimd)
{
    const GfVec3f *points = nullptr;
    size_t count = 0;
    PointsAt(chain, revisionIndex, &points, &count);
    if (!revision->layoutUsable || !revision->envelopeOk ||
        count != revision->precedingCount ||
        revision->output.size() != count) {
        return false;
    }
    // Against the forms the FOLD wrote -- this step reads RevisionTransforms
    // and writes none of it, so the table it skins against is the one that
    // slot already holds.
    return SkinRange(revision, points, WholeTransformsView(revision), 0, count,
                     /*whole=*/true, useSimd);
}

}  // namespace

void
RigExecBakedRunGeometryPrologue(RigExecBakedProgramImpl *program,
                                UsdTimeCode time, RigExecRigPose *pose,
                                bool all)
{
    RigExecBakedProgramImpl &B = *program;
    RIGEXEC_PROFILE_SCOPE_CAT(*B.profiler, "BakedChainBase", "geometry");
    // The path leaves' chain rule: the results every leaf read through
    // moved since the serial a leaf last saw.
    if (B.hasPropertyChains &&
        B.propertyResults != B.pathLeafChainResults) {
        B.pathLeafChainResults = B.propertyResults;
        ++B.pathLeafChainSerial;
    }
    // The weight objects' point gathers, read by WeightPacket steps.
    for (RigExecBakedProgramImpl::WeightObject &object : B.weightObjects) {
        RigExecBakedSamplePathLeaves(&B, &object.pointLeaves, time, all);
    }
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
    // The SkinTopology op's handle, adopted here and only here: topology,
    // topologyResolved and the partition are exported, so they are set
    // only for a revision whose base reads, on a generation that reaches
    // this prologue, however often the op itself runs.
    const auto resolveTopology =
        [](RigExecBakedProgramImpl::GeomRevision *revision) {
        if (!revision->skinTopologyFixed) {
            return;
        }
        revision->topology = revision->layoutHandle;
        revision->topologyResolved = true;
        // The op hands back the SAME handle for a layout that did not move,
        // so only a different one costs the comparison with Build's arrays.
        // Here rather than in a step because it reads the arrays.
        RigExecBakedAdoptPartition(revision);
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
    // A revision's leaves, once its layout is resolved: a skin revision
    // holding an epoch layout reads no per-frame layout arrays, so those
    // keys wait until a run reads them.
    std::vector<int> skip;
    const auto sampleRevision =
        [&B, time, all, &skip](RigExecBakedProgramImpl::GeomRevision *revision) {
            skip.clear();
            if (revision->topologyResolved && revision->topology) {
                const RigExecRevisionLeafDecl &decl = revision->leaves.decl;
                for (const RigExecRevisionLeafRole role :
                     {RigExecRevisionLeafRole::JointIndices,
                      RigExecRevisionLeafRole::JointWeights,
                      RigExecRevisionLeafRole::ElementSize}) {
                    if (decl.Role(role) >= 0) {
                        skip.push_back(decl.Role(role));
                    }
                }
            }
            RigExecBakedSamplePathLeaves(&B, &revision->leaves, time, all,
                                         skip);
        };
    // An external revision's payload leaf, after its enable and envelope
    // leaves: the plugin's answer at this time, over the provider values the
    // body would hand it (the chain's authored base, the generation's
    // resolved inputs; it binds nothing the region computes), and only past
    // the gates at which the stage assembler calls it.
    const auto sampleExternal =
        [&B, time](RigExecBakedProgramImpl::GeomRevision *revision,
                   const VtVec3fArray &base) {
            if (revision->op != RigExecRevisionOp::External ||
                !revision->leaves.decl.assembles) {
                return;
            }
            RigExecProviderValues values =
                RevisionValues(B, revision, base.cdata(), base.size());
            values.resolved = B.resolvedInputs;
            RigExecRevisionLeafView view;
            view.decl = &revision->leaves.decl;
            view.values = &revision->leaves.values;
            RigExecExternalPayload payload;
            if (RigExecExternalPayloadIsRead(view, values)) {
                RigExecAssembleExternalPayload(revision->moverPrim,
                                               revision->binding, values, time,
                                               &payload);
            }
            revision->externalPayloadChanged =
                !(payload == revision->externalPayload);
            revision->externalPayload = std::move(payload);
        };
    for (RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        // The chain base: an upstream value on the target, else the stage,
        // never the interactive overlay (as the dynamic walk reads it).
        VtVec3fArray basePoints;
        const auto upstream = B.upstream.find(chain.target);
        if (upstream != B.upstream.end() &&
            upstream->second.IsHolding<VtVec3fArray>()) {
            basePoints = upstream->second.UncheckedGet<VtVec3fArray>();
            chain.haveBase = true;
        } else {
            chain.haveBase = chain.baseQuery.IsValid() &&
                             chain.baseQuery.Get(&basePoints, time);
        }
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
            sampleRevision(&revision);
            sampleExternal(&revision, chain.lastBase);
        }
        for (RigExecBakedProgramImpl::GeomChain::Derived &derived :
                 chain.derived) {
            if (derived.matrixTarget) {
                // No base and no graph node: the dynamic walk evaluates a
                // projector target directly every generation. Its reads are
                // leaves like any revision's.
                derived.haveBase = true;
                derived.baseDirty = true;
                derived.revision.created = false;
                sampleRevision(&derived.revision);
                continue;
            }
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
            sampleRevision(&derived.revision);
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
        if (derived.matrixTarget) {
            step->counters.revisionsBuilt = 1;
            derived.haveMatrix = RigExecBakedRunProjectorTarget(
                B, chain, revision, &derived.matrix, &step->diagnostics);
            derived.haveResult = true;
            step->counters.chainsBuilt = 1;
            return;
        }
        FoldInfluences(B, &revision);
        RigExecMoverParameters parameters = AssembleRevision(
            B, &revision, chain.result.cdata(), chain.result.size(), time,
            step);
        const RigExecMoverStatus status =
            RigExecStatusForParameters(parameters, revision.moverPath);
        step->counters.revisionsBuilt = 1;
        // The 26k-point input is remembered by HANDLE, not by copy.
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
                                         B.useSimd, &revision.wireBasis);
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
        // Version n, the chain's final points; a chain of no revisions
        // publishes its base.
        const GfVec3f *points = nullptr;
        size_t count = 0;
        PointsAt(chain, chain.revisions.size(), &points, &count);
        // Double-buffered: publication is a refcount bump for the consumer
        // and the array one may still hold from last frame is never the one
        // being written.
        chain.spare.resize(count);
        std::copy(points, points + count, chain.spare.data());
        chain.result.swap(chain.spare);
        chain.haveResult = true;
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
    // where it starts. Only the chunk and the fuse read it. The fold, and an
    // assemble that neither measures a current-phase field nor reads its own
    // chain at `preceding`, do not declare version r, so they may run beside
    // the fuse that writes it.
    const auto chainDirtyBefore = [&chain, revisionIndex = revisionIndex] {
        return revisionIndex == 0
                   ? chain.baseDirty
                   : chain.revisions[size_t(revisionIndex) - 1].executed;
    };
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
            FoldTransformForms(&revision, B.useSimd);
        }
        return;
    }

    case RigExecBakedStepKind::RevisionStatic: {
        // A volume weight reading `preceding` on rigExec:weightTarget: the
        // field is measured against the
        // points AS THEY STAND HERE, not the authored base, so the volume
        // grabs whatever is inside it right now.
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
            PointsAt(chain, size_t(revisionIndex), &entering,
                     &enteringCount);
            const std::vector<GfVec3f> current(entering,
                                               entering + enteringCount);
            std::vector<float> field;
            std::string error;
            bool resolved = false;
            {
                // The weight oracle: volatile until S4 (bodyPurity.h).
                const RigExecVolatileRead volatileRead;
                resolved = B.resolveWeights(
                    B.weightObjects[size_t(revision.weightObject)].path,
                    current.size(), time, &field, &error, &current);
            }
            if (resolved) {
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
        // The scalar the fuse's one diagnostic is about, taken here (its
        // leaf) because this step always runs and the fuse may not. Compared
        // like every other source value: two different out-of-range weights
        // can leave the packet identical, and the line the fuse emits is
        // about the weight rather than about the packet.
        revision.defaultWeight =
            revision.leaves.Value<float>(revision.defaultWeightLeaf, 1.0f);
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
            // the identity the SkinTopology op preserves for a binding that
            // did not move, and the prologue adopted it only if it has the
            // partition's arrays; anything else here means the packet is
            // reading arrays the keys were not cut from, and the fuse runs
            // the revision whole instead.
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
        PointsAt(chain, size_t(revisionIndex), &points, &count);
        const bool chainDirty = chainDirtyBefore();
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
                chunk.ok = RigExecRunRevisionKernel(
                    revision.op, revision.parameters, &revision.output,
                    B.useSimd, &revision.wireBasis);
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
                                 /*whole=*/true, B.useSimd);
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
            // `ok` is part of the gate rather than a consequence of it: a
            // chunk whose last answer was a failure, or one the partition
            // reset without the packet moving, has nothing in its range to
            // keep, and reading that off its own flag keeps the invariant
            // local to this step.
            return;
        }
        chunk.ok = SkinRange(&revision, points,
                             ChunkTransformsView(chunk, B.useSimd),
                             size_t(chunk.begin), size_t(chunk.end),
                             /*whole=*/false, B.useSimd);
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
        revision.executed = chainDirtyBefore() || revision.staticDirty ||
                            (skin && revision.influencesChanged);
        step->counters.revisionsExecuted = revision.executed ? 1 : 0;
        if (revision.executed) {
            revision.resultStatus = revision.status.state;
            bool applied = packetValid && revision.status.AllowsApply();
            if (applied && skin && revision.partitionStale) {
                applied = FuseWholeRevision(chain, &revision,
                                            size_t(revisionIndex), B.useSimd);
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
        return;
    }

    default:
        return;
    }
}

// The report.

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
                            RigExecRigPose *pose)
{
    RigExecBakedProgramImpl &B = *program;
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
                if (!derived.matrixTarget) {
                    pose->movedProperties[derived.target] =
                        VtValue(derived.result);
                } else if (derived.haveMatrix) {
                    pose->movedProperties[derived.target] =
                        VtValue(derived.matrix);
                }
            }
        }
    }
}

RigExecSurfaceProjectorFrames
RigExecBakedProjectorFrames(const RigExecBakedProgramImpl &B,
                            const RigExecBakedProgramImpl::GeomRevision &revision)
{
    // Asset frames: each provider's rest times its base and final
    // computeMatrix, the tables every geometry revision reads.
    RigExecSurfaceProjectorFrames frames;
    const SdfPath providers[3] = {revision.binding.transform,
                                  revision.binding.transformSpace,
                                  revision.binding.carrySpace};
    const int slots[3] = {revision.transformSlot,
                          revision.transformSpaceSlot,
                          revision.carrySpaceSlot};
    for (int k = 0; k < 3; ++k) {
        if (providers[k].IsEmpty()) continue;
        frames.named[k] = true;
        const int slot = slots[k];
        if (slot < 0 || !B.restFrames[size_t(slot)].IsValid()) continue;
        const auto &rest = B.restFrames[size_t(slot)].points;
        frames.base[k] =
            RigExecWorldFromRest(rest, B.baseMatrix[size_t(slot)]);
        frames.final[k] =
            RigExecWorldFromRest(rest, B.finalMatrix[size_t(slot)]);
        frames.resolved[k] = true;
    }
    return frames;
}

bool
RigExecBakedRunProjectorTarget(
    const RigExecBakedProgramImpl &B,
    const RigExecBakedProgramImpl::GeomChain &chain,
    const RigExecBakedProgramImpl::GeomRevision &revision,
    GfMatrix4d *matrix, std::vector<std::string> *diagnostics)
{
    RigExecRevisionLeafView view;
    view.decl = &revision.leaves.decl;
    view.values = &revision.leaves.values;
    RigExecProjectorReads reads;
    RigExecReadProjectorTargetFromLeaves(revision.op, revision.binding, view,
                                         &reads);
    return RigExecRunProjectorTarget(
        revision.op, revision.binding,
        RigExecBakedProjectorFrames(B, revision), reads,
        std::vector<GfVec3f>(chain.lastBase.begin(), chain.lastBase.end()),
        std::vector<GfVec3f>(chain.result.begin(), chain.result.end()),
        matrix, diagnostics);
}

void
RigExecBakedBindPathLeaves(const UsdStageRefPtr &stage,
                           RigExecBakedPathLeaves *leaves)
{
    const size_t n = leaves->decl.keys.size();
    leaves->attributes.assign(n, UsdAttribute());
    leaves->hops.assign(n, std::vector<SdfPath>());
    leaves->varying.assign(n, 0);
    leaves->values.clear();
    leaves->values.reserve(n);
    for (size_t k = 0; k < n; ++k) {
        const RigExecRevisionLeafKey &key = leaves->decl.keys[k];
        if (stage && key.path.IsPropertyPath()) {
            leaves->attributes[k] = stage->GetAttributeAtPath(key.path);
        }
        bool varying = false;
        RigExecRevisionLeafHops(key, leaves->attributes[k], &leaves->hops[k],
                                &varying);
        leaves->varying[k] = varying ? 1 : 0;
        leaves->values.push_back(key.fallback);
    }
    leaves->changed.assign(n, 0);
    leaves->mustSample.assign(n, 0);
    leaves->sampled = false;
}

void
RigExecBakedSamplePathLeaves(RigExecBakedProgramImpl *program,
                             RigExecBakedPathLeaves *leaves, UsdTimeCode time,
                             bool all, const std::vector<int> &skip)
{
    RigExecBakedProgramImpl &B = *program;
    const bool overrides =
        B.interactiveOverrides && !B.interactiveOverrides->empty();
    all = all || !leaves->sampled || leaves->stamp != B.programStamp;
    const bool overridden = overrides || leaves->overrides;
    const bool chainsMoved = leaves->chainSerial != B.pathLeafChainSerial;
    const bool timeMoved = !leaves->sampled || time != leaves->time;
    // Default and a numeric time read different opinions of an attribute
    // that holds both, whatever its variance.
    const bool defaultMoved =
        timeMoved && (time.IsDefault() || leaves->time.IsDefault());
    std::vector<SdfPath> hops;
    for (size_t k = 0; k < leaves->decl.keys.size(); ++k) {
        leaves->changed[k] = 0;
        if (std::find(skip.begin(), skip.end(), int(k)) != skip.end()) {
            leaves->mustSample[k] = 1;
            continue;
        }
        const RigExecRevisionLeafKey &key = leaves->decl.keys[k];
        const bool rebind = all || leaves->mustSample[k];
        const bool atTime = key.time == RigExecRevisionLeafTime::AtTime;
        if (!rebind && !overridden && !chainsMoved &&
            !(atTime && timeMoved && (leaves->varying[k] || defaultMoved))) {
            continue;
        }
        if (rebind) {
            // An edit can author time samples where there were none.
            bool varying = false;
            RigExecRevisionLeafHops(key, leaves->attributes[k], &hops,
                                    &varying);
            leaves->varying[k] = varying ? 1 : 0;
        }
        leaves->mustSample[k] = 0;
        ++B.pathLeafSamples;
        // A read a chain or record can answer goes through its walk.
        const int walk = k < leaves->walks.size() ? leaves->walks[k] : -1;
        VtValue value =
            walk >= 0 ? RigExecBakedSampleWalkedPathLeaf(B, key, walk)
                      : RigExecSampleRevisionLeaf(key, leaves->attributes[k],
                                                  B.resolvedInputs, time,
                                                  &B.upstream);
        leaves->changed[k] = value == leaves->values[k] ? 0 : 1;
        leaves->values[k] = std::move(value);
        if (walk >= 0 && leaves->changed[k]) {
            B.readerWalkChanged[size_t(walk)] = 1;
        }
    }
    leaves->sampled = true;
    leaves->time = time;
    leaves->stamp = B.programStamp;
    leaves->overrides = overrides;
    leaves->chainSerial = B.pathLeafChainSerial;
}

// The skin layouts.

RigExecBakedProgramImpl::GeomRevision *
RigExecBakedLayoutRevision(RigExecBakedProgramImpl *program, size_t r)
{
    RigExecBakedProgramImpl &B = *program;
    if (r < B.revisionIndex.size()) {
        const auto &[chain, revision] = B.revisionIndex[r];
        return &B.chains[size_t(chain)].revisions[size_t(revision)];
    }
    r -= B.revisionIndex.size();
    if (r < B.derivedIndex.size()) {
        const auto &[chain, derived] = B.derivedIndex[r];
        return &B.chains[size_t(chain)].derived[size_t(derived)].revision;
    }
    return nullptr;
}

const RigExecBakedProgramImpl::GeomRevision *
RigExecBakedLayoutRevision(const RigExecBakedProgramImpl &program, size_t r)
{
    return RigExecBakedLayoutRevision(
        const_cast<RigExecBakedProgramImpl *>(&program), r);
}

void
RigExecBakedBuildLayoutSteps(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    const size_t count = B.revisionIndex.size() + B.derivedIndex.size();
    for (size_t r = 0; r < count; ++r) {
        RigExecBakedProgramImpl::GeomRevision &revision =
            *RigExecBakedLayoutRevision(&B, r);
        if (!revision.skinTopologyFixed) {
            continue;
        }
        revision.layoutLeaves = RigExecBakedPathLeaves();
        RigExecDeclareSkinLayoutLeaves(revision.moverPath,
                                       &revision.layoutLeaves.decl);
        RigExecBakedBindPathLeaves(B.stage, &revision.layoutLeaves);
        revision.layoutOverlay.assign(revision.layoutLeaves.decl.keys.size(),
                                      VtValue());
        revision.layoutFixed = RigExecSkinLayoutIsFixed(revision.moverPrim);
        revision.layoutFixedChanged = false;
        revision.layoutRan = false;
        revision.layoutHandle = nullptr;
        RigExecBakedStep step;
        step.isHead = true;
        step.part = 0;
        step.kind = RigExecBakedStepKind::SkinTopology;
        step.object = int(r);
        step.label = "SkinTopology " + revision.moverPath.GetString();
        step.writes.push_back(RigExecBakedOne(
            RigExecBakedSlotDomain::SkinTopology, uint32_t(r)));
        B.steps.push_back(std::move(step));
    }
}

void
RigExecBakedDeclareLayoutReads(RigExecBakedProgramImpl *program)
{
    RigExecBakedProgramImpl &B = *program;
    for (RigExecBakedStep &step : B.steps) {
        size_t r = 0;
        switch (step.kind) {
        case RigExecBakedStepKind::RevisionStatic:
        case RigExecBakedStepKind::RevisionChunk:
        case RigExecBakedStepKind::RevisionFuse:
            r = size_t(step.object);
            break;
        case RigExecBakedStepKind::Derived:
            r = B.revisionIndex.size() + size_t(step.object);
            break;
        default:
            continue;
        }
        const RigExecBakedProgramImpl::GeomRevision *revision =
            RigExecBakedLayoutRevision(B, r);
        if (revision && revision->skinTopologyFixed) {
            step.reads.push_back(RigExecBakedOne(
                RigExecBakedSlotDomain::SkinTopology, uint32_t(r)));
        }
    }
}

void
RigExecBakedSampleLayoutLeaves(RigExecBakedProgramImpl *program,
                               RigExecBakedProgramImpl::GeomRevision *revision,
                               UsdTimeCode time, bool all)
{
    RigExecBakedProgramImpl &B = *program;
    RigExecBakedPathLeaves &leaves = revision->layoutLeaves;
    const size_t n = leaves.decl.keys.size();
    revision->layoutFixedChanged = false;
    std::fill(leaves.changed.begin(), leaves.changed.end(), 0);
    bool rebind = all || !leaves.sampled || leaves.stamp != B.programStamp;
    bool overlayMoved = false;
    for (size_t k = 0; k < n; ++k) {
        rebind = rebind || leaves.mustSample[k];
        // What the read consults before the stage, compared by value: an
        // absent entry is held as an empty value.
        const VtValue *entry =
            B.resolvedInputs ? B.resolvedInputs->Find(leaves.decl.keys[k].path)
                             : nullptr;
        const bool same = entry ? revision->layoutOverlay[k] == *entry
                                : revision->layoutOverlay[k].IsEmpty();
        if (!same) {
            overlayMoved = true;
            revision->layoutOverlay[k] = entry ? *entry : VtValue();
        }
    }
    if (!rebind && !overlayMoved) {
        return;
    }
    if (rebind) {
        // An edit can author time samples or a connection, which the
        // evaluator's cache would refuse on its next refill.
        const bool fixed = RigExecSkinLayoutIsFixed(revision->moverPrim);
        revision->layoutFixedChanged = fixed != revision->layoutFixed;
        revision->layoutFixed = fixed;
        std::fill(leaves.mustSample.begin(), leaves.mustSample.end(), 0);
    }
    if (revision->layoutFixed) {
        for (size_t k = 0; k < n; ++k) {
            ++B.pathLeafSamples;
            VtValue value = RigExecSampleRevisionLeaf(
                leaves.decl.keys[k], leaves.attributes[k], B.resolvedInputs,
                time, &B.upstream);
            leaves.changed[k] = value == leaves.values[k] ? 0 : 1;
            leaves.values[k] = std::move(value);
        }
    }
    leaves.sampled = true;
    leaves.time = time;
    leaves.stamp = B.programStamp;
}

void
RigExecBakedRunLayoutOp(RigExecBakedProgramImpl::GeomRevision *revision)
{
    if (!revision->layoutFixed) {
        // The cache's refusal: the packet reads the arrays per frame. The
        // candidate stays, so the layout comes back as the same object.
        revision->layoutHandle = nullptr;
        return;
    }
    using Role = RigExecRevisionLeafRole;
    const RigExecBakedPathLeaves &leaves = revision->layoutLeaves;
    const VtIntArray indices = leaves.Value<VtIntArray>(
        leaves.decl.Role(Role::JointIndices), VtIntArray());
    const VtFloatArray weights = leaves.Value<VtFloatArray>(
        leaves.decl.Role(Role::JointWeights), VtFloatArray());
    const int elementSize =
        leaves.Value<int>(leaves.decl.Role(Role::ElementSize), 1);
    auto built = std::make_shared<RigExecSkinTopology>();
    RigExecBuildSkinTopology(
        TfSpan<const int>(indices.cdata(), indices.size()),
        TfSpan<const float>(weights.cdata(), weights.size()), elementSize,
        revision->influenceSlots.size(), built.get());
    // The same layout keeps its object, so the packet that carries it still
    // compares equal and the kernel does not re-run.
    if (revision->layoutCandidate && *revision->layoutCandidate == *built) {
        revision->layoutHandle = revision->layoutCandidate;
        return;
    }
    revision->layoutHandle = std::move(built);
    revision->layoutCandidate = revision->layoutHandle;
}

void
RigExecBakedRunLayoutTier(RigExecBakedProgramImpl *program, UsdTimeCode time,
                          RigExecRigPose *pose, bool force, bool sample,
                          bool verify)
{
    RigExecBakedProgramImpl &B = *program;
    // After the property revisions and rest ops this run executed, in the
    // head trace.
    uint32_t seq = 0;
    for (const RigExecBakedStep &step : B.steps) {
        if (step.kind != RigExecBakedStepKind::SkinTopology) {
            seq = std::max(seq, step.runSeq);
        }
    }
    for (size_t index = 0; index < B.steps.size() && B.steps[index].isHead; ++index) {
        RigExecBakedStep &step = B.steps[index];
        if (step.kind != RigExecBakedStepKind::SkinTopology) {
            continue;
        }
        step.runSeq = 0;
        RigExecBakedProgramImpl::GeomRevision *revision =
            RigExecBakedLayoutRevision(&B, size_t(step.object));
        if (!revision) {
            continue;
        }
        if (sample) {
            RigExecBakedSampleLayoutLeaves(&B, revision, time, force);
        }
        bool moved = revision->layoutFixedChanged;
        for (const char changed : revision->layoutLeaves.changed) {
            moved = moved || changed;
        }
        if (!(force || moved || !revision->layoutRan)) {
            continue;
        }
        RigExecBakedRunStepBodyAndStamp(&B,&step,time);
        ++B.headOpsRun;
    }
    if (!verify || !sample) {
        return;
    }
    // Every handle against the layout the stage and the overlay describe
    // now, read afresh: a leaf route that missed an edit shows here.
    size_t mismatches = 0;
    for (size_t index = 0; index < B.steps.size() && B.steps[index].isHead; ++index) {
        const RigExecBakedStep &step = B.steps[index];
        if (step.kind != RigExecBakedStepKind::SkinTopology) {
            continue;
        }
        const RigExecBakedProgramImpl::GeomRevision *revision =
            RigExecBakedLayoutRevision(&B, size_t(step.object));
        if (!revision) {
            continue;
        }
        RigExecBakedProgramImpl::GeomRevision fresh;
        fresh.moverPrim = revision->moverPrim;
        fresh.influenceSlots = revision->influenceSlots;
        fresh.layoutLeaves.decl = revision->layoutLeaves.decl;
        fresh.layoutLeaves.attributes = revision->layoutLeaves.attributes;
        fresh.layoutFixed = RigExecSkinLayoutIsFixed(revision->moverPrim);
        fresh.layoutLeaves.values.clear();
        for (size_t k = 0; k < fresh.layoutLeaves.decl.keys.size(); ++k) {
            fresh.layoutLeaves.values.push_back(RigExecSampleRevisionLeaf(
                fresh.layoutLeaves.decl.keys[k],
                fresh.layoutLeaves.attributes[k], B.resolvedInputs, time,
                &B.upstream));
        }
        RigExecBakedRunLayoutOp(&fresh);
        const auto &held = revision->layoutHandle;
        const auto &want = fresh.layoutHandle;
        if ((!held && !want) || (held && want && *held == *want)) {
            continue;
        }
        ++mismatches;
        pose->diagnostics.push_back("baked cone mismatch: head skin topology "
                                    "of " +
                                    revision->moverPath.GetString() +
                                    " differs");
    }
    if (mismatches > 0) {
        pose->bakedParityMismatches += mismatches;
        TF_WARN("rigExec: baked parity mismatch: %zu skin layout(s) the "
                "SkinTopology ops hold differ from the stage's",
                mismatches);
    }
}

namespace {

// The packet fields two assemblies disagree on, by name.
std::string
PacketDifference(const RigExecMoverParameters &a,
                 const RigExecMoverParameters &b)
{
    std::string out;
    const auto field = [&out](bool same, const char *name) {
        if (!same) {
            out += out.empty() ? name : std::string(", ") + name;
        }
    };
    field(a.kind == b.kind, "kind");
    field(a.enabled == b.enabled, "enabled");
    field(a.valid == b.valid, "valid");
    field(a.transform == b.transform, "transform");
    field(a.radialWeight == b.radialWeight, "radialWeight");
    field(a.weights == b.weights, "weights");
    field(a.blendDeltas == b.blendDeltas, "blendDeltas");
    field(a.blendSurfaceFrame == b.blendSurfaceFrame, "blendSurfaceFrame");
    field(a.referenceVolume == b.referenceVolume, "referenceVolume");
    field(a.strength == b.strength, "strength");
    field(a.topologyCounts == b.topologyCounts, "topologyCounts");
    field(a.topologyIndices == b.topologyIndices, "topologyIndices");
    field(a.restPoints == b.restPoints, "restPoints");
    field(a.auxPoints == b.auxPoints, "auxPoints");
    field(a.bindCoords == b.bindCoords, "bindCoords");
    field(a.widths == b.widths, "widths");
    field(a.skinTransforms == b.skinTransforms, "skinTransforms");
    field(a.skinIndices == b.skinIndices, "skinIndices");
    field(a.skinWeights == b.skinWeights, "skinWeights");
    field(a.skinElementSize == b.skinElementSize, "skinElementSize");
    field(a.skinningMethod == b.skinningMethod, "skinningMethod");
    field(a.skinTopology == b.skinTopology, "skinTopology");
    return out.empty() ? std::string("another field") : out;
}

}  // namespace

std::vector<std::string>
RigExecBakedProgramTesting::ShadowAssembly(
    const RigExecBakedProgram &program, const RigExecResolvedInputs &resolved,
    UsdTimeCode time,
    const std::function<void(const SdfPath &, RigExecRevisionLeafDecl *)>
        &edit,
    size_t *compared)
{
    RigExecBakedProgramImpl &B = *program._impl;
    std::vector<std::string> report;
    size_t count = 0;
    // Both assemblies over the caller's resolved inputs, each on a copy of
    // the revision, so the program keeps the state the run left.
    RigExecResolvedInputs local = resolved;
    RigExecResolvedInputs *const saved = B.resolvedInputs;
    B.resolvedInputs = &local;
    const auto shadow = [&](const RigExecBakedProgramImpl::GeomRevision &live,
                            const GfVec3f *base, size_t baseCount) {
        if (!live.leaves.decl.assembles) {
            return;
        }
        ++count;
        RigExecBakedProgramImpl::GeomRevision fromLeaves = live;
        RigExecBakedProgramImpl::GeomRevision fromStage = live;
        if (edit) {
            edit(live.moverPath, &fromLeaves.leaves.decl);
        }
        std::vector<std::string> missing, leafLines, stageLines;
        const RigExecMoverParameters a = RigExecBakedAssembleFromLeaves(
            &B, &fromLeaves, base, baseCount, &leafLines, &missing);
        const RigExecMoverParameters b = AssembleRevisionFromStage(
            B, local, &fromStage, base, baseCount, time, &stageLines,
            /*record=*/false);
        const std::string where = live.moverPath.GetString();
        for (const std::string &name : missing) {
            report.push_back(where + ": undeclared read " + name);
        }
        if (!(a == b)) {
            report.push_back(where + ": packets differ (" +
                             PacketDifference(a, b) + ")");
        }
        if (RigExecStatusForParameters(a, live.moverPath) !=
            RigExecStatusForParameters(b, live.moverPath)) {
            report.push_back(where + ": statuses differ");
        }
        if (leafLines != stageLines) {
            report.push_back(where + ": diagnostic lines differ");
        }
    };
    // A projector's matrix target: its reads from leaves and off the stage.
    const auto projector =
        [&](const RigExecBakedProgramImpl::GeomRevision &live) {
            ++count;
            RigExecBakedProgramImpl::GeomRevision fromLeaves = live;
            if (edit) {
                edit(live.moverPath, &fromLeaves.leaves.decl);
            }
            std::vector<std::string> missing;
            RigExecRevisionLeafView view;
            view.decl = &fromLeaves.leaves.decl;
            view.values = &fromLeaves.leaves.values;
            view.missing = &missing;
            RigExecProjectorReads a, b;
            RigExecReadProjectorTargetFromLeaves(live.op, live.binding, view,
                                                 &a);
            RigExecReadProjectorTarget(live.moverPrim, live.op, live.binding,
                                       &local, time, &b);
            const std::string where = live.moverPath.GetString();
            for (const std::string &name : missing) {
                report.push_back(where + ": undeclared read " + name);
            }
            if (a.rayOrigin != b.rayOrigin ||
                a.rayDirection != b.rayDirection || a.rayUp != b.rayUp ||
                a.shaderOffset != b.shaderOffset ||
                a.reproject != b.reproject || a.dials != b.dials ||
                a.faceVertexCounts != b.faceVertexCounts ||
                a.faceVertexIndices != b.faceVertexIndices) {
                report.push_back(where + ": projector reads differ");
            }
        };
    for (const RigExecBakedProgramImpl::GeomChain &chain : B.chains) {
        if (!chain.haveBase) {
            continue;
        }
        for (const RigExecBakedProgramImpl::GeomRevision &revision :
             chain.revisions) {
            shadow(revision, chain.lastBase.cdata(), chain.lastBase.size());
        }
        if (!chain.haveResult) {
            continue;
        }
        for (const RigExecBakedProgramImpl::GeomChain::Derived &derived :
             chain.derived) {
            if (!derived.matrixTarget && derived.haveBase) {
                shadow(derived.revision, chain.result.cdata(),
                       chain.result.size());
            }
            if (derived.matrixTarget) {
                projector(derived.revision);
            }
        }
    }
    B.resolvedInputs = saved;
    if (compared) {
        *compared = count;
    }
    return report;
}

}  // namespace rigExec
