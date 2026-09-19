//
// RigExec rig evaluator, v0.1-alpha.
//
// Transforms and solvers evaluate through OpenExec; the geometry mover
// chains evaluate through the in-memory RigExecMoverGraph. NOTHING is
// authored: the engine has no compiler, no generated prims, and no derived
// evaluation stage (spec §7.2 revised — the source stage is never written).
// A staged CPU implementation of the same kernels over the same composed
// reverse-sibling post-order walk (spec §4.2) is retained behind
// cpuParityMode as the scalar parity reference.
//
#ifndef RIGEXEC_RIG_EVALUATOR_H
#define RIGEXEC_RIG_EVALUATOR_H

#include "bakedProgram.h"
#include "moverGraph.h"
#include "profiler.h"
#include "tapSet.h"
#include "types.h"

#include "rigExecMath/rbf.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/tf/functionRef.h"
#include "pxr/base/tf/notice.h"
#include "pxr/base/tf/weakBase.h"
#include "pxr/base/vt/array.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/usd/notice.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usdGeom/xformCache.h"

#include <list>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <string>
#include <vector>

namespace rigExec {

/// One discovered mover application (spec §4.2): the reverse-sibling
/// post-order ordinal plus canonicalized targets.
struct RigExecMoverRecord {
    SdfPath moverPath;
    TfToken schemaType;
    std::vector<SdfPath> targets;  ///< canonicalized (prim -> .points etc.)
    int ordinal = 0;
    bool enabledFallback = true;
};

/// One weight object's resolved field, as a mover actually consumed it.
///
/// Dense and already range-policed, so a consumer can index it by element
/// without knowing whether the field came from an authored table, a
/// driven modulation, a placed volume, or a composition of those.
struct RigExecResolvedWeightField {
    SdfPath target;              ///< canonical points property weighted
    std::vector<float> weights;  ///< one per logical element
};

/// One evaluated generation of a rig.
struct RigExecRigPose {
    UsdTimeCode time = UsdTimeCode::Default();
    bool valid = false;

    /// Joint path -> frame straight from the exec solver network (base,
    /// before pose-domain movers).
    std::map<SdfPath, RigExecPointFrame> jointFramesBase;
    /// Joint path -> frame after hierarchy-ordered pose movers (final).
    std::map<SdfPath, RigExecPointFrame> jointFramesFinal;
    /// Joint path -> final rest->pose affine map (target-local).
    std::map<SdfPath, GfMatrix4d> jointMatricesFinal;

    /// Control path -> posed frame, ASSET-space like the joint frames.
    ///
    /// Normally this is the BASE phase because controls are animator inputs.
    /// If a pose constraint explicitly names a control in rigExec:moves, the
    /// revised frame is published instead, matching FBX's ability to
    /// constrain any transform object. Consumed by the imaging bridge to
    /// place synthesized control guides (spec §10.3 extension).
    std::map<SdfPath, RigExecPointFrame> controlFrames;

    /// Transform provider -> revised ASSET-SPACE matrix for the prim.
    ///
    /// For a provider that is a plain UsdGeomXformable (not a Joint or
    /// Control), a constraint revises its transform and geometry parented
    /// underneath rides along. That is the correct model for rigid objects:
    /// one matrix instead of point-deforming N vertices at constant weight 1.
    ///
    /// Relative to the ASSET ROOT (the rig prim's parent) -- neither
    /// local-to-parent nor local-to-world. Joint and control frames are
    /// authored rest:space plus avars and carry no stage placement, so this
    /// is the space they all share; solving against a target in any other
    /// space subtracts mismatched origins.
    std::map<SdfPath, GfMatrix4d> providerXforms;

    /// Transform provider -> the ASSET-SPACE matrix the revision was
    /// computed from.
    ///
    /// Paired one-to-one with providerXforms. Consumers need both: the
    /// change is only usable as a delta, because the imaging chain's own
    /// notion of a prim's world transform need not equal the stage's (a
    /// native-instancing prototype resolves to prototype-common space).
    /// Keyed identically, so a lookup that finds one finds the other.
    std::map<SdfPath, GfMatrix4d> providerBaseXforms;

    /// Aggregate solver path -> posed frame elements straight from the
    /// solver's computePointFrameArray (guide drawing and inspection).
    std::map<SdfPath, std::vector<RigExecPointFrame>> solverFrames;

    /// Exact property path -> final computed native value, in the
    /// property's own type.
    ///
    /// Two kinds of chain land here, and a consumer tells them apart by the
    /// held type rather than by a flag:
    ///   - point chains (points/normals/extent) as VtVec3fArray, computed by
    ///     the compiled mover graph (spec §7.2);
    ///   - property chains (float, GfVec3f, GfMatrix4d) from the statically
    ///     typed math movers, computed off the authored stage and also
    ///     supplied to exec as value overrides so consumers see them.
    ///
    /// Movers are not joint-only and never were: this map, providerXforms,
    /// and jointFramesFinal are three peer output domains, and a rig may
    /// publish any combination of them.
    std::map<SdfPath, VtValue> movedProperties;

    /// CPU reference-kernel results for the same chains, filled only when
    /// RigExecRigEvaluator::cpuParityMode is set (scalar-reference
    /// parity, spec §7.4).
    std::map<SdfPath, VtValue> movedPropertiesCpu;

    /// Resolved weight field of every weight object a mover consumed this
    /// generation, keyed by the weight object's prim path.
    ///
    /// This is what an authoring tool paints as an influence overlay: a
    /// weight object is otherwise invisible, and a rigger placing a
    /// volume needs to see the region it actually grabs rather than
    /// infer it from where the geometry ends up. Filled from the packets
    /// the movers already resolved, so it costs a copy and no extra
    /// evaluation.
    std::map<SdfPath, RigExecResolvedWeightField> weightFields;

    /// Volumetric weight object path -> the ASSET-SPACE placement matrix
    /// this generation resolved for it, for every volume reachable in
    /// the current epoch.
    ///
    /// The companion to weightFields, and published for the same
    /// consumer: an authoring tool that paints the influence overlay
    /// also has to DRAW the volume, and a falloff iso-surface can only be
    /// drawn in the space the field was measured in. Taken from the
    /// volume's own computeMatrix tap rather than left to the consumer to
    /// re-derive, because a second hand-rolled composition of
    /// posed:space + rest offsets + avars + rotation order is exactly the
    /// drift frameExtraction.h exists to prevent -- the same reasoning
    /// that made _ResolveVolumeWeights read this map instead of
    /// recomputing it.
    std::map<SdfPath, GfMatrix4d> weightFrames;

    /// Pass-through, failure, and unimplemented-operation reports
    /// (spec §6.6: disabled/failed movers pass through with diagnostics).
    std::vector<std::string> diagnostics;

    /// Independent scalar-reference comparisons, when cpuParityMode is on.
    /// Both are zero when no reference checks ran; callers must check the
    /// agreement count to distinguish that from a verified generation.
    size_t moverGraphParityMismatches = 0;
    size_t moverGraphParityAgreements = 0;

    /// Disagreements between the baked program and the dynamic path, when
    /// the evaluation mode is BakedWithParityCheck. Zero in every other
    /// mode, including Baked -- nothing compared is not the same fact as
    /// nothing differed, so read it beside GetEvaluationMode.
    size_t bakedParityMismatches = 0;

    /// Dependency levels evaluated to resolve solver->joint overrides.
    /// Retains the public name used by clients of the former fixed-point
    /// evaluator; resolution now follows the compiled DAG once per level.
    size_t solverOverrideRounds = 0;
    bool solverOverridesConverged = true;
    /// Aggregate solver computations requested by the dependency schedule.
    size_t solverEvaluations = 0;

    /// Work performed by the persistent geometry graphs this generation.
    /// Unchanged inputs execute no revisions and rebuild no schedules.
    size_t moverGraphRevisionsCreated = 0;
    size_t moverGraphRevisionsExecuted = 0;
    size_t moverGraphSchedulesBuilt = 0;
};

/// One provider, as the pose walk currently holds it: the frame it was
/// seeded with and the frame it carries after every revision committed so
/// far.
///
/// The dynamic walk keeps its frames in ordered maps and the baked program
/// keeps them in dense slot arrays, so the routines the two paths share take
/// the frame store as a visitor rather than as a container -- one body, two
/// stores, and no copy of the store into a third shape to call it.
/// TfFunctionRef borrows the callable, so nothing is allocated per frame and
/// nothing outlives the call.
using RigExecPoseFrameVisitor =
    TfFunctionRef<void(const SdfPath &provider,
                       const RigExecPointFrame &base,
                       const RigExecPointFrame &current)>;

/// Calls \p visit once for every provider the walk holds, in the walk's own
/// order. A provider the walk has no base frame for is not visited: it can
/// carry no revision delta, which is all a visitor of this shape asks about.
using RigExecPoseFrameEnumerator =
    TfFunctionRef<void(const RigExecPoseFrameVisitor &visit)>;

/// Answers the FINAL frame of one provider; false when the walk holds none.
using RigExecPoseFrameLookup =
    TfFunctionRef<bool(const SdfPath &provider, RigExecPointFrame *frame)>;

/// Rides \p frame on the revision of the deepest provider above \p xformPath
/// that the walk has already moved.
///
/// A frame read off the stage knows nothing about what the pose walk has done
/// to the transforms ABOVE it. The closest revised ancestor's delta already
/// contains every higher ancestor's, so applying that one delta applies all
/// of them. Which ancestor that is -- a STRICT namespace prefix, whose points
/// actually moved, deepest wins -- and the delta itself are here, in one
/// body, so that the dense baked program and the map walk cannot pick
/// different ancestors or measure different deltas. \p providers enumerates
/// whatever frame store the caller keeps.
///
/// False only when the delta will not resolve or the result is unusable; a
/// walk that has revised nothing above \p xformPath leaves \p frame alone
/// and returns true.
bool RigExecApplyRevisedAncestorDelta(
    const SdfPath &xformPath,
    const RigExecPoseFrameEnumerator &providers,
    RigExecPointFrame *frame);

/// Rebuilds a SingleChainIK input chain from its rest layout.
///
/// `neverTS` retains current rotations and root placement while rebuilding
/// child placement and handle lengths from the rest frames. A joint with no
/// authored rest transform carries the schema's identity fallback, which is
/// not a chain rest layout at all; its current static layout stands in.
///
/// Pure frame math over two chains, which is why it is a free function and
/// not a member: the solver keeps measuring its input chain, and the
/// preparation decides only whether those measurements are rest- or
/// animation-derived. Both the dynamic walk and the baked program's
/// constraint step call it, and a step body cannot reach a private member of
/// the evaluator.
bool RigExecPrepareRestDerivedIkChain(
    const std::vector<RigExecPointFrame> &current,
    const std::vector<RigExecPointFrame> &rest,
    std::vector<RigExecPointFrame> *prepared);

/// Compiles and evaluates one RigExecRoot prim.
/// The pinned reads a property chain re-uses every frame (rigEvaluator.cpp).
///
/// Opaque here on purpose: it holds one UsdAttributeQuery per input the
/// chains read, which is a value-resolution cache and therefore something
/// only the routine that fills it should be able to reach.
struct RigExecPropertyChainBindings;

class RigExecRigEvaluator : public TfWeakBase {
public:
    RigExecRigEvaluator(const UsdStageRefPtr &stage, const SdfPath &rigPath);
    ~RigExecRigEvaluator();

    /// Discovers joints and movers, validates targets, and prepares the
    /// exec tap set. Returns false with messages on validation failure.
    bool Compile(std::vector<std::string> *errors = nullptr);

    /// Evaluates one complete generation at an explicit time.
    RigExecRigPose Evaluate(UsdTimeCode time);

    /// Which path Evaluate takes; see RigExecEvaluationMode.
    ///
    /// Baked is a REQUEST. Compile builds the program only for an epoch the
    /// program can express, and Evaluate falls back to the dynamic path
    /// whenever there is no program, so setting this can never change an
    /// answer -- only how fast it arrives. Setting it on a compiled rig
    /// builds the program immediately when the epoch is clean, and on the
    /// next Evaluate otherwise -- a scene edit standing between the compile
    /// and the request must not leave the mode asking for a program that is
    /// never built.
    ///
    /// RIGEXEC_BAKE_REQUIRED=1 in the environment makes that fallback SAY so.
    /// A suite whose fixtures all decline the bake reports zero parity
    /// mismatches and goes green having compared nothing, which is the one
    /// way a parity run can lie; with the variable set, a generation that
    /// runs dynamically while this mode asks for the program publishes one
    /// "baked parity mismatch: bake required, evaluated dynamically: <why>"
    /// diagnostic and counts it on RigExecRigPose::bakedParityMismatches.
    /// It changes no evaluated value, and Dynamic ignores it entirely. Read
    /// once per process, so a tool must setenv before the first evaluator.
    /// Calling this makes the caller the OWNER of the mode: the source
    /// below becomes Explicit and nothing weaker moves it again -- not the
    /// rig's own rigExec:baked, not a recompile, not a notice. That holds
    /// even when the mode asked for is the one already in force, because
    /// what the call settles is who decides, not only what was decided.
    void SetEvaluationMode(RigExecEvaluationMode mode);
    RigExecEvaluationMode GetEvaluationMode() const {
        return _evaluationMode;
    }

    /// Who chose the mode above; see RigExecEvaluationModeSource.
    ///
    /// Read it beside the mode whenever "is this rig baked" is not the whole
    /// question: a tool deciding whether it may set the mode, a test
    /// separating the environment's answer from the stage's, and the
    /// fallback report, which is loud for an asset that asked through its
    /// attribute and silent for an evaluator that inherited the mode from a
    /// session-wide variable.
    RigExecEvaluationModeSource GetEvaluationModeSource() const {
        return _evaluationModeSource;
    }

    /// Whether the compiled epoch can be baked, appending one reason per
    /// feature that stops it.
    ///
    /// A rig that will not bake should say WHICH of its features stopped it:
    /// "it fell back" is not actionable, and a silent fallback reads as the
    /// mode not working.
    bool IsBakeable(std::vector<std::string> *reasons = nullptr) const;

    /// Every constraint operator this evaluator registers, by schema type,
    /// in registration order.
    ///
    /// The handler table is the one authority on which operators exist. Any
    /// second enumeration of them -- the baked program's list of the ones it
    /// can express, a test asserting the table and the schema agree -- asks
    /// here instead of keeping a private copy, because a private copy is a
    /// list that silently stops naming an operator somebody added.
    static const std::vector<TfToken> &GetConstraintOperatorTypeNames();

    /// Whether \p schemaType names one of them.
    static bool IsConstraintOperatorType(const TfToken &schemaType);

    /// How many times a baked program has been built for this rig.
    ///
    /// Observable so a test can hold the invalidation contract to account:
    /// an edit that misses everything the bake captured must NOT move this,
    /// and one that hits it must.
    size_t GetBakedProgramBuildCount() const {
        return _bakedProgramBuilds;
    }

    /// How many generations the baked program has answered.
    ///
    /// Also observable for a test's sake, and for the same reason: a baked
    /// test that silently fell back would compare the dynamic path with
    /// itself and pass while proving nothing.
    size_t GetBakedGenerationCount() const {
        return _bakedGenerations;
    }

    /// How many times a build was ATTEMPTED, including the attempts that
    /// refused.
    ///
    /// The pair with the count above: a rig the program cannot express
    /// answers "no program" every time it is asked, and asking once per
    /// frame would charge every frame for the refusal. A test holds this to
    /// one attempt per epoch, which the build count alone cannot say
    /// because it never moves for such a rig at all.
    size_t GetBakedProgramBuildAttemptCount() const {
        return _bakedProgramBuildAttempts;
    }

    /// How many clusters the standing baked program holds, and how many of
    /// them its last generation ran. Both zero when there is no program.
    ///
    /// Cone re-execution is the one part of the program whose correctness
    /// cannot be read off a published pose: a frame that re-ran everything
    /// publishes the same numbers as one that skipped the right half, so a
    /// test asserting only on the pose would pass over a cone that never
    /// skipped anything. These make the skip itself observable.
    size_t GetBakedClusterCount() const;
    size_t GetBakedClustersRunLastGeneration() const;

    /// The standing baked program, or null where this generation is dynamic.
    ///
    /// For the suites that assert on the program's STRUCTURE -- what a
    /// drag's cone may reach, which entry holds which version -- against the
    /// graph rather than against a number somebody wrote down. Nothing in
    /// the library reads it.
    const RigExecBakedProgram *GetBakedProgram() const {
        return _bakedProgram.get();
    }

    /// How many skin layouts the epoch's topology cache is holding answers
    /// for.
    ///
    /// The other thing about an interactive generation that a published pose
    /// cannot show: dropping the layouts and re-reading them publishes
    /// exactly the same deformation as keeping them, so only the cache's own
    /// occupancy says whether a drag paid for the re-read. A test that
    /// overrides an unrelated control reads this to see that it did not.
    size_t GetSkinTopologyCacheSize() const;

    /// Values that stand in for authored attributes while they are set,
    /// with NOTHING authored: the manipulation path (spec: docs/superpowers/
    /// specs/2026-09-10-hydra-preview-manipulation-design.md).
    ///
    /// An interactive drag asks "what would the rig look like if this avar
    /// were 3.2", once per mouse sample. Authoring the answer to ask it
    /// invalidates exec through the authoring stage, rewrites a layer spec,
    /// and notifies every observer of the stage -- for a value the artist
    /// has not committed to. These overrides ask the same question through
    /// the route exec already provides for it (ExecUsdSystem::
    /// ComputeWithOverrides, tapSet.cpp), so the generation Hydra draws is
    /// the previewed one while the document still holds the authored value.
    ///
    /// Each override is delivered TWICE, because there are two ways a value
    /// reaches a consumer and they must not disagree: as an exec override
    /// for everything exec computes, and through _resolvedInputs for the
    /// property chains and the CPU oracle that exec never runs. An override
    /// replaces any earlier one on the same key rather than joining it, and
    /// outranks a property chain's result, which is the manipulator's edit
    /// winning over the rig's own arithmetic for as long as it is held.
    ///
    /// Setting them does not evaluate; the caller decides when to publish.
    void SetInteractiveOverrides(std::vector<RigExecValueOverride> overrides);

    /// Drops every interactive override. The next Evaluate is the authored
    /// rig again -- which is what a released or aborted drag wants, and the
    /// reason the manipulator never has to undo a preview.
    void ClearInteractiveOverrides();

    bool HasInteractiveOverrides() const {
        return !_interactiveOverrides.empty();
    }

    /// Whether each generation resolves RigExecRigPose::weightFields.
    ///
    /// The influence overlay is a per-POINT field: resolving it walks every
    /// point of every weighted mesh, every frame, whether or not anything is
    /// going to look at it. Exactly one consumer exists (the imaging bridge's
    /// _FillWeightOverlay, plus the Python weight_field binding), and the
    /// bridge's is off unless a rigger has selected a weight object -- so the
    /// bridge turns this off and the overlay costs nothing until it is asked
    /// for. Defaults to ON: a caller that never sets it (every test, every
    /// tool, the Python bindings) sees the field exactly as before.
    void SetPublishWeightFields(bool publish) {
        _publishWeightFields = publish;
    }
    bool GetPublishWeightFields() const { return _publishWeightFields; }

    /// Composed mover-stack applications: descendants before their mover
    /// parent, sibling branches in reverse composed child order (the bottom
    /// usdview row executes first; spec §4.2).
    const std::vector<RigExecMoverRecord> &GetMoverOrder() const {
        return _movers;
    }

    /// Structural digest of the compiled mover topology: the v0.1
    /// binding-epoch identity. Structural edits change it and trigger
    /// recompilation on the next Evaluate (spec §4.2, §6.3).
    size_t GetBindingEpochDigest() const { return _structureDigest; }

    /// How the compiled geometry chain walk is batched.
    ///
    /// A level is a run of chains that do not read one another, so the walk
    /// may hand each of them to its own task; the levels run one after
    /// another, in order, and concatenating them reproduces the compiled
    /// chain order exactly. A level a rig cannot safely spread out -- one
    /// that is too short to pay for the dispatch, or that holds a phased
    /// read, a Profile Mover, or two chains sharing a weight object -- says
    /// so here and is walked in order like any other.
    size_t GetChainLevelCount() const { return _chainLevels.size(); }

    /// How many providers the compiled epoch holds a constant rest frame
    /// for, or 0 when it declined to.
    ///
    /// Observable because the decision is the thing worth testing: a rig
    /// whose rest channels can move within the epoch must keep the per-frame
    /// rest taps, and a test that only compared numbers could pass on
    /// arithmetic that happened to coincide. Zero means the epoch declined
    /// -- every rest is pulled per frame with the pose.
    size_t GetEpochRestFrameCount() const { return _epochRestFrames.size(); }

    /// How many live geometry-chain graph nodes the evaluator holds.
    ///
    /// Compile pre-creates one per retained chain target so a level-parallel
    /// walk never inserts into the map it is reading. Observable so a test
    /// can assert that an Evaluate which ran a parallel level did not grow
    /// it -- the invariant that makes the walk memory-safe, which otherwise
    /// only a race would reveal.
    size_t GetLiveGraphCount() const { return _liveGraphs.size(); }

    /// The chain targets of one level, in walk order. Empty out of range.
    std::vector<SdfPath> GetChainLevelTargets(size_t level) const;

    /// Whether Compile classified \p level as safe to run one task per
    /// chain. False out of range, and false for every level when the rig
    /// has none.
    bool IsChainLevelParallel(size_t level) const;

    /// Aggregate solver path -> its dependency level in the compiled pose
    /// schedule. Level 0 holds solvers with no solver prerequisites; each
    /// other solver sits exactly one schedule wave above its deepest
    /// prerequisite. Diagnostic access for the level audit: evaluation order
    /// itself comes from the interleaved pose steps.
    std::map<SdfPath, size_t> GetSolverBatchLevels() const
    {
        std::map<SdfPath, size_t> levels;
        for (const _SolverBatch &batch : _solverBatches) {
            for (const auto &[solver, tap] : batch.solvers) {
                levels[solver] = batch.level;
            }
        }
        return levels;
    }

    /// Transform provider -> the pose steps that write it, in the order the
    /// pose walk runs them: the aggregate solvers that name it on
    /// rigExec:joints and the frame constraints that move it, INTERLEAVED in
    /// one hierarchical stack (spec §4.2). The last entry is the step whose
    /// frame the pose publishes; a solver entry is also what an AtPrim read
    /// phase naming that solver resolves against.
    ///
    /// Diagnostic access only -- evaluation order itself comes from the
    /// interleaved pose steps, and this is read back from them.
    const std::map<SdfPath, std::vector<SdfPath>> &GetFrameChains() const
    {
        return _frameChains;
    }

    /// When set, Evaluate verifies geometry against the CPU reference and
    /// publishes that reference in RigExecRigPose::movedPropertiesCpu.
    /// Disabled for interactive use so every deformation runs only once.
    bool cpuParityMode = false;

    /// When set, Compile and Evaluate record scoped phase timings (property
    /// chains, first-frame pose, each solver batch and constraint, the exec
    /// snapshot, each geometry chain, derived maintenance, parity) into the
    /// profiler. Disabled by default; enabling it does not change any
    /// evaluated value.
    void SetProfilingEnabled(bool enabled)
    {
        _profiler.SetEnabled(enabled);
    }

    bool GetProfilingEnabled() const
    {
        return _profiler.IsEnabled();
    }

    /// Observational solver-guide frames are viewport data: the imaging
    /// bridge draws them and Python exposes them through solver_frames, but
    /// no posed joint, matrix, or deformed point reads them. A headless
    /// consumer (batch export, benchmarking) that never reads
    /// RigExecRigPose::solverFrames disables them here and skips the whole
    /// guide request. Enabled by default, so existing callers see no change.
    void SetSolverGuidesEnabled(bool enabled)
    {
        _solverGuidesEnabled = enabled;
    }

    bool GetSolverGuidesEnabled() const
    {
        return _solverGuidesEnabled;
    }

    /// Advances once for every stage notice this evaluator receives.
    ///
    /// For a consumer that caches values read off the same stage across
    /// generations (the imaging bridge's guide styling): comparing the
    /// serial with the one it cached under is the whole invalidation, and it
    /// cannot miss an edit, because it is bumped by the very notice handler
    /// the evaluator's own caches are dropped from.
    uint64_t GetStageEditSerial() const { return _stageEditSerial; }

    /// The timing harness. Events accumulate across evaluations until
    /// ClearProfile, so one trace can hold a whole multi-frame scrub.
    const RigExecProfiler &GetProfiler() const
    {
        return _profiler;
    }

    /// Drops every recorded profile event. Does not change the enabled
    /// state.
    void ClearProfile()
    {
        _profiler.Clear();
    }

    /// Writes the accumulated events as Chrome Trace Event JSON (openable
    /// in Perfetto or chrome://tracing). Returns false with a message when
    /// the file cannot be written.
    bool WriteProfileTrace(const std::string &path,
                           std::string *error = nullptr) const
    {
        return _profiler.WriteChromeTrace(path, error);
    }

    /// The stage the rig evaluates against.
    ///
    /// This IS the source stage. It used to be a private derived stage
    /// carrying a generated sublayer, because compilation authored lowered
    /// property applications; nothing is authored any more, so there is
    /// nothing for a derived stage to hold.
    const UsdStageRefPtr &GetEvaluationStage() const { return _stage; }

private:
    bool _ValidateMatrixMover(
        const UsdPrim &prim,
        const RigExecMoverRecord &record,
        std::string *error) const;
    bool _ValidateSkinMover(
        const UsdPrim &prim,
        const RigExecMoverRecord &record,
        std::string *error) const;

    VtVec3fArray _EvaluateChain(
        const SdfPath &target,
        const std::vector<const RigExecMoverRecord *> &chain,
        const RigExecRigPose &pose,
        const std::map<SdfPath, GfMatrix4d> &baseProviderMatrices,
        const std::map<SdfPath, GfMatrix4d> &finalProviderMatrices,
        UsdTimeCode time,
        std::vector<std::string> *diagnostics,
        const std::map<SdfPath, GfMatrix4d> &geometryConstraintDeltas) const;

    /// CPU-side resolution of one weight object's field, the parity
    /// oracle's mirror of the exec computeWeightPacket kernels.
    ///
    /// \p currentPoints, when non-null, are the IN-FLIGHT points at the
    /// consuming operation's position in the mover stack. A volumetric
    /// weight whose rigExec:samplePhase is `current` measures against
    /// those; everything else ignores them and reads the authored base.
    /// Passing null where `current` was authored is an error rather than
    /// a silent fall back to the base, because the two fields differ and
    /// quietly publishing the wrong one is exactly the failure the
    /// parity harness exists to catch.
    bool _ResolveWeights(
        const SdfPath &weightPrimPath, size_t count, UsdTimeCode time,
        std::vector<float> *weights, std::string *error,
        const std::vector<GfVec3f> *currentPoints = nullptr) const;

    /// Fills \p layout from the UsdSkelBlendShape at \p blendShapePath.
    ///
    /// Returns false to REFUSE the epoch cache -- the shape can move within
    /// this epoch and must be read per frame. The admission rule is the one
    /// the skin layout uses, minus the half that cannot arise: `offsets` and
    /// `pointIndices` are declared `uniform`, so USD will not let them carry
    /// time samples at all, which leaves an authored connection as the only
    /// way the value can change under a standing epoch.
    ///
    /// A shape that is merely INVALID (offsets and indices of different
    /// lengths, an index outside the mesh) is cached as invalid rather than
    /// refused: it is a stable wrong answer, and re-reading it every frame to
    /// reach the same conclusion is the cost this cache exists to avoid.
    bool _ResolveBlendSampleLayout(
        const SdfPath &blendShapePath, size_t pointCount,
        RigExecBlendSampleLayout *layout) const;

    /// The volumetric half of _ResolveWeights: sphere, plane, curve, and
    /// the combine that folds them.
    bool _ResolveVolumeWeights(
        const UsdPrim &prim, size_t count, UsdTimeCode time,
        std::vector<float> *weights, std::string *error,
        const std::vector<GfVec3f> *currentPoints) const;

    /// Reads \p prim's points-bearing target relationship and returns its
    /// authored value at \p time. Accepts either an exact property path
    /// or a prim path canonicalizing to .points.
    bool _ReadTargetPoints(
        const UsdPrim &prim, const char *relationshipName, UsdTimeCode time,
        std::vector<GfVec3f> *points) const;

    /// Compiles if this rig never has, recompiles if a structural edit moved
    /// the epoch digest, and clears the structure-dirty flag. False when the
    /// compile failed, with \p diagnostics saying why.
    ///
    /// Hoisted out of the dynamic generation because the evaluation-mode
    /// dispatch has to run BEFORE that generation and cannot choose a path
    /// until the epoch has settled -- otherwise the first frame of every
    /// session, and the first after every structural edit, runs dynamically
    /// for no reason other than the order of two statements. Idempotent: the
    /// dynamic generation calls it again and it does nothing.
    bool _SettleEpoch(std::vector<std::string> *diagnostics);

    /// The dynamic generation: OpenExec plus the in-memory pose walk. This
    /// is Evaluate's whole body when the mode is Dynamic, the fallback
    /// whenever the program cannot run, and the reference the parity mode
    /// compares against. \p diagnostics seeds the published pose, so
    /// anything _SettleEpoch already said is said once.
    RigExecRigPose _EvaluateDynamic(UsdTimeCode time,
                                    std::vector<std::string> diagnostics = {});

    /// Applies the standing interactive overrides to \p resolved, replacing
    /// matching entries of \p published when one is given -- the same two
    /// applications _EvaluateDynamic performs around the property chains.
    ///
    /// The baked program runs the chains itself and needs the overrides at
    /// exactly those two points; it calls this rather than carrying a second
    /// copy of the ordering rule, because a second copy is a second answer.
    void _ApplyInteractiveOverridesToResolved(
        RigExecResolvedInputs *resolved,
        std::map<SdfPath, VtValue> *published) const;

    /// Builds the baked program for the current epoch, or drops it. No-op
    /// unless the mode asks for one and the epoch is settled.
    /// Rebuilds the baked program, handing the outgoing one's persistent
    /// geometry state to its replacement (RigExecBakedProgram::
    /// AdoptGeometryStateFrom). \p outgoing is passed rather than read off
    /// the member because Compile retires the program at its head, where a
    /// failure has to drop it, and rebuilds at its tail.
    void _RebuildBakedProgram(
        std::unique_ptr<RigExecBakedProgram> outgoing = nullptr);

    size_t _ComputeStructureDigest() const;
    void _OnObjectsChanged(const UsdNotice::ObjectsChanged &notice,
                           const UsdStageWeakPtr &sender);

    UsdStageRefPtr _stage;
    SdfPath _rigPath;
    /// Whether the requests only the DYNAMIC path pulls were left
    /// unprepared by Compile, to be prepared at first use instead.
    ///
    /// A baked session never pulls them: every runtime TapSet::Evaluate is
    /// inside _EvaluateDynamic, and a baked frame reaches none of them --
    /// so preparing them at Compile builds an exec network the session then
    /// never asks a question of. Set only when the session asked for the
    /// program outright (Baked); Dynamic needs them on the next frame and
    /// Parity pulls both paths every frame, so both keep preparing eagerly.
    ///
    /// What this moves, and what it does not: the three requests below are
    /// deferred, and only those. The rest request is not -- the program's
    /// own epoch rest frames come from it. Nor are the guide and
    /// connected-pose requests, whose FAILURE is load-bearing at compile:
    /// a guide request that will not prepare retires the guide taps, which
    /// the baked frame path reads, and a connected-pose provider is one of
    /// the things that refuses the bake.
    ///
    /// The cost of a failure moves with the work. Today a request that
    /// cannot be prepared fails the compile; deferred, it fails the first
    /// dynamic generation instead, with a diagnostic on the pose. A rig
    /// that bakes never reaches either.
    bool _execPrepDeferred = false;
    /// Prepares what _execPrepDeferred left, once. False, with \p pose told
    /// why, when a request will not prepare.
    bool _RealizeDeferredExecPrep(RigExecRigPose *pose);
    /// What this session's evaluation mode will be, asked without changing
    /// it: _RefreshAttributeEvaluationMode runs at the tail of Compile, and
    /// the deferral decision is made well before that.
    RigExecEvaluationMode _PeekEvaluationMode() const;
    std::unique_ptr<RigExecTapSet> _taps;
    /// Observational solver-guide taps in their own prepared request: a
    /// failing or unused aggregate solver degrades guide drawing with a
    /// diagnostic instead of invalidating the rig snapshot.
    std::unique_ptr<RigExecTapSet> _guideTaps;
    /// Last observational guide request. Guides reuse the cached snapshot
    /// only when the time, override tuple, and tap dirtiness all match; the
    /// taps stay observational, so a stale cache can only omit guides, but
    /// the epoch reset below keeps even that from surviving Compile.
    std::vector<RigExecValueOverride> _guideInputs;
    UsdTimeCode _guideTime = UsdTimeCode::Default();
    RigExecSnapshot _guideSnapshot;
    bool _guideDirty = true;
    bool _solverGuidesEnabled = true;

    std::vector<SdfPath> _jointPaths;
    /// Every RigExecControl beneath the rig, discovered exactly the way the
    /// joint outputs are (spec §4.1: the rig is a namespace root, not a
    /// manifest). Unlike the joints an empty set is legal -- a rig driven
    /// entirely by avars on its joints has no control prims at all -- so it
    /// never fails Compile.
    std::vector<SdfPath> _controlPaths;
    /// Base computePointFrame per control, parallel to _controlPaths.
    std::vector<RigExecTapId> _controlFrameTaps;
    /// Joint -> the ORDERED STACK of solvers that write it, each with the
    /// element of that solver's aggregate the joint takes. Held in memory
    /// rather than authored.
    ///
    /// This is what Pass 0 used to write onto each joint as
    /// rigExec:frameSource / rigExec:frameElement. It is a compile-time
    /// choice -- which solvers write this joint, in what order, and which
    /// element of each one's aggregate frame array is this joint's -- so it
    /// belongs to the compiled graph, not to the scene. Compile() derives it
    /// from each solver's ordered rigExec:joints, which the Phase A
    /// validation already walks; Evaluate() indexes each solver's frame
    /// array with it directly instead of going through the joint's
    /// computePointFrame.
    ///
    /// rigExec:joints is a WRITE, not an exclusive claim (spec §4.2,
    /// "Solvers stack"): any number of aggregate solvers may name one
    /// joint. The vector is in POSE-WALK order, so its LAST entry is the
    /// writer that supplies the joint's base frame and every earlier entry
    /// is a version a reader can still name. Almost every rig has exactly
    /// one entry per joint, and a one-element stack has no order to get
    /// wrong -- so every count() user of this map is unaffected by the
    /// stack and must stay a membership test.
    std::map<SdfPath, std::vector<std::pair<SdfPath, int>>> _jointSolverBinding;
    /// Small time-keyed LRU of authoritative exec snapshots.
    ///
    /// A frame is fully determined by (time, input overrides). Repeated
    /// evaluation of recently-seen frames should not re-pull the upstream
    /// network: a single-entry cache thrashed on every interleave between
    /// distinct frames, forcing full recomputation on each pass. This LRU
    /// holds a handful of (time, inputs, snapshot) entries so that a frame
    /// visited within the last few distinct-frame cycles hits instead of
    /// missing. Genuine stage edits set the caller's dirty flag, which
    /// vetoes the hit independently of this cache; the tap-level
    /// ConsumeDirty is no longer consulted here because exec's time/value
    /// callbacks fire across sibling frames that share the system and
    /// would spuriously invalidate the entry just computed.
    struct _SnapshotCache {
        static constexpr std::size_t capacity = 4;
        struct Entry {
            UsdTimeCode time;
            std::vector<RigExecValueOverride> inputs;
            RigExecSnapshot snapshot;
        };
        /// Most-recently-used first.
        std::list<Entry> lru;
        std::map<UsdTimeCode, std::list<Entry>::iterator> index;

        /// Returns the live entry whose time AND inputs match, or null.
        Entry *Find(const std::vector<RigExecValueOverride> &inputs,
                    UsdTimeCode time)
        {
            if (lru.empty()) return nullptr;
            auto it = index.find(time);
            if (it == index.end()) return nullptr;
            auto pos = it->second;
            Entry &e = *pos;
            if (e.inputs == inputs) {
                lru.splice(lru.begin(), lru, pos);
                return &e;
            }
            return nullptr;
        }

        void Store(std::vector<RigExecValueOverride> inputs, UsdTimeCode time,
                   const RigExecSnapshot &snap)
        {
            auto existing = index.find(time);
            if (existing != index.end()) {
                Entry &e = *existing->second;
                e.inputs = std::move(inputs);
                e.snapshot = snap;
                lru.splice(lru.begin(), lru, existing->second);
                return;
            }
            lru.push_front(Entry{time, std::move(inputs), snap});
            index[time] = lru.begin();
            if (lru.size() > capacity) {
                index.erase(lru.back().time);
                lru.pop_back();
            }
        }

        void Clear()
        {
            lru.clear();
            index.clear();
        }
    };
    /// Each dependency level evaluates once. Earlier aggregate and joint
    /// outputs are supplied as overrides, so downstream requests reuse them.
    struct _SolverBatch {
        std::unique_ptr<RigExecTapSet> taps;
        std::map<SdfPath, RigExecTapId> solvers;
        std::set<SdfPath> dependencies;
        std::set<SdfPath> frameInputs;
        /// The joints whose REST this batch's solver measures from, and the
        /// pose step that wrote each one just before it -- empty where none
        /// did, which means "pin the authored rest" (spec §4.2, "the incoming
        /// frame replaces the authored rest"). EMPTY on every rig with no
        /// pose step below a solver that writes one of its joints, and then
        /// the batch pushes no computeRestFrame override at all.
        std::map<SdfPath, SdfPath> restInputs;
        size_t level = 0;
        _SnapshotCache cache;
        RigExecSnapshot snapshot;
        bool dirty = true;
    };
    std::vector<_SolverBatch> _solverBatches;
    std::map<SdfPath, std::vector<std::pair<SdfPath, int>>> _solverJoints;
    /// Solver -> the solvers it reads, as Compile derived them before they
    /// were levelled into batches. The values are ALWAYS solvers: a targeted
    /// posed provider is resolved to the solver that binds its joint, so a
    /// provider path never appears here, and every value is also a key.
    ///
    /// The batches themselves carry the same edges, but only for the solvers
    /// that are IN one: a solver that binds no joint and drives no geometry
    /// is required by nothing, so it never reaches a batch and is evaluated
    /// only to publish its guides. The dynamic path hands that case to exec,
    /// which re-derives the order; the baked program has no exec to ask, so
    /// it orders its guide-only pass over these edges instead.
    std::map<SdfPath, std::set<SdfPath>> _solverDependencies;
    /// Authored input prim -> batches reading it. Override-only Exec requests
    /// do not re-arm repeated value-invalidation callbacks in this USD build.
    std::map<SdfPath, std::set<size_t>> _solverInputBatches;
    /// Interleaves dependency-ready aggregate batches with the authored
    /// constraint walk. Frame inputs to solvers consume the current pose.
    struct _PoseStep {
        bool solverBatch = false;
        size_t index = 0;
    };
    std::vector<_PoseStep> _poseSteps;
    /// Seed only transform providers before solving; geometry/aggregate taps
    /// are evaluated after the complete pose dependency schedule.
    std::unique_ptr<RigExecTapSet> _firstFramePoseTaps;
    _SnapshotCache _firstFramePoseCache;
    bool _firstFramePoseDirty = true;
    /// Authoritative snapshot (full exec network) cache. Vetoes a hit when
    /// _authSnapshotDirty, which the stage-notice handler sets on any edit.
    _SnapshotCache _authSnapshotCache;
    bool _authSnapshotDirty = true;
    /// Time-keyed authoritative snapshot cache. jointOverrides is a pure
    /// function of (epoch, time), so a repeat evaluation of the same frame
    /// reuses the snapshot without rebuilding the 800+ element override
    /// vector or re-hashing it. Cleared on any genuine stage edit.
    std::map<UsdTimeCode, RigExecSnapshot> _authSnapTimeKeyed;
    std::map<SdfPath, RigExecTapId> _firstFramePoseFrames;
    /// Per-frame rest taps, used only when some provider's rest inputs can
    /// vary with time; otherwise the rests are evaluated once per epoch into
    /// _epochRestFrames and this is empty (see _restTaps).
    std::map<SdfPath, RigExecTapId> _firstFramePoseRests;
    /// A provider's rest frame is a function of its rest channels and its
    /// ancestors', none of which move with time on a rig whose rests are not
    /// animated. Asking exec for all of them on every frame recomputes a
    /// constant: they are pulled once at Compile through this request, and
    /// refreshed only when an edit arrives that no recompile covered.
    std::unique_ptr<RigExecTapSet> _restTaps;
    std::map<SdfPath, RigExecTapId> _restTapIds;
    std::map<SdfPath, RigExecPointFrame> _epochRestFrames;
    /// The time the epoch's rests were pulled at, and the time the first-frame-pose
    /// request is warmed at: the stage's start time code, always a real
    /// frame rather than Default (see Compile).
    UsdTimeCode _restTime = UsdTimeCode(0.0);
    /// The frame each pose provider is anchored to -- its nearest RigExec
    /// ancestor, or an empty path for the asset root -- and the providers
    /// that have a plain Xformable standing between the two. Namespace
    /// topology, so it is settled once per epoch rather than per frame.
    std::map<SdfPath, SdfPath> _poseProviderAnchors;
    std::vector<SdfPath> _interveningXformProviders;
    /// Direct posed providers read by transform expressions, including
    /// parent:space reached through connected default-space expressions.
    std::map<SdfPath, std::set<SdfPath>> _poseProviderInputs;
    std::map<SdfPath, std::unique_ptr<RigExecTapSet>> _connectedPoseTaps;
    struct _ConnectedPoseResult {
        std::vector<RigExecValueOverride> inputs;
        UsdTimeCode time = UsdTimeCode::Default();
        RigExecPointFrame base;
        RigExecPointFrame current;
        bool cached = false;
    };
    std::map<SdfPath, _ConnectedPoseResult> _connectedPoseCache;
    /// Namespace-pose predicate answers that are stage-constant: parent:space
    /// has no time samples, so the answer does not vary with evaluation time.
    /// Populated lazily on cache miss; cleared on epoch change with
    /// _connectedPoseCache. A time-sampled parent:space stays out of this
    /// cache and is re-read every frame.
    std::unordered_map<SdfPath, bool, SdfPath::Hash> _namespaceInheritsCache;
    /// Nearest pose-owning ancestor-or-self per provider path. The owner
    /// mapping depends only on epoch-static structure (_jointSolverBinding,
    /// _hierarchicalProviderSet, stage-constant namespace-pose answers), so
    /// it is valid across every evaluation until the epoch rebuilds.
    /// Cleared on epoch change with _namespaceInheritsCache.
    std::unordered_map<SdfPath, SdfPath, SdfPath::Hash> _nearestBlockingCache;
    /// Hierarchical (first-frame-pose) providers, compiled once per epoch.
    /// Replaces the per-frame std::set build in _EvaluateDynamic.
    std::unordered_set<SdfPath, SdfPath::Hash> _hierarchicalProviderSet;
    std::vector<RigExecTapId> _jointFrameTaps;
    std::vector<RigExecTapId> _jointFinalFrameTaps;
    std::vector<RigExecTapId> _jointFinalMatrixTaps;
    std::map<SdfPath, RigExecTapId> _solverArrayTaps;

    /// One compiled RigExecPoseInterpolator: its solved RBF constant, its
    /// driver, and the weight property of every pose it publishes.
    ///
    /// NOT A MOVER, and it cannot be one. A mover's inputs are resolved by
    /// the property chains, which run before exec does and therefore cannot
    /// see the pose; an interpolator reads the FINAL pose of a driver joint
    /// and writes floats that the geometry chains then consume. So it is its
    /// own phase of _EvaluateDynamic, sitting between the two.
    ///
    /// The solve -- inverting the matrix of every pose's kernel value at
    /// every other pose -- is a constant of the authored data, so it happens
    /// once here and the per-frame work is one kernel row, a multiply and a
    /// divide (libs/rigExecMath/rbf.h). Everything that constant is a
    /// function of is hashed into the epoch digest, or an edit to a pose
    /// rotation would leave a stale inverse behind.
    struct _PoseInterpolator {
        SdfPath prim;
        SdfPath driver;
        /// The driver's namespace ancestor that publishes a frame, or empty
        /// when there is none and the driver's local rotation is its world
        /// one. Resolved at compile: namespace topology does not move.
        SdfPath driverParent;
        bool allowNegativeWeights = true;
        /// <pose>.outputs:weight for the poses that are in the solve, in the
        /// solver's own index order.
        std::vector<SdfPath> poseWeights;
        /// The same for poses whose inputs:enabled is off. A disabled pose is
        /// left out of the solve entirely (schema: leaving it in would keep
        /// it in every other pose's matrix row) and publishes a hard zero.
        std::vector<SdfPath> disabledPoseWeights;
        RigExecRbfSolver solver;
    };
    std::vector<_PoseInterpolator> _poseInterpolators;
    /// Every weight property the phase publishes, flat and in publish order.
    /// Read by the ordering assertion at the head of the geometry chains.
    std::vector<SdfPath> _poseWeightProperties;

    /// Discovers, validates and SOLVES the rig's pose interpolators into
    /// \p out. Phase A of Compile: nothing here touches evaluator state.
    bool _CompilePoseInterpolators(
        const std::vector<SdfPath> &joints,
        const std::vector<SdfPath> &controls,
        std::vector<_PoseInterpolator> *out,
        std::vector<std::string> *notes,
        std::string *error) const;

    /// The pose-interpolator phase: one weight per pose, into
    /// _resolvedInputs and into \p pose->movedProperties.
    void _EvaluatePoseInterpolators(
        UsdTimeCode time,
        const std::map<SdfPath, RigExecPointFrame> &restFrames,
        const std::map<SdfPath, RigExecPointFrame> &finalFrames,
        RigExecRigPose *pose);

    /// Compiled mover-graph input bindings.
    ///
    /// The revision bindings are pure path resolution off the authored stage
    /// (they replace the rigExec:resolved* relationships the compiler used to
    /// author), so they are compile-time state. The taps supply the two
    /// dynamic values a revision needs that only evaluation can produce.
    struct _GraphRevision {
        SdfPath moverPath;
        SdfPath target;
        RigExecRevisionOp op;
        RigExecRevisionBinding binding;
        RigExecTapId transformTap = -1;
        /// computeMatrix of binding.transformSpace, or -1 (matrix).
        RigExecTapId transformSpaceTap = -1;
        /// computeMatrix per binding.influences entry, in that order (skin).
        std::vector<RigExecTapId> influenceTaps;
        RigExecTapId weightTap = -1;
        RigExecTapId driverFramesTap = -1;
        /// The mover asked for the provider's FINAL frame, so its transform
        /// is the aim-revised matrix computed in memory rather than the
        /// provider's own tapped computeMatrix.
        bool transformFinalPhase = false;
        /// Skin only: none of rigExec:jointIndices, jointWeights or
        /// elementSize can change within this epoch, so the layout may be
        /// resolved once and shared rather than re-read per frame. False
        /// whenever one of them is time-varying, carries an authored
        /// connection, or is written by a property chain.
        bool skinTopologyFixed = false;
    };
    /// Exact points target -> its revisions, in mover execution order.
    std::map<SdfPath, std::vector<_GraphRevision>> _graphChains;
    struct _LiveGraph {
        RigExecMoverGraph graph;
        VdfMaskedOutput source;
        std::vector<VdfMaskedOutput> revisions;
        std::vector<std::pair<SdfPath, RigExecRevisionOp>> identities;
        /// The authored base points currently standing in `source`, and
        /// whether they can still be standing there next frame.
        ///
        /// An authored base that is not time-varying is the same array at
        /// every time code in the epoch, so re-reading it and comparing it
        /// element by element against what the source already holds can only
        /// ever conclude "unchanged". Kept so that conclusion is reached once
        /// instead of once per frame; discarded by every change notice,
        /// because a value edit to the points is exactly what would make it
        /// wrong and does not begin a new epoch.
        VtVec3fArray basePoints;
        bool basePointsPushed = false;
        bool basePointsStatic = false;
    };
    /// Graph topology and computed checkpoints survive value/rest edits.
    /// Structural edits splice retained nodes by mover identity and operation.
    std::map<SdfPath, std::unique_ptr<_LiveGraph>> _liveGraphs;
    /// What this generation's property chains resolved, consulted by every
    /// static input read the evaluator and the packet assemblers make.
    ///
    /// Rebuilt at the top of each Evaluate, before anything reads an input:
    /// a property chain owes exec nothing, so it can always be resolved
    /// first, and every later read -- exec override, packet assembly, CPU
    /// oracle -- then sees one consistent value for the attribute.
    RigExecResolvedInputs _resolvedInputs;
    /// Authored inputs that cannot answer differently until the stage
    /// changes, held across generations; see RigExecStaticInputCache. Every
    /// notice clears it, and so does a change of interactive overrides.
    RigExecStaticInputCache _staticInputs;
    /// Uncommitted manipulation values; see SetInteractiveOverrides.
    std::vector<RigExecValueOverride> _interactiveOverrides;

    /// What every chain held at every point in the walk this generation.
    ///
    /// Only a phased read consults it, but it is filled unconditionally: the
    /// entries are copy-on-write handles to arrays the chain materialized
    /// anyway, so the cost of always having them is far below the cost of
    /// deciding per chain whether anyone will ask.
    RigExecChainSnapshots _chainSnapshots;

    /// Chain evaluation order: targets sorted so a chain that another chain
    /// reads at a non-base phase runs first.
    ///
    /// Compile-time state, because the dependencies come from the bindings
    /// and the bindings are epoch state. Replaces the "curvenets first, then
    /// everything else" pass that stood in for this while the only
    /// cross-chain dependency was the Profile Mover's.
    std::vector<SdfPath> _chainOrder;

    /// _chainOrder partitioned into dependency levels.
    ///
    /// Greedy over _chainOrder, so every level is a CONTIGUOUS run of it and
    /// the concatenation of the levels is _chainOrder itself. That is what
    /// makes a parallel level free of consequences beyond its speed: the walk
    /// publishes each chain's diagnostics, moved properties, weight fields
    /// and snapshots in the same order a serial walk would, whichever order
    /// the tasks happen to finish in.
    struct _ChainLevel {
        std::vector<SdfPath> targets;
        /// Whether the level's chains may run concurrently; see
        /// _IsChainLevelParallelSafe.
        bool parallel = false;
    };
    std::vector<_ChainLevel> _chainLevels;

    /// Whether a level of mutually independent chains may be walked with one
    /// task per chain.
    ///
    /// Independence in the dependency graph is necessary but not sufficient:
    /// the walk also touches per-generation state that is shared between
    /// chains, and each such use is a reason to keep the level serial rather
    /// than to lock something.
    bool _IsChainLevelParallelSafe(
        const std::vector<SdfPath> &targets) const;

    /// Exactly the intermediate values some phased read asks for:
    /// target -> the movers after which that chain must be snapshotted.
    ///
    /// Evaluating an intermediate head re-runs the chain prefix, so taking
    /// one after every revision would make a long chain quadratic for a
    /// feature almost no rig uses. Every phase is resolvable at compile to
    /// the single revision it names, so only those are taken.
    std::map<SdfPath, std::set<SdfPath>> _snapshotPoints;

    /// Profile Mover cut-meshes and factorizations, kept across frames.
    ///
    /// Lives on the evaluator rather than in the parameter packet because it
    /// is epoch state, not a value: the cut depends on the layout, and the
    /// layout is what an epoch IS. Keyed and digest-checked internally, so a
    /// curvenet edit rebinds and an unchanged one does not.
    mutable RigExecCurvenetBindCache _curvenetBindings;
    /// Per-epoch skin layouts, keyed by mover path.
    ///
    /// Lives here for the same reason the curvenet bindings do: it is epoch
    /// state, not a value, and it must not outlive the evaluator that read
    /// the stage it came from. Cleared by every change notice, by every
    /// interactive-override change, and by the commit of a new epoch.
    RigExecSkinTopologyCache _skinTopologies;
    /// Every property whose value can reach a cached skin layout: each skin
    /// mover's layout attributes, plus every attribute upstream of one of
    /// them along the authored connection chain the layout is read through.
    ///
    /// An interactive override drops the layouts only when it names one of
    /// these (SetInteractiveOverrides). Rebuilt lazily and invalidated
    /// exactly where the cache itself is, so the two can never disagree
    /// about which epoch's connections they describe.
    mutable std::set<SdfPath> _skinLayoutInputs;
    mutable bool _skinLayoutInputsValid = false;
    /// Fills _skinLayoutInputs from the standing mover order.
    void _ResolveSkinLayoutInputs() const;
    /// Whether any override in \p overrides can change a cached skin layout.
    ///
    /// Conservative in the direction of YES: a computation override, which
    /// names no property to compare, and a rig whose movers have not been
    /// resolved yet both answer true. The cost of a wrong yes is one
    /// re-read; the cost of a wrong no is a stale deformation.
    bool _OverridesReachSkinLayout(
        const std::vector<RigExecValueOverride> &overrides) const;
    /// Per-epoch blend sample shapes, keyed by the RigExecBlendSample prim.
    /// Same lifetime and same clearing rule as _skinTopologies above.
    RigExecBlendSampleCache _blendSampleShapes;
    /// One transform-valued input to a pose-domain constraint.
    ///
    /// RigExec providers publish computePointFrame and are therefore tapped;
    /// plain UsdGeomXformables have no such computation and are sampled from
    /// their native transform, asset-relative, during Evaluate(). Exactly one
    /// of frameTap/xformPath is populated by Compile().
    struct _FrameSourceBinding {
        SdfPath sourcePath;
        RigExecTapId frameTap = -1;
        SdfPath xformPath;
    };

    /// One compiled FBX-style pose constraint, in composed mover order.
    ///
    /// Position, Rotation, Scale, Parent, and Aim revise one provider;
    /// SingleChainIK revises an inferred namespace chain atomically. The
    /// authored values remain on the mover prim and are read per evaluation;
    /// this record holds only structural wiring.
    struct _FrameConstraint {
        SdfPath moverPath;
        TfToken schemaType;
        std::vector<SdfPath> targets;
        std::vector<_FrameSourceBinding> sources;
        _FrameSourceBinding worldUpObject;
        _FrameSourceBinding effector;
        std::vector<_FrameSourceBinding> poleObjects;
        std::vector<SdfPath> ikChain;
        /// Parallel to ikChain: 1 where a pose step BELOW this constraint
        /// wrote that joint, so the chain's rest reference for it is the
        /// frame that step left rather than its authored rest (spec §4.2).
        /// All zero on every rig with no step below the constraint, and the
        /// rest-derived IK preparation is then what it always was.
        std::vector<char> ikRestLive;
        /// Non-empty when the constraint writes the GEOMETRY domain: the
        /// <prim>.points property it revises. targets[0] stays the owning
        /// PRIM path either way, because every frame-domain map -- base and
        /// rest frames, the provider classifier, the frame chains -- is keyed
        /// by prim. The domain decides where the answer is published, not how
        /// it is solved.
        SdfPath pointsTarget;
        /// Optional common envelope field. Geometry resolves one value per
        /// point; the transform domain resolves one logical element. Empty
        /// means the constant synthesized from inputs:defaultWeight.
        SdfPath weightObject;
    };

    /// One property-domain revision: a float/vec3f/matrix math mover's
    /// operation over the preceding value of an exact scalar property.
    ///
    /// No tap and no binding. Every input is authored on the mover itself and
    /// the chain's base is the target attribute's own authored value, so the
    /// whole chain resolves off the stage before exec runs -- which is what
    /// lets the result be handed to exec as a value override rather than
    /// computed from it.
    struct _PropertyRevision {
        SdfPath moverPath;
        TfToken schemaType;
    };
    /// Exact scalar property target -> its revisions, in mover execution order.
    std::map<SdfPath, std::vector<_PropertyRevision>> _propertyChains;

    /// Property targets in dependency order. A chain that revises an input of
    /// another property mover (or of its bound weight object) runs first, so
    /// the consumer sees the same override Exec will receive rather than the
    /// target's authored fallback.
    std::vector<SdfPath> _propertyChainOrder;

    /// Everything the chains look up on the stage rather than compute, bound
    /// once and re-used until the stage moves: the target attribute and its
    /// value type, each mover's prim and weight-object targets, and a pinned
    /// UsdAttributeQuery for every input the revision loop reads by value.
    ///
    /// Dropped on every notice, beside _staticInputs and for the same reason
    /// -- a query caches where a value comes from, and only a notice can move
    /// that -- and again at the end of Compile, because a recompile may have
    /// replaced the chains the entries describe.
    std::unique_ptr<RigExecPropertyChainBindings> _propertyChainBindings;

    /// Evaluates every property chain at \p time.
    ///
    /// Fills \p results with the final value per target and appends one
    /// attribute override per target to \p overrides, so a consuming
    /// computation reads the revised value with nothing authored anywhere.
    /// Diagnostics record pass-throughs and failures (spec §6.6).
    void _EvaluatePropertyChains(
        UsdTimeCode time,
        std::map<SdfPath, VtValue> *results,
        std::vector<RigExecValueOverride> *overrides,
        std::vector<std::string> *diagnostics);
    /// Folds any plain Xformable lying between the asset root and a provider
    /// into that provider's seeded frames.
    ///
    /// Exec resolves a provider's parent space through a NamespaceAncestor
    /// that only RigExec types satisfy, so a `Scope` is correctly skipped and
    /// an `Xform` is silently dropped with it. This composes what was
    /// dropped, at evaluation, from the stage -- nothing is authored, and the
    /// rig follows the Xform wherever the author put it. See
    /// docs/superpowers/specs/2026-09-09-intervening-xform-design.md.
    ///
    /// Returns false only when a frame cannot be resolved at all; a rig with
    /// no such Xform returns true having done no work.
    bool _ComposeInterveningXforms(
        const UsdPrim &assetRoot,
        UsdGeomXformCache *xformCache,
        std::map<SdfPath, RigExecPointFrame> *restFrames,
        std::map<SdfPath, RigExecPointFrame> *baseFrames,
        std::map<SdfPath, RigExecPointFrame> *finalFrames,
        RigExecRigPose *pose) const;

    /// The frame a plain Xformable contributes to the pose: its transform
    /// relative to the asset root, at \p xformCache's time.
    ///
    /// A constraint target that is not a RigExec type, and a constraint
    /// source that is not a provider, both enter the walk this way -- read
    /// off the stage, never through exec, because a plain Xform has no
    /// computePointFrame to ask. False when \p path is not an Xformable or
    /// the asset root is gone; \p outFrame and \p outMatrix are optional
    /// and carry the same transform in the pose's two currencies.
    ///
    /// resetXformStack between the two is deliberately NOT diagnosed here:
    /// ComputeRelativeTransform stops accumulating at it and the partial
    /// matrix is what the pose has always used.
    bool _FrameFromXformRelativeToAsset(const UsdPrim &assetRoot,
                                        UsdGeomXformCache *xformCache,
                                        const SdfPath &path,
                                        RigExecPointFrame *outFrame,
                                        GfMatrix4d *outMatrix) const;

    /// Resolves a constraint source that is a native Xformable, carrying the
    /// delta of the deepest provider the walk has already revised above it.
    ///
    /// A native source that is not itself a written provider may still sit
    /// beneath a constrained transform provider. The closest revised
    /// ancestor contains all higher ancestor deltas, so applying it once to
    /// the stage-derived source frame applies all of them. \p providers
    /// enumerates the walk's frame store; the search over it -- strict
    /// prefix, points actually moved, deepest wins -- is here so the dense
    /// baked program and the map walk pick the same ancestor and compute the
    /// same delta.
    bool _ResolveNativeXformSource(
        const UsdPrim &assetRoot,
        UsdGeomXformCache *xformCache,
        const SdfPath &xformPath,
        const RigExecPoseFrameEnumerator &providers,
        RigExecPointFrame *out) const;

    /// Republishes every volume weight object's placement from the frames
    /// the walk holds NOW.
    ///
    /// A volume's field is measured in the space its own provider frame
    /// places it, so a constraint that moves the volume has to be visible to
    /// every weight resolved after it -- which means re-running this after
    /// every commit, not once before the walk. \p finalFrameOf is the walk's
    /// frame store; a provider it cannot answer for, or a frame no matrix
    /// can be built from, places at the identity.
    void _UpdateVolumePlacements(const RigExecPoseFrameLookup &finalFrameOf,
                                 RigExecRigPose *pose);

    /// One per-frame array of constraint source parameters, read RAW.
    ///
    /// Straight off the attribute at the frame's time: no connection walk,
    /// no resolved-input lookup, no interactive override. A source weight is
    /// an input of the constraint OPERATOR, not of the rig, and the
    /// evaluator and the baked program have to read it the same way -- so
    /// both read it here, and the cardinality diagnostic has one wording
    /// rather than one per caller. An absent or empty array is not a
    /// failure: it means the neutral value on every source.
    static bool _ReadConstraintSourceWeights(
        const UsdPrim &prim, const char *name, size_t count,
        UsdTimeCode time, std::vector<std::string> *diagnostics,
        std::vector<double> *weights);

    /// The same read for a per-source offset array, whose neutral is zero.
    static bool _ReadConstraintSourceOffsets(
        const UsdPrim &prim, const char *name, size_t count,
        UsdTimeCode time, std::vector<std::string> *diagnostics,
        std::vector<GfVec3d> *offsets);

    /// Whether a SingleChainIK chain's joints can move with time at all.
    ///
    /// "autoDetect" asks this to choose between solving the chain as it
    /// stands and solving a rest-derived copy of it: a chain whose
    /// translations and scales are authored once has no animated Ts for the
    /// solver to preserve, and rebuilding its layout from rest is then the
    /// better-conditioned question. Structural -- a solver binding, a time
    /// sample or a connection -- so it is settled for the epoch.
    bool _IkUsesAnimatedTs(const std::vector<SdfPath> &chain) const;

    /// The one diagnostic RIGEXEC_BAKE_REQUIRED exists to produce: says on
    /// \p pose that this generation ran dynamically while the mode asked for
    /// the program, and why. No-op unless the variable is set and the mode
    /// is Baked or BakedWithParityCheck. Publishes no value of its own.
    void _ReportBakeRequired(const std::string &detail,
                             RigExecRigPose *pose) const;

    /// The same fact, said to the ARTIST instead of to the harness: a plain
    /// line on \p pose reporting that the rig's rigExec:baked asked for the
    /// program and this generation was answered dynamically anyway, and why.
    /// No-op unless the attribute is what chose the mode. Publishes no value
    /// of its own, carries no "baked parity mismatch" prefix and moves no
    /// counter -- an authored attribute is a REQUEST, and a request that
    /// cannot be met is news, not a failure.
    void _ReportAttributeBakeFallback(const std::string &detail,
                                      RigExecRigPose *pose) const;

    /// Whether either report above would say anything about a fallback.
    ///
    /// The detail string they share costs an allocation to assemble and the
    /// ordinary dynamic path passes the same point on EVERY generation, so
    /// it is assembled only where somebody is listening.
    bool _FallbackIsWorthAnnouncing() const;

    /// Whether a refused bake has to say WHICH feature refused it.
    ///
    /// Not a gate on the work: the bakeability walk collects its refusals
    /// either way, because the list IS the answer. What it gates is whether
    /// they are KEPT -- carried on the epoch and reported once per
    /// generation -- and only two callers report them:
    /// RIGEXEC_BAKE_REQUIRED, and a rig that asked for the program through
    /// its own attribute and is owed the reason it did not get one.
    bool _WantsBakeRefusalReasons() const;

    /// Re-reads the rig's rigExec:baked and moves the mode to what it asks
    /// for, unless something stronger already chose (see
    /// RigExecEvaluationModeSource). Returns true when the mode MOVED, which
    /// is what a caller has to build or drop a program for.
    bool _RefreshAttributeEvaluationMode();

    /// Whether \p notice names the rig's rigExec:baked attribute -- changed
    /// in place, or resynced along with a prim above it.
    bool _NoticeNamesTheBakedAttribute(
        const UsdNotice::ObjectsChanged &notice) const;
    /// Re-pulls the epoch's rest frames after a stage edit no recompile
    /// covered. Returns false when the request could not produce them,
    /// which is what an incomplete per-frame rest tap used to mean.
    bool _RefreshEpochRestFrames();
    bool _EpochRestsMightVary() const;

    /// Providers whose base frame comes from their own USD transform rather
    /// than a computePointFrame tap: a plain Xform has no such computation.
    std::set<SdfPath> _xformDerivedProviders;

    /// RigExecRibbon path -> its driver curve's native points attribute.
    ///
    /// Compiled state that replaced the compiler's last authoring pass. The
    /// values are supplied to exec as overrides on the ribbon's
    /// rigExec:driverPoints / rigExec:restDriverPoints, because a
    /// relationship accessor can request computations on its targets but not
    /// a named attribute of them.
    std::map<SdfPath, SdfPath> _ribbonDriverPoints;
    /// All pose constraints, in the same execution order as _movers.
    std::vector<_FrameConstraint> _frameConstraints;
    /// Transform provider -> constraint mover paths that revise it, in
    /// mover execution order. This is the pose-domain counterpart of a points
    /// revision chain and supplies read-phase validation/snapshots.
    std::map<SdfPath, std::vector<SdfPath>> _frameChains;
    /// Provider -> base computePointFrame tap, for providers that are not
    /// joints (joints already have _jointFrameTaps).
    std::map<SdfPath, RigExecTapId> _providerBaseFrameTaps;

    /// Points target -> the derived normals/extent revisions it feeds.
    ///
    /// Derived maintenance has no authored mover (spec §7.6 revised): it is
    /// synthesized for any gprim that authors normals or extent alongside a
    /// moved points chain. Its input is that chain's FINAL points, so these
    /// are keyed by the points target and evaluated after it -- the reason
    /// they cannot simply live in _graphChains.
    std::map<SdfPath, std::vector<_GraphRevision>> _graphDerivedChains;
    /// Last derived result per normals/extent target, keyed by every input
    /// the recompute consumes: the chain's final points, the authored
    /// derived base, and the assembled topology/widths. An unchanged tuple
    /// republishes the stored result without touching the derived graph.
    struct _DerivedResult {
        std::vector<GfVec3f> points;
        VtVec3fArray base;
        std::vector<int> topologyCounts;
        std::vector<int> topologyIndices;
        std::vector<float> widths;
        VtVec3fArray result;
        bool cached = false;
    };
    std::map<SdfPath, _DerivedResult> _derivedCache;
    std::vector<RigExecMoverRecord> _movers;

    /// Baked falloff remaps for every volumetric weight object reachable
    /// in this epoch, ready to hand to RigExecTapSet::Evaluate as value
    /// overrides on each volume's computeFalloffLut stub.
    ///
    /// They live here rather than being rebuilt per frame because a
    /// falloff curve is epoch-structural: exec has no accessor for an
    /// attribute's spline (see RigExecFalloffLut in types.h), so the
    /// curve is resampled once at Compile and replayed unchanged until
    /// the next epoch.
    std::vector<RigExecValueOverride> _falloffLutOverrides;

    /// Volume weight objects whose rigExec:samplePhase is `current`,
    /// which have to measure the IN-FLIGHT points at their own position
    /// in the mover stack rather than the authored base.
    std::set<SdfPath> _currentPhaseWeights;

    /// computeMatrix taps for every volumetric weight object reachable
    /// in this epoch, and the matrices the current generation resolved
    /// them to. The CPU oracle reads the resolved matrix rather than
    /// recomputing the xformable frame chain a second time.
    std::map<SdfPath, RigExecTapId> _volumeWeightMatrixTaps;
    std::map<SdfPath, GfMatrix4d> _volumeWeightMatrices;

    bool _publishWeightFields = true;
    /// The compiled epoch flattened into an exec-free op list, built at the
    /// end of Compile when the mode asks for one.
    ///
    /// The program holds VALUES, not just structure, so the epoch digest
    /// cannot speak for it: the digest is unchanged by exactly the edits that
    /// make a captured constant wrong. The program therefore carries its own
    /// index of what the bake read, and a notice that hits it sets the flag
    /// below; the rebuild happens at the next Evaluate, once the dynamic
    /// generation has settled whether the epoch itself moved. A notice that
    /// misses the index leaves the program standing.
    std::unique_ptr<RigExecBakedProgram> _bakedProgram;
    /// Whether _bakedProgram has published a generation.
    ///
    /// What a rebuild inherits from its predecessor is the predecessor's
    /// REPORTED state -- which geometry nodes a consumer has already been
    /// told were created, what each of them last ran with -- so a program
    /// that never ran has nothing to hand over, and handing it over anyway
    /// makes the replacement's first generation claim less work than the
    /// dynamic path does. See _RebuildBakedProgram.
    bool _bakedProgramPublished = false;
    bool _bakedProgramStale = false;
    /// This epoch already asked for a program and was refused.
    ///
    /// The refusal is a property of the compiled epoch, so re-asking inside
    /// it costs a bakeability pass per frame to be told the same thing.
    /// Reset wherever the answer can have changed: Compile, and a notice
    /// that hit the program's capture index.
    bool _bakeRefused = false;
    /// Why it refused, when anyone asked to be told.
    ///
    /// Build is handed a reasons vector only under RIGEXEC_BAKE_REQUIRED:
    /// filling it walks every refusal on the rig instead of stopping at the
    /// first, and production pays nothing to collect strings nobody reads.
    /// Cleared wherever _bakeRefused is, for the same reason.
    std::vector<std::string> _bakeRefusalReasons;
    size_t _bakedProgramBuilds = 0;
    size_t _bakedProgramBuildAttempts = 0;
    size_t _bakedGenerations = 0;
    RigExecEvaluationMode _evaluationMode = RigExecEvaluationMode::Dynamic;
    RigExecEvaluationModeSource _evaluationModeSource =
        RigExecEvaluationModeSource::Default;

    size_t _structureDigest = 0;
    /// See GetStageEditSerial.
    uint64_t _stageEditSerial = 0;
    TfNotice::Key _noticeKey;
    bool _structureDirty = true;
    bool _compiled = false;

    /// Scoped phase timings for Compile and Evaluate. Off unless profiling
    /// is enabled; see SetProfilingEnabled.
    RigExecProfiler _profiler;

    /// The program is built from the compiled epoch tables above, which are
    /// private because they are not a published surface -- not because the
    /// one class whose whole job is to flatten them should re-derive them.
    friend class RigExecBakedProgram;
};

/// Test and diagnostic access to the constraint handler registry -- the one
/// table that says which constraint operators exist and what each honors.
/// Exposed so a test can assert the table and the schema agree, which is the
/// property that keeps a newly added operator from being registered in one
/// and forgotten in the other.
///
/// Count is 0 or 1: a type has at most one row.
size_t RigExecConstraintHandlerCount(const TfToken &schemaType);
size_t RigExecConstraintHandlerTotal();

/// Whether \p schemaType's operator honors rigExec:rotationOrder. False for a
/// type with no registry row.
bool RigExecConstraintUsesRotationOrder(const TfToken &schemaType);

}  // namespace rigExec

#endif  // RIGEXEC_RIG_EVALUATOR_H
