// Compiles source scene facts into the authoritative native operation program.
// Optional scalar reference checks run over detached independent source inputs.
#ifndef RIGEXEC_RIG_EVALUATOR_H
#define RIGEXEC_RIG_EVALUATOR_H

#include "bakedProgram.h"
#include "liveOperationGraph.h"
#include "moverGraphCaches.h"
#include "moverGraphTypes.h"
#include "profiler.h"
#include "solverKernels.h"
#include "tapSet.h"

#include "rigExecMath/rbf.h"
#include "rigExecMath/solvers.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/tf/functionRef.h"
#include "pxr/base/tf/hash.h"
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

struct RigExecSceneDescriptors;
struct RigExecScenePoseInterpolatorDescriptor;
struct RigExecSceneSpaceSwitchDescriptor;

/// One discovered mover application (spec §4.2): the reverse-sibling
/// post-order ordinal plus canonicalized targets.
struct RigExecMoverHandler;
struct RigExecMoverRecord {
    SdfPath moverPath;
    TfToken schemaType;
    const RigExecMoverHandler *handler = nullptr;
    std::vector<SdfPath> targets;  ///< canonicalized (prim -> .points etc.)
    int ordinal = 0;
    bool enabledFallback = true;
};

/// An operator input whose connection reads a property chain at a phase:
/// rigExecReadPhase on the input, `base` when none is authored.
/// `applied` is how many of the chain's revisions the value includes -- 0
/// for `base`, the revision count through a named prim for a checkpoint,
/// every revision for `final`. The value is published on the consumer
/// itself, so every reader of the input gets it through the routes a chain
/// result takes, whatever an intermediate hop of its connection declares.
///
/// `hops` is the consumer followed by every attribute its connection walk
/// passes before the target. An interactive override on any of them is
/// what the overlay walk meets first, so the record publishes nothing
/// while one stands. An override on the target is the chain's base: the
/// revisions run from it, and every record reads the chain's history from
/// there, as it would with that value authored on the target.
struct RigExecPhasedConnection {
    SdfPath consumer;
    SdfValueTypeName consumerType;
    SdfPath target;
    size_t applied = 0;
    SdfPathVector hops;
    /// Declared `final` (then `applied` is the revision count).
    bool final = false;
};

/// One weight object's resolved field, as a mover actually consumed it.
///
/// Dense and already range-policed, so a consumer can index it by element
/// without knowing whether the field came from an authored table, a
/// driven modulation, a placed volume, or a composition of those.
struct RigExecResolvedWeightField {
    SdfPath target;              ///< canonical points property weighted
    VtFloatArray weights;  ///< one per logical element
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
    /// RigExecRigEvaluator::cpuReference is set (scalar-reference
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

    /// Independent scalar-reference comparisons, when cpuReference is on.
    /// Both are zero when no reference checks ran; callers must check the
    /// agreement count to distinguish that from a verified generation.
    size_t referenceMismatches = 0;
    size_t referenceAgreements = 0;

    /// Exact pose comparison disagreements from explicit verification tools.
    size_t comparisonMismatches = 0;

    /// Operation bodies actually executed by the common graph this generation.
    size_t executedOpCount = 0;

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

/// Volume placements held outside the evaluator -- the baked program's
/// slot-indexed table -- for the CPU oracle to read in place of its own map.
/// Borrowed pointers, no allocation. A path resolves when \p index maps it to
/// a slot below \p count whose \p placed byte is set; any other path has no
/// placement, as a path missing from the evaluator's map has none.
struct RigExecVolumePlacementView {
    const std::map<SdfPath, int> *index = nullptr;
    const char *placed = nullptr;
    const GfMatrix4d *placements = nullptr;
    size_t count = 0;
    const std::map<SdfPath, GfMatrix4d> *byPath = nullptr;

    const GfMatrix4d *Find(const SdfPath &path) const
    {
        if (byPath) { const auto found=byPath->find(path); return found==byPath->end()?nullptr:&found->second; }
        if (!index) return nullptr;
        const auto it = index->find(path);
        if (it == index->end() || it->second < 0 ||
            size_t(it->second) >= count || !placed[it->second]) {
            return nullptr;
        }
        return placements + it->second;
    }
};

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
class RigExecGoldenSuiteObserver;
class RigExecInputReplayObserver;

/// What one stage notice did to the baked program (plan 2.1): the
/// evaluator's per-notice disposition, classifying the three branches of
/// its notice handler so the registry's notice adapter can retire and
/// re-resolve at the granularity the branch allows instead of cancelling
/// the warming generation wholesale.
enum class RigExecNoticeDisposition {
    /// No compiled program stood to classify against:
    /// the notice reached no program at all.
    None,
    /// Default-only avar value edits, patched in place (plus the patched
    /// property paths): the program already re-runs only their cone live,
    /// with no stamp bump. Retire exactly the patched avars' clusters.
    Patched,
    /// A value edit the capture index missed and the program could not
    /// route: the program is still right about its structure and its stamp
    /// was bumped, so the next generation runs everything once. Retire
    /// affected clusters and re-resolve.
    StampBumped,
    /// Value edits routed without a stamp bump (RigExecBakedProgram::
    /// ApplyValueEdits): the per-frame inputs they reached re-run their
    /// cone once, and what else they name is read by value-compared sources
    /// or by nothing. The paths are the notice's properties something in the
    /// program reads; a property nothing reads is left out, and retires
    /// nothing.
    Edited,
    /// The notice hit the capture index: the program is stale and will be
    /// rebuilt, the epoch moves, and epoch-half eviction plus generation
    /// cancel stand.
    Stale,
};

/// Calls and stage edits on this evaluator's stage are serialized by its
/// owning thread. Frozen workers use snapshots and join before owner adoption.
class RigExecRigEvaluator : public TfWeakBase {
public:
    RigExecRigEvaluator(const UsdStageRefPtr &stage, const SdfPath &rigPath);
    ~RigExecRigEvaluator();

    /// Discovers joints and movers, validates targets, and prepares the
    /// exec tap set. Returns false with messages on validation failure.
    ///
    /// Always compiles: a failure the settle path memoized (see
    /// _CompileUnlessKnownBroken) is forgotten, never replayed, here.
    bool Compile(std::vector<std::string> *errors = nullptr);

    /// Evaluates one complete generation at an explicit time.
    RigExecRigPose Evaluate(UsdTimeCode time);

    /// Whether the compiled epoch can be baked, appending one reason per
    /// feature that stops it.
    ///
    /// Refused export admission names the unsupported source feature.
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
    /// Observable for tests of generation execution and reuse.
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

    /// The standing compiled program, or null when compilation failed.
    ///
    /// For the suites that assert on the program's STRUCTURE -- what a
    /// drag's cone may reach, which entry holds which version -- against the
    /// graph rather than against a number somebody wrote down. Nothing in
    /// the library reads it.
    const RigExecBakedProgram *GetBakedProgram() const {
        return _bakedProgram.get();
    }

    /// The baked program's op trace and op graph for the last generation,
    /// or empty when the program did not answer it (bakedTrace.h).
    std::vector<RigExecOpTraceEntry> GetLastOpTrace() const;
    std::vector<RigExecOpGraphNode> GetOpGraph() const;

    /// How many skin layouts the epoch's topology cache is holding answers
    /// for. The cache is the dynamic walk's; a baked program builds its
    /// layouts in its SkinTopology ops and never fills it.
    ///
    /// The other thing about an interactive generation that a published pose
    /// cannot show: dropping the layouts and re-reading them publishes
    /// exactly the same deformation as keeping them, so only the cache's own
    /// occupancy says whether a drag paid for the re-read. A test that
    /// overrides an unrelated control reads this to see that it did not.
    size_t GetSkinTopologyCacheSize() const;
    /// The same for the blend sample shapes: how many samples the cache is
    /// holding an answer for, which a stage edit on one sample's shape
    /// drops by one and an edit elsewhere leaves where it was.
    size_t GetBlendSampleCacheSize() const;

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
    /// replaces any earlier one on the same key rather than joining it. One
    /// on a property math movers revise is that property's base: the chain
    /// revises it as it would the value authored there and publishes its
    /// own result, so every reader sees while the drag is held what it sees
    /// once the value is authored, in every evaluator.
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

    /// Values an upstream scene index publishes for authored attributes:
    /// authored-level, so each replaces the stage value of its attribute at
    /// every time, below any interactive override on the same key. Each
    /// entry names an attribute (\p attribute set, no computation). The list
    /// replaces the last one; an empty list lifts every value. Setting it
    /// does not evaluate.
    ///
    /// A value is admitted only on an attribute that is unconnected, has a
    /// stage value, holds one of the eight input-slot types the value holds
    /// exactly, and (while a baked program stands) is read through a read a
    /// bake lists as an input. An array value (only while
    /// RigExecUpstreamArrayAdmission is on) must also hold as many elements
    /// as the stage value at the generation's time. Every other key is
    /// ignored, and each generation reports "upstream input <path>:
    /// <reason>; ignored". Admission is re-decided at every Evaluate,
    /// against the program then standing.
    void SetUpstreamInputs(std::vector<RigExecValueOverride> inputs);

    /// Whether the caller has upstream values set (admitted or not).
    bool HasUpstreamInputs() const { return !_upstreamRequested.empty(); }

    /// The list SetUpstreamInputs last received.
    const std::vector<RigExecValueOverride> &GetUpstreamInputs() const {
        return _upstreamRequested;
    }

    /// The admitted keys, sorted.
    std::vector<SdfPath> GetUpstreamInputPaths() const;

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
    void SetPublishWeightFields(bool publish);
    bool GetPublishWeightFields() const { return _publishWeightFields; }

    /// The rig prim this evaluator was constructed with, for tools that
    /// report on the bake (a .rigexec file's rig field names it).
    const SdfPath &GetRigPath() const { return _rigPath; }
    /// Target-local spatial matrix primvars in this compiled epoch.
    std::vector<SdfPath> GetSurfaceProjectorTargets() const;

    /// Composed mover-stack applications: descendants before their mover
    /// parent, sibling branches in reverse composed child order (the bottom
    /// usdview row executes first; spec §4.2).
    const std::vector<RigExecMoverRecord> &GetMoverOrder() const {
        return _movers;
    }

    /// The read phases the last compile resolved on operator inputs'
    /// connections, in _ForEachConnectedInput order. Every connected input
    /// whose walk reaches a property-chain target has one -- `base` when it
    /// declares nothing -- except where the overlay walk already answers:
    /// a `final` whose walk passes no recorded hop reads the target's
    /// published value (unless it is a double reading a float chain, which
    /// only a record widens), and an undeclared input that math movers
    /// revise themselves, or whose value type differs from the target's,
    /// never reads the chain through its connection.
    const std::vector<RigExecPhasedConnection> &GetPhasedConnections() const {
        return _phasedConnections;
    }

    /// Whether the last compile has a property chain revising \p attribute.
    /// An override on such a property and a value authored there are both
    /// the base the chain revises.
    bool IsPropertyChainTarget(const SdfPath &attribute) const {
        return _propertyChains.count(attribute) > 0;
    }

    /// Operations the last compile set aside -- operation prim -> the error
    /// that disqualified it. A broken operation (a mover whose target is
    /// not on this stage, a solver naming a prim that is not a joint, ...)
    /// is warned about and left out; the rest of the rig -- its controls,
    /// joints and every other operation -- compiles and evaluates without
    /// it. Empty when nothing was skipped or the compile failed outright.
    const std::map<SdfPath, std::string> &GetSkippedOperations() const {
        return _skippedOperations;
    }

    /// Structural digest of the compiled mover topology: the v0.1
    /// binding-epoch identity. Structural edits change it and trigger
    /// recompilation on the next Evaluate (spec §4.2, §6.3).
    size_t GetBindingEpochDigest() const { return _structureDigest; }

    /// Number of actual retained, valid provider rest frames whose complete
    /// selected source closure is constant in the current compiled epoch.
    /// Build captures and composes these frames before the first Evaluate.
    /// Animated and produced rest inputs remain supported and run through
    /// the graph; they exclude only the affected providers and descendants
    /// from this count. Constant authored connections can remain static.
    size_t GetEpochRestFrameCount() const;

    /// Transform provider -> the pose steps that write it, in the order the
    /// pose walk runs them: the aggregate solvers that name it on
    /// rigExec:joints and the frame constraints that move it, INTERLEAVED in
    /// one hierarchical stack (spec §4.2). The last entry is the step whose
    /// frame the pose publishes; a solver entry is also what an AtPrim read
    /// phase naming that solver resolves against.
    ///
    /// Diagnostic access only -- evaluation order itself comes from the
    /// interleaved pose steps, and this is read back from them.
    /// The compiled epoch as one operation graph: every solver batch and
    /// solver, frame constraint, geometry chain and revision, property
    /// chain and revision, exec tap, provider value, space switch and
    /// pose interpolator, with every dependency the compile derived.
    ///
    /// Diagnostic access for the graph visualizer and the compile-accounting
    /// suite; evaluation itself never reads it. Ids are stable within the
    /// epoch (see liveOperationGraph.h) and each node names the profiler
    /// scope that fires it, so a generation's profile events attribute
    /// straight onto the structure. Empty before the first Compile.
    RigExecLiveOperationGraph DescribeLiveOperations() const;

    const std::map<SdfPath, std::vector<SdfPath>> &GetFrameChains() const
    {
        return _frameChains;
    }

    /// When set, Evaluate verifies geometry against the CPU reference and
    /// publishes that reference in RigExecRigPose::movedPropertiesCpu.
    /// Disabled for interactive use so every deformation runs only once.
    bool cpuReference = false;

    /// Owner-thread opt-in to bounded, last-run operation timings. No
    /// profiler events or locks are introduced in computation bodies.
    void SetOpTimingEnabled(bool enabled) { _opTimingEnabled = enabled; }
    bool GetOpTimingEnabled() const { return _opTimingEnabled; }

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
    void SetSolverGuidesEnabled(bool enabled);

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

    /// Classifies \p notice against the standing baked program without
    /// mutating anything: Patched (plus the property paths a patch would
    /// write, in \p patchedPaths) when the notice is nothing but
    /// patchable avar default values, Stale when it hits the capture
    /// index, Edited (plus the property paths the program reads, in
    /// \p patchedPaths) when every value it carries can be routed,
    /// StampBumped for any other value edit, None when no program stands.
    /// The notice handler branches on this same classification, so the
    /// query and the mutation can never disagree.
    RigExecNoticeDisposition ClassifyNoticeDisposition(
        const UsdNotice::ObjectsChanged &notice,
        std::vector<SdfPath> *patchedPaths = nullptr) const;

    /// The last notice's disposition and patched paths, as classified when
    /// the notice handler ran. None/empty before any notice, or when the
    /// last notice found no program.
    RigExecNoticeDisposition GetLastNoticeDisposition() const
    {
        return _lastNoticeDisposition;
    }
    const std::vector<SdfPath> &GetLastNoticePatchedPaths() const
    {
        return _lastNoticePatchedPaths;
    }

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

    /// Reads \p prim's points-bearing target relationship and returns its
    /// authored value at \p time. Accepts either an exact property path
    /// or a prim path canonicalizing to .points.
    bool _ReadTargetPoints(
        const UsdPrim &prim, const TfToken &relationshipName,
        UsdTimeCode time, std::vector<GfVec3f> *points) const;

    /// Compiles if this rig never has, recompiles if a structural edit moved
    /// the epoch digest, and clears the structure-dirty flag. False when the
    /// compile failed, with \p diagnostics saying why.
    ///
    /// Settle source notices and compile admission before dispatching the
    /// operation graph. Repeated settled calls do nothing.
    bool _SettleEpoch(std::vector<std::string> *diagnostics);

    /// The whole of Compile but for forgetting the failure memo: what the
    /// settle path runs, so that its own compiles can be memoized. One
    /// broken operation does not fail it: see _CompileEpochAttempt.
    bool _CompileEpoch(std::vector<std::string> *errors);

    struct _CompileFailure {
        std::string message;
        SdfPathVector operations;
    };

    /// One compile with _skippedOperations left out. Failure metadata is
    /// local to the attempt and names the operations to set aside.
    bool _CompileEpochAttempt(std::vector<std::string> *errors,
                              _CompileFailure *failure);

    /// RigExecSurfaceProjector prims and the points property each rides.
    /// Compile turns each into derived matrix targets on that chain
    /// (RigExecRevisionOp::SurfaceProjector / ShaderDials).
    struct _SurfaceProjectorRecord {
        SdfPath path;
        SdfPath target;   ///< the points property it rides
    };

    /// Discover and validate movers without changing the active epoch.
    /// \p inertMovers receives each mover prim with an empty rigExec:moves.
    bool _DiscoverMovers(std::vector<RigExecMoverRecord> &movers,
                         std::vector<_SurfaceProjectorRecord> &projectors,
                         SdfPathVector &inertMovers,
                         std::vector<std::string> *errors,
                         _CompileFailure *failure) const;

    /// Whether \p path is an operation the current compile set aside.
    bool _IsSkippedOperation(const SdfPath &path) const {
        return _skippedOperations.count(path) > 0;
    }

    /// The settle path's compile. A compile that failed is answered from
    /// _failedCompile, without compiling, for as long as its key matches:
    /// its errors are appended to \p diagnostics as the compile appended
    /// them, and false is returned. Otherwise it compiles, and memoizes a
    /// failure or forgets the memo on success.
    ///
    /// While the stage stays broken the compile's TF_WARNs therefore reach
    /// the host once, on the compile that failed, and the program build and
    /// attempt counters stand still; the pose diagnostics repeat every frame
    /// exactly as a real compile would write them.
    bool _CompileUnlessKnownBroken(std::vector<std::string> *diagnostics);

    /// Whether _failedCompile holds a failure of the stage this
    /// compile would read now.
    bool _FailedCompileMemoMatches() const;
    /// Applies the standing interactive overrides to \p resolved -- the
    /// owning-thread sampling input layer before graph execution.
    ///
    /// The baked program runs the chains itself and needs the overrides at
    /// exactly that point; it calls this rather than carrying a second copy
    /// of the ordering rule, because a second copy is a second answer.
    void _ApplyInteractiveOverridesToResolved(
        RigExecResolvedInputs *resolved) const;
    /// The same for the admitted source input values.
    void _ApplyValueInputsToResolved(RigExecResolvedInputs *resolved) const;

    /// Re-decides which of _upstreamRequested are admitted, against the
    /// stage and the program standing now, and installs the answer
    /// (_SetUpstreamAdmitted), at \p time (condition 4) or, with none
    /// given, at the last generation's. Owning thread.
    void _AdmitUpstreamInputs();
    void _AdmitUpstreamInputs(UsdTimeCode time);
    /// Installs \p admitted (sorted by path) and its drop lines: rebuilds
    /// _valueInputs and drops the skin layouts and blend shapes a placed,
    /// moved or lifted value can reach. A list equal to the standing one
    /// changes nothing.
    void _SetUpstreamAdmitted(std::vector<RigExecValueOverride> admitted,
                              std::vector<std::string> dropLines);
    /// _upstreamAdmitted merged with _interactiveOverrides into _valueInputs.
    void _RebuildValueInputs();

    /// Builds the compiled program for the settled epoch, or drops it on
    /// failed admission.
    /// Rebuilds the baked program, handing the outgoing one's persistent
    /// geometry state to its replacement (RigExecBakedProgram::
    /// AdoptGeometryStateFrom). \p outgoing is passed rather than read off
    /// the member because Compile retires the program at its head, where a
    /// failure has to drop it, and rebuilds at its tail.
    void _RebuildBakedProgram(
        std::unique_ptr<RigExecBakedProgram> outgoing = nullptr,
        const RigExecSceneDescriptors *scene = nullptr,
        const UsdStageWeakPtr &sceneStage = UsdStageWeakPtr(),
        uint64_t sceneSerial = 0);

    /// The structure digest in three segments: the discovered output sets
    /// (joints, controls, volume guides, pose interpolators), the solver
    /// wiring, and the mover wiring. Each segment's bytes are a pure function
    /// of the composed stage and of nothing another segment emits -- its own
    /// string, its own weight-object visiting set, its own profile region --
    /// so the three can run on three tasks, and the digest is the hash of
    /// their concatenation in this order, byte for byte the text one serial
    /// walk over all three emits. The Solvers segment's WALK is never split
    /// further: its cycle markers name the prim the walk was on when it
    /// closed a cycle, so its bytes depend on walk order within it. Its
    /// reads are: they are prefetched per prim, in parallel, ahead of the
    /// walk, which then only assembles them.
    enum _DigestSegment : unsigned {
        _DigestOutputSets = 1u << 0,
        _DigestSolvers = 1u << 1,
        _DigestMovers = 1u << 2,
        _DigestAllSegments = 7u,
        /// Not a segment: the Solvers segment reads every attribute from
        /// the stage each time it needs one, instead of through its
        /// per-call read memo. The bytes are the same either way; this is
        /// what RIGEXEC_VERIFY_DIGEST_MEMO checks that against.
        _DigestUnmemoizedReads = 1u << 3,
        /// Not a segment either: the call runs beside compile's own readers
        /// of the stage, so the Solvers segment's read prefetch keeps to a
        /// few lanes rather than the whole pool. It changes where the reads
        /// run and nothing about the bytes.
        _DigestBesideCompile = 1u << 4,
    };
    static constexpr size_t _DigestSegmentCount = 3;
    /// The prims one digest read outside the rig's namespace (unified-program
    /// spec §4.1, the prim-granular gate): relationship and connection
    /// targets, missing ones included, the prims read through them -- mover
    /// target gprims, weight objects, blend inputs and samples, pose-input
    /// providers -- and the ancestors walked above a prim.
    ///
    /// Two kinds, because what the digest reads of a walked ancestor is less
    /// than what it reads of a prim it looked at: an ancestor contributes its
    /// validity, type name and own properties and nothing below it, so an
    /// edit under an ancestor can only reach the digest through a prim the
    /// walk started from, which is in \c read. Keeping the two apart is what
    /// leaves an edit elsewhere under a shared ancestor -- a material beside
    /// the rig, under the asset root the ancestor chains all pass through --
    /// out of the gate.
    /// What one digest provably hashed, recorded so that an edit which
    /// changes one of these things can be known to move the digest without
    /// recomputing it (unified-program spec §7 EP2a, rule T-e). Each entry
    /// is recorded where the digest writes it, so a change to it changes
    /// the digest's text at that place.
    struct _CertainFootprint {
        /// Composed targets of a relationship the digest wrote out, and
        /// whether every place that wrote it sorted them first: a reorder
        /// moves the digest only through a place that did not.
        struct Targets {
            SdfPathVector targets;
            bool sorted = true;
        };
        /// The value and time-sample count of a solver's
        /// rigExec:jointElements, both of which the digest writes.
        struct JointElements {
            VtIntArray values;
            size_t samples = 0;
        };
        /// Prims under the rig whose PATH the digest writes because of their
        /// type -- the discovered joints, controls, volume weights and pose
        /// interpolators, and every aggregate solver -- with that type.
        /// Ordered, so the ones below a resynced prim are one range.
        std::map<SdfPath, TfToken> prims;
        /// A property as its owning prim and its name, which is how the
        /// digest reaches one: keyed this way, recording it builds no
        /// property path.
        struct PropertyKey {
            SdfPath prim;
            TfToken name;
            bool operator==(const PropertyKey &other) const {
                return prim == other.prim && name == other.name;
            }
        };
        struct PropertyKeyHash {
            size_t operator()(const PropertyKey &key) const {
                return TfHash::Combine(key.prim, key.name);
            }
        };
        /// Every existing relationship the digest wrote the targets of.
        std::unordered_map<PropertyKey, Targets, PropertyKeyHash> targets;
        /// Keyed by the solver prim.
        std::unordered_map<SdfPath, JointElements, SdfPath::Hash>
            jointElements;
        /// The prims of the pose-input closures: the digest writes every
        /// attribute one of them has, with its connection sources.
        std::unordered_set<SdfPath, SdfPath::Hash> closurePrims;
        /// The connected attributes among those, with their sources. An
        /// attribute of a closure prim that is not here was unconnected.
        std::unordered_map<SdfPath, SdfPathVector, SdfPath::Hash>
            closureConnections;
    };
    struct _DigestFootprint {
        /// Prims the digest looked at: an edit on, under or above one is
        /// suspect.
        std::unordered_set<SdfPath, SdfPath::Hash> read;
        /// Prims the digest only walked through as an ancestor: an edit on
        /// or above one is suspect.
        std::unordered_set<SdfPath, SdfPath::Hash> ancestors;
        _CertainFootprint certain;
    };
    /// The committed footprint of the digest in _structureDigest, in the
    /// form the notice gate asks it: \c read as recorded, and \c covered,
    /// every recorded prim of either kind plus every ancestor of one, so
    /// "on or above a recorded prim" is one lookup of the notice's prim.
    /// \c certain is each segment's certain footprint, as recorded.
    struct _DigestGate {
        bool valid = false;
        std::unordered_set<SdfPath, SdfPath::Hash> read;
        std::unordered_set<SdfPath, SdfPath::Hash> covered;
        _CertainFootprint certain[_DigestSegmentCount];
    };
    /// The text of the selected \p segments, concatenated in segment order.
    /// With \p footprint, also the prims outside the rig the segments read,
    /// and what they provably hashed.
    std::string _ComputeStructureDigest(
        unsigned segments, _DigestFootprint *footprint = nullptr) const;
    /// The gate form of the three segments' footprints, which it takes the
    /// certain footprints out of.
    static _DigestGate _MakeDigestGate(
        _DigestFootprint (&parts)[_DigestSegmentCount]);
    /// Notes the paths of a digest-suspect \p notice that could make the
    /// edit certainly structural, for _EditIsCertainlyStructural to judge at
    /// the next settle.
    void _NoteCertainStructuralCandidates(
        const UsdNotice::ObjectsChanged &notice);
    /// Whether the edits since the last settle certainly moved the digest:
    /// a candidate path whose state now differs from what the committed
    /// certain footprint recorded for it. False when there is no committed
    /// footprint or when the candidates overflowed.
    bool _EditIsCertainlyStructural() const;
    /// Whether \p notice could move the structure digest: the prim-granular
    /// gate of spec §4.1, read against _digestGate. Without a committed
    /// footprint every notice is suspect.
    bool _NoticeIsDigestSuspect(const UsdNotice::ObjectsChanged &notice) const;
    /// RIGEXEC_VERIFY_DIGEST_GATE=1: recomputes the digest after a notice
    /// the gate called neutral and fails fatally if it moved.
    void _VerifyNeutralNotice(const UsdNotice::ObjectsChanged &notice) const;
    /// Hashes the three segment texts, concatenated, into the epoch digest.
    /// Under RIGEXEC_VERIFY_DIGEST_SPLIT=1 it first recomputes the whole
    /// digest serially on this thread and fails fatally unless that text
    /// equals the concatenation. Under RIGEXEC_VERIFY_DIGEST_MEMO=1 it
    /// recomputes the Solvers segment with _DigestUnmemoizedReads and fails
    /// fatally unless that text equals the memoized one.
    size_t _JoinStructureDigest(
        const std::string (&parts)[_DigestSegmentCount]) const;
    /// The whole digest, for the settle path: the three segments on three
    /// tasks under a scoped dispatcher (serially with
    /// RIGEXEC_ENABLE_PARALLEL_EVAL=0), then joined. With \p gate, also the
    /// gate form of what they read.
    size_t _SettleStructureDigest(_DigestGate *gate = nullptr) const;
    void _OnObjectsChanged(const UsdNotice::ObjectsChanged &notice,
                           const UsdStageWeakPtr &sender);
    /// Derives rigExec:startFrame targets from the joint hierarchy for
    /// every RigExecFkChain with rigExec:startFramePolicy = "parent" and
    /// no authored targets, and authors them into the stage session
    /// layer (never the asset), retracting this evaluator's previous
    /// opinions first. Runs at the head of Compile, single-threaded,
    /// before the digest dispatch and every parallel stage read: the
    /// one exception to "compile authors nothing", so that scheduling,
    /// exec reachability, and the baked program all see the derived
    /// targets as authored. Fires no notices (TfNotice::Block): the
    /// imaging registry holds a non-recursive mutex across Compile and
    /// a notice here would re-enter it. Returns warnings (spanning
    /// providers, missing ancestors) for the caller to emit; they never
    /// fail the compile.
    std::vector<std::string> _ApplyDerivedStartFrames();

    UsdStageRefPtr _stage;
    SdfPath _rigPath;
    /// The seven rest input names (evaluatorDetail::_MakeRestInputNames),
    /// built in the constructor.
    std::vector<TfToken> _restInputNames;
    bool _solverGuidesEnabled = true;

    std::vector<SdfPath> _jointPaths;
    /// Every RigExecControl beneath the rig, discovered exactly the way the
    /// joint outputs are (spec §4.1: the rig is a namespace root, not a
    /// manifest). Unlike the joints an empty set is legal -- a rig driven
    /// entirely by avars on its joints has no control prims at all -- so it
    /// never fails Compile.
    std::vector<SdfPath> _controlPaths;
    /// Base computePointFrame per control, parallel to _controlPaths.
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
    struct _SolverBatch {
        std::set<SdfPath> solvers;
        std::set<SdfPath> dependencies,frameInputs;
        std::map<SdfPath,SdfPath> restInputs;
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
    /// Operation producers from discovery, independent of legacy walk numbering.
    std::map<SdfPath, std::set<SdfPath>> _nativePoseDependencies;
    std::map<SdfPath,int> _nativePoseOrdinals;
    std::set<SdfPath> _nativeProviderPaths;
    /// Original rest-epoch classification footprint, independent of extra
    /// native geometry frame sources. Owner-only observable transition facts.
    std::set<SdfPath> _restEpochProviderPaths;
    bool _epochRestsConstant = true;
    std::map<SdfPath,RigExecPointFrame> _epochRestFrames;
    /// The rest gate (_NoteRestEdits). The epoch's rest paths are every
    /// _restInputNames property of every key of _restTapIds, which is
    /// every provider and every provider ancestor the rests read, so the
    /// set is kept as that map and the name list rather than spelled out.
    /// A notice that resyncs or changes one of those paths, or resyncs a
    /// prim at or above a provider, adds the providers it reached here and
    /// marks the epoch's rest frames stale. A rest name changed on a prim
    /// that is not a key but has keys under it -- an inherited rest-frame
    /// publisher the type list does not name -- marks them stale and adds
    /// nothing. Nothing else does either. The commit clears both.
    ///
    /// The providers whose rest channels _SettleEpoch has to re-classify
    /// (_EpochRestsMightVary), consumed by the next settle.
    std::set<SdfPath> _restEditedProviders;
    /// _epochRestFrames no longer holds what the stage says. Consumed by
    /// the owning-thread epoch settle before the graph reads captured rest
    /// facts. A failed refresh leaves it set.
    bool _epochRestFramesStale = false;
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
    /// Reused by the owning-thread source sampler across evaluations.
    std::unordered_set<SdfPath, SdfPath::Hash> _hierarchicalProviderSet;
    /// Dense provider index: SdfPath → index into _providerPaths.
    /// Built once per epoch from the union of all paths the frame-domain
    /// maps can hold. _providerPaths is in SdfPath-sorted order so that
    /// iterating live indices reproduces std::map key order.
    std::vector<SdfPath> _providerPaths;
    std::unordered_map<SdfPath, int, SdfPath::Hash> _providerIndex;
    /// Per provider index: indices of strict hierarchical descendants that
    /// are seeded providers, in SdfPath order. Replaces the per-candidate
    /// finalFrames prefix walk in commitConstraintFrames.
    std::vector<std::vector<int>> _hierDescendants;
    std::set<SdfPath> _nativeGuideSolvers;

    /// Detached pose-interpolator discovery and solved immutable structure.
    struct _PoseInterpolator {
        std::shared_ptr<const RigExecScenePoseInterpolatorDescriptor> descriptor;
        SdfPath prim;
        SdfPath driver;
        /// The driver's namespace ancestor that publishes a frame, or empty
        /// when there is none and the driver's local rotation is its world
        /// one. Resolved at compile: namespace topology does not move.
        SdfPath driverParent;
        /// rigExec:driverAttributes: a NUMERIC driver. One to three float or
        /// double properties read in order as the driver's position, in
        /// place of a transform's translation; empty on a transform-driven
        /// interpolator.
        std::vector<SdfPath> driverAttributes;
        /// <pose>.outputs:weight for the poses that are in the solve, in the
        /// solver's own index order.
        std::vector<SdfPath> poseWeights;
        /// The same for poses whose inputs:enabled is off. A disabled pose is
        /// left out of the solve entirely (schema: leaving it in would keep
        /// it in every other pose's matrix row) and publishes a hard zero.
        std::vector<SdfPath> disabledPoseWeights;
    };
    std::vector<_PoseInterpolator> _poseInterpolators;
    /// Every weight property the phase publishes, flat and in publish order.
    /// Readers bind these property identities before common graph compilation.
    std::vector<SdfPath> _poseWeightProperties;

    /// Discovers, validates and SOLVES the rig's pose interpolators into
    /// \p out. Phase A of Compile: nothing here touches evaluator state.
    bool _CompilePoseInterpolators(
        const std::vector<SdfPath> &joints,
        const std::vector<SdfPath> &controls,
        std::vector<_PoseInterpolator> *out,
        std::vector<std::string> *notes,
        std::string *error, SdfPath *operation,
        RigExecSceneDescriptors *capturedScene = nullptr,
        RigExecProfiler *profile = nullptr) const;

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
        /// computeMatrix of binding.transformSpace, or -1 (matrix).
        /// computeMatrix of binding.carrySpace, or -1 (matrix): the rig's
        /// own rest->pose map, normally a TRS master's. The baked path's
        /// counterpart is carrySpaceSlot against the same base/final
        /// matrix table, so the two paths read the same value.
        /// Surface projector only: computeRestFrame of transform,
        /// transformSpace and carrySpace, for their world frames.
        /// computeMatrix per binding.influences entry, in that order (skin).
        /// The mover asked for the provider's FINAL frame, so its transform
        /// is the aim-revised matrix computed in memory rather than the
        /// provider's own tapped computeMatrix.
        bool transformFinalPhase = false;
        /// rigExec:pointFrame == "posed": the points this cluster moves
        /// have already been carried into the rig's posed frame by a skin,
        /// so the offset measured in its space has to be conjugated by
        /// the rig's carry -- or, with no rigExec:space named, by the
        /// measuring space's scale alone, which is all the correction
        /// there was before. See RigExecClusterInPointFrame.
        bool transformPosedPoints = false;
        /// Skin only: none of rigExec:jointIndices, jointWeights or
        /// elementSize can change within this epoch, so the layout may be
        /// resolved once and shared rather than re-read per frame. False
        /// whenever one of them is time-varying, carries an authored
        /// connection, or is written by a property chain.
        bool skinTopologyFixed = false;
    };
    /// Exact points target -> its revisions, in mover execution order.
    std::map<SdfPath, std::vector<_GraphRevision>> _graphChains;
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
    /// notice drops the entries it reaches (_ClearValueCaches).
    RigExecStaticInputCache _staticInputs;
    /// Uncommitted manipulation values; see SetInteractiveOverrides.
    std::vector<RigExecValueOverride> _interactiveOverrides;
    /// SetUpstreamInputs' list as given, the admitted part of it (sorted by
    /// path, one entry per attribute) and the drop lines each generation
    /// reports.
    std::vector<RigExecValueOverride> _upstreamRequested;
    std::vector<RigExecValueOverride> _upstreamAdmitted;
    std::vector<std::string> _upstreamDropLines;
    /// The time admission last judged condition 4 at (the last Evaluate's).
    UsdTimeCode _upstreamAdmissionTime = UsdTimeCode::Default();
    /// Condition 4's memo (path -> stage element count): filled by
    /// admission, dropped by any notice that reaches the path. Owning
    /// thread.
    RigExecUpstreamCountMemo _upstreamCounts;
    /// The admitted upstream values alone, by path: the chain base reads
    /// (runChain) answer from it and never from the interactive list.
    /// Rebuilt with the admitted list on the owning thread; read-only while
    /// a generation runs.
    std::map<SdfPath, VtValue> _upstreamValues;
    /// The values the dynamic walk places: the admitted upstream values
    /// merged with _interactiveOverrides, interactive winning on a key.
    /// Rebuilt by SetInteractiveOverrides, ClearInteractiveOverrides and
    /// every change of the admitted list; read wherever the walk decides
    /// what value an attribute holds or whether a cache still holds.
    std::vector<RigExecValueOverride> _valueInputs;

    std::map<SdfPath,std::set<SdfPath>> _nativePhaseCheckpoints;
    static bool _ValidateNativePhases(
        const std::map<SdfPath, std::vector<_GraphRevision>> &graphChains,
        const std::map<SdfPath, std::vector<SdfPath>> &frameChains,
        const std::vector<RigExecMoverRecord> &movers,
        std::map<SdfPath,std::set<SdfPath>> *checkpoints, _CompileFailure *failure);

    /// Per-epoch skin layouts, keyed by mover path.
    ///
    /// This is epoch state, not a value, and must not outlive the evaluator
    /// that read the stage it came from. Cleared by a change notice that reaches a
    /// layout input (_ClearValueCaches), by an interactive-override change
    /// that can reach one, and by the commit of a new epoch.
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
    bool _skinTopologyObservationPending = false;
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
    /// Same lifetime as _skinTopologies above; a notice drops a shape only
    /// where it reaches the sample, its blend shape or the points its
    /// layout was checked against (_BlendSampleSources).
    RigExecBlendSampleCache _blendSampleShapes;
    /// What each cached blend sample shape was read from: the sample prim,
    /// the UsdSkelBlendShape prim it names, and the points property of the
    /// chain it is applied to, whose length the layout was range-checked
    /// against. Derived from _graphChains, so it is rebuilt lazily after
    /// every commit (_blendSampleSourcesValid).
    struct _BlendSampleSource {
        SdfPath sample;
        SdfPath blendShape;
        SdfPath points;
    };
    std::vector<_BlendSampleSource> _blendSampleSources;
    bool _blendSampleSourcesValid = false;

    /// Drops what \p notice could have made stale in the value caches that
    /// outlive a generation -- the static inputs, the property chain
    /// bindings, the skin layouts, the blend sample shapes and the live
    /// graphs' pushed base points -- and nothing else (spec rule S5).
    void _ClearValueCaches(const UsdNotice::ObjectsChanged &notice,
                           bool avarValuesOnly);
    /// The same caches dropped whole, as every notice used to drop them.
    /// What a shadow evaluator runs (RIGEXEC_VERIFY_SCOPED_CLEARS), and what
    /// a notice at the root, on a resolved asset or inside a prototype runs.
    void _ClearValueCachesWholesale(bool avarValuesOnly);
    /// Set on the shadow evaluator RIGEXEC_VERIFY_SCOPED_CLEARS builds: its
    /// notices take _ClearValueCachesWholesale, never the scoped path.
    bool _wholesaleValueClears = false;
    /// The shadow itself (spec rule S6): an evaluator of the same rig on the
    /// same stage, told everything a caller tells this one, evaluated at
    /// every time this one is and compared with it pose for pose. Null
    /// unless the variable is set, and always null on the shadow.
    std::unique_ptr<RigExecRigEvaluator> _scopedClearShadow;
    /// Builds the shadow when the variable asks for one and there is none.
    void _EnsureScopedClearShadow();
    /// Evaluates the shadow at \p time and appends to \p pose every way
    /// the two disagree, counted on comparisonMismatches.
    void _VerifyScopedClears(UsdTimeCode time, RigExecRigPose *pose);
    /// Evaluate's body; Evaluate adds the shadow comparison around it.
    RigExecRigPose _EvaluateGeneration(UsdTimeCode time);
    /// One transform-valued input to a pose-domain constraint.
    ///
    /// RigExec providers publish computePointFrame and are therefore tapped;
    /// plain UsdGeomXformables have no such computation and are sampled from
    /// their native transform, asset-relative, during Evaluate(). Exactly one
    /// of frameTap/xformPath is populated by Compile().
    struct _FrameSourceBinding {
        SdfPath sourcePath;
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
        /// rigExec:space on a Rotation or Parent constraint -- the provider
        /// whose own rest->pose map carries the whole rig (the TRS masters
        /// on the biped). A partial axis mask needs it: picking Euler axes
        /// out of a rotation is not equivariant under an outer rotation, so
        /// under a turned master the mask throws part of the carry away
        /// with the axes it was told to drop. The same pair of taps the
        /// space switch reads (posed and default), so the two derive one
        /// carry. Both stay -1 when no space is named, and the kernel then
        /// runs its untouched branch.
        SdfPath spacePath;
        /// rigExec:blendShear (Scale, Parent): blend the sources' shear when
        /// every scale axis is governed. Off is the FBX behaviour.
        bool blendShear = false;
        /// rigExec:worldUpRotationOnly (Aim): orthonormalize the world-up
        /// object's frame before taking its rotation, so a scaled up object
        /// gives the same up direction as an unscaled one.
        bool worldUpRotationOnly = false;
        /// rigExec:weightBlend == "radial" on a transform-domain matrix
        /// mover: take a fraction of the rotation, as the point kernel does.
        bool radialBlend = false;
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
        /// Precomputed axis masks, authoritative only while `masksStatic`
        /// holds: no property chain revises a mask attribute, none is
        /// connected, and each is a single authored opinion. When it holds
        /// the per-frame mask reads are skipped; when it does not the values
        /// are ignored and the live read runs exactly as before.
        bool masksStatic = false;
        RigExecConstraintAxisMask precompTranslation;
        RigExecConstraintAxisMask precompRotation;
        RigExecConstraintAxisMask precompScale;
    };

    /// Detached switch structure and the provider paths needed by closure.
    struct _SpaceSwitch {
        std::shared_ptr<const RigExecSceneSpaceSwitchDescriptor> descriptor;
        SdfPath switchPath,target,spacePath;
        struct Source {SdfPath path;};
        std::vector<Source> sources;
    };
    /// Ordered discovery identities; common graph owns execution order.
    std::vector<_SpaceSwitch> _spaceSwitches;

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

    /// Bind property revisions and order their attribute/weight dependencies,
    /// and resolve the read phase of every connected operator input
    /// (_ForEachConnectedInput). \p movers and \p solvers name the
    /// operation a bad declaration sets aside; an input that only
    /// \p inertMovers or operations already set aside read is not resolved.
    bool _CompilePropertyChains(
        const std::vector<RigExecMoverRecord> &movers,
        std::map<SdfPath, std::vector<_PropertyRevision>> &chains,
        std::vector<SdfPath> &order,
        std::vector<RigExecPhasedConnection> &phased,
        const std::vector<UsdPrim> &solvers,
        const SdfPathVector &inertMovers,
        _CompileFailure *failure) const;
    /// Exact scalar property target -> its revisions, in mover execution order.
    std::map<SdfPath, std::vector<_PropertyRevision>> _propertyChains;
    /// Phased connections; see GetPhasedConnections.
    std::vector<RigExecPhasedConnection> _phasedConnections;

    /// Every attribute a property chain READS through a connection, so that
    /// an avar-only notice naming one of them still drops the chain
    /// bindings. See the comment beside it in Compile: a float chain input
    /// CAN be connected to a double avar, and the avar-only fast path was
    /// built on the assumption that it could not.
    std::set<SdfPath> _propertyChainInputs;

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
    /// A notice marks stale only the chains it reaches (_ClearValueCaches),
    /// and the next run rebinds those alone -- a query caches where a value
    /// comes from, and only a notice can move that. Dropped whole at the end
    /// of Compile, because a recompile may have replaced the chains the
    /// entries describe.
    std::unique_ptr<RigExecPropertyChainBindings> _propertyChainBindings;

    /// Samples an ordinary Xformable relative to the asset at the source boundary.
    bool _FrameFromXformRelativeToAsset(
        const UsdPrim &assetRoot, UsdGeomXformCache *xformCache,
        const SdfPath &path, RigExecPointFrame *outFrame,
        GfMatrix4d *outMatrix) const;

    /// Whether a SingleChainIK chain's joints can move with time at all.
    ///
    /// "autoDetect" asks this to choose between solving the chain as it
    /// stands and solving a rest-derived copy of it: a chain whose
    /// translations and scales are authored once has no animated Ts for the
    /// solver to preserve, and rebuilding its layout from rest is then the
    /// better-conditioned question. Structural -- a solver binding, a time
    /// sample or a connection -- so it is settled for the epoch.
    bool _IkUsesAnimatedTs(const std::vector<SdfPath> &chain) const;

    /// Re-pulls the epoch's rest frames after a stage edit no recompile
    /// covered, and clears _epochRestFramesStale. Returns false when the
    /// request could not produce them, which is what an incomplete per-frame
    /// rest tap used to mean, and leaves the flag set.
    /// Whether any of \p providers has a rest channel that can no longer be
    /// held as an epoch constant.
    /// The rest gate: records which epoch rest paths \p notice reached (see
    /// _restEditedProviders).
    void _NoteRestEdits(const UsdNotice::ObjectsChanged &notice);

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

    /// Volume weight objects whose rigExec:weightTarget reads `preceding`,
    /// which have to measure the IN-FLIGHT points at their own position
    /// in the mover stack rather than the authored base.
    std::set<SdfPath> _currentPhaseWeights;

    /// computeMatrix taps for every volumetric weight object reachable
    /// in this epoch, and the matrices the current generation resolved
    /// them to. The CPU oracle reads the resolved matrix rather than
    /// recomputing the xformable frame chain a second time.
    std::set<SdfPath> _nativeVolumeProviders;

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
    std::unique_ptr<RigExecGoldenSuiteObserver> _goldenSuiteObserver;
    std::unique_ptr<RigExecInputReplayObserver> _inputReplayObserver;
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
    /// Concrete admission reasons retained for the failed epoch.
    std::vector<std::string> _bakeRefusalReasons;
    size_t _bakedProgramBuilds = 0;
    size_t _bakedProgramBuildAttempts = 0;
    size_t _bakedGenerations = 0;
    /// Whether the last generation's region ran through _bakedProgram.
    bool _lastGenerationRanProgram = false;

    size_t _structureDigest = 0;
    /// What the digest in _structureDigest read outside the rig, which is
    /// what decides whether a notice sets _structureDirty. Committed with
    /// the digest at the end of Compile and again by every settle whose
    /// recomputed digest came out equal; invalid from the head of a Compile
    /// until its digest is joined, so a notice that arrives in between is
    /// suspect exactly as every notice was before the gate.
    _DigestGate _digestGate;
    /// Paths of the digest-suspect notices since the last settle that could
    /// make the edit certainly structural (_NoteCertainStructuralCandidates),
    /// judged against _digestGate by the next settle and dropped wherever
    /// _structureDirty is cleared or the gate is. Past a bound the list
    /// stops growing and the flag says the edit is not judged at all.
    std::vector<SdfPath> _certainCandidates;
    bool _certainCandidatesOverflowed = false;
    /// See GetStageEditSerial.
    uint64_t _stageEditSerial = 0;
    /// The last settle-path compile, when it failed.
    ///
    /// A compile's outcome is a function of the composed stage. Every notice
    /// bumps _stageEditSerial; public Compile forgets this memo and a
    /// successful compile clears it.
    ///
    /// \c errors is what the compile appended and nothing else: the
    /// settle's "structural recompilation failed" is the caller's line, and
    /// it adds it on a replay as it did the first time.
    struct _FailedCompileMemo {
        bool valid = false;
        uint64_t stageEditSerial = 0;
        std::vector<std::string> errors;
    };
    _FailedCompileMemo _failedCompile;
    /// See GetSkippedOperations. Rebuilt by every compile.
    std::map<SdfPath, std::string> _skippedOperations;
    /// The last notice's disposition and patched paths. Written by the
    /// notice handler on its own thread; read by the registry's notice
    /// adapter on the same thread (USD delivers notices synchronously),
    /// so no lock guards them.
    RigExecNoticeDisposition _lastNoticeDisposition =
        RigExecNoticeDisposition::None;
    std::vector<SdfPath> _lastNoticePatchedPaths;
    TfNotice::Key _noticeKey;
    bool _structureDirty = true;
    bool _compiled = false;
    /// Solver -> the start provider this compile derived for it from the
    /// joint hierarchy (rigExec:startFramePolicy = "parent"), as authored
    /// into the stage session layer. The map is what makes retraction
    /// exact: a recompile removes precisely these opinions before
    /// re-deriving, so a removed policy or a reparented chain leaves no
    /// stale target behind and a hand-authored session opinion is never
    /// touched.
    std::map<SdfPath, SdfPath> _derivedStartFrames;

    /// Scoped phase timings for Compile and Evaluate. Off unless profiling
    /// is enabled; see SetProfilingEnabled.
    RigExecProfiler _profiler;
    bool _opTimingEnabled = false;

    /// The program is built from the compiled epoch tables above, which are
    /// private because they are not a published surface -- not because the
    /// one class whose whole job is to flatten them should re-derive them.
    friend class RigExecBakedProgram;
    friend class RigExecInputReplayHeldProgram;
};

/// Temporarily changes upstream array admission for tests (on by default):
/// array values of the leaf reads a bake lists as array input slots.
/// Restore the previous setting after the test. Not for use while an
/// evaluation runs.
void RigExecSetUpstreamArrayAdmissionForTesting(bool on);

/// Lifts \p evaluator's upstream values for its lifetime and puts the same
/// list back when it ends. Lifting and placing them again are ordinary
/// upstream changes, so the next generation applies them with no rebuild.
class RigExecScopedUpstreamSuspension
{
public:
    explicit RigExecScopedUpstreamSuspension(RigExecRigEvaluator &evaluator)
        : _evaluator(evaluator), _saved(evaluator.GetUpstreamInputs())
    {
        _evaluator.SetUpstreamInputs({});
    }
    ~RigExecScopedUpstreamSuspension()
    {
        _evaluator.SetUpstreamInputs(std::move(_saved));
    }
    RigExecScopedUpstreamSuspension(const RigExecScopedUpstreamSuspension &) =
        delete;
    RigExecScopedUpstreamSuspension &operator=(
        const RigExecScopedUpstreamSuspension &) = delete;

private:
    RigExecRigEvaluator &_evaluator;
    std::vector<RigExecValueOverride> _saved;
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
