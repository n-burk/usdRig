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

#include "moverGraph.h"
#include "tapSet.h"
#include "types.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/vt/array.h"
#include "pxr/usd/sdf/layer.h"

#include <map>
#include <set>
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

    /// Chains where the compiled mover graph disagreed with the
    /// generated-prim result, and chains where it agreed.
    ///
    /// Machine-checkable on purpose. While the graph publishes and the
    /// generated-prim chains run alongside as the reference, the ONLY thing
    /// standing between a packet-assembly drift and shipped-wrong geometry
    /// is this comparison -- and a regression already reached usdview because
    /// the signal was a diagnostic string tests had to grep. A count that
    /// must be zero, and a count that must be non-zero, cannot be missed the
    /// same way: "checked nothing" and "everything agreed" stop looking
    /// alike.
    size_t moverGraphParityMismatches = 0;
    size_t moverGraphParityAgreements = 0;

    /// Refinement rounds the solver->joint overrides needed to reach a fixed
    /// point, and whether they reached one at all.
    ///
    /// More than one round means some solver consumed a joint that another
    /// solver poses (RigExecTwistDistribution reading rigExec:start/end is
    /// the case that exists today). Exposed as a number because parity cannot
    /// police it: both the graph and the lowered path read the same override,
    /// so a stale one makes them agree on the same wrong answer.
    size_t solverOverrideRounds = 0;
    bool solverOverridesConverged = true;
};

/// Compiles and evaluates one RigExecRoot prim.
class RigExecRigEvaluator {
public:
    RigExecRigEvaluator(const UsdStageRefPtr &stage, const SdfPath &rigPath);
    ~RigExecRigEvaluator();

    /// Discovers joints and movers, validates targets, and prepares the
    /// exec tap set. Returns false with messages on validation failure.
    bool Compile(std::vector<std::string> *errors = nullptr);

    /// Evaluates one complete generation at an explicit time.
    RigExecRigPose Evaluate(UsdTimeCode time);

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

    /// When set, Evaluate also runs the CPU reference kernels for the
    /// lowered point chains into RigExecRigPose::movedPropertiesCpu.
    bool cpuParityMode = false;

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

    VtVec3fArray _EvaluateChain(
        const SdfPath &target,
        const std::vector<const RigExecMoverRecord *> &chain,
        const RigExecRigPose &pose,
        const std::map<SdfPath, GfMatrix4d> &baseProviderMatrices,
        const std::map<SdfPath, GfMatrix4d> &finalProviderMatrices,
        UsdTimeCode time,
        std::vector<std::string> *diagnostics) const;

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

    size_t _ComputeStructureDigest() const;

    UsdStageRefPtr _stage;
    SdfPath _rigPath;
    std::unique_ptr<RigExecTapSet> _taps;
    /// Observational solver-guide taps in their own prepared request: a
    /// failing or unused aggregate solver degrades guide drawing with a
    /// diagnostic instead of invalidating the rig snapshot.
    std::unique_ptr<RigExecTapSet> _guideTaps;

    std::vector<SdfPath> _jointPaths;
    /// Every RigExecControl beneath the rig, discovered exactly the way the
    /// joint outputs are (spec §4.1: the rig is a namespace root, not a
    /// manifest). Unlike the joints an empty set is legal -- a rig driven
    /// entirely by avars on its joints has no control prims at all -- so it
    /// never fails Compile.
    std::vector<SdfPath> _controlPaths;
    /// Base computePointFrame per control, parallel to _controlPaths.
    std::vector<RigExecTapId> _controlFrameTaps;
    /// Solver->joint binding, held in memory rather than authored.
    ///
    /// This is what Pass 0 used to write onto each joint as
    /// rigExec:frameSource / rigExec:frameElement. It is a compile-time
    /// choice -- which solver poses this joint, and which element of its
    /// aggregate frame array is this joint's -- so it belongs to the
    /// compiled graph, not to the scene. Compile() derives it from each
    /// solver's ordered rigExec:joints, which the Phase A validation
    /// already walks; Evaluate() indexes the solver's frame array with it
    /// directly instead of going through the joint's computePointFrame.
    std::map<SdfPath, std::pair<SdfPath, int>> _jointSolverBinding;
    /// computePointFrameArray taps for the solvers that pose joints, in
    /// their own request (_solverFrameTaps): they must evaluate BEFORE the
    /// authoritative request, whose joint values are overridden with frames
    /// extracted from them. Distinct from the observational
    /// _solverArrayTaps, which only feed guide drawing.
    std::unique_ptr<RigExecTapSet> _solverFrameTaps;
    std::map<SdfPath, RigExecTapId> _jointSolverArrayTaps;
    /// One TwoBoneIk solver with an unauthored absolute length: the bone
    /// is measured from the bound joints' rest positions at Evaluate time
    /// (root to mid for upper, mid to end for lower), plus the authored
    /// length offset. An authored absolute length is exact and implies
    /// nothing. Records are structural (which solver measures from which
    /// joints); rest positions and offset values are read live, so rest
    /// edits need no recompile -- that is the point of implying them.
    struct _ImpliedIkLengths {
        SdfPath solver;
        /// Elements 0, 1, 2 (root, mid, end). Empty when the solver binds
        /// fewer than three elements, in which case nothing is implied.
        SdfPath joints[3];
        bool implyUpper = false;
        bool implyLower = false;
    };
    std::vector<_ImpliedIkLengths> _impliedIkLengths;
    /// computeRestFrame taps for the joints above, in their own request:
    /// rest frames are pure attribute reads, so they evaluate before the
    /// solver aggregates whose length inputs they supply.
    std::unique_ptr<RigExecTapSet> _restFrameTaps;
    std::map<SdfPath, RigExecTapId> _impliedRestTaps;
    std::vector<RigExecTapId> _jointFrameTaps;
    std::vector<RigExecTapId> _jointFinalFrameTaps;
    std::vector<RigExecTapId> _jointFinalMatrixTaps;
    std::map<SdfPath, RigExecTapId> _solverArrayTaps;

    /// Compiled mover-graph inputs, running alongside the generated-prim
    /// chains while the graph path is being proven equal to them.
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
        RigExecTapId weightTap = -1;
        RigExecTapId driverFramesTap = -1;
        /// The mover asked for the provider's FINAL frame, so its transform
        /// is the aim-revised matrix computed in memory rather than the
        /// provider's own tapped computeMatrix.
        bool transformFinalPhase = false;
        /// computeBlendChannel per resolved blend input, in the canonical
        /// sorted input order the packet is accumulated in (spec §7.3).
        std::vector<RigExecTapId> blendChannelTaps;
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
        /// Non-empty when the constraint writes the GEOMETRY domain: the
        /// <prim>.points property it revises. targets[0] stays the owning
        /// PRIM path either way, because every frame-domain map -- base and
        /// rest frames, the provider classifier, the frame chains -- is keyed
        /// by prim. The domain decides where the answer is published, not how
        /// it is solved.
        SdfPath pointsTarget;
        /// Optional per-element weight field for the geometry domain. Empty
        /// means the constant packet synthesized from inputs:defaultWeight.
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
        std::vector<std::string> *diagnostics) const;
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
    /// Provider -> computeRestFrame tap, for the paired final matrix.
    std::map<SdfPath, RigExecTapId> _providerRestFrameTaps;
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

    size_t _structureDigest = 0;
    bool _compiled = false;
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
