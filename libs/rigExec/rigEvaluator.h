//
// RigExec rig evaluator, v0.1-alpha.
//
// Transforms and solvers evaluate through OpenExec; the geometry mover
// chains evaluate through the in-memory RigExecMoverGraph. NOTHING is
// authored: the engine has no compiler, no generated prims, and no derived
// evaluation stage (spec §7.2 revised — the source stage is never written).
// A staged CPU implementation of the same kernels over the same composed
// post-order walk (spec §4.2) is retained behind cpuParityMode as the
// scalar parity reference.
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

/// One discovered mover application (spec §4.2): the composed post-order
/// ordinal plus canonicalized targets.
struct RigExecMoverRecord {
    SdfPath moverPath;
    TfToken schemaType;
    std::vector<SdfPath> targets;  ///< canonicalized (prim -> .points etc.)
    int ordinal = 0;
    bool enabledFallback = true;
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
    /// The BASE phase is the whole story for a control: controls are rig
    /// inputs, not outputs. Nothing in the pose domain writes them (a mover
    /// that did would make the animator's channel disagree with the thing
    /// they are dragging), so base and final are the same frame and only
    /// one is published. Consumed by the imaging bridge to place the
    /// synthesized control guides (spec §10.3 extension).
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

    /// Exact property path -> final computed native value
    /// (points/normals/extent as VtVec3fArray). Point chains evaluate
    /// through the generated OpenExec applications (spec §7.2).
    std::map<SdfPath, VtValue> movedProperties;

    /// CPU reference-kernel results for the same chains, filled only when
    /// RigExecRigEvaluator::cpuParityMode is set (scalar-reference
    /// parity, spec §7.4).
    std::map<SdfPath, VtValue> movedPropertiesCpu;

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

/// Compiles and evaluates one RigExecRig prim.
class RigExecRigEvaluator {
public:
    RigExecRigEvaluator(const UsdStageRefPtr &stage, const SdfPath &rigPath);
    ~RigExecRigEvaluator();

    /// Discovers joints and movers, validates targets, and prepares the
    /// exec tap set. Returns false with messages on validation failure.
    bool Compile(std::vector<std::string> *errors = nullptr);

    /// Evaluates one complete generation at an explicit time.
    RigExecRigPose Evaluate(UsdTimeCode time);

    /// Composed post-order mover applications (descendants first,
    /// composed child order; spec §4.2).
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
        UsdTimeCode time,
        std::vector<std::string> *diagnostics) const;

    bool _ResolveWeights(
        const SdfPath &weightPrimPath, size_t count, UsdTimeCode time,
        std::vector<float> *weights, std::string *error) const;

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
    /// Exact points target -> its revisions, in composed post-order.
    std::map<SdfPath, std::vector<_GraphRevision>> _graphChains;
    /// One pose-domain frame revision: an aim constraint on a transform
    /// provider, evaluated in memory instead of through a generated
    /// RigExecPointFrameMoverApplication.
    ///
    /// The kernel it replaces is two lines of math
    /// (RigExecApplyAimConstraint over the preceding frame), so the generated
    /// prim was carrying almost nothing except the wiring that named its
    /// inputs -- and that wiring is all resolvable from the mover itself.
    struct _FrameRevision {
        SdfPath moverPath;
        RigExecTapId aimTargetFrameTap = -1;  ///< aim target computePointFrame
    };
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
    /// Transform provider -> its aim revisions, in composed post-order.
    std::map<SdfPath, std::vector<_FrameRevision>> _frameChains;
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
    size_t _structureDigest = 0;
    bool _compiled = false;
};

}  // namespace rigExec

#endif  // RIGEXEC_RIG_EVALUATOR_H
