// RigExec baked program: the compiled epoch as a flat op list over dense
// slots, with no exec round trip on the per-frame path.
// The dynamic path re-derives the same numbers every frame through OpenExec
// requests, SdfPath-keyed maps and VtValue copies. Almost none of that work
// depends on the frame: on a rig whose shape is fixed for the epoch, the
// provider hierarchy, the default-space ladder, the solver rest descriptions,
// the constraint wiring and the descendant propagation pairs are all decided
// once. This class decides them once -- at Compile -- and leaves a per-frame
// program that reads only the inputs that actually vary, runs the same
// rigExecMath kernels in the same order over dense arrays, and publishes the
// same RigExecRigPose.
// It is a SECOND implementation of the evaluation semantics, so it is a
// request rather than a promise: a rig using any feature the program cannot
// express stays on the dynamic path, with a reason per feature. See
// IsBakeable.
#ifndef RIGEXEC_BAKED_PROGRAM_H
#define RIGEXEC_BAKED_PROGRAM_H

#include "bakedTrace.h"
#include "tapSet.h"

#include "rigExecMath/pointFrame.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/vt/value.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/usd/common.h"
#include "pxr/usd/usd/notice.h"
#include "pxr/usd/usd/timeCode.h"

#include "pxr/base/tf/type.h"
#include "pxr/usd/sdf/valueTypeName.h"

#include <cstddef>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {
class RigExecBlendSampleCache;

struct RigExecSceneDescriptors;
struct RigExecOpExclusionProof;

/// Whether a value of \p typeName fits a .rigexec input slot: a scalar,
/// token, 4x4 double matrix or 3-vector of either precision, any role.
/// Array types use RigExecUpstreamArraySlotType. Admission condition 2.
bool RigExecUpstreamSlotType(const SdfValueTypeName &typeName);

/// Whether \p typeName is an array type an upstream value may hold while
/// array admission is on (RigExecUpstreamArrayAdmission): an int, float,
/// double, vec2f or vec3f array, the element types a .rigexec array slot
/// carries.
bool RigExecUpstreamArraySlotType(const SdfValueTypeName &typeName);

/// Whether upstream array values are admitted (the array part of
/// admission conditions 2 and 3). On by default; tests can temporarily
/// change it with RigExecSetUpstreamArrayAdmissionForTesting.
bool RigExecUpstreamArrayAdmission();

/// Admission condition 4's memo, per array attribute: its stage element
/// count, answered with no stage read while the stage value cannot vary
/// (`varying` false: no time sample, asked when the entry was filled). An
/// entry with `varying` set holds no count: such a key reads the stage at
/// each time.
struct RigExecUpstreamCountEntry {
    size_t count = 0;
    bool varying = false;
};
using RigExecUpstreamCountMemo = std::map<SdfPath, RigExecUpstreamCountEntry>;

/// Why an upstream value \p value at attribute \p path is not admitted on
/// \p stage, or empty when it is: the attribute stands there, is
/// unconnected, has a stage value, has a slot type (condition 2) the value
/// holds exactly, (condition 3) \p listed, the standing program's
/// GetUpstreamAdmissible() or null when none stands, holds it, and
/// (condition 4, arrays) the value's element count equals the stage
/// value's at \p time. \p memo, when given, answers condition 4 for a key
/// the stage cannot vary and is filled the first time a key is judged;
/// with none the stage is read. The text is the "<reason>" of the
/// "upstream input <path>: <reason>; ignored" line. Owning thread: it reads
/// the stage.
std::string RigExecUpstreamDropReason(
    const UsdStageRefPtr &stage, const std::map<SdfPath, TfType> *listed,
    const SdfPath &path, const VtValue &value,
    UsdTimeCode time = UsdTimeCode::Default(),
    RigExecUpstreamCountMemo *memo = nullptr,
    const std::map<SdfPath, TfType> *listedArrays = nullptr);

/// One array attribute an upstream value may stand on: an array leaf read
/// that reads it through the upstream layer in every backend.
struct RigExecUpstreamArrayRow {
    /// Which reads take it: at the evaluated time, at Default (authored
    /// rest data), or both (a lattice cage, a wire curve). An upstream value
    /// replaces the attribute for both kinds.
    enum class Time { AtTime, AtDefault, Both };
    SdfPath path;
    /// The array type (VtIntArray, VtFloatArray, VtDoubleArray,
    /// VtVec2fArray or VtVec3fArray), the attribute's own.
    TfType type;
    Time time = Time::AtTime;
    /// The readers, comma separated: "chain base", "revision", "derived",
    /// "skin layout", "blend sample", "weight packet", "weight oracle".
    std::string consumer;
};

class RigExecRigEvaluator;

/// Every array attribute of \p evaluator's compiled epoch an upstream value
/// may stand on, sorted by path, one row per attribute: each chain's base
/// points; every array read a revision or derived target declares
/// (RigExecDeclareRevisionLeaves: topology, widths, surface points, bind
/// coordinates, rest points, pin points, lattice cages, wire curves, order,
/// knots and driver weights, skin jointIndices and jointWeights); dense blend
/// sample points; the point arrays a weight object's packet gathers, by
/// relationship target; and the point arrays the weight oracle reads, by
/// canonical attribute. Structural arrays are absent (painted weight
/// values and indices, solver and SplineIk arrays, constraint arrays,
/// property curve keys and tangents, ribbon solver points, derived bases,
/// sparse blend shapes), and so is every point array a weight object a
/// point revision binds (or one it composes) reads, whatever else reads
/// it: exec computes that packet in the dynamic walk and takes no array
/// override of an attribute it reads per element. The exporter lists exactly these as array input
/// slots. Owning thread: it reads the stage. Independent of
/// RigExecUpstreamArrayAdmission.
std::vector<RigExecUpstreamArrayRow> RigExecBakedUpstreamAdmissibleArrays(
    const RigExecRigEvaluator &evaluator);

struct RigExecRigPose;
/// The program's whole state, declared in bakedProgramImpl.h so that the
/// files building and running each domain of it can name the same type.
struct RigExecBakedProgramImpl;
struct RigExecWeightOracleFacts;
struct RigExecBakedEnvelopeObject;
struct RigExecBakedPropertyChainDesc;

/// Exact comparison of independently supplied poses, including published maps,
/// guides, operation counters and diagnostics. Appends disagreements to out.
/// Reference-only outputs and solver request counts are separate check domains.
void RigExecComparePoses(const RigExecRigPose &reference,
                         const RigExecRigPose &baked, RigExecRigPose *out);

/// One frame's stage-derived constraint seeds, in program order.
///
/// The stage-frame prologue of Run reads three things off the stage per
/// frame: the relative transforms seeding the XformDerived provider slots,
/// the ok/frame pair per native Xformable constraint source, and the
/// ok/matrix pair per geometry-domain delta base. A frozen worker cannot
/// read the stage, so the UI thread samples this struct through
/// SampleStageFrameSeeds and the seeds travel with the job, parallel to
/// xformSlots, nativeSources, and deltaBasePaths. Fresh stage data, not a
/// pure function of the sampled attribute values, so the control-state
/// digest folds it -- unlike the layout leaves and the other transports.
struct RigExecRequiredStageFramesAdmission {
    bool admitted = true;
    int32_t firstBadTarget = -1; // index into immutable xformSlots, not a path
    bool operator==(const RigExecRequiredStageFramesAdmission &o) const {
        return admitted == o.admitted && firstBadTarget == o.firstBadTarget;
    }
};

struct RigExecStageFrameSeeds {
    RigExecRequiredStageFramesAdmission requiredStageFramesAdmission;
    /// Per xformSlots entry: the relative transform and the frame, the
    /// pose's two currencies of the same seed. Typed admission records the
    /// first unavailable target. Failed/later entries remain captured values;
    /// the worker finishes pure preparation then declines publication.
    std::vector<GfMatrix4d> xformBase;
    std::vector<RigExecPointFrame> xformFrames;
    /// Per nativeSources entry: whether the stage answered, and the frame
    /// (default when it did not -- a source the stage cannot answer for is
    /// recorded and carried into the step, never a bail, as live).
    std::vector<char> nativeOk;
    std::vector<RigExecPointFrame> nativeFrames;
    /// Per deltaBasePaths entry: whether the stage answered, and the base
    /// matrix the geometry-domain delta is measured against.
    std::vector<char> deltaOk;
    std::vector<GfMatrix4d> deltaBase;
    std::vector<GfMatrix4d> intervening;
    std::vector<char> interveningReset;

    bool operator==(const RigExecStageFrameSeeds &o) const {
        return requiredStageFramesAdmission == o.requiredStageFramesAdmission &&
               xformBase == o.xformBase && xformFrames == o.xformFrames &&
               nativeOk == o.nativeOk && nativeFrames == o.nativeFrames &&
               deltaOk == o.deltaOk && deltaBase == o.deltaBase &&
               intervening == o.intervening && interveningReset == o.interveningReset;
    }
    bool operator!=(const RigExecStageFrameSeeds &o) const {
        return !(*this == o);
    }
};

enum class RigExecBakedBail { None, StageFrames, Step, Publish };

/// One compiled epoch, flattened.
class RigExecBakedProgram {
public:
    ~RigExecBakedProgram();

    RigExecBakedProgram(const RigExecBakedProgram &) = delete;
    RigExecBakedProgram &operator=(const RigExecBakedProgram &) = delete;

    /// Whether \p evaluator's compiled epoch can be expressed as a program,
    /// appending one reason per feature that cannot.
    ///
    /// The reasons are the point: a rig that will not bake should say which
    /// of its features stopped it, because "it fell back" is not actionable
    /// and a silent fallback reads as the mode not working.
    static bool IsBakeable(const RigExecRigEvaluator &evaluator,
                           std::vector<std::string> *reasons);

    /// Builds the program from \p evaluator's compiled epoch, or returns
    /// null with the reasons when the epoch is not bakeable.
    ///
    /// \p evaluator must outlive the program; the evaluator owns it and drops
    /// it whenever the epoch or the scene changes underneath.
    static std::unique_ptr<RigExecBakedProgram> Build(
        RigExecRigEvaluator *evaluator, std::vector<std::string> *reasons);

    /// Bake-time facts for a port of the evaluator's weight oracle, the
    /// envelope objects no WeightPacket step bakes, and the property
    /// chains. Members only because the evaluator's friendship reaches
    /// this class; called as RigExecBakedDescribeWeightOracle,
    /// RigExecBakedComposeEnvelopeObjects and
    /// RigExecBakedDescribePropertyChains (bakedProgramImpl.h), which
    /// document them.
    static void DescribeWeightOracle(const RigExecRigEvaluator &evaluator,
                                     const SdfPath &path, UsdTimeCode time,
                                     RigExecWeightOracleFacts *facts);
    static bool ComposeEnvelopeObjects(
        const RigExecRigEvaluator &evaluator,
        const RigExecBakedProgramImpl &program,
        std::vector<RigExecBakedEnvelopeObject> *objects,
        std::map<SdfPath, int> *index, std::string *error);
    static bool DescribePropertyChains(
        const RigExecRigEvaluator &evaluator,
        std::vector<RigExecBakedPropertyChainDesc> *chains,
        std::string *error);
    /// RigExecBakedUpstreamAdmissibleArrays.
    static std::vector<RigExecUpstreamArrayRow> DescribeUpstreamArrays(
        const RigExecRigEvaluator &evaluator);

    /// Runs the whole program at \p time and publishes into \p pose.
    ///
    /// Returns false with an invalid generation when execution cannot complete.
    bool Run(UsdTimeCode time, RigExecRigPose *pose);

    /// Asks the next Run to execute every step rather than the closure of
    /// what moved, then forget the request. A request, not state: the
    /// results are the same either way (the cone verifier's invariant);
    /// what changes is that every step's reads happen in that run, which a
    /// bake capturing them needs. Const because the evaluator hands its
    /// program out const; the flag is the only thing it touches.
    RigExecBakedBail GetLastBail() const { return _lastBail; }

    void RequestFullRun() const { _fullRunRequested = true; }

    /// Samples the stage-frame prologue's reads at \p time into \p seeds.
    ///
    /// The same three loops Run's stageFrames runs -- one UsdGeomXformCache
    /// built fresh for the frame, the same shared reader over the same
    /// slots and paths -- but writing into \p seeds instead of the
    /// program's per-frame state, which is untouched. UI thread only: the
    /// reader walks the live stage.
    ///
    /// An unavailable required target records typed refusal and the successful
    /// ordered prefix. The worker runs independent preparation and refuses
    /// publication at the same boundary as live. Native and delta sampling
    /// occurs only for admitted frames. False reports malformed arguments.
    bool SampleStageFrameSeeds(UsdTimeCode time,
                               RigExecStageFrameSeeds *seeds,
                               std::string *error = nullptr) const;

    /// Whether \p notice can have moved anything the bake captured.
    ///
    /// The epoch digest is deliberately blind to values, so it cannot answer
    /// this: it is unchanged by exactly the edits that make a captured
    /// constant wrong. The bake therefore records WHICH properties it read
    /// and which prims it read them from, and this asks that index. An edit
    /// that misses it -- a value on an input the program re-reads every
    /// frame, or anything on a prim the bake never looked at -- leaves the
    /// program standing, which is the whole point of having an index rather
    /// than dropping the program on every notice. Anything but a value on a
    /// property the bake named -- a connection or a target retargeted --
    /// hits it: the walk the bake recorded for that property is what routes
    /// later value edits (ApplyValueEdits), and only a rebuild re-records it.
    bool IsInvalidatedBy(const UsdNotice::ObjectsChanged &notice) const;

    /// Absorbs \p notice without a rebuild when it is nothing but new
    /// DEFAULT VALUES on avars the bake captured as constants, and says
    /// whether it did.
    ///
    /// This is what an Avar Editor slider, a typed value and a released
    /// gizmo in Default mode all author, one value at a time. Before this
    /// each of them rebuilt the whole program (~120 ms on the biped) because
    /// a captured constant is in the rebuild index -- to change one double
    /// in a dense table. Here the new value is written straight into that
    /// table's slot; the frame path already compares every avar slot against
    /// last run's by value, so the next generation re-runs exactly the cone
    /// of what moved and nothing else.
    ///
    /// All-or-nothing: returns false, having changed NOTHING, the moment any
    /// part of the notice is something else -- a resync, a field other than
    /// the default, a non-avar property the bake read, an avar reached
    /// through a connection, or one that has just become animated. The
    /// caller then takes the rebuild path exactly as before.
    bool ApplyAvarValueEdits(const UsdNotice::ObjectsChanged &notice);

    /// Whether \p path is one of the patchable avar properties: a constant
    /// binding whose value a notice can PATCH rather than rebuild (see
    /// RigExecBakedProgramImpl::patchableAvars).
    bool IsPatchableAvarPath(const SdfPath &path) const;

    /// The read-only half of ApplyAvarValueEdits (plan 2.1): decides the
    /// patch without writing anything, answering whether the notice is
    /// nothing but patchable avar default values and, in \p patchedPaths,
    /// the property paths it would patch. ApplyAvarValueEdits answers
    /// true exactly when this does; the evaluator's per-notice
    /// disposition query classifies through this, so the classification
    /// and the mutation can never disagree.
    bool DryRunAvarValueEdits(const UsdNotice::ObjectsChanged &notice,
                              std::vector<SdfPath> *patchedPaths) const;

    /// Tells the program that a notice it was NOT invalidated by still
    /// reached the stage.
    ///
    /// IsInvalidatedBy answers "could this have moved anything the bake
    /// captured", and its documented gap is the other half: a value edit on
    /// an input the frame path re-reads changes what the next generation
    /// must compute while leaving every captured constant right. Cone
    /// re-execution (§7) decides what to re-run from the SOURCES it compares
    /// by value, and a stage edit is the one thing no source of the program
    /// compares -- so the evaluator says so here, and the next run runs
    /// everything once. Interactive overrides do NOT call this: an override
    /// is a source value like any other and is compared like one.
    void BumpProgramStamp();

    /// Routes \p notice to the per-frame inputs it reached, when it is
    /// nothing but new VALUES the program can place, and says whether it
    /// did (unified-program spec rules S2, S3).
    ///
    /// An edit on an input a step reads every frame is answered the way the
    /// run after a drag is lifted is: the input's override number is marked
    /// edited, and the next run re-runs the steps that declare it and their
    /// cone -- instead of BumpProgramStamp's one run of everything. An edit
    /// on a property the program reads through a value-compared source (a
    /// property chain, a mover assembled by RevisionStatic, a stage
    /// transform) needs nothing, because the source compares it, and an
    /// edit on a property nothing reads needs nothing at all.
    ///
    /// All-or-nothing, and to be asked only of a notice IsInvalidatedBy
    /// declined: returns false, having changed NOTHING, for any resync, for
    /// layer metadata, for a field other than a value, for a property the
    /// bake named but no step declares, and for an input no reader re-reads.
    /// The caller then bumps the stamp.
    bool ApplyValueEdits(const UsdNotice::ObjectsChanged &notice);

    /// The read-only half of ApplyValueEdits: whether it would route
    /// \p notice, and in \p readPaths the property paths the notice names
    /// that anything in the program can read -- the paths a frame cache must
    /// still retire for. A path read by nothing is left out.
    bool DryRunValueEdits(const UsdNotice::ObjectsChanged &notice,
                          std::vector<SdfPath> *readPaths) const;

    /// Places the standing interactive overrides for the generations that
    /// follow, returning false when one of them names something the program
    /// cannot place.
    ///
    /// A placeable override is one the program can route the way the dynamic
    /// path does: an input the frame path reads (it is read the long way,
    /// through the generation's resolved inputs, while the override stands),
    /// or a property whose only reader already goes through those resolved
    /// inputs -- a property-chain mover, a geometry mover. An override on a
    /// value folded into bake state (a rest, a default-space ladder, a
    /// solver rest description) cannot be placed without rebaking, and a
    /// computation override names something that only exec can answer; both
    /// return false so the caller runs the generation dynamically. A wrong
    /// baked answer is never one of the outcomes.
    bool SetOverrides(const std::vector<RigExecValueOverride> &overrides);

    /// This run's admitted upstream values (RigExecRigEvaluator::
    /// SetUpstreamInputs), sorted by path, attribute entries only. They
    /// are authored-level: the next run's prologue compares them by value
    /// with what the last run placed and re-reads only the leaves under a
    /// value placed, moved or lifted; every such leaf reads through the
    /// upstream layer (RigExecResolvedInputs::GetAttributeOverStageLayer).
    /// A standing value that did not move costs nothing.
    void SetUpstreamInputs(const std::vector<RigExecValueOverride> &inputs);
    /// Owner-thread handover of successfully recomputed sparse layout handles.
    /// Call only after Run has joined; frozen workers never access this cache.
    void AdoptBlendSampleLayouts(RigExecBlendSampleCache *cache);

    /// Path -> value type of every attribute a read the bake lists as an
    /// input slot walks: the hops of every registered binding, of the
    /// property chains (targets, revision inputs, phased consumers and
    /// their hops), of the envelope-only weight objects' six reads and of
    /// the geometry assembly's scalar reads. Admission condition 3.
    /// Built on first use, on the owning thread, and kept for the program.
    const std::map<SdfPath, TfType> &GetUpstreamAdmissible() const;
    /// Every hop of every scalar read of a weight object the volatile
    /// oracle resolves (a constraint or property-mover envelope, a
    /// current-phase field, and every object those compose). An upstream
    /// value at one of these paths is placed into the generation's resolved
    /// inputs too, which is what the oracle reads. Built with the above.
    const std::set<SdfPath> &GetUpstreamOracle() const;

    /// Whether a run resolves RigExecRigPose::weightFields, following
    /// RigExecRigEvaluator::SetPublishWeightFields: the per-point overlay
    /// field is walked for every weighted mesh only when someone looks.
    void SetPublishWeightFields(bool publish);

    /// How many clusters the program's schedule holds, and how many of them
    /// the last generation actually ran.
    ///
    /// Observable so a test can hold cone re-execution to account in both
    /// directions: a frame that skipped nothing is a cone that is not
    /// working, and a frame that skipped something has to publish what the
    /// dynamic path publishes anyway.
    size_t GetClusterCount() const;
    size_t GetClustersRunLastGeneration() const;

    /// Dense provider slots in namespace DFS order.
    size_t GetProviderCount() const;
    /// Input channels the program reads from USD at all.
    size_t GetBoundInputCount() const;
    /// Of those, the ones re-read every frame; the rest are epoch constants.
    size_t GetVaryingInputCount() const;

    /// Takes over \p previous's persistent geometry state.
    ///
    /// Retains compatible native geometry revision state across an epoch rebuild.
    void AdoptGeometryStateFrom(RigExecBakedProgram &previous);

    /// Drops every reference the program holds that keeps the stage alive:
    /// the stage pointer itself and the blend-sample resolver, which
    /// captured a copy of it.
    ///
    /// For a RETIRING program only, which runs nothing afterwards: its prims,
    /// attributes and queries stay, and they do not own the stage. The
    /// evaluator calls this on its own thread before handing the program to
    /// a detached destroy, so the last reference to a stage can never be
    /// dropped -- and the stage torn down -- on a worker behind the host's
    /// back.
    void ReleaseStageReferences();

    /// The step graph this program runs.
    ///
    /// The graph IS the program's structure, so the suite that asserts its
    /// invariants -- every edge forward, every read written or sourced, no
    /// two steps writing the same slot without an edge -- has to be able to
    /// see it. RigExecBakedProgramImpl is declared in bakedProgramImpl.h,
    /// which only this library's own sources and its tests include.
    const RigExecBakedProgramImpl &GetStepGraph() const;

    /// The steps the last run executed, in completion order, and the step
    /// graph as plain records (bakedTrace.h). Observability only.
    std::vector<RigExecOpTraceEntry> GetLastOpTrace() const;
    std::vector<RigExecOpGraphNode> GetOpGraph() const;

private:
    // Test-only; reached through RigExecBakedProgramTesting
    // (bakedProgramImpl.h), because only this class is the evaluator's friend.
    friend struct RigExecBakedProgramTesting;
    friend class RigExecRigEvaluator;
    /// Project the common compiler's exact SCC members to authored operation
    /// owners for the evaluator's public compile report. Neutral Build is unchanged.
    std::map<SdfPath, std::string> _GetCycleSkips(
        const std::map<SdfPath, SdfPath> &switchOwners) const;
    /// Only the synchronous compile caller can offer its local capture.
    static std::unique_ptr<RigExecBakedProgram> _BuildWithSceneCapture(
        RigExecRigEvaluator *evaluator, std::vector<std::string> *reasons,
        const RigExecSceneDescriptors *scene, const UsdStageWeakPtr &sceneStage,
        uint64_t sceneSerial);
    static std::unique_ptr<RigExecBakedProgram> _BuildWithSceneCaptureAttempt(
        RigExecRigEvaluator *, std::vector<std::string> *,
        const RigExecSceneDescriptors *, const UsdStageWeakPtr &, uint64_t,
        const std::set<SdfPath> &excludedPoseWriters,
        const std::set<SdfPath> &excludedSolverBodies,
        const RigExecOpExclusionProof &exclusionProof,
        const std::map<SdfPath, std::string> &cycleSkipReasons);
    /// Binds \p evaluator's property chains into \p program's head tier
    /// (bakedProperties.cpp): chains, records, walks, head leaves and
    /// override slots. Build only.
    static void _BindPropertyChains(const RigExecRigEvaluator &evaluator,
                                    RigExecBakedProgramImpl *program);

    explicit RigExecBakedProgram(std::unique_ptr<RigExecBakedProgramImpl> impl);
    std::unique_ptr<RigExecBakedProgramImpl> _impl;
    /// RequestFullRun's one-shot flag, consumed by the next Run.
    mutable bool _fullRunRequested = false;
    RigExecBakedBail _lastBail = RigExecBakedBail::None;
};

}  // namespace rigExec

#endif  // RIGEXEC_BAKED_PROGRAM_H
