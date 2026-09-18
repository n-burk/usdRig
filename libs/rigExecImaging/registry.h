//
// RigExec imaging registry: process-global rendezvous between the
// UsdImaging scene-index plugin (which builds filter chains when engines
// construct) and the application activation (which owns the stage,
// evaluator, and time source). Order-independent: chains read the shared
// snapshot store atomically, so activation may happen before or after
// chain construction.
//
#ifndef RIGEXEC_IMAGING_REGISTRY_H
#define RIGEXEC_IMAGING_REGISTRY_H

#include "bridge.h"
#include "sceneIndices.h"
#include "snapshotStore.h"

#include "pxr/base/tf/notice.h"
#include "pxr/base/tf/weakBase.h"
#include "pxr/usd/usd/notice.h"
#include "pxr/usd/usdGeom/xformCache.h"

#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

namespace rigExec {

class RigExecImagingRegistry : public TfWeakBase {
public:
    static RigExecImagingRegistry &GetInstance();

    /// The shared store every results scene index reads from.
    const std::shared_ptr<RigExecSnapshotStore> &GetStore() const {
        return _store;
    }

    /// Called by the scene-index plugin for each constructed chain.
    void RegisterChain(
        const RigExecInternalPrimPruningSceneIndexRefPtr &pruning,
        const RigExecBindingResolvingSceneIndexRefPtr &binding,
        const RigExecResultsSceneIndexRefPtr &results,
        const RigExecXformOverrideSceneIndexRefPtr &xforms = nullptr);

    /// Activates evaluation for one rig, or every RigExecRoot when rigPath is
    /// empty.  Compilation and the first evaluation complete off to the side;
    /// the active stage and Hydra generation change only after every rig has
    /// succeeded.
    bool Activate(
        const UsdStageRefPtr &stage, const SdfPath &rigPath,
        UsdTimeCode initialTime, std::vector<std::string> *errors);

    /// Serialized evaluate-then-publish, broadcast to every chain.
    bool SetTime(UsdTimeCode time);

    /// Number of actual evaluator pulls for one active session, for profiling.
    size_t GetSessionEvaluationCount(const SdfPath &rigPath);

    /// Selects the weight object painted as the influence overlay, and
    /// republishes at the current time so the viewport updates without
    /// waiting for a frame change. An empty string turns the overlay off.
    ///
    /// The selection is remembered even with no bridge activated, so a
    /// host that sets it before opening a rig gets the overlay on the
    /// first generation rather than none at all.
    bool SetWeightOverlay(const std::string &weightPrimPath);

    /// Declares what one manipulation is going to change, and returns how
    /// many doubles UpdatePreview will expect (-1 when the declaration is
    /// rejected). \p packedAttributePaths is newline-separated absolute
    /// attribute paths.
    ///
    /// Declared once per drag so the per-sample call can be nothing but
    /// numbers. Resolving a path to a rig session, reading its value type, and
    /// deciding which lane it previews through are all done HERE, once,
    /// because the interactive cost of a manipulation is the cost of the thing
    /// that runs per mouse sample.
    ///
    /// Two lanes, chosen per attribute and invisible to the caller:
    ///
    ///   * an attribute under an active rig previews through the evaluator,
    ///     because moving a control has to re-run the rig
    ///     (RigExecRigEvaluator::SetInteractiveOverrides);
    ///   * an xformOp on a prim no rig drives previews through
    ///     RigExecXformOverrideSceneIndex, which is the same prim's transform
    ///     and nothing else.
    int BeginPreview(const std::string &packedAttributePaths);

    /// One mouse sample: the declared slots' values, flattened in declaration
    /// order -- 1 double for a scalar, 3 for a vector, 16 for a matrix.
    /// Evaluates and republishes. Authors nothing.
    bool UpdatePreview(const double *values, size_t count);

    /// Writes every active rig's recorded profile totals to \p path and
    /// clears them. See RigExecImaging_WriteProfileSummary.
    bool WriteProfileSummary(const std::string &path);

    /// Ends the manipulation: drops every override and delta and republishes
    /// the authored rig. The application authors the committed values itself,
    /// before or after this call -- the two are independent, which is why an
    /// aborted drag is this call alone.
    bool EndPreview();

    bool IsPreviewActive() const;

    /// Drops the bridge; chains remain and read the (cleared) store.
    void Deactivate();

    bool IsActive() const { return !_sessions.empty(); }

    /// Activates the stage's rigs unless this exact stage is already active
    /// (adapter-driven discovery, docs/specs/imaging-datasource-redesign.md
    /// §3.2).
    ///
    /// The stage scene index's rig adapter calls this the first time a
    /// RigExecRoot prim's data is built, so a host with no explicit
    /// activation (usdrecord) still evaluates. A no-op when already active
    /// on \p stage -- hosts that activate explicitly (the usdview plugin)
    /// are unaffected -- and otherwise exactly Activate with an empty rig
    /// path: every RigExecRoot on the stage, compiled and published at
    /// \p time.
    ///
    /// NOT a substitute for Activate: a rig authored into an already-active
    /// stage still needs an explicit re-activation, which is what the
    /// usdview plugin's root-set tracking does.
    ///
    /// Refuses (silently returns false) while active on a DIFFERENT stage:
    /// unlike an explicit Activate, the automatic path never steals a live
    /// activation. The refusal is silent because the adapter calls this on
    /// every data pull -- a second stage's rigs would otherwise warn once
    /// per pull, forever.
    bool EnsureActivated(
        const UsdStageRefPtr &stage, UsdTimeCode time);

    /// Whether \p path is an active rig root. The results index's
    /// _PrimsDirtied time trigger only fires for these, so output-prim
    /// dirties (including the universal ones from epoch swaps) never
    /// re-enter evaluation.
    bool IsActiveRigRoot(const SdfPath &path);

    /// Records a RigExecRoot sighting for eager activation. The rig adapter
    /// calls this from GetImagingSubprims, which the stage scene index runs
    /// for every prim during populate -- long before any data pull -- so the
    /// results index's _PrimsAdded can force the rig's data (and its
    /// time-varying flag, and its activation) into existence on the populate
    /// thread instead of whichever render worker pulls first.
    ///
    /// Always records, even while active: GetImagingSubprims runs inside
    /// GetChildPrimPaths walks the xform-override index makes WHILE HOLDING
    /// _mutex (a preview's _DirtyXformSubtree), so this must never take it.
    /// The set only holds distinct rig paths, so recording while active is
    /// bounded and harmless -- the activation it forces resolves to a no-op
    /// inside EnsureActivated. A rig authored into an already-active stage
    /// is still the usdview plugin's re-activation to make: the forced pull
    /// builds its data but EnsureActivated refuses to replace a live stage
    /// (see below). Cleared by Activate and Deactivate; a note that
    /// outlives a failed activation retries on the next resync, like the
    /// plugin's own retry.
    ///
    /// Paths only, no stage: the note is just the _PrimsAdded matcher, and
    /// the activation it forces runs through the pulled prim's own adapter,
    /// which always names the true stage. A path collision across two stages
    /// therefore still activates the right one.
    void NoteRigRoot(const SdfPath &rigPath);

    /// Whether \p path was noted by NoteRigRoot and is still pending.
    bool IsNotedRigRoot(const SdfPath &path);

private:
    RigExecImagingRegistry();

    // Weak references: chains are owned by their scene index graphs and
    // die with their engines; the registry prunes expired entries.
    struct Chain {
        TfWeakPtr<RigExecInternalPrimPruningSceneIndex> pruning;
        TfWeakPtr<RigExecBindingResolvingSceneIndex> binding;
        TfWeakPtr<RigExecResultsSceneIndex> results;
        TfWeakPtr<RigExecXformOverrideSceneIndex> xforms;
    };

    struct RigSession {
        SdfPath rigPath;
        SdfPath assetRoot;
        std::shared_ptr<RigExecSnapshotStore> store;
        std::unique_ptr<RigExecImagingBridge> bridge;
        RigExecBindingResolvingSceneIndex::BindingEpochConstPtr epoch;
        std::set<SdfPath> readRoots;
        bool readRootsDirty = true;
        bool dirty = true;
        size_t evaluationCount = 0;
    };

    using RigSessions = std::vector<RigSession>;

    bool _EvaluateSessions(
        RigSessions *sessions,
        const UsdStageRefPtr &stage,
        UsdTimeCode time,
        std::shared_ptr<RigExecImagingSnapshot> *snapshot,
        RigExecBindingResolvingSceneIndex::BindingEpochConstPtr *epoch,
        std::vector<std::string> *errors);

    RigExecImagingBridge::PublishResult _Publish(
        std::shared_ptr<RigExecImagingSnapshot> snapshot,
        const RigExecBindingResolvingSceneIndex::BindingEpochConstPtr &epoch);

    /// Broadcasts a publication to every chain. Call WITHOUT _mutex held:
    /// the sends re-enter this registry (the results index's time trigger),
    /// so _Broadcast snapshots the chain list under a short lock and sends
    /// outside it.
    void _Broadcast(const RigExecImagingBridge::PublishResult &result);
    void _RefreshReadRoots();

    /// Edit-driven re-evaluation: any authored change touching the rig's
    /// asset or transitive external read dependencies re-evaluates at the
    /// last-set time and republishes, so
    /// property edits redraw exactly like timeline changes.
    void _OnObjectsChanged(
        const UsdNotice::ObjectsChanged &notice,
        const UsdStageWeakPtr &sender);

    /// One declared value of a manipulation in progress; see BeginPreview.
    struct PreviewSlot {
        SdfPath primPath;
        TfToken attributeName;
        /// The rig this attribute previews through, or empty for the xform
        /// lane. Resolved once, at declaration: a drag does not change which
        /// rig a prim belongs to.
        SdfPath rigPath;
        /// double | float | vec3d | vec3f | matrix4d -- how to turn this
        /// slot's doubles back into the attribute's own type.
        TfToken valueKind;
        size_t arity = 1;
    };

    /// Turns one sample's doubles into the per-rig override vectors and the
    /// per-prim xform deltas. Stage reads happen here, off the Hydra thread.
    bool _ResolvePreviewSample(
        const double *values, size_t count,
        std::map<SdfPath, std::vector<RigExecValueOverride>> *byRig,
        std::map<SdfPath, GfMatrix4d> *xformDeltas) const;

    /// The composed world delta for \p primPath given its overridden ops.
    bool _ComposeXformDelta(
        const UsdPrim &prim,
        const std::map<TfToken, VtValue> &opValues,
        UsdGeomXformCache *cache,
        GfMatrix4d *delta) const;

    void _SetChainXformDeltas(const std::map<SdfPath, GfMatrix4d> &deltas);

    std::mutex _mutex;
    std::shared_ptr<RigExecSnapshotStore> _store;
    std::vector<Chain> _chains;
    RigSessions _sessions;
    UsdStageRefPtr _stage;
    /// Rig roots sighted by the rig adapter (see NoteRigRoot).
    ///
    /// Its own mutex, SEPARATE from _mutex on purpose: the adapter notes
    /// roots from inside scene index traversals that run while _mutex is
    /// held, and taking _mutex there is a self-deadlock (MSVC throws
    /// device_or_resource_busy). Lock order is _mutex THEN _notedMutex --
    /// Activate and Deactivate clear the set while holding _mutex -- and
    /// nothing ever takes _mutex while holding _notedMutex.
    std::mutex _notedMutex;
    std::set<SdfPath> _notedRigRoots;
    std::set<SdfPath> _generatedScopes;
    std::set<SdfPath> _assetRoots;
    std::set<SdfPath> _readRoots;
    bool _readRootsDirty = false;
    UsdTimeCode _lastTime = UsdTimeCode::Default();
    /// A manipulation in progress: the declared slots, and the xform-lane
    /// deltas currently standing (kept here as well as on the chains so a
    /// chain built mid-drag starts in step -- see RegisterChain).
    std::vector<PreviewSlot> _previewSlots;
    std::map<SdfPath, GfMatrix4d> _previewXformDeltas;
    bool _previewActive = false;
    /// The influence-overlay selection, held HERE rather than only on the
    /// bridge because it outlives one: a host may select before
    /// activation, and Deactivate/Activate must not silently drop it.
    SdfPath _weightOverlay;
    uint64_t _generation = 0;
    uint64_t _publishedEpochId = 0;
    TfNotice::Key _changeKey;
};

}  // namespace rigExec

// Visibility for the C surface below. Windows needs dllexport while building
// rigExecImaging and dllimport when a consumer includes this header -- the
// build defines RIGEXEC_IMAGING_EXPORTS to tell the two apart. Elsewhere the
// ELF/Mach-O equivalent is default visibility, which also keeps the symbols
// alive if the tree is ever built with -fvisibility=hidden.
#if defined(_WIN32)
#  if defined(RIGEXEC_IMAGING_EXPORTS)
#    define RIGEXEC_IMAGING_C_API __declspec(dllexport)
#  else
#    define RIGEXEC_IMAGING_C_API __declspec(dllimport)
#  endif
#elif defined(__GNUC__) || defined(__clang__)
#  define RIGEXEC_IMAGING_C_API __attribute__((visibility("default")))
#else
#  define RIGEXEC_IMAGING_C_API
#endif

// C surface for language-neutral activation (e.g. the usdview Python
// plugin via ctypes + UsdUtilsStageCache ids). Returns 0 on success.
extern "C" {

RIGEXEC_IMAGING_C_API int RigExecImaging_Activate(
    long long stageCacheId, const char *rigPath, double initialFrame);
RIGEXEC_IMAGING_C_API int RigExecImaging_SetTime(double frame);
/// Per-phase viewport profile totals to a text file, then cleared. Needs
/// RIGEXEC_IMAGING_PROFILE set when the rig was activated.
RIGEXEC_IMAGING_C_API int RigExecImaging_WriteProfileSummary(const char *path);
RIGEXEC_IMAGING_C_API void RigExecImaging_Deactivate();
/// Current published snapshot generation (0 before first publication).
RIGEXEC_IMAGING_C_API long long RigExecImaging_GetGeneration();

/// Reads one complete evaluated control matrix from the exact published
/// stage/time. Returns 1 on success, 0 without changing output otherwise.
/// isDefault selects UsdTimeCode::Default; ordinary frames must be finite.
/// Reads scalar properties this generation published, by property path.
///
/// \p packedPaths is newline-separated absolute property paths, the same
/// convention BeginPreview uses; \p out receives one float each and must
/// hold \p count of them. Returns how many were FOUND; a path the rig did
/// not publish leaves its slot at 0 and is not counted, so a caller can
/// tell "everything is zero" from "nothing was published".
///
/// This exists so a tool does not have to evaluate the rig a second time to
/// see numbers the viewport already computed. The Shape Editor was doing
/// exactly that -- 9-15 ms per refresh to recover pose-interpolator weights
/// sitting in the current snapshot.
RIGEXEC_IMAGING_C_API int RigExecImaging_GetMovedFloats(
    const char *packedPaths, float *out, int count);

RIGEXEC_IMAGING_C_API int RigExecImaging_GetControlFrameAssetSpace(
    long long stageCacheId, const char *primPath, double frame,
    int isDefault, double outMatrix[16]);

/// ASSET-SPACE axis-aligned bounds of everything the current generation
/// draws for \p primPath, as min xyz then max xyz in \p outMinMax.
/// Returns 1 when the prim draws something, 0 otherwise (\p outMinMax is
/// then untouched).
///
/// This exists because a rig draws nothing a bounding box can be computed
/// from the ordinary way. Guides are synthesized inside the imaging chain
/// and never authored, and the RigExec prim types are not UsdGeomImageable,
/// so UsdGeomBBoxCache -- which is what usdview's frame-selection goes
/// through -- correctly reports an empty box for every joint, control, and
/// solver on the stage. Framing a control therefore moved the camera
/// nowhere. The snapshot is the only place the drawn extent exists, so the
/// answer has to come from here.
///
/// Asset space, not world: guide frames carry no stage placement (see
/// RigExecImagingSnapshot::assetRoot), so the caller composes the asset
/// root's own world transform. No Hydra dependency -- snapshot data only.
RIGEXEC_IMAGING_C_API int RigExecImaging_GetGuideBoundsAssetSpace(
    const char *primPath, double outMinMax[6]);

/// The union of RigExecImaging_GetGuideBoundsAssetSpace over every prim in
/// the current generation, so framing one rig frames its whole guide set.
/// Returns 0 when guides belong to multiple asset roots because their
/// asset-space ranges cannot be combined before applying distinct root
/// transforms. Returns 1 when anything at all draws, 0 otherwise.
RIGEXEC_IMAGING_C_API int RigExecImaging_GetAllGuideBoundsAssetSpace(
    double outMinMax[6]);

/// Paints \p weightPrimPath's resolved weight field onto the geometry it
/// weights, as a grey-to-red vertex gradient in the viewport (the R&H
/// "Voodoo" influence display). Null or empty turns the overlay off.
/// Returns 0 on success, non-zero on failure.
///
/// This exists because a weight volume is invisible and its effect is only
/// legible after the fact: a rigger placing one is otherwise reading a
/// deformation and inferring the region that caused it. The overlay shows
/// the region directly, and shows the field a mover ACTUALLY consumed
/// rather than a re-derivation of it.
///
/// Republishes at the current time before returning, so the viewport
/// updates immediately rather than at the next frame change.
RIGEXEC_IMAGING_C_API int RigExecImaging_SetWeightOverlay(
    const char *weightPrimPath);

}

#endif  // RIGEXEC_IMAGING_REGISTRY_H
