//
// RigExec Hydra 2.0 filtering scene indices (spec §10.1):
//
//   upstream (UsdImaging chain)
//     -> RigExecInternalPrimPruningSceneIndex   (owned generated paths only)
//     -> RigExecBindingResolvingSceneIndex      (immutable binding epoch)
//     -> RigExecResultsSceneIndex               (cached snapshot overlays)
//     -> downstream (merging scene index / renderer)
//
// The public boundary contains only standard Hydra data (spec §10.2):
// HdXformSchema matrices, flat HdPrimvarsSchema entries for points and
// normals, HdExtentSchema min/max, and HdBlockDataSource masks for the
// derivative entries of owned points. Renderers never call RigExec.
//
#ifndef RIGEXEC_IMAGING_SCENE_INDICES_H
#define RIGEXEC_IMAGING_SCENE_INDICES_H

#include "snapshotStore.h"

#include "pxr/base/tf/token.h"

#include "pxr/imaging/hd/filteringSceneIndex.h"

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

class RigExecInternalPrimPruningSceneIndex;
using RigExecInternalPrimPruningSceneIndexRefPtr =
    TfRefPtr<RigExecInternalPrimPruningSceneIndex>;
class RigExecBindingResolvingSceneIndex;
using RigExecBindingResolvingSceneIndexRefPtr =
    TfRefPtr<RigExecBindingResolvingSceneIndex>;
class RigExecResultsSceneIndex;
using RigExecResultsSceneIndexRefPtr = TfRefPtr<RigExecResultsSceneIndex>;
class RigExecXformOverrideSceneIndex;
using RigExecXformOverrideSceneIndexRefPtr =
    TfRefPtr<RigExecXformOverrideSceneIndex>;

/// Whether a control guide with this shape/drawMode pair is DRAWN at all.
///
/// The single predicate behind both the synthesized prim and the computed
/// extent. allowedTokens is documentation rather than enforcement, so an
/// unrecognized pair has to mean something definite -- it means nothing is
/// drawn, and the extent has to agree or a host frames empty space around a
/// guide that does not exist.
bool RigExecControlGuideIsDrawn(
    const TfToken &shape, const TfToken &drawMode);

/// Removes only the derived __RigExecGenerated application paths owned by
/// the current system (spec §10.1). The predicate is deliberately narrow:
/// exactly the reserved scopes registered by the compiler; unrelated
/// authored prims, computations, and notices pass through unchanged.
class RigExecInternalPrimPruningSceneIndex final
    : public HdSingleInputFilteringSceneIndexBase {
public:
    static RigExecInternalPrimPruningSceneIndexRefPtr New(
        const HdSceneIndexBaseRefPtr &inputSceneIndex) {
        return TfCreateRefPtr(
            new RigExecInternalPrimPruningSceneIndex(inputSceneIndex));
    }

    /// Registers/replaces the owned generated scopes (one per rig).
    void SetOwnedScopes(const std::set<SdfPath> &scopes);

    HdSceneIndexPrim GetPrim(const SdfPath &primPath) const override;
    SdfPathVector GetChildPrimPaths(const SdfPath &primPath) const override;

protected:
    void _PrimsAdded(
        const HdSceneIndexBase &sender,
        const HdSceneIndexObserver::AddedPrimEntries &entries) override;
    void _PrimsRemoved(
        const HdSceneIndexBase &sender,
        const HdSceneIndexObserver::RemovedPrimEntries &entries) override;
    void _PrimsDirtied(
        const HdSceneIndexBase &sender,
        const HdSceneIndexObserver::DirtiedPrimEntries &entries) override;

private:
    explicit RigExecInternalPrimPruningSceneIndex(
        const HdSceneIndexBaseRefPtr &inputSceneIndex);

    bool _IsOwned(const SdfPath &path) const;

    std::set<SdfPath> _ownedScopes;
};

/// In-memory transform previews for prims the rig does not drive
/// (docs/superpowers/specs/2026-09-10-hydra-preview-manipulation-design.md).
///
/// This is the manipulation preview for a plain Xformable. A rig prim's
/// preview goes through the evaluator, because moving a control has to re-run
/// the rig; a prim with no rig behind it has nothing to re-run, so its preview
/// is its transform, held here and overlaid on the way past.
///
/// WHAT IS STORED IS A WORLD-SPACE DELTA, post-multiplied:
///
///     xform'(X) = xform(X) . delta     for X at or under the previewed prim
///
/// and not the prim's new local matrix, because by the time a prim reaches
/// this filter its transform is already FLATTENED -- the same reason the
/// results index marks its own driven matrices resetXformStack=true
/// (see _ComputeDrivenXform). Overriding a flattened matrix with a local one
/// would drop every ancestor, and overriding only the previewed prim would
/// leave its children behind at the parent's old place.
///
/// A post-multiplied delta answers both: applied to the prim it is the move,
/// and applied to a descendant it is the same move around the same pivot,
/// which is what inheriting a parent's transform means. One multiply per
/// prim, no ancestor lookups, and it is the quantity a manipulator already
/// computes -- a world translation of d is a delta of Translate(d).
///
/// Deltas are replaced as a SET rather than one at a time. A manipulation
/// changes the same handful of prims on every mouse sample, and computing the
/// dirty set as the union of what was previewed and what now is makes one
/// call enough to add, change, and drop in any combination.
///
/// Nothing here is ever authored. On release the application authors the
/// committed values to the stage and drops the deltas in one step, and the
/// prim's own authored transform is what is drawn from then on.
class RigExecXformOverrideSceneIndex final
    : public HdSingleInputFilteringSceneIndexBase {
public:
    static RigExecXformOverrideSceneIndexRefPtr New(
        const HdSceneIndexBaseRefPtr &inputSceneIndex) {
        return TfCreateRefPtr(
            new RigExecXformOverrideSceneIndex(inputSceneIndex));
    }

    /// Replaces every preview delta and dirties the xform locator across the
    /// subtree of each path that entered, left, or changed.
    void SetWorldDeltas(const std::map<SdfPath, GfMatrix4d> &deltas);

    /// Equivalent to SetWorldDeltas({}), named for the call site that means it.
    void ClearWorldDeltas() { SetWorldDeltas({}); }

    bool HasWorldDeltas() const;

    HdSceneIndexPrim GetPrim(const SdfPath &primPath) const override;

    SdfPathVector GetChildPrimPaths(const SdfPath &primPath) const override {
        return _GetInputSceneIndex()->GetChildPrimPaths(primPath);
    }

protected:
    void _PrimsAdded(
        const HdSceneIndexBase &sender,
        const HdSceneIndexObserver::AddedPrimEntries &entries) override;
    void _PrimsRemoved(
        const HdSceneIndexBase &sender,
        const HdSceneIndexObserver::RemovedPrimEntries &entries) override;
    void _PrimsDirtied(
        const HdSceneIndexBase &sender,
        const HdSceneIndexObserver::DirtiedPrimEntries &entries) override;

private:
    explicit RigExecXformOverrideSceneIndex(
        const HdSceneIndexBaseRefPtr &inputSceneIndex);

    /// The delta governing \p path: the nearest previewed ancestor's, or
    /// its own. Null when no preview applies.
    const GfMatrix4d *_FindDelta(const SdfPath &path) const;

    /// Dirties the xform locator on \p path and every descendant upstream.
    void _DirtyXformSubtree(
        const SdfPath &path,
        HdSceneIndexObserver::DirtiedPrimEntries *entries) const;

    /// Hydra pulls on its own thread while the application sets deltas on its
    /// own; the map is small and the critical sections are tiny.
    mutable std::mutex _mutex;
    std::map<SdfPath, GfMatrix4d> _deltas;
};

/// Atomically receives the compiler-produced immutable binding epoch and
/// carries the reverse output map (spec §10.1). With the USD backend the
/// compiled output addresses are stage paths, so prim shells pass through
/// unchanged; an epoch replacement resyncs the previously and newly
/// published outputs. The scene index never inspects Hydra to reconstruct
/// rig semantics.
class RigExecBindingResolvingSceneIndex final
    : public HdSingleInputFilteringSceneIndexBase {
public:
    /// The immutable epoch payload: which Hydra prims receive published
    /// results in this generation.
    struct BindingEpoch {
        uint64_t id = 0;
        std::set<SdfPath> publishedPrims;
    };
    using BindingEpochConstPtr = std::shared_ptr<const BindingEpoch>;

    static RigExecBindingResolvingSceneIndexRefPtr New(
        const HdSceneIndexBaseRefPtr &inputSceneIndex) {
        return TfCreateRefPtr(
            new RigExecBindingResolvingSceneIndex(inputSceneIndex));
    }

    /// Atomically swaps the epoch and dirties the affected outputs
    /// (universal locator for output-set changes, spec §10.4).
    void SetBindingEpoch(BindingEpochConstPtr epoch);

    BindingEpochConstPtr GetBindingEpoch() const {
        return std::atomic_load(&_epoch);
    }

    HdSceneIndexPrim GetPrim(const SdfPath &primPath) const override;
    SdfPathVector GetChildPrimPaths(const SdfPath &primPath) const override;

protected:
    void _PrimsAdded(
        const HdSceneIndexBase &sender,
        const HdSceneIndexObserver::AddedPrimEntries &entries) override;
    void _PrimsRemoved(
        const HdSceneIndexBase &sender,
        const HdSceneIndexObserver::RemovedPrimEntries &entries) override;
    void _PrimsDirtied(
        const HdSceneIndexBase &sender,
        const HdSceneIndexObserver::DirtiedPrimEntries &entries) override;

private:
    explicit RigExecBindingResolvingSceneIndex(
        const HdSceneIndexBaseRefPtr &inputSceneIndex);

    BindingEpochConstPtr _epoch;
};

/// Overlays cached xform, primvar, and extent leaves from the snapshot
/// store (spec §10.3). GetPrim() only reads the atomic immutable snapshot
/// and constructs/returns cached data-source overlays; it never computes,
/// waits, changes time, or locks the authoring stage.
/// Snapshot-only publishers must populate xformResetPaths for authored
/// reset boundaries; the USD bridge captures these before Hydra flattening.
class RigExecResultsSceneIndex final
    : public HdSingleInputFilteringSceneIndexBase {
public:
    static RigExecResultsSceneIndexRefPtr New(
        const HdSceneIndexBaseRefPtr &inputSceneIndex,
        std::shared_ptr<RigExecSnapshotStore> store) {
        return TfCreateRefPtr(
            new RigExecResultsSceneIndex(inputSceneIndex, std::move(store)));
    }

    /// Called by the publisher after a complete generation swap: sends
    /// coalesced dirtied notices from the per-prim changed-leaf sets
    /// (spec §10.4). Value changes start from the narrowest logical
    /// leaves and are expanded with
    /// HdContainerDataSourceEditor::ComputeDirtyLocators() because v0.1
    /// rebuilds the retained container handles per generation
    /// (spec §10.3); structural entries use universal dirtiness.
    /// Resolves \p primPath's world transform against every constraint-driven
    /// ancestor (or itself). False when none applies.
    bool _ComputeDrivenXform(
        const SdfPath &primPath,
        const HdContainerDataSourceHandle &inputDataSource,
        const RigExecImagingSnapshot &snapshot,
        GfMatrix4d *result, float shutterOffset = 0.0f) const;

    /// Dirties \p locators on every descendant present upstream, plus the
    /// synthesized guide children present only here.
    void _DirtySubtree(
        const SdfPath &path,
        const HdDataSourceLocatorSet &locators,
        HdSceneIndexObserver::DirtiedPrimEntries *entries) const;

    /// Dirties \p locators on the synthesized guide children announced
    /// under \p path. Their matrices bake the asset root's world transform,
    /// so no ancestor's dirtiness reaches them.
    void _DirtyAnnouncedGuideChildren(
        const SdfPath &path,
        const HdDataSourceLocatorSet &locators,
        HdSceneIndexObserver::DirtiedPrimEntries *entries) const;

    /// Does the current generation drive this prim's transform?
    bool _IsDrivenXform(const SdfPath &path) const;

    /// The asset root's world transform as THIS index reports it, so a
    /// driven asset root carries its guides with it.
    GfMatrix4d _ResolveAssetRootWorld(
        const SdfPath &assetRoot,
        const RigExecImagingSnapshot &snapshot) const;

    /// Brings _announcedDrivenXforms for \p path in line with the current
    /// generation. Runs whether or not this index is observed.
    void _RefreshDrivenXform(const SdfPath &path);

    /// Guide elements this prim should have in the current generation.
    /// One byte per payload element: bit 0 is its sphere and bit 1 its cone.
    std::vector<uint8_t> _DesiredGuideTopology(const SdfPath &path) const;

    /// Records the guide topology without emitting. Runs whether or not this
    /// index is observed.
    void _RefreshAnnouncedGuides(const SdfPath &path);

    /// The Hydra prim type this prim's synthesized control guide should
    /// have in the current generation, or an empty token when it should
    /// have none (spec §10.3 extension). The type IS the desired-state
    /// answer here: a control draws exactly one guide child, but changing
    /// guide:shape or guide:drawMode changes what kind of prim it is.
    TfToken _DesiredControlGuideType(const SdfPath &path) const;

    /// Records the control-guide announcement without emitting. Runs
    /// whether or not this index is observed, for the same reason
    /// _RefreshAnnouncedGuides does.
    void _RefreshAnnouncedControlGuide(const SdfPath &path);

    /// The Hydra prim types this prim's synthesized volume weight guide
    /// children should have in the current generation, in index order;
    /// empty for none (spec §4.1 volumetric extension, drawn side).
    ///
    /// Both a count and a set of types, unlike its two siblings, because
    /// a volume guide varies in both: the number of drawn iso-surfaces
    /// depends on the falloff band, and guide:drawMode decides what kind
    /// of prim each surface is.
    std::vector<TfToken> _DesiredVolumeGuideTypes(const SdfPath &path) const;

    /// Records the volume-guide announcement without emitting. Runs
    /// whether or not this index is observed, for the same reason
    /// _RefreshAnnouncedGuides does.
    void _RefreshAnnouncedVolumeGuides(const SdfPath &path);

    void NotifyGenerationPublished(
        const RigExecPublishedDirtyVector &dirtied);

    HdSceneIndexPrim GetPrim(const SdfPath &primPath) const override;
    SdfPathVector GetChildPrimPaths(const SdfPath &primPath) const override;

protected:
    void _PrimsAdded(
        const HdSceneIndexBase &sender,
        const HdSceneIndexObserver::AddedPrimEntries &entries) override;
    void _PrimsRemoved(
        const HdSceneIndexBase &sender,
        const HdSceneIndexObserver::RemovedPrimEntries &entries) override;
    void _PrimsDirtied(
        const HdSceneIndexBase &sender,
        const HdSceneIndexObserver::DirtiedPrimEntries &entries) override;

private:
    RigExecResultsSceneIndex(
        const HdSceneIndexBaseRefPtr &inputSceneIndex,
        std::shared_ptr<RigExecSnapshotStore> store);

    /// Reconciles the announced synthesized guide children of one
    /// published prim against the current snapshot, emitting exact
    /// PrimsAdded/PrimsRemoved entries (joints and aggregate solvers
    /// draw as guide geometry like OpenExec's IrJointScope).
    void _SyncGuideChildren(
        const SdfPath &path,
        HdSceneIndexObserver::AddedPrimEntries *added,
        HdSceneIndexObserver::RemovedPrimEntries *removed);

    /// The same reconciliation for the single synthesized control-guide
    /// child, whose desired state is a prim type rather than a count.
    void _SyncControlGuideChild(
        const SdfPath &path,
        HdSceneIndexObserver::AddedPrimEntries *added,
        HdSceneIndexObserver::RemovedPrimEntries *removed);

    /// The same reconciliation for the volume weight iso-surface
    /// children, whose desired state is a per-index list of prim types.
    void _SyncVolumeGuideChildren(
        const SdfPath &path,
        HdSceneIndexObserver::AddedPrimEntries *added,
        HdSceneIndexObserver::RemovedPrimEntries *removed);

    std::shared_ptr<RigExecSnapshotStore> _store;
    /// Guide topology already announced per published prim: one sphere/cone
    /// bit mask per payload element (mutated only on the serialized
    /// publication path).
    std::map<SdfPath, std::vector<uint8_t>> _announcedGuides;
    /// Control prims whose rigGuideCtrl child has been announced, and the
    /// prim type it was announced with.
    ///
    /// The type is kept, not just the fact of the announcement: an author
    /// switching guide:drawMode from wire to geometry leaves the child
    /// present but turns it from a basisCurves into a mesh, and a consumer
    /// only learns that from a fresh PrimsAdded carrying the new type.
    std::map<SdfPath, TfToken> _announcedControlGuides;

    /// Volume weight prims whose rigGuideVol_N children have been
    /// announced, and the prim type each was announced with.
    ///
    /// Types and not just a count, for both reasons at once: a
    /// guide:drawMode edit changes what kind of prim an existing child is,
    /// and the count itself moves when the falloff band changes. One
    /// vector answers both, so the announcement can never be half right.
    std::map<SdfPath, std::vector<TfToken>> _announcedVolumeGuides;

    /// Prims whose transform we published a driven override for, as of the
    /// last announcement. Needed to invalidate a subtree when a driven
    /// transform is REMOVED: by then the snapshot no longer mentions the
    /// prim, so nothing else records that its descendants carry a stale
    /// delta.
    std::set<SdfPath> _announcedDrivenXforms;

    /// Prims we have announced an influence-overlay displayColor on.
    ///
    /// Needed because a primvar APPEARING is not a dirty, it is a resync.
    /// HdSceneIndexAdapterSceneDelegate caches each rprim's primvar
    /// DESCRIPTORS and rebuilds them on PrimsAdded, not on a dirty --
    /// even a universal one. So turning the overlay on while the mesh is
    /// already synced leaves Storm drawing the mesh grey forever: the
    /// value is right there in the scene index, and nothing ever asks for
    /// it. Verified against a real usdview: the dirty-notice path leaves
    /// the mesh grey, a cold renderer rebuild draws it red.
    ///
    /// Kept as our own record for the same reason _announcedDrivenXforms
    /// is: by the time the overlay is turned OFF the snapshot no longer
    /// says the prim ever had one.
    std::set<SdfPath> _announcedWeightOverlays;
};

}  // namespace rigExec

#endif  // RIGEXEC_IMAGING_SCENE_INDICES_H
