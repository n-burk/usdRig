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

#include "pxr/imaging/hd/filteringSceneIndex.h"

#include <memory>
#include <set>

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
        GfMatrix4d *result) const;

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
        const RigExecImagingSnapshot &snapshot) const;

    /// Brings _announcedDrivenXforms for \p path in line with the current
    /// generation. Runs whether or not this index is observed.
    void _RefreshDrivenXform(const SdfPath &path);

    /// Guide element count this prim should have in the current generation.
    size_t _DesiredGuideCount(const SdfPath &path) const;

    /// Records the guide count without emitting. Runs whether or not this
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

    std::shared_ptr<RigExecSnapshotStore> _store;
    /// Guide-element counts already announced per published prim
    /// (mutated only on the serialized publication path).
    std::map<SdfPath, size_t> _announcedGuides;
    /// Control prims whose rigGuideCtrl child has been announced, and the
    /// prim type it was announced with.
    ///
    /// The type is kept, not just the fact of the announcement: an author
    /// switching guide:drawMode from wire to geometry leaves the child
    /// present but turns it from a basisCurves into a mesh, and a consumer
    /// only learns that from a fresh PrimsAdded carrying the new type.
    std::map<SdfPath, TfToken> _announcedControlGuides;

    /// Prims whose transform we published a driven override for, as of the
    /// last announcement. Needed to invalidate a subtree when a driven
    /// transform is REMOVED: by then the snapshot no longer mentions the
    /// prim, so nothing else records that its descendants carry a stale
    /// delta.
    std::set<SdfPath> _announcedDrivenXforms;
};

}  // namespace rigExec

#endif  // RIGEXEC_IMAGING_SCENE_INDICES_H
