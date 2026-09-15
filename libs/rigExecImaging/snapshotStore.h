//
// RigExec Hydra snapshot store (spec §8.2, §10.3).
//
// Evaluation always completes before Hydra pulls: the evaluator publishes
// complete immutable generations here, and the results scene index's
// GetPrim() only ever reads the current atomic snapshot. It never
// computes, waits, changes time, or locks the authoring stage.
//
#ifndef RIGEXEC_IMAGING_SNAPSHOT_STORE_H
#define RIGEXEC_IMAGING_SNAPSHOT_STORE_H

#include "pxr/pxr.h"
#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec3d.h"
#include "pxr/base/tf/token.h"
#include "pxr/base/vt/array.h"
#include "pxr/base/vt/types.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"

#include <atomic>
#include <algorithm>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

/// One drawn iso-surface of one placed influence volume.
///
/// Self-describing geometry rather than an index into a shared unit-shape
/// table (the way the control guides reference _ControlGuideShape): a
/// curve weight's guide is built from the AUTHORED curve, so its points
/// differ per prim and per generation and there is no fixed table entry to
/// point at. Carrying the shape here keeps sphere, plane, and curve on one
/// code path in the scene index, which is the only way the three cannot
/// drift apart in how they are announced and dirtied.
struct RigExecVolumeGuideElement {
    /// ASSET-space placement, including whatever dimensional scale the
    /// shape carries in its transform rather than in its points -- a
    /// sphere's iso-surface is the UNIT wire sphere scaled by the radius,
    /// so the radius appears exactly once and the implicit and wire draw
    /// modes are sized by the same factor. Shapes with no implicit form
    /// (the plane square, the curve tube) instead bake their dimensions
    /// into `points`, in the volume's own local space.
    GfMatrix4d xform{1.0};
    /// basisCurves | mesh | sphere. Per ELEMENT rather than per prim: a
    /// volume publishes its falloffMin and falloffMax surfaces as two
    /// elements, and the drawMode that picks the type is shared, but
    /// keeping it per element is what lets the announcement machinery
    /// treat a type change as the structural event it is.
    TfToken primType;
    VtVec3fArray points;   ///< empty for the implicits
    VtIntArray counts;     ///< curveVertexCounts, or faceVertexCounts
    VtIntArray indices;    ///< faceVertexIndices; empty for curves
    /// Optional mesh normals. `normalsInterpolation` is empty when normals
    /// are absent; curve tubes use faceVarying normals so their closed end
    /// caps can stay flat without splitting the manifold's ring vertices.
    VtVec3fArray normals;
    TfToken normalsInterpolation;
    /// Open plane surfaces need both faces; closed curve tubes do not.
    bool doubleSided = false;
    /// Wire width in the element's own LOCAL (pre-xform) units. Zero or
    /// negative publishes no widths at all -- the hairline fallback, and
    /// what the non-wire draw modes always want.
    double wireWidth = 0.0;

    bool operator==(const RigExecVolumeGuideElement &other) const {
        return xform == other.xform && primType == other.primType &&
               points == other.points && counts == other.counts &&
               indices == other.indices && normals == other.normals &&
               normalsInterpolation == other.normalsInterpolation &&
               doubleSided == other.doubleSided &&
               wireWidth == other.wireWidth;
    }
    bool operator!=(const RigExecVolumeGuideElement &other) const {
        return !(*this == other);
    }
};

/// The published results for one Hydra prim in one generation. Only
/// standard data crosses this boundary (spec §10.2): local xforms,
/// points/normals as flat primvars, and the two-element extent.
struct RigExecPublishedPrim {
    /// Asset-space anchor for guide frames on this prim.  Stored per prim so
    /// one atomic stage generation can contain multiple character rigs with
    /// different asset roots.
    SdfPath assetRoot;

    /// The revised LOCAL transform, and the local transform it revised.
    ///
    /// Both are needed because the RigExec scene index is installed
    /// downstream of Hydra's flattening (the only HdFlatteningSceneIndex in
    /// the chain lives inside UsdImagingNiPrototypePropagatingSceneIndex,
    /// which UsdImagingCreateSceneIndices builds before it appends plugin
    /// scene indices). Transforms arriving there are already world-space, so
    /// the revised local matrix cannot be published as-is; the consumer
    /// rebuilds it as a world-space delta from the pair.
    bool hasXform = false;
    GfMatrix4d xform{1.0};
    GfMatrix4d xformBase{1.0};

    bool hasPoints = false;
    VtVec3fArray points;

    bool hasNormals = false;
    VtVec3fArray normals;

    bool hasExtent = false;
    GfVec3d extentMin{0};
    GfVec3d extentMax{0};

    /// Explicit frame-relative shutter samples under this generation fence.
    /// Empty arrays mean single-sample publication; scalar values retain the
    /// offset-nearest-zero sample. Every populated array matches sampleOffsets.
    std::vector<float> sampleOffsets;
    std::vector<VtVec3fArray> pointsSamples;
    std::vector<VtVec3fArray> normalsSamples;
    std::vector<GfMatrix4d> xformSamples;
    std::vector<GfMatrix4d> xformBaseSamples;
    std::vector<GfVec3d> extentMinSamples;
    std::vector<GfVec3d> extentMaxSamples;

    /// Guide drawing payload (joints and aggregate solvers draw as guide
    /// geometry like OpenExec's IrJointScope): one rig-space frame matrix,
    /// derived cone length, primitive radius, and topology mask per guide
    /// element, plus constant styling from the authored guide attributes.
    bool hasGuides = false;
    std::vector<GfMatrix4d> guideFrames;
    std::vector<double> guideLengths;
    /// Radius of both the sphere and the cone, per element. Parallel to
    /// guideFrames, so a lookup valid for one is valid for all three.
    std::vector<double> guideRadii;
    /// Whether each element owns an origin sphere. Solvers set this for
    /// every frame. A joint sets it only on its first outgoing child link,
    /// so a branching joint still draws exactly one sphere; a leaf publishes
    /// one sphere-only element. Missing entries retain the historical
    /// sphere-per-frame behavior for manually constructed snapshots.
    std::vector<bool> guideDrawSpheres;
    GfVec3f guideColor{1.0f, 0.3f, 0.3f};
    float guideOpacity = 0.5f;
    /// The prim's RESOLVED UsdGeomImageable purpose, stamped onto whatever
    /// guide it draws. Shared by both payloads because it comes from the
    /// prim, not from the drawing: UsdGeomBBoxCache classifies the prim's
    /// extent by this exact attribute, so publishing anything else here
    /// would let a guide draw in one bucket and bound in another.
    TfToken guidePurpose;

    /// Control guide payload (spec §10.3 extension): a control draws ONE
    /// synthesized shape at its posed frame, chosen and sized by the
    /// authored guide attributes.
    ///
    /// Separate from the guideFrames vector above rather than folded into
    /// it, because the two are different drawings: the joint payload is one
    /// sphere plus zero-or-more child-link cones whose only authored
    /// dimension is a radius, while this is a single shape whose prim type
    /// depends on the authored shape/drawMode pair. Sharing the array would
    /// make "which element is which kind" a thing every consumer had to
    /// decide.
    ///
    /// guideColor/guideOpacity above are shared: both payloads read the
    /// same guide:displayColor / guide:displayOpacity attributes, and a
    /// prim never carries both payloads (a control is not a joint).
    bool hasControlGuide = false;
    /// The original evaluated control matrix for native manipulators. Kept
    /// before guide rigidization/scaling, including when guide drawing is off.
    bool hasControlFrame = false;
    GfMatrix4d controlFrame{1.0};
    /// Rigidized, ASSET-space, like guideFrames.
    GfMatrix4d controlGuideFrame{1.0};
    /// sphere|circle|box|cube|diamond|pyramid.
    TfToken controlGuideShape;
    /// wire|geometry.
    TfToken controlGuideDrawMode;
    /// Effective per-axis draw scale: positive evaluated frame-axis
    /// magnitudes times guide:scaleX/Y/Z. The guide multipliers are authored
    /// as three separate doubles (deliberately not a vec3, per direction);
    /// the product is stored as one vector because nothing downstream has a
    /// reason to take the axes apart again.
    GfVec3d controlGuideScale{1.0, 1.0, 1.0};
    /// Width for the wire draw mode, in the guide's LOCAL pre-scale units.
    /// Zero or negative publishes no widths at all (hairline fallback);
    /// the geometry draw mode ignores it.
    double controlGuideWireWidth = 0.05;

    /// Influence overlay (the "Voodoo" visualisation): the resolved
    /// weight field of ONE selected weight object, one value per logical
    /// element of THIS prim's points, painted onto the geometry as a
    /// grey-to-red vertex gradient.
    ///
    /// Keyed onto the WEIGHTED GEOMETRY rather than onto the weight
    /// object, because that is the prim Hydra draws. The selection lives
    /// in the registry (one overlay at a time, by design: two overlapping
    /// gradients on one mesh are unreadable), so this is already the
    /// resolved answer -- a consumer never has to know which volume it
    /// came from.
    ///
    /// Off by default and absent from the published container when off,
    /// so an ordinary render is byte-for-byte what it was.
    bool hasWeightOverlay = false;
    VtFloatArray weightOverlay;

    /// Volume weight guide payload: the falloffMin and falloffMax
    /// iso-surfaces one placed influence volume draws.
    ///
    /// A third payload rather than a reuse of either existing one, for
    /// the reason the control payload is separate from the joint payload:
    /// these are N shapes whose PRIM TYPE varies per element, while the
    /// joint payload is a sphere plus zero-or-more uniform cone links and
    /// the control payload is exactly one shape. Folding them together would
    /// make "which element is which kind" a question every consumer had to
    /// answer.
    ///
    /// guideColor/guideOpacity/guidePurpose above are shared, as they are
    /// between the other two payloads: a prim is a volume weight or a
    /// joint or a control, never two of them.
    bool hasVolumeGuides = false;
    std::vector<RigExecVolumeGuideElement> volumeGuides;
    /// Curvenet guides reuse the self-describing geometry payload, anchored
    /// to their native Points prim rather than the character asset. Empty
    /// retains the asset-space convention used by influence volumes.
    SdfPath volumeGuideAnchor;
    GfMatrix4d volumeGuideAnchorToAsset{1.0};
};

/// One complete immutable generation (spec §8.2: consumers see complete
/// RigExecSnapshot generations only).
struct RigExecImagingSnapshot {
    uint64_t generation = 0;
    std::map<SdfPath, RigExecPublishedPrim> prims;

    /// Scalar properties this generation published, keyed by PROPERTY path.
    ///
    /// `movedProperties` carries two unrelated kinds of result: geometry,
    /// which becomes Hydra data below, and the property-domain values a
    /// rig computes for its own consumption -- a float dial, a blend
    /// weight, a pose-interpolator output. The second kind has no Hydra
    /// representation and is deliberately dropped on the way into
    /// `prims`; see the comment at the filter in bridge.cpp.
    ///
    /// But a TOOL wants them. The Shape Editor exists to show which
    /// pose-space correctives are firing, and without this it has to run
    /// a SECOND evaluation of the whole rig to recover numbers this
    /// generation already computed -- measured at 9-15 ms per refresh
    /// against ~0 for a map lookup. So they are carried here: read-only,
    /// alongside the generation that produced them, and never touched by
    /// the scene index.
    std::map<SdfPath, float> movedFloats;

    /// True when any prim in this generation carries a driven transform.
    ///
    /// Resolving a driven transform means walking a prim's ancestors, and
    /// that has to happen for every prim on every pull -- including scenes
    /// where the rig deforms points and drives no transform at all. This
    /// lets that overwhelmingly common case cost one bool test.
    ///
    /// Do not set this by hand: RigExecSnapshotStore::Publish derives it from
    /// the prims. Setting it wrong loses transforms silently.
    bool hasDrivenXforms = false;
    /// Authored reset boundaries beneath driven transforms, captured before
    /// Hydra flattening erases that information. Stage-less publishers must
    /// supply their own boundaries; an empty set means ordinary inheritance.
    std::set<SdfPath> xformResetPaths;

    /// The rig's asset root (the rig prim's parent) for a single-rig
    /// generation.  Empty for a merged multi-root generation; guide consumers
    /// use RigExecPublishedPrim::assetRoot.
    ///
    /// Guide frames are ASSET-space, so placing them needs the asset root's
    /// transform -- NOT the guide parent's. A joint may sit under an
    /// intervening UsdGeomXform inside the asset, whose transform is already
    /// baked into the rig's own frames; composing the parent's flattened
    /// matrix would apply it twice.
    SdfPath assetRoot;

    /// WHICH STAGE, and WHICH TIME, this generation describes.
    ///
    /// The snapshot store is a process-global singleton, so without these a
    /// consumer that looks a prim up by path alone gets an answer from
    /// whatever rig happened to publish last: open a second stage whose rig
    /// uses the same paths and the first stage's poses are served for it,
    /// silently and plausibly. Time has the same shape of problem in one
    /// stage -- a query at frame 12 answered from the generation published
    /// for frame 30 is wrong in a way nothing downstream can detect.
    ///
    /// This matters for the compute-extent callback specifically, because
    /// that is the one consumer USD may call with a stage and a time of its
    /// own choosing, rather than being handed values by the bridge. It
    /// compares both and falls back to the rest pose on any mismatch, which
    /// is what makes the callback a pure function of (stage, time).
    ///
    /// Identity is the STAGE OBJECT, not its root layer's identifier: two
    /// stages routinely share a root layer and differ only in their session
    /// layer, and a string comparison hands one of them the other's poses.
    /// Weak on purpose -- a published generation must not keep a stage
    /// alive, and a stage that has gone away can no longer be queried.
    UsdStageWeakPtr stage;
    /// The evaluated sample time. Default() is its own value, not 0.
    bool sampleTimeIsDefault = true;
    double sampleTime = 0.0;

    /// True when this generation describes \p queryStage at \p time.
    bool Describes(const UsdStageWeakPtr &queryStage,
                   const UsdTimeCode &time) const {
        if (!stage || stage != queryStage) {
            return false;
        }
        if (time.IsDefault() || sampleTimeIsDefault) {
            return time.IsDefault() && sampleTimeIsDefault;
        }
        return time.GetValue() == sampleTime;
    }
};

using RigExecImagingSnapshotConstPtr =
    std::shared_ptr<const RigExecImagingSnapshot>;

/// Which published leaves changed for one prim between the previous and
/// the newly published generation (spec §10.4: value changes start from
/// the narrowest logical leaves; structural/output-set changes use
/// universal dirtiness).
enum RigExecPublishedChange : uint8_t {
    RigExecChangeNone = 0,
    RigExecChangeXform = 1 << 0,
    RigExecChangePoints = 1 << 1,
    RigExecChangeNormals = 1 << 2,
    RigExecChangeExtent = 1 << 3,
    /// The prim entered or left the published set, or the owned leaf set
    /// (representation) changed: universal/resync dirtiness.
    RigExecChangeStructural = 1 << 4,
    /// The prim's guide payload changed (frames/lengths/styling with an
    /// unchanged element count; count changes are structural).
    RigExecChangeGuides = 1 << 5,
    /// The influence overlay's VALUES changed with the overlay staying on
    /// for this prim. Turning it on or off changes the owned leaf set --
    /// the displayColor primvar appears or disappears -- and reports
    /// structural instead, exactly as points ownership does.
    RigExecChangeWeightOverlay = 1 << 6,
};

struct RigExecPublishedDirty {
    SdfPath path;
    uint8_t changes = RigExecChangeNone;
};

using RigExecPublishedDirtyVector = std::vector<RigExecPublishedDirty>;

/// Atomic published-generation store: the publisher swaps complete
/// generations; readers (Hydra GetPrim) take a shared reference without
/// blocking the publisher.
class RigExecSnapshotStore {
public:
    /// Atomically publishes a complete generation, returning per-prim
    /// changed-leaf sets diffed against the previous generation (the
    /// authoritative input for precise dirtying, spec §10.4). Prims whose
    /// published values are identical produce no entry.
    RigExecPublishedDirtyVector Publish(
        std::shared_ptr<RigExecImagingSnapshot> snapshot) {
        std::lock_guard<std::mutex> lock(_writeMutex);
        RigExecImagingSnapshotConstPtr previous =
            std::atomic_load(&_current);
        RigExecPublishedDirtyVector dirtied;
        if (snapshot) {
            // DERIVE the fast-path flag here rather than trusting whoever
            // built the snapshot. Consumers skip driven-transform resolution
            // entirely when it is false, so a builder that forgets to set it
            // does not get a slow path -- it gets silently missing
            // transforms. Publication is the one place every generation
            // passes through, so it is the only place the invariant holds.
            snapshot->hasDrivenXforms = false;
            for (const auto &[path, prim] : snapshot->prims) {
                if (prim.hasXform) {
                    snapshot->hasDrivenXforms = true;
                    break;
                }
            }
        }
        if (snapshot) {
            for (const auto &[path, prim] : snapshot->prims) {
                const RigExecPublishedPrim *before = nullptr;
                if (previous) {
                    const auto it = previous->prims.find(path);
                    if (it != previous->prims.end()) {
                        before = &it->second;
                    }
                }
                const uint8_t changes = _Diff(before, prim);
                if (changes != RigExecChangeNone) {
                    dirtied.push_back({path, changes});
                }
            }
        }
        if (previous) {
            for (const auto &[path, prim] : previous->prims) {
                if (!snapshot || !snapshot->prims.count(path)) {
                    dirtied.push_back({path, RigExecChangeStructural});
                }
            }
        }
        std::set<SdfPath> resetChanges;
        if (previous) {
            for (const auto &path : previous->xformResetPaths)
                if (!snapshot || !snapshot->xformResetPaths.count(path)) resetChanges.insert(path);
        }
        if (snapshot) {
            for (const auto &path : snapshot->xformResetPaths)
                if (!previous || !previous->xformResetPaths.count(path)) resetChanges.insert(path);
        }
        for (const auto &path : resetChanges) {
            auto entry = std::find_if(dirtied.begin(), dirtied.end(),
                [&path](const auto &dirty) { return dirty.path == path; });
            if (entry == dirtied.end()) dirtied.push_back({path, RigExecChangeXform});
            else entry->changes |= RigExecChangeXform;
        }
        std::atomic_store(
            &_current, RigExecImagingSnapshotConstPtr(std::move(snapshot)));
        return dirtied;
    }

    /// Lock-free read of the current complete generation (may be null
    /// before the first publication).
    RigExecImagingSnapshotConstPtr Get() const {
        return std::atomic_load(&_current);
    }

private:
    /// Leaf-exact diff of one prim's published values. A prim absent from
    /// the previous generation, or whose owned leaf set changed
    /// (ownership begins/ends: the derivative blocks appear/disappear
    /// with points ownership, spec §10.3.1), is structural.
    static uint8_t _Diff(
        const RigExecPublishedPrim *before,
        const RigExecPublishedPrim &after) {
        if (!before) {
            return RigExecChangeStructural;
        }
        if (before->hasXform != after.hasXform ||
            before->hasPoints != after.hasPoints ||
            before->hasNormals != after.hasNormals ||
            before->hasExtent != after.hasExtent ||
            before->hasGuides != after.hasGuides ||
            before->guideFrames.size() != after.guideFrames.size() ||
            before->hasControlGuide != after.hasControlGuide ||
            before->hasControlFrame != after.hasControlFrame ||
            // The overlay begins or ends owning a displayColor primvar,
            // which is a change to the owned leaf set and not to a value
            // in it -- the same rule points ownership follows above. It
            // is also what makes RigExecImaging_SetWeightOverlay redraw
            // immediately: universal dirtiness re-pulls the container and
            // the new primvar is simply there.
            before->hasWeightOverlay != after.hasWeightOverlay ||
            before->hasVolumeGuides != after.hasVolumeGuides ||
            before->volumeGuides.size() != after.volumeGuides.size()) {
            return RigExecChangeStructural;
        }
        // A volume guide element's prim type decides what KIND of prim the
        // synthesized child is (a wire iso-surface is a basisCurves, a
        // solid one a mesh or an implicit sphere), so editing
        // guide:drawMode is structural for the same reason editing a
        // control's guide:shape is.
        if (after.hasVolumeGuides) {
            for (size_t i = 0; i < after.volumeGuides.size(); ++i) {
                if (before->volumeGuides[i].primType !=
                    after.volumeGuides[i].primType) {
                    return RigExecChangeStructural;
                }
            }
        }
        // Every guide payload is placed in ASSET space, so re-anchoring the
        // prim to a different asset root moves the synthesized children even
        // though no guide value changed.
        if ((after.hasGuides || after.hasControlGuide ||
             after.hasVolumeGuides) &&
            before->assetRoot != after.assetRoot) {
            return RigExecChangeStructural;
        }
        // Shape and draw mode decide the synthesized child's PRIM TYPE, so
        // editing either is structural for exactly the reason a guide count
        // change is: the consumer has to be told the prim it cached is a
        // different prim now, not that one of its values moved.
        if (after.hasControlGuide &&
            (before->controlGuideShape != after.controlGuideShape ||
             before->controlGuideDrawMode != after.controlGuideDrawMode)) {
            return RigExecChangeStructural;
        }
        uint8_t changes = RigExecChangeNone;
        if (after.hasControlFrame && before->controlFrame != after.controlFrame) {
            changes |= RigExecChangeGuides;
        }
        // Styling is shared by both guide payloads (one prim never carries
        // both), so it is compared once here and folded into whichever one
        // this prim publishes.
        const bool styleChanged = before->guideColor != after.guideColor ||
                                  before->guideOpacity != after.guideOpacity;
        // Both halves matter: the consumer publishes a delta built from the
        // pair, so a moving base with a static revision still moves the prim.
        if (after.hasXform && (before->xform != after.xform ||
                               before->xformBase != after.xformBase ||
                               before->sampleOffsets != after.sampleOffsets ||
                               before->xformSamples != after.xformSamples ||
                               before->xformBaseSamples != after.xformBaseSamples)) {
            changes |= RigExecChangeXform;
        }
        if (after.hasPoints &&
            (before->points != after.points ||
             before->sampleOffsets != after.sampleOffsets ||
             before->pointsSamples != after.pointsSamples)) {
            changes |= RigExecChangePoints;
        }
        if (after.hasNormals && (before->normals != after.normals ||
            before->sampleOffsets != after.sampleOffsets ||
            before->normalsSamples != after.normalsSamples)) {
            changes |= RigExecChangeNormals;
        }
        if (after.hasExtent && (before->extentMin != after.extentMin ||
                                before->extentMax != after.extentMax ||
                                before->sampleOffsets != after.sampleOffsets ||
                                before->extentMinSamples != after.extentMinSamples ||
                                before->extentMaxSamples != after.extentMaxSamples)) {
            changes |= RigExecChangeExtent;
        }
        if (after.hasGuides &&
            (before->guideFrames != after.guideFrames ||
             before->guideLengths != after.guideLengths ||
             before->guideRadii != after.guideRadii ||
             before->guideDrawSpheres != after.guideDrawSpheres ||
             before->guidePurpose != after.guidePurpose || styleChanged)) {
            changes |= RigExecChangeGuides;
        }
        // Width rides the value arm rather than the structural one even
        // though crossing zero adds or drops the widths primvar outright:
        // RigExecChangeGuides dirties the synthesized child with the
        // universal locator set, so the consumer re-pulls the whole
        // container and sees the primvar appear or disappear either way.
        if (after.hasControlGuide &&
            (before->controlGuideFrame != after.controlGuideFrame ||
             before->controlGuideScale != after.controlGuideScale ||
             before->controlGuideWireWidth != after.controlGuideWireWidth ||
             before->guidePurpose != after.guidePurpose ||
             styleChanged)) {
            changes |= RigExecChangeGuides;
        }
        // Volume guides ride the SAME change bit as the other two guide
        // payloads. They are announced and dirtied by the same machinery
        // in the results scene index, which re-pulls the whole synthesized
        // child either way, so a separate bit would only be a second name
        // for the identical consequence.
        if (after.hasVolumeGuides &&
            (before->volumeGuides != after.volumeGuides ||
             before->volumeGuideAnchor != after.volumeGuideAnchor ||
             before->volumeGuideAnchorToAsset != after.volumeGuideAnchorToAsset ||
             before->guidePurpose != after.guidePurpose || styleChanged)) {
            changes |= RigExecChangeGuides;
        }
        if (after.hasWeightOverlay &&
            before->weightOverlay != after.weightOverlay) {
            changes |= RigExecChangeWeightOverlay;
        }
        return changes;
    }

    RigExecImagingSnapshotConstPtr _current;
    std::mutex _writeMutex;
};

}  // namespace rigExec

#endif  // RIGEXEC_IMAGING_SNAPSHOT_STORE_H
