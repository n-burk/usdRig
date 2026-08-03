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
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

/// The published results for one Hydra prim in one generation. Only
/// standard data crosses this boundary (spec §10.2): local xforms,
/// points/normals as flat primvars, and the two-element extent.
struct RigExecPublishedPrim {
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

    /// Explicit motion samples (spec 10.5 subset): frame-relative shutter
    /// offsets and the retained point sets captured at each offset under
    /// this generation fence. Empty means single-sample publication; when
    /// present, `points` holds the base (offset-nearest-zero) sample.
    std::vector<float> sampleOffsets;
    std::vector<VtVec3fArray> pointsSamples;

    /// Guide drawing payload (joints and aggregate solvers draw as guide
    /// geometry like OpenExec's IrJointScope): one rig-space frame matrix,
    /// cone length, and primitive radius per guide element, plus constant
    /// styling from the authored guide attributes.
    bool hasGuides = false;
    std::vector<GfMatrix4d> guideFrames;
    std::vector<double> guideLengths;
    /// Radius of both the sphere and the cone, per element. Parallel to
    /// guideFrames, so a lookup valid for one is valid for all three.
    std::vector<double> guideRadii;
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
    /// it, because the two are different drawings: the joint payload is N
    /// sphere+cone pairs whose only authored dimension is a radius, while
    /// this is a single shape whose prim type depends on the authored
    /// shape/drawMode pair. Sharing the array would make "which element is
    /// which kind" a thing every consumer had to decide.
    ///
    /// guideColor/guideOpacity above are shared: both payloads read the
    /// same guide:displayColor / guide:displayOpacity attributes, and a
    /// prim never carries both payloads (a control is not a joint).
    bool hasControlGuide = false;
    /// Rigidized, ASSET-space, like guideFrames.
    GfMatrix4d controlGuideFrame{1.0};
    /// sphere|circle|box|cube|diamond|pyramid.
    TfToken controlGuideShape;
    /// wire|geometry.
    TfToken controlGuideDrawMode;
    /// Per-axis draw scale. Authored as three separate doubles (deliberately
    /// not a vec3, per direction); stored as one vector because nothing
    /// downstream has a reason to take the axes apart again.
    GfVec3d controlGuideScale{1.0, 1.0, 1.0};
    /// Width for the wire draw mode, in the guide's LOCAL pre-scale units.
    /// Zero or negative publishes no widths at all (hairline fallback);
    /// the geometry draw mode ignores it.
    double controlGuideWireWidth = 0.05;
};

/// One complete immutable generation (spec §8.2: consumers see complete
/// RigExecSnapshot generations only).
struct RigExecImagingSnapshot {
    uint64_t generation = 0;
    std::map<SdfPath, RigExecPublishedPrim> prims;

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

    /// The rig's asset root (the rig prim's parent).
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
            before->hasControlGuide != after.hasControlGuide) {
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
        // Styling is shared by both guide payloads (one prim never carries
        // both), so it is compared once here and folded into whichever one
        // this prim publishes.
        const bool styleChanged = before->guideColor != after.guideColor ||
                                  before->guideOpacity != after.guideOpacity;
        // Both halves matter: the consumer publishes a delta built from the
        // pair, so a moving base with a static revision still moves the prim.
        if (after.hasXform && (before->xform != after.xform ||
                               before->xformBase != after.xformBase)) {
            changes |= RigExecChangeXform;
        }
        if (after.hasPoints &&
            (before->points != after.points ||
             before->sampleOffsets != after.sampleOffsets ||
             before->pointsSamples != after.pointsSamples)) {
            changes |= RigExecChangePoints;
        }
        if (after.hasNormals && before->normals != after.normals) {
            changes |= RigExecChangeNormals;
        }
        if (after.hasExtent && (before->extentMin != after.extentMin ||
                                before->extentMax != after.extentMax)) {
            changes |= RigExecChangeExtent;
        }
        if (after.hasGuides &&
            (before->guideFrames != after.guideFrames ||
             before->guideLengths != after.guideLengths ||
             before->guideRadii != after.guideRadii ||
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
        return changes;
    }

    RigExecImagingSnapshotConstPtr _current;
    std::mutex _writeMutex;
};

}  // namespace rigExec

#endif  // RIGEXEC_IMAGING_SNAPSHOT_STORE_H
