//
// RigExec baked playback for Hydra (M2b): answers a rig from its .rigexec
// file instead of evaluating it.
//
// Selected per rig at activation: a RigExecRoot carrying rigExec:asset
// plays the binary it names, and a rig without the attribute (or whose
// file cannot be opened) evaluates live. The live path is otherwise
// untouched -- this class owns no evaluator, takes no locks the registry
// does not already hold, and publishes snapshots in exactly the shape
// the results scene index reads.
//
// What one generation carries, and what it deliberately does not:
//
//   * deformed points, normals and extents, split out of the runtime's
//     moved properties by property name, exactly the three geometry
//     leaves the bridge publishes;
//   * constraint-driven provider transforms as (revised, base) pairs,
//     so parented geometry rides along through the same world-space
//     delta the live path computes;
//   * the selected weight object's resolved field as the influence
//     overlay, from the runtime's published weight fields;
//   * NO guides (joint skeletons, control shapes, volume iso-surfaces,
//     curvenet guides): guide drawing stays a live-path visualisation,
//     and a playback generation simply owns no guide leaves;
//   * NO movedFloats: the runtime publishes no scalar moved properties,
//     so tool reads of RigExecImaging_GetMovedFloats find nothing on a
//     playback session rather than a recomputed guess.
//
// Time maps to the nearest baked frame, ties to the lower one: scrubbing
// between two baked frames holds the nearer pose, deterministically.
// UsdTimeCode::Default reads as 0.0 for the mapping. Stage edits never
// dirty a playback session -- the binary is static -- so changing the
// asset, or the file it names, needs a re-activation, the same rule as
// a rig authored into an already-active stage.
//

#ifndef RIGEXEC_IMAGING_PLAYBACK_H
#define RIGEXEC_IMAGING_PLAYBACK_H

#include "bridge.h"

#include "rigExecRuntime/runtime.h"

#include "pxr/usd/sdf/path.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"

#include <cstdint>
#include <memory>
#include <string>

namespace rigExec {

/// Plays one rig's .rigexec file into the imaging chain. Constructed over
/// an externally owned store, like the bridge; Open() must succeed before
/// the first EvaluateAndPublishResult.
class RigExecBakedPlayback {
public:
    RigExecBakedPlayback(
        const UsdStageRefPtr &stage, const SdfPath &rigPath,
        std::shared_ptr<RigExecSnapshotStore> store);

    /// Opens the .rigexec at \p resolvedPath. False with the reason when
    /// the file cannot be read or parsed, or when it carries no baked
    /// frames (an unplayable binary falls back to live evaluation at
    /// activation, like an unreadable one, rather than failing it).
    bool Open(const std::string &resolvedPath, std::string *error);

    /// The opened file, empty until Open succeeds.
    const std::string &GetAssetPath() const { return _assetPath; }

    /// Replays the baked frame nearest \p time and publishes the complete
    /// generation, without notifying any scene index; the caller forwards
    /// the result to its chains. A frame the binary refuses clears the
    /// store (the bridge's rule: an unevaluated result is recoverable, a
    /// plausible wrong one is not) and returns ok == false.
    RigExecImagingBridge::PublishResult EvaluateAndPublishResult(
        UsdTimeCode time);

    /// Stable per file (FNV-1a over the bytes): the published prim set of
    /// a binary never changes, so one epoch is published ever.
    uint64_t GetBindingEpochDigest() const { return _epochDigest; }

    /// The reserved generated scope this rig owns (for pruning). Playback
    /// publishes nothing under it; the scope keeps the pruning behavior
    /// identical between the live and playback sessions of one rig.
    SdfPath GetGeneratedScope() const;

    /// Selects the weight object painted as the influence overlay; empty
    /// turns it off. Takes effect on the next publication.
    void SetWeightOverlay(const SdfPath &weightPrimPath) {
        _weightOverlay = weightPrimPath;
    }

    const SdfPath &GetWeightOverlay() const { return _weightOverlay; }

    /// The baked frame \p time plays: the nearest frame time, ties to the
    /// lower one. Default reads as 0.0. Exposed for tests.
    double MapTimeToFrame(UsdTimeCode time) const;

private:
    UsdStageRefPtr _stage;
    SdfPath _rigPath;
    SdfPath _assetRoot;
    std::shared_ptr<RigExecSnapshotStore> _store;
    std::unique_ptr<RigExecRuntimeReader> _reader;
    std::string _assetPath;
    uint64_t _epochDigest = 0;
    SdfPath _weightOverlay;
    uint64_t _generation = 0;
    uint64_t _publishedEpochDigest = 0;
};

/// Resolves the playback file a rig names, if any. True with the playable
/// path when \p rig carries a non-empty rigExec:asset that resolves
/// against the layer that authored it; false otherwise (unset, empty, or
/// unresolvable), in which case the rig evaluates live.
bool RigExecPlaybackAssetFor(const UsdPrim &rig, std::string *resolvedPath);

}  // namespace rigExec

#endif  // RIGEXEC_IMAGING_PLAYBACK_H
