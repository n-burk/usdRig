// RigExec baked playback for Hydra (M2b): answers a rig from its .rigexec
// file instead of evaluating it.
// Selected per rig at activation: a RigExecRoot carrying rigExec:asset
// plays the binary it names, and a rig without the attribute (or whose
// file cannot be opened) evaluates live. The live path is otherwise
// untouched -- this class owns no evaluator, takes no locks the registry
// does not already hold, and publishes snapshots in exactly the shape
// the results scene index reads.
// What one generation carries, and what it deliberately does not:
//   * deformed points, normals and extents, split out of the runtime's
//     moved properties by property name, exactly the three geometry
//     leaves the bridge publishes;
//   * constraint-driven provider transforms as (revised, base) pairs,
//     so parented geometry rides along through the same world-space
//     delta the live path computes;
//   * the selected weight object's resolved field as the influence
//     overlay, from the runtime's published weight fields;
//   * NO guides (joint skeletons, control shapes, volume iso-surfaces):
//     guide drawing stays a live-path visualisation,
//     and a playback generation simply owns no guide leaves;
//   * NO movedFloats: the runtime publishes no scalar moved properties,
//     so tool reads of RigExecImaging_GetMovedFloats find nothing on a
//     playback session rather than a recomputed guess.
// Each generation plays the binary over the stage's values at the
// requested time: the inputs the file marks Animated are read from the
// stage there (RigExecInputSampler), every other input keeps the value it
// had at the bake time. UsdTimeCode::Default is a time like any other; a
// keyed input with no default value holds no value there, as on the stage,
// and so does one whose sample is blocked. Stage edits never dirty a
// playback session, so an edit is read only at the next change of time,
// and only on an Animated input; changing the asset, or the file it names,
// needs a re-activation, the same rule as a rig authored into an
// already-active stage.
// Upstream scene-index values (docs/specs/upstream-inputs.md, "Playback")
// reach the binary through its inputs, as authored values: after each
// Apply, every admitted key is set where the stage value stood, and a
// lifted key returns to what the stage says. The sampler is never
// invalidated for them, so an upstream change dirties only what reads it.
#ifndef RIGEXEC_IMAGING_PLAYBACK_H
#define RIGEXEC_IMAGING_PLAYBACK_H

#include "bridge.h"

#include "rigExecRuntime/runtime.h"
#include "rigExecSampler/inputSampler.h"

#include "pxr/base/tf/type.h"
#include "pxr/base/vt/value.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/timeCode.h"

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace rigExec {

/// Plays one rig's .rigexec file into the imaging chain. Constructed over
/// an externally owned store, like the bridge; Open() must succeed before
/// the first EvaluateAndPublishResult.
class RigExecBakedPlayback {
public:
    RigExecBakedPlayback(
        const UsdStageRefPtr &stage, const SdfPath &rigPath,
        std::shared_ptr<RigExecSnapshotStore> store);
    // Out of line: the reader's destructor lives in the runtime library.
    ~RigExecBakedPlayback();

    /// Opens the .rigexec at \p resolvedPath and binds its inputs to the
    /// stage (an input the stage lacks keeps its bake-time value, with a
    /// warning). False with the reason when the file cannot be read or
    /// parsed (an unplayable binary falls back to live evaluation at
    /// activation rather than failing it).
    bool Open(const std::string &resolvedPath, std::string *error);

    /// The opened file, empty until Open succeeds.
    const std::string &GetAssetPath() const { return _assetPath; }

    /// Plays the binary over the stage's input values at \p time and
    /// publishes the complete generation, without notifying any scene
    /// index; the caller forwards the result to its chains. A run the
    /// binary refuses clears the store (the bridge's rule: an unevaluated
    /// result is recoverable, a plausible wrong one is not) and returns
    /// ok == false.
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

    /// Upstream values for this rig: authored-level, each replacing the
    /// stage value of its attribute at every time. The list replaces the
    /// last one; an empty list lifts every value. Admitted now and again at
    /// each EvaluateAndPublishResult, which applies them. A key is admitted
    /// when FindInput finds it as a listed input, the value holds exactly
    /// that input's type, and on the stage the attribute is unconnected and
    /// has a value; every other key is ignored with live's line, "upstream
    /// input <path>: <reason>; ignored" (GetUpstreamDropLines). Array values
    /// require the stage's count; a runtime count refusal lifts the key.
    void SetUpstreamInputs(std::vector<RigExecUpstreamValue> values);

    /// The list SetUpstreamInputs last received.
    const std::vector<RigExecUpstreamValue> &GetUpstreamInputs() const {
        return _upstreamRequested;
    }

    /// The admitted keys, sorted.
    std::vector<SdfPath> GetUpstreamInputPaths() const;

    /// One line per key the last admission ignored, in the order given.
    const std::vector<std::string> &GetUpstreamDropLines() const {
        return _upstreamDropLines;
    }

    /// The reader, for tests that compare its outputs; null before Open.
    const RigExecRuntimeReader *GetReaderForTesting() const {
        return _reader.get();
    }

private:
    // An admitted upstream key: the input it sets and the value.
    struct _UpstreamKey {
        size_t index = 0;
        std::string name;
        RrInputTag tag = RrInputTag::Double;
        bool animated = false;
        UsdAttribute attribute;
        VtValue value;
    };

    void _AdmitUpstream(UsdTimeCode time);
    bool _ApplyUpstream(UsdTimeCode time, bool sampled, std::string *error);
    bool _LiftUpstream(const _UpstreamKey &key, UsdTimeCode time,
                       bool sampled, std::string *error);

    UsdStageRefPtr _stage;
    SdfPath _rigPath;
    SdfPath _assetRoot;
    std::shared_ptr<RigExecSnapshotStore> _store;
    std::unique_ptr<RigExecRuntimeReader> _reader;
    RigExecInputSampler _sampler;
    std::string _assetPath;
    uint64_t _epochDigest = 0;
    SdfPath _weightOverlay;
    uint64_t _generation = 0;
    uint64_t _publishedEpochDigest = 0;
    // The listed inputs by path, with the type each holds (condition 3).
    std::map<SdfPath, TfType> _listedInputs;
    std::vector<RigExecUpstreamValue> _upstreamRequested;
    // Admitted by the last admission, and set on the reader by the last
    // application; a key in the second and not the first is lifted next.
    std::map<SdfPath, _UpstreamKey> _upstreamAdmitted;
    std::map<SdfPath, _UpstreamKey> _upstreamApplied;
    std::vector<std::string> _upstreamDropLines;
    UsdTimeCode _upstreamTime = UsdTimeCode::Default();
};

/// Resolves the playback file a rig names, if any. True with the playable
/// path when \p rig carries a non-empty rigExec:asset that resolves
/// against the layer that authored it; false otherwise (unset, empty, or
/// unresolvable), in which case the rig evaluates live.
bool RigExecPlaybackAssetFor(const UsdPrim &rig, std::string *resolvedPath);

}  // namespace rigExec

#endif  // RIGEXEC_IMAGING_PLAYBACK_H
