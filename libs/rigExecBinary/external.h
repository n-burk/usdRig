// External movers in a .rigexec file, and the playback contract a runtime
// calls in their place.
//
// A plugin mover assembles its payload from the stage, which playback does
// not have. Export therefore asks the plugin to split its assembled payload
// into EPOCH bytes, written once per revision, and FRAME bytes. Playback
// hands both back to the plugin's kernel together with the points the
// mover's binding reads at a declared phase, which the runtime evaluates
// itself -- so a posed playback moves those -- and with the preceding
// points to revise. The bytes are the plugin's own format: the engine never
// interprets them. The file holds one ExternalMover entry per plugin
// revision (rigexec.fbs): its type, its epoch bytes and the frame bytes of
// the bake's run.
//
// Everything here is plain data and function pointers: a runtime without
// USD can carry a kernel, and a runtime with no kernel for a type passes
// that type's movers through with a warning.
#ifndef RIGEXEC_BINARY_EXTERNAL_H
#define RIGEXEC_BINARY_EXTERNAL_H

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace rigExec {

/// One input the mover's binding reads at a declared phase, as playback
/// evaluated it this frame. In the binding's phase order, which is the
/// order of its input paths.
struct RigExecExternalPhasedPoints {
    /// The input's attribute path, e.g. "/Asset/Geom/Net.points".
    const char *path = nullptr;
    /// xyz triples, or null when playback holds no value at that phase;
    /// the frame bytes then carry whatever the export read.
    const float *xyz = nullptr;
    size_t count = 0;
};

/// A plugin mover's playback kernel.
struct RigExecExternalKernel {
    /// Decodes one revision's epoch bytes into immutable state, once per
    /// opened file. Null, with a reason, when the bytes are not ones this
    /// kernel can read; the revision then passes its points through.
    std::shared_ptr<const void> (*prepare)(
        const uint8_t *epoch, size_t epochSize, std::string *error) = nullptr;
    /// Revises \p xyz (\p pointCount triples, the preceding revision's
    /// points) in place to the full-strength candidate; the runtime applies
    /// the mover's envelope afterwards. Must not change the point count or
    /// keep state between calls. False fails the mover for this frame, and
    /// the preceding points stand.
    bool (*apply)(const void *state, const uint8_t *frame, size_t frameSize,
                  const RigExecExternalPhasedPoints *phased,
                  size_t phasedCount, float *xyz, size_t pointCount) = nullptr;

    bool IsSet() const { return prepare && apply; }
};

/// The revision op a plugin mover is written with (RigExecRevisionOp::
/// External, pinned by the format).
inline constexpr uint8_t RigExecWireExternalRevisionOp = 16;

}  // namespace rigExec

#endif  // RIGEXEC_BINARY_EXTERNAL_H
