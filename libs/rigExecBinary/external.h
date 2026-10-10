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

// Plain API4 type IDs, pinned to the wire InputTag inventory.
enum class RigExecExternalInputType : uint8_t {
    Double = 0, Float = 1, Bool = 2, Int = 3, Matrix4d = 4, Token = 5,
    Vec3d = 6, Vec3f = 7, IntArray = 8, FloatArray = 9,
    DoubleArray = 10, Vec2fArray = 11, Vec3fArray = 12,
    Vec3dArray = 13, Matrix4dArray = 14, TokenArray = 15, BoolArray = 16, Vec3i = 17
};

/// One declared external input, in the plugin's declaration order. Type
/// uses the format's InputTag numeric IDs. Scalars have count one; arrays
/// have their actual count, including zero. A failed read has hasValue
/// false and null data. Storage remains valid only for the apply call.
/// Matrix4d is sixteen row-major doubles; Vec3d/Vec3f are three contiguous
/// components. Bool is uint8_t and Int is int32_t. Token data is a direct
/// null-terminated UTF-8 string (not a pointer to a string pointer), with
/// count one even for a successfully read empty token. Array counts are
/// element counts; successful empty arrays retain hasValue true.
struct RigExecExternalInputValue {
    uint8_t type = 0;
    bool hasValue = false;
    size_t count = 0;
    const void *data = nullptr;
};

/// The provider values the mover's binding reads, as playback evaluated
/// them this frame: the plain form of the assembly's provider values.
/// Matrices are sixteen row-major doubles. Storage remains valid only for
/// the apply call.
struct RigExecExternalProviders {
    /// The binding's transform provider, or null when it names none.
    const double *transform = nullptr;
    /// One matrix per binding influence, in binding order; null with a
    /// zero count when the binding names none.
    const double *influences = nullptr;
    size_t influenceCount = 0;
    /// The chain's base points (xyz triples) the assembly measures
    /// against.
    const float *basePoints = nullptr;
    size_t basePointCount = 0;
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
                  size_t phasedCount,
                  const RigExecExternalInputValue *inputs, size_t inputCount,
                  float *xyz, size_t pointCount) = nullptr;
    /// Optional: apply with the provider values the binding reads as well,
    /// for a mover whose result follows its influences or transform. When
    /// set, playback calls it in place of apply, under apply's contract.
    bool (*applyWithProviders)(const void *state, const uint8_t *frame,
                               size_t frameSize,
                               const RigExecExternalPhasedPoints *phased,
                               size_t phasedCount,
                               const RigExecExternalInputValue *inputs,
                               size_t inputCount,
                               const RigExecExternalProviders &providers,
                               float *xyz, size_t pointCount) = nullptr;

    bool IsSet() const { return prepare && (apply || applyWithProviders); }
};

/// The revision op a plugin mover is written with (RigExecRevisionOp::
/// External, pinned by the format).
inline constexpr uint8_t RigExecWireExternalRevisionOp = 16;

}  // namespace rigExec

#endif  // RIGEXEC_BINARY_EXTERNAL_H
