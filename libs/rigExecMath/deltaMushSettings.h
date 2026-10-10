#ifndef RIGEXEC_MATH_DELTA_MUSH_SETTINGS_H
#define RIGEXEC_MATH_DELTA_MUSH_SETTINGS_H

#include <string_view>
#include <vector>

namespace rigExec {

enum class RigExecDeltaMushSmoothing { Rest = 0, Simple = 1, LengthWeighted = 2 };
enum class RigExecDeltaMushFrameTransport { Vertex = 0, Corner = 1 };

// Text parsers take a view so token text, strings and literals parse
// without building a temporary or touching a token registry.
inline bool RigExecParseDeltaMushSmoothing(std::string_view token, RigExecDeltaMushSmoothing *out) {
    if (token == "rest") *out = RigExecDeltaMushSmoothing::Rest;
    else if (token == "simple") *out = RigExecDeltaMushSmoothing::Simple;
    else if (token == "lengthWeighted") *out = RigExecDeltaMushSmoothing::LengthWeighted;
    else return false;
    return true;
}
inline bool RigExecParseDeltaMushFrameTransport(std::string_view token, RigExecDeltaMushFrameTransport *out) {
    if (token == "vertex") *out = RigExecDeltaMushFrameTransport::Vertex;
    else if (token == "corner") *out = RigExecDeltaMushFrameTransport::Corner;
    else return false;
    return true;
}

/// Iterative smoothing controls, distinct from the final mover envelope.
/// The defaults are the legacy deformation: rest-length weights, vertex
/// frames, uniform influence, polygon edges.
struct RigExecDeltaMushSettings {
    RigExecDeltaMushSmoothing smoothing = RigExecDeltaMushSmoothing::Rest;
    RigExecDeltaMushFrameTransport frameTransport = RigExecDeltaMushFrameTransport::Vertex;
    std::vector<float> smoothWeights;
    /// Optional flattened edge pairs; empty derives unique polygon edges.
    std::vector<int> edges;
    bool onlySmooth = false;

    bool operator==(const RigExecDeltaMushSettings &o) const {
        return smoothing == o.smoothing && frameTransport == o.frameTransport &&
            smoothWeights == o.smoothWeights && edges == o.edges && onlySmooth == o.onlySmooth;
    }
    bool operator!=(const RigExecDeltaMushSettings &o) const { return !(*this == o); }
    /// Whether these are the legacy defaults.
    bool IsLegacy() const {
        return smoothing == RigExecDeltaMushSmoothing::Rest &&
            frameTransport == RigExecDeltaMushFrameTransport::Vertex &&
            smoothWeights.empty() && edges.empty() && !onlySmooth;
    }
};

} // namespace rigExec
#endif
