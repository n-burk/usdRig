#ifndef RIGEXEC_MATH_DELTA_MUSH_SETTINGS_H
#define RIGEXEC_MATH_DELTA_MUSH_SETTINGS_H

#include <vector>
#include <string>

namespace rigExec {

enum class RigExecDeltaMushSmoothing { Rest = 0, Simple = 1, LengthWeighted = 2 };
enum class RigExecDeltaMushFrameTransport { Vertex = 0, Corner = 1 };

inline bool RigExecParseDeltaMushSmoothing(const std::string &token, RigExecDeltaMushSmoothing *out) {
    if (token == "rest") *out = RigExecDeltaMushSmoothing::Rest;
    else if (token == "simple") *out = RigExecDeltaMushSmoothing::Simple;
    else if (token == "lengthWeighted") *out = RigExecDeltaMushSmoothing::LengthWeighted;
    else return false;
    return true;
}
inline bool RigExecParseDeltaMushFrameTransport(const std::string &token, RigExecDeltaMushFrameTransport *out) {
    if (token == "vertex") *out = RigExecDeltaMushFrameTransport::Vertex;
    else if (token == "corner") *out = RigExecDeltaMushFrameTransport::Corner;
    else return false;
    return true;
}

/// Iterative smoothing controls, distinct from the final mover envelope.
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
};

} // namespace rigExec
#endif
