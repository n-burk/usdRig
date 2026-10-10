// RigExec pulled upstream inputs: what one pull of a chain's `rigExecInputs`
// sources leaves behind, and the value rule every reader of it applies.
//
// An upstream scene index publishes, on the prim that owns an attribute, a
// `rigExecInputs` container whose children are HdSampledDataSources named by
// attribute. The results scene index pulls them with no registry lock held
// (sceneIndices.cpp) and hands the table to its context, which stores it
// (RigExecImagingRegistry::SetUpstreamTable). Live hand-off and warming read
// only the table; nothing here calls a data source.
//
// TIME. T0 is the input scene index's time as RigExec last read it from a
// trigger. Sources answer relative to that time, so sample times are kept
// as absolute times (T0 + offset). A varying source is queried only at the
// times it returns and reconstructed as a stage reconstructs time samples:
// linear between bracketing samples for double, float, vec3d, vec3f,
// matrix4d and same-length float, double, vec2f and vec3f arrays under
// UsdInterpolationTypeLinear, held otherwise; the boundary sample holds
// outside the returned times, and a time equal to a sample's uses it as is.
//
// LAYOUT. One entry per attribute path. A uniform source keeps its value. A
// varying one keeps its samples, its value at T0 and, per integer frame of
// the pull window, the value (scalars) or the fold hash of the reconstructed
// array (arrays: RigExecUpstreamFoldHash), so the table holds no per-frame
// array. Values are shared VtValues: a held or uniform array shares one
// buffer with its sample.
#ifndef RIGEXEC_IMAGING_UPSTREAM_TABLE_H
#define RIGEXEC_IMAGING_UPSTREAM_TABLE_H

#include "rigExec/frozenContext.h"

#include "pxr/base/vt/value.h"
#include "pxr/usd/sdf/path.h"

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace rigExec {

/// One value a varying source returned, at absolute time \p time.
struct RigExecUpstreamTableSample {
    double time = 0.0;
    VtValue value;
};

/// One attribute's pulled source.
struct RigExecUpstreamTableEntry {
    bool varies = false;
    bool isArray = false;
    /// The value at T0 (a uniform source's only value) and, for an array,
    /// its fold hash.
    VtValue atT0;
    uint64_t hashAtT0 = 0;
    /// A varying source's samples, sorted by time.
    std::vector<RigExecUpstreamTableSample> samples;
    /// A varying scalar's value per window frame.
    std::vector<VtValue> frameValues;
    /// A varying array's fold hash per window frame.
    std::vector<uint64_t> frameHashes;
};

/// One chain's pulled sources (see the file comment).
struct RigExecUpstreamTable {
    double t0 = 0.0;
    /// The pull window, and its integer frames [firstFrame, firstFrame +
    /// frameCount).
    double windowLo = 0.0;
    double windowHi = 0.0;
    int64_t firstFrame = 0;
    size_t frameCount = 0;
    /// The stage's interpolation was Held when the table was pulled.
    bool held = false;
    std::map<SdfPath, RigExecUpstreamTableEntry> entries;

    bool AnyVaries() const;

    /// The value of \p entry at absolute time \p t: the T0 value at T0, the
    /// frame's value (or the reconstruction of an array with the frame's
    /// hash) at a window frame, a reconstruction elsewhere inside the
    /// window, and a uniform source's value anywhere. False when a varying
    /// source has no value at \p t (outside the window).
    bool ValueAt(const SdfPath &path, const RigExecUpstreamTableEntry &entry,
                 double t, RigExecUpstreamValue *out) const;

    /// Every entry's value at \p t, sorted by path. False, with \p out
    /// holding the entries that have one, when some varying source has
    /// none at \p t.
    bool ValuesAt(double t, std::vector<RigExecUpstreamValue> *out) const;
};

/// The value at absolute time \p t reconstructed from \p samples (sorted,
/// non-empty) under the Contract's rule; \p held forces held interpolation.
VtValue RigExecReconstructUpstream(
    const std::vector<RigExecUpstreamTableSample> &samples, double t,
    bool held);

/// Fills \p entry for a uniform source holding \p value. False for an
/// empty value.
bool RigExecFillUniformUpstreamEntry(const VtValue &value,
                                     RigExecUpstreamTableEntry *entry);

/// Fills \p entry for a varying source from the values it returned at its
/// sample times (\p samples, absolute times, any order; empty values are
/// skipped): the samples, the value at \p table's T0, and the per-frame
/// values or fold hashes over \p table's window. A frame that reconstructs
/// to a sample's own value reuses that sample's hash. Runs with no lock
/// held. False when no sample holds a value.
bool RigExecFillVaryingUpstreamEntry(
    const RigExecUpstreamTable &table,
    std::vector<RigExecUpstreamTableSample> samples,
    RigExecUpstreamTableEntry *entry);

/// Moves \p table to a new T0 inside its window with its sources unchanged:
/// every varying entry's value at \p t0 is re-derived from its samples and
/// per-frame tables, with no data source call.
void RigExecReanchorUpstreamTable(RigExecUpstreamTable *table, double t0);

/// Whether \p a and \p b describe the same sources: the same paths, kinds,
/// values, samples and per-frame values or hashes, and (when any source
/// varies) the same window. T0 and the values at T0 are not compared: a
/// table that only moved T0 describes the same signals.
bool RigExecSameUpstreamSources(const RigExecUpstreamTable &a,
                                const RigExecUpstreamTable &b);

/// The distinct array buffers \p table holds (samples, T0 values and
/// uniform values), counting a shared buffer once. For tests.
size_t RigExecUpstreamTableBufferCount(const RigExecUpstreamTable &table);

}  // namespace rigExec

#endif  // RIGEXEC_IMAGING_UPSTREAM_TABLE_H
