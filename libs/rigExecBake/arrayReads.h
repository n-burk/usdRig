// .rigexec array inputs: the array reads a bake lists as input slots, chosen
// in one function (RigExecBakeListArrayReads) so that the evaluator's own
// admission set can replace the choice without touching the capture.
// The reads are the program's array leaves: a revision's assembly arrays,
// a fixed skin revision's layout, every dense blend sample's points, every
// weight object's point gathers and painted values and indices, the points
// the weight oracle samples (at the attribute it resolves its relationship
// to), and every chain's base. A read the phase overlay answers (a points
// input whose read phase resolves to a chain's points) reads no slot, nor
// does one that crosses an attribute of another type. Internal to
// rigExecBake.
#ifndef RIGEXEC_BAKE_ARRAY_READS_H
#define RIGEXEC_BAKE_ARRAY_READS_H

#include "rigExecBinary/format.h"

#include <pxr/base/tf/token.h>
#include <pxr/usd/sdf/path.h>

#include <cstdint>
#include <string>
#include <vector>

namespace rigExec {

struct RigExecBakedProgramImpl;

/// Which site of the file an array read is.
enum class RigExecBakeArrayConsumer : uint8_t {
    /// A path-read row: a revision's assembly read, or a weight object's
    /// point gather.
    Row,
    /// A dense blend sample's points (BlendSample.points_read).
    BlendPoints,
    /// A fixed skin revision's jointIndices or jointWeights.
    Layout,
    /// A chain target's points, the chain's base.
    ChainBase,
    /// A weight object's rigExec:values or rigExec:indices.
    Painted,
    /// The points the weight oracle samples, and a curve weight's curve.
    OracleSamples,
    /// Raw weightTarget fallback; its canonical path is already in the object.
    OracleFallback,
    OracleCurve,
};

/// One array read the file lists an input slot for.
struct RigExecBakeArrayRead {
    RigExecBakeArrayConsumer consumer = RigExecBakeArrayConsumer::Row;
    fb::InputTag tag = fb::InputTag::FloatArray;
    /// The attributes the read reaches, head first: the head alone for a
    /// raw read, else every hop of its connection walk, each an attribute
    /// of the read's element type.
    std::vector<PXR_NS::SdfPath> hops;
    /// Read at Default (an authored rest value) rather than at the time; a
    /// painted array is folded at Default.
    bool rest = false;
    /// A weight object's gather, which keys a row only where its read
    /// answered; its row is written regardless.
    bool gather = false;
    /// The owner: chain and revision (the derived entry when \c derived)
    /// and blend channel and sample; or the weight object entry. \c indices
    /// names the indices of a layout or painted pair.
    uint32_t chain = 0;
    uint32_t revision = 0;
    bool derived = false;
    uint32_t channel = 0;
    uint32_t sample = 0;
    uint32_t object = 0;
    bool indices = false;
};

/// A weight object entry of the file, as the choice needs it.
struct RigExecBakeArrayObject {
    PXR_NS::SdfPath path;
    PXR_NS::TfToken type;
    /// The weight oracle resolves the entry: a constraint's or a property
    /// mover's envelope, a current-phase field, or an object one composes.
    bool oracle = false;
    /// A volume whose samples are the points in flight.
    bool samplesInFlight = false;
};

/// Every array read of \p program at \p time, the bake time, in program
/// order: each chain's revisions then derived targets (assembly rows, blend
/// sample points, layouts), the weight objects' gathers, the chain bases,
/// then per entry of \p objects its painted arrays and oracle points. False
/// with the reason for an array leaf no site of the file reads.
bool RigExecBakeListArrayReads(
    const RigExecBakedProgramImpl &program,
    const std::vector<RigExecBakeArrayObject> &objects, double time,
    std::vector<RigExecBakeArrayRead> *reads, std::string *error);

}  // namespace rigExec

#endif  // RIGEXEC_BAKE_ARRAY_READS_H
