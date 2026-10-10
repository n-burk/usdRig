// .rigexec path-read enumeration: every stage key the packet assembly of a
// revision or a weight object can read, whichever branch its inputs take.
// One run reaches only some of them (a disabled mover reads nothing past
// inputs:enabled, an invalid envelope stops before the op's reads, a volume
// gathers only once it can build); the enumeration lists the whole set from
// the program's structure, each key with the value its site consumes at
// that time. The file's path reads are this set, so an input set on the
// binary that takes another branch reads what the stage holds there.
//
// Keys and values follow the assembly's read sites (moverGraph.cpp's mover
// reads and _Array, the gathers in bakedWeights.cpp) site for site: the
// same key, the same rest flag (read at Default), the same fallback, the
// same value type, and Absent where the site finds no attribute. A
// connection-following site reads through the overlay the run's steps read
// through (the program's property results, and what each of a revision's
// point bindings resolves to), so an enumeration follows a run of the
// program at the same time.
#ifndef RIGEXEC_BAKE_REVISION_READS_H
#define RIGEXEC_BAKE_REVISION_READS_H

#include "rigExec/bakedProgramImpl.h"

#include <pxr/base/vt/value.h>
#include <pxr/usd/sdf/path.h>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace rigExec {

/// One key an assembly can read.
struct RigExecBakeRevisionRead {
    /// The attribute the site reads, or the path it names when no attribute
    /// stands there.
    PXR_NS::SdfPath path;
    /// Read at Default (an authored rest value) rather than at the time.
    bool rest = false;
    /// The site follows connections through RigExecResolvedInputs, so a
    /// property result or an upstream value can stand in for the attribute's
    /// own; false for a raw attribute read.
    bool resolved = false;
    /// The value the site consumes at the time, typed as the site types
    /// it. Empty for a known-absent attribute.
    PXR_NS::VtValue value;
    /// The site takes a value the run's overlay holds at the key itself (a
    /// property result, or the points a phased read is bound to), which the
    /// runtime recomputes rather than reads.
    bool overlaid = false;
};

using RigExecBakeRevisionReadSink =
    std::function<void(RigExecBakeRevisionRead &&)>;

/// Every key \p revision's assembly can read at \p time, delivered to
/// \p sink in the order its assembly reads them. \p revision is a chain
/// revision or a derived one of \p program (a derived matrix target reads
/// the projector's keys).
void RigExecBakeEnumerateRevisionReads(
    const RigExecBakedProgramImpl &program,
    const RigExecBakedProgramImpl::GeomRevision &revision,
    double time, const RigExecBakeRevisionReadSink &sink);

/// Every key the packet of weight object \p object (an index into
/// \p program's weight objects) gathers at \p time: the combine target's
/// points, and a volume's target, sample and curve points. A gather keys
/// only the attributes it read a value from, so an attribute with none is
/// not a key.
void RigExecBakeEnumerateWeightReads(
    const RigExecBakedProgramImpl &program, size_t object, double time,
    const RigExecBakeRevisionReadSink &sink);

/// Every chain revision's, derived target's and weight object's keys at
/// \p time, in program order, appended to \p reads.
void RigExecBakeEnumerateProgramReads(
    const RigExecBakedProgramImpl &program, double time,
    std::vector<RigExecBakeRevisionRead> *reads);

}  // namespace rigExec

#endif  // RIGEXEC_BAKE_REVISION_READS_H
