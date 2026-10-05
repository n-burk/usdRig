// .rigexec path-read enumeration: every stage key the packet assembly of a
// revision or a weight object can read, whichever branch its inputs take.
// One run records only the keys its branches reached (a disabled mover reads
// nothing past inputs:enabled, an invalid envelope stops before the op's
// reads, a volume gathers only once it can build); the enumeration lists the
// whole set from the program's structure, each key with the value its site
// consumes at that time. A bake writes both: the keys its run recorded, then
// every key the run's branches did not reach, so an input set on the binary
// that takes another branch reads what the stage holds there.
//
// Keys and values mirror the bake recorder (RigExecRecordStageRead and the
// recording array reads in moverGraph.cpp, the gathers in bakedWeights.cpp)
// site for site: the same key, the same rest flag (read at Default), the
// same fallback, the same value type, and Absent where the site records a
// missing attribute. A connection-following site reads through the overlay
// the run's steps read through (the program's property results, and a
// revision's declared read phases looked up in the run's snapshot store), so
// an enumeration follows a run of the program at the same time.
#ifndef RIGEXEC_BAKE_REVISION_READS_H
#define RIGEXEC_BAKE_REVISION_READS_H

#include "rigExec/bakedProgramImpl.h"
#include "rigExecBinary/inputTable.h"

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
    /// property result, or a phase's snapshot), which the recorder never
    /// records: the runtime recomputes it.
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
/// points, and a volume's target, sample and curve points. A gather records
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

/// What a check of recorded path reads against an enumeration counted.
struct RigExecBakePathReadCheck {
    /// Entries in the record.
    size_t recorded = 0;
    /// Distinct (path, rest) keys in the enumeration.
    size_t enumerated = 0;
    /// Entries whose key the enumeration does not list, accepted only when
    /// the check accepts them (a plugin mover's own reads).
    size_t unenumerated = 0;
};

/// Checks that every entry of \p recorded is a (path, rest) key of
/// \p enumerated with a bitwise-equal value (RigExecBakeSamePathValue over
/// RigExecBakeEncodePathValue), and that an entry recorded as following
/// connections is enumerated as such. A key enumerated more than once
/// matches when any of its values does. With \p acceptUnenumerated, an
/// entry whose key the enumeration does not list is counted rather than
/// refused: a plugin mover's assembly records reads of its own, which the
/// enumeration (stopping at a plugin op's enable) cannot list. \p text
/// resolves the record's string ids (paths and token values). On failure
/// \p error names the first offender in record order.
bool RigExecBakeCheckPathReads(
    const std::vector<RigExecWirePathRead> &recorded,
    const std::vector<RigExecBakeRevisionRead> &enumerated,
    const std::function<bool(uint32_t, std::string *)> &text,
    RigExecBakePathReadCheck *check, std::string *error,
    bool acceptUnenumerated = false);

/// Appends to \p record every (path, rest) key of \p enumerated it lacks,
/// in enumeration order, the first time each appears not overlaid: the
/// keys a run reaching their sites would record, with the values those
/// sites consume (a connection-following site's entry marked so). An
/// overlaid key is left out, as the recorder leaves it. \p intern gives a
/// path's or a token's string id; a key whose path the record already
/// names is compared by that id. False, naming the read, for a value no
/// record can carry.
bool RigExecBakeCompletePathReads(
    const std::vector<RigExecBakeRevisionRead> &enumerated,
    const std::function<uint32_t(const std::string &)> &intern,
    std::vector<RigExecWirePathRead> *record, std::string *error);

}  // namespace rigExec

#endif  // RIGEXEC_BAKE_REVISION_READS_H
