//
// .rigexec serialization: the live baked program as wire structs.
//
// Converts RigExecBakedProgramImpl's epoch-stable state into the plain wire
// structs libs/rigExecBinary/program.h defines. Per-run state -- scratch
// buffers, last-run comparisons, counters, timestamps, diagnostics -- is
// deliberately NOT converted: a file carries what Build decided, and the
// runtime re-derives the rest by running.
//
// Internal to rigExecBake: the Impl type is that library's business, and
// nothing outside it includes this header.
//
#ifndef RIGEXEC_BAKE_SERIALIZE_H
#define RIGEXEC_BAKE_SERIALIZE_H

#include "rigExecBinary/container.h"
#include "rigExecBinary/pose.h"
#include "rigExecBinary/geometry.h"
#include "rigExecBinary/program.h"

namespace rigExec {

struct RigExecBakedProgramImpl;

/// Converts the program's slot inventory, interning paths into \p writer's
/// string table.
RigExecWireSlotMeta RigExecBakeConvertSlotMeta(
    const RigExecBakedProgramImpl &program, RigExecBinaryWriter *writer);

/// Converts the epoch constants, interning rotation-order tokens.
RigExecWireConstants RigExecBakeConvertConstants(
    const RigExecBakedProgramImpl &program, RigExecBinaryWriter *writer);

/// Converts the step list, interning labels.
std::vector<RigExecWireStep> RigExecBakeConvertSteps(
    const RigExecBakedProgramImpl &program, RigExecBinaryWriter *writer);

/// Converts the cluster partition and the cone closures.
RigExecWireClustering RigExecBakeConvertClustering(
    const RigExecBakedProgramImpl &program);
RigExecWireCones RigExecBakeConvertCones(
    const RigExecBakedProgramImpl &program);

/// Converts the pose-domain build tables.
RigExecWireDomainPose RigExecBakeConvertDomainPose(
    const RigExecBakedProgramImpl &program, RigExecBinaryWriter *writer);

/// Converts the geometry-domain build tables.
RigExecWireDomainGeometry RigExecBakeConvertDomainGeometry(
    const RigExecBakedProgramImpl &program, RigExecBinaryWriter *writer);

}  // namespace rigExec

#endif  // RIGEXEC_BAKE_SERIALIZE_H
