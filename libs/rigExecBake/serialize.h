// .rigexec serialization: the live baked program as the file's object
// tables. Converts RigExecBakedProgramImpl's epoch-stable state into the
// RigExecWireFile a bake writes (rigExecBinary/format.h). Per-run state --
// scratch buffers, last-run comparisons, counters, timestamps -- is
// deliberately NOT converted here: a file carries what Build decided, the
// static data one run left is filled by RigExecBakeCaptureStatics, and the
// runtime re-derives the rest by running.
// Internal to rigExecBake: the Impl type is that library's business, and
// nothing outside it includes this header.
#ifndef RIGEXEC_BAKE_SERIALIZE_H
#define RIGEXEC_BAKE_SERIALIZE_H

#include "rigExecBinary/format.h"

#include <string>

namespace rigExec {

struct RigExecBakedProgramImpl;
struct RigExecBakeInputs;
class RigExecBakePathTable;
class RigExecBakePools;

/// Builds \p file's tables from \p program, plus the input list of
/// \p inputs (a collection over the same program interning through
/// \p paths): the slot inventory, constants, steps, clusters and cones, the
/// pose and geometry tables with every read in the field it was collected
/// for (each registered read, the geometry assembly's blend weight,
/// activation and default weight reads), the weight objects with their
/// oracle facts, the property chains and the phased consumers. Skin
/// topologies are written sparse. \p pools is seeded with \p inputs'
/// values and points first, so the ids its tables hold stand. The static
/// data a run leaves (RigExecBakeCaptureStatics), the external movers and
/// the root's own fields are not filled here. False, naming the table,
/// for a read the collection does not hold or a topology the format
/// cannot.
bool RigExecBakeFillFile(const RigExecBakedProgramImpl &program,
                         const RigExecBakeInputs &inputs,
                         RigExecBakePathTable *paths, RigExecBakePools *pools,
                         fb::RigExecWireFile *file, std::string *error);

}  // namespace rigExec

#endif  // RIGEXEC_BAKE_SERIALIZE_H
