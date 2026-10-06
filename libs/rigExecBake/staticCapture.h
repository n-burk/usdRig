// .rigexec static capture: the static data a bake's one forced run leaves
// in the program, written into the file's tables in place, and the pools
// every value and array of the file is stored once in.
// The run reads every step's inputs at the bake time, so what it leaves is
// complete: transforms read off the stage, constraint arrays, chain and
// derived bases, blend sample points and layouts, ribbon driver points.
// The path reads come from the assembly's enumeration rather than from the
// run, so a key the run's branches did not reach is held too. Internal to
// rigExecBake.
#ifndef RIGEXEC_BAKE_STATIC_CAPTURE_H
#define RIGEXEC_BAKE_STATIC_CAPTURE_H

#include "rigExecBake/revisionReads.h"
#include "rigExecBinary/format.h"

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace rigExec {

struct RigExecBakedProgramImpl;
struct RigExecBakeInputs;
class RigExecBakePathTable;

/// The pool key of a value: its tag, its bits, and the bytes of the member
/// its tag names (zeros when that member is absent).
std::string RigExecBakeValueKey(const fb::RigExecWireValue &value);

/// The file's value pool and its five typed array pools, each entry stored
/// once by its content bytes. values[0] is Double +0.0 and every array
/// pool's [0] is the empty array.
class RigExecBakePools {
public:
    RigExecBakePools();

    /// Takes \p inputs' values in order, so the ids its tables hold name
    /// the same entries here. False, with the reason, when its values do
    /// not start as the pool does or hold one entry twice; call it before
    /// anything else is pooled.
    bool Seed(const RigExecBakeInputs &inputs, std::string *error);

    /// The id of \p value, stored with only the member its tag names.
    uint32_t Value(const fb::RigExecWireValue &value);
    uint32_t Ints(const int32_t *data, size_t count);
    uint32_t Floats(const float *data, size_t count);
    uint32_t Doubles(const double *data, size_t count);
    /// \p xy holds \p count pairs.
    uint32_t Vec2fs(const float *xy, size_t count);
    /// \p xyz holds \p count triples.
    uint32_t Vec3fs(const float *xyz, size_t count);

    /// Hands every pool to \p file; the pools are empty afterwards.
    void MoveInto(fb::RigExecWireFile *file);

private:
    std::vector<fb::RigExecWireValue> _values;
    std::vector<fb::RigExecWireIntArray> _ints;
    std::vector<fb::RigExecWireFloatArray> _floats;
    std::vector<fb::RigExecWireDoubleArray> _doubles;
    std::vector<fb::RigExecWireVec2fArray> _vec2fs;
    std::vector<fb::RigExecWireVec3fArray> _vec3fs;
    // Content bytes -> id; lookups only.
    std::map<std::string, uint32_t> _valueIds, _intIds, _floatIds,
        _doubleIds, _vec2fIds, _vec3fIds;
};

/// Fills the static fields of \p file, whose tables RigExecBakeFillFile
/// built from the same program, from \p program after the bake's forced
/// run: the xform bases, the native sources' frames, the delta bases, the
/// constraint arrays with the lines their reads reported, the chain and
/// derived bases, every blend sample's dense points as its resolved input
/// at \p time, the bake time (the run's own read where the sample reads
/// it; a sample whose point binding answers computes its points from the
/// bound version instead), the layout each run resolved (a layout the
/// cache refused included), each ribbon solver's driver points (the run's
/// when they vary, else the constant), and the defaults of the array
/// inputs (RigExecBakeArraySlot). The path reads are one read row per
/// connection-following scalar read of \p inputs, one per (path, rest) key
/// an array read binds (a rest one with its Default-time value; a gather's
/// even where \p enumerated keys none), and one value row per other key of
/// \p enumerated no overlay stands on, at its first such value, sorted by
/// (path id, rest). False, naming the datum, for a value no row can hold.
bool RigExecBakeCaptureStatics(
    const RigExecBakedProgramImpl &program, double time,
    const RigExecBakeInputs &inputs,
    const std::vector<RigExecBakeRevisionRead> &enumerated,
    RigExecBakePathTable *paths, RigExecBakePools *pools,
    fb::RigExecWireFile *file, std::string *error);

}  // namespace rigExec

#endif  // RIGEXEC_BAKE_STATIC_CAPTURE_H
