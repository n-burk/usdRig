// .rigexec input collection: the input slots every read of the file takes,
// with each slot's value at the bake time as its default, and the reads
// the program and the geometry assembly make over them, as the file's own
// object types.
// The slots are the attributes on the walks of every registered program
// read (Baked mode, as RigExecBakedRead resolves it), of the weight
// objects' scalar reads (the registered reads of every object a
// WeightPacket step bakes, and the Resolved reads of every envelope-only
// object, as the oracle's _ResolvedRead does), of the property movers'
// inputs (Pinned or Resolved, as _PinnedRead resolves them), of the
// geometry assembly's blend weights, activations, default weights and
// connection-following scalar reads, plus every chain target (its raw
// value is the chain's base) and every phased consumer (where a phased
// value is published). Each slot's default is a raw typed Get at the bake
// time -- no walk, no overlay -- because the runtime performs the walks.
// Never evaluates. Internal to rigExecBake.
#ifndef RIGEXEC_BAKE_COMPUTED_CAPTURE_H
#define RIGEXEC_BAKE_COMPUTED_CAPTURE_H

#include "rigExecBinary/format.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace rigExec {

class RigExecRigEvaluator;
class RigExecBakePathTable;

/// A weight object the runtime resolves whose oracle facts hold one time's
/// answer of an animated attribute: the facts are what the file holds, the
/// attribute is what the stage animates.
struct RigExecBakeTimeVaryingFact {
    std::string object;
    std::string attribute;
};

/// The program table a registered read sits in. Avar bindings are split
/// as the program splits them: the varying ones, then the constant ones
/// that carry an override number.
enum class RigExecBakeReadFamily : uint8_t {
    AvarBinding,
    AvarConstantBinding,
    Ladder,
    SpaceSwitch,
    Interpolator,
    Solver,
    Constraint,
    WeightObject,
};

/// A registered program read and the table field it fills. Fields are
/// numbered in each table's field order: ladder 0-15 (rest, default and
/// posed space, six rest avars, six default avars, rotation order), space
/// switch 0 (active), interpolator 0 (enabled) then 1 + k (the k-th dial),
/// solver 0-13, constraint 0-20, weight object 0-18, avar binding 0.
struct RigExecBakeRegisteredRead {
    RigExecBakeReadFamily family = RigExecBakeReadFamily::Ladder;
    uint32_t object = 0;
    uint32_t field = 0;
    fb::RigExecWireInput read;
};

/// One blend channel's weight read and its samples' activation reads, as
/// the geometry assembly reads them (Resolved, falling back to 0 and 1).
struct RigExecBakeBlendRead {
    uint32_t chain = 0;
    /// The revision, or the derived entry when \c derived.
    uint32_t revision = 0;
    bool derived = false;
    uint32_t channel = 0;
    fb::RigExecWireInput read;
    std::vector<fb::RigExecWireInput> activations;
};

/// A main revision's inputs:defaultWeight read (Resolved, falling back to
/// 1).
struct RigExecBakeDefaultWeightRead {
    uint32_t chain = 0;
    uint32_t revision = 0;
    fb::RigExecWireInput read;
};

/// A scalar mover input an assembler reads through its connection, keyed
/// by the head attribute: the resolved walk, then the head's own value,
/// then the fallback (moverGraph.cpp _Read).
struct RigExecBakePathScalarRead {
    /// The head attribute's path id.
    uint32_t path = 0;
    bool headFallback = false;
    fb::RigExecWireInput read;
};

/// What the collection gathered. Every value id indexes \c values, every
/// points id \c vec3fArrays, every walk entry \c inputs; values[0] is
/// Double +0.0 and vec3fArrays[0] is empty, as the file's pools hold them.
struct RigExecBakeInputs {
    std::vector<fb::RigExecWireValue> values;
    std::vector<fb::RigExecWireVec3fArray> vec3fArrays;
    /// Every slot, listed ones first, those ordered by path text.
    std::vector<fb::InputSlot> inputs;
    uint32_t listedInputs = 0;
    /// The step-backed objects in the program's order, then the
    /// envelope-only ones, each with its reads and oracle facts.
    std::vector<fb::RigExecWireWeightObject> weightObjects;
    /// Per program constraint, the weight object its envelope arm reads,
    /// or -1.
    std::vector<int32_t> constraintWeightObjectIndex;
    std::vector<fb::RigExecWirePropertyChain> propertyChains;
    std::vector<fb::RigExecWirePhasedConsumer> phasedConsumers;
    std::vector<RigExecBakeRegisteredRead> registeredReads;
    std::vector<RigExecBakeBlendRead> blendWeightReads;
    std::vector<RigExecBakeDefaultWeightRead> defaultWeightReads;
    std::vector<RigExecBakePathScalarRead> pathScalarReads;
};

class RigExecBakeComputedCapture {
public:
    RigExecBakeComputedCapture(const RigExecBakeComputedCapture &) = delete;
    RigExecBakeComputedCapture &operator=(
        const RigExecBakeComputedCapture &) = delete;

    /// Collects from \p evaluator's STANDING program (the caller compiles
    /// first), interning every path and token it names into \p paths. Slot
    /// defaults and oracle facts are read at \p time, the bake time. Check
    /// Valid before reading the result.
    RigExecBakeComputedCapture(RigExecRigEvaluator &evaluator, double time,
                               RigExecBakePathTable *paths,
                               std::string *error);
    ~RigExecBakeComputedCapture();

    bool Valid() const { return _valid; }

    const RigExecBakeInputs &GetInputs() const;

    /// Every resolved weight object whose facts are of an animated
    /// attribute, in the order the composition pass met them (composing
    /// objects before the ones they compose).
    const std::vector<RigExecBakeTimeVaryingFact> &GetTimeVaryingFacts() const;

    /// The listed inputs' names, ascending: the names a client sets.
    const std::vector<std::string> &GetListedInputNames() const;

private:
    struct _State;
    std::unique_ptr<_State> _state;
    bool _valid = false;
};

}  // namespace rigExec

#endif  // RIGEXEC_BAKE_COMPUTED_CAPTURE_H
