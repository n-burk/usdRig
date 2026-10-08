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
// value is published), and every attribute an array read reaches
// (RigExecBakeListArrayReads). Each scalar slot's default is a raw typed
// Get at the bake time -- no walk, no overlay -- because the runtime
// performs the walks; an array slot's is taken after the bake's run
// (RigExecBakeArraySlot). Never evaluates. Internal to rigExecBake.
#ifndef RIGEXEC_BAKE_COMPUTED_CAPTURE_H
#define RIGEXEC_BAKE_COMPUTED_CAPTURE_H

#include "rigExecBinary/format.h"

#include "pxr/base/vt/value.h"
#include <map>
#include <tuple>
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

/// An array input whose default the bake's run decides: the attribute's
/// value at the bake time, or at Default when every read of it is at
/// Default; a chain's base, as the pool entry the chain stores; a fixed
/// skin revision's layout array, as the stored layout when the run stored
/// one. Until then the slot's default is its tag's empty array.
struct RigExecBakeArraySlot {
    uint32_t slot = 0;
    fb::InputTag tag = fb::InputTag::FloatArray;
    bool atDefault = false;
    /// The chain whose base it is, or -1.
    int32_t chain = -1;
    /// The chain revision whose layout array it is (chain, revision), or
    /// -1, and whether it holds the indices rather than the weights.
    int32_t layoutChain = -1;
    int32_t layoutRevision = -1;
    bool layoutIndices = false;
};

/// The path-read row of key (\c path, \c rest) an array read binds: its
/// read over the slots, which a rest row pairs with its Default-time value.
struct RigExecBakeArrayRow {
    uint32_t path = 0;
    bool rest = false;
    /// A weight object's gather: the row is written even where the
    /// enumeration keys none, since its read answered nothing at the time.
    bool gather = false;
    fb::RigExecWireInput read;
};

/// A dense blend sample's points read over the slots.
struct RigExecBakeBlendPointsRead {
    uint32_t chain = 0;
    /// The revision, or the derived entry when \c derived.
    uint32_t revision = 0;
    bool derived = false;
    uint32_t channel = 0;
    uint32_t sample = 0;
    fb::RigExecWireInput read;
};

/// A fixed main skin revision's layout inputs.
struct RigExecBakeLayoutSlots {
    uint32_t chain = 0;
    uint32_t revision = 0;
    uint32_t indices = 0;
    uint32_t weights = 0;
};

/// What the collection gathered. Every value id indexes \c values, every
/// walk entry \c inputs; values[0] is Double +0.0, as the file's pool
/// holds it.
struct RigExecBakeExternalInputs {
    std::vector<fb::RigExecWireExternalDeclaredInput> reads;
    std::vector<pxr::VtValue> fallbacks;
};
struct RigExecBakeInputs {
    std::vector<fb::RigExecWireValue> values;
    /// Every slot, listed ones first, those ordered by path text.
    std::vector<fb::InputSlot> inputs;
    uint32_t listedInputs = 0;
    /// The array inputs, in slot order; the path-read rows, blend sample
    /// points and layouts their reads bind; and per chain its base input,
    /// or -1. A weight object names its painted and oracle inputs itself.
    std::vector<RigExecBakeArraySlot> arraySlots;
    std::vector<RigExecBakeArrayRow> arrayRows;
    std::vector<RigExecBakeBlendPointsRead> blendPoints;
    std::vector<RigExecBakeLayoutSlots> layoutSlots;
    std::vector<int32_t> chainBaseSlots;
    /// The step-backed objects in the program's order, then the
    /// envelope-only ones, each with its reads and oracle facts.
    std::vector<fb::RigExecWireWeightObject> weightObjects;
    std::vector<fb::RigExecWireWeightField> weightFields;
    /// Per program constraint, the weight object its envelope arm reads,
    /// or -1.
    std::vector<int32_t> constraintWeightObjectIndex;
    std::map<std::pair<uint32_t,uint32_t>,RigExecBakeExternalInputs> externalInputs;
    /// (chain, revision/derived index, derived, layout) identifies the exact native owner.
    std::map<std::tuple<uint32_t,uint32_t,bool,bool>,RigExecBakeExternalInputs> leafSites;
    std::vector<std::array<int32_t,4>> constraintRawSlots;
    std::vector<int32_t> providerLeafSlots;
    std::vector<int32_t> crossDomainRawSlots;
    std::vector<int32_t> derivedBaseSlots;
    std::vector<std::vector<uint32_t>> headInputSlots;
    std::vector<std::vector<fb::RigExecWireInput>> headInputReads;
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
