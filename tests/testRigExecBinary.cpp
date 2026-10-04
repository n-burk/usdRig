// .rigexec container + bake conformance.
#include "rigExecBake/bake.h"
#include "rigExecBake/capture.h"
#include "rigExec/frozenContextInternal.h"
#include "rigExec/rigEvaluator.h"
#include "rigExecBinary/computed.h"
#include "rigExecBinary/container.h"
#include "rigExecBinary/external.h"
#include "rigExecExampleFixtures.h"
#include "rigExecMath/propertyMath.h"

#include "pxr/base/plug/registry.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/xform.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <set>
#include <string>
#include <vector>

using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE

static int failures = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            ++failures;                                                    \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
        }                                                                  \
    } while (0)

// After the macro: the comparison helpers report through CHECK.
#include "rigExecBinaryCompare.h"

static std::vector<uint8_t>
Bytes(const std::string &path)
{
    std::ifstream stream(path, std::ios::binary);
    return std::vector<uint8_t>(std::istreambuf_iterator<char>(stream),
                                std::istreambuf_iterator<char>());
}

static void
TestWrinkleWireExtensions()
{
    std::vector<uint8_t> bytes;
    std::string error;
    RigExecWireDomainGeometry geometry;
    geometry.chains.resize(1);
    geometry.chains[0].revisions.resize(1);
    geometry.chains[0].revisions[0].op = 15;
    bytes.clear();
    CHECK(RigExecWireEncodeDomainGeometry(geometry, &bytes));
    RigExecWireReader geoCursor(bytes.data(), bytes.size());
    RigExecWireDomainGeometry decodedGeometry;
    CHECK(RigExecWireDecodeDomainGeometry(&geoCursor, &decodedGeometry, &error));
    CHECK(decodedGeometry.chains.size() == 1);
    if (decodedGeometry.chains.size() == 1) {
        CHECK(decodedGeometry.chains[0].revisions.size() == 1);
        if (decodedGeometry.chains[0].revisions.size() == 1) {
            CHECK(decodedGeometry.chains[0].revisions[0].op == 15);
        }
    }
    // 10 and 11 stay reserved and 19 is past the last op. 16, a plugin
    // mover, decodes since minor 3; the runtime then requires its entry in
    // the ExternalMovers section (TestExternalMoversWire).
    for (const uint8_t op : {10, 11, 19}) {
        geometry.chains[0].revisions[0].op = op;
        bytes.clear();
        CHECK(RigExecWireEncodeDomainGeometry(geometry, &bytes));
        RigExecWireReader unknownOp(bytes.data(), bytes.size());
        CHECK(!RigExecWireDecodeDomainGeometry(
            &unknownOp, &decodedGeometry, &error));
    }
    geometry.chains[0].revisions[0].op = RigExecWireExternalRevisionOp;
    bytes.clear();
    CHECK(RigExecWireEncodeDomainGeometry(geometry, &bytes));
    RigExecWireReader pluginOp(bytes.data(), bytes.size());
    CHECK(RigExecWireDecodeDomainGeometry(&pluginOp, &decodedGeometry,
                                          &error));
}

// The ExternalMovers section: a strict round trip, frame entries that name
// no blob refused on both sides, and no trailing bytes.
static void
TestExternalMoversWire()
{
    RigExecWireExternalMovers movers;
    RigExecWireExternalRevision revision;
    revision.chain = 2;
    revision.revision = 1;
    revision.type = 7;
    revision.epoch = {'T', 'A', 'G', '1', 0, 255};
    movers.revisions.push_back(revision);
    movers.blobs = {{1, 2, 3}, {}};
    movers.frames = {{0}, {1}, {RigExecWireExternalNoFrame}, {0}};
    std::vector<uint8_t> bytes;
    CHECK(RigExecWireEncodeExternalMovers(movers, &bytes));
    RigExecWireReader cursor(bytes.data(), bytes.size());
    RigExecWireExternalMovers decoded;
    std::string error;
    CHECK(RigExecWireDecodeExternalMovers(&cursor, &decoded, &error));
    CHECK(decoded.revisions.size() == 1);
    if (decoded.revisions.size() == 1) {
        CHECK(decoded.revisions[0].chain == 2);
        CHECK(decoded.revisions[0].revision == 1);
        CHECK(decoded.revisions[0].type == 7);
        CHECK(decoded.revisions[0].epoch == revision.epoch);
    }
    CHECK(decoded.blobs == movers.blobs);
    CHECK(decoded.frames == movers.frames);

    RigExecWireExternalMovers dangling = movers;
    dangling.frames[1][0] = 2;
    bytes.clear();
    CHECK(!RigExecWireEncodeExternalMovers(dangling, &bytes));

    bytes.clear();
    CHECK(RigExecWireEncodeExternalMovers(movers, &bytes));
    // The last frame entry, rewritten to name a blob that does not exist.
    bytes[bytes.size() - 4] = 9;
    RigExecWireReader corrupt(bytes.data(), bytes.size());
    CHECK(!RigExecWireDecodeExternalMovers(&corrupt, &decoded, &error));

    bytes.clear();
    CHECK(RigExecWireEncodeExternalMovers(movers, &bytes));
    bytes.push_back(0);
    RigExecWireReader trailing(bytes.data(), bytes.size());
    CHECK(!RigExecWireDecodeExternalMovers(&trailing, &decoded, &error));
    bytes.resize(bytes.size() - 2);
    RigExecWireReader truncated(bytes.data(), bytes.size());
    CHECK(!RigExecWireDecodeExternalMovers(&truncated, &decoded, &error));
}

// The Computed section (temporary): a hand-built table survives
// a round trip byte for byte, and every truncation, oversized count and
// malformed index is refused on both sides.
static void
TestComputedWire()
{
    using v4::InputTag;
    const auto floatValue = [](float f) {
        v4::RigExecWireValue value;
        value.tag = InputTag::Float;
        uint32_t bits = 0;
        std::memcpy(&bits, &f, sizeof(f));
        value.bits = bits;
        return value;
    };
    RigExecWireComputed table;
    table.bakeTime = 1001.0;
    table.values.emplace_back();
    table.values.push_back(floatValue(0.5f));
    {
        // -0.0 keeps its sign: the bits are stored, not the number.
        v4::RigExecWireValue negativeZero;
        const double d = -0.0;
        std::memcpy(&negativeZero.bits, &d, sizeof(d));
        table.values.push_back(negativeZero);
        v4::RigExecWireValue matrix;
        matrix.tag = InputTag::Matrix4d;
        matrix.matrix = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 4, 5, 6, 1};
        table.values.push_back(matrix);
        v4::RigExecWireValue vec;
        vec.tag = InputTag::Vec3f;
        vec.vec3f = {1.0f, -2.0f, 3.0f};
        table.values.push_back(vec);
    }
    table.vec3fArrays.resize(2);
    table.vec3fArrays[1].v = {{1, 2, 3}, {4, 5, 6}};
    v4::InputSlot avar;
    avar.name = 1;
    avar.value = 2;
    avar.type = InputTag::Double;
    avar.flags = uint8_t(v4::InputSlotFlags::Listed) |
                 uint8_t(v4::InputSlotFlags::Animated) |
                 uint8_t(v4::InputSlotFlags::HasValue);
    v4::InputSlot weight;
    weight.name = 2;
    weight.value = 1;
    weight.type = InputTag::Float;
    weight.flags = uint8_t(v4::InputSlotFlags::Listed) |
                   uint8_t(v4::InputSlotFlags::HasValue);
    table.inputs = {avar, weight};
    table.listedInputs = 2;
    v4::RigExecWireWeightObject stepBacked;
    stepBacked.path = 3;
    stepBacked.type = 4;
    stepBacked.values = {0.25f};
    stepBacked.defaultWeight.tag = InputTag::Float;
    stepBacked.defaultWeight.flags = uint8_t(v4::InputReadFlags::Varying);
    stepBacked.defaultWeight.overrideIndex = 0;
    stepBacked.defaultWeight.constant = 1;
    stepBacked.defaultWeight.walk = {1};
    stepBacked.defaultWeight.selected = 0;
    v4::RigExecWireWeightObject envelope;
    envelope.envelopeOnly = true;
    envelope.path = 5;
    envelope.base = 0;
    envelope.driver.tag = InputTag::Float;
    envelope.driver.mode = v4::ReadMode::Resolved;
    envelope.driver.flags = uint8_t(v4::InputReadFlags::Varying);
    envelope.driver.constant = 1;
    envelope.driver.walk = {1, 0};
    envelope.oracleSamples = 1;
    envelope.oracleStaticError = "unknown rangePolicy on /W";
    stepBacked.oraclePlaneAxis = 3;
    stepBacked.oraclePlaneBounds = 4;
    table.weightObjects = {stepBacked, envelope};
    table.constraintWeightObjectIndex = {-1, 1};
    RigExecWireComputedFrame frame;
    frame.frame = 1001.0;
    frame.values = {2, 1};
    frame.hasValue = {1, 0};
    table.frames = {frame, frame};
    table.frames[1].frame = 1002.0;

    std::vector<uint8_t> bytes;
    std::string error;
    CHECK(RigExecWireEncodeComputed(table, &bytes, &error));
    RigExecWireReader cursor(bytes.data(), bytes.size());
    RigExecWireComputed decoded;
    CHECK(RigExecWireDecodeComputed(&cursor, &decoded, &error));
    std::vector<uint8_t> again;
    CHECK(RigExecWireEncodeComputed(decoded, &again, &error));
    CHECK(again == bytes);
    CHECK(decoded.values.size() == table.values.size());
    if (decoded.values.size() == table.values.size()) {
        CHECK(decoded.values[2].bits == table.values[2].bits);
        CHECK(decoded.values[3].matrix == table.values[3].matrix);
        CHECK(decoded.values[4].vec3f == table.values[4].vec3f);
    }
    CHECK(decoded.weightObjects.size() == 2);
    if (decoded.weightObjects.size() == 2) {
        CHECK(decoded.weightObjects[1].driver.walk ==
              std::vector<uint32_t>({1, 0}));
        CHECK(decoded.weightObjects[1].oracleStaticError ==
              "unknown rangePolicy on /W");
        CHECK(decoded.weightObjects[0].defaultWeight.selected == 0);
        CHECK(decoded.weightObjects[0].oraclePlaneAxis == 3 &&
              decoded.weightObjects[0].oraclePlaneBounds == 4);
    }
    CHECK(decoded.frames.size() == 2 && decoded.frames[1].frame == 1002.0);

    // Every strict prefix is refused, and so is a trailing byte.
    for (size_t size = 0; size < bytes.size(); ++size) {
        RigExecWireReader prefix(bytes.data(), size);
        RigExecWireComputed partial;
        CHECK(!RigExecWireDecodeComputed(&prefix, &partial, &error));
    }
    std::vector<uint8_t> trailing = bytes;
    trailing.push_back(0);
    RigExecWireReader trailingCursor(trailing.data(), trailing.size());
    CHECK(!RigExecWireDecodeComputed(&trailingCursor, &decoded, &error));
    // A value count no payload could hold is refused before anything is
    // sized (layout u32, bake time f64, then the count).
    std::vector<uint8_t> huge = bytes;
    for (size_t i = 12; i < 16; ++i) {
        huge[i] = 0xff;
    }
    RigExecWireReader hugeCursor(huge.data(), huge.size());
    CHECK(!RigExecWireDecodeComputed(&hugeCursor, &decoded, &error));

    // Malformed tables are refused at encode, as the decoder would.
    const auto refused = [&](RigExecWireComputed bad) {
        std::vector<uint8_t> out;
        std::string why;
        return !RigExecWireEncodeComputed(bad, &out, &why) && !why.empty();
    };
    RigExecWireComputed bad = table;
    bad.weightObjects[1].driver.walk = {2};
    CHECK(refused(bad));
    bad = table;
    bad.weightObjects[1].base = 1;
    CHECK(refused(bad));
    bad = table;
    std::swap(bad.weightObjects[0], bad.weightObjects[1]);
    bad.weightObjects[0].base = -1;
    CHECK(refused(bad));
    bad = table;
    bad.weightObjects[0].defaultWeight.selected = 1;
    CHECK(refused(bad));
    bad = table;
    bad.weightObjects[1].driver.selected = 0;
    CHECK(refused(bad));
    bad = table;
    bad.weightObjects[0].defaultWeight.constant = 2;
    CHECK(refused(bad));
    bad = table;
    bad.weightObjects[1].oracleSamples = 2;
    CHECK(refused(bad));
    bad = table;
    bad.constraintWeightObjectIndex = {2};
    CHECK(refused(bad));
    bad = table;
    bad.frames[0].values = {1, 1};
    CHECK(refused(bad));
    bad = table;
    bad.frames[0].hasValue.pop_back();
    CHECK(refused(bad));
    bad = table;
    bad.listedInputs = 1;
    CHECK(refused(bad));
    bad = table;
    bad.values[0] = floatValue(0.0f);
    CHECK(refused(bad));

    // A property chain on a third slot, read by a phased consumer on a
    // fourth: the tables survive the trip, and every broken back reference,
    // tag, mode or index is refused.
    RigExecWireComputed chained = table;
    {
        v4::RigExecWireValue enabled;
        enabled.tag = InputTag::Bool;
        enabled.bits = 1;
        chained.values.push_back(enabled);  // values[5]
        v4::InputSlot target;
        target.name = 6;
        target.value = 1;
        target.type = InputTag::Float;
        target.flags = uint8_t(v4::InputSlotFlags::Listed) |
                       uint8_t(v4::InputSlotFlags::HasValue);
        target.chain = 0;
        v4::InputSlot consumer;
        consumer.name = 7;
        consumer.value = 2;
        consumer.type = InputTag::Double;
        consumer.flags = uint8_t(v4::InputSlotFlags::Listed);
        consumer.phased = 0;
        chained.inputs.push_back(target);
        chained.inputs.push_back(consumer);
        chained.listedInputs = 4;
        for (RigExecWireComputedFrame &f : chained.frames) {
            f.values.push_back(1);
            f.values.push_back(2);
            f.hasValue.push_back(1);
            f.hasValue.push_back(0);
        }
        v4::PropertyRevision revision;
        revision.mover = 8;
        revision.op = v4::PropertyOp::Curve;
        revision.envelope = 1;
        revision.enabled.tag = InputTag::Bool;
        revision.enabled.mode = v4::ReadMode::Pinned;
        revision.enabled.constant = 5;
        for (v4::RigExecWireInput *read :
             {&revision.defaultWeight, &revision.value, &revision.min,
              &revision.max}) {
            read->tag = InputTag::Float;
            read->mode = v4::ReadMode::Pinned;
            read->constant = 1;
        }
        revision.defaultWeight.walk = {1};
        revision.value.mode = v4::ReadMode::Resolved;
        revision.value.flags = uint8_t(v4::InputReadFlags::Varying);
        revision.value.walk = {1, 0};
        revision.keys = {{0.0f, 0.0f}, {1.0f, -0.0f}};
        revision.hasTangentsAttr = true;
        v4::PropertyChain chain;
        chain.target = 2;
        chain.valueType = v4::PropertyValueType::Float;
        chain.revisions = {revision};
        chained.propertyChains = {chain};
        v4::PhasedConsumer phased;
        phased.chain = 0;
        phased.consumer = 3;
        phased.consumerType = v4::PropertyValueType::Double;
        phased.applied = 1;
        phased.hops = {3, 1};
        chained.phasedConsumers = {phased};
    }
    bytes.clear();
    CHECK(RigExecWireEncodeComputed(chained, &bytes, &error));
    RigExecWireReader chainedCursor(bytes.data(), bytes.size());
    RigExecWireComputed chainedBack;
    CHECK(RigExecWireDecodeComputed(&chainedCursor, &chainedBack, &error));
    again.clear();
    CHECK(RigExecWireEncodeComputed(chainedBack, &again, &error));
    CHECK(again == bytes);
    CHECK(chainedBack.propertyChains.size() == 1 &&
          chainedBack.phasedConsumers.size() == 1);
    if (chainedBack.propertyChains.size() == 1 &&
        chainedBack.propertyChains[0].revisions.size() == 1) {
        const v4::PropertyRevision &back =
            chainedBack.propertyChains[0].revisions[0];
        CHECK(back.op == v4::PropertyOp::Curve && back.envelope == 1 &&
              back.hasTangentsAttr && back.tangents.empty());
        CHECK(back.value.walk == std::vector<uint32_t>({1, 0}) &&
              back.value.mode == v4::ReadMode::Resolved);
        // -0.0f keeps its sign.
        CHECK(back.keys.size() == 2 && std::signbit(back.keys[1][1]));
        CHECK(chainedBack.inputs[2].chain == 0 &&
              chainedBack.inputs[3].phased == 0);
        CHECK(chainedBack.phasedConsumers[0].applied == 1);
        CHECK(chainedBack.phasedConsumers[0].hops ==
              std::vector<uint32_t>({3, 1}));
    }
    for (size_t size = 0; size < bytes.size(); ++size) {
        RigExecWireReader prefix(bytes.data(), size);
        RigExecWireComputed partial;
        CHECK(!RigExecWireDecodeComputed(&prefix, &partial, &error));
    }
    bad = chained;
    bad.inputs[2].chain = -1;
    CHECK(refused(bad));
    bad = chained;
    bad.inputs[3].phased = -1;
    CHECK(refused(bad));
    bad = chained;
    bad.propertyChains[0].target = 9;
    CHECK(refused(bad));
    bad = chained;
    bad.phasedConsumers[0].applied = 2;
    CHECK(refused(bad));
    bad = chained;
    bad.phasedConsumers[0].consumerType = v4::PropertyValueType::Vec3f;
    CHECK(refused(bad));
    // Hops start at the consumer and stop short of the target.
    bad = chained;
    bad.phasedConsumers[0].hops.clear();
    CHECK(refused(bad));
    bad = chained;
    bad.phasedConsumers[0].hops = {1, 3};
    CHECK(refused(bad));
    bad = chained;
    bad.phasedConsumers[0].hops = {3, 2};
    CHECK(refused(bad));
    bad = chained;
    bad.phasedConsumers[0].hops = {3, 9};
    CHECK(refused(bad));
    bad = chained;
    bad.propertyChains[0].revisions[0].value.tag = InputTag::Vec3f;
    bad.propertyChains[0].revisions[0].value.constant = 4;
    CHECK(refused(bad));
    bad = chained;
    bad.propertyChains[0].revisions[0].min.walk = {1, 0};
    CHECK(refused(bad));
    bad = chained;
    bad.propertyChains[0].revisions[0].defaultWeight.mode =
        v4::ReadMode::Baked;
    CHECK(refused(bad));
    bad = chained;
    bad.propertyChains[0].revisions[0].envelope = 2;
    CHECK(refused(bad));
    bad = chained;
    bad.propertyChains[0].revisions[0].op = v4::PropertyOp(7);
    CHECK(refused(bad));
    bad = chained;
    bad.propertyChains[0].revisions[0].enabled.tag = InputTag::Float;
    bad.propertyChains[0].revisions[0].enabled.constant = 1;
    CHECK(refused(bad));

    // The pose tables say whether the rig has property chains; a section
    // that disagrees is refused when it is applied.
    RigExecWireDomainGeometry geometry;
    geometry.weightObjects.resize(1);
    geometry.weightObjects[0].path = 3;
    geometry.weightObjects[0].type = 4;
    RigExecWireInputTable records;
    records.frames.resize(2);
    records.frames[0].frame = 1001.0;
    records.frames[1].frame = 1002.0;
    RigExecWireDomainPose pose;
    pose.constraints.resize(2);
    pose.hasPropertyChains = true;
    CHECK(RigExecWireApplyComputed(chained, geometry, records, &pose, &error));
    CHECK(pose.constraints[1].weightObjectIndex == 1);
    CHECK(!RigExecWireApplyComputed(table, geometry, records, &pose, &error));
    pose.hasPropertyChains = false;
    CHECK(RigExecWireApplyComputed(table, geometry, records, &pose, &error));
    CHECK(!RigExecWireApplyComputed(chained, geometry, records, &pose,
                                    &error));

    // A registered read whose walk crosses the chain's target, recorded
    // under uid 1: it survives the trip, and a read that is not Baked, not
    // long-way through a chain, of a type no record holds, pinned, or out
    // of uid order is refused; so is one whose directory entry differs.
    RigExecWireComputed crossing = chained;
    {
        RigExecWireChainRead entry;
        entry.uid = 1;
        entry.read.tag = InputTag::Float;
        entry.read.mode = v4::ReadMode::Baked;
        entry.read.flags = uint8_t(v4::InputReadFlags::Varying) |
                           uint8_t(v4::InputReadFlags::LongWay) |
                           uint8_t(v4::InputReadFlags::ViaChain);
        entry.read.overrideIndex = 4;
        entry.read.constant = 1;
        entry.read.walk = {1, 2};
        crossing.chainReads = {entry};
    }
    bytes.clear();
    CHECK(RigExecWireEncodeComputed(crossing, &bytes, &error));
    RigExecWireReader crossingCursor(bytes.data(), bytes.size());
    RigExecWireComputed crossingBack;
    CHECK(RigExecWireDecodeComputed(&crossingCursor, &crossingBack, &error));
    again.clear();
    CHECK(RigExecWireEncodeComputed(crossingBack, &again, &error));
    CHECK(again == bytes);
    CHECK(crossingBack.chainReads.size() == 1);
    if (crossingBack.chainReads.size() == 1) {
        const RigExecWireChainRead &back = crossingBack.chainReads[0];
        CHECK(back.uid == 1 && back.read.overrideIndex == 4 &&
              back.read.walk == std::vector<uint32_t>({1, 2}) &&
              back.read.flags == crossing.chainReads[0].read.flags);
    }
    for (size_t size = 0; size < bytes.size(); ++size) {
        RigExecWireReader prefix(bytes.data(), size);
        RigExecWireComputed partial;
        CHECK(!RigExecWireDecodeComputed(&prefix, &partial, &error));
    }
    bad = crossing;
    bad.chainReads[0].read.mode = v4::ReadMode::Resolved;
    bad.chainReads[0].read.overrideIndex = -1;
    bad.chainReads[0].read.flags = uint8_t(v4::InputReadFlags::Varying) |
                                   uint8_t(v4::InputReadFlags::ViaChain);
    CHECK(refused(bad));
    bad = crossing;
    bad.chainReads[0].read.flags = uint8_t(v4::InputReadFlags::Varying) |
                                   uint8_t(v4::InputReadFlags::LongWay);
    CHECK(refused(bad));
    bad = crossing;
    bad.chainReads[0].read.flags = uint8_t(v4::InputReadFlags::Varying) |
                                   uint8_t(v4::InputReadFlags::ViaChain);
    CHECK(refused(bad));
    bad = crossing;
    bad.chainReads[0].read.walk = {1};
    CHECK(refused(bad));
    bad = crossing;
    bad.chainReads[0].read.tag = InputTag::Vec3f;
    bad.chainReads[0].read.constant = 4;
    CHECK(refused(bad));
    bad = crossing;
    bad.chainReads[0].read.selected = 0;
    CHECK(refused(bad));
    bad = crossing;
    bad.chainReads.push_back(bad.chainReads[0]);
    CHECK(refused(bad));

    RigExecWireInputTable directory = records;
    directory.directory.resize(2);
    directory.directory[1].tag = RigExecWireInput::Tag::Float;
    directory.directory[1].overrideIndex = 4;
    directory.directory[1].head = 2;  // the name of walk[0]
    pose.hasPropertyChains = true;
    CHECK(RigExecWireApplyComputed(crossing, geometry, directory, &pose,
                                   &error));
    RigExecWireInputTable wrong = directory;
    wrong.directory.resize(1);
    CHECK(!RigExecWireApplyComputed(crossing, geometry, wrong, &pose, &error));
    wrong = directory;
    wrong.directory[1].tag = RigExecWireInput::Tag::Double;
    CHECK(!RigExecWireApplyComputed(crossing, geometry, wrong, &pose, &error));
    wrong = directory;
    wrong.directory[1].overrideIndex = 3;
    CHECK(!RigExecWireApplyComputed(crossing, geometry, wrong, &pose, &error));
    wrong = directory;
    wrong.directory[1].head = 1;
    CHECK(!RigExecWireApplyComputed(crossing, geometry, wrong, &pose, &error));

    // The read tables: a registered solver read recorded under uid 1 and an
    // avar binding's (Baked), a blend channel's weight and its sample's
    // activation, a revision's default weight and a forced float mover
    // scalar recorded as a double that reads its head when the walk yields
    // nothing (Resolved). They survive the trip; a registered read in
    // another mode or with an avar index off the avar families, a geometry
    // read that is not a Resolved Float, a path read naming no path,
    // widening a non-float or headed by another attribute than its path,
    // and a second read of one channel, revision or path are refused at
    // encode; a read naming no row, channel, revision or matching directory
    // entry, or reading another number of activations than its channel has
    // samples, is refused when applied, and a pose-driven channel's read is
    // accepted.
    RigExecWireComputed reads = crossing;
    {
        RigExecWireRegisteredRead solverRead;
        solverRead.family = RigExecWireRegisteredFamily::Solver;
        solverRead.field = 3;
        solverRead.uid = 1;
        solverRead.read = crossing.chainReads[0].read;
        RigExecWireRegisteredRead avarRead;
        avarRead.family = RigExecWireRegisteredFamily::AvarBinding;
        avarRead.avar = 22;
        avarRead.read.tag = InputTag::Double;
        avarRead.read.constant = 2;
        avarRead.read.walk = {0};
        reads.registeredReads = {solverRead, avarRead};
        v4::RigExecWireInput weightRead;
        weightRead.tag = InputTag::Float;
        weightRead.mode = v4::ReadMode::Resolved;
        weightRead.constant = 1;
        weightRead.walk = {1};
        RigExecWireBlendWeightRead blend;
        blend.read = weightRead;
        blend.activations = {weightRead};
        reads.blendWeightReads = {blend};
        RigExecWireDefaultWeightRead defaultWeight;
        defaultWeight.read = weightRead;
        reads.defaultWeightReads = {defaultWeight};
        RigExecWirePathScalarRead path;
        path.path = 2;
        path.widen = true;
        path.headFallback = true;
        path.read = weightRead;
        reads.pathScalarReads = {path};
    }
    bytes.clear();
    CHECK(RigExecWireEncodeComputed(reads, &bytes, &error));
    RigExecWireReader readsCursor(bytes.data(), bytes.size());
    RigExecWireComputed readsBack;
    CHECK(RigExecWireDecodeComputed(&readsCursor, &readsBack, &error));
    again.clear();
    CHECK(RigExecWireEncodeComputed(readsBack, &again, &error));
    CHECK(again == bytes);
    CHECK(readsBack.registeredReads.size() == 2 &&
          readsBack.blendWeightReads.size() == 1 &&
          readsBack.blendWeightReads[0].activations.size() == 1 &&
          readsBack.defaultWeightReads.size() == 1 &&
          readsBack.pathScalarReads.size() == 1);
    if (readsBack.registeredReads.size() == 2 &&
        readsBack.pathScalarReads.size() == 1) {
        CHECK(readsBack.registeredReads[0].family ==
                  RigExecWireRegisteredFamily::Solver &&
              readsBack.registeredReads[0].field == 3 &&
              readsBack.registeredReads[0].uid == 1 &&
              readsBack.registeredReads[0].avar == -1);
        CHECK(readsBack.registeredReads[1].avar == 22 &&
              readsBack.registeredReads[1].uid == -1);
        CHECK(readsBack.pathScalarReads[0].widen &&
              readsBack.pathScalarReads[0].headFallback &&
              readsBack.pathScalarReads[0].path == 2);
    }
    for (size_t size = 0; size < bytes.size(); ++size) {
        RigExecWireReader prefix(bytes.data(), size);
        RigExecWireComputed partial;
        CHECK(!RigExecWireDecodeComputed(&prefix, &partial, &error));
    }
    bad = reads;
    bad.registeredReads[0].read.mode = v4::ReadMode::Resolved;
    bad.registeredReads[0].read.overrideIndex = -1;
    bad.registeredReads[0].read.flags = 0;
    CHECK(refused(bad));
    bad = reads;
    bad.registeredReads[0].avar = 3;
    CHECK(refused(bad));
    bad = reads;
    bad.registeredReads[1].avar = -1;
    CHECK(refused(bad));
    bad = reads;
    bad.registeredReads[1].field = 1;
    CHECK(refused(bad));
    bad = reads;
    bad.blendWeightReads[0].read.mode = v4::ReadMode::Pinned;
    CHECK(refused(bad));
    bad = reads;
    bad.blendWeightReads[0].activations[0].mode = v4::ReadMode::Pinned;
    CHECK(refused(bad));
    bad = reads;
    bad.defaultWeightReads[0].read.tag = InputTag::Double;
    bad.defaultWeightReads[0].read.constant = 2;
    CHECK(refused(bad));
    bad = reads;
    bad.pathScalarReads[0].path = 0;
    CHECK(refused(bad));
    bad = reads;
    bad.pathScalarReads[0].read.tag = InputTag::Double;
    bad.pathScalarReads[0].read.constant = 2;
    CHECK(refused(bad));
    bad = reads;
    bad.pathScalarReads[0].read.walk = {0};
    CHECK(refused(bad));
    bad = reads;
    bad.pathScalarReads[0].read.walk.clear();
    CHECK(refused(bad));
    bad = reads;
    bad.pathScalarReads.push_back(bad.pathScalarReads[0]);
    bad.pathScalarReads[1].widen = false;
    CHECK(refused(bad));
    bad = reads;
    bad.defaultWeightReads.push_back(bad.defaultWeightReads[0]);
    CHECK(refused(bad));
    bad = reads;
    bad.blendWeightReads.push_back(bad.blendWeightReads[0]);
    CHECK(refused(bad));

    RigExecWireDomainGeometry blended = geometry;
    blended.chains.resize(1);
    blended.chains[0].revisions.resize(1);
    blended.chains[0].revisions[0].blendChannels.resize(1);
    blended.chains[0].revisions[0].blendChannels[0].samples.resize(1);
    pose.solvers.resize(1);
    CHECK(RigExecWireApplyComputed(reads, blended, directory, &pose, &error));
    bad = reads;
    bad.registeredReads[0].object = 1;
    CHECK(!RigExecWireApplyComputed(bad, blended, directory, &pose, &error));
    wrong = directory;
    wrong.directory[1].overrideIndex = 3;
    bad = reads;
    bad.chainReads.clear();
    CHECK(!RigExecWireApplyComputed(bad, blended, wrong, &pose, &error));
    bad = reads;
    bad.blendWeightReads[0].channel = 1;
    CHECK(!RigExecWireApplyComputed(bad, blended, directory, &pose, &error));
    bad = reads;
    bad.blendWeightReads[0].derived = true;
    CHECK(!RigExecWireApplyComputed(bad, blended, directory, &pose, &error));
    bad = reads;
    bad.blendWeightReads[0].activations.clear();
    CHECK(!RigExecWireApplyComputed(bad, blended, directory, &pose, &error));
    CHECK(error.find("activations") != std::string::npos);
    bad = reads;
    bad.defaultWeightReads[0].revision = 1;
    CHECK(!RigExecWireApplyComputed(bad, blended, directory, &pose, &error));
    RigExecWireDomainGeometry driven = blended;
    driven.chains[0].revisions[0].blendChannels[0].poseWeight = 0;
    CHECK(RigExecWireApplyComputed(reads, driven, directory, &pose, &error));
}

// The DomainPose section's trailing solver-space block: a solver's
// spaceMatrix, space slot, space rest and space read survive the trip; a
// payload that ends before the block decodes as the identity and no
// space; a payload that ends inside it, a spaceMatrix that is not a
// Matrix4d and a space slot below -1 are refused.
static void
TestSolverSpaceWire()
{
    RigExecWireDomainPose pose;
    pose.solvers.resize(1);
    RigExecWireSolver &solver = pose.solvers[0];
    solver.ikSpace.matrix = {1.25, 0, 0, 0, 0, 0.5, 0, 0,
                             0, 0, 2, 0, 0.5, -1, 3, 1};
    solver.ikSpace.varying = true;
    solver.ikSpace.bound = true;
    solver.ikSpace.overrideIndex = 2;
    solver.ikSpace.head = 5;
    solver.spaceSlot = 3;
    solver.spaceRest = {{{1, 2, 3}, {4, 5, 6}, {7, 8, 9}, {10, 11, 12}}};
    solver.spaceRead = 7;
    std::vector<uint8_t> bytes;
    std::string error;
    CHECK(RigExecWireEncodeDomainPose(pose, &bytes));
    {
        RigExecWireReader cursor(bytes.data(), bytes.size());
        RigExecWireDomainPose decoded;
        CHECK(RigExecWireDecodeDomainPose(&cursor, &decoded, &error));
        CHECK(decoded.solvers.size() == 1);
        if (decoded.solvers.size() == 1) {
            const RigExecWireSolver &got = decoded.solvers[0];
            CHECK(got.ikSpace.tag == RigExecWireInput::Tag::Matrix4d &&
                  got.ikSpace.matrix == solver.ikSpace.matrix &&
                  got.ikSpace.varying && got.ikSpace.bound &&
                  got.ikSpace.overrideIndex == 2 && got.ikSpace.head == 5);
            CHECK(got.spaceSlot == 3 && got.spaceRest == solver.spaceRest &&
                  got.spaceRead == 7);
        }
    }
    // Tag, matrix, flags, override number and head; then the slot, the
    // four rest landmarks and the read.
    const size_t block = (1 + 128 + 1 + 4 + 4) + 4 + 4 * 24 + 4;
    CHECK(bytes.size() > block);
    if (bytes.size() <= block) {
        return;
    }
    const size_t start = bytes.size() - block;
    CHECK(bytes[start] == uint8_t(RigExecWireInput::Tag::Matrix4d));
    {
        RigExecWireReader cursor(bytes.data(), start);
        RigExecWireDomainPose older;
        CHECK(RigExecWireDecodeDomainPose(&cursor, &older, &error));
        const RigExecWireInput identity = RigExecWireIdentityMatrixInput();
        CHECK(older.solvers.size() == 1);
        if (older.solvers.size() == 1) {
            const RigExecWireSolver &got = older.solvers[0];
            CHECK(got.ikSpace.tag == RigExecWireInput::Tag::Matrix4d &&
                  got.ikSpace.matrix == identity.matrix &&
                  !got.ikSpace.varying && !got.ikSpace.bound &&
                  got.ikSpace.overrideIndex == -1);
            CHECK(got.spaceSlot == -1 && got.spaceRead == 0 &&
                  got.spaceRest == (std::array<RigExecWireVec3d, 4>{}));
        }
    }
    for (size_t size = start + 1; size < bytes.size(); ++size) {
        RigExecWireReader prefix(bytes.data(), size);
        RigExecWireDomainPose partial;
        CHECK(!RigExecWireDecodeDomainPose(&prefix, &partial, &error));
    }
    std::vector<uint8_t> retagged = bytes;
    retagged[start] = uint8_t(RigExecWireInput::Tag::Double);
    {
        RigExecWireReader cursor(retagged.data(), retagged.size());
        RigExecWireDomainPose decoded;
        CHECK(!RigExecWireDecodeDomainPose(&cursor, &decoded, &error));
    }
    // A well-formed Double spaceMatrix, written by the encoder: only the
    // tag check refuses it.
    RigExecWireDomainPose doubled = pose;
    doubled.solvers[0].ikSpace = RigExecWireInput();
    std::vector<uint8_t> doubledBytes;
    CHECK(RigExecWireEncodeDomainPose(doubled, &doubledBytes));
    {
        RigExecWireReader cursor(doubledBytes.data(), doubledBytes.size());
        RigExecWireDomainPose decoded;
        CHECK(!RigExecWireDecodeDomainPose(&cursor, &decoded, &error));
    }
    std::vector<uint8_t> below = bytes;
    const int32_t slot = -2;
    std::memcpy(&below[start + 138], &slot, sizeof(slot));
    {
        RigExecWireReader cursor(below.data(), below.size());
        RigExecWireDomainPose decoded;
        CHECK(!RigExecWireDecodeDomainPose(&cursor, &decoded, &error));
    }
    std::vector<uint8_t> none = bytes;
    const int32_t noSlot = -1;
    std::memcpy(&none[start + 138], &noSlot, sizeof(noSlot));
    {
        RigExecWireReader cursor(none.data(), none.size());
        RigExecWireDomainPose decoded;
        CHECK(RigExecWireDecodeDomainPose(&cursor, &decoded, &error));
        CHECK(decoded.solvers.size() == 1 &&
              decoded.solvers[0].spaceSlot == -1);
    }
}

static void
TestContainerRoundTrip()
{
    RigExecBinaryWriter writer;
    CHECK(writer.AddString("") == 0);
    const uint32_t a = writer.AddString("/World/Rig/Joint");
    const uint32_t b = writer.AddString("/World/Rig/Joint");
    CHECK(a == b);
    CHECK(a != 0);
    const uint32_t c = writer.AddString("/World/Rig/Other");
    CHECK(c != a);
    const std::vector<uint8_t> manifest = {'{', '}'};
    writer.AddSection(RigExecBinarySection::Manifest, manifest);
    // An unknown tag is a well-formed file: the reader skips what it does
    // not know, which is the whole minor-version story.
    const std::vector<uint8_t> future = {1, 2, 3, 4};
    writer.AddSection(RigExecBinarySection(0x100), future);
    const std::vector<uint8_t> bytes = writer.Finish();

    std::string error;
    std::unique_ptr<RigExecBinaryReader> reader =
        RigExecBinaryReader::Open(bytes.data(), bytes.size(), &error);
    CHECK(reader);
    if (!reader) {
        std::printf("open diagnostic: %s\n", error.c_str());
        return;
    }
    CHECK(reader->GetVersion() == RigExecBinaryVersion);
    const uint8_t *data = nullptr;
    size_t size = 0;
    CHECK(reader->FindSection(RigExecBinarySection::Manifest, &data, &size));
    CHECK(size == manifest.size() && data[0] == '{' && data[1] == '}');
    CHECK(reader->FindSection(RigExecBinarySection(0x100), &data, &size));
    CHECK(size == future.size());
    CHECK(!reader->FindSection(RigExecBinarySection::Steps, &data, &size));
    std::string text;
    CHECK(reader->GetString(0, &text) && text.empty());
    CHECK(reader->GetString(a, &text) && text == "/World/Rig/Joint");
    CHECK(reader->GetString(c, &text) && text == "/World/Rig/Other");
    CHECK(!reader->GetString(c + 1, &text));
}

static void
TestContainerRejections()
{
    RigExecBinaryWriter writer;
    writer.AddString("/World/Rig");
    writer.AddSection(RigExecBinarySection::Manifest,
                      std::vector<uint8_t>{'{', '}'});
    const std::vector<uint8_t> good = writer.Finish();
    std::string error;

    // Every corruption below is rejected with a reason; none of them may
    // open, read out of bounds, or throw.
    std::vector<uint8_t> bad = good;
    bad[0] = 'X';
    CHECK(!RigExecBinaryReader::Open(bad.data(), bad.size(), &error));
    CHECK(error.find("magic") != std::string::npos);

    CHECK(!RigExecBinaryReader::Open(good.data(), 3, &error));

    for (const uint8_t legacyMajor : {1, 2}) {
        bad = good;
        bad[4] = legacyMajor;
        CHECK(!RigExecBinaryReader::Open(bad.data(), bad.size(), &error));
        CHECK(error.find("version") != std::string::npos);
    }

    // A section count the buffer cannot hold.
    bad = good;
    bad[8] = 64;
    CHECK(!RigExecBinaryReader::Open(bad.data(), bad.size(), &error));

    // A section running past the end (first entry's offset -> huge).
    bad = good;
    bad[16 + 4] = 0xff;
    bad[16 + 5] = 0xff;
    CHECK(!RigExecBinaryReader::Open(bad.data(), bad.size(), &error));

    // An unterminated final string.
    bad = good;
    // The string table is the FIRST section, so its last byte is read out
    // of the section table, not off the end of the file (which is the
    // manifest payload).
    {
        const size_t tableOffset =
            size_t(bad[20]) | (size_t(bad[21]) << 8) |
            (size_t(bad[22]) << 16) | (size_t(bad[23]) << 24) |
            (size_t(bad[24]) << 32) | (size_t(bad[25]) << 40) |
            (size_t(bad[26]) << 48) | (size_t(bad[27]) << 56);
        const size_t tableSize =
            size_t(bad[28]) | (size_t(bad[29]) << 8) |
            (size_t(bad[30]) << 16) | (size_t(bad[31]) << 24) |
            (size_t(bad[32]) << 32) | (size_t(bad[33]) << 40) |
            (size_t(bad[34]) << 48) | (size_t(bad[35]) << 56);
        bad[tableOffset + tableSize - 1] = 'x';
    }
    CHECK(!RigExecBinaryReader::Open(bad.data(), bad.size(), &error));
}

static SdfPath
FindRig(const UsdStageRefPtr &stage)
{
    for (const UsdPrim &prim : stage->TraverseAll()) {
        if (prim.GetTypeName() == TfToken("RigExecRoot")) {
            return prim.GetPath();
        }
    }
    return SdfPath();
}

/// Whether a fixture's capture is expected to vary across the baked
/// frames. Probed from pose outputs (body hash, frame header excluded):
/// the animated stages move over their table frames (several peak
/// mid-range with static endpoints, so endpoints alone mislabel
/// them); the static ones never move. Unknown stages skip the guard.
enum class _BinaryVariance {
    Unknown,
    Static,
    Animated,
};

static _BinaryVariance
_BinaryExpectedVariance(const std::string &fixture)
{
    const std::string name =
        std::filesystem::path(fixture).filename().string();
    static const char *animated[] = {
        "01_FkChainTail.usda", "02_TwoBoneIkLeg.usda",
        "03_IkFkBlendClamp.usda", "04_BlendShapeFace.usda",
        "05_TwistRibbonSpine.usda", "06_LatticeBulge.usda",
        "07_SurfaceDrape.usda", "08_AimEyes.usda",
        "09_PropertyMathMovers.usda", "10_AimXformTurret.usda",
        "11_VolumeWeights.usda",
        "13_ReadPhases.usda", "14_VolumeConstrainedSweep.usda",
        "16_ConnectionReadPhases.usda",
        "aimtest.usda", "aimtest_points.usda",
        "rotateConstraint.usda", "rigexec_flat.usda",
        "par_rot_aim.usd", "par_rot_aim_redorder.usd",
        "rot_par_combo.usd", "aim_par_combo_flattened.usd",
        "ArmShotAnim.usda", "simple_rig_anim.usd", "Biped_anim.usda",
    };
    static const char *statics[] = {
        "ArmRig.usda", "spider_leg.usd", "spider_leg_ik.usd",
        "simple_rig.usd", "spider_legs_assembly_ref.usda",
        "Biped.usda", "Biped_body.usda", "Biped_stack.usda",
    };
    for (const char *known : animated) {
        if (name == known) {
            return _BinaryVariance::Animated;
        }
    }
    for (const char *known : statics) {
        if (name == known) {
            return _BinaryVariance::Static;
        }
    }
    return _BinaryVariance::Unknown;
}

static std::vector<double>
_ParseTableFrames(const std::string &text)
{
    std::vector<double> frames;
    size_t begin = 0;
    while (begin <= text.size()) {
        const size_t end = text.find(",", begin);
        const std::string piece =
            text.substr(begin, end == std::string::npos
                        ? std::string::npos : end - begin);
        if (!piece.empty()) {
            frames.push_back(std::stod(piece));
        }
        if (end == std::string::npos) {
            break;
        }
        begin = end + 1;
    }
    return frames;
}

// The weight-gather capture hole (M2): the volume gather target and
// driver-curve arrays must reach the frame record. Fixture 11 TipCurve
// is an UNMOVED driver no chain base covers, so without the recorder
// hook in the gather it is unrepresentable and the runtime builds the
// wrong field. Filename-keyed like _BinaryExpectedVariance.
static void
_BinaryCheckWeightGatherReads(const std::string &fixture,
                              const std::set<std::string> &seenPathReads)
{
    const std::string base =
        std::filesystem::path(fixture).filename().string();
    std::vector<std::string> want;
    if (base == "11_VolumeWeights.usda") {
        want = {"/VolumeAsset/Drivers/TipCurve.points",
                "/VolumeAsset/Geom/Strip.points"};
    } else if (base == "14_VolumeConstrainedSweep.usda") {
        want = {"/SweepAsset/Geom/Strip.points"};
    }
    for (const std::string &path : want) {
        if (!seenPathReads.count(path)) {
            std::printf("weight-gather read missing from %s: %s\n",
                        base.c_str(), path.c_str());
        }
        CHECK(seenPathReads.count(path));
    }
}
// The property chains over section 18 alone, as a reference for what the
// section holds: runChain (rigEvaluatorProperties.cpp) with _PinnedRead and
// GetAttribute over the slot values of one frame, publishing into an
// overlay keyed by the slot names (the record's property path ids, both
// interned by the same writer). Envelopes come from \p envelope; a chain
// with an envelope and no resolver is left unevaluated and reported.
using _ChainOverlay = std::map<uint32_t, RigExecWirePropertyValue>;
using _ChainEnvelope = std::function<bool(
    int32_t object, const RigExecWireComputedFrame &frame,
    const _ChainOverlay &overlay, float *weight, std::string *error)>;

static bool
_ChainOverlayHit(const _ChainOverlay &overlay, uint32_t name,
                 v4::InputTag tag, v4::RigExecWireValue *out)
{
    const auto found = overlay.find(name);
    if (found == overlay.end()) {
        return false;
    }
    using Held = RigExecWirePropertyValue::Tag;
    const RigExecWirePropertyValue &held = found->second;
    v4::RigExecWireValue value;
    value.tag = tag;
    if (tag == v4::InputTag::Float && held.tag == Held::Float) {
        uint32_t bits = 0;
        std::memcpy(&bits, &held.f32, sizeof(bits));
        value.bits = bits;
    } else if (tag == v4::InputTag::Double && held.tag == Held::Double) {
        std::memcpy(&value.bits, &held.f64, sizeof(value.bits));
    } else if (tag == v4::InputTag::Matrix4d && held.tag == Held::Matrix4d) {
        value.matrix = held.matrix;
    } else if (tag == v4::InputTag::Vec3f && held.tag == Held::Vec3f) {
        value.vec3f = held.vec;
    } else {
        return false;
    }
    *out = value;
    return true;
}

// GetAttribute<T> from walk[from]: the head's double test for a float read,
// then per hop the overlay and the double test, then the most upstream
// readable hop.
static bool
_ChainLongWay(const RigExecWireComputed &computed,
              const RigExecWireComputedFrame &frame,
              const _ChainOverlay &overlay, const std::vector<uint32_t> &walk,
              size_t from, v4::InputTag tag, v4::RigExecWireValue *out)
{
    const auto narrow = [&](size_t k) {
        v4::RigExecWireValue wide;
        if (!_ChainLongWay(computed, frame, overlay, walk, k,
                           v4::InputTag::Double, &wide)) {
            return false;
        }
        double d = 0.0;
        std::memcpy(&d, &wide.bits, sizeof(d));
        const float f = static_cast<float>(d);
        uint32_t bits = 0;
        std::memcpy(&bits, &f, sizeof(bits));
        *out = v4::RigExecWireValue();
        out->tag = v4::InputTag::Float;
        out->bits = bits;
        return true;
    };
    if (tag == v4::InputTag::Float && from < walk.size() &&
        computed.inputs[walk[from]].type == v4::InputTag::Double) {
        return narrow(from);
    }
    for (size_t k = from; k < walk.size(); ++k) {
        const v4::InputSlot &slot = computed.inputs[walk[k]];
        if (_ChainOverlayHit(overlay, slot.name, tag, out)) {
            return true;
        }
        if (tag == v4::InputTag::Float && slot.type == v4::InputTag::Double) {
            return narrow(k);
        }
    }
    for (size_t k = walk.size(); k-- > from;) {
        const uint32_t slot = walk[k];
        if (frame.hasValue[slot] && computed.inputs[slot].type == tag) {
            *out = computed.values[frame.values[slot]];
            return true;
        }
    }
    return false;
}

// _PinnedRead (Pinned: the attribute's own overlay, then its own typed
// value) or GetAttribute (Resolved), falling back to the read's constant.
static v4::RigExecWireValue
_ChainRead(const RigExecWireComputed &computed,
           const RigExecWireComputedFrame &frame,
           const _ChainOverlay &overlay, const v4::RigExecWireInput &input)
{
    v4::RigExecWireValue value = computed.values[input.constant];
    if (input.mode == v4::ReadMode::Resolved) {
        _ChainLongWay(computed, frame, overlay, input.walk, 0, input.tag,
                      &value);
        return value;
    }
    CHECK(input.mode == v4::ReadMode::Pinned && input.walk.size() <= 1);
    if (input.walk.empty()) {
        return value;
    }
    const uint32_t slot = input.walk[0];
    if (_ChainOverlayHit(overlay, computed.inputs[slot].name, input.tag,
                         &value)) {
        return value;
    }
    if (frame.hasValue[slot] && computed.inputs[slot].type == input.tag) {
        value = computed.values[frame.values[slot]];
    }
    return value;
}

static float
_ChainFloat(const v4::RigExecWireValue &value)
{
    const uint32_t bits = uint32_t(value.bits);
    float f = 0.0f;
    std::memcpy(&f, &bits, sizeof(f));
    return f;
}

static GfVec3f
_ChainVec3f(const v4::RigExecWireValue &value)
{
    return GfVec3f(value.vec3f[0], value.vec3f[1], value.vec3f[2]);
}

static GfMatrix4d
_ChainMatrix(const RigExecWireMatrix4d &m)
{
    GfMatrix4d out;
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            out[r][c] = m[size_t(r * 4 + c)];
        }
    }
    return out;
}

static RigExecWireMatrix4d
_ChainWireMatrix(const GfMatrix4d &m)
{
    RigExecWireMatrix4d out{};
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            out[size_t(r * 4 + c)] = m[r][c];
        }
    }
    return out;
}

static bool
_ChainFinite(const RigExecWirePropertyValue &value)
{
    using Tag = RigExecWirePropertyValue::Tag;
    switch (value.tag) {
    case Tag::Float:
        return std::isfinite(value.f32);
    case Tag::Double:
        // _IsFinite has no double overload: a double goes through float.
        return std::isfinite(float(value.f64));
    case Tag::Matrix4d:
        for (double d : value.matrix) {
            if (!std::isfinite(d)) {
                return false;
            }
        }
        return true;
    case Tag::Vec3f:
        return std::isfinite(value.vec[0]) && std::isfinite(value.vec[1]) &&
               std::isfinite(value.vec[2]);
    }
    return false;
}

static bool
_ChainSameBits(const RigExecWirePropertyValue &a,
               const RigExecWirePropertyValue &b)
{
    using Tag = RigExecWirePropertyValue::Tag;
    if (a.tag != b.tag) {
        return false;
    }
    switch (a.tag) {
    case Tag::Float:
        return std::memcmp(&a.f32, &b.f32, sizeof(float)) == 0;
    case Tag::Double:
        return std::memcmp(&a.f64, &b.f64, sizeof(double)) == 0;
    case Tag::Matrix4d:
        return std::memcmp(a.matrix.data(), b.matrix.data(),
                           sizeof(double) * 16) == 0;
    case Tag::Vec3f:
        return std::memcmp(a.vec.data(), b.vec.data(), sizeof(float) * 3) ==
               0;
    }
    return false;
}

/// Runs every chain at \p frame into \p overlay (cleared first) and
/// appends the chains' diagnostic lines. False when a chain needs an
/// envelope and \p envelope is empty: that chain and everything after it
/// are left out.
static bool
_ComputedRunChains(const RigExecWireComputed &computed,
                   const RigExecWireComputedFrame &frame,
                   const RigExecBinaryReader &reader,
                   const _ChainEnvelope &envelope, _ChainOverlay *overlay,
                   std::vector<std::string> *diagnostics)
{
    using Tag = RigExecWirePropertyValue::Tag;
    overlay->clear();
    const auto text = [&](uint32_t id) {
        std::string s;
        reader.GetString(id, &s);
        return s;
    };
    for (size_t c = 0; c < computed.propertyChains.size(); ++c) {
        const v4::PropertyChain &chain = computed.propertyChains[c];
        const v4::InputSlot &target = computed.inputs[chain.target];
        const std::string targetText = text(target.name);
        v4::InputTag baseTag = v4::InputTag::Float;
        Tag tag = Tag::Float;
        switch (chain.valueType) {
        case v4::PropertyValueType::Double:
            baseTag = v4::InputTag::Double;
            tag = Tag::Double;
            break;
        case v4::PropertyValueType::Matrix4d:
            baseTag = v4::InputTag::Matrix4d;
            tag = Tag::Matrix4d;
            break;
        case v4::PropertyValueType::Vec3f:
            baseTag = v4::InputTag::Vec3f;
            tag = Tag::Vec3f;
            break;
        default:
            break;
        }
        if (!frame.hasValue[chain.target] || target.type != baseTag) {
            diagnostics->push_back("property chain " + targetText +
                                   ": target has no authored value; chain "
                                   "skipped");
            continue;
        }
        const v4::RigExecWireValue &base =
            computed.values[frame.values[chain.target]];
        RigExecWirePropertyValue value;
        value.tag = tag;
        switch (tag) {
        case Tag::Float:
            value.f32 = _ChainFloat(base);
            break;
        case Tag::Double:
            std::memcpy(&value.f64, &base.bits, sizeof(double));
            break;
        case Tag::Matrix4d:
            value.matrix = base.matrix;
            break;
        case Tag::Vec3f:
            value.vec = base.vec3f;
            break;
        }
        if (!_ChainFinite(value)) {
            diagnostics->push_back("property chain " + targetText +
                                   ": authored base is not finite; chain "
                                   "skipped");
            continue;
        }
        std::vector<size_t> phased;
        for (size_t k = 0; k < computed.phasedConsumers.size(); ++k) {
            if (computed.phasedConsumers[k].chain == c) {
                phased.push_back(k);
            }
        }
        std::vector<RigExecWirePropertyValue> history;
        for (const v4::PropertyRevision &revision : chain.revisions) {
            if (!phased.empty()) {
                history.push_back(value);
            }
            const std::string mover = text(revision.mover);
            if (_ChainRead(computed, frame, *overlay, revision.enabled).bits ==
                0) {
                diagnostics->push_back("diag " + mover +
                                       ": disabled; revision passed through");
                continue;
            }
            float weight = 1.0f;
            if (revision.envelope >= 0) {
                if (!envelope) {
                    return false;
                }
                std::string error;
                if (!envelope(revision.envelope, frame, *overlay, &weight,
                              &error)) {
                    diagnostics->push_back("diag " + mover + ": " + error +
                                           "; revision passed through");
                    continue;
                }
            } else {
                weight = _ChainFloat(_ChainRead(computed, frame, *overlay,
                                                revision.defaultWeight));
                if (!std::isfinite(weight) || weight < 0.0f ||
                    weight > 1.0f) {
                    diagnostics->push_back(
                        "diag " + mover +
                        ": inputs:defaultWeight must be finite and in "
                        "[0, 1]; revision passed through");
                    continue;
                }
            }
            const bool validOp = revision.op != v4::PropertyOp::Invalid;
            const RigExecPropertyOp op = RigExecPropertyOp(revision.op);
            RigExecWirePropertyValue next = value;
            bool usable = validOp;
            if (usable && (tag == Tag::Float || tag == Tag::Double)) {
                RigExecPropertyMathParams<float> params;
                params.op = op;
                params.value = _ChainFloat(
                    _ChainRead(computed, frame, *overlay, revision.value));
                params.min = _ChainFloat(
                    _ChainRead(computed, frame, *overlay, revision.min));
                params.max = _ChainFloat(
                    _ChainRead(computed, frame, *overlay, revision.max));
                usable = std::isfinite(params.value) &&
                         std::isfinite(params.min) &&
                         std::isfinite(params.max);
                std::vector<GfVec2f> keys, tangents;
                if (usable && op == RigExecPropertyOp::Curve) {
                    for (const RigExecWireVec2f &k : revision.keys) {
                        keys.emplace_back(k[0], k[1]);
                    }
                    for (const RigExecWireVec2f &t : revision.tangents) {
                        tangents.emplace_back(t[0], t[1]);
                    }
                    usable = !keys.empty() &&
                             RigExecValidateLinearKeys(keys.data(),
                                                       keys.size()) &&
                             (tangents.empty() ||
                              tangents.size() == keys.size());
                    params.keys = keys.data();
                    params.keyCount = keys.size();
                    if (!tangents.empty()) {
                        params.tangents = tangents.data();
                        params.tangentCount = tangents.size();
                    }
                }
                params.weight = weight;
                if (usable && tag == Tag::Float) {
                    next.f32 = RigExecApplyFloatMath(value.f32, params);
                } else if (usable) {
                    next.f64 = double(
                        RigExecApplyFloatMath(float(value.f64), params));
                }
            } else if (usable && tag == Tag::Vec3f) {
                RigExecPropertyMathParams<GfVec3f> params;
                params.op = op;
                params.value = _ChainVec3f(
                    _ChainRead(computed, frame, *overlay, revision.value));
                params.min = _ChainVec3f(
                    _ChainRead(computed, frame, *overlay, revision.min));
                params.max = _ChainVec3f(
                    _ChainRead(computed, frame, *overlay, revision.max));
                for (int i = 0; i < 3 && usable; ++i) {
                    usable = std::isfinite(params.value[i]) &&
                             std::isfinite(params.min[i]) &&
                             std::isfinite(params.max[i]);
                }
                params.weight = weight;
                if (usable) {
                    const GfVec3f r = RigExecApplyVec3fMath(
                        GfVec3f(value.vec[0], value.vec[1], value.vec[2]),
                        params);
                    next.vec = RigExecWireVec3f{r[0], r[1], r[2]};
                }
            } else if (usable) {
                const GfMatrix4d operand = _ChainMatrix(
                    _ChainRead(computed, frame, *overlay, revision.value)
                        .matrix);
                RigExecWirePropertyValue probe;
                probe.tag = Tag::Matrix4d;
                probe.matrix = _ChainWireMatrix(operand);
                GfMatrix4d out = _ChainMatrix(value.matrix);
                usable = _ChainFinite(probe) &&
                         RigExecApplyMatrixMath(_ChainMatrix(value.matrix),
                                                op, operand, weight, &out);
                next.matrix = _ChainWireMatrix(out);
            }
            if (!usable) {
                diagnostics->push_back("diag " + mover +
                                       ": inputs unusable; revision passed "
                                       "through");
                continue;
            }
            if (!_ChainFinite(next)) {
                diagnostics->push_back("diag " + mover +
                                       ": produced a non-finite value; "
                                       "revision passed through");
                continue;
            }
            value = next;
        }
        (*overlay)[target.name] = value;
        if (phased.empty()) {
            continue;
        }
        history.push_back(value);
        for (size_t k : phased) {
            const v4::PhasedConsumer &consumer = computed.phasedConsumers[k];
            RigExecWirePropertyValue v =
                history[std::min(size_t(consumer.applied), history.size() - 1)];
            if (consumer.consumerType == v4::PropertyValueType::Double &&
                v.tag == Tag::Float) {
                v.tag = Tag::Double;
                v.f64 = double(v.f32);
                v.f32 = 0;
            } else if (consumer.consumerType == v4::PropertyValueType::Float &&
                       v.tag == Tag::Double) {
                v.tag = Tag::Float;
                v.f32 = float(v.f64);
                v.f64 = 0;
            }
            (*overlay)[computed.inputs[consumer.consumer].name] = v;
        }
    }
    return true;
}

/// A chain-crossing read's value against the frame record's, bit for bit
/// on the member the tag names.
static bool
_ChainReadSameBits(const v4::RigExecWireValue &computed,
                   const RigExecWireValue &recorded)
{
    using Tag = RigExecWireInput::Tag;
    if (uint8_t(computed.tag) != uint8_t(recorded.tag)) {
        return false;
    }
    switch (recorded.tag) {
    case Tag::Double:
        return std::memcmp(&computed.bits, &recorded.f64, sizeof(double)) ==
               0;
    case Tag::Float: {
        const uint32_t bits = uint32_t(computed.bits);
        return std::memcmp(&bits, &recorded.f32, sizeof(float)) == 0;
    }
    case Tag::Bool:
        return (computed.bits != 0) == recorded.boolean;
    case Tag::Int:
        return int32_t(uint32_t(computed.bits)) == recorded.i32;
    case Tag::Matrix4d:
        return std::memcmp(computed.matrix.data(), recorded.matrix.data(),
                           sizeof(double) * 16) == 0;
    case Tag::Token:
        return uint32_t(computed.bits) == recorded.token;
    case Tag::Vec3d:
        return std::memcmp(computed.vec3d.data(), recorded.vec.data(),
                           sizeof(double) * 3) == 0;
    }
    return false;
}

/// The section's chains against the live program and, frame by frame,
/// the reference above against the recorded property values, bit for bit,
/// and each chain-crossing registered read (RigExecBakedRead the long way:
/// GetAttribute over its walk with the chains' results, else its constant)
/// against the value the record holds for its uid, where it holds one.
/// Returns the number of property values compared; *skipped counts frames
/// left out for want of an envelope resolver; *chainReads, when given,
/// receives the number of reads compared.
static size_t
_BinaryCheckChains(const RigExecBakedProgramImpl &program,
                   const RigExecWireComputed &computed,
                   const RigExecWireInputTable &table,
                   const RigExecBinaryReader &reader,
                   const _ChainEnvelope &envelope, size_t *skipped,
                   std::vector<std::vector<std::string>> *diagnostics,
                   size_t *chainReads = nullptr)
{
    if (chainReads) {
        *chainReads = 0;
    }
    CHECK(program.hasPropertyChains == !computed.propertyChains.empty());
    std::set<std::string> targets;
    for (const v4::PropertyChain &chain : computed.propertyChains) {
        std::string name;
        CHECK(reader.GetString(computed.inputs[chain.target].name, &name));
        targets.insert(name);
    }
    std::set<std::string> programTargets;
    for (const SdfPath &path : program.chainTargets) {
        programTargets.insert(path.GetString());
    }
    CHECK(targets == programTargets);
    size_t compared = 0;
    *skipped = 0;
    for (size_t f = 0; f < table.frames.size() && f < computed.frames.size();
         ++f) {
        const RigExecWireFrameInputs &record = table.frames[f];
        _ChainOverlay overlay;
        std::vector<std::string> lines;
        if (!_ComputedRunChains(computed, computed.frames[f], reader,
                                envelope, &overlay, &lines)) {
            ++*skipped;
            continue;
        }
        if (diagnostics) {
            diagnostics->push_back(lines);
        }
        CHECK(record.propertyPaths.size() == overlay.size());
        for (size_t i = 0; i < record.propertyPaths.size() &&
                           i < record.propertyValues.size();
             ++i) {
            const auto found = overlay.find(record.propertyPaths[i]);
            std::string name;
            reader.GetString(record.propertyPaths[i], &name);
            if (found == overlay.end()) {
                std::printf("chain reference publishes nothing at %s, frame "
                            "%g\n",
                            name.c_str(), record.frame);
                CHECK(found != overlay.end());
                continue;
            }
            const bool same =
                _ChainSameBits(found->second, record.propertyValues[i]);
            if (!same) {
                std::printf("chain reference differs at %s, frame %g: "
                            "computed %.9g/%.17g, recorded %.9g/%.17g\n",
                            name.c_str(), record.frame,
                            double(found->second.f32), found->second.f64,
                            double(record.propertyValues[i].f32),
                            record.propertyValues[i].f64);
            }
            CHECK(same);
            ++compared;
        }
        for (const RigExecWireChainRead &entry : computed.chainReads) {
            const auto at = std::lower_bound(record.uids.begin(),
                                             record.uids.end(), entry.uid);
            if (at == record.uids.end() || *at != entry.uid) {
                continue;
            }
            v4::RigExecWireValue value = computed.values[entry.read.constant];
            _ChainLongWay(computed, computed.frames[f], overlay,
                          entry.read.walk, 0, entry.read.tag, &value);
            const bool same = _ChainReadSameBits(
                value, record.values[size_t(at - record.uids.begin())]);
            if (!same) {
                std::string head;
                reader.GetString(
                    computed.inputs[entry.read.walk[0]].name, &head);
                std::printf("chain read reference differs at uid %u (%s), "
                            "frame %g\n",
                            entry.uid, head.c_str(), record.frame);
            }
            CHECK(same);
            if (chainReads) {
                ++*chainReads;
            }
        }
    }
    return compared;
}

/// Section 18 of a bake, decoded and checked against the sections it
/// extends and against the live program it came from: one step-backed
/// entry per program weight object, one envelope index per constraint
/// exactly where the constraint step resolves an envelope, one slot record
/// per baked frame, slots ordered by path text.
static bool
_BinaryCheckComputed(const RigExecBakedProgramImpl &program,
                     const std::vector<uint8_t> &bytes,
                     const std::vector<double> &frames,
                     RigExecWireComputed *computed,
                     RigExecWireInputTable *table,
                     std::unique_ptr<RigExecBinaryReader> *readerOut)
{
    std::string error;
    std::unique_ptr<RigExecBinaryReader> reader =
        RigExecBinaryReader::Open(bytes.data(), bytes.size(), &error);
    CHECK(reader);
    if (!reader) {
        return false;
    }
    const auto decode = [&](RigExecBinarySection tag, auto decodeFn,
                            auto *out) {
        const uint8_t *data = nullptr;
        size_t size = 0;
        if (!reader->FindSection(tag, &data, &size)) {
            return false;
        }
        RigExecWireReader cursor(data, size);
        return decodeFn(&cursor, out, &error);
    };
    RigExecWireDomainPose pose;
    RigExecWireDomainGeometry geometry;
    CHECK(decode(RigExecBinarySection::DomainPose,
                 RigExecWireDecodeDomainPose, &pose));
    CHECK(decode(RigExecBinarySection::DomainGeometry,
                 RigExecWireDecodeDomainGeometry, &geometry));
    CHECK(decode(RigExecBinarySection::InputTable,
                 RigExecWireDecodeInputTable, table));
    const bool decoded = decode(RigExecBinarySection::Computed,
                                RigExecWireDecodeComputed, computed);
    if (!decoded) {
        std::printf("computed section diagnostic: %s\n", error.c_str());
    }
    CHECK(decoded);
    if (!decoded) {
        return false;
    }
    const bool applied =
        RigExecWireApplyComputed(*computed, geometry, *table, &pose, &error);
    if (!applied) {
        std::printf("computed section diagnostic: %s\n", error.c_str());
    }
    CHECK(applied);
    CHECK(computed->frames.size() == frames.size());
    size_t stepBacked = 0;
    for (const v4::RigExecWireWeightObject &object : computed->weightObjects) {
        stepBacked += object.envelopeOnly ? 0 : 1;
    }
    CHECK(stepBacked == program.weightObjects.size());
    {
        const uint8_t *data = nullptr;
        size_t size = 0;
        reader->FindSection(RigExecBinarySection::Computed, &data, &size);
        size_t staticErrors = 0, inFlight = 0;
        for (const v4::RigExecWireWeightObject &object :
             computed->weightObjects) {
            staticErrors += object.oracleStaticError.empty() ? 0 : 1;
            inFlight += object.samplesInFlight ? 1 : 0;
        }
        std::printf("  computed section: %zu bytes of %zu, %zu slots, %zu "
                    "weight objects (%zu envelope-only, %zu in flight, %zu "
                    "with a static oracle error)\n",
                    size, bytes.size(), computed->inputs.size(),
                    computed->weightObjects.size(),
                    computed->weightObjects.size() - stepBacked, inFlight,
                    staticErrors);
    }
    CHECK(computed->constraintWeightObjectIndex.size() ==
          program.constraints.size());
    for (size_t k = 0; k < program.constraints.size() &&
                       k < computed->constraintWeightObjectIndex.size() &&
                       k < pose.constraints.size();
         ++k) {
        const RigExecBakedProgramImpl::Constraint &c = program.constraints[k];
        const int32_t index = computed->constraintWeightObjectIndex[k];
        const bool envelope =
            !c.weightObject.IsEmpty() && c.pointsTarget.IsEmpty();
        CHECK(envelope == (index >= 0));
        CHECK(pose.constraints[k].weightObjectIndex == index);
        if (index >= 0) {
            std::string path;
            CHECK(reader->GetString(
                computed->weightObjects[size_t(index)].path, &path));
            CHECK(path == c.weightObject.GetString());
        }
    }
    std::string previous;
    for (size_t s = 0; s < computed->inputs.size(); ++s) {
        std::string name;
        CHECK(reader->GetString(computed->inputs[s].name, &name));
        CHECK(s == 0 || previous < name);
        previous = name;
    }
    // One chain read per program input that reads the long way through a
    // chain target, each naming its own directory entry (the apply above
    // checked tag, override number and head).
    size_t crossing = 0;
    frozenDetail::_ForEachPatchableInput(program, [&](const auto &input) {
        if (!input.resolvedAttr) {
            return;
        }
        bool viaChain = false, varying = false;
        UsdAttribute selected;
        RigExecBakedClassifyInput<float>(input.resolvedAttr,
                                         UsdTimeCode::Default(),
                                         program.chainTargets, &viaChain,
                                         &varying, &selected);
        crossing += viaChain ? 1 : 0;
    });
    CHECK(crossing == computed->chainReads.size());
    // One registered read per input the frozen context's walk visits, each
    // naming the uid the frame records gave it (the apply above checked
    // tag, override number and head against the directory).
    size_t registered = 0;
    frozenDetail::_ForEachPatchableInput(program, [&](const auto &) {
        ++registered;
    });
    CHECK(registered == computed->registeredReads.size());
    size_t withUid = 0;
    for (const RigExecWireRegisteredRead &entry : computed->registeredReads) {
        withUid += entry.uid >= 0 ? 1 : 0;
    }
    std::printf("  computed reads: %zu registered (%zu with a uid), %zu "
                "blend weight(s), %zu default weight(s), %zu forced mover "
                "scalar(s)\n",
                computed->registeredReads.size(), withUid,
                computed->blendWeightReads.size(),
                computed->defaultWeightReads.size(),
                computed->pathScalarReads.size());
    // The chains as the section states them reproduce every recorded
    // property value and chain-crossing read (frames whose chains need an
    // envelope are left to the tests that resolve one).
    size_t skipped = 0, chainReads = 0;
    const size_t chainValues =
        _BinaryCheckChains(program, *computed, *table, *reader, nullptr,
                           &skipped, nullptr, &chainReads);
    if (!computed->propertyChains.empty()) {
        std::printf("  computed chains: %zu chain(s), %zu phased "
                    "consumer(s), %zu recorded value(s) reproduced bit for "
                    "bit, %zu chain read(s) and %zu recorded read value(s) "
                    "reproduced, %zu frame(s) left to an envelope resolver\n",
                    computed->propertyChains.size(),
                    computed->phasedConsumers.size(), chainValues,
                    computed->chainReads.size(), chainReads, skipped);
    }
    *readerOut = std::move(reader);
    return true;
}

/// RigExecResolvedInputs::GetAttribute over the section's slot values at
/// one frame, with no property-chain result published: the reference
/// the section is checked against here.
static bool
_ComputedLongWay(const RigExecWireComputed &computed,
                 const RigExecWireComputedFrame &frame,
                 const std::vector<uint32_t> &walk, size_t from,
                 v4::InputTag tag, v4::RigExecWireValue *out)
{
    for (size_t k = from; k < walk.size(); ++k) {
        if (tag == v4::InputTag::Float &&
            computed.inputs[walk[k]].type == v4::InputTag::Double) {
            v4::RigExecWireValue wide;
            if (!_ComputedLongWay(computed, frame, walk, k,
                                  v4::InputTag::Double, &wide)) {
                return false;
            }
            double d = 0.0;
            std::memcpy(&d, &wide.bits, sizeof(d));
            const float narrow = static_cast<float>(d);
            uint32_t bits = 0;
            std::memcpy(&bits, &narrow, sizeof(narrow));
            *out = v4::RigExecWireValue();
            out->tag = v4::InputTag::Float;
            out->bits = bits;
            return true;
        }
    }
    for (size_t k = walk.size(); k-- > from;) {
        const uint32_t slot = walk[k];
        if (frame.hasValue[slot] && computed.inputs[slot].type == tag) {
            *out = computed.values[frame.values[slot]];
            return true;
        }
    }
    return false;
}

static float
_ComputedReadFloat(const RigExecWireComputed &computed,
                   const RigExecWireComputedFrame &frame,
                   const v4::RigExecWireInput &input)
{
    v4::RigExecWireValue value = computed.values[input.constant];
    _ComputedLongWay(computed, frame, input.walk, 0, v4::InputTag::Float,
                     &value);
    float f = 0.0f;
    const uint32_t bits = uint32_t(value.bits);
    std::memcpy(&f, &bits, sizeof(f));
    return f;
}

// Two transform constraints whose envelopes no mover binds: a StaticWeight
// at 0.5 and a DynamicWeight whose driver connects to an animated double.
// Both become envelope-only entries read the Resolved way, and the oracle's
// formula over the section's slot values reproduces, bit for bit, the
// envelope the program recorded at every baked frame.
static void
TestComputedEnvelopeBake()
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    const auto xform = [&](const char *path, const GfVec3d &translation) {
        const UsdGeomXform x = UsdGeomXform::Define(stage, SdfPath(path));
        GfMatrix4d m(1.0);
        m.SetTranslateOnly(translation);
        x.MakeMatrixXform().Set(m);
        return x.GetPrim();
    };
    xform("/Asset", GfVec3d(0));
    xform("/Asset/TargetA", GfVec3d(0));
    xform("/Asset/TargetB", GfVec3d(0));
    xform("/Asset/Source", GfVec3d(10, 0, 0));
    const UsdPrim dial = xform("/Asset/Dial", GfVec3d(0));
    const UsdAttribute amount = dial.CreateAttribute(
        TfToken("avars:amount"), SdfValueTypeNames->Double);
    amount.Set(0.2, UsdTimeCode(1.0));
    amount.Set(0.8, UsdTimeCode(10.0));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const auto constraint = [&](const char *name, const char *target,
                                const SdfPath &weightPath) {
        const UsdPrim prim = stage->DefinePrim(
            SdfPath(std::string("/Asset/Rig/Movers/") + name),
            TfToken("RigExecPositionConstraint"));
        CHECK(prim.ApplyAPI(TfToken("RigExecMoverAPI")));
        prim.CreateRelationship(TfToken("rigExec:moves"))
            .SetTargets({SdfPath(target)});
        prim.CreateRelationship(TfToken("rigExec:sources"))
            .SetTargets({SdfPath("/Asset/Source")});
        prim.CreateRelationship(TfToken("rigExec:weightObject"))
            .SetTargets({weightPath});
    };
    const UsdPrim fixed = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/Fixed"), TfToken("RigExecStaticWeight"));
    fixed.CreateRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({SdfPath("/Asset/TargetA")});
    fixed.CreateAttribute(TfToken("rigExec:defaultWeight"),
                          SdfValueTypeNames->Float)
        .Set(0.5f);
    const UsdPrim driven = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/Driven"), TfToken("RigExecDynamicWeight"));
    driven.CreateRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({SdfPath("/Asset/TargetB")});
    driven.CreateAttribute(TfToken("rigExec:representation"),
                           SdfValueTypeNames->Token)
        .Set(TfToken("constant"));
    driven.CreateAttribute(TfToken("inputs:driver"), SdfValueTypeNames->Float)
        .AddConnection(amount.GetPath());
    constraint("A", "/Asset/TargetA", fixed.GetPath());
    constraint("B", "/Asset/TargetB", driven.GetPath());

    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    RigExecBakeOpts opts;
    opts.frames = {1.0, 5.0, 10.0};
    RigExecBakeResult result;
    std::string error;
    const bool baked = RigExecBakeToBinary(evaluator, opts, &result, &error);
    if (!baked) {
        std::printf("envelope bake diagnostic: %s\n", error.c_str());
    }
    CHECK(baked);
    if (!baked) {
        return;
    }
    const RigExecBakedProgramImpl &program =
        evaluator.GetBakedProgram()->GetStepGraph();
    RigExecWireComputed computed;
    RigExecWireInputTable table;
    std::unique_ptr<RigExecBinaryReader> reader;
    if (!_BinaryCheckComputed(program, result.bytes, opts.frames, &computed,
                              &table, &reader)) {
        return;
    }
    CHECK(program.weightObjects.empty());
    CHECK(computed.weightObjects.size() == 2);
    CHECK(table.frames.size() == opts.frames.size());
    std::string text;
    size_t checkedEnvelopes = 0;
    for (size_t k = 0; k < program.constraints.size(); ++k) {
        const int32_t index = computed.constraintWeightObjectIndex[k];
        CHECK(index >= 0);
        if (index < 0) {
            continue;
        }
        const v4::RigExecWireWeightObject &object =
            computed.weightObjects[size_t(index)];
        CHECK(object.envelopeOnly);
        CHECK(object.oracleStaticError.empty());
        CHECK(object.oraclePhaseError.empty());
        CHECK(object.defaultWeight.mode == v4::ReadMode::Resolved);
        reader->GetString(object.type, &text);
        const bool dynamic = text == "RigExecDynamicWeight";
        if (dynamic) {
            // inputs:driver walks to the avar it is connected to.
            CHECK(object.driver.walk.size() == 2);
            if (object.driver.walk.size() == 2) {
                const v4::InputSlot &leaf =
                    computed.inputs[object.driver.walk[1]];
                reader->GetString(leaf.name, &text);
                CHECK(text == amount.GetPath().GetString());
                CHECK(leaf.type == v4::InputTag::Double);
                CHECK(leaf.flags & uint8_t(v4::InputSlotFlags::Animated));
            }
        }
        for (size_t f = 0; f < table.frames.size(); ++f) {
            const RigExecWireComputedFrame &frame = computed.frames[f];
            const RigExecWireFrameInputs &record = table.frames[f];
            CHECK(frame.frame == record.frame);
            CHECK(record.constraintHaveWeight.size() > k &&
                  record.constraintHaveWeight[k]);
            if (record.constraintWeights.size() <= k) {
                continue;
            }
            // _ResolveWeights' two scalar arms at count 1, no base.
            float expected = 0.0f;
            if (dynamic) {
                const float base = 1.0f;
                const float driver =
                    _ComputedReadFloat(computed, frame, object.driver);
                const float scale =
                    _ComputedReadFloat(computed, frame, object.scale);
                const float bias =
                    _ComputedReadFloat(computed, frame, object.bias);
                expected = (base * driver) * scale + bias;
            } else {
                expected =
                    _ComputedReadFloat(computed, frame, object.defaultWeight);
            }
            uint32_t want = 0, got = 0;
            std::memcpy(&want, &expected, sizeof(expected));
            std::memcpy(&got, &record.constraintWeights[k], sizeof(got));
            if (want != got) {
                std::printf("envelope %zu at frame %g: computed %.9g, "
                            "recorded %.9g\n",
                            k, record.frame, double(expected),
                            double(record.constraintWeights[k]));
            }
            CHECK(want == got);
            ++checkedEnvelopes;
        }
    }
    CHECK(checkedEnvelopes == 2 * opts.frames.size());
}

// A geometry-domain constraint whose sphere weight samples the points in
// flight (testRigExecVolumeWeights' ConstraintEnvelopeFixture, with the
// sphere's height keyed): the step-backed object is marked in flight with
// no static samples, the constraint resolves no scalar envelope, and the
// program recorded a dense current-phase packet for its revision at every
// frame, moving with the sphere.
static void
TestComputedCurrentPhaseBake()
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const UsdPrim mesh =
        stage->DefinePrim(SdfPath("/Asset/Geom/M"), TfToken("Points"));
    mesh.CreateAttribute(TfToken("points"), SdfValueTypeNames->Point3fArray)
        .Set(VtVec3fArray{GfVec3f(0, 0, 0), GfVec3f(0, 4, 0),
                          GfVec3f(0, 8, 0)});
    const UsdGeomXform source =
        UsdGeomXform::Define(stage, SdfPath("/Asset/PullTo"));
    source.MakeMatrixXform().Set(
        GfMatrix4d(1.0).SetTranslate(GfVec3d(6, 0, 0)));
    const UsdPrim sphere = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/Sphere"), TfToken("RigExecSphereWeight"));
    sphere.CreateAttribute(TfToken("avars:tx"), SdfValueTypeNames->Double)
        .Set(0.0);
    const UsdAttribute height =
        sphere.CreateAttribute(TfToken("avars:ty"), SdfValueTypeNames->Double);
    height.Set(0.0, UsdTimeCode(1.0));
    height.Set(8.0, UsdTimeCode(2.0));
    sphere.CreateAttribute(TfToken("avars:tz"), SdfValueTypeNames->Double)
        .Set(0.0);
    sphere.CreateRelationship(TfToken("rigExec:weightTarget"))
        .SetTargets({SdfPath("/Asset/Geom/M.points")});
    sphere.CreateAttribute(TfToken("inputs:falloffMin"),
                           SdfValueTypeNames->Float)
        .Set(0.0f);
    sphere.CreateAttribute(TfToken("inputs:falloffMax"),
                           SdfValueTypeNames->Float)
        .Set(8.0f);
    sphere.CreateAttribute(TfToken("rigExec:falloffProfile"),
                           SdfValueTypeNames->Token)
        .Set(TfToken("linear"));
    sphere.GetRelationship(TfToken("rigExec:weightTarget"))
        .SetMetadata(TfToken("rigExecReadPhase"), std::string("preceding"));
    const UsdPrim constraint = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Sweep/Pull"),
        TfToken("RigExecPositionConstraint"));
    constraint.ApplyAPI(TfToken("RigExecMoverAPI"));
    constraint.CreateRelationship(TfToken("rigExec:moves"))
        .SetTargets({SdfPath("/Asset/Geom/M.points")});
    constraint.CreateRelationship(TfToken("rigExec:sources"))
        .SetTargets({source.GetPath()});
    constraint.CreateRelationship(TfToken("rigExec:weightObject"))
        .SetTargets({sphere.GetPath()});

    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    RigExecBakeOpts opts;
    opts.frames = {1.0, 2.0};
    RigExecBakeResult result;
    std::string error;
    const bool baked = RigExecBakeToBinary(evaluator, opts, &result, &error);
    if (!baked) {
        std::printf("current-phase bake diagnostic: %s\n", error.c_str());
    }
    CHECK(baked);
    if (!baked) {
        return;
    }
    const RigExecBakedProgramImpl &program =
        evaluator.GetBakedProgram()->GetStepGraph();
    RigExecWireComputed computed;
    RigExecWireInputTable table;
    std::unique_ptr<RigExecBinaryReader> reader;
    if (!_BinaryCheckComputed(program, result.bytes, opts.frames, &computed,
                              &table, &reader)) {
        return;
    }
    for (int32_t index : computed.constraintWeightObjectIndex) {
        CHECK(index == -1);
    }
    std::string text;
    const v4::RigExecWireWeightObject *found = nullptr;
    for (const v4::RigExecWireWeightObject &object : computed.weightObjects) {
        reader->GetString(object.path, &text);
        if (text == sphere.GetPath().GetString()) {
            found = &object;
        }
    }
    CHECK(found);
    if (!found) {
        return;
    }
    CHECK(!found->envelopeOnly);
    CHECK(found->samplesInFlight);
    CHECK(found->oracleSamples == -1);
    CHECK(found->oraclePhaseError.empty());
    CHECK(found->oracleStaticError.empty());
    CHECK(found->falloffMax.mode == v4::ReadMode::Baked);
    CHECK(found->falloffMax.walk.size() == 1);
    if (found->falloffMax.walk.size() == 1) {
        reader->GetString(computed.inputs[found->falloffMax.walk[0]].name,
                          &text);
        CHECK(text == "/Asset/Rig/Weights/Sphere.inputs:falloffMax");
    }
    // The packet the program measured against the entering points.
    std::vector<std::vector<float>> fields;
    for (const RigExecWireFrameInputs &record : table.frames) {
        for (const auto &chain : record.revisionPhasePackets) {
            for (const RigExecWireWeightPacket &packet : chain) {
                if (packet.valid && packet.values.size() == 3) {
                    fields.push_back(packet.values);
                }
            }
        }
    }
    CHECK(fields.size() == opts.frames.size());
    CHECK(fields.size() < 2 || fields[0] != fields[1]);
}

// The oracle's two scalar arms for an envelope-only object at count 1
// (rigEvaluatorGeometry.cpp _ResolveWeights): a constant StaticWeight and
// a DynamicWeight with no base, reading through GetAttribute with the
// oracle's own fallbacks.
static _ChainEnvelope
_ChainScalarEnvelopes(const RigExecWireComputed &computed,
                      const RigExecBinaryReader &reader)
{
    return [&computed, &reader](int32_t index,
                                const RigExecWireComputedFrame &frame,
                                const _ChainOverlay &overlay, float *weight,
                                std::string *error) {
        const v4::RigExecWireWeightObject &object =
            computed.weightObjects[size_t(index)];
        std::string type, rangePolicy, representation, path;
        reader.GetString(object.type, &type);
        reader.GetString(object.rangePolicy, &rangePolicy);
        reader.GetString(object.representation, &representation);
        reader.GetString(object.path, &path);
        const auto read = [&](const v4::RigExecWireInput &input,
                              float fallback) {
            v4::RigExecWireValue value;
            return _ChainLongWay(computed, frame, overlay, input.walk, 0,
                                 v4::InputTag::Float, &value)
                       ? _ChainFloat(value)
                       : fallback;
        };
        CHECK(object.envelopeOnly && object.oracleStaticError.empty() &&
              object.base < 0 && object.inputs.empty() &&
              representation == "constant" && object.values.empty());
        if (type == "RigExecStaticWeight") {
            float w = read(object.defaultWeight, 0.0f);
            if (!std::isfinite(w) ||
                (rangePolicy == "strict" && (w < 0.0f || w > 1.0f))) {
                *error = "weight range violation on " + path;
                return false;
            }
            if (rangePolicy == "clamp") {
                w = std::min(std::max(w, 0.0f), 1.0f);
            }
            *weight = w;
            return true;
        }
        CHECK(type == "RigExecDynamicWeight");
        const float driver = read(object.driver, 1.0f);
        const float scale = read(object.scale, 1.0f);
        const float bias = read(object.bias, 0.0f);
        float r = (1.0f * driver) * scale + bias;
        if (!std::isfinite(r)) {
            *error = "non-finite dynamic weight on " + path;
            return false;
        }
        if (r < 0.0f || r > 1.0f) {
            if (rangePolicy == "clamp") {
                r = std::min(std::max(r, 0.0f), 1.0f);
            } else {
                *error = "strict range violation on " + path;
                return false;
            }
        }
        *weight = r;
        return true;
    };
}

// tests/fixtures/computed_chains.usda: the chain tables as the evaluator
// compiled them (order, value types, read modes and walks, envelopes,
// curve keys, phased consumers), and the reference above reproducing every
// recorded property value bit for bit and the chains' diagnostic lines at
// every baked frame.
static void
TestComputedChainBake()
{
    const std::string fixture =
        (std::filesystem::path(RIGEXEC_EXAMPLES_DIR) / ".." / "tests" /
         "fixtures" / "computed_chains.usda")
            .string();
    const UsdStageRefPtr stage = UsdStage::Open(fixture);
    CHECK(stage);
    if (!stage) {
        return;
    }
    const SdfPath rigPath("/Asset/Rig");
    RigExecRigEvaluator evaluator(stage, rigPath);
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    RigExecBakeOpts opts;
    opts.frames = {1.0, 3.0, 5.0, 7.0, 10.0};
    RigExecBakeResult result;
    std::string error;
    const bool baked = RigExecBakeToBinary(evaluator, opts, &result, &error);
    if (!baked) {
        std::printf("chain bake diagnostic: %s\n", error.c_str());
    }
    CHECK(baked);
    if (!baked) {
        return;
    }
    const RigExecBakedProgramImpl &program =
        evaluator.GetBakedProgram()->GetStepGraph();
    RigExecWireComputed computed;
    RigExecWireInputTable table;
    std::unique_ptr<RigExecBinaryReader> reader;
    if (!_BinaryCheckComputed(program, result.bytes, opts.frames, &computed,
                              &table, &reader)) {
        return;
    }
    const auto text = [&](uint32_t id) {
        std::string s;
        reader->GetString(id, &s);
        return s;
    };
    const auto slotName = [&](uint32_t slot) {
        return text(computed.inputs[slot].name);
    };
    std::map<std::string, const v4::PropertyChain *> chains;
    for (size_t c = 0; c < computed.propertyChains.size(); ++c) {
        const v4::PropertyChain &chain = computed.propertyChains[c];
        chains[slotName(chain.target)] = &chain;
        CHECK(computed.inputs[chain.target].chain == int32_t(c));
    }
    CHECK(chains.size() == 8);
    const v4::PropertyChain *dial =
        chains["/Asset/Rig/Channels/Dial.rigExec:amount"];
    const v4::PropertyChain *wide =
        chains["/Asset/Rig/Channels/Wide.rigExec:level"];
    const v4::PropertyChain *vec =
        chains["/Asset/Rig/Channels/Vec.rigExec:offset"];
    const v4::PropertyChain *space =
        chains["/Asset/Rig/Channels/Space.rigExec:local"];
    const v4::PropertyChain *missing =
        chains["/Asset/Rig/Channels/Missing.rigExec:none"];
    CHECK(dial && wide && vec && space && missing);
    if (!dial || !wide || !vec || !space || !missing) {
        return;
    }
    // A consumer chain runs after the chain it reads.
    const auto position = [&](const v4::PropertyChain *chain) {
        return chain - computed.propertyChains.data();
    };
    CHECK(position(dial) <
          position(chains["/Asset/Rig/Channels/Readouts.rigExec:early"]));
    CHECK(dial->valueType == v4::PropertyValueType::Float);
    CHECK(wide->valueType == v4::PropertyValueType::Double);
    CHECK(vec->valueType == v4::PropertyValueType::Vec3f);
    CHECK(space->valueType == v4::PropertyValueType::Matrix4d);
    CHECK((computed.inputs[missing->target].flags &
           uint8_t(v4::InputSlotFlags::HasValue)) == 0);
    const char *order[] = {"Gain", "Shape", "Fade", "Nudge",
                           "Off",  "Over",  "Limit"};
    CHECK(dial->revisions.size() == 7);
    for (size_t r = 0; r < dial->revisions.size() && r < 7; ++r) {
        CHECK(text(dial->revisions[r].mover) ==
              std::string("/Asset/Rig/Movers/Dial/") + order[r]);
    }
    if (dial->revisions.size() == 7) {
        // Gain multiplies by the control's double avar, read through its
        // connection.
        const v4::PropertyRevision &gain = dial->revisions[0];
        CHECK(gain.op == v4::PropertyOp::Multiply);
        CHECK(gain.value.mode == v4::ReadMode::Resolved &&
              gain.value.tag == v4::InputTag::Float &&
              gain.value.walk.size() == 2);
        if (gain.value.walk.size() == 2) {
            CHECK(slotName(gain.value.walk[1]) ==
                  "/Asset/Rig/Controls/Dial.avars:tx");
            CHECK(computed.inputs[gain.value.walk[1]].type ==
                  v4::InputTag::Double);
        }
        const v4::PropertyRevision &shape = dial->revisions[1];
        CHECK(shape.op == v4::PropertyOp::Curve && shape.keys.size() == 3 &&
              shape.tangents.size() == 3 && shape.hasTangentsAttr);
        if (shape.keys.size() == 3) {
            CHECK(shape.keys[1] == RigExecWireVec2f({0.5f, 0.8f}));
        }
        const auto envelopeOf = [&](const v4::PropertyRevision &revision) {
            return revision.envelope >= 0
                       ? text(computed.weightObjects[size_t(
                                                         revision.envelope)]
                                  .path)
                       : std::string();
        };
        CHECK(envelopeOf(dial->revisions[2]) == "/Asset/Rig/Weights/Half");
        CHECK(envelopeOf(dial->revisions[3]) == "/Asset/Rig/Weights/Ramp");
        CHECK(dial->revisions[0].envelope == -1);
        const v4::PropertyRevision &off = dial->revisions[4];
        CHECK(off.enabled.mode == v4::ReadMode::Pinned &&
              off.enabled.tag == v4::InputTag::Bool &&
              off.enabled.walk.size() == 1);
        // MoverAPI declares inputs:enabled, so an unauthored one still
        // reads its own schema fallback.
        const v4::RigExecWireInput &enabled = dial->revisions[0].enabled;
        CHECK(enabled.mode == v4::ReadMode::Pinned &&
              enabled.walk.size() == 1);
        if (enabled.walk.size() == 1) {
            CHECK(computed.values[computed.inputs[enabled.walk[0]].value]
                      .bits == 1);
        }
        const v4::PropertyRevision &over = dial->revisions[5];
        CHECK(over.defaultWeight.mode == v4::ReadMode::Pinned &&
              over.defaultWeight.walk.size() == 1);
        CHECK(dial->revisions[6].op == v4::PropertyOp::Clamp);
    }
    CHECK(wide->revisions.size() == 1 &&
          wide->revisions[0].value.tag == v4::InputTag::Float);
    CHECK(vec->revisions.size() == 2 &&
          vec->revisions[0].value.tag == v4::InputTag::Vec3f);
    CHECK(space->revisions.size() == 1 &&
          space->revisions[0].value.tag == v4::InputTag::Matrix4d &&
          space->revisions[0].min.walk.empty() &&
          space->revisions[0].max.walk.empty());
    // The phased consumers: the dial's base, the dial after Shape (two
    // revisions applied), and the double chain's base read as a float.
    CHECK(computed.phasedConsumers.size() == 3);
    std::map<std::string, v4::PhasedConsumer> phased;
    for (size_t k = 0; k < computed.phasedConsumers.size(); ++k) {
        const v4::PhasedConsumer &consumer = computed.phasedConsumers[k];
        phased[slotName(consumer.consumer)] = consumer;
        CHECK(computed.inputs[consumer.consumer].phased == int32_t(k));
    }
    const v4::PhasedConsumer base =
        phased["/Asset/Rig/Movers/Readouts/Base.inputs:value"];
    const v4::PhasedConsumer early =
        phased["/Asset/Rig/Movers/Readouts/Early.inputs:value"];
    const v4::PhasedConsumer narrow =
        phased["/Asset/Rig/Movers/Readouts/Narrow.inputs:value"];
    CHECK(computed.propertyChains.data() + base.chain == dial &&
          base.applied == 0 &&
          base.consumerType == v4::PropertyValueType::Float);
    CHECK(computed.propertyChains.data() + early.chain == dial &&
          early.applied == 2);
    CHECK(computed.propertyChains.data() + narrow.chain == wide &&
          narrow.applied == 0 &&
          narrow.consumerType == v4::PropertyValueType::Float);

    // Follow's weight declares `final`, so it is the one registered read
    // that crosses a chain rather than a phased consumer: its walk runs
    // from its own attribute to the dial's target.
    CHECK(computed.chainReads.size() == 1);
    if (computed.chainReads.size() == 1) {
        const RigExecWireChainRead &weight = computed.chainReads[0];
        CHECK(weight.read.tag == v4::InputTag::Float &&
              weight.read.mode == v4::ReadMode::Baked &&
              weight.read.walk.size() == 2);
        if (weight.read.walk.size() == 2) {
            CHECK(slotName(weight.read.walk[0]) ==
                  "/Asset/Rig/Movers/Follow.inputs:defaultWeight");
            const int32_t chain = computed.inputs[weight.read.walk[1]].chain;
            CHECK(chain >= 0 &&
                  computed.propertyChains.data() + chain == dial);
        }
        CHECK(weight.uid < table.directory.size() &&
              text(table.directory[weight.uid].head) ==
                  "/Asset/Rig/Movers/Follow.inputs:defaultWeight");
    }

    // Every recorded value, with the envelopes resolved, and the chains'
    // lines as the evaluator printed them first at each frame.
    size_t skipped = 0, chainReads = 0;
    std::vector<std::vector<std::string>> lines;
    const size_t compared = _BinaryCheckChains(
        program, computed, table, *reader,
        _ChainScalarEnvelopes(computed, *reader), &skipped, &lines,
        &chainReads);
    std::printf("  computed chains: %zu value(s) and %zu chain read(s) "
                "reproduced bit for bit\n",
                compared, chainReads);
    CHECK(skipped == 0);
    CHECK(compared == 10 * opts.frames.size());
    CHECK(chainReads == opts.frames.size());
    CHECK(lines.size() == opts.frames.size());
    RigExecRigEvaluator replay(stage, rigPath);
    replay.SetEvaluationMode(RigExecEvaluationMode::Baked);
    for (size_t f = 0; f < lines.size(); ++f) {
        const RigExecRigPose pose = replay.Evaluate(UsdTimeCode(opts.frames[f]));
        CHECK(lines[f].size() == 3);
        const auto at = std::search(pose.diagnostics.begin(),
                                    pose.diagnostics.end(), lines[f].begin(),
                                    lines[f].end());
        if (at == pose.diagnostics.end()) {
            for (const std::string &line : lines[f]) {
                std::printf("  reference line: %s\n", line.c_str());
            }
            for (const std::string &line : pose.diagnostics) {
                std::printf("  evaluator line: %s\n", line.c_str());
            }
        }
        CHECK(at != pose.diagnostics.end());
    }
}

static void
TestBake(const std::string &fixture, const std::vector<double> &bakeFrames,
         const std::filesystem::path &scratch)
{
    const UsdStageRefPtr stage = UsdStage::Open(fixture);
    CHECK(stage);
    if (!stage) {
        return;
    }
    const SdfPath rigPath = FindRig(stage);
    CHECK(!rigPath.IsEmpty());
    if (rigPath.IsEmpty()) {
        return;
    }
    // Two fresh evaluators, one bake each: a bake is deterministic, so the
    // bytes are identical -- which is what makes a byte golden meaningful
    // for every slice that follows.
    std::vector<uint8_t> first;
    for (int pass = 0; pass < 2; ++pass) {
        RigExecRigEvaluator evaluator(stage, rigPath);
        evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
        RigExecBakeOpts opts;
        opts.frames = bakeFrames;
        RigExecBakeResult result;
        std::string error;
        CHECK(RigExecBakeToBinary(evaluator, opts, &result, &error));
        if (error.empty()) {
            // keep the diagnostic visible when the CHECK above fails
        } else {
            std::printf("bake diagnostic: %s\n", error.c_str());
        }
        if (result.bytes.empty()) {
            return;
        }
        std::unique_ptr<RigExecBinaryReader> reader =
            RigExecBinaryReader::Open(result.bytes.data(),
                                      result.bytes.size(), &error);
        CHECK(reader);
        if (!reader) {
            return;
        }
        const uint8_t *data = nullptr;
        size_t size = 0;
        CHECK(reader->FindSection(RigExecBinarySection::Manifest, &data,
                                 &size));
        CHECK(size == result.manifestJson.size());
        CHECK(result.manifestJson.find(rigPath.GetString()) !=
              std::string::npos);
        if (pass == 0) {
            _BinaryCompareProgram(evaluator, result.bytes);
            RigExecWireComputed computed;
            RigExecWireInputTable table;
            std::unique_ptr<RigExecBinaryReader> computedReader;
            _BinaryCheckComputed(evaluator.GetBakedProgram()->GetStepGraph(),
                                 result.bytes, bakeFrames, &computed, &table,
                                 &computedReader);
        }
        if (pass == 0) {
            first = result.bytes;
        } else {
            CHECK(result.bytes == first);
        }
    }
    // Capture fidelity: drive the capture API frame by frame, encoding
    // and decoding the table per frame and comparing against the live
    // program before the next Evaluate moves it on.
    {
        RigExecRigEvaluator evaluator(stage, rigPath);
        evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
        std::vector<std::string> compileErrors;
        CHECK(evaluator.Compile(&compileErrors));
        RigExecBinaryWriter writer;
        std::string captureError;
        RigExecBakeCapture capture(evaluator, &writer, &captureError);
        CHECK(capture.Valid());
        const RigExecBakedProgramImpl &program =
            evaluator.GetBakedProgram()->GetStepGraph();
        const std::vector<_BinaryOracleInput> oracle =
            _BinaryCollectOracle(program);
        const std::vector<double> &frames = bakeFrames;
        bool recordsVary = false;
        std::string varyDiff;
        bool haveFirst = false;
        RigExecWireFrameInputs firstRecord;
        std::set<std::string> seenPathReads;
        for (double frame : frames) {
            RigExecRigPose pose;
            if (!capture.CaptureFrame(frame, &pose, &captureError)) {
                CHECK(captureError.empty());
                std::printf("capture diagnostic: %s\n",
                            captureError.c_str());
                return;
            }
            // The string table so far, as bytes: the comparisons below
            // resolve references the way every other section does.
            const std::vector<uint8_t> bytes = writer.Finish();
            std::string openError;
            std::unique_ptr<RigExecBinaryReader> reader =
                RigExecBinaryReader::Open(bytes.data(), bytes.size(),
                                          &openError);
            CHECK(reader);
            if (!reader) {
                return;
            }
            std::vector<uint8_t> payload;
            CHECK(RigExecWireEncodeInputTable(capture.GetTable(),
                                              &payload));
            RigExecWireReader cursor(payload.data(), payload.size());
            RigExecWireInputTable decoded;
            std::string decodeError;
            CHECK(RigExecWireDecodeInputTable(&cursor, &decoded,
                                              &decodeError));
            _BinaryCompareTableStatic(program, decoded, oracle, *reader);
            CHECK(decoded.frames.size() ==
                  capture.GetTable().frames.size());
            _BinaryCompareTableFrame(program, decoded.frames.back(),
                                     oracle, decoded, *reader);
            for (const RigExecWirePathRead &read :
                 decoded.frames.back().pathReads) {
                std::string path;
                if (reader->GetString(read.path, &path)) {
                    seenPathReads.insert(path);
                }
            }
            if (!haveFirst) {
                firstRecord = decoded.frames.back();
                haveFirst = true;
            } else if (!_BinaryFrameInputsEqual(firstRecord,
                                               decoded.frames.back())) {
                if (!recordsVary) {
                    varyDiff = _BinaryFrameInputsDiff(firstRecord,
                                                      decoded.frames.back());
                }
                recordsVary = true;
            }
        }
        // The varies-guard: animated fixtures must capture different
        // records across frames (or the varying-input sampling is
        // vacuous), and static fixtures must capture identical ones
        // (or the capture is nondeterministic).
        const _BinaryVariance expected =
            _BinaryExpectedVariance(fixture);
        if (expected == _BinaryVariance::Animated && !recordsVary) {
            std::printf(
                "capture is static on animated fixture %s\n",
                fixture.c_str());
        }
        CHECK(expected != _BinaryVariance::Animated || recordsVary);
        if (expected == _BinaryVariance::Static && recordsVary) {
            std::printf(
                "capture varies on static fixture %s: %s\n",
                fixture.c_str(), varyDiff.c_str());
        }
        CHECK(expected != _BinaryVariance::Static || !recordsVary);
        _BinaryCheckWeightGatherReads(fixture, seenPathReads);
    }
    // The bytes survive a trip through a file: the CLI writes exactly this
    // vector, so the vector is the format, not an in-memory sketch of it.
    std::string flat = fixture;
    for (char &c : flat) {
        if (c == '/' || c == '\\' || c == ':') {
            c = '_';
        }
    }
    const std::string file = (scratch / (flat + ".rigexec")).string();
    {
        std::ofstream stream(file, std::ios::binary);
        stream.write(reinterpret_cast<const char *>(first.data()),
                     std::streamsize(first.size()));
    }
    CHECK(Bytes(file) == first);

    // No frames is an error, not an empty binary.
    RigExecRigEvaluator evaluator(stage, rigPath);
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    RigExecBakeOpts opts;
    RigExecBakeResult result;
    std::string error;
    CHECK(!RigExecBakeToBinary(evaluator, opts, &result, &error));
    CHECK(error.find("frames") != std::string::npos);
}

int
main(int argc, char **argv)
{
    PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);
    const auto scratch =
        std::filesystem::temp_directory_path() /
        ("rigexec-binary-conformance-" +
         std::to_string(
             std::chrono::steady_clock::now().time_since_epoch().count()));
    if (!std::filesystem::create_directory(scratch)) {
        return 1;
    }
    TestContainerRoundTrip();
    TestContainerRejections();
    TestWrinkleWireExtensions();
    TestExternalMoversWire();
    TestComputedWire();
    TestSolverSpaceWire();
    TestComputedEnvelopeBake();
    TestComputedCurrentPhaseBake();
    TestComputedChainBake();
    auto BakeOne = [&](const std::string &stage,
                       const std::vector<double> &frames) {
        std::printf("bake conformance: %s\n", stage.c_str());
        TestBake(stage, frames, scratch);
    };
    if (argc > 1 && std::filesystem::is_directory(argv[1])) {
        for (const RigExecExampleFixture &fixture :
             kRigExecExampleFixtures) {
            if (!fixture.bakesToday) {
                std::printf("bake conformance: skip %s (blocked by %s)\n",
                            fixture.stage, fixture.blockedBy);
                continue;
            }
            const std::string stage =
                (std::filesystem::path(argv[1]) / fixture.stage).string();
            BakeOne(stage, _ParseTableFrames(fixture.frames));
        }
    } else if (argc > 1) {
        for (int i = 1; i < argc; ++i) {
            BakeOne(argv[i], {1001, 1024, 1048});
        }
    } else {
        for (const RigExecExampleFixture &fixture :
             kRigExecExampleFixtures) {
            if (!fixture.bakesToday) {
                continue;
            }
            const std::string stage =
                (std::filesystem::path(RIGEXEC_EXAMPLES_DIR) /
                 fixture.stage).string();
            BakeOne(stage, _ParseTableFrames(fixture.frames));
        }
    }
    if (!failures) {
        std::filesystem::remove_all(scratch);
    }
    if (failures) {
        std::printf("testRigExecBinary: %d failures; fixture %s\n", failures,
                    scratch.string().c_str());
        return 1;
    }
    std::puts("testRigExecBinary: all tests passed");
    return 0;
}
