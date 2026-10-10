// .rigexec bake conformance: the FlatBuffer file a bake writes, held to the
// program it was baked from, and the format's refusals on real bakes.
#include "rigExecBake/bake.h"
#include "rigExecBake/revisionReads.h"
#include "rigExecBake/staticReport.h"
#include "rigExec/bakedSchedule.h"
#include "rigExec/frozenContextInternal.h"
#include "rigExec/rigEvaluator.h"
#include "rigExecBinary/external.h"
#include "rigExecBinary/format.h"
#include "rigExecBinary/transport.h"
#include "rigExecBinary/generated/presentation_generated.h"
#include "rigExecExampleFixtures.h"
#include "rigExecMath/pointRanges.h"
#include "rigExecMath/propertyMath.h"
#include "rigExecRigging/rigBuilder.h"

#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/base/tf/setenv.h"
#include "pxr/base/work/threadLimits.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/editContext.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/mesh.h"
#include "pxr/usd/usdGeom/xform.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <stdexcept>
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

// After the macro: the comparison and edit helpers report through CHECK.
#include "rigExecBinaryCompare.h"
#include "rigExecFileEdit.h"

static std::vector<uint8_t>
Bytes(const std::string &path)
{
    std::ifstream stream(path, std::ios::binary);
    return std::vector<uint8_t>(std::istreambuf_iterator<char>(stream),
                                std::istreambuf_iterator<char>());
}

static bool
_Contains(const std::string &text, const std::string &part)
{
    return text.find(part) != std::string::npos;
}

/// \p path with '/' separators, so a fixture is recognised whatever
/// directory form the caller passed.
static std::string
_Slashed(const std::string &path)
{
    std::string out = path;
    std::replace(out.begin(), out.end(), '\\', '/');
    return out;
}

static bool
_EndsWith(const std::string &text, const std::string &suffix)
{
    return text.size() >= suffix.size() &&
           text.compare(text.size() - suffix.size(), suffix.size(),
                        suffix) == 0;
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

/// A bake of \p fixture's rig at \p time, or empty bytes with a failed
/// CHECK.
static std::vector<uint8_t>
_BakeFixture(const std::string &fixture, double time)
{
    const UsdStageRefPtr stage = UsdStage::Open(fixture);
    CHECK(stage);
    if (!stage) {
        return {};
    }
    RigExecRigEvaluator evaluator(stage, FindRig(stage));
    RigExecBakeOpts opts;
    opts.time = time;
    RigExecBakeResult result;
    std::string error;
    const bool baked = RigExecBakeToBinary(evaluator, opts, &result, &error);
    if (!baked) {
        std::printf("bake diagnostic: %s\n", error.c_str());
    }
    CHECK(baked);
    return result.bytes;
}

/// Write's verdict on \p file and Open's on the same file packed without
/// validation: both refuse, each naming \p part; or both accept.
static bool
_Refused(const fb::RigExecWireFile &file, const std::string &part)
{
    std::vector<uint8_t> bytes;
    std::string writeWhy, openWhy;
    const bool written = RigExecFormatWrite(file, &bytes, &writeWhy);
    std::unique_ptr<fb::RigExecWireFile> opened;
    const std::vector<uint8_t> packed = RigExecTestPackUnchecked(file);
    const bool open =
        RigExecFormatOpen(packed.data(), packed.size(), &opened, &openWhy);
    const bool refused = !written && !open && _Contains(writeWhy, part) &&
                         _Contains(openWhy, part);
    if (!refused) {
        std::printf("  expected a refusal naming '%s'; write: %s; open: %s\n",
                    part.c_str(), written ? "(accepted)" : writeWhy.c_str(),
                    open ? "(accepted)" : openWhy.c_str());
    }
    return refused;
}

/// \p file written, opened and written again: the same bytes, and the
/// opened copy; null when either step refuses.
static std::unique_ptr<fb::RigExecWireFile>
_RoundTrip(const fb::RigExecWireFile &file, std::vector<uint8_t> *bytes)
{
    std::string why;
    if (!RigExecFormatWrite(file, bytes, &why)) {
        std::printf("  write refused: %s\n", why.c_str());
        CHECK(false);
        return nullptr;
    }
    std::unique_ptr<fb::RigExecWireFile> opened;
    if (!RigExecFormatOpen(bytes->data(), bytes->size(), &opened, &why)) {
        std::printf("  open refused: %s\n", why.c_str());
        CHECK(false);
        return nullptr;
    }
    std::vector<uint8_t> again;
    CHECK(RigExecFormatWrite(*opened, &again, &why) && again == *bytes);
    return opened;
}

/// \p file with its first revision's op set to \p op.
static fb::RigExecWireFile
_WithOp(const fb::RigExecWireFile &file, uint8_t op)
{
    fb::RigExecWireFile copy(file);
    copy.geometry->chains[0].revisions[0].op = op;
    return copy;
}

/// \p file's first revision made a plugin mover, with its entry: a Token
/// node naming the type, an epoch and the frame bytes of a successful
/// assembly.
static fb::RigExecWireFile
_WithPlugin(const fb::RigExecWireFile &file)
{
    fb::RigExecWireFile copy = _WithOp(file, RigExecWireExternalRevisionOp);
    copy.names.push_back("TestConformancePlugin");
    copy.paths.push_back(fb::PathNode(0, uint32_t(copy.names.size() - 1),
                                      fb::PathKind::Token));
    fb::RigExecWireExternalMover mover;
    mover.chain = 0;
    mover.revision = 0;
    mover.type = uint32_t(copy.paths.size() - 1);
    mover.epoch = {'T', 'A', 'G', '1', 0, 255};
    mover.phasedFallback.assign(
        copy.geometry->chains[0].revisions[0].binding->phaseInputs.size(), 0);
    mover.v2Frame = {1, 2, 3, 0, 255};
    mover.v2FrameValid = true;
    copy.externalMovers.push_back(std::move(mover));
    return copy;
}

/// True when \p file's first chain holds a revision, which the codec tests
/// below edit.
static bool
_HasRevision(const fb::RigExecWireFile *file)
{
    return file && file->geometry && !file->geometry->chains.empty() &&
           !file->geometry->chains[0].revisions.empty() &&
           file->geometry->chains[0].revisions[0].binding;
}

// Revision ops on a real bake: 15 (wrinkle) opens; 10 and 11 stay
// reserved and 19 is past the last op, so each is refused naming the op;
// 16, a plugin mover, is refused without its external entry and opens
// with one.
static void
TestWrinkleWireExtensions(const std::vector<uint8_t> &bytes)
{
    const std::unique_ptr<fb::RigExecWireFile> file =
        RigExecTestUnpack(bytes);
    CHECK(_HasRevision(file.get()));
    if (!_HasRevision(file.get())) {
        return;
    }
    std::vector<uint8_t> out;
    const auto wrinkle = _RoundTrip(_WithOp(*file, 15), &out);
    CHECK(wrinkle && wrinkle->geometry->chains[0].revisions[0].op == 15);
    int refused = 0;
    for (const uint8_t op : {10, 11, 19}) {
        const bool named = _Refused(_WithOp(*file, op), "revision op");
        CHECK(named);
        refused += named ? 1 : 0;
    }
    CHECK(_Refused(_WithOp(*file, RigExecWireExternalRevisionOp),
                   "without its external_movers entry"));
    const auto plugin = _RoundTrip(_WithPlugin(*file), &out);
    CHECK(plugin && plugin->externalMovers.size() == 1);
    std::printf("revision ops: 15 opens, %d of 10/11/19 refused, 16 refused "
                "without its entry and opened with one\n",
                refused);
}

// A plugin mover's entry on a real bake: a successful and a failed
// assembly survive Write -> Open -> Write byte for byte, and every rule of
// an entry is refused naming it.
static void
TestExternalMoversWire(const std::vector<uint8_t> &bytes)
{
    const std::unique_ptr<fb::RigExecWireFile> file =
        RigExecTestUnpack(bytes);
    CHECK(_HasRevision(file.get()));
    if (!_HasRevision(file.get())) {
        return;
    }
    const fb::RigExecWireFile plugin = _WithPlugin(*file);
    std::vector<uint8_t> out;
    const auto back = _RoundTrip(plugin, &out);
    CHECK(back && back->externalMovers.size() == 1);
    if (back && back->externalMovers.size() == 1) {
        const fb::RigExecWireExternalMover &a = plugin.externalMovers[0];
        const fb::RigExecWireExternalMover &b = back->externalMovers[0];
        CHECK(a.chain == b.chain && a.revision == b.revision &&
              a.type == b.type && a.epoch == b.epoch &&
              a.phasedFallback == b.phasedFallback &&
              a.v2Frame == b.v2Frame && b.v2FrameValid && b.inputs.empty());
        CHECK(RigExecFormatPathText(*back, b.type) ==
              "TestConformancePlugin");
    }
    // A failed frame-0 assembly holds no bytes and still round-trips.
    fb::RigExecWireFile failed(plugin);
    failed.externalMovers[0].v2Frame.clear();
    failed.externalMovers[0].v2FrameValid = false;
    const auto failedBack = _RoundTrip(failed, &out);
    CHECK(failedBack && !failedBack->externalMovers[0].v2FrameValid &&
          failedBack->externalMovers[0].v2Frame.empty());

    // A phased input of the revision's binding takes a pooled fallback, and
    // its point binding: a base phase, which binds no version.
    fb::RigExecWireFile phased(plugin);
    {
        fb::RigExecWireRevision &revision =
            phased.geometry->chains[0].revisions[0];
        fb::RigExecWireRevisionBinding &binding = *revision.binding;
        binding.phaseInputs.push_back(binding.target);
        binding.phases.push_back(RigExecWireReadPhase());
        RigExecWirePointsBinding unbound;
        unbound.inputPath = binding.target;
        unbound.diagnoseMiss = true;
        revision.pointBindings.push_back(std::move(unbound));
        phased.externalMovers[0].phasedFallback.push_back(0);
    }
    CHECK(_RoundTrip(phased, &out) != nullptr);

    int refusals = 0;
    const auto expect = [&](const fb::RigExecWireFile &edited,
                            const char *part) {
        const bool named = _Refused(edited, part);
        CHECK(named);
        refusals += named ? 1 : 0;
    };
    fb::RigExecWireFile edited(plugin);
    edited.externalMovers[0].chain = uint32_t(edited.geometry->chains.size());
    expect(edited, "external_movers[0]: names no revision");
    edited = plugin;
    edited.externalMovers[0].revision =
        uint32_t(edited.geometry->chains[0].revisions.size());
    expect(edited, "external_movers[0]: names no revision");
    edited = plugin;
    edited.geometry->chains[0].revisions[0].op = 0;
    expect(edited, "external_movers[0]: names a revision that is not a "
                   "plugin mover");
    edited = plugin;
    edited.externalMovers.push_back(edited.externalMovers[0]);
    expect(edited, "external_movers[1]: names a revision that is not a "
                   "plugin mover, or one another entry names");
    edited = plugin;
    edited.externalMovers[0].type = 0;
    expect(edited, "external_movers[0].type");
    edited = plugin;
    edited.externalMovers[0].type = edited.rig;
    expect(edited, "external_movers[0].type");
    edited = plugin;
    edited.externalMovers[0].v2FrameValid = false;
    expect(edited, "external_movers[0].v2_frame: bytes of an assembly that "
                   "failed");
    edited = plugin;
    edited.externalMovers[0].phasedFallback.push_back(0);
    expect(edited, "phased_fallback");
    edited = phased;
    edited.externalMovers[0].phasedFallback.back() =
        uint32_t(edited.vec3fArrays.size());
    expect(edited, "phased_fallback");
    edited = plugin;
    edited.externalMovers.clear();
    expect(edited, "a plugin mover without its external_movers entry");
    std::printf("external movers: a successful and a failed assembly "
                "round-trip; %d entry violations refused\n",
                refusals);
}

/// Every vtable entry of the table at \p table, as byte offsets into the
/// buffer at \p base: the two header entries and each field's.
static std::vector<size_t>
_VtableBytes(const uint8_t *base, const uint8_t *table)
{
    const uint8_t *vtable =
        table - flatbuffers::ReadScalar<flatbuffers::soffset_t>(table);
    const uint16_t size = flatbuffers::ReadScalar<uint16_t>(vtable);
    std::vector<size_t> out;
    for (uint16_t at = 0; at < size; ++at) {
        out.push_back(size_t(vtable - base) + at);
    }
    return out;
}

// Corruption of a real bake (the 01 fixture's file): every byte of the
// root table's vtable and of the first revision's, flipped, then a bounded
// deterministic sweep of one to four byte edits that keep the identifier.
// Each result is refused with a reason, or opens and writes back to a file
// Open accepts; none crashes (a Release build: no sanitizer watches).
static void
TestRealFileCorruption(const std::vector<uint8_t> &bytes)
{
    CHECK(bytes.size() > 8);
    if (bytes.size() <= 8) {
        return;
    }
    // Physical table access follows strict Open. A transport envelope wraps
    // the same FlatBuffer; corruption below still exercises its verifier.
    std::unique_ptr<fb::RigExecWireFile> accepted;
    std::string error;
    const bool acceptedOpen = RigExecFormatOpen(bytes.data(), bytes.size(), &accepted, &error);
    CHECK(acceptedOpen);
    if (!acceptedOpen || !accepted) return;
    rigExec::transport::Buffer decoded;
    const uint8_t *payload = bytes.data();
    size_t payloadSize = bytes.size();
    if (rigExec::transport::IsEnvelope(payload, payloadSize)) {
        const bool ready = rigExec::transport::Decode(payload, payloadSize, &decoded, &error);
        CHECK(ready);
        CHECK(decoded.data && decoded.size >= 8);
        if (!ready || !decoded.data || decoded.size < 8) return;
        payload = decoded.data.get();
        payloadSize = decoded.size;
    }
    // The decoded owner remains alive through every physical table access.
    std::vector<uint64_t> aligned((payloadSize + 7) / 8);
    std::memcpy(aligned.data(), payload, payloadSize);
    const uint8_t *base = reinterpret_cast<const uint8_t *>(aligned.data());
    const fb::File *root = flatbuffers::GetRoot<fb::File>(base);
    const fb::Revision *revision = nullptr;
    if (root->geometry() && root->geometry()->chains() &&
        root->geometry()->chains()->size() > 0) {
        const fb::Chain *chain = root->geometry()->chains()->Get(0);
        if (chain->revisions() && chain->revisions()->size() > 0) {
            revision = chain->revisions()->Get(0);
        }
    }
    CHECK(revision);
    if (!revision) {
        return;
    }
    const std::vector<size_t> rootBytes =
        _VtableBytes(base, reinterpret_cast<const uint8_t *>(root));
    const std::vector<size_t> revisionBytes =
        _VtableBytes(base, reinterpret_cast<const uint8_t *>(revision));
    size_t refused = 0, opened = 0;
    const auto judge = [&](const std::vector<uint8_t> &mutated) {
        std::unique_ptr<fb::RigExecWireFile> file;
        std::string why;
        if (!RigExecFormatOpen(mutated.data(), mutated.size(), &file, &why)) {
            ++refused;
            CHECK(!why.empty());
            return;
        }
        ++opened;
        std::vector<uint8_t> rewritten;
        std::unique_ptr<fb::RigExecWireFile> again;
        CHECK(RigExecFormatWrite(*file, &rewritten, &why) &&
              RigExecFormatOpen(rewritten.data(), rewritten.size(), &again,
                                &why));
    };
    for (const std::vector<size_t> *offsets : {&rootBytes, &revisionBytes}) {
        for (const size_t at : *offsets) {
            std::vector<uint8_t> mutated(payload, payload + payloadSize);
            mutated[at] ^= 0xff;
            judge(mutated);
        }
    }
    const size_t vtableRefused = refused, vtableOpened = opened;
    refused = opened = 0;
    uint64_t state = 0x2545f4914f6cdd1dull;
    const auto next = [&state] {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        return state;
    };
    const uint8_t interesting[] = {0x00, 0x01, 0x04, 0x08,
                                   0x7f, 0x80, 0xfe, 0xff};
    constexpr int rounds = 2000;
    for (int round = 0; round < rounds; ++round) {
        std::vector<uint8_t> mutated(payload, payload + payloadSize);
        const int edits = 1 + int(next() % 4);
        for (int e = 0; e < edits; ++e) {
            const size_t at = next() % payloadSize;
            if (at >= 4 && at < 8) {
                continue;  // keep the identifier: reach the verifier
            }
            switch (next() % 3) {
            case 0:
                mutated[at] = interesting[next() % 8];
                break;
            case 1:
                mutated[at] = uint8_t(next());
                break;
            default: {
                // An aligned word set to a small offset or length.
                const size_t word = at & ~size_t(3);
                const uint32_t small = uint32_t(next() % payloadSize);
                if (word >= 8 && word + 4 <= mutated.size()) {
                    std::memcpy(mutated.data() + word, &small, 4);
                }
                break;
            }
            }
        }
        judge(mutated);
    }
    std::printf("real-file corruption: %zu root and %zu revision vtable "
                "bytes flipped, %zu refused, %zu opened; sweep %zu refused, "
                "%zu opened (of %d, %zu bytes)\n",
                rootBytes.size(), revisionBytes.size(), vtableRefused,
                vtableOpened, refused, opened, rounds, payloadSize);
    // Every mutation was judged, and both kinds were refused at least once.
    CHECK(vtableRefused + vtableOpened ==
          rootBytes.size() + revisionBytes.size());
    CHECK(refused + opened == size_t(rounds));
    CHECK(vtableRefused > 0 && refused > 0);
}

// The weight-gather capture hole: the volume gather target and driver-curve
// arrays must be path reads of the file. Fixture 11 TipCurve is an UNMOVED
// driver no chain base covers, so without the read the runtime builds the
// wrong field. Keyed by file name.
static void
_BinaryCheckWeightGatherReads(const std::string &fixture,
                              const std::set<std::string> &pathReads)
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
        if (!pathReads.count(path)) {
            std::printf("weight-gather read missing from %s: %s\n",
                        base.c_str(), path.c_str());
        }
        CHECK(pathReads.count(path));
    }
}

// -- The property chains over the file alone -------------------------------
//
// A reference for what the file holds: runChain (rigEvaluatorProperties.cpp)
// with _PinnedRead and GetAttribute over the slots' defaults, publishing
// into an overlay keyed by slot name. Envelopes come from \p envelope; a
// chain with an envelope and no resolver is left unevaluated and reported.

/// A slot default or a read's constant in one shape.
struct _Val {
    fb::InputTag tag = fb::InputTag::Double;
    uint64_t bits = 0;
    RigExecWireMatrix4d matrix{};
    RigExecWireVec3d vec3d{};
    RigExecWireVec3f vec3f{};
};

static _Val
_ValOf(const fb::RigExecWireValue &value)
{
    _Val out;
    out.tag = value.tag;
    out.bits = value.bits;
    if (value.matrix) {
        out.matrix = *value.matrix;
    }
    if (value.vec3d) {
        out.vec3d = *value.vec3d;
    }
    if (value.vec3f) {
        out.vec3f = *value.vec3f;
    }
    return out;
}

static bool
_HasValue(const fb::InputSlot &slot)
{
    return (slot.flags() & uint8_t(fb::InputSlotFlags::HasValue)) != 0;
}

static _Val
_SlotValue(const fb::RigExecWireFile &file, uint32_t slot)
{
    return _ValOf(file.values[file.inputs[slot].value()]);
}

/// A property result as the chains publish it.
struct _ChainValue {
    enum class Tag : uint8_t { Float, Double, Matrix4d, Vec3f };
    Tag tag = Tag::Float;
    float f32 = 0;
    double f64 = 0;
    RigExecWireMatrix4d matrix{};
    RigExecWireVec3f vec{};
};

using _ChainOverlay = std::map<uint32_t, _ChainValue>;
using _ChainEnvelope =
    std::function<bool(int32_t object, const _ChainOverlay &overlay,
                       float *weight, std::string *error)>;

static bool
_ChainOverlayHit(const _ChainOverlay &overlay, uint32_t name,
                 fb::InputTag tag, _Val *out)
{
    const auto found = overlay.find(name);
    if (found == overlay.end()) {
        return false;
    }
    using Held = _ChainValue::Tag;
    const _ChainValue &held = found->second;
    _Val value;
    value.tag = tag;
    if (tag == fb::InputTag::Float && held.tag == Held::Float) {
        value.bits = _BinaryBits(held.f32);
    } else if (tag == fb::InputTag::Double && held.tag == Held::Double) {
        value.bits = _BinaryBits(held.f64);
    } else if (tag == fb::InputTag::Matrix4d && held.tag == Held::Matrix4d) {
        value.matrix = held.matrix;
    } else if (tag == fb::InputTag::Vec3f && held.tag == Held::Vec3f) {
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
_ChainLongWay(const fb::RigExecWireFile &file, const _ChainOverlay &overlay,
              const std::vector<uint32_t> &walk, size_t from,
              fb::InputTag tag, _Val *out)
{
    const auto narrow = [&](size_t k) {
        _Val wide;
        if (!_ChainLongWay(file, overlay, walk, k, fb::InputTag::Double,
                           &wide)) {
            return false;
        }
        double d = 0.0;
        std::memcpy(&d, &wide.bits, sizeof(d));
        *out = _Val();
        out->tag = fb::InputTag::Float;
        out->bits = _BinaryBits(static_cast<float>(d));
        return true;
    };
    if (tag == fb::InputTag::Float && from < walk.size() &&
        file.inputs[walk[from]].type() == fb::InputTag::Double) {
        return narrow(from);
    }
    for (size_t k = from; k < walk.size(); ++k) {
        const fb::InputSlot &slot = file.inputs[walk[k]];
        if (_ChainOverlayHit(overlay, slot.name(), tag, out)) {
            return true;
        }
        if (tag == fb::InputTag::Float && slot.type() == fb::InputTag::Double) {
            return narrow(k);
        }
    }
    for (size_t k = walk.size(); k-- > from;) {
        const uint32_t slot = walk[k];
        if (_HasValue(file.inputs[slot]) && file.inputs[slot].type() == tag) {
            *out = _SlotValue(file, slot);
            return true;
        }
    }
    return false;
}

// _PinnedRead (Pinned: the attribute's own overlay, then its own typed
// value) or GetAttribute (Resolved), falling back to the read's constant.
static _Val
_ChainRead(const fb::RigExecWireFile &file, const _ChainOverlay &overlay,
           const fb::RigExecWireInput &input)
{
    _Val value = _ValOf(file.values[input.constant]);
    if (input.mode == fb::ReadMode::Resolved) {
        _ChainLongWay(file, overlay, input.walk, 0, input.tag, &value);
        return value;
    }
    CHECK(input.mode == fb::ReadMode::Pinned && input.walk.size() <= 1);
    if (input.walk.empty()) {
        return value;
    }
    const uint32_t slot = input.walk[0];
    if (_ChainOverlayHit(overlay, file.inputs[slot].name(), input.tag,
                         &value)) {
        return value;
    }
    if (_HasValue(file.inputs[slot]) && file.inputs[slot].type() == input.tag) {
        value = _SlotValue(file, slot);
    }
    return value;
}

static float
_ChainFloat(const _Val &value)
{
    const uint32_t bits = uint32_t(value.bits);
    float f = 0.0f;
    std::memcpy(&f, &bits, sizeof(f));
    return f;
}

static GfVec3f
_ChainVec3f(const _Val &value)
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
_ChainFinite(const _ChainValue &value)
{
    using Tag = _ChainValue::Tag;
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

/// \p value against a property result the program holds, bit for bit.
static bool
_ChainSameAsHeld(const _ChainValue &value, const VtValue &held)
{
    using Tag = _ChainValue::Tag;
    if (held.IsHolding<float>()) {
        return value.tag == Tag::Float &&
               _BinarySame(value.f32, held.UncheckedGet<float>());
    }
    if (held.IsHolding<double>()) {
        return value.tag == Tag::Double &&
               _BinarySame(value.f64, held.UncheckedGet<double>());
    }
    if (held.IsHolding<GfMatrix4d>()) {
        const RigExecWireMatrix4d m =
            _ChainWireMatrix(held.UncheckedGet<GfMatrix4d>());
        return value.tag == Tag::Matrix4d &&
               std::memcmp(m.data(), value.matrix.data(),
                           sizeof(double) * 16) == 0;
    }
    if (held.IsHolding<GfVec3f>()) {
        const GfVec3f &v = held.UncheckedGet<GfVec3f>();
        return value.tag == Tag::Vec3f && _BinarySame(value.vec[0], v[0]) &&
               _BinarySame(value.vec[1], v[1]) &&
               _BinarySame(value.vec[2], v[2]);
    }
    return false;
}

/// Runs every chain over the slots' defaults into \p overlay (cleared
/// first) and appends the chains' diagnostic lines. False when a chain
/// needs an envelope and \p envelope is empty: that chain and everything
/// after it are left out.
static bool
_ComputedRunChains(const fb::RigExecWireFile &file,
                   const _ChainEnvelope &envelope, _ChainOverlay *overlay,
                   std::vector<std::string> *diagnostics)
{
    using Tag = _ChainValue::Tag;
    overlay->clear();
    // Final reads may point forward in the serialized chain table. Order
    // this independent reference evaluator by those declared dependencies.
    std::vector<size_t> order;
    std::vector<uint8_t> visited(file.propertyChains.size(), 0);
    std::function<bool(size_t)> visit = [&](size_t c) {
        if (visited[c] == 2) return true;
        if (visited[c] == 1) return false;
        visited[c] = 1;
        for (const auto &revision : file.propertyChains[c].revisions) {
            for (const auto *read : {revision.enabled.get(),
                    revision.defaultWeight.get(), revision.value.get(),
                    revision.min.get(), revision.max.get()}) {
                for (const auto *candidates : {&read->propertyCandidates,
                                               &read->doubleCandidates}) {
                    for (const auto &candidate : *candidates) {
                        if (candidate.version < 0) continue;
                        for (size_t other = 0; other < file.propertyChains.size(); ++other) {
                            const auto &source = file.propertyChains[other];
                            if (other != c && candidate.slot == source.target &&
                                uint32_t(candidate.version) > source.versionBase &&
                                !visit(other)) return false;
                        }
                    }
                }
            }
        }
        visited[c] = 2;
        order.push_back(c);
        return true;
    };
    for (size_t c = 0; c < file.propertyChains.size(); ++c) {
        const bool ordered = visit(c);
        CHECK(ordered);
        if (!ordered) return false;
    }
    for (size_t c : order) {
        const fb::RigExecWirePropertyChain &chain = file.propertyChains[c];
        const fb::InputSlot &target = file.inputs[chain.target];
        const std::string targetText = _BinaryText(file, target.name());
        fb::InputTag baseTag = fb::InputTag::Float;
        Tag tag = Tag::Float;
        switch (chain.valueType) {
        case fb::PropertyValueType::Double:
            baseTag = fb::InputTag::Double;
            tag = Tag::Double;
            break;
        case fb::PropertyValueType::Matrix4d:
            baseTag = fb::InputTag::Matrix4d;
            tag = Tag::Matrix4d;
            break;
        case fb::PropertyValueType::Vec3f:
            baseTag = fb::InputTag::Vec3f;
            tag = Tag::Vec3f;
            break;
        default:
            break;
        }
        if (!_HasValue(target) || target.type() != baseTag) {
            diagnostics->push_back("property chain " + targetText +
                                   ": target has no authored value; chain "
                                   "skipped");
            continue;
        }
        const _Val base = _SlotValue(file, chain.target);
        _ChainValue value;
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
        for (size_t k = 0; k < file.phasedConsumers.size(); ++k) {
            if (file.phasedConsumers[k].chain == c) {
                phased.push_back(k);
            }
        }
        std::vector<_ChainValue> history;
        for (const fb::RigExecWirePropertyRevision &revision :
             chain.revisions) {
            if (!phased.empty()) {
                history.push_back(value);
            }
            const std::string mover = _BinaryText(file, revision.mover);
            if (_ChainRead(file, *overlay, *revision.enabled).bits == 0) {
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
                if (!envelope(revision.envelope, *overlay, &weight, &error)) {
                    diagnostics->push_back("diag " + mover + ": " + error +
                                           "; revision passed through");
                    continue;
                }
            } else {
                weight = _ChainFloat(
                    _ChainRead(file, *overlay, *revision.defaultWeight));
                if (!std::isfinite(weight) || weight < 0.0f ||
                    weight > 1.0f) {
                    diagnostics->push_back(
                        "diag " + mover +
                        ": inputs:defaultWeight must be finite and in "
                        "[0, 1]; revision passed through");
                    continue;
                }
            }
            const bool validOp = revision.op != fb::PropertyOp::Invalid;
            const RigExecPropertyOp op = RigExecPropertyOp(revision.op);
            _ChainValue next = value;
            bool usable = validOp;
            if (usable && (tag == Tag::Float || tag == Tag::Double)) {
                RigExecPropertyMathParams<float> params;
                params.op = op;
                params.value =
                    _ChainFloat(_ChainRead(file, *overlay, *revision.value));
                params.min =
                    _ChainFloat(_ChainRead(file, *overlay, *revision.min));
                params.max =
                    _ChainFloat(_ChainRead(file, *overlay, *revision.max));
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
                params.value =
                    _ChainVec3f(_ChainRead(file, *overlay, *revision.value));
                params.min =
                    _ChainVec3f(_ChainRead(file, *overlay, *revision.min));
                params.max =
                    _ChainVec3f(_ChainRead(file, *overlay, *revision.max));
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
                    _ChainRead(file, *overlay, *revision.value).matrix);
                _ChainValue probe;
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
        (*overlay)[target.name()] = value;
        if (phased.empty()) {
            continue;
        }
        history.push_back(value);
        for (size_t k : phased) {
            const fb::RigExecWirePhasedConsumer &consumer =
                file.phasedConsumers[k];
            _ChainValue v = history[std::min(size_t(consumer.applied),
                                             history.size() - 1)];
            if (consumer.consumerType == fb::PropertyValueType::Double &&
                v.tag == Tag::Float) {
                v.tag = Tag::Double;
                v.f64 = double(v.f32);
                v.f32 = 0;
            } else if (consumer.consumerType ==
                           fb::PropertyValueType::Float &&
                       v.tag == Tag::Double) {
                v.tag = Tag::Float;
                v.f32 = float(v.f64);
                v.f64 = 0;
            }
            (*overlay)[file.inputs[consumer.consumer].name()] = v;
        }
    }
    return true;
}

/// \p value against \p expected, bit for bit on the member its tag names.
template <class T>
static bool
_ValIs(const _Val &value, const T &expected, const fb::RigExecWireFile &file)
{
    fb::RigExecWireValue probe;
    probe.tag = value.tag;
    probe.bits = value.bits;
    probe.matrix = std::make_unique<RigExecWireMatrix4d>(value.matrix);
    probe.vec3d = std::make_unique<RigExecWireVec3d>(value.vec3d);
    probe.vec3f = std::make_unique<RigExecWireVec3f>(value.vec3f);
    if constexpr (std::is_same_v<T, GfVec3f>) {
        return value.tag == fb::InputTag::Vec3f &&
               _BinarySame(value.vec3f[0], expected[0]) &&
               _BinarySame(value.vec3f[1], expected[1]) &&
               _BinarySame(value.vec3f[2], expected[2]);
    } else {
        return value.tag == _BinaryTagOf<T>() &&
               _BinaryConstantIs(probe, expected, file);
    }
}

/// The chains over the file's slots against the program's property
/// results from the bake's run, bit for bit, and each registered read that
/// crosses a chain target (the reference's long way over its walk) against
/// the program's own long-way read at the bake time. Returns the property
/// values compared; *skipped is 1 when a chain needs an envelope and
/// \p envelope is empty.
static size_t
_BinaryCheckChains(const RigExecBakedProgramImpl &program,
                   const fb::RigExecWireFile &file,
                   const _ChainEnvelope &envelope, size_t *skipped,
                   std::vector<std::string> *diagnostics,
                   size_t *chainReads)
{
    *skipped = 0;
    *chainReads = 0;
    _ChainOverlay overlay;
    std::vector<std::string> lines;
    if (!_ComputedRunChains(file, envelope, &overlay, &lines)) {
        *skipped = 1;
        return 0;
    }
    if (diagnostics) {
        *diagnostics = lines;
    }
    std::map<std::string, uint32_t> nameIds;
    for (const fb::InputSlot &slot : file.inputs) {
        nameIds[_BinaryText(file, slot.name())] = slot.name();
    }
    CHECK(program.propertyResults.size() == overlay.size());
    size_t compared = 0;
    for (const auto &entry : program.propertyResults) {
        const auto id = nameIds.find(entry.first.GetString());
        const auto found =
            id == nameIds.end() ? overlay.end() : overlay.find(id->second);
        if (found == overlay.end()) {
            std::printf("chain reference publishes nothing at %s\n",
                        entry.first.GetText());
            CHECK(found != overlay.end());
            continue;
        }
        const bool same = _ChainSameAsHeld(found->second, entry.second);
        if (!same) {
            std::printf("chain reference differs from the program at %s\n",
                        entry.first.GetText());
        }
        CHECK(same);
        ++compared;
    }
    std::map<std::string, const fb::RigExecWireInput *> crossing;
    _BinaryForEachRead(file, [&](const fb::RigExecWireInput &read) {
        if (read.mode == fb::ReadMode::Baked && !read.walk.empty() &&
            (read.flags & uint8_t(fb::InputReadFlags::ViaChain))) {
            crossing.emplace(_BinarySlotName(file, read.walk[0]), &read);
        }
    });
    CHECK(program.resolvedInputs);
    if (!program.resolvedInputs) {
        return compared;
    }
    const UsdTimeCode time(file.bakeTime);
    frozenDetail::_ForEachPatchableInput(program, [&](const auto &input) {
        if (!input.resolvedAttr) {
            return;
        }
        const auto found =
            crossing.find(input.resolvedAttr.GetPath().GetString());
        if (found == crossing.end()) {
            return;
        }
        const fb::RigExecWireInput &read = *found->second;
        auto expected = input.constant;
        program.resolvedInputs->GetAttribute(input.resolvedAttr, time,
                                             &expected);
        _Val value = _ValOf(file.values[read.constant]);
        _ChainLongWay(file, overlay, read.walk, 0, read.tag, &value);
        const bool same = _ValIs(value, expected, file);
        if (!same) {
            std::printf("chain read reference differs at %s\n",
                        found->first.c_str());
        }
        CHECK(same);
        ++*chainReads;
    });
    return compared;
}

/// The input list of a bake against the program it came from: one
/// step-backed weight object per program object, one Baked read per
/// registered program input and one crossing a chain per input that reads
/// through one, slots ordered by path text, and the chains reproducing the
/// program's property results.
static void
_BinaryCheckInputs(const RigExecBakedProgramImpl &program,
                   const fb::RigExecWireFile &file)
{
    size_t stepBacked = 0, inFlight = 0, staticErrors = 0;
    for (const fb::RigExecWireWeightObject &object :
         file.geometry->weightObjects) {
        stepBacked += object.envelopeOnly ? 0 : 1;
        inFlight += object.samplesInFlight ? 1 : 0;
        staticErrors += object.oracleStaticError.empty() ? 0 : 1;
    }
    CHECK(stepBacked == program.weightObjects.size());
    std::string previous;
    for (size_t s = 0; s < file.inputs.size(); ++s) {
        const std::string name = _BinaryText(file, file.inputs[s].name());
        CHECK(s == 0 || s == file.listedInputs || previous < name);
        previous = name;
    }
    size_t registered = 0, crossing = 0;
    frozenDetail::_ForEachPatchableInput(program, [&](const auto &input) {
        ++registered;
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
    size_t baked = 0, fileCrossing = 0;
    _BinaryForEachRead(file, [&](const fb::RigExecWireInput &read) {
        if (read.mode != fb::ReadMode::Baked) {
            return;
        }
        ++baked;
        fileCrossing +=
            (read.flags & uint8_t(fb::InputReadFlags::ViaChain)) ? 1 : 0;
    }, false);
    CHECK(registered == baked);
    CHECK(crossing == fileCrossing);
    std::printf("  input list: %zu slots, %zu weight objects (%zu "
                "envelope-only, %zu in flight, %zu with a static oracle "
                "error), %zu registered reads (%zu through a chain), %zu "
                "path reads\n",
                file.inputs.size(), file.geometry->weightObjects.size(),
                file.geometry->weightObjects.size() - stepBacked, inFlight,
                staticErrors, baked, fileCrossing,
                file.geometry->pathReads.size());
    // The chains as the file states them reproduce every property result
    // and chain-crossing read of the run (chains that need an envelope are
    // left to the tests that resolve one).
    size_t skipped = 0, chainReads = 0;
    const size_t chainValues = _BinaryCheckChains(program, file, nullptr,
                                                  &skipped, nullptr,
                                                  &chainReads);
    if (!file.propertyChains.empty()) {
        std::printf("  chains: %zu chain(s), %zu phased consumer(s), %zu "
                    "property result(s) and %zu chain read(s) reproduced "
                    "bit for bit, %zu left to an envelope resolver\n",
                    file.propertyChains.size(), file.phasedConsumers.size(),
                    chainValues, chainReads, skipped);
    }
}

/// RigExecResolvedInputs::GetAttribute over the slots' defaults, with no
/// property result published.
static bool
_ComputedLongWay(const fb::RigExecWireFile &file,
                 const std::vector<uint32_t> &walk, size_t from,
                 fb::InputTag tag, _Val *out)
{
    for (size_t k = from; k < walk.size(); ++k) {
        if (tag == fb::InputTag::Float &&
            file.inputs[walk[k]].type() == fb::InputTag::Double) {
            _Val wide;
            if (!_ComputedLongWay(file, walk, k, fb::InputTag::Double,
                                  &wide)) {
                return false;
            }
            double d = 0.0;
            std::memcpy(&d, &wide.bits, sizeof(d));
            *out = _Val();
            out->tag = fb::InputTag::Float;
            out->bits = _BinaryBits(static_cast<float>(d));
            return true;
        }
    }
    for (size_t k = walk.size(); k-- > from;) {
        const uint32_t slot = walk[k];
        if (_HasValue(file.inputs[slot]) && file.inputs[slot].type() == tag) {
            *out = _SlotValue(file, slot);
            return true;
        }
    }
    return false;
}

static float
_ComputedReadFloat(const fb::RigExecWireFile &file,
                   const fb::RigExecWireInput &input)
{
    _Val value = _ValOf(file.values[input.constant]);
    _ComputedLongWay(file, input.walk, 0, fb::InputTag::Float, &value);
    return _ChainFloat(value);
}

static const fb::RigExecWireInput *
_EnvelopeScalarRead(const fb::RigExecWireFile &file, int fieldIndex, int member)
{
    CHECK(file.geometry && fieldIndex >= 0 && size_t(fieldIndex) < file.geometry->weightFields.size());
    if (!file.geometry || fieldIndex < 0 || size_t(fieldIndex) >= file.geometry->weightFields.size()) return nullptr;
    const auto &field = file.geometry->weightFields[size_t(fieldIndex)];
    const fb::RigExecWireInput *read = nullptr;
    for (size_t r = 0; r < field.scalarReads.size() && r < field.scalarObjects.size() && r < field.scalarMembers.size(); ++r) {
        if (field.scalarObjects[r] == field.object && int(field.scalarMembers[r]) == member) {
            CHECK(!read);
            read = &field.scalarReads[r];
        }
    }
    CHECK(read);
    if (read) CHECK(read->mode == fb::ReadMode::Resolved && read->tag == fb::InputTag::Float && read->overrideIndex == -1);
    return read;
}

/// A bake of \p stage's rig at \p time, compared with the program; null
/// with a failed CHECK when it does not bake or open.
static std::unique_ptr<fb::RigExecWireFile>
_BakeAndCompare(RigExecRigEvaluator &evaluator, double time,
                const std::string &label)
{
    RigExecBakeOpts opts;
    opts.time = time;
    RigExecBakeResult result;
    std::string error;
    const bool baked = RigExecBakeToBinary(evaluator, opts, &result, &error);
    if (!baked) {
        std::printf("%s bake diagnostic: %s\n", label.c_str(), error.c_str());
    }
    CHECK(baked);
    if (!baked) {
        return nullptr;
    }
    std::unique_ptr<fb::RigExecWireFile> file =
        _BinaryCompareProgram(evaluator, result.bytes, label);
    if (file) {
        _BinaryCheckInputs(evaluator.GetBakedProgram()->GetStepGraph(),
                           *file);
    }
    return file;
}

// The phase fixtures and 13_ReadPhases: two bakes from fresh evaluators are
// the same bytes, and the file holds the program's frame records, record
// lists and point bindings. The record fixtures carry records, the example
// a bound phased input, the blend fixture, baked where its lifts have moved
// the shape, two blend samples whose bindings answer, and the volume
// fixture a volume the oracle resolves, whose sampled points are an input.
static void
TestPhaseBindingBakes()
{
    const std::filesystem::path fixtures =
        std::filesystem::path(RIGEXEC_EXAMPLES_DIR) / ".." / "tests" /
        "fixtures";
    struct Row {
        std::string stage;
        double time;
        bool records;
        bool bindings;
        bool samples = false;
        bool oracle = false;
    };
    const Row rows[] = {
        {(fixtures / "frame_record_fallbacks.usda").string(), 1.0, true,
         false},
        {(fixtures / "solver_checkpoint.usda").string(), 1.0, true, false},
        {(fixtures / "volume_placements.usda").string(), 1.0, false, false,
         false, true},
        {(std::filesystem::path(RIGEXEC_EXAMPLES_DIR) / "13_ReadPhases.usda")
             .string(),
         1001.0, false, true},
        {(fixtures / "phased_blend_samples.usda").string(), 3.0, false,
         false, true},
    };
    for (const Row &row : rows) {
        const UsdStageRefPtr stage = UsdStage::Open(row.stage);
        CHECK(stage);
        if (!stage) {
            continue;
        }
        const SdfPath rigPath = FindRig(stage);
        std::vector<uint8_t> first;
        _BinaryCompareStats stats;
        const int failuresBefore = failures;
        bool same = false;
        for (int pass = 0; pass < 2; ++pass) {
            RigExecRigEvaluator evaluator(stage, rigPath);
            RigExecBakeOpts opts;
            opts.time = row.time;
            RigExecBakeResult result;
            std::string error;
            const bool baked =
                RigExecBakeToBinary(evaluator, opts, &result, &error);
            CHECK(baked);
            if (!baked) {
                std::printf("%s bake diagnostic: %s\n", row.stage.c_str(),
                            error.c_str());
                break;
            }
            if (pass == 1) {
                same = result.bytes == first;
                CHECK(same);
                continue;
            }
            first = result.bytes;
            CHECK(_BinaryCompareProgram(evaluator, result.bytes, row.stage,
                                        &stats));
        }
        std::printf("phase tables %s: %zu records, %zu record lists, %zu "
                    "bindings, %zu sample bindings (%zu answered), %zu "
                    "oracle point inputs; %zu bytes, %s; %d failures\n",
                    row.stage.c_str(), stats.frameRecords, stats.recordLists,
                    stats.pointBindings, stats.sampleBindings,
                    stats.answeredSamples, stats.arrayOracle, first.size(),
                    same ? "two bakes identical" : "BAKES DIFFER",
                    failures - failuresBefore);
        CHECK((stats.arrayOracle > 0) == row.oracle);
        if (row.records) {
            CHECK(stats.frameRecords > 0 && stats.recordLists > 0);
        }
        if (row.bindings) {
            CHECK(stats.pointBindings > 0);
        }
        if (row.samples) {
            CHECK(stats.sampleBindings > 0 &&
                  stats.answeredSamples == stats.sampleBindings);
        }
    }
}

// tests/fixtures/raw_skin_layouts.usda baked at time 1: three skin layouts
// the sparse form cannot hold are stored raw, two sparse, each against the
// program's own arrays; two bakes are identical, and the file writes back
// to its bytes.
static void
TestRawSkinLayoutBake()
{
    const std::string fixture =
        (std::filesystem::path(RIGEXEC_EXAMPLES_DIR) / ".." / "tests" /
         "fixtures" / "raw_skin_layouts.usda")
            .string();
    const UsdStageRefPtr stage = UsdStage::Open(fixture);
    CHECK(stage);
    if (!stage) {
        return;
    }
    const SdfPath rigPath = FindRig(stage);
    std::vector<uint8_t> first;
    _BinaryCompareStats stats;
    const int failuresBefore = failures;
    bool same = false;
    for (int pass = 0; pass < 2; ++pass) {
        RigExecRigEvaluator evaluator(stage, rigPath);
        RigExecBakeOpts opts;
        opts.time = 1.0;
        RigExecBakeResult result;
        std::string error;
        const bool baked =
            RigExecBakeToBinary(evaluator, opts, &result, &error);
        CHECK(baked);
        if (!baked) {
            std::printf("raw skin layouts bake diagnostic: %s\n",
                        error.c_str());
            return;
        }
        if (pass == 1) {
            same = result.bytes == first;
            CHECK(same);
            continue;
        }
        first = result.bytes;
        const std::unique_ptr<_BinaryFile> file =
            _BinaryCompareProgram(evaluator, result.bytes, fixture, &stats);
        CHECK(file);
        std::vector<uint8_t> rewritten;
        CHECK(file && RigExecFormatWrite(*file, &rewritten, &error) &&
              rewritten == result.bytes);
    }
    std::printf("raw skin layouts: %zu topologies, %zu of them raw; %zu "
                "bytes, %s; %d failures\n",
                stats.topologies, stats.rawTopologies, first.size(),
                same ? "two bakes identical" : "BAKES DIFFER",
                failures - failuresBefore);
    CHECK(stats.topologies == 5 && stats.rawTopologies == 3);
}

// Two transform constraints whose envelopes no mover binds: a StaticWeight
// at 0.5 and a DynamicWeight whose driver connects to an animated double.
// Both become envelope-only entries read the Resolved way, and the oracle's
// formula over the file's slot defaults reproduces, bit for bit, the
// envelope the program's run resolved, in a bake at each of three times.
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

    for (const double time : {1.0, 5.0, 10.0}) {
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        const std::unique_ptr<fb::RigExecWireFile> file =
            _BakeAndCompare(evaluator, time, "envelope bake");
        if (!file) {
            return;
        }
        const RigExecBakedProgramImpl &program =
            evaluator.GetBakedProgram()->GetStepGraph();
        const auto &objects = file->geometry->weightObjects;
        CHECK(program.weightObjects.size() == 2);
        CHECK(objects.size() == 2);
        CHECK(file->pose->constraints.size() == program.constraints.size());
        size_t checkedEnvelopes = 0;
        for (size_t k = 0; k < program.constraints.size() &&
                           k < file->pose->constraints.size();
             ++k) {
            const int32_t index = file->pose->constraints[k].weightObjectIndex;
            CHECK(index >= 0 && size_t(index) < objects.size());
            if (index < 0 || size_t(index) >= objects.size()) {
                continue;
            }
            const fb::RigExecWireWeightObject &object =
                objects[size_t(index)];
            CHECK(!object.envelopeOnly);
            const int fieldIndex = file->pose->constraints[k].weightField;
            CHECK(fieldIndex >= 0 && size_t(fieldIndex) < file->geometry->weightFields.size());
            if (fieldIndex < 0 || size_t(fieldIndex) >= file->geometry->weightFields.size()) continue;
            const auto &field = file->geometry->weightFields[size_t(fieldIndex)];
            CHECK(field.object == index && uint8_t(field.form) == 1);
            size_t producers = 0;
            for (const auto &step : program.steps)
                producers += step.kind == RigExecBakedStepKind::WeightPacket && step.object == index ? 1 : 0;
            CHECK(producers == 1);
            CHECK(object.oracleStaticError.empty());
            CHECK(object.oraclePhaseError.empty());
            CHECK(object.defaultWeight->mode == fb::ReadMode::Baked);
            const bool dynamic =
                RigExecFormatPathText(*file, object.type) ==
                "RigExecDynamicWeight";
            const auto *defaultWeight = _EnvelopeScalarRead(*file, fieldIndex, 0);
            const auto *driverRead = dynamic ? _EnvelopeScalarRead(*file, fieldIndex, 1) : nullptr;
            const auto *scaleRead = dynamic ? _EnvelopeScalarRead(*file, fieldIndex, 2) : nullptr;
            const auto *biasRead = dynamic ? _EnvelopeScalarRead(*file, fieldIndex, 3) : nullptr;
            if (!defaultWeight || (dynamic && (!driverRead || !scaleRead || !biasRead))) continue;
            if (dynamic) {
                // inputs:driver walks to the avar it is connected to.
                CHECK(driverRead->walk.size() == 2);
                if (driverRead->walk.size() == 2) {
                    const fb::InputSlot &leaf =
                        file->inputs[driverRead->walk[1]];
                    CHECK(RigExecFormatPathText(*file, leaf.name()) ==
                          amount.GetPath().GetString());
                    CHECK(leaf.type() == fb::InputTag::Double);
                    CHECK(leaf.flags() &
                          uint8_t(fb::InputSlotFlags::Animated));
                }
            }
            // _ResolveWeights' two scalar arms at count 1, no base.
            float expected = 0.0f;
            if (dynamic) {
                const float base = 1.0f;
                const float driver =
                    _ComputedReadFloat(*file, *driverRead);
                const float scale = _ComputedReadFloat(*file, *scaleRead);
                const float bias = _ComputedReadFloat(*file, *biasRead);
                expected = (base * driver) * scale + bias;
            } else {
                expected = _ComputedReadFloat(*file, *defaultWeight);
            }
            const auto &scratch = program.constraints[k].weightScratch;
            CHECK(scratch.size() == 1);
            if (scratch.size() != 1) {
                continue;
            }
            if (!_BinarySame(expected, scratch[0])) {
                std::printf("envelope %zu at %g: computed %.9g, resolved "
                            "%.9g\n",
                            k, time, double(expected), double(scratch[0]));
            }
            CHECK(_BinarySame(expected, scratch[0]));
            ++checkedEnvelopes;
        }
        CHECK(checkedEnvelopes == 2);
    }
}

// A geometry-domain constraint whose sphere weight samples the points in
// flight (testRigExecVolumeWeights' ConstraintEnvelopeFixture, with the
// sphere's height keyed): the step-backed object is marked in flight with
// no static samples, the constraint resolves no scalar envelope, and the
// program's run measured a dense current-phase packet for its revision in
// a bake at each of two times, moving with the sphere between them.
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

    const std::vector<double> times = {1.0, 2.0};
    // The packet the program measured against the entering points, per bake.
    std::vector<std::vector<float>> fields;
    for (const double time : times) {
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        const std::unique_ptr<fb::RigExecWireFile> file =
            _BakeAndCompare(evaluator, time, "current-phase bake");
        if (!file) {
            return;
        }
        const RigExecBakedProgramImpl &program =
            evaluator.GetBakedProgram()->GetStepGraph();
        for (const fb::RigExecWireConstraint &c : file->pose->constraints) {
            CHECK(c.weightObjectIndex == -1);
        }
        const fb::RigExecWireWeightObject *found = nullptr;
        for (const fb::RigExecWireWeightObject &object :
             file->geometry->weightObjects) {
            if (RigExecFormatPathText(*file, object.path) ==
                sphere.GetPath().GetString()) {
                found = &object;
            }
        }
        CHECK(found);
        if (!found) {
            return;
        }
        CHECK(!found->envelopeOnly);
        CHECK(found->samplesInFlight);
        CHECK(found->oracleSamplesSlot == -1);
        CHECK(found->oraclePhaseError.empty());
        CHECK(found->oracleStaticError.empty());
        CHECK(found->falloffMax->mode == fb::ReadMode::Baked);
        CHECK(found->falloffMax->walk.size() == 1);
        if (found->falloffMax->walk.size() == 1) {
            CHECK(_BinarySlotName(*file, found->falloffMax->walk[0]) ==
                  "/Asset/Rig/Weights/Sphere.inputs:falloffMax");
        }
        for (const auto &chain : program.chains) {
            for (const auto &revision : chain.revisions) {
                const RigExecWeightPacket &packet =
                    revision.currentPhasePacket;
                if (packet.valid && packet.values.size() == 3) {
                    fields.push_back(packet.values);
                }
            }
        }
    }
    CHECK(fields.size() == times.size());
    CHECK(fields.size() < 2 || fields[0] != fields[1]);
}

// The oracle's two scalar arms for an envelope field at count 1
// (rigEvaluatorGeometry.cpp _ResolveWeights): a constant StaticWeight and
// a DynamicWeight with no base, reading through GetAttribute with the
// oracle's own fallbacks.
static _ChainEnvelope
_ChainScalarEnvelopes(const fb::RigExecWireFile &file)
{
    return [&file](int32_t index, const _ChainOverlay &overlay,
                   float *weight, std::string *error) {
        const fb::RigExecWireWeightObject &object =
            file.geometry->weightObjects[size_t(index)];
        const std::string type = _BinaryText(file, object.type);
        const std::string rangePolicy = _BinaryText(file, object.rangePolicy);
        const std::string representation =
            _BinaryText(file, object.representation);
        const std::string path = _BinaryText(file, object.path);
        int fieldIndex = -1;
        for (size_t f = 0; f < file.geometry->weightFields.size(); ++f) {
            const auto &field = file.geometry->weightFields[f];
            if (field.object == index && uint8_t(field.form) == 0) { fieldIndex = int(f); break; }
        }
        const auto *defaultWeight = _EnvelopeScalarRead(file, fieldIndex, 0);
        const bool dynamic = type == "RigExecDynamicWeight";
        const auto *driver = dynamic ? _EnvelopeScalarRead(file, fieldIndex, 1) : nullptr;
        const auto *scale = dynamic ? _EnvelopeScalarRead(file, fieldIndex, 2) : nullptr;
        const auto *bias = dynamic ? _EnvelopeScalarRead(file, fieldIndex, 3) : nullptr;
        if (!defaultWeight || (dynamic && (!driver || !scale || !bias))) return false;
        const auto read = [&](const fb::RigExecWireInput &input,
                              float fallback) {
            _Val value;
            return _ChainLongWay(file, overlay, input.walk, 0,
                                 fb::InputTag::Float, &value)
                       ? _ChainFloat(value)
                       : fallback;
        };
        std::string painted;
        CHECK(!object.envelopeOnly && object.oracleStaticError.empty() &&
              object.base < 0 && object.inputs.empty() &&
              representation == "constant" &&
              (object.valuesSlot < 0 ||
               (_BinarySlotBytes(file, object.valuesSlot, &painted) &&
                painted.empty())));
        if (type == "RigExecStaticWeight") {
            float w = read(*defaultWeight, 0.0f);
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
        const float driverValue = read(*driver, 1.0f);
        const float scaleValue = read(*scale, 1.0f);
        const float biasValue = read(*bias, 0.0f);
        float r = (1.0f * driverValue) * scaleValue + biasValue;
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
// property result of the run bit for bit and the chains' diagnostic lines,
// in a bake at each of five times.
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
    for (const double time : {1.0, 3.0, 5.0, 7.0, 10.0}) {
        RigExecRigEvaluator evaluator(stage, rigPath);
        const std::unique_ptr<fb::RigExecWireFile> owned =
            _BakeAndCompare(evaluator, time, "chain bake");
        if (!owned) {
            return;
        }
        const fb::RigExecWireFile &file = *owned;
        const RigExecBakedProgramImpl &program =
            evaluator.GetBakedProgram()->GetStepGraph();
        const auto text = [&](uint32_t id) {
            return RigExecFormatPathText(file, id);
        };
        const auto slotName = [&](uint32_t slot) {
            return _BinarySlotName(file, slot);
        };
        std::map<std::string, const fb::RigExecWirePropertyChain *> chains;
        for (size_t c = 0; c < file.propertyChains.size(); ++c) {
            const fb::RigExecWirePropertyChain &chain = file.propertyChains[c];
            chains[slotName(chain.target)] = &chain;
            CHECK(file.inputs[chain.target].chain() == int32_t(c));
        }
        CHECK(chains.size() == 8);
        const fb::RigExecWirePropertyChain *dial =
            chains["/Asset/Rig/Channels/Dial.rigExec:amount"];
        const fb::RigExecWirePropertyChain *wide =
            chains["/Asset/Rig/Channels/Wide.rigExec:level"];
        const fb::RigExecWirePropertyChain *vec =
            chains["/Asset/Rig/Channels/Vec.rigExec:offset"];
        const fb::RigExecWirePropertyChain *space =
            chains["/Asset/Rig/Channels/Space.rigExec:local"];
        const fb::RigExecWirePropertyChain *missing =
            chains["/Asset/Rig/Channels/Missing.rigExec:none"];
        CHECK(dial && wide && vec && space && missing);
        if (!dial || !wide || !vec || !space || !missing) {
            return;
        }
        // A consumer chain runs after the chain it reads.
        const auto position = [&](const fb::RigExecWirePropertyChain *chain) {
            return chain - file.propertyChains.data();
        };
        CHECK(position(dial) <
              position(chains["/Asset/Rig/Channels/Readouts.rigExec:early"]));
        CHECK(dial->valueType == fb::PropertyValueType::Float);
        CHECK(wide->valueType == fb::PropertyValueType::Double);
        CHECK(vec->valueType == fb::PropertyValueType::Vec3f);
        CHECK(space->valueType == fb::PropertyValueType::Matrix4d);
        CHECK(!_HasValue(file.inputs[missing->target]));
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
            const fb::RigExecWirePropertyRevision &gain = dial->revisions[0];
            CHECK(gain.op == fb::PropertyOp::Multiply);
            CHECK(gain.value->mode == fb::ReadMode::Resolved &&
                  gain.value->tag == fb::InputTag::Float &&
                  gain.value->walk.size() == 2);
            if (gain.value->walk.size() == 2) {
                CHECK(slotName(gain.value->walk[1]) ==
                      "/Asset/Rig/Controls/Dial.avars:tx");
                CHECK(file.inputs[gain.value->walk[1]].type() ==
                      fb::InputTag::Double);
            }
            const fb::RigExecWirePropertyRevision &shape = dial->revisions[1];
            CHECK(shape.op == fb::PropertyOp::Curve &&
                  shape.keys.size() == 3 && shape.tangents.size() == 3 &&
                  shape.hasTangentsAttr);
            if (shape.keys.size() == 3) {
                CHECK(shape.keys[1] == RigExecWireVec2f({0.5f, 0.8f}));
            }
            const auto envelopeOf =
                [&](const fb::RigExecWirePropertyRevision &revision) {
                    return revision.envelope >= 0
                               ? text(file.geometry
                                          ->weightObjects[size_t(
                                              revision.envelope)]
                                          .path)
                               : std::string();
                };
            CHECK(envelopeOf(dial->revisions[2]) == "/Asset/Rig/Weights/Half");
            CHECK(envelopeOf(dial->revisions[3]) == "/Asset/Rig/Weights/Ramp");
            CHECK(dial->revisions[0].envelope == -1);
            const fb::RigExecWirePropertyRevision &off = dial->revisions[4];
            CHECK(off.enabled->mode == fb::ReadMode::Pinned &&
                  off.enabled->tag == fb::InputTag::Bool &&
                  off.enabled->walk.size() == 1);
            // MoverAPI declares inputs:enabled, so an unauthored one still
            // reads its own schema fallback.
            const fb::RigExecWireInput &enabled = *dial->revisions[0].enabled;
            CHECK(enabled.mode == fb::ReadMode::Pinned &&
                  enabled.walk.size() == 1);
            if (enabled.walk.size() == 1) {
                CHECK(_SlotValue(file, enabled.walk[0]).bits == 1);
            }
            const fb::RigExecWirePropertyRevision &over = dial->revisions[5];
            CHECK(over.defaultWeight->mode == fb::ReadMode::Pinned &&
                  over.defaultWeight->walk.size() == 1);
            CHECK(dial->revisions[6].op == fb::PropertyOp::Clamp);
        }
        CHECK(wide->revisions.size() == 1 &&
              wide->revisions[0].value->tag == fb::InputTag::Float);
        CHECK(vec->revisions.size() == 2 &&
              vec->revisions[0].value->tag == fb::InputTag::Vec3f);
        CHECK(space->revisions.size() == 1 &&
              space->revisions[0].value->tag == fb::InputTag::Matrix4d &&
              space->revisions[0].min->walk.empty() &&
              space->revisions[0].max->walk.empty());
        // The phased consumers: the dial's base, the dial after Shape (two
        // revisions applied), and the double chain's base read as a float.
        CHECK(file.phasedConsumers.size() == 3);
        std::map<std::string, fb::RigExecWirePhasedConsumer> phased;
        for (size_t k = 0; k < file.phasedConsumers.size(); ++k) {
            const fb::RigExecWirePhasedConsumer &consumer =
                file.phasedConsumers[k];
            phased[slotName(consumer.consumer)] = consumer;
            CHECK(file.inputs[consumer.consumer].phased() == int32_t(k));
        }
        const fb::RigExecWirePhasedConsumer base =
            phased["/Asset/Rig/Movers/Readouts/Base.inputs:value"];
        const fb::RigExecWirePhasedConsumer early =
            phased["/Asset/Rig/Movers/Readouts/Early.inputs:value"];
        const fb::RigExecWirePhasedConsumer narrow =
            phased["/Asset/Rig/Movers/Readouts/Narrow.inputs:value"];
        CHECK(file.propertyChains.data() + base.chain == dial &&
              base.applied == 0 &&
              base.consumerType == fb::PropertyValueType::Float);
        CHECK(file.propertyChains.data() + early.chain == dial &&
              early.applied == 2);
        CHECK(file.propertyChains.data() + narrow.chain == wide &&
              narrow.applied == 0 &&
              narrow.consumerType == fb::PropertyValueType::Float);

        // Follow's weight declares `final`, so it is the one registered read
        // that crosses a chain rather than a phased consumer: its walk runs
        // from its own attribute to the dial's target, and it holds the
        // override number the program registered for that attribute.
        std::vector<const fb::RigExecWireInput *> crossing;
        _BinaryForEachRead(file, [&](const fb::RigExecWireInput &read) {
            if (read.mode == fb::ReadMode::Baked &&
                (read.flags & uint8_t(fb::InputReadFlags::ViaChain))) {
                crossing.push_back(&read);
            }
        }, false);
        CHECK(crossing.size() == 1);
        if (crossing.size() == 1) {
            const fb::RigExecWireInput &weight = *crossing[0];
            const std::string head =
                "/Asset/Rig/Movers/Follow.inputs:defaultWeight";
            CHECK(weight.tag == fb::InputTag::Float &&
                  weight.walk.size() == 2);
            if (weight.walk.size() == 2) {
                CHECK(slotName(weight.walk[0]) == head);
                const int32_t chain = file.inputs[weight.walk[1]].chain();
                CHECK(chain >= 0 && file.propertyChains.data() + chain == dial);
            }
            const auto numbers = program.overridableInputs.find(SdfPath(head));
            CHECK(numbers != program.overridableInputs.end() &&
                  std::find(numbers->second.begin(), numbers->second.end(),
                            weight.overrideIndex) != numbers->second.end());
            size_t memoCopies = 0;
            for (const auto &step : file.steps) for (const auto &read : step.headInputReads)
                if (read.overrideIndex == weight.overrideIndex) { CHECK(_BinarySameRead(read, weight)); ++memoCopies; }
            CHECK(memoCopies > 0);
        }

        // Every property result, with the envelopes resolved, and the
        // chains' lines as the evaluator printed them first at the time.
        size_t skipped = 0, chainReads = 0;
        std::vector<std::string> lines;
        const size_t compared = _BinaryCheckChains(
            program, file, _ChainScalarEnvelopes(file), &skipped, &lines,
            &chainReads);
        std::printf("  computed chains at %g: %zu value(s) and %zu chain "
                    "read(s) reproduced bit for bit\n",
                    time, compared, chainReads);
        CHECK(skipped == 0);
        CHECK(compared == 10);
        CHECK(chainReads == 1);
        CHECK(lines.size() == 3);
        RigExecRigEvaluator replay(stage, rigPath);
        const RigExecRigPose pose = replay.Evaluate(UsdTimeCode(time));
        const auto at = std::search(pose.diagnostics.begin(),
                                    pose.diagnostics.end(), lines.begin(),
                                    lines.end());
        if (at == pose.diagnostics.end()) {
            for (const std::string &line : lines) {
                std::printf("  reference line: %s\n", line.c_str());
            }
            for (const std::string &line : pose.diagnostics) {
                std::printf("  evaluator line: %s\n", line.c_str());
            }
        }
        CHECK(at != pose.diagnostics.end());
    }
}

/// Whether \p step declares a read of slot \p slot of \p domain.
static bool
_DeclaresRead(const fb::RigExecWireStep &step, fb::SlotDomain domain,
              uint32_t slot)
{
    for (const fb::SlotRange &range : step.reads) {
        if (range.domain() == domain && range.begin() <= slot &&
            slot < range.end()) {
            return true;
        }
    }
    return false;
}

/// Removes slot \p slot of \p domain from \p step's reads, keeping the rest
/// of a range that held it.
static void
_DropRead(fb::RigExecWireStep *step, fb::SlotDomain domain, uint32_t slot)
{
    std::vector<fb::SlotRange> kept;
    for (const fb::SlotRange &range : step->reads) {
        if (range.domain() != domain || slot < range.begin() ||
            slot >= range.end()) {
            kept.push_back(range);
            continue;
        }
        if (range.begin() < slot) {
            kept.emplace_back(domain, range.begin(), slot);
        }
        if (slot + 1 < range.end()) {
            kept.emplace_back(domain, slot + 1, range.end());
        }
    }
    step->reads = std::move(kept);
}

// A chain of 10000 points, past the default chunk vertex target and cut into
// G vertex groups at the default group target, under a target-space blend
// shape authored last, which mover discovery runs first (siblings run
// bottom to top), and three matrix movers (the range-chain fixture's shape:
// M0 on a driver that moves at frames 1 to 3, M1 with a weight out of range
// at frame 2, M2; the points authored at Default and at frames 1 to 3,
// moving in [0, 100) only), baked at frame 1. In format 20 every revision
// is Range, the blend shape too, and none is gated (no weight object):
// unchunked, cut into the chain's G groups with no keys, one group step per
// group. The blend shape's group steps read the chain base; every later
// revision's group g reads the previous revision's group g and no whole
// version; each join reads its own groups and the version entering it. The
// validator accepts the file. Edited copies that break a group rule are
// refused by Write and Open: a key on a group, a gap, two revisions cutting
// the chain differently, a group-shaped Smooth, a group step missing its
// read of the previous revision's group (after a matrix mover and after the
// blend shape), a group step declaring the previous revision's version, a
// join missing one of its own groups, and a join missing the version
// entering it.
static void
TestRangeChainBake()
{
    constexpr size_t kPoints = 10000;
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const UsdPrim moving = stage->DefinePrim(SdfPath("/Asset/Rig/Moving"),
                                             TfToken("RigExecControl"));
    const UsdAttribute tx =
        moving.CreateAttribute(TfToken("avars:tx"), SdfValueTypeNames->Double);
    for (int frame = 1; frame <= 3; ++frame) {
        tx.Set(double(frame), UsdTimeCode(frame));
    }
    const UsdPrim still = stage->DefinePrim(SdfPath("/Asset/Rig/Still"),
                                            TfToken("RigExecControl"));
    still.CreateAttribute(TfToken("avars:ty"), SdfValueTypeNames->Double)
        .Set(1.0);
    const UsdPrim shape =
        stage->DefinePrim(SdfPath("/Asset/Shape"), TfToken("Points"));
    const UsdAttribute pointsAttr = shape.CreateAttribute(
        TfToken("points"), SdfValueTypeNames->Point3fArray);
    VtVec3fArray points(kPoints);
    for (size_t i = 0; i < kPoints; ++i) {
        points[i] = GfVec3f(float(i % 100), float(i / 100), 0.0f);
    }
    pointsAttr.Set(points);
    for (int frame = 1; frame <= 3; ++frame) {
        VtVec3fArray at = points;
        for (size_t i = 0; i < 100; ++i) {
            at[i][2] = float(frame - 1);
        }
        pointsAttr.Set(at, UsdTimeCode(frame));
    }
    // The blend shape's target: every point raised by one.
    VtVec3fArray raised = points;
    for (GfVec3f &point : raised) {
        point[2] += 1.0f;
    }
    stage->DefinePrim(SdfPath("/Asset/Raised"), TfToken("Points"))
        .CreateAttribute(TfToken("points"), SdfValueTypeNames->Point3fArray)
        .Set(raised);
    const UsdPrim input = stage->DefinePrim(
        SdfPath("/Asset/Rig/BlendInputs/Raise"), TfToken("RigExecBlendInput"));
    input.CreateAttribute(TfToken("inputs:weight"), SdfValueTypeNames->Float)
        .Set(0.5f);
    const UsdPrim sample = stage->DefinePrim(
        SdfPath("/Asset/Rig/BlendInputs/Raise/Full"),
        TfToken("RigExecBlendSample"));
    sample.CreateAttribute(TfToken("rigExec:activation"),
                           SdfValueTypeNames->Float)
        .Set(1.0f);
    sample.CreateRelationship(TfToken("rigExec:targetPoints"))
        .SetTargets({SdfPath("/Asset/Raised.points")});
    input.CreateRelationship(TfToken("rigExec:samples"))
        .SetTargets({sample.GetPath()});
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    for (int i = 0; i < 3; ++i) {
        const UsdPrim mover = stage->DefinePrim(
            SdfPath("/Asset/Rig/Movers/M" + std::to_string(i)),
            TfToken("RigExecMatrixMover"));
        CHECK(mover.ApplyAPI(TfToken("RigExecMoverAPI")));
        mover.CreateRelationship(TfToken("rigExec:moves"))
            .SetTargets({SdfPath("/Asset/Shape.points")});
        mover.CreateRelationship(TfToken("rigExec:transform"))
            .SetTargets({i == 0 ? moving.GetPath() : still.GetPath()});
        const UsdAttribute weight = mover.CreateAttribute(
            TfToken("inputs:defaultWeight"), SdfValueTypeNames->Float);
        if (i == 1) {
            weight.Set(1.0f, UsdTimeCode(1.0));
            weight.Set(2.0f, UsdTimeCode(2.0));
            weight.Set(1.0f, UsdTimeCode(3.0));
        } else {
            weight.Set(1.0f);
        }
    }
    const UsdPrim blend = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Blend"), TfToken("RigExecBlendShapeMover"));
    CHECK(blend.ApplyAPI(TfToken("RigExecMoverAPI")));
    blend.CreateRelationship(TfToken("rigExec:moves"))
        .SetTargets({SdfPath("/Asset/Shape.points")});
    blend.CreateRelationship(TfToken("rigExec:blendInputs"))
        .SetTargets({input.GetPath()});
    blend.CreateAttribute(TfToken("inputs:defaultWeight"),
                          SdfValueTypeNames->Float)
        .Set(1.0f);

    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    const std::unique_ptr<fb::RigExecWireFile> file =
        _BakeAndCompare(evaluator, 1.0, "range chain bake");
    if (!file) {
        return;
    }
    const RigExecBakedProgramImpl &program =
        evaluator.GetBakedProgram()->GetStepGraph();
    const fb::RigExecWireDomainGeometry &g = *file->geometry;
    int chainIndex = -1;
    for (size_t c = 0; c < g.chains.size(); ++c) {
        if (RigExecFormatPathText(*file, g.chains[c].target) ==
            "/Asset/Shape.points") {
            chainIndex = int(c);
        }
    }
    CHECK(chainIndex >= 0);
    if (chainIndex < 0) {
        return;
    }
    const size_t c = size_t(chainIndex);
    const fb::RigExecWireChain &chain = g.chains[c];
    CHECK(chain.revisions.size() == 4);
    if (chain.revisions.size() != 4) {
        return;
    }
    // The chain's G groups, as Build cut them.
    const size_t ranges = RigExecFormatChainGroups(*file, c);
    CHECK(ranges >= 2);
    CHECK(c < program.chains.size() &&
          program.chains[c].groupBounds.size() == ranges + 1);
    // The blend shape runs first; each matrix mover after it, in whatever
    // order. Every one cuts the chain into one group step per group.
    CHECK(RigExecFormatPathText(*file, chain.revisions[0].moverPath) ==
          "/Asset/Rig/Movers/Blend");
    const uint32_t first = uint32_t(g.chainRevisionBegin[c]);
    std::set<std::string> movers;
    size_t rangeSteps = 0;
    for (size_t r = 0; r < chain.revisions.size(); ++r) {
        const fb::RigExecWireRevision &revision = chain.revisions[r];
        if (r > 0) {
            movers.insert(RigExecFormatPathText(*file, revision.moverPath));
        }
        CHECK(revision.op == uint8_t(r == 0 ? fb::RevisionOp::BlendShape
                                            : fb::RevisionOp::Matrix));
        CHECK(RigExecFormatIsRangeRevision(revision));
        CHECK(!RigExecFormatIsKeyedRevision(revision));
        CHECK(revision.chunks.size() == ranges);
        for (size_t k = 0; k < revision.chunks.size(); ++k) {
            CHECK(revision.chunks[k].key.empty());
            CHECK(uint64_t(revision.chunks[k].begin) ==
                  RigExecPointRangeBound(kPoints, ranges, k));
            CHECK(uint64_t(revision.chunks[k].end) ==
                  RigExecPointRangeBound(kPoints, ranges, k + 1));
        }
        const int32_t id = g.chainRevisionBegin[c] + int32_t(r);
        CHECK(size_t(id) < g.revisionChunkCount.size() &&
              size_t(g.revisionChunkCount[size_t(id)]) == ranges);
        for (const fb::RigExecWireStep &step : file->steps) {
            if (step.kind == fb::StepKind::RevisionChunk &&
                step.object == id) {
                ++rangeSteps;
                CHECK(_DeclaresRead(step, fb::SlotDomain::ChainBase,
                                    uint32_t(c)));
                // Group `part` of the entering version: the previous
                // revision's group, never its whole version.
                if (r > 0) {
                    const uint32_t entering = uint32_t(
                        chain.revisions[r - 1].chunkBase + step.part);
                    CHECK(_DeclaresRead(step, fb::SlotDomain::RevisionOut,
                                        entering));
                    CHECK(!_DeclaresRead(step, fb::SlotDomain::RevisionDone,
                                         uint32_t(id) - 1) &&
                          !_DeclaresRead(step, fb::SlotDomain::ChainDirty,
                                         uint32_t(id) - 1));
                }
            }
            if (step.kind == fb::StepKind::RevisionFuse &&
                step.object == id) {
                // A join declares its own groups and the version entering
                // it, as a fuse does.
                for (size_t k = 0; k < ranges; ++k) {
                    CHECK(_DeclaresRead(step, fb::SlotDomain::RevisionOut,
                                        uint32_t(revision.chunkBase + k)));
                }
                if (r > 0) {
                    CHECK(_DeclaresRead(step, fb::SlotDomain::RevisionDone,
                                        uint32_t(id) - 1) &&
                          _DeclaresRead(step, fb::SlotDomain::ChainDirty,
                                        uint32_t(id) - 1));
                }
            }
        }
    }
    CHECK(movers == std::set<std::string>({"/Asset/Rig/Movers/M0",
                                           "/Asset/Rig/Movers/M1",
                                           "/Asset/Rig/Movers/M2"}));
    CHECK(rangeSteps == 4 * ranges);
    {
        std::string why;
        CHECK(RigExecFormatValidate(*file, &why));
    }

    // The edits. Position 1 is the first matrix mover, whose groups read
    // the blend shape's; position 2 follows a matrix mover.
    const uint32_t firstRange = first + 1;
    const uint32_t second = first + 2;
    const uint32_t blendBase = uint32_t(chain.revisions[0].chunkBase);
    const uint32_t firstBase = uint32_t(chain.revisions[1].chunkBase);
    const auto revisionAt = [c](fb::RigExecWireFile &f,
                                size_t r) -> fb::RigExecWireRevision & {
        return f.geometry->chains[c].revisions[r];
    };
    const auto stepOf = [](fb::RigExecWireFile &f, fb::StepKind kind,
                           uint32_t id, int32_t part) {
        fb::RigExecWireStep *found = nullptr;
        for (fb::RigExecWireStep &step : f.steps) {
            if (step.kind == kind && step.object == int32_t(id) &&
                (kind != fb::StepKind::RevisionChunk || step.part == part)) {
                found = &step;
            }
        }
        return found;
    };
    int refused = 0;
    const auto refuse = [&](const char *label,
                            const std::function<bool(fb::RigExecWireFile &)>
                                &edit,
                            const std::string &part) {
        fb::RigExecWireFile copy(*file);
        const bool edited = edit(copy);
        CHECK(edited);
        const bool named = edited && _Refused(copy, part);
        if (!named) {
            std::printf("  range chain bake: %s\n", label);
        }
        CHECK(named);
        refused += named ? 1 : 0;
    };
    refuse("a key on a group", [&](fb::RigExecWireFile &f) {
        revisionAt(f, 1).chunks[1].key = {0};
        return true;
    }, "chunks[1]: a key on an unchunked revision");
    refuse("a gap", [&](fb::RigExecWireFile &f) {
        ++revisionAt(f, 1).chunks[1].begin;
        return true;
    }, "does not continue the chain's point partition");
    refuse("two revisions cutting the chain differently",
           [&](fb::RigExecWireFile &f) {
               fb::RigExecWireRevision &revision = revisionAt(f, 2);
               --revision.chunks[0].end;
               --revision.chunks[1].begin;
               return true;
           },
           ".revisions[2]: its point ranges differ from revisions[0]'s");
    refuse("a group-shaped smooth revision", [&](fb::RigExecWireFile &f) {
        revisionAt(f, 1).op = uint8_t(fb::RevisionOp::Smooth);
        return true;
    }, std::to_string(ranges) + " chunks, but not chunked");
    refuse("a group step missing the previous revision's group",
           [&](fb::RigExecWireFile &f) {
               fb::RigExecWireStep *step =
                   stepOf(f, fb::StepKind::RevisionChunk, second, 1);
               if (!step || !_DeclaresRead(*step, fb::SlotDomain::RevisionOut,
                                           firstBase + 1)) {
                   return false;
               }
               _DropRead(step, fb::SlotDomain::RevisionOut, firstBase + 1);
               return true;
           },
           " does not declare RevisionOut[" + std::to_string(firstBase + 1) +
               "]");
    refuse("a group step missing the blend shape's group",
           [&](fb::RigExecWireFile &f) {
               fb::RigExecWireStep *step =
                   stepOf(f, fb::StepKind::RevisionChunk, firstRange, 1);
               if (!step || !_DeclaresRead(*step, fb::SlotDomain::RevisionOut,
                                           blendBase + 1)) {
                   return false;
               }
               _DropRead(step, fb::SlotDomain::RevisionOut, blendBase + 1);
               return true;
           },
           " does not declare RevisionOut[" + std::to_string(blendBase + 1) +
               "]");
    refuse("a group step declaring the previous revision's version",
           [&](fb::RigExecWireFile &f) {
               fb::RigExecWireStep *step =
                   stepOf(f, fb::StepKind::RevisionChunk, second, 1);
               if (!step || _DeclaresRead(*step, fb::SlotDomain::RevisionDone,
                                          firstRange)) {
                   return false;
               }
               step->reads.emplace_back(fb::SlotDomain::RevisionDone,
                                        firstRange, firstRange + 1);
               step->reads.emplace_back(fb::SlotDomain::ChainDirty,
                                        firstRange, firstRange + 1);
               return true;
           },
           "which a range-pipelined range step does not read");
    refuse("a join missing one of its groups", [&](fb::RigExecWireFile &f) {
        fb::RigExecWireStep *join =
            stepOf(f, fb::StepKind::RevisionFuse, firstRange, -1);
        const uint32_t own = firstBase + uint32_t(ranges) - 1;
        if (!join || !_DeclaresRead(*join, fb::SlotDomain::RevisionOut, own)) {
            return false;
        }
        _DropRead(join, fb::SlotDomain::RevisionOut, own);
        return true;
    }, " does not declare RevisionOut[" +
           std::to_string(firstBase + ranges - 1) + "]");
    refuse("a join missing the version entering it",
           [&](fb::RigExecWireFile &f) {
               fb::RigExecWireStep *join =
                   stepOf(f, fb::StepKind::RevisionFuse, second, -1);
               if (!join ||
                   !_DeclaresRead(*join, fb::SlotDomain::RevisionDone,
                                  firstRange)) {
                   return false;
               }
               _DropRead(join, fb::SlotDomain::RevisionDone, firstRange);
               return true;
           },
           " does not declare RevisionDone[" + std::to_string(firstRange) +
               "]");
    std::printf("range chain bake: %zu Range revisions of %zu groups "
                "accepted; %d group rule violations refused\n",
                chain.revisions.size(), ranges, refused);
    CHECK(refused == 9);
}

// A connection-following read whose walk reaches a property chain's target
// takes the chain's result in the run: here a wrinkle scale connected, at
// its final read phase, to an attribute a float mover revises. The file
// holds it as a read row whose walk crosses the chain's target, so the
// runtime reads the chain's result, as the run's enumeration does. A walk
// over the stage alone would read the authored value.
static void
TestEnumeratedReadThroughPropertyResult()
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    const SdfPath rig("/Rig"), target("/Rig/Body.points");
    RigExecRigBuilder builder = RigExecRigBuilder::Create(stage, rig);
    constexpr int nx = 5, ny = 4;
    std::vector<GfVec3f> rest;
    VtIntArray counts, indices;
    for (int y = 0; y < ny; ++y) {
        for (int x = 0; x < nx; ++x) {
            rest.emplace_back(0.25f * float(x), 0.2f * float(y), 0.0f);
        }
    }
    for (int y = 0; y + 1 < ny; ++y) {
        for (int x = 0; x + 1 < nx; ++x) {
            const int a = y * nx + x;
            counts.push_back(4);
            for (const int i : {a, a + 1, a + 1 + nx, a + nx}) {
                indices.push_back(i);
            }
        }
    }
    VtVec3fArray compressed(rest.begin(), rest.end());
    for (GfVec3f &p : compressed) {
        p[0] *= 0.65f;
    }
    const UsdGeomMesh mesh = UsdGeomMesh::Define(stage, target.GetPrimPath());
    CHECK(mesh.CreatePointsAttr().Set(compressed));
    CHECK(mesh.CreateFaceVertexCountsAttr().Set(counts));
    CHECK(mesh.CreateFaceVertexIndicesAttr().Set(indices));
    RigExecWrinkleMoverHandle wrinkle =
        builder.NewMoverChain("Deform", target).AddWrinkleMover("Wrinkle");
    wrinkle.SetRestPoints(rest);
    const UsdAttribute driver = stage->GetPrimAtPath(rig).CreateAttribute(
        TfToken("wrinkleScale"), SdfValueTypeNames->Float);
    const float authored = 0.4f;
    CHECK(driver.Set(authored));
    const UsdAttribute scale =
        wrinkle.GetPrim().GetAttribute(TfToken("inputs:wrinkleScale"));
    CHECK(scale.SetConnections({driver.GetPath()}));
    wrinkle.SetReadPhase(TfToken("inputs:wrinkleScale"), "final");
    builder.NewMoverChain("Scale", driver.GetPath())
        .AddFloatMathMover("Blend", TfToken("blend"), 0.8f);

    RigExecRigEvaluator evaluator(stage, rig);
    RigExecBakeOpts opts;
    opts.time = 1.0;
    RigExecBakeResult result;
    std::string error;
    const bool baked = RigExecBakeToBinary(evaluator, opts, &result, &error);
    CHECK(baked);
    if (!baked) {
        std::printf("read through a property result: %s\n", error.c_str());
        return;
    }
    CHECK(result.pathReadsWritten > 0);
    const RigExecBakedProgramImpl &program =
        evaluator.GetBakedProgram()->GetStepGraph();
    const auto chained = program.propertyResults.find(driver.GetPath());
    CHECK(chained != program.propertyResults.end() &&
          chained->second.IsHolding<float>());
    if (chained == program.propertyResults.end() ||
        !chained->second.IsHolding<float>()) {
        return;
    }
    const float value = chained->second.UncheckedGet<float>();
    CHECK(value != authored);
    const std::unique_ptr<fb::RigExecWireFile> file =
        _BinaryCompareProgram(evaluator, result.bytes, "read through");
    CHECK(file);
    if (!file) {
        return;
    }
    const fb::RigExecWirePathRead *row = nullptr;
    for (const fb::RigExecWirePathRead &candidate : file->geometry->pathReads) {
        if (!candidate.rest && RigExecFormatPathText(*file, candidate.path) ==
                                   scale.GetPath().GetString()) {
            row = &candidate;
        }
    }
    CHECK(row && row->read && row->read->tag == fb::InputTag::Float &&
          (row->read->flags & uint8_t(fb::InputReadFlags::ViaChain)));
    bool throughDriver = false;
    if (row && row->read) {
        for (const uint32_t slot : row->read->walk) {
            throughDriver = throughDriver ||
                            (_BinarySlotName(*file, slot) ==
                                 driver.GetPath().GetString() &&
                             file->inputs[slot].chain() >= 0);
        }
    }
    CHECK(throughDriver);
    std::vector<RigExecBakeRevisionRead> enumerated;
    RigExecBakeEnumerateProgramReads(program, 1.0, &enumerated);
    bool enumeratedResult = false;
    for (const RigExecBakeRevisionRead &candidate : enumerated) {
        if (candidate.path == scale.GetPath() && !candidate.rest &&
            candidate.value.IsHolding<float>()) {
            enumeratedResult = enumeratedResult ||
                               _BinarySame(candidate.value.UncheckedGet<float>(),
                                           value);
        }
    }
    CHECK(enumeratedResult);
    std::printf("read through a property result: %s is a read row through "
                "the chain target %s, enumerated at the chain's result %g "
                "(authored %g); %zu rows written, %zu keys enumerated\n",
                scale.GetPath().GetText(), driver.GetPath().GetText(),
                double(value), double(authored), result.pathReadsWritten,
                result.pathReadsEnumerated);
}

// The static report lists a path read a geometry assembly makes at the time
// whose attribute the stage animates and no input slot evaluates: the stage
// with the mesh points held reports nothing, and keying the projector's
// rigExec:projectionMode token (a raw read at the time) in the session
// layer makes it report that attribute, once, as a revision read.
static void
TestStaticReportRevisionReads()
{
    const std::string path =
        (std::filesystem::path(RIGEXEC_EXAMPLES_DIR) / ".." / "tests" /
         "rigExecPathReadsHeldPoints.usda")
            .string();
    const UsdStageRefPtr stage = UsdStage::Open(path);
    CHECK(stage);
    if (!stage) {
        return;
    }
    const SdfPath rig("/PathReadAsset/Rig");
    std::vector<RigExecBakeStaticEntry> entries;
    std::string error;
    {
        RigExecRigEvaluator evaluator(stage, rig);
        CHECK(RigExecBakeStaticReport(evaluator, &entries, &error));
        CHECK(entries.empty());
        for (const RigExecBakeStaticEntry &entry : entries) {
            std::printf("  unexpected static %s: %s\n", entry.field.c_str(),
                        entry.source.c_str());
        }
    }
    const SdfPath mode(
        "/PathReadAsset/Rig/Movers/Projector.rigExec:projectionMode");
    {
        UsdEditContext context(stage, stage->GetSessionLayer());
        const UsdAttribute attribute =
            stage->GetPrimAtPath(mode.GetPrimPath())
                .CreateAttribute(mode.GetNameToken(),
                                 SdfValueTypeNames->Token, false);
        CHECK(attribute.Set(TfToken("material"), UsdTimeCode(1.0)) &&
              attribute.Set(TfToken("material"), UsdTimeCode(10.0)));
    }
    {
        RigExecRigEvaluator evaluator(stage, rig);
        CHECK(RigExecBakeStaticReport(evaluator, &entries, &error));
    }
    stage->GetSessionLayer()->Clear();
    const bool reported =
        entries.size() == 1 &&
        entries[0].field ==
            "revision read /PathReadAsset/Rig/Movers/Projector" &&
        entries[0].source == mode.GetString();
    CHECK(reported);
    std::printf("static report: a keyed raw path read is %s (%zu "
                "entr%s)\n",
                reported ? "reported" : "NOT reported", entries.size(),
                entries.size() == 1 ? "y" : "ies");
}

// The static report names a phased dense blend sample's points only when
// its binding can never answer, since playback then reads the points the
// bake held. On tests/fixtures/phased_blend_samples.usda with Shape.points
// keyed, both samples are phased and bound to a version of Shape's chain,
// whose Animated base is now stage-sampled, so the report names neither
// chain base nor sample points: an answering binding reads the version.
// Default-only and structural animation remains covered by the class guard.
static void
TestStaticReportAnsweredBlendSamples()
{
    const std::string path =
        (std::filesystem::path(RIGEXEC_EXAMPLES_DIR) / ".." / "tests" /
         "fixtures" / "phased_blend_samples.usda")
            .string();
    const UsdStageRefPtr stage = UsdStage::Open(path);
    CHECK(stage);
    if (!stage) {
        return;
    }
    const SdfPath rig("/Asset/Rig");
    const SdfPath shape("/Asset/Geom/Shape.points");
    {
        UsdEditContext context(stage, stage->GetSessionLayer());
        const UsdAttribute attribute = stage->GetAttributeAtPath(shape);
        VtVec3fArray value = {GfVec3f(0, 0, 2), GfVec3f(1, 0, 2),
                              GfVec3f(1, 1, 2), GfVec3f(0, 1, 2)};
        CHECK(attribute.Set(value, UsdTimeCode(1.0)));
        for (GfVec3f &point : value) {
            point[2] += 1.0f;
        }
        CHECK(attribute.Set(value, UsdTimeCode(9.0)));
    }
    std::vector<RigExecBakeStaticEntry> entries;
    size_t bound = 0;
    {
        RigExecRigEvaluator evaluator(stage, rig);
        std::string error;
        const bool ok = RigExecBakeStaticReport(evaluator, &entries, &error);
        CHECK(ok);
        if (!ok) {
            std::printf("  static report: %s\n", error.c_str());
        }
        if (ok && evaluator.GetBakedProgram()) {
            const RigExecBakedProgramImpl &B =
                evaluator.GetBakedProgram()->GetStepGraph();
            for (const auto &chain : B.chains) {
                for (const auto &revision : chain.revisions) {
                    for (const auto &channel : revision.blendChannels) {
                        for (const auto &sample : channel.samples) {
                            bound += !sample.phase.IsBase() &&
                                     sample.pointBinding.id >= 0 &&
                                     !sample.pointBinding.candidates.empty();
                        }
                    }
                }
            }
        }
    }
    stage->GetSessionLayer()->Clear();
    bool chainBase = false;
    size_t samplePoints = 0;
    for (const RigExecBakeStaticEntry &entry : entries) {
        std::printf("  static %s: %s\n", entry.field.c_str(),
                    entry.source.c_str());
        chainBase = chainBase ||
                    (entry.field == "chain base " + shape.GetString() &&
                     entry.source == shape.GetString());
        samplePoints += entry.field.rfind("blend sample points ", 0) == 0;
    }
    CHECK(bound == 2);
    CHECK(!chainBase);
    CHECK(samplePoints == 0);
    std::printf("static report: %zu phased sample(s) bound to a version, "
                "the chain base %s, %zu sample point source(s) named\n",
                bound, chainBase ? "named" : "NOT named", samplePoints);
}

/// A presentation with one control naming \p input, finished with
/// \p identifier.
static std::vector<uint8_t>
_PresentationBytes(const std::string &input,
                   const char *identifier = fb::PresentationIdentifier())
{
    flatbuffers::FlatBufferBuilder builder;
    const auto control = fb::CreatePresentationControl(
        builder, builder.CreateString("Tail.rz"),
        builder.CreateString(input), fb::PresentationUnit::Degrees);
    const std::vector<flatbuffers::Offset<fb::PresentationControl>> controls(
        1, control);
    const auto presentation = fb::CreatePresentation(
        builder, 1, builder.CreateVector(controls), 0,
        builder.CreateString("{}"));
    builder.Finish(presentation, identifier);
    return std::vector<uint8_t>(builder.GetBufferPointer(),
                                builder.GetBufferPointer() +
                                    builder.GetSize());
}

// The bake's options: a time must be finite; a presentation whose control
// names a listed input is embedded as the file's presentation byte for
// byte, and one naming an input the file does not list, or with another
// identifier, refuses the bake.
static void
TestBakeOptions(const std::string &fixture, const std::string &input)
{
    const UsdStageRefPtr stage = UsdStage::Open(fixture);
    CHECK(stage);
    if (!stage) {
        return;
    }
    const SdfPath rigPath = FindRig(stage);
    const auto bake = [&](const std::vector<uint8_t> &presentation,
                          RigExecBakeResult *result, std::string *error) {
        RigExecRigEvaluator evaluator(stage, rigPath);
        RigExecBakeOpts opts;
        opts.presentation = presentation;
        return RigExecBakeToBinary(evaluator, opts, result, error);
    };
    const auto embedded = [](const std::vector<uint8_t> &bytes) {
        std::unique_ptr<fb::RigExecWireFile> file;
        std::string error;
        CHECK(RigExecFormatOpen(bytes.data(), bytes.size(), &file, &error));
        return file ? file->presentation : std::vector<uint8_t>();
    };
    RigExecBakeResult result;
    std::string error;
    for (const double time : {std::numeric_limits<double>::infinity(),
                              -std::numeric_limits<double>::infinity()}) {
        RigExecRigEvaluator evaluator(stage, rigPath);
        RigExecBakeOpts opts;
        opts.time = time;
        error.clear();
        CHECK(!RigExecBakeToBinary(evaluator, opts, &result, &error));
        CHECK(error == "the bake time must be finite");
    }
    CHECK(bake({}, &result, &error));
    CHECK(embedded(result.bytes).empty());

    const std::vector<uint8_t> valid = _PresentationBytes(input);
    CHECK(bake(valid, &result, &error));
    if (!error.empty()) {
        std::printf("presentation bake diagnostic: %s\n", error.c_str());
    }
    CHECK(embedded(result.bytes) == valid);

    error.clear();
    CHECK(!bake(_PresentationBytes("/RigExecTest/NoSuch.attr"), &result,
                &error));
    CHECK(error == "presentation control Tail.rz names "
                   "/RigExecTest/NoSuch.attr, which is not a listed input");
    std::printf("presentation, unknown input: %s\n", error.c_str());

    error.clear();
    CHECK(!bake(_PresentationBytes(input, "XXXX"), &result, &error));
    CHECK(error == "the presentation is not a valid REXP buffer");
    std::printf("presentation, bad identifier: %s\n", error.c_str());
}

// --- Format 20: export roles and constants (D2 b) --------------------------

/// The indices the group-roles fixture's sparse weight names: points
/// [0, 100) and [N/2, N/2 + 50).
static std::vector<int>
_GroupRolesSupport(size_t points)
{
    std::vector<int> support;
    for (int i = 0; i < 100; ++i) {
        support.push_back(i);
    }
    for (int i = 0; i < 50; ++i) {
        support.push_back(int(points / 2) + i);
    }
    return support;
}

/// A chain of \p points points (10000: past the default chunk vertex
/// target, cut into G vertex groups at the default group target) under four
/// movers, authored so the chain runs Blend, Linear, Gated, Dual (siblings
/// run bottom to top): a target-space blend shape at full envelope; a
/// classicLinear skin and a dualQuaternion skin, each binding points
/// [0, N/2) to J0 and the rest to J1; and a matrix mover weighted by a
/// static sparse weight at a zero default naming _GroupRolesSupport only.
/// Every value is authored at Default; nothing is connected.
static UsdStageRefPtr
_MakeGroupRolesStage(size_t points)
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const auto control = [&](const char *path, const char *avar,
                             double value) {
        const UsdPrim prim =
            stage->DefinePrim(SdfPath(path), TfToken("RigExecControl"));
        prim.CreateAttribute(TfToken(avar), SdfValueTypeNames->Double)
            .Set(value);
        return prim;
    };
    const UsdPrim j0 = control("/Asset/Rig/J0", "avars:tx", 2.0);
    const UsdPrim j1 = control("/Asset/Rig/J1", "avars:ty", 3.0);
    const UsdPrim moving = control("/Asset/Rig/Moving", "avars:tx", 1.0);
    const SdfPath target("/Asset/Shape.points");
    VtVec3fArray base(points);
    for (size_t i = 0; i < points; ++i) {
        base[i] = GfVec3f(float(i % 100), float(i / 100), 0.0f);
    }
    stage->DefinePrim(target.GetPrimPath(), TfToken("Points"))
        .CreateAttribute(TfToken("points"), SdfValueTypeNames->Point3fArray)
        .Set(base);
    // The blend shape's target: every point raised by one.
    VtVec3fArray raised = base;
    for (GfVec3f &point : raised) {
        point[2] += 1.0f;
    }
    stage->DefinePrim(SdfPath("/Asset/Raised"), TfToken("Points"))
        .CreateAttribute(TfToken("points"), SdfValueTypeNames->Point3fArray)
        .Set(raised);
    const UsdPrim input = stage->DefinePrim(
        SdfPath("/Asset/Rig/BlendInputs/Raise"), TfToken("RigExecBlendInput"));
    input.CreateAttribute(TfToken("inputs:weight"), SdfValueTypeNames->Float)
        .Set(0.5f);
    const UsdPrim sample = stage->DefinePrim(
        SdfPath("/Asset/Rig/BlendInputs/Raise/Full"),
        TfToken("RigExecBlendSample"));
    sample.CreateAttribute(TfToken("rigExec:activation"),
                           SdfValueTypeNames->Float)
        .Set(1.0f);
    sample.CreateRelationship(TfToken("rigExec:targetPoints"))
        .SetTargets({SdfPath("/Asset/Raised.points")});
    input.CreateRelationship(TfToken("rigExec:samples"))
        .SetTargets({sample.GetPath()});
    const std::vector<int> support = _GroupRolesSupport(points);
    VtIntArray named(support.size());
    for (size_t i = 0; i < support.size(); ++i) {
        named[i] = support[i];
    }
    const UsdPrim weight = stage->DefinePrim(
        SdfPath("/Asset/Rig/Weights/Sparse"), TfToken("RigExecStaticWeight"));
    weight.CreateRelationship(TfToken("rigExec:weightTarget"), false)
        .SetTargets({target});
    weight.CreateAttribute(TfToken("rigExec:representation"),
                           SdfValueTypeNames->Token, false)
        .Set(TfToken("sparse"));
    weight.CreateAttribute(TfToken("rigExec:indices"),
                           SdfValueTypeNames->IntArray, false)
        .Set(named);
    weight.CreateAttribute(TfToken("rigExec:values"),
                           SdfValueTypeNames->FloatArray, false)
        .Set(VtFloatArray(support.size(), 1.0f));
    weight.CreateAttribute(TfToken("rigExec:defaultWeight"),
                           SdfValueTypeNames->Float, false)
        .Set(0.0f);
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const auto mover = [&](const char *name, const char *type) {
        const UsdPrim prim = stage->DefinePrim(
            SdfPath(std::string("/Asset/Rig/Movers/") + name), TfToken(type));
        CHECK(prim.ApplyAPI(TfToken("RigExecMoverAPI")));
        prim.CreateRelationship(TfToken("rigExec:moves")).SetTargets({target});
        return prim;
    };
    const auto skin = [&](const char *name, const char *method) {
        const UsdPrim prim = mover(name, "RigExecSkinMover");
        prim.CreateAttribute(TfToken("inputs:defaultWeight"),
                             SdfValueTypeNames->Float)
            .Set(1.0f);
        prim.CreateRelationship(TfToken("rigExec:influences"))
            .SetTargets({j0.GetPath(), j1.GetPath()});
        prim.CreateAttribute(TfToken("rigExec:elementSize"),
                             SdfValueTypeNames->Int)
            .Set(1);
        VtIntArray indices(points);
        for (size_t i = 0; i < points; ++i) {
            indices[i] = i < points / 2 ? 0 : 1;
        }
        prim.CreateAttribute(TfToken("rigExec:jointIndices"),
                             SdfValueTypeNames->IntArray)
            .Set(indices);
        prim.CreateAttribute(TfToken("rigExec:jointWeights"),
                             SdfValueTypeNames->FloatArray)
            .Set(VtFloatArray(points, 1.0f));
        prim.CreateAttribute(TfToken("rigExec:skinningMethod"),
                             SdfValueTypeNames->Token)
            .Set(TfToken(method));
    };
    skin("Dual", "dualQuaternion");
    const UsdPrim gated = mover("Gated", "RigExecMatrixMover");
    gated.CreateRelationship(TfToken("rigExec:transform"))
        .SetTargets({moving.GetPath()});
    gated.CreateRelationship(TfToken("rigExec:weightObject"), false)
        .SetTargets({weight.GetPath()});
    skin("Linear", "classicLinear");
    const UsdPrim blend = mover("Blend", "RigExecBlendShapeMover");
    blend.CreateRelationship(TfToken("rigExec:blendInputs"))
        .SetTargets({input.GetPath()});
    blend.CreateAttribute(TfToken("inputs:defaultWeight"),
                          SdfValueTypeNames->Float)
        .Set(1.0f);
    return stage;
}

/// The group-roles chain of a baked file: its index, group count and each
/// mover's revision index, found by path; -1 when absent.
struct _GroupRolesChain {
    int chain = -1;
    size_t groups = 0;
    int blend = -1;
    int linear = -1;
    int gated = -1;
    int dual = -1;
    bool Found() const
    {
        return chain >= 0 && blend >= 0 && linear >= 0 && gated >= 0 &&
               dual >= 0;
    }
};

static _GroupRolesChain
_FindGroupRolesChain(const fb::RigExecWireFile &file)
{
    _GroupRolesChain at;
    const fb::RigExecWireDomainGeometry &g = *file.geometry;
    for (size_t c = 0; c < g.chains.size(); ++c) {
        if (RigExecFormatPathText(file, g.chains[c].target) ==
            "/Asset/Shape.points") {
            at.chain = int(c);
        }
    }
    if (at.chain < 0) {
        return at;
    }
    at.groups = RigExecFormatChainGroups(file, size_t(at.chain));
    const fb::RigExecWireChain &chain = g.chains[size_t(at.chain)];
    for (size_t r = 0; r < chain.revisions.size(); ++r) {
        const std::string mover =
            RigExecFormatPathText(file, chain.revisions[r].moverPath);
        if (mover == "/Asset/Rig/Movers/Blend") {
            at.blend = int(r);
        } else if (mover == "/Asset/Rig/Movers/Linear") {
            at.linear = int(r);
        } else if (mover == "/Asset/Rig/Movers/Gated") {
            at.gated = int(r);
        } else if (mover == "/Asset/Rig/Movers/Dual") {
            at.dual = int(r);
        }
    }
    return at;
}

/// The groups of \p groups over \p points points that hold one of
/// \p indices.
static std::set<int>
_GroupsHolding(const std::vector<int> &indices, size_t points, size_t groups)
{
    std::set<int> held;
    for (const int i : indices) {
        for (size_t k = 0; k < groups; ++k) {
            if (RigExecPointRangeBound(points, groups, k) <= size_t(i) &&
                size_t(i) < RigExecPointRangeBound(points, groups, k + 1)) {
                held.insert(int(k));
            }
        }
    }
    return held;
}

/// The parts of revision \p r of chain \p c's RevisionChunk steps.
static std::set<int>
_ChunkParts(const fb::RigExecWireFile &file, size_t c, int r)
{
    std::set<int> parts;
    const int32_t id = file.geometry->chainRevisionBegin[c] + int32_t(r);
    for (const fb::RigExecWireStep &step : file.steps) {
        if (step.kind == fb::StepKind::RevisionChunk && step.object == id) {
            parts.insert(int(step.part));
        }
    }
    return parts;
}

/// Revision \p r of chain \p c's revision_chunk_count, or 0.
static size_t
_RevisionChunkCount(const fb::RigExecWireFile &file, size_t c, int r)
{
    const fb::RigExecWireDomainGeometry &g = *file.geometry;
    const size_t id = size_t(g.chainRevisionBegin[c]) + size_t(r);
    return id < g.revisionChunkCount.size()
               ? size_t(g.revisionChunkCount[id])
               : 0;
}

/// Whether \p file lists the input slot of attribute \p name.
static bool
_ListsInput(const fb::RigExecWireFile &file, const std::string &name)
{
    const int64_t slot = RigExecTestSlotOf(file, name);
    return slot >= 0 && uint64_t(slot) < uint64_t(file.listedInputs);
}

/// Whether \p file holds attribute \p name as a constant: a private slot
/// that is not Animated, whose default the runtime reads.
static bool
_HoldsConstant(const fb::RigExecWireFile &file, const std::string &name)
{
    const int64_t slot = RigExecTestSlotOf(file, name);
    if (slot < 0 || uint64_t(slot) < uint64_t(file.listedInputs)) {
        return false;
    }
    const uint8_t flags = file.inputs[size_t(slot)].flags();
    return (flags & uint8_t(fb::InputSlotFlags::Listed)) == 0 &&
           (flags & uint8_t(fb::InputSlotFlags::Animated)) == 0;
}

/// The default of attribute \p name's slot in \p file, or null.
static const fb::RigExecWireValue *
_SlotDefault(const fb::RigExecWireFile &file, const std::string &name)
{
    const int64_t slot = RigExecTestSlotOf(file, name);
    if (slot < 0 || file.inputs[size_t(slot)].value() >= file.values.size()) {
        return nullptr;
    }
    return &file.values[file.inputs[size_t(slot)].value()];
}

/// Moves the private slot of attribute \p name into \p file's listed prefix
/// at its place in the path order, renumbering every slot id the file
/// holds; false when \p file has no private slot for it. Every other rule
/// on the listed prefix keeps holding.
static bool
_ListSlot(fb::RigExecWireFile *file, const std::string &name)
{
    const int64_t found = RigExecTestSlotOf(*file, name);
    if (found < 0 || uint64_t(found) < uint64_t(file->listedInputs)) {
        return false;
    }
    const uint32_t from = uint32_t(found);
    uint32_t at = 0;
    while (at < file->listedInputs &&
           RigExecFormatPathText(*file, file->inputs[at].name()) < name) {
        ++at;
    }
    RigExecTestForEachSlotId(file, [from, at](uint32_t *slot) {
        if (*slot == from) {
            *slot = at;
        } else if (*slot >= at && *slot < from) {
            ++*slot;
        }
    });
    const fb::InputSlot moved = file->inputs[from];
    file->inputs.erase(file->inputs.begin() + std::ptrdiff_t(from));
    file->inputs.insert(
        file->inputs.begin() + std::ptrdiff_t(at),
        fb::InputSlot(moved.name(), moved.value(), moved.chain(),
                      moved.phased(), moved.type(),
                      uint8_t(moved.flags() |
                              uint8_t(fb::InputSlotFlags::Listed))));
    ++file->listedInputs;
    return true;
}

// The group-roles fixture baked at frame 1 (format 20, D2 b), from the
// export program the bake built. The file holds its roles: the blend shape
// and the matrix mover Range with no keys; the classicLinear skin a Range
// Skin, unchunked with its G groups keyed by their influences; the
// dualQuaternion skin Whole, chunked with G keyed groups and G published
// groups more; the matrix mover gated, with group steps only for the groups
// its weight names, so the groups it leaves are entered from the revision
// before it. The reads the Range skin and the gate rest on -- the skin's
// skinningMethod, elementSize and jointIndices and the weight's
// defaultWeight -- are constants: private slots holding their bake-time
// values. The skin's jointWeights and the Whole skin's same reads stay
// listed. Edited copies are refused by the validator: a Whole revision
// counting only its chunks, a Range skin group's key changed, and either
// constant moved into the listed inputs.
static void
TestGroupRolesBake()
{
    constexpr size_t kPoints = 10000;
    const UsdStageRefPtr stage = _MakeGroupRolesStage(kPoints);
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    const std::unique_ptr<fb::RigExecWireFile> file =
        _BakeAndCompare(evaluator, 1.0, "group roles bake");
    const RigExecBakedProgram *baked = evaluator.GetBakedProgram();
    if (!file || !baked) {
        return;
    }
    const RigExecBakedProgramImpl &program = baked->GetStepGraph();
    CHECK(program.roleMode == RigExecBakedRoleMode::Export);
    const _GroupRolesChain at = _FindGroupRolesChain(*file);
    CHECK(at.Found());
    if (!at.Found()) {
        return;
    }
    const size_t c = size_t(at.chain);
    const size_t groups = at.groups;
    const fb::RigExecWireChain &chain = file->geometry->chains[c];
    CHECK(chain.revisions.size() == 4);
    CHECK(groups >= 2);
    CHECK(c < program.chains.size() &&
          program.chains[c].groupBounds.size() == groups + 1 &&
          program.chains[c].revisions.size() == chain.revisions.size());
    if (groups < 2 || c >= program.chains.size() ||
        program.chains[c].revisions.size() != chain.revisions.size()) {
        return;
    }
    const auto &native = program.chains[c].revisions;
    std::set<int> every;
    for (size_t k = 0; k < groups; ++k) {
        every.insert(int(k));
    }
    const auto onBounds = [&](const fb::RigExecWireRevision &revision) {
        bool same = revision.chunks.size() == groups;
        for (size_t k = 0; same && k < groups; ++k) {
            same = uint64_t(revision.chunks[k].begin) ==
                       RigExecPointRangeBound(kPoints, groups, k) &&
                   uint64_t(revision.chunks[k].end) ==
                       RigExecPointRangeBound(kPoints, groups, k + 1);
        }
        return same;
    };

    // Range, no keys: the blend shape on every group, the matrix mover on
    // the groups its weight names.
    for (const int r : {at.blend, at.gated}) {
        const fb::RigExecWireRevision &revision = chain.revisions[size_t(r)];
        CHECK(native[size_t(r)].role == RigExecBakedRevisionRole::Range);
        CHECK(RigExecFormatIsRangeRevision(revision));
        CHECK(!RigExecFormatIsKeyedRevision(revision));
        CHECK(onBounds(revision));
        for (const fb::RigExecWireChunk &chunk : revision.chunks) {
            CHECK(chunk.key.empty());
        }
        CHECK(_RevisionChunkCount(*file, c, r) == groups);
    }
    CHECK(chain.revisions[size_t(at.blend)].op ==
          uint8_t(fb::RevisionOp::BlendShape));
    CHECK(chain.revisions[size_t(at.gated)].op ==
          uint8_t(fb::RevisionOp::Matrix));
    CHECK(_ChunkParts(*file, c, at.blend) == every);
    const std::set<int> written =
        _GroupsHolding(_GroupRolesSupport(kPoints), kPoints, groups);
    CHECK(!written.empty() && written.size() < groups);
    CHECK(_ChunkParts(*file, c, at.gated) == written);

    // The Range skin: unchunked, its chunks the groups, keyed.
    const fb::RigExecWireRevision &linear = chain.revisions[size_t(at.linear)];
    CHECK(native[size_t(at.linear)].role == RigExecBakedRevisionRole::Range);
    CHECK(linear.op == uint8_t(fb::RevisionOp::Skin));
    CHECK(!linear.chunked && RigExecFormatIsRangeRevision(linear));
    CHECK(RigExecFormatIsKeyedRevision(linear));
    CHECK(onBounds(linear));
    CHECK(_RevisionChunkCount(*file, c, at.linear) == groups);
    CHECK(_ChunkParts(*file, c, at.linear) == every);

    // The Whole skin: chunked, G keyed chunks, and its fuse's G groups.
    const fb::RigExecWireRevision &dual = chain.revisions[size_t(at.dual)];
    CHECK(native[size_t(at.dual)].role == RigExecBakedRevisionRole::Whole);
    CHECK(dual.op == uint8_t(fb::RevisionOp::Skin));
    CHECK(dual.chunked && !RigExecFormatIsRangeRevision(dual));
    CHECK(RigExecFormatIsKeyedRevision(dual));
    CHECK(onBounds(dual));
    CHECK(_RevisionChunkCount(*file, c, at.dual) ==
          dual.chunks.size() + groups);
    CHECK(_ChunkParts(*file, c, at.dual) == every);

    // Both skins' keys are their groups' influences: J0 (0) below N/2, J1
    // (1) from there.
    for (const fb::RigExecWireRevision *skin : {&linear, &dual}) {
        for (size_t k = 0; k < skin->chunks.size() && k < groups; ++k) {
            std::set<int> want;
            if (RigExecPointRangeBound(kPoints, groups, k) < kPoints / 2) {
                want.insert(0);
            }
            if (RigExecPointRangeBound(kPoints, groups, k + 1) >
                kPoints / 2) {
                want.insert(1);
            }
            CHECK(std::set<int>(skin->chunks[k].key.begin(),
                                skin->chunks[k].key.end()) == want);
        }
    }

    // Each group's writers as Open reads them: every revision writes every
    // group but the gated one, and a group enters from its last writer.
    std::vector<std::vector<char>> writes;
    std::vector<std::vector<int>> entering;
    RigExecFormatGroupWriters(*file, c, &writes, &entering);
    CHECK(writes.size() == chain.revisions.size() &&
          entering.size() == chain.revisions.size());
    for (size_t r = 0; r < writes.size() && r < entering.size(); ++r) {
        CHECK(writes[r].size() == groups && entering[r].size() == groups);
        for (size_t k = 0;
             k < groups && k < writes[r].size() && k < entering[r].size();
             ++k) {
            const auto writer = [&](size_t q) {
                return int(q) != at.gated || written.count(int(k)) != 0;
            };
            int want = -1;
            for (size_t q = 0; q < r; ++q) {
                if (writer(q)) {
                    want = int(q);
                }
            }
            CHECK((writes[r][k] != 0) == writer(r));
            CHECK(entering[r][k] == want);
        }
    }
    {
        std::string why;
        const bool valid = RigExecFormatValidate(*file, &why);
        if (!valid) {
            std::printf("  group roles bake refused: %s\n", why.c_str());
        }
        CHECK(valid);
    }

    // The constants: private, not animated, their bake-time values; exactly
    // the reads the export program pinned are among them.
    const std::string linearAt = "/Asset/Rig/Movers/Linear.";
    const std::string dualAt = "/Asset/Rig/Movers/Dual.";
    const std::string defaultWeight =
        "/Asset/Rig/Weights/Sparse.rigExec:defaultWeight";
    const std::set<SdfPath> &pinned = baked->GetExportPinnedPaths();
    for (const std::string &name :
         {linearAt + "rigExec:skinningMethod",
          linearAt + "rigExec:elementSize", linearAt + "rigExec:jointIndices",
          defaultWeight}) {
        const bool constant = _HoldsConstant(*file, name);
        if (!constant) {
            std::printf("  group roles bake: %s is not a constant\n",
                        name.c_str());
        }
        CHECK(constant);
        CHECK(pinned.count(SdfPath(name)) == 1);
    }
    for (const SdfPath &path : pinned) {
        CHECK(!_ListsInput(*file, path.GetString()));
    }
    for (const std::string &name :
         {linearAt + "rigExec:jointWeights", dualAt + "rigExec:skinningMethod",
          dualAt + "rigExec:elementSize", dualAt + "rigExec:jointIndices"}) {
        const bool listed = _ListsInput(*file, name);
        if (!listed) {
            std::printf("  group roles bake: %s is not listed\n",
                        name.c_str());
        }
        CHECK(listed);
    }
    const fb::RigExecWireValue *method =
        _SlotDefault(*file, linearAt + "rigExec:skinningMethod");
    CHECK(method && method->tag == fb::InputTag::Token &&
          method->bits <= 0xffffffffull &&
          _BinaryText(*file, uint32_t(method->bits)) == "classicLinear");
    const fb::RigExecWireValue *elementSize =
        _SlotDefault(*file, linearAt + "rigExec:elementSize");
    CHECK(elementSize && elementSize->tag == fb::InputTag::Int &&
          elementSize->bits == 1);
    const fb::RigExecWireValue *zero = _SlotDefault(*file, defaultWeight);
    CHECK(zero && zero->tag == fb::InputTag::Float &&
          zero->bits == _BinaryBits(0.0f));
    CHECK(linear.jointIndicesSlot >= 0 &&
          uint64_t(linear.jointIndicesSlot) >= uint64_t(file->listedInputs));
    CHECK(linear.jointWeightsSlot >= 0 &&
          uint64_t(linear.jointWeightsSlot) < uint64_t(file->listedInputs));

    // The edits, each refused by the validator.
    int refused = 0;
    const auto refuse =
        [&](const std::string &label,
            const std::function<bool(fb::RigExecWireFile &)> &edit) {
            fb::RigExecWireFile copy(*file);
            const bool edited = edit(copy);
            CHECK(edited);
            std::string why;
            const bool rejected = edited && !RigExecFormatValidate(copy, &why);
            std::printf("  group roles bake, %s: %s\n", label.c_str(),
                        rejected ? why.c_str() : "ACCEPTED");
            CHECK(rejected);
            refused += rejected ? 1 : 0;
            return why;
        };
    refuse("a Whole revision counting only its chunks",
           [&](fb::RigExecWireFile &f) {
               const size_t id =
                   size_t(f.geometry->chainRevisionBegin[c]) + size_t(at.dual);
               if (id >= f.geometry->revisionChunkCount.size()) {
                   return false;
               }
               f.geometry->revisionChunkCount[id] =
                   int32_t(dual.chunks.size());
               return true;
           });
    refuse("a Range skin group's key changed", [&](fb::RigExecWireFile &f) {
        std::vector<fb::RigExecWireChunk> &chunks =
            f.geometry->chains[c].revisions[size_t(at.linear)].chunks;
        if (chunks.empty()) {
            return false;
        }
        // Group 0 holds J0's points only; claim J1's instead.
        chunks[0].key = {1};
        return true;
    });
    for (const std::string &name :
         {linearAt + "rigExec:skinningMethod", defaultWeight}) {
        const std::string why =
            refuse("listing " + name, [&](fb::RigExecWireFile &f) {
                return _ListSlot(&f, name);
            });
        // Refused for the constant, not for how the edit moved the slot.
        CHECK(!_Contains(why, "malformed type, flags or default") &&
              !_Contains(why, "listed inputs are not sorted"));
    }
    std::printf("group roles bake: %zu groups; gated mover writes %zu; %zu "
                "constants; %d edits refused\n",
                groups, written.size(), pinned.size(), refused);
    CHECK(refused == 4);
}

// The bake's keep-set (D2 b): an admitted upstream input and a presentation
// input stay listed, so the export program rests no Range role or gate on
// them, and only that one changes. With the classicLinear skin's
// skinningMethod admitted upstream before the bake, the bake succeeds and
// reports it, the file lists it, and holds that skin Whole (chunked, its
// fuse's G groups more) with its layout reads listed, while the gate
// stands; the upstream input stands again after the bake. With a
// presentation naming the weight's defaultWeight, the file lists it and the
// matrix mover writes every group, while the skin stays a Range Skin.
static void
TestExportKeepSetBake()
{
    constexpr size_t kPoints = 10000;
    const std::string linearAt = "/Asset/Rig/Movers/Linear.";
    const std::string method = linearAt + "rigExec:skinningMethod";
    const std::string defaultWeight =
        "/Asset/Rig/Weights/Sparse.rigExec:defaultWeight";
    const auto bake = [&](RigExecRigEvaluator &evaluator,
                          const RigExecBakeOpts &opts,
                          RigExecBakeResult *result,
                          const std::string &label) {
        std::string error;
        const bool baked =
            RigExecBakeToBinary(evaluator, opts, result, &error);
        if (!baked) {
            std::printf("%s bake diagnostic: %s\n", label.c_str(),
                        error.c_str());
        }
        CHECK(baked);
        return baked ? _BinaryCompareProgram(evaluator, result->bytes, label)
                     : std::unique_ptr<fb::RigExecWireFile>();
    };

    {
        const UsdStageRefPtr stage = _MakeGroupRolesStage(kPoints);
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        RigExecValueOverride upstream;
        upstream.prim = SdfPath("/Asset/Rig/Movers/Linear");
        upstream.attribute = TfToken("rigExec:skinningMethod");
        upstream.value = VtValue(TfToken("classicLinear"));
        evaluator.SetUpstreamInputs({upstream});
        CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);
        const std::vector<SdfPath> admitted{SdfPath(method)};
        CHECK(evaluator.GetUpstreamInputPaths() == admitted);
        RigExecBakeOpts opts;
        opts.time = 1.0;
        RigExecBakeResult result;
        const std::unique_ptr<fb::RigExecWireFile> file =
            bake(evaluator, opts, &result, "upstream keep-set bake");
        CHECK(result.upstreamInputs == std::vector<std::string>{method});
        CHECK(evaluator.GetUpstreamInputPaths() == admitted);
        const _GroupRolesChain at =
            file ? _FindGroupRolesChain(*file) : _GroupRolesChain();
        CHECK(at.Found());
        if (file && at.Found()) {
            const size_t c = size_t(at.chain);
            const fb::RigExecWireRevision &skin =
                file->geometry->chains[c].revisions[size_t(at.linear)];
            CHECK(skin.chunked && !RigExecFormatIsRangeRevision(skin));
            CHECK(_RevisionChunkCount(*file, c, at.linear) ==
                  skin.chunks.size() + at.groups);
            CHECK(_ListsInput(*file, method));
            CHECK(_ListsInput(*file, linearAt + "rigExec:elementSize"));
            CHECK(_ListsInput(*file, linearAt + "rigExec:jointIndices"));
            CHECK(_HoldsConstant(*file, defaultWeight));
            CHECK(_ChunkParts(*file, c, at.gated) ==
                  _GroupsHolding(_GroupRolesSupport(kPoints), kPoints,
                                 at.groups));
            std::printf("upstream keep-set bake: the skin is Whole and its "
                        "method listed\n");
        }
    }

    {
        const UsdStageRefPtr stage = _MakeGroupRolesStage(kPoints);
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);
        // A Live program lists the weight's default, so a presentation may
        // name it.
        CHECK(evaluator.GetBakedProgram() &&
              evaluator.GetBakedProgram()->GetUpstreamAdmissible().count(
                  SdfPath(defaultWeight)) == 1);
        RigExecBakeOpts opts;
        opts.time = 1.0;
        opts.presentation = _PresentationBytes(defaultWeight);
        RigExecBakeResult result;
        const std::unique_ptr<fb::RigExecWireFile> file =
            bake(evaluator, opts, &result, "presentation keep-set bake");
        const _GroupRolesChain at =
            file ? _FindGroupRolesChain(*file) : _GroupRolesChain();
        CHECK(at.Found());
        if (file && at.Found()) {
            const size_t c = size_t(at.chain);
            const fb::RigExecWireChain &chain = file->geometry->chains[c];
            CHECK(file->presentation == opts.presentation);
            CHECK(_ListsInput(*file, defaultWeight));
            std::set<int> every;
            for (size_t k = 0; k < at.groups; ++k) {
                every.insert(int(k));
            }
            CHECK(at.groups >= 2);
            CHECK(RigExecFormatIsRangeRevision(
                chain.revisions[size_t(at.gated)]));
            CHECK(_ChunkParts(*file, c, at.gated) == every);
            const fb::RigExecWireRevision &skin =
                chain.revisions[size_t(at.linear)];
            CHECK(!skin.chunked && RigExecFormatIsRangeRevision(skin));
            CHECK(_HoldsConstant(*file, method));
            std::printf("presentation keep-set bake: the mover is ungated "
                        "and its weight's default listed\n");
        }
    }
}

// The bake holds the evaluator in the export role mode for its own length
// only: after a bake, and after one that fails past the export build (a
// presentation naming no listed input), the evaluator is back in Live mode
// with no keep-set; the export program stands until the next Evaluate,
// which rebuilds once, in Live mode, and the one after it rebuilds nothing.
static void
TestBakeRestoresLiveRoles()
{
    const UsdStageRefPtr stage = _MakeGroupRolesStage(10000);
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    CHECK(evaluator.GetBakedRoleMode() == RigExecBakedRoleMode::Live);
    const auto restored = [&](const char *label) {
        CHECK(evaluator.GetBakedRoleMode() == RigExecBakedRoleMode::Live);
        CHECK(evaluator.GetBakedExportKeep().empty());
        const RigExecBakedProgram *exported = evaluator.GetBakedProgram();
        CHECK(exported &&
              exported->GetStepGraph().roleMode ==
                  RigExecBakedRoleMode::Export &&
              !exported->GetExportPinnedPaths().empty());
        const size_t builds = evaluator.GetBakedProgramBuildCount();
        CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);
        const bool rebuilt =
            evaluator.GetBakedProgramBuildCount() == builds + 1;
        const RigExecBakedProgram *live = evaluator.GetBakedProgram();
        const bool isLive =
            live &&
            live->GetStepGraph().roleMode == RigExecBakedRoleMode::Live &&
            live->GetExportPinnedPaths().empty();
        CHECK(evaluator.Evaluate(UsdTimeCode(1.0)).valid);
        const bool held = evaluator.GetBakedProgramBuildCount() == builds + 1;
        std::printf("bake role restore %s: %s\n", label,
                    rebuilt && isLive && held ? "one rebuild, Live"
                                              : "WRONG");
        CHECK(rebuilt && isLive && held);
    };
    RigExecBakeOpts opts;
    opts.time = 1.0;
    RigExecBakeResult result;
    std::string error;
    const bool baked = RigExecBakeToBinary(evaluator, opts, &result, &error);
    if (!baked) {
        std::printf("bake role restore diagnostic: %s\n", error.c_str());
    }
    CHECK(baked);
    restored("after a bake");

    opts.presentation = _PresentationBytes("/Asset/Rig/NoSuch.inputs:x");
    error.clear();
    CHECK(!RigExecBakeToBinary(evaluator, opts, &result, &error));
    CHECK(error == "presentation control Tail.rz names "
                   "/Asset/Rig/NoSuch.inputs:x, which is not a listed input");
    restored("after a failed bake");
}

// An export is lowered for the reference concurrency whatever the baking
// machine's work limit. The fixture is the first whose
// serial cost gives different grains at one and two workers, so a bake that
// followed the limit would differ under the two.
static void
TestTheExportIgnoresTheWorkLimit()
{
    if (!TfGetenv("RIGEXEC_BAKED_GRAIN_US", "").empty()) {
        std::printf("export work limit: skipped, RIGEXEC_BAKED_GRAIN_US is set\n");
        return;
    }
    struct RestoreLimit {
        unsigned limit;
        ~RestoreLimit() { WorkSetConcurrencyLimit(limit); }
    } restore{WorkGetConcurrencyLimit()};
    std::string chosen;
    double cost = 0;
    for (const char *name : {"02_TwoBoneIkLeg.usda", "09_PropertyMathMovers.usda",
                             "06_LatticeBulge.usda"}) {
        const std::string path =
            (std::filesystem::path(RIGEXEC_EXAMPLES_DIR) / name).string();
        const std::vector<uint8_t> bytes = _BakeFixture(path, 1001.0);
        std::unique_ptr<fb::RigExecWireFile> file;
        std::string why;
        if (bytes.empty() ||
            !RigExecFormatOpen(bytes.data(), bytes.size(), &file, &why) ||
            !file->clustering) {
            std::printf("  export work limit: %s does not open (%s)\n", name,
                        why.c_str());
            continue;
        }
        const double serial = file->clustering->serialCost;
        std::printf("  export work limit: %s serial cost %.2f us\n", name,
                    serial);
        if (RigExecBakedScheduleGrainUs(serial, 1) !=
            RigExecBakedScheduleGrainUs(serial, 2)) {
            chosen = path;
            cost = serial;
            break;
        }
    }
    CHECK(!chosen.empty());
    if (chosen.empty()) {
        return;
    }
    std::printf("export work limit: %s (serial cost %.2f us)\n", chosen.c_str(),
                cost);
    WorkSetConcurrencyLimit(1);
    const std::vector<uint8_t> one = _BakeFixture(chosen, 1001.0);
    WorkSetConcurrencyLimit(2);
    const std::vector<uint8_t> two = _BakeFixture(chosen, 1001.0);
    CHECK(!one.empty() && one == two);
    std::unique_ptr<fb::RigExecWireFile> file;
    std::string why;
    CHECK(RigExecFormatOpen(one.data(), one.size(), &file, &why));
    CHECK(file && file->clustering &&
          file->clustering->grainUs ==
              RigExecBakedScheduleGrainUs(cost,
                                          kRigExecBakedReferenceConcurrency));
}

// Format 9, examples/biped/Biped.usda at fixture time 1: 3,467,672 bytes,
// measured 2026-10-06 with head steps and memo bindings. Budget: plus 5%.
constexpr size_t kBipedBytes = 3467672;
constexpr size_t kBipedBudget = kBipedBytes + kBipedBytes / 20;

static bool sizeBudgetChecked = false;

/// The file's size against the biped budget, and what each root field
/// costs in the raw inner payload: the bytes a copy with that field emptied
/// packs smaller by. The budget applies to the published transport bytes.
static void
_BinarySizeBudget(const std::string &fixture, const RigExecBakeResult &result)
{
    if (!_EndsWith(_Slashed(fixture), "/biped/Biped.usda")) {
        return;
    }
    sizeBudgetChecked = true;
    const size_t bytes = result.bytes.size();
    CHECK(bytes <= kBipedBudget);
    std::printf("size budget %s: %zu bytes, budget %zu (measured %zu on "
                "2026-10-06, format 9)\n",
                fixture.c_str(), bytes, kBipedBudget, kBipedBytes);
    const std::unique_ptr<fb::RigExecWireFile> file =
        RigExecTestUnpack(result.bytes);
    if (!file) {
        return;
    }
    size_t rawBytes = bytes;
    rigExec::transport::Buffer decoded;
    if (rigExec::transport::IsEnvelope(result.bytes.data(), bytes)) {
        std::string error;
        const bool ready = rigExec::transport::Decode(
            result.bytes.data(), bytes, &decoded, &error);
        CHECK(ready);
        if (!ready) return;
        rawBytes = decoded.size;
    }
    CHECK(RigExecTestPackUnchecked(*file).size() == rawBytes);
    std::printf("  raw inner payload: %zu bytes\n", rawBytes);
    using F = fb::RigExecWireFile;
    const auto topologies = [](F &f) {
        const auto clear = [](fb::RigExecWireRevision &r) {
            r.topology.reset();
            r.partitionTopology.reset();
        };
        for (auto &chain : f.geometry->chains) {
            for (auto &r : chain.revisions) {
                clear(r);
            }
            for (auto &d : chain.derived) {
                if (d.revision) {
                    clear(*d.revision);
                }
            }
        }
    };
    const std::pair<const char *, std::function<void(F &)>> fields[] = {
        {"names+paths", [](F &f) { f.names.clear(); f.paths.clear(); }},
        {"values", [](F &f) { f.values.clear(); }},
        {"int_arrays", [](F &f) { f.intArrays.clear(); }},
        {"float_arrays", [](F &f) { f.floatArrays.clear(); }},
        {"double_arrays", [](F &f) { f.doubleArrays.clear(); }},
        {"vec2f_arrays", [](F &f) { f.vec2fArrays.clear(); }},
        {"vec3f_arrays", [](F &f) { f.vec3fArrays.clear(); }},
        {"inputs", [](F &f) { f.inputs.clear(); }},
        {"slot_meta+constants",
         [](F &f) {
             f.slotMeta = std::make_unique<fb::RigExecWireSlotMeta>();
             f.constants = std::make_unique<fb::RigExecWireConstants>();
         }},
        {"steps+clustering+cones",
         [](F &f) {
             f.steps.clear();
             f.clustering = std::make_unique<fb::RigExecWireClustering>();
             f.cones = std::make_unique<fb::RigExecWireCones>();
         }},
        {"  of which head memo",
         [](F &f) {
             for (auto &step : f.steps) {
                 step.headInputSlots.clear();
                 step.headInputReads.clear();
                 step.headVaryingLeaves = false;
                 step.headAlwaysRuns = false;
             }
         }},
        {"pose",
         [](F &f) { f.pose = std::make_unique<fb::RigExecWireDomainPose>(); }},
        {"geometry",
         [](F &f) {
             f.geometry = std::make_unique<fb::RigExecWireDomainGeometry>();
         }},
        {"  of which skin topologies", topologies},
        {"  of which path_reads",
         [](F &f) { f.geometry->pathReads.clear(); }},
        {"property_chains+phased_consumers",
         [](F &f) {
             f.propertyChains.clear();
             f.phasedConsumers.clear();
         }},
        {"external_movers", [](F &f) { f.externalMovers.clear(); }},
        {"compile_diagnostics", [](F &f) { f.compileDiagnostics.clear(); }},
        {"presentation", [](F &f) { f.presentation.clear(); }},
    };
    for (const auto &field : fields) {
        F copy(*file);
        field.second(copy);
        const size_t cleared = RigExecTestPackUnchecked(copy).size();
        CHECK(cleared <= rawBytes);
        std::printf("  size %-34s %10zu bytes\n", field.first,
                    rawBytes - std::min(rawBytes, cleared));
    }
}

static int compareRows = 0;
static int compareTried = 0;
static int classGuardStatic = 0;
static int classGuardInputs = 0;

static void
TestBake(const std::string &fixture, const std::vector<double> &tableFrames,
         const char *animation, const std::filesystem::path &scratch,
         const std::vector<uint8_t> *cliArtifact = nullptr)
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
    // The table's first frame, or the probe time without a table.
    const double bakeTime = tableFrames.empty()
                                ? std::numeric_limits<double>::quiet_NaN()
                                : tableFrames.front();
    const double t = std::isnan(bakeTime)
                         ? RigExecBakedProbeTime(stage).GetValue()
                         : bakeTime;
    // In the registered gate the CLI made the first fresh bake. This
    // independent evaluator makes the second, against the same authored
    // stage, root, time and inherited environment. Standalone calls still
    // make both fresh bakes here.
    std::vector<uint8_t> first = cliArtifact ? *cliArtifact
                                             : std::vector<uint8_t>();
    std::unique_ptr<fb::RigExecWireFile> file;
    for (int pass = 0; pass < (cliArtifact ? 1 : 2); ++pass) {
        RigExecRigEvaluator evaluator(stage, rigPath);
        RigExecBakeOpts opts;
        opts.time = bakeTime;
        RigExecBakeResult result;
        std::string error;
        CHECK(RigExecBakeToBinary(evaluator, opts, &result, &error));
        if (!error.empty()) {
            std::printf("bake diagnostic: %s\n", error.c_str());
        }
        if (result.bytes.empty()) {
            return;
        }
        if (pass == 1) {
            CHECK(result.bytes == first);
            continue;
        }
        if (cliArtifact) {
            CHECK(result.bytes == first);
        } else {
            first = result.bytes;
        }
        ++compareTried;
        const int failuresBefore = failures;
        _BinaryCompareStats stats;
        file = _BinaryCompareProgram(
            evaluator, cliArtifact ? first : result.bytes, fixture, &stats);
        if (!file) {
            return;
        }
        // The file is of the bake time, and its object form writes back to
        // the same bytes.
        CHECK(_BinarySame(file->bakeTime, t));
        std::vector<uint8_t> rewritten;
        CHECK(RigExecFormatWrite(*file, &rewritten, &error) &&
              rewritten == result.bytes);
        std::set<std::string> rowPaths;
        for (const fb::RigExecWirePathRead &row : file->geometry->pathReads) {
            rowPaths.insert(RigExecFormatPathText(*file, row.path));
        }
        _BinaryCheckWeightGatherReads(fixture, rowPaths);
        const int failed = failures - failuresBefore;
        std::printf("file %s: %zu bytes\n", fixture.c_str(),
                    result.bytes.size());
        std::printf("compare %s: %zu steps, %zu inputs (%zu defaults), %zu "
                    "reads, %zu override numbers, %zu path reads (%zu read "
                    "rows, %zu value rows, %zu enumerated keys), %zu "
                    "topologies (%zu trailing pads dropped), %zu blend "
                    "samples, %zu bases, %zu property chains, %zu oracle "
                    "facts: %d failures\n",
                    fixture.c_str(), file->steps.size(), file->inputs.size(),
                    stats.defaults, stats.reads, stats.overrideNumbers,
                    file->geometry->pathReads.size(), stats.readRows,
                    stats.valueRows, stats.enumeratedKeys, stats.topologies,
                    stats.droppedEntries, stats.samples, stats.bases,
                    stats.propertyChains, stats.oracleFacts, failed);
        std::printf("  array inputs: %zu (%zu live rows, %zu rest rows, %zu "
                    "blend samples, %zu layout pairs, %zu chain bases, %zu "
                    "painted arrays, %zu oracle points)\n",
                    stats.arrayInputs, stats.arrayRows, stats.arrayRestRows,
                    stats.arrayBlendSamples, stats.arrayLayouts,
                    stats.arrayBases, stats.arrayPainted, stats.arrayOracle);
        if (failed == 0) {
            ++compareRows;
        }
        _BinaryCheckInputs(evaluator.GetBakedProgram()->GetStepGraph(),
                           *file);
        _BinarySizeBudget(fixture, result);
        // The counts the bake reports: the file's rows, and the distinct
        // (path, rest) keys of the run's enumeration.
        CHECK(result.pathReadsWritten == file->geometry->pathReads.size());
        {
            std::vector<RigExecBakeRevisionRead> enumerated;
            RigExecBakeEnumerateProgramReads(
                evaluator.GetBakedProgram()->GetStepGraph(), t, &enumerated);
            std::set<std::pair<std::string, bool>> keys;
            for (const RigExecBakeRevisionRead &read : enumerated) {
                keys.emplace(read.path.GetString(), read.rest);
            }
            CHECK(result.pathReadsEnumerated == keys.size());
        }
        // The class guard: a stage classed static holds an animated source
        // in static data, one classed inputs holds none, so a class cannot
        // outlive its reason.
        if (animation) {
            std::vector<RigExecBakeStaticEntry> entries;
            CHECK(RigExecBakeStaticReport(evaluator, &entries, &error));
            for (const RigExecBakeStaticEntry &entry : entries) {
                std::printf("  static %s: %s\n", entry.field.c_str(),
                            entry.source.c_str());
            }
            const bool expectStatic = std::string(animation) == "static";
            const bool ok = expectStatic ? !entries.empty() : entries.empty();
            std::printf("class guard %s: %s, %zu animated static "
                        "source(s)%s\n",
                        fixture.c_str(), animation, entries.size(),
                        ok ? "" : " -- WRONG CLASS");
            CHECK(ok);
            if (ok) {
                ++(expectStatic ? classGuardStatic : classGuardInputs);
            }
        }
    }
    if (!file) {
        return;
    }
    // The bytes survive a trip through a file: the CLI writes exactly this
    // vector, so the vector is the format, not an in-memory sketch of it.
    std::string flat = fixture;
    for (char &c : flat) {
        if (c == '/' || c == '\\' || c == ':') {
            c = '_';
        }
    }
    const std::string path = (scratch / (flat + ".rigexec")).string();
    {
        std::ofstream stream(path, std::ios::binary);
        stream.write(reinterpret_cast<const char *>(first.data()),
                     std::streamsize(first.size()));
    }
    const std::vector<uint8_t> onDisk = Bytes(path);
    CHECK(onDisk == first);
    {
        std::unique_ptr<fb::RigExecWireFile> reopened;
        std::string error;
        CHECK(RigExecFormatOpen(onDisk.data(), onDisk.size(), &reopened,
                                &error));
    }

    // A bake is of the authored epoch: a held drag refuses it rather than
    // printing its value into the defaults.
    RigExecRigEvaluator evaluator(stage, rigPath);
    RigExecValueOverride drag;
    drag.prim = rigPath;
    drag.attribute = TfToken("avars:tx");
    drag.value = VtValue(0.25);
    evaluator.SetInteractiveOverrides({drag});
    RigExecBakeOpts opts;
    opts.time = bakeTime;
    RigExecBakeResult result;
    std::string error;
    CHECK(!RigExecBakeToBinary(evaluator, opts, &result, &error));
    CHECK(error == "cannot bake with interactive overrides standing");
    CHECK(result.bytes.empty());
}

// RIGEXEC_PROVIDER_PRUNE set to \p on (and RIGEXEC_VERIFY_PROVIDER_PRUNE with
// it) for the scope's lifetime, then restored; Build reads both.
struct _PruneKnobs {
    explicit _PruneKnobs(bool on)
        : prune(TfGetenv("RIGEXEC_PROVIDER_PRUNE")),
          verify(TfGetenv("RIGEXEC_VERIFY_PROVIDER_PRUNE"))
    {
        TfSetenv("RIGEXEC_PROVIDER_PRUNE", on ? "1" : "0");
        TfSetenv("RIGEXEC_VERIFY_PROVIDER_PRUNE", on ? "1" : "0");
    }
    ~_PruneKnobs()
    {
        Restore("RIGEXEC_PROVIDER_PRUNE", prune);
        Restore("RIGEXEC_VERIFY_PROVIDER_PRUNE", verify);
    }
    static void Restore(const std::string &name, const std::string &saved)
    {
        if (saved.empty()) {
            TfUnsetenv(name);
        } else {
            TfSetenv(name, saved);
        }
    }
    std::string prune, verify;
};

// Biped_anim baked with RIGEXEC_PROVIDER_PRUNE off and on: each file holds
// the program it was baked from (_BinaryCompareProgram) and validates; the
// pruned file carries fewer provider operations and lists the same public
// inputs, name and type, in the same order. The private slots only the
// unpruned file holds (read by pruned provider leaves alone) are printed.
// The runtime replay of the pruned bake against the evaluator runs in
// testRigExecRuntimeInputs and verify_binary_biped_Biped_anim; this suite
// does not link the runtime.
static void
TestAPrunedBakeReplaysTheSameValues()
{
    const std::string fixture =
        (std::filesystem::path(RIGEXEC_EXAMPLES_DIR) / "biped" /
         "Biped_anim.usda")
            .string();
    struct _Baked {
        bool ok = false;
        size_t pruned = 0, violations = 0, providerOps = 0;
        std::vector<std::pair<std::string, uint8_t>> listed;
        std::vector<std::string> unlisted;
    };
    const auto bake = [&](bool prune) {
        _Baked out;
        const _PruneKnobs knobs(prune);
        const UsdStageRefPtr stage = UsdStage::Open(fixture);
        CHECK(stage);
        if (!stage) {
            return out;
        }
        RigExecRigEvaluator evaluator(stage, FindRig(stage));
        RigExecBakeOpts opts;
        opts.time = 1.0;
        RigExecBakeResult result;
        std::string error;
        const bool baked =
            RigExecBakeToBinary(evaluator, opts, &result, &error);
        if (!baked) {
            std::printf("pruned bake diagnostic: %s\n", error.c_str());
        }
        CHECK(baked);
        CHECK(evaluator.GetBakedProgram() != nullptr);
        if (!baked || !evaluator.GetBakedProgram()) {
            return out;
        }
        const RigExecBakedProgramImpl &program =
            evaluator.GetBakedProgram()->GetStepGraph();
        out.pruned = program.providerStepsPruned;
        out.violations = program.providerPruneViolations;
        const std::unique_ptr<_BinaryFile> file = _BinaryCompareProgram(
            evaluator, result.bytes,
            fixture + (prune ? " (pruned)" : " (unpruned)"));
        CHECK(file);
        if (!file) {
            return out;
        }
        std::string why;
        const bool valid = RigExecFormatValidate(*file, &why);
        if (!valid) {
            std::printf("  pruned=%d bake does not validate: %s\n", int(prune),
                        why.c_str());
        }
        CHECK(valid);
        out.providerOps =
            file->providerProgram ? file->providerProgram->ops.size() : 0;
        for (size_t s = 0; s < file->inputs.size(); ++s) {
            const std::string name = _BinaryText(*file, file->inputs[s].name());
            if (s < file->listedInputs) {
                out.listed.emplace_back(name, uint8_t(file->inputs[s].type()));
            } else {
                out.unlisted.push_back(name);
            }
        }
        out.ok = valid;
        return out;
    };
    const _Baked off = bake(false);
    const _Baked on = bake(true);
    CHECK(off.ok && on.ok);
    if (!off.ok || !on.ok) {
        return;
    }
    CHECK(off.pruned == 0);
    CHECK(on.pruned > 0);
    CHECK(on.violations == 0);
    CHECK(on.providerOps < off.providerOps);
    CHECK(on.listed == off.listed);
    size_t onlyUnpruned = 0;
    for (const std::string &name : off.unlisted) {
        if (std::find(on.unlisted.begin(), on.unlisted.end(), name) ==
            on.unlisted.end()) {
            ++onlyUnpruned;
            std::printf("  private slot only the unpruned bake holds: %s\n",
                        name.c_str());
        }
    }
    std::printf("pruned bake: %zu provider step(s) pruned, provider ops %zu "
                "-> %zu, %zu listed input(s) identical, %zu of %zu private "
                "slot(s) left\n",
                on.pruned, off.providerOps, on.providerOps, on.listed.size(),
                onlyUnpruned, off.unlisted.size());
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
    if (argc > 1 && std::string(argv[1]) == "--conformance") {
        // CLI artifact, one fresh independent bake, then every physical
        // owner/default/route assertion and the authored-epoch refusal.
        if (argc != 6) {
            std::fprintf(stderr, "usage: --conformance STAGE TIME CLASS ARTIFACT\n");
            return 2;
        }
        double time = 0.0;
        try {
            size_t consumed = 0;
            time = std::stod(argv[3], &consumed);
            if (consumed != std::strlen(argv[3]) || !std::isfinite(time)) {
                throw std::invalid_argument("nonfinite or malformed time");
            }
        } catch (const std::exception &) {
            std::fprintf(stderr, "conformance requires a finite bake time\n");
            return 2;
        }
        const std::string animation(argv[4]);
        if (animation != "inputs" && animation != "static") {
            std::fprintf(stderr, "conformance CLASS must be inputs or static\n");
            return 2;
        }
        const std::vector<uint8_t> artifact = Bytes(argv[5]);
        CHECK(!artifact.empty());
        if (!artifact.empty()) {
            TestBake(argv[2], {time}, animation.c_str(), scratch, &artifact);
        }
        CHECK(compareTried == 1 && compareRows == 1);
        CHECK(classGuardStatic + classGuardInputs == 1);
        if (_EndsWith(_Slashed(argv[2]), "/biped/Biped.usda")) {
            CHECK(sizeBudgetChecked);
        }
        if (!failures) std::filesystem::remove_all(scratch);
        std::printf("binary conformance: %s time=%.17g class=%s failures=%d\n",
                    argv[2], time, argv[4], failures);
        return failures ? 1 : 0;
    }
    // The tail rig's file, baked at its fixture time: the real bake the
    // codec and corruption cases edit.
    const std::string tail =
        (std::filesystem::path(RIGEXEC_EXAMPLES_DIR) / "01_FkChainTail.usda")
            .string();
    const std::vector<uint8_t> tailBytes = _BakeFixture(tail, 1001.0);
    TestWrinkleWireExtensions(tailBytes);
    TestExternalMoversWire(tailBytes);
    TestRealFileCorruption(tailBytes);
    TestComputedEnvelopeBake();
    TestComputedCurrentPhaseBake();
    TestComputedChainBake();
    TestRangeChainBake();
    TestPhaseBindingBakes();
    TestRawSkinLayoutBake();
    TestEnumeratedReadThroughPropertyResult();
    TestStaticReportRevisionReads();
    TestStaticReportAnsweredBlendSamples();
    TestBakeOptions(tail, "/TailAsset/Rig/Controls/Tail2.avars:rz");
    TestGroupRolesBake();
    TestExportKeepSetBake();
    TestBakeRestoresLiveRoles();
    TestTheExportIgnoresTheWorkLimit();
    TestAPrunedBakeReplaysTheSameValues();
    // Each example's positive capture/default/source conformance and two
    // fresh bakes run in its existing verify_binary registration before
    // the unchanged runtime defaults, frame sampling and drag ledger.
    // This suite keeps the focused malformed, synthetic and multi-time
    // protocols, including their independent numerical assertions.
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
