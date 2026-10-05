// .rigexec bake conformance: the FlatBuffer file a bake writes, held to the
// program it was baked from, and the format's refusals on real bakes.
#include "rigExecBake/bake.h"
#include "rigExecBake/revisionReads.h"
#include "rigExecBake/staticReport.h"
#include "rigExec/frozenContextInternal.h"
#include "rigExec/rigEvaluator.h"
#include "rigExecBinary/external.h"
#include "rigExecBinary/format.h"
#include "rigExecBinary/generated/presentation_generated.h"
#include "rigExecExampleFixtures.h"
#include "rigExecMath/propertyMath.h"
#include "rigExecRigging/rigBuilder.h"

#include "pxr/base/plug/registry.h"
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
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
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

    // A phased input of the revision's binding takes a pooled fallback.
    fb::RigExecWireFile phased(plugin);
    {
        fb::RigExecWireRevisionBinding &binding =
            *phased.geometry->chains[0].revisions[0].binding;
        binding.phaseInputs.push_back(binding.target);
        binding.phases.push_back(RigExecWireReadPhase());
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
    // An aligned copy for the accessors that locate the tables.
    std::vector<uint64_t> aligned((bytes.size() + 7) / 8);
    std::memcpy(aligned.data(), bytes.data(), bytes.size());
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
            std::vector<uint8_t> mutated = bytes;
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
        std::vector<uint8_t> mutated = bytes;
        const int edits = 1 + int(next() % 4);
        for (int e = 0; e < edits; ++e) {
            const size_t at = next() % bytes.size();
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
                const uint32_t small = uint32_t(next() % bytes.size());
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
                vtableOpened, refused, opened, rounds, bytes.size());
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
    for (size_t c = 0; c < file.propertyChains.size(); ++c) {
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
        CHECK(s == 0 || previous < name);
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
    });
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
        evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
        const std::unique_ptr<fb::RigExecWireFile> file =
            _BakeAndCompare(evaluator, time, "envelope bake");
        if (!file) {
            return;
        }
        const RigExecBakedProgramImpl &program =
            evaluator.GetBakedProgram()->GetStepGraph();
        const auto &objects = file->geometry->weightObjects;
        CHECK(program.weightObjects.empty());
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
            CHECK(object.envelopeOnly);
            CHECK(object.oracleStaticError.empty());
            CHECK(object.oraclePhaseError.empty());
            CHECK(object.defaultWeight->mode == fb::ReadMode::Resolved);
            const bool dynamic =
                RigExecFormatPathText(*file, object.type) ==
                "RigExecDynamicWeight";
            if (dynamic) {
                // inputs:driver walks to the avar it is connected to.
                CHECK(object.driver->walk.size() == 2);
                if (object.driver->walk.size() == 2) {
                    const fb::InputSlot &leaf =
                        file->inputs[object.driver->walk[1]];
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
                    _ComputedReadFloat(*file, *object.driver);
                const float scale = _ComputedReadFloat(*file, *object.scale);
                const float bias = _ComputedReadFloat(*file, *object.bias);
                expected = (base * driver) * scale + bias;
            } else {
                expected = _ComputedReadFloat(*file, *object.defaultWeight);
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
        evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
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
        CHECK(found->oracleSamples == -1);
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

// The oracle's two scalar arms for an envelope-only object at count 1
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
        const auto read = [&](const fb::RigExecWireInput &input,
                              float fallback) {
            _Val value;
            return _ChainLongWay(file, overlay, input.walk, 0,
                                 fb::InputTag::Float, &value)
                       ? _ChainFloat(value)
                       : fallback;
        };
        CHECK(object.envelopeOnly && object.oracleStaticError.empty() &&
              object.base < 0 && object.inputs.empty() &&
              representation == "constant" && object.values.empty());
        if (type == "RigExecStaticWeight") {
            float w = read(*object.defaultWeight, 0.0f);
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
        const float driver = read(*object.driver, 1.0f);
        const float scale = read(*object.scale, 1.0f);
        const float bias = read(*object.bias, 0.0f);
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
        evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
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
        });
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
        replay.SetEvaluationMode(RigExecEvaluationMode::Baked);
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
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
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
        evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
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
        evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
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
        evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
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
        evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
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

// examples/biped/Biped.usda baked at its fixture time (1): 3,212,960 bytes,
// measured 2026-10-05. The budget is that plus 5%.
constexpr size_t kBipedBytes = 3212960;
constexpr size_t kBipedBudget = kBipedBytes + kBipedBytes / 20;

static bool sizeBudgetChecked = false;

/// The file's size against the biped budget, and what each root field
/// costs: the bytes a copy with that field emptied packs smaller by.
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
                "2026-10-05)\n",
                fixture.c_str(), bytes, kBipedBudget, kBipedBytes);
    const std::unique_ptr<fb::RigExecWireFile> file =
        RigExecTestUnpack(result.bytes);
    if (!file) {
        return;
    }
    CHECK(RigExecTestPackUnchecked(*file).size() == bytes);
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
        CHECK(cleared <= bytes);
        std::printf("  size %-34s %10zu bytes\n", field.first,
                    bytes - std::min(bytes, cleared));
    }
}

static int compareRows = 0;
static int compareTried = 0;
static int classGuardStatic = 0;
static int classGuardInputs = 0;

static void
TestBake(const std::string &fixture, const std::vector<double> &tableFrames,
         const char *animation, const std::filesystem::path &scratch)
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
    // Two fresh evaluators, one bake each: a bake is deterministic, so the
    // bytes are identical -- which is what makes a byte golden meaningful.
    std::vector<uint8_t> first;
    std::unique_ptr<fb::RigExecWireFile> file;
    for (int pass = 0; pass < 2; ++pass) {
        RigExecRigEvaluator evaluator(stage, rigPath);
        evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
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
        first = result.bytes;
        ++compareTried;
        const int failuresBefore = failures;
        _BinaryCompareStats stats;
        file = _BinaryCompareProgram(evaluator, result.bytes, fixture, &stats);
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
                    "topologies (%zu entries dropped), %zu blend samples, "
                    "%zu bases, %zu property chains, %zu oracle facts: %d "
                    "failures\n",
                    fixture.c_str(), file->steps.size(), file->inputs.size(),
                    stats.defaults, stats.reads, stats.overrideNumbers,
                    file->geometry->pathReads.size(), stats.readRows,
                    stats.valueRows, stats.enumeratedKeys, stats.topologies,
                    stats.droppedEntries, stats.samples, stats.bases,
                    stats.propertyChains, stats.oracleFacts, failed);
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
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
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
    TestEnumeratedReadThroughPropertyResult();
    TestStaticReportRevisionReads();
    TestBakeOptions(tail, "/TailAsset/Rig/Controls/Tail2.avars:rz");
    auto BakeOne = [&](const std::string &stage,
                       const std::vector<double> &frames,
                       const char *animation) {
        std::printf("bake conformance: %s\n", stage.c_str());
        TestBake(stage, frames, animation, scratch);
    };
    int bakingRows = 0;
    bool fixtureTable = false;
    if (argc > 1 && std::filesystem::is_directory(argv[1])) {
        fixtureTable = true;
        for (const RigExecExampleFixture &fixture :
             kRigExecExampleFixtures) {
            if (!fixture.bakesToday) {
                std::printf("bake conformance: skip %s (blocked by %s)\n",
                            fixture.stage, fixture.blockedBy);
                continue;
            }
            const std::string stage =
                (std::filesystem::path(argv[1]) / fixture.stage).string();
            BakeOne(stage, _ParseTableFrames(fixture.frames),
                    fixture.animation);
            ++bakingRows;
        }
    } else if (argc > 1) {
        for (int i = 1; i < argc; ++i) {
            BakeOne(argv[i], {1001}, nullptr);
        }
    } else {
        fixtureTable = true;
        for (const RigExecExampleFixture &fixture :
             kRigExecExampleFixtures) {
            if (!fixture.bakesToday) {
                continue;
            }
            const std::string stage =
                (std::filesystem::path(RIGEXEC_EXAMPLES_DIR) /
                 fixture.stage).string();
            BakeOne(stage, _ParseTableFrames(fixture.frames),
                    fixture.animation);
            ++bakingRows;
        }
    }
    std::printf("class guard: %d static row(s) holding an animated source "
                "in static data, %d inputs row(s) holding none, of %d "
                "baking row(s)\n",
                classGuardStatic, classGuardInputs, bakingRows);
    CHECK(classGuardStatic + classGuardInputs == bakingRows);
    std::printf("compare: %d of %d baking row(s) hold the program they were "
                "baked from\n",
                compareRows, compareTried);
    CHECK(compareRows == compareTried);
    CHECK(bakingRows == 0 || compareTried == bakingRows);
    // The fixture table holds the biped, so its run checks the budget.
    CHECK(!fixtureTable || sizeBudgetChecked);
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
