//
// RigExecFrameCache, Stream A: the (epochDigest, controlDigest) store.
//
// The key, entry, and stats types keep the semantics Stream 0 asserted, so
// those tests stand unchanged below. The stub half of the Stream 0 file is
// replaced here by the store's own contract, one rule per test group in the
// testRigExecStaticInputCache style:
//
//   * KEYING. A pose is a function of its two digests and nothing else:
//     either half misses, time is not key material, and a hit serves the
//     published pose bit-identically.
//   * DIGEST. RigExecControlStateDigest covers the sampled sources plus the
//     interactive overrides: a control change misses, an unrelated edit
//     hits, enqueue order never matters, and floating point folds bitwise.
//     RigExecRefusalControlDigest is the D7 time-plus-serial fallback for
//     rigs with no baked program: the frame moves it, an edit moves it at
//     every frame, a drag moves it, and it never equals a sampled digest
//     under the same epoch.
//   * LRU. A per-rig byte cap with least-recently-used eviction, accounted
//     in RigExecFrameCachePoseBytes; oversized entries are dropped, never
//     partially stored.
//   * THREAD. Lookup runs concurrently with background Publish: the stress
//     below hammers both and holds the counters to account, and the cache
//     is still functional afterwards.
//   * DROP. Corrupt input is declined, never served: invalid poses,
//     oversized entries, and null out-params all miss.
//
#include "rigExec/frameCache.h"
#include "rigExec/frozenContext.h"

#include <atomic>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

using namespace rigExec;

static int failures = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            ++failures;                                                    \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
        }                                                                  \
    } while (0)

namespace {

// A valid pose whose bytes do not move with the tag: the tag rides a point
// coordinate, so same-shape poses are same-bytes poses and the LRU tests can
// size caps in whole entries.
RigExecRigPose
MakePose(double tag, size_t joints)
{
    RigExecRigPose pose;
    pose.valid = true;
    for (size_t i = 0; i < joints; ++i) {
        RigExecPointFrame frame;
        frame.points[0] = GfVec3d(tag, double(i), 0.0);
        pose.jointFramesFinal[SdfPath("/Joint" + std::to_string(i))] = frame;
    }
    return pose;
}

bool
PosesMatch(const RigExecRigPose &a, const RigExecRigPose &b)
{
    return a.valid == b.valid &&
           a.jointFramesBase == b.jointFramesBase &&
           a.jointFramesFinal == b.jointFramesFinal &&
           a.jointMatricesFinal == b.jointMatricesFinal &&
           a.controlFrames == b.controlFrames &&
           a.providerXforms == b.providerXforms &&
           a.providerBaseXforms == b.providerBaseXforms &&
           a.solverFrames == b.solverFrames &&
           a.movedProperties == b.movedProperties &&
           a.diagnostics == b.diagnostics;
}

RigExecFrameInputs
MakeInputs(std::initializer_list<std::pair<const char *, double>> samples)
{
    RigExecFrameInputs inputs;
    inputs.time = UsdTimeCode(3.0);
    for (const auto &s : samples) {
        inputs.Add(SdfPath(s.first), VtValue(s.second));
    }
    return inputs;
}

RigExecValueOverride
MakeOverride(const char *prim, double value)
{
    RigExecValueOverride o;
    o.prim = SdfPath(prim);
    o.computation = TfToken("computePointFrame");
    o.value = VtValue(value);
    return o;
}

// A held type VtValue cannot hash and the digest has no fold for: frames
// carrying one must bypass the cache.
struct Unhashable {
    int x = 0;
    bool operator==(const Unhashable &o) const { return x == o.x; }
};

// Two digests, both significant: keys differing in either half are different
// keys, and identical halves are identical keys.
void
TestKeyEqualitySeesBothDigests()
{
    const RigExecFrameCacheKey base{7, 11};
    const RigExecFrameCacheKey same{7, 11};
    const RigExecFrameCacheKey otherEpoch{8, 11};
    const RigExecFrameCacheKey otherControl{7, 12};
    CHECK(base == same);
    CHECK(!(base != same));
    CHECK(base != otherEpoch);
    CHECK(base != otherControl);
    CHECK(!(base == otherEpoch));
}

// Ordered containers of keys stay usable: strict-weak ordering over the
// epoch half first, the control half second.
void
TestKeyOrderingIsEpochThenControl()
{
    const RigExecFrameCacheKey a{1, 9};
    const RigExecFrameCacheKey b{2, 0};
    const RigExecFrameCacheKey c{1, 10};
    CHECK(a < b);
    CHECK(!(b < a));
    CHECK(a < c);
    CHECK(!(c < a));
    CHECK(!(a < a));
}

// Equal keys hash equal, so a hashed store cannot lose an entry it holds.
// (Unequal keys hashing unequal is not asserted: a collision is allowed, a
// lost entry is not.)
void
TestEqualKeysHashEqual()
{
    const RigExecFrameCacheKeyHash hash;
    CHECK(hash(RigExecFrameCacheKey{3, 5}) ==
          hash(RigExecFrameCacheKey{3, 5}));
}

// Time rides the entry, never the key: the same key names the same inputs at
// whatever frame they were evaluated at, and the entry remembers which frame
// that was.
void
TestTimeRidesTheEntry()
{
    RigExecFrameCacheEntry entry;
    entry.time = UsdTimeCode(12.0);
    CHECK(entry.time == UsdTimeCode(12.0));
    CHECK(entry.bytes == 0);
}

// An empty cache misses, and the miss is counted. The pose out-param is
// untouched -- a miss must never publish a pose it did not read.
void
TestAnEmptyCacheMisses()
{
    RigExecFrameCache cache;
    const RigExecFrameCacheKey key{1, 2};
    RigExecRigPose pose;
    pose.valid = true;
    CHECK(!cache.Lookup(key, &pose));
    CHECK(pose.valid);
    const RigExecFrameCacheStats stats = cache.Stats();
    CHECK(stats.hits == 0);
    CHECK(stats.misses == 1);
    CHECK(stats.published == 0);
    CHECK(stats.entryCount == 0);
    CHECK(stats.bytes == 0);
}

// A null out-param misses rather than faults, and the miss is counted like
// any other.
void
TestLookupWithNullPoseMisses()
{
    RigExecFrameCache cache;
    const RigExecFrameCacheKey key{1, 2};
    CHECK(cache.Publish(key, UsdTimeCode(1.0), MakePose(1.0, 1)));
    CHECK(!cache.Lookup(key, nullptr));
    const RigExecFrameCacheStats stats = cache.Stats();
    CHECK(stats.hits == 0);
    CHECK(stats.misses == 1);
    CHECK(stats.entryCount == 1);
}

// Publish then lookup: the hit serves the published pose bit-identically,
// and the counters say one publish, one hit.
void
TestPublishThenLookupHits()
{
    RigExecFrameCache cache;
    const RigExecFrameCacheKey key{4, 9};
    const RigExecRigPose published = MakePose(2.5, 3);
    CHECK(cache.Publish(key, UsdTimeCode(6.0), published));
    RigExecRigPose found;
    CHECK(cache.Lookup(key, &found));
    CHECK(PosesMatch(published, found));
    const RigExecFrameCacheStats stats = cache.Stats();
    CHECK(stats.hits == 1);
    CHECK(stats.misses == 0);
    CHECK(stats.published == 1);
    CHECK(stats.evictions == 0);
    CHECK(stats.entryCount == 1);
    CHECK(stats.bytes == RigExecFrameCachePoseBytes(published));
}

// Either digest half misses: the epoch the program was built for and the
// control state are both key material, and neighbors of a held key are cold.
void
TestEitherDigestHalfMisses()
{
    RigExecFrameCache cache;
    CHECK(cache.Publish({1, 2}, UsdTimeCode(1.0), MakePose(1.0, 1)));
    RigExecRigPose pose;
    CHECK(!cache.Lookup({9, 2}, &pose));
    CHECK(!cache.Lookup({1, 8}, &pose));
    CHECK(cache.Lookup({1, 2}, &pose));
    const RigExecFrameCacheStats stats = cache.Stats();
    CHECK(stats.hits == 1);
    CHECK(stats.misses == 2);
    CHECK(stats.entryCount == 1);
}

// Time is not key material: publishing the same key at a second frame
// replaces the pose in place -- one entry, latest pose served.
void
TestTimeIsNotKeyMaterial()
{
    RigExecFrameCache cache;
    const RigExecFrameCacheKey key{1, 2};
    CHECK(cache.Publish(key, UsdTimeCode(1.0), MakePose(1.0, 1)));
    CHECK(cache.Publish(key, UsdTimeCode(2.0), MakePose(2.0, 1)));
    const RigExecFrameCacheStats stats = cache.Stats();
    CHECK(stats.published == 2);
    CHECK(stats.entryCount == 1);
    RigExecRigPose found;
    CHECK(cache.Lookup(key, &found));
    CHECK(PosesMatch(MakePose(2.0, 1), found));
}

// An invalid pose is not a pose: Publish declines it and the store is
// unchanged, so a fail-closed evaluation can never poison the cache.
void
TestPublishInvalidPoseDeclines()
{
    RigExecFrameCache cache;
    RigExecRigPose pose;
    pose.valid = false;
    CHECK(!cache.Publish({1, 2}, UsdTimeCode(1.0), pose));
    const RigExecFrameCacheStats stats = cache.Stats();
    CHECK(stats.published == 0);
    CHECK(stats.entryCount == 0);
    CHECK(stats.bytes == 0);
    RigExecRigPose found;
    CHECK(!cache.Lookup({1, 2}, &found));
}

// Evict drops the entry and counts it; a second evict of the same key finds
// nothing and counts nothing.
void
TestEvictDropsAndCounts()
{
    RigExecFrameCache cache;
    CHECK(cache.Publish({1, 2}, UsdTimeCode(1.0), MakePose(1.0, 1)));
    CHECK(cache.Evict({1, 2}));
    RigExecRigPose pose;
    CHECK(!cache.Lookup({1, 2}, &pose));
    CHECK(!cache.Evict({1, 2}));
    const RigExecFrameCacheStats stats = cache.Stats();
    CHECK(stats.published == 1);
    CHECK(stats.evictions == 1);
    CHECK(stats.entryCount == 0);
    CHECK(stats.bytes == 0);
}

// The byte cap round-trips, and Clear drops every entry and zeroes the
// counters while leaving the cap -- the size the rig was given -- in place.
void
TestCapRoundTripsAndClearZeroesCounters()
{
    RigExecFrameCache cache;
    CHECK(cache.GetByteCap() == kRigExecFrameCacheDefaultByteCap);
    cache.SetByteCap(1024 * 1024);
    CHECK(cache.GetByteCap() == 1024 * 1024);

    CHECK(cache.Publish({1, 2}, UsdTimeCode(1.0), MakePose(1.0, 1)));
    RigExecRigPose pose;
    CHECK(cache.Lookup({1, 2}, &pose));
    CHECK(!cache.Lookup({9, 9}, &pose));
    CHECK(cache.Stats().hits == 1);
    CHECK(cache.Stats().misses == 1);
    cache.Clear();
    const RigExecFrameCacheStats stats = cache.Stats();
    CHECK(stats.hits == 0);
    CHECK(stats.misses == 0);
    CHECK(stats.published == 0);
    CHECK(stats.evictions == 0);
    CHECK(stats.entryCount == 0);
    CHECK(stats.bytes == 0);
    CHECK(cache.GetByteCap() == 1024 * 1024);
    CHECK(!cache.Lookup({1, 2}, &pose));
}

// Pose bytes are the struct shell plus every published map: an empty pose
// costs the shell, one joint costs a key plus a frame, and Publish accounts
// exactly what the function measures.
void
TestPoseBytesCountsMaps()
{
    RigExecRigPose empty;
    empty.valid = true;
    CHECK(RigExecFrameCachePoseBytes(empty) == sizeof(RigExecRigPose));

    const RigExecRigPose one = MakePose(1.0, 1);
    const size_t expect =
        sizeof(RigExecRigPose) + sizeof(SdfPath) + sizeof(RigExecPointFrame);
    CHECK(RigExecFrameCachePoseBytes(one) == expect);
    // The tag rides a coordinate, so same-shape poses are same-bytes poses.
    CHECK(RigExecFrameCachePoseBytes(MakePose(7.5, 1)) == expect);

    RigExecFrameCache cache;
    CHECK(cache.Publish({1, 2}, UsdTimeCode(1.0), one));
    CHECK(cache.Stats().bytes == expect);
}

// The digest covers the value, not the sampling order: shuffled inputs name
// the same control state.
void
TestDigestIgnoresEnqueueOrder()
{
    const RigExecFrameInputs fwd =
        MakeInputs({{"/Rig/A", 1.0}, {"/Rig/B", 2.0}, {"/Rig/C", 3.0}});
    const RigExecFrameInputs rev =
        MakeInputs({{"/Rig/C", 3.0}, {"/Rig/B", 2.0}, {"/Rig/A", 1.0}});
    CHECK(RigExecControlStateDigest(fwd) == RigExecControlStateDigest(rev));
}

// A control change misses: any sampled value moving moves the digest --
// including +0.0 to -0.0, which hash equal under TfHash but need not
// evaluate equal.
void
TestDigestSeesValueChanges()
{
    const RigExecFrameInputs base = MakeInputs({{"/Rig/A", 1.0}});
    const RigExecFrameInputs moved = MakeInputs({{"/Rig/A", 1.5}});
    CHECK(RigExecControlStateDigest(base) != RigExecControlStateDigest(moved));

    const RigExecFrameInputs plusZero = MakeInputs({{"/Rig/A", 0.0}});
    const RigExecFrameInputs minusZero = MakeInputs({{"/Rig/A", -0.0}});
    CHECK(RigExecControlStateDigest(plusZero) !=
          RigExecControlStateDigest(minusZero));

    RigExecFrameInputs floats;
    floats.Add(SdfPath("/Rig/A"), VtValue(1.0f));
    RigExecFrameInputs doubles;
    doubles.Add(SdfPath("/Rig/A"), VtValue(1.0));
    CHECK(RigExecControlStateDigest(floats) !=
          RigExecControlStateDigest(doubles));
}

// The mixer chains non-commutatively: duplicated values do not cancel, and
// permuted values do not collide. An xor-only fold maps [W,W] to the seed
// for every W -- changing both duplicates would serve a stale pose -- and
// [A,B] to [B,A].
void
TestDigestSeesDuplicateValues()
{
    // The same double on two paths, changed on both: the digest moves.
    const RigExecFrameInputs before =
        MakeInputs({{"/Rig/A", 10.2}, {"/Rig/B", 10.2}});
    const RigExecFrameInputs after =
        MakeInputs({{"/Rig/A", 11.7}, {"/Rig/B", 11.7}});
    CHECK(RigExecControlStateDigest(before) !=
          RigExecControlStateDigest(after));
    // One of the pair moved: moves too.
    const RigExecFrameInputs half =
        MakeInputs({{"/Rig/A", 10.2}, {"/Rig/B", 11.7}});
    CHECK(RigExecControlStateDigest(before) !=
          RigExecControlStateDigest(half));
    CHECK(RigExecControlStateDigest(after) !=
          RigExecControlStateDigest(half));
    // Swapped values across paths: order matters.
    const RigExecFrameInputs ab =
        MakeInputs({{"/Rig/A", 1.0}, {"/Rig/B", 2.0}});
    const RigExecFrameInputs ba =
        MakeInputs({{"/Rig/A", 2.0}, {"/Rig/B", 1.0}});
    CHECK(RigExecControlStateDigest(ab) != RigExecControlStateDigest(ba));
    // Identical vectors still digest identically.
    const RigExecFrameInputs again =
        MakeInputs({{"/Rig/A", 10.2}, {"/Rig/B", 10.2}});
    CHECK(RigExecControlStateDigest(before) ==
          RigExecControlStateDigest(again));
}

// Paths and types are digest material: the same value at another path, or
// under another type, names another control state.
void
TestDigestSeesPathsAndTypes()
{
    const RigExecFrameInputs atA = MakeInputs({{"/Rig/A", 1.0}});
    const RigExecFrameInputs atB = MakeInputs({{"/Rig/B", 1.0}});
    CHECK(RigExecControlStateDigest(atA) != RigExecControlStateDigest(atB));

    RigExecFrameInputs ints;
    ints.Add(SdfPath("/Rig/A"), VtValue(1));
    CHECK(RigExecControlStateDigest(atA) != RigExecControlStateDigest(ints));

    RigExecFrameInputs tokens;
    tokens.Add(SdfPath("/Rig/A"), VtValue(TfToken("one")));
    RigExecFrameInputs strings;
    strings.Add(SdfPath("/Rig/A"), VtValue(std::string("one")));
    CHECK(RigExecControlStateDigest(tokens) !=
          RigExecControlStateDigest(strings));
}

// An unrelated edit hits: inputs the frame never sampled are not digest
// material, so the digest over the same vector is unchanged -- while a
// source that held no value digests distinctly from every valued source.
void
TestDigestHitsAcrossUnrelatedEdits()
{
    const RigExecFrameInputs base = MakeInputs({{"/Rig/A", 1.0}});
    const RigExecFrameInputs same = MakeInputs({{"/Rig/A", 1.0}});
    CHECK(RigExecControlStateDigest(base) == RigExecControlStateDigest(same));

    RigExecFrameInputs valueless;
    valueless.Add(SdfPath("/Rig/A"), VtValue(), /*hasValue=*/false);
    CHECK(RigExecControlStateDigest(base) !=
          RigExecControlStateDigest(valueless));
    // And the valueless digest is stable: a stale value riding along cannot
    // move it.
    RigExecFrameInputs stale;
    stale.Add(SdfPath("/Rig/A"), VtValue(9.0), /*hasValue=*/false);
    CHECK(RigExecControlStateDigest(valueless) ==
          RigExecControlStateDigest(stale));
}

// The first sample at a path wins, matching FrameInputs::Find: a resampled
// path digests as its enqueue-time value, never a later one.
void
TestDigestFirstSampleWins()
{
    RigExecFrameInputs inputs;
    inputs.Add(SdfPath("/Rig/A"), VtValue(1.0));
    inputs.Add(SdfPath("/Rig/A"), VtValue(9.0));
    const RigExecFrameInputs first = MakeInputs({{"/Rig/A", 1.0}});
    CHECK(RigExecControlStateDigest(inputs) ==
          RigExecControlStateDigest(first));
}

// Overrides are digest material -- they never reach the stage, so folding
// them in is what stops a drag serving a pre-drag pose -- while override
// order never matters and a repeated key keeps the last value, matching the
// evaluator's replace-not-join rule.
void
TestDigestSeesOverrides()
{
    const RigExecFrameInputs inputs = MakeInputs({{"/Rig/A", 1.0}});
    const std::vector<RigExecValueOverride> none;
    const std::vector<RigExecValueOverride> held{MakeOverride("/Rig/C", 2.0)};
    CHECK(RigExecControlStateDigest(inputs, none) !=
          RigExecControlStateDigest(inputs, held));

    const std::vector<RigExecValueOverride> moved{MakeOverride("/Rig/C", 3.0)};
    CHECK(RigExecControlStateDigest(inputs, held) !=
          RigExecControlStateDigest(inputs, moved));

    const std::vector<RigExecValueOverride> elsewhere{
        MakeOverride("/Rig/D", 2.0)};
    CHECK(RigExecControlStateDigest(inputs, held) !=
          RigExecControlStateDigest(inputs, elsewhere));

    const std::vector<RigExecValueOverride> fwd{
        MakeOverride("/Rig/C", 2.0), MakeOverride("/Rig/D", 3.0)};
    const std::vector<RigExecValueOverride> rev{
        MakeOverride("/Rig/D", 3.0), MakeOverride("/Rig/C", 2.0)};
    CHECK(RigExecControlStateDigest(inputs, fwd) ==
          RigExecControlStateDigest(inputs, rev));

    const std::vector<RigExecValueOverride> dup{
        MakeOverride("/Rig/C", 2.0), MakeOverride("/Rig/C", 5.0)};
    const std::vector<RigExecValueOverride> last{MakeOverride("/Rig/C", 5.0)};
    CHECK(RigExecControlStateDigest(inputs, dup) ==
          RigExecControlStateDigest(inputs, last));
}

// Time is digest-excluded: the same control state at another frame digests
// equal, which is what lets a scrub hit across times.
void
TestDigestIgnoresTime()
{
    RigExecFrameInputs early = MakeInputs({{"/Rig/A", 1.0}});
    early.time = UsdTimeCode(1.0);
    RigExecFrameInputs late = MakeInputs({{"/Rig/A", 1.0}});
    late.time = UsdTimeCode(48.0);
    CHECK(RigExecControlStateDigest(early) == RigExecControlStateDigest(late));
}

// Digestibility is the fail-closed gate: standard held types pass, and a
// frame carrying a value no hash can name must bypass the cache.
void
TestDigestibleFlagsUnhashableValues()
{
    const RigExecFrameInputs plain = MakeInputs({{"/Rig/A", 1.0}});
    CHECK(RigExecControlStateDigestible(plain));

    RigExecFrameInputs empty;
    CHECK(RigExecControlStateDigestible(empty));

    RigExecFrameInputs held;
    held.Add(SdfPath("/Rig/A"), VtValue(1.0));
    held.Add(SdfPath("/Rig/B"), VtValue(Unhashable{3}));
    CHECK(!held.values.back().value.CanHash());
    CHECK(!RigExecControlStateDigestible(held));

    RigExecValueOverride o = MakeOverride("/Rig/C", 1.0);
    o.value = VtValue(Unhashable{4});
    CHECK(!RigExecControlStateDigestible(plain, {o}));
    CHECK(RigExecControlStateDigestible(plain, {MakeOverride("/Rig/C", 1.0)}));
}

// The D7 fallback digest keys the frame plus the stage-edit serial plus the
// standing overrides: the same frame re-digests equal, another frame digests
// apart (Default most of all), an edit (a moved serial) digests apart at
// every frame, a drag always digests apart from the authored frame, override
// order never matters, and the domain tag keeps it from ever equaling a
// sampled digest under the same epoch.
void
TestRefusalDigestKeysTimeAndOverrides()
{
    const std::vector<RigExecValueOverride> none;
    const std::vector<RigExecValueOverride> held{MakeOverride("/Rig/C", 2.0)};
    CHECK(RigExecRefusalControlDigest(UsdTimeCode(2.0), 7, none) ==
          RigExecRefusalControlDigest(UsdTimeCode(2.0), 7, none));
    CHECK(RigExecRefusalControlDigest(UsdTimeCode(2.0), 7, none) !=
          RigExecRefusalControlDigest(UsdTimeCode(3.0), 7, none));
    CHECK(RigExecRefusalControlDigest(UsdTimeCode(2.0), 7, none) !=
          RigExecRefusalControlDigest(UsdTimeCode::Default(), 7, none));
    CHECK(RigExecRefusalControlDigest(UsdTimeCode(2.0), 7, none) !=
          RigExecRefusalControlDigest(UsdTimeCode(2.0), 8, none));
    CHECK(RigExecRefusalControlDigest(UsdTimeCode(2.0), 7, none) !=
          RigExecRefusalControlDigest(UsdTimeCode(2.0), 7, held));

    const std::vector<RigExecValueOverride> moved{MakeOverride("/Rig/C", 3.0)};
    CHECK(RigExecRefusalControlDigest(UsdTimeCode(2.0), 7, held) !=
          RigExecRefusalControlDigest(UsdTimeCode(2.0), 7, moved));

    const std::vector<RigExecValueOverride> fwd{
        MakeOverride("/Rig/C", 2.0), MakeOverride("/Rig/D", 3.0)};
    const std::vector<RigExecValueOverride> rev{
        MakeOverride("/Rig/D", 3.0), MakeOverride("/Rig/C", 2.0)};
    CHECK(RigExecRefusalControlDigest(UsdTimeCode(2.0), 7, fwd) ==
          RigExecRefusalControlDigest(UsdTimeCode(2.0), 7, rev));

    const std::vector<RigExecValueOverride> dup{
        MakeOverride("/Rig/C", 2.0), MakeOverride("/Rig/C", 5.0)};
    const std::vector<RigExecValueOverride> last{MakeOverride("/Rig/C", 5.0)};
    CHECK(RigExecRefusalControlDigest(UsdTimeCode(2.0), 7, dup) ==
          RigExecRefusalControlDigest(UsdTimeCode(2.0), 7, last));

    // The domain tag: a mode toggle crosses sampled and refusal digests
    // without moving the epoch, so the two must never agree.
    const RigExecFrameInputs inputs = MakeInputs({{"/Rig/A", 1.0}});
    CHECK(RigExecRefusalControlDigest(UsdTimeCode(3.0), 7, none) !=
          RigExecControlStateDigest(inputs));
    CHECK(RigExecRefusalControlDigest(UsdTimeCode(3.0), 7, none) !=
          RigExecControlStateDigest(inputs, held));

    // Digestibility is the same fail-closed gate: time always folds, so
    // only the overrides can carry an unhashable type.
    CHECK(RigExecRefusalControlDigestible(none));
    CHECK(RigExecRefusalControlDigestible(held));
    RigExecValueOverride o = MakeOverride("/Rig/C", 1.0);
    o.value = VtValue(Unhashable{5});
    CHECK(!o.value.CanHash());
    CHECK(!RigExecRefusalControlDigestible({o}));
}

// The cap fits two entries: the third publish evicts the
// least-recently-used one, and the held bytes never exceed the cap.
void
TestLRUEvictsLeastRecentlyUsed()
{
    const size_t entry = RigExecFrameCachePoseBytes(MakePose(0.0, 1));
    RigExecFrameCache cache;
    cache.SetByteCap(2 * entry);
    CHECK(cache.Publish({1, 1}, UsdTimeCode(1.0), MakePose(1.0, 1)));
    CHECK(cache.Publish({1, 2}, UsdTimeCode(2.0), MakePose(2.0, 1)));
    CHECK(cache.Stats().evictions == 0);
    CHECK(cache.Publish({1, 3}, UsdTimeCode(3.0), MakePose(3.0, 1)));

    const RigExecFrameCacheStats stats = cache.Stats();
    CHECK(stats.evictions == 1);
    CHECK(stats.entryCount == 2);
    CHECK(stats.bytes == 2 * entry);
    CHECK(stats.bytes <= cache.GetByteCap());

    RigExecRigPose pose;
    CHECK(!cache.Lookup({1, 1}, &pose));
    CHECK(cache.Lookup({1, 2}, &pose));
    CHECK(PosesMatch(MakePose(2.0, 1), pose));
    CHECK(cache.Lookup({1, 3}, &pose));
}

// A lookup refreshes recency: the touched entry survives the next eviction
// and the untouched one -- published later but read never -- goes.
void
TestLookupRefreshesRecency()
{
    const size_t entry = RigExecFrameCachePoseBytes(MakePose(0.0, 1));
    RigExecFrameCache cache;
    cache.SetByteCap(2 * entry);
    CHECK(cache.Publish({1, 1}, UsdTimeCode(1.0), MakePose(1.0, 1)));
    CHECK(cache.Publish({1, 2}, UsdTimeCode(2.0), MakePose(2.0, 1)));
    RigExecRigPose pose;
    CHECK(cache.Lookup({1, 1}, &pose));
    CHECK(cache.Publish({1, 3}, UsdTimeCode(3.0), MakePose(3.0, 1)));

    CHECK(cache.Lookup({1, 1}, &pose));
    CHECK(!cache.Lookup({1, 2}, &pose));
    CHECK(cache.Lookup({1, 3}, &pose));
    CHECK(cache.Stats().entryCount == 2);
}

// A republish refreshes recency too: overwriting an entry makes it newest,
// so the next eviction takes its untouched sibling.
void
TestOverwriteRefreshesRecency()
{
    const size_t entry = RigExecFrameCachePoseBytes(MakePose(0.0, 1));
    RigExecFrameCache cache;
    cache.SetByteCap(2 * entry);
    CHECK(cache.Publish({1, 1}, UsdTimeCode(1.0), MakePose(1.0, 1)));
    CHECK(cache.Publish({1, 2}, UsdTimeCode(2.0), MakePose(2.0, 1)));
    CHECK(cache.Publish({1, 1}, UsdTimeCode(3.0), MakePose(9.0, 1)));
    CHECK(cache.Publish({1, 3}, UsdTimeCode(4.0), MakePose(3.0, 1)));

    RigExecRigPose pose;
    CHECK(cache.Lookup({1, 1}, &pose));
    CHECK(PosesMatch(MakePose(9.0, 1), pose));
    CHECK(!cache.Lookup({1, 2}, &pose));
    CHECK(cache.Lookup({1, 3}, &pose));
}

// An entry larger than the whole cap is dropped rather than stored: the
// publish declines and the store is unchanged.
void
TestOversizedPublishDeclines()
{
    const size_t entry = RigExecFrameCachePoseBytes(MakePose(0.0, 1));
    RigExecFrameCache cache;
    cache.SetByteCap(entry - 1);
    CHECK(!cache.Publish({1, 1}, UsdTimeCode(1.0), MakePose(1.0, 1)));
    const RigExecFrameCacheStats stats = cache.Stats();
    CHECK(stats.published == 0);
    CHECK(stats.entryCount == 0);
    CHECK(stats.bytes == 0);
}

// Lowering the cap evicts down to it immediately, least-recently-used
// first: the newest entry is the one left standing.
void
TestLoweringCapEvictsDown()
{
    const size_t entry = RigExecFrameCachePoseBytes(MakePose(0.0, 1));
    RigExecFrameCache cache;
    CHECK(cache.Publish({1, 1}, UsdTimeCode(1.0), MakePose(1.0, 1)));
    CHECK(cache.Publish({1, 2}, UsdTimeCode(2.0), MakePose(2.0, 1)));
    CHECK(cache.Publish({1, 3}, UsdTimeCode(3.0), MakePose(3.0, 1)));
    cache.SetByteCap(entry);

    const RigExecFrameCacheStats stats = cache.Stats();
    CHECK(stats.entryCount == 1);
    CHECK(stats.bytes == entry);
    CHECK(stats.evictions == 2);
    RigExecRigPose pose;
    CHECK(!cache.Lookup({1, 1}, &pose));
    CHECK(!cache.Lookup({1, 2}, &pose));
    CHECK(cache.Lookup({1, 3}, &pose));
}

// Lookup runs concurrently with background Publish: four threads hammer a
// sixteen-key cache five hundred times each, and every lookup is counted
// exactly once while the held bytes stay under the cap.
void
TestConcurrentLookupDuringPublish()
{
    RigExecFrameCache cache;
    cache.SetByteCap(1024 * 1024);
    constexpr int kKeys = 16;
    for (int i = 0; i < kKeys; ++i) {
        const RigExecFrameCacheKey key{7, uint64_t(i)};
        CHECK(cache.Publish(key, UsdTimeCode(double(i)), MakePose(double(i), 1)));
    }

    constexpr int kThreads = 4;
    constexpr int kOps = 500;
    std::atomic<int> lookups{0};
    std::atomic<int> publishes{0};
    std::vector<std::thread> workers;
    for (int t = 0; t < kThreads; ++t) {
        workers.emplace_back([&, t]() {
            for (int n = 0; n < kOps; ++n) {
                const RigExecFrameCacheKey key{7, uint64_t((n + t) % kKeys)};
                if (n % 3 == 0) {
                    if (cache.Publish(key, UsdTimeCode(double(n)),
                                      MakePose(double(n), 1))) {
                        ++publishes;
                    }
                } else {
                    RigExecRigPose pose;
                    cache.Lookup(key, &pose);
                    ++lookups;
                }
            }
        });
    }
    for (auto &w : workers) {
        w.join();
    }

    const RigExecFrameCacheStats stats = cache.Stats();
    CHECK(stats.hits + stats.misses == size_t(lookups.load()));
    CHECK(stats.published == size_t(kKeys) + size_t(publishes.load()));
    CHECK(stats.entryCount == size_t(kKeys));
    CHECK(stats.bytes <= cache.GetByteCap());
    CHECK(stats.evictions == 0);

    // Still functional afterwards: a fresh key round-trips.
    const RigExecFrameCacheKey fresh{7, 999};
    CHECK(cache.Publish(fresh, UsdTimeCode(1.0), MakePose(1.0, 1)));
    RigExecRigPose pose;
    CHECK(cache.Lookup(fresh, &pose));
    CHECK(PosesMatch(MakePose(1.0, 1), pose));
}

// Concurrent publishers of one key converge: hundreds of overwrites leave a
// single entry, every publish counted, and the served pose is one of the
// published ones -- valid, never torn.
void
TestConcurrentPublishOfOneKey()
{
    RigExecFrameCache cache;
    constexpr int kThreads = 4;
    constexpr int kOps = 200;
    const RigExecFrameCacheKey key{3, 3};
    std::vector<std::thread> workers;
    for (int t = 0; t < kThreads; ++t) {
        workers.emplace_back([&, t]() {
            for (int n = 0; n < kOps; ++n) {
                CHECK(cache.Publish(key, UsdTimeCode(double(n)),
                                    MakePose(double(t), 1)));
            }
        });
    }
    for (auto &w : workers) {
        w.join();
    }
    const RigExecFrameCacheStats stats = cache.Stats();
    CHECK(stats.published == size_t(kThreads * kOps));
    CHECK(stats.entryCount == 1);
    RigExecRigPose pose;
    CHECK(cache.Lookup(key, &pose));
    CHECK(pose.valid);
    CHECK(pose.jointFramesFinal.size() == 1);
}

// Clear races background publishers without losing count: three threads
// publish and evict across many keys while a fourth clears repeatedly. An
// entry that lands in an already-cleared shard survives with its bytes
// still accounted, so evicting everything at the end brings both counters
// to exactly zero -- never below, where a wrapped byte count would make
// every later publish drain the store.
void
TestClearDuringConcurrentPublishKeepsAccountingHonest()
{
    RigExecFrameCache cache;
    constexpr int kThreads = 3;
    constexpr int kOps = 400;
    constexpr int kKeys = 64;
    std::vector<std::thread> workers;
    for (int t = 0; t < kThreads; ++t) {
        workers.emplace_back([&, t]() {
            for (int n = 0; n < kOps; ++n) {
                const RigExecFrameCacheKey key{5, uint64_t((n * 7 + t) % kKeys)};
                if (n % 5 == 4) {
                    cache.Evict(key);
                } else {
                    cache.Publish(key, UsdTimeCode(double(n)),
                                  MakePose(double(n), 1 + (n % 3)));
                }
            }
        });
    }
    workers.emplace_back([&]() {
        for (int n = 0; n < kOps / 4; ++n) {
            cache.Clear();
        }
    });
    for (auto &w : workers) {
        w.join();
    }

    // Whatever survived is counted, and only that: dropping every key one
    // by one (plus a final Clear for anything the loop's view missed) ends
    // at exactly zero bytes and zero entries.
    for (int i = 0; i < kKeys; ++i) {
        cache.Evict({5, uint64_t(i)});
    }
    cache.Clear();
    const RigExecFrameCacheStats stats = cache.Stats();
    CHECK(stats.entryCount == 0);
    CHECK(stats.bytes == 0);

    // A wrapped counter would evict this immediately after publishing it.
    const RigExecFrameCacheKey fresh{5, 999};
    CHECK(cache.Publish(fresh, UsdTimeCode(1.0), MakePose(1.0, 1)));
    RigExecRigPose pose;
    CHECK(cache.Lookup(fresh, &pose));
    CHECK(cache.Stats().entryCount == 1);
}

}  // namespace

int
main()
{
    TestKeyEqualitySeesBothDigests();
    TestKeyOrderingIsEpochThenControl();
    TestEqualKeysHashEqual();
    TestTimeRidesTheEntry();
    TestAnEmptyCacheMisses();
    TestLookupWithNullPoseMisses();
    TestPublishThenLookupHits();
    TestEitherDigestHalfMisses();
    TestTimeIsNotKeyMaterial();
    TestPublishInvalidPoseDeclines();
    TestEvictDropsAndCounts();
    TestCapRoundTripsAndClearZeroesCounters();
    TestPoseBytesCountsMaps();
    TestDigestIgnoresEnqueueOrder();
    TestDigestSeesValueChanges();
    TestDigestSeesDuplicateValues();
    TestDigestSeesPathsAndTypes();
    TestDigestHitsAcrossUnrelatedEdits();
    TestDigestFirstSampleWins();
    TestDigestSeesOverrides();
    TestDigestIgnoresTime();
    TestDigestibleFlagsUnhashableValues();
    TestRefusalDigestKeysTimeAndOverrides();
    TestLRUEvictsLeastRecentlyUsed();
    TestLookupRefreshesRecency();
    TestOverwriteRefreshesRecency();
    TestOversizedPublishDeclines();
    TestLoweringCapEvictsDown();
    TestConcurrentLookupDuringPublish();
    TestConcurrentPublishOfOneKey();
    TestClearDuringConcurrentPublishKeepsAccountingHonest();

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecFrameCache: all tests passed\n");
    return 0;
}
