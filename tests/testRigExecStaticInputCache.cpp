//
// RigExecStaticInputCache: the three rules it rests on, one test each.
//
// The cache holds one authored value per attribute path and serves it to
// every later read in the generation. That is only sound while the three
// rules below hold, and each of them has a failure mode that no end-to-end
// rig test can see reliably -- a wrong answer that depends on which time
// codes were evaluated first, or a race that fires in one run out of
// thirteen. So they are asserted directly here:
//
//   * ADMISSION. A held attribute must answer the same at every time code a
//     caller can ask for, INCLUDING Default. Neither "has no connections"
//     nor "might not be time varying" is enough on its own: USD reports
//     ValueMightBeTimeVarying() == false for an attribute whose strongest
//     opinion is exactly one time sample of a non-composable type, and a
//     Default read never sees a time sample at all -- so for that attribute
//     Get(Default) and Get(t) legitimately disagree, and an entry keyed by
//     path alone would serve whichever came first to both.
//   * TYPING. An entry answers the type its first read typed it as, and
//     nothing else -- including its "no value" answer, which another type
//     may well not share.
//   * THREAD. The cache is single-threaded state with no lock. A read from
//     any other thread must bounce off it rather than race on it, which is
//     what lets the level-parallel chain walk read through the same resolved
//     inputs.
//
#include "rigExec/moverGraph.h"

#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/stage.h"

#include <cstdio>
#include <string>
#include <thread>

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

namespace {

UsdStageRefPtr
MakeStage()
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Prim"), TfToken("Scope"));
    return stage;
}

UsdAttribute
MakeFloat(const UsdStageRefPtr &stage, const char *name)
{
    return stage->GetPrimAtPath(SdfPath("/Prim"))
        .CreateAttribute(TfToken(name), SdfValueTypeNames->Float);
}

// A plain authored value is held, and held answers are answers: the second
// read of the same attribute never reaches the stage again.
void
TestAPlainValueIsHeld()
{
    UsdStageRefPtr stage = MakeStage();
    const UsdAttribute weight = MakeFloat(stage, "inputs:defaultWeight");
    weight.Set(0.25f);

    RigExecStaticInputCache cache;
    float value = -1.0f;
    bool handled = false;
    CHECK(cache.Read(weight, weight.GetPath(), UsdTimeCode(1.0), &value,
                     &handled));
    CHECK(handled);
    CHECK(value == 0.25f);
    CHECK(cache.GetSize() == 1);
    CHECK(cache.GetHitCount() == 0);  // the first read FILLED the entry

    value = -1.0f;
    CHECK(cache.Read(weight, weight.GetPath(), UsdTimeCode(2.0), &value,
                     &handled));
    CHECK(handled);
    CHECK(value == 0.25f);
    CHECK(cache.GetHitCount() == 1);

    // Clear is the whole of invalidation: the owner calls it on every notice.
    cache.Clear();
    CHECK(cache.GetSize() == 0);
    weight.Set(0.75f);
    CHECK(cache.Read(weight, weight.GetPath(), UsdTimeCode(1.0), &value,
                     &handled));
    CHECK(value == 0.75f);
}

// A connected attribute is refused: the connection is where the value comes
// from, and it can reach something animated.
void
TestAConnectedValueIsRefused()
{
    UsdStageRefPtr stage = MakeStage();
    const UsdAttribute weight = MakeFloat(stage, "inputs:defaultWeight");
    const UsdAttribute source = MakeFloat(stage, "inputs:source");
    source.Set(0.5f);
    weight.Set(0.25f);
    CHECK(weight.AddConnection(source.GetPath()));

    RigExecStaticInputCache cache;
    float value = -1.0f;
    bool handled = true;
    cache.Read(weight, weight.GetPath(), UsdTimeCode(1.0), &value, &handled);
    CHECK(!handled);
    CHECK(cache.GetRefusalCount() == 1);
}

// Two or more time samples: time-varying by anybody's definition.
void
TestATimeVaryingValueIsRefused()
{
    UsdStageRefPtr stage = MakeStage();
    const UsdAttribute weight = MakeFloat(stage, "inputs:defaultWeight");
    weight.Set(0.25f, UsdTimeCode(1.0));
    weight.Set(0.75f, UsdTimeCode(2.0));
    CHECK(weight.ValueMightBeTimeVarying());

    RigExecStaticInputCache cache;
    float value = -1.0f;
    bool handled = true;
    cache.Read(weight, weight.GetPath(), UsdTimeCode(1.0), &value, &handled);
    CHECK(!handled);
}

// EXACTLY ONE time sample: USD says it is not time-varying, and a Default
// read still cannot see it. The attribute therefore has two legitimate
// answers, so the cache must hold neither.
void
TestASingleTimeSampleIsRefused()
{
    UsdStageRefPtr stage = MakeStage();
    const UsdAttribute weight = MakeFloat(stage, "inputs:defaultWeight");
    weight.Set(0.75f, UsdTimeCode(1.0));
    // The premise of this test, stated as an assertion so it fails loudly if
    // a future USD changes the rule rather than silently testing nothing.
    CHECK(weight.GetNumTimeSamples() == 1);
    CHECK(!weight.ValueMightBeTimeVarying());
    CHECK(!weight.HasAuthoredConnections());
    float atDefault = -1.0f, atFrame = -1.0f;
    const bool gotDefault = weight.Get(&atDefault, UsdTimeCode::Default());
    CHECK(weight.Get(&atFrame, UsdTimeCode(1.0)));
    CHECK(atFrame == 0.75f);
    CHECK(!gotDefault || atDefault != atFrame);

    RigExecStaticInputCache cache;
    float value = -1.0f;
    bool handled = true;
    cache.Read(weight, weight.GetPath(), UsdTimeCode::Default(), &value,
               &handled);
    CHECK(!handled);

    // And the order the two reads arrive in changes nothing, which is the
    // property the whole admission rule exists for.
    RigExecStaticInputCache other;
    handled = true;
    other.Read(weight, weight.GetPath(), UsdTimeCode(1.0), &value, &handled);
    CHECK(!handled);
}

// An entry answers one type. A first read that found nothing for type A must
// not be served as "nothing" to a later read of type B.
void
TestAnEntryAnswersOnlyItsOwnType()
{
    UsdStageRefPtr stage = MakeStage();
    const UsdAttribute token =
        stage->GetPrimAtPath(SdfPath("/Prim"))
            .CreateAttribute(TfToken("rigExec:skinningMethod"),
                             SdfValueTypeNames->Token);
    token.Set(TfToken("dualQuaternion"));

    RigExecStaticInputCache cache;
    // Read it as a float first: nothing to find, and the entry is typed
    // float from here on.
    float wrongType = -1.0f;
    bool handled = false;
    CHECK(!cache.Read(token, token.GetPath(), UsdTimeCode(1.0), &wrongType,
                      &handled));
    CHECK(handled);

    // The same attribute read as what it actually is.
    TfToken value;
    CHECK(cache.Read(token, token.GetPath(), UsdTimeCode(1.0), &value,
                     &handled));
    CHECK(value == TfToken("dualQuaternion"));
}

// The thread guard. Reading from a thread that does not own the cache must
// report "not handled" -- so the caller reads the long way -- and must leave
// the map untouched, because touching it is the race.
void
TestAReadFromAnotherThreadBypasses()
{
    UsdStageRefPtr stage = MakeStage();
    const UsdAttribute weight = MakeFloat(stage, "inputs:defaultWeight");
    weight.Set(0.25f);

    RigExecStaticInputCache cache;
    float value = -1.0f;
    bool handled = false;
    CHECK(cache.Read(weight, weight.GetPath(), UsdTimeCode(1.0), &value,
                     &handled));
    CHECK(handled);
    const size_t sizeBefore = cache.GetSize();
    const size_t hitsBefore = cache.GetHitCount();

    bool workerHandled = true;
    float workerValue = -1.0f;
    bool workerResult = true;
    std::thread worker([&]() {
        workerResult = cache.Read(weight, weight.GetPath(), UsdTimeCode(1.0),
                                  &workerValue, &workerHandled);
    });
    worker.join();
    CHECK(!workerHandled);
    CHECK(!workerResult);
    CHECK(cache.GetBypassCount() == 1);
    // Nothing was read, inserted or counted on the worker's behalf.
    CHECK(cache.GetSize() == sizeBefore);
    CHECK(cache.GetHitCount() == hitsBefore);

    // The long way round -- what the caller does with handled == false --
    // still gives the authored value, so the bypass costs correctness
    // nothing.
    float direct = -1.0f;
    CHECK(weight.Get(&direct, UsdTimeCode(1.0)));
    CHECK(direct == 0.25f);

    // A previously unseen attribute read from a worker must not create an
    // entry either.
    const UsdAttribute other = MakeFloat(stage, "inputs:other");
    other.Set(0.5f);
    std::thread second([&]() {
        float unused = -1.0f;
        bool unusedHandled = true;
        cache.Read(other, other.GetPath(), UsdTimeCode(1.0), &unused,
                   &unusedHandled);
        CHECK(!unusedHandled);
    });
    second.join();
    CHECK(cache.GetSize() == sizeBefore);
    CHECK(cache.GetBypassCount() == 2);

    // Clear() re-stamps the owner, which is how a cache moves threads when
    // its owner does.
    std::thread newOwner([&]() {
        cache.Clear();
        float v = -1.0f;
        bool h = false;
        CHECK(cache.Read(weight, weight.GetPath(), UsdTimeCode(1.0), &v, &h));
        CHECK(h);
        CHECK(v == 0.25f);
    });
    newOwner.join();
}

}  // namespace

int
main()
{
    TestAPlainValueIsHeld();
    TestAConnectedValueIsRefused();
    TestATimeVaryingValueIsRefused();
    TestASingleTimeSampleIsRefused();
    TestAnEntryAnswersOnlyItsOwnType();
    TestAReadFromAnotherThreadBypasses();

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecStaticInputCache: all tests passed\n");
    return 0;
}
