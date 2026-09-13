//
// Every shipped example, under the parity check, with a drag standing on it.
//
// The per-example ctest entries (example_parity_*) hold each rig against the
// dynamic path frame by frame; this is the half they cannot reach, because
// rigExecPose has no gizmo. An interactive override the program cannot place
// is not a wrong answer -- the whole generation falls back to the dynamic
// path and every published value still agrees -- so a numeric comparison
// passes and the only visible symptom is that the drag got slow. What this
// suite asserts, per fixture, is therefore that the generations under a drag
// CAME FROM THE PROGRAM, and that the parity check found nothing.
//
// The fixtures come from tests/exampleFixtures.cmake through the generated
// header: one table, read here and by the ctest entries, so the two cannot
// disagree about which frames a rig is tested at or which prim a drag lands
// on. A fixture that does not bake yet is skipped and reported, and turns
// itself on when its operator group removes the refusal.
//
// argv[1] = path to the examples directory.
//
#include "rigExec/rigEvaluator.h"
#include "rigExec/types.h"

#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/base/vt/value.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"

#include <cstdio>
#include <string>
#include <vector>

#include "rigExecExampleFixtures.h"

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

SdfPath
FindRig(const UsdStageRefPtr &stage)
{
    for (const UsdPrim &prim : stage->Traverse()) {
        if (prim.GetTypeName() == "RigExecRoot") {
            return prim.GetPath();
        }
    }
    return SdfPath();
}

std::vector<UsdTimeCode>
ParseFrames(const std::string &text)
{
    std::vector<UsdTimeCode> frames;
    for (const std::string &piece : TfStringSplit(text, ",")) {
        if (!piece.empty()) {
            frames.push_back(UsdTimeCode(TfStringToDouble(piece)));
        }
    }
    return frames;
}

// A drag is a value, and the value has to be the attribute's own type: an
// override holding a double where the input is a float never reaches the
// input, and the generation then falls back for a reason that reads like the
// program declining rather than like the table being wrong.
bool
BumpedValue(const UsdAttribute &attribute, double delta, VtValue *out)
{
    if (!attribute) {
        return false;
    }
    const SdfValueTypeName type = attribute.GetTypeName();
    if (type == SdfValueTypeNames->Double) {
        double value = 0.0;
        attribute.Get(&value, UsdTimeCode::Default());
        *out = VtValue(value + delta);
        return true;
    }
    if (type == SdfValueTypeNames->Float) {
        float value = 0.0f;
        attribute.Get(&value, UsdTimeCode::Default());
        *out = VtValue(float(value + delta));
        return true;
    }
    return false;
}

// One drag, held across the fixture's frames. \p generations is the running
// count the caller has already seen; it is advanced by the frames evaluated
// here, and a mismatch between the two is the whole point of the exercise --
// see the file comment.
//
// Each frame is evaluated TWICE. SetInteractiveOverrides invalidates the
// static input cache on the way in and on the way out, and a cache that
// refilled during the drag gets its chance to hold the overridden value only
// on a second pass.
void
DragOne(const std::string &where, RigExecRigEvaluator *rig,
        const std::vector<RigExecValueOverride> &overrides,
        const std::vector<UsdTimeCode> &frames, size_t *generations)
{
    rig->SetInteractiveOverrides(overrides);
    for (const UsdTimeCode frame : frames) {
        for (int pass = 0; pass < 2; ++pass) {
            const RigExecRigPose pose = rig->Evaluate(frame);
            if (!pose.valid) {
                ++failures;
                std::printf("FAIL %s: an invalid generation under a drag\n",
                            where.c_str());
            }
            if (pose.bakedParityMismatches) {
                ++failures;
                std::printf("FAIL %s: %zu baked parity mismatch(es) at %g\n",
                            where.c_str(), pose.bakedParityMismatches,
                            frame.GetValue());
                for (const std::string &diagnostic : pose.diagnostics) {
                    std::printf("    %s\n", diagnostic.c_str());
                }
            }
            ++*generations;
        }
    }
    // The assertion the numeric comparison cannot make: an override the
    // program could not place sends the generation down the dynamic path,
    // where it computes the same answer more slowly and silently.
    if (rig->GetBakedGenerationCount() != *generations) {
        ++failures;
        std::printf("FAIL %s: %zu of %zu generation(s) came from the "
                    "program; the drag was not placed\n",
                    where.c_str(), rig->GetBakedGenerationCount(),
                    *generations);
    }
    rig->ClearInteractiveOverrides();
}

void
TestOneFixture(const std::string &examplesDir,
               const RigExecExampleFixture &fixture)
{
    const std::string where = fixture.stage;
    const UsdStageRefPtr stage =
        UsdStage::Open(examplesDir + "/" + where);
    CHECK(stage);
    if (!stage) {
        return;
    }
    const SdfPath rigPath = FindRig(stage);
    CHECK(!rigPath.IsEmpty());
    if (rigPath.IsEmpty()) {
        return;
    }
    const std::vector<UsdTimeCode> frames = ParseFrames(fixture.frames);
    CHECK(!frames.empty());
    if (frames.empty()) {
        return;
    }

    RigExecRigEvaluator rig(stage, rigPath);
    // The guide frames are one of the compared domains and the tool's
    // entries ask for them, so this suite must too or the two halves of the
    // table measure different things.
    rig.SetSolverGuidesEnabled(true);
    rig.SetEvaluationMode(RigExecEvaluationMode::BakedWithParityCheck);
    std::vector<std::string> errors;
    if (!rig.Compile(&errors)) {
        ++failures;
        std::printf("FAIL %s: does not compile\n", where.c_str());
        for (const std::string &error : errors) {
            std::printf("    %s\n", error.c_str());
        }
        return;
    }
    // A fixture the table calls bakeable that refuses is a failure here
    // rather than a skip: everything below would then compare the dynamic
    // path with itself and pass having proved nothing.
    std::vector<std::string> reasons;
    if (!rig.IsBakeable(&reasons)) {
        ++failures;
        std::printf("FAIL %s: the table says it bakes, and it declined\n",
                    where.c_str());
        for (const std::string &reason : reasons) {
            std::printf("    %s\n", reason.c_str());
        }
        return;
    }

    // The authored rig first, across every frame of the fixture.
    size_t generations = 0;
    for (const UsdTimeCode frame : frames) {
        const RigExecRigPose pose = rig.Evaluate(frame);
        ++generations;
        CHECK(pose.valid);
        if (pose.bakedParityMismatches) {
            ++failures;
            std::printf("FAIL %s: %zu baked parity mismatch(es) at %g\n",
                        where.c_str(), pose.bakedParityMismatches,
                        frame.GetValue());
            for (const std::string &diagnostic : pose.diagnostics) {
                std::printf("    %s\n", diagnostic.c_str());
            }
        }
    }
    if (rig.GetBakedGenerationCount() != generations) {
        ++failures;
        std::printf("FAIL %s: %zu of %zu authored generation(s) came from "
                    "the program\n", where.c_str(),
                    rig.GetBakedGenerationCount(), generations);
    }

    // The animator's drag: a control avar. Skipped where the example authors
    // no controls -- 04, 06, 07 and the constraint stages pose their rigs
    // from plain Xforms -- which the table records as an empty field rather
    // than leaving a search to guess at.
    if (*fixture.controlPrim) {
        const SdfPath control(fixture.controlPrim);
        const TfToken avar(fixture.controlAvar);
        const UsdPrim prim = stage->GetPrimAtPath(control);
        VtValue dragged;
        if (!prim || !BumpedValue(prim.GetAttribute(avar), 0.5, &dragged)) {
            ++failures;
            std::printf("FAIL %s: no double or float %s on %s\n",
                        where.c_str(), avar.GetText(), control.GetText());
        } else {
            DragOne(where + " (control avar)", &rig,
                    {RigExecValueOverride{control, TfToken(), avar, dragged}},
                    frames, &generations);
        }
    }

    // The rigger's drag: an operator's own input, which places by a
    // different rule -- the prim has to be routed through the generation's
    // resolved inputs, not merely registered as a bound value. Empty only
    // for a stage that authors no operator at all (components/spider_leg.usd
    // is a joint hierarchy and nothing else); the table refuses a record
    // that names one half of the pair, so this cannot hide a typo.
    if (*fixture.operatorPrim) {
        const SdfPath op(fixture.operatorPrim);
        const TfToken input(fixture.operatorInput);
        const UsdPrim prim = stage->GetPrimAtPath(op);
        VtValue dragged;
        if (!prim || !BumpedValue(prim.GetAttribute(input), 0.25, &dragged)) {
            ++failures;
            std::printf("FAIL %s: no double or float %s on %s\n",
                        where.c_str(), input.GetText(), op.GetText());
        } else {
            DragOne(where + " (operator input)", &rig,
                    {RigExecValueOverride{op, TfToken(), input, dragged}},
                    frames, &generations);
        }
    }

    // Releasing a drag must leave the rig on the path it was asked for, not
    // stranded on the one a fallback took.
    const RigExecRigPose released = rig.Evaluate(frames.front());
    ++generations;
    CHECK(released.valid);
    CHECK(released.bakedParityMismatches == 0);
    CHECK(rig.GetEvaluationMode() ==
          RigExecEvaluationMode::BakedWithParityCheck);
    CHECK(rig.GetBakedGenerationCount() == generations);
    std::printf("  %-34s %2zu frame(s), %zu generation(s) from the program\n",
                where.c_str(), frames.size(),
                rig.GetBakedGenerationCount());
}

std::string
SchemaResourceDir(const std::string &examplesDir)
{
#ifdef RIGEXEC_SCHEMA_RESOURCE_DIR
    (void)examplesDir;
    return TfAbsPath(RIGEXEC_SCHEMA_RESOURCE_DIR);
#else
    return TfAbsPath(examplesDir + "/../plugin/rigExecSchema/resources");
#endif
}

}  // namespace

int
main(int argc, char **argv)
{
    if (argc < 2) {
        std::printf("usage: testRigExecExampleParity <examplesDir>\n");
        return 2;
    }
    const std::string examplesDir = argv[1];
    const std::string resources = SchemaResourceDir(examplesDir);
    if (PlugRegistry::GetInstance().RegisterPlugins(resources).empty()) {
        std::printf("FATAL: no schema plugin found at %s\n",
                    resources.c_str());
        return 2;
    }

    size_t baked = 0;
    for (const RigExecExampleFixture &fixture : kRigExecExampleFixtures) {
        if (!fixture.bakesToday) {
            std::printf("  %-34s skipped, waiting on %s\n", fixture.stage,
                        fixture.blockedBy);
            continue;
        }
        TestOneFixture(examplesDir, fixture);
        ++baked;
    }
    // An empty sweep passes silently and proves nothing, and the table
    // shrinking to no bakeable fixture is a regression rather than a rest.
    CHECK(baked > 0);

    std::printf("%s: %zu bakeable fixture(s) of %zu, %d failure(s)\n",
                failures ? "FAILED" : "ok", baked,
                sizeof(kRigExecExampleFixtures) /
                    sizeof(kRigExecExampleFixtures[0]),
                failures);
    return failures ? 1 : 0;
}
