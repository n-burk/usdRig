//
// RIGEXEC_FRAME_CACHE_VERIFY shadow suite (Stream D): every cache hit also
// live-evaluates and diffs through RigExecComparePoses, the same judge as
// BakedWithParityCheck.
//
// A match serves the cached pose; a mismatch reports in the comparator's
// words and serves the live pose instead -- a plausible wrong pose is never
// served. A live runner that fails (or is absent) leaves the hit unverified
// rather than substituted: the shadow only ever substitutes a proven live
// pose.
//
// The test name carries the Cones_ substring so the CI exclusion pattern
// (`-E 'Cones_|ExampleParity'`) catches this suite without edits, the way
// it catches the baked cone suites.
//
// STAGE SHADOWS. The tests above prove the judge on synthetic poses; the
// tests below run it on the validation plan's three rigs -- the biped, the
// animated 9-mesh, and a bake-refusal rig -- with faithful cache keys: each
// probe frame settles live, publishes the settled re-run, and
// shadow-verifies the served hit against a third live run. A one-float
// mutation control on each rig proves a match means something.

#include "rigExec/frameCache.h"
#include "rigExec/frameCacheSparsity.h"
#include "rigExec/frozenContext.h"
#include "rigExec/parallel.h"
#include "rigExec/rigEvaluator.h"

#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/vec3f.h"
#include "pxr/base/tf/setenv.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/base/vt/array.h"
#include "pxr/usd/sdf/path.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"

#include <cstdio>
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

namespace {

RigExecRigPose
MakePose(double tag)
{
    RigExecRigPose pose;
    pose.valid = true;
    pose.time = UsdTimeCode(1.0);
    for (int i = 0; i < 3; ++i) {
        RigExecPointFrame frame;
        frame.points[0] = GfVec3d(tag, double(i), 0.0);
        pose.jointFramesFinal[SdfPath("/Joint" + std::to_string(i))] = frame;
        pose.jointMatricesFinal[SdfPath("/Joint" + std::to_string(i))] =
            GfMatrix4d(1.0);
    }
    pose.diagnostics.push_back("steady");
    return pose;
}

// The bridge's _ComputeCacheKey, for tests that drive the cache directly:
// sample, then the epoch half plus the control digest -- or the
// stage-edit-serial refusal key when there is no baked program.
bool
ShadowKeyFor(RigExecRigEvaluator &evaluator, UsdTimeCode time,
             const std::vector<RigExecValueOverride> &overrides,
             RigExecFrameInputs *inputs, RigExecFrameCacheKey *key)
{
    if (!RigExecSampleFrameInputs(evaluator, time, overrides, inputs)) {
        if (evaluator.GetBakedProgram() != nullptr) {
            return false;
        }
        if (!RigExecRefusalControlDigestible(overrides)) {
            return false;
        }
        key->epochDigest = RigExecFrameCacheEpochDigest(evaluator);
        key->controlDigest = RigExecRefusalControlDigest(
            time, evaluator.GetStageEditSerial(), overrides);
        return true;
    }
    if (!RigExecControlStateDigestible(*inputs, overrides)) {
        return false;
    }
    key->epochDigest = RigExecFrameCacheEpochDigest(evaluator);
    key->controlDigest = RigExecControlStateDigest(*inputs, overrides);
    return true;
}

// The Stream 0 9-mesh rig (testRigExecFrozenContext.cpp,
// MakeAnimated9MeshRig): 9 skinned meshes over two shared controls,
// animated at 1..40.
UsdStageRefPtr
MakeAnimated9MeshRig()
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const UsdPrim alongX = stage->DefinePrim(
        SdfPath("/Asset/Rig/AlongX"), TfToken("RigExecControl"));
    const UsdPrim alongY = stage->DefinePrim(
        SdfPath("/Asset/Rig/AlongY"), TfToken("RigExecControl"));
    UsdAttribute tx = alongX.GetAttribute(TfToken("avars:tx"));
    UsdAttribute ty = alongY.GetAttribute(TfToken("avars:ty"));
    for (int t = 1; t <= 40; ++t) {
        tx.Set(10.0 + 0.1 * double(t), UsdTimeCode(double(t)));
        ty.Set(20.0 - 0.05 * double(t), UsdTimeCode(double(t)));
    }
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));

    static constexpr size_t kMeshes = 9;
    static constexpr size_t kPoints = RigExecGeometryParallelThreshold + 37;
    VtIntArray indices(kPoints * 2);
    for (size_t i = 0; i < kPoints; ++i) {
        indices[i * 2] = 0;
        indices[i * 2 + 1] = 1;
    }
    for (size_t mesh = 0; mesh < kMeshes; ++mesh) {
        const SdfPath meshPath(
            TfStringPrintf("/Asset/Geom/Mesh_%zu", mesh));
        const UsdPrim prim = stage->DefinePrim(meshPath, TfToken("Mesh"));
        VtVec3fArray points(kPoints);
        for (size_t i = 0; i < kPoints; ++i) {
            points[i] = GfVec3f(float(i) * 0.5f + float(mesh),
                                float(i) * -0.25f,
                                float(mesh) * 3.0f - float(i) * 0.125f);
        }
        prim.GetAttribute(TfToken("points")).Set(points);

        const UsdPrim skin = stage->DefinePrim(
            SdfPath(TfStringPrintf("/Asset/Rig/Movers/Skin_%zu", mesh)),
            TfToken("RigExecSkinMover"));
        skin.ApplyAPI(TfToken("RigExecMoverAPI"));
        skin.GetRelationship(TfToken("rigExec:moves"))
            .SetTargets({meshPath.AppendProperty(TfToken("points"))});
        skin.GetAttribute(TfToken("inputs:defaultWeight")).Set(1.0f);
        skin.CreateRelationship(TfToken("rigExec:influences"))
            .SetTargets({alongX.GetPath(), alongY.GetPath()});
        skin.CreateAttribute(TfToken("rigExec:elementSize"),
                             SdfValueTypeNames->Int).Set(2);
        skin.CreateAttribute(TfToken("rigExec:jointIndices"),
                             SdfValueTypeNames->IntArray).Set(indices);
        VtFloatArray weights(kPoints * 2);
        const float xWeight = 0.1f + 0.05f * float(mesh);
        const float yWeight = 0.9f - 0.05f * float(mesh);
        for (size_t i = 0; i < kPoints; ++i) {
            weights[i * 2] = xWeight;
            weights[i * 2 + 1] = yWeight;
        }
        skin.CreateAttribute(TfToken("rigExec:jointWeights"),
                             SdfValueTypeNames->FloatArray).Set(weights);
    }
    return stage;
}

// The bake-refusal rig (testRigExecImagingFrameCache.cpp, MakeRefusalRig):
// the joint's posed:space reads the control's animated posed:space, so the
// value the bake would capture is an exec answer instead. Frames genuinely
// differ, so per-time memos cannot cross-serve undetected.
UsdStageRefPtr
MakeRefusalRig()
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const UsdPrim root = stage->DefinePrim(SdfPath("/Asset/Rig/Root"),
                                           TfToken("RigExecControl"));
    root.GetAttribute(TfToken("avars:tx")).Set(3.0);
    UsdAttribute rootSpace = root.GetAttribute(TfToken("posed:space"));
    if (!rootSpace) {
        rootSpace = root.CreateAttribute(TfToken("posed:space"),
                                         SdfValueTypeNames->Matrix4d);
    }
    for (int t = 1; t <= 4; ++t) {
        GfMatrix4d space(1.0);
        space.SetTranslate(GfVec3d(10.0 * double(t), 0, 0));
        rootSpace.Set(space, UsdTimeCode(double(t)));
    }
    const UsdPrim joint = stage->DefinePrim(SdfPath("/Asset/Rig/Bone"),
                                            TfToken("RigExecJoint"));
    joint.GetAttribute(TfToken("avars:ty")).Set(2.0);
    joint.CreateAttribute(TfToken("posed:space"), SdfValueTypeNames->Matrix4d)
        .AddConnection(root.GetPath().AppendProperty(TfToken("posed:space")));

    stage->DefinePrim(SdfPath("/Asset/Geom"), TfToken("Scope"));
    const UsdPrim mesh = stage->DefinePrim(SdfPath("/Asset/Geom/Slab"),
                                           TfToken("Mesh"));
    VtVec3fArray points{GfVec3f(0, 0, 0), GfVec3f(1, 0, 0), GfVec3f(0, 1, 0)};
    mesh.GetAttribute(TfToken("points")).Set(points);
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const UsdPrim skin = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Skin"), TfToken("RigExecSkinMover"));
    skin.ApplyAPI(TfToken("RigExecMoverAPI"));
    skin.GetRelationship(TfToken("rigExec:moves"))
        .SetTargets({mesh.GetPath().AppendProperty(TfToken("points"))});
    skin.CreateRelationship(TfToken("rigExec:influences"))
        .SetTargets({joint.GetPath()});
    skin.CreateAttribute(TfToken("rigExec:elementSize"),
                         SdfValueTypeNames->Int).Set(1);
    skin.CreateAttribute(TfToken("rigExec:jointIndices"),
                         SdfValueTypeNames->IntArray).Set(VtIntArray{0, 0, 0});
    skin.CreateAttribute(TfToken("rigExec:jointWeights"),
                         SdfValueTypeNames->FloatArray)
        .Set(VtFloatArray{1.0f, 1.0f, 1.0f});
    return stage;
}

// Drives the shadow on one live rig: settle, publish a settled re-run of
// each probe frame under its faithful key, then shadow-verify the served
// hit against a third live run. A one-float mutation control proves the
// judge is live on these poses -- otherwise a match would prove nothing.
void
CheckShadowOnLiveRig(const char *what, RigExecRigEvaluator *evaluator,
                     const std::vector<double> &settle,
                     const std::vector<double> &probe)
{
    for (double t : settle) {
        CHECK(evaluator->Evaluate(UsdTimeCode(t)).valid);
    }
    RigExecFrameCache cache;
    std::vector<RigExecValueOverride> noOverrides;
    RigExecRigPose lastHit;
    double lastTime = 0.0;
    for (double t : probe) {
        const UsdTimeCode time(t);
        CHECK(evaluator->Evaluate(time).valid);
        const RigExecRigPose settled = evaluator->Evaluate(time);
        CHECK(settled.valid);
        RigExecFrameInputs inputs;
        RigExecFrameCacheKey key;
        CHECK(ShadowKeyFor(*evaluator, time, noOverrides, &inputs, &key));
        CHECK(cache.Publish(key, time, settled));
        RigExecRigPose hit;
        CHECK(cache.Lookup(key, &hit));
        const RigExecShadowVerdict verdict = RigExecVerifyHitWithLive(
            hit, [&] { return evaluator->Evaluate(time); });
        if (!verdict.match) {
            std::printf("shadow mismatch on %s at %f:\n%s", what, t,
                        verdict.report.c_str());
        }
        CHECK(verdict.match);
        CHECK(verdict.mismatches == 0);
        lastHit = hit;
        lastTime = t;
    }
    // Sensitivity: the same hit with one float moved must fail the judge.
    RigExecRigPose mutated = lastHit;
    bool touched = false;
    for (auto &entry : mutated.movedProperties) {
        if (entry.second.IsHolding<VtVec3fArray>() &&
            !entry.second.UncheckedGet<VtVec3fArray>().empty()) {
            VtVec3fArray points =
                entry.second.UncheckedGet<VtVec3fArray>();
            points[0][0] += 1.0f;
            entry.second = points;
            touched = true;
            break;
        }
    }
    if (!touched) {
        mutated.diagnostics.push_back("shadow sensitivity probe");
        touched = true;
    }
    CHECK(touched);
    const RigExecShadowVerdict bad = RigExecVerifyHitWithLive(
        mutated, [&] { return evaluator->Evaluate(UsdTimeCode(lastTime)); });
    CHECK(!bad.match);
    CHECK(bad.mismatches > 0);
}

}  // namespace

// The switch reads live: tests toggle it in-process, production pays the
// lookup only on the shadow path.
void
TestVerifySwitch()
{
    TfSetenv("RIGEXEC_FRAME_CACHE_VERIFY", "0");
    CHECK(!RigExecFrameCacheVerifyRequested());
    TfSetenv("RIGEXEC_FRAME_CACHE_VERIFY", "1");
    CHECK(RigExecFrameCacheVerifyRequested());
    TfSetenv("RIGEXEC_FRAME_CACHE_VERIFY", "");
    CHECK(!RigExecFrameCacheVerifyRequested());
}

// A hit the live run agrees with serves the cached pose untouched.
void
TestVerifyMatchServesCached()
{
    const RigExecRigPose cached = MakePose(1.0);
    size_t liveRuns = 0;
    const RigExecShadowVerdict verdict = RigExecVerifyHitWithLive(
        cached, [&cached, &liveRuns]() {
            ++liveRuns;
            return cached;
        });
    CHECK(liveRuns == 1);
    CHECK(verdict.match);
    CHECK(verdict.mismatches == 0);
    CHECK(verdict.report.empty());
    CHECK(verdict.poseToServe.jointFramesFinal ==
          cached.jointFramesFinal);
}

// A hit the live run disagrees with reports the domain and serves the live
// pose: the cached one is never served on a mismatch.
void
TestVerifyMismatchServesLive()
{
    const RigExecRigPose cached = MakePose(1.0);
    RigExecRigPose live = MakePose(2.0);
    const RigExecShadowVerdict verdict = RigExecVerifyHitWithLive(
        cached, [&live]() { return live; });
    CHECK(!verdict.match);
    CHECK(verdict.mismatches == 3);
    CHECK(verdict.report.find("final joint frame") != std::string::npos);
    CHECK(verdict.poseToServe.jointFramesFinal ==
          live.jointFramesFinal);

    // A key the live run never published is its own mismatch domain.
    RigExecRigPose dropped = cached;
    dropped.jointFramesFinal.erase(SdfPath("/Joint1"));
    const RigExecShadowVerdict missing = RigExecVerifyHitWithLive(
        dropped, [&cached]() { return cached; });
    CHECK(!missing.match);
    CHECK(missing.report.find("no final joint frame") != std::string::npos);
    CHECK(missing.poseToServe.jointFramesFinal ==
          cached.jointFramesFinal);
}

// Time and validity are not compared (the parity generation publishes the
// live pose's own): a cached pose from another frame still verifies.
void
TestVerifyIgnoresTime()
{
    const RigExecRigPose cached = MakePose(1.0);
    RigExecRigPose live = cached;
    live.time = UsdTimeCode(9.0);
    const RigExecShadowVerdict verdict = RigExecVerifyHitWithLive(
        cached, [&live]() { return live; });
    CHECK(verdict.match);
}

// A live runner that fails -- or is absent -- leaves the hit unverified
// rather than matched: the shadow substitutes only a proven live pose.
void
TestVerifyLiveFailureIsUnverified()
{
    const RigExecRigPose cached = MakePose(1.0);
    const RigExecShadowVerdict failed = RigExecVerifyHitWithLive(
        cached, []() { return RigExecRigPose(); });
    CHECK(!failed.match);
    CHECK(failed.mismatches == 0);
    CHECK(!failed.report.empty());
    CHECK(failed.poseToServe.jointFramesFinal ==
          cached.jointFramesFinal);

    const RigExecShadowVerdict none =
        RigExecVerifyHitWithLive(cached, nullptr);
    CHECK(!none.match);
    CHECK(!none.report.empty());
    CHECK(none.poseToServe.jointFramesFinal == cached.jointFramesFinal);
}

// The shadow composes with sparse reuse: a planned hit verifies against a
// live run of the request, running the live path exactly once.
void
TestVerifySparseHit()
{
    const RigExecRigPose cached = MakePose(1.0);
    RigExecFrameCache cache;
    const RigExecFrameCacheKey key{7, 11};
    CHECK(cache.Publish(key, UsdTimeCode(1.0), cached));

    RigExecRigPose hit;
    CHECK(cache.Lookup(key, &hit));
    size_t liveRuns = 0;
    const RigExecShadowVerdict verdict = RigExecVerifyHitWithLive(
        hit, [&cached, &liveRuns]() {
            ++liveRuns;
            return cached;
        });
    CHECK(verdict.match);
    CHECK(liveRuns == 1);
    CHECK(verdict.poseToServe.jointFramesFinal ==
          cached.jointFramesFinal);
}

// The biped under the shadow: settle on 1-2, publish settled re-runs of 3
// and 4, and verify each served hit against a third live run.
void
TestShadowOnBiped(const std::string &examplesDir)
{
    const std::string stagePath = examplesDir + "/biped/Biped_anim.usda";
    UsdStageRefPtr stage = UsdStage::Open(stagePath);
    CHECK(stage);
    if (!stage) {
        return;
    }
    SdfPath rig;
    for (const UsdPrim &prim : stage->TraverseAll()) {
        if (prim.GetTypeName() == "RigExecRoot") {
            rig = prim.GetPath();
            break;
        }
    }
    CHECK(!rig.IsEmpty());
    if (rig.IsEmpty()) {
        return;
    }
    RigExecRigEvaluator evaluator(stage, rig);
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    CheckShadowOnLiveRig("biped", &evaluator, {1.0, 2.0}, {3.0, 4.0});
}

// The animated 9-mesh under the shadow, including two far-sweep frames: the
// same settle/publish/verify loop as the biped, at 3, 4, 39, and 40.
void
TestShadowOn9Mesh()
{
    UsdStageRefPtr stage = MakeAnimated9MeshRig();
    const SdfPath rig("/Asset/Rig");
    RigExecRigEvaluator evaluator(stage, rig);
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    std::vector<std::string> reasons;
    if (!evaluator.IsBakeable(&reasons)) {
        ++failures;
        std::printf("FAIL: the 9mesh rig is not bakeable\n");
        return;
    }
    CheckShadowOnLiveRig("9mesh", &evaluator, {1.0, 2.0},
                         {3.0, 4.0, 39.0, 40.0});
}

// A bake-refusal rig under the shadow: refusal keys (no sampling), dynamic
// live runs, same publish/verify loop. The rig genuinely animates, so the
// per-time memos cannot cross-serve undetected.
void
TestShadowOnRefusalRig()
{
    UsdStageRefPtr stage = MakeRefusalRig();
    const SdfPath rig("/Asset/Rig");
    RigExecRigEvaluator evaluator(stage, rig);
    evaluator.SetEvaluationMode(RigExecEvaluationMode::Baked);
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    std::vector<std::string> reasons;
    if (evaluator.IsBakeable(&reasons) ||
        evaluator.GetBakedProgram() != nullptr) {
        ++failures;
        std::printf("FAIL: the refusal rig baked\n");
        return;
    }
    CheckShadowOnLiveRig("refusal", &evaluator, {1.0, 2.0}, {3.0, 4.0});
}

int
main(int argc, char **argv)
{
    TestVerifySwitch();
    TestVerifyMatchServesCached();
    TestVerifyMismatchServesLive();
    TestVerifyIgnoresTime();
    TestVerifyLiveFailureIsUnverified();
    TestVerifySparseHit();
    if (argc > 1) {
        TestShadowOnBiped(argv[1]);
    } else {
        std::printf("skipping the biped shadow (no examples directory)\n");
    }
    TestShadowOn9Mesh();
    TestShadowOnRefusalRig();
    TfSetenv("RIGEXEC_FRAME_CACHE_VERIFY", "");
    if (failures == 0) {
        std::printf("PASS testRigExecFrameCacheCones_Verify\n");
    } else {
        std::printf("FAIL testRigExecFrameCacheCones_Verify: %d failure(s)\n",
                    failures);
    }
    return failures == 0 ? 0 : 1;
}
