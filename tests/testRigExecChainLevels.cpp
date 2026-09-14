//
// Independent geometry chains are walked one task per chain, which is only
// worth doing if the rig cannot tell. Three things have to hold, and each is
// asserted here against the same rig:
//
//  - the same numbers. A chain's points must come out bit for bit as they do
//    when the whole walk is serial -- not close, identical -- whether the
//    walk is spread over the machine, pinned to one thread, or taken down the
//    serial branch entirely by the kill switch (ctest runs this suite a
//    second time with RIGEXEC_ENABLE_PARALLEL_EVAL=0).
//  - the same order. Diagnostics are merged in chain order, not completion
//    order, so a rig that reports two failures reports them in the same
//    sequence on every run. A mutex around the diagnostics vector would make
//    the walk safe and still fail this.
//  - the same classification. A level is spread out only when Compile said
//    it may be: too few chains to pay for the dispatch, a weight object two
//    chains share, or a phased read one of them makes, and the level is
//    walked in order like it always was.
//
#include "rigExec/parallel.h"
#include "rigExec/rigEvaluator.h"

#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/stringUtils.h"
#include "pxr/base/work/threadLimits.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/relationship.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

using namespace rigExec;

static int failures = 0;
#define CHECK(condition) do { if (!(condition)) { ++failures; \
    std::printf("FAIL %d: %s\n", __LINE__, #condition); } } while (0)

namespace {

// Above RigExecGeometryParallelThreshold on purpose: then the per-point
// kernels dispatch too, and this is also the test of a chain task opening a
// parallel region of its own.
constexpr size_t kPointCount = RigExecGeometryParallelThreshold + 37;

// Meshes 1 and 4 carry an out-of-range envelope, so those two chains fail and
// say so. Two failures out of six is what makes the ORDER of the diagnostics
// mean something.
constexpr size_t kFailingMesh[] = {1, 4};

SdfPath
MeshPath(size_t mesh)
{
    return SdfPath(TfStringPrintf("/Asset/Geom/Mesh_%zu", mesh));
}

SdfPath
TargetPath(size_t mesh)
{
    return MeshPath(mesh).AppendProperty(TfToken("points"));
}

SdfPath
MoverPath(size_t mesh)
{
    return SdfPath(TfStringPrintf("/Asset/Rig/Movers/Skin_%zu", mesh));
}

// Distinct geometry per mesh, so a chain that published another chain's
// result would be caught by the values and not only by a count.
VtVec3fArray
BasePoints(size_t mesh)
{
    VtVec3fArray points(kPointCount);
    for (size_t i = 0; i < kPointCount; ++i) {
        points[i] = GfVec3f(float(i) * 0.5f + float(mesh),
                            float(i) * -0.25f,
                            float(mesh) * 3.0f - float(i) * 0.125f);
    }
    return points;
}

// Per-mesh influence weights, also distinct: mesh m moves by
// (0.1 + 0.05 m) * (10, 0, 0) + (0.9 - 0.05 m) * (0, 20, 0).
float AlongX(size_t mesh) { return 0.1f + 0.05f * float(mesh); }
float AlongY(size_t mesh) { return 0.9f - 0.05f * float(mesh); }

// \p meshCount skinned meshes over two shared controls. Every mesh is
// authored the same way whatever meshCount is, so the same mesh in a rig of
// two and in a rig of six is the same computation -- which is what lets a
// serial walk and a parallel one be compared value by value.
UsdStageRefPtr
MakeMultiMeshRig(size_t meshCount)
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const UsdPrim alongX = stage->DefinePrim(
        SdfPath("/Asset/Rig/AlongX"), TfToken("RigExecControl"));
    alongX.GetAttribute(TfToken("avars:tx")).Set(10.0);
    const UsdPrim alongY = stage->DefinePrim(
        SdfPath("/Asset/Rig/AlongY"), TfToken("RigExecControl"));
    alongY.GetAttribute(TfToken("avars:ty")).Set(20.0);
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));

    VtIntArray indices(kPointCount * 2);
    for (size_t i = 0; i < kPointCount; ++i) {
        indices[i * 2] = 0;
        indices[i * 2 + 1] = 1;
    }
    for (size_t mesh = 0; mesh < meshCount; ++mesh) {
        const UsdPrim prim =
            stage->DefinePrim(MeshPath(mesh), TfToken("Mesh"));
        prim.GetAttribute(TfToken("points")).Set(BasePoints(mesh));

        const UsdPrim skin =
            stage->DefinePrim(MoverPath(mesh), TfToken("RigExecSkinMover"));
        skin.ApplyAPI(TfToken("RigExecMoverAPI"));
        skin.GetRelationship(TfToken("rigExec:moves"))
            .SetTargets({TargetPath(mesh)});
        const bool fails = mesh == kFailingMesh[0] || mesh == kFailingMesh[1];
        skin.GetAttribute(TfToken("inputs:defaultWeight"))
            .Set(fails ? 2.0f : 1.0f);
        skin.CreateRelationship(TfToken("rigExec:influences"))
            .SetTargets({alongX.GetPath(), alongY.GetPath()});
        skin.CreateAttribute(TfToken("rigExec:elementSize"),
                             SdfValueTypeNames->Int).Set(2);
        skin.CreateAttribute(TfToken("rigExec:jointIndices"),
                             SdfValueTypeNames->IntArray).Set(indices);
        VtFloatArray weights(kPointCount * 2);
        for (size_t i = 0; i < kPointCount; ++i) {
            weights[i * 2] = AlongX(mesh);
            weights[i * 2 + 1] = AlongY(mesh);
        }
        skin.CreateAttribute(TfToken("rigExec:jointWeights"),
                             SdfValueTypeNames->FloatArray).Set(weights);
    }
    return stage;
}

bool
CompileOrReport(RigExecRigEvaluator *evaluator, const char *what)
{
    std::vector<std::string> errors;
    if (evaluator->Compile(&errors)) {
        return true;
    }
    ++failures;
    std::printf("FAIL: %s did not compile\n", what);
    for (const std::string &error : errors) {
        std::printf("  %s\n", error.c_str());
    }
    return false;
}

VtVec3fArray
MovedPoints(const RigExecRigPose &pose, size_t mesh)
{
    const auto found = pose.movedProperties.find(TargetPath(mesh));
    if (found == pose.movedProperties.end()) {
        return VtVec3fArray();
    }
    return found->second.Get<VtVec3fArray>();
}

// Bit for bit. GfIsClose would pass on a result that was assembled from the
// wrong revision of an input and happened to land nearby.
bool
Identical(const VtVec3fArray &a, const VtVec3fArray &b, const char *what)
{
    if (a.size() != b.size()) {
        ++failures;
        std::printf("FAIL: %s: %zu points vs %zu\n", what, a.size(), b.size());
        return false;
    }
    for (size_t i = 0; i < a.size(); ++i) {
        if (a[i][0] != b[i][0] || a[i][1] != b[i][1] || a[i][2] != b[i][2]) {
            ++failures;
            std::printf("FAIL: %s: point %zu (%.9g %.9g %.9g) vs "
                        "(%.9g %.9g %.9g)\n", what, i,
                        double(a[i][0]), double(a[i][1]), double(a[i][2]),
                        double(b[i][0]), double(b[i][1]), double(b[i][2]));
            return false;
        }
    }
    return true;
}

// A skinned mesh whose envelope is valid lands exactly on the analytic
// answer's neighbourhood; a failed one is passed through untouched. Both are
// checked, because "every chain returned its base" would otherwise satisfy
// every equality in this file.
void
CheckExpectedDeformation(const RigExecRigPose &pose, size_t mesh,
                         const char *what)
{
    const VtVec3fArray points = MovedPoints(pose, mesh);
    const VtVec3fArray base = BasePoints(mesh);
    CHECK(points.size() == kPointCount);
    if (points.size() != kPointCount) {
        std::printf("  (%s: mesh %zu published nothing)\n", what, mesh);
        return;
    }
    const bool fails = mesh == kFailingMesh[0] || mesh == kFailingMesh[1];
    for (size_t i = 0; i < kPointCount; ++i) {
        const GfVec3f expected =
            fails ? base[i]
                  : base[i] + GfVec3f(AlongX(mesh) * 10.0f,
                                      AlongY(mesh) * 20.0f, 0.0f);
        // RELATIVE, not absolute. The points run out past x = 1500, where
        // one float32 ULP is already 1.22e-4 -- so a fixed 1e-4 is a
        // tolerance narrower than the type can represent, and the engine's
        // accumulation order differing from this line's by a single ulp
        // fails it. Scaled by magnitude, the check still catches a chain
        // publishing the wrong mesh's points (they differ by whole units).
        const double scale =
            std::max(1.0, double(GfVec3d(expected).GetLength()));
        if ((GfVec3d(points[i]) - GfVec3d(expected)).GetLength()
                >= 1e-6 * scale) {
            ++failures;
            std::printf("FAIL: %s: mesh %zu point %zu is (%g %g %g), "
                        "expected (%g %g %g)\n", what, mesh, i,
                        double(points[i][0]), double(points[i][1]),
                        double(points[i][2]), double(expected[0]),
                        double(expected[1]), double(expected[2]));
            return;
        }
    }
}

// Six independent chains are one level, and that level is the one the walk
// may spread out.
void
TestIndependentChainsAreOneParallelLevel()
{
    UsdStageRefPtr stage = MakeMultiMeshRig(6);
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    if (!CompileOrReport(&evaluator, "six independent meshes")) {
        return;
    }
    CHECK(evaluator.GetChainLevelCount() == 1);
    CHECK(evaluator.IsChainLevelParallel(0));
    // Out of range is not parallel, and does not crash.
    CHECK(!evaluator.IsChainLevelParallel(1));
    CHECK(evaluator.GetChainLevelTargets(1).empty());

    const std::vector<SdfPath> targets = evaluator.GetChainLevelTargets(0);
    CHECK(targets.size() == 6);
    for (size_t mesh = 0; mesh < targets.size() && mesh < 6; ++mesh) {
        // Chain order is the compiled order, which for independent chains is
        // path order -- Mesh_0 .. Mesh_5.
        CHECK(targets[mesh] == TargetPath(mesh));
    }
}

// Two chains are not enough work to be worth a dispatch, and a rig that holds
// exactly the meshes of a bigger one produces exactly the bigger one's values
// for them.
void
TestSerialAndParallelWalksAgreeExactly()
{
    UsdStageRefPtr wide = MakeMultiMeshRig(6);
    RigExecRigEvaluator parallel(wide, SdfPath("/Asset/Rig"));
    if (!CompileOrReport(&parallel, "six independent meshes")) {
        return;
    }
    CHECK(parallel.IsChainLevelParallel(0));
    const RigExecRigPose spread = parallel.Evaluate(UsdTimeCode::Default());
    CHECK(spread.valid);

    UsdStageRefPtr narrow = MakeMultiMeshRig(2);
    RigExecRigEvaluator serial(narrow, SdfPath("/Asset/Rig"));
    if (!CompileOrReport(&serial, "two independent meshes")) {
        return;
    }
    CHECK(serial.GetChainLevelCount() == 1);
    CHECK(!serial.IsChainLevelParallel(0));
    const RigExecRigPose inOrder = serial.Evaluate(UsdTimeCode::Default());
    CHECK(inOrder.valid);

    for (size_t mesh = 0; mesh < 2; ++mesh) {
        Identical(MovedPoints(spread, mesh), MovedPoints(inOrder, mesh),
                  "parallel level vs serial walk");
    }
    for (size_t mesh = 0; mesh < 6; ++mesh) {
        CheckExpectedDeformation(spread, mesh, "parallel level");
    }
    CheckExpectedDeformation(inOrder, 0, "serial walk");
    CheckExpectedDeformation(inOrder, 1, "serial walk");

    // The serial rig's own diagnostics are the wide rig's, restricted to the
    // meshes it has: same messages, same relative order.
    std::vector<std::string> wideFailures;
    for (const std::string &message : spread.diagnostics) {
        if (message.find("Skin_0") != std::string::npos ||
            message.find("Skin_1") != std::string::npos) {
            wideFailures.push_back(message);
        }
    }
    std::vector<std::string> narrowFailures;
    for (const std::string &message : inOrder.diagnostics) {
        if (message.find("Skin_0") != std::string::npos ||
            message.find("Skin_1") != std::string::npos) {
            narrowFailures.push_back(message);
        }
    }
    CHECK(!narrowFailures.empty());
    CHECK(wideFailures == narrowFailures);
}

// Same evaluator, same stage, several generations: identical values and an
// identical diagnostics sequence every time. Merging per chain in chain order
// is what this asserts; a mutex around the shared vector would not pass it.
void
TestRepeatedWalksAreIdentical()
{
    UsdStageRefPtr stage = MakeMultiMeshRig(6);
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    if (!CompileOrReport(&evaluator, "six independent meshes")) {
        return;
    }
    // The first generation of a rig creates its graphs and executes every
    // revision; steady state is what repeats, so that is what is compared.
    CHECK(evaluator.Evaluate(UsdTimeCode::Default()).valid);
    const RigExecRigPose first = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(first.valid);

    // Both failing chains reported, and reported in chain order.
    std::vector<std::string> failureOrder;
    for (const std::string &message : first.diagnostics) {
        if (message.find("MoverFailed") != std::string::npos) {
            failureOrder.push_back(message);
        }
    }
    CHECK(failureOrder.size() >= 2);
    if (failureOrder.size() >= 2) {
        const std::string firstMover =
            MoverPath(kFailingMesh[0]).GetString();
        const std::string secondMover =
            MoverPath(kFailingMesh[1]).GetString();
        CHECK(failureOrder.front().find(firstMover) != std::string::npos);
        CHECK(failureOrder.back().find(secondMover) != std::string::npos);
    }

    // Sixty generations, not a handful. A memory-safety regression in the
    // parallel walk -- an insertion into a map a sibling task is reading --
    // is probabilistic per generation, and one generation is about the
    // smallest window this rig can offer it. The deterministic half of the
    // same invariant is TestTheParallelWalkCreatesNoGraphNode below; this is
    // the half that catches the ones an invariant cannot name.
    const size_t graphNodes = evaluator.GetLiveGraphCount();
    for (int run = 0; run < 60; ++run) {
        const RigExecRigPose again = evaluator.Evaluate(UsdTimeCode::Default());
        CHECK(again.valid);
        CHECK(again.diagnostics == first.diagnostics);
        CHECK(again.movedProperties.size() == first.movedProperties.size());
        CHECK(again.moverGraphRevisionsExecuted ==
              first.moverGraphRevisionsExecuted);
        CHECK(evaluator.GetLiveGraphCount() == graphNodes);
        for (size_t mesh = 0; mesh < 6; ++mesh) {
            if (!Identical(MovedPoints(again, mesh), MovedPoints(first, mesh),
                           "repeated walk")) {
                return;  // one report is enough
            }
        }
    }
}

// The invariant that makes the parallel walk memory-safe, asserted instead
// of hoped for.
//
// Chains in one level run concurrently and each indexes _liveGraphs by its
// own target; Compile pre-creates every node so that indexing is a lookup
// and never an insertion. Delete the pre-creation and the walk inserts into
// a std::map that sibling tasks are reading -- which crashes in a quarter of
// runs and passes in the rest, so a single pass of the suite is not evidence.
// The node count not moving across a parallel generation is evidence.
void
TestTheParallelWalkCreatesNoGraphNode()
{
    UsdStageRefPtr stage = MakeMultiMeshRig(6);
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    if (!CompileOrReport(&evaluator, "six independent meshes")) {
        return;
    }
    CHECK(evaluator.IsChainLevelParallel(0));
    // One per chain target, created at Compile and before any Evaluate.
    const size_t nodes = evaluator.GetLiveGraphCount();
    CHECK(nodes >= 6);
    for (int run = 0; run < 8; ++run) {
        CHECK(evaluator.Evaluate(UsdTimeCode::Default()).valid);
        CHECK(evaluator.GetLiveGraphCount() == nodes);
    }
}

// The same binary, the same code path, one thread instead of twenty. This is
// the half of "bit-identical" that the kill switch cannot answer: the tasks
// still run through the dispatcher, they just cannot overlap.
void
TestOneThreadAndManyAgree()
{
    UsdStageRefPtr stage = MakeMultiMeshRig(6);
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    if (!CompileOrReport(&evaluator, "six independent meshes")) {
        return;
    }
    CHECK(evaluator.Evaluate(UsdTimeCode::Default()).valid);
    const RigExecRigPose many = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(many.valid);

    const unsigned limit = WorkGetConcurrencyLimit();
    WorkSetConcurrencyLimit(1);
    const RigExecRigPose one = evaluator.Evaluate(UsdTimeCode::Default());
    WorkSetConcurrencyLimit(limit);

    CHECK(one.valid);
    CHECK(one.diagnostics == many.diagnostics);
    for (size_t mesh = 0; mesh < 6; ++mesh) {
        Identical(MovedPoints(one, mesh), MovedPoints(many, mesh),
                  "one thread vs many");
    }
}

// Weight objects in a parallel level: each chain publishes its own resolved
// field, and the fields are merged in chain order like everything else.
//
// (A weight object BOUND BY TWO CHAINS would keep the level in order --
// Compile checks for that -- but the schema validation reaches it first: a
// weight object's target has to be the mover's own target, so two chains
// cannot name one today. The classification does not rely on that staying
// true.)
void
TestWeightObjectsInAParallelLevel()
{
    UsdStageRefPtr stage = MakeMultiMeshRig(6);
    const size_t halved[] = {0, 3};
    for (size_t mesh : halved) {
        const SdfPath weightPath(
            TfStringPrintf("/Asset/Rig/Weights/Half_%zu", mesh));
        const UsdPrim weight =
            stage->DefinePrim(weightPath, TfToken("RigExecStaticWeight"));
        weight.GetAttribute(TfToken("rigExec:representation"))
            .Set(TfToken("constant"));
        weight.GetAttribute(TfToken("rigExec:defaultWeight")).Set(0.5f);
        weight.GetRelationship(TfToken("rigExec:weightTarget"))
            .SetTargets({TargetPath(mesh)});
        stage->GetPrimAtPath(MoverPath(mesh))
            .CreateRelationship(TfToken("rigExec:weightObject"))
            .SetTargets({weightPath});
    }
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    if (!CompileOrReport(&evaluator, "per-chain weight objects")) {
        return;
    }
    CHECK(evaluator.GetChainLevelCount() == 1);
    CHECK(evaluator.IsChainLevelParallel(0));

    const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(pose.valid);
    // Both fields published, one per chain, and to the right chain.
    CHECK(pose.weightFields.size() == 2);
    for (size_t mesh : halved) {
        const auto field = pose.weightFields.find(
            SdfPath(TfStringPrintf("/Asset/Rig/Weights/Half_%zu", mesh)));
        CHECK(field != pose.weightFields.end());
        if (field != pose.weightFields.end()) {
            CHECK(field->second.target == TargetPath(mesh));
            CHECK(field->second.weights.size() == kPointCount);
        }
        const VtVec3fArray points = MovedPoints(pose, mesh);
        const VtVec3fArray base = BasePoints(mesh);
        CHECK(points.size() == kPointCount);
        if (points.size() == kPointCount) {
            const GfVec3f half =
                base[0] + GfVec3f(AlongX(mesh) * 10.0f,
                                  AlongY(mesh) * 20.0f, 0.0f) * 0.5f;
            CHECK((GfVec3d(points[0]) - GfVec3d(half)).GetLength() < 1e-3);
        }
    }
    // And a chain beside them, with no weight object at all, is untouched by
    // the fact that its neighbours have one.
    CheckExpectedDeformation(pose, 2, "full-strength chain beside halved ones");
}

// One chain reading another at a phase is a dependency: the two cannot be in
// the same level, and the level that holds the reader is walked in order
// because what it would read is recorded by the walk itself.
void
TestPhasedReadSplitsAndSerializesLevels()
{
    UsdStageRefPtr stage = MakeMultiMeshRig(6);
    // Mesh_5 is also deformed by a lattice whose cage is Mesh_0 as Mesh_0's
    // own chain leaves it.
    const UsdPrim lattice = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Lattice_5"),
        TfToken("RigExecLatticeMover"));
    lattice.ApplyAPI(TfToken("RigExecMoverAPI"));
    lattice.GetRelationship(TfToken("rigExec:moves"))
        .SetTargets({TargetPath(5)});
    const UsdRelationship cage =
        lattice.CreateRelationship(TfToken("rigExec:cage"));
    cage.SetTargets({MeshPath(0)});
    cage.SetMetadata(TfToken("rigExecReadPhase"), std::string("final"));

    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    if (!CompileOrReport(&evaluator, "phased cage read")) {
        return;
    }
    CHECK(evaluator.GetChainLevelCount() == 2);
    // The five chains nothing reads are still one parallel level; the reader
    // is what had to move out of it.
    CHECK(evaluator.IsChainLevelParallel(0));
    CHECK(evaluator.GetChainLevelTargets(0).size() == 5);
    CHECK(!evaluator.IsChainLevelParallel(1));
    // The reader is alone in the second level, after the chain it reads.
    const std::vector<SdfPath> second = evaluator.GetChainLevelTargets(1);
    CHECK(second.size() == 1);
    if (second.size() == 1) {
        CHECK(second[0] == TargetPath(5));
    }
    const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(pose.valid);
    CHECK(MovedPoints(pose, 0).size() == kPointCount);
}

}  // namespace

// argv[1], optional: run every case this many times.
//
// The two guards that make the level-parallel walk memory-safe fail
// PROBABILISTICALLY when they are broken -- a broken build passes a single
// run about three times in four. The invariants asserted above catch the two
// known ways to break them; repeating catches the rest, and the whole file
// costs well under a tenth of a second, so one entry runs it many times over.
int
main(int argc, char **argv)
{
    // The codeless schema has to be loadable before a RigExecControl means
    // anything to exec; ctest runs this with no plugin path set.
    PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);

    const int rounds = argc > 1 ? std::max(1, std::atoi(argv[1])) : 1;
    for (int round = 0; round < rounds && !failures; ++round) {
        TestIndependentChainsAreOneParallelLevel();
        TestSerialAndParallelWalksAgreeExactly();
        TestRepeatedWalksAreIdentical();
        TestTheParallelWalkCreatesNoGraphNode();
        TestOneThreadAndManyAgree();
        TestWeightObjectsInAParallelLevel();
        TestPhasedReadSplitsAndSerializesLevels();
    }

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecChainLevels: all tests passed\n");
    return 0;
}
