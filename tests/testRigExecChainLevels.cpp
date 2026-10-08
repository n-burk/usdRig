// Common producer graph tests: independent geometry branches, exact serial/parallel
// values, stable held results and causal phased reads.
#include "rigExec/parallel.h"
#include "rigExec/rigEvaluator.h"
#include "rigExecGraphDependencyCheck.h"

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
CompileOrReport(RigExecRigEvaluator *evaluator, const char *what,
                RigExecRigPose *firstPose=nullptr)
{
    std::vector<std::string> errors;
    if (evaluator->Compile(&errors)) {
        auto pose=evaluator->Evaluate(UsdTimeCode::Default());
        if(pose.valid && evaluator->GetBakedProgram()) {
            if(firstPose)*firstPose=std::move(pose);
            return true;
        }
        errors.insert(errors.end(),pose.diagnostics.begin(),pose.diagnostics.end());
    }
    ++failures;
    std::printf("FAIL: %s has no executable compiled program\n", what);
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
// The neighbourhood is RELATIVE, because the published points are float32
// and the fixture's coordinates reach ~1600. One ulp there is 1.2e-4, so an
// absolute 1e-4 asked for agreement finer than the type can represent and
// passed or failed on which way the last bit of a weighted sum happened to
// round -- it held under gcc and clang and did not under MSVC, on a result
// that was correct on both. What the test is actually asserting is that the
// skin RAN and landed on the analytic answer rather than somewhere else
// entirely; the deformation it is separating is 10 and 20 units, so a
// tolerance a few ulps wide still fails every wrong answer this file can
// produce. Bit-exactness is asserted where it belongs, by Identical()
// above, which compares two runs of the same arithmetic.
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
        // Four float ulps of the point's distance from the origin,
        // never below the absolute floor a point near it needs.
        const double magnitude =
            std::max(GfVec3d(expected).GetLength(), 1.0);
        const double tolerance = std::max(1e-4, magnitude * 5e-7);
        if ((GfVec3d(points[i]) - GfVec3d(expected)).GetLength() >=
                tolerance) {
            ++failures;
            std::printf("FAIL: %s: mesh %zu point %zu is (%.9g %.9g %.9g), "
                        "expected (%.9g %.9g %.9g)\n", what, mesh, i,
                        double(points[i][0]), double(points[i][1]),
                        double(points[i][2]), double(expected[0]),
                        double(expected[1]), double(expected[2]));
            return;
        }
    }
}

// Independent chain outputs have no causal paths between them.
void
TestIndependentChainsHaveNoCausalEdges()
{
    UsdStageRefPtr stage = MakeMultiMeshRig(6);
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    if (!CompileOrReport(&evaluator, "six independent meshes")) {
        return;
    }
    const auto graph=evaluator.GetOpGraph();
    std::vector<size_t> outputs;
    for(size_t mesh=0;mesh<6;++mesh) {
        const auto id=rigExecTest::FindOperation(graph,TargetPath(mesh),"ChainStatus");
        CHECK(id<graph.size());outputs.push_back(id);
    }
    for(size_t a=0;a<outputs.size();++a)for(size_t b=a+1;b<outputs.size();++b) {
        CHECK(!rigExecTest::HasDependencyPath(graph,outputs[a],outputs[b]));
        CHECK(!rigExecTest::HasDependencyPath(graph,outputs[b],outputs[a]));
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
    CHECK(!parallel.GetOpGraph().empty());
    const RigExecRigPose spread = parallel.Evaluate(UsdTimeCode::Default());
    CHECK(spread.valid);

    UsdStageRefPtr narrow = MakeMultiMeshRig(2);
    RigExecRigEvaluator serial(narrow, SdfPath("/Asset/Rig"));
    if (!CompileOrReport(&serial, "two independent meshes")) {
        return;
    }
    CHECK(!serial.GetOpGraph().empty());
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
    // same invariant is TestHeldExecutionKeepsCompiledGraph below; this is
    // the half that catches the ones an invariant cannot name.
    const size_t graphNodes = evaluator.GetOpGraph().size();
    for (int run = 0; run < 60; ++run) {
        const RigExecRigPose again = evaluator.Evaluate(UsdTimeCode::Default());
        CHECK(again.valid);
        CHECK(again.diagnostics == first.diagnostics);
        CHECK(again.movedProperties.size() == first.movedProperties.size());
        CHECK(again.executedOpCount ==
              first.executedOpCount);
        CHECK(evaluator.GetOpGraph().size() == graphNodes);
        for (size_t mesh = 0; mesh < 6; ++mesh) {
            if (!Identical(MovedPoints(again, mesh), MovedPoints(first, mesh),
                           "repeated walk")) {
                return;  // one report is enough
            }
        }
    }
}

// Held runs use the same compiled graph and retain valid published outputs.
void
TestHeldExecutionKeepsCompiledGraph()
{
    UsdStageRefPtr stage = MakeMultiMeshRig(6);
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    if (!CompileOrReport(&evaluator, "six independent meshes")) {
        return;
    }
    CHECK(!evaluator.GetOpGraph().empty());
    // The full graph is compiled before any Evaluate.
    const size_t nodes = evaluator.GetOpGraph().size();
    CHECK(nodes >= 6);
    for (int run = 0; run < 8; ++run) {
        const auto pose=evaluator.Evaluate(UsdTimeCode::Default());
        CHECK(pose.valid);
        if(run>0)CHECK(pose.executedOpCount==0);
        CHECK(evaluator.GetOpGraph().size() == nodes);
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
    RigExecRigPose many;
    if (!CompileOrReport(&evaluator, "six independent meshes",&many)) {
        return;
    }
    CHECK(many.valid);
    CHECK(many.executedOpCount>0);

    const unsigned limit = WorkGetConcurrencyLimit();
    WorkSetConcurrencyLimit(1);
    RigExecRigEvaluator single(stage,SdfPath("/Asset/Rig"));
    CHECK(single.Compile());
    const RigExecRigPose one = single.Evaluate(UsdTimeCode::Default());
    WorkSetConcurrencyLimit(limit);

    CHECK(one.valid);
    CHECK(one.executedOpCount>0);
    CHECK(one.diagnostics == many.diagnostics);
    for (size_t mesh = 0; mesh < 6; ++mesh) {
        Identical(MovedPoints(one, mesh), MovedPoints(many, mesh),
                  "one thread vs many");
    }
}

// Independent weight fields: each chain publishes its own resolved
// field, and the fields are merged in chain order like everything else.
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
    CHECK(!evaluator.GetOpGraph().empty());

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

// A phased read depends on its selected producer and leaves unrelated
// branches independent in the actual common graph.
void
TestPhasedReadHasOnlyCausalDependencies()
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
    const auto graph=evaluator.GetOpGraph();
    const size_t source=rigExecTest::FindOperation(graph,TargetPath(0),"ChainStatus");
    const size_t consumer=rigExecTest::FindOperation(graph,SdfPath("/Asset/Rig/Movers/Lattice_5"),"RevisionStatic");
    CHECK(source<graph.size() && consumer<graph.size());
    CHECK(rigExecTest::HasDependencyPath(graph,source,consumer));
    for(size_t mesh=1;mesh<5;++mesh) {
        const size_t unrelated=rigExecTest::FindOperation(graph,TargetPath(mesh),"ChainStatus");
        CHECK(unrelated<graph.size());
        CHECK(!rigExecTest::HasDependencyPath(graph,unrelated,consumer));
        CHECK(!rigExecTest::HasDependencyPath(graph,consumer,unrelated));
    }
    const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(pose.valid);
    CHECK(MovedPoints(pose, 0).size() == kPointCount);
}

}  // namespace

// argv[1], optional: run every case this many times.
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
        TestIndependentChainsHaveNoCausalEdges();
        TestSerialAndParallelWalksAgreeExactly();
        TestRepeatedWalksAreIdentical();
        TestHeldExecutionKeepsCompiledGraph();
        TestOneThreadAndManyAgree();
        TestWeightObjectsInAParallelLevel();
        TestPhasedReadHasOnlyCausalDependencies();
    }

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecChainLevels: all tests passed\n");
    return 0;
}
