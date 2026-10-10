// Scoped cache clears (unified-program spec rules S5, S6): a stage notice
// drops, of the value caches that outlive a generation, exactly what it
// could have made stale -- and what it drops, the next generation reads
// again.
// Each case edits one value the caches hold or were read from, evaluates,
// and holds the pose to two things: a second evaluator compiled fresh on the
// edited stage, which cannot have kept anything, and the cache's own
// occupancy or the property chain's executed part identities, which show
// whether an unrelated edit left a cache standing. The three edits are
// the ones the spec names as the hazards of a scoped clear:
//   * a default edit on an unconnected property-chain mover input, whose
//     value the chain binding captured as a constant;
//   * an inputs:method edit on a skin mover whose rigExec:skinningMethod is
//     connected to it -- a layout input reached through a connection, on the
//     mover prim and not on the mesh;
//   * a blend sample edit: the UsdSkelBlendShape's offsets, the sample's own
//     activation, and the points the layout was range-checked against.
// Registered plain (baked and dynamic in turn), under the parity entries,
// and under RIGEXEC_VERIFY_SCOPED_CLEARS=1, where every evaluator is shadowed
// by one that drops the caches whole and the two poses are compared.
#include "rigExec/inputReplay.h"
#include "rigExec/bakedProgram.h"
#include "rigExec/bakedProgramImpl.h"
#include "rigExec/profiler.h"
#include "rigExec/rigEvaluator.h"

#include "pxr/base/gf/vec3f.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/getenv.h"
#include "pxr/base/vt/array.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/sdf/types.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usd/stage.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <cstdio>
#include <string>
#include <set>
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

std::unique_ptr<RigExecRigEvaluator>
Compiled(const UsdStageRefPtr &stage)
{
    auto evaluator =
        std::make_unique<RigExecRigEvaluator>(stage, SdfPath("/Asset/Rig"));
    std::vector<std::string> errors;
    if (!evaluator->Compile(&errors)) {
        ++failures;
        for (const std::string &error : errors) {
            std::printf("FAIL compile: %s\n", error.c_str());
        }
    }
    return evaluator;
}

// Equal, in everything a pose publishes, to what an evaluator compiled fresh
// on the stage as it now stands publishes. The mover-graph counters and the
// line that reports them are left out: they count what a generation BUILT,
// and a fresh evaluator's first generation builds every node the edited one
// kept. So is the line a recompile adds (a sample's activation is epoch
// identity), which reports the edit rather than the pose.
void
CheckAgreesWithFresh(const std::string &what, const UsdStageRefPtr &stage,
                     RigExecRigPose pose)
{
    RigExecRigPose fresh =
        Compiled(stage)->Evaluate(UsdTimeCode::Default());
    CHECK(pose.valid);
    CHECK(fresh.valid);
    CHECK(pose.comparisonMismatches == 0);
    const auto keepPosed = [](std::vector<std::string> *lines) {
        std::vector<std::string> kept;
        for (std::string &line : *lines) {
            if (line.rfind("mover graph:", 0) != 0 &&
                line.rfind("structural edit:", 0) != 0) {
                kept.push_back(std::move(line));
            }
        }
        *lines = std::move(kept);
    };
    keepPosed(&fresh.diagnostics);
    keepPosed(&pose.diagnostics);
    RigExecRigPose diff;
    RigExecComparePoses(fresh, pose, &diff);
    if (diff.comparisonMismatches != 0) {
        std::printf("FAIL %s: %zu difference(s) from a fresh evaluator:\n",
                    what.c_str(), diff.comparisonMismatches);
        for (size_t i = 0; i < diff.diagnostics.size() && i < 12; ++i) {
            std::printf("    %s\n", diff.diagnostics[i].c_str());
        }
        for (const std::string &line : fresh.diagnostics) {
            std::printf("    fresh:  %s\n", line.c_str());
        }
        for (const std::string &line : pose.diagnostics) {
            std::printf("    edited: %s\n", line.c_str());
        }
    }
    CHECK(diff.comparisonMismatches == 0);
}

// Exact property parts the last generation executed for this target.
std::set<int>
ChainPartsRan(const RigExecRigEvaluator &evaluator, const SdfPath &target)
{
    std::set<int> parts;
    const auto *program = evaluator.GetBakedProgram();
    CHECK(program);
    if (!program) return parts;
    const auto &B = program->GetStepGraph();
    for (const auto &entry : evaluator.GetLastOpTrace()) {
        if (entry.kind != "PropertyRevision") continue;
        CHECK(entry.step < B.opGraph.ops.size());
        if (entry.step >= B.opGraph.ops.size()) continue;
        const size_t body = B.opGraph.ops[entry.step].originalIndex;
        CHECK(body < B.steps.size());
        if (body >= B.steps.size()) continue;
        const auto &step = B.steps[body];
        CHECK(step.kind == RigExecBakedStepKind::PropertyRevision);
        CHECK(step.object >= 0 && size_t(step.object) < B.propertyChains.size());
        if (step.object < 0 || size_t(step.object) >= B.propertyChains.size()) continue;
        const auto &chain = B.propertyChains[size_t(step.object)];
        if (chain.target != target) continue;
        CHECK(step.part >= 0 && size_t(step.part) <= chain.revisions.size());
        CHECK(parts.insert(step.part).second);
    }
    return parts;
}

float
FloatAt(const RigExecRigPose &pose, const SdfPath &path)
{
    const auto found = pose.movedProperties.find(path);
    if (found == pose.movedProperties.end() ||
        !found->second.IsHolding<float>()) {
        return std::nanf("");
    }
    return found->second.UncheckedGet<float>();
}

// Property chains.

// dial = 2, multiplied by TimesTen's inputs:value, then AddOne's
// inputs:value added -- which is connected to Channels.gain -- published at
// /Asset/Rig/Channels.rigExec:dial. A mover beside them moves a point, and
// Shape.foo is a value nothing reads.
const char *kChainFixture = R"USDA(#usda 1.0

def Scope "Asset"
{
    def RigExecRoot "Rig"
    {
        def Scope "Channels"
        {
            float rigExec:dial = 2
            float gain = 1
        }

        def RigExecControl "Driver"
        {
            double avars:tx = 1
        }

        def Scope "Movers"
        {
            reorder nameChildren = ["AddOne", "TimesTen"]

            def RigExecFloatMathMover "TimesTen" (
                prepend apiSchemas = ["RigExecMoverAPI"]
            )
            {
                uniform token rigExec:operation = "multiply"
                float inputs:value = 10
                rel rigExec:moves = </Asset/Rig/Channels.rigExec:dial>
            }

            def RigExecFloatMathMover "AddOne" (
                prepend apiSchemas = ["RigExecMoverAPI"]
            )
            {
                uniform token rigExec:operation = "add"
                float inputs:value.connect = </Asset/Rig/Channels.gain>
                rel rigExec:moves = </Asset/Rig/Channels.rigExec:dial>
            }

            def RigExecMatrixMover "MoveShape" (
                prepend apiSchemas = ["RigExecMoverAPI"]
            )
            {
                float inputs:defaultWeight = 1
                rel rigExec:moves = </Asset/Shape.points>
                rel rigExec:transform = </Asset/Rig/Driver>
            }
        }
    }

    def Points "Shape"
    {
        point3f[] points = [(0, 0, 0)]
        float foo = 0
    }
}
)USDA";

UsdStageRefPtr
Open(const char *text)
{
    const SdfLayerRefPtr layer = SdfLayer::CreateAnonymous(".usda");
    if (!layer || !rigExec::RigExecInputReplayImportFromString(layer, text)) {
        ++failures;
        std::printf("FAIL: could not build a fixture\n");
        return UsdStageRefPtr();
    }
    return UsdStage::Open(layer);
}

void
TestAChainInputDefaultEditReachesTheChain()
{
    const UsdStageRefPtr stage = Open(kChainFixture);
    if (!stage) {
        return;
    }
    const SdfPath dial("/Asset/Rig/Channels.rigExec:dial");
    auto evaluator = Compiled(stage);
    evaluator->SetProfilingEnabled(true);
    RigExecRigPose pose = evaluator->Evaluate(UsdTimeCode::Default());
    CHECK(FloatAt(pose, dial) == 21.0f);
    const auto *program = evaluator->GetBakedProgram();
    CHECK(program);
    if (!program) return;
    const auto &chains = program->GetStepGraph().propertyChains;
    const auto chain = std::find_if(chains.begin(), chains.end(),
        [&](const auto &entry) { return entry.target == dial; });
    CHECK(chain != chains.end());
    if (chain == chains.end()) return;
    CHECK(chain->revisions.size() == 2);
    if (chain->revisions.size() != 2) return;
    CHECK(chain->revisions[0].mover == SdfPath("/Asset/Rig/Movers/TimesTen"));
    CHECK(chain->revisions[1].mover == SdfPath("/Asset/Rig/Movers/AddOne"));

    // A value nothing the chain reads: the chain keeps its answer and does
    // not run.
    stage->GetPrimAtPath(SdfPath("/Asset/Shape"))
        .GetAttribute(TfToken("foo")).Set(1.0f);
    evaluator->ClearProfile();
    pose = evaluator->Evaluate(UsdTimeCode::Default());
    CHECK(FloatAt(pose, dial) == 21.0f);
    CHECK(ChainPartsRan(*evaluator, dial).empty());
    CheckAgreesWithFresh(std::string("unread value, ") + "graph",
                         stage, pose);

    // A default edit on an unconnected input, whose value the binding held
    // as a constant: the chain is rebound and runs.
    stage->GetPrimAtPath(SdfPath("/Asset/Rig/Movers/TimesTen"))
        .GetAttribute(TfToken("inputs:value")).Set(20.0f);
    evaluator->ClearProfile();
    pose = evaluator->Evaluate(UsdTimeCode::Default());
    CHECK(FloatAt(pose, dial) == 41.0f);
    // The unchanged base is retained; TimesTen changes its output, so AddOne follows.
    CHECK((ChainPartsRan(*evaluator, dial) == std::set<int>{1, 2}));
    CheckAgreesWithFresh(std::string("chain input default, ") +
                             "graph",
                         stage, pose);

    // A default edit one hop upstream of a connected input.
    stage->GetPrimAtPath(SdfPath("/Asset/Rig/Channels"))
        .GetAttribute(TfToken("gain")).Set(5.0f);
    evaluator->ClearProfile();
    pose = evaluator->Evaluate(UsdTimeCode::Default());
    CHECK(FloatAt(pose, dial) == 45.0f);
    // Only AddOne reads gain. TimesTen and the authored base retain their versions.
    CHECK((ChainPartsRan(*evaluator, dial) == std::set<int>{2}));
    CheckAgreesWithFresh(std::string("connection source default, ") +
                             "graph",
                         stage, pose);

    // And the frame after: nothing moved, so nothing runs.
    evaluator->ClearProfile();
    pose = evaluator->Evaluate(UsdTimeCode::Default());
    CHECK(FloatAt(pose, dial) == 45.0f);
    CHECK(ChainPartsRan(*evaluator, dial).empty());
}

// Skin layouts.

const SdfPath kSkinTarget("/Asset/Geom/Mesh.points");
const SdfPath kSkin("/Asset/Rig/Movers/Skin");
constexpr size_t kSkinPoints = 4;

// One mesh, two controls -- one of them rotating, so the two skinning
// methods disagree -- and a skin mover with two influence slots per point,
// whose rigExec:skinningMethod is connected to inputs:method on itself.
UsdStageRefPtr
MakeSkinnedRig()
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
    alongY.GetAttribute(TfToken("avars:rz")).Set(90.0);

    const UsdPrim mesh =
        stage->DefinePrim(kSkinTarget.GetPrimPath(), TfToken("Mesh"));
    VtVec3fArray points(kSkinPoints);
    for (size_t i = 0; i < kSkinPoints; ++i) {
        points[i] = GfVec3f(float(i), float(i) * 2.0f, float(i) * 3.0f);
    }
    mesh.GetAttribute(TfToken("points")).Set(points);
    mesh.CreateAttribute(TfToken("foo"), SdfValueTypeNames->Float).Set(0.0f);

    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const UsdPrim skin = stage->DefinePrim(kSkin, TfToken("RigExecSkinMover"));
    skin.ApplyAPI(TfToken("RigExecMoverAPI"));
    skin.GetRelationship(TfToken("rigExec:moves")).SetTargets({kSkinTarget});
    skin.GetAttribute(TfToken("inputs:defaultWeight")).Set(1.0f);
    skin.CreateRelationship(TfToken("rigExec:influences"))
        .SetTargets({alongX.GetPath(), alongY.GetPath()});
    skin.CreateAttribute(TfToken("rigExec:elementSize"),
                         SdfValueTypeNames->Int).Set(2);
    VtIntArray indices(kSkinPoints * 2);
    VtFloatArray weights(kSkinPoints * 2);
    for (size_t i = 0; i < kSkinPoints; ++i) {
        indices[i * 2] = 0;
        indices[i * 2 + 1] = 1;
        weights[i * 2] = 0.5f;
        weights[i * 2 + 1] = 0.5f;
    }
    skin.CreateAttribute(TfToken("rigExec:jointIndices"),
                         SdfValueTypeNames->IntArray).Set(indices);
    skin.CreateAttribute(TfToken("rigExec:jointWeights"),
                         SdfValueTypeNames->FloatArray).Set(weights);
    const UsdAttribute source = skin.CreateAttribute(
        TfToken("inputs:method"), SdfValueTypeNames->Token);
    source.Set(TfToken("classicLinear"));
    const UsdAttribute declared = skin.CreateAttribute(
        TfToken("rigExec:skinningMethod"), SdfValueTypeNames->Token);
    declared.Set(TfToken("classicLinear"));
    declared.AddConnection(source.GetPath());
    return stage;
}

VtVec3fArray
Deformed(const RigExecRigPose &pose, const SdfPath &target)
{
    const auto found = pose.movedProperties.find(target);
    return found == pose.movedProperties.end()
               ? VtVec3fArray()
               : found->second.Get<VtVec3fArray>();
}

void
TestAnInputsMethodEditOnASkinMover()
{
    const UsdStageRefPtr stage = MakeSkinnedRig();
    auto evaluator = Compiled(stage);
    RigExecRigPose pose = evaluator->Evaluate(UsdTimeCode::Default());
    const VtVec3fArray linear = Deformed(pose, kSkinTarget);
    CHECK(linear.size() == kSkinPoints);
    CHECK(evaluator->GetBakedProgram() != nullptr);
    CHECK(evaluator->GetSkinTopologyCacheSize() > 0);

    // The mesh the layout deforms, and the mover's own envelope: neither is
    // a layout input, so the layout stays cached -- the case "the notice
    // names a mesh" would have dropped it for.
    stage->GetPrimAtPath(kSkinTarget.GetPrimPath())
        .GetAttribute(TfToken("foo")).Set(1.0f);
    CHECK(evaluator->GetSkinTopologyCacheSize() > 0);
    stage->GetPrimAtPath(kSkin)
        .GetAttribute(TfToken("inputs:defaultWeight")).Set(1.0f);
    CHECK(evaluator->GetSkinTopologyCacheSize() > 0);
    pose = evaluator->Evaluate(UsdTimeCode::Default());
    CHECK(Deformed(pose, kSkinTarget) == linear);
    CheckAgreesWithFresh(std::string("skin, unrelated edits, ") +
                             "graph",
                         stage, pose);

    // The method, one connection hop upstream of rigExec:skinningMethod: a
    // layout input reached through a connection, so the layouts go.
    stage->GetPrimAtPath(kSkin)
        .GetAttribute(TfToken("inputs:method"))
        .Set(TfToken("dualQuaternion"));
    CHECK(evaluator->GetSkinTopologyCacheSize() == 0);
    pose = evaluator->Evaluate(UsdTimeCode::Default());
    const VtVec3fArray dual = Deformed(pose, kSkinTarget);
    CHECK(dual.size() == kSkinPoints);
    CHECK(dual != linear);
    CHECK(evaluator->GetSkinTopologyCacheSize() > 0);
    CheckAgreesWithFresh(std::string("skin, inputs:method, ") +
                             "graph",
                         stage, pose);

    // A weight paint is a layout input itself.
    VtFloatArray weights(kSkinPoints * 2);
    for (size_t i = 0; i < kSkinPoints; ++i) {
        weights[i * 2] = 1.0f;
        weights[i * 2 + 1] = 0.0f;
    }
    stage->GetPrimAtPath(kSkin)
        .GetAttribute(TfToken("rigExec:jointWeights")).Set(weights);
    CHECK(evaluator->GetSkinTopologyCacheSize() == 0);
    pose = evaluator->Evaluate(UsdTimeCode::Default());
    CHECK(Deformed(pose, kSkinTarget) != dual);
    CheckAgreesWithFresh(std::string("skin, weight paint, ") +
                             "graph",
                         stage, pose);
}

// Blend sample shapes.

const SdfPath kBlendTarget("/Asset/Geom/Face.points");

// A four-point mesh and a blend shape mover with two channels, each with one
// SPARSE sample -- a rigExec:blendShape naming a UsdSkelBlendShape -- so both
// shapes are cached by sample prim.
UsdStageRefPtr
MakeBlendRig()
{
    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    stage->DefinePrim(SdfPath("/Asset"), TfToken("Scope"));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    const UsdPrim mesh =
        stage->DefinePrim(kBlendTarget.GetPrimPath(), TfToken("Mesh"));
    mesh.GetAttribute(TfToken("points"))
        .Set(VtVec3fArray{{0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0}});
    mesh.GetAttribute(TfToken("faceVertexCounts")).Set(VtIntArray{4});
    mesh.GetAttribute(TfToken("faceVertexIndices"))
        .Set(VtIntArray{0, 1, 2, 3});
    const auto shape = [&stage](const char *name, const GfVec3f &offset,
                                int index) {
        const UsdPrim prim = stage->DefinePrim(
            SdfPath(std::string("/Asset/Shapes/") + name),
            TfToken("BlendShape"));
        prim.CreateAttribute(TfToken("offsets"),
                             SdfValueTypeNames->Vector3fArray)
            .Set(VtVec3fArray{offset});
        prim.CreateAttribute(TfToken("pointIndices"),
                             SdfValueTypeNames->IntArray)
            .Set(VtIntArray{index});
        return prim;
    };
    const UsdPrim up = shape("Up", GfVec3f(0, 0, 1), 2);
    const UsdPrim side = shape("Side", GfVec3f(1, 0, 0), 0);
    SdfPathVector channels;
    for (const auto &[name, blendShape] :
         {std::make_pair("Raise", up), std::make_pair("Push", side)}) {
        const SdfPath channelPath =
            SdfPath("/Asset/Rig/Channels").AppendChild(TfToken(name));
        const UsdPrim channel =
            stage->DefinePrim(channelPath, TfToken("RigExecBlendInput"));
        const UsdPrim sample = stage->DefinePrim(
            channelPath.AppendChild(TfToken("Full")),
            TfToken("RigExecBlendSample"));
        sample.CreateAttribute(TfToken("rigExec:activation"),
                               SdfValueTypeNames->Float).Set(1.0f);
        sample.CreateRelationship(TfToken("rigExec:blendShape"))
            .SetTargets({blendShape.GetPath()});
        channel.GetRelationship(TfToken("rigExec:samples"))
            .SetTargets({sample.GetPath()});
        channel.GetAttribute(TfToken("inputs:weight")).Set(1.0f);
        channels.push_back(channelPath);
    }
    const UsdPrim blend = stage->DefinePrim(
        SdfPath("/Asset/Rig/Movers/Blend"), TfToken("RigExecBlendShapeMover"));
    blend.ApplyAPI(TfToken("RigExecMoverAPI"));
    blend.GetRelationship(TfToken("rigExec:moves")).SetTargets({kBlendTarget});
    blend.GetRelationship(TfToken("rigExec:blendInputs")).SetTargets(channels);
    return stage;
}

void
TestABlendSampleEdit()
{
    const UsdStageRefPtr stage = MakeBlendRig();
    auto evaluator = Compiled(stage);
    RigExecRigPose pose = evaluator->Evaluate(UsdTimeCode::Default());
    VtVec3fArray points = Deformed(pose, kBlendTarget);
    CHECK(points.size() == 4);
    if (points.size() == 4) {
        CHECK(points[2] == GfVec3f(1, 1, 1));
        CHECK(points[0] == GfVec3f(1, 0, 0));
    }
    CHECK(evaluator->GetBlendSampleCacheSize() == 2);

    // One shape's offsets: that sample's cached shape goes, the other's
    // stays. Offsets are topology, so the program rebuilds.
    const size_t builds = evaluator->GetBakedProgramBuildCount();
    stage->GetPrimAtPath(SdfPath("/Asset/Shapes/Up"))
        .GetAttribute(TfToken("offsets"))
        .Set(VtVec3fArray{GfVec3f(0, 0, 2)});
    CHECK(evaluator->GetLastNoticeDisposition() ==
          RigExecNoticeDisposition::Stale);
    CHECK(evaluator->GetBlendSampleCacheSize() == 1);
    pose = evaluator->Evaluate(UsdTimeCode::Default());
    CHECK(evaluator->GetBakedProgramBuildCount() == builds + 1);
    points = Deformed(pose, kBlendTarget);
    CHECK(points.size() == 4 && points[2] == GfVec3f(1, 1, 2));
    CHECK(evaluator->GetBlendSampleCacheSize() == 2);
    CheckAgreesWithFresh(std::string("blend shape offsets, ") +
                             "graph",
                         stage, pose);

    // The other sample's own activation: a value on the sample prim.
    stage->GetPrimAtPath(SdfPath("/Asset/Rig/Channels/Push/Full"))
        .GetAttribute(TfToken("rigExec:activation")).Set(0.5f);
    CHECK(evaluator->GetBlendSampleCacheSize() == 1);
    pose = evaluator->Evaluate(UsdTimeCode::Default());
    CheckAgreesWithFresh(std::string("blend sample activation, ") +
                             "graph",
                         stage, pose);

    // The points both layouts were range-checked against.
    stage->GetPrimAtPath(kBlendTarget.GetPrimPath())
        .GetAttribute(TfToken("points"))
        .Set(VtVec3fArray{{0, 0, 0}, {2, 0, 0}, {2, 2, 0}, {0, 2, 0}});
    CHECK(evaluator->GetBlendSampleCacheSize() == 0);
    pose = evaluator->Evaluate(UsdTimeCode::Default());
    points = Deformed(pose, kBlendTarget);
    CHECK(points.size() == 4 && points[2] == GfVec3f(2, 2, 2));
    CheckAgreesWithFresh(std::string("blend target points, ") +
                             "graph",
                         stage, pose);
}

// The sparse samples of the blend mover, in channel order.
std::vector<const RigExecBakedProgramImpl::GeomBlendChannel::Sample *>
SparseSamples(const RigExecRigEvaluator &evaluator)
{
    std::vector<const RigExecBakedProgramImpl::GeomBlendChannel::Sample *> out;
    const RigExecBakedProgram *program = evaluator.GetBakedProgram();
    if (!program) {
        return out;
    }
    for (const auto &chain : program->GetStepGraph().chains) {
        for (const auto &revision : chain.revisions) {
            for (const auto &channel : revision.blendChannels) {
                for (const auto &sample : channel.samples) {
                    if (!sample.blendShape.IsEmpty()) {
                        out.push_back(&sample);
                    }
                }
            }
        }
    }
    return out;
}

// A sparse layout is epoch data: a channel weight keyed over three frames
// re-runs the blend revision at every frame, and no frame builds a sample's
// layout again -- it stays the same object.
void
TestASparseLayoutIsBuiltOncePerEpoch()
{
    const UsdStageRefPtr stage = MakeBlendRig();
    const UsdAttribute weight =
        stage->GetPrimAtPath(SdfPath("/Asset/Rig/Channels/Raise"))
            .GetAttribute(TfToken("inputs:weight"));
    CHECK(weight.Set(0.0f, UsdTimeCode(1.0)));
    CHECK(weight.Set(1.0f, UsdTimeCode(3.0)));
    auto evaluator = Compiled(stage);
    RigExecRigPose pose = evaluator->Evaluate(UsdTimeCode(1.0));
    CHECK(pose.valid);
    VtVec3fArray previous = Deformed(pose, kBlendTarget);
    std::vector<uint64_t> built;
    std::vector<std::shared_ptr<const RigExecBlendSampleLayout>> held;
    for (const auto *sample : SparseSamples(*evaluator)) {
        CHECK(sample->layout != nullptr && sample->layoutBuilds > 0);
        built.push_back(sample->layoutBuilds);
        held.push_back(sample->layout);
    }
    CHECK(built.size() == 2);
    const size_t builds = evaluator->GetBakedProgramBuildCount();
    for (const double frame : {2.0, 3.0}) {
        pose = evaluator->Evaluate(UsdTimeCode(frame));
        const VtVec3fArray points = Deformed(pose, kBlendTarget);
        // The weight moved the points, so the revision ran.
        CHECK(points.size() == 4 && points != previous);
        previous = points;
        const auto samples = SparseSamples(*evaluator);
        CHECK(samples.size() == built.size());
        for (size_t i = 0; i < samples.size() && i < built.size(); ++i) {
            CHECK(samples[i]->layoutBuilds == built[i]);
            CHECK(samples[i]->layout == held[i]);
        }
    }
    CHECK(previous.size() == 4 && previous[2] == GfVec3f(1, 1, 1));
    CHECK(evaluator->GetBakedProgramBuildCount() == builds);
}

}  // namespace

int
main()
{
    PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);
    {
        TestAChainInputDefaultEditReachesTheChain();
        TestAnInputsMethodEditOnASkinMover();
        TestABlendSampleEdit();
        TestASparseLayoutIsBuiltOncePerEpoch();
    }
    if (failures) {
        std::printf("testRigExecScopedClears: %d failure(s)\n", failures);
        return 1;
    }
    std::printf("testRigExecScopedClears: all tests passed\n");
    return 0;
}
