// Geometry operators and read phases evaluated without weight objects.
// Session-layer edits isolate blend shapes, guide points, ribbons and
// geometry read phases while preserving the shipped example stages.
// The claim, per case, is the one testRigExecExampleParity makes: every
// frame agrees with the dynamic path exactly, and every generation CAME FROM
// THE PROGRAM -- a rig that quietly declined would otherwise compare the
// dynamic path with itself and pass having proved nothing.
// argv[1] = the examples directory, argv[2] = the ribbon probe layer.
#include "rigExec/rigEvaluator.h"
#include "rigExec/types.h"
#include "rigExec/movers/moverRegistry.h"

#include "pxr/base/plug/registry.h"
#include "pxr/base/vt/value.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/relationship.h"
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

#define CHECK_ORACLE(x) do { if (!(x)) { std::printf("FAIL %d: %s\n", __LINE__, #x); return 1; } } while (false)

int TestBoundFrameOracleInputs()
{
    RigExecLoadComputations();
    const auto *handler = RigExecFindMoverHandler(TfToken("RigExecCurveMover"));
    CHECK_ORACLE(handler && handler->oracle);
    const SdfPath mover("/Rig/Guides"), owner("/Rig/Spline"), target("/Geom.points");
    const UsdTimeCode time(3);
    RigExecOracleScene scene;
    RigExecOraclePrim prim;
    prim.exists=true;prim.path=mover;prim.type=TfToken("RigExecCurveMover");
    RigExecOracleRelationship driver;
    driver.exists=true;driver.targets={owner};
    prim.relationships[TfToken("rigExec:driverFrames")]=driver;
    RigExecOracleAttribute mode;
    mode.exists=true;mode.atTime=VtValue(TfToken("emitGuidePoints"));
    prim.attributes[TfToken("rigExec:mode")]=mode;
    RigExecOracleFrameInput input;
    input.owner=owner;input.time=time;input.generation=7;input.available=true;
    input.value.frames.resize(2);
    input.value.frames[0].points={{GfVec3d(2,3,4),GfVec3d(4,3,4),
                                  GfVec3d(2,4,4),GfVec3d(2,3,5)}};
    input.value.frames[1].points={{GfVec3d(10,8,0),GfVec3d(11,8,0),
                                  GfVec3d(10,10,0),GfVec3d(10,8,1)}};
    input.value.rests={{{GfVec3d(0,0,0),GfVec3d(1,0,0),GfVec3d(0,1,0),GfVec3d(0,0,1)}},
                       {{GfVec3d(10,0,0),GfVec3d(11,0,0),GfVec3d(10,1,0),GfVec3d(10,0,1)}}};
    const std::unordered_map<SdfPath,GfMatrix4d,SdfPath::Hash> matrices;
    const VtVec3fArray base{GfVec3f(1,2,3),GfVec3f(4,5,6)};
    VtVec3fArray points=base;
    std::vector<std::string> diagnostics;
    size_t reads=0;
    bool exactRead=true;
    bool provide=true;
    const RigExecMoverOracleContext context{
        scene,prim,mover,target,time,scene,{},{},matrices,matrices,
        &diagnostics,&points,base,{},nullptr,nullptr,
        [&](const SdfPath &reader,const SdfPath &source,UsdTimeCode requested) {
            ++reads;exactRead=exactRead && reader==mover && source==owner && requested==time;
            return provide ? &input : nullptr;
        }};
    CHECK_ORACLE(handler->oracle(context)==RigExecOracleResult::Blend);
    CHECK_ORACLE(points==VtVec3fArray({GfVec3f(2,3,4),GfVec3f(10,8,0)}));
    CHECK_ORACLE(reads==1 && exactRead && diagnostics.empty());
    // Upstream input mutation changes the reference, with no expected output
    // or production geometry result supplied to the handler.
    input.value.frames[0].points[0][0]=6;
    CHECK_ORACLE(handler->oracle(context)==RigExecOracleResult::Blend);
    CHECK_ORACLE(points[0]==GfVec3f(6,3,4));
    input.value.frames[0].points[0][0]=2;
    points=base;input.available=false;
    CHECK_ORACLE(handler->oracle(context)==RigExecOracleResult::PassThrough && points==base);
    CHECK_ORACLE(diagnostics.back()=="MoverFailed /Rig/Guides: driver frame input unavailable");
    input.available=true;input.time=UsdTimeCode::Default();
    CHECK_ORACLE(handler->oracle(context)==RigExecOracleResult::PassThrough && points==base);
    input.time=time;input.generation=0;
    CHECK_ORACLE(handler->oracle(context)==RigExecOracleResult::PassThrough && points==base);
    input.generation=7;provide=false;
    CHECK_ORACLE(handler->oracle(context)==RigExecOracleResult::PassThrough && points==base);
    provide=true;input.owner=SdfPath("/Rig/Other");
    CHECK_ORACLE(handler->oracle(context)==RigExecOracleResult::PassThrough && points==base);
    input.owner=owner;input.value.rests.pop_back();
    CHECK_ORACLE(handler->oracle(context)==RigExecOracleResult::PassThrough && points==base);
    CHECK_ORACLE(diagnostics.back()=="MoverFailed /Rig/Guides: driver frame cardinality mismatch");
    input.value.rests.push_back({{GfVec3d(10,0,0),GfVec3d(11,0,0),GfVec3d(10,1,0),GfVec3d(10,0,1)}});
    points=VtVec3fArray{GfVec3f(1,2,3)};
    CHECK_ORACLE(handler->oracle(context)==RigExecOracleResult::PassThrough);
    CHECK_ORACLE(points==VtVec3fArray({GfVec3f(1,2,3)}));
    CHECK_ORACLE(diagnostics.back()=="MoverFailed /Rig/Guides: guide cardinality mismatch");
    prim.attributes[TfToken("rigExec:mode")].atTime=VtValue(TfToken("ribbon"));
    RigExecOracleRelationship binds;
    binds.exists=true;binds.targets={SdfPath("/Bind.st")};
    prim.relationships[TfToken("rigExec:bindCoordinates")]=binds;
    RigExecOraclePrim bind;
    bind.exists=true;bind.path=SdfPath("/Bind");
    RigExecOracleAttribute coordinates;
    coordinates.exists=true;coordinates.atTime=VtValue(VtVec2fArray{GfVec2f(.25f,0)});
    bind.attributes[TfToken("st")]=coordinates;scene.prims[bind.path]=bind;
    diagnostics.clear();
    CHECK_ORACLE(handler->oracle(context)==RigExecOracleResult::Blend);
    CHECK_ORACLE(points==VtVec3fArray({GfVec3f(3.25f,6.75f,6)}));
    CHECK_ORACLE(diagnostics.empty());
    scene.prims[bind.path].attributes[TfToken("st")].atTime=VtValue(VtVec2fArray());
    const auto previous=points;
    CHECK_ORACLE(handler->oracle(context)==RigExecOracleResult::PassThrough && points==previous);
    CHECK_ORACLE(diagnostics.back()=="MoverFailed /Rig/Guides: ribbon bind cardinality mismatch");
    return 0;
}

int TestBlendActivationOracleInput()
{
    const auto *handler=RigExecFindMoverHandler(TfToken("RigExecBlendShapeMover"));
    CHECK_ORACLE(handler && handler->oracle);
    const SdfPath mover("/Rig/Blend"),channel("/Rig/Channel"),sample("/Rig/Sample"),shape("/Shape");
    const UsdTimeCode time(3);
    RigExecOracleScene scene;scene.resolveFromFacts=true;
    const auto attribute=[](const SdfPath &path,const VtValue &value,const SdfValueTypeName &type) {
        RigExecOracleAttribute a;a.exists=true;a.path=path;a.type=type;
        a.atDefault=a.atTime=value;return a;
    };
    const auto relationship=[](const SdfPath &path) {
        RigExecOracleRelationship r;r.exists=true;r.targets={path};return r;
    };
    RigExecOraclePrim prim;prim.exists=true;prim.path=mover;
    prim.relationships[TfToken("rigExec:blendInputs")]=relationship(channel);
    RigExecOraclePrim input;input.exists=true;input.path=channel;
    input.attributes[TfToken("inputs:weight")]=attribute(channel.AppendProperty(TfToken("inputs:weight")),VtValue(.25f),SdfValueTypeNames->Float);
    input.relationships[TfToken("rigExec:samples")]=relationship(sample);
    scene.prims[channel]=input;
    RigExecOraclePrim samplePrim;samplePrim.exists=true;samplePrim.path=sample;
    const SdfPath activation=sample.AppendProperty(TfToken("rigExec:activation"));
    samplePrim.attributes[TfToken("rigExec:activation")]=attribute(activation,VtValue(.5f),SdfValueTypeNames->Float);
    RigExecOraclePrim shapePrim;shapePrim.exists=true;shapePrim.path=shape;
    const VtValue shapePoints(VtVec3fArray{GfVec3f(4,0,0)});
    shapePrim.attributes[TfToken("points")]=attribute(shape.AppendProperty(TfToken("points")),shapePoints,SdfValueTypeNames->Point3fArray);
    shapePrim.attributes[TfToken("offsets")]=attribute(shape.AppendProperty(TfToken("offsets")),shapePoints,SdfValueTypeNames->Vector3fArray);
    scene.prims[shape]=shapePrim;
    const std::unordered_map<SdfPath,GfMatrix4d,SdfPath::Hash> matrices;
    const VtVec3fArray base{GfVec3f(0)};
    VtVec3fArray points=base;
    std::vector<std::string> diagnostics;
    const RigExecMoverOracleContext context{
        scene,prim,mover,SdfPath("/Geom.points"),time,scene,{},{},matrices,matrices,
        &diagnostics,&points,base,
        [](const SdfPath &,const SdfPath &)->const VtValue* {return nullptr;}};
    for(bool sparse:{false,true}) {
        samplePrim.relationships.clear();
        samplePrim.relationships[TfToken(sparse?"rigExec:blendShape":"rigExec:targetPoints")]=
            relationship(sparse?shape:shape.AppendProperty(TfToken("points")));
        scene.prims[sample]=samplePrim;
        if(sparse)scene.blendShapes.insert(shape);
        points=base;diagnostics.clear();scene.overlay.clear();
        CHECK_ORACLE(handler->oracle(context)==RigExecOracleResult::Blend);
        // A quarter channel at the authored half activation is half of 4.
        CHECK_ORACLE(points==VtVec3fArray({GfVec3f(2,0,0)}));
        points=base;scene.overlay[activation]=VtValue(.25f);
        CHECK_ORACLE(handler->oracle(context)==RigExecOracleResult::Blend);
        CHECK_ORACLE(points==VtVec3fArray({GfVec3f(4,0,0)}));
        points=base;scene.overlay[activation]=VtValue(TfToken("wrong type"));
        CHECK_ORACLE(handler->oracle(context)==RigExecOracleResult::Blend);
        CHECK_ORACLE(points==VtVec3fArray({GfVec3f(2,0,0)}));
        points=base;scene.overlay[activation]=VtValue(0.0f);
        CHECK_ORACLE(handler->oracle(context)==RigExecOracleResult::Blend);
        CHECK_ORACLE(points==base);
        CHECK_ORACLE(diagnostics.back()=="MoverFailed /Rig/Blend: non-positive activation at /Rig/Sample");
    }
    return 0;
}

#undef CHECK_ORACLE

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

// Unbind every weight object in the rig, on the session layer, so the file
// on disk is untouched and the rig that remains is the one the shipped
// example authors minus an envelope this group does not own. An explicit
// empty target list is what blocks the weaker opinion; clearing the
// authoring would only expose it again.
void
UnbindWeightObjects(const UsdStageRefPtr &stage)
{
    stage->SetEditTarget(UsdEditTarget(stage->GetSessionLayer()));
    static const TfToken weightObject("rigExec:weightObject");
    size_t unbound = 0;
    for (const UsdPrim &prim : stage->Traverse()) {
        const UsdRelationship relationship =
            prim.GetRelationship(weightObject);
        SdfPathVector targets;
        if (relationship && relationship.GetTargets(&targets) &&
            !targets.empty()) {
            relationship.SetTargets({});
            ++unbound;
        }
    }
    // A rig whose weight objects moved out from under this helper would
    // still bake, and the case would silently stop being the one it says it
    // is -- so say how many were found rather than assume.
    CHECK(unbound > 0);
}

// One rig across its frames, both paths, compared. Returns what was
// published on \p watch at each frame, so the caller can go on to prove the
// operation moved something rather than passing on two identical copies of
// the rest pose.
std::vector<VtValue>
RunParity(const std::string &where, const UsdStageRefPtr &stage,
          const std::vector<UsdTimeCode> &frames, const SdfPath &watch,
          RigExecRigEvaluator **keep = nullptr)
{
    std::vector<VtValue> published;
    const SdfPath rigPath = FindRig(stage);
    CHECK(!rigPath.IsEmpty());
    if (rigPath.IsEmpty()) {
        return published;
    }
    // Leaked deliberately when the caller asks to keep it: a drag has to run
    // on the evaluator that already holds the program, and this is a test
    // binary that exits straight after.
    RigExecRigEvaluator *rig = new RigExecRigEvaluator(stage, rigPath);
    // The guides are a compared domain, and the example entries ask for
    // them; a suite that left them off would measure less than its siblings.
    rig->SetSolverGuidesEnabled(true);
    rig->cpuReference = true;
    std::vector<std::string> errors;
    if (!rig->Compile(&errors)) {
        ++failures;
        std::printf("FAIL %s: does not compile\n", where.c_str());
        for (const std::string &error : errors) {
            std::printf("    %s\n", error.c_str());
        }
        return published;
    }
    std::vector<std::string> reasons;
    if (!rig->IsBakeable(&reasons)) {
        ++failures;
        std::printf("FAIL %s: declined the bake\n", where.c_str());
        for (const std::string &reason : reasons) {
            std::printf("    %s\n", reason.c_str());
        }
        return published;
    }
    for (const UsdTimeCode frame : frames) {
        const RigExecRigPose pose = rig->Evaluate(frame);
        CHECK(pose.valid);
        if (pose.referenceMismatches) {
            ++failures;
            std::printf("FAIL %s: %zu baked parity mismatch(es) at %g\n",
                        where.c_str(), pose.referenceMismatches,
                        frame.GetValue());
            for (const std::string &diagnostic : pose.diagnostics) {
                std::printf("    %s\n", diagnostic.c_str());
            }
        }
        const auto moved = pose.movedProperties.find(watch);
        if (moved == pose.movedProperties.end()) {
            ++failures;
            std::printf("FAIL %s: nothing published on %s at %g\n",
                        where.c_str(), watch.GetText(), frame.GetValue());
        } else {
            published.push_back(moved->second);
        }
    }
    if (rig->GetBakedGenerationCount() != frames.size()) {
        ++failures;
        std::printf("FAIL %s: %zu of %zu generation(s) came from the "
                    "program\n", where.c_str(), rig->GetBakedGenerationCount(),
                    frames.size());
    }
    std::printf("  %-42s %2zu frame(s) from the program\n", where.c_str(),
                frames.size());
    if (keep) {
        *keep = rig;
    } else {
        delete rig;
    }
    return published;
}

// The operation has to have DONE something: an op that silently published
// its input would agree with a dynamic path that did the same, and every
// comparison above would pass.
void
CheckMoves(const std::string &where, const std::vector<VtValue> &published)
{
    if (published.size() < 2) {
        ++failures;
        std::printf("FAIL %s: %zu published value(s)\n", where.c_str(),
                    published.size());
        return;
    }
    for (size_t i = 1; i < published.size(); ++i) {
        if (published[i] != published.front()) {
            return;
        }
    }
    ++failures;
    std::printf("FAIL %s: every frame published the same value; the "
                "operation is a pass-through here\n", where.c_str());
}

// One drag, held across the frames, asserting the override PLACED -- an
// override the program cannot reach falls back to the dynamic path, answers
// identically, and shows up nowhere except in this count. Each frame twice,
// because SetInteractiveOverrides invalidates the static input cache on the
// way in and a cache that refilled mid-drag only gets its chance on a second
// pass.
void
DragOne(const std::string &where, RigExecRigEvaluator *rig,
        const std::vector<RigExecValueOverride> &overrides,
        const std::vector<UsdTimeCode> &frames)
{
    const size_t before = rig->GetBakedGenerationCount();
    rig->SetInteractiveOverrides(overrides);
    size_t generations = 0;
    for (const UsdTimeCode frame : frames) {
        for (int pass = 0; pass < 2; ++pass) {
            const RigExecRigPose pose = rig->Evaluate(frame);
            CHECK(pose.valid);
            if (pose.referenceMismatches) {
                ++failures;
                std::printf("FAIL %s: %zu baked parity mismatch(es) at %g\n",
                            where.c_str(), pose.referenceMismatches,
                            frame.GetValue());
            }
            ++generations;
        }
    }
    if (rig->GetBakedGenerationCount() - before != generations) {
        ++failures;
        std::printf("FAIL %s: %zu of %zu generation(s) came from the "
                    "program; the drag was not placed\n", where.c_str(),
                    rig->GetBakedGenerationCount() - before, generations);
    }
    rig->ClearInteractiveOverrides();
    std::printf("  %-42s %2zu generation(s) under a drag\n", where.c_str(),
                generations);
}

const std::vector<UsdTimeCode> &
ExampleFrames()
{
    // The frames tests/exampleFixtures.cmake gives 04, 12 and 13, so a case
    // here and the example entry that will replace it measure one thing.
    static const std::vector<UsdTimeCode> frames{
        UsdTimeCode(1001), UsdTimeCode(1012), UsdTimeCode(1024),
        UsdTimeCode(1036), UsdTimeCode(1048)};
    return frames;
}

// blendShape: two channels, one of them carrying an in-between sample, and
// the two interactive inputs that reach them -- a channel's weight and a
// sample's activation, which place through resolvedRoutedPrims rather than
// through a bound value.
void
TestBlendShape(const std::string &examplesDir)
{
    const auto stage =
        UsdStage::Open(examplesDir + "/04_BlendShapeFace.usda");
    CHECK(stage);
    if (!stage) {
        return;
    }
    UnbindWeightObjects(stage);
    const SdfPath card("/FaceAsset/Geom/FaceCard.points");
    RigExecRigEvaluator *rig = nullptr;
    CheckMoves("04 blendShape",
               RunParity("04 blendShape", stage, ExampleFrames(), card, &rig));
    if (!rig) {
        return;
    }
    DragOne("04 blendShape (channel weight)", rig,
            {RigExecValueOverride{SdfPath("/FaceAsset/Rig/BlendInputs/Smile"),
                                  TfToken(), TfToken("inputs:weight"),
                                  VtValue(0.75f)}},
            ExampleFrames());
    DragOne("04 blendShape (sample activation)", rig,
            {RigExecValueOverride{
                SdfPath("/FaceAsset/Rig/BlendInputs/Smile/Half"), TfToken(),
                TfToken("rigExec:activation"), VtValue(0.35f)}},
            ExampleFrames());
    delete rig;
}

// Read phases, all three forms a geometry input can name, on the lattice
// whose cage two movers rewrite. `base` reads the authored cage and takes no
// snapshot; `final` and the mid-walk prim both come out of the run-local
// snapshot store, which is the half no shipped bakeable rig reaches -- and
// the three answers must DIFFER, or the store is feeding one value under
// three names.
void
TestReadPhases(const std::string &examplesDir)
{
    static const TfToken readPhase("rigExecReadPhase");
    static const TfToken cage("rigExec:cage");
    const SdfPath lattice("/ReadPhaseAsset/Rig/Movers/Geometry/SlabLattice");
    const SdfPath slab("/ReadPhaseAsset/Geom/Slab.points");
    std::vector<std::vector<VtValue>> answers;
    for (const char *phase : {"base", "final",
                              "/ReadPhaseAsset/Rig/Movers/Cage/CageLift"}) {
        // A fresh stage per phase: the phase is compiled into the chain, and
        // re-authoring it under a live evaluator would test the rebuild
        // rather than the phase.
        const auto stage =
            UsdStage::Open(examplesDir + "/13_ReadPhases.usda");
        CHECK(stage);
        if (!stage) {
            return;
        }
        UnbindWeightObjects(stage);
        const UsdRelationship input =
            stage->GetPrimAtPath(lattice).GetRelationship(cage);
        CHECK(input && input.SetMetadata(readPhase,
                                         VtValue(std::string(phase))));
        const std::string where =
            std::string("13 read phase ") + phase;
        answers.push_back(
            RunParity(where, stage, ExampleFrames(), slab));
        // Not "base": the authored cage is what it reads, and the authored
        // cage does not animate -- a slab that stood still under it would be
        // this example working, not a pass-through. The other two read a
        // cage two animated movers rewrote, so they have to move.
        if (std::string(phase) != "base") {
            CheckMoves(where, answers.back());
        }
    }
    CHECK(answers.size() == 3);
    if (answers.size() != 3 || answers[0].empty()) {
        return;
    }
    // Pairwise, because "final" agreeing with "base" and the mid-walk point
    // agreeing with "final" are two different failures: the first says no
    // snapshot was taken, the second says the walk's record was taken at the
    // wrong point.
    CHECK(answers[0] != answers[1]);
    CHECK(answers[1] != answers[2]);
    CHECK(answers[0] != answers[2]);
}

// emitGuidePoints and ribbon, off a solver that bakes. The probe layer is
// the only thing that reaches them until RigExecRibbon bakes; see its own
// header for why it sublayers the animated biped.
void
TestCurveModes(const std::string &probeLayer)
{
    const auto stage = UsdStage::Open(probeLayer);
    CHECK(stage);
    if (!stage) {
        return;
    }
    const std::vector<UsdTimeCode> frames{
        UsdTimeCode(1), UsdTimeCode(2), UsdTimeCode(3), UsdTimeCode(4),
        UsdTimeCode(5), UsdTimeCode(6), UsdTimeCode(7), UsdTimeCode(8)};
    CheckMoves("probe emitGuidePoints",
               RunParity("probe emitGuidePoints", stage, frames,
                         SdfPath("/Biped/SpineGuides.points")));
    CheckMoves("probe ribbon",
               RunParity("probe ribbon", stage, frames,
                         SdfPath("/Biped/SpineStrip.points")));
}

}  // namespace

int
main(int argc, char **argv)
{
    if (argc < 3) {
        std::printf("usage: testRigExecGeometryOpsBakedParity <examplesDir> "
                    "<ribbonProbe.usda>\n");
        return 1;
    }
    PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);
    const std::string examplesDir = argv[1];
    TestBlendShape(examplesDir);
    TestReadPhases(examplesDir);
    CHECK(TestBoundFrameOracleInputs() == 0);
    CHECK(TestBlendActivationOracleInput() == 0);
    TestCurveModes(argv[2]);
    std::printf("testRigExecGeometryOpsBakedParity: %d failure(s)\n", failures);
    return failures ? 1 : 0;
}
