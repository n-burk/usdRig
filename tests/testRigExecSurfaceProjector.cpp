// SurfaceProjector frames remain target-local under static asset placement.
// Exercise the shared graph without authoring during evaluation.
#include "rigExec/rigEvaluator.h"

#include "pxr/base/gf/rotation.h"
#include "pxr/base/plug/registry.h"
#include "pxr/base/vt/array.h"
#include "pxr/usd/sdf/layer.h"
#include "pxr/usd/usd/attribute.h"
#include "pxr/usd/usd/editContext.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usdGeom/xformable.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

using namespace rigExec;
PXR_NAMESPACE_USING_DIRECTIVE

static int failures = 0;
static int checks = 0;

#define CHECK(cond)                                                        \
    do {                                                                  \
        ++checks;                                                         \
        if (!(cond)) {                                                    \
            ++failures;                                                   \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);      \
        }                                                                 \
    } while (0)

namespace {

const char *kFixture = R"USDA(#usda 1.0
def Xform "World"
{
    def Xform "Asset"
    {
        def RigExecRoot "Rig"
        {
            def RigExecControl "Driver"
            {
                double avars:tx.timeSamples = {1: 0, 2: 0.25, 3: 0}
            }
            def RigExecControl "Source"
            {
                double rest:tx = 4
                double rest:ty = -2
                double rest:tz = 6
            }
            def Scope "Dials"
            {
                double amount = 0.3
            }
            def Scope "Movers"
            {
                def RigExecMatrixMover "Deform" (
                    prepend apiSchemas = ["RigExecMoverAPI"]
                )
                {
                    float inputs:defaultWeight = 1
                    rel rigExec:moves = </World/Asset/Surface.points>
                    rel rigExec:transform = </World/Asset/Rig/Driver>
                }
                def RigExecSurfaceProjector "Project" (
                    prepend apiSchemas = ["RigExecMoverAPI"]
                )
                {
                    rel rigExec:moves = </World/Asset/Surface.points>
                    rel rigExec:sources = </World/Asset/Rig/Source>
                    token rigExec:shaderPrimvar = "projectionFrame"
                    uniform token rigExec:shaderDialPrimvar = "projectionDials"
                    rel rigExec:shaderDialSources = </World/Asset/Rig/Dials.amount>
                    matrix4d rigExec:shaderOffset = ((1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0.4, 0.2, 0, 1))
                }
            }
        }
        def Mesh "Surface"
        {
            double3 xformOp:translate = (4, -2, 7)
            uniform token[] xformOpOrder = ["xformOp:translate"]
            point3f[] points = [(-2, -2, 0), (2, -2, 0), (2, 2, 0), (-2, 2, 0)]
            int[] faceVertexCounts = [4]
            int[] faceVertexIndices = [0, 1, 2, 3]
        }
    }
}
)USDA";

GfMatrix4d Translate(const GfVec3d &value)
{
    return GfMatrix4d(1.0).SetTranslate(value);
}

GfMatrix4d Rotate(double degrees)
{
    return GfMatrix4d(1.0).SetRotate(GfRotation(GfVec3d(0, 1, 0), degrees));
}

GfMatrix4d Scale(const GfVec3d &value)
{
    return GfMatrix4d(1.0).SetScale(value);
}

struct Placement {
    const char *name;
    GfMatrix4d asset;
    GfMatrix4d parent;
};

std::vector<Placement> Placements()
{
    return {{"identity", GfMatrix4d(1.0), GfMatrix4d(1.0)},
            {"translate20", Translate(GfVec3d(20, 0, 0)), GfMatrix4d(1.0)},
            {"rotate30", Rotate(30), GfMatrix4d(1.0)},
            {"scale2", Scale(GfVec3d(2)), GfMatrix4d(1.0)},
            {"nonuniform", Scale(GfVec3d(2, 0.6, 3)), GfMatrix4d(1.0)},
            {"nested", Scale(GfVec3d(2)) * Rotate(30) *
                       Translate(GfVec3d(20, 3, -8)),
                       Rotate(-15) * Translate(GfVec3d(-6, 9, 2))}};
}

void Place(const UsdStageRefPtr &stage, const SdfPath &path,
           const GfMatrix4d &matrix)
{
    CHECK(UsdGeomXformable(stage->GetPrimAtPath(path))
              .AddTransformOp(UsdGeomXformOp::PrecisionDouble,
                              TfToken("placement"))
              .Set(matrix));
}

using LayerState = std::vector<std::pair<SdfLayerHandle, std::string>>;

LayerState SnapshotLayers(const UsdStageRefPtr &stage)
{
    LayerState result;
    for (const SdfLayerHandle &layer : stage->GetUsedLayers()) {
        std::string text;
        CHECK(layer->ExportToString(&text));
        result.emplace_back(layer, std::move(text));
    }
    return result;
}

void CheckLayersUnchanged(const LayerState &before)
{
    for (const auto &[layer, text] : before) {
        std::string after;
        CHECK(layer->ExportToString(&after));
        CHECK(after == text);
    }
}

bool Near(const GfMatrix4d &a, const GfMatrix4d &b, double tolerance)
{
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (!std::isfinite(a[i][j]) || !std::isfinite(b[i][j]) ||
                std::abs(a[i][j] - b[i][j]) > tolerance) {
                return false;
            }
        }
    }
    return true;
}

bool Compile(RigExecRigEvaluator *rig)
{
    std::vector<std::string> errors;
    const bool compiled = rig->Compile(&errors);
    CHECK(compiled);
    for (const std::string &error : errors) {
        std::printf("    %s\n", error.c_str());
    }
    if (!compiled) {
        return false;
    }
    {
        std::vector<std::string> reasons;
        const bool bakeable = rig->IsBakeable(&reasons);
        CHECK(bakeable);
        for (const std::string &reason : reasons) {
            std::printf("    %s\n", reason.c_str());
        }
        return bakeable;
    }
    return true;
}

bool ReadMatrix(const RigExecRigPose &pose, const SdfPath &path,
                GfMatrix4d *value)
{
    const auto found = pose.movedProperties.find(path);
    CHECK(found != pose.movedProperties.end());
    if (found == pose.movedProperties.end()) {
        return false;
    }
    CHECK(found->second.IsHolding<GfMatrix4d>());
    if (!found->second.IsHolding<GfMatrix4d>()) {
        return false;
    }
    *value = found->second.Get<GfMatrix4d>();
    return true;
}

void CheckPose(const RigExecRigPose &pose)
{
    CHECK(pose.valid);
    for (const std::string &diagnostic : pose.diagnostics) {
        if (diagnostic.find("SurfaceProjector") != std::string::npos) {
            std::printf("    %s\n", diagnostic.c_str());
            CHECK(false);
        }
    }
}

void TestAnalyticSurface()
{
    const SdfPath rigPath("/World/Asset/Rig");
    const SdfPath pointsPath("/World/Asset/Surface.points");
    const SdfPath framePath("/World/Asset/Surface.primvars:projectionFrame");
    const SdfPath dialsPath("/World/Asset/Surface.primvars:projectionDials");
    const VtVec3fArray rest = {{-2, -2, 0}, {2, -2, 0},
                             {2, 2, 0}, {-2, 2, 0}};
    for (const Placement &placement : Placements()) {
        for (const bool reproject : {false, true}) {
            {
                std::printf("  analytic %-10s %s %s\n", placement.name,
                            reproject ? "reproject" : "material",
                            "graph");
                const SdfLayerRefPtr layer = SdfLayer::CreateAnonymous(".usda");
                CHECK(layer->ImportFromString(kFixture));
                const UsdStageRefPtr stage = UsdStage::Open(layer);
                CHECK(stage);
                if (!stage) {
                    continue;
                }
                Place(stage, SdfPath("/World"), placement.parent);
                Place(stage, SdfPath("/World/Asset"), placement.asset);
                const UsdPrim projector = stage->GetPrimAtPath(
                    SdfPath("/World/Asset/Rig/Movers/Project"));
                CHECK(projector.GetAttribute(TfToken("rigExec:projectionMode"))
                          .Set(TfToken(reproject ? "reproject" : "material")));
                const LayerState before = SnapshotLayers(stage);
                RigExecRigEvaluator rig(stage, rigPath);
                if (!Compile(&rig)) {
                    continue;
                }
                CHECK(rig.GetSurfaceProjectorTargets() ==
                      std::vector<SdfPath>{framePath});
                for (int frame = 1; frame <= 3; ++frame) {
                    const RigExecRigPose pose = rig.Evaluate(UsdTimeCode(frame));
                    CheckPose(pose);
                    const double tx = frame == 2 ? 0.25 : 0.0;
                    GfMatrix4d projectorMatrix;
                    if (ReadMatrix(pose, framePath, &projectorMatrix)) {
                        const GfMatrix4d expected = Translate(
                            GfVec3d(0.4 + (reproject ? 0 : tx), 0.2, 0));
                        CHECK(Near(projectorMatrix, expected, 1e-8));
                    }
                    GfMatrix4d packed;
                    if (ReadMatrix(pose, dialsPath, &packed)) {
                        GfMatrix4d expected(0.0);
                        expected[0][0] = 0.3;
                        CHECK(Near(packed, expected, 1e-12));
                    }
                    const auto moved = pose.movedProperties.find(pointsPath);
                    CHECK(moved != pose.movedProperties.end());
                    if (moved != pose.movedProperties.end()) {
                        CHECK(moved->second.IsHolding<VtVec3fArray>());
                        if (moved->second.IsHolding<VtVec3fArray>()) {
                            VtVec3fArray expected = rest;
                            for (GfVec3f &point : expected) {
                                point[0] += static_cast<float>(tx);
                            }
                            CHECK(moved->second.Get<VtVec3fArray>() == expected);
                        }
                    }
                }
                CHECK(rig.GetBakedGenerationCount() ==
                      3);
                CheckLayersUnchanged(before);
            }
        }
    }
}

void TestOutputSemantics()
{
    const SdfPath firstPath("/World/Asset/Rig/Movers/Project");
    const SdfPath secondPath("/World/Asset/Rig/Movers/ProjectOther");
    for (int combination = 0; combination < 4; ++combination) {
        const SdfLayerRefPtr layer = SdfLayer::CreateAnonymous(".usda");
        CHECK(layer->ImportFromString(kFixture));
        const UsdStageRefPtr stage = UsdStage::Open(layer);
        CHECK(stage);
        if (!stage) {
            continue;
        }
        const UsdPrim first = stage->GetPrimAtPath(firstPath);
        SdfPath conflict("/World/Asset/Surface.primvars:projectionFrame");
        if (combination == 0) {
            CHECK(first.GetAttribute(TfToken("rigExec:shaderDialPrimvar"))
                      .Set(TfToken("projectionFrame")));
        } else {
            const UsdPrim second = stage->DefinePrim(
                secondPath, TfToken("RigExecSurfaceProjector"));
            CHECK(second.ApplyAPI(TfToken("RigExecMoverAPI")));
            CHECK(second.GetRelationship(TfToken("rigExec:moves"))
                      .SetTargets({SdfPath("/World/Asset/Surface.points")}));
            CHECK(second.GetRelationship(TfToken("rigExec:sources"))
                      .SetTargets({SdfPath("/World/Asset/Rig/Source")}));
            CHECK(second.GetAttribute(TfToken("rigExec:shaderPrimvar"))
                      .Set(TfToken(combination == 1 ? "" :
                                   combination == 2 ? "projectionDials" :
                                                      "projectionFrame")));
            CHECK(second.GetAttribute(TfToken("rigExec:shaderDialPrimvar"))
                      .Set(TfToken(combination == 2 ? "" :
                                   combination == 1 ? "projectionFrame" :
                                                      "projectionDials")));
            if (combination == 2) {
                conflict = SdfPath("/World/Asset/Surface.primvars:projectionDials");
            }
        }
        const LayerState before = SnapshotLayers(stage);
        RigExecRigEvaluator rig(stage, SdfPath("/World/Asset/Rig"));
        std::vector<std::string> errors;
        const bool compiled = rig.Compile(&errors);
        // The compiler isolates invalid operations, preserving the valid
        // deform chain rather than rejecting the entire rig.
        CHECK(compiled);
        if (combination != 3) {
            CHECK(!errors.empty());
            CHECK(rig.GetSkippedOperations().count(firstPath) == 1);
            CHECK(rig.GetSkippedOperations().size() ==
                  (combination == 0 ? 1 : 2));
            if (combination != 0) {
                CHECK(rig.GetSkippedOperations().count(secondPath) == 1);
            }
            CHECK(rig.GetSurfaceProjectorTargets().empty());
            const RigExecRigPose pose = rig.Evaluate(UsdTimeCode(2));
            CheckPose(pose);
            CHECK(pose.movedProperties.count(
                      SdfPath("/World/Asset/Surface.points")) == 1);
            CHECK(pose.movedProperties.count(
                      SdfPath("/World/Asset/Surface.primvars:projectionFrame")) == 0);
            CHECK(pose.movedProperties.count(
                      SdfPath("/World/Asset/Surface.primvars:projectionDials")) == 0);
            if (!errors.empty()) {
                const std::string &error = errors.front();
                CHECK(error.find(conflict.GetString()) != std::string::npos);
                CHECK(error.find(firstPath.GetString()) != std::string::npos);
                CHECK(error.find(combination == 0 ? firstPath.GetString() :
                                                   secondPath.GetString()) !=
                      std::string::npos);
                CHECK(error.find("SurfaceProjector") != std::string::npos);
                CHECK(error.find("ShaderDials") != std::string::npos);
                CHECK(error.find("choose different") != std::string::npos);
            }
        } else {
            CHECK(errors.empty());
            CHECK(rig.GetSkippedOperations().empty());
            CHECK(rig.GetSurfaceProjectorTargets() == std::vector<SdfPath>{
                SdfPath("/World/Asset/Surface.primvars:projectionFrame")});
        }
        CheckLayersUnchanged(before);
    }
}

void TestBiped(const std::string &examples)
{
    const SdfPath rigPath("/Biped/Rig");
    const SdfPath left("/Biped/Geom/l_eye_geo.primvars:eyeProjector");
    const SdfPath right("/Biped/Geom/r_eye_geo.primvars:eyeProjector");
    std::vector<GfMatrix4d> reference;
    std::vector<VtValue> referencePoints;
    for (const Placement &placement : Placements()) {
        {
            std::printf("  biped    %-10s %s\n", placement.name,
                        "graph");
            const UsdStageRefPtr stage = UsdStage::Open(
                examples + "/biped/Biped_stack.usda");
            CHECK(stage);
            if (!stage) {
                continue;
            }
            // Author only a session placement; the shipped layers stay intact.
            const UsdEditContext edit(stage, stage->GetSessionLayer());
            Place(stage, SdfPath("/Biped"), placement.asset * placement.parent);
            const LayerState before = SnapshotLayers(stage);
            RigExecRigEvaluator rig(stage, rigPath);
            if (!Compile(&rig)) {
                continue;
            }
            const std::vector<SdfPath> targets = rig.GetSurfaceProjectorTargets();
            CHECK(targets == std::vector<SdfPath>({left, right}));
            const RigExecRigPose pose = rig.Evaluate(UsdTimeCode(1));
            CheckPose(pose);
            std::vector<GfMatrix4d> matrices;
            std::vector<VtValue> points;
            for (const SdfPath &target : {left, right}) {
                GfMatrix4d matrix;
                if (ReadMatrix(pose, target, &matrix)) {
                    matrices.push_back(matrix);
                }
                const SdfPath geometry = target.GetPrimPath()
                    .AppendProperty(TfToken("points"));
                const auto moved = pose.movedProperties.find(geometry);
                CHECK(moved != pose.movedProperties.end());
                if (moved != pose.movedProperties.end()) {
                    CHECK(moved->second.IsHolding<VtVec3fArray>());
                    if (moved->second.IsHolding<VtVec3fArray>()) {
                        CHECK(!moved->second.Get<VtVec3fArray>().empty());
                    }
                    points.push_back(moved->second);
                }
            }
            if (reference.empty()) {
                reference = matrices;
                referencePoints = points;
            } else {
                CHECK(matrices.size() == reference.size());
                for (size_t i = 0; i < std::min(matrices.size(), reference.size());
                     ++i) {
                    CHECK(Near(matrices[i], reference[i], 1e-5));
                }
                CHECK(points == referencePoints);
            }
            CHECK(rig.GetBakedGenerationCount() ==
                  1);
            CheckLayersUnchanged(before);
        }
    }
}

} // namespace

int main(int argc, char **argv)
{
    PlugRegistry::GetInstance().RegisterPlugins(RIGEXEC_SCHEMA_RESOURCE_DIR);
    CHECK(argc == 2);
    if (argc != 2) {
        return 1;
    }
    TestOutputSemantics();
    TestAnalyticSurface();
    TestBiped(argv[1]);
    std::printf("SurfaceProjector: %d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
