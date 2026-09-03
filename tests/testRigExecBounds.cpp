//
// RigExec native bounds (host-durability redesign): UsdGeomBBoxCache must
// answer for RigExec prims in ANY host, with no RigExec code running.
//
// This executable deliberately does NOT link rigExecImaging. It registers
// the codeless schema plugin and nothing else, so the only way an extent
// can be produced is the one the redesign relies on: Plug reads
// implementsComputeExtent off the schema type, loads the library named by
// the schema plugInfo's LibraryPath, and that library's
// TF_REGISTRY_FUNCTION registers the compute-extent function on demand.
//
// Linking the imaging library here would register it at load time and the
// test would pass without proving anything -- which is exactly the failure
// this file exists to prevent.
//
#include "pxr/base/plug/registry.h"
#include "pxr/base/tf/pathUtils.h"
#include "pxr/base/gf/range3d.h"
#include "pxr/usd/usd/primRange.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usd/relationship.h"
#include "pxr/usd/usdGeom/bboxCache.h"
#include "pxr/usd/usdGeom/boundable.h"
#include "pxr/usd/usdGeom/imageable.h"
#include "pxr/usd/usdGeom/tokens.h"

#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

static int failures = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        if (!(cond)) {                                                     \
            ++failures;                                                    \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);    \
        }                                                                  \
    } while (0)

static bool
_Near(const GfVec3d &a, const GfVec3d &b, double tolerance = 1e-4)
{
    for (size_t i = 0; i < 3; ++i) {
        if (std::abs(a[i] - b[i]) > tolerance) {
            return false;
        }
    }
    return true;
}

int
main(int argc, char **argv)
{
    if (argc < 2) {
        std::printf("usage: testRigExecBounds <examplesDir>\n");
        return 2;
    }
    const std::string examplesDir = argv[1];
#ifdef RIGEXEC_SCHEMA_RESOURCE_DIR
    const std::string resources = TfAbsPath(RIGEXEC_SCHEMA_RESOURCE_DIR);
#else
    const std::string resources =
        TfAbsPath(examplesDir + "/../plugin/rigExecSchema/resources");
#endif
    if (PlugRegistry::GetInstance().RegisterPlugins(resources).empty()) {
        std::printf("FATAL: no schema plugin found at %s\n",
                    resources.c_str());
        return 2;
    }

    UsdStageRefPtr stage =
        UsdStage::Open(examplesDir + "/01_FkChainTail.usda");
    CHECK(stage);
    if (!stage) {
        return 1;
    }

    // Boundable at all: the codeless RigExecXformable inherits
    // UsdGeomBoundable, so every joint and control is one.
    const SdfPath control("/TailAsset/Rig/Controls/Tail1");
    const SdfPath joint("/TailAsset/Rig/Joints/Seg1");
    CHECK(stage->GetPrimAtPath(control).IsA<UsdGeomBoundable>());
    CHECK(stage->GetPrimAtPath(joint).IsA<UsdGeomBoundable>());

    // What every host actually consults.
    UsdGeomBBoxCache cache(
        UsdTimeCode(1001),
        {UsdGeomTokens->default_, UsdGeomTokens->guide,
         UsdGeomTokens->proxy, UsdGeomTokens->render});

    // Tail1's diamond: unit shape at rest (0, 5, 0), per-axis scale 1.6.
    // The bounds come from AUTHORED attributes alone -- no rig has been
    // compiled and no generation published in this process.
    {
        const GfRange3d range =
            cache.ComputeWorldBound(stage->GetPrimAtPath(control))
                .ComputeAlignedRange();
        if (range.IsEmpty()) {
            std::printf("  control bbox EMPTY -- the compute-extent "
                        "function was never loaded\n");
        }
        CHECK(!range.IsEmpty());
        // 1.6 scale inflated by half the 0.05 wire width, because that
        // is what the curve actually occupies.
        CHECK(_Near(range.GetMin(), GfVec3d(-1.64, 3.36, -1.64)));
        CHECK(_Near(range.GetMax(), GfVec3d(1.64, 6.64, 1.64)));
    }

    // Evaluated control-scale magnitudes multiply the authored guide scale.
    // The unevaluated extent fallback has only authored values available, but
    // must produce the same size -- including keeping a reflected axis visible
    // by its magnitude.
    {
        const UsdPrim prim = stage->GetPrimAtPath(control);
        constexpr double expectedAvarScaleFloor = 1e-4;
        CHECK(prim.GetAttribute(TfToken("avars:sx")).Set(-2.0));
        CHECK(prim.GetAttribute(TfToken("avars:sy")).Set(0.5));
        CHECK(prim.GetAttribute(TfToken("avars:sz")).Set(3.0));
        UsdGeomBBoxCache scaledCache(
            UsdTimeCode(1001),
            {UsdGeomTokens->default_, UsdGeomTokens->guide,
             UsdGeomTokens->proxy, UsdGeomTokens->render});
        const GfRange3d range =
            scaledCache.ComputeWorldBound(prim).ComputeAlignedRange();
        CHECK(!range.IsEmpty());
        // guide scale 1.6 * |avar scale|, then half the 0.05 wire width.
        CHECK(_Near(range.GetMin(), GfVec3d(-3.28, 4.18, -4.92)));
        CHECK(_Near(range.GetMax(), GfVec3d(3.28, 5.82, 4.92)));

        // A zero AVAR scale is normalized to the shared nonzero floor; unlike
        // zero guide:scale below, it keeps a very thin but real guide/bound.
        CHECK(prim.GetAttribute(TfToken("avars:sx")).Set(0.0));
        CHECK(prim.GetAttribute(TfToken("avars:sy")).Set(-0.0));
        CHECK(prim.GetAttribute(TfToken("avars:sz"))
                  .Set(-0.5 * expectedAvarScaleFloor));
        UsdGeomBBoxCache floorCache(
            UsdTimeCode(1001),
            {UsdGeomTokens->default_, UsdGeomTokens->guide,
             UsdGeomTokens->proxy, UsdGeomTokens->render});
        const GfRange3d floorRange =
            floorCache.ComputeWorldBound(prim).ComputeAlignedRange();
        CHECK(!floorRange.IsEmpty());
        const double floorHalf =
            1.6 * expectedAvarScaleFloor * 1.025;
        CHECK(_Near(floorRange.GetMin(),
                    GfVec3d(-floorHalf, 5.0 - floorHalf, -floorHalf),
                    1e-6));
        CHECK(_Near(floorRange.GetMax(),
                    GfVec3d(floorHalf, 5.0 + floorHalf, floorHalf),
                    1e-6));

        // Restore the fixture before subtree and invalid-guide assertions.
        CHECK(prim.GetAttribute(TfToken("avars:sx")).Set(1.0));
        CHECK(prim.GetAttribute(TfToken("avars:sy")).Set(1.0));
        CHECK(prim.GetAttribute(TfToken("avars:sz")).Set(1.0));
    }

    // Seg1's guide, plus every joint NESTED under it: a Boundable's extent
    // is authoritative for its whole subtree, because BBoxCache stops
    // descending at one. Seg1..Seg4 sit at x = 0, 2, 4, 6 with radius 1;
    // links end at their child origins and the leaf contributes no stub.
    {
        const GfRange3d range =
            cache.ComputeWorldBound(stage->GetPrimAtPath(joint))
                .ComputeAlignedRange();
        CHECK(!range.IsEmpty());
        CHECK(_Near(range.GetMin(), GfVec3d(-1, 4, -1)));
        CHECK(_Near(range.GetMax(), GfVec3d(7, 6, 1)));
    }

    // Framing the whole rig spans every provider beneath it.
    {
        const GfRange3d range =
            cache.ComputeWorldBound(
                     stage->GetPrimAtPath(SdfPath("/TailAsset/Rig")))
                .ComputeAlignedRange();
        CHECK(!range.IsEmpty());
        // The terminal joint sphere reaches x=7; Tail4's larger pyramid
        // extends slightly farther. Z comes from Tail3's 1.8 scale with the
        // same wire inflation.
        CHECK(_Near(range.GetMin(), GfVec3d(-1.64, 3.36, -1.845)));
        CHECK(_Near(range.GetMax(), GfVec3d(7.5375, 6.64, 1.845)));
    }

    // A guide-only component must contribute its fallback bounds to the rig
    // and to any imageable ancestors above it. A merely Typed rig root makes
    // the joint's valid bound disappear from fit-stage camera framing. No
    // guide length is authored in this fixture: this is the hierarchy-derived
    // cold fallback path end to end.
    {
        const UsdStageRefPtr component = UsdStage::Open(
            examplesDir + "/components/spider_leg.usd");
        CHECK(component);
        if (component) {
            const SdfPath rigPath("/RigRoot");
            const SdfPath jointPath("/RigRoot/Joints/Shoulder");
            const SdfPath childPath(
                "/RigRoot/Joints/Shoulder/ankle");
            if (component->GetPrimAtPath(rigPath) &&
                component->GetPrimAtPath(jointPath) &&
                component->GetPrimAtPath(childPath)) {
                const UsdPrim rig = component->GetPrimAtPath(rigPath);
                CHECK(rig.IsA<UsdGeomImageable>());

                // Exercise the outermost imageable ancestor when the
                // recreated component supplies one, and the rig itself when
                // it is a top-level prim. Both are valid component layouts.
                UsdPrim framingPrim = rig;
                for (UsdPrim ancestor = rig.GetParent();
                     ancestor && !ancestor.IsPseudoRoot();
                     ancestor = ancestor.GetParent()) {
                    if (ancestor.IsA<UsdGeomImageable>()) {
                        framingPrim = ancestor;
                    }
                }
                UsdGeomBBoxCache componentCache(
                    UsdTimeCode::Default(),
                    {UsdGeomTokens->default_, UsdGeomTokens->guide});
                const GfRange3d range =
                    componentCache.ComputeWorldBound(framingPrim)
                        .ComputeAlignedRange();
                CHECK(!range.IsEmpty());
                const GfVec3d childRest(
                    4.476721406958012, -2.6550437955052333, 0.0);
                CHECK(_Near(range.GetMin(),
                            GfVec3d(-1, childRest[1] - 1, -1)));
                CHECK(_Near(range.GetMax(),
                            GfVec3d(childRest[0] + 1, 1, 1)));

                // The property is gone from the composed schema and the
                // parent joint's own cold extent still includes its child.
                const UsdPrim jointPrim = component->GetPrimAtPath(jointPath);
                CHECK(!jointPrim.HasProperty(TfToken("guide:length")));
                UsdGeomBBoxCache jointCache(
                    UsdTimeCode::Default(),
                    {UsdGeomTokens->default_, UsdGeomTokens->guide});
                const GfRange3d jointRange =
                    jointCache.ComputeWorldBound(jointPrim)
                        .ComputeAlignedRange();
                CHECK(!jointRange.IsEmpty());
                CHECK(_Near(jointRange.GetMin(),
                            GfVec3d(-1, childRest[1] - 1, -1)));
                CHECK(_Near(jointRange.GetMax(),
                            GfVec3d(childRest[0] + 1, 1, 1)));
            }
        }
    }

    // Placed weight volumes have a cold authored fallback just like controls
    // and joints. No evaluator or imaging bridge is linked into this test:
    // these bounds prove the schema plugin metadata loads the callback and a
    // rigger can frame an unwired SphereWeight, PlaneWeight, or valid
    // CurveWeight immediately after authoring it.
    {
        const UsdStageRefPtr volumes = UsdStage::CreateInMemory();
        const SdfPath rigPath("/Volumes/Rig");
        volumes->DefinePrim(rigPath, TfToken("RigExecRoot"));

        const SdfPath spherePath =
            rigPath.AppendChild(TfToken("Sphere"));
        const UsdPrim sphere = volumes->DefinePrim(
            spherePath, TfToken("RigExecSphereWeight"));
        GfMatrix4d sphereRest(1.0);
        sphereRest.SetTranslate(GfVec3d(10, 0, 0));
        CHECK(sphere.GetAttribute(TfToken("rest:space")).Set(sphereRest));
        CHECK(sphere.GetAttribute(TfToken("inputs:scaleX")).Set(2.0f));
        CHECK(sphere.GetAttribute(TfToken("inputs:falloffMax")).Set(3.0f));

        const SdfPath planePath = rigPath.AppendChild(TfToken("Plane"));
        const UsdPrim plane = volumes->DefinePrim(
            planePath, TfToken("RigExecPlaneWeight"));
        CHECK(plane.GetAttribute(TfToken("inputs:falloffMax")).Set(2.0f));
        CHECK(plane.GetAttribute(TfToken("inputs:extentU")).Set(2.0f));
        CHECK(plane.GetAttribute(TfToken("inputs:extentV")).Set(3.0f));

        const SdfPath curveSourcePath =
            rigPath.AppendChild(TfToken("CurveSource"));
        const UsdPrim curveSource =
            volumes->DefinePrim(curveSourcePath, TfToken("BasisCurves"));
        CHECK(curveSource
                  .CreateAttribute(TfToken("points"),
                                   SdfValueTypeNames->Point3fArray)
                  .Set(VtVec3fArray{GfVec3f(-2, 0, 0),
                                    GfVec3f(2, 0, 0)}));
        const SdfPath curveWeightPath =
            rigPath.AppendChild(TfToken("CurveWeight"));
        const UsdPrim curveWeight = volumes->DefinePrim(
            curveWeightPath, TfToken("RigExecCurveWeight"));
        CHECK(curveWeight
                  .CreateRelationship(TfToken("rigExec:curve"))
                  .SetTargets({curveSourcePath.AppendProperty(
                      TfToken("points"))}));
        CHECK(curveWeight.GetAttribute(TfToken("inputs:falloffMin"))
                  .Set(0.5f));

        UsdGeomBBoxCache volumeCache(
            UsdTimeCode::Default(),
            {UsdGeomTokens->default_, UsdGeomTokens->guide});
        const GfRange3d sphereRange =
            volumeCache.ComputeWorldBound(sphere).ComputeAlignedRange();
        CHECK(!sphereRange.IsEmpty());
        // Outer radius 3, X divisor 2, and a 0.05 wire widened by half in
        // unit-sphere space: (6,3,3) * 1.025 about rest translation X=10.
        CHECK(_Near(sphereRange.GetMin(), GfVec3d(3.85, -3.075, -3.075)));
        CHECK(_Near(sphereRange.GetMax(), GfVec3d(16.15, 3.075, 3.075)));

        const GfRange3d planeRange =
            volumeCache.ComputeWorldBound(plane).ComputeAlignedRange();
        CHECK(!planeRange.IsEmpty());
        // Default Y plane, unbounded marker ticks at 1.25 times U/V, and
        // both signed-distance surfaces at Y=0 and Y=2.
        CHECK(_Near(planeRange.GetMin(), GfVec3d(-3.775, -0.025, -2.525)));
        CHECK(_Near(planeRange.GetMax(), GfVec3d(3.775, 2.025, 2.525)));

        const GfRange3d curveRange =
            volumeCache.ComputeWorldBound(curveWeight).ComputeAlignedRange();
        CHECK(!curveRange.IsEmpty());
        CHECK(curveRange.GetMin()[0] <= -3.0);
        CHECK(curveRange.GetMax()[0] >= 3.0);
        CHECK(curveRange.GetMin()[1] <= -1.0);
        CHECK(curveRange.GetMax()[1] >= 1.0);

        // The cold callback must reject exactly the curves the publication
        // bridge cannot build: fewer than two distinct consecutive points or
        // any non-finite point. Otherwise framing would find geometry the
        // viewport intentionally does not publish.
        CHECK(curveSource.GetAttribute(TfToken("points"))
                  .Set(VtVec3fArray{GfVec3f(1, 2, 3),
                                    GfVec3f(1, 2, 3)}));
        UsdGeomBBoxCache duplicateCurveCache(
            UsdTimeCode::Default(),
            {UsdGeomTokens->default_, UsdGeomTokens->guide});
        CHECK(duplicateCurveCache.ComputeWorldBound(curveWeight)
                  .ComputeAlignedRange().IsEmpty());
        CHECK(curveSource.GetAttribute(TfToken("points"))
                  .Set(VtVec3fArray{
                      GfVec3f(0, 0, 0),
                      GfVec3f(std::numeric_limits<float>::infinity(), 0, 0)}));
        UsdGeomBBoxCache nonFiniteCurveCache(
            UsdTimeCode::Default(),
            {UsdGeomTokens->default_, UsdGeomTokens->guide});
        CHECK(nonFiniteCurveCache.ComputeWorldBound(curveWeight)
                  .ComputeAlignedRange().IsEmpty());

        // Required curve authoring stays required, and `none` remains a real
        // guide suppression mode rather than a phantom framing extent.
        const UsdPrim invalidCurve = volumes->DefinePrim(
            rigPath.AppendChild(TfToken("InvalidCurve")),
            TfToken("RigExecCurveWeight"));
        CHECK(volumeCache.ComputeWorldBound(invalidCurve)
                  .ComputeAlignedRange().IsEmpty());
        CHECK(sphere.GetAttribute(TfToken("guide:drawMode"))
                  .Set(TfToken("none")));
        UsdGeomBBoxCache hiddenCache(
            UsdTimeCode::Default(),
            {UsdGeomTokens->default_, UsdGeomTokens->guide});
        CHECK(hiddenCache.ComputeWorldBound(sphere)
                  .ComputeAlignedRange().IsEmpty());
    }

    // A control whose scale draws nothing bounds nothing -- rather than
    // reporting a degenerate box at the origin, which would frame the
    // camera on empty space.
    {
        UsdPrim prim = stage->GetPrimAtPath(control);
        CHECK(prim.GetAttribute(TfToken("guide:scaleY")).Set(0.0));
        UsdGeomBBoxCache fresh(UsdTimeCode(1001),
                               {UsdGeomTokens->default_});
        CHECK(fresh.ComputeWorldBound(prim).ComputeAlignedRange().IsEmpty());
    }

    if (failures) {
        std::printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("testRigExecBounds: all tests passed\n");
    return 0;
}
