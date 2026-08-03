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
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/bboxCache.h"
#include "pxr/usd/usdGeom/boundable.h"
#include "pxr/usd/usdGeom/tokens.h"

#include <cmath>
#include <cstdio>
#include <string>

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
_Near(const GfVec3d &a, const GfVec3d &b)
{
    for (size_t i = 0; i < 3; ++i) {
        if (std::abs(a[i] - b[i]) > 1e-4) {
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

    // Seg1's guide, plus every joint NESTED under it: a Boundable's extent
    // is authoritative for its whole subtree, because BBoxCache stops
    // descending at one. Seg1..Seg4 sit at x = 0, 2, 4, 6 with radius 1 and
    // a cone of length 2, so the subtree reaches 6 + 2 + 1.
    {
        const GfRange3d range =
            cache.ComputeWorldBound(stage->GetPrimAtPath(joint))
                .ComputeAlignedRange();
        CHECK(!range.IsEmpty());
        CHECK(_Near(range.GetMin(), GfVec3d(-1, 4, -1)));
        CHECK(_Near(range.GetMax(), GfVec3d(9, 6, 1)));
    }

    // Framing the whole rig spans every provider beneath it.
    {
        const GfRange3d range =
            cache.ComputeWorldBound(
                     stage->GetPrimAtPath(SdfPath("/TailAsset/Rig")))
                .ComputeAlignedRange();
        CHECK(!range.IsEmpty());
        // Reaches the joint chain's far end (9), not merely the outermost
        // control -- the subtree rule again -- and Z comes from Tail3's
        // 1.8 scale with the same wire inflation.
        CHECK(_Near(range.GetMin(), GfVec3d(-1.64, 3.36, -1.845)));
        CHECK(_Near(range.GetMax(), GfVec3d(9, 6.64, 1.845)));
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
