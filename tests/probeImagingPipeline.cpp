//
// End-to-end probe of the REAL UsdImaging scene index chain.
//
// The C++ unit tests build the chain by hand, and a hand-built chain is only
// ever as truthful as its author's model of the real one. This probe builds
// no chain: it calls UsdImagingCreateSceneIndices exactly as usdview does,
// lets the RigExec scene index plugin insert itself wherever UsdImaging
// decides to insert it, activates through the same C entry point the usdview
// plugin uses, and then reads transforms off the terminal scene index --
// the same data source HdSceneIndexAdapterSceneDelegate::GetTransform reads
// to drive Storm.
//
// It exists because a synthetic harness put the flattening scene index
// downstream of RigExec, where the real chain puts it upstream, and the
// resulting test passed while the viewport stayed static.
//
// WHAT THIS PROBE DOES NOT COVER, so nobody mistakes a pass for one:
//   - Invalidation. Every assertion is a direct GetPrim() pull, and a pull
//     always sees the freshly swapped store. A build that published results
//     and never dirtied anything would pass here and still render a frozen
//     viewport. That is testRigExecImaging's recording-observer assertions.
//   - The full usdview chain. UsdImagingGL adds override/display/prefix,
//     merge, application/renderer and optional caching indices on top of
//     UsdImagingCreateSceneIndices. This covers the RigExec placement, not
//     everything downstream of it.
//   - Native instancing. Inside a prototype the "world" space these
//     transforms live in is prototype-common space, which is untested.
//
#include "rigExecImaging/registry.h"

#include "pxr/imaging/hd/sceneIndex.h"
#include "pxr/imaging/hd/xformSchema.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/xformCache.h"
#include "pxr/usd/usdUtils/stageCache.h"
#include "pxr/usdImaging/usdImaging/sceneIndices.h"

#include <cstdio>
#include <string>
#include <vector>

PXR_NAMESPACE_USING_DIRECTIVE

namespace {

int failures = 0;

void
Check(bool condition, const std::string &what)
{
    std::printf("  %-58s %s\n", what.c_str(), condition ? "ok" : "FAIL");
    if (!condition) {
        ++failures;
    }
}

bool
ReadXform(const HdSceneIndexBaseRefPtr &sceneIndex, const SdfPath &path,
          GfMatrix4d *out)
{
    const HdSceneIndexPrim prim = sceneIndex->GetPrim(path);
    if (!prim.dataSource) {
        return false;
    }
    HdXformSchema schema = HdXformSchema::GetFromParent(prim.dataSource);
    if (!schema || !schema.GetMatrix()) {
        return false;
    }
    *out = schema.GetMatrix()->GetTypedValue(0.0);
    return true;
}

}  // namespace

int
main(int argc, char **argv)
{
    if (argc < 2) {
        std::printf("usage: probeImagingPipeline <examplesDir>\n");
        return 2;
    }
    const std::string examples = argv[1];
    const std::string path = examples + "/10_AimXformTurret.usda";

    UsdStageRefPtr stage = UsdStage::Open(path);
    if (!stage) {
        std::printf("could not open %s\n", path.c_str());
        return 2;
    }

    // Build the chain the way usdview does. The RigExec plugin inserts itself
    // through UsdImagingSceneIndexPlugin; nothing here places it.
    UsdImagingCreateSceneIndicesInfo info;
    info.stage = stage;
    const UsdImagingSceneIndices sceneIndices =
        UsdImagingCreateSceneIndices(info);
    const HdSceneIndexBaseRefPtr terminal = sceneIndices.finalSceneIndex;
    Check(terminal != nullptr, "terminal scene index exists");
    if (!terminal) {
        return 1;
    }

    // Activate through the same C surface the usdview plugin calls, with an
    // empty rig path so the registry discovers the rig itself.
    const long long cacheId =
        UsdUtilsStageCache::Get().Insert(stage).ToLongInt();
    Check(RigExecImaging_Activate(cacheId, "", 1001.0) == 0,
          "RigExecImaging_Activate(empty rig path)");

    const SdfPath turret("/TurretAsset/Geom/Turret");
    const SdfPath barrel("/TurretAsset/Geom/Turret/Barrel");
    const SdfPath sight("/TurretAsset/Geom/Turret/Sight");

    GfMatrix4d turretAt1001(1.0), barrelAt1001(1.0), sightAt1001(1.0);
    Check(ReadXform(terminal, turret, &turretAt1001), "turret xform present");
    Check(ReadXform(terminal, barrel, &barrelAt1001), "barrel xform present");
    Check(ReadXform(terminal, sight, &sightAt1001), "sight xform present");

    RigExecImaging_SetTime(1024.0);

    GfMatrix4d turretAt1024(1.0), barrelAt1024(1.0), sightAt1024(1.0);
    Check(ReadXform(terminal, turret, &turretAt1024),
          "turret xform still present at 1024");
    Check(ReadXform(terminal, barrel, &barrelAt1024),
          "barrel xform still present at 1024");
    Check(ReadXform(terminal, sight, &sightAt1024),
          "sight xform still present at 1024");

    // The driven Xform must move...
    Check(turretAt1001 != turretAt1024, "driven Xform changes 1001 -> 1024");
    // ...and so must the geometry parented under it. This is the assertion
    // the whole probe exists for: the parent moving while the children stay
    // put is precisely "animates in the scene index, static in the viewport".
    Check(barrelAt1001 != barrelAt1024, "parented Barrel changes 1001 -> 1024");
    Check(sightAt1001 != sightAt1024, "parented Sight changes 1001 -> 1024");

    // Both meshes have an identity local transform in the example, so after
    // flattening their world transforms must equal the turret's exactly.
    Check(GfIsClose(barrelAt1024, turretAt1024, 1e-9),
          "Barrel world == Turret world (identity local)");
    Check(GfIsClose(sightAt1024, turretAt1024, 1e-9),
          "Sight world == Turret world (identity local)");

    // And the aim must actually aim -- at the target's WORLD position.
    //
    // A direction-sign check ("z swings from -x to +x") is not enough: a
    // constraint that solves the provider's LOCAL frame against a
    // world-space target also swings the right way, and passes. Compare
    // against the vector computed independently from the stage instead.
    UsdGeomXformCache cache(UsdTimeCode(1024));
    const GfVec3d turretWorldOrigin =
        cache.GetLocalToWorldTransform(stage->GetPrimAtPath(turret))
            .ExtractTranslation();

    // The target's origin does NOT come from xformOps -- a RigExecControl is
    // positioned by rest:space plus its avars, so an xform cache reads it as
    // identity. Derive it from the authored rig data instead, which keeps
    // this oracle independent of the engine's own frame plumbing.
    //
    // rest:space is ASSET space: it carries no stage placement. Composing the
    // asset root's world transform onto it is what makes this a world-space
    // oracle -- without that step it would happily confirm a solve that mixed
    // asset and world origins, which is exactly the bug it exists to catch.
    const UsdPrim targetPrim =
        stage->GetPrimAtPath(SdfPath("/TurretAsset/Rig/Controls/TrackTarget"));
    GfMatrix4d targetRest(1.0);
    targetPrim.GetAttribute(TfToken("rest:space")).Get(&targetRest);
    double targetTx = 0.0;
    targetPrim.GetAttribute(TfToken("avars:tx"))
        .Get(&targetTx, UsdTimeCode(1024));
    const GfMatrix4d assetRootWorld = cache.GetLocalToWorldTransform(
        stage->GetPrimAtPath(SdfPath("/TurretAsset")));
    const GfVec3d targetWorldOrigin = assetRootWorld.Transform(
        targetRest.ExtractTranslation() + GfVec3d(targetTx, 0, 0));

    const GfVec3d expectedAim =
        (targetWorldOrigin - turretWorldOrigin).GetNormalized();
    std::printf("  turret world origin=(%.3f,%.3f,%.3f) "
                "target=(%.3f,%.3f,%.3f)\n",
                turretWorldOrigin[0], turretWorldOrigin[1],
                turretWorldOrigin[2], targetWorldOrigin[0],
                targetWorldOrigin[1], targetWorldOrigin[2]);

    const GfVec3d aimAt1001 =
        turretAt1001.TransformDir(GfVec3d(0, 0, 1)).GetNormalized();
    const GfVec3d aimAt1024 =
        turretAt1024.TransformDir(GfVec3d(0, 0, 1)).GetNormalized();
    std::printf("  aim z-axis 1001=(%.3f,%.3f,%.3f) 1024=(%.3f,%.3f,%.3f)\n",
                aimAt1001[0], aimAt1001[1], aimAt1001[2],
                aimAt1024[0], aimAt1024[1], aimAt1024[2]);
    std::printf("  expected  1024=(%.3f,%.3f,%.3f) from stage world origins\n",
                expectedAim[0], expectedAim[1], expectedAim[2]);
    Check(aimAt1001[0] < 0.0 && aimAt1024[0] > 0.0,
          "aim tracks the target across the scene");
    Check(GfIsClose(aimAt1024, expectedAim, 1e-6),
          "aim points at the target's WORLD position");

    // An aim constraint rotates; it must not move the prim. Checking only a
    // normalized direction would accept a result with the right orientation
    // and a wrong origin, scale, or shear.
    Check(GfIsClose(turretAt1024.ExtractTranslation(), turretWorldOrigin,
                    1e-6),
          "driven Xform keeps its authored world origin");

    // GUIDE PLACEMENT. The Base joint sits at the asset's origin in rig
    // space, and the asset itself is placed at x=+100, so its guide must be
    // drawn there too -- not back at the world origin.
    const SdfPath baseGuide(
        "/TurretAsset/Rig/Joints/Base/rigGuideSphere_0");
    GfMatrix4d guideXform(1.0);
    // MANDATORY, not conditional: a regression that stopped synthesizing
    // guides entirely would otherwise print a note and pass.
    Check(ReadXform(terminal, baseGuide, &guideXform),
          "joint guide prim exists in the real chain");
    const GfVec3d g = guideXform.ExtractTranslation();
    const GfVec3d assetOrigin = assetRootWorld.ExtractTranslation();
    std::printf("  guide origin=(%.3f,%.3f,%.3f) asset=(%.3f,%.3f,%.3f)\n",
                g[0], g[1], g[2],
                assetOrigin[0], assetOrigin[1], assetOrigin[2]);
    // The Base joint's rest:space is identity, so its guide's world origin
    // must equal the asset placement EXACTLY -- all three axes, not an
    // "within one unit" check on X that a wrong Y or Z would survive.
    Check(GfIsClose(g, assetOrigin, 1e-6),
          "joint guide sits exactly at the asset's placement");

    RigExecImaging_Deactivate();

    std::printf("%s\n", failures == 0 ? "probeImagingPipeline: PASS"
                                      : "probeImagingPipeline: FAILURES");
    return failures == 0 ? 0 : 1;
}
