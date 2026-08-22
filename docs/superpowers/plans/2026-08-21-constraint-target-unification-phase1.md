# Constraint Target Unification — Phase 1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make any `UsdGeomXformable` prim — Mesh and BasisCurves included — a legal transform-domain target for a source-frame constraint, by removing the write-path type inference that silently rewrote such targets into `.points`.

**Architecture:** `_CanonicalizeTarget` currently conflates two jobs: resolving a *write* target and resolving a *read* geometry input. It is renamed to `_ResolveGeometryInput` and kept for the three read call sites; the two write call sites drop the call entirely, so an authored path reaches `record.targets` exactly as written. The `IsPrimPath()` proxy that stood in for "is a transform provider" is replaced by the real predicate `UsdGeomXformable(prim)` — the same one `bindFrameSource` already uses for constraint *sources*. Geometry-domain constraint targets (`<prim>.points`) get an explicit "not yet supported" compile error, which Phase 4 replaces with an implementation.

**Tech Stack:** C++17, OpenUSD (install at `/Users/burkard/work/usd-install`), CMake + Ninja, hand-rolled `CHECK` test macro, ctest.

**Spec:** `docs/superpowers/specs/2026-08-21-constraint-target-unification-design.md` (sections 1, 2, 4.1, 5, 6)

## Global Constraints

- Schema is **codeless** (`schema.usda:23`, `skipCodeGeneration = true`). **Phase 1 changes no schema file** and therefore requires no `usdGenSchema` run and no cmake reconfigure.
- Build: `cmake --build build`. The build directory already exists and is configured.
- Test: `PYTHONPATH=/Users/burkard/work/usd-install/lib/python3.11/site-packages ctest --test-dir build --output-on-failure`
- Manual evaluation: `export PXR_PLUGINPATH_NAME="$PWD/build/usd/rigExecSchema/resources:$PWD/build/usd/rigExecImaging/resources:$PWD/plugin/rigExecUsdview"` then `./build/rigExecPose <stage> --frames <list> --targets --joints`
- All 12 existing ctest suites must stay green at every commit.
- Read-side inference **must not** be removed: `06_LatticeBulge.usda`, `07_SurfaceDrape.usda` (×2) and `13_ReadPhases.usda` name a bare geometry prim on `rigExec:cage` / `rigExec:surface` and depend on it.
- Do not touch `libs/rigExec/rigEvaluator.cpp` lines outside those named in each task; the file carries a large in-progress uncommitted change.

**Deviation from the spec, deliberate:** spec §5 lists "target binding" in phase 1, meaning the `_ConstraintTargetBinding` struct of §4.1. That struct exists to carry a *second* target kind, and phase 1 has only the transform domain — its Geometry arm would be dead on arrival. It therefore lands in phase 4 alongside the domain it carries. Phase 1 delivers the behavior change (the split plus the real predicate); phase 2's registry is what first needs a structured binding, and it can introduce one with both arms live.

---

### Task 1: Split the canonicalizer so write targets are never rewritten

**Files:**
- Modify: `libs/rigExec/rigEvaluator.cpp:246-260` (the function), `:727` (digest), `:1136` (mover records), `:2981`, `:3002`, `:3356` (read sites)
- Test: `tests/testRigExecConstraints.cpp`

**Interfaces:**
- Consumes: nothing from earlier tasks.
- Produces: `_ResolveGeometryInput(const UsdStageRefPtr &, const SdfPath &) -> SdfPath` — the read-side resolver, replacing `_CanonicalizeTarget`. No write-side replacement function exists; the call is simply removed at the two write sites.

- [x] **Step 1: Write the failing test**

Add to `tests/testRigExecConstraints.cpp`, immediately before `TestInvalidContractsFailClosed`:

```cpp
// A constraint revises a TRANSFORM. Any UsdGeomXformable can carry one --
// Mesh and BasisCurves included -- which is the same predicate
// bindFrameSource already applies to constraint sources. The write path must
// therefore not rewrite a PointBased prim target into its .points property.
static void
TestXformableTargetsCompile()
{
    for (const char *targetType : {"Mesh", "BasisCurves", "Points",
                                   "Xform", "Sphere"}) {
        const UsdStageRefPtr stage = UsdStage::CreateInMemory();
        MakeXform(stage, SdfPath("/Asset"), Matrix());
        stage->DefinePrim(SdfPath("/Asset/Geom"), TfToken("Scope"));
        const UsdPrim target = stage->DefinePrim(
            SdfPath("/Asset/Geom/Target"), TfToken(targetType));
        CHECK(target);
        MakeXform(stage, SdfPath("/Asset/Source"),
                  Matrix(GfVec3d(0, 0, 0),
                         GfRotation(GfVec3d(0, 1, 0), 90.0)));
        stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
        stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
        const UsdPrim rotation = MakeConstraint(
            stage, "Rot", "RigExecRotationConstraint",
            {SdfPath("/Asset/Geom/Target")});
        rotation.CreateRelationship(TfToken("rigExec:sources"))
            .SetTargets({SdfPath("/Asset/Source")});

        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        CHECK(evaluator.Compile(&errors));
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode::Default());
        CHECK(pose.valid);
        const auto revised =
            pose.providerXforms.find(SdfPath("/Asset/Geom/Target"));
        CHECK(revised != pose.providerXforms.end());
        if (revised != pose.providerXforms.end()) {
            // The source is a 90-degree Y rotation, so the revised x axis
            // must point down -z.
            CHECK(Near(revised->second.TransformDir(GfVec3d(1, 0, 0)),
                       GfVec3d(0, 0, -1)));
        }
        // A transform-domain constraint writes no points.
        CHECK(pose.movedProperties.empty());
    }
}
```

Register it in `main()` by adding the call after `TestModeAndFailureContracts();`:

```cpp
    TestXformableTargetsCompile();
```

- [x] **Step 2: Run the test to verify it fails**

```bash
cmake --build build 2>&1 | tail -3
PYTHONPATH=/Users/burkard/work/usd-install/lib/python3.11/site-packages \
  ctest --test-dir build -R testRigExecConstraints --output-on-failure
```

Expected: FAIL. The `Mesh`, `BasisCurves` and `Points` iterations fail `CHECK(evaluator.Compile(&errors))` because `_CanonicalizeTarget` rewrote the target to `.points` and the gate at `:1200` rejected the property path. `Xform` and `Sphere` pass — they are not `UsdGeomPointBased`.

- [x] **Step 3: Rename the function and delete the two write-side calls**

In `libs/rigExec/rigEvaluator.cpp`, replace the function at `:246-260` with:

```cpp
// Resolves a READ-side geometry input: naming a PointBased prim means its
// .points property, because a geometry input has exactly one thing to read.
// Property paths stay exact (spec §4.2).
//
// This rule is deliberately NOT applied to write targets. On the write side a
// bare prim path names the transform domain and <prim>.points names the
// geometry domain -- two different write sets on the same prim -- so inferring
// between them is what made a constraint unable to target a Mesh at all.
SdfPath
_ResolveGeometryInput(const UsdStageRefPtr &stage, const SdfPath &target)
{
    if (target.IsPrimPath()) {
        const UsdPrim prim = stage->GetPrimAtPath(target);
        if (prim && prim.IsA<UsdGeomPointBased>()) {
            return target.AppendProperty(TfToken("points"));
        }
    }
    return target;
}
```

At `:727` (the epoch digest), change:

```cpp
                const SdfPath canonical = _CanonicalizeTarget(_stage, t);
```

to:

```cpp
                const SdfPath canonical = t;
```

At `:1136` (mover records), change:

```cpp
                const SdfPath canonical = _CanonicalizeTarget(_stage, t);
```

to:

```cpp
                const SdfPath canonical = t;
```

At `:2981`, `:3002` and `:3356`, rename the call only — `_CanonicalizeTarget(` becomes `_ResolveGeometryInput(`, arguments unchanged.

- [x] **Step 4: Run the test to verify it passes**

```bash
cmake --build build 2>&1 | tail -3
PYTHONPATH=/Users/burkard/work/usd-install/lib/python3.11/site-packages \
  ctest --test-dir build -R testRigExecConstraints --output-on-failure
```

Expected: PASS for all five target types. If `Mesh` still fails, the gate in Task 2 has not yet been relaxed — that is expected only if Step 3 was skipped; the `IsPrimPath()` check at `:1200` is now satisfied because the target stayed a prim path.

- [x] **Step 5: Verify the read-side examples still evaluate**

```bash
export PXR_PLUGINPATH_NAME="$PWD/build/usd/rigExecSchema/resources:$PWD/build/usd/rigExecImaging/resources:$PWD/plugin/rigExecUsdview"
for f in examples/06_LatticeBulge.usda examples/07_SurfaceDrape.usda examples/13_ReadPhases.usda; do
  echo "== $f"; ./build/rigExecPose "$f" --frames 1 | grep -E "compile|FAIL"
done
```

Expected: `compile: ok` for all three. These name a bare geometry prim on `rigExec:cage` / `rigExec:surface` and prove the read-side rule survived.

- [x] **Step 6: Run the full suite**

```bash
PYTHONPATH=/Users/burkard/work/usd-install/lib/python3.11/site-packages \
  ctest --test-dir build --output-on-failure 2>&1 | tail -8
```

Expected: `100% tests passed, 0 tests failed out of 12`.

- [x] **Step 7: Commit**

```bash
git add libs/rigExec/rigEvaluator.cpp tests/testRigExecConstraints.cpp
git commit -m "rigExec: write targets are never rewritten

_CanonicalizeTarget conflated resolving a write target with resolving a read
geometry input. On the write side a bare prim path and <prim>.points are two
different domains on the same prim, so inferring between them made a
constraint unable to target a Mesh at all. The function is now
_ResolveGeometryInput and serves only the three read sites; the two write
sites take the authored path exactly as written."
```

---

### Task 2: Replace the `IsPrimPath()` proxy with the real transform-provider predicate

**Files:**
- Modify: `libs/rigExec/rigEvaluator.cpp:1198-1206`
- Test: `tests/testRigExecConstraints.cpp`

**Interfaces:**
- Consumes: Task 1's guarantee that `record.targets[0]` is the authored path.
- Produces: two new compile diagnostics, matched by substring in tests — `"is not a transform provider"` and `"geometry-domain constraint targets are not supported yet"`.

- [x] **Step 1: Write the failing test**

Add to `tests/testRigExecConstraints.cpp`, immediately after `TestXformableTargetsCompile`:

```cpp
// The gate must test what it means. A non-Xformable prim cannot carry a
// revised transform, and a .points target names the geometry domain, which
// phase 1 does not implement -- both are hard errors with distinct wording.
static void
TestTransformProviderPredicate()
{
    // A non-Xformable target is rejected as such.
    {
        const UsdStageRefPtr stage = UsdStage::CreateInMemory();
        MakeXform(stage, SdfPath("/Asset"), Matrix());
        stage->DefinePrim(SdfPath("/Asset/Geom"), TfToken("Scope"));
        stage->DefinePrim(SdfPath("/Asset/Geom/NotXformable"),
                          TfToken("Scope"));
        MakeXform(stage, SdfPath("/Asset/Source"), Matrix());
        stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
        stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
        const UsdPrim rotation = MakeConstraint(
            stage, "Rot", "RigExecRotationConstraint",
            {SdfPath("/Asset/Geom/NotXformable")});
        rotation.CreateRelationship(TfToken("rigExec:sources"))
            .SetTargets({SdfPath("/Asset/Source")});
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        CHECK(!evaluator.Compile(&errors));
        CHECK(std::any_of(errors.begin(), errors.end(),
                          [](const std::string &error) {
                              return error.find("is not a transform provider") !=
                                     std::string::npos;
                          }));
    }

    // A .points target is the geometry domain: recognised, and explicitly
    // deferred rather than misreported as a bad transform provider.
    {
        const UsdStageRefPtr stage = UsdStage::CreateInMemory();
        MakeXform(stage, SdfPath("/Asset"), Matrix());
        stage->DefinePrim(SdfPath("/Asset/Geom"), TfToken("Scope"));
        stage->DefinePrim(SdfPath("/Asset/Geom/M"), TfToken("Mesh"));
        MakeXform(stage, SdfPath("/Asset/Source"), Matrix());
        stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
        stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
        const UsdPrim rotation = MakeConstraint(
            stage, "Rot", "RigExecRotationConstraint",
            {SdfPath("/Asset/Geom/M").AppendProperty(TfToken("points"))});
        rotation.CreateRelationship(TfToken("rigExec:sources"))
            .SetTargets({SdfPath("/Asset/Source")});
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        CHECK(!evaluator.Compile(&errors));
        CHECK(std::any_of(
            errors.begin(), errors.end(), [](const std::string &error) {
                return error.find(
                           "geometry-domain constraint targets are not "
                           "supported yet") != std::string::npos;
            }));
    }
}
```

Register it in `main()` after `TestXformableTargetsCompile();`:

```cpp
    TestTransformProviderPredicate();
```

- [x] **Step 2: Run the test to verify it fails**

```bash
cmake --build build 2>&1 | tail -3
PYTHONPATH=/Users/burkard/work/usd-install/lib/python3.11/site-packages \
  ctest --test-dir build -R testRigExecConstraints --output-on-failure
```

Expected: FAIL on both substring assertions. Both cases currently emit `"must move exactly one transform-provider prim"`.

- [x] **Step 3: Implement the predicate**

In `libs/rigExec/rigEvaluator.cpp`, replace the block at `:1198-1206`:

```cpp
            if (_IsSourceFrameConstraintType(record.schemaType)) {
                if (record.targets.size() != 1 ||
                    !record.targets[0].IsPrimPath()) {
                    reportError(
                        record.schemaType.GetString() + " " +
                        prim.GetPath().GetString() +
                        " must move exactly one transform-provider prim");
                    return false;
                }
            }
```

with:

```cpp
            if (_IsSourceFrameConstraintType(record.schemaType)) {
                if (record.targets.size() != 1) {
                    reportError(
                        record.schemaType.GetString() + " " +
                        prim.GetPath().GetString() +
                        " must move exactly one transform-provider prim");
                    return false;
                }
                const SdfPath &constraintTarget = record.targets[0];
                // <prim>.points names the geometry domain. It is a legal
                // spelling the compiler must recognise, not a malformed
                // transform target; the implementation lands in phase 4.
                if (constraintTarget.IsPropertyPath() &&
                    constraintTarget.GetNameToken() == "points") {
                    reportError(
                        record.schemaType.GetString() + " " +
                        prim.GetPath().GetString() +
                        " targets " + constraintTarget.GetString() +
                        ": geometry-domain constraint targets are not "
                        "supported yet; name the prim " +
                        constraintTarget.GetPrimPath().GetString() +
                        " to revise its transform");
                    return false;
                }
                // A constraint revises a transform, so the target must be
                // able to carry one. This is the predicate bindFrameSource
                // already applies to sources, and it admits any
                // UsdGeomXformable -- Mesh and BasisCurves included.
                const UsdPrim targetPrim =
                    constraintTarget.IsPrimPath()
                        ? _stage->GetPrimAtPath(constraintTarget)
                        : UsdPrim();
                if (!targetPrim || !UsdGeomXformable(targetPrim)) {
                    reportError(
                        record.schemaType.GetString() + " " +
                        prim.GetPath().GetString() + " targets " +
                        constraintTarget.GetString() +
                        ", which is not a transform provider; a constraint "
                        "target must be a UsdGeomXformable");
                    return false;
                }
            }
```

- [x] **Step 4: Run the test to verify it passes**

```bash
cmake --build build 2>&1 | tail -3
PYTHONPATH=/Users/burkard/work/usd-install/lib/python3.11/site-packages \
  ctest --test-dir build -R testRigExecConstraints --output-on-failure
```

Expected: PASS.

- [x] **Step 5: Run the full suite**

```bash
PYTHONPATH=/Users/burkard/work/usd-install/lib/python3.11/site-packages \
  ctest --test-dir build --output-on-failure 2>&1 | tail -8
```

Expected: `100% tests passed, 0 tests failed out of 12`.

- [x] **Step 6: Commit**

```bash
git add libs/rigExec/rigEvaluator.cpp tests/testRigExecConstraints.cpp
git commit -m "rigExec: a constraint target is any UsdGeomXformable

The gate tested IsPrimPath() as a proxy for 'is a transform provider'. It now
tests the real predicate -- the one bindFrameSource already applies to
sources -- so a Mesh or BasisCurves is as legal a target as an Xform, which
is what examples/10_AimXformTurret.usda has documented all along. A .points
target is recognised as the geometry domain and deferred to phase 4 with its
own message rather than being misreported."
```

---

### Task 3: Point-domain movers name the fix when given a bare prim

**Files:**
- Modify: `libs/rigExec/rigEvaluator.cpp:1261-1291` (typed geometry movers), `:2901-2906` (`_ValidateMatrixMover`)
- Test: `tests/testRigExecConstraints.cpp`

**Interfaces:**
- Consumes: Task 1's guarantee that a bare prim path now reaches these validators unrewritten.
- Produces: the shared diagnostic substring `"Did you mean"`, emitted by both validator families.

- [x] **Step 1: Write the failing test**

Add to `tests/testRigExecConstraints.cpp`, immediately after `TestTransformProviderPredicate`:

```cpp
// With write-path inference gone, a point-domain mover handed a bare mesh
// prim is an error -- and it must carry the fix, because the old behaviour
// silently rewrote it and that silence is what this change removes. Both
// validator families must say so: the typed geometry movers and MatrixMover,
// which validates separately.
static void
TestPointDomainMoverNamesTheFix()
{
    for (const char *moverType : {"RigExecSmoothMover", "RigExecMatrixMover"}) {
        const UsdStageRefPtr stage = UsdStage::CreateInMemory();
        MakeXform(stage, SdfPath("/Asset"), Matrix());
        stage->DefinePrim(SdfPath("/Asset/Geom"), TfToken("Scope"));
        stage->DefinePrim(SdfPath("/Asset/Geom/M"), TfToken("Mesh"));
        stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
        stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
        const UsdPrim mover = stage->DefinePrim(
            SdfPath("/Asset/Rig/Movers/M"), TfToken(moverType));
        CHECK(mover.ApplyAPI(TfToken("RigExecMoverAPI")));
        mover.CreateRelationship(TfToken("rigExec:moves"))
            .SetTargets({SdfPath("/Asset/Geom/M")});

        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        CHECK(!evaluator.Compile(&errors));
        CHECK(std::any_of(errors.begin(), errors.end(),
                          [](const std::string &error) {
                              return error.find("Did you mean") !=
                                     std::string::npos &&
                                     error.find("/Asset/Geom/M.points") !=
                                     std::string::npos;
                          }));
    }
}
```

Register it in `main()` after `TestTransformProviderPredicate();`:

```cpp
    TestPointDomainMoverNamesTheFix();
```

- [x] **Step 2: Run the test to verify it fails**

```bash
cmake --build build 2>&1 | tail -3
PYTHONPATH=/Users/burkard/work/usd-install/lib/python3.11/site-packages \
  ctest --test-dir build -R testRigExecConstraints --output-on-failure
```

Expected: FAIL. Both movers reject the target, but neither message contains `"Did you mean"`.

- [x] **Step 3: Add the hint helper**

In `libs/rigExec/rigEvaluator.cpp`, immediately after the `_ResolveGeometryInput` function added in Task 1, add:

```cpp
// The write path no longer infers <prim> -> <prim>.points, so a point-domain
// mover handed a bare PointBased prim gets the spelling it needed. Empty for
// any other target, so callers can append unconditionally.
std::string
_PointsTargetHint(const UsdStageRefPtr &stage, const SdfPath &target)
{
    if (!target.IsPrimPath()) {
        return std::string();
    }
    const UsdPrim prim = stage->GetPrimAtPath(target);
    if (!prim || !prim.IsA<UsdGeomPointBased>()) {
        return std::string();
    }
    return ". Did you mean " +
           target.AppendProperty(TfToken("points")).GetString() + "?";
}
```

- [x] **Step 4: Use it in the typed geometry mover validator**

In the `reportError` call at `:1283-1289`, change:

```cpp
                            reportError(
                                record.schemaType.GetString() + " " +
                                prim.GetPath().GetString() +
                                " target " + t.GetString() +
                                " is not a native UsdGeomPointBased "
                                "point3f[] points attribute");
```

to:

```cpp
                            reportError(
                                record.schemaType.GetString() + " " +
                                prim.GetPath().GetString() +
                                " target " + t.GetString() +
                                " is not a native UsdGeomPointBased "
                                "point3f[] points attribute" +
                                _PointsTargetHint(_stage, t));
```

- [x] **Step 5: Use it in `_ValidateMatrixMover`**

In `_ValidateMatrixMover`, change the first error at `:2903-2906`:

```cpp
        *error = who + ": moves must resolve to exactly one native "
                       "PointBased points property";
```

to:

```cpp
        *error = who + ": moves must resolve to exactly one native "
                       "PointBased points property" +
                 (record.targets.size() == 1
                      ? _PointsTargetHint(_stage, record.targets[0])
                      : std::string());
```

- [x] **Step 6: Run the test to verify it passes**

```bash
cmake --build build 2>&1 | tail -3
PYTHONPATH=/Users/burkard/work/usd-install/lib/python3.11/site-packages \
  ctest --test-dir build -R testRigExecConstraints --output-on-failure
```

Expected: PASS for both mover types.

- [x] **Step 7: Run the full suite**

```bash
PYTHONPATH=/Users/burkard/work/usd-install/lib/python3.11/site-packages \
  ctest --test-dir build --output-on-failure 2>&1 | tail -8
```

Expected: `100% tests passed, 0 tests failed out of 12`.

- [x] **Step 8: Commit**

```bash
git add libs/rigExec/rigEvaluator.cpp tests/testRigExecConstraints.cpp
git commit -m "rigExec: a point-domain mover names the .points spelling it wanted

Removing the write-path inference turns a bare mesh prim on a point-domain
mover from a silent rewrite into an error. Both validator families -- the
seven typed geometry movers and MatrixMover, which validates separately --
now carry the spelling the author needed."
```

---

### Task 4: The failing examples become an acceptance test

**Files:**
- Modify: `examples/aimtest.usda`, `examples/rotateConstraint.usda` (documentation headers only)
- Test: `tests/testRigExecConstraints.cpp`

**Interfaces:**
- Consumes: Tasks 1–3.
- Produces: nothing later tasks rely on.

- [x] **Step 1: Verify the two transform-domain examples now compile and move**

```bash
export PXR_PLUGINPATH_NAME="$PWD/build/usd/rigExecSchema/resources:$PWD/build/usd/rigExecImaging/resources:$PWD/plugin/rigExecUsdview"
./build/rigExecPose examples/rotateConstraint.usda --frames 1,50,100 --targets --joints
./build/rigExecPose examples/aimtest.usda --frames 1 --targets --joints
```

Expected: `compile: ok` for both, and `rotateConstraint.usda` reports an `xform /World/Geom/Cylinder` line at each frame.

`aimtest_points.usda` is expected to still fail, now with the phase-4 deferral message from Task 2. Confirm the wording:

```bash
./build/rigExecPose examples/aimtest_points.usda --frames 1 2>&1 | head -4
```

Expected: `geometry-domain constraint targets are not supported yet`.

- [x] **Step 2: Write the end-to-end regression test**

Add to `tests/testRigExecConstraints.cpp`, immediately after `TestPointDomainMoverNamesTheFix`:

```cpp
// The transform a Mesh target receives must equal the one an Xform target
// receives from the same constraint. This is the property that made the
// original bug invisible to every existing test: they all used Xform targets.
static void
TestMeshAndXformTargetsAgree()
{
    GfMatrix4d meshResult(1.0);
    GfMatrix4d xformResult(1.0);
    for (const char *targetType : {"Mesh", "Xform"}) {
        const UsdStageRefPtr stage = UsdStage::CreateInMemory();
        MakeXform(stage, SdfPath("/Asset"), Matrix());
        stage->DefinePrim(SdfPath("/Asset/Geom"), TfToken("Scope"));
        stage->DefinePrim(SdfPath("/Asset/Geom/Target"), TfToken(targetType));
        MakeXform(stage, SdfPath("/Asset/Source"),
                  Matrix(GfVec3d(0, 0, 0),
                         GfRotation(GfVec3d(0, 1, 0), 37.5)));
        stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
        stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
        const UsdPrim rotation = MakeConstraint(
            stage, "Rot", "RigExecRotationConstraint",
            {SdfPath("/Asset/Geom/Target")});
        rotation.CreateRelationship(TfToken("rigExec:sources"))
            .SetTargets({SdfPath("/Asset/Source")});
        RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
        std::vector<std::string> errors;
        CHECK(evaluator.Compile(&errors));
        const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode::Default());
        const auto revised =
            pose.providerXforms.find(SdfPath("/Asset/Geom/Target"));
        CHECK(revised != pose.providerXforms.end());
        if (revised != pose.providerXforms.end()) {
            (std::string(targetType) == "Mesh" ? meshResult : xformResult) =
                revised->second;
        }
    }
    CHECK(GfIsClose(meshResult, xformResult, 1e-12));
}
```

Register it in `main()` after `TestPointDomainMoverNamesTheFix();`:

```cpp
    TestMeshAndXformTargetsAgree();
```

- [x] **Step 3: Run the test**

```bash
cmake --build build 2>&1 | tail -3
PYTHONPATH=/Users/burkard/work/usd-install/lib/python3.11/site-packages \
  ctest --test-dir build -R testRigExecConstraints --output-on-failure
```

Expected: PASS.

- [x] **Step 4: Record the phase-4 status in the deferred example**

Add to the `doc` metadata of `examples/aimtest_points.usda`, replacing the existing `doc = """..."""` opening string with:

```
    doc = """Geometry-domain constraint target: rigExec:moves names
    <prim>.points, so the constraint deforms the point set rather than
    revising the prim transform. Phase 4 of the constraint target
    unification implements this; until then the compiler reports it as
    an explicit deferral rather than a malformed target.
    """
```

- [x] **Step 5: Run the full suite**

```bash
PYTHONPATH=/Users/burkard/work/usd-install/lib/python3.11/site-packages \
  ctest --test-dir build --output-on-failure 2>&1 | tail -8
```

Expected: `100% tests passed, 0 tests failed out of 12`.

- [x] **Step 6: Commit**

```bash
git add tests/testRigExecConstraints.cpp examples/aimtest_points.usda
git commit -m "rigExec: a Mesh target and an Xform target agree exactly

The regression the constraint suite could not catch: every target in it was
an Xform or a RigExecJoint, so nothing exercised the PointBased path that
the write-side inference rewrote. aimtest_points.usda records its
geometry-domain status pending phase 4."
```

---

## Phase 1 exit criteria

- `examples/rotateConstraint.usda` and `examples/aimtest.usda` compile and drive their Mesh targets.
- `examples/aimtest_points.usda` fails with the geometry-domain deferral message.
- `examples/06_LatticeBulge.usda`, `07_SurfaceDrape.usda`, `13_ReadPhases.usda` still compile.
- All 12 ctest suites green.
- `libs/rigExecSchema/schema.usda` and both generated schema files are **unmodified**.
