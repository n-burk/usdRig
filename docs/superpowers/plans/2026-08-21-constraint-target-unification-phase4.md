# Constraint Target Unification — Phase 4 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let a source-frame constraint write the geometry domain — `rigExec:moves = </Geom/M.points>` — by applying the same delta it would have published as a transform, per point, under the constraint's weight field.

**Architecture:** The constraint solves in the existing frame pass, at **weight 1**, against the target prim's own asset-relative frame (`frameFromXform`). The resulting delta `D = F_solved * F_base^-1` is stashed in an in-memory map, exactly as `finalMatrices` already carries a revised matrix from the frame pass (`rigEvaluator.cpp:5696`) to the geometry pass (`:5905`). The geometry pass then runs the target's points through a `RigExecRevisionOp::Matrix` revision whose transform is that delta and whose weight packet is either the bound `rigExec:weightObject` or a constant synthesized from `inputs:defaultWeight`. No new math: `RigExecApplyWeightedMatrix` (`solvers.cpp:367-378`) is already `lerp(P, D.TransformAffine(P), w)`.

**Tech Stack:** C++17, OpenUSD, CMake + Ninja, `CHECK` macro, ctest.

**Spec:** `docs/superpowers/specs/2026-08-21-constraint-target-unification-design.md` (sections 4.4, 4.5, 6)

## Global Constraints

- Branch `constraint-target-unification`, continuing from Phase 3 (`8c032c7`).
- All 12 ctest suites green at every commit.
- The 18 currently-compiling examples must stay byte-identical; compare against the captured baseline. Only `aimtest_points.usda` changes — from the phase-1 deferral error to a working geometry-domain constraint.
- **The envelope is applied exactly once.** The kernels fold `params.weight` into their result (`solvers.cpp:724-730`), so the geometry arm MUST call the solve with `weight = 1.0` and let the per-point lerp carry the envelope. Applying both squares it: `defaultWeight = 0.5` would give 0.5 on a transform and 0.25 on points.
- **The parity twin is mandatory.** The oracle at `:5736-5800` independently re-derives every points chain and refuses to publish the generation on disagreement (`:5820`). A constraint revision must be mirrored there, not skipped.

---

### Task 1: Bind a `.points` constraint target instead of rejecting it

**Files:**
- Modify: `libs/rigExec/rigEvaluator.cpp` — the phase-1 deferral in the source-frame gate; `_FrameConstraint` in `rigEvaluator.h`
- Test: `tests/testRigExecConstraints.cpp`

**Interfaces:**
- Produces: `_FrameConstraint::pointsTarget` (empty for a transform-domain constraint) and `_FrameConstraint::weightObject`.

- [x] **Step 1: Write the failing test**

Add to `tests/testRigExecConstraints.cpp` before `TestInvalidContractsFailClosed`:

```cpp
// A .points target names the geometry domain and must compile.
static void
TestGeometryDomainTargetCompiles()
{
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    MakeXform(stage, SdfPath("/Asset"), Matrix());
    stage->DefinePrim(SdfPath("/Asset/Geom"), TfToken("Scope"));
    const UsdPrim mesh =
        stage->DefinePrim(SdfPath("/Asset/Geom/M"), TfToken("Mesh"));
    mesh.CreateAttribute(TfToken("points"), SdfValueTypeNames->Point3fArray)
        .Set(VtVec3fArray{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}});
    MakeXform(stage, SdfPath("/Asset/Source"),
              Matrix(GfVec3d(0, 0, 0), GfRotation(GfVec3d(0, 1, 0), 90.0)));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const UsdPrim rotation = MakeConstraint(
        stage, "Rot", "RigExecRotationConstraint",
        {SdfPath("/Asset/Geom/M").AppendProperty(TfToken("points"))});
    rotation.CreateRelationship(TfToken("rigExec:sources"))
        .SetTargets({SdfPath("/Asset/Source")});

    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
    const RigExecRigPose pose = evaluator.Evaluate(UsdTimeCode::Default());
    CHECK(pose.valid);
    // It writes points, not a transform.
    CHECK(pose.movedProperties.count(
              SdfPath("/Asset/Geom/M").AppendProperty(TfToken("points"))) == 1);
    CHECK(pose.providerXforms.count(SdfPath("/Asset/Geom/M")) == 0);
}
```

Register it in `main()` after `TestMeshAndXformTargetsAgree();`.

- [x] **Step 2: Run to verify it fails**

```bash
cmake --build build 2>&1 | grep -E "error" | head -3
PYTHONPATH=/Users/burkard/work/usd-install/lib/python3.11/site-packages \
  ctest --test-dir build -R testRigExecConstraints --output-on-failure
```

Expected: FAIL on `CHECK(evaluator.Compile(&errors))` with the phase-1 message *"geometry-domain constraint targets are not supported yet"*.

- [x] **Step 3: Carry the domain on the constraint record**

In `libs/rigExec/rigEvaluator.h`, add to `_FrameConstraint`:

```cpp
        /// Non-empty when the constraint writes the GEOMETRY domain: the
        /// <prim>.points property it revises. targets[0] stays the owning
        /// prim path, because every frame-domain map is keyed by prim.
        SdfPath pointsTarget;
        /// Optional per-element weight field. Absent means the constant
        /// packet synthesized from inputs:defaultWeight.
        SdfPath weightObject;
```

- [x] **Step 4: Replace the deferral with a binding**

In `libs/rigExec/rigEvaluator.cpp`, in the source-frame gate, replace the `geometry-domain constraint targets are not supported yet` error block with an acceptance that keeps the prim path as the frame key:

```cpp
                if (constraintTarget.IsPropertyPath() &&
                    constraintTarget.GetNameToken() == "points") {
                    const UsdPrim owner =
                        _stage->GetPrimAtPath(constraintTarget.GetPrimPath());
                    if (!owner || !owner.IsA<UsdGeomPointBased>()) {
                        reportError(
                            record.schemaType.GetString() + " " +
                            prim.GetPath().GetString() + " targets " +
                            constraintTarget.GetString() +
                            ", whose owner is not a UsdGeomPointBased prim");
                        return false;
                    }
                    // Legal. The domain is carried in constraint compilation
                    // below; the record keeps the authored path.
                }
```

Then, where `_FrameConstraint` is built (`constraint.targets = mover.targets;`), set the domain fields and normalize the frame key to the prim:

```cpp
            constraint.targets = mover.targets;
            if (!constraint.targets.empty() &&
                constraint.targets[0].IsPropertyPath() &&
                constraint.targets[0].GetNameToken() == "points") {
                constraint.pointsTarget = constraint.targets[0];
                // Every frame-domain map -- newFrameChains, base/rest frames,
                // the provider classifier -- is keyed by PRIM. The geometry
                // domain differs only in where the answer is published.
                constraint.targets[0] = constraint.targets[0].GetPrimPath();
                SdfPathVector weightTargets;
                if (const UsdRelationship rel = moverPrim.GetRelationship(
                        TfToken("rigExec:weightObject"))) {
                    rel.GetTargets(&weightTargets);
                }
                if (weightTargets.size() > 1) {
                    reportError(constraint.schemaType.GetString() + " " +
                                mover.moverPath.GetString() +
                                " binds more than one rigExec:weightObject");
                    restorePreviousEpoch();
                    return false;
                }
                if (!weightTargets.empty()) {
                    constraint.weightObject = weightTargets[0];
                }
            } else if (moverPrim.GetRelationship(
                           TfToken("rigExec:weightObject"))
                           .HasAuthoredTargets()) {
                reportError(
                    constraint.schemaType.GetString() + " " +
                    mover.moverPath.GetString() +
                    " binds rigExec:weightObject on a transform-domain "
                    "constraint; a transform has one element and nothing to "
                    "vary over. Use inputs:defaultWeight");
                restorePreviousEpoch();
                return false;
            }
```

- [x] **Step 5: Run to verify it passes**

```bash
cmake --build build 2>&1 | grep -E "error" | head -3
PYTHONPATH=/Users/burkard/work/usd-install/lib/python3.11/site-packages \
  ctest --test-dir build -R testRigExecConstraints --output-on-failure
```

Expected: the compile assertion passes. The `movedProperties` assertion still FAILS — nothing publishes points yet. That is Task 2.

- [x] **Step 6: Commit the binding only**

Comment out the two publish assertions with a `// Task 2:` marker so the suite is green, then:

```bash
git add libs/rigExec/rigEvaluator.cpp libs/rigExec/rigEvaluator.h tests/testRigExecConstraints.cpp
git commit -m "rigExec: a .points constraint target binds to the geometry domain"
```

---

### Task 2: Publish the delta per point

**Files:**
- Modify: `libs/rigExec/rigEvaluator.cpp` — the constraint solve loop and the publish step
- Test: `tests/testRigExecConstraints.cpp`

**Interfaces:**
- Consumes: `_FrameConstraint::pointsTarget`, `::weightObject`.
- Produces: entries in `pose.movedProperties` keyed by the points property.

- [x] **Step 1: Solve at weight 1 for the geometry domain**

In the constraint solve loop, the envelope must not reach the kernel for a geometry-domain constraint. Where `solveContext.weight` is set:

```cpp
            // The envelope is applied ONCE. In the transform domain the
            // kernel's per-channel blend carries it; in the geometry domain
            // the per-point lerp does, so the solve runs unweighted and the
            // delta it produces is the full-strength one.
            solveContext.weight =
                constraint.pointsTarget.IsEmpty() ? weight : 1.0;
```

Apply the same substitution to the inline Aim branch's `params.weight`.

- [x] **Step 2: Stash the delta**

Declare next to `finalMatrices` (`:4863`):

```cpp
    /// Geometry-domain constraint deltas, produced by the frame pass and
    /// consumed by the point publish below. Same in-memory hand-off
    /// finalMatrices already performs for a "final" read phase.
    std::map<SdfPath, GfMatrix4d> constraintDeltas;
```

After `commitConstraintFrames` for a geometry-domain constraint, compute and store:

```cpp
                if (!constraint.pointsTarget.IsEmpty()) {
                    GfMatrix4d baseMatrix(1.0);
                    GfMatrix4d solvedMatrix(1.0);
                    if (frameFromXform(constraint.targets[0], nullptr,
                                       &baseMatrix) &&
                        RigExecPointsToMatrix(RigExecIdentityLandmarks(),
                                              candidate.points,
                                              &solvedMatrix)) {
                        constraintDeltas[constraint.pointsTarget] =
                            baseMatrix.GetInverse() * solvedMatrix;
                    }
                }
```

- [x] **Step 3: Apply it per point**

After the geometry chains run, for each entry in `constraintDeltas`, read the target's current points, apply the weighted matrix, and write `pose.movedProperties`:

```cpp
    for (const auto &[pointsTarget, delta] : constraintDeltas) {
        std::vector<GfVec3f> points;
        if (!_ReadTargetPoints(_stage->GetPrimAtPath(pointsTarget.GetPrimPath()),
                               "rigExec:moves", time, &points)) {
            const UsdAttribute attr = _stage->GetAttributeAtPath(pointsTarget);
            VtVec3fArray authored;
            if (!attr || !attr.Get(&authored, time)) {
                continue;
            }
            points.assign(authored.begin(), authored.end());
        }
        // The weight field: a bound object, else the constant envelope.
        RigExecWeightPacket packet;
        packet.representation = TfToken("constant");
        packet.rangePolicy = TfToken("strict");
        packet.defaultWeight = static_cast<float>(envelopeFor[pointsTarget]);
        packet.valid = true;
        VtVec3fArray moved(points.size());
        for (size_t i = 0; i < points.size(); ++i) {
            const float w = packet.Resolve(i, points.size());
            if (w < 0.0f) {
                moved = VtVec3fArray();  // cardinality mismatch: MoverFailed
                break;
            }
            const GfVec3d p(points[i]);
            moved[i] = GfVec3f(RigExecApplyWeightedMatrix(p, delta, w));
        }
        if (!moved.empty()) {
            pose.movedProperties[pointsTarget] = moved;
        }
    }
```

`envelopeFor` is a `std::map<SdfPath, double>` filled alongside `constraintDeltas` with the constraint's `inputs:defaultWeight`.

- [x] **Step 4: Restore the assertions and run**

Un-comment the Task 1 publish assertions.

```bash
cmake --build build 2>&1 | grep -E "error" | head -3
PYTHONPATH=/Users/burkard/work/usd-install/lib/python3.11/site-packages \
  ctest --test-dir build --output-on-failure 2>&1 | tail -6
```

Expected: `100% tests passed, 0 tests failed out of 12`.

- [x] **Step 5: Commit**

```bash
git add libs/rigExec/rigEvaluator.cpp tests/testRigExecConstraints.cpp
git commit -m "rigExec: a geometry-domain constraint bakes its delta per point"
```

---

### Task 3: The invariant

**Files:**
- Test: `tests/testRigExecConstraints.cpp`

- [x] **Step 1: Write the invariant test**

```cpp
// The design's defining property: the spelling picks WHERE the answer lands,
// not what it is. Asserted at defaultWeight 1 AND 0.5 -- 1 is the one value
// where a double-applied envelope would be invisible.
static void
TestTransformAndGeometrySpellingsAgree()
{
    for (const float envelope : {1.0f, 0.5f}) {
        std::vector<GfVec3f> viaTransform;
        std::vector<GfVec3f> viaPoints;
        for (const bool geometry : {false, true}) {
            const UsdStageRefPtr stage = UsdStage::CreateInMemory();
            MakeXform(stage, SdfPath("/Asset"), Matrix());
            stage->DefinePrim(SdfPath("/Asset/Geom"), TfToken("Scope"));
            const UsdPrim mesh = stage->DefinePrim(
                SdfPath("/Asset/Geom/M"), TfToken("Mesh"));
            const VtVec3fArray rest{{1, 0, 0}, {0, 2, 0}, {0, 0, 3}};
            mesh.CreateAttribute(TfToken("points"),
                                 SdfValueTypeNames->Point3fArray).Set(rest);
            MakeXform(stage, SdfPath("/Asset/Source"),
                      Matrix(GfVec3d(0, 0, 0),
                             GfRotation(GfVec3d(0, 1, 0), 90.0)));
            stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
            stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
            const SdfPath target =
                geometry ? SdfPath("/Asset/Geom/M")
                               .AppendProperty(TfToken("points"))
                         : SdfPath("/Asset/Geom/M");
            const UsdPrim rot = MakeConstraint(
                stage, "Rot", "RigExecRotationConstraint", {target});
            rot.CreateRelationship(TfToken("rigExec:sources"))
                .SetTargets({SdfPath("/Asset/Source")});
            rot.GetAttribute(TfToken("inputs:defaultWeight")).Set(envelope);

            RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
            std::vector<std::string> errors;
            CHECK(evaluator.Compile(&errors));
            const RigExecRigPose pose =
                evaluator.Evaluate(UsdTimeCode::Default());
            CHECK(pose.valid);
            if (geometry) {
                const auto it = pose.movedProperties.find(target);
                CHECK(it != pose.movedProperties.end());
                if (it != pose.movedProperties.end()) {
                    viaPoints.assign(it->second.begin(), it->second.end());
                }
            } else {
                const auto it =
                    pose.providerXforms.find(SdfPath("/Asset/Geom/M"));
                CHECK(it != pose.providerXforms.end());
                if (it != pose.providerXforms.end()) {
                    for (const GfVec3f &p : rest) {
                        viaTransform.push_back(GfVec3f(
                            it->second.TransformAffine(GfVec3d(p))));
                    }
                }
            }
        }
        CHECK(viaTransform.size() == viaPoints.size());
        for (size_t i = 0; i < viaTransform.size() &&
                           i < viaPoints.size(); ++i) {
            CHECK(Near(GfVec3d(viaTransform[i]), GfVec3d(viaPoints[i]), 1e-4));
        }
    }
}
```

Register it in `main()`.

- [x] **Step 2: Run**

```bash
PYTHONPATH=/Users/burkard/work/usd-install/lib/python3.11/site-packages \
  ctest --test-dir build -R testRigExecConstraints --output-on-failure
```

Expected: PASS at both envelopes. A failure at 0.5 with a pass at 1.0 means the envelope is applied twice — recheck Task 2 Step 1.

- [x] **Step 3: Verify the deferred example now works**

```bash
export PXR_PLUGINPATH_NAME="$PWD/build/usd/rigExecSchema/resources:$PWD/build/usd/rigExecImaging/resources:$PWD/plugin/rigExecUsdview"
./build/rigExecPose examples/aimtest_points.usda --frames 1 --targets --joints
```

Expected: `compile: ok`, and a moved-properties count of 1.

- [x] **Step 4: Commit**

```bash
git add tests/testRigExecConstraints.cpp
git commit -m "rigExec: the two target spellings agree, weighted and unweighted"
```

---

### Task 4: The parity twin and competing writers

**Files:**
- Modify: `libs/rigExec/rigEvaluator.cpp:5736-5800` (oracle), `:1557-1559` and `:1615` (writer keying)

- [x] **Step 1: Mirror the delta in the CPU oracle**

The oracle re-derives every points chain independently. Add the constraint delta to its derivation so a geometry-domain constraint is checked rather than skipped — the same `RigExecApplyWeightedMatrix` over the same `constraintDeltas` entry, computed from the oracle's own frame values.

- [x] **Step 2: Key competing writers by prim**

`byTarget` currently keys on the raw path, so `/M` and `/M.points` never meet. Key on `t.GetPrimPath()` instead so a constraint on `/M` and a deformer on `/M.points` are ordered rather than invisible to each other. Carry the authored path per writer for the diagnostic text.

- [x] **Step 3: Test both**

```cpp
// A transform-domain constraint on /M and a deformer on /M.points are both
// legal, but they must be ORDERED, not invisible to each other.
```

Assert the pair produces an ordering diagnostic rather than compiling silently, and that the parity oracle agrees on a geometry-domain constraint (no "mover graph parity" mismatch in `pose.diagnostics`).

- [x] **Step 4: Full suite plus the example baseline**

```bash
PYTHONPATH=/Users/burkard/work/usd-install/lib/python3.11/site-packages \
  ctest --test-dir build --output-on-failure 2>&1 | tail -6
```

Compare all 19 examples against the captured baseline; only `aimtest_points.usda` may differ.

- [x] **Step 5: Commit**

---

## Phase 4 exit criteria

- `aimtest_points.usda` compiles and publishes moved points.
- `</Geom/M>` and `</Geom/M.points>` agree at `defaultWeight` 1 and 0.5.
- The parity oracle checks geometry-domain constraints rather than skipping them.
- A constraint and a deformer on the same prim are ordered.
- All 12 ctest suites green; the other 18 examples byte-identical.
