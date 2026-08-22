# Constraint Target Unification — Phase 2 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the constraint family's hardcoded type-name string dispatch with one handler registry, so operator capabilities are declared in a table rather than rediscovered at each of ~27 sites.

**Architecture:** A `RigExecConstraintHandler` table holds one row per operator: its schema type, whether it is a source-frame constraint, whether it honors `rigExec:rotationOrder`, and a `solve` function pointer. `_IsSourceFrameConstraintType` and `_IsFrameConstraintType` become registry lookups. The four kernel-backed operators (Position, Rotation, Scale, Parent) move their solve into free functions taking a uniform `RigExecConstraintSolveContext`. Aim and SingleChainIk keep their inline branches — they consume evaluator state (world-up binding resolution, inferred chain frames) that a uniform context cannot carry — and are registered with `solve == nullptr` plus an explicit `dispatchesInline` flag, so the table stays the single source of truth about which operators exist.

**Tech Stack:** C++17, OpenUSD, CMake + Ninja, `CHECK` macro, ctest.

**Spec:** `docs/superpowers/specs/2026-08-21-constraint-target-unification-design.md` (section 4.2)

## Global Constraints

- **Phase 2 is behavior-neutral.** No schema change, no regen, no reconfigure. Every existing test must pass unchanged at every commit, and the 18 compiling examples must keep compiling with identical digests where they are reported.
- Build: `cmake --build build`
- Test: `PYTHONPATH=/Users/burkard/work/usd-install/lib/python3.11/site-packages ctest --test-dir build --output-on-failure`
- Branch: `constraint-target-unification`, continuing from Phase 1.
- The `rigExec:rotationOrder`-on-Position defect (spec 3.4) is **not** fixed here. Phase 2 records the capability in the table; Phase 3 turns it into a compile error. Recording it without acting on it keeps this phase behavior-neutral.

---

### Task 1: The registry table replaces the membership predicates

**Files:**
- Modify: `libs/rigExec/rigEvaluator.cpp:110-127` (the two predicates)
- Test: `tests/testRigExecConstraints.cpp`

**Interfaces:**
- Produces:
  - `struct RigExecConstraintHandler { TfToken schemaType; bool sourceFrame; bool usesRotationOrder; bool dispatchesInline; RigExecConstraintSolveFn solve; }`
  - `const RigExecConstraintHandler *_FindConstraintHandler(const TfToken &)` — null when the type is not a constraint.
  - `_IsSourceFrameConstraintType` / `_IsFrameConstraintType` keep their existing signatures and semantics, now backed by the table.

- [x] **Step 1: Write the failing test**

Add to `tests/testRigExecConstraints.cpp`, immediately before `TestInvalidContractsFailClosed`:

```cpp
// The registry is the single source of truth about which operators exist.
// Every concrete constraint in the schema must have exactly one row, and the
// membership predicates must agree with it -- otherwise a seventh operator
// can be added to one and forgotten in the other, which is the failure mode
// the table exists to remove.
static void
TestConstraintRegistryCoversTheSchema()
{
    const UsdSchemaRegistry &registry = UsdSchemaRegistry::GetInstance();
    for (const char *typeName :
         {"RigExecAimConstraint", "RigExecPositionConstraint",
          "RigExecRotationConstraint", "RigExecScaleConstraint",
          "RigExecParentConstraint", "RigExecSingleChainIkConstraint",
          "RigExecCustomConstraint"}) {
        CHECK(registry.FindConcretePrimDefinition(TfToken(typeName)));
        CHECK(RigExecConstraintHandlerCount(TfToken(typeName)) == 1);
    }
    // Seven rows, no more: an unregistered type must not resolve.
    CHECK(RigExecConstraintHandlerCount(TfToken("RigExecSmoothMover")) == 0);
    CHECK(RigExecConstraintHandlerTotal() == 7);
}
```

Add the two test-visible accessors to `libs/rigExec/rigEvaluator.h`, in the `rigExec` namespace next to the other free declarations:

```cpp
/// Test/diagnostic access to the constraint handler registry: how many rows
/// match \p schemaType (0 or 1), and how many rows exist in total.
size_t RigExecConstraintHandlerCount(const TfToken &schemaType);
size_t RigExecConstraintHandlerTotal();
```

Register the test in `main()` after `TestMeshAndXformTargetsAgree();`:

```cpp
    TestConstraintRegistryCoversTheSchema();
```

- [x] **Step 2: Run the test to verify it fails**

```bash
cmake --build build 2>&1 | grep -E "error" | head -5
```

Expected: FAIL to compile — `RigExecConstraintHandlerCount` is declared but not defined.

- [x] **Step 3: Define the solve-context type and the table**

In `libs/rigExec/rigEvaluator.cpp`, replace the two predicate bodies at `:110-127`:

```cpp
/// Source-blending constraints that revise one transform provider.
bool
_IsSourceFrameConstraintType(const TfToken &typeName)
{
    return typeName == "RigExecAimConstraint" ||
           typeName == "RigExecPositionConstraint" ||
           typeName == "RigExecRotationConstraint" ||
           typeName == "RigExecScaleConstraint" ||
           typeName == "RigExecParentConstraint";
}

/// Every constraint the evaluator compiles to frame wiring.
bool
_IsFrameConstraintType(const TfToken &typeName)
{
    return _IsSourceFrameConstraintType(typeName) ||
           typeName == "RigExecSingleChainIkConstraint";
}
```

with:

```cpp
/// Everything a constraint solve needs that is common to every operator. The
/// per-operator reads (offsets, masks) happen inside the solve, because that
/// is exactly what differs between operators.
struct _ConstraintSolveContext {
    const RigExecResolvedInputs *resolved = nullptr;
    UsdPrim prim;
    UsdTimeCode time;
    RigExecPointFrame inputFrame;
    const std::vector<RigExecConstraintSource> *sources = nullptr;
    RigExecConstraintAxisMask affect;
    RigExecEulerOrder order = RigExecEulerOrder::XYZ;
    double weight = 1.0;
};

using _ConstraintSolveFn =
    RigExecPointFrame (*)(const _ConstraintSolveContext &);

// Forward declarations; the bodies sit next to the dispatch they replaced.
RigExecPointFrame _SolvePositionConstraint(const _ConstraintSolveContext &);
RigExecPointFrame _SolveRotationConstraint(const _ConstraintSolveContext &);
RigExecPointFrame _SolveScaleConstraint(const _ConstraintSolveContext &);
RigExecPointFrame _SolveParentConstraint(const _ConstraintSolveContext &);

/// One row per constraint operator. This table is the single source of truth
/// about which operators exist and what each one honors; adding an operator
/// is a row here plus its solve, not an edit at every dispatch site.
///
/// solve == nullptr with dispatchesInline == true means the operator is
/// evaluated by a bespoke branch in Evaluate() because it consumes evaluator
/// state a uniform context cannot carry -- Aim resolves a world-up binding,
/// SingleChainIk writes an inferred joint chain atomically.
/// solve == nullptr with dispatchesInline == false means the operator has no
/// evaluator at all: a legal carrier prim that must not silently do nothing.
struct _ConstraintHandler {
    const char *schemaType;
    bool sourceFrame;
    bool usesRotationOrder;
    bool dispatchesInline;
    _ConstraintSolveFn solve;
};

const std::vector<_ConstraintHandler> &
_ConstraintHandlers()
{
    static const std::vector<_ConstraintHandler> handlers = {
        {"RigExecAimConstraint",           true,  true,  true,  nullptr},
        {"RigExecPositionConstraint",      true,  false, false,
         _SolvePositionConstraint},
        {"RigExecRotationConstraint",      true,  true,  false,
         _SolveRotationConstraint},
        {"RigExecScaleConstraint",         true,  false, false,
         _SolveScaleConstraint},
        {"RigExecParentConstraint",        true,  true,  false,
         _SolveParentConstraint},
        {"RigExecSingleChainIkConstraint", false, false, true,  nullptr},
        {"RigExecCustomConstraint",        false, false, false, nullptr},
    };
    return handlers;
}

const _ConstraintHandler *
_FindConstraintHandler(const TfToken &typeName)
{
    for (const _ConstraintHandler &handler : _ConstraintHandlers()) {
        if (typeName == handler.schemaType) {
            return &handler;
        }
    }
    return nullptr;
}

/// Source-blending constraints that revise one transform provider.
bool
_IsSourceFrameConstraintType(const TfToken &typeName)
{
    const _ConstraintHandler *handler = _FindConstraintHandler(typeName);
    return handler && handler->sourceFrame;
}

/// Every constraint the evaluator compiles to frame wiring.
bool
_IsFrameConstraintType(const TfToken &typeName)
{
    const _ConstraintHandler *handler = _FindConstraintHandler(typeName);
    return handler &&
           (handler->sourceFrame ||
            TfToken(handler->schemaType) == "RigExecSingleChainIkConstraint");
}
```

- [x] **Step 4: Define the two test accessors**

At the end of `libs/rigExec/rigEvaluator.cpp`, outside the anonymous namespace but inside the `rigExec` namespace, add:

```cpp
size_t
RigExecConstraintHandlerCount(const TfToken &schemaType)
{
    return _FindConstraintHandler(schemaType) ? 1 : 0;
}

size_t
RigExecConstraintHandlerTotal()
{
    return _ConstraintHandlers().size();
}
```

Add `#include "pxr/usd/usd/schemaRegistry.h"` to `tests/testRigExecConstraints.cpp` only if it is not already present (it is — line 12).

- [x] **Step 5: Run the test to verify it passes**

```bash
cmake --build build 2>&1 | grep -E "error" | head -5
PYTHONPATH=/Users/burkard/work/usd-install/lib/python3.11/site-packages \
  ctest --test-dir build -R testRigExecConstraints --output-on-failure
```

Expected: PASS.

- [x] **Step 6: Run the full suite — behavior must be unchanged**

```bash
PYTHONPATH=/Users/burkard/work/usd-install/lib/python3.11/site-packages \
  ctest --test-dir build --output-on-failure 2>&1 | tail -6
```

Expected: `100% tests passed, 0 tests failed out of 12`.

- [x] **Step 7: Commit**

```bash
git add libs/rigExec/rigEvaluator.cpp libs/rigExec/rigEvaluator.h tests/testRigExecConstraints.cpp
git commit -m "rigExec: one registry row per constraint operator"
```

---

### Task 2: The four kernel-backed operators solve through the table

**Files:**
- Modify: `libs/rigExec/rigEvaluator.cpp:5192-5240` (the if/else chain)
- Test: covered by the existing suite — this task is behavior-neutral by construction.

**Interfaces:**
- Consumes: `_ConstraintSolveContext`, `_ConstraintSolveFn`, `_FindConstraintHandler` from Task 1.
- Produces: four free functions matching `_ConstraintSolveFn`.

- [x] **Step 1: Write the solve functions**

In `libs/rigExec/rigEvaluator.cpp`, immediately after `_FindConstraintHandler`, add the four bodies. Each is the corresponding branch of the existing chain with `constraint`/`prim`/`time` replaced by context members:

```cpp
RigExecPointFrame
_SolvePositionConstraint(const _ConstraintSolveContext &c)
{
    RigExecPositionConstraintParams params;
    params.offset = _ResolvedRead(
        *c.resolved, c.prim, "inputs:translationOffset", GfVec3d(0), c.time);
    params.affect = c.affect;
    params.weight = c.weight;
    return RigExecApplyPositionConstraint(c.inputFrame, *c.sources, params);
}

RigExecPointFrame
_SolveRotationConstraint(const _ConstraintSolveContext &c)
{
    RigExecRotationConstraintParams params;
    params.offsetDegrees = _ResolvedRead(
        *c.resolved, c.prim, "inputs:rotationOffset", GfVec3d(0), c.time);
    params.affect = c.affect;
    params.rotationOrder = c.order;
    params.weight = c.weight;
    return RigExecApplyRotationConstraint(c.inputFrame, *c.sources, params);
}

RigExecPointFrame
_SolveScaleConstraint(const _ConstraintSolveContext &c)
{
    RigExecScaleConstraintParams params;
    params.offset = _ResolvedRead(
        *c.resolved, c.prim, "inputs:scaleOffset", GfVec3d(0), c.time);
    params.affect = c.affect;
    params.weight = c.weight;
    return RigExecApplyScaleConstraint(c.inputFrame, *c.sources, params);
}

RigExecPointFrame
_SolveParentConstraint(const _ConstraintSolveContext &c)
{
    RigExecParentConstraintParams params;
    params.translationAxes = _ReadConstraintAxisMask(
        *c.resolved, c.prim, "inputs:affectTranslationX",
        "inputs:affectTranslationY", "inputs:affectTranslationZ", c.time);
    params.rotationAxes = _ReadConstraintAxisMask(
        *c.resolved, c.prim, "inputs:affectRotationX",
        "inputs:affectRotationY", "inputs:affectRotationZ", c.time);
    // FBX disables scale by default; the explicit false fallback is the
    // contract, not an oversight (schema.usda:769-771).
    params.scaleAxes = _ReadConstraintAxisMask(
        *c.resolved, c.prim, "inputs:affectScaleX", "inputs:affectScaleY",
        "inputs:affectScaleZ", c.time, false);
    params.rotationOrder = c.order;
    params.weight = c.weight;
    return RigExecApplyParentConstraint(c.inputFrame, *c.sources, params);
}
```

- [x] **Step 2: Replace the dispatch chain with a table lookup**

Replace the four `if`/`else if` branches at `:5192-5240` — from `if (constraint.schemaType == "RigExecPositionConstraint") {` through the closing of the Parent branch — with:

```cpp
        const _ConstraintHandler *handler =
            _FindConstraintHandler(constraint.schemaType);
        if (handler && handler->solve) {
            _ConstraintSolveContext solveContext;
            solveContext.resolved = &_resolvedInputs;
            solveContext.prim = prim;
            solveContext.time = time;
            solveContext.inputFrame = inputFrame;
            solveContext.sources = &sources;
            solveContext.affect = affect;
            solveContext.order = order;
            solveContext.weight = weight;
            candidate = handler->solve(solveContext);
        } else {
```

The existing `} else {` that opens the Aim branch becomes the body of this `else`, so the Aim code is unchanged — verify the brace balance after editing.

- [x] **Step 3: Build and run the full suite**

```bash
cmake --build build 2>&1 | grep -E "error" | head -5
PYTHONPATH=/Users/burkard/work/usd-install/lib/python3.11/site-packages \
  ctest --test-dir build --output-on-failure 2>&1 | tail -6
```

Expected: `100% tests passed, 0 tests failed out of 12`. This task changes no behavior, so any failure is a transcription error in Step 1.

- [x] **Step 4: Verify the examples are byte-identical**

```bash
export PXR_PLUGINPATH_NAME="$PWD/build/usd/rigExecSchema/resources:$PWD/build/usd/rigExecImaging/resources:$PWD/plugin/rigExecUsdview"
for f in examples/08_AimEyes.usda examples/10_AimXformTurret.usda \
         examples/ArmRig.usda examples/rotateConstraint.usda; do
  printf "%-30s " "$(basename $f)"
  ./build/rigExecPose "$f" --frames 1 --targets --joints 2>&1 | md5
done
```

Record these digests. They must match the values captured before Task 2 — capture them first by stashing, or compare against the Phase 1 commit by checking out `02beeaa` into a scratch build if a mismatch appears.

- [x] **Step 5: Commit**

```bash
git add libs/rigExec/rigEvaluator.cpp
git commit -m "rigExec: kernel-backed constraints solve through the registry"
```

---

### Task 3: Rotation-order capability comes from the table

**Files:**
- Modify: `libs/rigExec/rigEvaluator.cpp:1514-1523`, `:1559-1563`
- Test: `tests/testRigExecConstraints.cpp`

**Interfaces:**
- Consumes: `_FindConstraintHandler` and `usesRotationOrder` from Task 1.

- [x] **Step 1: Write the characterization test**

This test pins today's behavior so Phase 3's fix is a visible, deliberate change rather than an accident. Add to `tests/testRigExecConstraints.cpp` before `TestInvalidContractsFailClosed`:

```cpp
// Phase 2 records which operators honor rigExec:rotationOrder; it does not
// yet reject the others. This pins the current behavior so that phase 3's
// change -- making it a compile error on Position and Scale -- shows up as a
// deliberate edit to this test rather than a silent drift.
static void
TestRotationOrderCapabilityIsRecorded()
{
    CHECK(RigExecConstraintUsesRotationOrder(
        TfToken("RigExecRotationConstraint")));
    CHECK(RigExecConstraintUsesRotationOrder(TfToken("RigExecAimConstraint")));
    CHECK(RigExecConstraintUsesRotationOrder(
        TfToken("RigExecParentConstraint")));
    CHECK(!RigExecConstraintUsesRotationOrder(
        TfToken("RigExecPositionConstraint")));
    CHECK(!RigExecConstraintUsesRotationOrder(
        TfToken("RigExecScaleConstraint")));

    // Phase 2 is behavior-neutral: authoring it where it is ignored still
    // compiles. Phase 3 turns this CHECK around.
    const UsdStageRefPtr stage = UsdStage::CreateInMemory();
    MakeXform(stage, SdfPath("/Asset"), Matrix());
    MakeXform(stage, SdfPath("/Asset/Target"), Matrix());
    MakeXform(stage, SdfPath("/Asset/Source"), Matrix(GfVec3d(1, 0, 0)));
    stage->DefinePrim(SdfPath("/Asset/Rig"), TfToken("RigExecRoot"));
    stage->DefinePrim(SdfPath("/Asset/Rig/Movers"), TfToken("Scope"));
    const UsdPrim position = MakeConstraint(
        stage, "Pos", "RigExecPositionConstraint", {SdfPath("/Asset/Target")});
    position.CreateRelationship(TfToken("rigExec:sources"))
        .SetTargets({SdfPath("/Asset/Source")});
    position.CreateAttribute(TfToken("rigExec:rotationOrder"),
                             SdfValueTypeNames->Token, /*custom=*/false)
        .Set(TfToken("ZYX"));
    RigExecRigEvaluator evaluator(stage, SdfPath("/Asset/Rig"));
    std::vector<std::string> errors;
    CHECK(evaluator.Compile(&errors));
}
```

Declare the accessor in `libs/rigExec/rigEvaluator.h` next to the Task 1 accessors:

```cpp
/// Whether \p schemaType's operator honors rigExec:rotationOrder. False for
/// a type with no registry row.
bool RigExecConstraintUsesRotationOrder(const TfToken &schemaType);
```

Register the test in `main()` after `TestConstraintRegistryCoversTheSchema();`:

```cpp
    TestRotationOrderCapabilityIsRecorded();
```

- [x] **Step 2: Run to verify it fails**

```bash
cmake --build build 2>&1 | grep -E "error" | head -3
```

Expected: FAIL to compile — accessor undefined.

- [x] **Step 3: Define the accessor and use the table at both validation sites**

At the end of `libs/rigExec/rigEvaluator.cpp`, beside the Task 1 accessors:

```cpp
bool
RigExecConstraintUsesRotationOrder(const TfToken &schemaType)
{
    const _ConstraintHandler *handler = _FindConstraintHandler(schemaType);
    return handler && handler->usesRotationOrder;
}
```

At `:1514-1523`, replace the type-name disjunction that selects which operators validate `rigExec:rotationOrder`:

```cpp
            if (record.schemaType == "RigExecAimConstraint") {
```

…through the end of that `else if (… "RigExecParentConstraint")` condition, with a single guard:

```cpp
            const _ConstraintHandler *orderHandler =
                _FindConstraintHandler(record.schemaType);
            if (orderHandler && orderHandler->usesRotationOrder) {
```

Preserve the existing body exactly. Do the same at `:1559-1563`, where the same three type names gate the time-sample rejection:

```cpp
            if ((record.schemaType == "RigExecAimConstraint" ||
                 record.schemaType == "RigExecRotationConstraint" ||
                 record.schemaType == "RigExecParentConstraint") &&
```

becomes:

```cpp
            if (orderHandler && orderHandler->usesRotationOrder &&
```

reusing the same local if it is in scope; otherwise re-look-up.

- [x] **Step 4: Build and run the full suite**

```bash
cmake --build build 2>&1 | grep -E "error" | head -5
PYTHONPATH=/Users/burkard/work/usd-install/lib/python3.11/site-packages \
  ctest --test-dir build --output-on-failure 2>&1 | tail -6
```

Expected: `100% tests passed, 0 tests failed out of 12`.

- [x] **Step 5: Commit**

```bash
git add libs/rigExec/rigEvaluator.cpp libs/rigExec/rigEvaluator.h tests/testRigExecConstraints.cpp
git commit -m "rigExec: rotation-order capability is a table column"
```

---

## Phase 2 exit criteria

- `_IsSourceFrameConstraintType` and `_IsFrameConstraintType` contain no type-name string literals.
- Position, Rotation, Scale and Parent dispatch through `handler->solve`.
- The two `rigExec:rotationOrder` validation sites read `usesRotationOrder`.
- All 12 ctest suites green; the 18 compiling examples still compile.
- No schema file modified.
