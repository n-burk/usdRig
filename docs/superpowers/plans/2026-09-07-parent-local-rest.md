# Parent-Local Rest Frames Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make a joint's rest offset describe a transform relative to its parent frame provider, so moving a parent's rest carries its children — and give the manipulator a held key that holds children still instead.

**Architecture:** `computeRestFrame` gains a `NamespaceAncestor` input and multiplies the parent's rest frame in. `_ComputeDefaultSpace` already divides that same factor back out, so it and every other world-rest consumer keep working untouched. A migration tool rebakes the seven in-repo examples; `gizmoMath.py` mirrors the composition change and its Pivot drag inverse gains the same factor.

**Tech Stack:** C++17, OpenUSD (PR-4156 checkout at `/Users/burkard/work/usd-pr4156`, install at `/Users/burkard/work/usd-install`), OpenExec computation registration, pybind11, Python 3.11, CMake + Ninja, ctest.

**Spec:** `docs/superpowers/specs/2026-09-07-parent-local-rest-design.md`

## Global Constraints

- Row-vector matrix convention throughout. Composition order is `local * space * parent`, matching the existing `local * space` in `_JointRestSpace`.
- Rest frames stay orthonormal (the `Ir` contract). Orthonormalize the local factor *before* the parent multiply; the parent frame is already orthonormal.
- A frame provider with no RigExec frame-provider ancestor keeps its current rest values and world frame exactly. Top-level rests do not change meaning.
- Build: `ninja -C build` and `ninja -C build-python`.
- C++ tests: `PYTHONPATH=/Users/burkard/work/usd-install/lib/python3.11/site-packages ctest --test-dir build --output-on-failure`
- Python scripts run under `/Users/burkard/work/usd-pr4156-venv/bin/python`.
- Probes need `PXR_PLUGINPATH_NAME="$PWD/build/usd/rigExecSchema/resources:$PWD/build/usd/rigExecImaging/resources"` (absolute paths; the *generated* schema resources, not the checked-in ones).
- Work on branch `parent-local-rest`. The worktree carries the user's long-lived unrelated edits — `git add` only the files each task names, never `git add -A`.

---

### Task 1: Parent-local rest in the evaluator

**Files:**
- Create: `tests/python/test_rest_local.py`
- Modify: `CMakeLists.txt` (register the new test beside `testRigExecInverse`, around line 378)
- Modify: `libs/rigExec/computations.cpp:230-253` (`_JointRestSpace`, `_ComputeJointRestFrame`), `:415-425` (the `computeRestFrame` registration inside `RIGEXEC_REGISTER_XFORMABLE`)
- Modify: `libs/rigExecSchema/schema.usda:243` (the `rest:space` doc string)

**Interfaces:**
- Consumes: nothing from earlier tasks.
- Produces: the semantics every later task depends on — `computeRestFrame` returns `orthonormalize(compose(rest:t, rest:r, XYZ) * rest:space) * restFrame(namespace ancestor)`. No new C++ symbols are exported; `_JointRestSpace` and `_ComputeJointRestFrame` keep their existing static signatures (`GfMatrix4d(const VdfContext &)` and `RigExecPointFrame(const VdfContext &)`).

- [ ] **Step 1: Write the failing test**

Create `tests/python/test_rest_local.py`:

```python
#!/usr/bin/env python
"""
A joint's rest offset is relative to its parent frame provider.

Moving a parent's rest must carry its descendants, the way moving its
avars or default channels already does. Before this change
computeRestFrame read no namespace ancestor (computations.cpp:415), so
a parent's rest edit moved the parent alone.

Usage: test_rest_local.py [<generated schema resources dir>]
"""
import os
import sys

import rigexec_test_env
rigexec_test_env.SetupPluginTest()

from pxr import Plug, Sdf, Usd  # noqa: E402

_HERE = os.path.dirname(os.path.abspath(__file__))
_EXAMPLE = os.path.normpath(os.path.join(
    _HERE, "..", "..", "examples", "components", "spider_leg_ik.usd"))
_SOLVER = "/RigRoot/Solvers/RigExecTwoBoneIk1"
_JOINTS = ("/RigRoot/Joints/Shoulder",
           "/RigRoot/Joints/Shoulder/ankle",
           "/RigRoot/Joints/Shoulder/ankle/foot")


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _RegisterSchema():
    if len(sys.argv) > 1:
        Plug.Registry().RegisterPlugins(sys.argv[1])
    _Check(Usd.SchemaRegistry().IsConcrete("RigExecJoint"),
           "RigExecJoint schema is not registered")


def _Origins(stage):
    """World joint origins with the IK solver unbound."""
    import _rigexec
    rig = _rigexec.Rig(stage, "/RigRoot")
    rig.compile()
    pose = rig.evaluate(0.0)
    return [tuple(pose.joint_frame(j, True).to_matrix4()[12:15])
            for j in _JOINTS]


def _OpenDisconnected():
    stage = Usd.Stage.Open(_EXAMPLE)
    stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
    stage.GetPrimAtPath(_SOLVER).GetRelationship(
        "rigExec:joints").SetTargets([])
    return stage


def _Bump(stage, primPath, name, delta):
    prim = stage.GetPrimAtPath(primPath)
    attr = prim.GetAttribute(name) or prim.CreateAttribute(
        name, Sdf.ValueTypeNames.Double)
    attr.Set((attr.Get() or 0.0) + delta)


def TestParentRestCarriesDescendants():
    base = _Origins(_OpenDisconnected())
    stage = _OpenDisconnected()
    _Bump(stage, _JOINTS[0], "rest:tx", 3.0)
    moved = _Origins(stage)
    for index, joint in enumerate(_JOINTS):
        delta = moved[index][0] - base[index][0]
        _Check(abs(delta - 3.0) < 1e-9,
               "%s moved by %.6f in x, expected 3.0: a parent's rest edit "
               "must carry every descendant" % (joint, delta))
        for axis in (1, 2):
            _Check(abs(moved[index][axis] - base[index][axis]) < 1e-9,
                   "%s moved off the x axis" % joint)


def TestRestMatchesAvarPropagation():
    """A rest edit and an avar edit of the same size move the chain alike."""
    restStage = _OpenDisconnected()
    _Bump(restStage, _JOINTS[0], "rest:tx", 3.0)
    avarStage = _OpenDisconnected()
    _Bump(avarStage, _JOINTS[0], "avars:tx", 3.0)
    for index, joint in enumerate(_JOINTS):
        rest = _Origins(restStage)[index]
        avar = _Origins(avarStage)[index]
        for axis in range(3):
            _Check(abs(rest[axis] - avar[axis]) < 1e-9,
                   "%s: rest:tx and avars:tx disagree on axis %d "
                   "(%.6f vs %.6f)" % (joint, axis, rest[axis], avar[axis]))


def TestTopLevelProviderUnchanged():
    """A provider with no RigExec ancestor keeps its absolute rest frame."""
    stage = _OpenDisconnected()
    before = _Origins(stage)[0]
    _Check(abs(before[1] - 4.545887511986089) < 1e-9,
           "Shoulder is not at its authored height; the top-level rest "
           "frame changed meaning, which it must not")


if __name__ == "__main__":
    _RegisterSchema()
    TestParentRestCarriesDescendants()
    TestRestMatchesAvarPropagation()
    TestTopLevelProviderUnchanged()
    print("test_rest_local: OK")
```

Register it in `CMakeLists.txt`, immediately after the `testRigExecInverse` block (line 378-380):

```cmake
    add_test(NAME testRestLocal COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rest_local.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
```

- [ ] **Step 2: Run it to make sure it fails**

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DUSD_INSTALL_DIR=/Users/burkard/work/usd-install \
  -DCMAKE_PREFIX_PATH=/Users/burkard/work/usd-install
PYTHONPATH=/Users/burkard/work/usd-install/lib/python3.11/site-packages \
  ctest --test-dir build -R testRestLocal --output-on-failure
```

Expected: FAIL in `TestParentRestCarriesDescendants` — `/RigRoot/Joints/Shoulder/ankle moved by 0.000000 in x, expected 3.0`.

- [ ] **Step 3: Add the ancestor input to the registration**

In `libs/rigExec/computations.cpp`, inside `RIGEXEC_REGISTER_XFORMABLE` at line 415, extend the `computeRestFrame` inputs. The `NamespaceAncestor` form is copied verbatim from `computedDefaultSpace` twelve lines below:

```cpp
        self.PrimComputation(_tokens->computeRestFrame)                      \
            .Callback<RigExecPointFrame>(&_ComputeJointRestFrame)            \
            .Inputs(                                                         \
                AttributeValue<GfMatrix4d>(_tokens->restSpace),              \
                AttributeValue<double>(_tokens->restTx),                     \
                AttributeValue<double>(_tokens->restTy),                     \
                AttributeValue<double>(_tokens->restTz),                     \
                AttributeValue<double>(_tokens->restRx),                     \
                AttributeValue<double>(_tokens->restRy),                     \
                AttributeValue<double>(_tokens->restRz),                     \
                NamespaceAncestor<RigExecPointFrame>(                        \
                    _tokens->computeRestFrame)                               \
                    .InputName(_tokens->parentRestFrame));                   \
```

`_tokens->parentRestFrame` already exists (`computations.cpp:131`); no new token is needed.

- [ ] **Step 4: Multiply the parent frame in**

Replace `_JointRestSpace` at `computations.cpp:229-248`:

```cpp
// Rest space relative to the namespace frame provider: the authored
// rest:space with the rest avars as a preceding local delta, carried into
// the parent's rest frame. A provider with no ancestor gets identity, so a
// top-level rest keeps its authored absolute meaning.
static GfMatrix4d
_JointRestSpace(const VdfContext &ctx)
{
    const GfMatrix4d *space =
        ctx.GetInputValuePtr<GfMatrix4d>(_tokens->restSpace);
    const GfMatrix4d local = _ComposeAvars(
        _ScalarInput(ctx, _tokens->restTx, 0),
        _ScalarInput(ctx, _tokens->restTy, 0),
        _ScalarInput(ctx, _tokens->restTz, 0),
        1.0, 1.0, 1.0,
        _ScalarInput(ctx, _tokens->restRx, 0),
        _ScalarInput(ctx, _tokens->restRy, 0),
        _ScalarInput(ctx, _tokens->restRz, 0),
        0.0, TfToken("XYZ"));
    GfMatrix4d rest = local * (space ? *space : GfMatrix4d(1.0));
    // Rest spaces are always orthonormalized (Ir contract). Orthonormalize
    // the local factor before the parent multiply: the parent's frame is
    // already orthonormal, so the product is too, and a missing or invalid
    // ancestor keeps its NaN sentinel intact through the multiply.
    rest.Orthonormalize(/* issueWarning = */ false);
    return rest * _SpaceFromFrame(
        ctx.GetInputValuePtr<RigExecPointFrame>(_tokens->parentRestFrame));
}
```

`_SpaceFromFrame` is declared at `computations.cpp:255`, *below* this function. Move the `_SpaceFromFrame` definition above `_JointRestSpace` so it is in scope; it has no dependency on anything between them.

- [ ] **Step 5: Run the test to verify it passes**

```bash
ninja -C build && ninja -C build-python
PYTHONPATH=/Users/burkard/work/usd-install/lib/python3.11/site-packages \
  ctest --test-dir build -R testRestLocal --output-on-failure
```

Expected: PASS, `test_rest_local: OK`.

- [ ] **Step 6: Update the schema doc**

In `libs/rigExecSchema/schema.usda:243-245`, replace the `rest:space` doc:

```
    matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1) ) (
        doc = """Bind transform relative to the namespace frame
        provider's rest frame; always orthonormalized. A provider with no
        RigExec ancestor resolves against identity, so its rest:space is
        its local-to-world bind transform."""
    )
```

Regenerate the schema per the project recipe (from `libs/rigExecSchema`, run `usdGenSchema schema.usda ../../plugin/rigExecSchema/resources`, then strip the `LibraryPath`/`@PLUG_INFO_*@` placeholders from `plugInfo.json`).

- [ ] **Step 7: Commit**

```bash
git add tests/python/test_rest_local.py CMakeLists.txt \
        libs/rigExec/computations.cpp libs/rigExecSchema/schema.usda \
        plugin/rigExecSchema/resources/generatedSchema.usda
git commit -m "feat: make joint rest offsets parent-relative"
```

Note: `plugin/rigExecSchema/resources/generatedSchema.usda` already carries unrelated working-tree edits. Stage it only if the regeneration touched it; if the diff is larger than the `rest:space` doc, restore it and regenerate from a clean copy.

---

### Task 2: Migration tool

**Files:**
- Create: `tools/migrateRestToLocal.py`
- Create: `tests/python/test_rest_migration.py`
- Modify: `CMakeLists.txt` (register the new test beside `testRestLocal`)

**Interfaces:**
- Consumes: Task 1's semantics.
- Produces: `migrateRestToLocal.MigrateStage(stage) -> int` (number of providers rewritten; 0 when already stamped) and `migrateRestToLocal.RIG_VERSION_ATTR = "rigExec:restFrameVersion"`. Task 3 calls `MigrateStage`.

- [ ] **Step 1: Write the failing test**

Create `tests/python/test_rest_migration.py`:

```python
#!/usr/bin/env python
"""
Migrating an absolute-rest asset preserves every world joint origin.

The rest semantics changed from absolute to parent-relative, so authored
rest transforms must be rebaked. The rebake is correct exactly when the
evaluated pose is unchanged, which is what this asserts.

Usage: test_rest_migration.py [<generated schema resources dir>]
"""
import os
import sys

import rigexec_test_env
rigexec_test_env.SetupPluginTest()

from pxr import Plug, Usd  # noqa: E402

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.normpath(os.path.join(_HERE, "..", ".."))
sys.path.insert(0, os.path.join(_REPO, "tools"))
import migrateRestToLocal  # noqa: E402

_EXAMPLES = (
    ("examples/01_FkChainTail.usda", "/TailAsset/Rig"),
    ("examples/02_TwoBoneIkLeg.usda", None),
    ("examples/03_IkFkBlendClamp.usda", None),
    ("examples/05_TwistRibbonSpine.usda", None),
    ("examples/ArmRig.usda", "/ArmAsset/Rig"),
    ("examples/components/spider_leg.usd", None),
    ("examples/components/spider_leg_ik.usd", "/RigRoot"),
)


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _RegisterSchema():
    if len(sys.argv) > 1:
        Plug.Registry().RegisterPlugins(sys.argv[1])
    _Check(Usd.SchemaRegistry().IsConcrete("RigExecJoint"),
           "RigExecJoint schema is not registered")


def _RigPath(stage, declared):
    if declared:
        return declared
    roots = [p.GetPath() for p in stage.Traverse()
             if p.GetTypeName() == "RigExecRoot"]
    _Check(len(roots) == 1, "expected exactly one RigExecRoot")
    return str(roots[0])


def _Origins(stage, rigPath):
    import _rigexec
    rig = _rigexec.Rig(stage, rigPath)
    rig.compile()
    time = (stage.GetStartTimeCode()
            if stage.HasAuthoredTimeCodeRange() else 0.0)
    pose = rig.evaluate(time)
    return {j: tuple(pose.joint_frame(j, True).to_matrix4()[12:15])
            for j in pose.joint_paths()}


def TestMigrationPreservesWorldOrigins():
    for relative, declared in _EXAMPLES:
        path = os.path.join(_REPO, relative)
        stage = Usd.Stage.Open(path)
        _Check(stage is not None, "could not open %s" % relative)
        rigPath = _RigPath(stage, declared)

        # Baseline under the OLD semantics, read off the unmigrated file
        # through the absolute-rest reference implementation the tool
        # carries for exactly this purpose.
        before = migrateRestToLocal.AbsoluteWorldRests(stage)

        stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
        rewritten = migrateRestToLocal.MigrateStage(stage)
        _Check(rewritten > 0, "%s: nothing was migrated" % relative)

        after = _Origins(stage, rigPath)
        for joint, origin in after.items():
            _Check(joint in before, "%s: %s appeared" % (relative, joint))
            for axis in range(3):
                _Check(abs(origin[axis] - before[joint][axis]) < 1e-9,
                       "%s: %s axis %d moved %.12f -> %.12f" % (
                           relative, joint, axis,
                           before[joint][axis], origin[axis]))


def TestMigrationIsIdempotent():
    path = os.path.join(_REPO, "examples/ArmRig.usda")
    stage = Usd.Stage.Open(path)
    stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
    first = migrateRestToLocal.MigrateStage(stage)
    _Check(first > 0, "first migration did nothing")
    origins = _Origins(stage, "/ArmAsset/Rig")
    second = migrateRestToLocal.MigrateStage(stage)
    _Check(second == 0, "second migration rewrote %d provider(s); the "
                        "version stamp did not suppress it" % second)
    _Check(_Origins(stage, "/ArmAsset/Rig") == origins,
           "the second migration moved joints")


if __name__ == "__main__":
    _RegisterSchema()
    TestMigrationPreservesWorldOrigins()
    TestMigrationIsIdempotent()
    print("test_rest_migration: OK")
```

Register in `CMakeLists.txt` beside `testRestLocal`:

```cmake
    add_test(NAME testRestMigration COMMAND ${Python3_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rest_migration.py"
        "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
```

- [ ] **Step 2: Run it to make sure it fails**

```bash
PYTHONPATH=/Users/burkard/work/usd-install/lib/python3.11/site-packages \
  ctest --test-dir build -R testRestMigration --output-on-failure
```

Expected: FAIL with `ModuleNotFoundError: No module named 'migrateRestToLocal'`.

- [ ] **Step 3: Write the tool**

Create `tools/migrateRestToLocal.py`:

```python
#!/usr/bin/env python
"""
Rebake absolute rest transforms into parent-relative ones.

Joint rest offsets used to be absolute in asset space: computeRestFrame
read no namespace ancestor, so a chain authored its joints' world bind
positions directly. They are now relative to the parent frame provider's
rest frame, which means every authored rest in an existing asset has to
be divided by its parent's.

For each frame provider with a frame-provider ancestor, parents first:

    R_local = R_absolute * R_parent_absolute^-1

R is the provider's whole composed rest -- compose(rest:t, rest:r) *
rest:space -- so one routine covers assets authoring rest:space matrices
and assets authoring rest:t/rest:r scalars alike. The result is
decomposed back into whichever spelling the prim actually authored.

Usage: migrateRestToLocal.py <stage.usd> [<stage.usd> ...]
"""
import sys

from pxr import Gf, Sdf, Usd

RIG_VERSION_ATTR = "rigExec:restFrameVersion"
RIG_VERSION = 2

_PROVIDER_TYPES = ("RigExecJoint", "RigExecControl")
_REST_T = ("rest:tx", "rest:ty", "rest:tz")
_REST_R = ("rest:rx", "rest:ry", "rest:rz")
_REST_SPACE = "rest:space"


def _IsProvider(prim):
    return bool(prim) and prim.GetTypeName() in _PROVIDER_TYPES


def _ParentProvider(prim):
    parent = prim.GetParent()
    while parent and parent.IsValid():
        if _IsProvider(parent):
            return parent
        parent = parent.GetParent()
    return None


def _Scalar(prim, name):
    attr = prim.GetAttribute(name)
    if attr:
        value = attr.Get()
        if value is not None:
            return float(value)
    return 0.0


def _Compose(translate, rotate):
    """compose(t, r) in XYZ, matching _ComposeAvars with no scale/spin."""
    matrix = Gf.Matrix4d(1.0)
    matrix.SetRotate(
        Gf.Rotation(Gf.Vec3d(1, 0, 0), rotate[0]) *
        Gf.Rotation(Gf.Vec3d(0, 1, 0), rotate[1]) *
        Gf.Rotation(Gf.Vec3d(0, 0, 1), rotate[2]))
    matrix.SetTranslateOnly(Gf.Vec3d(*translate))
    return matrix


def _AuthoredRest(prim):
    """The prim's composed rest: orthonormalize(local * rest:space)."""
    local = _Compose([_Scalar(prim, n) for n in _REST_T],
                     [_Scalar(prim, n) for n in _REST_R])
    space = Gf.Matrix4d(1.0)
    attr = prim.GetAttribute(_REST_SPACE)
    if attr:
        value = attr.Get()
        if value is not None:
            space = Gf.Matrix4d(value)
    return (local * space).GetOrthonormalized(False)


def AbsoluteWorldRests(stage):
    """
    World rest origins under the OLD absolute semantics.

    The migration's correctness check needs the pre-change answer, and
    the evaluator no longer produces it. This is the two-line reference
    implementation of what computeRestFrame used to do.
    """
    origins = {}
    for prim in stage.Traverse():
        if _IsProvider(prim):
            origins[str(prim.GetPath())] = tuple(
                _AuthoredRest(prim).ExtractTranslation())
    return origins


def _RigRoots(stage):
    return [p for p in stage.Traverse() if p.GetTypeName() == "RigExecRoot"]


def _AlreadyMigrated(stage):
    for root in _RigRoots(stage):
        attr = root.GetAttribute(RIG_VERSION_ATTR)
        if attr and (attr.Get() or 0) >= RIG_VERSION:
            return True
    return False


def _Stamp(stage):
    for root in _RigRoots(stage):
        attr = root.GetAttribute(RIG_VERSION_ATTR) or root.CreateAttribute(
            RIG_VERSION_ATTR, Sdf.ValueTypeNames.Int, custom=True)
        attr.Set(RIG_VERSION)


def MigrateStage(stage):
    """Rewrite authored rests parent-relative. Returns providers rewritten."""
    if _AlreadyMigrated(stage):
        return 0

    # Parents before children: a child divides by its parent's ABSOLUTE
    # rest, so the parent's absolute value must be read before it is
    # overwritten. Depth order over the namespace gives exactly that.
    providers = [p for p in stage.Traverse() if _IsProvider(p)]
    providers.sort(key=lambda p: len(p.GetPath().pathElementCount * "x"))
    absolute = {str(p.GetPath()): _AuthoredRest(p) for p in providers}

    rewritten = 0
    for prim in providers:
        parent = _ParentProvider(prim)
        if not parent:
            continue
        local = (absolute[str(prim.GetPath())] *
                 absolute[str(parent.GetPath())].GetInverse())
        _WriteRest(prim, local)
        rewritten += 1

    _Stamp(stage)
    return rewritten


def _WriteRest(prim, local):
    """
    Write `local` back into the spelling the prim already uses.

    A prim authoring rest:space keeps carrying its transform there, with
    the rest:t/r scalars left alone. A prim authoring only scalars gets
    scalars, so a hand-written rest:tx stays readable as a number rather
    than turning into a matrix.
    """
    hasSpace = bool(prim.GetAttribute(_REST_SPACE) and
                    prim.GetAttribute(_REST_SPACE).HasAuthoredValue())
    if hasSpace:
        # rest:space carries the whole transform; divide the local
        # scalars back out so local == compose(t, r) * rest:space holds.
        scalars = _Compose([_Scalar(prim, n) for n in _REST_T],
                           [_Scalar(prim, n) for n in _REST_R])
        prim.GetAttribute(_REST_SPACE).Set(scalars.GetInverse() * local)
        return

    rotation = local.ExtractRotation().Decompose(
        Gf.Vec3d(1, 0, 0), Gf.Vec3d(0, 1, 0), Gf.Vec3d(0, 0, 1))
    translation = local.ExtractTranslation()
    for name, value in zip(_REST_T, translation):
        attr = prim.GetAttribute(name) or prim.CreateAttribute(
            name, Sdf.ValueTypeNames.Double)
        attr.Set(float(value))
    for name, value in zip(_REST_R, rotation):
        attr = prim.GetAttribute(name) or prim.CreateAttribute(
            name, Sdf.ValueTypeNames.Double)
        attr.Set(float(value))


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 2
    for path in argv[1:]:
        stage = Usd.Stage.Open(path)
        if not stage:
            print("could not open %s" % path)
            return 2
        count = MigrateStage(stage)
        if count:
            stage.GetRootLayer().Save()
            print("%s: rebased %d provider(s)" % (path, count))
        else:
            print("%s: already migrated" % path)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
```

- [ ] **Step 4: Run the test to verify it passes**

```bash
PYTHONPATH=/Users/burkard/work/usd-install/lib/python3.11/site-packages \
  ctest --test-dir build -R testRestMigration --output-on-failure
```

Expected: PASS, `test_rest_migration: OK`.

If `TestMigrationPreservesWorldOrigins` fails only on `02_TwoBoneIkLeg` or `03_IkFkBlendClamp`, check whether the moved joints are solver-posed — a solver-posed joint's final frame comes from the solve, not from its rest, so compare only joints the solver does not drive. Do **not** loosen the 1e-9 tolerance to make a failure go away.

- [ ] **Step 5: Commit**

```bash
git add tools/migrateRestToLocal.py tests/python/test_rest_migration.py CMakeLists.txt
git commit -m "feat: add parent-local rest migration tool"
```

---

### Task 3: Migrate the in-repo assets and re-baseline the suite

**Files:**
- Modify: `examples/01_FkChainTail.usda`, `examples/02_TwoBoneIkLeg.usda`, `examples/03_IkFkBlendClamp.usda`, `examples/05_TwistRibbonSpine.usda`, `examples/ArmRig.usda`, `examples/components/spider_leg.usd`, `examples/components/spider_leg_ik.usd`
- Modify: whichever of `tests/testRigExecArm.cpp`, `tests/testRigExecConstraints.cpp`, `tests/testRigExecRigBuilder.cpp`, `tests/testRigExecSingleChainIk.cpp`, `tests/testRigExecDefaultSpaces.cpp` set nested rest translations (46 call sites across `tests/` and `libs/rigExecRigging/rigBuilder.*` — enumerate with the grep in Step 1)

**Interfaces:**
- Consumes: `migrateRestToLocal.MigrateStage` from Task 2.
- Produces: a green `ctest` suite under the new semantics. No new symbols.

- [ ] **Step 1: Enumerate what has to change**

```bash
grep -rn "SetRestTranslation\|set_rest_translation\|rest:tx\|rest:space" \
  tests/ libs/rigExecRigging/ | tee /tmp/rest-call-sites.txt | wc -l
PYTHONPATH=/Users/burkard/work/usd-install/lib/python3.11/site-packages \
  ctest --test-dir build --output-on-failure 2>&1 | tail -40
```

Record which tests fail before touching anything — that list is the work.

- [ ] **Step 2: Migrate the example assets**

```bash
/Users/burkard/work/usd-pr4156-venv/bin/python tools/migrateRestToLocal.py \
  examples/01_FkChainTail.usda examples/02_TwoBoneIkLeg.usda \
  examples/03_IkFkBlendClamp.usda examples/05_TwistRibbonSpine.usda \
  examples/ArmRig.usda examples/components/spider_leg.usd \
  examples/components/spider_leg_ik.usd
```

Expected output: one `rebased N provider(s)` line per file.

- [ ] **Step 3: Verify the examples still evaluate identically**

```bash
PYTHONPATH=/Users/burkard/work/usd-install/lib/python3.11/site-packages \
  ctest --test-dir build -R "testRestMigration|testRestLocal" --output-on-failure
```

`testRestMigration` now reports every example as already migrated, so relax it: it must skip a stage that reports 0 rather than failing. Change the `_Check(rewritten > 0, ...)` in `TestMigrationPreservesWorldOrigins` to `if rewritten == 0: continue` and keep `TestMigrationIsIdempotent` as the coverage that the rewrite path works.

Confirm `01_FkChainTail` reads as expected — `Seg1` keeps `(0,5,0)`, and `Seg2`/`Seg3`/`Seg4` each become `(2,0,0)`:

```bash
/Users/burkard/work/usd-install/bin/usdcat examples/01_FkChainTail.usda | grep -A1 "RigExecJoint\|rest:space" | head -30
```

- [ ] **Step 4: Re-baseline the C++ test expectations**

For each failing test from Step 1, the fix is the same shape: a nested joint's authored rest is now a parent-relative offset, so an expectation written as an absolute world position becomes the difference from its parent. Fix the *authored inputs* where the test builds a rig (make each nested joint's rest the offset from its parent), and leave the *expected world outputs* alone — the whole point is that world output does not change.

Run after each file:

```bash
ninja -C build && PYTHONPATH=/Users/burkard/work/usd-install/lib/python3.11/site-packages \
  ctest --test-dir build --output-on-failure
```

- [ ] **Step 5: Full suite green**

```bash
ninja -C build && ninja -C build-python
PYTHONPATH=/Users/burkard/work/usd-install/lib/python3.11/site-packages \
  ctest --test-dir build --output-on-failure
```

Expected: 100% tests passed.

- [ ] **Step 6: Commit**

```bash
git add examples/ tests/ libs/rigExecRigging/
git commit -m "refactor: rebake assets and tests for parent-local rest"
```

---

### Task 4: Mirror the change in the gizmo

**Files:**
- Modify: `plugin/rigExecUsdview/gizmoMath.py:428-431` (`RestSpace`), `:521-551` (the `RigFrames` docstring), `:690-705` (`ComputeRigFrames` — add `parentRest`), `:1357-1364` (`RigPivotTarget._Qw`)
- Modify: `tests/python/test_gizmo_math.py`

**Interfaces:**
- Consumes: Task 1's semantics.
- Produces: `RigFrames.parentRest` (a `Gf.Matrix4d`, the parent frame provider's asset-space rest frame, identity when there is none). `RigPivotTarget._Qw()` returns `Qrest * parentRest * assetToWorld`.

- [ ] **Step 1: Write the failing test**

Add to `tests/python/test_gizmo_math.py`, following the file's existing test style:

```python
def TestPivotDragIsParentRelative():
    """
    A Pivot drag on a child writes rest:t in the PARENT's frame.

    RigPivotTarget inverts the drag through _Qw(). Rest offsets are now
    carried into the parent's rest frame, so _Qw must include it -- a
    child under a rotated parent otherwise gets a rest:t that lands the
    joint somewhere other than where the handle was dragged.
    """
    stage = _OpenSpiderStage()
    parent = stage.GetPrimAtPath("/RigRoot/Joints/Shoulder")
    child = stage.GetPrimAtPath("/RigRoot/Joints/Shoulder/ankle")
    # Rotate the parent's rest so parent-relative and absolute differ.
    parent.GetAttribute("rest:rz").Set(90.0)

    frames = gizmoMath.ComputeRigFrames(stage, child, 0.0, set(), {})
    expected = gizmoMath.RestSpace(parent, 0.0)
    for row in range(4):
        for col in range(4):
            _Check(abs(frames.parentRest[row][col] - expected[row][col]) < 1e-9,
                   "parentRest[%d][%d] is %.9f, expected %.9f" % (
                       row, col, frames.parentRest[row][col],
                       expected[row][col]))

    target = gizmoMath.RigPivotTarget(stage, child, _Writer(), set())
    target.Refresh()
    before = gizmoMath.RestSpace(child, 0.0).ExtractTranslation()
    target.ApplyTranslate(Gf.Vec3d(1.0, 0.0, 0.0))
    after = gizmoMath.RestSpace(child, 0.0).ExtractTranslation()
    moved = Gf.Vec3d(after) - Gf.Vec3d(before)
    _Check(abs(moved[0] - 1.0) < 1e-9 and abs(moved[1]) < 1e-9
           and abs(moved[2]) < 1e-9,
           "a 1-unit world drag moved the child rest by %s, not (1,0,0)"
           % (moved,))
```

If `test_gizmo_math.py` has no `_OpenSpiderStage` or `_Writer` helper, add them mirroring whatever the neighbouring tests already use to open a stage and capture writes.

- [ ] **Step 2: Run it to make sure it fails**

```bash
PYTHONPATH=/Users/burkard/work/usd-install/lib/python3.11/site-packages \
  ctest --test-dir build -R testGizmoMath --output-on-failure
```

Expected: FAIL with `AttributeError: 'RigFrames' object has no attribute 'parentRest'`.

- [ ] **Step 3: Make RestSpace parent-relative**

Replace `gizmoMath.py:428-431`:

```python
def RestSpace(prim, time):
    """
    Mirror of _JointRestSpace: the local rest carried into the parent's
    rest frame. orthonormalize(restLocal * rest:space) * parentRest.
    """
    rest = RestLocal(prim, time) * _MatrixAttr(prim, REST_SPACE, time)
    rest = rest.GetOrthonormalized(False)
    parent = _FindParentXformable(prim, FindRigRoot(prim))
    if not parent:
        return rest
    return rest * RestSpace(parent, time)
```

- [ ] **Step 4: Carry the parent frame on RigFrames**

In `ComputeRigFrames` (`gizmoMath.py:699-705`), beside the existing `frames.rest` / `frames.Qrest` assignments:

```python
    frames.restLocal = RestLocal(prim, time)
    frames.rest = RestSpace(prim, time)
    frames.Qrest = _MatrixAttr(prim, REST_SPACE, time)\
        .GetOrthonormalized(False)
    frames.parentRest = (RestSpace(parent, time) if parent
                         else Gf.Matrix4d(1.0))
```

`parent` is already in scope at line 675 (`parent = _FindParentXformable(prim, frames.rigRoot)`). Declare `parentRest` on the `RigFrames` class beside `Qrest`.

- [ ] **Step 5: Fix the Pivot drag inverse**

Replace `RigPivotTarget._Qw` (`gizmoMath.py:1357-1361`):

```python
    def _Qw(self):
        # Qrest AND the parent's rest, not Q: rest:t/r are expressed
        # against rest:space carried into the parent's rest frame
        # (computations.cpp:230), so the parent's REST belongs in the
        # frame this drag inverts through -- but its POSE still does not.
        return self.frames.Qrest * self.frames.parentRest \
            * self.frames.assetToWorld
```

- [ ] **Step 6: Update the RigFrames docstring**

`gizmoMath.py:530-539` states the old absolute behaviour and cites `computations.cpp:200-219`. Replace the `Qrest` paragraph:

```
      Qrest      orthonormalize(rest:space): the local factor of the rest
                 frame. Pivot mode edits relative to Qrest * parentRest,
                 because computeRestFrame composes rest:t/rest:r against
                 rest:space and then carries the result into the parent's
                 rest frame (computations.cpp:230). A rest frame is
                 therefore relative to the parent's REST -- unmoved by an
                 animated ancestor and by a solver posing this joint,
                 both of which act on the POSE side.
      parentRest the namespace frame provider's rest frame, identity at
                 the top of a chain.
```

- [ ] **Step 7: Run the test to verify it passes**

```bash
PYTHONPATH=/Users/burkard/work/usd-install/lib/python3.11/site-packages \
  ctest --test-dir build -R testGizmoMath --output-on-failure
```

Expected: PASS.

- [ ] **Step 8: Commit**

```bash
git add plugin/rigExecUsdview/gizmoMath.py tests/python/test_gizmo_math.py
git commit -m "fix: invert Pivot drags through the parent rest frame"
```

---

### Task 5: Held `B` compensates children

**Files:**
- Modify: `plugin/rigExecUsdview/gizmoMath.py` (add `CompensateChildren` to `RigPivotTarget`)
- Modify: `plugin/rigExecUsdview/gizmoUI.py:2227` (`_DRAG_KEYS`), press handler near `:2369-2392`, release handler near `:2435-2452`, status label near `:1807`
- Modify: `tests/python/test_gizmo_math.py`
- Modify: `docs/viewport-gizmos.md` (hotkey table at 154-168, and the Pivot description at 33-36)

**Interfaces:**
- Consumes: `RigFrames.parentRest` and `RigPivotTarget._Qw` from Task 4.
- Produces: `RigPivotTarget.CompensateChildren(previousParentRest)` — call *after* a write, passing the parent's asset-space rest frame as it was *before* the drag step; it rewrites each unselected immediate child's rest so its world rest is unchanged.

- [ ] **Step 1: Write the failing test**

Add to `tests/python/test_gizmo_math.py`:

```python
def TestCompensationHoldsChildrenStill():
    """
    Holding B rewrites unselected children so their world rest is fixed.

    Translation and rotation both, because the compensation is a full
    matrix identity: restLocal(child)' = restLocal(child) *
    restWorld(parent) * restWorld(parent)'^-1.
    """
    for channel, amount in (("rest:tx", 2.5), ("rest:rz", 30.0)):
        stage = _OpenSpiderStage()
        parentPath = "/RigRoot/Joints/Shoulder"
        childPath = "/RigRoot/Joints/Shoulder/ankle"
        parent = stage.GetPrimAtPath(parentPath)
        child = stage.GetPrimAtPath(childPath)

        before = gizmoMath.RestSpace(child, 0.0)
        previousParentRest = gizmoMath.RestSpace(parent, 0.0)

        attr = parent.GetAttribute(channel)
        attr.Set((attr.Get() or 0.0) + amount)

        target = gizmoMath.RigPivotTarget(stage, parent, _Writer(), set())
        target.Refresh()
        target.CompensateChildren(previousParentRest)

        after = gizmoMath.RestSpace(child, 0.0)
        for row in range(4):
            for col in range(4):
                _Check(abs(after[row][col] - before[row][col]) < 1e-9,
                       "%s: child world rest [%d][%d] moved %.9f -> %.9f"
                       % (channel, row, col, before[row][col],
                          after[row][col]))


def TestSelectedChildIsNotCompensated():
    """A child being dragged too is moved deliberately; leave it alone."""
    stage = _OpenSpiderStage()
    parent = stage.GetPrimAtPath("/RigRoot/Joints/Shoulder")
    childPath = "/RigRoot/Joints/Shoulder/ankle"
    child = stage.GetPrimAtPath(childPath)

    before = gizmoMath.RestSpace(child, 0.0)
    previousParentRest = gizmoMath.RestSpace(parent, 0.0)
    parent.GetAttribute("rest:tx").Set(
        (parent.GetAttribute("rest:tx").Get() or 0.0) + 2.5)

    target = gizmoMath.RigPivotTarget(stage, parent, _Writer(),
                                      set(), selection={childPath})
    target.Refresh()
    target.CompensateChildren(previousParentRest)

    after = gizmoMath.RestSpace(child, 0.0)
    moved = (Gf.Vec3d(after.ExtractTranslation())
             - Gf.Vec3d(before.ExtractTranslation()))
    _Check(abs(moved[0] - 2.5) < 1e-9,
           "a selected child was compensated; it moved %s, expected "
           "(2.5, 0, 0)" % (moved,))
```

- [ ] **Step 2: Run it to make sure it fails**

```bash
PYTHONPATH=/Users/burkard/work/usd-install/lib/python3.11/site-packages \
  ctest --test-dir build -R testGizmoMath --output-on-failure
```

Expected: FAIL with `AttributeError: 'RigPivotTarget' object has no attribute 'CompensateChildren'`.

- [ ] **Step 3: Implement the compensation**

Add to `RigPivotTarget` in `gizmoMath.py`, after `ApplyRotate`. `_RigTarget.__init__` gains an optional `selection=None` keyword storing `self._selection = set(selection or ())`; pass it through from `MakeTarget` where `RigPivotTarget` is constructed (`gizmoMath.py:1847`), sourcing it from the caller's current prim selection.

```python
    def _ImmediateChildProviders(self):
        for child in self.prim.GetChildren():
            if child.GetTypeName() in ("RigExecJoint", "RigExecControl"):
                yield child

    def CompensateChildren(self, previousParentRest):
        """
        Hold unselected immediate children at their world rest.

        Rest is parent-relative, so moving this joint moves its whole
        subtree. Post-multiplying each child's local rest by the parent's
        rest delta inverse cancels exactly that:

            restLocal(child)' = restLocal(child)
                                * restWorld(parent)
                                * restWorld(parent)'^-1

        Immediate children suffice -- deeper descendants ride on their
        parents, which this leaves unmoved.
        """
        currentParentRest = RestSpace(self.prim, self.time)
        delta = previousParentRest * currentParentRest.GetInverse()
        if Gf.IsClose(delta, Gf.Matrix4d(1.0), 1e-12):
            return
        for child in self._ImmediateChildProviders():
            if str(child.GetPath()) in self._selection:
                continue
            local = (RestLocal(child, self.time)
                     * _MatrixAttr(child, REST_SPACE, self.time))
            adjusted = (local * delta).GetOrthonormalized(False)
            self._WriteChildRest(child, adjusted)

    def _WriteChildRest(self, child, local):
        """Write a child's local rest back through rest:t/r."""
        rotation = local.ExtractRotation().Decompose(
            Gf.Vec3d(1, 0, 0), Gf.Vec3d(0, 1, 0), Gf.Vec3d(0, 0, 1))
        translation = local.ExtractTranslation()
        space = _MatrixAttr(child, REST_SPACE, self.time)
        if space != _IDENTITY:
            # rest:space carries the transform; keep the scalars and move
            # the correction into the matrix, so the two never disagree.
            scalars = RestLocal(child, self.time)
            self.writer.SetMatrix(
                child.GetPath().AppendProperty(REST_SPACE),
                scalars.GetInverse() * local)
            return
        for name, value in zip(REST_T, translation):
            self.writer.Set(child.GetPath().AppendProperty(name),
                            float(value))
        for name, value in zip(REST_R, rotation):
            self.writer.Set(child.GetPath().AppendProperty(name),
                            float(value))
```

Match `self.writer`'s real interface — check how `_Write`/`_WriteVector` (`gizmoMath.py:1167`, `:1250`) call it and use the same methods rather than the `Set`/`SetMatrix` named above if they differ.

- [ ] **Step 4: Run the test to verify it passes**

```bash
PYTHONPATH=/Users/burkard/work/usd-install/lib/python3.11/site-packages \
  ctest --test-dir build -R testGizmoMath --output-on-failure
```

Expected: PASS.

- [ ] **Step 5: Wire the `B` key**

In `gizmoUI.py:2227`, add `QtCore.Qt.Key_B` to `_DRAG_KEYS`. Add a press handler beside the `Key_V` one at `:2387`:

```python
        if key == QtCore.Qt.Key_B:
            self._holdCompensate = True
            self._RefreshStatus()
            return True
```

and a release handler beside the `Key_V` one at `:2449`:

```python
        if key == QtCore.Qt.Key_B and self._holdCompensate:
            self._holdCompensate = False
            self._RefreshStatus()
            return True
```

Initialise `self._holdCompensate = False` beside the other `_hold*` flags. At the drag step where the Pivot target is applied, capture `previousParentRest = target.frames.parentRest` *before* the write and call `target.CompensateChildren(previousParentRest)` after it, only when `self._holdCompensate` and the target is a `RigPivotTarget`. Add `compensate` to the status label near `:1807` alongside the existing snap indicators.

- [ ] **Step 6: Document it**

`docs/viewport-gizmos.md`, hotkey table (154-168), after the `V` row:

```markdown
| `B` (hold) | compensate children: hold unselected child joints at their world rest (Pivot) |
```

And amend the Pivot paragraph at 33-36, which currently reads "like moving a Maya pivot without compensation" — that describes the *pose* not compensating. Add: a Pivot drag now carries child joints, because a rest offset is relative to the parent's rest frame; hold `B` to hold the children still instead.

- [ ] **Step 7: Full suite and commit**

```bash
ninja -C build && ninja -C build-python
PYTHONPATH=/Users/burkard/work/usd-install/lib/python3.11/site-packages \
  ctest --test-dir build --output-on-failure
```

```bash
git add plugin/rigExecUsdview/gizmoMath.py plugin/rigExecUsdview/gizmoUI.py \
        tests/python/test_gizmo_math.py docs/viewport-gizmos.md
git commit -m "feat: hold B to compensate child joints during a Pivot drag"
```

---

## Self-review notes

- **Spec §3 (semantics)** → Task 1. **§4 (manipulator)** → Tasks 4 and 5. **§5 (migration)** → Tasks 2 and 3. **§6 (testing)** → items 1-4 in Tasks 1-3, items 5-6 in Tasks 5 and 3.
- `RigFrames.parentRest` is introduced in Task 4 Step 4 and consumed in Task 5 Step 3 and Step 5 under that exact name.
- `MigrateStage` / `AbsoluteWorldRests` / `RIG_VERSION_ATTR` are defined in Task 2 Step 3 and used in Task 2 Step 1 and Task 3 Step 2 under those exact names.
- Two known soft spots, flagged rather than hidden: Task 2 Step 4 anticipates solver-posed joints breaking the origin comparison, and Task 5 Step 3 tells the implementer to check `self.writer`'s real method names rather than trusting the sketch.
