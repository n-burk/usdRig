# Viewport Gizmo Toolbar Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A toolbar above usdview's viewport with an undoable (Ctrl+Z / Ctrl+Shift+Z) translate/rotate/scale gizmo that edits RigExec avars (pose or pivot/rest channels, animation or default), and xformOps for plain xforms.

**Architecture:** Three Qt-free modules (`rigExecUndo.py` spec-snapshot undo stack, `gizmoMath.py` frame replica + edit targets + writer, `gizmoScreen.py` projection/handles/drag math) drive one Qt module (`gizmoUI.py`: toolbar, transparent overlay, event-filter controller), installed lazily by the existing plugin container on first stage load.

**Tech Stack:** Python 3.11, OpenUSD (PR-4156 build) `pxr` (Gf, Sdf, Ts, Usd, UsdGeom), PySide6 through `pxr.Usdviewq.qt`, testusdview for the integration test, CMake/ctest for headless Python tests.

**Spec:** `docs/superpowers/specs/2026-09-01-viewport-gizmo-toolbar-design.md`

## Global Constraints

- Environment: source `bin/_env.sh` for every command (`. bin/_env.sh`); `$PY` is the venv python, `$TESTUSDVIEW` is testusdview. Headless python tests additionally need `PYTHONPATH="$RIG/build-python/python:$PYTHONPATH"` for the optional `_rigexec` module and `PXR_PLUGINPATH_NAME` already points at the generated schema resources.
- Style: match `plugin/rigExecUsdview/volumeWeightUI.py` — imports `from pxr import Gf, Sdf, Tf, Ts, Usd, UsdGeom` and `from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets` (never PySide directly); PascalCase module functions and methods, `_onXxx` for Qt slots, UPPER_SNAKE constants, 79-column lines, docstrings that explain WHY and cite file:line evidence.
- Qt-free rule: `rigExecUndo.py`, `gizmoMath.py`, `gizmoScreen.py` must not import Qt. Only `gizmoUI.py` may.
- Row-vector convention everywhere (`Gf.Matrix4d`: leftmost factor applies first; `v * M`).
- Tests are plain scripts (no pytest): `_Check(cond, msg)` raising `AssertionError`, `main()` running `(name, callable)` groups, exit code 0 with a printed `..._OK` banner — the convention of `tests/testUsdviewVolumeWeightAuthoring.py`.
- Never author `xformOps` on RigExec prims; never touch `default:*` channels (the evaluator does not read them).
- Commits: only add the files named in the task; the working tree carries unrelated user changes. Commit message trailer:
  `Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>` and `Claude-Session: https://claude.ai/code/session_014a1ZRa9PujUhqJSZ5qWNYB`.

---

### Task 1: `rigExecUndo.py` — attribute snapshots and the undo stack

**Files:**
- Create: `plugin/rigExecUsdview/rigExecUndo.py`
- Create: `tests/python/test_rigexec_undo.py`
- Modify: `CMakeLists.txt:312-330` (register the test in the python-test block)
- Create: `bin/run_python_tests.sh`

**Interfaces:**
- Consumes: nothing from other tasks.
- Produces:
  - `AttributeSnapshot.Capture(layer: Sdf.Layer, specPath: Sdf.Path) -> AttributeSnapshot`; `snapshot.Restore()`; `snapshot == other` compares content; fields `exists`, `default` (value or `None`), `hasDefault`, `spline` (`Ts.Spline` or `None`), `timeSamples` (`dict`), `typeName` (`Sdf.ValueTypeName` or `None`), `variability`.
  - `Edit(label: str, entries: list[EditEntry])`, `EditEntry(layer, specPath, before, after)`.
  - `UndoStack()`: `Push(edit)`, `Undo() -> bool`, `Redo() -> bool`, `CanUndo()`, `CanRedo()`, `UndoText()`, `RedoText()`, `Clear()`, `AddListener(fn)` (fn called with no args after every change), `LIMIT = 200`.
  - `EditRecorder(stage, attrPaths: list[Sdf.Path])`: `Begin()`, `Commit(label) -> Edit | None`, `Abort()`.

- [ ] **Step 1: Write the failing test**

`tests/python/test_rigexec_undo.py`:

```python
#!/usr/bin/env python
"""
Headless test for plugin/rigExecUsdview/rigExecUndo.py.

Runs without Qt and without a usdview: the undo stack snapshots Sdf
attribute specs, so everything it does is observable on an in-memory
stage. Usage: test_rigexec_undo.py [<generated schema resources dir>]
"""
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(
    os.path.join(_HERE, "..", "..", "plugin", "rigExecUsdview")))

from pxr import Plug, Sdf, Ts, Usd  # noqa: E402

import rigExecUndo  # noqa: E402


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _RegisterSchema():
    if len(sys.argv) > 1:
        Plug.Registry().RegisterPlugins(sys.argv[1])
    _Check(Usd.SchemaRegistry().IsConcrete("RigExecControl"),
           "RigExecControl schema is not registered; pass the generated "
           "resources dir (build/usd/rigExecSchema/resources)")


def _Stage():
    stage = Usd.Stage.CreateInMemory()
    stage.DefinePrim("/Rig", "RigExecRoot")
    stage.DefinePrim("/Rig/Ctl", "RigExecControl")
    return stage


def _Knot(time, value):
    knot = Ts.Knot(typeName="double")
    knot.SetTime(time)
    knot.SetValue(value)
    knot.SetNextInterpolation(Ts.InterpCurve)
    return knot


def TestSnapshotAbsentSpec():
    stage = _Stage()
    layer = stage.GetRootLayer()
    path = Sdf.Path("/Rig/Ctl.avars:tx")
    before = rigExecUndo.AttributeSnapshot.Capture(layer, path)
    _Check(not before.exists, "unauthored attribute must snapshot as absent")
    stage.GetAttributeAtPath(path).Set(3.0)
    _Check(layer.GetAttributeAtPath(path) is not None, "spec authored")
    before.Restore()
    _Check(layer.GetAttributeAtPath(path) is None,
           "restoring an absent snapshot must remove the spec")


def TestSnapshotDefaultAndTimeSamples():
    stage = _Stage()
    layer = stage.GetRootLayer()
    path = Sdf.Path("/Rig/Ctl.avars:ty")
    attr = stage.GetAttributeAtPath(path)
    attr.Set(1.0)
    attr.Set(5.0, 10.0)
    attr.Set(7.0, 20.0)
    snap = rigExecUndo.AttributeSnapshot.Capture(layer, path)
    _Check(snap.exists and snap.hasDefault and snap.default == 1.0,
           "default captured")
    _Check(snap.timeSamples == {10.0: 5.0, 20.0: 7.0}, "samples captured")
    attr.Set(2.0)
    attr.Set(9.0, 10.0)
    attr.Set(9.0, 30.0)
    snap.Restore()
    _Check(attr.Get() == 1.0, "default restored")
    _Check(layer.ListTimeSamplesForPath(path) == [10.0, 20.0],
           "extra sample removed: %s" % layer.ListTimeSamplesForPath(path))
    _Check(attr.Get(10.0) == 5.0, "sample value restored")
    _Check(snap == rigExecUndo.AttributeSnapshot.Capture(layer, path),
           "round trip is content-equal")


def TestSnapshotSpline():
    stage = _Stage()
    layer = stage.GetRootLayer()
    path = Sdf.Path("/Rig/Ctl.avars:rz")
    attr = stage.GetAttributeAtPath(path)
    spline = Ts.Spline("double")
    spline.SetKnot(_Knot(1.0, 10.0))
    attr.SetSpline(spline)
    snap = rigExecUndo.AttributeSnapshot.Capture(layer, path)
    _Check(snap.spline is not None and snap.spline == spline,
           "spline captured by value")
    spline.SetKnot(_Knot(2.0, 20.0))
    attr.SetSpline(spline)
    _Check(len(attr.GetSpline().GetKnots()) == 2, "second knot authored")
    snap.Restore()
    _Check(len(attr.GetSpline().GetKnots()) == 1,
           "spline restored to the captured single knot")
    _Check(abs(attr.Get(1.0) - 10.0) < 1e-9, "knot value restored")
    # A snapshot with no spline must clear one that appeared later.
    snapNoSpline = rigExecUndo.AttributeSnapshot.Capture(
        layer, Sdf.Path("/Rig/Ctl.avars:rx"))
    stage.GetAttributeAtPath("/Rig/Ctl.avars:rx").SetSpline(spline)
    snapNoSpline.Restore()
    _Check(layer.GetAttributeAtPath("/Rig/Ctl.avars:rx") is None,
           "absent spec restored to absent even after a spline write")


def TestUndoStack():
    stage = _Stage()
    layer = stage.GetRootLayer()
    path = Sdf.Path("/Rig/Ctl.avars:tx")
    attr = stage.GetAttributeAtPath(path)
    stack = rigExecUndo.UndoStack()
    events = []
    stack.AddListener(lambda: events.append(1))
    _Check(not stack.CanUndo() and not stack.CanRedo(), "empty stack")
    before = rigExecUndo.AttributeSnapshot.Capture(layer, path)
    attr.Set(4.0)
    after = rigExecUndo.AttributeSnapshot.Capture(layer, path)
    stack.Push(rigExecUndo.Edit(
        "Translate", [rigExecUndo.EditEntry(layer, path, before, after)]))
    _Check(stack.CanUndo() and stack.UndoText() == "Translate", "pushed")
    _Check(len(events) == 1, "listener notified on push")
    _Check(stack.Undo(), "undo returns True")
    _Check(layer.GetAttributeAtPath(path) is None, "undo removed the spec")
    _Check(stack.CanRedo() and stack.RedoText() == "Translate", "redo")
    _Check(stack.Redo(), "redo returns True")
    _Check(attr.Get() == 4.0, "redo restored the value")
    stack.Undo()
    attr.Set(8.0)
    stack.Push(rigExecUndo.Edit("Other", [rigExecUndo.EditEntry(
        layer, path, before,
        rigExecUndo.AttributeSnapshot.Capture(layer, path))]))
    _Check(not stack.CanRedo(), "a push clears the redo branch")
    _Check(not stack.Redo(), "redo on an empty branch returns False")
    for i in range(rigExecUndo.UndoStack.LIMIT + 5):
        stack.Push(rigExecUndo.Edit("N%d" % i, []))
    count = 0
    while stack.Undo():
        count += 1
    _Check(count == rigExecUndo.UndoStack.LIMIT,
           "stack is bounded to LIMIT entries, undid %d" % count)
    stack.Clear()
    _Check(not stack.CanUndo() and not stack.CanRedo(), "cleared")


def TestEditRecorder():
    stage = _Stage()
    stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
    session = stage.GetSessionLayer()
    tx = Sdf.Path("/Rig/Ctl.avars:tx")
    ty = Sdf.Path("/Rig/Ctl.avars:ty")
    recorder = rigExecUndo.EditRecorder(stage, [tx, ty])
    recorder.Begin()
    _Check(recorder.Commit("nothing") is None,
           "no change commits to None")
    recorder.Begin()
    stage.GetAttributeAtPath(tx).Set(2.5)
    edit = recorder.Commit("Translate Ctl")
    _Check(edit is not None and edit.label == "Translate Ctl", "edit made")
    _Check(len(edit.entries) == 1 and edit.entries[0].layer == session,
           "only the changed attribute is recorded, in the session layer")
    _Check(session.GetAttributeAtPath(tx) is not None
           and stage.GetRootLayer().GetAttributeAtPath(tx) is None,
           "write landed in the edit target layer only")
    recorder.Begin()
    stage.GetAttributeAtPath(ty).Set(9.0)
    recorder.Abort()
    _Check(session.GetAttributeAtPath(ty) is None,
           "abort restores the pre-drag state")


def main():
    _RegisterSchema()
    groups = [
        ("absent spec", TestSnapshotAbsentSpec),
        ("default + time samples", TestSnapshotDefaultAndTimeSamples),
        ("spline", TestSnapshotSpline),
        ("undo stack", TestUndoStack),
        ("edit recorder", TestEditRecorder),
    ]
    for name, fn in groups:
        fn()
        print("  ok: %s" % name)
    print("RIGEXEC_UNDO_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `. bin/_env.sh && "$PY" tests/python/test_rigexec_undo.py "$RIG/build/usd/rigExecSchema/resources"`
Expected: `ModuleNotFoundError: No module named 'rigExecUndo'`

- [ ] **Step 3: Implement `rigExecUndo.py`**

```python
#
# RigExec usdview undo: attribute-spec snapshots and a bounded undo stack.
#
# Deliberately Qt-free so the gizmo's undo semantics can be tested
# headlessly (the same rule volumeWeightUI.py applies above its Qt
# banner). The stack records what an edit did to the EDIT TARGET LAYER,
# not what it did to the composed stage: usdview starts with the session
# layer as its edit target (volumeWeightUI.py:1676-1690), so the value an
# artist sees can be a session override of a file value, and "undo" must
# put the session spec back exactly -- including removing it when it did
# not exist -- rather than authoring the file value into the session.
#
# A spline knot outranks a default in value resolution (the trap
# volumeWeightUI.SetVisibleAtTime documents), so a snapshot captures all
# three arms of an attribute spec: default, spline, and time samples.
#
from pxr import Sdf, Ts


class AttributeSnapshot(object):
    """
    The complete authored state of one attribute spec in one layer.

    Captured before and after a drag; Restore() puts the layer back to
    exactly the captured state. `exists` False means the spec was not
    authored in that layer at all, and Restore() removes it.
    """

    def __init__(self, layer, specPath):
        self.layer = layer
        self.specPath = Sdf.Path(specPath)
        self.exists = False
        self.typeName = None
        self.variability = Sdf.VariabilityVarying
        self.hasDefault = False
        self.default = None
        self.spline = None
        self.timeSamples = {}

    @classmethod
    def Capture(cls, layer, specPath):
        snap = cls(layer, specPath)
        spec = layer.GetAttributeAtPath(snap.specPath)
        if spec is None:
            return snap
        snap.exists = True
        snap.typeName = spec.typeName
        snap.variability = spec.variability
        snap.hasDefault = spec.HasDefaultValue()
        if snap.hasDefault:
            snap.default = spec.default
        if spec.HasSpline():
            # Copy: the live spline object is owned by the spec and would
            # change under us as the drag writes new knots.
            snap.spline = Ts.Spline(spec.GetSpline())
        snap.timeSamples = {
            t: layer.QueryTimeSample(snap.specPath, t)
            for t in layer.ListTimeSamplesForPath(snap.specPath)}
        return snap

    def _EnsureSpec(self):
        spec = self.layer.GetAttributeAtPath(self.specPath)
        if spec is not None:
            return spec
        primPath = self.specPath.GetPrimPath()
        primSpec = Sdf.CreatePrimInLayer(self.layer, primPath)
        return Sdf.AttributeSpec(
            primSpec, self.specPath.name, self.typeName, self.variability)

    def Restore(self):
        with Sdf.ChangeBlock():
            spec = self.layer.GetAttributeAtPath(self.specPath)
            if not self.exists:
                if spec is not None:
                    primSpec = spec.owner
                    primSpec.RemoveProperty(spec)
                return
            spec = self._EnsureSpec()
            if self.hasDefault:
                spec.default = self.default
            elif spec.HasDefaultValue():
                spec.ClearDefaultValue()
            if self.spline is not None:
                spec.SetSpline(Ts.Spline(self.spline))
            elif spec.HasSpline():
                spec.ClearSpline()
            for t in list(self.layer.ListTimeSamplesForPath(self.specPath)):
                if t not in self.timeSamples:
                    self.layer.EraseTimeSample(self.specPath, t)
            for t, value in self.timeSamples.items():
                self.layer.SetTimeSample(self.specPath, t, value)

    def __eq__(self, other):
        if not isinstance(other, AttributeSnapshot):
            return NotImplemented
        return (self.exists == other.exists
                and self.hasDefault == other.hasDefault
                and self.default == other.default
                and self.spline == other.spline
                and self.timeSamples == other.timeSamples)

    def __ne__(self, other):
        result = self.__eq__(other)
        return result if result is NotImplemented else not result


class EditEntry(object):
    def __init__(self, layer, specPath, before, after):
        self.layer = layer
        self.specPath = Sdf.Path(specPath)
        self.before = before
        self.after = after


class Edit(object):
    """One undoable unit: a label and the per-attribute before/after."""

    def __init__(self, label, entries):
        self.label = label
        self.entries = list(entries)

    def _Apply(self, useAfter):
        for entry in self.entries:
            if entry.layer is None or entry.layer.expired:
                continue
            (entry.after if useAfter else entry.before).Restore()

    def Undo(self):
        self._Apply(useAfter=False)

    def Redo(self):
        self._Apply(useAfter=True)


class UndoStack(object):
    """A bounded linear undo stack; a push discards the redo branch."""

    LIMIT = 200

    def __init__(self):
        self._undo = []
        self._redo = []
        self._listeners = []

    def AddListener(self, fn):
        self._listeners.append(fn)

    def _Notify(self):
        for fn in list(self._listeners):
            fn()

    def Push(self, edit):
        self._undo.append(edit)
        del self._undo[:-self.LIMIT]
        self._redo = []
        self._Notify()

    def CanUndo(self):
        return bool(self._undo)

    def CanRedo(self):
        return bool(self._redo)

    def UndoText(self):
        return self._undo[-1].label if self._undo else ""

    def RedoText(self):
        return self._redo[-1].label if self._redo else ""

    def Undo(self):
        if not self._undo:
            return False
        edit = self._undo.pop()
        edit.Undo()
        self._redo.append(edit)
        self._Notify()
        return True

    def Redo(self):
        if not self._redo:
            return False
        edit = self._redo.pop()
        edit.Redo()
        self._undo.append(edit)
        self._Notify()
        return True

    def Clear(self):
        self._undo = []
        self._redo = []
        self._Notify()


class EditRecorder(object):
    """
    Brackets one drag: Begin() snapshots the named attributes in the
    current edit target's layer, Commit() returns the Edit (None when
    nothing changed), Abort() restores the snapshots.
    """

    def __init__(self, stage, attrPaths):
        self._stage = stage
        self._paths = [Sdf.Path(p) for p in attrPaths]
        self._layer = None
        self._before = {}

    def Begin(self):
        target = self._stage.GetEditTarget()
        self._layer = target.GetLayer()
        self._before = {}
        for path in self._paths:
            specPath = target.MapToSpecPath(path)
            self._before[path] = AttributeSnapshot.Capture(
                self._layer, specPath)

    def Commit(self, label):
        entries = []
        for path, before in self._before.items():
            after = AttributeSnapshot.Capture(self._layer, before.specPath)
            if after != before:
                entries.append(
                    EditEntry(self._layer, before.specPath, before, after))
        self._before = {}
        if not entries:
            return None
        return Edit(label, entries)

    def Abort(self):
        for before in self._before.values():
            before.Restore()
        self._before = {}
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `. bin/_env.sh && "$PY" tests/python/test_rigexec_undo.py "$RIG/build/usd/rigExecSchema/resources"`
Expected: five `ok:` lines then `RIGEXEC_UNDO_OK (5 groups)`. If `Sdf.AttributeSpec(...)` or `spec.owner` / `RemoveProperty` raise, check the binding names with `python -c "from pxr import Sdf; help(Sdf.PrimSpec.RemoveProperty)"` and adapt; the behaviour (spec removed) is what the test pins.

- [ ] **Step 5: Register the test and add the shell runner**

In `CMakeLists.txt`, inside the `if (RIGEXEC_BUILD_PYTHON AND RIGEXEC_BUILD_TESTS AND Python3_FOUND)` block, after `testRigExecPythonSchemaAuthoring` is added and BEFORE the `foreach(_python_test ...)`, add:

```cmake
    # Headless usdview-plugin tests (Qt-free modules under plugin/).
    # testGizmoMath and testGizmoScreen are written by later tasks; they
    # are registered here so CMakeLists.txt is touched by one task only.
    add_test(NAME testRigExecUndo
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_rigexec_undo.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testGizmoMath
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_gizmo_math.py"
            "${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
    add_test(NAME testGizmoScreen
        COMMAND ${Python3_EXECUTABLE}
            "${CMAKE_CURRENT_SOURCE_DIR}/tests/python/test_gizmo_screen.py")
```

and extend the foreach list: `foreach(_python_test testRigExecPython testRigExecPythonSchemaAuthoring testRigExecUndo testGizmoMath testGizmoScreen)`; also append the plugin dir to that test's PYTHONPATH by adding, after the existing `set_tests_properties ... ENVIRONMENT` line inside the loop:

```cmake
        set_property(TEST ${_python_test} APPEND PROPERTY ENVIRONMENT
            "PXR_PLUGINPATH_NAME=${CMAKE_CURRENT_BINARY_DIR}/usd/rigExecSchema/resources")
```

(The test inserts `plugin/rigExecUsdview` on `sys.path` itself.)

Create `bin/run_python_tests.sh` (chmod +x):

```bash
#!/bin/bash
# bin/run_python_tests.sh -- headless Qt-free tests for the usdview plugin
# modules (undo stack, gizmo math, gizmo screen helpers).
#
# Usage: bin/run_python_tests.sh [test_name ...]   (default: all)
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]:-$0}")/_env.sh"
rigexec_require_python

# The native evaluator binding is optional (built in build-python, not
# build); test_gizmo_math compares against it when importable.
export PYTHONPATH="$RIG/build-python/python:$PYTHONPATH"
SCHEMA="$RIG/build/usd/rigExecSchema/resources"

TESTS=("$@")
if [ ${#TESTS[@]} -eq 0 ]; then
    TESTS=(test_rigexec_undo test_gizmo_math test_gizmo_screen)
fi
for t in "${TESTS[@]}"; do
    echo "== $t"
    "$PY" "$RIG/tests/python/$t.py" "$SCHEMA"
done
```

Run: `bin/run_python_tests.sh test_rigexec_undo` → `RIGEXEC_UNDO_OK`.

- [ ] **Step 6: Commit**

```bash
git add plugin/rigExecUsdview/rigExecUndo.py tests/python/test_rigexec_undo.py bin/run_python_tests.sh CMakeLists.txt
git commit -m "usdview: attribute-spec undo stack for viewport tools" -m "Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>" -m "Claude-Session: https://claude.ai/code/session_014a1ZRa9PujUhqJSZ5qWNYB"
```

---

### Task 2: `gizmoMath.py` part A — Euler helpers and the rig frame replica

**Files:**
- Create: `plugin/rigExecUsdview/gizmoMath.py`
- Create: `tests/python/test_gizmo_math.py`

**Interfaces:**
- Consumes: nothing from other tasks.
- Produces (all in `gizmoMath`):
  - `ROTATION_ORDERS = ("XYZ", "XZY", "YXZ", "YZX", "ZXY", "ZYX")`
  - `NormalizeAvarScale(value: float) -> float`
  - `RotationFromEuler(order: str, rx, ry, rz) -> Gf.Matrix4d`
  - `ComposeAvarMatrix(tx, ty, tz, sx, sy, sz, rx, ry, rz, rspin, order) -> Gf.Matrix4d`
  - `DecomposeEuler(matrix: Gf.Matrix4d, order: str, hint=None) -> (rx, ry, rz)` degrees
  - `IsRigXformable(prim) -> bool`, `FindRigRoot(prim) -> Usd.Prim | None`
  - `SolverPosedPaths(rigRoot) -> set[Sdf.Path]`
  - `ScalarAvar(prim, name, time, fallback) -> float`
  - `RestSpace(prim, time) -> Gf.Matrix4d`, `AvarsMatrix(prim, time) -> Gf.Matrix4d`
  - `class RigFrames` with attributes `prim, rigRoot, reason (str, "" when editable), rest, posed, P, Q, restLocal, parentRest, parentPosed, assetToWorld` (all `Gf.Matrix4d`, asset space except `assetToWorld`)
  - `ComputeRigFrames(stage, prim, time, solverPosed=None) -> RigFrames`

- [ ] **Step 1: Write the failing test**

`tests/python/test_gizmo_math.py`:

```python
#!/usr/bin/env python
"""
Headless test for plugin/rigExecUsdview/gizmoMath.py.

Usage: test_gizmo_math.py [<generated schema resources dir>]

The frame replica is checked against the native evaluator when the
_rigexec binding is importable (build-python/python on PYTHONPATH); the
check is reported as skipped otherwise, never silently passed.
"""
import math
import os
import random
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(
    os.path.join(_HERE, "..", "..", "plugin", "rigExecUsdview")))

from pxr import Gf, Plug, Sdf, Usd, UsdGeom  # noqa: E402

import gizmoMath  # noqa: E402


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _Close(a, b, tol=1e-6):
    return abs(a - b) <= tol


def _MatClose(a, b, tol=1e-6):
    return all(_Close(a[r][c], b[r][c], tol)
               for r in range(4) for c in range(4))


def _RegisterSchema():
    if len(sys.argv) > 1:
        Plug.Registry().RegisterPlugins(sys.argv[1])
    _Check(Usd.SchemaRegistry().IsConcrete("RigExecControl"),
           "RigExecControl schema is not registered")


def _Rot(axis, deg):
    m = Gf.Matrix4d(1.0)
    m.SetRotate(Gf.Rotation(axis, deg))
    return m


def TestEulerRoundTrip():
    rng = random.Random(7)
    for order in gizmoMath.ROTATION_ORDERS:
        for _ in range(60):
            angles = [rng.uniform(-170.0, 170.0) for _ in range(3)]
            m = gizmoMath.RotationFromEuler(order, *angles)
            back = gizmoMath.DecomposeEuler(m, order, hint=angles)
            _Check(all(_Close(a, b, 1e-6) for a, b in zip(angles, back)),
                   "%s: %s -> %s" % (order, angles, back))
            m2 = gizmoMath.RotationFromEuler(order, *back)
            _Check(_MatClose(m, m2), "%s matrix round trip" % order)
    # No hint: the matrix still round-trips (angles may differ).
    m = gizmoMath.RotationFromEuler("ZXY", 200.0, 30.0, -100.0)
    back = gizmoMath.DecomposeEuler(m, "ZXY")
    _Check(_MatClose(m, gizmoMath.RotationFromEuler("ZXY", *back)),
           "hintless decomposition reproduces the matrix")
    # Gimbal lock: beta = +-90 keeps the hint's alpha and still matches.
    for beta in (90.0, -90.0):
        m = gizmoMath.RotationFromEuler("XYZ", 25.0, beta, 40.0)
        back = gizmoMath.DecomposeEuler(m, "XYZ", hint=(25.0, beta, 40.0))
        _Check(_MatClose(m, gizmoMath.RotationFromEuler("XYZ", *back), 1e-5),
               "gimbal lock %s -> %s" % (beta, back))
    # The convention matches _ComposeAvars: XYZ is Rx * Ry * Rz (row-vector).
    m = gizmoMath.RotationFromEuler("XYZ", 10.0, 20.0, 30.0)
    ref = _Rot(Gf.Vec3d(1, 0, 0), 10.0) * _Rot(Gf.Vec3d(0, 1, 0), 20.0) \
        * _Rot(Gf.Vec3d(0, 0, 1), 30.0)
    _Check(_MatClose(m, ref), "row-vector order Rx*Ry*Rz")


def TestComposeAvarMatrix():
    m = gizmoMath.ComposeAvarMatrix(1, 2, 3, 2, 2, 2, 0, 0, 90, 0, "XYZ")
    p = Gf.Vec3d(1, 0, 0) * m
    # scale 2 -> (2,0,0); rotate 90 about Z -> (0,2,0); translate -> (1,4,3)
    _Check(all(_Close(p[i], e, 1e-9) for i, e in enumerate((1, 4, 3))),
           "S*R*T order: %s" % p)
    _Check(gizmoMath.NormalizeAvarScale(0.0) == 1e-4, "zero scale floors")
    _Check(gizmoMath.NormalizeAvarScale(-1e-9) == -1e-4, "signed floor")
    _Check(gizmoMath.NormalizeAvarScale(float("nan")) == 1.0, "nan -> 1")
    _Check(gizmoMath.NormalizeAvarScale(3.0) == 3.0, "ordinary passes")
    spin = gizmoMath.ComposeAvarMatrix(0, 0, 0, 1, 1, 1, 0, 0, 0, 90, "XYZ")
    q = Gf.Vec3d(0, 1, 0) * spin
    _Check(_Close(q[2], 1.0, 1e-9), "rspin rotates about local +X")


def _ChainStage():
    stage = Usd.Stage.CreateInMemory()
    asset = UsdGeom.Xform.Define(stage, "/Asset")
    asset.AddTranslateOp().Set(Gf.Vec3d(100, 0, 0))
    stage.DefinePrim("/Asset/Rig", "RigExecRoot")
    stage.DefinePrim("/Asset/Rig/Controls", "Scope")
    parent = stage.DefinePrim("/Asset/Rig/Controls/Parent", "RigExecControl")
    child = stage.DefinePrim(
        "/Asset/Rig/Controls/Parent/Child", "RigExecControl")
    space = _Rot(Gf.Vec3d(0, 0, 1), 90.0)
    space.SetTranslateOnly(Gf.Vec3d(0, 5, 0))
    parent.GetAttribute("rest:space").Set(space)
    parent.GetAttribute("avars:tx").Set(1.0)
    parent.GetAttribute("avars:rz").Set(30.0)
    parent.GetAttribute("avars:sx").Set(2.0)
    child.GetAttribute("rest:space").Set(space)
    child.GetAttribute("rest:tx").Set(3.0)
    child.GetAttribute("rest:ry").Set(45.0)
    child.GetAttribute("avars:ty").Set(0.5)
    child.GetAttribute("avars:rx").Set(20.0)
    child.GetAttribute("avars:rotationOrder").Set("ZYX")
    return stage, parent, child


def TestRigFramesReplica():
    stage, parent, child = _ChainStage()
    t = Usd.TimeCode.Default()
    pf = gizmoMath.ComputeRigFrames(stage, parent, t)
    cf = gizmoMath.ComputeRigFrames(stage, child, t)
    _Check(pf.reason == "" and cf.reason == "", "chain is editable")
    _Check(pf.rigRoot.GetPath() == Sdf.Path("/Asset/Rig"), "root found")
    # Parent: posed = avars * rest (no ancestor xformable).
    expectedRest = gizmoMath.RestSpace(parent, t)
    _Check(_MatClose(pf.rest, expectedRest), "parent rest")
    _Check(_MatClose(pf.P, expectedRest), "parent P == rest")
    _Check(_MatClose(pf.posed, gizmoMath.AvarsMatrix(parent, t) * pf.rest),
           "parent posed = avars * rest")
    # Child: world = avars * rest * parentRest^-1 * parentPosed.
    expectedP = cf.rest * pf.rest.GetInverse() * pf.posed
    _Check(_MatClose(cf.P, expectedP), "child P")
    _Check(_MatClose(cf.posed, gizmoMath.AvarsMatrix(child, t) * expectedP),
           "child posed")
    _Check(_Close(cf.assetToWorld[3][0], 100.0), "asset placement")
    _Check(pf.rest.GetOrthonormalized(False) == pf.rest
           or _MatClose(pf.rest, pf.rest.GetOrthonormalized(False)),
           "rest is orthonormal")
    # Native comparison when the binding is available.
    try:
        import _rigexec
    except ImportError:
        print("  (native _rigexec comparison skipped: module not on "
              "PYTHONPATH; run via bin/run_python_tests.sh)")
        return
    rig = _rigexec.Rig(stage, "/Asset/Rig")
    rig.compile()
    pose = rig.evaluate(-1.0)
    for prim, frames in ((parent, pf), (child, cf)):
        native = pose.control_frame(str(prim.GetPath())).to_matrix4()
        nm = Gf.Matrix4d(*native)
        _Check(_MatClose(nm, frames.posed, 1e-5),
               "%s replica vs native:\n%s\n%s" % (prim.GetName(), nm,
                                                  frames.posed))


def TestRigFramesReasons():
    stage, parent, child = _ChainStage()
    t = Usd.TimeCode.Default()
    solver = stage.DefinePrim("/Asset/Rig/Solvers/Fk", "RigExecFkChain")
    joint = stage.DefinePrim("/Asset/Rig/Joints/J", "RigExecJoint")
    solver.GetRelationship("rigExec:joints").SetTargets([joint.GetPath()])
    jf = gizmoMath.ComputeRigFrames(stage, joint, t)
    _Check("solver" in jf.reason, "solver-posed joint refused: %r"
           % jf.reason)
    childJoint = stage.DefinePrim("/Asset/Rig/Joints/J/K", "RigExecJoint")
    kf = gizmoMath.ComputeRigFrames(stage, childJoint, t)
    _Check(kf.reason != "", "child of a solver-posed joint is refused")
    posed = child.GetAttribute("posed:space")
    posed.Set(Gf.Matrix4d(2.0))
    _Check("posed:space" in gizmoMath.ComputeRigFrames(stage, child, t).reason,
           "authored posed:space refused")
    posed.Set(Gf.Matrix4d(1.0))
    _Check(gizmoMath.ComputeRigFrames(stage, child, t).reason == "",
           "identity posed:space is fine")
    posed.AddConnection(parent.GetAttribute("posed:space").GetPath())
    _Check("connected" in gizmoMath.ComputeRigFrames(stage, child, t).reason,
           "connected posed:space refused")
    loose = stage.DefinePrim("/Loose", "RigExecControl")
    _Check("RigExecRoot" in gizmoMath.ComputeRigFrames(stage, loose, t).reason,
           "control outside a rig refused")


def main():
    _RegisterSchema()
    groups = [
        ("euler round trip", TestEulerRoundTrip),
        ("compose avars", TestComposeAvarMatrix),
        ("rig frames replica", TestRigFramesReplica),
        ("rig frames reasons", TestRigFramesReasons),
    ]
    for name, fn in groups:
        fn()
        print("  ok: %s" % name)
    print("RIGEXEC_GIZMO_MATH_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `bin/run_python_tests.sh test_gizmo_math`
Expected: `ModuleNotFoundError: No module named 'gizmoMath'`

- [ ] **Step 3: Implement part A of `gizmoMath.py`**

```python
#
# RigExec usdview gizmo math: the evaluator's avar composition replicated
# in Python, edit targets that map world-space gizmo deltas back onto
# avars / rest offsets / xformOps, and the writer that decides whether a
# value lands on the animation (spline knot at the current frame) or on
# the default.
#
# Qt-free by design (see volumeWeightUI.py's banner rule): everything here
# is exercised headlessly by tests/python/test_gizmo_math.py, including a
# comparison against the native evaluator through the _rigexec binding.
#
# Conventions (libs/rigExec/computations.cpp:144-175, 196-215, 284-285),
# row-vector, leftmost factor applies first:
#
#   avars = S * R(rotationOrder) * Rspin(+X) * T
#   rest  = orthonormalize(compose(rest:t, rest:r, XYZ) * rest:space)
#   posed = avars * rest * parentRest^-1 * parentPosed
#
# where "parent" is the nearest namespace-ancestor RigExecXformable and the
# whole rig is in ASSET space: the world transform of the RigExecRoot's
# parent prim places it (libs/rigExec/rigEvaluator.h:79-84).
#
import math

from pxr import Gf, Sdf, Tf, Usd, UsdGeom

ROTATION_ORDERS = ("XYZ", "XZY", "YXZ", "YZX", "ZXY", "ZYX")

_AXES = (Gf.Vec3d(1, 0, 0), Gf.Vec3d(0, 1, 0), Gf.Vec3d(0, 0, 1))
_CYCLIC = ("XYZ", "YZX", "ZXY")
_IDENTITY = Gf.Matrix4d(1.0)

# Attribute names, kept as constants so the targets and the tests agree.
AVAR_T = ("avars:tx", "avars:ty", "avars:tz")
AVAR_R = ("avars:rx", "avars:ry", "avars:rz")
AVAR_S = ("avars:sx", "avars:sy", "avars:sz")
AVAR_RSPIN = "avars:rspin"
AVAR_ORDER = "avars:rotationOrder"
REST_T = ("rest:tx", "rest:ty", "rest:tz")
REST_R = ("rest:rx", "rest:ry", "rest:rz")
REST_SPACE = "rest:space"
POSED_SPACE = "posed:space"


# ---------------------------------------------------------------------------
# Scalars and rotations
# ---------------------------------------------------------------------------

def NormalizeAvarScale(value):
    """
    The evaluator's floor (libs/rigExecMath/avarScale.h:34-44): finite
    magnitudes below 1e-4 become signed 1e-4, non-finite becomes 1.
    Applied before writing so the stage holds what the viewport shows.
    """
    if not math.isfinite(value):
        return 1.0
    if abs(value) < 1e-4:
        return math.copysign(1e-4, value) if value != 0.0 else 1e-4
    return value


def _NormalizeOrder(order):
    order = str(order or "XYZ").upper()
    return order if order in ROTATION_ORDERS else "XYZ"


def _AxisRotation(axisIndex, degrees):
    m = Gf.Matrix4d(1.0)
    m.SetRotate(Gf.Rotation(_AXES[axisIndex], degrees))
    return m


def RotationFromEuler(order, rx, ry, rz):
    """Rx/Ry/Rz applied in `order` sequence, row-vector (X first for XYZ)."""
    angles = (rx, ry, rz)
    m = Gf.Matrix4d(1.0)
    for axis in _NormalizeOrder(order):
        index = "XYZ".index(axis)
        if angles[index] != 0.0:
            m = m * _AxisRotation(index, angles[index])
    return m


def ComposeAvarMatrix(tx, ty, tz, sx, sy, sz, rx, ry, rz, rspin, order):
    """Mirror of _ComposeAvars: S * R(order) * Rspin * T."""
    m = Gf.Matrix4d(1.0)
    m.SetScale(Gf.Vec3d(NormalizeAvarScale(sx), NormalizeAvarScale(sy),
                        NormalizeAvarScale(sz)))
    m = m * RotationFromEuler(order, rx, ry, rz)
    if rspin != 0.0:
        m = m * _AxisRotation(0, rspin)
    t = Gf.Matrix4d(1.0)
    t.SetTranslate(Gf.Vec3d(tx, ty, tz))
    return m * t


def _Unwrap(angle, hint):
    """`angle` shifted by a multiple of 360 to land nearest `hint`."""
    return angle + 360.0 * round((hint - angle) / 360.0)


def DecomposeEuler(matrix, order, hint=None):
    """
    Euler angles (degrees) whose RotationFromEuler(order, ...) reproduces
    the rotation part of `matrix`. Of the two solutions the one nearest
    `hint` (a 3-tuple of degrees) wins, and each angle is unwrapped to
    the hint's turn, so a drag never flips 180 degrees between events.

    Derivation: with column-vector M = R^T the product is
    M = Rk(g) * Rj(b) * Ri(a) for order (i, j, k); parity eps is +1 for
    cyclic orders. Then sin(b) = -eps*M[k][i], a = atan2(eps*M[k][j],
    M[k][k]) and g = atan2(eps*M[j][i], M[i][i]) -- verified for XYZ and
    XZY by hand, and for all six orders by the round-trip test.
    """
    order = _NormalizeOrder(order)
    i, j, k = ("XYZ".index(axis) for axis in order)
    eps = 1.0 if order in _CYCLIC else -1.0
    rot = Gf.Matrix4d(matrix).GetOrthonormalized(False)

    def M(r, c):
        # Column-vector element = transpose of the row-vector matrix.
        return rot[c][r]

    cosBeta = math.hypot(M(k, j), M(k, k))
    beta = math.atan2(-eps * M(k, i), cosBeta)
    if cosBeta > 1e-9:
        alpha = math.atan2(eps * M(k, j), M(k, k))
        gamma = math.atan2(eps * M(j, i), M(i, i))
    else:
        # Gimbal lock: alpha is free; keep the hint's and solve gamma from
        # the residual rotation about axis k.
        alpha = math.radians(hint[i]) if hint is not None else 0.0
        partial = _AxisRotation(i, math.degrees(alpha)) * \
            _AxisRotation(j, math.degrees(beta))
        residual = partial.GetInverse() * rot
        gamma = math.atan2(eps * residual[i][j], residual[i][i])

    first = [0.0, 0.0, 0.0]
    first[i], first[j], first[k] = (
        math.degrees(alpha), math.degrees(beta), math.degrees(gamma))
    if hint is None:
        return tuple(first)
    second = [0.0, 0.0, 0.0]
    second[i] = first[i] + 180.0
    second[j] = 180.0 - first[j]
    second[k] = first[k] + 180.0
    best = None
    for candidate in (first, second):
        unwrapped = [_Unwrap(c, h) for c, h in zip(candidate, hint)]
        distance = sum(abs(u - h) for u, h in zip(unwrapped, hint))
        if best is None or distance < best[0]:
            best = (distance, unwrapped)
    return tuple(best[1])


# ---------------------------------------------------------------------------
# Rig frames (asset space)
# ---------------------------------------------------------------------------

def IsRigXformable(prim):
    xformable = Tf.Type.FindByName("RigExecXformable")
    if xformable.isUnknown:
        return prim.GetTypeName() in ("RigExecControl", "RigExecJoint")
    return prim.IsA(xformable)


def FindRigRoot(prim):
    """The enclosing RigExecRoot, or None."""
    parent = prim.GetParent()
    while parent and not parent.IsPseudoRoot():
        if parent.GetTypeName() == "RigExecRoot":
            return parent
        parent = parent.GetParent()
    return None


def _FindParentXformable(prim, rigRoot):
    parent = prim.GetParent()
    while parent and parent != rigRoot and not parent.IsPseudoRoot():
        if IsRigXformable(parent):
            return parent
        parent = parent.GetParent()
    return None


def SolverPosedPaths(rigRoot):
    """
    Every prim a solver poses: the union of all rigExec:joints targets
    under the rig. The evaluator injects those joints' frames as value
    overrides (computations.cpp:223-227), so their avars are inert.
    """
    paths = set()
    for prim in Usd.PrimRange(rigRoot):
        rel = prim.GetRelationship("rigExec:joints")
        if rel:
            paths.update(rel.GetTargets())
    return paths


def ScalarAvar(prim, name, time, fallback):
    attr = prim.GetAttribute(name)
    if attr:
        value = attr.Get(time)
        if value is not None:
            return float(value)
    return fallback


def _MatrixAttr(prim, name, time):
    attr = prim.GetAttribute(name)
    if attr:
        value = attr.Get(time)
        if value is not None:
            return Gf.Matrix4d(value)
    return Gf.Matrix4d(1.0)


def RestLocal(prim, time):
    """compose(rest:t, rest:r) -- XYZ order, no scale, no spin."""
    return ComposeAvarMatrix(
        ScalarAvar(prim, REST_T[0], time, 0.0),
        ScalarAvar(prim, REST_T[1], time, 0.0),
        ScalarAvar(prim, REST_T[2], time, 0.0),
        1.0, 1.0, 1.0,
        ScalarAvar(prim, REST_R[0], time, 0.0),
        ScalarAvar(prim, REST_R[1], time, 0.0),
        ScalarAvar(prim, REST_R[2], time, 0.0),
        0.0, "XYZ")


def RestSpace(prim, time):
    """Mirror of _JointRestSpace: orthonormalize(restLocal * rest:space)."""
    rest = RestLocal(prim, time) * _MatrixAttr(prim, REST_SPACE, time)
    return rest.GetOrthonormalized(False)


def AvarsMatrix(prim, time):
    order = prim.GetAttribute(AVAR_ORDER)
    orderValue = order.Get(time) if order else None
    return ComposeAvarMatrix(
        ScalarAvar(prim, AVAR_T[0], time, 0.0),
        ScalarAvar(prim, AVAR_T[1], time, 0.0),
        ScalarAvar(prim, AVAR_T[2], time, 0.0),
        ScalarAvar(prim, AVAR_S[0], time, 1.0),
        ScalarAvar(prim, AVAR_S[1], time, 1.0),
        ScalarAvar(prim, AVAR_S[2], time, 1.0),
        ScalarAvar(prim, AVAR_R[0], time, 0.0),
        ScalarAvar(prim, AVAR_R[1], time, 0.0),
        ScalarAvar(prim, AVAR_R[2], time, 0.0),
        ScalarAvar(prim, AVAR_RSPIN, time, 0.0),
        orderValue or "XYZ")


class RigFrames(object):
    """
    Everything the gizmo needs about one RigExecXformable at one time,
    in ASSET space (multiply by assetToWorld for world):

      rest       orthonormal rest frame (the evaluator's computeRestFrame)
      posed      avars * P   (the evaluator's computePointFrame)
      P          rest * parentRest^-1 * parentPosed: the frame the avars
                 are expressed in -- Pose mode edits happen relative to it
      Q          rest:space * parentRest^-1 * parentPosed: the frame the
                 rest offsets are expressed in -- Pivot mode edits
      restLocal  compose(rest:t, rest:r)
      reason     "" when the prim is editable through its avars, else why
                 not (solver-posed, posed:space authority, no rig root)
    """

    def __init__(self, prim):
        self.prim = prim
        self.rigRoot = None
        self.reason = ""
        self.rest = Gf.Matrix4d(1.0)
        self.posed = Gf.Matrix4d(1.0)
        self.P = Gf.Matrix4d(1.0)
        self.Q = Gf.Matrix4d(1.0)
        self.restLocal = Gf.Matrix4d(1.0)
        self.parentRest = Gf.Matrix4d(1.0)
        self.parentPosed = Gf.Matrix4d(1.0)
        self.assetToWorld = Gf.Matrix4d(1.0)


def ComputeRigFrames(stage, prim, time, solverPosed=None):
    frames = RigFrames(prim)
    frames.rigRoot = FindRigRoot(prim)
    if frames.rigRoot is None:
        frames.reason = "%s is not under a RigExecRoot" % prim.GetName()
        return frames
    if solverPosed is None:
        solverPosed = SolverPosedPaths(frames.rigRoot)

    parent = _FindParentXformable(prim, frames.rigRoot)
    if parent is not None:
        parentFrames = ComputeRigFrames(stage, parent, time, solverPosed)
        if parentFrames.reason:
            frames.reason = "parent %s: %s" % (
                parent.GetName(), parentFrames.reason)
        frames.parentRest = parentFrames.rest
        frames.parentPosed = parentFrames.posed

    if prim.GetPath() in solverPosed:
        frames.reason = ("%s is posed by a solver (rigExec:joints); its "
                         "avars are ignored" % prim.GetName())
    posed = prim.GetAttribute(POSED_SPACE)
    if posed:
        if posed.HasAuthoredConnections():
            frames.reason = ("%s has a connected posed:space; its avars "
                             "are ignored" % prim.GetName())
        elif _MatrixAttr(prim, POSED_SPACE, time) != _IDENTITY:
            frames.reason = ("%s has an authored posed:space; its avars "
                             "are ignored" % prim.GetName())

    frames.restLocal = RestLocal(prim, time)
    frames.rest = RestSpace(prim, time)
    toParent = frames.parentRest.GetInverse() * frames.parentPosed
    frames.P = frames.rest * toParent
    frames.Q = _MatrixAttr(prim, REST_SPACE, time) * toParent
    frames.posed = AvarsMatrix(prim, time) * frames.P

    assetRoot = frames.rigRoot.GetParent()
    if assetRoot and not assetRoot.IsPseudoRoot():
        frames.assetToWorld = UsdGeom.XformCache(time)\
            .GetLocalToWorldTransform(assetRoot)
    return frames
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `bin/run_python_tests.sh test_gizmo_math`
Expected: four `ok:` lines and `RIGEXEC_GIZMO_MATH_OK (4 groups)`, with no "skipped" line (the `_rigexec` module in `build-python/python` must import; if it fails to import because it is stale against the current libs, rebuild it with `cmake --build build-python` and re-run). If the native comparison fails only by a transpose, read `python/_rigexec.cpp:571-596` (`_FrameToMatrix`, `_Mat4ToVec`) and fix the test's matrix construction, not the replica. If it fails by values, the replica is wrong: re-read `libs/rigExec/computations.cpp:196-286` and fix the replica.

- [ ] **Step 5: Commit** (CMake registration is done by Task 1; do not edit CMakeLists.txt here)

```bash
git add plugin/rigExecUsdview/gizmoMath.py tests/python/test_gizmo_math.py
git commit -m "usdview: gizmo math -- Euler helpers and rig frame replica" -m "Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>" -m "Claude-Session: https://claude.ai/code/session_014a1ZRa9PujUhqJSZ5qWNYB"
```

---

### Task 3: `gizmoMath.py` part B — the writer and the edit targets

**Files:**
- Modify: `plugin/rigExecUsdview/gizmoMath.py` (append below part A)
- Modify: `tests/python/test_gizmo_math.py` (append test groups)

**Interfaces:**
- Consumes (Task 2): `ComposeAvarMatrix`, `RotationFromEuler`, `DecomposeEuler`, `NormalizeAvarScale`, `ComputeRigFrames`, `RigFrames`, `IsRigXformable`, `ScalarAvar`, the `AVAR_*`/`REST_*` constants.
- Produces:
  - `WRITE_ANIMATION = "animation"`, `WRITE_DEFAULT = "default"`, `CHANNELS_POSE = "pose"`, `CHANNELS_PIVOT = "pivot"`
  - `SetAnimated(attr, value, time)` (the `volumeWeightUI.SetAtTime` rule)
  - `class Writer(stage, time, mode)`: `Set(attr, value)`, `Warnings() -> list[str]`, `time`, `mode`
  - `class Target` base with: `kind` ("rig-pose" | "rig-pivot" | "xform-pose" | "xform-pivot"), `prim`, `label`, `supportsTranslate`, `supportsRotate`, `supportsScale`, `Refresh()`, `GizmoMatrix() -> Gf.Matrix4d` (world, orthonormal linear part), `AttributePaths() -> list[Sdf.Path]`, `BeginDrag()`, `ApplyTranslate(worldDelta: Gf.Vec3d)`, `ApplyRotate(worldAxis: Gf.Vec3d, degrees: float)`, `ApplyScale(axisIndex: int | None, factor: float)`.
  - `MakeTarget(stage, prim, channels, writer) -> (Target | None, reason: str)`

- [ ] **Step 1: Append the failing tests**

Append to `tests/python/test_gizmo_math.py` before `def main()`, and add the four new groups to `main()`'s list (`("writer", TestWriter), ("rig pose target", TestRigPoseTarget), ("rig pivot target", TestRigPivotTarget), ("xform targets", TestXformTargets)`):

```python
def TestWriter():
    stage, parent, child = _ChainStage()
    attr = child.GetAttribute("avars:tx")
    anim = gizmoMath.Writer(stage, Usd.TimeCode(1001.0),
                            gizmoMath.WRITE_ANIMATION)
    anim.Set(attr, 2.0)
    _Check(attr.HasSpline() and len(attr.GetSpline().GetKnots()) == 1,
           "animation mode writes a spline knot")
    _Check(_Close(attr.Get(Usd.TimeCode(1001.0)), 2.0), "knot value")
    anim.Set(attr, 3.0)
    _Check(len(attr.GetSpline().GetKnots()) == 1
           and _Close(attr.Get(Usd.TimeCode(1001.0)), 3.0),
           "re-writing the same frame updates the knot")
    _Check(anim.Warnings() == [], "no warnings in animation mode")
    default = gizmoMath.Writer(stage, Usd.TimeCode(1001.0),
                               gizmoMath.WRITE_DEFAULT)
    default.Set(attr, 9.0)
    _Check(stage.GetRootLayer().GetAttributeAtPath(
        attr.GetPath()).default == 9.0, "default mode writes the default")
    _Check(len(default.Warnings()) == 1
           and "avars:tx" in default.Warnings()[0],
           "default outranked by the spline is reported: %s"
           % default.Warnings())
    clean = child.GetAttribute("avars:ty")
    default.Set(clean, 1.0)
    _Check(not clean.HasSpline() and clean.Get() == 1.0, "plain default")
    # A vector attribute (xformOp) gets a time sample, not a spline.
    xf = UsdGeom.Xform.Define(stage, "/Asset/Box")
    op = xf.AddTranslateOp()
    anim.Set(op.GetAttr(), Gf.Vec3d(1, 2, 3))
    _Check(op.GetAttr().GetNumTimeSamples() == 1, "vec3 -> time sample")


def _Drag(target, fn):
    target.BeginDrag()
    fn()


def TestRigPoseTarget():
    stage, parent, child = _ChainStage()
    time = Usd.TimeCode.Default()
    writer = gizmoMath.Writer(stage, time, gizmoMath.WRITE_DEFAULT)
    target, reason = gizmoMath.MakeTarget(
        stage, child, gizmoMath.CHANNELS_POSE, writer)
    _Check(target is not None and reason == "", "child is a target")
    _Check(target.kind == "rig-pose" and target.supportsScale, "kind")
    before = gizmoMath.ComputeRigFrames(stage, child, time)
    gizmo = target.GizmoMatrix()
    origin = gizmo.ExtractTranslation()
    expectedOrigin = (before.posed * before.assetToWorld).ExtractTranslation()
    _Check(all(_Close(origin[i], expectedOrigin[i]) for i in range(3)),
           "gizmo sits at the posed world origin")
    # Translate by a world delta: the world origin moves by exactly that.
    delta = Gf.Vec3d(0.7, -1.3, 2.1)
    _Drag(target, lambda: target.ApplyTranslate(delta))
    after = gizmoMath.ComputeRigFrames(stage, child, time)
    moved = (after.posed * after.assetToWorld).ExtractTranslation()
    _Check(all(_Close(moved[i], expectedOrigin[i] + delta[i], 1e-6)
               for i in range(3)), "world translate maps onto avars:t")
    _Check(all(_Close(after.rest[r][c], before.rest[r][c])
               for r in range(4) for c in range(4)),
           "pose mode never touches rest")
    # Rotate about world Z by 30: the world linear part rotates by Rz(30).
    target.Refresh()
    base = gizmoMath.ComputeRigFrames(stage, child, time)
    baseWorld = (base.posed * base.assetToWorld).GetOrthonormalized(False)
    _Drag(target, lambda: target.ApplyRotate(Gf.Vec3d(0, 0, 1), 30.0))
    rotated = gizmoMath.ComputeRigFrames(stage, child, time)
    rotWorld = (rotated.posed * rotated.assetToWorld)\
        .GetOrthonormalized(False)
    expected = baseWorld * _Rot(Gf.Vec3d(0, 0, 1), 30.0)
    expected.SetTranslateOnly(rotWorld.ExtractTranslation())
    _Check(_MatClose(rotWorld, expected, 1e-6),
           "world rotate maps onto avars:r (order ZYX):\n%s\n%s"
           % (rotWorld, expected))
    _Check(_Close(child.GetAttribute("avars:rspin").Get(), 0.0),
           "rspin untouched")
    # Scale along local X by 1.5 multiplies avars:sx only.
    target.Refresh()
    _Drag(target, lambda: target.ApplyScale(0, 1.5))
    _Check(_Close(child.GetAttribute("avars:sx").Get(), 1.5), "sx scaled")
    _Check(_Close(child.GetAttribute("avars:sy").Get(), 1.0), "sy kept")
    target.Refresh()
    _Drag(target, lambda: target.ApplyScale(None, 2.0))
    _Check(_Close(child.GetAttribute("avars:sx").Get(), 3.0)
           and _Close(child.GetAttribute("avars:sz").Get(), 2.0),
           "uniform scale multiplies every axis from the drag base")
    target.Refresh()
    _Drag(target, lambda: target.ApplyScale(1, 0.0))
    _Check(_Close(child.GetAttribute("avars:sy").Get(), 1e-4),
           "scale floor applied on write")
    paths = target.AttributePaths()
    _Check(Sdf.Path("/Asset/Rig/Controls/Parent/Child.avars:tx") in paths
           and Sdf.Path("/Asset/Rig/Controls/Parent/Child.avars:sz") in paths
           and len(paths) == 9, "pose target owns the nine avars")
    # Refusals surface as reasons.
    joint = stage.DefinePrim("/Asset/Rig/Joints/J", "RigExecJoint")
    solver = stage.DefinePrim("/Asset/Rig/Solvers/Fk", "RigExecFkChain")
    solver.GetRelationship("rigExec:joints").SetTargets([joint.GetPath()])
    refused, reason = gizmoMath.MakeTarget(
        stage, joint, gizmoMath.CHANNELS_POSE, writer)
    _Check(refused is None and "solver" in reason, "solver-posed refused")
    child.GetAttribute("avars:rz").AddConnection(
        parent.GetAttribute("avars:rz").GetPath())
    refused, reason = gizmoMath.MakeTarget(
        stage, child, gizmoMath.CHANNELS_POSE, writer)
    _Check(refused is None and "avars:rz" in reason,
           "connected avar refused: %r" % reason)


def TestRigPivotTarget():
    stage, parent, child = _ChainStage()
    time = Usd.TimeCode.Default()
    writer = gizmoMath.Writer(stage, time, gizmoMath.WRITE_DEFAULT)
    target, reason = gizmoMath.MakeTarget(
        stage, child, gizmoMath.CHANNELS_PIVOT, writer)
    _Check(target is not None and target.kind == "rig-pivot", reason)
    _Check(target.supportsTranslate and target.supportsRotate
           and not target.supportsScale, "pivot: no scale")
    before = gizmoMath.ComputeRigFrames(stage, child, time)
    expectedOrigin = (before.restLocal * before.Q * before.assetToWorld)\
        .ExtractTranslation()
    origin = target.GizmoMatrix().ExtractTranslation()
    _Check(all(_Close(origin[i], expectedOrigin[i]) for i in range(3)),
           "pivot gizmo sits at the rest frame origin")
    avarsBefore = [child.GetAttribute(n).Get() for n in gizmoMath.AVAR_T]
    delta = Gf.Vec3d(1.0, 2.0, -0.5)
    _Drag(target, lambda: target.ApplyTranslate(delta))
    after = gizmoMath.ComputeRigFrames(stage, child, time)
    moved = (after.restLocal * after.Q * after.assetToWorld)\
        .ExtractTranslation()
    _Check(all(_Close(moved[i], expectedOrigin[i] + delta[i], 1e-6)
               for i in range(3)), "pivot translate maps onto rest:t")
    _Check([child.GetAttribute(n).Get() for n in gizmoMath.AVAR_T]
           == avarsBefore, "pivot mode never touches avars")
    target.Refresh()
    base = (after.restLocal * after.Q * after.assetToWorld)\
        .GetOrthonormalized(False)
    _Drag(target, lambda: target.ApplyRotate(Gf.Vec3d(1, 0, 0), -20.0))
    rotated = gizmoMath.ComputeRigFrames(stage, child, time)
    rotWorld = (rotated.restLocal * rotated.Q * rotated.assetToWorld)\
        .GetOrthonormalized(False)
    expected = base * _Rot(Gf.Vec3d(1, 0, 0), -20.0)
    expected.SetTranslateOnly(rotWorld.ExtractTranslation())
    _Check(_MatClose(rotWorld, expected, 1e-6), "pivot rotate onto rest:r")
    _Check(len(target.AttributePaths()) == 6, "six rest channels")


def TestXformTargets():
    stage = Usd.Stage.CreateInMemory()
    world = UsdGeom.Xform.Define(stage, "/World")
    world.AddRotateZOp().Set(90.0)
    box = UsdGeom.Xform.Define(stage, "/World/Box")
    api = UsdGeom.XformCommonAPI(box)
    api.SetTranslate(Gf.Vec3d(1, 0, 0))
    api.SetRotate(Gf.Vec3f(0, 0, 45))
    api.SetPivot(Gf.Vec3f(0, 2, 0))
    time = Usd.TimeCode.Default()
    writer = gizmoMath.Writer(stage, time, gizmoMath.WRITE_DEFAULT)
    target, reason = gizmoMath.MakeTarget(
        stage, box.GetPrim(), gizmoMath.CHANNELS_POSE, writer)
    _Check(target is not None and target.kind == "xform-pose", reason)
    cache = UsdGeom.XformCache(time)
    origin = target.GizmoMatrix().ExtractTranslation()
    expected = cache.GetLocalToWorldTransform(box.GetPrim())\
        .ExtractTranslation()
    _Check(all(_Close(origin[i], expected[i]) for i in range(3)),
           "xform gizmo at the local origin in world")
    delta = Gf.Vec3d(0, 3, 0)
    _Drag(target, lambda: target.ApplyTranslate(delta))
    cache.Clear()
    moved = cache.GetLocalToWorldTransform(box.GetPrim()).ExtractTranslation()
    _Check(all(_Close(moved[i], expected[i] + delta[i], 1e-6)
               for i in range(3)), "world translate through a rotated "
           "parent lands on xformOp:translate: %s" % moved)
    t = api.GetXformVectors(time)[0]
    # (0,3,0) * Rz(-90) = (3,0,0) in parent space, added to (1,0,0).
    _Check(_Close(t[0], 4.0, 1e-6) and _Close(t[1], 0.0, 1e-6),
           "parent-space translate value: %s" % t)
    target.Refresh()
    baseWorld = cache.GetLocalToWorldTransform(box.GetPrim())\
        .GetOrthonormalized(False)
    _Drag(target, lambda: target.ApplyRotate(Gf.Vec3d(0, 0, 1), 10.0))
    cache.Clear()
    rotWorld = cache.GetLocalToWorldTransform(box.GetPrim())\
        .GetOrthonormalized(False)
    exp = baseWorld * _Rot(Gf.Vec3d(0, 0, 1), 10.0)
    exp.SetTranslateOnly(rotWorld.ExtractTranslation())
    _Check(_MatClose(rotWorld, exp, 1e-5), "xform rotate")
    _Check(_Close(api.GetXformVectors(time)[1][2], 55.0, 1e-5),
           "rotateXYZ z = 55: %s" % (api.GetXformVectors(time)[1],))
    target.Refresh()
    _Drag(target, lambda: target.ApplyScale(None, 2.0))
    _Check(api.GetXformVectors(time)[2] == Gf.Vec3f(2, 2, 2), "scale")
    # Pivot target moves only the pivot, in parent space.
    pivotTarget, reason = gizmoMath.MakeTarget(
        stage, box.GetPrim(), gizmoMath.CHANNELS_PIVOT, writer)
    _Check(pivotTarget is not None and pivotTarget.kind == "xform-pivot",
           reason)
    _Check(pivotTarget.supportsTranslate and not pivotTarget.supportsRotate
           and not pivotTarget.supportsScale, "xform pivot: translate only")
    vectors = api.GetXformVectors(time)
    parentWorld = cache.GetParentToWorldTransform(box.GetPrim())
    expectedPivot = parentWorld.Transform(
        Gf.Vec3d(vectors[3]) + vectors[0])
    got = pivotTarget.GizmoMatrix().ExtractTranslation()
    _Check(all(_Close(got[i], expectedPivot[i], 1e-6) for i in range(3)),
           "pivot gizmo at (pivot + translate) in parent space")
    _Drag(pivotTarget, lambda: pivotTarget.ApplyTranslate(Gf.Vec3d(0, 1, 0)))
    pivot = api.GetXformVectors(time)[3]
    _Check(_Close(pivot[0], 1.0, 1e-5) and _Close(pivot[1], 2.0, 1e-5),
           "pivot moved by the parent-space delta: %s" % pivot)
    # Incompatible op stacks are refused with a reason.
    odd = UsdGeom.Xform.Define(stage, "/Odd")
    odd.AddTransformOp().Set(Gf.Matrix4d(1.0))
    refused, reason = gizmoMath.MakeTarget(
        stage, odd.GetPrim(), gizmoMath.CHANNELS_POSE, writer)
    _Check(refused is None and "XformCommonAPI" in reason, reason)
    scope = stage.DefinePrim("/Scope", "Scope")
    refused, reason = gizmoMath.MakeTarget(
        stage, scope, gizmoMath.CHANNELS_POSE, writer)
    _Check(refused is None and reason != "", "non-xformable refused")
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `bin/run_python_tests.sh test_gizmo_math`
Expected: `AttributeError: module 'gizmoMath' has no attribute 'Writer'`

- [ ] **Step 3: Append part B to `gizmoMath.py`**

```python
# ---------------------------------------------------------------------------
# Writing values: animation (spline knot at the frame) or default
# ---------------------------------------------------------------------------

WRITE_ANIMATION = "animation"
WRITE_DEFAULT = "default"
CHANNELS_POSE = "pose"
CHANNELS_PIVOT = "pivot"


def SetAnimated(attr, value, time):
    """
    volumeWeightUI.SetAtTime's rule, restated here so this module stays
    Qt-free (volumeWeightUI imports Qt at module scope): default time or
    existing time samples or a spline-incapable type -> Set(value, time);
    otherwise a curve-interpolated knot on the attribute's spline.
    """
    from pxr import Ts
    if attr.GetVariability() == Sdf.VariabilityUniform:
        attr.Set(value)
        return
    valueType = attr.GetTypeName().type
    if (time.IsDefault() or attr.GetNumTimeSamples() > 0 or
            not Ts.Spline.IsSupportedValueType(valueType)):
        attr.Set(value, time)
        return
    frame = time.GetValue()
    spline = attr.GetSpline()
    knot = spline.GetKnot(frame)
    if knot:
        knot.SetValue(value)
    else:
        knot = Ts.Knot(
            typeName=valueType.typeName, time=frame, value=value,
            nextInterp=Ts.InterpCurve)
    spline.SetKnot(knot)
    attr.SetSpline(spline)


class Writer(object):
    """
    Where a gizmo value lands. WRITE_ANIMATION authors at `time` through
    SetAnimated; WRITE_DEFAULT authors the default and records a warning
    for every attribute whose spline or time samples will outrank it
    (the default is then invisible in the viewport, and the toolbar says
    so rather than letting the drag look broken).
    """

    def __init__(self, stage, time, mode):
        self.stage = stage
        self.time = time
        self.mode = mode
        self._warnings = []

    def Set(self, attr, value):
        if self.mode == WRITE_DEFAULT:
            if attr.HasSpline() or attr.GetNumTimeSamples() > 0:
                message = ("%s: the default is outranked by its %s" % (
                    attr.GetPath(),
                    "spline" if attr.HasSpline() else "time samples"))
                if message not in self._warnings:
                    self._warnings.append(message)
            attr.Set(value)
        else:
            SetAnimated(attr, value, self.time)

    def Warnings(self):
        return list(self._warnings)


# ---------------------------------------------------------------------------
# Edit targets
# ---------------------------------------------------------------------------

def _Linear(matrix):
    """The 3x3 part as a 4x4 with zero translation."""
    m = Gf.Matrix4d(matrix)
    m.SetTranslateOnly(Gf.Vec3d(0, 0, 0))
    return m


def _RotationOnly(matrix):
    return _Linear(matrix).GetOrthonormalized(False)


def _WorldRotation(axis, degrees):
    m = Gf.Matrix4d(1.0)
    m.SetRotate(Gf.Rotation(Gf.Vec3d(axis).GetNormalized(), degrees))
    return m


class Target(object):
    """
    One prim being edited by the gizmo. Subclasses know which attributes
    they own and how a WORLD-space delta maps back onto them. Every
    Apply* is computed from the values captured by BeginDrag(), never
    incrementally, so a drag cannot accumulate rounding drift and an
    aborted drag has one well-defined state to return to.
    """

    kind = ""
    supportsTranslate = True
    supportsRotate = True
    supportsScale = True

    def __init__(self, stage, prim, writer):
        self.stage = stage
        self.prim = prim
        self.writer = writer
        self.label = prim.GetName()
        self._base = {}

    @property
    def time(self):
        return self.writer.time

    def Refresh(self):
        """Re-read the stage; call after a frame change or an undo."""

    def GizmoMatrix(self):
        raise NotImplementedError

    def AttributePaths(self):
        raise NotImplementedError

    def BeginDrag(self):
        self.Refresh()
        self._base = {name: ScalarAvar(self.prim, name, self.time, fallback)
                      for name, fallback in self._ScalarChannels()}

    def _ScalarChannels(self):
        return []

    def _Write(self, name, value):
        self.writer.Set(self.prim.GetAttribute(name), float(value))

    def ApplyTranslate(self, worldDelta):
        raise NotImplementedError

    def ApplyRotate(self, worldAxis, degrees):
        raise NotImplementedError

    def ApplyScale(self, axisIndex, factor):
        raise NotImplementedError


class _RigTarget(Target):
    """Shared frame bookkeeping for the two RigExecXformable modes."""

    def __init__(self, stage, prim, writer):
        Target.__init__(self, stage, prim, writer)
        self.frames = None
        self.Refresh()

    def Refresh(self):
        self.frames = ComputeRigFrames(self.stage, self.prim, self.time)

    def _WriteVector(self, names, values):
        with Sdf.ChangeBlock():
            for name, value in zip(names, values):
                self._Write(name, value)


class RigPoseTarget(_RigTarget):
    """Edits avars:t/r/s relative to P (see RigFrames)."""

    kind = "rig-pose"

    def _ScalarChannels(self):
        return ([(n, 0.0) for n in AVAR_T] + [(n, 0.0) for n in AVAR_R]
                + [(n, 1.0) for n in AVAR_S] + [(AVAR_RSPIN, 0.0)])

    def _Order(self):
        attr = self.prim.GetAttribute(AVAR_ORDER)
        value = attr.Get(self.time) if attr else None
        return _NormalizeOrder(value)

    def AttributePaths(self):
        return [self.prim.GetPath().AppendProperty(n)
                for n in AVAR_T + AVAR_R + AVAR_S]

    def GizmoMatrix(self):
        world = self.frames.posed * self.frames.assetToWorld
        return world.GetOrthonormalized(False)

    def _Pw(self):
        return self.frames.P * self.frames.assetToWorld

    def ApplyTranslate(self, worldDelta):
        local = _Linear(self._Pw()).GetInverse().TransformDir(
            Gf.Vec3d(worldDelta))
        self._WriteVector(AVAR_T, [self._base[n] + local[i]
                                   for i, n in enumerate(AVAR_T)])

    def ApplyRotate(self, worldAxis, degrees):
        base = [self._base[n] for n in AVAR_R]
        order = self._Order()
        spin = _AxisRotation(0, self._base[AVAR_RSPIN])
        pr = _RotationOnly(self._Pw())
        # S * R' * spin * Pr = S * R * spin * Pr * Rw
        rNew = (RotationFromEuler(order, *base) * spin * pr
                * _WorldRotation(worldAxis, degrees) * pr.GetInverse()
                * spin.GetInverse())
        self._WriteVector(AVAR_R, DecomposeEuler(rNew, order, hint=base))

    def ApplyScale(self, axisIndex, factor):
        values = []
        for i, name in enumerate(AVAR_S):
            value = self._base[name]
            if axisIndex is None or axisIndex == i:
                value = value * factor
            values.append(NormalizeAvarScale(value))
        self._WriteVector(AVAR_S, values)


class RigPivotTarget(_RigTarget):
    """Edits rest:t/r relative to Q (see RigFrames); no scale."""

    kind = "rig-pivot"
    supportsScale = False

    def _ScalarChannels(self):
        return [(n, 0.0) for n in REST_T] + [(n, 0.0) for n in REST_R]

    def AttributePaths(self):
        return [self.prim.GetPath().AppendProperty(n)
                for n in REST_T + REST_R]

    def _Qw(self):
        return self.frames.Q * self.frames.assetToWorld

    def GizmoMatrix(self):
        return (self.frames.restLocal * self._Qw()).GetOrthonormalized(False)

    def ApplyTranslate(self, worldDelta):
        local = _Linear(self._Qw()).GetInverse().TransformDir(
            Gf.Vec3d(worldDelta))
        self._WriteVector(REST_T, [self._base[n] + local[i]
                                   for i, n in enumerate(REST_T)])

    def ApplyRotate(self, worldAxis, degrees):
        base = [self._base[n] for n in REST_R]
        qr = _RotationOnly(self._Qw())
        rNew = (RotationFromEuler("XYZ", *base) * qr
                * _WorldRotation(worldAxis, degrees) * qr.GetInverse())
        self._WriteVector(REST_R, DecomposeEuler(rNew, "XYZ", hint=base))

    def ApplyScale(self, axisIndex, factor):
        pass


_XFORM_ORDER_NAMES = {
    UsdGeom.XformCommonAPI.RotationOrderXYZ: "XYZ",
    UsdGeom.XformCommonAPI.RotationOrderXZY: "XZY",
    UsdGeom.XformCommonAPI.RotationOrderYXZ: "YXZ",
    UsdGeom.XformCommonAPI.RotationOrderYZX: "YZX",
    UsdGeom.XformCommonAPI.RotationOrderZXY: "ZXY",
    UsdGeom.XformCommonAPI.RotationOrderZYX: "ZYX",
}


class _XformTarget(Target):
    """
    A plain UsdGeomXformable through UsdGeomXformCommonAPI, whose op
    stack is [translate, pivot, rotate, scale, !invert!pivot]: ops apply
    to points last-to-first, so scale then rotation happen about the
    pivot and the translate is in PARENT space -- which is why world
    deltas are mapped through the parent's transform, not the prim's.
    """

    def __init__(self, stage, prim, writer):
        Target.__init__(self, stage, prim, writer)
        self.api = UsdGeom.XformCommonAPI(prim)
        self.vectors = None
        self.parentWorld = Gf.Matrix4d(1.0)
        self.Refresh()

    def Refresh(self):
        cache = UsdGeom.XformCache(self.time)
        self.parentWorld = cache.GetParentToWorldTransform(self.prim)
        self.vectors = self.api.GetXformVectors(self.time)

    def BeginDrag(self):
        self.Refresh()
        t, r, s, p, order = self.vectors
        self._base = {"t": Gf.Vec3d(t), "r": Gf.Vec3f(r), "s": Gf.Vec3f(s),
                      "p": Gf.Vec3f(p), "order": order}

    def _Ops(self):
        """The four ops, created on demand (a no-op for existing ones)."""
        api = UsdGeom.XformCommonAPI
        return self.api.CreateXformOps(
            self._base["order"], api.OpTranslate, api.OpPivot,
            api.OpRotate, api.OpScale)

    def AttributePaths(self):
        prefix = self.prim.GetPath()
        _, _, _, _, order = self.vectors
        rotateName = "xformOp:rotate" + _XFORM_ORDER_NAMES[order]
        return [prefix.AppendProperty(n) for n in (
            "xformOp:translate", "xformOp:translate:pivot", rotateName,
            "xformOp:scale", "xformOpOrder")]



class XformPoseTarget(_XformTarget):
    kind = "xform-pose"

    def GizmoMatrix(self):
        local = UsdGeom.Xformable(self.prim).GetLocalTransformation(self.time)
        return (local * self.parentWorld).GetOrthonormalized(False)

    def ApplyTranslate(self, worldDelta):
        local = _Linear(self.parentWorld).GetInverse().TransformDir(
            Gf.Vec3d(worldDelta))
        ops = self._Ops()
        self.writer.Set(ops[0].GetAttr(), self._base["t"] + local)

    def ApplyRotate(self, worldAxis, degrees):
        order = _XFORM_ORDER_NAMES[self._base["order"]]
        base = [float(v) for v in self._base["r"]]
        pr = _RotationOnly(self.parentWorld)
        rNew = (RotationFromEuler(order, *base) * pr
                * _WorldRotation(worldAxis, degrees) * pr.GetInverse())
        angles = DecomposeEuler(rNew, order, hint=base)
        ops = self._Ops()
        self.writer.Set(ops[2].GetAttr(), Gf.Vec3f(*angles))

    def ApplyScale(self, axisIndex, factor):
        s = Gf.Vec3f(self._base["s"])
        for i in range(3):
            if axisIndex is None or axisIndex == i:
                s[i] = s[i] * factor
        ops = self._Ops()
        self.writer.Set(ops[3].GetAttr(), s)


class XformPivotTarget(_XformTarget):
    kind = "xform-pivot"
    supportsRotate = False
    supportsScale = False

    def GizmoMatrix(self):
        t, r, s, p, order = self.vectors
        m = _RotationOnly(self.parentWorld)
        m.SetTranslateOnly(self.parentWorld.Transform(Gf.Vec3d(p) + t))
        return m

    def ApplyTranslate(self, worldDelta):
        local = _Linear(self.parentWorld).GetInverse().TransformDir(
            Gf.Vec3d(worldDelta))
        ops = self._Ops()
        self.writer.Set(ops[1].GetAttr(),
                        Gf.Vec3f(self._base["p"] + Gf.Vec3f(local)))

    def ApplyRotate(self, worldAxis, degrees):
        pass

    def ApplyScale(self, axisIndex, factor):
        pass


def _ConnectedAvar(prim, names):
    for name in names:
        attr = prim.GetAttribute(name)
        if attr and attr.HasAuthoredConnections():
            return name
    return None


def MakeTarget(stage, prim, channels, writer):
    """(target, "") or (None, reason) for usdview's focus prim."""
    if not prim or not prim.IsValid():
        return None, "nothing selected"
    if IsRigXformable(prim):
        frames = ComputeRigFrames(stage, prim, writer.time)
        if frames.reason:
            return None, frames.reason
        names = (AVAR_T + AVAR_R + AVAR_S if channels == CHANNELS_POSE
                 else REST_T + REST_R)
        connected = _ConnectedAvar(prim, names)
        if connected:
            return None, "%s.%s is connected; edit its source instead" % (
                prim.GetName(), connected)
        if channels == CHANNELS_PIVOT:
            return RigPivotTarget(stage, prim, writer), ""
        return RigPoseTarget(stage, prim, writer), ""
    if prim.IsA(UsdGeom.Xformable):
        if not UsdGeom.XformCommonAPI(prim):
            return None, ("%s: xformOp stack is not XformCommonAPI-"
                          "compatible" % prim.GetName())
        if channels == CHANNELS_PIVOT:
            return XformPivotTarget(stage, prim, writer), ""
        return XformPoseTarget(stage, prim, writer), ""
    return None, "%s (%s) has no transform to edit" % (
        prim.GetName(), prim.GetTypeName() or "untyped")
```

`UsdGeom.XformCommonAPI.CreateXformOps` returns a 4-tuple `(translateOp, pivotOp, rotateOp, scaleOp)` in this USD; confirm with `python -c "from pxr import UsdGeom; help(UsdGeom.XformCommonAPI.CreateXformOps)"` and, if the binding takes a different argument shape, adapt `_Ops()` (the tests pin the observable result, not the call).

- [ ] **Step 4: Run the test to verify it passes**

Run: `bin/run_python_tests.sh test_gizmo_math`
Expected: eight `ok:` lines and `RIGEXEC_GIZMO_MATH_OK (8 groups)`.

- [ ] **Step 5: Commit**

```bash
git add plugin/rigExecUsdview/gizmoMath.py tests/python/test_gizmo_math.py
git commit -m "usdview: gizmo edit targets and animation/default writer" -m "Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>" -m "Claude-Session: https://claude.ai/code/session_014a1ZRa9PujUhqJSZ5qWNYB"
```

---

### Task 4: `gizmoScreen.py` — projection, handle geometry, hit-testing, drag math

**Files:**
- Create: `plugin/rigExecUsdview/gizmoScreen.py`
- Create: `tests/python/test_gizmo_screen.py`

**Interfaces:**
- Consumes: nothing from other tasks (pure `Gf`).
- Produces:
  - `TOOL_TRANSLATE = "translate"`, `TOOL_ROTATE = "rotate"`, `TOOL_SCALE = "scale"`
  - `GIZMO_PIXELS = 90.0`, `HIT_PIXELS = 8.0`, `CENTER_PIXELS = 6.0`, `RING_FRACTION = 0.85`, `RING_SEGMENTS = 48`, `SCALE_PIXELS_PER_UNIT = 120.0`
  - `class Handle`: `name` ("x" | "y" | "z" | "center"), `kind` ("axis" | "ring" | "center"), `axisIndex` (0/1/2 or `None`), `points` (list of `(x, y)` physical pixels; 2 for axis, `RING_SEGMENTS` for ring, 1 for center), `worldAxis` (`Gf.Vec3d`, unit, or `None`), `worldLength` (float), `color` (`(r, g, b)` 0..1), `center` (`(x, y)` projected gizmo origin)
  - `ViewProjection(camera: Gf.Camera) -> Gf.Matrix4d`
  - `ProjectPoint(viewProj, viewport, p: Gf.Vec3d) -> (x, y) | None`
  - `CameraBasis(camera) -> (viewDir, up, right)` unit `Gf.Vec3d`
  - `WorldPerPixel(camera, viewport, worldPoint) -> float | None`
  - `BuildHandles(tool, gizmoMatrix, camera, viewport, pixelRatio) -> list[Handle]`
  - `HitTest(handles, x, y, radius) -> Handle | None`
  - `AxisDragParameter(handle, press, current) -> float` (fraction of the handle's screen length)
  - `PlaneDragDelta(camera, viewport, worldOrigin, press, current) -> Gf.Vec3d`
  - `AxisFacesCamera(camera, worldAxis) -> bool`
  - `RotationDragAngle(center, press, current, axisFacesCamera) -> float` degrees
  - `ScaleDragFactor(handle, press, current, pixelRatio) -> float`

- [ ] **Step 1: Write the failing test**

`tests/python/test_gizmo_screen.py`:

```python
#!/usr/bin/env python
"""
Headless test for plugin/rigExecUsdview/gizmoScreen.py using a synthetic
Gf.Camera at (0, 0, 10) looking down -Z into an 800x600 viewport, so
+X is screen-right and +Y is screen-up and the gizmo origin projects to
(400, 300). Usage: test_gizmo_screen.py [ignored]
"""
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.normpath(
    os.path.join(_HERE, "..", "..", "plugin", "rigExecUsdview")))

from pxr import Gf  # noqa: E402

import gizmoScreen as gs  # noqa: E402

VIEWPORT = (0, 0, 800, 600)


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _Close(a, b, tol=1e-6):
    return abs(a - b) <= tol


def _Camera():
    camera = Gf.Camera()
    camera.transform = Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(0, 0, 10))
    return camera


def _Dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def TestProjection():
    camera = _Camera()
    vp = gs.ViewProjection(camera)
    centre = gs.ProjectPoint(vp, VIEWPORT, Gf.Vec3d(0, 0, 0))
    _Check(_Close(centre[0], 400, 1e-3) and _Close(centre[1], 300, 1e-3),
           "origin projects to the viewport centre: %s" % (centre,))
    right = gs.ProjectPoint(vp, VIEWPORT, Gf.Vec3d(1, 0, 0))
    _Check(right[0] > 400 and _Close(right[1], 300, 1e-3), "+X is right")
    up = gs.ProjectPoint(vp, VIEWPORT, Gf.Vec3d(0, 1, 0))
    _Check(up[1] < 300 and _Close(up[0], 400, 1e-3), "+Y is up")
    _Check(gs.ProjectPoint(vp, VIEWPORT, Gf.Vec3d(0, 0, 20)) is None,
           "behind the eye is None")
    viewDir, upVec, rightVec = gs.CameraBasis(camera)
    _Check(_Close(viewDir[2], -1.0) and _Close(upVec[1], 1.0)
           and _Close(rightVec[0], 1.0), "camera basis")
    wpp = gs.WorldPerPixel(camera, VIEWPORT, Gf.Vec3d(0, 0, 0))
    one = gs.ProjectPoint(vp, VIEWPORT, Gf.Vec3d(wpp, 0, 0))
    _Check(_Close(one[0], 401.0, 1e-3), "world-per-pixel: %s" % (one,))


def TestTranslateHandles():
    camera = _Camera()
    handles = gs.BuildHandles(gs.TOOL_TRANSLATE, Gf.Matrix4d(1.0), camera,
                              VIEWPORT, 1.0)
    byName = {h.name: h for h in handles}
    _Check(set(byName) == {"x", "y", "z", "center"}, "four handles")
    x = byName["x"]
    _Check(x.kind == "axis" and x.axisIndex == 0
           and _Close(x.points[1][0], 400 + gs.GIZMO_PIXELS, 1e-3)
           and _Close(x.points[1][1], 300, 1e-3),
           "x axis is GIZMO_PIXELS long on screen: %s" % (x.points,))
    y = byName["y"]
    _Check(_Close(y.points[1][1], 300 - gs.GIZMO_PIXELS, 1e-3), "y up")
    z = byName["z"]
    _Check(_Dist(z.points[0], z.points[1]) < 1.0,
           "z axis points at the camera and projects to a point")
    _Check(_Close(x.worldLength, gs.GIZMO_PIXELS
                  * gs.WorldPerPixel(camera, VIEWPORT, Gf.Vec3d(0, 0, 0)),
                  1e-9), "world length matches the pixel length")
    _Check(byName["center"].kind == "center"
           and _Close(byName["center"].points[0][0], 400, 1e-3), "centre")
    # Pixel ratio scales the on-screen size.
    hi = {h.name: h for h in gs.BuildHandles(
        gs.TOOL_TRANSLATE, Gf.Matrix4d(1.0), camera, VIEWPORT, 2.0)}
    _Check(_Close(hi["x"].points[1][0], 400 + 2 * gs.GIZMO_PIXELS, 1e-3),
           "pixel ratio 2 doubles the physical length")
    # A rotated gizmo frame rotates the handles.
    rotated = Gf.Matrix4d(1.0).SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), 90))
    rx = {h.name: h for h in gs.BuildHandles(
        gs.TOOL_TRANSLATE, rotated, camera, VIEWPORT, 1.0)}["x"]
    _Check(_Close(rx.points[1][1], 300 - gs.GIZMO_PIXELS, 1e-3)
           and _Close(rx.worldAxis[1], 1.0), "local X now points up")
    # Origin behind the eye: no handles at all.
    behind = Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(0, 0, 30))
    _Check(gs.BuildHandles(gs.TOOL_TRANSLATE, behind, camera, VIEWPORT, 1.0)
           == [], "nothing to draw behind the camera")


def TestRotateHandles():
    camera = _Camera()
    handles = gs.BuildHandles(gs.TOOL_ROTATE, Gf.Matrix4d(1.0), camera,
                              VIEWPORT, 1.0)
    byName = {h.name: h for h in handles}
    _Check(set(byName) == {"x", "y", "z"}, "three rings")
    z = byName["z"]
    _Check(z.kind == "ring" and len(z.points) == gs.RING_SEGMENTS, "ring")
    radius = gs.GIZMO_PIXELS * gs.RING_FRACTION
    _Check(all(_Close(_Dist(p, (400, 300)), radius, 0.5) for p in z.points),
           "z ring is a circle of RING_FRACTION * GIZMO_PIXELS")
    xs = [p[0] for p in byName["x"].points]
    _Check(max(xs) - min(xs) < 1.0, "x ring is edge-on: a vertical line")


def TestHitTest():
    camera = _Camera()
    handles = gs.BuildHandles(gs.TOOL_TRANSLATE, Gf.Matrix4d(1.0), camera,
                              VIEWPORT, 1.0)
    hit = gs.HitTest(handles, 445, 303, gs.HIT_PIXELS)
    _Check(hit is not None and hit.name == "x", "x axis hit")
    hit = gs.HitTest(handles, 398, 255, gs.HIT_PIXELS)
    _Check(hit is not None and hit.name == "y", "y axis hit")
    hit = gs.HitTest(handles, 401, 299, gs.HIT_PIXELS)
    _Check(hit is not None and hit.name == "center",
           "centre wins over the axes that start there: %s"
           % (hit and hit.name))
    _Check(gs.HitTest(handles, 600, 600, gs.HIT_PIXELS) is None, "miss")
    rings = gs.BuildHandles(gs.TOOL_ROTATE, Gf.Matrix4d(1.0), camera,
                            VIEWPORT, 1.0)
    radius = gs.GIZMO_PIXELS * gs.RING_FRACTION
    hit = gs.HitTest(rings, 400 + radius * 0.7071, 300 - radius * 0.7071,
                     gs.HIT_PIXELS)
    _Check(hit is not None and hit.name == "z", "z ring hit at 45 degrees")


def TestDragMath():
    camera = _Camera()
    handles = {h.name: h for h in gs.BuildHandles(
        gs.TOOL_TRANSLATE, Gf.Matrix4d(1.0), camera, VIEWPORT, 1.0)}
    x = handles["x"]
    t = gs.AxisDragParameter(x, (400, 300), (445, 310))
    _Check(_Close(t, 0.5, 1e-9), "half the handle length: %s" % t)
    delta = x.worldAxis * (t * x.worldLength)
    vp = gs.ViewProjection(camera)
    moved = gs.ProjectPoint(vp, VIEWPORT, Gf.Vec3d(delta))
    _Check(_Close(moved[0], 445.0, 1e-3), "the world delta projects back "
           "to the mouse travel: %s" % (moved,))
    # A degenerate (foreshortened) axis does not explode.
    z = handles["z"]
    _Check(abs(gs.AxisDragParameter(z, (400, 300), (500, 300))) < 100.0,
           "foreshortened axis is clamped")
    wpp = gs.WorldPerPixel(camera, VIEWPORT, Gf.Vec3d(0, 0, 0))
    plane = gs.PlaneDragDelta(camera, VIEWPORT, Gf.Vec3d(0, 0, 0),
                              (400, 300), (410, 290))
    _Check(_Close(plane[0], 10 * wpp, 1e-9) and _Close(plane[1], 10 * wpp,
                                                         1e-9)
           and _Close(plane[2], 0.0), "screen-plane delta: %s" % plane)
    _Check(gs.AxisFacesCamera(camera, Gf.Vec3d(0, 0, 1)), "+Z faces us")
    _Check(not gs.AxisFacesCamera(camera, Gf.Vec3d(0, 0, -1)), "-Z away")
    angle = gs.RotationDragAngle((400, 300), (500, 300), (400, 200), True)
    _Check(_Close(angle, 90.0, 1e-9), "CCW on screen is +90 facing: %s"
           % angle)
    angle = gs.RotationDragAngle((400, 300), (500, 300), (400, 200), False)
    _Check(_Close(angle, -90.0, 1e-9), "sign flips for an axis pointing "
           "away")
    angle = gs.RotationDragAngle((400, 300), (500, 300), (300, 301), True)
    _Check(179.0 < angle <= 180.0 or -180.0 <= angle < -179.0,
           "wrapped into (-180, 180]: %s" % angle)
    scale = {h.name: h for h in gs.BuildHandles(
        gs.TOOL_SCALE, Gf.Matrix4d(1.0), camera, VIEWPORT, 1.0)}
    _Check(set(scale) == {"x", "y", "z", "center"}, "scale handles")
    f = gs.ScaleDragFactor(scale["x"], (400, 300), (460, 300), 1.0)
    _Check(_Close(f, 1.5, 1e-9), "60 px along x: %s" % f)
    f = gs.ScaleDragFactor(scale["x"], (400, 300), (400, 360), 1.0)
    _Check(_Close(f, 1.0, 1e-9), "perpendicular travel does nothing")
    f = gs.ScaleDragFactor(scale["center"], (400, 300), (340, 300), 1.0)
    _Check(_Close(f, 0.5, 1e-9), "centre: horizontal travel, uniform")
    f = gs.ScaleDragFactor(scale["x"], (400, 300), (0, 300), 1.0)
    _Check(f >= 0.01, "factor is floored")


def main():
    groups = [
        ("projection", TestProjection),
        ("translate handles", TestTranslateHandles),
        ("rotate handles", TestRotateHandles),
        ("hit test", TestHitTest),
        ("drag math", TestDragMath),
    ]
    for name, fn in groups:
        fn()
        print("  ok: %s" % name)
    print("RIGEXEC_GIZMO_SCREEN_OK (%d groups)" % len(groups))
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `bin/run_python_tests.sh test_gizmo_screen`
Expected: `ModuleNotFoundError: No module named 'gizmoScreen'`

- [ ] **Step 3: Implement `gizmoScreen.py`**

```python
#
# RigExec usdview gizmo: screen-space geometry. Pure functions over Gf so
# the handle layout, hit-testing and drag mapping are testable with a
# synthetic Gf.Camera and no Qt.
#
# Pixel space is the PHYSICAL pixel space of StageView.computePickFrustum
# (curvenetUI.SurfacePicker.Project documents the mapping); callers
# convert Qt's logical coordinates with devicePixelRatioF() and back.
#
import math

from pxr import Gf

TOOL_TRANSLATE = "translate"
TOOL_ROTATE = "rotate"
TOOL_SCALE = "scale"

# Handle sizes in LOGICAL pixels (multiplied by the device pixel ratio).
GIZMO_PIXELS = 90.0
HIT_PIXELS = 8.0
CENTER_PIXELS = 6.0
RING_FRACTION = 0.85
RING_SEGMENTS = 48
SCALE_PIXELS_PER_UNIT = 120.0
_MIN_AXIS_PIXELS = 4.0

_AXES = (Gf.Vec3d(1, 0, 0), Gf.Vec3d(0, 1, 0), Gf.Vec3d(0, 0, 1))
_AXIS_NAMES = ("x", "y", "z")
_AXIS_COLORS = ((0.95, 0.25, 0.25), (0.35, 0.85, 0.3), (0.3, 0.5, 1.0))
_CENTER_COLOR = (0.95, 0.95, 0.95)


class Handle(object):
    def __init__(self, name, kind, axisIndex, points, worldAxis,
                 worldLength, color, center):
        self.name = name
        self.kind = kind
        self.axisIndex = axisIndex
        self.points = points
        self.worldAxis = worldAxis
        self.worldLength = worldLength
        self.color = color
        self.center = center

    def __repr__(self):
        return "<Handle %s %s>" % (self.name, self.kind)


def ViewProjection(camera):
    frustum = camera.frustum
    return frustum.ComputeViewMatrix() * frustum.ComputeProjectionMatrix()


def ProjectPoint(viewProj, viewport, p):
    """World -> physical pixels; None behind the eye."""
    clip = Gf.Vec4d(p[0], p[1], p[2], 1.0) * viewProj
    if clip[3] <= 1e-9:
        return None
    ndcX, ndcY = clip[0] / clip[3], clip[1] / clip[3]
    return ((ndcX + 1.0) * 0.5 * viewport[2] + viewport[0],
            (1.0 - ndcY) * 0.5 * viewport[3] + viewport[1])


def CameraBasis(camera):
    frustum = camera.frustum
    viewDir = Gf.Vec3d(frustum.ComputeViewDirection()).GetNormalized()
    up = Gf.Vec3d(frustum.ComputeUpVector()).GetNormalized()
    right = Gf.Cross(viewDir, up).GetNormalized()
    return viewDir, up, right


def WorldPerPixel(camera, viewport, worldPoint):
    """World units per physical pixel in the camera plane at worldPoint."""
    _, _, right = CameraBasis(camera)
    vp = ViewProjection(camera)
    a = ProjectPoint(vp, viewport, worldPoint)
    b = ProjectPoint(vp, viewport, Gf.Vec3d(worldPoint) + right)
    if a is None or b is None:
        return None
    pixels = math.hypot(b[0] - a[0], b[1] - a[1])
    if pixels < 1e-9:
        return None
    return 1.0 / pixels


def BuildHandles(tool, gizmoMatrix, camera, viewport, pixelRatio):
    """
    Screen-constant handles for `tool` at `gizmoMatrix` (world; its rows
    are the local axes). Empty when the origin is behind the eye or the
    frame is degenerate.
    """
    origin = Gf.Vec3d(gizmoMatrix.ExtractTranslation())
    wpp = WorldPerPixel(camera, viewport, origin)
    if wpp is None:
        return []
    vp = ViewProjection(camera)
    center = ProjectPoint(vp, viewport, origin)
    if center is None:
        return []
    length = GIZMO_PIXELS * pixelRatio * wpp
    axes = []
    for i in range(3):
        axis = Gf.Vec3d(gizmoMatrix.TransformDir(_AXES[i]))
        if axis.GetLength() < 1e-12:
            return []
        axes.append(axis.GetNormalized())

    handles = []
    if tool in (TOOL_TRANSLATE, TOOL_SCALE):
        for i in range(3):
            end = ProjectPoint(vp, viewport, origin + axes[i] * length)
            if end is None:
                continue
            handles.append(Handle(
                _AXIS_NAMES[i], "axis", i, [center, end], axes[i], length,
                _AXIS_COLORS[i], center))
        handles.append(Handle(
            "center", "center", None, [center], None, length,
            _CENTER_COLOR, center))
    elif tool == TOOL_ROTATE:
        radius = length * RING_FRACTION
        for i in range(3):
            u, v = axes[(i + 1) % 3], axes[(i + 2) % 3]
            points = []
            for s in range(RING_SEGMENTS):
                theta = 2.0 * math.pi * s / RING_SEGMENTS
                p = ProjectPoint(vp, viewport, origin + (
                    u * math.cos(theta) + v * math.sin(theta)) * radius)
                if p is None:
                    points = None
                    break
                points.append(p)
            if points is None:
                continue
            handles.append(Handle(
                _AXIS_NAMES[i], "ring", i, points, axes[i], radius,
                _AXIS_COLORS[i], center))
    return handles


def _PointSegmentDistance(p, a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    length2 = dx * dx + dy * dy
    if length2 < 1e-12:
        return math.hypot(p[0] - ax, p[1] - ay)
    t = ((p[0] - ax) * dx + (p[1] - ay) * dy) / length2
    t = max(0.0, min(1.0, t))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy))


def _HandleDistance(handle, p):
    if handle.kind == "center":
        return math.hypot(p[0] - handle.points[0][0],
                          p[1] - handle.points[0][1])
    if handle.kind == "axis":
        return _PointSegmentDistance(p, handle.points[0], handle.points[1])
    points = handle.points
    return min(_PointSegmentDistance(p, points[i],
                                     points[(i + 1) % len(points)])
               for i in range(len(points)))


def HitTest(handles, x, y, radius):
    """
    The handle under (x, y) within `radius` pixels, or None. The centre
    handle wins when it is hit at all: every axis starts there, and the
    small square is the thing the artist aimed at.
    """
    p = (x, y)
    for handle in handles:
        if handle.kind == "center" and _HandleDistance(handle, p) <= radius:
            return handle
    best = None
    for handle in handles:
        d = _HandleDistance(handle, p)
        if d <= radius and (best is None or d < best[0]):
            best = (d, handle)
    return best[1] if best else None


def AxisDragParameter(handle, press, current):
    """
    Mouse travel projected onto the handle's screen direction, as a
    fraction of the handle's screen length (so 1.0 == one handle length
    == handle.worldLength in world units). A foreshortened axis has its
    screen length floored so a few pixels cannot become a huge move.
    """
    ax, ay = handle.points[0]
    bx, by = handle.points[-1]
    dx, dy = bx - ax, by - ay
    length = math.hypot(dx, dy)
    if length < _MIN_AXIS_PIXELS:
        if length < 1e-9:
            return 0.0
        dx, dy = dx / length * _MIN_AXIS_PIXELS, dy / length * _MIN_AXIS_PIXELS
        length = _MIN_AXIS_PIXELS
    ux, uy = dx / length, dy / length
    travel = (current[0] - press[0]) * ux + (current[1] - press[1]) * uy
    return travel / length


def PlaneDragDelta(camera, viewport, worldOrigin, press, current):
    """World delta for a drag in the camera plane through worldOrigin."""
    wpp = WorldPerPixel(camera, viewport, worldOrigin)
    if wpp is None:
        return Gf.Vec3d(0, 0, 0)
    _, up, right = CameraBasis(camera)
    dx = current[0] - press[0]
    dy = current[1] - press[1]
    return right * (dx * wpp) - up * (dy * wpp)


def AxisFacesCamera(camera, worldAxis):
    viewDir, _, _ = CameraBasis(camera)
    return Gf.Dot(Gf.Vec3d(worldAxis), viewDir) < 0.0


def RotationDragAngle(center, press, current, axisFacesCamera):
    """
    Degrees swept around `center` from press to current, positive for
    counter-clockwise ON SCREEN when the rotation axis points at the
    camera (right-hand rule seen from the axis tip), wrapped to
    (-180, 180]. Screen y grows downward, hence the negation.
    """
    a0 = math.atan2(-(press[1] - center[1]), press[0] - center[0])
    a1 = math.atan2(-(current[1] - center[1]), current[0] - center[0])
    degrees = math.degrees(a1 - a0)
    while degrees > 180.0:
        degrees -= 360.0
    while degrees <= -180.0:
        degrees += 360.0
    return degrees if axisFacesCamera else -degrees


def ScaleDragFactor(handle, press, current, pixelRatio):
    """1 + travel / SCALE_PIXELS_PER_UNIT, floored at 0.01."""
    if handle.kind == "center":
        travel = current[0] - press[0]
    else:
        ax, ay = handle.points[0]
        bx, by = handle.points[-1]
        dx, dy = bx - ax, by - ay
        length = math.hypot(dx, dy)
        if length < 1e-9:
            travel = current[0] - press[0]
        else:
            travel = ((current[0] - press[0]) * dx
                      + (current[1] - press[1]) * dy) / length
    return max(0.01, 1.0 + travel / (SCALE_PIXELS_PER_UNIT * pixelRatio))
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `bin/run_python_tests.sh test_gizmo_screen`
Expected: five `ok:` lines and `RIGEXEC_GIZMO_SCREEN_OK (5 groups)`. If `TestProjection` fails on the +Y direction, the default `Gf.Camera` orientation differs from the assumption; fix the test camera (set `camera.transform` to a look-at that looks down -Z with +Y up), not the projection.

- [ ] **Step 5: Commit** (CMake registration is done by Task 1; do not edit CMakeLists.txt here)

```bash
git add plugin/rigExecUsdview/gizmoScreen.py tests/python/test_gizmo_screen.py
git commit -m "usdview: gizmo screen-space handles, hit-testing and drag math" -m "Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>" -m "Claude-Session: https://claude.ai/code/session_014a1ZRa9PujUhqJSZ5qWNYB"
```

---

### Task 5: `gizmoUI.py` — toolbar, overlay, controller, shortcuts; container wiring

**Files:**
- Create: `plugin/rigExecUsdview/gizmoUI.py`
- Modify: `plugin/rigExecUsdview/rigExecUsdview.py` (registerPlugins ~line 96-140, configureView ~151-155, `_OnStageReplaced` ~line 393, new helpers)

**Interfaces:**
- Consumes: Task 1 `rigExecUndo.UndoStack`, `rigExecUndo.EditRecorder`; Task 3 `gizmoMath.MakeTarget`, `gizmoMath.Writer`, `WRITE_*`, `CHANNELS_*`, `Target.*`; Task 4 `gizmoScreen.*`.
- Produces:
  - `TOOL_SELECT = "select"` plus re-exported `TOOL_TRANSLATE/ROTATE/SCALE`
  - `StageView(usdviewApi) -> QWidget | None`
  - `class GizmoController(QtCore.QObject)`: `SetTool(tool)`, `Tool()`, `SetChannels(mode)`, `Channels()`, `SetWriteMode(mode)`, `WriteMode()`, `Target()`, `Reason()`, `Status()`, `Handles()`, `HandleScreenPositions() -> dict[name, list[(x, y)]]` (LOGICAL pixels), `IsDragging()`, `Undo()`, `Redo()`, `SetVisible(bool)`, `IsVisible()`, `RefreshTarget()`, `toolbar` attribute, `undoStack` attribute
  - `InstallViewportTools(usdviewApi, undoStack) -> GizmoController | None` (idempotent), `GetController()`
- Container: `RigExecUsdviewContainer._UndoStack()`, `_EnsureViewportTools()`, `_ToggleViewportTools()`, menu item "Viewport Tools".

There is no headless test for this task (it is Qt); Task 6's testusdview script is its test. Still verify each step by launching: `bin/launch.sh` opens usdview on `examples/ArmShotAnim.usda`; the toolbar must appear above the viewport and selecting `HandIK` in the prim tree with Translate active must draw the gizmo. Do this verification through the testusdview harness rather than by hand: write `/tmp/gizmo_smoke.py` containing

```python
import os
def testUsdviewInputFunction(appController):
    import gizmoUI
    api = appController._usdviewApi
    c = gizmoUI.GetController()
    assert c is not None, "controller not installed"
    prim = api.stage.GetPrimAtPath("/Shot/HeroArm/Rig/Controls/HandIK")
    api.ClearPrimSelection(); api.AddPrimToSelection(prim)
    appController._processEvents()
    c.SetTool(gizmoUI.TOOL_TRANSLATE)
    appController._processEvents()
    print("target:", c.Target(), "reason:", repr(c.Reason()))
    print("handles:", c.HandleScreenPositions())
    view = gizmoUI.StageView(api)
    view.window().grab().save(os.environ.get("RIGEXEC_GIZMO_SHOT", "/tmp/gizmo_smoke.png"))
```

and run `. bin/_env.sh && "$PY" "$TESTUSDVIEW" --testScript /tmp/gizmo_smoke.py examples/ArmShotAnim.usda`, then look at the PNG (Read tool) to confirm the toolbar and the three coloured axes are visible.

- [ ] **Step 1: Write `gizmoUI.py`**

```python
#
# RigExec usdview viewport tools: a toolbar above the stage view and a
# translate/rotate/scale gizmo drawn on a transparent overlay, editing
# avars (pose) or rest offsets (pivot) on RigExec controls and joints, and
# xformOps on plain xforms, with every drag undoable.
#
# Qt lives only here. The math (gizmoMath), the screen geometry
# (gizmoScreen) and the undo stack (rigExecUndo) are Qt-free and tested
# headlessly; this module is the thin driver over them, exercised by
# tests/testUsdviewGizmo.py through testusdview.
#
# Viewport access follows curvenetUI: the StageView is reached through
# usdview's app controller, mouse events are intercepted with an event
# filter (returning True consumes the event so the camera never sees a
# gizmo drag), and Alt/Meta drags are passed through so the artist can
# still orbit with a tool active.
#
# Drawing is a transparent, mouse-transparent child widget of the
# StageView repainted with QPainter (not session-layer prims as curvenetUI
# does): a gizmo must be screen-constant, never occluded, and must not
# re-evaluate the rig every time the camera moves.
#
import math

from pxr import Gf, Sdf, Tf, Usd
from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets

import gizmoMath
import gizmoScreen
import rigExecUndo

try:
    from pxr.Usdviewq.qt import QtActionWidgets
except ImportError:  # PySide2: QAction/QActionGroup are in QtWidgets
    QtActionWidgets = QtWidgets

TOOL_SELECT = "select"
TOOL_TRANSLATE = gizmoScreen.TOOL_TRANSLATE
TOOL_ROTATE = gizmoScreen.TOOL_ROTATE
TOOL_SCALE = gizmoScreen.TOOL_SCALE

_TOOLS = (
    (TOOL_SELECT, "Select", "Leave the viewport to usdview's own picking"),
    (TOOL_TRANSLATE, "Translate", "Drag an axis or the centre square"),
    (TOOL_ROTATE, "Rotate", "Drag a ring"),
    (TOOL_SCALE, "Scale", "Drag an axis, or the centre for uniform"),
)
_CHANNELS = (
    (gizmoMath.CHANNELS_POSE, "Pose",
     "Edit avars:t/r/s (or xformOps on a plain xform)"),
    (gizmoMath.CHANNELS_PIVOT, "Pivot",
     "Edit rest:t/r (or the XformCommonAPI pivot on a plain xform)"),
)
_WRITE_MODES = (
    (gizmoMath.WRITE_ANIMATION, "Animation",
     "Author a spline knot / time sample at the current frame"),
    (gizmoMath.WRITE_DEFAULT, "Default",
     "Author the attribute's default value"),
)
_CAMERA_MODIFIERS = QtCore.Qt.AltModifier | QtCore.Qt.MetaModifier
_HOVER_COLOR = QtGui.QColor(255, 230, 60)

_controller = None


def StageView(usdviewApi):
    """usdview's StageView, or None (e.g. --norender)."""
    try:
        return usdviewApi._UsdviewApi__appController._stageView
    except AttributeError:
        return None


def _AppController(usdviewApi):
    return getattr(usdviewApi, "_UsdviewApi__appController", None)


def _Position(event, ratio):
    """Qt logical event coordinates -> physical pixels (Qt5 and Qt6)."""
    if hasattr(event, "position"):
        pos = event.position()
        x, y = pos.x(), pos.y()
    else:
        x, y = event.x(), event.y()
    return (x * ratio, y * ratio)


# ---------------------------------------------------------------------------
# Toolbar
# ---------------------------------------------------------------------------

class ViewportToolbar(QtWidgets.QToolBar):
    """
    Text-labelled tool buttons (the repo convention: no icons, tooltips
    explain), exclusive groups for tool / channels / write mode, undo and
    redo with application-wide shortcuts, and a status label that always
    says why there is no gizmo.
    """

    toolChanged = QtCore.Signal(str)
    channelsChanged = QtCore.Signal(str)
    writeModeChanged = QtCore.Signal(str)
    undoRequested = QtCore.Signal()
    redoRequested = QtCore.Signal()

    def __init__(self, parent):
        super(ViewportToolbar, self).__init__("RigExec Viewport Tools",
                                              parent)
        self.setObjectName("rigExecViewportTools")
        self.setMovable(False)
        self._toolActions = self._AddGroup(
            _TOOLS, self._onToolTriggered, TOOL_SELECT)
        self.addSeparator()
        self.addWidget(QtWidgets.QLabel(" Channels: "))
        self._channelActions = self._AddGroup(
            _CHANNELS, self._onChannelsTriggered, gizmoMath.CHANNELS_POSE)
        self.addSeparator()
        self.addWidget(QtWidgets.QLabel(" Write: "))
        self._writeActions = self._AddGroup(
            _WRITE_MODES, self._onWriteTriggered, gizmoMath.WRITE_ANIMATION)
        self.addSeparator()
        self._undoAction = QtActionWidgets.QAction("Undo", self)
        self._undoAction.setShortcut(QtGui.QKeySequence("Ctrl+Z"))
        self._undoAction.setShortcutContext(
            QtCore.Qt.ApplicationShortcut)
        self._undoAction.triggered.connect(self.undoRequested)
        self.addAction(self._undoAction)
        self._redoAction = QtActionWidgets.QAction("Redo", self)
        self._redoAction.setShortcuts([
            QtGui.QKeySequence("Ctrl+Shift+Z"),
            QtGui.QKeySequence(QtGui.QKeySequence.Redo)])
        self._redoAction.setShortcutContext(
            QtCore.Qt.ApplicationShortcut)
        self._redoAction.triggered.connect(self.redoRequested)
        self.addAction(self._redoAction)
        self.addSeparator()
        self._status = QtWidgets.QLabel("")
        self._status.setMinimumWidth(240)
        self.addWidget(self._status)
        self.SetUndoState(False, "", False, "")

    def _AddGroup(self, entries, slot, initial):
        group = QtActionWidgets.QActionGroup(self)
        group.setExclusive(True)
        actions = {}
        for key, label, tip in entries:
            action = QtActionWidgets.QAction(label, self)
            action.setCheckable(True)
            action.setToolTip(tip)
            action.setData(key)
            action.setChecked(key == initial)
            group.addAction(action)
            self.addAction(action)
            actions[key] = action
        group.triggered.connect(slot)
        return actions

    def _onToolTriggered(self, action):
        self.toolChanged.emit(action.data())

    def _onChannelsTriggered(self, action):
        self.channelsChanged.emit(action.data())

    def _onWriteTriggered(self, action):
        self.writeModeChanged.emit(action.data())

    @staticmethod
    def _Check(actions, key):
        action = actions.get(key)
        if action is not None and not action.isChecked():
            action.setChecked(True)

    def SetTool(self, tool):
        self._Check(self._toolActions, tool)

    def SetChannels(self, channels):
        self._Check(self._channelActions, channels)

    def SetWriteMode(self, mode):
        self._Check(self._writeActions, mode)

    def SetStatus(self, text):
        self._status.setText(text)

    def Status(self):
        return self._status.text()

    def SetUndoState(self, canUndo, undoText, canRedo, redoText):
        self._undoAction.setEnabled(canUndo)
        self._undoAction.setText("Undo %s" % undoText if undoText
                                 else "Undo")
        self._redoAction.setEnabled(canRedo)
        self._redoAction.setText("Redo %s" % redoText if redoText
                                 else "Redo")


# ---------------------------------------------------------------------------
# Overlay
# ---------------------------------------------------------------------------

class GizmoOverlay(QtWidgets.QWidget):
    """Transparent, mouse-transparent paint layer over the StageView."""

    def __init__(self, view, controller):
        super(GizmoOverlay, self).__init__(view)
        self._controller = controller
        self.setAttribute(QtCore.Qt.WA_TransparentForMouseEvents)
        self.setAttribute(QtCore.Qt.WA_NoSystemBackground)
        self.setAutoFillBackground(False)
        self.setGeometry(view.rect())
        self.show()

    def paintEvent(self, event):
        handles = self._controller.Handles()
        if not handles:
            return
        ratio = self._controller.PixelRatio()
        active = self._controller.ActiveHandleName()
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        for handle in handles:
            color = QtGui.QColor.fromRgbF(*handle.color)
            if handle.name == active:
                color = _HOVER_COLOR
            pen = QtGui.QPen(color, 2.0)
            painter.setPen(pen)
            points = [QtCore.QPointF(x / ratio, y / ratio)
                      for x, y in handle.points]
            if handle.kind == "axis":
                painter.drawLine(points[0], points[1])
                self._DrawCap(painter, points, color)
            elif handle.kind == "ring":
                painter.drawPolygon(QtGui.QPolygonF(points))
            else:
                half = gizmoScreen.CENTER_PIXELS
                painter.fillRect(QtCore.QRectF(
                    points[0].x() - half, points[0].y() - half,
                    2 * half, 2 * half), color)
        painter.end()

    def _DrawCap(self, painter, points, color):
        """Arrowhead for translate, square cap for scale."""
        a, b = points
        dx, dy = b.x() - a.x(), b.y() - a.y()
        length = math.hypot(dx, dy)
        if length < 1e-6:
            return
        ux, uy = dx / length, dy / length
        size = 7.0
        if self._controller.Tool() == TOOL_SCALE:
            painter.fillRect(QtCore.QRectF(
                b.x() - size / 2, b.y() - size / 2, size, size), color)
            return
        tip = b
        left = QtCore.QPointF(b.x() - ux * size * 1.6 + uy * size * 0.6,
                              b.y() - uy * size * 1.6 - ux * size * 0.6)
        right = QtCore.QPointF(b.x() - ux * size * 1.6 - uy * size * 0.6,
                               b.y() - uy * size * 1.6 + ux * size * 0.6)
        painter.setBrush(color)
        painter.drawPolygon(QtGui.QPolygonF([tip, left, right]))
        painter.setBrush(QtCore.Qt.NoBrush)


# ---------------------------------------------------------------------------
# Controller
# ---------------------------------------------------------------------------

class _Drag(object):
    def __init__(self, handle, press, camera, viewport, origin, recorder,
                 label):
        self.handle = handle
        self.press = press
        self.camera = camera
        self.viewport = viewport
        self.origin = origin
        self.recorder = recorder
        self.label = label


class GizmoController(QtCore.QObject):

    def __init__(self, usdviewApi, undoStack):
        super(GizmoController, self).__init__()
        self._api = usdviewApi
        self.undoStack = undoStack
        self._view = StageView(usdviewApi)
        self._tool = TOOL_SELECT
        self._channels = gizmoMath.CHANNELS_POSE
        self._write = gizmoMath.WRITE_ANIMATION
        self._target = None
        self._reason = ""
        self._warnings = []
        self._handles = []
        self._hover = None
        self._drag = None
        self._frame = None
        self._noticeKey = None

        self.toolbar = ViewportToolbar(usdviewApi.qMainWindow)
        self.toolbar.toolChanged.connect(self.SetTool)
        self.toolbar.channelsChanged.connect(self.SetChannels)
        self.toolbar.writeModeChanged.connect(self.SetWriteMode)
        self.toolbar.undoRequested.connect(self.Undo)
        self.toolbar.redoRequested.connect(self.Redo)
        appController = _AppController(usdviewApi)
        layout = appController._ui.glFrame.layout()
        layout.insertWidget(0, self.toolbar)

        self._overlay = GizmoOverlay(self._view, self)
        self._view.installEventFilter(self)
        self._view.signalFrustumChanged.connect(self._onViewChanged)

        dataModel = usdviewApi.dataModel
        dataModel.selection.signalPrimSelectionChanged.connect(
            self._onSelectionChanged)
        dataModel.currentFrameChanged.connect(self._onFrameChanged)
        dataModel.signalStageReplaced.connect(self._onStageReplaced)
        undoStack.AddListener(self._onStackChanged)
        self._ObserveStage(usdviewApi.stage)
        self._onStackChanged()
        self.RefreshTarget()

    # -- state ------------------------------------------------------------

    def Tool(self):
        return self._tool

    def SetTool(self, tool):
        self._AbortDrag()
        self._tool = tool
        self.toolbar.SetTool(tool)
        self.RefreshTarget()

    def Channels(self):
        return self._channels

    def SetChannels(self, channels):
        self._AbortDrag()
        self._channels = channels
        self.toolbar.SetChannels(channels)
        self.RefreshTarget()

    def WriteMode(self):
        return self._write

    def SetWriteMode(self, mode):
        self._AbortDrag()
        self._write = mode
        self.toolbar.SetWriteMode(mode)
        self.RefreshTarget()

    def Target(self):
        return self._target

    def Reason(self):
        return self._reason

    def Status(self):
        return self.toolbar.Status()

    def Handles(self):
        return self._handles

    def ActiveHandleName(self):
        if self._drag is not None:
            return self._drag.handle.name
        return self._hover

    def IsDragging(self):
        return self._drag is not None

    def PixelRatio(self):
        try:
            return float(self._view.devicePixelRatioF())
        except AttributeError:
            return 1.0

    def HandleScreenPositions(self):
        """{name: [(x, y), ...]} in LOGICAL pixels, for tests."""
        ratio = self.PixelRatio()
        return {h.name: [(x / ratio, y / ratio) for x, y in h.points]
                for h in self._handles}

    def SetVisible(self, visible):
        self.toolbar.setVisible(visible)
        self._overlay.setVisible(visible)
        if not visible:
            self.SetTool(TOOL_SELECT)

    def IsVisible(self):
        return self.toolbar.isVisible()

    # -- frame / stage --------------------------------------------------------

    def _Frame(self):
        """
        The current frame as a Usd.TimeCode. Prefers the frame delivered
        by currentFrameChanged: the data model's property is assigned
        AFTER the signal fires (rigExecUsdview.py:_FrameValue), so
        reading it from inside the handler is one scrub behind.
        """
        frame = self._frame if self._frame is not None else self._api.frame
        if isinstance(frame, Usd.TimeCode):
            return frame
        return Usd.TimeCode(float(frame))

    def _ObserveStage(self, stage):
        if self._noticeKey is not None:
            try:
                self._noticeKey.Revoke()
            except Exception:
                pass
            self._noticeKey = None
        if stage:
            self._noticeKey = Tf.Notice.Register(
                Usd.Notice.ObjectsChanged, self._onObjectsChanged, stage)

    def RefreshTarget(self):
        self._target = None
        self._reason = ""
        stage = self._api.stage
        if self._tool == TOOL_SELECT or not stage:
            self._UpdateHandles()
            self._UpdateStatus()
            return
        prim = self._api.dataModel.selection.getFocusPrim()
        writer = gizmoMath.Writer(stage, self._Frame(), self._write)
        target, reason = gizmoMath.MakeTarget(
            stage, prim, self._channels, writer)
        if target is not None:
            supported = {
                TOOL_TRANSLATE: target.supportsTranslate,
                TOOL_ROTATE: target.supportsRotate,
                TOOL_SCALE: target.supportsScale,
            }[self._tool]
            if not supported:
                reason = "%s: %s is not available in %s mode" % (
                    target.label, self._tool, self._channels)
                target = None
        self._target = target
        self._reason = reason
        self._UpdateHandles()
        self._UpdateStatus()

    def _UpdateHandles(self):
        self._handles = []
        if self._target is not None and self._view is not None:
            camera, _ = self._view.resolveCamera()
            if camera is not None:
                self._handles = gizmoScreen.BuildHandles(
                    self._tool, self._target.GizmoMatrix(), camera,
                    self._view.computeWindowViewport(), self.PixelRatio())
        self._overlay.update()

    def _UpdateStatus(self):
        if self._tool == TOOL_SELECT:
            text = "Select: usdview picking"
        elif self._target is None:
            text = self._reason or "nothing selected"
        else:
            text = "%s %s (%s, %s)" % (
                self._tool.capitalize(), self._target.label,
                self._channels, self._write)
            if self._warnings:
                text += " -- " + self._warnings[0]
        self.toolbar.SetStatus(text)

    # -- signals ----------------------------------------------------------

    def _onViewChanged(self):
        self._UpdateHandles()

    def _onSelectionChanged(self, added, removed):
        if self._drag is None:
            self.RefreshTarget()

    def _onFrameChanged(self, frame):
        self._frame = frame
        if self._drag is None:
            self.RefreshTarget()

    def _onStageReplaced(self):
        self._AbortDrag()
        self._frame = None
        self.undoStack.Clear()
        self._ObserveStage(self._api.stage)
        self.RefreshTarget()

    def _onObjectsChanged(self, notice, stage):
        # Our own drag writes arrive here synchronously; the drag already
        # refreshes the handles it needs, and re-resolving the target from
        # inside every Set() would double the cost of each mouse move.
        if self._drag is None:
            self.RefreshTarget()

    def _onStackChanged(self):
        stack = self.undoStack
        self.toolbar.SetUndoState(stack.CanUndo(), stack.UndoText(),
                                  stack.CanRedo(), stack.RedoText())

    # -- undo -------------------------------------------------------------

    def Undo(self):
        self._AbortDrag()
        if self.undoStack.Undo():
            self.RefreshTarget()
            self._api.UpdateViewport()

    def Redo(self):
        self._AbortDrag()
        if self.undoStack.Redo():
            self.RefreshTarget()
            self._api.UpdateViewport()

    # -- mouse ------------------------------------------------------------

    def eventFilter(self, obj, event):
        kind = event.type()
        if kind == QtCore.QEvent.Resize:
            self._overlay.setGeometry(self._view.rect())
            self._UpdateHandles()
            return False
        if kind == QtCore.QEvent.Paint:
            self._overlay.update()
            return False
        if self._tool == TOOL_SELECT or (self._target is None
                                         and self._drag is None):
            return False
        modifiers = getattr(event, "modifiers", None)
        if modifiers is not None and self._drag is None \
                and (modifiers() & _CAMERA_MODIFIERS):
            return False
        if kind == QtCore.QEvent.KeyPress:
            if self._drag is not None \
                    and event.key() == QtCore.Qt.Key_Escape:
                self._AbortDrag()
                self._api.UpdateViewport()
                return True
            return False
        if kind not in (QtCore.QEvent.MouseButtonPress,
                        QtCore.QEvent.MouseMove,
                        QtCore.QEvent.MouseButtonRelease):
            return False
        pos = _Position(event, self.PixelRatio())
        if kind == QtCore.QEvent.MouseButtonPress:
            if event.button() != QtCore.Qt.LeftButton or self._drag:
                return False
            handle = gizmoScreen.HitTest(
                self._handles, pos[0], pos[1],
                gizmoScreen.HIT_PIXELS * self.PixelRatio())
            if handle is None:
                return False
            self._BeginDrag(handle, pos)
            return True
        if kind == QtCore.QEvent.MouseMove:
            if self._drag is None:
                handle = gizmoScreen.HitTest(
                    self._handles, pos[0], pos[1],
                    gizmoScreen.HIT_PIXELS * self.PixelRatio())
                name = handle.name if handle else None
                if name != self._hover:
                    self._hover = name
                    self._overlay.update()
                return False
            self._UpdateDrag(pos)
            return True
        if self._drag is None:
            return False
        self._EndDrag()
        return True

    # -- drag -------------------------------------------------------------

    def _BeginDrag(self, handle, pos):
        target = self._target
        recorder = rigExecUndo.EditRecorder(
            self._api.stage, target.AttributePaths())
        recorder.Begin()
        target.BeginDrag()
        camera, _ = self._view.resolveCamera()
        label = "%s %s" % (self._tool.capitalize(), target.label)
        self._drag = _Drag(handle, pos, camera,
                           self._view.computeWindowViewport(),
                           Gf.Vec3d(target.GizmoMatrix().ExtractTranslation()),
                           recorder, label)
        self._warnings = []
        self._overlay.update()

    def _UpdateDrag(self, pos):
        drag = self._drag
        target = self._target
        handle = drag.handle
        if self._tool == TOOL_TRANSLATE:
            if handle.kind == "center":
                delta = gizmoScreen.PlaneDragDelta(
                    drag.camera, drag.viewport, drag.origin, drag.press, pos)
            else:
                t = gizmoScreen.AxisDragParameter(handle, drag.press, pos)
                delta = handle.worldAxis * (t * handle.worldLength)
            target.ApplyTranslate(delta)
        elif self._tool == TOOL_ROTATE:
            angle = gizmoScreen.RotationDragAngle(
                handle.center, drag.press, pos,
                gizmoScreen.AxisFacesCamera(drag.camera, handle.worldAxis))
            target.ApplyRotate(handle.worldAxis, angle)
        else:
            factor = gizmoScreen.ScaleDragFactor(
                handle, drag.press, pos, self.PixelRatio())
            target.ApplyScale(handle.axisIndex, factor)
        self._warnings = target.writer.Warnings()
        target.Refresh()
        self._UpdateHandles()
        self._UpdateStatus()
        self._api.UpdateViewport()

    def _EndDrag(self):
        drag = self._drag
        self._drag = None
        edit = drag.recorder.Commit(drag.label)
        if edit is not None:
            self.undoStack.Push(edit)
        self.RefreshTarget()

    def _AbortDrag(self):
        if self._drag is None:
            return
        drag = self._drag
        self._drag = None
        drag.recorder.Abort()
        self.RefreshTarget()


def InstallViewportTools(usdviewApi, undoStack):
    """Create the toolbar and controller once; None without a StageView."""
    global _controller
    if _controller is not None:
        return _controller
    if StageView(usdviewApi) is None or _AppController(usdviewApi) is None:
        return None
    _controller = GizmoController(usdviewApi, undoStack)
    return _controller


def GetController():
    return _controller
```

- [ ] **Step 2: Wire the container**

In `plugin/rigExecUsdview/rigExecUsdview.py`:

(a) In `registerPlugins`, after `self._activating = False`, add:

```python
        self._undoStack = None
        self._viewportTools = None
        self._viewportToolsWarned = False
```

(b) After the `self._curvenets = ...` registration add:

```python
        # The viewport toolbar (gizmos + undo). It attaches itself to the
        # StageView, which does not exist yet when plugins register
        # (appController.py: _configurePlugins runs before the view is
        # built), so it is installed on the first stage replacement and
        # this command only toggles it.
        self._viewportToolsCmd = plugRegistry.registerCommandPlugin(
            "RigExecUsdviewContainer.viewportTools",
            "Viewport Tools",
            lambda api: self._ToggleViewportTools())
```

(c) In `configureView` add `menu.addItem(self._viewportToolsCmd)` after the curvenets item.

(d) Add these methods next to `_OpenCurvenetPanel`:

```python
    def _UndoStack(self):
        """The undo stack shared by every RigExec viewport tool."""
        if self._undoStack is None:
            try:
                import rigExecUndo
            except ImportError:
                sys.path.insert(
                    0, os.path.dirname(os.path.abspath(__file__)))
                import rigExecUndo
            self._undoStack = rigExecUndo.UndoStack()
        return self._undoStack

    def _EnsureViewportTools(self):
        """
        Installs the toolbar above the viewport once the StageView exists.
        Returns the controller or None (headless, --norender, no Qt).
        """
        if self._viewportTools is not None:
            return self._viewportTools
        try:
            try:
                import gizmoUI
            except ImportError:
                sys.path.insert(
                    0, os.path.dirname(os.path.abspath(__file__)))
                import gizmoUI
            self._viewportTools = gizmoUI.InstallViewportTools(
                self._api, self._UndoStack())
        except Exception as error:
            if not self._viewportToolsWarned:
                self._viewportToolsWarned = True
                Tf.Warn("rigExecUsdview: viewport tools unavailable: %s"
                        % error)
            self._viewportTools = None
        return self._viewportTools

    def _ToggleViewportTools(self):
        controller = self._EnsureViewportTools()
        if controller is not None:
            controller.SetVisible(not controller.IsVisible())
```

(e) In `_OnStageReplaced`, after `self._ActivateCurrentStage()`, add `self._EnsureViewportTools()`.

- [ ] **Step 3: Smoke-test through testusdview**

Run the `/tmp/gizmo_smoke.py` script described above:

```bash
. bin/_env.sh && RIGEXEC_GIZMO_SHOT=/tmp/gizmo_smoke.png "$PY" "$TESTUSDVIEW" --testScript /tmp/gizmo_smoke.py examples/ArmShotAnim.usda
```

Expected: `target: <...RigPoseTarget...> reason: ''`, a `handles:` dict with `x`, `y`, `z`, `center`, and no traceback. Open `/tmp/gizmo_smoke.png` with the Read tool: the toolbar row (Select / Translate / Rotate / Scale / Channels / Write / Undo / Redo / status) must be above the viewport and the coloured axes visible at the HandIK control. If the overlay is invisible while the handles dict is populated, the QOpenGLWidget is covering the child: as a fallback set `self._overlay.raise_()` after each view paint in `eventFilter`, and if that still fails switch the overlay to painting inside the view by wrapping `view.paintGL` (call the original, then `QtGui.QPainter(view)` drawing the same handles). Record which approach worked in the module comment.

- [ ] **Step 4: Also confirm the headless C++ tests still pass** (the container module is imported there without Qt):

Run: `PYTHONPATH=/Users/burkard/work/usd-install/lib/python3.11/site-packages ctest --test-dir build --output-on-failure -R "testRigExecImaging|testRigExecArm"`
Expected: both pass.

- [ ] **Step 5: Commit**

```bash
git add plugin/rigExecUsdview/gizmoUI.py plugin/rigExecUsdview/rigExecUsdview.py
git commit -m "usdview: viewport toolbar with undoable translate/rotate/scale gizmo" -m "Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>" -m "Claude-Session: https://claude.ai/code/session_014a1ZRa9PujUhqJSZ5qWNYB"
```

---

### Task 6: testusdview integration test, runner script, docs

**Files:**
- Create: `tests/testUsdviewGizmo.py`
- Create: `bin/run_testusdview_gizmo.sh` (chmod +x)
- Create: `docs/viewport-gizmos.md`
- Modify: `README.md` (one bullet where the volume weight / curvenet panels are listed; find it with `grep -n "Curvenet Authoring\|Volume Weight" README.md`)

**Interfaces:**
- Consumes: Task 5 `gizmoUI.GetController()`, `gizmoUI.StageView()`, controller `SetTool/SetChannels/SetWriteMode/Target/Reason/Status/HandleScreenPositions/IsDragging`, `gizmoUI.TOOL_*`; Task 3 `gizmoMath.CHANNELS_*`, `WRITE_*`.
- Produces: the `RIGEXEC_GIZMO_OK` banner other scripts can grep for.

- [ ] **Step 1: Write the test script**

`tests/testUsdviewGizmo.py`:

```python
#
# testusdview script: the viewport gizmo toolbar end to end.
#
# Opens examples/ArmShotAnim.usda (the container installs the toolbar on
# stage load), selects the HandIK control, and drives synthetic mouse
# events through the gizmo's own projected handle positions, exactly as
# tests/testUsdviewCurvenetMove.py does for knots. Asserts what landed on
# the stage, that Ctrl+Z / Ctrl+Shift+Z round-trip it, and that the
# Default / Pivot / plain-Xform paths write where the spec says.
#
# Set RIGEXEC_GIZMO_SHOT=/path.png to save a window grab for inspection.
#
import os

from pxr import Gf, Sdf, Usd, UsdGeom

CONTROL = "/Shot/HeroArm/Rig/Controls/HandIK"
XFORM = "/Shot/HeroArm"


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _Mouse(QtCore, QtGui, view, kind, x, y, buttons=None, modifiers=None):
    pos = QtCore.QPointF(float(x), float(y))
    glob = view.mapToGlobal(QtCore.QPoint(int(x), int(y)))
    globF = QtCore.QPointF(float(glob.x()), float(glob.y()))
    if buttons is None:
        buttons = QtCore.Qt.LeftButton
    if modifiers is None:
        modifiers = QtCore.Qt.NoModifier
    return QtGui.QMouseEvent(kind, pos, globF, QtCore.Qt.LeftButton,
                             buttons, modifiers)


def _Lerp(a, b, t):
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)


class _Driver(object):
    def __init__(self, appController, controller):
        from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets
        import gizmoUI
        self.QtCore, self.QtGui, self.QtWidgets = QtCore, QtGui, QtWidgets
        self.app = appController
        self.api = appController._usdviewApi
        self.controller = controller
        self.view = gizmoUI.StageView(self.api)

    def Pump(self):
        self.app._processEvents()

    def Send(self, kind, x, y, **kw):
        self.QtWidgets.QApplication.sendEvent(
            self.view, _Mouse(self.QtCore, self.QtGui, self.view, kind,
                              x, y, **kw))
        self.Pump()

    def Drag(self, start, end, steps=4):
        QE = self.QtCore.QEvent.Type
        self.Send(QE.MouseButtonPress, *start)
        _Check(self.controller.IsDragging(),
               "press at %s did not start a drag (handles: %s)" % (
                   start, self.controller.HandleScreenPositions()))
        for i in range(1, steps + 1):
            self.Send(QE.MouseMove, *_Lerp(start, end, i / float(steps)))
        self.Send(QE.MouseButtonRelease, *end,
                  buttons=self.QtCore.Qt.NoButton)
        _Check(not self.controller.IsDragging(), "release ended the drag")

    def DragAxis(self, name, fraction=0.35):
        positions = self.controller.HandleScreenPositions()
        _Check(name in positions, "handle %r missing from %s" % (
            name, sorted(positions)))
        a, b = positions[name][0], positions[name][-1]
        self.Drag(_Lerp(a, b, 0.5), _Lerp(a, b, 0.5 + fraction))

    def DragRing(self, name, quarter=6):
        positions = self.controller.HandleScreenPositions()
        _Check(name in positions, "ring %r missing" % name)
        points = positions[name]
        self.Drag(points[0], points[quarter % len(points)])

    def Key(self, key, modifiers):
        try:
            from PySide6 import QtTest
        except ImportError:
            from PySide2 import QtTest
        self.view.setFocus()
        QtTest.QTest.keyClick(self.view, key, modifiers)
        self.Pump()

    def Select(self, path):
        prim = self.api.stage.GetPrimAtPath(path)
        _Check(prim, "missing prim %s" % path)
        self.api.ClearPrimSelection()
        self.api.AddPrimToSelection(prim)
        self.Pump()
        return prim


def testUsdviewInputFunction(appController):
    import gizmoMath
    import gizmoUI
    controller = gizmoUI.GetController()
    _Check(controller is not None,
           "the container did not install the viewport tools")
    d = _Driver(appController, controller)
    stage = d.api.stage
    session = stage.GetSessionLayer()
    _Check(stage.GetEditTarget().GetLayer() == session,
           "usdview's edit target is expected to be the session layer")
    frame = d.api.frame
    appController._frameSelection()
    d.Pump()

    # --- 1. Translate a control in Animation mode, undo, redo ------------
    prim = d.Select(CONTROL)
    controller.SetTool(gizmoUI.TOOL_TRANSLATE)
    d.Pump()
    _Check(controller.Target() is not None, controller.Reason())
    tx = prim.GetAttribute("avars:tx")
    before = tx.Get(frame)
    d.DragAxis("x")
    afterX = tx.Get(frame)
    _Check(abs(afterX - before) > 1e-6, "x drag changed avars:tx at the "
           "frame: %s -> %s" % (before, afterX))
    spec = session.GetAttributeAtPath(Sdf.Path(CONTROL + ".avars:tx"))
    _Check(spec is not None and spec.HasSpline(),
           "animation mode authored a spline knot in the session layer")
    _Check(controller.undoStack.CanUndo(), "drag pushed an edit")
    d.Key(d.QtCore.Qt.Key_Z, d.QtCore.Qt.ControlModifier)
    _Check(abs(tx.Get(frame) - before) < 1e-9,
           "Ctrl+Z restored avars:tx: %s" % tx.Get(frame))
    _Check(session.GetAttributeAtPath(Sdf.Path(CONTROL + ".avars:tx"))
           is None, "undo removed the session spec it created")
    d.Key(d.QtCore.Qt.Key_Z,
          d.QtCore.Qt.ControlModifier | d.QtCore.Qt.ShiftModifier)
    _Check(abs(tx.Get(frame) - afterX) < 1e-9,
           "Ctrl+Shift+Z re-applied avars:tx: %s" % tx.Get(frame))
    controller.Undo()
    d.Pump()

    # --- 2. Centre handle moves in the camera plane ----------------------
    positions = controller.HandleScreenPositions()
    c = positions["center"][0]
    ty = prim.GetAttribute("avars:ty")
    beforeTy = ty.Get(frame)
    d.Drag(c, (c[0] + 30, c[1] - 30))
    _Check(abs(ty.Get(frame) - beforeTy) > 1e-6
           or abs(tx.Get(frame) - before) > 1e-6, "centre drag moved")
    controller.Undo()
    d.Pump()

    # --- 3. Default mode writes the default and warns ---------------------
    controller.SetWriteMode(gizmoMath.WRITE_DEFAULT)
    d.Pump()
    d.DragAxis("x")
    spec = session.GetAttributeAtPath(Sdf.Path(CONTROL + ".avars:tx"))
    _Check(spec is not None and spec.HasDefaultValue()
           and not spec.HasSpline(),
           "default mode authored a default, not a knot")
    _Check("outranked" in controller.Status(),
           "status warns that the file's spline outranks the default: %r"
           % controller.Status())
    controller.Undo()
    d.Pump()
    controller.SetWriteMode(gizmoMath.WRITE_ANIMATION)

    # --- 4. Rotate and scale ---------------------------------------------
    controller.SetTool(gizmoUI.TOOL_ROTATE)
    d.Pump()
    _Check(controller.Target() is not None, controller.Reason())
    rz = [prim.GetAttribute(n).Get(frame) for n in gizmoMath.AVAR_R]
    d.DragRing("z")
    rzAfter = [prim.GetAttribute(n).Get(frame) for n in gizmoMath.AVAR_R]
    _Check(any(abs(a - b) > 1e-6 for a, b in zip(rz, rzAfter)),
           "ring drag changed a rotation avar: %s -> %s" % (rz, rzAfter))
    controller.Undo()
    d.Pump()
    controller.SetTool(gizmoUI.TOOL_SCALE)
    d.Pump()
    sx = prim.GetAttribute("avars:sx")
    d.DragAxis("x")
    _Check(abs(sx.Get(frame) - 1.0) > 1e-6, "scale drag changed avars:sx")
    controller.Undo()
    d.Pump()

    # --- 5. Pivot mode edits rest:*, never avars -------------------------
    controller.SetChannels(gizmoMath.CHANNELS_PIVOT)
    controller.SetTool(gizmoUI.TOOL_TRANSLATE)
    d.Pump()
    _Check(controller.Target() is not None, controller.Reason())
    restTx = prim.GetAttribute("rest:tx")
    d.DragAxis("x")
    _Check(abs(restTx.Get(frame)) > 1e-6, "pivot drag wrote rest:tx")
    _Check(abs(tx.Get(frame) - before) < 1e-9, "avars untouched by pivot")
    controller.SetTool(gizmoUI.TOOL_SCALE)
    d.Pump()
    _Check(controller.Target() is None and "not available" in
           controller.Reason(), "scale refused in pivot mode: %r"
           % controller.Reason())
    controller.Undo()
    d.Pump()
    controller.SetChannels(gizmoMath.CHANNELS_POSE)

    # --- 6. A plain Xform edits its xformOps ------------------------------
    xform = d.Select(XFORM)
    controller.SetTool(gizmoUI.TOOL_TRANSLATE)
    d.Pump()
    _Check(controller.Target() is not None
           and controller.Target().kind == "xform-pose", controller.Reason())
    d.DragAxis("y")
    op = session.GetAttributeAtPath(Sdf.Path(XFORM + ".xformOp:translate"))
    _Check(op is not None, "xformOp:translate authored in the session")
    controller.Undo()
    d.Pump()
    _Check(session.GetAttributeAtPath(
        Sdf.Path(XFORM + ".xformOp:translate")) is None, "xform undo")

    # --- 7. Select tool: a click on the old handle is not consumed --------
    controller.SetTool(gizmoUI.TOOL_SELECT)
    d.Pump()
    _Check(controller.HandleScreenPositions() == {}, "no handles")

    shot = os.environ.get("RIGEXEC_GIZMO_SHOT")
    if shot:
        controller.SetTool(gizmoUI.TOOL_TRANSLATE)
        d.Select(CONTROL)
        d.Pump()
        d.view.window().grab().save(shot)
    print("RIGEXEC_GIZMO_OK translate/rotate/scale, undo/redo, default, "
          "pivot, xform")
```

- [ ] **Step 2: Write the runner**

`bin/run_testusdview_gizmo.sh`:

```bash
#!/bin/bash
# bin/run_testusdview_gizmo.sh -- headless end-to-end test of the viewport
# gizmo toolbar (tests/testUsdviewGizmo.py) on examples/ArmShotAnim.usda.
#
# Usage: bin/run_testusdview_gizmo.sh [rendererDisplayName]
# Set RIGEXEC_GIZMO_SHOT=/path.png to keep a window grab.
set -euo pipefail
. "$(dirname "${BASH_SOURCE[0]:-$0}")/_env.sh"

rigexec_require_python
rigexec_require_usd "$TESTUSDVIEW"
rigexec_build

STAGE="$RIG/examples/ArmShotAnim.usda"
rigexec_require_stage "$STAGE"

if [ $# -gt 0 ] && [ "${1#-}" = "$1" ]; then
    set -- --renderer "$@"
fi

exec "$PY" "$TESTUSDVIEW" --testScript "$RIG/tests/testUsdviewGizmo.py" \
     "$@" "$STAGE"
```

- [ ] **Step 3: Run it**

Run: `RIGEXEC_GIZMO_SHOT=/tmp/gizmo_final.png bin/run_testusdview_gizmo.sh`
Expected: the last line is `RIGEXEC_GIZMO_OK ...` and the process exits 0. Look at `/tmp/gizmo_final.png` with the Read tool: toolbar above the viewport, gizmo axes on the HandIK control. Fix whatever fails in the module it belongs to (the assertion messages name the layer, attribute and mode).

- [ ] **Step 4: Docs**

`docs/viewport-gizmos.md`:

```markdown
# Viewport gizmos (usdview)

The RigExec usdview plugin adds a toolbar above the viewport
(`RigExec → Viewport Tools` toggles it) with an undoable
translate / rotate / scale gizmo.

## What it edits

| Selection | Pose channels | Pivot channels |
|---|---|---|
| `RigExecControl`, `RigExecJoint` | `avars:tx/ty/tz`, `avars:rx/ry/rz`, `avars:sx/sy/sz` | `rest:tx/ty/tz`, `rest:rx/ry/rz` (no scale: rest spaces are orthonormalized) |
| any `UsdGeomXformable` with an `XformCommonAPI` op stack | translate / rotate / scale ops | `xformOp:translate:pivot` (translate only) |

The gizmo sits at the evaluated frame: for a rig prim that is
`avars * rest * parentRest^-1 * parentPosed` placed by the asset root's
world transform (`libs/rigExec/computations.cpp`), so a rest offset or
an animated parent is already accounted for. Its axes are the target's
local frame, the one the avars are expressed in.

**Write** chooses where the value lands: *Animation* authors a spline
knot (or a time sample for vector ops) at the current frame, exactly like
the volume weight panel; *Default* authors the attribute's default. A
default is invisible while a spline or time samples exist on that
attribute, and the status label says so.

Everything is authored into the current edit target (usdview starts on
the session layer).

## Undo

Every drag is one undo step. `Ctrl+Z` undoes, `Ctrl+Shift+Z` (and the
platform redo key) redoes, from anywhere in the window; the toolbar's
Undo / Redo buttons show the step's label. Undo restores the exact
attribute spec in the layer that was edited, removing it when the drag
created it. `Escape` during a drag aborts it. The stack is cleared when
the stage is replaced.

## Not editable

The status label explains when there is no gizmo: a joint posed by a
solver (`rigExec:joints`), a prim with a connected or authored
`posed:space`, a connected avar, an xformOp stack `XformCommonAPI`
cannot represent, or nothing selected. A control revised by a
constraint (`rigExec:moves`) shows the gizmo at its unconstrained frame.

## Tests

- `bin/run_python_tests.sh` — headless: undo snapshots, the frame
  replica against the native evaluator, edit targets, screen math.
- `bin/run_testusdview_gizmo.sh` — testusdview end to end; prints
  `RIGEXEC_GIZMO_OK`.
```

Add a bullet to README.md next to the panels list, e.g.:

`- **Viewport Tools** (`RigExec → Viewport Tools`): undoable translate/rotate/scale gizmo for controls, joints and xforms; see `docs/viewport-gizmos.md`.`

- [ ] **Step 5: Commit**

```bash
git add tests/testUsdviewGizmo.py bin/run_testusdview_gizmo.sh docs/viewport-gizmos.md README.md
git commit -m "usdview: gizmo toolbar end-to-end test and docs" -m "Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>" -m "Claude-Session: https://claude.ai/code/session_014a1ZRa9PujUhqJSZ5qWNYB"
```
