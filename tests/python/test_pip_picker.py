#!/usr/bin/env python
"""Pip's shipped picker pages, read headless from the example stage.

The Body and Face pages in examples/2d/rubberhose/rubberhose_picker.usda
are generated scene data, and the generator drifts: a renamed control
silently kills its button, a moved button silently covers another. This
pins what "a working Pip picker" means -- every animator control
selectable, no dead buttons, no overlaps, mirrors paired -- through the
same pickerScene entry points the usdview panel uses.

Usage: test_pip_picker.py [schema resource dir]
"""
import pathlib
import sys

import rigexec_test_env

rigexec_test_env.SetupPluginTest()

from pxr import Plug, Tf, Usd  # noqa: E402

if len(sys.argv) > 1:
    Plug.Registry().RegisterPlugins(sys.argv[1])
assert Tf.Type.FindByName("RigExecPicker") != Tf.Type.Unknown, (
    "RigExecPicker schema is not registered -- pass the generated schema "
    "resource dir as argv[1]")

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]
                       / "plugin" / "rigExecUsdview"))
import pickerScene  # noqa: E402

FAILURES = []


def Check(condition, message):
    if not condition:
        FAILURES.append(message)
        print("  FAIL %s" % message)
    else:
        print("  ok   %s" % message)


STAGE = (pathlib.Path(__file__).resolve().parents[2] / "examples" / "2d"
         / "rubberhose" / "rubberhose_anim.usda")
if not STAGE.exists():
    print("(no %s, skipped)" % STAGE)
    sys.exit(0)

stage = Usd.Stage.Open(str(STAGE))
stack = [pathlib.Path(layer.identifier).name
         for layer in stage.GetLayerStack()]
Check("rubberhose_picker.usda" in stack,
      "the picker layer rides the rig's stack: %s" % stack)

pickers = pickerScene.load_all(stage)
Check(len(pickers) == 1, "one picker, got %d" % len(pickers))
if len(pickers) != 1:
    print("FAILED")
    sys.exit(1)
picker = pickers[0]
Check(picker.name == "Pip", "the tab is Pip, got %r" % picker.name)
Check([p.label for p in picker.panels] == ["Body", "Face"],
      "Body then Face, got %s" % [p.label for p in picker.panels])

live, dead, deco = picker.coverage()
Check(dead == 0, "no dead buttons (%d live, %d deco)" % (live, deco))

controls = set()
hidden = set()
for prim in stage.Traverse():
    if prim.GetTypeName() != "RigExecControl":
        continue
    path = str(prim.GetPath())
    controls.add(path)
    scale = prim.GetAttribute("guide:scaleX").Get()
    if scale is not None and scale == 0.0:
        hidden.add(path)

covered = set()
all_controls = True
for button in picker.buttons:
    for target in button.targets:
        covered.add(target)
        prim = stage.GetPrimAtPath(target)
        if not prim or prim.GetTypeName() != "RigExecControl":
            all_controls = False
Check(all_controls, "every button target is a RigExecControl")
Check(not (covered - controls),
      "no button points outside the rig: %s"
      % sorted(covered - controls))
Check(not ((controls - hidden) - covered),
      "every animator control has a button: missing %s"
      % sorted((controls - hidden) - covered))
Check(not (covered & hidden),
      "hidden solver helpers have no buttons: %s"
      % sorted(covered & hidden))

for panel in picker.panels:
    shown = [b for b in picker.visible(panel.id) if not b.decoration]
    clash = None
    for i, first in enumerate(shown):
        for second in shown[i + 1:]:
            if (first.x < second.x + second.w
                    and second.x < first.x + first.w
                    and first.y < second.y + second.h
                    and second.y < first.y + first.h):
                clash = (first.id, second.id)
    Check(clash is None, "%s: no live-button overlap%s"
          % (panel.label, "" if clash is None else ", %s on %s" % clash))

    unreachable = []
    for button in shown:
        hits = picker.hits(panel.id, button.x + button.w / 2.0,
                           button.y + button.h / 2.0, {}, False)
        if button not in hits:
            unreachable.append(button.id)
    Check(not unreachable, "%s: every live button clickable%s"
          % (panel.label,
             "" if not unreachable else ", missed %s" % unreachable[:4]))

broken = []
for button in picker.buttons:
    if not button.mirror:
        continue
    other = next((o for o in picker.buttons
                  if o.parent == button.parent
                  and button.parent + "/" + o.id == button.mirror), None)
    if other is None or other.mirror != button.parent + "/" + button.id:
        broken.append(button.id)
Check(not broken, "mirrors pair both ways: %s" % broken[:4])

print("")
if FAILURES:
    print("FAILED (%d)" % len(FAILURES))
    for message in FAILURES:
        print("  %s" % message)
    sys.exit(1)
print("test_pip_picker: all checks passed")
