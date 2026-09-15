#!/usr/bin/env python
"""
Headless test for plugin/rigExecUsdview/pickerScene.py: reading control
pickers out of a stage as scene data.

The picker is RigExecPicker prims in the rig's layer stack, not a sidecar
file. The whole reason for one prim per button is that an artist can then
override ONE of them in a layer they own, so that is what this asserts:
move, recolour, relabel, resize, retarget and deactivate a single button,
and deactivate a whole panel or a whole picker, all from a weaker-loses
opinion that never touches the published layer.

It also pins the discovery rule. Pickers are found BY SCHEMA -- `IsA`, so
a subclass is still a picker -- and never by path or filename, which is
what lets a stage carrying two characters produce two tabs with nothing
configured anywhere.

Everything runs on a stage built here: no Qt, no usdview, no rig.

Usage: test_picker_scene.py [schema resource dir]
"""
import sys

import rigexec_test_env

rigexec_test_env.SetupPluginTest()

from pxr import Gf, Plug, Sdf, Tf, Usd  # noqa: E402

if len(sys.argv) > 1:
    Plug.Registry().RegisterPlugins(sys.argv[1])
assert Tf.Type.FindByName("RigExecPicker") != Tf.Type.Unknown, (
    "RigExecPicker schema is not registered -- pass the generated schema "
    "resource dir as argv[1]")

import pickerScene  # noqa: E402

FAILURES = []


def Check(condition, message):
    if not condition:
        FAILURES.append(message)
        print("  FAIL %s" % message)
    else:
        print("  ok   %s" % message)


# ---------------------------------------------------------------- setup

CTL = "/Char/Rig/Controls/hand_ctl"
CTL2 = "/Char/Rig/Controls/foot_ctl"


def BuildStage():
    """A two-control rig and a picker with three buttons over it."""
    stage = Usd.Stage.CreateInMemory()
    stage.DefinePrim("/Char", "Xform")
    stage.DefinePrim("/Char/Rig", "RigExecRoot")
    stage.DefinePrim("/Char/Rig/Controls", "Scope")
    stage.DefinePrim(CTL, "RigExecControl")
    stage.DefinePrim(CTL2, "RigExecControl")
    # A joint is selectable too: the biped's fingers are posed as joints.
    stage.DefinePrim("/Char/Rig/Joints", "Scope")
    stage.DefinePrim("/Char/Rig/Joints/finger", "RigExecJoint")

    picker = stage.DefinePrim("/Char/Picker", "RigExecPicker")
    picker.GetRelationship("rigExec:picker:rig").SetTargets(
        [Sdf.Path("/Char/Rig")])
    picker.GetAttribute("ui:label").Set("Hero")

    body = stage.DefinePrim("/Char/Picker/Body", "RigExecPickerPanel")
    body.GetAttribute("ui:order").Set(0)
    body.GetAttribute("ui:size").Set(Gf.Vec2f(300.0, 400.0))

    face = stage.DefinePrim("/Char/Picker/Face", "RigExecPickerPanel")
    face.GetAttribute("ui:order").Set(1)

    hand = stage.DefinePrim("/Char/Picker/Body/b_hand",
                            "RigExecPickerButton")
    hand.GetAttribute("ui:position").Set(Gf.Vec2f(10.0, 20.0))
    hand.GetAttribute("ui:size").Set(Gf.Vec2f(30.0, 40.0))
    hand.GetAttribute("ui:fill").Set(Gf.Vec4f(0.0, 1.0, 0.0, 1.0))
    hand.GetAttribute("ui:text").Set("Hand")
    hand.GetRelationship("rigExec:picker:controls").SetTargets(
        [Sdf.Path(CTL)])

    finger = stage.DefinePrim("/Char/Picker/Body/b_finger",
                              "RigExecPickerButton")
    finger.GetAttribute("ui:position").Set(Gf.Vec2f(50.0, 20.0))
    finger.GetRelationship("rigExec:picker:controls").SetTargets(
        [Sdf.Path("/Char/Rig/Joints/finger")])

    # A backdrop: no targets of any kind, so it draws and never clicks.
    deco = stage.DefinePrim("/Char/Picker/Body/d_back",
                            "RigExecPickerButton")
    deco.GetAttribute("ui:size").Set(Gf.Vec2f(300.0, 400.0))
    deco.GetAttribute("ui:depth").Set(-1.0)

    # A button whose control was deleted from the rig: authored targets
    # that do not resolve. Dead, and distinct from a decoration.
    gone = stage.DefinePrim("/Char/Picker/Body/b_gone",
                            "RigExecPickerButton")
    gone.GetRelationship("rigExec:picker:controls").SetTargets(
        [Sdf.Path("/Char/Rig/Controls/never_built")])
    return stage


def Only(stage):
    pickers = pickerScene.load_all(stage)
    assert len(pickers) == 1, "expected one picker, got %d" % len(pickers)
    return pickers[0]


def Button(picker, name, panel):
    # Names are unique within a PANEL, not across a picker.
    return next((b for b in picker.buttons
                 if b.id == name and b.parent == panel), None)


# ------------------------------------------------------------ discovery

print("discovery")
stage = BuildStage()
prims = pickerScene.find(stage)
Check(len(prims) == 1, "one RigExecPicker found, got %d" % len(prims))
Check(str(prims[0].GetPath()) == "/Char/Picker",
      "at the right path: %s" % prims[0].GetPath())

rig = pickerScene.rig_for(prims[0])
Check(rig is not None and str(rig.GetPath()) == "/Char/Rig",
      "and it names the rig it drives: %s" % (rig and rig.GetPath()))

picker = Only(stage)
Check(picker.name == "Hero", "ui:label names the tab, got %r" % picker.name)
Check([p.label for p in picker.panels] == ["Body", "Face"],
      "panels in ui:order, got %s" % [p.label for p in picker.panels])

BODY = "/Char/Picker/Body"
hand = Button(picker, "b_hand", BODY)
Check(hand is not None and hand.live, "a button over a control is live")
Check(Button(picker, "b_finger", BODY).live,
      "a button over a posed JOINT is live too")
Check(not Button(picker, "b_gone", BODY).live,
      "a button whose control was never built is dead")
Check(Button(picker, "d_back", BODY).decoration,
      "a button with no targets at all is a decoration")
Check(not Button(picker, "b_gone", BODY).decoration,
      "and a dead button is NOT a decoration -- it named something")

live, dead, deco = picker.coverage()
Check((live, dead, deco) == (2, 1, 1),
      "coverage counts them apart: %s" % ((live, dead, deco),))

# A picker that names no rig still resolves, against the whole stage.
with Usd.EditContext(stage, Usd.EditTarget(stage.GetSessionLayer())):
    stage.OverridePrim("/Char/Picker").GetRelationship(
        "rigExec:picker:rig").SetTargets([])
    Check(Only(stage).coverage()[0] == 2,
          "a picker naming no rig falls back to the whole stage")
stage.GetSessionLayer().Clear()

# A picker may live anywhere, including INSIDE the RigExecRoot. Where it
# sits is a placement decision for whoever builds the rig; discovery
# prunes rig GRAPH (joints, controls, solvers, movers) rather than the
# rig root, so both layouts work and neither is faster to find.
with Usd.EditContext(stage, Usd.EditTarget(stage.GetSessionLayer())):
    stage.OverridePrim("/Char/Picker").SetActive(False)
    inside = stage.DefinePrim("/Char/Rig/Picker", "RigExecPicker")
    inside.GetRelationship("rigExec:picker:rig").SetTargets(
        [Sdf.Path("/Char/Rig")])
    pane = stage.DefinePrim("/Char/Rig/Picker/Body", "RigExecPickerPanel")
    btn = stage.DefinePrim("/Char/Rig/Picker/Body/b_in", "RigExecPickerButton")
    btn.GetRelationship("rigExec:picker:controls").SetTargets([Sdf.Path(CTL)])
    got = pickerScene.find(stage)
    Check(len(got) == 1 and str(got[0].GetPath()) == "/Char/Rig/Picker",
          "a picker INSIDE the rig root is found: %s"
          % [str(p.GetPath()) for p in got])
    Check(Only(stage).buttons[0].live,
          "and its buttons resolve from there")

    # ...but never inside rig graph, which is what the pruning buys.
    deep = stage.DefinePrim(CTL + "/Picker", "RigExecPicker")
    Check(len(pickerScene.find(stage)) == 1,
          "a picker parented under a CONTROL is pruned, not found")
    stage.RemovePrim(deep.GetPath())
stage.GetSessionLayer().Clear()

# ------------------------------------------------------------ overrides

print("")
print("overrides, authored in a layer the artist owns")
session = Usd.EditTarget(stage.GetSessionLayer())
with Usd.EditContext(stage, session):
    over = stage.OverridePrim(Sdf.Path(BODY).AppendChild("b_hand"))

    over.GetAttribute("ui:position").Set(Gf.Vec2f(111.0, 222.0))
    got = Button(Only(stage), "b_hand", BODY)
    Check((got.x, got.y) == (111.0, 222.0),
          "moved: (%g, %g)" % (got.x, got.y))

    over.GetAttribute("ui:size").Set(Gf.Vec2f(7.0, 8.0))
    got = Button(Only(stage), "b_hand", BODY)
    Check((got.w, got.h) == (7.0, 8.0), "resized: (%g, %g)" % (got.w, got.h))

    over.GetAttribute("ui:fill").Set(Gf.Vec4f(1.0, 0.0, 0.5, 1.0))
    got = Button(Only(stage), "b_hand", BODY)
    Check(list(got.fill) == [255, 0, 128, 255],
          "recoloured: %s" % list(got.fill))

    over.GetAttribute("ui:text").Set("MINE")
    over.GetAttribute("ui:fontSize").Set(14.0)
    over.GetAttribute("ui:bold").Set(True)
    got = Button(Only(stage), "b_hand", BODY)
    Check(got.text == "MINE" and got.font_size == 14.0 and got.bold,
          "relabelled: %r %gpt bold=%s"
          % (got.text, got.font_size, got.bold))

    over.GetAttribute("ui:shape").Set("circle")
    Check(Button(Only(stage), "b_hand", BODY).shape == "circle",
          "reshaped to a circle")

    over.GetRelationship("rigExec:picker:controls").SetTargets(
        [Sdf.Path(CTL2)])
    got = Button(Only(stage), "b_hand", BODY)
    Check(got.targets == [CTL2] and got.live,
          "retargeted, and still live: %s" % got.targets)

    # The headline case: take a button away without editing the rig.
    over.SetActive(False)
    after = Only(stage)
    Check(Button(after, "b_hand", BODY) is None,
          "deactivated: the button is gone from the model")
    Check(all(b.id != "b_hand" for b in after.visible(BODY)),
          "and it is not drawn")
    over.SetActive(True)
    Check(Button(Only(stage), "b_hand", BODY) is not None,
          "reactivated: it comes back")

    # A dead button can be RESCUED by an override rather than removed.
    rescue = stage.OverridePrim(Sdf.Path(BODY).AppendChild("b_gone"))
    rescue.GetRelationship("rigExec:picker:controls").SetTargets(
        [Sdf.Path(CTL)])
    Check(Button(Only(stage), "b_gone", BODY).live,
          "a dead button retargeted at a real control goes live")

    # Whole panels and whole pickers, same mechanism.
    stage.OverridePrim(Sdf.Path(BODY)).SetActive(False)
    Check([p.label for p in Only(stage).panels] == ["Face"],
          "a deactivated PANEL drops out of the tab list")
    stage.OverridePrim(Sdf.Path(BODY)).SetActive(True)

    stage.OverridePrim("/Char/Picker").SetActive(False)
    Check(pickerScene.find(stage) == [],
          "a deactivated PICKER is not found at all")
    stage.OverridePrim("/Char/Picker").SetActive(True)

stage.GetSessionLayer().Clear()

# --------------------------------------------------- more than one tab

print("")
print("two characters, two tabs")
with Usd.EditContext(stage, session):
    stage.DefinePrim("/Extra", "Xform")
    stage.DefinePrim("/Extra/Rig", "RigExecRoot")
    stage.DefinePrim("/Extra/Rig/head_ctl", "RigExecControl")
    second = stage.DefinePrim("/Extra/Picker", "RigExecPicker")
    second.GetRelationship("rigExec:picker:rig").SetTargets(
        [Sdf.Path("/Extra/Rig")])
    pane = stage.DefinePrim("/Extra/Picker/Body", "RigExecPickerPanel")
    btn = stage.DefinePrim("/Extra/Picker/Body/b_head",
                           "RigExecPickerButton")
    btn.GetRelationship("rigExec:picker:controls").SetTargets(
        [Sdf.Path("/Extra/Rig/head_ctl")])

    both = pickerScene.load_all(stage)
    Check(len(both) == 2, "two pickers found, got %d" % len(both))
    Check([p.name for p in both] == ["Hero", "Picker"],
          "named for the prim when ui:label is unauthored: %s"
          % [p.name for p in both])
    Check(both[1].buttons[0].live,
          "the second character's button resolves against ITS OWN rig")

    # Each resolves inside the rig it names, so one character's controls
    # cannot make the other character's buttons look alive.
    cross = stage.OverridePrim("/Extra/Picker/Body/b_head")
    cross.GetRelationship("rigExec:picker:controls").SetTargets(
        [Sdf.Path(CTL)])
    Check(not pickerScene.load_all(stage)[1].buttons[0].live,
          "a button pointing into the OTHER character's rig is not live")

stage.GetSessionLayer().Clear()

print("")
if FAILURES:
    print("FAILED (%d)" % len(FAILURES))
    for message in FAILURES:
        print("  %s" % message)
    sys.exit(1)
print("test_picker_scene: all checks passed")
