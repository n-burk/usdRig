#!/usr/bin/env python
"""
Space switches nested under other switched controls: one answer in every
evaluator.

The dynamic walk resolves switches round by round. A round reads the seed
standing at that point, so a switch reads a control whose own switch
resolves in the same round or later BEFORE that switch is applied. The
compiled graph binds each of those reads to the same version at Build:

    /Rig/Controls/Other
    /Rig/Controls/P          pSpaces: [P/S/C, world]
    /Rig/Controls/P/S        sSpaces: [Other, world]
    /Rig/Controls/P/S/C

S resolves first (its sources are not under a switched control), P second
(its source C sits under S). S reads its parent P before P's switch, and P
reads C after S's. Reading P's switched frame instead closes a cycle
between the two compose groups.

The contract:
  * the rig is bakeable, the program answers every generation, and the
    program and the walk agree bit for bit on every frame -- with the
    active index keyed through 0, 1, 0.5 and 0, so each switch really
    changes space and blends across a source that would close the cycle;
  * the same holds for the activeSpaceAttribute form, for a drag of the
    index through the interactive override path, and for the same nesting
    built into the biped;
  * a switch reading a control under its OWN target is a genuine cycle, and
    the common graph sets its members aside while siblings continue.

Usage: test_rigexec_space_switch_versions.py [<schema resources dir> [<examples dir>]]
"""
import os
import sys

from test_rigexec_python import _setup_environment
_setup_environment()

from pxr import Plug, Sdf, Usd  # noqa: E402

import rigexec  # noqa: E402

RIG = "/Rig"
FRAMES = (0.0, 1.0, 2.0, 3.0)
# Keyed per frame: each switch leaves source 0, reaches world, blends back
# across the half-way point and returns.
P_ACTIVE = (0.0, 1.0, 0.5, 0.0)
S_ACTIVE = (0.5, 0.0, 1.0, 0.5)
OTHER_TX = (0.0, 10.0, 20.0, 30.0)
P_RZ = (0.0, 15.0, -10.0, 5.0)
MODES = ("graph",)


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _Translate(x, y, z):
    """Row-vector identity rotation with translation in the last row."""
    return [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, x, y, z, 1]


def _Key(attribute, values):
    for frame, value in zip(FRAMES, values):
        attribute.Set(value, Usd.TimeCode(frame))


def _Switch(stage, name, target, sources):
    switch = stage.DefinePrim("/Rig/Movers/" + name, "RigExecSpaceSwitch")
    switch.CreateRelationship("rigExec:target").SetTargets([target])
    switch.CreateRelationship("rigExec:sources").SetTargets(sources)
    return switch


def _Avar(stage, path, name):
    return stage.GetPrimAtPath(path).CreateAttribute(
        "avars:" + name, Sdf.ValueTypeNames.Double)


class _Nested(object):
    """The rig in the module docstring. `dial` reads each index from a
    property on the target instead of inputs:activeSpace. `p_source` "Other"
    puts both switches in one round: no cycle to close, but S still reads P
    before P's switch, which the program has to reproduce bit for bit.
    `held` holds each index and Other still, so that only P's avars move
    from frame to frame."""

    def __init__(self, mode, dial=False, p_source="C", held=False):
        self.stage = Usd.Stage.CreateInMemory("nestedSpaces.usda")
        builder = rigexec.Builder.create(self.stage, RIG, "Test")
        other = builder.add_control("Other", _Translate(50.0, 100.0, 0.0))
        p = builder.add_control("P", _Translate(0.0, 100.0, 0.0))
        s = builder.add_control("S", _Translate(10.0, 0.0, 0.0), p)
        c = builder.add_control("C", _Translate(5.0, 0.0, 0.0), s)
        self.paths = {"Other": other.path, "P": p.path, "S": s.path,
                      "C": c.path}
        self.switches = {}
        self.actives = {}
        for name, target, source, keys in (
                ("sSpaces", s.path, other.path, S_ACTIVE),
                ("pSpaces", p.path, self.paths[p_source], P_ACTIVE)):
            # The rig root is not a provider: it is spelled "world".
            switch = _Switch(self.stage, name, target, [source, RIG])
            if dial:
                active = self.stage.GetPrimAtPath(target).CreateAttribute(
                    "spaces:active", Sdf.ValueTypeNames.Double)
                switch.CreateRelationship(
                    "rigExec:activeSpaceAttribute").SetTargets(
                        [active.GetPath()])
            else:
                active = switch.CreateAttribute(
                    "inputs:activeSpace", Sdf.ValueTypeNames.Double)
            if held:
                active.Set(keys[0])
            else:
                _Key(active, keys)
            self.switches[name] = switch
            self.actives[name] = active
        if held:
            _Avar(self.stage, other.path, "tx").Set(OTHER_TX[1])
        else:
            _Key(_Avar(self.stage, other.path, "tx"), OTHER_TX)
        _Key(_Avar(self.stage, p.path, "rz"), P_RZ)
        self.rig = rigexec.Rig(self.stage, RIG)
        self.rig.compile()
        self.rig.cpu_reference = True
        self.mode = mode


def _Frames(rig, paths, time):
    """Every named control's world matrix, as exact tuples."""
    pose = rig.evaluate(time)
    _Check(pose.valid, "invalid pose at %g: %s" % (time, pose.diagnostics))
    return pose, dict((name, tuple(pose.control_frame(path).to_matrix4()))
                      for name, path in paths.items())


def _CheckProgramAnswered(rig, what):
    _Check(len(rig.op_graph()) > 0, "%s: no compiled graph" % what)
    _Check(len(rig.op_graph()) > 0,
           "%s: the program did not answer the generation" % what)


def _CheckPose(pose, what):
    _Check(pose.valid,"pose publication valid; authored numeric assertions follow")


def _RunAllModes(make, paths_of, frames=FRAMES, what="nested"):
    """One compiled graph checked against authored world-space results.

    Callers below separately assert authored world-space matrix results.
    """
    fixture=make("graph")
    per_frame=[]
    for time in frames:
        label="%s frame %g" % (what,time)
        pose,frames_now=_Frames(fixture.rig,paths_of(fixture),time)
        _CheckProgramAnswered(fixture.rig,label)
        _CheckPose(pose,label)
        per_frame.append(frames_now)
    return per_frame


def _Origin(matrix):
    return (matrix[12], matrix[13], matrix[14])


def _Near(a, b, tolerance=1e-6):
    return all(abs(x - y) <= tolerance for x, y in zip(a, b))


def TestNestedSwitchBakesAndAgrees():
    dynamic = _RunAllModes(lambda mode: _Nested(mode),
                           lambda fixture: fixture.paths)
    # Frame 0: S halfway between Other and world, P fully in C's space.
    # Frame 1: P in world, so it keeps its own rest translation; S in Other.
    _Check(_Near(_Origin(dynamic[1]["P"]), (0.0, 100.0, 0.0)),
           "P in world holds its rest: %s" % (_Origin(dynamic[1]["P"]),))
    expected={"P": ((0,100,0),(0,100,0),(0,100,0),(15,100,0)),
              "S": ((10,100,0),(20,100,0),(10,100,0),(25,100,0)),
              "C": ((15,100,0),(25,100,0),(15,100,0),(30,100,0))}
    for name,origins in expected.items():
        for frame,origin in enumerate(origins):
            _Check(_Near(_Origin(dynamic[frame][name]),origin),
                   "%s frame%d expected%s got%s" %
                   (name,frame,origin,_Origin(dynamic[frame][name])))
    # The spaces really moved something: P differs between frames 0 and 3,
    # which differ only in the time-varying Other.tx and P.rz.
    _Check(_Origin(dynamic[0]["P"]) != _Origin(dynamic[3]["P"]),
           "P follows C across frames")


def TestSameRoundReadsThePreSwitchParent():
    _RunAllModes(lambda mode: _Nested(mode, p_source="Other"),
                 lambda fixture: fixture.paths, what="same round")


def TestRecomposedVersionFollowsItsAvars():
    """With the indices held, nothing but avars moves. S's step recomposes
    P's pre-switch frame from P's avars, so moving P must re-run S's step
    too -- the program skips everything outside the cone of what moved."""
    for p_source in ("C", "Other"):
        _RunAllModes(
            lambda mode: _Nested(mode, p_source=p_source, held=True),
            lambda fixture: fixture.paths,
            what="held indices, P in %s's space" % p_source)


def TestDialFormBakesAndAgrees():
    _RunAllModes(lambda mode: _Nested(mode, dial=True),
                 lambda fixture: fixture.paths, what="nested dial")


def TestDragOfTheIndexAgrees():
    """Interactive indices reach the same typed values as the independent reference."""
    for dial in (False,True):
        fixture=_Nested("graph",dial=dial)
        for name,value in (("pSpaces",0.25),("sSpaces",0.75),("pSpaces",1.0)):
            active=fixture.actives[name]
            fixture.rig.set_interactive_overrides([(str(active.GetPrim().GetPath()),
                str(active.GetName()),value)])
            pose,_=_Frames(fixture.rig,fixture.paths,2.0)
            _CheckProgramAnswered(fixture.rig,"interactive index")
            _Check(pose.valid,"pose publication valid; authored numeric assertions follow")
            _CheckPose(pose,"interactive index")
        fixture.rig.clear_interactive_overrides()
        pose,_=_Frames(fixture.rig,fixture.paths,2.0)
        _CheckPose(pose,"index release")


def TestSourceUnderOwnTargetIsSetAside():
    """Explicit source/carry cycles invalidate members, preserve a sibling,
    and recover when the authored cycle is removed."""
    for field in ("sources", "space"):
        stage=Usd.Stage.CreateInMemory("selfSpace.usda")
        builder=rigexec.Builder.create(stage,RIG,"Test")
        other=builder.add_control("Other",_Translate(5,0,0))
        p=builder.add_control("P",_Translate(0,100,0))
        c=builder.add_control("C",_Translate(5,0,0),p)
        switch=_Switch(stage,"pSpaces",p.path,[other.path,RIG])
        rig=rigexec.Rig(stage,RIG);rig.compile()
        before=rig.evaluate(0)
        _Check(before.valid and before.control_frame(p.path).valid,"initial acyclic frame")
        relation=switch.GetRelationship("rigExec:sources") if field=="sources" else \
            switch.CreateRelationship("rigExec:space")
        relation.SetTargets([c.path,RIG] if field=="sources" else [c.path])
        rig.compile()
        for frame in (1,1):
            pose=rig.evaluate(frame)
            _Check(pose.valid and rig.op_graph(),"cycle keeps unrelated graph operations")
            message="\n".join(pose.diagnostics)
            _Check("cycle" in message and p.path in message and c.path in message,
                   "named graph cycle: "+message)
            _Check(tuple(pose.control_frame(other.path).origin)==(5,0,0),"sibling survives cycle")
            for path in (p.path,c.path):
                try:
                    _Check(not pose.control_frame(path).valid,"cycle cannot retain a previous frame: "+path)
                except KeyError:
                    pass
        relation.SetTargets([other.path,RIG] if field=="sources" else [])
        rig.compile();recovered=rig.evaluate(2)
        _Check(recovered.valid and recovered.control_frame(p.path).valid,"cycle removal recovers")
        _Check(_Near(tuple(recovered.control_frame(p.path).origin),(0,100,0)),"recovered literal")


_BIPED_CONTROLS = "/Biped/Rig/Main/Shot/Aux/Controls/"
_NECK = _BIPED_CONTROLS + "M_Body/M_Torso/M_Chest/M_ChestTop/M_Neck"
_SKULL = _NECK + "/M_Head/M_HeadGimbal/skull_follow/M_Skull"
_UPFACE = _SKULL + "/M_UpFace"


class _Biped(object):
    """Biped_stack with the nesting of _Nested built into its head: the head
    is taken out of the neck's dependency (its spaces become hips and
    world), and the neck gains a space under the head. So the head resolves
    first and reads the neck -- four controls up -- before the neck's own
    switch, while the neck reads the switched head."""

    def __init__(self, examples, mode):
        self.stage = Usd.Stage.Open(
            os.path.join(examples, "biped", "Biped_stack.usda"))
        self.stage.SetEditTarget(self.stage.GetSessionLayer())
        spaces = "/Biped/Rig/Spaces/"
        head = self.stage.GetPrimAtPath(spaces + "head_spaces")
        neck = self.stage.GetPrimAtPath(spaces + "neck_spaces")
        _Check(head and neck, "the biped's head and neck switches exist")
        world = "/Biped/Rig/Main/Shot/Aux"
        head.GetRelationship("rigExec:sources").SetTargets(
            [_BIPED_CONTROLS + "M_Body", world])
        neck.GetRelationship("rigExec:sources").SetTargets([
            _BIPED_CONTROLS + "M_Body/M_Torso/M_Chest/M_ChestTop", world,
            _UPFACE])
        neck.GetAttribute("rigExec:spaceLabels").Set(
            ["local", "world", "face"])
        _Key(self.stage.GetPrimAtPath(_NECK).GetAttribute("avars:space"),
             (0.0, 2.0, 1.5, 0.0))
        _Key(self.stage.GetPrimAtPath(_SKULL).GetAttribute("avars:space"),
             (1.0, 0.0, 0.5, 1.0))
        _Key(_Avar(self.stage, _NECK, "rx"), (0.0, 20.0, -15.0, 10.0))
        _Key(_Avar(self.stage, _BIPED_CONTROLS + "M_Body", "tx"),
             (0.0, 5.0, -3.0, 2.0))
        rigs = [prim.GetPath() for prim in self.stage.Traverse()
                if prim.GetTypeName() == "RigExecRoot"]
        _Check(len(rigs) == 1, "one rig in the biped: %s" % rigs)
        self.rig = rigexec.Rig(self.stage, str(rigs[0]))
        self.rig.compile()
        self.rig.cpu_reference = True
        self.paths = dict((name, path) for name, path in (
            ("neck", _NECK), ("skull", _SKULL), ("upface", _UPFACE),
            ("armIk", _BIPED_CONTROLS + "L_ArmIK"),
            ("look", _BIPED_CONTROLS + "M_Look")))


def TestBipedNestedSwitchBakesAndAgrees(examples):
    _RunAllModes(lambda mode: _Biped(examples, mode),
                 lambda fixture: fixture.paths, what="biped")


def main():
    plugin_dir = sys.argv[1] if len(sys.argv) > 1 else None
    if plugin_dir:
        Plug.Registry().RegisterPlugins(plugin_dir)
    examples = sys.argv[2] if len(sys.argv) > 2 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "..", "examples")
    TestNestedSwitchBakesAndAgrees()
    TestSameRoundReadsThePreSwitchParent()
    TestRecomposedVersionFollowsItsAvars()
    TestDialFormBakesAndAgrees()
    TestDragOfTheIndexAgrees()
    TestSourceUnderOwnTargetIsSetAside()
    TestBipedNestedSwitchBakesAndAgrees(examples)
    print("OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
