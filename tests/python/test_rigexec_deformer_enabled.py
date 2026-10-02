"""inputs:enabled on a mover actually stops it deforming.

The Deformer Toggles panel is only useful if the switch it writes is one
the evaluator obeys. The panel's own test asserts the attribute lands in
the session layer; this one asserts the attribute MEANS something, which
is the half that a UI test cannot see.

Usage:
    python test_rigexec_deformer_enabled.py [schema_resources_dir]
"""
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from test_rigexec_python import _setup_environment  # noqa: E402

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(os.path.dirname(_HERE))
_STACK = os.path.join(_REPO, "examples", "biped", "Biped_stack.usda")
_RIG = "/Biped/Rig"


def main():
    _setup_environment()
    from pxr import Sdf, Usd
    import rigexec
    rigexec.load_schema_plugin(sys.argv[1] if len(sys.argv) > 1 else None)

    stage = Usd.Stage.Open(_STACK)
    stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
    rig = rigexec.Rig(stage, _RIG)
    rig.evaluation_mode = "baked"

    # The mesh with the most deformers, so the test exercises a real one.
    groups = {}
    for prim in stage.Traverse():
        rel = prim.GetRelationship("rigExec:moves")
        if not rel:
            continue
        for target in rel.GetTargets():
            text = str(target)
            if text.endswith(".points"):
                groups.setdefault(text, []).append(prim)
                break
    assert groups, "no deformers found"
    mesh = max(groups, key=lambda m: len(groups[m]))
    movers = groups[mesh]

    def points():
        pose = rig.evaluate(0)
        assert pose.valid, [d for d in pose.diagnostics if "error" in d]
        return [tuple(v) for v in pose.moved_property(mesh)]

    def enable(prim, on):
        attr = prim.GetAttribute("inputs:enabled")
        if not attr or not attr.IsValid():
            attr = prim.CreateAttribute("inputs:enabled",
                                        Sdf.ValueTypeNames.Bool)
        attr.Set(on)

    # Pose something, or every deformer contributes zero and disabling one
    # is indistinguishable from leaving it on -- the same rest-only trap
    # that made a broken foot look fine.
    head = stage.GetPrimAtPath(
        _RIG + "/Main/Shot/Aux/Controls/M_Body/M_Torso/M_Chest/M_ChestTop"
               "/M_Neck/M_Head")
    for name, value in (("rx", 8.0), ("ry", 30.0), ("rz", 22.0)):
        attr = head.GetAttribute("avars:" + name)
        if not attr or not attr.IsValid():
            attr = head.CreateAttribute("avars:" + name,
                                        Sdf.ValueTypeNames.Double)
        attr.Set(value)

    before = points()
    for prim in movers:
        enable(prim, False)
    after = points()
    worst = max(max(abs(a - b) for a, b in zip(p, q))
                for p, q in zip(before, after))
    assert worst > 1e-6, (
        "disabling all %d deformers on %s changed nothing (%.3e)"
        % (len(movers), mesh, worst))
    print("  disabling %d deformers moved %s by %.4f"
          % (len(movers), mesh.rsplit("/", 1)[-1], worst))

    for prim in movers:
        enable(prim, True)
    restored = points()
    back = max(max(abs(a - b) for a, b in zip(p, q))
               for p, q in zip(before, restored))
    assert back == 0.0, ("re-enabling did not restore exactly (%.3e)" % back)
    print("  re-enabling restored the mesh exactly")
    print("OK: inputs:enabled switches a deformer off and back on")


if __name__ == "__main__":
    main()
