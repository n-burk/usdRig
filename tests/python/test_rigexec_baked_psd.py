#!/usr/bin/env python
"""
The stacked biped -- the rig, its pose interpolators and its sparse blend
shapes composed together -- through the baked program, held to parity.

Two things the biped branch built on the dynamic path are steps of the
program here: a RigExecPoseInterpolator is a PoseInterpolator step that reads
its driver's final frame out of the slots and writes a range of PoseWeight
slots, and a blend sample naming a UsdSkelBlendShape carries a layout the
prologue resolves once per epoch. Neither has a fixture rig of its own at
this scale, so this holds the real one:

  * the stacked character must BAKE (a refusal would be the two-system
    configuration coming back through a side door: the correctives would run
    on the dynamic path while the rig ran on the program);
  * BakedWithParityCheck must report ZERO mismatches at rest and posed, which
    compares the published weights and the deformed points exactly; and
  * posing a driver must MOVE a weight, or the step ran and proved nothing.

Usage: test_rigexec_baked_psd.py [<generated schema resources dir>]
"""
import os
import sys

from test_rigexec_python import _setup_environment  # noqa: E402
_setup_environment()

from pxr import Plug, Usd  # noqa: E402

import rigexec  # noqa: E402

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.dirname(os.path.dirname(_HERE))
_STACK = os.path.join(_REPO, "examples", "biped", "Biped_psd_all.usda")
_RIG = "/Biped/Rig"
_POINTS = "/Biped/Geom/body_geo.points"

# A control whose rotation reaches an elbow driver, and the root.
_PROBES = ("arm_l_fk_elbow_l_bind", "hips_ctl")
_VALUES = (35.0, -40.0)


def _RegisterSchema():
    if len(sys.argv) > 1:
        Plug.Registry().RegisterPlugins(sys.argv[1])


def _Open(mode):
    stage = Usd.Stage.Open(_STACK)
    rig = rigexec.Rig(stage, _RIG)
    rig.compile()
    rig.evaluation_mode = mode
    controls = {p.GetName(): p.GetPath().pathString
                for p in stage.Traverse()
                if p.GetTypeName() == "RigExecControl"}
    weights = [p.GetPath().pathString + ".outputs:weight"
               for p in stage.Traverse()
               if p.GetTypeName() == "RigExecPose"]
    return stage, rig, controls, weights


def TestTheStackBakesWithParity():
    stage, rig, controls, weights = _Open("parity")
    assert weights, "the stacked biped is expected to carry poses"
    assert rig.is_bakeable(), (
        "the stacked biped does not bake; the correctives would run on the "
        "dynamic path while the rig runs on the program")
    rest = rig.evaluate(1.0)
    assert rest.valid
    assert rig.baked_cluster_count > 0, "parity ran with no program"
    assert not rest.baked_parity_mismatches, (
        "at rest: %d baked parity mismatch(es)" % rest.baked_parity_mismatches)
    published = [w for w in weights if w in rest.moved_properties()]
    assert len(published) == len(weights), (
        "%d of %d pose weights were not published" %
        (len(weights) - len(published), len(weights)))

    atRest = {w: rest.moved_property(w) for w in weights}
    moved = 0
    for name in _PROBES:
        for value in _VALUES:
            rig.set_interactive_overrides([(controls[name], "avars:rz", value)])
            pose = rig.evaluate(1.0)
            assert not pose.baked_parity_mismatches, (
                "%s rz=%g: %d baked parity mismatch(es)"
                % (name, value, pose.baked_parity_mismatches))
            moved += sum(1 for w in weights
                         if abs(pose.moved_property(w) - atRest[w]) > 1e-6)
    assert moved > 0, "posing the rig moved no pose weight at all"
    rig.clear_interactive_overrides()
    pose = rig.evaluate(1.0)
    assert not pose.baked_parity_mismatches, (
        "release: %d baked parity mismatch(es)" % pose.baked_parity_mismatches)


def main():
    _RegisterSchema()
    if not os.path.exists(_STACK):
        print("SKIP: %s not present" % _STACK)
        return 0
    TestTheStackBakesWithParity()
    print("  ok: the stacked biped bakes with parity")
    print("RIGEXEC_BAKED_PSD_OK (1 group)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
