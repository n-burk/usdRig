#!/usr/bin/env python
"""The limb frames RigExecTwoBoneIk expects, shared by every biped builder.

## The problem

`RigExecTwoBoneIk` never reads a joint's rest *orientation*. It publishes a
basis it derives from the geometry (`libs/rigExecMath/solvers.cpp`,
`RigExecSolveTwoBoneIk` -> `_FrameFromAxes`):

    X = unit(bone)                      root: mid-root,  mid: end-mid
    Y = unit(bendUp - X (X.bendUp))     bendUp = unit((pole-root) perp chord)
    Z = X cross Y                       the bend-plane normal

The rest is used only for handle lengths, and -- in the degenerate-pole case
only -- for the root rest's Y. So the *only* rest orientation the solver can
hold is exactly that basis.

This biped's LEFT limb frames already are that basis, to 1e-9. The RIGHT
limb differs by exactly 180 degrees about local **Z** (the bend-plane
normal, not the bone axis), because the rig mirrors its right side by
negating bone translation: `ankle_l_bind` local translate is
`(+42.840, 0, 0)` and `ankle_r_bind` is `(-42.840, 0, 0)`. Bind the solver
straight to those joints and the right root/mid come out rolled 180, which
throws a child at 42.840 along X twice that distance off.

## The fix, and why it is safe

Replace the world rest orientation of the four right-side root/mid joints
with the solver basis, then recompute every joint's parent-local rest from
the modified world frames so that every other joint's world frame -- and
every origin -- is unchanged. Four orientations and twelve `rest:space`
values move; nothing else does.

Skinning is unaffected, and that is provable rather than hopeful. With
row-vector composition (`W_child = L . W_parent`, `W_posed = W_rest . D`)
the skin transform is `T = inverse(W_bind) . W_posed`. A constant local
re-basis `C` gives `W' = C . W`, and the same physical pose gives
`W'_posed = C . W_posed`, so

    T' = inverse(C . W) . (C . W_posed)
       = inverse(W) . inverse(C) . C . W_posed
       = T

**The one hard condition**: the UsdSkel `restTransforms`/`bindTransforms`
must come from the SAME re-framed set as the RigExec `rest:space` values. If
the Skeleton keeps Maya's frames while the RigExec joints use these, then
`bake_rigexec_to_skel.py` bakes a 180-degree roll into the skin. That is
precisely why this module exists instead of the logic living in one builder.

A side benefit: because the root rest Y now *is* the bend direction, the
degenerate-pole fallback holds the rest pose with
`rigExec:preferredBendRadians = 0`.
"""
import os
import sys

from pxr import Gf

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from build_biped_skel import DATA, load_joints, local_matrix, skeleton_order

# The four chains the Maya build drives with an ikRPsolver
# (rig_bits.nxt:/limb, `joint_list` on /arm/limb and /leg/limb).
LIMBS = [
    ("arm_l", "shoulder_l_bind", "elbow_l_bind", "wrist_l_bind"),
    ("arm_r", "shoulder_r_bind", "elbow_r_bind", "wrist_r_bind"),
    ("leg_l", "thigh_l_bind", "knee_l_bind", "ankle_l_bind"),
    ("leg_r", "thigh_r_bind", "knee_r_bind", "ankle_r_bind"),
]

# rig_bits.nxt uses pv_offset / pv_magnitude = 40 for both limbs.
POLE_OFFSET = 40.0


def perp(v, axis):
    """The component of v perpendicular to a unit axis."""
    return v - axis * Gf.Dot(axis, v)


def from_rows(ex, ey, ez, origin):
    """Row-vector matrix from basis rows plus an origin."""
    return Gf.Matrix4d(ex[0], ex[1], ex[2], 0,
                       ey[0], ey[1], ey[2], 0,
                       ez[0], ez[1], ez[2], 0,
                       origin[0], origin[1], origin[2], 1)


def pole_position(start, mid, end, offset=POLE_OFFSET):
    """Where Maya's `get_pole_vector_position` puts the pole control.

    Projects the mid joint onto the start->end line at the ratio of the
    upper bone's length, then pushes out from the mid joint along that
    direction by `pv_offset` (ikfk.py:606). This is the branch the build
    actually takes: the limbs deviate ~2.3 degrees, above `is_colinear`'s
    0.008 rad threshold.
    """
    upper = (mid - start).GetLength()
    lower = (end - mid).GetLength()
    ratio = upper / (upper + lower) if (upper + lower) else 0.5
    proj = start + (end - start) * ratio
    out = mid - proj
    if out.GetLength() < 1e-6:
        out = Gf.Vec3d(0, 0, 1)
    return mid + out.GetNormalized() * offset


def solver_basis(rw, mw, ew, pole):
    """The world frames the solver publishes for root and mid at rest.

    Mirrors solvers.cpp: bendUp is orthogonalised against the CHORD first,
    then re-orthogonalised against each bone by _FrameFromAxes.
    """
    r = rw.ExtractTranslation()
    m = mw.ExtractTranslation()
    e = ew.ExtractTranslation()
    aim = (e - r).GetNormalized()
    bend_up = perp(pole - r, aim).GetNormalized()
    frames = []
    for origin, ex in ((r, (m - r).GetNormalized()),
                       (m, (e - m).GetNormalized())):
        ey = perp(bend_up, ex).GetNormalized()
        ez = Gf.Cross(ex, ey).GetNormalized()
        frames.append(from_rows(ex, ey, ez, origin))
    return frames, aim, bend_up


def world_frames(data, parent, ordered):
    """Maya's rest world frames, FK-composed from the parent-local rests."""
    worlds = {}
    for name, _path in ordered:
        lm = local_matrix(data[name])
        p = parent.get(name)
        worlds[name] = lm if p not in worlds else lm * worlds[p]
    return worlds


def reframe_limbs(data, parent, ordered, verbose=True):
    """Maya's rests with the limb root/mid frames replaced by the solver's.

    Returns `(worlds, locals, poles)`:
      worlds  name -> re-framed world rest (origins untouched)
      locals  name -> parent-local rest, recomputed so every world frame
              other than the re-framed ones is preserved exactly
      poles   limb tag -> pole control world position

    Pass `locals` to `add_joint`'s rest_space and to UsdSkel
    `restTransforms`; pass `worlds` to `bindTransforms` and to the IK
    controls' rest.
    """
    worlds = world_frames(data, parent, ordered)
    original = dict(worlds)
    poles = {}

    for tag, rn, mn, en in LIMBS:
        if not all(n in worlds for n in (rn, mn, en)):
            continue
        pole = pole_position(worlds[rn].ExtractTranslation(),
                             worlds[mn].ExtractTranslation(),
                             worlds[en].ExtractTranslation())
        poles[tag] = pole
        (s_root, s_mid), _aim, _up = solver_basis(
            worlds[rn], worlds[mn], worlds[en], pole)
        worlds[rn] = s_root
        worlds[mn] = s_mid

    locals_ = {}
    for name, _path in ordered:
        p = parent.get(name)
        locals_[name] = (worlds[name] if p not in worlds
                         else worlds[name] * worlds[p].GetInverse())

    if verbose:
        reframed = [n for n in worlds
                    if not Gf.IsClose(worlds[n], original[n], 1e-9)]
        rewritten = [n for n, _ in ordered
                     if not Gf.IsClose(locals_[n],
                                       local_matrix(data[n]), 1e-9)]
        print("  re-framed %d world orientations; %d parent-local rests "
              "rewritten" % (len(reframed), len(rewritten)))
        print("    re-framed: %s" % ", ".join(sorted(reframed)))
        print("    rewritten: %s" % ", ".join(sorted(rewritten)))

    return worlds, locals_, poles


def load_reframed(verbose=True):
    """Convenience: parse the joint data and return everything re-framed."""
    data, parent = load_joints(os.path.join(DATA, "joint_positions.data"))
    ordered = skeleton_order(data, parent)
    worlds, locals_, poles = reframe_limbs(data, parent, ordered, verbose)
    return data, parent, ordered, worlds, locals_, poles


if __name__ == "__main__":
    d, p, o, w, l, poles = load_reframed()
    print("\njoints: %d" % len(o))
    worst = 0.0
    worst_at = None
    for n, _ in o:
        want = d[n].get("world_translate")
        if not want:
            continue
        got = w[n].ExtractTranslation()
        e = max(abs(got[i] - want[i]) for i in range(3))
        if e > worst:
            worst, worst_at = e, n
    print("origins vs Maya world_translate: worst %.4g cm (%s)"
          % (worst, worst_at))
    print("%s" % ("PASS -- re-framing moved no origin" if worst < 1e-3
                  else "FAIL"))
