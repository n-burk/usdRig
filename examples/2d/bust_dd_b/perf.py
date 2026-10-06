"""Shion's animation layers: the curves, the secondary-motion solve and the
USD writer. Only control avars are authored; RigExec evaluates everything
else live.

Curves are cubic Hermite splines through keys, sampled every frame. A key
is (frame, value) with an automatic Catmull-Rom tangent, or
(frame, value, tangent) where tangent is a slope (units per frame), "flat",
or (in_slope, out_slope) for a broken tangent (a snap into a hold). Snaps
come from SPACING -- keys close together into long, drifting holds -- not
from stepping.
"""

import math
import os

import numpy as np

from pxr import Sdf, Usd

FPS = 30


# curves

def spline(keys, frames):
    keys = sorted(keys, key=lambda k: k[0])
    t = np.array([k[0] for k in keys], float)
    v = np.array([k[1] for k in keys], float)
    n = len(keys)
    m_in = np.zeros(n)
    m_out = np.zeros(n)
    for i, k in enumerate(keys):
        if len(k) > 2 and k[2] is not None:
            tan = k[2]
            if tan == "flat":
                m_in[i] = m_out[i] = 0.0
            elif isinstance(tan, tuple):
                m_in[i], m_out[i] = tan
            else:
                m_in[i] = m_out[i] = float(tan)
            continue
        if n == 1:
            continue
        if i == 0:
            s = (v[1] - v[0]) / (t[1] - t[0])
        elif i == n - 1:
            s = (v[-1] - v[-2]) / (t[-1] - t[-2])
        else:
            s = (v[i + 1] - v[i - 1]) / (t[i + 1] - t[i - 1])
            # a key that is a local extreme holds (no overshoot past it)
            if (v[i] - v[i - 1]) * (v[i + 1] - v[i]) <= 0:
                s = 0.0
        m_in[i] = m_out[i] = s
    f = np.asarray(frames, float)
    out = np.empty_like(f)
    for j, x in enumerate(f):
        if x <= t[0]:
            out[j] = v[0]
            continue
        if x >= t[-1]:
            out[j] = v[-1]
            continue
        i = int(np.searchsorted(t, x, side="right") - 1)
        h = t[i + 1] - t[i]
        s = (x - t[i]) / h
        h00 = 2 * s ** 3 - 3 * s ** 2 + 1
        h10 = s ** 3 - 2 * s ** 2 + s
        h01 = -2 * s ** 3 + 3 * s ** 2
        h11 = s ** 3 - s ** 2
        out[j] = h00 * v[i] + h10 * h * m_out[i] + h01 * v[i + 1] + h11 * h * m_in[i + 1]
    return out


def drift(frames, amp, periods=(71.0, 113.0, 157.0), phase=0.0):
    """A slow living drift for moving holds: three incommensurate sines."""
    f = np.asarray(frames, float)
    w = [0.55, 0.30, 0.15]
    return amp * sum(wi * np.sin(2 * math.pi * f / p + phase * (k + 1.3)) for k, (wi, p) in enumerate(zip(w, periods)))


def blink(starts, frames, close=1.0, lead=0):
    """Blink curve (0 open .. 1 closed): 2 f to close, 1 f closed, 4 f to
    open with a slow tail. Returns the closure amount."""
    f = np.asarray(frames, float)
    out = np.zeros_like(f)
    for s0 in starts:
        k = [(s0 - 0.001, 0.0, "flat"), (s0 + 1.0, 0.55 * close), (s0 + 2.0, close, "flat"),
             (s0 + 3.0, close, "flat"), (s0 + 4.2, 0.45 * close), (s0 + 5.5, 0.12 * close), (s0 + 7.5, 0.0, "flat")]
        seg = (f >= s0 - 1) & (f <= s0 + 8)
        out[seg] = np.maximum(out[seg], spline(k, f[seg]))
    return out


# secondary motion: damped springs chasing the head

# (frequency Hz, damping ratio, stiffness per joint (root -> tip), turn kick,
#  tilt follow)
SPRINGS = {
    "fringe": (2.2, 0.38, [0.95, 0.80, 0.65, 0.55], 1.6),
    "lock": (1.3, 0.40, [0.85, 0.65, 0.48, 0.36, 0.28, 0.22], 2.2),
    "side": (1.6, 0.30, [0.85, 0.65, 0.5], 1.8),
    "back": (0.9, 0.42, [0.7, 0.5, 0.36, 0.26], 2.6),
    "ahoge": (3.0, 0.16, [1.0, 1.0, 1.0], -3.0),
    "earring": (1.7, 0.10, [0.05, 0.02], 4.5),
    "ribbon": (1.5, 0.28, [0.5, 0.35, 0.25], 2.0),
}


def spring_kind(name):
    if name.startswith("F") or name == "Stray":
        return "fringe"
    if name.startswith("Lock"):
        return "lock"
    if name in ("LooseL", "TuckL"):
        return "side"
    if name.startswith("Back"):
        return "back"
    if name == "Ahoge":
        return "ahoge"
    if name == "Earring":
        return "earring"
    return "ribbon"


def secondary(A, chains, loops=3):
    """Per chain, per joint LOCAL rz (degrees). The joint's world angle
    chases a target: the head (or body) rotation times a per-joint
    stiffness, plus an inertial kick from the head's sideways and angular
    acceleration, plus a small breeze with its own period per lock."""
    n = len(A["Head.rz"])
    head_rot = A["Body.rz"] + A["Neck.rz"] + A["Head.rz"]
    body_rot = A["Body.rz"]
    # sideways motion of the head's front (turn + sway + tilt), in units
    lat = A["FaceAngle.ry"] * 0.11 + A["Body.tx"] * 1.0 + head_rot * 0.14
    lat_b = A["Body.tx"] + A["Body.rz"] * 0.3 + A["Torso.ry"] * 0.06

    def rep(a):
        return np.concatenate([a] * loops)

    H3, B3 = rep(head_rot), rep(body_rot)
    acc = np.gradient(np.gradient(rep(lat)))
    acc_b = np.gradient(np.gradient(rep(lat_b)))
    rot_acc = np.gradient(np.gradient(H3))
    breath3 = rep(A["Torso.ty"])
    acc_v = np.gradient(np.gradient(rep(A["FaceAngle.rx"])))
    out = {}
    for ci, (name, ch) in enumerate(sorted(chains.items())):
        kind = spring_kind(name)
        hz, zeta, stiff, kick = SPRINGS[kind]
        nj = len(ch["controls"])
        stiff = np.array((stiff + [stiff[-1]] * nj)[:nj])
        w = 2 * math.pi * hz / FPS
        beta = np.zeros((n * loops, nj))
        vel = np.zeros(nj)
        b = np.zeros(nj)
        ph = ci * 1.37
        period = 70.0 + 11.0 * (ci % 6)
        prof = np.linspace(0.6, 1.4, nj)
        rx0 = ch["pts"][0][0]
        side = 0.0 if abs(rx0) < 1.0 else float(np.sign(rx0))
        base = B3 if kind == "ribbon" else H3
        a_lat = acc_b if kind == "ribbon" else acc
        for f in range(n * loops):
            breeze = (1.2 / nj) * math.sin(2 * math.pi * f / period + ph) * np.linspace(0.3, 1.0, nj)
            if kind == "ribbon":
                breeze = breeze * 0.3 + 1.2 * (breath3[f] - 0.5) * np.linspace(0.3, 1.0, nj)
            target = base[f] * stiff - kick * 57.3 * a_lat[f] * prof / 6.0 + breeze
            if kind in ("ahoge", "earring"):
                target = target - 0.6 * kick * rot_acc[f] * prof
            if kind in ("fringe", "lock", "side", "back"):
                target = target + side * 1.4 * acc_v[f] * prof
            acc_ang = w * w * (target - b) - 2 * zeta * w * vel
            vel = vel + acc_ang
            b = b + vel
            beta[f] = b
        last = beta[(loops - 1) * n:loops * n]
        # close the loop: fade the tiny residual across the take
        resid = beta[loops * n - 1] - beta[(loops - 1) * n - 1]
        last = last - resid * np.linspace(0, 1, n)[:, None]
        local = np.zeros_like(last)
        local[:, 0] = last[:, 0] - (B3[:n] if kind == "ribbon" else H3[:n])
        for j in range(1, nj):
            local[:, j] = last[:, j] - last[:, j - 1]
        out[name] = local
    return out


# the rig-reveal sweep (layered mesh parameter demo)

SWEEP = 150


def sweep():
    F = np.arange(1, SWEEP + 1, dtype=float)
    A = {}

    def S(keys):
        return spline(keys, F)
    A["FaceAngle.ry"] = S([(1, 0), (8, 0, "flat"), (24, 31), (27, 30, "flat"), (34, 30, "flat"), (52, -31),
                           (55, -30, "flat"), (62, -30, "flat"), (76, 0.5), (79, 0, "flat"), (150, 0)])
    A["FaceAngle.rx"] = S([(1, 0), (76, 0, "flat"), (86, -16), (92, -15, "flat"), (104, 15.5),
                           (110, 15, "flat"), (122, 0, "flat"), (150, 0)])
    A["Head.rz"] = S([(1, 0), (8, 0, "flat"), (26, -7), (34, -6.5, "flat"), (54, 7), (62, 6.5, "flat"),
                      (78, 0, "flat"), (150, 0)])
    A["Neck.rz"] = np.roll(A["Head.rz"], 3) * 0.35
    A["Body.rz"] = S([(1, 0), (30, -1.2), (58, 1.2), (84, 0), (150, 0)])
    A["Body.tx"] = np.zeros_like(F)
    A["Torso.ty"] = 0.5 - 0.5 * np.cos(2 * np.pi * (F - 1) / 75.0)
    A["Torso.ry"] = 0.30 * np.roll(A["FaceAngle.ry"], 5)
    A["Look.tx"] = S([(1, 0), (6, 0, "flat"), (9, 0.9), (30, 0.8), (34, 0.8), (37, -0.9), (58, -0.8),
                      (66, -0.8), (70, 0), (150, 0)])
    A["Look.ty"] = S([(1, 0), (76, 0, "flat"), (80, 0.8), (92, 0.7), (96, -0.8), (110, -0.7), (116, 0), (150, 0)])
    A["Look.tz"] = S([(1, 0), (124, 0, "flat"), (127, 0.7), (136, 0.7, "flat"), (140, 0), (150, 0)])
    bl = blink([14, 66, 112], F)
    A["Eye_R.ty"] = -bl + S([(1, 0), (124, 0, "flat"), (127, 1.0), (136, 1.0, "flat"), (140, 0), (150, 0)])
    A["Eye_L.ty"] = A["Eye_R.ty"].copy()
    sm = S([(1, 0), (140, 0, "flat"), (143, 1.0), (150, 1.0)])
    A["Eye_R.tx"] = np.zeros_like(F)
    A["Eye_L.tx"] = np.zeros_like(F)
    A["Eye_R.tz"] = np.zeros_like(F)
    A["Eye_L.tz"] = np.zeros_like(F)
    A["Brow_R.ty"] = S([(1, 0), (20, 0.9), (34, 0.8), (44, 0), (90, 0), (98, -0.8), (108, -0.7), (116, 0), (124, 0, "flat"),
                        (127, 1.0), (138, 1.0), (144, 0), (150, 0)])
    A["Brow_L.ty"] = A["Brow_R.ty"].copy()
    A["Brow_R.rz"] = S([(1, 0), (44, 0, "flat"), (52, -20), (60, -19), (68, 0), (78, 12), (88, 12), (96, 0), (150, 0)])
    A["Brow_L.rz"] = A["Brow_R.rz"].copy()
    A["Mouth.ty"] = S([(1, 0), (40, 0, "flat"), (44, -1.0), (50, -1.0), (54, 0), (84, 0, "flat"), (88, -0.55),
                       (94, -0.55), (98, 0), (124, 0, "flat"), (127, -1.4), (136, -1.35), (140, -0.4), (146, -0.7), (150, -0.7)])
    A["Mouth.tx"] = S([(1, 0), (84, 0, "flat"), (88, 1.0), (94, 1.0), (98, 0), (120, 0, "flat"), (127, 0.4), (150, 0.2)])
    A["Mouth.rz"] = S([(1, 0), (100, 0, "flat"), (104, 16), (112, 16), (118, 0), (124, 0, "flat"), (127, -12), (136, -12),
                       (142, 18), (150, 18)])
    A["Mouth.ry"] = S([(1, 0), (100, 0, "flat"), (150, 0)])
    A["Fx.tx"] = S([(1, 0), (124, 0, "flat"), (128, 1.0), (150, 1.0)])
    A["Fx.ty"] = S([(1, 0), (125, 0, "flat"), (129, 0.35), (150, 1.0)])
    A["Fx.tz"] = S([(1, 0), (125, 0, "flat"), (130, 1.0), (150, 1.0)])
    A["Eye_R.tx"] = sm * 0.0
    return F, A


# writer

def write_anim(builder, path, rig_layer_name, script, length, doc=""):
    if os.path.exists(path):
        os.remove(path)
    layer = Sdf.Layer.CreateNew(path)
    layer.subLayerPaths = ["./" + rig_layer_name]
    layer.startTimeCode = 1
    layer.endTimeCode = length
    layer.timeCodesPerSecond = FPS
    layer.framesPerSecond = FPS
    layer.defaultPrim = "Shion"
    layer.documentation = doc
    stage = Usd.Stage.Open(layer)
    F, A = script()
    ctl = builder.ctl

    def key(prim_path, channel, values):
        prim = stage.OverridePrim(prim_path)
        a = prim.CreateAttribute("avars:" + channel, Sdf.ValueTypeNames.Double, False)
        for f, v in zip(F, values):
            a.Set(float(round(float(v), 4)), Usd.TimeCode(float(f)))

    for k, values in sorted(A.items()):
        c, ch = k.split(".")
        key(ctl[c], ch, values)
    hair = secondary(A, builder.chains)
    for name, local in hair.items():
        for j, cpath in enumerate(builder.chains[name]["controls"]):
            key(cpath, "rz", local[:, j])
    layer.Save()
    return F, A, hair


def performance():
    """The current restrained performance, shared with the pose library."""
    import acting
    return acting.performance()


def write_all(builder, here, rig_name):
    import acting
    write_anim(builder, os.path.join(here, "bust_dd_b_sweep.usda"), rig_name, acting.sweep, 320, doc=(
        "A 320-frame control sweep: subtle speech, expressions, turns, arms and individual fingers. "
        "Only control avars are authored. Generated by build_bust.py."))
    write_anim(builder, os.path.join(here, "bust_dd_b_anim.usda"), rig_name, acting.performance, 240, doc=(
        "A 240-frame performance for bust_dd_b_rig.usda. Only control avars are authored; hair, "
        "earring and ribbon avars come from an offline damped-spring solve. Generated by build_bust.py."))
