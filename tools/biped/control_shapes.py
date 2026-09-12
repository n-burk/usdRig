#!/usr/bin/env python
"""Give the RigExec controls Maya's shapes, colours and sizes.

Nothing here is invented. Three sources, all authored by the Maya rig:

  * `build/control_positions.data` -- a `CurveData` file with 361 controls,
    each carrying the actual shape `cvPositions` and a Maya wireframe
    `color` index. The extent of those CVs is the control's real size in
    centimetres, and it is what makes a RigExec guide read at the same scale
    as the Maya control instead of at Hydra's unit fallback.
  * `core/shared/color.py` -- the studio's `NAME` / `NAME_RGB` constant
    pairs, parsed to turn those colour indices into RGB. Hand-rolling
    Maya's 32-entry palette would drift from whatever the studio actually
    uses; reading it cannot.
  * the control TYPE each control is built with in `rig_bits.nxt`, which is
    the only one of the three that has to be written down here, because it
    lives in graph attributes rather than in a data file.

Shape vocabulary maps almost one-to-one. RigExec allows
`sphere|circle|box|cube|diamond|pyramid`, and its schema settles the
ambiguity between the two box-ish tokens: "circle and box are the planar
shapes -- normal +Y, drawn in the local XZ plane -- while cube is the 3D
box". Maya's `square` is a four-CV planar curve with normal +Y at
(+-0.5, 0, +-0.5), so **Maya square -> RigExec box**, and `cube`, `diamond`,
`sphere`, `pyramid` and `circle` carry their own names across.

Maya's own library (`core/maya/rig/controls.data`) also holds `arrow`,
`arrowCircle`, `circle_cross`, `gear`, `lollipop`, `pad` and `triangle`,
which have no RigExec equivalent. None are used by the controls this rig
builds; if one ever is, it falls back to `circle` and says so.

**The frame the CVs are measured in matters as much as the CVs.** Maya
stores a control's CVs in the control's own space, and the spine and neck
controls in Maya are world-aligned (`world_rotate` 0). Their RigExec
counterparts sit on the spine/neck JOINT rest frames, which carry Maya's
joint orientation: for every one of them local X runs up the chain (world
+Y), local Y is world -Z and local Z is world -X (read straight out of
`rest:space` in the built stage). Measured in Maya's frame, spine_mid's
horizontal 21 cm ring is "flat on Y", became a `circle`, and RigExec drew
it in local XZ -- the world XY plane, a vertical ring facing the camera.
The hip and neck rings, measured the same way, put Maya's vertical extent
along the bone and came out as boxes taller than they are wide. So every
control's CVs are first rotated into that control's rest frame (Maya
`world_rotate`/`rotateOrder` out to world, then the `rest:space` rotation
into RigExec local) and only then measured and classified. The FK limb
rings, whose Maya frames already match the RigExec ones, are the check:
they come out flat on the bone axis either way.
"""
import json
import os
import re

from pxr import Gf, Sdf

DATA = ("C:/Users/walte/Documents/dev/squarebit/templates/templates/maya/"
        "biped/rig/default/data/build")
COLOR_PY = ("C:/Users/walte/Documents/dev/squarebit/core/core/shared/"
            "color.py")

# RigExec control name -> (Maya control name, RigExec guide:shape).
# Shapes come from the control_type each control is created with in
# rig_bits.nxt: pole vectors are diamonds, IK and FK controls cubes, the
# swing/clavicle/pelvis/chest_top controls squares (planar), spine_mid a
# circle. The token here is documentation of that type; the shape actually
# authored is derived from the CV geometry by `classify`, because the type
# name says nothing about which plane a planar control lies in. Sizes and
# colours are read from the data, never from here.
CONTROL_MAP = {}
for _side in ("l", "r"):
    CONTROL_MAP.update({
        "arm_%s_ik" % _side: ("arm_ik_%s" % _side, "cube"),
        "arm_%s_pv" % _side: ("arm_pv_%s" % _side, "diamond"),
        "arm_%s_root" % _side: ("shoulderSwing_%s" % _side, "box"),
        "arm_%s_fk_shoulder_%s_bind" % (_side, _side):
            ("shoulder_fk_%s" % _side, "cube"),
        "arm_%s_fk_elbow_%s_bind" % (_side, _side):
            ("elbow_fk_%s" % _side, "cube"),
        "arm_%s_fk_wrist_%s_bind" % (_side, _side):
            ("wrist_fk_%s" % _side, "cube"),
        "leg_%s_ik" % _side: ("leg_ik_%s" % _side, "cube"),
        "leg_%s_pv" % _side: ("leg_pv_%s" % _side, "diamond"),
        "leg_%s_root" % _side: ("thighSwing_%s" % _side, "box"),
        "leg_%s_fk_thigh_%s_bind" % (_side, _side):
            ("thigh_fk_%s" % _side, "cube"),
        "leg_%s_fk_knee_%s_bind" % (_side, _side):
            ("knee_fk_%s" % _side, "cube"),
        "leg_%s_fk_ankle_%s_bind" % (_side, _side):
            ("ankle_fk_%s" % _side, "cube"),
        "pelvis_%s_ctl" % _side: ("pelvis_%s" % _side, "box"),
        "clavicle_%s_ctl" % _side: ("clavicle_%s" % _side, "box"),
    })
CONTROL_MAP.update({
    # The body controls (body.py): Maya's `hips` (the whole-character
    # control, /spine/controls/fk/hips, control_type cube but a 16-CV
    # ring flat on world Y in the data) and `torso` (the FK swing between
    # hips and chest, /spine/controls/fk/torso, same kind of ring). Both
    # are world-aligned here as in Maya, so the CVs need no re-framing.
    "hips_ctl": ("hips", "circle"),
    "torso_ctl": ("torso", "circle"),
    "hip_swivel_ctl": ("hip_swivel", "cube"),
    "chest_top_ctl": ("chest_top", "box"),
    # The spine/neck controls and their offset-pivot parents
    # (build_biped_rigexec.add_pivot_control). The pivot control is the
    # one with Maya's pivotHeight -- hip_swivel, chest, head -- and sits
    # 35% along the chain; the nested control at the joint is Maya's
    # self-pivoting one: hips_gimbal, chest_top, head_gimbal.
    "spine_root_pivot": ("hip_swivel", "cube"),
    "spine_root_ctl": ("hips_gimbal", "cube"),
    "spine_mid_ctl": ("spine_mid", "circle"),
    "spine_end_pivot": ("chest", "cube"),
    "spine_end_ctl": ("chest_top", "box"),
    "neck_root_ctl": ("neck", "cube"),
    "neck_end_pivot": ("head", "cube"),
    "neck_end_ctl": ("head_gimbal", "cube"),
})

# Maya's transform.rotateOrder enum, in the order the rotations are applied
# to a row vector (point * Rx * Ry * Rz for "xyz").
MAYA_ROTATE_ORDERS = {0: "xyz", 1: "yzx", 2: "zxy", 3: "xzy", 4: "yxz",
                      5: "zyx"}
_AXIS = {"x": Gf.Vec3d(1, 0, 0), "y": Gf.Vec3d(0, 1, 0),
         "z": Gf.Vec3d(0, 0, 1)}
_AXIS_NAMES = "XYZ"


def load_palette(path=COLOR_PY):
    """Maya colour index -> RGB, parsed from the studio's colour module."""
    src = open(path).read()
    idx = dict(re.findall(r"^([A-Z][A-Z_]*) *= *(\d+) *$", src, re.M))
    rgb = dict(re.findall(r"^([A-Z][A-Z_]*)_RGB *= *(\[[^\]]*\]) *$",
                          src, re.M))
    out = {}
    for name, i in idx.items():
        if name in rgb:
            out[int(i)] = [float(v) for v in json.loads(rgb[name])]
    return out


def _half_range(points):
    """(half-range per axis, centre per axis) of a point list.

    HALF-RANGE, not max(abs(cv)). Maya shapes are frequently offset from
    their pivot -- the foot control wraps the foot while its pivot is at
    the ankle -- so max(abs()) measures offset + size and badly inflates it
    (leg_ik came out as a 65cm box). RigExec draws its guide at the
    control's posed origin, so the offset cannot be reproduced by scale
    anyway; the size can.
    """
    if not points:
        return [0.0] * 3, [0.0] * 3
    lo = [min(c[k] for c in points) for k in range(3)]
    hi = [max(c[k] for c in points) for k in range(3)]
    return ([(hi[k] - lo[k]) * 0.5 for k in range(3)],
            [(hi[k] + lo[k]) * 0.5 for k in range(3)])


def load_control_data(path=None):
    """Maya control name -> {color, extent, ...} from control_positions.data.

    `extent`/`centre` are measured in the Maya control's own space, as
    before. The raw `cvs`, the control's Maya `world_rotate`/`rotateOrder`
    and whether the shape is a single closed loop (`closedLoop`) are kept
    so the caller can re-measure in the RigExec control's rest frame.
    """
    path = path or os.path.join(DATA, "control_positions.data")
    blob = json.load(open(path))
    if blob.get("type") != "CurveData":
        raise SystemExit("%s is not CurveData" % path)
    out = {}
    for name, rec in blob["data"].items():
        cvs, forms = [], []
        for shape in (rec.get("shapes") or {}).values():
            cvs.extend(shape.get("cvPositions") or [])
            forms.append(shape.get("form"))
        extent, centre = _half_range(cvs)
        out[name] = {
            "color": rec.get("color"), "extent": extent, "centre": centre,
            "cvCount": len(cvs), "cvs": cvs,
            "world_rotate": rec.get("world_rotate") or [0.0, 0.0, 0.0],
            "rotateOrder": rec.get("rotateOrder", 0),
            # One periodic/closed curve is a ring however much it wobbles;
            # cubes and diamonds are open degree-1 curves tracing edges.
            "closedLoop": (len(forms) == 1 and
                           forms[0] in ("Periodic", "Closed")),
        }
    return out


def maya_world_rotation(rec):
    """Row-vector rotation taking the control's CVs into Maya world axes."""
    order = MAYA_ROTATE_ORDERS.get(rec.get("rotateOrder", 0), "xyz")
    euler = rec.get("world_rotate") or [0.0, 0.0, 0.0]
    m = Gf.Matrix3d(1)
    for ax in order:
        m = m * Gf.Matrix3d(Gf.Rotation(_AXIS[ax], euler["xyz".index(ax)]))
    return m


def rest_rotation(prim):
    """A RigExec xformable's world rest rotation; rows = local axes in world.

    `rest:space` is relative to the nearest RigExec namespace ancestor
    (identity when there is none, which is the case for every control under
    the Controls scope), composed as compose(rest:r) * rest:space * parent.
    """
    m = Gf.Matrix4d(1)
    p = prim
    while p and not p.IsPseudoRoot():
        attr = p.GetAttribute("rest:space")
        if attr and attr.HasAuthoredValue():
            local = Gf.Matrix4d(attr.Get())
            rot = Gf.Matrix4d(1)
            for ax in "xyz":
                r = p.GetAttribute("rest:r%s" % ax)
                deg = r.Get() if (r and r.HasAuthoredValue()) else 0.0
                if deg:
                    rot = rot * Gf.Matrix4d(Gf.Rotation(_AXIS[ax], deg),
                                            Gf.Vec3d(0))
            m = m * (rot * local)
        p = p.GetParent()
    return m.ExtractRotationMatrix().GetOrthonormalized()


def frame_extent(rec, rest_rot):
    """Half-range and centre of the Maya CVs in the RigExec rest frame.

    CV -> Maya world (the control's world_rotate) -> RigExec local (dot with
    the rest rotation's rows, which are the local axes expressed in world).
    """
    to_world = maya_world_rotation(rec)
    rows = [Gf.Vec3d(*rest_rot.GetRow(k)) for k in range(3)]
    pts = []
    for cv in rec["cvs"]:
        w = Gf.Vec3d(*cv) * to_world
        pts.append([Gf.Dot(w, rows[k]) for k in range(3)])
    return _half_range(pts)


def classify(rec, ext=None):
    """Pick a RigExec guide shape from the Maya curve's own geometry.

    Derived from the data rather than guessed per control, because guessing
    got it wrong: I had the FK limb controls as `cube` when Maya draws them
    as 8-CV circles lying in the YZ plane -- rings around the bone. The
    pole vector happened to be right only because a 14-CV 3D shape with
    equal extents really is a diamond.

    `ext` is the half-range per axis to classify against. It must be
    measured in the RigExec control's REST frame (`frame_extent`), not in
    Maya's control space: the spine/neck rest frames have the bone on local
    X where Maya's controls are world-aligned, and classifying in Maya's
    frame is exactly what drew spine_mid as a vertical ring and the hips as
    a box taller than wide. Without `ext` the Maya-space extent is used,
    which is only right when the two frames coincide (the limbs).

    What the CV data says, measured in the RigExec rest frame:
        arm_pv_l         14 CVs open   3D, equal          -> diamond
        elbow_fk_l        8 CVs loop   flat on X (bone)   -> ring about bone
        knee_fk_l         8 CVs loop   flat on X (bone)   -> ring about bone
        spine_mid         8 CVs loop   flat on X (bone)   -> ring about bone
        chest_top        16 CVs loop   flat on X (bone)   -> ring about bone
        hip_swivel        8 CVs loop   thinnest on X      -> ring about bone
        head             12 CVs loop   thinnest on Y      -> circle (normal Y)
        shoulderSwing_l   5 CVs open   flat on Y, square  -> box    (normal Y)

    A single periodic curve is a closed loop, and a closed loop is a RING
    whatever it does out of plane: hip_swivel is a saddle that dips 12 cm
    at front and back, neck a ring tilted to follow the jaw, thigh_fk a
    ring with a 4 cm wobble. Its plane is the plane of its two larger
    extents and its normal is the thinnest axis. Reading those as `cube`
    because they are not flat to within 0.5 cm was what wrapped the torso
    and neck in boxes. The wobble itself is not expressible and is dropped.

    **The honest limitation**: RigExec's planar guides (`circle`, `box`) are
    normal +Y, drawn in the local XZ plane -- the schema is explicit about
    it. So a ring around a bone whose axis is local X CANNOT be oriented
    correctly by shape token alone. Rotating the control's rest frame would
    fix the drawing and break the channel semantics (after the limb
    re-frame, `avars:rx` is twist because local X is the bone), which is a
    bad trade. Instead a `sphere` is flattened on the bone axis, which
    draws as a disc perpendicular to the bone -- the right plane and the
    right read, if not a true circle. Returned `flat_axis` tells the caller
    which axis to squash.

    Returns `(shape, flat_axis)` where `flat_axis` is 0/1/2 or None.
    """
    ext = list(ext if ext is not None else rec["extent"])
    cvs = rec.get("cvCount") or 0
    loop = bool(rec.get("closedLoop"))
    flat = [k for k in range(3) if ext[k] < 0.5]
    biggest = max(ext) or 1.0

    if loop and len(flat) < 2:
        normal = min(range(3), key=lambda k: ext[k])
    elif len(flat) == 1:
        normal = flat[0]
    else:
        normal = None

    if normal is not None:
        if normal == 1:
            # Planar in XZ, which is exactly RigExec's planar orientation.
            # A loop of 8+ CVs is a circle whatever its aspect (the scale
            # makes it an ellipse); an open curve has to be round to be one.
            other = [ext[k] for k in range(3) if k != 1]
            roundish = abs(other[0] - other[1]) < 0.15 * biggest
            circle = cvs >= 8 and (loop or roundish)
            return ("circle" if circle else "box"), 1
        # Planar in YZ or XY: RigExec cannot orient a planar guide here, so
        # flatten a sphere on the normal axis to get a disc in the right
        # plane.
        return "sphere", normal

    if not flat:
        equalish = (max(ext) - min(ext)) < 0.15 * biggest
        if equalish and 8 <= cvs <= 20:
            return "diamond", None
        return "cube", None

    # Degenerate in two axes -- a line. Nothing sensible; keep it small.
    return "box", None


def world_axis_name(vec):
    """'+Y' style name of the world axis a local axis lies closest to."""
    k = max(range(3), key=lambda i: abs(vec[i]))
    return ("+" if vec[k] >= 0 else "-") + _AXIS_NAMES[k]


def describe_plane(shape, flat_axis, rest_rot):
    """Where the drawn guide ends up in world, given the rest orientation."""
    if shape in ("circle", "box"):
        flat_axis = 1
    if flat_axis is None:
        return "3D"
    normal = Gf.Vec3d(*rest_rot.GetRow(flat_axis))
    in_plane = [world_axis_name(Gf.Vec3d(*rest_rot.GetRow(k)))
                for k in range(3) if k != flat_axis]
    return "normal local %s = world %s; plane world %s/%s" % (
        _AXIS_NAMES[flat_axis], world_axis_name(normal), in_plane[0],
        in_plane[1])


def apply_control_shapes(stage, rig_root="/Biped/Rig", verbose=True):
    """Author guide:shape, guide:displayColor and guide:scale* on controls.

    RigExec guides are unit-sized (radius 1, box spanning +-1), so the
    scale is the Maya shape's half-extent per axis in centimetres, measured
    in the control's rest frame. A planar shape reports a zero extent on
    its normal axis, which would collapse the guide, so that axis is given
    a thin slice instead.
    """
    palette = load_palette()
    maya = load_control_data()

    controls = [p for p in stage.Traverse()
                if p.GetTypeName() == "RigExecControl"]
    done, skipped = 0, []
    for prim in controls:
        name = prim.GetName()
        entry = CONTROL_MAP.get(name)
        if entry is None:
            skipped.append(name)
            continue
        maya_name, _hint = entry
        rec = maya.get(maya_name)
        if rec is None:
            skipped.append("%s (no Maya control %s)" % (name, maya_name))
            continue

        rest_rot = rest_rotation(prim)
        ext, _centre = frame_extent(rec, rest_rot)
        shape, flat_axis = classify(rec, ext)
        prim.CreateAttribute("guide:shape", Sdf.ValueTypeNames.Token,
                             True, Sdf.VariabilityUniform).Set(shape)

        rgb = palette.get(rec["color"])
        if rgb:
            prim.CreateAttribute("guide:displayColor",
                                 Sdf.ValueTypeNames.Color3f).Set(
                                     Gf.Vec3f(*rgb))

        # Scale is the Maya shape's half-range per axis. On the flat axis of
        # a planar control, use a thin slice of the control's size rather
        # than a fixed 1 cm: a 1 cm slab reads as a box on a 3 cm control
        # and as a sheet on a 20 cm one.
        biggest = max(ext) or 1.0
        for axis, value in enumerate(ext):
            if value <= 1e-6 or axis == flat_axis:
                ext[axis] = max(0.04 * biggest, 0.05)
        for axis, value in zip("XYZ", ext):
            prim.CreateAttribute("guide:scale%s" % axis,
                                 Sdf.ValueTypeNames.Double).Set(float(value))
        done += 1
        if verbose:
            print("    %-26s %-8s %-16s scale=(%.2f %.2f %.2f) color=%-3s %s"
                  % (name, shape, maya_name, ext[0], ext[1], ext[2],
                     rec["color"], describe_plane(shape, flat_axis,
                                                  rest_rot)))

    print("  shaped %d of %d controls from Maya data" % (done,
                                                         len(controls)))
    if skipped:
        print("  unmapped (left at RigExec defaults): %s"
              % ", ".join(sorted(skipped)))
    return done


if __name__ == "__main__":
    import sys

    pal = load_palette()
    ctl = load_control_data()
    print("palette entries: %d" % len(pal))
    print("maya controls   : %d" % len(ctl))
    print("mapped          : %d" % len(CONTROL_MAP))
    missing = sorted({m for m, _ in CONTROL_MAP.values()} - set(ctl))
    print("mapped names absent from the data: %s" % (missing or "none"))
    for i in (6, 13, 14, 7, 20, 22):
        print("  color %-3d -> %s" % (i, pal.get(i)))

    # `python control_shapes.py some_rig.usda` applies the shapes to an
    # in-memory copy of that stage and prints what each control would get,
    # including the world plane its guide is drawn in. Nothing is saved.
    if len(sys.argv) > 1:
        from pxr import Usd
        stage = Usd.Stage.Open(sys.argv[1])
        print("\napplied to a copy of %s (not saved):" % sys.argv[1])
        apply_control_shapes(stage, verbose=True)
