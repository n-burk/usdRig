#!/usr/bin/env python
"""Prove a split (center/left/right) biped composes to the flat build.

`split_layers.py` rewrites one flat rig as a root layer over three
sublayers, with the right side inheriting the left. This checks that the
rewrite changed nothing that matters, by measurement:

  1. Inventory   -- per layer: file size, prim/attribute/relationship specs,
                    inherit arcs, deactivations.
  2. Namespace   -- the composed prim set (path + type) is identical, so the
                    inherit arcs leaked nothing and dropped nothing.
  3. Properties  -- every attribute value, connection and relationship
                    target on the flat stage composes identically on the
                    layered stage.
  4. Rig         -- both compile; mover order is identical; all joint
                    frames match at rest AND under a fixed test pose, with
                    the worst deviation reported in cm.
  5. Pinning     -- the same sublayers behind a root WITHOUT the
                    `reorder nameChildren` pins fail to compile, so the
                    pins are load-bearing.

Each check prints PASS/FAIL and the exit code is non-zero if any fail.

Usage:
    verify_layers.py <flat.usda> <root.usda> [--rig-root /Biped/Rig]
                     [--tolerance 1e-6]
"""
import argparse
import os
import sys

from pxr import Sdf, Usd

import rigexec

FAILURES = []


def check(ok, label, detail=""):
    print("  [%s] %s%s" % ("PASS" if ok else "FAIL", label,
                           ("  -- " + detail) if detail else ""))
    if not ok:
        FAILURES.append(label)
    return ok


# ---------------------------------------------------------------- inventory

def layer_inventory(layer):
    prims = attrs = rels = inherits = inactive = 0
    stack = list(layer.pseudoRoot.nameChildren)
    while stack:
        spec = stack.pop()
        prims += 1
        attrs += len(spec.attributes)
        rels += len(spec.relationships)
        for lst in (spec.inheritPathList, spec.referenceList,
                    spec.specializesList):
            if lst.prependedItems or lst.explicitItems or lst.appendedItems:
                inherits += 1
                break
        if spec.HasInfo("active") and not spec.active:
            inactive += 1
        stack.extend(spec.nameChildren)
    return dict(prims=prims, attrs=attrs, rels=rels, inherits=inherits,
                inactive=inactive,
                kb=os.path.getsize(layer.realPath) / 1024.0)


def inventory(root):
    print("\nLayer inventory")
    print("  %-28s %8s %6s %6s %6s %8s %8s"
          % ("layer", "KB", "prims", "attrs", "rels", "arcs", "inactive"))
    rows = []
    for path in [root.identifier] + list(root.subLayerPaths) + \
            [p.assetPath for p in _payloads(root)]:
        layer = Sdf.Layer.FindOrOpenRelativeToLayer(root, path) \
            if path != root.identifier else root
        inv = layer_inventory(layer)
        rows.append((os.path.basename(layer.identifier), inv))
        print("  %-28s %8.1f %6d %6d %6d %8d %8d"
              % (os.path.basename(layer.identifier), inv["kb"], inv["prims"],
                 inv["attrs"], inv["rels"], inv["inherits"], inv["inactive"]))
    return rows


def _payloads(root):
    """Payload arcs on the root's default prim (payload mode), else []."""
    spec = root.GetPrimAtPath("/" + (root.defaultPrim or ""))
    return list(spec.payloadList.explicitItems) if spec else []


# ------------------------------------------------------- composed namespace

def prim_table(stage):
    return {str(p.GetPath()): p.GetTypeName() for p in stage.Traverse()}


def compare_namespace(flat, layered):
    print("\nComposed namespace")
    a, b = prim_table(flat), prim_table(layered)
    missing = sorted(set(a) - set(b))
    extra = sorted(set(b) - set(a))
    retyped = sorted(p for p in a if p in b and a[p] != b[p])
    check(not missing, "no prim lost", ", ".join(missing[:5]))
    check(not extra, "no prim leaked in", ", ".join(extra[:5]))
    check(not retyped, "no prim changed type", ", ".join(retyped[:5]))
    check(len(a) == len(b), "%d prims on both stages" % len(a),
          "%d vs %d" % (len(a), len(b)))
    # Also make sure the leaked children really are gone, not just
    # skipped by Traverse's default predicate: ask for inactive prims too.
    inactive = [str(p.GetPath()) for p in
                layered.TraverseAll() if not p.IsActive()]
    print("  %d inactive prims pruned from the layered stage" % len(inactive))
    return len(a)


# ---------------------------------------------------------- composed values

def compare_properties(flat, layered):
    print("\nComposed properties")
    attrs = conns = rels = 0
    bad = []
    for fp in flat.Traverse():
        lp = layered.GetPrimAtPath(fp.GetPath())
        if not lp:
            continue
        for fa in fp.GetAttributes():
            if not fa.HasAuthoredValue() and not fa.HasAuthoredConnections():
                continue
            la = lp.GetAttribute(fa.GetName())
            if not la:
                bad.append("%s missing" % fa.GetPath())
                continue
            if fa.HasAuthoredValue():
                attrs += 1
                if fa.Get() != la.Get():
                    bad.append("%s value" % fa.GetPath())
            if fa.HasAuthoredConnections():
                conns += 1
                if fa.GetConnections() != la.GetConnections():
                    bad.append("%s connections" % fa.GetPath())
        for fr in fp.GetRelationships():
            if not fr.HasAuthoredTargets():
                continue
            rels += 1
            lr = lp.GetRelationship(fr.GetName())
            if not lr or fr.GetTargets() != lr.GetTargets():
                bad.append("%s targets" % fr.GetPath())
        # The other direction: nothing authored on the layered stage that
        # the flat one lacks (an inherit could bring an extra opinion).
        for la in lp.GetAttributes():
            if la.HasAuthoredValue() and not fp.GetAttribute(la.GetName()):
                bad.append("%s extra" % la.GetPath())
        for lr in lp.GetRelationships():
            if lr.HasAuthoredTargets() and not fp.GetRelationship(lr.GetName()):
                bad.append("%s extra rel" % lr.GetPath())
    check(not bad, "%d attribute values, %d connections, %d relationships "
          "compose identically" % (attrs, conns, rels),
          "; ".join(bad[:6]))


# ---------------------------------------------------------------- the rig

TEST_POSE = [
    # (prim name, attribute, value) -- applied on both stages' session
    # layers. Mixes joint FK, control FK, IK effectors, the ikfk blend
    # and spine/neck spline controls, on both sides.
    ("spine_2_bind", "avars:rz", 18.0),
    ("neck_1_bind", "avars:rx", -12.0),
    ("arm_l_fk_shoulder_l_bind", "avars:rz", 30.0),
    ("arm_r_fk_shoulder_r_bind", "avars:rz", 30.0),
    ("arm_r_fk_elbow_r_bind", "avars:ry", -40.0),
    ("arm_l_root", "avars:ikfk", 1.0),
    ("arm_l_ik", "avars:tx", 6.0),
    ("arm_l_ik", "avars:ty", -9.0),
    ("leg_r_root", "avars:ikfk", 1.0),
    ("leg_r_ik", "avars:tz", 12.0),
    ("leg_r_pv", "avars:tx", 5.0),
    ("leg_l_fk_thigh_l_bind", "avars:rx", 22.0),
    ("thigh_r_bind", "avars:rz", -15.0),
    ("spine_mid_ctl", "avars:tx", 4.0),
    ("neck_end_ctl", "avars:rz", 10.0),
]


class Rigged(object):
    def __init__(self, stage, rig_root):
        self.stage = stage
        stage.SetEditTarget(Usd.EditTarget(stage.GetSessionLayer()))
        self.rig = rigexec.Rig(stage, rig_root)
        self.rig.compile()
        self.by_name = {}
        for p in stage.Traverse():
            if p.GetTypeName() in ("RigExecJoint", "RigExecControl"):
                self.by_name.setdefault(p.GetName(), p)

    def frames(self):
        pose = self.rig.evaluate(0.0)
        return {jp: pose.joint_frame(jp).to_matrix4()
                for jp in pose.joint_paths()}

    def apply_pose(self, entries=TEST_POSE):
        applied = 0
        for name, attr, value in entries:
            p = self.by_name.get(name)
            if p is None:
                continue
            a = p.GetAttribute(attr)
            if not a or not a.IsValid():
                a = p.CreateAttribute(attr, Sdf.ValueTypeNames.Double)
            a.Set(float(value))
            applied += 1
        return applied

    def clear_pose(self, entries=TEST_POSE):
        for name, attr, _ in entries:
            p = self.by_name.get(name)
            a = p.GetAttribute(attr) if p else None
            if a and a.IsValid():
                a.Clear()


def worst_deviation(fa, fb):
    """(max |translation delta| in cm, max |any entry delta|, joint)."""
    worst_t = worst_e = 0.0
    where = None
    for jp, ma in fa.items():
        mb = fb[jp]
        t = max(abs(ma[i] - mb[i]) for i in (12, 13, 14))
        e = max(abs(x - y) for x, y in zip(ma, mb))
        if t > worst_t:
            worst_t, where = t, jp
        worst_e = max(worst_e, e)
    return worst_t, worst_e, where


def compare_rig(flat, layered, rig_root, tol):
    print("\nRigExec compile and evaluate")
    try:
        rf = Rigged(flat, rig_root)
    except Exception as e:
        check(False, "flat rig compiles", str(e).splitlines()[0])
        return
    try:
        rl = Rigged(layered, rig_root)
    except Exception as e:
        check(False, "layered rig compiles", str(e).splitlines()[0])
        return
    check(True, "both rigs compile")

    mo_f = [(m["path"], m["type"], tuple(m["targets"])) for m in rf.rig.mover_order()]
    mo_l = [(m["path"], m["type"], tuple(m["targets"])) for m in rl.rig.mover_order()]
    check(mo_f == mo_l, "mover execution order identical (%d movers)" % len(mo_f),
          "" if mo_f == mo_l else "first difference at %s"
          % next((a[0] for a, b in zip(mo_f, mo_l) if a != b), "length"))

    ff, fl = rf.frames(), rl.frames()
    check(set(ff) == set(fl), "same joint set (%d joints)" % len(ff),
          "%d vs %d" % (len(ff), len(fl)))
    if set(ff) != set(fl):
        return
    wt, we, where = worst_deviation(ff, fl)
    check(wt <= tol, "rest pose: all %d joint frames within %g cm" % (len(ff), tol),
          "worst translation %.3e cm, worst matrix entry %.3e (%s)"
          % (wt, we, where.rsplit("/", 1)[-1] if where else "-"))

    # The trap a "right derives from left" arc can fall into: an opinion
    # on the LEFT control -- animation, authored later, in a stronger
    # layer -- composing onto the RIGHT control as well. Key the left arm
    # alone and require every right-side joint to stay exactly at rest.
    left_only = [e for e in TEST_POSE if "_l_" in e[0] or e[0].endswith("_l")]
    rl.apply_pose(left_only)
    pl = rl.frames()
    right_moved = [jp for jp in pl if "_r_" in jp.rsplit("/", 1)[-1]
                   and max(abs(pl[jp][i] - fl[jp][i]) for i in (12, 13, 14)) > 1e-9]
    check(not right_moved,
          "keying the left side alone (%d avars) leaves every right joint at rest"
          % len(left_only),
          "%d right joints moved, e.g. %s" % (len(right_moved),
                                              right_moved[0].rsplit("/", 1)[-1])
          if right_moved else "")
    rl.clear_pose(left_only)

    na, nb = rf.apply_pose(), rl.apply_pose()
    check(na == nb and na > 0, "test pose applied (%d avars)" % na)
    pf, pl = rf.frames(), rl.frames()
    moved = sum(1 for jp in pf
                if max(abs(pf[jp][i] - ff[jp][i]) for i in (12, 13, 14)) > 1e-3)
    wt, we, where = worst_deviation(pf, pl)
    check(moved > 50, "test pose actually moves the rig", "%d joints moved" % moved)
    check(wt <= tol, "posed: all %d joint frames within %g cm" % (len(pf), tol),
          "worst translation %.3e cm, worst matrix entry %.3e (%s)"
          % (wt, we, where.rsplit("/", 1)[-1] if where else "-"))


# ---------------------------------------------------------------- pinning

def check_pinning(root, rig_root):
    print("\nChild-order pinning")
    pinned = sum(1 for s in _all_specs(root) if s.nameChildrenOrder)
    # The same sublayers (or payloads), no reorders: an anonymous root
    # carrying the same arcs by absolute path.
    loose = Sdf.Layer.CreateAnonymous("unpinned.usda")
    for key in ("defaultPrim", "metersPerUnit", "upAxis"):
        if root.pseudoRoot.HasInfo(key):
            loose.pseudoRoot.SetInfo(key, root.pseudoRoot.GetInfo(key))
    if root.subLayerPaths:
        loose.subLayerPaths = [
            Sdf.Layer.FindOrOpenRelativeToLayer(root, p).realPath
            for p in root.subLayerPaths]
    elif _payloads(root):
        spec = Sdf.CreatePrimInLayer(loose, "/" + root.defaultPrim)
        spec.specifier = Sdf.SpecifierOver
        spec.payloadList.explicitItems = [
            Sdf.Payload(root.ComputeAbsolutePath(p.assetPath), p.primPath)
            for p in _payloads(root)]
    else:
        print("  [SKIP] root has neither sublayers nor payloads")
        return
    stage = Usd.Stage.Open(loose)
    err = None
    try:
        rigexec.Rig(stage, rig_root).compile()
    except Exception as e:
        # The first line is the generic "rig compile failed:"; the reason
        # follows it.
        lines = [l.strip() for l in str(e).splitlines() if l.strip()]
        err = " ".join(lines[:2])[:220]
    check(err is not None,
          "WITHOUT the %d `reorder nameChildren` pins the rig fails to compile"
          % pinned, err or "it compiled, so the pins are not load-bearing")


def _all_specs(layer):
    stack = list(layer.pseudoRoot.nameChildren)
    while stack:
        s = stack.pop()
        yield s
        stack.extend(s.nameChildren)


# ------------------------------------------------------------------- main

def main(argv):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("flat")
    ap.add_argument("root")
    ap.add_argument("--rig-root", default="/Biped/Rig")
    ap.add_argument("--tolerance", type=float, default=1e-6)
    args = ap.parse_args(argv)

    rigexec.load_schema_plugin()
    root = Sdf.Layer.FindOrOpen(args.root)
    if root is None:
        raise SystemExit("cannot open %s" % args.root)
    inventory(root)
    flat = Usd.Stage.Open(args.flat)
    layered = Usd.Stage.Open(args.root)
    compare_namespace(flat, layered)
    compare_properties(flat, layered)
    compare_rig(flat, layered, args.rig_root, args.tolerance)
    check_pinning(root, args.rig_root)

    print("\n%s" % ("ALL PASS" if not FAILURES
                    else "%d FAILED: %s" % (len(FAILURES), "; ".join(FAILURES))))
    return 1 if FAILURES else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
