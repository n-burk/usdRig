#!/usr/bin/env python
"""Split a built biped into center / left / right layers behind one root,
with the right side expressed as a REFERENCE to the left plus overrides.

Why this shape. Measured on the real rig, the two sides are structurally
IDENTICAL -- 121 prims each -- and only a handful of properties differ:

    rest:space              109  the mirrored geometry (97 joints, 12 ctls)
    guide:displayColor       12  blue vs red
    inputs:aimVector          6  mirrored aim axes
    ui:nodegraph:node:pos   121  editor layout, not rig data
    inputs:weight.connect     2  the ikfk blend reads its own side's ctl
    <every relationship>     30  wiring to that side's joints and controls

So the interesting split is by SIDE, and the right side really is the left
side with a handful of overrides. Each right prim that has anything to
share is authored as

    def RigExecControl "arm_r_ik" (
        prepend references = @./<stem>_left.usda@</Biped/Rig/Controls/arm_l_ik>
    ) {
        color3f guide:displayColor = (1, 0, 0)
        matrix4d rest:space = ...
    }

and gets its guide shape, scale, up-vector, world-up type and so on from
the left prim through the arc.

Why a file reference and not `inherits`. Both compose the left prim's
opinions onto the right and both rebase in-scope relationship targets
(verified), but `inherits` (and `specializes`, and an internal reference)
takes the left prim as COMPOSED ON THE STAGE -- so an animator keying
`avars:rz` on the left arm control keys the right arm too, because that
opinion sits on the inherited prim. Measured: keying the left shoulder
alone moved the right hand 25 cm. Pinning fallback avars on the right
cannot close that hole (a `custom` avar added to the left later leaks the
same way). Referencing the left LAYER FILE composes only what is saved in
that layer: rig data flows, animation in stronger layers does not. It also
survives `--payload`, which an inherit does not (an inherit is implied up
to the root node and outranks every payload, so the left would win over
the right's own overrides).

What is, and is not, shared -- measured, not guessed:

  * Joints share NOTHING. A RigExecJoint carries only rest:space and the
    editor position, and both differ by side. An inherit arc on a joint
    would buy no attributes and would compose the left joint's sided
    children under the right joint, so joints are authored plainly.
  * Relationships are never shared. `inherits` rebases targets that live
    INSIDE the arc's scope onto the instance (verified), but a limb's
    `rigExec:joints` / `rigExec:controls` / `rigExec:moves` point at joints
    nested under CENTER joints (hips_bind, chest_bind) and at sibling
    controls -- all outside any per-prim scope. They are authored per side.
    Sharing them would need the arc to sit on a common ancestor of solver
    and joints, i.e. /Biped/Rig, which is the whole rig. That is a builder
    restructure (limbs as class instances placed inside the joint
    hierarchy, skin influences remapped by path), not a post-process.
  * A left prim with sided children (nested FK controls) leaks those
    children into the right prim through the arc. Each leaked child gets
    `over "..." (active = false)` in the right layer, which prunes it from
    the composed namespace before RigExec sees it.

The right side therefore authors the differing attributes, the wiring, and
the deactivations, and nothing else. `--verbatim` writes the plain copy
instead, for comparison.

Sublayers, not payloads. The layers are a few hundred KB and a rig is
useless half-loaded: a limb with its joints but no solver, or the skin
mover without the joints it names, does not compile. `--payload` still
exists for a load-on-demand arrangement and verifies identically; the
right side's references open the left FILE directly, so they resolve even
if the left payload itself is unloaded.

The root layer also PINS child order (`reorder nameChildren`). RigExec
derives mover execution order from namespace order, and composing a prim's
children from three sublayers does not preserve the order they had in one
layer; without the pin the split rig fails to compile outright.
`verify_layers.py` demonstrates that.

Usage:
    split_layers.py <built.usda> <out_root.usda> [--payload] [--verbatim]

Writes `<out_root>`, plus `<stem>_center.usda`, `_left.usda`, `_right.usda`
beside it.
"""
import argparse
import os
import re
import sys

from pxr import Sdf, Usd

# A prim belongs to a side if its name carries the side token. Everything
# else -- spine, neck, hips, chest, the mesh, materials, the skin mover --
# is center. `_l_`/`_r_` infixes and `_l`/`_r` suffixes both occur
# (`shoulder_l_bind`, `arm_l_ik`, `pelvis_l`), so match both.
_LEFT = re.compile(r"(^|_)l($|_)")
_RIGHT = re.compile(r"(^|_)r($|_)")


def side_of(name):
    if _LEFT.search(name):
        return "left"
    if _RIGHT.search(name):
        return "right"
    return None


def classify(path):
    """Side for a prim path: the first sided component wins.

    Descendants inherit their ancestor's side, so a control nested under a
    left pivot lands in the left layer even if its own name is neutral.
    """
    for part in str(path).split("/"):
        if not part:
            continue
        s = side_of(part)
        if s:
            return s
    return "center"


def mirror_to_left(path):
    """The left-side path a right-side path mirrors: swap every `r` token."""
    def swap(part):
        return _RIGHT.sub(lambda m: m.group(1) + "l" + m.group(2), part)
    return Sdf.Path("/".join(swap(p) for p in str(path).split("/")))


def _attr_signature(spec):
    return (spec.typeName, spec.variability, spec.custom, spec.default,
            list(spec.connectionPathList.explicitItems))


def _rebased_targets(left_spec, left_prim, right_prim):
    """What a relationship's targets compose to through the file reference.

    Targets inside the referenced prim are mapped onto the right prim;
    targets outside it have no mapping across a file reference and are
    dropped, so any relationship that reaches outside must be authored on
    the right prim itself.
    """
    out = []
    for t in left_spec.targetPathList.explicitItems:
        if t == left_prim or t.HasPrefix(left_prim):
            out.append(t.ReplacePrefix(left_prim, right_prim))
    return out


def split(src_path, out_root, use_payload=False, inherit=True):
    src = Sdf.Layer.FindOrOpen(src_path)
    if src is None:
        raise SystemExit("cannot open %s" % src_path)

    root_dir = os.path.dirname(os.path.abspath(out_root)) or "."
    stem = os.path.splitext(os.path.basename(out_root))[0]
    names = {s: "%s_%s.usda" % (stem, s)
             for s in ("center", "left", "right")}
    layers = {}
    for s, fname in names.items():
        full = os.path.join(root_dir, fname)
        if os.path.exists(full):
            os.remove(full)
        layers[s] = Sdf.Layer.CreateNew(full)

    counts = {"center": 0, "left": 0, "right": 0}
    stats = {"inherits": 0, "deactivated": 0, "attrs_shared": 0,
             "attrs_local": 0, "rels_local": 0, "plain_right": 0}
    # Right prim path -> left prim path it inherits, for the nested case.
    inherited = {}

    def ensure_ancestors(layer, path):
        """Create `over` specs for every ancestor, so the child can land."""
        parts = str(path).strip("/").split("/")
        cur = ""
        for part in parts[:-1]:
            cur = cur + "/" + part
            if not layer.GetPrimAtPath(cur):
                spec = Sdf.CreatePrimInLayer(layer, Sdf.Path(cur))
                spec.specifier = Sdf.SpecifierOver
        return Sdf.Path(str(path))

    def copy_prim_own(spec, layer):
        """Copy one prim's own opinions (not its children) verbatim."""
        dst = Sdf.CreatePrimInLayer(layer, spec.path)
        dst.specifier = spec.specifier
        if spec.typeName:
            dst.typeName = spec.typeName
        for key in spec.ListInfoKeys():
            if key in ("specifier", "typeName"):
                continue
            try:
                dst.SetInfo(key, spec.GetInfo(key))
            except Exception:
                pass
        for prop in spec.properties:
            Sdf.CopySpec(src, prop.path, layer, prop.path)
        return dst

    def inherit_source(spec):
        """The left prim this right prim can inherit, or None.

        Inherit only where the arc actually shares something: the left
        counterpart exists, has the same type, and at least one attribute
        is identical. Joints fail the last test and are authored plainly.
        """
        lp = mirror_to_left(spec.path)
        if lp == spec.path:
            return None
        left = src.GetPrimAtPath(lp)
        if left is None or left.typeName != spec.typeName:
            return None
        shared = 0
        for prop in left.attributes:
            mine = spec.attributes.get(prop.name)
            if mine is not None and _attr_signature(mine) == _attr_signature(prop):
                shared += 1
        return lp if shared else None

    def copy_prim_inheriting(spec, layer, lp):
        """Author a right prim as a reference to its left twin plus diffs."""
        left = src.GetPrimAtPath(lp)
        rp = spec.path
        dst = Sdf.CreatePrimInLayer(layer, rp)
        dst.specifier = spec.specifier
        if spec.typeName:
            dst.typeName = spec.typeName
        # A prim whose parent already inherits its twin's parent composes
        # through that arc; adding a second one would be redundant.
        parent_inherits = inherited.get(rp.GetParentPath())
        if parent_inherits == lp.GetParentPath() and rp.name == lp.name:
            pass
        else:
            dst.referenceList.prependedItems = [
                Sdf.Reference("./" + names["left"], lp)]
            stats["inherits"] += 1
        inherited[rp] = lp
        for key in spec.ListInfoKeys():
            if key in ("specifier", "typeName"):
                continue
            if key in left.ListInfoKeys() and left.GetInfo(key) == spec.GetInfo(key):
                continue
            try:
                dst.SetInfo(key, spec.GetInfo(key))
            except Exception:
                pass
        for prop in spec.properties:
            twin = left.properties.get(prop.name)
            if isinstance(prop, Sdf.AttributeSpec):
                if twin is not None and isinstance(twin, Sdf.AttributeSpec) \
                        and _attr_signature(twin) == _attr_signature(prop):
                    stats["attrs_shared"] += 1
                    continue
                Sdf.CopySpec(src, prop.path, layer, prop.path)
                stats["attrs_local"] += 1
            else:
                want = list(prop.targetPathList.explicitItems)
                if twin is not None and isinstance(twin, Sdf.RelationshipSpec) \
                        and _rebased_targets(twin, lp, rp) == want:
                    continue
                Sdf.CopySpec(src, prop.path, layer, prop.path)
                stats["rels_local"] += 1
        # Leaked children: every left child the right prim does not itself
        # define would compose under the right prim with its left name.
        mine = set(c.name for c in spec.nameChildren)
        for child in left.nameChildren:
            if child.name in mine:
                continue
            leak = Sdf.CreatePrimInLayer(layer, rp.AppendChild(child.name))
            leak.specifier = Sdf.SpecifierOver
            leak.active = False
            stats["deactivated"] += 1
        return dst

    # Walk prim specs deepest-last so parents exist before children, and
    # copy each prim's OWN opinions to its side's layer. Sdf.CopySpec would
    # take the whole subtree, and a subtree is almost never single-sided --
    # /Biped is center but holds every left and right joint.
    def visit(spec):
        path = spec.path
        if path == Sdf.Path.absoluteRootPath:
            for child in spec.nameChildren:
                visit(child)
            return
        side = classify(path)
        layer = layers[side]
        ensure_ancestors(layer, path)
        children = list(spec.nameChildren)
        lp = inherit_source(spec) if (inherit and side == "right") else None
        if lp is not None:
            copy_prim_inheriting(spec, layer, lp)
        else:
            copy_prim_own(spec, layer)
            if side == "right":
                stats["plain_right"] += 1
        counts[side] += 1
        for child in children:
            visit(child)

    visit(src.pseudoRoot)

    for s in layers:
        layers[s].defaultPrim = src.defaultPrim
        layers[s].Save()

    # The root: sublayers (strongest first) or a payload per side.
    if os.path.exists(out_root):
        os.remove(out_root)
    root = Sdf.Layer.CreateNew(out_root)
    root.defaultPrim = src.defaultPrim
    # Stage metadata has to be on the ROOT layer to take effect -- an
    # opinion in a sublayer is ignored for metersPerUnit/upAxis, so a split
    # that left them behind would silently change the asset's units.
    for key in ("metersPerUnit", "upAxis", "timeCodesPerSecond"):
        if key in src.pseudoRoot.ListInfoKeys():
            root.pseudoRoot.SetInfo(key, src.pseudoRoot.GetInfo(key))

    # PIN THE CHILD ORDER. RigExec derives mover execution order from
    # namespace order (reversed sibling post-order, spec 4.2), and
    # composing a prim's children from three sublayers does NOT preserve
    # the order they had in one layer. Without this the split rig fails to
    # compile outright:
    #   "Unsatisfied final read: body_geo_skin reads final of
    #    thighNoTwist_l_bind but a writer with a later ordinal exists"
    # because the skin mover stops sorting last. `reorder nameChildren` in
    # the ROOT layer restores the authored order over the whole stack.
    def pin_order(spec):
        children = list(spec.nameChildren)
        if len(children) > 1:
            path = spec.path
            if path != Sdf.Path.absoluteRootPath:
                dst = Sdf.CreatePrimInLayer(root, path)
                dst.specifier = Sdf.SpecifierOver
                dst.nameChildrenOrder = [c.name for c in children]
                return 1 + sum(pin_order(c) for c in children)
        return sum(pin_order(c) for c in children)

    reordered = pin_order(src.pseudoRoot)

    if use_payload:
        # Payloads need a prim to hang from, so the root carries an over of
        # the default prim with three payload arcs. A right prim's local
        # overrides and its reference arc both live in the right payload
        # node, so the overrides win there just as they do in a sublayer.
        # (An `inherits` would NOT: it is implied up to the root node and
        # outranks every payload, so the left would win over the right.)
        default = src.defaultPrim or "Biped"
        spec = Sdf.CreatePrimInLayer(root, Sdf.Path("/" + default))
        spec.specifier = Sdf.SpecifierOver
        spec.payloadList.explicitItems = [
            Sdf.Payload("./" + names[s], "/" + default)
            for s in ("center", "left", "right")]
    else:
        # Sublayer order is strongest-first. Right last so a right-side
        # override in its own layer is the weakest of the three and cannot
        # silently win over a deliberate center opinion.
        root.subLayerPaths = ["./" + names[s]
                              for s in ("center", "left", "right")]
    root.Save()

    print("wrote %s (%s), %d child orders pinned"
          % (out_root, "payloads" if use_payload else "sublayers",
             reordered))
    for s in ("center", "left", "right"):
        full = os.path.join(root_dir, names[s])
        print("  %-8s %-30s %5d prims  %6.1f KB"
              % (s, names[s], counts[s], os.path.getsize(full) / 1024.0))
    if inherit:
        print("  right: %d prims reference their left twin, %d authored "
              "plainly (joints); %d attrs come through the arc, %d attrs "
              "and %d rels authored locally, %d leaked children "
              "deactivated"
              % (stats["inherits"], stats["plain_right"],
                 stats["attrs_shared"], stats["attrs_local"],
                 stats["rels_local"], stats["deactivated"]))
    return 0


def main(argv):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("source")
    ap.add_argument("out_root")
    ap.add_argument("--payload", action="store_true",
                    help="payload each side instead of sublayering")
    ap.add_argument("--verbatim", action="store_true",
                    help="copy the right side verbatim instead of "
                         "referencing the left")
    args = ap.parse_args(argv)
    return split(args.source, args.out_root, args.payload,
                 inherit=not args.verbatim)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
