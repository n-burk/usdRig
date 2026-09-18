"""Read a `.touch` file. No `pxr`, no Qt, no ctypes, no the conventional tool.

This is the format layer of the TouchPose port and the bottom of its
stack: it turns the studio's `.touch` document into plain records, and
nothing above it has to know what JSON keys the studio chose. Keeping it
host-free is what lets the importer, the spikes and (later) the picker
model all share one reader, and lets all three be tested with nothing
loaded.

WHAT A `.touch` FILE IS, from reading the shipped one
(`biped/rig/default/data/build/touch_sets.touch`, 247 sets, 567 KB):

It is JSON -- `{user, type: "TouchSetsData", time, data}` -- where `data`
maps a conventional `objectSet` name to its record. One entry is special:

  * `data["TouchSets"]` is the GROUP. Its `members` is the ordered list
    of set names; it carries no faces. It also carries the PALETTE:
    `touchColor0` .. `touchColor5`, six linear RGB triples, plus
    `leadColor`, `selectedColor` and `alpha` (0.478 in the shipped file)
    -- the global overlay opacity.

  * every other entry is a SET:
      - `members`: face components, each `"<mesh>.f[i]"` or
        `"<mesh>.f[a:b]"`, INCLUSIVE at both ends, plus exactly one
        trailing `"<setname>_data"` which is a node of the conventional tool, not geometry,
        and is dropped here.
      - `touch_hilight`: int 0..5, an INDEX INTO THE GROUP PALETTE. It is
        not a colour and not a boolean. Shipped distribution: 148 sets at
        0, 76 at 3, 13 at 2, 10 at 1.
      - `touchColor`: the set's own linear RGB, which is what the conventional tool
        product actually drew; `touch_hilight` selects the colour used
        for a DIFFERENT state (the six-colour palette is the layer/state
        ramp). Both are carried through; which one the overlay uses is a
        presentation decision, not a format one.
      - `command` / `touch_layers`: the conventional tool right-click menus, verbatim
        `conventional.cmds` source. Deliberately NOT parsed. They are the part of
        the product that does not port, and reading them here would drag
        a conventional-tool dependency into the bottom layer for nothing.

THE CONTROL BINDING IS THE NAME. There is no `control` field: a set
called `thumb_003_r_touch` binds the control `thumb_003_r`, in BUILD
naming. The `_touch` suffix is the whole convention, and three sets
(`toe_l`, `arm_l_prop`, `arm_r_prop`) are name-only with no faces at all
-- bound controls whose region was never painted.

THREE THINGS THE SHIPPED FILE DOES NOT EXERCISE, and which are handled
here anyway because the TouchPose Blender port's `domain/schema.py`
(branch `feature/blender-port`, read not copied) documents them as having
been verified against the conventional tool's writer:

  * The colour key was renamed. `touch_sets_data.py` defines the colour
    attribute twice -- `touch_color` and then `touchColor` -- and the
    second wins, so v1.0-era files on disk carry `touch_color` and the conventional tool
    itself silently randomises their colours on load. Both keys are read.
  * The envelope may carry a `version`. The shipped file has none, which
    puts it before the version ladder started; a future export will have
    one, and a `faces` block that states membership without the conventional tool's
    component grammar. Preferred when present.
  * `members` can also contain NESTED SET names, making the sets a tree.
    The shipped file is flat (every set's `parent` is `TouchSets`, and
    its only non-component member is its own `_data` node), so nothing
    here walks a tree -- but the names are kept rather than discarded, so
    a nested file is visibly unhandled rather than silently flattened.
"""
import io
import json
import re

SUFFIX = "_touch"
GROUP = "TouchSets"

# `body_geo.f[21416:21537]` or `body_geo.f[21590]`. the conventional tool ranges are
# inclusive, which is the one thing in this format that is easy to get
# wrong by one face at every range end -- 3783 ranges on body_geo alone.
_COMPONENT = re.compile(r"^(?P<mesh>[^.]+)\.f\[(?P<lo>\d+)(?::(?P<hi>\d+))?\]$")


class Palette(object):
    """The group's six-colour ramp, plus the overlay's global alpha."""

    def __init__(self, record):
        self.colors = [tuple(record["touchColor%d" % i])
                       for i in range(6) if ("touchColor%d" % i) in record]
        self.lead = tuple(record.get("leadColor", (0.0, 0.0, 0.0)))
        self.selected = tuple(record.get("selectedColor", (0.0, 0.0, 0.0)))
        self.alpha = float(record.get("alpha", 1.0))

    def color(self, index):
        if not self.colors:
            return (0.5, 0.5, 0.5)
        return self.colors[index % len(self.colors)]

    def __repr__(self):
        return "<Palette %d colors alpha=%.3f>" % (len(self.colors),
                                                   self.alpha)


class TouchSet(object):
    """One region: a control name, faces per mesh, a colour, a palette index.

    `faces` is `{mesh_name: sorted list of face indices}`. The ranges are
    expanded here rather than kept as ranges because every consumer --
    a GeomSubset's `indices`, a centroid, a face-under-cursor test --
    wants the flat list, and 30,801 indices is nothing.
    """

    def __init__(self, name, record):
        self.name = name
        self.control = name[:-len(SUFFIX)] if name.endswith(SUFFIX) else name
        self.hilight = int(record.get("touch_hilight", 0))
        # `touchColor` first, `touch_color` for v1.0-era files; see the
        # module docstring for why both exist.
        self.color = tuple(record.get("touchColor",
                                      record.get("touch_color",
                                                 (0.5, 0.5, 0.5))))
        self.faces = {}
        self.unparsed = []

        # v4.0 states membership without the conventional tool component grammar. Additive,
        # so it is preferred when present and the components are still
        # parsed when it is not.
        for mesh, indices in (record.get("faces") or {}).items():
            self.faces.setdefault(mesh, []).extend(int(i) for i in indices)

        for member in record.get("members", ()):
            match = _COMPONENT.match(member)
            if match is None:
                # The `<set>_data` node, and anything else the conventional tool slipped in.
                self.unparsed.append(member)
                continue
            lo = int(match.group("lo"))
            hi = int(match.group("hi") or lo)
            self.faces.setdefault(match.group("mesh"), []).extend(
                range(lo, hi + 1))
        for indices in self.faces.values():
            indices.sort()

    @property
    def meshes(self):
        return sorted(self.faces)

    @property
    def face_count(self):
        return sum(len(v) for v in self.faces.values())

    def faces_on(self, mesh):
        return self.faces.get(mesh, [])

    def __repr__(self):
        return "<TouchSet %s control=%s %d faces on %s>" % (
            self.name, self.control, self.face_count, ",".join(self.meshes))


class TouchDocument(object):
    """A parsed `.touch`: an ordered list of sets and one palette."""

    def __init__(self, payload):
        data = payload.get("data", {})
        group = data.get(GROUP, {})
        self.palette = Palette(group)
        self.user = payload.get("user")
        self.time = payload.get("time")
        # Absent in the shipped file, which predates the version ladder.
        self.version = payload.get("version")

        # The group's `members` gives the studio's ORDER, which is also
        # the draw order the conventional product used for overlapping regions.
        # Falling back to dict order keeps a hand-edited file readable.
        order = [m for m in group.get("members", ()) if m in data]
        order += [k for k in data if k != GROUP and k not in order]
        self.sets = [TouchSet(name, data[name]) for name in order]

    @property
    def meshes(self):
        out = set()
        for s in self.sets:
            out.update(s.faces)
        return sorted(out)

    def face_counts(self):
        """mesh -> (number of faces referenced, highest index seen + 1).

        The second number is the MINIMUM face count the mesh must have
        for this file to apply to it. It is a necessary check and not a
        sufficient one: a mesh with the right face count can still have
        had its faces reordered, which is what spike R1 measures.
        """
        counts = {}
        for s in self.sets:
            for mesh, indices in s.faces.items():
                have, top = counts.get(mesh, (0, 0))
                counts[mesh] = (have + len(indices),
                                max(top, indices[-1] + 1 if indices else 0))
        return counts

    def by_control(self):
        return {s.control: s for s in self.sets}


def read(path):
    with io.open(path, encoding="utf-8") as handle:
        return TouchDocument(json.load(handle))
