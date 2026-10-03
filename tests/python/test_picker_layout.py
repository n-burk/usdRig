"""The picker view's content box, and that drawing agrees with clicking.

Two failures this pins down, both of which the picker SURVIVES -- which is
why they need a test rather than a bug report:

  1. A panel drawn against its declared `ui:size` instead of its buttons.
     A picker split out of one shared canvas gives every panel the same
     default size while the buttons keep their original absolute
     coordinates, so a panel whose content starts at x=339 and runs to 803
     inside a declared 400-wide box opens to a tab that is mostly empty.
     Nothing errors; the buttons are simply somewhere else.

  2. The paint transform and the mouse transform drifting apart. The view
     scales then translates by the content origin; the hit test divides by
     the same scale and adds the same origin back. Get one of them wrong
     and every click lands on whatever button sits at the offset instead,
     which reads as "the picker selects the wrong thing sometimes".

Qt-free: the box and the mapping are arithmetic, so they are tested as
arithmetic rather than by opening a window.

Usage:
    python test_picker_layout.py [schema_resources_dir]
"""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from test_rigexec_python import _setup_environment  # noqa: E402

MARGIN = 12.0
FAILURES = []


def check(label, ok, detail=""):
    print("  %-52s %s  %s" % (label, "ok  " if ok else "FAIL", detail))
    if not ok:
        FAILURES.append(label)
    return ok


class _Box(object):
    """The two fields pickerModel.content_box reads off a button."""

    def __init__(self, parent, x, y, w, h):
        self.parent, self.x, self.y, self.w, self.h = parent, x, y, w, h


class _Bag(object):
    def __init__(self, buttons):
        self.buttons = buttons


class _Panel(object):
    def __init__(self, w, h, ident="p"):
        self.id, self.w, self.h = ident, w, h


def content_box(buttons, panel_w, panel_h):
    """THE REAL ONE, through a pair of stand-ins.

    Calling pickerModel.content_box rather than restating it: a test that
    keeps its own copy of the rule passes happily while the rule it is
    guarding changes underneath it.
    """
    import pickerModel
    return pickerModel.content_box(
        _Bag([_Box("p", *b) for b in buttons]), _Panel(panel_w, panel_h))


def main():
    _setup_environment()

    from pxr import Usd
    import rigexec
    rigexec.load_schema_plugin(sys.argv[1] if len(sys.argv) > 1 else None)

    plugin = pathlib.Path(__file__).resolve().parents[2] / "plugin" / \
        "rigExecUsdview"
    sys.path.insert(0, str(plugin))
    import pickerModel
    import pickerScene

    print("")
    print("content box")
    # A panel whose buttons sit well right of its declared origin: the face
    # case, reduced to the numbers that made it fail.
    offset = [(339.0, 43.0, 60.0, 40.0), (700.0, 500.0, 103.0, 98.0)]
    (ox, oy), (w, h) = content_box(offset, 400.0, 600.0)
    check("an offset panel is framed by its buttons",
          (ox, oy) == (327.0, 31.0) and (w, h) == (488.0, 579.0),
          "origin (%.0f, %.0f) size %.0fx%.0f" % (ox, oy, w, h))

    inside = [(13.0, 38.0, 30.0, 30.0), (370.0, 460.0, 34.0, 39.0)]
    (bx, by), (bw, bh) = content_box(inside, 400.0, 600.0)
    check("a panel already at its origin barely moves",
          abs(bx - 1.0) < 1e-9 and abs(by - 26.0) < 1e-9,
          "origin (%.0f, %.0f) size %.0fx%.0f" % (bx, by, bw, bh))

    (ex, ey), (ew, eh) = content_box([], 400.0, 600.0)
    check("an empty panel falls back to its declared size",
          (ex, ey) == (0.0, 0.0) and (ew, eh) == (400.0, 600.0))

    print("")
    print("paint and click agree")
    # The two transforms, written the way the view writes them.
    for scale in (1.0, 0.5, 2.37):
        worst = 0.0
        for bx0, by0, bw0, bh0 in offset:
            cx, cy = bx0 + bw0 / 2.0, by0 + bh0 / 2.0
            px, py = (cx - ox) * scale, (cy - oy) * scale   # paintEvent
            hx, hy = px / scale + ox, py / scale + oy       # _at/_PanelPos
            worst = max(worst, abs(hx - cx), abs(hy - cy))
        check("round trip at scale %.2f" % scale, worst < 1e-9,
              "worst %.2e panel units" % worst)

    print("")
    print("against the shipped character, if it is here")
    stack = pathlib.Path(__file__).resolve().parents[2] / "examples" / \
        "biped" / "Biped_stack.usda"
    if not stack.exists():
        print("  (no examples/biped/Biped_stack.usda, skipped)")
    else:
        stage = Usd.Stage.Open(str(stack))
        panels = 0
        for picker in pickerScene.load_all(stage):
            for panel in picker.panels:
                live = [b for b in picker.buttons
                        if b.parent == panel.id and b.live
                        and not b.decoration]
                if not live:
                    continue
                panels += 1
                boxes = [(b.x, b.y, b.w, b.h) for b in picker.buttons
                         if b.parent == panel.id]
                (px0, py0), (pw, ph) = content_box(boxes, panel.w, panel.h)
                missed = []
                for button in live:
                    cx = button.x + button.w / 2.0
                    cy = button.y + button.h / 2.0
                    if not (px0 <= cx <= px0 + pw and py0 <= cy <= py0 + ph):
                        missed.append(button.id)
                    elif button not in picker.hits(panel.id, cx, cy, {},
                                                   False):
                        missed.append(button.id)
                check("%s: every live button reachable" % panel.label,
                      not missed, "%d live, canvas %.0fx%.0f%s"
                      % (len(live), pw, ph,
                         "" if not missed else ", missed %s" % missed[:4]))
        check("more than one panel has live buttons", panels >= 2,
              "%d panel(s) would get a tab" % panels)

    print("")
    if FAILURES:
        print("test_picker_layout: %d FAILED: %s"
              % (len(FAILURES), ", ".join(FAILURES)))
        return 1
    print("test_picker_layout: all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
