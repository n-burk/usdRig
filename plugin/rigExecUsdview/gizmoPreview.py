#
# RigExec usdview gizmo: the manipulation preview channel.
#
# A drag does not author. It hands its uncommitted values to Hydra and
# authors once, on release (design note: docs/superpowers/specs/
# 2026-09-10-hydra-preview-manipulation-design.md), and this module is the
# one route those values take:
#
#   Push(pending)  per mouse sample -- the values, nowhere near the stage
#   End()          on release or abort -- the preview is over
#
# Qt-free, and the C++ side is reached through an installed SINK rather than
# directly, so the protocol is testable with no library, no display and no
# usdview (tests/python/test_gizmo_preview.py). A session with no sink --
# headless, or an older rigExecImaging that does not export the entry points --
# previews nothing and authors on release exactly the same way; the viewport
# simply does not update until then.
#
# WHY THE DECLARATION IS SEPARATE. The sink is told which attributes a
# manipulation will change once, and then fed nothing but numbers. Resolving a
# path to a rig, reading its value type and choosing its preview lane are all
# done on the C++ side at declaration time, because the only part of this that
# runs per mouse sample should be the part that has to.
#
# WHY THE KEY SET CAN CHANGE MID-DRAG. A rotate that starts writing avars:rspin
# partway through, or an xform drag whose op is created by the first Apply,
# adds an attribute the declaration did not mention. So the key set is compared
# on every push and re-declared when it differs -- which is rare, and cheap
# when it happens.
#
try:
    import gizmoMath
except ImportError:                    # loader that did not add our dir
    import os
    import sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import gizmoMath


# The installed sink: an object with Begin(packedPaths) -> int,
# Update(values) -> bool and End() -> bool. None means no Hydra preview.
_sink = None

# What the sink was last told this manipulation would change, so a push only
# re-declares when the answer has changed.
_declared = ()
# How many doubles the sink said it expects for _declared, as a check that the
# two sides agree about arity before any value is sent.
_expected = 0


def SetSink(sink):
    """Install the host's preview sink, or None to preview nothing."""
    global _sink, _declared, _expected
    _sink = sink
    _declared = ()
    _expected = 0


def HasSink():
    return _sink is not None


def Flatten(value):
    """
    `value` as a flat list of doubles, or None when it is not a kind the
    preview channel carries.

    The arity has to agree with what the C++ side derived from the
    ATTRIBUTE's type, so the shapes here are exactly the ones it accepts:
    a scalar, a 3-vector, and a 4x4 matrix.
    """
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        return [float(value)]
    # Gf.Vec3d / Gf.Vec3f / Gf.Matrix4d are all indexable; a matrix by row.
    try:
        length = len(value)
    except TypeError:
        return None
    if length == 3:
        try:
            return [float(value[i]) for i in range(3)]
        except (TypeError, ValueError):
            return None
    if length == 4:
        out = []
        for row in range(4):
            try:
                out.extend(float(value[row][column]) for column in range(4))
            except (TypeError, ValueError, IndexError):
                return None
        return out
    return None


def Push(pending):
    """
    One mouse sample: `pending` is {Sdf.Path: value} from the Writer.

    Tells Hydra what the artist is doing. The gizmo's own handles are already
    following it -- Writer.Set publishes each value into gizmoMath's preview
    map as it collects it -- so this call is what puts the same values in front
    of the renderer. Returns True when the sink took them.
    """
    global _declared, _expected
    # gizmoMath already holds these: Writer.Set put them in the preview map as
    # it collected them, so the handles and the frame maths are reading them
    # before this is called. What is left to do is tell Hydra.
    if _sink is None or not pending:
        return False
    keys = tuple(str(path) for path in pending.keys())
    values = []
    for value in pending.values():
        flat = Flatten(value)
        if flat is None:
            # An unpreviewable value would desynchronize every slot after it,
            # so the sample is dropped whole rather than sent misaligned. The
            # release still authors it.
            return False
        values.extend(flat)
    if keys != _declared:
        expected = _sink.Begin("\n".join(keys))
        if expected is None or expected < 0:
            _declared = ()
            _expected = 0
            return False
        _declared = keys
        _expected = int(expected)
    if len(values) != _expected:
        # The two sides disagree about arity -- a value whose type is not the
        # attribute's. Sending it would shift every later slot.
        return False
    return bool(_sink.Update(values))


def End():
    """
    The manipulation is over: drop the preview on both sides.

    Idempotent, and safe to call after a push that failed or never happened,
    because an abort and a commit both arrive here and neither knows which
    samples the sink accepted.
    """
    global _declared, _expected
    gizmoMath.SetPreviewValues({})
    _declared = ()
    _expected = 0
    if _sink is None:
        return False
    return bool(_sink.End())


def IsPreviewing():
    return bool(_declared)
