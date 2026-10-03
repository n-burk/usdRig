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
# ONE CHANNEL PER SESSION. Several usdview sessions can share this module in
# one process (usdOrchestrate's host), and each previews into its own stage's
# imaging context through its own sink. So the sink and the declaration state
# live on a PreviewChannel filed under the session (the usdview main window),
# and every entry point below takes the session it is for. A call with no
# session is the old single-session call: it reaches the one session there is
# (or the one owning the active window), and before any session has installed
# a sink -- headless, the tests -- a keyless channel of its own.
#
try:
    import gizmoMath
    import sessionRegistry
except ImportError:                    # loader that did not add our dir
    import os
    import sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import gizmoMath
    import sessionRegistry


class PreviewChannel(object):
    """
    One session's route to Hydra: its sink, and what that sink was told.

    `keyless` marks the channel callers without a session reach before
    any session exists; its End() drops every stage's preview values, as
    the module-level End() always did.
    """

    def __init__(self, keyless=False):
        # The installed sink: an object with Begin(packedPaths) -> int,
        # Update(values) -> bool and End() -> bool. None means no Hydra
        # preview.
        self._sink = None
        # What the sink was last told this manipulation would change, so a
        # push only re-declares when the answer has changed.
        self._declared = ()
        # How many doubles the sink said it expects for _declared, as a
        # check that the two sides agree about arity before any value is
        # sent.
        self._expected = 0
        # The stages this manipulation pushed values for, so End() drops
        # exactly their uncommitted values and no other session's.
        self._stages = []
        self._keyless = keyless

    def SetSink(self, sink):
        self._sink = sink
        self._declared = ()
        self._expected = 0

    def HasSink(self):
        return self._sink is not None

    def IsPreviewing(self):
        return bool(self._declared)

    def _NoteStage(self, stage):
        if stage is None:
            return
        for known in self._stages:
            if known is stage:
                return
        self._stages.append(stage)

    def Push(self, pending, stage=None):
        """See the module-level Push."""
        self._NoteStage(stage)
        # gizmoMath already holds these: Writer.Set put them in the preview
        # map as it collected them, so the handles and the frame maths are
        # reading them before this is called. What is left to do is tell
        # Hydra -- and the listeners, who are told whether or not there is a
        # Hydra to tell: a session with no sink (headless, or no
        # rigExecImaging) still has a drag in flight, and the editor's
        # fields should say so even when the viewport cannot.
        if not pending:
            return False
        _Notify(pending)
        if self._sink is None:
            return False
        keys = tuple(str(path) for path in pending.keys())
        values = []
        for value in pending.values():
            flat = Flatten(value)
            if flat is None:
                # An unpreviewable value would desynchronize every slot
                # after it, so the sample is dropped whole rather than sent
                # misaligned. The release still authors it.
                return False
            values.extend(flat)
        if keys != self._declared:
            expected = self._sink.Begin("\n".join(keys))
            if expected is None or expected < 0:
                self._declared = ()
                self._expected = 0
                return False
            self._declared = keys
            self._expected = int(expected)
        if len(values) != self._expected:
            # The two sides disagree about arity -- a value whose type is
            # not the attribute's. Sending it would shift every later slot.
            return False
        return bool(self._sink.Update(values))

    def End(self, stage=None, publish=True):
        """See the module-level End."""
        if self._keyless and stage is None and not self._stages:
            gizmoMath.SetPreviewValues({})
        else:
            self._NoteStage(stage)
            for known in self._stages:
                gizmoMath.SetPreviewValues({}, stage=known)
        self._stages = []
        declared = bool(self._declared)
        self._declared = ()
        self._expected = 0
        # AFTER the preview map is cleared: a listener that re-reads its
        # values on End has to find the stage's, not the ones just dropped.
        _Notify(None)
        if self._sink is None:
            return False
        if not declared:
            # Nothing was ever begun on the host side, so there is nothing
            # to drop. The host's End republishes the rig (about 10 ms on
            # the biped), and every selection change aborts a drag and
            # lands here.
            return True
        if publish:
            return bool(self._sink.End())
        try:
            return bool(self._sink.End(publish=False))
        except TypeError:
            # A sink that cannot defer its republish (a test double, an
            # older host) ends the ordinary way.
            return bool(self._sink.End())


# Every session's channel, filed under its main window.
_channels = sessionRegistry.SessionRegistry("gizmoPreview channels")

# What a caller without a session reaches while no session has a channel:
# the headless tests, and a process where no session installed its tools.
_keylessChannel = PreviewChannel(keyless=True)


# Who else is told what a drag is doing. The Avar Editor shows the number a
# manipulator is dragging WHILE it is dragged, and this is how it hears each
# sample: a listener is a callable, called with the same {Sdf.Path: value}
# the sink gets on every Push and with None on End. Callables rather than a
# Qt signal because this module is Qt-free.
_listeners = []


def AddListener(listener):
    """Call `listener(pending)` on every Push and `listener(None)` on End."""
    if listener is not None and listener not in _listeners:
        _listeners.append(listener)


def RemoveListener(listener):
    try:
        _listeners.remove(listener)
    except ValueError:
        pass


def _Notify(pending):
    # A listener that throws is dropped for that call only: a panel's bug
    # must never kill a drag.
    for listener in list(_listeners):
        try:
            listener(pending)
        except Exception:
            pass


def Channel(session=None, create=False):
    """
    `session`'s channel (created with `create`), or None.

    Without a session: the current session's channel (see
    sessionRegistry.SessionRegistry.Current), or the keyless one while no
    session has a channel at all.
    """
    if session is None:
        if len(_channels) == 0:
            return _keylessChannel
        return _channels.Current()
    channel = _channels.Get(session)
    if channel is None and create:
        channel = _channels.Set(session, PreviewChannel())
    return channel


def SetSink(sink, session=None):
    """Install `session`'s preview sink, or None to preview nothing."""
    channel = Channel(session, create=True)
    if channel is not None:
        channel.SetSink(sink)


def HasSink(session=None):
    channel = Channel(session)
    return channel is not None and channel.HasSink()


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


def Push(pending, session=None, stage=None):
    """
    One mouse sample: `pending` is {Sdf.Path: value} from the Writer.

    Tells `session`'s Hydra what the artist is doing; `stage` is the stage
    the values belong to, so End() can drop exactly them. The gizmo's own
    handles are already following it -- Writer.Set publishes each value
    into gizmoMath's preview map as it collects it -- so this call is what
    puts the same values in front of the renderer. Returns True when the
    sink took them.
    """
    channel = Channel(session)
    if channel is None:
        # No channel, so no sink -- but the drag is real and the listeners
        # are owed the sample.
        if pending:
            _Notify(pending)
        return False
    return channel.Push(pending, stage)


def End(session=None, stage=None, publish=True):
    """
    The manipulation is over: drop the preview on both sides.

    Idempotent, and safe to call after a push that failed or never happened,
    because an abort and a commit both arrive here and neither knows which
    samples the sink accepted. Drops the uncommitted values of the stages
    this session pushed for (and of `stage`), never another session's.

    `publish=False` is for a COMMIT about to author: the host drops its
    overrides without republishing, and the commit's own stage notice
    publishes the committed rig -- one evaluation instead of two, and no
    frame of the pre-drag pose in between.
    """
    channel = Channel(session)
    if channel is None:
        if stage is not None:
            gizmoMath.SetPreviewValues({}, stage=stage)
        _Notify(None)
        return False
    return channel.End(stage, publish)


def IsPreviewing(session=None):
    channel = Channel(session)
    return channel is not None and channel.IsPreviewing()


def Republish(session=None):
    """
    Ask the host to publish the authored rig again.

    For the one case End(publish=False) cannot cover: a commit that was
    expected to author, and whose stage notice would have published, but
    authored nothing. The host's end is idempotent -- with no preview left
    to drop it only republishes -- so the drag pose does not stay on screen.
    """
    channel = Channel(session)
    sink = getattr(channel, "_sink", None) if channel is not None else None
    if sink is None:
        return False
    return bool(sink.End())
