"""Hold N, say "left clavicle", and TouchPose selects it.

THE ADAPTER, and only the adapter. Everything about speech -- how a rig
name becomes words, what the recogniser is allowed to hear, what an
ambiguous phrase means, the helper process and its protocol -- lives in
`touchpose/voice`, a subpackage of the shared TouchPose repo beside this
checkout. That subpackage imports no `pxr`, no Qt and no DCC, and its
tests run under plain CPython with nothing installed; a test there walks
its AST to keep it that way. This file is the other half of that
bargain: the USD, the Qt and the usdview.

THE DEPENDENCY RUNS ONE WAY. TouchPose does not need voice. The
subpackage may be absent, the speech engine may be missing, the
microphone may be in use, the import may raise something nobody
predicted -- and TouchPose opens, picks and paints exactly as it always
did. `Available` answers with a REASON rather than an exception, the
panel disables its checkbox and puts the reason on it, and nothing is
printed unless something genuinely unexpected happened. When it IS
there it comes up with the panel: no second install step, no setting to
find.

LOADED FROM ITS PATH, NOT IMPORTED BY NAME. `import touchpose.voice` is
the right spelling everywhere except here: this very directory carries a
DIFFERENT package called `touchpose` (the `.touch` reader and the USD
exporter), and whichever of the two reached `sys.path` first would
shadow the other. So the subpackage is loaded straight off disk and
registered as `touchpose_voice`, which nothing else is called.
`TOUCHPOSE_VOICE_PATH` overrides where it is looked for.

THE VOCABULARY IS EVERY TOUCH LAYER, NOT THE LIVE ONE. `TouchModel` holds
one layer at a time, which is right for painting and wrong for speech: an
animator working on the body still wants to say "left eye". The entries
are read straight off the stage's `RigExecTouchRegion` prims across every
scope, and a control that is not on the live layer is selected through
`Pick`'s `extra` argument instead of its `regions`.

PUSH-TO-TALK IS AN APPLICATION FILTER. A `QAction` on the panel only
fires while the panel has focus, and an animator holding a key is looking
at the viewport. Modelled on `rigExecUsdview.gizmoUI.ViewportHotkeyFilter`,
including the ordering that releases the hold BEFORE the typing gate:
a key pressed over the viewport and released after the focus moved into a
text field must still end the utterance, or the recogniser listens for
ever.
"""
import os
import sys

from pxr import Tf
from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets

import touchPoseModel


# The key held to talk.
#
# NOT `N`, AND THE REASON IS WORTH KEEPING. `N` was chosen by checking
# usdview and the gizmo, was free in both, and was verified under
# `testusdview` -- which is exactly where the check could not fail,
# because the headless runners deliberately do not load extra panels.
# `bin/launch.sh` DOES register the Noodles editor, and Noodles binds
# plain `N` to "Show Noodles editor" (noodlesSettings.py,
# `shortcutShowEditor`). A QAction shortcut is delivered before an
# application-wide event filter ever sees the key, so in a real session
# the hold never arrived at all and voice looked simply broken, with
# nothing in the log: the quiet-degrade path had nothing to report
# because nothing had failed. Hence `_Conflict` below -- a key this
# plugin cannot actually receive is now SAID, not shrugged at.
#
# Backtick instead: the push-to-talk key by convention, reachable with
# the left hand while the right stays on the mouse, and claimed by
# nothing in usdview (Space, Shift+H, the arrows), the gizmo
# (Q W E R D X B L P J C V Insert + - =), the graph editor
# (A F I Home Delete Backspace) or Noodles (A D F G N Escape).
_DEFAULT_KEY = "Backtick"

# The free single letters, as of this writing, for anyone overriding:
# H K M O S T U. Anything Qt can name also works, `F9` included.
_KEY_NAMES = {
    "backtick": (QtCore.Qt.Key_QuoteLeft, "`"),
    "grave": (QtCore.Qt.Key_QuoteLeft, "`"),
    "`": (QtCore.Qt.Key_QuoteLeft, "`"),
}


def _ResolveKey(name):
    """A key name to (Qt key, label). Falls back to the default, loudly."""
    text = (name or "").strip()
    if not text:
        text = _DEFAULT_KEY
    found = _KEY_NAMES.get(text.lower())
    if found is not None:
        return found
    sequence = QtGui.QKeySequence(text)
    # An unparseable name gives an empty sequence, and a MULTI-key one
    # ("Ctrl+T") cannot be a hold; both fall back rather than leaving the
    # animator with a key that does nothing.
    if sequence.count() == 1:
        combination = sequence[0]
        key = getattr(combination, "key", lambda: int(combination))()
        try:
            key = QtCore.Qt.Key(int(key) & ~int(QtCore.Qt.KeyboardModifierMask))
        except (TypeError, ValueError):
            key = QtCore.Qt.Key(int(key))
        label = sequence.toString()
        if label and "+" not in label:
            return key, label
    if name:
        Tf.Warn("touchPose: TOUCHPOSE_VOICE_KEY=%r is not a single key; "
                "using %s" % (name, _DEFAULT_KEY))
    return _KEY_NAMES[_DEFAULT_KEY.lower()]


PUSH_TO_TALK_KEY, PUSH_TO_TALK_LABEL = _ResolveKey(
    os.environ.get("TOUCHPOSE_VOICE_KEY"))


def _Conflict(mainWindow):
    """Whatever already claims the push-to-talk key, or "".

    A QAction or QShortcut anywhere in the window takes the key before an
    application event filter is offered it, so such a key can never reach
    push-to-talk. Finding that at startup and SAYING it is the whole
    point: the alternative is what shipped, where `N` was silently eaten
    by the Noodles editor and voice appeared to be broken.
    """
    if mainWindow is None:
        return ""
    wanted = QtGui.QKeySequence(PUSH_TO_TALK_LABEL).toString().lower()
    if not wanted:
        return ""
    try:
        holders = list(mainWindow.findChildren(QtGui.QAction)) \
            if hasattr(QtGui, "QAction") else []
        holders += list(mainWindow.findChildren(QtWidgets.QShortcut)) \
            if hasattr(QtWidgets, "QShortcut") else []
        holders += list(mainWindow.actions())
        for holder in holders:
            sequences = (holder.shortcuts()
                         if hasattr(holder, "shortcuts") else
                         [holder.key()])
            for sequence in sequences:
                if sequence.toString().lower() == wanted:
                    name = ""
                    if hasattr(holder, "text"):
                        name = holder.text().replace("&", "").strip()
                    return name or holder.objectName() or "another command"
    except Exception as error:
        # Never let the conflict CHECK be the thing that breaks startup.
        Tf.Warn("touchPose: could not check the voice hotkey: %s" % error)
    return ""

_TEXT_WIDGETS = (QtWidgets.QLineEdit, QtWidgets.QAbstractSpinBox,
                 QtWidgets.QTextEdit, QtWidgets.QPlainTextEdit)

# touchpose_voice's selection modes -> touchPoseUI's. See `select` below
# for why this is a table and not a pass-through. `add` has no TouchPose
# mode of its own; a toggle over a set nothing in is an add.
_MODES = {"replace": "replace", "toggle": "toggle", "add": "toggle"}

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.abspath(os.path.join(_HERE, "..", ".."))
# The shared TouchPose repo as a sibling of this checkout -- the layout
# bin/_env.sh already assumes for usd-install and the venv.
_DEFAULT_CORE = os.path.abspath(
    os.path.join(_REPO, "..", "touchpose", "scripts", "touchpose", "voice"))

# The name the subpackage is registered under HERE. Not `touchpose.voice`,
# because `plugin/touchPose/touchpose` is a different package with the
# same name; see the module docstring.
_MODULE = "touchpose_voice"

_core = None
_reason = ""


def CorePath():
    """Where the voice subpackage is expected to live."""
    return os.environ.get("TOUCHPOSE_VOICE_PATH") or _DEFAULT_CORE


def Available():
    """The voice subpackage, or None WITH A REASON. Never raises.

    Asked every time the panel wants to know and answered from a cache,
    so a checkout with no TouchPose repo beside it costs one `isfile` and
    then nothing. Absence is a normal state, not an error: it is not
    logged, it is shown on the checkbox the animator is already looking
    at. A subpackage that is THERE and still will not load is the
    unexpected case, and that one is warned about, once.
    """
    global _core, _reason
    if _core is not None or _reason:
        return _core, _reason

    root = CorePath()
    init = os.path.join(root, "__init__.py")
    if not os.path.isfile(init):
        _reason = ("Voice-to-select is not installed. Expected the TouchPose "
                   "repo beside this one, at %s -- or set "
                   "TOUCHPOSE_VOICE_PATH." % root)
        return None, _reason

    try:
        import importlib.util
        module = sys.modules.get(_MODULE)
        if module is None:
            # `submodule_search_locations` is what makes the relative
            # imports inside the package (`from . import grammar`)
            # resolve; without it this loads one module, not a package.
            spec = importlib.util.spec_from_file_location(
                _MODULE, init, submodule_search_locations=[root])
            module = importlib.util.module_from_spec(spec)
            # Registered BEFORE it is executed, because the submodules it
            # imports look their own parent up by name.
            sys.modules[_MODULE] = module
            try:
                spec.loader.exec_module(module)
            except Exception:
                del sys.modules[_MODULE]
                raise
        _core = module
    except Exception as error:
        # Anything at all: a syntax error, a missing stdlib module, a
        # half-written file. It costs the feature, never the panel.
        _reason = "Voice-to-select could not load from %s: %s" % (root, error)
        Tf.Warn("touchPose: %s" % _reason)
        return None, _reason
    return _core, ""


def Backend():
    """The speech backend, imported only when voice is actually turned on.

    Separate from `Available` because it is the half that needs a
    subprocess and a speech engine: a machine with neither can still
    build the vocabulary, and the panel can still say why the rest of it
    is unavailable.
    """
    import importlib
    return importlib.import_module("%s.sapi" % _MODULE)


# ---------------------------------------------------------------------------
# The vocabulary: every touch region on the stage
# ---------------------------------------------------------------------------

def BuildEntries(stage, Entry):
    """`[Entry]` for every touch region on `stage`, across every layer.

    The handle is the CONTROL's prim path, which is what `Pick` selects
    and what the lead comparison in `Selected` returns -- so a handle
    round-trips without a lookup table. A region binding no control is
    dropped: a phrase that cannot select anything is worse than one the
    recogniser has never heard of, because it looks like it worked.
    """
    entries = []
    seen = set()
    for scope in touchPoseModel.FindRegionScopes(stage):
        group = touchPoseModel.LayerLabel(scope)
        for child in scope.GetChildren():
            if child.GetTypeName() != touchPoseModel.REGION_TYPE \
                    and not child.HasAttribute(touchPoseModel.FACES_ATTR):
                continue
            targets = touchPoseModel._TouchTargets(
                child, touchPoseModel.TYPED_CONTROL,
                touchPoseModel.CONTROL_REL)
            if not targets:
                continue
            control = str(targets[0])
            name = child.GetName()
            if name.endswith("_touch"):
                name = name[:-len("_touch")]
            # The same control reached from two layers is one thing to
            # say. Keeping both would make every such phrase ambiguous.
            if (control, name) in seen:
                continue
            seen.add((control, name))
            entries.append(Entry(control, name, group=group))
    return entries


# ---------------------------------------------------------------------------
# The host
# ---------------------------------------------------------------------------

class _MainThreadPump(QtCore.QObject):
    """Get a callable from the speech reader thread onto the GUI thread.

    A queued signal, which is the only mechanism Qt guarantees for this.
    The object is constructed on the GUI thread, so the slot runs there.
    """

    run = QtCore.Signal(object)

    def __init__(self, parent=None):
        super(_MainThreadPump, self).__init__(parent)
        self.run.connect(self._Run, QtCore.Qt.QueuedConnection)

    def _Run(self, fn):
        try:
            fn()
        except Exception as error:
            Tf.Warn("touchPose: voice callback failed: %s" % error)


class UsdviewVoiceHost(object):
    """`touchpose_voice.VoiceHost` over a `TouchPoseController`."""

    def __init__(self, controller, pump, notify=None):
        self._controller = controller
        self._pump = pump
        self._notify = notify
        self._Entry = None

    def SetEntryFactory(self, Entry):
        self._Entry = Entry

    # -- the protocol -----------------------------------------------------

    def vocabulary(self):
        stage = self._controller._Stage()
        if stage is None or self._Entry is None:
            return []
        return BuildEntries(stage, self._Entry)

    def selected(self):
        """The selection, LEAD LAST.

        usdview keeps its selection in the order it was made, so the last
        entry is the most recently added -- the same rule
        `TouchPoseController.SyncSelection` and the Avar Editor's
        `FocusPrim` use. `getFocusPrim` would return the FIRST, which is
        the opposite of what a bare phrase should follow.
        """
        selection = self._controller._Selection()
        if selection is None:
            return []
        return [str(p.GetPath()) for p in selection.getPrims()
                if p and p.IsValid() and not p.IsPseudoRoot()]

    def select(self, handles, mode="replace"):
        """Select the controls behind `handles`.

        Split, because the two halves do different things: a control
        that some LIVE touch set owns goes in `regions`, which is what
        lights the lead highlight on the skin, and everything else goes
        in `extra`, which selects it without pretending any set knows
        about it. One `Pick` either way, so the whole utterance is one
        selection change and one undo step.

        Every live set is searched, not just the active one: the eyes
        are their own sets, so looking only at the active one left
        "select left eye" selecting the control with no patch lit.
        """
        # The two vocabularies happen to agree, but they are different
        # vocabularies -- `touchpose_voice.host.REPLACE` and
        # `touchPoseUI.MODE_REPLACE` -- and this file cannot import
        # touchPoseUI (touchPoseUI imports THIS). Mapped rather than
        # passed through, so a rename on either side fails here and not
        # silently in `Pick`.
        mode = _MODES.get(str(mode), "replace")
        paths = [str(h) for h in handles]
        models = getattr(self._controller, "models", None)
        if not models:
            model = self._controller.model
            models = [model] if model is not None else []
        by_control = {}
        for model in models:
            for region in model.regions:
                if region.control and region.control not in by_control:
                    by_control[region.control] = region
        regions = [by_control[p] for p in paths if p in by_control]
        extra = [p for p in paths
                 if not any(r.control == p for r in regions)]
        return self._controller.Pick(regions, mode, extra=extra)

    def notify(self, decision):
        if self._notify is not None:
            self._notify(decision)

    def call_on_main_thread(self, fn):
        self._pump.run.emit(fn)


# ---------------------------------------------------------------------------
# Push to talk
# ---------------------------------------------------------------------------

class VoiceHotkeyFilter(QtCore.QObject):
    """The hold key, taken at the APPLICATION level.

    Qt runs application filters most-recently-installed first, and this
    plugin loads after usdview installed its own, so this sees N before
    anything else can claim it. Nothing is consumed unless it was acted
    on.
    """

    def __init__(self, voice):
        super(VoiceHotkeyFilter, self).__init__(voice)
        self._voice = voice

    def eventFilter(self, receiver, event):
        try:
            kind = event.type()
            if kind == QtCore.QEvent.WindowDeactivate:
                # Alt-Tab with the key down: usdview never sees the
                # release, and a recogniser left listening would pick up
                # whatever was said to the next application.
                self._voice.Release()
                return False
            if kind not in (QtCore.QEvent.KeyPress, QtCore.QEvent.KeyRelease):
                return False
            if event.key() != PUSH_TO_TALK_KEY:
                return False
            if event.isAutoRepeat():
                # Holding a key repeats it. The hold is one utterance.
                return self._voice.listening
            if kind == QtCore.QEvent.KeyRelease:
                # RELEASE FIRST, gate second: a key pressed over the
                # viewport and released after the focus moved into a text
                # field must still end the utterance. The claim is still
                # below the gate -- a release delivered to a line edit is
                # never ours to consume.
                handled = self._voice.Release()
                if _TypingFocus():
                    return False
                return handled
            if _TypingFocus():
                return False
            if event.modifiers() & (QtCore.Qt.ControlModifier
                                    | QtCore.Qt.AltModifier
                                    | QtCore.Qt.MetaModifier):
                return False
            if not _InMainWindow(self._voice.mainWindow, receiver):
                return False
            return self._voice.Press()
        except Exception as error:
            # An exception escaping an application-wide filter would break
            # every key in usdview, not just this one.
            Tf.Warn("touchPose: voice hotkey filter failed: %s" % error)
        return False


def _TypingFocus():
    focus = QtWidgets.QApplication.focusWidget()
    if focus is None:
        return False
    if isinstance(focus, _TEXT_WIDGETS):
        return True
    return (isinstance(focus, QtWidgets.QComboBox) and focus.isEditable())


def _InMainWindow(main, receiver):
    if main is None:
        return False
    if isinstance(receiver, QtWidgets.QWidget):
        return receiver.window() is main
    return QtWidgets.QApplication.activeWindow() is main


# ---------------------------------------------------------------------------
# The mode
# ---------------------------------------------------------------------------

class TouchPoseVoice(QtCore.QObject):
    """Voice-to-select, switched on and off as one thing.

    Owns the session, the helper process and the hotkey filter. Started
    ONCE when the checkbox goes on -- the engine, the compiled bridge and
    a 474-phrase grammar cost about half a second between them, and a
    push-to-talk that paid that per keypress would be unusable.
    """

    statusChanged = QtCore.Signal(str)

    def __init__(self, controller, usdviewApi, parent=None):
        super(TouchPoseVoice, self).__init__(parent)
        self._controller = controller
        self._api = usdviewApi
        self._pump = _MainThreadPump(self)
        self._host = UsdviewVoiceHost(controller, self._pump, self._Notify)
        self._session = None
        self._filter = None
        self._enabled = False
        self._listening = False

    # -- state ------------------------------------------------------------

    @property
    def enabled(self):
        return self._enabled

    @property
    def listening(self):
        return self._listening

    @property
    def mainWindow(self):
        try:
            return self._api.qMainWindow
        except Exception:
            return None

    @property
    def session(self):
        return self._session

    # -- on and off --------------------------------------------------------

    def SetEnabled(self, enabled):
        """Turn voice on or off. Returns what it actually is now."""
        enabled = bool(enabled)
        if enabled == self._enabled:
            return self._enabled
        if enabled:
            return self._Start()
        self._Stop()
        return False

    def _Start(self):
        core, reason = Available()
        if core is None:
            self.statusChanged.emit(reason)
            return False
        try:
            sapi = Backend()
        except Exception as error:
            self.statusChanged.emit("Voice unavailable: %s" % error)
            return False

        self._host.SetEntryFactory(core.Entry)
        entries = self._host.vocabulary()
        if not entries:
            self.statusChanged.emit(
                "Voice: this stage has no touch regions to name.")
            return False

        session = core.VoiceSession(
            self._host, backend_factory=lambda sink: sapi.SapiBackend(sink))
        try:
            session.start()
        except Exception as error:
            # Losing the microphone must cost the feature, not the panel.
            try:
                session.stop()
            except Exception:
                pass
            # NOT warned. A machine with no speech engine, or a
            # microphone another application already holds, is an
            # ordinary state and voice comes up by itself -- so a warning
            # here is a line in the terminal nobody asked for, once per
            # attempt. The panel's status line carries the reason, which
            # is where whoever wants it is looking.
            self.statusChanged.emit("Voice unavailable: %s" % error)
            return False

        self._session = session
        self._enabled = True
        self._Install()
        # Checked AFTER the filter is installed, so it reports the world
        # the hold will actually live in. A conflict does not stop voice
        # -- the recogniser is up and a different key will reach it -- it
        # is reported, because a hotkey that cannot arrive is the one
        # failure this feature has already shipped once in silence.
        taken = _Conflict(self.mainWindow)
        if taken:
            self.statusChanged.emit(
                "Voice on, but %s is already used by \"%s\" and will not "
                "reach it. Set TOUCHPOSE_VOICE_KEY to a free key "
                "(H K M O S T U) and relaunch."
                % (PUSH_TO_TALK_LABEL, taken))
            Tf.Warn("touchPose: the voice hotkey %s is claimed by \"%s\"; "
                    "push-to-talk will not receive it. Set "
                    "TOUCHPOSE_VOICE_KEY and relaunch."
                    % (PUSH_TO_TALK_LABEL, taken))
            return True
        self.statusChanged.emit(
            "Voice on. Hold %s and name a control (%d to choose from)."
            % (PUSH_TO_TALK_LABEL, len(entries)))
        return True

    def _Stop(self):
        self._Uninstall()
        self._listening = False
        session, self._session = self._session, None
        self._enabled = False
        if session is not None:
            try:
                session.stop()
            except Exception as error:
                Tf.Warn("touchPose: voice would not stop cleanly: %s" % error)
        self.statusChanged.emit("")

    def Refresh(self):
        """Rebuild the vocabulary. Call when the stage or its layers change."""
        if self._session is None:
            return False
        try:
            self._session.refresh()
        except Exception as error:
            self.statusChanged.emit("Voice: could not reload: %s" % error)
            return False
        return True

    # -- the key ------------------------------------------------------------

    def _Install(self):
        if self._filter is not None:
            return
        application = QtWidgets.QApplication.instance()
        if application is None:
            return
        self._filter = VoiceHotkeyFilter(self)
        application.installEventFilter(self._filter)

    def _Uninstall(self):
        if self._filter is None:
            return
        application = QtWidgets.QApplication.instance()
        if application is not None:
            application.removeEventFilter(self._filter)
        self._filter.setParent(None)
        self._filter = None

    def Press(self):
        if self._session is None or self._listening:
            return False
        try:
            started = self._session.press()
        except Exception as error:
            self.statusChanged.emit("Voice: %s" % error)
            return False
        if started:
            self._listening = True
            self.statusChanged.emit("Listening...")
        return started

    def Release(self):
        if self._session is None or not self._listening:
            return False
        self._listening = False
        try:
            self._session.release()
        except Exception as error:
            self.statusChanged.emit("Voice: %s" % error)
            return False
        return True

    # -- what the animator sees ---------------------------------------------

    def _Notify(self, decision):
        core, _reason_ = Available()
        if core is None:
            return
        if decision.kind == core.SELECT:
            self.statusChanged.emit(u"✓ %s" % decision.message)
        elif decision.kind == core.AMBIGUOUS:
            self.statusChanged.emit(decision.message)
        elif decision.kind == core.LOW_CONFIDENCE:
            self.statusChanged.emit(decision.message)
        else:
            self.statusChanged.emit(decision.message or "nothing heard")
