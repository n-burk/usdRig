#
# RigExec usdview plugins: per-session state.
#
# usdview gives every window its own AppController, UsdviewApi and plugin
# containers, but a Python module exists once per PROCESS. A host that runs
# several usdview sessions in one QApplication (usdOrchestrate's shared host)
# therefore shares every module global between its windows: the gizmo
# installed itself into the first window only, the second window's panels
# re-targeted the first window's, and one session's preview reached another
# session's evaluator.
#
# This module is where that state lives instead: one SessionRegistry per kind
# of thing, keyed by the SESSION -- the usdview main window
# (usdviewApi.qMainWindow). A headless api with no main window is its own
# session, so the Qt-free tests and the C++ tests' fake apis still work.
#
# Keys are held weakly where the key allows it, and an entry is dropped when
# its window's `destroyed` signal fires, so a closed session leaves nothing
# behind here. The handlers hold no strong reference to the key or to the
# value, so a registry never keeps a closed session alive through a Qt
# connection.
#
# There is deliberately no "current session" global. A caller with no api of
# its own asks Current(), which answers for the session that owns
# QApplication.activeWindow(), else for the ONLY session when there is exactly
# one (a plain usdview, which is the answer every such caller got before), and
# None when the question is ambiguous. Nothing ever wins by having been the
# last to register.
#
# Qt-free at import: Qt is reached lazily and only to ask for the active
# window, so a headless process imports this module unchanged.
#
import weakref


def SessionKey(session):
    """
    The key `session` is filed under.

    `session` is a UsdviewApi (its main window is the key; an api without
    one is its own key), or already a key -- a main window, or any object a
    headless caller uses as a session. None stays None.
    """
    if session is None:
        return None
    if hasattr(type(session), "qMainWindow") or hasattr(session,
                                                         "qMainWindow"):
        try:
            window = session.qMainWindow
        except Exception:
            window = None
        return window if window is not None else session
    return session


def ActiveWindow():
    """QApplication.activeWindow(), or None without Qt or an application."""
    try:
        from pxr.Usdviewq.qt import QtWidgets
    except Exception:
        return None
    try:
        if QtWidgets.QApplication.instance() is None:
            return None
        return QtWidgets.QApplication.activeWindow()
    except Exception:
        return None


def _WindowChain(widget):
    """
    `widget`'s top-level window, then the windows that own it.

    A panel is a top-level window of its own (Qt.Window) parented to
    usdview's main window, so its session is found by walking from its
    window to the parent's window, and so on.
    """
    seen = set()
    current = widget
    while current is not None and id(current) not in seen:
        seen.add(id(current))
        try:
            window = current.window()
        except Exception:
            window = current
        yield window
        try:
            current = window.parentWidget()
        except Exception:
            current = None


class _Entry(object):
    # `watch` is the slot connected to the key's `destroyed` signal for
    # this entry (None when the key has no such signal). The entry owns
    # that connection: whoever removes the entry while the key lives
    # disconnects it, so filing and removing a session repeatedly (a panel
    # closed and reopened, an install retried) never piles slots onto a
    # long-lived window.
    __slots__ = ("ref", "value", "watch", "__weakref__")

    def __init__(self, ref, value):
        self.ref = ref
        self.value = value
        self.watch = None

    def Key(self):
        return self.ref()


class _StrongRef(object):
    """The ref of a key that cannot be weakly referenced."""

    __slots__ = ("_key",)

    def __init__(self, key):
        self._key = key

    def __call__(self):
        return self._key


# Every registry in the process, weakly: what ForgetSession walks. A
# directory of registries, not anybody's state.
_registries = weakref.WeakSet()


class SessionRegistry(object):
    """
    One value per session, keyed weakly by the session's main window.

    `name` only labels the registry in reprs and warnings.
    """

    def __init__(self, name=""):
        self._name = name
        # id(key) -> _Entry. The entry's ref is checked on every lookup,
        # so a recycled id can never hand one session another's value.
        self._entries = {}
        _registries.add(self)

    def __repr__(self):
        return "<SessionRegistry %s: %d session(s)>" % (
            self._name or "?", len(self))

    # -- lookup ----------------------------------------------------------

    def _Find(self, key):
        entry = self._entries.get(id(key))
        if entry is None or entry.Key() is not key:
            return None
        return entry

    def Get(self, session, default=None):
        """The value filed for `session`, or `default`."""
        key = SessionKey(session)
        if key is None:
            return default
        entry = self._Find(key)
        return default if entry is None else entry.value

    def __contains__(self, session):
        key = SessionKey(session)
        return key is not None and self._Find(key) is not None

    def _Live(self):
        live = []
        for entry in list(self._entries.values()):
            key = entry.Key()
            if key is not None:
                live.append((key, entry))
        return live

    def Sessions(self):
        """The keys that currently have a value."""
        return [key for key, _entry in self._Live()]

    def Values(self):
        return [entry.value for _key, entry in self._Live()]

    def Items(self):
        return [(key, entry.value) for key, entry in self._Live()]

    def __len__(self):
        return len(self._Live())

    # -- mutation --------------------------------------------------------

    def Set(self, session, value):
        """File `value` for `session` (replacing any), and return it."""
        key = SessionKey(session)
        if key is None:
            raise ValueError("SessionRegistry %s: no session to file under"
                             % self._name)
        entry = self._Find(key)
        if entry is not None:
            entry.value = value
            return value
        token = id(key)
        selfRef = weakref.ref(self)
        holder = []

        def _Collected(_ref):
            registry = selfRef()
            if registry is not None and holder:
                registry._DropEntry(token, holder[0])

        try:
            ref = weakref.ref(key, _Collected)
        except TypeError:
            ref = _StrongRef(key)
        entry = _Entry(ref, value)
        holder.append(weakref.ref(entry))
        self._entries[token] = entry
        entry.watch = self._WatchDestroyed(key, token, weakref.ref(entry))
        return value

    def Pop(self, session, default=None):
        """Remove and return the value filed for `session`."""
        key = SessionKey(session)
        if key is None:
            return default
        entry = self._Find(key)
        if entry is None:
            return default
        del self._entries[id(key)]
        _Unwatch(key, entry)
        return entry.value

    def Clear(self):
        for key, entry in self._Live():
            _Unwatch(key, entry)
        self._entries.clear()

    def _DropEntry(self, token, entryRef):
        entry = entryRef() if entryRef is not None else None
        if entry is not None and self._entries.get(token) is entry:
            del self._entries[token]

    def _WatchDestroyed(self, key, token, entryRef):
        """Drop the entry when the window's C++ object is destroyed.

        The Python wrapper can outlive the window (anything still holding
        it keeps it), so weak keying alone would keep a closed session's
        value filed for as long as that reference lives.

        Returns the connected slot (for _Unwatch), or None.
        """
        signal = getattr(key, "destroyed", None)
        connect = getattr(signal, "connect", None)
        if connect is None:
            return None
        selfRef = weakref.ref(self)

        def _Destroyed(*_args):
            registry = selfRef()
            if registry is not None:
                registry._DropEntry(token, entryRef)

        try:
            connect(_Destroyed)
        except Exception:
            return None
        return _Destroyed

    # -- the session a caller without an api means -------------------------

    def SessionOf(self, widget):
        """The registered session that owns `widget`, or None."""
        if widget is None:
            return None
        keys = self.Sessions()
        if not keys:
            return None
        for window in _WindowChain(widget):
            for key in keys:
                if window is key:
                    return key
        return None

    def CurrentSession(self):
        """
        The session a caller that names none means: the one owning the
        active window, else the only one, else None (ambiguous).
        """
        keys = self.Sessions()
        if not keys:
            return None
        key = self.SessionOf(ActiveWindow())
        if key is not None:
            return key
        if len(keys) == 1:
            return keys[0]
        return None

    def Current(self, default=None):
        key = self.CurrentSession()
        if key is None:
            return default
        return self.Get(key, default)

    def PopCurrent(self, default=None):
        key = self.CurrentSession()
        if key is None:
            return default
        return self.Pop(key, default)


def _Unwatch(key, entry):
    """Disconnect `entry`'s destroyed slot from `key`, if it has one.

    Only for an entry removed while its key lives; an entry dropped BY the
    signal (or by the key's collection) needs nothing, its window is going.
    A window whose C++ object is already gone refuses the disconnect, which
    is equally fine.
    """
    slot = entry.watch
    entry.watch = None
    if slot is None:
        return
    try:
        getattr(key, "destroyed").disconnect(slot)
    except Exception:
        pass


def ForgetSession(session):
    """
    Drop `session` from every registry now; returns how many held it.

    A window's `destroyed` signal already does this. A host that ends a
    session without destroying its window (or before the deferred delete
    runs) calls this so nothing here keeps the session's controllers,
    panels or containers -- and through them its UsdviewApi -- alive.
    """
    key = SessionKey(session)
    if key is None:
        return 0
    dropped = 0
    for registry in list(_registries):
        if registry._Find(key) is not None:
            registry.Pop(key)
            dropped += 1
    return dropped


def SessionOfEvent(receiver, session):
    """
    Whether a key event delivered to `receiver` belongs to `session`.

    `session` is a key (a main window). A widget receiver belongs to the
    session whose window it is in, or whose window owns the panel it is in;
    any other receiver (a QWindow) is judged by the active window. This is
    the gate every application-wide event filter in these plugins applies
    before it does anything at all, so one window's keys never drive
    another session.
    """
    if session is None:
        return False
    target = receiver
    if not _IsWidget(receiver):
        target = ActiveWindow()
    if target is None:
        return False
    for window in _WindowChain(target):
        if window is session:
            return True
    return False


def _IsWidget(obj):
    try:
        from pxr.Usdviewq.qt import QtWidgets
    except Exception:
        return hasattr(obj, "window")
    return isinstance(obj, QtWidgets.QWidget)


def PerSessionInstanceMeta(baseClass):
    """
    A metaclass for `baseClass`'s subclasses whose `_instance` used to be a
    process-wide singleton.

    The class keeps its instances in a SessionRegistry it defines as
    `_sessions`; `Cls._instance` is then a view onto it, not a second copy:
    reading it answers the CURRENT session's instance (see
    SessionRegistry.Current -- in a plain usdview, the one instance there
    is), and assigning None forgets the current session's instance, which is
    what resetting the singleton meant. Library code asks `_sessions` with
    its own api; the view exists for callers that have none (scripts and
    the testusdview harness).
    """
    base = type(baseClass)

    class PerSessionInstance(base):

        @property
        def _instance(cls):
            return cls._sessions.Current()

        @_instance.setter
        def _instance(cls, value):
            if value is None:
                cls._sessions.PopCurrent()
                return
            api = getattr(value, "_api", None)
            if api is None:
                api = getattr(value, "usdviewApi", None)
            if api is None:
                raise TypeError("%s._instance needs an instance with a "
                                "usdview api" % cls.__name__)
            cls._sessions.Set(api, value)

    PerSessionInstance.__name__ = "PerSessionInstance_%s" % base.__name__
    return PerSessionInstance
