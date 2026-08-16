#
# museAssistant — Meta Muse assistant plugin for usdview
#
# A usdview PluginContainer hosting one chat window, opened with Cmd+Shift+M
# (Ctrl+Shift+M off macOS). The UI is deliberately a text field, a transcript
# and a single button that attaches the current viewport: everything else the
# assistant needs it reaches through its own tools rather than through chrome
# the user has to operate.
#
# Through those tools it can:
#   * read and modify USD data (Usd.Stage, Sdf, UsdGeom, …)
#   * query and drive the usdview API (selection, stage view, timeline)
#   * execute arbitrary Python with stage/api in scope and read the output
#   * capture the viewport and look at its own results
#
# Stage edits are always allowed; undo is the brake. There is no offline
# fallback — without a key the window says so and stays inert, because a
# fabricated answer that looks real is worse than a clear refusal. See
# museAgent for the back ends and their auth.
#
# Loaded as a Python PluginContainer via plugin/museAssistant/plugInfo.json.
# PXR_PLUGINPATH_NAME must contain the containing directory.
#
import atexit
import base64
import contextlib
import datetime
import io
import json
import os
import re
import sys
import textwrap
import threading
import traceback

from pxr import Gf, Sdf, Tf, Usd, UsdGeom, Vt

# Usdview's Qt shim is the only supported binding — it resolves to whichever
# of PySide2/PySide6/PyQt the USD build was compiled against.
try:
    from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets
    _HAS_QT = True
except Exception as _qtError:  # headless / import probe
    QtCore = QtGui = QtWidgets = None
    _HAS_QT = False
    _QT_IMPORT_ERROR = _qtError
else:
    _QT_IMPORT_ERROR = None

try:
    from pxr.Usdviewq.plugin import PluginContainer
except Exception:
    # Headless import probe where Usdviewq isn't on PYTHONPATH.
    class PluginContainer:  # type: ignore
        pass


# ---------------------------------------------------------------------------
# Non-Qt helpers — importable and testable without a display.
# ---------------------------------------------------------------------------

def stage_summary(stage):
    """One-line summary of a Usd.Stage for the LLM context window."""
    if not stage:
        return "No stage loaded."
    root = stage.GetRootLayer()
    session = stage.GetSessionLayer()
    edit = stage.GetEditTarget().GetLayer() if stage.GetEditTarget() else None
    lines = [
        "Stage: %s" % (stage.GetRootLayer().identifier if root else "<no root>"),
        "EditTarget: %s" % (edit.identifier if edit else "<none>"),
        "Session: %s" % (session.identifier if session else "<none>"),
        "DefaultPrim: %s" % (stage.GetDefaultPrim().GetPath() if stage.GetDefaultPrim() else "<none>"),
        "Prim count (traverse): %d" % sum(1 for _ in stage.Traverse()),
        "UpAxis: %s" % UsdGeom.GetStageUpAxis(stage),
    ]
    return "\n".join(lines)


def list_prims(stage, pattern=None, typeName=None, limit=200):
    if not stage:
        return []
    out = []
    for prim in stage.Traverse():
        if typeName and prim.GetTypeName() != typeName:
            continue
        path = str(prim.GetPath())
        if pattern and pattern not in path:
            continue
        out.append("%-48s  %-22s  %s" % (
            path, prim.GetTypeName() or "(untyped)",
            "active" if prim.IsActive() else "inactive"))
        if len(out) >= limit:
            out.append("... (%d more, truncated)" % limit)
            break
    return out


def get_prim_info(stage, primPath):
    if not stage:
        return "No stage."
    prim = stage.GetPrimAtPath(primPath)
    if not prim or not prim.IsValid():
        return "Prim not found: %s" % primPath
    lines = [
        "Prim: %s" % prim.GetPath(),
        "Type: %s" % (prim.GetTypeName() or "<untyped>"),
        "Active: %s  Instanceable: %s" % (prim.IsActive(), prim.IsInstance()),
        "Children: %s" % ", ".join(str(c.GetPath().name) for c in prim.GetChildren()) or "(none)",
        "Attributes:",
    ]
    for attr in prim.GetAttributes():
        try:
            val = attr.Get()
        except Exception:
            val = "<error>"
        lines.append("  %-30s %-18s = %r" % (attr.GetName(), str(attr.GetTypeName()), val))
    lines.append("Relationships:")
    for rel in prim.GetRelationships():
        try:
            tgts = rel.GetTargets()
        except Exception:
            tgts = []
        lines.append("  %-30s -> %s" % (rel.GetName(), tgts if tgts else "(none)"))
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Stored credentials — plain functions, testable without Qt.
# ---------------------------------------------------------------------------

# Kept out of the repo deliberately: a key in the checkout is one `git add -A`
# from being published. Written 0600 because it is a bearer credential.
CREDENTIALS_PATH = os.path.join(
    os.path.expanduser("~"), ".config", "muse", "credentials.json")


# The back-end settings the dialog persists alongside the key. Stored under
# their environment-variable names so there is exactly one vocabulary: what
# you would export in a shell is what appears in the file.
SAVED_SETTING_NAMES = ("MUSE_PROVIDER", "MUSE_OLLAMA_URL", "MUSE_MODEL")


def _read_settings_file(path=None):
    """The whole saved dict, or {} when there is nothing readable."""
    path = path or CREDENTIALS_PATH
    try:
        with open(path, "r", encoding="utf-8") as handle:
            data = json.load(handle)
    except (IOError, OSError, ValueError):
        return {}
    return data if isinstance(data, dict) else {}


def _write_settings_file(data, path=None):
    """Replace the saved dict, 0600. Returns the path written."""
    path = path or CREDENTIALS_PATH
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
        try:
            os.chmod(directory, 0o700)
        except OSError:
            pass
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(data, handle, indent=2, sort_keys=True)
    try:
        os.chmod(path, 0o600)
    except OSError:
        pass
    return path


def load_saved_api_key(path=None):
    """The key saved by the settings dialog, or None.

    Never overrides the environment: MUSE_API_KEY in the launching shell is the
    more explicit statement of intent, and a stale saved key silently winning
    over it would be maddening to debug.
    """
    key = _read_settings_file(path).get("MUSE_API_KEY") or None
    return key.strip() or None if isinstance(key, str) else None


def save_api_key(key, path=None):
    """Persist *key* for future sessions. Returns the path written.

    Merged rather than written whole: the back-end settings live in the same
    file, and saving a key used to be the way to lose them.
    """
    data = _read_settings_file(path)
    data["MUSE_API_KEY"] = key
    return _write_settings_file(data, path)


def load_saved_settings(path=None):
    """The saved back-end settings, as an env-shaped dict. Never the key."""
    data = _read_settings_file(path)
    saved = {}
    for name in SAVED_SETTING_NAMES:
        value = data.get(name)
        if isinstance(value, str) and value.strip():
            saved[name] = value.strip()
    return saved


def save_settings(values, path=None):
    """Merge *values* into the saved settings. An empty value clears its key.

    Merging matters: provider, Ollama address and model are edited
    independently, and the API key sits in the same file.
    """
    data = _read_settings_file(path)
    for name in SAVED_SETTING_NAMES:
        if name not in values:
            continue
        value = (values.get(name) or "").strip()
        if value:
            data[name] = value
        else:
            data.pop(name, None)
    return _write_settings_file(data, path)


def apply_saved_settings(path=None):
    """Seed the environment with the saved back-end settings.

    Called once as the plugin loads. The shell still wins, for the same reason
    it wins for the key: an export is the more explicit statement of intent,
    and a stale saved setting quietly overriding it is the kind of thing you
    debug for an hour.
    """
    applied = {}
    for name, value in load_saved_settings(path).items():
        if os.environ.get(name, "").strip():
            continue
        os.environ[name] = value
        applied[name] = value
    return applied


def forget_api_key(path=None):
    """Remove the saved key. True if there was one to remove.

    Only the key: the back-end settings share this file, and forgetting a
    credential is not a request to also forget which server to talk to.
    """
    path = path or CREDENTIALS_PATH
    data = _read_settings_file(path)
    if "MUSE_API_KEY" not in data:
        # Nothing to forget. Remove a file that exists but holds no key only
        # when it holds nothing else either, so an empty husk does not linger.
        if not data and os.path.exists(path):
            try:
                os.remove(path)
            except OSError:
                pass
        return False
    data.pop("MUSE_API_KEY", None)
    if data:
        _write_settings_file(data, path)
    else:
        try:
            os.remove(path)
        except OSError:
            return False
    return True


def apply_saved_api_key(path=None):
    """Put the saved key into the environment when the shell supplied none.

    Called once as the plugin loads, so a key set through the settings dialog
    survives a restart without being exported by hand.
    """
    for name in ("MUSE_API_KEY", "ANTHROPIC_API_KEY"):
        if os.environ.get(name, "").strip():
            return None
    key = load_saved_api_key(path)
    if key:
        os.environ["MUSE_API_KEY"] = key
    return key


def exec_python(code, stage=None, api=None, extra_globals=None):
    """
    Execute *code* with `stage` and `api` in scope.  Returns
    (success: bool, stdout: str, stderr: str, result: object).

    The execution namespace is deliberately permissive — the assistant is
    trusted to modify data and UI "as it sees fit".  All of Usd/Sdf/Gf/Vt/
    UsdGeom plus `stage`/`api`/`app` are pre-bound.
    """
    stdout_buf = io.StringIO()
    stderr_buf = io.StringIO()

    # Build the execution globals — import everything the assistant commonly
    # needs so generated code can be terse.
    g = {
        "__name__": "__muse_exec__",
        "__builtins__": __builtins__,
        "stage": stage,
        "api": api,
        "app": QtWidgets.QApplication.instance() if _HAS_QT and QtWidgets else None,
        "Sdf": Sdf,
        "Usd": Usd,
        "UsdGeom": UsdGeom,
        "Gf": Gf,
        "Vt": Vt,
        "Tf": Tf,
        "stage_summary": lambda: stage_summary(stage),
        "list_prims": lambda pattern=None, typeName=None, limit=200: list_prims(stage, pattern, typeName, limit),
        "get_prim_info": lambda p: get_prim_info(stage, p),
    }
    # Expose Qt if available so code can construct widgets without importing.
    if _HAS_QT:
        g.update({"QtCore": QtCore, "QtGui": QtGui, "QtWidgets": QtWidgets})
    if extra_globals:
        g.update(extra_globals)

    # Capture both stdout and stderr.
    success = True
    result = None
    try:
        with contextlib.redirect_stdout(stdout_buf), contextlib.redirect_stderr(stderr_buf):
            # Try eval for single-expression convenience, fall back to exec.
            try:
                compiled = compile(code, "<muse>", "eval")
                result = eval(compiled, g)
                if result is not None:
                    print(repr(result))
            except SyntaxError:
                compiled = compile(code, "<muse>", "exec")
                exec(compiled, g)
                result = g.get("_", None)
    except SystemExit:
        raise
    except Exception:
        success = False
        traceback.print_exc(file=stderr_buf)

    return success, stdout_buf.getvalue(), stderr_buf.getvalue(), result


# ---------------------------------------------------------------------------
# Assistant plumbing.
#
# The assistant runs a real tool loop (see museAgent): it inspects the stage,
# executes Python in this process, captures the viewport, reads each result and
# keeps going.  The LLM call itself happens on a worker thread; every tool call
# is marshalled back onto usdview's main thread, because USD and Qt are both
# main-thread-only.
# ---------------------------------------------------------------------------

import museAgent


def _escape_html(text):
    return (str(text).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;"))


def _strip_code_blocks(text):
    """Extract python code from markdown fences; if none, return text as-is."""
    # ```python ... ``` or ``` ... ```
    fences = re.findall(r"```(?:python)?\s*\n(.*?)```", text, flags=re.DOTALL)
    if fences:
        return "\n\n".join(fences).strip()
    return text.strip()


def build_stage_context(api, stage=None):
    """The stage facts worth spending system-prompt tokens on."""
    if stage is None:
        stage = getattr(api, "stage", None)
    lines = [stage_summary(stage)]
    try:
        selected = getattr(api, "selectedPrims", None) or []
        if selected:
            lines.append("Selected: %s" % ", ".join(str(p.GetPath()) for p in selected[:12]))
    except Exception:
        pass
    try:
        settings = getattr(getattr(api, "dataModel", None), "viewSettings", None)
        if settings is not None:
            lines.append("ViewSettings: displayGuide=%s showBBoxes=%s complexity=%s" % (
                getattr(settings, "displayGuide", "?"),
                getattr(settings, "showBBoxes", "?"),
                getattr(settings, "complexity", "?")))
    except Exception:
        pass
    return "\n".join(lines)


def capture_viewport_png(api, annotation="", embed_metadata=True):
    """
    Grab the viewport as PNG bytes with the camera that produced it written
    into the file's text chunks.  A screenshot saved this way still carries its
    camera when the user sends it back later.

    Returns (png_bytes or None, camera_info dict).
    """
    camera = _get_camera_info(api)
    pixmap = _capture_viewport_pixmap(api)
    if pixmap is None:
        return None, camera
    b64 = _pixmap_to_base64(pixmap)
    if not b64:
        return None, camera
    png = base64.b64decode(b64)
    if embed_metadata:
        png = museAgent.png_write_text(png, {
            museAgent.CAMERA_METADATA_KEY: json.dumps(camera, default=str),
            museAgent.SCENE_METADATA_KEY: _get_scene_description(api),
            museAgent.ANNOTATION_METADATA_KEY: annotation,
        })
    return png, camera

# ---------------------------------------------------------------------------
# Goal helpers (headless-testable, no Qt required)
# ---------------------------------------------------------------------------

def parse_goal_command(text):
    """Parse a /goal command. Returns (is_goal, goal_text_or_none, is_clear, is_show, help_text)."""
    stripped = text.strip()
    if not stripped.lower().startswith("/goal"):
        return (False, None, False, False, None)
    # Bare /goal -> show help/current
    parts = stripped.split(None, 1)
    if len(parts) == 1:
        return (True, None, False, True, None)
    arg = parts[1].strip()
    low = arg.lower()
    if low in ("clear", "reset", "none", "-"):
        return (True, None, True, False, None)
    if low in ("show", "status", "get"):
        return (True, None, False, True, None)
    if low in ("help", "?", "usage"):
        help_text = textwrap.dedent("""            /goal — long-horizon goal for Muse
            Usage:
              /goal <description>  — set goal (e.g. /goal rig the arm to match the sketch)
              /goal clear          — clear the goal
              /goal show           — show current goal
            The goal is prepended to every LLM system prompt and shown as a banner.
            Muse will keep subsequent turns aligned to it until cleared.""")
        return (True, None, False, False, help_text)
    return (True, arg, False, False, None)


def build_goal_context(goal):
    if not goal:
        return ""
    return "LONG-HORIZON GOAL (set via /goal, keep on task): %s" % goal


# ---------------------------------------------------------------------------
# Drawover helpers (headless stubs + Qt-aware capture)
# ---------------------------------------------------------------------------

def _get_camera_info(api):
    """Collect pertinent camera info from the usdview stageView. Returns a dict."""
    info = {}
    try:
        stage = getattr(api, "stage", None)
        info["stage_identifier"] = stage.GetRootLayer().identifier if stage and stage.GetRootLayer() else "none"
    except Exception:
        info["stage_identifier"] = "unknown"
    # Try to reach stageView via private api accessor (as curvenetUI does)
    view = None
    try:
        view = api._UsdviewApi__appController._stageView  # type: ignore
    except Exception:
        try:
            view = getattr(api, "_stageView", None) or getattr(api, "stageView", None)
        except Exception:
            pass
    if view is None:
        info["note"] = "stageView not reachable (headless or no viewport)"
        return info
    try:
        info["viewport_size"] = list(getattr(api, "viewportSize", (0,0)) or (0,0))
    except Exception:
        pass
    try:
        vp = view.computeWindowViewport() if hasattr(view, "computeWindowViewport") else None
        if vp:
            info["window_viewport"] = list(vp)
    except Exception:
        pass
    try:
        ratio = view.devicePixelRatioF() if hasattr(view, "devicePixelRatioF") else 1.0
        info["devicePixelRatio"] = float(ratio)
    except Exception:
        pass
    # Camera frustum / matrices
    try:
        camera, camPath = (None, None)
        if hasattr(view, "resolveCamera"):
            camera, camPath = view.resolveCamera()
        info["camera_prim_path"] = str(camPath) if camPath else "freeCamera"
        if camera is not None and hasattr(camera, "frustum"):
            fr = camera.frustum
            # Eye, center, up
            try:
                # frustum position/orientation
                # Compute view/proj matrices
                vm = fr.ComputeViewMatrix()
                pm = fr.ComputeProjectionMatrix()
                info["viewMatrix"] = [list(row) for row in vm] if hasattr(vm, "__iter__") else str(vm)
                info["projectionMatrix"] = [list(row) for row in pm] if hasattr(pm, "__iter__") else str(pm)
            except Exception as e:
                info["matrix_error"] = str(e)
            try:
                info["position"] = list(fr.position) if hasattr(fr, "position") else None
                info["viewDir"] = list(fr.viewDir) if hasattr(fr, "viewDir") else None
                info["upDir"] = list(fr.upDir) if hasattr(fr, "upDir") else None
            except Exception:
                pass
            try:
                info["near"] = float(fr.near) if hasattr(fr, "near") else None
                info["far"] = float(fr.far) if hasattr(fr, "far") else None
                info["fov"] = float(fr.fov) if hasattr(fr, "fov") else None
                info["aspect"] = float(fr.aspectRatio) if hasattr(fr, "aspectRatio") else None
            except Exception:
                pass
    except Exception as e:
        info["camera_error"] = str(e)
    # Timeline
    try:
        dm = getattr(api, "dataModel", None)
        if dm is not None:
            cf = getattr(dm, "currentFrame", None)
            info["currentFrame"] = str(cf) if cf is not None else None
            info["currentTime"] = float(cf.GetValue()) if hasattr(cf, "GetValue") else None
    except Exception:
        pass
    return info


def _get_scene_description(api, stage=None, limit=80):
    """Build a compact scene description for drawover context."""
    if stage is None:
        try:
            stage = getattr(api, "stage", None)
        except Exception:
            stage = None
    parts = []
    parts.append(stage_summary(stage))
    try:
        sel = getattr(api, "selectedPrims", []) or []
        if sel:
            parts.append("\nSelected (%d): %s" % (len(sel), ", ".join(str(p.GetPath()) for p in sel[:12])))
        else:
            parts.append("\nSelected: (none)")
    except Exception:
        pass
    # Add prim listing (truncated)
    try:
        prims = list_prims(stage, limit=limit)
        parts.append("\nPrims (first %d):\n" % limit + "\n".join(prims[:limit]))
    except Exception as e:
        parts.append("\nlist_prims error: %s" % e)
    # Add visible / render stats if available
    try:
        dm = getattr(api, "dataModel", None)
        if dm is not None and hasattr(dm, "viewSettings"):
            vs = dm.viewSettings
            parts.append("\nViewSettings: showGuides=%s showBBoxes=%s complexity=%s" % (
                getattr(vs, "showGuides", "?"), getattr(vs, "showBBoxes", "?"), getattr(vs, "complexity", "?")))
    except Exception:
        pass
    return "\n".join(parts)


def _qimage_to_base64(qimage, fmt="PNG"):
    """QImage -> base64 PNG string (without data: prefix). Returns str or None."""
    if qimage is None or qimage.isNull():
        return None
    try:
        import base64 as _b64
        from pxr.Usdviewq.qt import QtCore as _QtCore
        ba = _QtCore.QByteArray()
        buf = _QtCore.QBuffer(ba)
        buf.open(_QtCore.QIODevice.WriteOnly)
        qimage.save(buf, fmt)
        return _b64.b64encode(bytes(ba)).decode("ascii")
    except Exception:
        return None


def _pixmap_to_base64(pixmap, fmt="PNG"):
    if pixmap is None or pixmap.isNull():
        return None
    try:
        return _qimage_to_base64(pixmap.toImage(), fmt=fmt)
    except Exception:
        return None


def _capture_viewport_pixmap(api):
    """Grab the viewport framebuffer as QPixmap. Returns QPixmap or None."""
    if not _HAS_QT:
        return None
    view = None
    try:
        view = api._UsdviewApi__appController._stageView  # type: ignore
    except Exception:
        return None
    if view is None:
        return None
    try:
        # Prefer grabFrameBuffer (QOpenGLWidget path), fallback to grab()
        if hasattr(view, "grabFrameBuffer"):
            qimg = view.grabFrameBuffer()
            if qimg is not None and not qimg.isNull():
                return QtGui.QPixmap.fromImage(qimg)
        if hasattr(view, "grab"):
            pm = view.grab()
            if pm is not None and not pm.isNull():
                return pm
    except Exception:
        pass
    return None


def _EnsureGuideVisibleForRig(api):
    """
    RigExec controls/joints are purpose=guide.  Storm only draws them when
    viewSettings.displayGuide is True.  If any RigExecRoot exists on the stage
    and guide is off, turn it on.  Headless-safe.
    """
    try:
        stage = getattr(api, "stage", None)
        if stage is None:
            try:
                stage = getattr(getattr(api, "dataModel", None), "stage", None)
            except Exception:
                stage = None
        if stage is not None:
            has_rig = False
            for prim in stage.Traverse():
                if prim.GetTypeName() == "RigExecRoot":
                    has_rig = True
                    break
            if has_rig:
                vs = getattr(getattr(api, "dataModel", None), "viewSettings", None)
                if vs is not None and hasattr(vs, "displayGuide"):
                    if not vs.displayGuide:
                        vs.displayGuide = True
                        return True
        return False
    except Exception:
        return False




# ---------------------------------------------------------------------------
# Drawover overlay — transparent paint layer over the viewport
# ---------------------------------------------------------------------------

if _HAS_QT:

    class _MainThreadCall(object):
        __slots__ = ("fn", "result", "error", "done", "wait")

        def __init__(self, fn, wait=True):
            self.fn = fn
            self.result = None
            self.error = None
            self.wait = wait
            self.done = threading.Event()

    class _MainThreadEvent(QtCore.QEvent):
        TYPE = QtCore.QEvent.Type(QtCore.QEvent.registerEventType())

        def __init__(self, call):
            super(_MainThreadEvent, self).__init__(_MainThreadEvent.TYPE)
            self.call = call

    class MainThreadInvoker(QtCore.QObject):
        """Runs callables on the thread that owns this object."""

        def event(self, event):
            if event.type() == _MainThreadEvent.TYPE:
                call = event.call
                try:
                    call.result = call.fn()
                except Exception as exc:
                    call.error = exc
                finally:
                    call.done.set()
                return True
            return super(MainThreadInvoker, self).event(event)

        def invoke(self, fn, timeout=300.0):
            """Run *fn* on the main thread and return its result."""
            if QtCore.QThread.currentThread() is self.thread():
                return fn()
            call = _MainThreadCall(fn)
            QtCore.QCoreApplication.postEvent(self, _MainThreadEvent(call))
            if not call.done.wait(timeout):
                raise RuntimeError(
                    "timed out after %gs waiting for usdview's main thread" % timeout)
            if call.error is not None:
                raise call.error
            return call.result

        def post(self, fn):
            """Run *fn* on the main thread without waiting for it."""
            if QtCore.QThread.currentThread() is self.thread():
                fn()
                return
            QtCore.QCoreApplication.postEvent(
                self, _MainThreadEvent(_MainThreadCall(fn, wait=False)))

    class UsdviewExecutor(object):
        """
        The assistant's hands inside the running session.  Every method here
        is called from the agent thread and executes on the main thread.
        """

        def __init__(self, panel):
            self._panel = panel

        @property
        def _api(self):
            return self._panel._api

        def run_python(self, code):
            return self._panel._invoker.invoke(lambda: self._run_python(code))

        def _run_python(self, code):
            # The read-only gate went with the panel that held its checkbox: the
            # chat window is deliberately one input and one button, and edits
            # are always allowed. Undo is the brake now.
            api = self._api
            success, stdout, stderr, result = exec_python(
                code, stage=getattr(api, "stage", None), api=api)
            parts = []
            if stdout:
                parts.append("stdout:\n%s" % stdout.rstrip())
            if stderr:
                parts.append("stderr:\n%s" % stderr.rstrip())
            if result is not None and not stdout:
                parts.append("result: %r" % (result,))
            if not parts:
                parts.append("(executed, no output)")
            # A stage edit may have changed the edit target or added a rig.
            try:
                self._panel._refresh_stage_label()
                _EnsureGuideVisibleForRig(api)
            except Exception:
                pass
            return {
                "text": "\n\n".join(parts),
                "summary": "failed" if not success else (stdout.strip().splitlines() or ["ok"])[-1][:100],
                "failed": not success,
            }

        def inspect_stage(self, mode, pattern=None, type_name=None, limit=None, prim_path=None):
            return self._panel._invoker.invoke(
                lambda: self._inspect_stage(mode, pattern, type_name, limit, prim_path))

        def _inspect_stage(self, mode, pattern, type_name, limit, prim_path):
            api = self._api
            stage = getattr(api, "stage", None)
            if stage is None:
                return {"text": "No stage is loaded in this usdview session.",
                        "summary": "no stage"}
            if mode == "prim":
                if not prim_path:
                    return {"text": "prim mode needs prim_path.", "summary": "bad args"}
                return {"text": get_prim_info(stage, prim_path),
                        "summary": prim_path}
            if mode == "list":
                rows = list_prims(stage, pattern=pattern, typeName=type_name,
                                  limit=int(limit or 200))
                body = "\n".join(rows) if rows else "(no prims matched)"
                return {"text": body, "summary": "%d prim(s)" % len(rows)}
            return {"text": build_stage_context(api, stage), "summary": "stage summary"}

        def capture_viewport(self, note=""):
            return self._panel._invoker.invoke(lambda: self._capture_viewport(note))

        def _capture_viewport(self, note):
            api = self._api
            png, camera = capture_viewport_png(api, annotation=note, embed_metadata=False)
            if png is None:
                return {
                    "text": ("The viewport could not be captured (no GL widget "
                             "reachable). Camera state:\n%s"
                             % json.dumps(camera, indent=2, default=str)),
                    "image_b64": None, "summary": "capture failed", "failed": True,
                }
            b64 = base64.b64encode(png).decode("ascii")
            self._panel._log_capture(b64, note)
            return {
                "text": "Viewport capture%s.\nCamera:\n%s" % (
                    " — %s" % note if note else "",
                    json.dumps(camera, indent=2, default=str)),
                "image_b64": b64,
                "summary": note or "captured",
                "failed": False,
            }

    class _AgentThread(threading.Thread):
        """Runs one assistant turn off the UI thread."""

        def __init__(self, panel, messages, system_prompt):
            super(_AgentThread, self).__init__(daemon=True, name="museAgent")
            self._panel = panel
            self._messages = messages
            self._system_prompt = system_prompt
            # Not `_stop`: threading.Thread._stop is an internal method, and
            # shadowing it makes is_alive() raise.
            self._stop_flag = threading.Event()

        def stop(self):
            self._stop_flag.set()

        def run(self):
            panel = self._panel
            try:
                conversation = museAgent.run_agent(
                    messages=self._messages,
                    executor=UsdviewExecutor(panel),
                    system_prompt=self._system_prompt,
                    on_event=lambda kind, payload: panel._invoker.post(
                        lambda k=kind, p=payload: panel._on_agent_event(k, p)),
                    should_stop=self._stop_flag.is_set,
                )
                panel._invoker.post(lambda: panel._on_agent_finished(conversation, None))
            except museAgent.AgentError as error:
                message = str(error)
                panel._invoker.post(lambda: panel._on_agent_finished(None, message))
            except Exception:
                message = traceback.format_exc()
                panel._invoker.post(lambda: panel._on_agent_finished(None, message))


    def _main_window(api):
        """usdview's main window, so the popup centres on it and stays on top."""
        try:
            return api.qMainWindow
        except Exception:
            return None


    class _EscapeFilter(QtCore.QObject):
        """Claims Escape for the chat window, ahead of usdview's own filter.

        usdview installs an application-wide AppEventFilter that swallows every
        Escape to reset focus from the mouse position and returns True before
        any widget sees the key (Usdviewq/appEventFilter.py). A widget-level
        eventFilter or keyPressEvent therefore never runs, which is why Escape
        appeared dead while Return worked — Return takes that filter's
        JealousFocus branch and passes through.

        Qt activates application filters most-recently-installed first, and
        this plugin loads after usdview has installed its own, so installing
        here is what puts us in front. Escape is claimed only while the chat
        window holds focus; every other Escape falls through untouched and
        usdview behaves exactly as before.
        """

        def __init__(self, popup):
            super(_EscapeFilter, self).__init__(popup)
            self._popup = popup

        def eventFilter(self, obj, event):
            try:
                if event.type() != QtCore.QEvent.Type.KeyPress:
                    return False
                if event.key() != QtCore.Qt.Key.Key_Escape:
                    return False
                popup = self._popup
                if popup is None or not popup.isVisible():
                    return False
                focus = QtWidgets.QApplication.focusWidget()
                if focus is None:
                    return False
                if focus is not popup and not popup.isAncestorOf(focus):
                    return False
                popup._on_escape()
                return True
            except Exception:
                traceback.print_exc()
                return False


    class MuseSettingsDialog(QtWidgets.QDialog):
        """Muse ▸ Settings… — set the API key and see where it will be sent.

        The endpoint, auth header and model are shown read-only because they
        are derived from the key rather than chosen: seeing them here is what
        turns "401 Unauthorized" from a mystery into a fact about the key.
        """

        def __init__(self, api, parent=None):
            super(MuseSettingsDialog, self).__init__(parent)
            self._api = api
            self.setWindowTitle("Muse — Settings")
            self.setMinimumWidth(460)
            self._ollama_models = []
            self._build_ui()
            self._on_provider_changed()

        def _build_ui(self):
            layout = QtWidgets.QVBoxLayout(self)
            layout.setSpacing(10)

            form = QtWidgets.QFormLayout()

            # Which back end. Explicit rather than inferred, because an Ollama
            # server has no recognisable address to infer from.
            self._provider = QtWidgets.QComboBox()
            for label, value in (
                    ("Anthropic / Meta Muse (from the key)",
                     museAgent.PROVIDER_ANTHROPIC),
                    ("Ollama (local server)", museAgent.PROVIDER_OLLAMA)):
                self._provider.addItem(label, value)
            saved_provider = (os.environ.get("MUSE_PROVIDER", "").strip().lower()
                              or museAgent.resolve_provider())
            index = self._provider.findData(
                museAgent.PROVIDER_OLLAMA
                if saved_provider == museAgent.PROVIDER_OLLAMA
                else museAgent.PROVIDER_ANTHROPIC)
            self._provider.setCurrentIndex(max(index, 0))
            self._provider.currentIndexChanged.connect(self._on_provider_changed)
            form.addRow("Back end", self._provider)

            self._key_edit = QtWidgets.QLineEdit()
            self._key_edit.setEchoMode(QtWidgets.QLineEdit.EchoMode.Password)
            self._key_edit.setPlaceholderText("LLM_… (Meta Muse) or sk-ant-… (Anthropic)")
            self._key_edit.textChanged.connect(self._refresh_routing)
            self._key_row_label = QtWidgets.QLabel("API key")
            form.addRow(self._key_row_label, self._key_edit)

            self._ollama_url = QtWidgets.QLineEdit()
            self._ollama_url.setText(museAgent.resolve_ollama_base_url())
            self._ollama_url.setPlaceholderText(
                museAgent.OLLAMA_DEFAULT_BASE_URL)
            self._ollama_url.editingFinished.connect(self._reload_models)
            self._ollama_url_label = QtWidgets.QLabel("Ollama server")
            form.addRow(self._ollama_url_label, self._ollama_url)

            modelRow = QtWidgets.QHBoxLayout()
            self._model = QtWidgets.QComboBox()
            self._model.setSizePolicy(
                QtWidgets.QSizePolicy.Policy.Expanding,
                QtWidgets.QSizePolicy.Policy.Fixed)
            self._model.currentIndexChanged.connect(self._refresh_routing)
            modelRow.addWidget(self._model, 1)
            self._reload_btn = QtWidgets.QPushButton("Refresh")
            self._reload_btn.clicked.connect(self._reload_models)
            modelRow.addWidget(self._reload_btn)
            self._model_row = QtWidgets.QWidget()
            self._model_row.setLayout(modelRow)
            modelRow.setContentsMargins(0, 0, 0, 0)
            self._model_label = QtWidgets.QLabel("Model")
            form.addRow(self._model_label, self._model_row)

            self._current = QtWidgets.QLabel()
            self._current.setStyleSheet("color:#858585; font-size:11px")
            form.addRow("", self._current)
            layout.addLayout(form)

            self._remember = QtWidgets.QCheckBox(
                "Remember on this machine (%s, readable only by you)"
                % CREDENTIALS_PATH.replace(os.path.expanduser("~"), "~"))
            # Checked when anything is already saved -- the box now governs
            # the back-end settings as well as the key.
            self._remember.setChecked(
                load_saved_api_key() is not None or bool(load_saved_settings()))
            layout.addWidget(self._remember)

            self._routing = QtWidgets.QLabel()
            self._routing.setTextFormat(QtCore.Qt.TextFormat.RichText)
            self._routing.setStyleSheet(
                "background:#252526; border:1px solid #3c3c3c;"
                " border-radius:6px; padding:8px; font-size:11px")
            layout.addWidget(self._routing)

            buttons = QtWidgets.QDialogButtonBox(
                QtWidgets.QDialogButtonBox.StandardButton.Save
                | QtWidgets.QDialogButtonBox.StandardButton.Cancel)
            self._forget_btn = buttons.addButton(
                "Forget saved key",
                QtWidgets.QDialogButtonBox.ButtonRole.DestructiveRole)
            self._forget_btn.clicked.connect(self._on_forget)
            buttons.accepted.connect(self._on_save)
            buttons.rejected.connect(self.reject)
            layout.addWidget(buttons)

            self._describe_current()

        def _selected_provider(self):
            return self._provider.currentData() or museAgent.PROVIDER_ANTHROPIC

        def _on_provider_changed(self):
            """Show only the fields the chosen back end actually uses."""
            isOllama = self._selected_provider() == museAgent.PROVIDER_OLLAMA
            for widget in (self._key_edit, self._key_row_label):
                widget.setVisible(not isOllama)
            for widget in (self._ollama_url, self._ollama_url_label,
                           self._model_row, self._model_label):
                widget.setVisible(isOllama)
            if isOllama and not self._ollama_models:
                self._reload_models()
            else:
                self._refresh_routing()
            self._describe_current()

        def _reload_models(self):
            """Ask the server what it has, and say so when it has nothing.

            Tool support is shown per model rather than filtered out, because
            "the model I wanted is missing" and "the model I wanted cannot
            call tools" are different problems with different fixes.
            """
            url = self._ollama_url.text().strip() or \
                museAgent.OLLAMA_DEFAULT_BASE_URL
            self._reload_btn.setEnabled(False)
            try:
                # Refresh is the explicit "go and look again", so it bypasses
                # the per-session cache the send path relies on.
                museAgent.clear_ollama_model_cache()
                self._ollama_models = museAgent.fetch_ollama_models(url)
            finally:
                self._reload_btn.setEnabled(True)

            wanted = (self._model.currentData()
                      or os.environ.get("MUSE_MODEL", "").strip()
                      or museAgent.OLLAMA_DEFAULT_MODEL)
            self._model.blockSignals(True)
            self._model.clear()
            for entry in self._ollama_models:
                marks = []
                if entry["tools"]:
                    marks.append("tools")
                if entry["vision"]:
                    marks.append("vision")
                label = "%s   [%s]" % (
                    entry["name"], ", ".join(marks) or "no tools")
                self._model.addItem(label, entry["name"])
            if not self._ollama_models:
                self._model.addItem("(server not reachable)", "")
            index = self._model.findData(wanted)
            self._model.setCurrentIndex(max(index, 0))
            self._model.blockSignals(False)
            self._refresh_routing()

        def _describe_current(self):
            if self._selected_provider() == museAgent.PROVIDER_OLLAMA:
                count = len(self._ollama_models)
                capable = sum(1 for e in self._ollama_models if e["tools"])
                self._current.setText(
                    "%d model(s) on this server, %d can call tools."
                    % (count, capable) if count else
                    "No models listed — is the server running?")
                return
            key, source = museAgent.resolve_api_key()
            if key:
                saved = " · also saved on this machine" if load_saved_api_key() else ""
                self._current.setText(
                    "In use: %s, %d chars, ends %s%s"
                    % (source, len(key), key[-4:], saved))
            else:
                self._current.setText("No key set — Muse cannot reach a model.")

        def _refresh_routing(self):
            """Show where the request in this dialog would actually go."""
            if self._selected_provider() == museAgent.PROVIDER_OLLAMA:
                base = (self._ollama_url.text().strip()
                        or museAgent.OLLAMA_DEFAULT_BASE_URL)
                model = self._model.currentData() or ""
                problem = museAgent.describe_model_problem(
                    model, museAgent.PROVIDER_OLLAMA, self._ollama_models)
                self._routing.setText(
                    "Endpoint &nbsp;<b>%s/v1/messages</b><br>"
                    "Header &nbsp;&nbsp;&nbsp;<span style='color:#858585'>"
                    "none — Ollama authenticates nothing</span><br>"
                    "Model &nbsp;&nbsp;&nbsp;&nbsp;<b>%s</b>%s"
                    % (_escape_html(base),
                       _escape_html(model or "(none selected)"),
                       ("<br><br><span style='color:#e0a030'>%s</span>"
                        % _escape_html(problem)) if problem else ""))
                return
            typed = self._key_edit.text().strip()
            key = typed or museAgent.resolve_api_key()[0]
            if not key:
                self._routing.setText(
                    "<i>Enter a key to see where it will be sent.</i>")
                return
            # resolve_base_url reads the environment, so ask about the typed
            # key directly rather than temporarily mutating os.environ.
            if key.startswith(museAgent.META_KEY_PREFIX):
                base = museAgent.META_BASE_URL
                why = "Meta Muse key"
            else:
                # The ENVIRONMENT's endpoint, not resolve_base_url(): that
                # applies provider inference, so with MUSE_PROVIDER=ollama
                # saved from a previous session it answers with the local
                # server no matter which back end this dialog has selected --
                # and the panel would show an Ollama endpoint next to an
                # Anthropic key. What the hosted branch wants is an explicit
                # gateway if one is set, and Anthropic otherwise.
                base, why = museAgent._env_base_url()
                if not base:
                    base, why = "https://api.anthropic.com", "Anthropic default"
            style = museAgent.resolve_auth_style(base)
            header = ("Authorization: Bearer" if style == "bearer" else "x-api-key")
            self._routing.setText(
                "Endpoint &nbsp;<b>%s</b> &nbsp;<span style='color:#858585'>(%s)</span><br>"
                "Header &nbsp;&nbsp;&nbsp;<b>%s</b><br>"
                "Model &nbsp;&nbsp;&nbsp;&nbsp;<b>%s</b>"
                % (_escape_html(base), _escape_html(why),
                   _escape_html(header),
                   _escape_html(museAgent.resolve_model(base))))

        def _on_forget(self):
            if forget_api_key():
                self._remember.setChecked(False)
                QtWidgets.QMessageBox.information(
                    self, "Muse",
                    "Saved key removed. The key already in this process stays "
                    "active until usdview restarts.")
            else:
                QtWidgets.QMessageBox.information(
                    self, "Muse", "There was no saved key.")
            self._describe_current()

        def _on_save(self):
            provider = self._selected_provider()
            if provider == museAgent.PROVIDER_OLLAMA:
                url = (self._ollama_url.text().strip()
                       or museAgent.OLLAMA_DEFAULT_BASE_URL)
                model = self._model.currentData() or ""
                problem = museAgent.describe_model_problem(
                    model, museAgent.PROVIDER_OLLAMA, self._ollama_models)
                if problem:
                    # Refused rather than warned-and-saved: a model that
                    # cannot call tools makes Muse answer fluently and change
                    # nothing, which reads as the assistant working.
                    QtWidgets.QMessageBox.warning(self, "Muse", problem)
                    return
                settings = {
                    "MUSE_PROVIDER": museAgent.PROVIDER_OLLAMA,
                    "MUSE_OLLAMA_URL": url,
                    "MUSE_MODEL": model,
                }
            else:
                # Leaving Ollama clears its pins, or the next session inherits
                # a local model id it will send to Anthropic.
                settings = {
                    "MUSE_PROVIDER": "",
                    "MUSE_OLLAMA_URL": "",
                    "MUSE_MODEL": "",
                }
            for name, value in settings.items():
                if value:
                    os.environ[name] = value
                else:
                    os.environ.pop(name, None)
            if self._remember.isChecked():
                try:
                    save_settings(settings)
                except OSError as error:
                    QtWidgets.QMessageBox.warning(
                        self, "Muse",
                        "The back end is set for this session, but could not "
                        "be saved:\n%s" % error)
            else:
                try:
                    save_settings({name: "" for name in settings})
                except OSError:
                    pass

            typed = self._key_edit.text().strip()
            if typed:
                os.environ["MUSE_API_KEY"] = typed
                if self._remember.isChecked():
                    try:
                        save_api_key(typed)
                    except OSError as error:
                        QtWidgets.QMessageBox.warning(
                            self, "Muse",
                            "The key is set for this session, but could not be "
                            "saved:\n%s" % error)
                elif load_saved_api_key():
                    forget_api_key()
            self.accept()


    def OpenMuseSettings(usdviewApi):
        """Command-plugin entry point for Muse ▸ Settings…"""
        if not _HAS_QT:
            Tf.Warn("museAssistant: Qt not available; settings cannot be shown")
            return None
        dialog = MuseSettingsDialog(usdviewApi, parent=_main_window(usdviewApi))
        dialog.exec()
        return dialog


    class MuseChatPopup(QtWidgets.QWidget):
        """The whole Muse UI: a chat window that appears on a chord.

        Modelled on the Noodles Tab hotbox rather than on a docked panel — it
        floats over usdview, takes a line, and gets out of the way. Escape
        hides it while idle and stops the agent while it is working, so there
        is still a brake on a running tool loop without a button for one.

        The old panel's drawover, /goal, attach-files, Run Python, Inspect
        stage and key dialog are gone by request. The assistant still reaches
        all of that through its own tools; what went away is the human-facing
        chrome, not the capability.
        """

        _instance = None

        @classmethod
        def GetInstance(cls, usdviewApi):
            if cls._instance is None:
                cls._instance = cls(usdviewApi)
            return cls._instance

        def __init__(self, usdviewApi, parent=None):
            super(MuseChatPopup, self).__init__(_main_window(usdviewApi))
            self._api = usdviewApi
            self._conversation = []
            self._attachments = []
            self._agent_thread = None
            # Prompt history, oldest first. _history_index is None when not
            # walking it; _history_prefix is what had been typed when the walk
            # started, and _history_draft is the unsent text to restore on the
            # way back down.
            self._history = []
            self._history_index = None
            self._history_prefix = ""
            self._history_draft = ""
            self._invoker = MainThreadInvoker()
            self._reported_readiness = False
            # Offset from the window origin while dragging, and
            # whether the user has placed the window themselves.
            self._drag_offset = None
            self._placed_by_user = False

            self.setWindowTitle("Muse")
            self.setWindowFlags(QtCore.Qt.WindowType.Tool
                                | QtCore.Qt.WindowType.FramelessWindowHint)
            self._build_ui()
            self.resize(620, 460)

        # -- construction ---------------------------------------------------

        def _build_ui(self):
            self.setStyleSheet(
                "QWidget { background:#1e1e1e; color:#d4d4d4;"
                " font-size:12px; border-radius:8px }")
            outer = QtWidgets.QVBoxLayout(self)
            outer.setContentsMargins(10, 8, 10, 8)
            outer.setSpacing(6)

            # A frameless window has no title bar to drag, so this strip is it.
            # A QLabel does not consume mouse events, so presses on it reach
            # the popup's mousePressEvent and start a move.
            self._header = QtWidgets.QLabel("Muse   ⠿")
            self._header.setStyleSheet(
                "color:#6f6f6f; font-size:10px; letter-spacing:1px;"
                " padding:2px 2px 4px 2px")
            self._header.setCursor(QtCore.Qt.CursorShape.OpenHandCursor)
            outer.addWidget(self._header)

            self._transcript = QtWidgets.QTextBrowser()
            self._transcript.setOpenExternalLinks(True)
            self._transcript.setStyleSheet(
                "QTextBrowser { background:#1e1e1e; border:1px solid #3c3c3c;"
                " border-radius:6px; padding:6px }")
            outer.addWidget(self._transcript, 1)

            row = QtWidgets.QHBoxLayout()
            row.setSpacing(6)

            self._input = QtWidgets.QTextEdit()
            self._input.setPlaceholderText("Ask Muse to change the stage…")
            self._input.setFixedHeight(56)
            self._input.setStyleSheet(
                "QTextEdit { background:#252526; border:1px solid #3c3c3c;"
                " border-radius:6px; padding:5px }")
            self._input.installEventFilter(self)
            # Typing ends the walk, so the next Up searches on what is now in
            # the box. _apply_history_text blocks signals, so recalling does
            # not trip this.
            self._input.textChanged.connect(self._on_input_edited)
            # Widget-level filters never see Escape while usdview's
            # application filter is installed; see _EscapeFilter.
            self._escape_filter = _EscapeFilter(self)
            application = QtWidgets.QApplication.instance()
            if application is not None:
                application.installEventFilter(self._escape_filter)
            row.addWidget(self._input, 1)

            # The one button, per the brief: attach what the viewport shows.
            self._camera_btn = QtWidgets.QToolButton()
            self._camera_btn.setText("📷")
            self._camera_btn.setToolTip(
                "Attach the current viewport, with its camera, to the next message")
            self._camera_btn.setFixedSize(38, 56)
            self._camera_btn.setStyleSheet(
                "QToolButton { background:#252526; border:1px solid #3c3c3c;"
                " border-radius:6px; font-size:18px }"
                "QToolButton:hover { background:#2d2d30 }")
            self._camera_btn.clicked.connect(self._on_capture_viewport)
            row.addWidget(self._camera_btn)
            outer.addLayout(row)

            self._status = QtWidgets.QLabel(self._idle_hint())
            self._status.setStyleSheet("color:#858585; font-size:10px")
            outer.addWidget(self._status)

        @staticmethod
        def _idle_hint():
            return "⏎ send · ⇧⏎ newline · esc close"

        # -- window behaviour ------------------------------------------------

        def eventFilter(self, obj, event):
            if obj is self._input and event.type() == QtCore.QEvent.Type.KeyPress:
                key = event.key()
                mods = event.modifiers()
                if key == QtCore.Qt.Key.Key_Escape:
                    self._on_escape()
                    return True
                if key in (QtCore.Qt.Key.Key_Return, QtCore.Qt.Key.Key_Enter):
                    if mods & QtCore.Qt.KeyboardModifier.ShiftModifier:
                        return False          # newline
                    self._on_send()
                    return True
                if key in (QtCore.Qt.Key.Key_Up, QtCore.Qt.Key.Key_Down):
                    # Only at the edges of the text, so a multi-line draft can
                    # still be navigated with the arrows: Up recalls only from
                    # the first line, Down only from the last.
                    cursor = self._input.textCursor()
                    block = cursor.blockNumber()
                    lastBlock = self._input.document().blockCount() - 1
                    atEdge = (block == 0 if key == QtCore.Qt.Key.Key_Up
                              else block == lastBlock)
                    if atEdge and self._recall_history(
                            -1 if key == QtCore.Qt.Key.Key_Up else 1):
                        return True
                    return False
            return super(MuseChatPopup, self).eventFilter(obj, event)

        def keyPressEvent(self, event):
            if event.key() == QtCore.Qt.Key.Key_Escape:
                self._on_escape()
                return
            super(MuseChatPopup, self).keyPressEvent(event)

        def _on_escape(self):
            """Escape stops a running turn, or closes when idle.

            One key for both keeps the window free of a Stop button while still
            leaving a way to interrupt a forty-step tool loop.
            """
            if self._agent_thread is not None and self._agent_thread.is_alive():
                self._agent_thread.stop()
                self._status.setText("Stopping after the current step…")
                return
            self.hide()

        def _isDragHandle(self, point):
            """True when *point* is on chrome rather than on a control."""
            child = self.childAt(point)
            return child is None or child in (self._header, self._status)

        def mousePressEvent(self, event):
            """Begin a window move from the header, status line or margins.

            The grab area is hit-tested rather than inferred from which widget
            consumed the press: QTextBrowser and QTextEdit handle mouse input
            on their viewport child, so an unconsumed press can still arrive
            here and would otherwise drag the window while the user was trying
            to select text.
            """
            if (event.button() == QtCore.Qt.MouseButton.LeftButton
                    and self._isDragHandle(event.position().toPoint())):
                self._drag_offset = (event.globalPosition().toPoint()
                                     - self.frameGeometry().topLeft())
                self._header.setCursor(QtCore.Qt.CursorShape.ClosedHandCursor)
                event.accept()
                return
            super(MuseChatPopup, self).mousePressEvent(event)

        def mouseMoveEvent(self, event):
            if self._drag_offset is not None:
                self.move(event.globalPosition().toPoint() - self._drag_offset)
                event.accept()
                return
            super(MuseChatPopup, self).mouseMoveEvent(event)

        def mouseReleaseEvent(self, event):
            if self._drag_offset is not None:
                self._drag_offset = None
                # Where the user put it wins over centring from now on.
                self._placed_by_user = True
                self._header.setCursor(QtCore.Qt.CursorShape.OpenHandCursor)
                event.accept()
                return
            super(MuseChatPopup, self).mouseReleaseEvent(event)

        def showEvent(self, event):
            super(MuseChatPopup, self).showEvent(event)
            parent = self.parentWidget()
            if parent is not None and not self._placed_by_user:
                # Centre the first time only; re-centring on every open would
                # undo the drag the moment the window was reopened.
                centre = parent.geometry().center()
                self.move(centre.x() - self.width() // 2,
                          centre.y() - self.height() // 2)
            self._input.setFocus()
            # The key is read fresh on every send, so report it every time the
            # window appears rather than once at construction.
            self._report_backend_readiness()

        def closeEvent(self, event):
            self.hide()
            event.ignore()

        # -- transcript ------------------------------------------------------

        def _append(self, html_or_text, is_html=False):
            cursor = self._transcript.textCursor()
            cursor.movePosition(QtGui.QTextCursor.MoveOperation.End)
            if is_html:
                cursor.insertHtml(html_or_text + "<br>")
            else:
                cursor.insertText(html_or_text + "\n")
            self._transcript.setTextCursor(cursor)
            self._transcript.ensureCursorVisible()

        def _log_system(self, text):
            self._append('<span style="color:#858585"><i>%s</i></span>'
                         % _escape_html(text), is_html=True)

        def _log_user(self, text):
            self._append(
                '<div style="margin:6px 0"><b style="color:#4fc1ff">you</b>  %s</div>'
                % _escape_html(text).replace("\n", "<br>"), is_html=True)

        def _log_assistant(self, text):
            body = _escape_html(text)
            body = re.sub(r"```(?:python)?\s*\n(.*?)```",
                          r'<pre style="background:#252526; color:#d4d4d4;'
                          r' padding:8px; border-radius:6px;'
                          r' border:1px solid #3c3c3c">\1</pre>',
                          body, flags=re.DOTALL)
            self._append(
                '<div style="margin:6px 0"><b style="color:#dcdcaa">muse</b>  %s</div>'
                % body.replace("\n", "<br>"), is_html=True)

        def _append_inline_image(self, b64, width=320):
            self._append('<img src="data:image/png;base64,%s" width="%d">'
                         % (b64, width), is_html=True)

        def _log_capture(self, b64, note):
            """The assistant's own captures, shown as it takes them."""
            self._append('<div style="color:#858585; font-size:10px">📷 %s</div>'
                         % _escape_html(note or "viewport"), is_html=True)
            self._append_inline_image(b64)

        def _refresh_stage_label(self):
            """Kept because UsdviewExecutor calls it after a stage edit."""
            return None

        def _report_backend_readiness(self):
            """Say once per session when there is no key.

            Without this the window looks alive and answers nothing, which is
            the failure mode the whole plugin was written to avoid.
            """
            if self._reported_readiness:
                return
            self._reported_readiness = True

            provider = museAgent.resolve_provider()
            base_url, _base = museAgent.resolve_base_url()
            if provider == museAgent.PROVIDER_OLLAMA:
                # No key to check. What CAN be wrong is the server being off
                # or the chosen model not supporting tools, and both of those
                # produce the same "looks alive, does nothing" symptom a
                # missing key does -- so both are reported here.
                model = museAgent.resolve_model(base_url, provider)
                models = museAgent.fetch_ollama_models(base_url)
                if not models:
                    self._log_system(
                        "Ollama at %s is not answering. Start it, or pick a "
                        "different back end in Muse ▸ Settings…" % base_url)
                    return
                problem = museAgent.describe_model_problem(
                    model, provider, models)
                if problem:
                    self._log_system(problem)
                else:
                    self._log_system("Ready — %s on %s." % (model, base_url))
                return

            api_key, _name = museAgent.resolve_api_key()
            if not api_key:
                self._log_system(
                    "No API key. Set MUSE_API_KEY in the environment that "
                    "launches usdview, then relaunch — this window cannot "
                    "reach a model without one.")

        # -- attachments -----------------------------------------------------

        def _on_capture_viewport(self):
            """Attach what the viewport shows, with the camera that shows it."""
            pixmap = _capture_viewport_pixmap(self._api)
            if pixmap is None:
                self._status.setText("Viewport not grabbable.")
                return
            b64 = _pixmap_to_base64(pixmap)
            if not b64:
                self._status.setText("Viewport capture failed.")
                return
            self._attachments.append(museAgent.Attachment(
                b64=b64,
                media_type="image/png",
                label="viewport %d" % (len(self._attachments) + 1),
                camera=_get_camera_info(self._api),
                scene=_get_scene_description(
                    self._api, stage=getattr(self._api, "stage", None)),
                source="viewport",
            ))
            self._append_inline_image(b64, width=220)
            self._status.setText(
                "%d viewport capture(s) attached — they go with your next message."
                % len(self._attachments))

        # -- sending ---------------------------------------------------------

        def _set_busy(self, busy):
            self._camera_btn.setEnabled(not busy)
            self._status.setText("Working… esc stops" if busy else self._idle_hint())

        def _on_input_edited(self):
            self._history_index = None

        def _recall_history(self, step):
            """Walk prompt history, matching what has already been typed.

            Shell-style: the text present when the walk starts becomes a prefix
            filter, so typing "add a" and pressing Up offers only the prompts
            that began that way, newest first. An empty box matches everything,
            which makes Up plain "previous prompt".

            Coming back down past the newest match restores the draft that was
            being typed, so a recall never costs unsent text.

            Args:
                step: -1 for Up (older), 1 for Down (newer).

            Returns:
                bool: True if the key was consumed.
            """
            if not self._history:
                return False

            if self._history_index is None:
                if step > 0:
                    return False        # nothing to come back to yet
                self._history_draft = self._input.toPlainText()
                self._history_prefix = self._history_draft.strip()
                self._history_index = len(self._history)

            prefix = self._history_prefix.lower()
            index = self._history_index
            while True:
                index += step
                if index < 0:
                    return True         # oldest match reached; stay put
                if index >= len(self._history):
                    # Past the newest match: back to what was being typed.
                    self._history_index = None
                    self._apply_history_text(self._history_draft)
                    return True
                if self._history[index].lower().startswith(prefix):
                    self._history_index = index
                    self._apply_history_text(self._history[index])
                    return True

        def _apply_history_text(self, text):
            """Replace the input, cursor at the end, without losing the walk."""
            self._input.blockSignals(True)
            try:
                self._input.setPlainText(text)
                cursor = self._input.textCursor()
                cursor.movePosition(QtGui.QTextCursor.MoveOperation.End)
                self._input.setTextCursor(cursor)
            finally:
                self._input.blockSignals(False)

        def _remember_prompt(self, text):
            """Record a sent prompt and end any walk in progress."""
            if text and (not self._history or self._history[-1] != text):
                self._history.append(text)
            self._history_index = None
            self._history_prefix = ""
            self._history_draft = ""

        def _on_send(self):
            if self._agent_thread is not None and self._agent_thread.is_alive():
                self._status.setText("Already working — esc stops it first.")
                return

            text = self._input.toPlainText().strip()
            attachments = list(self._attachments)
            if not text and not attachments:
                return
            if not text:
                text = ("Look at the attached viewport capture and the camera "
                        "metadata with it, then make the change it shows.")

            self._input.clear()
            self._remember_prompt(text)
            self._attachments = []
            self._log_user(text)

            self._conversation.append({
                "role": "user",
                "content": museAgent.build_user_content(text, attachments),
            })
            system_prompt = museAgent.build_system_prompt(
                stage_context=build_stage_context(self._api))

            self._set_busy(True)
            self._agent_thread = _AgentThread(
                self, list(self._conversation), system_prompt)
            self._agent_thread.start()

        # -- agent callbacks (always on the main thread) ----------------------

        def _on_agent_event(self, kind, payload):
            try:
                if kind == "thinking":
                    self._append(
                        '<div style="color:#7a7a7a; font-style:italic;'
                        ' border-left:2px solid #3c3c3c; padding-left:8px;'
                        ' margin:4px 0">%s</div>'
                        % _escape_html(payload.get("text", "")).replace("\n", "<br>"),
                        is_html=True)
                elif kind == "text":
                    self._log_assistant(payload.get("text", ""))
                elif kind == "tool_use":
                    name = payload.get("name", "?")
                    arguments = payload.get("input") or {}
                    if name == "run_python":
                        self._append(
                            '<div style="color:#dcdcaa; font-size:10px;'
                            ' margin-top:6px">▶ run_python</div>'
                            '<pre style="background:#252526; color:#d4d4d4;'
                            ' padding:8px; border-radius:6px;'
                            ' border:1px solid #3c3c3c">%s</pre>'
                            % _escape_html(arguments.get("code", "")), is_html=True)
                    else:
                        self._append(
                            '<div style="color:#dcdcaa; font-size:10px;'
                            ' margin-top:6px">▶ %s %s</div>'
                            % (_escape_html(name),
                               _escape_html(json.dumps(arguments, default=str)[:200])),
                            is_html=True)
                    self._status.setText("Running %s… esc stops" % name)
                elif kind == "tool_result":
                    if payload.get("image_b64"):
                        self._append_inline_image(payload["image_b64"])
                    summary = payload.get("summary", "")
                    if summary:
                        self._append(
                            '<div style="color:#6a9955; font-size:10px">↳ %s</div>'
                            % _escape_html(str(summary)[:400]), is_html=True)
                elif kind == "iteration":
                    index = payload.get("index", 0)
                    if index:
                        self._status.setText(
                            "Working… step %d · esc stops" % (index + 1))
                elif kind == "error":
                    self._log_system(payload.get("message", "error"))
            except Exception:
                traceback.print_exc()

        def _on_agent_finished(self, conversation, error):
            self._set_busy(False)
            self._agent_thread = None
            if error:
                self._append(
                    '<pre style="background:#3a1e1e; color:#f48771; padding:8px;'
                    ' border-radius:6px; border:1px solid #5a2d2d">%s</pre>'
                    % _escape_html(error), is_html=True)
                self._status.setText("Failed — see above.")
                return
            if conversation:
                self._conversation = conversation


else:
    # Headless stub so GetInstance doesn't crash import probes
    class MuseChatPopup:  # type: ignore
        _instance = None

        @classmethod
        def GetInstance(cls, usdviewApi):
            raise RuntimeError("Qt not available: %s" % (_QT_IMPORT_ERROR,))

        def show(self): pass
        def raise_(self): pass
        def activateWindow(self): pass


# ---------------------------------------------------------------------------
# Utility: open the panel (command entry point)
# ---------------------------------------------------------------------------

def OpenMuseAssistant(usdviewApi):
    """Command-plugin entry point: show the chat window.

    Toggles rather than only showing, because the same chord opens and
    dismisses it — a popup you can only open is a window you have to go and
    close.
    """
    if not _HAS_QT:
        Tf.Warn("museAssistant: Qt not available; chat cannot be shown: %s" % _QT_IMPORT_ERROR)
        return None
    chat = MuseChatPopup.GetInstance(usdviewApi)
    if chat.isVisible():
        chat.hide()
        return chat
    chat.show()
    chat.raise_()
    chat.activateWindow()
    return chat


# ---------------------------------------------------------------------------
# PluginContainer
# ---------------------------------------------------------------------------

_container = None  # strong ref — usdview's loader doesn't retain containers without commands

class MuseAssistantContainer(PluginContainer):

    # Cmd+Shift+M on macOS, Ctrl+Shift+M elsewhere. Plain Cmd+M is the system
    # Minimize Window shortcut, so it is left alone.
    SHORTCUT = "Ctrl+Shift+M"

    def registerPlugins(self, plugRegistry, plugCtx):
        global _container
        _container = self
        self._api = plugCtx
        self._shortcut = None

        # A key saved through the settings dialog should survive a restart
        # without being exported by hand; the environment still wins.
        apply_saved_api_key()
        # ...and so should the back end it was saved for. Without this a
        # session that chose Ollama comes back pointed at Anthropic, with a
        # key it does not need and a 401 to explain.
        apply_saved_settings()

        self._openCmd = plugRegistry.registerCommandPlugin(
            "MuseAssistantContainer.open",
            "Muse Chat\t%s" % self.SHORTCUT,
            lambda api: OpenMuseAssistant(api))

        self._settingsCmd = plugRegistry.registerCommandPlugin(
            "MuseAssistantContainer.settings",
            "Settings…",
            lambda api: OpenMuseSettings(api))

        atexit.register(self._shutdown)

    def configureView(self, plugRegistry, plugUIBuilder):
        # File ▸ Muse ▸ … puts the assistant where the rest of the app's
        # commands live, rather than in a top-level menu of its own.
        fileMenu = plugUIBuilder.findOrCreateMenu("File")
        submenu = fileMenu.findOrCreateSubmenu("Muse")
        self._keepInPlace(submenu.addItem(self._openCmd))
        self._keepInPlace(submenu.addItem(self._settingsCmd))

        # The top-level menu stays as the quick route.
        menu = plugUIBuilder.findOrCreateMenu("Muse")
        self._keepInPlace(menu.addItem(self._openCmd))
        self._keepInPlace(menu.addItem(self._settingsCmd))

        self._installShortcut()

    @staticmethod
    def _keepInPlace(action):
        """Stop macOS relocating the item out of the menu we put it in.

        Qt gives every action TextHeuristicRole by default, which lets the
        macOS native menu bar scan the text and move anything looking like
        "Settings", "Preferences", "About" or "Quit" into the application menu
        beside the Apple logo. That is why usdview's own File ▸ Quit is absent
        there — and why "Settings…" vanished from the Muse menu. NoRole means
        the item stays where it was added on every platform.
        """
        if not _HAS_QT or action is None:
            return action
        try:
            action.setMenuRole(QtGui.QAction.MenuRole.NoRole)
        except Exception:
            pass
        return action

    def _installShortcut(self):
        """Bind the chord on the main window.

        usdview's registerCommandPlugin takes no shortcut, so the binding is
        made directly. ApplicationShortcut rather than WindowShortcut so it
        still fires while focus is in the viewport or the Noodles canvas.
        """
        if not _HAS_QT:
            return
        try:
            window = self._api.qMainWindow
            if window is None:
                return
            self._shortcut = QtGui.QShortcut(
                QtGui.QKeySequence(self.SHORTCUT), window)
            self._shortcut.setContext(
                QtCore.Qt.ShortcutContext.ApplicationShortcut)
            self._shortcut.activated.connect(
                lambda: OpenMuseAssistant(self._api))
        except Exception as error:
            Tf.Warn("museAssistant: could not bind %s: %s"
                    % (self.SHORTCUT, error))

    def _shutdown(self):
        try:
            if MuseChatPopup._instance is not None:
                MuseChatPopup._instance.hide()
        except Exception:
            pass


Tf.Type.Define(MuseAssistantContainer)
