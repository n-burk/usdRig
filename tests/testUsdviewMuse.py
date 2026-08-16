#
# testusdview verification that the Muse assistant WORKS INSIDE USDVIEW —
# which the headless test in testMuseAgent.py cannot check: that one exercises
# message shaping and the tool loop against a fake executor, so a panel that
# fails to construct, never reaches the stage view, deadlocks marshalling tool
# calls onto the main thread, or silently drops the model's edits would still
# pass it.
#
# What is asserted here is everything that depends on the running app:
#   * the chat window constructs against the real usdviewApi
#   * pressing Send actually starts work (before the fix, every request was
#     rejected by the API for malformed roles and the panel fell back to
#     printing canned help)
#   * the assistant's run_python REALLY EDITS THE STAGE — asserted by reading
#     the prim back off the stage afterwards, not by trusting the transcript
#   * tool calls issued from the agent thread execute on the main thread
#     without deadlocking, and results flow back to the model
#   * capture_viewport returns a real image block from the live GL widget
#   * the camera button attaches the viewport with its camera metadata
#   * escape closes the window through the real key path, past usdview's
#     application-wide event filter, without stealing Escape elsewhere
#   * stage edits are unconditional now that the read-only toggle is gone
#
# museAssistant/museAgent are imported by name: testusdview execs this file
# with no __file__ bound, and the launcher already puts the plugin directory
# on PYTHONPATH -- which is also how usdview itself finds them.
#
import base64
import json
import os
import sys
import time
import types

TIMEOUT_SECONDS = 30.0


def _block(kind, **kwargs):
    block = types.SimpleNamespace(type=kind)
    for key, value in kwargs.items():
        setattr(block, key, value)
    return block


def _installScriptedModel(turns, captured):
    """Replace the anthropic SDK with a scripted one; record every request."""
    class _Stream(object):
        def __init__(self, message):
            self._message = message

        def __enter__(self):
            return self

        def __exit__(self, *exc):
            return False

        def get_final_message(self):
            return self._message

    class _Messages(object):
        def stream(self, **kwargs):
            captured.append(kwargs)
            index = len(captured) - 1
            return _Stream(turns[min(index, len(turns) - 1)])

    class _Client(object):
        def __init__(self, api_key=None, **kwargs):
            self.messages = _Messages()

    module = types.ModuleType("anthropic")
    module.Anthropic = _Client
    sys.modules["anthropic"] = module


def _pump(panel, predicate, what):
    """
    Spin usdview's event loop until *predicate* holds.  This is what proves
    the threading design: the agent thread blocks on tool calls until the main
    thread drains its posted events, so a test that never pumps would hang —
    and so would the real app.
    """
    from pxr.Usdviewq.qt import QtWidgets

    deadline = time.time() + TIMEOUT_SECONDS
    while time.time() < deadline:
        QtWidgets.QApplication.processEvents()
        if predicate():
            return
        time.sleep(0.01)
    raise AssertionError("timed out after %gs waiting for %s" % (TIMEOUT_SECONDS, what))


def _idle(panel):
    return panel._agent_thread is None or not panel._agent_thread.is_alive()


def _assertValidRequests(captured):
    for position, request in enumerate(captured):
        messages = request["messages"]
        if not messages:
            raise AssertionError("request %d had no messages" % position)
        for index, message in enumerate(messages):
            if message.get("role") not in ("user", "assistant"):
                raise AssertionError(
                    "request %d messages.%d.role is %r — the API only accepts "
                    "'user' or 'assistant'" % (position, index, message.get("role")))
        if messages[0]["role"] != "user":
            raise AssertionError("request %d does not open on a user turn" % position)
        for index in range(1, len(messages)):
            if messages[index]["role"] == messages[index - 1]["role"]:
                raise AssertionError(
                    "request %d has two %r turns in a row at index %d — the API "
                    "rejects this" % (position, messages[index]["role"], index))
        if "You are Muse" not in (request.get("system") or ""):
            raise AssertionError("request %d lost the system prompt" % position)


def testUsdviewInputFunction(appController):
    from pxr.Usdviewq.qt import QtWidgets

    import museAgent
    import museAssistant

    api = appController._usdviewApi
    stage = api.stage
    if not stage:
        raise AssertionError("no stage")

    os.environ["MUSE_API_KEY"] = "sk-ant-test-not-a-real-key"

    panel = museAssistant.MuseChatPopup.GetInstance(api)
    if panel is None:
        raise AssertionError("the panel did not construct")
    for attribute in ("_input", "_camera_btn", "_transcript"):
        if not hasattr(panel, attribute):
            raise AssertionError("the chat window is missing %s" % attribute)

    # ---- 1. a real edit, driven end to end through the tool loop ----------
    target = "/MuseTest/Ball"
    turns = [
        types.SimpleNamespace(stop_reason="tool_use", content=[
            _block("text", text="Checking what is on the stage first."),
            _block("tool_use", id="a1", name="inspect_stage", input={"mode": "summary"}),
        ]),
        types.SimpleNamespace(stop_reason="tool_use", content=[
            _block("tool_use", id="a2", name="run_python", input={"code": (
                "from pxr import UsdGeom, Gf\n"
                "sphere = UsdGeom.Sphere.Define(stage, '%s')\n"
                "sphere.CreateRadiusAttr(2.0)\n"
                "UsdGeom.XformCommonAPI(sphere).SetTranslate(Gf.Vec3d(1, 2, 3))\n"
                "print('created', sphere.GetPath())\n" % target)}),
        ]),
        types.SimpleNamespace(stop_reason="tool_use", content=[
            _block("tool_use", id="a3", name="capture_viewport", input={"note": "confirm"}),
        ]),
        types.SimpleNamespace(stop_reason="end_turn", content=[
            _block("text", text="Created %s at (1,2,3) with radius 2." % target),
        ]),
    ]
    captured = []
    _installScriptedModel(turns, captured)

    panel._input.setPlainText("create a sphere at %s and show me" % target)
    panel._on_send()
    if panel._agent_thread is None:
        raise AssertionError(
            "Send did not start the assistant — the panel is not wired to the agent")
    _pump(panel, lambda: _idle(panel), "the assistant to finish")

    # The claim that matters: did the stage actually change?
    prim = stage.GetPrimAtPath(target)
    if not prim or not prim.IsValid():
        raise AssertionError(
            "the assistant reported success but %s does not exist — run_python "
            "never reached the stage" % target)
    radius = prim.GetAttribute("radius").Get()
    if radius != 2.0:
        raise AssertionError("radius is %r, expected 2.0" % radius)
    from pxr import UsdGeom
    translate = prim.GetAttribute("xformOp:translate").Get()
    if translate is None or tuple(translate) != (1.0, 2.0, 3.0):
        raise AssertionError("translate is %r, expected (1,2,3)" % (translate,))

    if len(captured) != 4:
        raise AssertionError("expected 4 model round trips, got %d" % len(captured))
    _assertValidRequests(captured)

    # The viewport capture must come back as a real image the model can see.
    final = captured[-1]["messages"]
    images = [
        block
        for message in final if isinstance(message.get("content"), list)
        for entry in message["content"] if isinstance(entry, dict)
        for block in (entry.get("content") or []) if isinstance(block, dict)
        if block.get("type") == "image"
    ]
    if not images:
        raise AssertionError("capture_viewport produced no image block for the model")
    png = base64.b64decode(images[0]["source"]["data"])
    if not png.startswith(b"\x89PNG"):
        raise AssertionError("the capture was not a PNG")
    if len(png) < 1000:
        raise AssertionError(
            "the capture is %d bytes — the framebuffer grab looks empty" % len(png))

    # The camera that produced it must ride along as text.
    capture_text = json.dumps([
        entry
        for message in final if isinstance(message.get("content"), list)
        for entry in message["content"] if isinstance(entry, dict)
        and entry.get("type") == "tool_result"
    ], default=str)
    if "viewMatrix" not in capture_text and "camera_prim_path" not in capture_text:
        raise AssertionError("the capture carried no camera information")

    transcript = panel._transcript.toPlainText()
    if target not in transcript:
        raise AssertionError("the transcript never mentions the work that was done")

    # ---- 1a. the menu items exist and macOS cannot steal them ------------
    # Qt defaults every action to TextHeuristicRole, which lets the macOS
    # native menu bar move anything reading like "Settings"/"Preferences" into
    # the application menu beside the Apple logo. That is exactly how
    # "Settings…" disappeared from the Muse menu, so the role is asserted, not
    # just the presence.
    from pxr.Usdviewq.qt import QtGui as _QtGui

    def _menu_named(name):
        for child in api.qMainWindow.menuBar().children():
            if isinstance(child, QtWidgets.QMenu) \
                    and str(child.title()).replace("&", "") == name:
                return child
        return None

    fileMenu = _menu_named("File")
    if fileMenu is None:
        raise AssertionError("usdview has no File menu to hang Muse off")
    submenu = None
    for act in fileMenu.actions():
        if act.menu() is not None and str(act.text()).replace("&", "") == "Muse":
            submenu = act.menu()
            break
    if submenu is None:
        raise AssertionError("File ▸ Muse submenu was never created")

    for menu, where in ((submenu, "File > Muse"), (_menu_named("Muse"), "Muse")):
        if menu is None:
            raise AssertionError("%s menu is missing" % where)
        # Bind the list: the temporary returned by actions() can be collected
        # mid-iteration and take its QAction wrappers with it.
        entries = [a for a in menu.actions() if not a.isSeparator()]
        labels = [str(a.text()) for a in entries]
        roles = [a.menuRole() for a in entries]
        if not any("Settings" in label for label in labels):
            raise AssertionError("%s has no Settings item: %s" % (where, labels))
        for label, role in zip(labels, roles):
            if role != _QtGui.QAction.MenuRole.NoRole:
                raise AssertionError(
                    "%s > %r has role %s — macOS will move it out of this menu"
                    % (where, label, role))

    # The settings dialog must build against the live api.
    dialog = museAssistant.MuseSettingsDialog(api, parent=api.qMainWindow)
    dialog._key_edit.setText("LLM_probe_example")
    routing = dialog._routing.text()
    if "api.meta.ai" not in routing or "Bearer" not in routing:
        raise AssertionError("settings did not resolve a Meta key: %r" % routing)
    dialog._key_edit.setText("sk-ant-probe")
    routing = dialog._routing.text()
    if "api.anthropic.com" not in routing or "x-api-key" not in routing:
        raise AssertionError("settings did not resolve an Anthropic key: %r" % routing)
    dialog.reject()
    # Destroy it, do not merely hide it.
    #
    # _EscapeFilter claims Escape only while focus is INSIDE the chat window
    # (deliberately -- every other Escape has to fall through to usdview). A
    # rejected-but-undestroyed dialog still parented to the main window leaves
    # this headless application with no focus widget at all, so the filter
    # correctly declines and 1b below fails for a reason that has nothing to
    # do with the key path it is testing. Verified by substituting a bare
    # three-widget QDialog here, which reproduces it exactly -- so this is
    # about activation, not about the dialog under test.
    dialog.setParent(None)
    dialog.deleteLater()
    QtWidgets.QApplication.processEvents()
    api.qMainWindow.activateWindow()
    QtWidgets.QApplication.processEvents()

    # ---- 1b. escape closes the window, through the real key path ---------
    # usdview installs an application-wide AppEventFilter that swallows every
    # Escape to reset focus (Usdviewq/appEventFilter.py) and returns True
    # before any widget sees it, so a widget-level handler never runs. This
    # must go through QApplication.notify() — calling the popup's own method
    # directly passes whether or not the key ever reaches it, which is exactly
    # how this shipped broken.
    from pxr.Usdviewq.qt import QtGui as _QtGui, QtCore as _QtCore

    panel.show()
    panel._input.setFocus()
    _pump(panel, lambda: panel.isVisible(), "the chat window to appear")

    escape = _QtGui.QKeyEvent(_QtCore.QEvent.Type.KeyPress,
                              _QtCore.Qt.Key.Key_Escape,
                              _QtCore.Qt.KeyboardModifier.NoModifier)
    QtWidgets.QApplication.instance().notify(
        QtWidgets.QApplication.focusWidget() or panel, escape)
    QtWidgets.QApplication.processEvents()
    if panel.isVisible():
        raise AssertionError(
            "escape did not close the chat window — usdview's application "
            "event filter is swallowing the key again")

    # ...and it must not steal Escape from the rest of usdview.
    claimed = panel._escape_filter.eventFilter(
        panel, _QtGui.QKeyEvent(_QtCore.QEvent.Type.KeyPress,
                                _QtCore.Qt.Key.Key_Escape,
                                _QtCore.Qt.KeyboardModifier.NoModifier))
    if claimed:
        raise AssertionError(
            "escape was claimed while the chat window was hidden — usdview's "
            "own focus-reset behaviour would be broken everywhere else")
    panel.show()
    _pump(panel, lambda: panel.isVisible(), "the chat window to reappear")

    # ---- 1bb. the window can be dragged, and stays where it is put -------
    # It is frameless, so there is no title bar: the header strip, status line
    # and margins are the grab areas, hit-tested so a drag never competes with
    # selecting text in the transcript.
    def _mouse(kind, globalPt, target, buttons=None):
        buttons = _QtCore.Qt.MouseButton.LeftButton if buttons is None else buttons
        event = _QtGui.QMouseEvent(
            kind, _QtCore.QPointF(target.mapFromGlobal(globalPt)),
            _QtCore.QPointF(globalPt), _QtCore.Qt.MouseButton.LeftButton,
            buttons, _QtCore.Qt.KeyboardModifier.NoModifier)
        QtWidgets.QApplication.sendEvent(target, event)
        QtWidgets.QApplication.processEvents()

    panel.show()
    QtWidgets.QApplication.processEvents()
    origin = panel.pos()
    grab = panel._header.mapToGlobal(_QtCore.QPoint(20, 5))
    _mouse(_QtCore.QEvent.Type.MouseButtonPress, grab, panel)
    _mouse(_QtCore.QEvent.Type.MouseMove, grab + _QtCore.QPoint(120, 80), panel)
    moved = panel.pos()
    if (moved.x() - origin.x(), moved.y() - origin.y()) != (120, 80):
        raise AssertionError(
            "dragging the header moved the window by (%d, %d), expected (120, 80)"
            % (moved.x() - origin.x(), moved.y() - origin.y()))
    _mouse(_QtCore.QEvent.Type.MouseButtonRelease, grab + _QtCore.QPoint(120, 80),
           panel, buttons=_QtCore.Qt.MouseButton.NoButton)
    if panel._drag_offset is not None:
        raise AssertionError("the drag did not end on release")

    # Reopening must not yank it back to the centre.
    placed = panel.pos()
    panel.hide()
    QtWidgets.QApplication.processEvents()
    panel.show()
    QtWidgets.QApplication.processEvents()
    if panel.pos() != placed:
        raise AssertionError(
            "the window re-centred on reopen, discarding where it was put")

    # A press in the transcript is text selection, not a window move.
    before = panel.pos()
    inside = panel._transcript.mapToGlobal(_QtCore.QPoint(30, 20))
    _mouse(_QtCore.QEvent.Type.MouseButtonPress, inside, panel._transcript)
    _mouse(_QtCore.QEvent.Type.MouseMove, inside + _QtCore.QPoint(60, 20),
           panel._transcript)
    _mouse(_QtCore.QEvent.Type.MouseButtonRelease, inside + _QtCore.QPoint(60, 20),
           panel._transcript, buttons=_QtCore.Qt.MouseButton.NoButton)
    if panel.pos() != before:
        raise AssertionError(
            "dragging inside the transcript moved the window instead of "
            "selecting text")

    # ---- 1c. prompt history recalls by prefix ----------------------------
    # Up/Down must go through the real key path: the input is a QTextEdit, so
    # the arrows have a default meaning (move between lines) that the recall
    # has to share rather than swallow.
    for remembered in ("add a sphere", "add a cube", "add a light"):
        panel._remember_prompt(remembered)

    def _arrow(key):
        event = _QtGui.QKeyEvent(_QtCore.QEvent.Type.KeyPress, key,
                                 _QtCore.Qt.KeyboardModifier.NoModifier)
        QtWidgets.QApplication.instance().notify(panel._input, event)
        QtWidgets.QApplication.processEvents()
        return panel._input.toPlainText()

    UP, DOWN = _QtCore.Qt.Key.Key_Up, _QtCore.Qt.Key.Key_Down

    panel._input.setPlainText("")
    QtWidgets.QApplication.processEvents()
    walked = [_arrow(UP), _arrow(UP), _arrow(UP)]
    if walked != ["add a light", "add a cube", "add a sphere"]:
        raise AssertionError("an empty box did not walk history newest first: %s" % walked)

    # What is already typed filters the walk, and coming back down restores it.
    panel._input.setPlainText("add a c")
    QtWidgets.QApplication.processEvents()
    if _arrow(UP) != "add a cube":
        raise AssertionError("prefix search did not find the matching prompt")
    if _arrow(DOWN) != "add a c":
        raise AssertionError("coming back down did not restore the typed draft")

    # Editing restarts the search against the new text.
    panel._input.setPlainText("add a s")
    QtWidgets.QApplication.processEvents()
    if _arrow(UP) != "add a sphere":
        raise AssertionError("editing did not restart the search on the new prefix")

    # A prefix nothing matches must leave the text alone.
    panel._input.setPlainText("zzz no such prompt")
    QtWidgets.QApplication.processEvents()
    if _arrow(UP) != "zzz no such prompt":
        raise AssertionError("an unmatched prefix overwrote what was typed")

    # A multi-line draft keeps the arrows for line navigation.
    panel._input.setPlainText("first line\nsecond line")
    cursor = panel._input.textCursor()
    cursor.movePosition(_QtGui.QTextCursor.MoveOperation.End)
    panel._input.setTextCursor(cursor)
    QtWidgets.QApplication.processEvents()
    if _arrow(UP) != "first line\nsecond line":
        raise AssertionError("Up on the last line recalled instead of moving the cursor")
    if panel._input.textCursor().blockNumber() != 0:
        raise AssertionError("Up did not move the cursor up a line")
    panel._input.clear()
    QtWidgets.QApplication.processEvents()

    # ---- 2. the camera button attaches the viewport, camera and all ------
    # File attachment and paste went with the panel; the one button that
    # remains is the whole attachment story now, so it is what gets asserted.
    if panel._attachments:
        raise AssertionError("the chat window started with attachments pending")

    panel._on_capture_viewport()
    if len(panel._attachments) != 1:
        raise AssertionError("the camera button attached nothing")
    attachment = panel._attachments[0]
    if not attachment.has_camera():
        raise AssertionError(
            "the viewport capture carried no camera — the model would have to "
            "guess the projection it is looking at")
    grabbed = base64.b64decode(attachment.b64)
    if not grabbed.startswith(b"\x89PNG") or len(grabbed) < 1000:
        raise AssertionError(
            "the attached capture is %d bytes and does not look like a PNG"
            % len(grabbed))

    captured2 = []
    _installScriptedModel([types.SimpleNamespace(
        stop_reason="end_turn",
        content=[_block("text", text="I can see the viewport you attached.")])],
        captured2)
    panel._input.setPlainText("what am I looking at?")
    panel._on_send()
    _pump(panel, lambda: _idle(panel), "the attachment turn to finish")

    _assertValidRequests(captured2)
    sent = json.dumps(captured2[0]["messages"], default=str)
    if '"type": "image"' not in sent:
        raise AssertionError("the attached viewport never reached the model")
    if "viewMatrix" not in sent and "camera_prim_path" not in sent:
        raise AssertionError(
            "the capture's camera metadata never reached the model")
    if panel._attachments:
        raise AssertionError(
            "attachments were not cleared after sending — the next message "
            "would silently repeat them")

    # ---- 3. edits are always allowed now --------------------------------
    # The old "Allow stage edits" checkbox went with the panel: the chat window
    # is one input and one button by design, so there is nowhere to put a gate
    # and run_python is unconditional. Undo is the brake. This pins that
    # contract so the refusal path cannot creep back in unnoticed.
    allowed = "/MuseTest/AlwaysAllowed"
    captured3 = []
    _installScriptedModel([
        types.SimpleNamespace(stop_reason="tool_use", content=[
            _block("tool_use", id="c1", name="run_python",
                   input={"code": "stage.DefinePrim('%s', 'Cube')" % allowed})]),
        types.SimpleNamespace(stop_reason="end_turn", content=[
            _block("text", text="Made the cube.")]),
    ], captured3)
    panel._input.setPlainText("make a cube")
    panel._on_send()
    _pump(panel, lambda: _idle(panel), "the edit turn to finish")

    if not stage.GetPrimAtPath(allowed).IsValid():
        raise AssertionError(
            "run_python did not edit the stage — %s was never created" % allowed)
    result = json.dumps(captured3[-1]["messages"], default=str)
    if "read-only" in result:
        raise AssertionError("the removed read-only gate is still refusing edits")

    # ---- 4. it can restyle the usdview UI on demand ----------------------
    # "Manipulate my python session / modify UI on demand" is a claim about the
    # live QMainWindow, so assert against the real widget tree.
    captured4 = []
    _installScriptedModel([
        types.SimpleNamespace(stop_reason="tool_use", content=[
            _block("tool_use", id="d1", name="run_python", input={"code": (
                "dock = QtWidgets.QDockWidget('MuseTestDock', api.qMainWindow)\n"
                "dock.setObjectName('MuseTestDock')\n"
                "dock.setWidget(QtWidgets.QLabel('added by Muse'))\n"
                "api.qMainWindow.addDockWidget("
                "QtCore.Qt.DockWidgetArea.RightDockWidgetArea, dock)\n"
                "api.dataModel.viewSettings.showHUD = False\n"
                "print('dock added')\n")}),
        ]),
        types.SimpleNamespace(stop_reason="end_turn", content=[
            _block("text", text="Added a dock and turned the HUD off.")]),
    ], captured4)
    panel._input.setPlainText("add a dock widget and hide the HUD")
    panel._on_send()
    _pump(panel, lambda: _idle(panel), "the UI turn to finish")

    docks = [d for d in api.qMainWindow.findChildren(QtWidgets.QDockWidget)
             if d.objectName() == "MuseTestDock"]
    if not docks:
        raise AssertionError(
            "the assistant could not modify the usdview UI — no dock was added")
    if api.dataModel.viewSettings.showHUD:
        raise AssertionError("the assistant could not change usdview view settings")
    docks[0].setParent(None)
    api.dataModel.viewSettings.showHUD = True

    print("MUSE_USDVIEW_OK  %d round trips, stage edited at %s, capture %d bytes, "
          "viewport attachment + camera forwarded, edits unconditional, "
          "UI dock added, view settings driven"
          % (len(captured), target, len(png)))
