#
# THE EXECUTION STACK PANEL, end to end.
#
# This exists because the panel shipped with no test and was broken in
# usdview -- and ONLY in usdview. `execStackUI` compiled the rig with
# `rigexec.Rig(stage)` on the stage usdview hands out, which comes from a
# `UsdStageCache`; the Python binding refuses a stage its wrapper does not
# own ("needs a `__owner` capsule"), so the panel reported "Rig compile
# failed" for every rig while the same rig compiled fine from a script. A
# unit test over an in-memory stage could not have caught it: the bug is
# entirely about WHICH stage object the panel is handed.
#
# So the assertions are about the ordered list actually being populated,
# not about the widget existing:
#   * the RigExec menu carries the item (and the Window menu does not),
#   * opening it on the biped lists movers in a real order,
#   * that order is IDENTICAL to a script-side compile of the same layers,
#   * the deformer-after-constraint invariant holds,
#   * and the filters narrow the list rather than emptying it.
#
import os

from pxr import Usd

RIG = "/Biped/Rig"


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _RigExecMenu(appController):
    from pxr.Usdviewq.qt import QtWidgets
    menus = {}
    for child in appController._mainWindow.menuBar().children():
        if isinstance(child, QtWidgets.QMenu):
            menus[str(child.title()).replace("&", "")] = child
    return menus


def _Trigger(menu, title):
    for action in menu.actions():
        if action.text() == title:
            action.trigger()
            return True
    return False


def _ShownMovers(panel):
    """How many mover rows the current filter leaves visible."""
    return sum(1 for e in panel._movers
               if panel._Keep(e["type"], e["path"].rsplit("/", 1)[-1],
                              e.get("targets") or []))


def testUsdviewInputFunction(appController):
    import execStackUI
    import rigexec

    appController._processEvents()
    api = appController._usdviewApi
    stage = api.stage

    # --- 1. registration, in the RigExec menu only ----------------------
    reg = appController._plugRegistry
    _Check(reg.getCommandPlugin(
        "RigExecUsdviewContainer.execStack") is not None,
        "RigExec -> Execution Stack command is registered")
    menus = _RigExecMenu(appController)
    _Check("RigExec" in menus, "there is a RigExec menu: %s" % sorted(menus))
    rigMenu = menus["RigExec"]
    titles = [a.text() for a in rigMenu.actions()]
    title = next((t for t in titles if "Execution Stack" in t), None)
    _Check(title is not None,
           "RigExec menu has an Execution Stack item: %s" % titles)
    window = menus.get("Window")
    if window is not None:
        _Check(not any("Execution Stack" in a.text()
                       for a in window.actions()),
               "the panel is in the RigExec menu ONLY, not Window")

    # --- 2. opening it populates the ORDERED list -----------------------
    _Check(_Trigger(rigMenu, title), "the menu item triggered")
    appController._processEvents()
    panel = execStackUI.ExecStackPanel.GetInstance(api)
    _Check(panel is not None, "the panel was created")
    appController._processEvents()

    _Check(getattr(panel, "_compileError", None) is None,
           "the rig compiled inside usdview, but: %s"
           % getattr(panel, "_compileError", None))
    movers = list(panel._movers)
    _Check(len(movers) > 10,
           "the ordered mover list is populated, got %d" % len(movers))
    _Check(panel._solvers, "solvers were listed too")

    # --- 3. the order matches a script-side compile ---------------------
    # Same layers, a stage this process owns -- which is exactly the twin
    # the panel now builds for itself.
    twin = Usd.Stage.Open(stage.GetRootLayer(), stage.GetSessionLayer())
    rig = rigexec.Rig(twin, RIG)
    rig.compile()
    order = rig.mover_order
    if callable(order):
        order = order()
    expected = [str(e["path"]) for e in order]
    got = [str(e["path"]) for e in movers]
    _Check(got == expected,
           "panel order differs from a script compile:\n  panel  %s\n  "
           "script %s" % (got[:6], expected[:6]))

    # --- 4. the deformer-after-constraint invariant ---------------------
    deform = [e for e in movers
              if "SkinMover" in e["type"] or "MatrixMover" in e["type"]]
    constraints = [e for e in movers if "Constraint" in e["type"]]
    if deform and constraints:
        first = min(e["ordinal"] for e in deform)
        last = max(e["ordinal"] for e in constraints)
        _Check(first > last,
               "a deformer at ordinal %d runs before the last constraint "
               "at %d -- it would read stale joint transforms"
               % (first, last))

    # --- 5. filtering narrows rather than empties ------------------------
    box = panel._filterBox
    counts = {}
    for i in range(box.count()):
        box.setCurrentIndex(i)
        appController._processEvents()
        counts[str(box.itemText(i))] = _ShownMovers(panel)
    _Check(any(v > 0 for v in counts.values()),
           "at least one filter shows mover rows: %s" % counts)
    _Check(max(counts.values()) == len(movers),
           "some filter shows every mover (the 'all' case): %s" % counts)
    _Check(min(counts.values()) < len(movers),
           "some filter actually narrows the list: %s" % counts)
    box.setCurrentIndex(0)
    appController._processEvents()

    shot = os.getenv("RIGEXEC_EXECSTACK_SHOT")
    if shot:
        appController._mainWindow.grab().save(shot)

    print("RIGEXEC_EXEC_STACK_OK compiled in usdview, %d solvers, %d movers, "
          "order matches a script compile, filters %s"
          % (len(panel._solvers), len(movers), counts))
