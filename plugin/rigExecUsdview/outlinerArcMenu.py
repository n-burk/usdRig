#
# RigExec usdview plugin: "Add Composition Arc" on usdview's own prim menu.
#
# The Layer Opinions panel offers the arc flows on right-click, but the
# place a prim is usually right-clicked is usdview's hierarchy -- and the
# viewport, which raises the same menu. This puts the same submenu there,
# driving the same compositionArcsUI flows onto the same undo stack, so an
# arc added from the outliner undoes with Ctrl+Z like one added from the
# panel.
#
# HOW IT GETS IN. usdview has no plugin hook for the prim context menu:
# PrimContextMenu is filled from a fixed list in primContextMenuItems, and
# appController._showPrimContextMenu builds and execs it in two lines
# (appController.py, _showPrimContextMenu). Both the hierarchy's
# customContextMenuRequested and the viewport's right-click go through
# that one method, so it is replaced on the controller INSTANCE -- not the
# class -- with a version that builds the same menu and appends one
# submenu before exec. Reached through the name-mangled
# _UsdviewApi__appController, like gizmoUI and curvenetUI reach the stage
# view. Anything unexpected (an older usdview without the method, a
# PrimContextMenu that fails to import) leaves usdview's menu untouched.
#
from pxr.Usdviewq.qt import QtGui, QtWidgets

_HOOKED_ATTR = "_rigExecArcMenuInstalled"


def InstallPrimContextMenuHook(usdviewApi, undoStack, extras=()):
    """
    Append "Add Composition Arc" to usdview's prim context menu, once.
    `extras` are further item builders, each called as fn(menu, prim) and
    free to add nothing for a prim they do not concern. Returns True when
    the hook is in place.
    """
    controller = getattr(usdviewApi, "_UsdviewApi__appController", None)
    if controller is None:
        return False
    if getattr(controller, _HOOKED_ATTR, False):
        return True
    original = getattr(controller, "_showPrimContextMenu", None)
    try:
        from pxr.Usdviewq.primContextMenu import PrimContextMenu
    except ImportError:
        return False
    if original is None:
        return False

    def _ShowPrimContextMenu(item):
        try:
            menu = PrimContextMenu(controller._mainWindow, item, controller)
            prim = _PrimFor(item, usdviewApi)
            AppendArcMenu(menu, usdviewApi, undoStack, prim)
            for extra in extras:
                extra(menu, prim)
        except Exception:
            # Never cost the artist usdview's own menu over ours.
            return original(item)
        # Kept on the controller as usdview does, so it outlives exec_.
        controller.contextMenu = menu
        menu.exec_(QtGui.QCursor.pos())

    controller._showPrimContextMenu = _ShowPrimContextMenu
    setattr(controller, _HOOKED_ATTR, True)
    return True


def _PrimFor(item, usdviewApi):
    """
    The prim the menu is about: the one right-clicked, else the focus
    prim (right-clicking the empty hierarchy below the rows).
    """
    prim = getattr(item, "prim", None) if item is not None else None
    if prim is None or not prim.IsValid():
        prim = usdviewApi.prim
    return prim


def AppendArcMenu(menu, usdviewApi, undoStack, prim):
    """
    Hang the arc flows off `menu` as an "Add Composition Arc" submenu.

    Parented to `menu` explicitly for the reason layerOpinionsUI gives:
    QMenu.addMenu(title) does not transfer ownership, and the submenu is
    collected before it opens.
    """
    import compositionArcsUI
    try:
        import panelIcons
        icon = panelIcons.Icon("arcAdd")
    except Exception:
        icon = None

    def _OnAuthored(edit, warnings, label):
        if edit is not None and undoStack is not None:
            undoStack.Push(edit)
        message = ("%s  (%s)" % (label, "; ".join(warnings)) if warnings
                   else label)
        try:
            usdviewApi.PrintStatus(message)
        except Exception:
            pass

    menu.addSeparator()
    submenu = QtWidgets.QMenu("Add Composition Arc", menu)
    if icon is not None and not icon.isNull():
        submenu.setIcon(icon)
    menu.addMenu(submenu)
    compositionArcsUI.PopulateArcMenu(
        submenu, usdviewApi, prim, None, _OnAuthored,
        getattr(usdviewApi, "qMainWindow", None))
    return submenu
