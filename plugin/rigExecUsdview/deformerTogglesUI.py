#
# RigExec usdview plugin: turn individual deformers off and watch the mesh.
#
# WHY THIS EXISTS
#
# "The face breaks when I move the head" is a sentence about a hundred
# deformers at once. Numbers can say a mesh is 2.8 units from rigid, but
# not WHICH mover put it there, and bisecting by hand means editing the
# asset. This panel does the bisect in the viewport: every mover that
# writes a mesh, grouped by the mesh it writes, with a checkbox.
#
# Toggling writes `inputs:enabled` into the SESSION layer, so the asset on
# disk is never touched and closing usdview throws the experiment away.
# That is also why there is a Reset: a session layer is invisible in the
# outliner and it would otherwise be easy to leave a deformer off and
# spend an hour wondering why the rig looks wrong.
#
# SOLO is the fast way in. Picking one mover disables every OTHER mover on
# the same mesh, so the viewport shows that deformer's contribution alone.
# Alt-clicking the name does the same thing without hunting for the
# button.
#
import os
import sys

from pxr import Sdf, Usd
from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets

# The attribute every mover honours. Absent means enabled.
_ENABLED = "inputs:enabled"
_MOVES = "rigExec:moves"


def _MeshOf(target):
    """The mesh a mover writes, or None if it writes something else.

    Movers also drive scalars (inputs:defaultWeight, face:driverRx) and
    matrices. Those are rig plumbing, not deformers, and listing them
    would bury the twenty that matter under six hundred that do not.
    """
    text = str(target)
    if not text.endswith(".points"):
        return None
    return text[: -len(".points")]


class DeformerTogglesPanel(QtWidgets.QDialog):
    """Every deformer on the stage, grouped by mesh, with a checkbox."""

    _instance = None

    @classmethod
    def GetInstance(cls, usdviewApi):
        if cls._instance is None:
            cls._instance = cls(usdviewApi)
        else:
            cls._instance._api = usdviewApi
        return cls._instance

    def __init__(self, usdviewApi, parent=None):
        super(DeformerTogglesPanel, self).__init__(
            parent or usdviewApi.qMainWindow)
        self._api = usdviewApi
        self._loading = False
        self.setWindowTitle("Deformer Toggles")
        self.resize(560, 720)

        layout = QtWidgets.QVBoxLayout(self)

        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel("Find:"))
        self._search = QtWidgets.QLineEdit()
        self._search.setPlaceholderText("mover, type or mesh, e.g. wire")
        self._search.textChanged.connect(lambda _t: self._ApplyFilter())
        row.addWidget(self._search, 1)
        layout.addLayout(row)

        self._tree = QtWidgets.QTreeWidget()
        self._tree.setHeaderLabels(["Deformer", "Type"])
        self._tree.setColumnWidth(0, 330)
        self._tree.setAlternatingRowColors(True)
        self._tree.itemChanged.connect(self._OnItemChanged)
        self._tree.itemClicked.connect(self._OnItemClicked)
        layout.addWidget(self._tree, 1)

        buttons = QtWidgets.QHBoxLayout()
        for label, slot in (("All On", lambda: self._SetAll(True)),
                            ("All Off", lambda: self._SetAll(False)),
                            ("Invert", self._Invert),
                            ("Solo Selected", self._SoloSelected),
                            ("Reset (clear session)", self._Reset),
                            ("Refresh", self.Rebuild)):
            b = QtWidgets.QPushButton(label)
            b.clicked.connect(slot)
            buttons.addWidget(b)
        layout.addLayout(buttons)

        self._status = QtWidgets.QLabel("")
        self._status.setWordWrap(True)
        layout.addWidget(self._status)

    # ---------------------------------------------------------------- data

    def _Stage(self):
        return self._api.stage

    def _SessionTarget(self):
        """Edits go to the session layer and nowhere else."""
        stage = self._Stage()
        return Usd.EditContext(stage, stage.GetSessionLayer())

    def _Deformers(self):
        """(mesh, prim) for every mover that writes mesh points."""
        out = {}
        stage = self._Stage()
        if not stage:
            return out
        for prim in stage.Traverse():
            rel = prim.GetRelationship(_MOVES)
            if not rel:
                continue
            for target in rel.GetTargets():
                mesh = _MeshOf(target)
                if mesh:
                    out.setdefault(mesh, []).append(prim)
                    break
        return out

    def Rebuild(self):
        self._loading = True
        try:
            self._tree.clear()
            groups = self._Deformers()
            total = 0
            for mesh in sorted(groups):
                parent = QtWidgets.QTreeWidgetItem(
                    self._tree, [mesh.rsplit("/", 1)[-1], ""])
                parent.setFirstColumnSpanned(True)
                parent.setData(0, QtCore.Qt.UserRole, "")
                font = parent.font(0)
                font.setBold(True)
                parent.setFont(0, font)
                for prim in sorted(groups[mesh], key=lambda p: p.GetName()):
                    item = QtWidgets.QTreeWidgetItem(
                        parent, [prim.GetName(), str(prim.GetTypeName())])
                    item.setData(0, QtCore.Qt.UserRole, str(prim.GetPath()))
                    item.setFlags(item.flags() | QtCore.Qt.ItemIsUserCheckable)
                    item.setCheckState(
                        0, QtCore.Qt.Checked if self._IsEnabled(prim)
                        else QtCore.Qt.Unchecked)
                    total += 1
                parent.setExpanded(True)
            self._status.setText(
                "%d deformers on %d meshes. Toggles write inputs:enabled to "
                "the SESSION layer -- nothing on disk changes, and Reset "
                "clears every one." % (total, len(groups)))
        finally:
            self._loading = False
        self._ApplyFilter()

    def _IsEnabled(self, prim):
        attr = prim.GetAttribute(_ENABLED)
        if not attr or not attr.IsValid():
            return True
        value = attr.Get()
        return True if value is None else bool(value)

    def _SetEnabled(self, prim, on):
        with self._SessionTarget():
            attr = prim.GetAttribute(_ENABLED)
            if not attr or not attr.IsValid():
                attr = prim.CreateAttribute(_ENABLED, Sdf.ValueTypeNames.Bool)
            attr.Set(bool(on))

    # -------------------------------------------------------------- events

    def _OnItemChanged(self, item, column):
        if self._loading or column != 0:
            return
        path = item.data(0, QtCore.Qt.UserRole)
        if not path:
            return
        prim = self._Stage().GetPrimAtPath(path)
        if not prim:
            return
        self._SetEnabled(prim, item.checkState(0) == QtCore.Qt.Checked)
        self._Redraw()

    def _OnItemClicked(self, item, column):
        # Alt-click solos, which is the gesture this panel is really for.
        if QtWidgets.QApplication.keyboardModifiers() & QtCore.Qt.AltModifier:
            self._Solo(item)

    def _Solo(self, item):
        path = item.data(0, QtCore.Qt.UserRole)
        if not path:
            return
        parent = item.parent()
        if parent is None:
            return
        self._loading = True
        try:
            for i in range(parent.childCount()):
                sibling = parent.child(i)
                keep = sibling is item
                sp = sibling.data(0, QtCore.Qt.UserRole)
                prim = self._Stage().GetPrimAtPath(sp) if sp else None
                if prim:
                    self._SetEnabled(prim, keep)
                sibling.setCheckState(
                    0, QtCore.Qt.Checked if keep else QtCore.Qt.Unchecked)
        finally:
            self._loading = False
        self._Redraw()

    def _SoloSelected(self):
        items = self._tree.selectedItems()
        if items:
            self._Solo(items[0])

    def _Walk(self):
        for i in range(self._tree.topLevelItemCount()):
            group = self._tree.topLevelItem(i)
            for j in range(group.childCount()):
                yield group.child(j)

    def _SetAll(self, on):
        self._loading = True
        try:
            for item in self._Walk():
                if item.isHidden():
                    continue
                path = item.data(0, QtCore.Qt.UserRole)
                prim = self._Stage().GetPrimAtPath(path) if path else None
                if prim:
                    self._SetEnabled(prim, on)
                item.setCheckState(
                    0, QtCore.Qt.Checked if on else QtCore.Qt.Unchecked)
        finally:
            self._loading = False
        self._Redraw()

    def _Invert(self):
        self._loading = True
        try:
            for item in self._Walk():
                if item.isHidden():
                    continue
                on = item.checkState(0) != QtCore.Qt.Checked
                path = item.data(0, QtCore.Qt.UserRole)
                prim = self._Stage().GetPrimAtPath(path) if path else None
                if prim:
                    self._SetEnabled(prim, on)
                item.setCheckState(
                    0, QtCore.Qt.Checked if on else QtCore.Qt.Unchecked)
        finally:
            self._loading = False
        self._Redraw()

    def _Reset(self):
        """Clear every enabled opinion this panel wrote.

        Clearing rather than setting True: an asset that ships a deformer
        disabled must go back to disabled, not get switched on by a reset.
        """
        stage = self._Stage()
        session = stage.GetSessionLayer()
        for item in self._Walk():
            path = item.data(0, QtCore.Qt.UserRole)
            if not path:
                continue
            # Remove the property spec outright. Clearing the value would
            # leave an empty spec behind, which still counts as an
            # opinion and would shadow whatever the asset says.
            primSpec = session.GetPrimAtPath(Sdf.Path(path))
            if primSpec is not None and _ENABLED in primSpec.properties:
                del primSpec.properties[_ENABLED]
        self._Redraw()
        self.Rebuild()

    def _ApplyFilter(self):
        text = self._search.text().strip().lower()
        for i in range(self._tree.topLevelItemCount()):
            group = self._tree.topLevelItem(i)
            shown = 0
            for j in range(group.childCount()):
                item = group.child(j)
                hay = "%s %s %s" % (item.text(0), item.text(1), group.text(0))
                hit = (not text) or text in hay.lower()
                item.setHidden(not hit)
                shown += 1 if hit else 0
            group.setHidden(shown == 0)

    def _Redraw(self):
        # A session-layer edit already notifies the stage; this only makes
        # the viewport repaint without waiting for the next interaction.
        try:
            self._api.UpdateViewport()
        except Exception:
            view = getattr(self._api, "stageView", None)
            if view is not None:
                view.updateGL()


def OpenDeformerTogglesPanel(usdviewApi):
    panel = DeformerTogglesPanel.GetInstance(usdviewApi)
    panel.Rebuild()
    panel.show()
    panel.raise_()
    panel.activateWindow()
    return panel
