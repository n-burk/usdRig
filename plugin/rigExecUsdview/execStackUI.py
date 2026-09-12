#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# at the root of this repository.
#
"""The rig's execution stack, as a filterable list.

Answers "what runs, in what order, and what does it write" against the
stage currently open in usdview. Ordering mistakes here are silent and
expensive: a mover chain applies in REVERSE add order, so a skin mover
authored last runs FIRST and deforms with pre-pose joint transforms. The
joints all read correct while the mesh is wrong by centimetres, and
nothing in the prim browser hints at it.

Two sections, because RigExec evaluates in two stages:

  POSE PHASE   aggregate solvers (FK chains, two-bone IK, blends, twist
               distributions, ribbons, spline IK). They claim joints
               through `rigExec:joints` and are scheduled by DEPENDENCY,
               not in a line -- so they are grouped by type with what each
               claims, rather than numbered. Numbering them would imply an
               order that does not exist.
  EXECUTION    `Rig.mover_order`, which IS a strict order (reverse-sibling
               post-order, spec section 4.2). This is the list to read
               when something deforms wrongly.

The footer checks the invariant that matters: every deformer must run
after every constraint, or a deformer reading final joint transforms sees
unposed frames.
"""
import os
import sys

from pxr import Usd
from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets


# Cache the twin so a repopulate does not reopen the stage on every
# selection change; keyed on the root layer's identity.
_TWIN = {}


def _CompilableStage(stage):
    """A stage `rigexec.Rig` will accept, over the same layers.

    usdview hands out a stage it holds in a `UsdStageCache`, and the
    Python binding for `rigexec.Rig` refuses it -- "needs a `__owner`
    capsule" -- because that wrapper does not own the stage. The panel
    only READS through this stage (solver list, compile, mover order), so
    a second stage opened over the same root and session layers gives the
    identical composition and the identical mover order, and any edit the
    animator makes lands in those same layers and is seen here.

    Without this the Execution Stack panel reported "Rig compile failed"
    in usdview for every rig, while the same rig compiled fine from a
    script.
    """
    root = stage.GetRootLayer()
    key = root.identifier
    twin = _TWIN.get(key)
    if twin is None or twin.GetRootLayer() != root:
        twin = Usd.Stage.Open(root, stage.GetSessionLayer())
        _TWIN[key] = twin
    return twin

# Schema-type tokens per filter. A type may appear in several groups -- a
# single-chain IK is legitimately both a constraint and IK -- so the
# filters overlap on purpose.
_GROUPS = {
    "All": None,
    "Deformers": ("SkinMover", "MatrixMover", "CurveMover", "LatticeMover",
                  "SurfaceMover", "SmoothMover", "BlendShapeMover",
                  "VolumeCorrectMover", "CurvenetMover",
                  "CurvenetAdjusterMover"),
    "Constraints": ("Constraint",),
    "Solvers": ("FkChain", "TwoBoneIk", "BlendPointFrames",
                "TwistDistribution", "Ribbon", "SplineIk"),
    "IK": ("TwoBoneIk", "SingleChainIk", "SplineIk"),
    "FK": ("FkChain",),
    "Weights": ("Weight",),
    "Movers only": ("Mover",),
}

_SOLVER_TYPES = ("RigExecFkChain", "RigExecTwoBoneIk",
                 "RigExecBlendPointFrames", "RigExecTwistDistribution",
                 "RigExecRibbon", "RigExecSplineIk")

_DEFORMER_TOKENS = ("SkinMover", "MatrixMover", "CurveMover",
                    "LatticeMover", "SurfaceMover", "SmoothMover",
                    "BlendShapeMover", "VolumeCorrectMover")


def _tail(path, keep=1):
    parts = str(path).split("/")
    return "/".join(parts[-keep:]) if len(parts) > keep else str(path)


class ExecStackPanel(QtWidgets.QDialog):
    """Dockable-ish dialog listing the execution stack of the open stage."""

    _instance = None

    @classmethod
    def GetInstance(cls, usdviewApi):
        if cls._instance is None:
            cls._instance = cls(usdviewApi)
        else:
            cls._instance._api = usdviewApi
        return cls._instance

    def __init__(self, usdviewApi, parent=None):
        super(ExecStackPanel, self).__init__(
            parent or usdviewApi.qMainWindow)
        self._api = usdviewApi
        self.setWindowTitle("Execution Stack")
        self.resize(920, 640)

        layout = QtWidgets.QVBoxLayout(self)

        controls = QtWidgets.QHBoxLayout()
        controls.addWidget(QtWidgets.QLabel("Rig root:"))
        self._rootBox = QtWidgets.QComboBox()
        self._rootBox.setMinimumWidth(220)
        self._rootBox.currentIndexChanged.connect(lambda _i: self.Rebuild())
        controls.addWidget(self._rootBox)

        controls.addWidget(QtWidgets.QLabel("Show:"))
        self._filterBox = QtWidgets.QComboBox()
        for name in ("All", "Deformers", "Constraints", "Solvers", "IK",
                     "FK", "Weights", "Movers only"):
            self._filterBox.addItem(name)
        self._filterBox.currentIndexChanged.connect(
            lambda _i: self._Repopulate())
        controls.addWidget(self._filterBox)

        controls.addWidget(QtWidgets.QLabel("Find:"))
        self._search = QtWidgets.QLineEdit()
        self._search.setPlaceholderText("name, type or target, e.g. elbow")
        self._search.textChanged.connect(lambda _t: self._Repopulate())
        controls.addWidget(self._search, 1)

        refresh = QtWidgets.QPushButton("Refresh")
        refresh.clicked.connect(self.Rebuild)
        controls.addWidget(refresh)
        layout.addLayout(controls)

        self._tree = QtWidgets.QTreeWidget()
        self._tree.setHeaderLabels(["#", "Type", "Name", "Writes"])
        self._tree.setRootIsDecorated(True)
        self._tree.setAlternatingRowColors(True)
        self._tree.setColumnWidth(0, 44)
        self._tree.setColumnWidth(1, 210)
        self._tree.setColumnWidth(2, 250)
        self._tree.itemSelectionChanged.connect(self._OnSelect)
        layout.addWidget(self._tree, 1)

        self._status = QtWidgets.QLabel("")
        self._status.setWordWrap(True)
        layout.addWidget(self._status)

        self._solvers = []
        self._movers = []

    # -- data ------------------------------------------------------------

    def _Stage(self):
        model = getattr(self._api, "dataModel", None)
        return model.stage if model else None

    def _RigRoots(self, stage):
        return [str(p.GetPath()) for p in stage.Traverse()
                if p.GetTypeName() == "RigExecRoot"] if stage else []

    def Rebuild(self):
        """Recompile the rig and re-read its order. Safe to call often."""
        self._solvers, self._movers = [], []
        stage = self._Stage()
        if stage is None:
            self._status.setText("No stage open.")
            self._tree.clear()
            return

        roots = self._RigRoots(stage)
        current = self._rootBox.currentText()
        self._rootBox.blockSignals(True)
        self._rootBox.clear()
        for r in roots:
            self._rootBox.addItem(r)
        if current in roots:
            self._rootBox.setCurrentIndex(roots.index(current))
        self._rootBox.blockSignals(False)

        if not roots:
            self._status.setText(
                "No RigExecRoot on this stage, so there is no execution "
                "stack to show.")
            self._tree.clear()
            return

        root = self._rootBox.currentText() or roots[0]

        # Solvers come off the stage: they are scheduled by dependency, so
        # there is no order to read.
        for prim in stage.Traverse():
            t = prim.GetTypeName()
            if t not in _SOLVER_TYPES:
                continue
            if not str(prim.GetPath()).startswith(root):
                continue
            rel = prim.GetRelationship("rigExec:joints")
            claims = [str(x) for x in rel.GetTargets()] if rel else []
            self._solvers.append((t, prim.GetName(),
                                  str(prim.GetPath()), claims))
        self._solvers.sort()

        # The ordered part needs a compile, which can legitimately fail on
        # a rig mid-authoring -- report it rather than throwing into Qt.
        try:
            import rigexec
        except ImportError:
            self._status.setText(
                "The rigexec Python module is not importable, so the "
                "ordered mover list is unavailable. Solvers are still "
                "listed above.")
            self._Repopulate()
            return
        try:
            rig = rigexec.Rig(_CompilableStage(stage), root)
            rig.compile()
            order = rig.mover_order
            if callable(order):
                order = order()
            self._movers = list(order)
            self._compileError = None
        except Exception as exc:  # compile errors are expected while rigging
            self._movers = []
            self._compileError = str(exc).strip().splitlines()[-1]
        self._Repopulate()

    # -- view ------------------------------------------------------------

    def _Keep(self, schemaType, name, targets):
        tokens = _GROUPS.get(self._filterBox.currentText())
        if tokens is not None:
            if not any(tok.lower() in schemaType.lower() for tok in tokens):
                return False
        needle = self._search.text().strip().lower()
        if needle:
            haystack = " ".join([schemaType, name] +
                                [str(t) for t in targets]).lower()
            if needle not in haystack:
                return False
        return True

    def _Repopulate(self):
        self._tree.clear()
        mono = QtGui.QFont("Consolas")
        mono.setStyleHint(QtGui.QFont.Monospace)

        solverRoot = QtWidgets.QTreeWidgetItem(
            ["", "POSE PHASE", "solvers", "scheduled by dependency, not "
             "in a line"])
        self._tree.addTopLevelItem(solverRoot)
        shown_solvers = 0
        for t, name, path, claims in self._solvers:
            if not self._Keep(t, name, claims):
                continue
            item = QtWidgets.QTreeWidgetItem(
                ["", t.replace("RigExec", ""), name,
                 "claims %d: %s" % (len(claims),
                                    ", ".join(_tail(c) for c in claims[:6]))])
            item.setData(0, QtCore.Qt.UserRole, path)
            solverRoot.addChild(item)
            shown_solvers += 1
        solverRoot.setExpanded(True)

        moverRoot = QtWidgets.QTreeWidgetItem(
            ["", "EXECUTION ORDER", "movers", "strict order, first runs "
             "first"])
        self._tree.addTopLevelItem(moverRoot)
        shown_movers = 0
        for e in self._movers:
            t = e["type"]
            name = e["path"].rsplit("/", 1)[-1]
            targets = e.get("targets") or []
            if not self._Keep(t, name, targets):
                continue
            item = QtWidgets.QTreeWidgetItem(
                [str(e["ordinal"]), t.replace("RigExec", ""), name,
                 ", ".join(_tail(x) for x in targets[:3])])
            item.setData(0, QtCore.Qt.UserRole, e["path"])
            for col in range(4):
                item.setFont(col, mono)
            moverRoot.addChild(item)
            shown_movers += 1
        moverRoot.setExpanded(True)

        self._status.setText(self._StatusText(shown_solvers, shown_movers))

    def _StatusText(self, shown_solvers, shown_movers):
        if getattr(self, "_compileError", None):
            return ("Rig compile failed, so there is no mover order: %s"
                    % self._compileError)

        bits = ["%d of %d solvers, %d of %d movers shown"
                % (shown_solvers, len(self._solvers), shown_movers,
                   len(self._movers))]

        deform = [e for e in self._movers
                  if any(tok.lower() in e["type"].lower()
                         for tok in _DEFORMER_TOKENS)]
        constraints = [e for e in self._movers if "Constraint" in e["type"]]
        if deform and constraints:
            first = min(e["ordinal"] for e in deform)
            last = max(e["ordinal"] for e in constraints)
            if first > last:
                bits.append(
                    "Ordering ok: every deformer (first at %d) runs after "
                    "every constraint (last at %d), so a deformer reading "
                    "final joint transforms sees posed frames." % (first,
                                                                   last))
            else:
                bits.append(
                    "WARNING: a deformer runs at %d, BEFORE a constraint "
                    "at %d. A deformer reading final joint transforms will "
                    "skin with pre-pose frames." % (first, last))
        return "  ".join(bits)

    def _OnSelect(self):
        """Selecting a row selects the prim, so the usual panels follow."""
        items = self._tree.selectedItems()
        if not items:
            return
        path = items[0].data(0, QtCore.Qt.UserRole)
        stage = self._Stage()
        if not path or stage is None:
            return
        prim = stage.GetPrimAtPath(str(path).split(".")[0])
        if not prim or not prim.IsValid():
            return
        try:
            self._api.dataModel.selection.setPrim(prim)
        except Exception:
            pass


def OpenExecStackPanel(usdviewApi):
    panel = ExecStackPanel.GetInstance(usdviewApi)
    panel.Rebuild()
    panel.show()
    panel.raise_()
    panel.activateWindow()
    return panel
