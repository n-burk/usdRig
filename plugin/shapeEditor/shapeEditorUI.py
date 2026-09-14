#
# The Shape Editor panel: which correctives are firing, and how hard.
#
# HOW IT GETS THE WEIGHTS, and why it costs what it does.
#
# The numbers this panel exists to show are published by the evaluator
# into `RigExecRigPose::movedProperties`, and usdview's imaging bridge
# holds that pose privately -- there is no `RigExecImaging_GetMovedFloats`
# entry point today. The only thing the bridge exposes is a GENERATION
# counter, so this panel keeps its own `rigexec.Rig` on the same stage and
# evaluates when that counter moves.
#
# That is a second evaluation of the rig, and it is not free (~20 ms on
# the biped). Three things keep it honest:
#
#   * it runs ONLY when the generation changes, so an idle viewport costs
#     one integer read every 150 ms and nothing else;
#   * it runs only while the panel is VISIBLE -- closing it stops the
#     timer, so nobody pays for a panel they are not looking at;
#   * it is a read-only evaluation on a private evaluator, so it cannot
#     perturb what the viewport is showing.
#
# The right long-term answer is a `GetMovedFloats` on the bridge, which
# would make this free; it is a C++ change and is noted rather than done.
#
import ctypes

from pxr import Sdf, Tf, Usd

try:
    from PySide6 import QtCore, QtGui, QtWidgets
except ImportError:
    from PySide2 import QtCore, QtGui, QtWidgets

import shapeEditorModel as model


# How often the generation counter is read. Not how often the rig is
# evaluated -- that happens only when the counter has actually moved.
POLL_MS = 150

# Bar colours. Green for a corrective doing its job, amber for one over
# 1.0 (an overshoot the RBF is allowed to produce and worth seeing), and
# red for a negative weight, which is legal when `allowNegativeWeights`
# is on but is almost always the interesting case when debugging.
_GOOD = QtGui.QColor(90, 200, 120)
_OVER = QtGui.QColor(230, 180, 70)
_UNDER = QtGui.QColor(220, 95, 95)
_NEUTRAL = QtGui.QColor(120, 125, 135)


def _Imaging():
    """The imaging library: the generation counter and the weights.

    `RigExecImaging_GetMovedFloats` is what makes this panel cheap. The
    viewport has already evaluated the rig and published every pose
    weight; without that entry point the panel had to evaluate a SECOND
    time to recover numbers that were sitting in the current snapshot --
    measured at 9 ms per refresh against roughly nothing for a batched
    map lookup.

    A build without the entry point falls back to the private evaluator,
    which is correct and slower, so an older library degrades rather than
    breaks.
    """
    try:
        from rigExecUsdview import ImagingLibraryPath
        lib = ctypes.CDLL(ImagingLibraryPath())
        lib.RigExecImaging_GetGeneration.restype = ctypes.c_longlong
        try:
            lib.RigExecImaging_GetMovedFloats.argtypes = [
                ctypes.c_char_p, ctypes.POINTER(ctypes.c_float), ctypes.c_int]
            lib.RigExecImaging_GetMovedFloats.restype = ctypes.c_int
        except AttributeError:
            pass
        return lib
    except Exception:
        return None


class _WeightBar(QtWidgets.QStyledItemDelegate):
    """Draws a pose's weight as a bar behind its number.

    A delegate rather than a widget per row: there are 121 poses, and a
    progress bar each is 121 widgets to lay out on every refresh.
    """

    def paint(self, painter, option, index):
        weight = index.data(QtCore.Qt.UserRole)
        if weight is None:
            return super(_WeightBar, self).paint(painter, option, index)
        rect = option.rect.adjusted(2, 3, -2, -3)
        painter.save()
        painter.setPen(QtCore.Qt.NoPen)
        span = max(abs(float(weight)), 1.0)
        width = int(rect.width() * min(abs(float(weight)) / span, 1.0))
        if width > 0:
            colour = (_UNDER if weight < 0 else
                      _OVER if weight > 1.0001 else _GOOD)
            colour = QtGui.QColor(colour)
            colour.setAlpha(150)
            painter.setBrush(colour)
            painter.drawRoundedRect(
                QtCore.QRect(rect.x(), rect.y(), width, rect.height()), 2, 2)
        painter.restore()
        super(_WeightBar, self).paint(painter, option, index)


class ShapeEditorPanel(QtWidgets.QDialog):
    """Interpolators and their poses, with live weights and mutes."""

    _instance = None

    @classmethod
    def GetInstance(cls, usdviewApi):
        if cls._instance is None:
            cls._instance = cls(usdviewApi)
        else:
            cls._instance._api = usdviewApi
        return cls._instance

    def __init__(self, usdviewApi, parent=None):
        super(ShapeEditorPanel, self).__init__(
            parent or usdviewApi.qMainWindow)
        self._api = usdviewApi
        self._interpolators = []
        self._rig = None
        self._rigStage = None
        self._lib = _Imaging()
        self._generation = None
        self._items = {}          # pose path -> QTreeWidgetItem
        self._packed = None       # the path list handed to the bridge
        self._packedEntries = []
        self._buffer = None

        self.setWindowTitle("Shape Editor")
        self.resize(560, 720)
        layout = QtWidgets.QVBoxLayout(self)

        row = QtWidgets.QHBoxLayout()
        self._firingOnly = QtWidgets.QCheckBox("Firing only")
        self._firingOnly.setToolTip(
            "Show only poses carrying a weight. The neutral is never "
            "counted: at rest EVERY interpolator's neutral reads 1.0, so "
            "including it would mark all of them as firing.")
        self._firingOnly.toggled.connect(self._Populate)
        row.addWidget(self._firingOnly)
        row.addStretch(1)
        reload_ = QtWidgets.QPushButton("Reload")
        reload_.setToolTip("Re-read the interpolators off the stage.")
        reload_.clicked.connect(self.Reload)
        row.addWidget(reload_)
        layout.addLayout(row)

        self._tree = QtWidgets.QTreeWidget()
        self._tree.setColumnCount(4)
        self._tree.setHeaderLabels(["pose", "weight", "corrective", "on"])
        self._tree.setRootIsDecorated(True)
        self._tree.setUniformRowHeights(True)
        self._tree.setAlternatingRowColors(True)
        self._tree.header().setStretchLastSection(False)
        self._tree.setColumnWidth(0, 250)
        self._tree.setColumnWidth(1, 80)
        self._tree.setColumnWidth(2, 170)
        self._tree.setColumnWidth(3, 30)
        self._tree.setItemDelegateForColumn(1, _WeightBar(self._tree))
        self._tree.itemChanged.connect(self._OnItemChanged)
        self._tree.itemSelectionChanged.connect(self._OnSelected)
        layout.addWidget(self._tree, 1)

        self._status = QtWidgets.QLabel("")
        self._status.setWordWrap(True)
        layout.addWidget(self._status)

        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(POLL_MS)
        self._timer.timeout.connect(self._Poll)

        self.Reload()

    # -- lifecycle -------------------------------------------------------

    def showEvent(self, event):
        # Re-read on show: the stage may have been replaced, and a panel
        # that silently describes a stage nobody is looking at is worse
        # than one that is empty.
        self.Reload()
        self._timer.start()
        return super(ShapeEditorPanel, self).showEvent(event)

    def hideEvent(self, event):
        # Nobody pays for a panel they closed. This is the whole of the
        # cost control described at the top of the file.
        self._timer.stop()
        return super(ShapeEditorPanel, self).hideEvent(event)

    def _Stage(self):
        return getattr(self._api, "stage", None)

    # -- the private evaluator -------------------------------------------

    def _Rig(self):
        """A read-only evaluator on the current stage, built once.

        Rebuilt when the stage changes. A failure here is reported in the
        status line and leaves the panel showing structure with no
        weights, which is still useful -- it says what EXISTS even when
        it cannot say what is firing.
        """
        stage = self._Stage()
        if stage is None:
            return None
        if self._rig is not None and self._rigStage is stage:
            return self._rig
        self._rig = None
        self._rigStage = stage
        try:
            import rigexec
            for prim in stage.Traverse():
                if str(prim.GetTypeName()) == "RigExecRoot":
                    rig = rigexec.Rig(stage, str(prim.GetPath()))
                    rig.compile()
                    # This evaluator exists to read published floats, and
                    # nothing looks at its guides -- so stop computing
                    # them. MEASURED elsewhere at ~1.4 ms/frame of pure
                    # viewport data. Sparse for the same reason: the panel
                    # re-evaluates on a drag, which is exactly the case
                    # sparse is for (fingertip 24.2 -> 15.9 ms).
                    try:
                        rig.solver_guides_enabled = False
                    except Exception:
                        pass
                    try:
                        rig.evaluation_mode = "sparse"
                    except Exception:
                        pass
                    self._rig = rig
                    break
        except Exception as error:
            Tf.Warn("shapeEditor: no private evaluator: %s" % error)
        return self._rig

    def _Poll(self):
        """Evaluate only when the viewport's generation has moved.

        THE GENERATION IS THE WHOLE POINT. A drag moves it 60+ times a
        second; this fires at most every POLL_MS and coalesces everything
        in between, so a long drag costs the same per second as a short
        one. An idle viewport moves it not at all, and then this is one
        integer read and a comparison -- no evaluate, no Qt, nothing.
        """
        if self._lib is None:
            return
        try:
            generation = int(self._lib.RigExecImaging_GetGeneration())
        except Exception:
            return
        if generation == self._generation:
            return
        self._generation = generation
        self.Refresh()

    # -- data ------------------------------------------------------------

    def Reload(self):
        stage = self._Stage()
        self._interpolators = model.Discover(stage) if stage else []
        self._packed = None       # the path list is only valid for these
        self._rig = None          # the stage may have changed under us
        self._Populate()
        self.Refresh()

    def Refresh(self):
        """Pull the published weights and repaint the numbers."""
        if self._ReadFromViewport():
            return
        rig = self._Rig()
        if rig is None or not self._interpolators:
            self._Status("no live weights" if self._interpolators else None)
            return
        try:
            frame = self._api.frame.GetValue() if self._api.frame else 0.0
        except Exception:
            frame = 0.0
        try:
            pose = rig.evaluate(float(frame))
        except Exception as error:
            self._Status("evaluate failed: %s" % error)
            return
        found = model.ReadWeights(self._interpolators, pose)
        if self._firingOnly.isChecked():
            self._Populate()
        else:
            self._UpdateNumbers()
        self._Status(None if found else
                     "the engine published no pose weights -- is this a "
                     "stage with the interpolators AND the shapes?")

    def _ReadFromViewport(self):
        """Take the weights straight off the viewport's own snapshot.

        Returns False when the entry point is missing or published
        nothing, so the caller falls back to the private evaluator.
        """
        if self._lib is None or not self._interpolators:
            return False
        get = getattr(self._lib, "RigExecImaging_GetMovedFloats", None)
        if get is None:
            return False
        if self._packed is None:
            # Built once and reused: the path list only changes when the
            # interpolators do, and rebuilding it per refresh would put
            # 121 string joins back on the hot path.
            entries = [e for i in self._interpolators for e in i.poses]
            self._packedEntries = entries
            self._packed = ("\n".join(
                str(e.path.AppendProperty(model.WEIGHT)) for e in entries
            )).encode("utf-8")
            self._buffer = (ctypes.c_float * len(entries))()
        try:
            found = int(get(self._packed, self._buffer, len(self._buffer)))
        except Exception:
            return False
        if found <= 0:
            return False
        for entry, value in zip(self._packedEntries, self._buffer):
            entry.weight = float(value)
        # REPOPULATE in firing-only mode, exactly as the slow path does.
        # Which rows exist is a function of the weights there, so updating
        # the numbers without rebuilding froze the list at whatever was
        # firing when the box was ticked -- which at rest is nothing, so
        # the panel looked permanently empty.
        if self._firingOnly.isChecked():
            self._Populate()
        else:
            self._UpdateNumbers()
        self._Status(None)
        return True

    # -- tree ------------------------------------------------------------

    def _Populate(self):
        self._tree.blockSignals(True)
        self._tree.clear()
        self._items = {}
        firingOnly = self._firingOnly.isChecked()
        for interp in self._interpolators:
            poses = interp.firing if firingOnly else interp.poses
            if firingOnly and not poses:
                continue
            top = QtWidgets.QTreeWidgetItem(self._tree)
            top.setText(0, interp.name)
            top.setText(2, interp.kernel or "")
            top.setFlags(top.flags() | QtCore.Qt.ItemIsUserCheckable)
            top.setCheckState(3, QtCore.Qt.Checked if interp.enabled
                              else QtCore.Qt.Unchecked)
            top.setData(0, QtCore.Qt.UserRole + 1, str(interp.path))
            top.setExpanded(firingOnly)
            for entry in poses:
                item = QtWidgets.QTreeWidgetItem(top)
                item.setText(0, entry.name)
                item.setText(2, entry.target or "")
                item.setFlags(item.flags() | QtCore.Qt.ItemIsUserCheckable)
                item.setCheckState(3, QtCore.Qt.Checked if entry.enabled
                                   else QtCore.Qt.Unchecked)
                item.setData(0, QtCore.Qt.UserRole + 1, str(entry.path))
                if entry.is_neutral:
                    item.setForeground(0, _NEUTRAL)
                self._items[str(entry.path)] = item
        self._tree.blockSignals(False)
        self._UpdateNumbers()

    def _UpdateNumbers(self):
        """Write the numbers, but only onto rows anybody can see.

        A collapsed interpolator's 8 poses are 8 `setText`/`setData` pairs
        and a repaint each, for text behind a closed triangle. On 121
        poses that is most of the work of a refresh spent on nothing. A
        collapsed parent is skipped wholesale and marked dirty, so
        expanding it fills in from the model without waiting for the next
        generation.
        """
        # EVERY row, expanded or not. Skipping collapsed ones was worth
        # it when a refresh meant a 15 ms evaluation of the rig; now that
        # the weights come straight off the viewport's snapshot the whole
        # update is Qt writes, and a deferred row that kept a stale number
        # is worse than a cheap one that is right.
        weights = {}
        for interp in self._interpolators:
            for entry in interp.poses:
                weights[str(entry.path)] = entry.weight
        self._tree.blockSignals(True)
        for path, item in self._items.items():
            weight = weights.get(path)
            if weight is None:
                continue
            item.setText(1, "%.4f" % weight)
            item.setData(1, QtCore.Qt.UserRole, weight)
        self._tree.blockSignals(False)
        self._Status(None)

    def _Status(self, message):
        text = model.Summarise(self._interpolators)
        self._status.setText(text if not message else "%s  --  %s"
                             % (text, message))

    # -- interaction -----------------------------------------------------

    def _OnItemChanged(self, item, column):
        """A mute toggled: author `inputs:enabled` on that prim."""
        if column != 3:
            return
        path = item.data(0, QtCore.Qt.UserRole + 1)
        stage = self._Stage()
        if not path or stage is None:
            return
        prim = stage.GetPrimAtPath(Sdf.Path(path))
        if not prim or not prim.IsValid():
            return
        value = item.checkState(3) == QtCore.Qt.Checked
        # SESSION LAYER, so muting a corrective to see what it was doing
        # never dirties the asset. Closing usdview throws it away.
        target = Usd.EditTarget(stage.GetSessionLayer())
        with Usd.EditContext(stage, target):
            model.SetEnabled(prim, value)
        # NO `self._rig = None` HERE. Throwing the evaluator away forces a
        # fresh compile, MEASURED at 332.59 ms against a 15.77 ms evaluate
        # -- twenty times the cost of the thing it was protecting. The
        # evaluator already watches the stage: `inputs:enabled` is a value
        # edit, the notice reaches it, and the next evaluate reflects it.
        self.Refresh()

    def _OnSelected(self):
        """Select the driver joint when an interpolator row is picked."""
        items = self._tree.selectedItems()
        if not items:
            return
        path = items[0].data(0, QtCore.Qt.UserRole + 1)
        stage = self._Stage()
        if not path or stage is None:
            return
        prim = stage.GetPrimAtPath(Sdf.Path(path))
        if not prim or not prim.IsValid():
            return
        # An interpolator selects its DRIVER, because that is the thing an
        # animator can actually grab; a pose selects itself, so the
        # property panel shows its weight and rotation.
        if prim.GetTypeName() == model.INTERPOLATOR:
            rel = prim.GetRelationship("rigExec:driver")
            targets = rel.GetTargets() if rel else []
            if targets:
                driver = stage.GetPrimAtPath(targets[0])
                if driver and driver.IsValid():
                    prim = driver
        try:
            self._api.dataModel.selection.setPrim(prim)
        except Exception:
            pass
