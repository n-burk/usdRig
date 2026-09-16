#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# at the root of this repository.
#
"""RigExec -> Profiler: where the open rig's time goes, and on how many
threads.

The panel face of `profilerModel`, which is also the CLI
(`tools/rigexec_schedule.py`). One measurement behind both, so the number
you read here is the number you can reproduce on the command line and
diff against a later run.

WHY IT RUNS ON DEMAND AND NOT CONTINUOUSLY. Measuring costs a compile and
a few hundred evaluations, and the `author` workload deliberately dirties
the epoch -- so doing that on every selection change would make usdview
feel exactly as slow as the thing being measured. The button is the point.

The panel opens a SECOND stage over the same layers, for the same reason
the Execution Stack panel does: usdview hands out a stage it holds in a
UsdStageCache and the `rigexec.Rig` binding refuses it. Reads compose
identically, and the measurement authors nothing that outlives it -- the
avar it drives is restored and the interactive overrides are cleared.
"""
import os
import sys

from pxr import Usd
from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets

try:
    import profilerModel
except ImportError:  # pragma: no cover - plugin path, not test path
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import profilerModel


# Same twin-stage cache and the same reason as execStackUI's.
_TWIN = {}


def _CompilableStage(stage):
    root = stage.GetRootLayer()
    key = root.identifier
    twin = _TWIN.get(key)
    if twin is None or twin.GetRootLayer() != root:
        twin = Usd.Stage.Open(root, stage.GetSessionLayer())
        _TWIN[key] = twin
    return twin


class ProfilerPanel(QtWidgets.QDialog):

    _instance = None

    @classmethod
    def GetInstance(cls, usdviewApi):
        if cls._instance is None:
            cls._instance = cls(usdviewApi)
        else:
            cls._instance._api = usdviewApi
        return cls._instance

    def __init__(self, usdviewApi, parent=None):
        super(ProfilerPanel, self).__init__(
            parent or usdviewApi.qMainWindow)
        self._api = usdviewApi
        self._report = None
        self.setWindowTitle("RigExec Profiler")
        self.resize(760, 680)

        layout = QtWidgets.QVBoxLayout(self)

        controls = QtWidgets.QHBoxLayout()
        controls.addWidget(QtWidgets.QLabel("Rig root:"))
        self._rootBox = QtWidgets.QComboBox()
        self._rootBox.setMinimumWidth(200)
        controls.addWidget(self._rootBox)

        controls.addWidget(QtWidgets.QLabel("Samples:"))
        self._samples = QtWidgets.QSpinBox()
        self._samples.setRange(5, 500)
        self._samples.setValue(30)
        controls.addWidget(self._samples)

        self._runButton = QtWidgets.QPushButton("Measure")
        self._runButton.clicked.connect(self.Measure)
        controls.addWidget(self._runButton)

        self._traceButton = QtWidgets.QPushButton("Save trace...")
        self._traceButton.setEnabled(False)
        self._traceButton.clicked.connect(self._SaveTrace)
        controls.addWidget(self._traceButton)

        self._jsonButton = QtWidgets.QPushButton("Save JSON...")
        self._jsonButton.setEnabled(False)
        self._jsonButton.clicked.connect(self._SaveJson)
        controls.addWidget(self._jsonButton)

        controls.addStretch(1)
        layout.addLayout(controls)

        self._text = QtWidgets.QPlainTextEdit()
        self._text.setReadOnly(True)
        self._text.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        font = QtGui.QFont("Consolas")
        font.setStyleHint(QtGui.QFont.Monospace)
        font.setPointSize(9)
        self._text.setFont(font)
        layout.addWidget(self._text, 1)

        self._status = QtWidgets.QLabel("")
        layout.addWidget(self._status)

        self.Rebuild()

    # -- population ------------------------------------------------------

    def Rebuild(self):
        """Refill the rig-root chooser from the open stage."""
        stage = self._api.dataModel.stage if self._api else None
        self._rootBox.blockSignals(True)
        self._rootBox.clear()
        roots = profilerModel.rig_roots(stage) if stage else []
        for root in roots:
            self._rootBox.addItem(root)
        self._rootBox.blockSignals(False)
        if not roots:
            self._text.setPlainText(
                "No RigExecRoot on this stage, so there is no schedule to\n"
                "measure. Open a rig and press Measure.")
            self._runButton.setEnabled(False)
        else:
            self._runButton.setEnabled(True)
            if not self._report:
                self._text.setPlainText(
                    "Press Measure.\n\n"
                    "It compiles the rig and times three workloads:\n"
                    "  replay  evaluate again with nothing changed\n"
                    "  drag    interactive overrides, what a manipulator does\n"
                    "  author  set an avar on the stage and evaluate\n\n"
                    "`author` is usually an order of magnitude above `drag`\n"
                    "because an authored edit invalidates the compiled epoch\n"
                    "and the next evaluate pays to rebuild it. That is the\n"
                    "single most common way to measure a rig wrongly, so the\n"
                    "three are reported apart.\n\n"
                    "Takes a few seconds and briefly makes the viewport\n"
                    "unresponsive: it is running the rig as hard as it can.")

    def Measure(self):
        stage = self._api.dataModel.stage if self._api else None
        if stage is None or self._rootBox.count() == 0:
            return
        root = self._rootBox.currentText()
        self._status.setText("Measuring %s..." % root)
        self._runButton.setEnabled(False)
        QtWidgets.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
        QtWidgets.QApplication.processEvents()
        try:
            report, rig = profilerModel.build_report(
                _CompilableStage(stage), root,
                samples=self._samples.value())
        except Exception as error:  # a bad rig must not take usdview with it
            self._text.setPlainText("Measurement failed:\n\n%s" % error)
            self._status.setText("")
            self._report = None
            self._traceButton.setEnabled(False)
            self._jsonButton.setEnabled(False)
            return
        finally:
            QtWidgets.QApplication.restoreOverrideCursor()
            self._runButton.setEnabled(True)

        self._report = report
        self._rig = rig
        self._text.setPlainText(profilerModel.render(report, 12))
        self._traceButton.setEnabled(True)
        self._jsonButton.setEnabled(True)
        cost = report["cost"]
        self._status.setText(
            "drag %.2f ms  |  replay %.2f ms  |  author %.0f ms  |  "
            "compile %.0f ms"
            % (cost["drag"]["ms"]["best"], cost["replay"]["ms"]["best"],
               cost["author"]["ms"]["best"], report["compile"]["ms"]))

    # -- saving ----------------------------------------------------------

    def _SaveTrace(self):
        if not self._report:
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save Chrome trace", "rigexec.trace.json",
            "Trace JSON (*.json)")
        if not path:
            return
        profilerModel.write_trace(self._rig,
                                  self._report["driven"]["control"],
                                  self._report["driven"]["avar"], path)
        self._status.setText("Wrote %s -- open it at ui.perfetto.dev" % path)

    def _SaveJson(self):
        if not self._report:
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save report", "rigexec_schedule.json", "JSON (*.json)")
        if not path:
            return
        import json
        with open(path, "w") as fp:
            json.dump(self._report, fp, indent=2, sort_keys=True)
        self._status.setText("Wrote %s -- diff it against a later run" % path)


def OpenProfilerPanel(usdviewApi):
    panel = ProfilerPanel.GetInstance(usdviewApi)
    panel.Rebuild()
    panel.show()
    panel.raise_()
    panel.activateWindow()
    return panel
