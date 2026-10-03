"""Live picker, TouchPose and deformation check for the Shion rig stage.

Run testusdview with this script and examples/2d/bust_dd_b/bust_dd_b_rig.usda.
Set RIGEXEC_SHION_REVIEW_DIR to retain picker and viewport captures.
"""

import os
from pathlib import Path

import numpy as np


def testUsdviewInputFunction(appController):
    import pickerUI
    import touchPoseUI

    appController._processEvents()
    api = appController._usdviewApi
    stage = api.stage
    stage.SetEditTarget(stage.GetSessionLayer())
    appController._dataModel.viewSettings.displayGuide = False
    panel = pickerUI.OpenPickerPanel(api)
    appController._processEvents()
    picker = panel._picker
    assert picker.name == "Shion" and picker.coverage()[1] == 0
    assert len(panel._views) == 4
    selection = appController._dataModel.selection
    for name in ("Mouth", "Hand_R", "Index_R_0"):
        button = next(b for b in picker.buttons if b.id == name)
        view = next(v for v in panel._views if v._panel.id == button.parent)
        view.picked.emit([button], "replace")
        appController._processEvents()
        assert [str(p.GetPath()) for p in selection.getPrims()] == button.targets

    controller = touchPoseUI.TouchPoseController.GetInstance(api)
    controller.Load()
    assert controller.model.mesh_path == "/Shion/Geom/Character"
    assert controller.SetActive(True)
    model = controller.model
    regions, covered, faces = model.Coverage()
    assert covered == faces and regions > 60
    for xy, expected in (
        ((0, -28), "Torso_ctl"),
        ((28, -30), "Elbow_R_ctl"),
        ((0, -4.5), "FaceAngle_ctl"),
        ((-31, -19), "Hand_L_ctl"),
    ):
        region = model.RegionAt((xy[0], xy[1], 100), (0, 0, -1))
        assert region is not None and region.control.endswith(expected), (
            xy,
            region,
            expected,
        )
        controller.Select(region)
        appController._processEvents()
        assert [str(p.GetPath()) for p in selection.getPrims()] == [region.control]

    before = np.array(model.points)
    mouth = stage.GetAttributeAtPath(
        "/Shion/Rig/Controls/Body_ctl/Neck_ctl/Head_ctl/Mouth_ctl.avars:ty"
    )
    mouth.Set(-0.6)
    appController._processEvents()
    model.SyncPose(api.frame, force=True)
    assert np.max(np.abs(np.array(model.points) - before)) > 0.1, (
        "Live jaw edit did not reach TouchPose"
    )
    mouth.Set(0.0)
    appController._processEvents()
    model.SyncPose(api.frame, force=True)
    assert np.max(np.abs(np.array(model.points) - before)) < 0.002
    controller.SetActive(False)
    selection.clearPrims()
    appController._processEvents()
    out = os.environ.get("RIGEXEC_SHION_REVIEW_DIR")
    if out:
        Path(out).mkdir(parents=True, exist_ok=True)
        panel.grab().save(str(Path(out) / "picker.png"))
        appController._stageView.grabFramebuffer().save(str(Path(out) / "viewport.png"))
    panel.close()
    print(
        "RIGEXEC_SHION_VIEWER_OK: picker selection, whole-character TouchPose, live jaw update"
    )
