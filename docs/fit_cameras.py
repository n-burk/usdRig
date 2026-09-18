#!/usr/bin/env python
"""Author one 3/4 MainCam per docs example -- the camera the GIF uses.

Usage: python docs/fit_cameras.py [example.usda ...]

The camera is not computed here: ``render_media.prepare_stage`` sizes the
guides, finds the drivers, applies any per-example angle override and
fits the frame exactly as the GIF renderer does, and
``render_media.camera_ops`` turns that one camera into xform ops. Writing
it into the stage is all this script adds, so the MainCam an artist opens
the example with and the picture on the operator's page are the same view
rather than two views that drifted apart.

Re-runnable: an existing MainCam block is replaced in place.
"""
import glob
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# render_media owns the camera: bake is still exported from there for
# anything that wants an evaluated stage, but nothing here needs it.
from render_media import RIG, camera_ops, prepare_stage

from pxr import Gf, Usd, UsdGeom  # noqa: E402

CAM_TEMPLATE = '''    def Camera "MainCam"
    {
        float2 clippingRange = (%s, %s)
        float focalLength = %s
        float horizontalAperture = %s
        float verticalAperture = %s
        double3 xformOp:rotateXYZ = (%s, %s, %s)
        double3 xformOp:translate = (%s, %s, %s)
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateXYZ"]
    }
'''

CAM_HEAD = '    def Camera "MainCam"'
CAM_OPEN = "    {"
CAM_CLOSE = "    }"


def camera_block_span(text):
    """(start, end) character span of the MainCam block, or None.

    A line scan and not a regular expression. The expression this replaced
    was ``def Camera "MainCam"\\n    \\{\\n(?:.*\\n)*?    \\}\\n`` compiled
    with ``re.S``, which makes ``.`` match a newline too: the lazy group
    then ran past the camera's own closing brace to the LAST
    four-space-indented ``}`` in the file, and the "replacement" deleted
    every sibling prim authored after the camera. It only ever looked
    right because MainCam happens to be the last prim in each example --
    a property nobody promised and nothing checks.
    """
    lines = text.splitlines(True)
    offset = 0
    for index, line in enumerate(lines):
        if line.rstrip("\r\n") != CAM_HEAD:
            offset += len(line)
            continue
        cursor = offset + len(line)
        if (index + 1 >= len(lines) or
                lines[index + 1].rstrip("\r\n") != CAM_OPEN):
            raise ValueError("MainCam is not followed by an opening brace")
        cursor += len(lines[index + 1])
        for tail in lines[index + 2:]:
            cursor += len(tail)
            # The first brace at the prim's own indent closes the prim: a
            # nested block inside it is indented deeper, so this stops at
            # the camera and never walks on into the next sibling.
            if tail.rstrip("\r\n") == CAM_CLOSE:
                return offset, cursor
        raise ValueError("MainCam block is never closed")
    return None


def _check_ops(camera, translate, rotate):
    """The authored ops really are the fitted camera, or this raises.

    Op composition order is easy to get backwards and the mistake is
    invisible until a reader opens the example and finds the asset behind
    the camera, so it is asserted rather than reasoned about.
    """
    stage = Usd.Stage.CreateInMemory()
    probe = UsdGeom.Xform.Define(stage, "/Probe")
    probe.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(*translate))
    probe.AddRotateXYZOp(UsdGeom.XformOp.PrecisionDouble).Set(
        Gf.Vec3d(*rotate))
    built = probe.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    for row in range(4):
        for column in range(4):
            if abs(built[row][column] - camera.transform[row][column]) > 1e-6:
                raise ValueError("authored xform ops do not rebuild the "
                                 "fitted camera: %s vs %s"
                                 % (built, camera.transform))


def fit_camera_block(src):
    scene = prepare_stage(src)
    camera = scene.camera
    # The yaw and pitch come from the SCENE, not from the module defaults:
    # a handful of examples are rendered from a different angle because
    # the global 3/4 view points along their motion instead of across it
    # (render_media.CAMERA_OVERRIDES), and MainCam has to be that camera.
    translate, rotate = camera_ops(camera, scene.yaw, scene.pitch)
    _check_ops(camera, translate, rotate)
    numbers = ["%.4g" % v for v in
               (camera.clippingRange.GetMin(), camera.clippingRange.GetMax(),
                camera.focalLength, camera.horizontalAperture,
                camera.verticalAperture) + tuple(rotate) + tuple(translate)]
    return CAM_TEMPLATE % tuple(numbers)


def inject_camera(src, block):
    with open(src) as stream:
        text = stream.read()
    span = camera_block_span(text)
    if span:
        return text[:span[0]] + block + text[span[1]:]
    stripped = text.rstrip("\n")
    # Raised, not asserted: `python -O` drops an assert, and the branch it
    # guards then writes a camera into a file whose last prim is still
    # open, producing a stage that no longer parses.
    if not stripped.endswith("}"):
        raise ValueError("unexpected stage ending in %s: the file does not "
                         "close with '}'" % src)
    return stripped[:-1].rstrip("\n") + "\n\n" + block + "}\n"


def main(argv):
    if len(argv) > 1:
        files = [a if os.path.isabs(a) else os.path.join(RIG, a)
                 for a in argv[1:]]
    else:
        files = sorted(glob.glob(os.path.join(RIG, "docs", "examples",
                                              "*.usda")))
    failures = 0
    for src in files:
        name = os.path.basename(src)
        try:
            block = fit_camera_block(src)
            updated = inject_camera(src, block)
            with open(src, "w", newline="\n") as stream:
                stream.write(updated)
        except Exception as failure:  # noqa: BLE001 - report, don't stop
            failures += 1
            print("FAIL %s: %s: %s" % (name, type(failure).__name__, failure))
        else:
            print("ok   %s" % name)
    print("%d/%d cameras fit" % (len(files) - failures, len(files)))
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
