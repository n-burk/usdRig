#!/usr/bin/env python
"""Render Pip (or any flat 2D RigExec stage) through Storm, offscreen.

    source bin/_env.sh
    RIG=$(pwd) "$PY" examples/2d/rubberhose/render_preview.py \
        examples/2d/rubberhose/rubberhose_anim.usda OUT_DIR \
        --frames 1,13,25 [--guides 0.85] [--size 1920x1080] [--ss 2]
        [--center 0,5.0] [--height 11.6] [--range 1:120:1]

The rig is evaluated LIVE by the RigExec imaging plugin: nothing is baked.
Pass 1 draws /Pip/Geom unlit (flat displayColor, the 2D look); pass 2, when
--guides is given, draws the rig's guides (joints, controls) lit, over the
top at that opacity. Both are composited on a warm paper gradient.
"""
import argparse
import os
import sys

RIG = os.environ.get("RIG") or os.path.abspath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
sys.path.insert(0, os.path.join(RIG, "docs"))
import render_media as rm  # noqa: E402  (sets PXR_PLUGINPATH_NAME)
from PIL import Image  # noqa: E402
from pxr import Gf, Usd, UsdImagingGL  # noqa: E402

BG_TOP = (250, 238, 212)
BG_BOTTOM = (236, 208, 160)


def ortho_camera(center, height, aspect, z=50.0):
    cam = Gf.Camera()
    cam.projection = Gf.Camera.Orthographic
    cam.verticalAperture = height * 10.0
    cam.horizontalAperture = height * 10.0 * aspect
    cam.clippingRange = Gf.Range1f(1.0, 200.0)
    cam.transform = Gf.Matrix4d().SetTranslate(Gf.Vec3d(center[0], center[1], z))
    return cam


def draw(vp, stage, time, root, flat):
    from OpenGL import GL
    params = UsdImagingGL.RenderParams()
    params.frame = Usd.TimeCode(time)
    params.complexity = 1.0
    params.enableLighting = not flat
    params.enableSceneMaterials = False
    params.enableSceneLights = False
    params.gammaCorrectColors = False
    params.enableSampleAlphaToCoverage = True
    params.clearColor = Gf.Vec4f(0, 0, 0, 0)
    params.cullStyle = UsdImagingGL.CullStyle.CULL_STYLE_NOTHING
    params.drawMode = UsdImagingGL.DrawMode.DRAW_SHADED_SMOOTH
    params.showGuides = not flat
    params.showProxy = True
    params.showRender = False
    params.highlight = False
    vp.fbo.bind()
    GL.glViewport(0, 0, vp.width, vp.height)
    GL.glEnable(GL.GL_DEPTH_TEST)
    GL.glDepthFunc(GL.GL_LESS)
    GL.glDepthMask(GL.GL_TRUE)
    GL.glEnable(GL.GL_BLEND)
    GL.glBlendFunc(GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA)
    GL.glClearColor(0, 0, 0, 0)
    GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
    prim = stage.GetPrimAtPath(root)
    vp.engine.Render(prim, params)
    for _ in range(16):
        if vp.engine.IsConverged():
            break
        vp.engine.Render(prim, params)
    vp.fbo.bind()
    return rm._to_pil(vp.fbo.toImage())


def main(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("stage")
    ap.add_argument("out")
    ap.add_argument("--frames", default=None)
    ap.add_argument("--range", default=None, help="start:end:step")
    ap.add_argument("--guides", type=float, default=0.0)
    ap.add_argument("--size", default="1920x1080")
    ap.add_argument("--ss", type=int, default=2)
    ap.add_argument("--center", default="0,5.05")
    ap.add_argument("--height", type=float, default=11.8)
    ap.add_argument("--geom", default="/Pip/Geom")
    ap.add_argument("--rig", default="/Pip/Rig")
    ap.add_argument("--prefix", default="")
    args = ap.parse_args(argv)

    w, h = [int(v) for v in args.size.split("x")]
    ss = max(1, args.ss)
    if args.range:
        a, b, s = [float(v) for v in args.range.split(":")]
        frames = []
        f = a
        while f <= b + 1e-6:
            frames.append(round(f, 4))
            f += s
    else:
        frames = [float(v) for v in (args.frames or "1").split(",")]
    center = [float(v) for v in args.center.split(",")]
    os.makedirs(args.out, exist_ok=True)

    vp = rm.Viewport(w * ss, h * ss)
    stage = Usd.Stage.Open(args.stage)
    vp.set_camera(ortho_camera(center, args.height, w / float(h)))
    bg = rm._gradient(w * ss, h * ss, BG_TOP, BG_BOTTOM).convert("RGBA")
    for i, f in enumerate(frames):
        img = bg.copy()
        img.alpha_composite(draw(vp, stage, f, args.geom, True))
        if args.guides > 0:
            g = draw(vp, stage, f, args.rig, False)
            if args.guides < 1.0:
                alpha = g.getchannel("A").point(lambda v: int(v * args.guides))
                g.putalpha(alpha)
            img.alpha_composite(g)
        img = img.convert("RGB")
        if ss > 1:
            img = img.resize((w, h), Image.LANCZOS)
        name = "%s%04d.png" % (args.prefix, i) if args.range else \
            "%sf%06.1f.png" % (args.prefix, f)
        img.save(os.path.join(args.out, name))
        print("wrote", name, flush=True)
    vp.close()


if __name__ == "__main__":
    main(sys.argv[1:])
