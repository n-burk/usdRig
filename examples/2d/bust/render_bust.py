"""Offscreen renders of the 2D bust through Storm, the way usdview draws it.

    source bin/_env.sh
    RIG=$(pwd) "$PY" examples/2d/bust/render_bust.py <stage> <out_prefix> \
        --frames 1,30,60 [--textured] [--guides 1.0] [--wire] [--height 50] [--center 0,-8]

Each frame is TWO passes composited in PIL over a pastel gradient:

  art     the whole stage, UNLIT (enableLighting off), so every vertex
          colour lands on screen exactly as authored -- the flat look of a
          layered mesh deformation model. --textured instead uses a neutral
          camera light and sRGB display conversion for painted materials;
  guides  only the RigExecRoot subtree with showGuides on: joints,
          controls, solver guides and the lattice wires, lit, drawn OVER
          the art at the requested opacity.

The rig evaluates live in Hydra through the rigExecImaging plugin; nothing
is baked. Extra --sublayers compose over the stage in memory.
"""

import argparse
import os
import sys

RIG = os.environ.get("RIG") or os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, os.path.join(RIG, "docs"))

import render_media as rm  # noqa: E402
from PIL import Image  # noqa: E402

BG_TOP = (164, 196, 246)
BG_BOTTOM = (252, 222, 236)


def open_stage(path, sublayers=()):
    from pxr import Sdf, Usd

    if not sublayers:
        return Usd.Stage.Open(path)
    root = Sdf.Layer.CreateAnonymous("bust_root.usda")
    root.subLayerPaths = [os.path.abspath(p) for p in sublayers] + \
        [os.path.abspath(path)]
    return Usd.Stage.Open(root)


def ortho_camera(center, height, aspect):
    from pxr import Gf

    cam = Gf.Camera()
    cam.SetOrthographicFromAspectRatioAndSize(
        aspect, height, Gf.Camera.FOVVertical)
    cam.clippingRange = Gf.Range1f(1.0, 400.0)
    m = Gf.Matrix4d(1.0).SetTranslate(Gf.Vec3d(center[0], center[1], 100.0))
    cam.transform = m
    return cam


def render_pass(vp, stage, time, guides=False, wire=False, root=None,
                lit=False, textured=False):
    """One RGBA pass; art passes are unlit, the guide pass is lit."""
    from OpenGL import GL
    from pxr import Gf, Usd, UsdImagingGL

    params = UsdImagingGL.RenderParams()
    params.frame = Usd.TimeCode(time)
    params.complexity = 1.0
    params.showProxy = True
    params.showRender = False
    params.showGuides = bool(guides)
    params.gammaCorrectColors = False
    params.enableSampleAlphaToCoverage = True
    params.enableSceneMaterials = True
    params.enableSceneLights = False
    # Storm's unlit fallback ignores PreviewSurface texture networks.
    params.enableLighting = bool(lit or textured)
    if textured:
        params.colorCorrectionMode = "sRGB"
    params.clearColor = Gf.Vec4f(0, 0, 0, 0)
    params.cullStyle = UsdImagingGL.CullStyle.CULL_STYLE_NOTHING
    if wire:
        params.drawMode = UsdImagingGL.DrawMode.DRAW_WIREFRAME_ON_SURFACE
        params.wireframeColor = Gf.Vec4f(0.16, 0.10, 0.22, 0.55)
    else:
        params.drawMode = UsdImagingGL.DrawMode.DRAW_SHADED_SMOOTH
    vp.fbo.bind()
    GL.glViewport(0, 0, vp.width, vp.height)
    GL.glEnable(GL.GL_DEPTH_TEST)
    GL.glDepthFunc(GL.GL_LESS)
    GL.glDepthMask(GL.GL_TRUE)
    GL.glEnable(GL.GL_BLEND)
    GL.glBlendFunc(GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA)
    GL.glClearColor(0, 0, 0, 0)
    GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
    subtree = root if root is not None else stage.GetPseudoRoot()
    vp.engine.Render(subtree, params)
    for _ in range(32):
        if vp.engine.IsConverged():
            break
        vp.engine.Render(subtree, params)
    vp.fbo.bind()
    return rm._to_pil(vp.fbo.toImage())


def find_rig_root(stage):
    for prim in stage.Traverse():
        if prim.GetTypeName() == "RigExecRoot":
            return prim
    return None


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("stage")
    ap.add_argument("out")
    ap.add_argument("--frames", default="1")
    ap.add_argument("--sublayers", default="")
    ap.add_argument("--guides", type=float, default=0.0,
                    help="guide pass opacity (0 = no guide pass)")
    ap.add_argument("--wire", action="store_true")
    ap.add_argument("--textured", action="store_true", help="render painted PreviewSurface materials with a neutral camera light")
    ap.add_argument("--height", type=float, default=50.0)
    ap.add_argument("--center", default="0,-8.0")
    ap.add_argument("--size", default="1920x1080")
    ap.add_argument("--ss", type=int, default=1, help="supersample factor")
    args = ap.parse_args(argv)

    w, h = [int(v) for v in args.size.split("x")]
    W, H = w * args.ss, h * args.ss
    center = [float(v) for v in args.center.split(",")]
    frames = [float(f) for f in args.frames.split(",")]
    subs = [s for s in args.sublayers.split(",") if s]

    vp = rm.Viewport(W, H)
    if args.textured:
        # The colour AOV enables Storm's sRGB display conversion, as in usdview.
        vp.engine.SetRendererAov("color")
    rm.ImagingBridge()
    stage = open_stage(args.stage, subs)
    rig = find_rig_root(stage)
    cam = ortho_camera(center, args.height, W / float(H))
    vp.set_camera(cam)
    if args.textured:
        from pxr import Glf
        light = Glf.SimpleLight()
        light.position = (0, 0, 1, 0)
        light.diffuse = (1, 1, 1, 1)
        light.ambient = (0, 0, 0, 1)
        light.specular = (0, 0, 0, 1)
        vp.engine.SetLightingState([light], Glf.SimpleMaterial(), (0, 0, 0, 1))
    bg = rm._gradient(W, H, BG_TOP, BG_BOTTOM).convert("RGBA")
    os.makedirs(os.path.dirname(os.path.abspath(args.out)) or ".", exist_ok=True)
    for i, f in enumerate(frames):
        art = render_pass(vp, stage, f, wire=args.wire, textured=args.textured)
        frame = bg.copy()
        frame.alpha_composite(art)
        if args.guides > 0 and rig is not None:
            g = render_pass(vp, stage, f, guides=True, root=rig, lit=True)
            if args.guides < 1.0:
                a = g.getchannel("A").point(lambda v: int(v * args.guides))
                g.putalpha(a)
            frame.alpha_composite(g)
        if args.ss > 1:
            frame = frame.resize((w, h), Image.LANCZOS)
        path = "%s_%04d.png" % (args.out, int(round(f * 10)) if f != int(f) else int(f))
        frame.convert("RGB").save(path)
        print("wrote", path)
    vp.close()


if __name__ == "__main__":
    main()
