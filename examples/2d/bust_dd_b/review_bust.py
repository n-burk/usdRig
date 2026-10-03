"""Render evaluated Shion meshes on the CPU for reproducible art review.

Run in the RigExec Python environment:
    python examples/2d/bust_dd_b/review_bust.py --out /path/to/review
Requires numpy, Pillow, OpenCV and numba, as does the art preview module.
The renderer reads the generated USD materials and the actual RigExec pose.
"""

import argparse
import os
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageOps

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
sys.path.insert(0, str(REPO / "docs"))
import render_media as rm
import preview


def meshes(stage):
    from pxr import UsdGeom, UsdShade

    result = []
    for prim in stage.GetPrimAtPath("/Shion/Geom").GetChildren():
        mesh = UsdGeom.Mesh(prim)
        if not mesh:
            continue
        pts = np.array(mesh.GetPointsAttr().Get(), dtype=float)
        tris = np.array(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
        uv = np.array(UsdGeom.PrimvarsAPI(mesh).GetPrimvar("st").Get())
        for sub in UsdGeom.Subset.GetAllGeomSubsets(mesh):
            mat, _ = UsdShade.MaterialBindingAPI(sub.GetPrim()).ComputeBoundMaterial()
            tex = stage.GetPrimAtPath(str(mat.GetPath()) + "/Texture")
            asset = tex.GetAttribute("inputs:file").Get()
            result.append(
                dict(
                    name=sub.GetPrim().GetName(),
                    path=str(prim.GetPath()),
                    points=pts,
                    uv=uv,
                    tris=tris[np.array(sub.GetIndicesAttr().Get())],
                    tex=np.array(Image.open(asset.resolvedPath).convert("RGBA")),
                    z=float(
                        np.median(pts[tris[np.array(sub.GetIndicesAttr().Get())], 2])
                    ),
                )
            )
    return result


def render(data, moved, region=(-44, -53, 44, 15), ppu=14):
    posed = []
    for m in data:
        p = np.asarray(moved.get(m["path"] + ".points", m["points"]))
        posed.append(dict(m, points=p, z=float(np.mean(p[m["tris"], 2]))))
    w, h = int((region[2] - region[0]) * ppu), int((region[3] - region[1]) * ppu)
    bg = preview.background(w, h, top=(237, 239, 244), bottom=(219, 228, 235))
    return Image.fromarray(
        np.uint8(
            np.clip(preview.render_meshes(posed, region, ppu, ss=2, bg=bg), 0, 1) * 255
        )
    )


def main():
    ap = argparse.ArgumentParser(__doc__)
    ap.add_argument("--stage", default=str(HERE / "bust_dd_b_rig.usda"))
    ap.add_argument("--out", required=True)
    ap.add_argument("--frames", default="")
    ap.add_argument("--presets", action="store_true")
    ap.add_argument("--ppu", type=int, default=14)
    ap.add_argument(
        "--region", default="-44,-53,44,15", help="x0,y0,x1,y1 in model units"
    )
    ap.add_argument("--only", default="", help="comma-separated named poses")
    args = ap.parse_args()
    rigexec = rm._schema_plugin()
    from pxr import Usd

    stage = Usd.Stage.Open(os.path.abspath(args.stage))
    stage.SetEditTarget(stage.GetSessionLayer())
    rig = rigexec.Rig(stage, "/Shion/Rig")
    rig.compile()
    data = meshes(stage)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    controls = {
        p.GetName().removesuffix("_ctl"): p
        for p in stage.Traverse()
        if p.GetTypeName() == "RigExecControl"
    }
    cases = {"Neutral": {}}
    if args.presets:
        from acting import POSES

        cases = POSES
    if args.only:
        names = args.only.split(",")
        cases = {name: cases[name] for name in names}
    region = tuple(map(float, args.region.split(",")))
    tiles = []
    for name, values in cases.items():
        stage.GetSessionLayer().Clear()
        for key, value in values.items():
            ctl, axis = key.split(".")
            controls[ctl].GetAttribute("avars:" + axis).Set(value)
        pose = rig.evaluate(1.0)
        im = render(data, pose.moved_properties(), region=region, ppu=args.ppu)
        im.save(out / (name.lower().replace(" ", "_") + ".png"))
        tile = ImageOps.contain(im, (324, 342), Image.Resampling.LANCZOS)
        card = Image.new("RGB", (324, 378), "#f4f5f8")
        card.paste(tile, ((324 - tile.width) // 2, 36 + (342 - tile.height) // 2))
        from matplotlib import font_manager

        ImageDraw.Draw(card).text(
            (14, 10),
            name,
            fill="#28334c",
            font=ImageFont.truetype(font_manager.findfont("DejaVu Sans"), 16),
        )
        tiles.append(card)
        print("rendered", name, flush=True)
    cols = min(4, len(tiles))
    sheet = Image.new(
        "RGB", (cols * 324, ((len(tiles) + cols - 1) // cols) * 378), "#f4f5f8"
    )
    for i, tile in enumerate(tiles):
        sheet.paste(tile, ((i % cols) * 324, (i // cols) * 378))
    sheet.save(out / "contact_sheet.png")
    if args.frames:
        # Animation uses its authored layer values, not the review overrides.
        stage.GetSessionLayer().Clear()
        for frame in map(float, args.frames.split(",")):
            im = render(
                data,
                rig.evaluate(frame).moved_properties(),
                region=region,
                ppu=args.ppu,
            )
            im.save(out / ("frame_%04d.png" % frame))
    print(out, flush=True)


if __name__ == "__main__":
    main()
