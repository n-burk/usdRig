#!/usr/bin/env python
"""
Bake generated glyphs into the gizmo toolbar's icon set.

THE PIPELINE, end to end, so the set can be regenerated rather than only
admired. The art is produced by the Codex CLI's built-in image generation, one
glyph per call, with a fixed style preamble:

    codex exec --skip-git-repo-check -s danger-full-access -C <dir>       "Use your built-in image generation tool to create ONE image and save it
       as <name>.png in the current working directory.

       Style, exactly:
       - pure black background (#000000), glyph in pure white (#FFFFFF)
       - a SINGLE centered line-art glyph: no text, no border, no shadow, no
         gradient, no 3D, no background shapes, no frame
       - uniform stroke weight, roughly 3% of the image width, with rounded
         caps and rounded joins
       - the glyph occupies the middle ~76% of the canvas, centered
       - flat, geometric, minimal, in the visual language of Blender/Maya
         viewport toolbar icons

       The glyph: <one sentence describing it>"

Black ground rather than transparency on purpose: asked for transparency an
image model tends to draw a checkerboard, whereas white-on-black keys cleanly
and keeps the antialiasing.

Then this script, which is where the twelve independent generations become a
SET. The generator centres and sizes each glyph only roughly, and roughly is
not good enough for a toolbar: twelve icons each at their own scale read as
twelve unrelated pictures, and the eye notices long before it can say why.

  * alpha comes from LUMINANCE, so the black ground becomes transparent and the
    antialiased edge of a stroke becomes a partially transparent edge rather
    than a grey fringe against whatever the toolbar is painted;
  * the glyph is cropped to its own ink, scaled so its LONGEST side is the same
    fraction of the canvas for every icon, and centred on that bounding box --
    which is what makes them a family;
  * the ink is flattened to white, because the toolbar tints it at draw time
    (gizmoIcons) and a stray off-white pixel would tint differently.

Usage: bakeGizmoIcons.py <source dir> <dest dir> [size]

The shipped set is 128 px, matching icons/ at the repository root:

    python tools/bakeGizmoIcons.py <generated> plugin/rigExecUsdview/icons 128
"""

import os
import sys

import numpy as np
from PIL import Image

# The fraction of the canvas the longest side of a glyph fills. 0.78 leaves a
# margin that keeps a round glyph (rotate) from touching the button's edge
# while a square one (scale) still reads at 16 px.
_EXTENT = 0.78
# Anything dimmer than this is background, not an antialiased stroke edge. The
# generator's blacks are clean, so this only has to survive JPEG-ish ringing.
_FLOOR = 24
# Stroke weight as a fraction of the glyph's own extent, which every icon is
# brought to. The generator does roughly honour a weight asked for in the
# prompt -- these arrive between 6.5% and 9.8% -- but roughly is not a set, and
# the number that matters is not the one in the source image anyway.
#
# 0.115 rather than the ~0.09 a vector version of these glyphs would use,
# because this art is RASTER and arrives at the toolbar by being scaled down.
# A 1 px line drawn by a vector renderer at 18 px is a 1 px line; the same line
# resampled from 128 px is spread across two rows of pixels at half opacity
# each, and reads as grey. The extra weight is what that resampling gives back.
_WEIGHT = 0.115
# A cap on how far a stroke is thickened, in multiples of what it started as.
# Past this the glyph stops being the one that was drawn: counters close and a
# line-art icon turns into a blob.
_MAX_GROWTH = 2.6


def _Dilate(alpha, radius):
    """
    Thicken `alpha` by about `radius` pixels.

    Iterated 3x3 maximum, which grows a shape by one pixel a pass. Not a true
    Euclidean disc -- the corners come out very slightly square -- but the
    result is downscaled by 10x on its way to an icon, where the difference is
    a fraction of a pixel and the alternative is a dependency on scipy.
    """
    radius = int(round(radius))
    if radius <= 0:
        return alpha
    out = alpha
    for _ in range(radius):
        padded = np.pad(out, 1, mode="constant")
        stack = np.stack([
            padded[:-2, 1:-1], padded[2:, 1:-1],
            padded[1:-1, :-2], padded[1:-1, 2:],
            padded[1:-1, 1:-1],
            # The diagonals at reduced strength: with them the growth is
            # closer to round, and at full strength it would be square.
            0.7 * padded[:-2, :-2], 0.7 * padded[:-2, 2:],
            0.7 * padded[2:, :-2], 0.7 * padded[2:, 2:],
        ])
        out = np.clip(stack.max(axis=0), 0.0, 1.0)
    return out


def _StrokeWidth(mask):
    """
    The mean stroke width of a line-art mask, in pixels.

    A stroke of length L and width w has area L*w and a boundary of about 2L,
    so w is about twice the area over the boundary. Crude for a filled shape --
    a solid diamond reports its own diagonal -- which is exactly right here: a
    filled glyph has no stroke to thicken and this returns something large
    enough that the normalisation leaves it alone.
    """
    ink = float(mask.sum())
    if ink <= 0:
        return 0.0
    padded = np.pad(mask, 1, mode="constant")
    eroded = (padded[:-2, 1:-1] & padded[2:, 1:-1]
              & padded[1:-1, :-2] & padded[1:-1, 2:] & padded[1:-1, 1:-1])
    boundary = float(ink - eroded.sum())
    return 2.0 * ink / max(boundary, 1.0)


def _Bake(source, size):
    image = Image.open(source).convert("RGBA")
    rgb = np.asarray(image, dtype=np.float32)[..., :3]
    # Rec. 709 luminance: the glyph is white on black, so brightness IS
    # coverage, and taking it per pixel keeps the antialiasing the generator
    # produced instead of throwing it away with a hard threshold.
    luma = (0.2126 * rgb[..., 0] + 0.7152 * rgb[..., 1] + 0.0722 * rgb[..., 2])
    alpha = np.clip((luma - _FLOOR) / (255.0 - _FLOOR), 0.0, 1.0)

    rows = np.where(alpha.max(axis=1) > 0.35)[0]
    cols = np.where(alpha.max(axis=0) > 0.35)[0]
    if len(rows) == 0 or len(cols) == 0:
        raise ValueError("%s has no glyph in it" % source)
    top, bottom = int(rows[0]), int(rows[-1]) + 1
    left, right = int(cols[0]), int(cols[-1]) + 1
    cropped = alpha[top:bottom, left:right]

    # Normalise the WEIGHT as well as the extent. The generator draws each
    # glyph at whatever stroke it likes, and a set whose strokes disagree reads
    # as a set of accidents; worse, the thin ones vanish at 16 px.
    extent = float(max(cropped.shape))
    width = _StrokeWidth(cropped > 0.5)
    target = _WEIGHT * extent
    if 0.0 < width < target:
        grown = min(target, width * _MAX_GROWTH)
        cropped = _Dilate(cropped, (grown - width) / 2.0)
        # Dilating grows the bounding box too, so it is re-taken.
        rows = np.where(cropped.max(axis=1) > 0.35)[0]
        cols = np.where(cropped.max(axis=0) > 0.35)[0]
        cropped = cropped[int(rows[0]):int(rows[-1]) + 1,
                          int(cols[0]):int(cols[-1]) + 1]

    # One scale for both axes: squashing a glyph to fill a square would make
    # the stroke weight differ between its horizontal and vertical runs, which
    # is the one thing every icon here shares.
    height, width = cropped.shape
    scale = (_EXTENT * size) / max(height, width)
    new = (max(1, int(round(width * scale))), max(1, int(round(height * scale))))
    plate = Image.fromarray((cropped * 255).astype(np.uint8), mode="L")
    plate = plate.resize(new, Image.LANCZOS)

    canvas = Image.new("L", (size, size), 0)
    canvas.paste(plate, ((size - new[0]) // 2, (size - new[1]) // 2))
    out = Image.new("RGBA", (size, size), (255, 255, 255, 0))
    out.putalpha(canvas)
    # White ink, tinted at draw time by the toolbar. Stored white so the same
    # file works on any chrome.
    out.paste((255, 255, 255), (0, 0, size, size), canvas)
    out.putalpha(canvas)
    return out


def main():
    source, dest = sys.argv[1], sys.argv[2]
    size = int(sys.argv[3]) if len(sys.argv) > 3 else 128
    if not os.path.isdir(dest):
        os.makedirs(dest)
    names = sorted(n for n in os.listdir(source) if n.endswith(".png"))
    for name in names:
        baked = _Bake(os.path.join(source, name), size)
        baked.save(os.path.join(dest, name))
        print("baked %s" % name)
    print("%d icon(s)" % len(names))


if __name__ == "__main__":
    main()
