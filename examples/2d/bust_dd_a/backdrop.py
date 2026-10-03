"""The graphic backdrop card behind Kaede (scene dressing, not rigged): warm
paper, a big tomato disc behind the head offset toward her open side, a
thin ring, and a halftone field -- a flat poster look in the same hard-edged
cel language as the character."""

import math

import cv2
import numpy as np

import design as D
import paint as P

BBOX = (-52.0, -44.0, 60.0, 34.0)
PAPER = "#EFE7D8"
PAPER_DOT = "#E3D7C2"
DISC = "#EE5A45"
DISC_DOT = "#D9412F"
RING = "#27213A"
CENTRE = (6.0, 2.2)
RADIUS = 15.5


def paint(path, tpu=20):
    L = P.Layer("Backdrop", BBOX, tpu=tpu, ss=2)
    X, Y = L.XY()
    L.paint(np.ones(X.shape, np.float32), PAPER)
    # a halftone field: dots on a 45-degree lattice, bigger toward the lower
    # left (the shadow side of the room)
    pitch = 1.25
    u = (X + Y) / math.sqrt(2) / pitch
    v = (X - Y) / math.sqrt(2) / pitch
    du = (u - np.round(u)) * pitch
    dv = (v - np.round(v)) * pitch
    d = np.hypot(du, dv)
    grad = np.clip(0.55 - (X + 30.0) / 60.0 - (Y + 20.0) / 70.0, 0, 1)
    r = 0.42 * grad
    dots = np.clip((r - d) * L.s + 0.5, 0, 1)
    L.paint(dots, PAPER_DOT)
    # the disc, with its own darker dot band on the lower-right
    cx, cy = CENTRE
    disc = L.disc(cx, cy, RADIUS)
    L.paint(disc, DISC)
    g2 = np.clip(((X - cx) * 0.6 - (Y - cy) * 0.8) / RADIUS, 0, 1)
    r2 = 0.46 * np.clip((g2 - 0.25) * 1.6, 0, 1)
    L.paint(np.clip((r2 - d) * L.s + 0.5, 0, 1) * disc, DISC_DOT)
    # a thin ring offset from the disc
    ring = L.disc(cx + 1.6, cy - 1.0, RADIUS + 2.2) - L.disc(cx + 1.6, cy - 1.0, RADIUS + 1.95)
    L.paint(np.clip(ring, 0, 1), RING)
    rgba = L.finish(bleed=2)
    rgba[..., 3] = 255
    cv2.imwrite(path, cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGRA), [cv2.IMWRITE_PNG_COMPRESSION, 9])
    return path, L.bbox
