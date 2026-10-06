"""Named acting poses and control-only performance/sweep for Shion.

All values are editable avars, not baked mesh points. The normal performance
uses small expressions; the sweep separately demonstrates the wider range.
"""

import numpy as np
from perf import blink

POSES = {
    "Neutral": {},
    "Soft smile": {
        "Mouth.rz": 7.0,
        "Eye_L.tx": 0.15,
        "Eye_R.tx": 0.15,
        "Brow_L.ty": 0.12,
    },
    "Whisper": {
        "Mouth.ty": -0.16,
        "Mouth.tx": -0.12,
        "FaceAngle.ry": -8.0,
        "Eye_L.tz": 0.18,
        "Eye_R.tz": 0.18,
    },
    "Blink": {"Eye_L.ty": -1.0, "Eye_R.ty": -1.0},
    "Speech A": {"Mouth.ty": -0.70, "Brow_R.ty": 0.12},
    "Speech I teeth": {"Mouth.ty": -0.32, "Mouth.tx": 0.85, "Teeth.ty": 0.9},
    "Speech U": {"Mouth.ty": -0.30, "Mouth.tx": -0.85},
    "Speech O": {"Mouth.ty": -0.70, "Mouth.tx": -0.7},
    "Surprised": {
        "Mouth.ty": -0.75,
        "Mouth.tx": -0.3,
        "Eye_L.ty": 0.6,
        "Eye_R.ty": 0.6,
        "Brow_L.ty": 0.7,
        "Brow_R.ty": 0.7,
        "FaceAngle.rx": -5.0,
    },
    "Tongue out": {
        "Mouth.ty": -0.4,
        "Tongue.ty": 0.9,
        "Mouth.rz": 9.0,
        "Eye_L.ty": -0.25,
        "Head.rz": -4.0,
    },
    "Wide smile": {
        "Mouth.ty": -0.55,
        "Mouth.tx": 1.0,
        "Mouth.rz": 15.0,
        "Teeth.ty": 0.7,
        "Eye_L.tx": 0.4,
        "Eye_R.tx": 0.4,
    },
    "Pushed expression": {
        "Mouth.ty": -1.5,
        "Mouth.tx": 0.4,
        "Eye_L.ty": 0.9,
        "Eye_R.ty": 0.9,
        "Brow_L.ty": 1.0,
        "Brow_R.ty": 1.0,
        "Look.tz": 0.45,
    },
    "Turn left": {
        "FaceAngle.ry": -25.0,
        "FaceAngle.rx": 7.0,
        "Torso.ry": -6.0,
        "Look.tx": -0.2,
    },
    "Turn right": {
        "FaceAngle.ry": 25.0,
        "FaceAngle.rx": -7.0,
        "Torso.ry": 6.0,
        "Look.tx": 0.2,
    },
    "Look down": {"FaceAngle.rx": 15.0, "Look.ty": -0.4, "Mouth.ty": -0.15},
    "Look up": {"FaceAngle.rx": -15.0, "Look.ty": 0.4},
    "Greeting": {
        "Shoulder_R.rz": -5.0,
        "Elbow_R.rz": 8.0,
        "Hand_R.rz": -10.0,
        "Hand_R.turn": 20.0,
        "Hand_R.tz": 2.0,
        "Mouth.rz": 10.0,
        "Head.rz": -3.0,
    },
    "Fingers curl": {"Hand_L.curl": 0.9, "Hand_R.curl": 0.5, "Hand_R.turn": -25.0},
    "Point": {
        "Shoulder_R.rz": -8.0,
        "Elbow_R.rz": -5.0,
        "Hand_R.rz": -15.0,
        "Hand_R.tz": 2.0,
        "Hand_R.curl": 0.9,
        "Index_R_0.curl": -0.9,
    },
    "Lip fullness": {"Mouth.ty": -0.4, "Mouth.tx": 0.65, "Lip.ty": 0.65},
}


def _sample(beats, length):
    frames = np.arange(1, length + 1, dtype=float)
    keys = set(k for _, pose in beats for k in pose)
    keys.update(
        (
            "Head.rz",
            "Neck.rz",
            "Body.rz",
            "Body.tx",
            "FaceAngle.ry",
            "FaceAngle.rx",
            "Torso.ry",
            "Torso.ty",
        )
    )
    # Smoothstep segments hold pose targets without spline overshoot.
    times = np.array([f for f, _ in beats])
    result = {}
    for key in keys:
        values = np.array([p.get(key, 0.0) for _, p in beats])
        i = np.clip(np.searchsorted(times, frames, side="right") - 1, 0, len(times) - 2)
        t = np.clip((frames - times[i]) / (times[i + 1] - times[i]), 0, 1)
        t = t * t * (3 - 2 * t)
        result[key] = values[i] * (1 - t) + values[i + 1] * t
    return frames, result


def performance():
    beats = [
        (1, {}),
        (22, POSES["Soft smile"]),
        (36, POSES["Whisper"]),
        (44, POSES["Speech A"]),
        (51, POSES["Speech I teeth"]),
        (57, POSES["Speech U"]),
        (63, POSES["Speech O"]),
        (71, POSES["Soft smile"]),
        (82, POSES["Speech A"]),
        (91, POSES["Speech I teeth"]),
        (101, POSES["Soft smile"]),
        (124, dict(POSES["Greeting"], **{"FaceAngle.ry": 12.0})),
        (148, dict(POSES["Greeting"], **{"Hand_R.rz": 12.0, "Eye_L.ty": -1.0})),
        (168, POSES["Surprised"]),
        (182, POSES["Tongue out"]),
        (207, POSES["Soft smile"]),
        (240, {}),
    ]
    f, a = _sample(beats, 240)
    phase = (f - 1) / 239 * 2 * np.pi
    a["Torso.ty"] += 0.23 * (1 - np.cos(phase * 2))
    a["Body.rz"] += 0.7 * np.sin(phase)
    a["Neck.rz"] += 0.5 * np.sin(phase + 0.5) - 0.5 * np.sin(0.5)
    for tag in ("L", "R"):
        closing = blink([27, 104, 213], f)
        current = a.get("Eye_" + tag + ".ty", np.zeros_like(f))
        a["Eye_" + tag + ".ty"] = np.where(closing > 0, -closing, current)
    return f, a


def sweep():
    names = list(POSES)
    beats = [(1, {})]
    for i, name in enumerate(names[1:]):
        frame = 1 + (i + 1) * 16
        beats += [(frame - 5, POSES[name]), (frame, POSES[name])]
    beats += [(320, {})]
    return _sample(beats, 320)
