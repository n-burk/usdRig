"""The expression STATE: every art generator takes one of these.

All values are >= 0 -- they are exactly the rig's blend-channel weights
(RigExecBlendSample activations must be positive), so the keyforms a
generator is sampled at are the states the rig interpolates between.
"""

import copy

KEYS = [
    "eyeR.close", "eyeR.wide", "eyeR.smile", "eyeR.flat",
    "eyeL.close", "eyeL.wide", "eyeL.smile", "eyeL.flat",
    "look.r", "look.l", "look.u", "look.d", "iris.shrink",
    "browR.up", "browR.down", "browR.angry", "browR.worried",
    "browL.up", "browL.down", "browL.angry", "browL.worried",
    "mouth.open", "mouth.wide", "mouth.round", "mouth.smile", "mouth.frown",
    "mouth.smirkR", "mouth.smirkL", "mouth.teeth", "mouth.tongue", "mouth.lipThick",
    "blush", "sweat", "hatch",
]

REST = {k: 0.0 for k in KEYS}


def state(**kw):
    s = dict(REST)
    for k, v in kw.items():
        k = k.replace("__", ".")
        if k not in s:
            raise KeyError(k)
        s[k] = float(v)
    return s


def S(d=None, **kw):
    s = dict(REST)
    if d:
        for k, v in d.items():
            if k not in s:
                raise KeyError(k)
            s[k] = float(v)
    for k, v in kw.items():
        k = k.replace("__", ".")
        s[k] = float(v)
    return s


def side_key(side):
    return "R" if side > 0 else "L"


def eye_params(st, side):
    t = "eye%s." % side_key(side)
    return st[t + "close"], st[t + "wide"], st[t + "smile"], st[t + "flat"]


def look(st):
    return st["look.r"] - st["look.l"], st["look.u"] - st["look.d"]


def brow_params(st, side):
    t = "brow%s." % side_key(side)
    return st[t + "up"], st[t + "down"], st[t + "angry"], st[t + "worried"]


def mouth_params(st):
    return dict(open=st["mouth.open"], wide=st["mouth.wide"] - st["mouth.round"],
                smile=st["mouth.smile"] - st["mouth.frown"], smirk=st["mouth.smirkR"] - st["mouth.smirkL"],
                teeth=st["mouth.teeth"])


copy = copy
