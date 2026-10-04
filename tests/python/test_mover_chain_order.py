"""Every ordinal-named mover chain is LISTED in reverse.

A mover's place in the pose stack is the REVERSE of its namespace order,
so an op chain written `..._0_blend`, `..._1_multiply` -- the ordinal
saying which runs first -- has to appear in `reorder nameChildren`
BACKWARDS, multiply before blend. List it forwards and the multiply runs
first and the blend overwrites it: the later ops vanish, silently and
without an error anywhere.

That is not hypothetical. Both `jaw__mouth_corner_*_driver_avars_tz`
chains shipped listed forwards, so the jaw's `x -1/30` scale never
reached the driver. The driver ran to the raw jaw angle (-50 instead of
1.667), left every pose's support, and the corner correctives first
blew up to weights of +/-588 and then read zero for the rest of the
jaw's travel.

This is arithmetic on the authored layers -- no stage, no build, no
schema -- because the ordering is a property of the text.

Usage:
    python test_mover_chain_order.py
"""
import pathlib
import re
import sys

# `..._<ordinal>_<operation>`, the naming the converter emits for a chain
# of float-math movers writing one channel.
_CHAIN = re.compile(r"^(?P<stem>.+)_(?P<ordinal>\d+)_(?P<op>[a-z]+)$")
_REORDER = re.compile(r"reorder nameChildren = \[([^\]]*)\]")

_EXAMPLES = pathlib.Path(__file__).resolve().parents[2] / "examples"


def _Chains(names):
    """{stem: [(ordinal, position)]} for the ordinal-named names given."""
    found = {}
    for position, name in enumerate(names):
        match = _CHAIN.match(name)
        if match:
            found.setdefault(match.group("stem"), []).append(
                (int(match.group("ordinal")), position))
    return {stem: entries for stem, entries in found.items()
            if len(entries) > 1}


def _Offenders(text):
    """Every chain this layer lists in ascending ordinal."""
    out = []
    for block in _REORDER.finditer(text):
        names = [token.strip().strip('"')
                 for token in block.group(1).split(",") if token.strip()]
        for stem, entries in _Chains(names).items():
            order = [ordinal for ordinal, _position
                     in sorted(entries, key=lambda e: e[1])]
            if order != sorted(order, reverse=True):
                out.append((stem, order))
    return out


def main():
    layers = sorted(_EXAMPLES.rglob("*.usda"))
    if not layers:
        print("no example layers found under %s" % _EXAMPLES)
        return 1
    failures = []
    chains = 0
    for layer in layers:
        text = layer.read_text(encoding="utf-8", errors="ignore")
        for block in _REORDER.finditer(text):
            names = [t.strip().strip('"')
                     for t in block.group(1).split(",") if t.strip()]
            chains += len(_Chains(names))
        for stem, order in _Offenders(text):
            failures.append((layer.name, stem, order))

    for name, stem, order in failures:
        print("FAIL %s: %s is listed %s; the pose stack runs the REVERSE "
              "of namespace order, so it must be listed descending"
              % (name, stem, order))
    print("checked %d layer(s), %d ordinal-named mover chain(s), "
          "%d failure(s)" % (len(layers), chains, len(failures)))
    if failures:
        return 1
    print("RIGEXEC_MOVER_CHAIN_ORDER_OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
