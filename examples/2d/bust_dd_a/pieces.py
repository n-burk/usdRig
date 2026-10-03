"""Paint every piece of Kaede (deterministic) -- the one entry point the
builder and the previews share."""

import time

import art
import art_body
import art_hair
import art_head


def all_pieces(verbose=True, body=True):
    t0 = time.time()
    out = []
    fr, lk = art_hair.face_shadow_polys()
    face = art_head.face_pieces(fr, lk)
    out += face
    out += art.eye_mask_pieces(face[0])
    out += art_head.side_plane_pieces()
    out += art_head.ear_pieces()
    out += art_head.neck_pieces()
    out += art.eye_pieces()
    out += art.brow_pieces()
    out += art.nose_pieces()
    out += art.mouth_pieces()
    out += art_hair.hair_pieces()
    if body:
        out += art_body.shirt_pieces()
    if body:
        out += art_body.cardigan_pieces()
    if body:
        out += art_body.collar_pieces()
    if body:
        out += art_body.ribbon_pieces()
    out += art_body.earring_pieces()
    out += art_body.pins_pieces()
    out += art_body.blush_pieces()
    out += art_body.fx_pieces()
    for p in out:
        if p.name == "BlushPatch":
            p.meta["translucent"] = True
    if verbose:
        n = sum(len(p.rest) for p in out)
        print("painted %d pieces, %d points in %.1fs" % (len(out), n, time.time() - t0))
    return out
