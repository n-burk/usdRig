#!/usr/bin/env python
"""
Emits examples/12_CurvenetProfile.usda.

Hand-authoring a curvenet is not reasonable -- the net below has 28 splines
over 68 pooled control points, and every ring handle is a circle-Bezier
constant -- so this script is the source of truth and the .usda it writes is
generated output. Edit here, re-run, do not edit the .usda.

The scene demonstrates the claim the 2022 paper actually makes: the curvenet's
knots are posed by an ORDINARY RigExec mover (a matrix mover driven by an FK
joint through a weight object), and the Profile Mover then propagates that
articulation onto a tube whose tessellation the rig never mentions.
"""
import math
import os

# ---------------------------------------------------------------------------
# tube
# ---------------------------------------------------------------------------

TUBE_RADIUS = 1.0
TUBE_HEIGHT = 6.0
TUBE_SIDES = 16
TUBE_RINGS = 13  # rings of vertices, so 12 quad bands

# ---------------------------------------------------------------------------
# curvenet: three profile rings plus four longitudinal curves
# ---------------------------------------------------------------------------

NET_RADIUS = 1.06          # sits just off the surface, as §3 assumes
RING_HEIGHTS = [1.5, 3.0, 4.5]
LONGITUDINAL_ENDS = (0.25, 5.75)
SPOKES = 4                 # compass points where rings and rails meet
# Circle through four cubic Bezier spans: handles at k * radius, tangentially.
CIRCLE_K = 4.0 / 3.0 * math.tan(math.pi / (2.0 * SPOKES))


def build_tube():
    points, normals = [], []
    for ring in range(TUBE_RINGS):
        y = TUBE_HEIGHT * ring / (TUBE_RINGS - 1)
        for side in range(TUBE_SIDES):
            a = 2.0 * math.pi * side / TUBE_SIDES
            c, s = math.cos(a), math.sin(a)
            points.append((TUBE_RADIUS * c, y, TUBE_RADIUS * s))
            # Vertex interpolation, not faceVarying: faceVarying normals fail
            # the derived-normal cardinality check and freeze at the rest pose.
            normals.append((c, 0.0, s))
    counts, indices = [], []
    for ring in range(TUBE_RINGS - 1):
        for side in range(TUBE_SIDES):
            n = (side + 1) % TUBE_SIDES
            a = ring * TUBE_SIDES + side
            b = ring * TUBE_SIDES + n
            c = (ring + 1) * TUBE_SIDES + n
            d = (ring + 1) * TUBE_SIDES + side
            # Counter-clockwise seen from OUTSIDE: up the tube first, then
            # around. The other winding gives inward face normals, which the
            # derived-normal pass then computes as the negation of the
            # authored ones.
            counts.append(4)
            indices.extend([a, d, c, b])
    return points, normals, counts, indices


def build_curvenet():
    """Returns (points, splineIndices, knotLabels)."""
    points = []
    splines = []

    def add_point(p):
        points.append(p)
        return len(points) - 1

    def ring_point(radius, y, spoke):
        a = 2.0 * math.pi * spoke / SPOKES
        return (radius * math.cos(a), y, radius * math.sin(a))

    def ring_tangent(radius, spoke):
        # d/da (r cos a, 0, r sin a) = (-r sin a, 0, r cos a)
        a = 2.0 * math.pi * spoke / SPOKES
        return (-radius * math.sin(a), 0.0, radius * math.cos(a))

    # Ring knots, shared by the ring spans AND the longitudinal rails: this
    # sharing is the entire connectivity model, and it is what makes each of
    # these an INTERSECTION (valence 4) in §3's terms.
    ring_knots = {}
    for height in RING_HEIGHTS:
        for spoke in range(SPOKES):
            ring_knots[(height, spoke)] = add_point(
                ring_point(NET_RADIUS, height, spoke))

    # Ring spans.
    for height in RING_HEIGHTS:
        for spoke in range(SPOKES):
            a = ring_knots[(height, spoke)]
            b = ring_knots[(height, (spoke + 1) % SPOKES)]
            ta = ring_tangent(NET_RADIUS, spoke)
            tb = ring_tangent(NET_RADIUS, (spoke + 1) % SPOKES)
            step = 2.0 * math.pi / SPOKES
            h0 = add_point(tuple(points[a][i] + CIRCLE_K * ta[i] * 1.0
                                 for i in range(3)))
            h1 = add_point(tuple(points[b][i] - CIRCLE_K * tb[i] * 1.0
                                 for i in range(3)))
            splines.extend([a, h0, h1, b])
            del step

    # Longitudinal rails: bottom anchor -> ring -> ring -> ring -> top anchor.
    for spoke in range(SPOKES):
        chain = [add_point(ring_point(NET_RADIUS, LONGITUDINAL_ENDS[0], spoke))]
        chain += [ring_knots[(h, spoke)] for h in RING_HEIGHTS]
        chain.append(
            add_point(ring_point(NET_RADIUS, LONGITUDINAL_ENDS[1], spoke)))
        for i in range(len(chain) - 1):
            a, b = chain[i], chain[i + 1]
            pa, pb = points[a], points[b]
            h0 = add_point(tuple(pa[k] + (pb[k] - pa[k]) / 3.0
                                 for k in range(3)))
            h1 = add_point(tuple(pa[k] + (pb[k] - pa[k]) * 2.0 / 3.0
                                 for k in range(3)))
            splines.extend([a, h0, h1, b])

    return points, splines


def weight_for(point):
    """
    Per-control-point influence of the bend joint: zero below the hinge,
    ramping to one over the upper half. Authored explicitly rather than
    generated from a volume so the example stays readable and the numbers
    are obvious.
    """
    y = point[1]
    lo, hi = 2.0, 4.5
    t = (y - lo) / (hi - lo)
    return max(0.0, min(1.0, t))


def fmt3(v):
    return "(%s)" % ", ".join("%.6g" % x for x in v)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    out = os.path.join(here, "12_CurvenetProfile.usda")

    tube_points, tube_normals, counts, indices = build_tube()
    net_points, splines = build_curvenet()
    weights = [weight_for(p) for p in net_points]

    lo = (-TUBE_RADIUS * 1.6, -0.2, -TUBE_RADIUS * 1.6)
    hi = (TUBE_RADIUS * 1.6, TUBE_HEIGHT + 0.4, TUBE_RADIUS * 1.6)

    body = '''#usda 1.0
(
    """
    RigExec example 12 - curvenets and the Profile Mover.

    An implementation of de Goes, Sheffler & Fleischer, "Character
    Articulation through Profile Curves" (ACM TOG 41(4), SIGGRAPH 2022).
    See docs/curvenet.md; the papers are in docs/papers/.

    The net is three profile rings joined by four longitudinal rails. Every
    ring knot is shared by two ring spans and two rails, so §3 labels it an
    INTERSECTION, and the frames, widths and twist along every curve are
    deduced from that layout -- nothing here authors a normal or a tangent
    frame.

    What poses the net is an ORDINARY matrix mover: RigExecCurvenet is a
    UsdGeomPointBased, so its control-point pool is an exact native
    point3f[] and the existing rig machinery articulates it with no
    curvenet-specific code. That is the paper's "articulate their knots
    using traditional rigging tools", and it is why the mover chain runs
    curvenet targets before the geometry they drive.

    The tube's tessellation appears nowhere in the rig. Re-mesh it and the
    same net still articulates it -- the property Fig. 8 of the paper
    demonstrates across five different tessellations of one hand.

    GENERATED by examples/build_curvenet_example.py. Edit that, not this.
    """
    defaultPrim = "CurvenetAsset"
    endTimeCode = 1048
    metersPerUnit = 0.01
    startTimeCode = 1001
    timeCodesPerSecond = 24
    upAxis = "Y"
)

def Xform "CurvenetAsset"
{
    def RigExecRoot "Rig"
    {
        uniform token rigExec:partition = "CurvenetAsset"

        def Scope "Controls"
        {
            def RigExecControl "BendCtl" (
                prepend apiSchemas = ["RigExecControlAPI"]
            )
            {
                double avars:rz = 0
                double avars:rz.spline = {
                    1001: 0; pre (0, 0); post linear,
                    1024: 38; post linear,
                    1048: 0; post linear,
                }
                matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 3, 0, 1) )
            }
        }

        def Scope "Solvers"
        {
            def RigExecFkChain "BendChain"
            {
                rel rigExec:controls = </CurvenetAsset/Rig/Controls/BendCtl>
                rel rigExec:joints = </CurvenetAsset/Rig/Joints/Bend>
            }
        }

        def Scope "Joints"
        {
            def RigExecJoint "Bend"
            {
                double guide:length = 1.2
                matrix4d rest:space = ( (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0), (0, 3, 0, 1) )
            }
        }

        def Scope "Weights"
        {
            # Influence of the bend on each POOL POINT of the curvenet -- 68
            # numbers, against 208 tube vertices the rig never mentions. That
            # ratio is the paper's headline claim about authoring cost.
            def RigExecStaticWeight "NetBend"
            {
                uniform float rigExec:defaultWeight = 0
                uniform token rigExec:rangePolicy = "clamp"
                uniform token rigExec:representation = "dense"
                uniform float[] rigExec:values = [%(weights)s]
                rel rigExec:weightTarget = </CurvenetAsset/Geom/Net.points>
            }
        }

        def Scope "Movers"
        {
            def Scope "Geometry"
            {
                # Display ProfileMover above PoseNet. The mover stack executes
                # bottom-to-top, so the net is posed before it drives Tube.
                reorder nameChildren = ["ProfileMover", "PoseNet"]

                # 1. Pose the curvenet's knots, exactly as any other geometry
                #    would be posed.
                def RigExecMatrixMover "PoseNet" (
                    prepend apiSchemas = ["RigExecMoverAPI"]
                )
                {
                    rel rigExec:moves = </CurvenetAsset/Geom/Net.points>
                    rel rigExec:transform = </CurvenetAsset/Rig/Joints/Bend>
                    uniform token rigExec:transformReadPhase = "base"
                    rel rigExec:weightObject = </CurvenetAsset/Rig/Weights/NetBend>
                }

                # 2. Propagate the posed net onto the surface.
                def RigExecCurvenetMover "ProfileMover" (
                    prepend apiSchemas = ["RigExecMoverAPI"]
                )
                {
                    rel rigExec:curvenet = </CurvenetAsset/Geom/Net>
                    float inputs:strength = 1
                    rel rigExec:moves = </CurvenetAsset/Geom/Tube.points>
                }
            }
        }
    }

    def Scope "Geom"
    {
        def Mesh "Tube"
        {
            float3[] extent = [%(lo)s, %(hi)s]
            int[] faceVertexCounts = [%(counts)s]
            int[] faceVertexIndices = [%(indices)s]
            normal3f[] normals = [%(normals)s] (
                interpolation = "vertex"
            )
            point3f[] points = [%(points)s]
            uniform token subdivisionScheme = "none"
        }

        def RigExecCurvenet "Net"
        {
            uniform token rigExec:basis = "bezier"
            int[] rigExec:splineIndices = [%(splines)s]
            uniform int rigExec:samplesPerSpline = 5
            color3f guide:displayColor = (0.95, 0.35, 1)
            double guide:radius = 0.05
            point3f[] points = [%(netpoints)s]
            float[] widths = [%(netwidths)s]
        }
    }
}
''' % {
        "weights": ", ".join("%.4g" % w for w in weights),
        "lo": fmt3(lo),
        "hi": fmt3(hi),
        "counts": ", ".join(str(c) for c in counts),
        "indices": ", ".join(str(i) for i in indices),
        "normals": ", ".join(fmt3(n) for n in tube_normals),
        "points": ", ".join(fmt3(p) for p in tube_points),
        "splines": ", ".join(str(s) for s in splines),
        "netpoints": ", ".join(fmt3(p) for p in net_points),
        "netwidths": ", ".join("0.04" for _ in net_points),
    }

    with open(out, "w", newline="\n") as handle:
        handle.write(body)

    knots = set()
    for i in range(0, len(splines), 4):
        knots.add(splines[i])
        knots.add(splines[i + 3])
    print("wrote %s" % out)
    print("  tube:     %d points, %d faces" % (len(tube_points), len(counts)))
    print("  curvenet: %d pool points, %d splines, %d knots"
          % (len(net_points), len(splines) // 4, len(knots)))


if __name__ == "__main__":
    main()
