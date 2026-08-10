#
# usdview panel for authoring curvenets (docs/curvenet.md).
#
# The toolkit the 2022 paper describes is the specification for this file:
#
#   "The user can then insert control points at arbitrary locations on the
#    surface and click-and-drag curves resembling surface profiles. [...]
#    Importantly, we allow endpoints to be shared by multiple splines. [...]
#    Our toolkit also includes operations such as split and merge splines,
#    weld and break control points, project endpoints to the surface mesh,
#    and flatten tangents, to cite a few."
#
# The one thing that makes this possible in usdview at all is that a click on
# the model can be turned into a 3D point on the surface:
# stageView.computePickFrustum(x, y) followed by stageView.pick(frustum)
# returns hits carrying hitPoint and hitNormal in world space.
#
# NOTE the pick has to be done directly. usdview's own pickObject() emits
# signalPrimSelected with a "point" whose x and y it has already overwritten
# with scaled MOUSE coordinates (stageView.py, pickObject) -- so the signal
# cannot be used to recover where on the surface the artist clicked.
#
import math

from pxr import Gf, Sdf, Tf, Usd, UsdGeom, Vt

from pxr.Usdviewq.qt import QtCore, QtGui, QtWidgets


# ---------------------------------------------------------------------------
# stage helpers
# ---------------------------------------------------------------------------

CURVENET_TYPE = "RigExecCurvenet"
MOVER_TYPE = "RigExecCurvenetMover"


def FindRigPrim(stage):
    if not stage:
        return None
    for prim in stage.Traverse():
        if prim.GetTypeName() == "RigExecRig":
            return prim
    return None


def FindCurvenetPrims(stage):
    if not stage:
        return []
    return [p for p in stage.Traverse() if p.GetTypeName() == CURVENET_TYPE]


def MakeUniqueName(parent, base):
    if not parent.GetChild(base):
        return base
    index = 1
    while parent.GetChild("%s_%d" % (base, index)):
        index += 1
    return "%s_%d" % (base, index)


def GetPoints(prim):
    if not prim:
        return []
    attr = prim.GetAttribute("points")
    if not attr:
        return []
    value = attr.Get()
    return [Gf.Vec3f(p) for p in value] if value else []


def SetPoints(prim, points):
    attr = prim.GetAttribute("points")
    if not attr:
        attr = prim.CreateAttribute("points", Sdf.ValueTypeNames.Point3fArray)
    attr.Set(Vt.Vec3fArray([Gf.Vec3f(p) for p in points]))
    # UsdGeomPoints wants a width per point; keep them in step so the knots
    # stay visible and the prim stays valid.
    widths = prim.GetAttribute("widths")
    if not widths:
        widths = prim.CreateAttribute("widths", Sdf.ValueTypeNames.FloatArray)
    existing = widths.Get()
    default = float(existing[0]) if existing else 0.02
    widths.Set(Vt.FloatArray([default] * len(points)))


def GetSplines(prim):
    if not prim:
        return []
    attr = prim.GetAttribute("rigExec:splineIndices")
    if not attr:
        return []
    value = attr.Get()
    return list(value) if value else []


def SetSplines(prim, indices):
    attr = prim.GetAttribute("rigExec:splineIndices")
    if not attr:
        attr = prim.CreateAttribute("rigExec:splineIndices",
                                    Sdf.ValueTypeNames.IntArray)
    attr.Set(Vt.IntArray([int(i) for i in indices]))


# ---------------------------------------------------------------------------
# §3 derived structure
#
# Mirrors RigExecBuildCurvenetTopology's classification so the panel can show
# the artist what the deformation will actually see. Deliberately limited to
# the CLASSIFICATION -- valences, curve grouping -- and not the frame math:
# one implementation of the frames is enough, and it lives in C++ where the
# solve uses it.
# ---------------------------------------------------------------------------

KIND_UNUSED = "unused"
KIND_HANDLE = "handle"
KIND_ANCHOR = "anchor"
KIND_INTERIOR = "interior"
KIND_INTERSECTION = "intersection"


class Topology(object):

    def __init__(self, pointCount, splineIndices):
        self.pointCount = pointCount
        self.splines = [splineIndices[i:i + 4]
                        for i in range(0, len(splineIndices) - 3, 4)]
        self.valence = [0] * pointCount
        self.isHandle = [False] * pointCount
        self.incident = [[] for _ in range(pointCount)]
        for index, spline in enumerate(self.splines):
            a, h0, h1, b = spline
            if max(spline) >= pointCount or min(spline) < 0:
                continue
            self.valence[a] += 1
            self.valence[b] += 1
            self.incident[a].append(index)
            self.incident[b].append(index)
            self.isHandle[h0] = True
            self.isHandle[h1] = True

        self.kind = []
        for i in range(pointCount):
            if self.valence[i] >= 3:
                self.kind.append(KIND_INTERSECTION)
            elif self.valence[i] == 2:
                self.kind.append(KIND_INTERIOR)
            elif self.valence[i] == 1:
                self.kind.append(KIND_ANCHOR)
            elif self.isHandle[i]:
                self.kind.append(KIND_HANDLE)
            else:
                self.kind.append(KIND_UNUSED)

        self.curves = self._GroupCurves()

    def _Labelled(self, knot):
        return self.kind[knot] in (KIND_INTERSECTION, KIND_ANCHOR)

    def _Other(self, splineIndex, knot):
        a, _, _, b = self.splines[splineIndex]
        return b if knot == a else a

    def _GroupCurves(self):
        curves = []
        consumed = [False] * len(self.splines)
        for knot in range(self.pointCount):
            if not self._Labelled(knot):
                continue
            for start in self.incident[knot]:
                if consumed[start]:
                    continue
                chain = []
                current, spline = knot, start
                while True:
                    consumed[spline] = True
                    chain.append(spline)
                    nextKnot = self._Other(spline, current)
                    current = nextKnot
                    if self._Labelled(nextKnot):
                        break
                    following = [s for s in self.incident[nextKnot]
                                 if s != spline and not consumed[s]]
                    if not following:
                        break
                    spline = following[0]
                curves.append((knot, current, chain, False))
        # Whatever is left is a cycle of interior knots.
        for index in range(len(self.splines)):
            if consumed[index]:
                continue
            origin = self.splines[index][0]
            chain, current, spline = [], origin, index
            while True:
                consumed[spline] = True
                chain.append(spline)
                nextKnot = self._Other(spline, current)
                current = nextKnot
                if nextKnot == origin:
                    break
                following = [s for s in self.incident[nextKnot]
                             if s != spline and not consumed[s]]
                if not following:
                    break
                spline = following[0]
            curves.append((origin, current, chain, True))
        return curves

    def CountsByKind(self):
        counts = {}
        for kind in self.kind:
            counts[kind] = counts.get(kind, 0) + 1
        return counts

    def IsolatedCurves(self):
        """Curves with no intersection at either end.

        §3 gives these a rotation-only deformation gradient, so they cannot
        express any width or twist change. Almost always this means the
        artist meant to weld an endpoint and did not.
        """
        out = []
        for start, end, chain, closed in self.curves:
            if closed:
                out.append((start, end, chain, closed))
            elif (self.kind[start] != KIND_INTERSECTION and
                  self.kind[end] != KIND_INTERSECTION):
                out.append((start, end, chain, closed))
        return out


def EvalBezier(p0, p1, p2, p3, t):
    u = 1.0 - t
    return (Gf.Vec3f(p0) * (u * u * u) + Gf.Vec3f(p1) * (3.0 * u * u * t) +
            Gf.Vec3f(p2) * (3.0 * u * t * t) + Gf.Vec3f(p3) * (t * t * t))


# ---------------------------------------------------------------------------
# editing operations
#
# Module level, taking the prim, so every one of them is exercisable without
# a display, a QApplication, or a single widget -- which is what
# tests/testUsdviewCurvenetAuthoring.py does. The panel below is a thin shell
# over these.
# ---------------------------------------------------------------------------

def CreateCurvenet(stage, parent, name="Curvenet"):
    path = parent.GetPath().AppendChild(MakeUniqueName(parent, name))
    prim = stage.DefinePrim(path, CURVENET_TYPE)
    SetPoints(prim, [])
    SetSplines(prim, [])
    prim.CreateAttribute("rigExec:basis",
                         Sdf.ValueTypeNames.Token).Set("bezier")
    prim.CreateAttribute("rigExec:samplesPerSpline",
                         Sdf.ValueTypeNames.Int).Set(5)
    return prim


def AppendKnot(prim, point):
    points = GetPoints(prim)
    points.append(Gf.Vec3f(point))
    SetPoints(prim, points)
    return len(points) - 1


def AddSpline(prim, knotA, knotB):
    """
    Adds one cubic spline between two EXISTING pool entries.

    The handles are new points at the thirds, giving the straight spline; the
    artist shapes them afterwards. The knots are REUSED rather than
    duplicated -- that reuse is the connectivity, and it is what turns a
    shared endpoint into an intersection.
    """
    points = GetPoints(prim)
    a, b = Gf.Vec3f(points[knotA]), Gf.Vec3f(points[knotB])
    h0 = len(points)
    points.append(a + (b - a) * (1.0 / 3.0))
    h1 = len(points)
    points.append(a + (b - a) * (2.0 / 3.0))
    SetPoints(prim, points)
    SetSplines(prim, GetSplines(prim) + [knotA, h0, h1, knotB])
    return len(GetSplines(prim)) // 4 - 1


def MoveKnot(prim, knot, position):
    """Moves a knot and carries its handles, or the curve tears away."""
    points = GetPoints(prim)
    if knot < 0 or knot >= len(points):
        return
    position = Gf.Vec3f(position)
    delta = position - Gf.Vec3f(points[knot])
    points[knot] = position
    for a, h0, h1, b in Topology(len(points), GetSplines(prim)).splines:
        if a == knot:
            points[h0] = Gf.Vec3f(points[h0]) + delta
        if b == knot:
            points[h1] = Gf.Vec3f(points[h1]) + delta
    SetPoints(prim, points)


def WeldKnots(prim, knotA, knotB):
    """
    Merges two endpoints into one pool entry.

    The only operation that CREATES an intersection, because an intersection
    is nothing but an endpoint three or more splines name.
    """
    if knotA == knotB:
        return False
    splines = GetSplines(prim)
    keep, drop = min(knotA, knotB), max(knotA, knotB)
    rewritten = []
    for i in range(0, len(splines) - 3, 4):
        spline = list(splines[i:i + 4])
        for slot in (0, 3):
            if spline[slot] == drop:
                spline[slot] = keep
        rewritten.append(spline)
    points = GetPoints(prim)
    # The survivor lands at the midpoint so neither curve jumps.
    points[keep] = (Gf.Vec3f(points[keep]) + Gf.Vec3f(points[drop])) * 0.5
    SetPoints(prim, points)
    SetSplines(prim, [i for spline in rewritten for i in spline])
    return True


def BreakKnot(prim, knot):
    """Duplicates a shared endpoint so each spline after the first gets
    its own copy, undoing a weld."""
    points = GetPoints(prim)
    rewritten = list(GetSplines(prim))
    first = True
    broke = 0
    for i in range(0, len(rewritten) - 3, 4):
        for slot in (0, 3):
            if rewritten[i + slot] != knot:
                continue
            if first:
                first = False   # one spline keeps the original
                continue
            points.append(Gf.Vec3f(points[knot]))
            rewritten[i + slot] = len(points) - 1
            broke += 1
    SetPoints(prim, points)
    SetSplines(prim, rewritten)
    return broke


def SplitSpline(prim, splineIndex):
    """
    Subdivides one spline at its midpoint by de Casteljau.

    Exact: the curve does not move when it gains a knot, which is what makes
    this safe to use on a net that is already bound.
    """
    splines = GetSplines(prim)
    if splineIndex < 0 or (splineIndex + 1) * 4 > len(splines):
        return False
    points = GetPoints(prim)
    a, h0, h1, b = splines[splineIndex * 4:splineIndex * 4 + 4]
    p0, p1 = Gf.Vec3f(points[a]), Gf.Vec3f(points[h0])
    p2, p3 = Gf.Vec3f(points[h1]), Gf.Vec3f(points[b])
    q0, q1, q2 = (p0 + p1) * 0.5, (p1 + p2) * 0.5, (p2 + p3) * 0.5
    r0, r1 = (q0 + q1) * 0.5, (q1 + q2) * 0.5
    mid = (r0 + r1) * 0.5

    base = len(points)
    points.extend([q0, r0, mid, r1, q2])
    SetPoints(prim, points)
    rewritten = (list(splines[:splineIndex * 4]) +
                 [a, base, base + 1, base + 2] +
                 [base + 2, base + 3, base + 4, b] +
                 list(splines[splineIndex * 4 + 4:]))
    SetSplines(prim, rewritten)
    return True


def DeleteSpline(prim, splineIndex):
    splines = GetSplines(prim)
    if splineIndex < 0 or (splineIndex + 1) * 4 > len(splines):
        return False
    SetSplines(prim, list(splines[:splineIndex * 4]) +
               list(splines[splineIndex * 4 + 4:]))
    return True


def ProjectKnots(prim, mesh):
    """Snaps every pool point onto the surface."""
    points = GetPoints(prim)
    moved = 0
    for index, point in enumerate(points):
        closest = ClosestPointOnMesh(mesh, Gf.Vec3d(point))
        if closest is None:
            continue
        if (closest - Gf.Vec3d(point)).GetLength() > 1e-9:
            moved += 1
        points[index] = Gf.Vec3f(closest)
    SetPoints(prim, points)
    return moved


def FlattenTangents(prim, mesh):
    """
    Projects each handle into the surface tangent plane at its knot.

    §3 initializes handles "perpendicular to the surface normals"; flattening
    restores that after editing, keeping the net lying along the surface
    instead of arcing off it.
    """
    points = GetPoints(prim)
    for a, h0, h1, b in Topology(len(points), GetSplines(prim)).splines:
        for knot, handle in ((a, h0), (b, h1)):
            normal = SurfaceNormalAt(mesh, Gf.Vec3d(points[knot]))
            if normal is None:
                continue
            offset = Gf.Vec3d(points[handle]) - Gf.Vec3d(points[knot])
            offset -= normal * Gf.Dot(offset, normal)
            points[handle] = Gf.Vec3f(Gf.Vec3d(points[knot]) + offset)
    SetPoints(prim, points)


def BindCurvenet(stage, curvenet, mesh, rig):
    """Creates the RigExecCurvenetMover that makes the net deform the mesh."""
    movers = stage.DefinePrim(rig.GetPath().AppendChild("Movers"), "Scope")
    geometry = stage.DefinePrim(
        movers.GetPath().AppendChild("Geometry"), "Scope")
    name = MakeUniqueName(geometry, "ProfileMover")
    mover = stage.DefinePrim(geometry.GetPath().AppendChild(name), MOVER_TYPE)
    mover.CreateRelationship("rigExec:curvenet").SetTargets(
        [curvenet.GetPath()])
    mover.CreateRelationship("rigExec:moves").SetTargets(
        [mesh.GetPath().AppendProperty("points")])
    mover.CreateAttribute("inputs:strength", Sdf.ValueTypeNames.Float).Set(1.0)
    return mover


def CheckBindPreconditions(stage, mesh, rig):
    """
    The two engine constraints a curvenet-driven mesh has to satisfy.

    Both fail SILENTLY if missed -- the rig looks bound and the geometry comes
    out wrong -- so they are reported at bind time, where the artist can still
    act on them.
    """
    warnings = []
    normals = mesh.GetAttribute("normals")
    if normals and normals.GetMetadata("interpolation") == "faceVarying":
        warnings.append(
            "normals are faceVarying, which defeats derived normal "
            "maintenance; use vertex interpolation")
    if rig:
        assetRoot = rig.GetParent()
        if assetRoot and assetRoot.IsValid():
            cache = UsdGeom.XformCache()
            relative = cache.ComputeRelativeTransform(mesh, assetRoot)[0]
            if not Gf.IsClose(relative, Gf.Matrix4d(1.0), 1e-6):
                warnings.append(
                    "the mesh is not identity relative to the asset root; "
                    "point movers apply asset-space values to local points")
    return warnings


# ---------------------------------------------------------------------------
# picking
# ---------------------------------------------------------------------------

class SurfacePicker(object):
    """
    Turns a viewport click into a point on the surface.

    Wraps the stage view's own pick so the caller gets hitPoint/hitNormal
    rather than usdview's signalPrimSelected, whose point is not usable
    (see the module docstring).
    """

    def __init__(self, usdviewApi):
        self._api = usdviewApi

    def StageView(self):
        try:
            return self._api._UsdviewApi__appController._stageView
        except AttributeError:
            return None

    def Project(self, worldPoints):
        """
        World points -> viewport pixels, in the same PHYSICAL-pixel space the
        pick frustum uses.

        This is what makes "click on that knot" work. Matching a click to a
        knot by WORLD distance cannot: any tolerance small enough to separate
        two neighbouring knots is a handful of pixels on screen, and any
        tolerance big enough to hit reliably swallows its neighbours. Screen
        distance is the thing the artist is actually aiming with.

        Returns a list of (x, y) or None per point (None when behind the eye).
        """
        view = self._api._UsdviewApi__appController._stageView \
            if hasattr(self._api, "_UsdviewApi__appController") else None
        if view is None:
            return [None] * len(worldPoints)
        camera, _ = view.resolveCamera()
        if camera is None:
            return [None] * len(worldPoints)
        frustum = camera.frustum
        matrix = frustum.ComputeViewMatrix() * frustum.ComputeProjectionMatrix()
        viewport = view.computeWindowViewport()

        out = []
        for point in worldPoints:
            clip = Gf.Vec4d(point[0], point[1], point[2], 1.0) * matrix
            if clip[3] <= 1e-9:
                out.append(None)       # behind the eye
                continue
            ndcX, ndcY = clip[0] / clip[3], clip[1] / clip[3]
            # The exact inverse of computePickFrustum's mapping.
            out.append(((ndcX + 1.0) * 0.5 * viewport[2] + viewport[0],
                        (1.0 - ndcY) * 0.5 * viewport[3] + viewport[1]))
        return out

    def Pick(self, x, y):
        """Returns (primPath, Gf.Vec3d point, Gf.Vec3d normal) or None."""
        view = self.StageView()
        if view is None:
            return None
        try:
            inBounds, frustum = view.computePickFrustum(x, y)
            if not inBounds:
                return None
            hits = view.pick(frustum)
        except Exception as error:
            Tf.Warn("curvenetUI: pick failed: %s" % error)
            return None
        if not hits:
            return None
        hit = hits[0]
        path = getattr(hit, "hitPrimPath", None)
        point = getattr(hit, "hitPoint", None)
        if path is None or point is None:
            return None
        normal = getattr(hit, "hitNormal", None)
        return (path, Gf.Vec3d(point[0], point[1], point[2]),
                Gf.Vec3d(normal[0], normal[1], normal[2]) if normal is not None
                else Gf.Vec3d(0, 0, 1))


# ---------------------------------------------------------------------------
# the panel
# ---------------------------------------------------------------------------

MODE_OFF = "off"
MODE_DRAW = "draw"
MODE_MOVE = "move"
MODE_WELD = "weld"

MODE_HELP = {
    MODE_OFF: "Editing off. The viewport behaves normally.",
    MODE_DRAW: ("Click the model to place a knot; drag out of it to shape the "
                "outgoing tangent. Click an existing knot to WELD to it, "
                "which is how intersections are made. Esc ends the chain."),
    MODE_MOVE: ("Drag a knot; it re-projects onto the surface. Hold Shift to "
                "move it freely off the surface."),
    MODE_WELD: "Click two knots in turn to merge them into one.",
}


class CurvenetPanel(QtWidgets.QWidget):

    _instance = None

    @classmethod
    def GetInstance(cls, usdviewApi):
        if cls._instance is None:
            cls._instance = CurvenetPanel(usdviewApi)
        return cls._instance

    def __init__(self, usdviewApi, parent=None):
        super(CurvenetPanel, self).__init__(parent)
        self._api = usdviewApi
        self._picker = SurfacePicker(usdviewApi)
        self._mode = MODE_OFF
        self._curvenetPath = None
        self._chainKnot = -1        # open chain's trailing knot, or -1
        self._pendingWeld = -1
        self._dragKnot = -1
        self._selected = -1
        self._dragHandle = None     # (knotIndex, anchorPoint) while drawing
        self._filterInstalled = False
        self._listener = None

        self.setWindowTitle("RigExec Curvenet Authoring")
        self.setWindowFlags(QtCore.Qt.Window)
        self.resize(430, 640)
        self._BuildUI()
        self._Refresh()

        self._listener = Tf.Notice.Register(
            Usd.Notice.ObjectsChanged, self._OnObjectsChanged,
            self._api.stage)

    # -- ui -------------------------------------------------------------

    def _BuildUI(self):
        layout = QtWidgets.QVBoxLayout(self)

        # target curvenet
        box = QtWidgets.QGroupBox("Curvenet")
        form = QtWidgets.QVBoxLayout(box)
        row = QtWidgets.QHBoxLayout()
        self._netCombo = QtWidgets.QComboBox()
        self._netCombo.currentIndexChanged.connect(self._OnNetChanged)
        row.addWidget(self._netCombo, 1)
        newButton = QtWidgets.QPushButton("New")
        newButton.setToolTip(
            "Create a RigExecCurvenet under the selected prim's parent")
        newButton.clicked.connect(self._OnNewCurvenet)
        row.addWidget(newButton)
        form.addLayout(row)
        layout.addWidget(box)

        # modes
        box = QtWidgets.QGroupBox("Tool")
        grid = QtWidgets.QGridLayout(box)
        self._modeButtons = {}
        for column, (mode, label) in enumerate(
                [(MODE_OFF, "Off"), (MODE_DRAW, "Draw"),
                 (MODE_MOVE, "Move"), (MODE_WELD, "Weld")]):
            button = QtWidgets.QToolButton()
            button.setText(label)
            button.setCheckable(True)
            button.setChecked(mode == MODE_OFF)
            button.clicked.connect(
                lambda checked=False, m=mode: self._SetMode(m))
            grid.addWidget(button, 0, column)
            self._modeButtons[mode] = button
        self._help = QtWidgets.QLabel(MODE_HELP[MODE_OFF])
        self._help.setWordWrap(True)
        grid.addWidget(self._help, 1, 0, 1, 4)
        layout.addWidget(box)

        # structure readout
        box = QtWidgets.QGroupBox("Structure (paper section 3)")
        column = QtWidgets.QVBoxLayout(box)
        self._structure = QtWidgets.QLabel("-")
        self._structure.setWordWrap(True)
        self._structure.setTextFormat(QtCore.Qt.RichText)
        column.addWidget(self._structure)
        layout.addWidget(box)

        # operations
        box = QtWidgets.QGroupBox("Operations")
        grid = QtWidgets.QGridLayout(box)
        operations = [
            ("Break knot", self._OnBreak,
             "Split a shared endpoint back into one point per spline"),
            ("Split spline", self._OnSplit,
             "Subdivide the last spline at its midpoint"),
            ("Delete spline", self._OnDeleteSpline,
             "Remove the last spline of the chain"),
            ("Project knots", self._OnProject,
             "Snap every knot onto the bound surface"),
            ("Flatten tangents", self._OnFlatten,
             "Project each handle into the surface tangent plane at its knot"),
            ("Close loop", self._OnCloseLoop,
             "Join the open chain's two ends with a new spline"),
        ]
        for index, (label, slot, tip) in enumerate(operations):
            button = QtWidgets.QPushButton(label)
            button.setToolTip(tip)
            button.clicked.connect(slot)
            grid.addWidget(button, index // 2, index % 2)
        layout.addWidget(box)

        # binding
        box = QtWidgets.QGroupBox("Profile Mover")
        column = QtWidgets.QVBoxLayout(box)
        self._bindLabel = QtWidgets.QLabel(
            "Select the mesh to deform, then bind.")
        self._bindLabel.setWordWrap(True)
        column.addWidget(self._bindLabel)
        bind = QtWidgets.QPushButton("Bind selected mesh")
        bind.setToolTip("Create a RigExecCurvenetMover writing the selected "
                        "mesh's points")
        bind.clicked.connect(self._OnBind)
        column.addWidget(bind)
        layout.addWidget(box)

        # display
        box = QtWidgets.QGroupBox("Display")
        column = QtWidgets.QVBoxLayout(box)
        self._showNet = QtWidgets.QCheckBox("Draw the net in the viewport")
        self._showNet.setChecked(True)
        self._showNet.stateChanged.connect(lambda _: self._UpdateDisplay())
        column.addWidget(self._showNet)
        note = QtWidgets.QLabel(
            "Drawn from the AUTHORED pool, in the session layer, so it shows "
            "the net as designed rather than as posed.")
        note.setWordWrap(True)
        note.setStyleSheet("color: gray;")
        column.addWidget(note)
        layout.addWidget(box)

        layout.addStretch(1)
        self._status = QtWidgets.QLabel("")
        self._status.setWordWrap(True)
        layout.addWidget(self._status)

    # -- state ----------------------------------------------------------

    def _Stage(self):
        return self._api.stage

    def _Curvenet(self):
        stage = self._Stage()
        if not stage or not self._curvenetPath:
            return None
        prim = stage.GetPrimAtPath(self._curvenetPath)
        return prim if prim and prim.IsValid() else None

    def _Refresh(self):
        stage = self._Stage()
        nets = FindCurvenetPrims(stage)
        self._netCombo.blockSignals(True)
        self._netCombo.clear()
        for net in nets:
            self._netCombo.addItem(net.GetPath().pathString)
        if self._curvenetPath:
            index = self._netCombo.findText(self._curvenetPath.pathString)
            if index >= 0:
                self._netCombo.setCurrentIndex(index)
            elif nets:
                self._curvenetPath = nets[0].GetPath()
                self._netCombo.setCurrentIndex(0)
            else:
                self._curvenetPath = None
        elif nets:
            self._curvenetPath = nets[0].GetPath()
            self._netCombo.setCurrentIndex(0)
        self._netCombo.blockSignals(False)
        self._UpdateStructure()
        self._UpdateDisplay()

    def _UpdateStructure(self):
        prim = self._Curvenet()
        if not prim:
            self._structure.setText(
                "<i>No curvenet. Press New to make one.</i>")
            return
        points = GetPoints(prim)
        topology = Topology(len(points), GetSplines(prim))
        counts = topology.CountsByKind()
        isolated = topology.IsolatedCurves()

        lines = [
            "<b>%d</b> pool points &nbsp; <b>%d</b> splines &nbsp; "
            "<b>%d</b> curves" % (len(points), len(topology.splines),
                                  len(topology.curves)),
            "intersections <b>%d</b> &nbsp; anchors <b>%d</b> &nbsp; "
            "interior <b>%d</b> &nbsp; handles <b>%d</b>"
            % (counts.get(KIND_INTERSECTION, 0), counts.get(KIND_ANCHOR, 0),
               counts.get(KIND_INTERIOR, 0), counts.get(KIND_HANDLE, 0)),
        ]
        if counts.get(KIND_INTERSECTION, 0) == 0 and topology.splines:
            lines.append(
                "<span style='color:#c86400'>No intersections. Every curve "
                "is isolated, so section 3 gives it a rotation-only "
                "gradient: it cannot change width or twist. Weld endpoints "
                "to build a net.</span>")
        elif isolated:
            lines.append(
                "<span style='color:#c86400'>%d isolated curve(s): no "
                "intersection at either end, so rotation only.</span>"
                % len(isolated))
        if counts.get(KIND_UNUSED, 0):
            lines.append("<span style='color:gray'>%d unused pool point(s)"
                         "</span>" % counts[KIND_UNUSED])
        self._structure.setText("<br>".join(lines))

    def _SetStatus(self, text):
        self._status.setText(text)

    # -- modes ----------------------------------------------------------

    def _SetMode(self, mode):
        self._mode = mode
        for name, button in self._modeButtons.items():
            button.setChecked(name == mode)
        self._help.setText(MODE_HELP[mode])
        self._chainKnot = -1
        self._pendingWeld = -1
        if mode == MODE_OFF:
            self._RemoveFilter()
        else:
            self._InstallFilter()

    def _InstallFilter(self):
        view = self._picker.StageView()
        if view is not None and not self._filterInstalled:
            view.installEventFilter(self)
            self._filterInstalled = True

    def _RemoveFilter(self):
        view = self._picker.StageView()
        if view is not None and self._filterInstalled:
            view.removeEventFilter(self)
            self._filterInstalled = False

    # usdview claims Alt (and Meta, for the platforms that swallow Alt) as the
    # camera-manipulation modifier. An editing tool that ate those events
    # would leave the artist unable to orbit while drawing, which makes
    # drawing on a character impossible.
    _CAMERA_MODIFIERS = (QtCore.Qt.AltModifier | QtCore.Qt.MetaModifier)

    def eventFilter(self, obj, event):
        if self._mode == MODE_OFF:
            return False
        modifiers = getattr(event, "modifiers", None)
        if modifiers is not None and (modifiers() & self._CAMERA_MODIFIERS):
            return False
        kind = event.type()
        if kind == QtCore.QEvent.MouseButtonPress:
            if event.button() == QtCore.Qt.LeftButton:
                return self._OnPress(event)
        elif kind == QtCore.QEvent.MouseMove:
            if event.buttons() & QtCore.Qt.LeftButton:
                return self._OnDrag(event)
        elif kind == QtCore.QEvent.MouseButtonRelease:
            if event.button() == QtCore.Qt.LeftButton:
                return self._OnRelease(event)
        elif kind == QtCore.QEvent.KeyPress:
            if event.key() == QtCore.Qt.Key_Escape:
                self._chainKnot = -1
                self._SetStatus("Chain ended.")
                return True
        return False

    def _Position(self, event):
        """
        Mouse position in the coordinates the pick frustum expects.

        Qt reports widget-local LOGICAL pixels; computePickFrustum divides by
        computeWindowViewport, which is in PHYSICAL pixels. usdview's own
        mousePressEvent bridges the two by multiplying by devicePixelRatioF
        (stageView.py: "multiplying by devicePixelRatio is only necessary
        because this is a QGLWidget"), and anything picking by hand has to do
        the same.

        Skipping it costs nothing on a 1.0-ratio display and silently breaks
        the whole tool on a HiDPI one: every pick lands at a fraction of
        where the artist clicked, usually off the model, so placing a knot
        appears to do nothing at all.
        """
        try:
            position = event.position()
            x, y = position.x(), position.y()
        except AttributeError:
            x, y = event.x(), event.y()
        view = self._picker.StageView()
        ratio = view.devicePixelRatioF() if view is not None else 1.0
        return int(round(x * ratio)), int(round(y * ratio))

    # How near a click has to land, in LOGICAL pixels. A screen radius, not a
    # world one: a world tolerance either misses at normal zoom or swallows
    # neighbouring knots, and it changes meaning with every camera move.
    _PICK_RADIUS_PIXELS = 14

    def _LocalToWorld(self):
        """
        The curvenet's transform.

        Its `points` are LOCAL, while picks come back in world space, so
        every comparison and every placement has to cross that boundary.
        Identity for a net under an untransformed asset root -- which is the
        only case the engine's point movers support anyway -- but wrong
        everywhere else if skipped.
        """
        prim = self._Curvenet()
        if not prim:
            return Gf.Matrix4d(1.0)
        return UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
            self._api.frame)

    def _WorldPoints(self):
        matrix = self._LocalToWorld()
        return [matrix.Transform(Gf.Vec3d(p)) for p in GetPoints(
            self._Curvenet())]

    def _ToLocal(self, worldPoint):
        return Gf.Vec3f(self._LocalToWorld().GetInverse().Transform(
            Gf.Vec3d(worldPoint)))

    def _NearestControlPoint(self, x, y, knotsOnly=False):
        """
        Pool point nearest the click in SCREEN space, or -1.

        \\p x and \\p y are physical pixels, as _Position returns them.
        """
        prim = self._Curvenet()
        if not prim:
            return -1
        points = GetPoints(prim)
        if not points:
            return -1
        topology = Topology(len(points), GetSplines(prim))
        projected = self._picker.Project(self._WorldPoints())
        view = self._picker.StageView()
        ratio = view.devicePixelRatioF() if view is not None else 1.0
        limit = self._PICK_RADIUS_PIXELS * ratio

        # Everything under the cursor, then the one NEAREST THE CAMERA.
        #
        # Screen distance alone picks through the model: a net wraps right
        # round a head, so the knots on the far side project into the same
        # pixels as the near ones and are just as likely to win. Depth is
        # what the artist means by "that one".
        world = self._WorldPoints()
        eye = Gf.Vec3d(0.0)
        camera, _ = view.resolveCamera() if view is not None else (None, None)
        if camera is not None:
            eye = camera.frustum.GetPosition()

        # Ranked by screen distance in coarse buckets, then knot-over-handle,
        # then nearer the camera.
        #
        # Distance has to lead. Ranking by depth first meant a tangent handle
        # a millimetre nearer the eye stole every click aimed at its own
        # knot; ranking knots absolutely first meant handles could never be
        # grabbed at all, because on a real net every handle has a knot
        # within the pick radius -- and then tangents cannot be shaped.
        #
        # The bucket is what makes the other two terms matter: points landing
        # within a couple of pixels of each other are, as far as the artist
        # aiming is concerned, in the same place, and THERE the structure
        # (knot) and the depth (nearer) decide. A ring seen down its axis
        # projects its front and back knots onto the same pixel, so that case
        # is ordinary, not exotic.
        bucket = max(2.0 * ratio, 1.0)
        best, bestRank = -1, None
        for index, screen in enumerate(projected):
            if screen is None:
                continue
            isHandle = topology.kind[index] == KIND_HANDLE
            if knotsOnly and isHandle:
                continue
            distance = math.hypot(screen[0] - x, screen[1] - y)
            if distance > limit:
                continue
            rank = (int(distance / bucket),
                    1 if isHandle else 0,
                    (world[index] - eye).GetLength(),
                    distance)
            if best < 0 or rank < bestRank:
                best, bestRank = index, rank
        return best

    def _KnotTolerance(self):
        """
        How near a click has to be to count as hitting an existing knot.

        Measured against the geometry, never against stage units. The unit
        guess this replaced returned metersPerUnit * 5 whenever the net had
        fewer than two points -- 5.0 on a character 0.6 units tall -- so on a
        NEW net every click after the first landed "on" the first knot and
        welded to it. No second knot was ever created, no spline ever formed,
        and the tool drew nothing at all.
        """
        prim = self._Curvenet()
        if prim:
            points = GetPoints(prim)
            if len(points) > 1:
                lo = Gf.Vec3d(points[0])
                hi = Gf.Vec3d(points[0])
                for p in points:
                    for k in range(3):
                        lo[k] = min(lo[k], p[k])
                        hi[k] = max(hi[k], p[k])
                diagonal = (hi - lo).GetLength()
                if diagonal > 1e-9:
                    return diagonal * 0.03
        # A new or single-knot net has no scale of its own, so take it from
        # the surface being drawn on.
        for candidate in (self._BoundMesh(),) + tuple(self._api.selectedPrims):
            if not candidate:
                continue
            extent = candidate.GetAttribute("extent")
            value = extent.Get() if extent else None
            if value and len(value) == 2:
                diagonal = (Gf.Vec3d(value[1]) - Gf.Vec3d(value[0])).GetLength()
                if diagonal > 1e-9:
                    return diagonal * 0.02
        stage = self._Stage()
        if stage:
            default = stage.GetDefaultPrim()
            if default:
                bounds = UsdGeom.BBoxCache(
                    self._api.frame,
                    [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
                box = bounds.ComputeWorldBound(default).ComputeAlignedRange()
                if not box.IsEmpty():
                    diagonal = (box.GetMax() - box.GetMin()).GetLength()
                    if diagonal > 1e-9:
                        return diagonal * 0.02
        return 0.01

    def _OnPress(self, event):
        prim = self._Curvenet()
        if not prim:
            self._SetStatus("No curvenet selected — press New.")
            return False
        x, y = self._Position(event)
        # The surface hit is only needed to PLACE a point. Selecting one is
        # pure screen-space projection, and requiring a hit for it made knots
        # near the silhouette unselectable -- they float just outside the
        # surface (§3), so a click on the knot can easily miss the mesh.
        hit = self._picker.Pick(x, y)

        if self._mode == MODE_WELD:
            existing = self._NearestControlPoint(x, y, knotsOnly=True)
            if existing < 0:
                self._SetStatus("No knot within %d pixels of the click."
                                % self._PICK_RADIUS_PIXELS)
                return True
            if self._pendingWeld < 0:
                self._pendingWeld = existing
                self._selected = existing
                self._SetStatus("Weld: knot %d picked, now pick the second."
                                % existing)
                self._UpdateDisplay()
            else:
                self._Weld(self._pendingWeld, existing)
                self._pendingWeld = -1
            return True

        if self._mode == MODE_MOVE:
            # Handles are selectable too: shaping a tangent is the same
            # gesture as moving a knot, and a tool that can only grab knots
            # cannot shape a curve at all.
            self._dragKnot = self._NearestControlPoint(x, y)
            self._selected = self._dragKnot
            if self._dragKnot < 0:
                self._SetStatus(
                    "No control point within %d pixels. Knots are the larger "
                    "markers; handles are the small dim ones."
                    % self._PICK_RADIUS_PIXELS)
            else:
                topology = Topology(len(GetPoints(prim)), GetSplines(prim))
                self._SetStatus(
                    "Dragging %s %d — release to drop it on the surface."
                    % (topology.kind[self._dragKnot], self._dragKnot))
            self._UpdateDisplay()
            return True

        if self._mode == MODE_DRAW:
            existing = self._NearestControlPoint(x, y, knotsOnly=True)
            if existing >= 0:
                knot = existing
                welded = True
            elif hit is None:
                # Only PLACING needs the surface.
                self._SetStatus(
                    "Clicked off the model (%d, %d) — a new knot is placed ON "
                    "the surface, so aim at the geometry." % (x, y))
                return True
            else:
                knot = AppendKnot(prim, self._ToLocal(hit[1]))
                welded = False
            if self._chainKnot >= 0 and self._chainKnot != knot:
                AddSpline(prim, self._chainKnot, knot)
                self._SetStatus(
                    "Welded to knot %d, making an intersection." % knot
                    if welded else "Added a spline to knot %d." % knot)
            else:
                self._SetStatus("Chain started at knot %d." % knot)
            self._chainKnot = knot
            self._selected = knot
            self._dragHandle = (knot, hit[1] if hit else None)
            self._Refresh()
            return True
        return False

    def _OnDrag(self, event):
        x, y = self._Position(event)
        prim = self._Curvenet()
        if not prim:
            return False

        if self._mode == MODE_MOVE and self._dragKnot >= 0:
            hit = self._picker.Pick(x, y)
            if hit is None:
                self._SetStatus("Dragged off the model — knots follow the "
                                "surface, so stay over the geometry.")
                return True
            _, point, normal = hit
            if event.modifiers() & QtCore.Qt.ShiftModifier:
                # Keep the point floating at the height above the surface it
                # already had, instead of snapping it flat. §3 assumes the
                # curves lie NEAR the surface rather than on it, and §4.3
                # transports that offset, so preserving it while sliding a
                # knot along is a real thing to want.
                mesh = self._BoundMesh()
                current = self._WorldPoints()[self._dragKnot]
                closest = ClosestPointOnMesh(mesh, current) if mesh else None
                height = (current - closest).GetLength() if closest else 0.0
                if height > 1e-9 and normal.GetLength() > 1e-9:
                    point = point + normal.GetNormalized() * height

            local = self._ToLocal(point)
            topology = Topology(len(GetPoints(prim)), GetSplines(prim))
            if topology.kind[self._dragKnot] == KIND_HANDLE:
                # A handle moves alone: dragging its knot's handles with it
                # is what MoveKnot is for, and doing it here would drag the
                # handle by its own delta twice over.
                points = GetPoints(prim)
                points[self._dragKnot] = local
                SetPoints(prim, points)
            else:
                MoveKnot(prim, self._dragKnot, local)
            self._UpdateDisplay()
            return True

        if self._mode == MODE_DRAW and self._dragHandle is not None:
            hit = self._picker.Pick(x, y)
            if hit is None:
                return True
            _, point, _ = hit
            self._ShapeLastTangent(prim, self._ToLocal(point))
            return True
        return False

    def _OnRelease(self, event):
        self._dragKnot = -1
        self._dragHandle = None
        return False

    # -- edits ----------------------------------------------------------

    def _ShapeLastTangent(self, prim, target):
        """Drags the outgoing handle of the chain's last spline."""
        splines = GetSplines(prim)
        if len(splines) < 4:
            return
        points = GetPoints(prim)
        a, h0, h1, b = splines[-4:]
        anchor = Gf.Vec3f(points[a])
        # Mirror the incoming handle so the join stays smooth (G1), which is
        # the pen-tool convention artists expect.
        points[h0] = target
        points[h1] = Gf.Vec3f(points[b]) + (anchor - target) * 0.5
        SetPoints(prim, points)

    def _Weld(self, knotA, knotB):
        prim = self._Curvenet()
        if prim and WeldKnots(prim, knotA, knotB):
            self._SetStatus("Welded %d into %d."
                            % (max(knotA, knotB), min(knotA, knotB)))
            self._Refresh()

    def _OnBreak(self):
        prim = self._Curvenet()
        if not prim:
            return
        topology = Topology(len(GetPoints(prim)), GetSplines(prim))
        shared = [i for i in range(topology.pointCount)
                  if topology.kind[i] == KIND_INTERSECTION]
        if not shared:
            self._SetStatus("Nothing to break: no shared endpoints.")
            return
        knot = shared[-1]
        BreakKnot(prim, knot)
        self._SetStatus("Broke knot %d apart." % knot)
        self._Refresh()

    def _OnSplit(self):
        prim = self._Curvenet()
        if not prim:
            return
        count = len(GetSplines(prim)) // 4
        if count and SplitSpline(prim, count - 1):
            self._SetStatus("Split the last spline at its midpoint.")
            self._Refresh()

    def _OnDeleteSpline(self):
        prim = self._Curvenet()
        if not prim:
            return
        count = len(GetSplines(prim)) // 4
        if count and DeleteSpline(prim, count - 1):
            self._chainKnot = -1
            self._SetStatus("Deleted the last spline. (Its pool points "
                            "remain; unused points are harmless.)")
            self._Refresh()

    def _OnProject(self):
        prim = self._Curvenet()
        mesh = self._BoundMesh()
        if not prim or not mesh:
            self._SetStatus("Bind a mesh first: projection needs a surface.")
            return
        moved = ProjectKnots(prim, mesh)
        self._SetStatus("Projected %d point(s) onto %s."
                        % (moved, mesh.GetPath().name))
        self._Refresh()

    def _OnFlatten(self):
        prim = self._Curvenet()
        mesh = self._BoundMesh()
        if not prim or not mesh:
            self._SetStatus("Bind a mesh first: flatten needs a surface.")
            return
        FlattenTangents(prim, mesh)
        self._SetStatus("Flattened every tangent handle.")
        self._Refresh()

    def _OnCloseLoop(self):
        prim = self._Curvenet()
        if not prim:
            return
        topology = Topology(len(GetPoints(prim)), GetSplines(prim))
        anchors = [i for i in range(topology.pointCount)
                   if topology.kind[i] == KIND_ANCHOR]
        if len(anchors) < 2:
            self._SetStatus("Need two open ends to close.")
            return
        AddSpline(prim, anchors[-1], anchors[0])
        self._SetStatus("Closed the chain.")
        self._Refresh()

    # -- creation and binding -------------------------------------------

    def _OnNewCurvenet(self):
        stage = self._Stage()
        if not stage:
            return
        selected = self._api.selectedPrims
        parent = None
        if selected:
            parent = selected[0].GetParent()
        if parent is None or not parent.IsValid():
            rig = FindRigPrim(stage)
            parent = rig.GetParent() if rig else stage.GetPseudoRoot()
        prim = CreateCurvenet(stage, parent)
        self._curvenetPath = prim.GetPath()
        self._SetStatus("Created %s." % self._curvenetPath)
        self._Refresh()

    def _BoundMesh(self):
        """The mesh a Profile Mover already binds this curvenet to."""
        stage = self._Stage()
        prim = self._Curvenet()
        if not stage or not prim:
            return None
        for candidate in stage.Traverse():
            if candidate.GetTypeName() != MOVER_TYPE:
                continue
            rel = candidate.GetRelationship("rigExec:curvenet")
            targets = rel.GetTargets() if rel else []
            if not targets or targets[0].GetPrimPath() != prim.GetPath():
                continue
            moves = candidate.GetRelationship("rigExec:moves")
            movesTargets = moves.GetTargets() if moves else []
            if movesTargets:
                mesh = stage.GetPrimAtPath(movesTargets[0].GetPrimPath())
                if mesh and mesh.IsValid():
                    return mesh
        # Fall back to whatever the artist has selected.
        for candidate in self._api.selectedPrims:
            if candidate.IsA(UsdGeom.Mesh):
                return candidate
        return None

    def _OnBind(self):
        stage = self._Stage()
        prim = self._Curvenet()
        if not stage or not prim:
            self._SetStatus("No curvenet.")
            return
        meshes = [p for p in self._api.selectedPrims if p.IsA(UsdGeom.Mesh)]
        if not meshes:
            self._SetStatus("Select the mesh to deform first.")
            return
        mesh = meshes[0]
        rig = FindRigPrim(stage)
        if not rig:
            self._SetStatus("No RigExecRig on this stage.")
            return
        BindCurvenet(stage, prim, mesh, rig)
        warnings = CheckBindPreconditions(stage, mesh, rig)
        message = "Bound %s to %s." % (prim.GetPath().name,
                                       mesh.GetPath().name)
        if warnings:
            message += "  WARNING: " + "; ".join(warnings) + "."
        self._bindLabel.setText(message)
        self._SetStatus(message)
        self._Refresh()

    # -- display ---------------------------------------------------------

    def _DisplayPath(self):
        prim = self._Curvenet()
        if not prim:
            return None
        return prim.GetPath().AppendChild("__curvenetDisplay")

    def _KnotDisplayPath(self):
        prim = self._Curvenet()
        if not prim:
            return None
        return prim.GetPath().AppendChild("__curvenetKnots")

    @staticmethod
    def _SetShown(prim, shown):
        """
        Shows or hides a display prim, rather than creating and destroying it.

        Adding or removing a prim RESYNCS the stage, and on a stage carrying
        a RigExecRig every resync re-evaluates the rig -- so a create/destroy
        per redraw meant a full rig re-evaluation on every knot placed, and
        (worse) surfaced a Tf coding error out of that re-evaluation, which
        Python then raised from whatever unrelated call came next.
        Defining these once and thereafter only setting attributes keeps the
        whole authoring loop to value edits, which is what it should have
        been anyway.
        """
        if not prim:
            return
        UsdGeom.Imageable(prim).CreateVisibilityAttr().Set(
            UsdGeom.Tokens.inherited if shown else UsdGeom.Tokens.invisible)

    def _UpdateDisplay(self):
        """
        Draws the net in the SESSION layer: the splines as BasisCurves and
        the knots as Points, colour-coded by their section 3 role.

        Session layer keeps it out of the artist's file entirely -- it is a
        viewport aid, regenerated from the pool on every edit, and nothing
        downstream should ever read it.

        Purpose is DEFAULT, not "guide". A guide only draws when the viewer
        has guides enabled, which meant the thing the artist was actively
        drawing was invisible unless they knew to go and switch it on.
        """
        stage = self._Stage()
        path = self._DisplayPath()
        knotPath = self._KnotDisplayPath()
        if not stage or path is None:
            return
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            if not self._showNet.isChecked():
                self._SetShown(stage.GetPrimAtPath(path), False)
                self._SetShown(stage.GetPrimAtPath(knotPath), False)
                return
            prim = self._Curvenet()
            points = GetPoints(prim)
            splines = GetSplines(prim)
            topology = Topology(len(points), splines)
            scale = self._KnotTolerance()

            # --- the splines -------------------------------------------
            vertices, counts = [], []
            for a, h0, h1, b in topology.splines:
                if max(a, h0, h1, b) >= len(points):
                    continue
                steps = 12
                for i in range(steps + 1):
                    vertices.append(EvalBezier(points[a], points[h0],
                                               points[h1], points[b],
                                               i / float(steps)))
                counts.append(steps + 1)
            if counts:
                curves = UsdGeom.BasisCurves.Define(stage, path)
                curves.CreatePointsAttr(Vt.Vec3fArray(vertices))
                curves.CreateCurveVertexCountsAttr(Vt.IntArray(counts))
                curves.CreateTypeAttr("linear")
                curves.CreateWrapAttr("nonperiodic")
                curves.CreateWidthsAttr(
                    Vt.FloatArray([scale * 0.12] * len(vertices)))
                curves.SetWidthsInterpolation("vertex")
                curves.CreateDisplayColorAttr(
                    Vt.Vec3fArray([Gf.Vec3f(0.95, 0.35, 1.0)]))
                UsdGeom.Imageable(curves).CreatePurposeAttr("default")
                self._SetShown(curves.GetPrim(), True)
            else:
                # A net with knots but no splines yet: empty it and hide it,
                # never remove it.
                existing = stage.GetPrimAtPath(path)
                if existing:
                    UsdGeom.BasisCurves(existing).CreatePointsAttr(
                        Vt.Vec3fArray())
                    UsdGeom.BasisCurves(existing).CreateCurveVertexCountsAttr(
                        Vt.IntArray())
                    self._SetShown(existing, False)

            # --- the knots ---------------------------------------------
            #
            # Drawn separately so the FIRST click already shows something: a
            # lone knot has no spline yet, and a tool that stays blank until
            # the second click reads as broken. Colour is the section 3
            # classification, which is the thing the artist most needs to see
            # and cannot otherwise tell by looking.
            colours = {
                KIND_INTERSECTION: Gf.Vec3f(0.2, 1.0, 1.0),
                KIND_ANCHOR: Gf.Vec3f(1.0, 0.85, 0.2),
                KIND_INTERIOR: Gf.Vec3f(1.0, 1.0, 1.0),
                KIND_HANDLE: Gf.Vec3f(0.45, 0.35, 0.5),
                KIND_UNUSED: Gf.Vec3f(0.4, 0.4, 0.4),
            }
            knotPositions, knotColours, knotWidths = [], [], []
            for index, point in enumerate(points):
                kind = topology.kind[index]
                knotPositions.append(Gf.Vec3f(point))
                if index == self._selected:
                    # The grabbed point, so the artist can see WHICH one they
                    # got before they start dragging it.
                    knotColours.append(Gf.Vec3f(1.0, 0.25, 0.15))
                    knotWidths.append(scale * 0.62)
                    continue
                knotColours.append(colours.get(kind, colours[KIND_UNUSED]))
                knotWidths.append(
                    scale * (0.22 if kind == KIND_HANDLE else 0.40))
            if knotPositions:
                markers = UsdGeom.Points.Define(stage, knotPath)
                markers.CreatePointsAttr(Vt.Vec3fArray(knotPositions))
                markers.CreateWidthsAttr(Vt.FloatArray(knotWidths))
                markers.SetWidthsInterpolation("vertex")
                displayColor = markers.CreateDisplayColorAttr(
                    Vt.Vec3fArray(knotColours))
                UsdGeom.Primvar(displayColor).SetInterpolation("vertex")
                UsdGeom.Imageable(markers).CreatePurposeAttr("default")
                self._SetShown(markers.GetPrim(), True)
            else:
                existing = stage.GetPrimAtPath(knotPath)
                if existing:
                    UsdGeom.Points(existing).CreatePointsAttr(Vt.Vec3fArray())
                    self._SetShown(existing, False)

    # -- notices ---------------------------------------------------------

    def _OnNetChanged(self, index):
        text = self._netCombo.itemText(index)
        self._curvenetPath = Sdf.Path(text) if text else None
        self._chainKnot = -1
        self._UpdateStructure()
        self._UpdateDisplay()

    def _OnObjectsChanged(self, notice, stage):
        if stage != self._Stage():
            return
        # Ignore our own display writes, or every edit re-enters this.
        ours = [p for p in (self._DisplayPath(), self._KnotDisplayPath())
                if p is not None]
        for changed in list(notice.GetResyncedPaths()) + \
                list(notice.GetChangedInfoOnlyPaths()):
            if any(changed.HasPrefix(p) for p in ours):
                return
        self._UpdateStructure()

    def closeEvent(self, event):
        self._RemoveFilter()
        super(CurvenetPanel, self).closeEvent(event)


# ---------------------------------------------------------------------------
# geometry helpers used by project/flatten
# ---------------------------------------------------------------------------

def _MeshTriangles(mesh):
    points = mesh.GetAttribute("points").Get()
    counts = mesh.GetAttribute("faceVertexCounts").Get()
    indices = mesh.GetAttribute("faceVertexIndices").Get()
    if not points or not counts or not indices:
        return [], []
    triangles = []
    cursor = 0
    for count in counts:
        for i in range(1, count - 1):
            triangles.append((indices[cursor], indices[cursor + i],
                              indices[cursor + i + 1]))
        cursor += count
    return list(points), triangles


def _ClosestOnTriangle(p, a, b, c):
    ab, ac, ap = b - a, c - a, p - a
    d1, d2 = Gf.Dot(ab, ap), Gf.Dot(ac, ap)
    if d1 <= 0.0 and d2 <= 0.0:
        return a
    bp = p - b
    d3, d4 = Gf.Dot(ab, bp), Gf.Dot(ac, bp)
    if d3 >= 0.0 and d4 <= d3:
        return b
    vc = d1 * d4 - d3 * d2
    if vc <= 0.0 and d1 >= 0.0 and d3 <= 0.0:
        return a + ab * (d1 / (d1 - d3) if d1 != d3 else 0.0)
    cp = p - c
    d5, d6 = Gf.Dot(ab, cp), Gf.Dot(ac, cp)
    if d6 >= 0.0 and d5 <= d6:
        return c
    vb = d5 * d2 - d1 * d6
    if vb <= 0.0 and d2 >= 0.0 and d6 <= 0.0:
        return a + ac * (d2 / (d2 - d6) if d2 != d6 else 0.0)
    va = d3 * d6 - d5 * d4
    if va <= 0.0 and (d4 - d3) >= 0.0 and (d5 - d6) >= 0.0:
        denominator = (d4 - d3) + (d5 - d6)
        return b + (c - b) * ((d4 - d3) / denominator if denominator else 0.0)
    denominator = va + vb + vc
    if denominator == 0.0:
        return a
    return a + ab * (vb / denominator) + ac * (vc / denominator)


def ClosestPointOnMesh(mesh, point):
    points, triangles = _MeshTriangles(mesh)
    if not triangles:
        return None
    best, bestDistance = None, None
    for i0, i1, i2 in triangles:
        candidate = _ClosestOnTriangle(
            point, Gf.Vec3d(points[i0]), Gf.Vec3d(points[i1]),
            Gf.Vec3d(points[i2]))
        distance = (candidate - point).GetLength()
        if bestDistance is None or distance < bestDistance:
            best, bestDistance = candidate, distance
    return best


def SurfaceNormalAt(mesh, point):
    points, triangles = _MeshTriangles(mesh)
    if not triangles:
        return None
    best, bestDistance = None, None
    for i0, i1, i2 in triangles:
        a, b, c = (Gf.Vec3d(points[i0]), Gf.Vec3d(points[i1]),
                   Gf.Vec3d(points[i2]))
        candidate = _ClosestOnTriangle(point, a, b, c)
        distance = (candidate - point).GetLength()
        if bestDistance is None or distance < bestDistance:
            normal = Gf.Cross(b - a, c - a)
            if normal.GetLength() > 1e-12:
                best, bestDistance = normal.GetNormalized(), distance
    return best


def OpenCurvenetPanel(usdviewApi):
    panel = CurvenetPanel.GetInstance(usdviewApi)
    panel.show()
    panel.raise_()
    panel.activateWindow()
    return panel
