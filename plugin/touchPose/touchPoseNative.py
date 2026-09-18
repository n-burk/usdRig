"""TouchPose's native half, bound with ctypes. No Qt, no numpy logic.

Everything that costs anything lives in rigExecImaging (C++):

  * the pick -- a BVH over the POSED triangles, refit when the rig
    publishes a new pose, so a ray is microseconds instead of the 1.6 ms
    the numpy cast took;
  * the marquee and the paint brush -- one parallel pass over the faces;
  * the highlight -- a Storm shader reading two primvars that a Hydra
    scene index adds to the mesh. Nothing is authored on the stage.

This module only moves arrays across the boundary. The C declarations it
binds are `libs/rigExecImaging/touchPose.h`; keep the two in step.
"""
import ctypes
import os

import numpy

_c_double_p = ctypes.POINTER(ctypes.c_double)
_c_float_p = ctypes.POINTER(ctypes.c_float)
_c_int32_p = ctypes.POINTER(ctypes.c_int32)


def _LibraryFileName():
    import sys
    if sys.platform.startswith("win"):
        return "rigExecImaging.dll"
    if sys.platform == "darwin":
        return "librigExecImaging.dylib"
    return "librigExecImaging.so"


def LibraryPath():
    """Where rigExecImaging is.

    rigExecUsdview's resolver first, so the viewport plugin and TouchPose
    load the SAME library -- the highlight state is process-global, and two
    copies of the library would be two unconnected highlights. Then the
    RIGEXEC_IMAGING_DLL override, then the build and install layouts relative
    to this file.
    """
    try:
        from rigExecUsdview import ImagingLibraryPath
        path = ImagingLibraryPath()
        if path and os.path.isfile(path):
            return path
    except Exception:
        pass
    explicit = os.environ.get("RIGEXEC_IMAGING_DLL")
    if explicit:
        return explicit
    here = os.path.dirname(os.path.abspath(__file__))
    name = _LibraryFileName()
    for candidate in (os.path.join(here, "..", "..", name),
                      os.path.join(here, "..", "..", "build", name)):
        candidate = os.path.normpath(candidate)
        if os.path.isfile(candidate):
            return candidate
    return os.path.normpath(os.path.join(here, "..", "..", "build", name))


_lib = None


def Library():
    """The bound library, loaded once. Raises OSError when it is missing."""
    global _lib
    if _lib is not None:
        return _lib
    lib = ctypes.CDLL(LibraryPath())
    ll, i, d, f = ctypes.c_longlong, ctypes.c_int, ctypes.c_double, ctypes.c_float
    sig = {
        "RigExecTouchPose_Open": (ll, [ll, ctypes.c_char_p]),
        "RigExecTouchPose_OpenTopology": (
            ll, [_c_int32_p, i, _c_int32_p, i, i, ctypes.c_char_p]),
        "RigExecTouchPose_Close": (None, [ll]),
        "RigExecTouchPose_GetFaceCount": (i, [ll]),
        "RigExecTouchPose_GetPointCount": (i, [ll]),
        "RigExecTouchPose_SyncPose": (i, [ll, d, i, i]),
        "RigExecTouchPose_SetPoints": (i, [ll, _c_float_p, i]),
        "RigExecTouchPose_SetTransform": (i, [ll, _c_double_p]),
        "RigExecTouchPose_GetPoints": (i, [ll, _c_float_p, i]),
        "RigExecTouchPose_GetBounds": (i, [ll, _c_double_p]),
        "RigExecTouchPose_SetFaceRegions": (i, [ll, _c_int32_p, i, i]),
        "RigExecTouchPose_Cast": (i, [ll, _c_double_p, _c_double_p, _c_double_p]),
        "RigExecTouchPose_CastBruteForce": (
            i, [ll, _c_double_p, _c_double_p, _c_double_p]),
        "RigExecTouchPose_RegionsInRect": (
            i, [ll, _c_double_p, d, d, _c_double_p, d, d, d, d, _c_int32_p, i]),
        "RigExecTouchPose_Brush": (
            i, [ll, _c_double_p, _c_double_p, d, _c_int32_p, i]),
        "RigExecTouchPose_FaceCentroid": (i, [ll, i, _c_double_p]),
        "RigExecTouchPose_SetRegionColors": (i, [ll, _c_float_p, _c_float_p, i]),
        "RigExecTouchPose_SetHighlightEnabled": (i, [ll, i]),
        "RigExecTouchPose_SetHighlightState": (
            i, [ll, i, i, _c_int32_p, i, i, f, _c_float_p, _c_float_p]),
        "RigExecTouchPose_GetHighlightTable": (i, [ll, _c_float_p, i]),
        "RigExecTouchPose_GetSceneIndexCount": (i, []),
        "RigExecTouchPose_GetTableUpdateCount": (ll, []),
        "RigExecTouchPose_GetShaderSource": (i, [ctypes.c_char_p, i]),
    }
    for name, (restype, argtypes) in sig.items():
        fn = getattr(lib, name)
        fn.restype = restype
        fn.argtypes = argtypes
    _lib = lib
    return lib


def Available():
    try:
        Library()
        return True
    except OSError:
        return False


def _Doubles(values, count):
    array = (ctypes.c_double * count)(*[float(v) for v in values])
    return array


def _Floats3(color):
    return (ctypes.c_float * 3)(*[float(c) for c in color])


def _Int32Buffer(array):
    array = numpy.ascontiguousarray(array, dtype=numpy.int32)
    return array, array.ctypes.data_as(_c_int32_p)


def _MatrixRows(matrix):
    """16 doubles, row-major, from a Gf.Matrix4d or a nested sequence."""
    return [float(matrix[r][c]) for r in range(4) for c in range(4)]


def ShaderSource():
    lib = Library()
    size = lib.RigExecTouchPose_GetShaderSource(None, 0)
    buffer = ctypes.create_string_buffer(size + 1)
    lib.RigExecTouchPose_GetShaderSource(buffer, size + 1)
    return buffer.value.decode("utf-8")


def SceneIndexCount():
    return int(Library().RigExecTouchPose_GetSceneIndexCount())


def TableUpdateCount():
    return int(Library().RigExecTouchPose_GetTableUpdateCount())


class NativeMesh(object):
    """One touched mesh on the C++ side. See touchPose.h for every call."""

    def __init__(self, handle):
        if not handle:
            raise ValueError("rigExecImaging could not open the mesh")
        self._lib = Library()
        self._handle = handle
        self.face_count = int(self._lib.RigExecTouchPose_GetFaceCount(handle))
        self.point_count = int(self._lib.RigExecTouchPose_GetPointCount(handle))

    # -- construction --------------------------------------------------------

    @classmethod
    def FromStage(cls, stage, mesh_path):
        """Open straight off a live stage, found through UsdUtils.StageCache.

        The C++ side keeps only a WEAK pointer to the stage. The cache entry
        is left in place rather than erased after the call: a Python-created
        stage is held by Python through a weak pointer, so erasing the entry
        can release the last strong reference and destroy the stage under
        the caller (measured: the next `GetPrimAtPath` raises). In usdview
        rigExecUsdview has already cached the stage and owns the release.
        """
        from pxr import UsdUtils
        lib = Library()
        cache = UsdUtils.StageCache.Get()
        stage_id = (cache.GetId(stage) if cache.Contains(stage)
                    else cache.Insert(stage))
        handle = lib.RigExecTouchPose_Open(
            stage_id.ToLongInt(), str(mesh_path).encode("utf-8"))
        return cls(handle)

    @classmethod
    def FromTopology(cls, counts, indices, point_count, mesh_path=None):
        lib = Library()
        counts, pc = _Int32Buffer(counts)
        indices, pi = _Int32Buffer(indices)
        handle = lib.RigExecTouchPose_OpenTopology(
            pc, len(counts), pi, len(indices), int(point_count),
            str(mesh_path).encode("utf-8") if mesh_path else None)
        return cls(handle)

    def Close(self):
        if self._handle:
            self._lib.RigExecTouchPose_Close(self._handle)
            self._handle = 0

    def __del__(self):
        try:
            self.Close()
        except Exception:
            pass

    # -- pose ----------------------------------------------------------------

    def SyncPose(self, time=None, force=False):
        """Pull what the viewport draws. True when anything moved."""
        is_default = time is None or getattr(time, "IsDefault", lambda: False)()
        frame = 0.0 if is_default else float(
            time.GetValue() if hasattr(time, "GetValue") else time)
        return self._lib.RigExecTouchPose_SyncPose(
            self._handle, frame, int(is_default), int(bool(force))) > 0

    def SetPoints(self, points):
        points = numpy.ascontiguousarray(points, dtype=numpy.float32)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("points must be Nx3, got %r" % (points.shape,))
        if self._lib.RigExecTouchPose_SetPoints(
                self._handle, points.ctypes.data_as(_c_float_p),
                len(points)) != 0:
            raise ValueError("%d points for a mesh of %d"
                             % (len(points), self.point_count))

    def SetTransform(self, matrix):
        self._lib.RigExecTouchPose_SetTransform(
            self._handle, _Doubles(_MatrixRows(matrix), 16))

    def Points(self):
        out = numpy.empty((self.point_count, 3), dtype=numpy.float32)
        self._lib.RigExecTouchPose_GetPoints(
            self._handle, out.ctypes.data_as(_c_float_p), self.point_count)
        return out

    def Bounds(self):
        out = (ctypes.c_double * 6)()
        if not self._lib.RigExecTouchPose_GetBounds(self._handle, out):
            return None
        return tuple(out[0:3]), tuple(out[3:6])

    # -- regions -------------------------------------------------------------

    def SetFaceRegions(self, region_of, region_count):
        region_of, pointer = _Int32Buffer(region_of)
        self._lib.RigExecTouchPose_SetFaceRegions(
            self._handle, pointer, len(region_of), int(region_count))

    # -- picking -------------------------------------------------------------

    def Cast(self, origin, direction, brute_force=False):
        """(face, t) along a world ray, or (-1, -1.0)."""
        t = ctypes.c_double(-1.0)
        fn = (self._lib.RigExecTouchPose_CastBruteForce if brute_force
              else self._lib.RigExecTouchPose_Cast)
        face = fn(self._handle, _Doubles(origin, 3), _Doubles(direction, 3),
                  ctypes.byref(t))
        return (int(face), float(t.value)) if face >= 0 else (-1, -1.0)

    def RegionsInRect(self, view_projection, width, height, eye,
                      x0, y0, x1, y1):
        capacity = 256
        while True:
            out = numpy.empty(capacity, dtype=numpy.int32)
            count = self._lib.RigExecTouchPose_RegionsInRect(
                self._handle, _Doubles(_MatrixRows(view_projection), 16),
                float(width), float(height), _Doubles(eye, 3),
                float(x0), float(y0), float(x1), float(y1),
                out.ctypes.data_as(_c_int32_p), capacity)
            if count <= capacity:
                return out[:max(count, 0)].tolist()
            capacity = count

    def Brush(self, center, direction, radius):
        capacity = 4096
        direction_arg = (_Doubles(direction, 3) if direction is not None
                         else None)
        while True:
            out = numpy.empty(capacity, dtype=numpy.int32)
            count = self._lib.RigExecTouchPose_Brush(
                self._handle, _Doubles(center, 3), direction_arg,
                float(radius), out.ctypes.data_as(_c_int32_p), capacity)
            if count <= capacity:
                return out[:max(count, 0)].copy()
            capacity = count

    def FaceCentroid(self, face):
        out = (ctypes.c_double * 3)()
        if not self._lib.RigExecTouchPose_FaceCentroid(self._handle, int(face),
                                                       out):
            return None
        return numpy.array(out[:], dtype=numpy.float64)

    # -- highlight -----------------------------------------------------------

    def SetRegionColors(self, hover_rgb, edit_rgb):
        hover = numpy.ascontiguousarray(hover_rgb, dtype=numpy.float32)
        edit = numpy.ascontiguousarray(edit_rgb, dtype=numpy.float32)
        self._lib.RigExecTouchPose_SetRegionColors(
            self._handle, hover.ctypes.data_as(_c_float_p),
            edit.ctypes.data_as(_c_float_p), len(hover))

    def SetHighlightEnabled(self, enabled):
        return self._lib.RigExecTouchPose_SetHighlightEnabled(
            self._handle, int(bool(enabled))) == 0

    def SetHighlightState(self, hover, lead, selected, editing, opacity,
                          lead_rgb, selected_rgb):
        selected, pointer = _Int32Buffer(list(selected))
        return self._lib.RigExecTouchPose_SetHighlightState(
            self._handle, -1 if hover is None else int(hover),
            -1 if lead is None else int(lead), pointer, len(selected),
            int(bool(editing)), float(opacity), _Floats3(lead_rgb),
            _Floats3(selected_rgb)) == 1

    def HighlightTable(self):
        rows = self._lib.RigExecTouchPose_GetHighlightTable(self._handle, None, 0)
        out = numpy.zeros((max(rows, 0), 4), dtype=numpy.float32)
        if rows > 0:
            self._lib.RigExecTouchPose_GetHighlightTable(
                self._handle, out.ctypes.data_as(_c_float_p), rows)
        return out
