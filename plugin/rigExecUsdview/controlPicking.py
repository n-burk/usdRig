"""Screen-space tolerance for viewport control curves, using visible GPU hits."""

from pxr import Gf, UsdImagingGL


class ControlPicking:
    def __init__(self, view):
        self.view = view
        self.original = view.pick
        self.pick = self._Pick
        view.pick = self.pick

    def Detach(self):
        if self.view.pick == self.pick:
            self.view.pick = self.original

    def _IsControl(self, path):
        stage = self.view._dataModel.stage
        prim = stage.GetPrimAtPath(path) if stage else None
        return bool(prim and prim.GetTypeName() == 'RigExecControl'
                    and prim.GetAttribute('guide:drawMode').Get() == 'wire')

    def _Pick(self, frustum):
        hits = self.original(frustum)
        if hits and self._IsControl(hits[0].hitPrimPath):
            return hits
        renderer = self.view._getRenderer()
        stage = self.view._dataModel.stage
        if not renderer or not stage:
            return hits

        # StageView's frustum spans one physical pixel. Expand it to a
        # nine-logical-pixel window; keep ordinary depth testing so a
        # hidden curve cannot win over the surface in front of it.
        wide = Gf.Frustum(frustum)
        window = frustum.GetWindow()
        center = window.GetMidpoint()
        half = window.GetSize() * (4.5 * self.view.devicePixelRatioF())
        wide.SetWindow(Gf.Range2d(center - half, center + half))
        params = UsdImagingGL.Engine.PickParams()
        params.resolveMode = 'resolveAll'
        candidates = renderer.TestIntersection(
            params, wide.ComputeViewMatrix(), wide.ComputeProjectionMatrix(),
            stage.GetPseudoRoot(), self.view._renderParams)
        projection = wide.ComputeViewMatrix() * wide.ComputeProjectionMatrix()
        controls = {}
        nearest, distance = None, 1.0
        for hit in candidates:
            path = hit.hitPrimPath
            if path not in controls:
                controls[path] = self._IsControl(path)
            if not controls[path]:
                continue
            point = projection.Transform(Gf.Vec3d(hit.hitPoint))
            d = point[0] ** 2 + point[1] ** 2
            if d < distance:
                nearest, distance = hit, d
        return [nearest] if nearest is not None else hits
