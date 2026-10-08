"""Own commit notice classification preserves first-session preview caches."""
import types
import unittest
from test_gizmo_deferred_release import APP, Controller
from pxr import Sdf, Tf, Usd


class OwnCommit(unittest.TestCase):
    def setUp(self):
        self.stage = Usd.Stage.CreateInMemory()
        self.stage.DefinePrim('/Rig', 'RigExecRoot')
        self.prim = self.stage.DefinePrim('/Rig/Controls/C', 'RigExecControl')
        self.attr = self.prim.GetAttribute('avars:tx')
        self.stage.SetEditTarget(self.stage.GetSessionLayer())
        self.controller = Controller(self.stage)
        self.events = []
        self.notices = []
        self.target = types.SimpleNamespace(stage=self.stage, time=Usd.TimeCode(1),
            prim=self.prim, RigRootPath=lambda: Sdf.Path('/Rig'),
            Refresh=lambda: self.events.append('refresh'))
        self.controller._target = self.target
        self.controller._targetPaths = (self.prim.GetPath(),)
        self.controller._SelectionSignature = lambda: self.controller._targetPaths
        self.controller._TargetPrimPaths = lambda target: [target.prim.GetPath()]
        self.controller._solverPosed = types.SimpleNamespace(
            InvalidateResynced=lambda paths: self.events.append('resynced'),
            InvalidateChanged=lambda paths: self.events.append('changed'),
            Clear=lambda: None)
        self.controller._hoverRigStage = self.stage
        self.controller._snapGeom = {self.prim.GetPath(): object()}
        self.controller._DropSnapGeom = lambda *args: self.events.append('snap')
        self.controller._committing = (self.target, {self.attr.GetPath(): 2.})
        def notice(n, sender):
            self.notices.append((list(n.GetResyncedPaths()),
                [(p, list(n.GetChangedFields(p))) for p in n.GetChangedInfoOnlyPaths()]))
            self.controller._onObjectsChanged(n, sender)
        self.callback = notice
        self.key = Tf.Notice.Register(Usd.Notice.ObjectsChanged, self.callback, self.stage)
    def tearDown(self):
        self.key.Revoke()
        self.controller._committing = None
        self.controller.Detach()
        self.controller.window.close()
        APP.processEvents()
    def author(self, value):
        with Sdf.ChangeBlock():
            self.attr.Set(value)
    def test_first_session_write_keeps_preview_caches(self):
        self.author(2.)
        self.assertEqual(self.events, ['snap'])
        self.assertIsNone(self.controller._hoverRigStage)
        self.assertEqual(self.notices[0][0], [self.attr.GetPath()])
        self.assertTrue(any(p.IsPrimPath() and not fields for p, fields in self.notices[0][1]))
        self.author(2.)
        self.assertTrue(all(event == 'snap' for event in self.events))
    def test_metadata_is_not_a_value_commit(self):
        self.author(2.)
        self.attr.SetCustomDataByKey('note', 'real edit')
        self.assertIn('changed', self.events)
        self.assertIn('refresh', self.events)
    def test_connections_are_not_a_value_commit(self):
        self.author(2.)
        self.attr.SetConnections([Sdf.Path('/Rig/Controls/C.avars:ty')])
        self.assertIn('changed', self.events)
        self.assertIn('refresh', self.events)
    def test_resynced_metadata_is_not_a_value_commit(self):
        with Sdf.ChangeBlock():
            self.attr.Set(2.)
            self.attr.SetCustomDataByKey('note', 'real edit')
        self.assertIn('resynced', self.events)
        self.assertIn('refresh', self.events)
    def test_resynced_connections_are_not_a_value_commit(self):
        with Sdf.ChangeBlock():
            self.attr.Set(2.)
            self.attr.SetConnections([Sdf.Path('/Rig/Controls/C.avars:ty')])
        self.assertIn('resynced', self.events)
        self.assertIn('refresh', self.events)
    def test_prim_resync_invalidates(self):
        self.author(2.)
        self.stage.DefinePrim('/Rig/New', 'Xform')
        self.assertIn('resynced', self.events)
        self.assertIsNone(self.controller._hoverRigStage)
    def test_wrong_value_invalidates(self):
        self.author(3.)
        self.assertIn('resynced', self.events)
        self.assertIn('refresh', self.events)
    def test_unrelated_property_resync_invalidates(self):
        self.author(2.)
        self.prim.CreateAttribute('custom:test', Sdf.ValueTypeNames.Double).Set(5.)
        self.assertIn('resynced', self.events)
        self.assertIn('refresh', self.events)
    def test_ancestor_metadata_invalidates(self):
        self.author(2.)
        self.stage.GetPrimAtPath('/Rig').SetCustomDataByKey('note', 'real edit')
        self.assertIn('changed', self.events)
        self.assertIsNone(self.controller._hoverRigStage)


if __name__ == '__main__': unittest.main()
