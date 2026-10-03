#!/usr/bin/env python
"""
Headless test for plugin/rigExecUsdview/layerStackModel.py: the sublayer
tree the Layer Stack panel draws, and muting a branch on and off.

Builds a small layer tree in a temporary directory -- a root over a face
branch and a body branch, each with its own sublayers -- so the relative
sublayer paths resolve the way a shipped character's do.

Usage: test_layer_stack_model.py [schema resource dir]
"""
import os
import shutil
import sys
import tempfile

import rigexec_test_env

rigexec_test_env.SetupPluginTest()

from pxr import Sdf, Usd  # noqa: E402

import layerStackModel as lsm  # noqa: E402


def _Check(condition, message):
    if not condition:
        raise AssertionError(message)


def _Write(directory, name, text):
    with open(os.path.join(directory, name), "w") as handle:
        handle.write("#usda 1.0\n" + text)


def _Tree(directory):
    _Write(directory, "root.usda",
           '(\n    subLayers = [@./face.usda@, @./body.usda@]\n)\n')
    _Write(directory, "face.usda",
           '(\n    subLayers = [@./face_picker.usda@]\n)\n'
           'over "Char"\n{\n    def Xform "Face"\n    {\n    }\n}\n')
    _Write(directory, "face_picker.usda",
           'over "Char"\n{\n    def Xform "FacePicker"\n    {\n    }\n}\n')
    _Write(directory, "body.usda",
           '(\n    subLayers = [@./body_rig.usda@]\n)\n')
    _Write(directory, "body_rig.usda", 'def Xform "Char"\n{\n}\n')
    return Usd.Stage.Open(os.path.join(directory, "root.usda"))


def main():
    directory = tempfile.mkdtemp(prefix="layerStackModel")
    try:
        stage = _Tree(directory)
        root = lsm.BuildTree(stage)
        names = [n.displayName for n in root.Walk()]
        _Check(names == ["root.usda", "face.usda", "face_picker.usda",
                         "body.usda", "body_rig.usda"],
               "the tree lists every layer, strongest first: %s" % names)
        _Check(not root.canMute and all(n.canMute
                                        for n in list(root.Walk())[1:]),
               "only the root cannot be muted")
        _Check(stage.GetPrimAtPath("/Char/Face").IsValid(),
               "the face is composed before muting")

        face = root.children[0]
        _Check(lsm.SetMuted(stage, face.identifier, True),
               "muting the face branch changes the stage")
        _Check(not stage.GetPrimAtPath("/Char/Face").IsValid() and
               not stage.GetPrimAtPath("/Char/FacePicker").IsValid(),
               "the muted branch and its sublayers are gone from the stage")
        _Check(stage.GetPrimAtPath("/Char").IsValid(),
               "the body is untouched")
        _Check(not lsm.SetMuted(stage, face.identifier, True),
               "muting a muted layer changes nothing")

        root = lsm.BuildTree(stage)
        face = root.children[0]
        _Check(face.muted and not face.active,
               "the tree reports the face muted")
        _Check(face.children and face.children[0].hidden and
               not face.children[0].muted,
               "the face's sublayer is still listed, hidden by its parent")

        _Check(not lsm.SetMuted(stage, stage.GetRootLayer().identifier,
                                True),
               "the root layer cannot be muted")

        _Check(lsm.UnmuteAll(stage) == 1, "unmute all unmutes the face")
        _Check(stage.GetPrimAtPath("/Char/FacePicker").IsValid(),
               "the face branch is back")
        _Check(lsm.BuildTree(None) is None, "no stage, no tree")
    finally:
        shutil.rmtree(directory, ignore_errors=True)
    print("test_layer_stack_model: ALL PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
