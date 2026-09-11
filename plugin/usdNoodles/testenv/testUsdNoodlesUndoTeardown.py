#!/pxrpythonsubst
#
# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the terms set forth in the LICENSE.txt file available
# at the root of this repository.
#


"""
Process teardown with Python commands still on the undo stack.

NoodlesUndoManager::instance() is a function-local static, so its stacks are
destroyed during __cxa_finalize -- after Py_Finalize has torn the interpreter
down. A command holding Python callables that drops its references there calls
PyThreadState_Get with no thread state and kills the process, turning a clean
quit into a crash.

This has to run in a SUBPROCESS. The fault is in interpreter finalization, so
the test process would have to die to observe it, and by then it cannot report
anything. The exit status of a child is the only way to see it.
"""

import subprocess
import sys
import textwrap
import unittest


def _run(body):
    """Run body in a child interpreter, return its exit status."""
    return subprocess.run(
        [sys.executable, "-c", textwrap.dedent(body)],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
    )


class UndoTeardownTest(unittest.TestCase):
    """Exiting with commands still on the stack must not crash."""

    def test_importing_alone_exits_cleanly(self) -> None:
        """The control: no command pushed, so nothing holds a Python object."""
        result = _run("import UsdNoodles._usdNoodles")
        self.assertEqual(result.returncode, 0, result.stderr.decode())

    def test_pushed_command_survives_teardown(self) -> None:
        """The regression: one command outliving the interpreter.

        Before the fix this aborted with "PyThreadState_Get: the function must
        be called with the GIL held" and a non-zero status.
        """
        result = _run("""
            from UsdNoodles._usdNoodles import pushLambdaCommand
            pushLambdaCommand("teardown", lambda: None, lambda: None)
        """)
        self.assertEqual(
            result.returncode, 0,
            "exiting with a command on the undo stack crashed (%d):\n%s"
            % (result.returncode, result.stderr.decode()))

    def test_a_full_stack_survives_teardown(self) -> None:
        """Undone commands sit on the redo stack and must be safe too."""
        result = _run("""
            from UsdNoodles._usdNoodles import (
                pushLambdaCommand, NoodlesUndoManager)
            for i in range(8):
                pushLambdaCommand("edit %d" % i, lambda: None, lambda: None)
            NoodlesUndoManager.instance().undo()
            NoodlesUndoManager.instance().undo()
        """)
        self.assertEqual(
            result.returncode, 0,
            "exiting with undo and redo stacks populated crashed (%d):\n%s"
            % (result.returncode, result.stderr.decode()))

    def test_clearing_still_releases_references(self) -> None:
        """The teardown guard must not become a leak during a live session.

        Abandoning references is only correct once the interpreter is gone;
        while it is alive, clear() has to drop them for real or every undone
        edit leaks for the rest of the session.
        """
        result = _run("""
            import gc, sys, weakref
            from UsdNoodles._usdNoodles import (
                pushLambdaCommand, NoodlesUndoManager)

            class Marker(object):
                pass

            marker = Marker()
            ref = weakref.ref(marker)

            def undo_func():
                return marker

            pushLambdaCommand("edit", lambda: None, undo_func)
            NoodlesUndoManager.instance().clear()
            del marker, undo_func
            gc.collect()
            sys.exit(0 if ref() is None else 1)
        """)
        self.assertEqual(
            result.returncode, 0,
            "clear() leaked the command's Python references:\n%s"
            % result.stderr.decode())


if __name__ == "__main__":
    unittest.main()
