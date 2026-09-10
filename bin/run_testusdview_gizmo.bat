@echo off
rem Headless end-to-end test of the viewport gizmo toolbar
rem (tests\testUsdviewGizmo.py) on examples\ArmShotAnim.usda. The Windows twin
rem of run_testusdview_gizmo.sh.
rem
rem Drives synthetic mouse and key events through the gizmo's own projected
rem handle positions and asserts what landed on the stage: the avars, the
rem undo/redo round trip, Default vs Animation, Pivot vs Pose, a plain xform's
rem op stack, and the Maya parity behaviours. Prints RIGEXEC_GIZMO_OK.
rem
rem Usage: run_testusdview_gizmo.bat [rendererDisplayName]   (e.g. Embree)
rem Set RIGEXEC_GIZMO_SHOT=path.png to keep a window grab.
call "%~dp0_vcvars.bat"
call "%~dp0_env.bat"
call "%~dp0_require_python.bat"
if errorlevel 1 exit /b 1
if exist "%RIG%\build\CMakeCache.txt" (
    cmake --build "%RIG%\build" >nul
    if errorlevel 1 exit /b 1
)
set RENDERER_ARG=
if not "%~1"=="" set RENDERER_ARG=--renderer %~1
"%PY%" "%USD%\bin\testusdview" --testScript "%RIG%\tests\testUsdviewGizmo.py" %RENDERER_ARG% "%RIG%\examples\ArmShotAnim.usda"
exit /b %errorlevel%
