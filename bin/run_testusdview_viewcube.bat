@echo off
rem Headless end-to-end test of the Maya-style view cube
rem (tests\testUsdviewViewCube.py) on examples\ArmShotAnim.usda. The Windows
rem twin of run_testusdview_viewcube.sh.
rem
rem Drives synthetic mouse events through the cube's own projected region
rem points and asserts what landed on the stage: the free camera's view
rem direction and up vector after face/edge/corner clicks, hover, the animated
rem orbit, drag-to-tumble, Alt-press propagation, the camera prim round trip,
rem Home and the menu toggle. Prints RIGEXEC_VIEWCUBE_OK.
rem
rem Usage: run_testusdview_viewcube.bat [rendererDisplayName]   (e.g. Embree)
rem Set RIGEXEC_VIEWCUBE_SHOT=path.png to keep a window grab.
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
"%PY%" "%USD%\bin\testusdview" --testScript "%RIG%\tests\testUsdviewViewCube.py" %RENDERER_ARG% "%RIG%\examples\ArmShotAnim.usda"
exit /b %errorlevel%
