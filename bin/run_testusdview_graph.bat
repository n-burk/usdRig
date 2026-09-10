@echo off
rem Headless end-to-end test of the usdview graph editor
rem (tests\testUsdviewGraphEditor.py) on examples\ArmShotAnim.usda. The Windows
rem twin of run_testusdview_graph.sh.
rem
rem Drives synthetic mouse and key events at the pixels the graph canvas itself
rem reports for its keys and tangent handles, and asserts what landed on the
rem stage: the curve set for a prim and for a property selection, a key drag
rem written into the session layer, the Ctrl+Z round trip, insert and delete,
rem the Maya tangent types, tangent handle drags, break/unify, the infinity
rem mapping, the ruler scrub and marquee selection. Prints RIGEXEC_GRAPH_OK.
rem
rem Usage: run_testusdview_graph.bat [rendererDisplayName]   (e.g. Embree)
rem Set RIGEXEC_GRAPH_SHOT=path.png to keep a window grab.
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
"%PY%" "%USD%\bin\testusdview" --testScript "%RIG%\tests\testUsdviewGraphEditor.py" %RENDERER_ARG% "%RIG%\examples\ArmShotAnim.usda"
exit /b %errorlevel%
