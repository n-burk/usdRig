@echo off
rem Headless end-to-end test of the Avar Editor panel
rem (tests\testUsdviewAvarEditor.py) on biped_full2.usda. The Windows twin
rem of run_testusdview_avars.sh.
rem
rem Asserts the RigExec menu item is registered, that the panel follows
rem usdview's prim selection, and that driving a row's spin box or slider
rem through the panel's own Qt signals moves the EVALUATED rig by the
rem amount the maths says (the elbow's chord under a 30 degree FK rotate,
rem the IK/FK blend under the custom ikfk dial), republishes to Hydra, keeps
rem a file-animated channel's other keys, and lands as one entry per edit
rem on the shared undo stack. Prints RIGEXEC_AVAR_EDITOR_OK.
rem
rem Usage: run_testusdview_avars.bat [rendererDisplayName]   (e.g. Embree)
rem Set RIGEXEC_AVARS_SHOT=path.png to keep a window grab.
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
"%PY%" "%USD%\bin\testusdview" --testScript "%RIG%\tests\testUsdviewAvarEditor.py" %RENDERER_ARG% "%RIG%\biped_full2.usda"
exit /b %errorlevel%
