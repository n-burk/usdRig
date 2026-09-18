@echo off
rem Headless end-to-end test that the IK/FK switch fades the inactive
rem control set (tests\testUsdviewIkFkOpacity.py) on biped_rig_v3.usda.
rem The Windows twin of run_testusdview_ikfk_opacity.sh.
rem
rem Why it exists: `guide:displayOpacity.connect = <dial>` compiled and
rem changed nothing, because UsdAttribute::Get never follows a connection.
rem The imaging bridge now does, and this asserts the DRAWN opacity from
rem the terminal Hydra scene index -- never the authored attribute -- as
rem tools\biped\params.py wires each limb's controls (into the session
rem layer; the file is never written) and the limb's avars:ikfk is driven
rem on the param node, a different prim from every guide it fades. The IK
rem guides come up with the dial and the FK guides go down against it,
rem floored at guide:displayOpacityMin. Prints RIGEXEC_IKFK_OPACITY_OK.
rem
rem Usage: run_testusdview_ikfk_opacity.bat [rendererDisplayName]  (e.g. Embree)
rem Set RIGEXEC_IKFK_OPACITY_SHOT=path.png to keep a window grab.
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
"%PY%" "%USD%\bin\testusdview" --testScript "%RIG%\tests\testUsdviewIkFkOpacity.py" %RENDERER_ARG% "%RIG%\biped_rig_v3.usda"
exit /b %errorlevel%
