@echo off
rem The Layer Opinions panel, end to end (tests/testUsdviewLayerOpinions.py).
rem Asserts the RigExec menu item is registered, that the panel follows
rem usdview's prim selection, and that an edit, a refused parse and a delete
rem each land on the layer the clicked row names -- with undo restoring it.
rem Prints RIGEXEC_LAYER_OPINIONS_OK.
rem Usage: run_testusdview_opinions.bat [stage.usda] (default: examples\ArmRig.usda)
rem Set RIGEXEC_OPINIONS_SHOT=path.png to keep a window grab.
call "%~dp0_vcvars.bat"
call "%~dp0_env.bat"
call "%~dp0_require_python.bat"
if errorlevel 1 exit /b 1
set "STAGE=%~1"
if "%STAGE%"=="" set "STAGE=%RIG%\examples\ArmRig.usda"
"%PY%" "%USD%\bin\testusdview" --testScript "%RIG%\tests\testUsdviewLayerOpinions.py" "%STAGE%"
exit /b %errorlevel%
