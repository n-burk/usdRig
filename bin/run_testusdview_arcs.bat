@echo off
rem Guided composition-arc flows in the Layer Opinions panel, end to end.
rem Asserts the right-click menu offers every arc, that the dialog builds its
rem form from the arc's fields and re-previews as they change, that an illegal
rem request disables Author, and that authoring reaches the stage and the undo
rem stack. Then the same for the arcs already there: one row per arc, its menu,
rem the moves, and reopening one prefilled and applying it in place.
rem Prints RIGEXEC_COMPOSITION_ARCS_OK.
rem Usage: run_testusdview_arcs.bat [stage.usda]  (default: examples\ArmRig.usda)
rem Set RIGEXEC_ARCS_SHOT=path.png to keep a grab of the reference flow.
call "%~dp0_vcvars.bat"
call "%~dp0_env.bat"
call "%~dp0_require_python.bat"
if errorlevel 1 exit /b 1
set "STAGE=%~1"
if "%STAGE%"=="" set "STAGE=%RIG%\examples\ArmRig.usda"
"%PY%" "%USD%\bin\testusdview" --testScript "%RIG%\tests\testUsdviewCompositionArcs.py" "%STAGE%"
exit /b %errorlevel%
