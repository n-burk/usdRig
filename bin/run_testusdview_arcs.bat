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
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
call "%~dp0_env.bat"
set "STAGE=%~1"
if "%STAGE%"=="" set "STAGE=%RIG%\examples\ArmRig.usda"
python "%USD%\bin\testusdview" --testScript "%RIG%\tests\testUsdviewCompositionArcs.py" "%STAGE%"
exit /b %errorlevel%
