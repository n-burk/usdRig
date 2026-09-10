@echo off
rem The Layer Opinions panel, end to end (tests/testUsdviewLayerOpinions.py).
rem Asserts the RigExec menu item is registered, that the panel follows
rem usdview's prim selection, and that an edit, a refused parse and a delete
rem each land on the layer the clicked row names -- with undo restoring it.
rem Prints RIGEXEC_LAYER_OPINIONS_OK.
rem Usage: run_testusdview_opinions.bat [stage.usda] (default: examples\ArmRig.usda)
rem Set RIGEXEC_OPINIONS_SHOT=path.png to keep a window grab.
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
call "%~dp0_env.bat"
set "STAGE=%~1"
if "%STAGE%"=="" set "STAGE=%RIG%\examples\ArmRig.usda"
python "%USD%\bin\testusdview" --testScript "%RIG%\tests\testUsdviewLayerOpinions.py" "%STAGE%"
exit /b %errorlevel%
