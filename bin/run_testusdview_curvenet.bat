@echo off
rem Headless verification of the curvenet authoring panel inside usdview.
rem Complements tests\testUsdviewCurvenetAuthoring.py, which covers the stage
rem edits with no widgets; this covers everything that needs the running app.
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
call "%~dp0_env.bat"
cmake --build "%RIG%\build" >nul
if errorlevel 1 exit /b 1
set RENDERER_ARG=
if not "%~1"=="" set RENDERER_ARG=--renderer %~1
python "%USD%\bin\testusdview" --testScript "%RIG%\tests\testUsdviewCurvenetPanel.py" %RENDERER_ARG% "%RIG%\examples\12_CurvenetProfile.usda"
exit /b %errorlevel%
