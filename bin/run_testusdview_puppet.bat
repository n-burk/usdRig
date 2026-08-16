@echo off
rem Curvenet puppet verification against the puppetA character.
rem Usage: run_testusdview_puppet.bat [stage.usda]
rem   stage defaults to the puppetA curvenet beside the repository.
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
call "%~dp0_env.bat"
set "STAGE=%~1"
if "%STAGE%"=="" set "STAGE=%RIG%\..\chars\puppetA\puppetA_curvenet.usda"
python "%USD%\bin\testusdview" --testScript "%RIG%\tests\testUsdviewCurvenetPuppet.py" "%STAGE%"
exit /b %errorlevel%
