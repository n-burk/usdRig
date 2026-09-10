@echo off
rem Curvenet draw verification against the puppetA character.
rem Usage: run_testusdview_draw.bat [stage.usda]
rem   stage defaults to the puppetA curvenet beside the repository.
call "%~dp0_vcvars.bat"
call "%~dp0_env.bat"
call "%~dp0_require_python.bat"
if errorlevel 1 exit /b 1
set "STAGE=%~1"
if "%STAGE%"=="" set "STAGE=%RIG%\..\chars\puppetA\puppetA_curvenet.usda"
"%PY%" "%USD%\bin\testusdview" --testScript "%RIG%\tests\testUsdviewCurvenetDraw.py" "%STAGE%"
exit /b %errorlevel%
