@echo off
rem bin\launch_usdview.bat -- interactive usdview on a RigExec stage with the
rem live Hydra integration. The Windows twin of usdview.sh.
rem
rem Usage: launch_usdview.bat [stage.usda] [rendererDisplayName | usdview flags...]
rem   stage defaults to an empty stage carrying a single World Xform, so that
rem     opening the app to build something is the no-argument case; pass
rem     examples\ArmShotAnim.usda for the rig that deforms on the timeline.
rem   a bare second argument is the renderer (e.g. Embree); anything starting
rem   with - is passed to usdview untouched.
setlocal EnableDelayedExpansion
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
call "%~dp0_env.bat"

rem Register the Muse assistant's plugin container. _env.bat puts museAssistant
rem on PYTHONPATH for every helper, but registering it is deliberately left to
rem the interactive launchers: the headless testusdview runners share that env
rem and must not load an extra panel into the app they are asserting against.
rem
rem Hosted providers need MUSE_API_KEY (or ANTHROPIC_API_KEY). Local Apple FM
rem and Ollama do not.
set "PXR_PLUGINPATH_NAME=%PXR_PLUGINPATH_NAME%;%RIG%\plugin\museAssistant"

rem Fail early and legibly rather than deep inside python.
set "USDVIEW=%USD%\bin\usdview"
if not exist "%USDVIEW%" (
    >&2 echo ERROR: %USDVIEW% not found ^(USD=%USD%^)
    >&2 echo        set USD=\path\to\usd-install and retry.
    exit /b 1
)

rem Build if the tree is already configured; a no-op when up to date. Silent
rem unless it fails, because these scripts are run to see their own output.
if exist "%RIG%\build\CMakeCache.txt" (
    cmake --build "%RIG%\build" >nul 2>&1
    if errorlevel 1 (
        >&2 echo ERROR: build failed; run bin\build_rigexec.bat to see why.
        exit /b 1
    )
)

rem A leading non-flag argument is the stage; otherwise open a blank one.
set "ARG=%~1"
if defined ARG if not "!ARG:~0,1!"=="-" goto :stage_arg

rem Rewritten every run rather than kept, so an edited or truncated leftover
rem cannot turn into a confusing "blank" stage on the next launch.
set "STAGE=%TEMP%\rigexec-blank.usda"
> "%STAGE%" echo #usda 1.0
>>"%STAGE%" echo def Xform "World" {
>>"%STAGE%" echo }
goto :renderer

:stage_arg
set "STAGE=%~1"
shift
if not exist "%STAGE%" (
    >&2 echo ERROR: stage not found: %STAGE%
    exit /b 1
)

rem A bare (non-flag) argument here is the renderer display name; everything
rem left over is handed to usdview untouched.
:renderer
set "RENDERER="
set "FLAGS="
set "ARG=%~1"
if not defined ARG goto :run
if "!ARG:~0,1!"=="-" goto :collect
set "RENDERER=--renderer %~1"
shift

:collect
if "%~1"=="" goto :run
set "FLAGS=!FLAGS! %1"
shift
goto :collect

:run
python "%USDVIEW%" !RENDERER!!FLAGS! "%STAGE%"
exit /b %errorlevel%
