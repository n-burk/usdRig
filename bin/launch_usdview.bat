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
call "%~dp0_vcvars.bat"
call "%~dp0_env.bat"
call "%~dp0_require_python.bat"
if errorlevel 1 exit /b 1

rem Register the Muse assistant's plugin container. _env.bat puts museAssistant
rem on PYTHONPATH for every helper, but registering it is deliberately left to
rem the interactive launchers: the headless testusdview runners share that env
rem and must not load an extra panel into the app they are asserting against.
rem
rem Hosted providers need MUSE_API_KEY (or ANTHROPIC_API_KEY). Local Apple FM
rem and Ollama do not.
set "PXR_PLUGINPATH_NAME=%PXR_PLUGINPATH_NAME%;%RIG%\plugin\museAssistant"

rem TouchPose, for the same reason and with the same caveat. This launcher
rem is the one an animator opens, and the toolset is not something they
rem should have to pick a launcher for: without this the RigExec menu
rem simply has no TouchPose item and nothing says why. plugin\touchPose
rem is NOT on _env.bat's PYTHONPATH the way rigExecUsdview is, so both
rem the plugin path and the import path are added here.
set "PXR_PLUGINPATH_NAME=%PXR_PLUGINPATH_NAME%;%RIG%\plugin\touchPose"
set "PYTHONPATH=%RIG%\plugin\touchPose;%PYTHONPATH%"
set "TOUCHPOSE_PLUGIN_DIR=%RIG%\plugin\touchPose"

rem The Shape Editor rides along, for the same reason and with the same
rem caveat: a self-contained plugin directory whose container asks
rem findOrCreateMenu for the RigExec menu, so its item lands under the
rem same menu whichever container loads first. plugin\shapeEditor is not
rem on _env.bat's PYTHONPATH either, so the module search path is added
rem beside the plugin path.
set "PXR_PLUGINPATH_NAME=%PXR_PLUGINPATH_NAME%;%RIG%\plugin\shapeEditor"
set "PYTHONPATH=%RIG%\plugin\shapeEditor;%PYTHONPATH%"

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

rem Register the usdNoodles node-graph editor the build staged under
rem build\python\UsdNoodles -- after building, so a first build has produced
rem it. An OpenUSD built from PR #4156 with noodles also installs the older
rem editor as pxr.UsdNoodles; the in-repo copy takes its place when both are
rem present (UsdNoodles\__init__.py, _supersedeInstalledCopy), so registering
rem it is always right. rigexec_register_usdnoodles in _env.sh is the POSIX
rem twin.
if exist "%RIG%\build\python\UsdNoodles\plugInfo.json" (
    set "PXR_PLUGINPATH_NAME=%PXR_PLUGINPATH_NAME%;%RIG%\build\python\UsdNoodles"
    set "USD_NOODLES="
    if exist "%USD%\Lib\site-packages\pxr\UsdNoodles" set "USD_NOODLES=%USD%\Lib\site-packages\pxr\UsdNoodles"
    if exist "%USD%\lib\python\pxr\UsdNoodles" set "USD_NOODLES=%USD%\lib\python\pxr\UsdNoodles"
    if defined USD_NOODLES (
        >&2 echo usdNoodles: plugin\usdNoodles supersedes !USD_NOODLES!
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
"%PY%" "%USDVIEW%" !RENDERER!!FLAGS! "%STAGE%"
exit /b %errorlevel%
