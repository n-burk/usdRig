@echo off
rem bin\run_usdview_touchpose.bat -- INTERACTIVE usdview with TouchPose.
rem
rem Usage: run_usdview_touchpose.bat [stage.usda] [renderer | usdview flags...]
rem   stage defaults to examples\biped\Biped_all.usda: the touch
rem   regions stacked over the LAYERED rig, which is also the stage
rem   the Control Picker resolves against, so both panels work on
rem   it. Biped_touch.usda is the same regions over the FLAT rig.
rem   Biped_touch_regions.usda is the touch data alone and shows
rem   nothing opened by itself -- it is `over`s with no geometry
rem   under them, by design.
rem
rem Then: RigExec -> TouchPose, tick the box, and hover the character.
rem The region under the cursor lights up; click it and the control that
rem owns it becomes usdview's selection, so the Avar Editor and the
rem viewport gizmo follow. While the box is ticked the MESH is not
rem selectable -- a click on unpainted skin selects nothing rather than
rem picking `body_geo`. Untick it (or close the window) and usdview's own
rem picking is back exactly as it was.
rem
rem Alt still drives the camera, untouched.
rem
rem Why this exists rather than a flag on launch_usdview.bat: that script
rem is the RigExec launcher and registers museAssistant and usdNoodles;
rem this one adds plugin\touchPose to PXR_PLUGINPATH_NAME the same way,
rem APPENDING to the canonical value _env.bat sets rather than replacing
rem it (pointing it at the SOURCE schema resources instead of the
rem generated ones fails with a duplicate-plugin registration error).
rem
rem DELIBERATELY NO `cmake --build`: an already-open usdview holds a lock
rem on build\rigExec.dll that would fail the link. TouchPose's native half
rem lives in rigExecImaging (touchPose*.cpp); build first when it changed.
setlocal EnableDelayedExpansion
call "%~dp0_env.bat"
call "%~dp0_require_python.bat"
if errorlevel 1 exit /b 1

set "PXR_PLUGINPATH_NAME=%PXR_PLUGINPATH_NAME%;%RIG%\plugin\touchPose"
rem The Shape Editor rides along: a sibling plugin directory asking
rem for the same RigExec menu, so both land under it.
set "PXR_PLUGINPATH_NAME=%PXR_PLUGINPATH_NAME%;%RIG%\plugin\shapeEditor"
set "PYTHONPATH=%RIG%\plugin\touchPose;%RIG%\plugin\shapeEditor;%PYTHONPATH%"

set "USDVIEW=%USD%\bin\usdview"
if not exist "%USDVIEW%" (
    >&2 echo ERROR: %USDVIEW% not found ^(USD=%USD%^)
    exit /b 1
)

set "STAGE=%~1"
if not defined STAGE set "STAGE=%RIG%\examples\biped\Biped_all.usda"
if not exist "%STAGE%" (
    >&2 echo ERROR: stage not found: %STAGE%
    >&2 echo        build it with:
    >&2 echo          bin\run_touchpose.bat import_touch examples\biped\Biped.usda
    exit /b 1
)

rem A bare (non-flag) argument after the stage is the renderer display
rem name; everything left over goes to usdview untouched, the same
rem convention launch_usdview.bat uses.
shift
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
