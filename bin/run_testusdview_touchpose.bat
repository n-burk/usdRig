@echo off
rem Headless end-to-end test of the TouchPose pick loop
rem (tests\testUsdviewTouchPose.py) on examples\biped\Biped_touch.usda --
rem the COMBINED layer, which sublayers Biped_touch_regions.usda over
rem Biped.usda. The touch-only layer alone has nothing to draw and
rem nothing to select; it is asserted by the importer's own verify pass.
rem
rem Why it exists: three of TouchPose's claims cannot be checked from a
rem script, and all three have been wrong before.
rem
rem   * The HIGHLIGHT has to be seen. It is a Storm shader tint driven by
rem     primvars a Hydra scene index adds (rigExecImaging,
rem     touchPoseHighlight.h), so a wrong shader or a filtered primvar is
rem     perfectly correct in every data structure and moves ZERO pixels.
rem     Only a frame grab can tell, so the test counts pixels -- and
rem     asserts the session layer is byte-identical, since nothing may be
rem     authored.
rem   * The PICK has to follow the DEFORMED mesh. The posed points live
rem     in Hydra, not on the stage, so a test that never runs Hydra
rem     cannot tell a correct pick from one against the rest mesh.
rem   * The EVENT FILTER has to be reached the way Qt reaches it, with a
rem     real QMouseEvent through QApplication.sendEvent -- calling the
rem     method directly would prove the method works and not that the
rem     mode is installed over usdview's stage view.
rem
rem Asserts: TouchPose is registered and MERGED into the existing RigExec
rem menu; every region binds a real drivable prim; a known face's centroid
rem projected to screen and clicked selects that region's control and NOT
rem the mesh; shift adds; the highlight moves a measured fraction of the
rem frame; hips +12 cm moves the region and the pick follows it; a click
rem that misses the character is not consumed; a click on unpainted skin
rem IS consumed and selects nothing; and turning the mode off restores
rem usdview's own picking. Prints RIGEXEC_TOUCHPOSE_OK.
rem
rem Usage: run_testusdview_touchpose.bat [stage.usda] [rendererDisplayName]
rem Set TOUCHPOSE_SHOT=path.png to keep a frame grab of the highlight.
rem
rem DELIBERATELY NO `cmake --build`, unlike run_testusdview_picker.bat: an
rem open usdview holds a lock on build\rigExec.dll and the build fails on
rem the link step, taking the test with it. TouchPose's native half lives
rem in rigExecImaging (touchPose*.cpp), so build first with
rem build_rigexec.bat --no-test when that has changed.
setlocal EnableDelayedExpansion
call "%~dp0_env.bat"
call "%~dp0_require_python.bat"
if errorlevel 1 exit /b 1

rem APPENDED, never replaced: _env.bat is the canonical value and points
rem PXR_PLUGINPATH_NAME at the GENERATED schema resources. The TouchPose
rem container is its own plugin directory, registered here the same way
rem launch_usdview.bat registers museAssistant and usdNoodles.
set "PXR_PLUGINPATH_NAME=%PXR_PLUGINPATH_NAME%;%RIG%\plugin\touchPose"
set "PYTHONPATH=%RIG%\plugin\touchPose;%PYTHONPATH%"

rem testusdview `exec`s the script, so it has no __file__ to locate the
rem package from. Handed over explicitly rather than guessed at, exactly
rem as run_touchpose_view.bat does for the spikes.
set "TOUCHPOSE_PLUGIN_DIR=%RIG%\plugin\touchPose"

set "STAGE=%~1"
if not defined STAGE set "STAGE=%RIG%\examples\biped\Biped_touch.usda"
if not exist "%STAGE%" (
    >&2 echo ERROR: stage not found: %STAGE%
    >&2 echo        build it with:
    >&2 echo          bin\run_touchpose.bat import_touch examples\biped\Biped.usda
    exit /b 1
)

set RENDERER_ARG=
if not "%~2"=="" set RENDERER_ARG=--renderer %~2

"%PY%" "%USD%\bin\testusdview" --testScript "%RIG%\tests\testUsdviewTouchPose.py" %RENDERER_ARG% "%STAGE%"
exit /b %errorlevel%
