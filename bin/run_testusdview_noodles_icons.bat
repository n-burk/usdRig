@echo off
rem Headless end-to-end test of NODE TITLE-BAR ICONS
rem (tests\testUsdviewNoodlesIcons.py) on tests\fixtures\noodles_icons.usda.
rem
rem Why it exists: every docs example authors
rem `uniform asset ui:nodegraph:node:icon = @../../icons/<node>.png@` on its
rem prims, and the Noodles Editor drew the built-in box on every node anyway.
rem The node factory set a PYTHON attribute on the node after the pin setters
rem had run their last sync to the C++ NodeData, so titleIconPath -- the only
rem field the C++ icon producer reads -- stayed empty. Nothing raised and
rem nothing was missing on screen, which is why no test caught it: the fallback
rem icon looks like an icon.
rem
rem The second half is path resolution. The authored path is relative to the
rem LAYER, so handing the renderer the authored string would make the icon
rem depend on where usdview was started, and a file that does not resolve must
rem still anchor and then fall back silently.
rem
rem So this registers the in-repo editor exactly as launch_usdview.bat does,
rem opens the editor, adds the fixture's prims from the prim tree, and checks
rem the resolved path, the loadable image, the missing-file fallback, and --
rem end to end -- that the title area's pixels change when the icon is
rem cleared. Prints RIGEXEC_NOODLES_ICONS_OK.
rem
rem Usage: run_testusdview_noodles_icons.bat [rendererDisplayName]
rem Set RIGEXEC_NOODLES_SHOT=path.png to keep a grab of the open editor.
rem
rem No build step: an open usdview holds a lock on build\rigExec.dll. Build
rem first (bin\build_rigexec.bat) so build\python\UsdNoodles is current.
setlocal EnableDelayedExpansion
call "%~dp0_env.bat"
call "%~dp0_require_python.bat"
if errorlevel 1 exit /b 1

if not exist "%RIG%\build\python\UsdNoodles\plugInfo.json" (
    >&2 echo ERROR: build\python\UsdNoodles is not staged; run bin\build_rigexec.bat
    exit /b 1
)
set "PXR_PLUGINPATH_NAME=%PXR_PLUGINPATH_NAME%;%RIG%\build\python\UsdNoodles"

set RENDERER_ARG=
if not "%~1"=="" set RENDERER_ARG=--renderer %~1

"%PY%" "%USD%\bin\testusdview" --testScript "%RIG%\tests\testUsdviewNoodlesIcons.py" %RENDERER_ARG% "%RIG%\tests\fixtures\noodles_icons.usda"
exit /b %errorlevel%
