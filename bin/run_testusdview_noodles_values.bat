@echo off
rem Headless end-to-end test of READING AND EDITING ATTRIBUTE VALUES ON THE
rem NODE ROWS (tests\testUsdviewNoodlesValues.py) on
rem tests\fixtures\noodles_values.usda.
rem
rem Why it exists: the inline value cells are decided by the node's own row
rem geometry -- row height, row slots, the width the layout reserved -- and
rem none of that exists until a node has been laid out by the C++ producer
rem inside a running editor. The unit tests in plugin\usdNoodles\testenv
rem cover the value model and the cell arithmetic; they cannot cover a click
rem landing on the cell it aimed at, a drag committing as ONE undo entry, or
rem the allowedTokens popup listing exactly the tokens the schema declares.
rem
rem It also pins the regression the whole design is arranged around: the
rem outer tenth of each row, on both sides, must STILL start a connection
rem drag. The value cells stop a padding short of it on purpose, and this is
rem where that is proved rather than asserted.
rem
rem So this registers the in-repo editor exactly as launch_usdview.bat does
rem -- both copies present if the install has one -- opens the editor, adds
rem the fixture's prims as nodes, and drives everything with real Qt events.
rem Prints RIGEXEC_NOODLES_VALUES_OK.
rem
rem Usage: run_testusdview_noodles_values.bat [rendererDisplayName]
rem Set RIGEXEC_NOODLES_SHOT=path.png to keep grabs of the editor.
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

"%PY%" "%USD%\bin\testusdview" --testScript "%RIG%\tests\testUsdviewNoodlesValues.py" %RENDERER_ARG% "%RIG%\tests\fixtures\noodles_values.usda"
exit /b %errorlevel%
