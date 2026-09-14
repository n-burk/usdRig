@echo off
rem Headless end-to-end test of TOUCHPOSE IN THE NODE GRAPH
rem (tests\testUsdviewNoodlesTouch.py) on examples\biped\Biped_all.usda --
rem the combined stage: the layered rig plus the TouchPose regions, which
rem is the only one where both halves of the graph exist at once. Over
rem the regions layer alone there are regions and no controls to link to,
rem and over the rig alone there are no regions.
rem
rem Why it exists: usdNoodles builds its graph from whatever prims a load
rem path hands it, and TouchPose is invisible to the one the editor uses
rem when usdview opens a stage. MEASURED before this test existed:
rem NodeGraphStage.load on Biped_all.usda produced ONE node -- /Biped --
rem and zero links, because that loader reads the stage's ROOT CHILDREN
rem and the 98 regions hang off /Biped/TouchPose. Nothing about that is
rem visible from a unit test of the pieces: each piece was already right
rem (the regions are prims, the relationship reads as a link), and the
rem graph was still empty.
rem
rem Asserts, on the stage usdview itself composed: the TouchPose group is
rem found by property and not by path; every authored region becomes a
rem node; every region's touchpose:control is drawn as a relationship
rem link whose target is BOTH the prim the region names AND a node that
rem is in the graph, so the noodle lands somewhere; the region nodes have
rem authored positions, none at the origin and none overlapping; the
rem 144-to-1,362-int face array costs a pin row and not a rendered value;
rem the Scope schema's proxyPrim/purpose/visibility rows are gone; and
rem selecting the group prim expands to the regions and their controls.
rem Prints RIGEXEC_NOODLES_TOUCH_OK.
rem
rem Usage: run_testusdview_noodles_touch.bat [stage.usda] [rendererDisplayName]
rem Set RIGEXEC_NOODLES_SHOT=path.png to keep a window grab.
rem
rem DELIBERATELY NO `cmake --build` and no Noodles panel: this needs no
rem C++ of its own (an open usdview holds a lock on build\rigExec.dll and
rem the link fails), and a headless runner must not load an extra panel
rem into the app it is asserting against. The test drives the graph MODEL
rem -- the same NodeGraphStage.load the panel calls -- so no GL context
rem is needed and no assertion here can fail for GL reasons.
setlocal EnableDelayedExpansion
call "%~dp0_env.bat"
call "%~dp0_require_python.bat"
if errorlevel 1 exit /b 1

set "STAGE=%~1"
if not defined STAGE set "STAGE=%RIG%\examples\biped\Biped_all.usda"
if not exist "%STAGE%" (
    >&2 echo ERROR: stage not found: %STAGE%
    >&2 echo        build the regions with:
    >&2 echo          bin\run_touchpose.bat import_touch examples\biped\Biped.usda
    exit /b 1
)

set RENDERER_ARG=
if not "%~2"=="" set RENDERER_ARG=--renderer %~2

"%PY%" "%USD%\bin\testusdview" --testScript "%RIG%\tests\testUsdviewNoodlesTouch.py" %RENDERER_ARG% "%STAGE%"
exit /b %errorlevel%
