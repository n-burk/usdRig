@echo off
rem Headless end-to-end test of the Execution Stack panel
rem (tests\testUsdviewExecStack.py) on biped_rigged.usda. The Windows twin
rem of run_testusdview_execstack.sh.
rem
rem Why it exists: the panel was broken in usdview and ONLY in usdview --
rem it compiled the rig with the stage usdview hands out, which comes from
rem a UsdStageCache and which the rigexec.Rig binding refuses ("needs a
rem __owner capsule"), so it showed "Rig compile failed" for every rig
rem while the same rig compiled fine from a script. Nothing over an
rem in-memory stage can catch that, because the bug is about WHICH stage
rem object the panel is handed.
rem
rem Asserts the RigExec menu item is registered (and absent from Window),
rem that opening it populates the ordered mover list, that the order is
rem identical to a script-side compile of the same layers, that every
rem deformer runs after every constraint, and that the filters narrow the
rem list rather than emptying it. Prints RIGEXEC_EXEC_STACK_OK.
rem
rem Usage: run_testusdview_execstack.bat [rendererDisplayName]  (e.g. Embree)
rem Set RIGEXEC_EXECSTACK_SHOT=path.png to keep a window grab.
call "%~dp0_vcvars.bat"
call "%~dp0_env.bat"
call "%~dp0_require_python.bat"
if errorlevel 1 exit /b 1
if exist "%RIG%\build\CMakeCache.txt" (
    cmake --build "%RIG%\build" >nul
    if errorlevel 1 exit /b 1
)
set RENDERER_ARG=
if not "%~1"=="" set RENDERER_ARG=--renderer %~1
"%PY%" "%USD%\bin\testusdview" --testScript "%RIG%\tests\testUsdviewExecStack.py" %RENDERER_ARG% "%RIG%\biped_rigged.usda"
exit /b %errorlevel%
