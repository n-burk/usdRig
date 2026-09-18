@echo off
rem Headless Qt-free tests for the usdview plugin modules (undo stack, gizmo
rem math, gizmo screen helpers, the layer-opinion and composition-arc
rem authoring models). The Windows twin of run_python_tests.sh.
rem
rem These need no build and no display; ctest runs the same files, so this is
rem the quick loop while editing a plugin module.
rem
rem Usage: run_python_tests.bat [test_name ...]   (default: all)
setlocal EnableDelayedExpansion
call "%~dp0_env.bat"
call "%~dp0_require_python.bat"
if errorlevel 1 exit /b 1

rem The native evaluator binding is optional (built in build-python, not
rem build); test_gizmo_math compares against it when importable.
set "PYTHONPATH=%RIG%\build-python\python;%PYTHONPATH%"
set "SCHEMA=%RIG%\build\usd\rigExecSchema\resources"

set "TESTS=%*"
if not defined TESTS set "TESTS=test_rigexec_undo test_gizmo_math test_gizmo_screen test_gizmo_settings test_gizmo_drag test_gizmo_snap test_viewcube_math test_rigexec_stage_edits test_graph_model test_graph_screen test_layer_opinions_model test_composition_arcs_model test_gizmo_preview test_touchpose_model test_gizmo_marquee test_picker_scene"

rem Only the tests listed here read argv[1], to Plug-register the generated
rem schema. The rest never touch sys.argv -- they need neither a schema nor a
rem build -- so handing them %SCHEMA% would tell a reader otherwise.
set "SCHEMA_TESTS= test_rigexec_undo test_gizmo_math test_rigexec_stage_edits test_graph_model test_picker_scene "

for %%T in (%TESTS%) do (
    echo == %%T
    echo !SCHEMA_TESTS! | findstr /c:" %%T " >nul
    if errorlevel 1 (
        "%PY%" "%RIG%\tests\python\%%T.py"
    ) else (
        "%PY%" "%RIG%\tests\python\%%T.py" "%SCHEMA%"
    )
    if errorlevel 1 (
        >&2 echo FAILED: %%T
        exit /b 1
    )
)
exit /b 0
