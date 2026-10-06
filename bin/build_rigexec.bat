@echo off
rem Configures, builds, and tests RigExec against the installed OpenUSD.
rem The Windows twin of build_rigexec.sh, and it passes the same three cache
rem variables.
rem
rem Usage: build_rigexec.bat [--no-test]
rem JOBS caps parallel compile jobs (default 8): the rigExec translation
rem units each eat ~1GB, so Ninja's CPU-count default OOMs the compiler on
rem smaller machines. build_rigexec.sh honors the same variable.
if not defined JOBS set "JOBS=8"
call "%~dp0_vcvars.bat"
call "%~dp0_env.bat"
call "%~dp0_require_python.bat"
if errorlevel 1 exit /b 1

rem CMAKE_PREFIX_PATH is not optional: without it pxrConfig's
rem find_dependency(OpenSubdiv 3.6.1) can resolve against an older OpenSubdiv
rem elsewhere on the machine and the configure fails with a version mismatch.
rem USD_INSTALL_DIR must be passed too, or a USD anywhere but the default
rem sibling path is not found at all.
set "_RIGEXEC_MOVER_ARGS="
if defined RIGEXEC_MOVER_PLUGIN_DIRS set _RIGEXEC_MOVER_ARGS="-DRIGEXEC_MOVER_PLUGIN_DIRS=%RIGEXEC_MOVER_PLUGIN_DIRS%"
cmake -S "%RIG%" -B "%RIG%\build" -G Ninja ^
      -DCMAKE_BUILD_TYPE=Release ^
      -DUSD_INSTALL_DIR="%USD%" ^
      -DCMAKE_PREFIX_PATH="%USD%" %_RIGEXEC_MOVER_ARGS%
if errorlevel 1 exit /b 1
cmake --build "%RIG%\build" -j %JOBS%
if errorlevel 1 exit /b 1

if "%~1"=="--no-test" exit /b 0
ctest --test-dir "%RIG%\build" --output-on-failure
exit /b %errorlevel%
