@echo off
rem Configures, builds, installs, and tests RigExec against the OpenUSD
rem vendored inside a Houdini install, in build-houdini. The Houdini twin of
rem build_rigexec.bat: same generator, same build type, same test gate, but
rem USD comes from cmake\houdini\pxrConfig.cmake (headers in
rem toolkit\include, import libraries in custom\houdini\dsolib) instead of a
rem stock OpenUSD install, and the Python bindings target Houdini's own
rem Python 3.13. The default stock-USD build is untouched.
rem
rem Usage: build_rigexec_houdini.bat [--no-test] [--no-install]
rem
rem Environment overrides:
rem   HFS / HOUDINI_ROOT  Houdini install (default: Steam Houdini Indie)
rem   RIG                 repo root (default: this file's parent directory)
rem   BUILD               build dir (default: %RIG%\build-houdini)
rem   JOBS                parallel compile jobs (default: 8; the rigExec
rem                       translation units each eat ~1GB against the
rem                       Houdini headers, so Ninja's CPU-count default
rem                       OOMs the compiler on smaller machines)
rem   PYBIND11_DIR        pybind11 CMake dir (default: probed from the
rem                       interpreters on PATH; Houdini's own python has none)
setlocal

call "%~dp0_vcvars.bat"

if not defined RIG (for %%I in ("%~dp0..") do set "RIG=%%~fI")
if not defined BUILD set "BUILD=%RIG%\build-houdini"
if not defined HOUDINI_ROOT if defined HFS set "HOUDINI_ROOT=%HFS%"
if not defined HOUDINI_ROOT set "HOUDINI_ROOT=D:\SteamLibrary\steamapps\common\Houdini Indie"
if not exist "%HOUDINI_ROOT%\bin\hython.exe" (
    >&2 echo ERROR: no Houdini install at "%HOUDINI_ROOT%".
    >&2 echo        Set HFS=^<houdini-root^> and retry.
    exit /b 1
)

rem The bindings build against Houdini's interpreter, which ships no
rem pybind11; reuse any pip-installed copy. Its headers are
rem version-agnostic, so a copy found under another interpreter's
rem site-packages is fine.
if not defined PYBIND11_DIR (
    for /f "delims=" %%D in ('python -m pybind11 --cmakedir 2^>nul') do if not defined PYBIND11_DIR set "PYBIND11_DIR=%%D"
)
if not defined PYBIND11_DIR (
    for /f "delims=" %%D in ('py -3.10 -m pybind11 --cmakedir 2^>nul') do if not defined PYBIND11_DIR set "PYBIND11_DIR=%%D"
)
if not defined PYBIND11_DIR (
    for /f "delims=" %%D in ('py -3.12 -m pybind11 --cmakedir 2^>nul') do if not defined PYBIND11_DIR set "PYBIND11_DIR=%%D"
)
if not defined PYBIND11_DIR (
    >&2 echo ERROR: pybind11 not found. Install it for any interpreter
    >&2 echo        (pip install pybind11) or set PYBIND11_DIR=^<cmakedir^>.
    exit /b 1
)

cmake -S "%RIG%" -B "%BUILD%" -G Ninja ^
      -DCMAKE_BUILD_TYPE=Release ^
      -DUSD_INSTALL_DIR="%RIG%\cmake\houdini" ^
      -DHOUDINI_ROOT="%HOUDINI_ROOT%" ^
      -DRIGEXEC_TEST_USD_DIR="%HOUDINI_ROOT%" ^
      -DRIGEXEC_BUILD_USDNOODLES=OFF ^
      -DRIGEXEC_BUILD_PYTHON=ON ^
      -DPython3_EXECUTABLE="%HOUDINI_ROOT%\python313\python.exe" ^
      -Dpybind11_DIR="%PYBIND11_DIR%" ^
      -DCMAKE_INSTALL_PREFIX="%BUILD%\install"
if errorlevel 1 exit /b 1
if not defined JOBS set "JOBS=8"
cmake --build "%BUILD%" -j %JOBS%
if errorlevel 1 exit /b 1

if "%~1"=="--no-install" exit /b 0
if "%~2"=="--no-install" exit /b 0
cmake --install "%BUILD%"
if errorlevel 1 exit /b 1

if "%~1"=="--no-test" exit /b 0
if "%~2"=="--no-test" exit /b 0
rem The test binaries link the vendored USD DLLs, so Houdini's bin has to be
rem on PATH while they run. PXR_PLUGINPATH_NAME points at the generated
rem plugInfo pair, mirroring _env.bat; Houdini's python already imports pxr.
set "PATH=%BUILD%;%HOUDINI_ROOT%\bin;%PATH%"
set "PXR_PLUGINPATH_NAME=%BUILD%\usd\rigExecSchema\resources;%BUILD%\usd\rigExecImaging\resources"
ctest --test-dir "%BUILD%" --output-on-failure
exit /b %errorlevel%
