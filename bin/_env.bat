@echo off
rem Shared environment for the RigExec helper scripts in this directory.
rem
rem Called, never run directly:  call "%~dp0_env.bat"
rem
rem Every path resolves from this file's own location, so the checkout can live
rem anywhere and the helpers work from any working directory. usd-install is
rem expected as a sibling of the repository (see README). Override RIG, USD,
rem VENV or PY in the environment to point elsewhere.
rem
rem The POSIX twin is _env.sh, and the two are kept deliberately parallel:
rem same variable names, same defaults, same failure messages.
rem
rem No setlocal: these values are for the caller.
if not defined RIG (for %%I in ("%~dp0..") do set "RIG=%%~fI")
if not defined USD (for %%I in ("%RIG%\..\usd-install") do set "USD=%%~fI")

rem The python that has to import pxr. An explicit PY wins; then a venv, in
rem either layout; then whatever python is on PATH, which is the right answer
rem when the USD build's own interpreter is already the active one.
if not defined VENV (
    for %%V in ("%RIG%\..\usd-pr4156-venv" "%RIG%\..\usd-venv" "%RIG%\..\venv" "%RIG%\.venv") do (
        if not defined VENV if exist "%%~fV\Scripts\python.exe" set "VENV=%%~fV"
    )
)
if not defined PY if defined VENV if exist "%VENV%\Scripts\python.exe" set "PY=%VENV%\Scripts\python.exe"
if not defined PY for /f "delims=" %%P in ('where python 2^>nul') do if not defined PY set "PY=%%P"
if not defined PY set "PY=python"

rem Where THIS USD build keeps its python modules. A Windows install uses
rem Lib\site-packages; a build configured the POSIX way puts them under
rem lib\python. Both are added when both exist rather than one being assumed,
rem which is what let build_rigexec.bat and gen_schema.bat disagree about it.
rem build\python is where the build stages the UsdNoodles package beside its
rem native module; launch_usdview.bat registers it.
set "PYTHONPATH=%RIG%\plugin\rigExecUsdview;%RIG%\plugin\museAssistant;%RIG%\build\python"
if exist "%USD%\Lib\site-packages" set "PYTHONPATH=%PYTHONPATH%;%USD%\Lib\site-packages"
if exist "%USD%\lib\python" set "PYTHONPATH=%PYTHONPATH%;%USD%\lib\python"

set "PATH=%RIG%\build;%USD%\bin;%USD%\lib;%PATH%"

rem The schema resources MUST be the GENERATED directory: only the generated
rem plugInfo carries the LibraryPath and implementsComputeExtent that let Plug
rem load the compute-extent registration. The checked-in copy under plugin\ is
rem a data-only fallback and will not give RigExec prims their bounds.
set "PXR_PLUGINPATH_NAME=%RIG%\build\usd\rigExecSchema\resources;%RIG%\build\usd\rigExecImaging\resources;%RIG%\plugin\rigExecUsdview"

rem The python tests resolve pxr themselves through this, rather than
rem inheriting a PYTHONPATH that may or may not carry it (see
rem tests\python\rigexec_test_env.py). Set here so a helper-run test and a
rem ctest-run test read the same install.
set "RIGEXEC_USD_INSTALL=%USD%"

rem RIGEXEC_IMAGING_DLL is deliberately NOT set here. rigExecUsdview
rem .ImagingLibraryPath() resolves the platform's library name and the
rem installed-vs-build layout on its own, and both testusdview scripts go
rem through it. Set the variable yourself only to point at a library outside
rem either layout.
exit /b 0
