@echo off
rem Shared environment for the RigExec helper scripts in this directory.
rem
rem Called, never run directly:  call "%~dp0_env.bat"
rem
rem Every path resolves from this file's own location, so the checkout can live
rem anywhere and the helpers work from any working directory. usd-install is
rem expected as a sibling of the repository (see README). Override RIG or USD
rem in the environment to point elsewhere.
if not defined RIG (for %%I in ("%~dp0..") do set "RIG=%%~fI")
if not defined USD (for %%I in ("%RIG%\..\usd-install") do set "USD=%%~fI")

set "PATH=%RIG%\build;%USD%\bin;%USD%\lib;%PATH%"
set "PYTHONPATH=%RIG%\plugin\rigExecUsdview;%RIG%\plugin\museAssistant;%USD%\Lib\site-packages"

rem The schema resources MUST be the GENERATED directory: only the generated
rem plugInfo carries the LibraryPath and implementsComputeExtent that let Plug
rem load the compute-extent registration. The checked-in copy under plugin\ is
rem a data-only fallback and will not give RigExec prims their bounds.
set "PXR_PLUGINPATH_NAME=%RIG%\build\usd\rigExecSchema\resources;%RIG%\build\usd\rigExecImaging\resources;%RIG%\plugin\rigExecUsdview"

rem RIGEXEC_IMAGING_DLL is deliberately NOT set here. rigExecUsdview
rem .ImagingLibraryPath() resolves the platform's library name and the
rem installed-vs-build layout on its own, and both testusdview scripts go
rem through it. Set the variable yourself only to point at a library outside
rem either layout.
exit /b 0
