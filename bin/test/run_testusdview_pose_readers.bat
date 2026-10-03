@echo off
rem testusdview: the Shape Editor's pose-reader overlay (falloff cones,
rem twist fans, translation spheres) draws over the viewport, live.
rem The Windows twin of run_testusdview_pose_readers.sh.
rem
rem Usage: run_testusdview_pose_readers.bat [renderer]
call "%~dp0..\_vcvars.bat"
call "%~dp0..\_env.bat"
call "%~dp0..\_require_python.bat"
if errorlevel 1 exit /b 1
if exist "%RIG%\build\CMakeCache.txt" (
    cmake --build "%RIG%\build" >nul
    if errorlevel 1 exit /b 1
)
set "PXR_PLUGINPATH_NAME=%PXR_PLUGINPATH_NAME%;%RIG%\plugin\shapeEditor"
set "PYTHONPATH=%RIG%\plugin\shapeEditor;%PYTHONPATH%"
set RENDERER_ARG=
if not "%~1"=="" set RENDERER_ARG=--renderer %~1
"%PY%" "%USD%\bin\testusdview" --testScript "%RIG%\tests\testUsdviewPoseReaderViz.py" %RENDERER_ARG% "%RIG%\examples\biped\Biped_stack.usda"
exit /b %errorlevel%
