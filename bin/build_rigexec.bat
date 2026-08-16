@echo off
rem Configures, builds, and tests RigExec against the installed OpenUSD.
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
if errorlevel 1 exit /b 1
call "%~dp0_env.bat"
rem The build/test step reads the USD python modules from the Windows tree
rem layout, which differs from the site-packages path the plugins use.
set "PYTHONPATH=%USD%\lib\python;%PYTHONPATH%"

cmake -S "%RIG%" -B "%RIG%\build" -G Ninja -DCMAKE_BUILD_TYPE=Release
if errorlevel 1 exit /b 1
cmake --build "%RIG%\build"
if errorlevel 1 exit /b 1
ctest --test-dir "%RIG%\build" --output-on-failure
exit /b %errorlevel%
