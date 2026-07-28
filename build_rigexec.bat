@echo off
rem Configures, builds, and tests RigExec against the installed OpenUSD.
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
if errorlevel 1 exit /b 1
set USD=D:\work\usdRig\usd-install
set PATH=%USD%\bin;%USD%\lib;%PATH%
set PYTHONPATH=%USD%\lib\python;%PYTHONPATH%

cmake -S D:\work\usdRig\usdRig -B D:\work\usdRig\usdRig\build -G Ninja -DCMAKE_BUILD_TYPE=Release
if errorlevel 1 exit /b 1
cmake --build D:\work\usdRig\usdRig\build
if errorlevel 1 exit /b 1
ctest --test-dir D:\work\usdRig\usdRig\build --output-on-failure
exit /b %errorlevel%
