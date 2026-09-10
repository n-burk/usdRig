@echo off
rem Verifies the Muse assistant, headless then inside usdview. The Windows twin
rem of test_muse.sh.
rem
rem   1. tests\testMuseAgent.py    message shaping, tool loop, PNG camera
rem                                metadata (no Qt, no display, no network,
rem                                no API key)
rem   2. tests\testUsdviewMuse.py  the panel driving a REAL stage in a real
rem                                usdview: the model's edits must land, the
rem                                viewport capture must be a real image, and
rem                                an attached screenshot's camera must reach it
rem
rem Neither default test talks to a model -- both script their transport -- so
rem no API key or local server is needed. Passing prints MUSE_AGENT_OK and
rem MUSE_USDVIEW_OK. MUSE_LIVE=1 uses the selected real provider.
rem
rem Usage: test_muse.bat [stage.usda]   (default: examples\ArmShotAnim.usda)
setlocal
call "%~dp0_vcvars.bat"
call "%~dp0_env.bat"
call "%~dp0_require_python.bat"
if errorlevel 1 exit /b 1

set "STAGE=%~1"
if "%STAGE%"=="" set "STAGE=%RIG%\examples\ArmShotAnim.usda"
if not exist "%STAGE%" (
    >&2 echo ERROR: stage not found: %STAGE%
    exit /b 1
)
if not exist "%USD%\bin\testusdview" (
    >&2 echo ERROR: %USD%\bin\testusdview not found ^(USD=%USD%^)
    >&2 echo        set USD=\path\to\usd-install and retry.
    exit /b 1
)

rem _env.bat has already set PYTHONPATH and the schema plugin paths. Only the
rem Muse panel's own plugin container is added here, which is what separates
rem this from the other headless runners: they must not load an extra panel
rem into the app they are asserting against.
set "PXR_PLUGINPATH_NAME=%PXR_PLUGINPATH_NAME%;%RIG%\plugin\museAssistant"

echo == headless: agent core ==
rem The unit suite scripts each transport itself. Do not let a provider
rem selected for the opt-in live leg redirect earlier fake-Anthropic cases to a
rem real local server; local-provider cases set their own fake endpoints.
set "MUSE_PROVIDER="
set "MUSE_APPLE_URL="
set "MUSE_LMSTUDIO_URL="
"%PY%" "%RIG%\tests\testMuseAgent.py"
if errorlevel 1 exit /b 1

echo.
echo == in usdview: panel against a live stage ==
"%PY%" "%USD%\bin\testusdview" --testScript "%RIG%\tests\testUsdviewMuse.py" "%STAGE%"
if errorlevel 1 exit /b 1

rem Opt-in: the only test that proves asking in English changes the stage.
rem Makes real model calls through the configured provider, so it is not run by
rem default.
if "%MUSE_LIVE%"=="1" (
    echo.
    echo == LIVE: real model, real stage ^(MUSE_LIVE=1^) ==
    "%PY%" "%USD%\bin\testusdview" --testScript "%RIG%\tests\testUsdviewMuseLive.py" "%STAGE%"
    exit /b %errorlevel%
)
echo.
echo (set MUSE_LIVE=1 to also run the live end-to-end test against the real model)
exit /b 0
