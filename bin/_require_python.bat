@echo off
rem Fail early and legibly rather than deep inside python. The Windows twin of
rem rigexec_require_python in _env.sh, and it checks the same two things in
rem the same order: that there is an interpreter, and that it can see pxr.
rem
rem Called after _env.bat:  call "%~dp0_require_python.bat" || exit /b 1
rem
rem Usage note: cmd has no `||` on a `call`, so the caller checks errorlevel:
rem     call "%~dp0_require_python.bat"
rem     if errorlevel 1 exit /b 1
if not defined PY (
    >&2 echo ERROR: no python found. Set PY=\path\to\python.exe and retry.
    exit /b 1
)
where "%PY%" >nul 2>&1
if errorlevel 1 if not exist "%PY%" (
    >&2 echo ERROR: python not found at %PY%
    >&2 echo        Set PY=\path\to\python.exe or VENV=\path\to\venv, and retry.
    exit /b 1
)
rem `from pxr import Usd` and not `import pxr`: the latter is a namespace
rem package that imports on any interpreter, including one whose version
rem cannot load the build's extension modules at all. Loading a real module is
rem the only check that tells the two apart.
"%PY%" -c "from pxr import Usd" >nul 2>&1
if errorlevel 1 (
    >&2 echo ERROR: %PY% cannot load pxr.
    >&2 echo        USD=%USD%
    >&2 echo        Either that is not an OpenUSD install -- set
    >&2 echo        USD=\path\to\usd-install -- or this interpreter is not the
    >&2 echo        one it was built against, in which case set PY to that one.
    exit /b 1
)
exit /b 0
