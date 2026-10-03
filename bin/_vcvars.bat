@echo off
rem Shared Visual Studio toolchain initialization for the RigExec helpers.
rem
rem Called, never run directly:  call "%~dp0_vcvars.bat"
rem
rem The helpers rebuild the tree before they run, so cl.exe and ninja have to
rem be on PATH. Which vcvars64.bat provides them is NOT a fixed path: it moves
rem with the edition (Community, Professional, Enterprise, BuildTools), the
rem year, and a non-default install drive, so it is discovered rather than
rem written down:
rem
rem   1. an environment already initialized -- a Developer Command Prompt, or
rem      a second helper called after the first -- is left alone;
rem   2. RIGEXEC_VCVARS, when the caller points at one explicitly;
rem   3. vswhere.exe, which every VS 2017+ installer puts at one fixed path
rem      and which is the supported way to ask where VS is;
rem   4. the well-known locations, for an install vswhere does not report.
rem
rem Finding none is NOT an error. A machine building with clang-cl, or one
rem whose compiler is already on PATH, is a supported way to work; the build
rem step itself reports a toolchain that is really missing.
rem
rem No setlocal anywhere in this file: the whole point is the environment
rem vcvars64.bat sets, and endlocal would discard it.

if defined VCINSTALLDIR exit /b 0

if defined RIGEXEC_VCVARS (
    if exist "%RIGEXEC_VCVARS%" (
        call "%RIGEXEC_VCVARS%" >nul 2>&1
        exit /b 0
    )
    >&2 echo WARNING: RIGEXEC_VCVARS=%RIGEXEC_VCVARS% does not exist.
)

rem %ProgramFiles(x86)% carries parentheses, which cmd mis-parses inside a
rem parenthesized block; copy it to a plain name first and use that below.
set "_RIGEXEC_PFX86=%ProgramFiles(x86)%"
if not defined _RIGEXEC_PFX86 set "_RIGEXEC_PFX86=%ProgramFiles%"
set "_RIGEXEC_VSWHERE=%_RIGEXEC_PFX86%\Microsoft Visual Studio\Installer\vswhere.exe"
set "_RIGEXEC_VSOUT=%TEMP%\rigexec_vswhere.txt"
set "_RIGEXEC_VCVARS="

rem vswhere's answer goes through a file rather than through a for /f
rem backquote: cmd re-quotes a quoted program path inside one and the command
rem comes back "not recognized", and the current directory is not searched
rem when NoDefaultCurrentDirectoryInExePath is set, so neither of the two
rem obvious ways round it is dependable.
if exist "%_RIGEXEC_VSWHERE%" (
    "%_RIGEXEC_VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath > "%_RIGEXEC_VSOUT%" 2>nul
    for /f "usebackq tokens=*" %%I in ("%_RIGEXEC_VSOUT%") do (
        if exist "%%I\VC\Auxiliary\Build\vcvars64.bat" (
            set "_RIGEXEC_VCVARS=%%I\VC\Auxiliary\Build\vcvars64.bat"
        )
    )
    del "%_RIGEXEC_VSOUT%" >nul 2>&1
)

if not defined _RIGEXEC_VCVARS (
    for %%Y in (2026 2025 2022 2019) do (
        for %%E in (Enterprise Professional Community BuildTools Preview) do (
            if not defined _RIGEXEC_VCVARS (
                if exist "%ProgramFiles%\Microsoft Visual Studio\%%Y\%%E\VC\Auxiliary\Build\vcvars64.bat" (
                    set "_RIGEXEC_VCVARS=%ProgramFiles%\Microsoft Visual Studio\%%Y\%%E\VC\Auxiliary\Build\vcvars64.bat"
                ) else (
                    if exist "%_RIGEXEC_PFX86%\Microsoft Visual Studio\%%Y\%%E\VC\Auxiliary\Build\vcvars64.bat" (
                        set "_RIGEXEC_VCVARS=%_RIGEXEC_PFX86%\Microsoft Visual Studio\%%Y\%%E\VC\Auxiliary\Build\vcvars64.bat"
                    )
                )
            )
        )
    )
)

rem Both streams are discarded, not just stdout: vcvars64.bat runs vswhere
rem itself and complains on stderr on some machines even as it succeeds, so
rem its noise is not a signal. Whether it worked is read from the environment
rem it is supposed to have set.
if defined _RIGEXEC_VCVARS (
    call "%_RIGEXEC_VCVARS%" >nul 2>&1
    if defined VCINSTALLDIR exit /b 0
    >&2 echo WARNING: "%_RIGEXEC_VCVARS%" did not initialize the toolchain.
    >&2 echo          Run it in a terminal to see why.
    exit /b 0
)

>&2 echo NOTE: no Visual Studio vcvars64.bat found; using whatever compiler is
>&2 echo       already on PATH. Set RIGEXEC_VCVARS=\path\to\vcvars64.bat to
>&2 echo       choose one.
exit /b 0
