@echo off
rem Regenerates the C++ headers of the .rigexec FlatBuffers schemas.
rem The Windows twin of gen_flatbuffers.sh.
rem
rem Writes into libs\rigExecBinary\generated, which is CHECKED IN: run it
rem only after editing libs\rigExecBinary\rigexec.fbs or presentation.fbs,
rem bump RigExecFormatVersion (libs\rigExecBinary\format.h) with any
rem rigexec.fbs change, and review the diff.
rem
rem flatc is not vendored. Point FLATC at a flatc 25.12.19 binary, or put
rem one on PATH; any other version is refused, because the generated
rem headers static_assert the runtime headers in thirdparty\flatbuffers.
setlocal
call "%~dp0_env.bat"
set "_REQUIRED=flatc version 25.12.19"

if not defined FLATC for /f "delims=" %%F in ('where flatc 2^>nul') do if not defined FLATC set "FLATC=%%F"
if defined FLATC if exist "%FLATC%" goto :have_flatc
>&2 echo ERROR: flatc not found. Set FLATC=\path\to\flatc.exe [%_REQUIRED%]
>&2 echo        or put it on PATH, and retry.
exit /b 1
:have_flatc
rem The run below happens from %RIG%: a relative path must not depend on it.
for %%F in ("%FLATC%") do set "FLATC=%%~fF"

rem The version goes through a file rather than a for /f backquote, which
rem re-quotes a quoted program path (see _vcvars.bat).
set "_VERSION_OUT=%TEMP%\rigexec_flatc_version.txt"
set "_VERSION="
"%FLATC%" --version > "%_VERSION_OUT%" 2>nul
for /f "usebackq delims=" %%V in ("%_VERSION_OUT%") do if not defined _VERSION set "_VERSION=%%V"
del "%_VERSION_OUT%" >nul 2>&1
if "%_VERSION%"=="%_REQUIRED%" goto :have_version
>&2 echo ERROR: %FLATC% reports "%_VERSION%"; %_REQUIRED% is required.
exit /b 1
:have_version

cd /d "%RIG%"

rem A float or double scalar in a table loses -0.0: the builder omits a
rem value equal to its default. rigexec.fbs keeps those in F64/F32 structs
rem instead. Comments are stripped, then every table body is split into
rem fields.
powershell -NoProfile -ExecutionPolicy Bypass -Command "$t = [IO.File]::ReadAllText('libs\rigExecBinary\rigexec.fbs') -replace '//[^\r\n]*', ''; $bad = foreach ($m in [regex]::Matches($t, '(?<![A-Za-z0-9_])table\s+([A-Za-z_]\w*)[^{]*\{([^}]*)\}')) { foreach ($f in ($m.Groups[2].Value -split ';')) { if ($f -cmatch '^\s*[A-Za-z_]\w*\s*:\s*(float|double|float32|float64)\s*(=|\(|$)') { $m.Groups[1].Value + '.' + $f.Trim() } } }; if ($bad) { [Console]::Error.WriteLine('ERROR: float/double scalar fields in rigexec.fbs tables (use F64/F32):'); foreach ($b in $bad) { [Console]::Error.WriteLine('  ' + $b) }; exit 1 }; exit 0"
if errorlevel 1 exit /b 1

rem --reflect-types emits the type tables RigExecFormatOpen bounds a buffer
rem with before the verifier reads it.
"%FLATC%" --cpp --cpp-std c++17 --scoped-enums --gen-object-api ^
    --object-prefix RigExecWire --object-suffix "" ^
    --cpp-field-case-style lower --warnings-as-errors --reflect-types ^
    --include-prefix rigExecBinary/generated/ --keep-prefix ^
    -o libs/rigExecBinary/generated ^
    libs/rigExecBinary/rigexec.fbs libs/rigExecBinary/presentation.fbs
if errorlevel 1 exit /b 1

echo regenerated libs\rigExecBinary\generated -- review the diff
exit /b 0
