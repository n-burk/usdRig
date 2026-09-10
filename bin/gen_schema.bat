@echo off
rem Generates the codeless RigExec schema plugin from schema.usda.
rem The Windows twin of gen_schema.sh.
rem
rem Writes into plugin\rigExecSchema\resources, which is CHECKED IN: run it
rem only after editing libs\rigExecSchema\schema.usda, and review the diff.
setlocal
call "%~dp0_env.bat"
call "%~dp0_require_python.bat"
if errorlevel 1 exit /b 1
if not exist "%USD%\bin\usdGenSchema" (
    >&2 echo ERROR: %USD%\bin\usdGenSchema not found ^(USD=%USD%^)
    >&2 echo        set USD=\path\to\usd-install and retry.
    exit /b 1
)
cd /d "%RIG%\libs\rigExecSchema"
"%PY%" "%USD%\bin\usdGenSchema" schema.usda ..\..\plugin\rigExecSchema\resources
if errorlevel 1 exit /b 1
rem Substitute the build-system placeholders for a codeless resource plugin.
rem The generated plugInfo names a LibraryPath that a codeless schema has no
rem library for, and Plug refuses to load the plugin while it is present.
"%PY%" -c "import io; p=r'..\..\plugin\rigExecSchema\resources\plugInfo.json'; s=io.open(p,encoding='utf-8').read(); s=s.replace('\"LibraryPath\": \"@PLUG_INFO_LIBRARY_PATH@\", ',''); s=s.replace('\"@PLUG_INFO_RESOURCE_PATH@\"','\".\"'); s=s.replace('\"@PLUG_INFO_ROOT@\"','\".\"'); io.open(p,'w',encoding='utf-8').write(s)"
if errorlevel 1 exit /b 1
echo regenerated plugin\rigExecSchema\resources -- review the diff before committing
exit /b 0
