@echo off
rem Generates the codeless RigExec schema plugin from schema.usda.
setlocal
set USD=D:\work\usdRig\usd-install
set PYTHONPATH=%USD%\Lib\site-packages
set PATH=%USD%\bin;%USD%\lib;%PATH%
cd /d D:\work\usdRig\usdRig\libs\rigExecSchema
python %USD%\bin\usdGenSchema schema.usda ..\..\plugin\rigExecSchema\resources
if errorlevel 1 exit /b 1
rem Substitute the build-system placeholders for a codeless resource plugin.
python -c "import io,re; p=r'..\..\plugin\rigExecSchema\resources\plugInfo.json'; s=io.open(p,encoding='utf-8').read(); s=s.replace('\"LibraryPath\": \"@PLUG_INFO_LIBRARY_PATH@\", ',''); s=s.replace('\"@PLUG_INFO_RESOURCE_PATH@\"','\".\"'); s=s.replace('\"@PLUG_INFO_ROOT@\"','\".\"'); io.open(p,'w',encoding='utf-8').write(s)"
exit /b %errorlevel%
