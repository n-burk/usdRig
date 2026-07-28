@echo off
set USD=D:\work\usdRig\usd-install
set RIG=D:\work\usdRig\usdRig
set PATH=%RIG%\build;%USD%\bin;%USD%\lib;%PATH%
set PYTHONPATH=%RIG%\plugin\rigExecUsdview;%USD%\Lib\site-packages
set PXR_PLUGINPATH_NAME=%RIG%\plugin\rigExecSchema\resources;%RIG%\plugin\rigExecImaging\resources;%RIG%\plugin\rigExecUsdview
python %1
exit /b %errorlevel%
