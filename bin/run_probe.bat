@echo off
rem Runs a python probe script in the RigExec plugin environment.
rem Usage: run_probe.bat <script.py>
call "%~dp0_env.bat"
call "%~dp0_require_python.bat"
if errorlevel 1 exit /b 1
"%PY%" %1
exit /b %errorlevel%
