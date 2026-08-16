@echo off
rem Runs a python probe script in the RigExec plugin environment.
rem Usage: run_probe.bat <script.py>
call "%~dp0_env.bat"
python %1
exit /b %errorlevel%
