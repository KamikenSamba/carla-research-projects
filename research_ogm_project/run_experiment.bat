@echo off
setlocal
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0run_experiment.ps1" %*
set EXITCODE=%ERRORLEVEL%
pause
exit /b %EXITCODE%
