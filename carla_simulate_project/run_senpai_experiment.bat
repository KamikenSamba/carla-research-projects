@echo off
setlocal
cd /d "%~dp0"
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0run_senpai_experiment.ps1" %*
exit /b %ERRORLEVEL%
