@echo off
setlocal
set "PLAYGROUND_PYTHON=%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
if exist "%PLAYGROUND_PYTHON%" (
    "%PLAYGROUND_PYTHON%" "%~dp0joint_playground.py"
) else (
    py -3 "%~dp0joint_playground.py"
)
if errorlevel 1 pause
