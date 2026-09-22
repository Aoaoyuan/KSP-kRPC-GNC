@echo off
setlocal
cd /d "%~dp0"

if exist "%~dp0local-python.cmd" call "%~dp0local-python.cmd"
if defined KSP_PYTHON goto run

where py >nul 2>nul
if not errorlevel 1 (
    set "KSP_PYTHON=py"
    set "KSP_PYTHON_ARGS=-3"
    goto run
)

where python >nul 2>nul
if not errorlevel 1 (
    set "KSP_PYTHON=python"
    goto run
)

echo Python 3 was not found. Install Python or create local-python.cmd.
pause
exit /b 1

:run
rem One-take cinematic preview.
"%KSP_PYTHON%" %KSP_PYTHON_ARGS% run_with_local_deps.py ShootHeavyFlight.py --profile recap
set "KSP_EXIT=%ERRORLEVEL%"
echo.
if not "%KSP_EXIT%"=="0" echo Task stopped with error %KSP_EXIT%.
pause
exit /b %KSP_EXIT%
