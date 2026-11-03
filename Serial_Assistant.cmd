@echo off
setlocal
pushd "%~dp0"
where python >nul 2>&1
if errorlevel 1 (
  py -3 -B tools\uart\gui.py
) else (
  python -B tools\uart\gui.py
)
set "UART_EXIT=%ERRORLEVEL%"
popd
exit /b %UART_EXIT%
