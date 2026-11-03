@echo off
setlocal
pushd "%~dp0"
where python >nul 2>&1
if errorlevel 1 (
  py -3 -B tools\fpga_toolchain.py build %*
) else (
  python -B tools\fpga_toolchain.py build %*
)
set "FPGA_BUILD_EXIT=%ERRORLEVEL%"
popd
exit /b %FPGA_BUILD_EXIT%
