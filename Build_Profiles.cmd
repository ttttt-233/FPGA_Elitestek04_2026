@echo off
setlocal
pushd "%~dp0"
set "FPGA_PYTHON=python"
where python >nul 2>&1
if errorlevel 1 set "FPGA_PYTHON=py -3"
%FPGA_PYTHON% -B tools\build_profile.py 720p-quality %*
if errorlevel 1 goto done
%FPGA_PYTHON% -B tools\build_profile.py 480p-60 %*
if errorlevel 1 goto done
%FPGA_PYTHON% -B tools\build_multiboot.py %*
:done
set "FPGA_BUILD_EXIT=%ERRORLEVEL%"
popd
exit /b %FPGA_BUILD_EXIT%
