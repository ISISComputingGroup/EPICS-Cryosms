@echo off
setlocal
set "ARCH=%1"
set "TESTPATH=%~dp0cryosmsApp/src/O.%ARCH%"
if exist "%TESTPATH%\runner.exe" (
    call %TESTPATH%\dllPath.bat
    %TESTPATH%\runner.exe --gtest_output=xml:./test-reports/TEST-cryosms.xml
) else (
    @echo No tests to run
)
