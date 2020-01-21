:: Run all tests
@echo off
SET TOP="."

SET Tests_failed=%errorlevel%

:: Change the directory depending on if you have a src sub directory
call CRYOSMSApp\src\O.windows-x64-debug\runner.exe --gtest_output=xml:%TOP%\test-reports\TEST-cryosms.xml

exit /B %Tests_failed%
