@echo off
rem Build build\kws.bin for the iCESugar v1.5.
rem Finds OSS CAD Suite by itself (C:\Program Files\oss-cad-suite, C:\oss-cad-suite, ...) - no environment.bat needed.
rem Usage: build.bat                     build
rem        build.bat --prog E:           build and copy kws.bin to the iCELink drive E:
rem        build.bat --tools "D:\x\oss-cad-suite"
setlocal
cd /d "%~dp0"
where python >nul 2>nul && (set PY=python) || (set PY=py -3)
%PY% "%~dp0build.py" %*
set RC=%ERRORLEVEL%
if not "%RC%"=="0" echo. & echo Build FAILED - see build\build.log
rem keep the window open when started by double-click from Explorer
echo %cmdcmdline% | find /i "/c" >nul && pause
exit /b %RC%
