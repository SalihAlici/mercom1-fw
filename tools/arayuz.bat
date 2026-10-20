@echo off
rem MERCOM-1 ana arayuz (kartta ANA FIRMWARE yuklu olmali). Cift tikla. Port farkliysa: arayuz.bat COM5
set PORT=%1
if "%PORT%"=="" set PORT=COM8
where pythonw 1>NUL 2>NUL
if %errorlevel%==0 (start "" pythonw "%~dp0mercom_gui.py" %PORT%) else (start "" python "%~dp0mercom_gui.py" %PORT%)
