@echo off
rem MERCOM-1 renkli motor terminali. Cift tikla. Port farkliysa: renkli_terminal.bat COM5
set PORT=%1
if "%PORT%"=="" set PORT=COM8
where pythonw 1>NUL 2>NUL
if %errorlevel%==0 (start "" pythonw "%~dp0terminal_gui.py" %PORT%) else (start "" python "%~dp0terminal_gui.py" %PORT%)
