@echo off
rem Kartin seri cikisini canli izler (islemci besleme sinamasi, blink, lazer testi). Port farkliysa: seri_izle.bat COM5
set PYTHONIOENCODING=utf-8
python "%~dp0seri_izle.py" %1
pause
