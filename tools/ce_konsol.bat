@echo off
rem CE motoru hizli konsolu: aci yaz, motor gider, hareket ekranda. Arayuz kapali olmali. Port farkliysa: ce_konsol.bat COM5
set PYTHONIOENCODING=utf-8
python "%~dp0ce_konsol.py" %1
pause
