@echo off
title MERCOM-1 kilit testi
rem Kartta ANA FIRMWARE yuklu olmali, arayuz kapali olmali (port). Baslik IMU'ya gore tilt 30'a alinir, kilit alinir,
rem otomatik moda gecilir ve 60 s boyunca nisan hatasi yazilir. "OTOMATIK MOD ACIK" yazinca tabani yavas ve kucuk eg.
set PYTHONIOENCODING=utf-8
echo Kilit testi basliyor. Baslik 30 dereceye alinacak, kilit alinacak, otomatik moda gecilecek.
echo "OTOMATIK MOD ACIK" yazisini gorunce tabani YAVAS ve KUCUK eg: +-8 derece, 10 saniyede bir gidis-gelis.
echo.
python "%~dp0kilit_testi.py"
echo.
echo Test bitti. Bu pencereyi kapatip arayuz.bat ile arayuzu acabilirsin.
pause
