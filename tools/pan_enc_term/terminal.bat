@echo off
title MERCOM-1 motor terminali
echo Motor terminali: COM8, 115200. Komut yaz, Enter ile gonder. Cikis: Ctrl+C
echo Ornek: durum ^| hiz 40 ^| ivme 100 ^| git 90 ^| konum 0 ^| salin 50 ^| dur ^| acil
echo.
python -m platformio device monitor -p COM8 -b 115200 --echo -f send_on_enter
echo.
echo Monitor kapandi. Pencereyi kapatabilirsin.
pause
