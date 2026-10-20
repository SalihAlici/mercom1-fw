@echo off
title MERCOM-1 enkoder aci okuyucu
echo Enkoder aci okuyucu: COM8, 115200. Kartta enc_aci kodu yuklu olmali. Cikis: Ctrl+C
echo Komutlar (yaz, Enter): sifir = anten acilarini sifirla     hiz 5 = saniyede 5 satir
echo.
python -m platformio device monitor -p COM8 -b 115200 --echo -f send_on_enter
echo.
echo Monitor kapandi. Pencereyi kapatabilirsin.
pause
