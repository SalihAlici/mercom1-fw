@echo off
rem Nucleo'nun 3V3 hattini karttaki ST-Link uzerinden saniyede bir olcer. Normal deger 3.2 - 3.3 V.
rem Kod yuklemeden once bu pencereyi kapat (ST-Link'i mesgul eder).
title Nucleo 3V3 voltaji - normal 3.2 ile 3.3 arasi - kapatmak icin pencereyi kapat
set OO=%USERPROFILE%\.platformio\packages\tool-openocd
echo Nucleo 3V3 hatti olculuyor. Normal deger 3.2 - 3.3 V. Telleri tek tek cikar, deger duzelince suclu o teldir.
echo.
:dongu
"%OO%\bin\openocd.exe" -s "%OO%\openocd\scripts" -f interface/stlink.cfg -c "transport select dapdirect_swd" -f target/stm32h7x.cfg -c "init" -c "shutdown" 2>&1 | findstr /C:"Target voltage"
timeout /t 1 /nobreak 1>NUL 2>NUL
goto dongu
