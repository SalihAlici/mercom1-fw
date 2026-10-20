# İki Teensy takılıyken doğru karta yükleme (30 Eyl): kartı SERİ NUMARASIYLA seçer, portunu 134 baud ile açıp yalnız onu
# yükleme moduna alır, sonra teensy_loader_cli ile yükler (ilk deneme "error writing" verirse tekrarlar).
# Kullanım: python teensy_yukle.py <seri_no> <firmware.hex>
#   terminal: 15425680      yer istasyonu köprüsü: 18913180
import os, subprocess, sys, time
import serial
from serial.tools import list_ports

SN, HEX = sys.argv[1], sys.argv[2]
LOADER = os.path.join(os.path.expanduser("~"), ".platformio", "packages", "tool-teensy", "teensy_loader_cli.exe")
port = next((p.device for p in list_ports.comports() if p.serial_number == SN), None)
if port is None: print("seri no %s olan Teensy bulunamadi" % SN); raise SystemExit(1)
print("hedef: %s (seri no %s)" % (port, SN))
try:
    s = serial.Serial(port, 134); time.sleep(0.2); s.close()
except Exception as e:
    print("134 baud acilamadi:", e)
time.sleep(1.5)
for deneme in range(1, 4):
    r = subprocess.run([LOADER, "-mmcu=TEENSY41", "-w", "-v", HEX], capture_output=True, text=True, timeout=30)
    out = r.stdout + r.stderr
    if "Booting" in out: print("YUKLENDI (deneme %d)" % deneme); break
    print("deneme %d: %s" % (deneme, out.strip().splitlines()[-1] if out.strip() else "?"))
    time.sleep(1)
time.sleep(4)
for p in list_ports.comports():
    if p.vid == 0x16C0: print("  %s seri no %s" % (p.device, p.serial_number))
