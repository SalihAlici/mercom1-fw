# Kartin seri portunu (ST-Link) bulur ve gelen satirlari canli yazar. Kullanim: python seri_izle.py [COM8]
# Isleme sinamasi (vdd_test), blink, lazer_test gibi araclarin ciktisini izlemek icin. Kapatmak icin pencereyi kapat ya da Ctrl+C.
import sys, time
import serial
from serial.tools import list_ports

port = sys.argv[1] if len(sys.argv) > 1 else next((p.device for p in list_ports.comports() if p.vid in (0x0483, 0x16C0)), None)
if not port:
    print("ST-Link seri portu bulunamadi; kart USB'de mi?"); time.sleep(5); raise SystemExit
print("port:", port, " (kapatmak icin Ctrl+C)\n")
while True:
    try:
        with serial.Serial(port, 115200, timeout=0.5) as s:
            while True:
                line = s.readline()
                if line:
                    print(time.strftime("%H:%M:%S"), line.decode("ascii", "replace").rstrip())
    except serial.SerialException:
        print(time.strftime("%H:%M:%S"), "port koptu, yeniden deneniyor..."); time.sleep(1)
    except KeyboardInterrupt:
        break
