# LoRa uçtan uca testi (30 Eyl): yer istasyonu köprüsü üzerinden terminale komut gönderir, cevap ve telemetriyi ölçer.
# Kullanım: python lora_test.py [yer_istasyonu_portu]   (verilmezse terminal olmayan Teensy / USB-seri dönüştürücü)
import struct, sys, time
import mercom_link as ml
from serial.tools import list_ports

TERMINAL_SERIALS = ("15425680",)
LORA_VIDS = (0x1A86, 0x10C4, 0x0403, 0x067B)
if len(sys.argv) > 1: port = sys.argv[1]
else:
    port = next((p.device for p in list_ports.comports()
                 if (p.vid == 0x16C0 and p.serial_number not in TERMINAL_SERIALS) or p.vid in LORA_VIDS), None)
if port is None: print("YER ISTASYONU BULUNAMADI"); raise SystemExit(1)
print("yer istasyonu:", port)
T0 = time.time(); texts = []; pong = []; acks = []; tele = []
link = ml.Link(port, 9600); orig = link._handle
def h(mid, p):
    t = time.time() - T0
    if mid == ml.MSG_TEXT: texts.append((t, p.decode("ascii", "replace")))
    elif mid == ml.MSG_PONG: pong.append(t)
    elif mid == ml.MSG_ACK: acks.append((t, p[0], p[1]))
    else:
        orig(mid, p)
        if mid == ml.MSG_TELEM and link.last_telem is not None: tele.append((t, dict(zip(ml.TELEM_COLS, link.last_telem))))
link._handle = h
def bekle(kosul, sn):
    t = time.time()
    while time.time() - t < sn:
        if kosul(): return time.time() - t
        time.sleep(0.02)
    return None

ts = time.time() - T0; link.send(ml.MSG_PING)
gecikme = bekle(lambda: pong, 5.0)
print("1) PING -> PONG: %s" % (("%.2f s" % gecikme) if gecikme is not None else "CEVAP YOK"))
bekle(lambda: len(texts) >= 2, 3.0)
for t, s in texts: print("   [kart, LoRa] %s" % s[:100])
n0 = len(tele); time.sleep(6.0)
print("2) telemetri: 6 s'de %d cerceve" % (len(tele) - n0))
if tele:
    d = tele[-1][1]
    print("   son: mod %s kilit %d lazer %d suruculer %d | hata %.2f | pan %.2f tilt %.2f | IMU %d" % (
        ml.MODES.get(int(d["mode"])), int(d["flags"]) & 1, (int(d["flags"]) >> 1) & 1, (int(d["flags"]) >> 4) & 1,
        d["aim_err"], d["cmd_pan"], d["cmd_tilt"], (int(d["flags"]) >> 2) & 1))
# 3) lazer kapat / aç: telemetri bayrağından doğrula
def lazer_bekle(v): return bekle(lambda: tele and ((int(tele[-1][1]["flags"]) >> 1) & 1) == v, 6.0)
link.send(ml.MSG_LASER, bytes([0])); g1 = lazer_bekle(0)
link.send(ml.MSG_LASER, bytes([1])); g2 = lazer_bekle(1)
print("3) lazer kapat: %s, lazer ac: %s" % (("%.1f s" % g1) if g1 is not None else "OLMADI", ("%.1f s" % g2) if g2 is not None else "OLMADI"))
print("   acklar:", [(hex(i), s) for _, i, s in acks[-4:]])
link.close()
