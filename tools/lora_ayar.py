# İki E22 modülünün ayarlarını okur ve çözer (30 Eyl): terminal (USB üzerinden MSG_LORA_CFG) ve yer istasyonu köprüsü
# (portu 9601 baud ile açınca köprü E22'yi ayar moduna alır). İsteğe bağlı: ikisini de aynı ayara yazar.
# Kullanım: python lora_ayar.py            -> oku
#           python lora_ayar.py yaz         -> MERGEN ayarlarını yaz (sabit noktalı mod, diğer takımlarla çakışmasın):
#               terminal 0x4D01, yer istasyonu 0x4D02, NETID 0x4D, 9600 8N1, hava 9,6 kbps, 240 B, 22 dBm,
#               kanal 15 = 865,125 MHz (863–870 MHz lisanssız bant), şifre anahtarı (CRYPT, okunamaz), sonra yeniden oku
import struct, sys, time
import serial
import mercom_link as ml
from serial.tools import list_ports

TERMINAL_SN, YER_SN = "15425680", "18913180"
# MERGEN ayarı (30 Eyl): ADDH ADDL NETID REG0 REG1 REG2(kanal) REG3(0x40 sabit nokta | WOR 2000 ms) CRYPT_H CRYPT_L
CRYPT_H, CRYPT_L = 0x00, 0x00   # şifre anahtarı: kendi anahtarınızı yazın (iki modülde aynı olmalı; 0x0000 = şifresiz)
AYAR_TERMINAL = bytes([0x4D, 0x01, 0x4D, 0x64, 0x00, 0x0F, 0x53, CRYPT_H, CRYPT_L])   # REG3 0x53: sabit nokta + LBT
AYAR_YER      = bytes([0x4D, 0x02, 0x4D, 0x64, 0x00, 0x0F, 0x53, CRYPT_H, CRYPT_L])
UART = ["1200", "2400", "4800", "9600", "19200", "38400", "57600", "115200"]
HAVA = ["0,3k", "1,2k", "2,4k", "4,8k", "9,6k", "19,2k", "38,4k", "62,5k"]

def coz(b):
    if len(b) < 12 or b[0] != 0xC1: return "gecersiz cevap: " + " ".join("%02X" % x for x in b)
    r = b[3:12]
    return ("adres %02X%02X, NETID %02X, UART %s baud, parite %d, hava hizi %s bps, paket %d B, guc %d dBm, kanal %d (%.3f MHz), %s, LBT %d" % (
        r[0], r[1], r[2], UART[r[3] >> 5], (r[3] >> 3) & 3, HAVA[r[3] & 7], [240, 128, 64, 32][r[4] >> 6], [22, 17, 13, 10][r[4] & 3],
        r[5], 850.125 + r[5], "SABIT NOKTA" if r[6] & 0x40 else "seffaf", (r[6] >> 4) & 1)) + "   [" + " ".join("%02X" % x for x in r) + "]"

def port_of(sn): return next((p.device for p in list_ports.comports() if p.serial_number == sn), None)

def terminal(yaz=None):
    texts = []
    link = ml.Link(port_of(TERMINAL_SN)); orig = link._handle
    def h(mid, p):
        if mid == ml.MSG_TEXT: texts.append(p.decode("ascii", "replace"))
        elif mid in (ml.MSG_ACK, ml.MSG_PONG): pass
        else: orig(mid, p)
    link._handle = h
    time.sleep(0.3)
    link.send(ml.MSG_LORA_CFG, yaz if yaz else b""); time.sleep(1.2)
    link.close()
    for s in texts:
        if s.startswith("lora ayar"):
            hx = s.split(":", 1)[1].split("CEVAP")[0].split()
            return bytes(int(x, 16) for x in hx) if hx else b""
    return b""

def yer(yaz=None):
    s = serial.Serial(port_of(YER_SN), 9601, timeout=0.1)      # köprü: 9601 = E22 ayar modu
    time.sleep(0.4); s.reset_input_buffer()
    if yaz:
        s.write(bytes([0xC0, 0x00, 0x09]) + yaz); time.sleep(0.5); s.read(64); time.sleep(0.1)
    s.write(bytes([0xC1, 0x00, 0x09])); time.sleep(0.5)
    b = s.read(64)
    s.baudrate = 115200; time.sleep(0.3)                        # köprü şeffaf moda döner
    s.close()
    return b

if len(sys.argv) > 1 and sys.argv[1] == "yaz":
    print("MERGEN ayarlari yaziliyor (sabit nokta, 0x4D01 / 0x4D02, kanal 15)...")
    terminal(AYAR_TERMINAL); yer(AYAR_YER); time.sleep(0.5)
print("TERMINAL E22     :", coz(terminal()))
print("YER ISTASYONU E22:", coz(yer()))
