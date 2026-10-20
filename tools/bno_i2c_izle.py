# BNO085 resetinden önce I2C hattında ne oluyor? Saniyede bir kartın I2C tanı sayaçlarını ve sensör reset sayısını okur,
# değişen her şeyi zamanıyla yazar. Kullanım: python bno_i2c_izle.py [saniye=120]
import re, struct, sys, time
import mercom_link as ml
from serial.tools import list_ports

SURE = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
AD = ("baslik okunamadi", "govde okunamadi", "buyuk paket", "gecersiz kanal", "yazma hatasi",
      "SAAT UZATMA ZAMAN ASIMI (pltf)", "NACK", "hat kaybi", "FIFO hatasi", "paket")
port = ml.terminal_port()
if port is None: print("KART YOK"); raise SystemExit(1)
texts = []
link = ml.Link(port); orig = link._handle
def h(mid, p):
    if mid == ml.MSG_TEXT: texts.append((time.time(), p.decode("ascii", "replace")))
    elif mid in (ml.MSG_ACK, ml.MSG_PONG): pass
    else: orig(mid, p)
link._handle = h
t0 = time.time()
while link.last_telem is None and time.time() - t0 < 8:
    link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 20)); time.sleep(0.5)
link.send(ml.MSG_SET_MODE, bytes([1])); time.sleep(0.3)
for a in (0, 1, 2): link.send(ml.MSG_ENABLE, bytes([a, 1])); time.sleep(0.15)
print("%s | suruculer acik, %d s izleniyor" % (port, SURE))
PI = re.compile(r"i2c: bas=(\d+) gov=(\d+) buyuk=(\d+) kanal=(\d+) yaz=(\d+) pltf=(\d+) nack=(\d+) alf=(\d+) fef=(\d+) paket=(\d+)")
PR = re.compile(r"reset=(\d+)")
ilk = onceki = None; r_ilk = r_once = None; seen = 0
ts = time.time()
while time.time() - ts < SURE:
    link.send(ml.MSG_IMU_DIAG); time.sleep(1.0)
    yeni = texts[seen:]; seen = len(texts)
    for tt, s in yeni:
        m = PI.search(s)
        if m:
            v = [int(x) for x in m.groups()]
            if ilk is None: ilk = v
            if onceki is not None:
                d = [v[i] - onceki[i] for i in range(9)]
                if any(d): print("  %6.1f s: I2C  " % (tt - ts) + ", ".join("%s +%d" % (AD[i], d[i]) for i in range(9) if d[i]))
            onceki = v
        m = PR.search(s)
        if m and "imu:" in s:
            r = int(m.group(1))
            if r_ilk is None: r_ilk = r
            if r_once is not None and r != r_once: print("  %6.1f s: SENSOR RESETI (x%d)" % (tt - ts, r - r_once))
            r_once = r
        if s.startswith("imu: rapor akisi kesildi") or s.startswith("imu: yeniden") or s.startswith("imu: cevre"):
            print("  %6.1f s: [kart] %s" % (tt - ts, s[:90]))
if ilk and onceki:
    print("\ntoplam %d s:" % SURE)
    for i in range(10): print("  %-32s %d" % (AD[i], onceki[i] - ilk[i]))
    print("  %-32s %d" % ("SENSOR RESETI", (r_once or 0) - (r_ilk or 0)))
link.close()
