# BNO085 neden yeniden açılıyor? Sensörün kendi bildirdiği reset sebebini sayar.
# Kartta reset sebebi izlemeyi açar, belirtilen süre boyunca 2 saniyede bir durumu sorar, her yeni reseti sebebiyle yazar.
# Kullanım: python bno_reset_izle.py [saniye=120] [motor]
#   motor verilirse sürücüler AÇIK tutulur (motor gürültüsü etkisi); verilmezse sürücüler KAPATILIR.
import re, struct, sys, time
import mercom_link as ml
from serial.tools import list_ports

SURE = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
MOTOR = len(sys.argv) > 2
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
if link.last_telem is None: print("TELEMETRI YOK"); raise SystemExit(1)
link.send(ml.MSG_SET_MODE, bytes([1])); time.sleep(0.3)
for a in (0, 1, 2): link.send(ml.MSG_ENABLE, bytes([a, 1 if MOTOR else 0])); time.sleep(0.15)
print("%s | suruculer %s | %d s izlenecek" % (port, "ACIK" if MOTOR else "KAPALI", SURE))

PAT = re.compile(r"guc=(\d+) yazilim=(\d+) bekci=(\d+) rstpin=(\d+) diger=(\d+) cevapsiz=(\d+), son: (.*)")
AD = ("guc acilisi", "yazilim reseti", "sensorun ic bekci kopegi", "RST pini", "diger", "cevap yok")
onceki = None; ilk = None; seen = 0; kart = ""
link.send(ml.MSG_IMU_DIAG, bytes([1])); time.sleep(0.5)          # izlemeyi aç
ts = time.time()
while time.time() - ts < SURE:
    link.send(ml.MSG_IMU_DIAG); time.sleep(2.0)
    yeni = texts[seen:]; seen = len(texts)
    for tt, s in yeni:
        if s.startswith("kart:"): kart = s
        m = PAT.search(s)
        if m:
            v = tuple(int(x) for x in m.groups()[:6])
            if ilk is None: ilk = v
            if onceki is not None and v != onceki:
                for i in range(6):
                    if v[i] > onceki[i]: print("  %6.1f s: sensor yeniden acildi, sebep: %s (x%d)" % (tt - ts, AD[i].upper(), v[i] - onceki[i]))
            onceki = v
        elif s.startswith("imu: rapor akisi kesildi") or s.startswith("imu: cevre birimi") or s.startswith("imu: yeniden basladi") or s.startswith("imu: I2C"):
            print("  %6.1f s: [kart] %s" % (tt - ts, s[:100]))
        elif s.startswith("MERCOM-1 acildi"):
            print("  %6.1f s: !! KART YENIDEN BASLADI: %s" % (tt - ts, s)); link.send(ml.MSG_IMU_DIAG, bytes([1]))
if onceki is not None and ilk is not None:
    print("\n%d saniyede sensor resetleri (sebebe gore):" % SURE)
    for i in range(6):
        print("  %-26s %d" % (AD[i], onceki[i] - ilk[i]))
print(kart)
link.send(ml.MSG_IMU_DIAG, bytes([0])); time.sleep(0.2)          # izlemeyi kapat
link.close()
