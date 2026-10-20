# En kötü durum reset testi (29 Eyl): pan, tilt ve CE sürekli salınırken (STEP pinleri sürekli anahtarlanıyor)
# BNO085 resetlerini, I2C hatalarını, IMU hızını ve enkoder okuma hızını sayar.
# Kullanım: python bno_hareket_testi.py [saniye=300] [pan_genlik=10] [tilt_genlik=5]
import re, struct, sys, time
import mercom_link as ml
from serial.tools import list_ports

SURE = float(sys.argv[1]) if len(sys.argv) > 1 else 300.0
PAN_A = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0
TILT_A = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0
port = ml.terminal_port()
if port is None: print("KART YOK"); raise SystemExit(1)
texts = []
link = ml.Link(port); orig = link._handle
T0 = time.time()
def h(mid, p):
    if mid == ml.MSG_TEXT: texts.append((time.time() - T0, p.decode("ascii", "replace")))
    elif mid in (ml.MSG_ACK, ml.MSG_PONG): pass
    else: orig(mid, p)
link._handle = h
def tel(): return dict(zip(ml.TELEM_COLS, link.last_telem))
while link.last_telem is None and time.time() - T0 < 8:
    link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 50)); time.sleep(0.5)
if link.last_telem is None: print("TELEMETRI YOK"); raise SystemExit(1)
PI = re.compile(r"i2c: bas=(\d+) gov=(\d+) buyuk=(\d+) kanal=(\d+) yaz=(\d+) pltf=(\d+) nack=(\d+) alf=(\d+) fef=(\d+)")
PR = re.compile(r"imu: \S+, rapor=(\d+), jiro=\d+, reset=(\d+)")
PE = re.compile(r"enk okuma (\d+), i2c icin atlanan (\d+)")
def durum():
    n0 = len(texts); link.send(ml.MSG_IMU_DIAG); time.sleep(0.6)
    out = {}
    for _, s in texts[n0:]:
        m = PI.search(s)
        if m: out["i2c"] = sum(int(x) for x in m.groups())
        m = PR.search(s)
        if m: out["reset"] = int(m.group(2))
        m = PE.search(s)
        if m: out["enk"] = int(m.group(1))
    return out

link.send(ml.MSG_SET_MODE, bytes([1])); time.sleep(0.3)
for a in (0, 1, 2): link.send(ml.MSG_ENABLE, bytes([a, 1])); time.sleep(0.15)
time.sleep(0.8)
d = tel(); P0, TL0, C0 = d["cmd_pan"], d["cmd_tilt"], d["cmd_ce"]
ta, tb = max(1.0, TL0 - TILT_A), min(89.0, TL0 + TILT_A)
osc = [(0, (P0 - PAN_A) % 360.0, (P0 + PAN_A) % 360.0), (1, ta, tb), (2, 0.5, 3.5)]
for ax, a, b in osc:
    link.send(ml.MSG_OSC, struct.pack("<BffH", ax, a, b, 200)); time.sleep(0.15)
print("%s | %d s | salinim: pan %.1f<->%.1f, tilt %.1f<->%.1f, CE 0.5<->3.5 (en yuksek hiz, uclarda 0,2 s)" % (port, SURE, osc[0][1], osc[0][2], ta, tb))
time.sleep(1.0)
d0 = durum(); c0 = tel()["imu_count"]; ts = time.time(); t_ilk = ts
n = bayat = 0; last = None; hiz_top = [0.0, 0.0, 0.0]; hiz_n = 0; seen = len(texts)
son_rapor = ts; onceki = dict(d0)
while time.time() - ts < SURE:
    time.sleep(0.02); t = link.last_telem
    if t is not last:
        last = t; n += 1; x = dict(zip(ml.TELEM_COLS, t))
        if not (int(x["flags"]) >> 2) & 1: bayat += 1
        for i, k in enumerate(("vel_pan", "vel_tilt", "vel_ce")): hiz_top[i] += abs(x[k])
        hiz_n += 1
    for tt, s in texts[seen:]:
        if s.startswith("imu: rapor akisi") or s.startswith("imu: yon yeniden") or s.startswith("imu: yeniden") or s.startswith("MERCOM-1 acildi"):
            print("   %6.1f s  [kart] %s" % (tt - (ts - T0), s[:100]))
    seen = len(texts)
    if time.time() - son_rapor >= 30:
        son_rapor = time.time(); dd = durum(); seen = len(texts)
        print("   %4.0f s: sensor reseti %d, i2c hatasi %d, IMU %.0f rapor/s, bayat %%%.1f, ort. hiz pan %.0f tilt %.0f CE %.0f derece/s" % (
            time.time() - ts, dd.get("reset", 0) - d0.get("reset", 0), dd.get("i2c", 0) - d0.get("i2c", 0),
            (tel()["imu_count"] - c0) / (time.time() - ts), 100.0 * bayat / max(1, n),
            hiz_top[0] / max(1, hiz_n), hiz_top[1] / max(1, hiz_n), hiz_top[2] / max(1, hiz_n)))
d1 = durum(); c1 = tel()["imu_count"]; dt = time.time() - ts
for ax, _, _ in osc: link.send(ml.MSG_OSC, bytes([ax])); time.sleep(0.1)
time.sleep(1.5)
link.send(ml.MSG_GOTO, struct.pack("<Bf", 0, P0)); link.send(ml.MSG_GOTO, struct.pack("<Bf", 1, TL0)); link.send(ml.MSG_GOTO, struct.pack("<Bf", 2, C0))
time.sleep(3.0)
print("\nSONUC (%.0f s, uc eksen surekli hareketli):" % dt)
print("  sensor reseti      : %d" % (d1.get("reset", 0) - d0.get("reset", 0)))
print("  I2C hatasi         : %d" % (d1.get("i2c", 0) - d0.get("i2c", 0)))
print("  IMU                : %.0f rapor/s, bayat ornek %%%.1f" % ((c1 - c0) / dt, 100.0 * bayat / max(1, n)))
print("  enkoder okuma      : eksen basina %.0f/s" % ((d1.get("enk", 0) - d0.get("enk", 0)) / dt / 3))
print("  ortalama hiz       : pan %.0f, tilt %.0f, CE %.0f derece/s" % (hiz_top[0] / max(1, hiz_n), hiz_top[1] / max(1, hiz_n), hiz_top[2] / max(1, hiz_n)))
d = tel(); print("  eksenler baslangica dondu: pan %.2f tilt %.2f CE %.2f" % (d["cmd_pan"], d["cmd_tilt"], d["cmd_ce"]))
link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 20)); time.sleep(0.1); link.close()
