# Takip testi kurulumu. Kullanım: takip_kur.py <plx_en 0|1> [plx_ly] [plx_ht] [plx_d]   (verilmeyen parametreye dokunulmaz)
# IMU'yu bekler, sürücüleri ve lazeri açar, paralaks ayarını yazar, kilit alır, otomatiğe geçer.
import struct, sys, time
sys.path.insert(0, r"C:\Users\Salih\Desktop\hareketli uydu\mercom1-fw\tools")
import mercom_link as ml
from serial.tools import list_ports

EN = float(sys.argv[1]); LY = float(sys.argv[2]) if len(sys.argv) > 2 else -0.13
HT = float(sys.argv[3]) if len(sys.argv) > 3 else None; DD = float(sys.argv[4]) if len(sys.argv) > 4 else None
port = ml.terminal_port()
texts = []
link = ml.Link(port); orig = link._handle
def h(mid, p):
    if mid == ml.MSG_TEXT: texts.append(p.decode("ascii", "replace"))
    elif mid in (ml.MSG_ACK, ml.MSG_PONG): pass
    else: orig(mid, p)
link._handle = h
def setp(n, v): link.send(ml.MSG_PARAM_SET, struct.pack("<Hf", ml.PARAM_NAMES.index(n), float(v))); time.sleep(0.1)
def tel(): return dict(zip(ml.TELEM_COLS, link.last_telem))
t0 = time.time()
while link.last_telem is None and time.time() - t0 < 8:
    link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 50)); time.sleep(0.5)
if link.last_telem is None: print("TELEMETRI YOK"); raise SystemExit(1)
t1 = time.time(); iyi = 0
while time.time() - t1 < 15 and iyi < 25:
    iyi = iyi + 1 if (tel()["flags"] >> 2) & 1 else 0; time.sleep(0.04)
c0 = tel()["imu_count"]; time.sleep(2.0); hz = (tel()["imu_count"] - c0) / 2.0
print("IMU: %s, %.0f rapor/s, kart acik kalma %.0f s" % ("VAR" if iyi >= 25 else "YOK", hz, tel()["t_ms"] / 1000))
if iyi < 25: link.close(); print("HAZIR DEGIL"); raise SystemExit(1)
link.send(ml.MSG_SET_MODE, bytes([1])); time.sleep(0.3)
for a in (0, 1, 2): link.send(ml.MSG_ENABLE, bytes([a, 1])); time.sleep(0.15)
link.send(ml.MSG_LASER, bytes([1])); setp("plx_ly", LY)
if HT is not None: setp("plx_ht", HT)
if DD is not None: setp("plx_d", DD)
setp("plx_en", EN); time.sleep(0.8)
for n in ("imu_mount_p", "plx_en", "plx_ht", "plx_ly", "plx_d"): link.send(ml.MSG_PARAM_GET, struct.pack("<H", ml.PARAM_NAMES.index(n))); time.sleep(0.1)
time.sleep(0.3); print("kartta:", {n: round(link.params.get(ml.PARAM_NAMES.index(n), float("nan")), 3) for n in ("imu_mount_p", "plx_en", "plx_ht", "plx_ly", "plx_d")})
ok = False
for deneme in range(5):
    link.send(ml.MSG_LOCK); time.sleep(0.8)
    link.send(ml.MSG_SET_MODE, bytes([2])); time.sleep(1.0)
    d = tel()
    if d["mode"] == 2 and d["flags"] & 1: ok = True; break
    time.sleep(1.0)
d = tel()
print("paralaks=%d ly=%+.2f | mod=%s kilit=%d lazer=%d hata %.3f | pan %.2f tilt %.2f" % (EN, LY, ml.MODES.get(d["mode"]), d["flags"] & 1, (d["flags"] >> 1) & 1, d["aim_err"], d["cmd_pan"], d["cmd_tilt"]))
for t in texts[-3:]: print("  [kart]", t[:110])
link.close(); print("HAZIR" if ok else "HAZIR DEGIL")
raise SystemExit(0 if ok else 1)
