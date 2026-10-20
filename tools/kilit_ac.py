"""Başlığı IMU'ya göre 30°'ye alır, pan 0 / CE 2 sıfırlar, kilit alır, OTOMATİK moda geçer ve kartı öyle bırakır.
Sonra arayüz açılıp canlı izlenebilir. Kullanım: python kilit_ac.py [COM8] [tilt_derece]"""
import math, re, struct, sys, time, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mercom_link as ml

PORT = next((a for a in sys.argv[1:] if a.upper().startswith("COM")), "COM8")
TILT = next((float(a) for a in sys.argv[1:] if not a.upper().startswith("COM")), 30.0)
D2R = math.pi / 180
MOUNT = (-2.417, 5.512, 93.441)      # params.c imu_mount r p y

link = ml.Link(PORT); texts = []
orig = link._handle
def h(mid, payload):
    if mid == ml.MSG_TEXT: texts.append(payload.decode("ascii", "replace"))
    elif mid == ml.MSG_ACK: pass
    else: orig(mid, payload)
link._handle = h
time.sleep(0.5); link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 50)); time.sleep(0.3)
def tel(): return dict(zip(ml.TELEM_COLS, link.last_telem))
def snd(mid, p=b""): link.send(mid, p); time.sleep(0.2)
def wait_idle(a):
    still = 0; t0 = time.time()
    while time.time() - t0 < 25:
        time.sleep(0.02); still = still + 1 if abs(tel()["vel_" + a]) < 0.01 else 0
        if still > 25: return
def R_IH():
    r, p, y = (v * D2R for v in MOUNT)
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]
def pose():
    n0 = len(texts); link.send(ml.MSG_IMU_DIAG); time.sleep(0.6)
    q = None
    for t in texts[n0:]:
        m = re.match(r"imu quat x10000: w=(-?\d+) x=(-?\d+) y=(-?\d+) z=(-?\d+)", t)
        if m: q = [int(v) / 10000 for v in m.groups()]
    if not q: return None
    w, x, y, z = q
    RNI = [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
           [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
           [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]
    M = R_IH(); RNH = [[sum(RNI[i][k] * M[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    el = math.degrees(math.asin(max(-1, min(1, RNH[2][0])))); up = RNH[2][2]
    return el, up, math.degrees(math.acos(max(-1, min(1, up))))

for ax in (0, 1, 2): snd(ml.MSG_ENABLE, bytes([ax, 1]))
time.sleep(0.8); snd(ml.MSG_SET_MODE, bytes([1]))
for it in range(4):
    pz = pose()
    if not pz: print("IMU okunamadi"); break
    el, up, T = pz
    if el < 0: T = -T                     # anten asagi bakiyorsa mekanik tilt negatif sayilir
    print("poz: elevasyon %.1f  baslik-yukari %.2f  mekanik tilt ~%.1f" % (el, up, T), flush=True)
    if abs(T - TILT) < 2 and up > 0.5: break
    snd(ml.MSG_AXIS_ZERO, struct.pack("<Bf", 1, T)); snd(ml.MSG_GOTO, struct.pack("<Bf", 1, TILT)); wait_idle("tilt"); time.sleep(0.8)
snd(ml.MSG_AXIS_ZERO, struct.pack("<Bf", 0, 0.0)); snd(ml.MSG_AXIS_ZERO, struct.pack("<Bf", 2, 2.0))
n0 = len(texts); snd(ml.MSG_LOCK); time.sleep(0.4)
for t in texts[n0:]:
    if "kilit" in t: print("  [kart]", t)
n0 = len(texts); snd(ml.MSG_SET_MODE, bytes([2])); time.sleep(0.5)
for t in texts[n0:]: print("  [kart]", t)
d = tel(); print("mod=%s kilit=%d hata=%.2f  -> kart OTOMATIK modda birakildi" % (ml.MODES.get(d["mode"]), d["flags"] & 1, d["aim_err"]))
link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 20)); time.sleep(0.1); link.close()
