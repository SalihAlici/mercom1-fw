# Başlığı IMU'ya göre normal poza getir (tilt 30, pan 0, CE 2), sonra kilit + otomatik izleme
import math, re, struct, sys, time
sys.path.insert(0, r"C:\Users\Salih\Desktop\hareketli uydu\mercom1-fw\tools")
import mercom_link as ml

# Çıktı hem ekrana hem dosyaya (kilit_testi_son.log): pencere kapansa da sonuç okunabilsin
import builtins, os
_log = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "kilit_testi_son.log"), "w", encoding="utf-8")
_print = builtins.print
def print(*a, **k):
    _print(*a, **k)
    try:
        _log.write(" ".join(str(x) for x in a) + "\n"); _log.flush()
    except Exception:
        pass

D2R = math.pi / 180
MOUNT = (-2.417, 5.512, 93.441)      # params.c imu_mount r p y
link = ml.Link("COM8"); texts = []
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
def quat():
    n0 = len(texts); link.send(ml.MSG_IMU_DIAG); time.sleep(0.6)
    for t in texts[n0:]:
        m = re.match(r"imu quat x10000: w=(-?\d+) x=(-?\d+) y=(-?\d+) z=(-?\d+)", t)
        if m: return [int(v) / 10000 for v in m.groups()]
    return None
def pose():
    q = quat()
    if not q: return None
    w, x, y, z = q
    RNI = [[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
           [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
           [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]]
    M = R_IH()
    RNH = [[sum(RNI[i][k] * M[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    el = math.degrees(math.asin(max(-1, min(1, RNH[2][0])))); headUp = RNH[2][2]
    T = math.degrees(math.acos(max(-1, min(1, headUp))))          # mekanik tilt (taban düzse): 0 yatay, 90 zenit, >90 devrilmiş
    return el, headUp, T

for ax in (0, 1, 2): snd(ml.MSG_ENABLE, bytes([ax, 1]))
time.sleep(0.8); snd(ml.MSG_SET_MODE, bytes([1]))
for it in range(4):
    pz = pose()
    if not pz: print("IMU okunamadi"); break
    el, up, T = pz
    print("poz: elevasyon %.1f  baslik-yukari %.2f  mekanik tilt ~%.1f" % (el, up, T), flush=True)
    if abs(T - 30) < 2 and up > 0.5: break
    snd(ml.MSG_AXIS_ZERO, struct.pack("<Bf", 1, T))          # kartın tilt'i gerçek mekanik açı olsun
    snd(ml.MSG_GOTO, struct.pack("<Bf", 1, 30.0)); wait_idle("tilt"); time.sleep(0.8)
snd(ml.MSG_AXIS_ZERO, struct.pack("<Bf", 0, 0.0)); snd(ml.MSG_AXIS_ZERO, struct.pack("<Bf", 2, 2.0))
d = tel(); print("ayar bitti: pan 0, tilt %.1f, ce 2  | IMU roll %.1f pitch %.1f" % (d["cmd_tilt"], d["roll"], d["pitch"]))

BEKLE = int(sys.argv[1]) if len(sys.argv) > 1 else 0
for kalan in range(BEKLE, 0, -5):
    print("otomatik moda %d s kaldi..." % kalan, flush=True); time.sleep(5)
n0 = len(texts); snd(ml.MSG_LOCK); time.sleep(0.4)
for t in texts[n0:]:
    if "kilit" in t: print("  [kart]", t)
n0 = len(texts); snd(ml.MSG_SET_MODE, bytes([2])); time.sleep(0.4); d = tel()
for t in texts[n0:]: print("  [kart]", t)
if ml.MODES.get(d["mode"]) != "AUTO":
    print("otomatik moda GIRILEMEDI");
else:
    SURE = 60
    print("OTOMATIK MOD ACIK: tabani yavas ve kucuk eg (+-8, 10 s periyot). %d s" % SURE, flush=True)
    print("   t   tilt_kmt  pan_kmt | hata  yanal dikey | mod", flush=True)
    t0 = time.time(); emax = 0; n = 0; esum = 0
    while time.time() - t0 < SURE:
        time.sleep(1.0); d = tel(); n += 1; esum += d["aim_err"]; emax = max(emax, d["aim_err"])
        print("  %4.0f  %7.2f  %7.2f | %5.2f %+6.2f %+6.2f | %s" % (time.time() - t0, d["cmd_tilt"], d["cmd_pan"], d["aim_err"], d["aim_side"], d["aim_up"], ml.MODES.get(d["mode"])), flush=True)
        if ml.MODES.get(d["mode"]) != "AUTO": break
    print("ozet: hata ort %.2f  en fazla %.2f" % (esum / max(n, 1), emax))
    for t in texts:
        if "nisan" in t or "takildi" in t: print("  [kart]", t)
    snd(ml.MSG_SET_MODE, bytes([1]))
snd(ml.MSG_UNLOCK)
link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 20)); time.sleep(0.1); link.close()
print("bitti; suruculer acik")
