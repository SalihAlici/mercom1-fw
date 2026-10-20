# Yön bağlama testi (29 Eyl): IMU yeniden başladığında hedef kayıyor mu?
# Kilit alınır, başlık manuelde hedeften uzaklaştırılır, IMU bilerek yeniden başlatılır (yaw referansı sıfırlanır),
# otomatiğe geçilir. Başlık hedefe dönmeli. Gerçek hata enkoderden ölçülür (taban yerinde durmalı, dokunulmaz).
# Kullanım: python baglama_testi.py <yaw_anchor 0|1>
import math, struct, sys, time
import mercom_link as ml
from serial.tools import list_ports

# Kullanım: python baglama_testi.py <yaw_anchor 0|1> [pan tilt] [pan tilt] ...   (varsayılan bozmalar: +5/+3, -5/-3, +8/0)
ANCHOR = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
_b = [float(a) for a in sys.argv[2:]]
BOZMALAR = list(zip(_b[0::2], _b[1::2])) if len(_b) >= 2 else [(+5, +3), (-5, -3), (+8, 0)]
ARA_S = 6.0                                         # bozmalar arası bekleme: sensörü üst üste yeniden başlatmak onu kilitledi
port = ml.terminal_port()
if port is None: print("KART YOK"); raise SystemExit(1)
texts = []
link = ml.Link(port); orig = link._handle
def h(mid, p):
    if mid == ml.MSG_TEXT: texts.append(p.decode("ascii", "replace"))
    elif mid in (ml.MSG_ACK, ml.MSG_PONG): pass
    else: orig(mid, p)
link._handle = h
def tel(): return dict(zip(ml.TELEM_COLS, link.last_telem))
def imu_ok(d): return (int(d["flags"]) >> 2) & 1
def bekle_imu(sn=10.0):
    t = time.time(); iyi = 0
    while time.time() - t < sn and iyi < 15:
        iyi = iyi + 1 if imu_ok(tel()) else 0; time.sleep(0.04)
    return iyi >= 15
def setp(n, v): link.send(ml.MSG_PARAM_SET, struct.pack("<Hf", ml.PARAM_NAMES.index(n), float(v))); time.sleep(0.1)
def bekle_dur(sn=8.0):
    t = time.time(); time.sleep(0.3)
    while time.time() - t < sn:
        d = tel()
        if abs(d["vel_pan"]) < 0.05 and abs(d["vel_tilt"]) < 0.05: return True
        time.sleep(0.05)
    return False
def gercek(d):
    dp = ((d["enc_pan"] - ENC0[0] + 180) % 360) - 180; dt = d["enc_tilt"] - ENC0[1]
    return math.hypot(dp * math.cos(math.radians(TILT0)), dt), dp, dt

t0 = time.time()
while link.last_telem is None and time.time() - t0 < 8:
    link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 50)); time.sleep(0.5)
if link.last_telem is None: print("TELEMETRI YOK"); raise SystemExit(1)
if not bekle_imu(): print("IMU yok"); raise SystemExit(1)
link.send(ml.MSG_SET_MODE, bytes([1])); time.sleep(0.4)
for a in (0, 1, 2): link.send(ml.MSG_ENABLE, bytes([a, 1])); time.sleep(0.15)
link.send(ml.MSG_LASER, bytes([1])); setp("yaw_anchor", ANCHOR)
link.send(ml.MSG_PARAM_GET, struct.pack("<H", ml.PARAM_NAMES.index("yaw_anchor"))); time.sleep(0.3)
print("yon baglama (yaw_anchor) = %s" % link.params.get(ml.PARAM_NAMES.index("yaw_anchor")))
time.sleep(1.2)                                     # sürücüler açılınca motor otursun
link.send(ml.MSG_LOCK); time.sleep(0.8)
link.send(ml.MSG_SET_MODE, bytes([2])); time.sleep(2.5)
d = tel(); PAN0, TILT0 = d["cmd_pan"], d["cmd_tilt"]; ENC0 = (d["enc_pan"], d["enc_tilt"])
print("hedef: pan %.2f tilt %.2f (enkoder %.2f / %.2f), IMU hatasi %.2f" % (PAN0, TILT0, ENC0[0], ENC0[1], d["aim_err"]))
print("  bozma      | IMU yeniden basladi | yaw duzeltmesi       | 4 s sonra: IMU hatasi / GERCEK hata (pan, tilt)")
sonuc = []
for dp, dt in BOZMALAR:
    link.send(ml.MSG_SET_MODE, bytes([1])); time.sleep(0.4)
    link.send(ml.MSG_GOTO, struct.pack("<Bf", 0, (PAN0 + dp) % 360.0)); link.send(ml.MSG_GOTO, struct.pack("<Bf", 1, TILT0 + dt))
    bekle_dur(); time.sleep(0.5)
    n0 = len(texts)
    link.send(ml.MSG_IMU_INIT)                       # sensörü bilerek yeniden başlat: yaw referansı sıfırlanır
    time.sleep(1.5); ok = bekle_imu(8.0); time.sleep(0.5)
    for _ in range(5):
        link.send(ml.MSG_SET_MODE, bytes([2])); time.sleep(0.2)
        if int(tel()["mode"]) == 2: break
        time.sleep(0.5)
    time.sleep(4.0)
    d = tel(); g, gp, gt = gercek(d)
    dz = [t for t in texts[n0:] if "yon yeniden baglandi" in t]
    duz = dz[-1].split("duzeltmesi ")[1].split(" (")[0] if dz else "yok"
    print("  %+3d / %+3d  | %-19s | %-20s | %.2f / %.2f  (%+.2f, %+.2f)" % (dp, dt, "evet" if ok else "HAYIR", duz, d["aim_err"], g, gp, gt))
    sonuc.append(g)
    time.sleep(ARA_S)
link.send(ml.MSG_SET_MODE, bytes([1])); time.sleep(0.3)
link.send(ml.MSG_GOTO, struct.pack("<Bf", 0, PAN0)); link.send(ml.MSG_GOTO, struct.pack("<Bf", 1, TILT0)); bekle_dur()
print("baslik hedef pozuna geri alindi (manuel). En buyuk gercek hata %.2f derece (O10 esigi 0,64)" % max(sonuc))
link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 20)); time.sleep(0.1); link.close()
