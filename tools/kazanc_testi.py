# Takip kazancı testi (30 Eyl): kp_pan = kp_tilt = K için
#   duruş: 15 s otomatik, hareketli örnek oranı (titreme) ve hata
#   bozma: başlık manuelde (+5,+3) ve (-5,-3) uzaklaştırılır, otomatiğe geçilir; Ø20/Ø10'a giriş süresi, hedefi aşma
# Hata IMU'dan (aim_err); 4,47 m'de Ø10 sınırı 0,64°, Ø20 1,28°.
# Kullanım: python kazanc_testi.py [K1 K2 ...]   (varsayılan 8 12 16; sonunda kp 8'e döner)
import math, struct, sys, time
import mercom_link as ml
from serial.tools import list_ports

KLIST = [float(a) for a in sys.argv[1:]] or [8.0, 12.0, 16.0]
O10, O20 = 0.64, 1.28
port = ml.terminal_port()
if port is None: print("KART YOK"); raise SystemExit(1)
link = ml.Link(port); orig = link._handle
rows = []
def h(mid, p):
    if mid in (ml.MSG_TEXT, ml.MSG_ACK, ml.MSG_PONG): return
    orig(mid, p)
    if mid == ml.MSG_TELEM and link.last_telem is not None: rows.append((time.time(), dict(zip(ml.TELEM_COLS, link.last_telem))))
link._handle = h
def tel(): return dict(zip(ml.TELEM_COLS, link.last_telem))
def setp(n, v): link.send(ml.MSG_PARAM_SET, struct.pack("<Hf", ml.PARAM_NAMES.index(n), float(v))); time.sleep(0.1)
def mode(m):
    for _ in range(6):
        link.send(ml.MSG_SET_MODE, bytes([m])); time.sleep(0.25)
        if int(tel()["mode"]) == m: return True
        time.sleep(0.4)
    return False
def bekle_dur(sn=8.0):
    t = time.time(); time.sleep(0.3)
    while time.time() - t < sn:
        d = tel()
        if abs(d["vel_pan"]) < 0.05 and abs(d["vel_tilt"]) < 0.05: return
        time.sleep(0.05)
def kayit(sn):
    t0 = time.time(); n0 = len(rows); time.sleep(sn)
    return [(t - t0, d) for t, d in rows[n0:]]

t0 = time.time()
while link.last_telem is None and time.time() - t0 < 8:
    link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 50)); time.sleep(0.5)
link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 50))
mode(1)
link.send(ml.MSG_ENABLE, bytes([1])); time.sleep(0.3); link.send(ml.MSG_LASER, bytes([1])); time.sleep(0.8)
d = tel()
if d["cmd_tilt"] < 15:
    link.send(ml.MSG_GOTO, struct.pack("<Bf", 1, 25.0)); bekle_dur(10)
time.sleep(1.0)
link.send(ml.MSG_LOCK); time.sleep(0.8); mode(2); time.sleep(3.0)
d = tel(); PAN0, TILT0 = d["cmd_pan"], d["cmd_tilt"]
print("hedef: pan %.2f tilt %.2f, hata %.2f" % (PAN0, TILT0, d["aim_err"]))
print("  kp | duruş: hareket %%  hata ort/en çok | bozma: Ø20'ye  Ø10'a   aşım pan/tilt   4 s sonra")
ozet = []
for K in KLIST:
    setp("kp_pan", K); setp("kp_tilt", K)
    mode(1); link.send(ml.MSG_GOTO, struct.pack("<Bf", 0, PAN0)); link.send(ml.MSG_GOTO, struct.pack("<Bf", 1, TILT0)); bekle_dur()
    mode(2); time.sleep(2.0)
    r = kayit(15.0)
    hareket = 100.0 * sum(1 for _, x in r if abs(x["vel_pan"]) > 0.05 or abs(x["vel_tilt"]) > 0.05) / max(1, len(r))
    e = [x["aim_err"] for _, x in r]
    satir = "  %2.0f | %13.0f   %.2f / %.2f      |" % (K, hareket, sum(e) / max(1, len(e)), max(e) if e else 0)
    adimlar = []
    for dp, dt in ((5, 3), (-5, -3)):
        mode(1)
        link.send(ml.MSG_GOTO, struct.pack("<Bf", 0, (PAN0 + dp) % 360)); link.send(ml.MSG_GOTO, struct.pack("<Bf", 1, TILT0 + dt)); bekle_dur(); time.sleep(0.4)
        s0 = tel(); side0, up0 = s0["aim_side"], s0["aim_up"]
        mode(2)
        r = kayit(4.0)
        t20 = next((t for t, x in r if x["aim_err"] < O20), None)
        t10 = None
        for i, (t, x) in enumerate(r):
            if x["aim_err"] < O10 and all(y["aim_err"] < O10 for tt, y in r[i:] if tt - t < 0.3): t10 = t; break
        # aşım: hatanın başlangıçtakinin ters yönüne geçtiği en büyük değer
        asp = max([0.0] + [-x["aim_side"] * math.copysign(1, side0) for _, x in r])
        ast = max([0.0] + [-x["aim_up"] * math.copysign(1, up0) for _, x in r])
        son = r[-1][1]["aim_err"] if r else float("nan")
        adimlar.append((t20, t10, asp, ast, son))
    f = lambda v: ("%.2f s" % v) if v is not None else " yok "
    for t20, t10, asp, ast, son in adimlar:
        print(satir + " %-7s %-7s %.2f / %.2f      %.2f" % (f(t20), f(t10), asp, ast, son)); satir = "     |" + " " * 34 + "|"
    ozet.append((K, hareket, adimlar))
setp("kp_pan", 8); setp("kp_tilt", 8)
mode(1); link.send(ml.MSG_GOTO, struct.pack("<Bf", 0, PAN0)); link.send(ml.MSG_GOTO, struct.pack("<Bf", 1, TILT0)); bekle_dur()
print("bitti: kp 8'e dondu, manuel mod, baslik hedef pozunda")
link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 20)); time.sleep(0.1); link.close()
