# Bozma (toparlanma) testi: motorları elle zorlamak yerine yazılımla yapılır.
# Kilit alınır, başlık hedeften bilerek uzaklaştırılır (manuel modda pan/tilt birkaç derece), sonra otomatik moda
# geçilir ve lekenin hedefe kaç saniyede döndüğü ölçülür. Taban yerinde durur, dokunulmaz.
# Kullanım: python bozma_testi.py [yenikilit]
#   yenikilit verilirse test başında şimdiki bakış hedef alınır; verilmezse karttaki kilit kullanılır.
import csv, math, os, struct, sys, time
import mercom_link as ml
from serial.tools import list_ports

BOZMALAR = [(+5, +5), (-5, -5), (+10, 0), (0, +8), (-10, -8), (+8, +8)]      # pan, tilt (derece)
ESIK10 = math.degrees(math.atan(0.05 / 4.47))                                  # Ø10 cm çember, 4,47 m: 0,64°
ESIK20 = math.degrees(math.atan(0.10 / 4.47))                                  # Ø20 cm çember: 1,28°

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
# Taban bu testte yerinden oynamadığı için enkoder açıları lekenin GERÇEK yerini verir (IMU referansından bağımsız)
def gercek_hata(d):
    dp = ((d["enc_pan"] - ENC0[0] + 180) % 360) - 180; dt = d["enc_tilt"] - ENC0[1]
    return math.hypot(dp * math.cos(math.radians(tilt0)), dt)
KAYIT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "bozma_kayit.csv")
_kf = open(KAYIT, "w", newline=""); _kw = csv.writer(_kf); _kw.writerow(["t", "olay"] + list(ml.TELEM_COLS)); _kt0 = time.time(); _klast = [None]
def kaydet(olay=""):
    t = link.last_telem
    if t is None or (t is _klast[0] and not olay): return
    _klast[0] = t; _kw.writerow(["%.3f" % (time.time() - _kt0), olay] + list(t))
def uyu(sn, olay=""):
    kaydet(olay); t = time.time()
    while time.time() - t < sn: time.sleep(0.01); kaydet()
def imu_ok(d): return (int(d["flags"]) >> 2) & 1
def bekle_imu(sn=10.0):
    t = time.time(); iyi = 0
    while time.time() - t < sn and iyi < 15:
        iyi = iyi + 1 if imu_ok(tel()) else 0; time.sleep(0.04)
    return iyi >= 15
def git(axis, deg):
    link.send(ml.MSG_GOTO, struct.pack("<Bf", axis, deg))
def bekle_dur(hp, ht, sn=8.0):
    t = time.time()
    while time.time() - t < sn:
        d = tel()
        dp = ((d["cmd_pan"] - hp + 180) % 360) - 180
        if abs(dp) < 0.15 and abs(d["cmd_tilt"] - ht) < 0.15 and abs(d["vel_pan"]) < 0.05 and abs(d["vel_tilt"]) < 0.05: return True
        time.sleep(0.03)
    return False

t0 = time.time()
while link.last_telem is None and time.time() - t0 < 8:
    link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 50)); time.sleep(0.5)
if link.last_telem is None: print("TELEMETRI YOK"); raise SystemExit(1)
if not bekle_imu(): print("IMU verisi yok"); link.close(); raise SystemExit(1)

link.send(ml.MSG_SET_MODE, bytes([1])); time.sleep(0.4)
for a in (0, 1, 2): link.send(ml.MSG_ENABLE, bytes([a, 1])); time.sleep(0.15)
link.send(ml.MSG_LASER, bytes([1])); time.sleep(0.3)
d = tel()
if len(sys.argv) > 1 or not (int(d["flags"]) & 1):
    link.send(ml.MSG_LOCK); time.sleep(0.8)
    print("yeni kilit alindi")
d = tel()
if not (int(d["flags"]) & 1): print("kilit alinamadi"); link.close(); raise SystemExit(1)
# hedefe bakan eklem açıları: önce otomatikte oturt, sonra oku
link.send(ml.MSG_SET_MODE, bytes([2])); time.sleep(2.0)
d = tel(); pan0, tilt0 = d["cmd_pan"], d["cmd_tilt"]; ENC0 = (d["enc_pan"], d["enc_tilt"])
print("hedefte: pan %.2f tilt %.2f, hata %.2f derece | esikler: O10 %.2f, O20 %.2f derece" % (pan0, tilt0, d["aim_err"], ESIK10, ESIK20))
print("  bozma (pan/tilt) | baslangic: IMU / gercek | O20 icine | O10 icine | 3 s sonra: IMU / gercek | IMU kesintisi")

sonuc = []
for dp, dt in BOZMALAR:
    link.send(ml.MSG_SET_MODE, bytes([1])); time.sleep(0.4)
    hp, ht = (pan0 + dp) % 360.0, tilt0 + dt
    git(0, hp); git(1, ht)
    if not bekle_dur(hp, ht): print("  %+3d / %+3d: eksen hedefe varmadi, atlandi" % (dp, dt)); continue
    uyu(1.0, "bozuldu %+d/%+d" % (dp, dt))
    if not bekle_imu(8.0): print("  %+3d / %+3d: IMU yok, atlandi" % (dp, dt)); continue
    e0 = tel()["aim_err"]; g0 = gercek_hata(tel()); kaydet("otomatige geciliyor")
    girdi = False
    for _ in range(5):
        link.send(ml.MSG_SET_MODE, bytes([2])); time.sleep(0.15)
        if int(tel()["mode"]) == 2: girdi = True; break
        time.sleep(0.5)
    if not girdi: print("  %+3d / %+3d: otomatige gecilemedi" % (dp, dt)); continue
    ts = time.time(); t20 = t10 = None; son = None; sonimu = float("nan"); kesinti = 0; last = None; ic10 = None
    while time.time() - ts < 8.0:
        time.sleep(0.01); t = link.last_telem
        if t is last: continue
        last = t; d = tel(); el = time.time() - ts; kaydet()
        if not imu_ok(d): kesinti += 1
        e = gercek_hata(d)                                         # eşikler gerçek (enkoder) hataya göre
        if t20 is None and e < ESIK20: t20 = el
        if e < ESIK10:
            if ic10 is None: ic10 = el
            if t10 is None and el - ic10 >= 0.3: t10 = ic10          # 0,3 s içeride kaldıysa say
        else:
            ic10 = None
            if t10 is not None and el < 3.0: t10 = None              # dışarı taştıysa yeniden say
        if el >= 3.0 and son is None: son = e; sonimu = d["aim_err"]
    sonuc.append((dp, dt, g0, t20, t10, son, kesinti))
    f = lambda v: ("%.2f s" % v) if v is not None else "GIRMEDI"
    print("     %+3d / %+3d     |    %5.2f / %5.2f        |  %-8s |  %-8s |    %5.2f / %5.2f        | %s" % (
        dp, dt, e0, g0, f(t20), f(t10), sonimu, son if son is not None else float("nan"), ("%d ornek" % kesinti) if kesinti else "yok"))
    uyu(1.0)

d = tel()
print("bitti: mod %s, kilit %d, hata IMU %.2f / gercek %.2f derece (sistem otomatik takipte birakildi)" % (ml.MODES.get(int(d["mode"])), int(d["flags"]) & 1, d["aim_err"], gercek_hata(d)))
_kf.close()
t10s = [r[4] for r in sonuc if r[4] is not None]
if sonuc:
    print("ozet: %d bozmanin %d tanesi O10 icine dondu; en uzun %.2f s (sartname 8 s)" % (len(sonuc), len(t10s), max(t10s) if t10s else float("nan")))
for t in texts[-4:]: print("  [kart]", t[:110])
link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 20)); time.sleep(0.1); link.close()
