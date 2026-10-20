# Yarışma öncesi tam sistem testi (30 Eyl 2026). USB ile terminale bağlanır, her maddeyi sırayla sınar, sonunda özet verir.
# Taban hareketsiz kalmalı (toparlanma ve yön bağlama testleri enkoderden gerçek hata ölçer). Başlık oynar, lazer yanar.
# Bazı maddeler mevcut betikleri çağırır (bno_hareket_testi, bozma_testi, baglama_testi).
# Kullanım: python yarisma_oncesi_test.py
import math, os, re, struct, subprocess, sys, time
import mercom_link as ml

HERE = os.path.dirname(os.path.abspath(__file__))
PY = sys.executable
BEKLENEN = {"plx_en": 1, "plx_d": 4.0, "plx_ht": 0.166, "plx_ly": -0.13, "aim_db": 0.3, "kp_pan": 8, "kp_tilt": 8, "tilt_min": -90,
            "tilt_max": 90, "ce_min": 0, "ce_max": 4, "cl_pan": 1, "cl_tilt": 1, "cl_ce": 0, "i2c_sync": 1, "imu_poll_us": 1000,
            "yaw_anchor": 1, "imu_mount_p": 5.512, "lora_telem_hz": 1, "aim_kff": 0.7, "aim_kff_pan": 0}
SONUC = []
def kayit(ad, ok, ayrinti=""):
    SONUC.append((ad, ok, ayrinti))
    print("%s %-46s %s" % ("[ OK ]" if ok else ("[ -- ]" if ok is None else "[HATA]"), ad, ayrinti), flush=True)

class Kart:
    def __init__(self):
        self.texts = []; self.acks = []
        self.link = ml.Link(ml.terminal_port()); orig = self.link._handle
        def h(mid, p):
            if mid == ml.MSG_TEXT: self.texts.append((time.time(), p.decode("ascii", "replace")))
            elif mid == ml.MSG_ACK: self.acks.append((time.time(), p[0], p[1]))
            elif mid == ml.MSG_PONG: self.pong = struct.unpack("<II", p)
            else: orig(mid, p)
        self.link._handle = h; self.pong = None
        t0 = time.time()
        while self.link.last_telem is None and time.time() - t0 < 8:
            self.link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 50)); time.sleep(0.4)
        self.link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 50))
    def t(self): return dict(zip(ml.TELEM_COLS, self.link.last_telem))
    def f(self): return int(self.t()["flags"])
    def send(self, mid, p=b""): self.link.send(mid, p)
    def param(self, n):
        self.send(ml.MSG_PARAM_GET, struct.pack("<H", ml.PARAM_NAMES.index(n))); time.sleep(0.06)
    def setp(self, n, v): self.send(ml.MSG_PARAM_SET, struct.pack("<Hf", ml.PARAM_NAMES.index(n), float(v))); time.sleep(0.08)
    def metin(self, mid, desen, bekle=1.0, p=b""):
        n0 = len(self.texts); self.send(mid, p); time.sleep(bekle)
        return [s for _, s in self.texts[n0:] if re.search(desen, s)]
    def bekle(self, kosul, sn):
        t = time.time()
        while time.time() - t < sn:
            try:
                if kosul(self.t()): return time.time() - t
            except Exception: pass
            time.sleep(0.03)
        return None
    def mode(self, m):
        for _ in range(6):
            self.send(ml.MSG_SET_MODE, bytes([m])); time.sleep(0.3)
            if int(self.t()["mode"]) == m: return True
            time.sleep(0.4)
        return False
    def git(self, ax, deg, sn=20):
        self.send(ml.MSG_GOTO, struct.pack("<Bf", ax, deg)); time.sleep(0.3)
        return self.bekle(lambda d: abs(d[("vel_pan", "vel_tilt", "vel_ce")[ax]]) < 0.05, sn)
    def close(self):
        try: self.send(ml.MSG_TELEM_RATE, struct.pack("<H", 20)); time.sleep(0.1); self.link.close()
        except Exception: pass

def un(v): return ((v + 180) % 360) - 180

def betik(ad, argv, zaman=600):
    print("   ... %s calisiyor" % " ".join(argv), flush=True)
    r = subprocess.run([PY, os.path.join(HERE, argv[0])] + argv[1:], capture_output=True, text=True, timeout=zaman, cwd=HERE,
                       env=dict(os.environ, PYTHONIOENCODING="utf-8"))
    return r.stdout + r.stderr

T_BAS = time.time()
print("=== MERCOM-1 yarisma oncesi test (%s) ===" % time.strftime("%H:%M"))
k = Kart()

# 1) bağlantı, sürüm
k.send(ml.MSG_PING); time.sleep(0.6)
kayit("Baglanti + yazilim surumu", k.pong is not None, ("surum %d.%d.%d, acik kalma %.0f s" % (k.pong[1] >> 16, (k.pong[1] >> 8) & 255, k.pong[1] & 255, k.pong[0] / 1000)) if k.pong else "PONG yok")

# 2) parametreler
for n in BEKLENEN: k.param(n)
time.sleep(0.5)
yanlis = []
for n, v in BEKLENEN.items():
    got = k.link.params.get(ml.PARAM_NAMES.index(n))
    if got is None or abs(got - v) > 1e-3 * max(1, abs(v)): yanlis.append("%s=%s (beklenen %s)" % (n, got, v))
kayit("Yarisma parametreleri (%d adet)" % len(BEKLENEN), not yanlis, "; ".join(yanlis) if yanlis else "hepsi dogru")

# 3) IMU 20 s
def imu_reset():
    for s in k.metin(ml.MSG_IMU_DIAG, r"reset=\d+", 0.8):
        m = re.search(r"reset=(\d+)", s)
        if m: return int(m.group(1))
    return None
r0 = imu_reset(); c0 = k.t()["imu_count"]; t0 = k.t()["t_ms"]; bayat = 0; n = 0
ts = time.time(); last = None
while time.time() - ts < 20:
    t = k.link.last_telem
    if t is not last:
        last = t; n += 1
        if not (int(dict(zip(ml.TELEM_COLS, t))["flags"]) >> 2) & 1: bayat += 1
    time.sleep(0.01)
r1 = imu_reset(); d = k.t(); hz = (d["imu_count"] - c0) / ((d["t_ms"] - t0) / 1000.0)
kayit("IMU (20 s)", hz > 80 and r1 == r0 and bayat == 0, "%.0f rapor/s, reset %s, bayat ornek %d" % (hz, (r1 - r0) if None not in (r0, r1) else "?", bayat))

# 4) enkoderler
enc = k.metin(ml.MSG_ENC_DIAG, r"^enc ", 0.8)
eok = True; ea = []
for s in enc:
    m = re.search(r"enc (\w+): raw=\d+ agc=(\d+) OCF=(\d) COF=(\d) yakin=(\d) uzak=(\d) ok=(\d+) err=(\d+).*sicrama=(\d+)", s)
    if m:
        ad, agc, ocf, cof, yak, uz, okc, err, sic = m.group(1), *map(int, m.groups()[1:])
        ea.append("%s AGC %d hata %d sicrama %d" % (ad, agc, err, sic))
        if not ocf or cof or uz or (ad != "ce" and yak) or err > 0: eok = False
kayit("Enkoderler (tani)", eok and len(ea) == 3, ", ".join(ea) if ea else "cevap yok")

# 5) sürücüler ve lazer
k.mode(1)
ok_s = True; ayr = []
for ax, bit in ((0, 5), (1, 6), (2, 7)):
    k.send(ml.MSG_ENABLE, bytes([ax, 1])); a = k.bekle(lambda d: (int(d["flags"]) >> bit) & 1, 3)
    k.send(ml.MSG_ENABLE, bytes([ax, 0])); b = k.bekle(lambda d: not (int(d["flags"]) >> bit) & 1, 3)
    if a is None or b is None: ok_s = False; ayr.append("%s cevap yok" % ("pan", "tilt", "ce")[ax])
kayit("Suruculer tek tek ac / kapat", ok_s, ", ".join(ayr) if ayr else "uc eksen de")
k.send(ml.MSG_LASER, bytes([1])); a = k.bekle(lambda d: (int(d["flags"]) >> 1) & 1, 3)
k.send(ml.MSG_LASER, bytes([0])); b = k.bekle(lambda d: not (int(d["flags"]) >> 1) & 1, 3)
kayit("Lazer ac / kapat", a is not None and b is not None)
k.send(ml.MSG_ENABLE, bytes([1])); time.sleep(0.5); k.send(ml.MSG_LASER, bytes([1])); time.sleep(0.3)

# 6) eksen hareketleri, enkoder doğrulaması, kapalı çevrim sahte düzeltme
def duzeltme():
    out = {}
    for s in k.metin(ml.MSG_AXIS_INFO, r"duzeltme=", 0.6):
        m = re.search(r"eksen (\w+): .*duzeltme=(\d+)", s)
        if m: out[m.group(1)] = int(m.group(2))
    return out
time.sleep(10)                                   # enkoder güveni (1000 okuma) otursun
d = k.t(); P0, T0, C0 = d["cmd_pan"], d["cmd_tilt"], d["cmd_ce"]; E0 = (d["enc_pan"], d["enc_tilt"], d["enc_ce"])
dz0 = duzeltme(); hat = []
for ax, hedef in ((0, (P0 + 5) % 360), (0, P0), (1, T0 + 5), (1, T0), (2, 3.0), (2, max(0.0, C0))):
    k.git(ax, hedef)
    d = k.t(); cmd = d[("cmd_pan", "cmd_tilt", "cmd_ce")[ax]]
    if abs(un(cmd - hedef)) > (0.05, 0.05, 0.23)[ax]: hat.append("%s hedef %.2f sayac %.2f" % (("pan", "tilt", "ce")[ax], hedef, cmd))
d = k.t()
denc = [un(d["enc_pan"] - E0[0]), d["enc_tilt"] - E0[1]]
dz1 = duzeltme()
yeni = {a: dz1.get(a, 0) - dz0.get(a, 0) for a in ("pan", "tilt")}
kayit("Eksen hareketleri (+-5 pan/tilt, CE 0-3; CE 1 mikroadim 0,225)", not hat and max(abs(x) for x in denc) < 0.3,
      ("; ".join(hat) + " | " if hat else "") + "donuste enkoder farki pan %+.2f tilt %+.2f" % tuple(denc))
kayit("Kapali cevrim sahte duzeltme yok", all(v == 0 for v in yeni.values()), str(yeni))

# 7) eksen kapsamı: pan tam tur (90'ar), tilt 85'e kadar
ok_k = True; ayr = []
for hedef in (90, 180, 270, 0):
    h = (P0 + hedef) % 360; t = k.git(0, h, 25)
    d = k.t(); e = un(d["enc_pan"] - E0[0] - hedef)
    if t is None or abs(e) > 0.5: ok_k = False; ayr.append("pan %d: enkoder farki %+.2f" % (hedef, e))
for hedef in (85.0, T0):
    t = k.git(1, hedef, 25)
    d = k.t()
    if t is None or abs(d["cmd_tilt"] - hedef) > 0.05: ok_k = False; ayr.append("tilt %.0f olmadi" % hedef)
d = k.t()
kayit("Eksen kapsami (pan 360, tilt %.0f->85->geri)" % T0, ok_k, ", ".join(ayr) if ayr else "pan 4 durak enkoderle dogru, tilt 85 ve geri")
k.close()

# 8) hareket halinde IMU reset (120 s, üç eksen salınım)
out = betik("IMU hareketli", ["bno_hareket_testi.py", "120"])
m = re.search(r"sensor reseti\s*:\s*(\d+).*?I2C hatasi\s*:\s*(\d+).*?IMU\s*:\s*(\d+) rapor/s, bayat ornek %([\d.]+)", out, re.S)
kayit("Hareket halinde IMU (120 s salinim)", bool(m) and m.group(1) == "0" and m.group(2) == "0",
      ("reset %s, I2C hatasi %s, %s rapor/s, bayat %%%s" % m.groups()) if m else out[-200:])

# 9) otomatik takip: duruş + bozma (şartname yöntem 2) — enkoderden gerçek hata
out = betik("bozma", ["bozma_testi.py", "yenikilit"])
satirlar = [l for l in out.splitlines() if re.match(r"\s+[+-]?\d+ /\s+[+-]?\d+", l)]
gir = [l for l in satirlar if "GIRMEDI" not in l.split("|")[3]] if satirlar else []
m = re.search(r"en uzun ([\d.]+) s", out)
kayit("Toparlanma (6 bozma, sartname 8 s)", bool(satirlar) and len(gir) == len(satirlar) and m and float(m.group(1)) < 8.0,
      ("%d/%d bozma O10'a dondu, en uzun %s s" % (len(gir), len(satirlar), m.group(1) if m else "?")))
print("\n".join("      " + l for l in out.splitlines() if "|" in l or "bitti" in l))

# 10) yön bağlama: başlık uzaktayken IMU bilerek yeniden başlatılır
out = betik("baglama", ["baglama_testi.py", "1", "5", "3", "-8", "0"])
g = [float(x) for x in re.findall(r"\|\s+[\d.]+ / ([\d.]+)\s+\(", out)]
kayit("IMU yeniden baslayinca hedef korunuyor", len(g) == 2 and max(g) < 1.2, "gercek hata %s derece" % ", ".join("%.2f" % x for x in g))

# 11) sürücü kapalıyken otomatik bekler
k = Kart()
k.mode(1); k.send(ml.MSG_ENABLE, bytes([1])); time.sleep(0.5); k.send(ml.MSG_LOCK); time.sleep(0.8); k.mode(2); time.sleep(1.5)
k.send(ml.MSG_ENABLE, bytes([0])); time.sleep(0.5); a = k.t(); time.sleep(3); b = k.t()
k.send(ml.MSG_ENABLE, bytes([1])); dv = k.bekle(lambda d: int(d["mode"]) == 2 and (int(d["flags"]) >> 4) & 1, 4)
kayit("Surucu kapaliyken otomatik bekliyor", abs(un(a["cmd_pan"] - b["cmd_pan"])) < 0.01 and abs(a["cmd_tilt"] - b["cmd_tilt"]) < 0.01 and dv is not None,
      "sayaclar sabit kaldi, surucu acilinca devam")
k.mode(1); k.send(ml.MSG_UNLOCK); time.sleep(0.3)
k.close()

# 12) LoRa
lp = ml.lora_port()
if lp is None:
    kayit("LoRa baglantisi", None, "yer istasyonu takili degil, atlandi")
else:
    out = betik("lora", ["lora_test.py"])
    m1 = re.search(r"PING -> PONG: ([\d.]+) s", out); m2 = re.search(r"telemetri: 6 s'de (\d+)", out); m3 = re.search(r"lazer kapat: ([\d.]+) s, lazer ac: ([\d.]+) s", out)
    kayit("LoRa (ping, telemetri, komut)", bool(m1 and m2 and m3) and int(m2.group(1)) >= 3,
          ("ping %s s, 6 s'de %s telemetri, lazer %s/%s s" % (m1.group(1) if m1 else "-", m2.group(1) if m2 else "-", m3.group(1) if m3 else "-", m3.group(2) if m3 else "-")))

# 13) kalıcılık: çalışırken değiştirilen parametre yeniden başlatmadan sonra kalıyor mu (şartname)
k = Kart()
k.setp("vmax_ce", 19.0); k.param("vmax_ce"); time.sleep(0.3)
once = k.link.params.get(ml.PARAM_NAMES.index("vmax_ce"))
k.send(ml.MSG_PARAM_SAVE); time.sleep(0.4)
kaydet_ack = [s for _, i, s in k.acks if i == ml.MSG_PARAM_SAVE]
k.send(ml.MSG_SYS_RESET); time.sleep(0.5); k.close(); time.sleep(6)
k = Kart(); k.param("vmax_ce"); time.sleep(0.4)
sonra = k.link.params.get(ml.PARAM_NAMES.index("vmax_ce"))
kayit("Parametre kaliciligi (karta kaydet + yeniden basla)", sonra is not None and abs(sonra - 19.0) < 1e-3,
      "degistirilen 19 -> yeniden baslayinca %s; karta kaydet cevabi %s (0 = destekleniyor)" % (sonra, kaydet_ack[-1] if kaydet_ack else "yok"))
k.setp("vmax_ce", 20.0); k.send(ml.MSG_PARAM_SAVE); time.sleep(0.5)
geri = [s for _, i, s in k.acks if i == ml.MSG_PARAM_SAVE]
kayit("Yarisma degerleri geri kaydedildi (vmax_ce 20)", bool(geri) and geri[-1] == 0)
# son kontrol: kalıcı kayıttan açılış, bütün yarışma değerleri (params.json dahil)
import json
TUM = dict(BEKLENEN); TUM.update(json.load(open(os.path.join(HERE, "params.json"))))
k.send(ml.MSG_SYS_RESET); time.sleep(0.5); k.close(); time.sleep(6)
k = Kart()
for n in TUM: k.param(n)
n0 = len(k.texts); k.send(ml.MSG_PING); time.sleep(0.8)
nv = next((s for _, s in k.texts[n0:] if "param kaydi" in s), "")
yan = ["%s=%s" % (n, k.link.params.get(ml.PARAM_NAMES.index(n))) for n, v in TUM.items()
       if k.link.params.get(ml.PARAM_NAMES.index(n)) is None or abs(k.link.params[ml.PARAM_NAMES.index(n)] - v) > 1e-3 * max(1, abs(v))]
kayit("Kalici kayittan acilis: %d yarisma degeri" % len(TUM), not yan and "yuklendi" in nv, ("; ".join(yan) or "hepsi dogru") + " | " + nv)

# bitiş: güvenli durum
k.mode(1); k.send(ml.MSG_LASER, bytes([0])); time.sleep(0.2); k.send(ml.MSG_ENABLE, bytes([0])); time.sleep(0.3)
k.close()
print("\n=== OZET (%.0f dk) ===" % ((time.time() - T_BAS) / 60))
for ad, ok, ay in SONUC: print("%s %-46s %s" % ("[ OK ]" if ok else ("[ -- ]" if ok is None else "[HATA]"), ad, ay))
print("gecen %d / %d" % (sum(1 for _, ok, _ in SONUC if ok), sum(1 for _, ok, _ in SONUC if ok is not None)))
print("son durum: manuel mod, lazer kapali, suruculer kapali")
