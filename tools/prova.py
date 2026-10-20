# Genel prova (30 Eyl): kilit + otomatik takip + kayıt + özet. Kullanıcı tabanı elle ±8° sallar.
# Kullanım: python prova.py [saniye=330] [tezgah] [mevcut]
#   mevcut: kart zaten otomatik takipte ve kilitliyse kilide dokunulmaz (kullanıcının seçtiği hedef korunur)
#   tezgah verilirse paralaks tezgah ölçüleriyle (duvar 2,254 m, tilt ekseni zeminden 0,19 m) çalışır (yalnız bu açılış için).
# Hata eşikleri açısal: yarışma mesafesinde (4,47 m) Ø10 = 0,64°, Ø20 = 1,28°; tezgahta (2,25 m) Ø10 = 1,27°.
import csv, os, struct, sys, time
import mercom_link as ml
from serial.tools import list_ports

SURE = float(sys.argv[1]) if len(sys.argv) > 1 else 330.0
TEZGAH = "tezgah" in sys.argv[2:]
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "prova_kayit.csv")
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
def setp(n, v): link.send(ml.MSG_PARAM_SET, struct.pack("<Hf", ml.PARAM_NAMES.index(n), float(v))); time.sleep(0.1)
def mode(m):
    for _ in range(8):
        link.send(ml.MSG_SET_MODE, bytes([m])); time.sleep(0.3)
        if int(tel()["mode"]) == m: return True
        time.sleep(0.5)
    return False
while link.last_telem is None and time.time() - T0 < 8:
    link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 50)); time.sleep(0.5)
if link.last_telem is None: print("TELEMETRI YOK"); raise SystemExit(1)
link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 50))
_d = tel()
if not ("mevcut" in sys.argv[2:] and int(_d["mode"]) == 2 and int(_d["flags"]) & 1):
    mode(1)
    link.send(ml.MSG_ENABLE, bytes([1])); time.sleep(0.3); link.send(ml.MSG_LASER, bytes([1])); time.sleep(1.0)
if TEZGAH: setp("plx_d", 2.254); setp("plx_ht", 0.19)
d = tel()
if "mevcut" in sys.argv[2:] and int(d["mode"]) == 2 and int(d["flags"]) & 1:
    print("mevcut kilit korunuyor (kullanici hedefe kilitlemis, otomatik takip suruyor)")
else:
    link.send(ml.MSG_LOCK); time.sleep(0.8)
    if not mode(2): print("OTOMATIGE GECILEMEDI"); raise SystemExit(1)
d = tel()
print("BASLADI: otomatik takip, kilit %d, hata %.2f, paralaks %s | %d s kayit -> %s" % (int(d["flags"]) & 1, d["aim_err"], "tezgah (2,254 m)" if TEZGAH else "yarisma (4,47 m)", SURE, OUT), flush=True)
rows = []; last = None; ts = time.time(); seen = len(texts)
with open(OUT, "w", newline="") as fh:
    w = csv.writer(fh); w.writerow(["t"] + list(ml.TELEM_COLS))
    while time.time() - ts < SURE:
        time.sleep(0.005); t = link.last_telem
        if t is not last:
            last = t; x = dict(zip(ml.TELEM_COLS, t)); x["t"] = time.time() - ts; rows.append(x)
            w.writerow([x["t"]] + list(t))
        for tt, s in texts[seen:]:
            if "imu:" in s or "nisan:" in s or "MERCOM-1 acildi" in s: print("   %6.1f s [kart] %s" % (tt - (ts - T0), s[:100]), flush=True)
        seen = len(texts)
        if int(time.time() - ts) % 30 == 0 and rows and rows[-1]["t"] > 1 and int(rows[-1]["t"]) != getattr(sys, "_son", -1):
            sys._son = int(rows[-1]["t"])
            son = [r for r in rows if r["t"] > rows[-1]["t"] - 30 and int(r["mode"]) == 2]
            if son:
                e = sorted(r["aim_err"] for r in son)
                print("   %4.0f s: son 30 s hata medyan %.2f  %%90 %.2f  en cok %.2f derece" % (rows[-1]["t"], e[len(e) // 2], e[int(0.9 * (len(e) - 1))], e[-1]), flush=True)
link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 20)); time.sleep(0.1); link.close()

auto = [r for r in rows if int(r["mode"]) == 2 and int(r["flags"]) & 1]
e = sorted(r["aim_err"] for r in auto)
if not e: print("otomatik ornek yok"); raise SystemExit
q = lambda p: e[min(len(e) - 1, int(p * (len(e) - 1)))]
pct = lambda thr: 100.0 * sum(1 for x in e if x < thr) / len(e)
stale = 100.0 * sum(1 for r in auto if not (int(r["flags"]) >> 2) & 1) / len(auto)
print("\nOZET (%d s otomatik, %d ornek)" % (auto[-1]["t"] - auto[0]["t"], len(auto)))
print("  hata medyan %.2f, %%90 %.2f, %%99 %.2f, en cok %.2f derece" % (q(0.5), q(0.9), q(0.99), e[-1]))
print("  yarisma mesafesinde (4,47 m): O10 icinde %%%.1f, O20 icinde %%%.1f" % (pct(0.64), pct(1.28)))
if TEZGAH: print("  tezgah mesafesinde (2,25 m): O10 icinde %%%.1f, O20 icinde %%%.1f" % (pct(1.27), pct(2.54)))
print("  IMU bayat ornek %%%.1f" % stale)
# O20 dışına çıkışlar ve geri dönüş süreleri
cik = []; dis = None
for r in auto:
    if r["aim_err"] >= 1.28 and dis is None: dis = r["t"]
    elif r["aim_err"] < 1.28 and dis is not None: cik.append((dis, r["t"] - dis)); dis = None
if cik:
    print("  O20 disina %d kez cikti; en uzun %.1f s (sartname 8 s); en uzunlar: %s" % (len(cik), max(c[1] for c in cik), ", ".join("%.0f. s'de %.1f s" % c for c in sorted(cik, key=lambda c: -c[1])[:4])))
else:
    print("  O20 disina hic cikmadi")
