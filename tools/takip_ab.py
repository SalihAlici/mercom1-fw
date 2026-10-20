# Hareket halinde takip ayarı karşılaştırması (30 Eyl). Kullanıcı tabanı sürekli sallar; ayarlar sırayla 70 s uygulanır.
# Mevcut kilide dokunulmaz. Her dilimde: taban hareketi (telafi açıları, eksen hızı) ve hareket sırasındaki nişan hatası.
# Kullanım: python takip_ab.py [dilim_s=70]
import csv, os, struct, sys, time
import mercom_link as ml
from serial.tools import list_ports

DILIM = float(sys.argv[1]) if len(sys.argv) > 1 else 70.0
AYARLAR = [("1) simdiki (pan kp 8, pan onden besleme 0)", {"kp_pan": 8.0, "aim_kff_pan": 0.0}),
           ("2) pan kp 12", {"kp_pan": 12.0, "aim_kff_pan": 0.0}),
           ("3) pan onden besleme 0,5", {"kp_pan": 8.0, "aim_kff_pan": 0.5})]
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "takip_ab_kayit.csv")
port = ml.terminal_port()
if port is None: print("KART YOK"); raise SystemExit(1)
link = ml.Link(port); orig = link._handle
rows = []
def h(mid, p):
    if mid in (ml.MSG_TEXT, ml.MSG_ACK, ml.MSG_PONG): return
    orig(mid, p)
link._handle = h
def tel(): return dict(zip(ml.TELEM_COLS, link.last_telem))
def setp(n, v): link.send(ml.MSG_PARAM_SET, struct.pack("<Hf", ml.PARAM_NAMES.index(n), float(v))); time.sleep(0.08)
t0 = time.time()
while link.last_telem is None and time.time() - t0 < 8:
    link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 50)); time.sleep(0.5)
link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 50)); time.sleep(0.3)
d = tel()
if not (int(d["mode"]) == 2 and int(d["flags"]) & 1):
    print("UYARI: kart otomatik+kilitli degil (mod %s); simdiki bakis kilitleniyor" % ml.MODES.get(int(d["mode"])))
    link.send(ml.MSG_ENABLE, bytes([1])); time.sleep(0.3); link.send(ml.MSG_LOCK); time.sleep(0.8)
    link.send(ml.MSG_SET_MODE, bytes([2])); time.sleep(1.0)
print("BASLADI: %d ayar x %d s, kilit korunuyor. Tabani surekli salla." % (len(AYARLAR), DILIM), flush=True)
sonuc = []
with open(OUT, "w", newline="") as fh:
    w = csv.writer(fh); w.writerow(["ayar", "t"] + list(ml.TELEM_COLS))
    for ad, prm in AYARLAR:
        for k, v in prm.items(): setp(k, v)
        print("  %s uygulandi" % ad, flush=True)
        ts = time.time(); last = None; seg = []
        while time.time() - ts < DILIM:
            time.sleep(0.005); t = link.last_telem
            if t is last: continue
            last = t; x = dict(zip(ml.TELEM_COLS, t)); x["t"] = time.time() - ts
            w.writerow([ad, x["t"]] + list(t))
            if x["t"] > 5.0: seg.append(x)                 # geçişten sonraki 5 s sayılmaz
        sonuc.append((ad, seg))
setp("kp_pan", 8.0); setp("aim_kff_pan", 0.0)
link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 20)); time.sleep(0.1); link.close()
print("\nayarlar eski haline dondu (pan kp 8, pan onden besleme 0)\n")
print("  ayar                                      | taban hareketi: tilt/pan araligi  ort. eksen hizi | HAREKET SIRASINDA hata: medyan  %90   en cok  O10 ici | duraganken")
for ad, seg in sonuc:
    if not seg: continue
    tt = [x["cmd_tilt"] for x in seg]; p0 = seg[0]["cmd_pan"]
    pp = [((x["cmd_pan"] - p0 + 180) % 360) - 180 for x in seg]
    hiz = sum(max(abs(x["vel_pan"]), abs(x["vel_tilt"])) for x in seg) / len(seg)
    mov = [x for x in seg if abs(x["vel_pan"]) > 0.3 or abs(x["vel_tilt"]) > 0.3]
    rest = [x for x in seg if not (abs(x["vel_pan"]) > 0.3 or abs(x["vel_tilt"]) > 0.3)]
    e = sorted(x["aim_err"] for x in mov) or [float("nan")]
    er = sorted(x["aim_err"] for x in rest) or [float("nan")]
    print("  %-41s |  %5.1f / %5.1f derece     %5.1f d/s     |   %5.2f  %5.2f  %5.2f  %%%5.1f  | medyan %.2f" % (
        ad, max(tt) - min(tt), max(pp) - min(pp), hiz, e[len(e) // 2], e[int(0.9 * (len(e) - 1))], e[-1],
        100.0 * sum(1 for v in e if v < 0.64) / len(e), er[len(er) // 2]))
