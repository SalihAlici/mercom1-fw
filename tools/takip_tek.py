# Tek ayarla hareketli takip ölçümü (30 Eyl). Kilit korunur; ayar uygulanır, süre boyunca kaydedilir, sonunda eski ayara dönülür.
# Kullanım: python takip_tek.py <ayar 1|2|3> [saniye=45]
#   1: şimdiki (pan kp 8, pan önden besleme 0)   2: pan kp 12   3: pan önden besleme 0,5
import csv, os, struct, sys, time
import mercom_link as ml
from serial.tools import list_ports

AYAR = int(sys.argv[1]) if len(sys.argv) > 1 else 1
SURE = float(sys.argv[2]) if len(sys.argv) > 2 else 45.0
AYARLAR = {1: ("simdiki (pan kp 8, onden besleme 0)", {"kp_pan": 8.0, "aim_kff_pan": 0.0}),
           2: ("pan kp 12", {"kp_pan": 12.0, "aim_kff_pan": 0.0}),
           3: ("pan onden besleme 0,5", {"kp_pan": 8.0, "aim_kff_pan": 0.5})}
ad, prm = AYARLAR[AYAR]
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "takip_tek_%d.csv" % AYAR)
port = ml.terminal_port()
if port is None: print("KART YOK"); raise SystemExit(1)
link = ml.Link(port); orig = link._handle
def h(mid, p):
    if mid in (ml.MSG_TEXT, ml.MSG_ACK, ml.MSG_PONG): return
    orig(mid, p)
link._handle = h
def tel(): return dict(zip(ml.TELEM_COLS, link.last_telem))
def setp(n, v): link.send(ml.MSG_PARAM_SET, struct.pack("<Hf", ml.PARAM_NAMES.index(n), float(v))); time.sleep(0.08)
t0 = time.time()
while link.last_telem is None and time.time() - t0 < 8:
    link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 50)); time.sleep(0.5)
link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 50)); time.sleep(0.2)
d = tel()
if not (int(d["mode"]) == 2 and int(d["flags"]) & 1):
    print("UYARI: kart otomatik+kilitli degildi; simdiki bakis kilitlendi")
    link.send(ml.MSG_ENABLE, bytes([1])); time.sleep(0.3); link.send(ml.MSG_LOCK); time.sleep(0.8)
    link.send(ml.MSG_SET_MODE, bytes([2])); time.sleep(1.0)
for k, v in prm.items(): setp(k, v)
print("BASLADI: ayar %d (%s), %d s" % (AYAR, ad, SURE), flush=True)
seg = []; last = None; ts = time.time()
with open(OUT, "w", newline="") as fh:
    w = csv.writer(fh); w.writerow(["t"] + list(ml.TELEM_COLS))
    while time.time() - ts < SURE:
        time.sleep(0.005); t = link.last_telem
        if t is last: continue
        last = t; x = dict(zip(ml.TELEM_COLS, t)); x["t"] = time.time() - ts; seg.append(x)
        w.writerow([x["t"]] + list(t))
setp("kp_pan", 8.0); setp("aim_kff_pan", 0.0)
link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 20)); time.sleep(0.1); link.close()
tt = [x["cmd_tilt"] for x in seg]; p0 = seg[0]["cmd_pan"]
pp = [((x["cmd_pan"] - p0 + 180) % 360) - 180 for x in seg]
mov = [x for x in seg if abs(x["vel_pan"]) > 0.3 or abs(x["vel_tilt"]) > 0.3]
hiz = sorted(max(abs(x["vel_pan"]), abs(x["vel_tilt"])) for x in mov) or [0.0]
e = sorted(x["aim_err"] for x in mov) or [float("nan")]
print("SONUC ayar %d (%s):" % (AYAR, ad))
print("  taban hareketi: tilt araligi %.1f, pan araligi %.1f derece; hareketli sure %.0f s; eksen hizi medyan %.1f, en cok %.1f d/s" % (
    max(tt) - min(tt), max(pp) - min(pp), len(mov) * 0.02, hiz[len(hiz) // 2], hiz[-1]))
print("  HAREKET SIRASINDA hata: medyan %.2f, %%90 %.2f, en cok %.2f derece; O10 (0,64) icinde %%%.1f, O20 (1,28) icinde %%%.1f" % (
    e[len(e) // 2], e[int(0.9 * (len(e) - 1))], e[-1], 100.0 * sum(1 for v in e if v < 0.64) / max(1, len(e)), 100.0 * sum(1 for v in e if v < 1.28) / max(1, len(e))))
print("  ayar eski haline dondu")
