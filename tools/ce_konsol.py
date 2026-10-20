# CE motoru için hızlı konsol: açı yaz, motor o açıya gider, hareket ekranda canlı görünür.
#   30        -> CE 30 dereceye gider (sınır yok, ±720)
#   h 40      -> hız 40 derece/s
#   t 20      -> tilt 20 dereceye      p 45 -> pan 45 dereceye
#   s         -> dur        q -> çık (CE sınırı 0-4'e geri döner)
import struct, sys, time
sys.path.insert(0, r"C:\Users\Salih\Desktop\hareketli uydu\mercom1-fw\tools")
import mercom_link as ml
from serial.tools import list_ports

port = sys.argv[1] if len(sys.argv) > 1 else next((p.device for p in list_ports.comports() if p.vid in (0x0483, 0x16C0)), None)
if not port:
    print("kart bulunamadi"); time.sleep(3); raise SystemExit
texts = []
link = ml.Link(port); orig = link._handle
def h(mid, p):
    if mid == ml.MSG_TEXT: texts.append(p.decode("ascii", "replace"))
    elif mid == ml.MSG_ACK:
        if p[1] != 0: print("  kart reddetti:", ml.ACK.get(p[1], p[1]))
    elif mid == ml.MSG_PONG: pass
    else: orig(mid, p)
link._handle = h
def setp(name, val): link.send(ml.MSG_PARAM_SET, struct.pack("<Hf", ml.PARAM_NAMES.index(name), float(val))); time.sleep(0.05)
def tel(): return dict(zip(ml.TELEM_COLS, link.last_telem))
AX = {"ce": 2, "tilt": 1, "pan": 0}

t0 = time.time()
while link.last_telem is None and time.time() - t0 < 8:
    link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 50)); time.sleep(0.5)
if link.last_telem is None:
    print("telemetri yok; ana firmware yuklu mu?"); time.sleep(3); raise SystemExit
link.send(ml.MSG_SET_MODE, bytes([1])); time.sleep(0.2)
for a in (0, 1, 2): link.send(ml.MSG_ENABLE, bytes([a, 1])); time.sleep(0.1)
setp("ce_min", -720.0); setp("ce_max", 720.0); setp("vmax_ce", 30.0)
print("port %s | uc surucu ACIK | CE siniri kaldirildi (+-720), hiz 30 derece/s" % port)
print("aci yaz + Enter (orn. 30 veya -45). h 60 = hiz, t 20 = tilt, p 45 = pan, s = dur, q = cik\n")

def izle(ad):
    ax = AX[ad]; sakin = 0; t0 = time.time()
    while time.time() - t0 < 60:
        time.sleep(0.1); d = tel()
        print("  %-4s komut %8.2f  hiz %6.1f  enkoder %8.2f" % (ad, d["cmd_" + ad], d["vel_" + ad], d["enc_" + ad]), end="\r", flush=True)
        sakin = sakin + 1 if abs(d["vel_" + ad]) < 0.01 else 0
        if sakin >= 5: break
    print()

while True:
    try:
        s = input("aci> ").strip().lower().replace(",", ".")
    except (EOFError, KeyboardInterrupt):
        s = "q"
    if not s: continue
    if s == "q": break
    if s == "s":
        link.send(ml.MSG_STOP); print("  durduruldu"); continue
    parts = s.split()
    try:
        if parts[0] == "h":
            setp("vmax_ce", float(parts[1])); print("  CE hiz = %s derece/s" % parts[1]); continue
        ad = "ce"; val = None
        if parts[0] in ("t", "p"):
            ad = "tilt" if parts[0] == "t" else "pan"; val = float(parts[1])
        else:
            val = float(parts[0])
    except (ValueError, IndexError):
        print("  anlamadim"); continue
    link.send(ml.MSG_GOTO, struct.pack("<Bf", AX[ad], val))
    print("  %s -> %.2f derece" % (ad, val))
    izle(ad)

setp("ce_min", 0.0); setp("ce_max", 4.0); setp("vmax_ce", 20.0)
link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 20)); time.sleep(0.1); link.close()
print("CE siniri 0-4'e geri alindi. Suruculer acik birakildi.")
