# 29 Eyl düzeltmelerinin testi:
#  1) telemetri eksen başına sürücü bitleri (arayüz kutucukları bunlarla senkron)
#  2) otomatik takip sürücü kapalıyken bekler: sayaçlar kaçmaz, sürücü açılınca devam eder
#  3) kapalı çevrim: normal harekette sahte düzeltme yok; sayaç-enkoder farkı oluşunca (kaçırma benzetimi: enc_off
#     çalışırken kaydırılır) sayaç enkodere eşitlenir, eksen hedefe geri yürür
import re, struct, time
import mercom_link as ml
from serial.tools import list_ports

port = ml.terminal_port()
if port is None: print("KART YOK"); raise SystemExit(1)
T0 = time.time(); texts = []
link = ml.Link(port); orig = link._handle
def h(mid, p):
    if mid == ml.MSG_TEXT: texts.append((time.time() - T0, p.decode("ascii", "replace")))
    elif mid in (ml.MSG_ACK, ml.MSG_PONG): pass
    else: orig(mid, p)
link._handle = h
def tel(): return dict(zip(ml.TELEM_COLS, link.last_telem))
def setp(n, v): link.send(ml.MSG_PARAM_SET, struct.pack("<Hf", ml.PARAM_NAMES.index(n), float(v))); time.sleep(0.12)
def duzeltme(yaz=False):
    n0 = len(texts); link.send(ml.MSG_AXIS_INFO); time.sleep(0.5)
    out = {}
    for _, s in texts[n0:]:
        m = re.search(r"eksen (\w+): .*duzeltme=(\d+)", s)
        if m: out[m.group(1)] = int(m.group(2))
        if yaz and "kapali cevrim" in s and not s.startswith("eksen ce"): print("      " + s)
    return out
def bekle_dur(sn=8.0):
    t = time.time(); time.sleep(0.3)
    while time.time() - t < sn:
        d = tel()
        if abs(d["vel_pan"]) < 0.05 and abs(d["vel_tilt"]) < 0.05: return True
        time.sleep(0.05)
    return False
def en_bitleri(d): return "pan %d tilt %d ce %d (genel %d)" % ((int(d["flags"]) >> 5) & 1, (int(d["flags"]) >> 6) & 1, (int(d["flags"]) >> 7) & 1, (int(d["flags"]) >> 4) & 1)
def yeni_metin(n0):
    for tt, s in texts[n0:]: print("      [kart] %s" % s[:110])

while link.last_telem is None and time.time() - T0 < 8:
    link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 50)); time.sleep(0.5)
if link.last_telem is None: print("TELEMETRI YOK"); raise SystemExit(1)
for n in ("cl_pan", "cl_tilt", "cl_ce"): link.send(ml.MSG_PARAM_GET, struct.pack("<H", ml.PARAM_NAMES.index(n))); time.sleep(0.1)
time.sleep(0.3)
print("kapali cevrim: pan %s tilt %s ce %s" % tuple(link.params.get(ml.PARAM_NAMES.index(n)) for n in ("cl_pan", "cl_tilt", "cl_ce")))
link.send(ml.MSG_SET_MODE, bytes([1])); time.sleep(0.3)

print("\n1) SURUCU BITLERI")
link.send(ml.MSG_ENABLE, bytes([0])); time.sleep(0.4); print("   hepsi kapali : " + en_bitleri(tel()))
link.send(ml.MSG_ENABLE, bytes([1, 1])); time.sleep(0.4); print("   yalniz tilt  : " + en_bitleri(tel()))
link.send(ml.MSG_ENABLE, bytes([1])); time.sleep(0.4); print("   hepsi acik   : " + en_bitleri(tel()))
link.send(ml.MSG_LASER, bytes([1])); time.sleep(0.3); print("   lazer biti   : %d" % ((int(tel()["flags"]) >> 1) & 1))

print("\n   enkoderlerin guvenilir olmasi bekleniyor (1000 saglikli okuma, ~10 s)...")
time.sleep(11)

print("\n2) OTOMATIK TAKIP SURUCU KAPALIYKEN BEKLER")
link.send(ml.MSG_LOCK); time.sleep(0.8); link.send(ml.MSG_SET_MODE, bytes([2])); time.sleep(2.0)
d = tel(); print("   otomatik: mod %s kilit %d hata %.2f" % (ml.MODES.get(int(d["mode"])), int(d["flags"]) & 1, d["aim_err"]))
n0 = len(texts)
link.send(ml.MSG_ENABLE, bytes([0])); time.sleep(0.5)
a = tel(); time.sleep(4.0); b = tel()
print("   suruculer kapali 4 s: pan sayaci %.2f -> %.2f, tilt sayaci %.2f -> %.2f (degismemeli), mod %s" % (a["cmd_pan"], b["cmd_pan"], a["cmd_tilt"], b["cmd_tilt"], ml.MODES.get(int(b["mode"]))))
link.send(ml.MSG_ENABLE, bytes([1])); time.sleep(3.0)
d = tel(); print("   suruculer acildi: mod %s hata %.2f" % (ml.MODES.get(int(d["mode"])), d["aim_err"]))
yeni_metin(n0)

print("\n3) KAPALI CEVRIM")
link.send(ml.MSG_SET_MODE, bytes([1])); time.sleep(0.5)
d = tel(); P0, T0_ = d["cmd_pan"], d["cmd_tilt"]
r0 = duzeltme(True)
for dp, dt in ((10, 5), (-10, -5), (0, 0)):
    link.send(ml.MSG_GOTO, struct.pack("<Bf", 0, (P0 + dp) % 360)); link.send(ml.MSG_GOTO, struct.pack("<Bf", 1, T0_ + dt)); bekle_dur(); time.sleep(0.5)
r1 = duzeltme()
d = tel()
print("   normal hareket (+-10 pan, +-5 tilt): duzeltme pan %d tilt %d (0 olmali) | sayac-enkoder farki pan %+.2f tilt %+.2f" % (
    r1.get("pan", 0) - r0.get("pan", 0), r1.get("tilt", 0) - r0.get("tilt", 0), d["err_pan"], d["err_tilt"]))
# kaçırma benzetimi: enkoder okumasını 3° kaydır = motor sayaçtan 3° geride kaldı
e0 = tel()["enc_pan"]; n0 = len(texts)
setp("enc_off_pan", 3.0); time.sleep(2.0)
r2 = duzeltme(True); d = tel()
print("   pan 3 derece kacirma benzetimi: duzeltme +%d, enkoder %.2f -> %.2f (motor hedefe geri yurudu mu: ~+3)" % (
    r2.get("pan", 0) - r1.get("pan", 0), e0, d["enc_pan"]))
setp("enc_off_pan", 0.0); time.sleep(2.0)
r3 = duzeltme(); d = tel()
print("   geri alindi: duzeltme +%d, enkoder %.2f, pan sayaci %.2f (hedef %.2f)" % (r3.get("pan", 0) - r2.get("pan", 0), d["enc_pan"], d["cmd_pan"], P0))
yeni_metin(n0)
link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 20)); time.sleep(0.1); link.close()
