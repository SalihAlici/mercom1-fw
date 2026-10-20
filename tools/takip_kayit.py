# Otomatik takipte telemetriyi CSV'ye canlı yazar ve sonunda özet çıkarır. Kullanım: takip_kayit.py [saniye] [kur]
#   kur verilirse önce: manuel mod, sürücüler açık, lazer açık, plx_en=1, kilit, otomatik.
import struct, sys, time, csv
sys.path.insert(0, r"C:\Users\Salih\Desktop\hareketli uydu\mercom1-fw\tools")
import mercom_link as ml
from serial.tools import list_ports

SURE = float(sys.argv[1]) if len(sys.argv) > 1 else 90.0
KUR = len(sys.argv) > 2
OUT = r"C:\Users\Salih\Desktop\hareketli uydu\mercom1-fw\tools\takip_kayit.csv"
port = ml.terminal_port()
texts = []
link = ml.Link(port); orig = link._handle
def h(mid, p):
    if mid == ml.MSG_TEXT: texts.append((time.time(), p.decode("ascii", "replace")))
    elif mid in (ml.MSG_ACK, ml.MSG_PONG): pass
    else: orig(mid, p)
link._handle = h
def setp(n, v): link.send(ml.MSG_PARAM_SET, struct.pack("<Hf", ml.PARAM_NAMES.index(n), float(v))); time.sleep(0.1)
t0 = time.time()
while link.last_telem is None and time.time() - t0 < 8:
    link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 50)); time.sleep(0.5)
if link.last_telem is None:
    print("TELEMETRI YOK"); raise SystemExit
if KUR:
    link.send(ml.MSG_SET_MODE, bytes([1])); time.sleep(0.3)
    for a in (0, 1, 2): link.send(ml.MSG_ENABLE, bytes([a, 1])); time.sleep(0.15)
    link.send(ml.MSG_LASER, bytes([1])); time.sleep(0.2)
    setp("plx_en", 1)
    time.sleep(1.0)
    link.send(ml.MSG_LOCK); time.sleep(0.8)
    link.send(ml.MSG_SET_MODE, bytes([2])); time.sleep(1.0)
    d = dict(zip(ml.TELEM_COLS, link.last_telem))
    print("kurulum: mod=%s kilit=%d lazer=%d hata %.3f" % (ml.MODES.get(d["mode"]), d["flags"] & 1, (d["flags"] >> 1) & 1, d["aim_err"]), flush=True)
cols = ["t"] + list(ml.TELEM_COLS)
rows = []; last = None; t0 = time.time(); sonflush = t0
with open(OUT, "w", newline="") as fh:
    w = csv.writer(fh); w.writerow(cols)
    while time.time() - t0 < SURE:
        time.sleep(0.005); t = link.last_telem
        if t is not last:
            last = t; d = dict(zip(ml.TELEM_COLS, t)); d["t"] = time.time() - t0; rows.append(d)
            w.writerow([d[c] for c in cols])
            if time.time() - sonflush > 1.0: fh.flush(); sonflush = time.time()
link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 20)); time.sleep(0.1); link.close()
print("kayit: %d ornek, %.0f s -> %s" % (len(rows), SURE, OUT))
