# Taban yatık tutulurken kart ne görüyor? (30 Eyl) Kilide dokunmadan kayıt alır; tutulan (durgun ve yatık) anlarda
# kartın ölçtüğü nişan hatasını (yan/üst) ve motorların yaptığı telafiyi yazar.
# Kullanım: python tutma_kayit.py [saniye=40]
import struct, sys, time
import mercom_link as ml
from serial.tools import list_ports

SURE = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0
port = ml.terminal_port()
if port is None: print("KART YOK"); raise SystemExit(1)
link = ml.Link(port); orig = link._handle
def h(mid, p):
    if mid in (ml.MSG_TEXT, ml.MSG_ACK, ml.MSG_PONG): return
    orig(mid, p)
link._handle = h
t0 = time.time()
while link.last_telem is None and time.time() - t0 < 8:
    link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 50)); time.sleep(0.5)
link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 50)); time.sleep(0.3)
d = dict(zip(ml.TELEM_COLS, link.last_telem))
print("BASLADI: mod %s, kilit %d, hata %.2f | %d s kayit" % (ml.MODES.get(int(d["mode"])), int(d["flags"]) & 1, d["aim_err"], SURE), flush=True)
rows = []; last = None; ts = time.time()
while time.time() - ts < SURE:
    time.sleep(0.005); t = link.last_telem
    if t is last: continue
    last = t; x = dict(zip(ml.TELEM_COLS, t)); x["t"] = time.time() - ts; rows.append(x)
link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 20)); time.sleep(0.1); link.close()
p0, t0_ = rows[0]["cmd_pan"], rows[0]["cmd_tilt"]
un = lambda v: ((v - p0 + 180) % 360) - 180
print("  t     | motor telafisi pan  tilt | kartin gordugu hata: toplam  yan(+sol)  ust(+yukari) | eksen hizi")
son = -1
for x in rows:
    if x["t"] - son < 1.0: continue
    son = x["t"]
    print("  %4.1f  |   %+6.2f   %+6.2f      |   %5.2f   %+6.2f   %+6.2f      | %4.1f" % (
        x["t"], un(x["cmd_pan"]), x["cmd_tilt"] - t0_, x["aim_err"], x["aim_side"], x["aim_up"], max(abs(x["vel_pan"]), abs(x["vel_tilt"]))))
