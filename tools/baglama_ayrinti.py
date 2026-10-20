# Yön bağlama testinin ayrıntılı dökümü: her bozmada kartın bütün mesajları ve telemetri zaman çizelgesi.
# Kullanım: python baglama_ayrinti.py [pan_bozma tilt_bozma] ...   (varsayılan: 5 3  -5 -3  8 0)
import math, struct, sys, time
import mercom_link as ml
from serial.tools import list_ports

args = [float(a) for a in sys.argv[1:]]
BOZMALAR = list(zip(args[0::2], args[1::2])) if len(args) >= 2 else [(5, 3), (-5, -3), (8, 0)]
port = ml.terminal_port()
T0 = time.time()
texts = []; tl = []
link = ml.Link(port); orig = link._handle
def h(mid, p):
    if mid == ml.MSG_TEXT: texts.append((time.time() - T0, p.decode("ascii", "replace")))
    elif mid == ml.MSG_ACK: texts.append((time.time() - T0, "ACK id=%d durum=%d" % (p[0], p[1])))
    elif mid == ml.MSG_PONG: pass
    else:
        orig(mid, p)
        if mid == ml.MSG_TELEM and link.last_telem is not None: tl.append((time.time() - T0, dict(zip(ml.TELEM_COLS, link.last_telem))))
link._handle = h
def tel(): return dict(zip(ml.TELEM_COLS, link.last_telem))
def imu_ok(d): return (int(d["flags"]) >> 2) & 1
def bekle_imu(sn=10.0):
    t = time.time(); iyi = 0
    while time.time() - t < sn and iyi < 15:
        iyi = iyi + 1 if imu_ok(tel()) else 0; time.sleep(0.04)
    return iyi >= 15
def bekle_dur(sn=8.0):
    t = time.time(); time.sleep(0.3)
    while time.time() - t < sn:
        d = tel()
        if abs(d["vel_pan"]) < 0.05 and abs(d["vel_tilt"]) < 0.05: return True
        time.sleep(0.05)
    return False
while link.last_telem is None and time.time() - T0 < 8:
    link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 50)); time.sleep(0.5)
bekle_imu()
link.send(ml.MSG_SET_MODE, bytes([1])); time.sleep(0.4)
for a in (0, 1, 2): link.send(ml.MSG_ENABLE, bytes([a, 1])); time.sleep(0.15)
link.send(ml.MSG_LASER, bytes([1])); time.sleep(1.0)
link.send(ml.MSG_LOCK); time.sleep(0.8)
link.send(ml.MSG_SET_MODE, bytes([2])); time.sleep(2.5)
d = tel(); PAN0, TILT0 = d["cmd_pan"], d["cmd_tilt"]; ENC0 = (d["enc_pan"], d["enc_tilt"])
print("hedef: pan %.2f tilt %.2f enk %.2f/%.2f" % (PAN0, TILT0, ENC0[0], ENC0[1]))
for dp, dt in BOZMALAR:
    link.send(ml.MSG_SET_MODE, bytes([1])); time.sleep(0.4)
    link.send(ml.MSG_GOTO, struct.pack("<Bf", 0, (PAN0 + dp) % 360.0)); link.send(ml.MSG_GOTO, struct.pack("<Bf", 1, TILT0 + dt))
    bekle_dur(); time.sleep(0.5)
    ts = time.time() - T0; nt = len(texts); nl = len(tl)
    print("\n===== bozma %+g / %+g, IMU yeniden baslatiliyor (t=%.2f) =====" % (dp, dt, ts))
    link.send(ml.MSG_IMU_INIT)
    time.sleep(1.5); bekle_imu(8.0); time.sleep(0.5)
    for _ in range(5):
        link.send(ml.MSG_SET_MODE, bytes([2])); time.sleep(0.2)
        if int(tel()["mode"]) == 2: break
        time.sleep(0.5)
    time.sleep(4.0)
    for tt, s in texts[nt:]: print("   %6.2f  [kart] %s" % (tt - ts, s[:110]))
    print("     t    mod byr | enk pan  enk tilt | IMU hata  yan    ust  | yaw(ham)  | imu_count")
    son = -1.0
    for tt, x in tl[nl:]:
        if tt - son < 0.25: continue
        son = tt
        dpn = ((x["enc_pan"] - ENC0[0] + 180) % 360) - 180
        print("   %6.2f  %d  %3d | %+6.2f  %+6.2f   | %5.2f  %+5.2f  %+5.2f | %7.2f   | %d" % (
            tt - ts, x["mode"], x["flags"], dpn, x["enc_tilt"] - ENC0[1], x["aim_err"], x["aim_side"], x["aim_up"], x["yaw"], x["imu_count"]))
link.send(ml.MSG_SET_MODE, bytes([1])); time.sleep(0.3)
link.send(ml.MSG_GOTO, struct.pack("<Bf", 0, PAN0)); link.send(ml.MSG_GOTO, struct.pack("<Bf", 1, TILT0)); bekle_dur()
link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 20)); time.sleep(0.1); link.close()
