# BNO085 reset deneyi (29 Eyl): IMU okuma aralığını (imu_poll_us) değiştirip sensör reset sıklığını karşılaştırır.
# Sıra dönüşümlü (ör. 0, 2000, 0, 2000) ki zamanla değişen bir etki (ısınma, kablo) bir tarafa yığılmasın.
# Kullanım: python bno_ab_test.py <her_biri_saniye> [--param ad] <deger1> <deger2> ...   (varsayılan param imu_poll_us)
import re, struct, sys, time
import mercom_link as ml
from serial.tools import list_ports

SURE = float(sys.argv[1]) if len(sys.argv) > 1 else 90.0
ARG = sys.argv[2:]
PARAM = "imu_poll_us"
if "--param" in ARG:
    i = ARG.index("--param"); PARAM = ARG[i + 1]; ARG = ARG[:i] + ARG[i + 2:]
DEGERLER = [float(a) for a in ARG] or [0.0, 2000.0, 0.0, 2000.0]
ONCEKI = None
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
def sayaclar(deneme=0):
    n0 = len(texts)
    link.send(ml.MSG_IMU_DIAG); link.send(ml.MSG_PING); time.sleep(1.5)   # kart o an sensörü başlatıyorsa geç cevap verir
    r = k = None
    if deneme < 2 and not any("reset=" in s for s in texts[n0:]): return sayaclar(deneme + 1)
    for s in texts[n0:]:
        m = re.search(r"reset=(\d+)", s)
        if m: r = int(m.group(1))
        m = re.search(r"kesilme (\d+)", s)
        if m: k = int(m.group(1))
    return r, k, tel()["imu_count"]
t0 = time.time()
while link.last_telem is None and time.time() - t0 < 8:
    link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 20)); time.sleep(0.5)
if link.last_telem is None: print("TELEMETRI YOK"); raise SystemExit(1)
link.send(ml.MSG_SET_MODE, bytes([1])); time.sleep(0.3)
for a in (0, 1, 2): link.send(ml.MSG_ENABLE, bytes([a, 1])); time.sleep(0.15)
print("%s | suruculer acik, her deger %d s" % (port, SURE))
link.send(ml.MSG_PARAM_GET, struct.pack("<H", ml.PARAM_NAMES.index(PARAM))); time.sleep(0.3)
ONCEKI = link.params.get(ml.PARAM_NAMES.index(PARAM))
print("  %11s | sensor reseti | akis kesilmesi | ort. rapor/s | bayat ornek" % PARAM[:11])
toplam = {}
for v in DEGERLER:
    link.send(ml.MSG_PARAM_SET, struct.pack("<Hf", ml.PARAM_NAMES.index(PARAM), v)); time.sleep(3.0)
    r0, k0, c0 = sayaclar(); ts = time.time(); bayat = n = 0; last = None
    while time.time() - ts < SURE:
        time.sleep(0.02); t = link.last_telem
        if t is last: continue
        last = t; n += 1
        if not (int(dict(zip(ml.TELEM_COLS, t))["flags"]) >> 2) & 1: bayat += 1
    r1, k1, c1 = sayaclar(); dt = time.time() - ts
    rr = (r1 - r0) if None not in (r0, r1) else -1
    kk = (k1 - k0) if None not in (k0, k1) else -1
    print("  %11.0f | %13d | %14d | %12.0f | %%%.1f" % (v, rr, kk, (c1 - c0) / dt, 100.0 * bayat / max(1, n)))
    if rr >= 0 and kk >= 0:                          # sayaç okunamayan blok toplama katılmaz
        a = toplam.setdefault(v, [0, 0, 0.0]); a[0] += rr; a[1] += kk; a[2] += dt
print("toplam:")
for v, (rr, kk, dt) in toplam.items():
    print("  %s %5.0f: %d reset, %d kesilme, %.0f s -> dakikada %.1f reset" % (PARAM, v, rr, kk, dt, 60.0 * rr / dt))
if ONCEKI is not None: link.send(ml.MSG_PARAM_SET, struct.pack("<Hf", ml.PARAM_NAMES.index(PARAM), ONCEKI)); time.sleep(0.2)
link.close()
