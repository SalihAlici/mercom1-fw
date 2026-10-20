# bno_teensy testini sürer: evreler halinde kesme benzerini kapatıp açar, her evrede sensör resetlerini sayar.
# Kullanım: python izle.py [evre_s=120]    (sıra: kapalı, açık, kapalı)
import re, sys, time
import serial
from serial.tools import list_ports

EVRE = float(sys.argv[1]) if len(sys.argv) > 1 else 120.0
SIRA = [0, 1, 0]
port = next((p.device for p in list_ports.comports() if p.vid == 0x16C0), None)
if port is None: print("TEENSY YOK"); raise SystemExit(1)
ser = serial.Serial(port, 115200, timeout=0.2)
PAT = re.compile(r"s=(\d+) mod=(\d) rapor=(\d+) reset=(\d+) kesinti=(\d+)")
print("%s | her evre %d s, sira %s (1 = ana yazilimdaki 354 us'lik kesmenin benzeri acik)" % (port, EVRE, SIRA))
t_bas = time.time(); buf = b""
# sensör başlasın
while time.time() - t_bas < 6:
    buf += ser.read(512)
ozet = []
for mod in SIRA:
    ser.write(b"1" if mod else b"0"); ser.flush()
    ts = time.time(); satirlar = []; ilk = son = None
    while time.time() - ts < EVRE:
        buf += ser.read(512)
        while b"\n" in buf:
            ln, buf = buf.split(b"\n", 1)
            s = ln.decode("ascii", "replace").strip()
            m = PAT.search(s)
            if m:
                v = [int(x) for x in m.groups()]
                if v[1] != mod: continue
                if ilk is None: ilk = v
                son = v; satirlar.append(v[2])
            elif s:
                print("   %6.1f s [mod %d] %s" % (time.time() - ts, mod, s))
    if ilk and son:
        r = son[3] - ilk[3]; k = son[4] - ilk[4]; sure = son[0] - ilk[0]
        ort = sum(satirlar) / max(1, len(satirlar))
        ozet.append((mod, r, k, sure, ort))
        print("evre mod=%d: %d s, sensor reseti %d, kesinti %d, ortalama %.0f rapor/s" % (mod, sure, r, k, ort))
print("\nOZET")
for mod, r, k, sure, ort in ozet:
    print("  kesme benzeri %-6s: %3d s'de %2d reset (dakikada %.1f), %.0f rapor/s" % ("ACIK" if mod else "kapali", sure, r, 60.0 * r / max(1, sure), ort))
ser.write(b"0"); ser.close()
