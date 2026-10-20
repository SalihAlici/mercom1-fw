"""motor_term yüklü kartta üç motoru sırayla dener: sürücüyü açar, küçük bir ileri geri hareket verir, sürücüyü
bulduğu duruma geri getirir. Enkoder okumaz; motorun gerçekten dönüp dönmediğine gözle bakılır.
Kullanım: python motor_kontrol.py [COM8] [pan tilt ce]"""
import re
import sys
import time

import serial

PORT = next((a for a in sys.argv[1:] if a.upper().startswith("COM")), "COM8")
SECILI = [a for a in sys.argv[1:] if a in ("pan", "tilt", "ce")] or ["pan", "tilt", "ce"]
HAREKET = {"pan": 20.0, "tilt": 20.0, "ce": 10.0}       # derece, gidip geri gelir

s = serial.Serial(PORT, 115200, timeout=0.05)
time.sleep(0.3); s.reset_input_buffer()
t0 = time.time()


def oku(sure, bekle=None):
    son = time.time() + sure; buf = ""
    while time.time() < son:
        buf += s.read(4096).decode("ascii", "replace")
        if bekle and bekle in buf:
            break
    return buf


def komut(c, sure=0.3, bekle=None, yaz=True):
    s.write((c + "\r\n").encode())
    out = oku(sure, bekle)
    if yaz:
        for ln in out.splitlines():
            if ln.strip():
                print(f"[{time.time() - t0:5.1f}] {c:9s} -> {ln.strip()}")
    return out


ilk = komut("durum", yaz=False)
m = re.search(r"eksen=(\w+)", ilk); ilk_eksen = m.group(1) if m else "pan"
m = re.search(r"en=pan:(\d) tilt:(\d) ce:(\d)", ilk)
ilk_en = dict(zip(("pan", "tilt", "ce"), m.groups())) if m else {}
print("baslangic: secili eksen", ilk_eksen, " suruculer", ilk_en)

sonuc = {}
for ax in SECILI:
    print(f"\n=== {ax.upper()} ===")
    d = komut("eksen " + ax, yaz=False)
    m = re.search(r"oran=([\d.]+)\s+mikro=(\d+)\s+limit=(-?\d+)\.\.(-?\d+)", d)
    k = re.search(r"konum=(-?[\d.]+)", d)
    print(f"ayar: oran={m[1]} mikro={m[2]} limit={m[3]}..{m[4]}  konum={k[1]}")
    komut("tel 0", yaz=False)                           # tel testi açık kaldıysa STEP ve DIR pinlerini bırak
    komut("en 1")
    time.sleep(0.7)
    a = HAREKET[ax]
    if float(k[1]) + a > float(m[4]):                   # üst limite yakınsa öbür yöne git
        a = -a
    o1 = komut(f"git {a}", 6.0, "vardi")
    time.sleep(0.8)
    o2 = komut(f"git {-a}", 6.0, "vardi")
    sonuc[ax] = ("vardi" in o1) and ("vardi" in o2)
    time.sleep(0.5)
    if ilk_en.get(ax) == "0":
        komut("en 0")                                   # bulduğum gibi bırak
    time.sleep(1.5)

komut("eksen " + ilk_eksen, yaz=False)
print("\nozet (kartin darbe uretimi):", "  ".join(f"{a}={'tamam' if v else 'VARMADI'}" for a, v in sonuc.items()))
print(komut("durum", yaz=False).strip())
s.close()
