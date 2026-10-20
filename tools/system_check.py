#!/usr/bin/env python3
"""MERCOM-1 tam sistem denetimi: bütün bileşenleri sırayla sınar ve tek bir özet tablo yazar.

    python system_check.py COM8            her şey, motorlar dahil (her eksen birkaç derece oynar)
    python system_check.py COM8 --motorsuz motorları oynatmadan
    python system_check.py COM8 --sensor   yalnız enkoderler ve BNO085 (sürücü, motor, lazer yok)

Kartta ANA FIRMWARE yüklü olmalı. Sınananlar:
  kart ve protokol · 3 enkoder (haberleşme + mıknatıs) · BNO085 (akış hızı + reset) · 3 sürücü UART hattı ·
  3 motor (küçük ileri-geri hareket; enkoderden yön ve gerçek mikroadım, IMU'dan anten hareketi) · lazer
"""
import os
import re
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mercom_link as ml  # noqa: E402

AX = ("pan", "tilt", "ce")
MOVE_STEPS = {"pan": 80, "tilt": 80, "ce": 6}       # 8 mikroadımda: pan/tilt 18° motor = 3,6° anten; CE 1,35°
results = []                                        # (bileşen, durum, ayrıntı)


def wrapd(a):
    return (a + 180.0) % 360.0 - 180.0


def add(name, ok, detail):
    results.append((name, ok, detail))
    print("  %-22s %-6s %s" % (name, {True: "TAMAM", False: "SORUN", None: "ATLANDI"}[ok], detail), flush=True)


class Bench:
    def __init__(self, port):
        self.link = ml.Link(port)
        self.texts, self.acks = [], []
        orig = self.link._handle

        def handler(mid, payload):
            if mid == ml.MSG_TEXT:
                self.texts.append(payload.decode("ascii", "replace"))
            elif mid == ml.MSG_ACK:
                self.acks.append((payload[0], payload[1]))
            elif mid == ml.MSG_PARAM_VALUE:
                pid, val, ok = struct.unpack("<HfB", payload)
                if ok:
                    self.link.params[pid] = val
            elif mid == ml.MSG_PONG:
                self.pong = struct.unpack("<II", payload)
            else:
                orig(mid, payload)
        self.pong = None
        self.link._handle = handler
        time.sleep(0.4)

    def ask(self, mid, payload=b"", wait=0.6):
        """komut gönder, o sürede gelen metin satırlarını döndür"""
        n0 = len(self.texts)
        self.link.send(mid, payload)
        time.sleep(wait)
        return self.texts[n0:]

    def param(self, name):
        pid = ml.PARAM_NAMES.index(name)
        self.link.send(ml.MSG_PARAM_GET, struct.pack("<H", pid))
        time.sleep(0.15)
        return self.link.params.get(pid)

    def telem(self, secs=0.6):
        rows, t0, last = [], time.time(), None
        while time.time() - t0 < secs:
            time.sleep(0.02)
            t = self.link.last_telem
            if t and t is not last:
                last = t
                rows.append(dict(zip(ml.TELEM_COLS, t)))
        return rows

    def mean(self, rows, key):
        v = [r[key] for r in rows]
        return v[0] + sum(wrapd(x - v[0]) for x in v) / len(v)


def check_board(b):
    b.ask(ml.MSG_PING, wait=0.5)
    if not b.pong:
        add("kart / protokol", False, "PING cevabı yok: kartta ana firmware yüklü mü, COM portu doğru mu?")
        return False
    up, ver = b.pong
    add("kart / protokol", True, "firmware %d.%d.%d, açık kalma %.0f s" % (ver >> 16, (ver >> 8) & 255, ver & 255, up / 1000))
    b.link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 50))
    time.sleep(0.3)
    rows = b.telem(0.6)
    add("telemetri", bool(rows), "%d paket / 0,6 s, döngü %d µs" % (len(rows), rows[-1]["loop_us"]) if rows else "paket gelmiyor")
    return True


def parse_enc(lines):
    out = {}
    for t in lines:
        m = re.match(r"enc (\w+): raw=(\d+) agc=(\d+) OCF=(\d) COF=(\d) yakin=(\d) uzak=(\d) ok=(\d+) err=(\d+)", t)
        if m:
            out[m[1]] = dict(raw=int(m[2]), agc=int(m[3]), ocf=int(m[4]), cof=int(m[5]), yakin=int(m[6]), uzak=int(m[7]), ok=int(m[8]), err=int(m[9]))
    return out


def check_encoders(b):
    e1 = parse_enc(b.ask(ml.MSG_ENC_DIAG, wait=0.7))
    time.sleep(2.0)
    e2 = parse_enc(b.ask(ml.MSG_ENC_DIAG, wait=0.7))
    health = {}
    for a in AX:
        if a not in e1 or a not in e2:
            add("enkoder " + a, False, "tanı satırı gelmedi")
            health[a] = False
            continue
        dok, derr = e2[a]["ok"] - e1[a]["ok"], e2[a]["err"] - e1[a]["err"]
        d = e2[a]
        if dok < 100:
            add("enkoder " + a, False, "cevap yok (2 s'de %d geçerli okuma): CS teli, SCK/MISO/MOSI köprüsü, 5V ve 3.3V pinlerinin ikisi de 3V3'te mi" % dok)
            health[a] = False
        elif derr > 0.02 * (dok + derr):
            add("enkoder " + a, False, "haberleşme hatalı: %d hata / %d okuma; kablo teması ya da eksik besleme pini" % (derr, dok + derr))
            health[a] = False
        elif d["uzak"] or not d["ocf"]:
            add("enkoder " + a, False, "haberleşme temiz ama mıknatıs UZAK (AGC %d, OCF %d): aralık 1-2 mm olmalı" % (d["agc"], d["ocf"]))
            health[a] = False
        elif d["yakin"]:
            add("enkoder " + a, True, "çalışıyor ama mıknatıs çok YAKIN (AGC %d), doğruluk düşer" % d["agc"])
            health[a] = True
        else:
            add("enkoder " + a, True, "AGC %d, %d okuma/s, hata %d" % (d["agc"], dok // 2, derr))
            health[a] = True
    return health


def check_imu(b):
    def diag():
        for t in b.ask(ml.MSG_IMU_DIAG, wait=0.6):
            m = re.match(r"imu: (\w+), rapor=(\d+), reset=(\d+)", t)
            if m:
                return m[1], int(m[2]), int(m[3])
        return None
    d1 = diag()
    if not d1 or d1[0] != "hazir":
        b.ask(ml.MSG_IMU_INIT, wait=4.0)
        d1 = diag()
    if not d1 or d1[0] != "hazir":
        add("BNO085 IMU", False, "bulunamadı: VIN->3V3, GND, SCL->D15, SDA->D14; modülün gücünü kesip yeniden ver")
        return False
    time.sleep(3.0)
    d2 = diag()
    rate = (d2[1] - d1[1]) / 3.6 if d2 else 0
    resets = (d2[2] - d1[2]) if d2 else 0
    ok = rate > 80 and resets == 0
    add("BNO085 IMU", ok, "%.0f rapor/s, bu sürede reset %d (toplam %d)" % (rate, resets, d2[2] if d2 else -1)
        + ("" if ok else "  -> besleme/kablo"))
    return ok


def check_tmc_uart(b):
    lines = b.ask(ml.MSG_TMC_TXCHK, wait=1.0)
    for t in lines:
        m = re.match(r"uart (\w+) (\w+): verici (\w+)", t)
        if m:
            add("UART vericisi " + m[1], m[3] == "OK", "pin %s: %s" % (m[2], t.split(": ", 1)[1]))
    if not any(t.startswith("uart ") for t in lines):
        add("UART vericisi", False, "sınama cevabı gelmedi (firmware eski olabilir)")
    lines = b.ask(ml.MSG_TMC_INIT, wait=2.5) + b.ask(ml.MSG_TMC_DIAG, wait=1.2)
    for a in AX:
        t = next((x for x in reversed(lines) if x.startswith("tmc %s" % a)), None)
        if t is None:
            add("sürücü UART " + a, None, "cevap satırı yok")
        elif "hatta yok" in t or "cevap yok" in t:
            add("sürücü UART " + a, False, "sürücü cevap vermiyor: 24 V ve VIO var mı, UART teli modülün UART pininde mi (motor yine STEP/DIR ile döner)")
        else:
            add("sürücü UART " + a, True, t.split(": ", 1)[1][:70])


def check_motor(b, axis, enc_ok, imu_ok):
    ai = ml.AXES[axis]
    n = MOVE_STEPS[axis]
    ratio = b.param("enc_ratio_" + axis) or (1.0 if axis == "ce" else 5.0)
    b.link.send(ml.MSG_SET_MODE, bytes([1])); time.sleep(0.15)
    b.link.send(ml.MSG_ENABLE, bytes([ai, 1])); time.sleep(0.5)
    before = b.telem(0.6)
    if not before:
        add("motor " + axis, False, "telemetri yok")
        return
    b.acks.clear()
    b.link.send(ml.MSG_STEP_MOVE, struct.pack("<Bif", ai, n, 200.0))
    time.sleep(n / 200.0 + 1.2)
    mid = b.telem(0.6)
    b.link.send(ml.MSG_STEP_MOVE, struct.pack("<Bif", ai, -n, 200.0))
    time.sleep(n / 200.0 + 1.2)
    after = b.telem(0.4)
    if any(st != 0 for _, st in b.acks):
        add("motor " + axis, False, "komut reddedildi: " + ", ".join(ml.ACK.get(st, str(st)) for _, st in b.acks if st))
        return
    notes, moved = [], None
    if enc_ok:
        d_out = wrapd(b.mean(mid, "enc_" + axis) - b.mean(before, "enc_" + axis))
        back = wrapd(b.mean(after, "enc_" + axis) - b.mean(before, "enc_" + axis))
        d_motor = d_out * ratio
        if abs(d_motor) < 1.0 and axis != "ce" or abs(d_motor) < 0.08:
            moved = False
            notes.append("enkoder hareket görmedi (%.2f° motor)" % d_motor)
        else:
            moved = True
            micro = n * 1.8 / abs(d_motor)
            near = min((8, 16, 32, 64), key=lambda m: abs(m - micro))
            notes.append("motor %+.2f° döndü -> mikroadım ~ %.1f (%d), enkoder yönü %s, geri dönüş farkı %.2f°"
                         % (d_motor, micro, near, "aynı" if d_motor > 0 else "TERS", back * ratio))
    if imu_ok and axis in ("pan", "tilt"):
        key = "yaw" if axis == "pan" else "pitch"
        d_imu = wrapd(b.mean(mid, key) - b.mean(before, key))
        notes.append("IMU %s %+.2f° (anten)" % (key, d_imu))
        if moved is None:
            moved = abs(d_imu) > 0.5
        if moved and enc_ok and abs(d_imu) > 0.5:
            notes.append("oran ~ %.2f" % abs(wrapd(b.mean(mid, "enc_" + axis) - b.mean(before, "enc_" + axis)) * ratio / d_imu))
    if moved is None:
        add("motor " + axis, None, "hareket doğrulanamadı: bu eksenin enkoderi ve IMU sağlıklı değil; gözle bak")
    else:
        add("motor " + axis, moved, "; ".join(notes) if moved else "; ".join(notes) + " -> 24 V, VIO >= 3,0 V, EN teli, STEP/DIR telleri, Vref")


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    port = args[0] if args else "COM8"
    motors = "--motorsuz" not in sys.argv
    print("MERCOM-1 sistem denetimi, port %s%s\n" % (port, "" if motors else " (motorlar oynatılmayacak)"))
    try:
        b = Bench(port)
    except Exception as e:
        print("%s açılamadı: kart USB'de mi, başka bir pencere (arayüz, terminal, monitör) portu tutuyor mu?" % port)
        print("  ayrıntı: %s" % e)
        return
    try:
        if not check_board(b):
            return
        enc_ok = check_encoders(b)
        imu_ok = check_imu(b)
        if "--sensor" not in sys.argv:                      # --sensor: yalnız enkoder + IMU; sürücü, motor ve lazere dokunma
            check_tmc_uart(b)
            if motors:
                for a in AX:
                    try:
                        check_motor(b, a, enc_ok.get(a, False), imu_ok)
                    finally:                                # sınanan sürücüyü açık bırakma (CE duruşta 1,1 A çekiyor)
                        b.link.send(ml.MSG_ENABLE, bytes([ml.AXES[a], 0])); time.sleep(0.2)
                b.link.send(ml.MSG_STOP)
            else:
                for a in AX:
                    add("motor " + a, None, "--motorsuz")
            b.link.send(ml.MSG_LASER, bytes([1])); time.sleep(1.0); b.link.send(ml.MSG_LASER, bytes([0]))
            add("lazer", None, "1 s yakıldı; gözle doğrula")
    finally:
        b.link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 20))
        time.sleep(0.1)
        b.link.close()
    bad = [r for r in results if r[1] is False]
    print("\nÖZET: %d tamam, %d sorun, %d atlandı" % (sum(r[1] is True for r in results), len(bad), sum(r[1] is None for r in results)))
    for name, _, detail in bad:
        print("  SORUN  %s: %s" % (name, detail))


if __name__ == "__main__":
    main()
