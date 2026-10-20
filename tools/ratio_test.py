#!/usr/bin/env python3
"""Redüktör oranı ve adım doğrulama testi.
    python ratio_test.py COM8 tilt 3200 [adım/s]
Motoru tam N mikroadım döndürür; öncesi/sonrası enkoder ve IMU açısını karşılaştırır.
  - enkoder (motor mili): adım kaçırma ve yön kontrolü
  - IMU (başlık): çıkış açısı -> gerçek redüktör oranı = motor derecesi / IMU derecesi
Motorlar 'en on' ile etkin ve mod MANUAL olmalı; betik bunu kendisi yapar."""
import os, sys, time, struct, statistics
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mercom_link as ml

AX = {"pan": 0, "tilt": 1, "ce": 2}
RATIO_PID = {"pan": ml.PARAM_NAMES.index("enc_ratio_pan"), "tilt": ml.PARAM_NAMES.index("enc_ratio_tilt"), "ce": ml.PARAM_NAMES.index("enc_ratio_ce")}

def wrapd(a):
    return (a + 180.0) % 360.0 - 180.0

def sample(link, secs):
    rows = []; t0 = time.time(); last = None
    while time.time() - t0 < secs:
        time.sleep(0.02)
        if link.last_telem and link.last_telem is not last:
            last = link.last_telem; rows.append(dict(zip(ml.TELEM_COLS, last)))
    return rows

def mean_of(rows, key):
    v = [r[key] for r in rows]
    # 0-360 sarmalı ortalama (küçük dağılım varsayımı)
    ref = v[0]; return ref + statistics.mean(wrapd(x - ref) for x in v)

def main():
    port, axn = sys.argv[1], sys.argv[2]
    steps = int(sys.argv[3]); sps = float(sys.argv[4]) if len(sys.argv) > 4 else 800.0
    ax = AX[axn]
    link = ml.Link(port); time.sleep(0.3)
    msgs = []
    orig = link._handle
    def h(mid, payload):
        if mid == ml.MSG_TEXT: msgs.append(payload.decode("ascii", "replace"))
        elif mid == ml.MSG_PARAM_VALUE:
            pid, val, ok = struct.unpack("<HfB", payload)
            if ok: link.params[pid] = val
        elif mid == ml.MSG_ACK:
            rid, st = payload
            if st != 0: msgs.append(f"ack 0x{rid:02X}: {ml.ACK.get(st, st)}")
        else: orig(mid, payload)
    link._handle = h
    link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 50)); time.sleep(0.1)
    link.send(ml.MSG_SET_MODE, bytes([1])); time.sleep(0.1)
    link.send(ml.MSG_ENABLE, bytes([1])); time.sleep(0.3)
    # parametreler
    vals = {}
    for name in ("mot_usteps", f"enc_ratio_{axn}", f"dps_{axn}", f"mot_dir_{axn}", f"enc_dir_{axn}"):
        pid = ml.PARAM_NAMES.index(name)
        link.send(ml.MSG_PARAM_GET, struct.pack("<H", pid)); time.sleep(0.15)
        vals[name] = link.params.get(pid)
    usteps = vals.get("mot_usteps") or 16.0
    ratio = vals.get(f"enc_ratio_{axn}") or 1.0
    motor_deg_cmd = steps * 1.8 / usteps
    print(f"eksen={axn} adim={steps} ({motor_deg_cmd:.1f} motor derecesi, {motor_deg_cmd/ratio:.2f} cikis derecesi enc_ratio={ratio}) hiz={sps} adim/s")
    before = sample(link, 1.0)
    if not before: print("telemetri yok"); link.close(); return
    b_enc = mean_of(before, f"enc_{axn}"); b_rpy = [mean_of(before, k) for k in ("roll", "pitch", "yaw")]
    imu_ok = before[-1]["imu_count"] > 0
    print(f"once : enc={b_enc:8.3f}  roll={b_rpy[0]:8.3f} pitch={b_rpy[1]:8.3f} yaw={b_rpy[2]:8.3f}")
    link.send(ml.MSG_STEP_MOVE, struct.pack("<Bif", ax, steps, sps))
    t_move = abs(steps) / sps
    t0 = time.time()
    while time.time() - t0 < t_move + 3.0:
        time.sleep(0.05)
        d = dict(zip(ml.TELEM_COLS, link.last_telem)) if link.last_telem else None
        if d and time.time() - t0 > 0.3 and abs(d[f"vel_{axn}"]) < 1e-6: break
    time.sleep(1.0)
    after = sample(link, 1.0)
    a_enc = mean_of(after, f"enc_{axn}"); a_rpy = [mean_of(after, k) for k in ("roll", "pitch", "yaw")]
    print(f"sonra: enc={a_enc:8.3f}  roll={a_rpy[0]:8.3f} pitch={a_rpy[1]:8.3f} yaw={a_rpy[2]:8.3f}")
    d_enc_out = wrapd(a_enc - b_enc)           # cikis derecesi (enc / ratio)
    d_enc_motor = d_enc_out * ratio
    d_rpy = [wrapd(a - b) for a, b in zip(a_rpy, b_rpy)]
    print()
    print(f"enkoder: motor {d_enc_motor:+.2f} deg olctu, komut {motor_deg_cmd:+.2f} deg -> fark {d_enc_motor - motor_deg_cmd:+.2f} deg"
          f"  ({'yon TERS: mot_dir ya da enc_dir -1' if d_enc_motor * motor_deg_cmd < 0 else 'yon ayni'})")
    if imu_ok:
        k = max(range(3), key=lambda i: abs(d_rpy[i]))
        nm = ("roll", "pitch", "yaw")[k]
        print(f"IMU: roll {d_rpy[0]:+.2f} pitch {d_rpy[1]:+.2f} yaw {d_rpy[2]:+.2f}  -> en cok {nm} degisti: {d_rpy[k]:+.2f} deg")
        if abs(d_rpy[k]) > 0.5:
            print(f"GERCEK ORAN = motor / cikis = {abs(d_enc_motor):.2f} / {abs(d_rpy[k]):.2f} = {abs(d_enc_motor)/abs(d_rpy[k]):.2f}   (parametre enc_ratio_{axn}={ratio})")
        else:
            print("IMU degisimi kucuk, oran hesaplanamadi (daha fazla adim ver)")
    else:
        print("IMU yok: oran icin IMU gerekli, enkoder yalniz motor milini gorur")
    for m in msgs: print("   [kart]", m)
    link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 20)); time.sleep(0.1); link.close()

if __name__ == "__main__":
    main()
