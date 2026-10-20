#!/usr/bin/env python3
"""Yaw kayması testi: python yaw_drift_test.py COM8 [saniye] [--log dosya.csv]
Yaw'ı sıfırlar, telemetriden roll/pitch/yaw kaydeder, sonunda kayma ve gürültü özetini yazar."""
import os, sys, time, math, statistics
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mercom_link as ml

def wrap(a):
    return (a + 180.0) % 360.0 - 180.0

def main():
    port = sys.argv[1]
    dur = float(sys.argv[2]) if len(sys.argv) > 2 and not sys.argv[2].startswith("--") else 300.0
    logfile = sys.argv[sys.argv.index("--log") + 1] if "--log" in sys.argv else "yaw_drift.csv"
    link = ml.Link(port)
    msgs = []
    orig = link._handle
    def handler(mid, payload):
        if mid == ml.MSG_TEXT:
            t = payload.decode("ascii", "replace"); msgs.append((time.time(), t)); print("   [kart]", t, flush=True)
        else:
            orig(mid, payload)
    link._handle = handler
    time.sleep(0.5)
    link.send(ml.MSG_YAW_ZERO)
    time.sleep(0.5)
    rows = []
    t0 = time.time(); last = None
    with open(logfile, "w") as f:
        f.write("t,roll,pitch,yaw\n")
        while time.time() - t0 < dur:
            time.sleep(0.02)
            d = dict(zip(ml.TELEM_COLS, link.last_telem)) if link.last_telem else None
            if not d or d is last: continue
            last = d
            t = time.time() - t0
            r, p, y = d["roll"], d["pitch"], wrap(d["yaw"])
            rows.append((t, r, p, y)); f.write(f"{t:.3f},{r:.3f},{p:.3f},{y:.3f}\n")
            if int(t) % 30 == 0 and (len(rows) < 2 or int(rows[-2][0]) % 30 != 0):
                print(f"{t:6.0f} s  roll={r:8.3f} pitch={p:8.3f} yaw={y:8.3f}", flush=True)
    link.close()
    if len(rows) < 100:
        print("yeterli veri yok:", len(rows)); return
    ts = [x[0] for x in rows]; ys = [x[3] for x in rows]
    n = len(ts); mt = sum(ts)/n; my = sum(ys)/n
    slope = sum((t-mt)*(y-my) for t,y in zip(ts,ys)) / sum((t-mt)**2 for t in ts)
    resid = [y - (my + slope*(t-mt)) for t,y in zip(ts,ys)]
    first = statistics.mean(ys[:200]); lastm = statistics.mean(ys[-200:])
    print(f"\nornek={n}  sure={ts[-1]:.0f} s  ({n/ts[-1]:.0f} Hz)")
    print(f"yaw baslangic={first:.3f}  bitis={lastm:.3f}  toplam kayma={lastm-first:.3f} deg")
    print(f"egim={slope*60:.4f} deg/dk  ->  5 dk'da {slope*300:.3f} deg,  gurultu std={statistics.pstdev(resid):.3f} deg")
    rs=[x[1] for x in rows]; ps=[x[2] for x in rows]
    print(f"roll ort={statistics.mean(rs):.3f} std={statistics.pstdev(rs):.3f}   pitch ort={statistics.mean(ps):.3f} std={statistics.pstdev(ps):.3f}")
    # donma: ayni deger 2 s'den uzun surerse
    frozen = 0.0; lastk = None; tk = ts[0]
    for x in rows:
        k = (x[1], x[2], x[3])
        if k != lastk: lastk = k; tk = x[0]
        elif x[0] - tk > 2.0: frozen = max(frozen, x[0] - tk)
    if frozen: print(f"UYARI: veri en uzun {frozen:.0f} s boyunca donuk kaldi (akis kesintisi)")
    else: print("akis kesintisiz")
    print("kayit:", logfile)

if __name__ == "__main__":
    main()
