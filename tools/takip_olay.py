# Kayıttaki yatırma olayını ayrıntılı döker: taban yatışı, IMU kesintileri, kart yeniden başlaması, yön sıçraması.
# Kullanım: takip_olay.py <csv> [adim_s]
import csv, math, sys
P = sys.argv[1]; ADIM = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
rows = []
with open(P, newline="") as fh:
    for r in csv.DictReader(fh):
        try: rows.append({k: float(v) for k, v in r.items()})
        except (ValueError, TypeError): pass
if not rows: print("bos"); raise SystemExit
un = lambda v, ref: ((v - ref + 180) % 360) - 180
print("%s: %d ornek, %.0f s, kart saati %.0f -> %.0f s" % (P.split("\\")[-1], len(rows), rows[-1]["t"], rows[0]["t_ms"] / 1000, rows[-1]["t_ms"] / 1000))
# kart yeniden baslamasi, IMU kesintisi
for a, b in zip(rows, rows[1:]):
    if b["t_ms"] < a["t_ms"] - 500: print("  !! KART YENIDEN BASLADI  t=%.1f s (kart saati %.0f -> %.0f ms)" % (b["t"], a["t_ms"], b["t_ms"]))
    if b["imu_count"] < a["imu_count"]: print("  !! imu sayaci geri gitti t=%.1f" % b["t"])
k = 0; kes = []
while k < len(rows):
    j = k
    while j + 1 < len(rows) and rows[j + 1]["imu_count"] == rows[k]["imu_count"]: j += 1
    if rows[j]["t"] - rows[k]["t"] > 0.25: kes.append((rows[k]["t"], rows[j]["t"]))
    k = j + 1
print("  IMU kesintisi (>0,25 s): %d adet, toplam %.1f s" % (len(kes), sum(b - a for a, b in kes)))
for a, b in kes[:40]: print("     %.1f - %.1f s (%.1f s)" % (a, b, b - a))
p0 = rows[0]["cmd_pan"]; t0 = rows[0]["cmd_tilt"]; y0 = rows[0]["yaw"]
print("  baslangic: cmd_pan %.2f cmd_tilt %.2f | IMU roll %.2f pitch %.2f yaw %.2f | mod %d bayrak %d" % (p0, t0, rows[0]["roll"], rows[0]["pitch"], rows[0]["yaw"], rows[0]["mode"], rows[0]["flags"]))
print("   t[s]  mod byr | pan kmt  tilt kmt | enk pan  enk tilt | IMU roll  pitch   yaw(f) | hata  yan   ust | imu/s")
nxt = 0.0; prev = rows[0]
for d in rows:
    if d["t"] < nxt: continue
    hz = (d["imu_count"] - prev["imu_count"]) / max(1e-3, d["t"] - prev["t"]) if d is not prev else 0
    print("  %6.1f  %d  %3d | %+7.2f  %+7.2f | %+7.2f  %+7.2f | %+7.2f %+7.2f %+7.2f | %.2f %+.2f %+.2f | %3.0f" % (
        d["t"], d["mode"], d["flags"], un(d["cmd_pan"], p0), d["cmd_tilt"] - t0, d["enc_pan"], d["enc_tilt"],
        d["roll"], d["pitch"], un(d["yaw"], y0), d["aim_err"], d["aim_side"], d["aim_up"], hz))
    prev = d; nxt = d["t"] + ADIM
