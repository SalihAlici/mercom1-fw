# takip_kayit.csv dosyasını özetler: taban ne kadar eğildi (pan/tilt komutundan), nişan hatası, başlığın yön değişimi (paralaks düzeltmesi izi).
import csv, math, sys
OUT = r"C:\Users\Salih\Desktop\hareketli uydu\mercom1-fw\tools\takip_kayit.csv"
rows = []
with open(OUT, newline="") as fh:
    for r in csv.DictReader(fh):
        try: rows.append({k: float(v) for k, v in r.items()})
        except ValueError: pass
if not rows: print("kayit bos"); raise SystemExit
T = rows[-1]["t"]
print("kayit: %d ornek, %.0f s" % (len(rows), T))
auto = [d for d in rows if int(d["mode"]) == 2 and int(d["flags"]) & 1]
print("otomatik+kilitli ornek: %d" % len(auto))
if not auto: raise SystemExit
def unwrap(v, ref): return ref + ((v - ref + 180) % 360 - 180)
p0, t0_, y0, pi0 = auto[0]["cmd_pan"], auto[0]["cmd_tilt"], auto[0]["yaw"], auto[0]["pitch"]
e = [d["aim_err"] for d in auto]
print("nisan hatasi: ort %.2f  medyan %.2f  en fazla %.2f derece | <0.72: %%%.0f  <1.43: %%%.0f" % (
    sum(e) / len(e), sorted(e)[len(e) // 2], max(e), 100.0 * sum(1 for x in e if x < 0.72) / len(e), 100.0 * sum(1 for x in e if x < 1.43) / len(e)))
dp = [unwrap(d["cmd_pan"], p0) - p0 for d in auto]; dt = [d["cmd_tilt"] - t0_ for d in auto]
print("pan komutu degisimi %.2f..%.2f  tilt komutu degisimi %.2f..%.2f derece (tabanin hareketi)" % (min(dp), max(dp), min(dt), max(dt)))
dy = [unwrap(d["yaw"], y0) - y0 for d in auto]; dpi = [d["pitch"] - pi0 for d in auto]
print("baslik yonu (IMU) degisimi: yaw %.2f..%.2f  pitch %.2f..%.2f derece (paralaks duzeltmesinin izi + takip hatasi)" % (min(dy), max(dy), min(dpi), max(dpi)))
print("5 s'lik dilimler: pan/tilt komut farki | hata ort/en fazla | baslik yaw/pitch farki")
k = 0
while k * 5 < T:
    seg = [i for i, d in enumerate(auto) if k * 5 <= d["t"] < (k + 1) * 5]
    if seg:
        ee = [e[i] for i in seg]
        print("  %3d-%3d s: pan %+6.2f tilt %+6.2f | %.2f / %.2f | yaw %+5.2f pitch %+5.2f" % (
            k * 5, (k + 1) * 5, sum(dp[i] for i in seg) / len(seg), sum(dt[i] for i in seg) / len(seg),
            sum(ee) / len(ee), max(ee), sum(dy[i] for i in seg) / len(seg), sum(dpi[i] for i in seg) / len(seg)))
    k += 1
