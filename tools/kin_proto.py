#!/usr/bin/env python3
"""MERCOM-1 kinematik prototipi. Bağımlılık yok, düz Python.

Ne doğrular:
  1. Kilit anında saklanan dünya doğrultusu d_N, sonra her anda
     d_B = R_NBᵀ · d_N  ->  pan = atan2, tilt = atan2  çözümü hedefi tam tutuyor mu.
  2. Jiroskop tabanlı ileri besleme formülleri sayısal türevle uyuşuyor mu:
        tilt_dot = wy·cos(pan) − wx·sin(pan)
        pan_dot  = −wz + tan(tilt)·(wx·cos(pan) + wy·sin(pan))
  3. Karşılaştırma: rapordaki "sanal roll" formülü hangi azimutta ne kadar hata verir.
  4. C kodu için altın vektör dosyası üretir: tools/kin_golden.csv

Eksen kuralları (kodda da aynı):
  Taban çerçevesi B: z pan ekseni boyunca yukarı, x pan=0 iken anten yönü, sağ el kuralı.
  pan yukarıdan bakınca saat yönünün tersi pozitif, tilt yukarı pozitif.
  Platform yönelimi R_NB = Rz(yaw)·Ry(pitch)·Rx(roll), IMU'dan gelen quaternion bunu verir.
"""
import csv
import math
import os

D2R = math.pi / 180.0
R2D = 180.0 / math.pi


# ---- küçük matris yardımcıları ----
def Rx(a):
    c, s = math.cos(a), math.sin(a)
    return [[1, 0, 0], [0, c, -s], [0, s, c]]


def Ry(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, 0, s], [0, 1, 0], [-s, 0, c]]


def Rz(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


def mm(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mv(A, v):
    return [sum(A[i][k] * v[k] for k in range(3)) for i in range(3)]


def T(A):
    return [[A[j][i] for j in range(3)] for i in range(3)]


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def angle_between(a, b):
    c = max(-1.0, min(1.0, dot(a, b) / (math.sqrt(dot(a, a)) * math.sqrt(dot(b, b)))))
    return math.acos(c)


def R_NB(roll, pitch, yaw):
    return mm(Rz(yaw), mm(Ry(pitch), Rx(roll)))


# ---- terminal kinematiği ----
def boresight_B(pan, tilt):
    return [math.cos(tilt) * math.cos(pan), math.cos(tilt) * math.sin(pan), math.sin(tilt)]


def solve_pan_tilt(dB):
    pan = math.atan2(dB[1], dB[0])
    tilt = math.atan2(dB[2], math.hypot(dB[0], dB[1]))
    return pan, tilt


def lock(R0, pan0, tilt0):
    """Kilit: o anki platform yönelimi ve eksen açılarından dünya doğrultusu."""
    return mv(R0, boresight_B(pan0, tilt0))


def track(R, dN):
    """Otomatik mod: platform yönelimi R ve hedef d_N'den eksen komutları."""
    return solve_pan_tilt(mv(T(R), dN))


def feedforward(pan, tilt, w):
    wx, wy, wz = w
    tilt_dot = wy * math.cos(pan) - wx * math.sin(pan)
    pan_dot = -wz + math.tan(tilt) * (wx * math.cos(pan) + wy * math.sin(pan))
    return pan_dot, tilt_dot


def body_rate(R_prev, R_next, dt):
    """ω_B = vee(Rᵀ·Ṙ), sayısal türevle."""
    Rdot = [[(R_next[i][j] - R_prev[i][j]) / dt for j in range(3)] for i in range(3)]
    R_mid = [[(R_next[i][j] + R_prev[i][j]) / 2 for j in range(3)] for i in range(3)]
    S = mm(T(R_mid), Rdot)
    return [S[2][1], S[0][2], S[1][0]]


# ---- platform hareketi: şartname zarfı ----
def platform(t, period=10.0, roll_amp=8.0, pitch_amp=8.0, yaw_amp=5.0):
    w = 2 * math.pi / period
    roll = roll_amp * D2R * math.sin(w * t)
    pitch = pitch_amp * D2R * math.sin(w * t + 1.3)
    yaw = yaw_amp * D2R * math.sin(w * t + 2.1)
    return roll, pitch, yaw


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def main():
    # Kilit: platform düz, anten pan0 azimutunda, 30° elevasyonda
    pan0, tilt0 = 40.0 * D2R, 30.0 * D2R
    dN = lock(R_NB(0, 0, 0), pan0, tilt0)

    dt = 0.001
    t_end = 20.0
    n = int(t_end / dt)
    max_point_err = 0.0
    max_ff_err_pan = 0.0
    max_ff_err_tilt = 0.0
    max_pan_rate = 0.0
    max_tilt_rate = 0.0
    prev = None
    rows = []
    for k in range(n + 1):
        t = k * dt
        roll, pitch, yaw = platform(t)
        R = R_NB(roll, pitch, yaw)
        pan, tilt = track(R, dN)

        # 1. hedef gerçekten tutuluyor mu
        err = angle_between(mv(R, boresight_B(pan, tilt)), dN)
        max_point_err = max(max_point_err, err)

        # 2. ileri besleme, sayısal türevle karşılaştırma
        if prev is not None:
            Rp, panp, tiltp = prev
            w = body_rate(Rp, R, dt)
            pan_dot_num = wrap(pan - panp) / dt
            tilt_dot_num = (tilt - tiltp) / dt
            pan_mid, tilt_mid = wrap(panp + wrap(pan - panp) / 2), (tilt + tiltp) / 2
            pan_dot_an, tilt_dot_an = feedforward(pan_mid, tilt_mid, w)
            max_ff_err_pan = max(max_ff_err_pan, abs(pan_dot_an - pan_dot_num))
            max_ff_err_tilt = max(max_ff_err_tilt, abs(tilt_dot_an - tilt_dot_num))
            max_pan_rate = max(max_pan_rate, abs(pan_dot_num))
            max_tilt_rate = max(max_tilt_rate, abs(tilt_dot_num))
            if k % 10 == 0:
                rows.append([f"{t:.3f}", f"{roll * R2D:.6f}", f"{pitch * R2D:.6f}", f"{yaw * R2D:.6f}",
                             f"{pan * R2D:.6f}", f"{tilt * R2D:.6f}",
                             f"{pan_dot_an * R2D:.6f}", f"{tilt_dot_an * R2D:.6f}"])
        prev = (R, pan, tilt)

    print("1) Geometrik çözüm, 20 s şartname hareketi:")
    print(f"   en büyük yönelim hatası      : {max_point_err * R2D:.2e} derece  (sıfır olmalı, sadece yuvarlama)")
    print(f"   en büyük pan hızı            : {max_pan_rate * R2D:.2f} derece/s")
    print(f"   en büyük tilt hızı           : {max_tilt_rate * R2D:.2f} derece/s")
    print("2) İleri besleme formülleri ile sayısal türev farkı:")
    print(f"   pan_dot  farkı en büyük      : {max_ff_err_pan * R2D:.2e} derece/s")
    print(f"   tilt_dot farkı en büyük      : {max_ff_err_tilt * R2D:.2e} derece/s")
    print("   (1e-3 altı: formüller ve işaretler doğru)")

    # 3. rapordaki sanal roll formülü: dPan = dRoll·sin(El), dTilt = dRoll·cos(El)
    print("3) 6° roll bozucusunda 'sanal roll' formülünün yönelim hatası, anten azimutuna göre:")
    roll6 = 6.0 * D2R
    print("   azimut   exact_dpan  exact_dtilt   formül_dpan formül_dtilt   hata")
    for az_deg in (0, 30, 60, 90, 135, 180, 270):
        az = az_deg * D2R
        dN_az = lock(R_NB(0, 0, 0), az, tilt0)
        pan_e, tilt_e = track(R_NB(roll6, 0, 0), dN_az)
        dpan_e, dtilt_e = wrap(pan_e - az), tilt_e - tilt0
        dpan_f, dtilt_f = roll6 * math.sin(tilt0), roll6 * math.cos(tilt0)
        # formül uygulanınca anten nereye bakar, hedeften kaç derece sapar
        err_f = angle_between(mv(R_NB(roll6, 0, 0), boresight_B(az + dpan_f, tilt0 + dtilt_f)), dN_az)
        print(f"   {az_deg:5d}°  {dpan_e * R2D:9.3f}  {dtilt_e * R2D:10.3f}   {dpan_f * R2D:10.3f} {dtilt_f * R2D:11.3f}   {err_f * R2D:6.3f}°")

    # 4. altın vektörler
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "kin_golden.csv")
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "roll_deg", "pitch_deg", "yaw_deg", "pan_deg", "tilt_deg", "pan_dot_dps", "tilt_dot_dps"])
        w.writerows(rows)
    print(f"4) Altın vektörler yazıldı: {out} ({len(rows)} satır, 10 ms aralık)")


if __name__ == "__main__":
    main()
