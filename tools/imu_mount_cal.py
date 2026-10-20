#!/usr/bin/env python3
"""IMU montaj kalibrasyonu, iki adım.

  python imu_mount_cal.py COM8 a     anten sıfır konumunda (yatay bakıyor, pan sıfır): yerçekimi ve quaternion kaydedilir
  python imu_mount_cal.py COM8 b     anten elle ~30° yukarı eğilmiş: quaternion kaydedilir, montaj matrisi hesaplanır,
                                     imu_mount_r/p/y parametreleri karta yazılır ve imu_mount.json'a kaydedilir

Montaj matrisi R_IH: başlık çerçevesindeki bir vektörü IMU çerçevesine çevirir.
Başlık çerçevesi: x boresight (anten yatay bakarken ileri), z yukarı, y sol.
  z_H = sıfır konumunda yerçekimi yönü (IMU çerçevesinde, yukarı)
  y_H = yukarı eğme hareketinin dönme ekseninin tersi (yukarı eğmek −y etrafında dönmedir)
  x_H = y_H × z_H
Parametreler R_IH = Rz(yaw)·Ry(pitch)·Rx(roll) olacak şekilde Euler açısı olarak saklanır.
"""
import json
import math
import os
import re
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mercom_link as ml  # noqa: E402

STATE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "imu_mount.json")
R2D = 180.0 / math.pi


def capture(link, n=10):
    """imudiag'ı n kez okuyup quaternion ve yerçekimi ortalaması döndürür."""
    qs, gs = [], []
    texts = []
    orig = link._handle

    def handler(mid, payload):
        if mid == ml.MSG_TEXT:
            texts.append(payload.decode("ascii", "replace"))
        else:
            orig(mid, payload)

    link._handle = handler
    for _ in range(n):
        link.send(ml.MSG_IMU_DIAG)
        time.sleep(0.25)
    link._handle = orig
    for t in texts:
        m = re.search(r"quat x10000: w=(-?\d+) x=(-?\d+) y=(-?\d+) z=(-?\d+)", t)
        if m:
            qs.append([int(v) / 10000.0 for v in m.groups()])
        m = re.search(r"yercekimi \(cm/s2\): x=(-?\d+) y=(-?\d+) z=(-?\d+)", t)
        if m:
            gs.append([int(v) / 100.0 for v in m.groups()])
    if not qs or not gs:
        sys.exit("imu cevap vermedi; imuinit deneyin")
    q = [sum(c) / len(qs) for c in zip(*qs)]
    g = [sum(c) / len(gs) for c in zip(*gs)]
    n_ = math.sqrt(sum(v * v for v in q))
    q = [v / n_ for v in q]
    spread = max(max(c) - min(c) for c in zip(*qs))
    return q, g, spread


def qconj(q):
    return [q[0], -q[1], -q[2], -q[3]]


def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return [w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2]


def norm(v):
    n = math.sqrt(sum(c * c for c in v))
    return [c / n for c in v]


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def euler_from_R(R):
    """R = Rz(yaw)·Ry(pitch)·Rx(roll) için açılar, derece."""
    pitch = math.asin(max(-1.0, min(1.0, -R[2][0])))
    roll = math.atan2(R[2][1], R[2][2])
    yaw = math.atan2(R[1][0], R[0][0])
    return roll * R2D, pitch * R2D, yaw * R2D


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return
    port, step = sys.argv[1], sys.argv[2].lower()
    link = ml.Link(port)
    time.sleep(0.3)

    if step == "a":
        q, g, spread = capture(link)
        json.dump({"qA": q, "gA": g}, open(STATE, "w"))
        gn = norm(g)
        print(f"sifir konumu kaydedildi. yercekimi (IMU cerc.) x={gn[0]:+.3f} y={gn[1]:+.3f} z={gn[2]:+.3f}  |g|={math.sqrt(dot(g, g)):.2f}")
        print(f"quaternion w={q[0]:+.4f} x={q[1]:+.4f} y={q[2]:+.4f} z={q[3]:+.4f}  (10 okuma yayilimi {spread:.4f}, 0.002 alti iyi)")
        up = max(range(3), key=lambda i: abs(gn[i]))
        print(f"yukari bakan IMU ekseni: {'xyz'[up]}{'+' if gn[up] > 0 else '-'}")
        print("simdi anteni elle yaklasik 30 derece YUKARI egip sabit tutun ve 'b' adimini calistirin")
    elif step == "b":
        if not os.path.exists(STATE):
            sys.exit("once 'a' adimi")
        st = json.load(open(STATE))
        qA, gA = st["qA"], st["gA"]
        qB, gB, spread = capture(link)
        # A'dan B'ye goreli donme, A anindaki IMU cercevesinde
        qrel = qmul(qconj(qA), qB)
        ang = 2.0 * math.atan2(math.sqrt(sum(c * c for c in qrel[1:])), qrel[0])
        if ang > math.pi:
            ang -= 2 * math.pi
        if abs(ang) < 5 / R2D:
            sys.exit(f"donme sadece {ang * R2D:.1f} derece; anteni daha fazla egin")
        axis = norm(qrel[1:])
        if ang < 0:
            axis = [-c for c in axis]
        zH = norm(gA)
        yH = [-c for c in axis]                     # yukari egmek -y etrafinda
        # z'ye dik hale getir
        yH = norm([yH[i] - dot(yH, zH) * zH[i] for i in range(3)])
        xH = cross(yH, zH)
        tilt_axis_err = math.degrees(math.acos(max(-1, min(1, abs(dot(norm(axis), zH))))))
        # R_IH sutunlari: xH yH zH (IMU cercevesinde)
        R = [[xH[i], yH[i], zH[i]] for i in range(3)]
        r, p, y = euler_from_R(R)
        print(f"egme acisi {abs(ang) * R2D:.1f} derece, egme ekseni ile yercekimi arasi {tilt_axis_err:.1f} derece (90'a yakin olmali)")
        print(f"baslik x (boresight) IMU'da: {xH[0]:+.3f} {xH[1]:+.3f} {xH[2]:+.3f}")
        print(f"baslik y (sol)       IMU'da: {yH[0]:+.3f} {yH[1]:+.3f} {yH[2]:+.3f}")
        print(f"baslik z (yukari)    IMU'da: {zH[0]:+.3f} {zH[1]:+.3f} {zH[2]:+.3f}")
        print(f"imu_mount_r={r:.3f} imu_mount_p={p:.3f} imu_mount_y={y:.3f}")
        for name, val in (("imu_mount_r", r), ("imu_mount_p", p), ("imu_mount_y", y)):
            link.send(ml.MSG_PARAM_SET, struct.pack("<Hf", ml.PARAM_NAMES.index(name), val))
            time.sleep(0.1)
        st.update({"qB": qB, "R_IH": R, "mount_rpy_deg": [r, p, y]})
        json.dump(st, open(STATE, "w"), indent=1)
        print(f"karta yazildi ve {STATE} dosyasina kaydedildi (flash kaydi henuz yok, acilista tekrar yuklenecek)")
    else:
        print(__doc__)
    link.close()


if __name__ == "__main__":
    main()
