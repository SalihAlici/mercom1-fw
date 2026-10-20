# Tezgah benzetimi 2: montaj açısı doğru; taban elle sola yatırılıyor (dönme ekseni tabanın sol alt kenarı).
# Yazılımın paralaks modeli (dönme merkezi pan ekseni üzerinde, tilt ekseninin plx_ht altında) gerçek geometriyle
# karşılaştırılır: paralaks kapalı / açık iken duvardaki nokta kaç cm kayar?
import numpy as np
def Rx(a): c, s = np.cos(a), np.sin(a); return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
def Ry(a): c, s = np.cos(a), np.sin(a); return np.array([[c, -0 * s, s], [0, 1, 0], [-s, 0, c]])
def Rz(a): c, s = np.cos(a), np.sin(a); return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

E0 = np.radians(27.47)                     # ışının gerçek yükselişi (ölçüldü)
WALL = 2.00                                # lazer ucundan duvara yatay mesafe
HTILT = 0.19                               # tilt ekseninin zeminden yüksekliği (lazer ucu 28 cm'den geri hesap)
LH_REAL = np.array([0.10, -0.13, 0.05])    # lazerin tilt ekseni noktasına göre konumu, başlık çerçevesi

def kayma(r_deg, yarim_gen, plx, ht, ly, d, lx=0.10, lz=0.05):
    r = np.radians(r_deg)
    piv = np.array([0.0, yarim_gen, 0.0])                       # sol alt kenar
    lfw = np.array([lx, ly, lz])
    def gercek(RWB, pan, tau):
        RBH = Rz(pan) @ Ry(-tau)
        pB = np.array([0, 0, HTILT]) + RBH @ LH_REAL
        return piv + RWB @ (pB - piv), RWB @ RBH
    def fw_L(RWH, tau): return RWH @ (ht * np.array([np.sin(tau), 0, np.cos(tau)]) + lfw)
    L0, R0 = gercek(np.eye(3), 0.0, E0)
    X = L0[0] + WALL
    d0 = R0[:, 0]
    s0 = L0 + (X - L0[0]) / d0[0] * d0
    TW = fw_L(R0, E0) + d * d0                                   # kilit
    RWB = Rx(-r)
    pan, tau = 0.0, E0
    for _ in range(40):                                          # kapalı çevrimin sabit noktası
        RWH = RWB @ Rz(pan) @ Ry(-tau)
        dref = d0
        if plx:
            v = TW - fw_L(RWH, tau); dref = v / np.linalg.norm(v)
        dB = RWB.T @ dref
        pan = np.arctan2(dB[1], dB[0]); tau = np.arctan2(dB[2], np.hypot(dB[0], dB[1]))
    L1, R1 = gercek(RWB, pan, tau)
    b = R1[:, 0]
    s1 = L1 + (X - L1[0]) / b[0] * b
    return (s1[1] - s0[1]) * 100, (s1[2] - s0[2]) * 100

if __name__ == "__main__":
    D = float(np.hypot(2.00, 1.04))
    print("hedef mesafesi (lazerden noktaya) %.3f m | + = sola / yukari, cm" % D)
    for yg in (0.08, 0.12, 0.16):
        print("\ndonme kenari pan ekseninden %.0f cm solda" % (yg * 100))
        print("  yatis | paralaks KAPALI   | ACIK, tezgah ayari (ht 0,19 ly -0,13) | ACIK, ly kenara gore (-0,13-kenar) | ACIK, ly isareti TERS (+0,13)")
        for r in (8, 13, 18):
            a = kayma(r, yg, 0, 0.19, -0.13, D)
            b = kayma(r, yg, 1, 0.19, -0.13, D)
            c = kayma(r, yg, 1, 0.19, -0.13 - yg, D)
            e = kayma(r, yg, 1, 0.19, +0.13, D)
            print("  %3d   | %+5.1f / %+5.1f     | %+5.1f / %+5.1f                        | %+5.1f / %+5.1f                     | %+5.1f / %+5.1f" % (r, *a, *b, *c, *e))
