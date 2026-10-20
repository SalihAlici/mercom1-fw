# Tezgah benzetimi: taban elle sola yatırılınca (dönme ekseni tabanın sol alt kenarı) duvardaki nokta ne kadar kayar?
# Katkılar ayrı ayrı: (1) montaj eğimi hatası, (2) lazerin yer değiştirmesi (paralaks). Paralaks düzeltmesi KAPALI (yön tutuluyor).
import numpy as np, sys
def Rx(a): c, s = np.cos(a), np.sin(a); return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
def Ry(a): c, s = np.cos(a), np.sin(a); return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
def Rz(a): c, s = np.cos(a), np.sin(a); return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
lH = np.array([0.10, -0.13, 0.05])

def kayma(r_deg, delta_deg, e_true_deg, Htilt, yarim_gen, D=2.0, pivot="kenar", Hpiv=0.0):
    """delta: gerçek ışın, yazılımın sandığı eksenin kaç derece ÜSTÜNDE. Dönüş: (sola cm, yukarı cm, pan, tilt, burulma)"""
    r, dl, e = np.radians(r_deg), np.radians(delta_deg), np.radians(e_true_deg)
    ea = e - dl                                   # yazılımın sandığı yükseliş
    d0 = np.array([np.cos(ea), 0, np.sin(ea)])
    bH = np.array([np.cos(dl), 0, np.sin(dl)])
    piv = np.array([0, yarim_gen, 0.0]) if pivot == "kenar" else np.array([0, 0, Hpiv])
    def lazer(RWB, pan, tau):
        RBH = Rz(pan) @ Ry(-tau)
        pB = np.array([0, 0, Htilt]) + RBH @ lH
        return piv + RWB @ (pB - piv), RWB @ RBH
    def nokta(L, b, X): t = (X - L[0]) / b[0]; return L + t * b
    L0, R0 = lazer(np.eye(3), 0.0, ea)
    X = L0[0] + D
    s0 = nokta(L0, R0 @ bH, X)
    RWB = Rx(-r)                                  # sola yatış: sol taraf aşağı
    dB = RWB.T @ d0
    pan = np.arctan2(dB[1], dB[0]); tau = np.arctan2(dB[2], np.hypot(dB[0], dB[1]))
    L1, R1 = lazer(RWB, pan, tau)
    s1 = nokta(L1, R1 @ bH, X)
    Rrel = R1 @ R0.T; bur = np.degrees(np.arccos(np.clip((np.trace(Rrel) - 1) / 2, -1, 1)))
    return (s1[1] - s0[1]) * 100, (s1[2] - s0[2]) * 100, np.degrees(pan), np.degrees(tau - ea), bur, (L1 - L0) * 100

if __name__ == "__main__":
    e_true = float(sys.argv[1]) if len(sys.argv) > 1 else 31.0
    print("gercek isin yukselisi %.1f derece, duvar 2 m | + = sola / yukari (cm)" % e_true)
    for Ht, yg in ((0.25, 0.15), (0.35, 0.15), (0.45, 0.15)):
        print("\ntilt ekseni masadan %.0f cm yukarida, donme kenari pan ekseninden %.0f cm solda" % (Ht * 100, yg * 100))
        print("  yatis | yalniz paralaks (montaj dogru)      | montaj hatasi -8   -4   -2   +2   +4   +8 derece (toplam sola kayma, cm)")
        for r in (8, 13, 18):
            y, z, pan, dt, bur, dL = kayma(r, 0.0, e_true, Ht, yg)
            s = "  %3d   | sola %+5.1f yukari %+5.1f (pan %+5.1f, burulma %4.1f) |" % (r, y, z, pan, bur)
            for dl in (-8, -4, -2, 2, 4, 8): s += " %+6.1f" % kayma(r, dl, e_true, Ht, yg)[0]
            print(s)
    print("\nyarismadaki durum: donme merkezi pan ekseni uzerinde, tilt ekseninin 16,6 cm altinda, hedef 4,47 m, yatis 8 derece")
    for dl in (0, 1, 2, 3, 5):
        y, z, pan, dt, bur, dL = kayma(8, dl, 26.6, 0.166, 0.0, D=4.0, pivot="merkez", Hpiv=0.0)
        print("  montaj hatasi %d derece: sola %+5.1f cm, yukari %+5.1f cm" % (dl, y, z))
