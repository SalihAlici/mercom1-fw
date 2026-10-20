# IMU montaj eğimini (imu_mount_p) cetvelle ölçülen ışın yükselişinden hesaplar.
#
# Neden: yazılım ışının yönünü IMU'dan bilir; IMU ile lazer arasındaki açı (montaj eğimi) yanlışsa taban yatınca
# nokta yay çizer. Tabanı elle yatırarak bu açı bulunamıyor, çünkü elle yatırmada lazerin kendisi de 10 cm kadar
# yer değiştiriyor (paralaks) ve iki etki birbirine karışıyor. Bu ölçüm taban DÜZ ve HAREKETSİZken yapılır.
#
# Kullanım:  python montaj_olc.py <lazer_cm> <nokta_cm> <mesafe_cm> [uygula]
#   lazer_cm  : lazerin çıkış ucunun zeminden yüksekliği
#   nokta_cm  : duvardaki lazer noktasının AYNI zeminden yüksekliği
#   mesafe_cm : lazerin çıkış ucundan duvara YATAY mesafe
#   uygula    : verilirse bulunan değer karta yazılır (kalıcı olması için params.c varsayılanı da değiştirilmeli)
import math, struct, sys, time
import mercom_link as ml
from serial.tools import list_ports

D2R = math.pi / 180.0


def mount(r, p, y):
    r, p, y = r * D2R, p * D2R, y * D2R
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def elevation(roll, pitch, mr, mp, my):
    """IMU'nun roll/pitch açısı ve montaj açılarıyla yazılımın hesapladığı ışın yükselişi (derece). Yaw etkilemez."""
    bx, by, bz = (row[0] for row in mount(mr, mp, my))            # ışın ekseni, IMU çerçevesinde
    r, p = roll * D2R, pitch * D2R
    # R_NI = Rz(yaw) Ry(pitch) Rx(roll); üçüncü satırı (dünya yukarı ekseni) yaw'dan bağımsız
    z = -math.sin(p) * bx + math.cos(p) * math.sin(r) * by + math.cos(p) * math.cos(r) * bz
    return math.degrees(math.asin(max(-1.0, min(1.0, z))))


def solve_p(roll, pitch, mr, my, e_true):
    lo, hi = -40.0, 40.0
    f = lambda p: elevation(roll, pitch, mr, p, my) - e_true
    if f(lo) * f(hi) > 0: return None
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if f(lo) * f(mid) <= 0: hi = mid
        else: lo = mid
    return 0.5 * (lo + hi)


def main():
    if len(sys.argv) < 4:
        print(__doc__ or "kullanim: montaj_olc.py <lazer_cm> <nokta_cm> <mesafe_cm> [uygula]"); return 1
    h_l, h_n, dist = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
    uygula = len(sys.argv) > 4
    e_true = math.degrees(math.atan2(h_n - h_l, dist))
    print("olcum: nokta lazerden %.1f cm yukarida, yatay mesafe %.1f cm -> isinin gercek yukselisi %.2f derece" % (h_n - h_l, dist, e_true))
    port = ml.terminal_port()
    if port is None: print("KART YOK"); return 1
    link = ml.Link(port); orig = link._handle
    def h(mid, p):
        if mid in (ml.MSG_TEXT, ml.MSG_ACK, ml.MSG_PONG): return
        orig(mid, p)
    link._handle = h
    t0 = time.time()
    while link.last_telem is None and time.time() - t0 < 8:
        link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 50)); time.sleep(0.5)
    if link.last_telem is None: print("TELEMETRI YOK"); link.close(); return 1
    names = ("imu_mount_r", "imu_mount_p", "imu_mount_y")
    for n in names: link.send(ml.MSG_PARAM_GET, struct.pack("<H", ml.PARAM_NAMES.index(n))); time.sleep(0.1)
    time.sleep(0.3)
    mr, mp, my = (link.params.get(ml.PARAM_NAMES.index(n)) for n in names)
    if None in (mr, mp, my): print("parametre okunamadi"); link.close(); return 1
    rs, ps, n, last = 0.0, 0.0, 0, None
    t0 = time.time()
    while time.time() - t0 < 2.0:                                  # 2 s ortalama, yalnız taze IMU örnekleri
        time.sleep(0.01); t = link.last_telem
        if t is last: continue
        last = t; d = dict(zip(ml.TELEM_COLS, t))
        if (int(d["flags"]) >> 2) & 1: rs += d["roll"]; ps += d["pitch"]; n += 1
    if n < 10: print("IMU verisi yok (kablo?)"); link.close(); return 1
    roll, pitch = rs / n, ps / n
    e_now = elevation(roll, pitch, mr, mp, my)
    p_new = solve_p(roll, pitch, mr, my, e_true)
    print("IMU: roll %.2f pitch %.2f | simdiki montaj egimi %.3f -> yazilimin sandigi yukselis %.2f derece (fark %+.2f)" % (roll, pitch, mp, e_now, e_true - e_now))
    if p_new is None: print("cozum yok: olcumleri kontrol et"); link.close(); return 1
    print("DOGRU MONTAJ EGIMI: imu_mount_p = %.3f   (olcumde 1 cm hata = yaklasik 0,2 derece)" % p_new)
    if uygula:
        d = dict(zip(ml.TELEM_COLS, link.last_telem))
        if int(d["mode"]) != 1: link.send(ml.MSG_SET_MODE, bytes([1])); time.sleep(0.4)
        link.send(ml.MSG_PARAM_SET, struct.pack("<Hf", ml.PARAM_NAMES.index("imu_mount_p"), p_new)); time.sleep(0.3)
        link.send(ml.MSG_PARAM_GET, struct.pack("<H", ml.PARAM_NAMES.index("imu_mount_p"))); time.sleep(0.4)
        print("karta yazildi, geri okunan: %.3f (kilit yeniden alinmali)" % link.params.get(ml.PARAM_NAMES.index("imu_mount_p")))
    link.close(); return 0


if __name__ == "__main__":
    raise SystemExit(main())
