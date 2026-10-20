#!/usr/bin/env python3
"""MERCOM-1 PC tarafı: seri bağlantı, çerçeveleme, komut satırı, CSV kayıt.

Kullanım:
    python mercom_link.py COM8            # portu belirt
    python mercom_link.py COM8 --log t.csv

Komutlar (bağlandıktan sonra):
    ping                 kart cevap veriyor mu
    mode manual|auto     mod değiştir
    en on|off [eksen]    motor sürücülerini aç/kapat; eksen verilirse yalnız o: en off tilt
    jog <eksen> <d/s>    rampalı hız: jog pan 5    (eksen: pan tilt ce ya da 0 1 2); sınıra gelince durur
    goto <eksen> <derece> ivmeli profille açıya git: goto tilt 30   (pan kısa yoldan)
    osc <eksen> <A> <B> [bekleme ms]   A ile B arasında salın: osc pan 10 40 500;  osc <eksen> off durdurur
    zero <eksen> [derece]  şu anki konumu 0 (ya da verilen açı) kabul et
    axes                 eksenlerin hedef, hız, sınır ve düzeltme sayaçları
    move <eksen> <adım> [adım/s]  tam N mikroadım at ve dur: move tilt 3200 800
    stop                 tüm eksenleri durdur
    laser on|off
    get <ad|id>          parametre oku:  get kp_pan
    set <ad|id> <değer>  parametre yaz:  set vmax_pan 30
    params               tüm parametreleri listele
    save                 parametreleri kalıcı yaz
    rate <Hz>            telemetri hızı, 0 kapatır
    t                    son telemetriyi göster
    log <dosya> | log off
    clear                arızayı temizle
    encdiag              üç enkoderin AGC ve mıknatıs bayrakları
    imudiag              BNO085 durumu, rapor sayacı, son açılar
    tmcdiag              üç sürücünün sürüm, akım ayarı, DRV_STATUS
    tmcinit              sürücüleri yeniden tara ve ayarla (parametre değiştirince)
    i2cscan              I2C'de cevap veren adresleri listele (BNO085 0x4A olmalı)
    imuinit              IMU'yu yeniden başlat (kablo sonradan takıldıysa)
    yawzero              şu anki yaw'ı gösterimde sıfır yap (yaw her açılışta rastgele başlar)
    q                    çık
"""
import struct
import sys
import threading
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial gerekli:  pip install pyserial")

SOF = 0xA5

MSG_PING, MSG_SET_MODE, MSG_JOG, MSG_GOTO, MSG_LOCK, MSG_UNLOCK, MSG_LASER = 1, 2, 3, 4, 5, 6, 7
MSG_PARAM_GET, MSG_PARAM_SET, MSG_PARAM_SAVE, MSG_ENABLE, MSG_TELEM_RATE, MSG_STOP, MSG_CLEAR_FAULT = 8, 9, 10, 11, 12, 13, 14
MSG_ENC_DIAG, MSG_IMU_DIAG, MSG_TMC_DIAG, MSG_TMC_INIT, MSG_I2C_SCAN, MSG_IMU_INIT, MSG_YAW_ZERO, MSG_IMU_PINTEST, MSG_SYS_RESET, MSG_STEP_MOVE, MSG_TMC_RAW, MSG_TMC_LOOP, MSG_STEP_BLINK, MSG_TMC_TXCHK = 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28
MSG_OSC, MSG_AXIS_ZERO, MSG_AXIS_INFO, MSG_TILT_FROM_IMU = 0x1D, 0x1E, 0x1F, 0x20
MSG_LORA_CFG = 0x21
MSG_PONG, MSG_TELEM, MSG_PARAM_VALUE, MSG_TEXT, MSG_ACK = 0x81, 0x82, 0x83, 0x84, 0x85
KNOWN_RX_IDS = {MSG_PONG, MSG_TELEM, MSG_PARAM_VALUE, MSG_TEXT, MSG_ACK}

MODES = {0: "INIT", 1: "MANUAL", 2: "AUTO", 3: "FAULT"}
AXES = {"pan": 0, "tilt": 1, "ce": 2, "0": 0, "1": 1, "2": 2}
ACK = {0: "ok", 1: "uzunluk hatali", 2: "arguman hatali", 3: "bu modda olmaz", 4: "henuz yok", 5: "reddedildi"}

# params.h ile aynı sırada
PARAM_NAMES = [
    "kp_pan", "kp_tilt", "kp_ce",
    "ki_pan", "ki_tilt", "ki_ce",
    "vmax_pan", "vmax_tilt", "vmax_ce",
    "amax_pan", "amax_tilt", "amax_ce",
    "dps_pan", "dps_tilt", "dps_ce",
    "enc_off_pan", "enc_off_tilt", "enc_off_ce",
    "enc_dir_pan", "enc_dir_tilt", "enc_dir_ce",
    "mot_dir_pan", "mot_dir_tilt", "mot_dir_ce",
    "pan_min", "pan_max", "tilt_min", "tilt_max", "ce_min", "ce_max",
    "imu_mount_r", "imu_mount_p", "imu_mount_y",
    "yaw_drift_tau", "yaw_drift_en", "laser_err_thr", "telem_hz",
    "mot_irun_ma", "mot_ihold_pct", "mot_usteps", "tmc_rsense", "imu_rate_hz",
    "enc_ratio_pan", "enc_ratio_tilt", "enc_ratio_ce",
    "cl_pan", "cl_tilt", "cl_ce",
    "sl_thr_pan", "sl_thr_tilt", "sl_thr_ce",
    "enc_db_pan", "enc_db_tilt", "enc_db_ce",
    "aim_sign_pan", "aim_sign_tilt", "aim_vmax", "aim_imax", "aim_err_max",
    "aim_kff", "aim_db",
    "bl_pan", "bl_tilt", "bl_ce",
    "aim_kff_pan",
    "plx_en", "plx_ht", "plx_lx", "plx_ly", "plx_lz", "plx_d",
    "yaw_anchor", "imu_poll_us", "imu_i2c_soft", "imu_gyro", "enc_read", "i2c_sync", "lora_telem_hz",
]

TELEM_FMT = "<IBBH15fHHIII3f"
TELEM_SIZE = struct.calcsize(TELEM_FMT)  # 84
TELEM_COLS = (["t_ms", "mode", "flags", "fault"]
              + [f"enc_{a}" for a in ("pan", "tilt", "ce")]
              + [f"cmd_{a}" for a in ("pan", "tilt", "ce")]
              + [f"vel_{a}" for a in ("pan", "tilt", "ce")]
              + ["roll", "pitch", "yaw"]
              + [f"err_{a}" for a in ("pan", "tilt", "ce")]
              + ["loop_us", "loop_max_us", "imu_count", "rx_frames", "rx_errors"]
              + ["aim_err", "aim_side", "aim_up"])


def crc16(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode(mid: int, payload: bytes = b"") -> bytes:
    body = bytes([len(payload), mid]) + payload
    return bytes([SOF]) + body + struct.pack("<H", crc16(body))


class Parser:
    def __init__(self):
        self.buf = bytearray()
        self.errors = 0

    def feed(self, data: bytes):
        self.buf += data
        while True:
            i = self.buf.find(SOF)
            if i < 0:
                self.buf.clear()
                return
            if i > 0:
                del self.buf[:i]
            if len(self.buf) < 5:
                return
            ln = self.buf[1]
            # Sahte SOF'a takılmamak için uzunluk ve kimlik makul olmalı
            if ln > 200 or (len(self.buf) >= 3 and self.buf[2] not in KNOWN_RX_IDS):
                del self.buf[:1]
                continue
            total = 5 + ln
            if len(self.buf) < total:
                return
            body = bytes(self.buf[1:3 + ln])
            crc_rx = struct.unpack_from("<H", self.buf, 3 + ln)[0]
            if crc16(body) == crc_rx:
                mid, payload = self.buf[2], bytes(self.buf[3:3 + ln])
                del self.buf[:total]
                yield mid, payload
            else:
                self.errors += 1
                del self.buf[:1]


# Kartlar seri numarasıyla ayrılır (30 Eyl): bilgisayarda terminal ve LoRa yer istasyonu köprüsü (ikisi de Teensy 4.1) aynı anda takılı olabilir.
TERMINAL_SERIALS = ("15425680",)          # terminaldeki Teensy 4.1
LORA_BRIDGE_SERIALS = ("18913180",)       # yer istasyonu köprüsü (Teensy 4.1 + E22, tools/lora_koprusu)
LORA_VIDS = (0x1A86, 0x10C4, 0x0403, 0x067B)   # USB-seri dönüştürücüye takılı E22 de yer istasyonu olabilir


def is_lora_port(p) -> bool:
    return p.vid in LORA_VIDS or (p.vid == 0x16C0 and p.serial_number in LORA_BRIDGE_SERIALS)


def terminal_port():
    """Terminale doğrudan USB ile bağlı port: önce bilinen seri no, yoksa köprü olmayan ilk ST-Link/Teensy."""
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    p = next((q.device for q in ports if q.serial_number in TERMINAL_SERIALS), None)
    return p or next((q.device for q in ports if q.vid in (0x16C0, 0x0483) and not is_lora_port(q)), None)


def lora_port():
    from serial.tools import list_ports
    return next((q.device for q in list_ports.comports() if is_lora_port(q)), None)


class Link:
    def __init__(self, port: str, baud: int = 115200, logfile: str | None = None):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.parser = Parser()
        self.last_telem = None
        self.params = {}
        self.telem_count = 0
        self.log = None
        self.lock = threading.Lock()
        if logfile:
            self.start_log(logfile)
        self.running = True
        self.th = threading.Thread(target=self._reader, daemon=True)
        self.th.start()

    def start_log(self, path: str):
        self.stop_log()
        self.log = open(path, "w", buffering=1)
        self.log.write(",".join(["pc_time"] + TELEM_COLS) + "\n")
        print(f"kayit basladi: {path}")

    def stop_log(self):
        if self.log:
            self.log.close()
            self.log = None
            print("kayit durdu")

    def send(self, mid: int, payload: bytes = b""):
        with self.lock:
            self.ser.write(encode(mid, payload))

    def _reader(self):
        while self.running:
            data = self.ser.read(256)
            if not data:
                continue
            for mid, payload in self.parser.feed(data):
                self._handle(mid, payload)

    def _handle(self, mid: int, payload: bytes):
        if mid == MSG_TELEM and len(payload) == TELEM_SIZE:
            vals = struct.unpack(TELEM_FMT, payload)
            self.last_telem = vals
            self.telem_count += 1
            if self.log:
                self.log.write(f"{time.time():.3f}," + ",".join(f"{v:.4f}" if isinstance(v, float) else str(v) for v in vals) + "\n")
        elif mid == MSG_TEXT:
            print(f"[kart] {payload.decode('ascii', 'replace')}")
        elif mid == MSG_PONG:
            up, ver = struct.unpack("<II", payload)
            print(f"pong: uptime {up / 1000:.1f} s, fw {ver >> 16}.{(ver >> 8) & 0xFF}.{ver & 0xFF}")
        elif mid == MSG_ACK:
            rid, st = payload
            print(f"ack 0x{rid:02X}: {ACK.get(st, st)}")
        elif mid == MSG_PARAM_VALUE:
            pid, val, ok = struct.unpack("<HfB", payload)
            if ok: self.params[pid] = val
            name = PARAM_NAMES[pid] if pid < len(PARAM_NAMES) else f"#{pid}"
            print(f"{name} = {val:.6g}" if ok else f"parametre {pid} yok")
        else:
            print(f"bilinmeyen mesaj 0x{mid:02X} ({len(payload)} bayt)")

    def show_telem(self):
        t = self.last_telem
        if not t:
            print("telemetri gelmedi")
            return
        d = dict(zip(TELEM_COLS, t))
        print(f"t={d['t_ms'] / 1000:.2f}s mod={MODES.get(d['mode'], d['mode'])} flags=0x{d['flags']:02X} fault=0x{d['fault']:04X}")
        for a in ("pan", "tilt", "ce"):
            print(f"  {a:5s} enc={d['enc_' + a]:8.3f}  cmd={d['cmd_' + a]:8.3f}  vel={d['vel_' + a]:7.2f}  err={d['err_' + a]:7.3f}")
        print(f"  rpy=({d['roll']:.2f}, {d['pitch']:.2f}, {d['yaw']:.2f})  loop={d['loop_us']} us  max={d['loop_max_us']} us")
        print(f"  imu={d['imu_count']}  rx={d['rx_frames']}  rx_err={d['rx_errors']}  pc_crc_err={self.parser.errors}  telem={self.telem_count}")

    def close(self):
        self.running = False
        self.stop_log()
        self.ser.close()


def param_id(s: str) -> int | None:
    if s.isdigit():
        return int(s)
    return PARAM_NAMES.index(s) if s in PARAM_NAMES else None


def repl(link: Link):
    print("baglandi. 'ping' dene, 'q' cik.")
    while True:
        try:
            line = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not line:
            continue
        cmd, *args = line.split()
        try:
            if cmd == "q":
                break
            elif cmd == "ping":
                link.send(MSG_PING)
            elif cmd == "mode":
                m = {"manual": 1, "auto": 2}[args[0]]
                link.send(MSG_SET_MODE, bytes([m]))
            elif cmd == "en":
                on = 1 if args[0] == "on" else 0
                link.send(MSG_ENABLE, bytes([AXES[args[1]], on]) if len(args) > 1 else bytes([on]))
            elif cmd == "stepblink":
                link.send(MSG_STEP_BLINK, bytes([AXES[args[0]] if args else 0]))
            elif cmd == "tmctx":
                link.send(MSG_TMC_TXCHK)
            elif cmd == "tmcloop":
                link.send(MSG_TMC_LOOP, bytes([AXES[args[0]] if args else 0]))
            elif cmd == "tmcraw":
                link.send(MSG_TMC_RAW, bytes([AXES[args[0]] if args else 0] + ([int(args[1])] if len(args) > 1 else [])))
            elif cmd == "move":
                link.send(MSG_STEP_MOVE, struct.pack("<Bif", AXES[args[0]], int(args[1]), float(args[2]) if len(args) > 2 else 800.0))
            elif cmd == "jog":
                link.send(MSG_JOG, struct.pack("<Bf", AXES[args[0]], float(args[1])))
            elif cmd == "osc":
                ax = AXES[args[0]]
                if len(args) >= 2 and args[1] == "off": link.send(MSG_OSC, bytes([ax]))
                else: link.send(MSG_OSC, struct.pack("<BffH", ax, float(args[1]), float(args[2]), int(args[3]) if len(args) > 3 else 500))
            elif cmd == "zero":
                link.send(MSG_AXIS_ZERO, struct.pack("<Bf", AXES[args[0]], float(args[1]) if len(args) > 1 else 0.0))
            elif cmd == "axes":
                link.send(MSG_AXIS_INFO)
            elif cmd == "tiltimu":
                link.send(MSG_TILT_FROM_IMU)
            elif cmd == "goto":
                link.send(MSG_GOTO, struct.pack("<Bf", AXES[args[0]], float(args[1])))
            elif cmd == "stop":
                link.send(MSG_STOP)
            elif cmd == "lock":
                link.send(MSG_LOCK)
            elif cmd == "unlock":
                link.send(MSG_UNLOCK)
            elif cmd == "laser":
                link.send(MSG_LASER, bytes([1 if args[0] == "on" else 0]))
            elif cmd == "get":
                pid = param_id(args[0])
                if pid is None:
                    print("parametre adi yok")
                else:
                    link.send(MSG_PARAM_GET, struct.pack("<H", pid))
            elif cmd == "set":
                pid = param_id(args[0])
                if pid is None:
                    print("parametre adi yok")
                else:
                    link.send(MSG_PARAM_SET, struct.pack("<Hf", pid, float(args[1])))
            elif cmd == "params":
                for i in range(len(PARAM_NAMES)):
                    link.send(MSG_PARAM_GET, struct.pack("<H", i))
                    time.sleep(0.01)
            elif cmd == "save":
                link.send(MSG_PARAM_SAVE)
            elif cmd == "rate":
                link.send(MSG_TELEM_RATE, struct.pack("<H", int(args[0])))
            elif cmd == "t":
                link.show_telem()
            elif cmd == "log":
                if args[0] == "off":
                    link.stop_log()
                else:
                    link.start_log(args[0])
            elif cmd == "clear":
                link.send(MSG_CLEAR_FAULT)
            elif cmd == "encdiag":
                link.send(MSG_ENC_DIAG)
            elif cmd == "imudiag":
                link.send(MSG_IMU_DIAG)
            elif cmd == "tmcdiag":
                link.send(MSG_TMC_DIAG)
            elif cmd == "tmcinit":
                link.send(MSG_TMC_INIT)
            elif cmd == "i2cscan":
                link.send(MSG_I2C_SCAN)
            elif cmd == "imuinit":
                link.send(MSG_IMU_INIT)
            elif cmd == "yawzero":
                link.send(MSG_YAW_ZERO)
            elif cmd == "imupintest":
                link.send(MSG_IMU_PINTEST)
            elif cmd == "reset":
                link.send(MSG_SYS_RESET)
            else:
                print("bilinmeyen komut, --help'e bak")
        except (KeyError, IndexError, ValueError) as e:
            print(f"kullanim hatasi: {e}")
        time.sleep(0.05)


def main():
    if len(sys.argv) < 2 or sys.argv[1] in ("-h", "--help"):
        print(__doc__)
        return
    port = sys.argv[1]
    logfile = None
    if "--log" in sys.argv:
        logfile = sys.argv[sys.argv.index("--log") + 1]
    link = Link(port, logfile=logfile)
    try:
        repl(link)
    finally:
        link.close()


if __name__ == "__main__":
    main()
