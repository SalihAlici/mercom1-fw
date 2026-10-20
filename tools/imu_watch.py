#!/usr/bin/env python3
"""BNO085 canlı izleme: python imu_watch.py COM8 [saniyede_kac_satir] [--zero] [--log dosya.csv]

Her satırda roll, pitch, yaw (derece) ve IMU rapor sayacı basılır. Ctrl+C ile çıkılır.
--zero: başlarken o anki yaw'ı sıfır kabul eder (yaw her açılışta rastgele bir yerden başlar).
Örnek:  python imu_watch.py COM8 5
        python imu_watch.py COM8 10 --log egme.csv
"""
import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mercom_link as ml  # noqa: E402


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return
    port = sys.argv[1]
    hz = float(sys.argv[2]) if len(sys.argv) > 2 and not sys.argv[2].startswith("--") else 5.0
    logfile = sys.argv[sys.argv.index("--log") + 1] if "--log" in sys.argv else None

    link = ml.Link(port, logfile=logfile)
    time.sleep(0.3)
    link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 50))
    time.sleep(0.5)
    if "--zero" in sys.argv:
        link.send(ml.MSG_YAW_ZERO)
        time.sleep(0.2)
    print("   zaman     roll    pitch      yaw   rapor   (Ctrl+C ile cik)")
    t0 = time.time()
    last_count = None
    try:
        while True:
            if link.last_telem:
                d = dict(zip(ml.TELEM_COLS, link.last_telem))
                ok = (d["flags"] >> 2) & 1
                stale = "" if ok else "   IMU YOK"
                print(f"{time.time() - t0:8.1f}  {d['roll']:7.2f}  {d['pitch']:7.2f}  {d['yaw']:8.2f}  {d['imu_count']:6d}{stale}")
                last_count = d["imu_count"]
            else:
                print("telemetri bekleniyor...")
            time.sleep(1.0 / hz)
    except KeyboardInterrupt:
        pass
    finally:
        link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 20))
        time.sleep(0.1)
        link.close()


if __name__ == "__main__":
    main()
