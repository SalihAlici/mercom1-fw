#!/usr/bin/env python3
"""Etkileşimsiz hızlı test: python quicktest.py COM8

Karta bağlanır, ping atar, 2 s telemetri toplar, enkoder / IMU / sürücü
durumlarını sorar ve özet basar. Hiçbir motor komutu göndermez.
"""
import struct
import sys
import time

sys.path.insert(0, __file__.rsplit("\\", 1)[0] if "\\" in __file__ else __file__.rsplit("/", 1)[0])
import mercom_link as ml  # noqa: E402


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM8"
    link = ml.Link(port)
    time.sleep(0.3)
    print(f"--- {port} acildi, ping ---")
    link.send(ml.MSG_PING)
    time.sleep(0.4)

    print("--- telemetri 100 Hz, 2 s ---")
    link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 100))
    n0 = link.telem_count
    time.sleep(2.0)
    n1 = link.telem_count
    print(f"2 s'de {n1 - n0} telemetri paketi geldi (beklenen ~200), PC tarafi CRC hatasi: {link.parser.errors}")
    link.show_telem()

    for name, mid in (("enkoder", ml.MSG_ENC_DIAG), ("i2c tarama", ml.MSG_I2C_SCAN),
                      ("imu", ml.MSG_IMU_DIAG), ("surucu", ml.MSG_TMC_DIAG)):
        print(f"--- {name} ---")
        link.send(mid)
        time.sleep(0.6)

    link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 20))
    time.sleep(0.2)
    link.close()


if __name__ == "__main__":
    main()
