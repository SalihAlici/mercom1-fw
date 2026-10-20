#!/usr/bin/env python3
"""Enkoder canlı izleme: python enc_watch.py COM8 [saniye] [pan|tilt|ce]

Her 0,5 s'de pan enkoderinin ham değeri, açı, AGC ve mıknatıs bayrakları basılır.
AGC 60–200 arası ve OCF=1 iyi. AGC 255 + uzak=1: mıknatıs çok uzak. AGC 0 + yakin=1: çok yakın.
"""
import os
import re
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mercom_link as ml  # noqa: E402


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM8"
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else 15.0
    ch = sys.argv[3] if len(sys.argv) > 3 else "pan"
    link = ml.Link(port)
    lines = []
    orig = link._handle

    def handler(mid, payload):
        if mid == ml.MSG_TEXT:
            t = payload.decode("ascii", "replace")
            if t.startswith(f"enc {ch}:"):
                lines.append(t)
            elif not t.startswith("enc "):
                print(f"   [kart] {t}")      # acilis / reset mesajlari gorunsun
        else:
            orig(mid, payload)

    link._handle = handler
    time.sleep(0.3)
    link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 50))
    print(f"   [{ch}]   t    raw     aci    agc  OCF yakin uzak   sayaclar / son SPI kelimesi   (Ctrl+C ile cik)")
    print("   spi=0xFFFF: MISO bos ya da enkoder beslenmiyor · 0x0000: MISO GND'de · degisken: CLK ya da CS")
    t0 = time.time()
    try:
        while time.time() - t0 < dur:
            link.send(ml.MSG_ENC_DIAG)
            time.sleep(0.5)
            if lines:
                m = re.search(r"raw=(\d+) agc=(\d+) OCF=(\d) COF=(\d) yakin=(\d) uzak=(\d) ok=(\d+) err=(\d+) spi=0x([0-9A-F]+)(?: hatali=0x([0-9A-F]+) parite=(\d+) EF=(\d+))?", lines[-1])
                d = dict(zip(ml.TELEM_COLS, link.last_telem)) if link.last_telem else None
                if m:
                    raw, agc, ocf, cof, yakin, uzak, ok, err = map(int, m.groups()[:8])
                    spi = m.group(9)
                    aci = d[f"enc_{ch}"] if d else raw * 360.0 / 16384
                    ek = f" parite={m.group(11)} EF={m.group(12)} hatali=0x{m.group(10)}" if m.group(10) else ""
                    print(f"{time.time() - t0:5.1f}  {raw:5d}  {aci:8.2f}  {agc:3d}   {ocf}    {yakin}    {uzak}   ok={ok} err={err} spi=0x{spi}{ek}")
                lines.clear()
    except KeyboardInterrupt:
        pass
    finally:
        link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 20))
        time.sleep(0.1)
        link.close()


if __name__ == "__main__":
    main()
