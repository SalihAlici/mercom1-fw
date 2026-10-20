// LAZER SINAMASI: PC0 (Nucleo A1, lazer MOSFET kapısı) 2 s açık / 1 s kapalı sürülür.
// Yeşil LED (LD1) lazerle aynı anda yanar; lazer bağlı değilse bile pinin çalıştığı görülür.
// Seri port 115200: '1' sürekli açık, '0' kapalı, 'b' yanıp sönme (varsayılan).
#include <Arduino.h>

static const PinName LAZER = PC_0;    // ana firmware ile aynı pin (hw.h PN_LASER_EN)
static const PinName LED   = PB_0;    // LD1 yeşil

static void lazer(int on)
{
    digitalWriteFast(LAZER, on ? HIGH : LOW);
    digitalWriteFast(LED, on ? HIGH : LOW);
}

void setup()
{
    Serial.begin(115200);
    pinMode(pinNametoDigitalPin(LAZER), OUTPUT);
    pinMode(pinNametoDigitalPin(LED), OUTPUT);
    lazer(0);
    Serial.println("\nLAZER SINAMASI: PC0 (A1) 2 s acik / 1 s kapali. '1' acik, '0' kapali, 'b' yanip sonme");
}

void loop()
{
    static char mod = 'b';
    static uint32_t tur = 0;
    if (Serial.available()) {
        char c = Serial.read();
        if (c == '1' || c == '0' || c == 'b') { mod = c; Serial.print("mod: "); Serial.println(c); }
    }
    if (mod == '1') { lazer(1); delay(200); return; }
    if (mod == '0') { lazer(0); delay(200); return; }
    lazer(1); Serial.print("lazer ACIK  tur "); Serial.println(++tur); delay(2000);
    lazer(0); Serial.println("lazer kapali");               delay(1000);
}
