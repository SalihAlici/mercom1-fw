// KART SINAMASI: üç LED sırayla yanar (yeşil LD1, sarı LD2, kırmızı LD3, her biri 500 ms), aynı anda işlemci
// kendi 3,3 V beslemesini (VDDA, iç referans VREFINT ile) ve çekirdek sıcaklığını ölçüp saniyede bir seri porta yazar:
// hangi LED yanıyor, VDDA, sıcaklık, açık kalma süresi. Besleme 3,0–3,6 V dışına çıkarsa UYARI yazar ve üç LED birden yanar.
// Dış pinlere dokunmaz. tools/seri_izle.bat ile canlı izlenir.
#include <Arduino.h>

#define VREFINT_CAL   (*(uint16_t *)0x1FF1E860UL)   // fabrika: VREFINT'in 3,3 V'ta 16 bit ADC değeri
#define TS_CAL1       (*(uint16_t *)0x1FF1E820UL)   // sıcaklık sensörü 30 °C
#define TS_CAL2       (*(uint16_t *)0x1FF1E840UL)   // sıcaklık sensörü 110 °C

struct Led { const char *ad; PinName pin; };
static const Led LEDLER[3] = { { "YESIL",   PB_0 }, { "SARI", PE_1 }, { "KIRMIZI", PB_14 } };

static uint32_t vddaMv(void)
{
    uint32_t acc = 0;
    for (int i = 0; i < 16; i++) acc += analogRead(AVREF);
    uint32_t vref = acc / 16;
    return vref ? (3300UL * VREFINT_CAL) / vref : 0;
}

static int32_t cekirdekC10(uint32_t vdda_mV)
{
    uint32_t acc = 0;
    for (int i = 0; i < 16; i++) acc += analogRead(ATEMP);
    int32_t tdata = (int32_t)((uint64_t)(acc / 16) * vdda_mV / 3300UL);   // kalibrasyon 3,3 V'ta
    return 300 + (int32_t)((int64_t)(tdata - (int32_t)TS_CAL1) * 800 / ((int32_t)TS_CAL2 - (int32_t)TS_CAL1));
}

void setup()
{
    Serial.begin(115200);
    analogReadResolution(16);
    for (auto &l : LEDLER) { pinMode(pinNametoDigitalPin(l.pin), OUTPUT); digitalWriteFast(l.pin, LOW); }
    Serial.println("\nKART SINAMASI: LED'ler sirayla, VDDA ve cekirdek sicakligi saniyede bir");
}

void loop()
{
    static uint32_t n = 0, sonYazma = 0;
    static int hangi = 0;
    static uint32_t tmin = 9999, tmax = 0, vmin = 9999, vmax = 0;

    for (int i = 0; i < 3; i++) digitalWriteFast(LEDLER[i].pin, i == hangi ? HIGH : LOW);
    uint32_t v = vddaMv();
    int32_t t = cekirdekC10(v);
    bool ok = v >= 3000 && v <= 3600;
    if (!ok) for (auto &l : LEDLER) digitalWriteFast(l.pin, HIGH);
    if (v < vmin) vmin = v; if (v > vmax) vmax = v;
    if ((uint32_t)t < tmin) tmin = t; if ((uint32_t)t > tmax) tmax = t;

    if (millis() - sonYazma >= 1000) {
        sonYazma = millis(); n++;
        Serial.print(n); Serial.print(" s  LED: "); Serial.print(ok ? LEDLER[hangi].ad : "UCU BIRDEN");
        Serial.print("  VDDA "); Serial.print(v / 1000); Serial.print('.'); if ((v / 10) % 100 < 10) Serial.print('0'); Serial.print((v / 10) % 100);
        Serial.print(" V  cekirdek "); Serial.print(t / 10); Serial.print('.'); Serial.print(t % 10); Serial.print(" C");
        Serial.print("   [en dusuk/en yuksek: "); Serial.print(vmin / 1000); Serial.print('.'); if ((vmin / 10) % 100 < 10) Serial.print('0'); Serial.print((vmin / 10) % 100);
        Serial.print("-"); Serial.print(vmax / 1000); Serial.print('.'); if ((vmax / 10) % 100 < 10) Serial.print('0'); Serial.print((vmax / 10) % 100);
        Serial.print(" V, "); Serial.print(tmin / 10); Serial.print("-"); Serial.print(tmax / 10); Serial.print(" C]");
        Serial.println(ok ? "" : "  UYARI: besleme aralik disi!");
    }
    delay(500);
    hangi = (hangi + 1) % 3;
}
