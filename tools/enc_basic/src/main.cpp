// AS5048A basit okuma — çalışan yöntem (11 Eylül 2026):
// yazılımsal SPI (bit-bang, ~250 kHz, mod 1) + düşük slew hızlı pinler.
// Bağlantı (Nucleo-H753ZI CN7, 26 Eyl dizilimi):  SCK=D13 (PA5)  MISO=D12 (PA6)  MOSI=D8 (PF3)
//   CS pan=D11 (PB5)  CS tilt=D10 (PD14)  CS ce=D9 (PD15)
//   Adaptör 5V ve 3.3V pinleri birlikte 3V3'e, GND ortak.
// Seri 115200: her 100 ms'de üç kanalın açısı, AGC ve tanı bayrakları.
#include <Arduino.h>
#include "stm32h7xx_ll_gpio.h"

static const PinName PIN_SCK  = PA_5;
static const PinName PIN_MISO = PA_6;
static const PinName PIN_MOSI = PF_3;
static const PinName PIN_CS[3] = { PB_5, PD_14, PD_15 };
static const char   *NAME[3]   = { "pan", "tilt", "ce" };

#ifndef SPI_DLY
#define SPI_DLY 36
#endif
static inline void spiDelay(void) { for (volatile int i = 0; i < SPI_DLY; i++) { } }

static void slowPin(PinName p)
{
    pinMode(pinNametoDigitalPin(p), OUTPUT);
    GPIO_TypeDef *port = get_GPIO_Port(STM_PORT(p));
    LL_GPIO_SetPinSpeed(port, STM_LL_GPIO_PIN(p), LL_GPIO_SPEED_FREQ_LOW);
}

static uint16_t xfer16(int ch, uint16_t tx)
{
    uint16_t rx = 0;
    digitalWriteFast(PIN_MOSI, (tx >> 15) & 1); spiDelay();
    digitalWriteFast(PIN_CS[ch], LOW); spiDelay(); spiDelay(); spiDelay();
    for (int i = 15; i >= 0; i--) {
        digitalWriteFast(PIN_MOSI, (tx >> i) & 1); spiDelay();
        digitalWriteFast(PIN_SCK, HIGH); spiDelay();
        rx = (uint16_t)(rx << 1) | (digitalReadFast(PIN_MISO) ? 1 : 0);   // dusen kenardan ONCE ornekle
        digitalWriteFast(PIN_SCK, LOW);
    }
    spiDelay(); digitalWriteFast(PIN_CS[ch], HIGH); spiDelay();
    return rx;
}

static uint16_t withParity(uint16_t v)
{
    uint16_t x = v & 0x7FFF, p = x;
    p ^= p >> 8; p ^= p >> 4; p ^= p >> 2; p ^= p >> 1;
    return (p & 1) ? (x | 0x8000) : x;
}
static bool parityOk(uint16_t v)
{
    uint16_t p = v; p ^= p >> 8; p ^= p >> 4; p ^= p >> 2; p ^= p >> 1;
    return (p & 1) == 0;
}

// Yazmaç oku: komut + NOP. Dönüş: 1 geçerli. raw çıktı 14 bit.
static int readReg(int ch, uint16_t reg, uint16_t *out, uint16_t *rawWord)
{
    xfer16(ch, withParity(0x4000 | reg));
    uint16_t r = xfer16(ch, withParity(0x4000 | 0x0000));
    *rawWord = r;
    if (!parityOk(r) || (r & 0x4000)) {
        // hata bayrağını temizle
        xfer16(ch, withParity(0x4000 | 0x0001));
        xfer16(ch, withParity(0x4000 | 0x0000));
        return 0;
    }
    *out = r & 0x3FFF;
    return 1;
}

void setup()
{
    Serial.begin(115200);
    slowPin(PIN_SCK); slowPin(PIN_MOSI);
    for (int i = 0; i < 3; i++) { slowPin(PIN_CS[i]); digitalWriteFast(PIN_CS[i], HIGH); }
    pinMode(pinNametoDigitalPin(PIN_MISO), INPUT);
    digitalWriteFast(PIN_SCK, LOW);
    delay(1500);
    Serial.println("\nAS5048A basit okuma (bit-bang SPI). Kanal: pan=D10 tilt=D9 ce=D8");
}

void loop()
{
    static uint32_t okc[3] = {0}, errc[3] = {0};
    char line[200];
    for (int ch = 0; ch < 3; ch++) {
        uint16_t ang = 0, dg = 0, mag = 0, w1 = 0, w2 = 0, w3 = 0;
        int a = readReg(ch, 0x3FFF, &ang, &w1);
        int d = readReg(ch, 0x3FFD, &dg, &w2);
        int m = readReg(ch, 0x3FFE, &mag, &w3);
        if (a) okc[ch]++; else errc[ch]++;
        if (w1 == 0xFFFF || w1 == 0x0000) {
            snprintf(line, sizeof line, "%-4s: bagli degil (spi=0x%04X)", NAME[ch], w1);
        } else if (!a) {
            snprintf(line, sizeof line, "%-4s: HATA cevap=0x%04X ok=%lu err=%lu", NAME[ch], w1,
                     (unsigned long)okc[ch], (unsigned long)errc[ch]);
        } else {
            int agc = dg & 0xFF, ocf = (dg >> 8) & 1, cof = (dg >> 9) & 1, yakin = (dg >> 10) & 1, uzak = (dg >> 11) & 1;
            const char *note = (d && m && dg == ang && mag == ang) ? "  <-- tani=aci=buyukluk: cip komutu gormuyor, MOSI/SCK kablosu!" : "";
            snprintf(line, sizeof line, "%-4s: %7.2f deg raw=%5u buyukluk=%5u agc=%3d OCF=%d COF=%d yakin=%d uzak=%d ok=%lu err=%lu%s",
                     NAME[ch], ang * 360.0f / 16384.0f, ang, m ? mag : 0, d ? agc : -1, ocf, cof, yakin, uzak,
                     (unsigned long)okc[ch], (unsigned long)errc[ch], note);
        }
        Serial.println(line);
    }
    // MISO cekme testi: kelime cekme yonune gore degisiyorsa MISO'yu kimse surmuyor (cip besleme/CS/MISO teli)
    {
        uint16_t up[3], dn[3]; char b[120];
        pinMode(pinNametoDigitalPin(PIN_MISO), INPUT_PULLUP);
        for (int ch = 0; ch < 3; ch++) { xfer16(ch, 0xFFFF); up[ch] = xfer16(ch, 0xFFFF); }
        pinMode(pinNametoDigitalPin(PIN_MISO), INPUT_PULLDOWN);
        for (int ch = 0; ch < 3; ch++) { xfer16(ch, 0xFFFF); dn[ch] = xfer16(ch, 0xFFFF); }
        pinMode(pinNametoDigitalPin(PIN_MISO), INPUT);
        snprintf(b, sizeof b, "MISO cekme testi  pullup: pan=%04X tilt=%04X ce=%04X   pulldown: pan=%04X tilt=%04X ce=%04X",
                 up[0], up[1], up[2], dn[0], dn[1], dn[2]);
        Serial.println(b);
    }
    // MOSI testi (tilt): bilerek parite hatali komut gonder. Cip MOSI'yi goruyorsa EF (bit14) kalkar.
    {
        char b[140];
        xfer16(1, 0x7FFF);                       // hatali parite
        uint16_t r1 = xfer16(1, 0x0000);         // NOP: beklenen 0x4xxx (EF)
        uint16_t r2 = xfer16(1, 0x4001);         // hata temizle
        uint16_t r3 = xfer16(1, 0x0000);         // NOP
        uint16_t r4 = xfer16(1, 0x0000);         // ikinci NOP: MOSI gorulurse 0x0000 beklenir
        snprintf(b, sizeof b, "MOSI testi (tilt): hatali komut sonrasi=%04X (EF=%d)  temizle=%04X  NOP=%04X  NOP=%04X",
                 r1, (r1 >> 14) & 1, r2, r3, r4);
        Serial.println(b);
    }
    // CS testi (tilt): CS YUKSEKKEN saat gonder. Cip CS'i goruyorsa MISO serbest kalir (pullup=FFFF, pulldown=0000).
    {
        char b[120]; uint16_t up, dn;
        auto clk16 = [&]() { uint16_t r = 0; for (int i = 15; i >= 0; i--) { digitalWriteFast(PIN_MOSI, 1); spiDelay(); digitalWriteFast(PIN_SCK, HIGH); spiDelay(); r = (uint16_t)(r << 1) | (digitalReadFast(PIN_MISO) ? 1 : 0); digitalWriteFast(PIN_SCK, LOW); } return r; };
        digitalWriteFast(PIN_CS[1], HIGH);
        pinMode(pinNametoDigitalPin(PIN_MISO), INPUT_PULLUP);   clk16(); up = clk16();
        pinMode(pinNametoDigitalPin(PIN_MISO), INPUT_PULLDOWN); clk16(); dn = clk16();
        pinMode(pinNametoDigitalPin(PIN_MISO), INPUT);
        snprintf(b, sizeof b, "CS testi (tilt, CS yuksek): pullup=%04X pulldown=%04X  %s", up, dn,
                 (up == 0xFFFF && dn == 0x0000) ? "CS calisiyor" : "CS CIPE ULASMIYOR (cip hep secili)");
        Serial.println(b);
    }
    Serial.println("---");
    delay(100);
}
