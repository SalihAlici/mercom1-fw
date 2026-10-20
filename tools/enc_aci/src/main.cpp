// ENKODER AÇI OKUYUCU: üç AS5048A'nın açısını saniyede 10 kez terminale yazar. Seri port 115200.
//
//   motor  = enkoderin okuduğu mil açısı, 0–360 derece. Mıknatıs motorun ARKA milinde, yani redüktörden ÖNCE.
//   anten  = açılıştan (ya da "sifir" komutundan) beri dönülen açı, tur sayılarak ve redüktör oranına bölünerek
//            bulunur (pan 5, tilt 5, ce 1). Kart açıldığı andaki konum 0 kabul edilir.
//   AGC    = mıknatıs uzaklığı göstergesi: 60–200 iyi, 255 mıknatıs uzak, 0 çok yakın.
//
// Komutlar (yaz, Enter):  sifir = anten açılarını sıfırla     hiz <n> = saniyede n satır (1–50)
//
// Bağlantı (CN7, 26 Eyl dizilimi):  SCK=D13 (PA5)  MISO=D12 (PA6)  CS pan=D11 (PB5)  tilt=D10 (PD14)  ce=D9 (PD15)  MOSI=D8 (PF3)
//   Adaptörün 5V ve 3.3V pinlerinin İKİSİ de 3V3'e, GND ortak.
// SPI yazılımla (bit-bang, ~250 kHz, mod 1) ve düşük slew hızlı pinlerle sürülür; uzun kabloda çalışan yöntem bu.
#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "stm32h7xx_ll_gpio.h"

static const PinName PIN_SCK  = PA_5;
static const PinName PIN_MISO = PA_6;
static const PinName PIN_MOSI = PF_3;
static const PinName PIN_CS[3] = { PB_5, PD_14, PD_15 };
static const char   *AD[3]     = { "pan ", "tilt", "ce  " };
static const float   ORAN[3]   = { 5.0f, 5.0f, 1.0f };

enum Durum { YOK, HATA, UZAK, IYI };

static long     s_toplam[3] = { 0, 0, 0 };      // açılıştan beri sayım (14 bit/tur), tur sayılmış
static uint16_t s_onceki[3];
static bool     s_ilk[3] = { true, true, true };
static uint16_t s_ham[3];
static uint32_t s_ok[3], s_hata[3];
static Durum    s_durum[3] = { YOK, YOK, YOK };
static int      s_agc[3];
static uint32_t s_satirMs = 100;

static inline void spiDelay(void) { for (volatile int i = 0; i < 36; i++) { } }

static void slowPin(PinName p)
{
    pinMode(pinNametoDigitalPin(p), OUTPUT);
    LL_GPIO_SetPinSpeed(get_GPIO_Port(STM_PORT(p)), STM_LL_GPIO_PIN(p), LL_GPIO_SPEED_FREQ_LOW);
}

static uint16_t xfer16(int ch, uint16_t tx)
{
    uint16_t rx = 0;
    digitalWriteFast(PIN_MOSI, (tx >> 15) & 1); spiDelay();
    digitalWriteFast(PIN_CS[ch], LOW); spiDelay(); spiDelay(); spiDelay();
    for (int i = 15; i >= 0; i--) {
        digitalWriteFast(PIN_MOSI, (tx >> i) & 1); spiDelay();
        digitalWriteFast(PIN_SCK, HIGH); spiDelay();
        rx = (uint16_t)(rx << 1) | (digitalReadFast(PIN_MISO) ? 1 : 0);   // düşen kenardan önce örnekle
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

// Yazmaç oku: komut + NOP. 1 = geçerli, 0 = hata, -1 = hatta çip yok (hep 1 ya da hep 0)
static int readReg(int ch, uint16_t reg, uint16_t *out)
{
    xfer16(ch, withParity(0x4000 | reg));
    uint16_t r = xfer16(ch, withParity(0x4000 | 0x0000));
    if (r == 0xFFFF || r == 0x0000) return -1;
    if (!parityOk(r) || (r & 0x4000)) {
        xfer16(ch, withParity(0x4000 | 0x0001));    // hata bayrağını temizle
        xfer16(ch, withParity(0x4000 | 0x0000));
        return 0;
    }
    *out = r & 0x3FFF;
    return 1;
}

static void aciOku(int ch)
{
    uint16_t v = 0;
    int r = readReg(ch, 0x3FFF, &v);
    if (r != 1) { s_hata[ch]++; if (r < 0) { s_durum[ch] = YOK; s_ilk[ch] = true; } return; }
    s_ok[ch]++;
    if (s_durum[ch] != IYI) return;                 // mıknatıs yokken gelen açı gürültüdür, tur sayma
    if (s_ilk[ch]) { s_onceki[ch] = v; s_ilk[ch] = false; }
    int d = (int)v - (int)s_onceki[ch];
    if (d > 8192) d -= 16384; else if (d < -8192) d += 16384;
    s_toplam[ch] += d; s_onceki[ch] = v; s_ham[ch] = v;
}

static void taniOku(int ch)
{
    uint16_t v = 0;
    int r = readReg(ch, 0x3FFD, &v);
    if (r < 0) { s_durum[ch] = YOK; s_ilk[ch] = true; return; }
    if (r == 0) { if (s_durum[ch] == YOK) s_durum[ch] = HATA; return; }
    s_agc[ch] = v & 0xFF;
    bool ocf = (v >> 8) & 1, uzak = (v >> 11) & 1;
    Durum yeni = (!ocf || uzak) ? UZAK : IYI;
    if (yeni == IYI && s_durum[ch] != IYI) s_ilk[ch] = true;   // yeniden geçerli oldu: sıçrama sayılmasın
    s_durum[ch] = yeni;
}

static void satirYaz()
{
    for (int ch = 0; ch < 3; ch++) {
        Serial.print(AD[ch]); Serial.print(" ");
        if (s_durum[ch] == YOK)       Serial.print("bagli degil                        ");
        else if (s_durum[ch] == HATA) Serial.print("HABERLESME HATASI (tel/besleme)    ");
        else if (s_durum[ch] == UZAK) { Serial.print("MIKNATIS UZAK  AGC "); Serial.print(s_agc[ch]); Serial.print("             "); }
        else {
            float motor = (float)s_ham[ch] * (360.0f / 16384.0f);
            float anten = (float)s_toplam[ch] * (360.0f / 16384.0f) / ORAN[ch];
            Serial.print("motor "); if (motor < 100.0f) Serial.print(" "); if (motor < 10.0f) Serial.print(" ");
            Serial.print(motor, 2);
            Serial.print("  anten "); if (anten >= 0.0f) Serial.print("+");
            Serial.print(anten, 2);
            Serial.print("  AGC "); Serial.print(s_agc[ch]);
        }
        Serial.print(ch < 2 ? "  |  " : "");
    }
    Serial.println();
}

static void komut(char *s)
{
    char *k = strtok(s, " \t"), *a = strtok(nullptr, " \t");
    if (!k) return;
    if (!strcmp(k, "sifir") || !strcmp(k, "s")) {
        for (int i = 0; i < 3; i++) s_toplam[i] = 0;
        Serial.println(">> anten acilari sifirlandi");
    } else if (!strcmp(k, "hiz") && a && atoi(a) >= 1 && atoi(a) <= 50) {
        s_satirMs = 1000u / (uint32_t)atoi(a);
        Serial.print(">> saniyede "); Serial.print(atoi(a)); Serial.println(" satir");
    } else {
        Serial.println(">> komutlar: sifir | hiz <1-50>");
    }
}

void setup()
{
    Serial.begin(115200);
    slowPin(PIN_SCK); slowPin(PIN_MOSI);
    for (int i = 0; i < 3; i++) { slowPin(PIN_CS[i]); digitalWriteFast(PIN_CS[i], HIGH); }
    pinMode(pinNametoDigitalPin(PIN_MISO), INPUT_PULLUP);      // hat boştayken kararlı 0xFFFF okunsun
    digitalWriteFast(PIN_SCK, LOW);
    delay(1000);
    Serial.println("\nENKODER ACI OKUYUCU   motor = mil acisi 0-360   anten = acilistan beri donus (reduktor hesaba katilmis)");
    Serial.println("komutlar: sifir | hiz <1-50>     AGC 60-200 iyi, 255 miknatis uzak");
    for (int ch = 0; ch < 3; ch++) taniOku(ch);
}

void loop()
{
    static char buf[32]; static uint8_t n = 0;
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\r' || c == '\n') { if (n) { buf[n] = 0; komut(buf); n = 0; } }
        else if (c >= 0x20 && c < 0x7F && n < sizeof buf - 1) buf[n++] = (char)tolower((unsigned char)c);
    }

    for (int ch = 0; ch < 3; ch++) aciOku(ch);      // ~1 kHz: hızlı dönen milde de tur kaçırmaz

    static uint32_t sonSatir = 0;
    uint32_t simdi = millis();
    if (simdi - sonSatir >= s_satirMs) {
        sonSatir = simdi;
        for (int ch = 0; ch < 3; ch++) taniOku(ch);
        satirYaz();
    }
    delayMicroseconds(300);
}
