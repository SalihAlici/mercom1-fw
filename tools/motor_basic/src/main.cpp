// PAN MOTORU: 50° sağa, 50° sola (başlangıca geri), sürekli tekrar. UART kullanılmaz.
// STEP=D7 (PG12)  DIR=D6 (PE9)  EN=D0 (PB7, düşük = etkin)
//
// Açı ANTEN (redüktör çıkışı) açısıdır. Adım sayısı = açı × redüktör / (1,8° / mikroadım).
// Aşağıdaki üç sabit gerçek düzenekle uyuşmuyorsa anten 50° yerine başka açı döner:
//   MS1/MS2 boşta ise 8 mikroadım, ikisi de 3V3'te ise 16 mikroadım.
#include <Arduino.h>
#include <math.h>

static const float ACI_DERECE = 50.0f;   // anten açısı
static const float REDUKTOR   = 5.0f;    // motor turu / anten turu
static const int   MIKROADIM  = 8;       // MS1/MS2 boşta = 8

static const PinName PIN_STEP = PG_12;
static const PinName PIN_DIR  = PE_9;
static const PinName PIN_EN   = PB_7;
static const PinName PIN_LED  = PB_0;

static const float HIZ_MIN = 100.0f;     // adım/s, kalkış ve duruş hızı
static const float HIZ_MAX = 500.0f;     // adım/s, seyir hızı
static const int   RAMPA   = 200;        // hızlanma ve yavaşlama adım sayısı

// n adım at; başta hızlan, sonda yavaşla (yüklü antende adım kaçırmasın)
static void git(long n, bool sag)
{
    digitalWriteFast(PIN_DIR, sag ? HIGH : LOW);
    delayMicroseconds(100);
    digitalWriteFast(PIN_LED, HIGH);
    for (long i = 0; i < n; i++) {
        long kenar = (i < n - 1 - i) ? i : (n - 1 - i);              // en yakın uca uzaklık
        float k = (kenar >= RAMPA) ? 1.0f : (float)kenar / (float)RAMPA;
        float hiz = HIZ_MIN + (HIZ_MAX - HIZ_MIN) * k;
        uint32_t periyot_us = (uint32_t)(1000000.0f / hiz);
        digitalWriteFast(PIN_STEP, HIGH);
        delayMicroseconds(100);
        digitalWriteFast(PIN_STEP, LOW);
        delayMicroseconds(periyot_us - 100);
    }
    digitalWriteFast(PIN_LED, LOW);
}

static long s_adim;

void setup()
{
    Serial.begin(115200);
    pinMode(pinNametoDigitalPin(PIN_STEP), OUTPUT); digitalWriteFast(PIN_STEP, LOW);
    pinMode(pinNametoDigitalPin(PIN_DIR),  OUTPUT); digitalWriteFast(PIN_DIR,  LOW);
    pinMode(pinNametoDigitalPin(PIN_EN),   OUTPUT); digitalWriteFast(PIN_EN,   HIGH);   // önce kapalı
    pinMode(pinNametoDigitalPin(PIN_LED),  OUTPUT); digitalWriteFast(PIN_LED,  LOW);
    s_adim = lroundf(ACI_DERECE * REDUKTOR / (1.8f / (float)MIKROADIM));
    delay(1500);
    Serial.print("\nPAN 50 derece testi. Adim sayisi = "); Serial.println(s_adim);
    digitalWriteFast(PIN_EN, LOW);                                                        // sürücü etkin
    delay(1500);
}

void loop()
{
    Serial.println("SAGA 50 derece");  git(s_adim, true);   delay(1500);
    Serial.println("SOLA 50 derece");  git(s_adim, false);  delay(1500);
}
