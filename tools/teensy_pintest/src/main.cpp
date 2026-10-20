// TEENSY 4.1 PİN SINAMASI (MERCOM-1 pin planı): her çıkış pini sırayla 1,5 s YÜKSEK yapılır, geri okunur ve
// seri porta yazılır (multimetreyle Teensy pininde ve modül pininde 3,3 V doğrulanır). Sonra üç enkoder yazılım SPI ile
// okunup ham değer yazılır, MISO seviyesi ve I2C taraması (BNO085 0x4A) yazılır. Döngü ~35 s.
#include <Arduino.h>
#include <Wire.h>

struct P { uint8_t pin; const char *ad; };
static const P CIKISLAR[] = {
    { 41, "STEP pan" }, { 40, "DIR pan" }, { 39, "EN pan" },
    { 38, "STEP tilt" }, { 37, "DIR tilt" }, { 36, "EN tilt" },
    { 35, "STEP CE" }, { 34, "DIR CE" }, { 33, "EN CE" },
    { 6, "enkoder SCK" }, { 11, "enkoder MOSI" }, { 8, "CS pan" }, { 9, "CS tilt" }, { 12, "CS CE" },
    { 5, "lazer" }, { 26, "LoRa M0" }, { 27, "LoRa M1" },
};
static const uint8_t PIN_MISO_ = 7, PIN_AUX_ = 28, CS[3] = { 8, 9, 12 };
static const char *ENK[3] = { "pan", "tilt", "ce" };

static uint16_t xfer16(uint8_t cs, uint16_t tx)
{
    uint16_t rx = 0;
    digitalWriteFast(cs, LOW); delayMicroseconds(5);
    for (int i = 15; i >= 0; i--) {
        digitalWriteFast(6, HIGH);
        digitalWriteFast(11, (tx >> i) & 1); delayMicroseconds(5);
        digitalWriteFast(6, LOW);
        rx = (uint16_t)(rx << 1) | (digitalReadFast(PIN_MISO_) ? 1 : 0);
        delayMicroseconds(5);
    }
    digitalWriteFast(cs, HIGH); digitalWriteFast(11, LOW); delayMicroseconds(5);
    return rx;
}

void setup()
{
    Serial.begin(115200);
    for (auto &p : CIKISLAR) { pinMode(p.pin, OUTPUT); digitalWrite(p.pin, LOW); }
    digitalWrite(39, HIGH); digitalWrite(36, HIGH); digitalWrite(33, HIGH);   // EN pasif
    digitalWrite(8, HIGH); digitalWrite(9, HIGH); digitalWrite(12, HIGH);     // CS bosta yuksek
    pinMode(PIN_MISO_, INPUT_PULLUP); pinMode(PIN_AUX_, INPUT); pinMode(10, INPUT);
    pinMode(13, OUTPUT);
    Wire.begin(); Wire.setClock(100000);
    delay(500);
    Serial.println("\n=== TEENSY 4.1 PIN SINAMASI (MERCOM-1) ===");
}

void loop()
{
    for (auto &p : CIKISLAR) {
        int idle = (p.pin == 39 || p.pin == 36 || p.pin == 33 || p.pin == 8 || p.pin == 9 || p.pin == 12) ? HIGH : LOW;
        digitalWrite(p.pin, !idle); digitalWrite(13, HIGH);
        delay(50);
        Serial.print("pin "); Serial.print(p.pin); Serial.print(" ("); Serial.print(p.ad); Serial.print(") -> ");
        Serial.print(idle ? "DUSUK 0 V" : "YUKSEK 3,3 V"); Serial.print("   geri okuma="); Serial.println(digitalRead(p.pin));
        delay(1450);
        digitalWrite(p.pin, idle); digitalWrite(13, LOW);
        delay(300);
    }
    Serial.print("MISO (7) bosta seviye: "); Serial.print(digitalRead(PIN_MISO_)); Serial.print("   LoRa AUX (28): "); Serial.println(digitalRead(PIN_AUX_));
    for (int i = 0; i < 3; i++) {
        xfer16(CS[i], 0xFFFF);                      // NOP + acı okuma komutu (0x3FFF | parite = 0xFFFF)
        uint16_t r = xfer16(CS[i], 0xFFFF);
        Serial.print("enkoder "); Serial.print(ENK[i]); Serial.print(" (CS "); Serial.print(CS[i]); Serial.print("): ham 0x");
        Serial.print(r, HEX);
        if (r == 0x0000) Serial.println("  -> cevap yok (MISO hep 0: besleme/GND/MISO teli ya da CS ulasmiyor)");
        else if (r == 0xFFFF) Serial.println("  -> MISO hep 1");
        else { Serial.print("  aci="); Serial.print((r & 0x3FFF) * 360.0f / 16384.0f, 1); Serial.println(" derece  -> CEVAP VAR"); }
    }
    int n = 0; uint8_t found[8];
    for (uint8_t a = 0x08; a <= 0x77 && n < 8; a++) { Wire.beginTransmission(a); if (Wire.endTransmission() == 0) found[n++] = a; }
    Serial.print("I2C: "); if (!n) Serial.println("cihaz yok (BNO085 0x4A bekleniyor)");
    for (int i = 0; i < n; i++) { Serial.print("0x"); Serial.print(found[i], HEX); Serial.print(found[i] == 0x4A ? " (BNO085) " : " "); }
    if (n) Serial.println();
    Serial.println("--- dongu bitti, bastan ---");
}
