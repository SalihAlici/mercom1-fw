// KART SAĞLIK SINAMASI: üç kullanıcı LED'i sırayla yanar (yeşil LD1, sarı LD2, kırmızı LD3), seri porta sayaç yazılır.
// Hiçbir dış pine dokunmaz; işlemci çalışıyor mu, programlanabiliyor mu diye bakmak için.
#include <Arduino.h>

static const PinName LEDLER[3] = { PB_0, PE_1, PB_14 };   // LD1 yeşil, LD2 sarı, LD3 kırmızı

void setup()
{
    Serial.begin(115200);
    for (int i = 0; i < 3; i++) { pinMode(pinNametoDigitalPin(LEDLER[i]), OUTPUT); digitalWriteFast(LEDLER[i], LOW); }
    Serial.println("\nLED SINAMASI basladi: yesil, sari, kirmizi sirayla yanar");
}

void loop()
{
    static uint32_t n = 0;
    for (int i = 0; i < 3; i++) {
        digitalWriteFast(LEDLER[i], HIGH); delay(250);
        digitalWriteFast(LEDLER[i], LOW);
    }
    Serial.print("calisiyor, tur "); Serial.println(++n);
}
