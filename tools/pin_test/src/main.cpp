// CE SÜRÜCÜ HATTI PİN SINAMASI: STEP (D3/PE13), DIR (D2/PG14), EN (D40/PE10) sırayla 3 s yüksek tutulur,
// aralarda hepsi düşük. Multimetreyle sürücü modülünün STEP, DIR, EN pinlerinde 3,3 V'un sırayla geldiği görülür.
// Seri port 115200'e hangi pinin yüksek olduğu yazılır. Yeşil LED de aynı anda yanar.
#include <Arduino.h>

struct P { const char *ad; PinName pin; PinName led; };
// Her pin yüksekken kendi LED'i yanar: STEP = yeşil LD1, DIR = sarı LD2, EN = kırmızı LD3
static const P PINLER[] = { { "STEP CE (D3 / PE13)", PE_13, PB_0 }, { "DIR  CE (D2 / PG14)", PG_14, PE_1 }, { "EN   CE (D40 / PE10)", PE_10, PB_14 } };

void setup()
{
    Serial.begin(115200);
    for (auto &p : PINLER) {
        pinMode(pinNametoDigitalPin(p.pin), OUTPUT); digitalWriteFast(p.pin, LOW);
        pinMode(pinNametoDigitalPin(p.led), OUTPUT); digitalWriteFast(p.led, LOW);
    }
    Serial.println("\nPIN SINAMASI: STEP (yesil LED), DIR (sari LED), EN (kirmizi LED) sirayla 4 s yuksek, 2 s hepsi dusuk");
}

void loop()
{
    for (auto &p : PINLER) {
        digitalWriteFast(p.pin, HIGH); digitalWriteFast(p.led, HIGH);
        Serial.print("YUKSEK: "); Serial.println(p.ad);
        delay(4000);
        digitalWriteFast(p.pin, LOW); digitalWriteFast(p.led, LOW);
        Serial.println("hepsi dusuk");
        delay(2000);
    }
}
