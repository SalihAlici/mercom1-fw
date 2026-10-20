// LoRa yer istasyonu köprüsü: USB seri <-> Serial4 (E22-900T22D, 9600 8N1).
// 30 Eyl: E22'ler SABİT NOKTALI (adresli) modda: yer istasyonu 0x4D02, terminal 0x4D01, kanal 15 (865,125 MHz), NETID 0x4D,
// şifre anahtarı ortak (tools/lora_ayar.py). Bilgisayardan gelen baytlar paketlenip başına terminalin adresi + kanal yazılır;
// gelen paketlerin 3 baytlık başlığını modül kendisi çıkarır.
// Yer istasyonu bağlantısı (30 Eyl): Serial4 (TX4 17 -> E22 RXD, RX4 16 <- E22 TXD), M0 37, M1 39, AUX 41.
// (Terminaldeki E22 ise Serial6 24/25, M0 26, M1 27, AUX 28.) M0 = M1 = 0: iletim modu.
// Protokol çerçeveleri (MERCOM) olduğu gibi geçer; köprü içeriğe bakmaz. LED 13 veri geçtikçe yanar.
// Ayar modu: bilgisayar portu 9601 baud ile açarsa M1 = 1 (E22 ayar modu, C1/C0 komutları doğrudan modüle gider);
// başka her hızda M1 = 0 (şeffaf). Portu 134 baud ile açmak Teensy'yi yükleme moduna alır (Teensy çekirdeği).
#include <Arduino.h>

static const int PIN_M0 = 37, PIN_M1 = 39, PIN_AUX = 41, PIN_LED = 13;
#define LORA Serial4
static const uint8_t HEDEF[3] = { 0x4D, 0x01, 0x0F };   // terminal adresi + kanal
static uint8_t txMem[2048], rxMem[2048];

void setup()
{
    digitalWrite(PIN_M0, LOW); pinMode(PIN_M0, OUTPUT);
    digitalWrite(PIN_M1, LOW); pinMode(PIN_M1, OUTPUT);
    pinMode(PIN_AUX, INPUT);
    pinMode(PIN_LED, OUTPUT);
    Serial.begin(115200);                  // USB (hız önemsiz)
    LORA.begin(9600);                   // E22 fabrika ayarı; ayar modunda da hep 9600
    LORA.addMemoryForWrite(txMem, sizeof txMem);
    LORA.addMemoryForRead(rxMem, sizeof rxMem);
}

void loop()
{
    static uint32_t ledMs = 0, lastBaud = 0;
    uint32_t b = Serial.baud();
    if (b != lastBaud) {                   // bilgisayar portu yeni hızla açtı: şeffaf / ayar modu seçimi
        lastBaud = b;
        digitalWrite(PIN_M1, b == 9601 ? HIGH : LOW);
        delay(80);
        while (LORA.available()) LORA.read();
        if (b == 9603) {                   // tarama: bütün donanım seri portlarını 9600'de 12 s dinle, gelen baytları say
            HardwareSerial *ports[8] = { &Serial1, &Serial2, &Serial3, &Serial4, &Serial5, &Serial6, &Serial7, &Serial8 };
            const int rxpin[8] = { 0, 7, 15, 16, 21, 25, 28, 34 };
            uint32_t cnt[8] = { 0 }; char ilk[8][24]; int il[8] = { 0 };
            for (int i = 0; i < 8; i++) { if (i != 3) ports[i]->begin(9600); }
            uint32_t t0 = millis();
            while (millis() - t0 < 12000) {
                for (int i = 0; i < 8; i++) while (ports[i]->available()) {
                    int c = ports[i]->read(); cnt[i]++;
                    if (il[i] < 23 && c >= 32 && c < 127) { ilk[i][il[i]++] = (char)c; ilk[i][il[i]] = 0; }
                }
            }
            for (int i = 0; i < 8; i++) Serial.printf("Serial%d (RX pini %d): %lu bayt %s\r\n", i + 1, rxpin[i], (unsigned long)cnt[i], il[i] ? ilk[i] : "");
            for (int i = 0; i < 8; i++) { if (i != 3) ports[i]->end(); }
        }
        if (b == 9602) {                   // öz sınama: AUX seviyesi ve E22 ayarları düz metin olarak
            delay(500);
            Serial.printf("kopru ozsinama: AUX(41)=%d M0(37)=%d M1(39)=%d\r\n", digitalRead(PIN_AUX), digitalRead(PIN_M0), digitalRead(PIN_M1));
            digitalWrite(PIN_M1, HIGH); delay(100);
            while (LORA.available()) LORA.read();
            const uint8_t cmd[3] = { 0xC1, 0x00, 0x09 };
            LORA.write(cmd, 3); LORA.flush();
            uint8_t rx[16]; int n = 0; uint32_t t0 = millis();
            while (millis() - t0 < 400 && n < 12) { while (LORA.available() && n < 16) rx[n++] = (uint8_t)LORA.read(); }
            digitalWrite(PIN_M1, LOW); delay(80);
            Serial.printf("E22 ayar cevabi (%d bayt):", n);
            for (int i = 0; i < n; i++) Serial.printf(" %02X", rx[i]);
            Serial.printf("%s\r\n", n ? "" : "  CEVAP YOK");
            Serial.printf("AUX sonra=%d\r\n", digitalRead(PIN_AUX));
        }
    }
    bool act = false;
    // bilgisayar -> LoRa: kısa sessizlik (3 ms) ya da 200 bayt olunca tek paket, başına hedef adres + kanal.
    // Ayar modunda (9601 baud) baytlar başlıksız, olduğu gibi geçer (C0/C1 komutları).
    static uint8_t pk[208]; static int pn = 0; static uint32_t lastIn = 0;
    while (Serial.available() > 0 && pn < 200) { pk[pn++] = (uint8_t)Serial.read(); lastIn = micros(); act = true; }
    if (pn > 0 && (pn >= 200 || (uint32_t)(micros() - lastIn) > 3000)) {
        if (lastBaud == 9601) LORA.write(pk, pn);
        else if (LORA.availableForWrite() >= pn + 3) { LORA.write(HEDEF, 3); LORA.write(pk, pn); }
        pn = 0;
    }
    // LoRa -> bilgisayar
    int n = LORA.available();
    if (n > 0) {
        uint8_t buf[64];
        if (n > (int)sizeof buf) n = sizeof buf;
        for (int i = 0; i < n; i++) buf[i] = (uint8_t)LORA.read();
        Serial.write(buf, n);
        act = true;
    }
    if (act) { digitalWriteFast(PIN_LED, HIGH); ledMs = millis(); }
    if (millis() - ledMs > 30) digitalWriteFast(PIN_LED, LOW);
}
