// En basit BNO085 testi, Teensy 4.1. Kablolar ana sistemle aynı: SDA 18, SCL 19, 100 kHz.
// Ana firmware'deki raporların aynısı açılır (yön 100 Hz, jiroskop 100 Hz, yerçekimi 10 Hz); motor, enkoder,
// kontrol kesmesi YOK. Sürücüler kapalı (EN yüksek), lazer kapalı tutulur.
// Seri komutlar: '1' -> ana firmware'deki kontrol kesmesinin benzeri açılır (1 kHz, her seferinde 354 µs meşgul),
//                '0' -> kapatılır. Saniyede bir satır: s=<süre> mod=<0/1> rapor=<son 1 s> reset=<toplam> kesinti=<toplam>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

static Adafruit_BNO08x bno(-1);
static sh2_SensorValue_t val;
static bool ready = false;
static uint32_t nGrv = 0, nResets = 0, nStalls = 0, lastGrvMs = 0;
static IntervalTimer fakeCtrl;
static volatile bool fakeOn = false;

static void fakeIsr()                       // ana firmware'de kontrol kesmesi enkoder okurken ~354 µs sürüyor
{
    uint32_t t0 = micros();
    while (micros() - t0 < 354) { }
}

static bool enableReports()
{
    bool ok = bno.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);
    bno.enableReport(SH2_GRAVITY, 100000);
    bno.enableReport(SH2_GYROSCOPE_CALIBRATED, 10000);
    return ok;
}

void setup()
{
    // sürücüler kapalı, adım yok, lazer kapalı (ana firmware'deki pinler)
    const int en[3] = { 39, 36, 33 }, st[3] = { 41, 38, 35 };
    for (int i = 0; i < 3; i++) { digitalWrite(en[i], HIGH); pinMode(en[i], OUTPUT); digitalWrite(st[i], LOW); pinMode(st[i], OUTPUT); }
    digitalWrite(5, LOW); pinMode(5, OUTPUT);
    Serial.begin(115200);
    delay(500);
    Serial.println("BNO085 basit test (Teensy 4.1, orijinal Adafruit kutuphanesi)");
    Wire.begin();
    Wire.setClock(100000);
}

void loop()
{
    while (Serial.available()) {
        int c = Serial.read();
        if (c == '1' && !fakeOn) { fakeOn = true; fakeCtrl.priority(48); fakeCtrl.begin(fakeIsr, 1000); Serial.println("kesme benzeri ACIK"); }
        if (c == '0' && fakeOn)  { fakeOn = false; fakeCtrl.end(); Serial.println("kesme benzeri KAPALI"); }
    }
    if (!ready) {
        static uint32_t lastTry = 0;
        if (millis() - lastTry > 1000) {
            lastTry = millis();
            ready = bno.begin_I2C(0x4A, &Wire) && enableReports();
            Serial.println(ready ? "BNO085 baslatildi" : "BNO085 baslatilamadi, tekrar denenecek");
            lastGrvMs = millis();
        }
        return;
    }
    if (bno.wasReset()) { nResets++; enableReports(); Serial.print("SENSOR RESETI #"); Serial.println(nResets); }
    if (bno.getSensorEvent(&val) && val.sensorId == SH2_GAME_ROTATION_VECTOR) {
        if (millis() - lastGrvMs > 300) nStalls++;
        lastGrvMs = millis();
        nGrv++;
    }
    static uint32_t lastPrint = 0, lastN = 0;
    if (millis() - lastPrint >= 1000) {
        lastPrint = millis();
        Serial.print("s="); Serial.print(millis() / 1000);
        Serial.print(" mod="); Serial.print(fakeOn ? 1 : 0);
        Serial.print(" rapor="); Serial.print(nGrv - lastN);
        Serial.print(" reset="); Serial.print(nResets);
        Serial.print(" kesinti="); Serial.println(nStalls);
        lastN = nGrv;
    }
}
