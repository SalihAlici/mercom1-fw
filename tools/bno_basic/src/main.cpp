// En basit BNO085 testi. Kablolar: VIN -> IOREF (3,3 V), GND -> GND, SCL -> D15, SDA -> D14.
// Seri port 115200. Her saniye: I2C tarama sonucu ve roll/pitch/yaw. Baska hicbir sey yok.
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

static Adafruit_BNO08x bno(-1);
static sh2_SensorValue_t val;
static bool ready = false;
static uint32_t nReports = 0;

static int scanBus()
{
    int n = 0;
    for (uint8_t a = 8; a < 120; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) { Serial.print("  I2C adres 0x"); Serial.println(a, HEX); n++; }
    }
    return n;
}

static bool startSensor()
{
    if (!bno.begin_I2C(0x4A, &Wire)) return false;
    return bno.enableReport(SH2_GAME_ROTATION_VECTOR, 20000);   // 50 Hz
}

void setup()
{
    Serial.begin(115200);
    delay(300);
    Serial.println("BNO085 basit test");
    Wire.setSCL(PB8);   // D15
    Wire.setSDA(PB9);   // D14
    Wire.begin();
    Wire.setClock(100000);
}

void loop()
{
    static uint32_t lastPrint = 0;

    if (ready && bno.getSensorEvent(&val) && val.sensorId == SH2_GAME_ROTATION_VECTOR) {
        nReports++;
        float w = val.un.gameRotationVector.real, x = val.un.gameRotationVector.i;
        float y = val.un.gameRotationVector.j,    z = val.un.gameRotationVector.k;
        float roll  = atan2f(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)) * 57.2958f;
        float pitch = asinf(fmaxf(-1, fminf(1, 2 * (w * y - z * x)))) * 57.2958f;
        float yaw   = atan2f(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) * 57.2958f;
        if (millis() - lastPrint >= 200) {
            lastPrint = millis();
            Serial.print("roll="); Serial.print(roll, 2);
            Serial.print("  pitch="); Serial.print(pitch, 2);
            Serial.print("  yaw="); Serial.print(yaw, 2);
            Serial.print("  rapor="); Serial.println(nReports);
        }
    }

    if (!ready && millis() - lastPrint >= 1000) {
        lastPrint = millis();
        Serial.print("tarama: ");
        int n = scanBus();
        if (n == 0) Serial.println("  hic cihaz yok");
        ready = startSensor();
        Serial.println(ready ? "BNO085 baslatildi, raporlar geliyor..." : "BNO085 baslatilamadi, tekrar denenecek");
    }
}
