#include "bno085.h"
#include "../hw.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <math.h>

static Adafruit_BNO08x   s_bno(-1);
static sh2_SensorValue_t s_val;
static float    s_q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
static float    s_rpy[3] = { 0.0f, 0.0f, 0.0f };
static uint32_t s_count = 0;
static uint32_t s_resets = 0;
static uint32_t s_lastMs = 0;
static uint32_t s_intervalUs = 10000;
static uint8_t  s_ok = 0;

static float s_grav[3] = { 0.0f, 0.0f, 0.0f };   // sensör çerçevesinde yerçekimi, m/s²
static float s_gyro[3] = { 0.0f, 0.0f, 0.0f };   // sensör çerçevesinde açısal hız, rad/s
static uint32_t s_gyroCount = 0;

static uint8_t s_gyroOn = 1;                         // 29 Eyl deneyi: jiroskop raporu hattaki trafiği ikiye katlıyor

static bool enableReports()
{
    bool ok = s_bno.enableReport(SH2_GAME_ROTATION_VECTOR, s_intervalUs);
    s_bno.enableReport(SH2_GRAVITY, 100000);          // 10 Hz, tanı için
    if (s_gyroOn) s_bno.enableReport(SH2_GYROSCOPE_CALIBRATED, s_intervalUs);   // 100 Hz, nişan ileri beslemesi
    return ok;
}

// Jiroskop raporunu aç/kapat (kapalıyken ileri besleme 0 olur, PI takip sürer). Sensör çalışıyorsa hemen uygulanır.
extern "C" void bno085_set_gyro(int on)
{
    uint8_t want = on ? 1 : 0;
    if (want == s_gyroOn) return;
    s_gyroOn = want;
    if (s_ok) { I2C_BUSY_BEGIN(); s_bno.enableReport(SH2_GYROSCOPE_CALIBRATED, want ? s_intervalUs : 0); I2C_BUSY_END(); }
}

static void quatToRpy()
{
    float w = s_q[0], x = s_q[1], y = s_q[2], z = s_q[3];
    float r00 = 1.0f - 2.0f * (y * y + z * z);
    float r10 = 2.0f * (x * y + w * z);
    float r20 = 2.0f * (x * z - w * y);
    float r21 = 2.0f * (y * z + w * x);
    float r22 = 1.0f - 2.0f * (x * x + y * y);
    float s = -r20;
    if (s > 1.0f) s = 1.0f;
    if (s < -1.0f) s = -1.0f;
    const float k = 57.2957795f;
    s_rpy[0] = atan2f(r21, r22) * k;
    s_rpy[1] = asinf(s) * k;
    s_rpy[2] = atan2f(r10, r00) * k;
}

static bool s_opened = false;

// ---- tanı: reset sebebi izleme (varsayılan kapalı) ----
static uint32_t s_epoch = 0;
static uint8_t  s_causeTrack = 0;
static int      s_lastCause = -1;
static uint32_t s_causeHist[6] = { 0, 0, 0, 0, 0, 0 };

static void noteCause(int c)
{
    if (c < 0 || c > 5) c = 0;
    s_lastCause = c;
    s_causeHist[c]++;
}

// Sensöre ürün kimliğini sorar; yanıtın ilk alanı son resetin sebebidir. En fazla 0,5 s sürer (lib yaması).
static void queryCause()
{
    sh2_ProductIds_t ids;
    memset(&ids, 0, sizeof ids);
    if (sh2_getProdIds(&ids) == SH2_OK && ids.numEntries > 0) noteCause(ids.entry[0].resetCause);
    else noteCause(0);
}

extern "C" int bno085_init(uint32_t rate_hz)
{
    if (rate_hz == 0) rate_hz = 100;
    s_intervalUs = 1000000UL / rate_hz;
    // Kütüphane açık bir oturumu ikinci kez açamıyor (tek SHTP örneği); önce sh2 oturumunu kapat.
    // KOŞULSUZ: begin_I2C, oturumu açtıktan sonra ürün kimliği sorgusunda zaman aşımına düşerse false döner ama
    // oturum açık kalır. 29 Eyl: yalnız s_opened ile kapatınca örnek sızdı, sonraki bütün başlatmalar başarısız oldu
    // (sensör ACK veriyor, "BULUNAMADI"), ancak Teensy yeniden başlayınca düzeldi. sh2_close boş oturumda güvenli (lib yaması).
    I2C_BUSY_BEGIN();
    sh2_close();
    if (s_opened) { s_opened = false; delay(50); }
    s_ok = 0;
    bool began = s_bno.begin_I2C(0x4A, &Wire);
    board_i2c_pads(-1);        // kütüphane Wire.begin() ile pin kenar ayarını varsayılana döndürdü: yeniden uygula
    if (!began) { I2C_BUSY_END(); return 0; }
    s_opened = true;
    s_ok = enableReports() ? 1 : 0;
    I2C_BUSY_END();
    s_epoch++;
    s_lastMs = millis();   // akış kesildi kontrolü başlatma anından itibaren saysın, eski oturumdan değil
    return s_ok;
}

// Okuma aralığı: bir okumanın BİTİŞİNDEN sonrakinin başına kadar en az bu kadar µs hat boş kalır.
// Bu boşlukta kontrol kesmesi enkoderleri okuyabilir (I2C sürerken enkoder okunmaz, bkz. g_i2c_busy). 0 = boşluksuz.
static uint32_t s_pollGapUs = 0, s_lastPollEndUs = 0;
extern "C" void bno085_set_poll_gap(uint32_t us) { s_pollGapUs = us; }

static int pollInner(void);
extern "C" int bno085_poll(void)
{
    if (!s_ok) return 0;
    if (s_pollGapUs && (uint32_t)(micros() - s_lastPollEndUs) < s_pollGapUs) return 0;
    I2C_BUSY_BEGIN();
    int r = pollInner();
    I2C_BUSY_END();
    s_lastPollEndUs = micros();
    return r;
}

static int pollInner(void)
{
    if (s_bno.wasReset()) {
        s_resets++;
        s_epoch++;
        // raporları açmadan önce sor: sensör yeni açıldı, kuyruk boş. Kütüphane gelen olayları getSensorEvent'in
        // verdiği tampona yazar; o tampon ilk rapordan önce henüz atanmamış olur, bu yüzden ilk rapordan sonra sorulur.
        if (s_causeTrack && s_count > 0) queryCause();
        enableReports();
    }
    if (!s_bno.getSensorEvent(&s_val)) return 0;
    if (s_val.sensorId == SH2_GYROSCOPE_CALIBRATED) {
        s_gyro[0] = s_val.un.gyroscope.x; s_gyro[1] = s_val.un.gyroscope.y; s_gyro[2] = s_val.un.gyroscope.z;
        s_gyroCount++;
        return 0;
    }
    if (s_val.sensorId == SH2_GRAVITY) {
        s_grav[0] = s_val.un.gravity.x; s_grav[1] = s_val.un.gravity.y; s_grav[2] = s_val.un.gravity.z;
        return 0;
    }
    if (s_val.sensorId != SH2_GAME_ROTATION_VECTOR) return 0;
    float w = s_val.un.gameRotationVector.real, x = s_val.un.gameRotationVector.i;
    float y = s_val.un.gameRotationVector.j,    z = s_val.un.gameRotationVector.k;
    float n2 = w * w + x * x + y * y + z * z;
    if (n2 < 0.9f || n2 > 1.1f) return 0;      // bozuk paket, birim quaternion değil
    s_q[0] = w; s_q[1] = x; s_q[2] = y; s_q[3] = z;
    quatToRpy();
    s_count++;
    s_lastMs = millis();
    return 1;
}

extern "C" void bno085_quat(float q[4]) { for (int i = 0; i < 4; i++) q[i] = s_q[i]; }
extern "C" void bno085_gravity(float g[3]) { for (int i = 0; i < 3; i++) g[i] = s_grav[i]; }
extern "C" void bno085_gyro(float w[3]) { for (int i = 0; i < 3; i++) w[i] = s_gyro[i]; }
extern "C" uint32_t bno085_gyro_count(void) { return s_gyroCount; }
extern "C" void bno085_rpy(float rpy[3]) { for (int i = 0; i < 3; i++) rpy[i] = s_rpy[i]; }
extern "C" uint32_t bno085_count(void) { return s_count; }
extern "C" uint32_t bno085_resets(void) { return s_resets; }
extern "C" int bno085_ok(void) { return s_ok; }
extern "C" uint32_t bno085_last_ms(void) { return s_lastMs; }

extern "C" uint32_t bno085_epoch(void) { return s_epoch; }
extern "C" volatile uint32_t mercom_i2c_cnt[10];   // lib/Adafruit_BNO08x yaması: I2C tanı sayaçları
extern "C" void bno085_i2c_counts(uint32_t c[10]) { for (int i = 0; i < 10; i++) c[i] = mercom_i2c_cnt[i]; }
extern "C" void bno085_cause_track(int on) { s_causeTrack = on ? 1 : 0; }
extern "C" int  bno085_cause_tracking(void) { return s_causeTrack; }
extern "C" int  bno085_last_cause(void) { return s_lastCause; }
extern "C" void bno085_cause_hist(uint32_t hist[6]) { for (int i = 0; i < 6; i++) hist[i] = s_causeHist[i]; }
