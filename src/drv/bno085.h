#pragma once
// BNO085 IMU, I2C, Game Rotation Vector (manyetometresiz). Adafruit BNO08x kütüphanesi sarılır.
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int      bno085_init(uint32_t rate_hz);   // 1 ok
int      bno085_poll(void);               // 1: yeni quaternion geldi, ana döngüden sık çağrılır
void     bno085_quat(float q[4]);         // w, x, y, z
void     bno085_gravity(float g[3]);      // sensör çerçevesinde yerçekimi, m/s²; aşağı bakan eksen -9.8 okur
void     bno085_gyro(float w[3]);         // sensör çerçevesinde açısal hız, rad/s (kalibreli jiroskop, 100 Hz)
uint32_t bno085_gyro_count(void);
void     bno085_rpy(float rpy_deg[3]);    // roll, pitch, yaw derece; R = Rz(yaw) Ry(pitch) Rx(roll)
uint32_t bno085_count(void);
uint32_t bno085_resets(void);
int      bno085_ok(void);
uint32_t bno085_last_ms(void);            // son rapor zamanı
void     bno085_set_gyro(int on);             // jiroskop raporu (ileri besleme) aç/kapat
void     bno085_set_poll_gap(uint32_t us);    // iki okuma arası en az bekleme, µs (0 = boşluksuz)
uint32_t bno085_epoch(void);
// I2C tanı sayaçları: 0 başlık okunamadı, 1 gövde okunamadı, 2 büyük paket, 3 geçersiz kanal, 4 yazma hatası,
// 5 PLTF (saat uzatma zaman aşımı), 6 NACK, 7 hat kaybı, 8 FIFO hatası, 9 başarılı paket
void     bno085_i2c_counts(uint32_t c[10]);              // sensör her yeniden başladığında (kendiliğinden ya da init) artar: yaw referansı yenilendi

// Tanı: sensörün kendi bildirdiği reset sebebi (SH-2 ürün kimliği yanıtı). İzleme kapalıyken uçuş yolu değişmez.
// Sebep kodları: 1 güç açılışı, 2 yazılım reseti, 3 sensörün iç bekçi köpeği, 4 RST pini, 5 diğer; 0 cevap yok
void     bno085_cause_track(int on);              // 1: her resetten sonra sebebi sor ve say
int      bno085_cause_tracking(void);
int      bno085_last_cause(void);                 // son resetin sebebi, -1 henüz yok
void     bno085_cause_hist(uint32_t hist[6]);     // sebep koduna göre sayaçlar

#ifdef __cplusplus
}
#endif
