#pragma once
// Nişan katmanı: IMU başlıkta. Kilit = başlığın bakış doğrultusu (dünya çerçevesi, d_N0).
// Otomatik modda her IMU örneğinde (100 Hz) şu anki bakış doğrultusu d_N ile kilit arasındaki
// hata dönüşü hesaplanır, başlık çerçevesine çevrilir ve PI denetleyiciyle pan/tilt hızına dönüşür:
//   pan_hız  = sign_pan  * (Kp*e_z + Ki*∫e_z) / cos(el)     (pan tabanın z ekseni etrafında; bakışı cos(el) ile çevirir)
//   tilt_hız = sign_tilt * -(Kp*e_y + Ki*∫e_y)              (+y etrafında dönüş bakışı aşağı indirir)
// Başlık çerçevesi (imu_mount_cal ile aynı): x bakış, z yukarı, y sol. R_IH = Rz(y)Ry(p)Rx(r) mount parametreleri.
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void  pointing_init(void);
void  pointing_set_quat(const float q[4]);      // yeni IMU örneği (w,x,y,z): bakış doğrultusunu günceller
void  pointing_set_gyro(const float w_s[3]);    // sensör çerçevesinde açısal hız (rad/s); başlık çerçevesine çevrilir
int   pointing_lock(void);                      // şu anki bakışı hedef yap; 1 ok, 0 IMU örneği yok
void  pointing_unlock(void);
int   pointing_locked(void);
void  pointing_set_tilt(float deg);             // tilt eklem açısı (adım sayacı, derece); paralaks için her örnekten ve kilitten önce verilir
// Yön bağlama: tabanın yaw'ı her örnekte R_NB = R_NH * R_BH^T ile izlenir (R_BH = Rz(pan) Ry(-tilt)).
// IMU yeniden başladığında (ya da örnek akışı kesildiğinde) birkaç örnek beklenir, sonra yaw düzeltmesi
// tabanın yaw'ı kesintiden önceki değerine dönecek şekilde ayarlanır. Taban kendi ekseninde dönmüyor varsayılır.
void  pointing_set_joints(float pan_deg, float tilt_deg);   // eklem açıları (pan enkoderden), her örnekten önce
void  pointing_imu_restarted(void);             // IMU yeniden başladı ya da akış kesildi: yön yeniden bağlanacak
uint32_t pointing_anchor_count(void);           // yapılan yön bağlama sayısı
float pointing_anchor_last(void);               // son bağlamadaki yaw düzeltmesi, derece
float pointing_plx_shift(void);                 // paralaks referansının kilit yönünden sapması, derece (0 = düzeltme yok)
void  pointing_reset_integrators(void);
int   pointing_step(float dt, float v_pan_act, float v_tilt_act, float *v_pan, float *v_tilt);  // kilitliyse hız komutları (derece/s); 1 ok
                                                // v_*_act: eksenlerin şu anki gerçek hızı (kendi hareketimiz ölçülen hızdan çıkarılır)
void  pointing_error(float *total, float *side, float *up);  // derece: toplam, yanal (pan yönü, +sol), dikey (+yukarı)
float pointing_elevation(void);                 // bakışın dünya elevasyonu, derece
float pointing_azimuth(void);                   // bakışın dünya azimutu, derece 0..360 (IMU yaw referansına göre)
int   pointing_have_imu(void);
int   pointing_pose_ok(void);                   // 1: başlık normal pozda (zenite yakın değil, devrilmemiş); pan hesabı geçerli

#ifdef __cplusplus
}
#endif
