#pragma once
// Parametre tablosu. Her parametre bir float, kimlikle erişilir.
// Kimlikler tools/mercom_link.py ile aynı sırada olmalı.
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    P_KP_PAN = 0, P_KP_TILT, P_KP_CE,            // konum takipçisi P kazancı [1/s]
    P_KI_PAN,     P_KI_TILT, P_KI_CE,            // integral kazancı [1/s^2]
    P_VMAX_PAN,   P_VMAX_TILT, P_VMAX_CE,        // hız sınırı [derece/s]
    P_AMAX_PAN,   P_AMAX_TILT, P_AMAX_CE,        // ivme sınırı [derece/s^2]
    P_DPS_PAN,    P_DPS_TILT, P_DPS_CE,          // mikroadım başına derece
    P_ENC_OFF_PAN, P_ENC_OFF_TILT, P_ENC_OFF_CE, // enkoder sıfır kayması [derece]
    P_ENC_DIR_PAN, P_ENC_DIR_TILT, P_ENC_DIR_CE, // +1 / -1
    P_MOT_DIR_PAN, P_MOT_DIR_TILT, P_MOT_DIR_CE, // +1 / -1
    P_PAN_MIN, P_PAN_MAX,                        // yazılım limitleri [derece]
    P_TILT_MIN, P_TILT_MAX,
    P_CE_MIN, P_CE_MAX,
    P_IMU_MOUNT_R, P_IMU_MOUNT_P, P_IMU_MOUNT_Y, // IMU montaj düzeltmesi [derece]
    P_YAW_DRIFT_TAU,                             // yaw kayma kestirimi zaman sabiti [s]
    P_YAW_DRIFT_EN,                              // 0/1
    P_LASER_ERR_THR,                             // otomatik lazer eşiği [derece]
    P_TELEM_HZ,                                  // telemetri hızı
    P_MOT_IRUN_MA,                               // sürücü çalışma akımı [mA rms], üçüne ortak
    P_MOT_IHOLD_PCT,                             // tutma akımı, çalışma akımının yüzdesi
    P_MOT_USTEPS,                                // mikroadım: 16 -> dps_pan 0.0225 ile uyumlu
    P_TMC_RSENSE,                                // stepstick sense direnci [ohm], BTT 0.11
    P_IMU_RATE_HZ,                               // BNO085 rapor hızı
    P_ENC_RATIO_PAN, P_ENC_RATIO_TILT, P_ENC_RATIO_CE, // enkoder motor milindeyse redüktör oranı (5 = motor 5 tur, çıkış 1 tur)
    P_CL_PAN, P_CL_TILT, P_CL_CE,                // kapalı çevrim 0/1: enkoderle adım kaçırma düzeltmesi
    P_SL_THR_PAN, P_SL_THR_TILT, P_SL_THR_CE,    // kaçırma eşiği [derece anten]: harekette fark bunu aşarsa konum enkoderden alınır
    P_ENC_DEADBAND_PAN, P_ENC_DEADBAND_TILT, P_ENC_DEADBAND_CE, // duruşta ölü bant [derece]
    P_AIM_SIGN_PAN, P_AIM_SIGN_TILT,             // nişan döngüsü işaretleri +1/-1 (lazer kaçarsa çevir)
    P_AIM_VMAX,                                  // otomatik modda hız tavanı [derece/s]
    P_AIM_IMAX,                                  // integral sınırı [derece]
    P_AIM_ERR_MAX,                               // bu hatanın üstünde otomatik mod bırakılır [derece]
    P_AIM_KFF,                                   // jiroskop ileri besleme kazancı 0..1 (taban hızının ne kadarı dengelenir)
    P_AIM_DB,                                    // nişan hatası ölü bandı [derece]: altında motor komutu yok, integral birikmez
    P_BL_PAN, P_BL_TILT, P_BL_CE,                // dişli boşluğu telafisi [derece anten]: yön değişince motor bu kadar fazladan sürülür
    P_AIM_KFF_PAN,                               // pan için ileri besleme kazancı (ayrı: pan boşluğu ile ff salınım yapıyor)
    // Paralaks düzeltmesi (29 Eyl, PARALAKS_DUZELTME.md): yön yerine hedef NOKTASI tutulur
    P_PLX_EN,                                    // 0 kapalı, 1 açık (tezgah testi geçmeden açılmaz)
    P_PLX_HT,                                    // tilt ekseninin platform dönme merkezinden yüksekliği [m]
    P_PLX_LX, P_PLX_LY, P_PLX_LZ,                // lazerin tilt ekseni noktasına göre konumu, başlık çerçevesi [m]: ileri, sol(+)/sağ(-), yukarı
    P_PLX_D,                                     // kilit anında lazerden hedefe mesafe [m]
    // IMU yön bağlama (29 Eyl): BNO085 yeniden başlayınca yaw o anki yöne göre sıfırdan kurulur, hedef kayar.
    // Açıkken tabanın yönü (IMU + eklem açıları) izlenir, yeniden başlamada yaw buna göre düzeltilir.
    P_YAW_ANCHOR,                                // 0 kapalı, 1 açık
    P_IMU_POLL_US,                               // IMU okumaları arası en az süre [µs]; 0 = boşluksuz (deney, 29 Eyl)
    P_IMU_I2C_SOFT,                              // 1: I2C pinleri yumuşak kenar (çınlamaya karşı), 0: kütüphane varsayılanı
    P_IMU_GYRO,                                  // 1: jiroskop raporu açık (ileri besleme), 0: kapalı (hat trafiği yarıya iner)
    P_ENC_READ,                                  // 1: enkoderler okunur, 0: hiç okunmaz (deney: SPI kenarları I2C'yi bozuyor mu)
    P_I2C_SYNC,                                  // 1: I2C aktarımı sürerken enkoder okunmaz (BNO085 reset çözümü), 0: eski davranış
    P_LORA_TELEM_HZ,                             // LoRa etkinken telemetri sıklığı [Hz] (hava hızı ~300 bayt/s, çerçeve ~100 bayt)
    PARAM_COUNT
};

void        params_defaults(void);
int         params_get(uint16_t id, float *out);
int         params_set(uint16_t id, float val);
float       params_val(uint16_t id);            // hızlı okuma, geçersiz id -> 0
const char *params_name(uint16_t id);
int         params_save(void);                  // 0 ok, 1 bellek yok, 2-3 geri okuma tutmadı
int         params_load(void);                  // açılışta: kayıt sağlamsa varsayılanların üstüne yükler
int         params_clear(void);                 // kaydı geçersiz kılar (sonraki açılış varsayılanlarla)
int         params_nv_state(void);              // açılış sonucu: >0 yüklenen adet, -1 kayıt yok, -2 bozuk, -3 bellek yok
int         params_nv_saves(void);              // bu oturumdaki başarılı kayıt sayısı

#ifdef __cplusplus
}
#endif
