#pragma once
// Telemetri paketi. Python tarafı: struct '<IBBH15fHHIII3f', 96 bayt.
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// flags bitleri
#define TF_LOCKED   (1u << 0)
#define TF_LASER    (1u << 1)
#define TF_IMU_OK   (1u << 2)
#define TF_ENC_OK   (1u << 3)
#define TF_MOT_EN   (1u << 4)   // en az bir sürücü açık
#define TF_EN_PAN   (1u << 5)   // eksen başına sürücü açık (29 Eyl: arayüz kutucukları kartın gerçek durumunu göstersin)
#define TF_EN_TILT  (1u << 6)
#define TF_EN_CE    (1u << 7)

// fault bitleri
#define FLT_ENC_PAN   (1u << 0)
#define FLT_ENC_TILT  (1u << 1)
#define FLT_ENC_CE    (1u << 2)
#define FLT_IMU       (1u << 3)
#define FLT_LIMIT     (1u << 4)
#define FLT_STEP_LOSS (1u << 5)
#define FLT_LOOP_OVER (1u << 6)

typedef struct __attribute__((packed)) {
    uint32_t t_ms;
    uint8_t  mode;
    uint8_t  flags;
    uint16_t fault;
    float    enc_deg[3];     // enkoderden okunan eksen açısı
    float    cmd_deg[3];     // komut açısı
    float    vel_dps[3];     // komut hızı
    float    rpy_deg[3];     // IMU roll, pitch, yaw
    float    err_deg[3];     // cmd - enc
    uint16_t loop_us;        // son kontrol döngüsü süresi
    uint16_t loop_max_us;    // en uzun döngü, telemetride sıfırlanır
    uint32_t imu_count;
    uint32_t rx_frames;
    uint32_t rx_errors;
    float    aim_err;        // nişan hatası toplam [derece] (kilitliyken)
    float    aim_side;       // yanal hata, +sol
    float    aim_up;         // dikey hata, +yukarı
} telem_t;

#ifdef __cplusplus
}
#endif
