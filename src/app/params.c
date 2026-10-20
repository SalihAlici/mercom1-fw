#include "params.h"
#include <math.h>
#include "../hw.h"

static float s_val[PARAM_COUNT];

typedef struct { const char *name; float def; } pdef_t;

// Varsayılanlar: NEMA17 1,8°, 8 mikroadım (MS pinleri boş, UART yok), pan/tilt 1:5, CE 1:1 (25 Eyl 2026 ölçümü)
static const pdef_t k_def[PARAM_COUNT] = {
    [P_KP_PAN]  = { "kp_pan",   8.0f }, [P_KP_TILT] = { "kp_tilt",  8.0f }, [P_KP_CE] = { "kp_ce", 8.0f },   // ilk deneme: yumuşak
    [P_KI_PAN]  = { "ki_pan",   2.0f }, [P_KI_TILT] = { "ki_tilt",  2.0f }, [P_KI_CE] = { "ki_ce",  2.0f },
    [P_VMAX_PAN]  = { "vmax_pan",  50.0f }, [P_VMAX_TILT] = { "vmax_tilt", 40.0f }, [P_VMAX_CE] = { "vmax_ce", 20.0f },   // 25 Eyl gece: hızlar artırıldı   // pan oranı 10: 30 °/s anten = 300 °/s motor; 60'ta geri dönüşte kilitlendi (25 Eyl)
    [P_AMAX_PAN]  = { "amax_pan", 150.0f }, [P_AMAX_TILT] = { "amax_tilt", 120.0f }, [P_AMAX_CE] = { "amax_ce", 100.0f },   // tilt 40/150 başlık 20° yukarıdayken kaçırdı (25 Eyl)
    [P_DPS_PAN]   = { "dps_pan",  0.0225f }, [P_DPS_TILT] = { "dps_tilt", 0.045f }, [P_DPS_CE] = { "dps_ce", 0.225f },   // 1.8/(8*oran); pan oranı 10 (25 Eyl IMU ölçümü)
    [P_ENC_OFF_PAN] = { "enc_off_pan", 0.0f }, [P_ENC_OFF_TILT] = { "enc_off_tilt", 0.0f }, [P_ENC_OFF_CE] = { "enc_off_ce", 0.0f },
    [P_ENC_DIR_PAN] = { "enc_dir_pan", 1.0f }, [P_ENC_DIR_TILT] = { "enc_dir_tilt", 1.0f }, [P_ENC_DIR_CE] = { "enc_dir_ce", -1.0f },  // CE ters (25 Eyl)
    [P_MOT_DIR_PAN] = { "mot_dir_pan", 1.0f }, [P_MOT_DIR_TILT] = { "mot_dir_tilt", 1.0f }, [P_MOT_DIR_CE] = { "mot_dir_ce", 1.0f },
    // Çalışma aralıkları (21 Eyl 2026): pan 0–360 (tek tam tur, hedef 180 civarında olacak şekilde monte edilir),
    // tilt 0–90, CE 0–4 (toplam 4°; çalışma ortası 2°, pan CE'yi bu ortaya geri çeker)
    [P_PAN_MIN]  = { "pan_min",    0.0f }, [P_PAN_MAX]  = { "pan_max", 360.0f },
    [P_TILT_MIN] = { "tilt_min", -90.0f }, [P_TILT_MAX] = { "tilt_max", 90.0f },   // 30 Eyl: -90..+90 (tezgahta taban çok yatınca tilt 0° sınırına dayanıyordu)
    [P_CE_MIN]   = { "ce_min",     0.0f }, [P_CE_MAX]   = { "ce_max",    4.0f },
    // 10 Eylül 2026 kalibrasyonu: BNO085 başlıkta yatay, çip yukarı, Y oku boresight yönünde (tools/imu_mount.json)
    // 29 Eylül 2026: eğim (p) lazerin kendisiyle ölçüldü. Taban düzken lazer ucu zeminden 28 cm, duvardaki nokta 132 cm,
    // yatay mesafe 200 cm -> ışının gerçek yükselişi 27,47°; eski −4,746 ile yazılım 37,71° sanıyordu (10° hata, taban
    // yatınca leke yay çiziyordu). Yeniden ölçmek için: tools/montaj_olc.py
    [P_IMU_MOUNT_R] = { "imu_mount_r", -2.417f }, [P_IMU_MOUNT_P] = { "imu_mount_p", 5.512f }, [P_IMU_MOUNT_Y] = { "imu_mount_y", 93.441f },
    [P_YAW_DRIFT_TAU] = { "yaw_drift_tau", 60.0f },
    [P_YAW_DRIFT_EN]  = { "yaw_drift_en",   0.0f },
    [P_LASER_ERR_THR] = { "laser_err_thr",  0.5f },
    [P_TELEM_HZ]      = { "telem_hz",      50.0f },
    [P_MOT_IRUN_MA]   = { "mot_irun_ma", 1000.0f },
    [P_MOT_IHOLD_PCT] = { "mot_ihold_pct", 50.0f },
    [P_MOT_USTEPS]    = { "mot_usteps",     8.0f },
    [P_TMC_RSENSE]    = { "tmc_rsense",     0.11f },
    [P_IMU_RATE_HZ]   = { "imu_rate_hz",  100.0f },
    // Mıknatıs motorun arka milinde: pan ve tilt 1:5 planet, CE doğrudan
    // 25 Eyl 2026 IMU ölçümü: pan 10,05 (redüktör 5 × ek kademe 2), tilt 4,93 ≈ 5, CE 1
    [P_ENC_RATIO_PAN] = { "enc_ratio_pan", 10.0f }, [P_ENC_RATIO_TILT] = { "enc_ratio_tilt", 5.0f }, [P_ENC_RATIO_CE] = { "enc_ratio_ce", 1.0f },
    // Kapalı çevrim: harekette fark eşiği aşarsa konum enkoderden alınır, 5 düzeltmede arıza; duruşta ölü bant dışı fark düzeltilir
    // 29 Eyl: pan/tilt AÇIK. Enkoderler artık hatasız (I2C senkronu), sayaç-enkoder farkı eksen başına tutuluyor (encoff).
    // Sebep: pan adım kaçırıp 72° geride kaldı, tilt sayacı sahte 90° sınırına dayandı. CE kapalı: mıknatısı çok yakın (AGC 0)
    [P_CL_PAN] = { "cl_pan", 1.0f }, [P_CL_TILT] = { "cl_tilt", 1.0f }, [P_CL_CE] = { "cl_ce", 0.0f },
    // Kaçırma eşiği: 3 tam adımdan biraz fazla (pan/tilt 5,4° motor / 5 = 1,08°; CE redüktörsüz 5,4°)
    [P_SL_THR_PAN] = { "sl_thr_pan", 1.5f }, [P_SL_THR_TILT] = { "sl_thr_tilt", 1.5f }, [P_SL_THR_CE] = { "sl_thr_ce", 6.0f },
    [P_ENC_DEADBAND_PAN] = { "enc_db_pan", 0.5f }, [P_ENC_DEADBAND_TILT] = { "enc_db_tilt", 0.6f }, [P_ENC_DEADBAND_CE] = { "enc_db_ce", 2.5f },   // > 1 tam adım (yük altında duruş sapması: pan 0,18°, tilt 0,36°, CE 1,8°); 0,15 iken sürekli seğirme
    // Nişan döngüsü: Kp 1/s (kp_pan, kp_tilt), Ki (ki_pan, ki_tilt) yukarıda. İlk denemede işaretler doğrulanır.
    [P_AIM_SIGN_PAN] = { "aim_sign_pan", 1.0f }, [P_AIM_SIGN_TILT] = { "aim_sign_tilt", 1.0f },
    [P_AIM_VMAX] = { "aim_vmax", 45.0f },   // 8 s kurali: 180 derece elle cevirme ~6 s; 60'ta pan kilitlenmisti
    // 29 Eyl: asagidaki iki satir yanlislikla ustteki yorumun icinde kalmisti (varsayilan 0 okunuyordu; pointing.c 0'da 2 kullaniyor)
    [P_AIM_IMAX] = { "aim_imax", 2.0f }, [P_AIM_ERR_MAX] = { "aim_err_max", 120.0f },
    // Benzetim (25 Eyl): kff 0,7 → şartname hareketinde 0,2°, 0,9 → 0,09°; gerçek gecikmeye göre ayarlanır
    [P_AIM_KFF] = { "aim_kff", 0.7f },   // tilt için (boşluğu ~0,2°, sorun yok)
    // Ölü bant: hedefte dururken IMU gürültüsü + adım kuantizasyonu motoru titretiyordu (25 Eyl). Ø10 payı 0,72°
    // 0,1 iken pan boşluğu (0,7°) yüzünden bozma sonrası sürekli titriyordu (25 Eyl gece). 30 Eyl: 0,15 denendi (duruşta
    // titreme yoktu), 0,3'e geri alındı
    [P_AIM_DB] = { "aim_db", 0.3f },
    // Boşluk (25 Eyl IMU ölçümü: pan 0,7°, tilt 0,2°); ölçülenin ~%80'i, fazlası ters yönde sıçratır
    [P_BL_PAN] = { "bl_pan", 0.0f }, [P_BL_TILT] = { "bl_tilt", 0.0f }, [P_BL_CE] = { "bl_ce", 0.0f },   // KAPALI: yapışkan telafi de duruşta ±0,6° salınım yaptı (25 Eyl gece); pan boşluğu mekanik çözülecek
    // 25 Eyl gece taraması: pan'da ff 0,7 → duruşta %98 hareket, 115 yön değişimi/15 s; ff 0 → %8, 0. Pan boşluğu (0,7°) ff ile salınıyor
    // 1 Eki: yarışma videosunda yaw'da ±12–15 cm gecikme (ff 0). Masada 0,5 ve 0,7 titremesiz; 0,2 seçildi
    [P_AIM_KFF_PAN] = { "aim_kff_pan", 0.2f },
    // Paralaks: lazer dönme merkezinden 21,6 cm yukarıda (tilt ekseni 16,6 + 5), pan ekseninin 13 cm sağında, 10 cm ileride; hedef 4 m ileri 2 m yukarı
    // 30 Eyl (yarışma günü): paralaks varsayılan AÇIK. Tezgah testi (29 Eyl, taban elle 15–18° yatırıldı, duvar 2 m):
    // kapalıyken 14 cm, açıkken 2 cm kayma. plx_ht platform ölçüsü bilinmiyor, 0,166 kullanılıyor
    [P_PLX_EN] = { "plx_en", 1.0f }, [P_PLX_HT] = { "plx_ht", 0.166f },
    [P_PLX_LX] = { "plx_lx", 0.10f }, [P_PLX_LY] = { "plx_ly", -0.13f }, [P_PLX_LZ] = { "plx_lz", 0.05f },
    [P_PLX_D] = { "plx_d", 4.0f },   // 30 Eyl: şartname "lazer çıkışından 4 m eğik mesafe, 30° elevasyon" (teyit edildi); eski 4,47 "4 m ileri 2 m yukarı" okumasıydı
    [P_YAW_ANCHOR] = { "yaw_anchor", 1.0f },
    // 29 Eyl: okuma aralığı tek başına resetleri azaltmadı (2 ms: 13, boşluksuz: 6 ve 8). Kök sebep enkoder SPI kenarlarının
    // I2C'ye sızması (enc_read 0 -> 0 reset). i2c_sync 1 iken I2C sürerken enkoder okunmaz; okumalar arası 1 ms boşlukta
    // (bitişten başlangıca) enkoderlere sıra gelir
    [P_IMU_POLL_US] = { "imu_poll_us", 1000.0f },
    // 29 Eyl deneyleri (bno_ab_test.py, sürücüler açık): yumuşak kenar 21 / varsayılan 19 reset (fayda yok -> 0 = kütüphane
    // varsayılanı). Jiroskop kapalı: yön raporu 80 -> 97/s, bayat örnek %3,4 -> %1,2, reset 5,9 -> 4,3/dk (sebep değil;
    // ileri besleme için açık kalıyor). Asıl şüpheli: BNO085'in saat uzatmada SDA kurulum süresi ihlali -> VIN-SDA 2,2–3 kΩ
    [P_IMU_I2C_SOFT] = { "imu_i2c_soft", 0.0f },
    [P_IMU_GYRO] = { "imu_gyro", 1.0f },
    [P_ENC_READ] = { "enc_read", 1.0f },
    [P_I2C_SYNC] = { "i2c_sync", 1.0f },
    [P_LORA_TELEM_HZ] = { "lora_telem_hz", 1.0f },   // saniyede 1 (30 Eyl); hava hızı 9,6 kbps, çerçeve ~100 bayt
};

void params_defaults(void)
{
    for (int i = 0; i < PARAM_COUNT; i++) s_val[i] = k_def[i].def;
}

int params_get(uint16_t id, float *out)
{
    if (id >= PARAM_COUNT) return 0;
    *out = s_val[id];
    return 1;
}

int params_set(uint16_t id, float val)
{
    if (id >= PARAM_COUNT) return 0;
    s_val[id] = val;
    return 1;
}

float params_val(uint16_t id)
{
    return (id < PARAM_COUNT) ? s_val[id] : 0.0f;
}

const char *params_name(uint16_t id)
{
    return (id < PARAM_COUNT) ? k_def[id].name : "?";
}

// ---- kalıcı kayıt (30 Eyl 2026; şartname: "tüm parametreler güç kapanıp açılınca son hâliyle kayıtlı kalmalı") ----
// Düzen: başlık {sihir "MRC1", adet, sürüm} + adet x float + FNV-1a kontrol toplamı. Açılışta kayıt sağlamsa varsayılanların
// üstüne yüklenir, değilse varsayılanlar kalır. Adet farklıysa (yazılıma parametre eklendi) ortak kısım yüklenir, yeniler
// varsayılanda kalır. telem_hz oturum ayarı: kaydedilir ama yüklenmez. Kayıt yeni yazılım yüklense de silinmez.
#define NV_MAGIC 0x3143524Du
#define NV_MAX   256
typedef struct { uint32_t magic; uint16_t count; uint16_t ver; } nvhdr_t;
static float s_nvBuf[NV_MAX];
static int   s_nvState = -1;                     // açılışta: >0 yüklenen adet, -1 kayıt yok, -2 bozuk, -3 bellek yok
static int   s_nvSaves;

static uint32_t fnv(const void *p, uint32_t n, uint32_t h)
{
    const uint8_t *b = (const uint8_t *)p;
    while (n--) { h ^= *b++; h *= 16777619u; }
    return h;
}

int params_save(void)
{
    nvhdr_t h = { NV_MAGIC, PARAM_COUNT, 1 };
    uint32_t c = fnv(s_val, sizeof s_val, fnv(&h, sizeof h, 2166136261u)), rc = 0;
    if (board_nv_write(0, &h, sizeof h) || board_nv_write(sizeof h, s_val, sizeof s_val) ||
        board_nv_write(sizeof h + sizeof s_val, &c, sizeof c)) return 1;
    // geri oku: yazılan kayıt açılışta kabul edilecek mi
    nvhdr_t r;
    if (board_nv_read(0, &r, sizeof r) || board_nv_read(sizeof r, s_nvBuf, sizeof s_val) || board_nv_read(sizeof r + sizeof s_val, &rc, sizeof rc)) return 2;
    if (r.magic != NV_MAGIC || r.count != PARAM_COUNT || fnv(s_nvBuf, sizeof s_val, fnv(&r, sizeof r, 2166136261u)) != rc || rc != c) return 3;
    s_nvSaves++;
    return 0;
}

int params_load(void)
{
    nvhdr_t h; uint32_t c;
    if (board_nv_read(0, &h, sizeof h)) { s_nvState = -3; return 1; }
    if (h.magic != NV_MAGIC || h.count == 0 || h.count > NV_MAX) { s_nvState = -1; return 1; }
    uint16_t n = h.count;
    if (board_nv_read(sizeof h, s_nvBuf, n * 4u) || board_nv_read(sizeof h + n * 4u, &c, sizeof c)) { s_nvState = -3; return 1; }
    if (fnv(s_nvBuf, n * 4u, fnv(&h, sizeof h, 2166136261u)) != c) { s_nvState = -2; return 1; }
    if (n > PARAM_COUNT) n = PARAM_COUNT;
    for (uint16_t i = 0; i < n; i++)
        if (i != P_TELEM_HZ && isfinite(s_nvBuf[i])) s_val[i] = s_nvBuf[i];
    s_nvState = n;
    return 0;
}

int params_clear(void)
{
    uint32_t z = 0xFFFFFFFFu;                    // sihir bozulur: sonraki açılışta yazılım varsayılanları
    return board_nv_write(0, &z, sizeof z) ? 1 : 0;
}

int params_nv_state(void) { return s_nvState; }
int params_nv_saves(void) { return s_nvSaves; }
