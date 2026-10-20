#pragma once
// Eksen katmanı: her eksen için ivmeli (yamuk) hız profili, yazılım sınırları, A–B salınımı ve
// isteğe bağlı kapalı çevrim (enkoderle adım kaçırma düzeltmesi). 1 kHz kontrol kesmesinden çağrılır.
//
// Birimler: anten derecesi ve derece/s. Adım dönüşümü P_DPS_* (mikroadım başına derece) ve P_MOT_DIR_* ile.
// Pan: P_PAN_MIN<=0 ve P_PAN_MAX>=360 ise sınırsız, hedefe hep kısa yoldan gider. Tilt ve CE: sert sınır.
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AXIS_N 3

void     axis_init(void);
void     axis_set_target(int ax, float deg);            // konum modu; sınır dışı hedef sınıra kırpılır
void     axis_set_velocity(int ax, float dps);          // hız modu (jog); sınıra yaklaşınca kendisi yavaşlar, 0 = rampalı dur
void     axis_stop(int ax);                             // rampalı dur (salınımı da bitirir)
void     axis_halt(int ax);                             // anında dur (adım kaçabilir)
void     axis_halt_all(void);
void     axis_osc(int ax, float a_deg, float b_deg, uint16_t dwell_ms);  // A ile B arasında gidip gel
void     axis_osc_stop(int ax);
int      axis_osc_on(int ax);
void     axis_set_here(int ax, float deg);              // şu anki konumu deg kabul et (adım sayacı)
float    axis_pos(int ax);                              // komut konumu, derece, sürekli (sarmalanmamış)
float    axis_target(int ax);
float    axis_vel(int ax);                              // derece/s, işaretli
int      axis_busy(int ax);                             // 1: hareket sürüyor
int      axis_at_limit(int ax);                         // -1 alt sınırda, +1 üst sınırda, 0 değil
void     axis_enc_update(int ax, float enc_deg_cont, int valid);  // kesmeden: sürekli enkoder açısı
uint32_t axis_resyncs(int ax);                          // enkoderden konum düzeltme sayısı
void     axis_cl_info(int ax, int *valid, float *err);  // tanı: enkoder geçerli mi, sayaç-enkoder farkı (derece)
uint32_t axis_stalls(int ax);                           // takılma sayısı (5 düzeltme üst üste: hareket durduruldu)
uint32_t axis_stalls(int ax);                           // takılma sayısı (5 düzeltme üst üste: hareket durduruldu)
void     axis_tick(void);                               // 1 kHz

#ifdef __cplusplus
}
#endif
