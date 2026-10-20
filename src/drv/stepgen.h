#pragma once
// Yazılımsal adım üreteci. 20 kHz kesmede her eksen için birikim sayacı.
// Darbe bir tikte yükselir, sonraki tikte düşer: en yüksek 10 000 adım/s.
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define STEPGEN_AXES 3

void    stepgen_init(void);
void    stepgen_set_rate(int axis, float steps_per_s);   // işaretli, 0 = dur
void    stepgen_move(int axis, int32_t steps, float steps_per_s); // tam N adım at, sonra dur (işaret yönü verir)
int     stepgen_moving(int axis);                        // 1: sayılı hareket sürüyor
float   stepgen_get_rate(int axis);
void    stepgen_stop_all(void);
int32_t stepgen_position(int axis);                      // açılıştan beri net adım
void    stepgen_set_position(int axis, int32_t pos);
void    stepgen_tick(void);                              // kesme içinden

#ifdef __cplusplus
}
#endif
