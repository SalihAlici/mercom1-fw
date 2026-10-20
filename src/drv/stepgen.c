#include "stepgen.h"
#include "../hw.h"

#define Q 16
#define ONE (1u << Q)
#define MAX_RATE_Q ((int32_t)(ONE / 2))   // tik başına en fazla 0,5 adım

typedef struct {
    volatile int32_t rate_q;    // adım/tik, Q16 işaretli
    uint32_t acc;
    int8_t   dir;               // +1 / -1, DIR pininin son durumu
    uint8_t  high;              // darbe yüksekte mi
    volatile int32_t pos;
    float    rate_sps;
    volatile int32_t remain;    // sayılı harekette kalan adım, 0 = serbest
} axis_t;

static axis_t s_ax[STEPGEN_AXES];

void stepgen_init(void)
{
    for (int i = 0; i < STEPGEN_AXES; i++) {
        s_ax[i].rate_q = 0; s_ax[i].acc = 0; s_ax[i].dir = 1;
        s_ax[i].high = 0; s_ax[i].pos = 0; s_ax[i].rate_sps = 0.0f; s_ax[i].remain = 0;
        board_dir_write(i, 1);
    }
}

void stepgen_set_rate(int axis, float steps_per_s)
{
    if (axis < 0 || axis >= STEPGEN_AXES) return;
    s_ax[axis].remain = 0;
    float per_tick = steps_per_s / (float)STEP_TICK_HZ;
    int32_t q = (int32_t)(per_tick * (float)ONE);
    if (q >  MAX_RATE_Q) q =  MAX_RATE_Q;
    if (q < -MAX_RATE_Q) q = -MAX_RATE_Q;
    s_ax[axis].rate_sps = steps_per_s;
    s_ax[axis].rate_q = q;   // tek 32 bit yazma, kesmeye karşı güvenli
}

float stepgen_get_rate(int axis)
{
    return (axis >= 0 && axis < STEPGEN_AXES) ? s_ax[axis].rate_sps : 0.0f;
}

void stepgen_stop_all(void)
{
    for (int i = 0; i < STEPGEN_AXES; i++) { s_ax[i].rate_q = 0; s_ax[i].rate_sps = 0.0f; s_ax[i].remain = 0; }
}

void stepgen_move(int axis, int32_t steps, float steps_per_s)
{
    if (axis < 0 || axis >= STEPGEN_AXES || steps == 0) return;
    float sps = steps_per_s < 0 ? -steps_per_s : steps_per_s;
    stepgen_set_rate(axis, steps > 0 ? sps : -sps);
    s_ax[axis].remain = steps > 0 ? steps : -steps;
}

int stepgen_moving(int axis)
{
    return (axis >= 0 && axis < STEPGEN_AXES) ? (s_ax[axis].remain > 0) : 0;
}

int32_t stepgen_position(int axis)
{
    return (axis >= 0 && axis < STEPGEN_AXES) ? s_ax[axis].pos : 0;
}

void stepgen_set_position(int axis, int32_t pos)
{
    if (axis >= 0 && axis < STEPGEN_AXES) s_ax[axis].pos = pos;
}

void stepgen_tick(void)
{
    for (int i = 0; i < STEPGEN_AXES; i++) {
        axis_t *a = &s_ax[i];
        if (a->high) { board_step_write(i, 0); a->high = 0; }

        int32_t r = a->rate_q;
        if (r == 0) { continue; }

        int8_t want = (r > 0) ? 1 : -1;
        if (want != a->dir) {
            a->dir = want;
            board_dir_write(i, want > 0);
            a->acc = 0;          // yön değişiminde bir tik bekle, DIR otursun
            continue;
        }
        a->acc += (uint32_t)(r > 0 ? r : -r);
        if (a->acc >= ONE) {
            a->acc -= ONE;
            board_step_write(i, 1);
            a->high = 1;
            a->pos += want;
            if (a->remain > 0 && --a->remain == 0) { a->rate_q = 0; a->rate_sps = 0.0f; }
        }
    }
}
