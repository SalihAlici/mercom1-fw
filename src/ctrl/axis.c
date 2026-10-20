#include "axis.h"
#include <math.h>
#include "../hw.h"
#include "../app/params.h"
#include "../app/fsm.h"
#include "../app/telemetry.h"
#include "../drv/stepgen.h"

#define DT          (1.0f / (float)CTRL_HZ)
#define VMIN_SPS    12.0f      // kalkış ve varış hızı (adım/s): pan 0,27 °/s, tilt 0,54 °/s. 40 iken otomatik modda ince hız komutları çırpınıyordu
#define PAN_FAR     1.0e7f     // sınırsız pan için "sonsuz" hedef
#define SL_TICKS    3          // kaçırma: bu kadar tik üst üste eşik aşılırsa düzelt
#define SL_MAX      5          // bir harekette bu kadar düzeltme olursa arıza
#define REST_TICKS  200        // duruşta ölü bant dışı fark bu kadar tik sürerse düzelt

enum { M_IDLE = 0, M_MOVE, M_SETTLE };

typedef struct {
    int      mode;
    float    target;        // hedef, derece (sürekli)
    float    vcap;          // bu hareketin hız tavanı (jog'da istenen hız, aksi hâlde vmax)
    float    v;             // anlık hız, derece/s, işaretli
    float    off;           // hedef için boşluk kayması: motor hedefi = anten hedefi + off, off = ±boşluk/2 (yaklaşma yönü)
    float    ant;           // anten konumu tahmini: motor boşluk içinde gezerken sabit kalır (oyun modeli)
    int      osc_on;
    float    osc_a, osc_b;
    int      osc_toB;
    uint16_t osc_dwell;
    uint32_t osc_timer;     // varıştan beri tik
    float    enc;           // sürekli enkoder açısı
    float    encoff;        // adım sayacı - enkoder farkı (29 Eyl): sayaç axis_set_here ile (IMU'dan tilt, "Kabul et") yeniden
                            // tanımlanınca enkoder aynı kalır; kapalı çevrim enkoderi bu farkla karşılaştırır
    int      enc_valid;
    uint32_t resyncs, stalls;
    int      sl_ticks, sl_count, rest_ticks;
} ax_t;

static ax_t s_ax[AXIS_N];

// ---- parametre yardımcıları ----
static float dps(int ax)     { float d = params_val(P_DPS_PAN + ax); return d > 0.0f ? d : 0.045f; }
static float mdir(int ax)    { return params_val(P_MOT_DIR_PAN + ax) < 0.0f ? -1.0f : 1.0f; }
static float vmax(int ax)    { float v = params_val(P_VMAX_PAN + ax); return v > 0.1f ? v : 0.1f; }
static float amax(int ax)    { float a = params_val(P_AMAX_PAN + ax); return a > 0.1f ? a : 0.1f; }
static float lim_min(int ax) { return params_val(P_PAN_MIN + 2 * ax); }
static float lim_max(int ax) { return params_val(P_PAN_MAX + 2 * ax); }
static int   pan_free(void)  { return lim_min(0) <= 0.0f && lim_max(0) >= 360.0f; }
static int   closed(int ax)  { return params_val(P_CL_PAN + ax) != 0.0f; }

static float wrap360(float a) { while (a >= 360.0f) a -= 360.0f; while (a < 0.0f) a += 360.0f; return a; }
static float wrap180(float a) { while (a >  180.0f) a -= 360.0f; while (a < -180.0f) a += 360.0f; return a; }

// Motor tarafı konum: adım sayacından, motor yönü katılmış. steps * dps * mdir  (enkoderle karşılaştırılan bu)
static float motorPos(int ax) { return (float)stepgen_position(ax) * dps(ax) * mdir(ax); }
static float blash(int ax)    { float b = params_val(P_BL_PAN + ax); return b > 0.0f ? b : 0.0f; }
// Anten tarafı konum (oyun modeli): motor anteni yalnız boşluğun ucuna dayanınca sürükler.
static void updateAnt(int ax)
{
    ax_t *a = &s_ax[ax];
    float m = motorPos(ax), h = 0.5f * blash(ax);
    if (m - a->ant >  h) a->ant = m - h;
    if (m - a->ant < -h) a->ant = m + h;
}
float axis_pos(int ax) { updateAnt(ax); return s_ax[ax].ant; }
float axis_target(int ax) { return s_ax[ax].target; }
float axis_vel(int ax) { return s_ax[ax].v; }
int   axis_busy(int ax) { return s_ax[ax].mode != M_IDLE; }
int   axis_osc_on(int ax) { return s_ax[ax].osc_on; }
uint32_t axis_resyncs(int ax) { return s_ax[ax].resyncs; }
// Tanı: kapalı çevrimin gördüğü sayaç-enkoder farkı (derece) ve enkoder geçerli mi
void axis_cl_info(int ax, int *valid, float *err)
{
    if (ax < 0 || ax >= AXIS_N) { *valid = 0; *err = 0.0f; return; }
    *valid = s_ax[ax].enc_valid;
    *err = motorPos(ax) - (s_ax[ax].enc + s_ax[ax].encoff);
}
uint32_t axis_stalls(int ax) { return s_ax[ax].stalls; }

int axis_at_limit(int ax)
{
    if (ax == 0 && pan_free()) return 0;
    float p = axis_pos(ax), tol = 2.0f * dps(ax);
    if (p <= lim_min(ax) + tol) return -1;
    if (p >= lim_max(ax) - tol) return  1;
    return 0;
}

static float clampLim(int ax, float deg)
{
    if (ax == 0 && pan_free()) return deg;
    if (deg < lim_min(ax)) deg = lim_min(ax);
    if (deg > lim_max(ax)) deg = lim_max(ax);
    return deg;
}

void axis_init(void)
{
    for (int i = 0; i < AXIS_N; i++) {
        ax_t *a = &s_ax[i];
        a->mode = M_IDLE; a->target = 0.0f; a->vcap = 0.0f; a->v = 0.0f; a->off = 0.0f; a->ant = 0.0f;
        a->osc_on = 0; a->osc_toB = 0; a->osc_dwell = 0; a->osc_timer = 0;
        a->enc = 0.0f; a->encoff = 0.0f; a->enc_valid = 0; a->resyncs = 0; a->stalls = 0; a->sl_ticks = 0; a->sl_count = 0; a->rest_ticks = 0;
    }
}

static void startMove(int ax, float target, float vcap)
{
    ax_t *a = &s_ax[ax];
    a->target = target;
    // Boşluk telafisi: hareket yönüne göre kayma ±b/2. Yön değişince motor hedefi b kadar kayar,
    // motor boşluğu geçerken anten durur; anten konumu (motor - off) kayma değişiminde sıçramaz.
    // Yapışkan: boşluk yönü yalnız gerçek takip hızlarında (>= 1 °/s) ya da konum komutunda (vcap=0) değişir;
    // gürültü düzeyindeki küçük hız komutları boşlukta çırpınma yapmasın (25 Eyl)
    float b = blash(ax);
    if (b > 0.0f && (vcap <= 0.0f || vcap >= 1.0f)) {
        float d = target - axis_pos(ax);
        if (d > 0.001f)       a->off =  0.5f * b;
        else if (d < -0.001f) a->off = -0.5f * b;
    }
    a->vcap = (vcap > 0.0f && vcap < vmax(ax)) ? vcap : vmax(ax);
    a->sl_count = 0;
    if (a->mode == M_SETTLE) {           // sayılı son yaklaşımı iptal et, hız profiline dön
        stepgen_set_rate(ax, 0.0f);
    }
    a->mode = M_MOVE;
}

void axis_set_target(int ax, float deg)
{
    if (ax < 0 || ax >= AXIS_N) return;
    s_ax[ax].osc_on = 0;
    float t;
    if (ax == 0 && pan_free()) t = axis_pos(ax) + wrap180(deg - wrap360(axis_pos(ax)));   // kısa yoldan
    else                       t = clampLim(ax, deg);
    startMove(ax, t, 0.0f);
}

void axis_set_velocity(int ax, float dpsReq)
{
    if (ax < 0 || ax >= AXIS_N) return;
    s_ax[ax].osc_on = 0;
    if (dpsReq == 0.0f) { axis_stop(ax); return; }
    float p = axis_pos(ax), t;
    if (ax == 0 && pan_free()) t = p + (dpsReq > 0.0f ? PAN_FAR : -PAN_FAR);
    else                       t = dpsReq > 0.0f ? lim_max(ax) : lim_min(ax);
    startMove(ax, t, fabsf(dpsReq));
}

void axis_stop(int ax)
{
    if (ax < 0 || ax >= AXIS_N) return;
    ax_t *a = &s_ax[ax];
    a->osc_on = 0;
    if (a->mode == M_IDLE) return;
    // hedefi durma mesafesine çek; profil kendisi yavaşlatıp durur
    float s = a->v * a->v / (2.0f * amax(ax));
    float t = axis_pos(ax) + (a->v >= 0.0f ? s : -s);
    if (a->mode == M_SETTLE) stepgen_set_rate(ax, 0.0f);
    a->target = clampLim(ax, t);
    a->mode = M_MOVE;
}

void axis_halt(int ax)
{
    if (ax < 0 || ax >= AXIS_N) return;
    ax_t *a = &s_ax[ax];
    a->osc_on = 0; a->mode = M_IDLE; a->v = 0.0f;
    stepgen_set_rate(ax, 0.0f);
    a->target = axis_pos(ax);
}

void axis_halt_all(void) { for (int i = 0; i < AXIS_N; i++) axis_halt(i); }

void axis_osc(int ax, float a_deg, float b_deg, uint16_t dwell_ms)
{
    if (ax < 0 || ax >= AXIS_N) return;
    ax_t *a = &s_ax[ax];
    if (!(ax == 0 && pan_free())) { a_deg = clampLim(ax, a_deg); b_deg = clampLim(ax, b_deg); }
    else {                                        // pan: iki ucu şu anki konuma göre yakın tut
        float p = axis_pos(ax);
        a_deg = p + wrap180(a_deg - wrap360(p));
        b_deg = a_deg + wrap180(b_deg - wrap360(a_deg));
    }
    a->osc_a = a_deg; a->osc_b = b_deg; a->osc_dwell = dwell_ms; a->osc_toB = 1; a->osc_timer = 0;
    startMove(ax, b_deg, 0.0f);
    a->osc_on = 1;                               // startMove'dan sonra: hedef verme salınımı kapatmasın
}

void axis_osc_stop(int ax) { if (ax >= 0 && ax < AXIS_N) axis_stop(ax); }

void axis_set_here(int ax, float deg)
{
    if (ax < 0 || ax >= AXIS_N) return;
    axis_halt(ax);
    stepgen_set_position(ax, (int32_t)lroundf((deg + s_ax[ax].off) / dps(ax) * mdir(ax)));
    s_ax[ax].ant = deg;
    s_ax[ax].target = deg;
    s_ax[ax].encoff = motorPos(ax) - s_ax[ax].enc;     // sayaç yeniden tanımlandı: enkoder farkı da yenilenir
    s_ax[ax].sl_ticks = 0; s_ax[ax].rest_ticks = 0;
}

void axis_enc_update(int ax, float enc_deg_cont, int valid)
{
    if (ax < 0 || ax >= AXIS_N) return;
    ax_t *a = &s_ax[ax];
    // Enkoder yeniden güvenilir olduğunda (açılış, kablo oynaması) farkı o anki sayaçtan kur: kapalı çevrim yalnız
    // enkoder kesintisiz okunurken oluşan kaçırmaları düzeltsin, kesinti sırasındaki bilinmeyen sayım farkını değil
    if (valid && !a->enc_valid) { a->encoff = motorPos(ax) - enc_deg_cont; a->sl_ticks = 0; a->rest_ticks = 0; }
    a->enc = enc_deg_cont; a->enc_valid = valid;
}

// Enkoderle uyumsuzluk: adım sayacını enkodere (sayaç ölçeğinde) eşitle, hedef aynı kalır
static void resync(int ax)
{
    ax_t *a = &s_ax[ax];
    float before = motorPos(ax);
    stepgen_set_position(ax, (int32_t)lroundf((a->enc + a->encoff) / dps(ax) * mdir(ax)));
    a->ant += motorPos(ax) - before;
    a->resyncs++;
}

static void tickAxis(int ax)
{
    ax_t *a = &s_ax[ax];
    float d = dps(ax), acc = amax(ax);

    // ---- kapalı çevrim: adım kaçırma ----
    // 1 Eki: otomatikte CE kilit anındaki açısında tutulur (nişan döngüsü CE'yi sürmez). Hakem braketi elle çevirip
    // bırakınca CE o açıya geri döner; elle tutulurken takılsa da hedef korunur, 0,2 s'de bir yeniden dener.
    const int ceHold = (ax == AX_CE && fsm_mode() == MODE_AUTO && board_motor_enabled(AX_CE));
    if (closed(ax) && a->enc_valid) {
        float err = motorPos(ax) - (a->enc + a->encoff);   // enkoder motor milinde: motor tarafı konumla karşılaştır
        float thr = params_val(P_SL_THR_PAN + ax);
        if (thr <= 0.0f) thr = 1.0f;
        // Hareket hâlinde enkoder değeri I2C senkronu yüzünden 10–30 ms bayat olabilir: eşik hızla büyür (29 Eyl: pan 50 °/s'de
        // 60 s'de 12 sahte düzeltme, gerçek kaçırma 0,00°; 30 °/s'de 0). Gerçek takılma (onlarca derece) yine yakalanır.
        thr += fabsf(a->v) * 0.03f;
        if (a->mode != M_IDLE) {
            a->rest_ticks = 0;
            if (fabsf(err) > thr) {
                if (++a->sl_ticks >= SL_TICKS) {
                    a->sl_ticks = 0;
                    resync(ax);
                    if (++a->sl_count > SL_MAX) {          // motor takılı: bu hareketi durdur (arıza DEĞİL, komutlar açık kalır)
                        if (ceHold) {                       // CE elle tutuluyor: dur ama hedefi koru, bırakılınca geri döner
                            a->mode = M_IDLE; a->v = 0.0f; stepgen_set_rate(ax, 0.0f); a->rest_ticks = 0;
                            return;
                        }
                        axis_halt(ax);
                        a->stalls++;
                        return;
                    }
                    a->v = (a->v > 0.0f) ? VMIN_SPS * d : -VMIN_SPS * d;   // düşük hızdan yeniden kalk
                    if (a->mode == M_SETTLE) { stepgen_set_rate(ax, 0.0f); a->mode = M_MOVE; }
                }
            } else a->sl_ticks = 0;
        } else if (fsm_mode() != MODE_AUTO || ceHold) {            // otomatikte pan/tilt'i nişan döngüsü düzeltir; duruş düzeltmesi seğirtiyordu
            float band = params_val(P_ENC_DEADBAND_PAN + ax);
            if (band <= 0.0f) band = 0.15f;
            // CE tutma: takılmadan sonra sayaç enkoderle uyuşsa da CE hedefte değilse geri yürü
            float uzak = ceHold ? fabsf(a->target - axis_pos(ax)) : 0.0f;
            if (fabsf(err) > band || uzak > band) {
                if (++a->rest_ticks >= REST_TICKS) {       // duruşta kaymış: konumu enkoderden al, hedefe geri yürü
                    a->rest_ticks = 0;
                    float t = a->target;
                    if (fabsf(err) > band) resync(ax);
                    startMove(ax, t, 0.0f);
                }
            } else a->rest_ticks = 0;
        }
    }

    // ---- salınım: varınca bekle, öbür uca git ----
    if (a->osc_on && a->mode == M_IDLE) {
        if (++a->osc_timer >= (uint32_t)a->osc_dwell) {
            a->osc_timer = 0;
            a->osc_toB = !a->osc_toB;
            startMove(ax, a->osc_toB ? a->osc_b : a->osc_a, 0.0f);
            a->osc_on = 1;
        }
    }

    if (a->mode == M_SETTLE) {
        if (!stepgen_moving(ax)) { a->mode = M_IDLE; a->v = 0.0f; a->osc_timer = 0; }
        return;
    }
    if (a->mode != M_MOVE) return;

    // ---- yamuk profil ----
    updateAnt(ax);
    float pos = motorPos(ax);
    float dist = (a->target + a->off) - pos;            // motor hedefi = anten hedefi + boşluk kayması
    float want = (dist > 0.0f) ? 1.0f : (dist < 0.0f ? -1.0f : 0.0f);
    float v = a->v, av = fabsf(v);
    float vmin = VMIN_SPS * d;
    float vcap = a->vcap;

    if (want == 0.0f && av < vmin * 1.5f) { a->mode = M_IDLE; a->v = 0.0f; stepgen_set_rate(ax, 0.0f); a->osc_timer = 0; return; }

    float sgn = (v >= 0.0f) ? 1.0f : -1.0f;
    if (av > 0.0f && sgn != want) {
        // ters yöne gidiyor: önce dur
        av -= acc * DT; if (av < 0.0f) av = 0.0f;
        v = sgn * av;
    } else {
        sgn = want;
        float stopDist = av * av / (2.0f * acc);
        if (fabsf(dist) <= stopDist || av > vcap) { av -= acc * DT; if (av < vmin) av = vmin; }
        else                                      { av += acc * DT; if (av > vcap) av = vcap; }
        v = sgn * av;
    }
    a->v = v;

    // ---- son yaklaşım: kalan adım az ise sayılı hareketle tam yerinde dur ----
    float remSteps = fabsf(dist) / d;
    float stepsPerTick = av / d * DT;
    if (sgn == want && remSteps <= stepsPerTick * 2.0f + 1.0f) {
        int32_t n = (int32_t)lroundf(remSteps); if (n < 1) n = 1;
        float sps = av / d; if (sps < VMIN_SPS) sps = VMIN_SPS;
        stepgen_move(ax, (int32_t)(want * mdir(ax)) * n, sps);
        a->mode = M_SETTLE;
        return;
    }
    stepgen_set_rate(ax, mdir(ax) * v / d);
}

void axis_tick(void)
{
    for (int i = 0; i < AXIS_N; i++) tickAxis(i);
}
