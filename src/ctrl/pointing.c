#include "pointing.h"
#include <math.h>
#include "../app/params.h"

#define D2R (3.14159265f / 180.0f)
#define R2D (180.0f / 3.14159265f)

static float s_RIH[3][3];        // başlık -> IMU
static float s_mountKey[3] = { 1e9f, 1e9f, 1e9f };
static float s_q[4] = { 1, 0, 0, 0 };
static int   s_haveQ = 0;
static float s_dN[3] = { 1, 0, 0 };      // şu anki bakış, dünya
static float s_RNH[3][3];                // başlık -> dünya (satır: dünya ekseni)
static float s_dN0[3] = { 1, 0, 0 };     // kilit
static int   s_locked = 0;
static float s_eH[3] = { 0, 0, 0 };      // hata dönüş vektörü, başlık çerçevesi (rad)
static float s_intZ = 0.0f, s_intY = 0.0f;
static float s_wH[3] = { 0, 0, 0 };      // başlık çerçevesinde ölçülen açısal hız, derece/s
static float s_ownSidePrev = 0.0f, s_ownUpPrev = 0.0f;   // bir örnek önceki kendi hızımız (IMU gecikmesiyle eşlensin)
static int   s_actZ = 0, s_actY = 0;                   // ölü bant histerezisi: düzeltme etkin mi
// Paralaks: kilitte hedef NOKTASI T_W saklanır; her örnekte lazerin konumu L_W hesaplanır, istenen yön normalize(T_W - L_W).
// Pan ekseni dönme merkezinde (P = 0) olduğu için pan açısı denklemden düşer: L_W = R_WH * (H_T*(sin t, 0, cos t) + l_H), t = tilt açısı.
static float s_TW[3] = { 0, 0, 0 };
static int   s_plxHave = 0;
static float s_tiltDeg = 0.0f;                         // tilt eklem açısı (adım sayacı), derece
static float s_dRef[3] = { 1, 0, 0 };                  // şu an kullanılan referans yön (teşhis)

// Yön bağlama (29 Eyl): IMU yeniden başlayınca GRV yaw'ı o anki sensör yönüne göre sıfırdan kurulur; başlık o sırada
// hedeften X° uzaktaysa kilit X° kayıyordu (bozma testi: IMU hatası 0,4° gösterirken enkoderden gerçek hata 4–9°).
// Dünya çerçevesi Rz(s_yawCorr) ile düzeltilir; düzeltme, tabanın yaw'ı kesintiden önceki değerinde kalacak şekilde seçilir.
#define ANCHOR_SETTLE 6                                 // yeniden başlamadan sonra atılan örnek (ilk rapor eski olabiliyor)
static float    s_yawCorr = 0.0f;                        // rad, dünya çerçevesine uygulanan yaw düzeltmesi
static float    s_baseYaw = 0.0f;                        // rad, tabanın süzülmüş yaw'ı (düzeltilmiş çerçevede)
static int      s_baseHave = 0;
static int      s_anchorPend = 0;                        // >0: bağlama bekliyor, kalan örnek
static float    s_panJ = 0.0f, s_tiltJ = 0.0f;           // eklem açıları, derece
static uint32_t s_anchorCount = 0;
static float    s_anchorLast = 0.0f;                     // derece

static float wrapPi(float a)
{
    while (a >  3.14159265f) a -= 6.2831853f;
    while (a < -3.14159265f) a += 6.2831853f;
    return a;
}

// Tabanın x ekseninin dünya yaw'ı: x_B = R_NH * R_BH^T * e_x, R_BH = Rz(sp*pan) Ry(-tilt)
static float baseYaw(float R[3][3])
{
    float sp = (params_val(P_AIM_SIGN_PAN) < 0.0f) ? -1.0f : 1.0f;
    float a = sp * s_panJ * D2R, t = s_tiltJ * D2R;
    float v[3] = { cosf(t) * cosf(a), -sinf(a), -sinf(t) * cosf(a) };
    float x = R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2];
    float y = R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2];
    return atan2f(y, x);
}

// Dünya çerçevesini z etrafında d kadar döndür: R <- Rz(d) R
static void yawRotate(float R[3][3], float d)
{
    float c = cosf(d), s = sinf(d);
    for (int j = 0; j < 3; j++) {
        float r0 = R[0][j], r1 = R[1][j];
        R[0][j] = c * r0 - s * r1;
        R[1][j] = s * r0 + c * r1;
    }
}

static void laserPosW(float L[3])
{
    float t = s_tiltDeg * D2R, ht = params_val(P_PLX_HT);
    float v[3] = { ht * sinf(t) + params_val(P_PLX_LX), params_val(P_PLX_LY), ht * cosf(t) + params_val(P_PLX_LZ) };
    for (int i = 0; i < 3; i++) L[i] = s_RNH[i][0] * v[0] + s_RNH[i][1] * v[1] + s_RNH[i][2] * v[2];
}

static void mountMatrix(void)
{
    float r = params_val(P_IMU_MOUNT_R), p = params_val(P_IMU_MOUNT_P), y = params_val(P_IMU_MOUNT_Y);
    if (r == s_mountKey[0] && p == s_mountKey[1] && y == s_mountKey[2]) return;
    s_mountKey[0] = r; s_mountKey[1] = p; s_mountKey[2] = y;
    r *= D2R; p *= D2R; y *= D2R;
    float cr = cosf(r), sr = sinf(r), cp = cosf(p), sp = sinf(p), cy = cosf(y), sy = sinf(y);
    // R = Rz(y) Ry(p) Rx(r)
    s_RIH[0][0] = cy * cp; s_RIH[0][1] = cy * sp * sr - sy * cr; s_RIH[0][2] = cy * sp * cr + sy * sr;
    s_RIH[1][0] = sy * cp; s_RIH[1][1] = sy * sp * sr + cy * cr; s_RIH[1][2] = sy * sp * cr - cy * sr;
    s_RIH[2][0] = -sp;     s_RIH[2][1] = cp * sr;                s_RIH[2][2] = cp * cr;
}

static void quatToR(const float q[4], float R[3][3])
{
    float w = q[0], x = q[1], y = q[2], z = q[3];
    R[0][0] = 1 - 2 * (y * y + z * z); R[0][1] = 2 * (x * y - w * z);     R[0][2] = 2 * (x * z + w * y);
    R[1][0] = 2 * (x * y + w * z);     R[1][1] = 1 - 2 * (x * x + z * z); R[1][2] = 2 * (y * z - w * x);
    R[2][0] = 2 * (x * z - w * y);     R[2][1] = 2 * (y * z + w * x);     R[2][2] = 1 - 2 * (x * x + y * y);
}

void pointing_init(void)
{
    s_haveQ = 0; s_locked = 0; s_intZ = s_intY = 0.0f;
    s_mountKey[0] = 1e9f; mountMatrix();
}

void pointing_set_quat(const float q[4])
{
    mountMatrix();
    float RNI[3][3]; quatToR(q, RNI);
    if (s_yawCorr != 0.0f) yawRotate(RNI, s_yawCorr);
    // R_NH = R_NI * R_IH
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            s_RNH[i][j] = RNI[i][0] * s_RIH[0][j] + RNI[i][1] * s_RIH[1][j] + RNI[i][2] * s_RIH[2][j];
    // Yön bağlama: tabanın yaw'ını izle; yeniden başlamadan sonra onu eski değerine döndür
    {
        float by = baseYaw(s_RNH);
        if (s_anchorPend > 0) {
            if (--s_anchorPend == 0) {
                if (s_baseHave && params_val(P_YAW_ANCHOR) > 0.5f) {
                    float d = wrapPi(s_baseYaw - by);
                    s_yawCorr = wrapPi(s_yawCorr + d);
                    yawRotate(s_RNH, d);
                    s_anchorLast = d * R2D; s_anchorCount++;
                    by = s_baseYaw;
                }
                s_baseYaw = by; s_baseHave = 1;
            }
        } else if (!s_baseHave) {
            s_baseYaw = by; s_baseHave = 1;
        } else {
            s_baseYaw = wrapPi(s_baseYaw + 0.005f * wrapPi(by - s_baseYaw));   // 100 Hz'de ~2 s süzme
        }
    }
    for (int i = 0; i < 3; i++) { s_dN[i] = s_RNH[i][0]; s_q[i] = q[i]; }   // bakış = başlık x ekseni
    s_q[3] = q[3];
    s_haveQ = 1;
    if (s_locked) {
        // referans yön: paralaks açıksa hedef noktasına giden yön, değilse kilitteki sabit yön
        float dR[3] = { s_dN0[0], s_dN0[1], s_dN0[2] };
        if (s_plxHave && params_val(P_PLX_EN) > 0.5f) {
            float L[3]; laserPosW(L);
            float v[3] = { s_TW[0] - L[0], s_TW[1] - L[1], s_TW[2] - L[2] };
            float m = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
            if (m > 0.5f) { dR[0] = v[0] / m; dR[1] = v[1] / m; dR[2] = v[2] / m; }   // hedef yarım metreden yakınsa düzeltme yapma
        }
        s_dRef[0] = dR[0]; s_dRef[1] = dR[1]; s_dRef[2] = dR[2];
        // hata dönüşü (dünya): e_N = d_N x d_ref ; başlığa çevir: e_H = R_NH^T e_N
        float eN[3] = { s_dN[1] * dR[2] - s_dN[2] * dR[1],
                        s_dN[2] * dR[0] - s_dN[0] * dR[2],
                        s_dN[0] * dR[1] - s_dN[1] * dR[0] };
        // küçük açı: |e| = sin(θ); büyük açıda θ = asin(|e|) ile ölçekle (d·d0 < 0 ise π - asin)
        float n = sqrtf(eN[0] * eN[0] + eN[1] * eN[1] + eN[2] * eN[2]);
        float d = s_dN[0] * dR[0] + s_dN[1] * dR[1] + s_dN[2] * dR[2];
        float th = atan2f(n, d);
        float k = (n > 1e-6f) ? th / n : 1.0f;
        for (int i = 0; i < 3; i++) s_eH[i] = k * (s_RNH[0][i] * eN[0] + s_RNH[1][i] * eN[1] + s_RNH[2][i] * eN[2]);
    } else {
        s_eH[0] = s_eH[1] = s_eH[2] = 0.0f;
    }
}

int pointing_lock(void)
{
    if (!s_haveQ) return 0;
    for (int i = 0; i < 3; i++) s_dN0[i] = s_dN[i];
    {   // hedef noktası: T_W = L_W + D * d_W (platform düz ve hareketsizken kilit alınmalı)
        float L[3]; laserPosW(L);
        float D = params_val(P_PLX_D); if (D < 0.5f) D = 0.5f;
        for (int i = 0; i < 3; i++) s_TW[i] = L[i] + D * s_dN0[i];
        s_plxHave = 1;
    }
    s_locked = 1; s_intZ = s_intY = 0.0f;
    s_eH[0] = s_eH[1] = s_eH[2] = 0.0f;
    return 1;
}

void pointing_unlock(void) { s_locked = 0; s_intZ = s_intY = 0.0f; s_eH[0] = s_eH[1] = s_eH[2] = 0.0f; }

void pointing_set_gyro(const float w_s[3])
{
    mountMatrix();
    // ω_H = R_IH^T ω_I  (R_IH: başlık -> IMU)
    for (int i = 0; i < 3; i++)
        s_wH[i] = (s_RIH[0][i] * w_s[0] + s_RIH[1][i] * w_s[1] + s_RIH[2][i] * w_s[2]) * R2D;
}
int  pointing_locked(void) { return s_locked; }
void pointing_set_tilt(float deg) { s_tiltDeg = deg; }
void pointing_set_joints(float pan_deg, float tilt_deg) { s_panJ = pan_deg; s_tiltJ = tilt_deg; }
void pointing_imu_restarted(void) { s_anchorPend = ANCHOR_SETTLE; s_intZ = s_intY = 0.0f; }
uint32_t pointing_anchor_count(void) { return s_anchorCount; }
float pointing_anchor_last(void) { return s_anchorLast; }
float pointing_plx_shift(void)       // paralaks referansının kilit yönünden sapması, derece (teşhis)
{
    float d = s_dRef[0] * s_dN0[0] + s_dRef[1] * s_dN0[1] + s_dRef[2] * s_dN0[2];
    if (d > 1.0f) d = 1.0f; if (d < -1.0f) d = -1.0f;
    return acosf(d) * R2D;
}
void pointing_reset_integrators(void) { s_intZ = s_intY = 0.0f; s_actZ = s_actY = 0; }
int  pointing_have_imu(void) { return s_haveQ; }

// Pan hesabı (v_pan = u/cos el) başlığın "yukarı" ekseninin pan ekseniyle aynı yöne baktığını varsayar.
// Başlık zenite 10°'den yakınsa ya da 70°'den fazla devrilmişse (25 Eyl: roll 131° pozunda döngü ters yöne sürdü) geçersiz.
int pointing_pose_ok(void)
{
    if (!s_haveQ) return 0;
    float headUp = s_RNH[2][2];                 // z_H · z_N
    float el = pointing_elevation();
    return (headUp > 0.34f && el < 80.0f && el > -80.0f) ? 1 : 0;   // 30 Eyl: alt sınır -30 -> -80 (tilt -90..90 ile uyumlu)
}

float pointing_elevation(void)
{
    float z = s_dN[2]; if (z > 1.0f) z = 1.0f; if (z < -1.0f) z = -1.0f;
    return asinf(z) * R2D;
}

float pointing_azimuth(void)
{
    float a = atan2f(s_dN[1], s_dN[0]) * R2D;
    while (a < 0.0f) a += 360.0f; while (a >= 360.0f) a -= 360.0f;
    return a;
}

void pointing_error(float *total, float *side, float *up)
{
    // e_H.z: +sol yönde dönüş gerekiyor; e_H.y: +y dönüşü bakışı aşağı indirir -> yukarı hata = -e_y
    *side = s_eH[2] * R2D;
    *up   = -s_eH[1] * R2D;
    *total = sqrtf(s_eH[1] * s_eH[1] + s_eH[2] * s_eH[2]) * R2D;
}

int pointing_step(float dt, float v_pan_act, float v_tilt_act, float *v_pan, float *v_tilt)
{
    if (!s_locked || !s_haveQ) { *v_pan = *v_tilt = 0.0f; return 0; }
    if (s_anchorPend > 0) { *v_pan = *v_tilt = 0.0f; s_intZ = s_intY = 0.0f; return 1; }   // yön bağlanana kadar dur (~60 ms)
    float kp_pan = params_val(P_KP_PAN), ki_pan = params_val(P_KI_PAN);
    float kp_tilt = params_val(P_KP_TILT), ki_tilt = params_val(P_KI_TILT);
    float vmax = params_val(P_AIM_VMAX); if (vmax <= 0.0f) vmax = 30.0f;
    float imax = params_val(P_AIM_IMAX); if (imax <= 0.0f) imax = 2.0f;     // integral sınırı, derece
    float ez = s_eH[2] * R2D, ey = -s_eH[1] * R2D;                          // derece: yanal (+sol), dikey (+yukarı)
    // Ölü bant + histerezis: hata db'yi aşınca tam kuvvetle düzeltilir, db/2'nin altına inince bırakılır.
    // Duruşta IMU gürültüsü ve adım kuantizasyonu motoru titretmesin (25 Eyl); hata çoğunlukla db/2 altında kalır.
    // Histerezisli sürüm (aşınca tam kuvvet) yön değişimlerinde boşlukta çırpındı (%100 hareket); bu sürüm
    // yalnız bandı aşan kısmı düzeltir: duruşta motor kıpırdamaz (ölçüldü: %0), hata en fazla ~db kalır.
    float db = params_val(P_AIM_DB); if (db < 0.0f) db = 0.0f;
    if (ez > -db && ez < db) ez = 0.0f; else ez -= (ez > 0.0f) ? db : -db;
    if (ey > -db && ey < db) ey = 0.0f; else ey -= (ey > 0.0f) ? db : -db;
    s_actZ = (ez != 0.0f); s_actY = (ey != 0.0f);

    s_intZ += ez * dt; s_intY += ey * dt;
    if (s_intZ >  imax) s_intZ =  imax; if (s_intZ < -imax) s_intZ = -imax;
    if (s_intY >  imax) s_intY =  imax; if (s_intY < -imax) s_intY = -imax;

    float el = pointing_elevation() * D2R;
    float c = cosf(el); if (c < 0.2f) c = 0.2f;                             // 78° üstünde pan kazancını sınırla
    float sp = (params_val(P_AIM_SIGN_PAN)  < 0.0f) ? -1.0f : 1.0f;
    float st = (params_val(P_AIM_SIGN_TILT) < 0.0f) ? -1.0f : 1.0f;
    // Dış döngü: istenen bakış hızı (yanal +sol, dikey +yukarı), derece/s
    float rSide = kp_pan * ez + ki_pan * s_intZ;
    float rUp   = kp_tilt * ey + ki_tilt * s_intY;
    // İleri besleme: ölçülen başlık hızından kendi hareketimizi çıkar = tabanın bakışa verdiği hız; onu dengele
    float kff = params_val(P_AIM_KFF); if (kff < 0.0f) kff = 0.0f; if (kff > 1.0f) kff = 1.0f;
    float wSide = s_wH[2], wUp = -s_wH[1];                                  // z etrafı bakışı sola, +y etrafı aşağı götürür
    float baseSide = wSide - s_ownSidePrev, baseUp = wUp - s_ownUpPrev;
    float kffp = params_val(P_AIM_KFF_PAN); if (kffp < 0.0f) kffp = 0.0f; if (kffp > 1.0f) kffp = 1.0f;
    float uSide = rSide - kffp * baseSide;                                  // pan: ayrı kazanç (varsayılan 0)
    float uUp   = rUp   - kff  * baseUp;
    float vp = sp * uSide / c;
    float vt = st * uUp;
    s_ownSidePrev = sp * v_pan_act * c;                                      // bu örnekteki gerçek hızımız, sonraki örnekte kullanılır
    s_ownUpPrev   = st * v_tilt_act;
    int sat = 0;
    if (vp >  vmax) { vp =  vmax; sat = 1; } if (vp < -vmax) { vp = -vmax; sat = 1; }
    if (vt >  vmax) { vt =  vmax; sat = 1; } if (vt < -vmax) { vt = -vmax; sat = 1; }
    if (sat) { s_intZ -= ez * dt; s_intY -= ey * dt; }                      // doyumda integral büyümesin
    *v_pan = vp; *v_tilt = vt;
    return 1;
}
