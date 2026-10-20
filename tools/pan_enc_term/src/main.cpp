// PAN MOTOR + ENKODER: terminalden döndürme ve enkoderle kapalı çevrim kontrol
// (UART yok; STEP/DIR/EN + AS5048A yazılımsal SPI). Seri port 115200, komut yaz, Enter ile gönder.
// Açılar ANTEN (redüktör çıkışı) derecesidir. Enkoder motorun arka milinde, redüktörden önce.
//
// DÖNDÜRME
//   git <derece>      bulunduğu yerden o kadar git:  git 50   git -12.5
//   konum <derece>    sıfıra göre mutlak konuma git: konum 0
//   hiz <derece/s>    en yüksek hız                  ivme <derece/s2>  hızlanma ve yavaşlama
//   dur               yavaşlayarak dur               acil              anında dur
//   salin <derece>    git, geri gel, sürekli tekrar; "dur" ile biter
//   sifir             şu anki konumu 0 kabul et
//   en 1 | en 0       seçili eksenin sürücüsünü aç / serbest bırak;  en 1 hepsi | en 0 hepsi  üçü birden
// ENKODER VE KONTROL
//   enc               enkoder tanısı: ham değer, AGC, bayraklar, hata sayaçları
//   kalibre           küçük bir hareketle enkoder yönünü ve ölçeğini bul
//   kapali 1 | 0      kapalı çevrim aç/kapa: adım kaçarsa enkoderden görüp düzeltir,
//                     duruşta enkoder hedefte değilse ince düzeltme yapar
//   olubant <derece>  duruşta düzeltme eşiği, anten derecesi (varsayılan 0.15)
//   izle <hz> | izle 0  komut açısı, enkoder açısı ve farkı sürekli yaz
// AYAR
//   eksen pan|tilt|ce   oran <x>   mikro <n>   limit <min> <max>   durum   yardim
//
// Pinler (26 Eyl dizilimi): enkoder SCK=D13 (PA5) MISO=D12 (PA6) MOSI=D8 (PF3).  EN her sürücüye AYRI, düşük = etkin.
//   pan  STEP=D7 (PG12) DIR=D6 (PE9)  EN=D0  (PB7)  CS=D11 (PB5)
//   tilt STEP=D5 (PE11) DIR=D4 (PE14) EN=D41 (PE7)  CS=D10 (PD14)
//   ce   STEP=D3 (PE13) DIR=D2 (PG14) EN=D40 (PE10) CS=D9  (PD15)
//   D41 ve D40, CN10 soketinde D0 pininin altında (D42 tilt UART için ayrıldı).
//   Enkoder adaptöründe 5V ve 3.3V pinlerinin İKİSİ de 3V3, GND ortak.
#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "stm32h7xx_ll_gpio.h"

struct Eksen { const char *ad; PinName step, dir, en, cs; float oran, lim_min, lim_max; };
static Eksen EKSEN[3] = {
    { "pan",  PG_12, PE_9,  PB_7, PB_5,  5.0f, -360.0f, 360.0f },
    { "tilt", PE_11, PE_14, PE_7, PD_14, 5.0f,  -90.0f,  90.0f },
    { "ce",   PE_13, PG_14, PE_10, PD_15, 1.0f,  -45.0f,  45.0f },  // GEÇİCİ: masa testi için ±45. Yarışma değeri 0–4°
};
static const PinName PIN_LED  = PB_0;
static const PinName PIN_SCK  = PA_5;
static const PinName PIN_MISO = PA_6;
static const PinName PIN_MOSI = PF_3;

static const float TICK_HZ = 40000.0f;          // adım kesmesi
static const float DT      = 1.0f / TICK_HZ;
static const float MAX_SPS = 16000.0f;          // en yüksek adım/s
static const float SAYIM_DERECE = 360.0f / 16384.0f;   // enkoder sayımı -> motor derecesi

// ---- ayarlar ----
static int   s_ax      = 0;
static int   s_mikro   = 8;
static float s_hiz     = 20.0f;                 // anten derece/s
static float s_ivme    = 40.0f;                 // anten derece/s2
static float s_olubant = 0.15f;                 // anten derecesi
static volatile bool s_enAx[3] = { true, true, true };   // her sürücünün EN durumu

// ---- adım üreteci (40 kHz kesme) ----
static volatile long  s_pos[3] = { 0, 0, 0 };
static volatile long  s_hedef  = 0;
static volatile float s_v      = 0.0f;          // adım/s, >= 0
static volatile int   s_yon    = 1;
static volatile float s_vmax   = 0.0f, s_acc = 0.0f, s_vmin = 0.0f;
static volatile bool  s_mesgul = false;
static volatile bool  s_sessiz = false;         // hareket enkoder düzeltmesinden; "vardi" yazma
static float s_kalan = 0.0f;
static bool  s_darbe = false;

// ---- enkoder (1 kHz kesme) ----
static volatile uint16_t e_raw   = 0;
static volatile long     e_acc   = 0;           // sıfırdan beri açılmış sayım (çok turlu)
static volatile bool     e_ilk   = true;
static volatile uint8_t  e_agc   = 0, e_diag = 0;   // diag: bit0 OCF, bit1 COF, bit2 yakın, bit3 uzak
static volatile bool     e_diagOk = false;
static volatile uint8_t  e_seri  = 255;         // arka arkaya hatalı okuma
static volatile uint32_t e_ok    = 0, e_hata = 0;
static volatile bool     e_gecerli = false;

// ---- kapalı çevrim ----
static volatile bool     k_acik    = false;
static bool              k_kalibre = false;
static volatile float    k_yon     = 1.0f;      // enkoder yönü, +1 / -1
static volatile float    k_ofset   = 0.0f;      // adım
static volatile float    k_fark    = 0.0f;      // adım: komut - ölçüm
static volatile float    k_olubantAdim = 3.0f;
static volatile uint32_t k_kacirma = 0, k_duzeltme = 0;
static volatile uint8_t  k_hareketKacirma = 0;
static volatile bool     k_olayIptal = false, k_olayEncKayip = false;

// ---- salınım, izleme ----
static long     s_salinAdim = 0, s_salinTaban = 0;
static bool     s_salinUcta = false;
static uint32_t s_salinBekle = 0;
static uint32_t s_izleMs = 0;                   // 0 = kapalı

static float adimMotorDerece() { return 1.8f / (float)s_mikro; }
static float adimBasiDerece()  { return 1.8f / ((float)s_mikro * EKSEN[s_ax].oran); }
static float konumDerece()     { return (float)s_pos[s_ax] * adimBasiDerece(); }
static float olcumAdim()       { return k_yon * (float)e_acc * SAYIM_DERECE / adimMotorDerece() + k_ofset; }
static float encDerece()       { return olcumAdim() * adimBasiDerece(); }

// ================= enkoder SPI (yazılımsal, mod 1, yumuşak kenar) =================
static inline void spiDelay(void) { for (volatile int i = 0; i < 36; i++) { } }

static uint16_t xfer16(PinName cs, uint16_t tx)
{
    uint16_t rx = 0;
    digitalWriteFast(cs, LOW); spiDelay();
    for (int i = 15; i >= 0; i--) {
        digitalWriteFast(PIN_MOSI, (tx >> i) & 1); spiDelay();
        digitalWriteFast(PIN_SCK, HIGH); spiDelay();
        digitalWriteFast(PIN_SCK, LOW);
        rx = (uint16_t)(rx << 1) | (digitalReadFast(PIN_MISO) ? 1 : 0);
    }
    spiDelay(); digitalWriteFast(cs, HIGH); spiDelay();
    return rx;
}

static uint16_t pariteli(uint16_t v)
{
    uint16_t x = v & 0x7FFF, p = x;
    p ^= p >> 8; p ^= p >> 4; p ^= p >> 2; p ^= p >> 1;
    return (p & 1) ? (x | 0x8000) : x;
}
static bool pariteTamam(uint16_t v)
{
    uint16_t p = v; p ^= p >> 8; p ^= p >> 4; p ^= p >> 2; p ^= p >> 1;
    return (p & 1) == 0;
}

static void yumusakCikis(PinName p, GPIO_TypeDef *port, uint32_t llpin, int seviye)
{
    pinMode(pinNametoDigitalPin(p), OUTPUT);
    digitalWriteFast(p, seviye);
    LL_GPIO_SetPinSpeed(port, llpin, LL_GPIO_SPEED_FREQ_LOW);
}

// ================= profil =================
static void profilUygula()
{
    float k = 1.0f / adimBasiDerece();          // adım / anten derecesi
    float vmax = s_hiz * k;
    if (vmax > MAX_SPS) {
        vmax = MAX_SPS; s_hiz = MAX_SPS / k;
        Serial.print("uyari: bu oran ve mikroadimda hiz en fazla ");
        Serial.print(s_hiz, 1); Serial.println(" derece/s olabilir, o degere sinirlandi");
    }
    float vmin = (vmax < 60.0f) ? vmax : 60.0f;
    noInterrupts();
    s_vmax = vmax; s_acc = s_ivme * k; s_vmin = vmin; k_olubantAdim = s_olubant * k;
    interrupts();
}

// Enkoder ölçümünü şu anki adım sayacına eşitle (sıçrama olmasın)
static void hizala()
{
    noInterrupts();
    k_ofset = 0.0f;
    k_ofset = (float)s_pos[s_ax] - olcumAdim();
    interrupts();
}

// ================= 40 kHz: yamuk hız profili + adım üretimi =================
static void tik()
{
    const Eksen &e = EKSEN[s_ax];
    if (s_darbe) { digitalWriteFast(e.step, LOW); s_darbe = false; }

    long pos = s_pos[s_ax];
    long d = s_hedef - pos;
    int  istenen = (d > 0) - (d < 0);
    float v = s_v;

    if (v <= 0.0f) {
        if (istenen == 0) { s_mesgul = false; return; }
        s_yon = istenen;
        digitalWriteFast(e.dir, istenen > 0 ? HIGH : LOW);
        s_kalan = 0.0f; s_mesgul = true;
        s_v = s_vmin;                           // bir tik bekle, DIR otursun
        return;
    }

    bool fren = (istenen != s_yon) || ((float)labs(d) <= v * v / (2.0f * s_acc));
    if (fren || v > s_vmax) {
        v -= s_acc * DT;
        if (v < s_vmin) v = (istenen == s_yon) ? s_vmin : 0.0f;
    } else {
        v += s_acc * DT;
        if (v > s_vmax) v = s_vmax;
    }

    s_kalan += v * DT;
    if (s_kalan >= 1.0f) {
        s_kalan -= 1.0f;
        digitalWriteFast(e.step, HIGH); s_darbe = true;
        pos += s_yon; s_pos[s_ax] = pos;
        if (pos == s_hedef && v <= s_vmin * 2.0f) v = 0.0f;
    }
    s_v = v;
}

// ================= 1 kHz: enkoder okuma + kapalı çevrim =================
static void kontrol()
{
    static uint16_t taniSayac = 0;
    static uint8_t  asimSay = 0, duzSay = 0;
    static uint16_t durgunMs = 0;
    const Eksen &e = EKSEN[s_ax];

    // açı: komut çerçevesi, adım sayacının anlık görüntüsü, cevap çerçevesi
    xfer16(e.cs, pariteli(0x4000 | 0x3FFF));
    long  pos = s_pos[s_ax];
    float v   = s_v;
    uint16_t r = xfer16(e.cs, pariteli(0x4000 | 0x0000));
    bool ok = pariteTamam(r) && !(r & 0x4000);

    if (ok) {
        uint16_t raw = r & 0x3FFF;
        if (e_ilk) { e_ilk = false; e_acc = 0; }
        else {
            int dlt = (int)raw - (int)e_raw;
            if (dlt >  8192) dlt -= 16384;
            if (dlt < -8192) dlt += 16384;
            e_acc += dlt;
        }
        e_raw = raw; e_ok++; e_seri = 0;
    } else {
        e_hata++; if (e_seri < 255) e_seri++;
        if (r & 0x4000) {                       // hata bayrağı takılı kalır, temizle
            xfer16(e.cs, pariteli(0x4000 | 0x0001));
            xfer16(e.cs, pariteli(0x4000 | 0x0000));
        }
    }

    if (++taniSayac >= 200 && ok) {             // 5 Hz: AGC ve tanı bayrakları
        taniSayac = 0;
        xfer16(e.cs, pariteli(0x4000 | 0x3FFD));
        uint16_t g = xfer16(e.cs, pariteli(0x4000 | 0x0000));
        if (pariteTamam(g) && !(g & 0x4000)) {
            e_agc = (uint8_t)(g & 0xFF); e_diag = (uint8_t)((g >> 8) & 0x0F);
            e_diagOk = (e_diag & 0x01) && !(e_diag & 0x08);     // OCF = 1 ve mıknatıs uzak değil
        }
    }
    e_gecerli = (e_seri < 20) && e_diagOk;
    if (!ok) return;

    float olcum = olcumAdim();
    float fark  = (float)pos - olcum;
    k_fark = fark;

    if (!k_acik || !s_enAx[s_ax]) { asimSay = 0; duzSay = 0; durgunMs = 0; return; }
    if (!e_gecerli) { k_acik = false; k_olayEncKayip = true; return; }

    if (v > 0.0f) {
        // HAREKETTE: fark 3 tam adımı geçerse motor senkronu kaybetmiştir (kayma 4 tam adımın katıdır)
        durgunMs = 0; duzSay = 0;
        float esik = 3.0f * (float)s_mikro + v * 0.0005f;
        if (fabsf(fark) > esik) {
            if (++asimSay >= 3) {
                asimSay = 0;
                noInterrupts();
                s_pos[s_ax] = lroundf(olcum);   // gerçek konumu kabul et
                s_v = 0.0f;                     // düşük hızdan yeniden kalk
                if (++k_hareketKacirma >= 5) { s_hedef = s_pos[s_ax]; k_olayIptal = true; }
                interrupts();
                k_kacirma++;
            }
        } else asimSay = 0;
    } else {
        // DURUŞTA: enkoder hedeften ölü banttan fazla uzaksa farkı kadar yürü
        asimSay = 0;
        if (durgunMs < 60000) durgunMs++;
        if (durgunMs >= 150 && fabsf(fark) > k_olubantAdim) {
            if (++duzSay >= 30) {
                duzSay = 0; durgunMs = 0;
                noInterrupts();
                s_pos[s_ax] = lroundf(olcum);
                s_sessiz = true;
                interrupts();
                k_duzeltme++;
            }
        } else duzSay = 0;
    }
}

// ================= komut yardımcıları =================
static void hedefVer(long adim)
{
    float k = 1.0f / adimBasiDerece();
    long lo = lroundf(EKSEN[s_ax].lim_min * k), hi = lroundf(EKSEN[s_ax].lim_max * k);
    if (adim < lo || adim > hi) {
        adim = (adim < lo) ? lo : hi;
        Serial.print("limit: hedef "); Serial.print((float)adim / k, 1);
        Serial.println(" dereceye kirpildi (limit <min> <max> ile degistir)");
    }
    noInterrupts();
    s_hedef = adim; s_sessiz = false; k_hareketKacirma = 0;
    if (adim != s_pos[s_ax]) s_mesgul = true;
    interrupts();
}

static void yavasDur()
{
    s_salinAdim = 0;
    noInterrupts();
    long fren = (s_v > 0.0f) ? (long)ceilf(s_v * s_v / (2.0f * s_acc)) : 0;
    s_hedef = s_pos[s_ax] + (long)s_yon * fren;
    interrupts();
}

static void acilDur()
{
    s_salinAdim = 0;
    noInterrupts();
    s_hedef = s_pos[s_ax]; s_v = 0.0f;
    interrupts();
}

static bool bekleDursun(uint32_t zamanAsimiMs)
{
    uint32_t t0 = millis();
    delay(5);
    while (s_mesgul) { if (millis() - t0 > zamanAsimiMs) return false; delay(2); }
    return true;
}

static void encYaz()
{
    Serial.print("enkoder "); Serial.print(EKSEN[s_ax].ad);
    Serial.print(": ham="); Serial.print(e_raw);
    Serial.print("  motor acisi="); Serial.print((float)e_raw * SAYIM_DERECE, 2);
    Serial.print("  AGC="); Serial.print(e_agc);
    Serial.print("  OCF="); Serial.print(e_diag & 1);
    Serial.print(" COF="); Serial.print((e_diag >> 1) & 1);
    Serial.print(" yakin="); Serial.print((e_diag >> 2) & 1);
    Serial.print(" uzak="); Serial.print((e_diag >> 3) & 1);
    Serial.print("  ok="); Serial.print(e_ok); Serial.print(" hata="); Serial.println(e_hata);
    if (e_seri >= 20)             Serial.println("  -> CEVAP YOK: enkoder kablosu, CS pini ya da besleme (5V ve 3.3V pinlerinin ikisi de 3V3)");
    else if (e_diag & 0x08)       Serial.println("  -> GECERSIZ: miknatis cok uzak (AGC 255). Miknatis ile cip arasi 1-2 mm olmali");
    else if (!(e_diag & 0x01))    Serial.println("  -> GECERSIZ: cip hazir degil (OCF=0), miknatis algilanmiyor");
    else if (e_diag & 0x04)       Serial.println("  -> kullanilir ama miknatis cok yakin, dogruluk dusuk");
    else                          Serial.println("  -> SAGLIKLI (AGC 60-200 arasi iyi)");
}

static void durumYaz()
{
    const Eksen &e = EKSEN[s_ax];
    Serial.print("eksen="); Serial.print(e.ad);
    Serial.print("  komut="); Serial.print(konumDerece(), 2);
    Serial.print("  hedef="); Serial.print((float)s_hedef * adimBasiDerece(), 2);
    Serial.print("  enkoder="); Serial.print(encDerece(), 2);
    Serial.print("  fark="); Serial.print(k_fark * adimBasiDerece(), 3); Serial.println(" derece");
    Serial.print("hiz="); Serial.print(s_hiz, 1); Serial.print(" d/s  ivme="); Serial.print(s_ivme, 1);
    Serial.print(" d/s2  oran="); Serial.print(e.oran, 2); Serial.print("  mikro="); Serial.print(s_mikro);
    Serial.print("  limit="); Serial.print(e.lim_min, 0); Serial.print(".."); Serial.print(e.lim_max, 0);
    Serial.print("  surucu="); Serial.print(s_enAx[s_ax] ? "acik" : "serbest");
    Serial.print(s_mesgul ? "  HAREKETTE" : "  duruyor");
    Serial.print("  en=");
    for (int i = 0; i < 3; i++) { Serial.print(EKSEN[i].ad); Serial.print(s_enAx[i] ? ":1 " : ":0 "); }
    Serial.println();
    Serial.print("enkoder="); Serial.print(e_gecerli ? "saglikli" : "GECERSIZ"); Serial.print(" (AGC "); Serial.print(e_agc);
    Serial.print(")  kalibre="); Serial.print(k_kalibre ? "evet" : "hayir");
    Serial.print("  kapali cevrim="); Serial.print(k_acik ? "ACIK" : "kapali");
    Serial.print("  olubant="); Serial.print(s_olubant, 2);
    Serial.print("  kacirma="); Serial.print(k_kacirma); Serial.print("  duzeltme="); Serial.println(k_duzeltme);
}

static void yardimYaz()
{
    Serial.println("dondurme: git <d> | konum <d> | hiz <d/s> | ivme <d/s2> | dur | acil | salin <d> | sifir | en 1|0");
    Serial.println("enkoder : enc | kalibre | kapali 1|0 | olubant <d> | izle <hz>|0");
    Serial.println("ayar    : eksen pan|tilt|ce | oran <x> | mikro <n> | limit <min> <max> | durum | yardim");
}

// Küçük bir hareketle enkoder yönünü ve ölçeğini bul
static bool kalibreEt()
{
    if (!e_gecerli) { Serial.println("kalibre: enkoder gecersiz, once onu duzelt:"); encYaz(); return false; }
    if (!s_enAx[s_ax]) { Serial.println("kalibre: once surucuyu ac (en 1)"); return false; }
    if (s_mesgul)   { Serial.println("kalibre: once dur"); return false; }
    bool eskiAcik = k_acik; k_acik = false; s_salinAdim = 0;

    long n = lroundf(3.0f / adimBasiDerece());          // 3 derece anten
    if (n < 8) n = 8;
    float vE = s_vmax, aE = s_acc, mE = s_vmin;
    noInterrupts(); s_vmax = 150.0f; s_acc = 600.0f; s_vmin = 60.0f; interrupts();

    Serial.print("kalibre: "); Serial.print((float)n * adimBasiDerece(), 2); Serial.println(" derece ileri gidip geri geliyor...");
    long c0 = e_acc, p0 = s_pos[s_ax];
    noInterrupts(); s_hedef = p0 + n; s_sessiz = true; s_mesgul = true; interrupts();
    bool tamam = bekleDursun(8000); delay(300);
    long c1 = e_acc;
    noInterrupts(); s_hedef = p0; s_sessiz = true; s_mesgul = true; interrupts();
    tamam = bekleDursun(8000) && tamam; delay(200);
    noInterrupts(); s_vmax = vE; s_acc = aE; s_vmin = mE; interrupts();
    if (!tamam) { Serial.println("kalibre: hareket zamaninda bitmedi"); return false; }

    float beklenen = (float)n * adimMotorDerece();       // motor derecesi
    float olculen  = (float)(c1 - c0) * SAYIM_DERECE;
    float oranO    = fabsf(olculen) / beklenen;
    Serial.print("kalibre: motor beklenen "); Serial.print(beklenen, 2);
    Serial.print(" derece, enkoder olctu "); Serial.print(olculen, 2); Serial.println(" derece");

    if (oranO < 0.2f) {
        Serial.println("kalibre BASARISIZ: enkoder hareket gormedi. Motor donmuyor ya da miknatis mile bagli degil.");
        return false;
    }
    k_yon = (olculen >= 0.0f) ? 1.0f : -1.0f;
    k_kalibre = true;
    Serial.print("kalibre: enkoder yonu "); Serial.print(k_yon > 0 ? "AYNI" : "TERS");
    Serial.print(", olcek "); Serial.println(oranO, 3);
    if (oranO < 0.85f || oranO > 1.15f) {
        Serial.print("UYARI: olcek 1 degil. Surucunun gercek mikroadimi buyuk ihtimalle ");
        Serial.print(lroundf((float)s_mikro / oranO));
        Serial.println(". 'mikro <n>' ile duzeltip kalibreyi tekrarla; bu haliyle kapali cevrim acilmaz.");
        k_kalibre = false; return false;
    }
    hizala();
    k_acik = eskiAcik;
    return true;
}

static bool esit(const char *a, const char *b, const char *kisa) { return !strcmp(a, b) || !strcmp(a, kisa); }

static void komut(char *satir)
{
    for (char *p = satir; *p; p++) { if (*p == ',') *p = '.'; else *p = (char)tolower((unsigned char)*p); }
    char *k  = strtok(satir, " \t");
    char *a1 = strtok(nullptr, " \t");
    char *a2 = strtok(nullptr, " \t");
    if (!k) return;
    float x1 = a1 ? (float)atof(a1) : 0.0f;
    float adimDer = 1.0f / adimBasiDerece();

    if (esit(k, "git", "g")) {
        if (!a1) { Serial.println("kullanim: git <derece>"); return; }
        s_salinAdim = 0;
        hedefVer(s_pos[s_ax] + lroundf(x1 * adimDer));
        Serial.print("gidiyor: "); Serial.print(x1, 2); Serial.println(" derece");
    } else if (esit(k, "konum", "k")) {
        if (!a1) { Serial.println("kullanim: konum <derece>"); return; }
        s_salinAdim = 0;
        hedefVer(lroundf(x1 * adimDer));
        Serial.print("hedef konum: "); Serial.print(x1, 2); Serial.println(" derece");
    } else if (esit(k, "hiz", "h")) {
        if (!a1 || x1 <= 0.0f) { Serial.println("kullanim: hiz <derece/s>, sifirdan buyuk"); return; }
        s_hiz = x1; profilUygula();
        Serial.print("hiz = "); Serial.print(s_hiz, 1); Serial.println(" derece/s");
    } else if (esit(k, "ivme", "i")) {
        if (!a1 || x1 <= 0.0f) { Serial.println("kullanim: ivme <derece/s2>, sifirdan buyuk"); return; }
        s_ivme = x1; profilUygula();
        Serial.print("ivme = "); Serial.print(s_ivme, 1); Serial.println(" derece/s2");
    } else if (esit(k, "dur", "d")) {
        yavasDur(); Serial.println("duruyor (yavaslayarak)");
    } else if (esit(k, "acil", "a") || !strcmp(k, "x")) {
        acilDur(); Serial.println("ACIL DURUS");
    } else if (esit(k, "salin", "s")) {
        if (!a1 || x1 == 0.0f) { Serial.println("kullanim: salin <derece>"); return; }
        s_salinTaban = s_pos[s_ax]; s_salinAdim = lroundf(x1 * adimDer); s_salinUcta = true;
        hedefVer(s_salinTaban + s_salinAdim);
        Serial.print("salinim: "); Serial.print(x1, 1); Serial.println(" derece gidip geliyor, dur ile biter");
    } else if (!strcmp(k, "sifir")) {
        if (s_mesgul) { Serial.println("once dur"); return; }
        noInterrupts(); s_pos[s_ax] = 0; s_hedef = 0; interrupts();
        hizala();
        Serial.println("konum sifirlandi (adim sayaci ve enkoder)");
    } else if (!strcmp(k, "en")) {
        if (!a1) { Serial.println("kullanim: en 1 | en 0 (secili eksen)   en 1 hepsi | en 0 hepsi"); return; }
        bool ac = (x1 != 0.0f);
        if (a2 && !strcmp(a2, "hepsi")) {
            if (!ac) acilDur();
            for (int i = 0; i < 3; i++) { s_enAx[i] = ac; digitalWriteFast(EKSEN[i].en, ac ? LOW : HIGH); }
            Serial.println(ac ? "uc surucu de acik" : "uc surucu de serbest");
            return;
        }
        if (!ac) { acilDur(); s_enAx[s_ax] = false; digitalWriteFast(EKSEN[s_ax].en, HIGH); Serial.println("surucu serbest (motor tutmuyor); enkoder elle cevirmeyi izlemeye devam eder"); return; }
        if (k_kalibre && e_gecerli) {           // elle çevrildiyse yeni konumu enkoderden al
            noInterrupts(); long p = lroundf(olcumAdim()); s_pos[s_ax] = p; s_hedef = p; s_v = 0.0f; interrupts();
            Serial.print("konum enkoderden alindi: "); Serial.print(konumDerece(), 2); Serial.println(" derece");
        }
        digitalWriteFast(EKSEN[s_ax].en, LOW); s_enAx[s_ax] = true;
        Serial.println("surucu acik");
    } else if (!strcmp(k, "enc")) {
        encYaz();
    } else if (!strcmp(k, "kalibre")) {
        if (kalibreEt()) Serial.println("kalibre TAMAM. 'kapali 1' ile kapali cevrimi acabilirsin.");
    } else if (!strcmp(k, "kapali")) {
        if (!a1) { Serial.println("kullanim: kapali 1 | kapali 0"); return; }
        if (x1 == 0.0f) { k_acik = false; Serial.println("kapali cevrim KAPALI (acik cevrim, enkoder yalniz izleniyor)"); return; }
        if (!k_kalibre && !kalibreEt()) { Serial.println("kapali cevrim acilamadi"); return; }
        if (!e_gecerli) { Serial.println("kapali cevrim acilamadi: enkoder gecersiz"); encYaz(); return; }
        hizala();
        k_acik = true;
        Serial.println("kapali cevrim ACIK: adim kacarsa enkoderden gorup duzeltir, durusta ince duzeltme yapar");
    } else if (!strcmp(k, "olubant")) {
        if (!a1 || x1 <= 0.0f) { Serial.println("kullanim: olubant <derece>, ornek 0.15"); return; }
        s_olubant = x1; profilUygula();
        Serial.print("olu bant = "); Serial.print(s_olubant, 3); Serial.println(" derece");
    } else if (!strcmp(k, "izle")) {
        float hz = a1 ? x1 : 5.0f;
        if (hz <= 0.0f) { s_izleMs = 0; Serial.println("izleme kapali"); return; }
        if (hz > 50.0f) hz = 50.0f;
        s_izleMs = (uint32_t)(1000.0f / hz);
        Serial.println("izleme acik; kapatmak icin: izle 0");
    } else if (esit(k, "eksen", "e")) {
        if (s_mesgul) { Serial.println("once dur"); return; }
        int yeni = -1;
        for (int i = 0; i < 3; i++) if (a1 && !strcmp(a1, EKSEN[i].ad)) yeni = i;
        if (yeni < 0) { Serial.println("kullanim: eksen pan | tilt | ce"); return; }
        noInterrupts();
        k_acik = false; k_kalibre = false; k_yon = 1.0f; k_ofset = 0.0f;
        s_ax = yeni; s_hedef = s_pos[yeni]; s_v = 0.0f;
        e_ilk = true; e_acc = 0; e_seri = 255; e_diagOk = false; e_gecerli = false; e_ok = 0; e_hata = 0;
        interrupts();
        profilUygula(); delay(450); hizala(); durumYaz();
    } else if (!strcmp(k, "oran")) {
        if (s_mesgul) { Serial.println("once dur"); return; }
        if (!a1 || x1 <= 0.0f) { Serial.println("kullanim: oran <x>"); return; }
        EKSEN[s_ax].oran = x1; profilUygula(); durumYaz();
    } else if (!strcmp(k, "mikro")) {
        if (s_mesgul) { Serial.println("once dur"); return; }
        int n = a1 ? atoi(a1) : 0;
        if (n < 1 || n > 256) { Serial.println("kullanim: mikro 8 | 16 | 32 | 64"); return; }
        bool acikti = k_acik; k_acik = false;
        s_mikro = n; profilUygula(); hizala();
        if (acikti) { k_kalibre = false; Serial.println("mikroadim degisti: kapali cevrim kapatildi, 'kapali 1' ile yeniden ac"); }
        durumYaz();
    } else if (!strcmp(k, "limit")) {
        if (!a1 || !a2 || atof(a1) >= atof(a2)) { Serial.println("kullanim: limit <min> <max>"); return; }
        EKSEN[s_ax].lim_min = (float)atof(a1); EKSEN[s_ax].lim_max = (float)atof(a2); durumYaz();
    } else if (!strcmp(k, "durum") || !strcmp(k, "?")) {
        durumYaz();
    } else if (!strcmp(k, "yardim") || !strcmp(k, "help")) {
        yardimYaz();
    } else {
        Serial.print("bilinmeyen komut: "); Serial.println(k); yardimYaz();
    }
}

void setup()
{
    Serial.begin(115200);
    for (int i = 0; i < 3; i++) {
        pinMode(pinNametoDigitalPin(EKSEN[i].step), OUTPUT); digitalWriteFast(EKSEN[i].step, LOW);
        pinMode(pinNametoDigitalPin(EKSEN[i].dir),  OUTPUT); digitalWriteFast(EKSEN[i].dir,  LOW);
        pinMode(pinNametoDigitalPin(EKSEN[i].en),   OUTPUT); digitalWriteFast(EKSEN[i].en,   HIGH);   // önce kapalı
    }
    pinMode(pinNametoDigitalPin(PIN_LED), OUTPUT); digitalWriteFast(PIN_LED, LOW);

    // Enkoder SPI: yumuşak kenarlı çıkışlar; üç CS de yüksekte dursun (ortak hatta diğer enkoderler sussun)
    yumusakCikis(PIN_SCK,  GPIOA, LL_GPIO_PIN_5,  LOW);
    yumusakCikis(PIN_MOSI, GPIOF, LL_GPIO_PIN_3,  LOW);
    yumusakCikis(PB_5,     GPIOB, LL_GPIO_PIN_5,  HIGH);
    yumusakCikis(PD_14,    GPIOD, LL_GPIO_PIN_14, HIGH);
    yumusakCikis(PD_15,    GPIOD, LL_GPIO_PIN_15, HIGH);
    pinMode(pinNametoDigitalPin(PIN_MISO), INPUT);

    profilUygula();

    HardwareTimer *t1 = new HardwareTimer(TIM13);
    t1->setOverflow((uint32_t)TICK_HZ, HERTZ_FORMAT);
    t1->attachInterrupt(tik);
    t1->setInterruptPriority(1, 0);
    t1->resume();

    HardwareTimer *t2 = new HardwareTimer(TIM14);
    t2->setOverflow(1000, HERTZ_FORMAT);
    t2->attachInterrupt(kontrol);
    t2->setInterruptPriority(3, 0);
    t2->resume();

    delay(1000);
    for (int i = 0; i < 3; i++) digitalWriteFast(EKSEN[i].en, LOW);      // üç sürücü de tutsun (tilt anteni taşıyor)
    Serial.println("\nPAN MOTOR + ENKODER terminali hazir. Uc surucu acik, kapali cevrim kapali.");
    yardimYaz(); durumYaz(); encYaz();
}

void loop()
{
    // satır oku; terminalin araya soktuğu ESC dizilerini ve harf dışı baytları at
    static char buf[64]; static uint8_t n = 0; static uint8_t esc = 0;
    while (Serial.available() > 0) {
        uint8_t c = (uint8_t)Serial.read();
        if (esc == 1) { esc = (c == '[') ? 2 : 0; continue; }
        if (esc == 2) { if (c >= 0x40 && c <= 0x7E) esc = 0; continue; }
        if (c == 0x1B) { esc = 1; continue; }
        if (c == '\r' || c == '\n') { if (n) { buf[n] = 0; komut(buf); n = 0; } }
        else if (c == 8 || c == 127) { if (n) n--; }
        else if (c < 0x20 || c > 0x7E) { }
        else if (n < sizeof buf - 1) buf[n++] = (char)c;
    }

    // kapalı çevrim olayları
    static uint32_t sonKacirma = 0;
    if (k_kacirma != sonKacirma) {
        sonKacirma = k_kacirma;
        Serial.print("KACIRMA: motor adim kacirdi, enkoderden duzeltildi (toplam "); Serial.print(sonKacirma); Serial.println(")");
    }
    if (k_olayIptal) {
        k_olayIptal = false; s_salinAdim = 0;
        Serial.println("HAREKET IPTAL: motor ust uste 5 kez takildi. hiz/ivme dusur ya da surucu akimini artir.");
    }
    if (k_olayEncKayip) {
        k_olayEncKayip = false;
        Serial.println("ENKODER KAYBI: kapali cevrim kapatildi, acik cevrim suruyor."); encYaz();
    }

    // varış bildirimi ve salınım
    static bool onceki = false;
    bool mesgul = s_mesgul;
    digitalWriteFast(PIN_LED, mesgul ? HIGH : LOW);
    if (onceki && !mesgul) {
        if (s_salinAdim != 0) s_salinBekle = millis() + 1000;
        else if (!s_sessiz) {
            Serial.print("vardi: komut="); Serial.print(konumDerece(), 2);
            Serial.print("  enkoder="); Serial.print(encDerece(), 2);
            Serial.print("  fark="); Serial.print(k_fark * adimBasiDerece(), 3);
            Serial.println(k_kalibre ? " derece" : " derece (enkoder kalibre edilmedi)");
        }
    }
    onceki = mesgul;
    if (s_salinAdim != 0 && !mesgul && (int32_t)(millis() - s_salinBekle) >= 0) {
        s_salinUcta = !s_salinUcta;
        hedefVer(s_salinTaban + (s_salinUcta ? s_salinAdim : 0));
        onceki = true;
    }

    // izleme
    static uint32_t sonIzle = 0;
    if (s_izleMs && millis() - sonIzle >= s_izleMs) {
        sonIzle = millis();
        Serial.print("komut="); Serial.print(konumDerece(), 2);
        Serial.print("  enkoder="); Serial.print(encDerece(), 2);
        Serial.print("  fark="); Serial.print(k_fark * adimBasiDerece(), 3);
        Serial.print("  agc="); Serial.print(e_agc);
        Serial.println(k_acik ? "  [kapali cevrim]" : "");
    }
}
