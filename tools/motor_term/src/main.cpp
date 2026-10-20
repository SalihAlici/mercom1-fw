// TERMİNALDEN MOTOR KONTROLÜ  (UART yok, yalnız STEP/DIR/EN). Üç eksen AYNI ANDA hareket edebilir.
// Seri port 115200. Komut yaz, Enter ile gönder. Açılar ANTEN (redüktör çıkışı) derecesidir.
//
// SEÇİLİ EKSEN
//   git <derece>      bulunduğu yerden o kadar git:  git 50   git -12.5
//   konum <derece>    sıfıra göre mutlak konuma git: konum 0
//   salin <derece>    o kadar git, geri gel, sürekli tekrar; "dur" ile biter
//   sifir             şu anki konumu 0 kabul et                 (sifir hepsi: üç eksen)
//   en 1 | en 0       sürücüyü aç / serbest bırak               (en 1 hepsi | en 0 hepsi: üçü birden)
//   eksen pan|tilt|ce seçili ekseni değiştir (hareket sürerken de olur)
//   oran <x>          redüktör oranı (motor turu / anten turu)
//   mikro <n>         mikroadım: MS1/MS2 boşta 8, ikisi 3V3 ise 16, yalnız MS2 3V3 ise 64   (mikro <n> hepsi)
//   limit <min> <max> yazılım sınırı, derece
//   tavan <hiz> <ivme> bu eksenin çıkabileceği en yüksek hız ve ivme; ortak değer bunu aşarsa eksen tavanıyla çalışır.
//                     CE redüktörsüz olduğu için 60 / 300 ile başlar. tavan 0 0 = tavan yok
// ÜÇ EKSEN BİRDEN
//   hepsi <derece>      üç motoru da bulunduğu yerden o kadar, AYNI ANDA döndür:  hepsi 90   hepsi -90
//   hepsikonum <derece> üç motoru da aynı anda o mutlak konuma götür:              hepsikonum 0
//   hepsisalin          üç motor aynı anda, HER BİRİ KENDİ LİMİTLERİ arasında gidip gelir; "dur" ile biter
//   hepsisalin <derece> üç motor aynı anda bulunduğu yerden o kadar gidip gelir
// GENEL
//   hiz <derece/s>    en yüksek hız (üç eksene ortak)      ivme <derece/s2>  hızlanma ve yavaşlama ivmesi
//   dur               bütün eksenleri yavaşlayarak durdur, salınımı bitir
//   acil              bütün eksenleri anında durdur (adım kaçabilir)
//   durum             ayarları ve konumları yaz  (kısaltma: ?)          yardim
// Kısaltmalar: g k h i d a s e
//
// Pinler: EN her sürücüye AYRI, düşük = etkin. D41 ve D40, CN10 soketinde D0 pininin altında (D42 tilt UART için ayrıldı).
//   pan  STEP=D7 (PG12) DIR=D6 (PE9)  EN=D0  (PB7)
//   tilt STEP=D5 (PE11) DIR=D4 (PE14) EN=D41 (PE7)
//   ce   STEP=D3 (PE13) DIR=D2 (PG14) EN=D40 (PE10)
#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

// hiz_tavan, ivme_tavan: o eksenin çıkabileceği en yüksek değer (derece/s, derece/s2); 0 = tavan yok.
// CE redüktörsüz: aynı anten ivmesi için motordan pan ve tilt'in 5 katı tork ister, yük ataleti de 25'e bölünmez.
// "hiz 200 / ivme 4000" pan ve tilt'te döner, CE'de motor kilitlenir. Bu yüzden CE ortak değerlere kendi tavanıyla uyar.
struct Eksen { const char *ad; PinName step, dir, en; float oran, lim_min, lim_max; int mikro; float hiz_tavan, ivme_tavan; };
static Eksen EKSEN[3] = {
    { "pan",  PG_12, PE_9,  PB_7,  5.0f, -360.0f, 360.0f, 8,  0.0f,   0.0f },
    { "tilt", PE_11, PE_14, PE_7,  5.0f,  -90.0f,  90.0f, 8,  0.0f,   0.0f },
    { "ce",   PE_13, PG_14, PE_10, 1.0f,  -45.0f,  45.0f, 8, 60.0f, 300.0f },  // GEÇİCİ: masa testi için ±45. Yarışma değeri 0–4°
};
static const PinName PIN_LED = PB_0;

static const float    TICK_HZ = 40000.0f;       // adım kesmesi
static const float    DT      = 1.0f / TICK_HZ;
static const float    MAX_SPS = 16000.0f;       // eksen başına en yüksek adım/s (kesme 40 kHz, darbe 1 tik)
static const uint32_t SALIN_BEKLE_MS = 1000;    // salınımda uçlarda bekleme

// ---- ayarlar (anten derecesi cinsinden; hız ve ivme üç eksene ortak) ----
static int   s_ax   = 0;                        // seçili eksen
static float s_hiz  = 20.0f;                    // derece/s
static float s_ivme = 40.0f;                    // derece/s2
static bool  s_enAx[3] = { true, true, true };  // her sürücünün EN durumu

// ---- kesme ile paylaşılan durum (eksen başına, adım cinsinden) ----
static volatile long  s_pos[3]    = { 0, 0, 0 };
static volatile long  s_hedef[3]  = { 0, 0, 0 };
static volatile float s_v[3]      = { 0.0f, 0.0f, 0.0f };   // anlık hız, adım/s, >= 0
static volatile int   s_yon[3]    = { 1, 1, 1 };
static volatile float s_vmax[3], s_acc[3], s_vmin[3];
static volatile bool  s_mesgul[3] = { false, false, false };
static float s_kalan[3] = { 0.0f, 0.0f, 0.0f };             // DDA birikimi
static bool  s_darbe[3] = { false, false, false };

// ---- salınım (eksen başına): A ile B arasında gidip gelir ----
static bool     s_salin[3] = { false, false, false };
static long     s_salinA[3], s_salinB[3];
static bool     s_salinUcta[3];                 // true: B ucuna gidiyor ya da B ucunda
static uint32_t s_salinBekle[3];

static float adimDerece(int i)  { return 1.8f / ((float)EKSEN[i].mikro * EKSEN[i].oran); }
static float konumDerece(int i) { return (float)s_pos[i] * adimDerece(i); }
static bool  herhangiMesgul()   { return s_mesgul[0] || s_mesgul[1] || s_mesgul[2]; }
// Eksenin gerçekte kullandığı hız ve ivme: ortak değer, eksenin tavanıyla sınırlı
static float eksenHiz(int i)    { float t = EKSEN[i].hiz_tavan;  return (t > 0.0f && s_hiz  > t) ? t : s_hiz; }
static float eksenIvme(int i)   { float t = EKSEN[i].ivme_tavan; return (t > 0.0f && s_ivme > t) ? t : s_ivme; }

// Hız ve ivmeyi her eksenin adım birimine çevirip kesmeye ver
static void profilUygula()
{
    float hizMax = 1e9f;                        // en yavaş kalan eksen ortak hızı sınırlar
    for (int i = 0; i < 3; i++) { float m = MAX_SPS * adimDerece(i); if (m < hizMax) hizMax = m; }
    if (s_hiz > hizMax) {
        s_hiz = hizMax;                         // gösterilen değer de gerçek sınırla aynı olsun
        Serial.print("uyari: bu oran ve mikroadimda hiz en fazla ");
        Serial.print(hizMax, 1); Serial.println(" derece/s olabilir, o degere sinirlandi");
    }
    for (int i = 0; i < 3; i++) {
        float k = 1.0f / adimDerece(i);         // adım / derece
        float vmax = eksenHiz(i) * k;
        float vmin = (vmax < 60.0f) ? vmax : 60.0f;   // kalkış ve duruş hızı
        noInterrupts();
        s_vmax[i] = vmax; s_acc[i] = eksenIvme(i) * k; s_vmin[i] = vmin;
        interrupts();
    }
}

// Ortak hız ya da ivme bir eksenin tavanını aşıyorsa haber ver
static void tavanNotu()
{
    for (int i = 0; i < 3; i++) {
        if (eksenHiz(i) >= s_hiz && eksenIvme(i) >= s_ivme) continue;
        Serial.print("not: "); Serial.print(EKSEN[i].ad); Serial.print(" kendi tavaniyla calisir: hiz ");
        Serial.print(eksenHiz(i), 1); Serial.print(" d/s, ivme "); Serial.print(eksenIvme(i), 1);
        Serial.println(" d/s2  (degistirmek icin o ekseni secip: tavan <hiz> <ivme>)");
    }
}

// 40 kHz: her eksen için yamuk hız profili + adım üretimi
static void tik()
{
    for (int i = 0; i < 3; i++) {
        const Eksen &e = EKSEN[i];
        if (s_darbe[i]) { digitalWriteFast(e.step, LOW); s_darbe[i] = false; }

        long pos = s_pos[i];
        long d = s_hedef[i] - pos;
        int  istenen = (d > 0) - (d < 0);
        float v = s_v[i];

        if (v <= 0.0f) {                        // duruyor
            if (istenen == 0) { s_mesgul[i] = false; continue; }
            s_yon[i] = istenen;
            digitalWriteFast(e.dir, istenen > 0 ? HIGH : LOW);
            s_kalan[i] = 0.0f; s_mesgul[i] = true;
            s_v[i] = s_vmin[i];                 // bir tik bekle, DIR otursun
            continue;
        }

        bool fren = (istenen != s_yon[i]) || ((float)labs(d) <= v * v / (2.0f * s_acc[i]));
        if (fren || v > s_vmax[i]) {
            v -= s_acc[i] * DT;
            if (v < s_vmin[i]) v = (istenen == s_yon[i]) ? s_vmin[i] : 0.0f;   // hedefe kadar yavaş git; ters yöndeyse dur
        } else {
            v += s_acc[i] * DT;
            if (v > s_vmax[i]) v = s_vmax[i];
        }

        s_kalan[i] += v * DT;
        if (s_kalan[i] >= 1.0f) {
            s_kalan[i] -= 1.0f;
            digitalWriteFast(e.step, HIGH); s_darbe[i] = true;
            pos += s_yon[i]; s_pos[i] = pos;
            // vardı. İkinci koşul: yüksek ivmede son adımda hız sqrt(2a) dolayında kalır; eskiden 2*vmin'i aşınca
            // hedefi 1 adım geçip geri dönüyordu. Bu hızdan durmak profilin zaten istediği yavaşlamayla aynı.
            if (pos == s_hedef[i] && (v <= s_vmin[i] * 2.0f || v * v <= 4.0f * s_acc[i])) v = 0.0f;
        }
        s_v[i] = v;
    }
}

// Hedefi eksenin limitlerine kırpar
static long kirp(int ax, long adim, bool yaz)
{
    float k = 1.0f / adimDerece(ax);
    long lo = lroundf(EKSEN[ax].lim_min * k), hi = lroundf(EKSEN[ax].lim_max * k);
    if (adim >= lo && adim <= hi) return adim;
    adim = (adim < lo) ? lo : hi;
    if (yaz) {
        Serial.print("limit: "); Serial.print(EKSEN[ax].ad); Serial.print(" hedefi "); Serial.print((float)adim / k, 1);
        Serial.println(" dereceye kirpildi (limit <min> <max> ile degistir)");
    }
    return adim;
}

static void hedefVer(int ax, long adim)
{
    adim = kirp(ax, adim, true);
    noInterrupts();
    s_hedef[ax] = adim;
    if (adim != s_pos[ax]) s_mesgul[ax] = true;
    interrupts();
}

static void serbestUyar(int ax)
{
    if (s_enAx[ax]) return;
    Serial.print("uyari: "); Serial.print(EKSEN[ax].ad); Serial.println(" surucusu serbest, motor donmez (en 1)");
}

// A ile B arasında salınımı başlat; önce B ucuna gider
static void salinBaslat(int ax, long a, long b)
{
    s_salinA[ax] = kirp(ax, a, false); s_salinB[ax] = kirp(ax, b, true);
    s_salinUcta[ax] = true; s_salinBekle[ax] = millis(); s_salin[ax] = true;
    serbestUyar(ax);
    hedefVer(ax, s_salinB[ax]);
}

static void yavasDur()
{
    noInterrupts();
    for (int i = 0; i < 3; i++) {
        s_salin[i] = false;
        long fren = (s_v[i] > 0.0f) ? (long)ceilf(s_v[i] * s_v[i] / (2.0f * s_acc[i])) : 0;
        s_hedef[i] = s_pos[i] + (long)s_yon[i] * fren;
    }
    interrupts();
}

static void eksenAcilDur(int ax)
{
    noInterrupts();
    s_salin[ax] = false; s_hedef[ax] = s_pos[ax]; s_v[ax] = 0.0f;
    interrupts();
}

static void acilDur() { for (int i = 0; i < 3; i++) eksenAcilDur(i); }

static void konumlarYaz()
{
    for (int i = 0; i < 3; i++) { Serial.print(EKSEN[i].ad); Serial.print("="); Serial.print(konumDerece(i), 2); if (i < 2) Serial.print(" "); }
}

// "pan+ce" gibi liste; hiçbiri yoksa "yok"
static void listeYaz(const bool *b0, const volatile bool *b1)
{
    bool ilk = true;
    for (int i = 0; i < 3; i++) {
        if (!(b0 ? b0[i] : b1[i])) continue;
        if (!ilk) Serial.print("+");
        Serial.print(EKSEN[i].ad); ilk = false;
    }
    if (ilk) Serial.print("yok");
}

// İlk iki satırın başı renkli terminalin (terminal_gui.py) ayrıştırdığı biçimdedir; sona ekleme serbest, başı değiştirme.
static void durumYaz()
{
    const Eksen &e = EKSEN[s_ax];
    Serial.print("eksen="); Serial.print(e.ad);
    Serial.print("  konum="); Serial.print(konumDerece(s_ax), 2);
    Serial.print("  hedef="); Serial.print((float)s_hedef[s_ax] * adimDerece(s_ax), 2);
    Serial.print(" derece  ("); Serial.print(s_pos[s_ax]); Serial.print(" adim)   | ");
    konumlarYaz();
    Serial.print("  hareket="); listeYaz(nullptr, s_mesgul);
    Serial.print("  salinim="); listeYaz(s_salin, nullptr);
    Serial.println();
    Serial.print("hiz="); Serial.print(s_hiz, 1); Serial.print(" d/s  ivme="); Serial.print(s_ivme, 1);
    Serial.print(" d/s2  oran="); Serial.print(e.oran, 2); Serial.print("  mikro="); Serial.print(e.mikro);
    Serial.print("  limit="); Serial.print(e.lim_min, 0); Serial.print(".."); Serial.print(e.lim_max, 0);
    Serial.print("  surucu="); Serial.print(s_enAx[s_ax] ? "acik" : "serbest");
    Serial.print(s_mesgul[s_ax] ? "  HAREKETTE" : "  duruyor");
    Serial.print("  en=");
    for (int i = 0; i < 3; i++) { Serial.print(EKSEN[i].ad); Serial.print(s_enAx[i] ? ":1 " : ":0 "); }
    Serial.print(" tavan=");
    if (e.hiz_tavan > 0.0f || e.ivme_tavan > 0.0f) { Serial.print(e.hiz_tavan, 0); Serial.print("/"); Serial.print(e.ivme_tavan, 0); }
    else Serial.print("yok");
    Serial.print("  gercek="); Serial.print(eksenHiz(s_ax), 1); Serial.print("/"); Serial.print(eksenIvme(s_ax), 1);
    Serial.println();
}

static void yardimYaz()
{
    Serial.println("komutlar: git <d> | konum <d> | hiz <d/s> | ivme <d/s2> | dur | acil | salin <d> | sifir");
    Serial.println("          en 1|0 | eksen pan|tilt|ce | oran <x> | mikro <n> | limit <min> <max> | durum | yardim");
    Serial.println("komutlar: hepsi <d> (uc motor birden) | hepsikonum <d> | hepsisalin (her eksen kendi limitinde) | hepsisalin <d>");
    Serial.println("          tavan <hiz> <ivme> = secili eksenin en yuksek hiz ve ivmesi (ce reduktorsuz: 60 / 300), 0 0 = tavan yok");
    Serial.println("          tel 1|0 = secili eksenin STEP ve DIR pinlerini sabit 3,3 V / 0 V yapar (multimetreyle tel sinamasi)");
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
    float adimDer = 1.0f / adimDerece(s_ax);
    bool  a1Hepsi = a1 && !strcmp(a1, "hepsi"), a2Hepsi = a2 && !strcmp(a2, "hepsi");

    if (esit(k, "git", "g")) {
        if (!a1) { Serial.println("kullanim: git <derece>"); return; }
        s_salin[s_ax] = false; serbestUyar(s_ax);
        hedefVer(s_ax, s_pos[s_ax] + lroundf(x1 * adimDer));
        Serial.print("gidiyor: "); Serial.print(EKSEN[s_ax].ad); Serial.print(" "); Serial.print(x1, 2); Serial.println(" derece");
    } else if (esit(k, "konum", "k")) {
        if (!a1) { Serial.println("kullanim: konum <derece>"); return; }
        s_salin[s_ax] = false; serbestUyar(s_ax);
        hedefVer(s_ax, lroundf(x1 * adimDer));
        Serial.print("hedef konum: "); Serial.print(EKSEN[s_ax].ad); Serial.print(" "); Serial.print(x1, 2); Serial.println(" derece");
    } else if (!strcmp(k, "hepsi") || !strcmp(k, "hepsikonum")) {
        bool mutlak = (k[5] != 0);
        if (!a1) { Serial.println(mutlak ? "kullanim: hepsikonum <derece>   ornek: hepsikonum 0" : "kullanim: hepsi <derece>   ornek: hepsi 90"); return; }
        for (int i = 0; i < 3; i++) {
            s_salin[i] = false; serbestUyar(i);
            long adim = lroundf(x1 / adimDerece(i));
            hedefVer(i, mutlak ? adim : s_pos[i] + adim);
        }
        Serial.print(mutlak ? "hedef konum: uc eksen birden " : "gidiyor: uc eksen birden "); Serial.print(x1, 2); Serial.println(" derece");
        tavanNotu();
    } else if (!strcmp(k, "hepsisalin")) {
        for (int i = 0; i < 3; i++) {
            if (a1 && x1 != 0.0f) salinBaslat(i, s_pos[i], s_pos[i] + lroundf(x1 / adimDerece(i)));
            else salinBaslat(i, lroundf(EKSEN[i].lim_min / adimDerece(i)), lroundf(EKSEN[i].lim_max / adimDerece(i)));
        }
        Serial.print("salinim: uc eksen birden, ");
        if (a1 && x1 != 0.0f) { Serial.print(x1, 1); Serial.println(" derece gidip geliyor, dur ile biter"); }
        else {
            for (int i = 0; i < 3; i++) {
                Serial.print(EKSEN[i].ad); Serial.print(" "); Serial.print(EKSEN[i].lim_min, 0); Serial.print(".."); Serial.print(EKSEN[i].lim_max, 0);
                Serial.print(i < 2 ? ", " : "");
            }
            Serial.println(" arasinda gidip geliyor, dur ile biter");
        }
        tavanNotu();
    } else if (esit(k, "hiz", "h")) {
        if (!a1 || x1 <= 0.0f) { Serial.println("kullanim: hiz <derece/s>, sifirdan buyuk"); return; }
        s_hiz = x1; profilUygula();
        Serial.print("hiz = "); Serial.print(s_hiz, 1); Serial.println(" derece/s");
        tavanNotu();
    } else if (esit(k, "ivme", "i")) {
        if (!a1 || x1 <= 0.0f) { Serial.println("kullanim: ivme <derece/s2>, sifirdan buyuk"); return; }
        s_ivme = x1; profilUygula();
        Serial.print("ivme = "); Serial.print(s_ivme, 1); Serial.println(" derece/s2");
        tavanNotu();
    } else if (!strcmp(k, "tavan")) {
        if (!a1 || !a2 || x1 < 0.0f || atof(a2) < 0.0f) { Serial.println("kullanim: tavan <hiz> <ivme>   secili eksenin en yuksek degerleri, 0 = tavan yok"); return; }
        EKSEN[s_ax].hiz_tavan = x1; EKSEN[s_ax].ivme_tavan = (float)atof(a2);
        profilUygula(); durumYaz();
    } else if (esit(k, "dur", "d")) {
        yavasDur(); Serial.println("duruyor (yavaslayarak, butun eksenler)");
    } else if (esit(k, "acil", "a") || !strcmp(k, "x")) {
        acilDur(); Serial.println("ACIL DURUS (butun eksenler)");
    } else if (esit(k, "salin", "s")) {
        if (!a1 || x1 == 0.0f) { Serial.println("kullanim: salin <derece>   (uc eksen birden: hepsisalin)"); return; }
        salinBaslat(s_ax, s_pos[s_ax], s_pos[s_ax] + lroundf(x1 * adimDer));
        Serial.print("salinim: "); Serial.print(EKSEN[s_ax].ad); Serial.print(" "); Serial.print(x1, 1); Serial.println(" derece gidip geliyor, dur ile biter");
    } else if (!strcmp(k, "sifir")) {
        for (int i = 0; i < 3; i++)
            if ((a1Hepsi || i == s_ax) && (s_mesgul[i] || s_salin[i])) { Serial.println("once dur"); return; }
        noInterrupts();
        for (int i = 0; i < 3; i++) if (a1Hepsi || i == s_ax) { s_pos[i] = 0; s_hedef[i] = 0; }
        interrupts();
        Serial.println(a1Hepsi ? "uc eksenin konumu sifirlandi" : "konum sifirlandi");
    } else if (!strcmp(k, "en")) {
        if (!a1) { Serial.println("kullanim: en 1 | en 0 (secili eksen)   en 1 hepsi | en 0 hepsi"); return; }
        bool ac = (x1 != 0.0f);
        for (int i = 0; i < 3; i++) {
            if (!a2Hepsi && i != s_ax) continue;
            if (!ac) eksenAcilDur(i);
            s_enAx[i] = ac;
            digitalWriteFast(EKSEN[i].en, ac ? LOW : HIGH);
        }
        if (a2Hepsi) Serial.println(ac ? "uc surucu de acik" : "uc surucu de serbest");
        else         Serial.println(ac ? "surucu acik" : "surucu serbest (motor tutmuyor)");
    } else if (esit(k, "eksen", "e")) {
        int yeni = -1;
        for (int i = 0; i < 3; i++) if (a1 && !strcmp(a1, EKSEN[i].ad)) yeni = i;
        if (yeni < 0) { Serial.println("kullanim: eksen pan | tilt | ce"); return; }
        s_ax = yeni; durumYaz();
    } else if (!strcmp(k, "oran")) {
        if (s_mesgul[s_ax] || s_salin[s_ax]) { Serial.println("once dur"); return; }
        if (!a1 || x1 <= 0.0f) { Serial.println("kullanim: oran <x>"); return; }
        EKSEN[s_ax].oran = x1; profilUygula(); durumYaz();
    } else if (!strcmp(k, "mikro")) {
        int n = a1 ? atoi(a1) : 0;
        if (n < 1 || n > 256) { Serial.println("kullanim: mikro 8 | 16 | 32 | 64 (secili eksen)   mikro <n> hepsi"); return; }
        for (int i = 0; i < 3; i++)
            if ((a2Hepsi || i == s_ax) && (s_mesgul[i] || s_salin[i])) { Serial.println("once dur"); return; }
        for (int i = 0; i < 3; i++) if (a2Hepsi || i == s_ax) EKSEN[i].mikro = n;
        profilUygula(); durumYaz();
    } else if (!strcmp(k, "tel")) {
        // Multimetreyle tel sınaması: STEP ve DIR pinlerini sabit 3,3 V (1) ya da 0 V (0) yapar. Motor en fazla 1 mikroadım oynar.
        if (!a1) { Serial.println("kullanim: tel 1 | tel 0   secili eksenin STEP ve DIR pinlerini sabit 3,3 V / 0 V yapar (multimetreyle olcmek icin)"); return; }
        if (s_mesgul[s_ax] || s_salin[s_ax]) { Serial.println("once dur"); return; }
        bool yuksek = (x1 != 0.0f);
        digitalWriteFast(EKSEN[s_ax].step, yuksek ? HIGH : LOW);
        digitalWriteFast(EKSEN[s_ax].dir,  yuksek ? HIGH : LOW);
        Serial.print("tel testi: "); Serial.print(EKSEN[s_ax].ad);
        Serial.print(yuksek ? " STEP ve DIR = 3,3 V" : " STEP ve DIR = 0 V");
        Serial.print("   EN = "); Serial.println(s_enAx[s_ax] ? "0 V (surucu acik)" : "3,3 V (surucu serbest)");
        if (yuksek) Serial.println("  surucunun STEP ve DIR pinlerinde GND'ye gore 3,3 V olc; 0 V cikan tel kopuk ya da yanlis pinde. Bitince: tel 0");
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
    profilUygula();

    HardwareTimer *t = new HardwareTimer(TIM13);
    t->setOverflow((uint32_t)TICK_HZ, HERTZ_FORMAT);
    t->attachInterrupt(tik);
    t->setInterruptPriority(1, 0);
    t->resume();

    delay(1000);
    // GEÇİCİ (21 Eyl): CE sürücüsü duruşta 1,1 A çekiyor (Vref sonuna kadar açık olabilir). Akım düzelene kadar CE serbest
    // başlar; kullanmak için: eksen ce, en 1. Düzelince s_enAx[2] yeniden true yapılacak.
    s_enAx[2] = false;
    for (int i = 0; i < 3; i++) digitalWriteFast(EKSEN[i].en, s_enAx[i] ? LOW : HIGH);
    Serial.println("\nMOTOR TERMINALI hazir. Pan ve tilt suruculeri acik, CE serbest (acmak icin: eksen ce, en 1).");
    yardimYaz(); durumYaz();
}

void loop()
{
    // satır oku
    // Terminal programları araya ESC dizileri sokabiliyor (ör. ESC[?61;6;7;...c); bunlar komutu bozmasın diye atılır.
    static char buf[64]; static uint8_t n = 0; static uint8_t esc = 0;   // esc: 0 yok, 1 ESC geldi, 2 CSI içinde
    while (Serial.available() > 0) {
        uint8_t c = (uint8_t)Serial.read();
        if (esc == 1) { esc = (c == '[') ? 2 : 0; continue; }
        if (esc == 2) { if (c >= 0x40 && c <= 0x7E) esc = 0; continue; }
        if (c == 0x1B) { esc = 1; continue; }
        if (c == '\r' || c == '\n') { if (n) { buf[n] = 0; komut(buf); n = 0; } }
        else if (c == 8 || c == 127) { if (n) n--; }
        else if (c < 0x20 || c > 0x7E) { }                                // diğer kontrol ve ASCII dışı baytları yok say
        else if (n < sizeof buf - 1) buf[n++] = (char)c;
    }

    // varış bildirimi ve salınım (eksen başına)
    static bool onceki[3] = { false, false, false };
    static bool bildir = false;
    for (int i = 0; i < 3; i++) {
        bool m = s_mesgul[i];
        if (onceki[i] && !m) {
            if (s_salin[i]) s_salinBekle[i] = millis() + SALIN_BEKLE_MS;
            else bildir = true;
        }
        onceki[i] = m;
        if (s_salin[i] && !m && (int32_t)(millis() - s_salinBekle[i]) >= 0) {
            s_salinUcta[i] = !s_salinUcta[i];
            hedefVer(i, s_salinUcta[i] ? s_salinB[i] : s_salinA[i]);
            onceki[i] = true;
        }
    }
    digitalWriteFast(PIN_LED, herhangiMesgul() ? HIGH : LOW);
    if (bildir) {                                // salınım dışındaki bütün hareketler bitince tek satır
        bool suren = false;
        for (int i = 0; i < 3; i++) if (s_mesgul[i] && !s_salin[i]) suren = true;
        if (!suren) { Serial.print("vardi: "); konumlarYaz(); Serial.println(" derece"); bildir = false; }
    }
}
