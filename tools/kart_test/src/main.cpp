// KART YÜK VE ARIZA SINAMASI (STM32H753, Nucleo-144)
//  - Yük: işlemci sürekli %100 çalışır: 256 KB RAM yaz/oku sınaması, flash CRC32 (96 KB), kayan nokta hesap doğrulaması.
//  - Arıza okuma: açılışta sıfırlama sebebi (RCC_RSR), önceki çalışmadaki HardFault kaydı (DTCM'de saklanır),
//    PVD ile besleme çöküşü sayacı (2,85 V altı), bağımsız bekçi köpek (IWDG 4 s: kilitlenirse kart resetlenir ve sebep yazılır).
//  - Her saniye: yanan LED, VDDA (iç referansla), çekirdek sıcaklığı, en düşük/en yüksek değerler, hata sayaçları.
//  - LED'ler yeşil → sarı → kırmızı sırayla 500 ms; besleme aralık dışında ya da hata varsa üçü birden yanar.
// Dış pinlere dokunmaz. tools/seri_izle.bat ile canlı izlenir. SWD ile yüklenir.
#include <Arduino.h>
#include <IWatchdog.h>
#include <math.h>
#include <string.h>

#define VREFINT_CAL   (*(uint16_t *)0x1FF1E860UL)
#define TS_CAL1       (*(uint16_t *)0x1FF1E820UL)
#define TS_CAL2       (*(uint16_t *)0x1FF1E840UL)

struct Led { const char *ad; PinName pin; };
static const Led LEDLER[3] = { { "YESIL", PB_0 }, { "SARI", PE_1 }, { "KIRMIZI", PB_14 } };

// ---- HardFault kaydı: DTCM (0x20000000) açılışta silinmez, reset sonrası okunur ----
struct FaultInfo { uint32_t magic, cfsr, hfsr, bfar, mmfar, pc, lr, uptime_ms; };
static FaultInfo *const FAULT = (FaultInfo *)0x20000000UL;
#define FAULT_MAGIC 0xFA17CAFEUL

extern "C" void hardfault_c(uint32_t *sp)
{
    FAULT->magic = FAULT_MAGIC; FAULT->cfsr = SCB->CFSR; FAULT->hfsr = SCB->HFSR;
    FAULT->bfar = SCB->BFAR; FAULT->mmfar = SCB->MMFAR; FAULT->pc = sp[6]; FAULT->lr = sp[5]; FAULT->uptime_ms = HAL_GetTick();
    for (auto &l : LEDLER) digitalWriteFast(l.pin, HIGH);
    while (1) { }                                   // IWDG 4 s içinde resetler, açılışta kayıt yazılır
}
extern "C" __attribute__((naked)) void HardFault_Handler(void)
{
    __asm volatile("tst lr, #4\n ite eq\n mrseq r0, msp\n mrsne r0, psp\n b hardfault_c\n");
}

// ---- PVD: besleme 2,85 V altına inerse sayaç ----
static volatile uint32_t pvdSayac = 0;
extern "C" void PVD_AVD_IRQHandler(void) { HAL_PWR_PVD_IRQHandler(); }
extern "C" void HAL_PWR_PVDCallback(void) { pvdSayac++; }

// ---- yük sınamaları ----
static uint32_t ramBuf[65536];                     // 256 KB, AXI SRAM
static uint32_t ramTur = 0, ramHata = 0, flashHata = 0, hesapHata = 0, flashCrcRef = 0;
static double hesapRef = 0.0;

static uint32_t crc32(const uint8_t *p, uint32_t n)
{
    uint32_t c = 0xFFFFFFFFUL;
    for (uint32_t i = 0; i < n; i++) {
        c ^= p[i];
        for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320UL & (0UL - (c & 1)));
    }
    return ~c;
}
static double hesap(void)
{
    double s = 0.0;
    for (int k = 1; k <= 3000; k++) s += sin(k * 0.001) * cos(k * 0.002) + sqrt((double)k) * 1e-4;
    return s;
}
static void ramSinama(void)
{
    static const uint32_t desen[3] = { 0x55555555UL, 0xAAAAAAAAUL, 0x00000000UL };
    uint32_t d = desen[ramTur % 3];
    for (uint32_t i = 0; i < 65536; i++) ramBuf[i] = d ^ (i * 2654435761UL);
    for (uint32_t i = 0; i < 65536; i++) if (ramBuf[i] != (d ^ (i * 2654435761UL))) ramHata++;
    ramTur++;
}

static uint32_t vddaMv(void)
{
    uint32_t acc = 0; for (int i = 0; i < 16; i++) acc += analogRead(AVREF);
    uint32_t vref = acc / 16; return vref ? (3300UL * VREFINT_CAL) / vref : 0;
}
static int32_t cekirdekC10(uint32_t vdda_mV)
{
    uint32_t acc = 0; for (int i = 0; i < 16; i++) acc += analogRead(ATEMP);
    int32_t tdata = (int32_t)((uint64_t)(acc / 16) * vdda_mV / 3300UL);
    return 300 + (int32_t)((int64_t)(tdata - (int32_t)TS_CAL1) * 800 / ((int32_t)TS_CAL2 - (int32_t)TS_CAL1));
}
static void volt(uint32_t mv) { Serial.print(mv / 1000); Serial.print('.'); if ((mv / 10) % 100 < 10) Serial.print('0'); Serial.print((mv / 10) % 100); }

void setup()
{
    Serial.begin(115200);
    analogReadResolution(16);
    for (auto &l : LEDLER) { pinMode(pinNametoDigitalPin(l.pin), OUTPUT); digitalWriteFast(l.pin, LOW); }
    delay(300);
    Serial.println("\n=== KART YUK VE ARIZA SINAMASI ===");

    uint32_t rsr = RCC->RSR;
    Serial.print("sifirlama sebebi:");
    if (rsr & RCC_RSR_PORRSTF)   Serial.print(" guc-acilis(POR)");
    if (rsr & RCC_RSR_BORRSTF)   Serial.print(" besleme-cokmesi(BOR)");
    if (rsr & RCC_RSR_PINRSTF)   Serial.print(" reset-pini");
    if (rsr & RCC_RSR_IWDG1RSTF) Serial.print(" BEKCI-KOPEK(kilitlenme!)");
    if (rsr & RCC_RSR_WWDG1RSTF) Serial.print(" pencere-bekci");
    if (rsr & RCC_RSR_SFTRSTF)   Serial.print(" yazilim");
    if (rsr & RCC_RSR_LPWRRSTF)  Serial.print(" dusuk-guc");
    Serial.println();
    RCC->RSR |= RCC_RSR_RMVF;

    if (FAULT->magic == FAULT_MAGIC) {
        Serial.print("ONCEKI CALISMADA HARDFAULT: CFSR=0x"); Serial.print(FAULT->cfsr, HEX);
        Serial.print(" HFSR=0x"); Serial.print(FAULT->hfsr, HEX);
        Serial.print(" BFAR=0x"); Serial.print(FAULT->bfar, HEX);
        Serial.print(" PC=0x"); Serial.print(FAULT->pc, HEX);
        Serial.print(" LR=0x"); Serial.print(FAULT->lr, HEX);
        Serial.print(" acik kalma="); Serial.print(FAULT->uptime_ms / 1000); Serial.println(" s");
        FAULT->magic = 0;
    } else {
        Serial.println("onceki calismada HardFault kaydi yok");
    }

    // PVD 2,85 V
    PWR_PVDTypeDef pvd; pvd.PVDLevel = PWR_PVDLEVEL_6; pvd.Mode = PWR_PVD_MODE_IT_RISING_FALLING;
    HAL_PWR_ConfigPVD(&pvd); HAL_PWR_EnablePVD();
    HAL_NVIC_SetPriority(PVD_AVD_IRQn, 2, 0); HAL_NVIC_EnableIRQ(PVD_AVD_IRQn);

    flashCrcRef = crc32((const uint8_t *)0x08000000UL, 96 * 1024);
    hesapRef = hesap();
    Serial.print("flash CRC32 referansi (96 KB): 0x"); Serial.println(flashCrcRef, HEX);
    Serial.println("RAM sinamasi 256 KB, hesap dogrulamasi, PVD 2.85 V, IWDG 4 s: basladi");
    IWatchdog.begin(4000000);
}

void loop()
{
    static uint32_t sonYazma = 0, sonLed = 0, n = 0;
    static int hangi = 0;
    static uint32_t vmin = 9999, vmax = 0; static int32_t tmin = 9999, tmax = -999;

    // yük: RAM, flash, hesap
    ramSinama();
    if (crc32((const uint8_t *)0x08000000UL, 96 * 1024) != flashCrcRef) flashHata++;
    double h = hesap();
    if (memcmp(&hesapRef, &h, sizeof(double)) != 0) hesapHata++;
    IWatchdog.reload();

    uint32_t v = vddaMv(); int32_t t = cekirdekC10(v);
    if (v < vmin) vmin = v; if (v > vmax) vmax = v; if (t < tmin) tmin = t; if (t > tmax) tmax = t;
    bool ok = v >= 3000 && v <= 3600 && ramHata == 0 && flashHata == 0 && hesapHata == 0;

    if (millis() - sonLed >= 500) { sonLed = millis(); hangi = (hangi + 1) % 3; }
    for (int i = 0; i < 3; i++) digitalWriteFast(LEDLER[i].pin, (ok ? i == hangi : true) ? HIGH : LOW);

    if (millis() - sonYazma >= 1000) {
        sonYazma = millis(); n++;
        Serial.print(n); Serial.print(" s  LED: "); Serial.print(ok ? LEDLER[hangi].ad : "UCU BIRDEN");
        Serial.print("  VDDA "); volt(v); Serial.print(" V (min "); volt(vmin); Serial.print(")");
        Serial.print("  cekirdek "); Serial.print(t / 10); Serial.print('.'); Serial.print(t % 10);
        Serial.print(" C (max "); Serial.print(tmax / 10); Serial.print(")");
        Serial.print("  RAM "); Serial.print(ramTur); Serial.print(" tur/"); Serial.print(ramHata); Serial.print(" hata");
        Serial.print("  flash "); Serial.print(flashHata ? "HATA" : "ok");
        Serial.print("  hesap "); Serial.print(hesapHata); Serial.print(" hata");
        Serial.print("  PVD "); Serial.print(pvdSayac);
        Serial.println(ok ? "" : "  UYARI!");
    }
}
