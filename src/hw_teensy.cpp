// Kart katmanı, TEENSY 4.1 sürümü (plan B, 29 Eyl 2026). hw.h'deki sözleşmenin aynısı; algoritma kodu bu dosyayı bilmez.
// Nucleo sürümü hw.cpp'dedir. Pin planı: MERCOM1_OZELLIKLER.md "Plan B: Teensy 4.1".
#if defined(ARDUINO_TEENSY41)
#include "hw.h"
#include <Wire.h>
#include <IntervalTimer.h>
#include <avr/eeprom.h>          // E2END (kalıcı bellek boyu)

static IntervalTimer s_timStep;                 // 20 kHz adım üreteci
static IntervalTimer s_timCtrl;                 // 1 kHz kontrol döngüsü
static int s_motorEnabled[3] = { 0, 0, 0 };

// 29 Eyl devir notu (başka bilgisayarda bu donanımla ÇALIŞAN yöntem) esas alındı:
//  - enkoder yazılım SPI: yarım saat 5 µs (<= 100 kHz), MISO INPUT_PULLUP, CS 8/9/12, pad DSE=2 (SPEED/SRE temiz)
//  - STEP/DIR: çekirdek varsayılan pad'i; darbe en az 20 µs (bizde 50 µs, 20 kHz tik)
//  - çıkışlar önce latch'e yazılır, sonra OUTPUT yapılır (açılışta EN/CS'de kısa düşük darbe olmasın)
// 29 Eyl ölçümü (bu kart, bu kablolar): pin sınamasında çekirdek varsayılanı DSE(7) ile üç enkoder okundu;
// DSE(2) ile ana firmware 0x0000 okudu. Bu yüzden pad ayarına dokunulmuyor.
static void padSoft(uint8_t pin)
{
    (void)pin;
}
static void outLow(uint8_t pin)  { digitalWrite(pin, LOW);  pinMode(pin, OUTPUT); digitalWrite(pin, LOW); }
static void outHigh(uint8_t pin) { digitalWrite(pin, HIGH); pinMode(pin, OUTPUT); digitalWrite(pin, HIGH); }

extern "C" void board_init(void)
{
    // Motorlar açılışta pasif: EN yüksek
    outHigh(PN_EN_PAN); outHigh(PN_EN_TILT); outHigh(PN_EN_CE);
    outLow(PN_STEP_PAN); outLow(PN_STEP_TILT); outLow(PN_STEP_CE);
    outLow(PN_DIR_PAN);  outLow(PN_DIR_TILT);  outLow(PN_DIR_CE);

    // Enkoder CS hatları boşta yüksek; eski CE CS (10) boş, giriş
    outHigh(PN_ENC_CS_PAN); outHigh(PN_ENC_CS_TILT); outHigh(PN_ENC_CS_CE);
    pinMode(PN_ENC_CS_OLD, INPUT);

    outLow(PN_LASER_EN);
    outLow(PN_LED_HB);
    outLow(PN_LED_FAULT);

    // LoRa şeffaf mod: M0 = M1 = 0
    outLow(PN_LORA_M0); outLow(PN_LORA_M1);
    pinMode(PN_LORA_AUX, INPUT);

    outHigh(PN_IMU_RST);           // bağlı değil, zararsız
}

extern "C" void board_timers_start(void (*step_cb)(void), void (*ctrl_cb)(void))
{
    s_timStep.priority(16);        // en yüksek: adım darbeleri hiçbir şeyden etkilenmesin (USB/I2C 64+)
    s_timCtrl.priority(48);
    s_timStep.begin(step_cb, 1000000UL / STEP_TICK_HZ);   // 50 µs
    s_timCtrl.begin(ctrl_cb, 1000000UL / CTRL_HZ);        // 1000 µs
}

// ---- adım / yön: sabit pinle tek register yazımı ----
extern "C" void board_step_write(int axis, int high)
{
    switch (axis) {
    case 0: digitalWriteFast(PN_STEP_PAN,  high ? HIGH : LOW); break;
    case 1: digitalWriteFast(PN_STEP_TILT, high ? HIGH : LOW); break;
    case 2: digitalWriteFast(PN_STEP_CE,   high ? HIGH : LOW); break;
    }
}
extern "C" void board_dir_write(int axis, int positive)
{
    switch (axis) {
    case 0: digitalWriteFast(PN_DIR_PAN,  positive ? HIGH : LOW); break;
    case 1: digitalWriteFast(PN_DIR_TILT, positive ? HIGH : LOW); break;
    case 2: digitalWriteFast(PN_DIR_CE,   positive ? HIGH : LOW); break;
    }
}

// ---- enkoder yazılım SPI: mod 1 (CPOL 0, CPHA 1), 16 bit, MSB önce, yarım saat 5 µs ----
static inline void spiDelay(void) { delayMicroseconds(5); }
static inline void csWrite(int idx, int high)
{
    switch (idx) {
    case 0: digitalWriteFast(PN_ENC_CS_PAN,  high); break;
    case 1: digitalWriteFast(PN_ENC_CS_TILT, high); break;
    case 2: digitalWriteFast(PN_ENC_CS_CE,   high); break;
    }
}

extern "C" void board_spi_init(void)
{
    pinMode(PIN_ENC_SCK,  OUTPUT); digitalWriteFast(PIN_ENC_SCK,  LOW);
    pinMode(PIN_ENC_MOSI, OUTPUT); digitalWriteFast(PIN_ENC_MOSI, LOW);
    pinMode(PIN_ENC_MISO, INPUT_PULLUP);                 // kopuk hatta 0xFFFF okunur; asla OUTPUT yapılmaz
    padSoft(PIN_ENC_SCK); padSoft(PIN_ENC_MOSI);
    padSoft(PN_ENC_CS_PAN); padSoft(PN_ENC_CS_TILT); padSoft(PN_ENC_CS_CE);
}

extern "C" void board_spi_reinit(void) { board_spi_init(); }

extern "C" uint16_t board_spi_xfer16(int cs_index, uint16_t tx)
{
    uint16_t rx = 0;
    csWrite(0, HIGH); csWrite(1, HIGH); csWrite(2, HIGH);   // önce hepsi pasif
    digitalWriteFast(PIN_ENC_SCK, LOW);
    digitalWriteFast(PIN_ENC_MOSI, LOW);
    spiDelay();
    csWrite(cs_index, LOW);
    spiDelay();
    for (int i = 15; i >= 0; i--) {
        digitalWriteFast(PIN_ENC_SCK, HIGH);             // yükselen kenar
        digitalWriteFast(PIN_ENC_MOSI, (tx >> i) & 1);   // MOSI kenarın hemen ardından, düşen kenara kadar sabit
        spiDelay();
        digitalWriteFast(PIN_ENC_SCK, LOW);              // düşen kenar: çip MOSI'yi örnekler
        rx = (uint16_t)(rx << 1) | (digitalReadFast(PIN_ENC_MISO) ? 1 : 0);
        spiDelay();
    }
    csWrite(cs_index, HIGH);
    digitalWriteFast(PIN_ENC_MOSI, LOW);
    spiDelay();
    return rx;
}

// ---- IMU I2C: Wire, 18/19, 100 kHz ----
// Kenar ayarı (29 Eyl): Teensy Wire pinleri hızlı kenar (SRE) + DSE(4) ile kurar. Uzun kabloda düşen kenar çınlıyor;
// STM32'de 11 Eyl'de aynı sorun pinler düşük hıza alınarak çözülmüştü (sonra 5 dk'da 0 sensör reseti). Burada karşılığı:
// yavaş kenar (SRE yok), en zayıf sürüş DSE(1), SPEED(0). Açık-drenaj düşük seviye için yeterli (~0,5 mA çekilir).
// Adafruit kütüphanesi her başlatmada Wire.begin() çağırıp pinleri varsayılana döndürdüğü için bno085_init sonrası yeniden uygulanır.
static int s_i2cSoft = 0;                            // denendi, fayda yok (29 Eyl); varsayılan kütüphane ayarı
volatile uint8_t g_i2c_busy = 0;
extern "C" void board_i2c_pads(int soft)
{
    if (soft >= 0) s_i2cSoft = soft ? 1 : 0;
    uint32_t pad = s_i2cSoft
        ? (IOMUXC_PAD_ODE | IOMUXC_PAD_DSE(1) | IOMUXC_PAD_SPEED(0) | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(3) | IOMUXC_PAD_HYS)
        : (IOMUXC_PAD_ODE | IOMUXC_PAD_SRE | IOMUXC_PAD_DSE(4) | IOMUXC_PAD_SPEED(1) | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(3) | IOMUXC_PAD_HYS);
    *(portControlRegister(PIN_IMU_SDA)) = pad;
    *(portControlRegister(PIN_IMU_SCL)) = pad;
}

extern "C" void board_i2c_init(void)
{
    Wire.begin();
    Wire.setClock(100000);
    board_i2c_pads(-1);
}

extern "C" int board_i2c_scan(uint8_t *found, int cap)
{
    int n = 0;
    for (uint8_t a = 0x08; a <= 0x77 && n < cap; a++) {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0) found[n++] = a;
    }
    return n;
}

extern "C" int board_i2c_scan_swapped(uint8_t *found, int cap)
{
    (void)found; (void)cap;
    return 0;                                            // Teensy Wire'da 18/19 yer değiştirilemez
}

extern "C" int board_i2c_scan_slow(uint8_t *found, int cap)
{
    Wire.setClock(50000);
    int n = board_i2c_scan(found, cap);
    Wire.setClock(100000);
    return n;
}

extern "C" void board_i2c_reinit(void)
{
    Wire.end();
    pinMode(PIN_IMU_SCL, INPUT);
    pinMode(PIN_IMU_SDA, INPUT);
    delay(2);
    board_i2c_init();
}

extern "C" int board_i2c_probe(uint8_t addr)
{
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0 ? 1 : 0;
}

extern "C" void board_i2c_levels(int *scl, int *sda, int *rst)
{
    Wire.end();
    pinMode(PIN_IMU_SCL, INPUT_PULLDOWN);
    pinMode(PIN_IMU_SDA, INPUT_PULLDOWN);
    delay(2);
    *scl = digitalRead(PIN_IMU_SCL) ? 1 : 0;
    *sda = digitalRead(PIN_IMU_SDA) ? 1 : 0;
    *rst = digitalRead(PN_IMU_RST) ? 1 : 0;
    board_i2c_init();
}

extern "C" void board_i2c_activity(int *scl_edges, int *sda_edges)
{
    Wire.end();
    pinMode(PIN_IMU_SCL, INPUT);
    pinMode(PIN_IMU_SDA, INPUT);
    delay(1);
    int ps = digitalRead(PIN_IMU_SCL), pd = digitalRead(PIN_IMU_SDA);
    int es = 0, ed = 0;
    uint32_t t0 = micros();
    while (micros() - t0 < 20000) {
        int s = digitalRead(PIN_IMU_SCL), d = digitalRead(PIN_IMU_SDA);
        if (s != ps) { es++; ps = s; }
        if (d != pd) { ed++; pd = d; }
    }
    *scl_edges = es; *sda_edges = ed;
    board_i2c_init();
}

extern "C" void board_i2c_bus_recover(void)
{
    pinMode(PIN_IMU_SDA, INPUT_PULLUP);
    pinMode(PIN_IMU_SCL, OUTPUT);
    for (int i = 0; i < 9 && digitalRead(PIN_IMU_SDA) == LOW; i++) {
        digitalWrite(PIN_IMU_SCL, LOW);  delayMicroseconds(5);
        digitalWrite(PIN_IMU_SCL, HIGH); delayMicroseconds(5);
    }
    pinMode(PIN_IMU_SDA, OUTPUT);
    digitalWrite(PIN_IMU_SDA, LOW);  delayMicroseconds(5);
    digitalWrite(PIN_IMU_SCL, HIGH); delayMicroseconds(5);
    digitalWrite(PIN_IMU_SDA, HIGH); delayMicroseconds(5);
    pinMode(PIN_IMU_SDA, INPUT_PULLUP);
    pinMode(PIN_IMU_SCL, INPUT_PULLUP);
}

extern "C" void board_imu_rst_write(int high) { digitalWrite(PN_IMU_RST, high ? HIGH : LOW); }
extern "C" void board_imu_hw_reset(void)
{
    digitalWrite(PN_IMU_RST, LOW);  delay(20);
    digitalWrite(PN_IMU_RST, HIGH); delay(400);
}

// ---- bekçi köpek: RTWDOG (WDOG3). Ana döngü beslemezse kart resetlenir ----
static bool s_wdogOn = false;
extern "C" void board_watchdog_start(uint32_t ms)
{
    // RTWDOG: 32 kHz LPO saati, 256 ön bölücü -> 128 Hz sayaç; TOVAL = ms * 128 / 1000
    uint32_t toval = (ms * 128UL) / 1000UL;
    if (toval < 2) toval = 2;
    if (toval > 0xFFFF) toval = 0xFFFF;
    WDOG3_CNT = 0xD928C520;                              // kilidi aç
    WDOG3_TOVAL = toval;
    WDOG3_CS = WDOG_CS_UPDATE | WDOG_CS_CLK(1) | WDOG_CS_PRES | WDOG_CS_EN | WDOG_CS_CMD32EN;
    s_wdogOn = true;
}
extern "C" void board_watchdog_feed(void)
{
    if (!s_wdogOn) return;
    __disable_irq();
    WDOG3_CNT = 0xB480A602;                              // besleme sözcüğü
    __enable_irq();
}

extern "C" const char *board_reset_cause(void)
{
    uint32_t r = SRC_SRSR;
    SRC_SRSR = r;                                        // bayraklari temizle (yazarak silinir)
    if (r & (1u << 4) || r & (1u << 7)) return "WATCHDOG (ana dongu takildi)";
    if (r & (1u << 3)) return "yazilim / USB yeniden programlama";
    if (r & (1u << 0)) return "guc acilisi";
    if (r & (1u << 8)) return "sicaklik korumasi";
    return "bilinmiyor";
}

// ---- TMC2209 UART: Teensy'de bağlı değil ----
extern "C" void board_tmc_uart_begin(uint32_t baud) { (void)baud; }
extern "C" void board_tmc_uart_send(int ch, const uint8_t *buf, size_t n) { (void)ch; (void)buf; (void)n; }
extern "C" int  board_tmc_uart_read_byte(int ch) { (void)ch; return -1; }
extern "C" void board_tmc_uart_flush_rx(int ch) { (void)ch; }
extern "C" int  board_tmc_uart_line_level(int ch) { (void)ch; return 1; }
extern "C" int  board_tmc_uart_tx_check(int ch) { (void)ch; return 0; }
extern "C" void board_tmc_uart_selftest(int ch, int *rx_ok, int *rx_byte, int *tx_ok)
{
    (void)ch; *rx_ok = 0; *rx_byte = -1; *tx_ok = 0;
}

extern "C" void board_delay_ms(uint32_t ms) { delay(ms); }
extern "C" void board_reset(void) { SCB_AIRCR = 0x05FA0004; while (1) { } }

extern "C" void board_motor_enable(int axis, int on)
{
    if (axis < 0 || axis > 2) return;
    s_motorEnabled[axis] = on ? 1 : 0;
    int lvl = on ? LOW : HIGH;                           // EN aktif düşük
    switch (axis) {
    case 0: digitalWriteFast(PN_EN_PAN,  lvl); break;
    case 1: digitalWriteFast(PN_EN_TILT, lvl); break;
    case 2: digitalWriteFast(PN_EN_CE,   lvl); break;
    }
}
extern "C" void board_motors_enable(int on) { for (int i = 0; i < 3; i++) board_motor_enable(i, on); }
extern "C" int  board_motor_enabled(int axis) { return (axis >= 0 && axis <= 2) ? s_motorEnabled[axis] : 0; }
extern "C" int  board_motors_enabled(void) { return s_motorEnabled[0] || s_motorEnabled[1] || s_motorEnabled[2]; }

extern "C" void board_laser(int on)     { digitalWriteFast(PN_LASER_EN, on ? HIGH : LOW); }
extern "C" void board_led_hb(int on)    { digitalWriteFast(PN_LED_HB, on ? HIGH : LOW); }
extern "C" void board_led_fault(int on) { digitalWriteFast(PN_LED_FAULT, on ? HIGH : LOW); }

extern "C" uint32_t board_micros(void) { return micros(); }
extern "C" uint32_t board_millis(void) { return millis(); }

// ---- kalıcı bellek: çekirdeğin EEPROM öykünmesi (flash, E2END+1 = 4284 bayt) ----
extern "C" int board_nv_read(uint16_t addr, void *buf, uint16_t n)
{
    if ((uint32_t)addr + n > (uint32_t)E2END + 1) return -1;
    eeprom_read_block(buf, (const void *)(uintptr_t)addr, n);
    return 0;
}
extern "C" int board_nv_write(uint16_t addr, const void *buf, uint16_t n)
{
    if ((uint32_t)addr + n > (uint32_t)E2END + 1) return -1;
    const uint8_t *b = (const uint8_t *)buf;
    for (uint16_t i = 0; i < n; i++) {
        eeprom_write_byte((uint8_t *)(uintptr_t)(addr + i), b[i]);   // değişmeyen bayt atlanır (çekirdek karşılaştırır)
        if ((i & 15) == 15) board_watchdog_feed();                   // sektör silinirse tek yazma ~50 ms sürebilir
    }
    return 0;
}

#endif  // ARDUINO_TEENSY41
