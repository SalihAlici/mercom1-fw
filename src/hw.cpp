// Kart katmanı: pinler, timer'lar, hızlı GPIO. Çatıya dokunan tek yer burası
// ve sürücü dosyaları. Algoritma kodu bu dosyayı bilmez.
#include "hw.h"
#if !defined(ARDUINO_TEENSY41)          // STM32 (Nucleo) katmanı; Teensy için hw_teensy.cpp
#include "PeripheralPins.h"
#include <SPI.h>
#include <Wire.h>
#include <IWatchdog.h>

// 1 MHz: 5 V beslenen modülde 3,3 V'luk MOSI sınırda, yavaş saat kenar toleransını artırır
static const PinName k_csPins[3] = { PN_ENC_CS_PAN, PN_ENC_CS_TILT, PN_ENC_CS_CE };

// Tek pinli kurucu: RX yok, yarı çift yönlü. TMC2209 hattı.
static Uart s_tmcSer0(PN_TMC_UART_PAN);
static Uart s_tmcSer1(PN_TMC_UART_TILT);
static Uart s_tmcSer2(PN_TMC_UART_CE);
static Uart *const k_tmcSer[3] = { &s_tmcSer0, &s_tmcSer1, &s_tmcSer2 };
static const PinName k_tmcPin[3]  = { PN_TMC_UART_PAN, PN_TMC_UART_TILT, PN_TMC_UART_CE };
static const PinName k_tmcBase[3] = { PB_6, PE_8, PA_0 };     // GPIO okuma/yazma için ALT bitsiz adlar

static HardwareTimer *s_timStep = nullptr;  // TIM13, 20 kHz
static HardwareTimer *s_timCtrl = nullptr;  // TIM14, 1 kHz
static const PinName k_enPins[3] = { PN_EN_PAN, PN_EN_TILT, PN_EN_CE };
static int s_motorEnabled[3] = { 0, 0, 0 };

static const PinName k_stepPins[3] = { PN_STEP_PAN, PN_STEP_TILT, PN_STEP_CE };
static const PinName k_dirPins[3]  = { PN_DIR_PAN,  PN_DIR_TILT,  PN_DIR_CE  };

static void outLow(PinName pn)  { pinMode(pinNametoDigitalPin(pn), OUTPUT); digitalWriteFast(pn, LOW); }
static void outHigh(PinName pn) { pinMode(pinNametoDigitalPin(pn), OUTPUT); digitalWriteFast(pn, HIGH); }

extern "C" void board_init(void)
{
    // Motorlar açılışta pasif: EN yüksek
    for (int i = 0; i < 3; i++) outHigh(k_enPins[i]);
    for (int i = 0; i < 3; i++) { outLow(k_stepPins[i]); outLow(k_dirPins[i]); }

    // Enkoder CS hatları boşta yüksek
    outHigh(PN_ENC_CS_PAN);
    outHigh(PN_ENC_CS_TILT);
    outHigh(PN_ENC_CS_CE);

    outLow(PN_LASER_EN);
    outLow(PN_LED_HB);
    outLow(PN_LED_FAULT);

    // LoRa şeffaf mod: M0 = M1 = 0
    outLow(PN_LORA_M0);
    outLow(PN_LORA_M1);
    pinMode(pinNametoDigitalPin(PN_LORA_AUX), INPUT);

    // IMU RST: boşta yüksek (sensör çalışır), kablo yoksa zararsız
    outHigh(PN_IMU_RST);
}

extern "C" int board_i2c_scan_swapped(uint8_t *found, int cap)
{
    Wire.end();
    Wire.setSCL(PIN_IMU_SDA);   // kablolar ters takıldıysa bu eşleme doğru olur
    Wire.setSDA(PIN_IMU_SCL);
    Wire.begin();
    Wire.setClock(100000);
    int n = board_i2c_scan(found, cap);
    Wire.end();
    board_i2c_init();
    return n;
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

extern "C" int board_i2c_scan_slow(uint8_t *found, int cap)
{
    Wire.setClock(50000);
    int n = board_i2c_scan(found, cap);
    Wire.setClock(100000);
    return n;
}

// I2C çevre birimini baştan kur. STM32 I2C'si hat glitch'inde BUSY'de takılı kalabilir; o zaman 100 kHz'de
// hiçbir adres ACK vermez ama Wire.end()+begin() sonrası sensör hemen görünür (11 Eyl 5 dk testi, 141. s).
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
    // Kartın kendi pull-up'ları olmadan oku: hattı yüksekte tutan yalnız modülün dirençleri olur.
    // Modül beslenmiyorsa ya da kablo kopuksa hat düşük/yüzer okunur.
    // İçeriden ~40k aşağı çek: modülün 10k pull-up'ı bağlıysa hat yine yüksek okunur, kablo kopuksa düşük
    Wire.end();
    pinMode(PIN_IMU_SCL, INPUT_PULLDOWN);
    pinMode(PIN_IMU_SDA, INPUT_PULLDOWN);
    delay(2);
    *scl = digitalRead(PIN_IMU_SCL) ? 1 : 0;
    *sda = digitalRead(PIN_IMU_SDA) ? 1 : 0;
    *rst = digitalReadFast(PN_IMU_RST) ? 1 : 0;
    board_i2c_init();
}

extern "C" void board_watchdog_start(uint32_t ms)
{
    IWatchdog.begin(ms * 1000UL);
}

extern "C" void board_watchdog_feed(void)
{
    IWatchdog.reload();
}

extern "C" const char *board_reset_cause(void)
{
    const char *c = "bilinmiyor";
    if      (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDG1RST)) c = "WATCHDOG (ana dongu takildi)";
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDG1RST)) c = "pencere watchdog";
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST))   c = "BESLEME DUSTU (brown-out)";
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))   c = "yazilim reseti";
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST))   c = "reset pini / USB yeniden takildi";
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST))   c = "guc acilisi";
    __HAL_RCC_CLEAR_RESET_FLAGS();
    return c;
}

extern "C" void board_imu_rst_write(int high)
{
    digitalWriteFast(PN_IMU_RST, high ? HIGH : LOW);
}

extern "C" void board_imu_hw_reset(void)
{
    digitalWriteFast(PN_IMU_RST, LOW);
    delay(20);
    digitalWriteFast(PN_IMU_RST, HIGH);
    delay(400);   // BNO085 açılış süresi, veri sayfası en fazla ~300 ms
}

// Kart I2C okumasının ortasında resetlendiyse sensör SDA'yı düşükte tutabilir.
// Wire başlamadan önce SCL'i elle darbeleyerek sensörün aktarımı bitirmesi sağlanır.
extern "C" void board_i2c_bus_recover(void)
{
    pinMode(PIN_IMU_SDA, INPUT_PULLUP);
    pinMode(PIN_IMU_SCL, OUTPUT);
    for (int i = 0; i < 9 && digitalRead(PIN_IMU_SDA) == LOW; i++) {
        digitalWrite(PIN_IMU_SCL, LOW);  delayMicroseconds(5);
        digitalWrite(PIN_IMU_SCL, HIGH); delayMicroseconds(5);
    }
    // STOP koşulu: SCL yüksekken SDA düşükten yükseğe
    pinMode(PIN_IMU_SDA, OUTPUT);
    digitalWrite(PIN_IMU_SDA, LOW);  delayMicroseconds(5);
    digitalWrite(PIN_IMU_SCL, HIGH); delayMicroseconds(5);
    digitalWrite(PIN_IMU_SDA, HIGH); delayMicroseconds(5);
    pinMode(PIN_IMU_SDA, INPUT_PULLUP);
    pinMode(PIN_IMU_SCL, INPUT_PULLUP);
}

extern "C" void board_timers_start(void (*step_cb)(void), void (*ctrl_cb)(void))
{
    s_timStep = new HardwareTimer(TIM13);
    s_timStep->setOverflow(STEP_TICK_HZ, HERTZ_FORMAT);
    s_timStep->attachInterrupt(step_cb);
    s_timStep->setInterruptPriority(1, 0);
    s_timStep->resume();

    s_timCtrl = new HardwareTimer(TIM14);
    s_timCtrl->setOverflow(CTRL_HZ, HERTZ_FORMAT);
    s_timCtrl->attachInterrupt(ctrl_cb);
    s_timCtrl->setInterruptPriority(3, 0);
    s_timCtrl->resume();
}

// Enkoder SPI'ı YAZILIMLA sürülür (bit bit), mod 1, ~250 kHz, yumuşak kenarlı pinler.
// Sebep (11 Eyl): H7 donanım SPI'sı 470 kHz altına inemiyor ve uzun dupont kablolarda keskin
// kenar çınlayınca AS5048A saat darbelerini yanlış sayıyor. Yazılım SPI + düşük pin hızı 120/120 doğru okudu.
// Bir 16 bit çerçeve ~40 µs; üç enkoder x 2 çerçeve = ~240 µs, 1 kHz kesme içinde kabul edilebilir.
static inline void spiDelay(void)
{
    // ~1 µs @ 480 MHz (volatile döngü: iterasyon ~28 ns, 11 Eyl ölçümü)
    for (volatile int i = 0; i < 36; i++) { }
}

extern "C" void board_spi_init(void)
{
    pinMode(PIN_ENC_SCK,  OUTPUT); digitalWriteFast(PA_5, LOW);
    pinMode(PIN_ENC_MOSI, OUTPUT); digitalWriteFast(PN_ENC_MOSI, LOW);   // MOSI (hw.h dizilimi)
    pinMode(PIN_ENC_MISO, INPUT);
    // Yumuşak kenar: SCK (PA5), MOSI (PF3) ve üç CS (PB5, PD14, PD15)
    LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_5,  LL_GPIO_SPEED_FREQ_LOW);
    LL_GPIO_SetPinSpeed(GPIOF, LL_GPIO_PIN_3,  LL_GPIO_SPEED_FREQ_LOW);
    LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_5,  LL_GPIO_SPEED_FREQ_LOW);
    LL_GPIO_SetPinSpeed(GPIOD, LL_GPIO_PIN_14, LL_GPIO_SPEED_FREQ_LOW);
    LL_GPIO_SetPinSpeed(GPIOD, LL_GPIO_PIN_15, LL_GPIO_SPEED_FREQ_LOW);
}

extern "C" void board_spi_reinit(void)
{
    board_spi_init();
}

extern "C" uint16_t board_spi_xfer16(int cs_index, uint16_t tx)
{
    PinName cs = k_csPins[cs_index];
    uint16_t rx = 0;
    digitalWriteFast(cs, LOW);
    spiDelay();
    for (int i = 15; i >= 0; i--) {
        digitalWriteFast(PN_ENC_MOSI, (tx >> i) & 1);   // MOSI, yükselen kenardan önce hazır
        spiDelay();
        digitalWriteFast(PA_5, HIGH);                // SCK yüksel: çip çıkışını değiştirir
        spiDelay();
        digitalWriteFast(PA_5, LOW);                 // SCK düş: çip MOSI'yi örnekler, biz MISO'yu
        rx = (uint16_t)(rx << 1) | (digitalReadFast(PA_6) ? 1 : 0);
    }
    spiDelay();
    digitalWriteFast(cs, HIGH);
    spiDelay();                                      // CS yüksekte en az 350 ns
    return rx;
}

volatile uint8_t g_i2c_busy = 0;

// Kenar ayarı: STM32'de board_i2c_init zaten düşük hız kuruyor; Adafruit kütüphanesinin Wire.begin()'inden sonra da yeniden uygulanır
extern "C" void board_i2c_pads(int soft)
{
    if (soft == 0) return;
    LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_8, LL_GPIO_SPEED_FREQ_LOW);   // SCL
    LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_9, LL_GPIO_SPEED_FREQ_LOW);   // SDA
}

extern "C" void board_i2c_init(void)
{
    Wire.setSCL(PIN_IMU_SCL);
    Wire.setSDA(PIN_IMU_SDA);
    Wire.begin();
    // 100 kHz: 400 kHz'de uzun kablo + modülün 10k pull-up'ı ile hat tutmadı (10 Eyl testi).
    Wire.setClock(100000);
    // Yumuşak kenar: I2C'de de düşen kenar çınlıyor (11 Eyl: 100 kHz taramada yok, 50 kHz'de var)
    LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_8, LL_GPIO_SPEED_FREQ_LOW);   // SCL
    LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_9, LL_GPIO_SPEED_FREQ_LOW);   // SDA
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

// Pini, seçilen çevre biriminin KENDİ AF numarasıyla açık kollektör + pull-up yap.
// (Eski kod LPUART1'i açıp pini elle USART1 işlevine çeviriyordu; veri pine hiç çıkmıyordu, sürücüler bu yüzden sustu.)
static void tmcPinOpenDrain(int ch)
{
    uint32_t f = pinmap_function(k_tmcPin[ch], PinMap_UART_TX);
    pin_function(k_tmcPin[ch], STM_PIN_DATA(STM_MODE_AF_OD, GPIO_PULLUP, STM_PIN_AFNUM(f)));
}

extern "C" void board_tmc_uart_begin(uint32_t baud)
{
    for (int ch = 0; ch < 3; ch++) {
        k_tmcSer[ch]->begin(baud);
        tmcPinOpenDrain(ch);            // tek hat: sürücü cevap verirken hattı biz sürmeyiz
    }
}

extern "C" void board_tmc_uart_send(int ch, const uint8_t *buf, size_t n)
{
    if (ch < 0 || ch > 2) return;
    k_tmcSer[ch]->write(buf, n);
    k_tmcSer[ch]->enableHalfDuplexRx();   // TX bitene kadar bekler, sonra alıcıyı açar
}

extern "C" int board_tmc_uart_read_byte(int ch)
{
    return (ch < 0 || ch > 2) ? -1 : k_tmcSer[ch]->read();
}

extern "C" void board_tmc_uart_flush_rx(int ch)
{
    if (ch < 0 || ch > 2) return;
    while (k_tmcSer[ch]->read() >= 0) { }
}

extern "C" int board_tmc_uart_line_level(int ch)
{
    if (ch < 0 || ch > 2) return 0;
    // Pini geçici olarak giriş yap, seviyeyi oku, sonra UART işlevine geri ver
    pinMode(pinNametoDigitalPin(k_tmcBase[ch]), INPUT_PULLUP);
    delayMicroseconds(50);
    int lvl = digitalRead(pinNametoDigitalPin(k_tmcBase[ch]));
    tmcPinOpenDrain(ch);
    return lvl;
}

// Telsiz verici sınaması: 0x00 gönderirken pinin kendi giriş yazmacında düşük darbe görülmeli.
// Görülmüyorsa çevre birimi pine bağlı değildir (yanlış AF / yanlış çevre birimi).
extern "C" int board_tmc_uart_tx_check(int ch)
{
    if (ch < 0 || ch > 2) return 0;
    int seen = 0;
    uint8_t z = 0x00;
    k_tmcSer[ch]->write(&z, 1);
    uint32_t t0 = micros();
    while (micros() - t0 < 600) { if (!digitalReadFast(k_tmcBase[ch])) { seen = 1; break; } }
    k_tmcSer[ch]->enableHalfDuplexRx();
    return seen;
}

// Telli öz sınama: A0 (PA3) pini seçilen hattın pinine telle bağlıyken.
// 1) A0'dan yazılımsal UART ile 0x55 gönder, alıcı okumalı.  2) UART'tan 0x00 gönder, A0'da düşük darbe görülmeli.
extern "C" void board_tmc_uart_selftest(int ch, int *rx_ok, int *rx_byte, int *tx_ok)
{
    *rx_ok = 0; *rx_byte = -1; *tx_ok = 0;
    if (ch < 0 || ch > 2) return;
    const uint32_t bit_us = 1000000UL / 115200UL;   // ~8.7 us
    int a0 = pinNametoDigitalPin(PA_3);
    pinMode(a0, OUTPUT); digitalWrite(a0, HIGH); delay(2);
    board_tmc_uart_flush_rx(ch);
    k_tmcSer[ch]->enableHalfDuplexRx();
    uint8_t b = 0x55;
    noInterrupts();
    digitalWriteFast(PA_3, LOW); delayMicroseconds(bit_us);            // start
    for (int i = 0; i < 8; i++) { digitalWriteFast(PA_3, (b >> i) & 1); delayMicroseconds(bit_us); }
    digitalWriteFast(PA_3, HIGH); delayMicroseconds(bit_us * 2);       // stop
    interrupts();
    delay(2);
    int c = k_tmcSer[ch]->read();
    *rx_byte = c; *rx_ok = (c == 0x55);
    pinMode(a0, INPUT_PULLUP); delay(1);
    int seen_low = 0;
    uint8_t z = 0x00;
    k_tmcSer[ch]->write(&z, 1);
    uint32_t t0 = micros();
    while (micros() - t0 < 600) { if (!digitalReadFast(PA_3)) { seen_low = 1; break; } }
    k_tmcSer[ch]->enableHalfDuplexRx();
    *tx_ok = seen_low;
    pinMode(a0, INPUT);
}

extern "C" void board_delay_ms(uint32_t ms) { delay(ms); }
extern "C" void board_reset(void) { NVIC_SystemReset(); }

extern "C" void board_step_write(int axis, int high)
{
    digitalWriteFast(k_stepPins[axis], high ? HIGH : LOW);
}

extern "C" void board_dir_write(int axis, int positive)
{
    digitalWriteFast(k_dirPins[axis], positive ? HIGH : LOW);
}

extern "C" void board_motor_enable(int axis, int on)
{
    if (axis < 0 || axis > 2) return;
    s_motorEnabled[axis] = on ? 1 : 0;
    digitalWriteFast(k_enPins[axis], on ? LOW : HIGH);  // aktif düşük
}

extern "C" void board_motors_enable(int on)
{
    for (int i = 0; i < 3; i++) board_motor_enable(i, on);
}

extern "C" int board_motor_enabled(int axis) { return (axis >= 0 && axis <= 2) ? s_motorEnabled[axis] : 0; }
extern "C" int board_motors_enabled(void) { return s_motorEnabled[0] || s_motorEnabled[1] || s_motorEnabled[2]; }

extern "C" void board_laser(int on)     { digitalWriteFast(PN_LASER_EN, on ? HIGH : LOW); }
extern "C" void board_led_hb(int on)    { digitalWriteFast(PN_LED_HB, on ? HIGH : LOW); }
extern "C" void board_led_fault(int on) { digitalWriteFast(PN_LED_FAULT, on ? HIGH : LOW); }

extern "C" uint32_t board_micros(void) { return micros(); }
extern "C" uint32_t board_millis(void) { return millis(); }
extern "C" int board_nv_read(uint16_t addr, void *buf, uint16_t n) { (void)addr; (void)buf; (void)n; return -1; }
extern "C" int board_nv_write(uint16_t addr, const void *buf, uint16_t n) { (void)addr; (void)buf; (void)n; return -1; }

#endif  // !ARDUINO_TEENSY41
