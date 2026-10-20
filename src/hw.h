#pragma once
// MERCOM-1 pin planı, NUCLEO-H753ZI (MB1364)
// Dosya adı "board.h" olamaz: Arduino çekirdeğinin kendi board.h'si var ve gölgelenir.
// Sensörler CN7, motorlar CN10, güç ve haberleşme sol taraf.
// Ayrıntı: pin planı sayfası (MERCOM-1 Pin Planı).

#include <Arduino.h>
#include <stddef.h>

#if defined(ARDUINO_TEENSY41)
// ================= TEENSY 4.1 (plan B, 29 Eyl 2026) =================
// Pin numaraları Teensy'nin kendi numaraları. Yazılım SPI ve STEP/DIR için herhangi bir GPIO olur.
#define PIN_ENC_SCK      6
#define PIN_ENC_MISO     7
#define PIN_ENC_MOSI     11
#define PN_ENC_MOSI      11
#define PN_ENC_CS_PAN    8
#define PN_ENC_CS_TILT   9
#define PN_ENC_CS_CE     12    // 29 Eyl devir notu: CS10 yolunda okuma olmadı, CS12 çalıştı
#define PN_ENC_CS_OLD    10    // boş bırakılır, yazılımda INPUT

#define PIN_IMU_SCL      19    // Wire
#define PIN_IMU_SDA      18
#define PN_IMU_RST       32    // bağlı değil; boş pin (RST kullanılmıyor)

#define PN_STEP_PAN      41
#define PN_DIR_PAN       40
#define PN_EN_PAN        39
#define PN_STEP_TILT     38
#define PN_DIR_TILT      37
#define PN_EN_TILT       36
#define PN_STEP_CE       35
#define PN_DIR_CE        34
#define PN_EN_CE         33
// TMC UART bağlanmaz (hiç çalışmadı, gerekmiyor). Aşağıdakiler yalnız derleme için, boş pinler:
#define PN_TMC_UART_PAN  29
#define PN_TMC_UART_TILT 30
#define PN_TMC_UART_CE   31

#define PN_LASER_EN      5

// LoRa E22: Serial6 (24 TX6 -> RXD, 25 RX6 <- TXD)
#define LORA_SERIAL      Serial6
#define PN_LORA_M0       26
#define PN_LORA_M1       27
#define PN_LORA_AUX      28

#define PN_LED_HB        13    // kart LED'i
#define PN_LED_FAULT     31    // Teensy'de ikinci LED yok; boş pin

#else
// ================= NUCLEO-H753ZI (MB1364) =================
// ---- Enkoderler: SPI1, SCK/MISO/MOSI üçüne ortak, CS ayrı ----
#define PIN_ENC_SCK      PA5    // D13
#define PIN_ENC_MISO     PA6    // D12
// 26 Eyl dizilimi (CN7 yukarıdan aşağı): D13 SCK, D12 MISO, D11 CS pan, D10 CS tilt, D9 CS CE, D8 MOSI.
// SPI yazılımla sürüldüğü için herhangi bir GPIO olur; MOSI en alta alındı ki üç CS teli yan yana kalsın.
#ifdef ENC_PINS_OLD                     // eski dizilim (25 Eyl'e kadar): -D ENC_PINS_OLD ile derlenir, kablo karşılaştırma testi için
#define PIN_ENC_MOSI     PB5    // D11
#define PN_ENC_MOSI      PB_5
#define PN_ENC_CS_PAN    PD_14  // D10
#define PN_ENC_CS_TILT   PD_15  // D9
#define PN_ENC_CS_CE     PF_3   // D8
#else
#define PIN_ENC_MOSI     PF3    // D8  (varyant dosyasına göre MB1364'te D8 = PF3)
#define PN_ENC_MOSI      PF_3
#define PN_ENC_CS_PAN    PB_5   // D11
#define PN_ENC_CS_TILT   PD_14  // D10
#define PN_ENC_CS_CE     PD_15  // D9
#endif

// ---- IMU: I2C1 ----
#define PIN_IMU_SCL      PB8    // D15
#define PIN_IMU_SDA      PB9    // D14
#define PN_IMU_RST       PA_3   // A0, BNO085 RST (aktif düşük). INT kullanılmıyor, yazılım sorguluyor.

// ---- Motor sürücüler: STEP/DIR ayrı, UART ve EN ortak ----
// 28 Eyl: GPIOE ve GPIOG pinleri hasarlı (pan DIR D6/PE9 sinyal üretmedi, tilt ve CE hatları da öyle).
// Sürücü sinyalleri hasarsız portlara taşındı: pan DIR A0, tilt ve CE CN7 iç sütununa (D15..D8'in iç komşuları).
// Eski dizilim -D DRV_PINS_OLD ile derlenir (sağlam bir MB1364 kartta).
#ifdef DRV_PINS_OLD
#define PN_STEP_PAN      PG_12  // D7
#define PN_DIR_PAN       PE_9   // D6
#define PN_STEP_TILT     PE_11  // D5
#define PN_DIR_TILT      PE_14  // D4
#define PN_STEP_CE       PE_13  // D3
#define PN_DIR_CE        PG_14  // D2
#else
#define PN_STEP_PAN      PG_12  // D7   (bu pin çalışıyor)
#define PN_DIR_PAN       PA_3   // A0   (CN9 dış sütun, en üst)
#define PN_STEP_TILT     PB_15  // D17  (CN7 iç sütun, D14'ün yanı)
#define PN_DIR_TILT      PB_12  // D19  (CN7 iç sütun, GND'nin yanı)
#define PN_STEP_CE       PC_7   // D21  (CN7 iç sütun, D12'nin yanı)
#define PN_DIR_CE        PA_4   // D24  (CN7 iç sütun, D9'un yanı)
#endif
// TMC2209 UART: her sürücüye AYRI tek telli hat (21 Eyl). Üç sürücüde de MS1/MS2 aynı kalır (adres 0);
// UART düşerse üçü de aynı mikroadımda kalır. PB_6 yalın hâliyle LPUART1'e düşer, USART1 için ALT2 şart.
#define PN_TMC_UART_PAN  PB_6_ALT2  // D1,  USART1
#define PN_TMC_UART_TILT PE_8       // D42, UART7  (CN10-18, D0'ın hemen altı)
#define PN_TMC_UART_CE   PA_0       // D32, UART4  (CN10-29, iç sütun)
// EN aktif düşük, her sürücüye AYRI (21 Eyl): UART çalışmadığı için tek motoru serbest bırakmanın tek yolu bu.
// Üç sürücünün bütün telleri CN10 soketinde: D42, D41, D40 pinleri D0'ın hemen altında (CN10-18, 20, 24).
#define PN_EN_PAN        PB_7   // D0   (bu pin çalışıyor)
#ifdef DRV_PINS_OLD
#define PN_EN_TILT       PE_7   // D41
#define PN_EN_CE         PE_10  // D40
#else
#define PN_EN_TILT       PC_6   // D16  (CN7 iç sütun, D15'in yanı)
#define PN_EN_CE         PA_15  // D20  (CN7 iç sütun, D13'ün yanı)
#endif

// ---- Lazer ----
#define PN_LASER_EN      PC_0   // A1, MOSFET kapısı

// ---- LoRa E22: USART2 ----
#define PN_LORA_TX       PD_5   // MCU TX -> E22 RXD
#define PN_LORA_RX       PD_6   // MCU RX <- E22 TXD
#define PN_LORA_M0       PD_3
#define PN_LORA_M1       PD_4
#define PN_LORA_AUX      PD_7

// ---- Kart üstü LED ----
#define PN_LED_HB        PB_0   // LD1 yeşil, kalp atışı
#define PN_LED_FAULT     PB_14  // LD3 kırmızı, arıza

#endif

// Eksen indeksleri, her yerde aynı sıra kullanılır
#define AX_PAN   0
#define AX_TILT  1
#define AX_CE    2

#define CTRL_HZ      1000   // kontrol döngüsü, TIM14
#define STEP_TICK_HZ 20000  // adım üreteci kesmesi, TIM13
// TIM6 ve TIM7 çekirdekte tone() ve Servo için ayrılmış, kullanılmaz.

#ifdef __cplusplus
extern "C" {
#endif

void board_init(void);
void board_timers_start(void (*step_cb)(void), void (*ctrl_cb)(void));

// Adım üreteci tarafından kesme içinde çağrılır, hızlı olmalı
void board_step_write(int axis, int high);
void board_dir_write(int axis, int positive);

// Enkoder SPI: cs_index = AX_PAN / AX_TILT / AX_CE, 16 bit aktarım, mod 1
void     board_spi_init(void);
void     board_spi_reinit(void);                    // çevre birimi takıldıysa kapat-aç
uint16_t board_spi_xfer16(int cs_index, uint16_t tx);

// IMU I2C: PB8/PB9, 400 kHz. Wire nesnesini hazırlar, sürücü Wire'ı doğrudan kullanır.
void     board_i2c_init(void);
int      board_i2c_scan_swapped(uint8_t *found, int cap);  // SCL/SDA yer değiştirmiş olarak tarar, sonra normale döner
int      board_i2c_scan(uint8_t *found, int cap);   // cevap veren adresleri yazar, sayısını döndürür
int      board_i2c_probe(uint8_t addr);             // 1: adres ACK verdi (bloklamaz, tek transfer)
void     board_i2c_reinit(void);                    // STM32 I2C çevre birimini sıfırla (akış kesilince kurtarma)
void     board_i2c_pads(int soft);                  // I2C pin kenarı: 1 yumuşak (çınlamaya karşı), 0 kütüphane varsayılanı, -1 son ayarı yeniden uygula
// I2C meşgul sayacı (29 Eyl): >0 iken hatta aktarım sürüyor olabilir; kontrol kesmesi enkoder SPI'sini o tikte atlar.
// Enkoder hatlarının kenarları I2C kablosuna sızıp BNO085'i resetliyordu (enc_read 0: 0 reset, 1: dakikada 4,3).
// Yalnız ana döngü artırıp azaltır, kesme yalnız okur.
extern volatile uint8_t g_i2c_busy;
#define I2C_BUSY_BEGIN() (g_i2c_busy++)
#define I2C_BUSY_END()   do { if (g_i2c_busy) g_i2c_busy--; } while (0)
void     board_i2c_levels(int *scl, int *sda, int *rst); // hat seviyeleri: modül besleniyorsa SCL/SDA 1 okur
void     board_i2c_activity(int *scl_edges, int *sda_edges); // 20 ms boyunca hat geçişlerini sayar (UART modu tanısı)
int      board_i2c_scan_slow(uint8_t *found, int cap);  // 100 kHz'de tarama
void     board_watchdog_start(uint32_t ms);         // bağımsız watchdog; ana döngü beslemezse kart resetlenir
void     board_watchdog_feed(void);
int      board_nv_read(uint16_t addr, void *buf, uint16_t n);        // kalıcı bellek (Teensy: flash EEPROM öykünmesi, 4 KB); 0 ok, -1 yok/taşma
int      board_nv_write(uint16_t addr, const void *buf, uint16_t n); // aynı bayt yeniden yazılmaz; yazarken kesmeler kısa süre kapanır
const char *board_reset_cause(void);                // son resetin sebebi, açılışta bir kez okunur ve bayraklar temizlenir
void     board_imu_hw_reset(void);                  // RST pinini 20 ms düşük tutar, sonra sensörün açılmasını bekler
void     board_imu_rst_write(int high);             // RST pinini elle sür (tanı için)
void     board_i2c_bus_recover(void);               // SDA takılı kaldıysa SCL'i 9 kez darbeleyip serbest bırakır

// TMC2209 tek hat UART, PB6. Gönderim bitince alıcı açılır.
void     board_tmc_uart_begin(uint32_t baud);                 // üç hattı birden açar
void     board_tmc_uart_send(int ch, const uint8_t *buf, size_t n);   // ch: 0 pan, 1 tilt, 2 ce
int      board_tmc_uart_read_byte(int ch);                    // -1: bayt yok
void     board_tmc_uart_flush_rx(int ch);
int      board_tmc_uart_line_level(int ch);                   // 1: hat boşta yüksek
int      board_tmc_uart_tx_check(int ch);                     // telsiz sınama: gönderirken pinde düşük darbe görüldü mü
void     board_tmc_uart_selftest(int ch, int *rx_ok, int *rx_byte, int *tx_ok); // hat pini ile A0 arasına tel takılıyken
void     board_delay_ms(uint32_t ms);
void     board_reset(void);                          // işlemciyi yazılımla yeniden başlat

void board_motors_enable(int on);            // 1: üç sürücü de aktif (EN düşük)
void board_motor_enable(int axis, int on);   // tek eksen
int  board_motors_enabled(void);             // herhangi biri aktif mi
int  board_motor_enabled(int axis);
void board_laser(int on);
void board_led_hb(int on);
void board_led_fault(int on);

uint32_t board_micros(void);
uint32_t board_millis(void);

#ifdef __cplusplus
}
#endif
