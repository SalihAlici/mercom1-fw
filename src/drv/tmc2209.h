#pragma once
// TMC2209 UART sürücüsü, kütüphanesiz. Her sürücüye ayrı tek telli hat (ch); adres MS1/MS2 ile, ayrı hatta hep 0.
// Datagram: yazma 8 bayt [0x05, adres, reg|0x80, d3, d2, d1, d0, crc]
//           okuma isteği 4 bayt [0x05, adres, reg, crc], cevap 8 bayt [0x05, 0xFF, reg, d3..d0, crc]
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TMC_REG_GCONF      0x00
#define TMC_REG_GSTAT      0x01
#define TMC_REG_IFCNT      0x02
#define TMC_REG_SLAVECONF  0x03
#define TMC_REG_IOIN       0x06
#define TMC_REG_IHOLD_IRUN 0x10
#define TMC_REG_TPOWERDOWN 0x11
#define TMC_REG_TPWMTHRS   0x13
#define TMC_REG_CHOPCONF   0x6C
#define TMC_REG_DRV_STATUS 0x6F
#define TMC_REG_PWMCONF    0x70

typedef struct {
    uint8_t  ch;          // UART hattı: 0 pan, 1 tilt, 2 ce
    uint8_t  addr;        // 0..3, MS1 bit0, MS2 bit1 (ayrı hatlarda 0)
    uint8_t  present;     // IOIN okundu ve sürüm 0x21
    uint8_t  configured;  // yazmalar IFCNT ile doğrulandı
    uint8_t  version;
    uint8_t  ifcnt;
    uint8_t  cs;          // akım ölçeği 0..31
    uint8_t  vsense;
    uint32_t drv_status;
    uint32_t err_count;
} tmc2209_t;

void    tmc2209_init(tmc2209_t *d, uint8_t ch, uint8_t addr);
int     tmc2209_probe(tmc2209_t *d);                      // 1: sürücü hatta
int     tmc2209_write(tmc2209_t *d, uint8_t reg, uint32_t val);
int     tmc2209_read(tmc2209_t *d, uint8_t reg, uint32_t *val);   // 1: geçerli cevap
int     tmc2209_configure(tmc2209_t *d, uint16_t irun_ma, uint8_t ihold_pct, uint16_t usteps, float rsense);
int     tmc2209_read_status(tmc2209_t *d);                // DRV_STATUS -> d->drv_status
uint8_t tmc2209_crc(const uint8_t *data, uint8_t n);

// DRV_STATUS bitleri
#define TMC_ST_OTPW   (1u << 0)
#define TMC_ST_OT     (1u << 1)
#define TMC_ST_S2GA   (1u << 2)
#define TMC_ST_S2GB   (1u << 3)
#define TMC_ST_S2VSA  (1u << 4)
#define TMC_ST_S2VSB  (1u << 5)
#define TMC_ST_OLA    (1u << 6)
#define TMC_ST_OLB    (1u << 7)
#define TMC_ST_STEALTH (1u << 30)
#define TMC_ST_STST   (1u << 31)
#define TMC_ST_CS_ACTUAL(s) (((s) >> 16) & 0x1F)

#ifdef __cplusplus
}
#endif
