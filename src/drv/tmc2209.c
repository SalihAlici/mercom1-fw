#include "tmc2209.h"
#include "../hw.h"
#include <string.h>

#define SYNC      0x05
#define MASTER    0xFF
#define WRITE_BIT 0x80

// TMC veri sayfasındaki CRC8, poly 0x07, bit sırası ters
uint8_t tmc2209_crc(const uint8_t *data, uint8_t n)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < n; i++) {
        uint8_t b = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (b & 1)) crc = (uint8_t)((crc << 1) ^ 0x07);
            else                      crc = (uint8_t)(crc << 1);
            b >>= 1;
        }
    }
    return crc;
}

void tmc2209_init(tmc2209_t *d, uint8_t ch, uint8_t addr)
{
    memset(d, 0, sizeof(*d));
    d->ch = ch;
    d->addr = addr & 3;
}

int tmc2209_write(tmc2209_t *d, uint8_t reg, uint32_t val)
{
    uint8_t dg[8] = { SYNC, d->addr, (uint8_t)(reg | WRITE_BIT),
                      (uint8_t)(val >> 24), (uint8_t)(val >> 16), (uint8_t)(val >> 8), (uint8_t)val, 0 };
    dg[7] = tmc2209_crc(dg, 7);
    board_tmc_uart_flush_rx(d->ch);
    board_tmc_uart_send(d->ch, dg, 8);
    board_delay_ms(1);
    return 1;
}

// Cevabı 0x05 0xFF eşlemesiyle arar, olası yankıyı atlar
int tmc2209_read(tmc2209_t *d, uint8_t reg, uint32_t *val)
{
    uint8_t req[4] = { SYNC, d->addr, reg, 0 };
    req[3] = tmc2209_crc(req, 3);

    for (int attempt = 0; attempt < 2; attempt++) {
        board_tmc_uart_flush_rx(d->ch);
        board_tmc_uart_send(d->ch, req, 4);

        uint8_t rep[8];
        uint8_t k = 0;
        uint32_t t0 = board_millis();
        while (board_millis() - t0 < 12) {
            int c = board_tmc_uart_read_byte(d->ch);
            if (c < 0) continue;
            uint8_t b = (uint8_t)c;
            if (k == 0) { if (b == SYNC) rep[k++] = b; continue; }
            if (k == 1) { if (b == MASTER) rep[k++] = b; else k = (b == SYNC) ? 1 : 0; continue; }
            rep[k++] = b;
            if (k == 8) break;
        }
        if (k == 8 && (rep[2] & 0x7F) == reg && tmc2209_crc(rep, 7) == rep[7]) {
            *val = ((uint32_t)rep[3] << 24) | ((uint32_t)rep[4] << 16) | ((uint32_t)rep[5] << 8) | rep[6];
            return 1;
        }
        d->err_count++;
        board_delay_ms(2);
    }
    return 0;
}

int tmc2209_probe(tmc2209_t *d)
{
    uint32_t ioin = 0;
    if (!tmc2209_read(d, TMC_REG_IOIN, &ioin)) { d->present = 0; return 0; }
    d->version = (uint8_t)(ioin >> 24);
    d->present = (d->version == 0x21) ? 1 : 0;
    return d->present;
}

static uint8_t mresFromUsteps(uint16_t u)
{
    switch (u) {
    case 256: return 0; case 128: return 1; case 64: return 2; case 32: return 3;
    case 16:  return 4; case 8:   return 5; case 4:  return 6; case 2:  return 7;
    case 1:   return 8; default:  return 4;
    }
}

int tmc2209_configure(tmc2209_t *d, uint16_t irun_ma, uint8_t ihold_pct, uint16_t usteps, float rsense)
{
    uint32_t v;
    if (!tmc2209_read(d, TMC_REG_IFCNT, &v)) { d->configured = 0; return 0; }
    uint8_t ifcnt0 = (uint8_t)v;

    // Akım ölçeği: CS = 32*sqrt2*I*(Rs+0.02)/Vfs - 1. Önce hassas aralık (vsense=1, 0.180 V) denenir.
    float i_a = (float)irun_ma / 1000.0f;
    float cs_f = 32.0f * 1.41421f * i_a * (rsense + 0.02f) / 0.180f - 1.0f;
    uint8_t vsense = 1;
    if (cs_f > 31.0f) {
        vsense = 0;
        cs_f = 32.0f * 1.41421f * i_a * (rsense + 0.02f) / 0.325f - 1.0f;
    }
    int cs = (int)(cs_f + 0.5f);
    if (cs < 0) cs = 0;
    if (cs > 31) cs = 31;
    int ihold = cs * ihold_pct / 100;
    if (ihold < 0) ihold = 0;
    if (ihold > 31) ihold = 31;

    uint32_t gconf      = (1u << 6) | (1u << 7) | (1u << 8);   // pdn_disable, mstep_reg_select, multistep_filt; StealthChop
    uint32_t chopconf   = 0x10000053u | ((uint32_t)mresFromUsteps(usteps) << 24) | ((uint32_t)vsense << 17);
    uint32_t ihold_irun = (uint32_t)ihold | ((uint32_t)cs << 8) | (8u << 16);

    tmc2209_write(d, TMC_REG_SLAVECONF,  2u << 8);   // SENDDELAY 3x8 bit: cevap için alıcıya zaman
    tmc2209_write(d, TMC_REG_GCONF,      gconf);
    tmc2209_write(d, TMC_REG_CHOPCONF,   chopconf);
    tmc2209_write(d, TMC_REG_IHOLD_IRUN, ihold_irun);
    tmc2209_write(d, TMC_REG_TPOWERDOWN, 20);
    tmc2209_write(d, TMC_REG_TPWMTHRS,   0);          // her hızda StealthChop

    if (!tmc2209_read(d, TMC_REG_IFCNT, &v)) { d->configured = 0; return 0; }
    d->ifcnt = (uint8_t)v;
    d->cs = (uint8_t)cs;
    d->vsense = vsense;
    d->configured = ((uint8_t)(d->ifcnt - ifcnt0) == 6) ? 1 : 0;
    return d->configured;
}

int tmc2209_read_status(tmc2209_t *d)
{
    uint32_t v;
    if (!tmc2209_read(d, TMC_REG_DRV_STATUS, &v)) return 0;
    d->drv_status = v;
    return 1;
}
