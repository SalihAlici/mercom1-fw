#include "as5048a.h"
#include "../hw.h"

#define REG_NOP        0x0000
#define REG_CLEAR_ERR  0x0001
#define REG_DIAG_AGC   0x3FFD
#define REG_MAGNITUDE  0x3FFE
#define REG_ANGLE      0x3FFF
#define CMD_READ       0x4000
#define BIT_PARITY     0x8000
#define BIT_ERROR      0x4000

// Çift parite: 16 bitteki 1 sayısı çift olacak şekilde bit15 ayarlanır
static uint16_t withParity(uint16_t v)
{
    uint16_t x = v & 0x7FFF;
    uint16_t p = x;
    p ^= p >> 8; p ^= p >> 4; p ^= p >> 2; p ^= p >> 1;
    return (p & 1) ? (x | BIT_PARITY) : x;
}

static int parityOk(uint16_t v)
{
    uint16_t p = v;
    p ^= p >> 8; p ^= p >> 4; p ^= p >> 2; p ^= p >> 1;
    return (p & 1) == 0;
}

// Bir yazmacı okur: komut gönder, NOP ile cevabı al. 1 geçerli.
static int readReg(as5048a_t *e, uint16_t reg, uint16_t *out)
{
    board_spi_xfer16(e->cs, withParity(CMD_READ | reg));
    uint16_t r = board_spi_xfer16(e->cs, withParity(CMD_READ | REG_NOP));
    e->last_rx = r;
    // 0x0000: MISO hattı düşükte asılı ya da boşta (çip hattı sürmüyor). Paritesi tuttuğu için eskiden
    // "geçerli, açı 0" sayılıyordu; artık hata. Gerçek açı tam 0 olsa bile tek örnek atlanır, zararı yok.
    if (r == 0x0000 || !parityOk(r) || (r & BIT_ERROR)) {
        e->last_bad = r;
        if (r == 0x0000 || !parityOk(r)) e->n_parity++; else e->n_ef++;
        return 0;
    }
    *out = r & 0x3FFF;
    return 1;
}

void as5048a_init(as5048a_t *e, int cs)
{
    e->cs = cs; e->raw = 0; e->agc = 0; e->diag = 0; e->ok = 0;
    e->fail_streak = 0; e->ok_count = 0; e->err_count = 0;
    e->last_rx = 0; e->last_bad = 0; e->n_parity = 0; e->n_ef = 0;
    as5048a_clear_error(e);
}

int as5048a_read_angle(as5048a_t *e)
{
    uint16_t v;
    if (readReg(e, REG_ANGLE, &v)) {
        e->raw = v; e->ok = 1; e->fail_streak = 0; e->ok_count++;
        return 1;
    }
    e->ok = 0; e->err_count++;
    if (e->fail_streak < 255) e->fail_streak++;
    // Hata bayrağı (EF) bir kez kalkınca temizlenene kadar her cevap hatalı gelir; hemen temizle
    if (e->last_rx & BIT_ERROR) as5048a_clear_error(e);
    return 0;
}

int as5048a_read_diag(as5048a_t *e)
{
    uint16_t v;
    if (!readReg(e, REG_DIAG_AGC, &v)) return 0;
    e->agc  = (uint8_t)(v & 0xFF);
    e->diag = (uint8_t)((v >> 8) & 0x0F);
    return 1;
}

void as5048a_clear_error(as5048a_t *e)
{
    board_spi_xfer16(e->cs, withParity(CMD_READ | REG_CLEAR_ERR));
    board_spi_xfer16(e->cs, withParity(CMD_READ | REG_NOP));
}

float as5048a_raw_to_deg(uint16_t raw)
{
    return (float)raw * (360.0f / (float)AS5048A_CPR);
}
