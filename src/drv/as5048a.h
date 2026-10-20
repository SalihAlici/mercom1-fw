#pragma once
// AS5048A 14 bit mutlak manyetik enkoder, SPI mod 1, 16 bit çerçeve.
// Okuma iki aktarımla yapılır: komut, ardından NOP ile cevap.
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AS5048A_CPR 16384

typedef struct {
    int      cs;          // board_spi_xfer16 için CS indeksi (AX_PAN, AX_TILT, AX_CE)
    uint16_t raw;         // son geçerli açı 0..16383
    uint8_t  agc;         // 0..255, mıknatıs boşluğu göstergesi, orta bant iyi
    uint8_t  diag;        // bit0 OCF (hazır), bit1 COF, bit2 COMP_L (alan çok GÜÇLÜ, mıknatıs yakın), bit3 COMP_H (alan çok ZAYIF, mıknatıs uzak)
    uint8_t  ok;          // son okuma geçerli mi
    uint8_t  fail_streak; // arka arkaya hatalı okuma
    uint32_t ok_count;
    uint32_t err_count;
    uint16_t last_rx;     // son gelen ham SPI kelimesi, kablo tanısı için
    uint16_t last_bad;    // son hatalı açı cevabı
    uint32_t n_parity;    // parite hatası sayısı
    uint32_t n_ef;        // hata bayrağı (EF) sayısı
} as5048a_t;

void  as5048a_init(as5048a_t *e, int cs);
int   as5048a_read_angle(as5048a_t *e);   // 1 geçerli, 0 hata; sonuç e->raw
int   as5048a_read_diag(as5048a_t *e);    // AGC ve tanı bayraklarını okur
void  as5048a_clear_error(as5048a_t *e);
float as5048a_raw_to_deg(uint16_t raw);   // 0..360

#ifdef __cplusplus
}
#endif
