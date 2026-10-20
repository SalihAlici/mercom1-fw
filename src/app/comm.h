#pragma once
// Çerçeve ayrıştırıcı ve kodlayıcı. Çatıdan bağımsız, bayt bayt beslenir.
#include <stdint.h>
#include <stddef.h>
#include "proto.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*comm_frame_cb)(uint8_t id, const uint8_t *payload, uint8_t len);

typedef struct {
    uint8_t  state;
    uint8_t  len;
    uint8_t  id;
    uint8_t  idx;
    uint8_t  buf[PROTO_MAX_PAYLOAD];
    uint16_t crc_calc;
    uint8_t  crc_lo;
    uint32_t rx_frames;
    uint32_t rx_errors;
} comm_rx_t;

void     comm_rx_init(comm_rx_t *rx);
void     comm_rx_byte(comm_rx_t *rx, uint8_t b, comm_frame_cb cb);

// Çerçeveyi out'a yazar, yazılan bayt sayısını döndürür. Sığmazsa 0.
size_t   comm_encode(uint8_t *out, size_t out_cap, uint8_t id, const void *payload, uint8_t len);

uint16_t crc16_ccitt(const uint8_t *data, size_t n, uint16_t crc);

#ifdef __cplusplus
}
#endif
