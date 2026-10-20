#include "comm.h"
#include <string.h>

enum { ST_SOF = 0, ST_LEN, ST_ID, ST_PAYLOAD, ST_CRC_LO, ST_CRC_HI };

uint16_t crc16_ccitt(const uint8_t *data, size_t n, uint16_t crc)
{
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int k = 0; k < 8; k++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

void comm_rx_init(comm_rx_t *rx)
{
    memset(rx, 0, sizeof(*rx));
    rx->state = ST_SOF;
}

void comm_rx_byte(comm_rx_t *rx, uint8_t b, comm_frame_cb cb)
{
    switch (rx->state) {
    case ST_SOF:
        if (b == PROTO_SOF) rx->state = ST_LEN;
        break;
    case ST_LEN:
        if (b > PROTO_MAX_PAYLOAD) {
            // Sahte SOF: bu bayt gerçek SOF olabilir, o zaman LEN beklemeye devam
            rx->rx_errors++;
            rx->state = (b == PROTO_SOF) ? ST_LEN : ST_SOF;
            break;
        }
        rx->len = b;
        rx->idx = 0;
        rx->crc_calc = crc16_ccitt(&b, 1, 0xFFFF);
        rx->state = ST_ID;
        break;
    case ST_ID:
        if (b == 0 || b >= 0x80) {              // MCU'ya yalnız 0x01..0x7F gelir
            rx->rx_errors++;
            rx->state = (b == PROTO_SOF) ? ST_LEN : ST_SOF;
            break;
        }
        rx->id = b;
        rx->crc_calc = crc16_ccitt(&b, 1, rx->crc_calc);
        rx->state = (rx->len == 0) ? ST_CRC_LO : ST_PAYLOAD;
        break;
    case ST_PAYLOAD:
        rx->buf[rx->idx++] = b;
        rx->crc_calc = crc16_ccitt(&b, 1, rx->crc_calc);
        if (rx->idx >= rx->len) rx->state = ST_CRC_LO;
        break;
    case ST_CRC_LO:
        rx->crc_lo = b;
        rx->state = ST_CRC_HI;
        break;
    case ST_CRC_HI: {
        uint16_t crc_rx = (uint16_t)rx->crc_lo | ((uint16_t)b << 8);
        if (crc_rx == rx->crc_calc) {
            rx->rx_frames++;
            if (cb) cb(rx->id, rx->buf, rx->len);
        } else {
            rx->rx_errors++;
        }
        rx->state = ST_SOF;
        break;
    }
    default:
        rx->state = ST_SOF;
    }
}

size_t comm_encode(uint8_t *out, size_t out_cap, uint8_t id, const void *payload, uint8_t len)
{
    size_t total = 1 + 1 + 1 + (size_t)len + 2;
    if (out_cap < total || len > PROTO_MAX_PAYLOAD) return 0;
    out[0] = PROTO_SOF;
    out[1] = len;
    out[2] = id;
    if (len) memcpy(&out[3], payload, len);
    uint16_t crc = crc16_ccitt(&out[1], 2 + (size_t)len, 0xFFFF);
    out[3 + len] = (uint8_t)(crc & 0xFF);
    out[4 + len] = (uint8_t)(crc >> 8);
    return total;
}
