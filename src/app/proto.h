#pragma once
// MERCOM-1 haberleşme protokolü. PC tarafı tools/mercom_link.py ile aynı.
//
// Çerçeve:  [0xA5][LEN][ID][PAYLOAD x LEN][CRC16 lo][CRC16 hi]
// CRC16-CCITT (poly 0x1021, init 0xFFFF), LEN'den PAYLOAD sonuna kadar.
// Tüm çok baytlı sayılar little-endian, float 32 bit IEEE.

#include <stdint.h>

#define PROTO_SOF          0xA5
#define PROTO_MAX_PAYLOAD  200

// PC -> MCU
enum {
    MSG_PING        = 0x01, // yük yok
    MSG_SET_MODE    = 0x02, // u8 mode (bkz. fsm.h)
    MSG_JOG         = 0x03, // u8 axis, f32 hız (derece/s), 0 = dur
    MSG_GOTO        = 0x04, // u8 axis, f32 açı (derece): ivmeli profille git; pan kısa yoldan, tilt/CE sınıra kırpılır
    MSG_LOCK        = 0x05, // yük yok: şu anki bakış doğrultusunu hedef yap (IMU gerekli)
    MSG_UNLOCK      = 0x06, // yük yok
    MSG_LASER       = 0x07, // u8 on
    MSG_PARAM_GET   = 0x08, // u16 id
    MSG_PARAM_SET   = 0x09, // u16 id, f32 değer
    MSG_PARAM_SAVE  = 0x0A, // yük yok: tüm parametreleri kalıcı belleğe yaz; 1 bayt 0: kalıcı kaydı sil (sonraki açılış varsayılan)
    MSG_ENABLE      = 0x0B, // u8 on: üç sürücü birden  |  u8 eksen, u8 on: tek sürücü (EN pinleri ayrı)
    MSG_TELEM_RATE  = 0x0C, // u16 Hz, 0 = kapalı
    MSG_STOP        = 0x0D, // yük yok, tüm eksenleri durdur
    MSG_CLEAR_FAULT = 0x0E, // yük yok
    MSG_ENC_DIAG    = 0x0F, // yük yok, cevap TEXT: üç enkoderin AGC ve bayrakları
    MSG_IMU_DIAG    = 0x10, // yük yok, cevap TEXT: IMU durumu, sayaç, son açı
    MSG_TMC_DIAG    = 0x11, // yük yok, cevap TEXT: üç sürücünün sürüm, akım, DRV_STATUS
    MSG_TMC_INIT    = 0x12, // yük yok, sürücüleri yeniden tara ve ayarla
    MSG_I2C_SCAN    = 0x13, // yük yok, cevap TEXT: I2C'de cevap veren adresler
    MSG_IMU_INIT    = 0x14, // yük yok, IMU'yu yeniden başlat (kablo sonradan takıldıysa)
    MSG_YAW_ZERO    = 0x15, // yük yok, şu anki yaw'ı gösterim için sıfır kabul et
    MSG_TMC_TXCHK   = 0x1C, // yük yok: üç UART hattında telsiz verici sınaması (pinde darbe görülüyor mu)
    MSG_STEP_BLINK  = 0x1B, // u8 axis: o eksenin STEP pinini 20 s boyunca 3 s yüksek / 3 s düşük sürer (multimetre ile pin/tel testi)
    MSG_TMC_LOOP    = 0x1A, // u8 hat (0 pan,1 tilt,2 ce): o hattın pini ile A0 arasına tel takılıyken alıcı/verici öz sınaması
    MSG_TMC_RAW     = 0x19, // u8 hat (0 pan,1 tilt,2 ce) [, u8 adres=0]: IOIN okuma isteği gönderir, 30 ms içinde gelen HAM baytları hex yazar (yankı/cevap tanısı)
    MSG_STEP_MOVE   = 0x18, // u8 axis, i32 mikroadım (işaretli), f32 adım/s: tam N adım at, dur (oran ölçümü)
    MSG_SYS_RESET   = 0x17, // yük yok: kartı yeniden başlatır (NVIC_SystemReset), sensör asılı kaldığında son çare
    MSG_IMU_PINTEST = 0x16, // yük yok: A0 düşükken ve yüksekken I2C tarar, kablonun RST mi P0/P1 mi olduğunu söyler
    MSG_OSC         = 0x1D, // u8 axis, f32 A, f32 B, u16 bekleme ms: A ile B arasında salın; u8 axis tek başına: salınımı durdur
    MSG_AXIS_ZERO   = 0x1E, // u8 axis, f32 deg: şu anki konumu deg kabul et (adım sayacı + enkoder sıfırı)
    MSG_AXIS_INFO   = 0x1F, // yük yok, cevap TEXT: eksenlerin hedef, hız, sınır ve düzeltme sayaçları
    MSG_TILT_FROM_IMU = 0x20, // yük yok: tilt açısını IMU'nun ölçtüğü elevasyona eşitle (taban düz olmalı)
    MSG_LORA_CFG    = 0x21, // yük yok: E22 ayarlarını oku (C1 00 09) ve TEXT ile bildir; 9 bayt yük: ADDH ADDL NETID REG0 REG1 REG2 REG3 CRYPT_H CRYPT_L yaz (C0, kalıcı)
};

// MCU -> PC
enum {
    MSG_PONG        = 0x81, // u32 uptime ms, u32 fw sürümü
    MSG_TELEM       = 0x82, // telem_t (telemetry.h)
    MSG_PARAM_VALUE = 0x83, // u16 id, f32 değer, u8 ok
    MSG_TEXT        = 0x84, // ASCII metin, sonlandırıcı yok
    MSG_ACK         = 0x85, // u8 yanıtlanan id, u8 durum (0 ok)
};

// ACK durumları
enum {
    ACK_OK          = 0,
    ACK_BAD_LEN     = 1,
    ACK_BAD_ARG     = 2,
    ACK_WRONG_MODE  = 3,
    ACK_NOT_IMPL    = 4,
    ACK_REFUSED     = 5,
};
