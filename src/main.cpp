// MERCOM-1 ana döngü. Adım 1 ve 2: kart iskeleti, 1 kHz tik, komut protokolü,
// telemetri, parametreler, durum makinesi, açık çevrim jog (adım 4 motor testi için).
// Enkoder, TMC ayarı, IMU ve eksen takipçisi sonraki adımlarda eklenir.
#include <Arduino.h>
#include <string.h>
#include <stdio.h>
#include "hw.h"

extern "C" {
#include "app/proto.h"
#include "app/comm.h"
#include "app/params.h"
#include "app/telemetry.h"
#include "app/fsm.h"
#include "drv/stepgen.h"
#include "drv/as5048a.h"
#include "drv/tmc2209.h"
#include "drv/bno085.h"
#include "ctrl/axis.h"
#include "ctrl/pointing.h"
}

static as5048a_t        s_enc[3];
static tmc2209_t        s_tmc[3];
static uint8_t          s_imuOk = 0;
static float            s_yawZero = 0.0f;   // yalnız gösterim için; GRV yaw'ı her açılışta rastgele başlar
// Enkoder tanı okuması da SPI kullanır; ana döngüden yapılırsa kesmedeki okumayla çakışıp SPI'yı kilitler.
// İstek bayrakla kesmeye iletilir, okuma kesmede yapılır, ana döngü yalnız sonucu yazar.
static volatile uint32_t s_spiReinits = 0;
static volatile uint8_t s_encDiagReq = 0;
static volatile uint8_t s_encDiagDone = 0;
static uint8_t          s_encDiagOk[3];
static comm_rx_t        s_rx;
static telem_t          s_tel;
static volatile uint32_t s_ctrlTicks = 0;
static uint32_t         s_lastTelemMs = 0;
static uint32_t         s_lastHbMs = 0;
static volatile uint16_t s_loopMaxUs = 0;
static volatile uint32_t s_overruns = 0;
static uint8_t          s_laserOn = 0;
static float            s_encDegCont[3] = { 0, 0, 0 };   // sürekli (sarmalanmamış) enkoder açısı, eksen katmanı için
static int32_t  s_encCont[3]   = { 0, 0, 0 };    // motor mili, ham birim (16384/tur), sürekli
static uint16_t s_encSinceOk[3] = { 1, 1, 1 };   // son geçerli okumadan beri geçen tik
static uint32_t s_encGlitch[3] = { 0, 0, 0 };
static uint8_t  s_encMagOk[3]  = { 1, 1, 1 };    // mıknatıs menzilde (tanı yazmacından, 600 ms'de bir)
static volatile uint8_t s_encWarn[3] = { 0, 0, 0 };   // kesme: enkoder 200 ms okunamadı, ana döngü uyarı yazsın
static volatile uint32_t s_encReads = 0, s_encBusySkips = 0;   // başarılı enkoder okuması / I2C meşgul diye atlanan tik
static uint16_t s_encTrust[3] = { 0, 0, 0 };          // art arda sağlıklı okuma sayısı; kapalı çevrim ancak 1000 tik (1 s) sonra güvenir

// ---------------- LoRa (E22-900T22D, şeffaf mod M0=M1=0, 9600 8N1) ----------------
// 30 Eyl: kablosuz erişim. Protokol USB ile aynı çerçeveler. LoRa'dan gelen komutun cevabı LoRa'ya gider; LoRa bağlantısı
// etkinken (son 15 s'de çerçeve geldiyse) telemetri lora_telem_hz ile LoRa'ya da gönderilir, kart mesajları da kopyalanır.
// Havadaki hız ~300 bayt/s: LoRa'ya yazarken tampon yetmiyorsa çerçeve atlanır, ana döngü asla beklemez.
#ifdef LORA_SERIAL
// 30 Eyl: sabit noktalı (adresli) mod, diğer takımların E22'leriyle çakışmasın. Terminal 0x4D01, yer istasyonu 0x4D02,
// kanal 15 (865,125 MHz, 863–870 MHz lisanssız bant), NETID 0x4D, şifre anahtarı iki modülde aynı (tools/lora_ayar.py).
// Sabit modda her paketin başına hedefin ADDH ADDL KANAL baytları yazılır; alıcı modül bu 3 baytı çıkarıp yalnız veriyi verir.
static const uint8_t LORA_HEDEF[3] = { 0x4D, 0x02, 0x0F };   // yer istasyonu adresi + kanal
static comm_rx_t s_rxLora;
static uint8_t   s_outLora = 0;           // 1: sendFrame LoRa'ya yazar (LoRa'dan gelen komut işlenirken, LoRa telemetrisi)
static uint32_t  s_loraLastRxMs = 0;
static uint32_t  s_loraLastTelemMs = 0;
static uint32_t  s_loraDropped = 0;       // kuyruk dolu diye atlanan çerçeve
static uint8_t   s_loraTxMem[1024];
static int loraActive(void) { return s_loraLastRxMs && (millis() - s_loraLastRxMs) < 15000; }
// Gönderim kuyruğu (30 Eyl): E22 sabit modda art arda yazılan veriyi 240 baytlık paketlere bölüyor, yalnız ilk paket doğru
// adrese gidiyor (enkoder tanısının 2. ve 3. satırı kayboluyordu). Her çerçeve ayrı yazılır: önceki yazım UART'tan çıktıktan
// ve modül yayını bitirdikten (AUX = 1) sonra, en az 20 ms arayla.
// Küçük çerçeveler (ACK, parametre cevabı) bir pakette birleştirilir: tek başlık + 237 bayta kadar bütün çerçeveler.
#define LQ_N 32
#define LQ_PKT 237                          // E22 paketi 240 bayt, 3'ü başlık
static uint8_t  s_lq[LQ_N][PROTO_MAX_PAYLOAD + 5];
static uint16_t s_lqLen[LQ_N];
static uint8_t  s_lqHead = 0, s_lqTail = 0;
static uint32_t s_lqLastWriteMs = 0;
static uint16_t s_lqLastLen = 0;
static int loraQueue(const uint8_t *buf, size_t k)
{
    uint8_t nx = (uint8_t)((s_lqHead + 1) % LQ_N);
    if (nx == s_lqTail || k > sizeof s_lq[0]) { s_loraDropped++; return 0; }
    memcpy(s_lq[s_lqHead], buf, k);
    s_lqLen[s_lqHead] = (uint16_t)k; s_lqHead = nx;
    return 1;
}
static void loraPump(void)
{
    if (s_lqTail == s_lqHead) return;
    // önceki yazım: UART süresi (~1,05 ms/bayt) + 20 ms geçmeli, sonra modül boşta (AUX 1) olmalı
    if (millis() - s_lqLastWriteMs < (uint32_t)(s_lqLastLen * 105 / 100 + 20)) return;
    if (digitalRead(PN_LORA_AUX) == LOW) return;
    // önce kaç çerçevenin sığdığını say, tampon yeterse kopyala ve kuyruktan düş
    uint16_t n = 3; uint8_t t = s_lqTail;
    while (t != s_lqHead && n + s_lqLen[t] <= 3 + LQ_PKT) { n += s_lqLen[t]; t = (uint8_t)((t + 1) % LQ_N); }
    if (n == 3) { s_lqTail = (uint8_t)((s_lqTail + 1) % LQ_N); s_loraDropped++; return; }   // sığmayan çerçeve (olmamalı)
    if ((size_t)LORA_SERIAL.availableForWrite() < n) return;
    uint8_t pkt[3 + LQ_PKT]; uint16_t m = 3;
    memcpy(pkt, LORA_HEDEF, 3);
    while (s_lqTail != t) { memcpy(pkt + m, s_lq[s_lqTail], s_lqLen[s_lqTail]); m += s_lqLen[s_lqTail]; s_lqTail = (uint8_t)((s_lqTail + 1) % LQ_N); }
    LORA_SERIAL.write(pkt, m);
    s_lqLastWriteMs = millis(); s_lqLastLen = m;
}
// LoRa'dan gelen döndürme (basılı tutunca döner) komutu 1,5 s tazelenmezse eksen durur: bırakma komutu kaybolursa motor
// dönmeye devam etmesin. Arayüz LoRa'dayken düğme basılı kaldıkça komutu 0,5 s'de bir yeniden gönderir.
static uint32_t s_loraJogMs = 0;
static int8_t   s_loraJogAxis = -1;
#endif

// ---------------- gönderme yardımcıları ----------------
static void sendFrame(uint8_t id, const void *p, uint8_t n)
{
    uint8_t buf[PROTO_MAX_PAYLOAD + 5];
    size_t k = comm_encode(buf, sizeof buf, id, p, n);
    if (!k) return;
#ifdef LORA_SERIAL
    if (s_outLora) { loraQueue(buf, k); return; }
#endif
    Serial.write(buf, k);
}
static void sendText(const char *s)
{
    sendFrame(MSG_TEXT, s, (uint8_t)strlen(s));
#ifdef LORA_SERIAL
    if (!s_outLora && loraActive()) { s_outLora = 1; sendFrame(MSG_TEXT, s, (uint8_t)strlen(s)); s_outLora = 0; }   // kart mesajları LoRa'ya da
#endif
}
static void sendAck(uint8_t id, uint8_t status) { uint8_t p[2] = { id, status }; sendFrame(MSG_ACK, p, 2); }

// snprintf %f nano libc'de kapalı olabilir; dereceyi binde bir hassasiyetle tam sayı yazar
static long mdeg(float v) { return (long)(v * 1000.0f + (v >= 0 ? 0.5f : -0.5f)); }

// ---------------- IMU başlatma: donanım reseti + birkaç deneme ----------------
// tools/bno_basic ile birebir aynı sıra: yalnız Wire üzerinden başlat. Donanım reseti ve hat kurtarma
// çıkarıldı; basit program onlarsız sorunsuz çalıştı, ana firmware onlarla raporsuz kaldı.
static uint32_t s_imuStalls = 0;
static const char *s_bootCause = "bilinmiyor";   // kartın son açılış sebebi (IMU_DIAG ile sorulur)

static int imuStart()
{
    I2C_BUSY_BEGIN();                            // yoklama ve başlatma boyunca enkoder SPI'si beklesin
    for (int attempt = 0; attempt < 3; attempt++) {
        board_watchdog_feed();
        if (attempt) delay(300);
        if (!board_i2c_probe(0x4A)) {
            // ACK yok: önce STM32 I2C çevre birimini sıfırla, bir daha yokla
            board_i2c_reinit();
            if (!board_i2c_probe(0x4A)) continue;
        }
        if (bno085_init((uint32_t)params_val(P_IMU_RATE_HZ))) { I2C_BUSY_END(); return 1; }
    }
    I2C_BUSY_END();
    return 0;
}

// ---------------- nişan katmanı yardımcıları ----------------
static uint8_t  s_aimHold = 0;
static uint8_t  s_aimRefReset = 1;   // otomatik moda girişte kaçma koruması referansı yenilensin
static uint8_t  s_tiltZeroed = 0;
static uint32_t s_aimBigMs = 0;
static uint32_t s_lastAimMs = 0;

// Tilt sıfırı: taban düzken IMU'nun ölçtüğü bakış elevasyonu = tilt açısı
static void tiltFromImu()
{
    float el = pointing_elevation();
    axis_set_here(AX_TILT, el);
    char l[80]; snprintf(l, sizeof l, "tilt acisi IMU'dan alindi: %ld mderece", mdeg(el));
    sendText(l);
    s_tiltZeroed = 1;
}

// Her IMU örneğinde (100 Hz): otomatik modda hız komutları
static void pointingUpdate()
{
    uint32_t now = millis();
    float dt = (s_lastAimMs && now > s_lastAimMs) ? (now - s_lastAimMs) * 0.001f : 0.01f;
    uint32_t gap = s_lastAimMs ? (now - s_lastAimMs) : 10;
    s_lastAimMs = now;
    // Bayat veri koruması (25 Eyl: IMU 15 Hz'e düşünce döngü bayat yönelimle 45° kaçtı): örnek boşluğu 60 ms'i
    // aşarsa bu örnekle sürme, motorları tut; akış düzelince ilk taze örnekle devam et
    if (fsm_mode() == MODE_AUTO && gap > 60) {
        if (!s_aimHold) { s_aimHold = 1; axis_stop(0); axis_stop(1); sendText("nisan: IMU orneklemesi seyrekledi, konum tutuluyor"); }
        pointing_reset_integrators();
        return;
    }
    if (dt > 0.1f) dt = 0.1f;
    // IMU'nun yönelimi ilk raporlarda henüz oturmamış olabiliyor (−42° gibi sıfır alındı): 2 s rapor bekle
    if (!s_tiltZeroed && !axis_busy(AX_TILT) && bno085_count() > 200 && millis() > 3000) tiltFromImu();
    if (fsm_mode() != MODE_AUTO) { s_aimHold = 0; return; }
    if (s_aimHold) { s_aimHold = 0; pointing_reset_integrators(); }
    float tot, side, up; pointing_error(&tot, &side, &up);
    // Tasarım kararı (25 Eyl gece): otomatik mod HİÇBİR açı/hata sınırında kendi kendine vazgeçmez, hep düzeltmeye çalışır.
    // Tek istisna: geometri tanımsızken (başlık zenite çok yakın / devrilmiş) motorlar tutulur, poz düzelince devam eder.
    if (!pointing_pose_ok()) {
        if (!s_aimHold) { s_aimHold = 1; axis_stop(0); axis_stop(1); sendText("nisan: baslik zenite cok yakin, poz duzelene kadar tutuluyor"); }
        pointing_reset_integrators();
        return;
    }
    // Sürücü kapalıyken adım gönderilmez (29 Eyl: sürücüler kapalıyken otomatik takip sürdü, motorlar dönmedi ama
    // sayaçlar döndü: pan 697° kaçtı, tilt sayacı 90° sınırına dayandı ve düzeltme durdu). Vazgeçmez, bekler:
    // sürücüler açılınca kendiliğinden devam eder.
    {
        static uint8_t drvWait = 0;
        if (!board_motor_enabled(AX_PAN) || !board_motor_enabled(AX_TILT)) {
            axis_halt(AX_PAN); axis_halt(AX_TILT);
            pointing_reset_integrators();
            if (!drvWait) { drvWait = 1; sendText("nisan: pan/tilt surucusu kapali, otomatik takip bekliyor (surucu acilinca devam eder)"); }
            return;
        }
        if (drvWait) { drvWait = 0; sendText("nisan: suruculer acik, otomatik takip devam ediyor"); }
    }
    // Kaçma koruması (25 Eyl: pan 175° yanlış yöne döndü, hata 14°'de kaldı, 120° eşiği yakalamadı):
    // hata son 0,5°'nin altına indiğinden beri pan 45° / tilt 30°'den fazla yol aldıysa döngü hedefi kovalayamıyor demektir
    float vp, vt;
    if (pointing_step(dt, axis_vel(AX_PAN), axis_vel(AX_TILT), &vp, &vt)) {
        // eksen katmanının en düşük hızının altındaki istekler sıfır sayılır (çırpınma önlenir)
        if (vp > -0.3f && vp < 0.3f) vp = 0.0f;
        if (vt > -0.4f && vt < 0.4f) vt = 0.0f;
        axis_set_velocity(AX_PAN, vp);
        axis_set_velocity(AX_TILT, vt);
    }
}

// ---------------- sürücü kurulumu ----------------
static void tmcInitAll()
{
    static const char *nm[3] = { "pan", "tilt", "ce" };
    char line[100];
    uint16_t irun  = (uint16_t)params_val(P_MOT_IRUN_MA);
    uint8_t  ihold = (uint8_t)params_val(P_MOT_IHOLD_PCT);
    uint16_t ust   = (uint16_t)params_val(P_MOT_USTEPS);
    float    rs    = params_val(P_TMC_RSENSE);
    for (int i = 0; i < 3; i++) {
        tmc2209_init(&s_tmc[i], (uint8_t)i, 0);      // ayrı hat: hat = eksen, adres hep 0
        if (!tmc2209_probe(&s_tmc[i])) {
            snprintf(line, sizeof line, "tmc %s (hat %d): cevap yok", nm[i], i);
            sendText(line);
            continue;
        }
        int ok = tmc2209_configure(&s_tmc[i], irun, ihold, ust, rs);
        snprintf(line, sizeof line, "tmc %s: surum 0x%02X, ayar %s, cs=%u vsense=%u %u mA %u ustep",
                 nm[i], s_tmc[i].version, ok ? "OK" : "IFCNT uyusmadi", s_tmc[i].cs, s_tmc[i].vsense, irun, ust);
        sendText(line);
    }
}

// ---------------- kesmeler ----------------
static void controlStep();
static void stopAll();
static void onStepTick() { stepgen_tick(); }
// Kontrol adımı doğrudan 1 kHz kesmede koşar: ana döngüdeki I2C ve seri port onu geciktiremez.
static void onCtrlTick()
{
    s_ctrlTicks++;
    uint32_t t0 = micros();
    controlStep();
    uint32_t dt = micros() - t0;
    s_tel.loop_us = (uint16_t)dt;
    if (dt > s_loopMaxUs) s_loopMaxUs = (uint16_t)dt;
    if (dt > 900) s_overruns++;                 // arıza değil: yalnız sayılır (ping çıktısında "dongu asimi")
}

// ---------------- komut işleme ----------------
static float rdF32(const uint8_t *p) { float f; memcpy(&f, p, 4); return f; }
static uint16_t rdU16(const uint8_t *p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }

// Anında dur: mod geçişi, sürücü kapatma, arıza
static void stopAll()
{
    axis_halt_all();
    stepgen_stop_all();
}

static void handleFrame(uint8_t id, const uint8_t *p, uint8_t n)
{
    switch (id) {
    case MSG_PING: {
        uint32_t pl[2] = { millis(), (uint32_t)MERCOM_FW_VERSION };
        sendFrame(MSG_PONG, pl, sizeof pl);
        char line[80];
        snprintf(line, sizeof line, "mod %s, dongu asimi %lu, imu %s (kesilme %lu), spi yeniden baslatma %lu", fsm_mode_name(fsm_mode()),
                 (unsigned long)s_overruns, s_imuOk ? "var" : "yok", (unsigned long)s_imuStalls, (unsigned long)s_spiReinits);
        sendText(line);
        {
            char nv[120];
            int st = params_nv_state();
            if (st > 0) snprintf(nv, sizeof nv, "param kaydi: acilista karttaki kayittan yuklendi (%d adet), bu oturumda %d kez kaydedildi", st, params_nv_saves());
            else        snprintf(nv, sizeof nv, "param kaydi: acilista %s, yazilim varsayilanlari; bu oturumda %d kez kaydedildi",
                                 st == -2 ? "kayit BOZUK" : st == -3 ? "bellek yok" : "kayit yok", params_nv_saves());
            sendText(nv);
        }
#ifdef LORA_SERIAL
        snprintf(line, sizeof line, "lora: %s, cerceve %lu (bozuk %lu), atlanan %lu",
                 loraActive() ? "etkin" : "sessiz", (unsigned long)s_rxLora.rx_frames, (unsigned long)s_rxLora.rx_errors, (unsigned long)s_loraDropped);
        sendText(line);
#endif
        break;
    }
    case MSG_SET_MODE: {
        if (n != 1) { sendAck(id, ACK_BAD_LEN); break; }
        if (p[0] == MODE_AUTO) {
            uint8_t fresh = (s_imuOk && (millis() - bno085_last_ms()) < 300) ? 1 : 0;
            if (!pointing_locked() || !fresh) { sendAck(id, ACK_REFUSED); sendText(pointing_locked() ? "otomatik: IMU verisi yok" : "otomatik: once Kilitle"); break; }
            if (!board_motor_enabled(0) || !board_motor_enabled(1)) { sendAck(id, ACK_REFUSED); sendText("otomatik: pan ve tilt suruculeri acik olmali"); break; }
            if (!pointing_pose_ok()) { sendAck(id, ACK_REFUSED); sendText("otomatik: baslik zenite cok yakin ya da devrilmis (IMU roll/pitch buyuk); sistemi duzeltip anteni 0-80 derece arasina getirin"); break; }
            // CE serbest kalırsa başlık CE ekseninde savrulur (pan yönünde): otomatikte CE hep tutulur
            if (!board_motor_enabled(2)) { board_motor_enable(2, 1); sendText("otomatik: CE surucusu acildi (baslik tutulsun)"); }
        }
        int ok = fsm_request(p[0]);
        if (ok) { stopAll(); pointing_reset_integrators(); s_aimRefReset = 1; }   // mod geçişinde eksenler durur
        sendAck(id, ok ? ACK_OK : ACK_WRONG_MODE);
        break;
    }
    case MSG_JOG: {
        if (n != 5) { sendAck(id, ACK_BAD_LEN); break; }
        if (fsm_mode() != MODE_MANUAL) { sendAck(id, ACK_WRONG_MODE); break; }
        int axis = p[0];
        if (axis > 2) { sendAck(id, ACK_BAD_ARG); break; }
        if (!board_motor_enabled(axis)) { sendAck(id, ACK_REFUSED); break; }
        float dps = rdF32(&p[1]);
        float vmax = params_val(P_VMAX_PAN + axis);
        if (dps >  vmax) dps =  vmax;
        if (dps < -vmax) dps = -vmax;
        axis_set_velocity(axis, dps);
#ifdef LORA_SERIAL
        if (s_outLora && dps != 0.0f) { s_loraJogMs = millis() | 1u; s_loraJogAxis = (int8_t)axis; }
        else if (axis == s_loraJogAxis) { s_loraJogMs = 0; s_loraJogAxis = -1; }
#endif
        sendAck(id, ACK_OK);
        break;
    }
    case MSG_STOP:                              // rampalı dur, bütün eksenler
        for (int i = 0; i < 3; i++) axis_stop(i);
        sendAck(id, ACK_OK);
        break;
    case MSG_GOTO: {
        if (n != 5) { sendAck(id, ACK_BAD_LEN); break; }
        if (fsm_mode() != MODE_MANUAL) { sendAck(id, ACK_WRONG_MODE); break; }
        int axis = p[0];
        if (axis > 2) { sendAck(id, ACK_BAD_ARG); break; }
        if (!board_motor_enabled(axis)) { sendAck(id, ACK_REFUSED); break; }
        axis_set_target(axis, rdF32(&p[1]));
        sendAck(id, ACK_OK);
        break;
    }
    case MSG_OSC: {
        // u8 axis, f32 A, f32 B, u16 bekleme ms -> başlat;  yalnız u8 axis -> durdur
        if (n != 1 && n != 11) { sendAck(id, ACK_BAD_LEN); break; }
        int axis = p[0];
        if (axis > 2) { sendAck(id, ACK_BAD_ARG); break; }
        if (n == 1) { axis_osc_stop(axis); sendAck(id, ACK_OK); break; }
        if (fsm_mode() != MODE_MANUAL) { sendAck(id, ACK_WRONG_MODE); break; }
        if (!board_motor_enabled(axis)) { sendAck(id, ACK_REFUSED); break; }
        axis_osc(axis, rdF32(&p[1]), rdF32(&p[5]), rdU16(&p[9]));
        sendAck(id, ACK_OK);
        break;
    }
    case MSG_AXIS_ZERO: {
        // Şu anki konumu deg kabul et: adım sayacı ve enkoder sıfırı birlikte
        if (n != 5) { sendAck(id, ACK_BAD_LEN); break; }
        int axis = p[0];
        if (axis > 2) { sendAck(id, ACK_BAD_ARG); break; }
        float deg = rdF32(&p[1]);
        axis_set_here(axis, deg);
        noInterrupts();
        float encNow = s_encDegCont[axis] + params_val(P_ENC_OFF_PAN + axis);   // kaymasız ham açı
        params_set(P_ENC_OFF_PAN + axis, encNow - deg);
        interrupts();
        sendAck(id, ACK_OK);
        break;
    }
    case MSG_AXIS_INFO: {
        static const char *nm[3] = { "pan", "tilt", "ce" };
        char line[150];
        for (int i = 0; i < 3; i++) {
            snprintf(line, sizeof line, "eksen %s: konum=%ld hedef=%ld hiz=%ld mderece  %s%s salinim=%d duzeltme=%lu  vmax=%ld amax=%ld sinir=%ld..%ld dps=%ld u",
                     nm[i], mdeg(axis_pos(i)), mdeg(axis_target(i)), mdeg(axis_vel(i)),
                     axis_busy(i) ? "HAREKETTE" : "duruyor", axis_at_limit(i) ? " SINIRDA" : "",
                     axis_osc_on(i), (unsigned long)axis_resyncs(i),
                     (long)params_val(P_VMAX_PAN + i), (long)params_val(P_AMAX_PAN + i),
                     (long)params_val(P_PAN_MIN + 2 * i), (long)params_val(P_PAN_MAX + 2 * i), (long)(params_val(P_DPS_PAN + i) * 1e6f));
            sendText(line);
            int v; float e; axis_cl_info(i, &v, &e);
            snprintf(line, sizeof line, "eksen %s kapali cevrim: %s, enkoder %s (guven %u), sayac-enkoder farki %ld mderece, takilma %lu",
                     nm[i], params_val(P_CL_PAN + i) != 0.0f ? "acik" : "kapali", v ? "gecerli" : "GECERSIZ",
                     (unsigned)s_encTrust[i], mdeg(e), (unsigned long)axis_stalls(i));
            sendText(line);
        }
        break;
    }
    case MSG_LOCK: {
        uint8_t fresh = (s_imuOk && (millis() - bno085_last_ms()) < 300) ? 1 : 0;
        if (!fresh || !pointing_lock()) { sendAck(id, ACK_REFUSED); sendText("kilit: IMU verisi yok"); break; }
        // Kilit anında IMU oturmuş ve anten hedefte: tilt sıfırını da buradan yenile (açılıştaki ilk örnekler bazen yanlış)
        if (!axis_busy(AX_TILT)) tiltFromImu();
        char l[80]; snprintf(l, sizeof l, "kilit alindi: azimut %ld, elevasyon %ld mderece", mdeg(pointing_azimuth()), mdeg(pointing_elevation()));
        sendText(l);
        sendAck(id, ACK_OK);
        break;
    }
    case MSG_UNLOCK:
        pointing_unlock();
        if (fsm_mode() == MODE_AUTO) { fsm_request(MODE_MANUAL); axis_stop(0); axis_stop(1); sendText("kilit birakildi, manuel moda donuldu"); }
        sendAck(id, ACK_OK);
        break;
    case MSG_TILT_FROM_IMU:
        if (!pointing_have_imu()) { sendAck(id, ACK_REFUSED); break; }
        tiltFromImu();
        sendAck(id, ACK_OK);
        break;
    case MSG_LASER: {
        if (n != 1) { sendAck(id, ACK_BAD_LEN); break; }
        s_laserOn = p[0] ? 1 : 0;
        board_laser(s_laserOn);
        sendAck(id, ACK_OK);
        break;
    }
    case MSG_ENABLE: {
        // 1 bayt: u8 on -> üç sürücü birden.  2 bayt: u8 eksen, u8 on -> tek sürücü
        if (n == 1) {
            if (!p[0]) stopAll();
            board_motors_enable(p[0] ? 1 : 0);
        } else if (n == 2) {
            if (p[0] > 2) { sendAck(id, ACK_BAD_ARG); break; }
            if (!p[1]) axis_halt(p[0]);
            board_motor_enable(p[0], p[1] ? 1 : 0);
        } else { sendAck(id, ACK_BAD_LEN); break; }
        sendAck(id, ACK_OK);
        break;
    }
    case MSG_PARAM_GET: {
        if (n != 2) { sendAck(id, ACK_BAD_LEN); break; }
        uint16_t pid = rdU16(p);
        float v = 0.0f;
        int ok = params_get(pid, &v);
        uint8_t out[7]; memcpy(out, &pid, 2); memcpy(out + 2, &v, 4); out[6] = ok ? 1 : 0;
        sendFrame(MSG_PARAM_VALUE, out, 7);
        break;
    }
    case MSG_PARAM_SET: {
        if (n != 6) { sendAck(id, ACK_BAD_LEN); break; }
        uint16_t pid = rdU16(p);
        float v = rdF32(p + 2);
        int ok = params_set(pid, v);
        sendAck(id, ok ? ACK_OK : ACK_BAD_ARG);
        break;
    }
    case MSG_PARAM_SAVE: {
        char line[96];
        if (n == 1 && p[0] == 0) {
            int r = params_clear();
            sendAck(id, r == 0 ? ACK_OK : ACK_NOT_IMPL);
            sendText(r == 0 ? "param kaydi silindi: sonraki acilista yazilim varsayilanlari" : "param kaydi silinemedi");
            break;
        }
        if (n != 0) { sendAck(id, ACK_BAD_LEN); break; }
        board_watchdog_feed();
        uint32_t t0 = millis();
        int r = params_save();
        board_watchdog_feed();
        sendAck(id, r == 0 ? ACK_OK : ACK_NOT_IMPL);
        if (r == 0) snprintf(line, sizeof line, "parametreler karta kalici kaydedildi (%d adet, %lu ms)", (int)PARAM_COUNT, (unsigned long)(millis() - t0));
        else        snprintf(line, sizeof line, "param kaydi BASARISIZ (kod %d, %lu ms)", r, (unsigned long)(millis() - t0));
        sendText(line);
        break;
    }
    case MSG_TELEM_RATE: {
        if (n != 2) { sendAck(id, ACK_BAD_LEN); break; }
        params_set(P_TELEM_HZ, (float)rdU16(p));
        sendAck(id, ACK_OK);
        break;
    }
    case MSG_CLEAR_FAULT:
        fsm_clear_fault();
        board_led_fault(0);
        sendAck(id, ACK_OK);
        break;
    case MSG_IMU_DIAG: {
        char line[120];
        // 1 baytlık yük: sensör reset sebebi izlemeyi aç (1) / kapat (0). Yük yoksa yalnız durum yazılır.
        if (n >= 1) bno085_cause_track(p[0] ? 1 : 0);
        {
            uint32_t hc[6]; bno085_cause_hist(hc);
            static const char *cn[6] = { "cevap yok", "GUC ACILISI", "yazilim reseti", "SENSORUN IC BEKCI KOPEGI", "RST PINI", "diger" };
            int lc = bno085_last_cause();
            snprintf(line, sizeof line, "imu reset sebebi: izleme %s, guc=%lu yazilim=%lu bekci=%lu rstpin=%lu diger=%lu cevapsiz=%lu, son: %s",
                     bno085_cause_tracking() ? "acik" : "kapali", (unsigned long)hc[1], (unsigned long)hc[2], (unsigned long)hc[3],
                     (unsigned long)hc[4], (unsigned long)hc[5], (unsigned long)hc[0], (lc >= 0 && lc <= 5) ? cn[lc] : "yok");
            sendText(line);
            snprintf(line, sizeof line, "kart: acik kalma %lu s, son kart reseti: %s, imu kesilme %lu, enk okuma %lu, i2c icin atlanan %lu",
                     (unsigned long)(millis() / 1000), s_bootCause, (unsigned long)s_imuStalls,
                     (unsigned long)s_encReads, (unsigned long)s_encBusySkips);
            sendText(line);
            uint32_t ic[10]; bno085_i2c_counts(ic);
            snprintf(line, sizeof line, "i2c: bas=%lu gov=%lu buyuk=%lu kanal=%lu yaz=%lu pltf=%lu nack=%lu alf=%lu fef=%lu paket=%lu",
                     (unsigned long)ic[0], (unsigned long)ic[1], (unsigned long)ic[2], (unsigned long)ic[3], (unsigned long)ic[4],
                     (unsigned long)ic[5], (unsigned long)ic[6], (unsigned long)ic[7], (unsigned long)ic[8], (unsigned long)ic[9]);
            sendText(line);
        }
        float q[4]; bno085_quat(q);
        snprintf(line, sizeof line, "imu: %s, rapor=%lu, jiro=%lu, reset=%lu, son=%lu ms once, rpy(mderece)=%ld %ld %ld",
                 bno085_ok() ? "hazir" : "BULUNAMADI", (unsigned long)bno085_count(), (unsigned long)bno085_gyro_count(), (unsigned long)bno085_resets(),
                 (unsigned long)(millis() - bno085_last_ms()),
                 mdeg(s_tel.rpy_deg[0]), mdeg(s_tel.rpy_deg[1]), mdeg(s_tel.rpy_deg[2]));
        sendText(line);
        snprintf(line, sizeof line, "imu quat x10000: w=%ld x=%ld y=%ld z=%ld",
                 (long)(q[0] * 10000.0f), (long)(q[1] * 10000.0f), (long)(q[2] * 10000.0f), (long)(q[3] * 10000.0f));
        sendText(line);
        float g[3]; bno085_gravity(g);
        snprintf(line, sizeof line, "imu yercekimi (cm/s2): x=%ld y=%ld z=%ld  (yukari bakan eksen +981)",
                 (long)(g[0] * 100), (long)(g[1] * 100), (long)(g[2] * 100));
        sendText(line);
        float w[3]; bno085_gyro(w);
        snprintf(line, sizeof line, "imu jiroskop (mderece/s): x=%ld y=%ld z=%ld",
                 (long)(w[0] * 57295.8f), (long)(w[1] * 57295.8f), (long)(w[2] * 57295.8f));
        sendText(line);
        break;
    }
    case MSG_TMC_DIAG: {
        char line[140];
        static const char *nm[3] = { "pan", "tilt", "ce" };
        for (int i = 0; i < 3; i++) {
            tmc2209_t *d = &s_tmc[i];
            if (!d->present) { snprintf(line, sizeof line, "tmc %s: hatta yok (err=%lu)", nm[i], (unsigned long)d->err_count); sendText(line); continue; }
            int ok = tmc2209_read_status(d);
            uint32_t s = d->drv_status;
            snprintf(line, sizeof line, "tmc %s: v=0x%02X ayar=%d ifcnt=%u cs=%u cs_act=%lu%s%s%s%s%s%s err=%lu%s",
                     nm[i], d->version, d->configured, d->ifcnt, d->cs, (unsigned long)TMC_ST_CS_ACTUAL(s),
                     (s & TMC_ST_OTPW) ? " SICAK" : "", (s & TMC_ST_OT) ? " ASIRI-ISI" : "",
                     (s & (TMC_ST_S2GA | TMC_ST_S2GB | TMC_ST_S2VSA | TMC_ST_S2VSB)) ? " KISA-DEVRE" : "",
                     (s & (TMC_ST_OLA | TMC_ST_OLB)) ? " bobin-acik" : "",
                     (s & TMC_ST_STEALTH) ? " stealth" : "", (s & TMC_ST_STST) ? " duruyor" : "",
                     (unsigned long)d->err_count, ok ? "" : " (status okunamadi)");
            sendText(line);
        }
        break;
    }
    case MSG_TMC_INIT:
        tmcInitAll();
        sendAck(id, ACK_OK);
        break;
    case MSG_I2C_SCAN: {
        uint8_t found[16];
        I2C_BUSY_BEGIN();
        int n = board_i2c_scan(found, 16);
        I2C_BUSY_END();
        char line[120];
        int k = snprintf(line, sizeof line, "i2c tarama: %d cihaz", n);
        for (int i = 0; i < n && k < (int)sizeof line - 6; i++)
            k += snprintf(line + k, sizeof line - k, " 0x%02X", found[i]);
        sendText(line);
        int scl, sda, rst; board_i2c_levels(&scl, &sda, &rst);
        snprintf(line, sizeof line, "i2c hat testi (asagi cekili): SCL=%d SDA=%d RST=%d  (1 = kablo module ulasiyor ve modul besleniyor, 0 = kablo kopuk)",
                 scl, sda, rst);
        sendText(line);
        if (n == 0) {
            int es, ed; board_i2c_activity(&es, &ed);
            uint8_t f2[4];
            int nSlow = board_i2c_scan_slow(f2, 4);
            snprintf(line, sizeof line, "i2c ek tani: 20 ms'de SCL %d, SDA %d gecis; 50 kHz tarama %d cihaz", es, ed, nSlow);
            sendText(line);
            if (ed > 4) sendText("SDA kendiliginden kipirdiyor: cip CANLI ama UART modunda (P0/P1 yuksek). Guc keserken P0/P1 bos olmali.");
            else if (nSlow) sendText("50 kHz'de bulundu: hat kalitesi cok sinirda, kablolari kisaltin ya da 4.7k pull-up ekleyin.");
            else sendText("i2c: hic cevap yok -> SCL D15, SDA D14, VIN, GND, RST kablolarini kontrol");
        }
        break;
    }
    case MSG_YAW_ZERO: {
        float rpy[3]; bno085_rpy(rpy);
        s_yawZero = rpy[2];
        sendAck(id, ACK_OK);
        break;
    }
    case MSG_IMU_PINTEST: {
        char line[120];
        uint8_t found[4];
        board_watchdog_feed();
        board_imu_rst_write(0);
        delay(400);
        board_watchdog_feed();
        int nLow = board_i2c_scan(found, 4);
        int addrLow = nLow ? found[0] : 0;
        board_imu_rst_write(1);
        delay(600);
        board_watchdog_feed();
        int nHigh = board_i2c_scan(found, 4);
        int addrHigh = nHigh ? found[0] : 0;
        snprintf(line, sizeof line, "pin testi: A0 DUSUKKEN %d cihaz (0x%02X), A0 YUKSEKKEN %d cihaz (0x%02X)", nLow, addrLow, nHigh, addrHigh);
        sendText(line);
        if (nLow && !nHigh)       sendText("SONUC: kablo RST'de degil, P0 ya da P1'de. A0 yuksek olunca modul I2C'den cikiyor.");
        else if (!nLow && nHigh)  sendText("SONUC: kablo RST'de, dogru. Modul I2C'de gorunuyor.");
        else if (!nLow && !nHigh) {
            board_watchdog_feed();
            int nSw = board_i2c_scan_swapped(found, 4);
            snprintf(line, sizeof line, "SCL/SDA ters varsayilarak tarama: %d cihaz (0x%02X)", nSw, nSw ? found[0] : 0);
            sendText(line);
            if (nSw) sendText("SONUC: SCL ve SDA kablolari yer degistirmis. D15'e SCL, D14'e SDA gelmeli.");
            else     sendText("SONUC: her iki eslemede de cevap yok: modul besleniyor ama I2C'de degil. P0/P1 jumper'i, DI pini ya da modul arizasi.");
        }
        else                      sendText("SONUC: A0 kablosunun modulde etkisi yok, RST'ye bagli degil; modul gorunuyor.");
        s_imuOk = 0;   // yeniden başlatma döngüsü sensörü bulursa açar
        break;
    }
    case MSG_IMU_INIT:
        // Açılıştaki diziyi taklit et: çevre birimi sıfırla, hattı 300 ms boşta bırak, sonra başlat.
        // (Arayüzden yalnız imuStart yetmiyordu, kart reseti düzeltiyordu.)
        s_imuOk = 0;
        board_i2c_reinit();
        delay(300);
        board_watchdog_feed();
        s_imuOk = (uint8_t)imuStart();
        sendText(s_imuOk ? "imu: BNO085 bulundu, GRV raporu acik" : "imu: BNO085 BULUNAMADI, 2 s'de bir yeniden denenecek");
        sendAck(id, s_imuOk ? ACK_OK : ACK_REFUSED);
        break;
    case MSG_STEP_BLINK: {
        int axis = (n >= 1) ? p[0] : 0;
        if (axis > 2) { sendAck(id, ACK_BAD_ARG); break; }
        stopAll();
        sendText("STEP pini testi: 20 s boyunca 3 s YUKSEK (3,3 V) / 3 s DUSUK (0 V); multimetre ile D7/D5/D3 ve surucu STEP pinini olc");
        for (int i = 0; i < 20; i++) {
            board_step_write(axis, (i / 3) % 2 == 0 ? 1 : 0);
            for (int k = 0; k < 10; k++) { board_watchdog_feed(); delay(100); }
        }
        board_step_write(axis, 0);
        sendText("STEP pini testi bitti");
        sendAck(id, ACK_OK);
        break;
    }
    case MSG_TMC_TXCHK: {
        static const char *nm[3] = { "pan D1", "tilt D42", "ce D32" };
        char line[120];
        for (int ch = 0; ch < 3; ch++) {
            int ok = board_tmc_uart_tx_check(ch);
            int lvl = board_tmc_uart_line_level(ch);
            snprintf(line, sizeof line, "uart %s: verici %s, hat bosta %s", nm[ch],
                     ok ? "OK (pinde darbe goruldu)" : "HATA (pinde darbe yok)", lvl ? "yuksek" : "DUSUK");
            sendText(line);
        }
        break;
    }
    case MSG_TMC_LOOP: {
        int ch = (n >= 1 && p[0] <= 2) ? p[0] : 0;
        int rxok = 0, rxb = -1, txok = 0;
        board_tmc_uart_selftest(ch, &rxok, &rxb, &txok);
        char line[120];
        snprintf(line, sizeof line, "uart oz sinama hat %d (pin<->A0 tel): alici %s (okunan %d, beklenen 85), verici %s",
                 ch, rxok ? "OK" : "HATA", rxb, txok ? "OK" : "HATA");
        sendText(line);
        if (!rxok && !txok) sendText("  ikisi de hata: tel takili degil ya da yanlis pinler (pan D1, tilt D42, ce D32; A0 = CN9)");
        break;
    }
    case MSG_TMC_RAW: {
        int ch = (n >= 1 && p[0] <= 2) ? p[0] : 0;
        uint8_t addr = (n >= 2) ? (p[1] & 3) : 0;
        uint8_t req[4] = { 0x05, addr, 0x06, 0 };
        req[3] = tmc2209_crc(req, 3);
        board_tmc_uart_flush_rx(ch);
        board_tmc_uart_send(ch, req, 4);
        uint8_t rx[32]; int k = 0;
        uint32_t t0 = millis();
        while (millis() - t0 < 30 && k < (int)sizeof rx) { int c = board_tmc_uart_read_byte(ch); if (c >= 0) rx[k++] = (uint8_t)c; }
        char line[140]; int m = snprintf(line, sizeof line, "tmc ham (hat %d, adres %u) gonderilen 05 %02X 06 %02X, alinan %d bayt:", ch, addr, addr, req[3], k);
        for (int i = 0; i < k && m < (int)sizeof line - 4; i++) m += snprintf(line + m, sizeof line - m, " %02X", rx[i]);
        sendText(line);
        {
            int lvl = board_tmc_uart_line_level(ch);
            snprintf(line, sizeof line, "  UART hatti bosta: %s", lvl ? "YUKSEK (normal)" : "DUSUK (kisa devre / surucu hatti tutuyor / GND'ye bagli)");
            sendText(line);
        }
        if (k == 0) sendText("  hic cevap yok -> 24 V ya da VIO yok, tel modulun UART pinine ulasmiyor, ya da adres uyusmuyor");
        else if (k >= 8) sendText("  cevap var: surucu konusuyor");
        else sendText("  eksik/bozuk cevap: baud ya da hat kalitesi");
        break;
    }
    case MSG_STEP_MOVE: {
        if (n != 9) { sendAck(id, ACK_BAD_LEN); break; }
        if (fsm_mode() != MODE_MANUAL) { sendAck(id, ACK_WRONG_MODE); break; }
        int axis = p[0];
        if (axis > 2) { sendAck(id, ACK_BAD_ARG); break; }
        if (!board_motor_enabled(axis)) { sendAck(id, ACK_REFUSED); break; }
        int32_t steps; memcpy(&steps, &p[1], 4);
        float sps = rdF32(&p[5]);
        if (sps < 1.0f) sps = 1.0f;
        if (sps > 8000.0f) sps = 8000.0f;
        axis_halt(axis);
        stepgen_move(axis, steps, sps);
        sendAck(id, ACK_OK);
        break;
    }
    case MSG_SYS_RESET:
        sendAck(id, ACK_OK);
        Serial.flush();
        delay(50);
        board_reset();
        break;
#ifdef LORA_SERIAL
    case MSG_LORA_CFG: {
        // E22 ayar modu (M1=1, M0=0; UART 9600 8N1): C1 00 09 oku, 9 bayt yük verilmişse önce C0 00 09 ile yaz (kalıcı)
        // 1 bayt yük: o kadar test mesajı LoRa'dan yayınla (yer istasyonu tanısı için, 300 ms arayla)
        char line[120];
        if (n == 1) {
            for (int i = 0; i < p[0]; i++) {
                snprintf(line, sizeof line, "MERCOM lora test %d/%d", i + 1, p[0]);
                LORA_SERIAL.write(LORA_HEDEF, 3);
                LORA_SERIAL.write((const uint8_t *)line, strlen(line)); LORA_SERIAL.write((const uint8_t *)"\r\n", 2);
                delay(300); board_watchdog_feed();
            }
            sendAck(id, ACK_OK);
            break;
        }
        uint8_t cmd[12], rx[16]; int nrx = 0;
        digitalWrite(PN_LORA_M0, LOW); digitalWrite(PN_LORA_M1, HIGH);
        delay(80); board_watchdog_feed();
        while (LORA_SERIAL.available()) LORA_SERIAL.read();
        if (n == 9) {
            cmd[0] = 0xC0; cmd[1] = 0x00; cmd[2] = 0x09; memcpy(&cmd[3], p, 9);
            LORA_SERIAL.write(cmd, 12); LORA_SERIAL.flush();
            uint32_t t0 = millis(); while (millis() - t0 < 300) { while (LORA_SERIAL.available() && nrx < (int)sizeof rx) rx[nrx++] = (uint8_t)LORA_SERIAL.read(); }
            nrx = 0; delay(50); board_watchdog_feed();
        }
        cmd[0] = 0xC1; cmd[1] = 0x00; cmd[2] = 0x09;
        LORA_SERIAL.write(cmd, 3); LORA_SERIAL.flush();
        uint32_t t0 = millis();
        while (millis() - t0 < 300 && nrx < 12) { while (LORA_SERIAL.available() && nrx < (int)sizeof rx) rx[nrx++] = (uint8_t)LORA_SERIAL.read(); }
        digitalWrite(PN_LORA_M1, LOW); delay(80); board_watchdog_feed();   // şeffaf moda dön
        while (LORA_SERIAL.available()) LORA_SERIAL.read();
        int k = snprintf(line, sizeof line, "lora ayar (%d bayt):", nrx);
        for (int i = 0; i < nrx && k < (int)sizeof line - 4; i++) k += snprintf(line + k, sizeof line - k, " %02X", rx[i]);
        if (nrx == 0) snprintf(line + k, sizeof line - k, " CEVAP YOK (M1/TX/RX/besleme?)");
        sendText(line);
        sendAck(id, nrx >= 12 ? ACK_OK : ACK_REFUSED);
        break;
    }
#endif
    case MSG_ENC_DIAG: {
        // AGC 0..255: ~128 civari iyi, dusukse miknatis cok yakin, yuksekse uzak
        char line[150];
        static const char *nm[3] = { "pan", "tilt", "ce" };
        // Okumayı kesme yapsın, en fazla birkaç ms bekle
        s_encDiagDone = 0;
        s_encDiagReq = 1;
        uint32_t t0 = millis();
        while (!s_encDiagDone && millis() - t0 < 20) { }
        for (int i = 0; i < 3; i++) {
            int ok = s_encDiagDone ? s_encDiagOk[i] : 0;
            // AGC: 0 = alan cok guclu, 255 = alan cok zayif; 60-200 arasi iyi. OCF=1 hazir demek.
            snprintf(line, sizeof line, "enc %s: raw=%u agc=%u OCF=%d COF=%d yakin=%d uzak=%d ok=%lu err=%lu spi=0x%04X hatali=0x%04X parite=%lu EF=%lu sicrama=%lu%s",
                     nm[i], s_enc[i].raw, s_enc[i].agc,
                     (s_enc[i].diag >> 0) & 1, (s_enc[i].diag >> 1) & 1,
                     (s_enc[i].diag >> 2) & 1, (s_enc[i].diag >> 3) & 1,
                     (unsigned long)s_enc[i].ok_count, (unsigned long)s_enc[i].err_count,
                     s_enc[i].last_rx, s_enc[i].last_bad, (unsigned long)s_enc[i].n_parity, (unsigned long)s_enc[i].n_ef,
                     (unsigned long)s_encGlitch[i], ok ? "" : " (diag okunamadi)");
            sendText(line);
        }
        break;
    }
    default:
        sendAck(id, ACK_BAD_ARG);
    }
}

#ifdef LORA_SERIAL
// LoRa'dan gelen çerçeve: aynı işleyici, cevaplar LoRa'ya
static void handleFrameLora(uint8_t id, const uint8_t *p, uint8_t n)
{
    s_loraLastRxMs = millis();
    s_outLora = 1;
    handleFrame(id, p, n);
    s_outLora = 0;
}
#endif

// ---------------- kontrol döngüsü, 1 kHz ----------------
static uint8_t s_encAllOk = 0;

// Enkoder motor milinde: ham açı sarmalarını sayıp motor açısını sürekli tutar,
// redüktör oranına bölüp çıkış açısına çevirir. Açılıştaki tur belirsizliği (72° dilimi)
// sonra IMU ile çözülecek; şimdilik açılış konumu 0 kabul edilir.
// (eski tur sayacı kaldırıldı: s_encCont sürekli sayım)
static uint16_t s_encPrevRaw[3] = { 0, 0, 0 };
static uint8_t  s_encHaveRaw[3] = { 0, 0, 0 };

// Sürekli sayım: her geçerli okumada ±yarım tur içine sarılmış fark eklenir. Paritesi tutan ama bozuk bir okuma
// (25 Eyl: pan'da tur sayacını 36° kaydırdı, kapalı çevrim 60 kez "düzeltti") tur sayacını bozmasın diye
// fiziksel olarak imkânsız sıçramalar atılır: motor mili 1 ms'de en fazla ~2,3° (10 000 adım/s) döner.

static float encToDeg(int axis, uint16_t raw)
{
    static uint32_t lastOkTick[3] = { 0, 0, 0 };
    if (!s_encHaveRaw[axis]) { s_encHaveRaw[axis] = 1; s_encPrevRaw[axis] = raw; s_encCont[axis] = 0; lastOkTick[axis] = s_ctrlTicks; }
    int32_t d = (int32_t)raw - (int32_t)s_encPrevRaw[axis];
    if (d >  AS5048A_CPR / 2) d -= AS5048A_CPR;
    if (d < -AS5048A_CPR / 2) d += AS5048A_CPR;
    // İzin verilen sıçrama son GEÇERLİ okumadan beri geçen SÜREYLE orantılı (29 Eyl): I2C senkronu yüzünden bir eksen
    // 9–20 ms okunmayabiliyor; eski "okuma sayısı" hesabı hızlı pan hareketini bozuk okuma sanıp enkoder güvenini sıfırlıyordu
    uint32_t el = s_ctrlTicks - lastOkTick[axis];
    if (el < 1) el = 1;
    if (el > 1000) el = 1000;
    int32_t izin = 120 * (int32_t)el + 200;                         // ~2,6° motor / ms + pay
    if (d > izin || d < -izin) {                                  // bozuk okuma: örneği at, sayacı bozma
        s_encGlitch[axis]++;
        if (s_encSinceOk[axis] < 1000) s_encSinceOk[axis]++;
        return s_encDegCont[axis];
    }
    s_encCont[axis] += d;
    s_encPrevRaw[axis] = raw;
    s_encSinceOk[axis] = 1;
    lastOkTick[axis] = s_ctrlTicks;
    float motorDeg = (float)s_encCont[axis] * (360.0f / (float)AS5048A_CPR);
    float ratio = params_val(P_ENC_RATIO_PAN + axis);
    if (ratio <= 0.0f) ratio = 1.0f;
    return motorDeg / ratio * params_val(P_ENC_DIR_PAN + axis) - params_val(P_ENC_OFF_PAN + axis);   // sürekli, sarmalanmamış
}

static float wrapPan(int axis, float deg)
{
    if (axis != 0) return deg;                 // tilt ve CE ham kalsın: eksi değer testte bilgi verir
    while (deg >= 360.0f) deg -= 360.0f;       // pan gösterimi 0..360 (rapora uygun)
    while (deg < 0.0f)    deg += 360.0f;
    return deg;
}

static void controlStep()
{
    // Adım 3: enkoderler her çevrimde okunur. Cevap vermeyen enkoder (takılı değil) 50 tikte bir denenir,
    // yazılım SPI her çerçeve için ~40 µs harcadığından döngü süresi boşa dolmasın.
    static uint16_t encSkip[3] = { 0, 0, 0 };
    uint8_t allOk = 1;
#if defined(ARDUINO_TEENSY41)
    // Teensy'de yazılım SPI 5 µs yarım saatle sürülür: bir açı okuması ~350 µs. Üçü bir tike sığmaz;
    // her tikte sırayla TEK enkoder okunur (her enkoder 333 Hz). Okunmayan eksenlerin son değeri korunur.
    static uint8_t encTurn = 0;
    static uint8_t encOk[3] = { 0, 0, 0 };
    encTurn = (uint8_t)((encTurn + 1) % 3);
#endif
    // 29 Eyl deneyi: basit BNO kodunda 6 dk 0 reset, ana yazılımda dakikada 5–9. Şüpheli: enkoder yazılım SPI'sinin
    // güçlü/hızlı kenarları I2C kablosuna sızıyor. enc_read 0 iken enkoderlere (açı, tanı, takılma kontrolü) hiç dokunulmaz.
    const uint8_t encRead = params_val(P_ENC_READ) > 0.5f ? 1 : 0;
    // Çözüm: I2C aktarımı sürerken enkoder okunmaz (o tik atlanır, son değerler korunur). Ana döngü sensör okumaları
    // arasında imu_poll_us kadar boşluk bırakır; enkoderler o boşluklarda okunur.
    const uint8_t busySkip = (params_val(P_I2C_SYNC) > 0.5f && g_i2c_busy) ? 1 : 0;
    if (busySkip) s_encBusySkips++;
    if (encRead && !busySkip) {
    for (int i = 0; i < 3; i++) {
#if defined(ARDUINO_TEENSY41)
        if (i != encTurn) { if (!encOk[i]) allOk = 0; continue; }
        encOk[i] = 0;
#endif
        if (s_enc[i].fail_streak >= 50) {
            if (++encSkip[i] < 50) { allOk = 0; continue; }
            encSkip[i] = 0;
        }
        if (as5048a_read_angle(&s_enc[i])) {
            uint32_t g0 = s_encGlitch[i];
            s_encReads++;
            s_encDegCont[i] = encToDeg(i, s_enc[i].raw);
            s_tel.enc_deg[i] = wrapPan(i, s_encDegCont[i]);
            // Güven: mıknatıs menzilde ve okuma sıçramasız ise sayaç artar; 1 s art arda sağlıklı olmadan kapalı çevrime verilmez
            // (25 Eyl: ortak tel çıkınca AGC 255 + rastgele açı; kapalı çevrim çöp değerle tilt'i 12° sıçrattı)
            if (s_encMagOk[i] && s_encGlitch[i] == g0) { if (s_encTrust[i] < 1000) s_encTrust[i]++; }
            else s_encTrust[i] = 0;
            axis_enc_update(i, s_encDegCont[i], s_encTrust[i] >= 1000 ? 1 : 0);
#if defined(ARDUINO_TEENSY41)
            encOk[i] = 1;
#endif
        } else {
            if (s_encSinceOk[i] < 1000) s_encSinceOk[i]++;
            s_encTrust[i] = 0;
            axis_enc_update(i, s_encDegCont[i], 0);
            allOk = 0;
            // Enkoder okunamıyorsa arıza DEĞİL (25 Eyl: elle çevirince kablo oynayıp arıza veriyordu).
            // Nişan döngüsü IMU ile çalışır; kapalı çevrim bu eksende kendiliğinden devre dışı kalır (valid=0).
            if (s_enc[i].fail_streak == 200) s_encWarn[i] = 1;   // 200 ms üst üste: ana döngü bir kez uyarı yazar
        }
    }
    s_encAllOk = allOk;

    if (s_encDiagReq) {
        for (int i = 0; i < 3; i++) s_encDiagOk[i] = (uint8_t)as5048a_read_diag(&s_enc[i]);
        s_encDiagReq = 0;
        s_encDiagDone = 1;
    }

    // Mıknatıs sağlığı: 200 ms'de bir eksen (her eksen 600 ms'de bir) tanı yazmacı okunur.
    // AGC 255 / uzak bayrağı varken açı rastgeledir (25 Eyl: pan'da 587 bozuk okuma, sahte düzeltmeler)
    static uint16_t magTick = 0; static uint8_t magAx = 0;
    if (++magTick >= 200) {
        magTick = 0;
        if (as5048a_read_diag(&s_enc[magAx])) {
            int ocf = (s_enc[magAx].diag >> 0) & 1, uzak = (s_enc[magAx].diag >> 3) & 1;
            s_encMagOk[magAx] = (ocf && !uzak && s_enc[magAx].agc < 250) ? 1 : 0;
        }
        magAx = (uint8_t)((magAx + 1) % 3);
    }

    // SPI takılma tespiti: her 500 ms'de pan enkoderinin açı ve tanı yazmaçlarını art arda oku.
    // Canlı bir çip iki farklı yazmaç için aynı ham kelimeyi veremez; aynıysa çevre birimi
    // son kelimeyi tekrarlıyor demektir, SPI'yı kapatıp aç.
    static uint16_t stuckTick = 0;
    if (++stuckTick >= 500) {
        stuckTick = 0;
        uint16_t a = s_enc[0].last_rx;
        as5048a_read_diag(&s_enc[0]);
        uint16_t d = s_enc[0].last_rx;
        if (a == d && a != 0xFFFF && a != 0x0000) {
            board_spi_reinit();
            s_spiReinits++;
        }
    }
    } else if (!encRead) {
        s_encAllOk = 0;
        for (int i = 0; i < 3; i++) { s_encTrust[i] = 0; axis_enc_update(i, s_encDegCont[i], 0); }
    }

    // Eksen katmanı: profil, sınırlar, salınım, kapalı çevrim
    uint8_t m = fsm_mode();
    if (m != MODE_MANUAL && m != MODE_AUTO) {
        if (axis_busy(0) || axis_busy(1) || axis_busy(2) ||
            stepgen_get_rate(0) != 0.0f || stepgen_get_rate(1) != 0.0f || stepgen_get_rate(2) != 0.0f)
            stopAll();
    } else {
        axis_tick();
    }
    // Nişan katmanı (IMU ve kinematik) buraya eklenecek
    for (int i = 0; i < 3; i++) {
        float pos = axis_pos(i);
        s_tel.vel_dps[i] = axis_vel(i);
        s_tel.cmd_deg[i] = wrapPan(i, pos);
        s_tel.err_deg[i] = pos - s_encDegCont[i];      // adım kaçırma göstergesi (sürekli açılar)
    }
}

static void sendTelemetry()
{
    // Kesme s_tel'in eksen alanlarını yazıyor; tutarlı bir kopya al
    telem_t t;
    noInterrupts();
    t = s_tel;
    uint16_t maxUs = s_loopMaxUs;
    s_loopMaxUs = 0;
    interrupts();
    t.t_ms  = millis();
    t.mode  = fsm_mode();
    t.fault = fsm_fault_code();
    uint8_t imuFresh = (s_imuOk && (millis() - bno085_last_ms()) < 200) ? 1 : 0;
    t.flags = (s_laserOn ? TF_LASER : 0) | (board_motors_enabled() ? TF_MOT_EN : 0)
            | (s_encAllOk ? TF_ENC_OK : 0) | (imuFresh ? TF_IMU_OK : 0) | (pointing_locked() ? TF_LOCKED : 0)
            | (board_motor_enabled(0) ? TF_EN_PAN : 0) | (board_motor_enabled(1) ? TF_EN_TILT : 0) | (board_motor_enabled(2) ? TF_EN_CE : 0);
    { float e, sd, up; pointing_error(&e, &sd, &up); t.aim_err = e; t.aim_side = sd; t.aim_up = up; }
    t.imu_count = bno085_count();
    t.loop_max_us = maxUs;
    t.rx_frames = s_rx.rx_frames;
    t.rx_errors = s_rx.rx_errors;
    sendFrame(MSG_TELEM, &t, sizeof t);
}

// ---------------- Arduino ----------------
void setup()
{
    Serial.begin(115200);
    board_init();
    params_defaults();
    params_load();
    comm_rx_init(&s_rx);
#ifdef LORA_SERIAL
    comm_rx_init(&s_rxLora);
    LORA_SERIAL.begin(9600);                                    // E22 fabrika ayarı
    LORA_SERIAL.addMemoryForWrite(s_loraTxMem, sizeof s_loraTxMem);
#endif
    memset(&s_tel, 0, sizeof s_tel);
    board_spi_init();
    for (int i = 0; i < 3; i++) as5048a_init(&s_enc[i], i);   // timer'lar henüz yok, SPI güvenle kullanılır
    stepgen_init();
    axis_init();
    pointing_init();
    fsm_init();
    fsm_request(MODE_MANUAL);
    {
        char line[96];
        s_bootCause = board_reset_cause();                 // bayraklar okununca silinir; tanı için saklanır
        snprintf(line, sizeof line, "MERCOM-1 acildi, mod MANUAL, motorlar pasif, reset sebebi: %s", s_bootCause);
        sendText(line);
    }

    // IMU
    // Kart resetinde sensör güç kaybetmez: önce takılı kalmış I2C aktarımını bitir, sonra donanımsal reset
    board_i2c_init();
    delay(300);
    s_imuOk = (uint8_t)imuStart();
    sendText(s_imuOk ? "imu: BNO085 bulundu, GRV raporu acik" : "imu: BNO085 BULUNAMADI (SCL D15, SDA D14, adres 0x4A)");

    // Motor sürücüleri
    board_tmc_uart_begin(115200);
    tmcInitAll();

    // Timer'lar en son, kontrol kesmesi ancak her şey hazırken başlasın
    board_timers_start(onStepTick, onCtrlTick);
    // Ana döngü 3 s içinde beslemezse kart resetlenir (I2C/kütüphane takılması, vb.)
    board_watchdog_start(3000);
}

void loop()
{
    board_watchdog_feed();

    // Gelen baytlar
    while (Serial.available() > 0) {
        int b = Serial.read();
        if (b >= 0) comm_rx_byte(&s_rx, (uint8_t)b, handleFrame);
    }
#ifdef LORA_SERIAL
    while (LORA_SERIAL.available() > 0) {
        int b = LORA_SERIAL.read();
        if (b >= 0) comm_rx_byte(&s_rxLora, (uint8_t)b, handleFrameLora);
    }
    loraPump();
    if (s_loraJogMs && millis() - s_loraJogMs > 1500) {          // LoRa döndürmesi tazelenmedi: durdur
        if (s_loraJogAxis >= 0) axis_stop(s_loraJogAxis);
        s_loraJogMs = 0; s_loraJogAxis = -1;
        sendText("lora: dondurme komutu 1,5 s tazelenmedi, eksen durduruldu");
    }
#endif

    // Rapor akışı kesildiyse (kablo oynadı, sensör kendi kendine resetlendi) yeniden başlat
    static uint32_t s_imuRetryMs = 0;
    if (s_imuOk && bno085_count() > 0 && millis() - bno085_last_ms() > 300) {
        s_imuStalls++;
        {
            char l[110];
            snprintf(l, sizeof l, "imu: rapor akisi kesildi (sensor reset sayisi %lu, rapor %lu)",
                     (unsigned long)bno085_resets(), (unsigned long)bno085_count());
            sendText(l);
        }
        // 1. adim: yalniz STM32 I2C cevre birimini sifirla, sensore dokunma. Akis donerse suclu STM32 tarafidir.
        board_i2c_reinit();
        uint32_t t0 = millis(); int back = 0;
        while (millis() - t0 < 400) { board_watchdog_feed(); if (bno085_poll()) { back = 1; break; } }
        if (back) {
            sendText("imu: I2C cevre birimi sifirlaninca akis geri geldi (sensor yeniden baslatilmadi)");
        } else {
            s_imuOk = 0;
            // Sensör az önce çalışıyordu: 2 s bekleme yerine 0,2 s sonra yeniden başlat (kesinti ~3 s'den ~1,3 s'ye iner).
            // 2 s aralık yalnız sensör hiç takılı değilken ana döngüyü yormamak içindi.
            s_imuRetryMs = millis() - 1800;
            sendText("imu: cevre birimi sifirlama yetmedi, sensor yeniden baslatilacak");
        }
    }
    // IMU yoksa 2 s'de bir yalnız adres yokla (mikrosaniyeler sürer); ancak ACK gelirse uzun başlatmayı çalıştır.
    // Böylece sensör takılı değilken ana döngü ve haberleşme yavaşlamaz.
    if (!s_imuOk && millis() - s_imuRetryMs > 2000) {
        s_imuRetryMs = millis();
        I2C_BUSY_BEGIN();
        int ack = board_i2c_probe(0x4A);
        if (!ack) { board_i2c_reinit(); ack = board_i2c_probe(0x4A); }
        I2C_BUSY_END();
        if (ack) {
            s_imuOk = (uint8_t)imuStart();
            if (s_imuOk) { char l[80]; snprintf(l, sizeof l, "imu: yeniden basladi, kesilme sayisi %lu", (unsigned long)s_imuStalls); sendText(l); }
        }
    }

    // IMU raporu geldiyse al (I2C bloklayıcı, kontrol kesmesini etkilemez)
    bno085_set_poll_gap((uint32_t)params_val(P_IMU_POLL_US));
    {
        static int soft = -1;
        int want = params_val(P_IMU_I2C_SOFT) > 0.5f ? 1 : 0;
        if (want != soft) { soft = want; board_i2c_pads(soft); }
    }
    if (s_imuOk) bno085_set_gyro(params_val(P_IMU_GYRO) > 0.5f ? 1 : 0);
    if (s_imuOk && bno085_poll()) {
        float rpy[3]; bno085_rpy(rpy);
        float yaw = rpy[2] - s_yawZero;
        // Gösterim 0..360, enkoderle aynı; roll ve pitch ±180 kalır
        while (yaw >= 360.0f) yaw -= 360.0f;
        while (yaw < 0.0f)    yaw += 360.0f;
        noInterrupts();
        s_tel.rpy_deg[0] = rpy[0]; s_tel.rpy_deg[1] = rpy[1]; s_tel.rpy_deg[2] = yaw;
        interrupts();
        float q[4]; bno085_quat(q);
        float w[3]; bno085_gyro(w);
        static uint32_t gyroSeen = 0;
        uint32_t gc = bno085_gyro_count();
        if (gc != gyroSeen) { pointing_set_gyro(w); gyroSeen = gc; }
        else { float z[3] = { 0, 0, 0 }; pointing_set_gyro(z); }   // jiroskop gelmediyse taban hızı bilinmiyor: ileri besleme 0
        pointing_set_tilt(axis_pos(AX_TILT));      // paralaks: lazerin konumu tilt açısına bağlı
        // Yön bağlama için eklem açıları: pan enkoderden (elle zorlanınca adım sayacı yanılır), tilt adım sayacından
        pointing_set_joints(s_encTrust[AX_PAN] >= 1000 ? s_encDegCont[AX_PAN] : axis_pos(AX_PAN), axis_pos(AX_TILT));
        {
            // Sensör yeniden başladıysa ya da akış 150 ms'den uzun kesildiyse yaw referansı güvenilmez: yeniden bağla
            static uint32_t lastQms = 0, seenEpoch = 0;
            uint32_t nowQ = millis();
            if ((lastQms && nowQ - lastQms > 150) || bno085_epoch() != seenEpoch) { seenEpoch = bno085_epoch(); pointing_imu_restarted(); }
            lastQms = nowQ;
        }
        pointing_set_quat(q);
        {
            static uint32_t anchorsSeen = 0;
            if (pointing_anchor_count() != anchorsSeen) {
                anchorsSeen = pointing_anchor_count();
                char l[90]; snprintf(l, sizeof l, "imu: yon yeniden baglandi, yaw duzeltmesi %ld mderece (%lu. kez)",
                                     mdeg(pointing_anchor_last()), (unsigned long)anchorsSeen);
                sendText(l);
            }
        }
        pointingUpdate();
    }
    // IMU akışı kesildiyse otomatik modda motorları tut (arıza değil, bekle)
    if (fsm_mode() == MODE_AUTO && (!s_imuOk || millis() - bno085_last_ms() > 120)) {
        if (!s_aimHold) { s_aimHold = 1; axis_stop(0); axis_stop(1); sendText("nisan: IMU verisi kesildi, konum tutuluyor"); }
    }

    // Telemetri
    float hz = params_val(P_TELEM_HZ);
    if (hz > 0.0f) {
        uint32_t period = (uint32_t)(1000.0f / hz);
        if (period == 0) period = 1;
        if (millis() - s_lastTelemMs >= period) { s_lastTelemMs = millis(); sendTelemetry(); }
    }
#ifdef LORA_SERIAL
    {
        float lh = params_val(P_LORA_TELEM_HZ);
        if (lh > 0.0f && loraActive()) {
            uint32_t per = (uint32_t)(1000.0f / lh);
            if (millis() - s_loraLastTelemMs >= per) { s_loraLastTelemMs = millis(); s_outLora = 1; sendTelemetry(); s_outLora = 0; }
        }
    }
#endif

    // Enkoder okunamıyor uyarısı (arıza değil)
    for (int i = 0; i < 3; i++) {
        if (s_encWarn[i]) {
            s_encWarn[i] = 0;
            static const char *nm[3] = { "pan", "tilt", "ce" };
            char l[90]; snprintf(l, sizeof l, "%s enkoderi okunamiyor (kablo?); nisan IMU ile suruyor, kapali cevrim bu eksende devre disi", nm[i]);
            sendText(l);
        }
    }

    // Takılma bildirimi: kapalı çevrim 5 kez üst üste düzeltince hareketi durdurur, arızaya sokmaz
    {
        static uint32_t stallSeen[3] = { 0, 0, 0 };
        static const char *nm[3] = { "pan", "tilt", "ce" };
        for (int i = 0; i < 3; i++) {
            uint32_t n = axis_stalls(i);
            if (n != stallSeen[i]) {
                stallSeen[i] = n;
                char l[110];
                snprintf(l, sizeof l, "%s: motor takildi, hareket durduruldu (hiz/ivme dusur ya da kabloya bak). Toplam %lu", nm[i], (unsigned long)n);
                sendText(l);
            }
        }
    }

    // Kalp atışı: 1 Hz, arızada kırmızı yanar
    if (millis() - s_lastHbMs >= 500) {
        s_lastHbMs = millis();
        static uint8_t hb = 0; hb ^= 1;
        board_led_hb(hb);
        board_led_fault(fsm_mode() == MODE_FAULT);
    }
}
