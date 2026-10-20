# MERCOM-1 yazılımı: kod nasıl çalışır, parametreler neye göre ayarlandı

Güncel durum: 1 Ekim 2026, yazılım 0.1.1, kart Teensy 4.1. Kaynak: `src/`.
Günlük karar ve test kayıtları `MERCOM1_OZELLIKLER.md` belgesinde. Bu belge kodun bugünkü hâlini anlatır.

---

## 1. Sistem bir bakışta

Amaç: hareket eden bir platformun üstündeki terminalin lazerini (anten bakışını) sabit bir hedefe kilitli tutmak.

| Parça | Ne | Ayrıntı |
|---|---|---|
| Denetleyici | Teensy 4.1 (600 MHz) | USB seri no 15425680 |
| Motorlar | 3 × NEMA17 (1,8°), TMC2209 sürücü | STEP/DIR ile sürülür. Mikroadım MS pinleriyle 8, akım sürücünün trimpotuyla ayarlı. UART bağlı değil |
| Pan (yatay dönüş) | 1:10 dişli (planet 5 × ek kademe 2) | 0,0225° / mikroadım, 360° serbest dönüş |
| Tilt (yükseliş) | 1:5 planet dişli | 0,045° / mikroadım, −90..+90° |
| CE (çapraz yükseliş) | 1:1, dişlisiz | 0,225° / mikroadım, 0..4° |
| Enkoder | 3 × AS5048A, **motorun arka milinde** | Yazılım SPI. Çıkış açısı = motor açısı / dişli oranı |
| IMU | BNO085, **anten başlığında** | I2C 0x4A. Game Rotation Vector (manyetometresiz) + kalibre jiroskop, ~90 rapor/s |
| Lazer | pin 5 | |
| Telsiz | E22-900T22D LoRa, Serial6 | Sabit adresli mod |

En önemli tasarım noktası şu: **IMU başlıkta.** Lazerin baktığı yönü doğrudan ölçer. Nişan döngüsü motor sayacına değil bu gerçek bakışa göre düzeltir, dişli boşluğu da dahil. Enkoderler yalnızca adım kaçırmayı yakalamak için kullanılır.

---

## 2. Pinler (Teensy 4.1, `src/hw.h`)

| Görev | Pin |
|---|---|
| Enkoder SCK / MISO / MOSI | 6 / 7 / 11 |
| Enkoder CS: pan / tilt / CE | 8 / 9 / 12 |
| IMU SCL / SDA (Wire) | 19 / 18 |
| Pan STEP / DIR / EN | 41 / 40 / 39 |
| Tilt STEP / DIR / EN | 38 / 37 / 36 |
| CE STEP / DIR / EN | 35 / 34 / 33 |
| Lazer | 5 |
| LoRa (Serial6) TX / RX / M0 / M1 / AUX | 24 / 25 / 26 / 27 / 28 |
| Kalp atışı LED'i | 13 |

Yer istasyonu köprüsü (ayrı Teensy 4.1, seri no 18913180, `tools/lora_koprusu/`) E22'ye Serial4 ile bağlı: TX 17, RX 16, M0 37, M1 39, AUX 41.

---

## 3. Dosyalar ve katmanlar

```
src/
  main.cpp          setup/loop, kesmeler, komut işleyici, enkoder okuma, IMU yönetimi, telemetri, LoRa
  hw.h              pin planı + donanım katmanı arayüzü
  hw_teensy.cpp     Teensy: zamanlayıcılar, yazılım SPI, I2C, bekçi köpek, kalıcı bellek (EEPROM)
  hw.cpp            eski STM32 (Nucleo-H753) karşılığı, Teensy derlemesinde kullanılmaz
  app/
    proto.h         mesaj kimlikleri ve ACK kodları
    comm.c/h        çerçeve çözme/kurma, CRC16
    params.c/h      78 parametre, varsayılanlar, kalıcı kayıt
    telemetry.h     96 baytlık telemetri paketi
    fsm.c/h         modlar: INIT, MANUAL, AUTO, (FAULT)
  ctrl/
    pointing.c/h    NİŞAN: IMU yönelimi → hata → pan/tilt hız komutu
    axis.c/h        EKSEN: hız profili, sınırlar, salınım, kapalı çevrim (adım kaçırma)
  drv/
    stepgen.c/h     adım üreteci (DDA, 20 kHz)
    as5048a.c/h     enkoder sürücüsü
    bno085.cpp/h    IMU sürücüsü (yamalı Adafruit kütüphanesi üstünde)
    tmc2209.c/h     sürücü UART'ı (Teensy'de bağlı değil, kullanılmıyor)
lib/Adafruit_BNO08x yamalı kütüphane (yamalar: lib/.../MERCOM_YAMA.md)
tools/              arayüz (mercom_gui.py), protokol (mercom_link.py), LoRa köprüsü, test betikleri
```

### Veri akışı

```
BNO085 (başlıkta) ──I2C, ~90 Hz──► loop(): bno085_poll()
                                      ├─ pointing_set_quat()  → başlığın dünya yönelimi, bakış yönü, hata
                                      └─ pointingUpdate()     → (AUTO) pointing_step() → pan/tilt hız isteği
                                                                     │
                                                          axis_set_velocity()
                                                                     ▼
1 kHz kesme: controlStep() → enkoder oku → axis_tick() (profil + kapalı çevrim) → stepgen_set_rate()
                                                                     ▼
20 kHz kesme: stepgen_tick() → STEP/DIR darbeleri → TMC2209 → motor → dişli → başlık
```

---

## 4. Zamanlama

| İş | Nerede | Sıklık | Not |
|---|---|---|---|
| Adım darbeleri | `stepgen_tick` (IntervalTimer) | 20 kHz (50 µs) | En yüksek öncelik (16). Hiçbir şey adımları geciktiremez |
| Kontrol adımı | `controlStep` (IntervalTimer) | 1 kHz | Öncelik 48. 900 µs'yi aşarsa yalnızca sayılır ("dongu asimi") |
| IMU okuma + nişan | `loop()` | Rapor geldikçe (~90 Hz) | İki okuma arasında ≥ `imu_poll_us` boşluk |
| USB telemetri | `loop()` | `telem_hz` | Varsayılan 50, arayüz 20 ister |
| LoRa telemetri | `loop()` | `lora_telem_hz` (1 Hz) | Yalnızca LoRa son 15 s'de komut gönderdiyse |
| Bekçi köpek | RTWDOG | 3 s | `loop()` beslemezse kart yeniden başlar |
| Kalp atışı LED'i | `loop()` | 1 Hz | |

---

## 5. Açılış sırası (`setup()`)

1. USB seri açılır. `params_defaults()` yazılımdaki varsayılanları yükler. Ardından `params_load()`: kartta sağlam bir kalıcı kayıt varsa onun değerleri bunların üstüne yazılır.
2. LoRa portu (Serial6, 9600) açılır.
3. Enkoderler, adım üreteci, eksen ve nişan katmanları kurulur. Mod MANUAL olur, sürücüler **kapalı** başlar.
4. IMU başlatılır (3 deneme).
5. TMC taraması yapılır. Teensy'de UART bağlı olmadığı için "cevap yok" mesajı normaldir.
6. Zamanlayıcılar ve bekçi köpek başlar.
7. ~3 s sonra (200 IMU raporu gelince) tilt sayacı, IMU'nun ölçtüğü bakış yükselişine eşitlenir (`tiltFromImu`). Pan sayacı açılıştaki konumu 0 kabul eder; pan göreli çalışır.

---

## 6. Modlar (`fsm.c`)

| Mod | Ne yapar |
|---|---|
| INIT | Yalnızca açılışta, hemen MANUAL'a geçer |
| MANUAL | Arayüzden basılı tutarak döndürme, açıya gitme, salınım, sayılı adım, "Kabul et" (sıfırlama), lazer, sürücüler |
| AUTO | Nişan döngüsü pan ve tilt'i sürer, CE tutulur |
| FAULT | Kodda var ama hiçbir yol bu moda götürmüyor. Tasarım kararı: arıza durumu yok, sistem hep çalışmaya devam eder |

**AUTO'ya geçiş şartları** (`MSG_SET_MODE`):
- hedef kilitli (Kilitle basılmış);
- IMU verisi taze (300 ms'den yeni);
- pan ve tilt sürücüleri açık;
- başlık devrik değil, yükselişi −80°..+80° arasında.

CE sürücüsü kapalıysa kendiliğinden açılır, çünkü serbest kalırsa başlık savrulur. Her mod geçişinde eksenler durur.

---

## 7. Otomatik takip nasıl çalışır (`pointing.c`, `main.cpp: pointingUpdate`)

Her IMU örneğinde (~90 Hz) şu adımlar çalışır:

1. **Başlığın yönelimi.** `R_NH = R_NI(quat) · R_IH(montaj)`.
   - `R_NI`, IMU'nun dünya yönelimidir (Game Rotation Vector kuaterniyonu).
   - `R_IH`, IMU'nun başlığa montaj açısıdır (`imu_mount_r/p/y`, sıra Rz·Ry·Rx).
   - Bakış yönü `d` = başlığın ileri ekseni (R_NH'nin 1. sütunu).
2. **Kilit** (`MSG_LOCK`, "Hedefe kilitle").
   - O anki bakış `d0` saklanır.
   - Hedef noktası `T = L + D·d0` hesaplanır. `L` lazerin dünya konumu, `D = plx_d`.
   - Tilt sayacı IMU'dan yenilenir.
   - **Kilit, platform düz ve hareketsizken alınmalı.**
3. **Referans yön.**
   - Paralaks açıksa `d_ref = (T − L_şimdi) / |T − L_şimdi|`, yani lazerin yeni yerinden hedef noktasına giden yön.
   - Kapalıysa `d_ref = d0`.
   - Lazer konumu şöyle hesaplanır: `L = R_NH · (ht·sin t + lx, ly, ht·cos t + lz)`, burada `t` tilt açısıdır. Model, platformun tilt ekseninin `plx_ht` altındaki bir nokta etrafında döndüğünü varsayar.
4. **Hata.**
   - `e = d × d_ref` başlık çerçevesine çevrilir: yanal (+sol) ve dikey (+yukarı), derece cinsinden.
   - Büyük açılarda atan2 ile doğru ölçeklenir.
5. **Ölü bant.** Hata `aim_db` içindeyse 0 sayılır. Aşarsa yalnızca bandı aşan kısım düzeltilir; duruşta motorlar kıpırdamaz.
6. **PI.**
   - `r_yan = kp_pan·e_yan + ki_pan·∫e_yan` ve `r_yukarı = kp_tilt·e_yukarı + ki_tilt·∫e_yukarı`.
   - İntegral ±`aim_imax` ile sınırlı, hız doyumdayken büyümez.
7. **İleri besleme.**
   - Jiroskop başlığın dönüş hızını ölçer; bir önceki örnekteki kendi motor hızımız bundan çıkarılınca tabanın getirdiği hız kalır.
   - Bunun tilt'te `aim_kff` (0,7), pan'da `aim_kff_pan` (0) kadarı önceden dengelenir.
8. **Eklem hızına çevirme.**
   - `v_pan = işaret·u_yan / cos(yükseliş)`. Yükseliş 78°'yi geçince cos 0,2'de tutulur.
   - `v_tilt = işaret·u_yukarı`.
   - İkisi de ±`aim_vmax` ile sınırlı.
9. Pan'da 0,3 °/s'nin, tilt'te 0,4 °/s'nin altındaki istekler 0 sayılır (çırpınma önlenir). Sonuç eksen katmanına gider (`axis_set_velocity`).

**Korumalar.** Otomatik mod hiçbir durumda kendi kendine bırakılmaz. Sorun olduğunda yalnızca bekler, düzelince devam eder (tasarım kararı).

| Durum | Davranış |
|---|---|
| İki IMU örneği arası > 60 ms | Eksenler tutulur, integral sıfırlanır |
| IMU akışı > 120 ms kesik | Eksenler tutulur |
| Poz geçersiz (başlık 70°'den fazla devrik ya da \|yükseliş\| ≥ 80°) | Eksenler tutulur |
| Pan ya da tilt sürücüsü kapalı | Eksenler tutulur (sayaçlar kaçmasın); sürücü açılınca devam eder |
| IMU yeniden başladı | 6 örnek beklenir, yaw tabanın kesinti öncesi yönüne bağlanır (`yaw_anchor`); hedef kaymaz |

CE otomatik modda nişan döngüsüyle sürülmez; **kilit anındaki açısında tutulur** (1 Eki). Hakem braketi elle çevirip bırakınca CE enkoderle kaymayı görür ve o açıya geri döner; elle tutulurken takılsa da hedefi korur, 0,2 s'de bir yeniden dener. Bant `enc_db_ce` 2,5°, altını pan karşılar.

---

## 8. Eksen katmanı (`axis.c`, 1 kHz)

- **Birim:** anten çıkışında derece. Konum = adım sayacı × `dps` × `mot_dir`.
- **Yamuk hız profili:** hız `vmax`'ı, ivme `amax`'ı aşmaz.
  - Hedefe birkaç adım kala sayılı son yaklaşım yapılır; sayaç hedefe en fazla 1 mikroadım uzakta durur (CE'de 0,225°).
  - Kalkış ve varış hızı 12 adım/s.
- **Sınırlar:**
  - Pan 0..360 tanımlıyken serbesttir: kısa yoldan gider, sonsuz dönebilir.
  - Tilt ve CE sınıra kırpılır.
- **Boşluk telafisi** (`bl_*`): kod var, kapalı (0).
- **Kapalı çevrim** (`cl_pan`, `cl_tilt`, `cl_ce` açık; CE kartta kalıcı 1, yazılım varsayılanı 0):
  - Hareket hâlinde sayaç–enkoder farkı `sl_thr + 0,03·|hız|` eşiğini 3 tik üst üste aşarsa sayaç enkodere eşitlenir.
  - Bir harekette 5'ten fazla eşitleme olursa hareket durur ("motor takıldı" mesajı). Bu bir arıza durumu değildir.
  - MANUAL'da dururken fark `enc_db`'yi 200 ms aşarsa konum enkoderden alınır ve hedefe geri yürünür. AUTO'da bu yapılmaz, nişan döngüsü düzeltir.
  - Sayaç yeniden tanımlanınca (IMU'dan tilt, "Kabul et") enkoder farkı `encoff` olarak saklanır. Mutlak enkoder açısı sayacı bozmaz.
  - Enkodere ancak 1000 art arda sağlıklı okumadan (~3 s) sonra güvenilir.

---

## 9. Enkoder okuma (`main.cpp: controlStep`)

- Yazılım SPI ile bir okuma ~350 µs sürer, bu yüzden her tikte yalnızca bir enkoder okunur. Her enkoder 333 Hz'de okunmuş olur.
- **I2C senkronu (`i2c_sync`):** IMU'nun I2C aktarımı sürerken enkoder okunmaz, o tik atlanır.
  - Sebep: enkoder SPI'sinin keskin kenarları I2C kablosuna sızıp BNO085'i resetliyordu.
  - Çözümden sonra 5 dk hareketli testte 0 reset ölçüldü.
- **Sürekli sayım:** her okumada ±yarım tur içine sarılmış fark eklenir. Geçen süreye göre fiziksel olarak imkânsız sıçramalar bozuk okuma sayılıp atılır.
- **Mıknatıs sağlığı:** 200 ms'de bir eksenin tanı yazmacı okunur (AGC, OCF, "uzak" bayrağı). Sağlıksızsa o enkodere güvenilmez.
- **SPI takılma tespiti:** 500 ms'de bir yapılır; takılmışsa SPI yeniden başlatılır.
- Enkoder okunamazsa arıza sayılmaz: uyarı yazılır, o eksende kapalı çevrim devre dışı kalır, nişan IMU ile sürer.

---

## 10. IMU yönetimi

- Açılan raporlar:
  - Game Rotation Vector, 100 Hz istenir, ~90 Hz gelir;
  - kalibre jiroskop, 100 Hz (ileri besleme için);
  - yerçekimi, 10 Hz (tanı için).
- Rapor akışı 300 ms kesilirse önce I2C çevre birimi sıfırlanır. Akış dönmezse sensör yeniden başlatılır. Sensör hiç yoksa 2 s'de bir yalnızca adresi yoklanır; ana döngü yavaşlamaz.
- `imu_poll_us`: iki IMU okuması arasındaki boşluk. Enkoderler bu boşlukta okunur.

---

## 11. Haberleşme

**Çerçeve:** `A5 | LEN | ID | yük | CRC16` (CCITT, little-endian). USB (115200) ve LoRa (9600) aynı çerçeveyi kullanır.

**Başlıca komutlar** (tam liste: `src/app/proto.h`):

| Kimlik | Komut | Yük |
|---|---|---|
| 0x01 | PING | Cevap: PONG + durum metinleri (mod, IMU, kalıcı kayıt, LoRa) |
| 0x02 | Mod | 1 = manuel, 2 = otomatik |
| 0x03 | Basılı tutunca döndür | eksen, hız °/s (0 = dur) |
| 0x04 | Açıya git | eksen, açı |
| 0x05 / 0x06 | Kilitle / bırak | — |
| 0x07 | Lazer | açık/kapalı |
| 0x08 / 0x09 | Parametre oku / yaz | kimlik (+ değer) |
| 0x0A | Kalıcı kaydet | yük yok: kaydet; 1 bayt 0: kaydı sil |
| 0x0B | Sürücü | 1 bayt: üçü birden; 2 bayt: eksen + durum |
| 0x0D | DUR | — |
| 0x0F / 0x10 / 0x1F | Enkoder / IMU / eksen tanısı | Cevap metin olarak gelir |
| 0x1D | Salınım | eksen, A, B, bekleme ms |
| 0x1E | Kabul et | eksen, açı: şu anki konumu bu açı say |
| 0x20 | Tilt'i IMU'dan al | — |
| 0x21 | LoRa ayarı | E22 ayarını oku / yaz |

**ACK kodları:** 0 tamam, 1 yanlış uzunluk, 2 yanlış değer, 3 yanlış mod, 4 desteklenmiyor, 5 reddedildi.

**Telemetri** (96 bayt): zaman, mod, bayraklar, enkoder açıları, komut açıları, hızlar, IMU roll/pitch/yaw, sayaç–enkoder farkı, döngü süresi, IMU rapor sayısı, nişan hatası (toplam / yanal / dikey).
- Bayraklar: kilit, lazer, IMU taze, enkoderler sağlam, sürücü açık, eksen başına sürücü açık.

**LoRa (E22-900T22D, sabit adresli mod):**
- Adresler ve kanal:
  - terminal 0x4D01, yer istasyonu 0x4D02, NETID 0x4D;
  - kanal 15 (865,125 MHz), hava hızı 9,6 kbps, 22 dBm;
  - dinle-sonra-konuş (LBT) açık, iki modülde aynı şifre anahtarı.
- Her paketin başında hedefin adresi ve kanalı vardır (terminal → `4D 02 0F`). Böylece başka takımların modülleriyle karışmaz.
- **Gönderim kuyruğu:** 32 çerçeve; tek pakette en fazla 237 bayt. Paketler arasında UART süresi + 20 ms beklenir ve modülün boş olması (AUX) şartı aranır.
- "LoRa etkin" sayılmak için son 15 s içinde LoRa'dan çerçeve gelmiş olmalı. O zaman telemetri saniyede 1 gider ve kart mesajları LoRa'ya da kopyalanır.
- **Döndürme güvenliği:** LoRa'dan gelen döndürme komutu 1,5 s tazelenmezse eksen durur (bırakma komutu kaybolursa motor dönmesin). Arayüz, düğme basılı kaldıkça komutu 0,5 s'de bir yeniden gönderir.
- Arayüz LoRa'da onay bekleyen komutları 2,5 s içinde ACK gelmezse 3 kez yeniden gönderir.

**Kalıcı kayıt** (şartname: "güç kapanıp açılınca son hâliyle kayıtlı kalmalı"):
- Teensy'nin flash EEPROM öykünmesine "MRC1" başlığı + 78 değer + FNV-1a kontrol toplamı yazılır.
- Arayüzde "Karta yaz" ve "Bilgisayardan yükle ve karta yaz" kaydı otomatik yapar. "Kalıcı kaydı sil", bir sonraki açılışı yazılım varsayılanlarına döndürür.
- `telem_hz` kaydedilir ama açılışta yüklenmez.
- Yeni yazılım yüklense de kayıt silinmez; **kayıtlı değer, yazılımdaki varsayılanı ezer.**

---

## 12. Parametreler: değer, anlam ve neye göre ayarlandı

Üç değer yazan satırlar eksen sırasıyladır: pan / tilt / CE. Kimlik sırası `src/app/params.h` ile `tools/mercom_link.py` dosyasındaki `PARAM_NAMES` listesinde aynıdır.

### 12.1 Nişan döngüsü (otomatik takip)

| Ad | Değer | Birim | Ne yapar | Neye göre |
|---|---|---|---|---|
| `kp_pan`, `kp_tilt` | 8, 8 | 1/s | Hatanın kaç katı hızla düzeltileceği | 30 Eyl kazanç testi (`kazanc_testi.py`): 8 en iyi sonucu verdi |
| `ki_pan`, `ki_tilt` | 2, 2 | 1/s² | Kalıcı küçük sapmayı sıfırlayan integral | İlk ayar; ±`aim_imax` sınırıyla güvenli |
| `kp_ce`, `ki_ce` | 8, 2 | — | **Kullanılmıyor**: CE takipte sürülmez | — |
| `aim_imax` | 2 | ° | İntegral sınırı | Integral taşmasın diye |
| `aim_vmax` | 45 | °/s | Otomatikte en yüksek düzeltme hızı | 8 s kuralı: 180° elle bozmada ~6 s'de dönüş. 60 iken pan geri dönüşte kilitlendi (25 Eyl) |
| `aim_db` | 0,3 | ° | Ölü bant: altında motor kıpırdamaz | 0,1'de pan boşluğu (0,7°) yüzünden sürekli titredi. 0,15 denendi, 0,3 seçildi. Ø10 payı 4 m'de 0,72° |
| `aim_kff` | 0,7 | 0..1 | Tilt ileri beslemesi: tabanın hızının ne kadarı önceden dengelenir | 25 Eyl benzetim: 0,7 → şartname hareketinde 0,2° hata. Tilt boşluğu küçük (0,2°) |
| `aim_kff_pan` | **0,5** (kartta kalıcı; yazılım varsayılanı 0,2) | 0..1 | Pan ileri beslemesi | 25 Eyl: 0,7 iken pan boşluğu ile salınım yaptı (o zaman ölü bant 0,1'di), kapatıldı. 1 Eki yarışma videosunda yaw'da ±12–15 cm gecikme görülünce 0,5 ve 0,7 denendi: masada dururken titreme yok. Önce 0,5, sonra 0,2 seçildi (10 °/s yaw'da tahmini ~9 cm; 0,5'te ~6,5 cm; 0,7'de ~4,7 cm) |
| `aim_sign_pan`, `aim_sign_tilt` | +1, +1 | ±1 | Düzeltme yönü | İlk denemede doğrulandı; lazer hedeften kaçarsa ters çevrilir |
| `aim_err_max` | 120 | ° | **Kullanılmıyor** | Tasarım kararı: otomatik mod hiçbir hatada vazgeçmez |
| `laser_err_thr` | 0,5 | ° | **Kullanılmıyor** | — |

### 12.2 Paralaks (lazer platformla birlikte yer değiştirince)

| Ad | Değer | Birim | Ne yapar | Neye göre |
|---|---|---|---|---|
| `plx_en` | 1 | 0/1 | Paralaks düzeltmesi açık/kapalı | 29 Eyl tezgâh testi (taban 15–18° yatırıldı, duvar 2 m): kapalıyken 14 cm, açıkken 2 cm kayma |
| `plx_ht` | 0,166 | m | Platformun dönme merkezinden tilt eksenine dikey mesafe | Platform ölçüsü bilinmiyordu, tezgâh değeri kullanıldı. **30 Eyl yarışma videosu:** jüri platformunun merkezi çok daha aşağıda görünüyor (~0,9 m tahmini). Jüriye sorulup ayarlanmalı (bkz. 14) |
| `plx_lx`, `plx_ly`, `plx_lz` | 0,10 / −0,13 / 0,05 | m | Lazer çıkışının tilt ekseni noktasına göre konumu (ileri, sol+, yukarı) | Terminal üzerinde ölçüldü: 10 cm ileride, 13 cm sağda, 5 cm yukarıda |
| `plx_d` | 4,0 | m | Kilit anında lazerden hedefe mesafe | Şartname: lazer çıkışından 4 m eğik mesafe, 30° yükseliş (30 Eyl teyit edildi) |

Tezgâh denemelerinde geçici olarak 1,501 / 0,19 ya da 2,254 / 0,19 kullanıldı. Bu değerler "Karta yaz" ile yazılırsa **kalıcı olur.** Yarışmadan önce "Bilgisayardan yükle ve karta yaz" ile `params.json` değerlerine dön.

### 12.3 IMU

| Ad | Değer | Birim | Ne yapar | Neye göre |
|---|---|---|---|---|
| `imu_mount_r`, `imu_mount_p`, `imu_mount_y` | −2,417 / 5,512 / 93,441 | ° | IMU'nun başlığa montaj açısı | 10 Eyl kalibrasyonu. Eğim (p) 29 Eyl'de lazerle yeniden ölçüldü (lazer ucu 28 cm, duvardaki nokta 132 cm, mesafe 200 cm → ışın 27,47°). Eski değer 10° hatalıydı. Yeniden ölçmek için: `tools/montaj_olc.py` |
| `imu_rate_hz` | 100 | Hz | İstenen rapor hızı | Gelen ~90 Hz |
| `imu_poll_us` | 1000 | µs | İki IMU okuması arası boşluk | BNO085 reset çözümü (`i2c_sync` ile birlikte). Enkoderler bu boşlukta okunur |
| `i2c_sync` | 1 | 0/1 | I2C sürerken enkoder okunmaz | BNO085 resetinin kök sebebi buydu. 29 Eyl: 5 dk üç eksen harekette 0 reset |
| `imu_gyro` | 1 | 0/1 | Jiroskop raporu | İleri besleme için gerekli |
| `imu_i2c_soft` | 0 | 0/1 | I2C pinlerinde yumuşak kenar | 29 Eyl deneyinde fayda yok |
| `enc_read` | 1 | 0/1 | Enkoderler okunsun mu | Deney anahtarı (0 iken kapalı çevrim de kapanır) |
| `yaw_anchor` | 1 | 0/1 | IMU yeniden başlayınca yönü eskisine bağla | Bozma testinde IMU restartında hedef 4–9° kayıyordu; sonrasında 0,4–0,6° |
| `yaw_drift_tau`, `yaw_drift_en` | 60, 0 | — | **Kullanılmıyor** | — |

### 12.4 Eksen hareketi

| Ad | Değer | Birim | Ne yapar | Neye göre |
|---|---|---|---|---|
| `vmax_*` | 50 / 40 / 20 | °/s | Hız sınırı | 25 Eyl gece artırıldı. Pan 60'ta geri dönüşte kilitlendi |
| `amax_*` | 150 / 120 / 100 | °/s² | İvme sınırı | Tilt 40/150 iken başlık 20° yukarıdayken adım kaçırdı (25 Eyl) |
| `dps_*` | 0,0225 / 0,045 / 0,225 | °/mikroadım | Mikroadım başına çıkış açısı | 1,8° / (8 mikroadım × oran). Oranlar 25 Eyl IMU ölçümü: pan 10,05, tilt 4,93, CE 1 |
| `mot_dir_*` | 1 / 1 / 1 | ±1 | Motor yönü | Bağlantıya göre |
| `pan_min`, `pan_max` | 0 / 360 | ° | 0..360 tanımlıyken pan serbest | 30 Eyl: kablolar 360°'ye izin veriyor |
| `tilt_min`, `tilt_max` | −90 / 90 | ° | Tilt sınırı | 30 Eyl kararı (taban çok yatınca 0° sınırına dayanıyordu) |
| `ce_min`, `ce_max` | 0 / 4 | ° | CE sınırı | Mekanik aralık 4°, ortası 2° |
| `bl_*` | 0 / 0 / 0 | ° | Dişli boşluğu telafisi | Kapalı: telafi duruşta ±0,6° salınım yaptı (25 Eyl). Ölçülen boşluk pan 0,7°, tilt 0,2° |

### 12.5 Enkoder ve kapalı çevrim

| Ad | Değer | Birim | Ne yapar | Neye göre |
|---|---|---|---|---|
| `enc_ratio_*` | 10 / 5 / 1 | — | Enkoder motor milinde: çıkış = motor / oran | Dişli oranları |
| `enc_dir_*` | 1 / 1 / −1 | ±1 | Enkoder yönü | CE ters takılı (25 Eyl) |
| `enc_off_*` | 0 / 0 / 0 | ° | Enkoder sıfır kayması | "Kabul et" komutu günceller |
| `cl_*` | 1 / 1 / **1** | 0/1 | Kapalı çevrim (adım kaçırma düzeltmesi) | 29 Eyl pan/tilt açıldı: pan bir denemede 72° adım kaçırmıştı. CE 1 Eki açıldı (mıknatıs düzelmiş, AGC 109; yön ve ölçek doğrulandı: 2,025° komut / 1,99° enkoder); otomatikte CE tutma için gerekli. Kartta kalıcı 1, yazılım varsayılanı hâlâ 0 |
| `sl_thr_*` | 1,5 / 1,5 / 6 | ° | Harekette kaçırma eşiği (+0,03 × hız) | 3 tam adımdan biraz fazla. Hız payı: 50 °/s'de sahte düzeltmeler vardı, payla 0 |
| `enc_db_*` | 0,5 / 0,6 / 2,5 | ° | Manuel modda duruşta düzeltme ölü bandı | 1 tam adımdan büyük; 0,15'te sürekli seğiriyordu |

### 12.6 Sürücü (Teensy'de etkisiz)

`mot_irun_ma` 1000, `mot_ihold_pct` 50, `mot_usteps` 8, `tmc_rsense` 0,11. Bunlar TMC2209'a UART üzerinden yazılmak içindi. Teensy'de UART bağlı değil, değiştirmek bir şeyi değiştirmez. Gerçekte mikroadımı MS pinleri (8), akımı sürücünün trimpotu belirler.

### 12.7 Haberleşme

| Ad | Değer | Ne yapar | Neye göre |
|---|---|---|---|
| `telem_hz` | 50 | USB telemetri sıklığı | Oturum ayarı; arayüz bağlanınca 20 ister, kalıcı kayıttan yüklenmez |
| `lora_telem_hz` | 1 | LoRa telemetri sıklığı | Tercih (30 Eyl); 9,6 kbps hava hızında güvenli |

### 12.8 Yarışma değerleri

Bu değerler kartta kalıcı kayıtlı ve yazılımın varsayılanlarıyla aynıdır. `tools/params.json` de bunları tutar:

- `plx_en` 1, `plx_d` 4,0, `plx_ht` 0,166, `plx_lx` 0,10, `plx_ly` −0,13, `plx_lz` 0,05;
- `aim_db` 0,3, `kp_pan` ve `kp_tilt` 8, `aim_kff` 0,7, `aim_kff_pan` 0,5 (1 Eki, kartta), `cl_ce` 1;
- `tilt_min` −90, `tilt_max` 90, `ce_min` 0, `ce_max` 4;
- `vmax_ce` 20, `lora_telem_hz` 1.

---

## 13. Kullanım sırası (yarışmada)

1. Arayüzü aç ve bağlan. USB'den "Teensy 4.1 terminal", kablosuz için "LoRa yer istasyonu" seç.
2. **Sürücüleri aç**, sonra nişan al. Sürücüler açılırken motorlar ±2 tam adım sıçrayabilir; en çok CE'de (±3,6°).
3. Lazeri aç, basılı tutarak döndürme ya da "Git" ile lazeri hedefin merkezine getir.
4. **Platform düz ve hareketsizken "Hedefe kilitle"**, ardından "Otomatik".
5. Durumu izle: nişan hatası, IMU "veri geliyor", sürücü kutucukları.
6. Değer değiştireceksen kutuya yaz ve "Karta yaz"a bas. Değer kalıcı kaydedilir.

Arayüzde **DUR** otomatik takipteyse önce manuel moda geçer, kilit kalır. **ACİL DUR** sürücüleri kapatır ve manuel moda geçer; sürücüler yeniden açılınca sistem kendiliğinden hareket etmez. USB bağlantısı koparsa arayüz saniyede bir kendiliğinden yeniden bağlanır.

---

## 14. Bilinen sınırlar

- **Öteleme görülemez.**
  - IMU yalnızca açıyı ölçer. Platform terminali yerinden taşırsa (yukarı-aşağı kayma, ya da uzaktaki bir merkez etrafında dönme) lazer o kadar kayar.
  - Dönme merkezi doğru girilirse (`plx_ht`) dönmeden gelen kayma düzeltilir. Saf kayma (heave) düzeltilemez.
  - 30 Eyl yarışma videosunda ölçülen:
    - platform düz ve hareketsizken lazer merkezden ortalama 0,6 cm uzakta;
    - platform eğik bekleyince sabit 10–12 cm aşağıda, platform düzelince sıfıra dönüyor;
    - salınımda ±7–10 cm.
  - Yapılacak: jüriden platformun dönme merkezini öğren, `plx_ht`'yi o merkezden tilt eksenine olan dikey mesafe yap, terminali tablanın ortasına yerleştir (`tools/yarisma_video_analiz.png`).
- **Yaw'da pan gecikmesi.** 1 Ekim testinde yaw sırasında hata yatayda ±12–15 cm çıktı; o sırada pan ileri beslemesi kapalıydı (`tools/yarisma_video_1ekim.png`). Aynı gün `aim_kff_pan` 0,2 yapıldı (koda ve karta). Hesaplanan gecikme ~1,55° → ~1,3°; platformda henüz denenmedi.
- **Pan dişli boşluğu 0,7°.** Duruşta küçük bir salınım ve ölü bant ihtiyacı bundan gelir. Mekanik çözüm bekliyor.
- **Pan göreli çalışır.** Açılışta pan 0 rastgeledir; tilt IMU'dan kurulur.
- **CE takipte sürülmüyor**, kilit açısında tutuluyor (elle çevrilince geri döner).
- **Kart kaydı ile yazılım varsayılanı farklı:** kartta `cl_ce` 1 ve `aim_kff_pan` 0,5; yazılımda 0 ve 0,2. "Kalıcı kaydı sil" yapılırsa kart yazılımdaki değerlerle açılır ve CE tutma kapanır.
- **IMU kablosu motor kablolarından ayrı geçmeli.** 1 Ekim'de yan yana bağlıyken motorlar dönerken IMU resetlendi.
- **Kalıcı kayıt yazılım varsayılanını ezer.** Yazılımda bir varsayılanı değiştirip yüklediysen kayıt eskisini kullanmaya devam eder. Yeni varsayılanın geçerli olması için "Kalıcı kaydı sil" ve ardından yeniden başlat.
