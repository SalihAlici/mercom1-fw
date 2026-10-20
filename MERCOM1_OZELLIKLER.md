# MERCOM-1 Sistem Özellikleri ve Kararlar

Takım MERGEN · TEKNOFEST 2026 Hareketli Uydu Terminali · Teslim: 30 Eylül 2026
Bu belge 21 Eylül 2026 itibarıyla kararlaştırılan özellikleri toplar. "Sınanmadı" yazan maddeler donanımda henüz denenmemiştir.


---

## 1. Yarışmanın istediği (şartname özeti)

| Konu | İster |
|---|---|
| Görev | Stewart platformu hareket ederken antenin boresight eksenindeki lazeri sabit bir hedef noktada tutmak |
| Hedef | Lazer çıkışından 4 m eğik mesafede, 30° elevasyonda, merkezi terminalin konduğu noktadan 2 m yukarıda. Eş merkezli çemberler Ø10, 20, 30 … 100 cm |
| Platform hareketi | Şartname metni: "Platform hareket zarfı: Roll ±8°, Pitch ±8°, Yaw: 0–360° sürekli (Stewart platformu 360° dönüş yapmayacaktır. Fakat bağlantı 360 derece içerisinde azimuth eksenine göre yapılabilecektir.), Periyot: 10 s". Sinüs varsayımıyla en yüksek açısal hız 8° × 2π / 10 s ≈ 5 °/s. Yaw: terminal platforma herhangi bir azimut yönünde bağlanabilir → pan hedefe dönmek için ±180°'ye kadar gidebilir; pan kabloları 360° dönmeye izin veriyor |
| Süre | Hareket halinde 5 dakika kesintisiz stabilize takip |
| Geri dönüş | Takip bozulunca en fazla 8 s içinde hedefe dönüş, arıza olmadan. **İki bozma yöntemi de uygulanacak** (23 Eylül): 1) anten elle çevrilir, 2) takip durdurulup başka açıya gönderilir. Kilit yönü her iki durumda da korunur |
| Modlar | Manuel (arayüzden girilen açıya git) ve otomatik (sensörlerle yönelim). Stabilizasyon hareket başlar başlamaz devrede olmalı |
| Eksen yeteneği | Azimut 0–360°, elevasyon 0–90° |
| Arayüz | Bilgisayar/tablet üzerinde; yönlendirme, takipten düşürme, hareket limitlerinin ayarı. (GPS parametresi girişi Türksat göreviyle birlikte kapsam dışı, takım kararı 23 Eylül) |
| Erişim | Hareket halindeyken denetleyiciye kablolu ya da kablosuz erişim |
| Kalıcılık | Tüm parametreler güç kapanıp açılınca son hâliyle kayıtlı kalmalı |
| Fiziksel | Anten çapı 10–50 cm, toplam en fazla 20 kg, tüm bileşenler terminal üzerinde, besleme 5–36 VDC ya da 220 VAC, nominal tüketim ≤ 140 W |

Türksat uydu takibi görevi kapsam dışı bırakıldı (takım kararı). Şartnamede maddesi duruyor.

### Çemberlerin açı karşılığı (4 m'de)

| Çember | Yarıçap | İzin verilen sapma |
|---|---|---|
| Ø10 cm | 5 cm | 0,72° |
| Ø20 cm | 10 cm | 1,43° |
| Ø30 cm | 15 cm | 2,15° |
| Ø50 cm | 25 cm | 3,6° |
| Ø100 cm | 50 cm | 7,1° |

Hedef: Ø10 çember. Paralaks telafisi ve BNO reset sorununun çözümü olmadan gerçekçi sınır Ø20.

---

## 2. Donanım

| Bileşen | Model | Not |
|---|---|---|
| Denetleyici | NUCLEO-H753ZI (MB1364) | STM32H753ZI, 480 MHz |
| Motorlar | 3 × NEMA17 adım motoru | Etiket akımı henüz bilinmiyor |
| Sürücüler | 3 × MKS TMC2209 V2.0 | Takımın kendi taşıyıcı kartlarında; Rsense 0,11 Ω; Vref 1,25 V ≈ 0,88 A |
| Enkoderler | 3 × AS5048A (AS5048A-EK-AB adaptör) | Mıknatıs motorun ARKA milinde, redüktörden önce |
| IMU | BNO085 | Anten başlığında, I2C, regülatörsüz kart: yalnız 3,3 V |
| Telsiz | 2 × E22-900T22D LoRa | 22 dBm; biri terminalde, biri yer istasyonunda |
| Lazer | 520 nm, 5 mW | MOSFET ile anahtarlanır, anten braketinin tam ortasında |
| Güç | 24 V kaynak (sürücüler) + 24→5 V dönüştürücü önerildi | Sahada besleme ASELSAN'dan |

### Mekanik zincir

Taban plakası → **PAN** (dikey motor, 1:5 planet redüktör) → **TİLT** (yatay motor, 1:5 planet redüktör) → **CE** (redüktörsüz, doğrudan) → başlık: BNO085 + lazer + anten.

| Eksen | Çalışma aralığı | Redüktör | Not |
|---|---|---|---|
| Pan | 0–360°, sınırsız, her zaman kısa yoldan | 1:5 (motorla doğrulandı) | Asıl modelde kablolar dönüşü engellemeyecek (takım kararı) |
| Tilt | 0–90° (otomatik modda 85° ile sınırlanır) | 1:5 (elle doğrulandı) | Hedef 30°, platformla 22–38° arasında gezer |
| CE | 0–4°, çalışma ortası 2° | 1:1 | Rapora göre. Mekanik dayanak biraz geniş bırakılabilir; yazılım sınırı parametre |

### Mikroadım

| Eksen | Hedef | Anten tarafında bir adım |
|---|---|---|
| Pan | 16 | 0,0225° |
| Tilt | 16 | 0,0225° |
| CE | 64 | 0,028° |

UART çalışırsa firmware'den ayarlanır ve MS pinleri boş kalır. UART çalışmazsa MS pinleriyle: pan ve tilt için MS1+MS2 → 3V3 (16) ya da boş (8, o da yeterli); CE için yalnız MS2 → 3V3 (64). UART yokken MS1/MS2 boş = 8 mikroadım.

---

## 3. Pin planı (geçerli)

| Cihaz | Sinyal | Nucleo pini |
|---|---|---|
| BNO085 | SCL, SDA | D15 (PB8), D14 (PB9) |
| Enkoderler, ortak | CLK, MISO, MOSI | D13 (PA5), D12 (PA6), D8 (PF3) — 26 Eyl: MOSI en alta alındı |
| Enkoderler, ayrı | CS pan, tilt, CE | D11 (PB5), D10 (PD14), D9 (PD15) — CN7'de yukarıdan aşağı yan yana |
| Pan sürücü | STEP, DIR, EN | D7 (PG12), **A0 (PA3)**, D0 (PB7) — 28 Eyl: DIR D6'dan A0'a (D6/PE9 ölü) |
| Tilt sürücü | STEP, DIR, EN | **D17 (PB15), D19 (PB12), D16 (PC6)** — CN7 iç sütun: D14, GND ve D15'in yanı (28 Eyl, GPIOE ölü) |
| CE sürücü | STEP, DIR, EN | **D21 (PC7), D24 (PA4), D20 (PA15)** — CN7 iç sütun: D12, D9 ve D13'ün yanı (28 Eyl) |
| Sürücü UART | pan, tilt, CE | D1 (PB6), D42 (PE8), D32 (PA0) — bağlı değil, kullanılmıyor |
| Lazer | LASER_EN | A1 (PC0) |
| LoRa E22 | TX, RX, M0, M1, AUX | PD5, PD6, PD3, PD4, PD7 (CN9 iç sütun: D53, D52, D55, D54, D51) |
| Yedek | boş | A3 (PB1), A5 (PF10), A6 (PF4), A7 (PF5) |
| Kart LED'leri | kalp atışı, arıza | PB0 (LD1), PB14 (LD3) |

Üç sürücünün bütün telleri CN10 soketindedir. Dış sütunda yukarıdan aşağı: D7, D6, D5, D4, D3, D2, D1, D0, D42, D41, GND, D40. D32 aynı soketin alt tarafında, iç sütundadır.

### Plan B: Teensy 4.1 pin planı (28 Eylül, Nucleo işlemcisi öldüğü için)

Kart USB yukarıda, üstten görünüş. Bütün sinyaller 3,3 V; Teensy pinleri 5 V toleranslı DEĞİL. TMC UART bağlanmaz.

| Teensy pini | Sinyal | Not |
|---|---|---|
| **Sol sütun** | | |
| GND | ortak toprak | 24 V kaynağın eksisi ve regülatör eksisi de buraya |
| 0–4 | boş | Serial1 (0/1) yedek kaldı |
| 5 | LASER_EN | MOSFET kapısı |
| 6 | enkoder SCK | yazılım SPI, üç enkodere ortak |
| 7 | enkoder MISO | ortak |
| 8 | CS pan | |
| 9 | CS tilt | |
| 12 | CS CE | 29 Eyl devir notu: CS10 yolunda okuma olmadı, CS12 çalıştı |
| 10 | boş | eski CE CS; yazılımda giriş |
| 11 | enkoder MOSI | ortak |
| 3.3V | sensör beslemesi | enkoder ×3, BNO085, TMC VIO; Teensy en fazla 250 mA verir |
| 24 (TX6) | LoRa RXD | Serial6 |
| 25 (RX6) | LoRa TXD | Serial6 |
| 26 | LoRa M0 | |
| 27 | LoRa M1 | |
| 28 | LoRa AUX | giriş |
| 29–32 | boş | |
| **Sağ sütun** | | |
| VIN | 5 V giriş | USB'den gelir; sahada 5 V regülatör buraya, o zaman USB takılıyken VUSB–VIN köprüsü kesilmeli |
| GND | toprak | |
| 3.3V | aynı 3,3 V rayı | |
| 23–20 | boş | Serial5 yedek |
| 19 | BNO085 SCL | Wire; pull-up BNO kartında |
| 18 | BNO085 SDA | Wire |
| 17–14 | boş | Serial3/Serial4 yedek |
| 13 | kart LED'i | kalp atışı |
| 41 | STEP pan | |
| 40 | DIR pan | |
| 39 | EN pan | aktif düşük |
| 38 | STEP tilt | |
| 37 | DIR tilt | |
| 36 | EN tilt | |
| 35 | STEP CE | |
| 34 | DIR CE | |
| 33 | EN CE | |

TMC UART pinleri bağlanmaz (hiç çalışmadı, gerekmiyor; STEP/DIR/EN + MS pinleri yeter). Yazılım: yalnız `hw.cpp` yeniden yazılır (IntervalTimer 20 kHz / 1 kHz, pin yazma, pad kenar hızı düşük), geri kalan kod aynı; PlatformIO ortamı `teensy41`. PC tarafında port USB kimliği 16C0 (Teensy) olarak da tanınır.

### Bağlantı kuralları

- AS5048A adaptöründe **5V ve 3.3V pinlerinin ikisi de 3V3'e** bağlanır. 5 V kaynağı kullanılmaz.
- Mıknatıs çapsal olmalı, milin ortasında, çipe 1–2 mm. Doğru değer: AGC 60–200, OCF 1. (Pan'da yuva derin olduğu için AGC 255 okunuyordu; 22 Eylül'de düzeltildi.)
- Sürücülerde VIO (3.3-TMC) → 3V3 ve **en az 3,0 V** ölçülmeli. GND ortak. 24 V → VM. 24 V kaynağın eksisi Nucleo GND ile ortak.
- EN ve UART her sürücüye **ayrı** teldir. MS1/MS2 üç sürücüde de boş (UART adresi 0).
- BNO085: VIN → 3V3, GND, SCL, SDA. RST ve INT bağlanmaz. 5 V verilmez. Kablo kısa, modül sabit.
- Motor kablosu güç varken takılıp çıkarılmaz.
- Uzun dupont kabloda bütün dijital çıkışlar düşük kenar hızında sürülür (yazılımda ayarlı).

---

## 4. Yazılım altyapısı

- PlatformIO + Arduino (STM32duino 3.0), kart `nucleo_h753zi`, yükleme `mbed` (E: sürücüsüne kopyalama), seri port COM8, 115200.
- Adım üretimi: TIM13 kesmesi 20 kHz, yazılımsal DDA, en fazla 10 000 adım/s. STEP/DIR düz GPIO.
- **Eksen katmanı** `src/ctrl/axis.c` (25 Eyl): eksen başına yamuk hız profili (vmax, amax parametre), yazılım sınırları (pan 0–360 = sınırsız ve kısa yoldan; tilt, CE sert sınır), hız modu (jog, sınıra yaklaşınca kendisi yavaşlar), A–B salınımı, "burası = X" (adım sayacı + enkoder sıfırı), isteğe bağlı kapalı çevrim (harekette komut–enkoder farkı eşiği aşarsa konum enkoderden alınır, 5 düzeltmede arıza; duruşta ölü bant dışı fark düzeltilir; varsayılan KAPALI). Son yaklaşım sayılı adımla, hedefe tam varır. Komutlar: GOTO 0x04, OSC 0x1D, AXIS_ZERO 0x1E, AXIS_INFO 0x1F; STOP rampalı, ENABLE-kapalı anında. Benzetimle (float32, 1 kHz) ve motorlarla doğrulandı.
- Kontrol döngüsü: TIM14 kesmesi 1 kHz. Enkoder SPI'ına yalnız bu kesme dokunur.
- Enkoder SPI: **yazılımsal (bit-bang)**, mod 1, yaklaşık 250 kHz. Donanım SPI uzun kabloda çerçeve hatası veriyordu.
- BNO085: I2C **100 kHz**, Game Rotation Vector, 100 Hz. Akış 300 ms kesilirse I2C çevre birimi sıfırlanır, yetmezse sensör yeniden başlatılır.
- Bekçi köpeği 3 s. Açılış mesajında reset sebebi yazılır.
- Protokol: `[0xA5][LEN][ID][yük][CRC16]`. Telemetri 84 bayt, USB'de 50 Hz.
- TMC2209: kütüphanesiz kendi UART sürücüsü, her sürücüye ayrı tek telli yarı çift yönlü hat. **Sınanmadı.** (Eski ortak hat kodunda çevre birimi/pin işlevi hatası vardı, düzeltildi.)

### Araçlar (`tools/`)

| Araç | İş |
|---|---|
| `mercom_gui.py` | Ana firmware arayüzü (seri port açılır listeden seçilir, ST-Link olan başta ve önseçili; liste açılınca yenilenir), iki sekme, takım renkleri (lacivert 24;30;54 zemin, turuncu 224;77;0 düğmeler, açık turuncu 255;119;1 başlıklar), açılışta tam ekran (25 Eyl gece 8). **Kontrol ve veri**: solda büyük nişan hatası (yeşil < 0,72° = Ø10, sarı < 1,43° = Ø20, kırmızı üstü), sağ-sol / yukarı-aşağı bileşenleri, Manuel mod · Hedefe kilitle · Otomatik takip durum düğmeleri (yanar/söner, tekrar basınca kapanır), lazer, üç sürücü, tilt'i IMU'dan ayarla; durum ışıkları (bağlantı, IMU, 3 enkoder, sürücüler); IMU roll/pitch/yaw; enkoder tablosu (ölçülen, istenen, fark, hız, mıknatıs sinyali, durum). Sağda eksen kartları: sürücü açık, istenen/ölçülen/fark/hız, açıya git, ±0,1/1/10° adım, basılı tutunca döner (◀ hız ▶), salınım A↔B + uçta bekleme, "şu an bulunduğu açı = X kabul et". **Ayarlar**: eksen başına 14 satır + otomatik takip ayarları; karttan oku / karta yaz / bilgisayara kaydet / bilgisayardan yükle → `tools/params.json`. Üst şerit: bağlan, kart süresi + lazer, mod ve kilit rozetleri, DUR (rampalı), ACİL DUR (sürücüler kapanır). Alt: kart mesajları + CSV kaydı |
| `mercom_link.py` | Komut satırı: ping, en, jog, move, encdiag, imudiag, tmcdiag, tmctx, tmcraw, tmcloop, reset … |
| `kilit_testi.bat` / `kilit_testi.py` | Kilit-otomatik testi: başlığı 30°'ye alır, kilitler, otomatik moda geçer, 60 s nişan hatasını yazar ve `kilit_testi_son.log`'a kaydeder. Arayüz kapalıyken çift tıkla; "OTOMATIK MOD ACIK" yazınca tabanı yavaş eğ |
| `system_check.py` | Tek komutla tam sistem denetimi (enkoder, IMU, UART, motor, lazer). `--sensor` yalnız enkoder + IMU, `--motorsuz` motorsuz. Donanımda çalışıyor (21 Eylül) |
| `yaw_drift_test.py` | 5 dakikalık yaw kayması ve kesinti kaydı |
| `ratio_test.py` | Enkoder + IMU ile redüktör oranı ölçümü |
| `pan_enc_term/` | Terminalden motor + enkoder kapalı çevrim test kodu, `terminal_gui.py` renkli terminal |
| `motor_term/` | Terminalden üç eksenli motor test kodu (yalnız STEP/DIR/EN). Üç eksen aynı anda hareket edebilir: `hepsi <d>`, `hepsikonum <d>`, `hepsisalin` (her eksen kendi limitinde). Mikroadım eksen başına (`mikro <n>`). CE limiti masa testi için geçici ±45 (yarışma değeri 0–4°). Hız ve ivme ortak, ama eksen başına tavan var (`tavan <hiz> <ivme>`): CE redüktörsüz olduğu için 60 °/s ve 300 °/s² ile sınırlı, `hiz 200 ivme 4000` ile CE motoru kilitlendi (21 Eylül) |
| `enc_aci/` | Üç enkoderin açısını terminale yazan test kodu (motor mil açısı 0–360, tur sayılmış anten açısı, AGC); `aci_terminal.bat` ile açılır; komutlar `sifir`, `hiz <n>` |
| `led_blink/` | Kart sağlık sınaması: üç LED sırayla, seri sayaç; hiçbir dış pine dokunmaz |
| `lazer_test/` | Lazer sınaması (26 Eyl): PC0 (A1) MOSFET kapısını 2 s açık / 1 s kapalı sürer, yeşil LED aynı anda yanar; seri porttan '1' sürekli açık, '0' kapalı, 'b' yanıp sönme. SWD ile yüklenir (`upload_protocol = stlink`) |
| `pin_test/` | CE sürücü hattı pin sınaması (27 Eyl): D3 STEP, D2 DIR, D40 EN sırayla 4 s yüksek, her biri kendi LED'iyle (yeşil/sarı/kırmızı); multimetreyle sokette doğrulama |
| `vdd_test/` | Kart sınaması (28 Eyl): LED'ler sırayla, VDDA (iç referans VREFINT) ve çekirdek sıcaklığı saniyede bir seri porta |
| `kart_test/` | **Kart yük ve arıza sınaması** (28 Eyl): %100 işlemci yükü (256 KB RAM yaz/oku, 96 KB flash CRC32, kayan nokta doğrulama), VDDA + sıcaklık + en düşük/en yüksek, sıfırlama sebebi (RCC_RSR), önceki HardFault kaydı (DTCM'de), PVD 2,85 V çöküş sayacı, IWDG 4 s. İlk koşu: 3,26 V, 65 °C yük altında, 0 hata |
| `teensy_pintest/` | Teensy 4.1 pin sınaması (29 Eyl): plandaki 17 çıkışı sırayla 1,5 s yüksek yapar ve geri okur, üç enkoderi ham okur, I2C tarar; seri porta yazar. İlk koşu: bütün pinler doğru sürülüyor ve geri okunuyor |
| `seri_izle.bat` / `seri_izle.py` | Kartın seri çıkışını canlı gösteren pencere; ST-Link portunu kendisi bulur, kopunca yeniden bağlanır |
| `kart_voltaji.bat` | Kartın 3V3 hattını ST-Link üzerinden saniyede bir ölçer (OpenOCD). Kart susarsa ilk bakılacak yer |
| `motor_term/motor_kontrol.py` | Üç motoru sırayla küçük ileri-geri oynatır (motor_term yüklüyken) |
| `motor_basic/`, `enc_basic/`, `bno_basic/` | Bağımsız basit test kodları (yüklenince ana firmware gider) |
| `kin_proto.py` | Kinematik formüllerin Python doğrulaması |
| `kazanc_testi.py` | Takip kazancı testi (30 Eyl): `python kazanc_testi.py [K1 K2 ...]`; her kp için duruş titremesi, bozmadan Ø20/Ø10'a giriş süresi, hedefi aşma; sonunda kp 8'e döner |
| `duzeltme_testi.py` | 29 Eyl düzeltmeleri testi: sürücü bitleri, sürücü kapalıyken otomatiğin beklemesi, kapalı çevrim (sahte düzeltme yok, 3° kaçırma benzetimi düzeltiliyor) |
| `bno_hareket_testi.py` | En kötü durum IMU reset testi (29 Eyl): `python bno_hareket_testi.py [saniye] [pan_genlik] [tilt_genlik]`; üç eksen sürekli salınırken sensör reseti, I2C hatası, IMU hızı, bayat örnek ve enkoder okuma hızı; sonunda eksenleri başlangıca döndürür |
| `bno_teensy/` | Basit BNO085 testi, Teensy 4.1 (29 Eyl): orijinal Adafruit kütüphanesiyle ana firmware'in raporlarını okur, sürücü ve lazer kapalı; seri komut '1' / '0' ile 354 µs'lik kesme benzerini açar/kapar. `izle.py [evre_s]` evreleri sürer ve reset sayar. Yükleme: `pio run -d tools/bno_teensy -e teensy41 -t upload` (sonra ana firmware geri yüklenmeli) |
| `lora_koprusu/` | LoRa yer istasyonu köprüsü (Teensy 4.1 seri no 18913180 + E22 Serial4, M0 37, M1 39, AUX 41): USB ↔ LoRa, giden paketlere terminal adresi + kanal. USB portu 9601 baud = E22 ayar modu, 9602 = öz sınama, 9603 = bütün seri portları tarama |
| `lora_ayar.py` | İki E22'nin ayarını okur/çözer; `yaz` ile MERGEN ayarlarını yazar (sabit nokta, 4D01/4D02, kanal 15, 9,6 kbps, şifre) |
| `gui_lora_test.py` | Arayüzün LoRa üzerinden uçtan uca testi (30 Eyl): gerçek arayüzü yer istasyonuna bağlar, 33 düğme işlevini sırayla çalıştırıp etkisini doğrular, ekrandaki verilerin değişme sayısını ve telemetri hızını yazar. Motorlar birkaç derece oynar, lazer yanıp söner |
| `yarisma_oncesi_test.py` | Yarışma öncesi tam sistem testi (30 Eyl, ~5 dk, taban sabit, USB): bağlantı, parametreler, IMU, enkoder, sürücü/lazer, eksen hareketi ve kapsamı, hareketli IMU, toparlanma, IMU yeniden başlama, sürücü kapalıyken bekleme, LoRa, kalıcılık. Sonuç `yarisma_oncesi_test_sonuc.txt` |
| `kalicilik_testi.py` | Parametre kalıcılığı testi (30 Eyl): kaydet, yeniden başlat, hareket hâlinde kayıt, kaydı sil; sonunda yarışma değerlerini kalıcı kaydeder |
| `lora_test.py` | LoRa uçtan uca testi: yer istasyonundan PING, telemetri sayısı, lazer kapat/aç |
| `teensy_yukle.py` | İki Teensy takılıyken doğru karta yükleme: `python teensy_yukle.py <seri_no> <firmware.hex>` (terminal 15425680, köprü 18913180); portu 134 baud ile açıp yalnız o kartı yükleme moduna alır |
| `bno_i2c_izle.py` | I2C hata sayaçlarını ve sensör resetlerini saniyede bir okur, değişenleri zamanıyla yazar (29 Eyl): `python bno_i2c_izle.py [saniye]` |
| `bno_ab_test.py` | IMU okuma aralığı deneyi (29 Eyl): `python bno_ab_test.py <saniye> <deger1> <deger2> ...` (`imu_poll_us` değerleri, dönüşümlü); her blokta sensör reseti, akış kesilmesi, rapor/s, bayat örnek oranı |
| `bno_reset_izle.py` | BNO085'in kendi bildirdiği reset sebebini sayar (29 Eyl): `python bno_reset_izle.py [saniye] [motor]`; güç açılışı / iç sistem reseti / bekçi köpeği / RST pini ayrı ayrı |
| `baglama_testi.py`, `baglama_ayrinti.py` | Yön bağlama testi (29 Eyl): `python baglama_testi.py <yaw_anchor 0\|1>`; kilit, başlık 5–8° uzaklaştırılır, IMU bilerek yeniden başlatılır, otomatikte gerçek hata enkoderden ölçülür. `baglama_ayrinti.py` aynı testin mesaj ve telemetri dökümü |
| `bozma_testi.py` | Toparlanma testi (29 Eyl): kilit alır, manuelde başlığı pan/tilt 5–10° uzaklaştırır, otomatiğe geçip lekenin Ø20 / Ø10 içine dönme süresini ölçer. Hata hem IMU'dan hem enkoderden (gerçek) yazılır; ikisi ayrışıyorsa IMU referansı kaymıştır. Kayıt `bozma_kayit.csv`. Taban sabit durmalı |
| `takip_kur.py`, `takip_kayit.py`, `takip_olay.py`, `takip_ozet.py` | Otomatik takip testi: kurulum (`takip_kur.py <plx_en> [ly] [ht] [d]`), canlı CSV kaydı, olay dökümü ve özet |
| `montaj_olc.py` | IMU montaj eğimini cetvel ölçüsünden hesaplar (29 Eyl): `python montaj_olc.py <lazer_cm> <nokta_cm> <mesafe_cm> [uygula]`. Taban düz ve hareketsizken lazerin çıkış ucunun ve duvardaki noktanın zeminden yüksekliği ile yatay mesafe verilir; doğru `imu_mount_p` yazdırılır, `uygula` ile karta yazılır |
| `takip_1_kapali.csv`, `takip_2_montaj58.csv`, `takip_kayit.csv` | 29 Eyl elle yatırma kayıtları (paralaks kapalı; montaj eğimi −4,746 / +5,8 / +2,0) |

---

## 5. Kontrol algoritması (kararlar)

**Yaklaşım: doğrudan başlık stabilizasyonu.** IMU anten başlığında durur (gemi VSAT antenleri ve kamera gimballeri gibi). Taban yönelimi hesaplanmaz; başlığın dünyadaki yönü doğrudan hedefte tutulur. Redüktör boşluğu ve esneme döngünün içinde düzelir.

1. **Eksen katmanı.** Her eksen hedef açı ya da hız alır; ivmeli profille gider, sınırları uygular, enkoderle adım kaçırmayı yakalar. Harekette fark 3 tam adımı geçerse gerçek konum enkoderden alınır ve düşük hızdan yeniden kalkılır; üst üste 5 takılmada hareket iptal edilir. Duruşta ölü bant (0,15°) dışındaki fark düzeltilir. Mantık `pan_enc_term` içinde yazılı; ana firmware'e taşınacak.
2. **Nişan katmanı.** "Kilitle" anında başlığın baktığı yön dünya çerçevesinde saklanır. Her IMU örneğinde (100 Hz) şu anki yön ile kayıtlı yön arasındaki hata bulunur; elevasyon hatası tilt'e, yanal hata pan ve CE'ye hız komutu olarak verilir. Jiroskoptan ileri besleme eklenir.
3. **CE kullanımı: kaba-ince paylaşım.** Yanal hatanın küçük ve hızlı kısmını boşluksuz CE düzeltir; pan platformun büyük hareketini karşılar ve CE'yi sürekli 2° ortasına geri çeker.
4. **Paralaks telafisi.** Hedef 4 m'de bir nokta olduğu için platform eğilince antenin yer değiştirmesi hesaba katılır. Parametreler: terminal alt yüzeyi → tilt ekseni yüksekliği ve tilt ekseni → lazer ucu mesafesi. **Ölçüler bekleniyor; varsayılan 30 cm ve 10 cm.** Sahada arayüzden ayarlanır.
5. **Durum makinesi.** Manuel · Kilitle · Otomatik · Arıza. Kilit yönü mod değişse de silinmez; yalnız "bırak" ya da yeni "kilitle" ile değişir. 8 saniye şartı bu sayede kendiliğinden sağlanır.

### Referanslar

| Referans | Nasıl |
|---|---|
| Hedef yönü | Operatör lazeri hedefe getirir, "Kilitle"ye basar. Kuzey ya da koordinat gerekmez. Kilit platform hareket etmeden önce alınır |
| Tilt sıfırı | Platform duruyorken IMU yerçekiminden otomatik |
| CE sıfırı | Enkoder açıyı mutlak verir; ortası bir kez kalibre edilip kalıcı belleğe yazılır |
| Pan | Sınırsız, kısa yoldan. Açı 0–360 sarmalı gösterilir. Arayüzde "toplam dönüş" sayacı; isteğe bağlı sınır parametresi (varsayılan kapalı) |

IMU'da pusula yok: yaw referansı sensör çalıştığı sürece geçerlidir. Güç kesilince ya da sensör resetlenince yeniden nişan almak gerekir.

### Sınır ve arıza davranışı

- Sınıra gelen eksen yavaşlayarak durur ve tutar; arızaya geçilmez, kilit silinmez. Diğer eksenler çalışır. Hedef yeniden erişilebilir olunca takip kendiliğinden sürer.
- Hata eşiği (0,5°) aşılınca lazer söner.
- Mekanik dayanağa vurulursa enkoder takılmayı görür, hareket iptal edilir.

### 8 saniye testi için iki yol

- **Arayüzden:** "Takibi durdur" → başka açıya gönder → "Otomatik".
- **Elle:** eksen başına "serbest bırak" (ayrı EN pinleri sayesinde) → anten çevrilir → tekrar etkinleştirilince hedefe döner.

---

## 6. Haberleşme

- Geliştirme ve ayar: USB (ST-LINK sanal seri port).
- Yarışma: yer istasyonu bilgisayarda, terminalle **LoRa (E22-900T22D)** üzerinden. Protokol aynı; telsizde kısa ve seyrek telemetri (yaklaşık 30 bayt, 5–10 Hz).

**Terminalden yere:** zaman, mod ve durum bayrakları, pan/tilt/CE açıları, hedef açılar, işaretleme hatası, IMU roll/pitch/yaw, hata kodu, paket sayacı.
**Yerden terminale:** mod seç, açıya git, jog, kilitle/bırak, takibi durdur, eksen serbest bırak, limit yaz, lazer aç/kapat, yaw sıfırla, hata temizle, parametre oku/yaz.

### Yer istasyonu arayüzü (23 Eylül kararı iki sekme; 25 Eylül gece 8 son hâli)

Takım renkleri: zemin lacivert RGB 24;30;54 (#181e36), düğme turuncusu RGB 224;77;0 (#e04d00), açık turuncu RGB 255;119;1 (#ff7701) başlıklarda, seçili sekmede, düğme üstüne gelince ve hareket hâlindeki hız değerinde. Yeşil = iyi / otomatik, sarı = dur / uyarı, kırmızı = acil / hata. Açılışta tam ekran.

| Bölüm | İçerik |
|---|---|
| Üst şerit (her sekmede) | Seri port + Bağlan; rozetler: kart süresi ve lazer durumu, mod (MANUEL MOD turuncu / OTOMATİK TAKİP yeşil), HEDEF KİLİTLİ; sağda **DUR: hareketi durdur** (rampalı, mod değişmez) ve **ACİL DUR: motorları bırak** (üç sürücü kapanır) |
| **Kontrol ve veri** — sol | HEDEF TAKİBİ: büyük nişan hatası (yeşil Ø10 içinde < 0,72°, sarı Ø20 içinde < 1,43°, kırmızı dışında), sağ-sol / yukarı-aşağı bileşenleri, üç durum düğmesi **Manuel mod · Hedefe kilitle · Otomatik takip**: açıkken kendi rengiyle yanar (turkuaz / turuncu / yeşil), tekrar basınca kapanır (kilit → kilit bırakılır, otomatikteyse manuele döner; otomatik → manuel mod; manuel kapatılamaz), lazer, üç sürücü, tilt açısını IMU'dan ayarla. DURUM IŞIKLARI: kart bağlantısı, IMU, enkoder ×3, sürücüler. IMU: roll/pitch/yaw + veri hızı + yeniden başlat. ENKODERLER: ölçülen, istenen, fark, hız, mıknatıs sinyali (AGC), durum; kart döngü süresi ve paket sayacı |
| **Kontrol ve veri** — sağ | PAN / TILT / CE kartları: sürücü açık (motor tutar), istenen / ölçülen / fark / hız, açıya git, adım at ±0,1 / 1 / 10°, basılı tutunca döner (◀ hız ▶), salınım A ↔ B + uçta bekleme, "şu an bulunduğu açı = X, kabul et" |
| **Ayarlar** | Eksen başına 14 satır (hız, ivme, sınırlar, mikroadım, oran, motor/enkoder yönü, kapalı çevrim, kaçırma eşiği, duruş ölü bandı, Kp, Ki, dişli boşluğu) ve otomatik takip ayarları (yön işaretleri, hız tavanı, integral sınırı, ileri besleme tilt/pan, ölü bant). Karttan oku / karta yaz (kalıcı kayıt dahil) / bilgisayara kaydet / bilgisayardan yükle ve karta yaz (`tools/params.json`) / kalıcı kaydı sil |
| Alt | Kart mesajları (renkli: kilit yeşil, takılma kırmızı) ve CSV veri kaydı |

Kaldırılanlar (25 Eyl: gereksiz düğmeler): Ping, arızayı temizle (arıza durumu yok), yaw sıfırla, IMU tanı, eksen bilgisi, eksen başına "dur" ve "serbest bırak" (sürücü kutusunu kapatmak aynı iş). "Kilidi bırak" ayrı düğme olarak kaldırıldı: kilit düğmesine tekrar basmak kilidi bırakır. Planlanan 60 s hata grafiği yazılmadı.

---

## 7. Ölçülmüş değerler

| Ölçüm | Sonuç |
|---|---|
| BNO085 yaw kayması (kesintisiz bölümlerde) | 0,005–0,05 °/dk; 5 dakikada 0,05–0,25° |
| BNO085 gürültüsü | yaklaşık 0,01° |
| BNO085 reset | 11 Eylül: 5 dakikada 1–8 kez, her resette 0,3–0,5° sıçrama. **24 Eylül: 5 dakikada 0 reset, 100 rapor/s kesintisiz** (VCC IOREF'ten, ayrı GND teli; masada hareketsiz). Yaw kayması 0,001 °/dk, gürültü 0,008°. Hareketli ve kalıcı kabloyla tekrar sınanacak |
| Tilt enkoderi | AGC 158–168, 1 kHz'de hata 0 (iki besleme pini bağlıyken). 24 Eylül tam sistem: TAMAM |
| CE enkoderi | 24 Eylül: AGC 67 kararlı, hata 0, TAMAM (mıknatıs biraz yakın, 100 civarı daha rahat olur) |
| Pan enkoderi | 24 Eylül: AGC 172–183 kararlı, hata 0, TAMAM (yuva derinliği 22 Eylül'de düzeltildi) |
| Pan redüktör oranı | 5 (motorla, `git 90` → anten 90°). Eski elle ölçülen 8 değeri yanlıştı |
| Pan motoru | 8 mikroadımda 360 °/s ve 2000 °/s² ile döndü; adım kaçırma testi yapılmadı |
| Kontrol döngüsü | yaklaşık 136 µs / 1000 µs |

### Hata bütçesi (Ø10 çember payı 0,72°)

IMU gürültüsü ve kayma 0,05–0,25° · takip gecikmesi (ileri beslemeyle) ≈ 0,1° · adım çözünürlüğü 0,02–0,05° · paralaks telafisiz 0,8°, telafiyle ≈ 0,15° · BNO reseti başına 0,3–0,5°.

---

## 8. Açık konular ve riskler

1. ~~BNO085 kendini resetliyor~~ **Çözüldü (25 Eylül): modüldeki çıkmış lehim.** Düzeltme sonrası motorlu testte 0 reset. Kalıcı kablolamada 10 µF kondansatör ve kablo gerginlik önleyici yine önerilir; hareket hâlinde 5 dk testi yarışma düzeneğinde tekrarlanacak.
2. **Ayrı UART hatları sınandı, yeni EN pinleri çalışıyor** (tilt 21 Eylül'de EN D41 ile kusursuz döndü). Sürücüler UART'ta hâlâ cevapsız; motorlar STEP/DIR ile dönüyor.
3. **CE sürücüsü duruşta 1,1 A çekiyor** (pan+tilt birlikte 0,4 A). Vref büyük ihtimalle sonuna kadar açık; ölçülüp 1,25 V'a alınacak. Düzelene kadar motor_term CE'yi serbest başlatıyor.
4. **3V3 hattı çökmeleri (21–22 Eylül):** enkoder fişlerinde ters/yanlış takılan besleme telleri kartın 3V3'ünü sıfıra çekti, kart sustu, mbed yüklemesi FAIL.TXT verdi. Çözüm: telleri tek tek takıp `kart_voltaji.bat` ile izlemek. Kalıcı kablolamada fişler tek yönlü ya da işaretli olacak.
5. ~~Pan 90° komutunda eksik dönüyor~~ Çözüldü: pan toplam oranı 10 (25 Eylül). Mekanik sebep: redüktör sonrası 2:1 kademe.
6. ~~CE mıknatıs yuvası~~ Üç enkoder de 24 Eylül'de TAMAM (AGC 175 / 158 / 67). Belirti dersi: haberleşme temiz ama AGC 255 ve açı rastgele = adaptörün bir besleme pini boşta (üç modülde birden görülürse ortak dağıtım teli).
7. **Üç motor da STEP/DIR ile döndü** (21 Eylül); 25 Eylül tam denetimde üçü de enkoderle doğrulandı. CE titremesi ve akımı için madde 3.
8. **Paralaks ölçüleri** (iki cetvel ölçüsü) bekleniyor.
9. **Besleme:** 24 → 5 V dönüştürücü (en az 3 A) ve motor etiket akımı bilgisi.
10. **Yazılmayanlar:** CE paylaşımı, paralaks, LoRa köprüsü. Eksen katmanı ve arayüz ayar sekmesi 25 Eylül'de yazıldı.
11. **Kablolama:** yarışma düzeneğinde dupont yerine lehimli ya da kilitli konnektör.
12. **CE ekseni başlığı tutuyor mu?** (25 Eylül) Pan küçük hareketlerde antene ulaşmıyor gibi; elle test: üç sürücü açıkken başlığı pan yönünde çevirmeye çalış, dönüyorsa CE tutmuyor (8 mikroadım, Vref, mekanik). Çözülmeden Ø10 gerçekçi değil.

---

## 9. Takvim

| Gün | İş |
|---|---|
| 22 Eylül | Donanımı bitir, `system_check.py` ile hepsini doğrula; eksen katmanına başla |
| 23 Eylül | Eksen katmanı üç eksende, sınırlar, arayüzden "açıya git" (manuel mod) |
| 24 Eylül | Nişan katmanı: kilitle, hata hesabı, otomatik mod; tabanı elle eğerek ilk deneme |
| 25 Eylül | Jiroskop ileri beslemesi, CE paylaşımı, ayar; 8 s ve 5 dk testleri masada |
| 26 Eylül | Parametrelerin kalıcı kaydı, arayüz alanları, arıza davranışı |
| 27 Eylül | LoRa |
| 28 Eylül | Kalıcı kablolama, tam sistem testi |
| 29 Eylül | Senaryo provası ve kayıt |

Yarışma günü akışı: anteni hedefe çevir → lazeri ortaya getir → "Kilitle" → platform hareket eder, sistem tutar.

### 9.1 Kullanım sırası (29 Eylül, Teensy, bugünkü bulgulara göre)

**Alanda bir kez (kurulum)**
1. Terminali platforma sağlam bağla. Sistem yalnız dönmeyi ölçer; taban 1 cm kayarsa leke de 1 cm kayar.
2. Pan kabloları 360° dönmeye izin veriyor; terminal platforma hangi yönde bağlanırsa bağlansın pan hedefe en kısa yoldan döner.
3. Ölç: lazer ucundan hedef merkezine mesafe (`plx_d`, 4 m ileri + 2 m yukarı = 4,47 m) ve platformun dönme merkezinden tilt eksenine yükseklik (`plx_ht`, bilinmiyorsa 0,166).
4. Yarışma ayarları (30 Eyl) kartın varsayılanı ve `tools/params.json`'da: CE 0–4, tilt −90…+90, paralaks açık, `plx_d` 4,00 (şartname: 4 m eğik mesafe, 30°), `plx_ht` 0,166, ölü bant 0,3°. Mesafe farklıysa: Arayüz > Ayarlar > "Karttan oku" → `plx_d` → "Karta yaz" → "Bilgisayara kaydet". Kart yeniden başlarsa arayüz params.json'u otomatik yazar.

**Her denemeden önce**
1. 24 V ve USB açık, `tools/arayuz.bat`, port "COM10 Teensy 4.1", Bağlan.
2. Durum ışıkları yeşil, IMU kutusunda "veri geliyor · ~90 Hz". Açılıştan sonra 5 s bekle.
3. **Önce "Üç sürücü açık"**, sonra nişan: sürücü açılınca motorlar oturur (CE 1:1, ±3,6°'ye kadar), sonra açarsan leke kayar.
4. Platform **düz ve hareketsiz**. Lazeri aç, lekeyi hedef merkezine arayüzün ±1° / ±0,1° düğmeleriyle getir (başlığı elle çevirme).
5. "Hedefe kilitle" → "Otomatik takip". 5–10 s lekenin yerinde durduğunu izle.

**Takip sırasında**
- Dokunma. Nişan hatası göstergesi: 4,47 m'de Ø10 sınırı 0,64°, Ø20 sınırı 1,28°.
- Kayıt alanında "imu: rapor akisi kesildi" / "yon yeniden baglandi" / "KART YENIDEN BASLADI" görülmemeli (29 Eyl düzeltmesinden sonra 5 dk hareketli testte 0).

**LoRa ile (kablosuz) kullanım**
- Yer istasyonu Teensy'sini bilgisayara tak; arayüzde port listesinden **"LoRa yer istasyonu (kablosuz)"** seç, Bağlan. Telemetri saniyede 1 gelir, bütün komutlar (sürücü, motor, lazer, kilit, mod) çalışır, cevap ~1 s.
- USB kablosu da takılı kalabilir; iki bağlantı aynı anda çalışır. Arayüz tek bağlantı açar: hangisini seçersen.
- İki Teensy takılıyken firmware yüklemesi `tools/teensy_yukle.py 15425680 .pio/build/teensy41/firmware.hex` ile yapılır (düz `pio run -t upload` yanlış karta gidebilir).

**Sorun olursa**
- Leke kaydıysa: Manuel mod → düğmelerle düzelt (platform düzken) → Kilitle → Otomatik.
- Kart yeniden başladıysa: ayarlar params.json'dan otomatik yazılır; sürücüleri aç → nişan → Kilitle → Otomatik.
- IMU verisi yoksa: "IMU'yu yeniden başlat".

**Yapma**
- Sürücüler kapalıyken nişan alıp sonra sürücüleri açma.
- Kilidi platform eğik ya da hareketliyken alma (paralaks hedef noktası kilit anındaki duruma göre hesaplanır).
- Tabanı kendi ekseninde çevirme (yön bağlama tabanın dönmediğini varsayar; yarışma platformu yalnız eğilir).
- `i2c_sync`, `imu_poll_us`, `enc_read`, `yaw_anchor` ayarlarını değiştirme (IMU reset çözümü bunlara bağlı).

---
