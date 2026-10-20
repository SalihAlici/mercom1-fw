# MERCOM-1 yazılımı: amaç, görevler ve çalışma ilkesi

Takım: MERGEN · Terminal: MERCOM-1 · Yarışma: TEKNOFEST Hareketli Uydu Terminali (SoTM) · 30 Eylül 2026

## 1. Yarışma ve istekler

Yarışmada terminal, hareket hâlindeki bir aracı taklit eden 6 eksenli Stewart platformunun üstüne konur. Platform tabanı roll ve pitch eksenlerinde yaklaşık ±8° eğer, bir salınımın süresi yaklaşık 10 saniyedir; öteleme yoktur. Terminalin üstündeki lazer, 4 metre uzaktaki hedef levhaya sürekli nişan almalıdır. Puanlama levhadaki iç içe dairelere göredir: Ø10 cm daire tam puan, Ø20 cm daire kısmi puan. Ölçüm 5 dakika sürer.

Hakem iki şekilde bozma yapar: anteni elle başka yöne çevirir ya da arayüzden başka bir açıya gönderir. Terminal 8 saniye içinde hedefe geri dönmelidir. İki mod istenir: manuel (operatör arayüzden açı verir) ve otomatik (terminal sensörleriyle hedefte kalır). Takım kararıyla GPS ve uydu (Türksat) görevi kapsam dışı bırakıldı; yalnız manuel ve otomatik yönelim vardır.

Mekanik: pan (sağ-sol, redüktörlü, 10:1), tilt (yukarı-aşağı, redüktörlü, 5:1) ve CE (ince eksen, 1:1, 0–4° aralığında, pan yönünde) olmak üzere üç adım motoru; her eksende bir AS5048A manyetik enkoder; başlıkta (lazerle birlikte hareket eden brakette) bir BNO085 IMU; lazer; yer istasyonuyla haberleşme için LoRa E22.

Yazılım sınırları takım kararıdır: pan 0–360° (sınırsız, kısa yoldan döner), tilt 0–90°, CE 0–4°.

## 2. Yazılımın amacı

Yazılımın tek amacı lazeri hedefte tutmaktır. Bunu üç kabiliyetle yapar:

1. **Manuel yönelim:** operatörün verdiği açıya her ekseni düzgün bir hız profiliyle götürmek, sınırların dışına çıkmamak, motorları kaçırmadan sürmek.
2. **Hedef kilidi:** operatör lazeri hedefe getirip "kilitle" dediğinde, başlığın o anki bakış yönünü dünya çerçevesinde (yerçekimine ve yaw'a göre) hafızaya almak.
3. **Otomatik takip:** taban ne kadar eğilirse eğilsin, anten elle çevrilse bile, başlığın bakış yönünü kilitlenen yöne geri getirip orada tutmak; bunu titremeden, 8 saniyeden çok daha kısa sürede ve hiç vazgeçmeden yapmak.

Bunun yanında arayüze anlık veri (açılar, IMU, enkoder sağlığı, nişan hatası) göndermek, ayarların çalışma anında değiştirilmesine izin vermek ve her durumu metin mesajıyla bildirmek de yazılımın görevidir.

## 3. Denetim yaklaşımı: PID var mı, neye göre?

Otomatik modda klasik bir PID değil, **ölü bantlı PI + ileri besleme** kullanılır. Nedenleri ve işleyişi:

**Hata nasıl ölçülür.** IMU'dan gelen kuaterniyon, montaj kalibrasyonuyla başlık çerçevesine çevrilir; buradan başlığın bakış vektörü elde edilir. Kilitlenen bakış vektörü ile şu anki bakış vektörünün vektörel çarpımı, açı hatasını yön bilgisiyle verir. Bu hata başlık çerçevesine geri çevrilip iki bileşene ayrılır: **yanal** hata (pan'ın kapatacağı) ve **dikey** hata (tilt'in kapatacağı). Yani denetim, enkoder açılarına değil, doğrudan "lazer hedeften kaç derece sapmış"a göre yapılır. Enkoderler bu döngüde kullanılmaz; açı sınırları, konum sayacı ve adım kaçırma kontrolü için kullanılır.

**P terimi.** Her bileşen için hız komutu = Kp × hata. Kp = 8 1/s: 1° hata 8 °/s düzeltme hızı ister. Büyük hatada hız 45 °/s tavanına oturur (aim_vmax).

**I terimi.** Ki = 2 1/s². Sürtünme ve dişli boşluğu yüzünden kalan küçük kalıcı hatayı zamanla siler. Integral, birikim sınırıyla (aim_imax 2°) ve çıkış doyumdayken durdurularak (anti-windup) kontrol altında tutulur.

**D terimi kullanılmaz.** Türev teriminin görevi hatanın değişim hızını öngörmektir; bizde bu bilgi doğrudan **jiroskoptan** gelir. Tabanın dönüş hızı ölçülür, başlığın kendi dönüş hızı çıkarılır, kalan "taban hızı" ileri besleme olarak komuta eklenir (tilt'te 0,7 katsayı). Böylece taban eğilmeye başladığı anda motor karşı yönde hareket eder, hata büyümeden dengelenir; türevin sağlayacağı öngörü, gürültü büyütmeden elde edilir. Pan'da ileri besleme 0'dır: pan redüktöründe 0,7° dişli boşluğu var, ileri besleme bu boşlukta salınıma yol açtı (ölçüldü).

**Ölü bant.** 0,3°'nin altındaki hata yok sayılır (hatanın yalnız aşan kısmı düzeltilir). Sebep: dişli boşluğunda motor küçük hatayı kovalarken sürekli ileri geri titriyordu. 0,3° hedefte 2 cm'ye denk gelir, Ø10 dairenin içindedir.

**Geometri düzeltmesi.** Pan komutu cos(yükseliş) ile bölünür: başlık yukarı baktıkça yanal 1° için pan daha çok dönmelidir.

**Koruma kuralları.** IMU verisi 60 ms'den uzun kesilirse ya da başlık zenitin ötesine geçmişse motorlar durdurulur, veri gelince devam edilir. Otomatik mod hiçbir hata büyüklüğünde kendi kendine manuel'e düşmez; hakem anteni 90° çevirse de geri gelir (takım kararı).

**Hız komutları nereye gider.** PI çıkışı doğrudan motora değil, eksen katmanının hız moduna verilir; orada ivme sınırı (amax) uygulanır. Böylece komut ani değişse bile motor adım kaçırmaz. Bu döngü IMU hızında (100 Hz) çalışır; eksen katmanı 1 kHz, adım üretimi 20 kHz'dir.

Ölçülen sonuç: sabit tabanda hata 0,02–0,15°, bozma sonrası toparlama 0,5 s civarı (istek 8 s).

## 4. Manuel mod ve eksen katmanı

Her eksen için yamuk hız profili kullanılır: ivmeyle hızlanma, sabit hız, ivmeyle yavaşlama. Kalkış hızı 12 adım/s'dir ki motor durgun hâlden takılmadan kalksın. Hedef açı sınırların dışındaysa sınıra kırpılır; pan için sınır yoktur, hedefe her iki yönden kısa olan yoldan gidilir. Jog (basılı tutunca döner) aynı profilin hız modudur. Salınım (A↔B, uçlarda bekleme) motor ve mekanik testleri içindir. "Burası = X" komutu adım sayacını ve enkoder sıfırını birlikte ayarlar.

Adım üreteci DDA yöntemiyle çalışır: 20 kHz'de her eksen için bir birikim sayacına hız eklenir, taşma olduğunda bir STEP darbesi çıkar. Bu sayede 0,3 °/s'den 200 °/s'ye kadar hız kesintisiz üretilir ve herhangi bir GPIO pini kullanılabilir.

## 5. Sensörler

**Enkoderler (AS5048A, 14 bit):** yazılımsal SPI ile 1 kHz'de okunur; parite kontrolü, bir milisaniyede 2,6°'den fazla değişen okumanın reddi (sıçrama), mıknatıs sağlığı (AGC 60–200 iyi, 255 uzak), çok turlu açı için tur sayacı. Bir saniye kesintisiz sağlıklı okuma olmadan enkodere güvenilmez. Redüktör oranı ve yön işaretiyle anten açısına çevrilir.

**IMU (BNO085):** oyun dönüş vektörü (manyetometresiz, yaw kayması çok düşük: 5 dakikada 0,05–0,25°) 100 Hz, kalibre jiroskop 100 Hz. Montaj kalibrasyonu üç açıyla yapılır ve motor sürülerek ölçülmüştür. Rapor akışı kesilirse yazılım sensörü yeniden başlatır.

## 6. Haberleşme, arayüz ve parametreler

Kart ile PC arasında çerçeveli ikili protokol vardır (başlangıç baytı, kimlik, uzunluk, veri, CRC). Kart 50 Hz'de 96 baytlık telemetri gönderir: mod, bayraklar, IMU açıları, üç eksen için komut/enkoder/hız, nişan hatası ve bileşenleri, döngü süresi ve paket sayaçları. Komutlar: mod, kilitle/bırak, git, jog, dur, sürücü aç/kapat, salınım, sıfırla, parametre oku/yaz, tanı. Aynı protokol LoRa üstünden de taşınabilir.

Arayüz iki sekmedir: "Kontrol ve veri" (nişan hatası göstergesi Ø10/Ø20 renkli, mod düğmeleri, durum ışıkları, IMU, enkoderler, eksen kartları) ve "Ayarlar" (hız, ivme, sınır, mikroadım, oran, yönler, kazançlar; karta yaz, bilgisayara kaydet). Kart yeniden başlarsa arayüz bilgisayardaki ayarları kendisi geri yazar.

Parametreler çalışma anında değiştirilir, kartta kalıcı kayıt yoktur; yarışma ayar seti bilgisayarda tutulur.

## 7. Yazılımın blokları (özet)

| Blok | Hız | Görev |
|---|---|---|
| Adım üreteci (stepgen) | 20 kHz kesme | DDA ile STEP/DIR darbeleri, mikroadım konum sayacı |
| Eksen katmanı (axis) | 1 kHz kesme | yamuk hız profili, sınırlar, jog, salınım, kapalı çevrim (kapalı) |
| Enkoder sürücüsü (as5048a) | 1 kHz | SPI çerçeve, parite, sıçrama reddi, mıknatıs sağlığı, tur sayacı |
| IMU sürücüsü (bno085) | 100 Hz, ana döngü | kuaterniyon ve jiroskop, montaj dönüşümü, kesinti algılama |
| Nişan döngüsü (pointing) | 100 Hz, ana döngü | kilit, hata vektörü, PI + ileri besleme, ölü bant, koruma |
| Durum makinesi (fsm) | ana döngü | AÇILIŞ → MANUEL ↔ OTOMATİK, geçiş şartları |
| Haberleşme (comm) | ana döngü | çerçeve çözme, komutlar, telemetri, metin mesajları |
| Parametreler (params) | — | isim/varsayılan/değer tablosu |
| Donanım katmanı (hw) | — | pinler, zamanlayıcılar, SPI, I2C, seri, sürücü EN, lazer, LED |

## 8. Teensy 4.1'e geçiş: donanım katmanı

Yukarıdaki blokların sekizi Teensy'de değişmez; yalnız hw.cpp yeniden yazılır. Sözleşmesi aynı kalır (board_init, board_timers_start, board_step_write, board_dir_write, board_spi_xfer16, board_i2c_init, board_motor_enable, board_laser, LED). İçeriği:

- **Pin tablosu:** enkoder 6 SCK, 7 MISO, 8 CS pan, 9 CS tilt, 10 CS CE, 11 MOSI; BNO 19 SCL, 18 SDA; sürücüler 41/40/39 pan, 38/37/36 tilt, 35/34/33 CE (STEP/DIR/EN); lazer 5; LoRa Serial6 (24 TX6 → RXD, 25 RX6 ← TXD), 26 M0, 27 M1, 28 AUX; LED 13. TMC UART bağlanmaz.
- **Zamanlayıcılar:** iki IntervalTimer; 50 µs olan en yüksek öncelikte, 1 ms olan bir alt öncelikte; USB ve I2C daha altta. Adım darbeleri haberleşmeden etkilenmez.
- **Hızlı pin yazma:** STEP/DIR/CS için sabit pin numarasıyla digitalWriteFast (tek register yazımı).
- **Kenar yumuşatma:** SCK, MOSI ve CS pinlerinin pad ayarı düşük hız ve düşük sürüş gücüne çekilir; Nucleo'da bu yapılmadan uzun kablolarda enkoder çerçeveleri bozuluyordu, Teensy'nin kenarları daha da keskindir.
- **Yazılım SPI gecikmeleri:** nanosaniye gecikme fonksiyonuyla saat ~250 kHz'de tutulur.
- **I2C:** Wire, 18/19, 100 kHz.
- **USB seri:** Teensy'nin yerleşik USB'si, 12 Mbit; arayüzün port bulucusuna Teensy USB kimliği (16C0) eklenir.
- **LoRa:** Serial6, 9600 sonra 115200.
- **Sürücü EN:** açılışta üçü de yüksek (motorlar serbest), komutla düşürülür.
- **Bekçi köpek ve reset sebebi:** ilk sürümde yok, sonra eklenir.

Derleme: platformio.ini'ye teensy41 ortamı eklenir; STM32'ye özgü çağrılar hw.cpp dışında olmadığı için başka dosyaya dokunulmaz. Kontrol sırası: çıplak Teensy'de telemetri, sonra enkoderler (AGC), IMU, motorlar tek tek, son olarak kilit ve otomatik testi.
