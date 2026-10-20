# MERCOM-1 hareketli uydu terminali: sistemin tam açıklaması

Takım MERGEN · Terminal MERCOM-1 · TEKNOFEST Hareketli Uydu Terminali (SoTM) yarışması · 30 Eylül 2026
Bu metin, sistemi hiç görmemiş birine fiziksel yapıyı, eksen hareketlerini, denetim yöntemini ve yazılımı tek parça hâlinde anlatır.

## 1. Yarışma ve istekler

Terminal, hareket hâlindeki bir aracı taklit eden 6 eksenli Stewart platformunun üstüne konur. Platform tabanı roll ve pitch eksenlerinde yaklaşık ±8° eğer, bir salınım yaklaşık 10 saniyedir; öteleme yoktur. Terminalin başlığındaki lazer, 4 metre uzaktaki hedef levhaya 5 dakika boyunca nişan almalıdır. Puanlama levhadaki dairelere göredir: Ø10 cm tam puan, Ø20 cm kısmi puan. 4 m'de 1° sapma 7 cm'dir; Ø10 daire ±0,72°, Ø20 daire ±1,43° demektir.

Hakem iki şekilde bozma yapar: anteni elle başka yöne çevirir ya da arayüzden başka bir açıya gönderir. Terminal 8 saniye içinde hedefe geri dönmelidir. İki mod istenir: manuel (operatör arayüzden açı verir) ve otomatik (terminal sensörleriyle hedefte kalır). Takım kararıyla GPS ve uydu görevi kapsam dışı; yalnız manuel ve otomatik yönelim vardır.

## 2. Fiziksel yapı: neler nerede, ne dönünce ne oluyor

Terminal, üst üste binen dört kademeden oluşur. Alttan üste:

**Taban plakası.** Stewart platformuna sabitlenir. Üstünde kontrol kartı (NUCLEO-H753ZI; işlemcisi 27–28 Eylül'de hasar gördüğü için plan B olarak Teensy 4.1), üç TMC2209 sürücü modülü, 24 V güç kaynağı, 5 V regülatör ve LoRa E22 modülü vardır. Bu plaka platformla birlikte eğilir ama kendi kendine dönmez.

**Pan kademesi (sağ-sol dönüş).** Taban plakasının ortasından yukarı çıkan düşey bir mil vardır. Pan motoru (NEMA17) taban plakasına bağlıdır; 5:1 planet redüktör ve ardından 2:1 ek bir kademe (toplam 10:1, ölçülen 10,05) üstünden bu mili döndürür. **Pan dönünce** milin üstündeki her şey (çatal, tilt motoru, başlık, CE motoru, anten braketi, lazer, IMU) düşey eksen etrafında bir bütün olarak döner; lazer noktası hedef levhada yatay olarak sağa-sola kayar. Pozitif pan komutu gösterilen açıyı artırır ve başlığı sola (yukarıdan bakınca saat yönünün tersine) çevirir. Motor 10 tur atınca anten 1 tur döner. Redüktörün faydası: 10 kat tork ve 10 kat ince çözünürlük (bir mikroadım 0,0225°). Bedeli: dişli boşluğu ~0,7°; yön değişince motor 0,7° dönmeden başlık kımıldamaz. Bu, otomatik moddaki en büyük mekanik sorundur.

**Tilt kademesi (yukarı-aşağı).** Pan milinin tepesinde bir çatal vardır; tilt motoru bu çatala bağlıdır ve 5:1 planet redüktörle (ölçülen 4,93) başlığı yatay enine eksen etrafında çevirir. **Tilt dönünce** başlık (CE motoru, anten braketi, lazer, IMU ile birlikte) yukarı-aşağı yatar; lazer noktası hedef levhada dikey olarak yukarı-aşağı kayar. Taban plakası ve pan mili yerinde kalır. Pozitif tilt lazeri kaldırır; 0° lazer yatay, 90° tepe (zenit). Dişli boşluğu ~0,2°. Başlık tam dengeli değildir: tilt sürücüsü kapalıyken kendi ağırlığıyla düşebilir, bu yüzden çalışma boyunca açık tutulur.

**CE kademesi (ince eksen).** Tilt ile yatan başlığın üstünde, redüktörsüz, doğrudan bağlı bir NEMA17 motor daha vardır: CE motoru. Milinde anten braketi takılıdır; lazer ve IMU bu brakettedir. **CE dönünce** yalnız anten braketi (lazer ve IMU ile) döner; dönüş ekseni pan ile aynı yöndedir (başlığın "yukarı" ekseni), yani lazer noktası hedefte küçük bir miktar sağa-sola kayar. Aralık yalnız 0–4°'dir. Redüktörü ve boşluğu yoktur; pan'ın 0,7°'lik boşluğunun yapamadığı ince yanal düzeltme için tasarlanmıştır ("kaba-ince paylaşım"). Bu paylaşım henüz yazılmadı: otomatik modda CE sürücüsü açılır ve 2° orta konumunu tutar, düzeltmeyi pan ve tilt yapar. Elle oynatılırsa lazer yanal kayar, otomatik mod bunu pan ile geri alır.

**Anten braketi, lazer, IMU.** Braket CE milindedir; en üstteki, her üç eksenle birlikte hareket eden parçadır. Lazer (520 nm, 5 mW) braketin bakış ekseninde durur, MOSFET modülüyle 5 V'tan sürülür, kart A1 (Nucleo) / pin 5 (Teensy) ile açılır. IMU (BNO085) da bu brakettedir: pan, tilt ve CE ile birlikte döndüğü için doğrudan lazerin baktığı yönü ölçer. IMU brakete yaklaşık 93° döndürülmüş takılıdır; montaj kalibrasyonu (roll −2,4°, pitch −4,7°, yaw 93,4°) bu dönüşü düzeltir ve IMU eksenini "bakış yönü, yukarı, sol" eksenlerine çevirir. IMU'nun tutuşu ve kablosu kritiktir: kablo iki kez koptu, lehim bir kez açıldı; akış kesilirse otomatik mod motorları tutar.

**Enkoderler.** Her eksende bir AS5048A (14 bit, SPI). Modül, kendi motorunun arka mil ucundaki mıknatısı okur; yani motor tarafındadır, redüktörden önce: anten 1 tur dönünce pan enkoderi 10 tur, tilt 5 tur, CE 1 tur sayar. Enkoder motorun ne yaptığını görür, redüktör boşluğunu görmez; bu yüzden nişan enkoderle değil IMU ile yapılır, enkoderler açı sınırı ve adım kaçırma kontrolü içindir. Mıknatıs mil ucunun tam üstünde, 1–2 mm boşlukta durmalı; sinyal göstergesi (AGC) 60–200 iyidir, 255 mıknatısın uzak olduğunu gösterir.

**Mesafeler.** Hedef 4 m. Paralaks için iki kol ölçüsü gerekir: terminal alt yüzeyinden tilt eksenine yükseklik ve tilt ekseninden lazer ucuna mesafe; kesin ölçüler alınmadı, varsayılan 30 cm ve 10 cm (8° eğimde 30 cm kol lazeri ~4 cm kaydırır). Motor kabloları geçicidir; masada pan 360° çevrilmez, ±90° ile denenir.

**Güç.** 24 V yalnız sürücülerin VM girişine gider (motor gücü). 3,3 V'u kart üretir: üç enkoder, IMU ve sürücü lojiği (VIO). LoRa ve lazer 5 V regülatörden beslenir, kartın 5 V pininden değil. Bütün topraklar tek noktada birleşir; ayrı toprak, kopuk tel ve 24 V'un kart pinine değmesi bu projede bir işlemciye mal oldu.

## 3. Eksenlerin sayısal kuralları

| Eksen | Hareket | Redüktör | Boşluk | Sınır | Bir mikroadım | Enkoder oranı / yönü | Hız / ivme |
|---|---|---|---|---|---|---|---|
| Pan | başlığın tamamı, düşey eksen | 10:1 | ~0,7° | yok (0–360 gösterim, kısa yol) | 0,0225° | 10 / +1 | 50 °/s, 150 °/s² |
| Tilt | başlık, yatay enine eksen | 5:1 | ~0,2° | 0–90° | 0,045° | 5 / +1 | 40 °/s, 120 °/s² |
| CE | yalnız anten braketi, pan yönü | yok | yok | 0–4° | 0,225° | 1 / −1 | 20 °/s, 100 °/s² |

Mikroadım = derece / dps, dps = 1,8 / (mikroadım × oran); sürücüler 8 mikroadımda. Enkoder açısı: ham 0–16383 → derece, tur sayacıyla çok turlu, orana bölünür, yön işaretiyle çarpılır. Fiziksel yönler `mot_dir_*`, `enc_dir_*` ve otomatik moddaki `aim_sign_*` parametreleriyle belirlenmiş ve deneyle doğrulanmıştır.

## 4. Hareket nasıl üretilir

- **Adım üreteci (20 kHz kesme):** her eksen için bir DDA birikim sayacı; her 50 µs'de sayaca hız eklenir, taşma olunca STEP darbesi çıkar, DIR hızın işaretinden ayarlanır. Herhangi bir GPIO pini motor sürebilir, timer kanalı gerekmez; 0,3 °/s'den 200 °/s'ye kadar hız kesintisiz üretilir.
- **Eksen katmanı (1 kHz kesme):** yamuk hız profili (ivmeyle kalkış, sabit hız, ivmeyle duruş, kalkış hızı 12 adım/s). Komutlar: açıya git, hız modu (jog; basılı tutunca döner), A↔B salınım (test), "burası = X" (adım sayacı ve enkoder sıfırı birlikte), rampalı dur. Sınırlar burada uygulanır: tilt ve CE sınıra kırpılır, pan serbest ve kısa yoldan.
- **Sürücüler:** her sürücünün EN pini ayrı ve aktif düşük. Açılışta üçü de pasif (motorlar serbest); arayüzden ya da otomatik moda geçince açılır. STEP/DIR/EN dışında UART kullanılmaz; mikroadım MS pinleriyle 8'e sabitlenmiştir.

## 5. Otomatik mod: nereye, neye göre döner

1. Operatör manuel modda lazeri hedefe getirir ve "kilitle" der. Kart, IMU'dan hesapladığı başlık bakış vektörünü dünya çerçevesinde (yerçekimi ve yaw'a göre) saklar.
2. Otomatikte her IMU örneğinde (100 Hz) şu anki bakış vektörü ile saklanan vektörün vektörel çarpımından açı hatası bulunur, başlık çerçevesine çevrilir ve ikiye ayrılır: **yanal hata → pan**, **dikey hata → tilt**. Denetim enkoder açılarına değil, doğrudan "lazer hedeften kaç derece sapmış" bilgisine göredir.
3. Klasik PID değil, **ölü bantlı PI + ileri besleme:** hız komutu = Kp × hata + Ki × ∫hata + ileri besleme. Kp = 8 1/s (1° hata → 8 °/s), Ki = 2 1/s² (sürtünme ve boşluktan kalan kalıcı hatayı siler; integral sınırı 2°, doyumda durdurma), çıkış tavanı 45 °/s. D terimi yoktur: türevin sağlayacağı öngörü jiroskoptan gelir; tabanın dönüş hızından başlığın kendi hızı çıkarılır, kalan "taban hızı" komuta eklenir (tilt'te 0,7 katsayı; pan'da 0, çünkü 0,7° boşlukta salınım yaptı). 0,3°'nin altındaki hata yok sayılır (ölü bant; boşlukta titremeyi önler, hedefte 2 cm). Pan komutu cos(yükseliş) ile bölünür.
4. Hız komutları eksen katmanının hız moduna verilir; ivme sınırı orada uygulanır, motor adım kaçırmaz.
5. Koruma: IMU verisi 60 ms'den uzun kesilirse ya da başlık zenit ötesine geçmişse motorlar tutulur, veri gelince devam eder. Otomatik mod hiçbir hata büyüklüğünde kendi kendine manuel'e düşmez; hakem anteni elle 90° çevirse de geri gelir.
6. Ölçülen: sabit tabanda hata 0,02–0,15°, bozma sonrası toparlanma yaklaşık 0,5 s (istek 8 s).

## 6. Sensör işleme

- **Enkoderler:** yazılımsal SPI ile 1 kHz'de okunur (mod 1, 16 bit, ~250 kHz, yumuşak kenarlı pinler; keskin kenar uzun dupont kabloda çerçeveyi bozuyordu). Parite kontrolü, bir milisaniyede 2,6°'den fazla değişen okumanın reddi, mıknatıs sağlığı, tur sayacı, 1 s güven süresi.
- **IMU:** oyun dönüş vektörü (manyetometresiz, yaw kayması 5 dakikada 0,05–0,25°) 100 Hz ve kalibre jiroskop 100 Hz, I2C 100 kHz. Rapor akışı kesilirse sensör yazılımla yeniden başlatılır. Tilt sıfırı IMU'nun ölçtüğü mekanik eğimden ayarlanır ("Tilt'i IMU'dan al").

## 7. Yazılımın yapısı (STM32 ve Teensy ortak)

```
src/main.cpp            kurulum, ana döngü, komut işleme, telemetri, IMU sorgulama, enkoder açı hesabı
src/hw.h                pin planı (Nucleo ve Teensy blokları) ve kart katmanı sözleşmesi
src/hw.cpp              Nucleo-H753ZI kart katmanı;  src/hw_teensy.cpp  Teensy 4.1 kart katmanı
src/drv/stepgen.c       DDA adım üreteci (20 kHz)
src/drv/as5048a.c       enkoder çerçevesi, parite, tanı
src/drv/bno085.cpp      IMU: oyun dönüş vektörü + jiroskop, kesinti algılama
src/drv/tmc2209.c       sürücü UART (kullanılmıyor)
src/ctrl/axis.c         eksen katmanı: profil, sınırlar, jog, salınım, kapalı çevrim
src/ctrl/pointing.c     kilit, hata vektörü, PI + ileri besleme
src/app/fsm.c           AÇILIŞ → MANUEL ↔ OTOMATİK
src/app/comm.c, proto.h çerçeveli ikili protokol (başlangıç baytı, kimlik, uzunluk, veri, CRC)
src/app/params.c        parametre tablosu (isim, varsayılan, değer)
src/app/telemetry.h     50 Hz'de 96 baytlık telemetri paketi
tools/mercom_gui.py     iki sekmeli arayüz;  tools/mercom_link.py  protokol kütüphanesi
```

Kart katmanı sözleşmesi: `board_init`, `board_timers_start(step_cb, ctrl_cb)`, `board_step_write`, `board_dir_write`, `board_spi_xfer16`, `board_i2c_init`, `board_motor_enable`, `board_laser`, LED, bekçi köpek, reset sebebi, `board_reset`. Algoritma kodu yalnız bunları çağırır; STM32'den Teensy'ye geçişte sadece bu katman yeniden yazıldı. Teensy pinleri: enkoder SCK 6, MISO 7, CS pan 8, CS tilt 9, CS CE 10, MOSI 11; BNO SCL 19, SDA 18; pan STEP 41 / DIR 40 / EN 39; tilt 38/37/36; CE 35/34/33; lazer 5; LoRa Serial6 (24 → RXD, 25 ← TXD), M0 26, M1 27, AUX 28; LED 13.

Arayüz iki sekmedir: "Kontrol ve veri" (nişan hatası Ø10/Ø20 renkli, Manuel / Hedefe kilitle / Otomatik takip düğmeleri, durum ışıkları, IMU, enkoderler, eksen kartları) ve "Ayarlar" (hız, ivme, sınır, mikroadım, oran, yönler, kazançlar). Kart 50 Hz telemetri gönderir; parametreler çalışma anında değişir, kartta kalıcı kayıt yoktur, yarışma ayar seti bilgisayarda tutulur ve kart yeniden başlarsa arayüz otomatik geri yazar.

## 8. Kurallar ve kararlar

- Pan sınırsız, kısa yoldan; tilt 0–90; CE 0–4. Parametredir, arayüzden değişir.
- Motorlar açılışta serbest; her sürücü ayrı açılır/kapanır; acil dur üçünü kapatır. Tilt sürücüsü çalışırken açık kalır (başlık dengesiz).
- Arıza durumu yoktur: enkoder okunamaması, adım kaçırma, döngü aşımı yalnız metin mesajı verir, komutlar kilitlenmez.
- Otomatik mod kendi kendine manuel'e düşmez; yalnız IMU yokken ya da poz bozukken motorları tutar.
- Enkodere 1 s kesintisiz sağlıklı okuma olmadan güvenilmez; kapalı çevrim (enkoderden adım düzeltme) varsayılan kapalıdır, nişan IMU ile çalışır; duruşta enkoderden yeniden hizalama yalnız manuel modda.
- Enkoder SPI'sı yazılımla, ~250 kHz, yumuşak kenar; I2C 100 kHz.
- 24 V kabloları kart soketlerinden uzak, bütün topraklar tek nokta; sıra hep önce USB, sonra 3,3 V modülleri, en son 24 V.
- Yarışma sırası: manuel modda hedefe getir → kilitle → otomatik. Bozma sonrası kart kendi geri gelir.

## 9. Yarışma ayar seti

Kp 8, Ki 2, ölü bant 0,3°, otomatik hız tavanı 45 °/s, ileri besleme tilt 0,7 / pan 0, integral sınırı 2°, kapalı çevrim kapalı, duruş ölü bantları 0,5 / 0,6 / 2,5°, dişli boşluğu telafisi 0, hızlar 50/40/20 °/s, ivmeler 150/120/100 °/s², mikroadım 8, oranlar 10/5/1, enkoder yönleri +1/+1/−1, IMU montaj açıları −2,417 / −4,746 / 93,441°.
