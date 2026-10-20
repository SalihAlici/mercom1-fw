# MERCOM-1: eksenler nasıl çalışır, kod nasıl kuruludur

Bu metin, MERCOM-1 hareketli uydu terminalinin (TEKNOFEST SoTM yarışması, takım MERGEN) üç hareket eksenini, yön kurallarını ve STM32/Teensy firmware'inin yapısını başka birine ya da bir yapay zekâya aktarmak için yazılmıştır.

## 1. Mekanik ve eksenler

Terminalin başlığında bir lazer var; amaç, taban eğilse bile lazeri 4 m uzaktaki hedefte tutmak. Üç adım motoru (NEMA17, TMC2209 sürücü, 8 mikroadım) ve her eksende bir AS5048A manyetik enkoder (14 bit) var. Başlıkta (anten braketinde, üç eksenle birlikte hareket eden parçada) bir BNO085 IMU var; montaj kalibrasyonuyla IMU ekseni başlık çerçevesine (bakış yönü, yukarı, sol) dönüştürülür, böylece IMU doğrudan lazerin nereye baktığını ölçer.

**Pan (sağ-sol, azimut).** Başlığın tamamını düşey eksen etrafında döndürür. Redüktör oranı 10:1 (5:1 planet redüktör + 2:1 ek kademe; motor 10 tur atınca anten 1 tur). Dişli boşluğu yaklaşık 0,7°. Yazılımda sınır yoktur: açı 0–360 olarak gösterilir, içeride ±180 tutulur ve hedefe her zaman kısa yoldan gidilir (350'den 10'a 20 derecelik yoldan). Pozitif pan komutu gösterilen açıyı 0'dan 360'a doğru artırır; başlık çerçevesinde bu, bakış yönünü sola (yukarıdan bakınca saat yönünün tersine) çevirir. Fiziksel yön `mot_dir_pan` (+1) ve otomatik moddaki `aim_sign_pan` (+1) parametreleriyle belirlenir ve deneyle doğrulanmıştır.

**Tilt (yukarı-aşağı, yükseliş).** Başlığı yatay enine eksen etrafında döndürür. Oran 5:1, dişli boşluğu yaklaşık 0,2°. Sınır 0–90°: 0° lazer yatay, 90° tepe (zenit). Pozitif tilt lazeri yukarı kaldırır. Tilt sıfırı mutlak değildir; açılışta ya da "Tilt'i IMU'dan al" komutuyla IMU'nun ölçtüğü mekanik eğimden ayarlanır (başlığın "yukarı" vektörü ile yerçekimi arasındaki açıdan).

**CE (ince eksen).** Motor doğrudan, 1:1, dişli boşluğu yok. Pan ile aynı yönde (azimut) döner ama yalnız 0–4° aralığında; anten braketi bu eksenin üstündedir. Amacı, pan'ın 0,7°'lik boşluğunun yapamadığı ince yanal düzeltmeyi boşluksuz yapmaktır ("kaba-ince paylaşım"). Bu paylaşım henüz yazılmadı: otomatik modda CE sürücüsü açılır ve konumunu (2°, orta) tutar, düzeltmeyi pan ve tilt yapar. Manuel modda CE 0–4 arasında herhangi bir açıya gönderilebilir.

**Açı-adım dönüşümü.** Kart her ekseni anten derecesi cinsinden komutlar; mikroadım = derece / dps, dps = 1,8 / (mikroadım × oran): pan 0,0225°, tilt 0,045°, CE 0,225° bir mikroadım. Enkoder açısı da aynı birime çevrilir: enkoder ham değeri (0–16383) × 360/16384, tur sayacıyla çok turlu yapılır, `enc_ratio` ile bölünür (pan 10, tilt 5, CE 1) ve `enc_dir` ile işaretlenir (pan +1, tilt +1, CE −1).

## 2. Hareket nasıl üretilir

- **Adım üreteci (20 kHz kesme):** her eksen için bir DDA birikim sayacı; her 50 µs'de sayaca hız eklenir, taşma olunca STEP darbesi çıkar, DIR hızın işaretiyle ayarlanır. Böylece herhangi bir GPIO pini motor sürebilir; timer kanalı gerekmez.
- **Eksen katmanı (1 kHz kesme):** yamuk hız profili. Parametreler: en yüksek hız (pan 50, tilt 40, CE 20 °/s), ivme (150, 120, 100 °/s²), kalkış hızı 12 adım/s. Komutlar: açıya git (GOTO), hız modu (JOG, basılı tutunca döner), A↔B salınım (test), "burası = X" (adım sayacı ve enkoder sıfırı birlikte ayarlanır), dur (rampalı). Sınırlar burada uygulanır: tilt ve CE sınıra kırpılır, pan serbest.
- **Sürücüler:** her sürücünün EN pini ayrı ve aktif düşüktür. Açılışta üçü de pasiftir (motorlar serbest); arayüzden ya da otomatik moda geçince açılır. STEP/DIR/EN dışında UART kullanılmaz; mikroadım MS pinleriyle sabit 8'dir.

## 3. Otomatik mod: nereye, neye göre döner

1. Operatör manuel modda lazeri hedefe getirir ve "kilitle" der. Kart, IMU'dan hesapladığı başlık bakış vektörünü (dünya çerçevesinde, yerçekimi + yaw'a göre) saklar.
2. Otomatikte her IMU örneğinde (100 Hz) şu anki bakış vektörü ile saklanan vektörün vektörel çarpımından açı hatası bulunur, başlık çerçevesine çevrilir ve ikiye ayrılır: **yanal hata → pan**, **dikey hata → tilt**. Enkoderler bu döngüde kullanılmaz; hata doğrudan "lazer hedeften kaç derece sapmış" bilgisidir.
3. Her bileşen için hız komutu = Kp × hata + Ki × ∫hata + ileri besleme; Kp 8 1/s, Ki 2 1/s², integral sınırı 2°, çıkış tavanı 45 °/s. İleri besleme jiroskoptan gelen taban dönüş hızıdır (tilt'te 0,7 katsayı, pan'da 0 çünkü boşlukta salınım yapıyordu). D terimi yoktur; öngörüyü jiroskop sağlar. 0,3°'nin altındaki hata yok sayılır (ölü bant), pan komutu cos(yükseliş) ile bölünür.
4. Bu hızlar eksen katmanına hız modu olarak verilir; ivme sınırı orada uygulanır.
5. Koruma: IMU verisi 60 ms kesilirse ya da başlık zenit ötesine geçmişse motorlar durur, veri gelince devam eder. Otomatik mod hiçbir hatada kendi kendine vazgeçmez; hakem anteni elle 90° çevirse de geri gelir. Ölçülen toparlanma yaklaşık 0,5 s (istek 8 s), duruşta hata 0,02–0,15°.

## 4. Kodun yapısı (STM32 ve Teensy ortak)

```
src/main.cpp            kurulum, ana döngü, komut işleme, telemetri, IMU sorgulama, enkoder açı hesabı
src/hw.h                pin planı (Nucleo ve Teensy blokları) ve kart katmanı sözleşmesi
src/hw.cpp              Nucleo-H753ZI kart katmanı (STM32duino, HAL/LL)
src/hw_teensy.cpp       Teensy 4.1 kart katmanı (IntervalTimer, digitalWriteFast)
src/drv/stepgen.c       DDA adım üreteci (20 kHz)
src/drv/as5048a.c       enkoder çerçevesi, parite, tanı
src/drv/bno085.cpp      IMU: oyun dönüş vektörü + jiroskop, kesinti algılama
src/drv/tmc2209.c       sürücü UART (kullanılmıyor)
src/ctrl/axis.c         eksen katmanı: profil, sınırlar, jog, salınım, kapalı çevrim
src/ctrl/pointing.c     kilit, hata vektörü, PI + ileri besleme
src/app/fsm.c           AÇILIŞ → MANUEL ↔ OTOMATİK
src/app/comm.c, proto.h çerçeveli ikili protokol, komut kimlikleri
src/app/params.c        parametre tablosu (isim, varsayılan, değer)
src/app/telemetry.h     50 Hz'de 96 baytlık telemetri paketi
tools/mercom_gui.py     iki sekmeli arayüz;  tools/mercom_link.py  protokol kütüphanesi
```

Kart katmanı sözleşmesi (hw.h): `board_init`, `board_timers_start(step_cb, ctrl_cb)`, `board_step_write`, `board_dir_write`, `board_spi_xfer16`, `board_i2c_init`, `board_motor_enable`, `board_laser`, LED, bekçi köpek, reset sebebi. Algoritma kodu yalnız bu fonksiyonları çağırır; STM32'den Teensy'ye geçişte sadece bu katman yeniden yazıldı.

## 5. Kurallar ve kararlar

- Pan sınırsız, kısa yoldan; tilt 0–90; CE 0–4. Bunlar parametredir, arayüzden değişir.
- Motorlar açılışta serbest; her sürücü ayrı açılır/kapanır. Acil dur = üç sürücü kapanır.
- Arıza durumu yoktur: enkoder okunamaması, adım kaçırma, döngü aşımı yalnız metin mesajı verir, komutlar kilitlenmez.
- Otomatik mod kendi kendine manuel'e düşmez; yalnız IMU yokken ya da poz bozukken motorları tutar.
- Enkodere 1 s kesintisiz sağlıklı okuma olmadan güvenilmez; bir milisaniyede 2,6°'den fazla sıçrayan okuma atılır; mıknatıs sinyali (AGC) 60–200 iyidir, 255 uzak demektir.
- Kapalı çevrim (enkoderden adım düzeltme) varsayılan kapalıdır; nişan IMU ile çalışır. Duruşta enkoderden yeniden hizalama yalnız manuel modda yapılır.
- Enkoder SPI'sı yazılımla, ~250 kHz ve yumuşak kenarlı pinlerle sürülür; uzun dupont kablolarda keskin kenar çerçeveyi bozar.
- I2C 100 kHz; BNO085 verisi kesilirse yazılım sensörü yeniden başlatır.
- Parametreler kartta kalıcı değildir; yarışma ayar seti bilgisayarda (params.json) tutulur, kart yeniden başlarsa arayüz otomatik geri yazar.
- Yarışma sırası: manuel modda hedefe getir → kilitle → otomatik. Bozma (elle çevirme ya da başka açıya gönderme) sonrası kart kendi geri gelir.

## 6. Yarışma ayar seti (25 Eylül gecesi doğrulanan)

Kp 8, Ki 2, ölü bant 0,3°, otomatik hız tavanı 45 °/s, ileri besleme tilt 0,7 / pan 0, integral sınırı 2°, kapalı çevrim kapalı, duruş ölü bantları 0,5 / 0,6 / 2,5°, dişli boşluğu telafisi 0, hızlar 50/40/20, ivmeler 150/120/100, mikroadım 8, oranlar 10/5/1, enkoder yönleri +1/+1/−1, IMU montaj açıları −2,417 / −4,746 / 93,441°.

## 7. Fiziksel yapı: neler nerede

**Katmanlar, alttan üste.**

1. **Stewart platformu (yarışma düzeneği, bizim değil):** terminalin oturduğu taban; roll ve pitch eksenlerinde ±8° eğilir, salınım süresi ~10 s, öteleme yok.
2. **Terminal taban plakası:** platforma sabitlenir. Üstünde kontrol kartı (Nucleo-H753ZI, plan B Teensy 4.1), üç TMC2209 sürücü, 24 V güç kaynağı ve 5 V regülatör, LoRa modülü. IMU burada değil, başlıktadır (aşağıda).
3. **Pan kademesi:** tabandan yukarı çıkan düşey mil. Pan motoru (NEMA17) 5:1 planet redüktörle ve ardından 2:1 ek bir kademeyle (toplam 10:1, ölçülen 10,05) bu mili döndürür. Redüktörün işi: motorun torkunu 10 kat artırıp başlığın ağırlığını ve ataletini rahat çevirmek, çözünürlüğü 10 kat inceltmek (bir mikroadım 0,0225°). Bedeli dişli boşluğu: ~0,7°, yani yön değişince motor 0,7° dönmeden başlık kımıldamaz. Bu boşluk otomatik moddaki en büyük mekanik sorundur; pan'da ileri besleme bu yüzden kapalı, CE ekseni bu yüzden var.
4. **Tilt kademesi:** pan milinin tepesindeki çatal (yoke) üstünde yatay enine eksen. Tilt motoru 5:1 planet redüktörle (ölçülen 4,93) başlığı bu eksen etrafında yukarı-aşağı çevirir; boşluk ~0,2°. Başlığın dengesi tam değildir: tilt sürücüsü kapalıyken başlık kendi ağırlığıyla düşebilir, o yüzden tilt sürücüsü çalışma boyunca açık kalır.
5. **CE kademesi (başlığın üstünde):** tilt ile dönen başlığa bağlı, redüktörsüz, doğrudan mil. CE motorunun miline anten braketi takılıdır; lazer bu brakettedir. CE, pan ile aynı yönde (azimut) ama yalnız 0–4° döner. Redüktörü yok, boşluğu yok: ince yanal düzeltme için tasarlandı. Elle oynatılırsa lazer yanal kayar; otomatik mod bunu pan ile geri alır.
6. **Lazer ve IMU:** lazer 520 nm, 5 mW, anten braketinde, bakış ekseninde; MOSFET modülüyle 5 V'tan sürülür, kart A1 (Nucleo) / pin 5 (Teensy) ile açar-kapar. IMU (BNO085) da bu brakettedir: pan, tilt ve CE ile birlikte döner, yani doğrudan lazerin yönünü ölçer. Montaj kalibrasyonu (roll −2,4°, pitch −4,7°, yaw 93,4°) IMU'nun brakete ~93° döndürülmüş takılı olduğunu söyler; yazılım bu dönüşümle IMU eksenini bakış eksenine çevirir. IMU'nun braketteki tutuşu ve kablosu kritiktir: kablo iki kez koptu, lehim bir kez açıldı; akış kesilirse otomatik mod motorları tutar.

**Enkoderler nerede.** Her AS5048A modülü kendi motorunun arka mil ucundaki mıknatısı okur, yani motor tarafındadır, redüktörden önce: pan enkoderi anten 1 tur döndüğünde 10 tur sayar (oran 10), tilt 5, CE 1. Sonuç: enkoderler motorun ne yaptığını görür, redüktör boşluğunu görmez; bu yüzden nişan enkoderle değil IMU ile yapılır, enkoderler sınır ve adım kaçırma kontrolü içindir. Mıknatıs mil ucunun tam üstünde, 1–2 mm boşlukta durmalı; sinyal göstergesi (AGC) 60–200 arasında olmalı, 255 mıknatısın uzak olduğunu gösterir.

**Mesafeler.** Hedef levha terminalden 4 m uzakta; 4 m'de 1° sapma 7 cm'dir, Ø10 daire ±0,72°, Ø20 daire ±1,43° demektir. Terminalin iki kol ölçüsü paralaks telafisi için önemlidir: terminal alt yüzeyinden tilt eksenine yükseklik ve tilt ekseninden lazer ucuna mesafe. Kesin ölçüler henüz alınmadı; yazılımda varsayılan 30 cm ve 10 cm duruyor (taban eğilince lazer bu kollar kadar yer değiştirir; 8° eğimde 30 cm kol ~4 cm kayma yapar, hedefte bunun etkisi hesaba katılmalı). Motor kabloları henüz geçicidir; bu yüzden masada pan 360° çevrilmez, ±90° ile sınırlı denenir.

**Güç.** 24 V kaynak yalnız sürücülerin VM girişine gider (motor gücü). 3,3 V'u kart üretir: üç enkoder, IMU ve sürücü lojiği (VIO) buradan beslenir. LoRa ve lazer 5 V regülatörden beslenir, kartın 5 V pininden değil. Bütün topraklar tek noktada birleşir; ayrı toprak, tel kopuğu ve 24 V'un kart pinlerine değmesi bu projede bir işlemciye mal oldu.
