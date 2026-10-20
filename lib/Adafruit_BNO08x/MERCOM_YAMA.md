# Adafruit BNO08x 1.2.7, MERCOM-1 yaması

Bu klasör `adafruit/Adafruit BNO08x` 1.2.7 sürümünün kopyasıdır. PlatformIO proje içindeki `lib/` klasörünü
`.pio/libdeps` altındaki indirilen kopyadan önce kullanır.

Tek değişiklik `src/sh2.c` içinde: `getProdIdOp` işlemine 0,5 s zaman aşımı eklendi.

Sebep: özgün kütüphanede ürün kimliği sorgusunun zaman aşımı yok. Sensör cevap vermediğinde `begin_I2C()`
sonsuza dek bekliyor, ana döngü beslenmediği için 3 s bekçi köpek kartı sıfırlıyordu. 29 Eylül 2026'da iki kayıtta
görüldü (kart MANUAL moda düştü, lazer kapandı, motorlar boşa çıktı).

İkinci değişiklik `src/sh2.c` içinde `sh2_close()`: açık oturum yoksa (`pShtp == 0`) `shtp_close` çağrılmaz.
Sebep: zaman aşımı eklenince `begin_I2C()` oturumu açtıktan sonra false dönebiliyor. Kütüphanede tek SHTP örneği
var; oturum kapatılmazsa sonraki her başlatma "örnek yok" hatasıyla başarısız oluyor (29 Eyl 2026'da görüldü:
sensör adreste ACK veriyor, başlatılamıyor, yalnız Teensy yeniden başlayınca düzeliyor). `src/drv/bno085.cpp`
artık her başlatmadan önce `sh2_close()`'u koşulsuz çağırıyor; bu korumayla boş oturumda güvenli.

Kütüphane güncellenirse bu yamalar yeniden uygulanmalıdır.
