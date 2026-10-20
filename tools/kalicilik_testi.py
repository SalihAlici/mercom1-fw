# Parametre kalıcılığı testi (30 Eyl 2026). USB ile terminale bağlanır:
#  1) açılış durumu, 2) yarışma değerleri doğru mu, 3) değer değiştir + kaydet + yeniden başlat -> değer kalıyor mu,
#  4) hareket hâlinde kayıt (döngü aşımı, enkoder düzeltmesi), 5) kaydı sil -> varsayılan, 6) yarışma değerlerini yeniden kaydet.
# Sonunda kartta yarışma değerleri kalıcı kayıtlı kalır. Kullanım: python kalicilik_testi.py
import json, os, re, struct, time
import mercom_link as ml

HERE = os.path.dirname(os.path.abspath(__file__))
src = open(os.path.join(HERE, "yarisma_oncesi_test.py"), encoding="utf-8").read()
BEK = eval(re.search(r"BEKLENEN = (\{.*?\})\n", src, re.S).group(1))
BEK.update(json.load(open(os.path.join(HERE, "params.json"))))
SONUC = []
def kayit(ad, ok, ay=""):
    SONUC.append((ad, ok, ay)); print("%s %-52s %s" % ("[ OK ]" if ok else "[HATA]", ad, ay), flush=True)

class Kart:
    def __init__(self):
        for _ in range(20):
            p = ml.terminal_port()
            if p: break
            time.sleep(0.5)
        self.texts, self.acks = [], []
        self.link = ml.Link(p); orig = self.link._handle
        def h(mid, pl):
            if mid == ml.MSG_TEXT: self.texts.append(pl.decode("ascii", "replace"))
            elif mid == ml.MSG_ACK: self.acks.append((pl[0], pl[1]))
            else: orig(mid, pl)
        self.link._handle = h
        t0 = time.time()
        while self.link.last_telem is None and time.time() - t0 < 8:
            self.link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 20)); time.sleep(0.4)
    def send(self, mid, p=b""): self.link.send(mid, p)
    def t(self): return dict(zip(ml.TELEM_COLS, self.link.last_telem))
    def ping(self, desen):
        n = len(self.texts); self.send(ml.MSG_PING); time.sleep(0.6)
        return next((s for s in self.texts[n:] if re.search(desen, s)), "")
    def save(self, p=b""):
        n = len(self.acks); m = len(self.texts); self.send(ml.MSG_PARAM_SAVE, p); time.sleep(1.2)
        a = [s for i, s in self.acks[n:] if i == ml.MSG_PARAM_SAVE]
        return (a[-1] if a else None), next((s for s in self.texts[m:] if "param kaydi" in s or "kalici" in s), "")
    def params(self, names):
        for nm in names: self.send(ml.MSG_PARAM_GET, struct.pack("<H", ml.PARAM_NAMES.index(nm))); time.sleep(0.05)
        time.sleep(0.5)
        return {nm: self.link.params.get(ml.PARAM_NAMES.index(nm)) for nm in names}
    def setp(self, nm, v): self.send(ml.MSG_PARAM_SET, struct.pack("<Hf", ml.PARAM_NAMES.index(nm), float(v))); time.sleep(0.1)
    def reset(self):
        self.send(ml.MSG_SYS_RESET); time.sleep(0.3)
        try: self.link.close()
        except Exception: pass
        time.sleep(6)
        return Kart()
    def close(self):
        try: self.link.close()
        except Exception: pass

def yanlis(vals):
    return ["%s=%s (beklenen %s)" % (n, vals[n], v) for n, v in BEK.items() if vals[n] is None or abs(vals[n] - v) > 1e-3 * max(1, abs(v))]

k = Kart()
kayit("Acilis durumu", True, k.ping(r"param kaydi"))
y = yanlis(k.params(list(BEK)))
kayit("Yarisma degerleri (%d adet)" % len(BEK), not y, "; ".join(y) or "hepsi dogru")
a, s = k.save()
kayit("Kaydet (yarisma degerleri)", a == 0, s)

k.setp("vmax_ce", 19.0); k.setp("aim_db", 0.25)
a, s = k.save()
kayit("Degistir (vmax_ce 19, aim_db 0.25) + kaydet", a == 0, s)
k = k.reset()
p = k.ping(r"param kaydi")
v = k.params(["vmax_ce", "aim_db", "telem_hz"])
kayit("Yeniden baslayinca degistirilen degerler duruyor", abs((v["vmax_ce"] or 0) - 19) < 1e-3 and abs((v["aim_db"] or 0) - 0.25) < 1e-4 and "yuklendi" in p,
      "vmax_ce %s, aim_db %s | %s" % (v["vmax_ce"], v["aim_db"], p))
k.setp("vmax_ce", 20.0); k.setp("aim_db", 0.3)

# hareket hâlinde kayıt: pan ve tilt dönerken
def asim():
    m = re.search(r"dongu asimi (\d+)", k.ping(r"dongu asimi")); return int(m.group(1)) if m else -1
def duz():
    n = len(k.texts); k.send(ml.MSG_AXIS_INFO); time.sleep(0.6)
    return sum(int(x) for x in re.findall(r"duzeltme=(\d+)", " ".join(k.texts[n:])))
k.send(ml.MSG_SET_MODE, bytes([1])); time.sleep(0.3); k.send(ml.MSG_ENABLE, bytes([1])); time.sleep(12)   # enkoder güveni
o0, d0 = asim(), duz()
k.send(ml.MSG_JOG, struct.pack("<Bf", 0, 20.0)); k.send(ml.MSG_JOG, struct.pack("<Bf", 1, 5.0)); time.sleep(1.0)
k.setp("vmax_ce", 20.5)                                   # kaydın gerçekten flaşa yazması için bir değer değişsin
a, s = k.save()
k.send(ml.MSG_JOG, struct.pack("<Bf", 0, -20.0)); k.send(ml.MSG_JOG, struct.pack("<Bf", 1, -5.0)); time.sleep(2.2)
k.send(ml.MSG_JOG, struct.pack("<Bf", 0, 0.0)); k.send(ml.MSG_JOG, struct.pack("<Bf", 1, 0.0)); time.sleep(1.5)
o1, d1 = asim(), duz()
kayit("Hareket halinde kayit", a == 0 and d1 - d0 == 0, "%s | dongu asimi +%d, sahte enkoder duzeltmesi +%d" % (s, o1 - o0, d1 - d0))
k.setp("vmax_ce", 20.0)
k.send(ml.MSG_ENABLE, bytes([0])); time.sleep(0.3)

# kaydı sil -> varsayılan
a, s = k.save(b"\x00")
k = k.reset()
p = k.ping(r"param kaydi")
kayit("Kaydi sil -> sonraki acilis varsayilan", a == 0 and "kayit yok" in p, p)

# yarışma değerlerini yeniden kalıcı kaydet ve doğrula
y = yanlis(k.params(list(BEK)))
a, s = k.save()
k = k.reset()
p = k.ping(r"param kaydi")
y2 = yanlis(k.params(list(BEK)))
kayit("Yarisma degerleri kalici kayitli", not y and a == 0 and not y2 and "yuklendi" in p, ("; ".join(y + y2) or "hepsi dogru") + " | " + p)
k.close()
print("\ngecen %d / %d" % (sum(1 for _, ok, _ in SONUC if ok), len(SONUC)))
