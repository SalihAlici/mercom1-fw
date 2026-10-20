#!/usr/bin/env python3
"""MERCOM-1 motor terminali, renkli pencere.

Kullanım:  python terminal_gui.py [COM8]
Karttaki pan_enc_term ya da motor_term koduyla konuşur. Komutlar aynıdır; alttaki satıra yaz, Enter ile gönder.
Esc = dur, Shift+Esc = acil, yukarı/aşağı ok = komut geçmişi.
"""
import math
import queue
import re
import sys
import threading
import time
import tkinter as tk

import serial

ARGS = [a for a in sys.argv[1:] if not a.startswith("--")]
PORT = ARGS[0] if ARGS else "COM8"
BAUD = 115200
SELFTEST = "--selftest" in sys.argv
SHOT = next((a.split("=", 1)[1] for a in sys.argv if a.startswith("--shot=")), None)

# ---- renkler ----
BG, PANEL, PANEL2, BORDER = "#0e1319", "#151c25", "#1b2430", "#263140"
TEXT, DIM = "#d9e1ea", "#6f7d8d"
ACCENT, GREEN, AMBER, RED, PURPLE = "#4cc2ff", "#5fd38d", "#f2b84b", "#ff6b6b", "#b693ff"
MONO = ("Cascadia Mono", 11)
MONO_BIG = ("Cascadia Mono", 17, "bold")
UI = ("Segoe UI", 10)
UI_SMALL = ("Segoe UI", 8)

# satır renklendirme: ilk eşleşen kural kazanır
RULES = [
    (re.compile(r"^(dondurme|enkoder|ayar|komutlar)\s*:|^\s+en 1\|0"), "dim"),
    (re.compile(r"KACIRMA|IPTAL|ACIL|GECERSIZ|BASARISIZ|CEVAP YOK|KAYBI|acilamadi|bilinmeyen komut|once dur|serbest"), "red"),
    (re.compile(r"^uyari|UYARI|^limit:"), "amber"),
    (re.compile(r"^vardi|SAGLIKLI|TAMAM|ACIK|surucu acik|sifirlandi|alindi"), "green"),
    (re.compile(r"^kalibre|^enkoder \w+:|^\s+->"), "purple"),
]

RE_IZLE = re.compile(r"^komut=(-?[\d.]+)\s+enkoder=(-?[\d.]+)\s+fark=(-?[\d.]+)\s+agc=(\d+)")
RE_D1 = re.compile(r"^eksen=(\w+)\s+(?:komut|konum)=(-?[\d.]+)\s+hedef=(-?[\d.]+)(?:\s+enkoder=(-?[\d.]+)\s+fark=(-?[\d.]+))?")
RE_D2 = re.compile(r"^hiz=([\d.]+) d/s\s+ivme=([\d.]+) d/s2\s+oran=([\d.]+)\s+mikro=(\d+)\s+limit=(-?\d+)\.\.(-?\d+)\s+surucu=(\w+)\s+(HAREKETTE|duruyor)")
RE_D3 = re.compile(r"^enkoder=(\w+) \(AGC (\d+)\)\s+kalibre=(\w+)\s+kapali cevrim=(\w+)\s+olubant=([\d.]+)\s+kacirma=(\d+)\s+duzeltme=(\d+)")
# üç eksenli kart yazılımı durum satırının sonuna bunu ekler; varsa "uc eksen" düğmeleri açılır
RE_UC = re.compile(r"\|\s*pan=(-?[\d.]+) tilt=(-?[\d.]+) ce=(-?[\d.]+)\s+hareket=(\S+)\s+salinim=(\S+)")


class App:
    def __init__(self, root):
        self.root = root
        self.ser = None
        self.q = queue.Queue()
        self.alive = True
        self.history, self.hidx = [], 0
        self.show_status_until = 0.0
        self.has_encoder = False
        self.izle_on = False
        self.st = dict(eksen="pan", komut=0.0, hedef=0.0, enc=None, fark=None, agc=None, hiz=None, ivme=None,
                       oran=None, mikro=None, surucu=None, hareket=False, enc_ok=None, kalibre=None, kapali=None,
                       kacirma=0, duzeltme=0, uc=None, uc_hareket="yok", uc_salin="yok")
        self._build()
        self.log("MERCOM-1 motor terminali. Komut yaz, Enter ile gonder.  Esc = dur   Shift+Esc = acil   yardim = komut listesi", "dim")
        if not SELFTEST:
            self.connect()
            self.root.after(2000, self.poll)
        self.root.after(30, self.pump)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    # ---------------- arayüz ----------------
    def _build(self):
        r = self.root
        r.title("MERCOM-1 Motor Terminali")
        r.configure(bg=BG)
        sw, sh = r.winfo_screenwidth(), r.winfo_screenheight()
        w, h = min(1080, sw - 60), min(700, sh - 110)          # küçük ekranda da görev çubuğuna taşmasın
        r.geometry(f"{w}x{h}+{(sw - w) // 2}+{max(10, (sh - h) // 2 - 30)}")
        r.minsize(820, 520)

        top = tk.Frame(r, bg=BG); top.pack(fill="x", padx=14, pady=(12, 6))
        tk.Label(top, text="MERCOM-1", bg=BG, fg=ACCENT, font=("Segoe UI", 16, "bold")).pack(side="left")
        tk.Label(top, text="  motor terminali", bg=BG, fg=TEXT, font=("Segoe UI", 16)).pack(side="left")
        self.lbl_conn = tk.Label(top, text="baglaniyor...", bg=BG, fg=AMBER, font=UI)
        self.lbl_conn.pack(side="right")

        tk.Label(r, text="git <d> · konum <d> · hiz <d/s> · ivme <d/s2> · salin <d> · dur · acil · sifir · en 1|0 · eksen pan|tilt|ce · durum · yardim\n"
                         "uc eksen birden: hepsi <d> · hepsikonum <d> · hepsisalin          enkoder: enc · kalibre · kapali 1|0 · olubant <d>",
                 bg=BG, fg=DIM, font=UI_SMALL).pack(side="bottom", fill="x", padx=14, pady=(6, 10))
        body = tk.Frame(r, bg=BG); body.pack(fill="both", expand=True, padx=14, pady=4)
        left = tk.Frame(body, bg=BG); left.pack(side="left", fill="y")
        right = tk.Frame(body, bg=BG); right.pack(side="left", fill="both", expand=True, padx=(12, 0))

        # --- sol: kadran + kartlar ---
        dial_outer, dial_box = self._panel(left, "ANTEN ACISI"); dial_outer.pack(fill="x")
        self.cv = tk.Canvas(dial_box, width=230, height=206, bg=PANEL, highlightthickness=0)
        self.cv.pack(padx=10, pady=(0, 2))
        axrow = tk.Frame(dial_box, bg=PANEL); axrow.pack(pady=(0, 8))
        self.b_axis = {}
        for name in ("pan", "tilt", "ce"):
            b = self._btn(axrow, name.upper(), lambda n=name: self.send("eksen " + n), ACCENT, width=6)
            b.pack(side="left", padx=3)
            self.b_axis[name] = b

        cards = tk.Frame(left, bg=BG); cards.pack(fill="x", pady=(8, 0))
        self.c_komut = self._card(cards, "KOMUT", ACCENT, 0, 0)
        self.c_enc = self._card(cards, "ENKODER", GREEN, 0, 1)
        self.c_fark = self._card(cards, "FARK", TEXT, 1, 0)
        self.c_agc = self._card(cards, "AGC", TEXT, 1, 1)

        info_outer, info = self._panel(left, "DURUM"); info_outer.pack(fill="x", pady=(8, 0))
        self.lbl_info = tk.Label(info, text="", bg=PANEL, fg=TEXT, font=("Cascadia Mono", 9), justify="left", anchor="w")
        self.lbl_info.pack(fill="x", padx=10, pady=(0, 10))

        # --- sağ: düğmeler + günlük + giriş ---
        bar = tk.Frame(right, bg=BG); bar.pack(fill="x")
        for txt, cmd in (("-90", "git -90"), ("-10", "git -10"), ("-1", "git -1"), ("+1", "git 1"), ("+10", "git 10"), ("+90", "git 90")):
            self._btn(bar, txt, lambda c=cmd: self.send(c), ACCENT, width=5).pack(side="left", padx=(0, 4))
        self._btn(bar, "Konum 0", lambda: self.send("konum 0"), TEXT).pack(side="left", padx=(8, 4))
        self._btn(bar, "Salin 50", lambda: self.send("salin 50"), TEXT).pack(side="left", padx=4)
        self._btn(bar, "ACIL", lambda: self.send("acil"), RED, fill=True).pack(side="right")
        self._btn(bar, "DUR", lambda: self.send("dur"), AMBER, fill=True).pack(side="right", padx=6)

        # üç eksen birden: kart destekliyorsa (durum satırında "| pan=.." varsa) handle() içinde gösterilir
        self.bar = bar
        self.bar3 = tk.Frame(right, bg=BG)
        tk.Label(self.bar3, text="UC EKSEN", bg=BG, fg=DIM, font=UI_SMALL).pack(side="left", padx=(0, 8))
        for txt, cmd in (("-90", "hepsi -90"), ("-10", "hepsi -10"), ("+10", "hepsi 10"), ("+90", "hepsi 90")):
            self._btn(self.bar3, txt, lambda c=cmd: self.send(c), PURPLE, width=5).pack(side="left", padx=(0, 4))
        self._btn(self.bar3, "Hepsi 0", lambda: self.send("hepsikonum 0"), TEXT).pack(side="left", padx=(8, 4))
        self._btn(self.bar3, "Limit salinimi", lambda: self.send("hepsisalin"), TEXT).pack(side="left", padx=4)
        self.lbl_uc = tk.Label(self.bar3, text="", bg=BG, fg=TEXT, font=("Cascadia Mono", 10))
        self.lbl_uc.pack(side="right")

        bar2 = tk.Frame(right, bg=BG); bar2.pack(fill="x", pady=(8, 8))
        tk.Label(bar2, text="Hiz", bg=BG, fg=DIM, font=UI).pack(side="left")
        self.e_hiz = self._entry(bar2, 6); self.e_hiz.pack(side="left", padx=(4, 10))
        tk.Label(bar2, text="Ivme", bg=BG, fg=DIM, font=UI).pack(side="left")
        self.e_ivme = self._entry(bar2, 6); self.e_ivme.pack(side="left", padx=(4, 6))
        self._btn(bar2, "Uygula", self.apply_profile, TEXT).pack(side="left", padx=(0, 12))
        for e in (self.e_hiz, self.e_ivme):
            e.bind("<Return>", lambda ev: self.apply_profile())
        self._btn(bar2, "Sifirla", lambda: self.send("sifir"), TEXT).pack(side="left", padx=2)
        self._btn(bar2, "Enc", lambda: self.send("enc"), PURPLE).pack(side="left", padx=2)
        self._btn(bar2, "Kalibre", lambda: self.send("kalibre"), PURPLE).pack(side="left", padx=2)
        self.b_kapali = self._btn(bar2, "Kapali cevrim: -", self.toggle_kapali, GREEN); self.b_kapali.pack(side="right")
        self.b_en = self._btn(bar2, "Surucu: -", self.toggle_en, GREEN); self.b_en.pack(side="right", padx=6)

        inp = tk.Frame(right, bg=BORDER); inp.pack(side="bottom", fill="x", pady=(8, 0))
        inner = tk.Frame(inp, bg=PANEL2); inner.pack(fill="x", padx=1, pady=1)
        tk.Label(inner, text=" >", bg=PANEL2, fg=ACCENT, font=("Cascadia Mono", 13, "bold")).pack(side="left")
        self.ent = tk.Entry(inner, bg=PANEL2, fg=TEXT, insertbackground=ACCENT, font=("Cascadia Mono", 13), relief="flat",
                            highlightthickness=0, bd=0)
        self.ent.pack(side="left", fill="x", expand=True, ipady=8, padx=6)
        self.ent.bind("<Return>", self.on_enter)
        self.ent.bind("<Up>", lambda e: self.hist(-1))
        self.ent.bind("<Down>", lambda e: self.hist(+1))
        self.root.bind("<Escape>", lambda e: self.send("dur"))
        self.root.bind("<Shift-Escape>", lambda e: self.send("acil"))
        self.ent.focus_set()
        logf = tk.Frame(right, bg=BORDER); logf.pack(fill="both", expand=True)
        self.txt = tk.Text(logf, bg=PANEL, fg=TEXT, insertbackground=TEXT, font=MONO, relief="flat", wrap="word",
                           padx=12, pady=10, state="disabled", spacing1=2, highlightthickness=0, bd=0)
        sb = tk.Scrollbar(logf, command=self.txt.yview, bg=PANEL, troughcolor=PANEL, relief="flat", width=12)
        self.txt.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y", padx=(0, 1), pady=1)
        self.txt.pack(side="left", fill="both", expand=True, padx=(1, 0), pady=1)
        for tag, col in (("dim", DIM), ("red", RED), ("amber", AMBER), ("green", GREEN), ("purple", PURPLE), ("text", TEXT), ("time", "#47525f")):
            self.txt.tag_configure(tag, foreground=col)
        self.txt.tag_configure("cmd", foreground=ACCENT, font=("Cascadia Mono", 11, "bold"))


        self.draw_dial()
        self.refresh()

    def _panel(self, parent, title):
        outer = tk.Frame(parent, bg=BORDER)
        inner = tk.Frame(outer, bg=PANEL); inner.pack(fill="both", expand=True, padx=1, pady=1)
        tk.Label(inner, text=title, bg=PANEL, fg=DIM, font=UI_SMALL).pack(anchor="w", padx=10, pady=(8, 2))
        return outer, inner        # outer paketlenir (kenarlık), içerik inner içine konur

    def _card(self, parent, title, color, row, col):
        outer = tk.Frame(parent, bg=BORDER)
        outer.grid(row=row, column=col, sticky="nsew", padx=(0 if col == 0 else 8, 0), pady=(0 if row == 0 else 8, 0))
        parent.grid_columnconfigure(col, weight=1)
        inner = tk.Frame(outer, bg=PANEL); inner.pack(fill="both", expand=True, padx=1, pady=1)
        tk.Label(inner, text=title, bg=PANEL, fg=DIM, font=UI_SMALL).pack(anchor="w", padx=10, pady=(8, 0))
        lbl = tk.Label(inner, text="-", bg=PANEL, fg=color, font=MONO_BIG, width=8, anchor="w")
        lbl.pack(anchor="w", padx=10, pady=(0, 6))
        return lbl

    def _btn(self, parent, text, cmd, color, width=None, fill=False):
        b = tk.Button(parent, text=text, command=cmd, font=("Segoe UI", 10, "bold"), relief="flat", bd=0, cursor="hand2",
                      bg=(color if fill else PANEL2), fg=(BG if fill else color), activebackground=color, activeforeground=BG,
                      padx=12, pady=6, highlightthickness=0)
        if width:
            b.configure(width=width, padx=4)
        return b

    def _entry(self, parent, width):
        return tk.Entry(parent, width=width, bg=PANEL2, fg=TEXT, insertbackground=ACCENT, font=MONO, relief="flat",
                        justify="center", highlightthickness=1, highlightbackground=BORDER, highlightcolor=ACCENT)

    # ---------------- kadran ----------------
    def draw_dial(self):
        cv, cx, cy, R = self.cv, 115, 103, 80
        cv.delete("all")
        cv.create_oval(cx - R, cy - R, cx + R, cy + R, outline=BORDER, width=2)
        for a in range(0, 360, 10):
            big = a % 30 == 0
            r1 = R - (12 if big else 6)
            x1, y1 = cx + r1 * math.sin(math.radians(a)), cy - r1 * math.cos(math.radians(a))
            x2, y2 = cx + R * math.sin(math.radians(a)), cy - R * math.cos(math.radians(a))
            cv.create_line(x1, y1, x2, y2, fill=(DIM if big else BORDER), width=(2 if big else 1))
        for a, t in ((0, "0"), (90, "90"), (180, "180"), (270, "-90")):
            x, y = cx + (R + 14) * math.sin(math.radians(a)), cy - (R + 14) * math.cos(math.radians(a))
            cv.create_text(x, y, text=t, fill=DIM, font=UI_SMALL)

        def needle(angle, color, length, width):
            x, y = cx + length * math.sin(math.radians(angle)), cy - length * math.cos(math.radians(angle))
            cv.create_line(cx, cy, x, y, fill=color, width=width, capstyle="round")

        st = self.st
        if st["hedef"] is not None and abs(st["hedef"] - st["komut"]) > 0.05:
            needle(st["hedef"], AMBER, R - 22, 1)
        if st["enc"] is not None and st["enc_ok"]:
            needle(st["enc"], GREEN, R - 30, 2)
        needle(st["komut"], ACCENT, R - 16, 4)
        cv.create_oval(cx - 6, cy - 6, cx + 6, cy + 6, fill=ACCENT, outline="")
        cv.create_text(cx, cy + 36, text=f"{st['komut']:.1f}°", fill=TEXT, font=("Cascadia Mono", 14, "bold"))
        cv.create_text(cx, cy + 54, text=st["eksen"].upper() + ("  hareket" if st["hareket"] else ""), fill=(AMBER if st["hareket"] else DIM), font=UI_SMALL)

    def refresh(self):
        st = self.st
        self.c_komut.configure(text=f"{st['komut']:.2f}")
        enc_valid = st["enc"] is not None and st["enc_ok"] is not False
        self.c_enc.configure(text=(f"{st['enc']:.2f}" if enc_valid else "-"), fg=(GREEN if enc_valid else DIM))
        if st["fark"] is not None and enc_valid:
            f = st["fark"]
            self.c_fark.configure(text=f"{f:+.3f}", fg=(GREEN if abs(f) < 0.2 else AMBER if abs(f) < 1.5 else RED))
        else:
            self.c_fark.configure(text="-", fg=DIM)
        if st["agc"] is not None:
            a = st["agc"]
            self.c_agc.configure(text=str(a), fg=(GREEN if 40 <= a <= 220 else RED))
        else:
            self.c_agc.configure(text="-", fg=DIM)

        def v(x, fmt="{}"):
            return "-" if x is None else fmt.format(x)
        enc_txt = "-" if st["enc_ok"] is None else ("saglikli" if st["enc_ok"] else "GECERSIZ")
        uc_txt = "" if st["uc"] is None else "\npan {:.1f}   tilt {:.1f}   ce {:.1f}".format(*st["uc"])
        self.lbl_info.configure(text=(
            f"hiz    {v(st['hiz'])} d/s      ivme  {v(st['ivme'])} d/s2\n"
            f"oran   {v(st['oran'])}          mikro {v(st['mikro'])}\n"
            f"enkoder {enc_txt}    kalibre {v(st['kalibre'])}\n"
            f"kacirma {st['kacirma']}          duzeltme {st['duzeltme']}" + uc_txt))
        k = st["kapali"]
        self.b_kapali.configure(text="Kapali cevrim: " + ("-" if k is None else "ACIK" if k else "kapali"), fg=(GREEN if k else DIM))
        s = st["surucu"]
        self.b_en.configure(text="Surucu: " + ("-" if s is None else "acik" if s else "serbest"), fg=(GREEN if s else RED if s is False else DIM))
        if st["uc"] is not None:
            durum = ("salinim: " + st["uc_salin"]) if st["uc_salin"] != "yok" else ("hareket: " + st["uc_hareket"]) if st["uc_hareket"] != "yok" else "duruyor"
            self.lbl_uc.configure(text=durum, fg=(DIM if durum == "duruyor" else AMBER))
        for name, b in self.b_axis.items():               # seçili eksenin düğmesi dolu görünür
            on = (name == st["eksen"])
            b.configure(bg=(ACCENT if on else PANEL2), fg=(BG if on else ACCENT))
        self.draw_dial()

    # ---------------- günlük ----------------
    def log(self, line, tag=None):
        if tag is None:
            tag = "text"
            for rx, t in RULES:
                if rx.search(line):
                    tag = t
                    break
        self.txt.configure(state="normal")
        self.txt.insert("end", time.strftime("%H:%M:%S  "), "time")
        self.txt.insert("end", line + "\n", tag)
        if int(self.txt.index("end-1c").split(".")[0]) > 2000:
            self.txt.delete("1.0", "200.0")
        self.txt.configure(state="disabled")
        self.txt.see("end")

    # ---------------- seri port ----------------
    def connect(self):
        if not self.alive or self.ser:
            return
        try:
            self.ser = serial.Serial(PORT, BAUD, timeout=0.05)
        except Exception:
            self.ser = None
            self.lbl_conn.configure(text=f"{PORT} acilamadi: baska bir program kullaniyor ya da kart takili degil  (yeniden deneniyor)", fg=RED)
            self.root.after(2000, self.connect)
            return
        self.lbl_conn.configure(text=f"● {PORT} · {BAUD} · bagli", fg=GREEN)
        threading.Thread(target=self.reader, args=(self.ser,), daemon=True).start()
        self.izle_on = False
        self.root.after(300, lambda: self.send("durum", echo=False))

    def reader(self, ser):
        buf = b""
        try:
            while self.alive and self.ser is ser:
                data = ser.read(256)
                if not data:
                    continue
                buf += data
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    self.q.put(line.decode("ascii", "replace").rstrip("\r"))
        except Exception:
            pass
        self.q.put(None)

    def send(self, cmd, echo=True):
        cmd = cmd.strip()
        if not cmd:
            return
        if echo:
            self.log("> " + cmd, "cmd")
            if cmd.split()[0].lower() in ("durum", "?"):
                self.show_status_until = time.time() + 1.5
        if not self.ser:
            if echo:
                self.log("bagli degil, komut gonderilemedi", "red")
            return
        try:
            self.ser.write((cmd + "\r\n").encode("ascii", "ignore"))
        except Exception:
            self.q.put(None)

    def poll(self):
        if self.alive:
            if self.ser:
                self.send("durum", echo=False)
            self.root.after(2000, self.poll)

    def pump(self):
        try:
            while True:
                line = self.q.get_nowait()
                if line is None:
                    self.lost()
                else:
                    self.handle(line)
        except queue.Empty:
            pass
        if self.alive:
            self.root.after(30, self.pump)

    def lost(self):
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.lbl_conn.configure(text=f"{PORT} baglantisi koptu (yeniden deneniyor)", fg=RED)
        self.root.after(1500, self.connect)

    # ---------------- gelen satırlar ----------------
    def handle(self, line):
        if not line.strip():
            return
        st = self.st
        m = RE_IZLE.match(line)
        if m:
            st["komut"], st["enc"], st["fark"], st["agc"] = float(m[1]), float(m[2]), float(m[3]), int(m[4])
            self.refresh()
            return
        show = time.time() < self.show_status_until
        m = RE_D1.match(line)
        if m:
            st["eksen"], st["komut"], st["hedef"] = m[1], float(m[2]), float(m[3])
            if m[4] is not None:
                st["enc"], st["fark"] = float(m[4]), float(m[5])
            u = RE_UC.search(line)
            if u:
                if st["uc"] is None:
                    self.bar3.pack(fill="x", pady=(8, 0), after=self.bar)
                st["uc"], st["uc_hareket"], st["uc_salin"] = (float(u[1]), float(u[2]), float(u[3])), u[4], u[5]
            self.refresh()
            if show:
                self.log(line, "dim")
            return
        m = RE_D2.match(line)
        if m:
            st["hiz"], st["ivme"], st["oran"], st["mikro"] = m[1], m[2], m[3], m[4]
            st["surucu"], st["hareket"] = (m[7] == "acik"), (m[8] == "HAREKETTE")
            for e, val in ((self.e_hiz, m[1]), (self.e_ivme, m[2])):
                if self.root.focus_get() is not e and e.get() != val:
                    e.delete(0, "end"); e.insert(0, val)
            self.refresh()
            if show:
                self.log(line, "dim")
            return
        m = RE_D3.match(line)
        if m:
            st["enc_ok"], st["agc"], st["kalibre"] = (m[1] == "saglikli"), int(m[2]), m[3]
            st["kapali"], st["kacirma"], st["duzeltme"] = (m[4] == "ACIK"), int(m[6]), int(m[7])
            if not self.has_encoder:
                self.has_encoder = True
            if not self.izle_on:                      # canlı değerler için kartın izleme akışını aç
                self.izle_on = True
                self.send("izle 10", echo=False)
            self.refresh()
            if show:
                self.log(line, "dim")
            return
        if line.startswith("izleme "):
            return
        self.log(line)

    # ---------------- kullanıcı ----------------
    def on_enter(self, ev=None):
        cmd = self.ent.get().strip()
        self.ent.delete(0, "end")
        if not cmd:
            return
        if not self.history or self.history[-1] != cmd:
            self.history.append(cmd)
        self.hidx = len(self.history)
        if cmd.lower() in ("temizle", "cls", "clear"):
            self.txt.configure(state="normal"); self.txt.delete("1.0", "end"); self.txt.configure(state="disabled")
            return
        self.send(cmd)

    def hist(self, d):
        if not self.history:
            return "break"
        self.hidx = max(0, min(len(self.history), self.hidx + d))
        self.ent.delete(0, "end")
        if self.hidx < len(self.history):
            self.ent.insert(0, self.history[self.hidx])
        return "break"

    def apply_profile(self):
        h, i = self.e_hiz.get().strip().replace(",", "."), self.e_ivme.get().strip().replace(",", ".")
        if h:
            self.send("hiz " + h)
        if i:
            self.send("ivme " + i)
        self.ent.focus_set()

    def toggle_kapali(self):
        self.send("kapali 0" if self.st["kapali"] else "kapali 1")

    def toggle_en(self):
        self.send("en 0" if self.st["surucu"] else "en 1")

    def close(self):
        self.alive = False
        if self.ser:
            try:
                self.ser.write(b"izle 0\r\n"); self.ser.flush(); time.sleep(0.05); self.ser.close()
            except Exception:
                pass
        self.root.destroy()


def selftest(app, root):
    sample = [
        "eksen=pan  komut=42.50  hedef=90.00  enkoder=42.31  fark=0.190 derece   | pan=42.50 tilt=-12.00 ce=3.50  hareket=pan+tilt  salinim=yok",
        "hiz=80.0 d/s  ivme=300.0 d/s2  oran=5.00  mikro=8  limit=-180..180  surucu=acik  HAREKETTE",
        "enkoder=saglikli (AGC 157)  kalibre=evet  kapali cevrim=ACIK  olubant=0.15  kacirma=1  duzeltme=4",
    ]
    app.show_status_until = 0
    for s in sample:
        app.handle(s)
    app.log("> kalibre", "cmd")
    for s in ["kalibre: 3.02 derece ileri gidip geri geliyor...", "kalibre: motor beklenen 15.07 derece, enkoder olctu 15.12 derece",
              "kalibre: enkoder yonu AYNI, olcek 1.003", "kalibre TAMAM. 'kapali 1' ile kapali cevrimi acabilirsin."]:
        app.handle(s)
    app.log("> kapali 1", "cmd")
    app.handle("kapali cevrim ACIK: adim kacarsa enkoderden gorup duzeltir, durusta ince duzeltme yapar")
    app.log("> git 90", "cmd")
    app.handle("gidiyor: 90.00 derece")
    app.handle("KACIRMA: motor adim kacirdi, enkoderden duzeltildi (toplam 1)")
    app.handle("komut=42.50  enkoder=42.31  fark=0.190  agc=157  [kapali cevrim]")
    app.handle("vardi: komut=90.00  enkoder=89.97  fark=0.030 derece")
    app.log("> git 500", "cmd")
    app.handle("limit: hedef 180.0 dereceye kirpildi (limit <min> <max> ile degistir)")
    app.log("> enc", "cmd")
    app.handle("enkoder pan: ham=6317  motor acisi=138.80  AGC=255  OCF=0 COF=0 yakin=0 uzak=1  ok=97567 hata=4")
    app.handle("  -> GECERSIZ: miknatis cok uzak (AGC 255). Miknatis ile cip arasi 1-2 mm olmali")
    app.st["komut"] = 42.5
    app.refresh()

    def finish():
        if SHOT:
            try:
                from PIL import ImageGrab
                root.lift(); root.attributes("-topmost", True); root.update()
                full = ImageGrab.grab()
                k = full.size[0] / root.winfo_screenwidth()            # ekran ölçeklemesi (%125, %150)
                x, y = root.winfo_rootx(), root.winfo_rooty()          # yalnız bu pencere, ekranın geri kalanı değil
                full.crop((int(x * k), int(y * k), int((x + root.winfo_width()) * k), int((y + root.winfo_height()) * k))).save(SHOT)
                print("ekran goruntusu:", SHOT)
            except Exception as e:
                print("ekran goruntusu alinamadi:", e)
        print("SELFTEST OK")
        app.close()
    root.after(1200, finish)


def main():
    root = tk.Tk()
    app = App(root)
    if SELFTEST:
        root.lift(); root.attributes("-topmost", True)
        selftest(app, root)
    root.mainloop()


if __name__ == "__main__":
    main()
