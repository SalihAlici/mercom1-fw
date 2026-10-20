#!/usr/bin/env python3
"""MERCOM-1 masaüstü arayüzü: python mercom_gui.py [COM8]

İki sekme:
  Kontrol ve veri : hedef takibi (kilit / otomatik), durum ışıkları, IMU, enkoderler, eksen başına git / adım / basılı-tut / salınım
  Ayarlar         : eksen başına hız, ivme, sınır, mikroadım, oran, yönler, kapalı çevrim, kazançlar; karta yaz / oku, PC'ye kaydet / yükle
Tkinter kullanır, ek kurulum gerekmez. Seri portu aynı anda başka bir program açmamalı.
"""
import json
import math
import os
import re
import struct
import sys
import time
import tkinter as tk
from tkinter import messagebox, ttk

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mercom_link as ml  # noqa: E402

AXES = ("pan", "tilt", "ce")
AXIS_TITLE = {"pan": "PAN", "tilt": "TILT", "ce": "CE"}
# LoRa yer istasyonu: E22 modülü USB-seri dönüştürücüyle bağlı (CH340/CH9102, CP210x, FTDI, PL2303). 9600 baud, E22 fabrika ayarı.
LORA_VIDS = (0x1A86, 0x10C4, 0x0403, 0x067B)
PARAMS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "params.json")

# ---- renkler: takım renkleri lacivert 24;30;54 (#181e36) zemin, turuncu 224;77;0 (#e04d00) düğme, açık turuncu 255;119;1 (#ff7701) başlık / üstüne gelince ----
BG, PANEL, PANEL2, BORDER = "#181e36", "#1f2745", "#2b3458", "#5262a8"   # BORDER: kutu çerçevesi (2 px, mavi-mor)
HEAD = "#262f56"                                                            # kutu başlık bandı
TEXT, DIM = "#eef0f7", "#9aa4c4"
ACCENT, ACCENT2 = "#e04d00", "#ff7701"
GREEN, AMBER, RED, PURPLE, TEAL, SLATE = "#22c55e", "#f5b400", "#dc2626", "#7c9cff", "#2dd4bf", "#4b5680"
WHITE_TEXT = {ACCENT, RED, SLATE, PANEL2}          # bu zeminlerde yazı beyaz, diğerlerinde lacivert
UI, UI_B, UI_S = ("Segoe UI", 10), ("Segoe UI", 10, "bold"), ("Segoe UI", 8)
MONO, MONO_XL = ("Consolas", 11), ("Consolas", 40, "bold")

# Ayarlar sekmesindeki satırlar: (etiket, parametre adı şablonu, açıklama). {a} eksen adı olur.
SETTING_ROWS = [
    ("En yüksek hız",     "vmax_{a}",      "\"Git\" ve basılı-tut hareketlerinin hız tavanı"),
    ("İvme",             "amax_{a}",      "kalkış ve duruş sertliği; motor adım kaçırıyorsa düşür"),
    ("Alt sınır",           "{lim}_min",     "pan 0..360 = sınırsız, kısa yoldan döner"),
    ("Üst sınır",           "{lim}_max",     "tilt -90..90, CE 0..4"),
    ("Mikroadım",               "usteps_{a}",    "sürücü kartındaki MS ayarıyla aynı: boş 8, MS1+MS2 16, yalnız MS2 64"),
    ("Redüktör oranı",          "enc_ratio_{a}", "motor turu / anten turu: pan 10, tilt 5, CE 1"),
    ("Motor yönü",    "mot_dir_{a}",   "artı komutta anten ters dönüyorsa -1 yap"),
    ("Enkoder yönü",  "enc_dir_{a}",   "enkoder motora ters sayıyorsa -1 yap"),
    ("Kapalı çevrim",   "cl_{a}",        "1: adım kaçırınca enkoderden düzeltir"),
    ("Kaçırma eşiği",       "sl_thr_{a}",    "harekette komut-enkoder farkı bunu aşarsa düzeltme yapılır"),
    ("Duruş ölü bandı",     "enc_db_{a}",    "duruşta bunun üstündeki fark düzeltilir"),
    ("Takip kazancı Kp",  "kp_{a}",        "otomatik modda hataya tepki; motor salınım yaparsa düşür"),
    ("Takip kazancı Ki", "ki_{a}",        "kalıcı küçük hatayı siler; 0 = kapalı"),
    ("Dişli boşluğu",       "bl_{a}",        "yön değişince motor bu kadar fazladan sürülür; şimdilik 0"),
]
AIM_ROWS = [                     # eksene bağlı olmayan nişan ayarları
    ("Takip yönü pan",   "aim_sign_pan",  "otomatik modda lazer hedeften kaçıyorsa çevir"),
    ("Takip yönü tilt",  "aim_sign_tilt", ""),
    ("Takip hız tavanı",     "aim_vmax",      "otomatik modda motorların en yüksek hızı"),
    ("Integral sınırı",        "aim_imax",      "Ki'nin biriktirebileceği en fazla hata"),
    ("İleri besleme tilt",   "aim_kff",       "taban eğilirken jiroskopla önceden karşılama; 0 kapalı"),
    ("İleri besleme pan",    "aim_kff_pan",   "yaw'da pan'ı jiroskopla önceden karşılar; 0 kapalı"),
    ("Takip ölü bandı",        "aim_db",        "bu hatanın altında motor oynamaz; duruşta titriyorsa artır"),
    ("Paralaks düzeltmesi", "plx_en",       "1: yön yerine hedef NOKTASI tutulur; tezgah testinden önce açma"),
    ("Tilt ekseni yüksekliği", "plx_ht",        "platformun dönme merkezinden tilt eksenine"),
    ("Lazer ileri",            "plx_lx",        "tilt ekseninden ışın yönünde; etkisi küçük"),
    ("Lazer sola",             "plx_ly",        "pan ekseninden; lazer SAĞDAYSA eksi"),
    ("Lazer yukarı",           "plx_lz",        "tilt ekseninin üstünde"),
    ("Hedef mesafesi",         "plx_d",         "kilit anında lazerden hedefe; 4 m ileri 2 m yukarı = 4,47"),
]


def btn(parent, text, color, command=None, width=None, dark_text=None, small=False):
    """Düz renkli köşeli düğme. Yazı rengi zemine göre kendiliğinden seçilir."""
    if dark_text is None:
        dark_text = color not in WHITE_TEXT
    fg = BG if dark_text else "white"
    b = tk.Button(parent, text=text, bg=color, fg=fg, activebackground=ACCENT2, activeforeground=BG,
                  font=("Segoe UI", 9, "bold") if small else UI_B, relief="flat", bd=0, cursor="hand2",
                  padx=8 if small else 12, pady=2 if small else 4, command=command)
    if width:
        b.config(width=width)
    return b


class App(tk.Tk):
    def __init__(self, port):
        super().__init__()
        self.title("MERCOM-1")
        self.geometry("1400x800")
        self.minsize(1200, 700)
        try:
            self.state("zoomed")
        except tk.TclError:
            pass
        self.configure(bg=BG)
        self.link = None
        self.port_var = tk.StringVar(value=port)
        self.enc_diag = {a: "" for a in AXES}
        self.last_t_ms = None
        self.last_count = (0, 0.0)
        self.imu_hz = 0.0
        self.telem = None
        self.osc_on = {a: False for a in AXES}
        self.setting_vars = {}
        self.en_vars = {}
        self._style()
        self._build()
        self.after(50, self._tick)
        self.after(1000, self._poll_diag)
        self.after(200, self._retry_tick)
        self.after(1000, self._watch_link)
        self.protocol("WM_DELETE_WINDOW", self._close)

    # ================= tema =================
    def _style(self):
        st = ttk.Style(self)
        st.theme_use("clam")
        st.configure(".", background=BG, foreground=TEXT, font=UI, bordercolor=BORDER, lightcolor=PANEL, darkcolor=PANEL)
        st.configure("TFrame", background=BG)
        st.configure("TLabel", background=BG, foreground=TEXT)
        st.configure("TEntry", fieldbackground=PANEL2, foreground=TEXT, insertcolor=ACCENT2, bordercolor=BORDER, lightcolor=BORDER, darkcolor=BORDER)
        st.configure("TCheckbutton", background=PANEL, foreground=TEXT, font=UI_B)
        st.map("TCheckbutton", background=[("active", PANEL)], indicatorcolor=[("selected", GREEN), ("!selected", PANEL2)],
               foreground=[("selected", GREEN)])
        st.configure("TCombobox", fieldbackground=PANEL2, background=PANEL2, foreground=TEXT, arrowcolor=ACCENT2, padding=(6, 3),
                     bordercolor=BORDER, lightcolor=BORDER, darkcolor=BORDER, selectbackground=PANEL2, selectforeground=TEXT)
        st.map("TCombobox", fieldbackground=[("readonly", PANEL2)], foreground=[("readonly", TEXT)], background=[("active", PANEL2)],
               selectbackground=[("readonly", PANEL2)], selectforeground=[("readonly", TEXT)])
        for k, v in (("background", PANEL2), ("foreground", TEXT), ("selectBackground", ACCENT), ("selectForeground", "white"), ("font", UI)):
            self.option_add("*TCombobox*Listbox." + k, v)
        st.configure("TNotebook", background=BG, borderwidth=0, lightcolor=BG, darkcolor=BG, bordercolor=BG)
        st.layout("TNotebook.Tab", [])                                   # ttk sekme başlıkları gizli; üstte kendi düğmelerimiz var
        st.configure("Head.TCheckbutton", background=HEAD, foreground=TEXT, font=UI_B)
        st.map("Head.TCheckbutton", background=[("active", HEAD)], indicatorcolor=[("selected", GREEN), ("!selected", PANEL2)],
               foreground=[("selected", GREEN)])

    def _panel(self, parent, title):
        outer = tk.Frame(parent, bg=BORDER)
        inner = tk.Frame(outer, bg=PANEL)
        inner.pack(fill="both", expand=True, padx=2, pady=2)
        head = tk.Frame(inner, bg=HEAD); head.pack(fill="x")
        tk.Label(head, text=title, bg=HEAD, fg=ACCENT2, font=("Segoe UI", 10, "bold"), padx=10, pady=3).pack(side="left")
        tk.Frame(inner, bg=PANEL, height=2).pack(fill="x")
        return outer, inner, head

    def _pill(self, parent, text, color):
        return tk.Label(parent, text=text, bg=color, fg=self._pill_fg(color), font=("Segoe UI", 9, "bold"), padx=10, pady=2)

    @staticmethod
    def _pill_fg(color):
        return TEXT if color in (BORDER, PANEL2) else ("white" if color in WHITE_TEXT else BG)

    def _lbl(self, parent, text, fg=DIM, font=UI, bg=PANEL, **kw):
        return tk.Label(parent, text=text, bg=bg, fg=fg, font=font, **kw)

    # ================= arayüz =================
    def _build(self):
        top = tk.Frame(self, bg=BG); top.pack(fill="x", padx=10, pady=(8, 4))
        try:                                     # MERGEN logosu (tools/mergen_logo.png); dosya yoksa yazı
            self._logo = tk.PhotoImage(file=os.path.join(os.path.dirname(os.path.abspath(__file__)), "mergen_logo.png"))
            tk.Label(top, image=self._logo, bg=BG).pack(side="left")
        except tk.TclError:
            tk.Label(top, text="MERGEN", bg=BG, fg=ACCENT2, font=("Segoe UI", 18, "bold")).pack(side="left")
        tk.Label(top, text="      Seri port", bg=BG, fg=DIM, font=UI).pack(side="left")
        self.cb_port = ttk.Combobox(top, textvariable=self.port_var, width=26, state="readonly", postcommand=self._refresh_ports)
        self.cb_port.pack(side="left", padx=4)
        self._refresh_ports(prefer=self.port_var.get())
        self.btn_conn = btn(top, "Bağlan", ACCENT, self._toggle_conn); self.btn_conn.pack(side="left")
        self.pill_conn = self._pill(top, "bağlı değil", BORDER); self.pill_conn.pack(side="left", padx=(14, 4))
        self.pill_mode = self._pill(top, "mod: —", BORDER); self.pill_mode.pack(side="left", padx=4)
        self.pill_lock = self._pill(top, "hedef kilidi yok", BORDER); self.pill_lock.pack(side="left", padx=4)
        b = btn(top, "ACİL DUR: motorları bırak", RED, self._estop); b.config(font=("Segoe UI", 11, "bold")); b.pack(side="right", padx=(6, 0))
        b = btn(top, "DUR: hareketi durdur", AMBER, self._stop_click); b.config(font=("Segoe UI", 11, "bold")); b.pack(side="right", padx=4)

        # alt: kart mesajları (önce paketlenir ki sekmeler onu ezmesin)
        bottom = tk.Frame(self, bg=BG); bottom.pack(side="bottom", fill="x", padx=10, pady=(0, 8))
        r = tk.Frame(bottom, bg=BG); r.pack(fill="x", pady=(0, 3))
        tk.Label(r, text="KART MESAJLARI", bg=BG, fg=ACCENT2, font=("Segoe UI", 9, "bold")).pack(side="left")
        btn(r, "Mesajları temizle", SLATE, lambda: self.txt.delete("1.0", "end"), small=True).pack(side="right")
        self.btn_log = btn(r, "Veri kaydını başlat", SLATE, self._toggle_log, small=True); self.btn_log.pack(side="right", padx=6)
        self.log_var = tk.StringVar(value="kayit.csv")
        ttk.Entry(r, textvariable=self.log_var, width=16).pack(side="right")
        tk.Label(r, text="dosya adı", bg=BG, fg=DIM, font=UI_S).pack(side="right", padx=4)
        lf = tk.Frame(bottom, bg=BORDER); lf.pack(fill="x")
        self.txt = tk.Text(lf, height=4, bg=PANEL, fg=TEXT, insertbackground=TEXT, font=("Consolas", 10), relief="flat", bd=0, padx=8, pady=6)
        self.txt.pack(fill="x", padx=2, pady=2)
        for tag, col in (("dim", DIM), ("red", RED), ("amber", AMBER), ("green", GREEN), ("time", "#5c6690")):
            self.txt.tag_configure(tag, foreground=col)

        tabbar = tk.Frame(self, bg=BG); tabbar.pack(fill="x", padx=10, pady=(2, 0))
        self.tab_btns = []
        for i, txt in enumerate(("Kontrol ve veri", "Ayarlar")):
            cell = tk.Frame(tabbar, bg=BG); cell.pack(side="left", padx=(0, 8))
            b = tk.Button(cell, text=txt, font=("Segoe UI", 12, "bold"), relief="flat", bd=0, cursor="hand2", padx=28, pady=7,
                          activebackground=ACCENT2, activeforeground="white", command=lambda i=i: self.nb.select(i))
            b.pack(fill="x")
            line = tk.Frame(cell, bg=BG, height=4); line.pack(fill="x")
            self.tab_btns.append((b, line))
        self.nb = ttk.Notebook(self); self.nb.pack(fill="both", expand=True, padx=10, pady=(0, 6))
        self.nb.bind("<<NotebookTabChanged>>", lambda ev: self._tab_changed())
        tab1 = tk.Frame(self.nb, bg=BG); self.nb.add(tab1, text="Kontrol ve veri")
        tab2 = tk.Frame(self.nb, bg=BG); self.nb.add(tab2, text="Ayarlar")
        self._build_control(tab1)
        self._build_settings(tab2)

    def _tab_changed(self):
        cur = self.nb.index(self.nb.select())
        for i, (b, line) in enumerate(self.tab_btns):
            if i == cur:
                b.config(bg=ACCENT, fg="white"); line.config(bg=ACCENT2)
            else:
                b.config(bg=PANEL2, fg=DIM); line.config(bg=BG)

    def _build_control(self, root):
        root.columnconfigure(0, weight=0); root.columnconfigure(1, weight=1); root.rowconfigure(0, weight=1)
        left = tk.Frame(root, bg=BG); left.grid(row=0, column=0, sticky="nsw", padx=(0, 8), pady=6)
        right = tk.Frame(root, bg=BG); right.grid(row=0, column=1, sticky="nsew", pady=6)

        # ---- HEDEF TAKİBİ ----
        outer, f, head = self._panel(left, "HEDEF TAKİBİ"); outer.pack(fill="x")
        body = tk.Frame(f, bg=PANEL); body.pack(fill="x", padx=10, pady=(4, 0))
        self.lbl_aim = tk.Label(body, text="—", bg=PANEL, fg=DIM, font=MONO_XL, width=6, anchor="e"); self.lbl_aim.pack(side="left")
        col = tk.Frame(body, bg=PANEL); col.pack(side="left", padx=(6, 0), anchor="s", pady=(0, 8))
        self._lbl(col, "° nişan hatası").pack(anchor="w")
        self.lbl_ring = self._lbl(col, "", font=UI_B); self.lbl_ring.pack(anchor="w")
        self.lbl_aim2 = self._lbl(f, "sağ-sol —      yukarı-aşağı —", font=MONO); self.lbl_aim2.pack(anchor="w", padx=12)
        r = tk.Frame(f, bg=PANEL); r.pack(fill="x", padx=10, pady=(4, 2))
        # üç durum düğmesi: açıkken kendi rengiyle yanar, tekrar basınca kapanır
        self.btn_manual = btn(r, "Manuel mod", PANEL2, self._manual_click, width=14); self.btn_manual.pack(side="left", padx=(0, 4))
        self.btn_lock = btn(r, "Hedefe kilitle", PANEL2, self._lock_click, width=14); self.btn_lock.pack(side="left", padx=4)
        self.btn_auto = btn(r, "Otomatik takip", PANEL2, self._auto_click, width=14); self.btn_auto.pack(side="left", padx=4)
        self._state_btns((False, False, False))
        r = tk.Frame(f, bg=PANEL); r.pack(fill="x", padx=10, pady=(4, 8))
        self.var_laser = tk.BooleanVar(value=False)
        ttk.Checkbutton(r, text="Lazer açık", variable=self.var_laser, command=self._laser_click).pack(side="left")
        self.var_en_all = tk.BooleanVar(value=False)
        ttk.Checkbutton(r, text="Üç sürücü açık", variable=self.var_en_all, command=self._enable_all).pack(side="left", padx=(14, 0))
        btn(r, "Tilt açısını IMU'dan ayarla", SLATE, lambda: self._send(ml.MSG_TILT_FROM_IMU), small=True).pack(side="right")

        # ---- DURUM ----
        outer, f, head = self._panel(left, "DURUM IŞIKLARI"); outer.pack(fill="x", pady=(6, 0))
        r = tk.Frame(f, bg=PANEL); r.pack(fill="x", padx=10, pady=(4, 8))
        self.health = {}
        for key, txt in (("link", "kart bağlantısı"), ("imu", "IMU"), ("enc_pan", "enkoder pan"), ("enc_tilt", "enkoder tilt"), ("enc_ce", "enkoder CE"), ("mot", "sürücüler")):
            c = tk.Frame(r, bg=PANEL); c.pack(side="left", padx=(0, 10))
            dot = tk.Label(c, text="●", bg=PANEL, fg=BORDER, font=("Segoe UI", 12)); dot.pack(side="left")
            self._lbl(c, txt, font=UI_S).pack(side="left")
            self.health[key] = dot

        # ---- IMU ----
        outer, f, head = self._panel(left, "IMU"); outer.pack(fill="x", pady=(6, 0))
        self.lbl_imu_state = self._lbl(head, "", font=UI_S, bg=HEAD); self.lbl_imu_state.pack(side="right", padx=10)
        g = tk.Frame(f, bg=PANEL); g.pack(fill="x", padx=10, pady=(4, 8))
        self.imu_vals = {}
        for i, (name, key) in enumerate((("Roll", "roll"), ("Pitch", "pitch"), ("Yaw", "yaw"))):
            self._lbl(g, name, font=UI_S).grid(row=0, column=i, padx=8, sticky="w")
            lbl = tk.Label(g, text="—", bg=PANEL, fg=TEXT, font=("Consolas", 17, "bold"), width=9, anchor="w"); lbl.grid(row=1, column=i, padx=8, sticky="w")
            self.imu_vals[key] = lbl
        btn(g, "IMU'yu yeniden başlat", SLATE, lambda: self._send(ml.MSG_IMU_INIT), small=True).grid(row=1, column=3, padx=(10, 0), sticky="e")

        # ---- Enkoderler ----
        outer, f, head = self._panel(left, "ENKODERLER"); outer.pack(fill="x", pady=(6, 0))
        self.lbl_loop = self._lbl(head, "", font=UI_S, bg=HEAD); self.lbl_loop.pack(side="right", padx=10)
        g = tk.Frame(f, bg=PANEL); g.pack(fill="x", padx=10, pady=(4, 8))
        for j, h in enumerate(("eksen", "ölçülen °", "istenen °", "fark °", "hız °/s", "mıknatıs\nsinyali", "durum")):
            self._lbl(g, h, font=("Segoe UI", 8, "bold"), justify="center").grid(row=0, column=j, padx=4)
        self.enc_cells = {}
        for i, a in enumerate(AXES, start=1):
            tk.Label(g, text=a.upper(), bg=PANEL, fg=TEXT, font=UI_B).grid(row=i, column=0, padx=4, sticky="w")
            cells = []
            for j in range(1, 7):
                lbl = tk.Label(g, text="—", bg=PANEL, fg=TEXT, font=MONO, width=8, anchor="e"); lbl.grid(row=i, column=j, padx=4)
                cells.append(lbl)
            self.enc_cells[a] = cells

        # ---- Eksenler ----
        self.axis_vals = {}
        self.goto_val, self.jog_speed, self.osc_a, self.osc_b, self.osc_dwell, self.osc_lbl, self.zero_val = {}, {}, {}, {}, {}, {}, {}
        for a in AXES:
            outer, f, head = self._panel(right, AXIS_TITLE[a]); outer.pack(fill="x", pady=(0, 6))
            v = tk.BooleanVar(value=False); self.en_vars[a] = v
            ttk.Checkbutton(head, text="Sürücü açık", variable=v, style="Head.TCheckbutton", command=lambda a=a: self._enable_axis(a)).pack(side="left", padx=(10, 0))
            self.osc_lbl[a] = self._lbl(head, "", fg=PURPLE, font=UI_B, bg=HEAD); self.osc_lbl[a].pack(side="right", padx=10)
            vals = {}
            vr = tk.Frame(f, bg=PANEL); vr.pack(fill="x", padx=10, pady=(4, 0))
            for key, txt in (("cmd", "istenen açı °"), ("enc", "ölçülen açı °"), ("err", "fark °"), ("vel", "hız °/s")):
                c = tk.Frame(vr, bg=PANEL); c.pack(side="left", padx=(0, 20))
                self._lbl(c, txt, font=UI_S).pack(anchor="w")
                lbl = tk.Label(c, text="—", bg=PANEL, fg=TEXT, font=("Consolas", 16, "bold"), width=8, anchor="w"); lbl.pack(anchor="w")
                vals[key] = lbl
            self.axis_vals[a] = vals
            # satır 1: açıya git + adım at + (sağda) bulunulan açıyı kabul et
            r = tk.Frame(f, bg=PANEL); r.pack(fill="x", padx=10, pady=(6, 0))
            self._lbl(r, "Açıya git").pack(side="left")
            gv = tk.StringVar(value="0"); self.goto_val[a] = gv
            e = ttk.Entry(r, textvariable=gv, width=6, justify="center"); e.pack(side="left", padx=(4, 0))
            e.bind("<Return>", lambda ev, a=a: self._goto(a))
            self._lbl(r, "°").pack(side="left", padx=(2, 4))
            btn(r, "Git", ACCENT, lambda a=a: self._goto(a), width=4).pack(side="left")
            self._lbl(r, "     Adım at").pack(side="left")
            for d in (-10, -1, -0.1, 0.1, 1, 10):
                btn(r, f"{d:+g}°", PANEL2, lambda a=a, d=d: self._step(a, d), width=4, small=True).pack(side="left", padx=(3, 0))
            btn(r, "Kabul et", SLATE, lambda a=a: self._zero(a), small=True).pack(side="right")
            self._lbl(r, "°").pack(side="right", padx=(2, 6))
            zv = tk.StringVar(value="0"); self.zero_val[a] = zv
            ttk.Entry(r, textvariable=zv, width=5, justify="center").pack(side="right")
            self._lbl(r, "Şu an bulunduğu açı:").pack(side="right", padx=(0, 4))
            # satır 2: basılı tutunca döner + salınım
            r = tk.Frame(f, bg=PANEL); r.pack(fill="x", padx=10, pady=(6, 10))
            self._lbl(r, "Basılı tutunca döner").pack(side="left")
            sp = tk.StringVar(value="10"); self.jog_speed[a] = sp
            bl = btn(r, "◀", TEAL, width=3, small=True); bl.pack(side="left", padx=(6, 0))
            ttk.Entry(r, textvariable=sp, width=4, justify="center").pack(side="left", padx=3)
            self._lbl(r, "°/s").pack(side="left", padx=(0, 3))
            br = btn(r, "▶", TEAL, width=3, small=True); br.pack(side="left")
            for b, sgn in ((bl, -1), (br, +1)):
                b.bind("<ButtonPress-1>", lambda ev, a=a, s=sgn: self._jog(a, s))
                b.bind("<ButtonRelease-1>", lambda ev, a=a: self._jog(a, 0))
            self._lbl(r, "     Salınım:").pack(side="left")
            va = tk.StringVar(value="0"); self.osc_a[a] = va
            ttk.Entry(r, textvariable=va, width=5, justify="center").pack(side="left", padx=(4, 0))
            self._lbl(r, "° ↔").pack(side="left", padx=3)
            vb = tk.StringVar(value={"pan": "90", "tilt": "45", "ce": "4"}[a]); self.osc_b[a] = vb
            ttk.Entry(r, textvariable=vb, width=5, justify="center").pack(side="left")
            self._lbl(r, "°,  uçta bekle").pack(side="left", padx=(3, 3))
            vd = tk.StringVar(value="500"); self.osc_dwell[a] = vd
            ttk.Entry(r, textvariable=vd, width=5, justify="center").pack(side="left")
            self._lbl(r, "ms").pack(side="left", padx=(3, 6))
            btn(r, "Salınımı başlat", PURPLE, lambda a=a: self._osc(a, True), small=True).pack(side="left", padx=(0, 4))
            btn(r, "Salınımı durdur", AMBER, lambda a=a: self._osc(a, False), small=True).pack(side="left")

    def _build_settings(self, root):
        r = tk.Frame(root, bg=BG); r.pack(fill="x", pady=(6, 6))
        btn(r, "Karttan oku", SLATE, self._params_read).pack(side="left")
        btn(r, "Karta yaz", GREEN, self._params_write).pack(side="left", padx=6)
        btn(r, "Bilgisayara kaydet", ACCENT, self._params_save_pc).pack(side="left", padx=(18, 6))
        btn(r, "Bilgisayardan yükle ve karta yaz", PURPLE, self._params_load_pc).pack(side="left")
        btn(r, "Kalıcı kaydı sil", SLATE, self._params_clear_nv).pack(side="left", padx=(18, 0))
        self.lbl_params = tk.Label(r, text="", bg=BG, fg=DIM, font=UI); self.lbl_params.pack(side="left", padx=16)
        self.lbl_nv = tk.Label(r, text="", bg=BG, fg=DIM, font=UI); self.lbl_nv.pack(side="left")
        tk.Label(root, text="Kutuya yeni değeri yazıp \"Karta yaz\"a bas: değer karta yazılır ve kalıcı kaydedilir, güç gidip gelince de kalır. "
                            "\"Bilgisayara kaydet\" yedek içindir. \"Kalıcı kaydı sil\": kart bir sonraki açılışta yazılım varsayılanlarıyla başlar.",
                 bg=BG, fg=DIM, font=UI_S, wraplength=1300, justify="left").pack(anchor="w", pady=(0, 6))
        row = tk.Frame(root, bg=BG); row.pack(fill="both", expand=True)
        outer, f, head = self._panel(row, "EKSEN AYARLARI"); outer.pack(side="left", fill="y", anchor="n")
        g = tk.Frame(f, bg=PANEL); g.pack(anchor="w", padx=10, pady=(2, 8))
        for j, a in enumerate(AXES, start=1):
            tk.Label(g, text=a.upper(), bg=PANEL, fg=ACCENT2, font=("Segoe UI", 11, "bold")).grid(row=0, column=j, padx=6)
        self._lbl(g, "ne işe yarar", font=UI_S).grid(row=0, column=4, sticky="w", padx=(6, 0))
        for i, (label, tmpl, desc) in enumerate(SETTING_ROWS, start=1):
            tk.Label(g, text=label, bg=PANEL, fg=TEXT, font=UI).grid(row=i, column=0, sticky="w", padx=(0, 8), pady=1)
            for j, a in enumerate(AXES, start=1):
                v = tk.StringVar(value=""); self.setting_vars[(tmpl, a)] = v
                ttk.Entry(g, textvariable=v, width=8, justify="center").grid(row=i, column=j, padx=6)
            self._lbl(g, desc, font=UI_S, wraplength=330, justify="left").grid(row=i, column=4, sticky="w", padx=(6, 0))
        outer, f2, head = self._panel(row, "OTOMATİK TAKİP AYARLARI"); outer.pack(side="left", fill="y", anchor="n", padx=(8, 0))
        g2 = tk.Frame(f2, bg=PANEL); g2.pack(anchor="w", padx=10, pady=(2, 8))
        for i, (label, name, desc) in enumerate(AIM_ROWS):
            tk.Label(g2, text=label, bg=PANEL, fg=TEXT, font=UI).grid(row=i, column=0, sticky="w", padx=(0, 8), pady=1)
            v = tk.StringVar(value=""); self.setting_vars[(name, "")] = v
            ttk.Entry(g2, textvariable=v, width=8, justify="center").grid(row=i, column=1, padx=6)
            self._lbl(g2, desc, font=UI_S, wraplength=260, justify="left").grid(row=i, column=2, sticky="w", padx=(6, 0))
        tk.Label(f2, text="Mikroadım kartta \"mikroadım başına derece\" olarak tutulur: 1,8 / mikroadım / oran.\nMotor akımı sürücü kartındaki vidadan ayarlanır, yazılımdan değil.",
                 bg=PANEL, fg=DIM, font=UI_S, justify="left").pack(anchor="w", padx=10, pady=(4, 8))

    # ================= bağlantı =================
    def _port_name(self):
        v = self.port_var.get().strip()
        return v.split()[0] if v.upper().startswith("COM") else ""

    def _refresh_ports(self, prefer=None):
        """Bilgisayardaki seri portları listeler (ST-Link olan başa gelir); seçili port listede kalırsa korunur."""
        try:
            from serial.tools import list_ports
            ports = [q for q in list_ports.comports() if not str(q.hwid).upper().startswith("ROOT")]   # sistemin sanal portlari haric
            ports = sorted(ports, key=lambda q: (0 if q.vid in (0x0483, 0x16C0) and not ml.is_lora_port(q) else (1 if ml.is_lora_port(q) else 2), q.device))
        except Exception:  # noqa: BLE001
            ports = []
        vals = []
        for q in ports:
            tag = ("LoRa yer istasyonu" if ml.is_lora_port(q) else
                   "ST-Link" if q.vid == 0x0483 else
                   "Teensy 4.1 terminal" if q.vid == 0x16C0 else (q.description or ""))
            vals.append(f"{q.device}  {tag}".strip())
        self.cb_port["values"] = vals or ["seri port bulunamadı"]
        cur = self._port_name() if prefer is None else prefer
        for v in vals:
            if v.split()[0] == cur:
                self.port_var.set(v)
                return
        self.port_var.set(vals[0] if vals else "seri port bulunamadı")

    def _toggle_conn(self):
        if self.link or getattr(self, "_want_conn", False):
            self._disconnect()
            return
        name = self._port_name()
        if not name:
            self._log("seri port bulunamadi: kartin USB kablosunu tak, listeyi acinca yenilenir", "amber")
            return
        self.is_lora = False
        try:
            from serial.tools import list_ports
            q = next((q for q in list_ports.comports() if q.device == name), None)
            self.is_lora = bool(q) and ml.is_lora_port(q)
        except Exception:  # noqa: BLE001
            pass
        if not self._open_link(name):
            return
        self._conn_name = name
        self._want_conn = True
        self.btn_conn.config(text="Bağlantıyı kes", bg=SLATE)
        self._log(f"{name} acildi" + (" (LoRa, 9600 baud: telemetri saniyede 1, ayarlari okumak icin Ayarlar > Karttan oku)" if self.is_lora else ""), "green")
        if not self.is_lora:
            self.after(400, self._params_read)

    def _open_link(self, name, quiet=False):
        """Portu açar, gelen mesaj işleyicisini kurar. Port sonradan koparsa gönderme hata vermez (yeniden bağlanma bekler)."""
        try:
            link = ml.Link(name, 9600 if self.is_lora else 115200)
        except Exception as e:  # noqa: BLE001
            if not quiet:
                self._log(f"{name} acilamadi: {e}", "red")
            return False
        self.link = link
        orig = self.link._handle

        def handler(mid, payload):
            if mid == ml.MSG_TEXT:
                self._on_text(payload.decode("ascii", "replace"))
            elif mid == ml.MSG_PONG:
                up, ver = struct.unpack("<II", payload)
                self._log(f"kart yanit verdi: calisma suresi {up / 1000:.1f} s, yazilim {ver >> 16}.{(ver >> 8) & 0xFF}.{ver & 0xFF}", "dim")
            elif mid == ml.MSG_ACK:
                rid, st = payload
                self._ack_got(rid)
                if rid == ml.MSG_PARAM_SAVE:
                    ok = st == 0
                    self.lbl_nv.config(text=("kalıcı kayıt tamam " if ok else "KALICI KAYIT YAPILAMADI ") + time.strftime("%H:%M:%S"),
                                       fg=GREEN if ok else RED)
                if st != 0:
                    self._log(f"komut 0x{rid:02X} kabul edilmedi: {ml.ACK.get(st, st)}", "amber")
            elif mid == ml.MSG_PARAM_VALUE:
                pid, val, ok = struct.unpack("<HfB", payload)
                if ok:
                    self.link.params[pid] = val
            else:
                orig(mid, payload)

        self.link._handle = handler
        raw_send = self.link.send

        def safe_send(mid, payload=b""):
            try:
                raw_send(mid, payload)
            except Exception:  # noqa: BLE001  port koptu: yeniden bağlanma halleder
                pass

        self.link.send = safe_send
        self.link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 50))
        self.link.send(ml.MSG_PING)
        return True

    def _watch_link(self):
        """USB koptuysa (kart yeniden başladı, kablo oynadı) okuma iş parçacığı durur: portu saniyede bir yeniden açmayı dener."""
        if getattr(self, "_want_conn", False):
            if self.link and not self.link.th.is_alive():
                self._relog = self.link.log.name if self.link.log else None
                try:
                    self.link.close()
                except Exception:  # noqa: BLE001
                    pass
                self.link = None
                self._log("BAGLANTI KOPTU: kart yeniden basliyor ya da kablo oynadi; yeniden baglaniliyor...", "red")
            if self.link is None:
                self.pill_conn.config(text="BAĞLANTI KOPTU · yeniden bağlanıyor", bg=RED, fg="white")
                self.health["link"].config(fg=RED)
                if self._open_link(self._conn_name, quiet=True):
                    self._log(f"{self._conn_name} yeniden baglandi", "green")
                    if not self.is_lora:
                        self.after(400, self._params_read)
                    if getattr(self, "_relog", None):           # kayıt sürüyordu: ayrı dosyada devam (öncekinin üstüne yazma)
                        base, ext = os.path.splitext(self._relog)
                        self._start_log_file(f"{base}_devam_{time.strftime('%H%M%S')}{ext or '.csv'}")
                        self._relog = None
        self.after(1000, self._watch_link)

    def _disconnect(self):
        self._want_conn = False
        if self.link:
            try:
                self.link.send(ml.MSG_STOP)
                self.link.send(ml.MSG_TELEM_RATE, struct.pack("<H", 20))
                time.sleep(0.1)
                self.link.close()
            except Exception:  # noqa: BLE001
                pass
            self.link = None
        self.btn_conn.config(text="Bağlan", bg=ACCENT)
        self.pill_conn.config(text="bağlı değil", bg=BORDER, fg=TEXT)
        self.pill_mode.config(text="mod: —", bg=BORDER, fg=TEXT)
        self.pill_lock.config(text="hedef kilidi yok", bg=BORDER, fg=TEXT)
        self.telem = None
        self._state_btns((False, False, False))
        for dot in self.health.values():
            dot.config(fg=BORDER)

    # LoRa yarı çift yönlü: terminal yayın yaparken gelen komut kaybolabilir (30 Eyl testinde ~%25). LoRa'dayken onay (ACK)
    # döndüren komutlar 1,5 s içinde onaylanmazsa 3 kereye kadar yeniden gönderilir. Komutlar mutlak değerli (açıya git,
    # lazer aç, mod ...) olduğu için tekrar göndermek güvenli. Anahtar: komut + eksen / parametre no.
    _ACKLI = (ml.MSG_SET_MODE, ml.MSG_JOG, ml.MSG_STOP, ml.MSG_GOTO, ml.MSG_OSC, ml.MSG_AXIS_ZERO, ml.MSG_LOCK, ml.MSG_UNLOCK,
              ml.MSG_TILT_FROM_IMU, ml.MSG_LASER, ml.MSG_ENABLE, ml.MSG_PARAM_SET, ml.MSG_IMU_INIT, ml.MSG_PARAM_SAVE)

    def _send(self, mid, payload=b""):
        if not self.link:
            return
        self.link.send(mid, payload)
        if getattr(self, "is_lora", False) and mid in self._ACKLI:
            if mid in (ml.MSG_GOTO, ml.MSG_JOG, ml.MSG_OSC, ml.MSG_AXIS_ZERO) or (mid == ml.MSG_ENABLE and len(payload) == 2):
                key = (mid, payload[:1])
            elif mid == ml.MSG_PARAM_SET:
                key = (mid, payload[:2])
            else:
                key = (mid,)
            self._pending = getattr(self, "_pending", {})
            self._pending[key] = [payload, time.time(), 1]

    def _ack_got(self, rid):
        """Bu komut türünün en eski bekleyen kaydını kapat (ACK eksen bilgisi taşımaz)."""
        pend = getattr(self, "_pending", {})
        ks = sorted((k for k in pend if k[0] == rid), key=lambda k: pend[k][1])
        if ks:
            del pend[ks[0]]

    def _retry_tick(self):
        pend = getattr(self, "_pending", {})
        if self.link and getattr(self, "is_lora", False):
            now = time.time()
            for k in list(pend):
                payload, t, n = pend[k]
                if now - t > 2.5:
                    if n >= 4:
                        del pend[k]
                        self._log(f"komut 0x{k[0]:02X} LoRa'da 4 denemede onaylanmadi", "amber")
                    else:
                        self.link.send(k[0], payload)
                        pend[k] = [payload, now, n + 1]
                        self._retries = getattr(self, "_retries", 0) + 1
        else:
            pend.clear()
        self.after(200, self._retry_tick)

    # ================= komutlar =================
    def _estop(self):
        self._send(ml.MSG_SET_MODE, bytes([1]))           # otomatik takip kapanır: sürücü açılınca kendiliğinden yürümesin
        self._send(ml.MSG_ENABLE, bytes([0]))
        for a in AXES:
            self.osc_on[a] = False
            self.en_vars[a].set(False)
        self.var_en_all.set(False)
        self._log("ACIL DUR: hareket durduruldu, suruculer kapatildi (motorlar serbest), manuel mod", "red")

    def _stop_click(self):
        """DUR: otomatik takipteyse önce manuel moda geçer (yoksa takip motorları hemen yeniden sürer), sonra eksenleri durdurur."""
        if self.telem and ml.MODES.get(self.telem["mode"]) == "AUTO":
            self._send(ml.MSG_SET_MODE, bytes([1]))
            self._log("DUR: otomatik takip kapatildi, manuel moda gecildi (kilit duruyor)", "amber")
        self._send(ml.MSG_STOP)

    def _state_btns(self, on):
        """Manuel / kilit / otomatik düğmelerinin yanık-sönük görünümü."""
        for b, active, col, txt in zip((self.btn_manual, self.btn_lock, self.btn_auto), on, (TEAL, ACCENT, GREEN),
                                       ("Manuel mod", "Hedefe kilitle", "Otomatik takip")):
            if active:
                b.config(bg=col, fg="white" if col in WHITE_TEXT else BG)
            else:
                b.config(bg=PANEL2, fg=TEXT)

    def _manual_click(self):
        if self.telem and ml.MODES.get(self.telem["mode"]) == "MANUAL":
            self._log("zaten manuel modda (manuel mod kapatilamaz; otomatik takip icin once kilitle)", "dim")
            return
        self._send(ml.MSG_SET_MODE, bytes([1]))

    def _lock_click(self):
        if self.telem and self.telem["flags"] & 0x01:
            self._send(ml.MSG_UNLOCK)          # tekrar basildi: kilit birakilir (otomatikteyse manuele doner)
        else:
            self._send(ml.MSG_LOCK)

    def _auto_click(self):
        if self.telem and ml.MODES.get(self.telem["mode"]) == "AUTO":
            self._send(ml.MSG_SET_MODE, bytes([1]))   # tekrar basildi: takip kapanir, manuel mod
        else:
            self._send(ml.MSG_SET_MODE, bytes([2]))

    # Kutucuklar kartın gerçek durumunu gösterir (telemetri bayrakları, _sync_boxes). Tıklanınca kart komutu işleyene
    # kadar kısa süre telemetriden güncellenmez, yoksa eski telemetri kutucuğu bir an geri çevirir.
    def _hold_sync(self):
        self._sync_hold_until = time.time() + 0.8

    def _laser_click(self):
        self._hold_sync()
        self._send(ml.MSG_LASER, bytes([1 if self.var_laser.get() else 0]))

    def _enable_all(self):
        self._hold_sync()
        on = self.var_en_all.get()
        self._send(ml.MSG_ENABLE, bytes([1 if on else 0]))
        for a in AXES:
            self.en_vars[a].set(on)
            if not on:
                self.osc_on[a] = False

    def _enable_axis(self, a):
        self._hold_sync()
        on = self.en_vars[a].get()
        self._send(ml.MSG_ENABLE, bytes([AXES.index(a), 1 if on else 0]))
        if not on:
            self.osc_on[a] = False
        self.var_en_all.set(all(self.en_vars[x].get() for x in AXES))

    def _sync_boxes(self, flags):
        """Lazer ve sürücü kutucuklarını kartın bildirdiği duruma getirir (bayrak bitleri: 1 lazer, 5-7 pan/tilt/CE sürücü)."""
        if time.time() < getattr(self, "_sync_hold_until", 0.0):
            return
        if self.var_laser.get() != bool(flags & 0x02):
            self.var_laser.set(bool(flags & 0x02))
        for i, a in enumerate(AXES):
            on = bool(flags & (0x20 << i))
            if self.en_vars[a].get() != on:
                self.en_vars[a].set(on)
                if not on:
                    self.osc_on[a] = False
        allon = all(self.en_vars[a].get() for a in AXES)
        if self.var_en_all.get() != allon:
            self.var_en_all.set(allon)

    def _jog(self, axis, sign):
        try:
            sp = float(self.jog_speed[axis].get().replace(",", "."))
        except ValueError:
            sp = 0.0
        self.osc_on[axis] = False
        self._send(ml.MSG_JOG, struct.pack("<Bf", AXES.index(axis), sign * sp))
        # LoRa'da basılı kaldıkça 0,5 s'de bir tazele (kart 1,5 s tazelenmeyen döndürmeyi kendisi durdurur)
        self._jog_keep = getattr(self, "_jog_keep", {})
        if axis in self._jog_keep:
            self.after_cancel(self._jog_keep.pop(axis))
        if getattr(self, "is_lora", False) and sign != 0:
            self._jog_keep[axis] = self.after(500, lambda: self._jog(axis, sign))

    def _goto(self, axis, deg=None):
        if deg is None:
            try:
                deg = float(self.goto_val[axis].get().replace(",", "."))
            except ValueError:
                self._log(f"{axis}: aci sayi olmali", "amber")
                return
        self.osc_on[axis] = False
        self._send(ml.MSG_GOTO, struct.pack("<Bf", AXES.index(axis), deg))

    def _step(self, axis, delta):
        if not self.telem:
            return
        self._goto(axis, self.telem["cmd_" + axis] + delta)

    def _osc(self, axis, on):
        ax = AXES.index(axis)
        if not on:
            self._send(ml.MSG_OSC, bytes([ax]))
            self.osc_on[axis] = False
            return
        try:
            a = float(self.osc_a[axis].get().replace(",", ".")); b = float(self.osc_b[axis].get().replace(",", "."))
            dw = int(float(self.osc_dwell[axis].get()))
        except ValueError:
            self._log("salinim: iki aci ve bekleme suresi sayi olmali", "amber")
            return
        self._send(ml.MSG_OSC, struct.pack("<BffH", ax, a, b, max(0, min(65535, dw))))
        self.osc_on[axis] = True

    def _zero(self, axis):
        try:
            deg = float(self.zero_val[axis].get().replace(",", "."))
        except ValueError:
            deg = 0.0
        self.osc_on[axis] = False
        self._send(ml.MSG_AXIS_ZERO, struct.pack("<Bf", AXES.index(axis), deg))
        self._log(f"{axis}: su anki konum {deg:g} derece olarak kabul edildi", "dim")

    # ================= parametreler =================
    def _pname(self, tmpl, a):
        return tmpl.format(a=a, lim=a) if a else tmpl

    def _params_read(self):
        if not self.link:
            self.lbl_params.config(text="once karta baglan")
            return
        if getattr(self, "is_lora", False):
            self.lbl_params.config(text="LoRa uzerinden okunuyor...")
            def parca(i=0):
                if not self.link:
                    return
                for pid in range(i, min(i + 6, len(ml.PARAM_NAMES))):
                    self.link.send(ml.MSG_PARAM_GET, struct.pack("<H", pid))
                if i + 6 < len(ml.PARAM_NAMES):
                    self.after(400, lambda: parca(i + 6))
                else:
                    self.after(2500, self._params_fill)
            parca()
            return
        for pid in range(len(ml.PARAM_NAMES)):
            self.link.send(ml.MSG_PARAM_GET, struct.pack("<H", pid))
            time.sleep(0.004)
        self.after(600, self._params_fill)

    def _params_fill(self):
        if not self.link:
            return
        p = self.link.params
        n = 0
        for (tmpl, a), var in self.setting_vars.items():
            if tmpl == "usteps_{a}":
                dps = p.get(ml.PARAM_NAMES.index(f"dps_{a}")); ratio = p.get(ml.PARAM_NAMES.index(f"enc_ratio_{a}"))
                if dps and ratio:
                    var.set(f"{1.8 / (dps * ratio):.0f}"); n += 1
                continue
            name = self._pname(tmpl, a)
            if name in ml.PARAM_NAMES and ml.PARAM_NAMES.index(name) in p:
                var.set(f"{p[ml.PARAM_NAMES.index(name)]:g}"); n += 1
        self.lbl_params.config(text=f"karttan {n} ayar okundu  {time.strftime('%H:%M:%S')}")

    def _params_collect(self):
        out = {}
        for (tmpl, a), var in self.setting_vars.items():
            txt = var.get().strip().replace(",", ".")
            if not txt:
                continue
            try:
                val = float(txt)
            except ValueError:
                self._log(f"ayar: {self._pname(tmpl, a)} sayi degil: {txt}", "amber")
                continue
            if tmpl == "usteps_{a}":
                try:
                    ratio = float(self.setting_vars[("enc_ratio_{a}", a)].get().replace(",", "."))
                except ValueError:
                    ratio = 5.0 if a != "ce" else 1.0
                out[f"dps_{a}"] = 1.8 / (val * ratio)
                out["mot_usteps"] = val
            else:
                out[self._pname(tmpl, a)] = val
        return out

    def _params_write(self, vals=None):
        if not self.link:
            self._log("ayar yazmak icin once karta baglan", "amber")
            return
        vals = vals or self._params_collect()
        n = 0
        for name, val in vals.items():
            if name in ml.PARAM_NAMES:
                self._send(ml.MSG_PARAM_SET, struct.pack("<Hf", ml.PARAM_NAMES.index(name), float(val)))
                time.sleep(0.004); n += 1
        self.lbl_params.config(text=f"karta {n} ayar yazildi  {time.strftime('%H:%M:%S')}")
        self.lbl_nv.config(text="kalıcı kayıt bekleniyor...", fg=DIM)
        self._save_when_done(time.time())
        self.after(300, self._params_read)

    def _save_when_done(self, t0):
        """Ayar yazmaları onaylanınca kalıcı kayıt komutu gider (LoRa'da tekrar gönderilen yazma kayıttan sonra kalmasın)."""
        pend = getattr(self, "_pending", {})
        if self.link and any(k[0] == ml.MSG_PARAM_SET for k in pend) and time.time() - t0 < 30:
            self.after(300, lambda: self._save_when_done(t0))
            return
        self._send(ml.MSG_PARAM_SAVE)

    def _params_clear_nv(self):
        if not self.link:
            self._log("once karta baglan", "amber")
            return
        if messagebox.askyesno("Kalıcı kaydı sil", "Karttaki kalıcı ayar kaydı silinsin mi?\n\nŞu anki ayarlar değişmez; kart bir sonraki "
                                                   "açılışta yazılım varsayılanlarıyla başlar. Geri almak için \"Karta yaz\"a bas."):
            self._send(ml.MSG_PARAM_SAVE, b"\x00")
            self.lbl_nv.config(text="kalıcı kayıt silindi, sonraki açılış varsayılan", fg=AMBER)

    def _params_save_pc(self):
        vals = self._params_collect()
        with open(PARAMS_FILE, "w", encoding="utf-8") as fh:
            json.dump(vals, fh, indent=1, ensure_ascii=False)
        self.lbl_params.config(text=f"bilgisayara kaydedildi: {os.path.basename(PARAMS_FILE)}")

    def _params_load_pc(self):
        if not os.path.exists(PARAMS_FILE):
            self.lbl_params.config(text="bilgisayarda kayit yok: params.json")
            return
        with open(PARAMS_FILE, encoding="utf-8") as fh:
            vals = json.load(fh)
        for (tmpl, a), var in self.setting_vars.items():
            if tmpl == "usteps_{a}":
                dps, ratio = vals.get(f"dps_{a}"), vals.get(f"enc_ratio_{a}")
                if dps and ratio:
                    var.set(f"{1.8 / (dps * ratio):.0f}")
                continue
            name = self._pname(tmpl, a)
            if name in vals:
                var.set(f"{vals[name]:g}")
        self._params_write(vals)

    def _toggle_log(self):
        if not self.link:
            self._log("kayit icin once karta baglan", "amber")
            return
        if self.link.log:
            self.link.stop_log()
            self.btn_log.config(text="Veri kaydını başlat", bg=SLATE)
            self._log("veri kaydi durdu", "dim")
        else:
            path = self.log_var.get().strip() or "kayit.csv"
            if os.path.exists(path):                         # önceki kaydı silme: saat ekle
                base, ext = os.path.splitext(path)
                path = f"{base}_{time.strftime('%H%M%S')}{ext or '.csv'}"
            self._start_log_file(path)

    def _start_log_file(self, path):
        try:
            self.link.start_log(path)
        except OSError as e:
            self._log(f"kayit dosyasi acilamadi: {e}", "red")
            return
        self.btn_log.config(text="Veri kaydını durdur", bg=RED)
        self._log(f"veri kaydi basladi: {os.path.abspath(path)}", "green")

    # ================= gelen veri =================
    def _on_text(self, t):
        m = re.match(r"enc (pan|tilt|ce): raw=(\d+) agc=(\d+) OCF=(\d) COF=(\d) yakin=(\d) uzak=(\d)", t)
        if m:
            a = m.group(1)
            agc, ocf, yakin, uzak = int(m.group(3)), int(m.group(4)), int(m.group(6)), int(m.group(7))
            if "okunamadi" in t:
                state = "yok"
            elif uzak:
                state = "mıknatıs uzak"
            elif yakin:
                state = "mıknatıs yakın"
            elif not ocf:
                state = "hazır değil"
            else:
                state = "iyi"
            self.enc_diag[a] = (agc, state)
            return
        tag = "red" if ("takildi" in t or "BULUNAMADI" in t or "kesildi" in t or "okunamiyor" in t) else ("green" if "kilit alindi" in t else None)
        self._log(t, tag)

    def _log(self, s, tag=None):
        self.txt.insert("end", time.strftime("%H:%M:%S "), "time")
        self.txt.insert("end", s + "\n", tag or "")
        self.txt.see("end")

    def _poll_diag(self):
        if self.link:
            if getattr(self, "is_lora", False):
                self._lora_tick = getattr(self, "_lora_tick", 0) + 1
                if self._lora_tick % 5 == 0:
                    self.link.send(ml.MSG_ENC_DIAG)      # 5 s'de bir enkoder durumu; aynı zamanda LoRa bağlantısını canlı tutar
            else:
                self.link.send(ml.MSG_ENC_DIAG)
        self.after(1000, self._poll_diag)

    def _tick(self):
        if self.link and self.link.last_telem:
            d = dict(zip(ml.TELEM_COLS, self.link.last_telem))
            self.telem = d
            t_ms = d["t_ms"]
            if self.last_t_ms is not None and t_ms < self.last_t_ms:
                self._log("KART YENIDEN BASLADI: ayarlar karttaki kalici kayittan geldi; suruculeri yeniden ac, hedefe yeniden kilitle", "amber")
                self.after(1500, self._params_read)
                for a in AXES:
                    self.en_vars[a].set(False)
                self.var_en_all.set(False)
            self.last_t_ms = t_ms
            # IMU hızı iki telemetri arasındaki kart saatinden (LoRa'da saniyede 1 telemetri: sabit 1 s pencere boş kalabiliyordu)
            n, tprev = self.last_count
            if not tprev or d["t_ms"] < tprev:
                self.last_count = (d["imu_count"], d["t_ms"])
            elif d["t_ms"] - tprev >= 400:
                self.imu_hz = (d["imu_count"] - n) / ((d["t_ms"] - tprev) / 1000.0)
                self.last_count = (d["imu_count"], d["t_ms"])
            imu_ok = (d["flags"] >> 2) & 1
            for k in ("roll", "pitch", "yaw"):
                self.imu_vals[k].config(text=f"{d[k]:.2f}", fg=TEXT if imu_ok else DIM)
            self.lbl_imu_state.config(text=f"{'veri geliyor' if imu_ok else 'VERİ GELMİYOR'} · {self.imu_hz:.0f} Hz", fg=GREEN if imu_ok else RED)
            self.health["imu"].config(fg=GREEN if imu_ok else RED)
            for a in AXES:
                c = self.enc_cells[a]
                c[0].config(text=f"{d['enc_' + a]:.2f}")
                c[1].config(text=f"{d['cmd_' + a]:.2f}")
                c[2].config(text=f"{d['err_' + a]:.2f}")
                c[3].config(text=f"{d['vel_' + a]:.2f}")
                agc_state = self.enc_diag.get(a)
                if agc_state:
                    agc, state = agc_state
                    c[4].config(text=f"{agc:3d}")
                    c[5].config(text=state, fg=GREEN if state == "iyi" else RED)
                    self.health["enc_" + a].config(fg=GREEN if state == "iyi" else RED)
                v = self.axis_vals[a]
                v["cmd"].config(text=f"{d['cmd_' + a]:.2f}")
                v["enc"].config(text=f"{d['enc_' + a]:.2f}")
                err = d["err_" + a]
                v["err"].config(text=f"{err:+.2f}", fg=TEXT if abs(err) < 0.5 else RED)
                vel = d["vel_" + a]
                v["vel"].config(text=f"{vel:.1f}", fg=ACCENT2 if abs(vel) > 0.01 else TEXT)
                self.osc_lbl[a].config(text="SALINIM SÜRÜYOR" if self.osc_on[a] else "")
            self.lbl_loop.config(text=f"kart döngüsü {d['loop_us']} µs · alınan paket {d['rx_frames']} · bozuk {d['rx_errors']}")
            mode = ml.MODES.get(d["mode"], str(d["mode"]))
            self._state_btns((mode == "MANUAL", bool(d["flags"] & 0x01), mode == "AUTO"))
            mbg = {"MANUAL": ACCENT, "AUTO": GREEN}.get(mode, BORDER)
            self.pill_mode.config(text={"MANUAL": "MANUEL MOD", "AUTO": "OTOMATİK TAKİP", "INIT": "AÇILIŞ"}.get(mode, mode), bg=mbg, fg=self._pill_fg(mbg))
            en = bool(d["flags"] & 0x10)
            self.pill_conn.config(text=f"kart {d['t_ms'] / 1000:.0f} s açık · " + ("lazer AÇIK" if d["flags"] & 0x02 else "lazer kapalı"),
                                  bg=RED if d["flags"] & 0x02 else PANEL2, fg="white")
            self.health["mot"].config(fg=GREEN if en else DIM)
            self._sync_boxes(d["flags"])
            # Telemetri tazeliği: LoRa'da paket kaybolabilir; eski değerler güncel sanılmasın
            if self.link.last_telem is not getattr(self, "_seen_telem", None):
                self._seen_telem = self.link.last_telem; self._telem_time = time.time()
            yas = time.time() - getattr(self, "_telem_time", time.time())
            if yas > (4.0 if getattr(self, "is_lora", False) else 1.0):
                self.health["link"].config(fg=RED)
                self.pill_conn.config(text=f"VERİ GELMİYOR {yas:.0f} s", bg=AMBER, fg=BG)
            else:
                self.health["link"].config(fg=GREEN)
            if d["flags"] & 0x01:
                e = d["aim_err"]
                dist = self.link.params.get(ml.PARAM_NAMES.index("plx_d")) or 4.0      # hedef mesafesi: Ø10 = 5 cm yarıçap
                t10, t20 = math.degrees(math.atan(0.05 / dist)), math.degrees(math.atan(0.10 / dist))
                col = GREEN if e < t10 else (AMBER if e < t20 else RED)
                self.lbl_aim.config(text=f"{e:.2f}", fg=col)
                self.lbl_ring.config(text="Ø10 daire içinde" if e < t10 else ("Ø20 daire içinde" if e < t20 else "Ø20 dışında"), fg=col)
                self.lbl_aim2.config(text=f"sağ-sol {d['aim_side']:+.2f}°   yukarı-aşağı {d['aim_up']:+.2f}°", fg=TEXT)
                self.pill_lock.config(text="HEDEF KİLİTLİ", bg=GREEN, fg=BG)
            else:
                self.lbl_aim.config(text="—", fg=DIM); self.lbl_ring.config(text="")
                self.lbl_aim2.config(text="hedef kilidi yok: önce \"Hedefe kilitle\"", fg=DIM)
                self.pill_lock.config(text="hedef kilidi yok", bg=BORDER, fg=TEXT)
        self.after(50, self._tick)

    def _close(self):
        self._disconnect()
        self.destroy()


def find_port():
    """Terminale doğrudan bağlı portu bulur (Teensy seri numarasıyla, LoRa köprüsü değil); yoksa ST-Link; bulamazsa COM8."""
    try:
        p = ml.terminal_port()
        if p:
            return p
        from serial.tools import list_ports
        for p in list_ports.comports():
            if (p.vid == 0x0483 and p.pid in (0x374E, 0x374B, 0x3752, 0x3753)) or p.vid == 0x16C0:   # ST-Link ya da Teensy
                return p.device
    except Exception:  # noqa: BLE001
        pass
    return "COM8"


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    app = App(port)
    if "--selftest" in sys.argv:
        app._toggle_conn()
        app.after(3000, app._close)
    app.mainloop()


if __name__ == "__main__":
    main()
