# Arayüzün LoRa üzerinden uçtan uca testi (30 Eyl). Gerçek arayüz (mercom_gui.App) yer istasyonu portuna bağlanır;
# düğmelerin çağırdığı fonksiyonlar sırayla çalıştırılır, her birinin etkisi kartın telemetrisinden / ekrandaki
# değerlerden doğrulanır. Ayrıca ekrandaki sensör değerlerinin ne sıklıkla değiştiği sayılır.
# Kullanım: python gui_lora_test.py
import os, struct, time
import mercom_gui as g
import mercom_link as ml

PORT = ml.lora_port()
if PORT is None: print("YER ISTASYONU YOK"); raise SystemExit(1)
app = g.App(PORT)
app._toggle_conn()
acks = []
_h0 = app.link._handle
def _h(mid, p):
    if mid == ml.MSG_ACK: acks.append((time.time(), p[0], p[1]))
    _h0(mid, p)
app.link._handle = _h

def T(): return app.telem or {}
def F(): return int(T().get("flags", 0))
def un(v): return ((v + 180) % 360) - 180
def ack_ok(mid, since): return any(i == mid and s == 0 and t >= since for t, i, s in acks)
S = {}
def rel_pan(): return un(T().get("cmd_pan", 0) - S.get("p0", 0))

def seq(*fs):
    def f():
        for x in fs: x()
    return f
def set_and(var, val, fn): return lambda: (var.set(val), fn())
def mark(k, v): return lambda: S.__setitem__(k, v() if callable(v) else v)

LOGF = os.path.join(os.path.dirname(os.path.abspath(__file__)), "lora_gui_kayit.csv")
steps = [
    ("Baglanti + ilk telemetri", lambda: None, lambda: T() and app.imu_vals["roll"].cget("text") != "—", 10),
    ("Ayarlar > Karttan oku", app._params_read, lambda: "okundu" in app.lbl_params.cget("text"), 20),
    ("Lazer ac (kutucuk)", set_and(app.var_laser, True, app._laser_click), lambda: F() & 0x02, 6),
    ("Lazer kapat (kutucuk)", set_and(app.var_laser, False, app._laser_click), lambda: not F() & 0x02, 6),
    ("Uc surucu ac (kutucuk)", set_and(app.var_en_all, True, app._enable_all), lambda: (F() >> 5) & 7 == 7, 6),
    ("CE surucusu kapat (eksen kutucugu)", set_and(app.en_vars["ce"], False, lambda: app._enable_axis("ce")), lambda: not (F() >> 7) & 1, 6),
    ("CE surucusu ac (eksen kutucugu)", set_and(app.en_vars["ce"], True, lambda: app._enable_axis("ce")), lambda: (F() >> 7) & 1, 6),
    ("Adim at: pan +1", seq(mark("p0", lambda: T()["cmd_pan"]), lambda: app._step("pan", 1.0)), lambda: abs(rel_pan() - 1.0) < 0.05, 8),
    ("Adim at: pan -1", lambda: app._step("pan", -1.0), lambda: abs(rel_pan()) < 0.05, 8),
    ("Adim at: tilt +1", seq(mark("t0", lambda: T()["cmd_tilt"]), lambda: app._step("tilt", 1.0)), lambda: abs(T()["cmd_tilt"] - S["t0"] - 1.0) < 0.05, 8),
    ("Adim at: tilt -1", lambda: app._step("tilt", -1.0), lambda: abs(T()["cmd_tilt"] - S["t0"]) < 0.05, 8),
    ("Aciya git: pan +3", lambda: (app.goto_val["pan"].set("%.2f" % ((S["p0"] + 3.0) % 360)), app._goto("pan")), lambda: abs(rel_pan() - 3.0) < 0.05, 8),
    ("Aciya git: pan geri", lambda: (app.goto_val["pan"].set("%.2f" % S["p0"]), app._goto("pan")), lambda: abs(rel_pan()) < 0.05, 8),
    ("Basili tutunca doner: pan 2 d/s, 2 s", seq(lambda: app.jog_speed["pan"].set("2"), lambda: app._jog("pan", 1),
                                               lambda: app.after(2000, lambda: app._jog("pan", 0))),
     lambda: rel_pan() > 1.0 and abs(T().get("vel_pan", 1)) < 0.05, 12),
    ("Salinim baslat (pan +-2)", lambda: (app.osc_a["pan"].set("%.2f" % ((S["p0"] - 2) % 360)), app.osc_b["pan"].set("%.2f" % ((S["p0"] + 2) % 360)),
                                          app.osc_dwell["pan"].set("300"), app._osc("pan", True)), lambda: abs(T().get("vel_pan", 0)) > 1.0, 8),
    ("Salinim durdur", lambda: app._osc("pan", False), lambda: abs(T().get("vel_pan", 1)) < 0.05, 10),
    ("Pan'i baslangica goturme (Git)", lambda: (app.goto_val["pan"].set("%.2f" % S["p0"]), app._goto("pan")), lambda: abs(rel_pan()) < 0.05, 10),
    ("Tilt acisini IMU'dan ayarla", seq(mark("ts", time.time), lambda: app._send(ml.MSG_TILT_FROM_IMU)), lambda: ack_ok(ml.MSG_TILT_FROM_IMU, S["ts"]), 6),
    ("Kabul et (CE = 0)", seq(mark("ts", time.time), lambda: app.zero_val["ce"].set("0"), lambda: app._zero("ce")), lambda: ack_ok(ml.MSG_AXIS_ZERO, S["ts"]), 6),
    ("(hazirlik) kilit varsa birak", lambda: app._send(ml.MSG_UNLOCK) if F() & 0x01 else None, lambda: not F() & 0x01, 8),
    ("Hedefe kilitle", app._lock_click, lambda: F() & 0x01, 8),
    ("Otomatik takip", app._auto_click, lambda: T().get("mode") == 2, 6),
    ("Otomatikte nisan hatasi ekranda", mark("ts", time.time), lambda: time.time() - S["ts"] > 3 and app.lbl_aim.cget("text") not in ("—", ""), 8),
    ("Manuel mod", app._manual_click, lambda: T().get("mode") == 1, 6),
    ("Kilidi birak (Hedefe kilitle'ye tekrar)", app._lock_click, lambda: not F() & 0x01, 6),
    ("DUR dugmesi", seq(mark("ts", time.time), lambda: app._send(ml.MSG_STOP)), lambda: ack_ok(ml.MSG_STOP, S["ts"]), 12),
    ("Enkoder durumu (5 s'de bir sorgu)", lambda: None, lambda: all(a in app.enc_diag for a in ("pan", "tilt", "ce")), 14),
    ("IMU durumu ('veri geliyor')", lambda: None, lambda: "veri geliyor" in app.lbl_imu_state.cget("text"), 6),
    ("Veri kaydi baslat", lambda: (app.log_var.set(LOGF), app._toggle_log()), lambda: app.link.log is not None, 4),
    ("Veri kaydi durdur (5 s sonra)", lambda: app.after(5000, app._toggle_log), lambda: app.link.log is None and os.path.exists(LOGF), 9),
    ("Bilgisayardan yukle ve karta yaz", app._params_load_pc, lambda: "yazildi" in app.lbl_params.cget("text"), 12),
    ("IMU'yu yeniden baslat", seq(mark("ts", time.time), lambda: app._send(ml.MSG_IMU_INIT)), lambda: ack_ok(ml.MSG_IMU_INIT, S["ts"]) and (F() >> 2) & 1, 14),
    ("ACIL DUR (suruculer kapanir)", app._estop, lambda: (F() >> 4) & 1 == 0, 6),
]

# bir önceki adım olmadıysa anlamsız olan (geri dönüş) adımlar atlanır
BAGIMLI = {"Adim at: pan -1", "Adim at: tilt -1", "Aciya git: pan geri", "Otomatik takip", "Otomatikte nisan hatasi ekranda",
           "Kilidi birak (Hedefe kilitle'ye tekrar)", "Salinim durdur"}
watch = {"IMU roll": lambda: app.imu_vals["roll"].cget("text"), "IMU yaw": lambda: app.imu_vals["yaw"].cget("text"),
         "enkoder pan": lambda: app.enc_cells["pan"][0].cget("text"), "enkoder tilt": lambda: app.enc_cells["tilt"][0].cget("text"),
         "pan istenen": lambda: app.axis_vals["pan"]["cmd"].cget("text"), "kart saati": lambda: app.pill_conn.cget("text")}
chg = {k: 0 for k in watch}; last = {k: None for k in watch}
res = []; st = {"i": -1, "t": 0.0, "t0": time.time(), "tc0": None}

def start(i):
    st["t"] = time.time()
    try: steps[i][1]()
    except Exception as e: print("adim hatasi:", steps[i][0], e)

def finish():
    dur = time.time() - st["t0"]
    print("\nARAYUZ LoRa TESTI (%s), %.0f s" % (PORT, dur))
    for n, ok, t in res: print("  %-44s %s  (%.1f s)" % (n, "OK    " if ok else ("ATLANDI" if ok is None else "OLMADI"), t))
    print("  gecen: %d / %d" % (sum(1 for _, ok, _ in res if ok), len(res)))
    print("  tekrar gonderilen komut: %d" % getattr(app, "_retries", 0))
    tc = app.link.telem_count - (st["tc0"] or 0)
    print("\nEKRANDAKI VERILER (%.0f s boyunca degisme sayisi):" % dur)
    for k in watch: print("  %-14s %4d kez" % (k, chg[k]))
    print("  telemetri: %d cerceve -> saniyede %.2f" % (tc, tc / dur))
    print("  enkoder durumu:", {a: app.enc_diag.get(a) for a in ("pan", "tilt", "ce")})
    print("  IMU:", app.lbl_imu_state.cget("text"), "| isiklar:", {k: app.health[k].cget("fg") for k in app.health})
    app._close()

def run():
    for k, f in watch.items():
        try: v = f()
        except Exception: v = None
        if last[k] is not None and v != last[k]: chg[k] += 1
        last[k] = v
    if st["tc0"] is None and app.link: st["tc0"] = app.link.telem_count
    i = st["i"]
    if i < 0:
        st["i"] = 0; start(0)
    else:
        name, act, chk, to = steps[i]
        try: ok = bool(chk())
        except Exception: ok = False
        if ok or time.time() - st["t"] > to:
            res.append((name, ok, time.time() - st["t"]))
            print("  %-44s %s" % (name, "OK" if ok else "OLMADI"), flush=True)
            st["i"] = i + 1
            while st["i"] < len(steps) and steps[st["i"]][0] in BAGIMLI and res and res[-1][1] is False:
                res.append((steps[st["i"]][0], None, 0.0)); print("  %-44s ATLANDI (onceki adim olmadi)" % steps[st["i"]][0]); st["i"] += 1
            if st["i"] >= len(steps): finish(); return
            start(st["i"])
    app.after(100, run)

app.after(1500, run)
app.mainloop()
