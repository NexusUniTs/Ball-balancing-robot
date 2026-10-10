import time
import math
import random
from collections import deque
import threading
import tkinter as tk
from tkinter import messagebox

MODO = "sim"

try:
    import board
    import busio
    from adafruit_pca9685 import PCA9685
    from adafruit_motor import servo
    HW_SERVO = True
except ImportError:
    HW_SERVO = False

try:
    from evdev import InputDevice, ecodes
    HW_TOUCH = True
except ImportError:
    HW_TOUCH = False

TOUCH_DEVICE = '/dev/input/event4'

TOUCH_DIM_X_MM = 140.0
TOUCH_DIM_Y_MM = 180.0

TOUCH_CENTER_RAW_X = 1980
TOUCH_CENTER_RAW_Y = 1850
TOUCH_MAX_RAW_X = 3960
TOUCH_MAX_RAW_Y = 3700

TOUCH_FILTER_ALPHA = 0.5

KIN_H = 100.0
KIN_E = 50.0
KIN_B = 50.0
KIN_F = 30.0
KIN_G = 80.0

SERVO_AZIMUTH_DEG = (0.0, 120.0, 240.0)

SERVO_MIN_DEG = 0.0
SERVO_MAX_DEG = 180.0

PID_X_KP = 0.10
PID_X_KI = 0.005
PID_X_KD = 0.05

PID_Y_KP = 0.10
PID_Y_KI = 0.005
PID_Y_KD = 0.05

PID_OUTPUT_LIMIT_DEG = 12.0
PID_D_FILTER_ALPHA = 0.3

LOOP_HZ = 50
GUI_HZ = 30
MAX_PASSO_SERVO_DEG = 5.0
STORICO_SECONDI = 15

SFONDO = "#0b1020"
PANNELLO = "#121a33"
TESTO = "#d6e2ff"
TENUE = "#6b7aa6"
CIANO = "#22d3ee"
VIOLA = "#a78bfa"
AMBRA = "#fbbf24"
ROSSO = "#f43f5e"
VERDE = "#34d399"
GRIGLIA = "#1e2a4d"
BRACCIO = "#c4b5fd"
PALLINA = "#fb923c"
PERCORSO = "#4c3d86"

MODI = {"sim": "Simulazione", "sim_servo": "Sim + servo", "reale": "Reale"}

PERCORSI = {"centro": "Centro", "cerchio": "Cerchio", "quadrato": "Quadrato",
            "infinito": "Infinito", "fiore": "Fiore"}
PERIODI = {"centro": 1.0, "cerchio": 6.0, "quadrato": 8.0, "infinito": 8.0, "fiore": 15.0}

class TouchScreenEvdev:

    def __init__(self, device_path):
        self.dev = InputDevice(device_path)
        self.dev.grab()

        self.centro_x_raw = TOUCH_CENTER_RAW_X
        self.centro_y_raw = TOUCH_CENTER_RAW_Y

        self.scala_x = TOUCH_DIM_X_MM / TOUCH_MAX_RAW_X
        self.scala_y = TOUCH_DIM_Y_MM / TOUCH_MAX_RAW_Y

        self.last_x_raw = self.centro_x_raw
        self.last_y_raw = self.centro_y_raw

        tasti = self.dev.capabilities().get(ecodes.EV_KEY, [])
        self.ha_btn_touch = ecodes.BTN_TOUCH in tasti
        self.toccato = not self.ha_btn_touch

    def leggi_posizione(self):
        try:
            for event in self.dev.read():
                if event.type == ecodes.EV_ABS:
                    if event.code == ecodes.ABS_X:
                        self.last_x_raw = event.value
                    elif event.code == ecodes.ABS_Y:
                        self.last_y_raw = event.value
                elif event.type == ecodes.EV_KEY and event.code == ecodes.BTN_TOUCH:
                    self.toccato = event.value != 0
        except BlockingIOError:
            pass

        x_mm = (self.last_x_raw - self.centro_x_raw) * self.scala_x
        y_mm = (self.last_y_raw - self.centro_y_raw) * self.scala_y

        return y_mm, -x_mm, self.toccato

    def rilascia(self):
        self.dev.ungrab()

class ServoFinti:

    def __init__(self):
        self.angoli = [ANGOLO_HOME] * 3

    def muovi(self, angoli):
        self.angoli = list(angoli)

    def vai_a(self, angoli):
        self.muovi(angoli)

    def spegni(self):
        pass

class ServoVeri:

    def __init__(self):
        print("[INFO] Inizializzazione bus I2C e controller PCA9685...")
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50
        self.motori = [servo.Servo(self.pca.channels[ch], min_pulse=500, max_pulse=2500) for ch in (0, 1, 2)]
        self.angoli = [ANGOLO_HOME] * 3

    def muovi(self, angoli):
        nuovi = []
        for m, a, prima in zip(self.motori, angoli, self.angoli):
            a = max(prima - MAX_PASSO_SERVO_DEG, min(prima + MAX_PASSO_SERVO_DEG, a))
            m.angle = a
            nuovi.append(a)
        self.angoli = nuovi

    def vai_a(self, angoli):
        for _ in range(100):
            self.muovi(angoli)
            if all(abs(a - b) < 0.1 for a, b in zip(self.angoli, angoli)):
                break
            time.sleep(1.0 / LOOP_HZ)

    def spegni(self):
        self.vai_a([ANGOLO_HOME] * 3)
        time.sleep(0.3)
        self.pca.deinit()

class PallinaSimulata:

    def __init__(self):
        self.ha_btn_touch = True
        self.reset()

    def reset(self):
        self.x = random.uniform(-50, 50)
        self.y = random.uniform(-40, 40)
        self.vx = self.vy = 0.0
        self.pitch = self.roll = 0.0
        self.sul_piatto = True
        self.tempo_fuori = 0.0

    def spingi(self):
        angolo = random.uniform(0, 2 * math.pi)
        self.vx += 200 * math.cos(angolo)
        self.vy += 200 * math.sin(angolo)

    def aggiorna(self, pitch_deg, roll_deg, dt):
        if not self.sul_piatto:
            self.tempo_fuori += dt
            if self.tempo_fuori > 1.5:
                self.reset()
            return

        passi = 10
        h = dt / passi
        for _ in range(passi):
            k = min(1.0, h / 0.05)
            self.pitch += (pitch_deg - self.pitch) * k
            self.roll += (roll_deg - self.roll) * k

            g = 5.0 / 7.0 * 9810.0
            ax = g * math.sin(math.radians(self.pitch)) - 0.25 * self.vx
            ay = g * math.sin(math.radians(self.roll)) - 0.25 * self.vy
            self.vx += ax * h
            self.vy += ay * h
            self.x += self.vx * h
            self.y += self.vy * h

        if abs(self.x) > TOUCH_DIM_Y_MM / 2 or abs(self.y) > TOUCH_DIM_X_MM / 2:
            self.sul_piatto = False
            self.tempo_fuori = 0.0

    def leggi_posizione(self):
        rumore_x = random.gauss(0, 0.8)
        rumore_y = random.gauss(0, 0.8)
        return self.x + rumore_x, self.y + rumore_y, self.sul_piatto

    def rilascia(self):
        pass

class PID:

    def __init__(self, kp, ki, kd, limite_output):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limite_output = limite_output
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_value = None
        self.derivative = 0.0
        self.prev_time = time.monotonic()

    def update(self, setpoint, current_value, vel_setpoint=0.0):
        current_time = time.monotonic()
        dt = current_time - self.prev_time
        if dt <= 0.0:
            dt = 1e-4

        error = setpoint - current_value

        self.integral += error * dt
        self.integral = max(-50.0, min(50.0, self.integral))

        if self.prev_value is not None:
            raw_d = vel_setpoint - (current_value - self.prev_value) / dt
            self.derivative += PID_D_FILTER_ALPHA * (raw_d - self.derivative)

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * self.derivative)

        self.prev_value = current_value
        self.prev_time = current_time

        return max(-self.limite_output, min(self.limite_output, output))

def _matrice_rotazione(pitch_rad, roll_rad):
    cp, sp = math.cos(-pitch_rad), math.sin(-pitch_rad)
    cr, sr = math.cos(roll_rad), math.sin(roll_rad)
    return ((cp, sp * sr, sp * cr),
            (0.0, cr, -sr),
            (-sp, cp * sr, cp * cr))

def _soluzioni_squadretta(pitch_deg, roll_deg, indice):
    R = _matrice_rotazione(math.radians(pitch_deg), math.radians(roll_deg))

    b = math.radians(SERVO_AZIMUTH_DEG[indice])
    ux, uy = math.cos(b), math.sin(b)

    px_l, py_l = KIN_E * ux, KIN_E * uy
    P = (R[0][0] * px_l + R[0][1] * py_l,
         R[1][0] * px_l + R[1][1] * py_l,
         R[2][0] * px_l + R[2][1] * py_l + KIN_H)

    d = (P[0] - KIN_B * ux, P[1] - KIN_B * uy, P[2])

    L = d[0] ** 2 + d[1] ** 2 + d[2] ** 2 + KIN_F ** 2 - KIN_G ** 2
    M = 2 * KIN_F * d[2]
    N = 2 * KIN_F * (d[0] * ux + d[1] * uy)
    r = math.hypot(M, N)
    if r < 1e-9 or abs(L / r) > 1.0:
        return None

    phi = math.atan2(N, M)
    k = math.asin(L / r)
    return (k - phi, math.pi - k - phi)

def _vicino(a, rif):
    return abs(math.atan2(math.sin(a - rif), math.cos(a - rif)))

_sol_home = _soluzioni_squadretta(0.0, 0.0, 0)
if _sol_home is None:
    raise SystemExit("[ERRORE] Geometria KIN_* impossibile: il piatto in piano non è raggiungibile.")
ALPHA_HOME = max(_sol_home, key=math.cos)
ANGOLO_HOME = 180.0 - math.degrees(ALPHA_HOME)

def calcola_angoli_servo(pitch_deg, roll_deg, angoli_precedenti):
    angoli = []
    for i in range(3):
        sol = _soluzioni_squadretta(pitch_deg, roll_deg, i)
        if sol is None:
            angoli.append(angoli_precedenti[i])
            continue
        a = min(sol, key=lambda x: _vicino(x, ALPHA_HOME))
        angolo = 180.0 - math.degrees(math.atan2(math.sin(a), math.cos(a)))
        angoli.append(max(SERVO_MIN_DEG, min(SERVO_MAX_DEG, angolo)))
    return angoli

def punto_sul_piatto(R, x, y, z=0.0):
    return (R[0][0] * x + R[0][1] * y + R[0][2] * z,
            R[1][0] * x + R[1][1] * y + R[1][2] * z,
            R[2][0] * x + R[2][1] * y + R[2][2] * z + KIN_H)

def punti_gamba(R, indice, angolo_servo):
    b = math.radians(SERVO_AZIMUTH_DEG[indice])
    ux, uy = math.cos(b), math.sin(b)
    S = (KIN_B * ux, KIN_B * uy, 0.0)
    a = math.radians(180.0 - angolo_servo)
    K = (S[0] + KIN_F * math.cos(a) * ux, S[1] + KIN_F * math.cos(a) * uy, KIN_F * math.sin(a))
    P = punto_sul_piatto(R, KIN_E * ux, KIN_E * uy)
    return S, K, P

def punto_percorso(nome, t):
    if nome == "cerchio":
        w = 2 * math.pi / PERIODI["cerchio"]
        return 40 * math.cos(w * t), 40 * math.sin(w * t)
    if nome == "quadrato":
        lato = 80.0
        s = (t % PERIODI["quadrato"]) / PERIODI["quadrato"] * 4 * lato
        m = lato / 2
        if s < lato:
            return -m + s, -m
        if s < 2 * lato:
            return m, -m + (s - lato)
        if s < 3 * lato:
            return m - (s - 2 * lato), m
        return -m, m - (s - 3 * lato)
    if nome == "infinito":
        w = 2 * math.pi / PERIODI["infinito"]
        return 50 * math.sin(w * t), 30 * math.sin(2 * w * t)
    if nome == "fiore":
        th = math.pi * t / PERIODI["fiore"]
        r = 45 * math.cos(5 * th)
        return r * math.cos(th), r * math.sin(th)
    return 0.0, 0.0

class Controllore:

    def __init__(self):
        self.pid_pitch = PID(PID_X_KP, PID_X_KI, PID_X_KD, PID_OUTPUT_LIMIT_DEG)
        self.pid_roll = PID(PID_Y_KP, PID_Y_KI, PID_Y_KD, PID_OUTPUT_LIMIT_DEG)
        self.setpoint_x, self.setpoint_y = 0.0, 0.0
        self.filt_x = self.filt_y = None
        self.pitch = self.roll = 0.0
        self.angoli = [ANGOLO_HOME] * 3
        self.t0 = time.monotonic()
        self.storico = deque(maxlen=STORICO_SECONDI * LOOP_HZ)
        self.percorso = "centro"
        self.t_percorso = time.monotonic()

    def imposta_percorso(self, nome):
        self.percorso = nome
        self.t_percorso = time.monotonic()

    def obiettivo(self, t):
        x, y = punto_percorso(self.percorso, t)
        dolce = min(1.0, t / 2.0)
        return x * dolce, y * dolce

    def step(self, x, y, toccato):
        t = time.monotonic() - self.t_percorso
        self.setpoint_x, self.setpoint_y = self.obiettivo(t)
        avanti_x, avanti_y = self.obiettivo(t + 0.01)
        vel_x = (avanti_x - self.setpoint_x) / 0.01
        vel_y = (avanti_y - self.setpoint_y) / 0.01

        if toccato:
            if self.filt_x is None:
                self.filt_x, self.filt_y = x, y
            else:
                self.filt_x += TOUCH_FILTER_ALPHA * (x - self.filt_x)
                self.filt_y += TOUCH_FILTER_ALPHA * (y - self.filt_y)
            self.pitch = self.pid_pitch.update(self.setpoint_x, self.filt_x, vel_x)
            self.roll = self.pid_roll.update(self.setpoint_y, self.filt_y, vel_y)
        else:
            self.filt_x = self.filt_y = None
            self.pid_pitch.reset()
            self.pid_roll.reset()
            self.pitch = self.roll = 0.0

        self.angoli = calcola_angoli_servo(self.pitch, self.roll, self.angoli)

        self.storico.append({
            "t": time.monotonic() - self.t0,
            "x": x if toccato else None,
            "y": y if toccato else None,
            "filt_x": self.filt_x,
            "filt_y": self.filt_y,
            "target_x": self.setpoint_x,
            "target_y": self.setpoint_y,
            "pitch": self.pitch,
            "roll": self.roll,
            "angoli": list(self.angoli),
        })
        return self.angoli

def modo_possibile(modo):
    if modo in ("sim_servo", "reale") and not HW_SERVO:
        return False
    if modo == "reale" and not HW_TOUCH:
        return False
    return True

def crea_io(modo):
    if modo in ("sim_servo", "reale"):
        uscita = ServoVeri()
    else:
        uscita = ServoFinti()
    if modo == "reale":
        touch = TouchScreenEvdev(TOUCH_DEVICE)
    else:
        touch = PallinaSimulata()
    return uscita, touch

class Bottone(tk.Label):

    def __init__(self, genitore, testo, comando, colore=CIANO):
        super().__init__(genitore, text=testo, bg=PANNELLO, fg=colore, padx=12, pady=6,
                         font=("Helvetica", 13, "bold"), cursor="hand2")
        self.colore = colore
        self.bind("<Button-1>", lambda evento: comando())

    def evidenzia(self, attivo):
        if attivo:
            self.config(bg=self.colore, fg=SFONDO)
        else:
            self.config(bg=PANNELLO, fg=self.colore)

class App:

    def __init__(self, root):
        self.root = root
        root.title("Nexus MK2 monitor")
        root.configure(bg=SFONDO)
        larghezza, altezza = root.winfo_screenwidth(), root.winfo_screenheight()
        root.geometry(f"{larghezza}x{altezza}+0+0")

        if modo_possibile(MODO):
            self.modo = MODO
        else:
            print(f"[WARN] MODO = {MODO} non disponibile su questo computer: uso la simulazione.")
            self.modo = "sim"
        self.uscita, self.touch = crea_io(self.modo)
        self.uscita.vai_a([ANGOLO_HOME] * 3)
        self.ctrl = Controllore()
        self.percorso = "centro"
        self.in_corsa = False
        self.errore = None
        self.attivo = True
        self.blocco = threading.RLock()
        self.foto = []
        self.ist = self.istantanea()
        self.vista_az = math.radians(30)
        self.vista_el = math.radians(25)
        self.trascina_da = None

        self.costruisci()
        root.protocol("WM_DELETE_WINDOW", self.chiudi)

        self.thread = threading.Thread(target=self.controllo, daemon=True)
        self.thread.start()
        self.disegna()

    def costruisci(self):
        self.root.grid_columnconfigure(0, weight=1, uniform="meta")
        self.root.grid_columnconfigure(1, weight=1, uniform="meta")
        self.root.grid_rowconfigure(0, weight=1)

        sinistra = tk.Frame(self.root, bg=SFONDO)
        sinistra.grid(row=0, column=0, sticky="nsew", padx=(16, 8), pady=16)
        self.disegno = tk.Canvas(self.root, bg=PANNELLO, highlightthickness=0)
        self.disegno.grid(row=0, column=1, sticky="nsew", padx=(8, 16), pady=16)
        self.disegno.bind("<ButtonPress-1>", self.inizia_trascina)
        self.disegno.bind("<B1-Motion>", self.trascina)

        testata = tk.Frame(sinistra, bg=SFONDO)
        testata.pack(fill="x")
        tk.Label(testata, text="NEXUS MK2", font=("Helvetica", 22, "bold"),
                 fg=CIANO, bg=SFONDO).pack(side="left")
        self.stato = tk.Label(testata, font=("Helvetica", 14, "bold"), bg=SFONDO)
        self.stato.pack(side="right")

        comandi = tk.Frame(sinistra, bg=SFONDO)
        comandi.pack(fill="x", pady=(12, 6))
        self.b_avvia = Bottone(comandi, "Avvia", self.avvia_pausa)
        self.b_avvia.pack(side="left", padx=(0, 8))
        Bottone(comandi, "Reset", self.reset).pack(side="left", padx=(0, 8))
        Bottone(comandi, "Spingi pallina", self.spingi, AMBRA).pack(side="left")

        riga_modi = tk.Frame(sinistra, bg=SFONDO)
        riga_modi.pack(fill="x", pady=(0, 12))
        tk.Label(riga_modi, text="Modo:", fg=TENUE, bg=SFONDO,
                 font=("Helvetica", 13)).pack(side="left", padx=(0, 8))
        self.b_modi = {}
        for chiave, nome in MODI.items():
            colore = VIOLA if modo_possibile(chiave) else TENUE
            b = Bottone(riga_modi, nome, lambda k=chiave: self.imposta_modo(k), colore)
            b.pack(side="left", padx=(0, 8))
            self.b_modi[chiave] = b
        self.aggiorna_bottoni_modo()

        riga_percorsi = tk.Frame(sinistra, bg=SFONDO)
        riga_percorsi.pack(fill="x", pady=(0, 12))
        tk.Label(riga_percorsi, text="Percorso:", fg=TENUE, bg=SFONDO,
                 font=("Helvetica", 13)).pack(side="left", padx=(0, 8))
        self.b_percorsi = {}
        for chiave, nome in PERCORSI.items():
            b = Bottone(riga_percorsi, nome, lambda k=chiave: self.imposta_percorso(k), VERDE)
            b.pack(side="left", padx=(0, 8))
            self.b_percorsi[chiave] = b
        self.aggiorna_bottoni_percorso()

        self.pannello_numeri(sinistra)

        self.area_dati = tk.Frame(sinistra, bg=SFONDO)
        self.area_dati.pack(fill="both", expand=True, pady=(12, 0))
        self.area_dati.grid_columnconfigure(0, weight=1)
        self.area_dati.grid_rowconfigure(0, weight=4)
        self.area_dati.grid_rowconfigure(1, weight=3)
        self.area_dati.grid_rowconfigure(2, weight=3)

        self.tela_touch = tk.Canvas(self.area_dati, bg=PANNELLO, highlightthickness=0, height=1)
        self.tela_touch.grid(row=0, column=0, sticky="nsew", pady=(0, 8))
        self.tela_posizione = tk.Canvas(self.area_dati, bg=PANNELLO, highlightthickness=0, height=1)
        self.tela_posizione.grid(row=1, column=0, sticky="nsew", pady=(0, 8))
        self.tela_servo = tk.Canvas(self.area_dati, bg=PANNELLO, highlightthickness=0, height=1)
        self.tela_servo.grid(row=2, column=0, sticky="nsew")

    def controllo(self):
        periodo = 1.0 / LOOP_HZ
        prossimo = time.monotonic()
        ultimo = prossimo
        while self.attivo:
            ora = time.monotonic()
            dt = min(ora - ultimo, 0.1)
            ultimo = ora

            with self.blocco:
                if self.in_corsa:
                    try:
                        x, y, toccato = self.touch.leggi_posizione()
                        angoli = self.ctrl.step(x, y, toccato)
                        if isinstance(self.touch, PallinaSimulata):
                            self.touch.aggiorna(self.ctrl.pitch, self.ctrl.roll, dt)
                        self.uscita.muovi(angoli)
                    except Exception as e:
                        self.errore = str(e)
                        self.in_corsa = False

            prossimo += periodo
            attesa = prossimo - time.monotonic()
            if attesa > 0:
                time.sleep(attesa)
            else:
                prossimo = time.monotonic()

    def istantanea(self):
        return {
            "pitch": self.ctrl.pitch,
            "roll": self.ctrl.roll,
            "filt_x": self.ctrl.filt_x,
            "filt_y": self.ctrl.filt_y,
            "angoli": list(self.uscita.angoli),
        }

    def disegna(self):
        with self.blocco:
            self.foto = list(self.ctrl.storico)
            self.ist = self.istantanea()

        self.b_avvia.config(text="Pausa" if self.in_corsa else "Avvia")
        self.aggiorna_stato()
        self.aggiorna_numeri()
        self.disegna_touch()
        self.disegna_grafici()
        self.disegna_3d()
        self.root.after(int(1000 / GUI_HZ), self.disegna)

    def pannello_numeri(self, genitore):
        riquadro = tk.Frame(genitore, bg=PANNELLO)
        riquadro.pack(fill="x")
        self.valori = {}
        voci = [("pitch", "Pitch"), ("roll", "Roll"), ("x", "X mm"), ("y", "Y mm"),
                ("A", "Servo A"), ("B", "Servo B"), ("C", "Servo C")]
        for colonna, (chiave, nome) in enumerate(voci):
            riquadro.grid_columnconfigure(colonna, weight=1)
            tk.Label(riquadro, text=nome, fg=TENUE, bg=PANNELLO,
                     font=("Helvetica", 11)).grid(row=0, column=colonna, sticky="w", padx=10, pady=(8, 0))
            valore = tk.Label(riquadro, text="--", fg=TESTO, bg=PANNELLO, font=("Menlo", 18, "bold"), width=7, anchor="w")
            valore.grid(row=1, column=colonna, sticky="w", padx=10, pady=(0, 8))
            self.valori[chiave] = valore

    def aggiorna_numeri(self):
        d = self.ist
        self.valori["pitch"].config(text=f"{d['pitch']:+5.1f}°")
        self.valori["roll"].config(text=f"{d['roll']:+5.1f}°")
        if d["filt_x"] is None:
            self.valori["x"].config(text="--")
            self.valori["y"].config(text="--")
        else:
            self.valori["x"].config(text=f"{d['filt_x']:+6.1f}")
            self.valori["y"].config(text=f"{d['filt_y']:+6.1f}")
        for i, nome in enumerate("ABC"):
            self.valori[nome].config(text=f"{d['angoli'][i]:5.1f}°")

    def aggiorna_stato(self):
        if self.errore:
            testo, colore = f"Errore: {self.errore}", ROSSO
        elif not self.in_corsa:
            testo, colore = "In pausa", AMBRA
        elif self.ist["filt_x"] is None:
            testo, colore = "Nessuna pallina", ROSSO
        else:
            testo, colore = "Bilancia", VERDE
        self.stato.config(text=f"● {testo}   {MODI[self.modo]} · {PERCORSI[self.percorso]}", fg=colore)

    def disegna_touch(self):
        c = self.tela_touch
        c.delete("all")
        larg, alt = c.winfo_width(), c.winfo_height()
        c.create_text(12, 8, anchor="nw", text="Touch, vista dall'alto [mm]", fill=TESTO,
                      font=("Helvetica", 12))

        meta_x, meta_y = TOUCH_DIM_Y_MM / 2, TOUCH_DIM_X_MM / 2
        scala = min((larg - 40) / (2 * meta_x), (alt - 50) / (2 * meta_y))
        if scala <= 0:
            return
        cx, cy = larg / 2, 30 + (alt - 40) / 2

        def punto(x, y):
            return cx + x * scala, cy - y * scala

        for gx in range(-80, 81, 20):
            c.create_line(*punto(gx, -meta_y), *punto(gx, meta_y), fill=GRIGLIA)
        for gy in range(-60, 61, 20):
            c.create_line(*punto(-meta_x, gy), *punto(meta_x, gy), fill=GRIGLIA)
        c.create_rectangle(*punto(-meta_x, meta_y), *punto(meta_x, -meta_y), outline=CIANO, width=2)
        c.create_text(*punto(meta_x - 4, 4), anchor="se", text="+x", fill=TENUE)
        c.create_text(*punto(4, meta_y - 4), anchor="nw", text="+y", fill=TENUE)

        if self.percorso != "centro":
            linea = []
            for k in range(201):
                linea += punto(*punto_percorso(self.percorso, PERIODI[self.percorso] * k / 200))
            c.create_line(*linea, fill=PERCORSO, width=2)

        foto = self.foto[-75:]
        if foto:
            tx, ty = punto(foto[-1]["target_x"], foto[-1]["target_y"])
            c.create_oval(tx - 7, ty - 7, tx + 7, ty + 7, outline=VIOLA, width=2)

        traccia = []
        for f in foto:
            if f["filt_x"] is None:
                traccia = []
            else:
                traccia += punto(f["filt_x"], f["filt_y"])
        if len(traccia) >= 4:
            c.create_line(*traccia, fill=CIANO, width=2)

        if foto and foto[-1]["x"] is not None:
            rx, ry = punto(foto[-1]["x"], foto[-1]["y"])
            c.create_oval(rx - 3, ry - 3, rx + 3, ry + 3, fill=TENUE, outline="")
            fx, fy = punto(foto[-1]["filt_x"], foto[-1]["filt_y"])
            r = 20 * scala
            c.create_oval(fx - r, fy - r, fx + r, fy + r, outline=CIANO, width=2)
        else:
            c.create_text(cx, cy, text="Nessuna pallina", fill=TENUE, font=("Helvetica", 13))

    def disegna_grafico(self, c, titolo, serie, y_min, y_max, passo):
        c.delete("all")
        larg, alt = c.winfo_width(), c.winfo_height()
        sx, dx, su, giu = 50, larg - 15, 42, alt - 12
        if dx - sx < 50 or giu - su < 30:
            return
        c.create_text(12, 8, anchor="nw", text=titolo, fill=TESTO, font=("Helvetica", 12))

        foto = self.foto
        t_fine = foto[-1]["t"] if foto else 0.0
        t_inizio = t_fine - STORICO_SECONDI

        def px(t):
            return sx + (t - t_inizio) / STORICO_SECONDI * (dx - sx)

        def py(v):
            v = max(y_min, min(y_max, v))
            return giu - (v - y_min) / (y_max - y_min) * (giu - su)

        v = y_min
        while v <= y_max:
            c.create_line(sx, py(v), dx, py(v), fill=GRIGLIA)
            c.create_text(sx - 6, py(v), anchor="e", text=f"{v:g}", fill=TENUE, font=("Helvetica", 9))
            v += passo

        x_legenda = dx
        for nome, colore, _ in reversed(serie):
            c.create_text(x_legenda, 14, anchor="e", text=nome, fill=colore, font=("Helvetica", 11, "bold"))
            x_legenda -= 12 + 9 * len(nome)

        for nome, colore, valore in serie:
            punti = []
            for f in foto:
                val = valore(f)
                if val is None:
                    if len(punti) >= 4:
                        c.create_line(*punti, fill=colore, width=2)
                    punti = []
                else:
                    punti += [px(f["t"]), py(val)]
            if len(punti) >= 4:
                c.create_line(*punti, fill=colore, width=2)

    def disegna_grafici(self):
        self.disegna_grafico(
            self.tela_posizione, "Posizione pallina [mm]",
            [("x rif", "#0e7490", lambda f: f["target_x"]),
             ("y rif", "#5b4a9e", lambda f: f["target_y"]),
             ("x", CIANO, lambda f: f["filt_x"]),
             ("y", VIOLA, lambda f: f["filt_y"])],
            -80, 80, 40)
        casa = round(ANGOLO_HOME)
        self.disegna_grafico(
            self.tela_servo, "Angoli servo [gradi]",
            [("A", CIANO, lambda f: f["angoli"][0]),
             ("B", VIOLA, lambda f: f["angoli"][1]),
             ("C", AMBRA, lambda f: f["angoli"][2])],
            casa - 30, casa + 30, 15)

    def inizia_trascina(self, evento):
        self.trascina_da = (evento.x, evento.y)

    def trascina(self, evento):
        dx = evento.x - self.trascina_da[0]
        dy = evento.y - self.trascina_da[1]
        self.vista_az -= dx * 0.01
        self.vista_el = max(0.05, min(1.5, self.vista_el + dy * 0.01))
        self.trascina_da = (evento.x, evento.y)

    def proietta(self, p):
        ca, sa = math.cos(self.vista_az), math.sin(self.vista_az)
        ce, se = math.cos(self.vista_el), math.sin(self.vista_el)
        orizzontale = p[0] * ca - p[1] * sa
        profondita = p[0] * sa + p[1] * ca
        verticale = p[2] * ce + profondita * se
        return self.cx + orizzontale * self.scala3d, self.cy - verticale * self.scala3d

    def profondita(self, p):
        ca, sa = math.cos(self.vista_az), math.sin(self.vista_az)
        prof = p[0] * sa + p[1] * ca
        return prof * math.cos(self.vista_el) - p[2] * math.sin(self.vista_el)

    def poligono(self, punti, **stile):
        coordinate = []
        for p in punti:
            coordinate += self.proietta(p)
        self.disegno.create_polygon(*coordinate, **stile)

    def scatola(self, centro, u, t, lu, lt, lz):
        def vertice(su, st, sz):
            return (centro[0] + su * lu * u[0] + st * lt * t[0],
                    centro[1] + su * lu * u[1] + st * lt * t[1],
                    centro[2] + sz * lz)

        facce = [
            ([vertice(-1, -1, 1), vertice(1, -1, 1), vertice(1, 1, 1), vertice(-1, 1, 1)], "#2f5bd8"),
            ([vertice(-1, -1, -1), vertice(1, -1, -1), vertice(1, -1, 1), vertice(-1, -1, 1)], "#1d3a8a"),
            ([vertice(-1, 1, -1), vertice(1, 1, -1), vertice(1, 1, 1), vertice(-1, 1, 1)], "#1d3a8a"),
            ([vertice(-1, -1, -1), vertice(-1, 1, -1), vertice(-1, 1, 1), vertice(-1, -1, 1)], "#16306f"),
            ([vertice(1, -1, -1), vertice(1, 1, -1), vertice(1, 1, 1), vertice(1, -1, 1)], "#16306f"),
        ]
        facce.sort(key=lambda f: -sum(self.profondita(p) for p in f[0]))
        for punti, colore in facce:
            self.poligono(punti, fill=colore, outline="#0b1020")

    def disegna_3d(self):
        c = self.disegno
        c.delete("all")
        larg, alt = c.winfo_width(), c.winfo_height()
        self.scala3d = min(larg, alt) / 300
        self.cx, self.cy = larg / 2, alt / 2 + KIN_H * 0.4 * self.scala3d

        d = self.ist
        R = _matrice_rotazione(math.radians(d["pitch"]), math.radians(d["roll"]))
        angoli = d["angoli"]

        raggio_base = KIN_B + 40
        cerchio = [(raggio_base * math.cos(k * math.pi / 24), raggio_base * math.sin(k * math.pi / 24), -15)
                   for k in range(48)]
        self.poligono(cerchio, fill="#18213f", outline=GRIGLIA, width=2)

        gambe = [punti_gamba(R, i, angoli[i]) for i in range(3)]
        ordine = sorted(range(3), key=lambda i: -self.profondita(gambe[i][0]))
        for i in ordine:
            S, K, P = gambe[i]
            b = math.radians(SERVO_AZIMUTH_DEG[i])
            u = (math.cos(b), math.sin(b))
            t = (-u[1], u[0])
            centro = (S[0] - 12 * u[0] - 14 * t[0], S[1] - 12 * u[1] - 14 * t[1], -5)
            self.scatola(centro, u, t, 20, 10, 10)
            c.create_line(*self.proietta(S), *self.proietta(K), fill=VIOLA, width=7, capstyle="round")
            c.create_line(*self.proietta(K), *self.proietta(P), fill=BRACCIO, width=5, capstyle="round")
            for punto, colore in ((S, TESTO), (K, VIOLA), (P, CIANO)):
                x, y = self.proietta(punto)
                c.create_oval(x - 5, y - 5, x + 5, y + 5, fill=colore, outline=PANNELLO)

        mx, my = TOUCH_DIM_Y_MM / 2, TOUCH_DIM_X_MM / 2
        for gx in range(-80, 81, 20):
            c.create_line(*self.proietta(punto_sul_piatto(R, gx, -my)),
                          *self.proietta(punto_sul_piatto(R, gx, my)), fill=GRIGLIA)
        for gy in range(-60, 61, 20):
            c.create_line(*self.proietta(punto_sul_piatto(R, -mx, gy)),
                          *self.proietta(punto_sul_piatto(R, mx, gy)), fill=GRIGLIA)
        bordo = [punto_sul_piatto(R, x, y) for x, y in ((-mx, -my), (mx, -my), (mx, my), (-mx, my))]
        self.poligono(bordo, fill="", outline=CIANO, width=2)

        if self.percorso != "centro":
            linea = []
            for k in range(121):
                px, py = punto_percorso(self.percorso, PERIODI[self.percorso] * k / 120)
                linea += self.proietta(punto_sul_piatto(R, -px, -py))
            c.create_line(*linea, fill=PERCORSO, width=2)

        if d["filt_x"] is not None:
            centro_pallina = punto_sul_piatto(R, -d["filt_x"], -d["filt_y"], 20)
            x, y = self.proietta(centro_pallina)
            r = 20 * self.scala3d
            c.create_oval(x - r, y - r, x + r, y + r, fill=PALLINA, outline="")
            c.create_oval(x - r * 0.55, y - r * 0.6, x - r * 0.1, y - r * 0.15, fill="#fed7aa", outline="")

        for i in range(3):
            b = math.radians(SERVO_AZIMUTH_DEG[i])
            x, y = self.proietta(((raggio_base + 25) * math.cos(b), (raggio_base + 25) * math.sin(b), -15))
            c.create_text(x, y, text=f"{'ABC'[i]}  {angoli[i]:5.1f}°", fill=TESTO, font=("Menlo", 13, "bold"))

        c.create_text(14, 12, anchor="nw", text="Piattaforma 3D (trascina per ruotare)",
                      fill=TENUE, font=("Helvetica", 12))

    def avvia_pausa(self):
        with self.blocco:
            self.in_corsa = not self.in_corsa
            if self.in_corsa:
                self.errore = None
                self.ctrl.pid_pitch.reset()
                self.ctrl.pid_roll.reset()
                self.ctrl.filt_x = self.ctrl.filt_y = None

    def reset(self):
        with self.blocco:
            self.in_corsa = False
            self.errore = None
            self.ctrl = Controllore()
            self.ctrl.imposta_percorso(self.percorso)
            if isinstance(self.touch, PallinaSimulata):
                self.touch.reset()
            self.uscita.vai_a([ANGOLO_HOME] * 3)

    def spingi(self):
        with self.blocco:
            if isinstance(self.touch, PallinaSimulata):
                self.touch.spingi()

    def imposta_modo(self, nuovo):
        if nuovo == self.modo or not modo_possibile(nuovo):
            return
        if nuovo in ("sim_servo", "reale"):
            ok = messagebox.askyesno("Servo veri",
                                     "I servo si muoveranno davvero.\nIl robot è libero e alimentato?")
            if not ok:
                return
        with self.blocco:
            self.reset()
            self.uscita.spegni()
            self.touch.rilascia()
            self.modo = nuovo
            self.uscita, self.touch = crea_io(nuovo)
            self.uscita.vai_a([ANGOLO_HOME] * 3)
        self.aggiorna_bottoni_modo()

    def imposta_percorso(self, nome):
        with self.blocco:
            self.percorso = nome
            self.ctrl.imposta_percorso(nome)
        self.aggiorna_bottoni_percorso()

    def aggiorna_bottoni_percorso(self):
        for chiave, b in self.b_percorsi.items():
            b.evidenzia(chiave == self.percorso)

    def aggiorna_bottoni_modo(self):
        for chiave, b in self.b_modi.items():
            b.evidenzia(chiave == self.modo)

    def chiudi(self):
        self.attivo = False
        self.thread.join(timeout=1.0)
        with self.blocco:
            self.in_corsa = False
            self.uscita.spegni()
            self.touch.rilascia()
        self.root.destroy()

def main():
    root = tk.Tk()
    App(root)
    root.mainloop()

if __name__ == "__main__":
    main()