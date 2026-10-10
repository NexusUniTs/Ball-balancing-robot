"""
Nexus, Ball balancing robot

Questo codice implementa un controller PID per bilanciare una pallina su un pannello touchscreen resistivo controllato da tre servomotori.

Viene ripetuto questo algoritmo, 50 volte al secondo:
  1. il touch dice dove si trova la pallina (in mm dal centro);
  2. il PID decide di quanto inclinare il piatto (pitch e roll, in gradi);
  3. la cinematica inversa trasforma l'inclinazione nei tre angoli dei servo;
  4. i servo si muovono, il piatto si inclina, la pallina rotola verso il centro.
Per approfondire, le formule della cinematica sono spiegate nel PDF "MK2 Cinematica inversa".
"""

import time
import math
import board                              # pin del Raspberry Pi
import busio                              # bus I2C verso la scheda dei servo
from adafruit_pca9685 import PCA9685      # scheda che genera i segnali PWM dei servo
from adafruit_motor import servo          # converte un angolo in gradi nel segnale PWM
from evdev import InputDevice, ecodes     # lettura del touchscreen come dispositivo di input Linux

# ==============================================================================
# 1. CONFIGURAZIONE HARDWARE E MECCANICA
# ==============================================================================
# Tutti i numeri che descrivono il robot stanno qui: se cambia la meccanica
# si modificano solo questi valori, non il resto del codice.

# -- Configurazione Touchscreen --

# COME TROVARE IL PERCORSO DEL TOUCHSCREEN (DEVICE PATH):
# 1. Apri il terminale del Raspberry Pi.
# 2. Esegui il comando: sudo python3 -m evdev.evtest
# 3. Individua nell'elenco stampato a schermo il nome del tuo controller USB 
#    (es. "VSDISPLAY", "USB Touch Controller", ecc.).
# 4. Annota il percorso ad esso associato (es. "/dev/input/event4") e inseriscilo 
#    nella variabile TOUCH_DEVICE qui sotto.
#
# Nota: se scolleghi e ricolleghi l'USB, il numero dell'evento potrebbe cambiare.

TOUCH_DEVICE = '/dev/input/event4'

# Dimensioni fisiche dell'area sensibile dello schermo (in millimetri)
TOUCH_DIM_X_MM = 140.0
TOUCH_DIM_Y_MM = 180.0

# Valori raw centrali e massimi (da calibrare leggendo l'output raw del sensore).
# Il touch non dà millimetri ma numeri "grezzi" (raw): con questi valori si
# capisce quale numero corrisponde al centro e quanti numeri ci sono per mm.
TOUCH_CENTER_RAW_X = 1980
TOUCH_CENTER_RAW_Y = 1850
TOUCH_MAX_RAW_X = 3960
TOUCH_MAX_RAW_Y = 3700

# Filtro sulla posizione (0..1): 1 = nessun filtro, valori bassi = più liscio ma più lento.
# Il touch resistivo è rumoroso: la posizione letta "trema" di qualche decimo
# di mm anche con la pallina ferma. Il filtro media le letture.
TOUCH_FILTER_ALPHA = 0.5

# -- Configurazione Cinematica (in millimetri) --
# Stessi simboli del PDF: KIN_H = H, KIN_E = e, KIN_B = d, KIN_F = f, KIN_G = g.
KIN_H = 100.0  # Altezza degli snodi del piatto sopra l'asse dei servo, piatto in piano
KIN_E = 50.0   # Raggio del piatto (distanza centro piatto - snodi delle aste)
KIN_B = 50.0   # NUOVO: raggio della base (centro base -> asse del servo).
               # La versione precedente assumeva KIN_B == KIN_E.
KIN_F = 30.0   # Lunghezza della squadretta del servomotore
KIN_G = 80.0   # Lunghezza dell'asta di collegamento (pushrod)

# Posizione dei servo attorno alla base (gradi). A sull'asse X, B e C a 120/240,
# come nella formula originale delle altezze dei vertici.
# Nel PDF A è disegnato su -y: è la stessa geometria, solo ruotata.
SERVO_AZIMUTH_DEG = (0.0, 120.0, 240.0)

# Convenzione angoli: 90 = squadretta perpendicolare al pavimento.
# Come nella formula originale, angoli > 90 = squadretta inclinata verso l'esterno.
SERVO_MIN_DEG = 0.0
SERVO_MAX_DEG = 180.0

# -- Configurazione PID --
# Con la cinematica inversa i due assi si comportano allo stesso modo:
# l'accelerazione della pallina dipende solo dall'inclinazione del piatto, non da
# quanti motori la producono. Quindi stessi guadagni su X e Y.
# Unità: gradi di inclinazione per mm di errore (KP), per mm/s (KD), per mm*s (KI).
# Tuning sul robot reale: se la pallina oscilla, alzare un po' KD; se i servo
# vibrano, abbassare KD o TOUCH_FILTER_ALPHA; KI solo se resta fuori centro.
PID_X_KP = 0.10
PID_X_KI = 0.005
PID_X_KD = 0.05

PID_Y_KP = 0.10
PID_Y_KI = 0.005
PID_Y_KD = 0.05

PID_OUTPUT_LIMIT_DEG = 12.0  # Limite di sicurezza inclinazione (gradi)
PID_D_FILTER_ALPHA = 0.3     # Filtro sulla derivata (0..1), valori bassi = meno rumore

LOOP_HZ = 50    # cicli di controllo al secondo
PRINT_HZ = 10   # righe di telemetria al secondo (stampare è lento, quindi meno spesso)


class TouchScreenEvdev:
    """Legge il touchscreen e restituisce la posizione della pallina in mm."""

    def __init__(self, device_path):
        self.dev = InputDevice(device_path)
        self.dev.grab() # Acquisizione esclusiva per inibire il puntatore di sistema

        self.centro_x_raw = TOUCH_CENTER_RAW_X
        self.centro_y_raw = TOUCH_CENTER_RAW_Y

        # mm per ogni unità raw: serve per passare da numeri grezzi a millimetri
        self.scala_x = TOUCH_DIM_X_MM / TOUCH_MAX_RAW_X
        self.scala_y = TOUCH_DIM_Y_MM / TOUCH_MAX_RAW_Y

        self.last_x_raw = self.centro_x_raw
        self.last_y_raw = self.centro_y_raw

        # BTN_TOUCH è il segnale "c'è qualcosa sul pannello sì/no".
        # Se il controller non invia BTN_TOUCH, si assume il contatto sempre presente
        tasti = self.dev.capabilities().get(ecodes.EV_KEY, [])
        self.ha_btn_touch = ecodes.BTN_TOUCH in tasti
        self.toccato = not self.ha_btn_touch

    def leggi_posizione(self):
        """Ritorna (x_mm, y_mm, toccato)."""
        try:
            # Polling degli eventi in modalità non bloccante.
            # Il touch manda un evento solo quando qualcosa cambia: si leggono tutti
            # quelli arrivati dall'ultimo ciclo e si tiene l'ultimo valore.
            for event in self.dev.read():
                if event.type == ecodes.EV_ABS:
                    if event.code == ecodes.ABS_X:
                        self.last_x_raw = event.value
                    elif event.code == ecodes.ABS_Y:
                        self.last_y_raw = event.value
                elif event.type == ecodes.EV_KEY and event.code == ecodes.BTN_TOUCH:
                    self.toccato = event.value != 0
        except BlockingIOError:
            pass  # nessun evento nuovo: la pallina non si è mossa

        # Traslazione valori raw in coordinate millimetriche assolute
        # (distanza dal centro del pannello, in mm)
        x_mm = (self.last_x_raw - self.centro_x_raw) * self.scala_x
        y_mm = (self.last_y_raw - self.centro_y_raw) * self.scala_y

        # Scambio assi software applicato per conformità con la geometria attuale.
        # Ripristinare 'return x_mm, y_mm, ...' in caso di montaggio standard allineato.
        return y_mm, x_mm, self.toccato

    def rilascia(self):
        self.dev.ungrab()

class PID:
    """PID con derivata sulla misura filtrata (niente 'derivative kick') e anti-windup.

    Uscita = KP*errore + KI*integrale + KD*derivata, in gradi di inclinazione:
      P: più la pallina è lontana dal centro, più si inclina il piatto;
      I: somma gli errori nel tempo e corregge piccoli scostamenti che restano;
      D: guarda la velocità della pallina e frena prima che superi il centro.
    """

    def __init__(self, kp, ki, kd, limite_output):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limite_output = limite_output
        self.reset()

    def reset(self):
        # Riparte da zero: usato all'avvio e quando la pallina si stacca
        self.integral = 0.0
        self.prev_value = None
        self.derivative = 0.0
        self.prev_time = time.monotonic()  # orologio che non salta mai indietro

    def update(self, setpoint, current_value):
        current_time = time.monotonic()
        dt = current_time - self.prev_time   # secondi passati dall'ultimo calcolo
        if dt <= 0.0:
            dt = 1e-4

        error = setpoint - current_value

        self.integral += error * dt
        # Limite Anti-windup sull'integrale per evitare instabilità a lungo termine
        self.integral = max(-50.0, min(50.0, self.integral))

        if self.prev_value is not None:
            # Derivata della misura (stesso segno della derivata dell'errore con setpoint fisso).
            # Si usa la posizione e non l'errore: se un giorno il setpoint cambia
            # di colpo, la derivata non fa uno scatto.
            raw_d = -(current_value - self.prev_value) / dt
            # Filtro sulla derivata: dividere il rumore per dt piccolo lo ingigantisce,
            # quindi la si "ammorbidisce" prima di usarla.
            self.derivative += PID_D_FILTER_ALPHA * (raw_d - self.derivative)

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * self.derivative)

        self.prev_value = current_value
        self.prev_time = current_time

        # Mai oltre il limite di sicurezza, qualunque cosa calcoli il PID
        return max(-self.limite_output, min(self.limite_output, output))

# ==============================================================================
# CINEMATICA INVERSA 3D
# ==============================================================================
# Per ogni servo:
#   S = asse del servo sulla base, a raggio KIN_B, quota 0
#   P = snodo sul piatto, a raggio KIN_E, ruotato con pitch/roll e alzato di KIN_H
#   La squadretta (KIN_F) ruota nel piano radiale del servo; l'asta (KIN_G)
#   collega la sua punta a P. Si risolve: | P - (S + F*(cos a * u + sin a * z)) | = G
#   dove a = angolo della squadretta dall'orizzontale verso l'esterno.
# Angolo servo = 180 - a  (90 = verticale, > 90 = verso l'esterno, come prima).

def _matrice_rotazione(pitch_rad, roll_rad):
    # Matrice che inclina il piatto: applicata a un punto del piatto in piano,
    # dice dove finisce quel punto quando il piatto è inclinato.
    # +pitch alza il lato di A (asse X), +roll alza il lato +Y (come calcola_altezze_vertici)
    cp, sp = math.cos(-pitch_rad), math.sin(-pitch_rad)
    cr, sr = math.cos(roll_rad), math.sin(roll_rad)
    return ((cp, sp * sr, sp * cr),
            (0.0, cr, -sr),
            (-sp, cp * sr, cp * cr))

def _soluzioni_squadretta(pitch_deg, roll_deg, indice):
    R = _matrice_rotazione(math.radians(pitch_deg), math.radians(roll_deg))

    # Direzione del servo vista dall'alto (u = verso l'esterno, lungo il suo braccio)
    b = math.radians(SERVO_AZIMUTH_DEG[indice])
    ux, uy = math.cos(b), math.sin(b)

    # P: snodo sul piatto. Si parte dal punto a distanza e dal centro, lo si
    # ruota con il piatto e lo si alza di H.
    px_l, py_l = KIN_E * ux, KIN_E * uy
    P = (R[0][0] * px_l + R[0][1] * py_l,
         R[1][0] * px_l + R[1][1] * py_l,
         R[2][0] * px_l + R[2][1] * py_l + KIN_H)

    # d: vettore dall'asse del servo allo snodo (nel PDF: a, b, c)
    d = (P[0] - KIN_B * ux, P[1] - KIN_B * uy, P[2])

    # La condizione "punta della squadretta a distanza g da P" diventa
    # M*sin(a) + N*cos(a) = L, un'equazione con una sola incognita (a).
    L = d[0] ** 2 + d[1] ** 2 + d[2] ** 2 + KIN_F ** 2 - KIN_G ** 2
    M = 2 * KIN_F * d[2]                         # parte verticale
    N = 2 * KIN_F * (d[0] * ux + d[1] * uy)      # parte orizzontale, lungo il braccio
    r = math.hypot(M, N)
    if r < 1e-9 or abs(L / r) > 1.0:
        return None  # posizione non raggiungibile (snodo troppo lontano o troppo vicino)

    # Due soluzioni possibili: gomito verso l'esterno o verso l'interno
    phi = math.atan2(N, M)
    k = math.asin(L / r)
    return (k - phi, math.pi - k - phi)

def _vicino(a, rif):
    # Differenza tra due angoli, tenendo conto che 359° e 1° sono vicini
    return abs(math.atan2(math.sin(a - rif), math.cos(a - rif)))


# Angolo della squadretta con piatto in piano: si sceglie la soluzione con il
# gomito verso l'esterno (come nello schema A/B/C).
# Calcolato una volta all'avvio: con i valori attuali vale 131,4 gradi.
_sol_home = _soluzioni_squadretta(0.0, 0.0, 0)
if _sol_home is None:
    raise SystemExit("[ERRORE] Geometria KIN_* impossibile: il piatto in piano non è raggiungibile.")
ALPHA_HOME = max(_sol_home, key=math.cos)
ANGOLO_HOME = 180.0 - math.degrees(ALPHA_HOME)

def calcola_angoli_servo(pitch_deg, roll_deg, angoli_precedenti):
    """Ritorna i tre angoli servo (gradi). Se una posizione non è raggiungibile
    mantiene l'angolo precedente di quel servo."""
    angoli = []
    for i in range(3):
        sol = _soluzioni_squadretta(pitch_deg, roll_deg, i)
        if sol is None:
            angoli.append(angoli_precedenti[i])
            continue
        # Tra le due soluzioni si prende quella più simile alla posizione di
        # riposo: così il gomito resta sempre dalla stessa parte.
        a = min(sol, key=lambda x: _vicino(x, ALPHA_HOME))
        # Conversione nella convenzione del servo (90 = squadretta verticale)
        angolo = 180.0 - math.degrees(math.atan2(math.sin(a), math.cos(a)))
        angoli.append(max(SERVO_MIN_DEG, min(SERVO_MAX_DEG, angolo)))
    return angoli

# ==============================================================================
# MAIN LOOP
# ==============================================================================

def main():
    print("[INFO] Inizializzazione bus I2C e controller PCA9685...")
    i2c = busio.I2C(board.SCL, board.SDA)
    pca = PCA9685(i2c)
    pca.frequency = 50   # i servo standard vogliono un segnale a 50 Hz

    # Assegnazione motori ai rispettivi canali I2C sulla shield.
    # min/max_pulse: durata dell'impulso per 0° e 180°, dipende dal modello di servo.
    motori = [servo.Servo(pca.channels[ch], min_pulse=500, max_pulse=2500) for ch in (0, 1, 2)]

    def muovi(angoli):
        # Manda i tre angoli (A, B, C) ai tre servo
        for m, a in zip(motori, angoli):
            m.angle = a

    print(f"[INFO] Inizializzazione Touchscreen su {TOUCH_DEVICE}...")
    touch = TouchScreenEvdev(TOUCH_DEVICE)
    if not touch.ha_btn_touch:
        print("[WARN] Il touch non invia BTN_TOUCH: non posso capire quando la pallina si stacca.")

    # Homing: piatto in piano prima di cominciare
    home = [ANGOLO_HOME] * 3
    print(f"[INFO] Homing motori a {ANGOLO_HOME:.1f} gradi (piatto in piano)...")
    muovi(home)
    time.sleep(1.5)

    # Due PID: uno per l'inclinazione lungo X (pitch), uno lungo Y (roll)
    pid_pitch = PID(PID_X_KP, PID_X_KI, PID_X_KD, PID_OUTPUT_LIMIT_DEG)
    pid_roll  = PID(PID_Y_KP, PID_Y_KI, PID_Y_KD, PID_OUTPUT_LIMIT_DEG)

    setpoint_x, setpoint_y = 0.0, 0.0   # dove vogliamo la pallina: al centro
    filt_x = filt_y = None              # posizione filtrata (None = nessuna lettura ancora)
    angoli = list(home)
    periodo = 1.0 / LOOP_HZ             # 0,02 s
    prossimo = time.monotonic()         # istante in cui deve partire il prossimo ciclo
    ultimo_print = 0.0

    print("[INFO] Sistema avviato. Control loop in esecuzione.")

    try:
        while True:
            # 1. Acquisizione Sensore
            current_x, current_y, toccato = touch.leggi_posizione()

            current_y = -current_y

            # De-commentare la riga seguente se anche l'asse X spinge nel verso opposto:
            # current_x = -current_x

            if toccato:
                # 2. Filtro posizione + PID
                # Filtro: la nuova stima si sposta solo di una frazione (ALPHA)
                # verso la nuova lettura, così il rumore viene smorzato.
                if filt_x is None:
                    filt_x, filt_y = current_x, current_y
                else:
                    filt_x += TOUCH_FILTER_ALPHA * (current_x - filt_x)
                    filt_y += TOUCH_FILTER_ALPHA * (current_y - filt_y)
                pitch_target = pid_pitch.update(setpoint_x, filt_x)
                roll_target = pid_roll.update(setpoint_y, filt_y)
            else:
                # Pallina assente: piatto in piano e PID azzerato,
                # così quando la si rimette il controllo riparte pulito
                filt_x = filt_y = None
                pid_pitch.reset()
                pid_roll.reset()
                pitch_target = roll_target = 0.0

            # 3. Cinematica Inversa: da inclinazione (gradi) ad angoli dei tre servo
            angoli = calcola_angoli_servo(pitch_target, roll_target, angoli)

            # 4. Attuazione Fisica
            muovi(angoli)

            # Telemetria (10 Hz per non rallentare il loop)
            ora = time.monotonic()
            if ora - ultimo_print >= 1.0 / PRINT_HZ:
                ultimo_print = ora
                stato = "OK " if toccato else "---"
                print(f"{stato} X:{current_x:6.1f} Y:{current_y:6.1f} | "
                      f"PID_P:{pitch_target:5.1f} PID_R:{roll_target:5.1f} | "
                      f"A:{angoli[0]:5.1f} B:{angoli[1]:5.1f} C:{angoli[2]:5.1f}")

            # Mantenimento frequenza loop (50 Hz).
            # Si aspetta solo il tempo che manca alla scadenza del ciclo: se i calcoli
            # hanno preso 5 ms, si dorme 15 ms. Così ogni ciclo dura sempre 20 ms.
            prossimo += periodo
            attesa = prossimo - time.monotonic()
            if attesa > 0:
                time.sleep(attesa)
            else:
                prossimo = time.monotonic()   # in ritardo: si riparte da adesso

    except KeyboardInterrupt:
        # Ctrl+C: piatto in piano prima di spegnere
        print("\n[WARN] Interruzione manuale (SIGINT) rilevata. Arresto del sistema.")
        muovi(home)
        time.sleep(0.5)
    finally:
        # Eseguito sempre, anche in caso di errore: libera scheda servo e touch
        pca.deinit()
        touch.rilascia()
        print("[INFO] Terminazione completata. Risorse hardware rilasciate.")

if __name__ == "__main__":
    main()
