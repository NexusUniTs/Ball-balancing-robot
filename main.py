"""
Ball and Beam 3-DOF Controller (Stewart Platform)
-------------------------------------------------
Questo script implementa un controller PID per bilanciare una pallina
su un pannello touchscreen resistivo mosso da tre servomotori.

Hardware richiesto:
- Raspberry Pi
- Controller PWM PCA9685 (connesso via I2C)
- 3x Servomotori (collegati ai canali 0, 1, 2 del PCA9685)
- Touchscreen resistivo USB (letto tramite subsystem evdev di Linux)

Dipendenze:
- adafruit-circuitpython-pca9685
- adafruit-circuitpython-motor
- evdev
"""

import time
import math
import board
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from evdev import InputDevice, ecodes

# ==============================================================================
# 1. CONFIGURAZIONE HARDWARE E MECCANICA
# ==============================================================================

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
# Per un setup permanente, puoi usare i symlink fissi generati dal sistema, 
# verificando il contenuto della cartella: ls -l /dev/input/by-id/

TOUCH_DEVICE = '/dev/input/event4'

# Dimensioni fisiche dell'area sensibile dello schermo (in millimetri)
TOUCH_DIM_X_MM = 140.0
TOUCH_DIM_Y_MM = 180.0

# Valori raw centrali e massimi (da calibrare leggendo l'output raw del sensore)
TOUCH_CENTER_RAW_X = 1980
TOUCH_CENTER_RAW_Y = 1850
TOUCH_MAX_RAW_X = 3960
TOUCH_MAX_RAW_Y = 3700

# -- Configurazione Cinematica (in millimetri) --
KIN_H = 100.0  # Altezza del piatto a riposo (dalla base al centro dello snodo)
KIN_E = 50.0   # Raggio del piatto (distanza centro piatto - snodi delle aste)
KIN_F = 30.0   # Lunghezza della squadretta del servomotore
KIN_G = 80.0   # Lunghezza dell'asta di collegamento (pushrod)

# -- Configurazione PID --
# L'asse X (guidato da 1 motore) e l'asse Y (guidato da 2 motori) hanno inerzie
# differenti. Il tuning deve essere gestito separatamente per evitare overshooting.
PID_X_KP = 0.08
PID_X_KI = 0.0
PID_X_KD = 0.15

PID_Y_KP = 0.15
PID_Y_KI = 0.0
PID_Y_KD = 0.10

PID_OUTPUT_LIMIT_DEG = 12.0  # Limite di sicurezza inclinazione (gradi)

# ==============================================================================
# GUIDA ALLA RISOLUZIONE DEI PROBLEMI DI ORIENTAMENTO
# ==============================================================================
# Se l'hardware non risponde in modo geometricamente coerente, verificare:
#
# 1. ASSI SCAMBIATI (Il robot si inclina di lato quando la pallina va in avanti)
#    Causa: Il sensore touchscreen è ruotato di 90 gradi rispetto ai motori.
#    Soluzione: Nella classe TouchScreenEvdev, metodo leggi_posizione(), 
#    invertire i valori di ritorno (es. "return y_mm, x_mm" invece di "x, y").
#
# 2. ASSE INVERTITO (Il robot si inclina sull'asse giusto, ma nel verso opposto)
#    Causa: Motori o sensori sono montati in modo speculare alla cinematica.
#    Soluzione: Nel loop while principale, invertire il segno della variabile 
#    assegnata a quell'asse (es. current_y = -current_y) prima di passarla al PID.
# ==============================================================================

class TouchScreenEvdev:
    def __init__(self, device_path):
        self.dev = InputDevice(device_path)
        self.dev.grab() # Acquisizione esclusiva per inibire il puntatore di sistema
        
        self.centro_x_raw = TOUCH_CENTER_RAW_X
        self.centro_y_raw = TOUCH_CENTER_RAW_Y
        
        self.scala_x = TOUCH_DIM_X_MM / TOUCH_MAX_RAW_X
        self.scala_y = TOUCH_DIM_Y_MM / TOUCH_MAX_RAW_Y
        
        self.last_x_raw = self.centro_x_raw
        self.last_y_raw = self.centro_y_raw

    def leggi_posizione(self):
        try:
            # Polling degli eventi in modalità non bloccante
            for event in self.dev.read():
                if event.type == ecodes.EV_ABS:
                    if event.code == ecodes.ABS_X:
                        self.last_x_raw = event.value
                    elif event.code == ecodes.ABS_Y:
                        self.last_y_raw = event.value
        except BlockingIOError:
            pass 

        # Traslazione valori raw in coordinate millimetriche assolute
        x_mm = (self.last_x_raw - self.centro_x_raw) * self.scala_x
        y_mm = (self.last_y_raw - self.centro_y_raw) * self.scala_y
        
        # Scambio assi software applicato per conformità con la geometria attuale.
        # Ripristinare 'return x_mm, y_mm' in caso di montaggio standard allineato.
        return y_mm, x_mm
        
    def rilascia(self):
        self.dev.ungrab()

class PID:
    def __init__(self, kp, ki, kd, limite_output):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limite_output = limite_output
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time.time()

    def update(self, setpoint, current_value):
        current_time = time.time()
        dt = current_time - self.prev_time
        if dt <= 0.0: 
            dt = 1e-4

        error = setpoint - current_value
        
        self.integral += error * dt
        # Limite Anti-windup sull'integrale per evitare instabilità a lungo termine
        self.integral = max(-50.0, min(50.0, self.integral)) 
        
        derivative = (error - self.prev_error) / dt
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        
        self.prev_error = error
        self.prev_time = current_time
        
        return max(-self.limite_output, min(self.limite_output, output))

# ==============================================================================
# CINEMATICA (Stewart Platform)
# ==============================================================================

COS_30 = math.cos(math.radians(30))
SIN_30 = math.sin(math.radians(30))

def calcola_altezze_vertici(pitch_deg, roll_deg):
    theta = math.radians(pitch_deg)
    phi = math.radians(roll_deg)
    
    zA = KIN_H + (KIN_E * math.sin(theta))
    zB = KIN_H + (KIN_E * COS_30 * math.sin(phi)) - (KIN_E * SIN_30 * math.sin(theta))
    zC = KIN_H - (KIN_E * COS_30 * math.sin(phi)) - (KIN_E * SIN_30 * math.sin(theta))
    return zA, zB, zC

def calcola_angolo_servo(zi):
    Li = zi
    try:
        parte1 = math.asin(zi / Li)
        argomento_arccos = max(-1.0, min(1.0, (KIN_F**2 + Li**2 - KIN_G**2) / (2 * KIN_F * Li)))
        parte2 = math.acos(argomento_arccos)
        
        angolo_out = math.degrees(parte1 + parte2)
        return max(0.0, min(180.0, angolo_out))
    except ValueError:
        return 90.0

# ==============================================================================
# MAIN LOOP
# ==============================================================================

def main():
    print("[INFO] Inizializzazione bus I2C e controller PCA9685...")
    i2c = busio.I2C(board.SCL, board.SDA)
    pca = PCA9685(i2c)
    pca.frequency = 50
    
    # Assegnazione motori ai rispettivi canali I2C sulla shield
    motore_A = servo.Servo(pca.channels[0], min_pulse=500, max_pulse=2500)
    motore_B = servo.Servo(pca.channels[1], min_pulse=500, max_pulse=2500)
    motore_C = servo.Servo(pca.channels[2], min_pulse=500, max_pulse=2500)

    print(f"[INFO] Inizializzazione Touchscreen su {TOUCH_DEVICE}...")
    touch = TouchScreenEvdev(TOUCH_DEVICE)

    print("[INFO] Esecuzione homing motori (90 gradi)...")
    motore_A.angle = 90
    motore_B.angle = 90
    motore_C.angle = 90
    time.sleep(1.5)

    pid_pitch = PID(PID_X_KP, PID_X_KI, PID_X_KD, PID_OUTPUT_LIMIT_DEG)
    pid_roll  = PID(PID_Y_KP, PID_Y_KI, PID_Y_KD, PID_OUTPUT_LIMIT_DEG)
    
    setpoint_x, setpoint_y = 0.0, 0.0

    print("[INFO] Sistema avviato. Control loop in esecuzione.")

    try:
        while True:
            # 1. Acquisizione Sensore
            current_x, current_y = touch.leggi_posizione()

            # --- CORREZIONE ORIENTAMENTO FISICO ---
            # Fare riferimento alla guida a inizio script in caso di discrepanze.
            current_y = -current_y 
            
            # De-commentare la riga seguente se anche l'asse X spinge nel verso opposto:
            # current_x = -current_x

            # 2. Controllo PID
            pitch_target = pid_pitch.update(setpoint_x, current_x)
            roll_target = pid_roll.update(setpoint_y, current_y)

            # 3. Cinematica Inversa
            zA, zB, zC = calcola_altezze_vertici(pitch_target, roll_target)
            angolo_A = calcola_angolo_servo(zA)
            angolo_B = calcola_angolo_servo(zB)
            angolo_C = calcola_angolo_servo(zC)

            # 4. Attuazione Fisica
            motore_A.angle = angolo_A
            motore_B.angle = angolo_B
            motore_C.angle = angolo_C
            
            # Telemetria standard output
            print(f"X:{current_x:6.1f} Y:{current_y:6.1f} | PID_P:{pitch_target:5.1f} PID_R:{roll_target:5.1f} | M_A:{angolo_A:5.1f}")

            # Mantenimento frequenza loop target (~50Hz)
            time.sleep(0.02) 

    except KeyboardInterrupt:
        print("\n[WARN] Interruzione manuale (SIGINT) rilevata. Arresto del sistema.")
        motore_A.angle = 90
        motore_B.angle = 90
        motore_C.angle = 90
        time.sleep(0.5)
    finally:
        pca.deinit()
        touch.rilascia()
        print("[INFO] Terminazione completata. Risorse hardware rilasciate.")

if __name__ == "__main__":
    main()