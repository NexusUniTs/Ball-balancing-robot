from evdev import InputDevice, ecodes

# Cambia 'event0' con il numero del tuo dispositivo!
dev = InputDevice('/dev/input/event0')

# Prende il controllo esclusivo cosÃ¬ non muovi il mouse di sistema
dev.grab()

print(f"Pronto! Sto ascoltando le coordinate su: {dev.name}")
print("Tocca lo schermo. Premi Ctrl+C per uscire.")

# Variabili per memorizzare l'ultima posizione letta
x = 0
y = 0

try:
    for event in dev.read_loop():
        # Filtriamo solo gli eventi di tipo "movimento assoluto" (touchscreen)
        if event.type == ecodes.EV_ABS:
            
            if event.code == ecodes.ABS_X:
                x = event.value
                print(f"Coordinate -> X: {x} | Y: {y}")
                
            elif event.code == ecodes.ABS_Y:
                y = event.value
                print(f"Coordinate -> X: {x} | Y: {y}")

except KeyboardInterrupt:
    # Rilascia il dispositivo quando premiamo Ctrl+C
    dev.ungrab()
    print("\nTest terminato. Dispositivo rilasciato.")