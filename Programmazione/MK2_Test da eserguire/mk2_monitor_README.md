# MK2 Monitor

Nexus, Ball balancing robot.

*Nota:* il design dell'interfaccia grafica è stato progettato con l'Intelligenza artificiale seguendo le linee guida di design di Nexus. Il codice dell'interfaccia è stato scritto a mano, tranne la sezione dell'animazione 3d.

`mk2_monitor.py` apre una finestra divisa a metà: a sinistra i dati (posizione della pallina, angoli dei servo, grafici), a destra il disegno 3D della piattaforma. Usa lo stesso PID e la stessa cinematica di `mk2_controller.py`.

Ha tre modalità, che si scelgono dai pulsanti nella finestra:

| Modalità | Pallina | Servo | Dove funziona |
| --- | --- | --- | --- |
| Simulazione | simulata | solo disegno | Mac, Windows, Raspberry |
| Sim + servo | simulata | reale | solo Raspberry |
| Reale | touch reale | reale | solo Raspberry |

Sul Mac e su Windows i pulsanti "Sim + servo" e "Reale" sono grigi: è normale.

## Mac

1. Installa Python da [python.org/downloads](https://www.python.org/downloads/) (versione 3.10 o più recente). Il Python già presente sul Mac a volte apre una finestra vuota: quello di python.org funziona.
2. Apri il Terminale (Cmd + Spazio, scrivi "Terminale", Invio).
3. Spostati nella cartella del file. Scrivi `cd ` (con lo spazio), trascina la cartella dentro il Terminale e premi Invio.
4. Avvia:

```
python3 mk2_monitor.py
```

Non serve installare altre librerie per la simulazione.

## Windows

1. Installa Python da [python.org/downloads](https://www.python.org/downloads/). Nella prima schermata dell'installazione spunta **"Add python.exe to PATH"**.
2. Apri la cartella del file in Esplora file, clicca sulla barra dell'indirizzo, scrivi `cmd` e premi Invio: si apre il Prompt dei comandi già nella cartella giusta.
3. Avvia:

```
py mk2_monitor.py
```

Anche qui non serve installare altre librerie per la simulazione.

## Raspberry Pi

Va lanciato dal desktop del Raspberry (con schermo collegato): da SSH la finestra non si apre.

1. Attiva l'I2C, necessario per la scheda dei servo:

```
sudo raspi-config
```

   Interface Options, poi I2C, poi Yes. Riavvia.

2. Installa tkinter e crea un ambiente per le librerie (va fatto una volta sola):

```
sudo apt install python3-tk
python3 -m venv --system-site-packages ~/mk2-env
source ~/mk2-env/bin/activate
pip install adafruit-blinka adafruit-circuitpython-pca9685 adafruit-circuitpython-motor evdev
```

3. Permessi per leggere il touch senza `sudo` (una volta sola, poi riavvia):

```
sudo usermod -aG input $USER
```

4. Trova il percorso del touch e, se non è `/dev/input/event4`, cambia `TOUCH_DEVICE` in cima al file:

```
python3 -m evdev.evtest
```

5. Avvia (ogni volta, dalla cartella del file):

```
source ~/mk2-env/bin/activate
python3 mk2_monitor.py
```

Per far girare solo il controllo, senza finestra: `python3 mk2_controller.py`.

## Uso

- **Avvia / Pausa**: fa partire o ferma il controllo.
- **Reset**: piatto in piano, pallina nuova (in simulazione).
- **Spingi pallina**: un colpetto alla pallina simulata.
- **Modalità**: Simulazione, Sim + servo, Reale. Passando ai servo reali compare una richiesta di conferma.
- **Percorso**: Centro, Cerchio, Quadrato, Infinito, Fiore.
- Trascinando con il mouse sul disegno 3D si ruota la vista.

Prima prova sul robot: Simulazione, poi Sim + servo senza pallina (il piatto deve inclinarsi dal lato giusto e senza scatti), poi Reale con percorso Centro.

## Problemi comuni

| Messaggio o sintomo | Soluzione |
| --- | --- |
| `command not found: python3` (Mac) | Python non è installato: vedi il punto 1 della sezione Mac. |
| `'py' non è riconosciuto` (Windows) | Reinstalla Python spuntando "Add python.exe to PATH". |
| Finestra vuota o grigia (Mac) | Usa il Python di python.org, non quello di sistema. |
| `No module named 'tkinter'` (Raspberry) | `sudo apt install python3-tk` |
| `No module named 'board'` (Raspberry) | Manca `source ~/mk2-env/bin/activate` prima di avviare. |
| "Sim + servo" e "Reale" grigi sul Raspberry | Le librerie non sono installate nell'ambiente attivo: ripeti il punto 2. |
| `Permission denied` sul touch | Ripeti il punto 3 e riavvia. |
| Grafica lenta sul Raspberry | Abbassa `GUI_HZ` da 30 a 15 in cima al file. |