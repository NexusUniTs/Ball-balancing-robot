Progetto Nexus Robotics

> **Stato del progetto**: la progettazione meccanica è conclusa. La programmazione e i test sono ancora in corso — alcune sezioni di questa documentazione saranno completate progressivamente.
> 

---

## 1. Descrizione del progetto

Piattaforma robotica di bilanciamento: tre bracci, mossi da altrettanti servomotori posti alla base, muovono un piatto su cui è montato un pannello touch resistivo rettangolare. Su questo piatto si muove una pallina metallica. Un Raspberry Pi legge la posizione della pallina dal touch panel e, tramite un controllo PID, pilota i servomotori per mantenere la pallina in equilibrio al centro del piatto, oppure per farla muovere secondo pattern richiesti.

## 2. Funzionamento

1. Il touch panel resistivo rileva la posizione (x, y) della pallina sul piatto.
2. Il Raspberry Pi legge questa posizione via USB.
3. L'algoritmo PID calcola, in base alla posizione letta (e all'eventuale posizione/pattern target), l'inclinazione da dare al piatto.
4. Il Raspberry Pi invia i comandi ai tre servomotori tramite il driver PCA9685.
5. I tre bracci, collegati al piatto tramite snodi (rod end ball joint), inclinano il piatto di conseguenza.
6. Il ciclo si ripete in continuo, correggendo la posizione della pallina in tempo reale.

## 3. Componenti

| **Componente** | **Modello** | **Quantità** | **Fornitore/Link** | **Costo per unità** |
| --- | --- | --- | --- | --- |
| Micro controllore | Raspberry Pi 5 | 1 | prestato da Asperastra | 0 |
| Servomotori | Servo da almeno 6 kg·cm (scelti da 17 kg·cm) | 3 | Amazon | €12  |
| Driver servo | Scheda driver PCA9685 | 1 | Amazon | €8 |
| Sensore posizione | Wire resistive touch panel (interfaccia USB) | 1 | Amazon | €20 |
| Snodi | Rod end ball joint | 3 | Amazon | €2 |
| Pallina | Pallina di test da 50g | 5 | Amazon | €1 |
| Alimentatore | Alimentatore USB-C (Raspberry Pi) | 1 | Amazon | €10 |
| Alimentatore | Alimentatore 5V dedicato (driver PCA9685) | 1 | Amazon | €10 |
| Varie | Cavi, viti M3 | vari | Amazon | €5 |

**Costo totale stimato del progetto: €100** 

> *Nota: i costi sono stime indicative basate su prodotti equivalenti reperibili su Amazon; possono variare in base al venditore.*
> 

> **Attenzione:** Il Raspberry Pi e il driver PCA9685 richiedono alimentazioni separate: un alimentatore USB-C dedicato al Raspberry Pi, e un alimentatore 5V dedicato al driver/servomotori. Verificare sempre il voltaggio corretto prima di collegare l'alimentazione.
> 

## 4. Schema elettronico
<img width="2720" height="1760" alt="schema_illustrativo_rpi5_servo" src="https://github.com/user-attachments/assets/7f25ab6e-cacc-466e-a2c3-fa9f3d22e16e" />
## 5. Progettazione meccanica

Tutte le parti sono state progettate in Fusion 360 e sono disponibili nella cartella `Progettazione meccanica/`:

- **`Assieme.f3z`**/`Assieme.stl` – progetto Fusion 360 dell'assieme completo. Aprendo questo file è possibile visualizzare e modificare ogni singolo componente del modello.
- **`Assieme.png`** – immagine di anteprima dell'assieme.

Tutti i componenti (base, bracci, supporto del piatto) sono stampati in PLA. Il piatto touch è fissato tramite clip stampate in 3D.

## 6. Assemblaggio

Montaggio manuale con viti M3:

- Fissaggio dei tre servomotori alla base
- Fissaggio della base e dell'elettronica (Raspberry Pi, driver PCA9685)
- Collegamento dei bracci al piatto tramite i rod end ball joint
- Il piatto touch viene bloccato in posizione tramite clip stampate in 3D (nessuna vite necessaria per il fissaggio del touch panel)

## 7. Calibrazione

*[Da completare]*

## 8. Software

Il codice è scritto in Python, e l’ultima versione è disponibile nella cartella `Programmazione/`.

### Setup del progetto

*[Da completare, incluso il setup del touch panel resistivo]*

### Regolazione del comportamento (PID)

Nel codice sarà possibile modificare i coefficienti del controllo PID per cambiare il comportamento del robot (es. velocità di correzione, stabilità). Si consiglia di modificare un coefficiente alla volta e testare il comportamento prima di procedere con la modifica successiva.


### Interfaccia utente

*[Da completare]*

## 9. Attivazione

1. Assemblare completamente il robot, se non già fatto (vedi sezione 6).
2. Controllare che i servomotori siano collegati correttamente al driver PCA9685, seguendo lo schema elettronico (vedi sezione 4).
3. Accendere il Raspberry Pi.
4. Verificare il setup del touch panel *(sezione da completare — vedi sezione 8)*.
5. Avviare il codice *[nome dello script da definire]*.
