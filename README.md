Questo repository contiene la documentazione e i file relativi al progetto del club Nexus Robotics: una piattaforma robotica capace di mantenere in equilibrio una pallina metallica su un piano touch, e di muoverla secondo pattern richiesti, tramite un controllo PID.

> **Stato del progetto**: la progettazione meccanica è conclusa. La programmazione e i test sono ancora in corso.
> 

## Funzionamento

Il robot è composto da un piatto touch resistivo rettangolare, sul quale rotola liberamente una pallina metallica, sostenuto da tre bracci mossi da altrettanti servomotori posti alla base. Il piatto touch rileva in tempo reale la posizione (x, y) della pallina; un Raspberry Pi legge questa posizione e, tramite un algoritmo PID, calcola l'inclinazione da dare al piatto muovendo i tre servomotori (pilotati tramite driver PCA9685), in modo da mantenere la pallina in equilibrio al centro oppure farle seguire un percorso impostato via software.

## Struttura del repository

- **`Progettazione meccanica/`** – File di progettazione Fusion 360 dell'assieme completo (`Assieme.f3z` e `Assieme.stl`), immagine di anteprima (`Assieme.png`)
- **`Programmazione/`** – Codice del controllo PID e gestione servomotori *(in fase di sviluppo)*

## Componenti principali

| **Componente** | **Modello** | **Quantità** |
| --- | --- | --- |
| Micro controllore | Raspberry Pi 4 o 5 | 1 |
| Servomotori | Servo da almeno 17 kg·cm | 3 |
| Driver servo | Scheda driver PCA9685 | 1 |
| Sensore posizione | Wire resistive touch panel (interfaccia USB) | 1 |
| Snodi | Rod end ball joint | 3 |
| Pallina | Pallina di test da 50g | 5 |
| Alimentatore | Alimentatore 5V (dedicato al driver PCA9685) | 1 |
| Varie | Cavi, viti M3 | vari |

*Costi stimati indicativamente; per note aggiuntive sull'alimentazione vedi la documentazione completa.*

## Documentazione completa

Per il dettaglio su progettazione meccanica, assemblaggio, software e attivazione, consulta la documentazione disponibile nella home page del repository.

## Licenza

Vedi il file `LICENSE` nella home page del repository.
