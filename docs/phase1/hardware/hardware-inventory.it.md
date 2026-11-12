# Inventario hardware/software BlueROV2 — Fase 1 (ricognizione)

Progetto: integrazione HoloOcean ↔ BlueROV2 reale (ArduSub).
Data: **2026-10-13** (rilievo live 06:50–06:58 UTC). Autore: ricognizione automatica in sola lettura.
Stato del veicolo durante il rilievo: **fuori dall'acqua, DISARMATO, modo MANUAL** (HEARTBEAT `base_mode=81`, `custom_mode=19`, letto dalla cache mavlink2rest, E23).

## 0. Regole seguite e livelli di evidenza

Sul veicolo sono stati fatti **solo** ping ICMP e HTTP GET (curl `-m 5 -X GET`) verso BlueOS e verso la cache di mavlink2rest. Nessun messaggio MAVLink inviato, nessuna connessione TCP alla Surveyor (<SONAR-IP>:62312), nessuna UI SonarView aperta, nessuna POST/PUT/DELETE/PATCH. Sono stati scelti solo endpoint GET, controllati sulle specifiche OpenAPI salvate (`RoverLago/backups/2026-10-05_pre-change/blueos_api/openapi/*.json`). Nessun file è stato modificato fuori da `HoloOcean_BlueROV2_Phase1/`.

| Livello | Significato |
|---|---|
| **L1** | verificata oggi (2026-10-13) sul dispositivo o sul PC, con comando (rif. `Exx` nella sezione 12) |
| **L2** | verificata in un file locale (`percorso:riga`) |
| **L3** | documentazione pubblica o sorgente ufficiale (URL) |
| **L4** | dato storico (documento o misura precedente) da riconfermare |
| **L5** | sconosciuta: serve un'ispezione fisica o una verifica che qui non si può fare |

Abbreviazioni dei percorsi:
- `RL` = `<workstation>/RoverLago`
- `BK` = `RL\backups\2026-10-05_pre-change`
- `PARM` = `BK\ardusub\params_20261005T101049Z.parm` (913 righe, una per parametro; il numero di riga è citato come `PARM:n`)
- `EV` = `<workstation>/HoloOcean_BlueROV2_Phase1\evidence\live_2026-10-13`
- `HOA` = `<workstation>/HoloObstacleAvoidance`
- `SRC` = sorgente ArduPilot al tag `ArduSub-4.1.2` (= commit `2dd0bb7d4c85…`, E40), `https://github.com/ArduPilot/ardupilot/blob/ArduSub-4.1.2/…`

---

## 1. Sintesi

| Voce | Esito | Livello |
|---|---|---|
| Raggiungibilità | Primo tentativo (06:50Z): ROV **irraggiungibile**, perché l'adattatore del tether (Realtek RTL8152 "Ethernet 2") **non era collegato al PC** (E01–E05). Dopo il collegamento (06:53Z) BlueOS e la Surveyor rispondono al ping, 0% di perdita, RTT 4–10 ms (E06–E07) | L1 |
| Modello | **BlueROV2 Heavy (8 thruster)**: `FRAME_CONFIG=2` (VECTORED_6DOF), `SERVO1..8_FUNCTION=33..40` (Motor1..8); oggi `SERVO_OUTPUT_RAW` ha 1..8 a 1500 µs | L2 + L1 |
| Firmware | **ArduSub 4.1.2 STABLE**, git `2dd0bb7d` | L1 |
| BlueOS | **core 1.3.1**, bootstrap 1.3.1 | L1 |
| Autopilota | **Blue Robotics Navigator** su **Raspberry Pi 4 B** (Debian 12, kernel 5.10.33-v7l+, armv7l) | L1 |
| Parametri | 913 sul veicolo (oggi `PARAM_VALUE.param_count=913`, già in cache); backup 2026-10-05 con 913 parametri in `.parm`, `.params` e `.json`, contenuti identici fra loro | L1 + L2 |
| Integrità backup | SHA256SUMS.txt: **139/139 OK**, 0 errati, 0 mancanti (E30) | L1 |
| Confronto con il backup | 28 risposte GET su 28 coincidono col backup del 2026-10-05, a parte ordine, UUID, uptime e telemetria (sezione 9) | L1 |
| Discrepanze | orologio BlueOS indietro di **~7 giorni e 17 h**; servizio *Commander* in più nell'elenco web; stream "Prova1" (UDP 5601) presente ad agosto, assente oggi; nessuna discrepanza su versioni, frame, porte o sensori | L1 |

---

## 2. Veicolo e configurazione dei motori

### 2.1 Modello: standard o Heavy

| Evidenza | Valore | Fonte | Livello |
|---|---|---|---|
| `FRAME_CONFIG` | **2** = `SUB_FRAME_VECTORED_6DOF` | `PARM:358` | L2 |
| `SERVO1..8_FUNCTION` | 33, 34, 35, 36, 37, 38, 39, 40 = Motor1..Motor8 | `PARM:794,799,804,809,814,819,824,829` | L2 |
| Uscite attive oggi | `SERVO_OUTPUT_RAW`: servo1..8 = **1500** (neutro, disarmato); servo13 = 1100, servo14 = 1100, servo16 = 1500; servo9..12 e 15 = 0 (non pilotate) → **11 uscite attive** | E24 (GET mavlink2rest, cache) | L1 |
| Matrice VECTORED_6DOF | 8 motori (righe 156–166) | SRC `libraries/AP_Motors/AP_Motors6DOF.cpp#L156-L166` | L3 |
| Frame standard (VECTORED) | 6 motori, `FRAME_CONFIG=1` (righe 168–176): **non** è la configurazione attiva | idem | L3 |
| Retrofit Heavy Blue Robotics | aggiunge 2 thruster verticali (7, 8), per un totale di 4 verticali; eliche 5 CCW, 6 CW, 7 CW, 8 CCW | https://bluerobotics.com/learn/bluerov2-heavy-configuration-retrofit-kit-installation/ | L3 |
| Prova in acqua del 2026-08-14 | "BlueROV2 **Heavy** (8 thrusters)", mixing orizzontale verificato | `HOA\docs\real_vehicle_reference\README.md:3,86-99` | L4 |
| STATUSTEXT al boot "Frame: VECTORED_6DOF" | riportato ad agosto | `HOA\docs\real_vehicle_reference\README.md:25-26` | L4 |

**Conclusione: BlueROV2 Heavy, 8 thruster (L2 per la configurazione, L1 per le 8 uscite attive).** Il numero di thruster montati fisicamente non è verificabile da software (un'uscita a 1500 µs non prova che ci sia un ESC collegato). Le evidenze indirette sono la prova in acqua di agosto (L4) e la documentazione RoverLago, che vanno riconfermate a vista (L5).

### 2.2 Uscite ausiliarie, joystick e accessori

| Uscita / parametro | Valore | Significato (SRC 4.1.2) | Oggi (SERVO_OUTPUT_RAW / RC_CHANNELS) | Livello |
|---|---|---|---|---|
| `SERVO9_FUNCTION` | 0 (Disabled) | uscita 9 libera; ArduSub usa l'uscita **9** come `SERVO_CHAN_1` per i pulsanti servo_1 (`ArduSub/joystick.cpp:25`) | 0 µs | L2/L1 |
| `SERVO10..12_FUNCTION` | 0 | libere (`SERVO_CHAN_2/3` = uscite 10, 11) | 0 µs | L2/L1 |
| `SERVO13_FUNCTION` | **59 = RCIN9** | passthrough RC9 = **lights1** (`joystick.cpp:142`) | 1100 µs (spente); RC9 = 1100 | L2/L1 |
| `SERVO14_FUNCTION` | **60 = RCIN10** | passthrough RC10 = **lights2** (`joystick.cpp:143`) | 1100 µs; RC10 = 1100 | L2/L1 |
| `SERVO15_FUNCTION` | 0 | libera | 0 µs | L2/L1 |
| `SERVO16_FUNCTION` | **7 = mount tilt**, `SERVO16_REVERSED=1` | tilt della camera (`MNT_TYPE=1` servo, `MNT_RC_IN_TILT=8`, ±45°) | 1500 µs | `PARM:789-793,523-532` L2/L1 |
| `SERVOn_MIN/MAX/TRIM` (1..16) | 1100/1900/1500 per tutti; `REVERSED=0` tranne SERVO16 | — | — | `PARM:759-838` L2 |
| `MOT_PWM_MIN/MAX` | 1100/1900 | — | — | `PARM:553-554` L2 |
| `RELAY_PIN..6` | −1 (nessun relè) | — | — | `PARM:711-717` L2 |
| `BTN3_SFUNCTION`=63, `BTN4_SFUNCTION`=64, `BTN10_SFUNCTION`=76, `BTN9_SFUNCTION`=77 | servo_1 min / max / min momentaneo / max momentaneo → uscita **9** | tipica mappatura del **gripper** Blue Robotics (Newton) sull'uscita 9 | — | `PARM:175,177,127,187`; SRC `AP_JSButton.h:67-84`, `joystick.cpp:494-516` L2/L3 |
| Altri BTN | 13/14 = lights1 dimmer/brighter (33/32), shift 35/36 = lights2; 9/10 = tilt up/down (22/23); 7 = mount center (21) | default ArduSub, con lights2 aggiunte sui tasti shift | — | `PARM:124-187` L2 |
| `LEAK1_PIN` | 27 (`LEAK1_LOGIC=0`), `FS_LEAK_ENABLE=1` (solo avviso) | sensore di perdite sul pin GPIO 27 del Navigator | stato non letto (nessun allarme leak in cache) | `PARM:486-487,367` L2 |

**Accessori (presenza fisica):**

| Accessorio | Indizi | Livello |
|---|---|---|
| Luci (es. Lumen) su uscita 13 | `SERVO13_FUNCTION=59`; `RovCode\config.yaml:1-5` (luci su servo 13) | L2 per la configurazione; modello e numero **L5** |
| Secondo canale luci / **laser** su uscita 14 | `SERVO14_FUNCTION=60` (lights2); `RovCode\config.yaml:46-55` usa l'uscita 14 come **laser** (on 1900 / off 1100); sessioni `RovCode\output\laser_scaler_20260219_*\summary.json`: in 4 prove su 5 nessun punto laser valido, in una 7 campioni validi su 100 | presenza del laser **L5** (L4 debole) |
| **Gripper** su uscita 9 | `RovCode\config.yaml:30-45` (servo 9, 1900 apre / 1100 chiude); pulsanti BTN servo_1 (sopra); `SERVO9_FUNCTION=0` (pilotabile con DO_SET_SERVO) | **L5** (nessuna evidenza che sia montato) |
| Tilt camera | `SERVO16_FUNCTION=7`, oggi 1500 µs; `MOUNT_STATUS` in cache | L1/L2 (servo configurato); presenza fisica del servo L4 |

### 2.3 Mappa dei thruster (motore → uscita → posizione → fattori)

**Fonte della matrice:** `AP_Motors6DOF.cpp` al tag `ArduSub-4.1.2`, righe 156–166 (L3; file scaricato in sola lettura, E41). Ordine delle colonne in `add_motor_raw_6dof`: roll, pitch, yaw, throttle, forward, lateral, testing order.
L'uscita PWM vale `MOT_n_DIRECTION × (fattori · ingresso)` (`AP_Motors6DOF.cpp:355,479,556`: `_motor_reverse[i]*(rpy_out+linear_out)`).

**Tabella A — fattori del codice e direzioni configurate**

| Motore | Uscita Navigator (`SERVOn_FUNCTION`) | Roll | Pitch | Yaw | Throttle | Forward | Lateral | `MOT_n_DIRECTION` (`PARM`) |
|---|---|---|---|---|---|---|---|---|
| 1 | 1 (33) | 0 | 0 | +1 | 0 | −1 | +1 | **−1** (536) |
| 2 | 2 (34) | 0 | 0 | −1 | 0 | −1 | −1 | **−1** (537) |
| 3 | 3 (35) | 0 | 0 | −1 | 0 | +1 | +1 | **+1** (538) |
| 4 | 4 (36) | 0 | 0 | +1 | 0 | +1 | −1 | **+1** (539) |
| 5 | 5 (37) | +1 | −1 | 0 | −1 | 0 | 0 | **−1** (540) |
| 6 | 6 (38) | −1 | −1 | 0 | −1 | 0 | 0 | **+1** (541) |
| 7 | 7 (39) | +1 | +1 | 0 | −1 | 0 | 0 | **+1** (542) |
| 8 | 8 (40) | −1 | +1 | 0 | −1 | 0 | 0 | **−1** (543) |

`MOT_9..12_DIRECTION=1` non sono usati. Altri `MOT_*`: `MOT_FV_CPLNG_K=1.0`, `MOT_THST_EXPO=0.65`, `MOT_SPIN_MIN=0.15`, `MOT_SPIN_MAX=0.95`, `MOT_YAW_HEADROOM=200` (`PARM:545-566`).

**Tabella B — posizioni e verifica di coerenza**

| Motore | Posizione (Blue Robotics/PX4: elica di riferimento) | Coerenza con la matrice (dedotta) | Segno PWM effettivo (fattore × DIRECTION) | Verifica in acqua (2026-08-14, L4) |
|---|---|---|---|---|
| 1 | prua dritta (FR), orizzontale vettoriale; elica **CCW** | forward−/lateral+/yaw+: coerente con FR vettoriale (la matrice ArduSub usa fattori unitari; angolo fisico verificato in seguito: 30°) | surge **+**, sway **−**, yaw **−** | ✓ surge (+), sway+ (−), yaw+ (−) |
| 2 | prua sinistra (FL), orizzontale; elica **CCW** | coerente con FL | surge +, sway +, yaw + | ✓ |
| 3 | poppa dritta (RR), orizzontale; elica **CW** | coerente con RR | surge +, sway +, yaw − | ✓ |
| 4 | poppa sinistra (RL), orizzontale; elica **CW** | coerente con RL | surge +, sway −, yaw + | ✓ |
| 5 | prua dritta, verticale; elica **CCW** | roll+ = lato dritto, pitch− = prua → FR | heave +, roll −, pitch + | non verificato (verticali solo in trim ALT_HOLD 1421–1576 µs) |
| 6 | prua sinistra, verticale; elica **CW** | FL | heave −, roll −, pitch − | non verificato |
| 7 | poppa dritta, verticale; elica **CW** | RR | heave −, roll +, pitch + | non verificato |
| 8 | poppa sinistra, verticale; elica **CCW** | RL | heave +, roll −, pitch − | non verificato |

Fonti delle posizioni e delle eliche: PX4 Airframe Reference, "BlueROV2 (Heavy Configuration)" (https://docs.px4.io/main/en/airframes/airframe_reference), che riporta, ad esempio, "motor 1 … bow starboard horizontal … propeller CCW" (L3); guida del retrofit Blue Robotics (eliche 5–8, L3). La verifica storica è in `HOA\docs\real_vehicle_reference\README.md:86-99` (L4).

**Osservazioni**
1. `MOT_n_DIRECTION = −1` esattamente sui motori con elica CCW di riferimento (1, 2, 5, 8) e `+1` su quelli con elica CW (3, 4, 6, 7). Questo è coerente con un montaggio standard Blue Robotics (L2 + L3) ma **non prova** l'elica effettivamente montata.
2. I segni PWM dei motori 1–4 calcolati dalla matrice coincidono con la prova in acqua di agosto: surge+ (+,+,+,+), sway+ (−,+,+,−), yaw+ (−,+,−,+), con risposta di yaw nel verso giusto (+6.4° / −13.7°) (L4).
3. Per i verticali (5–8) nessuna prova dei segni in acqua: il segno "heave +" significa solo che il PWM sale quando si chiede di salire; la direzione fisica della spinta dipende dall'elica e dall'ESC (L5).

**Non determinabile senza ispezione fisica (L5):**
- quale thruster fisico è cablato a quale uscita del Navigator (1..8): servono la lettura delle etichette e il test motori in QGC/Cockpit **in acqua** (vietato in questa fase);
- l'elica (CW/CCW) montata in ciascuna posizione;
- ~~l'angolo reale dei thruster vettoriali (nominale 45°)~~ **[Aggiornamento 2026-11: angolo di montaggio dei quattro orizzontali verificato fisicamente a 30°; non è più una questione aperta.]** I bracci (posizioni x, y, z rispetto al baricentro o all'IMU), che servono per la matrice di allocazione in HoloOcean;
- il modello di thruster ed ESC (es. T200 + Basic ESC) e la tensione di alimentazione dei thruster;
- eventuali zavorre o galleggianti aggiunti (massa e assetto).

---

## 3. Elettronica di bordo

| Componente | Valore | Fonte | Livello |
|---|---|---|---|
| Computer di bordo | Raspberry Pi 4 B, SoC BCM2711 | E17 `/system-information/platform` | L1 |
| OS | Debian GNU/Linux 12, kernel 5.10.33-v7l+, hostname `blueos` | E16 `/system-information/system/info` | L1 |
| RAM | 3.87 GB totali, 0.91 GB usati; swap 100 MB | E19c | L1 |
| Temperatura CPU | 45.8 °C (06:54Z) | E18 | L1 |
| Autopilota | **Navigator** (Blue Robotics), `platform: navigator` (HAL Linux sul Pi) | E05b `/ardupilot-manager/v1.0/board` | L1 |
| Revisione del Navigator | baro di bordo **BMP280** (`BARO1_DEVID=226825` → I2C bus 1, indirizzo 0x76, tipo 0x03 BMP280): dunque revisione **precedente** ad agosto 2025 (le nuove montano il BMP390) | `PARM:71`; SRC `AP_Baro_Backend.h:43`; pagina prodotto Navigator | L2 + L3 |
| IMU | **ICM-20602** (`INS_ACC_ID=INS_GYR_ID=3080714` → SPI, tipo 0x2F); una sola IMU (`INS_ACC2_ID=0`) | `PARM:434,454,417`; SRC `AP_InertialSensor_Backend.h:117` | L2 + L3 |
| Bussola 1 | **MMC5983** (`COMPASS_DEV_ID=1245450` → SPI, tipo 0x13) | `PARM:199`; SRC `AP_Compass_Backend.h:67` | L2 + L3 |
| Bussola 2 | **AK09915** (`COMPASS_DEV_ID2=1379337` → I2C bus 1, 0x0C, tipo 0x15) | `PARM:200`; SRC `AP_Compass_Backend.h:69` | L2 + L3 |
| Orientamento della scheda | `AHRS_ORIENTATION=16` (ROTATION_ROLL_90). Oggi ATTITUDE roll 0.027 rad, pitch −0.067 rad (veicolo quasi in piano) | `PARM:15`; E26 | L2/L1 |
| Calibrazioni | offset di accelerometro, giroscopio e bussola presenti (`INS_ACCOFFS_*`, `COMPASS_OFS_*`, `COMPASS_OFS2_*`); `COMPASS_DEC=0` con `COMPASS_AUTODEC=1` | `PARM:242-250,427-450` | L2 |
| ADC / batteria | `BATT_MONITOR=4` (tensione e corrente analogiche), `BATT_VOLT_PIN=5`, `BATT_CURR_PIN=4`, `BATT_VOLT_MULT=11.0`, `BATT_AMP_PERVLT=37.8788`, `BATT_AMP_OFFSET=0.33`: valori tipici del **Power Sense Module** Blue Robotics. `BATT_CAPACITY=0`, failsafe batteria disattivati (`BATT_FS_*=0`, `BATT_LOW_VOLT=0`) | `PARM:95-114` | L2 (modulo L4/L5) |
| Batteria oggi | 14.83 V, 0.82 A, consumati 239 mAh dal boot | E21/E22 (`SYS_STATUS`, `BATTERY_STATUS`) | L1 |
| Tipo di batteria | 14.8 V compatibile con 4S Li-ion; ad agosto 15.9 V. Modello e capacità **sconosciuti** (potrebbe anche essere un alimentatore da banco) | `HOA\docs\real_vehicle_reference\README.md:70` | L5 |
| Leak | `LEAK1_PIN=27`, `FS_LEAK_ENABLE=1` | `PARM:486-487,367` | L2 |
| Porte seriali Navigator | C=`/dev/ttyS0`, B=`/dev/ttyAMA1`, E=`/dev/ttyAMA2`, F=`/dev/ttyAMA3` | E09 `/ardupilot-manager/v1.0/serials` | L1 |
| Uscite PWM | `BRD_PWM_COUNT=8`, `SCHED_LOOP_RATE=200` Hz | `PARM:118,730` | L2 |
| Tether | Lato PC: "Ethernet 2" Realtek USB FE (RTL8152), 100 Mbps, <TOPSIDE-IP>/24 manuale. Indica una scheda **FXTI / Fathom-X**, ma non lo prova | E42; `RL\docs\00_SYSTEM_INVENTORY.md:29` | L1 (adattatore) / L4 (FXTI) |

---

## 4. Sensori di navigazione

| Sensore | Dettagli | Oggi (cache mavlink2rest, valori in aria) | Livello |
|---|---|---|---|
| **Profondimetro esterno** | `BARO2_DEVID=751153` → I2C bus **6** (`BARO_EXT_BUS=6`), indirizzo **0x76**, tipo 0x0B (classe MS56XX). In ArduSub 4.1.2 il bus esterno viene sondato solo per **MS5837** a 0x76 e Keller LD (`AP_Baro.cpp:664-668`), con conversione per la versione 30BA (`AP_Baro_MS5611.cpp:488`). Quindi è un **MS5837 (Bar30 o Bar02)**; poiché oggi legge ~1027 hPa in aria, valore plausibile con la formula 30BA, è **probabilmente un Bar30** | `SCALED_PRESSURE2` 1027.2 hPa, 19.2 °C | `PARM:74,81`; E28 → L2 + L3 + L1; modello esatto **L5** |
| `BARO_PRIMARY` | 1 (= BARO2, cioè l'esterno) | — | `PARM:84` L2 |
| `BARO_SPEC_GRAV` | 1.0 (acqua dolce) | — | `PARM:86` L2 |
| `BARO2_GND_PRESS` | **97800 Pa**, contro ~102720 Pa misurati oggi (`SENSOR_OFFSETS.raw_press`) → profondità in aria **+0.52 m** (`VFR_HUD.alt=−0.52`). Problema già noto, ancora presente | — | `PARM:75`; E31 (SENSOR_OFFSETS), E35 → L1 |
| Posizione della presa di pressione | operatore ad agosto: sensore **sul fondo del telaio** ("depth ≈ chiglia") | — | `HOA\…\README.md:135-141` L4 |
| Baro interno (Navigator) | BMP280, `SCALED_PRESSURE` 997.3 hPa, 35.1 °C (temperatura interna al contenitore) | — | E27 L1 |
| IMU | `RAW_IMU` zacc ≈ −997 mg, temperatura 37.25 °C | `SCALED_IMU2`: solo magnetometro 2 (acc/gyro = 0) | E25 L1 |
| Stato sensori (`SYS_STATUS`) | presenti, abilitati e sani: gyro, accel, mag, pressione assoluta, AHRS, motori, batteria, prearm; **GPS e VISION_POSITION non sani**; controllo Z/XY disabilitato (modo MANUAL) | — | E21 L1 |
| GPS (via NMEA Injector) | `GPS_TYPE=14` (MAVLink); `GPS_RAW_INT`: NO_FIX, 0 satelliti | — | `PARM:407`; E30m → L1 |
| EKF | `AHRS_EKF_TYPE=3`, `EK3_SRC1_POSXY=3` (GPS), `VELXY=6` (ExtNav), `POSZ=1` (baro), `YAW=1` (bussola); `EKF_STATUS_REPORT.flags=167` (0xA7, const_pos_mode) | — | `PARM:11,321-336`; m2r_vehicles (E31) → L2/L1 |
| DVL | nessuno: `VISO_TYPE=1` ma nessun messaggio VISION_* | — | E21 (VISION non sano) L1; `RL\docs\00_SYSTEM_INVENTORY.md:118` L4 |
| Rangefinder in ArduSub | `RNGFND1_TYPE=0`: i `DISTANCE_SENSOR` del Ping1D vengono ignorati | — | `PARM:718` L2 |

---

## 5. Sonar, posizionamento acustico, GPS

| Dispositivo | Identità | Collegamento | Oggi | Livello |
|---|---|---|---|---|
| **Ping1D** (Blue Robotics) | device_type Ping1D, device_id 1, revisione 2, firmware riportato 1.0.0 (insolito) | `/dev/ttyAMA3` (porta Navigator F, `SERIAL5_PROTOCOL=-1`), bridge UDP **9090**, driver MAVLink attivo (comp 194) | `DISTANCE_SENSOR`: 38.38 m in aria (valore privo di significato), min 0.20 m, max 120 m, PITCH_270, ~2.8 Hz. **Il Ping1D sta trasmettendo** (default all'accensione; nessun comando inviato da noi) | E13, E39 → L1; firmware reale L5 |
| **Cerulean Surveyor 240-16** | MAC **<MAC>** (ARP oggi = MAC registrato in passato), IP **<SONAR-IP>**, porta **62312** (vista dal Ping Service come `UNKNOWN`, discovery di BlueOS); firmware 1.10.4 secondo il log di discovery del 17/09 | Ethernet di bordo | risponde al ping (TTL 32); **nessuna connessione TCP aperta**. Se stia trasmettendo non è determinabile senza contattarla | E07, E13, E42 → L1 (presenza); firmware L4 (`RL\docs\00_SYSTEM_INVENTORY.md:102`) |
| Registrazione Surveyor "~40.7 MB" | `<workstation>/BlueROV2MultimodalRecorder\records\real_sessions\20260915_120346_83f3e317\surveyor_raw.svlog` = **40 707 862 byte**, sessione 2026-09-15 10:03:46–10:05:25Z (`session.json`) | — | — | L2 (confermata) |
| Altre registrazioni svlog | sul ROV (elenco del 2026-10-05): 2025-10-22, 2026-09-10 (fino a 500 MB), 2026-09-15 (3 file, 13.8/16.7/16.8 MB, copiati anche in `BlueROV2MultimodalRecorder\Sonar15-09\2026-09-15\`) | — | non riletti oggi | `BK\blueos_listings\userdata_listing.json` L2 |
| SonarView (extension) | `ceruleansonar.sonarview` **1.14.4**, container `nicknothom/sonarview:1.14.4` in esecuzione (Up 17 min), UI sulla porta 7077 (non aperta) | — | E11, E12 → L1 |
| SonarView (desktop) | **1.14.83** | PC | E43 → L1 |
| **ROVL Mk III** (Cerulean) | transceiver topside: FTDI FT230X 0403:6015, SN `<serial>`, COM10, 115200 8N1 | PC (USB) | **non collegato oggi** (`Present=False`, E05; nessuna porta COM, E43). Transponder sul ROV: modello e posizione sconosciuti | L1 (assenza) / L4 (storico) / L5 (transponder) |
| **GlobalSat BU-353N5** | Prolific PL2303GC 067B:23A3, SN `<serial>`, COM11, NMEA 4800 baud, 1 Hz | PC (USB) | **non collegato oggi** (E05, E43) | L1 (assenza) / L4 (`RL\docs\03_BU353N5_GPS.md:7-11`) |
| NMEA Injector | UDP **27000** → MAVLink comp 220 | BlueOS | configurato | E14 → L1 |

---

## 6. Camera e video

| Voce | Valore | Livello |
|---|---|---|
| Sorgente | "H264 USB Camera: USB Camera", `/dev/video2` (H264 1920×1080) e `/dev/video0` (MJPG 1920×1080, YUYV 800×600); USB `usb-0000:01:00.0-1.2` | L1 (E38, identico al backup) |
| Modello | il nome è quello tipico della Blue Robotics Low-Light HD USB Camera, ma il modulo e l'ottica vanno confermati | L5 |
| Stream | "UDP Stream 0": **H264 1920×1080 @ 30 fps**, RTP verso **udp://<TOPSIDE-IP>:5600**, `running: true` | L1 (E37) |
| SDP | `RL\config\video\bluerov_udp5600_from_blueos.sdp:8-12` (payload 96, profile-level-id 4d4029, packetization-mode 1) | L2 |
| Stream secondario | "Prova1" (redirect, UDP 5601) visto il 2026-08-10/14; **assente** il 2026-10-05 e oggi | L4 → L1 (assente) |
| Intrinseci / FOV / latenza / posa camera-corpo | non misurati | L5 |
| Tilt | servo 16, 1500 µs, ±45° (`MNT_ANGMIN/MAX_TIL=±4500`) | L1/L2 |

---

## 7. Rete e software

| Voce | Valore | Livello |
|---|---|---|
| PC topside | <topside-PC>, Windows 11 Education 10.0.26200; "Ethernet 2" <TOPSIDE-IP>/24 (manuale); seconda interfaccia verso la rete locale | L1 (E03, E42) |
| BlueOS eth0 | **<ROV-IP>** (unmanaged), connesso, 0 disconnessioni; usb0 <ROV-AUX-IP> (server DHCP, scollegato) | L1 (E15) |
| `beacon/ip` | client <TOPSIDE-IP>, interfaccia <ROV-IP>; hostname `blueos`, nome veicolo "BlueROV2" | L1 (E19) |
| MAC BlueOS | <MAC> (ARP) | L1 (E42) |
| Endpoint MAVLink | GCS Client Link udpout <TOPSIDE-IP>:14550 (attivo); GCS Server Link udpin 0.0.0.0:14550 (disattivo); MAVLink2Rest udpout 127.0.0.1:14000; Internal Link tcpin 127.0.0.1:5777; Ping360 Heading udpin 0.0.0.0:14660. Stesso insieme del backup | L1 (E08) |
| Router | MAVLinkRouter | L1 (E10) |
| mavlink2rest | **0.11.23** (sha 3dbdf03…) | L1 (E33) |
| mavlink-camera-manager | 0.2.4 | L4 (`RL\docs\00_SYSTEM_INVENTORY.md:73`; endpoint `/info` non letto oggi) |
| Extension Kraken | Cockpit **v1.0.2**; SonarView **1.14.4**; major_tom 2024-07-29 (agente BlueOS Cloud) | L1 (E11–E12) |
| Servizi web | Cockpit, ArduPilot Manager, MAVLink Camera Manager, **Commander**, Bag Of Holding, System Information, Version Chooser, Ping Service, Beacon, MAVLink2Rest, ttyd, Bridget, Pardal, File Browser, SonarView, Wifi Manager, Kraken, NMEA Injector. *Commander* non compariva nel backup del 2026-10-05 | L1 (E10b) |
| QGroundControl | **4.2.8.0** (exe del 2023-07-14) | L1 (E43) |
| CeruleanTracker | exe con FileVersion 1.0.0.0; versione applicativa 2.10.24 secondo i documenti | L1 / L4 |
| SYSID | `SYSID_THISMAV=1`, `SYSID_MYGCS=255` | L2 (`PARM:889-890`) |
| Altri sistemi MAVLink | in cache c'è anche il **sistema 2, comp 0** (`REMOTE_LOG_BLOCK_STATUS`, 1 Hz): origine non identificata, probabilmente un logger di BlueOS | L1 (osservazione) / L5 (origine) |

### 7.1 Orologi

| Voce | Valore | Livello |
|---|---|---|
| Orologio BlueOS | `unix_time_seconds=1791207371` = **2026-10-05T13:36:11Z**, letto alle 06:54:01Z del PC → BlueOS **indietro di ~7 d 17 h 18 min** (il Pi non ha RTC né NTP). Anche i timestamp di mavlink2rest sono in questa base | L1 (E19b) |
| ArduSub | `SYSTEM_TIME.time_unix_usec=0` (nessuna ora UTC); `time_boot_ms=1 060 773` → boot ArduSub ≈ **06:36:24Z** (ora del PC) | L1 (E34) |
| PC | non verificato contro un riferimento esterno (il 2026-10-05 risultava "Local CMOS Clock", non sincronizzato) | L4 |

---

## 8. Parametri ArduSub rilevanti per la simulazione

Tutti da `PARM` (L2). Sono valori del 2026-10-05: oggi il conteggio (913) è confermato, ma i valori **non** sono stati riletti, perché servirebbe PARAM_REQUEST (vietato).

| Gruppo | Parametri (riga `PARM`) |
|---|---|
| Frame / motori | `FRAME_CONFIG=2` (358); `MOT_PWM_MIN/MAX=1100/1900` (553-554); `MOT_THST_EXPO=0.65` (564); `MOT_THST_HOVER=0.5` (565); `MOT_SPIN_MIN/MAX=0.15/0.95` (562,561); `MOT_FV_CPLNG_K=1.0` (551); `MOT_YAW_HEADROOM=200` (566); `MOT_BAT_*=0` (nessuna compensazione di tensione) (545-549) |
| Assetto (ATC) | `ATC_ANG_RLL/PIT/YAW_P=6.0` (32-34); `ATC_RAT_RLL/PIT_P=0.135, I=0.09, D=0.0036, IMAX=0.444, FLTD/FLTT=30` (40-57); `ATC_RAT_YAW_P=0.18, I=0.018, D=0, IMAX=0.222, FLT=5` (58-66); `ATC_ACCEL_*_MAX=110000` (27-29); `ATC_RATE_Y_MAX=180` (39); `ATC_INPUT_TC=0.15` (35); `ATC_THR_MIX_*=0.1/0.5/0.1` (68-70); `ATC_ANGLE_BOOST=1` (30); `ANGLE_MAX=4500` (22) |
| Posizione (PSC) | `PSC_POSXY_P=0.6` (611); `PSC_POSZ_P=1.0` (612); `PSC_VELXY_P=6.0, I=0.5, D=0` (619,617,613); `PSC_VELZ_P=4.0, I=0` (626,624); `PSC_ACCZ_P=0.5, I=0.1, IMAX=100` (606,604,605); `PSC_JERK_XY=5, PSC_JERK_Z=50` (609-610). Nota: `PSC_POSXY_P`, `PSC_POSZ_P`, `PSC_VELZ_P` sono stati modificati da SonarView il 2026-09-17 (diff in `BK\ardusub\params_DIFF_2026-09-17_vs_2026-10-05.txt`; prima erano 1.0 / 3.0 / 8.0) |
| Pilota | `PILOT_SPEED_UP=100` cm/s, `PILOT_SPEED_DN=0` (= usa PILOT_SPEED_UP), `PILOT_ACCEL_Z=100` (578-581); `JS_GAIN_DEFAULT=0.2`, `JS_GAIN_MIN/MAX=0.1/1.0` (480-485); `THR_DZ=100` (891); `RC1..6_DZ=30`, `RC4_DZ=40` (662-691) |
| Navigazione | `WPNAV_SPEED=100`, `WPNAV_SPEED_UP=250`, `WPNAV_SPEED_DN=150`, `WPNAV_ACCEL=250`, `WPNAV_RADIUS=200`, `WPNAV_RFND_USE=1` (903-911); `LOIT_SPEED=1250` (506); `CIRCLE_RADIUS=1000` (190) |
| Superficie | `SURFACE_DEPTH=-10` cm (888) |
| Failsafe | `FS_GCS_ENABLE=2` (366); `FS_PILOT_INPUT=2`, `FS_PILOT_TIMEOUT=3.0` s (368-369); `FS_LEAK_ENABLE=1` (367); `FS_EKF_ACTION=0`, `FS_EKF_THRESH=0.8` (364-365); `FS_PRESS_ENABLE=0`, `FS_TEMP_ENABLE=0` (370,372); `FS_CRASH_CHECK=0` (363); `FS_TERRAIN_ENAB=0` (374); `ARMING_CHECK=448` (24); `BATT_FS_*=0` (104-105) |
| INS / AHRS | `AHRS_ORIENTATION=16` (15); `AHRS_EKF_TYPE=3` (11); `EK3_ENABLE=1`, `EK2_ENABLE=0` (285,268); `EK3_IMU_MASK=1` (302); `INS_GYRO_FILTER=20`, `INS_ACCEL_FILTER=20` Hz (451,426); `INS_FAST_SAMPLE=1` (436); `INS_HNTCH_ENABLE=0`, `INS_NOTCH_ENABLE=0` (455,461); `AHRS_TRIM_X/Y=0.019/0.073` rad (17-18) |
| Telemetria | `SR0_EXTRA1=10` nel backup (QGC 4.2.8 lo riporta a 20 a ogni connessione: `RL\docs\14_CHANGELOG.md:30`); `SR0_EXTRA2=10`, `SR0_POSITION=3`, `SR0_RAW_SENS=2` (848-855). Frequenze viste oggi in cache: ATTITUDE/VFR_HUD/AHRS2 ~10 Hz; BATTERY_STATUS/SYSTEM_TIME ~3 Hz; SCALED_PRESSURE*/RAW_IMU/SERVO_OUTPUT_RAW/RC_CHANNELS/SYS_STATUS ~2 Hz; HEARTBEAT 1 Hz (L1) |
| Logging | `LOG_DISARMED=0`, `LOG_BITMASK=176126`, `LOG_BACKEND_TYPE=1` (494,493,492) |

---

## 9. Confronto: oggi (L1) contro backup 2026-10-05 (L2)

Confronto byte per byte delle risposte GET salvate in `EV\http\` con `BK\blueos_api\`:

| Endpoint | Esito |
|---|---|
| `version/current`, `bootstrap/current`, `firmware_info`, `vehicle_type`, `firmware_vehicle_type`, `board`, `serials`, `preferred_router`, `kraken/installed_extensions`, `nmea-injector/socks`, `cable-guy/ethernet`, `cable-guy/interfaces`, `beacon/hostname`, `beacon/vehicle_name`, `beacon/ip`, `system/info`, `platform`, `mavlink2rest/info`, `mavlink-camera-manager/v4l` | **identici** (19) |
| `ardupilot-manager/endpoints`, `ping/v1.0/sensors` | stesso contenuto, **solo ordine diverso** |
| `mavlink-camera-manager/streams` | identico tranne l'`id` (UUID rigenerato al boot) |
| `kraken/list_containers` | stesse immagini e stessi image_id; cambia solo l'uptime ("Up 17 minutes" contro "Up 10 days") |
| `helper/web_services` | aggiunto **Commander** (porta 9100), assente nel backup |
| `system/temperature`, `system/memory`, `unix_time_seconds`, `mavlink2rest/vehicles` | valori dinamici (attesi) |

**Affermazioni storiche e loro verifica**

| Affermazione storica | Esito oggi | Livello |
|---|---|---|
| BlueOS 1.3.1 | ✔ confermata (core 1.3.1, sha d47b4b32…; bootstrap 1.3.1) | L1 |
| ArduSub 4.1.2 | ✔ confermata (firmware_info 4.1.2 STABLE; AUTOPILOT_VERSION `flight_sw_version=0x040102FF`, custom "2dd0bb7d" = tag `ArduSub-4.1.2`) | L1 + L3 |
| QGC 4.2.8 | ✔ confermata (exe 4.2.8.0) | L1 |
| Navigator | ✔ confermata (`/board`) | L1 |
| 913 parametri nel backup | ✔ 913 in `.parm`, `.params` e `.json` (indici 0–912, nessun mancante; `.json` = `.parm`); il veicolo oggi riporta `param_count=913`. Il file `…RECONSTRUCTED_2026-09-17…parm` ne ha **904** (prima dell'abilitazione dei `VISO_*`) | L1 + L2 |
| BlueOS su <ROV-IP> | ✔ | L1 |
| Surveyor su <SONAR-IP>:62312 | ✔ IP e MAC confermati con ping/ARP; la porta 62312 è confermata solo indirettamente (discovery del Ping Service, `ethernet_discovery_info`); **nessuna connessione aperta** | L1 (indiretta per la porta) |
| Camera RTP/H.264 su UDP 5600 | ✔ (stream configurato e running; nessun pacchetto ricevuto o decodificato oggi) | L1 (configurazione) |
| Ping1D presente | ✔ (Ping Service + DISTANCE_SENSOR comp 194) | L1 |
| ROVL presente | ✘ **non collegato oggi** (nessun FTDI 0403:6015 presente); uso storico documentato | L1 (assenza) / L4 |
| Registrazione Surveyor ~40.7 MB | ✔ 40 707 862 byte (file locale del recorder, 2026-09-15) | L2 |
| FRAME_CONFIG=2 → Heavy | ✔ coerente con `SERVO1..8_FUNCTION`, con la matrice SRC e con le 8 uscite attive | L1 + L2 + L3 |

---

## 10. Backup: disponibilità e integrità

| Voce | Esito | Livello |
|---|---|---|
| `BK\SHA256SUMS.txt` | 139 voci ricalcolate con `Get-FileHash` (SHA256): **139 OK, 0 errate, 0 mancanti** (E30, `HoloOcean_BlueROV2_Phase1\evidence\sha256_verify_2026-10-13.txt`). Nota: `sha256sum -c` di Git Bash fallisce solo per i fine riga CRLF del file, non per i dati | L1 |
| File presenti ma non elencati in SHA256SUMS | `pc/preflight_bench_2026-10-05.json`, `pc/qgc/QGroundControl.ini.after_step1`, `…after_step2` (creati dopo il manifest) | L1 |
| Parametri | `.parm` 913, `.params` 913 (+4 righe di commento), `.json` 913; `backup_20261005T101049Z.json`: `param_count_reported=913`, `saved=913`, `missing_indices=[]`, disarmato, MANUAL | L2 |
| Storage ArduSub | `BK\blueos_files\configs\ardupilot-manager\firmware\storage\ArduSub.stg` (copia grezza dello storage parametri) | L2 |
| Log | DataFlash `.BIN` (11), tlog del router (2022–2026), `.bin` 2026-09-24 | L2 |
| Backup della SD | **assente** (BlueOS 1.3 non ha un backup completo) | L4 (`RL\docs\13_OPEN_ISSUES.md:25`) |
| Parametri attuali contro backup | **non verificabile** senza scaricare i parametri (vietato). Il conteggio coincide; ad agosto 33 parametri letti in `HOA\docs\real_vehicle_reference\data\mavlink_inventory.json` sono identici al backup | L1 (conteggio) / L4 |

---

## 11. Incognite e verifiche pendenti

| # | Priorità | Incognita / verifica | Come chiuderla | Livello attuale |
|---|---|---|---|---|
| 1 | **P1** | Corrispondenza fisica thruster → uscita Navigator 1..8 ed elica CW/CCW per posizione | ispezione visiva (etichette, eliche) e test motori in acqua con autorizzazione esplicita (fase successiva) | L5 |
| 2 | **P1** | Segni della spinta dei verticali 5–8 (mai verificati in acqua) | prova di heave/roll/pitch in acqua | L5 |
| 3 | **P1** | Parametri correnti uguali al backup? (oggi solo il conteggio 913) | download dei parametri in sola lettura, se autorizzato (PARAM_REQUEST_LIST è un invio MAVLink: escluso in questa fase) | L4 |
| 4 | **P1** | Riferimento di pressione: `BARO2_GND_PRESS=97800` → +0.52 m in aria | calibrazione in superficie prima dell'immersione (è una modifica: non eseguita) | L1 (problema confermato) |
| 5 | **P1** | Il **Ping1D trasmette in aria** e la Surveyor è accesa (stato di ping ignoto) | decisione del team; controllo fisico dell'alimentazione della Surveyor | L1 / L5 |
| 6 | **P1** | Geometria per HoloOcean: posizioni dei thruster, angolo dei vettoriali, baricentro e centro di spinta, massa, galleggiabilità, zavorre | misure fisiche, dati CAD Blue Robotics | L5 |
| 7 | **P1** | Orologio BlueOS indietro di ~7.7 giorni; ArduSub senza UTC | sincronizzare prima di registrare dati da allineare (modifica: non eseguita) | L1 |
| 8 | P2 | Modello del profondimetro (Bar30 contro Bar02) | etichetta del sensore; la lettura di ~1027 hPa in aria indica Bar30 | L5 (inferenza Bar30) |
| 9 | P2 | Presenza di luci (modello e numero), laser sull'uscita 14, gripper sull'uscita 9 | ispezione visiva | L5 |
| 10 | P2 | Batteria (chimica, capacità) e Power Sense Module | etichetta; `BATT_CAPACITY=0` | L5 |
| 11 | P2 | Modello della camera, intrinseci, latenza, posa camera-corpo | calibrazione | L5 |
| 12 | P2 | Transponder ROVL sul ROV (modello, posizione); transceiver non collegato oggi | ispezione, collegamento USB | L5 |
| 13 | P2 | Origine del sistema MAVLink 2 (`REMOTE_LOG_BLOCK_STATUS`) | lettura (GET) della configurazione dei servizi BlueOS | L5 |
| 14 | P3 | Firmware reale del Ping1D (riportato 1.0.0) | Ping Viewer o pagina Ping di BlueOS (richiede interrogare il dispositivo) | L4 |
| 15 | P3 | Firmware Surveyor 1.10.4 (dal log del 17/09) | solo con connessione autorizzata | L4 |
| 16 | P3 | `mavlink-camera-manager` 0.2.4 (endpoint `/info` non letto oggi: nel backup aveva risposto 201) | GET di `/mavlink-camera-manager/info` dopo averne verificato l'assenza di effetti | L4 |
| 17 | P3 | Servizio *Commander* comparso nell'elenco web dopo il backup | GET di confronto al prossimo boot | L1 (osservato) |
| 18 | P3 | Ordine dei thruster dell'agente BlueROV2 di HoloOcean contro l'ordine ArduSub (tabella A) | analisi del sorgente HoloOcean (fase 2) | non analizzato |

---

## 12. Evidenze grezze (per il manifest JSON)

*Section omitted from the public copy: it indexes raw private evidence (device responses, file paths, hashes) kept outside the repository.*

