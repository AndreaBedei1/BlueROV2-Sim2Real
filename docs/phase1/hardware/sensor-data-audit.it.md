# Audit dei dati sensoriali reali esistenti — BlueROV2 (ArduSub 4.1.2)

Fase 1, sola ricognizione. Data dell'audit: 2026-10-13.

Vincoli rispettati:
- nessun file modificato, spostato o cancellato fuori da `HoloOcean_BlueROV2_Phase1`;
- nessuna estrazione di archivi: `recording.zip` è stato solo elencato e confrontato via CRC;
- nessuna connessione al ROV (<ROV-subnet>) e nessun sonar attivato;
- nessuna operazione git di scrittura.

Le analisi sono state eseguite in sola lettura con script privati (pymavlink, ffprobe); script e output grezzi non sono pubblicati.

Legenda: **MISURATO** = calcolato in questo audit dai dati. **DOCUMENTATO** = riportato da una fonte, con file e riga. **INFERITO** = deduzione, non verificata.

---

## 0. Sintesi

| # | Risultato chiave |
|---|---|
| 1 | Esistono **immersioni reali** (Cockpit `.ass`, profondità ArduSub):<br>– **2026-09-10**: 5.6–6.7 m e 0.5–1.9 m;<br>– **2026-09-15**: 4.2–5.8 m.<br>Su disco però ci sono solo video Cockpit e svlog della Surveyor. **Telemetria e DataFlash di quelle immersioni non sono stati scaricati** (restano sul ROV). |
| 2 | Sul veicolo restano **87 log non scaricati (1.62 GB)**: `ardupilot_logs_listing.json`.<br>– BIN da `00000006` a `00000028` (tranne 12/13/16/19/23/24);<br>– tutti i tlog grandi, compresi quelli del 10/09 (00151, 00152, 00154), del 15/09 (00156–00164) e del 17/09 (00166). |
| 3 | La "registrazione Surveyor precedente di circa 40.7 MB" è `BlueROV2MultimodalRecorder\records\real_sessions\20260915_120346_83f3e317\surveyor_raw.svlog`: **40 707 862 B**, 98.9 s, solo Surveyor. Non è il file da 78 MB sul Desktop. |
| 4 | Sono documentati con misure i problemi del recorder: backlog, crescita di memoria, doppio beamforming, ritardi della pipeline sonar. La fonte **non è un file di documentazione del repo**: è un rapporto diagnostico interno del 15/09/2026 (non pubblicato). Il codice attuale contiene le mitigazioni (§3). |
| 5 | La latenza **fisica** dei sensori non è mai stata misurata.<br>– Tutte le latenze note sono della catena di ricezione o visualizzazione, oppure sono offset di orologio.<br>– La differenza tra ora di ricezione PC e ora UTC del dispositivo Surveyor è **costante, circa 7.0 s** (mediana 7.08 s, pendenza −0.0005 s/s): è un offset di orologio, non un backlog crescente. |
| 6 | **Ping1D**: nessun profilo registrato (tutti i `ping1d.jsonl` hanno 0 byte). Esistono solo `DISTANCE_SENSOR` in aria o in piscina, con valori non validi: 37–97 m, confidenza 1–26 %. |
| 7 | **ROVL**: nessun dato reale registrato. C'è solo una fixture sintetica nei test e posizioni GPS fittizie (0.00009°, 0.00009°) nei tlog QGC del 17/09. |
| 8 | Per caratterizzare thruster e mapping, i dati utili con output non neutri sono:<br>– tlog `00100` + BIN `00000016` (17/11/2025, immersione ≤0.5 m);<br>– tlog QGC del 18/06/2026, armato e in acqua (SP2 fino a 1126 hPa);<br>– i motion test in piscina del 14/08/2026. |

---

## 1. Inventario dei dataset

### 1.1 Surveyor 240-16 (`.svlog`)

**Formato (MISURATO dai byte di header e dal parsing completo)**
- Flusso di pacchetti **Blue Robotics Ping Protocol**: `"BR"`, payload_len u16 LE, msg_id u16, src, dst, payload, checksum u16 (somma dei byte).
- Il primo pacchetto è `msg_id 10` con JSON di sessione: `session_id`, `session_uptime`, `session_devices` (url `tcp://<SONAR-IP>:62312`, `product_id mbes24016`, mount tutti a 0), `sonarlink_version 1.14.4`, `timestamp` ISO UTC dell'host e altri campi.
- Messaggi presenti:
  - 504 attitude (up-vector, `utc_msec`, `pwr_up_msec`);
  - 3009 channel IQ, 8 pacchetti × 2 canali = 16 canali;
  - 3010 end-ping (range);
  - 3012 ATOF (`utc_msec`, ping number, punti);
  - 14, 0, 3042, 4 (controllo/ack; il significato non è stato verificato).
- Riferimenti nel codice: `BlueROV2MultimodalRecorder\docs\DATA_FORMAT.md:84-98` e `processing.py:25-43`.
- Ufficialmente il formato non ha un parser pubblico: "needs its own parser", `RoverLago\docs\research\logging_video_ping_notes.md:612-616`.

**Timestamp**
- Non c'è un timestamp per pacchetto lato host.
- Ogni ping porta `utc_msec` dell'**orologio del dispositivo**. SonarView invia al dispositivo l'ora UTC dell'host ("Sending UTC time to Device"): per la sessione a bordo l'host è BlueOS, a −3.77 s dal PC (`11_DATA_LOGGING.md:61-65`).
- C'è inoltre `pwr_up_msec`, monotono dall'accensione del dispositivo.
- Il JSON di testa porta l'ISO UTC dell'host.

| File | Byte | Data (UTC) | Contenuto | Rate MISURATO | Qualità MISURATA |
|---|---:|---|---|---|---|
| `Desktop\2026-09-10-12-19(1).svlog` | 78 107 012 | 2026-09-10 12:19:26 → 12:30:21 | 3 pacchetti meta (offset 0, 32 038 484, 32 049 976): file con più segmenti. 758 ping ATOF + channel (8/8 pacchetti per ping); range 0.29–5.29 m | ATOF/attitude **10 Hz** (dt mediano 100 ms) | 0 checksum errati, 0 ping mancanti. **Gap di 579.8 s** (12:19:57 → 12:29:37): circa 75 s di ping effettivi in 11 min. Il suffisso "(1)" indica una copia scaricata |
| `BlueROV2MultimodalRecorder\Sonar15-09\2026-09-15\2026-09-15-09-53.svlog` | 13 780 897 | 2026-09-15 09:53:30 → 09:53:57 | 134 ping, range 1–20.4 m, channel data | **5.00 Hz** (dt 200 ms, max 201 ms) | integro, 0 mancanti |
| `…\2026-09-15-09-54.svlog` | 16 737 611 | 09:54:30 → 09:55:02 | 162 ping | 5.00 Hz | 1 set di canali incompleto (5/8), 0 ping mancanti |
| `…\2026-09-15-09-55.svlog` | 16 783 217 | 09:55:48 → 09:56:20 | 163 ping | 5.00 Hz | integro |
| `…\records\real_sessions\20260915_120346_83f3e317\surveyor_raw.svlog` (**il "40.7 MB"**) | 40 707 862 | 2026-09-15 10:03:46 → 10:05:25 (host) | Scritto dal recorder desktop (meta `recorder: "BlueROV2 Multimodal Recorder"`), 3167 pacchetti 3009, 352 ATOF, 340 end-ping, 335 attitude | dispositivo 5 Hz; ricevuti in media 3.56 Hz | **162 checksum errati (tutti su 3009)**, **142 ping mancanti** nella sequenza ATOF, **214 su 475 ping con set di canali incompleto**. Le perdite sono già nel raw, quindi non sono imputabili alla sola GUI |
| `RoverLago\backups\…\SonarView\Surveyor 240-16\2026-09-10-13-14.svlog` | 221 501 | 2026-09-10 13:14:32 → 13:15:46 | Solo ATOF + attitude (nessun 3009: `channel_data_policy=disable`) | 10.0 Hz | integro |
| `…\Surveyor 240-16\2025-10-22-12-16.svlog` | 1 062 | 2025-10-22 | solo metadata + 3 msg id 0 | – | nessun dato |
| `…\records\real_sessions\2026091[67]_*\surveyor_raw.svlog` (6 file) | 478 ciascuno | 16–17/09 | solo il pacchetto meta | – | sessioni vuote |
| `BlueROV2MultimodalRecorder\recording.zip` | 111 174 201 | 2026-09-15 18:59 | **Duplicato esatto** dei 3 svlog `Sonar15-09` e dei 3 video `Video15-09` (verificato con CRC32 per file) | – | nessun dato nuovo |

**Riconciliazione delle dimensioni**

| Fonte | Valore | Stato |
|---|---|---|
| "circa 40.7 MB" | il raw della sessione recorder del 15/09 | coincide al byte |
| File sul Desktop | 78.1 MB | è un'altra registrazione (10/09, SonarView) |
| `11_DATA_LOGGING.md:75` ("500 MB in ~10 min il 2026-09-10") | – | **non verificabile** con i file locali |

Il backup BlueOS contiene solo 2 svlog. Secondo `04_SONARVIEW.md:15` sul ROV esistono registrazioni dal 2025-10-22 al 2026-09-15. Il backup di `userdata/SonarView` è quindi parziale (INFERITO: filtro per dimensione).

**Throughput MISURATO**
- svlog con channel data a 5 Hz: circa 0.51 MB/s, cioè 1.8 GB/h. Il 3009 vale il 99.8 % dei byte.
- A 10 Hz (file del 10/09): circa 1.0 MB/s durante il ping.
- Solo ATOF a 10 Hz: circa 3 kB/s.

### 1.2 Video camera

| File | Byte | Data | Formato | Frame MISURATI | Timestamp | Qualità |
|---|---:|---|---|---|---|---|
| `BlueROV2MultimodalRecorder\Video15-09\Cockpit (Sep 15, 2026 - 11꞉53꞉40 GMT+2) #e51b393d.mkv` | 33 950 942 | 2026-09-15 09:53:40 UTC | H.264 Main 1920×1080, Matroska, encoder `Chrome`, timebase 1 ms, **VFR** | 818 frame / 26.08 s, **31.3 fps** medi, dt mediano 32 ms, max **836 ms** | Ora di inizio solo nel nome file (ora locale, 1 s). PTS relativi | 1 gap > 200 ms; profondità in `.ass` 5.71–5.77 m, ALT_HOLD |
| `…#0471681d.mkv` (11:54:39) | 34 603 880 | 09:54:39 UTC | idem | 826 / 29.0 s, **28.4 fps**, max **900 ms** | idem | 5 gap > 200 ms; 4.99–5.05 m |
| `…#51381cf4.mkv` (11:55:57) | 35 589 224 | 09:55:57 UTC | idem | 856 / 28.4 s, **30.1 fps**, max 834 ms | idem | 4 gap; 4.18–4.23 m |
| `HoloObstacleAvoidance\RealImageData\Cockpit (Sep 10 … 14꞉48꞉47) #0522d3be.mkv` | 109 643 127 | 2026-09-10 12:48:47 UTC | idem, nominale 20 fps | 2641 / 140.5 s, **18.8 fps**, max **1549 ms** | idem | 12 gap; 5.58–6.66 m |
| `…14꞉51꞉37 #d6f5f965.mkv` | 22 054 055 | 12:51:37 UTC | idem | 529 / 30.3 s, **17.5 fps**, max 1500 ms | idem | 3 gap; 5.83–5.94 m |
| `…14꞉57꞉11 #8d0fd205.mkv` | 365 614 762 | 12:57:11 UTC | idem | 8734 / 337 s, **25.9 fps**, max 1516 ms | idem | 65 gap; 0.54–1.92 m |

Note sui video Cockpit:
- Ogni `.mkv` ha un `.ass` (sottotitoli "Cockpit Subtitle Telemetry") a **1 Hz**, con tempo relativo all'inizio. Contiene pitch, roll, heading, modo, **Depth**, tensione e corrente della batteria.
- È la registrazione **lato browser** (Cockpit/WebRTC, MediaRecorder di Chrome) dello stream già ricevuto e decodificato. Il frame rate variabile e i buchi fino a 1.5 s nascono nella catena di ricezione e rendering, **non dalla camera**. Lo stream sorgente è RTP H264 1080p30 (`11_DATA_LOGGING.md:19,82`).
- Video sorgente con PTS RTP o timestamp per frame: **nessuno**. I `camera_timestamps.csv` del recorder contengono solo l'intestazione (212 B) e non esiste nessun `camera_rgb.mkv`.

Altri video:
- `HoloObstacleAvoidance\experiments\real\missions\*\onboard_detector.mp4` e `rehearsal\*\onboard.mp4`: MPEG-4 960×540 a 8–10 fps, overlay del detector. Sono output elaborati, non grezzi, della piscina del 14–15/08.

### 1.3 Telemetria MAVLink: tlog del veicolo (BlueOS mavlink-router)

Percorso: `RoverLago\backups\2026-10-05_pre-change\blueos_files\configs\ardupilot-manager\logs\`. I file scaricati sono 29: 26 tlog e 3 `.bin` da 0 byte.

**Timestamp**
- Formato QGC: 8 byte big-endian in µs, **orologio del Raspberry/BlueOS** (`logging_video_ping_notes.md:355,410`).
- BlueOS è a −3.767 ± 0.034 s dal PC; misura del 2026-10-05, `11_DATA_LOGGING.md:61,109`.
- Il Pi non ha RTC: `00000-2022-01-28` e `00006` mostrano **salti di orologio** dentro il file (durata apparente 102 636 880 s e 11 188 858 s).

**Rate MISURATI** (tutti i tlog scaricati)

| Messaggio | Rate |
|---|---|
| ATTITUDE | 10–20 Hz |
| VFR_HUD | 3–10 Hz |
| SYSTEM_TIME | 3 Hz |
| SCALED_PRESSURE / SCALED_PRESSURE2 | **2 Hz** |
| RAW_IMU / SCALED_IMU2 | **2 Hz** |
| SERVO_OUTPUT_RAW | **2 Hz** (8.3 Hz in un solo file) |
| DISTANCE_SENSOR | 2.8–3.1 Hz, oppure 5.6 o 9.8 Hz in alcuni file |

- Perdita pacchetti (buchi di `seq` di sys1/comp1): **0–0.73 %**.
- Nei tlog di novembre 2025 e successivi `DISTANCE_SENSOR` non ha `signal_quality` (frame MAVLink1 o troncati).

| tlog | Byte | Inizio (orologio Pi) | Durata | Armato | Output non neutri (\|pwm−1500\|>25, ch1–6) | Pressione SP2 (hPa) | Note |
|---|---:|---|---:|---|---|---|---|
| 00008-2025-09-06_11-43-51 | 1 638 007 | 2025-09-06 11:43:51 | 345 s | **sì, 69 s** | **24/138** | 994–999 | banco (BIN 00000001: motor test) |
| 00010-2025-09-06_11-49-40 | 1 956 222 | 11:49:40 | 446 s | 3 s | 0/6 | 993–996 | |
| 00053-2025-10-08_15-29-53 | 461 135 | 2025-10-08 | 124 s | 2 s | 0/4 | 974–980 | |
| 00092-2025-11-13_11-12-53 | 1 984 229 | 2025-11-13 11:12 | 388 s | **sì, 98 s** | 23/196 (gain test) | 955–994 | |
| **00100-2025-11-17_13-39-19** | 1 435 274 | 2025-11-17 13:39 | 282 s | **sì, 99 s** (MANUAL → ALT_HOLD) | **188/198** | **956–1026** (+35 hPa mentre è armato) | probabile immersione parziale ≤0.5 m. Coincide con BIN 00000016 |
| altri 21 tlog (2022 → 2026-08-14) | 0.1–2.0 MB | vari | 28–554 s | no | – | 955–1010, stabile | banco/disarmato. **VFR_HUD.alt è costante** in tutti: non usarlo come profondità |

**Log del veicolo NON scaricati** (`blueos_listings\ardupilot_logs_listing.json`):
- **87 file, 1 619 287 180 B** in totale;
- BIN `00000006–11, 14, 15, 17, 18, 20–22, 25–28`. In particolare `00000026` (2026-09-10 13:42, 30.4 MB) e `00000027/28` (2026-09-15 09:23 / 10:07, 9.7 e 15.3 MB) coprono le immersioni con video e Surveyor;
- tlog `00151–00172`: 2026-08-16 → 2026-10-05, 10–103 MB ciascuno, tra cui `00164-2026-09-15_09-18-33.tlog` da **103 MB**.

Il backup ha scaricato solo i file piccoli (INFERITO: filtro per dimensione intorno a 2 MB).

**File `.bin` da 0 byte**
- File: `logs\00169-2026-09-24_11-18-25.bin`, `00171-…16-18-31.bin`, `00173-…21-18-33.bin`.
- Sono vuoti anche sul veicolo (listing: size 0).
- Sono creati dal logger di BlueOS a ogni rotazione da 5 h, senza flusso DataFlash via MAVLink (`logging_video_ping_notes.md:638-650`). Non sono corrotti: sono vuoti per costruzione.

### 1.4 DataFlash ArduSub (`firmware\logs\*.BIN`, scaricati: 11)

**Timestamp e logging**
- `TimeUS` dall'avvio, **nessuna ora UTC** (`time_unix_usec=0`, `11_DATA_LOGGING.md:62`).
- Si registra solo da armato (`LOG_DISARMED=0`, `11_DATA_LOGGING.md:14`).

**Rate MISURATI**

| Messaggio | Rate |
|---|---|
| IMU | **25 Hz** (campionamento 1 kHz, "fast sampling 8 kHz/1 kHz") |
| BARO[0], BARO[1] | 10 Hz |
| ATT, RCOU, RCIN, CTUN, XKF1 | 10 Hz |
| GPS (MAV) | 0.25 Hz |

**BARO**
- BARO[0] è il barometro interno, in aria. BARO[1] è il sensore esterno ad acqua (pressione con passo 10 Pa).

| BIN | Byte | Data (listing) | Durata | RCOU non neutri | BARO[1] alt (m) | Eventi |
|---|---:|---|---:|---|---|---|
| 00000001 | 973 569 | 2025-09-06 11:49 | 185 s | 121/836 | −0.016…0.003 | Motor test |
| 00000002/03 | 0.23 MB | 2025-09-06 / 10-08 | 118 / 95 s | 0 | ±0.01 | |
| 00000004 | 1 749 991 | 2025-10-22 11:29 | 382 s | 110/1542 | −0.014…0.002 | gain test |
| 00000005 | 651 484 | 2025-10-22 14:03 | 3250 s (logging rado) | 0 | −0.245…0.006 | baro calibration |
| 00000012 | 1 210 333 | 2025-11-13 11:19 | 317 s | 106/1127 | −0.16…−0.02 | heartbeat GCS lost |
| 00000013 | 566 536 | 2025-11-17 11:58 | 5182 s | 0 | −0.20…−0.19 | |
| **00000016** | 1 272 989 | 2025-11-17 13:43 | 219 s | **937/1142** | **−0.489…0.037** | `NOT_SURFACED`/`SURFACED` (EV 164/163); ALT_HOLD |
| 00000019 | 533 597 | 2026-06-18 18:06 | 390 s | 31/465 | −0.62…−0.51 | GCS lost |
| 00000023 / 24 | 0.26 MB | 2026-08-15 / 08-16 | 49 / 4003 s | 25/199 | −0.60…−0.49 | GCS lost |

### 1.5 Telemetria QGC (lato PC): `<user-home>/Documents\QGroundControl\Telemetry\` (12 tlog)

Timestamp: µs sull'**orologio del PC**, che non è sincronizzato (Local CMOS Clock, `11_DATA_LOGGING.md:60`).

| tlog | Byte | Durata | Armato | Non neutri | SP2 (hPa) | Note |
|---|---:|---:|---|---|---|---|
| 2026-04-08 12-42-49 | 11 334 194 | 1995 s | no | – | 1022–1026 | banco |
| 2026-06-16 ×3 | 0.25–5.5 MB | 45–1097 s | no | – | (SP2 assente) | banco |
| **2026-06-18 11-15-57** | 398 276 | 64 s | **39 s** | **78/79** | **1036–1122** | **in acqua** |
| **2026-06-18 18-05-18** | 3 379 160 | 606 s | **246 s** | **336/492** | **1072–1126**, T 31.6–31.9 °C | **in acqua, circa 0.6–1.1 m** (INFERITO dalla pressione; manca il riferimento in superficie) |
| 2026-09-17 ×3 | 0.34–2.0 MB | 64–416 s | no | – | 979–981 | GPS_RAW_INT fix 3 alla posizione **fittizia (0.00009°, 0.00009°)**: è l'incidente ROVL "Sim Data" (`05_ROVL_MKIII.md:87,91`) |
| 2026-10-05 ×3 | 0.13–0.21 MB | 20–37 s | no | – | 1028–1030 | banco |

Rate: ATTITUDE 18–20 Hz, GLOBAL_POSITION_INT 3 Hz, SP2/RAW_IMU/SERVO 2 Hz, DISTANCE_SENSOR circa 2.8 Hz. Perdita di sequenza 0–0.18 %.

### 1.6 Altri dataset reali lato PC

| Dataset | Byte | Data | Contenuto e timestamp | Rate MISURATI | Stato |
|---|---:|---|---|---|---|
| `HoloObstacleAvoidance\experiments\real\motion_tests\*.jsonl` (13 file) | 40 k–408 k | 2026-08-14 15:56–16:08 (piscina) | MAVLink via PC. `t` = ora Unix di ricezione sul PC (float) + `time_boot_ms`. Sequenze: static, lights, surge±, sway±, yaw±, step_surge, step_yaw, con `cmd` (power, dur, mode ALT_HOLD) | ATTITUDE/VFR/SERVO/SP2 **12–15 Hz**, RAW_IMU **22–24 Hz** (dove presente), DISTANCE_SENSOR 2–3 Hz | SERVO non neutri in surge/sway/yaw (18–79 campioni). SP2 1004–1021 hPa (vicino alla superficie). Ping1D **non valido** (37–97 m, confidenza 1–26 %). README: **"PILOT/CALIBRATION ONLY"**. Il manifest segnala 46 artefatti tracciati da git ma assenti dal working tree (`experiments\real\README.md`) |
| `…\anchor_dataset\session_20260814_16*\telemetry.jsonl` (3) | 1.2–2.1 MB | 2026-08-14 | idem, 42–77 s | idem | **SERVO non neutri 558–1012 campioni**: utili per il mapping |
| `…\quick_runs`, `rehearsal_runs`, `perception_recovery` | piccoli | 2026-08-15/16 | `cmd_trace` (comandi), ground truth overhead, percezione | – | derivati o di comando; `rehearsal` ha `sync_markers.jsonl` |
| `BlueROV2SimMatch\dataset\real\runs\real_K*` (9 run) | 12–28 kB | (sorgente 2026-08-16) | solo `cmd_vel` + `perception`; `meta.json`: "per-message arrival times were not recorded", traiettoria **not_measured** | – | derivato da `HoloObstacleAvoidance\quick_runs`; nessun sensore grezzo |
| `Middleware\results\validation\real_passive\` | 24 kB | 2026-10-05 | mavlink2rest `:6040`, 2 Hz, 5 campioni, `allow_actuation=false` | – | profondità a secco **0.511–0.515 m**: offset noto "+0.53 m fuori dall'acqua" (`07_PING1D_AND_ALTITUDE.md:7`) |
| `RovCode\output\teleop_session_20260219_*` (4) e `session_20260218_*` (15, con `rosbag2.db3` 0.1–1 MB) | fino a 8.5 MB | 2026-02-18/19 | `ts_unix_ns` = ricezione sul PC | ATTITUDE 10–42 Hz, SP2/RAW_IMU 2–4 Hz | banco, mai armato. **Timestamp duplicati a batch** (es. 451 unici su 539 ATTITUDE). Copia identica in `<copia locale>/RovCode` |
| `BlueROV2MultimodalRecorder\records\real_sessions\20260916_* / 20260917_*` (6) | header | 16–17/09 | sessioni di circa 2 ms (start → stop), nessuna connessione | – | **inutilizzabili** (`ping1d.jsonl` e `diagnostics.jsonl` da 0 B) |
| `BlueROVDepthDemo\output\depth_demo.log` | 3 985 | 2026-09-24 | solo log dei comandi (gripper) | – | nessun dato sensoriale |
| `HoloObstacleAvoidance\logs` | – | – | log di simulazione (sim_server, zenoh) | – | non reali |

---

## 2. Timestamp e sincronizzazione tra stream

| Stream | Orologio | Monotono? | UTC? | Come si allinea |
|---|---|---|---|---|
| tlog BlueOS | Pi (BlueOS) | no (salti senza RTC) | circa UTC, −3.77 s dal PC | offset misurato a inizio e fine sessione (procedura di `mission_session.py`, mai applicata ai dati storici) |
| tlog QGC / HOA JSONL / RovCode | PC, alla ricezione | no (wall clock) | ora locale del PC non sincronizzata | include latenza di rete (circa 10 ms, `11_DATA_LOGGING.md:62`) e il batching del client |
| DataFlash | `TimeUS` dall'avvio | sì | no | `SYSTEM_TIME` o `timesync_*.csv` (mai registrati nei dati storici) |
| `DISTANCE_SENSOR` | `time_boot_ms` dall'avvio del **driver ping** sul Pi | – | no | usare l'ora di ricezione (`11_DATA_LOGGING.md:102`) |
| svlog | `utc_msec` del dispositivo, impostato dall'host SonarView, + `pwr_up_msec` | pwr_up sì | sì (con l'errore dell'orologio dell'host) | nessun timestamp di ricezione per pacchetto. Per la sessione recorder c'è `host_utc_ns`/`host_monotonic_ns` per ping in `surveyor_pings.jsonl` |
| Video Cockpit | PTS relativi in ms, VFR | sì | no | inizio solo nel nome file (1 s). `.ass` a 1 Hz relativo. **Nessun marker di sincronizzazione** |
| Recorder (nuovo formato) | `monotonic_ns` + `utc_ns` dell'host per campione; PTS/DTS separati | sì | sì (orologio PC) | matching per monotonic (`docs/ARCHITECTURE.md:93-95` nel repo). **Nessuna sessione reale con dati in questo formato** |

**Allineamento possibile oggi (INFERITO)**

15/09:
- Sonar15-09 (09:53:30 / 09:54:30 / 09:55:48 UTC, orologio BlueOS) e video Cockpit (09:53:40 / 09:54:39 / 09:55:57 UTC, orologio PC) si sovrappongono con risoluzione circa ±1–4 s. Pesano l'offset BlueOS–PC e la risoluzione di 1 s del nome file.
- Profondità disponibile solo a 1 Hz nel `.ass`. Telemetria piena e DataFlash sono **sul ROV** (tlog 00158–00164, BIN 27–28).

10/09:
- svlog alle 12:19–12:30 UTC; video alle 12:48–13:03 UTC; svlog di backup alle 13:14 UTC.
- **Non c'è sovrapposizione temporale** tra il svlog sul Desktop e i video.

---

## 3. Problemi noti del recorder: dove sono documentati e cosa è stato misurato

**Nessun file `.md` del repository `BlueROV2MultimodalRecorder` descrive i problemi.**
- I documenti attuali (`README.md`, `docs\ARCHITECTURE.md:17-22`) descrivono solo l'architettura già corretta: code bounded, mailbox capacità 1, writer bufferizzati.
- I termini "backlog" e "memory growth" non compaiono in nessun file della documentazione, né in RoverLago o Holoocean2-3.

La **fonte primaria con misure** è il report diagnostico del 15/09/2026:
- rapporto diagnostico interno (non pubblicato);
- i riferimenti `app.py:NNN` citati nel report puntano alla versione **precedente** al refactor `4164f4f` (16/09). L'`app.py` attuale ha 1.7 kB.

| Problema | Misura riportata (DOCUMENTATO, rapporto interno 15/09) | Tipo di latenza | Mitigazione nel codice attuale |
|---|---|---|---|
| Backlog della coda GUI illimitata (`queue.Queue()` senza limite, `_poll_events` svuotava tutto) | frame BGR 1920×1080 = **6.22 MB**; fino a circa **187 MB/s** teorici di frame in coda a 30 fps | programma (GUI) | `runtime.py:67-68` (`LatestValueMailbox` sostituisce invece di accumulare); `runtime.py:126`; `live_app.py:24-26`; `offline_app.py:89` |
| Crescita della memoria e hang | Windows: **`RADAR_PRE_LEAK_64` alle 11:15:20**, **`AppHangB1` + Application Hang 1002 alle 11:45:23** (15/09); nessun crash dump o access violation | programma | idem; metriche RSS e heartbeat GUI (`offline_app.py:2056,2124`) |
| Doppio beamforming | calcolato nel worker **e** nella GUI. Beamforming mediano **83.5 ms**, rendering PIL **119 ms**, totale circa **202.5 ms** per aggiornamento: a 5 ping/s supera un core | programma | `surveyor_pipeline.py:56-57` ("at most one fan matrix per complete ping"); `:79-82` "Surveyor beamforming backlog; preview work skipped"; percorso replay con un solo calcolo (`offline_app.py:307`); beamformer vettorializzato (`processing.py:402-418`) |
| Ritardo della pipeline sonar (solo sonar) | dalla ricezione nel worker alla scrittura dell'evento GUI: mediana **539 ms**, p95 **1.90 s**, max **2.62 s** | programma | pipeline decoder/beamformer separata, code bounded |
| Flush sotto lock | oltre **4000 flush in circa 99 s** | programma (I/O) | writer asincroni batch (`runtime.py:224-424`); `surveyor_raw_writer_latency_ms` 0.15–0.38 ms nelle sessioni del 16–17/09, però senza dati |
| Banda di rete | picco circa **29 Mbit/s** (7.25 MB in 2 s); raw Surveyor in media **3.29 Mbit/s**; camera 5600 **1.12 Mbit/s**; link 1 Gbit/s. Conclusione: **non saturazione fisica** | rete | – |
| Ping incompleti | 338 ping elaborati, 237 completi, **101 NOT AVAILABLE**, 99 salti (massimo 6 ping) | acquisizione | Surveyor rimosso dal percorso live (commit `4164f4f`, `DATA_FORMAT.md:3-7`) |
| Leak secondario | `atof_by_ping` senza eviction | programma | non verificato nel codice attuale |
| Metadato errato | `ping_rate_hz = 240000` (frequenza acustica, non ping rate); `surveyor.tx = LOCKED` | metadato | confermato **ancora presente** nei dati (§1.1: il campo ATOF "ping_hz" vale 240 000) |

**Misure di questo audit sulla stessa sessione (`20260915_120346_83f3e317`)**
- `host_utc − device_utc` per ping: primo 6.998 s, mediana **7.076 s**, minimo 6.964 s, massimo 7.682 s, ultimo 7.032 s.
- La pendenza lineare è **−0.0005 s/s**: **non cresce**, quindi nel file scritto non c'è backlog cumulativo. È un **offset costante tra orologi**.
- Il dispositivo prende l'UTC dall'host SonarView, cioè BlueOS. BlueOS era a circa −3.8 s dal PC (misura del 05/10). Il residuo di circa 3.3 s **non è attribuibile** senza l'offset del 15/09: può essere offset dell'orologio dispositivo o host, buffering TCP iniziale, o tutti e due.
- Inter-arrivo all'host: mediana 286 ms contro 200 ms del dispositivo. 338 ping ricevuti contro circa 494 emessi: **circa 32 % di ping persi** prima della scrittura.

**Recorder attuale (16–17/09)**
- `README.md:120-121` e `DATA_FORMAT.md:3-5` dicono che il desktop non crea `surveyor_raw.svlog`. Le sessioni del 16–17/09 però lo creano (478 B, solo meta). È una **incongruenza** tra documentazione e codice.

---

## 4. Latenze: fisiche contro introdotte dal software

| Voce | Valore | Fonte | Tipo |
|---|---|---|---|
| Latenza fisica Surveyor (ping → dato) | **non misurata**. Il periodo di ping è 200 ms a 5 Hz e 100 ms a 10 Hz | MISURATO (periodo) | fisica, ignota |
| Latenza camera glass-to-glass | **non misurata**; protocollo proposto: orologio al ms ripreso dalla camera | `logging_video_ping_notes.md:247` | – |
| Latenza Ping1D | **non misurata** | – | – |
| Latenza ROVL | aggiornamento a 1 Hz (specifica); latenza non misurata | `05_ROVL_MKIII.md:16` | fisica |
| Rete MAVLink PC↔Pi | circa 10 ms | `11_DATA_LOGGING.md:62` | trasporto |
| Offset orologio BlueOS−PC | −3.767 ± 0.034 s (2026-10-05) | `11_DATA_LOGGING.md:61,109` | **orologio** (non latenza) |
| `host − device` Surveyor (sessione recorder) | 7.08 s costante | MISURATO | orologio + ignoto |
| Worker → GUI (sonar) | 539 ms mediana, 1.90 s p95, 2.62 s max | rapporto interno 15/09 | **programma** |
| Beamforming + render | 83.5 + 119 ms | rapporto interno 15/09 | **programma** |
| Buchi nei frame Cockpit | dt massimo 0.83–1.55 s, fps effettivi 17.5–31 | MISURATO | **ricezione/visualizzazione (browser)** |
| Network caching VLC | 1000 ms default (UNVERIFIED) | `logging_video_ping_notes.md:160` | visualizzazione |
| Ritardo GPS nell'EKF | 0.2 s di default (`GPS_DELAY_MS=0`); limite EKF 250 ms | `research\ardusub_notes.md:61,65` | stima, filtro |
| Latenza EKF su fondali in pendenza (forum) | circa 200 ms | `research\cerulean_notes.md:170-172` | filtro |
| Timestamp duplicati a batch (RovCode) | 451 unici su 539 | MISURATO | client PC |

---

## 5. Rumore e qualità dei sensori (da dati esistenti, tutti fuori dall'acqua o in acqua bassa)

| Sensore | Stima MISURATA | Condizione | Limite |
|---|---|---|---|
| Profondità (BARO[1], DataFlash 10 Hz) | rumore da differenza prima / √2: **2.1–3.4 mm** (BIN 01–04, 13, 19, 23, 24). Risoluzione 10 Pa ≈ 1 mm | statico, in aria o fermo | la pressione assoluta e la temperatura dipendono dalla calibrazione a terra. Offset +0.53 m a secco nel sistema attuale |
| Profondità (SCALED_PRESSURE2, tlog 2 Hz) | rumore 0.20–0.25 hPa ≈ 2.0–2.5 mm | statico | 2 Hz: inadatto a dinamiche |
| IMU (DataFlash IMU[0], 25 Hz) | gyro std **0.0004–0.0007 rad/s**, acc std **0.005–0.017 m/s²** (BIN 02, 03, 13) | a riposo, disarmato | log a 25 Hz già filtrato: non è un rumore raw a 1 kHz. Manca una registrazione lunga per Allan variance |
| IMU (RAW_IMU, HOA 22–24 Hz) | std statico in LSB raw: acc circa 0.4–0.6, gyro circa 0.4–0.6 (QGC 17/09) | a riposo | unità raw (mg, mrad/s); 2 Hz nei tlog |
| Ping1D | **nessun dato valido**: in aria 0–100 m casuali; in piscina 37–97 m con confidenza 1–26 % | – | nessun profilo `profile_data` registrato |
| Surveyor | 5 o 10 Hz stabili (jitter dt ≤1 ms); 0–12 punti ATOF per ping; range 1–20.4 m o 0.29–5.29 m | in acqua (10/09, 15/09) | attitude e georeferenziazione non disponibili (mount 0, nessuna posa) |
| Camera | VFR 17.5–31 fps nella registrazione Cockpit, buchi fino a 1.5 s | in acqua | nessun video sorgente con PTS RTP. Nessuna calibrazione intrinseca trovata |
| ROVL | – | – | nessun dato |

---

## 6. Utilizzabilità

| Scopo | Utilizzabile oggi | Dati | Cosa manca |
|---|---|---|---|
| Rumore del sensore di profondità | **parziale** | BIN BARO[1] statici; tlog SP2 | sessione dedicata a profondità costante in acqua, ≥10 min, logging ≥10 Hz con `LOG_DISARMED` o armato |
| Rumore IMU | **parziale** | BIN IMU 25 Hz a riposo | registrazione raw ad alta frequenza (`INS_LOG_BAT_*`) di durata ≥1 h per Allan variance |
| Ping1D | **no** | – | registrazione `profile_data` in acqua con fondo noto |
| Camera | **parziale** (qualitativa) | Cockpit mkv in acqua 10/09 e 15/09 | stream sorgente con timestamp per frame (recorder `camera_timestamps.csv`), calibrazione intrinseca, misura glass-to-glass |
| Surveyor | **sì per il formato e il rate**; **no per la georeferenziazione** | svlog 15/09 (5 Hz, channel) e 10/09 (10 Hz) | mount misurati, posa sincronizzata (tlog e BIN del 15/09 da scaricare), timestamp di ricezione |
| ROVL | **no** | – | qualsiasi `$USRTH` reale |
| Mapping e caratterizzazione thruster | **parziale** | tlog 00100 + BIN 00000016; QGC 2026-06-18; HOA motion_tests e anchor_dataset (SERVO a 12–15 Hz, PILOT) | step singoli per motore a profondità costante, con RCOU a 10 Hz o più e risposta IMU/ATT; posizione e corrente |
| Confronto sim-vs-real (dinamica) | **limitato** | HOA motion_tests (surge, sway, yaw step; ATTITUDE 15 Hz) | ground truth della posa (overhead disponibile solo per alcuni run), condizioni documentate, dati non PILOT |
| Confronto sim-vs-real (sonar/camera) | **limitato** | svlog 15/09 + Cockpit 15/09 sovrapposti (±s) | posa e profondità sincronizzate ad alta frequenza (scaricare `00164`/`00160` tlog e BIN 27/28), geometria della scena |

---

## 7. Evidenze per il manifest JSON

*Section omitted from the public copy: it indexes raw private evidence (device responses, file paths, hashes) kept outside the repository.*

## 8. Verifiche pendenti

1. **Scaricare dal ROV** i log mancanti (87 file, 1.62 GB), con priorità:
   - BIN `00000026` (10/09), `00000027`, `00000028` (15/09);
   - tlog `00151`, `00152`, `00154` (10/09) e `00156–00164` (15/09).

   Servono profondità, assetto e RCOU sincronizzati con svlog e video. È un'azione sul veicolo e richiede autorizzazione.
2. **Recuperare gli svlog completi** in `/userdata/SonarView/Surveyor 240-16/` sul ROV:
   - il backup ne contiene 2 su un periodo dichiarato 2025-10-22 → 2026-09-15;
   - verificare il file da circa 500 MB del 10/09 citato in `11_DATA_LOGGING.md:75`.
3. Ricostruire l'**offset BlueOS−PC del 10/09 e del 15/09**: per esempio `SYSTEM_TIME` nei tlog di bordo contro QGC, se esiste una sovrapposizione. Senza questo non si separa il residuo di circa 3.3 s del Surveyor.
4. Capire l'origine dei **162 checksum errati nei 3009** del raw del recorder: scrittura concorrente del vecchio `app.py`, oppure stream TCP. Il codice pre-refactor è solo nella storia git (`c13cb21`/`9394c82`), da leggere in sola lettura.
5. Verificare se **SonarView mette nel svlog una marca temporale di ricezione** (msg id 0/14/3042/4 non decodificati).
6. Ripristinare dalla storia git i **46 artefatti mancanti** di `HoloObstacleAvoidance\experiments\real`. Il README lo indica come restore, che è una scrittura: chiedere autorizzazione.
7. Confermare la **profondità reale nei tlog QGC del 18/06** (SP2 1072–1126 hPa): manca la pressione di riferimento in superficie.
8. Misurare la **latenza glass-to-glass della camera** e la latenza della ricezione di Cockpit (orologio al ms ripreso).
9. Registrare **Ping1D con profilo** e **ROVL `$USRTH`** in acqua: oggi nessun dato valido.
10. Verificare nel codice attuale l'eviction di `atof_by_ping` e correggere `ping_rate_hz=240000`. Allineare `DATA_FORMAT.md` al fatto che le sessioni creano ancora un `surveyor_raw.svlog` di 478 B.
11. Verificare perché `VFR_HUD.alt` è costante in tutti i tlog, compresi quelli armati in acqua. Prima di usarlo come profondità nel confronto sim-vs-real, controllare `GLOBAL_POSITION_INT.relative_alt`.
