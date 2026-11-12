# Fase 1 — Relazione conclusiva
## Integrazione HoloOcean ↔ BlueROV2 tramite ArduSub SITL

Data: 2026-10-13 · Cartella: `<workstation>/HoloOcean_BlueROV2_Phase1` (Desktop rilevato via `[Environment]::GetFolderPath('Desktop')`)
Natura dell'attività: ricognizione, verifica tecnica, analisi del codice e documentazione. **Nessun middleware/bridge implementato.**

### Dichiarazione di sicurezza
Il BlueROV2 (fuori dall'acqua) è stato raggiunto solo con **ICMP e HTTP GET** (BlueOS REST e cache MAVLink2Rest), 2026-10-13 06:52–06:54 UTC, dopo che l'utente ha collegato il tether (alle 06:50Z l'adattatore "Ethernet 2" non era collegato). Nessun messaggio MAVLink inviato, nessun armamento, comando motore, cambio modo/parametri, riavvio, calibrazione, connessione TCP alla Surveyor o attivazione sonar. Nessun repository modificato (solo comandi `git --no-optional-locks` di lettura). Unico test eseguito: `pytest -q` del Middleware (42 passed, solo mock). Il motore HoloOcean **non** è stato avviato. Evidenze grezze in `evidence/`.

---

## 1. Documenti prodotti

| # | File | Contenuto |
|---|---|---|
| 1 | `hardware-inventory.it.md` | inventario del veicolo con livelli L1–L5, mappa motori, confronto live vs backup |
| 2 | `software-inventory.it.md` | 17 repository/cartelle, commit, stato, dipendenze, codice riutilizzabile |
| 3 | `sitl-interface-spec.it.md` | protocollo SITL JSON (4.1.2 → master), MAVLink, WSL2 |
| 4 | `holoocean-physics-audit.it.md` | agente BlueROV2, frame, timestep, patch drag, tabelle di corrispondenza, procedure V1–V11 |
| 5 | `sensor-data-audit.it.md` | dataset reali, timestamp, rate, qualità, latenze, utilizzabilità |
| 6 | `simulator-comparison.it.md` | HoloOcean vs Stonefish vs Gazebo, licenze, rischi |
| 7 | `phase2-architecture-proposal.it.md` | architettura proposta, moduli, porte, clock, esperimento di profondità, test, rischi |
| 8 | `open-items.it.md` | 31 verifiche pendenti con priorità e istruzioni |
| 9 | `phase1_evidence_manifest.json (privato)` | manifest: documenti, repository, binari, file chiave, evidenze live, dati, URL |
| 10 | `phase1-final-report.it.md` | questo documento |
| — | `evidence/live_2026-10-13/`, `evidence/sha256_verify_2026-10-13.txt` | risposte GET/ping registrate oggi e verifica hash del backup |

Esiste anche un precedente report PDF interno (12/10, 7 pagine) dichiaratamente **solo documentale** («verifica fisica e accesso diretto ai repository locali ancora pendenti»). Questa relazione lo supera con verifiche dirette.

---

## 2. Cosa sappiamo con certezza

1. **Il veicolo è un BlueROV2 Heavy (8 thruster)**: `FRAME_CONFIG=2` (VECTORED_6DOF), `SERVO1..8_FUNCTION=33..40`, uscite 1–8 a 1500 µs (L1/L2; `hardware-inventory.it.md` §2.1).
2. **Stack software reale**: BlueOS 1.3.1, ArduSub 4.1.2 STABLE `2dd0bb7d` (= tag `ArduSub-4.1.2`), Navigator su Raspberry Pi 4 B, `SCHED_LOOP_RATE=200` (L1/L2).
3. **Matrice motori VECTORED_6DOF** confermata sul sorgente al tag 4.1.2 (`AP_Motors6DOF.cpp:156-166`): motori 1–4 vettoriali orizzontali (FR, FL, RR, RL), 5–8 verticali (FR, FL, RR, RL); `MOT_n_DIRECTION` = −1 su 1, 2, 5, 8 (`hardware-inventory.it.md` §2.3).
4. **HoloOcean installato**: client 2.3.0 (env `holoocean_joystick`, Python 3.10.20), pacchetto Ocean 2.3.0, binario ufficiale SHA256 `8c206c9c…` **non patchato**; sorgente engine = tag v2.3.0 `49e70552` (`holoocean-physics-audit.it.md` §1).
5. **Agente BlueROV2 HoloOcean**: scheme 0 = 8 forze in **N**, clamp ±28.75 N; indici 0–3 verticali, 4–7 vettoriali; 11.5 kg, neutro; drag solo quadratico isotropo (Cd 0.8, A 0.45 m²) + damping Unreal; niente massa aggiunta; frame client NWU/FLU in metri, rotazioni in gradi (`holoocean-physics-audit.it.md` §2–3).
6. **Timestep**: passo fisico = `min(1/tps, 1/30 s)`: a 20 Hz il rapporto fisico/clock è 0.6667, da 30 a 200 Hz è 1.0 — **conferma il dato storico** e ne delimita la validità (< 30 Hz) (`holoocean-physics-audit.it.md` §4, da `Battery/docs/holoocean_timestep_report.md`).
7. **Patch del drag**: bug di unità (N applicati come kg·cm/s² → drag ×0.01); fix `DragForce *= UEUnitsPerMeter` in `HolodeckBuoyantAgent.cpp:91-95`; patch in `Battery/patches/holoocean-2.3-drag-units.patch`; build patchata `<HOB>\packages\patched-after\…\Holodeck.exe` (SHA256 `4d6c2d46…`, 2026-10-07); rapporto drag applicato/atteso 0.0099999997 → 1.0000000074 su 153 casi.
8. **Tutti i risultati di simulazione esistenti** (Middleware v1, BlueROV2SimMatch, HoloObstacleAvoidance) sono stati ottenuti col **binario non patchato** a 30 Hz.
9. **Il Middleware v1** esistente (`Desktop\Middleware`, senza Git) offre un'API comune `VehicleBackend` con backend HoloOcean (servo PI proprio, scheme 0, 30 Hz), BlueROV **solo lettura** (interlock prima di ogni I/O) e Replay; 42 test passano oggi. **Non usa ArduSub in simulazione.**
10. **Non esiste un bridge HoloOcean↔ArduPilot pubblico** (`simulator-comparison.it.md` §7).

## 3. Verificato oggi sul dispositivo (L1, 2026-10-13 06:52–06:54Z)

| Voce | Risultato | Confronto con dato storico |
|---|---|---|
| BlueOS | core 1.3.1 | confermato |
| ArduSub | 4.1.2 STABLE `2dd0bb7d`, Navigator | confermato |
| Parametri | il veicolo riporta 913 parametri | conteggio confermato; valori non riletti (richiede invio MAVLink) |
| Rete | <ROV-IP> e <SONAR-IP> rispondono (0 % perdita) | confermato |
| Surveyor | IP/MAC confermati; porta 62312 solo indiretta | parzialmente confermato |
| Camera | H264 1920×1080@30 → `udp://<TOPSIDE-IP>:5600` | confermato; lo stream "Prova1" su 5601 di agosto non esiste più |
| Ping1D | `/dev/ttyAMA3`, DISTANCE_SENSOR attivo — **trasmette in aria** (preesistente) | confermato |
| ROVL / GPS BU-353N5 | non collegati oggi | da riconfermare |
| Stato | disarmato, MANUAL | — |
| Profondità in aria | +0.52 m (`BARO2_GND_PRESS=97800` obsoleto) | problema confermato |
| Orologio BlueOS | 2026-10-05 13:36Z, ~7.7 giorni indietro; ArduSub senza UTC | nuovo |
| Altro | servizio "Commander" nuovo in BlueOS; un sistema MAVLink 2 non spiegato | nuovo |
| Sensori (da device ID) | IMU ICM-20602, bussole MMC5983 + AK09915, baro interno BMP280, profondimetro esterno MS5837 (probabile Bar30) | inferenza L2/L3 |

Integrità backup: `SHA256SUMS.txt` **139/139 OK**; i tre file del backup hanno 913 parametri coerenti; il file ricostruito del 2026-09-17 ne ha 904. Delle risposte GET, 19 identiche byte a byte al backup del 2026-10-05; le altre differiscono solo per ordine, UUID, uptime o telemetria.

## 4. Cosa deriva dai dati precedenti (L2/L4)

- **Immersioni reali registrate**: 10/09 (5.6–6.7 m e 0.5–1.9 m) e 15/09 (4.2–5.8 m) da sottotitoli Cockpit; prove in piscina 2026-08-14/16 (segni PWM di surge/sway/yaw dei motori 1–4 coerenti con la matrice); tlog `00100` + BIN `00000016` del 2025-11-17 con uscite non neutre e profondità fino a 0.49 m; tlog QGC del 2026-06-18 armati 39 s e 246 s (`sensor-data-audit.it.md` §1).
- **87 log del veicolo (1.62 GB) mai scaricati**, inclusi BIN 26–28 e tlog 00151–00164 delle immersioni di settembre: il backup ha conservato solo file < ~2 MB.
- **Registrazione Surveyor "40,7 MB"** = `BlueROV2MultimodalRecorder\records\real_sessions\20260915_120346_83f3e317\surveyor_raw.svlog` (40 707 862 B, 98.9 s; 162 checksum errati, ~32 % ping persi). Il file da 78 MB sul Desktop è un'altra registrazione (10/09). `recording.zip` è un duplicato esatto di `Sonar15-09`+`Video15-09`.
- **Problemi del Multimodal Recorder** (documentati solo in un rapporto diagnostico interno del 15/09, non pubblicato): ritardo worker→GUI mediana 539 ms (p95 1.90 s); beamforming 83.5 ms + rendering 119 ms per update, calcolato due volte; coda illimitata di frame da 6.22 MB; crescita memoria (`RADAR_PRE_LEAK_64`) e blocco (`AppHangB1`); rete non satura (29 Mbit/s picco). Mitigazioni presenti nel codice attuale.
- **Latenze**: nessuna latenza fisica dei sensori è mai stata misurata; tutte le misure sono lato ricevitore/visualizzazione o offset di clock (Surveyor −7.08 s costante = offset; video Cockpit con buchi 0.8–1.5 s dovuti al browser; MANUAL_CONTROL→PWM 53–89 ms misurato nel progetto ObstacleAvoidance).
- **Rumore stimato (a secco o statico)**: pressione esterna ~2–3 mm; IMU DataFlash 25 Hz gyro σ 0.0004–0.0007 rad/s, accel σ 0.005–0.017 m/s².
- **Inutilizzabili oggi**: Ping1D (file vuoti, letture in aria 37–97 m), ROVL (solo posizioni GPS fittizie), video con timestamp per frame.

## 5. Verificato nella documentazione / sorgenti pubbliche (L3)

- **Interfaccia JSON ArduPilot** (`sitl-interface-spec.it.md`): SITL invia su UDP **9002** (+10·N) il pacchetto `<HHI16H>` (magic 18458, frame_rate, frame_count, pwm[16]); il simulatore risponde all'indirizzo sorgente con una riga JSON `"\n{...}\n"`: `timestamp`, `imu.gyro` (rad/s FRD), `imu.accel_body` (forza specifica, ≈[0,0,−9.81] a riposo), `position` (m NED), `velocity` (m/s NED), `attitude` o `quaternion`. **Lockstep**: un passo fisico per pacchetto; reinvio dello stesso frame dopo ~1 s senza risposta; timestamp all'indietro = reset.
- **Differenze di versione**: 4.1.2 ha il backend JSON ma ridotto (16 canali, `position` obbligatorio, parser `sscanf` che richiede JSON compatto senza spazi, nessun `no_time_sync`); 32 canali da 4.5; `no_lockstep`, `rc`, `battery` solo 4.7.2/master.
- **Sensori in SITL**: IMU/baro/bussola/GPS sono sintetizzati da SITL dallo stato inviato; la profondità viene dal baro SITL calcolato da `position.z` (densità 1.024 contro `BARO_SPEC_GRAV=1.0` del reale → ~2.4 %); il GPS SITL funziona sott'acqua → `SIM_GPS_DISABLE=1` per parità.
- **MAVLink identico SITL/reale**: modi STABILIZE 0, ALT_HOLD 2, AUTO 3, GUIDED 4, SURFACE 9, POSHOLD 16, MANUAL 19; MANUAL_CONTROL accettato solo da `SYSID_MYGCS` (255).
- **Stonefish** (`bvibhav/stonefish_bluerov2`, Apache-2.0, `6448383`): nodo ROS 2 ~200 righe, **senza lockstep**, PWM lineare, conversione frame parziale, niente gripper → riferimento di design, non codice. **Articolo MDPI** JMSE 14(2):198 (Pérez-Edo et al.): Stonefish+ArduSub SITL+gripper Newton per recupero oggetti, validazione solo percettiva, ~170 ms di latenza reale. **Gazebo** (`ardupilot_gazebo`, LGPL-3.0, `082a0fe`): miglior modello concettuale di lockstep e gestione frame.

## 6. Riconciliazioni tra analisi

| Tema | Conflitto | Conclusione |
|---|---|---|
| Unità comandi scheme 0 | `simulator-comparison.it.md` [NV] vs sorgente engine | **newton**, clamp ±28.75 N (`BlueROV2.h:11-13`) — prevale il sorgente |
| Backend JSON in 4.1.2 | `software-inventory.it.md` §8.1 lo dubitava | **esiste** al tag `ArduSub-4.1.2`, ridotto (nota aggiunta al documento) |
| Matrice motori | in `holoocean-physics-audit.it.md` §7.2 "da memoria" | **confermata** sul sorgente 4.1.2 da `hardware-inventory.it.md` §2.3; posizioni coerenti (M1→t4 FS, M2→t5 FP, M3→t7 BS, M4→t6 BP, M5→t0, M6→t1, M7→t3, M8→t2); **segni** ancora ipotesi |
| Loop rate ArduSub | 400 Hz citato | 400 = default SITL; **200** sul veicolo reale (`params_20261005T101049Z.parm`) |
| Registrazione 40.7 MB | attribuita al Desktop | è `surveyor_raw.svlog` del 15/09; il file Desktop è 78 MB del 10/09 |

## 7. Cosa manca (dettaglio in `open-items.it.md`)

- **Bloccanti per la Fase 2 (P0)**: SITL ArduSub 4.1.2 non ancora compilato/presente (nessun sorgente ArduPilot, Docker assente, distro WSL non ispezionate); spazio disco ~15 GB liberi; rete WSL2↔Windows da configurare/provare; uso della build HoloOcean patchata da confermare con `check_holoocean_backend.py`.
- **Simulazione (P1, eseguibili offline)**: segni del mapping canali→thruster; IMU per socket; RTF a 100/200 Hz; sleep dei corpi Unreal sulla build patchata; profondità SITL; clamp 28.75 N.
- **Veicolo (richiedono autorizzazione/acqua)**: download dei log mancanti; lettura parametri correnti; cablaggio fisico ed eliche; segni dei verticali; calibrazione pressione; sincronizzazione orologi; geometria/massa reali; Ping1D/ROVL/latenza camera in acqua.
- **Validazione fisica**: la patch corregge solo le unità; Cd, area, damping, inerzia e massa aggiunta non sono validati rispetto al veicolo reale.

## 8. Decisioni tecniche raccomandate

1. **ArduSub è il controllore unico** in sim e sul reale; l'autonomia parla solo MAVLink. HoloOcean è solo il modello fisico (scheme 0, forze in N). Il servo PI del Middleware v1 resta come baseline, non nel percorso principale.
2. **SITL ArduSub 4.1.2** (`2dd0bb7d`, frame `vectored_6dof`) in **WSL2 mirrored**, parametri di controllo copiati dal backup 913 parametri; bridge scritto per funzionare anche con 4.5/4.7.
3. **Bridge Python lockstep** su Windows nell'env `holoocean_joystick`: UDP 9002, un tick HoloOcean per frame nuovo, risposta all'ultimo stato per i duplicati, JSON compatto, PWM 1000/0 = motore spento.
4. **Build HoloOcean patchata** (hash registrato in ogni run), **dt = 1/200 s** con `SCHED_LOOP_RATE=200` (parità con il Navigator); fallback 100 Hz.
5. **Tempo**: tempo simulato = tick × dt; l'autonomia usa `time_boot_ms`, mai il wall clock.
6. **Sicurezza**: il nuovo backend ArduSub del Middleware accetta solo target SITL locali; `SYSID_THISMAV` diverso nel SITL; nessuna porta 14550 condivisa col reale.
7. **Primo esperimento**: E0 stato statico → E1 MOTOR_TEST per canale (valida il mapping) → E2 MANUAL neutro → E3 ALT_HOLD 1.0 → 2.0 m → E4 100 vs 200 Hz (`phase2-architecture-proposal.it.md` §7).
8. **Prima della Fase 3 (sim-to-real)**: scaricare i log delle immersioni di settembre e usarli per identificare i parametri idrodinamici; non usare `VFR_HUD.alt` come profondità.

## 9. Valutazione dello stato della Fase 1

**Stato: sostanzialmente completata sul piano documentale e di verifica non attuante; non chiusa del tutto.**

- Le 7 attività richieste sono state svolte e documentate con fonti verificabili (file:riga, commit, URL, output diagnostici in `evidence/`).
- Tutti i dati storici elencati sono stati verificati: 11 su 13 confermati (6 sul dispositivo, 5 in file locali); Surveyor:62312 confermato solo parzialmente; ROVL non verificabile oggi (non collegato) — `phase1_evidence_manifest.json (privato)` → `historical_claims`.
- Restano aperte verifiche che **per costruzione** non potevano essere eseguite in questa fase (acqua, invio MAVLink, download voluminosi, avvio del motore HoloOcean per le procedure V1–V11). Sono pianificate con istruzioni in `open-items.it.md`.
- Il rischio tecnico principale per la Fase 2 non è il protocollo (ben definito e verificato su 4.1.2), ma: (a) compilazione di SITL 4.1.2 in WSL2 con poco spazio disco, (b) segni del mapping motori, (c) prestazioni di HoloOcean a 200 Hz, (d) fedeltà del modello idrodinamico.

La Fase 2 può iniziare appena chiusi i P0 (T01–T04).
