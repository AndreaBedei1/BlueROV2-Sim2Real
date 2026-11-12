# Fase 2 — Architettura proposta del primo collegamento ArduSub SITL ↔ HoloOcean

Data: 2026-10-13. Documento **solo progettuale**: nessun codice del bridge è stato scritto in Fase 1.
Fonti di riferimento in questa cartella: `sitl-interface-spec.it.md` (protocollo), `holoocean-physics-audit.it.md` (fisica, frame, patch drag), `hardware-inventory.it.md` (veicolo reale, mapping motori), `software-inventory.it.md` (repository, Middleware v1), `simulator-comparison.it.md` (Stonefish/Gazebo), `sensor-data-audit.it.md` (dati reali).

---

## 1. Obiettivo e principio guida

Gli algoritmi di autonomia devono parlare **una sola interfaccia: MAVLink verso ArduSub**. In simulazione l'ArduSub è il SITL (stesso firmware, stessi parametri di controllo); sul mezzo reale è l'ArduSub 4.1.2 sul Navigator. HoloOcean diventa soltanto il **modello fisico** (sostituisce l'idrodinamica di `SIM_Submarine`), collegato al SITL tramite l'interfaccia JSON ufficiale.

Conseguenze dirette sullo stato attuale:

- Il servo PI/allocazione privato di `Middleware/src/rov_middleware/backends/holoocean.py:13-51` **non deve stare nel percorso principale**: mixer, stabilizzazione e depth hold li fa ArduSub (come già previsto in `Middleware/docs/middleware_design.md` §5). Il backend HoloOcean diretto resta utile come baseline/regressione.
- Non usare il control scheme 1 (PID di posa) né il 2 (accelerazioni, disabilita la fisica idrostatica) di HoloOcean: si usa **scheme 0 (8 forze in N)** — `holoocean-physics-audit.it.md` §2–3.
- Il protocollo SITL↔simulatore (UDP/JSON, interno) è distinto dal protocollo MAVLink (comando/telemetria), che è **identico** tra SITL e veicolo reale — `sitl-interface-spec.it.md` §9.

## 2. Architettura dei processi

```text
 Windows 11 (host)                                           WSL2 Ubuntu-24.04 (mirrored networking)
 ┌──────────────────────────────────────────────┐           ┌───────────────────────────────────────┐
 │ P1  HoloOcean engine (Holodeck.exe, Ocean)    │           │ P3  ArduSub SITL (4.1.2, 2dd0bb7d)    │
 │     build PATCHATA drag (<HOB>\...after)     │           │     frame vectored_6dof, -I0          │
 │          ▲ shared memory / semafori (lockstep)│           │     --model JSON:127.0.0.1            │
 │ P2  holo_sitl_bridge (Python 3.10,            │  UDP 9002 │   servo pkt <HHI16H> 40 B  ──────────►│
 │     env holoocean_joystick)                   │◄──────────┤   (SITL → bridge, porta effimera)     │
 │     - server UDP 9002                         │  JSON line│                                       │
 │     - PWM→N, frame FLU/NWU→FRD/NED            │──────────►│   reply "\n{...}\n" (bridge → SITL)   │
 │     - 1 tick HoloOcean per frame_count nuovo  │           │   MAVLink TCP 5760 (SERIAL0)          │
 │     - logger CSV/JSONL + manifest             │           └──────────────┬────────────────────────┘
 └──────────────────────────────────────────────┘                          │ TCP 5760 (o MAVProxy → UDP 14551)
 ┌──────────────────────────────────────────────┐                          │
 │ P4  Autonomia / Middleware "ArduSubBackend"   │◄─────────────────────────┘
 │     pymavlink, stessa API VehicleBackend       │   sul reale: UDP/TCP verso <ROV-IP> (Fase futura, solo con autorizzazione)
 │ P5  QGroundControl (opzionale, UDP 14550)      │
 └──────────────────────────────────────────────┘
```

Scelte motivate:

| Decisione | Scelta | Motivazione / fonte |
|---|---|---|
| Versione SITL | **ArduSub 4.1.2 (tag `ArduSub-4.1.2`, `2dd0bb7d`)**, bridge compatibile anche con 4.5/4.7 | parità con il veicolo (`hardware-inventory.it.md`); differenze JSON tra versioni in `sitl-interface-spec.it.md` §11. Rischio: build di 4.1 su Ubuntu 24.04 non verificata → fallback container Ubuntu 20.04/22.04 |
| Host SITL | WSL2 (distro `Ubuntu-24.04` già presente, non ispezionata) in **mirrored networking**, tutto su 127.0.0.1 | `sitl-interface-spec.it.md` §12; con NAT serve IP host variabile + regola firewall UDP 9002 |
| Binario HoloOcean | **build patchata** `<HOB>\packages\patched-after\...\Holodeck.exe` (SHA256 `4d6c2d46…`) caricata con `Battery/tools/holoocean_backend.py:51-78` (`make_env(binary=…)`) | il binario installato `%LOCALAPPDATA%\holoocean\2.3.0` (SHA256 `8c206c9c…`) ha drag ×0.01 (`holoocean-physics-audit.it.md` §5). Registrare sempre l'hash del binario nel manifest del run |
| Lockstep | **1 tick HoloOcean per ogni frame_count nuovo**; duplicati → risposta con ultimo stato senza avanzare | `sitl-interface-spec.it.md` §5: SITL reinvia lo stesso frame dopo ~1 s senza risposta |
| Passo fisico | **dt = 1/200 s** (ticks_per_sec=200) come obiettivo; 100 Hz come fallback | passo Unreal = `min(1/tps, 1/30)` → ok a 200 Hz; errore di integrazione drag −15 % a 100 Hz, −168 % a 30 Hz (`holoocean-physics-audit.it.md` §4); `SCHED_LOOP_RATE` del Navigator 4.1 = 200 (`sitl-interface-spec.it.md` §6) |
| Loop ArduSub SITL | `SCHED_LOOP_RATE` = rate fisico del bridge (200) | parità con il reale; evita frame JSON ripetuti |
| Tempo reale | **non richiesto**: lockstep → il tempo simulato è coerente anche se RTF < 1 | costo misurato ~22.5 ms/step (Battery) → a 200 Hz RTF stimato ≈ 0.2 (da misurare, V-T1) |

## 3. Moduli software da realizzare (nuovo progetto, es. `Desktop\HoloSitlBridge`, non dentro repository esistenti)

| Modulo | Responsabilità | Riuso |
|---|---|---|
| `sitl_protocol.py` | parsing pacchetto servo (`<HHI16H>` magic 18458; opz. `<HHI32H>` magic 29569 se ≥4.5); serializzazione JSON compatta `separators=(',',':')`, wrapping `"\n{...}\n"`, rifiuto NaN | da specifica, codice nuovo (evita copie GPL dagli esempi ArduPilot) |
| `lockstep_server.py` | socket UDP 9002, `recvfrom` → risposta all'indirizzo sorgente; gestione frame_count (nuovo / duplicato / salto / reset); timeout e statistiche | concetti da `ardupilot_gazebo` (LGPL-3: solo design, non codice) |
| `actuation.py` | PWM → comando normalizzato `u=(pwm-1500)/400` con deadband; **1000 o 0 = motore spento**; u → N con curva T200 (tabella Blue Robotics, tensione configurabile); clamp ±28.75 N del motore HoloOcean registrato come saturazione | tabella mapping in `holoocean-physics-audit.it.md` §7 |
| `motor_map.py` | canale ArduSub (M1…M8, VECTORED_6DOF) → indice thruster HoloOcean (0…7) con segno; **tabella configurabile + test**; ipotesi iniziale `M1→t4(−) M2→t5(−) M3→t7(+) M4→t6(+) M5→t0(−) M6→t1(−) M7→t3(−) M8→t2(−)` da validare (V-M1) | geometria da `BlueROV2.h/.cpp` e `Middleware/.../holoocean.py:24-28` |
| `frames.py` | NWU/FLU → NED/FRD: `S=diag(1,-1,-1)`; `p_ned=S p_nwu` (origine = home, superficie z=0); `v_ned=S v_nwu`; `R_ned=S R S`; quaternion [w,x,y,z]; gyro/accel IMU in body: FLU→FRD = (x,−y,−z) | `Middleware/src/rov_middleware/frames.py`, `holoocean-physics-audit.it.md` §3 |
| `holo_adapter.py` | scenario BlueROV2 con sensori su socket `COM`: PoseSensor, VelocitySensor, IMUSensor (specific force con gravità), opz. DepthSensor/DVL per confronto; `env.step(forces)` 1 tick; workaround sleep Unreal (dither ±0.02 N, **senza teleport** durante il collegamento SITL perché genera picchi IMU) | `Middleware` backend, `BlueROV2SimMatch/src/sim_server/vehicle_dynamics.py` |
| `state_packager.py` | costruisce JSON: `timestamp=tick·dt` (strettamente crescente), `imu.gyro`, `imu.accel_body` (≈[0,0,−9.81] a riposo), `position`, `velocity`, `quaternion` | — |
| `runlog.py` | CSV/JSONL per tick: frame_count, PWM[16], forze N, stato ground truth, JSON inviato, tempi wall; manifest con hash binario, versione SITL, parametri | `Middleware/src/rov_middleware/runlog.py` |
| `ardusub_backend.py` (Middleware) | nuovo backend che implementa `VehicleBackend` via pymavlink: heartbeat GCS, mode, arm (**solo con target SITL**), depth target, MANUAL_CONTROL/SET_ATTITUDE_TARGET; telemetria ATTITUDE, LOCAL_POSITION_NED, SCALED_PRESSURE2, VFR_HUD | API `Middleware/src/rov_middleware/api.py`; interlock reale **invariato** (vedi §8) |
| `sitl_params/holoocean_sub.parm` | parametri SITL: `SCHED_LOOP_RATE`, `SIM_GPS_DISABLE=1`, `BARO_SPEC_GRAV` coerente con densità SITL (1.024 → oppure accettare 2.4 % errore), failsafe come sul reale (`FS_PILOT_INPUT=2`, `FS_GCS_ENABLE=2`), PSC_*/ATC_* copiati dal backup 913 parametri | `RoverLago/backups/2026-10-05_pre-change/ardusub/params_20261005T101049Z.parm` |

## 4. Porte e protocolli

| Collegamento | Protocollo | Porta | Direzione | Note |
|---|---|---|---|---|
| SITL → bridge | UDP, binario `<HHI16H>` 40 B | 9002 (+10·I) | SITL invia da porta effimera | bridge risponde a `recvfrom` addr |
| bridge → SITL | UDP, JSON una riga | porta sorgente del SITL | — | campi obbligatori 4.1: timestamp, imu.gyro, imu.accel_body, position, velocity, attitude/quaternion |
| bridge ↔ HoloOcean | API Python / shared memory | — | lockstep nativo | 1 `env.step` per frame nuovo |
| autonomia ↔ SITL | MAVLink 2 | TCP 5760 (o UDP 14551 via MAVProxy) | bidirezionale | **evitare 14550**, usata da QGC per il reale |
| QGC ↔ SITL | MAVLink | UDP 14550 solo se il reale è scollegato | — | rischio di confondere veicolo reale e SITL: usare SYSID diverso (es. `SYSID_THISMAV=2` nel SITL) |
| autonomia ↔ reale (futuro) | MAVLink | <ROV-IP> (endpoint BlueOS esistenti) | — | **fuori da Fase 2** finché non autorizzato in acqua |

## 5. Conversioni necessarie (sintesi; dettaglio e formule in `holoocean-physics-audit.it.md` §7)

| ArduSub JSON | Origine HoloOcean | Formula | Verifica |
|---|---|---|---|
| `timestamp` [s] | contatore tick | `t = k·dt` | monotonia; Δ costante |
| `imu.gyro` [rad/s, FRD] | IMUSensor (COM) ω FLU | `(ωx, −ωy, −ωz)` | rotazione pura in yaw: segno coerente con ATTITUDE.yawspeed |
| `imu.accel_body` [m/s², FRD] | IMUSensor (COM) specific force FLU (+9.8 su z a riposo) | `(ax, −ay, −az)` → ≈(0,0,−9.8) | a riposo |a|≈9.8; nota g=9.8 HoloOcean vs 9.80665 ArduPilot |
| `position` [m, NED] | PoseSensor traslazione NWU | `(x, −y, −z)` − home | profondità SITL (baro) = −z_nwu |
| `velocity` [m/s, NED] | VelocitySensor world NWU | `(vx, −vy, −vz)` | derivata numerica di position |
| `quaternion` [w,x,y,z] body→NED | PoseSensor R (colonne F,L,U) | `R_ned = S R S`, poi quaternion | confronto ATTITUDE vs ground truth < 1° |
| PWM[ch] → forza thruster [N] | — | `u=(pwm−1500)/400`; `F=T200(u,V)`; segno/indice da `motor_map` | MOTOR_TEST per canale (V-M1) |

## 6. Gestione dei clock

1. **Tempo simulato unico = tick HoloOcean × dt**, inviato come `timestamp`. ArduSub SITL forza il proprio clock su questo valore (lockstep). Nessun riferimento al wall clock nel percorso di controllo.
2. Il **wall clock** è registrato solo per diagnostica (RTF, latenza bridge, tempo per tick).
3. L'autonomia deve usare il tempo del veicolo (`time_boot_ms` dei messaggi MAVLink) e non `time.time()`: così lo stesso codice funziona con SITL lento (RTF<1) e con il reale. Questo è un requisito per gli algoritmi, da verificare nel codice esistente (es. `waypoint.py` usa tempi passati dal backend).
4. Reset: se HoloOcean viene resettato il timestamp deve **ripartire con il SITL riavviato** (un timestamp che torna indietro produce "Detected physics reset").
5. Sul reale: PC non sincronizzato NTP, BlueOS ≈ −3.8/−4.4 s rispetto al PC, ArduSub senza UTC (`hardware-inventory.it.md`, `sensor-data-audit.it.md`). Per confronti sim/reale usare tempo relativo dal boot, non UTC.

## 7. Primo esperimento: controllo di profondità (solo simulazione)

Scopo: dimostrare la catena completa ArduSub SITL (ALT_HOLD reale del firmware) → PWM → HoloOcean → stato → SITL.

| Passo | Azione | Criterio |
|---|---|---|
| E0 | Avvio SITL 4.1.2 + bridge, veicolo **disarmato** fermo a z=−1.0 m per 60 s simulati | nessun timeout; EKF healthy; ATTITUDE vs ground truth < 1°; profondità SITL vs −z_nwu < 2 cm (dopo correzione densità) |
| E1 | MOTOR_TEST per ciascun canale 1…8 (comando MAVLink al **SITL**) a 10 % per 1 s | forza/momento risultante coerente con la matrice VECTORED_6DOF (segno di Fx,Fy,Fz,Mx,My,Mz) → valida `motor_map` |
| E2 | ARM nel SITL, modalità **MANUAL**, input neutro | deriva verticale coerente con galleggiamento neutro del modello (atteso ≈0) |
| E3 | Modalità **ALT_HOLD**, target 1.0 m; poi gradino a 2.0 m (via MANUAL_CONTROL z / SET_POSITION_TARGET_GLOBAL_INT solo depth) | errore stazionario < 5 cm; overshoot < 20 %; tempo di assestamento registrato; nessuna saturazione persistente ±28.75 N |
| E4 | Ripetizione E3 a 100 Hz e 200 Hz | differenze documentate; scelta del dt definitivo |
| E5 | Confronto qualitativo con i dati reali disponibili (tlog 2025-11-17 `00100` + BIN `00000016`, profondità fino a 0.49 m; tlog QGC 2026-06-18) | solo forma della risposta; nessuna calibrazione dichiarata |

Gli armamenti e i comandi motore di E1–E4 sono inviati **esclusivamente al SITL** (127.0.0.1); il veicolo reale deve essere scollegato o il client deve rifiutare qualunque `system_id`/indirizzo diverso dal SITL.

## 8. Sicurezza verso il veicolo reale

- Il backend reale del Middleware resta passivo (`allow_actuation=False`, `bluerov.py:84-98`).
- Il nuovo `ardusub_backend` accetta come target **solo** URL locali del SITL (allow-list `tcp:127.0.0.1:5760`/`udp:127.0.0.1:14551`) e verifica `SYSID_THISMAV` diverso da quello reale; test unitario che fallisce se viene passato `<ROV-subnet>`.
- Nessuna modifica a parametri, endpoint BlueOS, QGC del veicolo reale.

## 9. Test

### Unitari (senza motore né rete)
1. Parsing/serializzazione pacchetti 16 e 32 canali; magic errato scartato.
2. JSON compatto (nessuno spazio dopo `:`), nessun NaN, wrapping `\n`.
3. Frame: a riposo `accel_body ≈ (0,0,−9.8)`; yaw +90° NWU ↔ −90° NED; quaternion unitario; roundtrip `S R S`.
4. PWM 1000/0 → forza 0; 1500 → 0; 1900 → +max; deadband.
5. `motor_map`: matrice wrench ricostruita = matrice VECTORED_6DOF (segni) per ciascun motore.
6. Lockstep: frame duplicato → nessun `step`, risposta identica; frame saltato → log; frame_count all'indietro → reset.
7. Allow-list del backend ArduSub (rifiuto <ROV-subnet>).

### Integrazione
1. Stub fisico (integratore Python banale) ↔ SITL: verifica protocollo senza HoloOcean.
2. HoloOcean ↔ bridge senza SITL (forze sintetiche): conservazione dt, misura tempo/tick a 100/200 Hz (V-T1).
3. SITL ↔ bridge ↔ HoloOcean: E0–E4.
4. Regressione: Middleware HoloOcean diretto continua a passare i 42 test e A–E.

## 10. Criteri di completamento della Fase 2 (primo collegamento)

- SITL 4.1.2 compilato e avviabile in modo riproducibile (script + versione registrata).
- Bridge in lockstep per ≥ 10 min simulati senza timeout/reinvii non gestiti.
- E0–E3 superati con i criteri sopra; mapping motori validato (E1) e documentato.
- Build HoloOcean patchata identificata per hash in ogni manifest; risultati riproducibili su due run.
- Tutti i test unitari verdi; nessun collegamento al veicolo reale nel codice di Fase 2.

## 11. Rischi principali

| # | Rischio | Impatto | Mitigazione |
|---|---|---|---|
| R1 | Build di ArduSub 4.1.2 SITL non compila su Ubuntu 24.04 | blocco | container Ubuntu 20.04; fallback Sub-4.5 con nota di non parità |
| R2 | Mapping motori/segni errato | controllo instabile | E1 MOTOR_TEST; tabella configurabile; test di matrice |
| R3 | Clamp ±28.75 N di HoloOcean vs T200 51.4/39.9 N @16 V | autorità ridotta, saturazioni | registrare saturazione; valutare patch motore (UE 5.3.2 sorgente in `<UE-5.3>`) |
| R4 | Costo per tick a 200 Hz → RTF basso | test lenti | accettabile in lockstep; scendere a 100 Hz se necessario |
| R5 | Sleep dei corpi Unreal | veicolo "congelato" | dither ±0.02 N; nessun teleport durante SITL |
| R6 | IMU a differenze finite con picchi dopo reset/teleport | EKF divergente | avvio SITL dopo stabilizzazione; nessun teleport |
| R7 | Modello idrodinamico povero (solo drag quadratico isotropo, niente massa aggiunta, inerzia da mesh) | discrepanza sim/reale | Fase 3: identificazione con dati reali (log da scaricare) |
| R8 | Differenze JSON tra versioni (sscanf 4.1/4.5) | parsing silenziosamente errato | JSON compatto; test con 4.1 e 4.5 |
| R9 | Rete WSL2 NAT / firewall | SITL non riceve | mirrored mode; regola firewall solo se necessario (richiede consenso utente) |
| R10 | Confusione SITL/reale su 14550 | comando al mezzo reale | SYSID diversi, porte diverse, allow-list |
| R11 | Spazio disco C: ~15 GB liberi | build/log falliscono | pulizia concordata o disco esterno |
| R12 | Densità/gravità diverse (1.024 vs 1.0; 9.8 vs 9.80665) | errore profondità ~2.4 % | parametri coerenti, test E0 |
