# Audit fisico HoloOcean 2.3.0 / BlueROV2 — Fase 1 (ricognizione)

Data: 2026-10-13. Ambito: sola lettura. Obiettivo: preparare il collegamento
**ArduSub SITL (backend JSON) ↔ HoloOcean (agente BlueROV2)**, in modo che gli
stessi algoritmi di autonomia girino in HoloOcean e sul BlueROV2 reale.

Nessun file è stato modificato fuori da questa cartella; nessun motore
HoloOcean/Unreal è stato avviato; nessuna build; nessun accesso a <ROV-subnet>;
solo comandi git in lettura (`git --no-optional-locks`). Le misure runtime citate
provengono da evidenze già archiviate (soprattutto nel progetto `Battery`).

Legenda delle fonti:

* **[CODE]** sorgente C++/Python di HoloOcean 2.3.0 (percorsi relativi a
  `<workstation>/Holoocean2-3\HoloOcean-2.3.0\HoloOcean-2.3.0\`,
  abbreviato `HO/`). Verificato: `HO/engine/Source` è identico (diff ricorsivo,
  ignorando CRLF, 0 differenze) al checkout pristine `<HOB>\src\before` al tag
  `v2.3.0`, commit `49e70552dfd97273b7dfbe755fbe65d7738b24b7` (2026-02-05).
* **[DOC]** documentazione ufficiale v2.3.0
  (<https://byu-holoocean.github.io/holoocean-docs/v2.3.0/>), copia locale in
  `HO/client/docs/` (contenuto confrontato con le pagine online BlueROV2 e
  Units-and-coordinates via WebFetch il 2026-10-13: coincidono).
* **[RUN-DOC]** misure runtime già documentate e archiviate (file citati).
* **[DERIV]** deduzione dal codice, non ancora verificata a runtime.
* **[DA VERIFICARE]** richiede una prova; procedura in §10.

---

## 1. Versioni installate e copia effettivamente usata

| Elemento | Valore | Fonte |
| --- | --- | --- |
| Client Python | `holoocean 2.3.0` in `<user-home>/miniconda3\envs\holoocean_joystick\lib\site-packages\holoocean` (Python 3.10.20). Installato da `file:///<user-home>/Downloads/HoloOcean-2.3.0/client` | `python -c "import holoocean"` (2026-10-13); `pip show`; `Battery/sources/verified_backend/check_final_original.json` (`client_install_origin`) |
| Client vs sorgente | I `.py` installati coincidono con `HO/client/src/holoocean` (diff: differiscono solo file non-Python: `fossen_dynamics/README.md`, `lcm/sensors.lcm`, `pylintrc`) | `diff -rq` 2026-10-13 |
| Altre installazioni client | Nessun altro `site-packages/holoocean` trovato negli env conda | ricerca 2026-10-13 |
| Pacchetto mondi installato | Unico: `%LOCALAPPDATA%\holoocean\2.3.0\worlds\Ocean`, `config.json` → `"version": "2.3.0"`, `path: Windows/Holodeck/Binaries/Win64/Holodeck` (SHA256 config `f9830d1a…`) | file locale |
| Mondi Ocean (config.json) | Rooms, ModernOffice, CorporateOffice, ExampleLevel, Tank (env ±7/−4..2 m), **SimpleUnderwater** (±70, −50..10 m), PierHarbor, OpenWater, FlatUnderwater, Dam; tutti `pre_start_steps: 20` | `config.json` |
| Binario ufficiale installato | `…\Ocean\Windows\Holodeck\Binaries\Win64\Holodeck.exe`, 255 731 200 B, SHA256 **`8c206c9cc05c640fb2efea6bd43bda33c585536ab16a931c55780855e7602253`**, timestamp PE (link) **2026-01-29 00:12:31 UTC**, file installato 2026-08-06 14:25 | `sha256sum`, header PE (2026-10-13) |
| Identità binario ufficiale | UE 5.3.2 Launcher (`++UE5+Release-5.3-CL-29314046`), MSVC 14.44.35207, SDK 10.0.22621.0, Development; progetto da checkout *develop* del team (commit non incorporato) | `Battery/docs/holoocean_patched_build.md` (da PDB/strings) |
| Build ricostruita di controllo (non patchata) | `<HOB>\packages\rebuilt-before\…\Holodeck.exe`, SHA256 `f52999dee2b064c492ec610d3b7ac450d77293ec3108d2ec20f22c42b2d6b524`, PE 2026-10-07 20:26:04 UTC | `sha256sum`, header PE |
| **Build patchata (drag)** | `<HOB>\packages\patched-after\Ocean\Windows\Holodeck\Binaries\Win64\Holodeck.exe`, 254 710 784 B, SHA256 **`4d6c2d46be8703758fbe4e9b62a99d4037aec0e220cca3a60b22b488d441684a`**, PE 2026-10-07 20:27:21 UTC | `sha256sum`, header PE; `Battery/sources/verified_backend/build_provenance/patched-after_manifest.json` |
| Il binario installato contiene la patch? | **No.** Hash = ufficiale `8c206c9c…`; `check_holoocean_backend.py` su di esso: `FAIL: detected ~0.01 drag scaling` | hash; `Battery/sources/verified_backend/check_final_original.json` |

**Chi usa cosa**

* `Middleware` (`src/rov_middleware/backends/holoocean.py:84-93`) chiama
  `holoocean.make(scenario_cfg=…)` senza percorso binario → usa il **pacchetto
  installato, non patchato**, a `ticks_per_sec = update_rate_hz = 30`
  (`config.py:79`), `frames_per_sec: False`, mondo `SimpleUnderwater`, BlueROV2,
  `control_scheme: 0`, sensori Pose/Velocity/IMU sul socket `COM`.
* `BlueROV2SimMatch` (`src/sim_server/runner.py:57,128`): `TICKS_PER_SEC = 30`,
  scheme 0, pacchetto installato (non patchato).
* `HoloObstacleAvoidance`: tps 30 obbligatorio (D-010); usa anche un motore
  modificato esterno (`docs/external_holoocean_engine.md`, client 2.2.2, cartella
  `<other-workstation>/Desktop\Holo\MondoTest` — percorso di un'altra macchina,
  non presente qui).
* `Battery` (HoloEnergy): unico progetto che usa la build patchata, tramite
  `tools/holoocean_backend.py --binary …` a **100 Hz**.

---

## 2. Agente BlueROV2: parametri esattamente dal sorgente

### 2.1 Parametri fisici [CODE]

| Parametro | Valore | File:riga |
| --- | --- | --- |
| Massa | `MassInKG = 11.5` kg | `HO/engine/Source/Holodeck/Agents/Private/BlueROV2.cpp:16` |
| Volume (costruttore) | `0.03554577` m³ | `BlueROV2.cpp:15` |
| Volume effettivo con `Perfect=true` (default) | `MassInKG / WaterDensity` = 11.5/997 = **0.0115346 m³** → galleggiamento **neutro** | `BlueROV2.cpp:31-39`, `BlueROV2.h:59-60` (`bool Perfect = true`) |
| Densità acqua | `WaterDensity = 997` kg/m³ (const) | `HolodeckCore/Public/HolodeckBuoyantAgent.h:26` |
| Gravità | `GetGravityZ()/-100` → **9.8 m/s²** (default mondo −980 cm/s²) | `HolodeckBuoyantAgent.cpp:13-14`; valore runtime 9.8 verificato [RUN-DOC] `Battery/docs/holoocean_drag_units_report.md` §6, §9 |
| COM (costruttore) | (0,0,3) cm, mesh frame | `BlueROV2.cpp:17-20` |
| COM con `Perfect` | `(t0+t2)/2` con Z = `t7.Z` → **(0, 0, −1) cm** in mesh frame UE (LHS) | `BlueROV2.cpp:32-33` |
| COB con `Perfect` | COM + 5 cm in Z → (0,0,+4) cm: **COB 5 cm sopra il COM** (momento raddrizzante) | `BlueROV2.cpp:35-36` |
| Coefficiente di drag | `CoefficientOfDrag = 0.8` | `BlueROV2.cpp:23` |
| Area di drag | `AreaOfDrag = 0.45` m², **isotropa** (stessa per ogni direzione) | `BlueROV2.cpp:24` |
| Legge di drag | solo **quadratica**: `F = −½·ρ·|v_rel|²·Cd·A·v̂_rel·ratio`, applicata al **COM** (nessuna coppia di drag) | `HolodeckBuoyantAgent.cpp:86-95` |
| Drag lineare / smorzamento | Nessun termine idrodinamico lineare; solo damping UE/Chaos: `SetLinearDamping(1.0)`, `SetAngularDamping(0.75)` (abilitati solo in scheme 0 e 1) | `BlueROV2.cpp:72-75`; chiamata in `BlueROV2ControlThrusters.cpp:23`, `BlueROV2ControlPD.cpp:28` |
| Forma del damping UE | `V *= max(0, 1 − c·dt)` (Chaos `PBDRigidsEvolutionGBF::Integrate`), indipendente dalla massa | [RUN-DOC] `Battery/docs/holoocean_timestep_report.md` §1 |
| Drag rotazionale | **Assente** (solo angular damping 0.75 s⁻¹) | [CODE] |
| Massa aggiunta / Coriolis | **Assenti** nel modello nativo | [CODE]; `Battery/docs/verified_backend_report.md:613` |
| Inerzia | **Non specificata** in HoloOcean: calcolata da UE dalla collision mesh scalata a 11.5 kg (`SetMassOverrideInKg`). Il commento Python `I = np.eye(3)` è dichiarato “may not be correct” | `HolodeckBuoyantAgent.cpp:17`; `HO/client/src/holoocean/agents.py` (classe `BlueROV2`, ~riga 985) — **[DA VERIFICARE]** valore reale del tensore |
| Rapporto di immersione | `ratio` = frazione di 1000 punti casuali nel bounding box sotto `SurfaceLevel = 0`; scala buoyancy e drag | `HolodeckBuoyantAgent.cpp:64-71`, `.h:51-54` |
| Corrente oceanica | getter `[x, −y, z]` (riflette Y verso UE), solo nel termine drag | `HolodeckBuoyantAgent.cpp:131-137` |
| Limite accelerazione | `GetAccelerationLimit() = 400` (“fall up to ~8 m”) | `BlueROV2.h:44` |

Evidenza runtime che `Perfect=true` è attivo nel binario cooked (il Blueprint
potrebbe in teoria sovrascriverlo): quiete neutra a velocità 0 e massa inferita
11.49999984–11.50000072 kg sulle tre build [RUN-DOC]
`Battery/docs/holoocean_drag_units_report.md` §14 (tabella controlli
indipendenti) e `Battery/docs/verified_backend_report.md` §16.

### 2.2 Thruster [CODE]

Spinta massima per thruster: `BR_MAX_THRUST = BR_MAX_LIN_ACCEL·11.5/4 = 10·11.5/4 =`
**28.75 N** (clamp simmetrico ±28.75 N), `BlueROV2.h:11-13`, clamp in
`BlueROV2.cpp:80`. Unità dei comandi scheme 0: **newton** (non normalizzati).

Direzioni (`BlueROV2.cpp:84-95`), espresse nel frame client body (FLU), poi
convertite `ClientToUE` (×100, riflessione Y), ruotate body→world e applicate
con `AddForceAtLocation` alla posizione del thruster (`BlueROV2.cpp:97-107`):

* `i < 4`: `(0, 0, F)` → verticali, F>0 spinge **verso l'alto** (+Z body).
* `i ≥ 4`, pari (4, 6): `(F/√2, +F/√2, 0)` → avanti-sinistra.
* `i ≥ 4`, dispari (5, 7): `(F/√2, −F/√2, 0)` → avanti-destra.

> **Aggiornamento 2026-11.** Nell'agente HoloOcean 2.3.0 i quattro thruster orizzontali sono fissati a **45°** (fattori 1/√2, codice compilato nell'engine). Il veicolo del laboratorio ha i quattro orizzontali montati a **30°** (verifica fisica). La differenza va gestita esplicitamente nel collegamento della Fase 2 (vedi `docs/phase2/`).

Posizioni (`BlueROV2.h:47-57`, cm, mesh frame UE **LHS**), poi sottratte del COM
(`BlueROV2.cpp:43-45`). Conversione [DERIV] in frame client body FLU, metri,
relativa al COM: `p = (x/100, −y/100, (z−(−1))/100)`.

| Idx | Nome (docstring Python / doc) | UE LHS cm | Client FLU m, rel. COM | Direzione F>0 (FLU) |
| ---: | --- | --- | --- | --- |
| 0 | Vertical Front Starboard | (12.00, 21.81, 7.09) | (+0.1200, −0.2181, +0.0809) | (0,0,+1) |
| 1 | Vertical Front Port | (12.00, −21.81, 7.09) | (+0.1200, +0.2181, +0.0809) | (0,0,+1) |
| 2 | Vertical Back Port | (−12.00, −21.81, 7.09) | (−0.1200, +0.2181, +0.0809) | (0,0,+1) |
| 3 | Vertical Back Starboard | (−12.00, 21.81, 7.09) | (−0.1200, −0.2181, +0.0809) | (0,0,+1) |
| 4 | Angled Front Starboard | (15.62, 9.88, −1.00) | (+0.1562, −0.0988, 0) | (+0.707, +0.707, 0) |
| 5 | Angled Front Port | (15.62, −9.88, −1.00) | (+0.1562, +0.0988, 0) | (+0.707, −0.707, 0) |
| 6 | Angled Back Port | (−15.62, −9.88, −1.00) | (−0.1562, +0.0988, 0) | (+0.707, +0.707, 0) |
| 7 | Angled Back Starboard | (−15.62, 9.88, −1.00) | (−0.1562, −0.0988, 0) | (+0.707, −0.707, 0) |

Questa tabella coincide con `BlueROV2SimMatch/src/sim_server/vehicle_dynamics.py:43-64`
e `Middleware/src/rov_middleware/backends/holoocean.py:23-28`, verificate con
probe open-loop [RUN-DOC] `HoloObstacleAvoidance/docs/BLUEROV2_SIMULATION_DYNAMICS.md`
(“Debugging history”): le matrici commentate nel docstring Python
(`agents.py`, classe `BlueROV2`, `thruster_d`/`thruster_p`) sono **errate**
(ordine e segni) e producevano surge inverso.

Verifica runtime già documentata: 10 N netti su X/Y/Z ricostruiti con errore
massimo 6.19e−7 N su tutte e tre le build [RUN-DOC]
`Battery/docs/holoocean_drag_units_report.md` §14.

### 2.3 Schemi di controllo

Registrazione [CODE] `Agents/Public/BlueROV2Controller.h:33-47` e
`HolodeckCore/Private/HolodeckPawnController.cpp:46-49,66-68`: il
`URawControlScheme` (aggiunto per primo) viene spostato all'indice 2.

| Indice | Engine | Input | Fisica nativa attiva |
| ---: | --- | --- | --- |
| **0** | `UBlueROV2ControlThrusters` | 8 forze [N], clamp ±28.75 | gravità, buoyancy, drag (`ApplyBuoyancyDragForce`), damping lineare 1.0 / angolare 0.75 (`BlueROV2ControlThrusters.cpp:23,29-30`) |
| **1** | `UBlueROV2ControlPD` | `[x,y,z]` m (world client) + `[roll,pitch,yaw]` gradi | come 0; PID: pos P=100, D=50; rot P=0.2, I=0.001, D=0.15; uscita clamp ±1 m/s² e ±1 (`BlueROV2ControlPD.h:12-37`) |
| **2** | `URawControlScheme` (custom dynamics) | 6 accelerazioni **world frame**: lin [m/s²] clamp ±10, ang clamp ±2 (`BlueROV2.cpp:54-67`, `AddForce(...,bAccelChange=true)`) | **Disattivati** gravità, buoyancy, damping (solo collisioni) [DOC] `blue-rov-agent.rst`; `agents.py:31-33` |

Incoerenza nel client [CODE]: `BlueROV2.control_schemes` (agents.py) restituisce
gli action space nell'ordine **[thrusters, accelerations, control]**, mentre
engine e documentazione usano **[0 thrusters, 1 PID, 2 accelerations]**. Solo i
limiti dell'action space sono sbagliati (l'engine usa l'indice), ma va tenuto
presente se si usa `env.action_space`.

---

## 3. Frame di coordinate, unità e sensori

### 3.1 Convenzioni generali

* **Unreal (interno)**: cm, sinistrorso (LHS), X avanti, Y destra, Z su.
* **Client HoloOcean** [DOC] `units-and-coordinates.rst`: metri, destrorso,
  world `x` avanti, `y` sinistra, `z` su. In termini robotici è un frame
  **NWU** se si associa x al Nord (scelta convenzionale: il mondo non è
  georeferenziato). Il body frame è **FLU**.
* Conversione [CODE] `General/Private/Conversion.cpp`:
  `ConvertLinearVector` = scala ×100 / ÷100 + `Y *= −1` (righe 95-111);
  `ConvertAngularVector` (pseudovettori) = `X *= −1, Z *= −1` (righe 113-131);
  `ConvertTorque` scala ×10⁴ (righe 143-161).
* Rotazioni client: `[roll, pitch, yaw]` in **gradi**, assi fissi x,y,z
  (equivalente a Z-Y-X intrinseco). `RotatorToRPY` (`Conversion.cpp:47-93`)
  restituisce gradi. Nota: `RPYToRotator` (righe 29-31) assegna due volte
  `R.M[2][1]` e mai `R.M[1][2]` (probabile bug, usato per teleport/scenario);
  **[DA VERIFICARE]** a roll≠0 e pitch≠0 combinati.
* Clock: `state["t"] = num_ticks / ticks_per_sec`, calcolato dal **client**,
  non letto dal motore (`environments.py:1394,1416`).

### 3.2 Sensori cinematici [CODE]

Tutti i sensori ereditano `UHolodeckSensor` con `TickGroup = TG_PostPhysics`
(`HolodeckCore/Private/HolodeckSensor.cpp:10`): leggono lo stato **dopo** il
passo fisico del tick.

| Sensore | Uscita | Frame / unità | Note dal codice |
| --- | --- | --- | --- |
| `IMUSensor` (`Sensors/Private/IMUSensor.cpp`) | 2×3 (`[accel; gyro]`), 4×3 con `ReturnBias` | **frame del sensore/socket**; m/s², rad/s | accel = `(v_now−v_prev)/DeltaTime` nel punto del sensore **+ (0,0,−GravityZ)** → **forza specifica** (a riposo +9.8 sull'asse “su” del sensore), poi `UnrotateVector` nel frame sensore (righe 141-154); gyro = ω world → frame sensore (156-165); conversione RHS (113-115). Rumore: gaussiano `AccelSigma/Cov`, `AngVelSigma/Cov`, bias random-walk `AccelBiasSigma`, `AngVelBiasSigma` (24-77, 117-121) |
| `DynamicsSensor` (`DynamicsSensor.cpp`) | 18 (RPY) o 19 (quat) | **world** client; m, m/s, m/s², rad/s², rad/s; RPY in **gradi** | layout `[acc(3), vel(3), pos(3), angacc(3), angvel(3), rpy(3) | quat x,y,z,w (4)]` (righe 93-120). Default `UseCOM=true`, `UseRPY=true` (`DynamicsSensor.h:48-51`). Accelerazioni per differenza finita /DeltaTime, **senza gravità**. Con `UseCOM` la velocità è al COM ma la posizione è la `GetComponentLocation()` del parent (origine root, non necessariamente il COM) — **[DA VERIFICARE]**. Quaternione emesso come `(Qx, −Qy, Qz, −Qw)` |
| `RotationSensor` | `[roll, pitch, yaw]` | world, **gradi** | `RotatorToRPY(GetComponentRotation())` |
| `OrientationSensor` | 3×3 | world | colonne = Forward, Left, Up del sensore (RHS) |
| `PoseSensor` | 4×4 `[R p; 0 1]` | world, m | `R` colonne = Forward, Left, Up ⇒ `R = R_world←body(FLU)`; `p` posizione del socket (righe 28-59) |
| `LocationSensor` | `[x,y,z]` | world, m | rumore `Sigma` |
| `VelocitySensor` | `[vx,vy,vz]` | **world**, m/s | velocità al punto del sensore |
| `DepthSensor` | `[z]` | m; **valore = z del sensore − SurfaceLevel**: **negativo** sott'acqua (non è una profondità positiva) | `DepthSensor.cpp:58-71`, rumore `Sigma` |
| `DVLSensor` | `[vx,vy,vz]` (+4 range se `ReturnRange`) | **frame sensore**, m/s; range m | velocità **rispetto al fondo/terra**, non all'acqua; 4 raggi a `Elevation`; rumore `VelSigma`, `RangeSigma` (righe 99-148) |

**Socket del BlueROV2** [DOC] `blue-rov-agent.rst`: tutti FLU tranne
**`IMUSocket`, ruotato 180° su x (x avanti, y destra, z giù = FRD)**.
Conseguenze [DERIV]:

* IMU su `IMUSocket`: accel a riposo ≈ (0, 0, **−9.8**), gyro in FRD → coincide
  già con la convenzione ArduSub (forza specifica FRD).
* IMU su `COM`: accel a riposo ≈ (0, 0, **+9.8**) in FLU → serve `diag(1,−1,−1)`.
  Il Middleware usa `COM` e `yaw_rate = −imu[5]` (`backends/holoocean.py:121`),
  validato in vasca virtuale (test D orario/antiorario PASS) [RUN-DOC]
  `Middleware/docs/session_validation.md`.
* Il segno a riposo della componente z di entrambi i socket è **[DA VERIFICARE]**
  a runtime (non c'è una misura archiviata dell'uscita IMU a riposo).

### 3.3 Limiti specifici dell'IMU simulata [CODE/DERIV]

* Accelerazione per differenza finita su `DeltaTime` del mondo: sotto 30 Hz
  (§4) è scalata di 2/3 perché la fisica integra solo 1/30 s.
* Al primo tick / dopo teleport `v_prev` è vecchia → spike di accelerazione
  (Battery “prime del sensore dopo teleport”, `holoocean_drag_units_report.md` §14).
* Nessun modello di vibrazione, saturazione, quantizzazione, misallineamento.

---

## 4. Passo temporale

### 4.1 Semantica [CODE + DOC]

* `ticks_per_sec` (default 30, `environments.py:164-171`): passato al motore
  come `-TicksPerSec=` (riga 1275) e via `SetTPSCommand`;
  `AHolodeckWorldSettings::FixupDeltaSeconds` restituisce sempre
  `ConstantTimeDeltaBetweenTicks = 1/TicksPerSec`
  (`HolodeckCore/Private/HolodeckWorldSettings.cpp`, `.h:25-29,47`;
  `ClientCommands/Private/AdjustTPSCommand.cpp:59-73`). DeltaTime del mondo
  **fisso**, indipendente dal tempo reale.
* `frames_per_sec` (default `true` = uguale a tps): solo limite di velocità
  di esecuzione (`t.MaxFPS`, `HolodeckGameMode.cpp:44-52`); `false` = più veloce
  possibile. Non cambia il passo simulato.
* Il motore avanza esattamente un tick per `env.tick()/step()` (lockstep col
  client).
* UE 5.3.2: senza substepping `dt_fisico = min(DeltaSeconds, MaxPhysicsDeltaTime)`,
  default `MaxPhysicsDeltaTime = 1/30 s`, `bSubstepping = false`,
  `bTickPhysicsAsync = false`, non modificati da `DefaultEngine.ini` di HoloOcean
  [RUN-DOC/source UE] `Battery/docs/holoocean_timestep_report.md` §1. Nessun
  riferimento a substep/MaxPhysicsDeltaTime nel sorgente HoloOcean (grep: solo
  un commento in `Uav.cpp:16`).

### 4.2 Claim storico “il passo fisico differisce da quello richiesto”: evidenze

1. **Prima osservazione** — `HoloObstacleAvoidance/docs/SCIENTIFIC_DECISIONS.md`
   D-010 (binario installato, non patchato): a `ticks_per_sec: 20` il veicolo
   si muoveva a **0.646×** la velocità riportata dai sensori; decisione: tps 30
   obbligatorio. (Misura “di sistema”, include damping/drag: non è una stima pura
   del passo.)
2. **Misura dedicata** — `Battery/docs/holoocean_timestep_report.md` (audit del
   2026-10-07), script `Battery/tools/audit_holoocean_timestep.py`, BlueROV2 in
   SimpleUnderwater, 20/30/40/60/100/200 Hz × 3 ripetizioni × 12 passi, quattro
   stimatori indipendenti (gravità, spinta nota, damping, cinematica). Build
   ufficiale (valori medi):

   | tps | dt client | DeltaTime mondo | dt fisico (gravità) | dt fisico / client | esito |
   | ---: | ---: | ---: | ---: | ---: | --- |
   | 20 | 0.050000 | 0.050000001 | 0.0333333373 | **0.6666667** | divergente |
   | 30 | 0.033333 | 0.033333335 | 0.0333333373 | 1.0000001 | coerente |
   | 40 | 0.025 | 0.024999999 | 0.0249999999 | 1.0000000 | coerente |
   | 60 | 0.016667 | 0.016666667 | 0.0166666684 | 1.0000000 | coerente |
   | 100 | 0.010 | 0.010000000 | 0.0100000004 | 1.0000000 | coerente |
   | 200 | 0.005 | 0.005000000 | 0.0050000002 | 1.0000001 | coerente |

   Predizione `min(f32(1/tps), f32(1/30))`, scarto ≤ 1.7e−7.
3. **Con la build patchata**: sì, verificato. `Battery/sources/verified_backend/timestep_{original,rebuilt,patched}/report.json`:
   `verified_ticks_per_sec = [30, 40, 60, 100, 200]`, `rejected = [20]` per tutte
   e tre le build (binari `8c206c9c…`, `f52999de…`, `4d6c2d46…`; letti il
   2026-10-13). La patch non tocca il passo.
4. Con drag corretto, l'integrazione è Eulero esplicito semi-implicito: errore
   del primo passo in decelerazione da 2.5 m/s = −168% a 20/30 Hz (inversione
   di velocità), −42% a 60 Hz, −15% a 100 Hz, −3.9% a 200 Hz;
   `K/m = ρ·Cd·A/(2m) = 15.6 m⁻¹`. Parametro di stabilità `K·|v_rel|·dt/m`
   (≤0.125 a 100 Hz per v≤0.8 m/s). Fonte: `holoocean_timestep_report.md` §4.

**Conclusione**: il passo fisico coincide con il clock solo per tps ≥ 30; sotto
30 Hz la fisica perde tempo silenziosamente (comportamento documentato di UE, non
documentato da HoloOcean). Con il drag corretto servono **≥ 100 Hz** per
transitori accurati (raccomandazione Battery). Middleware/SimMatch usano 30 Hz
sul binario non patchato (dove il drag è 100× troppo debole e il problema di
stabilità non appare).

---

## 5. Patch del drag

| Voce | Contenuto |
| --- | --- |
| File patch | `<workstation>/Battery\patches\holoocean-2.3-drag-units.patch` (SHA256 `00f1b303…a488fa`, 11 righe) + README; variante `holoocean-develop-drag-units.patch` (`3f5508df…`) per `develop` `f3d1230c` (**mai compilata né eseguita**) |
| Bug | `HolodeckBuoyantAgent.cpp:91-95` (v2.3.0): la forza di drag è calcolata in **N** (SI) ma passata direttamente ad `AddForceAtLocation`, la cui unità è kg·cm/s² (1 N = 100 unità). Gravità, buoyancy e thruster usano `ConvertLinearVector(…, ClientToUE)` (×100); il drag no ⇒ drag applicato = **1/100** del valore SI, sia con corrente sia per veicolo in moto in acqua ferma |
| Fix | dopo il calcolo: `DragForce *= UEUnitsPerMeter;` (100). Non `ConvertLinearVector`, perché `RelativeVel` è già in assi UE (la corrente è riflessa nel getter) e la Y verrebbe riflessa due volte |
| Sorgente patchato | `<HOB>\src\after` (commit `49e70552`, `git status`: solo `M …HolodeckBuoyantAgent.cpp`, +2 righe; SHA256 file `3876cccf…`). Pristine: `<HOB>\src\before` |
| Build che la contiene | **solo** `<HOB>\packages\patched-after\…\Holodeck.exe` (`4d6c2d46…`), compilata 2026-10-07 con UE 5.3.2 da sorgente, MSVC 14.44.35207, contenuti cooked ufficiali riusati (hard link). Il pacchetto installato in `%LOCALAPPDATA%` **non** la contiene |
| Controllo | `rebuilt-before` (`f52999de…`): dati di audit **bit-identici** al binario ufficiale |

**Evidenze runtime documentate** [RUN-DOC]
(`Battery/docs/verified_backend_report.md`, `holoocean_drag_units_report.md`,
`Battery/sources/verified_backend/drag_before_after.json`):

* Drag applicato/atteso: ufficiale **0.0099999997** (153 casi = 17 casi × 60/100/200 Hz × 3);
  patchata **1.0000000074** (stessi 153 casi: correnti ±X/±Y/±Z, veicolo ruotato,
  legge quadratica). Esempio: corrente 0.1 m/s → atteso 1.7946 N; ufficiale
  0.017946 N; patchata 1.7946000 N.
* Veicolo in moto in acqua ferma (117 casi): 1.0000001686 primo passo,
  1.0000000180 ogni passo.
* Test upstream `test_currents_underwater` (HoveringAUV, 1 m/s, 60 Hz):
  ufficiale 0.06320976 m/s² (= costante attesa upstream, cioè 0.01×SI);
  patchata 6.32097578 m/s².
* Spinta 10 N, gravità, neutralità, stabilità angolare: bit-identici
  controllo/patchata.
* `check_holoocean_backend.py`: installato → FAIL (~0.01); patchato → PASS
  (`check_final_original.json`, `check_final_patched.json`, 2026-10-07).
* Suite nativa 29/29 PASS; CI GitHub (8 job) success (`Battery/build/drag-final-receipt.json`).

**Cosa NON dimostra la patch (resta da verificare)**

* Solo l'**unità** del drag è corretta: i valori `Cd=0.8`, `A=0.45 m²`
  isotropi, l'assenza di drag lineare, di drag/coppie rotazionali, di massa
  aggiunta, il damping UE (1.0 / 0.75 s⁻¹, non fisico e mass-independent) e
  l'inerzia della mesh **non sono validati** contro il BlueROV2 reale.
* Con il drag 100× più forte il veicolo sarà molto più lento: la velocità di
  regime con 4 thruster orizzontali a 28.75 N (forza netta max in X ≈ 4·28.75·0.707
  ≈ 81 N) è [DERIV] `v = √(2F/(ρCdA)) ≈ √(162/358.9) ≈ 0.67 m/s` (trascurando il
  damping 1 s⁻¹, che la riduce ulteriormente); sul binario non patchato
  la velocità sarebbe limitata quasi solo dal damping lineare (~7 m/s
  teorici). **[DA VERIFICARE]** a runtime e contro il reale (BlueROV2 Heavy
  ~1–1.5 m/s).
* Gli altri agenti buoyant e il ramo `develop` non sono stati verificati a runtime.
* Upstream: nessuna issue/PR inviata (issue esistente #368).

---

## 6. Frequenze dei sensori, sensori custom, rumore

* `"Hz"` nella config sensore: solo **decimazione lato client**:
  `tick_every = ticks_per_sec / Hz` (intero; avviso e arrotondamento se non
  divisore; errore se Hz > tps) (`environments.py:333-352`); il sensore C++
  calcola comunque **ogni tick**, il client restituisce il dato solo quando
  `tick_count == tick_every`, altrimenti la chiave manca dallo stato
  (`sensors.py:62-65`, `environments.py:1370-1378`). Default Hz = tps.
  Camere/lidar hanno inoltre `TicksPerCapture` lato C++.
* Esempi documentati [DOC]: IMU 200 Hz (richiede tps ≥ 200), DVL 20 Hz,
  Depth 100 Hz, GPS 5 Hz; scenario `SimpleUnderwater-Hovering.json`: tps 30,
  IMU 30, DVL 20, Depth 30, GPS 5.
* Rumore: modelli gaussiani multivariati (`Sigma`/`Cov` scalari, 3-vettori o
  3×3), bias random-walk per IMU; DVL con rumore per-raggio trasformato;
  nessun ritardo/latenza modellato.
* Sensori custom: supportati [DOC] `HO/client/docs/develop/sensors.rst`
  (classe C++ `U…Sensor : UHolodeckSensor` in `Sensors/Public|Private`,
  `TickSensorComponent`, buffer float; classe Python in `sensors.py`).
  **Richiede ricompilare l'engine** (come la patch del drag) — i mondi Ocean
  non sono pubblici, quindi solo build code-only come in Battery.
* LCM: pubblicazione opzionale per sensore (`lcm_channel`).

---

## 7. Compatibilità con ArduSub SITL (backend JSON)

Riferimenti esterni (non verificati in questa fase, **[DA VERIFICARE]** sul
sorgente ArduPilot della versione che si userà): `libraries/SITL/SIM_JSON.cpp`
e `libraries/SITL/examples/JSON/readme.md`. Protocollo: ArduPilot invia via UDP
(porta 9002) un pacchetto binario `{uint16 magic (18458 = 16 canali, 29569 = 32),
uint16 frame_rate, uint32 frame_count, uint16 pwm[16]}`; il simulatore risponde
con una riga JSON con `timestamp` [s], `imu.gyro` [rad/s, body FRD],
`imu.accel_body` [m/s², body FRD, forza specifica], `position` [m, NED],
`attitude` [rad, roll/pitch/yaw] **oppure** `quaternion` [w,x,y,z], `velocity`
[m/s, NED]. Lockstep: un passo fisico per pacchetto servo.

Notazione: `S = diag(1, −1, −1)` (NWU↔NED e FLU↔FRD; `S = S⁻¹ = Sᵀ`).
Ipotesi: x di HoloOcean = Nord.

### 7.1 Tabella “campo ArduSub ← dato HoloOcean”

| Campo ArduSub | Origine HoloOcean | Unità origine | Formula | Verifica |
| --- | --- | --- | --- | --- |
| `timestamp` [s] | `state["t"]` = `num_ticks/tps` (client) | s | `t = n/tps`; deve essere strettamente crescente; con tps ≥ 30 coincide col tempo fisico | log Δt costante; confronto con integrazione di velocità/posizione |
| `imu.gyro` [rad/s, FRD] | `IMUSensor[1,:]` su `COM` (FLU) **oppure** su `IMUSocket` (già FRD) | rad/s | COM: `ω_frd = S·ω_flu = (p, −q, −r)`; IMUSocket: identità **[DA VERIFICARE]** | yaw orario visto dall'alto ⇒ `gyro.z > 0`; prua giù ⇒ `gyro.y < 0` |
| `imu.accel_body` [m/s², FRD, forza specifica] | `IMUSensor[0,:]` (include già +g, forza specifica) | m/s² | COM: `a_frd = S·a_flu` ⇒ a riposo ≈ (0,0,−9.8); IMUSocket: identità **[DA VERIFICARE]**. **Non** aggiungere g una seconda volta. Alternativa da `DynamicsSensor` (senza g, world): `a_frd = R_nedfrdᵀ·(S·a_world − [0,0,g])` con g=9.8 | a riposo neutro: (0,0,−9.80±rumore); g HoloOcean 9.8 vs 9.80665 ArduPilot (0.07%) |
| `position` [m, NED] | `PoseSensor[:3,3]` (COM socket) o `DynamicsSensor[6:9]` | m world NWU | `p_ned = S·p_nwu − p0` (origine a scelta); z NED = −z_nwu ⇒ depth = `SurfaceLevel − z_nwu` (SurfaceLevel = 0) | a 5 m sotto la superficie `position[2] ≈ +5` |
| `attitude` [rad] / `quaternion` [w,x,y,z] | `PoseSensor[:3,:3]` = `R_nwu←flu` | adimensionale | `R_ned←frd = S·R·S`; `roll = atan2(R32,R33)`, `pitch = −asin(R31)`, `yaw = atan2(R21,R11)`; quaternione da `R_ned←frd`. Evitare `RotationSensor`/RPY (gradi, singolarità, `RPYToRotator` sospetto) e il quaternione di `DynamicsSensor` (segni speciali) | yaw: veicolo verso +y HoloOcean (sinistra/Ovest) ⇒ yaw NED = −90° |
| `velocity` [m/s, NED] | `VelocitySensor` (world, punto del socket) o `DynamicsSensor[3:6]` (COM) | m/s world NWU | `v_ned = S·v_nwu` | derivata numerica di `position` ≈ `velocity` |
| Profondità/baro | non trasmessa: SITL deriva la pressione dalla `position` z (**[DA VERIFICARE]** comportamento baro SITL per ArduSub con JSON); in alternativa `DepthSensor` (valore **negativo** sott'acqua: depth = −z) | m | `depth = −DepthSensor` | confronto con `position[2]` |
| `rng_*` (opz.) | `DVLSensor` range / `RangeFinderSensor` | m | diretto | — |
| Magnetometro, GPS | non necessari: SITL li simula dalla posa | — | — | ArduSub senza GPS: EKF origin da impostare |

### 7.2 Tabella inversa “canale PWM ArduSub → thruster HoloOcean”

Catena per canale: `u = clip((PWM − 1500)/400, −1, 1)` → deadband T200 →
`F = T200_16V(u)` [N] (Battery: `holoenergy/profiles/thrusters/bluerobotics_t200.json`,
dati Blue Robotics 2019, a 16 V: max avanti **51.44 N**, max indietro
**39.91 N**, deadband |u| ≤ 0.07 ≈ ±28 µs) → segno di montaggio `s_i` →
indice HoloOcean `i` → `F_i` [N] in scheme 0 (clamp HoloOcean ±28.75 N).

Matrice ArduSub `SUB_FRAME_VECTORED_6DOF` (FRAME_CONFIG = 2) come la
ricordiamo da `libraries/AP_Motors/AP_Motors6DOF.cpp` — colonne roll, pitch, yaw,
throttle, forward, lateral — **da verificare sul sorgente**:
M1 (0,0,+1,0,−1,+1), M2 (0,0,−1,0,−1,−1), M3 (0,0,−1,0,+1,+1),
M4 (0,0,+1,0,+1,−1), M5 (+1,−1,0,−1,0,0), M6 (−1,−1,0,−1,0,0),
M7 (+1,+1,0,−1,0,0), M8 (−1,+1,0,−1,0,0).
Layout fisico BlueROV2 Heavy (documentazione Blue Robotics, da verificare):
M1 orizz. anteriore dx, M2 orizz. anteriore sx, M3 orizz. posteriore dx,
M4 orizz. posteriore sx, M5–M8 verticali ant. dx, ant. sx, post. dx, post. sx.

Mappa **ipotetica** [DERIV], ottenuta imponendo che i fattori ArduSub
producano la stessa coppia/forza con la geometria HoloOcean di §2.2 (controllo
di consistenza fatto a mano su forward, lateral, yaw, roll, pitch; assume
throttle positivo = salita):

| Canale ArduSub | Posizione | Indice HoloOcean | `s_i` | Controlli di segno |
| --- | --- | ---: | ---: | --- |
| SERVO1 / M1 | orizz. ant. dx | 4 | −1 | fwd −1, lat +1, yaw +1 coerenti |
| SERVO2 / M2 | orizz. ant. sx | 5 | −1 | fwd −1, lat −1, yaw −1 coerenti |
| SERVO3 / M3 | orizz. post. dx | 7 | +1 | fwd +1, lat +1, yaw −1 coerenti |
| SERVO4 / M4 | orizz. post. sx | 6 | +1 | fwd +1, lat −1, yaw +1 coerenti |
| SERVO5 / M5 | vert. ant. dx | 0 | −1 | roll +1, pitch −1, throttle −1 coerenti |
| SERVO6 / M6 | vert. ant. sx | 1 | −1 | roll −1, pitch −1 coerenti |
| SERVO7 / M7 | vert. post. dx | 3 | −1 | roll +1, pitch +1 coerenti |
| SERVO8 / M8 | vert. post. sx | 2 | −1 | roll −1, pitch +1 coerenti |

Avvertenze: (a) i `s_i` dipendono anche da `MOTn_DIRECTION`/eliche invertite del
veicolo reale e dalla mappatura `SERVOn_FUNCTION`; (b) la T200 è asimmetrica
avanti/indietro, quindi il segno va applicato **prima** di scegliere il ramo della
curva (il ramo dipende dal verso di rotazione dell'elica, non dalla direzione
della forza nel body); (c) ordine canale→thruster, segni e curva vanno verificati
con la procedura V7 (§10). Il reale (RoverLago/BlueROV2SimMatch) usa
FRAME_CONFIG=2 secondo `BlueROV2SimMatch/src/sim_server/vehicle_dynamics.py:14-16`.

---

## 8. Incompatibilità e soluzioni possibili (non implementate)

| # | Incompatibilità | Impatto | Soluzioni possibili |
| ---: | --- | --- | --- |
| 1 | **Drag 1/100** nel binario installato | dinamica traslazionale irrealistica (velocità limitata dal solo damping UE) | usare la build patchata `4d6c2d46…` per SITL; rieseguire `check_holoocean_backend.py` prima di ogni campagna |
| 2 | **Clamp 28.75 N** per thruster vs T200@16 V 51.4/39.9 N | saturazione a ~56% (avanti) del range ArduSub; autorità ridotta | (a) patch C++ di `BR_MAX_THRUST` (nuova build); (b) scheme 2 con dinamica propria (Fossen/HoloEnergy) applicando forze come accelerazioni; (c) accettare e scalare consapevolmente |
| 3 | **dt fisico ≤ 1/30 s**; con drag corretto servono ≥100 Hz; ArduSub gira tipicamente a 400 Hz (`SCHED_LOOP_RATE`, **[DA VERIFICARE]**) | IMU a bassa frequenza per l'EKF; costo per tick di HoloOcean (a 30 Hz con render ~22 ms/tick in HoloObstacleAvoidance) | tps 100–200 senza viewport (`frames_per_sec: false`, `show_viewport=False`); verificare che ArduSub accetti un IMU a 100–200 Hz; in alternativa interpolare/upsample lato bridge (sconsigliato per l'EKF) |
| 4 | Frame NWU/FLU e IMU socket FRD | errori di segno | bridge con `S=diag(1,−1,−1)`; preferire `COM`+conversione esplicita o `IMUSocket` dopo verifica V3 |
| 5 | IMU per differenza finita: spike dopo teleport/reset, scala errata < 30 Hz | EKF divergence all'avvio | scartare i primi N tick dopo reset; nessun teleport durante il volo; tps ≥ 30 |
| 6 | Modello idrodinamico povero (drag isotropo al COM, niente massa aggiunta, damping UE, inerzia mesh) | sim-to-real gap sui guadagni PID ArduSub | identificazione parametri (BlueROV2SimMatch S3); oppure scheme 2 + modello Fossen calibrato; modificare `Cd/A` via patch C++ |
| 7 | **Sleep del rigid body UE**: `AddForceAtLocation` non sveglia un corpo addormentato (velocità esattamente 0 con thruster saturi) | ArduSub vedrebbe il veicolo bloccato, integratori in windup | dither ±0.02 N + teleport same-pose come in Middleware (`holoocean.py:146-161`) e SimMatch (`runner.py` `_sleep_guard`); meglio: disabilitare lo sleep in C++ (nuova build) |
| 8 | Ordine/segni canali ArduSub ↔ thruster HoloOcean non verificati | comandi errati / instabilità | procedura V7; tabella §7.2 solo ipotesi |
| 9 | Clock: HoloOcean non è real-time garantito; il bridge deve essere lockstep | se ArduSub non è in lockstep, dt incoerenti | usare il lockstep del backend JSON (un `env.step` per pacchetto servo); timestamp = n/tps |
| 10 | g = 9.8 (HoloOcean) vs 9.80665 (ArduPilot) | offset di 0.007 m/s² in z a riposo | trascurabile; eventualmente correggere nel bridge (documentandolo) |
| 11 | Profondità/barometro | SITL calcola il baro dalla posizione; superficie a z=0 HoloOcean | origine NED con z=0 alla superficie; verificare `SIM_` baro per Sub |
| 12 | `action_space` Python BlueROV2 con ordine errato | validazioni lato client fuorvianti | non affidarsi a `env.action_space`; usare limiti C++ |
| 13 | Corrente: solo world frame, solo nel drag (nessuna corrente nell'IMU/DVL relativa all'acqua) | DVL misura velocità sul fondo (corretto), ma niente velocità relativa | ok per ArduSub (DVL opzionale) |

---

## 9. Verifiche già documentate

| ID | Verifica | Build | Risultato | Fonte |
| --- | --- | --- | --- | --- |
| D1 | Drag applicato/atteso (153 casi, correnti) | ufficiale / controllo / patchata | 0.0099999997 / 0.0099999997 / 1.0000000074 | `Battery/docs/verified_backend_report.md` §10–§17; `Battery/sources/verified_backend/drag_before_after.json` |
| D2 | Autodrag, veicolo in moto (117 casi) | patchata | 1.0000001686 (1° passo), 1.0000000180 | idem §17 |
| D3 | Test upstream `test_currents_underwater` | ufficiale / patchata | 0.06320976 / 6.32097578 m/s² | `Battery/sources/verified_backend/upstream_test_*.json` |
| D4 | Spinta netta 10 N X/Y/Z | tutte | errore ≤ 6.19e−7 N | `Battery/docs/holoocean_drag_units_report.md` §14 |
| D5 | Gravità 9.8 m/s², massa 11.5 kg, neutralità, ω≈0 | tutte | err. 7.2e−7 m/s²; massa 11.4999998–11.5000007; v=0 | idem |
| T1 | dt fisico vs clock (20–200 Hz) | tutte e tre | 20 Hz: 0.6666667; 30–200 Hz: 1.0000000 | `Battery/docs/holoocean_timestep_report.md`; `sources/verified_backend/timestep_*` |
| T2 | Errore di passo con drag corretto | patchata | −168% (30 Hz) … −3.9% (200 Hz) | idem §4 |
| T3 | Osservazione storica 0.646× a 20 Hz | installata | conferma qualitativa | `HoloObstacleAvoidance/docs/SCIENTIFIC_DECISIONS.md` D-010 |
| G1 | Geometria thruster (ordine/segni) | installata | probe open-loop coerenti con engine | `HoloObstacleAvoidance/docs/BLUEROV2_SIMULATION_DYNAMICS.md`; `BlueROV2SimMatch/src/sim_server/vehicle_dynamics.py` |
| F1 | Frame NWU→NED, yaw, yaw-rate dal gyro COM | installata, 30 Hz | test A–E PASS | `Middleware/docs/session_validation.md` |
| S1 | Sleep del rigid body | installata | osservato (48 run invalidate), mitigato | `BlueROV2SimMatch/src/sim_server/runner.py` `_sleep_guard` |
| B1 | Identità binari (hash) e sorgente patchato | — | ricontrollati 2026-10-13 | §1, §5 |

## 10. Verifiche ancora necessarie (procedure)

Tutte richiedono di avviare il motore: **non eseguite** in Fase 1. Usare
`<user-home>/miniconda3\envs\holoocean_joystick\python.exe`, binario
patchato (`--binary` / backend Battery) salvo diversa indicazione, mondo
`SimpleUnderwater`, `show_viewport=False`, `frames_per_sec=False`.

* **V1 – Backend corretto prima di ogni campagna**:
  `python Battery/tools/check_holoocean_backend.py --binary <HOB>/packages/patched-after/Ocean/Windows/Holodeck/Binaries/Win64/Holodeck.exe`
  → atteso `PASS`; ripetere sul pacchetto installato → atteso `FAIL` (controllo).
* **V2 – IMU a riposo, socket COM e IMUSocket**: BlueROV2 a z=−5 m, scheme 0
  forze nulle, sensori `IMUSensor` su `COM` e su `IMUSocket` (nomi diversi),
  noise 0, tps 100; scartare 5 tick; registrare 200 tick. Atteso: COM
  accel ≈ (0,0,+9.8), IMUSocket ≈ (0,0,−9.8); gyro ≈ 0.
* **V3 – Segni gyro/accel per socket**: applicare per 1 s yaw (thruster 4,5,6,7
  con pattern ±), poi pitch (verticali anteriori vs posteriori), poi surge.
  Confrontare il gyro con la derivata numerica di `PoseSensor` convertita in
  FRD (`S·R·S`); atteso accordo di segno e modulo (<1%). Stabilisce se
  `IMUSocket` è già FRD.
* **V4 – DynamicsSensor posizione con UseCOM**: confrontare `DynamicsSensor[6:9]`
  con `PoseSensor` su `COM` e su `Origin`; quantificare l'offset (atteso ≈ 1 cm
  in z se la posizione è l'origine root e non il COM).
* **V5 – Velocità di regime e decelerazione (drag patchato)**: surge a forza
  costante 4×28.75 N sui thruster orizzontali (pattern di sola traslazione X),
  tps 100 e 200, 30 s; misurare v∞ e confrontarla con la soluzione di
  `m·dv/dt = F − K·v² − c·m·v` (K = ½·997·0.8·0.45, c = 1 s⁻¹); poi rilasciare
  e misurare la decelerazione. Ripetere su sway, heave, yaw (yaw: solo
  angular damping). Confrontare con le misure reali (pool) quando disponibili.
* **V6 – Inerzia della mesh**: coppia di yaw nota (coppie di thruster
  opposti), primo passo da fermo a tps 100: `Izz = τ / (Δω/dt)` con damping
  corretto `(1 − 0.75·dt)`; idem roll/pitch (togliendo la coppia di
  raddrizzamento ρ·g·V·0.05·sinθ ≈ 5.6 N·m·sinθ). Registrare il tensore.
* **V7 – Mappatura PWM→thruster**: (1) leggere `AP_Motors6DOF.cpp` della
  versione ArduSub usata e confermare la matrice VECTORED_6DOF; (2) in SITL
  armato in MANUAL, inviare `RC_CHANNELS_OVERRIDE` puro forward, lateral,
  yaw, throttle, roll, pitch, uno per volta; (3) loggare i 16 PWM del pacchetto
  JSON e le forze applicate; (4) verificare in HoloOcean che ogni comando
  produca il moto atteso (segno e assenza di accoppiamenti); (5) solo dopo,
  confrontare con `MOTn_DIRECTION`/`SERVOn_FUNCTION` del veicolo reale (lettura
  parametri, senza scrittura).
* **V8 – Lockstep e timestamp**: bridge JSON a tps 100 e 200; loggare
  `frame_count`, timestamp inviato, tempo di parete; atteso Δt = 1/tps
  costante, nessun pacchetto perso, `SIM`-time monotono; misurare il costo per
  tick (deve restare sotto il budget se si vuole real-time).
* **V9 – Sleep**: comando costante piccolo (0.05 N) dopo 10 s di quiete;
  verificare se la velocità resta esattamente 0 sulla build patchata e se il
  dither ±0.02 N la risolve.
* **V10 – Baro/profondità in SITL ArduSub**: con il veicolo a z=−5 m verificare
  che ArduSub riporti depth 5 m (`GLOBAL_POSITION_INT.relative_alt`, `SCALED_PRESSURE2`).
* **V11 – `RPYToRotator`**: teleport con roll=20°, pitch=10°, yaw=30°,
  confrontare `PoseSensor` con la matrice attesa (Z-Y-X).

---

## 11. Elementi di evidenza per il manifest JSON

```text
holoocean_client_version            = 2.3.0  (env holoocean_joystick, Python 3.10.20)
holoocean_source_tag                = v2.3.0 / 49e70552dfd97273b7dfbe755fbe65d7738b24b7
desktop_source_equals_tag           = true (diff -rq --strip-trailing-cr, engine/Source)
ocean_config_json_sha256            = f9830d1a259645a8671e26a614c5e8be8750afe7c0ccba4612ed84e01d660bcd
installed_binary_path               = %LOCALAPPDATA%/holoocean/2.3.0/worlds/Ocean/Windows/Holodeck/Binaries/Win64/Holodeck.exe
installed_binary_sha256             = 8c206c9cc05c640fb2efea6bd43bda33c585536ab16a931c55780855e7602253
installed_binary_pe_timestamp_utc   = 2026-01-29T00:12:31
installed_binary_contains_drag_patch= false
patched_binary_path                 = <HOB>/packages/patched-after/Ocean/Windows/Holodeck/Binaries/Win64/Holodeck.exe
patched_binary_sha256               = 4d6c2d46be8703758fbe4e9b62a99d4037aec0e220cca3a60b22b488d441684a
patched_binary_pe_timestamp_utc     = 2026-10-07T20:27:21
control_binary_sha256               = f52999dee2b064c492ec610d3b7ac450d77293ec3108d2ec20f22c42b2d6b524
drag_patch_file_sha256              = 00f1b30383f4f40544a44c8660b84c4ade31c25bf6409b0d662a21728ab488fa
develop_patch_file_sha256           = 3f5508dfa7b40690884d1cb0266399b0edc84cf91fb828d3e6d90a448711c02e (not built)
patched_source_file_sha256          = 3876cccff1d5dfb8ece8c98fd3e1491d364019e9f03f80e69577fe1ac6e2ca9f (<HOB>/src/after)
original_source_file_sha256         = 580ce38bd1a226133d8e03c702e75740cd051b6efae90c1c0c1414c72ac0118d (Desktop copy, CRLF)
BlueROV2.cpp_sha256                 = c3a7da116e5346ce19aefaf9be72312d2a1cbaf51604371f46969cf8086209ec
BlueROV2.h_sha256                   = c5e5a0a725bd2a55851a8523789b0ae47facf57b87046f345ad4cdc0d99547ca
bluerov2_params                     = mass 11.5 kg; V=m/997 (Perfect); COM (0,0,-1) cm mesh; COB +5 cm; Cd 0.8; A 0.45 m2; lin damp 1.0; ang damp 0.75; max thrust 28.75 N
control_schemes                     = 0 thrusters[N]; 1 PID pose; 2 world accelerations (no gravity/buoyancy/damping)
drag_ratio_evidence                 = Battery/sources/verified_backend/drag_before_after.json (0.0099999997 -> 1.0000000074)
backend_check_evidence              = Battery/sources/verified_backend/check_final_{original,patched,rebuilt}.json
timestep_evidence                   = Battery/sources/verified_backend/timestep_{original,rebuilt,patched}/report.json (verified 30..200, rejected 20)
timestep_historical                 = HoloObstacleAvoidance/docs/SCIENTIFIC_DECISIONS.md D-010 (0.646x at 20 Hz)
frames_evidence                     = Middleware/docs/session_validation.md; Middleware/src/rov_middleware/frames.py
thruster_geometry_evidence          = BlueROV2SimMatch/src/sim_server/vehicle_dynamics.py; HoloObstacleAvoidance/docs/BLUEROV2_SIMULATION_DYNAMICS.md
t200_curve                          = Battery/holoenergy/profiles/thrusters/bluerobotics_t200.json (16 V: +51.44 N / -39.91 N; u=(PWM-1500)/400)
official_docs                       = https://byu-holoocean.github.io/holoocean-docs/v2.3.0/ (agents/agents/blue-rov-agent.html, usage/units-and-coordinates.html, usage/scenarios.html)
pending_verifications               = V1..V11 (sezione 10)
```
