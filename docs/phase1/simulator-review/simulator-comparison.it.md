# Confronto simulatori per ArduSub SITL: HoloOcean vs Stonefish vs Gazebo

**Progetto:** collegamento ArduSub SITL ↔ HoloOcean (agente BlueROV2), con la stessa autonomia in simulazione e sul BlueROV2 reale (ArduSub 4.1.2, Navigator, BlueOS 1.3.1).
**Fase:** 1, solo ricognizione e documentazione. Non è stato scritto né eseguito codice e non c'è stato traffico verso <ROV-subnet>.
**Data di raccolta delle fonti:** 2026-10-13
**Ambito:** il progetto resta centrato su HoloOcean. Stonefish e Gazebo servono **solo come riferimento tecnico**.

Convenzioni usate nel testo:
- **[V]** = verificato sulla fonte citata (README, sorgente o API GitHub, articolo).
- **[I]** = inferenza dell'autore del documento a partire dalle fonti, da confermare in Fase 2.
- **[NV]** = non verificabile con le fonti pubbliche accessibili il 2026-10-13.

---

## 1. Sintesi

1. **Il protocollo è comune ai tre simulatori.** Sia il bridge Stonefish (`bvibhav/stonefish_bluerov2`) sia il plugin Gazebo ufficiale (`ArduPilot/ardupilot_gazebo`) parlano con ArduSub SITL tramite il **backend JSON di ArduPilot** (`--model JSON` / `-f JSON`):
   - ArduPilot → simulatore: un pacchetto UDP binario con i PWM (magic 18458, 16 canali).
   - simulatore → ArduPilot: una riga JSON con `timestamp`, `imu.gyro`, `imu.accel_body`, `position`, `attitude` o `quaternion`, `velocity`.
   - Lo stesso schema è stato portato su Unreal Engine 5 da BiguaSim ("ArduBridge"), tecnicamente il parente più stretto di HoloOcean.
   - Fonti: [ArduPilot JSON readme](https://github.com/ArduPilot/ardupilot/blob/master/libraries/SITL/examples/JSON/readme.md), [BiguaSim ArduBridge](https://biguasim.readthedocs.io/en/v1.0.0/ardubridge/ardubridge.html).
2. **Non esiste un bridge pubblico HoloOcean ↔ ArduPilot.** Le ricerche "HoloOcean ArduSub", "HoloOcean SITL", "holoocean ardupilot JSON" e "HoloOcean MAVLink" del 2026-10-13 non hanno dato risultati pertinenti. Il bridge va scritto da zero **[V, assenza di evidenza]**.
3. **ArduSub 4.1.2 ha un'interfaccia JSON più ridotta del master attuale.** Il tag `ArduSub-4.1.2` (commit `2dd0bb7d4c85ac48437f139d66df648fc0e1d4ae`):
   - accetta solo il pacchetto a 16 canali;
   - ha 16 chiavi JSON, con **`position` obbligatoria**;
   - non conosce `no_time_sync`, `no_lockstep`, `rc`, `battery`, `velocity_wind`, `latitude`/`longitude`/`altitude`.
   - Fonti: [SIM_JSON.h @ ArduSub-4.1.2](https://github.com/ArduPilot/ardupilot/blob/ArduSub-4.1.2/libraries/SITL/SIM_JSON.h), [SIM_JSON.h @ master](https://github.com/ArduPilot/ardupilot/blob/master/libraries/SITL/SIM_JSON.h) **[V]**.
4. **HoloOcean ha già un lockstep nativo.** Il server Unreal si blocca a ogni tick in attesa del client Python tramite semafori. È lo stesso modello di sincronizzazione che ArduPilot SITL si aspetta, ma governato da un client diverso: il bridge dovrà fare da "orologio" comune. Fonte: [HoloOcean 2.3.0 – semaphores](https://byu-holoocean.github.io/holoocean-docs/v2.3.0/usage/semaphores.html) **[V]**.
5. **Riuso di codice:**
   - `stonefish_bluerov2` è **Apache-2.0** ed è in pratica uno script Python di circa 200 righe, molto grezzo. Va preso come *riferimento di progetto*, non come codice da riusare.
   - `ardupilot_gazebo` è **LGPL-3.0**, in C++ e legato all'API di Gazebo Sim. Si riusano i *concetti* (lockstep, gestione dei frame persi, trasformazioni ENU→NED), non il codice.

---

## 2. Stonefish: `bvibhav/stonefish_bluerov2`

**Repository e metadati** [V]:
- Repository: https://github.com/bvibhav/stonefish_bluerov2. Licenza **Apache-2.0** ([API](https://api.github.com/repos/bvibhav/stonefish_bluerov2)).
- Creato il 2024-07-08. 8 commit in tutto.
- Ultimo commit `6448383af6b7ef6083b0eac2c08102660591e318` del 2025-02-03, messaggio "Some messy data".
- Commit funzionale più recente sul bridge: `34569b69177d275507400571a18c80f0ebcb2b4e` del 2024-07-12 ("patch namespace fix") ([commits](https://api.github.com/repos/bvibhav/stonefish_bluerov2/commits)).

### 2.1 Architettura del bridge

| File | Ruolo |
|---|---|
| `scripts/ardusim_patch.py` | **Il bridge vero e proprio.** Nodo ROS 2 `ardusim_patch`: socket UDP ↔ topic ROS 2 |
| `launch/bluerov2_sim.py` | Include `stonefish_simulator.launch.py` di `stonefish_ros2` (scena `bluerov2_tank.scn`, `simulation_rate` 100.0) e avvia `ardusim_patch` nel namespace `bluerov2` |
| `launch/blueboat_sim.py`, `launch/bluesim.py` | Varianti BlueBoat/ArduRover |
| `scenarios/*.scn` | Scene Stonefish (XML): veicolo, propulsori, sensori, ambienti (tank, farm, turbine) |
| `data/` | Mesh (BlueROV2, Girona500, BlueBoat, turbine), `bluerov2.urdf` |
| `CMakeLists.txt`, `package.xml` | Pacchetto ament (ROS 2) |

Fonti: [albero del repo](https://api.github.com/repos/bvibhav/stonefish_bluerov2/git/trees/master?recursive=1), [ardusim_patch.py](https://github.com/bvibhav/stonefish_bluerov2/blob/6448383af6b7ef6083b0eac2c08102660591e318/scripts/ardusim_patch.py), [bluerov2_sim.py](https://github.com/bvibhav/stonefish_bluerov2/blob/6448383af6b7ef6083b0eac2c08102660591e318/launch/bluerov2_sim.py).

**Flusso dei dati** [V, salvo dove indicato]:

1. **Ricezione dei PWM.** Il nodo apre un socket UDP con timeout di 0.1 s e decodifica il pacchetto SITL con il formato `struct` `'HHI16H'`:
   - campi: magic, frame_rate, frame_count, 16 PWM;
   - 40 byte, magic **18458**;
   - i pacchetti con lunghezza o magic errati vengono scartati.
2. **Porta UDP.** Dipende dal namespace: 9012 per `bluerov2`, 9002 per `blueboat`.
   - In `main()` il namespace di default è `blueboat`, ma il file di launch imposta `bluerov2`.
   - Con il launch BlueROV2 la porta è quindi **9012**. Questo coincide con l'istanza SITL `-I1` solo se l'offset per istanza è di 10 **[I]**.
   - Il README avvia SITL senza `-I1`: possibile incongruenza **[NV]**.
3. **Mappatura PWM → setpoint.**
   - BlueROV2: `(pwm − 1500)/400` sui canali 0–7, cioè setpoint normalizzati in [−1, 1].
   - BlueBoat: `(pwm − 1500)/600` con banda morta di ±10 µs.
   - Il risultato viene pubblicato su `setpoint/pwm` (`std_msgs/Float64MultiArray`).
4. **Dinamica.** Stonefish legge i setpoint tramite il `ros_subscriber` della scena.
5. **Stato.** Il nodo legge `imu` (`sensor_msgs/Imu`) e `odometry` (`nav_msgs/Odometry`). Sottoscrive anche `gps`, che però resta inutilizzato perché il codice di invio GPS è commentato.
6. **Invio del JSON** all'indirizzo da cui è arrivato il pacchetto SITL, con questi campi:
   - `timestamp` = `get_clock().now()`;
   - `imu.gyro` e `imu.accel_body`;
   - `position`;
   - `attitude` (Eulero ricavato dal quaternione dell'odometria);
   - `velocity`.

Frammento illustrativo (≤10 righe, da `ardusim_patch.py` @ `6448383`, © bvibhav, Apache-2.0):

```python
parse_format = 'HHI16H'
...
gyro = (self.imu.angular_velocity.x, self.imu.angular_velocity.y, -self.imu.angular_velocity.z)
```

**Dipendenza da ROS** [V]:
- Il bridge è un nodo **ROS 2**. Il README indica ROS 2 Jazzy su Ubuntu 24.04, Stonefish e `stonefish_ros2` sul branch `v1.3`.
- Senza ROS 2 non funziona, perché lo stato arriva dai topic di `stonefish_ros2`.
- Fonte: [README](https://github.com/bvibhav/stonefish_bluerov2/blob/6448383af6b7ef6083b0eac2c08102660591e318/README.md).

### 2.2 Controllo dei propulsori

Scena `bluerov2.scn` [V]:
- **8 propulsori** in configurazione Heavy: 4 orizzontali a ±45°/±135° e 4 verticali.
- Parametri: `thrust coefficient` 0.167, `torque coefficient` 0.016, RPM massimi 3600, diametro dell'elica 0.076 m. Il verso è gestito con i flag `inverted`/`right`.
- La scena non specifica la dinamica del rotore (costanti di tempo).
- ArduSub va configurato come `Vectored-6DOF` ([README](https://github.com/bvibhav/stonefish_bluerov2)).

**Catena di comando** [I]: PWM → setpoint normalizzato → comando RPM proporzionale (fino a 3600 RPM) → spinta quadratica nel modello di propulsore di Stonefish, del tipo T = Kt·ρ·D⁴·n|n|.

Stima con i parametri della scena [I]: Kt = 0.167, D = 0.076 m, n = 60 giri/s, ρ = 1000 kg/m³ danno **circa 20 N** a fondo scala, valore inferiore alla spinta nominale di un T200. La curva **non è stata calibrata** sul T200 reale, o almeno il repo non ne dà prova.

### 2.3 Gestione del tempo [V]

- Nessun lockstep esplicito. Un timer ROS 2 a **50 Hz** chiama `looper`, che esegue un `recvfrom` bloccante (fino a 0.1 s) e risponde solo dopo un pacchetto valido. Di fatto il ritmo lo dà SITL.
- Stonefish gira in modo indipendente a `simulation_rate` 100 Hz.
- Il `timestamp` JSON è il clock del nodo ROS: tempo reale, salvo `use_sim_time` impostato dall'esterno.
- In callback ci sono `sleep` bloccanti: 0.5 s se mancano IMU o odometria, 1 s in caso di errore del socket.
- **Conseguenza [I]:** SITL e fisica non sono in lockstep. A 50 Hz di scambio contro i circa 400 Hz attesi da ArduSub, e con un IMU della scena a 20 Hz, il controllo d'assetto è degradato. Questo è coerente con la segnalazione sul forum Blue Robotics "Stabilize and depth hold modes in SITL Stonefish simulator are not working" ([thread](https://discuss.bluerobotics.com/t/stonefish-for-bluerov2/21621/2), contenuto non leggibile, solo il titolo **[NV]**).

### 2.4 Sensori

Sensori dichiarati nella scena `bluerov2.scn` [V]:

| Sensore | Frequenza | Note |
|---|---|---|
| Odometria (ground truth) | 100 Hz | rumore disattivato |
| INS | 100 Hz | — |
| IMU | 20 Hz | — |
| Pressione | 5 Hz | — |
| DVL | 5 Hz | fascio a 30° |
| GPS | 1 Hz | — |
| Stereocamera | 30 Hz | 640×480 |
| Sonar multibeam | 10 Hz | 90°, 0.25–50 m |
| FLS e side-scan | — | commentati |

Cosa arriva davvero ad ArduSub: **solo IMU e odometria** (posizione, assetto, velocità).
- Pressione, DVL e GPS non sono inoltrati.
- La profondità che ArduSub vede deriva dalla `position` z del JSON, tramite il barometro SITL **[I]** (vedi §5.3).

### 2.5 Manipolatore e gripper [V]

**Non è attivo nulla.** I cilindri "manipulator" nella scena sono commentati. Il bridge non gestisce canali per un gripper (per esempio un servo sul canale 9/10 per il Newton Gripper).

### 2.6 Limiti documentati o rilevati

1. **Frame non convertiti in modo completo** [V/I].
   - Viene negata solo la componente z della velocità angolare. Posizione, assetto e accelerazioni passano senza conversione.
   - Il world frame di Stonefish è basato su NED secondo la [documentazione environment](https://stonefish.readthedocs.io/en/latest/environment.html). L'orientamento esatto degli assi dei messaggi ROS di `stonefish_ros2` **[NV]**.
2. **Gravità nell'accelerometro** [V].
   - L'issue #2 (aperta il 2026-06-08) chiede se `bluerov2/imu` pubblichi l'accelerazione con o senza gravità: l'utente ha dovuto aggiungere la gravità per far funzionare il bridge.
   - È un punto critico anche per HoloOcean (§5.2) ([issues](https://api.github.com/repos/bvibhav/stonefish_bluerov2/issues?state=all)).
3. **Robustezza** [V]. Se il namespace non è `bluerov2`/`blueboat`, `PORT` resta indefinita. Le `sleep` bloccano l'executor.
4. **Versioni** [V/NV]. Nessuna versione di ArduSub dichiarata. Per la variante Rover il README cita parametri di ArduRover 4.5.
5. **Manutenzione** [V/NV].
   - Il repo è fermo al 2025-02 con commit "A utter mess" e "Some messy data".
   - Sul forum Blue Robotics un altro repo Stonefish/BlueROV2 (`markusbuchholz/stonefish_ros2_marine_robotics`, ad accesso su richiesta) risulta poi ritirato dall'autore ([thread 18731](https://discuss.bluerobotics.com/t/stonefish-for-bluerov2-and-blueboat/18731), [thread 21621](https://discuss.bluerobotics.com/t/stonefish-for-bluerov2/21621/2)).

### 2.7 Licenze dell'ecosistema Stonefish [V]

| Componente | Licenza | Ultimo push |
|---|---|---|
| Libreria [Stonefish](https://github.com/patrykcieslak/stonefish) | GPL-3.0 | 2026-10-09 |
| [stonefish_ros2](https://github.com/patrykcieslak/stonefish_ros2) | GPL-3.0 | 2025-12-04 |
| `stonefish_bluerov2` | Apache-2.0 | — |

---

## 3. Articolo J. Mar. Sci. Eng. 2026, 14(2), 198

**Metadati** [V] ([Crossref](https://api.crossref.org/works/10.3390/jmse14020198), [Semantic Scholar](https://api.semanticscholar.org/graph/v1/paper/DOI:10.3390/jmse14020198)):
- **Titolo:** "Underwater Object Recovery Using a Hybrid-Controlled ROV with Deep Learning-Based Perception".
- **Autori:** I. Pérez-Edo, S. López-Barajas, R. Marín-Prades, P. J. Sanz (Interactive Robotic Systems Lab, Universitat Jaume I, Castelló).
- **Pubblicazione:** online il 2026-01-18. DOI 10.3390/jmse14020198. Licenza CC BY 4.0.

**Accesso al testo.** Il sito MDPI risponde HTTP 403 ai fetch automatici. Il testo è stato letto tramite il proxy di sola lettura `r.jina.ai` sulla pagina pubblica MDPI. Le citazioni di sezione vanno ricontrollate sul PDF ufficiale **[V con riserva]**.

Non è propriamente un "digital twin Stonefish+SITL". È un sistema di **recupero di oggetti** (una scatola nera) con un BlueROV2 Heavy, un Newton Gripper, percezione YOLOv11 + PnP e un agente ROS in linguaggio naturale (GPT-4.1 mini). La simulazione Stonefish + ArduSub SITL è lo strumento di sviluppo.

### 3.1 Architettura di simulazione (§3.2, Fig. 8)

- Stonefish pubblica IMU, GPS, odometria e propulsori tramite `stonefish_ros`, cioè **ROS 1**.
- Un nodo bridge impacchetta lo stato in JSON per ArduSub SITL e decodifica i PWM binari in risposta.
- Il bridge pubblica un `Float64MultiArray` su `/bluerov/thrusters_setpoints`, che Stonefish usa per applicare le forze.
- È lo **stesso schema** di `bvibhav/stonefish_bluerov2`, ma con ROS Noetic su Ubuntu 20.04 (§3.1).
- L'articolo **non indica** la provenienza del codice del bridge, la versione di Stonefish o di ArduSub, la frequenza di simulazione, il lockstep o le convenzioni di frame **[V, assenza]**.

### 3.2 Controllo (§2.5, §2.2)

- ArduSub opera in **Depth Hold**: profondità, beccheggio e imbardata restano al firmware, mentre l'esterno fornisce riferimenti di moto.
- Un bridge basato su **pymavlink** converte i comandi ROS in PWM. I messaggi MAVLink usati non sono specificati.
- L'autonomia è attiva solo mentre l'operatore tiene premuto un pulsante di abilitazione (modulo "Autonomous Relay").

### 3.3 Validazione e risultati (§4)

| Misura | Valore |
|---|---|
| Errore di posa medio, simulazione | 1.73 cm |
| Errore di posa medio, vasca, modello addestrato solo in sim | 16.58 cm (≈ −0.25 rad) |
| Errore di rotazione in vasca dopo fine-tuning (150 immagini) | ≈ −0.073 rad |
| Errore di posa medio, porto di Castellón (≈ 6 m) | 15.61 cm, 0.1057 rad |
| Prove di presa | 10 in simulazione e 10 nella vasca CIRTESU (12×8×5 m) |
| Successo in vasca con acqua calma (§5) | circa 70–80% |

La tabella 8 non è stata estratta.

**Latenze del sistema reale** (§3.1/§4.4):

| Fase | Latenza |
|---|---|
| Immagine → comando, end-to-end | ≈ 170 ms |
| Video H.264 sul cavo | 40–50 ms |
| YOLO | ≈ 40 ms |
| PnP | ≈ 90 ms |
| Controllo | 0.6 ms |
| Agente LLM (fuori dal loop) | ≈ 5.2 s |

Non ci sono cifre di latenza o di frequenza per il loop SITL.

### 3.4 Limiti dichiarati (§5)

- Visione monoculare sensibile a torbidità e occlusioni.
- Il PID soffre le correnti del sistema di filtrazione della vasca, che è la causa principale dei fallimenti di presa.
- Si assume roll e pitch iniziali nulli. La soglia di 110 px è specifica dell'oggetto.
- Il veicolo non è localizzato in NED.

**Codice e dati.** Sono disponibili solo video, su YouTube e Zenodo (DOI 10.5281/zenodo.17979091). **Il codice non è rilasciato.**

**Rilevanza per noi** [I]:
- Conferma che il pattern "bridge JSON esterno + autonomia via MAVLink/pymavlink sopra ArduSub" funziona in pratica sul BlueROV2.
- Conferma che lo stesso stack si porta sul veicolo reale cambiando solo l'endpoint MAVLink. È l'obiettivo del nostro progetto.

---

## 4. Gazebo

### 4.1 Plugin ufficiale `ArduPilot/ardupilot_gazebo`

**Metadati** [V]:
- Repository: https://github.com/ArduPilot/ardupilot_gazebo. Licenza **LGPL-3.0**.
- Ultimo commit `082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5` del 2026-04-02.
- Il commit `65937b77aace16735df6f192badb0e6b4eddd056` (2026-02-12) aggiunge l'opzione `<no_time_sync>` ([commits](https://api.github.com/repos/ArduPilot/ardupilot_gazebo/commits?per_page=3)).
- Gazebo supportati: Garden, Harmonic (raccomandato), Ionic, Jetty, su Ubuntu 22.04 e macOS. Non dipende da ROS ([README](https://github.com/ArduPilot/ardupilot_gazebo), [ArduPilot docs](https://ardupilot.org/dev/docs/sitl-with-gazebo.html)).
- La documentazione ArduPilot su Gazebo **non cita ArduSub né WSL2** [V].

**`ArduPilotPlugin.cc`** [V] ([sorgente](https://github.com/ArduPilot/ardupilot_gazebo/blob/082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5/src/ArduPilotPlugin.cc)):

- **Parametri:**
  - globali: `fdm_addr` (127.0.0.1), `fdm_port_in` (9002), `lock_step`, `connectionTimeoutMaxCount`, `imuName`, `modelXYZToAirplaneXForwardZDown`, `gazeboXYZToNED`, `have_32_channels`, `no_time_sync`;
  - per ogni `<control>`: `channel`, `type` (VELOCITY/POSITION/EFFORT/COMMAND), `servo_min/max`, `multiplier`, `offset`, `cmd_topic`, guadagni PID.
- **Mappatura PWM:**
  - `raw = clamp((pwm − servo_min)/(servo_max − servo_min), 0, 1)`, poi `cmd = multiplier·(raw + offset)`.
  - PWM = 0 vuol dire canale non pronto, comando 0.
- **Lockstep:**
  - Con `lock_step` attivo, `PreUpdate` resta in loop su `ReceiveServoPacket()` finché arriva un pacchetto. Poi applica le forze con il dt di simulazione.
  - `PostUpdate` invia il JSON con `timestamp` = **tempo di simulazione**.
  - Il blocco permette anche il debug con GDB.
- **Frame:**
  - posizione e assetto passano per la composizione di trasformazioni `wldAToWldG · wldGToBdyG · bdyAToBdyG⁻¹` (ENU/FLU di Gazebo → NED/FRD di ArduPilot), con quaternione in ordine W, X, Y, Z;
  - la velocità viene ruotata con `wldAToWldG`;
  - l'IMU passa invariata: si assume che il sensore sia già montato in FRD (roll di π sulla posa del sensore).
- **Robustezza:**
  - frame duplicati: avviso e reinvio dell'ultimo stato;
  - frame saltati: conteggio e avviso;
  - `frame_count` che decresce: reset del controllore;
  - coda dei pacchetti: si tiene solo il più recente;
  - timeout oltre `connectionTimeoutMaxCount`: "Broken ArduPilot connection, resetting motor control".
- **Campi JSON inviati:**
  - `timestamp`, `imu`, `position`, `quaternion`, `velocity`;
  - `rng_1..6`, con `inf` sostituito da 2×range_max;
  - `windvane`;
  - `no_time_sync`, `no_lockstep`.

### 4.2 Modelli BlueROV2 per Gazebo

| Repo | Stato e licenza | Contenuto rilevante |
|---|---|---|
| [clydemcqueen/bluerov2_gz](https://github.com/clydemcqueen/bluerov2_gz) (`bluerov2_ignition` reindirizza qui) | Attivo: ultimo commit `4451a5d1b15d204581d43ef7d93df4a0508eddbb` del 2026-10-09. **Nessuna licenza** (`license: null`) [V] | Gazebo Harmonic, "proof-of-concept". Usa `Thruster`, `Hydrodynamics` e `Buoyancy` di Gazebo Sim, ArduSub `vectored`/`vectored_6dof` ([README](https://github.com/clydemcqueen/bluerov2_gz)) |
| [clydemcqueen/orca4](https://github.com/clydemcqueen/orca4) | MIT. Ultimo commit `eea318ed440774a881be6c6904708893598025d2` del 2026-06-22 [V] | ROS 2 Humble + Gazebo Harmonic + ArduSub tramite `ardupilot_gazebo`. Profondità dal barometro, assetto dall'IMU, SLAM stereo (ORB_SLAM2). Il README lo dichiara superato da Orca5 |
| [Robotic-Decision-Making-Lab/blue](https://github.com/Robotic-Decision-Making-Lab/blue) | MIT. Ultimo commit `812fea605cd2056ba20fac504f456457594595fb` del 2025-07-26 [V] | ROS 2 Humble/Iron, Gazebo Harmonic+, `ardusub_driver`. Il launch con `use_sim:=true` avvia Gazebo + ArduSub + MAVROS. Solo Linux. Cita Stonefish e **HoloOcean** come alternative "non testate" ([tutorial](https://robotic-decision-making-lab.github.io/blue/tutorials/simulation/), [installazione](https://robotic-decision-making-lab.github.io/blue/installation/)) |
| [patrickelectric/bluerov_ros_playground](https://github.com/patrickelectric/bluerov_ros_playground) | Nessuna licenza. Ultimo push 2019-02-20 [V] | Gazebo Classic + `freebuoyancy_gazebo` + branch `add_link` del vecchio plugin, `-f gazebo-bluerov2`: storico, obsoleto |
| [markusbuchholz/gazebosim_bluerov2_ardupilot_sitl](https://discuss.bluerobotics.com/t/gazebosim-and-sitl-for-blueboat-and-bluerov2/18730) | Annunciato il 2024-11-30 | Versioni e plugin non dichiarati **[NV]** |

**Dettagli di `bluerov2_gz/models/bluerov2/model.sdf`** [V] ([file](https://github.com/clydemcqueen/bluerov2_gz/blob/4451a5d1b15d204581d43ef7d93df4a0508eddbb/models/bluerov2/model.sdf)):

- **Massa e inerzia:** 10 kg; Ixx/Iyy/Izz = 0.099/0.178/0.269.
- **Idrodinamica** (Fossen, solo termini quadratici, massa aggiunta nulla):

  | Coefficiente | Valore |
  |---|---|
  | `xUabsU` | −33.732 |
  | `yVabsV` | −54.16 |
  | `zWabsW` | −73.225 |
  | `kPabsP`, `mQabsQ`, `nRabsR` | −3.992 ciascuno |

- **Propulsori:** `Thruster` con `thrust_coefficient` ±0.02 (il segno distingue eliche CW e CCW), D = 0.1 m, `velocity_control` true, `water_density` 998.
- **ArduPilotPlugin:**
  - `lock_step` 1;
  - `gazeboXYZToNED` = `0 0 0 3.142 0 1.571`;
  - `modelXYZToAirplaneXForwardZDown` = `0 0 0 3.142 0 0`;
  - canali 0–5 → `COMMAND` su `/model/bluerov2/joint/thrusterN_joint/cmd_thrust`, con servo 1100–1900, `offset` −0.5, `multiplier` 100. Con la formula del §4.1 questo dà **±50 N di spinta comandata, lineare nel PWM** [I].
- **IMU:** 1000 Hz con roll π per allinearla a FRD.
- **Assenze:** nessun sensore di profondità o pressione dedicato. ArduSub ricava la profondità dal barometro SITL [I].
- **Galleggiamento:** il modello cita il `BuoyancyPlugin` (volume di collisione × densità), ma il plugin non è dichiarato nel `model.sdf`. Si presume sia nel world file [I]. L'ultimo commit dice "Calculate displacement from total mass".

**Manipolatori su Gazebo:**
- Nessuno dei repo BlueROV2+ArduSub elencati gestisce in modo documentato un Newton Gripper o un Reach Alpha via ArduSub [V, assenza].
- `blue` cita in bibliografia lavori su "Lightweight Underwater Vehicle Manipulator Systems", ma il README non nomina il manipolatore **[NV]**.

---

## 5. HoloOcean 2.3.0: cosa offre e cosa manca

Le informazioni dettagliate su HoloOcean sono raccolte negli altri documenti di Fase 1. Qui servono solo i punti per il confronto.

### 5.1 Capacità rilevanti [V]

| Aspetto | Cosa dice la documentazione 2.3.0 |
|---|---|
| Rilascio e motore | 2.3.0 uscita il 2026-02-05. UE 5.3 dalla 2.0.0 ([changelog](https://byu-holoocean.github.io/holoocean-docs/v2.3.0/changelog/changelog.html)) |
| Sistemi operativi | Linux e Windows a 64 bit. Docker su host Windows può avere problemi di accesso alla GPU. Il container runtime non è distribuito per via dell'EULA Epic ([installazione](https://byu-holoocean.github.io/holoocean-docs/v2.3.0/usage/installation.html)) |
| Agente BlueROV2 | **8 propulsori**. Ordine: dal verticale anteriore destro in senso antiorario, poi i 4 orizzontali |
| Schemi di controllo dell'agente BlueROV2 | 0 = forze ai propulsori; 1 = PID su posa globale; 2 = Custom Dynamics (accelerazioni nel frame globale, gravità, galleggiamento e smorzamento disattivati) ([pagina agente](https://byu-holoocean.github.io/holoocean-docs/v2.3.0/agents/agents/blue-rov-agent.html), [control schemes](https://byu-holoocean.github.io/holoocean-docs/v2.3.0/agents/docs/control-schemes.html)) |
| Unità dei comandi | **Non indicate** nella pagina dell'agente **[NV]** |
| Frame | Destrorso, **x avanti, y sinistra, z su (NWU/FLU)**, metri, rotazioni [roll, pitch, yaw] in **gradi** ([coordinate](https://byu-holoocean.github.io/holoocean-docs/v2.3.0/usage/units-and-coordinates.html)). L'`IMUSocket` del BlueROV2 è ruotato di 180° su x, quindi x avanti, y destra, z giù |
| Tempo | `ticks_per_sec` (default 30) fissa la risoluzione temporale. `frames_per_sec` true/false/numero limita o sblocca il rendering, per cui senza limite si può andare più veloci del tempo reale. `Hz` dei sensori ≤ `ticks_per_sec` ([scenari](https://byu-holoocean.github.io/holoocean-docs/v2.3.0/usage/scenarios.html)) |
| Sincronizzazione | `act`/`tick`/`step` ([environments](https://byu-holoocean.github.io/holoocean-docs/v2.3.0/usage/environments.html)). Lockstep a semafori server/client ([semaphores](https://byu-holoocean.github.io/holoocean-docs/v2.3.0/usage/semaphores.html)) |
| Sensori | DVL, IMU, camera, sonar (imaging, profiling, sidescan, echosounder), profondità, GPS, LiDAR ray-cast, depth camera e segmentazione (2.3.0) |
| ROS 2 | Bridge ROS 2 dalla 2.0.0 |
| Manipolatore | **Nessun manipolatore o gripper** documentato per BlueROV2 |
| ArduPilot / MAVLink | **Nessuna menzione** |
| Licenza | Repo accessibile solo collegando l'account GitHub a Epic Games; l'API GitHub pubblica risponde 404. Licenza del codice client **[NV]**. Le world package sono soggette all'EULA di Unreal |

### 5.2 Implicazioni per il bridge [I]

1. **Frame da convertire in modo completo**, più rigorosamente di quanto faccia `ardusim_patch.py`:
   - world NWU → NED: rotazione di π attorno a x;
   - body FLU → FRD: rotazione di π attorno a x;
   - angoli da gradi a radianti;
   - BiguaSim usa esattamente `R_frd2ned = Rx(π)·R_flu2nwu·Rx(π)`, nella sua notazione GLU/NWU ([ArduBridge](https://biguasim.readthedocs.io/en/v1.0.0/ardubridge/ardubridge.html)).
2. **`accel_body` deve essere forza specifica** (accelerazione senza la gravità, cioè +g sull'asse z in giù a riposo, ovvero −9.81 in FRD).
   - Va verificato se l'IMU di HoloOcean include la gravità. L'issue #2 di `stonefish_bluerov2` mostra che questo è un errore tipico.
   - ArduSub 4.1.2 copia `accel_body` senza correzioni ([SIM_JSON.cpp @ ArduSub-4.1.2](https://github.com/ArduPilot/ardupilot/blob/ArduSub-4.1.2/libraries/SITL/SIM_JSON.cpp)).
3. **Orologio comune.** Ogni pacchetto PWM ricevuto deve corrispondere a un `step()` di HoloOcean con dt = 1/`ticks_per_sec`, e il `timestamp` JSON deve essere il **tempo di simulazione**, strettamente crescente.
   - Un timestamp costante blocca SITL ("link 1 down"), come documentato sul [forum Blue Robotics, 2021-11](https://discuss.bluerobotics.com/t/link-1-down-running-ardusub-sitl-with-custom-physics-sim-using-json-frame/11109).
   - Un timestamp che torna indietro azzera dt ("Physics time has gone backwards").
4. **Frequenza.**
   - ArduSub accetta un rate fisico ≠ 400 Hz: SIM_JSON imposta il rate a 1/dt per dt tra 0 e 0.1 s.
   - Il loop principale però resta a 400 Hz. Un `ticks_per_sec` di 30 (default HoloOcean) implicherebbe **più cicli ArduSub per ogni tick** e quindi un controllo d'assetto meno fedele.
   - Da misurare in Fase 2 il tick rate sostenibile con il rendering attivo (probabilmente 100–400 Hz solo con i sensori ottici a bassa frequenza).
5. **Mappatura PWM → comando di propulsore HoloOcean.** Lo schema 0 accetta valori la cui unità non è documentata (verosimilmente forze in N **[NV]**). Serve:
   - una curva PWM → spinta del T200 (dai dati Blue Robotics) invece della semplice mappa lineare `(pwm−1500)/400` di Stonefish;
   - il riordino dei canali ArduSub `vectored_6dof` (motori 1–8) nell'ordine dei propulsori HoloOcean.

### 5.3 Profondità in ArduSub SITL [V/I]

- In ArduSub 4.1.2 il barometro SITL usa `AP_Baro::SimpleUnderWaterAtmosphere` su `sitl->state.altitude` ([AP_Baro_SITL.cpp @ ArduSub-4.1.2](https://github.com/ArduPilot/ardupilot/blob/ArduSub-4.1.2/libraries/AP_Baro/AP_Baro_SITL.cpp)) **[V]**.
- Quindi la profondità percepita da ArduSub deriva dalla `position` D del JSON e non dal sensore di profondità di HoloOcean **[I]**. Il passaggio da `position` a `state.altitude` non è stato verificato nel sorgente.
- Per la coerenza sim-reale va deciso se iniettare la profondità del `DepthSensor` HoloOcean nella componente D di `position`.

---

## 6. Tabella comparativa

| Criterio | **HoloOcean 2.3.0** (target) | **Stonefish** (`bvibhav/stonefish_bluerov2`) | **Gazebo** (`ardupilot_gazebo` + `bluerov2_gz`/`orca4`/`blue`) |
|---|---|---|---|
| **Architettura del bridge** | **Non esiste.** Da creare: processo Python che usa l'API `holoocean` (step lockstep) e fa da server UDP JSON per SITL [V, assenza] | Nodo ROS 2 Python esterno (`ardusim_patch.py`): UDP ↔ topic ROS 2 ↔ `stonefish_ros2` [V] | Plugin C++ di sistema dentro il processo Gazebo (`ArduPilotPlugin`): UDP diretto, nessun ROS [V] |
| **Componenti riutilizzabili** | API Python `holoocean`, sensori nativi, schema 0 a propulsori | Logica del bridge come esempio (Apache-2.0); scena `.scn` come fonte di parametri | Algoritmi del plugin (lockstep, frame, gestione dei frame persi) come riferimento; coefficienti di Fossen di `bluerov2_gz` come dati di partenza |
| **Controllo dei propulsori** | Schema 0: vettore di 8 comandi per propulsore (unità non documentate) [V/NV] | PWM → `(pwm−1500)/400` → setpoint [−1, 1] → RPM (max 3600) → spinta quadratica di Stonefish (Kt 0.167, D 0.076) [V/I] | PWM → `multiplier·(raw+offset)` → `cmd_thrust` (±50 N in `bluerov2_gz`) → plugin `Thruster` (Kt 0.02, D 0.1, controllo in velocità) [V/I] |
| **Idrodinamica** | Modelli di Fossen integrati dalla 2.0 (dettagli BlueROV2 **[NV]**) | Calcolata dalla geometria delle mesh (`bluerov2_phy.obj`), galleggiamento con volumi espliciti [V] | Plugin `Hydrodynamics` (smorzamento quadratico di Fossen) + `Buoyancy` [V] |
| **Gestione del tempo** | Lockstep nativo client/server; `ticks_per_sec` (default 30); possibile più veloce del tempo reale [V] | **Nessun lockstep**: timer a 50 Hz, tempo reale, Stonefish a 100 Hz in modo indipendente [V] | **Lockstep** opzionale (`lock_step`=1 in `bluerov2_gz`), timestamp = tempo di simulazione [V] |
| **Sensori verso ArduSub** | Da progettare: IMU (FRD), stato ground truth o Dynamics sensor, profondità [I] | Solo IMU + odometria; pressione, DVL e GPS non inoltrati [V] | IMU + posa e velocità di ground truth + `rng_1..6` + windvane [V] |
| **Sensori per l'autonomia** (fuori da ArduSub) | Ricchi: camera, sonar (4 tipi), DVL, LiDAR, profondità; ROS 2 [V] | Stereocamera, multibeam, DVL, INS, GPS su ROS 2 [V] | Camera (streaming GStreamer), stereo (orca4) [V] |
| **Manipolatore / gripper** | Nessuno per il BlueROV2 [V] | Nessuno attivo; l'articolo JMSE usa un Newton Gripper reale, la modellazione in sim non è descritta [V] | Nessuno nei repo BlueROV2+ArduSub esaminati [V] |
| **Limiti documentati** | Unità dei comandi non documentate; Docker+GPU su Windows problematico [V] | Frame parziali, gravità nell'IMU, `sleep` bloccanti, codice "messy" [V] | `bluerov2_gz` "proof-of-concept", modelli da tarare; nel plugin c'è un possibile errore nella trasformazione delle velocità (somma di una traslazione) [V] |
| **Licenza** | Client **[NV]**; binari e mondi soggetti all'EULA Epic | Bridge Apache-2.0; Stonefish e `stonefish_ros2` GPL-3.0 [V] | Plugin LGPL-3.0; `bluerov2_gz` senza licenza; orca4 e blue MIT [V] |
| **Dipendenze** | Python ≥ 3.7, GPU, UE 5.3 runtime, account Epic [V] | ROS 2 Jazzy, Stonefish v1.3, `stonefish_ros2` v1.3, colcon [V] | Gazebo Garden/Harmonic+, `gz-sim7/8`, opzionalmente ROS 2 Humble + `ros_gz` [V] |
| **Versioni** | 2.3.0 del 2026-02-05 [V] | HEAD `6448383` (2025-02-03); ArduSub non dichiarato [V] | Plugin `082a0fe` (2026-04-02); `bluerov2_gz` `4451a5d` (2026-10-09) [V] |
| **Sistemi operativi** | **Windows 11 nativo** (target) e Linux [V] | Ubuntu 24.04, l'unico testato. Su Windows solo tramite WSL2 con GUI OpenGL [V/I] | Ubuntu 22.04, macOS. Su Windows solo WSL2/Docker [V/I] |
| **Topologia sul nostro host** | HoloOcean su Windows ↔ UDP ↔ SITL in WSL2/Docker. Serve gestire IP e NAT di WSL2: il bridge risponde all'indirizzo sorgente del pacchetto PWM [I] | Tutto in Linux | Tutto in Linux |
| **ArduSub 4.1.x** | Compatibile se il bridge invia solo le chiavi presenti in 4.1.2 (con `position` obbligatoria) e usa il pacchetto a 16 canali [V/I] | Compatibile in principio: usa solo il pacchetto a 16 canali e le chiavi di base [I] | Il plugin attuale invia anche `no_time_sync`/`no_lockstep`, che 4.1.2 **ignora** (il parser cerca solo le chiavi note) [V]. Il resto è compatibile [I] |
| **ArduSub più recente** (master) | Possibili 32 canali (`SERVO_32_ENABLE`), `rc`, `battery`, lat/lon/alt [V] | Idem | Idem; `no_time_sync` serve agli scenari senza sincronizzazione del tempo |

---

## 7. Altri tentativi con Unreal o Unity [V]

| Progetto | Motore | Bridge ArduSub SITL | Note |
|---|---|---|---|
| **BiguaSim** (Nautec/FURG) | UE 5.3, Linux/Windows | **Sì: "ArduBridge"**, Python. UDP 9002, magic 18458, profili di veicolo PWM→attuatore, conversione NWU/FLU→NED/FRD, profondità → pressione (ρ = 1026), "warmup guard" con motori a zero per circa 1700 frame durante il boot di SITL, mock di test JSON e MAVLink ([doc](https://biguasim.readthedocs.io/en/v1.0.0/ardubridge/ardubridge.html), [index](https://biguasim.readthedocs.io/en/latest/)) | Agenti BlueROV2, BlueROVHeavy, BlueBoat. La documentazione rimanda a Holodeck (stessa ascendenza di HoloOcean). Che derivi da HoloOcean, la licenza e l'URL del repo: **[NV]**. È **il riferimento progettuale più vicino** |
| **UNav-Sim** ([repo](https://github.com/open-airlab/UNav-Sim)) | UE 5.1 + AirSim | Nessuno documentato | ROS 1/2, Linux/Windows |
| **MARUS** | Unity | Nessuno documentato; in parte proprietario ([SMaRCSim survey](https://arxiv.org/html/2506.07781v1)) | — |
| **unity-arduboat-sim** ([repo](https://github.com/chrisdalke/unity-arduboat-sim)) | Unity | JSON SITL per ArduRover (boat) | Esempio minimale di bridge JSON con Unity |
| **DAVE** ([repo](https://github.com/Field-Robotics-Lab/dave)) | Gazebo (Classic) | Nessuno documentato nel README | Apache-2.0 |
| **BlueSim** ([repo](https://github.com/bluerobotics/bluesim)) | Non verificato | Avvia un'istanza SITL propria | **[NV]** |

---

## 8. Componenti effettivamente riutilizzabili per HoloOcean

### 8.1 Concetti e pattern di progetto (nessun vincolo di licenza)

1. **Protocollo JSON SITL** come unico confine tra autopilota e fisica.
   - È documentato da ArduPilot ([readme](https://github.com/ArduPilot/ardupilot/blob/master/libraries/SITL/examples/JSON/readme.md)).
   - Implementarlo da zero a partire dalla specifica evita qualsiasi derivazione di codice.
   - ArduPilot (GPL-3.0) gira come **processo separato** che comunica via UDP, quindi non c'è linking e la GPL non si propaga al bridge [I, interpretazione corrente della FSF sulla comunicazione tra processi distinti].
2. **Lockstep "un pacchetto PWM = un passo fisico"**, come `ArduPilotPlugin` con `lock_step` = 1, ma con `env.step()` di HoloOcean al posto di `PreUpdate`/`PostUpdate`. HoloOcean è già bloccante, quindi l'adattamento è naturale [I].
3. **Gestione dei frame**, ripresa dal plugin Gazebo: rilevare duplicati, salti e reset tramite `frame_count`; scartare i pacchetti in coda tenendo il più recente; timeout con azzeramento dei motori.
4. **Trasformazioni di frame esplicite e parametrizzate**, alla maniera di `gazeboXYZToNED`/`modelXYZToAirplaneXForwardZDown` e di `R_frd2ned` di BiguaSim, con test unitari dedicati. È la lezione principale contro gli errori di `ardusim_patch.py`.
5. **Warmup guard** (BiguaSim): motori a zero durante il boot di SITL.
6. **Mock del backend SITL** (BiguaSim `mock_bridge_json.py`): permette di provare il bridge senza HoloOcean, utile in CI su WSL2.
7. **Separazione dei livelli**, come nell'articolo JMSE: bridge fisico (JSON) distinto da autonomia via MAVLink/pymavlink. L'autonomia parla **solo MAVLink**, quindi gira identica su SITL e sul BlueROV2 reale.

### 8.2 Codice e dati

| Fonte | Licenza | Cosa si può riusare | Note di compatibilità |
|---|---|---|---|
| `bvibhav/stonefish_bluerov2/scripts/ardusim_patch.py` | Apache-2.0 | Al massimo la struttura `struct.unpack('HHI16H')` e la risposta all'indirizzo sorgente (righe banali, comunque sconsigliato) | Apache-2.0 è compatibile verso GPL-3/MIT con NOTICE e attribuzione. Il valore del codice è basso: meglio riscrivere |
| `ArduPilot/ardupilot_gazebo` | LGPL-3.0 | Nessuna riga di codice: è C++ legato a `gz-sim` | Un porting riga per riga in Python sarebbe opera derivata LGPL. Va reimplementato dalla specifica |
| `ArduPilot/ardupilot` `libraries/SITL/examples/JSON/*.py` | GPL-3.0 | Esempi Python ufficiali del backend JSON | Se copiati, il bridge diventa GPL-3.0. Accettabile se il progetto è GPL, altrimenti usarli solo come riferimento |
| `clydemcqueen/bluerov2_gz` `model.sdf` | **Nessuna licenza** (tutti i diritti riservati) | Solo i *valori numerici* (massa, inerzia, coefficienti di Fossen) come dati di partenza, citandone la fonte | Non copiare i file. I coefficienti vanno ricalibrati comunque |
| `clydemcqueen/orca4`, `Robotic-Decision-Making-Lab/blue` | MIT | Layer di autonomia ROS 2 ↔ MAVROS/ArduSub, launch con `use_sim` | MIT è compatibile con tutto. Utile per lo strato di autonomia, non per il bridge fisico |
| Stonefish, `stonefish_ros2` | GPL-3.0 | Nulla (non vengono usati) | — |
| BiguaSim ArduBridge | **[NV]** | Solo il design (documentazione pubblica) | Licenza non trovata: non copiare codice |

---

## 9. Lezioni apprese e rischi

| # | Rischio o lezione | Evidenza | Mitigazione proposta (Fase 2+) |
|---|---|---|---|
| R1 | **Convenzioni di frame sbagliate**: ArduSub diverge o i modi STABILIZE/ALT_HOLD non tengono | `ardusim_patch.py` nega solo `gyro.z`; thread "Stabilize and depth hold … not working" | Matrice di conversione NWU/FLU→NED/FRD unica e testata; test statici (veicolo fermo → `accel_body` ≈ [0, 0, −9.81], yaw 0 = Nord) |
| R2 | **Gravità e forza specifica nell'IMU** | Issue #2 di `stonefish_bluerov2` | Verificare l'output dell'IMU HoloOcean a veicolo fermo; in alternativa derivare `accel_body` dal Dynamics sensor |
| R3 | **Tempo non monotono o non sincronizzato** | Forum BR 2021 "link 1 down" (timestamp costante); SIM_JSON "Physics time has gone backwards" | `timestamp` = numero di tick × dt, strettamente crescente; nessun uso del clock di sistema |
| R4 | **Tick rate di HoloOcean basso** (default 30) contro ArduSub a 400 Hz | Documentazione HoloOcean scenari; SIM_JSON di 4.1.2 adatta il rate fisico | Misurare il tick rate massimo con rendering ridotto; valutare 100–200 Hz; camere e sonar a bassa frequenza |
| R5 | **Unità ignote dei comandi ai propulsori** in HoloOcean | Pagina dell'agente BlueROV2 | Taratura empirica (spinta statica) e confronto con la curva T200; mappatura canali `vectored_6dof` → ordine HoloOcean |
| R6 | **Differenze 4.1.2 vs master** | SIM_JSON.h ai due tag | Il bridge invia solo le chiavi presenti in 4.1.2 (`position` sempre); test sia su `ArduSub-4.1.2` sia su master |
| R7 | **Rete Windows ↔ WSL2/Docker** (NAT, firewall, IP variabile di WSL2) | Topologia del progetto; ArduPilot risponde all'indirizzo sorgente | Bridge in esecuzione su Windows accanto a HoloOcean, SITL in WSL2 con `--model JSON:<IP host Windows>`; documentare le regole del firewall. Modalità "mirrored networking" di WSL2 **[NV]** |
| R8 | **Profondità e pressione**: ArduSub SITL la ricava da `position.z` e non dal sensore di HoloOcean | `AP_Baro_SITL.cpp` @ 4.1.2 | Decidere la fonte (ground truth oppure DepthSensor) e documentarla; aggiungere rumore e ritardo coerenti con il Bar30 reale |
| R9 | **Gripper e manipolatore**: nessun simulatore di riferimento lo modella insieme ad ArduSub | §2.5, §4.2, §5.1 | Fase successiva: leggere il canale servo del gripper dal pacchetto PWM (come sul Navigator) e attuarlo in HoloOcean con un attore o socket dedicato |
| R10 | **Licenze**: copiare esempi GPL-3 di ArduPilot o porting LGPL rende il bridge un'opera derivata | §8.2 | Implementazione "clean room" dalla specifica JSON, con licenza del progetto da decidere (es. MIT o BSD-3) |
| R11 | **Repo di riferimento instabili o ritirati** (Stonefish BlueROV2 di Buchholz ritirato, bvibhav "messy") | Forum BR 18731/21621; log dei commit | Ancorare ogni riferimento a un hash di commit; non dipendere da quei repo a runtime |
| R12 | **Il sim-to-real dell'articolo JMSE riguarda la percezione, non la dinamica** | §4 dell'articolo | Prevedere in Fase 3 una validazione dinamica dedicata (step di spinta, decadimento in heave e yaw) confrontata con il BlueROV2 reale |

---

## 10. Elenco delle evidenze (per il manifest JSON)

Data di recupero: **2026-10-13** per tutte le voci.

| id | URL | Tipo | Commit / versione / data della fonte |
|---|---|---|---|
| E01 | https://github.com/bvibhav/stonefish_bluerov2 | repo README | HEAD `6448383af6b7ef6083b0eac2c08102660591e318` (2025-02-03) |
| E02 | https://api.github.com/repos/bvibhav/stonefish_bluerov2 | metadati API | Apache-2.0; creato 2024-07-08; push 2025-02-03 |
| E03 | https://api.github.com/repos/bvibhav/stonefish_bluerov2/commits | log dei commit | 8 commit, `1d79a15…` → `6448383…` |
| E04 | https://raw.githubusercontent.com/bvibhav/stonefish_bluerov2/master/scripts/ardusim_patch.py | sorgente | @ `6448383` |
| E05 | https://raw.githubusercontent.com/bvibhav/stonefish_bluerov2/master/scenarios/bluerov2.scn | scena | @ `6448383` |
| E06 | https://raw.githubusercontent.com/bvibhav/stonefish_bluerov2/master/launch/bluerov2_sim.py | launch | @ `6448383` |
| E07 | https://api.github.com/repos/bvibhav/stonefish_bluerov2/issues?state=all | issue | #1 (2025-04-24), #2 (2026-06-08) |
| E08 | https://api.github.com/repos/patrykcieslak/stonefish | metadati API | GPL-3.0; push 2026-10-09 |
| E09 | https://api.github.com/repos/patrykcieslak/stonefish_ros2 | metadati API | GPL-3.0; push 2025-12-04 |
| E10 | https://stonefish.readthedocs.io/en/latest/environment.html | documentazione | origine NED (frame) |
| E11 | https://www.mdpi.com/2077-1312/14/2/198 | articolo (letto tramite r.jina.ai, perché MDPI dà 403) | DOI 10.3390/jmse14020198, pubblicato 2026-01-18 |
| E12 | https://api.crossref.org/works/10.3390/jmse14020198 | metadati | CC BY 4.0, 34 riferimenti |
| E13 | https://api.semanticscholar.org/graph/v1/paper/DOI:10.3390/jmse14020198 | metadati | CorpusId 284908789 |
| E14 | https://github.com/ArduPilot/ardupilot/blob/master/libraries/SITL/examples/JSON/readme.md | specifica JSON | master al 2026-10-13 |
| E15 | https://github.com/ArduPilot/ardupilot/blob/ArduSub-4.1.2/libraries/SITL/SIM_JSON.h | sorgente | tag `ArduSub-4.1.2` = `2dd0bb7d4c85ac48437f139d66df648fc0e1d4ae` |
| E16 | https://github.com/ArduPilot/ardupilot/blob/ArduSub-4.1.2/libraries/SITL/SIM_JSON.cpp | sorgente | `2dd0bb7d…` |
| E17 | https://github.com/ArduPilot/ardupilot/blob/master/libraries/SITL/SIM_JSON.h | sorgente | master al 2026-10-13 |
| E18 | https://github.com/ArduPilot/ardupilot/blob/ArduSub-4.1.2/libraries/AP_Baro/AP_Baro_SITL.cpp | sorgente | `2dd0bb7d…` |
| E19 | https://ardupilot.org/dev/docs/sitl-with-gazebo.html | documentazione | — |
| E20 | https://ardupilot.org/dev/docs/sitl-with-JSON.html | documentazione | — |
| E21 | https://github.com/ArduPilot/ardupilot_gazebo | repo README | HEAD `082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5` (2026-04-02); LGPL-3.0 |
| E22 | https://raw.githubusercontent.com/ArduPilot/ardupilot_gazebo/main/src/ArduPilotPlugin.cc | sorgente | @ `082a0fe` |
| E23 | https://api.github.com/repos/ArduPilot/ardupilot_gazebo/commits?per_page=3 | log | `65937b77…` (no_time_sync, 2026-02-12) |
| E24 | https://github.com/clydemcqueen/bluerov2_gz | repo README | HEAD `4451a5d1b15d204581d43ef7d93df4a0508eddbb` (2026-10-09); senza licenza |
| E25 | https://raw.githubusercontent.com/clydemcqueen/bluerov2_gz/main/models/bluerov2/model.sdf | modello | @ `4451a5d` |
| E26 | https://api.github.com/repos/clydemcqueen/bluerov2_ignition | metadati | reindirizza a `bluerov2_gz` (stesso push 2026-10-10) |
| E27 | https://github.com/clydemcqueen/orca4 | repo README | HEAD `eea318ed440774a881be6c6904708893598025d2` (2026-06-22); MIT |
| E28 | https://github.com/Robotic-Decision-Making-Lab/blue | repo README | HEAD `812fea605cd2056ba20fac504f456457594595fb` (2025-07-26); MIT |
| E29 | https://robotic-decision-making-lab.github.io/blue/tutorials/simulation/ | documentazione | — |
| E30 | https://robotic-decision-making-lab.github.io/blue/installation/ | documentazione | — |
| E31 | https://api.github.com/repos/patrickelectric/bluerov_ros_playground | metadati | push 2019-02-20; senza licenza |
| E32 | https://discuss.bluerobotics.com/t/gazebosim-and-sitl-for-blueboat-and-bluerov2/18730 | forum | 2024-11-30 |
| E33 | https://discuss.bluerobotics.com/t/stonefish-for-bluerov2-and-blueboat/18731 | forum | 2024-12-01 → 2025-01 |
| E34 | https://discuss.bluerobotics.com/t/stonefish-for-bluerov2/21621/2 | forum | 2025-09-19 |
| E35 | https://discuss.bluerobotics.com/t/link-1-down-running-ardusub-sitl-with-custom-physics-sim-using-json-frame/11109 | forum | 2021-11-29/30 |
| E36 | https://byu-holoocean.github.io/holoocean-docs/v2.3.0/index.html | documentazione | v2.3.0 |
| E37 | https://byu-holoocean.github.io/holoocean-docs/v2.3.0/agents/agents/blue-rov-agent.html | documentazione | v2.3.0 |
| E38 | https://byu-holoocean.github.io/holoocean-docs/v2.3.0/agents/docs/control-schemes.html | documentazione | v2.3.0 |
| E39 | https://byu-holoocean.github.io/holoocean-docs/v2.3.0/usage/units-and-coordinates.html | documentazione | v2.3.0 |
| E40 | https://byu-holoocean.github.io/holoocean-docs/v2.3.0/usage/scenarios.html | documentazione | v2.3.0 |
| E41 | https://byu-holoocean.github.io/holoocean-docs/v2.3.0/usage/environments.html | documentazione | v2.3.0 |
| E42 | https://byu-holoocean.github.io/holoocean-docs/v2.3.0/usage/semaphores.html | documentazione | v2.3.0 |
| E43 | https://byu-holoocean.github.io/holoocean-docs/v2.3.0/usage/installation.html | documentazione | v2.3.0 |
| E44 | https://byu-holoocean.github.io/holoocean-docs/v2.3.0/changelog/changelog.html | changelog | 2.3.0 del 2026-02-05 |
| E45 | https://api.github.com/repos/byu-holoocean/HoloOcean | metadati API | **404** (repo con accesso legato a Epic): licenza [NV] |
| E46 | https://biguasim.readthedocs.io/en/v1.0.0/ardubridge/ardubridge.html | documentazione | BiguaSim v1.0.0 |
| E47 | https://biguasim.readthedocs.io/en/latest/ | documentazione | v1.0.0 |
| E48 | https://github.com/open-airlab/UNav-Sim | repo | — |
| E49 | https://arxiv.org/html/2506.07781v1 | survey (SMaRCSim, cita MARUS) | — |
| E50 | https://github.com/chrisdalke/unity-arduboat-sim | repo | — |
| E51 | https://github.com/Field-Robotics-Lab/dave | repo | Apache-2.0 |
| E52 | Ricerca web "HoloOcean ArduSub SITL JSON bridge", "\"HoloOcean\" ArduPilot OR ArduSub OR MAVLink github" | ricerca | 2026-10-13: nessun risultato pertinente |

---

## 11. Punti aperti (da chiudere in Fase 2)

1. Unità e segno dei comandi dello schema 0 del BlueROV2 in HoloOcean, e ordine dei propulsori rispetto ai motori 1–8 di ArduSub `vectored_6dof`.
2. Se l'IMU di HoloOcean riporta forza specifica (gravità inclusa) e in quale frame (`IMUSocket` FRD).
3. Tick rate massimo sostenibile su Windows 11 con la scena e i sensori previsti.
4. Offset della porta per istanza in SITL (`--sim-port-out` / `-I`) e comportamento di 4.1.2 con un rate fisico basso.
5. Calcolo di `state.altitude` da `position` in SIM_JSON/SIM_Aircraft @ 4.1.2.
6. Licenza effettiva del client HoloOcean e di BiguaSim.
7. Lettura del PDF ufficiale MDPI per confermare le citazioni di sezione e la Tabella 8.

---

> **Nota di riconciliazione (2026-10-13, report finale):** il punto 11.1 e la voce [NV] sulle unità dello schema 0 sono chiusi dall'analisi del sorgente engine in `holoocean-physics-audit.it.md` §2.2: 8 comandi in **newton**, clamp ±28.75 N (`BlueROV2.h:11-13`), indici 0–3 verticali, 4–7 vettoriali. L'ordine rispetto ai motori 1–8 ArduSub è un'ipotesi coerente con la matrice VECTORED_6DOF (verificata sul sorgente 4.1.2 in `hardware-inventory.it.md` §2.3), ancora da validare in SITL.
