# Specifica interfaccia ArduSub SITL ↔ simulatore fisico (backend JSON) — Fase 1 (ricognizione)

Progetto: collegamento di ArduSub SITL al simulatore HoloOcean (BlueROV2 Heavy, frame `VECTORED_6DOF`) e riuso della stessa autonomia sul BlueROV2 reale (ArduSub 4.1.2 su Blue Robotics Navigator, BlueOS 1.3.1).

Data di redazione e recupero delle evidenze: 2026-10-13. Documento solo descrittivo: nessun bridge è stato implementato e non c'è stato traffico verso <ROV-subnet>.

Documento correlato, lato simulatore: `holoocean-physics-audit.it.md` (stessa cartella), in particolare §7 ("Compatibilità con ArduSub SITL") e §10 V7 (mappatura PWM→thruster).

## Legenda

- **[CODE]**: verificato leggendo il sorgente al ref indicato.
- **[DOC]**: verificato su documentazione ufficiale.
- **[DERIV]**: deduzione dal codice, non provata a runtime.
- **[NON VERIFICATO]**: da verificare (vedi §14).

Riferimenti abbreviati `[En Lx-y]`: le *n* sono gli elementi di evidenza in §15, *x-y* sono le righe del file a quel ref.

Ref Git usati (SHA ottenuti dalla GitHub API, `git/refs`, il 2026-10-13):

| Ref | SHA | Versione (`ArduSub/version.h`) |
|---|---|---|
| `Sub-4.1` (= tag `ArduSub-4.1.2`, stesso SHA) | `2dd0bb7d4c85ac48437f139d66df648fc0e1d4ae` | ArduSub V4.1.2 |
| `Sub-4.5` | `abe1721cf52535af6eb2340e5cabed430dac76b5` | 4.5.x |
| tag `ArduSub-stable` | `fb773567896014da007bca58358704d4a0d8516f` | ArduSub V4.7.2 |
| `master` | `3297d9f115972f0dd0814cabb1a17f9231c71530` | ArduSub V4.8.0-dev |

Non esistono branch `Sub-4.6` né `Sub-4.7`: la 4.7.2 è raggiungibile solo tramite i tag `ArduSub-stable` e `ArduSub-beta` [E36].

---

## 0. Sintesi (protocollo in breve)

| Aspetto | Valore | Evidenza |
|---|---|---|
| Trasporto | UDP; un solo socket lato SITL, **non** in bind (porta sorgente effimera) | [E5 L57-94], [E7 L63-100] |
| Destinazione dei pacchetti servo | `target_ip:9002`. `target_ip` = parte dopo `:` in `--model JSON:<ip>`, oppure `--sim-address`; default `127.0.0.1` | [E4 L46-50], [E5 L63-66, L83-94], [E14 L206-218] |
| Porta in ascolto del simulatore | **UDP 9002** (`SIM_OUT_PORT`). La 9003 (`SIM_IN_PORT`) **non è usata** dal backend JSON | [E14 L213-218], [E5 L83-94] |
| Risposta | Il simulatore risponde **all'indirizzo e alla porta sorgente** del pacchetto ricevuto | [E2 L7-8], [E27 Sub-4.1 L258, L323] |
| Offset di istanza | `-I N` aggiunge `10·N` a tutte le porte (9002→9012, …) | [E14 L352-369] |
| SITL → sim | struct binaria little-endian: `uint16 magic=18458, uint16 frame_rate, uint32 frame_count, uint16 pwm[16]` (40 B). Su master/4.7, con `SERVO_32_ENABLE=1`: `magic=29569` e `pwm[32]` (72 B) | [E4 L39-44], [E6 L45-57], [E2 L12-33] |
| sim → SITL | Una riga JSON ASCII terminata da `\n` (consigliato anche un `\n` iniziale). Campi obbligatori: `timestamp` [s], `imu.gyro` [rad/s, body], `imu.accel_body` [m/s², body, forza specifica], `position` [m, NED], `velocity` [m/s, NED], più `attitude` [rad] **oppure** `quaternion` [w,x,y,z] | [E2 L41-68], [E4 L100-117] |
| Lockstep | Per ogni passo SITL invia un pacchetto servo, poi resta **bloccato** finché non arriva una risposta valida. Il tempo SITL avanza di `timestamp − timestamp_precedente`. Un passo fisico per pacchetto | [E5 L216-351], [E13 L735] |
| Timeout / resend | `recv` a 100 ms; dopo circa **1 s** senza risposta rinvia lo **stesso** pacchetto (stesso `frame_count`) con il messaggio "No JSON sensor message received, resending servos". La readme dice "10 secondi", ma il codice usa `wait_ms > 1000` | [E5 L29, L219-231], [E2 L37] |
| Frame | Mondo NED, corpo FRD; `accel_body` a riposo ≈ `[0,0,−9.81]` | [E2 L45-58], [E27 L176-187] |
| Versioni | Sub-4.1: parser `sscanf("[%f, %f, %f]")`, quindi **niente spazio dopo `:`** (usare `separators=(',',':')`), solo 16 canali, nessun `no_time_sync`. 4.5: +32 canali, +`no_time_sync`. 4.7.2/master: + parser tollerante, `latitude/longitude/altitude`, `velocity_wind`, `no_lockstep`, `rc`, `battery`; `position` non più obbligatoria | §11 |

---

## 1. Architettura: due protocolli distinti

```
 ┌──────────────────────────┐ MAVLink (TCP 5760/5762/5763, UDP 14550 via MAVProxy)  ┌──────────────┐
 │ Autonomia / GCS / pymav. │◀────────────────────────────────────────────────────▶│ ArduSub SITL │
 └──────────────────────────┘                     (interfaccia COMUNE)               │  (Linux/WSL) │
              │                                                                      └──────┬───────┘
              │ stessa API MAVLink                                  UDP JSON "fisica" │ (interno)
              ▼                                                    servo→ :9002 ; ←JSON│
 ┌──────────────────────────┐                                       ┌───────────────────▼───────────┐
 │ BlueROV2 reale (Navigator│                                       │ Bridge (Windows) + HoloOcean   │
 │ ArduSub 4.1.2, BlueOS)   │                                       │ lockstep, PWM→spinta, frame    │
 └──────────────────────────┘                                       └────────────────────────────────┘
```

- **Protocollo fisico (UDP/JSON).** Esiste solo in SITL e sostituisce l'hardware: gli ESC (uscite PWM) e i sensori, ricostruiti da SITL a partire dallo stato della fisica. Il codice di autonomia non lo usa mai [E5], [E7].
- **Protocollo di comando (MAVLink).** È identico in SITL e sul veicolo reale e costituisce l'"interfaccia comune" per l'autonomia (§9). In SITL l'endpoint è la porta seriale virtuale (TCP 5760…) o MAVProxy (UDP 14550). Sul veicolo reale l'endpoint è il router MAVLink di BlueOS [E30], [E16 L1110-1137], [E33].

---

## 2. Avvio di SITL (modi, porte, speedup, home)

### 2.1 Build

SITL si compila da sorgente su Linux, WSL2 o Docker con `waf`. `sim_vehicle.py` usa il target `bin/ardusub` per tutti i frame ArduSub [E18 Sub-4.1 L349-365; master vehicleinfo.json L693-709].

Frame noti a `sim_vehicle.py`:

| Frame | File di default |
|---|---|
| `vectored` (default) | `default_params/sub.parm` |
| `vectored_6dof` | `default_params/sub-6dof.parm` |
| `gazebo-bluerov2` | `default_params/sub.parm` |

Questo elenco è uguale in Sub-4.1 e in master [E18].

### 2.2 Tramite `sim_vehicle.py` (consigliato)

```bash
# da ardupilot/ArduSub
../Tools/autotest/sim_vehicle.py -v ArduSub -f vectored_6dof --model JSON:<IP_simulatore> \
    -l <lat>,<lon>,0,<heading> [--speedup 1] [-I 0] [--console] [--out=udp:<ip_gcs>:14550]
```

- `-f vectored_6dof` sceglie il file di default `sub-6dof.parm`, che imposta `FRAME_CONFIG 2` = VECTORED_6DOF (BlueROV2 Heavy) [E19 sub-6dof.parm Sub-4.1 L61; master L63].
- `--model` sostituisce il modello fisico, che altrimenti sarebbe il nome del frame (`ret["model"] = opts.model`). I parametri di default restano quelli del frame [E18 Sub-4.1 L403-410; master vehicleinfo.py L50-57].
- Va evitato `-f JSON:<ip>`, che è la forma indicata nella pagina docs [E1] e nella readme di Sub-4.1 (`-f json:127.0.0.1`) [E3 L3]. Per ArduSub il frame "JSON" non è in `vehicleinfo`: compare "WARNING: no config for frame" e non viene caricato nessun file di parametri di default [E18 Sub-4.1 L399-401]. **[DERIV]**
- Il modello viene scelto con un confronto `strncasecmp` sul prefisso del nome, quindi `json:` e `JSON:` sono equivalenti. Il costruttore JSON prende l'IP dopo `:` [E14 L480-483], [E5 L63-66].
- Solo master: `sim_vehicle.py --sim-address=<ip>` [E16 L1607-1610]. In Sub-4.1 l'opzione esiste solo nell'eseguibile (`--sim-address`, `--sim-port-in`, `--sim-port-out`) [E14 L106-110, L302-304, L434-441]. In entrambi i casi `set_interface_ports` sostituisce l'IP solo se l'indirizzo è diverso da `127.0.0.1` [E5 L88-90], [E7 L94-96].

### 2.3 Eseguibile diretto

```bash
build/sitl/bin/ardusub --model JSON:<IP_sim> --defaults Tools/autotest/default_params/sub-6dof.parm \
    --home <lat>,<lon>,0,<hdg> --speedup 1 -I 0 [--serial0 tcp:0] [--sim-port-out 9002]
```

Le opzioni `--speedup|-s`, `--instance|-I`, `--home|-O`, `--model|-M`, `--defaults`, `--base-port` e `--sim-*` sono presenti sia in Sub-4.1 sia in master [E14 L73-110], [E15 L87-117].

### 2.4 Porte (valori di default, istanza 0)

| Porta | Direzione / uso | Evidenza |
|---|---|---|
| UDP **9002** | SITL → simulatore (pacchetti servo). Il simulatore fa bind qui e risponde alla sorgente | [E14 L214, L218], [E2 L7-8] |
| UDP 9003 | `SIM_IN_PORT`, passata a `set_interface_ports` ma **ignorata** dal backend JSON | [E14 L213, L217], [E5 L83-94] |
| TCP 5760 | SERIAL0 (console/MAVLink, `wait`): MAVProxy si collega qui | [E30], [E16 L1133-1137], [E17 L816] |
| TCP 5762 / 5763 | SERIAL1 / SERIAL2 MAVLink: punto di accesso diretto per l'autonomia senza MAVProxy | [E30] |
| UDP 5501 | ingresso RC (`--rc-in-port`) | [E14 L207] |
| UDP 14550 (+14551 in Sub-4.1) | uscite `--out` di MAVProxy verso GCS/autonomia | [E17 L803-810], [E16 L1118-1129] |

`-I N` aggiunge 10·N a base port, RC-in, fg-view, sim in/out e irlock [E14 L352-369], [E15 L452-469]. `sim_vehicle.py` aggiunge lo stesso offset a 5760 e 14550 [E16 L1118, L1137].

### 2.5 Speedup

- `--speedup`/`-s` imposta `SIM_SPEEDUP` [E14 L340-343]. Default di `SIM_SPEEDUP`: −1 in Sub-4.1, 1 in master [E23 Sub-4.1 L71; master L229].
- Con il backend JSON lo speedup è solo un **limite superiore**. `sync_frame_time()` dorme quando la fisica va più veloce di `rate_hz·speedup` e non può accelerarla [E10 L297-336], [E5 L343-349]. Se HoloOcean è più lenta del tempo reale, il tempo simulato scorre semplicemente più lento (§8).
- Con speedup > 1, ArduSub.com raccomanda di aumentare la frequenza dell'heartbeat di MAVProxy (`set heartbeat 8`) [E29c].

### 2.6 Home / origine

- `--home lat,lon,alt,hdg` oppure `-L <nome>` (da `Tools/autotest/locations.txt`); con `sim_vehicle.py` anche `-l lat,lon,alt,hdg` [E14 L484-498], [E16 L1462-1469]. La guida ArduSub usa `-L RATBeach` [E29c].
- La `position` JSON è in metri NED **relativa alla home**, perché SITL aggiunge l'offset origine→home: `position.xy() += origin.get_distance_NE_double(home)` [E5 L286-287], [E7 L402-403].
- **Profondità.** `altitude = home.alt − position.z` [E10 L178]. Il barometro SITL di ArduSub calcola la pressione come acqua a profondità `−altitude` (in km) [E22 master L130-140; Sub-4.1 L110-121]. Va quindi usata **home con alt = 0** (superficie del mare), così che la pressione sia `101325 + 1024·g·z` Pa con z = position.z [E22 AP_Baro_HIL Sub-4.1 L37-72]. **[DERIV]**
- Con home non a quota 0 (per esempio CMAC, ~584 m), la profondità calcolata cambierebbe di segno nel modello. La calibrazione a terra all'avvio potrebbe compensare solo in parte. **[NON VERIFICATO]**

---

## 3. Formato dei pacchetti

### 3.1 SITL → simulatore (binario)

```c
struct servo_packet_16 { uint16_t magic = 18458; uint16_t frame_rate; uint32_t frame_count; uint16_t pwm[16]; }; // 40 byte
struct servo_packet_32 { uint16_t magic = 29569; uint16_t frame_rate; uint32_t frame_count; uint16_t pwm[32]; }; // 72 byte (non in Sub-4.1)
```

- Evidenze: Sub-4.1 [E4 L39-44]; master [E6 L45-57]. Il pacchetto a 32 canali viene scelto se `SRV_Channels::have_32_channels()`, cioè con `SERVO_32_ENABLE=1` [E7 L109-127], [E2 L23-33]. Presente anche in Sub-4.5 [E8 cpp L106].
- In Python: `struct.unpack('<HHI16H', data)` (oppure `'<HHI32H'`). Gli esempi ufficiali usano `'HHI16H'` (ordine nativo) e scartano pacchetti di lunghezza o magic errati [E27 Sub-4.1 L263-275; master L240-249].
- **`frame_rate`** = `rate_hz` di SITL troncato a uint16 [E5 L102]. Ad ogni risposta `rate_hz` diventa `1/Δt` della fisica [E5 L343-345]. In `update()` viene poi spostato di al massimo ±1 Hz verso `SIM_RATE_HZ` [E5 L435]. Questo vale per Sub-4.1. In master e 4.7 lo fa solo se `use_time_sync && !no_lockstep` [E7 L516-524, L612-614]. Per costruzione è quindi circa 1/Δt della fisica. La readme lo chiama "il passo che la simulazione dovrebbe fare", ma il backend "è libero di ignorarlo" [E2 L35]. **Il bridge deve usare un Δt fisso** (§7).
- **`frame_count`** cresce di 1 per ogni risposta valida ricevuta, quindi una volta per passo fisico [E5 L351], [E7 L527]. Si azzera al riavvio di SITL e un valore minore del precedente indica di resettare la fisica [E2 L37]. In caso di timeout il pacchetto viene rispedito **con lo stesso `frame_count`** [E5 L226-230].
- **`pwm[i]`** = `pwm_output[i]` del HAL SITL, in µs; `0xFFFF` diventa 0 [E13 L812-819], [E12 L304-311]. Canali senza funzione o non ancora scritti valgono 0 (master) oppure 1000 (Sub-4.1, finché `output_ready` è falso) [E13 L756-761], [E35]. **Il bridge deve trattare 0 e valori fuori da [1100,1900] come "nessun comando"**: per un thruster bidirezionale 1000 vorrebbe dire massima retromarcia. **[DERIV]**

### 3.2 Simulatore → SITL (JSON testuale)

Esempio valido per **tutte** le versioni (compatto, senza spazi dopo `:`):

```
\n{"timestamp":12.3450,"imu":{"gyro":[0.0,0.0,0.0],"accel_body":[0.0,0.0,-9.80665]},"position":[0.0,0.0,1.5],"quaternion":[1.0,0.0,0.0,0.0],"velocity":[0.0,0.0,0.0]}\n
```

| Campo | Tipo / unità / frame | Obbligatorio | Note | Evidenza |
|---|---|---|---|---|
| `timestamp` | double, s, tempo **assoluto** della fisica | sì | non il passo | [E2 L46, L68], [E4 L101] |
| `imu.gyro` | 3×float, rad/s, **body FRD** (p,q,r) | sì | | [E2 L47-48], [E4 L102] |
| `imu.accel_body` | 3×float, m/s², **body FRD**, forza specifica (a − g) | sì | a riposo `[0,0,−9.81]` (§6) | [E2 L49], [E27 L176-187] |
| `position` | 3×double, m, **NED** relativa alla home | sì in 4.1/4.5; opzionale in 4.7/master se ci sono `latitude`+`longitude`+`altitude` | | [E4 L104], [E6 L130-135], [E7 L387-404] |
| `velocity` | 3×float, m/s, **NED** | sì | | [E2 L51], [E4 L107] |
| `attitude` | 3×float, rad, roll/pitch/yaw (Euler 321, body→NED) | uno dei due | `dcm.from_euler(r,p,y)` | [E5 L290-295] |
| `quaternion` | 4×float `[q1,q2,q3,q4]=[w,x,y,z]`, body→NED | uno dei due | prevale su `attitude` se ci sono entrambi | [E2 L54-59], [E5 L290-292], [E27 L121-123] |
| `rng_1..rng_6` | float, m | no | richiede il driver rangefinder SITL | [E2 L72-81] |
| `windvane.direction/speed` | rad, m/s | no | | [E2 L83-95] |
| `airspeed` | m/s | no | | [E4 L116] |
| `no_time_sync` | bool | no | da 4.5 in poi; disattiva la sincronizzazione al tempo reale e porta `AHRS_EKF_TYPE` di default a 10 | [E8 h L134, cpp L310], [E7 L406-416] |
| `velocity_wind` | 3×float, m/s, NED | no | solo 4.7/master | [E6 L145], [E2 L97-103] |
| `latitude/longitude/altitude` | gradi, gradi, m AMSL | no | solo 4.7/master; la prima ricezione imposta la home | [E6 L130-132], [E7 L387-404] |
| `no_lockstep` | bool | no | solo 4.7/master (commit 759e03f1aa, 2025-11) | [E6 L150], [E7 L309-319] |
| `rc.rc_1..rc_12` | float, µs | no | solo 4.7/master | [E6 L151-162], [E7 L480-489] |
| `battery.voltage/current` | V, A | no | solo 4.7/master; altrimenti `SIM_BATT_VOLTAGE` | [E6 L163-164], [E7 L492-503] |

**Regole di parsing (critiche per la compatibilità):**

1. **Framing.** In `recv_fdm` ogni `\n` diventa `\0` [E5 L234-236]. Viene analizzata solo la stringa compresa tra il **penultimo e l'ultimo** `\0` del buffer [E5 L239-249]. Senza un terminatore precedente, il primo messaggio resta nel buffer e viene letto solo al giro successivo. Per questo l'esempio di Sub-4.1 invia `"\n" + json + "\n"` [E27 Sub-4.1 L320] e la readme dice "preceded by and terminated with" `\n` [E2 L61]. Se nel buffer arrivano più righe, conta **solo l'ultima** [DERIV].
2. **Chiavi.** Ogni chiave è cercata con `strstr` sull'intera stringa: prima la sezione (`imu`), poi la chiave dopo la sezione. Dopo la chiave il parser salta `strlen(key)+2` caratteri, cioè `key":` [E5 L136-160].
   - Nessun campo extra deve contenere come sottostringa le chiavi note (per esempio un campo `"imu_temp"` messo prima di `"imu"`). **[DERIV]**
3. **Spazi (solo Sub-4.1 e Sub-4.5).** I vettori si leggono con `sscanf(p, "[%f, %f, %f]")` [E5 L179, L189, L199], [E8 cpp L196-216]. `p` punta subito dopo `":`, quindi uno spazio prima di `[` fa **fallire** il parse: `json.dumps` di Python con i separatori di default produce `": ["`. Va quindi usato `json.dumps(obj, separators=(',', ':'))`, come fanno entrambi gli esempi ufficiali [E27 Sub-4.1 L320; master L289]. Gli spazi *dentro* l'array sono tollerati, perché nel formato di `sscanf` gli spazi corrispondono a zero o più caratteri di spaziatura.
4. **Parser tollerante (4.7.2/master).** `parse_array` salta fino a `-` o a una cifra [E7 L148-175]. Un numero come `.5`, o `+1`, verrebbe letto **male** (`.5` → 5).
   - Non inviare mai `NaN` o `Infinity` (JSON non valido, comportamento indefinito).
   - Usare float in formato standard (`0.5`, `1e-05`). **[DERIV]**
5. **Campi obbligatori mancanti.** Il parse restituisce 0 e compare "Did not contain all mandatory fields"; senza `attitude` né `quaternion` compare "Did not receive attitude or quaternion". In entrambi i casi **il tempo non avanza** in quel passo [E5 L249-260].
   - Con un vettore malformato il parse si interrompe restituendo una bitmask parziale. I campi successivi restano ai valori precedenti, senza errore esplicito [E5 L179-182].
6. **Messaggio di conferma.** Al primo messaggio, e a ogni cambio dell'insieme di campi, SITL stampa "JSON received:" con l'elenco dei campi letti. È il messaggio da usare per il debug [E5 L262-277], [E2 L129-144].

---

## 4. Cosa invia ArduSub (PWM, canali, mappatura motori)

### 4.1 Conversione spinta → PWM

- **Sub-4.1:** `calc_thrust_to_pwm(t) = constrain(1500 + t·400, MOT_PWM_MIN, MOT_PWM_MAX)` con t ∈ [−1,1], quindi 1100–1900 µs e neutro 1500 [E20 Sub-4.1 L233-236].
- **master:** la scala è asimmetrica, `1500 + t·(max−1500)` oppure `1500 + t·(1500−min)` [E20 master L230-235].
- Da disarmato, o con spool `SHUT_DOWN`, ogni motore riceve **1500** [E20 Sub-4.1 L213-228, L238-257]. Il verso di ciascun motore si inverte con `MOT_n_DIRECTION` (±1) [E20 Sub-4.1 L29-41].

### 4.2 Fattori di mixing (Sub-4.1 e master identici)

Valori da `AP_Motors6DOF::setup_motors`. Ordine colonne: roll, pitch, yaw, throttle, forward, lateral.

**VECTORED_6DOF** (`FRAME_CONFIG=2`, BlueROV2 Heavy) [E20 Sub-4.1 L156-165]:

| Motore (canale) | roll | pitch | yaw | throttle | forward | lateral |
|---|---|---|---|---|---|---|
| 1 | 0 | 0 | +1 | 0 | −1 | +1 |
| 2 | 0 | 0 | −1 | 0 | −1 | −1 |
| 3 | 0 | 0 | −1 | 0 | +1 | +1 |
| 4 | 0 | 0 | +1 | 0 | +1 | −1 |
| 5 | +1 | −1 | 0 | −1 | 0 | 0 |
| 6 | −1 | −1 | 0 | −1 | 0 | 0 |
| 7 | +1 | +1 | 0 | −1 | 0 | 0 |
| 8 | −1 | +1 | 0 | −1 | 0 | 0 |

**VECTORED** (`FRAME_CONFIG=1`, BlueROV2 standard, 6 motori) [E20 Sub-4.1 L168-175]: motori 1-4 come sopra; 5 = (+1,0,0,−1,0,0); 6 = (−1,0,0,−1,0,0).

Il motore *n* esce sul canale servo *n*: funzione `Motor n`, cioè `SERVOn_FUNCTION` 33…40. **[NON VERIFICATO in SITL: leggere `SERVO1..8_FUNCTION` dal parametro dump all'avvio.]**

Il modello interno `SIM_Submarine` legge `input.servos[0..7]` con lo stesso ordine e gli stessi fattori [E21 master L26-48], quindi conferma la convenzione canale i → motore i+1.

### 4.3 Altri canali

ArduSub usa ingressi RC in passthrough per le luci e la funzione Mount per il tilt della camera [E29b]:

- Ingressi RC: 1 pitch, 2 roll, 3 throttle, 4 yaw, 5 forward, 6 lateral, 7 camera pan, 8 camera tilt (velocità di sweep), 9 luci 1, 10 luci 2, 11 video switch.
- **Uscite in SITL (board generica, Sub-4.1):** `SERVO9_FUNCTION=59` (RCIN9, luci) e `SERVO10_FUNCTION=7` (mount tilt) [E24 Parameters Sub-4.1 L827-829].
  - `sub.parm` sovrascrive `SERVO8_FUNCTION 7` (tilt su canale 8, 1100-1900). Questo è coerente solo con il frame `vectored` a 6 motori [E19 sub.parm Sub-4.1 L81-83].
  - `sub-6dof.parm` **non** imposta `SERVO8_FUNCTION`.
- **Uscite su Navigator (Sub-4.1):** default `SERVO13_FUNCTION=59`, `SERVO14_FUNCTION=60` (luci), `SERVO16_FUNCTION=7` + `SERVO16_REVERSED=1` (tilt) e `SCHED_LOOP_RATE=200` [E24 Parameters Sub-4.1 L817-826].
  - Quindi **luci e tilt stanno su canali diversi in SITL e sul reale**: con i soli 16 canali del pacchetto 4.1 sono comunque tutti visibili (indici 12, 13, 15).
- Il gripper in 4.1 si configura con `SERVOn_FUNCTION` e pulsanti `BTNn_FUNCTION`. Non è nei default SITL esaminati. **[NON VERIFICATO]**

---

## 5. Cosa deve restituire il simulatore e come lo usa ArduSub

1. A ogni passo `Aircraft` copia i campi JSON nei suoi membri: `accel_body`, `gyro`, `velocity_ef`, `position` e `dcm` (da quaternione o Euler) [E5 L283-295]. Poi esegue:
   - `update_position()`, che converte i metri NED in lat/lon/alt [E5 L314], [E10 L173-178];
   - `update_mag_field_bf()`, che calcola il campo magnetico dalla posizione e dall'assetto, perché la fisica non lo fornisce [E5 L432].
2. `fill_fdm()` riempie la struttura `sitl_fdm`: `timestamp_us = time_now_us`, accelerazioni, rate in gradi/s, velocità NED, lat/lon/alt e così via [E10 L385-459]. Da questa struttura i **driver SITL sintetizzano i sensori**:
   - IMU: `AP_InertialSensor_SITL` aggiunge rumore, bias e vibrazione secondo `SIM_ACCn_RND`, `SIM_GYRn_RND` e simili [E23 master L1039-1060];
   - barometro: `AP_Baro_SITL` aggiunge rumore `SIM_BARO_RND`, drift e glitch [E22 master L62-87]. In ArduSub è tipo `BARO_TYPE_WATER` e usa il modello di pressione dell'acqua [E22 master L20-21, L130-140];
   - bussola: dal campo calcolato al punto 1, con rumore `SIM_MAG_RND`;
   - GPS: dalla posizione [E13 L172-186].
3. Conseguenza: **il bridge deve inviare la verità di terreno (ground truth) senza rumore**. Il rumore va configurato con i parametri `SIM_*`. Gli esempi ufficiali inviano grandezze ideali [E27].
4. **Profondità.** ArduSub ricava la profondità **solo dal barometro SITL**, a sua volta calcolato da `position.z`. Non serve un sensore di profondità in HoloOcean [E22]. **[DERIV]**
   - La densità nel modello è 1024 kg/m³ (`seaDensity = 1.024`) [E22 AP_Baro_HIL Sub-4.1 L45-63].
   - `BARO_SPEC_GRAV` ha default **1.0** (acqua dolce) [E22 AP_Baro.cpp Sub-4.1 L130-134]. Lasciato così, la profondità stimata in SITL è circa 2,4 % più grande della `position.z` vera. Va impostato `BARO_SPEC_GRAV=1.024` in SITL, oppure si accetta la discrepanza. Sul reale va usato il valore dell'acqua del test. **[DERIV]**
5. **GPS (parità con il reale).** Il GPS SITL fornisce una posizione **anche in immersione** [E13 L183-186]. Il BlueROV2 reale senza DVL o GPS di superficie non ha una stima di posizione, e `GUIDED` richiede `position_ok()` [E26 control_guided.cpp Sub-4.1 L36-39].
   - Per la parità si disattiva il GPS SITL: `SIM_GPS_DISABLE=1` in Sub-4.1 [E23 Sub-4.1 L316]; in master il parametro è sotto il gruppo `SIM_GPS1_` [E23 master L730-732], con il nome esatto **[NON VERIFICATO]**.
   - In alternativa si simula un DVL (fuori dallo scope di questa fase).
6. **Rangefinder.** Con `rng_1` e il driver `RNGFNDx_TYPE` di tipo SITL [E2 L70-81]. `sub.parm` imposta `RNGFND1_TYPE 1` (analogico) [E19].

---

## 6. Convenzioni di riferimento, gravità, quantità inerziali

- **Mondo:** NED (x Nord, y Est, z Giù), con origine nella home (§2.6) [E2 L50-51].
- **Corpo:** FRD (x avanti, y destra, z giù). `gyro` e `accel_body` sono nel corpo [E2 L47-49].
- **Assetto:** la DCM rappresenta la rotazione corpo→terra (`velocity_air_bf = dcm.transposed() * velocity_ef`) [E5 L304]. Euler in rad, sequenza 321 (`from_euler(roll,pitch,yaw)`) [E5 L294]. Quaternione `[w,x,y,z]` (q1 = w) [E5 L199], [E27 L121-123].
- **`accel_body` = forza specifica (ciò che misura un accelerometro), cioè `R_bn·(a_NED − g_NED)` con `g_NED = (0,0,+9.80665)`.** L'esempio ufficiale calcola `accel = Δv/Δt; accel.z -= GRAVITY_MSS; accel = dcm.transposed()*accel` [E27 Sub-4.1 L176-187; master L193-199]. Il modello interno usa la stessa forma: `accel_body = dcm.T*(accel_earth + (0,0,−g))` [E10 L761].
  - A riposo e livellato: `[0, 0, −9.80665]`.
  - Per un ROV a galleggiamento neutro fermo vale **lo stesso valore**, perché la spinta idrostatica è una forza di contatto che l'accelerometro misura. **[DERIV]**
  - Non va sottratta né aggiunta la spinta di galleggiamento.
- **Velocità e posizione:** NED, m/s e m. `velocity` deve essere coerente con la derivata di `position`. L'EKF fonde GPS/baro (dalla posizione) e IMU (dall'accelerazione): incoerenze producono innovazioni elevate. **[DERIV]**
- **Accelerazione numerica.** Se HoloOcean non fornisce un'accelerazione affidabile, si usa `Δv/Δt` della velocità NED del passo (metodo degli esempi ufficiali [E27]).
  - Il rumore numerico di questa derivata entra nell'IMU.
  - L'audit HoloOcean (§3.3) documenta i limiti dell'IMU simulata.
- **Conversione da HoloOcean.** HoloOcean usa un sistema destrorso con x avanti, y sinistra, z su (FLU/NWU), metri, rotazioni in gradi [E34]. Quindi:
  - posizione e velocità NED = `(x, −y, −z)`, se l'asse x del mondo HoloOcean viene preso come Nord;
  - vettori nel corpo FRD = `(x, −y, −z)`;
  - quaternione: stesso schema di `quaternion_to_AP` dell'esempio pybullet, `[w, x, −y, −z]` [E27 L121-123].
  - La conversione va verificata con test statici e dinamici (audit HoloOcean §10 V3). **[NON VERIFICATO]**

---

## 7. Frequenze e temporizzazione (lockstep)

### 7.1 Ciclo

Per ogni chiamata `_fdm_input_step()` → `_fdm_input_local()` SITL esegue [E12 L99-129, L236-290], [E13 L612-743]:

1. `_simulator_servos(input)`;
2. `sitl_model->update_model(input)` → `JSON::update()`: `output_servos()` (invio), poi `recv_fdm()`, che **blocca** [E5 L422-428];
3. `fill_fdm()`;
4. `hal.scheduler->stop_clock(state.timestamp_us)`: il clock di ArduSub viene **forzato** al tempo derivato dalla fisica [E13 L735], [E12 L285].

`wait_clock()` richiama `_fdm_input_step()` finché il clock simulato non raggiunge la scadenza del loop dello scheduler [E12 L133-145], [E13 L203-212]. Quindi ArduSub non avanza mai se la fisica non risponde (**lockstep**).

### 7.2 Avanzamento del tempo [E5 L332-351], [E7 L505-527]

```
deltat = timestamp − last_timestamp      (se timestamp < last: "Detected physics reset", deltat = 0)
time_now_us += deltat·1e6
se 0 < deltat < 0.1 s: adjust_frame_time(1/deltat); time_advance()   (→ sync col wall-clock se use_time_sync)
```

- **Timestamp fermo** (`deltat = 0`): il tempo SITL **non avanza** e non c'è sincronizzazione. ArduSub resta allo stesso istante e rifà subito lo scambio. Se succede in modo sistematico, ArduSub non progredisce. **[DERIV]**
- **Timestamp all'indietro:** SITL stampa "Detected physics reset", non resetta ArduSub e azzera solo la bitmask [E5 L333-337].
- **Salto ≥ 0,1 s:** il tempo avanza ma senza `time_advance`. Il primo pacchetto ha `last_timestamp = 0`, quindi conviene far partire il `timestamp` da circa 0 (per esempio `tick·dt`). **[DERIV]**
- `time_advance()` aggiunge `frame_time_us` solo se il backend non ha già fatto avanzare il tempo [E10 L261-271].

### 7.3 Frequenza della fisica

- `SIM_RATE_HZ` ha default **1200** in SITL [E23 Sub-4.1 L211; master L46-52, L554].
- `SCHED_LOOP_RATE` di ArduSub:
  - default **400** in SITL (`SCHEDULER_DEFAULT_LOOP_RATE` 400 per Copter e Sub) [E24 AP_Scheduler Sub-4.1 L35-39, L60];
  - **200** sul Navigator 4.1 [E24 Parameters Sub-4.1 L825].
- La readme chiede di tenere la frequenza della fisica **sopra** il loop rate del veicolo [E2 L35].
- Raccomandazione:
  - Δt della fisica ≤ 1/400 s (≥ 400 Hz), oppure impostare `SCHED_LOOP_RATE=200` in SITL per la parità con il Navigator e usare una fisica a ≥ 200 Hz.
  - Con Δt più grande del periodo di loop, un solo passo fisico copre più loop dello scheduler, che girano senza nuovi dati sensore: EKF e controllori degradano. **[DERIV]**
  - Il valore minimo accettabile va misurato. **[NON VERIFICATO]**

### 7.4 Perdita di pacchetti e duplicati [E5 L219-231]

- Il `recv` ha timeout di 100 ms in un ciclo. Dopo più di 1000 ms cumulativi SITL **rispedisce lo stesso pacchetto servo** (stesso `frame_count`) e ricomincia l'attesa.
  - Il commento nel codice e la readme dicono "10 secondi" [E2 L37], [E3 L21]. **Il codice dice ~1 s.**
- L'esempio di Sub-4.1, quando riceve un `frame_count` duplicato, fa `continue` **senza rispondere** [E27 Sub-4.1 L292-295]. Se a perdersi è stata la *risposta*, questo comportamento porterebbe a uno **stallo**: SITL rimanda lo stesso frame per sempre. **[DERIV]**
- **Specifica per il bridge:** con `frame_count == last`, **rispondere di nuovo con l'ultimo stato senza fare un passo fisico**.
- Con `frame_count < last`: resettare la fisica (riavvio di SITL) [E2 L37], [E27 L288-291].
- Con `frame_count > last+1`: registrare il buco. Non dovrebbe accadere in lockstep, perché SITL incrementa solo dopo una risposta valida. **[DERIV]**

### 7.5 Opzioni di sincronizzazione

- **`no_time_sync`** (4.5 e successive): `use_time_sync = false`. Niente `sync_frame_time` (quindi niente sleep) e `AHRS_EKF_TYPE` passa di default a 10, il che **non è adatto** a un test di autonomia realistico [E7 L406-416], [E8 cpp L310].
- **`no_lockstep`** (4.7.2 e master): se non ci sono dati, SITL non blocca e avanza il tempo di `1/SIM_RATE_HZ` [E7 L309-319].
  - Da **non** usare: rompe il determinismo e la relazione tra comando e risposta. **[DERIV]**
- Speedup: vedi §2.5. In Cygwin `min_sleep_time` vale 20 ms, contro 5 ms negli altri casi [E10 h L396-400].

---

## 8. Tempo simulato e tempo fisico

- Il tempo di ArduSub (`AP_HAL::micros64`) **è** il `timestamp` della fisica accumulato [E13 L735]. Tutti i timeout interni sono in tempo simulato: failsafe GCS, `FS_PILOT_TIMEOUT`, validità di 5 s di `SET_ATTITUDE_TARGET` in ALT_HOLD [E26 control_althold.cpp L48-49].
- **Se HoloOcean è più lenta del tempo reale**, il tempo simulato scorre più lentamente del muro. I processi esterni che misurano tempo reale (pymavlink, timeout dell'autonomia, QGC) vedono un veicolo "lento": per esempio gli heartbeat arrivano a 1 Hz *simulato*.
  - L'autonomia deve basarsi su `TIMESTAMP`/`time_boot_ms` dei messaggi MAVLink, non sull'orologio di sistema, se vuole un comportamento identico in SITL e sul reale. **[DERIV]**
- **Se la fisica è più veloce**, `sync_frame_time` la frena a `rate_hz·speedup` [E10 L297-336].
- Il `timestamp` JSON non deve corrispondere all'orologio di Windows: va generato come `n_tick·dt` (deterministico).

---

## 9. Interfaccia MAVLink (comune SITL / reale)

### 9.1 Endpoint

| Ambiente | Endpoint |
|---|---|
| SITL senza MAVProxy | `tcp:127.0.0.1:5762` (SERIAL1) [E30] |
| SITL con MAVProxy | `udpin:0.0.0.0:14550` (uscita `--out`) [E16], [E17], [E29a] |
| Veicolo reale | endpoint del router MAVLink di BlueOS. Gli endpoint "client" puntano tipicamente alla stazione di terra (esempio della documentazione: <TOPSIDE-IP>); gli endpoint "server" usano `0.0.0.0` [E33]. Valori di default precisi di BlueOS 1.3.1 **[NON VERIFICATO]**; da leggere nella pagina *MAVLink Endpoints* del veicolo, senza contattarlo in questa fase |

La stringa di connessione va resa **parametro di configurazione** dell'autonomia: è l'unica differenza obbligata.

### 9.2 Modi ArduSub (`custom_mode`) [E25]

| Modo | Numero | Note |
|---|---|---|
| STABILIZE | 0 | |
| ACRO | 1 | |
| ALT_HOLD (depth hold) | 2 | |
| AUTO | 3 | |
| GUIDED | 4 | |
| CIRCLE | 7 | |
| SURFACE | 9 | |
| POSHOLD | 16 | |
| MANUAL | 19 | |
| MOTOR_DETECT | 20 | |
| SURFTRAK | 21 | **solo** 4.5 e successive / master, non in 4.1 [E25 master L44] |

Fonti: Sub-4.1 `defines.h` L33-44; master `mode.h` L33-46.

### 9.3 Messaggi

- **HEARTBEAT ≥ 1 Hz** da ogni componente [E29a]. Failsafe GCS: `FS_GCS_ENABLE` di default = 2 (disarmo) [E24 Parameters Sub-4.1 L95; master L68].
- **Arm/disarm:** `COMMAND_LONG` con `MAV_CMD_COMPONENT_ARM_DISARM` (400), param1 = 1 o 0 [E29a].
- **Cambio modo:** `SET_MODE` con `base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED`, `custom_mode = n`, oppure `MAV_CMD_DO_SET_MODE`. pymavlink offre `mode_mapping()` [E29a].
- **MANUAL_CONTROL (69).** Range: x, y, r ∈ [−1000,1000]; z ∈ [0,1000] con neutro 500 [E29a]. Viene accettato **solo** se `msg.sysid == SYSID_MYGCS` (default **255**) e `target == SYSID_THISMAV` [E26 Sub-4.1 L540-548], [E24 Parameters Sub-4.1 L57].
  - Conta come input del pilota per il failsafe: `FS_PILOT_INPUT` di default = disarmo dopo `FS_PILOT_TIMEOUT` = 3 s [E24 Parameters Sub-4.1 L144-152], [E26 L569].
- **RC_CHANNELS_OVERRIDE (70).** Sostituisce gli ingressi RC, non le uscite; la mappatura canali è quella di §4.3 [E29a], [E29b]. In master è gestito esplicitamente in `GCS_MAVLink_Sub.cpp` L574-581 [E26].
- **SET_ATTITUDE_TARGET (82).**
  - Con il bit 6 di `type_mask` (throttle ignore), il messaggio viene memorizzato e usato da **ALT_HOLD** per 5 s [E26 Sub-4.1 L577-588; control_althold.cpp L48-59].
  - Senza i bit 6 e 7, `guided_set_angle(q, climb_rate)`, con thrust 0.5 = velocità verticale nulla [E26 L590-605].
- **SET_POSITION_TARGET_LOCAL_NED (84):** solo in GUIDED, oppure in AUTO con NavGuided [E26 Sub-4.1 L609-615].
- **SET_POSITION_TARGET_GLOBAL_INT (86):** in GUIDED/AUTO-guided, **e in ALT_HOLD solo per la profondità** (`alt` negativa = profondità) [E26 Sub-4.1 L682-705], [E29a].
- **GUIDED richiede `position_ok()`** [E26 control_guided.cpp L36-39]. Vedi la differenza SITL/reale di §5.5.

---

## 10. Componenti riutilizzabili e componenti da sviluppare

| Riutilizzabile | Uso |
|---|---|
| pymavlink (`mavutil.mavlink_connection`, `mode_mapping`, `arducopter_arm`) | client MAVLink comune [E29a] |
| Esempi JSON (`libraries/SITL/examples/JSON/pybullet/robot.py`) | schema di server UDP, unpack, framing, conversioni di frame [E27] |
| `SIM_Submarine.cpp` | **solo come riferimento**, è un modello semplificato (vedi sotto) [E21] |
| `default_params/sub-6dof.parm` | base di parametri SITL per VECTORED_6DOF [E19] |

Caratteristiche di `SIM_Submarine.cpp` [E21]:

- spinta quadratica `out·|out|·51.48 N`, con `out = (pwm−1500)/400` e deadzone 1475–1525 [E21 master L94-98; .h L60-87];
- coppie calcolate come `fattore_mixing · spinta · 0.25 m / I`, cioè **non** dalla geometria reale;
- drag quadratico, galleggiamento `SIM_BUOYANCY`, densità 1023.6 kg/m³.

Da implementare per HoloOcean:

1. **Server UDP** su `0.0.0.0:9002` (o sull'IP raggiungibile da WSL, §12), con risposta a `addr` di `recvfrom`, gestione di 16 e 32 canali, controllo di magic e lunghezza.
2. **Stepping in lockstep di HoloOcean:**
   - esattamente un `env.tick()` per ogni `frame_count` nuovo;
   - nessun passo sui duplicati (si rimanda l'ultimo stato);
   - reset quando `frame_count` diminuisce;
   - Δt fisso e `timestamp = n·dt`.
3. **PWM → spinta:**
   - curva T200 al voltaggio scelto, con deadband e saturazione;
   - inversione con `MOT_n_DIRECTION`;
   - ordine canale → thruster HoloOcean e relativi segni (audit HoloOcean §7.2 e V7);
   - clamp della spinta HoloOcean (audit §8, n. 2).
4. **Trasformazioni di frame** NWU/FLU → NED/FRD, quaternione `[w,x,y,z]`, gradi → radianti (§6).
5. **Pacchettizzazione dei sensori:** ground truth di posizione, velocità, assetto e rate; `accel_body` come forza specifica; JSON compatto con `\n` iniziale e finale.
6. **Logging e diagnostica:** frame mancanti, latenza per passo, Hz effettivi.

---

## 11. Variazioni tra versioni

Storico dei commit su `libraries/SITL/SIM_JSON.{cpp,h}` (GitHub API; date autore, dove indicate anche commit) [E28]:

| Commit | Data | Modifica |
|---|---|---|
| `0c6efa5f26` | 2020-05-20 | aggiunta del backend JSON |
| `44f727cfd2` | 2020-05-29 | rimossa la porta extra |
| `6a2966c31b` | 2020-06-30 | quaternione oppure Euler |
| `bdec15f708` | 2020-07-17 | rangefinder |
| `8aeee4bc83` | 2020-08-15 | windvane |
| `a301808e7a` | 2021-02-02 | airspeed |
| `fb275c9874` | 2021-06-21 | posizioni in double |
| `5eaf76cdc8` | 2021-07-10 | separazione origine/home |
| `4505dce64d` | autore 2021-04-25, commit 2021-08-02 | `no_time_sync` |
| `c3b576a72f` | 2023-03-16 | 32 servo |
| `adffd93894` | 2023-12-26 | `SocketAPM_native` |
| `b7ae62c57e` | 2025-11-27 | bitmask a 64 bit |
| `759e03f1aa` | 2025-11-27 | `no_lockstep` |
| `1f7b57a770` | 2025-11-27 | input globali lat/lon/alt |
| `8e4a98986b` | 2026-01-06 | `strtof` |
| `e5e9709416` | 2026-06-05 | fix della bitmask RNG |

Contenuto verificato per versione:

| Caratteristica | Sub-4.1 (4.1.2) | Sub-4.5 | ArduSub-stable 4.7.2 / master |
|---|---|---|---|
| Backend JSON presente | **sì** [E4], [E5] | sì [E8] | sì [E9], [E7] |
| Pacchetto 32 canali (magic 29569) | no, solo 16 [E4 L39-44] | sì [E8 cpp L106] | sì [E6 L52-57] |
| Parser dei vettori | `sscanf`, sensibile agli spazi [E5 L179] | `sscanf` [E8 cpp L196] | `parse_array`, tollerante [E7 L148-175], [E9 L149] |
| `position` obbligatoria | sì | sì [E8 h L121] | no se c'è lat/lon/alt [E6 L135] |
| `no_time_sync` | **no** (keytable a 16 voci) [E4 L100-117] | sì [E8 h L134] | sì |
| `velocity_wind`, `no_lockstep`, `rc`, `battery`, lat/lon/alt | no | no [E8 h L118-134] | sì [E9] |
| Messaggio di resend | ~1 s | ~1 s [E8 cpp L248] | ~1 s [E7 L326] |
| Init dei PWM prima dell'output | 1000 su tutti i canali [E13 L756-761] | — | 0 (array membro non inizializzato a 1000) [E12 L295-311] **[DERIV]** |
| `calc_thrust_to_pwm` | simmetrica ±400 [E20 L233-236] | — | asimmetrica min/max [E20 master L230-235] |
| Modello di galleggiamento `SIM_Submarine` | formula precedente [E21 diff] | — | proporzione immersa corretta [E21 master L233-243] |
| `sim_vehicle.py` su WSL2 | nessuna logica WSL2: `--out 127.0.0.1` [E17 L803-810] | — | individua l'IP dell'host Windows e manda `--out` lì; `--no-wsl2-network` [E16 L196-215, L1114-1129, L1556] |
| `--sim-address` in `sim_vehicle.py` | no (solo nell'eseguibile) | — | sì [E16 L1607] |
| Modo SURFTRAK (21) | no | sì (in master) | sì [E25] |
| Nome parametro GPS SITL | `SIM_GPS_DISABLE` [E23 L316] | — | gruppo `SIM_GPS1_*` [E23 master L730-732] |

### Versione di SITL da usare rispetto al veicolo reale 4.1.2

**Opzione A: SITL da `Sub-4.1` / tag `ArduSub-4.1.2`.**

Vantaggi:

- stesso firmware del veicolo: controllori, failsafe, gestione dei messaggi MAVLink, numerazione dei modi e default dei parametri identici, quindi massima trasferibilità dell'autonomia;
- il backend JSON esiste e basta per i campi obbligatori.

Svantaggi:

- parser rigido (serve il JSON compatto, §3.2);
- niente `no_time_sync`, `rc`, `battery`, 32 canali;
- `sim_vehicle.py` senza supporto WSL2 (uscita MAVProxy da impostare a mano);
- build di un branch del 2021-2022 su toolchain recenti con possibili problemi (Python, waf, gcc) **[NON VERIFICATO]**.

**Opzione B: SITL da 4.7.2 (`ArduSub-stable`) o master.**

Vantaggi: parser tollerante, supporto WSL2, campi aggiuntivi.

Svantaggi:

- comportamento del firmware **diverso** dal reale: nomi dei parametri PSC cambiati (`PSC_POSZ_P` → `PSC_D_POS_P`, ecc. [E19 sub.parm master L69-77]), modi aggiuntivi, PWM asimmetrico;
- i risultati di tuning non si trasferiscono 1:1.

**Raccomandazione [DERIV]:** usare come riferimento principale l'**opzione A (4.1.2)**, scrivendo il bridge in modo compatibile con *tutte* le versioni:

- JSON compatto con `\n` iniziale e finale;
- solo i campi comuni;
- gestione di 16 e 32 canali.

Una build master o 4.7.2 resta utile per diagnosi.

---

## 12. Vincoli di piattaforma (Windows host, HoloOcean su Windows)

- **SITL su Windows:**
  - **WSL2** (consigliato dalla documentazione ArduPilot) [E31];
  - Cygwin: supportato storicamente, il codice ha casi specifici (`min_sleep_time` 20 ms, `CYGWIN_BUILD`) [E10 h L396-400], [E12 L145-160]; non consigliato per il lockstep ad alta frequenza **[DERIV]**;
  - Docker: possibile, non verificato **[NON VERIFICATO]**.
- **Rete con WSL2 in NAT (default)** [E32a]:
  - Da Windows verso WSL: `localhost` funziona grazie a `localhostForwarding` per le porte in ascolto in WSL [E32b]. Esempio: QGC o autonomia su Windows → TCP 5760/5762 di SITL. L'inoltro UDP verso WSL non è documentato esplicitamente **[NON VERIFICATO]**.
  - Da WSL verso Windows serve l'**IP dell'host** (`ip route show | grep -i default | awk '{print $3}'`) [E32a]. Quindi SITL va lanciato con `--model JSON:<IP_host_Windows>`.
  - Il bridge su Windows fa bind su `0.0.0.0:9002`. Va consentito l'inbound UDP 9002 nel Windows Firewall per l'interfaccia vEthernet (WSL); può valere anche l'Hyper-V firewall (attivo di default da WSL 2.0.9 su Windows 11 22H2+) [E32a]. **[NON VERIFICATO in loco]**
  - La risposta va all'IP e alla porta effimera di WSL ricevuti con `recvfrom`.
  - L'IP dell'host NAT **cambia a ogni avvio di WSL**: deve essere letto all'avvio.
- **Rete con WSL2 mirrored** (`[wsl2] networkingMode=mirrored` in `<user-home>\.wslconfig`, Windows 11 22H2+):
  - Windows e WSL comunicano su `127.0.0.1` in entrambe le direzioni. Si usa quindi `--model JSON:127.0.0.1` e il bridge su `127.0.0.1:9002` [E32a], [E32b].
  - Con `sim_vehicle.py` di master va usato `--no-wsl2-network` [E31], [E16 L1556-1559].
  - Per l'inbound può servire `Set-NetFirewallHyperVVMSetting … -DefaultInboundAction Allow` [E32a].
  - **Raccomandato** per semplicità e stabilità dell'IP. **[DERIV]**
  - Questa macchina è Windows 11 build 26200, quindi il requisito 22H2+ è soddisfatto.
- **MAVLink verso un GCS su Windows:**
  - `sim_vehicle.py` di master, in modalità NAT, manda automaticamente `--out <IP_host>:14550` [E16 L1124-1127].
  - Con Sub-4.1 serve `--out=udp:<IP_host>:14550` manuale [E17 L799-810], oppure un collegamento TCP via localhost a 5760/5762.
- **Latenza.** Ogni passo richiede un round-trip UDP WSL↔Windows più un `tick` di HoloOcean. A 400 Hz il budget è 2,5 ms per passo in tempo reale; oltre questo il tempo simulato rallenta (§8), senza errori. **[DERIV]**

---

## 13. Checklist di conformità per il bridge (sintesi normativa)

1. Fare bind UDP su 9002 (+10·N) e rispondere a `recvfrom.addr`.
2. Accettare pacchetti da 40 B (magic 18458) e 72 B (magic 29569); scartare gli altri.
3. A ogni nuovo `frame_count`: applicare i PWM, fare **un** passo fisico di Δt fisso e rispondere.
   - Duplicato: rispondere con l'ultimo stato, senza fare il passo.
   - Valore che decresce: resettare.
4. PWM 0 o fuori da [1100,1900] = spinta 0. Tra 1100 e 1900: curva T200 con deadband.
5. JSON di risposta compatto, `"\n{...}\n"`, campi obbligatori con il quaternione `[w,x,y,z]`, valori finiti.
6. `timestamp = n·Δt` (partenza da circa 0), strettamente crescente.
7. Frame NED/FRD; `accel_body` = forza specifica; ground truth senza rumore.
8. Home SITL con alt = 0; `BARO_SPEC_GRAV` coerente con la densità del modello SITL (1.024).
9. Frequenza della fisica ≥ `SCHED_LOOP_RATE` (400 in SITL, 200 sul Navigator).

---

## 14. Domande aperte / verifiche necessarie

1. **Mappatura SERVO ↔ motori in SITL e sul reale.** Leggere `SERVO1..16_FUNCTION`, `MOT_n_DIRECTION`, `FRAME_CONFIG`, `MOT_PWM_MIN/MAX` dal veicolo reale (in una fase successiva, con autorizzazione) e da SITL. Confrontare con §4.2 e con l'audit HoloOcean V7.
2. **Build di Sub-4.1 su Ubuntu in WSL2** (versione di Python, `install-prereqs`, gcc): compila senza patch?
3. **Frequenza minima di HoloOcean** sostenibile in lockstep con i sensori attivi. Comportamento di EKF3 a 200 e a 400 Hz.
4. **Inoltro UDP** in WSL2 NAT ed eventuali regole del firewall Hyper-V: prova pratica con `--model JSON:<IP_host>` rispetto al modo mirrored.
5. **Nome esatto in master/4.7** del parametro che disattiva il GPS SITL (`SIM_GPS1_ENABLE`?). In 4.1: `SIM_GPS_DISABLE=1`.
6. **Home con altitudine ≠ 0:** effetto reale sulla profondità di ArduSub, dato che la calibrazione a terra all'avvio sottrae la pressione iniziale. Da verificare con un test.
7. **Avvio in immersione:** se HoloOcean parte con z > 0, la pressione di riferimento al boot di ArduSub dà profondità 0 nel punto sbagliato. Avviare con z = 0 oppure verificare.
8. **Default degli endpoint MAVLink di BlueOS 1.3.1** e `SYSID_MYGCS` sul veicolo. Conflitto possibile se QGC (sysid 255) e l'autonomia inviano entrambi MANUAL_CONTROL.
9. **Configurazione del gripper** in 4.1 (funzione servo / pulsanti) sul veicolo reale.
10. **Accelerazione fornita da HoloOcean** (con o senza gravità, frame) rispetto alla derivata `Δv/Δt`: scegliere la fonte di `accel_body` (audit HoloOcean §3.2-3.3).
11. **Stabilità del parser 4.1** con numeri in notazione esponenziale e lunghezza massima della riga (buffer 65000 B): non critico, da testare.
12. **Discrepanza della readme** ("10 secondi") rispetto al codice (~1 s) per il resend: confermare a runtime.

---

## 15. Elementi di evidenza (per il manifest JSON)

Tutti recuperati il **2026-10-13**. Base raw: `https://raw.githubusercontent.com/ArduPilot/ardupilot/<ref>/<path>`; blob: `https://github.com/ArduPilot/ardupilot/blob/<ref>/<path>`.

| ID | Risorsa | Ref / commit |
|---|---|---|
| E1 | https://ardupilot.org/dev/docs/sitl-with-JSON.html | docs (live) |
| E2 | https://github.com/ArduPilot/ardupilot/blob/master/libraries/SITL/examples/JSON/readme.md | master @ 3297d9f115972f0dd0814cabb1a17f9231c71530 |
| E3 | https://github.com/ArduPilot/ardupilot/blob/Sub-4.1/libraries/SITL/examples/JSON/readme.md | Sub-4.1 @ 2dd0bb7d4c85ac48437f139d66df648fc0e1d4ae |
| E4 | https://github.com/ArduPilot/ardupilot/blob/Sub-4.1/libraries/SITL/SIM_JSON.h | Sub-4.1 @ 2dd0bb7d |
| E5 | https://github.com/ArduPilot/ardupilot/blob/Sub-4.1/libraries/SITL/SIM_JSON.cpp | Sub-4.1 @ 2dd0bb7d |
| E6 | https://github.com/ArduPilot/ardupilot/blob/master/libraries/SITL/SIM_JSON.h | master @ 3297d9f1 |
| E7 | https://github.com/ArduPilot/ardupilot/blob/master/libraries/SITL/SIM_JSON.cpp | master @ 3297d9f1 |
| E8 | https://github.com/ArduPilot/ardupilot/blob/Sub-4.5/libraries/SITL/SIM_JSON.h (+ `.cpp`) | Sub-4.5 @ abe1721cf52535af6eb2340e5cabed430dac76b5 |
| E9 | https://github.com/ArduPilot/ardupilot/blob/ArduSub-stable/libraries/SITL/SIM_JSON.h (+ `.cpp`) | tag ArduSub-stable (V4.7.2) @ fb773567896014da007bca58358704d4a0d8516f |
| E10 | https://github.com/ArduPilot/ardupilot/blob/master/libraries/SITL/SIM_Aircraft.cpp (+ `.h`) | master @ 3297d9f1 |
| E11 | https://github.com/ArduPilot/ardupilot/blob/Sub-4.1/libraries/SITL/SIM_Aircraft.cpp (+ `.h`): `time_advance` L227-237, `rate_hz` h L223, `use_time_sync` h L233 | Sub-4.1 @ 2dd0bb7d |
| E12 | https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_SITL/SITL_State.cpp | master @ 3297d9f1 |
| E13 | https://github.com/ArduPilot/ardupilot/blob/Sub-4.1/libraries/AP_HAL_SITL/SITL_State.cpp | Sub-4.1 @ 2dd0bb7d |
| E14 | https://github.com/ArduPilot/ardupilot/blob/Sub-4.1/libraries/AP_HAL_SITL/SITL_cmdline.cpp | Sub-4.1 @ 2dd0bb7d |
| E15 | https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_SITL/SITL_cmdline.cpp (JSON L210, porte L253-266, `set_interface_ports` L677) | master @ 3297d9f1 |
| E16 | https://github.com/ArduPilot/ardupilot/blob/master/Tools/autotest/sim_vehicle.py | master @ 3297d9f1 |
| E17 | https://github.com/ArduPilot/ardupilot/blob/Sub-4.1/Tools/autotest/sim_vehicle.py | Sub-4.1 @ 2dd0bb7d |
| E18 | https://github.com/ArduPilot/ardupilot/blob/Sub-4.1/Tools/autotest/pysim/vehicleinfo.py ; https://github.com/ArduPilot/ardupilot/blob/master/Tools/autotest/pysim/vehicleinfo.json (+ `vehicleinfo.py`) | Sub-4.1 / master |
| E19 | https://github.com/ArduPilot/ardupilot/blob/Sub-4.1/Tools/autotest/default_params/sub.parm , `.../sub-6dof.parm` (stessi path su master) | Sub-4.1 / master |
| E20 | https://github.com/ArduPilot/ardupilot/blob/Sub-4.1/libraries/AP_Motors/AP_Motors6DOF.cpp (+ `.h` L21-30 enum dei frame); master stesso path | Sub-4.1 / master |
| E21 | https://github.com/ArduPilot/ardupilot/blob/master/libraries/SITL/SIM_Submarine.cpp (+ `.h`); Sub-4.1 stesso path (diff) | master / Sub-4.1 |
| E22 | https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Baro/AP_Baro_SITL.cpp ; Sub-4.1 `AP_Baro_SITL.cpp`, `AP_Baro_HIL.cpp` (L37-72), `AP_Baro.cpp` (L130-134) | master / Sub-4.1 |
| E23 | https://github.com/ArduPilot/ardupilot/blob/Sub-4.1/libraries/SITL/SITL.cpp ; master stesso path | Sub-4.1 / master |
| E24 | https://github.com/ArduPilot/ardupilot/blob/Sub-4.1/libraries/AP_Scheduler/AP_Scheduler.cpp ; https://github.com/ArduPilot/ardupilot/blob/Sub-4.1/ArduSub/Parameters.cpp ; master `ArduSub/Parameters.cpp` | Sub-4.1 / master |
| E25 | https://github.com/ArduPilot/ardupilot/blob/Sub-4.1/ArduSub/defines.h ; https://github.com/ArduPilot/ardupilot/blob/master/ArduSub/mode.h | Sub-4.1 / master |
| E26 | https://github.com/ArduPilot/ardupilot/blob/Sub-4.1/ArduSub/GCS_Mavlink.cpp ; `.../Sub-4.1/ArduSub/control_althold.cpp` ; `.../Sub-4.1/ArduSub/control_guided.cpp` ; https://github.com/ArduPilot/ardupilot/blob/master/ArduSub/GCS_MAVLink_Sub.cpp | Sub-4.1 / master |
| E27 | https://github.com/ArduPilot/ardupilot/blob/Sub-4.1/libraries/SITL/examples/JSON/pybullet/robot.py ; master stesso path | Sub-4.1 / master |
| E28 | https://api.github.com/repos/ArduPilot/ardupilot/commits?path=libraries/SITL/SIM_JSON.cpp (e `.h`) | API, 2026-10-13 |
| E29a | https://www.ardusub.com/developers/pymavlink.html | docs (live) |
| E29b | https://www.ardusub.com/developers/rc-input-and-output.html | docs (live) |
| E29c | https://www.ardusub.com/developers/sitl.html | docs (live) |
| E30 | https://ardupilot.org/dev/docs/learning-ardupilot-uarts-and-the-console.html | docs (live) |
| E31 | https://ardupilot.org/dev/docs/sitl-on-windows-wsl.html | docs (live) |
| E32a | https://learn.microsoft.com/en-us/windows/wsl/networking | MicrosoftDocs/WSL @ 7b28cc1e |
| E32b | https://learn.microsoft.com/en-us/windows/wsl/wsl-config | MicrosoftDocs/WSL @ 7ea1c6f9 |
| E33 | https://blueos.cloud/docs/1.3/usage/advanced/ | docs BlueOS 1.3 |
| E34 | https://holoocean.readthedocs.io/en/latest/usage/units-and-coordinates.html | docs (live) |
| E35 | https://github.com/ArduPilot/ardupilot/blob/Sub-4.1/libraries/AP_HAL_SITL/RCOutput.cpp (L51); master stesso path (L66) | Sub-4.1 / master |
| E36 | https://github.com/ArduPilot/ardupilot/blob/ArduSub-stable/ArduSub/version.h ; branch list (API `branches`) | 2026-10-13 |

Nota sugli SHA: lo SHA del ref `tags/ArduSub-4.1.2` coincide con la testa del branch `Sub-4.1` (`2dd0bb7d…`). Il branch `Sub-4.1` contiene quindi esattamente la 4.1.2, confermato anche da `version.h` = "ArduSub V4.1.2".
