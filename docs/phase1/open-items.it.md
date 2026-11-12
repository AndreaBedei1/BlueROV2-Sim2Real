# Fase 1 — Verifiche mancanti (TODO consolidato)

Data: 2026-10-13. Raccoglie le voci pendenti di tutti i documenti di questa cartella, deduplicate.
Priorità: **P0** = blocca l'avvio della Fase 2; **P1** = da chiudere durante/prima del primo esperimento SITL o prima di qualsiasi prova in acqua; **P2** = utile per la calibrazione sim-to-real; **P3** = documentale/minore.
Tipo di azione: **OFF** = solo PC/offline (consentito ora); **RO-DEV** = lettura sul veicolo senza effetti fisici (GET/ICMP; consentito); **AUTH** = richiede autorizzazione esplicita perché invia comandi o modifica stato del veicolo/PC; **WATER** = richiede il veicolo in acqua e una fase autorizzata.

| ID | Pr. | Tipo | Verifica | Istruzioni operative | Fonte |
|---|---|---|---|---|---|
| T01 | **P0** | OFF | Compilare ArduSub SITL **4.1.2** (`ArduSub-4.1.2`, `2dd0bb7d`) in WSL2 Ubuntu-24.04 | Ispezionare la distro (`wsl -d Ubuntu-24.04`), clonare ArduPilot al tag, `Tools/environment_install/install-prereqs-ubuntu.sh`, `./waf configure --board sitl && ./waf sub`. Se fallisce (Python/gcc recenti) → container Ubuntu 20.04. Registrare versione gcc/python e hash binario | `sitl-interface-spec.it.md` §2.1, §14.2; `software-inventory.it.md` §8.1 |
| T02 | **P0** | OFF | Spazio disco: ~15 GB liberi su C: | Concordare con l'utente cosa spostare (`<HOB>` 34 GB, `<UE-5.3>` ~20 GB) o usare disco esterno per ArduPilot e log | `software-inventory.it.md` §8.3 |
| T03 | **P0** | OFF | Rete WSL2 ↔ Windows per UDP 9002 | Verificare modalità (`<user-home>\.wslconfig`, `networkingMode=mirrored`); test con socket UDP echo prima del SITL. Modificare `.wslconfig` o il firewall richiede consenso dell'utente | `sitl-interface-spec.it.md` §12, §14.4 |
| T04 | **P0** | OFF | Scelta del binario HoloOcean: **patchato** (`4d6c2d46…`) | Eseguire `Battery/tools/check_holoocean_backend.py --binary <HOB>/packages/patched-after/...` (atteso PASS) e sul binario installato (atteso FAIL). Registrare hash nei manifest | `holoocean-physics-audit.it.md` §10 V1 |
| T05 | **P1** | OFF | Mapping canali ArduSub → thruster HoloOcean e segni | Ipotesi: M1→t4(−), M2→t5(−), M3→t7(+), M4→t6(+), M5→t0(−), M6→t1(−), M7→t3(−), M8→t2(−). Matrice VECTORED_6DOF confermata sul sorgente 4.1.2 (`AP_Motors6DOF.cpp:156-166`). Validare con MOTOR_TEST/RC override **nel SITL** (esperimento E1) | `holoocean-physics-audit.it.md` §7.2, V7; `hardware-inventory.it.md` §2.3 |
| T06 | **P1** | OFF | IMU HoloOcean: specific force e frame per socket COM/IMUSocket | Procedure V2–V3 (z=−5 m, forze nulle, tps 100; poi rotazioni pure) | `holoocean-physics-audit.it.md` §10 |
| T07 | **P1** | OFF | Costo per tick e RTF a 100/200 Hz senza viewport | Procedura V8 + harness `BlueROV2SimMatch/scripts/verify_env.py`; decidere dt definitivo | `holoocean-physics-audit.it.md` V8; `phase2-architecture-proposal.it.md` §2 |
| T08 | **P1** | OFF | Sleep dei corpi Unreal sulla build patchata | Procedura V9 (0.05 N dopo 10 s di quiete; con/senza dither ±0.02 N) | `holoocean-physics-audit.it.md` V9 |
| T09 | **P1** | OFF | Profondità SITL da `position.z` (densità 1.024 vs `BARO_SPEC_GRAV=1.0`), home alt 0, avvio non immerso | Procedura V10; test con home alt 0 e veicolo a z=0 al boot | `sitl-interface-spec.it.md` §14.6–14.7 |
| T10 | **P1** | OFF | Gestione resend (~1 s nel codice vs 10 s readme) e duplicati di frame_count in 4.1.2 | Test con stub fisico che ritarda la risposta | `sitl-interface-spec.it.md` §7.4, §14.12 |
| T11 | **P1** | OFF | Clamp ±28.75 N HoloOcean vs T200 (51.4/39.9 N @16 V) | Decidere: accettare + loggare saturazione (Fase 2) o patch `BR_MAX_THRUST` (nuova build UE) | `holoocean-physics-audit.it.md` §8.2 |
| T12 | **P1** | AUTH | **Scaricare dal ROV i log mancanti** (87 file, 1.62 GB): BIN 26–28, tlog 00151–00164 | Download via File Browser BlueOS (HTTP GET, nessun effetto fisico) **su disco esterno**; checksum; non duplicare in altre cartelle. Richiede l'ok dell'utente per volume e spazio | `sensor-data-audit.it.md` §8.1 |
| T13 | **P1** | AUTH | Parametri correnti = backup? (oggi confermato solo il conteggio 913) | Lettura parametri via MAVLink (PARAM_REQUEST_LIST è un invio: vietato in Fase 1) oppure confronto `ArduSub.stg` via File Browser GET | `hardware-inventory.it.md` §11.3 |
| T14 | **P1** | WATER | Cablaggio fisico thruster → uscita Navigator e elica CW/CCW per posizione | Ispezione visiva (a secco, consentita) + test motori in acqua con autorizzazione | `hardware-inventory.it.md` §11.1 |
| T15 | **P1** | WATER | Segni della spinta dei verticali 5–8 (mai provati in acqua) | Prova heave/roll/pitch a bassa intensità, operatore presente | `hardware-inventory.it.md` §11.2 |
| T16 | **P1** | AUTH | Riferimento di pressione: `BARO2_GND_PRESS=97800` → +0.52 m in aria (confermato oggi) | Calibrazione in superficie prima di ogni immersione (è una modifica di stato) | `hardware-inventory.it.md` §11.4 |
| T17 | **P1** | — | **Ping1D trasmette in aria** e Surveyor alimentata (stato ping ignoto) — preesistente | Decisione del team; verifica fisica dell'alimentazione Surveyor. Nessuna azione eseguita | `hardware-inventory.it.md` §11.5 |
| T18 | **P1** | AUTH | Orologio BlueOS indietro di ~7.7 giorni; PC non sincronizzato; ArduSub senza UTC | Sincronizzare prima di acquisizioni da allineare (modifica di stato) | `hardware-inventory.it.md` §7.1 |
| T19 | **P1** | OFF | Geometria/massa reale per HoloOcean (posizioni thruster, angolo vettoriali, CG/CB, zavorre) | Misure fisiche a secco + CAD Blue Robotics; confronto con `BlueROV2.h` (11.5 kg) | `hardware-inventory.it.md` §11.6 |
| T20 | P2 | OFF | Inerzia della mesh, velocità di regime, decelerazione (drag patchato) | Procedure V5–V6; confronto futuro con tlog 2025-11-17 `00100`/BIN `00000016` e log da T12 | `holoocean-physics-audit.it.md` V5–V6 |
| T21 | P2 | OFF | Offset BlueOS−PC per 10/09 e 15/09 | Confronto `SYSTEM_TIME` tlog bordo vs QGC | `sensor-data-audit.it.md` §8.3 |
| T22 | P2 | OFF | 162 checksum errati nel `surveyor_raw.svlog` del 15/09 | Lettura (sola) della storia git `c13cb21`/`9394c82` del Recorder | `sensor-data-audit.it.md` §8.4 |
| T23 | P2 | AUTH | Recuperare svlog completi su ROV (`/userdata/SonarView/Surveyor 240-16/`, file ~500 MB del 10/09) | GET File Browser verso disco esterno | `sensor-data-audit.it.md` §8.2 |
| T24 | P2 | WATER | Latenza glass-to-glass camera; Ping1D con profilo; ROVL `$USRTH` in acqua | Protocollo con orologio al ms ripreso; nuova sessione autorizzata | `sensor-data-audit.it.md` §8.8–8.9 |
| T25 | P2 | OFF | `VFR_HUD.alt` costante nei tlog: usare `GLOBAL_POSITION_INT.relative_alt`/`SCALED_PRESSURE2` | Rianalisi tlog con pymavlink | `sensor-data-audit.it.md` §8.11 |
| T26 | P2 | — | Modello profondimetro (Bar30 inferito), luci, laser (uscita 14), gripper (uscita 9), batteria, camera | Ispezione visiva a secco | `hardware-inventory.it.md` §11.8–11.11 |
| T27 | P2 | RO-DEV | Origine del sistema MAVLink 2 visto in mavlink2rest | GET configurazione servizi BlueOS | `hardware-inventory.it.md` §11.13 |
| T28 | P3 | OFF | Licenza client HoloOcean e BiguaSim; citazioni MDPI dal PDF ufficiale (pagina 403) | Download manuale del PDF dall'utente | `simulator-comparison.it.md` §11.6–11.7 |
| T29 | P3 | OFF | Citazioni marcate (†) in `software-inventory.it.md` | Ricontrollo riga per riga prima di usarle in pubblicazioni | `software-inventory.it.md` §8.12 |
| T30 | P3 | OFF | Aggiungere hash binario, tps e RTF ai manifest di Middleware | In Fase 2, nel nuovo `runlog` | `software-inventory.it.md` §8.9 |
| T31 | P3 | RO-DEV | `mavlink-camera-manager /info`, firmware Ping1D e Surveyor | GET dopo verifica assenza di effetti; firmware Surveyor solo con connessione autorizzata | `hardware-inventory.it.md` §11.14–11.16 |

## Ordine consigliato

1. T02 → T01 → T03 (sbloccano il SITL).
2. T04, T06, T07, T08 (HoloOcean pronto per il lockstep) in parallelo a T01.
3. Implementazione Fase 2, poi T05, T09, T10, T11 durante gli esperimenti E0–E3 di `phase2-architecture-proposal.it.md`.
4. Con l'utente: T12, T13, T23 (dati dal veicolo, nessun effetto fisico ma volumi grandi e invio MAVLink per T13).
5. In una fase in acqua autorizzata: T14–T16, T18, T24.
