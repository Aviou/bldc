# MCSA-Aufnahmemodus für die VESC-Firmware 6.02 — Design

Stand 2026-10-13. Branch `feat/mcsa-capture-6.02` (Basis `vedderb/bldc` `release_6_02`), Fork `Aviou/bldc`.

Auftrag: `docs/mcsa-vesc-firmware-auftrag.md` im Repo `Aviou/motorpi` (Branch `docs/mcsa-vesc-sampleprint`).
Messgrundlage: `docs/mcsa-vesc-sampleprint-2026-10.md` ebenda.
Designentscheidung Puffer gegen Streaming: zweites Gutachten durch Codex, Ergebnis in Abschnitt 2.

## 1. Ziel

Phasenströme und Rotorwinkel am Prüfstand so aufzeichnen, dass die Motorstrom-Signaturanalyse (MCSA) bessere Daten bekommt als mit `COMM_SAMPLE_PRINT`:

- länger: mehr Werte im selben Speicher, dazu eine Daueraufnahme, die nicht am RAM hängt;
- sauberer: Dezimationsfilter statt Auslassen, keine Rundung auf ganze ADC-Stufen, Winkel zeitlich passend zum gefilterten Strom;
- vollständiger: alle drei Shunt-Kanäle so, wie sie gemessen wurden;
- verlustfrei abholbar: jeder Wert hat einen Index, Verluste werden gemeldet statt still verworfen;
- phasenrichtig verbindbar: ein durchlaufender Umdrehungszähler macht getrennte Aufnahmen kohärent auswertbar.

Regelung, Konfigurationsformat, Firmware-Version und `COMM_SAMPLE_PRINT` bleiben unverändert.

### Nicht im Umfang

- Einbau in den Testablauf der motorpi-App (eigener Schritt danach).
- Flashen der Controller und jeder Schreibzugriff auf Geräte. Das macht der Nutzer nach der Anleitung in `docs/mcsa_capture_flashing.md`.
- Zweiter Motor auf Dual-Hardware. Aufgezeichnet wird nur Motor 1. HW 60 hat ohnehin nur einen.
- Mehrstufige Dezimation mit FIR (5.5, mögliche Erweiterung).

## 2. Entscheidung: gemeinsamer Ringpuffer mit Einzel- und Daueraufnahme

Verglichen wurden Puffer-Aufnahme (A), Streaming (B) und eine Kombination (C). Ergebnis, mit Codex abgeglichen:

- Die größten Qualitätsgewinne sind in A und B gleich: Dezimationsfilter, Nachkommabits, Winkel zur Fenstermitte, drei unabhängige Shuntwerte.
- Darüber hinaus gewinnt B für drehsynchrone Linien durch längere kohärente Integration. Ideal bei 12 000 ERPM: 4 × 1,7 s phasenrichtig verbunden ≈ +6 dB gegenüber einer Aufnahme, 20 s Daueraufnahme ≈ +10,7 dB. Systematische Fehler (Offset, Rekonstruktion) mitteln sich nicht weg.
- Über CAN 500 kbit/s trägt B nur einen Knoten zur Zeit und bei ia/ib/ic/Winkel höchstens etwa 1,5 kHz Ausgaberate. Über USB gibt es reichlich Reserve.

Gewählt ist C in einer Firmware: ein Ringpuffer mit absolutem Sample-Index, zwei Betriebsarten (Einzelaufnahme mit hoher Rate, Daueraufnahme mit fortlaufendem Abholen), dazu ein Umdrehungszähler, der unabhängig von der Aufnahme weiterläuft. Eine Flash-Runde.

## 3. Randbedingungen (gemessen bzw. aus Code und Map)

| Größe | Wert |
|---|---|
| Hardware | Flipsky FSESC 6.7 Pro, meldet HW „60“ (Build-Ziel `60`, `hwconf/trampa/vesc6/hw_60.h`), drei Phasen-Shunts |
| Stromskalierung | `FAC_CURRENT` = 3,3 V / 4095 / (0,5 mΩ · 20) = 80,6 mA pro ADC-Stufe |
| ISR-Rate | `foc_f_zv` = 60 kHz, `foc_sample_v0_v7` = 0 → 30 kHz (`mcpwm_foc_get_sampling_frequency_now()`) |
| Polpaare | 7 (`si_motor_poles` = 14) |
| CCM `.ram4` (Stock 6.02) | 57 640 von 63 488 B belegt, davon 36 000 B `COMM_SAMPLE_PRINT`-Puffer, 19 712 B LispBM-Heap; frei 5 848 B |
| Haupt-RAM | ~11,4 KB über `.bss`, als ChibiOS-Heap ausgewiesen; nicht für den Aufnahmepuffer verwenden |
| CAN | 500 kbit/s, Statusmeldungen 50 Hz × 4 Frames × 4 Knoten; Fragmentierung in `comm_can_send_buffer` ≈ 6,3 Nutzbyte pro Frame bei großen Paketen; praktisch ≈ 17–21 kB/s |
| Paketgröße | `PACKET_MAX_PL_LEN` = 512 (gilt auch für den CAN-Reassemblierungspuffer) |

## 4. Aufbau

```
mcpwm_foc_adc_int_handler()                    (FOC-ISR, 30 kHz)
  ... Stromerfassung, FOC, Tachometer ...
  mcsa_capture_isr_angle(phase)          <- neu, immer: Umdrehungszähler
  if mcsa_capture_isr_wants_sample():    <- neu, nur bei laufender Aufnahme
      mcsa_capture_isr_sample(&in)          CIC-Filter + Ringpuffer
  mc_interface_mc_timer_isr()
  m_last_adc_isr_duration = ...
  mcsa_capture_isr_end(dauer)            <- neu: ISR-Laufzeit

commands_process_packet()
  case COMM_CUSTOM_APP_DATA:
      if mcsa_capture_process_cmd(data, len, reply_func)  <- neu, prüft Magic
          break
      ... bisheriger Weg (appdata_func, LispBM) unverändert ...

mc_interface.c
  COMM_SAMPLE_PRINT-Arrays liegen in einer Union mit dem MCSA-Puffer (Layout unverändert)
  Sende-Zähler, mc_interface_sample_print_busy(), Sperre in mc_interface_sample_print_data()
  mc_interface_set_configuration() -> mcsa_capture_config_changed()
```

### Dateien

| Datei | Inhalt |
|---|---|
| `motor/mcsa_core.c/.h` | Hardwareunabhängiger Kern: Umdrehungszähler, CIC-Filter, Ringpuffer, Befehlsauswertung, Serialisierung. Keine ChibiOS-/HAL-Abhängigkeit, auf dem PC übersetzbar. |
| `motor/mcsa_capture.c/.h` | Einbindung in die Firmware: Union-Puffer in `.ram4`, Mutex, Zusammenspiel mit `COMM_SAMPLE_PRINT`, Umgebung für den Kern (f_ISR, `FAC_CURRENT`, Polpaare, Git-Hash, Laufzeit, kritische Abschnitte), Antwort über `reply_func`. |
| `motor/mcpwm_foc.c` | drei Hooks in der ISR (Abschnitt 8); füllt `mcsa_sample_in` aus dem (statischen) Motorzustand und TIM1. Nur für Motor 1. |
| `comm/commands.c` | Abzweig in `case COMM_CUSTOM_APP_DATA` |
| `motor/mc_interface.c` | Puffer-Makros, Sende-Zähler, `mc_interface_sample_print_busy()`, Sperre in `mc_interface_sample_print_data()`, Aufruf bei Konfigurationsänderung, `mcsa_capture_init()` |
| `motor/motor.mk` | neue Quelldateien |
| `tests/mcsa_core/` | PC-Tests des Kerns (C, ohne externe Bibliotheken) und gemeinsame Bibliothek für die Python-Tests |
| `mcsa_client/mcsa_capture_client.py` | Referenzclient (pyserial, numpy) |
| `mcsa_client/test_mcsa_capture_client.py` | Tests des Clients, u. a. gegen den Kern per ctypes |
| `docs/mcsa_capture_protocol.md` | Protokoll |
| `docs/mcsa_capture_flashing.md` | Flash- und Rückweg-Anleitung |

## 5. Signalpfad

### 5.1 Eingänge je ISR-Durchlauf

Erfasst am Ende der FOC-ISR, also im selben Durchlauf, in dem Ströme, Offsets und Winkel berechnet wurden:

| Größe | Quelle | Bemerkung |
|---|---|---|
| Rohströme a0, a1, a2 | `m_currents_adc[0..2]` (ADC-Zählwerte) | **vor** Offsetabzug und **vor** der Ersetzung eines Kanals durch −(a+b) |
| Offsets o0, o1, o2 | `m_conf->foc_offsets_current[0..2]` | beim Start eingefroren; Änderung beendet die Aufnahme |
| Winkel | `m_motor_state.phase` (rad) | derselbe Winkel, den die FOC und `COMM_SAMPLE_PRINT` verwenden |
| id, iq, vd, vq | `m_motor_state` | optional per Kanalmaske (Bedeutung 5.4) |
| CCR1–3, ARR | TIM1 | für den Flag-Kanal |
| Motorzustand | `m_state` | für den Flag-Kanal |

Hinweis zu den Rohströmen: Die FOC ersetzt je nach Modulation (bei niedriger Modulation fest Kanal 2) einen Kanal durch −(a+b). Die Aufnahme liefert trotzdem die gemessenen Werte, weil genau sie für die Fehlersuche im Messpfad gebraucht werden. Bei hoher Modulation ist der Shunt der Phase mit der längsten Einschaltdauer nicht sauber messbar; der Flag-Kanal (5.3) zeigt das an.

### 5.2 Umdrehungszähler

Läuft in jedem ISR-Durchlauf, auch ohne Aufnahme und im Stillstand.

- `frac16 = (uint16_t)(int32_t)(phase · 65536 / (2π))` (Wandlung durch Abschneiden, Fehler < 1/65536 Umdrehung); Winkel in 1/65536 elektrischer Umdrehung. NaN wird ignoriert.
- `θ_u += (int16_t)(frac16 − frac16_last)`; `θ_u` ist `int64_t`, nie zurückgesetzt außer beim Booten.
- Annahme: weniger als eine halbe elektrische Umdrehung pro ISR-Durchlauf (bei 30 kHz bis 15 kHz elektrisch erfüllt).
- Mechanischer Winkel bis auf einen beliebigen, im Lauf festen Nullpunkt: `θ_m = θ_u / (65536 · Polpaare)`.

Grenzen: Der Winkel ist bei sensorlosem Betrieb eine Schätzung des Beobachters. Über Beobachterverlust, Neustart des Motors oder Reboot hinweg ist die mechanische Zuordnung nicht garantiert. Innerhalb eines stabil laufenden Motorlaufs bleibt sie erhalten.

### 5.3 CIC-Dezimationsfilter

- Ordnung 3, Dezimierung R ∈ [1, 64], Differenzverzögerung 1, linearphasig.
- **Arithmetik modular und wohldefiniert:** Integratoren und Kämme rechnen in `uint32_t` (Ströme) bzw. `uint64_t` (id/iq/vd/vq/Winkel). Überläufe der Integratoren sind damit definiert und heben sich im Kamm auf (Hogenauer), solange das **Kammergebnis** in die Wortbreite passt. Das Kammergebnis wird anschließend als vorzeichenbehaftete Zahl gelesen.
- **Offset fest je Aufnahme:** Beim ersten ISR-Durchlauf nach `START` werden die drei Offsets eingefroren und auf 1/8 Stufe gerundet: `off_q3 = round(8·o)`, zerlegt in `oi = floor(off_q3 / 8)` und `of = off_q3 − 8·oi` ∈ [0, 7]. Ändert sich ein Offset während der Aufnahme (z. B. `foc_dc_cal`), endet die Aufnahme mit Grund `OFFSETS_CHANGED`; die bis dahin geschriebenen Daten bleiben gültig.
- **Ströme ia/ib/ic:** Eingang `x = a − oi` (ganze Stufen, |x| ≤ 4095 → 13 Bit mit Vorzeichen). Kammergebnis `y` ≤ 13 + 3·log2(64) = 31 Bit. Ausgabe in 1/8 Stufe mit **einer** Rundung: `out = round(8·y / R³) − of`. Berechnet ohne Überlauf über Quotient und Rest (`q = y / R³`, `r = y % R³`, `out = 8q + round(8r / R³) − of`), Rundung „halb von null weg“. Fehler insgesamt ≤ 1/16 Stufe (Offset) + 1/16 Stufe (Ausgabe). Wertebereich passt immer in `int16_t` (|out| ≤ 32 767).
- **id/iq** (Eingang 1/8 Stufe: `round(I / FAC_CURRENT · 8)`), **vd/vq** (Eingang mV): Kammergebnis in `int64_t`, Ausgabe `round(y / R³)`, `int16_t` mit Sättigung. Sättigungen werden je Aufnahme gezählt.
- **Winkel:** Eingang `θ_u − θ_start` (θ_start = θ_u im ersten Durchlauf nach `START`). Der Winkel durchläuft **dasselbe Filter** wie die Ströme. Weil der CIC linearphasig ist, sind alle Frequenzanteile gleich verzögert; der gefilterte Winkel passt damit zum gefilterten Strom, auch bei halbzahliger Gruppenlaufzeit. Ausgabe `θ_f = θ_start + round(y / R³)` (1/65536 Umdrehung). Gespeichert wird `uint16_t` = Bits 3–18 von θ_f, also 1/8192 elektrische Umdrehung modulo 8 Umdrehungen. Zusätzlich hält der Kern `θ_f` des neuesten Datensatzes als `int64_t`; daraus erhält jede `READ`-Antwort einen absoluten Anker (6.5). Erreicht |θ_f − θ_start| 2^44 (≈ 2,7 · 10^8 Umdrehungen), endet die Aufnahme mit `ANGLE_LIMIT`.
- **Flag-Kanal** (optional): je Datensatz Bit k (k = 0..2) gesetzt, wenn im Einflussbereich des Filters (die letzten 3·R ISR-Durchläufe) mindestens einmal `CCRk+1 > ARR − 500` galt, also Phase k so hoch moduliert war, dass ihr Shunt nicht sauber messbar ist (dieselbe Schwelle, die die FOC für die Kanalwahl nutzt). Bit 3 gesetzt, wenn der Motor in allen diesen Durchläufen im Zustand `RUNNING` war. Die CCR-Werte werden am Ende der ISR gelesen und gehören damit zur nächsten PWM-Periode; bei 30 kHz ist das ein Versatz von 2,4° elektrisch bei 200 Hz.
- Ausgabe alle R ISR-Durchläufe. Die ersten **2** Ausgabewerte nach `START` werden verworfen; der erste gespeicherte Wert stammt damit aus vollständig nach dem Start liegenden Eingangswerten (3·R ≥ 3(R−1)+1).
- Gruppenlaufzeit 3(R−1)/2 Eingangsabtastungen. Datensatz n (ab 0) gehört zum Eingangsindex `(n+3)·R − 1 − 3(R−1)/2` nach dem Start, also zur Zeit `t_n = ((n+3)·R − 1 − 1,5·(R−1)) / f_ISR`.
- Übertragungsfunktion |H(f)| = |sin(πRf/f_ISR) / (R·sin(πf/f_ISR))|³. Der Client korrigiert Amplituden mit |H| an der **tatsächlichen** Frequenz jeder Spektrallinie.

Nutzbares Band (absolute Unterdrückung jeder Rückfaltung ≥ 40 dB bzw. ≥ 30 dB), f_ISR = 30 kHz:

| R | Ausgaberate | Band ≥ 40 dB | Band ≥ 30 dB | Störer 11 kHz | 5000 Werte |
|---|---|---|---|---|---|
| 1 | 30 kHz | — (ungefiltert) | — | 0 dB | 0,17 s |
| 5 | 6 kHz | 1067 Hz | 1517 Hz | −58 dB | 0,83 s |
| 10 | 3 kHz | 551 Hz | 780 Hz | −61 dB | 1,67 s |
| 15 | 2 kHz | 370 Hz | 521 Hz | −68 dB | 2,50 s |
| 20 | 1,5 kHz | 278 Hz | 392 Hz | −79 dB | 3,33 s |
| 40 | 750 Hz | 139 Hz | 196 Hz | −98 dB | 6,67 s |

Am Bandrand ist das Nutzsignal selbst um bis zu 1,5 dB gedämpft; nach der Korrektur liegt der Abstand eines gleich starken Störers dort bei etwa 38,5 dB statt 40 dB. Zum Vergleich: Auslassen (heute) dämpft nichts, einfache Mittelung bei R = 10 den 11-kHz-Störer um 20 dB.

### 5.4 Hinweise für die Auswertung

- Park-Transformation und Filter vertauschen nicht: Die Grundwelle der gefilterten Phasenströme ist um |H(f_e)| gedämpft, ein direkt erfasstes iq ist dagegen nahezu Gleichanteil. Für die Drehmomentwelligkeit deshalb den Kanal **iq** direkt aufnehmen oder die Phasenstromspektren vor der Park-Transformation an den tatsächlichen Linienfrequenzen mit |H| korrigieren.
- id/iq sind bei ausgeschaltetem Motor null (die FOC setzt sie so). vd/vq sind die totzeitkompensierten Spannungen aus `update_valpha_vbeta`, nicht die PI-Ausgänge.
- Rohkanäle mit gesetztem Flag-Bit sind mit Vorsicht zu verwenden.

### 5.5 Mögliche Erweiterung (nicht in dieser Version)

Ein breiteres Nutzband bei gleicher Ausgaberate gibt es nur mit mehrstufiger Dezimation: CIC mit kleinerem Faktor, danach ein FIR-Tiefpass **vor** der letzten Dezimierung. Ein FIR hinter dem fertig dezimierten CIC korrigiert nur den Frequenzgang, bereits zurückgefaltete Störungen trennt er nicht mehr. Erst angehen, wenn die Messungen zeigen, dass das Band bei R = 20 nicht reicht.

## 6. Puffer und Betriebsarten

### 6.1 Gemeinsamer Puffer

- `mcsa_capture.c` definiert eine Union in `.ram4` (40 000 B, ausgerichtet auf 8 B):
  - `sp`: typisiertes Layout der zehn Sample-Print-Arrays (8 × `int16_t[2000]`, `uint8_t[2000]`, `int8_t[2000]` = 36 000 B), in derselben Reihenfolge wie bisher;
  - `mcsa`: `int16_t[20000]`.
  Größen und Offsets sind per `_Static_assert` geprüft. `.ram4` ist `NOLOAD`; es gibt keine initialisierten Zeiger in diesem Bereich.
- In `mc_interface.c` werden die bisherigen Array-Definitionen durch Makros auf `mcsa_capture_shared.sp.*` ersetzt. Alle Zugriffe bleiben textgleich.
- Reserve in `.ram4` danach: 1 848 B für Kernzustand und Antwortpuffer bleiben außerhalb (Kernzustand in `.bss`, Antwortpuffer aus `mempools`).

### 6.2 Zusammenspiel mit `COMM_SAMPLE_PRINT`

`COMM_SAMPLE_PRINT` liest den Puffer nicht nur während der Aufnahme, sondern auch danach im Sende-Thread, und `DEBUG_SAMPLING_SEND_LAST_SAMPLES` kann später erneut senden. Deshalb:

- `mc_interface.c` führt zwei Zähler: `m_sample_send_req` (erhöht von jedem, der den Sende-Thread anstößt: die ISR an den vier bestehenden Stellen, die Anfrage `SEND_LAST_SAMPLES`; jeweils im kritischen Abschnitt zusammen mit `chEvtSignalI`) und `m_sample_send_done` (nur vom Sende-Thread gesetzt, **nach** einem vollständigen Sendedurchlauf auf den vorher gelesenen Anfragestand). Der Sende-Thread wiederholt den Durchlauf, solange `req ≠ done`. Bei mehrfach zusammengefassten Anstößen sendet er wie bisher einmal; kommt während des Sendens ein neuer Anstoß, sendet er wie bisher ein weiteres Mal.
- `mc_interface_sample_print_busy()` = `m_sample_mode ≠ OFF` oder `req ≠ done`, im kritischen Abschnitt gelesen.
- Ein gemeinsamer Mutex (`mcsa_capture_lock()`) schützt alle Eigentumswechsel und jede MCSA-Befehlsverarbeitung, auch die Kopie bei `READ`. `mc_interface_sample_print_data()` nimmt ihn für die gesamte Funktion.
- Regeln:
  - `START` nur, wenn `!mc_interface_sample_print_busy()`; sonst `BUSY`. Mit `START` gelten die Sample-Print-Daten als überschrieben (`sp_valid = false`).
  - `COMM_SAMPLE_PRINT` während einer laufenden MCSA-Aufnahme: verworfen (keine Daten, keine Zustandsänderung).
  - `COMM_SAMPLE_PRINT` mit Aufnahmemodus bei beendeter MCSA-Aufnahme: übernimmt den Puffer; die MCSA-Aufnahme wird `INVALID` (Grund `SAMPLE_PRINT`), `sp_valid = true`.
  - `SEND_LAST_SAMPLES` nur, wenn `sp_valid`; sonst verworfen.
  - Modus `OFF` wird immer durchgereicht (schreibt nichts in den Puffer).
- Ohne MCSA-Nutzung verhält sich `COMM_SAMPLE_PRINT` exakt wie in Stock 6.02.

### 6.3 Datensätze

| Bit | Kanal | Typ | Einheit |
|---|---|---|---|
| 0 | ia (Shunt 0, roh) | int16 | 1/8 ADC-Stufe, offsetkorrigiert |
| 1 | ib (Shunt 1, roh) | int16 | 1/8 ADC-Stufe, offsetkorrigiert |
| 2 | ic (Shunt 2, roh) | int16 | 1/8 ADC-Stufe, offsetkorrigiert |
| 3 | id | int16 | 1/8 ADC-Stufe |
| 4 | iq | int16 | 1/8 ADC-Stufe |
| 5 | vd | int16 | mV |
| 6 | vq | int16 | mV |
| 7 | Winkel | uint16 | 1/8192 el. Umdrehung, mod 8 |
| 8 | Flags | uint16 | Bits 0–2 Phase hoch moduliert, Bit 3 Motor lief |

- Kanalmaske `uint16_t`; Kanäle im Datensatz in Bit-Reihenfolge.
- Standardmaske `0x0087` (ia, ib, ic, Winkel) = 8 B je Datensatz → 5000 Datensätze.
- Kapazität = ⌊40 000 / (2 · Anzahl Kanäle)⌋ Datensätze.

### 6.4 Index und Ring

- Jeder Datensatz hat einen absoluten Index n (`uint32_t`) ab Aufnahmestart, Speicherplatz `n mod Kapazität`.
- Der Ring wird über einen `volatile`-Zeiger beschrieben; die ISR schreibt den kompletten Datensatz und erhöht danach `n_written` (`volatile`). Ein Datensatz mit Index < `n_written` ist vollständig (Einkern-System, Thread sieht nie eine halb fertige ISR).
- Ältester verfügbarer Index: `n_written − Kapazität`, falls `n_written ≥ Kapazität`, sonst 0. Bereichsprüfungen nur über Differenzen (`n_written − start`), nie über `start + Kapazität`.
- Obergrenze `n_written` = 0xF000 0000; erreicht, endet die Aufnahme mit `INDEX_LIMIT` (bei R = 1 nach etwa 37 h).

### 6.5 Betriebsarten

| Modus | Verhalten |
|---|---|
| `ONESHOT` | Aufnahme bis Länge L (1 ≤ L ≤ Kapazität, 0 = Kapazität), danach `DONE` mit Grund `LENGTH`. Daten bleiben bis zum nächsten `START` oder bis `COMM_SAMPLE_PRINT` den Puffer übernimmt. |
| `CONTINUOUS` | Aufnahme bis `STOP`. Der Host holt fortlaufend ab. Was er nicht rechtzeitig holt, wird überschrieben und beim Lesen als `OVERWRITTEN` mit dem ältesten verfügbaren Index gemeldet. |

Start immer „sofort“. `START` nur, wenn der Motor im FOC-Modus konfiguriert ist (sonst `NOT_FOC`). Jede Konfigurationsänderung über `mc_interface_set_configuration()` beendet eine laufende Aufnahme mit Grund `CONFIG_CHANGED`.

### 6.6 Konsistentes Lesen und Winkelanker

`READ` läuft im Paket-Thread unter dem Mutex:

1. Im kurzen kritischen Abschnitt `n_written` und `θ_f` des neuesten Datensatzes lesen.
2. Bereich prüfen: Sitzung, `start ≥ ältester`, `start ≤ n_written`.
3. Datensätze kopieren und big-endian serialisieren.
4. Winkelanker: von `θ_f` des neuesten Datensatzes rückwärts bis `start` über die gespeicherten 16-Bit-Winkel abwickeln (Differenzen modulo 2^16, eindeutig nach 5.3). Ergebnis ist der absolute gefilterte Winkel des ersten gelieferten Datensatzes in 1/8192 Umdrehung (`int64_t`). Höchstens Kapazität viele Schritte.
5. `n_written` erneut lesen. Gilt `n_written − start > Kapazität`, wurde während der Kopie überschrieben: Antwort `OVERWRITTEN`.

Interrupts bleiben während Kopie und Abwicklung frei. Der Host wickelt innerhalb eines Blocks ab dem Anker ab; über Lücken hinweg braucht er keine Annahme über die Drehzahl.

## 7. Protokoll

Transport: `COMM_CUSTOM_APP_DATA` (= 36), über USB direkt oder per `COMM_FORWARD_CAN` (= 34). In `commands_process_packet()` ist die äußere Paket-ID bereits entfernt; `data` beginnt beim Magic. Antworten werden **synchron** im Kontext des eingehenden Pakets über dessen `reply_func` gesendet. Über die Bridge darf **für alle Clients zusammen** nur eine Anfrage offen sein (die CAN-Rückleitung der Bridge ist global, nicht an die Anfrage gebunden); der Client öffnet den Port deshalb exklusiv.

- Pakete mit anderem Magic gehen unverändert an `appdata_func` und LispBM.
- Mehrbyte-Werte big-endian; Fließkomma IEEE-754 float32 big-endian.

**Anfrage:** `[0x4D 'M'][0x43 'C'][Version = 1][Befehl][Argumente]`

**Antwortkopf (11 B):** `[0x4D][0x43][1][Befehl | 0x80][Status u8][Sitzung u16][Laufzeit ms u32]`. Die Sitzung ist der aktuelle Sitzungszähler, die Laufzeit `chVTGetSystemTimeX()` in ms. Der Host erkennt einen Reboot daran, dass die Laufzeit zurückspringt.

| Befehl | Code | Argumente | Antwortkörper |
|---|---|---|---|
| `INFO` | 0x01 | — | Protokollversion u8, CIC-Ordnung u8, R_max u8, Polpaare u8, Pufferbytes u32, f_ISR f32, `FAC_CURRENT` f32, Git-Hash 16 B (mit Nullen aufgefüllt) |
| `START` | 0x02 | Modus u8, Maske u16, R u8, Länge u32, Token u16 | bei `OK`: Kapazität u32, Bytes je Datensatz u8, Maske u16, R u8; sonst leer |
| `STOP` | 0x03 | Sitzung u16 | `n_written` u32, Stoppgrund u8 |
| `STATUS` | 0x04 | — | Zustand u8, Stoppgrund u8, Modus u8, Maske u16, R u8, CIC-Ordnung u8, Polpaare u8, Kapazität u32, Länge u32, `n_written` u32, ältester u32, f_ISR f32, A je Einheit f32 (= `FAC_CURRENT`/8), Offsets 3 × f32 (eingefroren, ADC-Stufen), θ_start i64 (1/65536 U.), ISR-Max ohne Aufnahme f32 (µs), ISR-Max mit Aufnahme f32 (µs), Sättigungen u32 |
| `READ` | 0x05 | Sitzung u16, Start u32, Anzahl u16 | bei `OK`: Start u32, Anzahl u16, Bytes je Datensatz u8, Winkelanker i64 (1/8192 U., 0 ohne Winkelkanal), Datensätze, CRC16 über die Datensätze; sonst: ältester u32, `n_written` u32 |

- **Idempotenter `START`:** Token ≠ 0 und gleich dem Token des letzten erfolgreichen `START`, gleiche Parameter, Zustand `RUNNING` oder `DONE` → Antwort `OK` mit der bestehenden Sitzung, ohne Neustart. Der Client wiederholt `START` nach Zeitüberschreitung mit demselben Token.
- `STOP` mit fremder Sitzung → `NOT_AVAILABLE`.
- `READ` mit `start = n_written` → `OK` mit Anzahl 0 (noch nichts Neues). `start > n_written` oder fremde Sitzung oder Zustand `IDLE`/`INVALID` → `NOT_AVAILABLE`.
- Höchstens ⌊(511 − 28) / Bytes je Datensatz⌋ Datensätze je `READ` (511 = `PACKET_MAX_PL_LEN` − 1 für die Paket-ID; 28 = Kopf + Felder + CRC), bei 8 B also **60**.
- Zustände: 0 `IDLE`, 1 `RUNNING`, 2 `DONE`, 3 `INVALID`.
- Stoppgründe: 0 keiner, 1 `LENGTH`, 2 `STOP_CMD`, 3 `OFFSETS_CHANGED`, 4 `CONFIG_CHANGED`, 5 `INDEX_LIMIT`, 6 `ANGLE_LIMIT`, 7 `SAMPLE_PRINT`.
- Statuscodes: 0 `OK`, 1 `BUSY`, 2 `BAD_ARG`, 3 `NOT_AVAILABLE`, 4 `OVERWRITTEN`, 5 `UNKNOWN_CMD`, 6 `BAD_VERSION`, 7 `NOT_FOC`. Zu kurze Anfragen und unbekannte Modi/Masken/R → `BAD_ARG`.
- Die CRC16 (CCITT, Polynom 0x1021, Startwert 0, wie im VESC-Paketrahmen) über die Datensätze sichert zusätzlich zur Transport-CRC den Weg durch die Bridge.

## 8. Echtzeit

- **Hooks in der FOC-ISR:**
  1. `mcsa_capture_isr_angle(phase)` vor `mc_interface_mc_timer_isr()`, immer: Umdrehungszähler (eine Float-Multiplikation, Wandlung, 64-Bit-Addition).
  2. Nur wenn eine Aufnahme läuft: `mcsa_sample_in` füllen (Rohströme, Offsets, id/iq/vd/vq, CCR1–3, ARR, Motorzustand) und `mcsa_capture_isr_sample(&in)` aufrufen.
  3. Am Ende der ISR nach dem Setzen von `m_last_adc_isr_duration`: `mcsa_capture_isr_end(dauer)`. Erfasst das Maximum getrennt für Durchläufe mit und ohne Aufnahme (ohne Aufnahme als gleitendes Fenster über 30 000 Durchläufe).
- Je Durchlauf mit Aufnahme: 3 Integratorstufen je Kanal. Je Ausgabewert: 3 Kammstufen je Kanal, Normierung (Hardwaredivision für Ströme, 64-Bit-Division für Winkel und id/iq/vd/vq), Speichern.
- Abschätzung für die Standardmaske: unter 1 µs im Mittel, unter 2 µs bei Ausgabewerten. Budget je Durchlauf bei 30 kHz: 33 µs. Nachweis nur über die Messung am Gerät (`STATUS`), mit Standardmaske und voller Maske.
- Kein Gleitkomma-`lrintf` in der ISR (Wandlung per Typumwandlung mit eigener Rundung), keine Allokation, keine blockierenden Aufrufe, keine Interruptsperre in der ISR.

## 9. Host-Client `mcsa_client/mcsa_capture_client.py`

- Abhängigkeiten: `pyserial`, `numpy`. Framing und CRC wie `scripts/mcsa/vesc_sampleprint.py` (motorpi).
- Transport: USB direkt oder `--can ID`. Port exklusiv, immer nur eine offene Anfrage, Zeitüberschreitung mit Wiederholung (`START` mit demselben Token).
- Befehle: `info`, `status`, `capture` (Einzelaufnahme vollständig abholen, fehlende Blöcke nachfordern), `stream --seconds S` (Daueraufnahme, fortlaufend abholen, Lücken mit exakten Indizes protokollieren), `offset-check` (kurze Einzelaufnahme im Stillstand, Rest-Offset je Kanal in Stufen; Warnung ab 1,5 Stufen).
- Ausgabe: CSV mit Index, Zeit, Kanälen in physikalischen Einheiten (A, V), abgewickeltem elektrischem Winkel in Umdrehungen und Flags; daneben JSON mit Kopf (INFO, STATUS, Parameter, Lücken, Reboot-Erkennung).
- Hilfsfunktionen: |H(f)|, Gruppenlaufzeit, Zeitachse `t_n`, mechanischer Winkel aus elektrischem Winkel und Polpaaren.

## 10. Tests ohne Hardware

1. **C-Tests des Kerns** (`tests/mcsa_core/`, `make` mit Host-gcc, `-Wall -Wextra -Werror -Wdouble-promotion`):
   - Umdrehungszähler: vorwärts, rückwärts, Sprung über ±π, Stillstand.
   - CIC: DC-Verstärkung exakt 1 für R = 1, 2, 7, 10, 64; Amplitudengang bei 400 Hz gegen |H| (< 0,5 %); 11-kHz-Störer bei R = 10 ≤ −55 dB; Zweitonsignal (Träger + Seitenband) mit korrekter Dämpfung beider Linien.
   - Lange Aufnahme mit großem Winkel (Integrator-Überlauf in uint64) vorwärts und rückwärts: Winkel bleibt exakt.
   - Festkomma: Offset 2047,375, Eingang 2047 → −3; Offset 2047,4 (kein Achtelwert) → Abweichung ≤ 1 Einheit; negative Ausgaben korrekt gerundet.
   - Winkel zur Fenstermitte: phasenstarrer Sinusstrom zu linear laufendem Winkel, R = 20 → Phase aus gefiltertem Strom und gefiltertem Winkel weicht < 0,1° ab.
   - Einschwingen: bei R = 10 erster Datensatz nach genau 30 Durchläufen.
   - Offsetänderung während der Aufnahme → `DONE`/`OFFSETS_CHANGED`.
   - Ring: `ONESHOT` stoppt bei L; `CONTINUOUS` überschreibt; `READ` → `OVERWRITTEN` mit korrektem ältesten Index, auch bei Überschreiben während der Kopie (Test-Hook); Winkelanker nach Überlauf korrekt.
   - Protokoll: alle Befehle, zu kurze Anfragen, falsches Magic (nicht verarbeitet), falsche Version, unbekannter Befehl, `START` bei laufender Aufnahme, idempotenter `START`, Sample-Print belegt, `STOP` fremde Sitzung, `READ` fremde Sitzung, `READ` am Ende (Anzahl 0), CRC.
   - Flag-Kanal: hohe Modulation einer Phase setzt ihr Bit für 3·R Durchläufe.
2. **Python-Tests** (`pytest mcsa_client/`): CRC-Testvektor, Framing kurz/lang, Pack/Unpack, Winkel-Abwicklung mit Anker, |H|; Ende-zu-Ende gegen den als Bibliothek übersetzten Kern über einen simulierten Transport: Einzelaufnahme direkt und über CAN-Weiterleitung, Daueraufnahme mit erzwungenen Überläufen (Lücken exakt), verlorene `START`-Antwort (Wiederholung startet nicht neu), Reboot-Erkennung, Offset-Prüfung.
3. **Firmware-Build** `make fw_60` ohne neue Warnungen; Map-Prüfung (`.ram4`-Belegung, Abstand zur Grenze, Größe des Kernzustands).

## 11. Am Prüfstand (Nutzer, nach dem Flashen)

Reihenfolge und Prüfschritte stehen in `docs/mcsa_capture_flashing.md`. Kurz:

1. Bridge per USB flashen. `INFO`/`STATUS`, Stillstandsaufnahme, `COMM_SAMPLE_PRINT` aus VESC Tool funktioniert wie vorher, auch nach einer MCSA-Aufnahme.
2. Gleicher Motor, gleiche Drehzahl: Stock-`COMM_SAMPLE_PRINT` gegen neuen Modus. iq-Kanal gegen Telemetrie, 1000-Hz-Störlinie verschwunden, ISR-Dauer mit Standard- und voller Maske.
3. CAN-Knoten per `--canFwd` flashen, Einzelaufnahme über CAN lückenlos.
4. Daueraufnahme: erst Bridge per USB, dann ein CAN-Knoten mit R = 20.
5. Dritter Kanal an Controller 3 (Offsetfehler, Messbericht Abschnitt 12).

## 12. Abnahmekriterien

- `make fw_60` baut ohne neue Warnungen. Der Diff an `mc_interface.c` beschränkt sich auf die Puffer-Makros, die Sende-Zähler, `mc_interface_sample_print_busy()`, die Sperre in `mc_interface_sample_print_data()` und den Aufruf bei Konfigurationsänderung.
- Mit Maske ia/ib/ic/Winkel fasst der Puffer 5000 Datensätze; bei R = 10 sind das 1,67 s.
- Alle C- und Python-Tests laufen grün.
- ISR-Mehrlast ist abgeschätzt und über `STATUS` am Gerät messbar.
- `FW_TEST_VERSION_NUMBER` bleibt 0 (sonst warnt VESC Tool und markiert die Firmware als Testversion); die Kennung des Builds liefert `INFO` mit dem Git-Hash.

## 13. Review-Verlauf

- 2026-10-13, Codex (Design-Review): CIC-Überläufe modular definiert; Sample-Print-Sendethread und `SEND_LAST_SAMPLES` abgesichert; gemeinsamer Mutex auch für `READ`; Winkelanker je `READ`; Abbruch bei Offset-/Konfigurationsänderung; idempotenter `START`, Sitzung und Laufzeit in jeder Antwort; ISR-Dauer am ISR-Ende gemessen; Flag-Kanal für Rohkanal-Gültigkeit; FIR-Hinweis korrigiert; Indexgrenze; typisierte Union statt Zeiger.
- 2026-10-13, Umsetzung: Client liegt in `mcsa_client/` statt `tools/`, weil `tools/` im bldc-Repo der (ignorierte) Toolchain-Ordner von `make arm_sdk_install` ist.
