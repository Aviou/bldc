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
- FIR-Nachfilter (sh. 5.4, mögliche Erweiterung).

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
  mcsa_capture_isr()            <- neu, direkt vor mc_interface_mc_timer_isr()
      Umdrehungszähler (immer)
      CIC-Filter + Ringpuffer (nur wenn Aufnahme läuft)

commands_process_packet()
  case COMM_CUSTOM_APP_DATA:
      if mcsa_capture_process_cmd(data, len, reply_func)  <- neu, prüft Magic
          break
      ... bisheriger Weg (appdata_func, LispBM) unverändert ...

mc_interface.c
  COMM_SAMPLE_PRINT-Arrays zeigen in den gemeinsamen Puffer (Layout unverändert)
```

### Dateien

| Datei | Inhalt |
|---|---|
| `motor/mcsa_core.c/.h` | Hardwareunabhängiger Kern: Umdrehungszähler, CIC-Filter, Ringpuffer, Befehlsauswertung, Serialisierung. Keine ChibiOS-/HAL-Abhängigkeit, auf dem PC übersetzbar. |
| `motor/mcsa_capture.c/.h` | Einbindung in die Firmware: gemeinsamer Puffer in `.ram4`, Puffer-Eigentum gegenüber `COMM_SAMPLE_PRINT`, Lesen der FOC-Größen, Antwort über `reply_func`, kritische Abschnitte. |
| `motor/mcpwm_foc.c` | ein Aufruf `mcsa_capture_isr(...)` vor `mc_interface_mc_timer_isr()`; übergibt Rohströme, Offsets, Winkel, id/iq/vd/vq als Werte (der Motorzustand ist in `mcpwm_foc.c` statisch). Nur für Motor 1. |
| `comm/commands.c` | Abzweig in `case COMM_CUSTOM_APP_DATA` |
| `motor/mc_interface.c` | Sample-Print-Arrays als Zeiger in den gemeinsamen Puffer, Eigentumsprüfung beim Start von `COMM_SAMPLE_PRINT` |
| `motor/motor.mk` | neue Quelldateien |
| `tests/mcsa_core/` | PC-Tests des Kerns (C, ohne externe Bibliotheken) und gemeinsame Bibliothek für die Python-Tests |
| `tools/mcsa_capture_client.py` | Referenzclient (pyserial, numpy) |
| `tools/test_mcsa_capture_client.py` | Tests des Clients, u. a. gegen den Kern per ctypes |
| `docs/mcsa_capture_protocol.md` | Protokoll |
| `docs/mcsa_capture_flashing.md` | Flash- und Rückweg-Anleitung |

## 5. Signalpfad

### 5.1 Eingänge je ISR-Durchlauf

Erfasst am Ende der FOC-ISR, also im selben Durchlauf, in dem Ströme, Offsets und Winkel berechnet wurden:

| Größe | Quelle | Bemerkung |
|---|---|---|
| Rohströme a0, a1, a2 | `m_currents_adc[0..2]` (ADC-Zählwerte) | **vor** Offsetabzug und **vor** der Ersetzung eines Kanals durch −(a+b) |
| Offsets o0, o1, o2 | `m_conf->foc_offsets_current[0..2]` | je Durchlauf gelesen; Änderung während einer Aufnahme setzt ein Flag |
| Winkel | `m_motor_state.phase` (rad) | derselbe Winkel, den die FOC und `COMM_SAMPLE_PRINT` verwenden |
| id, iq, vd, vq | `m_motor_state` | optional per Kanalmaske |

Hinweis zu den Rohströmen: Bei hoher Modulation ist der Shunt der Phase mit der längsten Einschaltdauer nicht sauber messbar; die FOC rekonstruiert diese Phase deshalb. Die Aufnahme liefert trotzdem den gemessenen Wert, weil genau er für die Fehlersuche im Messpfad gebraucht wird. Die Auswertung muss das wissen.

### 5.2 Umdrehungszähler

Läuft in jedem ISR-Durchlauf, auch ohne Aufnahme und im Stillstand.

- `frac16 = (uint16_t)(int32_t)lrintf(phase / (2π) · 65536)`; Winkel in 1/65536 elektrischer Umdrehung.
- `θ_u += (int16_t)(frac16 − frac16_last)`; `θ_u` ist `int64_t`, nie zurückgesetzt außer beim Booten.
- Annahme: weniger als eine halbe elektrische Umdrehung pro ISR-Durchlauf (bei 30 kHz bis 15 kHz elektrisch erfüllt).
- Mechanischer Winkel bis auf einen beliebigen, im Lauf festen Nullpunkt: `θ_m = θ_u / (65536 · Polpaare)`.

Grenzen: Der Winkel ist bei sensorlosem Betrieb eine Schätzung des Beobachters. Über Beobachterverlust, Neustart des Motors oder Reboot hinweg ist die mechanische Zuordnung nicht garantiert. Innerhalb eines stabil laufenden Motorlaufs bleibt sie erhalten.

### 5.3 CIC-Dezimationsfilter

- Ordnung 3, Dezimierung R ∈ [1, 64], Differenzverzögerung 1.
- Ströme ia/ib/ic: Eingang `x = a_k − round(o_k)` (ganze ADC-Stufen, vorzeichenbehaftet, |x| < 2^12), Integratoren und Kämme in `int32_t` mit Zweierkomplement-Überlauf (Wortbreite 12 + 1 + 3·log2(64) = 31 Bit ≤ 32). Ausgabe `y / R³` mit Rundung in Einheiten von 1/8 ADC-Stufe, anschließend Abzug des Nachkommaanteils des Offsets (ebenfalls in 1/8 Stufe). Speicherung `int16_t` mit Sättigung (Bereich ±4096 Stufen ≈ ±330 A).
- id/iq: Eingang in 1/8 ADC-Stufe (`I / FAC_CURRENT · 8`, gerundet), vd/vq: Eingang in mV. Integratoren `int64_t`, Ausgabe `y / R³` gerundet, `int16_t` mit Sättigung.
- Winkel: Eingang `θ_u − θ_start` (`int64_t`, θ_start = θ_u beim Start). Damit durchläuft der Winkel **dasselbe Filter** wie die Ströme; für eine gleichmäßig laufende Drehung ist der gefilterte Winkel genau der Winkel zur Gruppenlaufzeit des Filters, also passend zum gefilterten Strom, auch bei halbzahliger Verzögerung. Gespeichert: `uint16_t` = Bits 3–18 von `θ_start + y/R³`, also 1/8192 elektrische Umdrehung, modulo 8 Umdrehungen. Der Host wickelt Differenzen modulo 2^16 ab; das ist eindeutig, solange zwischen zwei Ausgabewerten weniger als 4 elektrische Umdrehungen liegen (bei R = 64 und 30 kHz bis 1875 Hz elektrisch).
- Ausgabe alle R ISR-Durchläufe. Der erste Ausgabewert wird erst geschrieben, wenn das Filter vollständig gefüllt ist (3·R Eingangswerte), damit keine Einschwingwerte im Puffer landen.
- Gruppenlaufzeit 3(R−1)/2 Eingangsabtastungen. Übertragungsfunktion bekannt: |H(f)| = |sin(πRf/f_ISR) / (R·sin(πf/f_ISR))|³; der Client korrigiert die Dämpfung im Nutzband.

Nutzbares Band (Rückfaltung aus allen Bändern ≥ 40 dB bzw. ≥ 30 dB gedämpft), f_ISR = 30 kHz:

| R | Ausgaberate | Band ≥ 40 dB | Band ≥ 30 dB | Störer 11 kHz | 5000 Werte |
|---|---|---|---|---|---|
| 1 | 30 kHz | — (ungefiltert) | — | 0 dB | 0,17 s |
| 5 | 6 kHz | 1067 Hz | 1517 Hz | −58 dB | 0,83 s |
| 10 | 3 kHz | 551 Hz | 780 Hz | −61 dB | 1,67 s |
| 15 | 2 kHz | 370 Hz | 521 Hz | −68 dB | 2,50 s |
| 20 | 1,5 kHz | 278 Hz | 392 Hz | −79 dB | 3,33 s |
| 40 | 750 Hz | 139 Hz | 196 Hz | −98 dB | 6,67 s |

Zum Vergleich: Auslassen (heute) dämpft nichts, einfache Mittelung bei R = 10 den 11-kHz-Störer um 20 dB.

### 5.4 Mögliche Erweiterung (nicht in dieser Version)

Ein FIR-Halbbandfilter nach dem CIC würde das nutzbare Band bei gleicher Ausgaberate auf etwa 0,4 × Ausgaberate verbreitern, vor allem für die Daueraufnahme über CAN. Erst angehen, wenn die Messungen zeigen, dass das Band bei R = 20 nicht reicht.

## 6. Puffer und Betriebsarten

### 6.1 Gemeinsamer Puffer

- `mcsa_capture.c` stellt einen Block von 40 000 B in `.ram4` bereit (36 000 B der bisherigen Sample-Print-Arrays plus 4 000 B aus dem freien CCM).
- Die zehn Sample-Print-Arrays in `mc_interface.c` werden zu Zeigern auf feste Abschnitte dieses Blocks. Größen, Reihenfolge und Zugriffe bleiben gleich.
- Eigentum: `FREE`, `SAMPLE_PRINT` oder `MCSA`, gewechselt unter `chSysLock`.
  - `COMM_SAMPLE_PRINT` startet nur, wenn keine MCSA-Aufnahme läuft. Läuft eine, wird die Anfrage verworfen (keine Daten). Ist eine MCSA-Aufnahme beendet, darf `COMM_SAMPLE_PRINT` den Puffer übernehmen; die MCSA-Daten gelten danach als ungültig (`READ` meldet `NOT_AVAILABLE`).
  - `START` wird mit `BUSY` abgelehnt, solange `COMM_SAMPLE_PRINT` aufnimmt oder sendet.
- Bei inaktiver MCSA-Aufnahme verhält sich `COMM_SAMPLE_PRINT` exakt wie in Stock 6.02.

### 6.2 Datensätze

- Kanäle in fester Reihenfolge, gewählt per Bitmaske:

| Bit | Kanal | Typ | Einheit |
|---|---|---|---|
| 0 | ia (Shunt 0) | int16 | 1/8 ADC-Stufe |
| 1 | ib (Shunt 1) | int16 | 1/8 ADC-Stufe |
| 2 | ic (Shunt 2) | int16 | 1/8 ADC-Stufe |
| 3 | id | int16 | 1/8 ADC-Stufe |
| 4 | iq | int16 | 1/8 ADC-Stufe |
| 5 | vd | int16 | mV |
| 6 | vq | int16 | mV |
| 7 | Winkel | uint16 | 1/8192 el. Umdrehung, mod 8 |

- Standardmaske `0x87` (ia, ib, ic, Winkel) = 8 B je Datensatz → 5000 Datensätze im Puffer.
- Kapazität = ⌊40 000 / (2 · Anzahl Kanäle)⌋ Datensätze.

### 6.3 Index und Ring

- Jeder Datensatz hat einen absoluten Index n (uint32) ab Aufnahmestart. Speicherplatz `n mod Kapazität`.
- Die ISR schreibt erst den kompletten Datensatz und erhöht danach `n_written`. Ein Datensatz mit Index < `n_written` ist daher immer vollständig.
- Ältester verfügbarer Index: `max(0, n_written − Kapazität)`.

### 6.4 Betriebsarten

| Modus | Verhalten |
|---|---|
| `ONESHOT` | Aufnahme bis Länge L (1 ≤ L ≤ Kapazität, 0 = Kapazität), danach Zustand `DONE`. Daten bleiben bis zum nächsten `START` oder bis `COMM_SAMPLE_PRINT` den Puffer übernimmt. |
| `CONTINUOUS` | Aufnahme bis `STOP`. Der Host holt fortlaufend ab. Was er nicht rechtzeitig holt, wird überschrieben und beim Lesen als `OVERWRITTEN` mit dem ältesten verfügbaren Index gemeldet. |

Start immer „sofort“. Eine Sitzungsnummer (uint16, je `START` +1) steht in jeder Antwort; der Host erkennt so Neustarts.

### 6.5 Konsistentes Lesen

`READ` läuft im Thread, der das Paket verarbeitet. Ablauf: Bereich prüfen, Datensätze kopieren, danach `n_written` erneut lesen. Gilt `n_written > start + Kapazität`, wurde während der Kopie überschrieben, und die Antwort ist `OVERWRITTEN`. Kein Sperren der Interrupts während der Kopie.

## 7. Protokoll

Transport: `COMM_CUSTOM_APP_DATA` (über USB direkt oder per `COMM_FORWARD_CAN`). Antworten gehen über die `reply_func` des eingehenden Pakets, also auch bei CAN-Weiterleitung zurück an den Anfragenden. Details, Byte-Layout und Beispiele in `docs/mcsa_capture_protocol.md`; hier die Festlegungen:

- Kopf jeder Anfrage: `[COMM_CUSTOM_APP_DATA][0x4D 0x43 ('MC')][Protokollversion 1][Befehl]`.
- Kopf jeder Antwort: `[COMM_CUSTOM_APP_DATA][0x4D 0x43][1][Befehl | 0x80][Status]`.
- Pakete mit anderem Magic gehen unverändert an `appdata_func` und LispBM. Pakete mit unserem Magic verlassen das Modul nie.
- Mehrbyte-Werte big-endian wie im übrigen VESC-Protokoll; Fließkomma als IEEE-754 float32 big-endian.

| Befehl | Code | Anfrage | Antwort (nach Status) |
|---|---|---|---|
| `INFO` | 0x01 | — | Protokollversion, Git-Hash (Text, 0-terminiert), Pufferbytes, f_ISR, R_max, Polpaare, `FAC_CURRENT` |
| `START` | 0x02 | Modus, Maske, R, Länge (u32) | Sitzung, Kapazität (Datensätze), Bytes je Datensatz |
| `STOP` | 0x03 | — | Sitzung, `n_written` |
| `STATUS` | 0x04 | — | Zustand, Sitzung, Modus, Maske, R, Kapazität, `n_written`, ältester Index, f_ISR, A je Einheit (= `FAC_CURRENT`/8), Offsets beim Start (3 × float, ADC-Stufen), Flag „Offsets geändert“, `θ_start` (int64), Polpaare, max. ISR-Dauer während der Aufnahme (µs), ISR-Dauer vor Start (µs) |
| `READ` | 0x05 | Sitzung, Startindex (u32), Anzahl (u16) | Sitzung, Startindex, Anzahl, Bytes je Datensatz, Datensätze, CRC16 über die Datensätze |

Befehle werden unter einem Mutex verarbeitet (USB- und CAN-Thread können gleichzeitig Pakete liefern). `START` richtet den Zustand vollständig ein und schaltet die ISR zuletzt frei.

Statuscodes: 0 `OK`, 1 `BUSY`, 2 `BAD_ARG`, 3 `NOT_AVAILABLE`, 4 `OVERWRITTEN`, 5 `UNKNOWN_CMD`, 6 `BAD_VERSION`. Bei Fehlern von `READ` folgen ältester Index und `n_written`, damit der Host weiß, wo er weitermachen kann.

`READ` liefert höchstens so viele Datensätze, wie in ein Paket passen: ⌊(`PACKET_MAX_PL_LEN` − 17) / Bytes je Datensatz⌋, bei 8 B also 61. Die Antwort nennt die tatsächliche Anzahl. Das Transportprotokoll sichert zusätzlich mit CRC16 auf jedem Abschnitt (USB-Rahmen, CAN-Reassemblierung); die CRC16 über die Datensätze sichert den Weg dazwischen ab.

## 8. Echtzeit

- Ohne Aufnahme: Umdrehungszähler (eine Float-Multiplikation, eine Rundung, eine 64-Bit-Addition) und eine Zustandsabfrage.
- Mit Aufnahme je Durchlauf: 3 Integratorstufen je Kanal (int32 bzw. int64). Je Ausgabewert: 3 Kammstufen je Kanal, Division durch R³ (int32-Hardwaredivision für ia/ib/ic; 64-Bit-Division für Winkel und id/iq/vd/vq), Speichern.
- Abschätzung für Standardmaske: < 1 µs im Mittel, < 2 µs bei Ausgabewerten. Budget je Durchlauf bei 30 kHz: 33 µs.
- Messung am Gerät: `STATUS` meldet die ISR-Dauer (`mcpwm_foc_get_last_adc_isr_duration()`) vor dem Start und das Maximum während der Aufnahme.
- Keine Allokation, keine blockierenden Aufrufe, keine Interruptsperre in der ISR. Antworten werden im Paket-Thread erzeugt.

## 9. Host-Client `tools/mcsa_capture_client.py`

- Abhängigkeiten: `pyserial`, `numpy`. Framing und CRC wie `scripts/mcsa/vesc_sampleprint.py` (motorpi).
- Transport: USB direkt oder `--can ID` (Weiterleitung über die Bridge). Immer nur eine offene Anfrage.
- Befehle: `info`, `status`, `capture` (Einzelaufnahme, vollständig abholen, fehlende Blöcke nachfordern), `stream --seconds S` (Daueraufnahme, fortlaufend abholen, Lücken mit exakten Indizes protokollieren), `offset-check` (kurze Einzelaufnahme im Stillstand, Rest-Offset je Kanal in Stufen; Warnung ab 1,5 Stufen).
- Ausgabe: CSV mit Index, Kanälen in physikalischen Einheiten (A, V) und abgewickeltem elektrischem Winkel in Umdrehungen; daneben JSON mit Kopf (Status, Konfiguration, Lücken, Zeitpunkte).
- Hilfsfunktionen für die Auswertung: CIC-Dämpfungskorrektur |H(f)|, Gruppenlaufzeit, mechanischer Winkel aus θ_start und Polpaaren.

## 10. Tests ohne Hardware

1. **C-Tests des Kerns** (`tests/mcsa_core/`, `make` mit Host-gcc):
   - CIC: Impuls- und Sprungantwort, DC-Verstärkung exakt 1, Dämpfung bei 400 Hz/11 kHz gegen die Formel.
   - Nachkommabits: konstanter Eingang mit Offset 2047,375 → Ausgabe exakt −3 (1/8 Stufen) statt 0.
   - Winkel: linear laufender Winkel und dazu phasenstarrer Sinusstrom → nach dem Filter stimmt die Phase von Strom zu Winkel (Abweichung < 0,1°), auch bei R = 20.
   - Umdrehungszähler: Rückwärts-/Vorwärtslauf, Überlauf des 16-Bit-Anteils.
   - Ring: Einzelaufnahme stoppt bei L, Daueraufnahme überschreibt, `READ` meldet `OVERWRITTEN` mit korrektem ältesten Index, auch wenn das Überschreiben während der Kopie passiert (simuliert über einen Hook im Test).
   - Protokoll: alle Befehle, falsche Längen, falsches Magic (wird nicht verarbeitet), falsche Version, unbekannter Befehl.
2. **Python-Tests** (`pytest`): Framing, CRC, Pack/Unpack, Winkel-Abwicklung, Lückenbuchhaltung; Ende-zu-Ende gegen den als Bibliothek übersetzten Kern über einen simulierten Transport (Einzel- und Daueraufnahme mit erzwungenen Überläufen).
3. **Firmware-Build** `make fw_60` ohne neue Warnungen; RAM-Bericht aus der Map (CCM-Belegung, Abstand zur Grenze).

## 11. Am Prüfstand (Nutzer, nach dem Flashen)

Reihenfolge und Prüfschritte stehen in `docs/mcsa_capture_flashing.md`. Kurz:

1. Bridge per USB flashen, `INFO`/`STATUS`, Stillstands-Aufnahme, `COMM_SAMPLE_PRINT` aus VESC Tool funktioniert wie vorher.
2. Gleicher Motor, gleiche Drehzahl: Stock-`COMM_SAMPLE_PRINT` gegen neuen Modus. Iq aus den Rohdaten gegen Telemetrie, 1000-Hz-Störlinie verschwunden, ISR-Dauer.
3. CAN-Knoten per `--canFwd` flashen, Einzelaufnahme über CAN lückenlos.
4. Daueraufnahme: erst Bridge per USB, dann ein CAN-Knoten mit R = 20.
5. Dritter Kanal an Controller 3 (Offsetfehler, Messbericht Abschnitt 12).

## 12. Abnahmekriterien

- `make fw_60` baut ohne neue Warnungen; `COMM_SAMPLE_PRINT` ist bei inaktivem MCSA-Modus unverändert (Code-Diff auf Zeigerumstellung und Eigentumsprüfung beschränkt).
- Mit Maske ia/ib/ic/Winkel fasst der Puffer 5000 Datensätze; bei R = 10 sind das 1,67 s.
- Der Client holt eine Einzelaufnahme über einen simulierten CAN-Weg vollständig ab; Index, CRC, Nachfordern und Überlaufmeldung sind getestet.
- ISR-Mehrlast ist abgeschätzt und über `STATUS` am Gerät messbar.
- Versionskennung: `FW_TEST_VERSION_NUMBER` bleibt 0 (sonst warnt VESC Tool und markiert die Firmware als Testversion); die Kennung des Builds liefert `INFO` mit dem Git-Hash.
