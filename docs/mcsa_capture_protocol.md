# MCSA-Aufnahmemodus — Protokoll

Firmware-Branch `feat/mcsa-capture-6.02` (Basis VESC 6.02, Build-Ziel `60`). Design und Begründungen: [`superpowers/specs/2026-10-13-mcsa-capture-design.md`](superpowers/specs/2026-10-13-mcsa-capture-design.md). Referenzclient: [`../mcsa_client/mcsa_capture_client.py`](../mcsa_client/mcsa_capture_client.py).

## Überblick

Die Firmware zeichnet die drei Shunt-Ströme, optional id/iq/vd/vq, den Rotorwinkel und einen Flag-Kanal auf. Grundlage ist der FOC-Interrupt (30 kHz bei `foc_f_zv` = 60 kHz und `foc_sample_v0_v7` = 0). Ein CIC-Filter 3. Ordnung dezimiert um R = 1…64. Die Werte landen in einem Ringpuffer von 40 000 Byte, den sich der Modus mit `COMM_SAMPLE_PRINT` teilt.

Es gibt zwei Betriebsarten:

- **Einzelaufnahme (`ONESHOT`):** füllt bis zu einer Länge und stoppt. Der Host holt die Daten danach in Ruhe ab.
- **Daueraufnahme (`CONTINUOUS`):** läuft bis `STOP`. Der Host holt fortlaufend ab. Was er nicht rechtzeitig holt, wird überschrieben und als Lücke mit exakten Indizes gemeldet.

`COMM_SAMPLE_PRINT`, Regelung, Konfiguration und Firmware-Version bleiben unverändert.

## Transport

- Paket-ID `COMM_CUSTOM_APP_DATA` = 36 (0x24), über USB direkt oder per `COMM_FORWARD_CAN` = 34 (0x22) an einen Controller hinter der Bridge.
- VESC-Rahmen wie üblich: `02 <len>` (bzw. `03 <len_hi> <len_lo>` ab 256 Byte), Nutzdaten, CRC16 (CCITT, Polynom 0x1021, Startwert 0, Testvektor `"123456789"` → 0x31C3), `03`.
- Pakete, deren Nutzdaten nicht mit dem Magic `4D 43` beginnen, gehen unverändert an LispBM und an Apps mit eigenem App-Data-Handler.
- **Eine offene Anfrage für alle Clients zusammen.** Die CAN-Rückleitung der Bridge ist global, also nicht an die einzelne Anfrage gebunden. Der Client öffnet den Port deshalb exklusiv. Parallel dürfen weder VESC Tool noch die motorpi-App mit derselben Bridge sprechen.
- Mehrbyte-Werte sind big-endian, Fließkommawerte IEEE-754 float32 big-endian.

Beispiele `INFO`:

```
direkt:        02 05 24 4D 43 01 01 B1 45 03
an CAN-ID 3:   02 07 22 03 24 4D 43 01 01 71 B0 03
```

## Anfrage

| Offset | Größe | Feld |
|---|---|---|
| 0 | 2 | Magic `4D 43` („MC“) |
| 2 | 1 | Protokollversion = 1 |
| 3 | 1 | Befehl |
| 4 | … | Argumente |

## Antwortkopf (11 Byte, in jeder Antwort)

| Offset | Größe | Feld |
|---|---|---|
| 0 | 2 | Magic `4D 43` |
| 2 | 1 | Protokollversion |
| 3 | 1 | Befehl \| 0x80 |
| 4 | 1 | Status |
| 5 | 2 | Sitzung (aktueller Sitzungszähler, u16) |
| 7 | 4 | Laufzeit des Controllers in ms (u32) |

Springt die Laufzeit zurück, hat der Controller neu gestartet; die Aufnahme ist dann verloren. Etwa alle 5 Tage läuft der Zähler über (32-Bit-Systemtick bei 10 kHz).

## Befehle

### `INFO` (0x01)

Keine Argumente. Antwortkörper (32 Byte):

| Offset | Typ | Feld |
|---|---|---|
| 0 | u8 | Protokollversion |
| 1 | u8 | CIC-Ordnung (3) |
| 2 | u8 | R_max (64) |
| 3 | u8 | Polpaare (`si_motor_poles` / 2) |
| 4 | u32 | Puffergröße in Byte (40 000) |
| 8 | f32 | f_ISR in Hz |
| 12 | f32 | `FAC_CURRENT` in A je ADC-Stufe |
| 16 | 16 B | Git-Hash des Builds, mit Nullen aufgefüllt |

### `START` (0x02)

| Offset | Typ | Argument |
|---|---|---|
| 0 | u8 | Modus: 1 = `ONESHOT`, 2 = `CONTINUOUS` |
| 1 | u16 | Kanalmaske (siehe unten) |
| 3 | u8 | Dezimierung R (1…64) |
| 4 | u32 | Länge in Datensätzen (`ONESHOT`; 0 = ganzer Puffer), sonst ignoriert |
| 8 | u16 | Token (Zufallszahl ≠ 0 des Clients) |

Beispiel `ONESHOT`, Maske 0x0087, R = 10, ganzer Puffer, Token 0x1234: `4D 43 01 02 01 00 87 0A 00 00 00 00 12 34`.

Antwortkörper bei `OK` (8 Byte): Kapazität in Datensätzen u32, Bytes je Datensatz u8, Maske u16, R u8. Bei Fehlern ist der Körper leer. Der Kopf trägt die neue Sitzung.

- **Wiederholung:** Ein `START` mit demselben Token ≠ 0 und denselben Parametern startet nicht neu, solange die Aufnahme läuft oder fertig ist. Er antwortet `OK` mit der bestehenden Sitzung. So schadet ein Wiederholen nach verlorener Antwort nicht.
- `BUSY`, wenn eine andere Aufnahme läuft oder `COMM_SAMPLE_PRINT` gerade aufnimmt bzw. sendet.
- `NOT_FOC`, wenn der Motor nicht im FOC-Modus konfiguriert ist.
- `BAD_ARG` bei ungültigem Modus, Maske 0 oder unbekannten Bits, R außerhalb 1…64 oder einer Länge über der Kapazität.

### `STOP` (0x03)

Argument: Sitzung u16. Antwortkörper (immer, 5 Byte): `n_written` u32, Stoppgrund u8. Bei fremder Sitzung oder ohne Aufnahme kommt `NOT_AVAILABLE`, die Aufnahme bleibt dann unberührt.

### `STATUS` (0x04)

Keine Argumente. Antwortkörper (64 Byte):

| Offset | Typ | Feld |
|---|---|---|
| 0 | u8 | Zustand |
| 1 | u8 | Stoppgrund |
| 2 | u8 | Modus |
| 3 | u16 | Kanalmaske |
| 5 | u8 | R |
| 6 | u8 | CIC-Ordnung |
| 7 | u8 | Polpaare |
| 8 | u32 | Kapazität (Datensätze) |
| 12 | u32 | Länge (`ONESHOT`), sonst 0 |
| 16 | u32 | `n_written` |
| 20 | u32 | ältester verfügbarer Index |
| 24 | f32 | f_ISR in Hz |
| 28 | f32 | Ampere je Einheit der Stromkanäle (= `FAC_CURRENT` / 8) |
| 32 | 3 × f32 | beim Start eingefrorene Offsets der drei Shunts in ADC-Stufen |
| 44 | i64 | θ_start, Umdrehungszähler beim Start (1/65536 el. Umdrehung) |
| 52 | f32 | maximale ISR-Dauer ohne Aufnahme (gleitendes Fenster über 30 000 Durchläufe) in µs |
| 56 | f32 | maximale ISR-Dauer während der Aufnahme in µs |
| 60 | u32 | Anzahl gesättigter Werte |

### `READ` (0x05)

| Offset | Typ | Argument |
|---|---|---|
| 0 | u16 | Sitzung |
| 2 | u32 | Startindex |
| 6 | u16 | gewünschte Anzahl (0xFFFF = so viele wie passen) |

Beispiel: `4D 43 01 05 00 01 00 00 00 00 FF FF`.

Antwortkörper bei `OK`:

| Offset | Typ | Feld |
|---|---|---|
| 0 | u32 | Startindex |
| 4 | u16 | Anzahl n |
| 6 | u8 | Bytes je Datensatz |
| 7 | i64 | Winkelanker: absoluter gefilterter Winkel des ersten Datensatzes in 1/8192 el. Umdrehung (0 ohne Winkelkanal) |
| 15 | n × Bytes | Datensätze |
| … | u16 | CRC16 über die Datensätze |

Bei `NOT_AVAILABLE` und `OVERWRITTEN` enthält der Körper den ältesten verfügbaren Index (u32) und `n_written` (u32).

- Höchstens ⌊(511 − 28) / Bytes je Datensatz⌋ Datensätze je Antwort, bei 8 Byte also 60.
- `start = n_written`: `OK` mit n = 0, es gibt noch nichts Neues.
- `start > n_written`, eine fremde Sitzung oder der Zustand `IDLE`/`INVALID`: `NOT_AVAILABLE`.
- `start` liegt vor dem ältesten verfügbaren Index oder wurde während des Kopierens überschrieben: `OVERWRITTEN`. Der Host trägt die Lücke `[start, ältester)` ein und liest beim ältesten weiter.

## Codes

| Status | | Zustand | | Stoppgrund | |
|---|---|---|---|---|---|
| 0 | `OK` | 0 | `IDLE` | 0 | keiner |
| 1 | `BUSY` | 1 | `RUNNING` | 1 | `LENGTH` (Einzelaufnahme voll) |
| 2 | `BAD_ARG` | 2 | `DONE` | 2 | `STOP_CMD` |
| 3 | `NOT_AVAILABLE` | 3 | `INVALID` | 3 | `OFFSETS_CHANGED` |
| 4 | `OVERWRITTEN` | | | 4 | `CONFIG_CHANGED` |
| 5 | `UNKNOWN_CMD` | | | 5 | `INDEX_LIMIT` |
| 6 | `BAD_VERSION` | | | 6 | `ANGLE_LIMIT` |
| 7 | `NOT_FOC` | | | 7 | `SAMPLE_PRINT` |
| | | | | 8 | `ANGLE_STEP` (4 oder mehr el. Umdrehungen zwischen zwei Datensätzen) |
| | | | | 9 | `ISR_OVERLOAD` (ein FOC-Durchlauf mit Aufnahme über 90 % seines Zyklus) |
| | | | | 10 | `FIFO_OVERRUN` (das Abholen kam nicht hinterher, siehe Echtzeit) |

## Kanäle

Die Datensätze enthalten die gewählten Kanäle in Bit-Reihenfolge, je 2 Byte, big-endian.

| Bit | Kanal | Typ | Einheit | Umrechnung |
|---|---|---|---|---|
| 0 | ia (Shunt 0, roh) | i16 | 1/8 ADC-Stufe, offsetkorrigiert | × `FAC_CURRENT`/8 → A |
| 1 | ib (Shunt 1, roh) | i16 | wie ia | |
| 2 | ic (Shunt 2, roh) | i16 | wie ia | |
| 3 | id | i16 | 1/8 ADC-Stufe | × `FAC_CURRENT`/8 → A |
| 4 | iq | i16 | 1/8 ADC-Stufe | |
| 5 | vd | i16 | mV | / 1000 → V |
| 6 | vq | i16 | mV | |
| 7 | Winkel | u16 | 1/8192 el. Umdrehung, modulo 8 Umdrehungen | siehe unten |
| 8 | Flags | u16 | Bits 0–2: Phase a/b/c hoch moduliert; Bit 3: Motor lief | |

Standardmaske 0x0087 (ia, ib, ic, Winkel) = 8 Byte je Datensatz, 5000 Datensätze im Puffer. Alle Kanäle: 18 Byte, 2222 Datensätze.

- **Rohkanäle:** Gespeichert wird der gemessene Wert jedes Shunts, bevor die FOC einen Kanal durch −(a+b) ersetzt. Ist das Flag-Bit einer Phase gesetzt (`CCR > ARR − 500` im Einflussbereich des Filters), war ihr Shunt wegen hoher Modulation womöglich nicht sauber messbar.
- **id/iq** sind bei ausgeschaltetem Motor null. **vd/vq** sind die totzeitkompensierten Spannungen aus `update_valpha_vbeta`.
- **Offsets:** Sie werden beim Start auf 1/8 Stufe gerundet eingefroren. Ändern sie sich während der Aufnahme (zum Beispiel durch `foc_dc_cal`), endet die Aufnahme mit `OFFSETS_CHANGED`.

## Winkel

- Die Firmware führt einen Umdrehungszähler θ_u (1/65536 elektrische Umdrehung, 64 Bit). Er läuft ab dem Boot in jedem FOC-Durchlauf weiter, auch ohne Aufnahme.
- Der gespeicherte Winkel ist der CIC-gefilterte Winkel, gehört also zur Fenstermitte des Filters wie die Ströme. Gespeichert werden seine Bits 3–18.
- Abwickeln je `READ`-Block ab dem Anker:

```python
a = angle_u16.astype(np.int64)
d = ((np.diff(a) + 32768) % 65536) - 32768
theta_q13 = anchor + np.concatenate(([0], np.cumsum(d)))   # 1/8192 el. Umdrehung
```

- Die Abwicklung setzt weniger als 4 elektrische Umdrehungen zwischen zwei Datensätzen voraus (f_e · R / f_ISR < 4, bei R = 64 bis 1875 Hz elektrisch). Andernfalls endet die Aufnahme mit `ANGLE_STEP`, statt falsche Winkel zu liefern.
- Mechanischer Winkel: θ_q13 / (8192 · Polpaare) Umdrehungen. Der Nullpunkt ist beliebig, bleibt aber innerhalb eines stabil laufenden Motorlaufs fest. Getrennte Aufnahmen und Blöcke hinter Lücken lassen sich so phasenrichtig verbinden. Über Beobachterverlust, Motorneustart oder Reboot hinweg ist das nicht garantiert.

## Zeitachse und Filter

- Datensatz n (ab 0) gehört zur Zeit t_n = ((n + 3) · R − 1 − 1,5 · (R − 1)) / f_ISR nach dem ersten ISR-Durchlauf der Aufnahme. Die ersten 2 Filterausgaben werden verworfen.
- Gruppenlaufzeit 3 · (R − 1) / 2 ISR-Durchläufe.
- Übertragungsfunktion |H(f)| = |sin(πRf/f_ISR) / (R · sin(πf/f_ISR))|³. Amplituden an der tatsächlichen Frequenz jeder Linie mit |H| korrigieren.

| R | Ausgaberate | Band mit ≥ 40 dB Aliasdämpfung | ≥ 30 dB | Störer 11 kHz | 5000 Werte |
|---|---|---|---|---|---|
| 1 | 30 kHz | ungefiltert | — | 0 dB | 0,17 s |
| 5 | 6 kHz | 1067 Hz | 1517 Hz | −58 dB | 0,83 s |
| 10 | 3 kHz | 551 Hz | 780 Hz | −61 dB | 1,67 s |
| 15 | 2 kHz | 370 Hz | 521 Hz | −68 dB | 2,50 s |
| 20 | 1,5 kHz | 278 Hz | 392 Hz | −79 dB | 3,33 s |
| 40 | 750 Hz | 139 Hz | 196 Hz | −98 dB | 6,67 s |

**Hinweise für die Auswertung:**
- **Park-Transformation und Filter sind nicht vertauschbar.** Für die Drehmomentwelligkeit den Kanal iq direkt aufnehmen oder die Phasenstromspektren vor der Park-Transformation mit |H| korrigieren.
- **Am Bandrand** ist das Nutzsignal um bis zu 1,5 dB gedämpft.

## Abläufe

**Einzelaufnahme:**
1. `INFO`.
2. `START` mit Modus 1.
3. `STATUS` abfragen, bis der Zustand nicht mehr `RUNNING` ist.
4. `READ` ab 0 in Blöcken bis `n_written`.

**Daueraufnahme:**
1. `START` mit Modus 2.
2. Fortlaufend `READ` ab dem nächsten Index. Immer nur eine Anfrage gleichzeitig. Bei n = 0 kurz warten, bei `OVERWRITTEN` die Lücke eintragen und beim ältesten Index weiterlesen.
3. `STOP` mit der Sitzung.
4. Den Rest bis zum gemeldeten `n_written` abholen.

**Robustheit im Client:**
- `READ`-Antworten mit falschem Startindex, fremder Sitzung oder unpassender Länge gelten als veraltet (zum Beispiel nach einer Zeitüberschreitung) und werden verworfen.
- Ändert sich bei `START` die Sitzung nicht, hat das Token zufällig einen früheren `START` wiederholt; der Client startet dann mit neuem Token.
- Springt die Laufzeit zurück, bricht die Aufnahme mit Reboot-Meldung ab.
- Bricht eine Daueraufnahme ab, sendet der Client noch `STOP`. Eine verwaiste Aufnahme beendet `mcsa_capture_client.py PORT [--can ID] stop`.

Durchsatz:
- **USB:** etwa 80 `READ` je Sekunde. Mit 8-Byte-Datensätzen (Standardmaske) sind das rund 4 700 Datensätze/s: Die Daueraufnahme ist ab R = 10 lückenlos, bei R = 5 entstehen Lücken. Mit nur einem Kanal (2 Byte) bleibt sie bis R = 2 lückenlos. Grenzen der Firmware selbst: Abschnitt Echtzeit.
- **CAN mit 500 kbit/s:** Die Statusmeldungen der Controller belegen schon etwa ein Viertel des Busses. Für eine Daueraufnahme über CAN mit 4 Kanälen deshalb R ≥ 20 wählen und immer nur einen Knoten zur Zeit aufnehmen lassen. Eine Einzelaufnahme mit beliebigem R ist über CAN in etwa 2–3 s abgeholt (40 KB Puffer, rund 85 Anfragen).

## Zusammenspiel mit `COMM_SAMPLE_PRINT`

Beide Funktionen nutzen denselben Speicher:

- **Während einer MCSA-Aufnahme** werden `COMM_SAMPLE_PRINT`-Anfragen verworfen.
- **`START`** wird mit `BUSY` abgelehnt, solange `COMM_SAMPLE_PRINT` aufnimmt oder sein Sende-Thread noch liest.
- **Eine neue `COMM_SAMPLE_PRINT`-Aufnahme** übernimmt den Speicher. Eine fertige MCSA-Aufnahme wird dadurch `INVALID` (Stoppgrund `SAMPLE_PRINT`). Die erste solche Übernahme löscht den Speicher, damit nie MCSA-Bytes als Sample-Print gesendet werden.
- **`SEND_LAST_SAMPLES`** wird nach einer MCSA-Aufnahme verworfen, bis `COMM_SAMPLE_PRINT` wieder selbst aufgenommen hat.
- **Ohne MCSA-Nutzung** verhält sich `COMM_SAMPLE_PRINT` wie in Stock 6.02.

Eine Konfigurationsänderung (`mc_interface_set_configuration`, zum Beispiel durch `vesc_tool --setMcConf`) beendet eine laufende Aufnahme mit `CONFIG_CHANGED`. Geprüft wird am Anfang und am Ende der Änderung.

## Echtzeit

**Aufteilung:**
- **FOC-Interrupt:** führt in jedem Durchlauf den Umdrehungszähler (etwa 90 Zyklen, auch ohne Aufnahme). Während einer Aufnahme rechnet er nur die Integratoren des CIC-Filters. Alle R Durchläufe legt er deren letzte Stufe in einen FIFO (1 792 Byte im CCM; je nach Maske 32 bis 448 Einträge).
- **Abhol-Thread** (`MCSA drain`, gleiche Priorität wie der Watchdog-Thread): wacht alle 0,5 ms auf und rechnet Kämme, Normierung und Datensätze. Er arbeitet in Stapeln zu 32 Einträgen und gibt dazwischen die CPU ab. `STATUS`, `READ` und `STOP` holen vorher ebenfalls ab.

**Schutz:**
- **`ISR_OVERLOAD`:** Ein FOC-Durchlauf mit Aufnahme über 90 % seines Zyklus (30 µs bei 30 kHz) beendet die Aufnahme.
- **`FIFO_OVERRUN`:** Kommt das Abholen nicht hinterher, endet die Aufnahme. Die Datensätze bis dahin bleiben lesbar. Zu viel Last führt so nie zu einem Watchdog-Neustart.

**Gemessen an der Bridge im Stillstand** (FW `9dd573db`, 14.10.2026):

| | Standardmaske 0x0087 | alle Kanäle 0x01FF | ein Kanal 0x0001 |
|---|---|---|---|
| Interrupt-Anteil je Durchlauf ohne / mit Ausgabewert | 1,65 / 2,6 µs | 4,5 / 5,6 µs | – / 2,0 µs (R = 1) |
| FOC-Durchlauf max. ohne / mit Aufnahme | 17,4 / 20,9 µs | 17,4 / 23,9 µs | 17,4 / 20,4–21,3 µs |
| Abholen je Eintrag, rein (Interrupts gesperrt) | ≈ 1 140 Zyklen | ≈ 2 500 Zyklen | – |
| Abholen je Eintrag im Betrieb, ohne FOC-Interrupt | ≈ 1 900–2 800 Zyklen | ≈ 8 000–11 000 Zyklen | ≈ 1 100–1 500 Zyklen |
| Einzelaufnahme vollständig ab | R = 2 | R = 10 | R = 1 |
| Daueraufnahme über USB lückenlos ab | R = 10 | R = 20 | R = 2 |

Darunter endet die Aufnahme mit `FIFO_OVERRUN` (bzw. bei der Daueraufnahme mit Standardmaske und R = 5 mit Lücken durch USB). Der FOC-Interrupt belegt im Stillstand 56 % der CPU, mit Aufnahme 59–62 %. Unter Last steigt er, die Grenzen verschieben sich dann zu größerem R.

**Messen:**
- `STATUS` meldet die ISR-Maxima mit und ohne Aufnahme.
- Der Terminalbefehl `mcsa_perf` (VESC Tool oder `term_cmd.py PORT -1 mcsa_perf` aus motorpi `scripts/mcsa`) zeigt seit dem letzten Aufruf: Zyklen des Interrupt-Anteils, Kosten und Pausen des Abhol-Threads, höchsten FIFO-Füllstand und die Last des FOC-Interrupts (bei Abständen unter 20 s). Danach setzt er die Werte zurück.
