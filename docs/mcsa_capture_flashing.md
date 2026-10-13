# MCSA-Firmware flashen, prüfen und zurückspielen

Für die vier Flipsky FSESC 6.7 Pro am Prüfstand (HW „60“, FW 6.02): eine Bridge per USB, drei Knoten per CAN. Protokoll: [`mcsa_capture_protocol.md`](mcsa_capture_protocol.md).

**Erst nur die Bridge flashen und prüfen.** Die CAN-Knoten folgen erst, wenn an der Bridge alles stimmt.

## 0. Voraussetzungen

- Auf dem Pi läuft kein Test, und die App greift nicht auf den VESC zu:
  `curl -s localhost:8000/api/v1/tests/status/` zeigt `test_running` und `busy` auf `false`.
- Der Port ist frei: `sudo fuser /dev/ttyACM0` liefert nichts. Notfalls die App für die Dauer anhalten.
- Alle Motoren stehen, das Netzteil ist an und bleibt während des Hochladens an.
- `vesc_tool` auf dem Pi kann Firmware hochladen: `vesc_tool --help | grep -E 'uploadFirmware|canFwd'` zeigt beide Optionen. Fehlen sie, mit VESC Tool 6.02 am Laptop per USB arbeiten (Firmware → Custom File).
- Welche Firmware läuft heute? `vesc_tool --offscreen --vescPort /dev/ttyACM0 --queryDeviceFwParams`. Lief bisher ein eigener Build mit geänderten Standardwerten (z. B. aus `~/Documents/4Droin/bldc` mit angepassten `appconf_default.h`/`mcconf_default.h`), ist das der Rückweg. Dieser Branch nutzt die Stock-Standardwerte; sie wirken nur bei „Standardwerte laden“, die gespeicherte Konfiguration bleibt beim Flashen erhalten.

## 1. Bauen

Auf dem Rechner mit dem Repo:

```bash
make fw_60
```

Ergebnis: `build/60/60.bin`. Den Git-Hash notieren (`git rev-parse --short HEAD`); `INFO` meldet ihn später zur Kontrolle.

Firmware und Client auf den Pi kopieren (der Pi hat keinen GitHub-Zugang):

```bash
scp build/60/60.bin mcsa_client/mcsa_capture_client.py motorpi:/home/pi/mcsa/
```

## 2. Konfiguration sichern

Für jeden Controller Motor- und App-Konfiguration ablegen; die Bridge ohne `--canFwd`, die Knoten mit ihrer CAN-ID:

```bash
cd /home/pi/mcsa
vesc_tool --offscreen --vescPort /dev/ttyACM0 --getMcConf mc_bridge.xml
vesc_tool --offscreen --vescPort /dev/ttyACM0 --getAppConf app_bridge.xml
vesc_tool --offscreen --vescPort /dev/ttyACM0 --canFwd 2 --getMcConf mc_2.xml
vesc_tool --offscreen --vescPort /dev/ttyACM0 --canFwd 2 --getAppConf app_2.xml
```

Dasselbe für CAN-ID 3 und 4.

## 3. Bridge flashen

```bash
vesc_tool --offscreen --vescPort /dev/ttyACM0 --uploadFirmware /home/pi/mcsa/60.bin
```

Der Controller startet danach neu; einige Sekunden warten, bis `/dev/ttyACM0` wieder da ist.

## 4. Bridge prüfen

Den Client wie die motorpi-Skripte in einem Wegwerf-Container starten, ohne Netzwerk, nur mit dem USB-Port:

```bash
docker run --rm --network none --device /dev/ttyACM0 -v /home/pi/mcsa:/mcsa -v /tmp/mcsa_out:/out --entrypoint python motor-web /mcsa/mcsa_capture_client.py /dev/ttyACM0 info
```

Im Folgenden steht `CLIENT` für alles bis einschließlich `mcsa_capture_client.py`.

1. **`INFO`** meldet den notierten Git-Hash, 40 000 Byte Puffer, f_ISR 30 000 Hz und 7 Polpaare.
2. **VESC Tool und App sehen keinen Unterschied:**
   - `vesc_tool --offscreen --vescPort /dev/ttyACM0 --queryDeviceFwParams` meldet weiter 6.02, HW „60“ und keine Testversion.
   - `--getMcConf` erneut auslesen und mit `mc_bridge.xml` vergleichen. Erwartet: identisch.
3. **`COMM_SAMPLE_PRINT` wie vorher:** `vesc_sampleprint.py /dev/ttyACM0 2000 10 0 1` aus `scripts/mcsa` (motorpi) liefert 2000 Pakete.
4. **Offset-Prüfung im Stillstand:** `CLIENT /dev/ttyACM0 offset-check`. Erwartet: Rest-Offset je Kanal unter 1,5 Stufen; Exit-Code 1 bedeutet Warnung.
5. **Einzelaufnahme im Stillstand:** `CLIENT /dev/ttyACM0 capture --decimation 10 --out /out/still_bridge`. Danach `status` ansehen: ISR-Maximum ohne und mit Aufnahme.
6. **`COMM_SAMPLE_PRINT` nach einer MCSA-Aufnahme:** Schritt 3 wiederholen. Erwartet: funktioniert, denn eine neue Aufnahme übernimmt den Speicher.
7. **Lauf mit Propeller** (12 000 ERPM, wie im Messbericht): Dafür muss ein Prozess den Motor drehen und zugleich aufnehmen. Der Client und `spin_capture.py` (motorpi) öffnen den Port jeweils exklusiv, und der Motor braucht alle 100 ms ein `COMM_SET_RPM` als Heartbeat, auch während eine Aufnahme läuft. **Das kombinierte Skript ist der nächste Schritt:** `spin_capture.py` ruft `McsaClient` über dieselbe serielle Verbindung auf und sendet zwischen den Anfragen weiter den Heartbeat. Bis dahin lässt sich nur im Stillstand messen.

   Danach aufnehmen:
   - mit Standardmaske und R = 10
   - mit Maske 0x01FF (alle Kanäle, ISR-Last prüfen)
   - als Daueraufnahme über 20 s

   Kontrolle:
   - Der Kanal iq stimmt mit der Telemetrie überein.
   - Die 1000-Hz-Störlinie aus dem Messbericht ist verschwunden.
   - `status` meldet ein ISR-Maximum mit Aufnahme deutlich unter 33 µs.
   - Die JSON-Datei der Daueraufnahme zeigt keine Lücken.

## 5. CAN-Knoten flashen

Einzeln, nacheinander, jeweils mit Kontrolle:

```bash
vesc_tool --offscreen --vescPort /dev/ttyACM0 --canFwd 2 --uploadFirmware /home/pi/mcsa/60.bin
CLIENT /dev/ttyACM0 --can 2 info
```

Danach CAN-ID 3 und 4. Mit `bus_scan.py` (motorpi) prüfen, dass alle vier Controller wieder antworten.

Prüfungen über CAN:
- **Einzelaufnahme im Stillstand:** `CLIENT /dev/ttyACM0 --can 3 capture --decimation 10 --out /out/m3_r10` wird lückenlos abgeholt, etwa 2–3 s.
- **Daueraufnahme:** `CLIENT /dev/ttyACM0 --can 3 stream --decimation 20 --seconds 20 --out /out/m3_stream`, immer nur ein Knoten zur Zeit (im Stillstand sofort möglich, unter Last mit dem kombinierten Skript). Bei Lücken in der JSON-Datei R erhöhen oder Kanäle weglassen.
- **Dritter Kanal an Controller 3** (Offsetfehler, Messbericht Abschnitt 12): Einzelaufnahme mit `--mask 0x0187` im Stillstand (Rest-Offset je Kanal) und später unter Last, ic gegen ia und ib vergleichen.

## 6. Rückweg

1. Firmware bauen:
   - Stock 6.02 aus dem Haupt-Clone:
     ```bash
     git -C ~/Documents/4Droin/bldc worktree add ~/Documents/4Droin/bldc-stock-602 origin/release_6_02
     ```
     Den Toolchain-Ordner verlinken (`ln -s ~/Documents/4Droin/bldc/tools ~/Documents/4Droin/bldc-stock-602/tools`), dann `make -C ~/Documents/4Droin/bldc-stock-602 fw_60`.
   - Oder die Firmware, die vorher lief (siehe Abschnitt 0).
2. Genauso hochladen wie oben, erst die CAN-Knoten, zuletzt die Bridge.
3. Die Konfiguration bleibt bei gleicher Firmware-Version erhalten. Weicht sie ab, die Sicherung aus Abschnitt 2 mit `--setMcConf` bzw. `--setAppConf` zurückspielen.

## Hinweise

- **Bricht die Übertragung ab (Kabel ab, Verbindung weg), bleibt die alte Firmware aktiv.** Die neue liegt zunächst in einem eigenen Flash-Bereich. Kritisch ist das kurze Umkopieren durch den Bootloader nach der Übertragung: währenddessen die Versorgung auf keinen Fall unterbrechen.
- **Lädt die motorpi-App ein Motorprofil** (`--setMcConf`), endet eine laufende Aufnahme mit `CONFIG_CHANGED`. Das ist gewollt: Das Profil überschreibt auch die Strom-Offsets (Messbericht, Abschnitt 16).
- **Der Client hält den Port exklusiv.** Solange er läuft, kann die App nicht auf den VESC zugreifen.
- **Nach einem abgebrochenen `stream`** stoppt der Client die Aufnahme selbst und speichert die Teildaten (Abbruchgrund in der JSON-Datei). War der Controller dabei nicht erreichbar, beendet `CLIENT /dev/ttyACM0 [--can ID] stop` die Aufnahme; solange sie läuft, werden VESC-Tool-Sampling und neue Aufnahmen abgelehnt.
- **CSV-Spalten:** `index`, `t_s`, Ströme in A, Spannungen in V, `angle_q13` (exakter Winkel in 1/8192 elektrischer Umdrehung), `angle_el_rev`, `flags`.
