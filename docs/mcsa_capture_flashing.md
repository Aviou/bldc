# MCSA-Firmware flashen, prüfen und zurückspielen

Für die vier Flipsky FSESC 6.7 Pro am Prüfstand (HW „60“, FW 6.02): eine Bridge per USB, drei Knoten per CAN. Protokoll: [`mcsa_capture_protocol.md`](mcsa_capture_protocol.md).

**Erst nur die Bridge flashen und prüfen.** Die CAN-Knoten folgen erst, wenn an der Bridge alles stimmt.

> **Das Flashen löscht die gespeicherte Motor- und App-Konfiguration.** An der Bridge so bei jedem Hochladen beobachtet. Danach fehlen unter anderem Controller-ID, CAN-Einstellungen und Motorparameter. Deshalb jedes Mal vorher sichern (Abschnitt 2) und danach zurückspielen (Abschnitt 3).

## 0. Voraussetzungen

- Auf dem Pi läuft kein Test, und die App greift nicht auf den VESC zu:
  `curl -s localhost:8000/api/v1/tests/status/` zeigt `test_running` und `busy` auf `false`.
- Der Port ist frei: `sudo fuser /dev/ttyACM0` liefert nichts. Notfalls die App für die Dauer anhalten.
- Alle Motoren stehen, das Netzteil ist an und bleibt während des Hochladens an.
- `vesc_tool` auf dem Pi kann Firmware hochladen: `vesc_tool --help | grep -E 'uploadFirmware|canFwd'` zeigt beide Optionen. Fehlen sie, mit VESC Tool 6.02 am Laptop per USB arbeiten (Firmware → Custom File).
- Welche Firmware läuft heute? `vesc_tool --offscreen --vescPort /dev/ttyACM0 --queryDeviceFwParams`. Lief bisher ein eigener Build mit geänderten Standardwerten (z. B. aus `~/Documents/4Droin/bldc` mit angepassten `appconf_default.h`/`mcconf_default.h`), ist das der Rückweg. Dieser Branch nutzt die Stock-Standardwerte. Nach dem Flashen gelten sie, bis die Sicherung zurückgespielt ist.

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

## 3. Bridge flashen und Konfiguration zurückspielen

Am Prüfstand erledigt das `flash_bridge.sh` aus motorpi `scripts/mcsa` (nach `/home/pi/mcsa` kopieren, zusammen mit `check_stopped.py`, `term_cmd.py` und `vesc_sampleprint.py`):

```bash
bash /home/pi/mcsa/flash_bridge.sh --check mcsa_60_<hash>.bin   # nur Vorbedingungen prüfen
bash /home/pi/mcsa/flash_bridge.sh mcsa_60_<hash>.bin
```

Das Skript bricht ab, wenn die App nicht im Leerlauf ist oder nicht alle vier Motoren fehlerfrei stehen. Danach:
1. Es sichert Motor- und App-Konfiguration der Bridge nach `backup-<Zeit>/` und prüft, dass die Sicherung Controller-ID 1 enthält.
2. Es lädt die Firmware hoch und wartet, bis der USB-Port wieder da ist.
3. Es spielt beide Konfigurationen zurück, liest sie erneut aus und vergleicht. Erwartet: „Konfiguration zurückgespielt und identisch“.
4. Es zeigt den Git-Hash der laufenden Firmware.

Von Hand dasselbe:

```bash
vesc_tool --offscreen --vescPort /dev/ttyACM0 --uploadFirmware /home/pi/mcsa/60.bin
# einige Sekunden warten, bis /dev/ttyACM0 wieder da ist
vesc_tool --offscreen --vescPort /dev/ttyACM0 --setMcConf mc_bridge.xml
vesc_tool --offscreen --vescPort /dev/ttyACM0 --setAppConf app_bridge.xml
```

Danach `--getMcConf`/`--getAppConf` erneut auslesen und mit der Sicherung vergleichen.

## 4. Bridge prüfen

Den Client wie die motorpi-Skripte in einem Wegwerf-Container starten, ohne Netzwerk, nur mit dem USB-Port:

```bash
docker run --rm --network none --device /dev/ttyACM0 -v /home/pi/mcsa:/mcsa -v /tmp/mcsa_out:/out --entrypoint python motor-web /mcsa/mcsa_capture_client.py /dev/ttyACM0 info
```

Im Folgenden steht `CLIENT` für alles bis einschließlich `mcsa_capture_client.py`.

1. **`INFO`** meldet den notierten Git-Hash, 40 000 Byte Puffer, f_ISR 30 000 Hz und 7 Polpaare.
2. **VESC Tool und App sehen keinen Unterschied:**
   - `vesc_tool --offscreen --vescPort /dev/ttyACM0 --queryDeviceFwParams` meldet weiter 6.02, HW „60“ und keine Testversion.
   - Die zurückgespielte Konfiguration ist identisch mit der Sicherung (Abschnitt 3).
3. **`COMM_SAMPLE_PRINT` wie vorher:** `vesc_sampleprint.py /dev/ttyACM0 2000 10 0 1` aus `scripts/mcsa` (motorpi) liefert 2000 Pakete.
4. **Offset-Prüfung im Stillstand:** `CLIENT /dev/ttyACM0 offset-check`. Erwartet: Rest-Offset je Kanal unter 1,5 Stufen; Exit-Code 1 bedeutet Warnung.
5. **Einzelaufnahme im Stillstand:** `CLIENT /dev/ttyACM0 capture --decimation 10 --out /out/still_bridge`. Danach `status` ansehen: ISR-Maximum ohne und mit Aufnahme. Genauer: `term_cmd.py /dev/ttyACM0 -1 mcsa_perf` (Protokoll, Abschnitt Echtzeit). Bei zu hoher Last endet eine Aufnahme mit `FIFO_OVERRUN` oder `ISR_OVERLOAD`; tragfähige R je Maske stehen dort ebenfalls.
6. **`COMM_SAMPLE_PRINT` nach einer MCSA-Aufnahme:** Schritt 3 wiederholen. Erwartet: funktioniert, denn eine neue Aufnahme übernimmt den Speicher.
7. **Lauf mit Propeller** (12 000 ERPM, wie im Messbericht): Dafür muss ein Prozess den Motor drehen und zugleich aufnehmen. Der Client und `spin_capture.py` (motorpi) öffnen den Port jeweils exklusiv, und der Motor braucht alle 100 ms ein `COMM_SET_RPM` als Heartbeat, auch während eine Aufnahme läuft. Das übernimmt `spin_capture.py --mcsa` aus dem Repo `Aviou/motorpi`, Branch `feat/mcsa-spin-capture`, Ordner `scripts/mcsa` (dort auch eine Kopie dieses Clients). Es hält Heartbeat, Telemetrie und Abbruchprüfungen während jeder Anfrage aufrecht und gibt den Motor bei jedem Abbruch sofort frei:

   ```bash
   python spin_capture.py /dev/ttyACM0 -1 12000 /out 5 --mcsa --mcsa-seconds=20
   ```

   Es prüft vor dem Anlaufen Firmware und Rest-Offset und nimmt bei Zieldrehzahl auf:
   - mit Standardmaske und R = 10
   - mit Maske 0x01FF (alle Kanäle, ISR-Last prüfen; im Stillstand erst ab R = 10 vollständig)
   - als Daueraufnahme über 20 s

   Unter Last ist die CPU bei 30 kHz Regeltakt fast voll (FOC-Interrupt 78 %): Einzelaufnahmen mit Standardmaske gehen erst ab R = 20, Daueraufnahmen erst ab R = 50; darunter endet die Aufnahme sauber mit `FIFO_OVERRUN`. Ab 25 kHz Regeltakt (`foc_f_zv` 50 000) gehen auch Daueraufnahmen mit etwa 3 kHz. Gemessene Grenzen: Protokoll, Abschnitt Echtzeit. Grenzen eigener Konfigurationen misst `spin_capture.py --mcsa --mcsa-perf --mcsa-masks=…`.

   Kontrolle:
   - Der Kanal iq stimmt mit der Telemetrie überein.
   - Die 1000-Hz-Störlinie aus dem Messbericht ist verschwunden.
   - `status` meldet ein ISR-Maximum mit Aufnahme deutlich unter 33 µs.
   - Die JSON-Datei der Daueraufnahme zeigt keine Lücken.

## 5. CAN-Knoten flashen

Einzeln, nacheinander, jeweils mit Kontrolle. Auch hier geht die Konfiguration verloren, und mit ihr die CAN-ID: Ohne gespeicherte Konfiguration nimmt der Knoten eine aus seiner UUID abgeleitete ID (`HW_DEFAULT_ID`, `hwconf/hw.h`), nicht mehr 2, 3 oder 4. Deshalb nach dem Hochladen mit `bus_scan.py` die neue ID suchen und die Sicherung an diese ID schicken. Mit der App-Konfiguration kehrt die alte ID zurück.

```bash
vesc_tool --offscreen --vescPort /dev/ttyACM0 --canFwd 2 --uploadFirmware /home/pi/mcsa/60.bin
python bus_scan.py /dev/ttyACM0                 # neue ID des Knotens, hier NEU genannt
vesc_tool --offscreen --vescPort /dev/ttyACM0 --canFwd NEU --setMcConf mc_2.xml
vesc_tool --offscreen --vescPort /dev/ttyACM0 --canFwd NEU --setAppConf app_2.xml
CLIENT /dev/ttyACM0 --can 2 info
```

Nicht zwei Knoten nacheinander flashen, ohne dazwischen zurückzuspielen: Zwei Knoten ohne Konfiguration könnten dieselbe Standard-ID haben.

Danach CAN-ID 3 und 4. Mit `bus_scan.py` (motorpi) prüfen, dass alle vier Controller wieder antworten.

Prüfungen über CAN:
- **Einzelaufnahme im Stillstand:** `CLIENT /dev/ttyACM0 --can 3 capture --decimation 10 --out /out/m3_r10` wird lückenlos abgeholt, etwa 2–3 s.
- **Daueraufnahme:** `CLIENT /dev/ttyACM0 --can 3 stream --decimation 20 --seconds 20 --out /out/m3_stream`, immer nur ein Knoten zur Zeit (im Stillstand mit dem Client, unter Last mit `spin_capture.py --mcsa`). Bei Lücken in der JSON-Datei R erhöhen oder Kanäle weglassen.
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
3. Danach jedes Mal die Sicherung aus Abschnitt 2 mit `--setMcConf` bzw. `--setAppConf` zurückspielen und vergleichen.

## Hinweise

- **Bricht die Übertragung ab (Kabel ab, Verbindung weg), bleibt die alte Firmware aktiv.** Die neue liegt zunächst in einem eigenen Flash-Bereich. Kritisch ist das kurze Umkopieren durch den Bootloader nach der Übertragung: währenddessen die Versorgung auf keinen Fall unterbrechen.
- **Lädt die motorpi-App ein Motorprofil** (`--setMcConf`), endet eine laufende Aufnahme mit `CONFIG_CHANGED`. Das ist gewollt: Das Profil überschreibt auch die Strom-Offsets (Messbericht, Abschnitt 16).
- **Der Client hält den Port exklusiv.** Solange er läuft, kann die App nicht auf den VESC zugreifen.
- **Nach einem abgebrochenen `stream`** stoppt der Client die Aufnahme selbst und speichert die Teildaten (Abbruchgrund in der JSON-Datei). War der Controller dabei nicht erreichbar, beendet `CLIENT /dev/ttyACM0 [--can ID] stop` die Aufnahme; solange sie läuft, werden VESC-Tool-Sampling und neue Aufnahmen abgelehnt.
- **CSV-Spalten:** `index`, `t_s`, Ströme in A, Spannungen in V, `angle_q13` (exakter Winkel in 1/8192 elektrischer Umdrehung), `angle_el_rev`, `flags`.
