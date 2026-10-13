#!/usr/bin/env python3
"""Referenzclient für den MCSA-Aufnahmemodus der VESC-Firmware (Branch feat/mcsa-capture-6.02).

Spricht einen VESC per USB direkt an oder einen Controller dahinter per COMM_FORWARD_CAN.
Protokoll: docs/mcsa_capture_protocol.md. Abhängigkeiten: pyserial, numpy.

Beispiele:
  python3 mcsa_capture_client.py /dev/ttyACM0 info
  python3 mcsa_capture_client.py /dev/ttyACM0 --can 3 capture --decimation 10 --out /tmp/m3
  python3 mcsa_capture_client.py /dev/ttyACM0 stream --decimation 10 --seconds 20 --out /tmp/m1
  python3 mcsa_capture_client.py /dev/ttyACM0 --can 2 offset-check
"""
import argparse
import json
import random
import struct
import sys
import time

import numpy as np

COMM_FORWARD_CAN = 34
COMM_CUSTOM_APP_DATA = 36

MAGIC = b'MC'
PROTO_VERSION = 1
CMD_INFO, CMD_START, CMD_STOP, CMD_STATUS, CMD_READ = 1, 2, 3, 4, 5
OK, BUSY, BAD_ARG, NOT_AVAILABLE, OVERWRITTEN, UNKNOWN_CMD, BAD_VERSION, NOT_FOC = range(8)
STATUS_NAMES = ['OK', 'BUSY', 'BAD_ARG', 'NOT_AVAILABLE', 'OVERWRITTEN', 'UNKNOWN_CMD',
                'BAD_VERSION', 'NOT_FOC']
STATE_IDLE, STATE_RUNNING, STATE_DONE, STATE_INVALID = range(4)
STATE_NAMES = ['IDLE', 'RUNNING', 'DONE', 'INVALID']
STOP_NAMES = ['NONE', 'LENGTH', 'STOP_CMD', 'OFFSETS_CHANGED', 'CONFIG_CHANGED',
              'INDEX_LIMIT', 'ANGLE_LIMIT', 'SAMPLE_PRINT', 'ANGLE_STEP']
MODE_ONESHOT, MODE_CONTINUOUS = 1, 2

CHANNELS = ['ia', 'ib', 'ic', 'id', 'iq', 'vd', 'vq', 'angle', 'flags']
MASK_DEFAULT = 0x0087
CIC_ORDER = 3
RESP_MAX = 511              # PACKET_MAX_PL_LEN - 1 (Paket-ID)
READ_OVERHEAD = 28
HDR_FMT = '>2sBBBHI'        # Magic, Version, Befehl|0x80, Status, Sitzung, Laufzeit ms
UPTIME_WRAP_MS = (2 ** 32 * 1000) // 10000   # Laufzeit springt nach ~4,97 Tagen auf 0
UPTIME_WRAP_WINDOW_MS = 600000
INFO_FMT = '>BBBBIff16s'
STATUS_FMT = '>BBBHBBBIIIIff3fqffI'
STATUS_KEYS = ['state', 'stop_reason', 'mode', 'mask', 'decimation', 'cic_order', 'pole_pairs',
               'capacity', 'length', 'n_written', 'oldest', 'f_isr', 'amp_per_unit',
               'offset0', 'offset1', 'offset2', 'theta_start', 'isr_us_idle_max',
               'isr_us_capture_max', 'saturated']


class ProtocolError(Exception):
    pass


def crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


def frame(payload):
    n = len(payload)
    head = bytes([2, n]) if n <= 255 else bytes([3, n >> 8, n & 0xFF])
    return head + payload + struct.pack('>H', crc16(payload)) + b'\x03'


class Deframer:
    """Sammelt Bytes und liefert vollständige, CRC-geprüfte VESC-Pakete."""

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf.extend(data)

    def packets(self):
        out = []
        while True:
            while self.buf and self.buf[0] not in (2, 3):
                self.buf.pop(0)
            if len(self.buf) < 3:
                break
            if self.buf[0] == 2:
                n, hl = self.buf[1], 2
            else:
                n, hl = (self.buf[1] << 8) | self.buf[2], 3
            total = hl + n + 3
            if len(self.buf) < total:
                break
            payload = bytes(self.buf[hl:hl + n])
            crc = struct.unpack('>H', self.buf[hl + n:hl + n + 2])[0]
            if self.buf[total - 1] == 3 and crc == crc16(payload):
                out.append(payload)
                del self.buf[:total]
            else:
                self.buf.pop(0)
        return out


def app_request(cmd, args=b''):
    return MAGIC + bytes([PROTO_VERSION, cmd]) + args


def wrap_packet(app_payload, can_id=None):
    pl = bytes([COMM_CUSTOM_APP_DATA]) + app_payload
    if can_id is not None:
        pl = bytes([COMM_FORWARD_CAN, can_id]) + pl
    return pl


def is_response(packet, cmd):
    return (len(packet) >= 12 and packet[0] == COMM_CUSTOM_APP_DATA and packet[1:3] == MAGIC
            and packet[4] == (cmd | 0x80))


class SerialTransport:
    """USB-Verbindung zum Bridge-VESC; mit can_id wird jede Anfrage weitergeleitet.

    Vor jeder Anfrage wird der Empfang geleert, damit verspätete Antworten früherer
    Anfragen nicht als neue gelten. Weitere Pakete bleiben für receive() vorgemerkt.
    """

    def __init__(self, port, can_id=None, timeout=1.0):
        import serial
        self.ser = serial.Serial(port, 115200, timeout=0.01, exclusive=True)
        self.ser.reset_input_buffer()
        self.deframer = Deframer()
        self.pending = []
        self.can_id = can_id
        self.timeout = timeout

    def request(self, app_payload, cmd):
        self.ser.reset_input_buffer()
        self.deframer = Deframer()
        self.pending = []
        self.ser.write(frame(wrap_packet(app_payload, self.can_id)))
        return self.receive(cmd)

    def receive(self, cmd):
        """Nächstes Antwortpaket auf `cmd`, ohne neu anzufragen."""
        t0 = time.monotonic()
        while True:
            for i, p in enumerate(self.pending):
                if is_response(p, cmd):
                    return self.pending.pop(i)[1:]
            if time.monotonic() - t0 >= self.timeout:
                raise TimeoutError(f'keine Antwort auf Befehl {cmd}')
            chunk = self.ser.read(4096)
            if chunk:
                self.deframer.feed(chunk)
                self.pending.extend(self.deframer.packets())

    def close(self):
        self.ser.close()


def parse_header(resp):
    if len(resp) < 11 or resp[:2] != MAGIC:
        raise ProtocolError('Antwort zu kurz oder fremd')
    _, version, cmd, status, session, uptime = struct.unpack(HDR_FMT, resp[:11])
    return {'version': version, 'cmd': cmd & 0x7F, 'status': status, 'session': session,
            'uptime_ms': uptime}, resp[11:]


def mask_channels(mask):
    return [name for i, name in enumerate(CHANNELS) if mask & (1 << i)]


def decode_records(data, nch):
    return np.frombuffer(data, dtype='>i2').astype(np.int16).reshape(-1, nch)


def unwrap_angle(angle_u16, anchor_q13):
    """Wickelt die 16-Bit-Winkel eines Blocks ab dem absoluten Anker ab (Einheit 1/8192 U.)."""
    a = np.asarray(angle_u16, dtype=np.int64) & 0xFFFF
    if len(a) == 0:
        return np.zeros(0, dtype=np.int64)
    if (anchor_q13 & 0xFFFF) != a[0]:
        raise ProtocolError('Winkelanker passt nicht zum ersten Datensatz')
    d = ((np.diff(a) + 32768) % 65536) - 32768
    return anchor_q13 + np.concatenate(([0], np.cumsum(d)))


def cic_gain(f, decimation, f_isr):
    """|H(f)| des CIC-Filters 3. Ordnung."""
    x = np.pi * np.asarray(f, dtype=float) / f_isr
    s = np.sin(x)
    small = np.abs(s) < 1e-12
    h = np.sin(decimation * x) / (decimation * np.where(small, 1.0, s))
    return np.where(small, 1.0, np.abs(h) ** CIC_ORDER)


def record_times(indices, decimation, f_isr):
    """Zeit jedes Datensatzes (Fenstermitte) ab dem ersten ISR-Durchlauf der Aufnahme, in s."""
    n = np.asarray(indices, dtype=float)
    return ((n + CIC_ORDER) * decimation - 1 - CIC_ORDER * (decimation - 1) / 2.0) / f_isr


class Recording:
    """Gesammelte Blöcke einer Aufnahme mit Lückenbuchhaltung."""

    def __init__(self, mask, decimation, f_isr):
        self.mask = mask
        self.channels = mask_channels(mask)
        self.nch = len(self.channels)
        self.decimation = decimation
        self.f_isr = f_isr
        self.blocks = []    # (start, records, angle_q13 oder None)
        self.gaps = []      # [von, bis) fehlende Indizes

    def add_block(self, start, data, anchor):
        recs = decode_records(data, self.nch)
        ang = None
        if 'angle' in self.channels:
            col = recs[:, self.channels.index('angle')].astype(np.uint16)
            ang = unwrap_angle(col, anchor)
        self.blocks.append((start, recs, ang))

    def arrays(self):
        if not self.blocks:
            return (np.zeros(0, dtype=np.int64), np.zeros((0, self.nch), dtype=np.int16),
                    np.zeros(0, dtype=np.int64) if 'angle' in self.channels else None)
        idx = np.concatenate([np.arange(s, s + len(r), dtype=np.int64) for s, r, _ in self.blocks])
        recs = np.concatenate([r for _, r, _ in self.blocks])
        ang = None
        if 'angle' in self.channels:
            ang = np.concatenate([a for _, _, a in self.blocks])
        return idx, recs, ang


class CaptureAborted(ProtocolError):
    """Aufnahme abgebrochen; `recording` enthält die bis dahin abgeholten Daten."""

    def __init__(self, message, info, recording):
        super().__init__(message)
        self.info = info
        self.recording = recording


class McsaClient:
    def __init__(self, transport, retries=3):
        self.t = transport
        self.retries = retries
        self.last_uptime = None
        self.reboots = 0
        self._guard_reboots = None

    def _note_uptime(self, uptime):
        last = self.last_uptime
        self.last_uptime = uptime
        # Veraltete Antworten sind hier schon aussortiert, die Laufzeit steigt also monoton
        if last is None or uptime >= last:
            return
        if last > UPTIME_WRAP_MS - UPTIME_WRAP_WINDOW_MS and uptime < UPTIME_WRAP_WINDOW_MS:
            return      # Überlauf des 32-Bit-Systemticks, kein Neustart
        self.reboots += 1
        if self._guard_reboots is not None:
            raise ProtocolError('Controller-Reboot erkannt, Aufnahme verloren')

    def _request(self, cmd, args=b'', accept=None):
        """Anfrage mit Wiederholung; `accept(hdr, body)` sortiert veraltete Antworten aus."""
        err = None
        for _ in range(self.retries):
            try:
                hdr, body = parse_header(self.t.request(app_request(cmd, args), cmd))
                while accept is not None and not accept(hdr, body):
                    hdr, body = parse_header(self.t.receive(cmd))
            except TimeoutError as e:
                err = e
                continue
            self._note_uptime(hdr['uptime_ms'])
            if hdr['status'] == BAD_VERSION:
                raise ProtocolError('Firmware spricht eine andere Protokollversion')
            return hdr, body
        raise err

    def info(self):
        hdr, b = self._request(CMD_INFO)
        if hdr['status'] != OK:
            raise ProtocolError(f"INFO: {STATUS_NAMES[hdr['status']]}")
        proto, order, rmax, pp, buf_bytes, f_isr, fac, git = struct.unpack(
            INFO_FMT, b[:struct.calcsize(INFO_FMT)])
        return {'protocol': proto, 'cic_order': order, 'decimation_max': rmax, 'pole_pairs': pp,
                'buffer_bytes': buf_bytes, 'f_isr': f_isr, 'fac_current': fac,
                'git_hash': git.rstrip(b'\0').decode(errors='replace')}

    def status(self):
        hdr, b = self._request(CMD_STATUS)
        if hdr['status'] != OK:
            raise ProtocolError(f"STATUS: {STATUS_NAMES[hdr['status']]}")
        s = dict(zip(STATUS_KEYS, struct.unpack(STATUS_FMT, b[:struct.calcsize(STATUS_FMT)])))
        s['offsets'] = [s.pop('offset0'), s.pop('offset1'), s.pop('offset2')]
        s['session'] = hdr['session']
        s['uptime_ms'] = hdr['uptime_ms']
        return s

    def start(self, mode, mask, decimation, length=0, token=None):
        """Startet eine neue Aufnahme. Bleibt die Sitzung gleich, hat das Token zufällig
        einen früheren START wiederholt; dann wird mit neuem Token erneut gestartet."""
        before = self.status()['session']
        for attempt in range(3):
            tok = token if (token is not None and attempt == 0) else random.randint(1, 0xFFFF)
            args = struct.pack('>BHBIH', mode, mask, decimation, length, tok)
            hdr, b = self._request(CMD_START, args)
            if hdr['status'] != OK:
                raise ProtocolError(f"START abgelehnt: {STATUS_NAMES[hdr['status']]}")
            if hdr['session'] != before:
                capacity, rec_bytes, mask_r, dec_r = struct.unpack('>IBHB', b[:8])
                return {'session': hdr['session'], 'capacity': capacity, 'rec_bytes': rec_bytes,
                        'mask': mask_r, 'decimation': dec_r, 'token': tok}
        raise ProtocolError('START: Sitzung blieb unverändert (Token-Kollision)')

    def stop(self, session):
        hdr, b = self._request(CMD_STOP, struct.pack('>H', session))
        if len(b) < 5:
            raise ProtocolError(f"STOP: {STATUS_NAMES[hdr['status']]}")
        n_written, reason = struct.unpack('>IB', b[:5])
        return hdr['status'], n_written, reason

    def stop_current(self):
        """Stoppt eine laufende Aufnahme, gleich welcher Client sie gestartet hat."""
        s = self.status()
        if s['state'] != STATE_RUNNING:
            return NOT_AVAILABLE, s['n_written'], s['stop_reason']
        return self.stop(s['session'])

    def read(self, session, start, count=0xFFFF, rec_bytes=None):
        def accept(hdr, body):
            if hdr['status'] != OK:
                return True
            if hdr['session'] != session or len(body) < 15:
                return False
            start_r, n, rb, _ = struct.unpack('>IHBq', body[:15])
            return (start_r == start and n <= count and (rec_bytes is None or rb == rec_bytes)
                    and len(body) == 15 + n * rb + 2)

        for _ in range(self.retries):
            hdr, b = self._request(CMD_READ, struct.pack('>HIH', session, start, count), accept)
            if hdr['status'] != OK:
                if len(b) < 8:
                    raise ProtocolError(f"READ: {STATUS_NAMES[hdr['status']]}")
                oldest, n_written = struct.unpack('>II', b[:8])
                return hdr['status'], {'oldest': oldest, 'n_written': n_written,
                                       'session': hdr['session']}
            start_r, n, rb, anchor = struct.unpack('>IHBq', b[:15])
            data = b[15:15 + n * rb]
            crc = struct.unpack('>H', b[15 + n * rb:17 + n * rb])[0]
            if crc16(data) == crc:
                return OK, {'start': start_r, 'count': n, 'rec_bytes': rb,
                            'anchor': anchor, 'data': data}
        raise ProtocolError('READ: CRC-Fehler trotz Wiederholung')

    def _check_session(self, expected, actual):
        if actual != expected:
            if self.reboots:
                raise ProtocolError('Controller-Reboot erkannt, Aufnahme verloren')
            raise ProtocolError('Sitzung hat gewechselt (anderer Client?)')

    def _read_into(self, rec, session, nxt):
        """Ein READ ab `nxt`; trägt Daten oder Lücken ein und liefert den neuen Index."""
        status, r = self.read(session, nxt, rec_bytes=2 * rec.nch)
        if status == OK:
            if r['count']:
                rec.add_block(r['start'], r['data'], r['anchor'])
            return nxt + r['count'], r['count']
        if status == OVERWRITTEN:
            if r['oldest'] > nxt:
                rec.gaps.append((nxt, r['oldest']))
                return r['oldest'], 0
            return nxt, 0       # veraltete Fehlermeldung, erneut lesen
        self._check_session(session, r['session'] if self.reboots == 0 else -1)
        raise ProtocolError(f'READ: {STATUS_NAMES[status]}')

    def capture(self, mask=MASK_DEFAULT, decimation=10, length=0, poll=0.05, wait=time.sleep):
        """Einzelaufnahme: starten, auf Ende warten, vollständig abholen."""
        info = self.info()
        rec = Recording(mask, decimation, info['f_isr'])
        self._guard_reboots = self.reboots
        try:
            st = self.start(MODE_ONESHOT, mask, decimation, length)
            n = length or st['capacity']
            timeout = (n + CIC_ORDER) * decimation / info['f_isr'] + 5.0
            t0 = time.monotonic()
            while True:
                s = self.status()
                self._check_session(st['session'], s['session'])
                if s['state'] != STATE_RUNNING:
                    break
                if time.monotonic() - t0 > timeout:
                    raise TimeoutError('Einzelaufnahme wird nicht fertig')
                wait(poll)
            nxt = 0
            while nxt < s['n_written']:
                nxt, got = self._read_into(rec, st['session'], nxt)
                if got == 0 and nxt < s['n_written'] and not rec.gaps:
                    raise ProtocolError('READ lieferte vor dem Ende keine Daten')
            return info, s, rec
        except (Exception, KeyboardInterrupt) as e:
            raise CaptureAborted(f'Einzelaufnahme abgebrochen: {e}', info, rec) from e
        finally:
            self._guard_reboots = None

    def stream(self, mask=MASK_DEFAULT, decimation=10, seconds=10.0, idle=0.002,
               clock=time.monotonic, wait=time.sleep):
        """Daueraufnahme: fortlaufend abholen, nach `seconds` stoppen und den Rest holen.
        Bei einem Abbruch wird die Aufnahme am Controller gestoppt (soweit erreichbar)."""
        info = self.info()
        rec = Recording(mask, decimation, info['f_isr'])
        session, final = None, None
        self._guard_reboots = self.reboots
        try:
            session = self.start(MODE_CONTINUOUS, mask, decimation)['session']
            t_end = clock() + seconds
            nxt = 0
            while final is None or nxt < final:
                if final is None and clock() >= t_end:
                    status, final, _ = self.stop(session)
                    if status != OK:
                        self._check_session(session, -1)
                    continue
                nxt, got = self._read_into(rec, session, nxt)
                if got == 0:
                    wait(idle)
            s = self.status()
            return info, s, rec
        except (Exception, KeyboardInterrupt) as e:
            self._guard_reboots = None
            if session is not None and final is None:
                try:
                    self.stop(session)
                except Exception:
                    pass
            raise CaptureAborted(f'Daueraufnahme abgebrochen: {e}', info, rec) from e
        finally:
            self._guard_reboots = None

    def offset_check(self, decimation=10, length=1000, threshold=1.5, wait=time.sleep):
        """Rest-Offset der drei Shunts im Stillstand, in ADC-Stufen."""
        info, s, rec = self.capture(0x0007, decimation, length, wait=wait)
        _, recs, _ = rec.arrays()
        resid = recs.astype(float).mean(axis=0) / 8.0
        return {'offsets': s['offsets'], 'residual_counts': resid.tolist(),
                'residual_amp': (resid * info['fac_current']).tolist(),
                'threshold_counts': threshold,
                'warning': bool(np.any(np.abs(resid) > threshold))}


COLUMN_FORMATS = {'index': '%d', 'angle_q13': '%d', 'flags': '%d', 'angle_el_rev': '%.6f'}


def to_table(rec, status):
    idx, recs, ang = rec.arrays()
    cols = {'index': idx, 't_s': record_times(idx, rec.decimation, rec.f_isr)}
    for j, ch in enumerate(rec.channels):
        x = recs[:, j]
        if ch in ('ia', 'ib', 'ic', 'id', 'iq'):
            cols[ch + '_A'] = x.astype(float) * status['amp_per_unit']
        elif ch in ('vd', 'vq'):
            cols[ch + '_V'] = x.astype(float) / 1000.0
        elif ch == 'angle':
            cols['angle_q13'] = ang
            cols['angle_el_rev'] = ang.astype(float) / 8192.0
        elif ch == 'flags':
            cols['flags'] = x.astype(np.uint16)
    return cols


def _json_default(o):
    return o.item() if hasattr(o, 'item') else str(o)


def write_outputs(prefix, info, status, rec, params, client):
    """CSV (ganzzahlige Spalten exakt, Winkel zusätzlich als 1/8192-Zählwert) und JSON-Kopf."""
    cols = to_table(rec, status)
    names = list(cols)
    fmts = [COLUMN_FORMATS.get(n, '%.9g') for n in names]
    columns = [np.asarray(cols[n]).tolist() for n in names]
    with open(prefix + '.csv', 'w') as f:
        f.write(','.join(names) + '\n')
        for row in zip(*columns):
            f.write(','.join(fm % v for fm, v in zip(fmts, row)) + '\n')
    meta = {
        'info': info,
        'status': status,
        'params': params,
        'channels': rec.channels,
        'records': int(len(cols['index'])),
        'gaps': [list(g) for g in rec.gaps],
        'reboots': client.reboots,
        'cic': {'order': CIC_ORDER, 'decimation': rec.decimation, 'f_isr': rec.f_isr,
                'group_delay_s': CIC_ORDER * (rec.decimation - 1) / 2.0 / rec.f_isr},
    }
    with open(prefix + '.json', 'w') as f:
        json.dump(meta, f, indent=2, default=_json_default)


def _summary(s, rec):
    idx = rec.arrays()[0]
    missing = sum(b - a for a, b in rec.gaps)
    return (f"{len(idx)} Datensätze, {len(rec.gaps)} Lücken ({missing} fehlend), "
            f"Stoppgrund {STOP_NAMES[s['stop_reason']]}, ISR max {s['isr_us_capture_max']:.1f} µs "
            f"(ohne Aufnahme {s['isr_us_idle_max']:.1f} µs), Sättigungen {s['saturated']}")


def main(argv=None):
    p = argparse.ArgumentParser(description='MCSA-Aufnahme vom VESC (COMM_CUSTOM_APP_DATA)')
    p.add_argument('port', help='z. B. /dev/ttyACM0')
    p.add_argument('--can', type=int, default=None, help='CAN-ID des Controllers hinter der Bridge')
    p.add_argument('--timeout', type=float, default=1.0, help='Antwort-Zeitlimit je Anfrage in s')
    sub = p.add_subparsers(dest='cmd', required=True)
    sub.add_parser('info')
    sub.add_parser('status')
    sub.add_parser('stop', help='laufende Aufnahme beenden (z. B. nach einem Abbruch)')
    pc = sub.add_parser('capture', help='Einzelaufnahme')
    pc.add_argument('--mask', type=lambda v: int(v, 0), default=MASK_DEFAULT)
    pc.add_argument('--decimation', type=int, default=10)
    pc.add_argument('--length', type=int, default=0, help='0 = ganzer Puffer')
    pc.add_argument('--out', required=True, help='Präfix für .csv und .json')
    ps = sub.add_parser('stream', help='Daueraufnahme')
    ps.add_argument('--mask', type=lambda v: int(v, 0), default=MASK_DEFAULT)
    ps.add_argument('--decimation', type=int, default=10)
    ps.add_argument('--seconds', type=float, required=True)
    ps.add_argument('--out', required=True)
    po = sub.add_parser('offset-check', help='Rest-Offset im Stillstand')
    po.add_argument('--decimation', type=int, default=10)
    po.add_argument('--length', type=int, default=1000)
    po.add_argument('--threshold', type=float, default=1.5, help='Warnschwelle in ADC-Stufen')
    args = p.parse_args(argv)

    transport = SerialTransport(args.port, args.can, args.timeout)
    client = McsaClient(transport)
    try:
        if args.cmd == 'info':
            print(json.dumps(client.info(), indent=2))
        elif args.cmd == 'status':
            print(json.dumps(client.status(), indent=2, default=_json_default))
        elif args.cmd == 'stop':
            status, n_written, reason = client.stop_current()
            print(f'{STATUS_NAMES[status]}: {n_written} Datensätze, Stoppgrund {STOP_NAMES[reason]}')
        elif args.cmd in ('capture', 'stream'):
            if args.cmd == 'capture':
                info, s, rec = client.capture(args.mask, args.decimation, args.length)
            else:
                info, s, rec = client.stream(args.mask, args.decimation, args.seconds)
            write_outputs(args.out, info, s, rec, vars(args), client)
            print(_summary(s, rec), file=sys.stderr)
        elif args.cmd == 'offset-check':
            r = client.offset_check(args.decimation, args.length, args.threshold)
            print(json.dumps(r, indent=2))
            return 1 if r['warning'] else 0
    except CaptureAborted as e:
        if getattr(args, 'out', None) and e.info:
            params = dict(vars(args), aborted=str(e))
            status = {'amp_per_unit': e.info['fac_current'] / 8.0}
            write_outputs(args.out, e.info, status, e.recording, params, client)
            print(f'Teildaten gespeichert unter {args.out}.csv/.json', file=sys.stderr)
        print(f'Fehler: {e}', file=sys.stderr)
        return 2
    except (ProtocolError, TimeoutError) as e:
        print(f'Fehler: {e}', file=sys.stderr)
        return 2
    finally:
        transport.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
