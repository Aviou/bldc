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
              'INDEX_LIMIT', 'ANGLE_LIMIT', 'SAMPLE_PRINT']
MODE_ONESHOT, MODE_CONTINUOUS = 1, 2

CHANNELS = ['ia', 'ib', 'ic', 'id', 'iq', 'vd', 'vq', 'angle', 'flags']
MASK_DEFAULT = 0x0087
CIC_ORDER = 3
RESP_MAX = 511              # PACKET_MAX_PL_LEN - 1 (Paket-ID)
READ_OVERHEAD = 28
HDR_FMT = '>2sBBBHI'        # Magic, Version, Befehl|0x80, Status, Sitzung, Laufzeit ms
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
    """USB-Verbindung zum Bridge-VESC; mit can_id wird jede Anfrage weitergeleitet."""

    def __init__(self, port, can_id=None, timeout=1.0):
        import serial
        self.ser = serial.Serial(port, 115200, timeout=0.01, exclusive=True)
        self.ser.reset_input_buffer()
        self.deframer = Deframer()
        self.can_id = can_id
        self.timeout = timeout

    def request(self, app_payload, cmd):
        self.ser.write(frame(wrap_packet(app_payload, self.can_id)))
        t0 = time.monotonic()
        while time.monotonic() - t0 < self.timeout:
            chunk = self.ser.read(4096)
            if chunk:
                self.deframer.feed(chunk)
                for p in self.deframer.packets():
                    if is_response(p, cmd):
                        return p[1:]
        raise TimeoutError(f'keine Antwort auf Befehl {cmd}')

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


class McsaClient:
    def __init__(self, transport, retries=3):
        self.t = transport
        self.retries = retries
        self.last_uptime = None
        self.reboots = 0

    def _request(self, cmd, args=b''):
        err = None
        for _ in range(self.retries):
            try:
                resp = self.t.request(app_request(cmd, args), cmd)
            except TimeoutError as e:
                err = e
                continue
            hdr, body = parse_header(resp)
            if self.last_uptime is not None and hdr['uptime_ms'] < self.last_uptime:
                self.reboots += 1
            self.last_uptime = hdr['uptime_ms']
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
        token = token if token is not None else random.randint(1, 0xFFFF)
        args = struct.pack('>BHBIH', mode, mask, decimation, length, token)
        hdr, b = self._request(CMD_START, args)
        if hdr['status'] != OK:
            raise ProtocolError(f"START abgelehnt: {STATUS_NAMES[hdr['status']]}")
        capacity, rec_bytes, mask_r, dec_r = struct.unpack('>IBHB', b[:8])
        return {'session': hdr['session'], 'capacity': capacity, 'rec_bytes': rec_bytes,
                'mask': mask_r, 'decimation': dec_r, 'token': token}

    def stop(self, session):
        hdr, b = self._request(CMD_STOP, struct.pack('>H', session))
        if len(b) < 5:
            raise ProtocolError(f"STOP: {STATUS_NAMES[hdr['status']]}")
        n_written, reason = struct.unpack('>IB', b[:5])
        return hdr['status'], n_written, reason

    def read(self, session, start, count=0xFFFF):
        for _ in range(self.retries):
            hdr, b = self._request(CMD_READ, struct.pack('>HIH', session, start, count))
            if hdr['status'] != OK:
                if len(b) < 8:
                    raise ProtocolError(f"READ: {STATUS_NAMES[hdr['status']]}")
                oldest, n_written = struct.unpack('>II', b[:8])
                return hdr['status'], {'oldest': oldest, 'n_written': n_written,
                                       'session': hdr['session']}
            start_r, n, rec_bytes, anchor = struct.unpack('>IHBq', b[:15])
            data = b[15:15 + n * rec_bytes]
            crc = struct.unpack('>H', b[15 + n * rec_bytes:17 + n * rec_bytes])[0]
            if crc16(data) == crc:
                return OK, {'start': start_r, 'count': n, 'rec_bytes': rec_bytes,
                            'anchor': anchor, 'data': data}
        raise ProtocolError('READ: CRC-Fehler trotz Wiederholung')
