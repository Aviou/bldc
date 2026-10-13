"""Tests für mcsa_client/mcsa_capture_client.py; Ende-zu-Ende gegen den Firmware-Kern per ctypes.

Aufruf aus dem Repo-Wurzelverzeichnis: python3 -m pytest -p no:cacheprovider mcsa_client -q
"""
import ctypes
import os
import subprocess

import numpy as np
import pytest

import mcsa_capture_client as mc

HERE = os.path.dirname(os.path.abspath(__file__))
TEST_DIR = os.path.normpath(os.path.join(HERE, '..', 'tests', 'mcsa_core'))
LIB_PATH = os.path.join(TEST_DIR, 'libmcsa_core.so')
FP = ctypes.POINTER(ctypes.c_float)


@pytest.fixture(scope='session')
def lib():
    subprocess.run(['make', '-s', '-C', TEST_DIR, 'lib'], check=True)
    L = ctypes.CDLL(LIB_PATH)
    L.mcsa_tl_new.restype = ctypes.c_void_p
    L.mcsa_tl_new.argtypes = [ctypes.c_uint32]
    L.mcsa_tl_free.argtypes = [ctypes.c_void_p]
    L.mcsa_tl_set_offsets.argtypes = [ctypes.c_void_p, ctypes.c_float, ctypes.c_float, ctypes.c_float]
    L.mcsa_tl_set_uptime.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
    L.mcsa_tl_set_sample_print_busy.argtypes = [ctypes.c_void_p, ctypes.c_int]
    L.mcsa_tl_reboot.argtypes = [ctypes.c_void_p]
    L.mcsa_tl_feed.argtypes = [ctypes.c_void_p, ctypes.c_uint32, FP, FP, FP, FP, ctypes.c_int]
    L.mcsa_tl_process.restype = ctypes.c_int
    L.mcsa_tl_process.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_uint32,
                                  ctypes.c_char_p, ctypes.c_uint32]
    return L


class Sim:
    """ISR-Eingänge: Rampe auf Shunt a (exakt prüfbar), Kosinus auf b, Drehwinkel mit f_e."""

    def __init__(self, lib, h, f_e=200.0, f_isr=30000.0, levels=None):
        self.lib, self.h, self.f_e, self.f_isr = lib, h, f_e, f_isr
        self.levels = levels
        self.t = 0

    def feed(self, n, running=1):
        i = np.arange(self.t, self.t + n)
        th = 2 * np.pi * self.f_e * i / self.f_isr
        phase = (np.mod(th + np.pi, 2 * np.pi) - np.pi).astype(np.float32)
        if self.levels is None:
            a0 = (2048 + (i % 100)).astype(np.float32)
            a1 = np.round(2048 + 500 * np.cos(th)).astype(np.float32)
            a2 = np.full(n, 2048, dtype=np.float32)
        else:
            a0, a1, a2 = (np.full(n, v, dtype=np.float32) for v in self.levels)
        p = lambda a: a.ctypes.data_as(FP)
        self.lib.mcsa_tl_feed(self.h, n, p(phase), p(a0), p(a1), p(a2), running)
        self.t += n


class LibTransport:
    """Simulierter Bridge-Weg: Framing, optionale CAN-Weiterleitung, Kern statt Firmware."""

    def __init__(self, lib, h, sim, can_id=None, feed_per_request=0):
        self.lib, self.h, self.sim = lib, h, sim
        self.can_id = can_id
        self.feed_per_request = feed_per_request
        self.uptime = 1000
        self.lose_response_of = set()
        self.requests = 0

    def request(self, app_payload, cmd):
        self.requests += 1
        d = mc.Deframer()
        d.feed(mc.frame(mc.wrap_packet(app_payload, self.can_id)))
        (pkt,) = d.packets()
        if pkt[0] == mc.COMM_FORWARD_CAN:
            assert pkt[1] == self.can_id
            pkt = pkt[2:]
        assert pkt[0] == mc.COMM_CUSTOM_APP_DATA
        if self.feed_per_request:
            self.sim.feed(self.feed_per_request)
        self.uptime += 1
        self.lib.mcsa_tl_set_uptime(self.h, self.uptime)
        req = bytes(pkt[1:])
        resp = ctypes.create_string_buffer(mc.RESP_MAX)
        n = self.lib.mcsa_tl_process(self.h, req, len(req), resp, mc.RESP_MAX)
        assert n > 0
        d.feed(mc.frame(bytes([mc.COMM_CUSTOM_APP_DATA]) + resp.raw[:n]))
        (back,) = d.packets()
        if cmd in self.lose_response_of:
            self.lose_response_of.discard(cmd)
            raise TimeoutError('simulierter Antwortverlust')
        assert mc.is_response(back, cmd)
        return back[1:]


@pytest.fixture
def rig(lib):
    handles = []

    def make(buf_bytes=40000, can_id=None, feed_per_request=0, f_e=200.0, levels=None):
        h = lib.mcsa_tl_new(buf_bytes)
        handles.append(h)
        sim = Sim(lib, h, f_e=f_e, levels=levels)
        t = LibTransport(lib, h, sim, can_id, feed_per_request)
        return mc.McsaClient(t), t, sim

    yield make
    for h in handles:
        lib.mcsa_tl_free(h)


def test_crc16_vector():
    assert mc.crc16(b'123456789') == 0x31C3


def test_frame_roundtrip_short_and_long():
    d = mc.Deframer()
    short = bytes(range(10))
    long = bytes(i & 0xFF for i in range(300))
    d.feed(b'\x00\x07' + mc.frame(short) + mc.frame(long))
    assert d.packets() == [short, long]


def test_unwrap_angle_forward_backward_and_anchor_check():
    anchor = 3 * 65536 + 100
    vals = [(anchor + k * 20000) & 0xFFFF for k in range(10)]
    assert list(mc.unwrap_angle(vals, anchor)) == [anchor + k * 20000 for k in range(10)]
    anchor = -5 * 8192
    vals = [(anchor - k * 30000) & 0xFFFF for k in range(10)]
    assert list(mc.unwrap_angle(vals, anchor)) == [anchor - k * 30000 for k in range(10)]
    with pytest.raises(mc.ProtocolError):
        mc.unwrap_angle([1, 2, 3], 0)


def test_cic_gain_and_record_times():
    assert mc.cic_gain(0.0, 10, 30000.0) == pytest.approx(1.0)
    assert mc.cic_gain(3000.0, 10, 30000.0) == pytest.approx(0.0, abs=1e-12)
    x = np.pi * 400.0 / 30000.0
    assert mc.cic_gain(400.0, 10, 30000.0) == pytest.approx(abs(np.sin(10 * x) / (10 * np.sin(x))) ** 3)
    assert mc.record_times([0], 1, 30000.0)[0] == pytest.approx(2 / 30000.0)
    assert mc.record_times([0], 10, 30000.0)[0] == pytest.approx((30 - 1 - 13.5) / 30000.0)


def test_info_and_status(rig):
    client, _, _ = rig()
    info = client.info()
    assert info['protocol'] == 1 and info['cic_order'] == 3 and info['pole_pairs'] == 7
    assert info['buffer_bytes'] == 40000 and info['git_hash'] == 'testlib'
    s = client.status()
    assert s['state'] == mc.STATE_IDLE and s['session'] == 0


NOWAIT = lambda dt: None


def ramp_ok(rec, idx):
    """Shunt a ist eine Rampe (Zählwert mod 100): Werte passen exakt zum Index."""
    ia = rec[:, 0].astype(int) // 8
    assert np.all(rec[:, 0] % 8 == 0)
    base = (ia[0] - idx[0]) % 100
    assert np.all(ia == (base + idx) % 100)


@pytest.mark.parametrize('can_id', [None, 3])
def test_oneshot_capture_direct_and_via_can(rig, can_id):
    client, t, sim = rig(can_id=can_id, feed_per_request=2000)
    info, s, rec = client.capture(mask=0x0083, decimation=1, length=500, wait=NOWAIT)
    idx, recs, ang = rec.arrays()
    assert s['state'] == mc.STATE_DONE and s['stop_reason'] == 1
    assert list(idx) == list(range(500)) and rec.gaps == []
    ramp_ok(recs, idx)
    # Winkel: 200 Hz bei 30 kHz, R = 1
    step = 200.0 / 30000.0 * 8192.0
    assert np.allclose(np.diff(ang), step, atol=1.0)


def test_stream_with_overruns_keeps_exact_indices_and_angle(rig):
    # Ring 800 B, Maske ia+Winkel -> 200 Datensätze; 150 neue je Anfrage, höchstens 120 je READ
    client, t, sim = rig(buf_bytes=800, feed_per_request=150)
    ticks = iter(range(10 ** 6))
    info, s, rec = client.stream(mask=0x0081, decimation=1, seconds=60,
                                 clock=lambda: next(ticks), wait=NOWAIT)
    idx, recs, ang = rec.arrays()
    assert rec.gaps, 'der Test muss Überläufe erzeugen'
    final = s['n_written']
    covered = set(idx.tolist())
    for a, b in rec.gaps:
        assert a < b
        assert not covered.intersection(range(a, b))
        covered.update(range(a, b))
    assert covered == set(range(final))
    ramp_ok(recs, idx)
    step = 200.0 / 30000.0 * 8192.0
    assert np.allclose(ang - ang[0], (idx - idx[0]) * step, atol=2.0)


def test_lost_start_response_does_not_restart(rig):
    client, t, sim = rig(feed_per_request=2000)
    t.lose_response_of.add(mc.CMD_START)
    info, s, rec = client.capture(mask=0x0001, decimation=1, length=100, wait=NOWAIT)
    assert s['session'] == 1
    assert len(rec.arrays()[0]) == 100


def test_lost_read_response_is_repeated(rig):
    client, t, sim = rig(feed_per_request=2000)
    st = client.start(mc.MODE_ONESHOT, 0x0001, 1, 50)
    client.status()
    t.lose_response_of.add(mc.CMD_READ)
    status, r = client.read(st['session'], 0)
    assert status == mc.OK and r['count'] == 50


def test_reboot_is_detected_and_stream_aborts(rig, lib):
    client, t, sim = rig(feed_per_request=100)
    ticks = iter(range(10 ** 6))

    def clock():
        k = next(ticks)
        if k == 20:
            lib.mcsa_tl_reboot(t.h)
            t.uptime = 0
        return k

    with pytest.raises(mc.ProtocolError, match='Reboot'):
        client.stream(mask=0x0001, decimation=1, seconds=1000, clock=clock, wait=NOWAIT)
    assert client.reboots == 1


def test_start_refused_while_sample_print_busy(rig, lib):
    client, t, sim = rig()
    lib.mcsa_tl_set_sample_print_busy(t.h, 1)
    with pytest.raises(mc.ProtocolError, match='BUSY'):
        client.start(mc.MODE_ONESHOT, 0x0087, 10)


def test_offset_check_at_standstill(rig, lib):
    client, t, sim = rig(feed_per_request=5000, f_e=0.0, levels=(2048, 2049, 2046))
    lib.mcsa_tl_set_offsets(t.h, 2047.5, 2048.0, 2046.0)
    r = client.offset_check(decimation=10, length=200, threshold=1.5, wait=NOWAIT)
    assert r['residual_counts'] == pytest.approx([0.5, 1.0, 0.0], abs=1e-9)
    assert r['warning'] is False
    r = client.offset_check(decimation=10, length=200, threshold=0.75, wait=NOWAIT)
    assert r['warning'] is True


def test_write_outputs(rig, tmp_path):
    client, t, sim = rig(feed_per_request=2000)
    info, s, rec = client.capture(mask=0x0187, decimation=10, length=100, wait=NOWAIT)
    prefix = str(tmp_path / 'm1')
    mc.write_outputs(prefix, info, s, rec, {'mode': 'capture'}, client)
    lines = open(prefix + '.csv').read().splitlines()
    assert lines[0] == 'index,t_s,ia_A,ib_A,ic_A,angle_el_rev,flags'
    assert len(lines) == 101
    import json
    meta = json.load(open(prefix + '.json'))
    assert meta['records'] == 100 and meta['gaps'] == []
    assert meta['cic']['group_delay_s'] == pytest.approx(13.5 / 30000.0)
