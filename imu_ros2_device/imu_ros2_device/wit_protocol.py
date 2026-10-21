"""WitMotion standard serial protocol (Yahboom 10-axis IMU CMP10A): 0x55 frames -> SI samples, no ROS."""
import math
import struct
from collections import deque

G = 9.80665
TIME, ACC, GYRO, ANGLE, MAG, PRESS, QUAT = 0x50, 0x51, 0x52, 0x53, 0x54, 0x56, 0x59
FRAME_LEN = 11


class FrameParser:
    """Byte stream -> checksummed frames; resynchronises on the 0x55 header after noise or a lost byte."""

    def __init__(self):
        self.buf = bytearray()
        self.bad_checksums = 0

    def feed(self, data, read_time, byte_time):
        """Return [(kind, payload, end_time)]; end_time backdates each frame from read_time by the bytes after it."""
        self.buf += data
        found, i = [], 0
        while len(self.buf) - i >= FRAME_LEN:
            if self.buf[i] != 0x55 or not 0x50 <= self.buf[i + 1] <= 0x5F:
                i += 1
            elif sum(self.buf[i:i + 10]) & 0xFF != self.buf[i + 10]:
                self.bad_checksums += 1
                i += 1
            else:
                found.append((self.buf[i + 1], bytes(self.buf[i + 2:i + 10]), i + FRAME_LEN))
                i += FRAME_LEN
        frames = [(kind, payload, read_time - (len(self.buf) - end) * byte_time)
                  for kind, payload, end in found]
        del self.buf[:i]
        return frames


class FrameClock:
    """Map chip time-of-day to host monotonic time; legacy streams keep byte stamps."""
    # shortcut: UART arrival bounds estimate offset; absolute latency needs hardware timing calibration.

    def __init__(self, max_gap=.5):
        self.max_gap = max_gap
        self.seen_time = False
        self.last_read = None
        self.last_stamp = -math.inf
        self.resets = self.dropped_frames = 0
        self._reset()

    def _reset(self):
        self.last_ms = self.cycle = self.mapped_cycle = None
        self.tick = 0.0
        self.last_kind = TIME
        self.stamp = None
        self.offset = None
        self.observations = deque()

    def _map(self, pending, now):
        if not pending:
            return []
        # Serial backdating is an arrival upper bound, not the sample interval.
        bound = min(host - tick for _, _, tick, host in pending)
        self.observations.append((now, bound))
        while self.observations[0][0] < now - 1.0:
            self.observations.popleft()
        target = min(v for _, v in self.observations)
        if self.offset is None:
            self.offset = target
        else:
            # Follow clock drift; do not turn a later poll into a sample-time jump.
            elapsed = max(0.0, now - self.last_read) if self.last_read is not None else 0.0
            self.offset = min(target, self.offset + .01 * elapsed)
        out = []
        for kind, payload, tick, host in pending:
            if tick != self.mapped_cycle:
                self.mapped_cycle = tick
                candidate = min(tick + self.offset, host)
                self.stamp = candidate if self.last_stamp < candidate else None
                if self.stamp is not None:
                    self.last_stamp = self.stamp
            if self.stamp is None:
                self.dropped_frames += 1
            else:
                out.append((kind, payload, self.stamp))
        return out

    def feed(self, frames, now):
        """All sensor frames in one TIME cycle share the same acquisition stamp."""
        if self.last_read is not None and (now < self.last_read or now - self.last_read > self.max_gap):
            self._reset()
            self.resets += 1
            if now < self.last_read:
                self.last_stamp = -math.inf
        out, pending = [], []
        self.seen_time |= any(kind == TIME for kind, _, _ in frames)
        for kind, payload, host in frames:
            if kind == TIME:
                self.cycle = None
                self.last_kind = TIME
                _, _, _, hour, minute, second, lo, hi = payload
                ms = lo + (hi << 8)
                if hour >= 24 or minute >= 60 or second >= 60 or ms >= 1000:
                    self.dropped_frames += 1
                    continue
                ms += ((hour * 60 + minute) * 60 + second) * 1000
                delta = (ms - self.last_ms) % 86400000 if self.last_ms is not None else None
                if delta == 0:
                    self.dropped_frames += 1
                    continue
                if delta is not None and delta > self.max_gap * 1000:
                    out += self._map(pending, now)
                    pending = []
                    self._reset()
                    self.resets += 1
                    delta = None
                self.tick += delta / 1000 if delta is not None else 0.0
                self.last_ms, self.cycle = ms, self.tick
            elif not self.seen_time:
                out.append((kind, payload, host))
                self.last_stamp = max(self.last_stamp, host)
                continue
            elif not ACC <= kind <= QUAT:
                continue
            elif kind <= self.last_kind:
                # A new cycle without TIME must not inherit the previous stamp.
                self.cycle = None
            self.last_kind = kind
            if self.cycle is None:
                self.dropped_frames += 1
            else:
                pending.append((kind, payload, self.cycle, host))
        out += self._map(pending, now)
        self.last_read = now
        return out


def quaternion_from_euler(roll, pitch, yaw):
    """Z-Y-X (yaw, pitch, roll) angles in radians -> (x, y, z, w); the module reports this convention."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def decode(kind, payload):
    """Frame payload -> SI values (m/s², rad/s, rad, °C); None for types the driver does not use."""
    v = struct.unpack('<4h', payload)
    if kind == ACC:
        return {'accel': tuple(x / 32768 * 16 * G for x in v[:3]), 'temperature': v[3] / 100}
    if kind == GYRO:
        return {'gyro': tuple(math.radians(x / 32768 * 2000) for x in v[:3])}
    if kind == ANGLE:
        euler = tuple(math.radians(x / 32768 * 180) for x in v[:3])
        return {'euler': euler, 'orientation': quaternion_from_euler(*euler), 'version': v[3] & 0xFFFF}
    if kind == MAG:
        return {'mag_lsb': v[:3]}  # its temperature field reads 0 on firmware 18161; ACC carries it
    if kind == QUAT:
        norm = math.sqrt(sum(x * x for x in v))
        if norm == 0:
            return None
        w, x, y, z = (q / norm for q in v)
        return {'orientation': (x, y, z, w)}
    if kind == PRESS:
        pressure, height_cm = struct.unpack('<ii', payload)
        return {'pressure': float(pressure), 'height': height_cm / 100}
    return None


class SampleAssembler:
    """Combine one output cycle's frames.

    The module sends each cycle's frames in ascending type order (0x51 acc, 0x52 gyro, 0x53 angle, 0x54 mag,
    0x59 quaternion), so a gyro frame pairs with the accelerometer frame received since the previous gyro
    frame, and an orientation frame with the acc/gyro received since the previous orientation frame. Frames are
    numbered on arrival: a lost frame never lets an older value be paired twice, and values older than max_age
    are never paired.
    """

    def __init__(self, max_age):
        self.max_age = max_age
        self.seq = 0
        self.last = {}            # kind -> (seq, time, values)
        self.orientation_seq = 0  # seq of the frame that last triggered an orientation sample
        self.cycle_seq = 0        # TIME is a boundary even when some sensor frames are lost

    def _since(self, kind, seq, now):
        entry = self.last.get(kind)
        if entry and entry[0] > seq and now - entry[1] <= self.max_age:
            return entry[2]
        return None

    def push(self, kind, payload, t):
        """Return ('raw'|'data'|'mag', time, values) samples completed by this frame."""
        if kind == TIME:
            self.cycle_seq = self.seq
            self.orientation_seq = self.seq
            return []
        values = decode(kind, payload)
        if values is None:
            return []
        self.seq += 1
        previous = self.last.get(kind, (0, None, None))[0]
        self.last[kind] = (self.seq, t, values)
        if kind == GYRO:
            acc = self._since(ACC, max(previous, self.cycle_seq), t)
            return [('raw', t, {'accel': acc['accel'], 'gyro': values['gyro']})] if acc else []
        if kind == MAG:
            return [('mag', t, values)]
        if kind == QUAT or (kind == ANGLE and self._since(QUAT, 0, t) is None):
            since, self.orientation_seq = self.orientation_seq, self.seq
            acc, gyro = self._since(ACC, since, t), self._since(GYRO, since, t)
            if acc and gyro:
                return [('data', t, {'accel': acc['accel'], 'gyro': gyro['gyro'],
                                     'orientation': values['orientation']})]
        return []

    def latest(self, kind, now=None):
        entry = self.last.get(kind)
        return entry[2] if entry and (now is None or 0 <= now - entry[1] <= self.max_age) else None

    def age(self, kind, now):
        entry = self.last.get(kind)
        return now - entry[1] if entry else None
