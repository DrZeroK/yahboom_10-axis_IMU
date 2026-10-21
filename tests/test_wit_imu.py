"""WitMotion 10-axis IMU: protocol parsing (no ROS needed) and ROS messages (Humble host only)."""
import math
import struct
import time
import unittest
from importlib.util import find_spec
from unittest.mock import patch

from imu_ros2_device.wit_protocol import (TIME, ACC, ANGLE, G, GYRO, MAG, QUAT, FrameClock, FrameParser, SampleAssembler,
                                          decode)

HAS_ROS = find_spec('rclpy') is not None
if HAS_ROS:
    import rclpy
    import serial
    from rclpy.serialization import serialize_message
    from imu_ros2_device.wit_imu_driver import WitImuDriver

# One factory-default cycle (acc, gyro, angle, mag) captured from the CMP10A on X5 /dev/ttyUSB2, 2026-10-02.
CAPTURED_CYCLE = bytes.fromhex(
    '555189fed4ffe1078f0a81' '55520000000000008f0a40' '555317ff8507c7d0f14618' '5554ccf9840438f000001e')


def frame(kind, *words):
    body = bytes([0x55, kind]) + struct.pack('<4h', *(list(words) + [0] * (4 - len(words))))
    return body + bytes([sum(body) & 0xFF])


def time_frame(ms):
    seconds, millis = divmod(ms % 86400000, 1000)
    minutes, seconds = divmod(seconds, 60)
    hours, minutes = divmod(minutes, 60)
    body = bytes((0x55, TIME, 0, 0, 0, hours, minutes, seconds, millis & 255, millis >> 8))
    return body + bytes((sum(body) & 255,))


class ClockTests(unittest.TestCase):
    def setUp(self):
        self.parser, self.clock = FrameParser(), FrameClock()

    def feed(self, stream, now, byte_time=10/115200):
        return self.clock.feed(self.parser.feed(stream, now, byte_time), now)

    def test_batch_uses_chip_intervals_and_one_stamp_per_cycle(self):
        frames = self.feed(b''.join(time_frame(ms) + CAPTURED_CYCLE for ms in (0, 20, 40)), 10)
        gyro = [t for k, _, t in frames if k == GYRO]
        self.assertEqual(len(gyro), 3)
        for a, b in zip(gyro, gyro[1:]):
            self.assertAlmostEqual(b-a, .02)
        for i in range(0, len(frames), 5):
            self.assertEqual(len({t for _, _, t in frames[i:i+5]}), 1)
        self.assertLessEqual(max(t for _, _, t in frames), 10)

    def test_no_time_output_preserves_legacy_stamps(self):
        frames = self.parser.feed(CAPTURED_CYCLE * 2, 10, 10/9600)
        self.assertEqual(self.clock.feed(frames, 10), frames)
        self.assertFalse(self.clock.seen_time)

    def test_split_frame_and_split_cycle_keep_the_cycle_stamp(self):
        stream = time_frame(2000) + CAPTURED_CYCLE
        frames = []
        for i in range(0, len(stream), 3):
            frames += self.feed(stream[i:i+3], 10 + i * .001)
        self.assertEqual([k for k, _, _ in frames], [TIME, ACC, GYRO, ANGLE, MAG])
        self.assertEqual(len({t for _, _, t in frames}), 1)

    def test_midnight_wrap_is_not_a_reset(self):
        frames = self.feed(time_frame(86399990)+CAPTURED_CYCLE+time_frame(10)+CAPTURED_CYCLE, 10)
        gyro = [t for k, _, t in frames if k == GYRO]
        self.assertAlmostEqual(gyro[1]-gyro[0], .02)
        self.assertEqual(self.clock.resets, 0)

    def test_reset_reanchors_to_host_time(self):
        self.feed(time_frame(10000)+CAPTURED_CYCLE, 10)
        frames = self.feed(time_frame(0)+CAPTURED_CYCLE, 10.1)
        self.assertEqual(self.clock.resets, 1)
        self.assertTrue(all(10 < t <= 10.1 for _, _, t in frames))

    def test_reset_inside_a_batch_never_reverses_sample_order(self):
        frames = self.feed(time_frame(10000)+CAPTURED_CYCLE+time_frame(0)+CAPTURED_CYCLE, 10)
        stamps = [t for k, _, t in frames if k == GYRO]
        self.assertEqual(len(stamps), 2)
        self.assertLess(stamps[0], stamps[1])
        self.assertLessEqual(stamps[1], 10)
        self.assertEqual(self.clock.resets, 1)

    def test_gap_does_not_compress_elapsed_time(self):
        first = self.feed(time_frame(0)+CAPTURED_CYCLE, 10)
        second = self.feed(time_frame(20)+CAPTURED_CYCLE, 12)
        self.assertEqual(self.clock.resets, 1)
        self.assertAlmostEqual(second[0][2]-first[0][2], 2)

    def test_clock_backwards_starts_a_new_host_epoch(self):
        self.feed(time_frame(0)+CAPTURED_CYCLE, 10)
        frames = self.feed(time_frame(20)+CAPTURED_CYCLE, 1)
        self.assertTrue(frames)
        self.assertTrue(all(0 < t <= 1 for _, _, t in frames))

    def test_missing_corrupt_invalid_and_duplicate_time_drop_the_cycle(self):
        invalid = bytearray(time_frame(20))
        invalid[5] = 24
        invalid[-1] = sum(invalid[:-1]) & 255
        bad_crc = bytearray(time_frame(20))
        bad_crc[-1] ^= 1
        for bad in (b'', bytes(bad_crc), bytes(invalid), time_frame(0)):
            with self.subTest(bad=bad.hex()):
                self.setUp()
                self.feed(time_frame(0)+CAPTURED_CYCLE, 10)
                self.assertEqual(self.feed(bad+CAPTURED_CYCLE, 10.02), [])
                frames = self.feed(time_frame(40)+CAPTURED_CYCLE, 10.04)
                self.assertEqual(sum(k == GYRO for k, _, _ in frames), 1)
                self.assertGreater(self.clock.dropped_frames, 0)

    def test_time_boundary_prevents_cross_cycle_accelerometer_pairing(self):
        assembler = SampleAssembler(.5)
        stream = time_frame(0)+frame(ACC)+time_frame(20)+frame(GYRO)
        samples = [s for k, p, t in self.feed(stream, 10) for s in assembler.push(k, p, t)]
        self.assertEqual(samples, [])

    def test_partial_cycle_retains_latest_values_for_diagnostics(self):
        assembler = SampleAssembler(.5)
        for k, p, t in self.feed(time_frame(0)+CAPTURED_CYCLE+time_frame(20), 10):
            assembler.push(k, p, t)
        self.assertAlmostEqual(assembler.latest(ACC)['temperature'], 27.03)

    def test_capture_starting_mid_cycle_waits_for_time(self):
        frames = self.feed(CAPTURED_CYCLE[22:]+time_frame(20)+CAPTURED_CYCLE, 10)
        self.assertEqual([k for k, _, _ in frames], [TIME, ACC, GYRO, ANGLE, MAG])
        self.assertEqual(self.clock.dropped_frames, 2)

    def test_time_boundary_preserves_quaternion_preference(self):
        assembler = SampleAssembler(.5)
        cycle = CAPTURED_CYCLE + frame(QUAT, 23170, 23170, 0, 0)
        stream = b''.join(time_frame(ms)+cycle for ms in (0, 20, 40))
        samples = [s for k, p, t in self.feed(stream, 10) for s in assembler.push(k, p, t)]
        orientations = [v['orientation'] for kind, _, v in samples if kind == 'data']
        self.assertEqual(len(orientations), 3)
        self.assertAlmostEqual(orientations[-1][0], math.sqrt(.5), places=4)

    def test_host_mapping_tracks_both_clock_drift_directions(self):
        for scale in (.998, 1.002):
            with self.subTest(scale=scale):
                self.setUp()
                stamps, ages = [], []
                for i in range(3000):
                    now = 10 + i * .02 * scale
                    frames = self.feed(time_frame(i*20)+CAPTURED_CYCLE, now)
                    stamp = next(t for k, _, t in frames if k == GYRO)
                    stamps.append(stamp)
                    ages.append(now-stamp)
                self.assertTrue(all(a < b for a, b in zip(stamps, stamps[1:])))
                self.assertLess(max(ages), .01)
                self.assertGreaterEqual(min(ages), 0)
                self.assertEqual(self.clock.resets, 0)


def rotate(q, v):
    x, y, z, w = q
    r = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
         [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
         [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    return [sum(r[i][j] * v[j] for j in range(3)) for i in range(3)]


def assemble(stream, read_time=100.0, byte_time=1e-3, max_age=.5):
    parser, assembler = FrameParser(), SampleAssembler(max_age)
    samples = []
    for kind, payload, t in parser.feed(stream, read_time, byte_time):
        samples += assembler.push(kind, payload, t)
    return samples


class ProtocolTests(unittest.TestCase):
    def test_pressure_latest_can_expire_without_changing_legacy_latest(self):
        from imu_ros2_device.wit_protocol import PRESS
        samples = SampleAssembler(.5)
        samples.push(PRESS, struct.pack('<ii',100000,1234),10.)
        self.assertEqual(samples.latest(PRESS,10.1)['height'],12.34)
        self.assertAlmostEqual(samples.age(PRESS,10.1),.1)
        self.assertIsNone(samples.latest(PRESS,10.6))
        self.assertIsNone(samples.latest(PRESS,9.9))
        self.assertEqual(samples.latest(PRESS)['pressure'],100000)

    def test_captured_cycle_decodes_to_manual_units(self):
        frames = FrameParser().feed(CAPTURED_CYCLE, 0.0, 0.0)
        self.assertEqual([f[0] for f in frames], [ACC, GYRO, ANGLE, MAG])
        acc, gyro, angle, mag = (decode(kind, payload) for kind, payload, _ in frames)
        for got, raw in zip(acc['accel'], (-375, -44, 2017)):
            self.assertAlmostEqual(got, raw / 32768 * 16 * G)
        self.assertAlmostEqual(acc['temperature'], 27.03)
        self.assertEqual(gyro['gyro'], (0.0, 0.0, 0.0))
        for got, raw in zip(angle['euler'], (-233, 1925, -12089)):
            self.assertAlmostEqual(got, math.radians(raw / 32768 * 180))
        self.assertEqual(angle['version'], 18161)
        self.assertEqual(mag['mag_lsb'], (-1588, 1156, -4040))

    def test_module_orientation_agrees_with_measured_gravity(self):
        # Rotating the body-frame specific force into the world frame must give +g on Z only;
        # this pins the Euler order and axis conventions to what the real module reports.
        data = [s for s in assemble(CAPTURED_CYCLE) if s[0] == 'data']
        self.assertEqual(len(data), 1)
        world = rotate(data[0][2]['orientation'], data[0][2]['accel'])
        self.assertAlmostEqual(world[0], 0.0, delta=.05)
        self.assertAlmostEqual(world[1], 0.0, delta=.05)
        self.assertAlmostEqual(world[2], math.hypot(*data[0][2]['accel']), delta=.01)

    def test_noise_split_reads_and_bad_checksums(self):
        bad = bytearray(frame(ANGLE, 1, 2, 3))
        bad[-1] ^= 1
        stream = b'\x00\x55\x13' + frame(ACC, 1, 2, 2048) + bytes(bad) + frame(GYRO, 4, 5, 6)
        parser = FrameParser()
        frames = []
        for i in range(0, len(stream), 3):
            frames += parser.feed(stream[i:i + 3], 0.0, 0.0)
        self.assertEqual([f[0] for f in frames], [ACC, GYRO])
        self.assertEqual(parser.bad_checksums, 1)
        self.assertLess(len(parser.buf), 11)

    def test_frames_are_backdated_by_the_bytes_after_them(self):
        frames = FrameParser().feed(frame(ACC) + frame(GYRO) + b'\x55\x51', 10.0, .001)
        self.assertAlmostEqual(frames[0][2], 10.0 - .013)
        self.assertAlmostEqual(frames[1][2], 10.0 - .002)

    def test_raw_needs_this_cycles_accelerometer(self):
        self.assertEqual([s[0] for s in assemble(frame(ACC, 0, 0, 2048) + frame(GYRO))], ['raw'])
        # The gyro frame of cycle 2 arrives without its accelerometer frame: never reuse cycle 1's.
        self.assertEqual([s[0] for s in assemble(frame(ACC) + frame(GYRO) + frame(GYRO))], ['raw'])

    def test_two_cycles_in_one_read_both_publish(self):
        samples = assemble(CAPTURED_CYCLE * 2)
        self.assertEqual([s[0] for s in samples], ['raw', 'data', 'mag'] * 2)
        times = [s[1] for s in samples]
        self.assertEqual(times, sorted(times))
        self.assertEqual(len(set(times)), len(times))

    def test_stale_values_are_not_paired(self):
        assembler = SampleAssembler(.5)
        assembler.push(ACC, frame(ACC)[2:10], 0.0)
        self.assertEqual(assembler.push(GYRO, frame(GYRO)[2:10], 1.0), [])

    def test_quaternion_output_replaces_euler_when_enabled(self):
        cycle = (frame(ACC, 0, 0, 2048) + frame(GYRO) + frame(ANGLE, 0, 0, 16384) + frame(MAG)
                 + frame(QUAT, 23170, 23170, 0, 0))  # w, x, y, z: roll 90°
        data = [s for s in assemble(cycle * 3) if s[0] == 'data']
        # First cycle: Euler until the first quaternion; then exactly one quaternion sample per cycle.
        self.assertEqual(len(data), 3)
        for _, _, values in data[1:]:
            x, y, z, w = values['orientation']
            self.assertAlmostEqual(math.hypot(x, y, z, w), 1.0)
            self.assertAlmostEqual(x, math.sqrt(.5), places=4)
            self.assertAlmostEqual(w, math.sqrt(.5), places=4)
        x, y, z, w = data[0][2]['orientation']  # yaw 90° from the angle frame
        self.assertAlmostEqual(z, math.sqrt(.5), places=4)


class Capture:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        serialize_message(msg)
        self.messages.append(msg)


@unittest.skipUnless(HAS_ROS, 'Run on a ROS Humble host')
class NodeTests(unittest.TestCase):
    def test_pressure_diagnostics_include_height_age_and_expire(self):
        from imu_ros2_device.wit_protocol import PRESS
        now = time.monotonic()
        self.node.assembler.push(PRESS,struct.pack('<ii',100000,1234),now)
        with patch('imu_ros2_device.wit_imu_driver.time.monotonic',return_value=now+.1):
            self.node.diagnostics()
        values = {v.key:v.value for v in self.node.diag_pub.messages[-1].status[0].values}
        self.assertEqual(values['pressure_pa'],'100000.0')
        self.assertEqual(values['baro_height_m'],'12.34')
        self.assertAlmostEqual(float(values['pressure_age_s']),.1)
        with patch('imu_ros2_device.wit_imu_driver.time.monotonic',return_value=now+1):
            self.node.diagnostics()
        values = {v.key:v.value for v in self.node.diag_pub.messages[-1].status[0].values}
        self.assertEqual(values['pressure_pa'],'None')

    @classmethod
    def setUpClass(cls):
        # Isolated domain; outputs are captured, never sent.
        rclpy.init(domain_id=199)

    @classmethod
    def tearDownClass(cls):
        rclpy.try_shutdown()

    def setUp(self):
        with patch('imu_ros2_device.wit_imu_driver.serial.Serial') as port:
            port.return_value.read.return_value = b''
            self.node = WitImuDriver()
        for name in ('raw_pub', 'data_pub', 'mag_pub', 'diag_pub'):
            setattr(self.node, name, Capture())

    def tearDown(self):
        self.node.destroy_node()

    def test_messages_follow_rep145_and_si_units(self):
        self.node.feed(CAPTURED_CYCLE, time.monotonic())
        raw = self.node.raw_pub.messages[-1]
        data = self.node.data_pub.messages[-1]
        mag = self.node.mag_pub.messages[-1]
        self.assertEqual(raw.header.frame_id, 'imu_link')
        self.assertEqual(raw.orientation_covariance[0], -1.0)
        self.assertAlmostEqual(raw.linear_acceleration.z, 2017 / 32768 * 16 * G)
        q = data.orientation
        self.assertAlmostEqual(math.hypot(q.x, q.y, q.z, q.w), 1.0)
        self.assertGreater(data.orientation_covariance[0], 0)
        self.assertGreater(data.orientation_covariance[8], data.orientation_covariance[0])
        self.assertAlmostEqual(mag.magnetic_field.z, -4040 * 6.67e-9)

    def test_stamps_follow_frame_order(self):
        self.node.feed(CAPTURED_CYCLE, time.monotonic())
        stamps = [m.messages[-1].header.stamp for m in (self.node.raw_pub, self.node.data_pub, self.node.mag_pub)]
        ns = [s.sec * 1_000_000_000 + s.nanosec for s in stamps]
        self.assertEqual(ns, sorted(ns))
        # mag ends the read; raw ended two frames (22 bytes at 9600 baud) earlier.
        self.assertAlmostEqual((ns[2] - ns[0]) / 1e9, 22 * 10 / 9600, places=4)

    def test_diagnostics_distinguish_silence_wrong_baud_and_ok(self):
        self.node.diagnostics()
        self.assertEqual(self.node.diag_pub.messages[-1].status[0].level, b'\x02')
        self.node.feed(bytes(range(256)) * 2, time.monotonic())
        self.node.diagnostics()
        self.assertIn('波特率', self.node.diag_pub.messages[-1].status[0].message)
        self.node.feed(CAPTURED_CYCLE, time.monotonic())
        self.node.diagnostics()
        status = self.node.diag_pub.messages[-1].status[0]
        self.assertEqual(status.level, b'\x00')
        self.assertIn('27.0 ℃', status.message)

    def test_serial_failure_stops_the_node(self):
        self.node.serial.read.side_effect = serial.SerialException('device disconnected')
        with self.assertRaises(serial.SerialException):
            self.node.poll()
        self.assertEqual(self.node.raw_pub.messages, [])

    def test_chip_intervals_are_mapped_into_ros_clock(self):
        self.node.byte_time = 10/115200
        with patch.object(self.node, 'get_clock') as clock:
            clock.return_value.now.return_value.nanoseconds = 100_000_000_000
            self.node.feed(time_frame(0)+CAPTURED_CYCLE+time_frame(20)+CAPTURED_CYCLE, 10)
        stamps = [m.header.stamp.sec + m.header.stamp.nanosec/1e9 for m in self.node.raw_pub.messages]
        self.assertEqual(len(stamps), 2)
        self.assertAlmostEqual(stamps[1]-stamps[0], .02)
        self.assertTrue(all(99.9 < t <= 100 for t in stamps))

    def test_time_diagnostics_remain_valid_after_partial_cycle(self):
        self.node.feed(time_frame(0)+CAPTURED_CYCLE+time_frame(100), time.monotonic())
        self.node.diagnostics()
        status = self.node.diag_pub.messages[-1].status[0]
        values = {v.key: v.value for v in status.values}
        self.assertEqual(status.level, b'\x00')
        self.assertEqual(values['timestamp_source'], 'chip_time')
        self.assertEqual(values['timestamp_dropped_frames'], '0')

    def test_ros_clock_restart_does_not_create_negative_stamps(self):
        with patch.object(self.node, 'get_clock') as clock:
            clock.return_value.now.return_value.nanoseconds = 0
            self.node.feed(time_frame(0)+CAPTURED_CYCLE, 10)
            self.assertEqual(self.node.raw_pub.messages, [])
            clock.return_value.now.return_value.nanoseconds = 100_000_000
            self.node.feed(time_frame(100)+CAPTURED_CYCLE, 10.1)
            self.assertEqual(len(self.node.raw_pub.messages), 1)


if __name__ == '__main__':
    unittest.main()
