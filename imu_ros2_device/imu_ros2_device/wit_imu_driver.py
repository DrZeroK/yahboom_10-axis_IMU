"""Yahboom 10-axis IMU (WitMotion protocol) -> /imu/data_raw, /imu/data (onboard fusion), /imu/mag."""
import time

import rclpy
import serial
from rclpy.executors import ExternalShutdownException
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.time import Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Header

from .wit_protocol import ACC, ANGLE, PRESS, FrameClock, FrameParser, SampleAssembler


class WitImuDriver(Node):
    def __init__(self):
        super().__init__('imu')
        defaults = {
            'port': '/dev/imu', 'baud': 9600, 'frame_id': 'imu_link',
            # Each rclpy wakeup costs ~2.8 ms CPU on the X5: 50 Hz polling took 17.5% of a core, 20 Hz 9.2%.
            # Enable chip TIME output before batching multiple cycles at higher sensor rates.
            'poll_period_sec': .05, 'stale_timeout_sec': .5,
            # Vendor nominal values (CMP10A manual), not measured on this wheelchair.
            'mag_tesla_per_lsb': 6.67e-9, 'orientation_stddev_rad': .0087, 'yaw_stddev_rad': .0175,
        }
        self.cfg = {name: self.declare_parameter(name, value).value for name, value in defaults.items()}
        c = self.cfg
        if not (c['baud'] > 0 and 0 < c['poll_period_sec'] < c['stale_timeout_sec']):
            raise ValueError('Invalid IMU configuration')
        self.byte_time = 10.0 / c['baud']  # 8N1: start + 8 data + stop bits
        self.parser = FrameParser()
        self.frame_clock = FrameClock(c['stale_timeout_sec'])
        self.assembler = SampleAssembler(c['stale_timeout_sec'])
        self.raw_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.data_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.mag_pub = self.create_publisher(MagneticField, 'imu/mag', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.bytes_received = 0
        self.raw_count = 0
        self.last_raw = None
        self.window = (time.monotonic(), 0)
        self.serial = serial.Serial(c['port'], c['baud'], timeout=0)
        self.create_timer(c['poll_period_sec'], self.poll)
        self.create_timer(1.0, self.diagnostics)
        self.get_logger().info(f"WitMotion IMU {c['port']} @ {c['baud']} -> imu/data_raw, imu/data, imu/mag")

    def poll(self):
        # A disconnected adapter raises SerialException: the node exits and launch respawns it.
        data = self.serial.read(4096)
        if data:
            self.feed(data, time.monotonic())

    def feed(self, data, now):
        self.bytes_received += len(data)
        ros_now = self.get_clock().now().nanoseconds
        frames = self.frame_clock.feed(self.parser.feed(data, now, self.byte_time), now)
        for kind, payload, t in frames:
            for sample, sample_time, values in self.assembler.push(kind, payload, t):
                stamp_ns = ros_now - int((now - sample_time) * 1e9)
                if stamp_ns < 0:  # No valid ROS stamp before the current clock epoch.
                    continue
                stamp = Time(nanoseconds=stamp_ns).to_msg()
                getattr(self, 'publish_' + sample)(Header(stamp=stamp, frame_id=self.cfg['frame_id']), values)

    def imu_message(self, header, values):
        msg = Imu(header=header)
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = values['accel']
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = values['gyro']
        return msg

    def publish_raw(self, header, values):
        msg = self.imu_message(header, values)
        msg.orientation_covariance[0] = -1.0  # REP-145: no orientation in data_raw
        self.raw_pub.publish(msg)
        self.raw_count += 1
        self.last_raw = time.monotonic()

    def publish_data(self, header, values):
        msg = self.imu_message(header, values)
        q = msg.orientation
        q.x, q.y, q.z, q.w = values['orientation']
        tilt, yaw = self.cfg['orientation_stddev_rad'] ** 2, self.cfg['yaw_stddev_rad'] ** 2
        msg.orientation_covariance = [tilt, 0.0, 0.0, 0.0, tilt, 0.0, 0.0, 0.0, yaw]
        self.data_pub.publish(msg)

    def publish_mag(self, header, values):
        msg = MagneticField(header=header)
        scale = self.cfg['mag_tesla_per_lsb']
        f = msg.magnetic_field
        f.x, f.y, f.z = (x * scale for x in values['mag_lsb'])
        self.mag_pub.publish(msg)

    def diagnostics(self):
        now = time.monotonic()
        start, count = self.window
        rate = (self.raw_count - count) / max(now - start, 1e-3)
        self.window = (now, self.raw_count)
        acc, angle = (self.assembler.latest(k) for k in (ACC, ANGLE))
        press = self.assembler.latest(PRESS, now)
        if self.last_raw is not None and now - self.last_raw <= self.cfg['stale_timeout_sec']:
            level = 0
            message = f'{rate:.1f} Hz · {acc["temperature"]:.1f} ℃ · 校验错误 {self.parser.bad_checksums}'
        elif self.bytes_received:
            level, message = 2, '收到串口数据但没有有效的加速度/角速度帧：检查波特率与输出内容'
        else:
            level, message = 2, 'IMU 无数据：检查 /dev/imu 连线'
        if level:
            self.get_logger().warning(message, throttle_duration_sec=10.0)
        values = {'port': self.cfg['port'], 'baud': self.cfg['baud'], 'rate_hz': round(rate, 1),
                  'bytes_received': self.bytes_received, 'checksum_errors': self.parser.bad_checksums,
                  'timestamp_source': 'chip_time' if self.frame_clock.seen_time else 'serial_estimate',
                  'timestamp_resets': self.frame_clock.resets,
                  'timestamp_dropped_frames': self.frame_clock.dropped_frames,
                  'firmware_version': angle['version'] if angle else None,
                  'temperature_c': acc['temperature'] if acc else None,
                  'pressure_pa': press['pressure'] if press else None,
                  'baro_height_m': press['height'] if press else None,
                  'pressure_age_s': self.assembler.age(PRESS, now) if press else None}
        # DiagnosticStatus.level is ROS 'byte', represented as bytes in rclpy.
        status = DiagnosticStatus(level=bytes([level]), name='IMU', hardware_id='x5-nav:wit-imu',
                                  message=message,
                                  values=[KeyValue(key=k, value=str(v)) for k, v in values.items()])
        self.diag_pub.publish(DiagnosticArray(
            header=Header(stamp=self.get_clock().now().to_msg()), status=[status]))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WitImuDriver()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except serial.SerialException as exc:
        get_logger('imu').error(f'IMU 串口不可用，退出等待重启：{exc}')
        raise SystemExit(1)
    finally:
        try:
            if node is not None:
                node.serial.close()
                node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            # Ctrl-C reaches the node from the terminal and again from ros2 launch; the second can land here.
            pass


if __name__ == '__main__':
    main()
