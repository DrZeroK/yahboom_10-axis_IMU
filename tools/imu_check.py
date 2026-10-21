"""只读订阅 IMU 话题 N 秒：频率、最大间隔、时间戳延迟/倒退、重力与四元数模长、最后一条 IMU 诊断。

用法：python3 imu_check.py [秒数]
"""
import math
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, MagneticField


def ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
    rclpy.init()
    node = rclpy.create_node('imu_check')
    rows = {'/imu/data_raw': [], '/imu/data': [], '/imu/mag': []}
    diag = []

    def make_cb(topic):
        def cb(msg):
            now = node.get_clock().now().nanoseconds
            extra = None
            if isinstance(msg, Imu):
                a = msg.linear_acceleration
                q = msg.orientation
                extra = (math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z), math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w),
                         msg.orientation_covariance[0])
            rows[topic].append((time.monotonic(), ns(msg.header.stamp), now, msg.header.frame_id, extra))
        return cb

    node.create_subscription(Imu, '/imu/data_raw', make_cb('/imu/data_raw'), qos_profile_sensor_data)
    node.create_subscription(Imu, '/imu/data', make_cb('/imu/data'), qos_profile_sensor_data)
    node.create_subscription(MagneticField, '/imu/mag', make_cb('/imu/mag'), qos_profile_sensor_data)
    node.create_subscription(DiagnosticArray, '/diagnostics',
                             lambda m: diag.extend(s for s in m.status if s.name == 'IMU'), 10)
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    for topic, r in rows.items():
        if len(r) < 2:
            print(f'{topic}: 只收到 {len(r)} 条')
            continue
        gaps = [b[0] - a[0] for a, b in zip(r, r[1:])]
        back = sum(1 for a, b in zip(r, r[1:]) if b[1] <= a[1])
        lag = [(x[2] - x[1]) / 1e6 for x in r]
        line = (f'{topic}: {len(r)} 条 {(len(r) - 1) / (r[-1][0] - r[0][0]):.2f} Hz，最大间隔 {max(gaps) * 1000:.0f} ms；'
                f'时间戳延迟 {sum(lag) / len(lag):.1f} ms（{min(lag):.1f}–{max(lag):.1f}），不递增 {back} 次；frame {r[-1][3]}')
        if r[-1][4]:
            g = [x[4][0] for x in r]
            line += f'；|a| {sum(g) / len(g):.3f} m/s²；|q| {r[-1][4][1]:.4f}；orientation_cov[0] {r[-1][4][2]:g}'
        print(line)
    if diag:
        s = diag[-1]
        level = s.level[0] if isinstance(s.level, (bytes, bytearray)) else int(s.level)
        values = {v.key: v.value for v in s.values}
        print(f'诊断：共 {len(diag)} 条，最后一条 level {level}，{s.message}，hardware_id {s.hardware_id}')
        print('  ' + '，'.join(f'{k}={values.get(k)}' for k in (
            'rate_hz', 'bytes_received', 'checksum_errors', 'timestamp_source', 'timestamp_resets',
            'timestamp_dropped_frames', 'firmware_version', 'temperature_c', 'pressure_pa', 'pressure_age_s')))
    else:
        print('诊断：没收到 IMU 诊断')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
