# 亚博 10 轴 IMU（WitMotion 协议）ROS 2 驱动

亚博 10 轴 IMU（CMP10A，WitMotion 标准串口协议，出厂 9600 8N1、10 Hz）的 ROS 2 Humble 驱动，有两个可互换的实现：

| 包 | 语言 | 说明 |
|---|---|---|
| `x5_wit_imu` | C++（rclcpp） | **推荐**。在 RDK X5 上实测 CPU 1.3%、RSS 12 MB |
| `imu_ros2_device` | Python（rclpy） | 原版，作为对拍基准保留。同条件下 CPU 10.4%、RSS 50 MB |

两者节点名、参数、话题、单位、协方差和诊断字段都相同。C++ 版的协议层由差分对拍保证和 Python 版逐帧一致。改写的原因、验证方法和测量数据见 [docs/开发过程.md](docs/开发过程.md)。

## 话题

| 话题 | 类型 | 内容 |
|---|---|---|
| `imu/data_raw` | `sensor_msgs/Imu` | 加速度 m/s²、角速度 rad/s；按 REP-145 不带姿态（`orientation_covariance[0] = -1`） |
| `imu/data` | `sensor_msgs/Imu` | 另加模块自带融合的姿态；模块输出四元数帧时优先用四元数，否则用欧拉角帧换算 |
| `imu/mag` | `sensor_msgs/MagneticField` | 磁场 T（按厂商标称系数换算） |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 名称 `IMU`：频率、温度、校验错误、时间戳来源、气压与气压高度等 |

时间戳：模块输出 TIME 帧时，按芯片时间映射到主机时钟，同一输出周期的帧共用一个采样时间；没有 TIME 帧时，按串口字节传输时间回推。

## 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `port` | `/dev/imu` | 串口 |
| `baud` | `9600` | 波特率 |
| `frame_id` | `imu_link` | |
| `poll_period_sec` | `0.05` | 轮询周期 |
| `stale_timeout_sec` | `0.5` | 超过这个时间的值不再配对；诊断判断数据是否中断 |
| `mag_tesla_per_lsb` | `6.67e-9` | 厂商标称值 |
| `orientation_stddev_rad` / `yaw_stddev_rad` | `0.0087` / `0.0175` | 姿态协方差 |

## 编译与运行

```bash
cd ~/ros2_ws/src && git clone https://github.com/DrZeroK/yahboom_10-axis_IMU.git
cd ~/ros2_ws && colcon build --symlink-install --packages-select x5_wit_imu
source install/setup.bash
ros2 launch x5_wit_imu imu.launch.py port:=/dev/ttyUSB0
```

launch 会等串口设备出现后再启动节点；串口断开时节点以退出码 1 结束，3 s 后自动重启。

## 测试

```bash
# 协议层与对拍，不需要 ROS（Mac/Linux 都行）
clang++ -std=c++17 -Ix5_wit_imu/include x5_wit_imu/test/test_wit_protocol.cpp -o /tmp/test_wit_protocol && /tmp/test_wit_protocol
clang++ -std=c++17 -O2 -Ix5_wit_imu/include x5_wit_imu/test/replay.cpp -o /tmp/wit_replay
python3 x5_wit_imu/test/replay_compare.py /tmp/wit_replay 300

# colcon 编译后（含节点测试，用 pty 假串口，ROS 域 199）
python3 -m unittest discover -s x5_wit_imu/test -p 'test_*.py'

# Python 版
PYTHONPATH=imu_ros2_device python3 -m unittest tests.test_wit_imu
```

## 工具

- `tools/imu_check.py`：只读订阅 IMU 话题 N 秒，输出频率、最大间隔、时间戳延迟/倒退、加速度与四元数模长、最后一条诊断。
- `tools/wit_imu_pty_feed.py` 与 `tools/bench_cpu.sh`：用 pty 假 IMU 比较两个驱动的 CPU。

## 许可

MIT，见 [LICENSE](LICENSE)。
