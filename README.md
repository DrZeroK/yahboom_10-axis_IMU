# 亚博 10 轴 IMU（WitMotion 协议）ROS 2 驱动

亚博 10 轴 IMU（CMP10A，WitMotion 标准串口协议，出厂 9600 8N1、10 Hz）的 ROS 2 Humble 驱动。

本提交是 Python 版驱动 `imu_ros2_device`，作为后续 C++ 改写的基线。

- 话题：`imu/data_raw`（REP-145，不带姿态）、`imu/data`（模块自带融合的姿态）、`imu/mag`、`/diagnostics`
- 时间戳：芯片输出 TIME 帧时按芯片时间映射到主机时钟；没有 TIME 帧时按串口字节回推
- 测试：`tests/test_wit_imu.py`（协议部分不需要 ROS）
