// 亚博 10 轴 IMU（WitMotion 协议）→ /imu/data_raw、/imu/data（模块自带融合）、/imu/mag、/diagnostics。
// 与 imu_ros2_device/wit_imu_driver.py 的参数、话题、单位、协方差和诊断字段一致。
#pragma once

#include <functional>
#include <optional>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

#include "x5_wit_imu/wit_protocol.hpp"

namespace x5_wit_imu {

class WitImuNode : public rclcpp::Node {
 public:
  explicit WitImuNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~WitImuNode() override;

  // 串口断开时抛 std::runtime_error：节点退出，由 launch 重启
  void poll();
  void feed(const uint8_t *data, size_t len, double now);
  void feed(const uint8_t *data, size_t len, double now, int64_t ros_now_ns);
  void diagnostics(double now);
  static double monotonic();

  // 发布出口；测试替换成收集函数，和 Python 测试里的 Capture 一样
  std::function<void(const sensor_msgs::msg::Imu &)> raw_out, data_out;
  std::function<void(const sensor_msgs::msg::MagneticField &)> mag_out;
  std::function<void(const diagnostic_msgs::msg::DiagnosticArray &)> diag_out;
  double byte_time;

 private:
  void open_serial();
  sensor_msgs::msg::Imu imu_message(const std_msgs::msg::Header &header, const Values &values) const;

  std::string port_, frame_id_;
  int baud_;
  double poll_period_, stale_timeout_, mag_scale_, tilt_stddev_, yaw_stddev_;
  int fd_ = -1;
  FrameParser parser_;
  FrameClock frame_clock_;
  SampleAssembler assembler_;
  uint64_t bytes_received_ = 0, raw_count_ = 0;
  std::optional<double> last_raw_;
  double window_start_;
  uint64_t window_count_ = 0;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr raw_pub_, data_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr poll_timer_, diag_timer_;
};

}  // namespace x5_wit_imu
