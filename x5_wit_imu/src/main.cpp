#include <memory>
#include <stdexcept>

#include "x5_wit_imu/wit_imu_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int rc = 0;
  try {
    rclcpp::spin(std::make_shared<x5_wit_imu::WitImuNode>());
  } catch (const std::runtime_error &e) {
    // 串口打不开或断开：退出码 1，launch 按 respawn 重启（与 Python 版相同）
    RCLCPP_ERROR(rclcpp::get_logger("imu"), "IMU 串口不可用，退出等待重启：%s", e.what());
    rc = 1;
  } catch (const std::invalid_argument &e) {
    RCLCPP_ERROR(rclcpp::get_logger("imu"), "IMU 参数错误：%s", e.what());
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
