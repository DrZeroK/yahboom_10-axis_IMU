// 节点测试，对应 tests/test_wit_imu.py 的 NodeTests：用 pty 假串口，输出被收集、不发出去（ROS 域 199）。
#include <fcntl.h>
#include <pty.h>
#include <unistd.h>

#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>

#include "test_frames.hpp"
#include "x5_wit_imu/wit_imu_node.hpp"

using diagnostic_msgs::msg::DiagnosticArray;
using sensor_msgs::msg::Imu;
using sensor_msgs::msg::MagneticField;

struct Fixture {
  int master = -1, slave = -1;
  std::shared_ptr<x5_wit_imu::WitImuNode> node;
  std::vector<Imu> raw, data;
  std::vector<MagneticField> mag;
  std::vector<DiagnosticArray> diag;

  Fixture() {
    char name[128];
    if (openpty(&master, &slave, name, nullptr, nullptr) != 0) std::abort();
    rclcpp::NodeOptions options;
    options.parameter_overrides({{"port", std::string(name)}});
    node = std::make_shared<x5_wit_imu::WitImuNode>(options);
    node->raw_out = [this](const Imu &m) { raw.push_back(m); };
    node->data_out = [this](const Imu &m) { data.push_back(m); };
    node->mag_out = [this](const MagneticField &m) { mag.push_back(m); };
    node->diag_out = [this](const DiagnosticArray &m) { diag.push_back(m); };
  }
  ~Fixture() {
    node.reset();
    if (master >= 0) close(master);
    if (slave >= 0) close(slave);
  }
  void feed(const Bytes &s, double now) { node->feed(s.data(), s.size(), now); }
  void feed(const Bytes &s, double now, int64_t ros_ns) { node->feed(s.data(), s.size(), now, ros_ns); }
  std::map<std::string, std::string> values() const {
    std::map<std::string, std::string> out;
    for (const auto &kv : diag.back().status[0].values) out[kv.key] = kv.value;
    return out;
  }
};

static int64_t ns(const builtin_interfaces::msg::Time &t) { return int64_t(t.sec) * 1000000000 + t.nanosec; }

static void messages_follow_rep145_and_si_units() {
  Fixture f;
  f.feed(CYCLE, x5_wit_imu::WitImuNode::monotonic());
  CHECK(f.raw.size() == 1 && f.data.size() == 1 && f.mag.size() == 1);
  if (f.raw.empty() || f.data.empty() || f.mag.empty()) return;
  CHECK(f.raw.back().header.frame_id == "imu_link");
  CHECK(f.raw.back().orientation_covariance[0] == -1.0);
  NEAR(f.raw.back().linear_acceleration.z, 2017 / 32768.0 * 16 * G, 1e-12);
  const auto &q = f.data.back().orientation;
  NEAR(std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w), 1.0, 1e-9);
  CHECK(f.data.back().orientation_covariance[0] > 0);
  CHECK(f.data.back().orientation_covariance[8] > f.data.back().orientation_covariance[0]);
  NEAR(f.mag.back().magnetic_field.z, -4040 * 6.67e-9, 1e-15);
}

static void stamps_follow_frame_order() {
  Fixture f;
  f.feed(CYCLE, x5_wit_imu::WitImuNode::monotonic());
  if (f.raw.empty() || f.data.empty() || f.mag.empty()) {
    CHECK(false);
    return;
  }
  const int64_t a = ns(f.raw.back().header.stamp), b = ns(f.data.back().header.stamp), c = ns(f.mag.back().header.stamp);
  CHECK(a <= b && b <= c);
  // 磁场帧在本次读取最后；data_raw 早结束两帧（9600 波特下 22 字节）
  NEAR((c - a) / 1e9, 22 * 10 / 9600.0, 1e-4);
}

static void diagnostics_distinguish_silence_wrong_baud_and_ok() {
  Fixture f;
  const double now = x5_wit_imu::WitImuNode::monotonic();
  f.node->diagnostics(now);
  CHECK(f.diag.back().status[0].level == 2);
  Bytes noise;
  for (int k = 0; k < 2; ++k)
    for (int i = 0; i < 256; ++i) noise.push_back(uint8_t(i));
  f.feed(noise, now);
  f.node->diagnostics(now);
  CHECK(f.diag.back().status[0].message.find("波特率") != std::string::npos);
  f.feed(CYCLE, x5_wit_imu::WitImuNode::monotonic());
  f.node->diagnostics(x5_wit_imu::WitImuNode::monotonic());
  const auto &status = f.diag.back().status[0];
  CHECK(status.level == 0);
  CHECK(status.message.find("27.0 ℃") != std::string::npos);
  CHECK(status.name == "IMU" && status.hardware_id == "x5-nav:wit-imu");
  CHECK(f.values().at("temperature_c") == "27.03");
  CHECK(f.values().at("firmware_version") == "18161");
  CHECK(f.values().at("pressure_pa") == "None");
}

static void serial_failure_stops_the_node() {
  Fixture f;
  close(f.master);  // 拔掉“USB”：从端收到 HUP
  f.master = -1;
  bool threw = false;
  try {
    f.node->poll();
  } catch (const std::runtime_error &) {
    threw = true;
  }
  CHECK(threw);
  CHECK(f.raw.empty());
}

static void poll_reads_from_the_port() {
  Fixture f;
  if (write(f.master, CYCLE.data(), CYCLE.size()) != ssize_t(CYCLE.size())) {
    CHECK(false);
    return;
  }
  usleep(20000);
  f.node->poll();
  CHECK(f.raw.size() == 1 && f.data.size() == 1 && f.mag.size() == 1);
}

static void chip_intervals_are_mapped_into_ros_clock() {
  Fixture f;
  f.node->byte_time = 10.0 / 115200;
  f.feed(time_frame(0) + CYCLE + time_frame(20) + CYCLE, 10, 100000000000LL);
  CHECK(f.raw.size() == 2);
  if (f.raw.size() != 2) return;
  const double a = ns(f.raw[0].header.stamp) / 1e9, b = ns(f.raw[1].header.stamp) / 1e9;
  NEAR(b - a, .02, 1e-6);
  CHECK(99.9 < a && b <= 100);
}

static void time_diagnostics_remain_valid_after_partial_cycle() {
  Fixture f;
  f.feed(time_frame(0) + CYCLE + time_frame(100), x5_wit_imu::WitImuNode::monotonic());
  f.node->diagnostics(x5_wit_imu::WitImuNode::monotonic());
  CHECK(f.diag.back().status[0].level == 0);
  CHECK(f.values().at("timestamp_source") == "chip_time");
  CHECK(f.values().at("timestamp_dropped_frames") == "0");
}

static void ros_clock_restart_does_not_create_negative_stamps() {
  Fixture f;
  f.feed(time_frame(0) + CYCLE, 10, 0);
  CHECK(f.raw.empty());
  f.feed(time_frame(100) + CYCLE, 10.1, 100000000);
  CHECK(f.raw.size() == 1);
}

static void pressure_diagnostics_include_height_age_and_expire() {
  Fixture f;
  Bytes press = {0x55, PRESS};
  const int32_t pressure = 100000, height = 1234;
  press.resize(10);
  std::memcpy(press.data() + 2, &pressure, 4);
  std::memcpy(press.data() + 6, &height, 4);
  uint8_t sum = 0;
  for (uint8_t x : press) sum = uint8_t(sum + x);
  press.push_back(sum);
  const double now = x5_wit_imu::WitImuNode::monotonic();
  f.feed(press, now);
  f.node->diagnostics(now + .1);
  CHECK(f.values().at("pressure_pa") == "100000.0");
  CHECK(f.values().at("baro_height_m") == "12.34");
  NEAR(std::stod(f.values().at("pressure_age_s")), .1, 1e-9);
  f.node->diagnostics(now + 1);
  CHECK(f.values().at("pressure_pa") == "None");
  CHECK(f.values().at("pressure_age_s") == "None");
}

int main(int argc, char **argv) {
  setenv("ROS_DOMAIN_ID", "199", 1);  // 隔离域；输出被收集，不会发出
  rclcpp::init(argc, argv);
  messages_follow_rep145_and_si_units();
  stamps_follow_frame_order();
  diagnostics_distinguish_silence_wrong_baud_and_ok();
  serial_failure_stops_the_node();
  poll_reads_from_the_port();
  chip_intervals_are_mapped_into_ros_clock();
  time_diagnostics_remain_valid_after_partial_cycle();
  ros_clock_restart_does_not_create_negative_stamps();
  pressure_diagnostics_include_height_age_and_expire();
  rclcpp::shutdown();
  std::printf(failures ? "test_wit_imu_node: %d 项失败\n" : "test_wit_imu_node: 9 组全部通过\n", failures);
  return failures ? 1 : 0;
}
