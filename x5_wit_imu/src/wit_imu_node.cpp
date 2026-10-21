#include "x5_wit_imu/wit_imu_node.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace x5_wit_imu {
namespace {

// 诊断值按 Python str() 的样子输出（'27.03'、'100000.0'、'None'），监测端按浮点解析
std::string py_float(double v) {
  char buf[400];
  // Python repr：1e-4 ≤ |v| < 1e16 用定点，其余用科学计数；两种都取能还原的最短位数
  const double a = std::fabs(v);
  const bool fixed = a == 0 || (a >= 1e-4 && a < 1e16);
  const auto r = fixed ? std::to_chars(buf, buf + sizeof buf, v, std::chars_format::fixed)
                       : std::to_chars(buf, buf + sizeof buf, v, std::chars_format::scientific);
  std::string s(buf, r.ptr);
  if (s.find_first_of(".eEn") == std::string::npos) s += ".0";
  return s;
}

speed_t baud_constant(int baud) {
  switch (baud) {
    case 4800: return B4800;
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
#ifdef B460800
    case 460800: return B460800;
#endif
#ifdef B921600
    case 921600: return B921600;
#endif
    default: throw std::invalid_argument("不支持的波特率 " + std::to_string(baud));
  }
}

}  // namespace

double WitImuNode::monotonic() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

WitImuNode::WitImuNode(const rclcpp::NodeOptions &options)
    : Node("imu", options),
      port_(declare_parameter<std::string>("port", "/dev/imu")),
      frame_id_(declare_parameter<std::string>("frame_id", "imu_link")),
      baud_(int(declare_parameter<int64_t>("baud", 9600))),
      // Python 版每次 rclpy 唤醒约 2.8 ms：20 Hz 占 9%。rclcpp 唤醒便宜得多，但周期保持一致以免改变时间戳行为
      poll_period_(declare_parameter<double>("poll_period_sec", .05)),
      stale_timeout_(declare_parameter<double>("stale_timeout_sec", .5)),
      // 厂商标称值（CMP10A 手册），未在本车实测
      mag_scale_(declare_parameter<double>("mag_tesla_per_lsb", 6.67e-9)),
      tilt_stddev_(declare_parameter<double>("orientation_stddev_rad", .0087)),
      yaw_stddev_(declare_parameter<double>("yaw_stddev_rad", .0175)),
      frame_clock_(stale_timeout_),
      assembler_(stale_timeout_) {
  if (!(baud_ > 0 && 0 < poll_period_ && poll_period_ < stale_timeout_))
    throw std::invalid_argument("Invalid IMU configuration");
  byte_time = 10.0 / baud_;  // 8N1：起始位 + 8 数据位 + 停止位
  raw_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
  data_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
  mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 10);
  diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  raw_out = [this](const sensor_msgs::msg::Imu &m) { raw_pub_->publish(m); };
  data_out = [this](const sensor_msgs::msg::Imu &m) { data_pub_->publish(m); };
  mag_out = [this](const sensor_msgs::msg::MagneticField &m) { mag_pub_->publish(m); };
  diag_out = [this](const diagnostic_msgs::msg::DiagnosticArray &m) { diag_pub_->publish(m); };
  window_start_ = monotonic();
  open_serial();
  poll_timer_ = create_wall_timer(std::chrono::duration<double>(poll_period_), [this] { poll(); });
  diag_timer_ = create_wall_timer(std::chrono::seconds(1), [this] { diagnostics(monotonic()); });
  RCLCPP_INFO(get_logger(), "WitMotion IMU %s @ %d -> imu/data_raw, imu/data, imu/mag", port_.c_str(), baud_);
}

WitImuNode::~WitImuNode() {
  if (fd_ >= 0) close(fd_);
}

void WitImuNode::open_serial() {
  // 与 pyserial(timeout=0) 一致：8N1、原始模式、无流控、非阻塞，打开后清空输入缓冲
  const speed_t speed = baud_constant(baud_);
  fd_ = open(port_.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) throw std::runtime_error("打不开 " + port_ + "：" + std::strerror(errno));
  termios tio{};
  if (tcgetattr(fd_, &tio) != 0) throw std::runtime_error(port_ + " 不是串口：" + std::strerror(errno));
  cfmakeraw(&tio);
  tio.c_cflag |= CLOCAL | CREAD;
  tio.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
  tio.c_iflag &= ~(IXON | IXOFF | IXANY);
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  cfsetispeed(&tio, speed);
  cfsetospeed(&tio, speed);
  if (tcsetattr(fd_, TCSANOW, &tio) != 0) throw std::runtime_error("设置 " + port_ + " 失败：" + std::strerror(errno));
  tcflush(fd_, TCIFLUSH);
}

void WitImuNode::poll() {
  pollfd p{fd_, POLLIN, 0};
  if (::poll(&p, 1, 0) < 0) {
    if (errno == EINTR) return;
    throw std::runtime_error(std::string("poll 失败：") + std::strerror(errno));
  }
  if (p.revents & (POLLERR | POLLHUP | POLLNVAL)) throw std::runtime_error("串口断开（" + port_ + "）");
  if (!(p.revents & POLLIN)) return;
  uint8_t buf[4096];
  const ssize_t n = read(fd_, buf, sizeof buf);
  if (n < 0 && (errno == EAGAIN || errno == EINTR)) return;
  // 可读却读到 0 字节：设备已拔出（pyserial 在这里同样报错）
  if (n <= 0) throw std::runtime_error("串口读取失败（" + port_ + "）：" + (n < 0 ? std::strerror(errno) : "设备无数据"));
  feed(buf, size_t(n), monotonic());
}

void WitImuNode::feed(const uint8_t *data, size_t len, double now) {
  feed(data, len, now, get_clock()->now().nanoseconds());
}

void WitImuNode::feed(const uint8_t *data, size_t len, double now, int64_t ros_now_ns) {
  bytes_received_ += len;
  for (const auto &f : frame_clock_.feed(parser_.feed(data, len, now, byte_time), now)) {
    for (const auto &s : assembler_.push(f.kind, f.payload, f.time)) {
      const int64_t stamp_ns = ros_now_ns - int64_t((now - s.time) * 1e9);
      if (stamp_ns < 0) continue;  // 当前时钟纪元之前没有合法的 ROS 时间戳
      std_msgs::msg::Header header;
      header.stamp = rclcpp::Time(stamp_ns);
      header.frame_id = frame_id_;
      if (s.kind == SampleKind::Raw) {
        auto msg = imu_message(header, s.values);
        msg.orientation_covariance[0] = -1.0;  // REP-145：data_raw 不带姿态
        raw_out(msg);
        ++raw_count_;
        last_raw_ = monotonic();
      } else if (s.kind == SampleKind::Data) {
        auto msg = imu_message(header, s.values);
        const auto &q = s.values.orientation;
        msg.orientation.x = q[0];
        msg.orientation.y = q[1];
        msg.orientation.z = q[2];
        msg.orientation.w = q[3];
        const double tilt = tilt_stddev_ * tilt_stddev_, yaw = yaw_stddev_ * yaw_stddev_;
        msg.orientation_covariance = {tilt, 0.0, 0.0, 0.0, tilt, 0.0, 0.0, 0.0, yaw};
        data_out(msg);
      } else {
        sensor_msgs::msg::MagneticField msg;
        msg.header = header;
        msg.magnetic_field.x = s.values.mag_lsb[0] * mag_scale_;
        msg.magnetic_field.y = s.values.mag_lsb[1] * mag_scale_;
        msg.magnetic_field.z = s.values.mag_lsb[2] * mag_scale_;
        mag_out(msg);
      }
    }
  }
}

sensor_msgs::msg::Imu WitImuNode::imu_message(const std_msgs::msg::Header &header, const Values &v) const {
  sensor_msgs::msg::Imu msg;
  msg.header = header;
  msg.linear_acceleration.x = v.accel[0];
  msg.linear_acceleration.y = v.accel[1];
  msg.linear_acceleration.z = v.accel[2];
  msg.angular_velocity.x = v.gyro[0];
  msg.angular_velocity.y = v.gyro[1];
  msg.angular_velocity.z = v.gyro[2];
  return msg;
}

void WitImuNode::diagnostics(double now) {
  const double rate = double(raw_count_ - window_count_) / std::max(now - window_start_, 1e-3);
  window_start_ = now;
  window_count_ = raw_count_;
  const Values *acc = assembler_.latest(ACC), *angle = assembler_.latest(ANGLE), *press = assembler_.latest(PRESS, now);
  uint8_t level;
  std::string message;
  char text[160];
  if (last_raw_ && now - *last_raw_ <= stale_timeout_ && acc) {
    level = 0;
    std::snprintf(text, sizeof text, "%.1f Hz · %.1f ℃ · 校验错误 %llu", rate, acc->temperature,
                  (unsigned long long)parser_.bad_checksums);
    message = text;
  } else if (bytes_received_) {
    level = 2;
    message = "收到串口数据但没有有效的加速度/角速度帧：检查波特率与输出内容";
  } else {
    level = 2;
    message = "IMU 无数据：检查 /dev/imu 连线";
  }
  if (level) RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "%s", message.c_str());
  std::snprintf(text, sizeof text, "%.1f", rate);
  const auto opt = [](const Values *v, auto field) { return v ? field(*v) : std::string("None"); };
  const std::vector<std::pair<std::string, std::string>> values = {
      {"port", port_},
      {"baud", std::to_string(baud_)},
      {"rate_hz", text},
      {"bytes_received", std::to_string(bytes_received_)},
      {"checksum_errors", std::to_string(parser_.bad_checksums)},
      {"timestamp_source", frame_clock_.seen_time ? "chip_time" : "serial_estimate"},
      {"timestamp_resets", std::to_string(frame_clock_.resets)},
      {"timestamp_dropped_frames", std::to_string(frame_clock_.dropped_frames)},
      {"firmware_version", opt(angle, [](const Values &v) { return std::to_string(v.version); })},
      {"temperature_c", opt(acc, [](const Values &v) { return py_float(v.temperature); })},
      {"pressure_pa", opt(press, [](const Values &v) { return py_float(v.pressure); })},
      {"baro_height_m", opt(press, [](const Values &v) { return py_float(v.height); })},
      {"pressure_age_s", press ? py_float(*assembler_.age(PRESS, now)) : "None"},
  };
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = level;
  status.name = "IMU";
  status.hardware_id = "x5-nav:wit-imu";
  status.message = message;
  for (const auto &[k, v] : values) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = k;
    kv.value = v;
    status.values.push_back(kv);
  }
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = get_clock()->now();
  array.status.push_back(status);
  diag_out(array);
}

}  // namespace x5_wit_imu
