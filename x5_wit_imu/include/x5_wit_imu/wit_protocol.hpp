// WitMotion 标准串口协议（亚博 10 轴 IMU CMP10A）：0x55 帧 → 国际单位样本，不依赖 ROS。
// 逐段对应 imu_ros2_device/wit_protocol.py；两边行为由 test/replay_compare.py 对拍。
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>
#include <limits>
#include <map>
#include <optional>
#include <utility>
#include <vector>

namespace x5_wit_imu {

constexpr double G = 9.80665;
constexpr double PI = 3.14159265358979323846;
constexpr uint8_t TIME = 0x50, ACC = 0x51, GYRO = 0x52, ANGLE = 0x53, MAG = 0x54, PRESS = 0x56, QUAT = 0x59;
constexpr size_t FRAME_LEN = 11;

using Payload = std::array<uint8_t, 8>;

struct Frame {
  uint8_t kind;
  Payload payload;
  double time;  // FrameParser 给的是到达时间估计；FrameClock 之后是采样时间
};

// 字节流 → 校验通过的帧；噪声或丢字节后按 0x55 帧头重新同步。
class FrameParser {
 public:
  uint64_t bad_checksums = 0;

  // time 按读取时刻减去帧后剩余字节的传输时间回推
  std::vector<Frame> feed(const uint8_t *data, size_t len, double read_time, double byte_time) {
    buf_.insert(buf_.end(), data, data + len);
    std::vector<Frame> found;
    const size_t n = buf_.size();
    size_t i = 0;
    while (n - i >= FRAME_LEN) {
      const uint8_t *b = buf_.data() + i;
      if (b[0] != 0x55 || b[1] < 0x50 || b[1] > 0x5F) {
        ++i;
        continue;
      }
      uint8_t sum = 0;
      for (size_t k = 0; k < 10; ++k) sum = uint8_t(sum + b[k]);
      if (sum != b[10]) {
        ++bad_checksums;
        ++i;
        continue;
      }
      Frame f{b[1], {}, read_time - double(n - (i + FRAME_LEN)) * byte_time};
      std::copy(b + 2, b + 10, f.payload.begin());
      found.push_back(f);
      i += FRAME_LEN;
    }
    buf_.erase(buf_.begin(), buf_.begin() + long(i));
    return found;
  }

  size_t buffered() const { return buf_.size(); }

 private:
  std::vector<uint8_t> buf_;
};

// 芯片日内时间 → 主机单调时间；没有 TIME 输出的老配置保持按字节回推的时间。
// shortcut: 串口到达时间只给偏移的上界，绝对延迟需要硬件授时标定。
class FrameClock {
 public:
  bool seen_time = false;
  uint64_t resets = 0, dropped_frames = 0;

  explicit FrameClock(double max_gap = .5) : max_gap_(max_gap) { reset(); }

  // 同一个 TIME 周期里的所有传感器帧共用一个采样时间
  std::vector<Frame> feed(const std::vector<Frame> &frames, double now) {
    if (last_read_ && (now < *last_read_ || now - *last_read_ > max_gap_)) {
      reset();
      ++resets;
      if (now < *last_read_) last_stamp_ = -INF;
    }
    std::vector<Frame> out;
    std::vector<Pending> pending;
    for (const auto &f : frames) seen_time = seen_time || f.kind == TIME;
    for (const auto &f : frames) {
      if (f.kind == TIME) {
        cycle_.reset();
        last_kind_ = TIME;
        const int hour = f.payload[3], minute = f.payload[4], second = f.payload[5];
        long ms = long(f.payload[6]) | (long(f.payload[7]) << 8);
        if (hour >= 24 || minute >= 60 || second >= 60 || ms >= 1000) {
          ++dropped_frames;
          continue;
        }
        ms += ((hour * 60L + minute) * 60L + second) * 1000L;
        std::optional<long> delta;
        if (last_ms_) delta = ((ms - *last_ms_) % DAY_MS + DAY_MS) % DAY_MS;  // 跨午夜不算复位
        if (delta && *delta == 0) {
          ++dropped_frames;
          continue;
        }
        if (delta && double(*delta) > max_gap_ * 1000) {
          map(pending, now, out);
          pending.clear();
          reset();
          ++resets;
          delta.reset();
        }
        tick_ += delta ? double(*delta) / 1000 : 0.0;
        last_ms_ = ms;
        cycle_ = tick_;
      } else if (!seen_time) {
        out.push_back(f);
        last_stamp_ = std::max(last_stamp_, f.time);
        continue;
      } else if (f.kind < ACC || f.kind > QUAT) {
        continue;
      } else if (f.kind <= last_kind_) {
        cycle_.reset();  // 没有 TIME 的新周期不能沿用上一周期的时间
      }
      last_kind_ = f.kind;
      if (!cycle_) {
        ++dropped_frames;
      } else {
        pending.push_back({f, *cycle_});
      }
    }
    map(pending, now, out);
    last_read_ = now;
    return out;
  }

 private:
  static constexpr double INF = std::numeric_limits<double>::infinity();
  static constexpr long DAY_MS = 86400000;
  struct Pending {
    Frame frame;
    double tick;
  };

  void reset() {
    last_ms_.reset();
    cycle_.reset();
    mapped_cycle_.reset();
    tick_ = 0.0;
    last_kind_ = TIME;
    stamp_.reset();
    offset_.reset();
    observations_.clear();
  }

  void map(const std::vector<Pending> &pending, double now, std::vector<Frame> &out) {
    if (pending.empty()) return;
    // 串口回推只是到达时间的上界，不是采样间隔
    double bound = INF;
    for (const auto &p : pending) bound = std::min(bound, p.frame.time - p.tick);
    observations_.emplace_back(now, bound);
    while (observations_.front().first < now - 1.0) observations_.pop_front();
    double target = INF;
    for (const auto &o : observations_) target = std::min(target, o.second);
    if (!offset_) {
      offset_ = target;
    } else {
      // 跟随时钟漂移，但不把一次晚到的读取变成采样时间跳变
      const double elapsed = last_read_ ? std::max(0.0, now - *last_read_) : 0.0;
      offset_ = std::min(target, *offset_ + .01 * elapsed);
    }
    for (const auto &p : pending) {
      if (!mapped_cycle_ || p.tick != *mapped_cycle_) {
        mapped_cycle_ = p.tick;
        const double candidate = std::min(p.tick + *offset_, p.frame.time);
        stamp_.reset();
        if (last_stamp_ < candidate) stamp_ = last_stamp_ = candidate;
      }
      if (!stamp_) {
        ++dropped_frames;
      } else {
        out.push_back({p.frame.kind, p.frame.payload, *stamp_});
      }
    }
  }

  double max_gap_;
  std::optional<double> last_read_;
  double last_stamp_ = -INF;
  std::optional<long> last_ms_;
  std::optional<double> cycle_, mapped_cycle_, stamp_, offset_;
  double tick_ = 0.0;
  uint8_t last_kind_ = TIME;
  std::deque<std::pair<double, double>> observations_;
};

// 一帧解出来的值；只有对应帧类型的字段有意义
struct Values {
  std::array<double, 3> accel{}, gyro{}, euler{};
  std::array<double, 4> orientation{0, 0, 0, 1};  // x, y, z, w
  double temperature = 0;
  int version = 0;
  std::array<int, 3> mag_lsb{};
  double pressure = 0, height = 0;
};

// Z-Y-X（yaw, pitch, roll）弧度 → (x, y, z, w)，模块按这个约定输出
inline std::array<double, 4> quaternion_from_euler(double roll, double pitch, double yaw) {
  const double cr = std::cos(roll / 2), sr = std::sin(roll / 2);
  const double cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
  const double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2);
  return {sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
          cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy};
}

inline double radians(double deg) { return deg * (PI / 180.0); }  // 与 Python math.radians 同样的舍入

// 帧数据 → 国际单位（m/s²、rad/s、rad、℃）；驱动不用的类型返回空
inline std::optional<Values> decode(uint8_t kind, const Payload &p) {
  std::array<int16_t, 4> v;
  std::memcpy(v.data(), p.data(), 8);  // 板端与 Mac 都是小端
  Values out;
  if (kind == ACC) {
    for (int k = 0; k < 3; ++k) out.accel[k] = v[k] / 32768.0 * 16 * G;
    out.temperature = v[3] / 100.0;
    return out;
  }
  if (kind == GYRO) {
    for (int k = 0; k < 3; ++k) out.gyro[k] = radians(v[k] / 32768.0 * 2000);
    return out;
  }
  if (kind == ANGLE) {
    for (int k = 0; k < 3; ++k) out.euler[k] = radians(v[k] / 32768.0 * 180);
    out.orientation = quaternion_from_euler(out.euler[0], out.euler[1], out.euler[2]);
    out.version = uint16_t(v[3]);
    return out;
  }
  if (kind == MAG) {
    for (int k = 0; k < 3; ++k) out.mag_lsb[k] = v[k];  // 固件 18161 上其温度字段恒为 0，温度取 ACC 的
    return out;
  }
  if (kind == QUAT) {
    const double norm = std::sqrt(double(v[0]) * v[0] + double(v[1]) * v[1] + double(v[2]) * v[2] + double(v[3]) * v[3]);
    if (norm == 0) return std::nullopt;
    out.orientation = {v[1] / norm, v[2] / norm, v[3] / norm, v[0] / norm};  // 帧里是 w, x, y, z
    return out;
  }
  if (kind == PRESS) {
    int32_t pressure, height_cm;
    std::memcpy(&pressure, p.data(), 4);
    std::memcpy(&height_cm, p.data() + 4, 4);
    out.pressure = double(pressure);
    out.height = height_cm / 100.0;
    return out;
  }
  return std::nullopt;
}

enum class SampleKind { Raw, Data, Mag };

struct Sample {
  SampleKind kind;
  double time;
  Values values;  // Raw: accel, gyro；Data: accel, gyro, orientation；Mag: mag_lsb
};

// 把一个输出周期的帧组合成样本。
// 模块每周期按类型升序发帧（0x51 加速度、0x52 角速度、0x53 角度、0x54 磁场、0x59 四元数），所以角速度帧
// 配上一个角速度帧之后收到的加速度帧，姿态帧配上一个姿态帧之后收到的加速度/角速度帧。帧按到达编号：
// 丢帧时旧值不会被配对两次，超过 max_age 的值也不配对。
class SampleAssembler {
 public:
  explicit SampleAssembler(double max_age) : max_age_(max_age) {}

  std::vector<Sample> push(uint8_t kind, const Payload &payload, double t) {
    if (kind == TIME) {
      cycle_seq_ = seq_;  // 即使丢了部分传感器帧，TIME 也是周期边界
      orientation_seq_ = seq_;
      return {};
    }
    const auto values = decode(kind, payload);
    if (!values) return {};
    ++seq_;
    const auto it = last_.find(kind);
    const uint64_t previous = it == last_.end() ? 0 : it->second.seq;
    last_[kind] = Entry{seq_, t, *values};
    if (kind == GYRO) {
      const Values *acc = since(ACC, std::max(previous, cycle_seq_), t);
      if (!acc) return {};
      Values raw;
      raw.accel = acc->accel;
      raw.gyro = values->gyro;
      return {Sample{SampleKind::Raw, t, raw}};
    }
    if (kind == MAG) return {Sample{SampleKind::Mag, t, *values}};
    if (kind == QUAT || (kind == ANGLE && !since(QUAT, 0, t))) {
      const uint64_t from = orientation_seq_;
      orientation_seq_ = seq_;
      const Values *acc = since(ACC, from, t), *gyro = since(GYRO, from, t);
      if (acc && gyro) {
        Values data;
        data.accel = acc->accel;
        data.gyro = gyro->gyro;
        data.orientation = values->orientation;
        return {Sample{SampleKind::Data, t, data}};
      }
    }
    return {};
  }

  // 不给 now 时返回最后一次的值，不管多旧
  const Values *latest(uint8_t kind, std::optional<double> now = std::nullopt) const {
    const auto it = last_.find(kind);
    if (it == last_.end()) return nullptr;
    if (now && !(0 <= *now - it->second.time && *now - it->second.time <= max_age_)) return nullptr;
    return &it->second.values;
  }

  std::optional<double> age(uint8_t kind, double now) const {
    const auto it = last_.find(kind);
    if (it == last_.end()) return std::nullopt;
    return now - it->second.time;
  }

 private:
  struct Entry {
    uint64_t seq;
    double time;
    Values values;
  };

  const Values *since(uint8_t kind, uint64_t seq, double now) const {
    const auto it = last_.find(kind);
    if (it != last_.end() && it->second.seq > seq && now - it->second.time <= max_age_) return &it->second.values;
    return nullptr;
  }

  double max_age_;
  uint64_t seq_ = 0, orientation_seq_ = 0, cycle_seq_ = 0;
  std::map<uint8_t, Entry> last_;
};

}  // namespace x5_wit_imu
