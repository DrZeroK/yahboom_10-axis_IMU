// 协议层测试，逐项对应 tests/test_wit_imu.py 的 ClockTests 与 ProtocolTests，不需要 ROS。
#include "test_frames.hpp"

static Payload payload_of(const Bytes &f) {
  Payload p;
  std::copy(f.begin() + 2, f.begin() + 10, p.begin());
  return p;
}

struct ClockFixture {
  FrameParser parser;
  FrameClock clock;
  std::vector<Frame> feed(const Bytes &s, double now, double byte_time = 10.0 / 115200) {
    return clock.feed(parser.feed(s.data(), s.size(), now, byte_time), now);
  }
};

static std::vector<double> stamps_of(const std::vector<Frame> &frames, uint8_t kind) {
  std::vector<double> out;
  for (const auto &f : frames)
    if (f.kind == kind) out.push_back(f.time);
  return out;
}

static std::vector<Sample> feed_assembler(SampleAssembler &a, const std::vector<Frame> &frames) {
  std::vector<Sample> out;
  for (const auto &f : frames)
    for (const auto &s : a.push(f.kind, f.payload, f.time)) out.push_back(s);
  return out;
}

static std::vector<Sample> assemble(const Bytes &stream, double read_time = 100.0, double byte_time = 1e-3) {
  FrameParser parser;
  SampleAssembler assembler(.5);
  return feed_assembler(assembler, parser.feed(stream.data(), stream.size(), read_time, byte_time));
}

// ---- ClockTests

static void batch_uses_chip_intervals_and_one_stamp_per_cycle() {
  ClockFixture c;
  const auto frames = c.feed(time_frame(0) + CYCLE + time_frame(20) + CYCLE + time_frame(40) + CYCLE, 10);
  const auto gyro = stamps_of(frames, GYRO);
  CHECK(gyro.size() == 3);
  for (size_t i = 1; i < gyro.size(); ++i) NEAR(gyro[i] - gyro[i - 1], .02, 1e-7);
  for (size_t i = 0; i < frames.size(); i += 5)
    for (size_t k = i; k < i + 5 && k < frames.size(); ++k) CHECK(frames[k].time == frames[i].time);
  for (const auto &f : frames) CHECK(f.time <= 10);
}

static void no_time_output_preserves_legacy_stamps() {
  ClockFixture c;
  const Bytes s = CYCLE + CYCLE;
  const auto frames = c.parser.feed(s.data(), s.size(), 10, 10.0 / 9600);
  const auto out = c.clock.feed(frames, 10);
  CHECK(out.size() == frames.size());
  for (size_t i = 0; i < out.size() && i < frames.size(); ++i) CHECK(out[i].time == frames[i].time && out[i].kind == frames[i].kind);
  CHECK(!c.clock.seen_time);
}

static void split_frame_and_split_cycle_keep_the_cycle_stamp() {
  ClockFixture c;
  const Bytes s = time_frame(2000) + CYCLE;
  std::vector<Frame> frames;
  for (size_t i = 0; i < s.size(); i += 3) {
    const Bytes part(s.begin() + long(i), s.begin() + long(std::min(i + 3, s.size())));
    for (const auto &f : c.feed(part, 10 + double(i) * .001)) frames.push_back(f);
  }
  CHECK(frames.size() == 5);
  const uint8_t kinds[] = {TIME, ACC, GYRO, ANGLE, MAG};
  for (size_t i = 0; i < frames.size() && i < 5; ++i) CHECK(frames[i].kind == kinds[i]);
  for (const auto &f : frames) CHECK(f.time == frames[0].time);
}

static void midnight_wrap_is_not_a_reset() {
  ClockFixture c;
  const auto frames = c.feed(time_frame(86399990) + CYCLE + time_frame(10) + CYCLE, 10);
  const auto gyro = stamps_of(frames, GYRO);
  CHECK(gyro.size() == 2);
  if (gyro.size() == 2) NEAR(gyro[1] - gyro[0], .02, 1e-7);
  CHECK(c.clock.resets == 0);
}

static void reset_reanchors_to_host_time() {
  ClockFixture c;
  c.feed(time_frame(10000) + CYCLE, 10);
  const auto frames = c.feed(time_frame(0) + CYCLE, 10.1);
  CHECK(c.clock.resets == 1);
  CHECK(!frames.empty());
  for (const auto &f : frames) CHECK(10 < f.time && f.time <= 10.1);
}

static void reset_inside_a_batch_never_reverses_sample_order() {
  ClockFixture c;
  const auto stamps = stamps_of(c.feed(time_frame(10000) + CYCLE + time_frame(0) + CYCLE, 10), GYRO);
  CHECK(stamps.size() == 2);
  if (stamps.size() == 2) {
    CHECK(stamps[0] < stamps[1]);
    CHECK(stamps[1] <= 10);
  }
  CHECK(c.clock.resets == 1);
}

static void gap_does_not_compress_elapsed_time() {
  ClockFixture c;
  const auto first = c.feed(time_frame(0) + CYCLE, 10);
  const auto second = c.feed(time_frame(20) + CYCLE, 12);
  CHECK(c.clock.resets == 1);
  NEAR(second[0].time - first[0].time, 2, 1e-7);
}

static void clock_backwards_starts_a_new_host_epoch() {
  ClockFixture c;
  c.feed(time_frame(0) + CYCLE, 10);
  const auto frames = c.feed(time_frame(20) + CYCLE, 1);
  CHECK(!frames.empty());
  for (const auto &f : frames) CHECK(0 < f.time && f.time <= 1);
}

static void missing_corrupt_invalid_and_duplicate_time_drop_the_cycle() {
  Bytes invalid = time_frame(20);
  invalid[5] = 24;
  uint8_t sum = 0;
  for (size_t i = 0; i < 10; ++i) sum = uint8_t(sum + invalid[i]);
  invalid[10] = sum;
  Bytes bad_crc = time_frame(20);
  bad_crc[10] ^= 1;
  for (const Bytes &bad : {Bytes{}, bad_crc, invalid, time_frame(0)}) {
    ClockFixture c;
    c.feed(time_frame(0) + CYCLE, 10);
    CHECK(c.feed(bad + CYCLE, 10.02).empty());
    const auto frames = c.feed(time_frame(40) + CYCLE, 10.04);
    CHECK(stamps_of(frames, GYRO).size() == 1);
    CHECK(c.clock.dropped_frames > 0);
  }
}

static void time_boundary_prevents_cross_cycle_accelerometer_pairing() {
  ClockFixture c;
  SampleAssembler a(.5);
  CHECK(feed_assembler(a, c.feed(time_frame(0) + frame(ACC) + time_frame(20) + frame(GYRO), 10)).empty());
}

static void partial_cycle_retains_latest_values_for_diagnostics() {
  ClockFixture c;
  SampleAssembler a(.5);
  feed_assembler(a, c.feed(time_frame(0) + CYCLE + time_frame(20), 10));
  CHECK(a.latest(ACC) && std::fabs(a.latest(ACC)->temperature - 27.03) < 1e-9);
}

static void capture_starting_mid_cycle_waits_for_time() {
  ClockFixture c;
  const auto frames = c.feed(Bytes(CYCLE.begin() + 22, CYCLE.end()) + time_frame(20) + CYCLE, 10);
  CHECK(frames.size() == 5);
  const uint8_t kinds[] = {TIME, ACC, GYRO, ANGLE, MAG};
  for (size_t i = 0; i < frames.size() && i < 5; ++i) CHECK(frames[i].kind == kinds[i]);
  CHECK(c.clock.dropped_frames == 2);
}

static void time_boundary_preserves_quaternion_preference() {
  ClockFixture c;
  SampleAssembler a(.5);
  const Bytes cycle = CYCLE + frame(QUAT, 23170, 23170, 0, 0);
  const auto samples = feed_assembler(a, c.feed(time_frame(0) + cycle + time_frame(20) + cycle + time_frame(40) + cycle, 10));
  std::vector<std::array<double, 4>> q;
  for (const auto &s : samples)
    if (s.kind == SampleKind::Data) q.push_back(s.values.orientation);
  CHECK(q.size() == 3);
  if (!q.empty()) NEAR(q.back()[0], std::sqrt(.5), 1e-4);
}

static void host_mapping_tracks_both_clock_drift_directions() {
  for (double scale : {.998, 1.002}) {
    ClockFixture c;
    std::vector<double> stamps, ages;
    for (int i = 0; i < 3000; ++i) {
      const double now = 10 + i * .02 * scale;
      const auto g = stamps_of(c.feed(time_frame(i * 20L) + CYCLE, now), GYRO);
      CHECK(g.size() == 1);
      if (g.empty()) return;
      stamps.push_back(g[0]);
      ages.push_back(now - g[0]);
    }
    for (size_t i = 1; i < stamps.size(); ++i) CHECK(stamps[i - 1] < stamps[i]);
    CHECK(*std::max_element(ages.begin(), ages.end()) < .01);
    CHECK(*std::min_element(ages.begin(), ages.end()) >= 0);
    CHECK(c.clock.resets == 0);
  }
}

// ---- ProtocolTests

static std::array<double, 3> rotate(const std::array<double, 4> &q, const std::array<double, 3> &v) {
  const double x = q[0], y = q[1], z = q[2], w = q[3];
  const double r[3][3] = {{1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)},
                          {2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)},
                          {2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)}};
  return {r[0][0] * v[0] + r[0][1] * v[1] + r[0][2] * v[2], r[1][0] * v[0] + r[1][1] * v[1] + r[1][2] * v[2],
          r[2][0] * v[0] + r[2][1] * v[1] + r[2][2] * v[2]};
}

static void pressure_latest_can_expire_without_changing_legacy_latest() {
  SampleAssembler a(.5);
  Payload p;
  const int32_t pressure = 100000, height = 1234;
  std::memcpy(p.data(), &pressure, 4);
  std::memcpy(p.data() + 4, &height, 4);
  a.push(PRESS, p, 10.);
  CHECK(a.latest(PRESS, 10.1) && a.latest(PRESS, 10.1)->height == 12.34);
  NEAR(*a.age(PRESS, 10.1), .1, 1e-9);
  CHECK(!a.latest(PRESS, 10.6));
  CHECK(!a.latest(PRESS, 9.9));
  CHECK(a.latest(PRESS) && a.latest(PRESS)->pressure == 100000);
}

static void captured_cycle_decodes_to_manual_units() {
  FrameParser parser;
  const auto frames = parser.feed(CYCLE.data(), CYCLE.size(), 0, 0);
  CHECK(frames.size() == 4);
  if (frames.size() != 4) return;
  const auto acc = *decode(frames[0].kind, frames[0].payload), gyro = *decode(frames[1].kind, frames[1].payload),
             angle = *decode(frames[2].kind, frames[2].payload), mag = *decode(frames[3].kind, frames[3].payload);
  const int raw_acc[] = {-375, -44, 2017}, raw_angle[] = {-233, 1925, -12089}, raw_mag[] = {-1588, 1156, -4040};
  for (int k = 0; k < 3; ++k) {
    NEAR(acc.accel[k], raw_acc[k] / 32768.0 * 16 * G, 1e-12);
    CHECK(gyro.gyro[k] == 0.0);
    NEAR(angle.euler[k], raw_angle[k] / 32768.0 * 180 * PI / 180, 1e-12);
    CHECK(mag.mag_lsb[k] == raw_mag[k]);
  }
  NEAR(acc.temperature, 27.03, 1e-12);
  CHECK(angle.version == 18161);
}

static void module_orientation_agrees_with_measured_gravity() {
  std::vector<Sample> data;
  for (const auto &s : assemble(CYCLE))
    if (s.kind == SampleKind::Data) data.push_back(s);
  CHECK(data.size() == 1);
  if (data.empty()) return;
  const auto a = data[0].values.accel;
  const auto world = rotate(data[0].values.orientation, a);
  NEAR(world[0], 0, .05);
  NEAR(world[1], 0, .05);
  NEAR(world[2], std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]), .01);
}

static void noise_split_reads_and_bad_checksums() {
  Bytes bad = frame(ANGLE, 1, 2, 3);
  bad[10] ^= 1;
  const Bytes s = Bytes{0x00, 0x55, 0x13} + frame(ACC, 1, 2, 2048) + bad + frame(GYRO, 4, 5, 6);
  FrameParser parser;
  std::vector<Frame> frames;
  for (size_t i = 0; i < s.size(); i += 3)
    for (const auto &f : parser.feed(s.data() + i, std::min<size_t>(3, s.size() - i), 0, 0)) frames.push_back(f);
  CHECK(frames.size() == 2 && frames[0].kind == ACC && frames[1].kind == GYRO);
  CHECK(parser.bad_checksums == 1);
  CHECK(parser.buffered() < 11);
}

static void frames_are_backdated_by_the_bytes_after_them() {
  const Bytes s = frame(ACC) + frame(GYRO) + Bytes{0x55, 0x51};
  const auto frames = FrameParser().feed(s.data(), s.size(), 10.0, .001);
  NEAR(frames[0].time, 10.0 - .013, 1e-12);
  NEAR(frames[1].time, 10.0 - .002, 1e-12);
}

static std::vector<SampleKind> kinds(const std::vector<Sample> &samples) {
  std::vector<SampleKind> out;
  for (const auto &s : samples) out.push_back(s.kind);
  return out;
}

static void raw_needs_this_cycles_accelerometer() {
  CHECK(kinds(assemble(frame(ACC, 0, 0, 2048) + frame(GYRO))) == std::vector<SampleKind>{SampleKind::Raw});
  // 第 2 周期的角速度帧没有加速度帧：不能复用第 1 周期的
  CHECK(kinds(assemble(frame(ACC) + frame(GYRO) + frame(GYRO))) == std::vector<SampleKind>{SampleKind::Raw});
}

static void two_cycles_in_one_read_both_publish() {
  const auto samples = assemble(CYCLE + CYCLE);
  const std::vector<SampleKind> want = {SampleKind::Raw, SampleKind::Data, SampleKind::Mag,
                                        SampleKind::Raw, SampleKind::Data, SampleKind::Mag};
  CHECK(kinds(samples) == want);
  for (size_t i = 1; i < samples.size(); ++i) CHECK(samples[i - 1].time < samples[i].time);
}

static void stale_values_are_not_paired() {
  SampleAssembler a(.5);
  a.push(ACC, payload_of(frame(ACC)), 0.0);
  CHECK(a.push(GYRO, payload_of(frame(GYRO)), 1.0).empty());
}

static void quaternion_output_replaces_euler_when_enabled() {
  const Bytes cycle = frame(ACC, 0, 0, 2048) + frame(GYRO) + frame(ANGLE, 0, 0, 16384) + frame(MAG) +
                      frame(QUAT, 23170, 23170, 0, 0);  // w, x, y, z：横滚 90°
  std::vector<Sample> data;
  for (const auto &s : assemble(cycle + cycle + cycle))
    if (s.kind == SampleKind::Data) data.push_back(s);
  // 第 1 周期在第一个四元数之前用欧拉角；之后每周期正好一个四元数样本
  CHECK(data.size() == 3);
  if (data.size() != 3) return;
  for (size_t i = 1; i < 3; ++i) {
    const auto q = data[i].values.orientation;
    NEAR(std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]), 1.0, 1e-9);
    NEAR(q[0], std::sqrt(.5), 1e-4);
    NEAR(q[3], std::sqrt(.5), 1e-4);
  }
  NEAR(data[0].values.orientation[2], std::sqrt(.5), 1e-4);  // 角度帧给的 yaw 90°
}

int main() {
  batch_uses_chip_intervals_and_one_stamp_per_cycle();
  no_time_output_preserves_legacy_stamps();
  split_frame_and_split_cycle_keep_the_cycle_stamp();
  midnight_wrap_is_not_a_reset();
  reset_reanchors_to_host_time();
  reset_inside_a_batch_never_reverses_sample_order();
  gap_does_not_compress_elapsed_time();
  clock_backwards_starts_a_new_host_epoch();
  missing_corrupt_invalid_and_duplicate_time_drop_the_cycle();
  time_boundary_prevents_cross_cycle_accelerometer_pairing();
  partial_cycle_retains_latest_values_for_diagnostics();
  capture_starting_mid_cycle_waits_for_time();
  time_boundary_preserves_quaternion_preference();
  host_mapping_tracks_both_clock_drift_directions();
  pressure_latest_can_expire_without_changing_legacy_latest();
  captured_cycle_decodes_to_manual_units();
  module_orientation_agrees_with_measured_gravity();
  noise_split_reads_and_bad_checksums();
  frames_are_backdated_by_the_bytes_after_them();
  raw_needs_this_cycles_accelerometer();
  two_cycles_in_one_read_both_publish();
  stale_values_are_not_paired();
  quaternion_output_replaces_euler_when_enabled();
  std::printf(failures ? "test_wit_protocol: %d 项失败\n" : "test_wit_protocol: 23 组全部通过\n", failures);
  return failures ? 1 : 0;
}
