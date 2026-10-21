// 对拍用：stdin 每行 "<now> <byte_time> <hex 字节>"，按 FrameParser → FrameClock → SampleAssembler 处理，
// 输出格式与 replay_compare.py 里的 Python 版一致。
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>

#include "x5_wit_imu/wit_protocol.hpp"

using namespace x5_wit_imu;

int main() {
  FrameParser parser;
  FrameClock clock(.5);
  SampleAssembler assembler(.5);
  std::string line;
  while (std::getline(std::cin, line)) {
    std::istringstream in(line);
    double now, byte_time;
    std::string hex;
    in >> now >> byte_time >> hex;
    std::vector<uint8_t> data;
    for (size_t i = 0; i + 1 < hex.size(); i += 2) data.push_back(uint8_t(std::stoi(hex.substr(i, 2), nullptr, 16)));
    for (const auto &f : clock.feed(parser.feed(data.data(), data.size(), now, byte_time), now)) {
      std::printf("F %02x %.9f\n", f.kind, f.time);
      for (const auto &s : assembler.push(f.kind, f.payload, f.time)) {
        const auto &v = s.values;
        if (s.kind == SampleKind::Mag) {
          std::printf("S mag %.9f %d %d %d\n", s.time, v.mag_lsb[0], v.mag_lsb[1], v.mag_lsb[2]);
        } else {
          std::printf("S %s %.9f %.9f %.9f %.9f %.9f %.9f %.9f", s.kind == SampleKind::Raw ? "raw" : "data", s.time,
                      v.accel[0], v.accel[1], v.accel[2], v.gyro[0], v.gyro[1], v.gyro[2]);
          if (s.kind == SampleKind::Data)
            std::printf(" %.9f %.9f %.9f %.9f", v.orientation[0], v.orientation[1], v.orientation[2], v.orientation[3]);
          std::printf("\n");
        }
      }
    }
  }
  const Values *press = assembler.latest(PRESS);
  std::printf("END %llu %llu %llu %d %.6f\n", (unsigned long long)parser.bad_checksums, (unsigned long long)clock.resets,
              (unsigned long long)clock.dropped_frames, int(clock.seen_time), press ? press->pressure : -1.0);
}
