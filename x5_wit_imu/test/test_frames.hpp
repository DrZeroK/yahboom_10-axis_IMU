// 两个测试共用的帧构造与断言
#pragma once

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "x5_wit_imu/wit_protocol.hpp"

using namespace x5_wit_imu;
using Bytes = std::vector<uint8_t>;

static int failures = 0;
#define CHECK(cond)                                                   \
  do {                                                                \
    if (!(cond)) {                                                    \
      std::printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #cond);      \
      ++failures;                                                     \
    }                                                                 \
  } while (0)
#define NEAR(a, b, tol) CHECK(std::fabs(double(a) - double(b)) <= (tol))

static inline Bytes hex(const std::string &s) {
  Bytes out;
  for (size_t i = 0; i < s.size(); i += 2) out.push_back(uint8_t(std::stoi(s.substr(i, 2), nullptr, 16)));
  return out;
}

// 2026-10-02 X5 /dev/ttyUSB2 实采的出厂默认周期（acc, gyro, angle, mag）
static const Bytes CYCLE = hex("555189fed4ffe1078f0a81" "55520000000000008f0a40" "555317ff8507c7d0f14618" "5554ccf9840438f000001e");

static inline Bytes operator+(Bytes a, const Bytes &b) {
  a.insert(a.end(), b.begin(), b.end());
  return a;
}

static inline Bytes frame(uint8_t kind, int16_t a = 0, int16_t b = 0, int16_t c = 0, int16_t d = 0) {
  Bytes f = {0x55, kind};
  for (int16_t w : {a, b, c, d}) {
    f.push_back(uint8_t(w & 0xFF));
    f.push_back(uint8_t((uint16_t(w) >> 8) & 0xFF));
  }
  uint8_t sum = 0;
  for (uint8_t x : f) sum = uint8_t(sum + x);
  f.push_back(sum);
  return f;
}

static inline Bytes time_frame(long ms) {
  ms %= 86400000;
  const long seconds_total = ms / 1000, millis = ms % 1000;
  const long hours = seconds_total / 3600, minutes = seconds_total / 60 % 60, seconds = seconds_total % 60;
  Bytes f = {0x55, TIME, 0, 0, 0, uint8_t(hours), uint8_t(minutes), uint8_t(seconds), uint8_t(millis & 255), uint8_t(millis >> 8)};
  uint8_t sum = 0;
  for (uint8_t x : f) sum = uint8_t(sum + x);
  f.push_back(sum);
  return f;
}

