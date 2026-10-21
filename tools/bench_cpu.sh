#!/usr/bin/env bash
# 用 pty 假 IMU 比较两个驱动的 CPU：10 Hz 回放一个实采周期，隔离 ROS 域，各跑 30 s。不碰真串口。
# 用法：先 source ROS 与编好的工作区，再
#   bash tools/bench_cpu.sh <C++ wit_imu_node 路径> <Python wit_imu_driver 路径>
set -eo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
WORK=$(mktemp -d "${TMPDIR:-/tmp}/wit-bench.XXXXXX")  # 结束后保留，便于查看日志
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID_BENCH:-199}
bench() {  # bench <名字> <可执行文件>
  : > "$WORK/pty.txt"
  python3 "$HERE/wit_imu_pty_feed.py" "$WORK/pty.txt" &
  local feeder=$!
  while [ ! -s "$WORK/pty.txt" ]; do sleep 0.1; done
  "$2" --ros-args -p port:="$(cat "$WORK/pty.txt")" -r __node:=imu_bench > "$WORK/$1.log" 2>&1 &
  local pid=$!
  sleep 5
  local t0 c0 t1 c1 rss
  t0=$(date +%s.%N); c0=$(awk '{print $14+$15}' "/proc/$pid/stat")
  sleep 30
  t1=$(date +%s.%N); c1=$(awk '{print $14+$15}' "/proc/$pid/stat")
  rss=$(awk '/VmRSS/{print $2}' "/proc/$pid/status")
  kill "$pid" "$feeder"
  wait "$pid" "$feeder" 2>/dev/null || true
  echo "$1: CPU $(echo "($c1-$c0)/$(getconf CLK_TCK)/($t1-$t0)*100" | bc -l | cut -c1-5)% 单核，RSS ${rss} kB"
}
bench cpp "$1"
bench python "$2"
echo "日志：$WORK"
