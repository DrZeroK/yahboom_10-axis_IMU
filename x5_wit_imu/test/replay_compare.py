"""C++ 协议层与 Python 原版对拍：随机生成带噪声、丢字节、时钟漂移/跳变、跨午夜的流，比较两边逐帧输出。

用法：python3 replay_compare.py <replay 可执行文件> [场景数]
Python 原版从同一工作区的 imu_ros2_device 包导入（Mac：src/x5-nav/src；板端：ros_ws/src）。
"""
import random
import struct
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'imu_ros2_device'))
from imu_ros2_device.wit_protocol import (ACC, ANGLE, GYRO, MAG, PRESS, QUAT, TIME,  # noqa: E402
                                          FrameClock, FrameParser, SampleAssembler)


def frame(kind, payload):
    body = bytes([0x55, kind]) + payload
    return body + bytes([sum(body) & 255])


def words(*w):
    return struct.pack('<4h', *w)


def time_frame(ms):
    ms %= 86400000
    s, millis = divmod(ms, 1000)
    m, s = divmod(s, 60)
    h, m = divmod(m, 60)
    return frame(TIME, bytes((0, 0, 0, h, m, s, millis & 255, millis >> 8)))


def scenario(rnd):
    """返回 [(now, byte_time, bytes)]。"""
    byte_time = rnd.choice((10 / 9600, 10 / 115200))
    use_time = rnd.random() < .8
    use_quat = rnd.random() < .5
    use_press = rnd.random() < .3
    chip_ms = rnd.choice((0, 86399000, rnd.randrange(86400000)))
    period = rnd.choice((10, 20, 50, 100))
    now = 10 + rnd.random() * 100
    drift = 1 + rnd.uniform(-.003, .003)
    stream = bytearray()
    for _ in range(rnd.randrange(50, 400)):
        r = lambda: rnd.randrange(-32768, 32768)
        cyc = bytearray()
        if use_time:
            cyc += time_frame(chip_ms)
        cyc += frame(ACC, words(r(), r(), r(), rnd.randrange(-500, 6000)))
        cyc += frame(GYRO, words(r(), r(), r(), 0))
        cyc += frame(ANGLE, words(r(), r(), r(), 18161))
        cyc += frame(MAG, words(r(), r(), r(), 0))
        if use_press:
            cyc += frame(PRESS, struct.pack('<ii', rnd.randrange(90000, 105000), rnd.randrange(-5000, 5000)))
        if use_quat:
            cyc += frame(QUAT, words(*(rnd.choice((0, r())) if rnd.random() < .02 else r() for _ in range(4))))
        x = rnd.random()
        if x < .03:
            cyc[rnd.randrange(len(cyc))] ^= 1 << rnd.randrange(8)
        elif x < .06:
            i = rnd.randrange(len(cyc))
            del cyc[i:i + rnd.randrange(1, 12)]
        elif x < .08:
            cyc[rnd.randrange(len(cyc)):rnd.randrange(len(cyc))] = bytes(rnd.randrange(256) for _ in range(rnd.randrange(1, 20)))
        stream += cyc
        y = rnd.random()
        chip_ms += period if y > .02 else rnd.choice((0, period * 30, -1000, 50000))  # 偶发复位、跳变、重复
        chip_ms %= 86400000
    chunks, i = [], 0
    while i < len(stream):
        n = rnd.choice((1, 3, 11, 44, 48, 100, 4096)) if rnd.random() < .5 else int(period * 1e-3 * len(stream) / 400 * 50) + 1
        y = rnd.random()
        step = .05 * drift if y > .03 else rnd.choice((.6, -2.0, 0.0, .001))  # 偶发长停顿、主机时间回退
        now = max(.5, now + step)
        chunks.append((now, byte_time, bytes(stream[i:i + n])))
        i += n
    return chunks


def python_replay(chunks):
    parser, clock, asm = FrameParser(), FrameClock(.5), SampleAssembler(.5)
    out = []
    for now, byte_time, data in chunks:
        for kind, payload, t in clock.feed(parser.feed(data, now, byte_time), now):
            out.append(f'F {kind:02x} {t:.9f}')
            for sample, st, v in asm.push(kind, payload, t):
                if sample == 'mag':
                    out.append('S mag %.9f %d %d %d' % (st, *v['mag_lsb']))
                else:
                    line = 'S %s %.9f ' % (sample, st) + ' '.join('%.9f' % x for x in (*v['accel'], *v['gyro']))
                    if sample == 'data':
                        line += ' ' + ' '.join('%.9f' % x for x in v['orientation'])
                    out.append(line)
    press = asm.latest(PRESS)
    out.append('END %d %d %d %d %.6f' % (parser.bad_checksums, clock.resets, clock.dropped_frames, clock.seen_time,
                                         press['pressure'] if press else -1.0))
    return out


def same(a, b):
    ta, tb = a.split(), b.split()
    if len(ta) != len(tb):
        return False
    for x, y in zip(ta, tb):
        if x == y:
            continue
        try:
            if abs(float(x) - float(y)) > 1e-8:
                return False
        except ValueError:
            return False
    return True


def main():
    exe, count = sys.argv[1], int(sys.argv[2]) if len(sys.argv) > 2 else 300
    total = 0
    for seed in range(count):
        chunks = scenario(random.Random(seed))
        want = python_replay(chunks)
        stdin = ''.join(f'{now!r} {bt!r} {data.hex()}\n' for now, bt, data in chunks)
        got = subprocess.run([exe], input=stdin, capture_output=True, text=True, check=True).stdout.splitlines()
        if len(got) != len(want) or not all(same(a, b) for a, b in zip(got, want)):
            for i, (a, b) in enumerate(zip(got, want)):
                if not same(a, b):
                    print(f'场景 {seed} 第 {i} 行不同：\n  C++    {a}\n  Python {b}')
                    break
            else:
                print(f'场景 {seed} 行数不同：C++ {len(got)}，Python {len(want)}')
            sys.exit(1)
        total += len(want)
    print(f'对拍一致：{count} 个场景，{total} 行输出')


if __name__ == '__main__':
    main()
