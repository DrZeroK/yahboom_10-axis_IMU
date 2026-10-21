"""假 IMU：开一个 pty，按出厂配置（10 Hz、无 TIME 帧）不停写 10-02 实采的一个周期。

把从端路径写进 argv[1] 指定的文件，然后一直运行，直到被结束。只写 pty，不碰真串口。
"""
import os
import sys
import time

CYCLE = bytes.fromhex('555189fed4ffe1078f0a81' '55520000000000008f0a40' '555317ff8507c7d0f14618' '5554ccf9840438f000001e')

master, slave = os.openpty()
with open(sys.argv[1], 'w') as f:
    f.write(os.ttyname(slave))
next_t = time.monotonic()
while True:
    os.write(master, CYCLE)
    next_t += .1
    time.sleep(max(0.0, next_t - time.monotonic()))
