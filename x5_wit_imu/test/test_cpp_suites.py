"""wh test ros 的入口：运行 colcon 编出来的 C++ 测试，并与 Python 原版对拍。

工作区：从本文件往上找第一个含 build/x5_wit_imu 的目录（包可以直接在 src/ 下，也可以套在仓库目录里），
可用 WH_WS 覆盖。
"""
import os
import subprocess
import sys
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
WS = Path(os.environ['WH_WS']) if 'WH_WS' in os.environ else next(
    (p for p in HERE.parents if (p / 'build' / 'x5_wit_imu').is_dir()), HERE.parents[2])
BUILD = WS / 'build' / 'x5_wit_imu'


def run(*cmd, env=None):
    for arg in cmd:
        if arg.startswith(str(BUILD)) and not Path(arg).exists():
            raise AssertionError(f'{arg} 不存在：先 colcon build --packages-select x5_wit_imu')
    return subprocess.run(cmd, capture_output=True, text=True, timeout=300, env=env)


class CppSuites(unittest.TestCase):
    def test_protocol(self):
        r = run(str(BUILD / 'test_wit_protocol'))
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)

    def test_node(self):
        env = dict(os.environ, ROS_DOMAIN_ID='199')  # 隔离域；输出被收集，不外发
        r = run(str(BUILD / 'test_wit_imu_node'), env=env)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)

    def test_replay_matches_python(self):
        r = run(sys.executable, str(HERE / 'replay_compare.py'), str(BUILD / 'wit_replay'), '100')
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)
        self.assertIn('对拍一致', r.stdout)


if __name__ == '__main__':
    unittest.main()
