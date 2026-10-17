"""Linux process integration check. Dummy tasks never import ROS or access robot hardware."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from robot_tasks import read_process, scan_tasks, stop_tasks

FIXTURE = '''import signal, subprocess, sys
child = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(600)'], stderr=subprocess.DEVNULL)
def stop(signum, frame):
    child.send_signal(signal.SIGINT)
    child.wait(timeout=3)
    sys.exit(0)
signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)
print(child.pid, flush=True)
signal.pause()
'''


def main():
    if not hasattr(os, 'pidfd_open'):
        raise RuntimeError('Run this integration check on Linux with pidfd support')
    owned = []
    with tempfile.TemporaryDirectory(prefix='rpbot-stop-check-') as folder:
        root = Path(folder) / 'pi5-stm32-slam'
        root.mkdir()
        script = root / 'ros2'
        script.write_text(FIXTURE)
        env = dict(os.environ, ROS_DOMAIN_ID='201')
        def start(args):
            process = subprocess.Popen([sys.executable, str(script), *args], env=env,
                                        stdout=subprocess.PIPE, text=True, stderr=subprocess.DEVNULL)
            owned.append(process)
            child = int(process.stdout.readline())
            return process, child
        try:
            mapping, mapping_child = start(['launch', 'pi5_slam_bringup', 'robot.launch.py', 'mode:=mapping'])
            nav, nav_child = start(['launch', 'pi5_slam_bringup', 'robot.launch.py', 'mode:=navigation'])
            bag, bag_child = start(['bag', 'record', '-o', str(root / 'artifacts/test-bag')])
            unrelated = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(600)'])
            owned.append(unrelated)
            def stop_one(process, scope):
                item = next(item for item in scan_tasks(root, 201, scope) if item['pid'] == process.pid)
                result = stop_tasks(root, 201, scope, [dict(pid=item['pid'], start_ticks=item['start_ticks'])])
                assert result['status'] == 'stopped', result
                process.wait(timeout=2)
                return result
            stop_one(mapping, 'mapping')
            assert nav.poll() is None and bag.poll() is None and unrelated.poll() is None
            stop_one(nav, 'navigation')
            assert bag.poll() is None and unrelated.poll() is None
            stop_one(bag, 'all')
            assert unrelated.poll() is None
            assert all(read_process(pid) is None for pid in (mapping_child, nav_child, bag_child))
            print(json.dumps(dict(result='pass', mapping='stopped', navigation='stopped', bag='closed',
                                  descendants='stopped', unrelated_process='untouched', hardware_access=False)))
        finally:
            for process in owned:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=4)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=2)
                if process.stdout:
                    process.stdout.close()


if __name__ == '__main__':
    main()
