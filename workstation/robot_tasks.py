"""Identify and stop this user's robot launches/bags by PID and Linux start time."""
import os
from pathlib import Path
import signal
import subprocess
import time


def read_process(pid):
    try:
        path = Path('/proc') / str(pid)
        uid = path.stat().st_uid
        if uid != os.getuid():
            return None
        fields = (path / 'stat').read_text().rsplit(')', 1)[1].split()
        if fields[0] == 'Z':
            return None
        words = [word.decode(errors='replace') for word in (path / 'cmdline').read_bytes().split(b'\0') if word]
        env = dict(part.split(b'=', 1) for part in (path / 'environ').read_bytes().split(b'\0') if b'=' in part)
        return dict(pid=int(pid), uid=uid, ppid=int(fields[1]), start_ticks=fields[19],
                    words=words, domain=int(env.get(b'ROS_DOMAIN_ID', b'0')),
                    cwd=str((path / 'cwd').resolve()),
                    cgroup=(path / 'cgroup').read_text())
    except (OSError, ValueError, IndexError):
        return None


def read_processes():
    result = {}
    for path in Path('/proc').iterdir():
        if path.name.isdigit():
            item = read_process(int(path.name))
            if item:
                result[item['pid']] = item
    return result


def identify_task(item, root):
    words = item['words']
    ros_index = next((i for i, word in enumerate(words) if Path(word).name == 'ros2'), None)
    if ros_index is None or ros_index > 1 or (ros_index == 1 and not Path(words[0]).name.startswith('python')):
        return None
    args = words[ros_index+1:]
    if args[:3] == ['launch', 'pi5_slam_bringup', 'robot.launch.py']:
        mode = next((word.split(':=', 1)[1] for word in args[3:] if word.startswith('mode:=')), 'bench')
        return dict(kind='launch', mode=mode, label=mode.capitalize(), simulated='use_sim_time:=true' in args)
    if args[:2] not in (['bag', 'record'], ['bag', 'play']):
        return None
    # A bag belongs to the robot only when its path is in these project directories.
    roots = [Path(root).resolve(), Path(root).parent / 'diag_bags', Path(root).parent / 'slam_tune_ws']
    candidates = []
    if args[1] == 'record':
        for index, word in enumerate(args):
            if word in ('-o', '--output') and index+1 < len(args):
                candidates.append(args[index+1])
            elif word.startswith('--output='):
                candidates.append(word.split('=', 1)[1])
    else:
        candidates = [word for word in args[2:] if not word.startswith('-')]
    for value in candidates:
        path = Path(value)
        path = (Path(item['cwd']) / path).resolve() if not path.is_absolute() else path.resolve()
        if any(path.is_relative_to(directory.resolve()) for directory in roots):
            mode = 'record' if args[1] == 'record' else 'playback'
            return dict(kind='bag', mode=mode, label=('Ghi bag: ' if mode == 'record' else 'Phát bag: ') + path.name, simulated=mode == 'playback')
    return None


def scan_tasks(root, domain, scope='all', *, processes=None):
    if scope not in ('all', 'mapping', 'navigation'):
        raise ValueError('Invalid stop scope')
    tasks = []
    for item in (read_processes() if processes is None else processes).values():
        if item['uid'] != os.getuid() or item['domain'] != domain:
            continue
        task = identify_task(item, root)
        if not task or (scope != 'all' and (task['kind'] != 'launch' or task['mode'] != scope)):
            continue
        tasks.append(dict(task, pid=item['pid'], start_ticks=item['start_ticks'], domain=domain,
                          source='Console service' if any('/' + service in item['cgroup'] for service in
                              ('rpbot-mapping.service', 'rpbot-navigation.service')) else 'Ngoài Console'))
    return sorted(tasks, key=lambda item: item['pid'])


def descendants(processes, roots):
    found = set(roots)
    while True:
        more = {pid for pid, item in processes.items() if item['ppid'] in found}
        if more <= found:
            return {pid: processes[pid] for pid in found if pid in processes}
        found.update(more)


def signal_process(item, signum):
    # An open pidfd keeps a recycled PID from redirecting the signal elsewhere.
    if item['pid'] <= 1 or item['pid'] == os.getpid():
        raise ValueError('Refusing to signal the helper or a system process')
    try:
        fd = os.pidfd_open(item['pid'])
    except ProcessLookupError:
        return
    try:
        current = read_process(item['pid'])
        if not current:
            return
        if current['uid'] != os.getuid() or current['start_ticks'] != item['start_ticks']:
            raise RuntimeError('PID đã thay đổi; chưa gửi tín hiệu dừng')
        signal.pidfd_send_signal(fd, signum)
    except ProcessLookupError:
        pass
    finally:
        os.close(fd)


def remaining_processes(tracked):
    result = {}
    for pid, expected in tracked.items():
        current = read_process(pid)
        if current and current['uid'] == expected['uid'] and current['start_ticks'] == expected['start_ticks']:
            result[pid] = current
    return result


def wait_stopped(tracked, seconds):
    deadline = time.monotonic() + seconds
    while True:
        remaining = remaining_processes(tracked)
        if not remaining or time.monotonic() >= deadline:
            return remaining
        time.sleep(0.1)


def stop_tasks(root, domain, scope, targets):
    if not isinstance(targets, list) or len(targets) > 64:
        raise ValueError('Invalid target list')
    processes = read_processes()
    allowed = {item['pid']: item for item in scan_tasks(root, domain, scope, processes=processes)}
    selected, gone = [], []
    seen = set()
    for target in targets:
        pid, start = int(target['pid']), str(target['start_ticks'])
        if pid in seen:
            continue
        seen.add(pid)
        if pid not in processes:
            gone.append(pid)
            continue
        if pid not in allowed or allowed[pid]['start_ticks'] != start:
            raise RuntimeError(f'Phiên PID {pid} đã đổi hoặc không thuộc tác vụ được chọn; hãy tải lại danh sách')
        selected.append(allowed[pid])
    if not selected:
        return dict(status='nothing_running', stopped=[], already_gone=gone, remaining=[])
    tracked = descendants(processes, {item['pid'] for item in selected})
    services = {}
    for item in selected:
        expected = processes[item['pid']]
        group = expected['cgroup'].strip().splitlines()[-1].split(':', 2)[-1]
        unit = group.rsplit('/', 1)[-1]
        if unit.endswith('.service') and not unit.startswith('user@'):
            result = subprocess.run(['systemctl', '--user', 'show', unit, '--property=MainPID', '--value'],
                                    text=True, capture_output=True, timeout=5)
            if result.returncode or result.stdout.strip() != str(item['pid']):
                raise RuntimeError(f'Phiên PID {item["pid"]} thuộc service {unit}; chưa xác nhận được tiến trình chính để dừng')
            services[item['pid']] = unit
    for item in selected:
        expected = processes[item['pid']]
        if item['pid'] in services:
            command = ['systemctl', '--user', '--no-block', 'stop', services[item['pid']]]
            result = subprocess.run(command, text=True, capture_output=True, timeout=5)
            if result.returncode:
                raise RuntimeError(result.stderr)
        else:
            signal_process(expected, signal.SIGINT)
    remaining = wait_stopped(tracked, 7)
    # If graceful ROS launch shutdown did not finish, terminate only the same
    # captured descendants. Never signal a process group or use pkill patterns.
    for item in remaining.values():
        signal_process(item, signal.SIGTERM)
    remaining = wait_stopped(tracked, 4) if remaining else {}
    return dict(status='partial' if remaining else 'stopped', stopped=selected,
                already_gone=gone, remaining=sorted(remaining))
