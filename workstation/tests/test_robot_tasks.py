from pathlib import Path
import signal
import subprocess
import sys
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import robot_tasks as tasks
import rpbot_console as console

ROOT = Path('/home/test/pi5-stm32-slam')


def process(pid, mode='mapping', *, domain=0, ppid=1, uid=1000, words=None, cgroup='0::/user.slice/session-1.scope\n'):
    return dict(pid=pid, uid=uid, ppid=ppid, start_ticks=str(pid*100), domain=domain, cwd=str(ROOT), cgroup=cgroup,
                words=words or ['python3', '/opt/ros/jazzy/bin/ros2', 'launch', 'pi5_slam_bringup', 'robot.launch.py', 'mode:='+mode])


class RobotTaskTests(unittest.TestCase):
    def setUp(self):
        self.uid = patch.object(tasks.os, 'getuid', return_value=1000, create=True)
        self.uid.start()
        self.addCleanup(self.uid.stop)

    def test_scope_and_owner_do_not_include_other_robot_domains_or_users(self):
        records = {1: process(100), 2: process(200, 'navigation'), 3: process(300, domain=61), 4: process(400, uid=2000)}
        self.assertEqual([x['pid'] for x in tasks.scan_tasks(ROOT, 0, 'mapping', processes=records)], [100])
        self.assertEqual([x['pid'] for x in tasks.scan_tasks(ROOT, 0, 'all', processes=records)], [100, 200])

    def test_bag_wrapper_is_not_counted_twice_and_unrelated_bag_is_excluded(self):
        record = ['python3', '/opt/ros/jazzy/bin/ros2', 'bag', 'record', '-o', str(ROOT / 'artifacts/drive')]
        wrapper = ['timeout', '-s', 'INT', '3600', 'ros2', 'bag', 'record', '-o', str(ROOT / 'artifacts/drive')]
        outside = ['python3', '/opt/ros/jazzy/bin/ros2', 'bag', 'record', '-o', '/unrelated/bag']
        self.assertEqual(tasks.identify_task(process(100, words=record), ROOT)['kind'], 'bag')
        self.assertIsNone(tasks.identify_task(process(200, words=wrapper), ROOT))
        self.assertIsNone(tasks.identify_task(process(300, words=outside), ROOT))

    def test_other_ros_launch_is_not_a_robot_task(self):
        item = process(100, words=['python3', '/opt/ros/jazzy/bin/ros2', 'launch', 'other_package', 'robot.launch.py'])
        self.assertIsNone(tasks.identify_task(item, ROOT))

    def test_stale_pid_identity_aborts_before_any_stop(self):
        item = process(100)
        with patch.object(tasks, 'read_processes', return_value={100: item}), patch.object(tasks, 'signal_process') as send:
            with self.assertRaisesRegex(RuntimeError, 'đã đổi'):
                tasks.stop_tasks(ROOT, 0, 'mapping', [dict(pid=100, start_ticks='old')])
        send.assert_not_called()

    def test_mapping_stop_does_not_signal_navigation_or_siblings(self):
        records = {100: process(100), 101: process(101, ppid=100), 200: process(200, 'navigation')}
        with patch.object(tasks, 'read_processes', return_value=records), patch.object(tasks, 'signal_process') as send, \
             patch.object(tasks, 'wait_stopped', return_value={}):
            result = tasks.stop_tasks(ROOT, 0, 'mapping', [dict(pid=100, start_ticks='10000')])
        self.assertEqual(result['status'], 'stopped')
        send.assert_called_once_with(records[100], signal.SIGINT)

    def test_fallback_terminates_only_captured_descendants(self):
        records = {100: process(100), 101: process(101, ppid=100), 200: process(200, 'navigation')}
        with patch.object(tasks, 'read_processes', return_value=records), patch.object(tasks, 'signal_process') as send, \
             patch.object(tasks, 'wait_stopped', side_effect=[{101: records[101]}, {}]):
            result = tasks.stop_tasks(ROOT, 0, 'mapping', [dict(pid=100, start_ticks='10000')])
        self.assertEqual(result['status'], 'stopped')
        self.assertEqual(send.call_count, 2)
        self.assertEqual(send.call_args.args, (records[101], signal.SIGTERM))

    def test_remaining_child_is_reported_as_partial(self):
        record = process(100)
        with patch.object(tasks, 'read_processes', return_value={100: record}), patch.object(tasks, 'signal_process'), \
             patch.object(tasks, 'wait_stopped', return_value={100: record}):
            result = tasks.stop_tasks(ROOT, 0, 'mapping', [dict(pid=100, start_ticks='10000')])
        self.assertEqual(result['status'], 'partial')
        self.assertEqual(result['remaining'], [100])

    def test_service_stop_goes_through_systemd_to_prevent_respawn(self):
        record = process(100, cgroup='0::/user.slice/app.slice/rpbot-mapping.service\n')
        results = [subprocess.CompletedProcess([], 0, '100\n', ''), subprocess.CompletedProcess([], 0, '', '')]
        with patch.object(tasks, 'read_processes', return_value={100: record}), patch.object(tasks, 'signal_process') as send, \
             patch.object(tasks, 'wait_stopped', return_value={}), patch.object(tasks.subprocess, 'run', side_effect=results) as run:
            tasks.stop_tasks(ROOT, 0, 'mapping', [dict(pid=100, start_ticks='10000')])
        send.assert_not_called()
        self.assertEqual(run.call_args.args[0], ['systemctl', '--user', '--no-block', 'stop', 'rpbot-mapping.service'])

    def test_changed_service_main_pid_is_not_stopped(self):
        record = process(100, 'navigation', cgroup='0::/user.slice/app.slice/robot-nav.service\n')
        with patch.object(tasks, 'read_processes', return_value={100: record}), patch.object(tasks, 'signal_process') as send, \
             patch.object(tasks.subprocess, 'run', return_value=subprocess.CompletedProcess([], 0, '200\n', '')) as run:
            with self.assertRaises(RuntimeError):
                tasks.stop_tasks(ROOT, 0, 'navigation', [dict(pid=100, start_ticks='10000')])
        self.assertEqual(run.call_count, 1)
        send.assert_not_called()

    def test_gone_targets_are_a_successful_noop(self):
        with patch.object(tasks, 'read_processes', return_value={}), patch.object(tasks, 'signal_process') as send:
            result = tasks.stop_tasks(ROOT, 0, 'all', [dict(pid=100, start_ticks='10000')])
        self.assertEqual(result['status'], 'nothing_running')
        send.assert_not_called()

    def test_pidfd_checks_start_time_again_before_signaling(self):
        expected = process(100)
        with patch.object(tasks.os, 'pidfd_open', return_value=10, create=True), patch.object(tasks.os, 'close'), \
             patch.object(tasks, 'read_process', return_value=dict(expected, start_ticks='reused')), \
             patch.object(tasks.signal, 'pidfd_send_signal', create=True) as send:
            with self.assertRaises(RuntimeError):
                tasks.signal_process(expected, signal.SIGINT)
        send.assert_not_called()

    def test_cancel_confirmation_never_sends_stop(self):
        app = console.Console.__new__(console.Console)
        app.dialogs = console.Dialogs(console.Language(), console.messagebox)
        app.append, app.control_task = Mock(), Mock()
        task = dict(pid=100, start_ticks='10000', mode='mapping', kind='launch', label='Mapping')
        with patch.object(console.messagebox, 'askyesno', return_value=False):
            app.confirm_stop_tasks(console.DEFAULTS, 0, 'mapping', [task])
        app.control_task.assert_not_called()


if __name__ == '__main__':
    unittest.main()
