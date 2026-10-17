import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import map_library
import remote_ops


class NavigationBackendTests(unittest.TestCase):
    def setUp(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        self.home = Path(folder.name)
        self.project = self.home / 'project with spaces'
        (self.project / 'maps').mkdir(parents=True)
        (self.project / 'install').mkdir()
        (self.project / 'install/setup.bash').touch()
        (self.project / 'maps/room.yaml').write_text(
            'image: room.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n'
            'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
        (self.project / 'maps/room.pgm').write_bytes(b'P5\n2 2\n255\n' + bytes([0, 205, 254, 255]))
        self.revision = map_library.map_bundle(self.project / 'maps', 'room')[0]['revision']
        self.wrapper = self.home / '.local/share/rpbot-console/navigation.sh'
        self.unit = self.home / '.config/systemd/user' / remote_ops.NAV_SERVICE
        # Windows has no flock; Linux runs these checks with real file locks.
        if os.name == 'nt':
            stub = SimpleNamespace(flock=Mock(), LOCK_EX=2, LOCK_SH=1)
            self.enter(patch.dict(sys.modules, {'fcntl': stub}))
        self.enter(patch.object(remote_ops.Path, 'home', return_value=self.home))
        self.enter(patch.object(remote_ops, 'ROOT', self.project))
        self.launches = self.enter(patch.object(remote_ops, 'launches', return_value=[]))
        self.tasks = self.enter(patch.object(remote_ops, 'scan_tasks', return_value=[]))
        self.run = self.enter(patch.object(remote_ops, 'run', side_effect=self.command_result))

    def enter(self, context):
        result = context.__enter__()
        self.addCleanup(context.__exit__, None, None, None)
        return result

    @staticmethod
    def command_result(command):
        return subprocess.CompletedProcess(command, 0, 'inactive\n' if 'show' in command else '', '')

    def start(self):
        return remote_ops.start_navigation('room', self.revision, 17)

    def session(self, **changes):
        result = dict(pid=100, mode='navigation', simulated_time=False, domain=17,
                      map=str((self.project / 'maps/room.yaml').resolve()))
        result.update(changes)
        return result

    def test_start_uses_selected_map_domain_and_no_motor_enable_or_boot_enable(self):
        self.assertEqual(self.start()['status'], 'started')
        command = shlex.split(self.wrapper.read_text().splitlines()[-1])[1:]
        self.assertEqual(command, [str(self.project / 'scripts/run.sh'), 'navigation',
                                   'map:=' + str((self.project / 'maps/room.yaml').resolve()),
                                   'allow_motor_enable:=false'])
        unit = self.unit.read_text()
        self.assertIn('ROS_DOMAIN_ID=17', unit)
        self.assertIn('Restart=no', unit)
        self.assertIn('KillMode=control-group', unit)
        self.assertNotIn('[Install]', unit)
        commands = [call.args[0] for call in self.run.call_args_list]
        self.assertEqual(commands[-1], ['systemctl', '--user', 'start', remote_ops.NAV_SERVICE])
        self.assertFalse(any(word in command for command in commands for word in ('enable', 'restart', 'stop')))

    def test_same_navigation_is_reused_without_any_service_commands(self):
        self.launches.return_value = [self.session()]
        for _ in range(2):
            self.assertEqual(self.start()['status'], 'already_running')
        self.run.assert_not_called()
        self.assertFalse(self.wrapper.exists())

    def test_running_sessions_are_left_unchanged(self):
        for changes in ({'mode': 'mapping'}, {'mode': 'bench'}, {'simulated_time': True},
                        {'domain': 0}, {'map': '/different/map.yaml'}, {'map': ''}):
            with self.subTest(changes=changes):
                self.launches.return_value = [self.session(**changes)]
                with self.assertRaisesRegex(RuntimeError, 'Dừng'):
                    self.start()
        self.launches.return_value = [self.session(), self.session(pid=101)]
        with self.assertRaises(RuntimeError):
            self.start()
        self.run.assert_not_called()
        self.assertFalse(self.wrapper.exists())

    def test_missing_or_changed_selection_never_starts_service(self):
        for name, revision in (('room', None), ('room', 'old'), ('missing', self.revision), ('../room', self.revision)):
            with self.subTest(name=name, revision=revision), self.assertRaises((ValueError, RuntimeError, FileNotFoundError)):
                remote_ops.start_navigation(name, revision, 17)
        self.run.assert_not_called()

    def test_replay_in_selected_domain_is_blocked(self):
        self.tasks.return_value = [dict(kind='bag', mode='play')]
        with self.assertRaisesRegex(RuntimeError, 'bag'):
            self.start()
        self.run.assert_not_called()

    def test_service_starting_or_stopping_does_not_rewrite_files(self):
        for state in ('activating', 'active', 'deactivating'):
            self.run.side_effect = None
            self.run.return_value = subprocess.CompletedProcess([], 0, state + '\n', '')
            with self.assertRaisesRegex(RuntimeError, 'khởi động'):
                self.start()
        self.assertFalse(self.wrapper.exists())

    def test_failed_start_is_reported_and_not_retried(self):
        def fail(command):
            if 'start' in command:
                return subprocess.CompletedProcess(command, 1, '', 'Launch failed')
            return self.command_result(command)
        self.run.side_effect = fail
        with self.assertRaisesRegex(RuntimeError, 'Launch failed'):
            self.start()
        self.assertEqual(sum('start' in call.args[0] for call in self.run.call_args_list), 1)

    @unittest.skipIf(os.name == 'nt', 'Requires Linux bash and systemd-analyze')
    def test_generated_wrapper_executes_exact_argv_and_unit_is_valid_on_linux(self):
        self.start()
        binaries = self.home / 'bin'
        binaries.mkdir()
        pgrep = binaries / 'pgrep'
        pgrep.write_text('#!/bin/sh\nexit 1\n')
        pgrep.chmod(0o755)
        scripts = self.project / 'scripts'
        scripts.mkdir()
        runner = scripts / 'run.sh'
        runner.write_text('#!/usr/bin/env python3\nimport json,os,sys\n'
                          'print(json.dumps(dict(argv=sys.argv[1:], domain=os.environ["ROS_DOMAIN_ID"])))\n')
        runner.chmod(0o755)
        # run() is mocked only for service management; these are real Linux tools.
        subprocess.run(['bash', '-n', str(self.wrapper)], check=True, capture_output=True, text=True)
        verified = subprocess.run(['systemd-analyze', '--user', 'verify', str(self.unit)], capture_output=True, text=True)
        self.assertEqual(verified.returncode, 0, verified.stdout + verified.stderr)
        env = dict(os.environ, PATH=str(binaries) + os.pathsep + os.environ['PATH'], ROS_DOMAIN_ID='17')
        executed = subprocess.run(['bash', str(self.wrapper)], env=env, capture_output=True, text=True, check=True)
        result = json.loads(executed.stdout)
        self.assertEqual(result['domain'], '17')
        self.assertEqual(result['argv'], ['navigation', 'map:=' + str(self.project / 'maps/room.yaml'),
                                          'allow_motor_enable:=false'])
        pgrep.write_text('#!/bin/sh\nexit 0\n')
        blocked = subprocess.run(['bash', str(self.wrapper)], env=env, capture_output=True, text=True)
        self.assertEqual(blocked.returncode, 75)
        self.assertEqual(blocked.stdout, '')


class NavigationGuiTests(unittest.TestCase):
    def setUp(self):
        global console
        import rpbot_console as console

    def test_gui_requires_selection_before_requesting_start(self):
        app = console.Console.__new__(console.Console)
        app.busy = app.stop_busy = False
        app.selected_record = Mock(return_value=None)
        app.task, app.tabs = Mock(), Mock()
        app.tab_frames = {'maps': 'maps-tab'}
        app.start_navigation()
        app.task.assert_not_called()
        app.tabs.select.assert_called_once_with('maps-tab')

    def test_gui_sends_selected_revision_and_waits_for_readiness(self):
        app = console.Console.__new__(console.Console)
        app.busy = app.stop_busy = False
        app.selected_record = Mock(return_value=(console.DEFAULTS, dict(name='room', revision='test-revision')))
        app.task_domain_value = Mock(return_value=17)
        app.task, app.root, app.check, app.stop_stream, app.append = Mock(), Mock(), Mock(), Mock(), Mock()
        app.status = {'navigation': Mock()}
        app.start_navigation()
        work, done = app.task.call_args.args[1:]
        with patch.object(console, 'remote', return_value=json.dumps(dict(status='started'))) as remote:
            done(work())
        remote.assert_called_once_with(console.DEFAULTS, 'start-nav', '--name', 'room',
                                       '--revision', 'test-revision', '--domain', '17')
        app.root.after.assert_called_once_with(2000, app.check)
        self.assertIsNone(app.last_data)

    def test_rviz_navigation_can_open_before_action_server_to_set_initial_pose(self):
        app = console.Console.__new__(console.Console)
        app.require_session = Mock(return_value=console.DEFAULTS)
        app.session, app.last_data, app.viewers = {'mode': 'navigation'}, None, {}
        app.append = Mock()
        with tempfile.TemporaryDirectory() as folder, patch.object(console, 'STATE', Path(folder)), \
             patch.object(console, 'session_args', return_value=['rviz']), \
             patch.object(console.subprocess, 'Popen') as launch:
            app.open_view('navigation')
        launch.assert_called_once()


if __name__ == '__main__':
    unittest.main()
