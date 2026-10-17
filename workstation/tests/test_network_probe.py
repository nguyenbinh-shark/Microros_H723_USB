from pathlib import Path
import queue
import socket
import subprocess
import sys
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import network_probe as probe
import rpbot_console as console


class ResolutionTests(unittest.TestCase):
    def test_numeric_ip_does_not_need_dns_or_wsl(self):
        fallback = Mock()
        with patch.object(probe.socket, 'gethostbyname') as lookup:
            self.assertEqual(probe.resolve_ipv4('192.168.2.10', fallback), '192.168.2.10')
        lookup.assert_not_called()
        fallback.assert_not_called()

    def test_local_hostname_uses_windows_resolver_first(self):
        fallback = Mock()
        with patch.object(probe.socket, 'gethostbyname', return_value='192.168.2.10') as lookup:
            self.assertEqual(probe.resolve_ipv4('sharkAGV.local', fallback), '192.168.2.10')
        lookup.assert_called_once_with('sharkAGV.local')
        fallback.assert_not_called()

    def test_wsl_fallback_when_windows_cannot_resolve(self):
        with patch.object(probe.socket, 'gethostbyname', side_effect=socket.gaierror()):
            fallback = Mock(return_value='192.168.2.10 STREAM sharkAGV.local\n192.168.2.10 DGRAM\n')
            self.assertEqual(probe.resolve_ipv4('sharkAGV.local', fallback), '192.168.2.10')

    def test_failed_resolution_has_actionable_error(self):
        with patch.object(probe.socket, 'gethostbyname', side_effect=socket.gaierror()):
            for fallback in (None, Mock(return_value=''), Mock(side_effect=RuntimeError('missing getent')),
                             Mock(side_effect=subprocess.TimeoutExpired('getent', 5))):
                with self.assertRaisesRegex(RuntimeError, r'sharkAGV.local.*cùng mạng.*mDNS'):
                    probe.resolve_ipv4('sharkAGV.local', fallback)

    def test_subsequent_lookup_can_find_a_changed_ip(self):
        with patch.object(probe.socket, 'gethostbyname', side_effect=['192.168.2.10', '192.168.2.20']):
            self.assertNotEqual(probe.resolve_ipv4('sharkAGV.local'), probe.resolve_ipv4('sharkAGV.local'))

    def test_ssh_uses_resolved_address_and_still_verifies_hostname_identity(self):
        settings = dict(console.DEFAULTS, host='sharkAGV.local', user='sharkagv')
        with patch.object(console, 'resolve_robot_address', return_value='192.168.2.10'), \
             patch.object(console, 'capture', return_value='ok') as capture:
            console.remote(settings, 'status')
        args = capture.call_args.args[0]
        self.assertIn('HostName=192.168.2.10', args)
        self.assertIn('HostKeyAlias=sharkagv.local', args)
        self.assertEqual(args[-2], 'sharkagv@sharkAGV.local')
        self.assertIn('BatchMode=yes', args)
        self.assertFalse(any('StrictHostKeyChecking=no' in arg or 'UserKnownHostsFile=' in arg for arg in args))

    def test_dds_uses_the_same_resolver_even_if_wsl_cannot_resolve_local(self):
        import tempfile
        with tempfile.TemporaryDirectory() as folder, patch.object(console, 'STATE', Path(folder)), \
             patch.object(console, 'resolve_robot_address', return_value='192.168.2.10') as resolver, \
             patch.object(console, 'pc_profile', return_value='<profiles/>') as profile:
            settings = dict(console.DEFAULTS, host='sharkAGV.local')
            console.session_args(settings, 'stream', dict(domain=0, profile_xml=''))
        resolver.assert_called_once_with(settings)
        profile.assert_called_once_with('', '192.168.2.10', 0)

    def test_dds_reuses_worker_resolved_address_without_dns_on_tk_thread(self):
        import tempfile
        with tempfile.TemporaryDirectory() as folder, patch.object(console, 'STATE', Path(folder)), \
             patch.object(console, 'resolve_robot_address') as resolver:
            settings = dict(console.DEFAULTS, host='sharkAGV.local')
            console.session_args(settings, 'stream', dict(domain=0, profile_xml=''), address='192.168.2.10')
            self.assertIn('192.168.2.10', (Path(folder) / 'fastdds.xml').read_text(encoding='utf-8'))
        resolver.assert_not_called()


class PingTests(unittest.TestCase):
    def run_ping(self, text, code=0):
        with patch.object(probe.subprocess, 'run', return_value=subprocess.CompletedProcess([], code, text, '')) as run:
            result = probe.ping_ipv4('192.168.2.10')
        self.assertEqual(run.call_args.args[0][-1], '192.168.2.10')
        self.assertEqual(run.call_args.kwargs['timeout'], 8)
        self.assertNotIn('shell', run.call_args.kwargs)
        return result

    def test_echo_replies_show_success_and_address(self):
        result = self.run_ping('Reply from 192.168.2.10: bytes=32 time<1ms TTL=64\n'
                               'Reply from 192.168.2.10: bytes=32 time=4ms TTL=64\n')
        self.assertEqual(result.replies, 2)
        self.assertEqual(result.level, 'ok')
        self.assertIn('2/2', result.summary)
        self.assertIn('192.168.2.10', result.summary)

    def test_unreachable_exit_zero_is_not_success(self):
        result = self.run_ping('Reply from 192.168.2.1: Destination host unreachable.\n'
                               'Packets: Sent = 2, Received = 2, Lost = 0 (0% loss)\n')
        self.assertEqual(result.replies, 0)
        self.assertEqual(result.level, 'warning')
        self.assertIn('SSH', result.summary)

    def test_partial_reply_is_warning(self):
        result = self.run_ping('Reply from 192.168.2.10: bytes=32 time=4ms TTL=64\nRequest timed out.')
        self.assertEqual(result.replies, 1)
        self.assertEqual(result.level, 'warning')

    def test_ping_timeout_cannot_be_presented_as_success(self):
        with patch.object(probe.subprocess, 'run', side_effect=subprocess.TimeoutExpired('ping', 8)):
            with self.assertRaises(subprocess.TimeoutExpired):
                probe.ping_ipv4('192.168.2.10')

    def test_ping_never_passes_hostnames_or_options_to_a_shell(self):
        with patch.object(probe.subprocess, 'run') as run:
            for address in ('sharkAGV.local', '-t', '192.168.2.10; shutdown'):
                with self.assertRaises(ValueError):
                    probe.ping_ipv4(address)
        run.assert_not_called()


class PingGuiTests(unittest.TestCase):
    def setUp(self):
        defaults = patch.dict(console.DEFAULTS, host='fixture.local', user='operator')
        defaults.start()
        self.addCleanup(defaults.stop)
        self.app = app = console.Console.__new__(console.Console)
        app.config = app.current_ping_settings = Mock(return_value=dict(console.DEFAULTS))
        app.ping_button, app.set_ping_status, app.append = Mock(), Mock(), Mock()
        app.root = Mock()
        app.closed = False
        app.pinging = False
        app.events = queue.Queue()

    def test_ping_runs_in_background_and_does_not_start_robot_tasks(self):
        app = self.app
        with patch.object(console.threading, 'Thread') as thread, \
             patch.object(console, 'ping_robot', return_value=probe.PingResult('192.168.2.10', 2, 'fixture')):
            app.ping_device()
            self.assertTrue(app.pinging)
            thread.return_value.start.assert_called_once()
            thread.call_args.kwargs['target']()
        app.drain()
        self.assertFalse(app.pinging)
        self.assertEqual(app.set_ping_status.call_args.args[1], 'ok')

    def test_old_host_reply_does_not_show_green_for_a_new_device(self):
        app = self.app
        app.current_ping_settings.return_value = dict(console.DEFAULTS, host='other.local')
        app.events.put(('ping', True, (dict(console.DEFAULTS), probe.PingResult('192.168.2.10', 2, 'fixture')), None))
        app.drain()
        self.assertEqual(app.set_ping_status.call_args.args[1], 'unknown')
        app.append.assert_not_called()

    def test_resolution_error_keeps_ssh_metadata_untouched(self):
        app = self.app
        app.metadata_failed = Mock()
        app.events.put(('ping', False, (dict(console.DEFAULTS), 'Không tìm được IPv4'), None))
        app.drain()
        self.assertEqual(app.set_ping_status.call_args.args[1], 'error')
        app.metadata_failed.assert_not_called()


class DeviceSettingsTests(unittest.TestCase):
    def test_fresh_install_has_no_assumed_device_or_ssh_account(self):
        self.assertEqual(console.DEFAULTS['host'], '')
        self.assertEqual(console.DEFAULTS['user'], '')

    def test_arbitrary_hostnames_and_ssh_users_are_accepted(self):
        for host, user in (('rover-2.local', 'operator'), ('my-pi.local', 'ubuntu'),
                           ('warehouse-agv', 'robot_user'), ('192.168.5.20', 'admin')):
            settings = dict(console.DEFAULTS, host=host, user=user)
            self.assertEqual(console.validate_settings(settings), settings)

    def test_ping_needs_no_ssh_account(self):
        app = console.Console.__new__(console.Console)
        values = dict(console.DEFAULTS, host='rover-2.local', user='')
        app.fields = {key: Mock(get=Mock(return_value=value)) for key, value in values.items()}
        self.assertEqual(app.current_ping_settings(), values)
        with self.assertRaisesRegex(ValueError, 'SSH'):
            app.current_settings()


if __name__ == '__main__':
    unittest.main()
