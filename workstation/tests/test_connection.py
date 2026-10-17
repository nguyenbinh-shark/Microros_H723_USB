import json
from pathlib import Path
import queue
import sys
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import rpbot_console as console
from connection_state import ConnectionState


class ConnectionTimingTests(unittest.TestCase):
    def test_network_outage_backs_off_and_logs_at_most_once_a_minute(self):
        state = ConnectionState()
        now, logged, delays = 100, [], []
        for _ in range(7):
            logged.append(state.metadata_failed(now, 'network unreachable'))
            delays.append(state.next_check - now)
            now = state.next_check
        self.assertEqual(delays, [2, 4, 8, 15, 15, 15, 15])
        self.assertEqual(sum(logged), 1)
        self.assertTrue(state.metadata_failed(161, 'timeout'))
        self.assertTrue(state.metadata_ok(162, has_session=True))
        self.assertEqual(state.failures, 0)
        self.assertEqual(state.next_check, 192)
        self.assertFalse(state.metadata_ok(163, has_session=False))
        self.assertEqual(state.next_check, 168)

    def test_stalled_but_alive_subscriber_gets_restarted_after_grace(self):
        state = ConnectionState()
        state.stream_attempt(100)
        self.assertFalse(state.restart_due(110, alive=True, updated=0))
        self.assertTrue(state.restart_due(115, alive=True, updated=0))
        state.stream_received()
        self.assertFalse(state.restart_due(120, alive=True, updated=115))
        self.assertTrue(state.restart_due(123, alive=True, updated=115))
        self.assertTrue(state.restart_due(120, alive=False, updated=119))

    def test_repeated_start_failures_do_not_spin_and_valid_frames_reset_backoff(self):
        state = ConnectionState()
        for attempt, delay in enumerate((2, 4, 8, 16, 30, 30)):
            now = 100 + attempt * 50
            state.stream_attempt(now)
            self.assertFalse(state.restart_due(now + delay - 0.1, alive=False, updated=0))
            self.assertTrue(state.restart_due(now + delay, alive=False, updated=0))
        state.stream_recovering = True
        self.assertTrue(state.stream_received())
        self.assertFalse(state.stream_received())
        state.stream_attempt(500)
        self.assertEqual(state.stream_retry_at, 502)


class ConnectionGuiTests(unittest.TestCase):
    def setUp(self):
        defaults = patch.dict(console.DEFAULTS, host='fixture.local', user='operator')
        defaults.start()
        self.addCleanup(defaults.stop)
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        state_patch = patch.object(console, 'STATE', Path(folder.name))
        state_patch.start()
        self.addCleanup(state_patch.stop)
        self.app = app = console.Console.__new__(console.Console)
        app.connection = ConnectionState()
        app.settings = app.connected_settings = dict(console.DEFAULTS)
        app.current_settings = Mock(return_value=dict(console.DEFAULTS))
        app.config = Mock(return_value=dict(console.DEFAULTS))
        app.session = dict(pid=10, domain=0, mode='mapping', simulated_time=False, profile_xml='')
        app.map_settings = dict(console.DEFAULTS)
        app.stream_key = (tuple(sorted(console.DEFAULTS.items())), 10, 0, '')
        app.stream = Mock()
        app.stream.alive.return_value = True
        app.stream.snapshot.side_effect = lambda: ({'topics': {}}, time.monotonic(), 1)
        app.last_meta = time.monotonic() - 60
        app.last_data = None
        app.last_check = 0
        app.pending_session = None
        app.connecting = False
        app.requested_settings = dict(console.DEFAULTS)
        app.closed = app.busy = app.stop_busy = False
        app.events = queue.Queue()
        app.status = {key: Mock() for key in ('connection', 'boot', 'mapping', 'sensors', 'motion', 'navigation')}
        app.root, app.append, app.task = Mock(), Mock(), Mock()
        app.topic_tree, app.task_domain, app.activity = Mock(), Mock(), Mock()
        app.topic_tree.get_children.return_value = ()
        app.start_stream = Mock()
        app.actions = app.stop_buttons = []
        app.auto = Mock()
        app.auto.get.return_value = True
        app.viewers = {}
        app.stream_seen = 1

    def report(self, session=None):
        return dict(robot=dict(hostname='fixture', service={}), session=session or dict(self.app.session), data=None)

    def flush_resume(self):
        callbacks = [call.args[1] for call in self.app.root.after.call_args_list if call.args[0] == 0]
        for callback in callbacks:
            callback()

    def test_expired_session_is_refreshed_then_original_button_runs_without_manual_check(self):
        app = self.app
        app.check = Mock()
        app.map_name = Mock()
        app.map_name.get.return_value = 'room'
        with patch.object(console.messagebox, 'showinfo') as dialog:
            app.save_map()
            app.task.assert_not_called()
            app.check.assert_called_once_with(quiet=True)
            app.events.put(('metadata', True, (dict(console.DEFAULTS), self.report()), app.checked))
            app.drain()
            self.flush_resume()
        dialog.assert_not_called()
        app.task.assert_called_once()
        with patch.object(console, 'remote', return_value='saved') as remote:
            app.task.call_args.args[1]()
        remote.assert_called_once_with(console.DEFAULTS, 'save', '--name', 'room', '--domain', '0')

    def test_rviz_can_use_fresh_ros_when_ssh_is_down(self):
        app = self.app
        app.connection.metadata_failed(time.monotonic(), 'offline')
        app.check = Mock()
        callback = Mock()
        self.assertEqual(app.require_session(callback, allow_telemetry=True), console.DEFAULTS)
        app.check.assert_not_called()
        self.assertIsNone(app.pending_session)

    def test_pending_reset_is_not_applied_to_a_replacement_session(self):
        app = self.app
        callback = Mock()
        app.check = Mock()
        app.require_session(callback)
        replacement = dict(app.session, pid=11)
        app.checked((dict(console.DEFAULTS), self.report(replacement)))
        self.flush_resume()
        callback.assert_not_called()
        self.assertIsNone(app.pending_session)

    def test_failed_refresh_clears_pending_action_before_later_reconnection(self):
        app = self.app
        callback = Mock()
        app.check = Mock()
        app.require_session(callback)
        app.events.put(('metadata', False, (dict(console.DEFAULTS), 'offline'), None))
        app.drain()
        self.assertIsNone(app.pending_session)
        app.checked((dict(console.DEFAULTS), self.report()))
        self.flush_resume()
        callback.assert_not_called()

    def test_old_host_reply_cannot_overwrite_the_current_connection(self):
        app = self.app
        replacement = dict(console.DEFAULTS, host='192.168.0.121')
        app.current_settings.return_value = replacement
        app.events.put(('metadata', True, (dict(console.DEFAULTS), self.report()), app.checked))
        app.drain()
        self.assertEqual(app.connected_settings, console.DEFAULTS)
        self.assertLess(app.last_meta, time.monotonic() - 30)
        self.assertEqual(app.connection.next_check, 0)

    def test_overdue_pending_action_does_not_execute(self):
        app = self.app
        callback = Mock()
        app.pending_session = (dict(console.DEFAULTS), app.session_identity(app.session), time.monotonic()-1, callback)
        app.resume_pending()
        self.flush_resume()
        callback.assert_not_called()

    def test_live_but_stalled_process_is_recovered_without_ssh_check(self):
        app = self.app
        app.stream.snapshot.return_value = ({}, time.monotonic()-10, 1)
        app.stream.snapshot.side_effect = None
        app.connection.next_check = time.monotonic()+30
        app.start_stream, app.check = Mock(), Mock()
        app.tick()
        app.start_stream.assert_called_once_with(console.DEFAULTS, app.stream_key)
        app.check.assert_not_called()
        self.assertIsNone(app.last_data)

    def test_repeated_metadata_failures_keep_retrying_without_log_spam_or_manual_prompt(self):
        app = self.app
        app.check = Mock()
        with patch.object(console.time, 'monotonic', return_value=100):
            app.metadata_failed('offline')
        with patch.object(console.time, 'monotonic', return_value=101):
            app.metadata_failed('offline')
            app.tick()
        app.check.assert_not_called()
        self.assertEqual(sum('Chi tiết' in call.args[0] for call in app.append.call_args_list), 1)
        with patch.object(console.time, 'monotonic', return_value=106):
            app.tick()
        app.check.assert_called_once_with(quiet=True)

    def test_background_check_does_not_replace_ready_activity(self):
        app = self.app
        with patch.object(console.threading, 'Thread') as worker:
            app.check(quiet=True)
        app.activity.set.assert_not_called()
        worker.return_value.start.assert_called_once()

    def test_automatic_map_refresh_does_not_preempt_a_pending_user_action(self):
        app = self.app
        app.pending_session = (console.DEFAULTS, None, time.monotonic()+20, Mock())
        app.refresh_maps(background=True)
        app.task.assert_not_called()

    def test_changing_robot_stops_displaying_old_telemetry_and_connects_to_new_address(self):
        app = self.app
        app.current_settings.return_value = dict(console.DEFAULTS, host='192.168.0.121')
        app.check = Mock()
        with patch.object(console.threading, 'Thread') as worker:
            app.tick()
        self.assertIsNone(app.session)
        self.assertIsNone(app.last_data)
        self.assertIsNone(app.stream)
        app.check.assert_called_once_with(quiet=True)
        worker.return_value.start.assert_called_once()

    def test_normal_metadata_refresh_does_not_flash_activity_when_ros_is_healthy(self):
        self.app.checked((dict(console.DEFAULTS), self.report()))
        self.app.activity.set.assert_not_called()

    def test_no_session_reply_clears_old_ros_connection_indicator(self):
        app = self.app
        report = self.report()
        report['session'] = None
        with patch.object(console.threading, 'Thread'):
            app.checked((dict(console.DEFAULTS), report))
        self.assertIsNone(app.stream)
        self.assertIsNone(app.last_data)
        app.status['connection'].set.assert_called_once_with(f"SSH: fixture ({console.DEFAULTS['host']})")

    def test_changed_hostname_address_recreates_dds_peers_without_changing_robot_session(self):
        app = self.app
        app.stream_key += ('192.168.2.10',)
        report = dict(self.report(), address='192.168.2.20')
        app.checked((dict(console.DEFAULTS), report))
        self.assertEqual(app.robot_address, '192.168.2.20')
        app.start_stream.assert_called_once()
        self.assertEqual(app.start_stream.call_args.args[1][-1], '192.168.2.20')


if __name__ == '__main__':
    unittest.main()
