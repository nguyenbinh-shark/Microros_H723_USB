"""Offline connection outage/recovery check through the real Tk event loop."""
import json
from pathlib import Path
import sys
import tempfile
import time
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import rpbot_console as console


class FixtureStream:
    instances = []

    def __init__(self, *args, **kwargs):
        self.running, self.stalled = True, False
        self.frames, self.updated = 0, 0
        self.instances.append(self)

    def alive(self):
        return self.running

    def snapshot(self):
        if not self.stalled:
            self.frames += 1
            self.updated = time.monotonic()
        topics = {name: dict(age_s=0.1, messages=self.frames) for name in
                  ('/map', '/scan', '/odom', '/imu/data', '/motor_fb', '/motor_status')}
        data = dict(topics=topics, map=dict(width=2, height=2, resolution=0.05), tf_map_to_base=True,
                    mapping_ready=True, navigation_available=False, motor_status=0)
        return data, self.updated, self.frames

    def close(self):
        self.running = False


def main():
    inspections, requests, errors = [], [], []
    session = dict(pid=12345, domain=0, mode='mapping', simulated_time=False, profile_xml='')
    def inspect(settings):
        inspections.append(time.monotonic())
        if len(inspections) <= 2:
            raise RuntimeError('Offline fixture: network unavailable')
        return dict(robot=dict(hostname='Offline fixture', service={}), session=session, data=None)
    def remote(settings, action, *args):
        requests.append((action, args))
        assert action in ('maps', 'save'), 'No actual robot control is allowed'
        return json.dumps(dict(maps=[])) if action == 'maps' else json.dumps(dict(saved='fixture.yaml'))
    with tempfile.TemporaryDirectory() as folder, patch.object(console, 'STATE', Path(folder)), \
         patch.object(console, 'inspect_robot', side_effect=inspect), \
         patch.object(console, 'remote', side_effect=remote), \
         patch.object(console, 'TelemetryStream', FixtureStream), \
         patch.object(console, 'session_args', return_value=['offline-fixture']), \
         patch.object(console.messagebox, 'showinfo') as info, \
         patch.object(console.messagebox, 'showerror') as error_dialog:
        (Path(folder) / 'settings.json').write_text(json.dumps(
            dict(console.DEFAULTS, host='fixture.local', user='operator')), encoding='utf-8')
        root = console.create_root()
        app = console.Console(root)
        result = dict(ok=False, phase='outage', stream=None)
        def failure(kind, error, trace):
            import traceback
            traceback.print_exception(kind, error, trace)
            errors.append(str(error))
            app.close()
        root.report_callback_exception = failure
        def poll():
            if result['phase'] == 'outage':
                if app.last_data is None or app.busy:
                    root.after(100, poll)
                    return
                assert len(inspections) >= 3, 'The GUI must recover without clicking Check'
                assert app.connection.failures == 0
                app.last_meta = time.monotonic()-60
                app.map_name.set('connection_fixture')
                button = next(button for button in app.actions if button.cget('text') == 'Lưu map')
                button.invoke()
                result['phase'] = 'action'
            elif result['phase'] == 'action':
                if app.busy or not any(action == 'save' for action, _ in requests):
                    root.after(100, poll)
                    return
                assert len(inspections) == 4, 'Save must refresh metadata by itself'
                assert sum(action == 'save' for action, _ in requests) == 1
                result['stream'] = app.stream
                app.stream.stalled = True
                app.stream.updated = time.monotonic()-10
                result['phase'] = 'stream'
            else:
                if app.stream is result['stream'] or app.last_data is None:
                    root.after(100, poll)
                    return
                assert not result['stream'].alive()
                assert len(FixtureStream.instances) == 2
                assert len(inspections) == 4, 'Recover only the PC subscriber, without another SSH check'
                info.assert_not_called()
                error_dialog.assert_not_called()
                assert app.pending_session is None
                assert all(not button.instate(['disabled']) for button in app.actions + app.stop_buttons)
                result['ok'] = True
                print('PASS: automatic network recovery, Save refreshes and resumes once, stalled live subscriber recovers; no Check clicks, dialogs, SSH or ROS')
                app.close()
                return
            root.after(100, poll)
        root.after(100, poll)
        root.after(15000, app.close)
        root.mainloop()
        if not result['ok']:
            raise SystemExit('Connection UI check failed in ' + result['phase'] + ': ' + '; '.join(errors))


if __name__ == '__main__':
    main()
