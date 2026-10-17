"""Exercise hostname/ping controls in the real Tk event loop, with no network."""
import json
from pathlib import Path
import sys
import tempfile
import threading
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import rpbot_console as console
from network_probe import PingResult
from status_dashboard import COLORS
from preview_studio import capture_window


def main():
    output = console.STATE / 'studio-preview'
    output.mkdir(parents=True, exist_ok=True)
    release, calls, errors = threading.Event(), [], []

    def ping(settings):
        calls.append(dict(settings))
        if not release.wait(3):
            raise RuntimeError('Fixture worker was not released')
        if len(calls) == 3:
            raise RuntimeError('Fixture: Không tìm được IPv4')
        return PingResult('192.168.2.10', 0 if len(calls) == 2 else 2, 'DỮ LIỆU PING GIẢ')

    with tempfile.TemporaryDirectory() as folder, patch.object(console, 'STATE', Path(folder)), \
         patch.object(console.Console, 'check'), patch.object(console.Console, 'tick'), \
         patch.object(console, 'ping_robot', side_effect=ping), \
         patch.object(console, 'remote') as remote:
        root = console.create_root()
        app = console.Console(root)
        root.report_callback_exception = lambda kind, error, trace: errors.append(str(error))
        root.title('KIỂM THỬ GIAO DIỆN · DỮ LIỆU PING GIẢ · RPBot Studio')
        app.tabs.select(app.tab_frames['settings'])
        app.activity.set('KIỂM THỬ · NGOẠI TUYẾN · DỮ LIỆU GIẢ')
        assert app.fields['host'].get() == '' and app.fields['user'].get() == ''
        assert app.tabs.select() == str(app.tab_frames['settings'])
        assert not any('sharkAGV' in button.cget('text') for button in app.actions)
        app.fields['host'].set('rover-2.local')
        # A network check must also work before the SSH account is entered.
        result = dict(phase=0, ok=False)

        def poll():
            if app.pinging:
                root.after(30, poll)
                return
            if result['phase'] == 0:
                assert app.ping_label.cget('background') == COLORS['ok'][0]
                assert '2/2' in app.ping_status.get()
                for size in ('1420x920', '1080x740'):
                    root.geometry(size)
                    root.update()
                    canvas = app.page_scrolls['settings'].canvas
                    assert app.ping_button.winfo_rooty() >= canvas.winfo_rooty()
                    assert (app.ping_button.winfo_rooty() + app.ping_button.winfo_height()
                            <= canvas.winfo_rooty() + canvas.winfo_height()), ('Ping hidden', size)
                    for entry in app.device_entries.values():
                        assert entry.winfo_rooty() >= canvas.winfo_rooty()
                        assert (entry.winfo_rooty() + entry.winfo_height()
                                <= canvas.winfo_rooty() + canvas.winfo_height()), ('Device field hidden', size)
                    capture_window(root, output / f'ping-fixture-{size}.png')
                assert calls[0]['host'] == 'rover-2.local' and calls[0]['user'] == ''
                app.fields['user'].set('operator')
                app.config()
                saved = json.loads((Path(folder) / 'settings.json').read_text(encoding='utf-8'))
                assert saved['host'] == 'rover-2.local' and saved['user'] == 'operator'
                result['phase'] = 1
                app.ping_button.invoke()
            elif result['phase'] == 1:
                assert app.ping_label.cget('background') == COLORS['warning'][0]
                assert 'SSH' in app.ping_status.get()
                result['phase'] = 2
                app.ping_button.invoke()
            elif result['phase'] == 2:
                assert app.ping_label.cget('background') == COLORS['error'][0]
                release.clear()
                result['phase'] = 3
                app.ping_button.invoke()
                app.fields['host'].set('other.local')
                root.after(100, release.set)
            else:
                assert app.ping_label.cget('background') == COLORS['unknown'][0]
                assert not app.connection.failures and app.session is None
                remote.assert_not_called()
                result['ok'] = True
                app.close()
                return
            root.after(30, poll)

        app.ping_button.invoke()
        assert app.pinging and app.ping_button.instate(['disabled'])
        # This callback must run while the worker is waiting: ping cannot block Tk.
        root.after(100, release.set)
        root.after(150, poll)
        root.after(6000, app.close)
        root.mainloop()
        if errors or not result['ok']:
            raise SystemExit(f'Ping GUI failed: {result}, {errors}')
    print('PASS: empty first-run fields, arbitrary hostname/account saved, ping before SSH setup, '
          'nonblocking success/loss/error states, stale replies ignored, fields and controls visible '
          'at both sizes; no SSH or robot commands')


if __name__ == '__main__':
    main()
