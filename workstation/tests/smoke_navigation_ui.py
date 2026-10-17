"""Exercise the native Nav start button offline; no robot requests are sent."""
import json
from pathlib import Path
import sys
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import rpbot_console as console


def main():
    calls, errors = [], []
    def remote(settings, action, *args):
        assert action == 'start-nav', action
        calls.append((action, args))
        return json.dumps(dict(status='started'))
    with patch.object(console.Console, 'check'), patch.object(console.Console, 'tick'), \
         patch.object(console.Console, 'preview_map'), patch.object(console, 'remote', side_effect=remote), \
         patch.object(console.messagebox, 'showinfo') as info:
        root = console.create_root()
        app = console.Console(root)
        app.auto.set(False)
        result = {'ok': False}
        def failure(kind, error, trace):
            import traceback
            traceback.print_exception(kind, error, trace)
            errors.append(str(error))
            app.close()
        root.report_callback_exception = failure
        def begin():
            app.tabs.select(app.tab_frames['navigation'])
            root.update_idletasks()
            button = next(button for button in app.actions if button.cget('text') == 'Bắt đầu Nav')
            assert button.winfo_ismapped(), 'Start Nav button must be visible'
            button.invoke()
            info.assert_called_once()
            assert not calls
            assert app.tabs.select() == str(app.tab_frames['maps'])
            app.map_settings = app.config()
            app.map_records = {'room': dict(name='room', revision='test-revision')}
            app.maps_tree.insert('', 'end', iid='room', text='room')
            app.maps_tree.selection_set('room')
            app.tabs.select(app.tab_frames['navigation'])
            button.invoke()
            root.after(100, verify)
        def verify():
            if app.busy:
                root.after(100, verify)
                return
            assert calls == [('start-nav', ('--name', 'room', '--revision', 'test-revision', '--domain', '0'))]
            assert all(not button.instate(['disabled']) for button in app.actions + app.stop_buttons)
            assert 'Đang kiểm tra Nav2' in app.status['navigation'].get()
            result['ok'] = True
            print('PASS: visible Start Nav button, missing-selection guidance, selected map/revision/domain request and controls restored; offline')
            app.close()
        root.after(100, begin)
        root.after(10000, app.close)
        root.mainloop()
        if not result['ok']:
            raise SystemExit('Nav UI check failed: ' + '; '.join(errors))


if __name__ == '__main__':
    main()
