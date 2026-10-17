"""Offline GUI check with dummy task metadata; never sends requests to the Pi."""
import json
from pathlib import Path
import sys
import time
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import rpbot_console as console


def main():
    calls = []
    entries = [dict(pid=101, start_ticks='1000', kind='launch', mode='mapping', label='Mapping', domain=0, source='Ngoài Console'),
               dict(pid=102, start_ticks='2000', kind='launch', mode='navigation', label='Navigation', domain=0, source='Ngoài Console'),
               dict(pid=103, start_ticks='3000', kind='bag', mode='record', label='Ghi bag: test', domain=0, source='Ngoài Console')]
    def remote(settings, action, *args):
        calls.append(action)
        assert action == 'tasks', 'Offline check must never send stop or motor requests'
        scope = args[args.index('--scope')+1] if '--scope' in args else 'all'
        return json.dumps(dict(tasks=[item for item in entries if scope == 'all' or item['mode'] == scope]))
    metadata = dict(robot=dict(hostname='Offline UI check', service={}, maps=[]), session=None, data=None)
    with patch.object(console, 'inspect_robot', return_value=metadata), patch.object(console, 'remote', side_effect=remote), \
         patch.object(console.messagebox, 'askyesno', return_value=False):
        root = console.create_root()
        app = console.Console(root)
        app.auto.set(False)
        result = {'ok': False}
        def failure(kind, error, trace):
            import traceback
            traceback.print_exception(kind, error, trace)
            app.close()
        root.report_callback_exception = failure
        def begin():
            app.tabs.select(app.tab_frames['tasks'])
            app.stop_robot_tasks('navigation')
            root.after(800, verify)
        def verify():
            if app.busy or app.stop_busy:
                root.after(100, verify)
                return
            if not app.task_records:
                app.refresh_tasks()
                root.after(200, verify)
                return
            assert len(app.tasks_tree.get_children()) == 3
            assert calls and all(action == 'tasks' for action in calls)
            app.task('UI responsiveness check', lambda: time.sleep(0.5))
            assert app.busy and all(not button.instate(['disabled']) for button in app.stop_buttons)
            root.after(700, capture)
        def capture():
            root.lift()
            root.attributes('-topmost', True)
            root.update()
            try:
                from PIL import ImageGrab
                ImageGrab.grab(bbox=(root.winfo_rootx(), root.winfo_rooty(),
                                    root.winfo_rootx()+root.winfo_width(), root.winfo_rooty()+root.winfo_height())).save(
                                        console.STATE / 'console-tasks-preview.png')
            except ImportError:
                pass
            print('PASS: task list, cancel confirmation and stop buttons available during normal work; no Pi requests')
            result['ok'] = True
            app.close()
        root.after(800, begin)
        root.after(10000, app.close)
        root.mainloop()
        if not result['ok']:
            raise SystemExit('Offline stop GUI check failed')


if __name__ == '__main__':
    main()
