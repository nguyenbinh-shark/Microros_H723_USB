"""Offline language switching, persistence and unchanged robot session identity."""
import json
from pathlib import Path
import sys
import tempfile
import time
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import rpbot_console as console
from console_layout import PAGES
from preview_studio import capture_window, check_vietnamese_fonts
from test_robot_status import healthy_data


def main():
    output = Path(__file__).resolve().parents[1] / 'build' / 'language-preview'
    output.mkdir(parents=True, exist_ok=True)
    errors = []
    with tempfile.TemporaryDirectory(prefix='studio-language-') as folder, \
         patch.object(console, 'STATE', Path(folder)), \
         patch.object(console.Console, 'check') as check, patch.object(console.Console, 'tick'), \
         patch.object(console.Console, 'refresh_tasks'), patch.object(console.Console, 'refresh_maps'), \
         patch.object(console, 'remote') as remote:
        saved = dict(console.DEFAULTS, host='fixture.local', user='operator', language='en')
        config = Path(folder) / 'settings.json'
        config.write_text(json.dumps(saved), encoding='utf-8')
        root = console.create_root()
        app = console.Console(root)
        def fail(kind, error, trace):
            import traceback
            traceback.print_exception(kind, error, trace)
            errors.append(str(error))
        root.report_callback_exception = fail
        root.update()
        def descendants(parent):
            for child in parent.winfo_children():
                yield child
                yield from descendants(child)
        title_label = next(widget for widget in descendants(root) if 'textvariable' in widget.keys()
                           and str(widget.cget('textvariable')) == str(app.page_title))
        assert app.i18n.code == 'en'
        assert app.current_settings() == app.settings and 'language' not in app.settings
        assert app.sidebar_buttons['maps'].cget('text') == 'Saved maps'
        assert app.ping_button.cget('text') == 'Ping device'
        app.session = dict(pid=123, mode='navigation', domain=0)
        app.connected_settings = app.current_settings()
        app.last_meta = time.monotonic()
        data = healthy_data()
        app.render_data(data)
        app.dashboard.refresh()
        root.update()
        assert app.dashboard.cards['connection'].value.cget('text') == 'RECEIVING ROS'
        assert app.dashboard.cards['navigation'].value.cget('text') == 'SERVER FOUND'
        assert app.status['mapping'].get().find('m/cell') != -1
        assert app.topic_tree.item('/odom', 'values')[2] == 'Receiving'
        for size in ('1420x920', '1080x740'):
            root.geometry(size)
            root.update()
            for key, *_ in PAGES:
                app.sidebar_buttons[key].invoke()
                root.update()
                assert app.tabs.select() == str(app.tab_frames[key])
                original_title = next(page[3] for page in PAGES if page[0] == key)
                assert app.page_title.get() == app.i18n.text(original_title)
                assert title_label.cget('text') == app.page_title.get(), 'Tk label must render the translated value'
                for button in app.actions + app.stop_buttons:
                    if button.winfo_ismapped() and str(app.page_scrolls[key].content) in str(button):
                        assert button.winfo_width() >= button.winfo_reqwidth() - 2, (size, key, button.cget('text'))
                stop = next(button for button in app.stop_buttons if button.cget('text') == 'Stop all robot tasks')
                assert stop.winfo_ismapped() and stop.winfo_width() >= stop.winfo_reqwidth() - 2
                assert app.language_picker.combobox.winfo_ismapped()
                assert capture_window(root, output / f'en-{key}-{size}.png')
        name = 'mapping_room_01'
        app.maps_tree.insert('', 'end', iid=name, text=name, values=('Lỗi file', ''))
        app.map_name.set('custom_map_02')
        app.selected_map_text.set('Map đã chọn: ' + name + ' (chưa nạp vào navigation)')
        app.tasks_tree.insert('', 'end', iid='999', text='Phiên robot', values=('999', 0, 'Ngoài Console'))
        assert app.maps_tree.item(name, 'text') == name
        assert app.maps_tree.item(name, 'values')[0] == 'File error'
        assert app.tasks_tree.item('999', 'text') == 'Robot session'
        assert app.tasks_tree.item('999', 'values')[2] == 'Outside Studio'
        assert app.selected_map_text.get() == 'Selected map: mapping_room_01 (not loaded into navigation)'
        session, settings, connection = app.session, app.connected_settings, app.connection
        stream = app.stream = Mock()
        app.busy = True
        app.actions[0].state(['disabled'])
        before_checks = check.call_count
        for code, label, text in (('vi', 'Tiếng Việt', 'Bản đồ đã lưu'), ('en', 'English', 'Saved maps'),
                                  ('vi', 'Tiếng Việt', 'Bản đồ đã lưu')):
            app.language_picker.variable.set(label)
            app.language_picker.combobox.event_generate('<<ComboboxSelected>>')
            root.update()
            assert app.i18n.code == code and app.sidebar_buttons['maps'].cget('text') == text
            assert app.busy and app.actions[0].instate(['disabled'])
            assert app.session is session and app.connected_settings is settings and app.connection is connection
            assert app.stream is stream and app.current_settings() == app.settings
            assert app.map_name.get() == 'custom_map_02'
            assert app.maps_tree.item(name, 'text') == name
            assert json.loads(config.read_text(encoding='utf-8')) == dict(app.settings, language=code)
            assert check.call_count == before_checks
            stream.close.assert_not_called()
            remote.assert_not_called()
        assert app.topic_tree.item('/odom', 'values')[2] == 'Đang nhận'
        assert app.tasks_tree.item('999', 'text') == 'Phiên robot'
        assert app.selected_map_text.get() == 'Map đã chọn: mapping_room_01 (chưa nạp vào navigation)'
        app.busy = False
        app.stream = None
        app.change_language('en')
        app.append('Lỗi: không thấy file')
        assert 'Error: file not found' in app.log.get('1.0', 'end')
        with patch.object(console.messagebox, 'showerror') as dialog:
            app.dialogs.showerror('Tên map', 'Dùng 1–64 chữ không dấu, số, dấu - hoặc _.')
            assert dialog.call_args.args == ('Map name', 'Use 1–64 ASCII letters, digits, hyphens or underscores.')
        check_vietnamese_fonts(root)
        app.close()
        # A new window must read the saved language without changing robot settings.
        root = console.create_root()
        app = console.Console(root)
        root.update()
        assert app.i18n.code == 'en' and app.ping_button.cget('text') == 'Ping device'
        assert app.current_settings() == app.settings
        app.change_language('vi')
        app.close()
    assert not errors, errors
    print('PASS: Studio English on startup; all seven pages at both sizes; live English/Vietnamese switching; '
          'status, tables, logs and dialogs; saved preference on reopen; connection/session/stream, map names '
          'and disabled controls unchanged; no robot requests')
    print('Captures:', output)


if __name__ == '__main__':
    main()
