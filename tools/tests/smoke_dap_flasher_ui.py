"""Native UI review using a fake flasher; never starts OpenOCD or touches USB."""
import argparse
from dataclasses import asdict
import importlib.machinery
import importlib.util
import json
from pathlib import Path
import runpy
import sys
import tempfile
import threading
import time
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'workstation' / 'tests'))
from preview_studio import capture_window, check_vietnamese_fonts


def load_flasher():
    loader = importlib.machinery.SourceFileLoader('dap_flasher', str(ROOT / 'tools' / 'dap_flasher.pyw'))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec)
    sys.modules[loader.name] = module
    loader.exec_module(module)
    return module


class FakeFlasher:
    instances = []

    def __init__(self, emit, ask):
        self.emit, self.ask = emit, ask
        self.stop = threading.Event()
        self.ocd = None
        self.chip = 'STM32H723 · DỮ LIỆU GIẢ'
        self.calls = []
        self.instances.append(self)

    def run(self, job, settings, *args):
        self.calls.append((job, asdict(settings), args))
        assert job == 'flash', 'Only the deliberate fake flash job is allowed'
        self.stop.wait(10)
        return False

    def ping(self):
        return 'up', 8.0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, default=ROOT / 'tools' / 'build' / 'gui-preview')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    module = load_flasher()
    branding = runpy.run_path(str(ROOT / 'workstation' / 'branding.py'))
    for key, field in (('name', 'AUTHOR'), ('bio', 'BIO'), ('website', 'WEBSITE'), ('github', 'GITHUB'), ('email', 'EMAIL')):
        assert module.AUTHOR_INFO[key] == branding[field], ('Author identity differs from RPBot Studio', key)
    errors, captures = [], []
    for mode in ('light', 'dark'):
        with tempfile.TemporaryDirectory(prefix='dap-ui-fixture-') as folder:
            fixture = Path(folder)
            image = fixture / 'firmware-kiểm-thử.bin'
            image.write_bytes(bytes(range(256)) * 3072)
            settings = module.Settings(image=str(image), theme=mode, safety=False,
                                       probe_serial='fixture-1', kill_other_openocd=False, show_openocd_log=True)
            with patch.object(module, 'SETTINGS_FILE', fixture / 'settings.json'), \
                 patch.object(module, 'Flasher', FakeFlasher), \
                 patch.object(module, 'list_probes', return_value=[('CMSIS-DAP · TEST-01', 'fixture-1')]), \
                 patch.object(module, 'find_openocd', return_value=(str(fixture / 'openocd.exe'), None)), \
                 patch('winsound.MessageBeep'), \
                 patch('webbrowser.open', return_value=True) as open_link:
                settings.save()
                state = dict(ok=False)

                def ready(root, app):
                    def fail(kind, error, trace):
                        import traceback
                        traceback.print_exception(kind, error, trace)
                        errors.append(str(error))
                        app.flasher.stop.set()
                        root.destroy()
                    root.report_callback_exception = fail

                    def capture(name, size='1280x860'):
                        root.title('KIỂM THỬ · DỮ LIỆU GIẢ · DAPLink Flasher')
                        root.geometry(size)
                        root.update()
                        assert app.btn_flash.winfo_ismapped()
                        assert app.btn_flash.winfo_width() >= app.btn_flash.winfo_reqwidth() - 2
                        assert app.btn_flash.winfo_y() + app.btn_flash.winfo_height() <= app.btn_flash.master.winfo_height()
                        assert app.txt.winfo_height() >= 90, ('Log too small', mode, size, app.txt.winfo_height())
                        for button in (app.btn_connect, app.btn_reset, app.btn_refresh, app.btn_browse):
                            assert button.winfo_width() >= button.winfo_reqwidth() - 2, ('Clipped button', button.cget('text'))
                        path = args.output / f'{mode}-{name}-{size}.png'
                        assert capture_window(root, path)
                        captures.append(path.name)

                    def begin():
                        import tkinter as tk
                        assert app.brand_logo is not None
                        assert app.cards['link'].level == 'muted'
                        assert app.cards['image'].level == 'ok'
                        assert app.cards['safety'].level == 'warn'
                        assert app.var_safety.get() is False, 'Do not change the saved safety choice'
                        assert app._settings().probe_serial == 'fixture-1'
                        font_probe = tk.Label(root, text='Nhật ký thao tác · dữ liệu kiểm thử', font=app.txt.cget('font'))
                        check_vietnamese_fonts(root)
                        font_probe.destroy()
                        capture('idle')
                        capture('idle', '1060x720')
                        app.language_picker.variable.set('English')
                        app.language_picker.combobox.event_generate('<<ComboboxSelected>>')
                        root.update()
                        assert app.i18n.code == 'en' and app.btn_flash.cget('text') == 'Flash firmware'
                        assert app.btn_about.cget('text') == 'About the author'
                        assert json.loads((fixture / 'settings.json').read_text(encoding='utf-8'))['language'] == 'en'
                        assert app._settings().probe_serial == 'fixture-1'
                        capture('english-idle', '1060x720')
                        app.btn_about._click()
                        about = app.about_window
                        root.update()
                        assert about is not None and about.winfo_ismapped()
                        assert about.title() == 'About the author · DAPLink Flasher'
                        assert app.btn_close_about.cget('text') == 'Close'
                        assert root.grab_current() is None, 'Author window must leave flash/stop accessible'
                        app.btn_about._click()
                        assert app.about_window is about, 'Reuse the open author window'
                        for key, link in app.author_links.items():
                            assert link.cget('text') == module.AUTHOR_INFO[key]
                            assert link.winfo_width() >= link.winfo_reqwidth() - 2, ('Clipped author contact', key)
                            link._click()
                        assert [call.args[0] for call in open_link.call_args_list] == [
                            branding['WEBSITE'], branding['GITHUB'], 'mailto:' + branding['EMAIL']]
                        path = args.output / f'{mode}-author.png'
                        check_vietnamese_fonts(about)
                        assert capture_window(about, path)
                        captures.append(path.name)
                        app.btn_close_about._click()
                        assert app.about_window is None and root.winfo_exists()
                        assert app.flasher.calls == [], 'Author information must not start a flasher job'
                        app.btn_about._click()
                        app.about_window.event_generate('<Escape>')
                        root.update()
                        assert app.about_window is None
                        app._change_language('vi')
                        assert app.btn_flash.cget('text') == 'Nạp firmware'
                        app.btn_flash._click()
                        root.after(80, writing)

                    def writing():
                        assert app._busy() and app.job == 'flash'
                        assert app.btn_flash.enabled and app.btn_flash.cget('text') == 'Dừng nạp'
                        assert not app.btn_browse.enabled and not app.btn_reset.enabled
                        assert all(str(widget.cget('state')) == 'disabled' for widget, _ in app.edit_controls)
                        assert app.flasher.calls[0][1]['safety'] is False
                        assert app.flasher.calls[0][1]['kill_other_openocd'] is False
                        assert app.flasher.calls[0][2] == (str(image),)
                        app._link_view('up', 'DỮ LIỆU GIẢ · hid 1000 kHz', chip='STM32H723', idcode=0x483)
                        app._on_plan([(i, module.SECTOR_SIZE) for i in range(6)], 6 * module.SECTOR_SIZE)
                        app._on_sector(0, 'done', module.SECTOR_SIZE, module.SECTOR_SIZE, module.FLASH_BASE)
                        app._on_sector(1, 'match', module.SECTOR_SIZE, module.SECTOR_SIZE, module.FLASH_BASE + module.SECTOR_SIZE)
                        app._on_sector(2, 'writing', 32768, module.SECTOR_SIZE, module.FLASH_BASE + 2 * module.SECTOR_SIZE)
                        app._on_progress(294912, 786432, 8192, 60, 36)
                        app._append('DỮ LIỆU GIẢ: đang ghi sector 2; không có thao tác phần cứng.', 'info')
                        capture('writing')
                        capture('writing', '1060x720')
                        app._link_view('down', 'DỮ LIỆU GIẢ · đường truyền gián đoạn')
                        assert app.cards['link'].level == 'warn'
                        assert app.btn_flash.enabled
                        capture('lost-link')
                        app.btn_about._click()
                        assert app.btn_flash.enabled and root.grab_current() is None
                        app.btn_close_about._click()
                        worker = app.worker
                        app._change_language('en')
                        assert app.worker is worker and app.job == 'flash' and not app.flasher.stop.is_set()
                        assert app.btn_flash.cget('text') == 'Stop flashing' and app.btn_flash.enabled
                        assert not app.btn_browse.enabled
                        app._on_sector(2, 'writing', 32768, module.SECTOR_SIZE, module.FLASH_BASE + 2 * module.SECTOR_SIZE)
                        assert app.lbl_now.cget('text').startswith('Sector 2: writing')
                        app._append('Sector 1: đã đúng sẵn, bỏ qua', 'ok')
                        assert 'Sector 1: already matches, skipped' in app.txt.get('1.0', 'end')
                        capture('english-writing', '1060x720')
                        app._change_language('vi')
                        app.btn_flash._click()
                        assert app.flasher.stop.is_set()
                        root.after(100, finished)

                    def finished():
                        if app._busy() or app.worker is not None:
                            root.after(50, finished)
                            return
                        assert len(app.flasher.calls) == 1
                        assert app.result is False
                        assert app.btn_flash.enabled
                        assert app.btn_browse.enabled
                        app._on_finished(True, 'flash')
                        assert app.lbl_pct.cget('text') == '100%'
                        capture('complete')
                        app.var_image.set(str(fixture / 'missing.bin'))
                        assert not app.btn_flash.enabled
                        app.btn_flash._click()
                        assert len(app.flasher.calls) == 1
                        assert app.cards['image'].level == 'err'
                        app.var_safety.set(True)
                        assert app.cards['safety'].level == 'info', 'Configured check is not proof that motors are off'
                        capture('invalid-file')
                        state['ok'] = True
                        app._close()

                    root.after(160, begin)
                    root.after(15000, lambda: (app.flasher.stop.set(), root.destroy()))
                module.run_gui(settings, on_ready=ready)
                if errors or not state['ok']:
                    raise SystemExit(f'Flasher UI failed: {mode}, {state}, {errors}')
    print('PASS: light/dark, 1280x860 and 1060x720, native Vietnamese glyphs, status/log/flash controls, '
          'author identity/links, reusable modeless author window, fake flash/stop, invalid-file guard, '
          'saved settings preserved; no OpenOCD or USB calls')
    print('Captures:', ', '.join(captures))
    print('Output:', args.output)


if __name__ == '__main__':
    main()
