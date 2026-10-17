"""Check the real portable GUI and OpenOCD --version; never opens an SWD session."""
import argparse
import ctypes
import json
from pathlib import Path
import os
import shutil
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'workstation' / 'tests'))
from preview_studio import capture_window
from smoke_exe import find_window


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--exe', type=Path, default=ROOT / 'tools' / 'build' / 'release' / 'DAPLinkFlasher.exe')
    parser.add_argument('--baseline', type=Path, default=ROOT / 'tools' / 'DAPLinkFlasher.exe')
    parser.add_argument('--language', choices=('vi', 'en'), default='vi')
    args = parser.parse_args()
    from PyInstaller.archive.readers import CArchiveReader
    ctypes.windll.shcore.SetProcessDpiAwareness(1)
    archive = CArchiveReader(str(args.exe))
    members = {name.replace('\\', '/'): name for name in archive.toc}
    modules = archive.open_embedded_archive('PYZ.pyz').toc
    assert all(name in modules for name in ('ui_language', 'ui_translations'))
    old = CArchiveReader(str(args.baseline))
    old_members = {name.replace('\\', '/'): name for name in old.toc}
    checked = 0
    for name in old_members:
        if name.startswith(('openocd/bin/', 'openocd/scripts/')):
            assert archive.extract(members[name]) == old.extract(old_members[name]), ('OpenOCD changed', name)
            checked += 1
    assert checked >= 39, 'Expected the complete working OpenOCD payload'
    assert any(name.startswith('hid.') and name.endswith('.pyd') for name in members), 'USB probe enumeration must remain available'
    for name in ('dap-flasher.png', 'dap-flasher.ico'):
        assert archive.extract(members['assets/' + name]) == (ROOT / 'tools' / 'assets' / name).read_bytes()
    assert not any(Path(name).name.lower() in ('settings.json', 'id_ed25519', 'id_rsa') for name in members)

    with tempfile.TemporaryDirectory(prefix='dap-exe-review-') as folder:
        scratch = Path(folder).resolve()
        assert scratch.parent == Path(tempfile.gettempdir()).resolve()
        delivery, unpack, profile = (scratch / name for name in ('Ứng dụng nạp', 'unpack', 'profile'))
        for directory in (delivery, unpack, profile):
            directory.mkdir()
        config = profile / 'dap_flasher'
        config.mkdir()
        (config / 'settings.json').write_text(json.dumps(dict(image='', theme='light', safety=True,
                                                            kill_other_openocd=False, language=args.language)), encoding='utf-8')
        copied = delivery / 'DAPLinkFlasher.exe'
        shutil.copy2(args.exe, copied)
        env = dict(os.environ, APPDATA=str(profile), LOCALAPPDATA=str(profile), TEMP=str(unpack), TMP=str(unpack),
                   PATH=os.path.join(os.environ['SystemRoot'], 'System32'), PYINSTALLER_RESET_ENVIRONMENT='1')
        for key in ('OPENOCD', 'OPENOCD_SCRIPTS', 'PYTHONPATH', 'PYTHONHOME'):
            env.pop(key, None)
        process = subprocess.Popen([str(copied)], cwd=delivery, env=env, stdin=subprocess.DEVNULL,
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                   creationflags=subprocess.CREATE_NO_WINDOW)
        window = None
        try:
            deadline = time.monotonic() + 25
            while time.monotonic() < deadline:
                assert process.poll() is None, ('App exited early', process.returncode)
                window = find_window(copied, title_contains='DAPLink Flasher')
                if window:
                    break
                time.sleep(0.15)
            assert window, 'EXE did not show its window'
            hwnd, title = window
            assert 'Trần Nguyên Bình' in title
            time.sleep(0.6)
            output = ROOT / 'tools' / 'build' / 'gui-preview'
            output.mkdir(parents=True, exist_ok=True)
            assert capture_window(SimpleNamespace(winfo_id=lambda: hwnd), output / f'exe-light-{args.language}.png')
            bundles = list(unpack.glob('_MEI*'))
            assert len(bundles) == 1
            bundle = bundles[0]
            result = subprocess.run([str(bundle / 'openocd' / 'bin' / 'openocd.exe'), '--version'], env=env,
                                    stdin=subprocess.DEVNULL, capture_output=True, text=True, errors='replace',
                                    timeout=10, creationflags=subprocess.CREATE_NO_WINDOW)
            assert result.returncode == 0, result.stdout + result.stderr
            assert 'Open On-Chip Debugger' in result.stdout + result.stderr
            assert list(delivery.iterdir()) == [copied]
            user = ctypes.windll.user32
            from ctypes import wintypes
            user.PostMessageW.argtypes = [wintypes.HWND, wintypes.UINT, wintypes.WPARAM, wintypes.LPARAM]
            user.PostMessageW(hwnd, 0x10, 0, 0)
            assert process.wait(timeout=10) == 0
            assert json.loads((config / 'settings.json').read_text(encoding='utf-8'))['language'] == args.language
        finally:
            if process.poll() is None:
                if window:
                    ctypes.windll.user32.PostMessageW(window[0], 0x10, 0, 0)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    subprocess.run(['taskkill.exe', '/PID', str(process.pid), '/T', '/F'], capture_output=True,
                                   timeout=10, creationflags=subprocess.CREATE_NO_WINDOW)
                    process.wait(timeout=5)
    print(f'PASS: real EXE starts/closes outside source; Python/OpenOCD absent from PATH; {checked} '
          'original OpenOCD files unchanged; bundled USB enumeration and branding present; '
          'OpenOCD --version runs without a target or SWD commands')
    print('Language:', args.language)


if __name__ == '__main__':
    main()
