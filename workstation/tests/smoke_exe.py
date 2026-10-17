"""Run the actual one-file EXE away from source, with a fresh offline profile.

--wsl additionally checks that WSL can read the extracted helper files. It
never imports ROS, contacts SSH, starts a robot session or sends commands.
"""
import argparse
import ctypes
from ctypes import wintypes
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rpbot_console import HERE, STATE, wsl_path
from preview_studio import capture_window


FILES = ['map_library.py', 'robot_tasks.py', 'remote_ops.py', 'ros_session.py',
         'ros_session.sh', 'mapping.rviz', 'navigation.rviz',
         'assets/shark-logo.png', 'assets/shark.ico']


def find_window(executable):
    """Find only a window owned by the exact isolated executable copy."""
    user, kernel = ctypes.windll.user32, ctypes.windll.kernel32
    callback_type = ctypes.WINFUNCTYPE(wintypes.BOOL, wintypes.HWND, wintypes.LPARAM)
    user.EnumWindows.argtypes = [callback_type, wintypes.LPARAM]
    user.GetWindowThreadProcessId.argtypes = [wintypes.HWND, ctypes.POINTER(wintypes.DWORD)]
    user.GetWindowTextW.argtypes = [wintypes.HWND, wintypes.LPWSTR, ctypes.c_int]
    kernel.OpenProcess.argtypes, kernel.OpenProcess.restype = [wintypes.DWORD, wintypes.BOOL, wintypes.DWORD], wintypes.HANDLE
    kernel.QueryFullProcessImageNameW.argtypes = [wintypes.HANDLE, wintypes.DWORD, wintypes.LPWSTR, ctypes.POINTER(wintypes.DWORD)]
    kernel.CloseHandle.argtypes = [wintypes.HANDLE]
    found = []

    @callback_type
    def visit(hwnd, _):
        title = ctypes.create_unicode_buffer(512)
        user.GetWindowTextW(hwnd, title, len(title))
        if 'RPBot Studio' not in title.value:
            return True
        pid = wintypes.DWORD()
        user.GetWindowThreadProcessId(hwnd, ctypes.byref(pid))
        handle = kernel.OpenProcess(0x1000, False, pid.value)
        if handle:
            try:
                name, length = ctypes.create_unicode_buffer(32768), wintypes.DWORD(32768)
                if kernel.QueryFullProcessImageNameW(handle, 0, name, ctypes.byref(length)):
                    if Path(name.value).resolve() == executable.resolve():
                        found.append((hwnd, title.value))
            finally:
                kernel.CloseHandle(handle)
        return True
    user.EnumWindows(visit, 0)
    return found[0] if found else None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--exe', type=Path, default=HERE / 'dist' / 'RPBotStudio.exe')
    parser.add_argument('--wsl', action='store_true')
    parser.add_argument('--distro', default='Ubuntu-24.04')
    parser.add_argument('--wsl-user', default='shark')
    args = parser.parse_args()
    if os.name != 'nt':
        raise SystemExit('This check requires Windows.')
    # Match the app's DPI awareness, otherwise PrintWindow captures are cropped
    # to virtualized coordinates on scaled Windows displays.
    ctypes.windll.shcore.SetProcessDpiAwareness(1)

    from PyInstaller.archive.readers import CArchiveReader
    import pefile
    archive = CArchiveReader(str(args.exe))
    members = {name.replace('\\', '/'): name for name in archive.toc}
    for name in FILES:
        assert archive.extract(members[name]) == (HERE / name).read_bytes(), ('Wrong bundled resource', name)
    assert not any(Path(name).name.lower() in ('settings.json', 'known_hosts', 'id_rsa', 'id_ed25519')
                   for name in members)
    modules = archive.open_embedded_archive('PYZ.pyz').toc
    assert all(name in modules for name in ('rpbot_console', 'console_layout', 'tkinter',
                                           'network_probe', 'robot_status', 'status_dashboard'))
    pe = pefile.PE(str(args.exe), fast_load=True)
    assert pe.OPTIONAL_HEADER.Subsystem == 2, 'EXE must use the windowed subsystem'
    assert pe.FILE_HEADER.Machine == 0x8664, 'Expected Windows x64'
    pe.close()

    output = STATE / 'studio-preview'
    output.mkdir(parents=True, exist_ok=True)
    report = dict(exe=str(args.exe.resolve()), bytes=args.exe.stat().st_size,
                  sha256=hashlib.sha256(args.exe.read_bytes()).hexdigest(), resources=FILES,
                  offline=True, wsl_checked=False)
    with tempfile.TemporaryDirectory(prefix='rpbot-exe-check-') as folder:
        scratch = Path(folder).resolve()
        assert scratch.parent == Path(tempfile.gettempdir()).resolve()
        delivery, unpack, profile = (scratch / name for name in ('Ứng dụng rời', 'unpack', 'profile'))
        for directory in (delivery, unpack, profile):
            directory.mkdir()
        copied = delivery / 'RPBotStudio.exe'
        shutil.copy2(args.exe, copied)
        env = dict(os.environ, LOCALAPPDATA=str(profile), TEMP=str(unpack), TMP=str(unpack),
                   PATH=os.path.join(os.environ['SystemRoot'], 'System32'),
                   PYINSTALLER_RESET_ENVIRONMENT='1')
        for key in ('PYTHONPATH', 'PYTHONHOME'):
            env.pop(key, None)
        process = subprocess.Popen([str(copied)], cwd=delivery, env=env, stdin=subprocess.DEVNULL,
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                                   creationflags=subprocess.CREATE_NO_WINDOW)
        window = None
        try:
            deadline = time.monotonic() + 25
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    raise RuntimeError(f'Packaged app exited early: {process.returncode}')
                window = find_window(copied)
                if window:
                    break
                time.sleep(0.15)
            assert window, 'The standalone GUI did not open'
            hwnd, title = window
            assert 'Trần Nguyên Bình' in title
            time.sleep(0.8)
            assert capture_window(SimpleNamespace(winfo_id=lambda: hwnd), output / 'exe-offline.png')
            bundles = list(unpack.glob('_MEI*'))
            assert len(bundles) == 1, bundles
            bundle = bundles[0]
            for name in FILES:
                assert (bundle / name).read_bytes() == (HERE / name).read_bytes()
            assert not (profile / 'RPBotConsole' / 'settings.json').exists()
            assert not (profile / 'RPBotConsole' / 'telemetry.log').exists()
            assert list(delivery.iterdir()) == [copied], 'Only the EXE may be delivered'
            if args.wsl:
                check = (
                    "import ast,hashlib,json,pathlib,subprocess,sys; p=pathlib.Path(sys.argv[1]); "
                    "names=json.loads(sys.argv[2]); "
                    "[ast.parse((p/n).read_text(encoding='utf-8')) for n in names if n.endswith('.py')]; "
                    "assert b'\\r\\n' not in (p/'ros_session.sh').read_bytes(); "
                    "subprocess.run(['bash','-n',str(p/'ros_session.sh')],check=True); "
                    "print(json.dumps({n:hashlib.sha256((p/n).read_bytes()).hexdigest() for n in names}))"
                )
                result = subprocess.run(['wsl.exe', '-d', args.distro, '-u', args.wsl_user, '--',
                                         'python3', '-c', check, wsl_path(bundle), json.dumps(FILES)],
                                        stdin=subprocess.DEVNULL, capture_output=True, text=True,
                                        encoding='utf-8', timeout=25, creationflags=subprocess.CREATE_NO_WINDOW)
                assert result.returncode == 0, result.stderr + result.stdout
                hashes = json.loads(result.stdout)
                assert hashes == {name: hashlib.sha256((HERE / name).read_bytes()).hexdigest() for name in FILES}
                report['wsl_checked'] = True
            user = ctypes.windll.user32
            user.PostMessageW.argtypes = [wintypes.HWND, wintypes.UINT, wintypes.WPARAM, wintypes.LPARAM]
            user.PostMessageW(hwnd, 0x10, 0, 0)
            assert process.wait(timeout=10) == 0, 'App did not close cleanly'
        finally:
            if process.poll() is None:
                if window:
                    ctypes.windll.user32.PostMessageW(window[0], 0x10, 0, 0)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    subprocess.run(['taskkill.exe', '/PID', str(process.pid), '/T', '/F'],
                                   capture_output=True, creationflags=subprocess.CREATE_NO_WINDOW, timeout=10)
                    process.wait(timeout=5)
    (STATE / 'exe-validation.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print('PASS: one-file x64 GUI, isolated folder with Vietnamese/spaces, no Python on PATH, '
          'embedded resources match source, fresh offline settings, clean exit; WSL:', report['wsl_checked'])
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
