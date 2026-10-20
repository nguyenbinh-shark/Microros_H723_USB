# Build on Windows: python -m PyInstaller workstation/RPBotStudio.spec
# Build on Ubuntu:  packaging/linux/build.sh
from pathlib import Path
import sys

WINDOWS = sys.platform == 'win32'
root = Path(SPECPATH)
# Explicit allowlist: no PC settings, logs, SSH material, or firmware artifacts.
runtime_files = [
    'map_library.py', 'robot_tasks.py', 'remote_ops.py', 'lite_stream.py',
    'ros_session.py', 'ros_session.sh', 'mapping.rviz', 'navigation.rviz',
]
datas = [(str(root / name), '.') for name in runtime_files]
datas += [(str(root / 'assets' / name), 'assets')
          for name in ('rpbot-studio.png', 'rpbot-studio-256.png', 'rpbot-studio.ico')]

version = None
if WINDOWS:
    from PyInstaller.utils.win32.versioninfo import (
        FixedFileInfo, StringFileInfo, StringStruct, StringTable,
        VarFileInfo, VarStruct, VSVersionInfo,
    )
    version = VSVersionInfo(
        ffi=FixedFileInfo(filevers=(0, 1, 0, 0), prodvers=(0, 1, 0, 0),
                          mask=0x3f, flags=0, OS=0x40004, fileType=1, subtype=0, date=(0, 0)),
        kids=[StringFileInfo([StringTable('040904B0', [
            StringStruct('CompanyName', 'Trần Nguyên Bình'),
            StringStruct('FileDescription', 'RPBot Studio - Mapping, Navigation and diagnostics'),
            StringStruct('FileVersion', '0.1.0'),
            StringStruct('InternalName', 'RPBotStudio'),
            StringStruct('OriginalFilename', 'RPBotStudio.exe'),
            StringStruct('ProductName', 'RPBot Studio'),
            StringStruct('ProductVersion', '0.1.0'),
        ])]), VarFileInfo([VarStruct('Translation', [1033, 1200])])],
    )

a = Analysis(
    [str(root / 'launch_rpbot.pyw')], pathex=[str(root)],
    binaries=[], datas=datas, hiddenimports=[], hookspath=[],
    hooksconfig={}, runtime_hooks=[], excludes=[],
    noarchive=False,
)
pyz = PYZ(a.pure)
exe = EXE(
    pyz, a.scripts, a.binaries, a.datas, [],
    name='RPBotStudio', console=False, debug=False,
    bootloader_ignore_signals=False, strip=False, upx=False,
    icon=str(root / 'assets' / 'rpbot-studio.ico') if WINDOWS else None, version=version,
)
