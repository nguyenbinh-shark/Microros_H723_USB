"""One-file build, using the same OpenOCD discovery as the source app.

Windows packs ST's OpenOCD into the EXE. Linux uses the system OpenOCD (apt install openocd);
build it on Ubuntu with packaging/linux/build.sh.
"""
from pathlib import Path
import runpy
import sys

WINDOWS = sys.platform == 'win32'
root = Path(SPECPATH)
entry = root / 'dap_flasher.pyw'
assets = root / 'assets'
data = [(str(assets / name), 'assets') for name in ('dap-flasher.png', 'dap-flasher-256.png', 'dap-flasher.ico')]
version = None
if WINDOWS:
    from PyInstaller.utils.win32.versioninfo import (
        FixedFileInfo, StringFileInfo, StringStruct, StringTable, VarFileInfo, VarStruct, VSVersionInfo,
    )
    app = runpy.run_path(str(entry))
    openocd, scripts = app['find_openocd']()
    if not scripts or not (Path(scripts) / 'target' / 'stm32h7x.cfg').is_file():
        raise SystemExit('Set OPENOCD_SCRIPTS to the OpenOCD scripts directory containing target/stm32h7x.cfg.')
    data += [(str(Path(openocd).parent), 'openocd/bin'), (str(Path(scripts)), 'openocd/scripts')]
    version = VSVersionInfo(
        ffi=FixedFileInfo(filevers=(0, 1, 0, 0), prodvers=(0, 1, 0, 0),
                          mask=0x3f, flags=0, OS=0x40004, fileType=1, subtype=0, date=(0, 0)),
        kids=[StringFileInfo([StringTable('040904B0', [
            StringStruct('CompanyName', 'Trần Nguyên Bình'),
            StringStruct('FileDescription', 'DAPLink Flasher - STM32H723 firmware tool'),
            StringStruct('FileVersion', '0.1.0'),
            StringStruct('InternalName', 'DAPLinkFlasher'),
            StringStruct('OriginalFilename', 'DAPLinkFlasher.exe'),
            StringStruct('ProductName', 'DAPLink Flasher'),
            StringStruct('ProductVersion', '0.1.0'),
        ])]), VarFileInfo([VarStruct('Translation', [1033, 1200])])],
    )
a = Analysis([str(entry)], pathex=[str(root), str(root.parent / 'workstation')], binaries=[], datas=data,
             hiddenimports=['hid'], hookspath=[], hooksconfig={}, runtime_hooks=[], excludes=[], noarchive=False)
pyz = PYZ(a.pure)
exe = EXE(pyz, a.scripts, a.binaries, a.datas, [], name='DAPLinkFlasher', console=False,
          debug=False, bootloader_ignore_signals=False, strip=False, upx=False,
          icon=str(assets / 'dap-flasher.ico') if WINDOWS else None, version=version)
