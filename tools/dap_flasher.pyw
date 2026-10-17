#!/usr/bin/env python3
"""DAPLink Flasher: one-file STM32H723 flasher for flaky (wireless) CMSIS-DAP links.

Double-click to open the window, then: 1. connect the probe, 2. pick the firmware,
3. set the link parameters, and press Flash.

Why not plain `openocd -c program`: it starts over every time the link drops, and every failed
attempt leaves the flash half erased. This tool keeps one OpenOCD session open, drives it over
the Tcl RPC port and programs one 128 KB sector at a time in 8 KB chunks. Every sector is
checked with an on-target CRC (verify_image_checksum), so sectors that already hold the image
are skipped, and a dropped link only costs the chunk in flight: the next session looks at what
that chunk left behind (written, untouched or half written) and carries on from there.

Needs Python 3.10+ with Tk and OpenOCD (on PATH, through %OPENOCD%, or the copy bundled with
STM32CubeIDE). hidapi (pip install hidapi) is optional; it only fills the probe list.

The same engine on the console:
    py dap_flasher.pyw --cli [FILE]      flash
    py dap_flasher.pyw --link-test       link quality only, does not halt the core
    py dap_flasher.pyw --reset           reset and run the STM32

Standalone DAPLinkFlasher.exe (no Python, no STM32CubeIDE needed on the target PC): pack this
file with ST's OpenOCD from STM32CubeIDE (its tools\\bin folder and st_scripts) using PyInstaller:
    py -m PyInstaller --onefile --windowed --name DAPLinkFlasher --icon dap_flasher.ico
       --add-data "<openocd tools\\bin>;openocd/bin" --add-data "<st_scripts>;openocd/scripts"
       --add-data "dap_flasher.ico;." tools/dap_flasher.pyw

Safety (RPBot): with the motor check on, a running core is only halted after the check
command reports /motor_status with bit 0x10 (MOTORS_ENABLED) clear. A halted MCU can leave
enabled RobStride motors on their last speed setpoint.
"""

from __future__ import annotations

import argparse
import collections
import glob
import json
import os
import queue
import re
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

# Inside DAPLinkFlasher.exe, PyInstaller unpacks the packed OpenOCD and icon here.
BUNDLE_DIR = Path(getattr(sys, "_MEIPASS", Path(__file__).resolve().parent))
_REPO_IMAGE = Path(__file__).resolve().parent.parent / "build" / "microros_H7.elf"
DEFAULT_IMAGE = str(_REPO_IMAGE) if _REPO_IMAGE.is_file() else ""
SETTINGS_FILE = Path(os.environ.get("APPDATA", str(Path.home()))) / "dap_flasher" / "settings.json"

# STM32H723 bank 1: 1 MB in eight 128 KB sectors, programmed in 256-bit flash words.
FLASH_BASE = 0x08000000
SECTOR_SIZE = 0x20000
SECTOR_COUNT = 8
FLASH_END = FLASH_BASE + SECTOR_SIZE * SECTOR_COUNT
FLASH_WORD = 32
CHUNK_SIZE = 8 * 1024

DBGMCU_IDCODE = 0x5C001000
DEV_ID_H72X = 0x483
DHCSR = 0xE000EDF0
DHCSR_S_HALT = 1 << 17
MOTORS_ENABLED_BIT = 0x10

# OpenOCD target setup, written to a temp file at run time so this script stays one file.
# The adapter (cmsis-dap, backend, serial) is chosen on the command line before it.
OPENOCD_TARGET_CFG = """\
transport select swd
# Required by ST's stm32h7x.cfg (STM32CubeIDE OpenOCD), ignored by upstream OpenOCD.
set AP_NUM 0
set GDB_PORT 3333
# Reset with SYSRESETREQ (whole chip) instead of VECTRESET (core only).
set CORE_RESET 0
source [find target/stm32h7x.cfg]
# Reset over SWD only, so NRST does not need to be wired
# (the wireless DAPLink RST pin doubles as RTS when esp_down=enable).
reset_config none
"""

# Backend/clock pairs tried in turn when a session cannot be opened (first = chosen one).
DEFAULT_VARIANTS = ["hid 1000", "hid 400", "usb_bulk 1000"]
# RPBot default: read /motor_status on the Pi through WSL. It must print a decimal number.
DEFAULT_SAFETY_CMD = (
    'wsl -d Ubuntu-24.04 -u shark -- ssh -o BatchMode=yes -o ConnectTimeout=5 sharkagv@192.168.0.120 '
    '"source /opt/ros/jazzy/setup.bash; ROS_DOMAIN_ID=0 timeout -k 1 6 ros2 topic echo /motor_status '
    'std_msgs/msg/UInt8 --once --field data --no-daemon --spin-time 2 2>/dev/null | head -1"'
)

NO_WINDOW = getattr(subprocess, "CREATE_NO_WINDOW", 0)


# --------------------------------------------------------------------------- settings

@dataclass
class Settings:
    image: str = DEFAULT_IMAGE
    probe_serial: str = ""
    backend: str = "hid"
    speed_khz: int = 1000
    rotate: bool = True
    variants: list = field(default_factory=lambda: list(DEFAULT_VARIANTS))
    max_minutes: int = 30
    force: bool = False
    safety: bool = True
    safety_cmd: str = DEFAULT_SAFETY_CMD
    kill_other_openocd: bool = True
    show_openocd_log: bool = False
    theme: str = "auto"

    @classmethod
    def load(cls) -> "Settings":
        s = cls()
        try:
            data = json.loads(SETTINGS_FILE.read_text(encoding="utf-8"))
            for k, v in data.items():
                if hasattr(s, k):
                    setattr(s, k, v)
        except (OSError, ValueError):
            pass
        return s

    def save(self) -> None:
        try:
            SETTINGS_FILE.parent.mkdir(parents=True, exist_ok=True)
            SETTINGS_FILE.write_text(json.dumps(asdict(self), indent=2, ensure_ascii=False), encoding="utf-8")
        except OSError:
            pass


# --------------------------------------------------------------------------- image

def _natural_key(text: str):
    return [int(t) if t.isdigit() else t for t in re.split(r"(\d+)", text)]


def find_openocd() -> tuple[str, str | None]:
    """%OPENOCD%, then the copy packed into DAPLinkFlasher.exe, then openocd on PATH,
    then the copy bundled with STM32CubeIDE."""
    exe = os.environ.get("OPENOCD")
    scripts = os.environ.get("OPENOCD_SCRIPTS")
    packed = BUNDLE_DIR / "openocd"
    if not exe and (packed / "bin" / "openocd.exe").is_file():
        return str(packed / "bin" / "openocd.exe"), scripts or str(packed / "scripts")
    exe = exe or shutil.which("openocd")
    if not exe:
        for plug in sorted(glob.glob(r"C:\ST\STM32CubeIDE_*\STM32CubeIDE\plugins"), key=_natural_key, reverse=True):
            exes = sorted(glob.glob(os.path.join(
                plug, "com.st.stm32cube.ide.mcu.externaltools.openocd.*", "tools", "bin", "openocd.exe")),
                key=_natural_key)
            if exes:
                exe = exes[-1]
                if not scripts:
                    found = sorted(glob.glob(os.path.join(
                        plug, "com.st.stm32cube.ide.mcu.debug.openocd_*", "resources", "openocd", "st_scripts")),
                        key=_natural_key)
                    scripts = found[-1] if found else None
                break
    if not exe or not os.path.isfile(exe):
        raise FileNotFoundError("Không tìm thấy OpenOCD. Cài STM32CubeIDE, hoặc đặt biến môi trường "
                                "OPENOCD=đường\\dẫn\\openocd.exe")
    return exe, scripts


def _target_cfg() -> str:
    path = Path(tempfile.gettempdir()) / "dap_flasher_stm32h723.cfg"
    if not path.is_file() or path.read_text() != OPENOCD_TARGET_CFG:
        path.write_text(OPENOCD_TARGET_CFG)
    return str(path)


def list_probes() -> list[tuple[str, str]]:
    """(label, serial) of the CMSIS-DAP probes on USB; empty without hidapi."""
    try:
        import hid
    except ImportError:
        return []
    found: dict[str, str] = {}
    try:
        for d in hid.enumerate():
            name = (d.get("product_string") or "").strip()
            if "CMSIS-DAP" in name.upper():
                sn = (d.get("serial_number") or "").strip()
                found[sn] = f"{name} · {sn}" if sn else name
    except OSError:
        return []
    return [(label, sn) for sn, label in found.items()]


def _elf_segments(raw: bytes) -> list[tuple[int, bytes]]:
    if raw[4] != 1 or raw[5] != 1:
        raise ValueError("chỉ hỗ trợ ELF 32-bit little-endian")
    phoff, = struct.unpack_from("<I", raw, 28)
    phentsize, phnum = struct.unpack_from("<HH", raw, 42)
    segments = []
    for i in range(phnum):
        p_type, p_offset, _vaddr, p_paddr, p_filesz = struct.unpack_from("<5I", raw, phoff + i * phentsize)
        if p_type == 1 and p_filesz:            # PT_LOAD, placed at its load address
            segments.append((p_paddr, raw[p_offset:p_offset + p_filesz]))
    return segments


def _hex_segments(text: str) -> list[tuple[int, bytes]]:
    records, upper = [], 0
    for n, line in enumerate(text.splitlines(), 1):
        line = line.strip()
        if not line:
            continue
        if not line.startswith(":"):
            raise ValueError(f"dòng {n}: không phải bản ghi Intel HEX")
        rec = bytes.fromhex(line[1:])
        if sum(rec) & 0xFF:
            raise ValueError(f"dòng {n}: sai checksum")
        offset, rtype, data = (rec[1] << 8) | rec[2], rec[3], rec[4:4 + rec[0]]
        if rtype == 0:
            records.append((upper + offset, data))
        elif rtype == 1:
            break
        elif rtype == 2:
            upper = int.from_bytes(data, "big") << 4
        elif rtype == 4:
            upper = int.from_bytes(data, "big") << 16
    segments: list[tuple[int, bytearray]] = []
    for addr, data in sorted(records):
        if segments and segments[-1][0] + len(segments[-1][1]) == addr:
            segments[-1][1].extend(data)
        else:
            segments.append((addr, bytearray(data)))
    return [(a, bytes(d)) for a, d in segments]


def load_image(path: Path) -> tuple[list[tuple[int, bytes]], list[str]]:
    """Flash segments of an .elf/.hex/.bin (.bin goes to 0x08000000) plus notes on skipped parts."""
    raw = path.read_bytes()
    if raw[:4] == b"\x7fELF":
        segments = _elf_segments(raw)
    elif path.suffix.lower() in (".hex", ".ihex"):
        segments = _hex_segments(raw.decode("ascii"))
    elif path.suffix.lower() == ".bin":
        segments = [(FLASH_BASE, raw)]
    else:
        raise ValueError(f"không hỗ trợ loại file {path.suffix or path.name}")
    keep, notes = [], []
    for addr, data in segments:
        if FLASH_BASE <= addr and addr + len(data) <= FLASH_END:
            keep.append((addr, data))
        else:
            notes.append(f"bỏ qua đoạn {addr:#010x} (+{len(data)} B) nằm ngoài flash")
    if not keep:
        raise ValueError("file không có gì để ghi vào flash")
    return keep, notes


@dataclass
class Sector:
    index: int
    addr: int                 # first byte of the sector
    data_addr: int            # first byte the image writes in it (flash-word aligned)
    data: bytes               # bytes written from data_addr on (whole flash words)
    state: str = "pending"    # pending checking erasing writing verifying done match dirty
    erased: bool = False      # erased in this job; chunks below next_chunk written since
    known_dirty: bool = False  # does not hold the image: skip the first CRC look
    in_flight: bool = False   # a chunk write was running when the session broke
    next_chunk: int = 0
    written: int = 0
    path: str = ""            # whole data file, for the sector CRC

    def chunks(self) -> list[tuple[int, bytes]]:
        return [(self.data_addr + o, self.data[o:o + CHUNK_SIZE]) for o in range(0, len(self.data), CHUNK_SIZE)]


def plan_sectors(segments: list[tuple[int, bytes]]) -> list[Sector]:
    bufs: dict[int, list] = {}
    for addr, data in segments:
        pos = 0
        while pos < len(data):
            a = addr + pos
            idx = (a - FLASH_BASE) // SECTOR_SIZE
            sec_start = FLASH_BASE + idx * SECTOR_SIZE
            n = min(len(data) - pos, sec_start + SECTOR_SIZE - a)
            entry = bufs.setdefault(idx, [bytearray(b"\xff" * SECTOR_SIZE), SECTOR_SIZE, 0])
            off = a - sec_start
            entry[0][off:off + n] = data[pos:pos + n]
            entry[1], entry[2] = min(entry[1], off), max(entry[2], off + n)
            pos += n
    sectors = []
    for idx in sorted(bufs):
        buf, lo, hi = bufs[idx]
        lo = lo // FLASH_WORD * FLASH_WORD
        hi = -(-hi // FLASH_WORD) * FLASH_WORD
        addr = FLASH_BASE + idx * SECTOR_SIZE
        sectors.append(Sector(idx, addr, addr + lo, bytes(buf[lo:hi])))
    return sectors


def fmt_addr(a: int) -> str:
    return f"0x{a >> 16:04X} {a & 0xFFFF:04X}"


# --------------------------------------------------------------------------- OpenOCD

class LinkError(Exception):
    """The OpenOCD session is gone or no longer answers."""


class CommandError(Exception):
    """OpenOCD answered, but the command failed."""


class StopRequested(Exception):
    pass


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class OpenOcd:
    """One OpenOCD process with a Tcl RPC connection."""

    def __init__(self, exe: str, scripts: str | None, backend: str, speed_khz: int, serial: str, on_line):
        self.exe, self.scripts, self.backend, self.speed_khz, self.serial = exe, scripts, backend, speed_khz, serial
        self.on_line = on_line
        self.proc: subprocess.Popen | None = None
        self.sock: socket.socket | None = None
        self.lines: collections.deque = collections.deque(maxlen=200)

    def start(self, timeout: float = 25.0) -> None:
        port = _free_port()
        args = [self.exe]
        if self.scripts:
            args += ["-s", self.scripts]
        args += ["-c", "adapter driver cmsis-dap", "-c", f"cmsis-dap backend {self.backend}"]
        if self.serial:
            args += ["-c", f"adapter serial {self.serial}"]
        args += ["-f", _target_cfg(),
                 "-c", "gdb_port disabled", "-c", "telnet_port disabled", "-c", f"tcl_port {port}",
                 "-c", f"adapter speed {self.speed_khz}"]
        self.proc = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     stdin=subprocess.DEVNULL, text=True, errors="replace",
                                     creationflags=NO_WINDOW)
        threading.Thread(target=self._pump, daemon=True).start()
        # The Tcl port listens before `init`; the first command waits until init is done.
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.proc.poll() is not None:
                time.sleep(0.2)
                raise LinkError(self.last_error() or f"OpenOCD thoát với mã {self.proc.returncode}")
            try:
                self.sock = socket.create_connection(("127.0.0.1", port), timeout=1)
                return
            except OSError:
                time.sleep(0.2)
        raise LinkError("OpenOCD không mở cổng Tcl")

    def _pump(self) -> None:
        proc = self.proc
        if proc is None or proc.stdout is None:
            return
        for line in proc.stdout:
            line = line.rstrip()
            if line:
                self.lines.append(line)
                self.on_line(line)

    def last_error(self) -> str:
        errors = [l for l in self.lines if l.startswith("Error")]
        return " | ".join(errors[-2:])

    def raw(self, tcl: str, timeout: float) -> str:
        if self.sock is None:
            raise LinkError("chưa có phiên OpenOCD")
        try:
            self.sock.settimeout(timeout)
            self.sock.sendall(tcl.encode() + b"\x1a")
            buf = bytearray()
            while not buf.endswith(b"\x1a"):
                part = self.sock.recv(65536)
                if not part:
                    raise LinkError(self.last_error() or "OpenOCD đã đóng kết nối")
                buf += part
        except OSError as e:
            raise LinkError(self.last_error() or f"OpenOCD không trả lời ({e})") from None
        return buf[:-1].decode(errors="replace")

    def call(self, tcl: str, timeout: float = 30.0) -> str:
        """Run a command; raise CommandError with OpenOCD's message if it fails."""
        out = self.raw(f"if {{[catch {{{tcl}}} _r]}} {{set _r \"@@ERR@@$_r\"}} else {{set _r}}", timeout)
        if out.startswith("@@ERR@@"):
            raise CommandError(out[7:].strip() or self.last_error() or f"lệnh '{tcl.split()[0]}' lỗi")
        return out

    def read_u32(self, addr: int, timeout: float = 10.0) -> int:
        try:
            out = self.call(f"mdw {addr:#x}", timeout)
        except CommandError as e:
            raise LinkError(f"đọc {addr:#x} lỗi: {e}") from None
        m = re.search(r":\s*([0-9a-fA-F]{8})", out)
        if not m:
            raise LinkError(f"mdw trả về lạ: {out.strip()!r}")
        return int(m.group(1), 16)

    def close(self) -> None:
        if self.sock is not None:
            try:
                self.sock.settimeout(2)
                self.sock.sendall(b"shutdown\x1a")
            except OSError:
                pass
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None
        if self.proc is not None:
            try:
                self.proc.wait(3)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                try:
                    self.proc.wait(3)
                except subprocess.TimeoutExpired:
                    pass
            self.proc = None


# --------------------------------------------------------------------------- engine

STATE_TEXT = {"pending": "chờ", "checking": "kiểm tra", "erasing": "đang xóa", "writing": "đang ghi",
              "verifying": "xác minh", "done": "vừa ghi", "match": "đã đúng", "dirty": "ghi lại"}


class Flasher:
    """Owns the OpenOCD session and runs the jobs. Reports through emit(kind, **data) and
    asks the user through ask(text). Every use of the session holds self.lock."""

    def __init__(self, emit, ask):
        self.emit = emit
        self.ask = ask
        self.s = Settings()
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.ocd: OpenOcd | None = None
        self.chip = ""
        self._openocd: tuple[str, str | None] | None = None
        self._variant = 0
        self._halted = False         # this session halted the core
        self._halted_once = False    # the firmware was stopped at least once in this job
        self._assume_safe = False
        self.attempts = 0
        self.sessions = 0
        self.last_error = ""
        self._write_bytes = 0
        self._write_time = 0.0

    def run(self, job: str, settings: Settings, *args) -> bool:
        """Run one job (connect, disconnect, flash, reset_run, link_test) with these settings."""
        with self.lock:
            self.s = settings
            self.stop.clear()
            self.attempts = self.sessions = 0
            self.last_error = ""
            self._variant = 0
            self._halted_once = self._halted
            self._assume_safe = False
            self._write_bytes, self._write_time = 0, 0.0
            return getattr(self, job)(*args)

    # ---- helpers

    def log(self, text: str, level: str = "info") -> None:
        self.emit("log", text=text, level=level)

    def _check_stop(self) -> None:
        if self.stop.is_set():
            raise StopRequested()

    def _sleep(self, seconds: float) -> None:
        if self.stop.wait(seconds):
            raise StopRequested()

    def _variants(self) -> list[str]:
        chosen = f"{self.s.backend} {self.s.speed_khz}"
        if not self.s.rotate:
            return [chosen]
        return [chosen] + [v for v in self.s.variants if v != chosen]

    def _stats(self) -> None:
        self.emit("stats", attempts=self.attempts, sessions=self.sessions, last_error=self.last_error)

    def _ocd_line(self, line: str) -> None:
        self.emit("ocd", text=line)

    def _open_session(self) -> OpenOcd:
        """One attempt at a session with the current backend/clock; the chip must answer."""
        if self._openocd is None:
            self._openocd = find_openocd()
            if self.s.kill_other_openocd and os.name == "nt":
                r = subprocess.run(["taskkill", "/F", "/IM", "openocd.exe"], capture_output=True,
                                   creationflags=NO_WINDOW)
                if r.returncode == 0:
                    self.log("Đã đóng OpenOCD khác đang giữ DAPLink", "warn")
        exe, scripts = self._openocd
        variants = self._variants()
        backend, speed = variants[self._variant % len(variants)].split()
        self.attempts += 1
        self.log(f"Kết nối lần {self.attempts}: {backend}, {speed} kHz")
        self.emit("link", state="connecting", detail=f"lần {self.attempts} · {backend} {speed} kHz")
        ocd = OpenOcd(exe, scripts, backend, int(speed), self.s.probe_serial, self._ocd_line)
        try:
            ocd.start()
            idcode = ocd.read_u32(DBGMCU_IDCODE, timeout=25)
            if idcode & 0xFFF != DEV_ID_H72X:
                raise LinkError(f"IDCODE {idcode:#010x} không phải STM32H72x/73x (dữ liệu nhiễu?)")
            info = ocd.call("flash info 0", 20)
        except (LinkError, CommandError) as e:
            ocd.close()
            self.last_error = str(e)
            self._variant += 1
            self.log(f"  không được: {e}", "warn")
            self.emit("link", state="down", detail=str(e))
            self._stats()
            raise LinkError(str(e)) from None
        m = re.search(r"(STM32\S+)\s*-\s*Rev:\s*(\w+)", info)
        self.chip = f"{m.group(1)} rev {m.group(2)}" if m else "STM32H72x/73x"
        self.ocd = ocd
        self._halted = False
        self.sessions += 1
        self.log(f"  đã kết nối {self.chip}, IDCODE {idcode:#010x}", "ok")
        self.emit("link", state="up", detail=f"{backend} {speed} kHz", chip=self.chip, idcode=idcode)
        self._stats()
        return ocd

    def _ensure_session(self, attempts: int | None = None) -> OpenOcd:
        if self.ocd is not None:
            return self.ocd
        tries = 0
        while True:
            self._check_stop()
            try:
                return self._open_session()
            except LinkError:
                tries += 1
                if attempts is not None and tries >= attempts:
                    raise
                self._sleep(1.5)

    def _close(self) -> None:
        if self.ocd is not None:
            self.ocd.close()
            self.ocd = None

    def _drop(self, reason: str, progressed: bool) -> None:
        self.last_error = reason
        self.log(f"Mất liên kết: {reason}", "warn")
        if not progressed:
            self._variant += 1
        self._close()
        self.emit("link", state="down", detail=reason)
        self._stats()

    def _alive(self, ocd: OpenOcd) -> bool:
        try:
            return ocd.read_u32(DBGMCU_IDCODE, timeout=5) & 0xFFF == DEV_ID_H72X
        except LinkError:
            return False

    def _motor_status(self) -> int | None:
        try:
            r = subprocess.run(self.s.safety_cmd, shell=True, capture_output=True, text=True,
                               errors="replace", timeout=40, creationflags=NO_WINDOW)
        except subprocess.TimeoutExpired:
            return None
        values = re.findall(r"^\s*(\d+)\s*$", r.stdout, re.M)
        return int(values[-1]) if values else None

    def _ensure_safe(self) -> None:
        """A running core may only be halted while the motors are reported disabled."""
        if not self.s.safety:
            return
        waiting = False
        while True:
            self._check_stop()
            self.log("Kiểm tra động cơ: đọc /motor_status ...")
            v = self._motor_status()
            if v is None:
                if self._assume_safe or self._halted_once:
                    # This job already stopped the firmware once: what runs now is the flash loader.
                    self.log("  không đọc được /motor_status (firmware đã dừng từ trước)", "warn")
                    return
                if self.ask("Không đọc được /motor_status (Pi hoặc ROS không trả lời).\n\n"
                            "Chỉ tiếp tục nếu bạn chắc chắn động cơ đang TẮT.\n\nTiếp tục nạp?"):
                    self._assume_safe = True
                    return
                raise StopRequested("không xác nhận được động cơ đã tắt")
            if not v & MOTORS_ENABLED_BIT:
                self.log(f"  motor_status = {v}: động cơ tắt", "ok")
                return
            if not waiting:
                self.log(f"  motor_status = {v}: ĐỘNG CƠ ĐANG BẬT. Chờ tắt (disarm trên tay RC) ...", "error")
                waiting = True
            self._sleep(5)

    def _halt(self, ocd: OpenOcd) -> None:
        if not ocd.read_u32(DHCSR) & DHCSR_S_HALT:
            self._ensure_safe()
            ocd.call("reset halt", 20)
        # Also when the core was already halted: OpenOCD's own state must say so too.
        ocd.call("halt 2000", 10)
        self._halted = self._halted_once = True
        # RAM contents do not matter while the core sits in reset: skip saving the work area.
        ocd.call("[target current] configure -work-area-backup 0", 5)

    def _matches(self, ocd: OpenOcd, path: str, addr: int) -> bool:
        """On-target CRC of `path` against flash at `addr`."""
        try:
            ocd.call(f"verify_image_checksum {{{path}}} {addr:#x} bin", 120)
            return True
        except CommandError:
            if not self._alive(ocd):          # a plain mismatch keeps the probe answering
                raise LinkError(ocd.last_error() or "mất liên kết khi tính CRC") from None
            return False

    def _set(self, s: Sector, state: str, addr: int | None = None) -> None:
        s.state = state
        self.emit("sector", index=s.index, state=state, written=s.written, total=len(s.data), addr=addr)

    # ---- jobs

    def connect(self) -> bool:
        if self.ocd is not None and self._alive(self.ocd):
            self.emit("link", state="up", detail=f"{self.ocd.backend} {self.ocd.speed_khz} kHz", chip=self.chip)
            return True
        self._close()
        try:
            self._ensure_session(attempts=12)
            return True
        except LinkError:
            self.log(f"Không kết nối được sau {self.attempts} lần: {self.last_error}", "error")
            self.emit("link", state="off", detail="")
            return False
        except StopRequested:
            self.log("Đã dừng kết nối", "warn")
            self.emit("link", state="off", detail="")
            return False

    def disconnect(self) -> bool:
        self._close()
        self.emit("link", state="off", detail="")
        self.log("Đã ngắt kết nối mạch nạp")
        return True

    def ping(self) -> tuple[str, float | None]:
        """Idle link check from the GUI: ('busy'|'off'|'up'|'down', ms). Never waits for a job."""
        if not self.lock.acquire(blocking=False):
            return "busy", None
        try:
            if self.ocd is None:
                return "off", None
            t = time.monotonic()
            if self._alive(self.ocd):
                return "up", (time.monotonic() - t) * 1000
            self._close()
            self.emit("link", state="down", detail="mạch nạp không trả lời")
            return "down", None
        finally:
            self.lock.release()

    def flash(self, image: str) -> bool:
        path = Path(image)
        segments, notes = load_image(path)
        for n in notes:
            self.log(n, "warn")
        sectors = plan_sectors(segments)
        total = sum(len(s.data) for s in sectors)
        self.emit("plan", sectors=[(s.index, len(s.data)) for s in sectors], total=total)
        self.log(f"Nạp {path.name}: {total / 1024:.1f} KB vào sector {', '.join(str(s.index) for s in sectors)}")
        if self.s.force:
            for s in sectors:
                s.known_dirty = True
        t0 = time.monotonic()
        deadline = t0 + self.s.max_minutes * 60
        with tempfile.TemporaryDirectory(prefix="dap_flasher_") as tmp:
            tmpdir = Path(tmp)
            files: dict[tuple[int, int], str] = {}

            def data_file(addr: int, data: bytes) -> str:
                key = (addr, len(data)) if data.count(0xFF) != len(data) else (-1, len(data))
                if key not in files:
                    p = tmpdir / (f"blank_{len(data)}.bin" if key[0] < 0 else f"{addr:08x}_{len(data)}.bin")
                    p.write_bytes(data)
                    files[key] = p.as_posix()
                return files[key]

            for s in sectors:
                s.path = data_file(s.data_addr, s.data)
            try:
                while True:
                    pending = [s for s in sectors if s.state not in ("done", "match")]
                    if not pending:
                        break
                    self._check_stop()
                    if time.monotonic() > deadline:
                        self.log(f"Hết {self.s.max_minutes} phút mà chưa nạp xong", "error")
                        return False
                    try:
                        ocd = self._ensure_session(attempts=1)
                    except LinkError:
                        self._sleep(1.5)
                        continue
                    try:
                        if not self._halted:
                            self._halt(ocd)
                    except (LinkError, CommandError) as e:
                        self._drop(str(e), progressed=False)
                        self._sleep(1.5)
                        continue
                    before = sum(s.written for s in sectors)
                    try:
                        for s in pending:
                            self._program_sector(ocd, s, data_file, sectors, total, t0)
                    except (LinkError, CommandError) as e:
                        progressed = sum(s.written for s in sectors) > before
                        if isinstance(e, CommandError) and progressed and self._alive(ocd):
                            self.log(f"Lệnh lỗi, thử lại trong cùng phiên: {e}", "warn")
                            self.last_error = str(e)
                            self._stats()
                        else:
                            self._drop(str(e), progressed)
                        self._sleep(1.0)
                fresh = sum(len(s.data) for s in sectors if s.state == "done")
                matched = sum(1 for s in sectors if s.state == "match")
                self.log(f"Flash đúng: ghi mới {fresh / 1024:.1f} KB, {matched} sector đúng sẵn, "
                         f"{time.monotonic() - t0:.0f} s, {self.attempts} lần kết nối", "ok")
                self.emit("progress", done=total, total=total, rate=self._rate(), eta=0.0,
                          elapsed=time.monotonic() - t0)
                return self._reset_after_flash()
            except StopRequested as e:
                modified = any(s.erased or s.state == "done" for s in sectors)
                self.log(f"Đã dừng{': ' + str(e) if str(e) else ''}", "warn")
                if modified:
                    self.log("Flash đang dở nên chip được giữ ở trạng thái halt. Bấm Nạp để nạp tiếp; "
                             "sector nào đã đúng sẽ được bỏ qua.", "warn")
                elif self.ocd is not None and self._halted:
                    try:
                        self.ocd.call("reset run", 15)
                        self._halted = False
                        self.log("Chưa ghi gì nên đã cho STM32 chạy lại", "ok")
                    except (LinkError, CommandError):
                        pass
                return False

    def _rate(self) -> float:
        return self._write_bytes / self._write_time if self._write_time > 0 else 0.0

    def _program_sector(self, ocd, s: Sector, data_file, sectors, total, t0) -> None:
        chunks = s.chunks()
        if s.erased and s.in_flight and s.next_chunk < len(chunks):
            # The chunk in flight when the link broke: written, untouched or half written?
            addr, data = chunks[s.next_chunk]
            self._set(s, "checking", addr)
            if self._matches(ocd, data_file(addr, data), addr):
                self.log(f"Sector {s.index}: khối {fmt_addr(addr)} đã ghi đủ, ghi tiếp")
                s.next_chunk += 1
                s.written += len(data)
            elif self._matches(ocd, data_file(addr, b"\xff" * len(data)), addr):
                self.log(f"Sector {s.index}: khối {fmt_addr(addr)} chưa ghi, ghi tiếp từ đó")
            else:
                self.log(f"Sector {s.index}: khối {fmt_addr(addr)} ghi dở, xóa và ghi lại cả sector", "warn")
                s.erased = False
                s.known_dirty = True
            s.in_flight = False
        if not s.erased:
            if not s.known_dirty:
                self._set(s, "checking", s.data_addr)
                if self._matches(ocd, s.path, s.data_addr):
                    s.written = len(s.data)
                    self._set(s, "match")
                    self.log(f"Sector {s.index}: đã đúng sẵn, bỏ qua", "ok")
                    self._progress(sectors, total, t0)
                    return
            s.known_dirty = True
            s.written = 0
            self._set(s, "erasing", s.addr)
            ocd.call(f"flash erase_sector 0 {s.index} {s.index}", 30)
            s.erased = True
            s.next_chunk = 0
        while s.next_chunk < len(chunks):
            self._check_stop()
            addr, data = chunks[s.next_chunk]
            self._set(s, "writing", addr)
            if data.count(0xFF) != len(data):      # erased flash already reads 0xFF
                s.in_flight = True
                t = time.monotonic()
                ocd.call(f"flash write_bank 0 {{{data_file(addr, data)}}} {addr - FLASH_BASE:#x}", 90)
                self._write_time += time.monotonic() - t
                self._write_bytes += len(data)
                s.in_flight = False
            s.next_chunk += 1
            s.written += len(data)
            self._progress(sectors, total, t0)
        self._set(s, "verifying", s.data_addr)
        if self._matches(ocd, s.path, s.data_addr):
            self._set(s, "done")
            self.log(f"Sector {s.index}: ghi xong, CRC đúng", "ok")
            return
        s.erased = False
        s.written = 0
        self._set(s, "dirty")
        raise CommandError(f"sector {s.index}: CRC sai sau khi ghi")

    def _progress(self, sectors, total, t0) -> None:
        done = sum(s.written for s in sectors)
        rate = self._rate()
        eta = (total - done) / rate if rate > 0 else None
        self.emit("progress", done=done, total=total, rate=rate, eta=eta, elapsed=time.monotonic() - t0)

    def _reset_after_flash(self) -> bool:
        try:
            for _ in range(6):
                try:
                    ocd = self._ensure_session(attempts=1)
                    ocd.call("reset run", 15)
                    self._halted = False
                    self.log("Đã reset, STM32 chạy firmware mới", "ok")
                    return True
                except (LinkError, CommandError) as e:
                    if self.ocd is not None:
                        self._drop(str(e), progressed=False)
                    self._sleep(1.5)
        except StopRequested:
            pass
        self.log("Flash đã đúng nhưng chưa reset được qua DAPLink: bấm reset trên mạch hoặc tắt/bật nguồn",
                 "warn")
        return True

    def reset_run(self) -> bool:
        try:
            for _ in range(10):
                try:
                    ocd = self._ensure_session(attempts=1)
                    ocd.call("reset run", 15)
                    self._halted = False
                    self.log("Đã reset STM32", "ok")
                    return True
                except (LinkError, CommandError) as e:
                    if self.ocd is not None:
                        self._drop(str(e), progressed=False)
                    self._sleep(1.5)
            self.log("Không reset được sau 10 lần thử", "error")
            return False
        except StopRequested:
            self.log("Đã dừng", "warn")
            return False

    def link_test(self, pings: int = 20) -> bool:
        """Time IDCODE reads and a 4 KB flash read. Does not halt the core."""
        try:
            try:
                ocd = self._ensure_session(attempts=8)
            except LinkError:
                self.log(f"Không mở được phiên sau {self.attempts} lần: {self.last_error}", "error")
                return False
            ok, times = 0, []
            for _ in range(pings):
                self._check_stop()
                t = time.monotonic()
                if not self._alive(ocd):
                    break
                ok += 1
                times.append(time.monotonic() - t)
            rate = None
            if ok == pings:
                t = time.monotonic()
                try:
                    ocd.call(f"mdw {FLASH_BASE:#x} 1024", 60)
                    rate = 4.0 / (time.monotonic() - t)
                except (LinkError, CommandError):
                    pass
            avg = sum(times) / len(times) * 1000 if times else 0
            msg = (f"Liên kết: mở phiên sau {self.attempts} lần thử, {ok}/{pings} lệnh đọc OK "
                   f"(trung bình {avg:.0f} ms)")
            if rate:
                msg += f", đọc {rate:.1f} KB/s"
            self.log(msg, "ok" if ok == pings else "warn")
            return ok == pings
        except StopRequested:
            self.log("Đã dừng", "warn")
            return False


# --------------------------------------------------------------------------- GUI

# Graphite neutrals tinted toward solder-mask blue-green; ENIG gold is the one accent and marks
# what is being written. Semantic colours (ok / warn / err / info) are separate from it.
THEMES = {
    "light": {"bg": "#EEF2F3", "surface": "#FFFFFF", "sunken": "#E2E8EA", "field": "#F8FAFA",
              "line": "#CDD6DA", "fg": "#18222A", "muted": "#5B6971", "accent": "#DDA93F",
              "accent_hover": "#E6B85A", "accent_ink": "#18222A", "accent_text": "#8C5E0C",
              "ok": "#2E8B55", "match": "#8CC9A3", "warn": "#C46C1C", "err": "#C63B3B",
              "err_ink": "#FFFFFF", "info": "#3B7DCF"},
    "dark": {"bg": "#10161A", "surface": "#172025", "sunken": "#0B1013", "field": "#0F161A",
             "line": "#29353C", "fg": "#E2E9EC", "muted": "#8996A0", "accent": "#E2AE48",
             "accent_hover": "#EBC067", "accent_ink": "#10161A", "accent_text": "#E8BC63",
             "ok": "#4DB673", "match": "#2E6A47", "warn": "#E6924A", "err": "#E05A5A",
             "err_ink": "#FFFFFF", "info": "#5A99E4"},
}
STATE_COLOR = {"checking": "info", "erasing": "warn", "writing": "accent", "verifying": "info",
               "done": "ok", "match": "match", "dirty": "err"}


def _mix(a: str, b: str, t: float) -> str:
    """Colour a moved toward b by t (0..1); Tk has no alpha."""
    ca = [int(a[i:i + 2], 16) for i in (1, 3, 5)]
    cb = [int(b[i:i + 2], 16) for i in (1, 3, 5)]
    return "#" + "".join(f"{round(x + (y - x) * t):02x}" for x, y in zip(ca, cb))


def _windows_dark_mode() -> bool:
    try:
        import winreg
        with winreg.OpenKey(winreg.HKEY_CURRENT_USER,
                            r"Software\Microsoft\Windows\CurrentVersion\Themes\Personalize") as k:
            return winreg.QueryValueEx(k, "AppsUseLightTheme")[0] == 0
    except (ImportError, OSError):
        return False


def run_gui(settings: Settings) -> None:
    import tkinter as tk
    import tkinter.font as tkfont
    from tkinter import filedialog, messagebox, ttk

    try:
        import ctypes
        ctypes.windll.shcore.SetProcessDpiAwareness(1)
    except (ImportError, AttributeError, OSError):
        pass

    root = tk.Tk()
    theme = settings.theme if settings.theme in THEMES else ("dark" if _windows_dark_mode() else "light")
    T = THEMES[theme]
    families = set(tkfont.families(root))

    def pick(*names: str) -> str:
        return next((n for n in names if n in families), names[-1])

    display = pick("Bahnschrift SemiBold", "Bahnschrift", "Segoe UI Semibold")
    numeric = pick("Bahnschrift", "Segoe UI")
    body = pick("Segoe UI Variable Text", "Segoe UI")
    body_b = pick("Segoe UI Variable Text Semibold", "Segoe UI Semibold", "Segoe UI")
    mono = pick("Cascadia Mono", "Consolas", "Courier New")
    F = {"title": (display, 17), "step": (display, 12), "badge": (display, 10), "eyebrow": (display, 9),
         "big": (display, 36), "kb": (numeric, 15), "body": (body, 10), "body_b": (body_b, 10),
         "small": (body, 9), "mono": (mono, 9), "mono_s": (mono, 8), "button": (display, 13),
         "button_s": (body_b, 9)}
    S = T["surface"]

    style = ttk.Style(root)
    style.theme_use("clam")
    style.configure(".", background=S, foreground=T["fg"], font=F["body"], bordercolor=T["line"],
                    lightcolor=T["line"], darkcolor=T["line"], troughcolor=T["sunken"], focuscolor=T["accent"],
                    selectbackground=T["accent"], selectforeground=T["accent_ink"], insertcolor=T["fg"],
                    fieldbackground=T["field"], arrowcolor=T["muted"])
    style.configure("TEntry", padding=(8, 6))
    style.map("TEntry", bordercolor=[("focus", T["accent"])], lightcolor=[("focus", T["accent"])])
    style.configure("TCombobox", padding=(8, 5), background=T["field"])
    style.map("TCombobox", fieldbackground=[("readonly", T["field"]), ("disabled", T["sunken"])],
              foreground=[("readonly", T["fg"]), ("disabled", T["muted"])],
              selectbackground=[("readonly", T["field"])], selectforeground=[("readonly", T["fg"])],
              bordercolor=[("focus", T["accent"])], lightcolor=[("focus", T["accent"])],
              background=[("active", T["field"]), ("pressed", T["field"])])
    style.configure("TSpinbox", padding=(8, 5), background=T["field"])
    style.map("TSpinbox", bordercolor=[("focus", T["accent"])], lightcolor=[("focus", T["accent"])],
              background=[("active", T["field"])])
    style.configure("TCheckbutton", background=S, foreground=T["fg"], indicatorbackground=T["field"],
                    indicatorforeground=T["accent_ink"], indicatormargin=(0, 0, 8, 0), padding=(0, 3),
                    focuscolor=S)
    style.map("TCheckbutton", indicatorbackground=[("selected", T["accent"]), ("active", T["sunken"])],
              background=[("active", S)], foreground=[("disabled", T["muted"])])
    def box_image(checked: bool, size: int = 16):
        img = tk.PhotoImage(width=size, height=size)
        img.put(T["accent"] if checked else T["line"], to=(1, 1, size - 1, size - 1))
        if checked:
            for x, y in ((4, 8), (5, 9), (6, 10), (7, 11), (8, 10), (9, 9), (10, 8), (11, 7), (12, 6)):
                img.put(T["accent_ink"], to=(x, y - 1, x + 2, y + 1))
        else:
            img.put(T["field"], to=(2, 2, size - 2, size - 2))
        for x, y in ((1, 1), (size - 2, 1), (1, size - 2), (size - 2, size - 2)):
            img.transparency_set(x, y, True)
        return img

    root.check_images = (box_image(False), box_image(True))   # Tk keeps no reference of its own
    style.element_create("Gold.Checkbutton.indicator", "image", root.check_images[0],
                         ("selected", root.check_images[1]), width=26, sticky="w")
    style.layout("TCheckbutton", [("Checkbutton.padding", {"sticky": "nswe", "children": [
        ("Gold.Checkbutton.indicator", {"side": "left", "sticky": ""}),
        ("Checkbutton.focus", {"side": "left", "sticky": "w", "children": [
            ("Checkbutton.label", {"sticky": "nswe"})]})]})])
    style.configure("Bg.TCheckbutton", background=T["bg"], focuscolor=T["bg"])
    style.map("Bg.TCheckbutton", background=[("active", T["bg"])])
    style.configure("Vertical.TScrollbar", background=T["sunken"], troughcolor=S, bordercolor=S,
                    lightcolor=T["sunken"], darkcolor=T["sunken"], arrowcolor=T["muted"], gripcount=0)
    style.map("Vertical.TScrollbar", background=[("active", T["line"])])
    for opt, val in (("background", T["field"]), ("foreground", T["fg"]), ("selectBackground", T["accent"]),
                     ("selectForeground", T["accent_ink"]), ("font", F["body"]), ("borderWidth", 0)):
        root.option_add(f"*TCombobox*Listbox.{opt}", val)

    BUTTONS = {   # kind: background, text, hover, border, font, padx, pady
        "primary": (T["accent"], T["accent_ink"], T["accent_hover"], T["accent"], F["button"], 20, 11),
        "stop": (T["err"], T["err_ink"], _mix(T["err"], "#ffffff", 0.12), T["err"], F["button"], 20, 11),
        "secondary": (T["sunken"], T["fg"], _mix(T["sunken"], T["fg"], 0.10), T["line"], F["body_b"], 16, 6),
        "ghost": (S, T["fg"], T["sunken"], T["line"], F["button_s"], 10, 5),
        "link": (S, T["accent_text"], S, S, (body, 9, "underline"), 0, 0),
    }

    class Btn(tk.Label):
        """Flat button drawn by us, so it looks the same in both themes."""

        def __init__(self, parent, text: str, command, kind: str = "ghost"):
            super().__init__(parent, text=text, takefocus=1, highlightthickness=1, cursor="hand2")
            self.command, self.kind, self.enabled, self.hover = command, kind, True, False
            self.bind("<Enter>", lambda e: self._hover(True))
            self.bind("<Leave>", lambda e: self._hover(False))
            self.bind("<Button-1>", self._click)
            self.bind("<Return>", self._click)
            self.bind("<space>", self._click)
            self._paint()

        def _hover(self, on: bool) -> None:
            self.hover = on
            self._paint()

        def _paint(self) -> None:
            bg, fg, hover, border, font, padx, pady = BUTTONS[self.kind]
            if not self.enabled:
                if self.kind in ("primary", "stop", "secondary"):
                    bg, border = T["sunken"], T["line"]
                fg = _mix(T["muted"], bg, 0.25)
            elif self.hover:
                bg = hover
            self.configure(bg=bg, fg=fg, font=font, padx=padx, pady=pady, highlightbackground=border,
                           highlightcolor=T["accent"], cursor="hand2" if self.enabled else "arrow")

        def set(self, text: str | None = None, kind: str | None = None, enabled: bool | None = None) -> None:
            if text is not None:
                self.configure(text=text)
            if kind is not None:
                self.kind = kind
            if enabled is not None:
                self.enabled = enabled
            self._paint()

        def _click(self, _e=None):
            if self.enabled and self.command:
                self.command()
            return "break"

    class App:
        def __init__(self):
            self.events: queue.Queue = queue.Queue()
            self.flasher = Flasher(self._emit, self._ask)
            self.worker: threading.Thread | None = None
            self.job = ""
            self.link_state = "off"
            self.image_ok = False
            self.sector_view: dict[int, dict] = {}
            self.image_range: tuple[int, int] | None = None
            self.image_total = 0
            self.done = 0
            self.result: bool | None = None
            self.started = 0.0
            self.probes: list[tuple[str, str]] = []

            root.title("DAPLink Flasher")
            icon = BUNDLE_DIR / "dap_flasher.ico"
            if icon.is_file():
                try:
                    root.iconbitmap(default=str(icon))
                except tk.TclError:
                    pass
            root.configure(bg=T["bg"])
            root.geometry("1140x820")
            root.minsize(1000, 760)
            self._title_bar()

            # header band
            head = tk.Frame(root, bg=T["sunken"])
            head.pack(fill="x")
            hi = tk.Frame(head, bg=T["sunken"])
            hi.pack(fill="x", padx=24, pady=(14, 13))
            tk.Label(hi, text="DAPLink Flasher", font=F["title"], bg=T["sunken"], fg=T["fg"]).pack(side="left")
            tk.Label(hi, text="STM32H723  ·  CMSIS-DAP không dây  ·  OpenOCD", font=F["small"],
                     bg=T["sunken"], fg=T["muted"]).pack(side="left", padx=(14, 0), pady=(6, 0))
            self.pill = tk.Label(hi, font=F["body_b"], padx=12, pady=3)
            self.pill.pack(side="right")

            main = tk.Frame(root, bg=T["bg"])
            main.pack(fill="both", expand=True, padx=24, pady=(20, 22))
            main.columnconfigure(1, weight=1)
            main.rowconfigure(0, weight=1)
            left = tk.Frame(main, bg=T["bg"], width=410)
            left.grid(row=0, column=0, sticky="ns", padx=(0, 24))
            left.pack_propagate(False)
            right = tk.Frame(main, bg=T["bg"])
            right.grid(row=0, column=1, sticky="nsew")

            self._build_steps(left)
            self._build_progress(right)

            self.var_image.trace_add("write", lambda *a: self._image_info())
            self._refresh_probes()
            self._image_info()
            self._link_view("off", "")
            self._readout_idle()
            self._startup_log()
            root.protocol("WM_DELETE_WINDOW", self._close)
            root.after(50, self._poll)
            root.after(1000, self._tick)
            root.after(3000, self._idle_ping)

        # ---- layout

        def _startup_log(self) -> None:
            try:
                exe, _ = find_openocd()
                m = re.search(r"STM32CubeIDE_([\d.]+)", exe)
                if Path(exe).is_relative_to(BUNDLE_DIR / "openocd"):
                    self._append("OpenOCD: bản đóng gói sẵn trong app", "ocd")
                else:
                    self._append(f"OpenOCD: bản kèm STM32CubeIDE {m.group(1)}" if m else f"OpenOCD: {exe}", "ocd")
            except FileNotFoundError as e:
                self._append(str(e), "err")
            if self.probes:
                names = ", ".join(lab for lab, _ in self.probes)
                self._append(f"Thấy {len(self.probes)} mạch nạp CMSIS-DAP: {names}", "info")
            self._append("Bấm Kết nối để kiểm tra đường truyền, hoặc bấm Nạp firmware luôn.", "info")

        def _title_bar(self) -> None:
            if theme != "dark":
                return
            try:
                import ctypes
                root.update_idletasks()
                hwnd = ctypes.windll.user32.GetParent(root.winfo_id())
                on = ctypes.c_int(1)
                ctypes.windll.dwmapi.DwmSetWindowAttribute(hwnd, 20, ctypes.byref(on), ctypes.sizeof(on))
            except (ImportError, AttributeError, OSError):
                pass

        def _step(self, parent, n: int, title: str):
            card = tk.Frame(parent, bg=S, highlightthickness=1, highlightbackground=T["line"])
            card.pack(fill="x", pady=(0, 10))
            head = tk.Frame(card, bg=S)
            head.pack(fill="x", padx=16, pady=(11, 7))
            badge = tk.Label(head, text=str(n), font=F["badge"], width=2, bg=T["sunken"], fg=T["muted"])
            badge.pack(side="left")
            tk.Label(head, text=title, font=F["step"], bg=S, fg=T["fg"]).pack(side="left", padx=(10, 0))
            inner = tk.Frame(card, bg=S)
            inner.pack(fill="x", padx=16, pady=(0, 12))
            return inner, badge, head

        def _build_steps(self, left) -> None:
            # The flash button sits at the bottom of the column so it never scrolls out of view.
            self.lbl_hint = tk.Label(left, text="", font=F["small"], bg=T["bg"], fg=T["muted"], anchor="w",
                                     justify="left", wraplength=400)
            self.lbl_hint.pack(side="bottom", fill="x", pady=(8, 0))
            self.btn_flash = Btn(left, "Nạp firmware", self._flash_or_stop, "primary")
            self.btn_flash.pack(side="bottom", fill="x")

            # 1. probe
            box, self.badge1, head1 = self._step(left, 1, "Kết nối mạch nạp")
            self.btn_reset = Btn(head1, "Reset STM32", lambda: self._start("reset_run"), "ghost")
            self.btn_reset.pack(side="right")
            row = tk.Frame(box, bg=S)
            row.pack(fill="x")
            self.var_probe = tk.StringVar()
            self.cmb_probe = ttk.Combobox(row, textvariable=self.var_probe, state="readonly")
            self.cmb_probe.pack(side="left", fill="x", expand=True)
            Btn(row, "↻", self._refresh_probes, "ghost").pack(side="left", padx=(6, 0))
            self.btn_connect = Btn(row, "Kết nối", self._toggle_connect, "secondary")
            self.btn_connect.pack(side="left", padx=(6, 0))
            self.lbl_link = tk.Label(box, text="", font=F["small"], bg=S, fg=T["muted"], anchor="w",
                                     justify="left", wraplength=360)
            self.lbl_link.pack(fill="x", pady=(8, 0))

            # 2. firmware
            box, self.badge2, _ = self._step(left, 2, "Chọn firmware")
            row = tk.Frame(box, bg=S)
            row.pack(fill="x")
            self.var_image = tk.StringVar(value=settings.image)
            ttk.Entry(row, textvariable=self.var_image).pack(side="left", fill="x", expand=True)
            Btn(row, "Chọn file…", self._browse, "secondary").pack(side="left", padx=(6, 0))
            self.lbl_image = tk.Label(box, text="", font=F["small"], bg=S, fg=T["muted"], anchor="w",
                                      justify="left", wraplength=360)
            self.lbl_image.pack(fill="x", pady=(8, 0))

            # 3. parameters
            box, self.badge3, _ = self._step(left, 3, "Thông số nạp")
            self.var_backend = tk.StringVar(value=settings.backend)
            self.var_speed = tk.StringVar(value=str(settings.speed_khz))
            self.var_minutes = tk.StringVar(value=str(settings.max_minutes))
            rows = ((("Giao tiếp", ttk.Combobox, dict(textvariable=self.var_backend, width=8, state="readonly",
                                                   values=["hid", "usb_bulk", "auto"]), ""),
                     ("SWD", ttk.Combobox, dict(textvariable=self.var_speed, width=5,
                                                    values=["200", "400", "1000", "1800", "4000"]), "kHz")),
                    (("Thử tối đa", ttk.Spinbox, dict(from_=1, to=240, textvariable=self.var_minutes, width=5),
                      "phút"),))
            for cells in rows:
                line = tk.Frame(box, bg=S)
                line.pack(fill="x", pady=3)
                for n, (label, cls, kw, unit) in enumerate(cells):
                    tk.Label(line, text=label, font=F["body"], bg=S, fg=T["muted"]).pack(
                        side="left", padx=(16 if n else 0, 8))
                    cls(line, **kw).pack(side="left")
                    if unit:
                        tk.Label(line, text=unit, font=F["small"], bg=S, fg=T["muted"]).pack(side="left", padx=(5, 0))
            self.var_rotate = tk.BooleanVar(value=settings.rotate)
            self.var_force = tk.BooleanVar(value=settings.force)
            self.var_safety = tk.BooleanVar(value=settings.safety)
            self.var_safety_cmd = tk.StringVar(value=settings.safety_cmd)
            opts = tk.Frame(box, bg=S)
            opts.pack(fill="x", pady=(6, 0))
            ttk.Checkbutton(opts, text="Tự đổi giao tiếp/xung khi không kết nối được",
                            variable=self.var_rotate).pack(anchor="w")
            ttk.Checkbutton(opts, text="Ghi lại cả sector đã đúng", variable=self.var_force).pack(anchor="w")
            srow = tk.Frame(opts, bg=S)
            srow.pack(fill="x")
            ttk.Checkbutton(srow, text="Chỉ dừng chip khi động cơ đã tắt",
                            variable=self.var_safety).pack(side="left")
            self.lnk_cmd = Btn(srow, "sửa lệnh", self._toggle_cmd, "link")
            self.lnk_cmd.pack(side="left", padx=(8, 0))
            self.ent_cmd = ttk.Entry(opts, textvariable=self.var_safety_cmd, font=F["mono_s"])

        def _build_progress(self, right) -> None:
            top = tk.Frame(right, bg=T["bg"])
            top.pack(fill="x")
            tk.Label(top, text="BỘ NHỚ FLASH  ·  1 MB  ·  8 × 128 KB", font=F["eyebrow"], bg=T["bg"],
                     fg=T["muted"]).pack(side="left")
            legend = tk.Frame(top, bg=T["bg"])
            legend.pack(side="right")
            for key, text in (("match", "đã đúng"), ("ok", "vừa ghi"), ("accent", "đang ghi"), ("err", "ghi lại")):
                sw = tk.Canvas(legend, width=10, height=10, bg=T["bg"], highlightthickness=0)
                sw.create_rectangle(0, 0, 10, 10, fill=T[key], width=0)
                sw.pack(side="left", padx=(12, 5))
                tk.Label(legend, text=text, font=F["small"], bg=T["bg"], fg=T["muted"]).pack(side="left")
            self.map = tk.Canvas(right, height=124, bg=T["bg"], highlightthickness=0)
            self.map.pack(fill="x", pady=(10, 14))
            self.map.bind("<Configure>", lambda e: self._draw_map())

            ro = tk.Frame(right, bg=T["bg"])
            ro.pack(fill="x")
            self.lbl_pct = tk.Label(ro, text="0%", font=F["big"], bg=T["bg"], fg=T["muted"], width=4, anchor="w")
            self.lbl_pct.pack(side="left")
            side = tk.Frame(ro, bg=T["bg"])
            side.pack(side="right")
            self.lbl_kb = tk.Label(side, text="", font=F["kb"], bg=T["bg"], fg=T["fg"], anchor="e")
            self.lbl_kb.pack(anchor="e")
            self.lbl_rate = tk.Label(side, text="", font=F["small"], bg=T["bg"], fg=T["muted"], anchor="e")
            self.lbl_rate.pack(anchor="e")
            mid = tk.Frame(ro, bg=T["bg"])
            mid.pack(side="left", fill="x", expand=True, padx=(8, 12))
            self.lbl_now = tk.Label(mid, text="", font=F["body_b"], bg=T["bg"], fg=T["fg"], anchor="w")
            self.lbl_now.pack(fill="x")
            self.lbl_detail = tk.Label(mid, text="", font=F["small"], bg=T["bg"], fg=T["muted"], anchor="w")
            self.lbl_detail.pack(fill="x")
            self.bar = tk.Canvas(right, height=6, bg=T["sunken"], highlightthickness=0)
            self.bar.pack(fill="x", pady=(12, 6))
            self.bar.bind("<Configure>", lambda e: self._draw_bar())
            self.lbl_stats = tk.Label(right, text="", font=F["small"], bg=T["bg"], fg=T["muted"], anchor="w")
            self.lbl_stats.pack(fill="x")

            logbar = tk.Frame(right, bg=T["bg"])
            logbar.pack(fill="x", pady=(18, 6))
            tk.Label(logbar, text="NHẬT KÝ", font=F["eyebrow"], bg=T["bg"], fg=T["muted"]).pack(side="left")
            self.var_ocdlog = tk.BooleanVar(value=settings.show_openocd_log)
            ttk.Checkbutton(logbar, text="kèm log OpenOCD", variable=self.var_ocdlog,
                            style="Bg.TCheckbutton").pack(side="right")
            card = tk.Frame(right, bg=S, highlightthickness=1, highlightbackground=T["line"])
            card.pack(fill="both", expand=True)
            self.txt = tk.Text(card, wrap="word", font=F["mono"], state="disabled", bg=S, fg=T["fg"],
                               relief="flat", borderwidth=0, padx=12, pady=10, insertbackground=T["fg"],
                               selectbackground=T["accent"], selectforeground=T["accent_ink"], spacing1=2)
            sb = ttk.Scrollbar(card, command=self.txt.yview)
            self.txt.configure(yscrollcommand=sb.set)
            sb.pack(side="right", fill="y")
            self.txt.pack(side="left", fill="both", expand=True)
            self.txt.tag_configure("time", foreground=_mix(T["muted"], S, 0.25))
            for tag in ("ok", "warn", "err"):
                self.txt.tag_configure(tag, foreground=T[tag])
            self.txt.tag_configure("ocd", foreground=T["muted"])

        # ---- settings, probes, image

        def _settings(self) -> Settings:
            s = Settings.load()
            s.image = self.var_image.get().strip()
            s.probe_serial = self._probe_serial()
            s.backend = self.var_backend.get()
            try:
                s.speed_khz = int(self.var_speed.get())
            except ValueError:
                pass
            try:
                s.max_minutes = max(1, int(self.var_minutes.get()))
            except ValueError:
                pass
            s.rotate = self.var_rotate.get()
            s.force = self.var_force.get()
            s.safety = self.var_safety.get()
            s.safety_cmd = self.var_safety_cmd.get().strip()
            s.show_openocd_log = self.var_ocdlog.get()
            return s

        def _probe_serial(self) -> str:
            label = self.var_probe.get()
            return next((sn for lab, sn in self.probes if lab == label), "")

        def _refresh_probes(self) -> None:
            self.probes = list_probes()
            labels = [lab for lab, _ in self.probes]
            auto = "Tự động (mạch nạp đầu tiên)"
            self.cmb_probe.configure(values=labels + [auto])
            want = settings.probe_serial
            chosen = next((lab for lab, sn in self.probes if sn == want), labels[0] if labels else auto)
            self.var_probe.set(chosen)
            if not self.probes and self.link_state == "off":
                self.lbl_link.configure(text="Không thấy mạch nạp CMSIS-DAP trên USB. Cắm dongle rồi bấm ↻.",
                                        fg=T["warn"])

        def _browse(self) -> None:
            cur = self.var_image.get().strip()
            start = str(Path(cur).parent) if cur else str(Path.home())
            path = filedialog.askopenfilename(initialdir=start, title="Chọn firmware",
                                              filetypes=[("Firmware", "*.elf *.hex *.bin"), ("Tất cả", "*.*")])
            if path:
                self.var_image.set(str(Path(path)))

        def _image_info(self) -> None:
            path = Path(self.var_image.get().strip())
            self.image_ok = False
            try:
                if not path.is_file():
                    raise ValueError("chưa chọn file" if not str(path) or str(path) == "." else "không thấy file")
                segments, notes = load_image(path)
                sectors = plan_sectors(segments)
                self.image_total = sum(len(s.data) for s in sectors)
                self.image_range = (min(s.data_addr for s in sectors),
                                    max(s.data_addr + len(s.data) for s in sectors))
                when = time.strftime("%H:%M %d/%m", time.localtime(path.stat().st_mtime))
                span = f"S{sectors[0].index}" + (f"–S{sectors[-1].index}" if len(sectors) > 1 else "")
                self.lbl_image.configure(text=f"{self.image_total / 1024:.1f} KB  ·  {span}  ·  build {when}"
                                              + (f"\n{notes[0]}" if notes else ""), fg=T["muted"])
                self.image_ok = True
                if not self._busy():
                    self.sector_view = {s.index: {"state": "pending", "written": 0, "total": len(s.data)}
                                        for s in sectors}
                    self.done, self.result = 0, None
                    self._readout_idle()
            except (OSError, ValueError) as e:
                self.lbl_image.configure(text=f"Không đọc được firmware: {e}", fg=T["err"])
                self.image_range, self.image_total = None, 0
                if not self._busy():
                    self.sector_view = {}
                    self._readout_idle()
            self._badge(self.badge2, self.image_ok, 2)
            self._draw_map()
            self._update_controls()

        def _toggle_cmd(self) -> None:
            if self.ent_cmd.winfo_ismapped():
                self.ent_cmd.pack_forget()
            else:
                self.ent_cmd.pack(fill="x", pady=(4, 0))

        # ---- jobs

        def _busy(self) -> bool:
            return bool(self.worker and self.worker.is_alive())

        def _start(self, job: str, *args) -> None:
            if self._busy():
                return
            s = self._settings()
            s.save()
            self.job = job
            self.started = time.monotonic()
            if job == "flash":
                self.done, self.result = 0, None
                self.lbl_stats.configure(text="")
            self._update_controls()

            def work() -> None:
                ok = False
                try:
                    ok = self.flasher.run(job, s, *args)
                except Exception as e:      # show anything unexpected instead of dying silently
                    self._emit("log", text=f"Lỗi: {e}", level="err")
                self._emit("finished", ok=ok, job=job)

            self.worker = threading.Thread(target=work, daemon=True)
            self.worker.start()
            self._update_controls()

        def _toggle_connect(self) -> None:
            if self._busy():
                if self.job == "connect":
                    self.flasher.stop.set()
                return
            self._start("disconnect" if self.link_state == "up" else "connect")

        def _flash_or_stop(self) -> None:
            if self._busy():
                if self.job == "flash":
                    self.flasher.stop.set()
                    self.btn_flash.set(text="Đang dừng…", enabled=False)
                return
            s = self._settings()
            if not Path(s.image).is_file():
                messagebox.showerror("DAPLink Flasher", f"Không thấy file firmware:\n{s.image}")
                return
            self._start("flash", s.image)

        def _update_controls(self) -> None:
            busy = self._busy()
            job = self.job if busy else ""
            if job == "flash":
                self.btn_flash.set(text="Dừng nạp", kind="stop", enabled=not self.flasher.stop.is_set())
            else:
                self.btn_flash.set(text="Nạp firmware", kind="primary", enabled=not busy and self.image_ok)
            if job == "connect":
                self.btn_connect.set(text="Hủy", enabled=True)
            else:
                self.btn_connect.set(text="Ngắt" if self.link_state == "up" else "Kết nối", enabled=not busy)
            self.btn_reset.set(enabled=not busy and self.link_state == "up")
            self.cmb_probe.configure(state="disabled" if busy else "readonly")
            if busy:
                hint = {"flash": "Mất link thì app tự kết nối lại và nạp tiếp chỗ dở.",
                        "connect": "Đang mở phiên OpenOCD qua DAPLink…"}.get(job, "")
            elif not self.image_ok:
                hint = "Chọn file .elf, .hex hoặc .bin để nạp."
            elif self.link_state != "up":
                hint = "Bấm Nạp ngay cũng được: app tự kết nối."
            else:
                hint = "Sẵn sàng nạp. Sector nào đã đúng sẽ được bỏ qua."
            self.lbl_hint.configure(text=hint)

        def _close(self) -> None:
            if self._busy():
                if not messagebox.askyesno("DAPLink Flasher", "Đang chạy. Dừng và thoát?"):
                    return
                self.flasher.stop.set()
                if self.flasher.ocd is not None:
                    self.flasher.ocd.close()
            elif self.flasher.ocd is not None:
                self.flasher.ocd.close()
            self._settings().save()
            root.destroy()

        def _idle_ping(self) -> None:
            if self.link_state == "up" and not self._busy():
                def work() -> None:
                    state, ms = self.flasher.ping()
                    self._emit("ping", state=state, ms=ms)
                threading.Thread(target=work, daemon=True).start()
            root.after(3000, self._idle_ping)

        # ---- worker -> UI

        def _emit(self, kind: str, **data) -> None:
            self.events.put((kind, data))

        def _ask(self, question: str) -> bool:
            holder = {"answer": False, "event": threading.Event()}
            self.events.put(("ask", {"question": question, "holder": holder}))
            while not holder["event"].wait(0.2):
                if self.flasher.stop.is_set():
                    return False
            return holder["answer"]

        def _poll(self) -> None:
            try:
                for _ in range(500):
                    kind, d = self.events.get_nowait()
                    getattr(self, f"_on_{kind}")(**d)
            except queue.Empty:
                pass
            root.after(50, self._poll)

        def _tick(self) -> None:
            if self._busy() and self.job == "flash":
                el = int(time.monotonic() - self.started)
                self.lbl_rate.configure(text=self._rate_text + f"đã chạy {el // 60:02d}:{el % 60:02d}")
            root.after(1000, self._tick)

        _rate_text = ""

        def _append(self, text: str, tag: str) -> None:
            self.txt.configure(state="normal")
            self.txt.insert("end", time.strftime("%H:%M:%S  "), "time")
            self.txt.insert("end", text + "\n", tag)
            if int(self.txt.index("end-1c").split(".")[0]) > 4000:
                self.txt.delete("1.0", "500.0")
            self.txt.see("end")
            self.txt.configure(state="disabled")

        def _on_log(self, text: str, level: str) -> None:
            self._append(text, {"error": "err"}.get(level, level))

        def _on_ocd(self, text: str) -> None:
            if self.var_ocdlog.get():
                self._append(text, "ocd")

        def _on_link(self, state: str, detail: str, chip: str = "", idcode: int = 0) -> None:
            self._link_view(state, detail, chip, idcode)

        def _link_view(self, state: str, detail: str, chip: str = "", idcode: int = 0, ms: float | None = None):
            self.link_state = state
            pill = {"off": ("muted", "Chưa kết nối"), "connecting": ("info", "Đang kết nối…"),
                    "up": ("ok", "Đã kết nối"), "down": ("warn", "Mất liên kết")}[state]
            color = T[pill[0]]
            self.pill.configure(text=f"●  {pill[1]}", fg=color, bg=_mix(color, T["sunken"], 0.82))
            if state == "up":
                chip = chip or self.flasher.chip
                text = f"{chip}  ·  {detail}" + (f"  ·  IDCODE {idcode:#010x}" if idcode else "")
                if ms is not None:
                    text += f"  ·  phản hồi {ms:.0f} ms"
                self.lbl_link.configure(text=text, fg=T["ok"])
            elif state == "connecting":
                self.lbl_link.configure(text=f"Đang thử {detail}", fg=T["info"])
            elif state == "down":
                self.lbl_link.configure(text=f"Mất liên kết: {detail[:140]}", fg=T["warn"])
            else:
                self.lbl_link.configure(text="Chip vẫn chạy khi kết nối, chỉ dừng lúc nạp.", fg=T["muted"])
            self._badge(self.badge1, state == "up", 1)
            self._update_controls()

        def _on_ping(self, state: str, ms: float | None) -> None:
            if state == "up" and self.link_state == "up":
                cur = self.lbl_link.cget("text").split("  ·  phản hồi")[0]
                self.lbl_link.configure(text=f"{cur}  ·  phản hồi {ms:.0f} ms")

        def _on_plan(self, sectors, total) -> None:
            self.sector_view = {i: {"state": "pending", "written": 0, "total": n} for i, n in sectors}
            self.image_total = total
            self.done = 0
            self._draw_map()
            self._draw_bar()

        def _on_sector(self, index, state, written, total, addr) -> None:
            self.sector_view[index] = {"state": state, "written": written, "total": total}
            self._draw_map()
            if state in ("checking", "erasing", "writing", "verifying"):
                where = f"  ·  {fmt_addr(addr)}" if addr is not None else ""
                self.lbl_now.configure(text=f"Sector {index}: {STATE_TEXT[state]}{where}")
                self.lbl_detail.configure(text={"checking": "so CRC trên chip với file",
                                                "erasing": "xóa 128 KB",
                                                "writing": f"{written / 1024:.0f} / {total / 1024:.0f} KB của sector",
                                                "verifying": "kiểm CRC cả sector"}[state])

        def _on_progress(self, done, total, rate, eta, elapsed) -> None:
            self.done, self.image_total = done, total
            pct = done / total * 100 if total else 0
            self.lbl_pct.configure(text=f"{pct:.0f}%", fg=T["fg"])
            self.lbl_kb.configure(text=f"{done / 1024:.1f} / {total / 1024:.1f} KB")
            parts = []
            if rate:
                parts.append(f"ghi {rate / 1024:.1f} KB/s")
            if eta:
                parts.append(f"còn ~{eta:.0f} s nếu link tốt")
            self._rate_text = "  ·  ".join(parts) + ("  ·  " if parts else "")
            self._draw_bar()

        def _on_stats(self, attempts, sessions, last_error) -> None:
            text = f"Kết nối {attempts} lần, mở được {sessions} phiên"
            if last_error:
                text += f"  ·  lỗi gần nhất: {last_error[:110]}"
            self.lbl_stats.configure(text=text)

        def _on_ask(self, question, holder) -> None:
            root.deiconify()
            root.lift()
            holder["answer"] = messagebox.askyesno("DAPLink Flasher", question, icon="warning")
            holder["event"].set()

        def _on_finished(self, ok, job) -> None:
            self.worker = None
            if job == "flash":
                self.result = ok
                if ok:
                    self.lbl_pct.configure(text="100%", fg=T["ok"])
                    self.lbl_now.configure(text="Nạp xong, CRC từng sector đều đúng")
                    self.lbl_detail.configure(text="STM32 đã reset và chạy firmware mới")
                else:
                    self.lbl_pct.configure(fg=T["err"])
                    self.lbl_now.configure(text="Chưa nạp xong")
                    self.lbl_detail.configure(text="Bấm Nạp firmware để nạp tiếp từ chỗ đang dở")
                el = int(time.monotonic() - self.started)
                self.lbl_rate.configure(text=self._rate_text + f"tổng {el // 60:02d}:{el % 60:02d}")
                self._draw_bar()
                root.title(f"DAPLink Flasher – {'nạp xong' if ok else 'chưa xong'}")
                try:
                    import winsound
                    winsound.MessageBeep(winsound.MB_ICONASTERISK if ok else winsound.MB_ICONHAND)
                except (ImportError, RuntimeError):
                    root.bell()
            self._update_controls()

        # ---- drawing

        def _badge(self, badge, done: bool, n: int) -> None:
            badge.configure(text="✓" if done else str(n), bg=T["accent"] if done else T["sunken"],
                            fg=T["accent_ink"] if done else T["muted"])

        def _readout_idle(self) -> None:
            self.lbl_pct.configure(text="0%", fg=T["muted"])
            total = self.image_total
            self.lbl_kb.configure(text=f"0 / {total / 1024:.1f} KB" if total else "")
            self.lbl_now.configure(text="Chưa nạp" if total else "Chưa có firmware")
            self.lbl_detail.configure(text="Sector đã đúng sẵn sẽ được bỏ qua"
                                      if total else "Chọn file ở bước 2")
            self.lbl_rate.configure(text="")
            self._rate_text = ""
            self._draw_bar()

        def _draw_bar(self) -> None:
            c = self.bar
            c.delete("all")
            w = max(c.winfo_width(), 10)
            frac = self.done / self.image_total if self.image_total else 0
            color = T["ok"] if self.result else T["err"] if self.result is False else T["accent"]
            if frac > 0:
                c.create_rectangle(0, 0, w * frac, 6, fill=color, width=0)

        def _draw_map(self) -> None:
            c = self.map
            c.delete("all")
            w = max(c.winfo_width(), 300)
            gap, y0, h = 4, 28, 46
            bw = (w - gap * (SECTOR_COUNT - 1)) / SECTOR_COUNT

            def x_of(addr: int) -> float:
                off = addr - FLASH_BASE
                i = min(off // SECTOR_SIZE, SECTOR_COUNT - 1)
                return i * (bw + gap) + (off - i * SECTOR_SIZE) / SECTOR_SIZE * bw

            if self.image_range:
                xs, xe = x_of(self.image_range[0]), x_of(self.image_range[1] - 1)
                col = T["accent_text"]
                c.create_line(xs, 19, xe, 19, fill=col, width=2)
                c.create_line(xs, 15, xs, 23, fill=col, width=2)
                c.create_line(xe, 15, xe, 23, fill=col, width=2)
                name = Path(self.var_image.get()).name
                c.create_text(xs, 6, anchor="w", text=f"{name}  ·  {self.image_total / 1024:.1f} KB",
                              fill=col, font=F["small"])
            for i in range(SECTOR_COUNT):
                x0 = i * (bw + gap)
                x1 = min(x0 + bw, w - 1)
                v = self.sector_view.get(i)
                if v is None:
                    c.create_rectangle(x0, y0, x1, y0 + h, fill=T["sunken"], outline=T["line"])
                    c.create_text(x0 + 2, y0 + h + 13, anchor="w", text=f"S{i}", font=F["badge"], fill=T["muted"])
                    c.create_text(x0 + 26, y0 + h + 13, anchor="w", text="trống", font=F["small"],
                                  fill=_mix(T["muted"], T["bg"], 0.3))
                    continue
                state = v["state"]
                color = T.get(STATE_COLOR.get(state, ""), T["line"])
                c.create_rectangle(x0, y0, x1, y0 + h, fill=S, outline=T["line"])
                full = state in ("done", "match")
                frac = 1.0 if full else (v["written"] / v["total"] if v["total"] else 0)
                if frac > 0:
                    c.create_rectangle(x0 + 1, y0 + 1, x0 + 1 + (x1 - x0 - 2) * frac, y0 + h - 1,
                                       fill=color, width=0)
                if state in ("checking", "erasing", "verifying", "dirty"):
                    c.create_rectangle(x0 + 1, y0 + h - 6, x1 - 1, y0 + h - 1, fill=color, width=0)
                if state in ("checking", "erasing", "writing", "verifying"):
                    c.create_rectangle(x0, y0, x1, y0 + h, outline=color, width=2)
                label = STATE_TEXT[state]
                if state == "writing" and v["total"]:
                    label += f" {v['written'] / v['total'] * 100:.0f}%"
                c.create_text(x0 + 2, y0 + h + 13, anchor="w", text=f"S{i}", font=F["badge"], fill=T["fg"])
                c.create_text(x0 + 26, y0 + h + 13, anchor="w", text=label, font=F["small"],
                              fill=color if state != "pending" else T["muted"])
            label_w = tkfont.Font(font=F["mono_s"]).measure("0x0800 0000") + 12
            for i in (0, 2, 4, 6):
                x = i * (bw + gap)
                if x + label_w < w - label_w:
                    c.create_text(x, y0 + h + 34, anchor="w", text=fmt_addr(FLASH_BASE + i * SECTOR_SIZE),
                                  font=F["mono_s"], fill=T["muted"])
            c.create_text(w - 1, y0 + h + 34, anchor="e", text=fmt_addr(FLASH_END - 1), font=F["mono_s"],
                          fill=T["muted"])

    App()
    root.mainloop()


# --------------------------------------------------------------------------- CLI

def run_cli(settings: Settings, job: str, assume_safe: bool, pings: int) -> int:
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")   # Vietnamese text on a cp1252 console
        except (AttributeError, ValueError):
            pass
    last = {"pct": -1}

    def emit(kind: str, **d) -> None:
        stamp = time.strftime("%H:%M:%S")
        if kind == "log":
            mark = {"ok": "+", "warn": "!", "error": "x", "err": "x"}.get(d["level"], " ")
            print(f"{stamp} {mark} {d['text']}", flush=True)
        elif kind == "ocd" and settings.show_openocd_log:
            print(f"{stamp}     {d['text']}", flush=True)
        elif kind == "progress":
            pct = int(d["done"] / d["total"] * 100)
            if pct != last["pct"]:
                last["pct"] = pct
                rate = f", {d['rate'] / 1024:.1f} KB/s" if d["rate"] else ""
                print(f"{stamp}   [{pct:3d}%] {d['done'] / 1024:.1f}/{d['total'] / 1024:.1f} KB{rate}", flush=True)

    def ask(question: str) -> bool:
        print("\n" + question.replace("\n\n", "\n") + f"\n-> {'có (--assume-safe)' if assume_safe else 'không'}")
        return assume_safe

    flasher = Flasher(emit, ask)
    try:
        if job == "link":
            ok = flasher.run("link_test", settings, pings)
        elif job == "reset":
            ok = flasher.run("reset_run", settings)
        else:
            ok = flasher.run("flash", settings, settings.image)
    except KeyboardInterrupt:
        flasher.stop.set()
        print("interrupted")
        return 130
    finally:
        if flasher.ocd is not None:
            flasher.ocd.close()
    return 0 if ok else 1


def main() -> int:
    ap = argparse.ArgumentParser(description="One-file STM32H723 flasher for flaky (wireless) CMSIS-DAP links.")
    ap.add_argument("image", nargs="?", help="firmware .elf/.hex/.bin")
    ap.add_argument("--cli", action="store_true", help="run on the console instead of the window")
    ap.add_argument("--link-test", nargs="?", type=int, const=20, metavar="N",
                    help="only test the link with N reads (does not halt the core)")
    ap.add_argument("--reset", action="store_true", help="only reset and run the STM32")
    ap.add_argument("--probe", metavar="SERIAL", help="CMSIS-DAP serial number (default: first probe)")
    ap.add_argument("--backend", choices=["hid", "usb_bulk", "auto"])
    ap.add_argument("--speed", type=int, metavar="KHZ")
    ap.add_argument("--no-rotate", action="store_true", help="keep the chosen backend/clock on failures")
    ap.add_argument("--minutes", type=int, help="give up after this many minutes")
    ap.add_argument("--force", action="store_true", help="rewrite sectors that already match")
    ap.add_argument("--no-safety", action="store_true", help="skip the /motor_status check")
    ap.add_argument("--safety-cmd", help="command printing /motor_status as a number")
    ap.add_argument("--assume-safe", action="store_true",
                    help="continue when /motor_status cannot be read (motors known to be off)")
    ap.add_argument("--openocd-log", action="store_true", help="print OpenOCD output")
    args = ap.parse_args()

    s = Settings.load()
    if args.image:
        s.image = str(Path(args.image).resolve())
    for attr, value in (("probe_serial", args.probe), ("backend", args.backend), ("speed_khz", args.speed),
                        ("max_minutes", args.minutes), ("safety_cmd", args.safety_cmd)):
        if value:
            setattr(s, attr, value)
    if args.no_rotate:
        s.rotate = False

    if args.cli or args.link_test is not None or args.reset:
        # The console never inherits the window's relaxed options.
        if not args.image and DEFAULT_IMAGE:
            s.image = DEFAULT_IMAGE
        s.force = args.force
        s.safety = not args.no_safety
        s.show_openocd_log = args.openocd_log
        job = "link" if args.link_test is not None else "reset" if args.reset else "flash"
        try:
            return run_cli(s, job, args.assume_safe, args.link_test or 20)
        except (FileNotFoundError, ValueError) as e:
            print(f"error: {e}", file=sys.stderr)
            return 2
    run_gui(s)
    return 0


if __name__ == "__main__":
    sys.exit(main())
