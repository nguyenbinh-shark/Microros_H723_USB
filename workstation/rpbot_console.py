"""Small desktop launcher for an existing ROS 2 robot and RViz.

Windows runs the PC-side ROS tools in WSL; Linux runs them directly."""
from datetime import datetime
import json
import os
from pathlib import Path
import queue
import re
import shlex
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time
import tkinter as tk
import xml.etree.ElementTree as ET
from tkinter import messagebox, ttk
from tkinter.scrolledtext import ScrolledText
from tkinter import filedialog, simpledialog
import base64
import hashlib
import ipaddress
from console_layout import build as build_layout
from stream_client import TelemetryStream
from connection_state import ConnectionState
from network_probe import ping_ipv4, resolve_ipv4
from map_library import map_name as validate_map_name
from ui_language import Dialogs, Language, LANGUAGES

HERE = Path(__file__).resolve().parent
WINDOWS = os.name == "nt"
STATE = (Path(os.environ.get("LOCALAPPDATA", str(Path.home()))) if WINDOWS else
         Path(os.environ.get("XDG_STATE_HOME") or Path.home() / ".local" / "state")) / "RPBotConsole"
DEFAULTS = {"host": "", "user": "",
            "distro": "Ubuntu-24.04", "wsl_user": "shark"}
HIDDEN = getattr(subprocess, "CREATE_NO_WINDOW", 0)
if getattr(sys, "frozen", False) and not WINDOWS:
    # The Linux build points LD_LIBRARY_PATH at its bundled libraries; ssh, bash
    # and ROS must load the system ones.
    os.environ.pop("LD_LIBRARY_PATH", None)
    if "LD_LIBRARY_PATH_ORIG" in os.environ:
        os.environ["LD_LIBRARY_PATH"] = os.environ.pop("LD_LIBRARY_PATH_ORIG")


def wsl_path(path):
    path = Path(path).resolve().as_posix()
    if not re.match(r"^[A-Za-z]:/", path):
        raise ValueError("Use a local Windows drive path for RPBot Console")
    return "/mnt/" + path[0].lower() + path[2:]


def validate_host(host):
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9.-]{0,252}", host):
        raise ValueError('Nhập tên thiết bị hoặc IPv4 hợp lệ, ví dụ robot.local hoặc 192.168.1.50.')
    return host


def validate_settings(settings):
    validate_host(settings['host'])
    for key in ("user", "wsl_user"):
        if not re.fullmatch(r"[a-z_][a-z0-9_-]{0,31}", settings[key]):
            label = 'SSH' if key == 'user' else 'WSL'
            raise ValueError(f'Nhập tài khoản {label} hợp lệ.')
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]{0,63}", settings["distro"]):
        raise ValueError("Invalid WSL distro name")
    return settings


def host_path(path):
    """A local file as the PC's ROS shell sees it."""
    return wsl_path(path) if WINDOWS else Path(path).resolve().as_posix()


def host_command(settings, *args):
    """Run in the PC's ROS environment: the WSL distro on Windows, directly on Linux."""
    if not WINDOWS:
        return list(args)
    return ["wsl.exe", "-d", settings["distro"], "-u", settings["wsl_user"], "--", *args]


def capture(command, *, input=None, timeout=45):
    result = subprocess.run(command, input=input, capture_output=True, text=True,
                            stdin=subprocess.DEVNULL if input is None else None,
                            encoding="utf-8", errors="replace", timeout=timeout,
                            creationflags=HIDDEN)
    if result.returncode:
        detail = (result.stderr + "\n" + result.stdout).strip()
        raise RuntimeError(detail or f"Command failed ({result.returncode})")
    return result.stdout.strip()


def ssh_command(settings, command, *, address=None):
    host_options = []
    if settings['host'].lower().endswith('.local'):
        # Windows can resolve mDNS even when WSL's resolver cannot. Keep the
        # original hostname for SSH config matching and host-key verification.
        host_options = ['-o', 'HostName=' + (address or resolve_robot_address(settings)),
                        '-o', 'HostKeyAlias=' + settings['host'].lower()]
    # Compression: the JSON telemetry and helper sources shrink ~9x on a 4G uplink.
    return host_command(settings, "ssh", "-T", "-o", "BatchMode=yes", "-o", "Compression=yes",
                        "-o", "ConnectTimeout=5", "-o", "ServerAliveInterval=5",
                        "-o", "ServerAliveCountMax=2",
                        *host_options, settings["user"] + "@" + settings["host"], command)


def remote(settings, action, *args, address=None, timeout=45, payload=None):
    # Only known actions; arguments are POSIX quoted before SSH passes them to a shell.
    command = shlex.join(["python3", "-", action, *args])
    prefix = f"OPS_PAYLOAD = {payload!r}\n" if payload else ""
    return capture(ssh_command(settings, command, address=address),
                   input=prefix + (HERE / "map_library.py").read_text(encoding="utf-8") + "\n" +
                         (HERE / "robot_tasks.py").read_text(encoding="utf-8") + "\n" +
                         (HERE / "remote_ops.py").read_text(encoding="utf-8"), timeout=timeout)


# Installed on the Pi: the 2-minute map autosave timer and the 4G lite stream run them.
OPS_FILES = ("map_library.py", "robot_tasks.py", "remote_ops.py", "ros_session.py", "lite_stream.py", "localize.py")
LITE_OPS = "~/.local/share/rpbot-console/ops/remote_ops.py"


def ops_bundle():
    files = {name: (HERE / name).read_text(encoding="utf-8") for name in OPS_FILES}
    revision = hashlib.sha256("".join(files[name] for name in sorted(files)).encode("utf-8")).hexdigest()
    return base64.b64encode(json.dumps(files).encode("utf-8")).decode("ascii"), revision


def lite_command(settings, domain, *, address=None):
    """Telemetry rendered on the Pi and sent over SSH: no DDS over a cellular uplink."""
    return ssh_command(settings, f"python3 {LITE_OPS} lite-stream --domain {int(domain)}", address=address)


def resolve_robot_address(settings):
    return resolve_ipv4(settings['host'], fallback=lambda host: capture(
        host_command(settings, 'getent', 'ahostsv4', host), timeout=5))


def ping_robot(settings):
    return ping_ipv4(resolve_robot_address(settings))


def session_args(settings, action, session, *, address=None):
    command = host_command(settings, "bash", host_path(HERE / "ros_session.sh"),
                          action, "--domain", str(session["domain"]))
    profile = STATE / "fastdds.xml"
    xml = pc_profile(session.get("profile_xml", ""), address or resolve_robot_address(settings), session["domain"])
    # A second Console/diagnostic process must never observe a half-written XML.
    with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=STATE, delete=False) as file:
        file.write(xml)
        temporary = file.name
    os.replace(temporary, profile)
    command += ["--profile", host_path(profile)]
    return command


def pc_profile(xml, host, domain):
    """Reach the robot's custom ports; listen on standard PC DDS ports."""
    ns = "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"
    ET.register_namespace("", ns)
    tag = lambda name: "{" + ns + "}" + name
    root = ET.fromstring(xml) if xml else ET.Element(tag("profiles"))
    participant = next((p for p in root.findall(tag("participant"))
                        if p.get("is_default_profile") == "true"), None)
    if participant is None:
        participant = ET.SubElement(root, tag("participant"), profile_name="rpbot_pc", is_default_profile="true")
    def child(parent, name):
        found = parent.find(tag(name))
        return found if found is not None else ET.SubElement(parent, tag(name))
    rtps = child(participant, "rtps")
    ports = rtps.find(tag("port"))
    def port_value(key, default):
        item = ports.find(tag(key)) if ports is not None else None
        return int(item.text) if item is not None else default
    base = port_value("portBase", 7400) + port_value("domainIDGain", 250) * domain
    offset, gain = port_value("offsetd1", 10), port_value("participantIDGain", 2)
    peers = child(child(rtps, "builtin"), "initialPeersList")
    address = socket.gethostbyname(host)
    for index in range(32):
        locator = ET.SubElement(peers, tag("locator"))
        udp = ET.SubElement(locator, tag("udpv4"))
        ET.SubElement(udp, tag("address")).text = address
        ET.SubElement(udp, tag("port")).text = str(base + offset + gain * index)
    # The PC already has a scoped firewall rule for standard domain-0 DDS.
    # SPDP advertises these local locators to the peer, so its port base need not match.
    local_ports = child(rtps, "port")
    for key, value in (("portBase", 7400), ("domainIDGain", 250),
                       ("participantIDGain", 2), ("offsetd0", 0),
                       ("offsetd1", 10), ("offsetd2", 1), ("offsetd3", 11)):
        child(local_ports, key).text = str(value)
    if ipaddress.IPv4Address(address) in VPN_NETWORK:
        # Over a VPN (e.g. Tailscale via 4G) the robot sends every sample to each
        # locator the PC advertises; unreachable LAN/WSL addresses then waste the
        # robot's uplink. Advertise only the PC's VPN address.
        transports = root.find(tag("transport_descriptors"))
        if transports is None:
            transports = ET.Element(tag("transport_descriptors"))
            root.insert(0, transports)
        descriptor = ET.SubElement(transports, tag("transport_descriptor"))
        ET.SubElement(descriptor, tag("transport_id")).text = "rpbot_vpn_udp"
        ET.SubElement(descriptor, tag("type")).text = "UDPv4"
        allowed = ET.SubElement(descriptor, tag("interfaceWhiteList"))
        ET.SubElement(allowed, tag("address")).text = local_address_for(address)
        user = rtps.find(tag("userTransports"))
        if user is not None:
            rtps.remove(user)
        ET.SubElement(ET.SubElement(rtps, tag("userTransports")), tag("transport_id")).text = "rpbot_vpn_udp"
        child(rtps, "useBuiltinTransports").text = "false"
    return ET.tostring(root, encoding="unicode")


# Tailscale and NetBird hand out addresses from the carrier-grade NAT range.
VPN_NETWORK = ipaddress.IPv4Network("100.64.0.0/10")


def local_address_for(address):
    """The local IPv4 address the OS routes to `address` (no packet is sent)."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
        probe.connect((address, 9))
        return probe.getsockname()[0]


def inspect_robot(settings):
    # Resolve .local in the metadata worker. Tk later reuses this address when
    # opening/recovering DDS, instead of doing mDNS/WSL lookups on its event loop.
    address = resolve_robot_address(settings) if settings['host'].lower().endswith('.local') else None
    robot = json.loads(remote(settings, "status", address=address))
    payload, revision = ops_bundle()
    if robot.get("ops_revision") != revision:
        # Keeps the Pi's autosave timer and lite stream in step with this Studio version.
        robot["ops_installed"] = json.loads(remote(settings, "install-ops", address=address, payload=payload))
        robot["ops_revision"] = revision
    sessions = [item for item in robot["launches"]
                if item["mode"] in ("mapping", "navigation") and not item["simulated_time"]]
    if len(sessions) != 1:
        note = "Robot chưa chạy Mapping hoặc Navigation. Bấm Bắt đầu mapping để tạo map, hoặc chọn map đã lưu rồi Bắt đầu Nav."
        if sessions:
            note = "Có nhiều phiên robot: " + "; ".join(
                f"{item['mode']} (PID {item['pid']}, domain {item['domain']})" for item in sessions)
        return {"robot": robot, "session": None, "data": None,
                "note": note, "address": address}
    session = sessions[0]
    return {"robot": robot, "session": session, "data": None, "address": address,
            "lite": (robot.get("uplink") or {}).get("kind") == "cellular"}


def check_connection(settings):
    report = inspect_robot(settings)
    if report["session"]:
        report["data"] = json.loads(capture(session_args(settings, "probe", report["session"],
                                                       address=report.get('address')), timeout=25))
    return report


def write_download(payload, parent):
    name = validate_map_name(payload["name"])
    files = []
    for key, suffix in (("yaml", ".yaml"), ("image", ".pgm")):
        raw = base64.b64decode(payload[key + "_b64"], validate=True)
        if hashlib.sha256(raw).hexdigest() != payload[key + "_sha256"]:
            raise ValueError("Dữ liệu tải về không khớp checksum; chưa ghi file")
        files.append((name + suffix, raw))
    target = Path(parent).resolve() / name
    target.mkdir(exist_ok=False)
    written = []
    try:
        for filename, raw in files:
            path = target / filename
            with path.open("xb") as file:
                written.append(path)
                file.write(raw)
    except Exception:
        for path in written:
            path.unlink()
        target.rmdir()
        raise
    return target


SYNC_DIR = Path.home() / "Documents" / "RPBot maps"
SYNC_PERIOD_S = 120


def download_changed_maps(settings, directory):
    """Copy each Pi map whose revision differs from the PC's last copy. Returns what was written."""
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    index_path = directory / ".revisions.json"
    try:
        index = json.loads(index_path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        index = {}
    written = []
    for record in json.loads(remote(settings, "maps"))["maps"]:
        name = record["name"]
        if "error" in record or index.get(name) == record["revision"]:
            continue
        staging = Path(tempfile.mkdtemp(dir=directory, prefix=".sync-"))
        try:
            payload = json.loads(remote(settings, "map-export", "--name", name,
                                        "--revision", record["revision"], timeout=120))
            write_download(payload, staging)
            target = directory / validate_map_name(name)
            if target.exists():
                shutil.rmtree(target)
            os.replace(staging / name, target)
        except Exception as exc:
            # An autosave may replace the map mid-download; the next sync picks it up.
            written.append(f"{name} (chưa tải: {exc})")
            continue
        finally:
            shutil.rmtree(staging, ignore_errors=True)
        index[name] = record["revision"]
        index_path.write_text(json.dumps(index, indent=2), encoding="utf-8")
        written.append(name)
    return written


class Console:
    # Defaults for the 4G lite mode and map sync (tests build Console without __init__).
    lite = False
    live_window = None
    syncing = False
    last_sync = -SYNC_PERIOD_S

    def __init__(self, root):
        self.root = root
        STATE.mkdir(parents=True, exist_ok=True)
        self.settings = dict(DEFAULTS)
        saved = {}
        config = STATE / "settings.json"
        if config.is_file():
            try:
                saved = json.loads(config.read_text(encoding="utf-8"))
                if not isinstance(saved, dict):
                    saved = {}
                self.settings.update({key: value for key, value in saved.items() if key in DEFAULTS})
            except (ValueError, OSError):
                pass
        self.i18n = Language(saved.get('language', 'vi'))
        self.dialogs = Dialogs(self.i18n, messagebox)
        self.file_dialogs = Dialogs(self.i18n, filedialog)
        self.input_dialogs = Dialogs(self.i18n, simpledialog)
        self.events = queue.Queue()
        self.busy = False
        self.session = None
        self.connected_settings = None
        self.robot_address = None
        self.last_data = None
        self.last_check = 0.0
        self.viewers = {}
        self.actions = []
        self.stop_buttons = []
        self.stop_busy = False
        self.task_records = {}
        self.connecting = False
        self.pinging = False
        self.last_meta = 0.0
        self.connection = ConnectionState()
        self.requested_settings = None
        self.pending_session = None
        self.stream = None
        self.stream_key = None
        self.stream_seen = 0
        self.map_records = {}
        self.preview_record = None
        self.preview_generation = 0
        self.closed = False
        root.report_callback_exception = lambda kind, error, trace: self.append("Lỗi thao tác: " + str(error))
        build_layout(self)
        self.i18n.attach(root)
        for variable in (self.activity, self.page_title, self.page_subtitle, self.page_number,
                         self.selected_map_text, self.floor_current, self.map_info, self.ping_status,
                         self.dashboard.linear, self.dashboard.angular, *self.status.values()):
            self.i18n.variable(variable)
        self.i18n.tree(self.tasks_tree, text=True, columns=(2,))
        self.i18n.tree(self.topic_tree, columns=(2,))
        self.i18n.tree(self.maps_tree, columns=(0,))
        self.i18n.tree(self.floor_tree, columns=(3,))
        if not self.settings['host'] or not self.settings['user']:
            self.tabs.select(self.tab_frames['settings'])
            self.activity.set('Nhập thiết bị và tài khoản SSH trong Kết nối')
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.after(150, self.drain)
        root.after(600, lambda: self.check(quiet=True) if self.requested_settings is None else None)
        root.after(100, self.tick)

    @staticmethod
    def new_name():
        return datetime.now().strftime("room_%Y%m%d_%H%M%S")

    def button(self, parent, text, action):
        button = ttk.Button(parent, text=text, command=action)
        self.actions.append(button)
        return button

    def stop_button(self, parent, text, action):
        button = ttk.Button(parent, text=text, command=action)
        self.stop_buttons.append(button)
        return button

    def current_settings(self):
        return validate_settings({key: var.get().strip() for key, var in self.fields.items()})

    def current_ping_settings(self):
        # ICMP needs a host, not an SSH login. Allow checking a new device before
        # its account is filled in; compare the same snapshot on completion.
        settings = {key: var.get().strip() for key, var in self.fields.items()}
        validate_host(settings['host'])
        return settings

    def config(self):
        settings = self.current_settings()
        if settings != self.settings:
            self.save_settings(settings)
            self.settings = dict(settings)
        return settings

    def save_settings(self, settings):
        (STATE / 'settings.json').write_text(json.dumps(dict(settings, language=self.i18n.code),
                                                       indent=2, ensure_ascii=False), encoding='utf-8')

    def change_language(self, code):
        self.i18n.select(code)
        self.language_picker.variable.set(LANGUAGES[self.i18n.code])
        for card in self.dashboard.cards.values():
            card.fit_detail()
        try:
            self.save_settings(self.settings)
        except OSError as exc:
            self.dialogs.showerror('Không lưu được ngôn ngữ', str(exc), parent=self.root)

    def append(self, text):
        line = f"[{datetime.now():%H:%M:%S}] {self.i18n.text(text)}\n"
        message = text.strip().casefold()
        if message.startswith(('lỗi', 'chưa dừng hết')):
            tag = 'error'
        elif message.startswith(('chưa kết nối', 'mất ', 'chưa xác nhận', 'đang tự phục hồi')):
            tag = 'warning'
        elif message.startswith(('đã kết nối', 'đã phục hồi', 'đã lưu', 'đã dừng ')):
            tag = 'success'
        else:
            tag = ''
        self.log.configure(state="normal")
        self.log.insert("end", line, (tag,) if tag else ())
        self.log.see("end")
        self.log.configure(state="disabled")
        with (STATE / "console.log").open("a", encoding="utf-8") as file:
            file.write(line)

    def task(self, title, work, done=None):
        if self.busy or self.stop_busy:
            return
        self.busy = True
        for button in self.actions:
            button.state(["disabled"])
        self.append(title)
        self.activity.set(title)

        def worker():
            try:
                result = work()
                self.events.put(("task", True, result, done))
            except Exception as exc:
                self.events.put(("task", False, str(exc), done))
        threading.Thread(target=worker, daemon=True).start()

    def control_task(self, title, work, done=None):
        if self.stop_busy:
            return
        self.stop_busy = True
        for button in self.actions + self.stop_buttons:
            button.state(["disabled"])
        self.activity.set(title)
        self.append(title)
        def worker():
            try:
                self.events.put(("stop", True, work(), done))
            except Exception as exc:
                self.events.put(("stop", False, str(exc), done))
        threading.Thread(target=worker, daemon=True).start()

    def drain(self):
        try:
            while True:
                kind, ok, result, done = self.events.get_nowait()
                if kind == "task":
                    self.busy = False
                    if not self.stop_busy:
                        for button in self.actions:
                            button.state(["!disabled"])
                        self.activity.set("Sẵn sàng" if ok else "Thao tác chưa hoàn tất")
                elif kind == "stop":
                    self.stop_busy = False
                    for button in self.stop_buttons + ([] if self.busy else self.actions):
                        button.state(["!disabled"])
                    self.activity.set("Đã kiểm tra tác vụ" if ok else "Chưa xác nhận dừng tác vụ")
                elif kind == "metadata":
                    self.connecting = False
                    settings, payload = result
                    try:
                        current = self.current_settings()
                    except ValueError:
                        current = None
                    if settings != current:
                        # A slow reply from the previous address must not replace the new robot.
                        self.connection.next_check = 0
                        if self.pending_session and self.pending_session[0] != current:
                            self.cancel_pending("Kết nối đã đổi; thao tác chưa được thực hiện.")
                        continue
                    if ok:
                        self.checked(result)
                    else:
                        self.metadata_failed(payload)
                    continue
                elif kind == "sync":
                    self.syncing = False
                    if ok and result:
                        self.append("Đã đồng bộ map về PC (" + str(SYNC_DIR) + "): " + ", ".join(result))
                    elif not ok:
                        self.append("Chưa đồng bộ được map về PC; sẽ thử lại: " + result)
                    continue
                elif kind == "ping":
                    self.pinging = False
                    self.ping_button.state(['!disabled'])
                    settings, payload = result
                    try:
                        current = self.current_ping_settings()
                    except ValueError:
                        current = None
                    if settings != current:
                        self.set_ping_status('Cấu hình đã đổi. Bấm Ping thiết bị để kiểm tra địa chỉ mới.', 'unknown')
                        continue
                    if ok:
                        self.set_ping_status(payload.summary, payload.level)
                        self.append(f"Ping {settings['host']}: {payload.summary}")
                        if payload.output:
                            self.append(payload.output)
                    else:
                        self.set_ping_status(str(payload), 'error')
                        self.append(f"Lỗi ping {settings['host']}: {payload}")
                    continue
                if ok:
                    if done:
                        done(result)
                    elif result:
                        self.append(str(result))
                else:
                    self.append("Lỗi: " + result)
                    if kind == "preview" and done:
                        done(result)
                    if self.session is None:
                        self.status["mapping"].set("Chưa xác nhận kết quả thao tác — đang tự cập nhật")
                        self.connection.next_check = 0
        except queue.Empty:
            pass
        if not self.closed:
            self.root.after(150, self.drain)

    def check(self, *, quiet=False):
        if self.connecting:
            return
        try:
            settings = self.config()
        except ValueError as exc:
            if not quiet:
                self.dialogs.showerror("Cấu hình kết nối", str(exc))
            return
        self.connecting = True
        self.requested_settings = dict(settings)
        if not quiet and not self.busy and not self.stop_busy:
            self.activity.set("Đang kiểm tra kết nối…")
        def worker():
            try:
                self.events.put(("metadata", True, (settings, inspect_robot(settings)), self.checked))
            except Exception as exc:
                self.events.put(("metadata", False, (settings, str(exc)), None))
        threading.Thread(target=worker, daemon=True).start()

    def set_ping_status(self, text, level):
        from status_dashboard import COLORS
        background, foreground, _ = COLORS[level]
        self.ping_status.set(text)
        self.ping_label.configure(background=background, foreground=foreground)

    def invalidate_ping(self, *_):
        self.set_ping_status('Cấu hình đã đổi. Bấm Ping thiết bị để kiểm tra địa chỉ mới.', 'unknown')

    def ping_device(self):
        if self.pinging:
            return
        try:
            settings = self.current_ping_settings()
        except ValueError as exc:
            self.set_ping_status(str(exc), 'error')
            return
        self.pinging = True
        self.ping_button.state(['disabled'])
        self.set_ping_status(f"Đang tìm và ping {settings['host']}…", 'info')
        def worker():
            try:
                self.events.put(('ping', True, (settings, ping_robot(settings)), None))
            except Exception as exc:
                self.events.put(('ping', False, (settings, str(exc)), None))
        threading.Thread(target=worker, daemon=True).start()

    def telemetry_fresh(self):
        if not self.stream or not self.stream.alive():
            return False
        _, updated, frames = self.stream.snapshot()
        return bool(frames and updated and time.monotonic() - updated < 2)

    def metadata_failed(self, error):
        if self.connection.metadata_failed(time.monotonic(), error):
            self.append("Chưa kết nối được SSH; Console sẽ tự thử lại. Chi tiết: " + error)
        self.cancel_pending("Chưa xác nhận được kết nối; thao tác chưa thực hiện. Khi mạng trở lại, bấm lại nút thao tác.")
        if not self.telemetry_fresh():
            self.status['connection'].set('Chưa kết nối được Pi — đang tự kết nối lại')
            if not self.busy and not self.stop_busy:
                self.activity.set('Đang chờ robot / mạng trở lại')
        elif not self.busy and not self.stop_busy:
            self.activity.set('Đang nhận ROS · SSH đang tự nối lại')

    def checked(self, result):
        settings, report = result
        robot, data = report["robot"], report["data"]
        previous = self.session_identity(self.session)
        self.connected_settings = settings
        self.robot_address = report.get('address')
        self.session = report["session"]
        self.last_meta = time.monotonic()
        self.sync_maps(settings, robot)
        recovered = self.connection.metadata_ok(self.last_meta, has_session=self.session is not None)
        if recovered:
            self.append('Đã kết nối lại SSH tới robot.')
        fresh = self.session is not None and previous == self.session_identity(self.session) and self.telemetry_fresh()
        if not fresh:
            self.status["connection"].set(f"SSH: {robot['hostname']} ({settings['host']})")
        if not self.busy and not self.stop_busy and (not fresh or recovered):
            self.activity.set('Đã kết nối robot' if self.session else 'Đã kết nối Pi · chưa có phiên robot')
        service = robot["service"]
        if service.get("UnitFileState") == "enabled":
            self.status["boot"].set("Đã bật · dịch vụ đang chạy" if service.get("ActiveState") == "active"
                                    else "Đã bật cho lần boot sau · dịch vụ chưa chạy")
        else:
            self.status["boot"].set("Chưa bật tự khởi động")
        if service.get("UnitFileState") == "enabled" and service.get("ActiveState") != "active" and self.session:
            self.status["boot"].set("Đã bật cho lần boot sau · phiên hiện tại mở bên ngoài Console")
        if self.session is None:
            self.stop_stream()
            self.last_data = None
            self.status["mapping"].set(report.get("note", "Chưa có phiên mapping thật"))
            self.status["sensors"].set("Chưa kiểm tra")
            self.status["motion"].set("Chưa kiểm tra")
            self.status["navigation"].set("Chưa kiểm tra")
            self.resume_pending()
            return
        lite = bool(report.get("lite"))
        if lite != self.lite:
            self.append("Robot đang dùng 4G: chế độ tiết kiệm — Pi gửi ảnh bản đồ và trạng thái qua SSH, không gửi DDS."
                        if lite else "Robot đã về Wi‑Fi/LAN: nhận ROS trực tiếp qua DDS.")
        self.lite = lite
        key = (tuple(sorted(settings.items())), self.session["pid"], self.session["domain"], self.session.get("profile_xml"), lite)
        if self.robot_address:
            key += (self.robot_address,)
        changed = self.stream_key != key
        if changed:
            self.connection.reset_stream()
            self.last_data = None
            self.last_check = 0
            for name in ("mapping", "sensors", "motion", "navigation"):
                self.status[name].set("Đang chờ dữ liệu ROS mới…")
            self.topic_tree.delete(*self.topic_tree.get_children())
            self.task_domain.set(self.session['domain'])
        _, updated, _ = self.stream.snapshot() if self.stream else (None, 0, 0)
        alive = bool(self.stream and self.stream.alive())
        if changed or self.connection.restart_due(time.monotonic(), alive=alive, updated=updated):
            self.start_stream(settings, key)
        if changed:
            self.append("Đã kết nối phiên " + self.session.get("mode", "mapping").capitalize() + " — đang nhận trạng thái ROS liên tục.")
        if data:
            self.render_data(data)
        report["data"] = self.last_data
        (STATE / "last-diagnostics.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        if getattr(self, "map_settings", None) != settings and not self.busy:
            self.root.after(100, lambda: self.refresh_maps(background=True))
        self.resume_pending()

    def sync_maps(self, settings, robot):
        """On Wi-Fi/LAN only, copy maps the PC lacks or that changed (e.g. autosaves made on 4G)."""
        if ((robot.get("uplink") or {}).get("kind") not in ("wifi", "lan") or self.syncing
                or time.monotonic() - self.last_sync < SYNC_PERIOD_S):
            return
        self.syncing, self.last_sync = True, time.monotonic()
        def worker():
            try:
                self.events.put(("sync", True, download_changed_maps(settings, SYNC_DIR), None))
            except Exception as exc:
                self.events.put(("sync", False, str(exc), None))
        threading.Thread(target=worker, daemon=True).start()

    def start_stream(self, settings, key):
        self.stop_stream()
        self.stream_key, self.stream_seen = key, 0
        self.connection.stream_attempt(time.monotonic())
        try:
            command = (lite_command(settings, self.session["domain"], address=self.robot_address) if self.lite
                       else session_args(settings, "stream", self.session, address=self.robot_address))
            self.stream = TelemetryStream(command, STATE / "telemetry.log", creationflags=HIDDEN)
        except Exception as exc:
            if not self.connection.stream_recovering:
                self.append('Chưa mở được luồng ROS; Console sẽ tự thử lại: ' + str(exc))
            self.connection.stream_recovering = True

    @staticmethod
    def session_identity(session):
        return (session['pid'], session['domain'], session['mode']) if session else None

    def cancel_pending(self, reason):
        if self.pending_session is not None:
            self.pending_session = None
            self.append(reason)

    def resume_pending(self):
        pending, self.pending_session = self.pending_session, None
        if pending is None:
            return
        settings, expected, deadline, callback = pending
        if settings != self.connected_settings:
            self.append('Kết nối đã đổi; thao tác chưa được thực hiện.')
            return
        if time.monotonic() > deadline or self.busy or self.stop_busy:
            self.append('Thao tác chưa thực hiện. Bạn có thể bấm lại nút thao tác khi sẵn sàng.')
            return
        if not self.session:
            self.dialogs.showinfo('Phiên robot', 'Pi đã kết nối nhưng chưa có đúng một phiên Mapping/Navigation. Xem trạng thái hoặc bắt đầu phiên cần dùng.')
            return
        if expected is not None and expected != self.session_identity(self.session):
            self.append('Phiên robot đã đổi; thao tác chưa thực hiện. Kiểm tra phiên hiện tại rồi bấm lại nút thao tác.')
            return
        confirmed = self.session_identity(self.session)
        def resume():
            if (not self.closed and time.monotonic() <= deadline and not self.busy and not self.stop_busy
                    and settings == self.current_settings() and confirmed == self.session_identity(self.session)):
                callback()
        self.root.after(0, resume)

    def render_data(self, data):
        self.last_data = data
        self.last_check = time.monotonic()
        domain = self.session["domain"]
        navigating = self.session.get("mode") == "navigation"
        receiving = any(topic["age_s"] is not None and topic["age_s"] < 2 for topic in data["topics"].values())
        self.status["connection"].set(("ROS: đang nhận trực tiếp" if receiving else "ROS: chưa nhận dữ liệu mới")
                                      + f" · Pi {self.connected_settings['host']}" + (" · SSH đang tự nối lại" if self.connection.failures else ""))
        if data["mapping_ready"]:
            info = data["map"]
            mode = "Navigation · map đã lưu" if navigating else "Mapping · map đang dựng"
            self.status["mapping"].set(f"{mode} · domain {domain} · {info['width']}×{info['height']} · {info['resolution']:.3f} m/ô · TF OK")
        else:
            mode = "Navigation" if navigating else "Mapping"
            self.status["mapping"].set(f"{mode} · domain {domain} · Thiếu map, TF hoặc dữ liệu scan/odom mới")
        fresh = lambda topic: (data["topics"][topic]["age_s"] is not None and data["topics"][topic]["age_s"] < 2)
        labels = (("/scan", "LiDAR"), ("/odom", "Odom"), ("/imu/data", "IMU"), ("/motor_fb", "Motor"))
        self.status["sensors"].set(" · ".join(label + (": OK" if fresh(topic) else ": chưa có dữ liệu mới") for topic, label in labels))
        motor = data.get("motor_status") if fresh("/motor_status") else None
        velocity = data.get("velocity") if fresh("/odom") else None
        motion = []
        if motor is not None:
            motion.append("Motor bật" if motor & 0x10 else "Motor tắt")
            if motor & 0x0c:
                motion.append("CAN bus-off")
            if motor & 0x03 != 0x03:
                motion.append("Thiếu phản hồi bánh xe")
            if motor & 0x20:
                motion.append("Lệnh ROS đã timeout")
        if velocity:
            motion.append(f"v={velocity['linear']:+.3f} m/s, ω={velocity['angular']:+.3f} rad/s")
        self.status["motion"].set(" · ".join(motion) or "Chưa có dữ liệu mới")
        self.status["navigation"].set("Đã phát hiện action server — kiểm tra trạng thái Nav2 active trong RViz" if data["navigation_available"]
                                      else "Navigation đang khởi động/chưa có action server" if navigating
                                      else "Chưa có action server Nav2 trong phiên hiện tại")
        for topic, info in data["topics"].items():
            age = info["age_s"]
            values = (info["messages"], "—" if age is None else f"{age:.2f}",
                      "Đang nhận" if age is not None and age < 2 else "Chưa có dữ liệu mới")
            if self.topic_tree.exists(topic):
                self.topic_tree.item(topic, values=values)
            else:
                self.topic_tree.insert("", "end", iid=topic, text=topic, values=values)

    def require_session(self, resume=None, *, allow_telemetry=False):
        settings = self.config()
        same_robot = settings == self.connected_settings
        if (self.session and same_robot and
                ((not self.connection.failures and time.monotonic() - self.last_meta <= 30)
                 or (allow_telemetry and self.telemetry_fresh()))):
            return settings
        if resume is not None and self.pending_session is None:
            expected = self.session_identity(self.session) if same_robot else None
            self.pending_session = (settings, expected, time.monotonic() + 20, resume)
            self.append('Đang tự xác nhận kết nối để thực hiện thao tác…')
        self.check(quiet=True)
        return None

    def open_view(self, action):
        settings = self.require_session(lambda: self.open_view(action), allow_telemetry=True)
        if not settings:
            return
        if (action == "navigation" and self.session.get("mode") != "navigation"
                and not (self.last_data or {}).get("navigation_available")):
            self.dialogs.showinfo("Navigation", "Chưa nhận được action server Nav2 trong phiên hiện tại. Kiểm tra chế độ và trạng thái robot trước.")
            return
        if self.lite and action == "rviz":
            # RViz would pull the full map, scans and TF over DDS; on 4G show the Pi-rendered map instead.
            if self.live_window:
                self.live_window.raise_()
                return
            from live_map_view import LiveMapWindow
            def closed():
                self.live_window = None
            self.live_window = LiveMapWindow(self.root, on_close=closed)
            if self.stream:
                data = self.stream.snapshot()[0] or {}
                self.live_window.update(self.stream.live()[0], data.get("pose"))
            self.append("Robot đang dùng 4G: mở bản đồ nhẹ do Pi vẽ (ảnh mỗi ~5 s, vị trí mỗi 1 s) thay cho RViz.")
            return
        if action in self.viewers and self.viewers[action].poll() is None:
            self.dialogs.showinfo("RViz", "Cửa sổ RViz này đang mở.")
            return
        log = STATE / (action + ".log")
        with log.open("w", encoding="utf-8") as file:
            self.viewers[action] = subprocess.Popen(session_args(settings, action, self.session),
                                                    stdin=subprocess.DEVNULL, stdout=file, stderr=subprocess.STDOUT,
                                                    creationflags=HIDDEN)
        self.append(f"Đang mở RViz ({action}). Log: {log}")

    def save_map(self):
        settings = self.require_session(self.save_map)
        if not settings:
            return
        if self.session.get("mode") == "navigation":
            self.dialogs.showinfo("Lưu map", "Robot đang Navigation với map đã lưu. Xem map trong tab Bản đồ đã lưu; tạo map mới cần chuyển sang Mapping.")
            return
        name = self.map_name.get().strip()
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", name):
            self.dialogs.showerror("Tên map", "Dùng 1–64 chữ không dấu, số, dấu - hoặc _.")
            return
        domain = str(self.session["domain"])

        with_3d = self.save_with_3d.get()

        def saved(result):
            report = json.loads(result)
            self.append("Đã lưu: " + report["saved"])
            if report.get("map3d"):
                self.append("Đã lưu map 3D cùng tên: " + report["map3d"] + " (camera SLAM đã dừng)")
            elif report.get("map3d_error"):
                self.append("Chưa lưu được map 3D: " + report["map3d_error"])
            self.map_name.set(self.new_name())
            self.root.after(100, self.refresh_maps)
        args = ["--name", name, "--domain", domain] + (["--with-3d"] if with_3d else [])
        self.task("Lưu bản đồ hiện tại trên Pi…", lambda: remote(settings, "save", *args, timeout=180), saved)

    def show_logs(self):
        settings = self.config()
        self.task("Đọc log robot…", lambda: remote(settings, "logs"))

    def reset_map(self):
        settings = self.require_session(self.reset_map)
        if not settings:
            return
        if self.session.get("mode") == "navigation" or (self.last_data or {}).get("navigation_available"):
            self.dialogs.showinfo("Tạo map lại", "Hãy dừng chế độ navigation và chuyển sang mapping trước khi tạo map mới.")
            return
        if not self.dialogs.askyesno(
                "Tạo map lại từ đầu?",
                "Map hiện tại trong bộ nhớ sẽ bị bỏ và robot bắt đầu dựng map mới.\n\n"
                "Các file map đã lưu trên Pi vẫn được giữ. Nếu muốn giữ map hiện tại, hãy lưu map trước khi tiếp tục.\n\n"
                "Tiếp tục tạo map mới?", default="no", icon="warning"):
            return
        session = dict(self.session)
        self.session = self.last_data = None
        self.last_check = 0
        self.status["mapping"].set("Đang yêu cầu tạo map mới…")

        def reset_done(result):
            report = json.loads(result)
            if report.get("reset") != "success":
                raise RuntimeError("Chưa xác nhận tạo map mới — kiểm tra lại dữ liệu.")
            self.map_name.set(self.new_name())
            self.append("SLAM đã tạo map lại từ đầu. Các map đã lưu vẫn được giữ.")
            self.root.after(2000, self.check)
        self.task("Yêu cầu SLAM tạo map lại…", lambda: remote(
            settings, "reset", "--domain", str(session["domain"]), "--pid", str(session["pid"])), reset_done)

    def service(self, action):
        settings = self.config()
        self.session = self.last_data = None
        self.last_check = 0
        for key in ("mapping", "sensors", "motion", "navigation"):
            self.status[key].set("Đang thay đổi phiên — cần kiểm tra lại")
        def finished(result):
            report = json.loads(result)
            existing = report.get("status") == "already_running"
            navigation = report.get("status") == "different_mode" and report.get("mode") == "navigation"
            if navigation:
                self.append("Robot đang Navigation. Đã kết nối vào phiên hiện có; chưa chuyển sang Mapping. Dừng phiên Navigation trước khi tạo map mới.")
            elif existing:
                self.append("Mapping đang chạy — kết nối vào phiên hiện có. Muốn dựng map từ đầu, bấm Tạo map lại.")
            else:
                self.append("Đã gửi yêu cầu bắt đầu mapping." if action == "start" else "Đã dừng phiên mapping của Console.")
            self.root.after(0 if existing or navigation else 2000, self.check)
        self.task("Kiểm tra / bắt đầu mapping…" if action == "start" else "Đang dừng mapping…",
                  lambda: remote(settings, action), finished)

    def start_navigation(self):
        if self.busy or self.stop_busy:
            return
        selected = self.selected_record()
        if not selected:
            self.tabs.select(self.tab_frames['maps'])
            return
        settings, record = selected
        domain = self.task_domain_value()
        def finished(result):
            report = json.loads(result)
            existing = report.get('status') == 'already_running'
            self.session = self.last_data = None
            self.last_check = 0
            self.stop_stream()
            self.status['navigation'].set('Đang kiểm tra Nav2…')
            self.append(("Nav đang chạy với map " if existing else "Đã gửi yêu cầu bắt đầu Nav với map ")
                        + record['name'] + f" · domain {domain}. Mở RViz Navigation để đặt vị trí robot.")
            self.root.after(0 if existing else 2000, self.check)
        self.task("Đang kiểm tra / bắt đầu Nav…",
                  lambda: remote(settings, 'start-nav', '--name', record['name'],
                                 '--revision', record['revision'], '--domain', str(domain)), finished)

    # Floors: one saved map per floor, switched at the elevator (pi5-stm32-slam scripts/floors.py).
    def floor_level(self):
        try:
            level = int(self.floor_level_var.get())
        except (tk.TclError, ValueError):
            level = None
        if level is None or not -9 <= level <= 99:
            self.dialogs.showerror("Tầng", "Số tầng phải từ -9 đến 99.")
            return None
        return level

    def show_floors(self, result):
        report = json.loads(result)
        floors = report.get("floors", [])
        self.floor_records = {str(item["level"]): item for item in floors}
        selected = self.floor_tree.selection()
        self.floor_tree.delete(*self.floor_tree.get_children())
        current = report.get("current")
        for item in floors:
            state = ["Map ✓" if item.get("map_exists") else "Thiếu map"]
            if item.get("map3d_exists"):
                state.append("3D ✓")
            state.append("Thang máy ✓" if item.get("elevator") else "Chưa có thang máy")
            level = str(item["level"])
            self.floor_tree.insert("", "end", iid=level, values=(
                ("▶ " if item["level"] == current else "") + level, item.get("label", ""),
                item.get("map", "—"), " · ".join(state)))
        keep = [iid for iid in selected if iid in self.floor_records]
        if keep or floors:
            self.floor_tree.selection_set(keep or [str(floors[0]["level"])])
        self.floor_current.set("Tầng hiện tại: " + (str(current) if current is not None else "chưa chọn")
                               + f" · {len(floors)} tầng đã đăng ký")

    def selected_floor(self):
        selected = self.floor_tree.selection()
        record = self.floor_records.get(selected[0]) if selected else None
        if not record:
            self.dialogs.showinfo("Tầng", "Tải danh sách tầng rồi chọn một tầng.")
        return record

    def refresh_floors(self):
        settings = self.config()
        self.task("Đọc danh sách tầng trên Pi…",
                  lambda: remote(settings, "floors", "--domain", str(self.task_domain_value())), self.show_floors)

    def register_floor(self):
        level = self.floor_level()
        if level is None:
            return
        selected = self.selected_record()
        if not selected:
            self.tabs.select(self.tab_frames['maps'])
            return
        settings, record = selected
        label = self.floor_label.get().strip() or f"Tầng {level}"
        # The 3D map uses the LiDAR map's name (Lưu map with "kèm map 3D").
        self.task(f"Gán map {record['name']} cho tầng {level}…",
                  lambda: remote(settings, "floor-register", "--level", str(level), "--name", record['name'],
                                 "--map3d", record['name'], "--label", label,
                                 "--domain", str(self.task_domain_value())), self.show_floors)

    def mark_elevator(self):
        level = self.floor_level()
        if level is None:
            return
        settings = self.config()
        def marked(result):
            self.append(f"Đã đánh dấu cửa thang máy tầng {level}: " + str(json.loads(result).get("elevator")))
            self.show_floors(result)
        self.task(f"Đánh dấu cửa thang máy tầng {level} tại vị trí robot…",
                  lambda: remote(settings, "floor-mark", "--level", str(level),
                                 "--domain", str(self.task_domain_value())), marked)

    def switch_floor(self):
        record = self.selected_floor()
        if not record:
            return
        if not self.dialogs.askyesno("Chuyển tầng",
                                     f"Robot đang đứng ở cửa thang máy tầng {record['level']} và đã dừng hẳn?\n"
                                     "Nav2 sẽ nạp map tầng này và đặt lại vị trí robot tại cửa thang máy."):
            return
        settings = self.config()
        def switched(result):
            report = json.loads(result)
            self.append(f"Đã chuyển sang tầng {report['level']} (map {report['map']})"
                        + (f", map 3D {report['map3d']}" if report.get("map3d") else ""))
            for warning in report.get("warnings", []):
                self.append("Cảnh báo: " + warning)
            self.show_floors(result)
        self.task(f"Chuyển sang tầng {record['level']}…",
                  lambda: remote(settings, "floor-switch", "--level", str(record['level']),
                                 "--domain", str(self.task_domain_value()), timeout=180), switched)

    def tick(self):
        now = time.monotonic()
        if self.stream and self.session:
            data, updated, frames = self.stream.snapshot()
            if frames != self.stream_seen:
                self.stream_seen = frames
                if self.connection.stream_received():
                    self.append('Đã phục hồi luồng trạng thái ROS trên PC.')
                self.render_data(data)
                if self.live_window and data.get("lite"):
                    self.live_window.update(self.stream.live()[0], data.get("pose"))
            if (not self.stream.alive()) or (updated and now - updated > 2):
                self.last_data = None
                self.status["connection"].set("Mất luồng trạng thái ROS — đang chờ kết nối lại")
                for key in ("mapping", "sensors", "motion", "navigation"):
                    self.status[key].set("Dữ liệu đã cũ")
                if hasattr(self, "topic_tree"):
                    for topic in self.topic_tree.get_children():
                        values = self.topic_tree.item(topic, "values")
                        self.topic_tree.item(topic, values=(values[0], "—", "Luồng đã dừng"))
        if self.last_check and now - self.last_check > 2:
            self.last_check = 0
        for action, process in list(self.viewers.items()):
            code = process.poll()
            if code is not None:
                self.append(f"RViz {action} đã đóng (mã {code})." + (f" Xem {STATE / (action + '.log')}" if code else ""))
                del self.viewers[action]
        if self.auto.get():
            try:
                settings = self.current_settings()
            except ValueError:
                settings = None
            if settings is not None:
                if self.session and settings != self.connected_settings:
                    self.stop_stream()
                    self.session = self.last_data = None
                    self.last_check = 0
                    self.status['connection'].set('Cấu hình đã đổi — đang tự kết nối tới robot đã chọn')
                    for key in ('mapping', 'sensors', 'motion', 'navigation'):
                        self.status[key].set('Đang chờ kết nối mới')
                if self.session and settings == self.connected_settings:
                    _, updated, _ = self.stream.snapshot() if self.stream else (None, 0, 0)
                    if self.connection.restart_due(now, alive=bool(self.stream and self.stream.alive()), updated=updated):
                        if not self.connection.stream_recovering:
                            self.append('Đang tự phục hồi luồng trạng thái ROS trên PC…')
                        self.connection.stream_recovering = True
                        self.last_data = None
                        self.start_stream(settings, self.stream_key)
                if not self.connecting and (settings != self.requested_settings or now >= self.connection.next_check):
                    self.check(quiet=True)
        if not self.closed:
            self.root.after(100, self.tick)

    def stop_stream(self):
        previous, self.stream = self.stream, None
        self.stream_key = None
        if previous:
            threading.Thread(target=previous.close, daemon=True).start()

    def close(self):
        self.closed = True
        self.pending_session = None
        if self.stream:
            stream, self.stream = self.stream, None
            threading.Thread(target=stream.close, daemon=False).start()
        for callback in self.root.tk.call('after', 'info'):
            # Dashboard timers belong to child widgets. Cancel the timer here;
            # each widget removes its own Tcl command during destroy().
            self.root.tk.call('after', 'cancel', callback)
        self.root.destroy()

    def open_logs(self):
        if WINDOWS:
            os.startfile(STATE)
        else:
            subprocess.Popen(["xdg-open", str(STATE)], stdin=subprocess.DEVNULL,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def tab_changed(self, event=None):
        if self.tabs.select() == str(self.tab_frames['tasks']):
            self.refresh_tasks()

    def task_domain_value(self):
        domain = int(self.task_domain.get())
        if not 0 <= domain <= 232:
            raise ValueError("ROS domain phải trong khoảng 0–232")
        return domain

    def refresh_tasks(self):
        if self.busy or self.stop_busy:
            return
        settings, domain = self.config(), self.task_domain_value()
        def loaded(result):
            self.task_settings = settings
            self.task_list_domain = domain
            self.task_records = {str(item['pid']): item for item in json.loads(result)['tasks']}
            self.tasks_tree.delete(*self.tasks_tree.get_children())
            for pid, item in self.task_records.items():
                self.tasks_tree.insert('', 'end', iid=pid, text=item['label'],
                                       values=(pid, item['domain'], item['source']))
            self.append(f"Có {len(self.task_records)} tác vụ robot trong domain {domain}.")
        self.task("Đang đọc tác vụ robot…", lambda: remote(settings, 'tasks', '--domain', str(domain)), loaded)

    def stop_robot_tasks(self, scope, *, selected=False):
        if self.stop_busy:
            return
        settings, domain = self.config(), self.task_domain_value()
        if selected:
            if settings != getattr(self, 'task_settings', None) or domain != getattr(self, 'task_list_domain', None):
                self.dialogs.showinfo("Tác vụ", "Hãy tải lại danh sách cho kết nối/domain đã chọn.")
                return
            tasks = [self.task_records[pid] for pid in self.tasks_tree.selection() if pid in self.task_records]
            if not tasks:
                self.dialogs.showinfo("Tác vụ", "Chọn tác vụ cần dừng trong danh sách.")
                return
            self.confirm_stop_tasks(settings, domain, scope, tasks)
        else:
            self.control_task("Đang kiểm tra tác vụ cần dừng…",
                              lambda: remote(settings, 'tasks', '--domain', str(domain), '--scope', scope),
                              lambda result: self.confirm_stop_tasks(settings, domain, scope, json.loads(result)['tasks']))

    def start_explore(self):
        settings = self.require_session(self.start_explore)
        if not settings:
            return
        if self.session.get("mode") != "mapping":
            self.dialogs.showinfo("Tự khám phá", "Tự khám phá chạy trên map đang dựng. Bắt đầu Mapping trước.")
            return
        dry_run = self.explore_dry_run.get()
        if not dry_run and not self.dialogs.askyesno(
                "Bắt đầu tự khám phá?",
                "Nav2 và explore_lite sẽ tự chọn vùng chưa quét và gửi lệnh /cmd_vel.\n\n"
                "Robot chỉ chạy khi công tắc A ở AUTO và E đang bật motor. Hãy đứng gần robot, cầm tay cầm, "
                "khu vực không có người hoặc bậc/hố. Gạt A về MANUAL hoặc bấm F để dừng ngay.\n\nTiếp tục?",
                default='no', icon='warning'):
            return
        domain = str(self.session["domain"])
        def started(result):
            report = json.loads(result)
            if report.get("explore") == "already_running":
                self.append("Tự khám phá đang chạy rồi.")
            else:
                self.append("Đã bật tự khám phá" + (" (chạy thử, bánh xe không nhận lệnh)" if dry_run else "")
                            + ". Nav2 cần khoảng 25 giây trước đích đầu tiên; gạt A xuống AUTO khi sẵn sàng.")
        self.task("Đang bật Nav2 + explore_lite trên Pi…",
                  lambda: remote(settings, "explore-start", "--domain", domain, *(["--dry-run"] if dry_run else [])),
                  started)

    def stop_explore(self):
        if self.stop_busy:
            return
        settings = self.config()
        def stopped(result):
            state = json.loads(result).get("explore")
            self.append("Đã dừng tự khám phá; Nav2 không còn gửi lệnh. Lái thủ công bằng tay cầm (A ở MANUAL)."
                        if state == "stopped" else "Tự khám phá không chạy.")
        self.control_task("Đang dừng tự khám phá…", lambda: remote(settings, "explore-stop"), stopped)

    def confirm_stop_tasks(self, settings, domain, scope, tasks):
        if not tasks:
            self.append(f"Không có tác vụ {scope} đang chạy trong domain {domain}.")
            self.check()
            self.root.after(100, self.refresh_tasks)
            return
        details = '\n'.join(f"• {item['label']} — PID {item['pid']}" for item in tasks)
        warning = '\n\nMap đang tạo chưa lưu sẽ bị mất. Các map đã lưu được giữ.' if any(
            item['kind'] == 'launch' and item['mode'] == 'mapping' for item in tasks) else ''
        if not self.dialogs.askyesno("Dừng tác vụ robot?", f"Dừng các tác vụ sau trên Pi {settings['host']}, domain {domain}?\n\n" + details + warning,
                                   default='no', icon='warning'):
            self.append("Đã hủy yêu cầu dừng tác vụ.")
            return
        targets = base64.b64encode(json.dumps([dict(pid=item['pid'], start_ticks=item['start_ticks']) for item in tasks]).encode()).decode('ascii')
        def finished(result):
            report = json.loads(result)
            if report['status'] == 'partial':
                self.append("Chưa dừng hết tác vụ. PID còn chạy: " + ', '.join(map(str, report['remaining'])))
            elif report['status'] == 'nothing_running':
                self.append("Các tác vụ đã kết thúc trước khi gửi yêu cầu dừng.")
            else:
                self.append(f"Đã dừng {len(report['stopped'])} tác vụ robot và các tiến trình con.")
            self.check()
            self.root.after(300, self.refresh_tasks)
        self.control_task("Đang dừng tác vụ robot…", lambda: remote(settings, 'stop-tasks', '--domain', str(domain),
                          '--scope', scope, '--targets', targets), finished)

    def build_map_tab(self, parent):
        for actions in ((("Tải lại danh sách", self.refresh_maps), ("Tải map về PC", self.download_map)),
                        (("Đổi tên", self.rename_map), ("Chuyển vào thùng rác", self.delete_map))):
            row = ttk.Frame(parent)
            row.pack(fill="x", pady=(0, 8))
            for text, action in actions:
                button = self.button(row, text, action)
                if text == 'Chuyển vào thùng rác':
                    button.configure(style='Danger.TButton')
                button.pack(side="left", padx=(0, 8))
        ttk.Label(parent, text="Chọn map chỉ để xem trước; map đang dựng không bị thay đổi.",
                  wraplength=1000).pack(side="bottom", anchor="w", pady=(8, 0))
        panes = ttk.Frame(parent)
        panes.pack(fill="both", expand=True)
        left, right = ttk.Frame(panes), ttk.Frame(panes)
        orientation = [None]
        def arrange(event):
            wide = event.width >= 720
            if wide == orientation[0]:
                return
            orientation[0] = wide
            left.grid(row=0, column=0, sticky='nsew', padx=(0, 14) if wide else 0, pady=0 if wide else (0, 14))
            right.grid(row=0 if wide else 1, column=1 if wide else 0, sticky='nsew')
            panes.columnconfigure(0, weight=2 if wide else 1)
            panes.columnconfigure(1, weight=3 if wide else 0)
            panes.rowconfigure(0, weight=1)
            panes.rowconfigure(1, weight=0 if wide else 1)
        parent.bind('<Configure>', arrange, add='+')
        self.maps_tree = ttk.Treeview(left, columns=("size", "when"), selectmode="browse", height=8)
        self.maps_tree.heading("#0", text="Tên map")
        self.maps_tree.column("#0", width=185, minwidth=120)
        for key, label, width in (("size", "Kích thước", 90), ("when", "Lưu lúc", 125)):
            self.maps_tree.heading(key, text=label)
            self.maps_tree.column(key, width=width)
        scroll = ttk.Scrollbar(left, command=self.maps_tree.yview)
        scroll.pack(side="right", fill="y")
        self.maps_tree.configure(yscrollcommand=scroll.set)
        self.maps_tree.pack(fill="both", expand=True)
        self.maps_tree.bind("<<TreeviewSelect>>", self.preview_map)
        self.map_info = tk.StringVar(value="Chọn một map để xem trước")
        ttk.Label(right, textvariable=self.map_info, wraplength=550).pack(fill="x", pady=(0, 6))
        self.preview_canvas = tk.Canvas(right, background="#101014", highlightthickness=0, height=250)
        self.preview_canvas.pack(fill="both", expand=True)
        self.preview_canvas.bind("<Configure>", self.draw_preview)
        self.preview_image = None

    def refresh_maps(self, *, background=False):
        if self.busy or self.stop_busy or (background and self.pending_session is not None):
            return
        settings = self.config()
        selected = self.maps_tree.selection()
        def loaded(result):
            self.map_settings = settings
            self.map_records = {item["name"]: item for item in json.loads(result)["maps"]}
            self.preview_generation += 1
            self.preview_record = None
            self.preview_image = None
            self.draw_preview()
            self.maps_tree.delete(*self.maps_tree.get_children())
            for name, item in self.map_records.items():
                values = ("Lỗi file", "") if "error" in item else (
                    f"{item['width']}×{item['height']}", datetime.fromtimestamp(item["modified"]).strftime("%d/%m %H:%M"))
                self.maps_tree.insert("", "end", iid=name, text=name, values=values)
            if self.map_records:
                name = selected[0] if selected and selected[0] in self.map_records else next(iter(self.map_records))
                self.maps_tree.selection_set(name)
            else:
                self.map_info.set("Chưa có map đã lưu trên Pi")
            self.append(f"Đã tải danh sách {len(self.map_records)} map.")
        self.task("Đang đọc danh sách map trên Pi…", lambda: remote(settings, "maps"), loaded)

    def selected_record(self):
        selected = self.maps_tree.selection()
        if not selected:
            self.dialogs.showinfo("Bản đồ", "Chọn một map trong danh sách trước.")
            return None
        record = self.map_records.get(selected[0])
        if not record or "error" in record:
            self.dialogs.showerror("Bản đồ", record.get("error", "Map không còn trong danh sách") if record else "Map không còn trong danh sách")
            return None
        settings = self.config()
        if settings != self.map_settings:
            self.dialogs.showinfo("Bản đồ", "Kết nối đã đổi. Hãy tải lại danh sách map.")
            return None
        return settings, record

    def preview_map(self, event=None):
        selected = self.maps_tree.selection()
        if not selected:
            return
        name = selected[0]
        self.preview_generation += 1
        generation = self.preview_generation
        self.preview_image = None
        self.preview_record = None
        self.draw_preview()
        record = self.map_records[name]
        if "error" in record:
            self.map_info.set("Map không đọc được: " + record["error"])
            return
        settings = dict(self.map_settings)
        self.map_info.set("Đang tải ảnh xem trước: " + name)
        self.selected_map_text.set("Map đã chọn: " + name + " (chưa nạp vào navigation)")
        def loaded(result):
            if generation != self.preview_generation:
                return
            info = json.loads(result)
            self.preview_record = info
            self.preview_image = tk.PhotoImage(data=info["png_b64"])
            self.draw_preview()
            self.map_info.set(f"{name}\n{info['width']}×{info['height']} ô · {info['resolution']:.3f} m/ô · "
                              f"{info['width']*info['resolution']:.1f}×{info['height']*info['resolution']:.1f} m")
        def worker():
            try:
                self.events.put(("preview", True, remote(settings, "map-preview", "--name", name), loaded))
            except Exception as exc:
                if generation == self.preview_generation:
                    def failed(text):
                        if generation == self.preview_generation:
                            self.map_info.set("Không tải được ảnh: " + text)
                    self.events.put(("preview", False, str(exc), failed))
        threading.Thread(target=worker, daemon=True).start()

    def draw_preview(self, event=None):
        self.preview_canvas.delete("all")
        if self.preview_image is None:
            width, height = self.preview_canvas.winfo_width(), self.preview_canvas.winfo_height()
            self.preview_canvas.create_text(width / 2, height / 2, text='Chọn một bản đồ\nđể xem trước',
                                            fill='#aaa9ad', font=('Segoe UI', 11), justify='center')
            return
        width, height = max(1, self.preview_canvas.winfo_width()-12), max(1, self.preview_canvas.winfo_height()-12)
        ratio = min(width / self.preview_image.width(), height / self.preview_image.height())
        if ratio >= 1:
            self.display_image = self.preview_image.zoom(max(1, min(4, int(ratio))))
        else:
            import math
            factor = max(1, math.ceil(1 / ratio))
            self.display_image = self.preview_image.subsample(factor)
        self.preview_canvas.create_image((width+12)/2, (height+12)/2, image=self.display_image)

    def download_map(self):
        selected = self.selected_record()
        if not selected:
            return
        settings, record = selected
        destination = self.file_dialogs.askdirectory(title="Chọn thư mục tải map về PC", initialdir=str(Path.home() / "Downloads"))
        if not destination:
            return
        def worker():
            payload = json.loads(remote(settings, "map-export", "--name", record["name"], "--revision", record["revision"]))
            return str(write_download(payload, destination))
        self.task("Đang tải YAML và PGM về PC…", worker, lambda path: self.append("Đã tải map về: " + path))

    def rename_map(self):
        selected = self.selected_record()
        if not selected:
            return
        settings, record = selected
        name = self.input_dialogs.askstring("Đổi tên map", "Tên mới (chữ không dấu, số, - hoặc _):", initialvalue=record["name"])
        if not name or name == record["name"]:
            return
        validate_map_name(name)
        self.task("Đang đổi tên map…", lambda: remote(settings, "map-rename", "--name", record["name"],
                  "--new-name", name, "--revision", record["revision"]),
                  lambda result: (self.append("Đã đổi tên map thành " + name), self.root.after(100, self.refresh_maps)))

    def delete_map(self):
        selected = self.selected_record()
        if not selected:
            return
        settings, record = selected
        if not self.dialogs.askyesno("Chuyển map vào thùng rác?", "Chuyển map " + record["name"] +
                                   " vào thư mục .trash trên Pi?\nMap đang dựng trong bộ nhớ không bị thay đổi.", default="no"):
            return
        self.task("Đang chuyển map vào thùng rác…", lambda: remote(settings, "map-delete", "--name", record["name"],
                  "--revision", record["revision"]),
                  lambda result: (self.append("Đã chuyển map vào " + json.loads(result)["trash"]),
                                  self.root.after(100, self.refresh_maps)))


def create_root():
    if os.name == "nt":
        import ctypes
        try:
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except (AttributeError, OSError):
            pass
    # The class name is the Linux WM_CLASS, matched by the launcher's StartupWMClass.
    return tk.Tk(className="RPBotStudio")


def main():
    root = create_root()
    Console(root)
    root.mainloop()


if __name__ == "__main__":
    main()
