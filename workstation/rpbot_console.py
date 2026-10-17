"""Small Windows launcher for an existing ROS 2 robot and WSL RViz."""
from datetime import datetime
import json
import os
from pathlib import Path
import queue
import re
import shlex
import socket
import subprocess
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
from console_layout import build as build_layout
from stream_client import TelemetryStream
from map_library import map_name as validate_map_name

HERE = Path(__file__).resolve().parent
STATE = Path(os.environ.get("LOCALAPPDATA", str(Path.home()))) / "RPBotConsole"
DEFAULTS = {"host": "192.168.0.120", "user": "sharkagv",
            "distro": "Ubuntu-24.04", "wsl_user": "shark"}
HIDDEN = getattr(subprocess, "CREATE_NO_WINDOW", 0)


def wsl_path(path):
    path = Path(path).resolve().as_posix()
    if not re.match(r"^[A-Za-z]:/", path):
        raise ValueError("Use a local Windows drive path for RPBot Console")
    return "/mnt/" + path[0].lower() + path[2:]


def validate_settings(settings):
    for key in ("user", "wsl_user"):
        if not re.fullmatch(r"[a-z_][a-z0-9_-]{0,31}", settings[key]):
            raise ValueError(f"Invalid {key}")
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9.-]{0,252}", settings["host"]):
        raise ValueError("Enter a hostname or IPv4 address")
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]{0,63}", settings["distro"]):
        raise ValueError("Invalid WSL distro name")
    return settings


def wsl_command(settings, *args):
    return ["wsl.exe", "-d", settings["distro"], "-u", settings["wsl_user"], "--", *args]


def capture(command, *, input=None, timeout=45):
    result = subprocess.run(command, input=input, capture_output=True, text=True,
                            encoding="utf-8", errors="replace", timeout=timeout,
                            creationflags=HIDDEN)
    if result.returncode:
        detail = (result.stderr + "\n" + result.stdout).strip()
        raise RuntimeError(detail or f"Command failed ({result.returncode})")
    return result.stdout.strip()


def remote(settings, action, *args):
    # Only known actions; arguments are POSIX quoted before SSH passes them to a shell.
    command = shlex.join(["python3", "-", action, *args])
    return capture(wsl_command(settings, "ssh", "-T", "-o", "BatchMode=yes",
                               "-o", "ConnectTimeout=5", "-o", "ServerAliveInterval=5",
                               "-o", "ServerAliveCountMax=2",
                               settings["user"] + "@" + settings["host"], command),
                   input=(HERE / "map_library.py").read_text(encoding="utf-8") + "\n" +
                         (HERE / "remote_ops.py").read_text(encoding="utf-8"))


def session_args(settings, action, session):
    command = wsl_command(settings, "bash", wsl_path(HERE / "ros_session.sh"),
                          action, "--domain", str(session["domain"]))
    profile = STATE / "fastdds.xml"
    xml = pc_profile(session.get("profile_xml", ""), settings["host"], session["domain"])
    # A second Console/diagnostic process must never observe a half-written XML.
    with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=STATE, delete=False) as file:
        file.write(xml)
        temporary = file.name
    os.replace(temporary, profile)
    command += ["--profile", wsl_path(profile)]
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
    return ET.tostring(root, encoding="unicode")


def inspect_robot(settings):
    robot = json.loads(remote(settings, "status"))
    sessions = [item for item in robot["launches"]
                if item["mode"] == "mapping" and not item["simulated_time"]]
    if len(sessions) != 1:
        return {"robot": robot, "session": None, "data": None,
                "note": "Chưa có đúng một phiên mapping thật. Dùng nút Bắt đầu mapping khi robot đã sẵn sàng."}
    session = sessions[0]
    return {"robot": robot, "session": session, "data": None}


def check_connection(settings):
    report = inspect_robot(settings)
    if report["session"]:
        report["data"] = json.loads(capture(session_args(settings, "probe", report["session"]), timeout=25))
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


class Console:
    def __init__(self, root):
        self.root = root
        STATE.mkdir(parents=True, exist_ok=True)
        self.settings = dict(DEFAULTS)
        config = STATE / "settings.json"
        if config.is_file():
            try:
                self.settings.update(json.loads(config.read_text(encoding="utf-8")))
            except (ValueError, OSError):
                pass
        self.events = queue.Queue()
        self.busy = False
        self.session = None
        self.connected_settings = None
        self.last_data = None
        self.last_check = 0.0
        self.last_attempt = 0.0
        self.viewers = {}
        self.actions = []
        self.connecting = False
        self.last_meta = 0.0
        self.stream = None
        self.stream_key = None
        self.stream_seen = 0
        self.map_records = {}
        self.preview_record = None
        self.preview_generation = 0
        self.closed = False
        root.report_callback_exception = lambda kind, error, trace: self.append("Lỗi thao tác: " + str(error))
        build_layout(self)
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.after(150, self.drain)
        root.after(600, self.check)
        root.after(100, self.tick)

    @staticmethod
    def new_name():
        return datetime.now().strftime("room_%Y%m%d_%H%M%S")

    def button(self, parent, text, action):
        button = ttk.Button(parent, text=text, command=action)
        self.actions.append(button)
        return button

    def config(self):
        settings = validate_settings({key: var.get().strip() for key, var in self.fields.items()})
        (STATE / "settings.json").write_text(json.dumps(settings, indent=2), encoding="utf-8")
        return settings

    def append(self, text):
        line = f"[{datetime.now():%H:%M:%S}] {text}\n"
        self.log.configure(state="normal")
        self.log.insert("end", line)
        self.log.see("end")
        self.log.configure(state="disabled")
        with (STATE / "console.log").open("a", encoding="utf-8") as file:
            file.write(line)

    def task(self, title, work, done=None):
        if self.busy:
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

    def drain(self):
        try:
            while True:
                kind, ok, result, done = self.events.get_nowait()
                if kind == "task":
                    self.busy = False
                    for button in self.actions:
                        button.state(["!disabled"])
                    self.activity.set("Sẵn sàng" if ok else "Thao tác chưa hoàn tất")
                elif kind == "metadata":
                    self.connecting = False
                    if not self.busy:
                        self.activity.set("Đã kiểm tra kết nối" if ok else "Đang chờ kết nối lại")
                if ok:
                    if done:
                        done(result)
                    elif result:
                        self.append(str(result))
                else:
                    self.append("Lỗi: " + result)
                    if kind == "preview" and done:
                        done(result)
                    # Never retain a green readiness indicator after a failed check.
                    if done == self.checked:
                        self.last_meta = 0
                        self.status["connection"].set("SSH chưa kết nối — dữ liệu ROS được kiểm tra riêng")
                    elif self.session is None:
                        self.status["mapping"].set("Chưa xác nhận kết quả thao tác — bấm Kiểm tra để cập nhật")
        except queue.Empty:
            pass
        if not self.closed:
            self.root.after(150, self.drain)

    def check(self):
        if self.connecting:
            return
        try:
            settings = self.config()
        except ValueError as exc:
            messagebox.showerror("Cấu hình kết nối", str(exc))
            return
        self.last_attempt = time.monotonic()
        self.connecting = True
        if not self.busy:
            self.activity.set("Đang kiểm tra kết nối…")
        def worker():
            try:
                self.events.put(("metadata", True, (settings, inspect_robot(settings)), self.checked))
            except Exception as exc:
                self.events.put(("metadata", False, str(exc), self.checked))
        threading.Thread(target=worker, daemon=True).start()

    def checked(self, result):
        settings, report = result
        robot, data = report["robot"], report["data"]
        self.connected_settings = settings
        self.session = report["session"]
        self.last_meta = time.monotonic()
        self.status["connection"].set(f"SSH: {robot['hostname']} ({settings['host']})")
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
            return
        key = (tuple(sorted(settings.items())), self.session["pid"], self.session["domain"], self.session.get("profile_xml"))
        if self.stream_key != key or not self.stream or not self.stream.alive():
            self.stop_stream()
            self.last_data = None
            self.last_check = 0
            for name in ("mapping", "sensors", "motion", "navigation"):
                self.status[name].set("Đang chờ dữ liệu ROS mới…")
            self.topic_tree.delete(*self.topic_tree.get_children())
            self.stream = TelemetryStream(session_args(settings, "stream", self.session),
                                           STATE / "telemetry.log", creationflags=HIDDEN)
            self.stream_key, self.stream_seen = key, 0
            self.append("Đã kết nối phiên mapping — đang nhận trạng thái ROS liên tục.")
        if data:
            self.render_data(data)
        report["data"] = self.last_data
        (STATE / "last-diagnostics.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        if getattr(self, "map_settings", None) != settings and not self.busy:
            self.root.after(100, self.refresh_maps)

    def render_data(self, data):
        self.last_data = data
        self.last_check = time.monotonic()
        domain = self.session["domain"]
        receiving = any(topic["age_s"] is not None and topic["age_s"] < 2 for topic in data["topics"].values())
        self.status["connection"].set(("ROS: đang nhận trực tiếp" if receiving else "ROS: chưa nhận dữ liệu mới")
                                      + f" · Pi {self.connected_settings['host']}" + (" · SSH chưa xác nhận" if not self.last_meta else ""))
        if data["mapping_ready"]:
            info = data["map"]
            self.status["mapping"].set(f"Có dữ liệu · domain {domain} · map {info['width']}×{info['height']} · {info['resolution']:.3f} m/ô · TF OK")
        else:
            self.status["mapping"].set(f"Domain {domain} · Thiếu map, TF hoặc dữ liệu scan/odom mới")
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
        self.status["navigation"].set("Có action server — mở RViz để vận hành" if data["navigation_available"] else "Chưa chạy Nav2 trên Pi — hiện dùng mapping")
        for topic, info in data["topics"].items():
            age = info["age_s"]
            values = (info["messages"], "—" if age is None else f"{age:.2f}",
                      "Đang nhận" if age is not None and age < 2 else "Chưa có dữ liệu mới")
            if self.topic_tree.exists(topic):
                self.topic_tree.item(topic, values=values)
            else:
                self.topic_tree.insert("", "end", iid=topic, text=topic, values=values)

    def require_session(self):
        settings = self.config()
        if not self.session or settings != self.connected_settings or time.monotonic() - self.last_meta > 30:
            messagebox.showinfo("Kết nối robot", "Bấm Kết nối / Kiểm tra để xác nhận phiên robot trước.")
            return None
        return settings

    def open_view(self, action):
        settings = self.require_session()
        if not settings:
            return
        if action == "navigation" and not self.last_data.get("navigation_available"):
            messagebox.showinfo("Navigation", "Pi hiện chưa chạy Nav2. Mapping và lưu map đã dùng được; chức năng chọn đích cần cấu hình Nav2 trên robot trước.")
            return
        if action in self.viewers and self.viewers[action].poll() is None:
            messagebox.showinfo("RViz", "Cửa sổ RViz này đang mở.")
            return
        log = STATE / (action + ".log")
        with log.open("w", encoding="utf-8") as file:
            self.viewers[action] = subprocess.Popen(session_args(settings, action, self.session),
                                                    stdout=file, stderr=subprocess.STDOUT,
                                                    creationflags=HIDDEN)
        self.append(f"Đang mở RViz ({action}). Log: {log}")

    def save_map(self):
        settings = self.require_session()
        if not settings:
            return
        name = self.map_name.get().strip()
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", name):
            messagebox.showerror("Tên map", "Dùng 1–64 chữ không dấu, số, dấu - hoặc _.")
            return
        domain = str(self.session["domain"])

        def saved(result):
            self.append("Đã lưu: " + json.loads(result)["saved"])
            self.map_name.set(self.new_name())
            self.root.after(100, self.refresh_maps)
        self.task("Lưu bản đồ hiện tại trên Pi…", lambda: remote(settings, "save", "--name", name, "--domain", domain), saved)

    def show_logs(self):
        settings = self.config()
        self.task("Đọc log robot…", lambda: remote(settings, "logs"))

    def reset_map(self):
        settings = self.require_session()
        if not settings:
            return
        if self.last_data.get("navigation_available"):
            messagebox.showinfo("Tạo map lại", "Hãy dừng chế độ navigation và chuyển sang mapping trước khi tạo map mới.")
            return
        if not messagebox.askyesno(
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
            if existing:
                self.append("Mapping đang chạy — kết nối vào phiên hiện có. Muốn dựng map từ đầu, bấm Tạo map lại.")
            else:
                self.append("Đã gửi yêu cầu bắt đầu mapping." if action == "start" else "Đã dừng phiên mapping của Console.")
            self.root.after(0 if existing else 2000, self.check)
        self.task("Kiểm tra / bắt đầu mapping…" if action == "start" else "Đang dừng mapping…",
                  lambda: remote(settings, action), finished)

    def tick(self):
        now = time.monotonic()
        if self.stream and self.session:
            data, updated, frames = self.stream.snapshot()
            if frames != self.stream_seen:
                self.stream_seen = frames
                self.render_data(data)
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
        if self.auto.get() and not self.connecting and now - self.last_attempt >= 15:
            self.check()
        if not self.closed:
            self.root.after(100, self.tick)

    def stop_stream(self):
        previous, self.stream = self.stream, None
        self.stream_key = None
        if previous:
            threading.Thread(target=previous.close, daemon=True).start()

    def close(self):
        self.closed = True
        if self.stream:
            stream, self.stream = self.stream, None
            threading.Thread(target=stream.close, daemon=False).start()
        self.root.destroy()

    def open_logs(self):
        os.startfile(STATE)

    def build_map_tab(self, parent):
        row = ttk.Frame(parent)
        row.pack(fill="x", pady=(0, 8))
        for text, action in (("Tải lại danh sách", self.refresh_maps), ("Tải map về PC", self.download_map),
                             ("Đổi tên", self.rename_map), ("Chuyển vào thùng rác", self.delete_map)):
            self.button(row, text, action).pack(side="left", padx=(0, 8))
        ttk.Label(parent, text="Chọn map chỉ để xem trước; map đang dựng không bị thay đổi.",
                  wraplength=1000).pack(side="bottom", anchor="w", pady=(8, 0))
        panes = ttk.Panedwindow(parent, orient="horizontal")
        panes.pack(fill="both", expand=True)
        left, right = ttk.Frame(panes), ttk.Frame(panes)
        panes.add(left, weight=2)
        panes.add(right, weight=3)
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
        self.preview_canvas = tk.Canvas(right, background="#252b36", highlightthickness=0, height=250)
        self.preview_canvas.pack(fill="both", expand=True)
        self.preview_canvas.bind("<Configure>", self.draw_preview)
        self.preview_image = None

    def refresh_maps(self):
        if self.busy:
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
            messagebox.showinfo("Bản đồ", "Chọn một map trong danh sách trước.")
            return None
        record = self.map_records.get(selected[0])
        if not record or "error" in record:
            messagebox.showerror("Bản đồ", record.get("error", "Map không còn trong danh sách") if record else "Map không còn trong danh sách")
            return None
        settings = self.config()
        if settings != self.map_settings:
            messagebox.showinfo("Bản đồ", "Kết nối đã đổi. Hãy tải lại danh sách map.")
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
        destination = filedialog.askdirectory(title="Chọn thư mục tải map về PC", initialdir=str(Path.home() / "Downloads"))
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
        name = simpledialog.askstring("Đổi tên map", "Tên mới (chữ không dấu, số, - hoặc _):", initialvalue=record["name"])
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
        if not messagebox.askyesno("Chuyển map vào thùng rác?", "Chuyển map " + record["name"] +
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
    return tk.Tk()


def main():
    root = create_root()
    Console(root)
    root.mainloop()


if __name__ == "__main__":
    main()
