"""Tk layout for RPBot Console. Workers and robot actions live in rpbot_console."""
import tkinter as tk
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText


def build(app):
    root = app.root
    root.title("RPBot Console — Mapping & Debug")
    root.geometry("1140x920")
    root.minsize(980, 780)
    style = ttk.Style(root)
    style.theme_use("clam")
    style.configure("TButton", padding=(10, 8))
    style.configure("TLabel", font=("Segoe UI", 10))
    style.configure("Title.TLabel", font=("Segoe UI", 18, "bold"))
    style.configure("TNotebook.Tab", padding=(18, 8))
    outer = ttk.Frame(root, padding=16)
    outer.pack(fill="both", expand=True)
    header = ttk.Frame(outer)
    header.pack(fill="x")
    ttk.Label(header, text="RPBot Console", style="Title.TLabel").pack(side="left")
    app.activity = tk.StringVar(value="Sẵn sàng kết nối")
    ttk.Label(header, textvariable=app.activity).pack(side="right")
    box = ttk.LabelFrame(outer, text="Kết nối robot", padding=8)
    box.pack(fill="x", pady=(10, 8))
    app.fields = {}
    for index, (key, label) in enumerate((("host", "IP Pi"), ("user", "User Pi"),
                                         ("distro", "WSL"), ("wsl_user", "User WSL"))):
        ttk.Label(box, text=label).grid(row=0, column=index*2, padx=(6, 4))
        var = tk.StringVar(value=app.settings[key])
        app.fields[key] = var
        ttk.Entry(box, textvariable=var, width=19 if key in ("host", "distro") else 12).grid(row=0, column=index*2+1, sticky="ew", padx=(0, 6))
        box.columnconfigure(index*2+1, weight=1)
    app.button(box, "Kết nối / Kiểm tra", app.check).grid(row=1, column=0, columnspan=3, sticky="w", pady=(8, 0))
    app.auto = tk.BooleanVar(value=True)
    ttk.Checkbutton(box, text="Tự kết nối lại", variable=app.auto).grid(row=1, column=3, columnspan=2, sticky="w")
    ttk.Label(box, text="Trạng thái ROS cập nhật liên tục").grid(row=1, column=5, columnspan=3, sticky="e")
    info = ttk.LabelFrame(outer, text="Trạng thái trực tiếp", padding=8)
    info.pack(fill="x", pady=(0, 10))
    app.status = {}
    for key, label in (("connection", "Kết nối"), ("mapping", "Chế độ / map"), ("sensors", "Cảm biến"),
                       ("motion", "Motor / vận tốc"), ("navigation", "Navigation"), ("boot", "Tự khởi động")):
        row = ttk.Frame(info)
        row.pack(fill="x", pady=1)
        ttk.Label(row, text=label, width=16).pack(side="left")
        var = tk.StringVar(value="Chưa kiểm tra")
        ttk.Label(row, textvariable=var, wraplength=850).pack(side="left", fill="x", expand=True)
        app.status[key] = var
    footer = ttk.Frame(outer)
    footer.pack(side="bottom", fill="x", pady=(8, 0))
    ttk.Label(footer, text="Nhật ký thao tác").pack(anchor="w", pady=(0, 3))
    app.log = ScrolledText(footer, height=3, font=("Consolas", 9), wrap="word", state="disabled")
    app.log.pack(fill="x")
    app.tabs = ttk.Notebook(outer)
    app.tabs.pack(fill="both", expand=True)
    app.tab_frames = {}
    for key, title in (("mapping", "Mapping"), ("maps", "Bản đồ đã lưu"), ("debug", "Debug"), ("navigation", "Navigation")):
        frame = ttk.Frame(app.tabs, padding=12)
        app.tabs.add(frame, text=title)
        app.tab_frames[key] = frame
    mapping = app.tab_frames["mapping"]
    ttk.Label(mapping, text="Dựng bản đồ môi trường", style="Title.TLabel").pack(anchor="w", pady=(0, 8))
    ttk.Label(mapping, text="Mở RViz để xem map và LiDAR. Bắt đầu sẽ dùng lại phiên đang chạy; Tạo map lại sẽ dựng từ đầu.",
              wraplength=1000).pack(anchor="w", pady=(0, 10))
    row = ttk.Frame(mapping)
    row.pack(fill="x")
    for text, action in (("Mở RViz · Map", lambda: app.open_view("rviz")),
                         ("Bắt đầu mapping", lambda: app.service("start")),
                         ("Dừng phiên Console", lambda: app.service("stop")), ("Tạo map lại", app.reset_map)):
        app.button(row, text, action).pack(side="left", padx=(0, 8))
    row = ttk.LabelFrame(mapping, text="Lưu map hiện tại trên Pi", padding=12)
    row.pack(fill="x", pady=16)
    app.map_name = tk.StringVar(value=app.new_name())
    ttk.Entry(row, textvariable=app.map_name, width=36).pack(side="left", padx=(0, 10))
    app.button(row, "Lưu map", app.save_map).pack(side="left")
    ttk.Label(mapping, text="Đóng Console/RViz vẫn giữ phiên robot chạy. Nút Dừng chỉ quản lý phiên do Console mở.",
              wraplength=1000).pack(anchor="w")
    app.build_map_tab(app.tab_frames["maps"])
    debug = app.tab_frames["debug"]
    row = ttk.Frame(debug)
    row.pack(fill="x", pady=(0, 10))
    app.button(row, "Đọc log robot", app.show_logs).pack(side="left", padx=(0, 8))
    app.button(row, "Mở thư mục log PC", app.open_logs).pack(side="left")
    app.topic_tree = ttk.Treeview(debug, columns=("count", "age", "state"), height=7)
    app.topic_tree.heading("#0", text="Topic")
    for column, label in (("count", "Số bản tin"), ("age", "Tuổi dữ liệu (s)"), ("state", "Trạng thái")):
        app.topic_tree.heading(column, text=label)
        app.topic_tree.column(column, width=130, anchor="center")
    app.topic_tree.pack(fill="x")
    nav = app.tab_frames["navigation"]
    ttk.Label(nav, text="Điều hướng với Nav2", style="Title.TLabel").pack(anchor="w", pady=(0, 10))
    app.selected_map_text = tk.StringVar(value="Chưa chọn map — chọn trong tab Bản đồ đã lưu")
    ttk.Label(nav, textvariable=app.selected_map_text).pack(anchor="w", pady=8)
    ttk.Label(nav, text="Map được chọn để xem và chuẩn bị cho navigation; việc chọn không nạp map vào SLAM đang chạy.\n"
              "Pi cần có Nav2 và localization hoạt động trước khi gửi đích.", wraplength=1000).pack(anchor="w", pady=8)
    app.button(nav, "Mở RViz · Navigation", lambda: app.open_view("navigation")).pack(anchor="w")
