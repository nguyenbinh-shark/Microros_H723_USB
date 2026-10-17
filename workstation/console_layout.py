"""Native desktop studio layout. All robot actions stay in rpbot_console."""
import tkinter as tk
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText
import webbrowser

import branding as brand
import studio_theme as theme
from status_dashboard import COLORS, StatusDashboard
from ui_language import picker

PAGES = (
    ('mapping', '01', 'Mapping', 'Mapping · Tạo bản đồ', 'Bắt đầu quét → xem trong RViz → lưu map trước khi dừng.'),
    ('navigation', '02', 'Navigation', 'Navigation · Điều hướng', 'Chọn map → đặt vị trí ban đầu → kiểm tra Nav2 → gửi đích.'),
    ('maps', '03', 'Bản đồ đã lưu', 'Bản đồ đã lưu', 'Chọn map để xem trước, tải về hoặc dùng cho Navigation.'),
    ('tasks', '04', 'Tác vụ', 'Tác vụ đang chạy', 'Xem phiên robot và dừng đúng tác vụ trong domain đã chọn.'),
    ('debug', '05', 'Debug', 'Debug · Dữ liệu & nhật ký', 'Kiểm tra tuổi bản tin, trạng thái chi tiết và log robot.'),
    ('settings', '06', 'Kết nối', 'Kết nối robot', 'Địa chỉ Pi, tài khoản SSH và cấu hình WSL trên PC.'),
    ('about', '07', 'Giới thiệu', 'RPBot Studio', 'Thông tin tác giả và liên hệ.'),
)


def line(parent, *, pady=0):
    ttk.Separator(parent).pack(fill='x', pady=pady)


def eyebrow(parent, text, *, pady=(0, 12)):
    ttk.Label(parent, text=text, style='Mono.TLabel').pack(anchor='w', pady=pady)


def action(app, parent, text, callback, *, style='TButton', stop=False):
    button = (app.stop_button if stop else app.button)(parent, text, callback)
    button.configure(style=style)
    return button


def scroll_panel(parent, *, paper=False, stretch=False):
    bg = theme.PAPER if paper else theme.PANEL
    frame_style = 'TFrame' if paper else 'Panel.TFrame'
    outer = ttk.Frame(parent, style=frame_style)
    canvas = tk.Canvas(outer, background=bg, highlightthickness=0, borderwidth=0, yscrollincrement=22)
    scroll = ttk.Scrollbar(outer, orient='vertical', command=canvas.yview)
    inner = ttk.Frame(canvas, style=frame_style, padding=0 if paper else (18, 16))
    window = canvas.create_window((0, 0), window=inner, anchor='nw')
    def resize(event=None):
        height = max(inner.winfo_reqheight(), canvas.winfo_height()) if stretch else inner.winfo_reqheight()
        canvas.itemconfigure(window, width=canvas.winfo_width(), height=height)
        canvas.configure(scrollregion=(0, 0, canvas.winfo_width(), height))
    inner.bind('<Configure>', resize)
    canvas.bind('<Configure>', resize)
    canvas.configure(yscrollcommand=scroll.set)
    canvas.pack(side='left', fill='both', expand=True)
    scroll.pack(side='right', fill='y')
    def wheel(event):
        if canvas.bbox('all') and canvas.bbox('all')[3] > canvas.winfo_height():
            if isinstance(event.widget, (ttk.Treeview, tk.Text, ttk.Spinbox)):
                return
            canvas.yview_scroll(-int(event.delta / 120) or (-1 if event.delta > 0 else 1), 'units')
            return 'break'
    def bind_wheel(widget):
        widget.bind('<MouseWheel>', wheel, add='+')
        for child in widget.winfo_children():
            bind_wheel(child)
    outer.bind_wheel = lambda: bind_wheel(outer)
    outer.canvas, outer.content = canvas, inner
    return outer, inner


def build_status(app, parent):
    app.dashboard = StatusDashboard(app, parent)
    app.dashboard.pack(fill='x', pady=(0, 12))


def build_mapping(app, page):
    row = ttk.Frame(page)
    row.pack(fill='x', pady=(0, 12))
    action(app, row, 'Bắt đầu mapping', lambda: app.service('start'), style='Primary.TButton').pack(side='left', padx=(0, 8))
    action(app, row, 'Mở RViz · Map', lambda: app.open_view('rviz')).pack(side='left', padx=(0, 8))
    action(app, row, 'Dừng tạo map', lambda: app.stop_robot_tasks('mapping'), style='Danger.TButton', stop=True).pack(side='left')
    ttk.Label(page, text='Lưu bản đồ hiện tại', font=('Segoe UI', 10, 'bold')).pack(anchor='w', pady=(2, 6))
    row = ttk.Frame(page)
    row.pack(fill='x')
    app.map_name = tk.StringVar(value=app.new_name())
    ttk.Entry(row, textvariable=app.map_name, width=26).pack(side='left', fill='x', expand=True, padx=(0, 8))
    action(app, row, 'Lưu map', app.save_map, style='Primary.TButton').pack(side='right')
    action(app, page, 'Tạo map lại', app.reset_map).pack(anchor='w', pady=(12, 0))
    ttk.Label(page, text='Tạo map lại sẽ dựng từ đầu. Các bản đồ đã lưu vẫn được giữ.',
              style='Muted.TLabel', font=('Segoe UI', 9), wraplength=600).pack(anchor='w', pady=(12, 0))


def build_navigation(app, page):
    app.selected_map_text = tk.StringVar(value='Chưa chọn map — chọn trong Bản đồ đã lưu')
    ttk.Label(page, textvariable=app.selected_map_text, wraplength=600, font=('Segoe UI', 10, 'bold')).pack(anchor='w', pady=(0, 8))
    action(app, page, 'Chọn map đã lưu  ↗', lambda: app.tabs.select(app.tab_frames['maps']), style='Link.TButton').pack(anchor='w', pady=(0, 12))
    row = ttk.Frame(page)
    row.pack(fill='x', pady=(0, 8))
    action(app, row, 'Bắt đầu Nav', app.start_navigation, style='Primary.TButton').pack(side='left', padx=(0, 8))
    action(app, row, 'Mở RViz · Navigation', lambda: app.open_view('navigation')).pack(side='left', padx=(0, 8))
    action(app, row, 'Dừng Nav', lambda: app.stop_robot_tasks('navigation'), style='Danger.TButton', stop=True).pack(side='left')
    ttk.Label(page, text='Dừng Mapping trước khi bắt đầu Nav. Dùng 2D Pose Estimate trong RViz để đặt vị trí và hướng robot, '
              'rồi chờ localization / Nav2 sẵn sàng trước khi gửi đích.', style='Muted.TLabel',
              font=('Segoe UI', 10), wraplength=850).pack(anchor='w', pady=(10, 0))


def build_tasks(app, page):
    eyebrow(page, '04 / TÁC VỤ ĐANG CHẠY')
    row = ttk.Frame(page)
    row.pack(fill='x', pady=(0, 10))
    ttk.Label(row, text='ROS domain', style='Muted.TLabel').pack(side='left', padx=(0, 8))
    app.task_domain = tk.IntVar(value=0)
    ttk.Spinbox(row, from_=0, to=232, width=5, textvariable=app.task_domain).pack(side='left', padx=(0, 12))
    action(app, row, 'Tải lại tác vụ', app.refresh_tasks).pack(side='left')
    row = ttk.Frame(page)
    row.pack(fill='x', pady=(0, 14))
    action(app, row, 'Dừng tác vụ đã chọn', lambda: app.stop_robot_tasks('all', selected=True), style='Danger.TButton', stop=True).pack(side='left', padx=(0, 8))
    action(app, row, 'Dừng tất cả', lambda: app.stop_robot_tasks('all'), style='Danger.TButton', stop=True).pack(side='left')
    app.tasks_tree = ttk.Treeview(page, columns=('pid', 'domain', 'source'), selectmode='extended', height=5)
    app.tasks_tree.heading('#0', text='Tác vụ robot')
    for key, title, width in (('pid', 'PID', 65), ('domain', 'Domain', 65), ('source', 'Nguồn', 140)):
        app.tasks_tree.heading(key, text=title)
        app.tasks_tree.column(key, width=width, minwidth=55)
    app.tasks_tree.pack(fill='both', expand=True)
    ttk.Label(page, text='Dừng theo domain đã chọn. Có thể chọn nhiều tác vụ bằng Ctrl. '
              'Hãy lưu map trước khi dừng Mapping; các file map và bag đã lưu được giữ.',
              style='Muted.TLabel', font=('Segoe UI', 9), wraplength=600).pack(anchor='w', pady=(12, 0))


def build_debug(app, page):
    eyebrow(page, '05 / DỮ LIỆU & CHẨN ĐOÁN')
    row = ttk.Frame(page)
    row.pack(fill='x', pady=(0, 16))
    action(app, row, 'Đọc log robot', app.show_logs, style='Ink.TButton').pack(side='left', padx=(0, 8))
    action(app, row, 'Mở thư mục log PC', app.open_logs).pack(side='left')
    app.topic_tree = ttk.Treeview(page, columns=('count', 'age', 'state'), height=6)
    app.topic_tree.heading('#0', text='Topic')
    app.topic_tree.column('#0', width=160, minwidth=90)
    for column, label, width in (('count', 'Bản tin', 70), ('age', 'Tuổi dữ liệu (s)', 100), ('state', 'Trạng thái', 160)):
        app.topic_tree.heading(column, text=label)
        app.topic_tree.column(column, width=width, minwidth=55, anchor='center')
    app.topic_tree.pack(fill='both', expand=True)
    ttk.Label(page, text='Dữ liệu được cập nhật trực tiếp. Mất luồng hoặc bản tin quá cũ sẽ được báo riêng.',
              style='Muted.TLabel', font=('Segoe UI', 9), wraplength=600).pack(anchor='w', pady=(12, 0))
    for key, title in (('connection', 'Kết nối'), ('mapping', 'Chế độ / map'), ('sensors', 'Cảm biến'),
                       ('motion', 'Motor'), ('navigation', 'Nav2'), ('boot', 'Tự khởi động')):
        ttk.Label(page, text=title, font=('Segoe UI', 10, 'bold')).pack(anchor='w', pady=(10, 3))
        ttk.Label(page, textvariable=app.status[key], wraplength=900).pack(anchor='w', fill='x')


def build_settings(app, page):
    app.fields = {}
    app.device_entries = {}
    row = ttk.Frame(page)
    row.pack(fill='x', pady=(0, 8))
    row.columnconfigure(1, weight=2)
    row.columnconfigure(3, weight=1)
    for key, title, column, width in (('host', 'Thiết bị / IP', 0, 25), ('user', 'Tài khoản SSH', 2, 16)):
        ttk.Label(row, text=title, font=('Segoe UI', 10, 'bold')).grid(row=0, column=column, padx=(0 if column == 0 else 14, 8), sticky='w')
        app.fields[key] = tk.StringVar(value=app.settings[key])
        entry = ttk.Entry(row, textvariable=app.fields[key], width=width)
        entry.grid(row=0, column=column+1, sticky='ew')
        app.device_entries[key] = entry
    row = ttk.Frame(page)
    row.pack(fill='x')
    action(app, row, 'Kết nối ngay', app.check, style='Primary.TButton').pack(side='left', padx=(0, 8))
    app.ping_button = ttk.Button(row, text='Ping thiết bị', command=app.ping_device)
    app.ping_button.pack(side='left', padx=(0, 8))
    app.ping_status = tk.StringVar(value='Nhập tên thiết bị hoặc IP của bạn (ví dụ robot.local), rồi bấm Ping thiết bị.')
    app.ping_label = tk.Label(page, textvariable=app.ping_status, background=COLORS['unknown'][0],
                              foreground=COLORS['unknown'][1], font=('Segoe UI', 10, 'bold'),
                              anchor='w', justify='left', padx=10, pady=8, wraplength=650)
    app.ping_label.pack(fill='x', pady=(10, 0))
    line(page, pady=(12, 12))
    for key, title, detail in (('distro', 'Ubuntu / WSL', 'Bản phân phối có ROS 2 và RViz'), ('wsl_user', 'Tài khoản WSL', 'Người dùng chạy ROS trên PC')):
        row = ttk.Frame(page)
        row.pack(fill='x', pady=(0, 13))
        labels = ttk.Frame(row)
        labels.pack(side='left', fill='x', expand=True)
        ttk.Label(labels, text=title, font=('Segoe UI', 10, 'bold')).pack(anchor='w')
        ttk.Label(labels, text=detail, style='Muted.TLabel', font=('Segoe UI', 9)).pack(anchor='w', pady=(3, 0))
        app.fields[key] = tk.StringVar(value=app.settings[key])
        ttk.Entry(row, textvariable=app.fields[key], width=25).pack(side='right', padx=(12, 0))
    for variable in app.fields.values():
        variable.trace_add('write', app.invalidate_ping)
    ttk.Label(page, text='Tự khởi động trên Pi', font=('Segoe UI', 10, 'bold')).pack(anchor='w', pady=(16, 5))
    ttk.Label(page, textvariable=app.status['boot'], wraplength=900).pack(anchor='w')
    ttk.Label(page, text='Studio lưu cấu hình trên PC và tự kết nối lại khi robot khả dụng. '
              'Không cần bấm Kiểm tra trước mỗi thao tác. Đóng app vẫn giữ phiên robot đang chạy.',
              style='Muted.TLabel', font=('Segoe UI', 9), wraplength=600).pack(anchor='w', pady=(16, 0))


def build_about(app, page):
    eyebrow(page, '07 / VỀ STUDIO')
    panel = tk.Frame(page, background=theme.INK, padx=22, pady=20)
    panel.pack(fill='x', pady=(0, 20))
    if app.brand_logo:
        tk.Label(panel, image=app.brand_logo, background=theme.INK).pack(side='left', padx=(0, 16))
    tk.Label(panel, text='RPBOT STUDIO\nMAPPING · NAVIGATION', background=theme.INK, foreground=theme.PAPER,
             font=('Bahnschrift', 17, 'bold'), justify='left').pack(side='left')
    ttk.Label(page, text=brand.AUTHOR, font=('Segoe UI', 19, 'bold')).pack(anchor='w')
    ttk.Label(page, text=brand.AUTHOR_EN + '  /  ' + brand.BIO, style='Muted.TLabel',
              font=('Segoe UI', 9), wraplength=600).pack(anchor='w', pady=(6, 12))
    ttk.Label(page, text='Sinh viên Cơ điện tử tại Đại học Bách Khoa Hà Nội. '
              'Quan tâm đến hệ thống điều khiển, mô hình hóa, mô phỏng và tích hợp hệ thống robot.',
              wraplength=600, justify='left').pack(anchor='w')
    line(page, pady=(18, 12))
    for title, url in (('Website  /  nguyenbinh-shark.github.io  ↗', brand.WEBSITE),
                       ('GitHub  /  nguyenbinh-shark  ↗', brand.GITHUB), ('Email  /  ' + brand.EMAIL + '  ↗', 'mailto:' + brand.EMAIL)):
        ttk.Button(page, text=title, style='Link.TButton', command=lambda target=url: webbrowser.open(target)).pack(anchor='w', pady=3)


def build(app):
    root = app.root
    root.title(brand.APP_NAME + ' · ' + brand.AUTHOR)
    width, height = min(1420, root.winfo_screenwidth()-60), min(920, root.winfo_screenheight()-90)
    root.geometry(f'{max(1100, width)}x{max(740, height)}')
    root.minsize(1080, 740)
    theme.apply(root)
    app.brand_logo = None
    if brand.LOGO.is_file():
        app.brand_logo = tk.PhotoImage(file=str(brand.LOGO))
    try:
        root.iconbitmap(default=str(brand.ICON))
    except tk.TclError:
        if app.brand_logo:
            root.iconphoto(True, app.brand_logo)
    header = ttk.Frame(root, padding=(18, 8))
    header.pack(fill='x')
    if app.brand_logo:
        tk.Label(header, image=app.brand_logo, background=theme.PAPER).pack(side='left', padx=(0, 12))
    ttk.Label(header, text='RPBOT', font=('Bahnschrift', 15, 'bold')).pack(side='left')
    ttk.Label(header, text=' /  STUDIO', style='Mono.TLabel').pack(side='left', padx=(6, 0))
    ttk.Button(header, text='Tác giả  ↗', style='Link.TButton', command=lambda: webbrowser.open(brand.WEBSITE)).pack(side='right')
    app.language_picker = picker(header, app.i18n, app.change_language)
    app.language_picker.pack(side='right', padx=(12, 8))
    app.activity = tk.StringVar(value='Studio tự kết nối khi robot khả dụng')
    ttk.Label(header, textvariable=app.activity, style='Mono.TLabel', font=theme.MONO).pack(side='right', padx=(20, 22))
    line(root)
    body = ttk.Frame(root)
    body.pack(fill='both', expand=True)
    sidebar = ttk.Frame(body, style='Side.TFrame', width=172, padding=(8, 14))
    sidebar.pack(side='left', fill='y')
    sidebar.pack_propagate(False)
    ttk.Label(sidebar, text='ĐIỀU KHIỂN', background=theme.SUNKEN, foreground=theme.INK,
              font=('Segoe UI', 9, 'bold')).pack(anchor='w', padx=10, pady=(0, 12))
    app.sidebar_buttons = {}
    for key, number, title, *_ in PAGES:
        button = ttk.Button(sidebar, text=title, style='Side.TButton',
                            command=lambda target=key: app.tabs.select(app.tab_frames[target]))
        button.pack(fill='x', pady=(0, 5))
        app.sidebar_buttons[key] = button
    credit = tk.Frame(sidebar, background=theme.SUNKEN)
    credit.pack(side='bottom', fill='x', padx=12)
    tk.Label(credit, text='CRAFTED BY', background=theme.SUNKEN, foreground=theme.MUTED,
             font=('Consolas', 8)).pack(anchor='w')
    tk.Label(credit, text=brand.AUTHOR, background=theme.SUNKEN, foreground=theme.INK,
             font=('Segoe UI', 9, 'bold')).pack(anchor='w', pady=(8, 2))
    tk.Label(credit, text='Robotics · Control\nEmbedded Systems', background=theme.SUNKEN, foreground=theme.MUTED,
             font=('Segoe UI', 8), justify='left').pack(anchor='w', pady=(0, 4))
    work = ttk.Frame(body, padding=(18, 12, 18, 12))
    work.pack(side='left', fill='both', expand=True)
    heading = ttk.Frame(work)
    heading.pack(fill='x', pady=(0, 10))
    app.page_title = tk.StringVar(value=PAGES[0][3])
    app.page_subtitle = tk.StringVar(value=PAGES[0][4])
    app.page_number = tk.StringVar(value='WORKSPACE / 01')
    row = ttk.Frame(heading)
    row.pack(fill='x', pady=(0, 4))
    stop_all = action(app, row, 'Dừng tất cả tác vụ robot', lambda: app.stop_robot_tasks('all'), style='Stop.TButton', stop=True)
    stop_all.pack(side='right', padx=(12, 0))
    title_label = ttk.Label(row, textvariable=app.page_title, style='Title.TLabel')
    title_label.pack(side='left', fill='x', expand=True)
    row.bind('<Configure>', lambda event: title_label.configure(wraplength=max(240, event.width-stop_all.winfo_reqwidth()-20)))
    ttk.Label(heading, textvariable=app.page_subtitle, style='Muted.TLabel', font=('Segoe UI', 9)).pack(anchor='w')
    build_status(app, work)
    app.workspace_split = tk.PanedWindow(work, orient='vertical', background=theme.LINE,
                                         borderwidth=0, sashwidth=7, sashrelief='flat', opaqueresize=True,
                                         sashcursor='sb_v_double_arrow')
    app.workspace_split.pack(fill='both', expand=True)
    workspace = ttk.Frame(app.workspace_split)
    app.workspace_split.add(workspace, minsize=145, stretch='always')
    footer = ttk.Frame(app.workspace_split)
    app.workspace_split.add(footer, minsize=90, stretch='never')
    row = ttk.Frame(footer)
    row.pack(fill='x', pady=(10, 8))
    ttk.Label(row, text='NHẬT KÝ THAO TÁC', style='Mono.TLabel').pack(side='left')
    ttk.Label(row, text='Kéo thanh phía trên để chỉnh chiều cao', style='Muted.TLabel', font=('Segoe UI', 9)).pack(side='right')
    app.log = ScrolledText(footer, height=9, font=('Consolas', 10), wrap='word', state='disabled',
                          background=theme.SUNKEN, foreground=theme.INK, relief='flat', borderwidth=0,
                          highlightthickness=0, padx=10, pady=8, insertbackground=theme.INK)
    for tag, level in (('error', 'error'), ('warning', 'warning'), ('success', 'ok')):
        bg, fg, _ = COLORS[level]
        app.log.tag_configure(tag, foreground=fg, background=bg)
    app.log.pack(fill='both', expand=True)
    def initial_split():
        height = app.workspace_split.winfo_height()
        log_height = max(110, min(230, round(height * 0.4)))
        app.workspace_split.sash_place(0, 0, max(145, min(height-90, height-log_height)))
    root.after_idle(initial_split)
    app.tabs = ttk.Notebook(workspace, style='Workspace.TNotebook')
    app.tabs.pack(side='left', fill='both', expand=True)
    app.tab_frames = {}
    app.page_scrolls, contents = {}, {}
    for key, _, title, *_ in PAGES:
        frame = ttk.Frame(app.tabs)
        app.tabs.add(frame, text=title)
        app.tab_frames[key] = frame
        scroll, content = scroll_panel(frame, paper=True, stretch=True)
        scroll.pack(fill='both', expand=True)
        app.page_scrolls[key], contents[key] = scroll, content
    build_mapping(app, contents['mapping'])
    build_navigation(app, contents['navigation'])
    eyebrow(contents['maps'], '03 / THƯ VIỆN BẢN ĐỒ')
    app.build_map_tab(contents['maps'])
    build_tasks(app, contents['tasks'])
    build_debug(app, contents['debug'])
    build_settings(app, contents['settings'])
    build_about(app, contents['about'])
    for scroll in app.page_scrolls.values():
        scroll.bind_wheel()
    def wrap_labels(widget):
        for child in widget.winfo_children():
            if isinstance(child, (tk.Label, ttk.Label)) and int(float(str(child.cget('wraplength') or 0))):
                limit = int(float(str(child.cget('wraplength'))))
                widget.bind('<Configure>', lambda event, label=child, maximum=limit:
                            label.configure(wraplength=max(180, min(maximum, event.width-44))), add='+')
            wrap_labels(child)
    for content in contents.values():
        wrap_labels(content)
    def changed(event=None):
        selected = app.tabs.select()
        for key, number, _, title, subtitle in PAGES:
            active = selected == str(app.tab_frames[key])
            app.sidebar_buttons[key].configure(style='ActiveSide.TButton' if active else 'Side.TButton')
            if active:
                app.page_title.set(title)
                app.page_subtitle.set(subtitle)
                app.page_number.set('WORKSPACE / ' + number)
        app.tab_changed(event)
    app.tabs.bind('<<NotebookTabChanged>>', changed)
    app.sidebar_buttons['mapping'].configure(style='ActiveSide.TButton')
