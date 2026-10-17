"""High contrast status widgets, driven by the existing console telemetry."""
import tkinter as tk
from tkinter import ttk
from tkinter import font as tkfont

from robot_status import summarize
import studio_theme as theme

COLORS = {
    'ok': ('#dcfce7', '#14532d', '#22a55a'),
    'warning': ('#fff3cd', '#7a4300', '#d99400'),
    'error': ('#fee2e2', '#991b1b', '#dc2626'),
    'info': ('#dbeafe', '#163fa0', '#3b82f6'),
    'unknown': ('#e5e7eb', '#374151', '#9ca3af'),
}


class StatusCard(tk.Frame):
    def __init__(self, parent, title, language):
        super().__init__(parent, background=theme.PANEL, highlightthickness=2, highlightbackground=theme.LINE,
                         padx=9, pady=6)
        self.header = tk.Frame(self, background=theme.PANEL)
        self.header.pack(fill='x')
        self.badge = tk.Label(self.header, text='CHỜ', font=('Segoe UI', 8, 'bold'), padx=4, pady=1)
        self.badge.pack(side='right')
        tk.Label(self.header, text=title, background=theme.PANEL, foreground=theme.INK,
                 font=('Segoe UI', 9, 'bold')).pack(side='left')
        self.value = tk.Label(self, background=theme.PANEL, foreground=theme.INK,
                              font=('Segoe UI', 12, 'bold'), anchor='w', justify='left', height=1)
        self.value.pack(fill='x', pady=(4, 2))
        self.detail = tk.Label(self, background=theme.PANEL, foreground='#374151',
                               font=('Segoe UI', 9), anchor='w', justify='left', height=1)
        self.detail.pack(fill='x')
        self.last = None
        self.language = language
        self.bind('<Configure>', lambda event: self.fit_detail())

    def fit_detail(self):
        if self.last is None:
            return
        text = self.language.text(self.last.detail)
        font = tkfont.Font(root=self, font=self.detail.cget('font'))
        width = max(80, self.winfo_width()-24)
        if font.measure(text) > width:
            while text and font.measure(text + '…') > width:
                text = text[:-1]
            text = text.rstrip() + '…'
        self.detail.configure(text=text)

    def show(self, state):
        if state == self.last:
            return
        self.last = state
        bg, fg, border = COLORS[state.level]
        self.configure(highlightbackground=border)
        self.badge.configure(text=state.badge, background=bg, foreground=fg)
        self.value.configure(text=state.value, foreground=fg)
        self.detail.configure(text=state.detail)
        self.fit_detail()


class StatusDashboard(ttk.Frame):
    def __init__(self, app, parent):
        super().__init__(parent)
        self.app, self.pending = app, False
        self.cards, self.sensor_widgets = {}, {}
        bar = ttk.Frame(self)
        bar.pack(fill='x', pady=(0, 7))
        ttk.Label(bar, text='TRẠNG THÁI ROBOT', font=('Segoe UI', 10, 'bold')).pack(side='left')
        button = app.button(bar, 'Kết nối / Kiểm tra', app.check)
        button.configure(style='Compact.TButton')
        button.pack(side='right')
        app.auto = tk.BooleanVar(value=True)
        ttk.Checkbutton(bar, text='Tự kết nối / phục hồi', variable=app.auto).pack(side='right', padx=(0, 12))
        row = ttk.Frame(self)
        row.pack(fill='x')
        for col, (key, title) in enumerate((('connection', 'KẾT NỐI'), ('mapping', 'CHẾ ĐỘ'),
                                           ('navigation', 'NAV2'), ('motion', 'MOTOR'))):
            row.columnconfigure(col, weight=1, uniform='cards')
            card = StatusCard(row, title, app.i18n)
            card.grid(row=0, column=col, sticky='nsew', padx=(0 if col == 0 else 7, 0))
            self.cards[key] = card
        lower = ttk.Frame(self)
        lower.pack(fill='x', pady=(7, 7))
        sensors = ttk.Frame(lower)
        sensors.pack(side='left', fill='x', expand=True)
        for col, name in enumerate(('LiDAR', 'Odom', 'IMU', 'Motor FB')):
            sensors.columnconfigure(col, weight=1, uniform='sensor')
            frame = tk.Frame(sensors, background='#e5e7eb', padx=8, pady=4)
            frame.grid(row=0, column=col, sticky='nsew', padx=(0, 6))
            title = tk.Label(frame, text=name, background='#e5e7eb', foreground=theme.INK,
                             font=('Segoe UI', 10, 'bold'), anchor='w')
            title.pack(anchor='w')
            value = tk.Label(frame, text='CHƯA CÓ', font=('Segoe UI', 9, 'bold'), anchor='w')
            value.pack(anchor='w')
            self.sensor_widgets[name] = (frame, title, value)
        motion = tk.Frame(lower, background=theme.INK, padx=13, pady=7)
        motion.pack(side='right', padx=(7, 0))
        self.linear = tk.StringVar(value='—')
        self.angular = tk.StringVar(value='—')
        for title, value, unit in (('v', self.linear, 'm/s'), ('ω', self.angular, 'rad/s')):
            tk.Label(motion, text=title, background=theme.INK, foreground='#ffffff',
                     font=('Segoe UI', 10, 'bold')).pack(side='left', padx=(0, 5))
            tk.Label(motion, textvariable=value, background=theme.INK, foreground='#ffffff',
                     font=('Consolas', 17, 'bold'), width=6, anchor='e').pack(side='left')
            tk.Label(motion, text=unit, background=theme.INK, foreground='#ffffff',
                     font=('Segoe UI', 9)).pack(side='left', padx=(4, 12))
        self.notice = tk.Label(self, text='', font=('Segoe UI', 10, 'bold'), anchor='w', justify='left', padx=10, pady=6)
        self.notice.pack(fill='x')
        self.notice.bind('<Configure>', lambda event: self.notice.configure(wraplength=max(300, event.width-24)))
        # Keep the original detailed fields for diagnostics and existing callbacks.
        app.status = {key: tk.StringVar(value='Chưa kiểm tra') for key in
                      ('connection', 'mapping', 'sensors', 'motion', 'navigation', 'boot')}
        for value in app.status.values():
            value.trace_add('write', self.schedule)
        app.activity.trace_add('write', self.schedule)
        self.refresh()

    def schedule(self, *_):
        if not self.pending and not self.app.closed:
            self.pending = True
            self.after_idle(self.refresh)

    def refresh(self):
        self.pending = False
        app = self.app
        if app.closed:
            return
        settings = app.connected_settings or app.settings
        state = summarize(app.session, app.last_data,
                          ssh_ok=bool(app.last_meta and not app.connection.failures),
                          ssh_failed=bool(app.connection.failures), connecting=app.connecting,
                          host=settings.get('host', ''))
        for key, card in self.cards.items():
            card.show(state.cards[key])
        for name, widgets in self.sensor_widgets.items():
            item = state.sensors[name]
            bg, fg, _ = COLORS[item.level]
            frame, title, value = widgets
            frame.configure(background=bg)
            title.configure(background=bg, foreground=fg)
            value.configure(background=bg, foreground=fg, text=item.badge)
        self.linear.set(state.linear)
        self.angular.set(state.angular)
        bg, fg, _ = COLORS[state.notice_level]
        notice = state.notice
        if state.notice_level != 'error' and (app.busy or app.stop_busy):
            bg, fg, _ = COLORS['info']
            notice = app.activity.get()
        self.notice.configure(text=notice, background=bg, foreground=fg)
