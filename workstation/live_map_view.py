"""Map view for the lite stream: a small PNG rendered on the Pi, with the robot
pose and its path drawn on top. It sends nothing to the robot itself: with
`on_pick` set, a press-and-drag picks a map pose (x, y, yaw) for the caller."""
import math
import time
import tkinter as tk
from tkinter import ttk


class LiveMapView:
    """Canvas plus a one-line caption; embed it anywhere or use LiveMapWindow."""

    def __init__(self, parent, *, height=None, background="#101014", caption_style=None):
        self.info = tk.StringVar(value="Đang chờ ảnh bản đồ đầu tiên từ Pi…")
        options = {"style": caption_style} if caption_style else {}
        ttk.Label(parent, textvariable=self.info, **options).pack(fill="x", pady=(0, 4))
        self.canvas = tk.Canvas(parent, background=background, highlightthickness=0,
                                **({"height": height} if height else {}))
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda event: self.draw())
        self.map = self.image = None
        self.shown, self.shown_key = None, None
        self.pose = None
        self.received = None
        self.waiting = "Đang chờ ảnh bản đồ từ Pi\n(cập nhật khoảng 2 giây một lần)"
        self.transform = None
        self.markers = {}   # name -> (x, y, yaw, colour) in map metres
        self.on_pick = None  # callback(x, y, yaw) after press + drag + release
        self.drag = None
        self.canvas.bind("<ButtonPress-1>", self.press)
        self.canvas.bind("<B1-Motion>", self.motion)
        self.canvas.bind("<ButtonRelease-1>", self.release)

    def to_map(self, cx, cy):
        left, top, scale, live = self.transform
        column, row = (cx - left) / scale, (cy - top) / scale
        return (live["origin"][0] + column * live["resolution"],
                live["origin"][1] + (live["height"] - row) * live["resolution"])

    def press(self, event):
        if self.on_pick and self.transform:
            self.drag = (event.x, event.y, event.x, event.y)
            self.draw()

    def motion(self, event):
        if self.drag:
            self.drag = self.drag[:2] + (event.x, event.y)
            self.draw()

    def release(self, event):
        if not self.drag:
            return
        x0, y0 = self.drag[:2]
        self.drag = None
        self.draw()
        if math.dist((x0, y0), (event.x, event.y)) <= 6:
            # A heading guessed as 0 put AMCL's laser on the wrong walls (2026-10-20).
            self.info.set("Nhấn rồi KÉO theo hướng mũi xe để chọn hướng — chưa gửi gì.")
            return
        # Canvas y grows downwards, map y upwards.
        yaw = math.atan2(-(event.y - y0), event.x - x0)
        x, y = self.to_map(x0, y0)
        self.on_pick(round(x, 2), round(y, 2), round(yaw, 3))

    def set_marker(self, name, pose=None, colour="#1fa34a"):
        if pose is None:
            self.markers.pop(name, None)
        else:
            self.markers[name] = (*pose, colour)
        self.draw()

    def update(self, live_map=None, pose=None):
        if live_map is not None and live_map is not self.map:
            self.map = live_map
            self.image = tk.PhotoImage(master=self.canvas, data=live_map["png_b64"])
            self.received = time.strftime("%H:%M:%S")
        self.pose = pose
        self.draw()

    def idle(self, message):
        """Show a message while there is no map yet (the last map stays if there is one)."""
        self.waiting = message
        if self.image is None:
            self.draw()
        else:
            self.info.set(message)

    def draw(self):
        canvas = self.canvas
        canvas.delete("all")
        width, height = max(1, canvas.winfo_width()), max(1, canvas.winfo_height())
        if self.image is None:
            self.transform = None
            canvas.create_text(width / 2, height / 2, text=self.waiting,
                               fill="#aaa9ad", font=("Segoe UI", 10), justify="center", width=max(80, width - 30))
            return
        image_w, image_h = self.image.width(), self.image.height()
        ratio = min(width / image_w, height / image_h)
        # Tk scales images by integer factors only.
        if ratio >= 1:
            factor = max(1, min(6, int(ratio)))
            key, scale = ("zoom", factor), factor
        else:
            factor = math.ceil(1 / ratio)
            key, scale = ("subsample", factor), 1 / factor
        if self.shown_key != (key, id(self.image)):
            self.shown = self.image.zoom(factor) if key[0] == "zoom" else self.image.subsample(factor)
            self.shown_key = (key, id(self.image))
        left, top = (width - image_w * scale) / 2, (height - image_h * scale) / 2
        canvas.create_image(left, top, anchor="nw", image=self.shown)
        live = self.map
        self.transform = (left, top, scale, live)
        def to_canvas(x, y):
            column = (x - live["origin"][0]) / live["resolution"]
            row = live["height"] - (y - live["origin"][1]) / live["resolution"]
            return left + column * scale, top + row * scale
        for x, y, yaw, colour in self.markers.values():
            cx, cy = to_canvas(x, y)
            canvas.create_oval(cx - 7, cy - 7, cx + 7, cy + 7, outline=colour, width=3)
            canvas.create_line(cx, cy, cx + 20 * math.cos(yaw), cy - 20 * math.sin(yaw),
                               fill=colour, width=3, arrow="last")
        if self.drag:
            x0, y0, x1, y1 = self.drag
            canvas.create_oval(x0 - 6, y0 - 6, x0 + 6, y0 + 6, outline="#e0a100", width=3)
            canvas.create_line(x0, y0, x1, y1, fill="#e0a100", width=3, arrow="last")
        points = [coordinate for x, y in live.get("path", []) for coordinate in to_canvas(x, y)]
        if len(points) >= 4:
            canvas.create_line(*points, fill="#4aa3ff", width=2)
        text = (f"Bản đồ {live['width']}×{live['height']} điểm ảnh · {live['resolution']:.2f} m/điểm"
                f" · ảnh lúc {self.received}")
        if self.pose:
            x, y = to_canvas(self.pose["x"], self.pose["y"])
            yaw = self.pose["yaw"]
            canvas.create_line(x, y, x + 16 * math.cos(yaw), y - 16 * math.sin(yaw),
                               fill="#ff5a36", width=3, arrow="last")
            canvas.create_oval(x - 5, y - 5, x + 5, y + 5, fill="#ff5a36", outline="")
            text += f" · robot x={self.pose['x']:+.2f} y={self.pose['y']:+.2f} m"
        else:
            text += " · chưa có vị trí robot (TF map→base_footprint)"
        self.info.set(text)


class LiveMapWindow:
    def __init__(self, root, on_close=None):
        self.window = tk.Toplevel(root)
        self.window.title("RPBot · Bản đồ đang dựng (4G tiết kiệm)")
        self.window.geometry("760x640")
        self.window.configure(background="#101014")
        frame = ttk.Frame(self.window, padding=10)
        frame.pack(fill="both", expand=True)
        self.view = LiveMapView(frame)
        self.info = self.view.info
        self.window.protocol("WM_DELETE_WINDOW", self.close)
        self.on_close = on_close
        self.closed = False

    def raise_(self):
        self.window.deiconify()
        self.window.lift()

    def update(self, live_map=None, pose=None):
        self.view.update(live_map, pose)

    def draw(self):
        self.view.draw()

    def close(self):
        self.closed = True
        self.window.destroy()
        if self.on_close:
            self.on_close()
