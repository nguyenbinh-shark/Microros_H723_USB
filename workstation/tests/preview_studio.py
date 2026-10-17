"""Offline layout review and native-window captures; never connects to the Pi."""
import ctypes
from ctypes import wintypes
from pathlib import Path
import sys
import tempfile
import time
import argparse
from unittest.mock import patch
from tkinter import font as tkfont, ttk

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import rpbot_console as console
from console_layout import PAGES
import branding


def check_vietnamese_fonts(root):
    """Inspect native glyph coverage, so fallback fonts cannot hide a missing accent."""
    vietnamese = 'ĐđĂăÂâÊêÔôƠơƯư' + ''.join(chr(code) for code in range(0x1EA0, 0x1EFA))
    styles, fonts = ttk.Style(root), set()
    def visit(widget):
        if 'text' in widget.keys() and 'font' in widget.keys():
            text = str(widget.cget('text'))
            if 'textvariable' in widget.keys() and widget.cget('textvariable'):
                text += str(root.getvar(widget.cget('textvariable')))
            if any(char in vietnamese for char in text):
                spec = str(widget.cget('font'))
                if not spec:
                    style = widget.cget('style') or widget.winfo_class()
                    spec = styles.lookup(style, 'font')
                actual = tkfont.Font(root=root, font=spec).actual()
                fonts.add((actual['family'], actual['weight'], actual['slant']))
        for child in widget.winfo_children():
            visit(child)
    visit(root)
    gdi, handle = ctypes.windll.gdi32, wintypes.HANDLE
    gdi.CreateCompatibleDC.argtypes, gdi.CreateCompatibleDC.restype = [handle], handle
    gdi.CreateFontW.argtypes = [ctypes.c_int]*5 + [wintypes.DWORD]*8 + [wintypes.LPCWSTR]
    gdi.CreateFontW.restype = handle
    gdi.SelectObject.argtypes, gdi.SelectObject.restype = [handle, handle], handle
    gdi.GetGlyphIndicesW.argtypes = [handle, wintypes.LPCWSTR, ctypes.c_int, ctypes.POINTER(wintypes.WORD), wintypes.DWORD]
    gdi.DeleteObject.argtypes = gdi.DeleteDC.argtypes = [handle]
    dc = gdi.CreateCompatibleDC(None)
    try:
        for family, weight, slant in sorted(fonts):
            font = gdi.CreateFontW(-30, 0, 0, 0, 700 if weight == 'bold' else 400,
                                   int(slant == 'italic'), 0, 0, 1, 0, 0, 0, 0, family)
            previous = gdi.SelectObject(dc, font)
            try:
                glyphs = (wintypes.WORD * len(vietnamese))()
                result = gdi.GetGlyphIndicesW(dc, vietnamese, len(vietnamese), glyphs, 1)
                missing = ''.join(char for char, glyph in zip(vietnamese, glyphs) if glyph == 0xffff)
                assert result != -1 and not missing, (family, weight, slant, 'Missing Vietnamese glyphs', missing)
            finally:
                gdi.SelectObject(dc, previous)
                gdi.DeleteObject(font)
    finally:
        gdi.DeleteDC(dc)
    print('Vietnamese glyph coverage OK:', sorted(fonts))


def capture_window(root, path):
    from PIL import Image
    user, gdi = ctypes.windll.user32, ctypes.windll.gdi32
    handle = wintypes.HANDLE
    user.GetAncestor.argtypes, user.GetAncestor.restype = [handle, wintypes.UINT], handle
    user.GetWindowDC.argtypes, user.GetWindowDC.restype = [handle], handle
    user.ReleaseDC.argtypes = [handle, handle]
    user.PrintWindow.argtypes = [handle, handle, wintypes.UINT]
    gdi.CreateCompatibleDC.argtypes, gdi.CreateCompatibleDC.restype = [handle], handle
    gdi.CreateCompatibleBitmap.argtypes, gdi.CreateCompatibleBitmap.restype = [handle, ctypes.c_int, ctypes.c_int], handle
    gdi.SelectObject.argtypes, gdi.SelectObject.restype = [handle, handle], handle
    gdi.DeleteObject.argtypes = [handle]
    gdi.DeleteDC.argtypes = [handle]
    gdi.GetDIBits.argtypes = [handle, handle, wintypes.UINT, wintypes.UINT, ctypes.c_void_p, ctypes.c_void_p, wintypes.UINT]
    hwnd = user.GetAncestor(root.winfo_id(), 2)
    rect = wintypes.RECT()
    user.GetWindowRect(hwnd, ctypes.byref(rect))
    width, height = rect.right-rect.left, rect.bottom-rect.top
    dc = user.GetWindowDC(hwnd)
    memory = gdi.CreateCompatibleDC(dc)
    bitmap = gdi.CreateCompatibleBitmap(dc, width, height)
    previous = gdi.SelectObject(memory, bitmap)
    try:
        if not user.PrintWindow(hwnd, memory, 2):
            raise RuntimeError('Windows could not capture the studio window')
        class BitmapHeader(ctypes.Structure):
            _fields_ = [('size', wintypes.DWORD), ('width', wintypes.LONG), ('height', wintypes.LONG),
                        ('planes', wintypes.WORD), ('bits', wintypes.WORD), ('compression', wintypes.DWORD),
                        ('image_size', wintypes.DWORD), ('x', wintypes.LONG), ('y', wintypes.LONG),
                        ('used', wintypes.DWORD), ('important', wintypes.DWORD)]
        header = BitmapHeader()
        header.size, header.width, header.height = ctypes.sizeof(header), width, -height
        header.planes, header.bits = 1, 32
        raw = ctypes.create_string_buffer(width*height*4)
        if not gdi.GetDIBits(memory, bitmap, 0, height, raw, ctypes.byref(header), 0):
            raise RuntimeError('Windows returned no window pixels')
        image = Image.frombytes('RGB', (width, height), raw.raw, 'raw', 'BGRX')
        image.save(path)
        return len(image.getcolors(width*height) or []) > 10
    finally:
        gdi.SelectObject(memory, previous)
        gdi.DeleteObject(bitmap)
        gdi.DeleteDC(memory)
        user.ReleaseDC(hwnd, dc)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--states', action='store_true', help='Also capture clearly labelled synthetic telemetry states')
    args = parser.parse_args()
    output = console.STATE / 'studio-preview'
    output.mkdir(parents=True, exist_ok=True)
    errors, clipped = [], []
    with tempfile.TemporaryDirectory() as folder, patch.object(console, 'STATE', Path(folder)), \
         patch.object(console.Console, 'check'), patch.object(console.Console, 'tick'), \
         patch.object(console.Console, 'refresh_tasks'), patch.object(console.Console, 'refresh_maps'), \
         patch.object(console.Console, 'preview_map'):
        root = console.create_root()
        app = console.Console(root)
        root.report_callback_exception = lambda kind, error, trace: errors.append(str(error))
        app.activity.set('Hệ thống sẵn sàng · đang ngoại tuyến')
        app.status['connection'].set('Pi hiện chưa kết nối. Studio tự nối lại khi robot khả dụng.')
        app.status['mapping'].set('Chưa có phiên robot được xác nhận')
        app.status['sensors'].set('Chờ dữ liệu LiDAR / odom / IMU')
        app.status['motion'].set('Chưa có dữ liệu motor / vận tốc')
        app.status['navigation'].set('Chưa kiểm tra Nav2')
        app.status['boot'].set('Chưa kiểm tra cấu hình trên Pi')
        app.append('Studio đang ngoại tuyến. Các chức năng robot sẽ khả dụng sau khi kết nối Pi.')
        root.update()
        check_vietnamese_fonts(root)
        captures = []
        for size in ('1420x920', '1080x740'):
            root.geometry(size)
            root.update()
            for key, number, _, title, _ in PAGES:
                app.sidebar_buttons[key].invoke()
                root.update()
                assert app.tabs.select() == str(app.tab_frames[key])
                assert app.page_title.get() == title
                assert app.sidebar_buttons[key].cget('style') == 'ActiveSide.TButton'
                global_stop = next(button for button in app.stop_buttons if button.cget('text') == 'Dừng tất cả tác vụ robot')
                assert global_stop.winfo_ismapped(), ('Global stop is hidden', size, key)
                assert global_stop.winfo_width() >= global_stop.winfo_reqwidth()-2, ('Global stop is clipped', size, key)
                page = app.page_scrolls[key]
                page.canvas.yview_moveto(0)
                root.update()
                for button in app.actions + app.stop_buttons:
                    if button.winfo_ismapped() and str(page.content) in str(button):
                        if button.winfo_width() < button.winfo_reqwidth()-2:
                            clipped.append((size, key, button.cget('text'), button.winfo_width(), button.winfo_reqwidth()))
                filename = output / f'{key}-{size}.png'
                captures.append((filename.name, capture_window(root, filename)))
        assert app.brand_logo is not None
        assert branding.AUTHOR in root.title()
        if args.states:
            from test_robot_status import healthy_data
            root.geometry('1420x920')
            app.tabs.select(app.tab_frames['navigation'])
            app.session = dict(pid=123, mode='navigation', domain=0)
            app.connected_settings = dict(app.settings)
            app.last_meta = time.monotonic()
            app.activity.set('KIỂM THỬ GIAO DIỆN · DỮ LIỆU GIẢ')
            root.title('KIỂM THỬ GIAO DIỆN · DỮ LIỆU GIẢ · RPBot Studio')
            for scenario in ('receiving', 'waiting-pose', 'can-fault', 'stale-odom', 'lost-stream', 'offline'):
                app.session = dict(pid=123, mode='navigation', domain=0)
                app.last_meta = time.monotonic()
                app.connection.failures = 0
                data = healthy_data()
                if scenario == 'waiting-pose':
                    data['tf_map_to_base'] = data['mapping_ready'] = False
                elif scenario == 'can-fault':
                    data['motor_status'] = 0x17
                elif scenario == 'stale-odom':
                    data['topics']['/odom']['age_s'] = 5
                if scenario in ('lost-stream', 'offline'):
                    app.last_data = None
                    if scenario == 'offline':
                        app.session = None
                        app.connection.failures = 1
                    app.dashboard.refresh()
                else:
                    app.render_data(data)
                root.update()
                filename = output / f'status-fixture-{scenario}.png'
                captures.append((filename.name, capture_window(root, filename)))
        assert not errors, errors
        app.close()
    print('Layout warnings:', clipped)
    print('Captures:', captures)
    print('PASS: all seven sidebar pages, titles, active navigation, logo and offline state; no Pi calls')
    print('Output:', output)
    if clipped:
        raise SystemExit('Some controls need more horizontal space')


if __name__ == '__main__':
    main()
