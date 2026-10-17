"""Live visual check: subscribes to robot data and optionally opens mapping RViz."""
import argparse
from pathlib import Path
import sys
import time
import tkinter as tk
from datetime import datetime
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rpbot_console import Console, STATE, create_root


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--open-rviz", action="store_true")
    parser.add_argument("--maps", action="store_true")
    parser.add_argument("--download-check", action="store_true")
    parser.add_argument("--reconnect-check", action="store_true")
    parser.add_argument("--navigation", action="store_true", help="Inspect live Nav2 without sending a goal")
    args = parser.parse_args()
    root = create_root()
    app = Console(root)
    app.auto.set(False)
    deadline = time.monotonic() + 50
    opened = None
    downloaded = None
    download_frames = None
    reconnect_pid = None
    result = {"ok": False}
    def callback_error(kind, error, trace):
        import traceback
        traceback.print_exception(kind, error, trace)
        result["error"] = str(error)
        app.close()
    root.report_callback_exception = callback_error

    def poll():
        nonlocal opened, downloaded, download_frames, reconnect_pid
        if time.monotonic() > deadline:
            print("Timed out:", app.activity.get(), app.map_info.get(), flush=True)
            app.close()
            return
        ready = app.last_data and (app.last_data.get("navigation_available") if args.navigation else app.last_data["mapping_ready"])
        if ready and app.stream and app.stream.frames >= 15:
            if args.navigation:
                assert app.session["mode"] == "navigation"
                app.tabs.select(app.tab_frames["navigation"])
            if args.reconnect_check and reconnect_pid is None:
                reconnect_pid = app.stream.process.pid
                print("Stopping only the PC subscriber", reconnect_pid, flush=True)
                app.stream.close()
                app.tick()
                assert app.last_data is None
                assert "Mất luồng" in app.status["connection"].get()
                app.check()
                root.after(200, poll)
                return
            if args.reconnect_check:
                assert app.stream.process.pid != reconnect_pid
                print("ROS subscriber restarted; robot session left running", flush=True)
            if args.maps or args.download_check:
                if not app.preview_record or app.busy:
                    root.after(200, poll)
                    return
                app.tabs.select(app.tab_frames["maps"])
            if args.download_check and downloaded is None:
                directory = STATE / "verification-downloads" / datetime.now().strftime("%Y%m%d_%H%M%S")
                directory.mkdir(parents=True)
                downloaded = directory / app.preview_record["name"]
                download_frames = app.stream.frames
                with patch("rpbot_console.filedialog.askdirectory", return_value=str(directory)):
                    app.download_map()
                root.after(200, poll)
                return
            if args.download_check:
                if app.busy:
                    root.after(200, poll)
                    return
                assert len(list(downloaded.glob("*.pgm"))) == 1
                assert len(list(downloaded.glob("*.yaml"))) == 1
                assert app.stream.frames > download_frames
                print("Map download verified while telemetry continued:", downloaded)
            if args.open_rviz and opened is None:
                app.open_view("rviz")
                opened = time.monotonic()
            if not args.open_rviz or time.monotonic() - opened > 6:
                if args.open_rviz and ("rviz" not in app.viewers or app.viewers["rviz"].poll() is not None):
                    app.close()
                    return
                root.update_idletasks()
                root.lift()
                root.attributes("-topmost", True)
                root.update()
                try:
                    from PIL import ImageGrab
                    box = (root.winfo_rootx(), root.winfo_rooty(),
                           root.winfo_rootx() + root.winfo_width(),
                           root.winfo_rooty() + root.winfo_height())
                    ImageGrab.grab(bbox=box).save(STATE / ("console-nav-preview.png" if args.navigation else
                                                          "console-maps-preview.png" if args.maps else "console-preview.png"))
                except ImportError:
                    pass
                print("GUI connected; mapping data and status labels OK")
                print("Continuous snapshots:", app.stream.frames)
                for key, value in app.status.items():
                    print(key + ": " + value.get())
                result["ok"] = True
                app.close()
                return
        if time.monotonic() > deadline:
            app.close()
            return
        root.after(200, poll)

    root.after(200, poll)
    root.mainloop()
    if not result["ok"]:
        raise SystemExit("GUI smoke check did not receive healthy mapping data")


if __name__ == "__main__":
    main()
