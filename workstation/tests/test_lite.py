import base64
import json
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch
import zlib

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import rpbot_console
from lite_stream import render_occupancy
from map_library import export_bundle
from remote_ops import classify_uplink
from stream_client import TelemetryStream

ROUTES = ("default via 192.168.31.1 dev wlan0 proto dhcp src 192.168.31.105 metric 150\n"
          "default via 192.168.42.129 dev enx020b54543730 proto dhcp src 192.168.42.152 metric 200\n")
DRIVERS = {"wlan0": "brcmfmac", "enx020b54543730": "rndis_host", "eth0": "macb"}


def png_pixels(png):
    """Decode the 8-bit grayscale, filter-0 PNGs written by png_gray."""
    width, height = int.from_bytes(png[16:20], "big"), int.from_bytes(png[20:24], "big")
    start = png.index(b"IDAT") + 4
    length = int.from_bytes(png[start - 8:start - 4], "big")
    raw = zlib.decompress(png[start:start + length])
    return [list(raw[row * (width + 1) + 1:(row + 1) * (width + 1)]) for row in range(height)]


class UplinkTests(unittest.TestCase):
    def test_lowest_metric_route_decides_and_usb_modem_is_cellular(self):
        self.assertEqual(classify_uplink(ROUTES, DRIVERS.get), {"interface": "wlan0", "kind": "wifi"})
        only_4g = ROUTES.splitlines()[1]
        self.assertEqual(classify_uplink(only_4g, DRIVERS.get)["kind"], "cellular")
        self.assertEqual(classify_uplink("default via 10.0.0.1 dev eth0 metric 100", DRIVERS.get)["kind"], "lan")
        self.assertEqual(classify_uplink("", DRIVERS.get), {"interface": None, "kind": "none"})


class RenderTests(unittest.TestCase):
    def test_occupied_cell_survives_downsampling_and_rows_are_flipped(self):
        width, height = 8, 4
        cells = [-1] * (width * height)
        cells[0] = 100          # bottom-left cell occupied
        cells[width * 3 + 7] = 0  # top-right cell free
        png, step = render_occupancy([value & 0xff for value in cells], width, height, max_side=4)
        self.assertEqual(step, 2)
        pixels = png_pixels(png)
        self.assertEqual((len(pixels), len(pixels[0])), (2, 4))
        self.assertEqual(pixels[1][0], 0)    # occupied, now in the bottom PNG row
        self.assertEqual(pixels[0][3], 254)  # free, top PNG row
        self.assertEqual(pixels[0][0], 205)  # unknown


class SyncTests(unittest.TestCase):
    def bundle(self, name, image):
        info = dict(revision="rev-" + image.decode())
        return export_bundle((info, None, None, {"image": "x.pgm", "resolution": 0.05,
                                                 "origin": [0, 0, 0]}, image), name)

    def test_only_new_or_changed_maps_are_downloaded(self):
        maps = [dict(name="autosave_20261020_1123", revision="rev-A"), dict(name="room_1", revision="rev-B")]
        exports = []
        def remote(settings, action, *args, **kwargs):
            if action == "maps":
                return json.dumps({"maps": maps})
            name = args[args.index("--name") + 1]
            exports.append(name)
            return json.dumps(self.bundle(name, b"A" if name.startswith("autosave") else b"B"))
        with tempfile.TemporaryDirectory() as directory, patch.object(rpbot_console, "remote", remote):
            first = rpbot_console.download_changed_maps({}, directory)
            self.assertEqual(first, ["autosave_20261020_1123", "room_1"])
            self.assertTrue((Path(directory) / "room_1" / "room_1.pgm").is_file())
            self.assertEqual(rpbot_console.download_changed_maps({}, directory), [])
            maps[0]["revision"] = "rev-C"
            def changed(settings, action, *args, **kwargs):
                if action == "maps":
                    return json.dumps({"maps": maps})
                return json.dumps(dict(self.bundle(args[args.index("--name") + 1], b"C")))
            with patch.object(rpbot_console, "remote", changed):
                self.assertEqual(rpbot_console.download_changed_maps({}, directory), ["autosave_20261020_1123"])
            image = (Path(directory) / "autosave_20261020_1123" / "autosave_20261020_1123.pgm").read_bytes()
            self.assertEqual(image, b"C")


class BundleTests(unittest.TestCase):
    def test_bundle_carries_every_helper_needed_on_the_pi(self):
        payload, revision = rpbot_console.ops_bundle()
        files = json.loads(base64.b64decode(payload))
        self.assertEqual(set(files), set(rpbot_console.OPS_FILES))
        self.assertEqual(len(revision), 64)


class StreamLiveMapTests(unittest.TestCase):
    def test_last_map_image_is_kept_while_later_frames_have_none(self):
        with tempfile.TemporaryDirectory() as directory:
            code = ("import json,sys; print(json.dumps({'topics': {}, 'live_map': {'png_b64': 'x'}}), flush=True);"
                    "print(json.dumps({'topics': {}, 'pose': {'x': 1}}), flush=True); sys.stdin.readline()")
            stream = TelemetryStream([sys.executable, "-u", "-c", code], Path(directory) / "s.log")
            try:
                deadline = time.monotonic() + 3
                while stream.frames < 2 and time.monotonic() < deadline:
                    time.sleep(0.01)
                data, _, _ = stream.snapshot()
                self.assertNotIn("live_map", data)
                self.assertEqual(stream.live(), ({"png_b64": "x"}, 1))
            finally:
                stream.close()
                stream.reader.join(timeout=2)


if __name__ == "__main__":
    unittest.main()


class ExploreTests(unittest.TestCase):
    def test_exploration_needs_one_mapping_session_and_runs_as_a_collected_user_unit(self):
        import subprocess
        import remote_ops
        with tempfile.TemporaryDirectory() as directory:
            root, workspace, home = Path(directory, "slam"), Path(directory, "ws"), Path(directory, "home")
            (root / "src/pi5_slam_bringup/launch").mkdir(parents=True)
            (root / "src/pi5_slam_bringup/launch/explore.launch.py").write_text("")
            (workspace / "install").mkdir(parents=True)
            (workspace / "install/setup.bash").write_text("")
            calls = []
            def run(args, **kwargs):
                calls.append(args)
                return subprocess.CompletedProcess(args, 3 if "is-active" in args else 0, "inactive\n" if "is-active" in args else "", "")
            mapping = {"domain": 0, "mode": "mapping", "simulated_time": False, "pid": 1}
            with patch.object(remote_ops, "ROOT", root), patch.object(remote_ops, "EXPLORE_WS", workspace), \
                 patch.object(remote_ops.Path, "home", return_value=home), patch.object(remote_ops, "run", run):
                with patch.object(remote_ops, "launches", return_value=[]):
                    with self.assertRaisesRegex(RuntimeError, "Mapping"):
                        remote_ops.start_explore(0, False)
                with patch.object(remote_ops, "launches", return_value=[mapping]):
                    self.assertEqual(remote_ops.start_explore(0, True)["explore"], "started")
            unit = next(args for args in calls if args[0] == "systemd-run")
            self.assertIn("--collect", unit)
            self.assertIn("--setenv=ROS_DOMAIN_ID=0", unit)
            self.assertEqual(unit[-1], "true")
            self.assertIn("explore.launch.py", (home / ".local/share/rpbot-console/explore.sh").read_text())
