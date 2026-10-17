from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from stream_client import TelemetryStream
from rpbot_console import Console


class StreamTests(unittest.TestCase):
    def test_snapshots_are_coalesced_and_stop_closes_only_own_worker(self):
        with tempfile.TemporaryDirectory() as directory:
            code = "import json,sys; [print(json.dumps({'topics': {}, 'sequence': i}),flush=True) for i in range(20)]; sys.stdin.readline()"
            stream = TelemetryStream([sys.executable, "-u", "-c", code], Path(directory) / "stream.log")
            try:
                deadline = time.monotonic() + 3
                while stream.frames < 20 and time.monotonic() < deadline:
                    time.sleep(0.01)
                data, updated, frames = stream.snapshot()
                self.assertEqual(frames, 20)
                self.assertEqual(data["sequence"], 19)
                self.assertGreater(updated, 0)
                self.assertTrue(stream.alive())
            finally:
                stream.close()
                stream.reader.join(timeout=2)
            self.assertFalse(stream.alive())

    def test_dead_stream_immediately_marks_data_stale(self):
        app = Console.__new__(Console)
        app.stream = Mock()
        app.stream.snapshot.return_value = ({}, time.monotonic(), 0)
        app.stream.alive.return_value = False
        app.stream_seen = 0
        app.session = {"pid": 1}
        app.last_check = 0
        app.last_data = {"motor_status": 16}
        app.status = {key: Mock() for key in ("connection", "mapping", "sensors", "motion", "navigation")}
        app.viewers = {}
        app.auto, app.root = Mock(), Mock()
        app.auto.get.return_value = False
        app.closed = False
        app.tick()
        self.assertIsNone(app.last_data)
        app.status["motion"].set.assert_called_once_with("Dữ liệu đã cũ")
        app.root.after.assert_called_once_with(100, app.tick)


if __name__ == "__main__":
    unittest.main()
