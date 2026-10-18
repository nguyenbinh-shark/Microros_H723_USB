"""Keep one PC-side ROS subscriber alive; coalesce snapshots so the GUI never queues old frames."""
import json
import subprocess
import threading
import time


class TelemetryStream:
    def __init__(self, command, log_path, *, creationflags=0):
        self.latest = None
        self.updated = 0.0
        self.frames = 0
        self.lock = threading.Lock()
        self.log = open(log_path, "a", encoding="utf-8")
        self.process = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        stderr=self.log, text=True, encoding="utf-8",
                                        errors="replace", bufsize=1, creationflags=creationflags)
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self):
        try:
            for line in self.process.stdout:
                try:
                    snapshot = json.loads(line)
                    if not isinstance(snapshot, dict) or "topics" not in snapshot:
                        continue
                except ValueError:
                    continue
                with self.lock:
                    self.latest, self.updated = snapshot, time.monotonic()
                    self.frames += 1
        finally:
            self.process.stdout.close()
            self.log.close()

    def snapshot(self):
        with self.lock:
            return self.latest, self.updated, self.frames

    def alive(self):
        return self.process.poll() is None

    def close(self):
        try:
            if self.process.poll() is None:
                self.process.stdin.write("stop\n")
                self.process.stdin.flush()
                self.process.wait(timeout=4)
        except (OSError, ValueError, subprocess.TimeoutExpired):
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2)
        finally:
            self.process.stdin.close()
