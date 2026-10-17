"""Retry timing for SSH discovery and the PC subscriber; no robot operations."""
from dataclasses import dataclass


@dataclass
class ConnectionState:
    next_check: float = 0.0
    failures: int = 0
    last_error: str = ''
    last_error_log: float = 0.0
    stream_started: float = 0.0
    stream_retry_at: float = 0.0
    stream_failures: int = 0
    stream_recovering: bool = False

    def metadata_ok(self, now, *, has_session):
        recovered = self.failures > 0
        self.failures = 0
        self.last_error = ''
        self.next_check = now + (30 if has_session else 5)
        return recovered

    def metadata_failed(self, now, error):
        log_error = self.failures == 0 or now - self.last_error_log >= 60
        self.failures += 1
        self.last_error = error
        self.next_check = now + min(2 ** min(self.failures, 4), 15)
        if log_error:
            self.last_error_log = now
        return log_error

    def reset_stream(self):
        self.stream_started = self.stream_retry_at = 0.0
        self.stream_failures = 0
        self.stream_recovering = False

    def stream_attempt(self, now):
        self.stream_started = now
        self.stream_failures += 1
        self.stream_retry_at = now + min(2 ** min(self.stream_failures, 5), 30)

    def stream_received(self):
        recovered = self.stream_recovering
        self.stream_failures = 0
        self.stream_recovering = False
        return recovered

    def restart_due(self, now, *, alive, updated):
        stalled = (not alive or now - (updated or self.stream_started) >= (8 if updated else 15))
        return stalled and now >= self.stream_retry_at
