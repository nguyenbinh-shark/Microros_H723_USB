"""Read-only hostname resolution and ICMP diagnostics for the PC console."""
from dataclasses import dataclass
import ipaddress
import os
import re
import socket
import subprocess


def resolve_ipv4(host, fallback=None):
    """Prefer the Windows resolver; optionally try the configured WSL resolver."""
    try:
        return str(ipaddress.IPv4Address(host))
    except ValueError:
        pass
    try:
        return str(ipaddress.IPv4Address(socket.gethostbyname(host)))
    except (OSError, ValueError):
        if fallback is not None:
            try:
                # getent ahostsv4 emits one address per line, followed by socket type.
                for line in fallback(host).splitlines():
                    parts = line.split()
                    if parts:
                        return str(ipaddress.IPv4Address(parts[0]))
            except (OSError, ValueError, RuntimeError, subprocess.TimeoutExpired):
                pass
    raise RuntimeError(f'Không tìm được IPv4 của {host}. Kiểm tra tên thiết bị, '
                       'robot cùng mạng và dịch vụ mDNS trên robot nếu dùng tên .local.')


@dataclass(frozen=True)
class PingResult:
    address: str
    replies: int
    output: str
    attempts: int = 2

    @property
    def level(self):
        return 'ok' if self.replies == self.attempts else 'warning'

    @property
    def summary(self):
        if self.replies:
            return f'Ping {self.replies}/{self.attempts} phản hồi · IP {self.address}'
        return f'Ping chưa có phản hồi · IP {self.address}. Vẫn có thể thử Kết nối ngay để kiểm tra SSH.'


def ping_ipv4(address):
    address = str(ipaddress.IPv4Address(address))
    command = (['ping.exe', '-4', '-n', '2', '-w', '1000', address] if os.name == 'nt'
               else ['ping', '-4', '-n', '-c', '2', '-W', '1', address])
    result = subprocess.run(command, stdin=subprocess.DEVNULL, capture_output=True, text=True,
                            encoding='oem' if os.name == 'nt' else 'utf-8', errors='replace',
                            timeout=8, creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0))
    output = (result.stdout + '\n' + result.stderr).strip()
    # Windows ping may exit 0 for "Destination host unreachable". Count actual
    # Echo Replies (TTL present), never use its exit code as proof of reachability.
    replies = min(2, len(re.findall(r'\bttl\s*=\s*\d+', output, flags=re.IGNORECASE)))
    return PingResult(address, replies, output)
