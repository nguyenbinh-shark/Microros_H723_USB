"""Low-bandwidth telemetry for a robot on a cellular uplink. Runs on the Pi.

The Pi subscribes to ROS locally and sends the PC one JSON line per second
over SSH: topic health, the robot pose and, at most every few seconds, a small
PNG of the map being built. No DDS traffic crosses the 4G link. Subscribe only.
"""
import argparse
import base64
import math
import os
from pathlib import Path
import sys
import time

if "png_gray" not in globals():
    from map_library import png_gray
if "uplink" not in globals():
    from remote_ops import uplink

MAP_PERIOD_S = 5.0
MAP_MAX_SIDE = 480
PATH_STEP_M = 0.25
PATH_POINTS = 2000


def render_occupancy(values, width, height, max_side=MAP_MAX_SIDE):
    """OccupancyGrid data to a small PNG (black occupied, white free, grey unknown).

    Downsamples by an integer step; an occupied cell wins its block so thin
    walls survive. Returns (png, step)."""
    import numpy as np
    step = max(1, math.ceil(max(width, height) / max_side))
    grid = np.frombuffer(bytes(values), dtype=np.int8).reshape(height, width)
    out_h, out_w = -(-height // step), -(-width // step)
    padded = np.full((out_h * step, out_w * step), -1, dtype=np.int8)
    padded[:height, :width] = grid
    blocks = padded.reshape(out_h, step, out_w, step).swapaxes(1, 2).reshape(out_h, out_w, -1)
    occupied = (blocks >= 65).any(axis=2)
    free = ((blocks >= 0) & (blocks <= 25)).any(axis=2)
    image = np.full((out_h, out_w), 205, dtype=np.uint8)
    image[free] = 254
    image[occupied] = 0
    # OccupancyGrid row 0 is the bottom (lowest y); PNG row 0 is the top.
    image = image[::-1]
    return png_gray(out_w, out_h, [row.tobytes() for row in image]), step


class LiteExtension:
    """Hooks for ros_session.probe: adds pose, uplink and a throttled map image."""

    def __init__(self):
        self.map = None
        self.map_seq = 0
        self.sent_seq = 0
        self.sent_at = 0.0
        self.sent_png, self.sent_full = None, 0.0
        self.path = []
        self.uplink = None
        self.uplink_at = 0.0

    def setup(self, node, buffer):
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from nav_msgs.msg import OccupancyGrid
        self.buffer = buffer
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        def received(msg):
            self.map, self.map_seq = msg, self.map_seq + 1
        self.subscription = node.create_subscription(OccupancyGrid, "/map", received, latched)

    def pose(self):
        import rclpy
        try:
            t = self.buffer.lookup_transform("map", "base_footprint", rclpy.time.Time()).transform
        except Exception:
            return None
        q = t.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return {"x": round(t.translation.x, 3), "y": round(t.translation.y, 3), "yaw": round(yaw, 3)}

    def frame(self, data, now):
        data["lite"] = True
        if now - self.uplink_at > 5:
            self.uplink, self.uplink_at = uplink(), now
        data["uplink"] = self.uplink
        pose = self.pose()
        data["pose"] = pose
        if pose and (not self.path or math.dist(self.path[-1], (pose["x"], pose["y"])) >= PATH_STEP_M):
            self.path = (self.path + [(pose["x"], pose["y"])])[-PATH_POINTS:]
        if self.map is not None and self.map_seq != self.sent_seq and now - self.sent_at >= MAP_PERIOD_S:
            info = self.map.info
            png, step = render_occupancy(self.map.data, info.width, info.height)
            self.sent_seq, self.sent_at = self.map_seq, now
            # SLAM republishes an unchanged map while the robot stands still; send only changes.
            if png == self.sent_png and now - self.sent_full < 60:
                return data
            self.sent_png, self.sent_full = png, now
            data["live_map"] = dict(png_b64=base64.b64encode(png).decode("ascii"),
                                    resolution=info.resolution * step,
                                    origin=[info.origin.position.x, info.origin.position.y],
                                    width=-(-info.width // step), height=-(-info.height // step),
                                    path=[[round(x, 2), round(y, 2)] for x, y in self.path])
        return data


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--domain", type=int, default=0)
    args = parser.parse_args()
    if not 0 <= args.domain <= 232:
        parser.error("Invalid ROS domain")
    # The Pi's own DDS profile (from scripts/env.sh) stays in effect.
    os.environ["ROS_DOMAIN_ID"] = str(args.domain)
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from ros_session import probe
    probe(0, stream=True, interval=1.0, extension=LiteExtension())


if __name__ == "__main__":
    main()
