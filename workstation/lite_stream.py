"""Low-bandwidth telemetry for a robot on a cellular uplink. Runs on the Pi.

The Pi subscribes to ROS locally and sends the PC one JSON line per second
over SSH: topic health, the robot pose and, at most every few seconds, the map
being built. After one full PNG only the changed 64-pixel tiles and the new path
points are sent; LiveMapAssembler rebuilds the full image on the PC. No DDS
traffic crosses the 4G link. Subscribe only.
"""
import argparse
import base64
import math
import os
from pathlib import Path
import sys
import time
import zlib

if "png_gray" not in globals():
    from map_library import png_gray
if "uplink" not in globals():
    from remote_ops import uplink

MAP_PERIOD_S = 2.0  # slam_toolbox map_update_interval; tile deltas keep this cheap
MAP_MAX_SIDE = 480
PATH_STEP_M = 0.25
PATH_POINTS = 2000
TILE = 64
FULL_EVERY_S = 120  # a full image now and then, so a viewer that missed a delta recovers
UNKNOWN = 205
GOAL_STATES = {1: "accepted", 2: "executing", 3: "canceling", 4: "succeeded", 5: "canceled", 6: "aborted"}


def occupancy_image(values, width, height, max_side=MAP_MAX_SIDE, origin=(0.0, 0.0), resolution=1.0):
    """OccupancyGrid data to a small grey image (black occupied, white free, grey unknown).

    Downsamples by an integer step; an occupied cell wins its block so thin
    walls survive. Blocks sit on a world-anchored grid, so when SLAM grows the
    map the old pixels keep their values and only shift by whole pixels.
    Returns (rows top first as a numpy array, step, left, top): the image's
    left column and top edge in output pixels of that world grid."""
    import numpy as np
    step = max(1, math.ceil(max(width, height) / max_side))
    column0, row0 = round(origin[0] / resolution), round(origin[1] / resolution)
    pad_x, pad_y = column0 % step, row0 % step
    grid = np.frombuffer(bytes(values), dtype=np.int8).reshape(height, width)
    out_h, out_w = -(-(height + pad_y) // step), -(-(width + pad_x) // step)
    padded = np.full((out_h * step, out_w * step), -1, dtype=np.int8)
    padded[pad_y:pad_y + height, pad_x:pad_x + width] = grid
    blocks = padded.reshape(out_h, step, out_w, step).swapaxes(1, 2).reshape(out_h, out_w, -1)
    occupied = (blocks >= 65).any(axis=2)
    free = ((blocks >= 0) & (blocks <= 25)).any(axis=2)
    image = np.full((out_h, out_w), UNKNOWN, dtype=np.uint8)
    image[free] = 254
    image[occupied] = 0
    left, bottom = (column0 - pad_x) // step, (row0 - pad_y) // step
    # OccupancyGrid row 0 is the bottom (lowest y); PNG row 0 is the top.
    return np.ascontiguousarray(image[::-1]), step, left, bottom + out_h


def image_png(image):
    return png_gray(image.shape[1], image.shape[0], [row.tobytes() for row in image])


def render_occupancy(values, width, height, max_side=MAP_MAX_SIDE):
    """OccupancyGrid data to a small PNG. Returns (png, step)."""
    image, step, _, _ = occupancy_image(values, width, height, max_side)
    return image_png(image), step


def changed_tiles(previous, image, shift):
    """Tile corners (x, y) where `image` differs from `previous` placed at `shift`."""
    import numpy as np
    height, width = image.shape
    moved = np.full_like(image, UNKNOWN)
    dx, dy = shift
    old_h, old_w = previous.shape
    x0, y0 = max(0, dx), max(0, dy)
    x1, y1 = min(width, dx + old_w), min(height, dy + old_h)
    if x1 > x0 and y1 > y0:
        moved[y0:y1, x0:x1] = previous[y0 - dy:y1 - dy, x0 - dx:x1 - dx]
    different = image != moved
    return [(x, y) for y in range(0, height, TILE) for x in range(0, width, TILE)
            if different[y:y + TILE, x:x + TILE].any()]


def png_gray_pixels(png):
    """(width, height, pixels) of an 8-bit grey, filter-0 PNG as written by png_gray."""
    if png[:8] != b"\x89PNG\r\n\x1a\n" or png[24:29] != b"\x08\x00\x00\x00\x00":
        raise ValueError("Unsupported PNG")
    width, height = int.from_bytes(png[16:20], "big"), int.from_bytes(png[20:24], "big")
    data, offset = b"", 8
    while offset < len(png):
        length = int.from_bytes(png[offset:offset + 4], "big")
        if png[offset + 4:offset + 8] == b"IDAT":
            data += png[offset + 8:offset + 8 + length]
        offset += 12 + length
    raw, stride = zlib.decompress(data), width + 1
    if len(raw) != stride * height or any(raw[row * stride] for row in range(height)):
        raise ValueError("Unsupported PNG filter")
    pixels = bytearray(width * height)
    for row in range(height):
        pixels[row * width:(row + 1) * width] = raw[row * stride + 1:(row + 1) * stride]
    return width, height, pixels


class LiveMapAssembler:
    """PC side: rebuilds the full `live_map` (png_b64, resolution, origin, width,
    height, path) that LiveMapView draws from full images and tile deltas."""

    def __init__(self):
        self.pixels = None
        self.width = self.height = 0
        self.seq = None
        self.path = []

    def feed(self, message):
        if "png_b64" in message:
            self.width, self.height, self.pixels = png_gray_pixels(base64.b64decode(message["png_b64"]))
            self.path = [list(point) for point in message.get("path", [])]
            png_b64 = message["png_b64"]
        else:
            if self.pixels is None or message.get("base") != self.seq:
                return None  # missed a delta; the next full image resynchronises
            width, height = message["width"], message["height"]
            dx, dy = message.get("shift", (0, 0))
            pixels = bytearray([UNKNOWN]) * (width * height)
            x0, x1 = max(0, dx), min(width, dx + self.width)
            if x1 > x0:
                for row in range(max(0, dy), min(height, dy + self.height)):
                    source = (row - dy) * self.width - dx
                    pixels[row * width + x0:row * width + x1] = self.pixels[source + x0:source + x1]
            for x, y, tile_b64 in message.get("tiles", []):
                tile_w, tile_h, tile = png_gray_pixels(base64.b64decode(tile_b64))
                if x < 0 or y < 0 or x + tile_w > width or y + tile_h > height:
                    raise ValueError("Map tile outside the image")
                for row in range(tile_h):
                    start = (y + row) * width + x
                    pixels[start:start + tile_w] = tile[row * tile_w:(row + 1) * tile_w]
            self.width, self.height, self.pixels = width, height, pixels
            self.path = (self.path + [list(point) for point in message.get("path_add", [])])[-PATH_POINTS:]
            rows = [pixels[row * width:(row + 1) * width] for row in range(height)]
            png_b64 = base64.b64encode(png_gray(width, height, rows)).decode("ascii")
        self.seq = message.get("seq")
        return dict(png_b64=png_b64, resolution=message["resolution"], origin=message["origin"],
                    width=self.width, height=self.height, path=[list(point) for point in self.path])


class LiteExtension:
    """Hooks for ros_session.probe: adds pose, uplink and a throttled map image."""

    def __init__(self):
        self.map = None
        self.map_seq = 0
        self.sent_seq = 0
        self.sent_at = 0.0
        self.sent_full = 0.0
        self.sent = None  # (image, step, left, top) that the viewer has
        self.seq = 0
        self.path = []
        self.path_total = self.path_sent = 0
        self.known_cells = 0
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
        self.nav = None
        try:
            from action_msgs.msg import GoalStatusArray
            from nav2_msgs.action import NavigateToPose
        except ImportError:
            return
        def status(msg):
            if msg.status_list:
                latest = max(msg.status_list, key=lambda item: (item.goal_info.stamp.sec, item.goal_info.stamp.nanosec))
                state = GOAL_STATES.get(latest.status, "unknown")
                if self.nav is None or self.nav.get("state") != state:
                    self.nav = {"state": state}
        def feedback(msg):
            if self.nav is not None and self.nav.get("state") in ("accepted", "executing"):
                self.nav.update(distance=round(msg.feedback.distance_remaining, 2),
                                eta=msg.feedback.estimated_time_remaining.sec,
                                recoveries=msg.feedback.number_of_recoveries)
        self.nav_subscriptions = (
            node.create_subscription(GoalStatusArray, "/navigate_to_pose/_action/status", status, latched),
            node.create_subscription(NavigateToPose.Impl.FeedbackMessage, "/navigate_to_pose/_action/feedback",
                                     feedback, 10))

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
        if getattr(self, "nav", None):
            data["nav"] = self.nav
        pose = self.pose()
        data["pose"] = pose
        if pose and (not self.path or math.dist(self.path[-1], (pose["x"], pose["y"])) >= PATH_STEP_M):
            self.path = (self.path + [(pose["x"], pose["y"])])[-PATH_POINTS:]
            self.path_total += 1
        if self.map is not None and self.map_seq != self.sent_seq and now - self.sent_at >= MAP_PERIOD_S:
            self.sent_seq, self.sent_at = self.map_seq, now
            live_map = self.map_update(now)
            if live_map:
                data["live_map"] = live_map
        return data

    def new_map(self):
        """SLAM reset (Quét mới) or another map (Nav): the old path and image no longer apply."""
        self.path = []
        self.path_sent = self.path_total
        self.sent = None

    def map_update(self, now):
        """A full image, or only the changed tiles; None when nothing changed."""
        info = self.map.info
        import numpy as np
        known = int(np.count_nonzero(np.frombuffer(bytes(self.map.data), dtype=np.int8) != -1))
        # A growing map never loses most of its known cells; a reset or a different map does.
        if known < 0.5 * self.known_cells:
            self.new_map()
        self.known_cells = known
        image, step, left, top = occupancy_image(self.map.data, info.width, info.height, MAP_MAX_SIDE,
                                                 (info.origin.position.x, info.origin.position.y),
                                                 info.resolution)
        height, width = image.shape
        cell = info.resolution * step
        message = dict(resolution=cell, width=width, height=height,
                       origin=[left * cell, (top - height) * cell])
        previous = self.sent
        full = previous is None or previous[1] != step or now - self.sent_full >= FULL_EVERY_S
        if not full:
            shift = (previous[2] - left, top - previous[3])
            tiles = changed_tiles(previous[0], image, shift)
            # SLAM republishes an unchanged map while the robot stands still; send only changes.
            if not tiles and shift == (0, 0) and image.shape == previous[0].shape:
                return None
            full = len(tiles) * TILE * TILE > 0.6 * width * height
        if full:
            message.update(png_b64=base64.b64encode(image_png(image)).decode("ascii"),
                           path=[[round(x, 2), round(y, 2)] for x, y in self.path])
            self.sent_full = now
        else:
            new_points = min(len(self.path), self.path_total - self.path_sent)
            message.update(base=self.seq, shift=list(shift),
                           tiles=[[x, y, base64.b64encode(image_png(image[y:y + TILE, x:x + TILE])).decode("ascii")]
                                  for x, y in tiles],
                           path_add=[[round(x, 2), round(y, 2)] for x, y in self.path[len(self.path) - new_points:]]
                           if new_points else [])
        self.seq += 1
        message["seq"] = self.seq
        self.sent, self.path_sent = (image, step, left, top), self.path_total
        return message


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
    probe(0, stream=True, interval=0.5, extension=LiteExtension())


if __name__ == "__main__":
    main()
