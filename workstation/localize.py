"""Refine a rough robot pose by matching one LiDAR scan to the saved map. Runs on the Pi.

The user only clicks near the robot; this searches +-1 m and +-35 deg around the
click for the pose where the scan's points land on the map's walls, so AMCL starts
from the right place without the robot having to drive. numpy only (no scipy).
"""
import math

import numpy as np

XY_RANGE = 1.0
YAW_RANGE = math.radians(35)
MAX_POINTS = 360
# Mean wall likelihood of the scan points. On the real robot (2026-10-20) the true pose
# scored 0.68 and the best wrong ones about 0.5; synthetic scans score ~0.87.
MIN_SCORE = 0.55
MARGIN = 1.15     # the best pose must beat the best distinct other pose by this factor
AMBIGUOUS = 0.92  # a distinct pose scoring this close to the best: do not trust it
WIDE_RADIUS = 3.0


def likelihood(occupied, weights=(0.6, 0.3, 0.1)):
    """1 on walls, falling to 0 len(weights) cells away (a cheap distance field)."""
    field = occupied.astype(np.float32)
    grown = occupied.copy()
    for weight in weights:
        step = grown.copy()
        step[1:] |= grown[:-1]
        step[:-1] |= grown[1:]
        step[:, 1:] |= grown[:, :-1]
        step[:, :-1] |= grown[:, 1:]
        field = np.maximum(field, np.where(step, np.float32(weight), np.float32(0)))
        grown = step
    return field


def scan_points(ranges, angle_min, angle_increment, range_min, range_max, laser=(0.0, 0.0, 0.0)):
    """Valid scan returns as an Nx2 array in the robot frame; `laser` is its x, y, yaw there."""
    ranges = np.asarray(ranges, dtype=np.float64)
    angles = angle_min + angle_increment * np.arange(len(ranges))
    keep = np.isfinite(ranges) & (ranges > max(range_min, 0.05)) & (ranges < min(range_max, 12.0))
    r, a = ranges[keep], angles[keep] + laser[2]
    points = np.stack([laser[0] + r * np.cos(a), laser[1] + r * np.sin(a)], axis=1)
    if len(points) > MAX_POINTS:
        points = points[np.linspace(0, len(points) - 1, MAX_POINTS).astype(int)]
    return points


def _scores(field, points, x, y, yaw, origin, resolution, n):
    """Score grid over translations +-n cells for one yaw; field rows are bottom-first."""
    c, s = math.cos(yaw), math.sin(yaw)
    wx = x + c * points[:, 0] - s * points[:, 1]
    wy = y + s * points[:, 0] + c * points[:, 1]
    cols = np.floor((wx - origin[0]) / resolution).astype(int)
    rows = np.floor((wy - origin[1]) / resolution).astype(int)
    height, width = field.shape
    ok = (cols - n >= 0) & (cols + n < width) & (rows - n >= 0) & (rows + n < height)
    total = np.zeros((2 * n + 1, 2 * n + 1), dtype=np.float32)
    for row, col in zip(rows[ok], cols[ok]):
        total += field[row - n:row + n + 1, col - n:col + n + 1]
    return total


def refine(grid, resolution, origin, points, guess):
    """Best (x, y, yaw) near `guess` for scan `points` on an OccupancyGrid.

    grid: int8 values, rows bottom-first (OccupancyGrid order). Returns a dict with
    the pose, score (0..1), how far it moved, and whether another pose fits as well."""
    grid = np.asarray(grid, dtype=np.int8)
    reach = XY_RANGE + float(np.max(np.hypot(points[:, 0], points[:, 1]), initial=1.0)) + 0.5
    gx, gy, gyaw = guess
    col0 = max(0, int((gx - reach - origin[0]) / resolution))
    row0 = max(0, int((gy - reach - origin[1]) / resolution))
    col1 = min(grid.shape[1], int((gx + reach - origin[0]) / resolution) + 1)
    row1 = min(grid.shape[0], int((gy + reach - origin[1]) / resolution) + 1)
    n = int(round(XY_RANGE / resolution))
    # Padded so walls at the map's edge still count for every shifted pose.
    field = np.pad(likelihood(grid[row0:row1, col0:col1] >= 65), n)
    crop_origin = (origin[0] + (col0 - n) * resolution, origin[1] + (row0 - n) * resolution)
    best, results = None, []
    for dyaw in np.arange(-YAW_RANGE, YAW_RANGE + 1e-9, math.radians(1.0)):
        total = _scores(field, points, gx, gy, gyaw + dyaw, crop_origin, resolution, n)
        index = np.unravel_index(int(np.argmax(total)), total.shape)
        value = float(total[index])
        results.append((value, dyaw, index, total))
        if best is None or value > best[0]:
            best = (value, dyaw, index)
    # Finer yaw around the best coarse one.
    value, dyaw, index = best
    for fine in np.arange(dyaw - math.radians(1.0), dyaw + math.radians(1.0) + 1e-9, math.radians(0.25)):
        total = _scores(field, points, gx, gy, gyaw + fine, crop_origin, resolution, n)
        i = np.unravel_index(int(np.argmax(total)), total.shape)
        if float(total[i]) > value:
            value, dyaw, index = float(total[i]), fine, i
    x = gx + (index[1] - n) * resolution
    y = gy + (index[0] - n) * resolution
    # Another pose at least 0.5 m or 10 deg away that fits nearly as well: corridors look alike.
    rival = 0.0
    for other, other_yaw, _, total in results:
        rows, cols = np.nonzero(total >= AMBIGUOUS * value)
        for r, c in zip(rows, cols):
            far = math.hypot((r - index[0]) * resolution, (c - index[1]) * resolution) >= 0.5
            if far or abs(other_yaw - dyaw) >= math.radians(10):
                rival = max(rival, float(total[r, c]))
                break
    count = max(1, len(points))
    yaw = math.atan2(math.sin(gyaw + dyaw), math.cos(gyaw + dyaw))
    score = value / count
    return dict(x=round(x, 3), y=round(y, 3), yaw=round(yaw, 4), score=round(score, 3),
                moved_m=round(math.hypot(x - gx, y - gy), 2), turned_deg=round(math.degrees(dyaw), 1),
                ambiguous=rival >= AMBIGUOUS * value, ok=score >= MIN_SCORE and rival < AMBIGUOUS * value)


def _coarse(field, origin, resolution, points, xs, ys, keep=40):
    """Score candidate positions x yaws (5 deg) with a thinned scan; best `keep` seeds."""
    height, width = field.shape
    thin = points[::3] if len(points) > 120 else points
    seeds = []
    for yaw in np.arange(0, 2 * math.pi, math.radians(5)):
        c, s = math.cos(yaw), math.sin(yaw)
        rx = c * thin[:, 0] - s * thin[:, 1]
        ry = s * thin[:, 0] + c * thin[:, 1]
        cols = ((xs[:, None] + rx[None]) - origin[0]) / resolution
        rows = ((ys[:, None] + ry[None]) - origin[1]) / resolution
        cols, rows = cols.astype(int), rows.astype(int)
        inside = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
        values = np.where(inside, field[np.clip(rows, 0, height - 1), np.clip(cols, 0, width - 1)], 0).mean(axis=1)
        for i in np.argsort(values)[-5:]:
            seeds.append((float(values[i]), float(xs[i]), float(ys[i]), float(yaw)))
    seeds.sort(reverse=True)
    return seeds[:keep]


def locate(grid, resolution, origin, points, guess=None, radius=WIDE_RADIUS):
    """Robot pose from one scan: near `guess` first, then within `radius` of it at any
    heading, or over the whole map when `guess` is None. ok only for a clear winner."""
    grid = np.asarray(grid, dtype=np.int8)
    if guess is not None:
        near = refine(grid, resolution, origin, points, guess)
        if near["score"] >= MIN_SCORE and not near["ambiguous"]:
            return dict(near, method="near", ok=True)
    # Candidates sit 0.25-0.5 m apart, so the coarse pass needs a wider field to see walls.
    field = likelihood(grid >= 65, (0.85, 0.7, 0.55, 0.4, 0.3, 0.2, 0.1))
    free = (grid >= 0) & (grid <= 25)
    step = max(1, int(round((0.25 if guess is not None else 0.5) / resolution)))
    rows, cols = np.nonzero(free[::step, ::step])
    xs = origin[0] + (cols * step + 0.5) * resolution
    ys = origin[1] + (rows * step + 0.5) * resolution
    if guess is not None:
        close = np.hypot(xs - guess[0], ys - guess[1]) <= radius
        xs, ys = xs[close], ys[close]
    if not len(xs):
        return dict(ok=False, score=0.0, method="none")
    found = []
    for _, x, y, yaw in _coarse(field, origin, resolution, points, xs, ys):
        result = refine(grid, resolution, origin, points, (x, y, yaw))
        if any(math.hypot(result["x"] - other["x"], result["y"] - other["y"]) < 1.0
               and abs(math.remainder(result["yaw"] - other["yaw"], 2 * math.pi)) < math.radians(20)
               for other in found):
            continue
        found.append(result)
    found.sort(key=lambda item: item["score"], reverse=True)
    best = found[0]
    runner = found[1]["score"] if len(found) > 1 else 0.0
    if guess is not None:
        best = dict(best, moved_m=round(math.hypot(best["x"] - guess[0], best["y"] - guess[1]), 2),
                    turned_deg=round(math.degrees(math.remainder(best["yaw"] - guess[2], 2 * math.pi)), 1))
    ok = best["score"] >= MIN_SCORE and best["score"] >= MARGIN * runner
    return dict(best, method="wide" if guess is not None else "global", runner_up=round(runner, 3), ok=ok)
