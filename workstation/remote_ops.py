"""Sent to the Pi over SSH stdin. Does not modify its ROS checkout or command motors."""
import argparse
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys

if "map_command" not in globals():
    def map_command(*args, **kwargs):
        from map_library import map_command as command
        return command(*args, **kwargs)

if "map_bundle" not in globals():
    def map_bundle(*args, **kwargs):
        from map_library import map_bundle as command
        return command(*args, **kwargs)

if "scan_tasks" not in globals():
    def scan_tasks(*args, **kwargs):
        from robot_tasks import scan_tasks as command
        return command(*args, **kwargs)
    def stop_tasks(*args, **kwargs):
        from robot_tasks import stop_tasks as command
        return command(*args, **kwargs)

SERVICE = "rpbot-mapping.service"
NAV_SERVICE = "rpbot-navigation.service"
AUTOSAVE = "rpbot-autosave"
EXPLORE = "rpbot-explore"
EXPLORE_WS = Path.home() / "explore_ws"
ROOT = Path.home() / "pi5-stm32-slam"
# Studio installs its helper modules here so the timer and the lite stream run without stdin.
OPS_DIR = Path.home() / ".local/share/rpbot-console/ops"

RESET_PROGRAM = r'''
import json
import sys
import rclpy
from slam_toolbox.srv import Reset

rclpy.init()
node = rclpy.create_node('rpbot_reset_map')
try:
    client = node.create_client(Reset, '/slam_toolbox/reset')
    if not client.wait_for_service(timeout_sec=8.0):
        raise RuntimeError('SLAM Toolbox reset service is unavailable; map was not reset')
    if '--inspect' in sys.argv:
        print(json.dumps({'reset_available': True, 'service': '/slam_toolbox/reset'}))
    else:
        request = Reset.Request()
        request.pause_new_measurements = False
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=12.0)
        if not future.done():
            raise RuntimeError('Reset response timed out; outcome unknown. Check map before trying again')
        response = future.result()
        if response is None or response.result != Reset.Response.RESULT_SUCCESS:
            raise RuntimeError('SLAM Toolbox did not confirm a successful reset')
        print(json.dumps({'reset': 'success', 'new_measurements': 'enabled'}))
finally:
    node.destroy_node()
    rclpy.shutdown()
'''


NAV_PROGRAM = r'''
import json
import math
import sys
import time
import rclpy

action, x, y, yaw = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
rclpy.init()
node = rclpy.create_node('rpbot_nav_command')

def place(pose):
    pose.position.x, pose.position.y = x, y
    pose.orientation.z, pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)

try:
    if action == 'pose':
        from geometry_msgs.msg import PoseWithCovarianceStamped
        publisher = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        deadline = time.monotonic() + 6
        while not publisher.get_subscription_count() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not publisher.get_subscription_count():
            raise RuntimeError('AMCL is not running (nobody subscribes /initialpose); start Nav first')
        message = PoseWithCovarianceStamped()
        message.header.frame_id = 'map'
        place(message.pose.pose)
        covariance = [0.0] * 36
        covariance[0] = covariance[7] = 0.25   # 0.5 m
        covariance[35] = 0.0685                # 15 deg
        message.pose.covariance = covariance
        for _ in range(3):
            message.header.stamp = node.get_clock().now().to_msg()
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.2)
        print(json.dumps({'initial_pose': [x, y, yaw]}))
    elif action == 'goal':
        from rclpy.action import ActionClient
        from nav2_msgs.action import NavigateToPose
        client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        if not client.wait_for_server(timeout_sec=8.0):
            raise RuntimeError('Nav2 is not ready (navigate_to_pose); wait or set the initial pose')
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        place(goal.pose.pose)
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        handle = future.result() if future.done() else None
        if handle is None or not handle.accepted:
            raise RuntimeError('Nav2 rejected the goal')
        # Nav2 keeps executing after this client exits; the lite stream reports progress.
        print(json.dumps({'goal': [x, y, yaw], 'accepted': True}))
    elif action == 'cancel':
        from action_msgs.srv import CancelGoal
        client = node.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('Nav2 is not running; nothing to cancel')
        # A zero goal id and zero stamp cancel every goal.
        future = client.call_async(CancelGoal.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=8.0)
        if not future.done() or future.result() is None:
            raise RuntimeError('Cancel timed out; stop with the RC switch A (MANUAL) or F')
        print(json.dumps({'cancelled': len(future.result().goals_canceling)}))
    else:
        raise ValueError('Unknown nav command')
finally:
    node.destroy_node()
    rclpy.shutdown()
'''


def nav_command(action, domain, x=0.0, y=0.0, yaw=0.0):
    """Initial pose, goal or cancel for Nav2, sent from the Pi itself (no DDS over 4G)."""
    import math
    if not 0 <= domain <= 232:
        raise ValueError("Invalid ROS domain")
    if action not in ("pose", "goal", "cancel"):
        raise ValueError("Unknown nav command")
    if not all(math.isfinite(value) and abs(value) < 10000 for value in (x, y, yaw)):
        raise ValueError("Invalid pose")
    if action != "cancel" and not [item for item in launches()
                                   if item["mode"] == "navigation" and item["domain"] == domain]:
        raise RuntimeError("Nav is not running in this domain; start Nav on a saved map first")
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), RMW_IMPLEMENTATION="rmw_fastrtps_cpp")
    result = run(['bash', '-c', 'source "$1/scripts/env.sh" >/dev/null && exec python3 -c "$2" "${@:3}"',
                  'rpbot-nav', str(ROOT), NAV_PROGRAM, action, repr(float(x)), repr(float(y)), repr(float(yaw))],
                 env=env)
    if result.returncode:
        lines = (result.stderr or result.stdout).strip().splitlines()
        raise RuntimeError(lines[-1] if lines else "Nav command failed")
    return json.loads(result.stdout.strip().splitlines()[-1])


POSE_PROGRAM = r'''
import json
import math
import time
import rclpy
from tf2_ros import Buffer, TransformListener

rclpy.init()
node = rclpy.create_node('rpbot_read_pose')
buffer = Buffer()
listener = TransformListener(buffer, node)
deadline = time.monotonic() + 6
pose = None
while pose is None and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
    try:
        t = buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform
    except Exception:
        continue
    q = t.rotation
    pose = {'x': round(t.translation.x, 3), 'y': round(t.translation.y, 3),
            'yaw': round(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)), 4)}
print(json.dumps(pose))
node.destroy_node()
rclpy.shutdown()
'''

# Keeps the AMCL pose on disk while Nav runs, so the next Nav start on the same map
# can begin where the robot stopped (it runs inside the Nav service's control group).
KEEPER_PROGRAM = r'''
import json
import math
import os
import sys
import time
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped

target, name = sys.argv[1], sys.argv[2]
rclpy.init()
node = rclpy.create_node('rpbot_pose_keeper')
last = [0.0]

def keep(message):
    now = time.time()
    if now - last[0] < 2.0:
        return
    last[0] = now
    p, q = message.pose.pose.position, message.pose.pose.orientation
    pose = {'map': name, 'x': round(p.x, 3), 'y': round(p.y, 3),
            'yaw': round(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)), 4),
            'time': now, 'source': 'amcl'}
    with open(target + '.tmp', 'w') as stream:
        json.dump(pose, stream)
    os.replace(target + '.tmp', target)

node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', keep, 10)
rclpy.spin(node)
'''

LAST_POSE = Path.home() / ".local/share/rpbot-console/last_pose.json"


def read_pose(domain):
    """The robot pose map -> base_footprint from the running stack, or None."""
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), RMW_IMPLEMENTATION="rmw_fastrtps_cpp")
    result = run(['bash', '-c', 'source "$1/scripts/env.sh" >/dev/null && exec python3 -c "$2"',
                  'rpbot-pose', str(ROOT), POSE_PROGRAM], env=env)
    try:
        return json.loads(result.stdout.strip().splitlines()[-1]) if not result.returncode else None
    except (ValueError, IndexError):
        return None


def keep_pose(name, pose, source):
    LAST_POSE.parent.mkdir(parents=True, exist_ok=True)
    temporary = LAST_POSE.with_suffix(".tmp")
    import time
    temporary.write_text(json.dumps(dict(pose, map=name, time=time.time(), source=source)))
    os.replace(temporary, LAST_POSE)


def last_pose(name):
    """The pose kept for map `name` (by Nav's keeper or the mapping autosave), or None."""
    try:
        pose = json.loads(LAST_POSE.read_text())
    except (OSError, ValueError):
        return None
    return pose if pose.get("map") == name else None


def wait_no_launches(seconds=40):
    import time
    deadline = time.monotonic() + seconds
    while launches():
        if time.monotonic() > deadline:
            raise RuntimeError("The previous session did not stop within %d s" % seconds)
        time.sleep(1)


def session_map_name(pid):
    from datetime import datetime
    return datetime.fromtimestamp(process_start(pid)).strftime("autosave_%Y%m%d_%H%M")


def nav_switch(name, revision, domain):
    """Leave mapping for Nav on a saved map. The live LiDAR map is autosaved first.

    AMCL starts from the robot's pose: from SLAM when the chosen map is this mapping
    session's autosave (same map frame), else the pose last kept for that map."""
    import time
    report = {}
    initial = None
    current = launches()
    mapping = [item for item in current if item["mode"] == "mapping"]
    if mapping:
        if run(["systemctl", "--user", "is-active", SERVICE]).returncode:
            raise RuntimeError("Mapping was not started by RPBot; stop it yourself first")
        if len(mapping) == 1 and session_map_name(mapping[0]["pid"]) == name:
            pose = read_pose(mapping[0]["domain"])
            if pose:
                initial = dict(pose, source="mapping", age_s=0)
        report["saved"] = autosave()
        report["explore"] = stop_explore()
        result = run(["systemctl", "--user", "stop", SERVICE])
        if result.returncode:
            raise RuntimeError(result.stderr)
        wait_no_launches()
        if report["saved"].get("autosave") == name or initial:
            # The chosen map is this session's autosave, just refreshed: Nav takes the newest copy.
            revision = map_bundle(ROOT / "maps", name)[0]["revision"]
    if initial is None:
        kept = last_pose(name)
        if kept:
            initial = dict(x=kept["x"], y=kept["y"], yaw=kept["yaw"], source="last",
                           age_s=round(time.time() - kept.get("time", 0)))
    report.update(start_navigation(name, revision, domain, initial))
    report["initial_pose"] = initial
    return report


def nav_stop(resume_mapping):
    """Stop the Nav service (the cmd_vel watchdog stops the wheels); optionally map again."""
    report = {}
    if run(["systemctl", "--user", "is-active", NAV_SERVICE]).returncode == 0:
        result = run(["systemctl", "--user", "stop", NAV_SERVICE])
        if result.returncode:
            raise RuntimeError(result.stderr)
        report["navigation"] = "stopped"
    elif [item for item in launches() if item["mode"] == "navigation"]:
        raise RuntimeError("Nav was not started by RPBot; stop it yourself")
    else:
        report["navigation"] = "inactive"
    if resume_mapping:
        wait_no_launches()
        result = run(["systemctl", "--user", "start", SERVICE])
        if result.returncode:
            raise RuntimeError(result.stderr)
        report["mapping"] = "started"
    return report


def reset_mapping(domain, pid, *, inspect=False):
    live = [item for item in launches() if item["domain"] == domain
            and item["mode"] == "mapping" and not item["simulated_time"]]
    if len(live) != 1 or live[0]["pid"] != pid:
        raise RuntimeError("Mapping session changed; reconnect before resetting the map")
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), RMW_IMPLEMENTATION="rmw_fastrtps_cpp")
    command = ['bash', '-c', 'source "$1/scripts/env.sh" && exec python3 -c "$2" "${@:3}"',
               'rpbot-reset', str(ROOT), RESET_PROGRAM]
    if inspect:
        command.append('--inspect')
    result = run(command, env=env)
    if result.returncode:
        raise RuntimeError(result.stderr + result.stdout)
    return json.loads(result.stdout)


def run(args, **kwargs):
    return subprocess.run(args, text=True, capture_output=True, timeout=30, **kwargs)


FLOOR_ACTIONS = {"floors": "list", "floor-register": "register", "floor-mark": "mark-elevator",
                 "floor-switch": "switch", "floor-remove": "remove"}


def floor_command(action, domain, level=None, map_name=None, map3d=None, label=None):
    """Run scripts/floors.py of pi5-stm32-slam in the robot domain; it never commands motors."""
    script = ROOT / "scripts/floors.py"
    if not script.is_file():
        raise RuntimeError("pi5-stm32-slam trên Pi chưa có scripts/floors.py; cập nhật repo trên Pi trước")
    args = [FLOOR_ACTIONS[action]]
    if level is not None:
        args += ["--level", str(int(level))]
    if map_name is not None:
        args += ["--map", map_name]
    if map3d is not None:
        args += ["--map3d", map3d]
    if label is not None:
        args += ["--label", label]
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), RMW_IMPLEMENTATION="rmw_fastrtps_cpp")
    # A floor switch loads a map, re-localizes and may restart the 3D map (up to ~90 s).
    result = subprocess.run(['bash', '-c', 'source "$1/scripts/env.sh" >/dev/null && exec python3 "$1/scripts/floors.py" "${@:2}"',
                             'rpbot-floors', str(ROOT), *args],
                            text=True, capture_output=True, timeout=150, env=env)
    try:
        report = json.loads(result.stdout.strip().splitlines()[-1])
    except (IndexError, ValueError):
        raise RuntimeError((result.stderr + result.stdout).strip() or "floors.py returned no result")
    if result.returncode or "error" in report:
        raise RuntimeError(report.get("error") or result.stderr)
    return report


def save_map3d(name):
    """Checkpoint the running pi5-visual-slam session under the LiDAR map's name (stops that session)."""
    session = Path.home() / "pi5-visual-slam/scripts/session.py"
    if not session.is_file():
        raise RuntimeError("pi5-visual-slam is not installed on the Pi")
    state = json.loads(run(["python3", str(session), "status"]).stdout)["result"]
    if not state.get("active"):
        raise RuntimeError("Camera SLAM is not running; only the LiDAR map was saved")
    result = subprocess.run(["python3", str(session), "checkpoint", "--name", name, "--database", state["database"]],
                            text=True, capture_output=True, timeout=120)
    report = json.loads(result.stdout or "{}")
    if not report.get("ok"):
        raise RuntimeError(report.get("error") or result.stderr)
    return report["result"]["database"]


def launches():
    result = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            if entry.stat().st_uid != os.getuid():
                continue
            cmd = (entry / "cmdline").read_bytes().split(b"\0")
            words = [part.decode(errors="replace") for part in cmd if part]
            if "pi5_slam_bringup" not in words or "robot.launch.py" not in words:
                continue
            env = dict(part.split(b"=", 1) for part in (entry / "environ").read_bytes().split(b"\0")
                       if b"=" in part)
            profile = env.get(b"FASTRTPS_DEFAULT_PROFILES_FILE", b"").decode()
            map_arg = next((w.split(":=", 1)[1] for w in words if w.startswith("map:=")), "")
            map_path = Path(map_arg).expanduser() if map_arg else None
            if map_path is not None and not map_path.is_absolute():
                map_path = (entry / "cwd").resolve() / map_path
            result.append(dict(pid=int(entry.name),
                               mode=next((w.split(":=", 1)[1] for w in words if w.startswith("mode:=")), "bench"),
                               simulated_time="use_sim_time:=true" in words,
                               domain=int(env.get(b"ROS_DOMAIN_ID", b"0")),
                               map=str(map_path.resolve()) if map_path is not None else "",
                               profile_xml=Path(profile).read_text() if profile else ""))
        except (OSError, ValueError):
            continue
    return result


# USB modems: RNDIS/ECM/NCM/MBIM sticks and QMI modules.
CELLULAR_DRIVERS = {"rndis_host", "cdc_ether", "cdc_ncm", "cdc_mbim", "huawei_cdc_ncm", "qmi_wwan", "option"}


def classify_uplink(routes, driver_of):
    """The lowest-metric IPv4 default route and its kind: cellular, wifi, lan or none."""
    best = None
    for line in routes.splitlines():
        words = line.split()
        if "dev" not in words:
            continue
        device = words[words.index("dev") + 1]
        metric = int(words[words.index("metric") + 1]) if "metric" in words else 0
        if best is None or metric < best[1]:
            best = (device, metric)
    if best is None:
        return {"interface": None, "kind": "none"}
    device = best[0]
    if driver_of(device) in CELLULAR_DRIVERS or device.startswith(("wwan", "ppp", "usb")):
        kind = "cellular"
    else:
        kind = "wifi" if device.startswith("wl") else "lan"
    return {"interface": device, "kind": kind}


def uplink():
    def driver_of(device):
        try:
            return Path(f"/sys/class/net/{device}/device/driver").resolve().name
        except OSError:
            return ""
    routes = subprocess.run(["ip", "-4", "route", "show", "default"], text=True,
                            capture_output=True, timeout=5).stdout
    return classify_uplink(routes, driver_of)


def power():
    """Pi supply health from the firmware throttle flags (an early sign of a flat battery)."""
    try:
        text = subprocess.run(["vcgencmd", "get_throttled"], text=True, capture_output=True, timeout=5).stdout
        flags = int(text.strip().split("=", 1)[1], 16)
    except (OSError, ValueError, IndexError, subprocess.TimeoutExpired):
        return None
    return {"flags": flags, "under_voltage_now": bool(flags & 0x1), "throttled_now": bool(flags & 0x4),
            "under_voltage_since_boot": bool(flags & 0x10000)}


def status():
    active = launches()
    props = run(["systemctl", "--user", "show", SERVICE, "--property=ActiveState,UnitFileState"]).stdout
    try:
        revision = (OPS_DIR / "revision").read_text().strip()
    except OSError:
        revision = None
    return dict(hostname=os.uname().nodename, launches=active,
                service=dict(line.split("=", 1) for line in props.splitlines() if "=" in line),
                maps=sorted(p.stem for p in (ROOT / "maps").glob("*.yaml")),
                project_exists=(ROOT / "scripts/run.sh").is_file(),
                ops_revision=revision, uplink=uplink(), explore=explore_state(), power=power())


def install_ops(payload):
    """Write Studio's helper modules and the map autosave timer (every 2 min while mapping)."""
    import base64
    import hashlib
    files = json.loads(base64.b64decode(payload, validate=True))
    if not files or any(not re.fullmatch(r"[a-z_]+\.py", name) for name in files):
        raise ValueError("Invalid helper bundle")
    OPS_DIR.mkdir(parents=True, exist_ok=True)
    for name, source in files.items():
        temporary = OPS_DIR / (name + ".tmp")
        temporary.write_text(source, encoding="utf-8")
        os.replace(temporary, OPS_DIR / name)
    revision = hashlib.sha256("".join(files[name] for name in sorted(files)).encode("utf-8")).hexdigest()
    units = Path.home() / ".config/systemd/user"
    units.mkdir(parents=True, exist_ok=True)
    (units / (AUTOSAVE + ".service")).write_text(f"""[Unit]
Description=RPBot map autosave (only while one real mapping session runs)

[Service]
Type=oneshot
Environment=RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ExecStart=/usr/bin/python3 {OPS_DIR / "remote_ops.py"} autosave
TimeoutStartSec=60
""")
    (units / (AUTOSAVE + ".timer")).write_text("""[Unit]
Description=RPBot map autosave every 2 minutes

[Timer]
OnBootSec=3min
OnUnitActiveSec=2min
AccuracySec=10s

[Install]
WantedBy=timers.target
""")
    for command in (["systemctl", "--user", "daemon-reload"],
                    ["systemctl", "--user", "enable", "--now", AUTOSAVE + ".timer"]):
        result = run(command)
        if result.returncode:
            raise RuntimeError(result.stderr)
    (OPS_DIR / "revision").write_text(revision + "\n")
    return {"ops_revision": revision, "autosave": "every 2 min while mapping"}


def process_start(pid):
    """Wall-clock start time of a process."""
    fields = Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()
    boot = next(int(line.split()[1]) for line in Path("/proc/stat").read_text().splitlines()
                if line.startswith("btime "))
    return boot + int(fields[19]) / os.sysconf("SC_CLK_TCK")


def autosave():
    """Overwrite autosave_<session start> with the live map; skip unless exactly one mapping runs."""
    import fcntl
    import shutil
    from datetime import datetime
    live = [item for item in launches() if item["mode"] == "mapping" and not item["simulated_time"]]
    if len(live) != 1:
        return {"autosave": "skipped", "reason": f"{len(live)} mapping sessions"}
    session = live[0]
    name = datetime.fromtimestamp(process_start(session["pid"])).strftime("autosave_%Y%m%d_%H%M")
    maps = ROOT / "maps"
    # The timer and a manual run must not share the staging directory.
    guard = (maps / ".autosave.lock").open("a")
    try:
        fcntl.flock(guard, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        return {"autosave": "skipped", "reason": "another autosave is running"}
    staging = maps / ".autosave"
    shutil.rmtree(staging, ignore_errors=True)
    staging.mkdir(parents=True)
    env = dict(os.environ, ROS_DOMAIN_ID=str(session["domain"]))
    result = run(["bash", str(ROOT / "scripts/save_map.sh"), str(staging / name)], env=env)
    if result.returncode or not (staging / (name + ".yaml")).is_file() or not (staging / (name + ".pgm")).is_file():
        raise RuntimeError("Autosave failed: " + (result.stderr + result.stdout).strip()[-500:])
    # Same basename, so the YAML's relative image reference stays valid; image first.
    with (maps / ".rpbot-library.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        os.replace(staging / (name + ".pgm"), maps / (name + ".pgm"))
        os.replace(staging / (name + ".yaml"), maps / (name + ".yaml"))
    shutil.rmtree(staging, ignore_errors=True)
    # Also keep where the robot is on this map, for a later Nav start on it.
    pose = read_pose(session["domain"])
    if pose:
        keep_pose(name, pose, "mapping")
    return {"autosave": name}


def explore_state():
    return run(["systemctl", "--user", "is-active", EXPLORE]).stdout.strip() or "inactive"


def start_explore(domain, dry_run):
    """Nav2 on the live SLAM map plus explore_lite, beside the running mapping session.

    Commands reach the wheels only while the RC switch A is AUTO and the motors are armed."""
    live = [item for item in launches() if item["domain"] == domain
            and item["mode"] == "mapping" and not item["simulated_time"]]
    if len(live) != 1:
        raise RuntimeError("Tự khám phá cần đúng một phiên Mapping thật đang chạy trong domain đã chọn")
    if not (EXPLORE_WS / "install/setup.bash").is_file():
        raise RuntimeError("explore_lite chưa được build trên Pi (~/explore_ws)")
    if not (ROOT / "src/pi5_slam_bringup/launch/explore.launch.py").is_file():
        raise RuntimeError("pi5-stm32-slam trên Pi chưa có launch/explore.launch.py")
    state = explore_state()
    if state in ("active", "activating", "reloading"):
        return {"explore": "already_running"}
    if state == "deactivating":
        raise RuntimeError("Tự khám phá đang dừng; chờ vài giây rồi thử lại")
    target = Path.home() / ".local/share/rpbot-console"
    target.mkdir(parents=True, exist_ok=True)
    wrapper = target / "explore.sh"
    wrapper.write_text("#!/usr/bin/env bash\nset -eo pipefail\n"
                       + "source " + shlex.quote(str(ROOT / "scripts/env.sh")) + "\n"
                       + "source " + shlex.quote(str(EXPLORE_WS / "install/setup.bash")) + "\n"
                       + 'exec ros2 launch pi5_slam_bringup explore.launch.py "dry_run:=$1"\n')
    wrapper.chmod(0o755)
    result = run(["systemd-run", "--user", "--unit", EXPLORE, "--collect",
                  "--property=KillSignal=SIGINT", "--property=TimeoutStopSec=20",
                  f"--setenv=ROS_DOMAIN_ID={domain}", "--setenv=RMW_IMPLEMENTATION=rmw_fastrtps_cpp",
                  "--setenv=PYTHONUNBUFFERED=1", str(wrapper), "true" if dry_run else "false"])
    if result.returncode:
        raise RuntimeError(result.stderr or result.stdout)
    return {"explore": "started", "dry_run": dry_run, "domain": domain,
            "note": "Nav2 needs ~25 s before the first goal"}


def stop_explore():
    if explore_state() == "inactive":
        return {"explore": "inactive"}
    result = run(["systemctl", "--user", "stop", EXPLORE])
    if result.returncode:
        raise RuntimeError(result.stderr)
    return {"explore": "stopped"}


def lite_stream(domain):
    """Replace this process with the lite telemetry stream in the robot's ROS environment."""
    script = OPS_DIR / "lite_stream.py"
    if not script.is_file():
        raise RuntimeError("Lite stream is not installed; reconnect RPBot Studio to install it")
    os.execvp("bash", ["bash", "-c", 'source "$1/scripts/env.sh" >/dev/null && exec python3 -u "$2" --domain "$3"',
                       "rpbot-lite", str(ROOT), str(script), str(domain)])


def install_autostart():
    if not (ROOT / "install/setup.bash").is_file():
        raise RuntimeError("Build pi5-stm32-slam on the Pi first")
    # Do not introduce a second stack when a different boot service already owns it.
    for name in ("pi5-slam-demo.service", "micro_ros.service"):
        for user in ([], ["--user"]):
            if run(["systemctl", *user, "is-enabled", name]).returncode == 0:
                raise RuntimeError(f"Another autostart service is enabled: {name}")
    target = Path.home() / ".local/share/rpbot-console"
    target.mkdir(parents=True, exist_ok=True)
    wrapper = target / "mapping.sh"
    # The launcher's flock prevents overlap even if launched concurrently elsewhere.
    wrapper.write_text("#!/usr/bin/env bash\nset -eo pipefail\n"
                       + "if pgrep -u \"$(id -u)\" -f '[r]os2 launch pi5_slam_bringup robot.launch.py' >/dev/null; then\n"
                       + "  echo 'An existing robot session is running; left unchanged.' >&2; exit 75\nfi\n"
                       + "exec " + shlex.quote(str(ROOT / "scripts/run.sh"))
                       + " mapping allow_motor_enable:=false\n")
    wrapper.chmod(0o755)
    units = Path.home() / ".config/systemd/user"
    units.mkdir(parents=True, exist_ok=True)
    (units / SERVICE).write_text(f"""[Unit]
Description=RPBot real mapping (STM32 + LiDAR, no automatic motor enable)
StartLimitIntervalSec=0

[Service]
Type=simple
WorkingDirectory={ROOT}
Environment=ROS_DOMAIN_ID=0
Environment=ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
Environment=RMW_IMPLEMENTATION=rmw_fastrtps_cpp
Environment=PYTHONUNBUFFERED=1
UnsetEnvironment=ROS_LOCALHOST_ONLY ROS_STATIC_PEERS ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT CYCLONEDDS_URI FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
ExecStart={wrapper}
Restart=on-failure
RestartSec=10
RestartPreventExitStatus=75
KillSignal=SIGINT
KillMode=control-group
TimeoutStopSec=30

[Install]
WantedBy=default.target
""")
    for command in (["systemctl", "--user", "daemon-reload"],
                    ["systemctl", "--user", "enable", SERVICE]):
        result = run(command)
        if result.returncode:
            raise RuntimeError(result.stderr)
    linger = run(["loginctl", "show-user", str(os.getuid()), "--property=Linger", "--value"]).stdout.strip()
    if linger != "yes":
        raise RuntimeError("Service installed; enable linger with loginctl enable-linger before unattended boot")
    return {"autostart": "enabled", "current_session": "unchanged", "linger": linger}


def start_navigation(name, revision, domain, initial_pose=None):
    """Start a saved map without replacing a running stack or enabling motors."""
    import fcntl

    if not 0 <= domain <= 232:
        raise ValueError("Invalid ROS domain")
    target = Path.home() / ".local/share/rpbot-console"
    target.mkdir(parents=True, exist_ok=True)
    # Serialize requests from different Consoles before changing the service files.
    with (target / "start-navigation.lock").open("a") as lock, \
            (ROOT / "maps/.rpbot-library.lock").open("a") as library_lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        fcntl.flock(library_lock, fcntl.LOCK_SH)
        info, map_yaml, *_ = map_bundle(ROOT / "maps", name)
        if not revision or revision != info["revision"]:
            raise RuntimeError("Map đã thay đổi. Hãy tải lại danh sách map trước khi bắt đầu Nav.")
        current = launches()
        if current:
            if (len(current) == 1 and current[0]["mode"] == "navigation"
                    and not current[0]["simulated_time"] and current[0]["domain"] == domain
                    and current[0].get("map") == str(map_yaml)):
                return dict(status="already_running", mode="navigation", name=name,
                            domain=domain, pid=current[0]["pid"])
            details = "; ".join(f"{item['mode']} (PID {item['pid']}, domain {item['domain']})" for item in current)
            raise RuntimeError("Chưa thể bắt đầu Nav. Hãy dùng Dừng tạo map / Dừng Nav hoặc tab Tác vụ để dừng phiên hiện tại trước: " + details)
        if any(item["kind"] == "bag" and item["mode"] == "play"
               for item in scan_tasks(ROOT, domain, "all")):
            raise RuntimeError("Đang phát bag trong domain đã chọn. Dừng phát bag trước khi bắt đầu Nav.")
        state = run(["systemctl", "--user", "show", NAV_SERVICE, "--property=ActiveState", "--value"])
        if state.stdout.strip() in ("active", "activating", "deactivating", "reloading"):
            raise RuntimeError("Phiên Nav đang khởi động hoặc đang dừng. Chờ rồi kiểm tra lại tác vụ.")
        if not (ROOT / "install/setup.bash").is_file():
            raise RuntimeError("Build pi5-stm32-slam on the Pi first")
        wrapper = target / "navigation.sh"
        command = [str(ROOT / "scripts/run.sh"), "navigation", "map:=" + str(map_yaml), "allow_motor_enable:=false"]
        if initial_pose:
            command.append("initial_pose:=%.3f,%.3f,%.4f" % (initial_pose["x"], initial_pose["y"], initial_pose["yaw"]))
        keeper = target / "pose_keeper.py"
        keeper.write_text(KEEPER_PROGRAM)
        keep = shlex.join(["bash", "-c", 'source "$1/scripts/env.sh" >/dev/null && exec python3 "$2" "$3" "$4"',
                           "rpbot-keeper", str(ROOT), str(keeper), str(LAST_POSE), name])
        wrapper.write_text("#!/usr/bin/env bash\nset -eo pipefail\n"
                           + "if pgrep -u \"$(id -u)\" -f '[r]os2 launch pi5_slam_bringup robot.launch.py' >/dev/null; then\n"
                           + "  echo 'An existing robot session is running; left unchanged.' >&2; exit 75\nfi\n"
                           + keep + " &\n"
                           + "exec " + shlex.join(command) + "\n")
        wrapper.chmod(0o755)
        units = Path.home() / ".config/systemd/user"
        units.mkdir(parents=True, exist_ok=True)
        def unit_quote(value):
            return '"' + str(value).replace('\\', '\\\\').replace('"', '\\"').replace('%', '%%') + '"'
        (units / NAV_SERVICE).write_text(f"""[Unit]
Description=RPBot navigation on a saved map

[Service]
Type=simple
WorkingDirectory={str(ROOT).replace('%', '%%')}
Environment=ROS_DOMAIN_ID={domain}
Environment=ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
Environment=RMW_IMPLEMENTATION=rmw_fastrtps_cpp
Environment=PYTHONUNBUFFERED=1
UnsetEnvironment=ROS_LOCALHOST_ONLY ROS_STATIC_PEERS ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT CYCLONEDDS_URI FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
ExecStart={unit_quote(wrapper)}
Restart=no
KillSignal=SIGINT
KillMode=control-group
TimeoutStopSec=30
""")
        for command in (["systemctl", "--user", "daemon-reload"],
                        ["systemctl", "--user", "start", NAV_SERVICE]):
            result = run(command)
            if result.returncode:
                raise RuntimeError(result.stderr or result.stdout or "Không khởi động được Nav; xem Debug → Đọc log robot.")
        return dict(status="started", mode="navigation", name=name, domain=domain,
                    service=NAV_SERVICE, note="Check Nav2 readiness before setting a goal")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("action", choices=("status", "save", "reset", "reset-info", "logs", "start", "stop", "install-autostart",
                                           "maps", "map-preview", "map-export", "map-rename", "map-delete", "tasks", "stop-tasks", "start-nav",
                                           "install-ops", "autosave", "lite-stream", "explore-start", "explore-stop",
                                           "nav-switch", "nav-stop", "nav-pose", "nav-goal", "nav-cancel",
                                           *FLOOR_ACTIONS))
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--resume-mapping", action="store_true")
    parser.add_argument("--x", type=float, default=0.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--level", type=int)
    parser.add_argument("--map3d")
    parser.add_argument("--label")
    parser.add_argument("--with-3d", action="store_true")
    parser.add_argument("--name")
    parser.add_argument("--new-name")
    parser.add_argument("--revision")
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--pid", type=int)
    parser.add_argument("--scope", choices=("mapping", "navigation", "all"), default="all")
    parser.add_argument("--targets")
    args = parser.parse_args()
    if not 0 <= args.domain <= 232:
        parser.error("Invalid ROS domain")
    if args.action == 'start-nav':
        print(json.dumps(start_navigation(args.name, args.revision, args.domain)))
    elif args.action == "nav-switch":
        print(json.dumps(nav_switch(args.name, args.revision, args.domain)))
    elif args.action == "nav-stop":
        print(json.dumps(nav_stop(args.resume_mapping)))
    elif args.action in ("nav-pose", "nav-goal", "nav-cancel"):
        print(json.dumps(nav_command(args.action[4:], args.domain, args.x, args.y, args.yaw)))
    elif args.action == "install-ops":
        print(json.dumps(install_ops(OPS_PAYLOAD)))
    elif args.action == "autosave":
        print(json.dumps(autosave()))
    elif args.action == "lite-stream":
        lite_stream(args.domain)
    elif args.action == "explore-start":
        print(json.dumps(start_explore(args.domain, args.dry_run)))
    elif args.action == "explore-stop":
        print(json.dumps(stop_explore()))
    elif args.action in FLOOR_ACTIONS:
        print(json.dumps(floor_command(args.action, args.domain, args.level, args.name, args.map3d, args.label),
                         ensure_ascii=False))
    elif args.action == 'tasks':
        print(json.dumps({'tasks': scan_tasks(ROOT, args.domain, args.scope), 'domain': args.domain}))
    elif args.action == 'stop-tasks':
        import base64
        if args.scope in ("mapping", "all"):
            # Exploration drives on the mapping session's map; never leave it running alone.
            stop_explore()
        targets = json.loads(base64.b64decode(args.targets, validate=True))
        print(json.dumps(stop_tasks(ROOT, args.domain, args.scope, targets)))
    elif args.action == "maps" or args.action.startswith("map-"):
        if args.action in ("map-rename", "map-delete"):
            import fcntl
            with (ROOT / "maps/.rpbot-library.lock").open("a") as lock:
                fcntl.flock(lock, fcntl.LOCK_EX)
                result = map_command(ROOT / "maps", args.action, args.name, args.new_name, args.revision)
        else:
            result = map_command(ROOT / "maps", args.action, args.name, args.new_name, args.revision)
        print(json.dumps(result))
    elif args.action == "status":
        print(json.dumps(status()))
    elif args.action == "install-autostart":
        print(json.dumps(install_autostart()))
    elif args.action in ("reset", "reset-info"):
        print(json.dumps(reset_mapping(args.domain, args.pid, inspect=args.action == "reset-info")))
    elif args.action == "save":
        if not args.name or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", args.name):
            raise ValueError("Map name: 1-64 letters, numbers, - or _")
        if not 0 <= args.domain <= 232:
            raise ValueError("Invalid ROS domain")
        live = [item for item in launches() if item["domain"] == args.domain
                and item["mode"] == "mapping" and not item["simulated_time"]]
        if len(live) != 1:
            raise RuntimeError("Save requires exactly one real mapping session in the selected domain")
        env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain))
        result = run(["bash", str(ROOT / "scripts/save_map.sh"), str(ROOT / "maps" / args.name)], env=env)
        if result.returncode:
            raise RuntimeError(result.stdout + result.stderr)
        yaml = ROOT / "maps" / (args.name + ".yaml")
        pgm = yaml.with_suffix(".pgm")
        if not yaml.is_file() or not pgm.is_file():
            raise RuntimeError("Map saver returned without both YAML and PGM files")
        report = {"saved": str(yaml), "pgm": str(pgm)}
        if args.with_3d:
            # The LiDAR map is already safe; a 3D failure is reported, not raised.
            try:
                report["map3d"] = save_map3d(args.name)
            except Exception as exc:
                report["map3d_error"] = str(exc)
        print(json.dumps(report))
    elif args.action == "logs":
        print(run(["journalctl", "--user", "-u", SERVICE, "-u", NAV_SERVICE, "-n", "100", "--no-pager"]).stdout)
        for item in launches():
            print(f"Active {item['mode']} launch PID {item['pid']} (domain {item['domain']})")
        print("If mapping was started outside this service, its original log is in ~/.ros/log.")
    else:
        current = launches()
        if args.action == "start" and current:
            live = [item for item in current if item["mode"] in ("mapping", "navigation") and not item["simulated_time"]]
            if len(live) != 1:
                details = "; ".join(f"{item['mode']} (PID {item['pid']}, domain {item['domain']})" for item in current)
                raise RuntimeError("Chưa thể mở Mapping. Các phiên đang chạy: " + details)
            active = live[0]
            print(json.dumps({"requested": "start", "status": "already_running" if active["mode"] == "mapping" else "different_mode",
                              "mode": active["mode"], "pid": active["pid"], "domain": active["domain"]}))
            return
        if args.action == "stop" and run(["systemctl", "--user", "is-active", SERVICE]).returncode:
            raise RuntimeError("Mapping is not owned by RPBot Console; existing session left unchanged")
        if args.action == "stop":
            stop_explore()
        result = run(["systemctl", "--user", args.action, SERVICE])
        if result.returncode:
            raise RuntimeError(result.stderr)
        print(json.dumps({"requested": args.action, "service": SERVICE,
                          "note": "Check data to confirm readiness"}))


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        sys.exit(1)
