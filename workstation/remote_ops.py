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

SERVICE = "rpbot-mapping.service"
ROOT = Path.home() / "pi5-stm32-slam"

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
            result.append(dict(pid=int(entry.name),
                               mode=next((w.split(":=", 1)[1] for w in words if w.startswith("mode:=")), "bench"),
                               simulated_time="use_sim_time:=true" in words,
                               domain=int(env.get(b"ROS_DOMAIN_ID", b"0")),
                               profile_xml=Path(profile).read_text() if profile else ""))
        except (OSError, ValueError):
            continue
    return result


def status():
    active = launches()
    props = run(["systemctl", "--user", "show", SERVICE, "--property=ActiveState,UnitFileState"]).stdout
    return dict(hostname=os.uname().nodename, launches=active,
                service=dict(line.split("=", 1) for line in props.splitlines() if "=" in line),
                maps=sorted(p.stem for p in (ROOT / "maps").glob("*.yaml")),
                project_exists=(ROOT / "scripts/run.sh").is_file())


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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("action", choices=("status", "save", "reset", "reset-info", "logs", "start", "stop", "install-autostart",
                                           "maps", "map-preview", "map-export", "map-rename", "map-delete"))
    parser.add_argument("--name")
    parser.add_argument("--new-name")
    parser.add_argument("--revision")
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--pid", type=int)
    args = parser.parse_args()
    if not 0 <= args.domain <= 232:
        parser.error("Invalid ROS domain")
    if args.action == "maps" or args.action.startswith("map-"):
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
        print(json.dumps({"saved": str(yaml), "pgm": str(pgm)}))
    elif args.action == "logs":
        print(run(["journalctl", "--user", "-u", SERVICE, "-n", "100", "--no-pager"]).stdout)
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
