"""PC-side ROS entry point. Diagnostics only subscribe; no motor commands."""
import argparse
import json
import os
from pathlib import Path
import time
import select
import sys


def environment(domain, profile):
    for key in ("ROS_LOCALHOST_ONLY", "ROS_STATIC_PEERS", "ROS_DISCOVERY_SERVER",
                "ROS_SUPER_CLIENT", "CYCLONEDDS_URI", "FASTRTPS_DEFAULT_PROFILES_FILE",
                "FASTDDS_DEFAULT_PROFILES_FILE"):
        os.environ.pop(key, None)
    os.environ.update(ROS_DOMAIN_ID=str(domain), RMW_IMPLEMENTATION="rmw_fastrtps_cpp",
                      ROS_AUTOMATIC_DISCOVERY_RANGE="SUBNET")
    if profile:
        if not Path(profile).is_file():
            raise FileNotFoundError(profile)
        os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = str(Path(profile).resolve())


def probe(seconds, *, stream=False):
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from nav_msgs.msg import OccupancyGrid, Odometry
    from sensor_msgs.msg import Imu, LaserScan, JointState
    from std_msgs.msg import UInt8
    from tf2_ros import Buffer, TransformListener

    rclpy.init()
    node = rclpy.create_node("rpbot_pc_diagnostics")
    data, counts, last = {}, {}, {}
    sensor = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def receive(topic, msg):
        counts[topic] = counts.get(topic, 0) + 1
        last[topic] = time.monotonic()
        if topic == "/map":
            data["map"] = dict(width=msg.info.width, height=msg.info.height,
                               resolution=msg.info.resolution)
        elif topic == "/motor_status":
            data["motor_status"] = int(msg.data)
        elif topic == "/odom":
            data["velocity"] = dict(linear=msg.twist.twist.linear.x,
                                    angular=msg.twist.twist.angular.z)

    subscriptions = []
    for topic, kind, qos in (("/map", OccupancyGrid, latched),
                             ("/scan", LaserScan, sensor), ("/odom", Odometry, sensor),
                             ("/imu/data", Imu, sensor), ("/motor_fb", JointState, sensor),
                             ("/motor_status", UInt8, sensor)):
        subscriptions.append(node.create_subscription(
            kind, topic, lambda msg, topic=topic: receive(topic, msg), qos))
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    started = time.monotonic()
    next_output, next_graph = started, started
    navigation_available = False
    try:
        while stream or time.monotonic() - started < seconds:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if stream and select.select([sys.stdin], [], [], 0)[0]:
                if sys.stdin.readline().strip() in ("", "stop"):
                    break
            if stream and now < next_output:
                continue
            if not stream and now - started < seconds:
                continue
            next_output = now + 0.2
            data["topics"] = {topic: dict(messages=counts.get(topic, 0),
                                      age_s=round(now - last[topic], 2) if topic in last else None)
                          for topic in ("/map", "/scan", "/odom", "/imu/data",
                                        "/motor_fb", "/motor_status")}
            data["tf_map_to_base"] = bool(buffer.can_transform(
                "map", "base_footprint", rclpy.time.Time()))
            if now >= next_graph:
                next_graph = now + 1.0
                navigation_available = any(name == "/navigate_to_pose/_action/send_goal"
                                          for name, _ in node.get_service_names_and_types())
            data["navigation_available"] = navigation_available
            data["mapping_ready"] = ("map" in data and data["tf_map_to_base"] and
                                  all(topic in last and now - last[topic] < 2.0
                                      for topic in ("/scan", "/odom")))
            print(json.dumps(data, ensure_ascii=False), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("action", choices=("probe", "stream", "rviz", "navigation"))
    parser.add_argument("--domain", type=int, default=0)
    parser.add_argument("--profile")
    parser.add_argument("--seconds", type=float, default=6)
    args = parser.parse_args()
    if not 0 <= args.domain <= 232:
        parser.error("ROS domain must be between 0 and 232")
    environment(args.domain, args.profile)
    if args.action in ("probe", "stream"):
        probe(max(2, min(args.seconds, 30)), stream=args.action == "stream")
    else:
        import shutil
        import subprocess
        import fcntl
        config = Path(__file__).with_name("mapping.rviz")
        if args.action == "navigation":
            from ament_index_python.packages import get_package_share_directory
            get_package_share_directory("nav2_rviz_plugins")
            config = Path(__file__).with_name("navigation.rviz")
        rviz = shutil.which("rviz2")
        if not rviz:
            raise RuntimeError("Install ros-jazzy-rviz2 on the PC (in the WSL distro on Windows)")
        # Prevent duplicate viewers when the console is reopened.
        lock = open(f"/tmp/rpbot-rviz-{os.getuid()}-{args.domain}-{args.action}.lock", "w")
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError("This RPBot RViz view is already open")
        process = subprocess.Popen([rviz, "-d", str(config)])
        try:
            raise SystemExit(process.wait())
        except KeyboardInterrupt:
            process.terminate()
            process.wait(timeout=10)


if __name__ == "__main__":
    main()
