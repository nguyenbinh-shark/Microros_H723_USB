import json
from pathlib import Path
import subprocess
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import rpbot_console as console
import remote_ops

NS = {"dds": "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}
PROFILE = '<profiles xmlns="' + NS["dds"] + '"><participant profile_name="robot" is_default_profile="true"><rtps><port><portBase>7650</portBase></port></rtps></participant></profiles>'


class ConsoleTests(unittest.TestCase):
    def setUp(self):
        defaults = patch.dict(console.DEFAULTS, host='fixture.local', user='operator')
        defaults.start()
        self.addCleanup(defaults.stop)
        resolver = patch.object(console, 'resolve_robot_address', return_value='192.168.0.120')
        resolver.start()
        self.addCleanup(resolver.stop)

    def test_reset_requires_same_real_mapping_session(self):
        real = dict(pid=100, mode="mapping", simulated_time=False, domain=0)
        replay = dict(pid=200, mode="mapping", simulated_time=True, domain=73)
        with patch.object(remote_ops, "launches", return_value=[real, replay]), \
             patch.object(remote_ops, "run") as run:
            for domain, pid in ((0, 999), (73, 200), (1, 100)):
                with self.assertRaisesRegex(RuntimeError, "session changed"):
                    remote_ops.reset_mapping(domain, pid)
        run.assert_not_called()

    def test_reset_inspection_never_sends_reset_request(self):
        self.run_reset_program(inspect=True)

    def test_reset_resumes_scan_processing_and_checks_result(self):
        self.run_reset_program()

    def test_reset_timeout_is_reported_without_retry(self):
        self.run_reset_program(done=False, error="outcome unknown")

    def test_reset_rejected_result_is_not_success(self):
        self.run_reset_program(response_code=1, error="did not confirm")

    def run_reset_program(self, *, inspect=False, done=True, response_code=0, error=None):
        fake_ros = ModuleType("rclpy")
        fake_ros.init, fake_ros.shutdown = Mock(), Mock()
        client = Mock()
        client.wait_for_service.return_value = True
        client.call_async.return_value.done.return_value = done
        client.call_async.return_value.result.return_value = SimpleNamespace(result=response_code)
        node = Mock()
        node.create_client.return_value = client
        fake_ros.create_node = Mock(return_value=node)
        fake_ros.spin_until_future_complete = Mock()
        services = ModuleType("slam_toolbox.srv")
        services.Reset = SimpleNamespace(Request=SimpleNamespace, Response=SimpleNamespace(RESULT_SUCCESS=0))
        modules = {"rclpy": fake_ros, "slam_toolbox": ModuleType("slam_toolbox"), "slam_toolbox.srv": services}
        with patch.dict(sys.modules, modules), patch.object(sys, "argv", ["reset"] + (["--inspect"] if inspect else [])), \
             patch("builtins.print") as output:
            if error:
                with self.assertRaisesRegex(RuntimeError, error):
                    exec(remote_ops.RESET_PROGRAM, {})
                output.assert_not_called()
            else:
                exec(remote_ops.RESET_PROGRAM, {})
        if inspect:
            client.call_async.assert_not_called()
        else:
            client.call_async.assert_called_once()
            self.assertIs(client.call_async.call_args.args[0].pause_new_measurements, False)
        node.destroy_node.assert_called_once()
        fake_ros.shutdown.assert_called_once()

    def test_reset_inspect_uses_domain_and_does_not_restart_robot(self):
        real = dict(pid=100, mode="mapping", simulated_time=False, domain=0)
        with patch.object(remote_ops, "launches", return_value=[real]), \
             patch.object(remote_ops, "run", return_value=subprocess.CompletedProcess([], 0, '{"reset_available":true}', "")) as run:
            self.assertTrue(remote_ops.reset_mapping(0, 100, inspect=True)["reset_available"])
        command = run.call_args.args[0]
        self.assertEqual(command[-1], "--inspect")
        self.assertEqual(run.call_args.kwargs["env"]["ROS_DOMAIN_ID"], "0")
        self.assertNotIn("systemctl", command)

    def test_gui_cancel_does_not_send_reset(self):
        app = console.Console.__new__(console.Console)
        app.require_session = Mock(return_value=console.DEFAULTS)
        app.last_data = {"navigation_available": False}
        app.session = {"mode": "mapping"}
        app.task = Mock()
        with patch.object(console.messagebox, "askyesno", return_value=False):
            app.reset_map()
        app.task.assert_not_called()

    def test_custom_robot_ports_use_standard_local_ports(self):
        root = ET.fromstring(console.pc_profile(PROFILE, "192.168.0.120", 0))
        self.assertEqual(root.find(".//dds:port/dds:portBase", NS).text, "7400")
        peers = root.findall(".//dds:initialPeersList/dds:locator/dds:udpv4", NS)
        self.assertEqual(len(peers), 32)
        self.assertEqual(peers[0].find("dds:port", NS).text, "7660")
        self.assertEqual(peers[-1].find("dds:port", NS).text, "7722")
        self.assertTrue(all(peer.find("dds:address", NS).text == "192.168.0.120" for peer in peers))

    def test_default_profile_respects_nonzero_domain(self):
        root = ET.fromstring(console.pc_profile("", "127.0.0.1", 77))
        self.assertEqual(root.find(".//dds:initialPeersList//dds:port", NS).text, "26660")

    def test_replay_mapping_is_not_selected_as_real_robot(self):
        real = dict(mode="mapping", simulated_time=False, domain=0)
        replay = dict(mode="mapping", simulated_time=True, domain=73)
        robot = {"launches": [replay, real]}
        with patch.object(console, "remote", return_value=json.dumps(robot)), \
             patch.object(console, "session_args", return_value=["probe"]) as command, \
             patch.object(console, "capture", return_value='{"mapping_ready":true}'):
            report = console.check_connection(console.DEFAULTS)
        self.assertEqual(report["session"], real)
        command.assert_called_once_with(console.DEFAULTS, "probe", real, address='192.168.0.120')

    def test_ambiguous_live_sessions_never_guess_domain(self):
        real = dict(pid=41, mode="mapping", simulated_time=False, domain=0)
        with patch.object(console, "remote", return_value=json.dumps({"launches": [real, dict(real, domain=1)]})), \
             patch.object(console, "capture") as capture:
            report = console.check_connection(console.DEFAULTS)
        self.assertIsNone(report["session"])
        capture.assert_not_called()

    def test_navigation_is_selected_while_replay_mapping_is_ignored(self):
        nav = dict(pid=90, mode="navigation", simulated_time=False, domain=0)
        replay = dict(pid=91, mode="mapping", simulated_time=True, domain=62)
        with patch.object(console, "remote", return_value=json.dumps({"launches": [nav, replay]})):
            report = console.inspect_robot(console.DEFAULTS)
        self.assertEqual(report["session"], nav)

    def test_start_in_navigation_reports_mode_without_starting_mapping(self):
        nav = dict(pid=90, mode="navigation", simulated_time=False, domain=0)
        with patch.object(sys, "argv", ["remote_ops", "start"]), \
             patch.object(remote_ops, "launches", return_value=[nav]), \
             patch.object(remote_ops, "run") as run, patch("builtins.print") as output:
            remote_ops.main()
            report = json.loads(output.call_args.args[0])
        self.assertEqual(report["status"], "different_mode")
        self.assertEqual(report["mode"], "navigation")
        run.assert_not_called()

    def test_reset_in_navigation_is_blocked_before_confirmation(self):
        app = console.Console.__new__(console.Console)
        app.require_session = Mock(return_value=console.DEFAULTS)
        app.session = {"mode": "navigation"}
        app.last_data = {"navigation_available": False}
        app.task = Mock()
        with patch.object(console.messagebox, "showinfo"), patch.object(console.messagebox, "askyesno") as confirm:
            app.reset_map()
        confirm.assert_not_called()
        app.task.assert_not_called()

    def test_gui_navigation_start_connects_without_claiming_mapping_started(self):
        app = console.Console.__new__(console.Console)
        app.config = Mock(return_value=console.DEFAULTS)
        app.status = {key: Mock() for key in ("mapping", "sensors", "motion", "navigation")}
        app.task, app.append, app.root, app.check = Mock(), Mock(), Mock(), Mock()
        app.service("start")
        app.task.call_args.args[2]('{"status":"different_mode","mode":"navigation"}')
        self.assertIn("Robot đang Navigation", app.append.call_args.args[0])
        self.assertIn("chưa chuyển sang Mapping", app.append.call_args.args[0])
        app.root.after.assert_called_once_with(0, app.check)

    def test_remote_ssh_uses_stdin_and_quotes_arguments(self):
        with patch.object(console, 'resolve_robot_address', return_value='192.168.0.120'), \
             patch.object(console, "capture", return_value="ok") as capture:
            console.remote(console.DEFAULTS, "save", "--name", "name with spaces; touch nope")
        command = capture.call_args.args[0]
        self.assertIn("BatchMode=yes", command)
        self.assertEqual(command[-1], "python3 - save --name 'name with spaces; touch nope'")
        self.assertIn("def main()", capture.call_args.kwargs["input"])

    def test_settings_reject_option_or_shell_injection(self):
        for host in ("-oProxyCommand=bad", "robot; command", "user@robot"):
            with self.assertRaises(ValueError):
                console.validate_settings(dict(console.DEFAULTS, host=host))

    def test_start_reuses_real_mapping_without_restarting_it(self):
        real = dict(pid=123, mode="mapping", simulated_time=False, domain=0)
        replay = dict(pid=456, mode="mapping", simulated_time=True, domain=73)
        with patch.object(sys, "argv", ["remote_ops", "start"]), \
             patch.object(remote_ops, "launches", return_value=[real, replay]), \
             patch.object(remote_ops, "run") as run, patch("builtins.print") as output:
            for _ in range(2):
                remote_ops.main()
                report = json.loads(output.call_args.args[0])
                self.assertEqual(report["status"], "already_running")
                self.assertEqual(report["pid"], 123)
        run.assert_not_called()

    def test_start_does_not_reuse_demo_bench_or_ambiguous_mapping(self):
        real = dict(pid=123, mode="mapping", simulated_time=False, domain=0)
        for sessions in ([dict(real, mode="bench")], [dict(real, simulated_time=True)], [real, dict(real, pid=124)]):
            with patch.object(sys, "argv", ["remote_ops", "start"]), \
                 patch.object(remote_ops, "launches", return_value=sessions), \
                 patch.object(remote_ops, "run") as run:
                with self.assertRaises(RuntimeError):
                    remote_ops.main()
            run.assert_not_called()

    def test_gui_existing_start_explains_and_rechecks_data(self):
        app = console.Console.__new__(console.Console)
        app.config = Mock(return_value=console.DEFAULTS)
        app.status = {key: Mock() for key in ("mapping", "sensors", "motion", "navigation")}
        app.task, app.append, app.root, app.check = Mock(), Mock(), Mock(), Mock()
        app.service("start")
        done = app.task.call_args.args[2]
        done('{"status":"already_running"}')
        self.assertIn("Mapping đang chạy", app.append.call_args.args[0])
        app.root.after.assert_called_once_with(0, app.check)

    def test_stop_does_not_kill_unmanaged_session(self):
        with patch.object(sys, "argv", ["remote_ops", "stop"]), \
             patch.object(remote_ops, "launches", return_value=[{"pid": 123}]), \
             patch.object(remote_ops, "run", return_value=subprocess.CompletedProcess([], 3, "", "")) as run:
            with self.assertRaisesRegex(RuntimeError, "left unchanged"):
                remote_ops.main()
        self.assertEqual(run.call_count, 1)
        self.assertIn("is-active", run.call_args.args[0])

    def test_map_name_cannot_escape_maps_directory(self):
        with patch.object(sys, "argv", ["remote_ops", "save", "--name", "../../overwrite"]), \
             patch.object(remote_ops, "run") as run:
            with self.assertRaises(ValueError):
                remote_ops.main()
        run.assert_not_called()

    def test_autostart_only_enables_without_restarting_live_robot(self):
        with tempfile.TemporaryDirectory() as folder:
            home = Path(folder)
            project = home / "pi5-stm32-slam"
            (project / "install").mkdir(parents=True)
            (project / "install/setup.bash").touch()
            def result(command):
                if "is-enabled" in command:
                    return subprocess.CompletedProcess(command, 1, "disabled", "")
                return subprocess.CompletedProcess(command, 0, "yes\n", "")
            with patch.object(remote_ops.Path, "home", return_value=home), \
                 patch.object(remote_ops.os, "getuid", return_value=1000, create=True), \
                 patch.object(remote_ops, "ROOT", project), patch.object(remote_ops, "run", side_effect=result) as run:
                self.assertEqual(remote_ops.install_autostart()["current_session"], "unchanged")
            commands = [call.args[0] for call in run.call_args_list]
            self.assertTrue(any("enable" in command for command in commands))
            self.assertFalse(any("start" in command or "restart" in command for command in commands))
            wrapper = (home / ".local/share/rpbot-console/mapping.sh").read_text()
            self.assertIn("allow_motor_enable:=false", wrapper)
            self.assertIn("exit 75", wrapper)
            self.assertIn("RestartPreventExitStatus=75", (home / ".config/systemd/user/rpbot-mapping.service").read_text())


if __name__ == "__main__":
    unittest.main()
